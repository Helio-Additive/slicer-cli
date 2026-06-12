//! Faithful 1:1 port of BambuStudio `src/libslic3r/Fill/Fill.cpp`.
//!
//! This file mirrors the C++ source line-by-line (snake_case). With the
//! config hierarchy wired (`layer.object().print().config()`,
//! `layer_region.region().config()`, `LayerRegion::flow/bridging_flow`),
//! `group_fills` (Fill.cpp:166-549) and `Layer::set_outlook_range`
//! (Fill.cpp:571-586) are ported here. The remaining `Layer::*` methods
//! (`make_fills`, `make_ironing`,
//! `generate_sparse_infill_polylines_for_anchoring`) and the debug
//! `export_group_fills_to_svg` stay BLOCKED on the virtual `Fill` dispatch
//! infrastructure — see the notes at the bottom of this file.
//!
//! C++ source: src/libslic3r/Fill/Fill.cpp
//! C++ header: src/libslic3r/Fill/Fill.hpp

use crate::clipper_utils::{
    clip_clipper_polygons_with_subject_bbox_expolygons, diff_ex_polygons_polygons, diff_polygons,
    expand_polygons, intersection_ex_expolygons_polygons, intersection_ex_polygons_polygons,
    offset_expolygon, opening_polygons_2, union_ex, union_safety_offset_ex,
    union_safety_offset_ex_expolygons, ApplySafetyOffset, OffsetJoinType,
};
use crate::extrusion_entity::ExtrusionRole;
use crate::fill::LockRegionParam;
use crate::flow::{Flow, FlowRole};
use crate::geometry::{
    deg2rad, get_extents, get_extents_expoly, get_extents_polygons, to_polygons, ExPolygon,
    Polygon,
};
use crate::layer::{Layer, LayerRegion};
use crate::libslic3r::{scale, EPSILON, SCALED_EPSILON};
use crate::print_config::InfillPattern;
use crate::surface::SurfaceType;
use crate::Coord;

// Fill.cpp:19
// #define NARROW_INFILL_AREA_THRESHOLD 3
pub const NARROW_INFILL_AREA_THRESHOLD: f64 = 3.0;

// Fill.cpp:23
// struct SurfaceFillParams
#[derive(Debug, Clone)]
pub struct SurfaceFillParams {
    // Fill.cpp:25-26
    // Zero based extruder ID.
    pub extruder: u32,
    // Fill.cpp:27-28
    // Infill pattern, adjusted for the density etc.
    pub pattern: InfillPattern,
    // Fill.cpp:29-30
    // for locked zag
    pub skin_pattern: InfillPattern,
    // Fill.cpp:31
    pub skeleton_pattern: InfillPattern,

    // Fill.cpp:33-35
    // FillBase
    // in unscaled coordinates
    pub spacing: f64,
    // Fill.cpp:36-37
    // infill / perimeter overlap, in unscaled coordinates
    pub overlap: f64,
    // Fill.cpp:38-39
    // Angle as provided by the region config, in radians.
    pub angle: f32,
    // Fill.cpp:40-41
    // Is bridging used for this fill? Bridging parameters may be used even if this->flow.bridge() is not set.
    pub bridge: bool,
    // Fill.cpp:42-43
    // Non-negative for a bridge.
    pub bridge_angle: f32,

    // Fill.cpp:45-46
    // FillParams
    pub density: f32,
    // Fill.cpp:47
    pub multiline: i32,
    // Fill.cpp:48-49
    // travel into wall length, ratio to line width
    pub monotonic_travel_into_wall: f32,
    // Fill.cpp:50-51
    // Don't adjust spacing to fill the space evenly.
    //    bool        	dont_adjust = false;
    // Fill.cpp:52-55
    // Length of the infill anchor along the perimeter line.
    // 1000mm is roughly the maximum length line that fits into a 32bit coord_t.
    pub anchor_length: f32,
    pub anchor_length_max: f32,
    // Fill.cpp:56-59
    // BBS
    // width, height of extrusion, nozzle diameter, is bridge
    // For the output, for fill generator.
    pub flow: Flow,

    // Fill.cpp:61-62
    // For the output
    pub extrusion_role: ExtrusionRole,

    // Fill.cpp:64
    // Various print settings?

    // Fill.cpp:66-67
    // Index of this entry in a linear vector.
    pub idx: usize,
    // Fill.cpp:68-71
    // infill speed settings
    pub sparse_infill_speed: f32,
    pub top_surface_speed: f32,
    pub solid_infill_speed: f32,
    // Fill.cpp:72
    pub infill_shift_step: f32, // param for cross zag
    // Fill.cpp:73
    pub infill_rotate_step: f32, // param for zig zag to get cross texture
    // Fill.cpp:74
    pub symmetric_infill_y_axis: bool,

    // Fill.cpp:76-78
    // Params for 2Dlattice infill angles
    pub lattice_angle_1: f32,
    pub lattice_angle_2: f32,
}

impl Default for SurfaceFillParams {
    // Mirrors the C++ in-class member initializers (Fill.cpp:25-78).
    fn default() -> Self {
        Self {
            extruder: 0,
            // InfillPattern(0) == ipConcentric (PrintConfig.hpp:77). Spelled
            // explicitly: the Rust enum's #[default] is Grid (struct-default
            // convenience), not the C++ zero value.
            pattern: InfillPattern::Concentric,
            skin_pattern: InfillPattern::Concentric,
            skeleton_pattern: InfillPattern::Concentric,
            spacing: 0.,
            overlap: 0.,
            angle: 0.,
            // C++: bool bridge; (uninitialized, but only set before use in group_fills)
            bridge: false,
            bridge_angle: 0.,
            density: 0.,
            multiline: 1,
            monotonic_travel_into_wall: 0.,
            anchor_length: 1000.,
            anchor_length_max: 1000.,
            flow: Flow::zero(),
            extrusion_role: ExtrusionRole::None,
            idx: 0,
            sparse_infill_speed: 0.,
            top_surface_speed: 0.,
            solid_infill_speed: 0.,
            infill_shift_step: 0.,
            infill_rotate_step: 0.,
            symmetric_infill_y_axis: false,
            lattice_angle_1: -45.0,
            lattice_angle_2: 45.0,
        }
    }
}

impl SurfaceFillParams {
    // Fill.cpp:80-114
    // bool operator<(const SurfaceFillParams &rhs) const
    //
    // The C++ macros expand to: for each KEY, if this->KEY < rhs.KEY return
    // true; if this->KEY > rhs.KEY return false. We translate the strict
    // weak ordering directly into an Ordering chain.
    pub fn lt(&self, rhs: &SurfaceFillParams) -> bool {
        use std::cmp::Ordering;

        // Fill.cpp:84-86
        // Sort first by decreasing bridging angle, so that the bridges are processed with priority when trimming one layer by the other.
        if self.bridge_angle > rhs.bridge_angle {
            return true;
        }
        if self.bridge_angle < rhs.bridge_angle {
            return false;
        }

        // Each comparator step: Less => true, Greater => false, Equal => fall through.
        macro_rules! return_compare_non_equal {
            ($a:expr, $b:expr) => {{
                let a = $a;
                let b = $b;
                if a < b {
                    return true;
                }
                if a > b {
                    return false;
                }
            }};
        }
        macro_rules! return_compare_non_equal_ord {
            ($a:expr, $b:expr) => {{
                match $a.cmp(&$b) {
                    Ordering::Less => return true,
                    Ordering::Greater => return false,
                    Ordering::Equal => {}
                }
            }};
        }

        // Fill.cpp:88
        return_compare_non_equal!(self.extruder, rhs.extruder);
        // Fill.cpp:89  RETURN_COMPARE_NON_EQUAL_TYPED(unsigned, pattern);
        return_compare_non_equal!(self.pattern as u32, rhs.pattern as u32);
        // Fill.cpp:90
        return_compare_non_equal!(self.spacing, rhs.spacing);
        // Fill.cpp:91
        return_compare_non_equal!(self.overlap, rhs.overlap);
        // Fill.cpp:92
        return_compare_non_equal!(self.angle, rhs.angle);
        // Fill.cpp:93
        return_compare_non_equal!(self.density, rhs.density);
        // Fill.cpp:94
        return_compare_non_equal_ord!(self.multiline, rhs.multiline);
        // Fill.cpp:95  RETURN_COMPARE_NON_EQUAL_TYPED(unsigned, dont_adjust); (commented out)
        // Fill.cpp:96
        return_compare_non_equal!(self.anchor_length, rhs.anchor_length);
        // Fill.cpp:97
        return_compare_non_equal!(self.anchor_length_max, rhs.anchor_length_max);
        // Fill.cpp:98
        return_compare_non_equal!(self.flow.width(), rhs.flow.width());
        // Fill.cpp:99
        return_compare_non_equal!(self.flow.height(), rhs.flow.height());
        // Fill.cpp:100
        return_compare_non_equal!(self.flow.nozzle_diameter(), rhs.flow.nozzle_diameter());
        // Fill.cpp:101  RETURN_COMPARE_NON_EQUAL_TYPED(unsigned, bridge);
        return_compare_non_equal!(self.bridge as u32, rhs.bridge as u32);
        // Fill.cpp:102  RETURN_COMPARE_NON_EQUAL_TYPED(unsigned, extrusion_role);
        return_compare_non_equal!(self.extrusion_role as u32, rhs.extrusion_role as u32);
        // Fill.cpp:103
        return_compare_non_equal!(self.sparse_infill_speed, rhs.sparse_infill_speed);
        // Fill.cpp:104
        return_compare_non_equal!(self.top_surface_speed, rhs.top_surface_speed);
        // Fill.cpp:105
        return_compare_non_equal!(self.solid_infill_speed, rhs.solid_infill_speed);
        // Fill.cpp:106
        return_compare_non_equal!(self.infill_shift_step, rhs.infill_shift_step);
        // Fill.cpp:107
        return_compare_non_equal!(self.infill_rotate_step, rhs.infill_rotate_step);
        // Fill.cpp:108
        return_compare_non_equal!(
            self.symmetric_infill_y_axis as u32,
            rhs.symmetric_infill_y_axis as u32
        );
        // Fill.cpp:109
        return_compare_non_equal!(self.lattice_angle_1, rhs.lattice_angle_1);
        // Fill.cpp:110
        return_compare_non_equal!(self.lattice_angle_2, rhs.lattice_angle_2);
        // Fill.cpp:111  RETURN_COMPARE_NON_EQUAL_TYPED(unsigned, skin_pattern);
        return_compare_non_equal!(self.skin_pattern as u32, rhs.skin_pattern as u32);
        // Fill.cpp:112  RETURN_COMPARE_NON_EQUAL_TYPED(unsigned, skeleton_pattern);
        return_compare_non_equal!(self.skeleton_pattern as u32, rhs.skeleton_pattern as u32);
        // Fill.cpp:113
        false
    }
}

// Fill.cpp:116-141
// bool operator==(const SurfaceFillParams &rhs) const
impl PartialEq for SurfaceFillParams {
    fn eq(&self, rhs: &Self) -> bool {
        // Fill.cpp:117-140
        self.extruder == rhs.extruder
            && self.pattern == rhs.pattern
            && self.spacing == rhs.spacing
            && self.overlap == rhs.overlap
            && self.angle == rhs.angle
            && self.bridge == rhs.bridge
            // this->bridge_angle == rhs.bridge_angle (commented out in C++)
            && self.density == rhs.density
            && self.multiline == rhs.multiline
            // this->dont_adjust == rhs.dont_adjust (commented out in C++)
            && self.anchor_length == rhs.anchor_length
            && self.anchor_length_max == rhs.anchor_length_max
            && self.flow == rhs.flow
            && self.extrusion_role == rhs.extrusion_role
            && self.sparse_infill_speed == rhs.sparse_infill_speed
            && self.top_surface_speed == rhs.top_surface_speed
            && self.solid_infill_speed == rhs.solid_infill_speed
            && self.infill_shift_step == rhs.infill_shift_step
            && self.infill_rotate_step == rhs.infill_rotate_step
            && self.symmetric_infill_y_axis == rhs.symmetric_infill_y_axis
            && self.lattice_angle_1 == rhs.lattice_angle_1
            && self.lattice_angle_2 == rhs.lattice_angle_2
            && self.skin_pattern == rhs.skin_pattern
            && self.skeleton_pattern == rhs.skeleton_pattern
    }
}

// Fill.cpp:144-154
// struct SurfaceFill
#[derive(Debug, Clone)]
pub struct SurfaceFill {
    // Fill.cpp:147
    pub region_id: usize,
    // Fill.cpp:148
    pub surface: crate::surface::Surface,
    // Fill.cpp:149
    pub expolygons: Vec<ExPolygon>,
    // Fill.cpp:150
    pub params: SurfaceFillParams,
    // Fill.cpp:151-152
    // BBS
    pub region_id_group: Vec<usize>,
    // Fill.cpp:153
    pub no_overlap_expolygons: Vec<ExPolygon>,
}

impl SurfaceFill {
    // Fill.cpp:145
    // SurfaceFill(const SurfaceFillParams& params) : region_id(size_t(-1)), surface(stCount, ExPolygon()), params(params) {}
    pub fn new(params: SurfaceFillParams) -> Self {
        Self {
            region_id: usize::MAX, // size_t(-1)
            // surface(stCount, ExPolygon())
            // DIVERGENCE: the crate SurfaceType enum has no `stCount` sentinel
            // (only SurfaceType::COUNT as a usize constant). The surface is a
            // placeholder always overwritten in group_fills (Fill.cpp:343)
            // before use, so the default-constructed Surface is equivalent.
            surface: crate::surface::Surface::default(),
            expolygons: Vec::new(),
            params,
            region_id_group: Vec::new(),
            no_overlap_expolygons: Vec::new(),
        }
    }
}

// Fill.cpp:156-164
// BBS: used to judge whether the internal solid infill area is narrow
// static bool is_narrow_infill_area(const ExPolygon& expolygon)
pub fn is_narrow_infill_area(expolygon: &ExPolygon) -> bool {
    // Fill.cpp:159
    // ExPolygons offsets = offset_ex(expolygon, -scale_(NARROW_INFILL_AREA_THRESHOLD));
    // C++ passes the delta in scaled units; this crate's offset helpers take
    // millimeters (the geo backend unscales the coordinates), so the
    // -scale_() wrapper drops out.
    let offsets = offset_expolygon(
        expolygon,
        -NARROW_INFILL_AREA_THRESHOLD,
        OffsetJoinType::Miter,
    );
    // Fill.cpp:160-161
    if offsets.is_empty() {
        return true;
    }

    // Fill.cpp:163
    false
}

// FillBase.cpp:80-92
// bool Fill::use_bridge_flow(const InfillPattern type)
//
// The C++ builds a lazily-initialized table by instantiating every filler
// through Fill::new_from_type (FillBase.cpp:33-67) and querying the virtual
// Fill::use_bridge_flow() (default false, FillBase.hpp:146). The only
// override returning true is Fill3DHoneycomb (Fill3DHoneycomb.hpp:19);
// FillGyroid restates the default false (FillGyroid.hpp:17). The table
// therefore collapses to this comparison. Housed here (its only caller is
// group_fills) until a Fill factory exists in the crate.
pub fn use_bridge_flow(pattern: InfillPattern) -> bool {
    pattern == InfillPattern::Honeycomb3D
}

// Fill.cpp:175-181
// auto append_flow_param = [](std::map<Flow, ExPolygons> &flow_params, Flow flow, const ExPolygon &exp)
//
// The C++ std::map<Flow, ExPolygons> is modeled as a Vec kept sorted
// ascending by key (the std::map iteration order). `find` uses the map's
// comparator equivalence `!(a<b) && !(b<a)` over Flow::operator<, which
// compares mm3_per_mm() (Flow.hpp:88-90) — mirrored by the crate's
// `Flow::partial_cmp`.
fn append_flow_param(flow_params: &mut Vec<(Flow, Vec<ExPolygon>)>, flow: Flow, exp: &ExPolygon) {
    use std::cmp::Ordering;
    let lt = |a: &Flow, b: &Flow| a.partial_cmp(b) == Some(Ordering::Less);
    // Fill.cpp:176-180
    match flow_params
        .iter_mut()
        .find(|(k, _)| !lt(k, &flow) && !lt(&flow, k))
    {
        Some((_, exps)) => exps.push(exp.clone()),
        None => {
            let pos = flow_params
                .iter()
                .position(|(k, _)| lt(&flow, k))
                .unwrap_or(flow_params.len());
            flow_params.insert(pos, (flow, vec![exp.clone()]));
        }
    }
}

// Fill.cpp:183-189
// auto append_density_param = [](std::map<float, ExPolygons> &density_params, float density, const ExPolygon &exp)
//
// Same Vec-as-std::map modeling as `append_flow_param`, keyed by f32 with
// the float's own `<` (exact comparison, as in C++).
fn append_density_param(
    density_params: &mut Vec<(f32, Vec<ExPolygon>)>,
    density: f32,
    exp: &ExPolygon,
) {
    // Fill.cpp:184-188
    match density_params.iter_mut().find(|(k, _)| *k == density) {
        Some((_, exps)) => exps.push(exp.clone()),
        None => {
            let pos = density_params
                .iter()
                .position(|(k, _)| density < *k)
                .unwrap_or(density_params.len());
            density_params.insert(pos, (density, vec![exp.clone()]));
        }
    }
}

// Fill.cpp:166-549
// std::vector<SurfaceFill> group_fills(const Layer &layer, LockRegionParam &lock_param)
//
// Porting notes (divergences forced by the crate's structure, all
// behavior-preserving):
//  * C++ reads `layer.lower_layer` (Fill.cpp:457) through the Layer's
//    sibling pointer; the Rust Layer stores only `lower_layer_id`, so the
//    caller passes the lower layer explicitly.
//  * The C++ `std::set<SurfaceFillParams>` is modeled as an insertion-order
//    Vec with comparator-equivalence find (`!(a<b) && !(b<a)` over
//    `SurfaceFillParams::lt`); the set's sorted iteration order at
//    Fill.cpp:329 is reproduced by sorting an index permutation with the
//    same comparator (set elements are unique under it, so no ties exist).
//  * Per-extruder vector options (`sparse_infill_speed.get_at(
//    layer.get_process_config_idx(...))`, `nozzle_diameter.get_at(...)`,
//    `filament_enable_overhang_speed.get_at(layer.get_filament_config_idx(
//    ...))`) collapse onto this crate's scalar config fields — same
//    convention as LayerRegion::bridging_flow / PrintRegion::flow.
//  * C++ Surface default-initializes `thickness = -1` ("unset"); this
//    crate's Surface defaults to 0.0 and the pipeline never assigns it, so
//    the `(surface.thickness == -1) ? layer.height : surface.thickness`
//    ternary maps to a `> 0.0` test.
//  * Flow construction / extruder lookup return Result in Rust (C++
//    asserts); errors propagate via crate::Result.
//
// Rust PrintRegionConfig field-name mapping (C++ name -> Rust name):
//   sparse_infill_pattern        -> fill_pattern
//   sparse_infill_density (pct)  -> fill_density (0..1 ratio; *100 below)
//   internal_solid_infill_pattern-> solid_fill_pattern
//   top_surface_pattern          -> top_fill_pattern
//   bottom_surface_pattern       -> bottom_fill_pattern
//   infill_direction             -> fill_angle
//   sparse_infill_speed          -> infill_speed
//   internal_solid_infill_speed  -> solid_infill_speed
//   top_surface_speed            -> top_solid_infill_speed
//   sparse_infill_anchor(_max)   -> infill_anchor(_max)
pub fn group_fills(
    layer: &Layer,
    lower_layer: Option<&Layer>,
    lock_param: &mut LockRegionParam,
) -> crate::Result<Vec<SurfaceFill>> {
    // Fill.cpp:168
    let mut surface_fills: Vec<SurfaceFill> = Vec::new();
    // Fill.cpp:169-171
    // Fill in a map of a region & surface to SurfaceFillParams.
    let mut set_surface_params: Vec<SurfaceFillParams> = Vec::new();
    let mut region_to_surface_params: Vec<Vec<Option<usize>>> =
        vec![Vec::new(); layer.regions().len()];
    // Fill.cpp:172
    let mut has_internal_voids = false;
    // Fill.cpp:173
    // const PrintObjectConfig &object_config = layer.object()->config();
    let object = layer.object();
    let object_config = object.config();

    // Fill.cpp:191
    for region_id in 0..layer.regions().len() {
        // Fill.cpp:192
        let layerm: &LayerRegion = &layer.regions()[region_id];
        // Fill.cpp:193
        region_to_surface_params[region_id] = vec![None; layerm.fill_surfaces.surfaces.len()];
        // Fill.cpp:194
        for (surface_index, surface) in layerm.fill_surfaces.surfaces.iter().enumerate() {
            // Fill.cpp:195-196
            if surface.surface_type == SurfaceType::InternalVoid {
                has_internal_voids = true;
            } else {
                // Fill.cpp:198-199
                let mut params = SurfaceFillParams::default();
                let region_config = layerm.region().config();
                // Fill.cpp:200
                let extrusion_role: FlowRole = if surface.is_top() {
                    FlowRole::TopSolidInfill
                } else if surface.is_solid() {
                    FlowRole::SolidInfill
                } else {
                    FlowRole::Infill
                };
                // Fill.cpp:201
                let is_bridge = layer.id() > 0 && surface.is_bridge();
                // Fill.cpp:202
                params.extruder = layerm
                    .region()
                    .extruder(extrusion_role)
                    .map_err(crate::Error::Config)?;
                // Fill.cpp:203
                params.pattern = region_config.fill_pattern;
                // Fill.cpp:204 — C++ Percent .value is the raw percent number;
                // the Rust field stores a 0..1 ratio.
                params.density = (region_config.fill_density * 100.0) as f32;
                // Fill.cpp:205
                params.multiline = region_config.fill_multiline;
                // Fill.cpp:206-209
                if params.pattern == InfillPattern::LockedZag {
                    params.skin_pattern = region_config.locked_skin_infill_pattern;
                    params.skeleton_pattern = region_config.locked_skeleton_infill_pattern;
                }
                // Fill.cpp:210-219
                if params.pattern == InfillPattern::CrossZag
                    || params.pattern == InfillPattern::LockedZag
                {
                    // Fill.cpp:211 — scale_(infill_shift_step)
                    params.infill_shift_step = scale(region_config.infill_shift_step) as f32;
                    params.symmetric_infill_y_axis = region_config.symmetric_infill_y_axis;
                } else if params.pattern == InfillPattern::ZigZag {
                    // Fill.cpp:214 — infill_rotate_step * M_PI / 360
                    params.infill_rotate_step =
                        (region_config.infill_rotate_step * std::f64::consts::PI / 360.0) as f32;
                    params.symmetric_infill_y_axis = region_config.symmetric_infill_y_axis;
                } else if params.pattern == InfillPattern::Lattice2D {
                    // Fill.cpp:217-218
                    params.lattice_angle_1 = region_config.sparse_infill_lattice_angle_1 as f32;
                    params.lattice_angle_2 = region_config.sparse_infill_lattice_angle_2 as f32;
                }

                // Fill.cpp:221-235
                if surface.is_solid() {
                    // Fill.cpp:222
                    params.density = 100.0;
                    // FIXME for non-thick bridges, shall we allow a bottom surface pattern?
                    // Fill.cpp:224-225
                    if surface.is_floating_vertical_shell() {
                        params.pattern = InfillPattern::FloatingConcentric;
                    // Fill.cpp:226-227
                    } else if surface.is_solid_infill() {
                        params.pattern = region_config.solid_fill_pattern;
                    // Fill.cpp:228-230
                    } else if surface.is_external() && !is_bridge {
                        params.pattern = if surface.is_top() {
                            region_config.top_fill_pattern
                        } else {
                            region_config.bottom_fill_pattern
                        };
                        params.density = if surface.is_top() {
                            region_config.top_surface_density as f32
                        } else {
                            region_config.bottom_surface_density as f32
                        };
                    // Fill.cpp:231-232
                    } else {
                        params.pattern =
                            if region_config.top_fill_pattern == InfillPattern::Monotonic {
                                InfillPattern::Monotonic
                            } else {
                                InfillPattern::Rectilinear
                            };
                    }
                    // Fill.cpp:233
                    if params.pattern == InfillPattern::MonotonicLine {
                        params.monotonic_travel_into_wall =
                            region_config.monotonic_travel_into_wall as f32;
                    }
                // Fill.cpp:234-235
                } else if params.density <= 0.0 {
                    continue;
                }

                // Fill.cpp:237-242
                params.extrusion_role = if is_bridge {
                    ExtrusionRole::BridgeInfill
                } else if surface.is_solid() {
                    if surface.is_top() {
                        ExtrusionRole::TopSolidInfill
                    } else if surface.is_bottom() {
                        ExtrusionRole::BottomSurface
                    } else if surface.is_floating_vertical_shell() {
                        ExtrusionRole::FloatingVerticalShell
                    } else {
                        ExtrusionRole::SolidInfill
                    }
                } else {
                    ExtrusionRole::InternalInfill
                };
                // Fill.cpp:243 — C++ Surface.bridge_angle defaults to -1
                // ("not a bridge"); the Rust Option maps None onto it.
                params.bridge_angle = surface.bridge_angle.unwrap_or(-1.0) as f32;
                // Fill.cpp:244
                params.angle = deg2rad(region_config.fill_angle) as f32;
                // Fill.cpp:245-248
                let support_multiline_infill = params.pattern == InfillPattern::Cubic
                    || params.pattern == InfillPattern::Grid
                    || params.pattern == InfillPattern::Rectilinear
                    || params.pattern == InfillPattern::Stars
                    || params.pattern == InfillPattern::AlignedRectilinear
                    || params.pattern == InfillPattern::Gyroid
                    || params.pattern == InfillPattern::Honeycomb
                    || params.pattern == InfillPattern::Lightning
                    || params.pattern == InfillPattern::Honeycomb3D
                    || params.pattern == InfillPattern::AdaptiveCubic
                    || params.pattern == InfillPattern::SupportCubic;
                // Fill.cpp:249
                params.multiline = if params.extrusion_role == ExtrusionRole::InternalInfill
                    && support_multiline_infill
                {
                    region_config.fill_multiline
                } else {
                    1
                };

                // Calculate the actual flow we'll be using for this infill.
                // Fill.cpp:252
                params.bridge = is_bridge || use_bridge_flow(params.pattern);
                // C++ `(surface.thickness == -1) ? layer.height : surface.thickness`
                // (Fill.cpp:256/300/304); see the sentinel porting note above.
                let surface_thickness = if surface.thickness > 0.0 {
                    surface.thickness
                } else {
                    layer.height
                };
                // Fill.cpp:253-256
                params.flow = if params.bridge {
                    // BBS: always enable thick bridge for internal bridge
                    layerm.bridging_flow(
                        extrusion_role,
                        (surface.is_bridge() && !surface.is_external())
                            || object_config.thick_bridges,
                        layer.height,
                    )?
                } else {
                    layerm.flow(extrusion_role, surface_thickness)?
                };
                // BBS: record speed params
                // Fill.cpp:258-275
                if !params.bridge {
                    // Fill.cpp:259-260
                    if params.extrusion_role == ExtrusionRole::InternalInfill {
                        params.sparse_infill_speed = region_config.infill_speed as f32;
                    // Fill.cpp:261-262
                    } else if params.extrusion_role == ExtrusionRole::TopSolidInfill {
                        params.top_surface_speed = region_config.top_solid_infill_speed as f32;
                    // Fill.cpp:263-264
                    } else if params.extrusion_role == ExtrusionRole::SolidInfill {
                        params.solid_infill_speed = region_config.solid_infill_speed as f32;
                    // Fill.cpp:265-274
                    } else if params.extrusion_role == ExtrusionRole::FloatingVerticalShell {
                        // Fill.cpp:266 — int filament_id = sparse_infill_filament - 1;
                        // (only feeds the get_at index, which collapses here)
                        let _filament_id = region_config.sparse_infill_filament as i32 - 1;
                        // Fill.cpp:267-268 — layerm.layer()->object()->print()->config()
                        // (layerm.layer() == layer)
                        let print_config = object.print().config();
                        let use_filament_bridge_speed =
                            print_config.filament_enable_overhang_speed;
                        // Fill.cpp:270-273
                        if use_filament_bridge_speed {
                            params.solid_infill_speed = print_config.filament_bridge_speed as f32;
                        } else {
                            params.solid_infill_speed = region_config.bridge_speed as f32;
                        }
                    }
                }
                // Calculate flow spacing for infill pattern generation.
                // Fill.cpp:277-294
                if surface.is_solid() || is_bridge {
                    // Fill.cpp:278
                    params.spacing = params.flow.spacing();
                    // Don't limit anchor length for solid or bridging infill.
                    // Fill.cpp:280-281
                    params.anchor_length = 1000.0;
                    params.anchor_length_max = 1000.0;
                } else {
                    // Internal infill. Calculating infill line spacing independent of the current layer height and 1st layer status,
                    // so that internall infill will be aligned over all layers of the current region.
                    // Fill.cpp:285
                    // C++: layerm.region().flow(*layer.object(), frInfill, layer.object()->config().layer_height, false).spacing()
                    // (the shared config-level core of PrintRegion::flow, over
                    // the wired Arc hierarchy)
                    params.spacing = crate::print_region::flow_from_configs(
                        FlowRole::Infill,
                        object_config.layer_height,
                        false,
                        object.print().config().initial_layer_line_width,
                        object_config.line_width,
                        object.print().config().nozzle_diameter,
                        region_config,
                    )
                    .map_err(crate::Error::Config)?
                    .spacing();
                    // Anchor a sparse infill to inner perimeters with the following anchor length:
                    // Fill.cpp:287-289
                    params.anchor_length = region_config.infill_anchor.value as f32;
                    if region_config.infill_anchor.percent {
                        params.anchor_length =
                            (params.anchor_length as f64 * 0.01 * params.spacing) as f32;
                    }
                    // Fill.cpp:290-292
                    params.anchor_length_max = region_config.infill_anchor_max.value as f32;
                    if region_config.infill_anchor_max.percent {
                        params.anchor_length_max =
                            (params.anchor_length_max as f64 * 0.01 * params.spacing) as f32;
                    }
                    // Fill.cpp:293
                    params.anchor_length = params.anchor_length.min(params.anchor_length_max);
                }

                // get locked region param
                // Fill.cpp:297-319
                if params.pattern == InfillPattern::LockedZag {
                    // Fill.cpp:298-299
                    // C++: auto nozzle_diameter = float(object->print()->config().nozzle_diameter.get_at(layerm.region().extruder(extrusion_role) - 1));
                    // The extruder(role) call is kept for its Unknown-role
                    // error semantics; the per-extruder get_at collapses.
                    let _ = layerm
                        .region()
                        .extruder(extrusion_role)
                        .map_err(crate::Error::Config)?;
                    let nozzle_diameter = object.print().config().nozzle_diameter;
                    // Fill.cpp:300
                    let skin_flow = if params.bridge {
                        params.flow
                    } else {
                        Flow::new_from_config_width(
                            extrusion_role,
                            region_config.skin_infill_line_width,
                            nozzle_diameter,
                            surface_thickness,
                        )?
                    };
                    // add skin flow
                    // Fill.cpp:302
                    append_flow_param(&mut lock_param.skin_flow_params, skin_flow, &surface.expolygon);

                    // Fill.cpp:304
                    let skeleton_flow = if params.bridge {
                        params.flow
                    } else {
                        Flow::new_from_config_width(
                            extrusion_role,
                            region_config.skeleton_infill_line_width,
                            nozzle_diameter,
                            surface_thickness,
                        )?
                    };
                    // add skeleton flow
                    // Fill.cpp:306
                    append_flow_param(
                        &mut lock_param.skeleton_flow_params,
                        skeleton_flow,
                        &surface.expolygon,
                    );

                    // add skin density
                    // Fill.cpp:309
                    let skin_density = (0.01 * region_config.skin_infill_density) as f32;
                    // Fill.cpp:311-313
                    append_density_param(
                        &mut lock_param.skin_density_params,
                        skin_density,
                        &surface.expolygon,
                    );
                    append_density_param(
                        &mut lock_param.skin_depths_params,
                        scale(region_config.skin_infill_depth) as f32,
                        &surface.expolygon,
                    );
                    append_density_param(
                        &mut lock_param.locked_depths_params,
                        scale(region_config.infill_lock_depth) as f32,
                        &surface.expolygon,
                    );

                    // add skeleton densitys
                    // Fill.cpp:317-318
                    let skeleton_density = (0.01 * region_config.skeleton_infill_density) as f32;
                    append_density_param(
                        &mut lock_param.skeleton_density_params,
                        skeleton_density,
                        &surface.expolygon,
                    );
                }
                // Fill.cpp:320-323 — std::set find/insert by comparator
                // equivalence (see porting notes).
                let it_params = match set_surface_params
                    .iter()
                    .position(|p| !p.lt(&params) && !params.lt(p))
                {
                    Some(pos) => pos,
                    None => {
                        set_surface_params.push(params);
                        set_surface_params.len() - 1
                    }
                };
                // Fill.cpp:324 — &surface - &front() == surface_index
                region_to_surface_params[region_id][surface_index] = Some(it_params);
            }
        }
    }

    // Fill.cpp:328-332 — iterate the set in its sorted (comparator) order
    // and assign each entry its index in the linear vector.
    let mut sorted_params: Vec<usize> = (0..set_surface_params.len()).collect();
    sorted_params.sort_by(|&a, &b| {
        if set_surface_params[a].lt(&set_surface_params[b]) {
            std::cmp::Ordering::Less
        } else if set_surface_params[b].lt(&set_surface_params[a]) {
            std::cmp::Ordering::Greater
        } else {
            std::cmp::Ordering::Equal
        }
    });
    surface_fills.reserve(set_surface_params.len());
    for &insertion_idx in &sorted_params {
        // Fill.cpp:330
        set_surface_params[insertion_idx].idx = surface_fills.len();
        // Fill.cpp:331
        surface_fills.push(SurfaceFill::new(set_surface_params[insertion_idx].clone()));
    }

    // Fill.cpp:334-359
    for region_id in 0..layer.regions().len() {
        // Fill.cpp:335
        let layerm = &layer.regions()[region_id];
        // Fill.cpp:336-337
        for (surface_index, surface) in layerm.fill_surfaces.surfaces.iter().enumerate() {
            if surface.surface_type != SurfaceType::InternalVoid {
                // Fill.cpp:338-339
                if let Some(params_pos) = region_to_surface_params[region_id][surface_index] {
                    // Fill.cpp:340
                    let fill = &mut surface_fills[set_surface_params[params_pos].idx];
                    // Fill.cpp:341-347
                    if fill.region_id == usize::MAX {
                        fill.region_id = region_id;
                        fill.surface = surface.clone();
                        // C++ emplace_back(std::move(fill.surface.expolygon))
                        // — the surface copy inside `fill` is left with an
                        // empty expolygon, exactly like the moved-from C++ one.
                        let exp = std::mem::take(&mut fill.surface.expolygon);
                        fill.expolygons.push(exp);
                        // BBS
                        fill.region_id_group.push(region_id);
                        fill.no_overlap_expolygons = layerm.fill_no_overlap_expolygons.clone();
                    } else {
                        // Fill.cpp:348-356
                        fill.expolygons.push(surface.expolygon.clone());
                        // BBS
                        // Fill.cpp:351-355
                        if !fill.region_id_group.contains(&region_id) {
                            fill.region_id_group.push(region_id);
                            // C++ union_ex(fill.no_overlap_expolygons, layerm.fill_no_overlap_expolygons)
                            fill.no_overlap_expolygons = crate::clipper_utils::union(
                                &fill.no_overlap_expolygons,
                                &layerm.fill_no_overlap_expolygons,
                            );
                        }
                    }
                }
            }
        }
    }

    // Fill.cpp:361-374
    {
        // Fill.cpp:362
        let mut all_polygons: Vec<Polygon> = Vec::new();
        let n_fills = surface_fills.len();
        // Fill.cpp:363-364
        for i in 0..n_fills {
            let fill = &mut surface_fills[i];
            if !fill.expolygons.is_empty() {
                // Fill.cpp:365-370
                if fill.expolygons.len() > 1 || !all_polygons.is_empty() {
                    // Fill.cpp:366
                    let polys = to_polygons(&std::mem::take(&mut fill.expolygons));
                    // Make a union of polygons, use a safety offset, subtract the preceding polygons.
                    // Bridges are processed first (see SurfaceFill::operator<())
                    // Fill.cpp:369
                    fill.expolygons = if all_polygons.is_empty() {
                        union_safety_offset_ex(&polys)
                    } else {
                        diff_ex_polygons_polygons(&polys, &all_polygons, ApplySafetyOffset::Yes)
                    };
                    // Fill.cpp:370
                    all_polygons.extend(polys);
                // Fill.cpp:371-372
                } else if i + 1 != n_fills {
                    all_polygons.extend(to_polygons(&fill.expolygons));
                }
            }
        }
    }

    // we need to detect any narrow surfaces that might collapse
    // when adding spacing below
    // such narrow surfaces are often generated in sloping walls
    // by bridge_over_infill() and combine_infill() as a result of the
    // subtraction of the combinable area from the layer infill area,
    // which leaves small areas near the perimeters
    // we are going to grow such regions by overlapping them with the void (if any)
    // TODO: detect and investigate whether there could be narrow regions without
    // any void neighbors
    // Fill.cpp:385-450
    if has_internal_voids {
        // Internal voids are generated only if "infill_only_where_needed" or "infill_every_layers" are active.
        // Fill.cpp:387-392
        let mut distance_between_surfaces: Coord = 0;
        let mut surfaces_polygons: Vec<Polygon> = Vec::new();
        let mut voids: Vec<Polygon> = Vec::new();
        let mut region_internal_infill: i32 = -1;
        let mut region_solid_infill: i32 = -1;
        let mut region_some_infill: i32 = -1;
        // Fill.cpp:393-403
        for surface_fill in &surface_fills {
            if !surface_fill.expolygons.is_empty() {
                // Fill.cpp:395
                distance_between_surfaces = distance_between_surfaces
                    .max(surface_fill.params.flow.scaled_spacing());
                // Fill.cpp:396
                let polys = to_polygons(&surface_fill.expolygons);
                if surface_fill.surface.surface_type == SurfaceType::InternalVoid {
                    voids.extend(polys);
                } else {
                    surfaces_polygons.extend(polys);
                }
                // Fill.cpp:397-402
                if surface_fill.surface.surface_type == SurfaceType::InternalSolid {
                    region_internal_infill = surface_fill.region_id as i32;
                }
                if surface_fill.surface.is_solid() {
                    region_solid_infill = surface_fill.region_id as i32;
                }
                if surface_fill.surface.surface_type != SurfaceType::InternalVoid {
                    region_some_infill = surface_fill.region_id as i32;
                }
            }
        }
        // Fill.cpp:404
        if !voids.is_empty() && !surfaces_polygons.is_empty() {
            // First clip voids by the printing polygons, as the voids were ignored by the loop above during mutual clipping.
            // Fill.cpp:406
            voids = diff_polygons(&voids, &surfaces_polygons);
            // Corners of infill regions, which would not be filled with an extrusion path with a radius of distance_between_surfaces/2
            // Fill.cpp:408-410 — C++ passes scaled deltas; the crate's offset
            // helpers take millimeters (ClipperSafetyOffset = 10 scaled units,
            // ClipperUtils.hpp).
            let collapsed = diff_polygons(
                &surfaces_polygons,
                &opening_polygons_2(
                    &surfaces_polygons,
                    crate::unscale(distance_between_surfaces / 2),
                    crate::unscale(distance_between_surfaces / 2)
                        + 10.0 / crate::SCALING_FACTOR,
                ),
            );
            // FIXME why the voids are added to collapsed here? First it is expensive, second the result may lead to some unwanted regions being
            // added if two offsetted void regions merge.
            // polygons_append(voids, collapsed);
            // Fill.cpp:414
            let extensions = intersection_ex_polygons_polygons(
                &expand_polygons(&collapsed, crate::unscale(distance_between_surfaces)),
                &voids,
                ApplySafetyOffset::Yes,
            );
            // Now find an internal infill SurfaceFill to add these extrusions to.
            // Fill.cpp:416-423
            let mut region_id: usize = 0;
            if region_internal_infill != -1 {
                region_id = region_internal_infill as usize;
            } else if region_solid_infill != -1 {
                region_id = region_solid_infill as usize;
            } else if region_some_infill != -1 {
                region_id = region_some_infill as usize;
            }
            // Fill.cpp:424
            let layerm = &layer.regions()[region_id];
            // Fill.cpp:425-429
            let mut internal_solid_fill: Option<usize> = None;
            for (i, surface_fill) in surface_fills.iter().enumerate() {
                if surface_fill.surface.surface_type == SurfaceType::InternalSolid
                    && (layer.height - surface_fill.params.flow.height()).abs() < EPSILON
                {
                    internal_solid_fill = Some(i);
                    break;
                }
            }
            match internal_solid_fill {
                // Fill.cpp:430-444
                None => {
                    // Produce another solid fill.
                    let mut params = SurfaceFillParams::default();
                    // Fill.cpp:433
                    params.extruder = layerm
                        .region()
                        .extruder(FlowRole::SolidInfill)
                        .map_err(crate::Error::Config)?;
                    // Fill.cpp:434
                    params.pattern = if layerm.region().config().top_fill_pattern
                        == InfillPattern::Monotonic
                    {
                        InfillPattern::Monotonic
                    } else {
                        InfillPattern::Rectilinear
                    };
                    // Fill.cpp:435
                    params.density = 100.0;
                    // Fill.cpp:436
                    params.extrusion_role = ExtrusionRole::InternalInfill;
                    // Fill.cpp:437
                    params.angle = deg2rad(layerm.region().config().fill_angle) as f32;
                    // calculate the actual flow we'll be using for this infill
                    // Fill.cpp:439 — C++ layerm.flow(frSolidInfill) reads
                    // m_layer->height; threaded as layer.height.
                    params.flow = layerm.flow(FlowRole::SolidInfill, layer.height)?;
                    // Fill.cpp:440
                    params.spacing = params.flow.spacing();
                    // Fill.cpp:441-444
                    surface_fills.push(SurfaceFill::new(params));
                    let back = surface_fills.last_mut().unwrap();
                    back.surface.surface_type = SurfaceType::InternalSolid;
                    back.surface.thickness = layer.height;
                    back.expolygons = extensions;
                }
                // Fill.cpp:445-448
                Some(i) => {
                    let mut extensions = extensions;
                    extensions.append(&mut surface_fills[i].expolygons);
                    surface_fills[i].expolygons = union_ex(&extensions);
                }
            }
        }
    }

    // BBS: detect narrow internal solid infill area and use ipConcentricInternal pattern instead
    // Fill.cpp:453-546
    if layer.object().config().detect_narrow_internal_solid_infill {
        // Fill.cpp:454 — declared but unused in the C++ body too.
        let _narrow_threshold: f64 = scale(NARROW_INFILL_AREA_THRESHOLD) as f64 * 2.0;
        // Fill.cpp:455-456
        let mut lower_internal_areas: Vec<ExPolygon> = Vec::new();
        // (lower_internal_bbox is computed but never read afterwards in C++.)
        // Fill.cpp:457-464
        if let Some(lower_layer) = lower_layer {
            for layerm in lower_layer.regions() {
                // Fill.cpp:459
                let internal_surfaces = layerm
                    .fill_surfaces
                    .filter_by_types(&[SurfaceType::Internal, SurfaceType::InternalVoid]);
                // Fill.cpp:460-461
                for surface in internal_surfaces {
                    lower_internal_areas.push(surface.expolygon.clone());
                }
            }
            // Fill.cpp:463
            let _lower_internal_bbox = get_extents(&lower_internal_areas);
        }
        // Fill.cpp:465-466
        let surface_fills_size = surface_fills.len();
        for i in 0..surface_fills_size {
            // Fill.cpp:467-468
            if surface_fills[i].surface.surface_type != SurfaceType::InternalSolid {
                continue;
            }

            // Fill.cpp:470-473
            let expolygons_size = surface_fills[i].expolygons.len();
            let mut narrow_expoly_idx: Vec<usize> = Vec::new();
            let mut narrow_floating_expoly_idx: Vec<usize> = Vec::new();
            // (C++ also declares `std::vector<bool> use_floating_filler;` — unused.)
            // BBS: get the index list of narrow expolygon
            // Fill.cpp:475-487
            for j in 0..expolygons_size {
                // Fill.cpp:476
                let bbox = get_extents_expoly(&surface_fills[i].expolygons[j]);
                // Fill.cpp:477 — bbox.inflated(scale_(2)); the crate's
                // BoundingBox names it `expanded`. expand a little
                let clipped_internals = clip_clipper_polygons_with_subject_bbox_expolygons(
                    &lower_internal_areas,
                    &bbox.expanded(scale(2.0)),
                    false,
                );
                // Fill.cpp:478
                let clipped_internal_bbox = get_extents_polygons(&clipped_internals);
                // Fill.cpp:479-486
                if is_narrow_infill_area(&surface_fills[i].expolygons[j]) {
                    // Fill.cpp:480 — offset_ex(expoly, SCALED_EPSILON): the
                    // crate's offset helpers take millimeters.
                    if !clipped_internals.is_empty()
                        && bbox.intersects(&clipped_internal_bbox)
                        && !intersection_ex_expolygons_polygons(
                            &offset_expolygon(
                                &surface_fills[i].expolygons[j],
                                SCALED_EPSILON / crate::SCALING_FACTOR,
                                OffsetJoinType::Miter,
                            ),
                            &clipped_internals,
                        )
                        .is_empty()
                    {
                        narrow_floating_expoly_idx.push(j);
                    } else {
                        narrow_expoly_idx.push(j);
                    }
                }
            }

            // Fill.cpp:489-492
            if narrow_expoly_idx.is_empty() && narrow_floating_expoly_idx.is_empty() {
                // BBS: has no narrow expolygon
                continue;
            // Fill.cpp:493-497
            } else if narrow_floating_expoly_idx.len() == expolygons_size {
                surface_fills[i].params.pattern = InfillPattern::FloatingConcentric;
                surface_fills[i].params.extrusion_role = ExtrusionRole::FloatingVerticalShell;
                surface_fills[i].surface.surface_type = SurfaceType::FloatingVerticalShell;
            // Fill.cpp:498-500
            } else if narrow_expoly_idx.len() == expolygons_size {
                surface_fills[i].params.pattern = InfillPattern::ConcentricInternal;
            } else {
                // BBS: some expolygons are narrow, spilit surface_fills[i] and rearrange the expolygons
                // Fill.cpp:505-518
                if !narrow_expoly_idx.is_empty() {
                    let mut params = surface_fills[i].params.clone();
                    params.pattern = InfillPattern::ConcentricInternal;
                    // Fill.cpp:508-513
                    let region_id_i = surface_fills[i].region_id;
                    let thickness_i = surface_fills[i].surface.thickness;
                    let region_id_group_i = surface_fills[i].region_id_group.clone();
                    let no_overlap_i = surface_fills[i].no_overlap_expolygons.clone();
                    surface_fills.push(SurfaceFill::new(params));
                    let back_idx = surface_fills.len() - 1;
                    surface_fills[back_idx].region_id = region_id_i;
                    surface_fills[back_idx].surface.surface_type = SurfaceType::InternalSolid;
                    surface_fills[back_idx].surface.thickness = thickness_i;
                    surface_fills[back_idx].region_id_group = region_id_group_i;
                    surface_fills[back_idx].no_overlap_expolygons = no_overlap_i;
                    // Fill.cpp:514-517
                    for j in 0..narrow_expoly_idx.len() {
                        // BBS: move the narrow expolygons to new surface_fills.back();
                        let exp = std::mem::take(
                            &mut surface_fills[i].expolygons[narrow_expoly_idx[j]],
                        );
                        surface_fills[back_idx].expolygons.push(exp);
                    }
                }

                // Fill.cpp:520-534
                if !narrow_floating_expoly_idx.is_empty() {
                    let mut params = surface_fills[i].params.clone();
                    params.pattern = InfillPattern::FloatingConcentric;
                    params.extrusion_role = ExtrusionRole::FloatingVerticalShell;
                    // Fill.cpp:524-529
                    let region_id_i = surface_fills[i].region_id;
                    let thickness_i = surface_fills[i].surface.thickness;
                    let region_id_group_i = surface_fills[i].region_id_group.clone();
                    let no_overlap_i = surface_fills[i].no_overlap_expolygons.clone();
                    surface_fills.push(SurfaceFill::new(params));
                    let back_idx = surface_fills.len() - 1;
                    surface_fills[back_idx].region_id = region_id_i;
                    surface_fills[back_idx].surface.surface_type =
                        SurfaceType::FloatingVerticalShell;
                    surface_fills[back_idx].surface.thickness = thickness_i;
                    surface_fills[back_idx].region_id_group = region_id_group_i;
                    surface_fills[back_idx].no_overlap_expolygons = no_overlap_i;
                    // Fill.cpp:530-533
                    for j in 0..narrow_floating_expoly_idx.len() {
                        // BBS: move the narrow expolygons to new surface_fills.back();
                        let exp = std::mem::take(
                            &mut surface_fills[i].expolygons[narrow_floating_expoly_idx[j]],
                        );
                        surface_fills[back_idx].expolygons.push(exp);
                    }
                }

                // Fill.cpp:536-538
                let mut to_be_delete = narrow_floating_expoly_idx.clone();
                to_be_delete.extend(narrow_expoly_idx.iter().cloned());
                to_be_delete.sort_unstable();

                // Fill.cpp:540-543
                for j in (0..to_be_delete.len()).rev() {
                    // BBS: delete the narrow expolygons from old surface_fills
                    surface_fills[i].expolygons.remove(to_be_delete[j]);
                }
            }
        }
    }

    // Fill.cpp:548
    Ok(surface_fills)
}

// Fill.cpp:571-586
// void Layer::set_outlook_range(LockRegionParam &lock_param)
//
// C++ defines this Layer method inside Fill.cpp; the Rust port mirrors that
// file placement with an `impl Layer` block here.
impl Layer {
    pub fn set_outlook_range(&mut self, lock_param: &mut LockRegionParam) {
        // Fill.cpp:572
        for region_id in 0..self.regions().len() {
            // Fill.cpp:573
            let layerm = &mut self.regions_mut()[region_id];

            // Fill.cpp:575
            // C++: layerm.region().config().infill_instead_top_bottom_surfaces
            //      && layerm.region().config().sparse_infill_pattern == ipLockedZag
            if layerm.region().config().infill_instead_top_bottom_surfaces
                && layerm.region().config().fill_pattern == InfillPattern::LockedZag
            {
                // Fill.cpp:576-579
                for surface in &layerm.fill_surfaces.surfaces {
                    if surface.surface_type != SurfaceType::Internal {
                        lock_param.outlook.push(surface.expolygon.clone());
                    }
                }
                // Fill.cpp:580
                lock_param.outlook = union_safety_offset_ex_expolygons(&lock_param.outlook);
                // Fill.cpp:581-583
                // C++: layerm.fill_surfaces.keep_type(SurfaceType::stInternal, exps);
                // (the two-argument keep_type overload is
                // `keep_type_collect_exps` in this crate)
                let mut exps: Vec<ExPolygon> = Vec::new();
                layerm
                    .fill_surfaces
                    .keep_type_collect_exps(SurfaceType::Internal, &mut exps);
                layerm.fill_surfaces.append(exps, SurfaceType::Internal);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// BLOCKED symbols (not ported here)
// ---------------------------------------------------------------------------
//
// The following `Fill.cpp` symbols require infrastructure that does not yet
// exist in this Rust crate and therefore cannot be ported faithfully without
// fabricating behavior. They are intentionally left unported. The config
// hierarchy (layer.object().print().config() / layerm.region().config())
// is WIRED and is no longer a blocker for any of them.
//
// - export_group_fills_to_svg (Fill.cpp:551-570)
//     Debug-only (SLIC3R_DEBUG_SLICE_PROCESSING), depends on SVG + surface
//     type legend helpers; not part of byte-exact G-code output.
//
// - Layer::make_fills (Fill.cpp:588-770)
// - Layer::generate_sparse_infill_polylines_for_anchoring (Fill.cpp:772-875)
// - Layer::make_ironing (Fill.cpp:879-1103)
//     BLOCKED on the virtual Fill dispatch infrastructure: a Fill base type
//     with Fill::new_from_type (FillBase.cpp:33-67) returning concrete
//     fillers (FillConcentricInternal, FillLightning::Filler,
//     FillMonotonicLineWGapFill, FillFloatingConcentric, ...) behind a
//     common fill_surface / fill_surface_extrusion interface
//     (FillBase.cpp:94-172), plus the FillAdaptive::Octree /
//     FillLightning::Generator inputs. The crate currently has standalone
//     per-pattern fillers without the polymorphic base; a divergent
//     simplified make_fills lives in layer.rs (Layer::make_fills) driving
//     the divergent group_fills in fill/mod.rs. Once the Fill factory
//     exists, the faithful group_fills above is the drop-in input for the
//     faithful make_fills.
