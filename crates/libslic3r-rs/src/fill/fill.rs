//! Faithful 1:1 port of BambuStudio `src/libslic3r/Fill/Fill.cpp`.
//!
//! This file mirrors the C++ source line-by-line (snake_case). It ports the
//! self-contained pieces of `Fill.cpp` that do not require the
//! `Print -> PrintObject -> Layer -> LayerRegion` configuration object graph
//! or virtual `Fill` dispatch, both of which are not yet present in this Rust
//! crate. The large `Layer::*` methods (`group_fills`, `make_fills`,
//! `make_ironing`, `set_outlook_range`,
//! `generate_sparse_infill_polylines_for_anchoring`) and the debug
//! `export_group_fills_to_svg` are BLOCKED — see the module-level notes at the
//! bottom of this file for the exact reasons.
//!
//! C++ source: src/libslic3r/Fill/Fill.cpp
//! C++ header: src/libslic3r/Fill/Fill.hpp

use crate::clipper_utils::{offset_expolygon, OffsetJoinType};
use crate::extrusion_entity::ExtrusionRole;
use crate::flow::Flow;
use crate::geometry::ExPolygon;
use crate::libslic3r::scale;
use crate::print_config::InfillPattern;

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
            // InfillPattern(0) is the first enum value of the C++ InfillPattern.
            pattern: InfillPattern::default(),
            skin_pattern: InfillPattern::default(),
            skeleton_pattern: InfillPattern::default(),
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
    let offsets = offset_expolygon(
        expolygon,
        -(scale(NARROW_INFILL_AREA_THRESHOLD) as f64),
        OffsetJoinType::Miter,
    );
    // Fill.cpp:160-161
    if offsets.is_empty() {
        return true;
    }

    // Fill.cpp:163
    false
}

// ---------------------------------------------------------------------------
// BLOCKED symbols (not ported here)
// ---------------------------------------------------------------------------
//
// The following `Fill.cpp` symbols require infrastructure that does not yet
// exist in this Rust crate and therefore cannot be ported faithfully without
// fabricating behavior. They are intentionally left unported:
//
// - group_fills (Fill.cpp:166-549)
//     Needs the full Print -> PrintObject -> Layer -> LayerRegion config
//     graph: layer.object()->config(), layerm.region().config(),
//     layerm.region().extruder(role), layerm.bridging_flow(...),
//     layerm.flow(...), layer.get_process_config_idx(...),
//     layer.get_filament_config_idx(...), plus the full LockRegionParam
//     fields (skin_flow_params / skeleton_flow_params / *_density_params /
//     *_depths_params) which are an empty stub in the current crate. It also
//     references many InfillPattern variants absent from the crate enum
//     (ipLockedZag, ipCrossZag, ip2DLattice, ipFloatingConcentric,
//     ipConcentricInternal, ipStars, ipSupportBase, ipCount, ...) and the
//     ClipperUtils::clip_clipper_polygons_with_subject_bbox helper. A
//     divergent simplified variant currently lives in fill/mod.rs.
//
// - export_group_fills_to_svg (Fill.cpp:551-570)
//     Debug-only (SLIC3R_DEBUG_SLICE_PROCESSING), depends on SVG + surface
//     type legend helpers; not part of byte-exact G-code output.
//
// - Layer::set_outlook_range (Fill.cpp:571-586)
// - Layer::make_fills (Fill.cpp:588-770)
// - Layer::generate_sparse_infill_polylines_for_anchoring (Fill.cpp:772-875)
// - Layer::make_ironing (Fill.cpp:879-1103)
//     All are Layer methods that depend on virtual Fill dispatch
//     (Fill::new_from_type, f->fill_surface_extrusion, dynamic_cast to
//     concrete fillers), the Print/PrintObject/Layer object graph, and config
//     accessors that are not threaded through the current Rust Layer.
