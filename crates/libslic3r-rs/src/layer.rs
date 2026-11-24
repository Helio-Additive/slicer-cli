//! Layer and LayerRegion structures
//!
//! Direct port of BambuStudio's Layer.cpp and Layer.hpp
//!
//! # C++ Reference
//! - Layer.cpp
//! - Layer.hpp
//!
//! # Overview
//! A Layer represents a horizontal slice through a PrintObject at a specific Z height.
//! Each layer contains:
//! - Collection of LayerRegions (one per PrintRegion)
//! - Perimeter paths (outer walls)
//! - Fill/infill paths (interior)
//! - References to upper/lower layers

use crate::clipper_utils::{union_ex, OffsetJoinType};

/// R730 — per-filler cost census for `fill_loop`. R729 localised the
/// CLIPPER_UNION_GEO slowdown to this loop (0.248s -> 1.806s, 7.3x); this says
/// WHICH filler. `FILLPROF=1` prints at exit.
pub static FILLPROF_FVS_NS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static FILLPROF_CI_NS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static FILLPROF_FVS_N: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static FILLPROF_CI_N: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static FILLPROF_FVS_V: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static FILLPROF_CI_V: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static FILLPROF_LOOP_NS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static FILLPROF_GROUP_NS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

pub fn fillprof_report() {
    use std::sync::atomic::Ordering::Relaxed;
    if !crate::probe_enabled("FILLPROF") {
        return;
    }
    let f = FILLPROF_FVS_NS.load(Relaxed) as f64 / 1e6;
    let c = FILLPROF_CI_NS.load(Relaxed) as f64 / 1e6;
    let tot = FILLPROF_LOOP_NS.load(Relaxed) as f64 / 1e6;
    eprintln!(
        "[GFPROF] union_block {:.1}ms | narrow_block {:.1}ms (calls={}) | bbox_clip {:.1}ms | is_narrow {:.1}ms (calls={})",
        crate::fill::GFPROF_UNION_NS.load(Relaxed) as f64 / 1e6,
        crate::fill::GFPROF_NARROW_NS.load(Relaxed) as f64 / 1e6,
        crate::fill::GFPROF_NARROW_CALLS.load(Relaxed),
        crate::fill::GFPROF_CLIP_NS.load(Relaxed) as f64 / 1e6,
        crate::fill::GFPROF_ISNARROW_NS.load(Relaxed) as f64 / 1e6,
        crate::fill::GFPROF_ISNARROW_CALLS.load(Relaxed),
    );
    eprintln!(
        "[FILLPROF] group_fills {:.1}ms | loop_total {:.1}ms | FloatingConcentric {:.1}ms calls={} verts={} | ConcentricInternal {:.1}ms calls={} verts={} | other {:.1}ms | concentric share {:.0}%",
        FILLPROF_GROUP_NS.load(Relaxed) as f64 / 1e6, tot, f, FILLPROF_FVS_N.load(Relaxed), FILLPROF_FVS_V.load(Relaxed),
        c, FILLPROF_CI_N.load(Relaxed), FILLPROF_CI_V.load(Relaxed),
        (tot - f - c).max(0.0),
        if tot > 0.0 { 100.0 * (f + c) / tot } else { 0.0 }
    );
}

/// The conformal fill of one island, as `FillRectilinear::fill_surface` takes it
/// (FillRectilinear.cpp:3075-3081):
///     if (params.conformal) {
///         Polylines conformal_out;
///         if (FillConformal::fill_surface(*this, surface, params, conformal_out))
///             return conformal_out;
///     }
/// with the filler's members and the FillParams set as `Layer::make_fills`
/// (Fill.cpp:698-704, 787-811, 844-845) and
/// `Layer::generate_sparse_infill_polylines_for_anchoring` (Fill.cpp:911-958,
/// `using_internal_flow` = None there) set them. `None` when the surface is not
/// conformal, its filler is not one that runs that method, or the fill declined
/// (the caller then lays its lines).
///
/// The fillers that run `FillRectilinear::fill_surface` (it is virtual, and
/// FillMonotonic, FillGrid, FillSupportBase ... override it) are FillRectilinear,
/// FillAlignedRectilinear, FillZigZag and FillCrossZag (FillBase.cpp:41-42,
/// 62-63). `conformal_infill` is only set for surfaces that are not solid
/// (Fill.cpp:240), whose pattern is the region's sparse infill pattern
/// (Fill.cpp:239), `config_pattern` here.
// FIDELITY-NOTE: the skin pattern of ipLockedZag also goes through
// FillRectilinear::fill_surface with `conformal` set (FillRectilinear.cpp:3942-3967);
// `fill_surface_locked_zag` of this crate lays the skin as lines.
#[allow(clippy::too_many_arguments)]
fn conformal_fill_surface(
    config_pattern: crate::print_config::InfillPattern,
    params: &crate::fill::SurfaceFillParams,
    surface: &crate::surface::Surface,
    expoly: &crate::geometry::ExPolygon,
    region_id: usize,
    layer_id: usize,
    print_z: f64,
    density: f64,
    link_max_length: crate::Coord,
    resolution: f64,
    using_internal_flow: Option<bool>,
    zag: &ZagFill,
) -> Option<Vec<crate::geometry::Polyline>> {
    if !params.conformal_infill {
        return None;
    }
    if !matches!(
        config_pattern,
        crate::print_config::InfillPattern::Rectilinear
            | crate::print_config::InfillPattern::AlignedRectilinear
            | crate::print_config::InfillPattern::ZigZag
            | crate::print_config::InfillPattern::CrossZag
    ) {
        return None;
    }
    // Fill.cpp:698-704 — the filler's members.
    let mut f = crate::fill::fill_base::Fill::new();
    // f->set_bounding_box(bbox): the fill drivers of this crate do not carry the
    // object's bounding box; the conformal fill reads it only in code that its
    // entry does not call (FillConformal.cpp:42-48).
    f.layer_id = layer_id;
    f.lock_region_id = region_id;
    f.z = print_z;
    // f->angle is in radians (Fill.cpp:253, deg2rad of infill_direction);
    // SurfaceFillParams::angle of this crate is in degrees.
    f.angle = (std::f64::consts::PI * params.angle as f64 / 180.0) as f32;
    // Fill.cpp:704-709 — `f->angle -=` / `+= infill_rotate_step * (f->layer_id / 2)`
    // for ipZigZag, in make_fills only.
    if let Some(infill_rotate_step) = zag.zigzag_rotate_step {
        let half_layers = (layer_id / 2) as f32;
        f.angle = if layer_id % 2 == 0 {
            (-infill_rotate_step).mul_add(half_layers, f.angle)
        } else {
            infill_rotate_step.mul_add(half_layers, f.angle)
        };
    }
    // Fill.cpp:787-789
    f.link_max_length = link_max_length;
    f.loop_clipping = crate::scale(params.flow.nozzle_diameter() * 0.15);
    // Fill.cpp:844 — f->spacing = surface_fill.params.spacing;
    f.spacing = params.spacing;
    // Fill.cpp:791-811 — the FillParams.
    let mut fp = crate::fill::FillParams::new();
    fp.density = density;
    fp.multiline = params.multiline;
    fp.pattern = config_pattern;
    fp.dont_adjust = false;
    fp.anchor_length = params.anchor_length as f64;
    fp.anchor_length_max = params.anchor_length_max as f64;
    fp.resolution = resolution;
    if let Some(using_internal_flow) = using_internal_flow {
        // Fill.cpp:802-804
        fp.flow = params.flow.clone();
        fp.extrusion_role = params.extrusion_role;
        fp.using_internal_flow = using_internal_flow;
    } else {
        // Fill.cpp:939-958 leaves `params.flow` as constructed: `Flow()`, width 0,
        // so the hub of the conformal fill takes the line spacing for its line
        // width there (FillRadialZigZag.cpp:2076-2084).
        fp.flow = crate::flow::Flow::default_constructed();
    }
    fp.conformal = params.conformal_infill;
    fp.conformal_stagger = params.conformal_stagger;
    fp.conformal_link_keep_layers = params.conformal_link_keep_layers;
    fp.conformal_link_flip_layers = params.conformal_link_flip_layers;
    fp.conformal_pole = params.conformal_pole;
    fp.conformal_ray_count = params.conformal_ray_count;
    fp.conformal_hub_radius = params.conformal_hub_radius;
    // Fill.cpp:817-821 — `params.horiz_move` of an ipCrossZag surface, in
    // make_fills only (0 with FILL_ZAG_CONSISTENT=0 or FILL_CROSSZAG_SHIFT=0).
    fp.horiz_move = zag.horiz_move;
    // Fill.cpp:823, 826, 838-840 — params.symmetric_infill_y_axis and
    // params.symmetric_y_axis; the caller has mirrored `expoly` (Fill.cpp:841).
    if let Some(symmetric_y_axis) = zag.symmetric_y_axis {
        fp.symmetric_infill_y_axis = true;
        fp.symmetric_y_axis = symmetric_y_axis;
    }
    // Fill.cpp:845 — surface_fill.surface.expolygon = std::move(expoly);
    let mut conformal_surface = surface.clone();
    conformal_surface.expolygon = expoly.clone();
    let mut conformal_out: Vec<crate::geometry::Polyline> = Vec::new();
    if crate::fill::fill_conformal::fill_surface(&f, Some(&conformal_surface), &fp, &mut conformal_out) {
        Some(conformal_out)
    } else {
        None
    }
}

/// Fill.cpp:944 — `params.density = float(0.01 * surface_fill.params.density)`:
/// the product is taken in double (0.01 is a double literal) and rounded to
/// float once. FILL_DENSITY_DOUBLE=0 multiplies in float, which gives
/// 0.099999994 for 10 % where C++ has 0.1f.
fn fill_params_density(density: f32) -> f64 {
    if crate::faithful_gate("FILL_DENSITY_DOUBLE") {
        ((0.01f64 * density as f64) as f32) as f64
    } else {
        (0.01 * density) as f64
    }
}

/// Print.hpp:442 — `PrintObject::bounding_box()`:
///     BoundingBox(Point(- m_size.x() / 2, - m_size.y() / 2), Point(m_size.x() / 2, m_size.y() / 2))
/// with PrintObject.cpp:96 `m_size = (bbox.size() * (1. / SCALING_FACTOR)).cast<coord_t>()`,
/// `bbox` the object's raw bounding box (PrintObject.cpp:82). It is what
/// Fill.cpp:599, 699 hand the filler (`f->set_bounding_box(bbox)`).
// FIDELITY-NOTE: PrintObject.cpp:85-91 turns the raw bounding box by the
// difference of the Z rotations of the first model instance and this print
// instance; the mesh this crate's PrintObject holds is taken as it is.
pub(crate) fn print_object_bounding_box(
    mesh: &crate::triangle_mesh::TriangleMesh,
) -> Option<crate::geometry::BoundingBox> {
    let bbox = mesh.compute_bounding_box();
    let inv_scaling_factor = 1.0f64 / 0.00001f64;
    let size_x = ((bbox.max.x as f64 - bbox.min.x as f64) * inv_scaling_factor) as crate::Coord;
    let size_y = ((bbox.max.y as f64 - bbox.min.y as f64) * inv_scaling_factor) as crate::Coord;
    let mut out = crate::geometry::BoundingBox::empty();
    out.merge_point(crate::geometry::Point::new(-size_x / 2, -size_y / 2));
    out.merge_point(crate::geometry::Point::new(size_x / 2, size_y / 2));
    Some(out)
}

/// The members `Layer::make_fills` gives a FillZigZag, FillCrossZag or
/// FillLockedZag and its `FillParams` beyond those of a FillRectilinear
/// (Fill.cpp:699, 704-709, 817-826, 838-841).
#[derive(Clone, Default)]
struct ZagFill {
    /// FillRectilinear.hpp:165,174,187 — `has_consistent_pattern()`.
    consistent_pattern: bool,
    /// Fill.cpp:699 — `f->set_bounding_box(bbox)`.
    object_bounding_box: Option<crate::geometry::BoundingBox>,
    /// Fill.cpp:817-821 — `params.horiz_move`.
    horiz_move: f32,
    /// Fill.cpp:704-709 — the step `f->angle` turns by, for ipZigZag.
    zigzag_rotate_step: Option<f32>,
    /// Fill.cpp:838-840 — `params.symmetric_y_axis`, where
    /// `params.symmetric_infill_y_axis` is set.
    symmetric_y_axis: Option<crate::Coord>,
    /// Fill.cpp:812-815 — the surface's pattern is ipLockedZag
    /// (`params.locked_zag = true`, the lock region param and the skin and
    /// skeleton patterns handed to the filler).
    locked_zag: bool,
}

/// `in_make_fills`: `Layer::make_fills` (Fill.cpp:698-846) sets all of them;
/// `Layer::generate_sparse_infill_polylines_for_anchoring` (Fill.cpp:911-971)
/// makes the same filler and sets its bounding box only.
///
/// FILL_ZAG_CONSISTENT=0 fills ipZigZag and ipCrossZag as a FillRectilinear, and
/// FILL_LOCKED_ZAG=0 ipLockedZag.
// FIDELITY-NOTE: in `generate_sparse_infill_polylines_for_anchoring`
// (Fill.cpp:969-971) an ipLockedZag filler runs `FillRectilinear::fill_surface`
// with `params.locked_zag` unset and no lock region param: the lines of a
// FillRectilinear with `has_consistent_pattern()`. Only the consistent pattern
// is taken there.
fn zag_fill(
    params: &crate::fill::SurfaceFillParams,
    object_bounding_box: Option<crate::geometry::BoundingBox>,
    layer_id: usize,
    in_make_fills: bool,
) -> ZagFill {
    use crate::print_config::InfillPattern as P;
    let pattern = match params.sparse_pattern {
        Some(p @ (P::ZigZag | P::CrossZag))
            if crate::faithful_gate("FILL_ZAG_CONSISTENT") => p,
        Some(p @ P::LockedZag) if crate::faithful_gate("FILL_LOCKED_ZAG") => p,
        _ => return ZagFill::default(),
    };
    let mut out = ZagFill {
        consistent_pattern: true,
        object_bounding_box,
        ..ZagFill::default()
    };
    if !in_make_fills {
        return out;
    }
    out.locked_zag = pattern == P::LockedZag;
    let half_layers = (layer_id / 2) as f32;
    if pattern == P::ZigZag {
        // Fill.cpp:704-709; the sum is taken where the angle is.
        out.zigzag_rotate_step = Some(params.infill_rotate_step);
    } else {
        // Fill.cpp:817-821:
        //     if (f->layer_id % 2 == 0)
        //         params.horiz_move -= surface_fill.params.infill_shift_step * (f->layer_id / 2);
        //     else
        //         params.horiz_move += surface_fill.params.infill_shift_step * (f->layer_id / 2);
        out.horiz_move = if layer_id % 2 == 0 {
            (-params.infill_shift_step).mul_add(half_layers, 0.0f32)
        } else {
            params.infill_shift_step.mul_add(half_layers, 0.0f32)
        };
    }
    // Fill.cpp:823, 826 — params.symmetric_infill_y_axis; Fill.cpp:838-840:
    //     params.symmetric_y_axis = f->extended_object_bounding_box().center().x();
    if params.symmetric_infill_y_axis {
        if let Some(object_bbox) = out.object_bounding_box.as_ref() {
            out.symmetric_y_axis = Some(
                crate::fill::fill_rectilinear::extended_object_bounding_box(object_bbox)
                    .center()
                    .x,
            );
        }
    }
    out
}

/// ExPolygon.cpp:151-156 — `ExPolygon::symmetric_y`, MultiPoint.cpp:472-477.
fn expolygon_symmetric_y(expoly: &mut crate::geometry::ExPolygon, x_axis: crate::Coord) {
    for pt in expoly.contour.points.iter_mut() {
        pt.x = 2 * x_axis - pt.x;
    }
    for hole in expoly.holes.iter_mut() {
        for pt in hole.points.iter_mut() {
            pt.x = 2 * x_axis - pt.x;
        }
    }
}

/// R474 diagnostic (`FVS_DEBUG=1`): how many Concentric / FloatingConcentric
/// expolygons reach the filler, and how many of them had NO `no_overlap_expolygons`
/// to fill (Fill.cpp:740 — C++ emits nothing for those).
pub static FVS_EXPOLYS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static FVS_EMPTY_NOOVERLAP: std::sync::atomic::AtomicUsize =
    std::sync::atomic::AtomicUsize::new(0);
pub static FVS_REGION_SEEN: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static FVS_REGION_EMPTY: std::sync::atomic::AtomicUsize =
    std::sync::atomic::AtomicUsize::new(0);
/// R480: why `detect_floating_line` is forced off for a bead.
pub static FVS_LINES: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static FVS_FLAG_OFF: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static FVS_SELF_INT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static FVS_CLOSED_DUP: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static FVS_CLOSED_NODUP: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static FVS_NO_AREAS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
/// R481: detect_narrow_internal_solid_infill classification counters.
pub static FVS_CAND: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static FVS_NARROW: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static FVS_LOWER_EMPTY: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static FVS_CLIP_EMPTY: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static FVS_LAYERS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static FVS_NO_LOWER: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static FVS_LOWER_NOSURF: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
/// R482 (VSHELL_DEBUG=1): how many regularized vertical-shell pieces the
/// PrintObject.cpp:2040-2054 scattered-drop filter removes, and their area (in
/// milli-mm2). Islands lost here never become InternalSolid surfaces.
pub static VS_REG_IN: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static VS_REG_DROP: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static VS_REG_DROP_AREA: std::sync::atomic::AtomicUsize =
    std::sync::atomic::AtomicUsize::new(0);
pub static VS_C1: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static VS_C1_TINY: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static VS_C2: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static VS_SAVED: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static VS_SHELL_IN: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static VS_SHELL_IN_AREA: std::sync::atomic::AtomicUsize =
    std::sync::atomic::AtomicUsize::new(0);
pub static VS_REG_OUT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static VS_REG_OUT_AREA: std::sync::atomic::AtomicUsize =
    std::sync::atomic::AtomicUsize::new(0);
/// R486: the three terms that form `shell` in the vertical-shell promotion step
/// (PrintObject.cpp:1993-1999), in milli-mm2.
pub static VS_T_N: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static VS_T_INTERNAL: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static VS_T_SHELLINT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static VS_T_DIFF: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static VS_T_SOLID: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static VS_T_HOLES: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static VS_T_HOLES_EMPTY: std::sync::atomic::AtomicUsize =
    std::sync::atomic::AtomicUsize::new(0);
pub static VS_T_SHELLINT_EMPTY: std::sync::atomic::AtomicUsize =
    std::sync::atomic::AtomicUsize::new(0);

use crate::extrusion_entity::{
    extrusion_entities_append_paths, ExtrusionEntityCollection, ExtrusionLoop, ExtrusionPath,
    ExtrusionRole,
};
use crate::fill::fill_rectilinear::{
    generate_fill_rectilinear, generate_fill_rectilinear_monotonic,
};
use crate::fill::{generate_infill, InfillConfig, InfillPath, InfillPattern};
use crate::flow::{Flow, FlowRole};
use crate::geometry::{
    BoundingBox, ExPolygons, Point, Polygon, Polygons, Polyline, Polylines,
};
use crate::perimeter_generator::WallGeneratorMode;
use crate::region_config::PrintRegionConfig;
use crate::surface::{Surface, SurfaceType};
use crate::surface_collection::SurfaceCollection;
use crate::{scale, Result};

use std::sync::Arc;

/// Represents a region within a layer, tied to a specific print configuration.
/// Layer.hpp:35
#[derive(Debug, Clone)]
pub struct LayerRegion {
    /// Layer.hpp:36
    layer_id: usize,
    /// Layer.hpp:37
    region_id: usize,

    // Surfaces
    /// Layer.hpp:40
    pub slices: SurfaceCollection,

    /// Layer.hpp:44
    pub raw_slices: SurfaceCollection,

    /// Layer.hpp:47-48
    pub raw_counter_circle_compensation: Polygons,
    pub raw_holes_circle_compensation: Polygons,

    // Thin fills
    /// Layer.hpp:53
    pub thin_fills: ExtrusionEntityCollection,

    // Fill regions
    /// Layer.hpp:57
    pub fill_expolygons: ExPolygons,
    /// Layer.hpp:60
    pub fill_surfaces: SurfaceCollection,
    /// Layer.hpp:63
    pub fill_no_overlap_expolygons: ExPolygons,

    /// Layer.hpp:66
    pub unsupported_bridge_edges: Polylines,

    /// Layer.hpp:72 `std::vector<AdaptiveMergePatch> adaptive_merge_patches`: the multi-diameter
    /// adaptive layer height patches of this region (none unless a caller fills them).
    pub adaptive_merge_patches: Vec<crate::adaptive_layer_merge::AdaptiveMergePatch>,

    // Extrusions
    /// Layer.hpp:70
    pub perimeters: ExtrusionEntityCollection,

    /// Layer.hpp:74
    pub fills: ExtrusionEntityCollection,

    /// Lower layer slices for overhang detection (set before make_perimeters)
    /// LoopNodes produced by the last make_perimeters() call (node_id LOCAL to
    /// the call) — drained by Layer::make_perimeters_with_neighbors into
    /// Layer::loop_nodes with a rebase (z-direction speed continuity).
    pub loop_nodes_out: Vec<LoopNode>,

    /// R932 LAYER_REGION_PERIMETER_ISLANDS: the pin keeps the walls of each island of a
    /// `make_perimeters` call as ONE collection in `perimeters` (PerimeterGenerator.cpp:1340-1342
    /// `this->loops->append(entities)`, :1837-1838 `this->loops->append(extrusion_coll)`); the live
    /// consumers here read `perimeters.entities` flat (layer.rs, "The islands of `loops` one after the
    /// other"). Each entry is the range of `perimeters.entities` one island collection of the pin holds,
    /// and that collection with its entities moved out (its flags and `loop_node_range`, rebased onto
    /// Layer::loop_nodes as the pin's is). `perimeter_island_collections` rebuilds the pin's structure.
    pub perimeter_islands: Vec<(std::ops::Range<usize>, ExtrusionEntityCollection)>,

    /// Per-entity cooling-node ids for `perimeters.entities` (aligned by
    /// index; -1 = none). Native stores this on the ExtrusionEntity base
    /// (set_cooling_node, ExtrusionEntity.hpp:144); the Rust port keeps a
    /// parallel vec to avoid threading a field through every entity literal.
    /// Consumed by the gated `; COOLING_NODE:` emission (GCode.cpp:5738-5747).
    pub entity_cooling_nodes: Vec<i32>,

    pub lower_slices: Option<Vec<crate::geometry::ExPolygon>>,

    /// Upper layer slices for top-surface detection (set before make_perimeters)
    pub upper_slices: Option<Vec<crate::geometry::ExPolygon>>,

    /// Owning PrintRegion, shared via Arc with Print::print_regions and
    /// PrintObjectRegions::all_regions (the very same Arc identity).
    /// C++: `const PrintRegion *m_region;` (Layer.hpp:131), reached through
    /// `LayerRegion::region()` (Layer.hpp:38). Stamped by
    /// PrintObject::wire_layer_hierarchy at sync points; None until wired.
    pub(crate) region: Option<Arc<crate::print_region::PrintRegion>>,

    /// Snapshot of the owning PrintObject's config, shared via Arc.
    /// C++ reaches it via the parent pointers `m_layer->m_object->config()`
    /// (Layer.hpp:130 + Print.hpp:369). Stamped at sync points; None until wired.
    pub(crate) object_config: Option<Arc<crate::print_config::PrintObjectConfig>>,

    /// Snapshot of the owning Print's config, shared via Arc.
    /// C++: `m_layer->m_object->print()->config()` (PrintBase.hpp:632 +
    /// Print.hpp:885). Stamped at sync points; None until wired.
    pub(crate) print_config: Option<Arc<crate::print_config::PrintConfig>>,
}

/// R934 Layer.cpp:284-361 `generate_region_perimeters`, the lambda of `Layer::make_perimeters` that
/// runs a region's walls (BBS multi-diameter adaptive layer height): a region inside a band is extruded on
/// the group top only, at the height of the whole band. `get_layer(i)` stands for
/// `this->object()->get_layer(int(i))`, `layer_count` for `this->object()->layer_count()`, `region_id` for
/// the index of `lm` in the layer's regions. The fill outputs are `lm`'s own members (the
/// `layerms.size() == 1` call, Layer.cpp:366); `perimeter_regions` is not taken (see
/// `make_perimeters_overload`). Nothing live calls it yet: the live Layer::make_perimeters calls
/// `make_perimeters_with` for every region (the `adaptive_merge_patches.empty()` arm, :287-290).
#[allow(clippy::too_many_arguments)]
pub fn generate_region_perimeters<'a>(
    lm: &mut LayerRegion,
    src: &SurfaceCollection,
    region_id: usize,
    layer_count: usize,
    get_layer: &dyn Fn(usize) -> &'a Layer,
    layer_height: f64,
    layer_id: usize,
    print_z: f64,
    patch_cfg: &dyn Fn(&mut crate::perimeter_generator::PerimeterConfig),
) -> Result<()> {
    // Layer.cpp:287-290
    if lm.adaptive_merge_patches.is_empty() {
        return lm.make_perimeters_with(src, layer_height, layer_id, print_z, patch_cfg);
    }

    // Layer.cpp:292-302: swallowed by the group top, so nothing is extruded here. The fill surfaces still
    // pass through shell discovery (fill_only).
    let group_top_patch = match lm.adaptive_merge_patches.iter().find(|p| p.group_top) {
        Some(p) => p.clone(),
        None => {
            let mut args = MakePerimetersArgs::plain();
            args.fill_only = true;
            return lm.make_perimeters_overload(src, layer_height, layer_id, print_z, &args, patch_cfg);
        }
    };

    // Layer.cpp:304-323: the footprint the band above will actually extrude.
    let mut upper_slices: Option<ExPolygons> = None;
    let top_idx = group_top_patch.top_idx;
    if top_idx + 1 < layer_count {
        let upper_layer = get_layer(top_idx + 1);
        let mut upper_footprint: Polygons = Vec::new();
        for upper_region in &upper_layer.regions {
            if upper_region.adaptive_merge_patches.is_empty() {
                upper_footprint.extend(crate::surface::to_polygons(&upper_region.slices.surfaces));
            } else {
                for upper_patch in &upper_region.adaptive_merge_patches {
                    // polygons_append(dst, to_polygons(ExPolygons)): contour, then holes.
                    for ex in upper_patch.area() {
                        upper_footprint.push(ex.contour.clone());
                        upper_footprint.extend(ex.holes.iter().cloned());
                    }
                }
            }
        }
        // union_ex(const Polygons&) with pftNonZero (ClipperUtils.hpp)
        upper_slices = Some(crate::clipper_utils::union_ex_clib(&upper_footprint, 1));
    }

    // Layer.cpp:325-356
    let mut generated = false;
    let patches = lm.adaptive_merge_patches.clone();
    for patch in &patches {
        if !patch.group_top || patch.area().is_empty() {
            continue;
        }
        let mut core_slices = SurfaceCollection::new();
        let patch_source: &SurfaceCollection =
            if patch.bottom_idx == 0 { &get_layer(0).regions[region_id].slices } else { src };
        for s in &patch_source.surfaces {
            for e in crate::clipper_utils::intersection_clib(std::slice::from_ref(&s.expolygon), patch.area()) {
                // Surface(const Surface &other, const ExPolygon &_expolygon)
                core_slices.surfaces.push(Surface { expolygon: e, ..s.clone() });
            }
        }
        if core_slices.surfaces.is_empty() {
            continue;
        }
        // Layer.cpp:340-351: a suppressed orphan below is sliced but not extruded, so the walls over it overhang.
        let lower_slices = if patch.bottom_idx > 0 {
            let lower = get_layer(patch.bottom_idx - 1);
            let regions: Vec<crate::adaptive_layer_merge::AdaptiveSuppressedOrphanRegion<'_>> = lower
                .regions
                .iter()
                .map(|r| crate::adaptive_layer_merge::AdaptiveSuppressedOrphanRegion {
                    adaptive_merge_patches: &r.adaptive_merge_patches,
                    slices: &r.slices.surfaces,
                })
                .collect();
            let dropped = crate::adaptive_layer_merge::adaptive_suppressed_orphan_area(&regions);
            if dropped.is_empty() {
                Some(lower.lslices.clone())
            } else {
                Some(crate::clipper_utils::difference_clib(&lower.lslices, &dropped))
            }
        } else {
            None
        };
        let args = MakePerimetersArgs {
            perimeter_layer_height: patch.merged_height,
            clear_outputs: !generated,
            lower_slices_override: lower_slices,
            upper_slices_override: upper_slices.clone(),
            fill_only: false,
            starts_at_object_base: patch.bottom_idx == 0,
        };
        lm.make_perimeters_overload(&core_slices, layer_height, layer_id, print_z, &args, patch_cfg)?;
        generated = true;
    }
    // Layer.cpp:357-360
    if !generated {
        lm.perimeters.entities.clear();
        lm.perimeter_islands.clear();
        lm.thin_fills.entities.clear();
    }
    Ok(())
}

/// R934 Layer.cpp:233-238 `has_first_band_source`: a group top whose band starts at object layer 0 and
/// keeps an area, which makes `Layer::make_perimeters` run a region with no slices of its own (:239).
pub fn has_first_band_source(layerm: &LayerRegion) -> bool {
    layerm.adaptive_merge_patches.iter().any(|patch| patch.group_top && patch.bottom_idx == 0 && !patch.area().is_empty())
}

/// R934: the arguments of the pin's `LayerRegion::make_perimeters` overload beyond the slices and the
/// fill outputs (LayerRegion.cpp:131; the defaults of Layer.hpp's declaration are `plain()`).
#[derive(Debug, Clone)]
pub struct MakePerimetersArgs {
    /// `perimeter_layer_height`: 0 means the physical layer height (LayerRegion.cpp:146).
    pub perimeter_layer_height: f64,
    /// `clear_outputs` (LayerRegion.cpp:133)
    pub clear_outputs: bool,
    /// `lower_slices_override` (LayerRegion.cpp:182-183)
    pub lower_slices_override: Option<Vec<crate::geometry::ExPolygon>>,
    /// `upper_slices_override` (LayerRegion.cpp:187-188)
    pub upper_slices_override: Option<Vec<crate::geometry::ExPolygon>>,
    /// `fill_only` (LayerRegion.cpp:133, 167-176)
    pub fill_only: bool,
    /// `starts_at_object_base` (LayerRegion.cpp:147)
    pub starts_at_object_base: bool,
}

impl MakePerimetersArgs {
    /// `perimeter_layer_height = 0., clear_outputs = true, lower_slices_override = nullptr,
    /// upper_slices_override = nullptr, fill_only = false, starts_at_object_base = false`.
    pub fn plain() -> Self {
        Self {
            perimeter_layer_height: 0.,
            clear_outputs: true,
            lower_slices_override: None,
            upper_slices_override: None,
            fill_only: false,
            starts_at_object_base: false,
        }
    }
}

impl LayerRegion {
    /// Create a new LayerRegion
    /// Layer.cpp:15
    pub fn new(layer_id: usize, region_id: usize) -> Self {
        Self {
            layer_id,
            region_id,
            slices: SurfaceCollection::new(),
            raw_slices: SurfaceCollection::new(),
            raw_counter_circle_compensation: Polygons::new(),
            raw_holes_circle_compensation: Polygons::new(),
            thin_fills: ExtrusionEntityCollection::new(),
            loop_nodes_out: Vec::new(),
            perimeter_islands: Vec::new(),
            entity_cooling_nodes: Vec::new(),
            fill_expolygons: ExPolygons::new(),
            fill_surfaces: SurfaceCollection::new(),
            fill_no_overlap_expolygons: ExPolygons::new(),
            unsupported_bridge_edges: Polylines::new(),
            adaptive_merge_patches: Vec::new(),
            perimeters: ExtrusionEntityCollection::new(),
            fills: ExtrusionEntityCollection::new(),
            lower_slices: None,
            upper_slices: None,
            region: None,
            object_config: None,
            print_config: None,
        }
    }

    /// R932: `perimeters` as the pin's LayerRegion holds it: one collection per island of the last
    /// `make_perimeters` call (PerimeterGenerator.cpp:1340-1342, 1837-1838), the walls that belong to no
    /// recorded island at the top level where they are. With no island recorded (the gate
    /// LAYER_REGION_PERIMETER_ISLANDS off, or the walls of the body PG_CLASSIC_PIN / PG_ARACHNE_PIN
    /// replace) this is `perimeters` as stored.
    pub fn perimeter_island_collections(&self) -> ExtrusionEntityCollection {
        let mut out = ExtrusionEntityCollection::new();
        out.no_sort = self.perimeters.no_sort;
        out.is_reverse = self.perimeters.is_reverse;
        out.loop_node_range = self.perimeters.loop_node_range;
        let mut islands = self.perimeter_islands.iter().peekable();
        let mut i = 0usize;
        while i < self.perimeters.entities.len() {
            match islands.peek() {
                Some((range, _)) if range.start == i => {
                    let (range, shell) = islands.next().unwrap();
                    let mut coll = shell.clone();
                    coll.entities = self.perimeters.entities[range.clone()].to_vec();
                    out.entities.push(crate::extrusion_entity::ExtrusionEntityType::Collection(Box::new(coll)));
                    i = range.end.max(i + 1);
                }
                _ => {
                    out.entities.push(self.perimeters.entities[i].clone());
                    i += 1;
                }
            }
        }
        out
    }

    /// Get layer ID
    /// Layer.hpp:78
    pub fn layer_id(&self) -> usize {
        self.layer_id
    }

    /// Get region ID
    /// Layer.hpp:79
    pub fn region_id(&self) -> usize {
        self.region_id
    }

    /// Get the owning PrintRegion.
    /// C++: `const PrintRegion& region() const { return *m_region; }` (Layer.hpp:38)
    ///
    /// Fails fast when the config hierarchy has not been stamped yet —
    /// support layers carry no regions, so only true printing regions reach
    /// this accessor.
    pub fn region(&self) -> &crate::print_region::PrintRegion {
        self.region
            .as_deref()
            .expect("config hierarchy not wired — call wire_layer_hierarchy")
    }

    /// Get slices
    /// Layer.hpp:81
    pub fn get_slices(&self) -> &SurfaceCollection {
        &self.slices
    }

    /// Calculate flow for a given role.
    /// LayerRegion.cpp:21-29
    /// C++: Flow LayerRegion::flow(FlowRole role) const
    ///          { return this->flow(role, m_layer->height); }
    /// C++: Flow LayerRegion::flow(FlowRole role, double layer_height) const
    ///
    /// This crate's LayerRegion has no Layer back-pointer, so `layer_height`
    /// (C++ `m_layer->height`) is threaded by the caller and the two C++
    /// overloads collapse into this single method.
    pub fn flow(&self, role: FlowRole, layer_height: f64) -> Result<Flow> {
        // LayerRegion.cpp:28
        // C++: return m_region->flow(*m_layer->object(), role, layer_height, m_layer->id() == 0);
        // The object/print configs are reached through the Arc snapshots
        // stamped by wire_layer_hierarchy instead of the C++ parent pointers;
        // `m_layer->id()` is mirrored by this->layer_id (stamped at creation).
        let print_config = self
            .print_config
            .as_deref()
            .expect("config hierarchy not wired — call wire_layer_hierarchy");
        let object_config = self
            .object_config
            .as_deref()
            .expect("config hierarchy not wired — call wire_layer_hierarchy");
        crate::print_region::flow_from_configs(
            role,
            layer_height,
            self.layer_id == 0,
            print_config.initial_layer_line_width,
            object_config.line_width,
            print_config.nozzle_diameter,
            self.region().config(),
        )
        .map_err(crate::Error::Config)
    }

    /// Calculate bridging flow.
    /// LayerRegion.cpp:31-46
    /// C++: Flow LayerRegion::bridging_flow(FlowRole role, bool thick_bridge) const
    ///
    /// The non-thick branch calls C++ `this->flow(role)` which reads
    /// `m_layer->height`; `layer_height` is threaded by the caller for the
    /// same reason as in [`LayerRegion::flow`].
    pub fn bridging_flow(&self, role: FlowRole, thick_bridge: bool, layer_height: f64) -> Result<Flow> {
        // LayerRegion.cpp:33
        // C++: const PrintRegion &region = this->region();
        let region = self.region();
        // LayerRegion.cpp:34
        // C++: const PrintRegionConfig &region_config = region.config();
        let region_config = region.config();
        // LayerRegion.cpp:35
        // C++: const PrintObject &print_object = *this->layer()->object();
        // (only used to reach print()->config(); read off the stored Arc below)
        if thick_bridge {
            // The old Slic3r way (different from all other slicers): Use rounded extrusions.
            // Get the configured nozzle_diameter for the extruder associated to the flow role requested.
            // Here this->extruder(role) - 1 may underflow to MAX_INT, but then the get_at() will follback to zero'th element, so everything is all right.
            // LayerRegion.cpp:40
            // C++: auto nozzle_diameter = float(print_object.print()->config().nozzle_diameter.get_at(region.extruder(role) - 1));
            // This crate's PrintConfig nozzle_diameter is a single-extruder
            // scalar, so get_at(extruder - 1) collapses to a direct read;
            // region.extruder(role) is still evaluated for its Unknown-role
            // error semantics.
            let _ = region.extruder(role).map_err(crate::Error::Config)?;
            let nozzle_diameter = self
                .print_config
                .as_deref()
                .expect("config hierarchy not wired — call wire_layer_hierarchy")
                .nozzle_diameter;
            // Applies default bridge spacing.
            // LayerRegion.cpp:42
            // C++: return Flow::bridging_flow(float(sqrt(region_config.bridge_flow)) * nozzle_diameter, nozzle_diameter);
            // LAYER_REGION_FLOW_PIN (faithful gate, R923): the product is one
            // of FLOATS at the pin: float(sqrt(bridge_flow)) times the float
            // nozzle_diameter_for_filament returns (PrintConfig.cpp:454), and
            // Flow::bridging_flow takes floats (Flow.hpp:105). `=0` keeps the
            // earlier arm, which multiplied doubles.
            if crate::faithful_gate("LAYER_REGION_FLOW_PIN") {
                let nozzle_diameter_f = nozzle_diameter as f32;
                let dmr = (region_config.bridge_flow_ratio.sqrt() as f32) * nozzle_diameter_f;
                return Ok(Flow::bridging_flow(dmr as f64, nozzle_diameter_f as f64));
            }
            Ok(Flow::bridging_flow(
                region_config.bridge_flow_ratio.sqrt() * nozzle_diameter,
                nozzle_diameter,
            ))
        } else {
            // The same way as other slicers: Use normal extrusions. Apply bridge_flow while maintaining the original spacing.
            // LayerRegion.cpp:45
            // C++: return this->flow(role).with_flow_ratio(region_config.bridge_flow);
            self.flow(role, layer_height)?
                .with_flow_ratio(region_config.bridge_flow_ratio)
                .map_err(|e| e.into())
        }
    }

    // Placeholder methods

    /// Clip slices to fill surfaces
    /// LayerRegion.cpp:50-66
    /// C++: void LayerRegion::slices_to_fill_surfaces_clipped()
    pub fn slices_to_fill_surfaces_clipped(&mut self) {
        use crate::clipper_utils::intersection_ex;
        use crate::surface::SurfaceType;

        /// LayerRegion.cpp:52-56
        /// C++: std::array<SurfacesPtr, size_t(stCount)> by_surface;
        /// C++: for (Surface &surface : this->slices.surfaces)
        /// C++:     by_surface[size_t(surface.surface_type)].emplace_back(&surface);
        const ST_COUNT: usize = crate::surface::SurfaceType::COUNT; // stCount from C++ (Surface.hpp:29)
        let mut by_surface: Vec<Vec<usize>> = vec![Vec::new(); ST_COUNT];
        for (idx, surface) in self.slices.surfaces.iter().enumerate() {
            let surface_type_idx = surface.surface_type as usize;
            by_surface[surface_type_idx].push(idx);
        }

        /// LayerRegion.cpp:57-58
        /// C++: this->fill_surfaces.surfaces.clear();
        self.fill_surfaces.surfaces.clear();

        /// LayerRegion.cpp:59-65
        /// C++: for (size_t surface_type = 0; surface_type < size_t(stCount); ++ surface_type) {
        /// C++:     const SurfacesPtr &this_surfaces = by_surface[surface_type];
        /// C++:     if (! this_surfaces.empty())
        /// C++:         this->fill_surfaces.append(intersection_ex(this_surfaces, this->fill_expolygons), SurfaceType(surface_type));
        /// C++: }
        for surface_type_idx in 0..ST_COUNT {
            let this_surfaces_indices = &by_surface[surface_type_idx];
            if !this_surfaces_indices.is_empty() {
                let this_surfaces: Vec<crate::surface::Surface> = this_surfaces_indices
                    .iter()
                    .map(|&idx| self.slices.surfaces[idx].clone())
                    .collect();
                let intersected = intersection_ex(&this_surfaces, &self.fill_expolygons);
                self.fill_surfaces
                    .append(intersected, SurfaceType::from_u8(surface_type_idx as u8));
            }
        }

    }

    /// Prepare fill surfaces
    /// LayerRegion.cpp:645-689
    /// C++: void LayerRegion::prepare_fill_surfaces()
    pub fn prepare_fill_surfaces(&mut self) {
        /// LayerRegion.cpp:650-652
        /// C++: Note: in order to make the psPrepareInfill step idempotent, we should never
        /// C++: alter fill_surfaces boundaries on which our idempotency relies since that's
        /// C++: the only meaningful information returned by psPerimeters.

        // LAYER_REGION_PREPARE_PIN (faithful gate, R923): the pin's
        // prepare_fill_surfaces (LayerRegion.cpp:913-955) does not clip: the
        // clip is detect_surfaces_type's last step (PrintObject.cpp:1618-1643)
        // and fill_surfaces come in as the caller left them. `=0` keeps the
        // earlier arm, which clipped again here and truncated min_area.
        let prepare_pin = crate::faithful_gate("LAYER_REGION_PREPARE_PIN");
        if !prepare_pin {
            /// First clip slices to fill boundaries
            /// LayerRegion.cpp:50-66 (called before this function in the flow)
            self.slices_to_fill_surfaces_clipped();
        }

        /// LayerRegion.cpp:657
        /// C++: bool spiral_mode = this->layer()->object()->print()->config().spiral_mode;
        // Reached through the LayerRegion's own print-config Arc (stamped by
        // wire_layer_hierarchy); field mapping: spiral_mode -> spiral_vase.
        let spiral_mode = self
            .print_config
            .as_deref()
            .expect("config hierarchy not wired — call wire_layer_hierarchy")
            .spiral_vase;

        /// LayerRegion.cpp:658-672 (commented out with #if 0 in C++)
        /// The top/bottom demotion logic is disabled in BambuStudio

        /// LayerRegion.cpp:675-682
        /// C++: turn too small internal regions into solid regions according to the user setting
        /// C++: if (! spiral_mode && this->region().config().sparse_infill_density.value > 0) {
        /// C++:     double min_area = scale_(scale_(this->region().config().minimum_sparse_infill_area.value));
        /// C++:     for (Surface &surface : this->fill_surfaces.surfaces)
        /// C++:         if (surface.surface_type == stInternal && surface.area() <= min_area)
        /// C++:             surface.surface_type = stInternalSolid;
        /// C++: }
        // Field mapping: sparse_infill_density -> fill_density on the wired
        // region Arc. NOTE: this crate keeps minimum_sparse_infill_area on
        // PrintObjectConfig (print_config.rs) rather than PrintRegionConfig,
        // so it is read through the object-config Arc — same resolved value.
        if !spiral_mode && self.region().config().fill_density > 0.0 {
            let minimum_sparse_infill_area = self
                .object_config
                .as_deref()
                .expect("config hierarchy not wired — call wire_layer_hierarchy")
                .minimum_sparse_infill_area;
            let scale_factor = crate::SCALING_FACTOR as f64;
            // LayerRegion.cpp:943 — double min_area = scale_(scale_(minimum_sparse_infill_area.value));
            // scale_(v) is v / SCALING_FACTOR, a division by 0.00001
            // (libslic3r.h:58-84), and min_area stays a double.
            let min_area: f64 = if prepare_pin {
                (minimum_sparse_infill_area / 0.00001) / 0.00001
            } else {
                (minimum_sparse_infill_area * scale_factor * scale_factor) as i64 as f64
            };

            for surface in &mut self.fill_surfaces.surfaces {
                if surface.surface_type == crate::surface::SurfaceType::Internal
                    && surface.area() <= min_area
                {
                    surface.surface_type = crate::surface::SurfaceType::InternalSolid;
                }
            }
        }
    }

    /// Generate perimeters for this region
    /// LayerRegion.cpp:131-172
    /// C++: void LayerRegion::make_perimeters(const SurfaceCollection &slices, const PerimeterRegions &perimeter_regions, SurfaceCollection *fill_surfaces, ExPolygons *fill_no_overlap, std::vector<LoopNode> &loop_nodes)
    pub fn make_perimeters(
        &mut self,
        surface_fill: &SurfaceCollection,
        layer_height: f64,
        layer_id: usize,
        print_z: f64,
    ) -> Result<()> {
        self.make_perimeters_with(surface_fill, layer_height, layer_id, print_z, &|_| {})
    }

    /// `make_perimeters` (LayerRegion.cpp:131-206) with the `PerimeterConfig` handed to `patch` before the
    /// generator is made from it. The configs of this crate's layers do not hold every option the pin's
    /// PerimeterGenerator reads (PerimeterGenerator.cpp:872, 935, 938, 1131, 1141, 1275, 1378); the
    /// comparison with the pin (tests/perimeter_generator_native_parity.rs) sets those through `patch`
    /// and runs the same body the slicing path runs. `make_perimeters` patches nothing.
    pub fn make_perimeters_with(
        &mut self,
        surface_fill: &SurfaceCollection,
        layer_height: f64,
        layer_id: usize,
        print_z: f64,
        patch: &dyn Fn(&mut crate::perimeter_generator::PerimeterConfig),
    ) -> Result<()> {
        self.make_perimeters_overload(surface_fill, layer_height, layer_id, print_z, &MakePerimetersArgs::plain(), patch)
    }

    /// R934: the pin's full overload, LayerRegion.cpp:131-206 `make_perimeters(slices, perimeter_regions,
    /// fill_surfaces, fill_no_overlap, loop_nodes, perimeter_layer_height, clear_outputs,
    /// lower_slices_override, upper_slices_override, fill_only, starts_at_object_base)`. The outputs
    /// `fill_surfaces` / `fill_no_overlap` / `loop_nodes` are this region's members as in
    /// `make_perimeters_with`; `perimeter_regions` is not taken yet (consumed by apply_fuzzy_skin only,
    /// PerimeterGenerator.cpp:348, 661). With `MakePerimetersArgs::plain()` this is
    /// `make_perimeters_with` byte for byte.
    pub fn make_perimeters_overload(
        &mut self,
        surface_fill: &SurfaceCollection,
        layer_height: f64,
        layer_id: usize,
        print_z: f64,
        args: &MakePerimetersArgs,
        patch: &dyn Fn(&mut crate::perimeter_generator::PerimeterConfig),
    ) -> Result<()> {
        // LayerRegion.cpp:133-140: `fill_only` sends the walls, thin fills and loop nodes the generator
        // makes to the discarded collections (:167-176); without `clear_outputs` the new walls go after
        // the ones already there.
        let keep = !args.clear_outputs || args.fill_only;
        let saved = if keep {
            Some((
                std::mem::take(&mut self.perimeters),
                std::mem::take(&mut self.perimeter_islands),
                std::mem::take(&mut self.thin_fills),
                std::mem::take(&mut self.loop_nodes_out),
            ))
        } else {
            None
        };
        let res = self.make_perimeters_body(surface_fill, layer_height, layer_id, print_z, args, patch);
        if let Some((perimeters, islands, thin_fills, loop_nodes)) = saved {
            if args.fill_only {
                // discarded_perimeters / discarded_thin_fills / discarded_loop_nodes (:137-139)
                self.perimeters = perimeters;
                self.perimeter_islands = islands;
                self.thin_fills = thin_fills;
                self.loop_nodes_out = loop_nodes;
            } else {
                // `this->loops->append(...)` onto the walls kept (PerimeterGenerator.cpp:1340-1342,
                // 1837-1838); the loop of a node is the index among all walls, as the body stores it.
                let base = perimeters.entities.len();
                let new_perimeters = std::mem::replace(&mut self.perimeters, perimeters);
                let new_islands = std::mem::replace(&mut self.perimeter_islands, islands);
                let mut new_nodes = std::mem::replace(&mut self.loop_nodes_out, loop_nodes);
                for n in new_nodes.iter_mut() {
                    n.loop_id += base;
                }
                self.perimeters.entities.extend(new_perimeters.entities);
                for (range, coll) in new_islands {
                    self.perimeter_islands.push((range.start + base..range.end + base, coll));
                }
                self.loop_nodes_out.extend(new_nodes);
                self.perimeters.loop_node_range = (0, self.loop_nodes_out.len());
                let new_thin = std::mem::replace(&mut self.thin_fills, thin_fills);
                self.thin_fills.entities.extend(new_thin.entities);
            }
        }
        res
    }

    fn make_perimeters_body(
        &mut self,
        surface_fill: &SurfaceCollection,
        layer_height: f64,
        layer_id: usize,
        print_z: f64,
        args: &MakePerimetersArgs,
        patch: &dyn Fn(&mut crate::perimeter_generator::PerimeterConfig),
    ) -> Result<()> {
        use crate::perimeter_generator::{PerimeterConfig, PerimeterGenerator};

        // LayerRegion.cpp:133-134
        // C++: this->perimeters.clear();
        // C++: this->thin_fills.clear();
        self.perimeters.entities.clear();
        self.perimeter_islands.clear();
        self.thin_fills.entities.clear();

        // LayerRegion.cpp:136-138
        // C++: const PrintConfig &print_config = this->layer()->object()->print()->config();
        // C++: const PrintRegionConfig &region_config = this->region().config();
        // C++: const PrintObjectConfig& object_config = this->layer()->object()->config();
        // All three are reached through the Arc snapshots stamped by
        // wire_layer_hierarchy; the Arcs are cloned into locals up front so no
        // borrow of self is held while building PerimeterConfig / mutating
        // self below. layer_id/print_z replace the C++ Layer back-pointer
        // reads this->layer()->id() / this->layer()->print_z.
        let print_config = self
            .print_config
            .clone()
            .expect("config hierarchy not wired — call wire_layer_hierarchy");
        let region = self
            .region
            .clone()
            .expect("config hierarchy not wired — call wire_layer_hierarchy");
        let config = region.config();
        let object_config = self
            .object_config
            .clone()
            .expect("config hierarchy not wired — call wire_layer_hierarchy");

        // LayerRegion.cpp:139-148
        // C++: // This needs to be in sync with PrintObject::_slice() slicing_mode_normal_below_layer!
        // C++: bool spiral_mode = print_config.spiral_mode &&
        // C++:     //FIXME account for raft layers.
        // C++:     (this->layer()->id() >= size_t(region_config.bottom_shell_layers.value) &&
        // C++:      this->layer()->print_z >= region_config.bottom_shell_thickness - EPSILON);
        // Rust field mapping: spiral_mode -> PrintConfig::spiral_vase,
        // bottom_shell_layers -> PrintRegionConfig::bottom_solid_layers,
        // bottom_shell_thickness -> PrintRegionConfig::bottom_solid_min_thickness.
        let spiral_mode = print_config.spiral_vase
            && (layer_id >= config.bottom_solid_layers as usize
                && print_z >= config.bottom_solid_min_thickness - crate::libslic3r::EPSILON);
        // C++ feeds spiral_mode into the PerimeterGenerator ctor and the
        // arachne/classic dispatch (LayerRegion.cpp:150-179). The Rust
        // PerimeterConfig does not carry a spiral field yet, so the gate is
        // computed faithfully here but not yet consumed downstream.
        let _ = spiral_mode;

        // LayerRegion.cpp:150-172
        // C++: PerimeterGenerator g(
        // C++:     &slices,
        // C++:     this->layer()->height,
        // C++:     this->flow(frPerimeter),
        // C++:     &region_config,
        // C++:     &this->layer()->object()->config(),
        // C++:     &print_config,
        // C++:     spiral_mode,
        // C++:     &this->perimeters,
        // C++:     &this->thin_fills,
        // C++:     fill_surfaces,
        // C++:     fill_no_overlap,
        // C++:     &loop_nodes
        // C++: );

        // Get flows for different perimeter types
        // C++: this->flow(frPerimeter) (LayerRegion.cpp:153) — the LayerRegion
        // reads region/object/print configs off its stored Arcs; layer_height
        // stands in for the missing m_layer->height back-pointer read.
        // R934 LayerRegion.cpp:144-151: `wall_layer_height` (the merged height of a multi-diameter core,
        // else the physical one) and `first_object_layer = this->layer()->id() == 0 || starts_at_object_base`
        // feed every flow of the walls: `wall_flow(role) = region().flow(object, role, wall_layer_height,
        // first_object_layer)` (PrintRegion::flow; `this->flow(role)` is the same call with the layer's
        // height and `id() == 0`, LayerRegion.cpp:28, so `MakePerimetersArgs::plain()` keeps the bytes).
        let wall_layer_height = if args.perimeter_layer_height > 0. { args.perimeter_layer_height } else { layer_height };
        let first_object_layer = self.layer_id == 0 || args.starts_at_object_base;
        let wall_flow = |role: FlowRole| -> Result<Flow> {
            crate::print_region::flow_from_configs(
                role,
                wall_layer_height,
                first_object_layer,
                print_config.initial_layer_line_width,
                object_config.line_width,
                print_config.nozzle_diameter,
                config,
            )
            .map_err(crate::Error::Config)
        };
        let perimeter_flow = wall_flow(FlowRole::Perimeter)?;
        let external_perimeter_flow = wall_flow(FlowRole::ExternalPerimeter)?;

        if crate::probe_enabled("MPPROBE") {
            // R555: mirror of [CPP-MPPROBE] -- what make_perimeters receives.
            let h: usize = surface_fill
                .surfaces
                .iter()
                .map(|s| s.expolygon.holes.len())
                .sum();
            let p: usize = surface_fill
                .surfaces
                .iter()
                .map(|s| {
                    s.expolygon.contour.points.len()
                        + s.expolygon.holes.iter().map(|x| x.points.len()).sum::<usize>()
                })
                .sum();
            mpprobe(layer_id as i32, -1, surface_fill.surfaces.len(), h, p);
        }
        if crate::probe_enabled("WTPCFG") {
            use std::sync::atomic::{AtomicBool, Ordering::Relaxed};
            static ONCE: AtomicBool = AtomicBool::new(false);
            if !ONCE.swap(true, Relaxed) {
                eprintln!(
                    "[WTPCFG] arachne_min_bead_width={} arachne_min_feature_size={} \
                     arachne_wall_transition_length={} wall_transition_angle={} \
                     wall_transition_filter_deviation={} wall_distribution_count={} \
                     nozzle_diameter={}",
                    object_config.arachne_min_bead_width,
                    object_config.arachne_min_feature_size,
                    object_config.arachne_wall_transition_length,
                    object_config.wall_transition_angle,
                    object_config.wall_transition_filter_deviation,
                    object_config.wall_distribution_count,
                    print_config.nozzle_diameter,
                );
            }
        }

        // Build PerimeterConfig from PrintRegionConfig
        let mut perimeter_config = PerimeterConfig {
            // PerimeterGenerator.cpp:1536-1553 -- raw option values; the
            // arithmetic lives in perimeter_generator at the C++ line.
            min_nozzle_diameter: print_config.nozzle_diameter,
            arachne_min_feature_size: object_config.arachne_min_feature_size,
            arachne_min_bead_width: object_config.arachne_min_bead_width,
            arachne_wall_transition_length: object_config.arachne_wall_transition_length,
            wall_transition_angle: object_config.wall_transition_angle,
            wall_transition_filter_deviation: object_config.wall_transition_filter_deviation,
            wall_distribution_count: object_config.wall_distribution_count,
            perimeter_count: config.perimeters as usize,
            perimeter_extrusion_width: perimeter_flow.width(),
            external_perimeter_extrusion_width: external_perimeter_flow.width(),
            smaller_external_perimeter_width: external_perimeter_flow.width() * 0.9, // TODO: Get from config
            perimeter_spacing: perimeter_flow.spacing(),
            external_perimeter_spacing: external_perimeter_flow.spacing(),
            smaller_external_perimeter_spacing: external_perimeter_flow.spacing() * 0.9,
            external_to_internal_spacing: (external_perimeter_flow.spacing()
                + perimeter_flow.spacing())
                / 2.0,
            // LayerRegion.cpp:159 `wall_layer_height`
            layer_height: wall_layer_height,
            perimeter_flow: perimeter_flow.clone(),
            ext_perimeter_flow: external_perimeter_flow.clone(),
            smaller_ext_perimeter_flow: external_perimeter_flow.clone(), // TODO: Create proper smaller flow
            join_type: crate::clipper_utils::OffsetJoinType::Miter,
            gap_fill_threshold: if config.gap_fill_enabled {
                config.gap_fill_speed.max(1.0)
            } else {
                0.0
            },
            // PerimeterGenerator.cpp:1185 — this->config->sparse_infill_density.value.
            // PrintRegionConfig stores the percent option as the fraction fill_density;
            // the consumer only performs the == 0 comparison, which is equivalent.
            sparse_infill_density: config.fill_density,
            detect_thin_wall: config.thin_walls,
            // PerimeterGenerator.cpp:911 m_scaled_resolution = scaled<double>(print_config.resolution).
            // This is fed to ExPolygon::simplify_p(), which calls
            // `geometry::simplify::douglas_peucker` — that variant RE-SCALES the
            // tolerance internally (`tolerance_sq = scale(tolerance)^2`,
            // geometry/simplify.rs), so it expects the UNSCALED value in **mm**.
            // Read from the print config (C++: print_config->resolution).
            // DO NOT pre-scale here: passing scaled units would be squared again and
            // collapse every contour to a point (empty perimeters). NOTE the contrast
            // with `multi_point::douglas_peucker`, which does NOT scale and is the one
            // used by the arc fitter / Polyline::simplify — different unit convention.
            surface_simplify_resolution: print_config.resolution,
            // PerimeterGenerator.cpp:914 uses print_config->enable_arc_fitting to pick the
            // surface simplify resolution factor (0.2x when arc fitting + no fuzzy skin).
            // R810 PG_ARC_FITTING_CFG (faithful ON; `=0` restores the constant):
            // this was hard-wired to true, which is right only for profiles that
            // enable arc fitting. The 02.09.01.70 H2D process profiles turn it off,
            // and the walls are then simplified at the full resolution.
            arc_fitting_enabled: if crate::faithful_gate("PG_ARC_FITTING_CFG") {
                print_config.arc_fitting_enabled
            } else {
                true
            },
            // LayerRegion.cpp:176
            // C++: if (this->layer()->object()->config().wall_generator.value == PerimeterGeneratorType::Arachne && !spiral_mode)
            // C++:     g.process_arachne();
            // C++: else
            // C++:     g.process_classic();
            // The Classic/Arachne dispatch lives on the OBJECT config in C++;
            // mapped here onto PerimeterConfig::wall_generator_mode, which the
            // generator dispatches on internally. (The !spiral_mode gate is
            // not consumed yet — see the spiral_mode note above.)
            wall_generator_mode: match object_config.perimeter_mode {
                crate::print_config::PerimeterMode::Classic => WallGeneratorMode::Classic,
                crate::print_config::PerimeterMode::Arachne => WallGeneratorMode::Arachne,
            },
            fuzzy_skin_mode: config.fuzzy_skin_mode,
            fuzzy_skin_thickness: config.fuzzy_skin_thickness,
            fuzzy_skin_point_distance: config.fuzzy_skin_point_distance,
            wall_sequence: config.wall_sequence,
            detect_overhang_wall: config.overhangs,
            // LayerRegion.cpp:180-189: the overrides first; a band anchored at the object base takes no
            // lower slices (`! first_object_layer`; at layer id 0 `lower_slices` is what the caller set,
            // none for an object's first layer).
            lower_slices: if let Some(o) = &args.lower_slices_override {
                Some(o.clone())
            } else if !args.starts_at_object_base {
                self.lower_slices.clone()
            } else {
                None
            },
            upper_slices: if let Some(o) = &args.upper_slices_override { Some(o.clone()) } else { self.upper_slices.clone() },
            // BambuStudio default top_one_wall_type == Alltop (config key only_one_wall_top);
            // gates the top_fills top-surface detection (PerimeterGenerator.cpp:1118).
            top_one_wall: true,
            // PrintConfig.cpp:1288 default top_area_threshold = 200%.
            top_area_threshold: 200.0,
            // PerimeterGenerator.cpp:874 — solid_infill_flow.scaled_spacing()
            // (LayerRegion.cpp:173: g.solid_infill_flow = this->flow(frSolidInfill)).
            solid_infill_spacing: wall_flow(FlowRole::SolidInfill)
                .map(|f| f.spacing())
                .unwrap_or_else(|_| perimeter_flow.spacing()),
            // PerimeterGenerator.cpp:1167 — config->sparse_infill_line_width.value.
            sparse_infill_line_width: config.sparse_infill_line_width,
            // PerimeterGenerator.cpp:1392 — config->infill_wall_overlap (percent -> fraction).
            infill_wall_overlap: object_config.infill_wall_overlap,
            layer_id: layer_id,
            // PrintConfig.cpp:942 — gates outer-wall LoopNode collection
            // (PG.cpp:1074); BBL profiles set 1, native default false.
            z_direction_outwall_speed_continuous: print_config.z_direction_outwall_speed_continuous,
            raft_layers: 0,
            // LayerRegion.cpp:172 — g.overhang_flow = this->bridging_flow(frPerimeter, thick_bridges);
            // Used by detect_bridge_wall for the 100%-overhang (erOverhangPerimeter) walls.
            // R934 LayerRegion.cpp:196-198: a merged overhang wall takes the merged height,
            // `wall_flow(frPerimeter).with_flow_ratio(region_config.bridge_flow)`.
            overhang_flow: if wall_layer_height > layer_height + crate::libslic3r::EPSILON {
                wall_flow(FlowRole::Perimeter).ok().and_then(|f| f.with_flow_ratio(config.bridge_flow_ratio).ok())
            } else {
                self.bridging_flow(FlowRole::Perimeter, object_config.thick_bridges, layer_height).ok()
            },
            // What `PerimeterGenerator::process_classic` of the pin reads beside the above (R921).
            // `precise_outer_wall` and `alternate_extra_wall` are not among the options this crate's
            // region config holds: the defaults of PrintConfig.cpp.
            precise_outer_wall: false,
            alternate_extra_wall: false,
            // PerimeterGenerator.cpp:938, 1131 `object_config->top_one_wall_type`
            top_one_wall_type: object_config.top_one_wall_type,
            // PerimeterGenerator.cpp:938 `object_config->only_one_wall_first_layer`
            only_one_wall_first_layer: object_config.only_one_wall_first_layer,
            // LayerRegion.cpp:148, 191 `first_object_layer = this->layer()->id() == 0 || starts_at_object_base`
            starts_at_object_base: layer_id == 0 || args.starts_at_object_base,
            // PerimeterGenerator.cpp:1275-1276
            brim_outer_only: object_config.brim_type == crate::print_config::BrimType::OuterOnly && object_config.brim_width > 0.,
            // PerimeterGenerator.cpp:895 `config->gap_infill_speed`
            gap_infill_speed: config.gap_fill_speed,
            // PerimeterGenerator.cpp:1378 `config->filter_out_gap_fill`
            filter_out_gap_fill: config.filter_out_gap_fill,
            // LayerRegion.cpp:153-156, 168
            spiral_vase: spiral_mode,
            // LayerRegion.cpp:199 `g.solid_infill_flow = wall_flow(frSolidInfill);`
            solid_infill_flow: wall_flow(FlowRole::SolidInfill).ok(),
            // PerimeterGenerator.cpp:1411 `config->infill_wall_overlap`: the region config of this crate
            // holds the fraction.
            infill_wall_overlap_percent: object_config.infill_wall_overlap * 100.,
            // PerimeterGenerator.cpp:268-276 `is_enable_overhang_speed`: one extruder, no filament override.
            enable_overhang_speed: object_config.enable_overhang_speed,
            // What `PerimeterGenerator::process_arachne` and `process_no_bridge` of the pin read beside the
            // above (R922). PerimeterGenerator.cpp:652 `config->outer_wall_line_width`
            outer_wall_line_width: config.outer_wall_line_width,
            // PerimeterGenerator.cpp:1956 `config->counterbore_hole_bridging`: not among the options this
            // crate's region config holds; the default of PrintConfig.cpp.
            counterbore_hole_bridging: crate::print_config::enums::CounterboreHoleBridgingOption::ChbNone,
            // PerimeterGenerator.cpp:1982 `config->bridge_angle`
            bridge_angle: config.bridge_angle,
            // LayerRegion.cpp:193 `g.slice_z = this->layer()->slice_z`: read by the fuzzy skin only, whose
            // port here takes no z.
            slice_z: 0.,
        };

        patch(&mut perimeter_config);

        // Create PerimeterGenerator
        let generator = PerimeterGenerator::new(perimeter_config);

        // Extract ExPolygons from surface_fill for processing
        let slices: Vec<crate::ExPolygon> = surface_fill
            .surfaces
            .iter()
            .map(|s| s.expolygon.clone())
            .collect();

        // LayerRegion.cpp:202-205
        // C++: if (this->layer()->object()->config().wall_generator.value == PerimeterGeneratorType::Arachne && !spiral_mode)
        // C++:     g.process_arachne();
        // C++: else
        // C++:     g.process_classic();
        // R921 PG_CLASSIC_PIN (on unless =0): the classic walls by the port of the pin's
        // `process_classic` (PerimeterGenerator.cpp:859-1452), which is compared with the pin over whole
        // layers (tests/perimeter_generator_native_parity.rs). It hands back all of `loops`, `gap_fill`,
        // `fill_surfaces`, `fill_no_overlap` and `loop_nodes`; the gap fill block further down belongs to
        // the body it replaces (`PG_CLASSIC_PIN=0`: `generate` and `process_classic_legacy`).
        let classic_pin = crate::faithful_gate("PG_CLASSIC_PIN")
            && (object_config.perimeter_mode == crate::print_config::PerimeterMode::Classic || spiral_mode);
        // R922 PG_ARACHNE_PIN (on unless =0): the Arachne walls by the port of the pin's `process_arachne`
        // (PerimeterGenerator.cpp:1489-1848), compared with the pin over whole layers by the same test.
        // `PG_ARACHNE_PIN=0` is the body it replaces (`generate` and `process_arachne_legacy`).
        let arachne_pin = crate::faithful_gate("PG_ARACHNE_PIN")
            && object_config.perimeter_mode == crate::print_config::PerimeterMode::Arachne
            && !spiral_mode;
        if classic_pin || arachne_pin {
            let result = if arachne_pin { generator.process_arachne(&surface_fill.surfaces) } else { generator.process_classic(&surface_fill.surfaces) };
            // The islands of `loops` one after the other, as the body this replaces stores them: the
            // loop of a node is then the index among the walls of all islands.
            let mut nodes = result.loop_nodes;
            // R932 LAYER_REGION_PERIMETER_ISLANDS (on unless =0): record which walls form one island
            // collection of the pin (PerimeterGenerator.cpp:1340-1342, 1837-1838); nothing the export
            // reads changes.
            let record_islands = crate::faithful_gate("LAYER_REGION_PERIMETER_ISLANDS");
            for island in result.entities.entities {
                let base = self.perimeters.entities.len();
                match island {
                    crate::extrusion_entity::ExtrusionEntityType::Collection(mut coll) => {
                        for n in nodes[coll.loop_node_range.0..coll.loop_node_range.1].iter_mut() {
                            n.loop_id += base;
                        }
                        let island_entities = std::mem::take(&mut coll.entities);
                        let end = base + island_entities.len();
                        self.perimeters.entities.extend(island_entities);
                        if record_islands {
                            self.perimeter_islands.push((base..end, *coll));
                        }
                    }
                    other => self.perimeters.entities.push(other),
                }
            }
            self.perimeters.loop_node_range = (0, nodes.len());
            self.loop_nodes_out = nodes;
            for infill_expoly in result.infill_area {
                let fill_surface = Surface {
                    expolygon: infill_expoly,
                    surface_type: SurfaceType::Internal,
                    thickness: layer_height,
                    thickness_layers: 1,
                    bridge_angle: None,
                    extra_perimeters: 0,
                };
                self.fill_surfaces.surfaces.push(fill_surface.clone());
                self.fill_expolygons.push(fill_surface.expolygon);
            }
            self.fill_no_overlap_expolygons.extend(result.no_overlap_area);
            self.thin_fills.entities.extend(result.gap_fill.entities);
            return Ok(());
        }
        // Generate perimeters
        let mut result = generator.generate(&slices);
        // R253 (GAPTRIM_PRETAIL): per-surface pre-tail inputs for the
        // native-order gap trim (re-run the fill-boundary tail on trimmed last).
        let tail_inputs = std::mem::take(&mut result.tail_inputs);
        let result = result;

        // LayerRegion.cpp:171
        // C++: The PerimeterGenerator writes directly to this->perimeters via pointer
        // In Rust, we copy from the result

        // Store perimeter entities (ExtrusionLoops)
        // PerimeterGenerator.cpp:1242-1420 - traverse_loops creates these
        self.perimeters = result.entities;
        // z-direction speed continuity: hand the LoopNodes to the Layer.
        self.loop_nodes_out = result.loop_nodes;

        // Store infill area
        let perimeter_top_band = result.top_band.clone();
        for infill_expoly in result.infill_area {
            let fill_surface = Surface {
                expolygon: infill_expoly,
                surface_type: SurfaceType::Internal,
                thickness: layer_height,
                thickness_layers: 1,
                bridge_angle: None,
                extra_perimeters: 0,
            };
            self.fill_surfaces.surfaces.push(fill_surface.clone());
            self.fill_expolygons.push(fill_surface.expolygon);
        }

        // Store the no-overlap infill area. PerimeterGenerator.cpp:1429
        // (this->fill_no_overlap->insert(... polyWithoutOverlap ...)). Consumed by
        // group_fills -> SurfaceFill::no_overlap_expolygons and the
        // FillMonotonicLineWGapFill top-surface gap-fill in make_fills.
        self.fill_no_overlap_expolygons
            .extend(result.no_overlap_area);

        // Gap fill — faithful port of PerimeterGenerator.cpp:1327-1374.
        // The previous version traced each gap polygon's CONTOUR at full perimeter
        // width, which over-extruded ~4.6x (contour ~2x length, full width vs thin).
        // C++ instead: collapse the gaps to the truly-thin band, run medial_axis to get
        // variable-width centerlines, and emit those via variable_width().
        if crate::env_f64("GFPROBE_LID") == Some(self.layer_id as f64) {
            for ex in &result.gap_fills {
                let mut line = format!("GFRAW lid={} n={}", self.layer_id, ex.contour.points.len());
                for pt in &ex.contour.points {
                    line.push_str(&format!(" {},{}", pt.x(), pt.y()));
                }
                eprintln!("{}", line);
            }
        }
        if !result.gap_fills.is_empty() {
            use crate::clipper_utils::{difference_clib, offset2_clib, OffsetJoinType};
            use crate::extrusion_entity::{ExtrusionEntityType, ExtrusionRole};
            use crate::perimeter_generator::convert_thin_walls_to_extrusion_paths;

            let perimeter_width = perimeter_flow.width();
            let perimeter_spacing = perimeter_flow.spacing();
            // PerimeterGenerator.cpp:1329-1330 — INSET_OVERLAP_TOLERANCE = 0.4
            // (libslic3r.h:72; the 0.45 here was WRONG, R143). Gated: default keeps
            // the legacy value (byte-locked 147987).
            let min = if crate::faithful_gate("F1_UNION") {
                0.2 * perimeter_width * (1.0 - 0.4)
            } else {
                0.2 * perimeter_width * (1.0 - 0.45)
            };
            let max = 2.0 * perimeter_spacing;
            // ClipperUtils.hpp:29 — ClipperSafetyOffset = 10.f scaled units; with
            // SCALING_FACTOR = 1e-5 that is 10 * 1e-5 = 1e-4 mm (clipper_utils operates
            // in mm). The collection side (perimeter_generator.rs) already uses 0.0001.
            const CLIPPER_SAFETY_OFFSET: f64 = 0.0001;

            // PerimeterGenerator.cpp:1331-1334 — keep only the band wider than `min`
            // (opening by min/2) and narrower than `max` (subtract the max-opening).
            // C++:
            //   diff_ex(opening_ex(gaps, min/2),
            //           offset2_ex(gaps, -max/2, +(max/2 + ClipperSafetyOffset)))
            // where opening_ex(g, d) == offset2_ex(g, -d, +d, DefaultJoinType=jtMiter,
            // DefaultMiterLimit=3.0) (ClipperUtils.hpp:428-429), and the offset2_ex call
            // also uses DefaultJoinType=jtMiter (ClipperUtils.hpp:396-397, no explicit
            // join type at the call site). Both offsets route through clipper-z-sys
            // (vendored ClipperLib @ 1e5, jtMiter, MiterLimit=3.0) instead of geo-clipper
            // @ 1µm, which over-segments the gap-region contours feeding medial_axis.
            // `diff_ex` also routes through the vertex-exact vendored ClipperLib
            // (difference_clib) so the gap band contours are not re-segmented.
            // R173: native runs the collapse PER SURFACE (PG.cpp:1327 sits
            // inside the surface loop) — the merged-set collapse lets nearby
            // islands' gaps interact through the ±max/2 offsets (L0 oracle:
            // 23-vs-48 polylines from identical inputs). Gated F1_UNION;
            // default keeps the merged collapse (byte-locked).
            let gap_sets: Vec<ExPolygons> = if crate::faithful_gate("F1_UNION")
                && !result.gap_fills_per_surface.is_empty()
            {
                result.gap_fills_per_surface.clone()
            } else {
                vec![result.gap_fills.clone()]
            };
            let mut gaps_ex: ExPolygons = Vec::new();
            // R278 (F1_UNION): native band deltas are doubles from coord_t —
            // min = 0.2*W*(1-0.4), max = 2.*S with W/S = trunc(v/1e-5)
            // (PG.cpp:1329-1334); halving in DOUBLE, +ClipperSafetyOffset(10)
            // in the scaled expr before the f32 boundary.
            let (bmin, bmax) = if crate::faithful_gate("F1_UNION") {
                let w_sc = (perimeter_width / 0.00001).trunc();
                let s_sc = (perimeter_spacing / 0.00001).trunc();
                (
                    0.2 * w_sc * (1.0 - 0.4) / crate::SCALING_FACTOR,
                    2.0 * s_sc / crate::SCALING_FACTOR,
                )
            } else {
                (min, max)
            };
            let gfp = crate::env_f64("GFPROBE_LID") == Some(self.layer_id as f64);
            if gfp {
                eprintln!(
                    "GFPARAM lid={} min={:.6} max={:.6} half_min={:.6}",
                    self.layer_id,
                    bmin / crate::SCALING_FACTOR.recip(),
                    bmax / crate::SCALING_FACTOR.recip(),
                    (bmin / 2.0) / crate::SCALING_FACTOR.recip()
                );
            }
            let gfdump = |tag: &str, v: &ExPolygons| {
                for ex in v {
                    let mut line = format!("{} lid={} n={}", tag, self.layer_id, ex.contour.points.len());
                    for pt in &ex.contour.points {
                        line.push_str(&format!(" {},{}", pt.x(), pt.y()));
                    }
                    for h in &ex.holes {
                        line.push_str(&format!(" H{}", h.points.len()));
                        for pt in &h.points {
                            line.push_str(&format!(" {},{}", pt.x(), pt.y()));
                        }
                    }
                    eprintln!("{}", line);
                }
            };
            for gap_set in &gap_sets {
                let opened_min =
                    offset2_clib(gap_set, bmin / 2.0, bmin / 2.0, OffsetJoinType::Miter);
                let wide_part = offset2_clib(
                    gap_set,
                    bmax / 2.0,
                    bmax / 2.0 + CLIPPER_SAFETY_OFFSET,
                    OffsetJoinType::Miter,
                );
                let piece = difference_clib(&opened_min, &wide_part);
                if gfp {
                    gfdump("GFC1", &opened_min);
                    gfdump("GFC2", &wide_part);
                    gfdump("GFC3", &piece);
                }
                gaps_ex.extend(piece);
            }

            // PerimeterGenerator.cpp:914 — surface_simplify_resolution =
            //   (enable_arc_fitting && fuzzy_skin == None) ? 0.2 * m_scaled_resolution
            //                                              : m_scaled_resolution,
            // where m_scaled_resolution = scaled<double>(print_config.resolution)
            // (PerimeterGenerator.cpp:911). ExPolygon::douglas_peucker takes the
            // *scaled* tolerance directly (forwards to MultiPoint::_douglas_peucker
            // without rescale), so scale print_config.resolution here.
            let scaled_resolution = crate::scale(print_config.resolution) as f64;
            let surface_simplify_resolution = if config.fuzzy_skin_mode
                == crate::region_config::FuzzySkinMode::None
                && (print_config.arc_fitting_enabled
                    || !crate::faithful_gate("PG_ARC_FITTING_CFG"))
            {
                // R810: the 0.2 factor needs enable_arc_fitting, which was
                // assumed here as it was in PerimeterConfig above.
                0.2 * scaled_resolution
            } else {
                scaled_resolution
            };

            // PerimeterGenerator.cpp:1335-1340 — medial axis of each thin gap region.
            let mut polylines: crate::geometry::ThickPolylines = Vec::new();
            for ex in &mut gaps_ex {
                // PerimeterGenerator.cpp:1337-1338 — BBS: Use DP simplify to avoid
                // duplicated points and accelerate medial-axis calculation.
                ex.douglas_peucker(surface_simplify_resolution);
                // PerimeterGenerator.cpp:1339
                if crate::faithful_gate("MEDIALAXIS_NATIVE") {
                    // Native params are doubles built FROM scaled coord_t values
                    // (PG.cpp:864-865 scaled_width/scaled_spacing = trunc(v/1e-5),
                    // :1329-1330 min = 0.2*W*(1-0.4), max = 2.*S) — recompute here
                    // instead of the mm×1e5 roundtrip which drifts ulps.
                    let w_sc = (perimeter_width / 0.00001).trunc();
                    let s_sc = (perimeter_spacing / 0.00001).trunc();
                    ex.medial_axis_scaled(0.2 * w_sc * (1.0 - 0.4), 2.0 * s_sc, &mut polylines);
                } else {
                    ex.medial_axis(min, max, &mut polylines);
                }
            }
            if crate::env_f64("GFPROBE_LID") == Some(self.layer_id as f64) {
                for ex in &gaps_ex {
                    let mut line = format!("GFGAPEX lid={} n={}", self.layer_id, ex.contour.points.len());
                    for pt in &ex.contour.points {
                        line.push_str(&format!(" {},{}", pt.x(), pt.y()));
                    }
                    eprintln!("{}", line);
                }
                for tp in &polylines {
                    let mut line = format!(
                        "GFMA lid={} n={} ep={},{}",
                        self.layer_id,
                        tp.points.len(),
                        tp.endpoints[0] as i32,
                        tp.endpoints[1] as i32
                    );
                    for pt in &tp.points {
                        line.push_str(&format!(" {},{}", pt.x(), pt.y()));
                    }
                    line.push_str(" W");
                    let ws: &[f64] = if tp.edge_widths.is_empty() { &tp.widths } else { &tp.edge_widths };
                    for w in ws {
                        line.push_str(&format!(" {:.1}", w));
                    }
                    eprintln!("{}", line);
                }
            }

            // PerimeterGenerator.cpp:1357-1360 — filter tiny gap fills
            // (config.filter_out_gap_fill default 0.0 in this profile -> no extra filter).

            // PerimeterGenerator.cpp:1364 — variable_width(polylines, erGapFill, solid_infill_flow)
            if !polylines.is_empty() {
                let gap_fill_flow = self
                    .flow(FlowRole::SolidInfill, layer_height)
                    .unwrap_or_else(|_| perimeter_flow.clone());
                // GAPFILL_LOOP (on unless =0): VariableWidth.cpp:220-230 keeps the
                // paths of one thick polyline together and, when the first point
                // of the first equals the last point of the last, appends them as
                // one ExtrusionLoop; the port appended every path on its own, so a
                // closed gap fill was never split at the nozzle's position, clipped
                // at its end or turned counter-clockwise by `extrude_loop`.
                let gapfill_loop = crate::faithful_gate("GAPFILL_LOOP");
                let mut paths: Vec<crate::extrusion_entity::ExtrusionPath> = Vec::new();
                // The end index, in `paths`, of each thick polyline's paths.
                let mut group_ends: Vec<usize> = Vec::new();
                for tp in &polylines {
                    paths.extend(convert_thin_walls_to_extrusion_paths(
                        &vec![tp.clone()],
                        ExtrusionRole::GapFill,
                        &gap_fill_flow,
                    ));
                    group_ends.push(paths.len());
                }

                // PerimeterGenerator.cpp:1373 — remove the gap-fill extrusion footprint
                // from the infill area so the gap-filled band is NOT also classified as
                // (and later solidified as) internal infill:
                //   last = diff_ex(last, gap_fill.polygons_covered_by_width(10.f));
                // C++ subtracts from `last` before the final infill-boundary inset; the
                // Rust perimeter generator already produced the inset `infill_area` (now in
                // fill_surfaces / fill_expolygons), so we apply the equivalent diff here on
                // the populated infill region. `10.f` is ClipperSafetyOffset = 10 scaled
                // units (ExtrusionEntity.cpp:55-58 polygons_covered_by_width scaled_epsilon).
                let mut covered: Vec<crate::geometry::Polygon> = Vec::new();
                for path in &paths {
                    path.polygons_covered_by_width(&mut covered, 10.0);
                }
                if crate::probe_enabled("GFPROBE") {
                    let mut iv: i64 = 0;
                    let mut ix: i64 = 0;
                    let mut iy: i64 = 0;
                    for pg in &covered {
                        iv += pg.points.len() as i64;
                        for pt in &pg.points {
                            ix += pt.x();
                            iy += pt.y();
                        }
                    }
                    eprintln!(
                        "GFPROBE lid={} np={} iv={} ix={} iy={}",
                        self.layer_id,
                        covered.len(),
                        iv,
                        ix,
                        iy
                    );
                    if crate::env_f64("GFPROBE_LID") == Some(self.layer_id as f64) {
                        for pg in &covered {
                            let mut line = format!("GFPOLY lid={} n={}", self.layer_id, pg.points.len());
                            for pt in &pg.points {
                                line.push_str(&format!(" {},{}", pt.x(), pt.y()));
                            }
                            eprintln!("{}", line);
                        }
                        for path in &paths {
                            let mut line = format!(
                                "GFPATH lid={} w={:.6} n={}",
                                self.layer_id,
                                path.width,
                                path.polyline.points.len()
                            );
                            for pt in &path.polyline.points {
                                line.push_str(&format!(" {},{}", pt.x(), pt.y()));
                            }
                            eprintln!("{}", line);
                        }
                    }
                }

                let mut paths_iter = paths.into_iter();
                let mut group_begin = 0usize;
                for group_end in group_ends {
                    let group: Vec<crate::extrusion_entity::ExtrusionPath> =
                        paths_iter.by_ref().take(group_end - group_begin).collect();
                    group_begin = group_end;
                    // VariableWidth.cpp:223-229
                    if gapfill_loop
                        && !group.is_empty()
                        && group.first().unwrap().first_point()
                            == group.last().unwrap().last_point()
                    {
                        self.thin_fills.entities.push(ExtrusionEntityType::Loop(
                            crate::extrusion_entity::ExtrusionLoop::new(
                                group,
                                crate::extrusion_entity::ExtrusionLoopRole::DEFAULT,
                            ),
                        ));
                    } else {
                        for path in group {
                            self.thin_fills
                                .entities
                                .push(ExtrusionEntityType::Path(path));
                        }
                    }
                }

                if !covered.is_empty() {
                    use crate::clipper_utils::{difference, union_polygons_ex};
                    // Native diff_ex runs ClipperLib @1e5; the geo union+difference
                    // grid the trimmed fill region @1um (the last un-gridded op on
                    // fill_expolygons — L23 top band 29pts/-0.07mm2). Gated full-res.
                    let tf = crate::faithful_gate("TOPFILL_FAITHFUL");
                    let covered_ex = if tf {
                        crate::clipper_utils::union_ex_clib(&covered, 1)
                    } else {
                        union_polygons_ex(&covered)
                    };
                    // R253 (GAPTRIM_PRETAIL): NATIVE ORDER — trim the gap
                    // footprint from the pre-tail `last` and re-run the
                    // fill-boundary tail (PG.cpp:1373 runs the diff BEFORE the
                    // tail; the post-hoc trim below cuts corridors through the
                    // already-merged area, creating ragged hole-bearing blobs).
                    if crate::faithful_gate("GAPTRIM_PRETAIL") && !tail_inputs.is_empty() {
                        let mut new_infill: ExPolygons = Vec::new();
                        let mut new_no: ExPolygons = Vec::new();
                        for ti in &tail_inputs {
                            let trimmed_last =
                                crate::clipper_utils::difference_clib(&ti.last, &covered_ex);
                            let (i2, n2, _b2) = generator.classic_fill_boundary_tail(
                                &trimmed_last,
                                &ti.top_fills,
                                &ti.fill_clip,
                                ti.loop_number,
                            );
                            new_infill.extend(i2);
                            new_no.extend(n2);
                        }
                        self.fill_surfaces
                            .surfaces
                            .retain(|s| s.surface_type != SurfaceType::Internal);
                        for ex in &new_infill {
                            self.fill_surfaces.surfaces.push(Surface {
                                expolygon: ex.clone(),
                                surface_type: SurfaceType::Internal,
                                thickness: layer_height,
                                thickness_layers: 1,
                                bridge_angle: None,
                                extra_perimeters: 0,
                            });
                        }
                        self.fill_expolygons = new_infill;
                        self.fill_no_overlap_expolygons = new_no;
                        if crate::probe_enabled("NOPROBE2") {
                            for e in self.fill_no_overlap_expolygons.iter() {
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
                                    "NOPROBE2 lid={} holes={} iv={} ix={} iy={}",
                                    self.layer_id, e.holes.len(), iv, ix, iy
                                );
                            }
                        }
                    } else {
                    // Subtract from the Internal fill_surfaces (the infill region produced
                    // by the perimeter generator; top/bottom skins are unaffected).
                    let internal: ExPolygons = self
                        .fill_surfaces
                        .surfaces
                        .iter()
                        .filter(|s| s.surface_type == SurfaceType::Internal)
                        .map(|s| s.expolygon.clone())
                        .collect();
                    if !internal.is_empty() {
                        let trimmed = if tf {
                            // Restore the top band after the trim (native never
                            // trims it — the gap diff runs on `last` upstream of
                            // the band union, PG.cpp:1373 vs 1407-1413).
                            let cut = crate::clipper_utils::difference_clib(&internal, &covered_ex);
                            if perimeter_top_band.is_empty() {
                                cut
                            } else {
                                let mut m = crate::geometry::to_polygons(&cut);
                                m.extend(crate::geometry::to_polygons(&perimeter_top_band));
                                crate::clipper_utils::union_ex_clib(&m, 1)
                            }
                        } else {
                            difference(&internal, &covered_ex)
                        };
                        self.fill_surfaces
                            .surfaces
                            .retain(|s| s.surface_type != SurfaceType::Internal);
                        for ex in &trimmed {
                            self.fill_surfaces.surfaces.push(Surface {
                                expolygon: ex.clone(),
                                surface_type: SurfaceType::Internal,
                                thickness: layer_height,
                                thickness_layers: 1,
                                bridge_angle: None,
                                extra_perimeters: 0,
                            });
                        }
                    }
                    // Keep fill_expolygons (the union infill boundary used as vshell holes)
                    // consistent with the trimmed infill region.
                    if !self.fill_expolygons.is_empty() {
                        self.fill_expolygons = if tf {
                            let cut = crate::clipper_utils::difference_clib(
                                &self.fill_expolygons,
                                &covered_ex,
                            );
                            if perimeter_top_band.is_empty() {
                                cut
                            } else {
                                let mut m = crate::geometry::to_polygons(&cut);
                                m.extend(crate::geometry::to_polygons(&perimeter_top_band));
                                crate::clipper_utils::union_ex_clib(&m, 1)
                            }
                        } else {
                            difference(&self.fill_expolygons, &covered_ex)
                        };
                    }
                    } // end legacy post-hoc trim (GAPTRIM_PRETAIL replaces it)
                }
            }
        }

        Ok(())
    }

    /// Process external surfaces
    /// LayerRegion.cpp:518-640
    /// C++: void LayerRegion::process_external_surfaces(const Layer *lower_layer, const Polygons *lower_layer_covered)
    ///
    /// Faithful driver: builds the flow-derived expansion config exactly as the
    /// C++ member does (shell_width / expansion_min from external-perimeter and
    /// perimeter flows, closing_radius from the solid-infill flow), then runs the
    /// wave-expansion port [`crate::region_expansion::process_external_surfaces_wave`]
    /// over this region's `fill_surfaces` in place.
    ///
    /// The C++ `lower_layer` / `lower_layer_covered` parameters feed the void-trim
    /// path in `PrintObject::process_external_surfaces`; the wave port does not yet
    /// consume them, so they are omitted here. `layer_height` (C++
    /// `m_layer->height`) is threaded by the caller, matching [`LayerRegion::flow`].
    pub fn process_external_surfaces(&mut self, layer_height: f64) -> Result<()> {
        use crate::region_expansion::{process_external_surfaces_wave, ExternalSurfaceConfig};

        // LayerRegion.cpp:526-539 — width of the perimeters.
        // C++: int num_perimeters = this->region().config().wall_loops;
        let num_perimeters = self.region().config().perimeters as usize;

        // LayerRegion.cpp:529-534
        // C++: Flow external_perimeter_flow = this->flow(frExternalPerimeter);
        // C++: Flow perimeter_flow          = this->flow(frPerimeter);
        // ExternalSurfaceConfig::from_flows reproduces the shell_width /
        // expansion_min computation (LayerRegion.cpp:532-534) and the
        // num_perimeters == 0 SCALED_EPSILON fallback (LayerRegion.cpp:536-538).
        // Note: from_flows works in mm (unscaled) widths/spacings; the wave port
        // scales internally, so feed the unscaled flow values.
        let (external_perimeter_width, external_perimeter_spacing) = if num_perimeters > 0 {
            let f = self.flow(FlowRole::ExternalPerimeter, layer_height)?;
            (f.width(), f.spacing())
        } else {
            (0.0, 0.0)
        };
        let perimeter_spacing = if num_perimeters > 0 {
            self.flow(FlowRole::Perimeter, layer_height)?.spacing()
        } else {
            0.0
        };

        // LayerRegion.cpp:550
        // C++: const float closing_radius = ... this->flow(frSolidInfill).scaled_spacing();
        let solid_infill_spacing = self.flow(FlowRole::SolidInfill, layer_height)?.spacing();

        let mut config = ExternalSurfaceConfig::from_flows(
            external_perimeter_width,
            external_perimeter_spacing,
            perimeter_spacing,
            solid_infill_spacing,
            num_perimeters,
        );

        // LayerRegion.cpp:569 — custom bridge angle (degrees).
        // C++: const double custom_angle = this->region().config().bridge_angle.value;
        config.custom_bridge_angle = self.region().config().bridge_angle;

        // LayerRegion.cpp:599-602 — minimum sparse infill area logic guards.
        // C++: if (!...->config().spiral_mode && this->region().config().sparse_infill_density.value > 0)
        // C++:     double min_area = scale_(scale_(...minimum_sparse_infill_area.value));
        config.spiral_mode = self
            .print_config
            .as_deref()
            .expect("config hierarchy not wired — call wire_layer_hierarchy")
            .spiral_vase;
        config.sparse_infill_density = self.region().config().fill_density;
        // minimum_sparse_infill_area lives on PrintObjectConfig in this crate.
        config.minimum_sparse_infill_area = self
            .object_config
            .as_deref()
            .expect("config hierarchy not wired — call wire_layer_hierarchy")
            .minimum_sparse_infill_area;

        // LAYER_REGION_EXTERNAL_PIN (faithful gate, R924): the pin's solid and
        // sparse templates carry `layer_thickness` (LayerRegion.cpp:891-898),
        // the thickness `fill_surfaces_extract_expolygons` read from the last
        // surface of the last of {stInternalSolid}, {stInternal}, {stTop} that
        // has a surface (LayerRegion.cpp:212-219, 820-823). The wave port left
        // the default (-1) on them.
        let mut layer_thickness: Option<f64> = None;
        if crate::faithful_gate("LAYER_REGION_EXTERNAL_PIN") {
            for surface_type in [SurfaceType::InternalSolid, SurfaceType::Internal, SurfaceType::Top] {
                for surface in &self.fill_surfaces.surfaces {
                    if surface.surface_type == surface_type {
                        layer_thickness = Some(surface.thickness);
                    }
                }
            }
        }

        // BRDG probe (diagnostic only): which layer / region the bridge lines that follow belong to.
        if crate::probe_enabled("BRDG") {
            eprintln!("BRDG layer id={} region={}", self.layer_id, self.region_id);
        }
        // LayerRegion.cpp:518 — operate on this region's fill_surfaces in place.
        let mut surfaces = vec![std::mem::take(&mut self.fill_surfaces.surfaces)];
        process_external_surfaces_wave(&mut surfaces, &config);
        self.fill_surfaces.surfaces = surfaces.into_iter().next().unwrap_or_default();
        if let Some(layer_thickness) = layer_thickness {
            // LayerRegion.cpp:891-898 `solid_templ.thickness = layer_thickness`,
            // `sparse_templ.thickness = layer_thickness`.
            for surface in &mut self.fill_surfaces.surfaces {
                if matches!(surface.surface_type, SurfaceType::InternalSolid | SurfaceType::Internal) {
                    surface.thickness = layer_thickness;
                }
            }
        }
        Ok(())
    }

    /// Calculate infill area threshold
    /// LayerRegion.cpp:690-694
    /// C++: double LayerRegion::infill_area_threshold() const
    /// C++: {
    /// C++:     double ss = this->flow(frSolidInfill).scaled_spacing();
    /// C++:     return ss*ss;
    /// C++: }
    ///
    /// C++ reads `this->flow(frSolidInfill)`; this crate has no `Print` back-pointer
    /// on `LayerRegion`, so the solid-infill flow is supplied by the caller.
    pub fn infill_area_threshold_with_flow(&self, solid_infill_flow: &Flow) -> f64 {
        // LayerRegion.cpp:692
        let ss = solid_infill_flow.scaled_spacing() as f64;
        // LayerRegion.cpp:693
        ss * ss
    }

    /// Calculate infill area threshold (legacy 1mm² fallback retained for callers
    /// that have no flow handy; the faithful version is
    /// `infill_area_threshold_with_flow`).
    /// LayerRegion.cpp:690-694
    pub fn infill_area_threshold(&self) -> f64 {
        scale(1.0) as f64 // 1mm² threshold
    }

    /// Trim surfaces by trimming polygons. Used by the elephant foot compensation at the 1st layer.
    /// LayerRegion.cpp:696-703
    /// C++: void LayerRegion::trim_surfaces(const Polygons &trimming_polygons)
    pub fn trim_surfaces(&mut self, trimming_polygons: &Polygons) {
        // LayerRegion.cpp:698-701
        // #ifndef NDEBUG
        //     for (const Surface &surface : this->slices.surfaces)
        //         assert(surface.surface_type == stInternal);
        // #endif
        debug_assert!(self
            .slices
            .surfaces
            .iter()
            .all(|s| s.surface_type == SurfaceType::Internal));

        // LayerRegion.cpp:702
        // this->slices.set(intersection_ex(this->slices.surfaces, trimming_polygons), stInternal);
        let clip: ExPolygons = trimming_polygons
            .iter()
            .map(|p| crate::geometry::ExPolygon::from(p.clone()))
            .collect();
        let trimmed = crate::clipper_utils::intersection_ex(&self.slices.surfaces, &clip);
        self.slices.set_expolygons(trimmed, SurfaceType::Internal);
    }

    /// LayerRegion.cpp:972-981 `void LayerRegion::elephant_foot_compensation_step(const float
    /// elephant_foot_compensation_perimeter_step, const Polygons &trimming_polygons)` with the two arguments
    /// as a pair, `layerm.elephant_foot_compensation_step((step, &trimming_polygons))`; with one `f64` it is
    /// the shrink of the slices the slicing path of this crate has called by that name
    /// ([`LayerRegion::elephant_foot_compensation_step_legacy`]).
    pub fn elephant_foot_compensation_step<A: ElephantFootCompensationStepArgs>(&mut self, args: A) {
        args.apply(self)
    }

    /// LayerRegion.cpp:972-981: the slices trimmed by the compensated outline, with what an opening by the
    /// step removes of them put back (the thin parts the compensation would lose).
    fn elephant_foot_compensation_step_pin(&mut self, elephant_foot_compensation_perimeter_step: f32, trimming_polygons: &Polygons) {
        use crate::clipper::{ClipType, Paths};
        use crate::clipper_utils as u;
        use crate::clipper_utils::ApplySafetyOffset;
        use crate::support::support_common::cu;
        // `ClipperUtils::SurfacesProvider`: per surface its contour, then its holes.
        let expolygons: ExPolygons = self.slices.surfaces.iter().map(|s| s.expolygon.clone()).collect();
        let surfaces: Paths = cu::expolygons_provider(&expolygons);
        let trimming: Paths = cu::polygons_provider(trimming_polygons);
        // ClipperUtils.cpp:709 `intersection(const Surfaces &, const Polygons &)`
        let mut tmp: Polygons = cu::clipper_paths(ClipType::ctIntersection, &surfaces, &trimming, ApplySafetyOffset::No);
        // ClipperUtils.hpp:426, ClipperUtils.cpp:625-631 `opening(const Surfaces &, delta)`: shrunk, then
        // expanded, by the same delta.
        let delta = elephant_foot_compensation_perimeter_step as f64;
        let opened: Paths = u::expand_paths::<Paths>(
            &u::shrink_paths::<Paths>(&surfaces, delta, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
            delta,
            cu::DEFAULT_JOIN_TYPE,
            cu::DEFAULT_MITER_LIMIT,
        );
        // ClipperUtils.cpp:693 `diff(const Surfaces &, const Polygons &)`
        tmp.extend(cu::clipper_paths(ClipType::ctDifference, &surfaces, &opened, ApplySafetyOffset::No));
        self.slices.set_expolygons(cu::union_ex(&tmp), SurfaceType::Internal);
    }

    /// The function the slicing path of this crate has called `elephant_foot_compensation_step`: the slices
    /// shrunk inward by `compensation_xy` millimetres. It is not LayerRegion.cpp's function of that name.
    pub fn elephant_foot_compensation_step_legacy(&mut self, compensation_xy: f64) {
        if compensation_xy <= 0.0 || self.slices.surfaces.is_empty() {
            return;
        }

        // C++ reference: PrintObjectSlice.cpp:1228-1244
        // Slic3r::elephant_foot_compensation(lslices, ext_perimeter_flow, compensation)
        // For simple shapes this is equivalent to shrinking the polygon by compensation_xy mm.
        let slices_polys: ExPolygons = self
            .slices
            .surfaces
            .iter()
            .map(|s| s.expolygon.clone())
            .collect();

        // Shrink polygon by compensation amount (inward offset)
        let result = crate::clipper_utils::shrink(
            &slices_polys,
            compensation_xy,
            crate::clipper_utils::OffsetJoinType::Miter,
        );

        // Replace slices with compensated geometry
        let surface_type = self
            .slices
            .surfaces
            .first()
            .map(|s| s.surface_type)
            .unwrap_or(crate::surface::SurfaceType::Internal);
        let thickness = self
            .slices
            .surfaces
            .first()
            .map(|s| s.thickness)
            .unwrap_or(0.2);

        self.slices.surfaces.clear();
        for ep in result {
            self.slices.surfaces.push(crate::surface::Surface {
                expolygon: ep,
                surface_type,
                thickness,
                thickness_layers: 1,
                bridge_angle: None,
                extra_perimeters: 0,
            });
        }
    }

    /// Check if region has any extrusions
    /// Layer.hpp:140
    pub fn has_extrusions(&self) -> bool {
        !self.perimeters.entities.is_empty() || !self.fills.entities.is_empty()
    }
}

/// What [`LayerRegion::elephant_foot_compensation_step`] is called with: the pair of the pin's arguments
/// (LayerRegion.cpp:972), or the one `f64` of the body this crate's slicing path calls.
pub trait ElephantFootCompensationStepArgs {
    fn apply(self, layerm: &mut LayerRegion);
}
impl ElephantFootCompensationStepArgs for f64 {
    fn apply(self, layerm: &mut LayerRegion) {
        layerm.elephant_foot_compensation_step_legacy(self)
    }
}
impl ElephantFootCompensationStepArgs for (f32, &Polygons) {
    fn apply(self, layerm: &mut LayerRegion) {
        layerm.elephant_foot_compensation_step_pin(self.0, self.1)
    }
}

// NOTE: the canonical PrintRegion lives in crate::print_region (shared via
// Arc into LayerRegion::region); a dead local shadow struct was removed here.

/// Convert a perimeter loop to an extrusion loop
/// Layer.cpp:200-240
fn convert_polygon_to_extrusion_loop(
    polygon: &Polygon,
    flow: &Flow,
    role: ExtrusionRole,
) -> ExtrusionLoop {
    let mut paths = Vec::new();

    // Create a single path from the polygon
    // Layer.cpp:242-260
    let mut path = ExtrusionPath::new(role);
    path.polyline.points = polygon.points.clone();
    path.mm3_per_mm = flow.mm3_per_mm().unwrap_or(0.0);
    path.width = flow.width();
    path.height = flow.height();

    paths.push(path);

    ExtrusionLoop::new(paths, crate::extrusion_entity::ExtrusionLoopRole::DEFAULT)
}

/// Convert a polyline to an extrusion path
/// Layer.cpp:242-260
fn convert_polyline_to_extrusion_path(
    polyline: &Polyline,
    flow: &Flow,
    role: ExtrusionRole,
) -> ExtrusionPath {
    // Layer.cpp:242-260
    let mut path = ExtrusionPath::new(role);
    path.polyline = polyline.clone();
    path.mm3_per_mm = flow.mm3_per_mm().unwrap_or(0.0);
    path.width = flow.width();
    path.height = flow.height();
    path
}

/// Extract and union surfaces of given types
/// Layer.cpp:262-285
fn extract_and_union_surfaces(surfaces: &[Surface], types: &[SurfaceType]) -> ExPolygons {
    let mut result = ExPolygons::new();

    for surface in surfaces {
        if types.contains(&surface.surface_type) {
            result.push(surface.expolygon.clone());
        }
    }

    // Union overlapping expolygons
    union_ex(&result)
}

/// Outer-wall contour snapshot for z-direction speed continuity
/// ExtrusionEntity.hpp:21-26 (NodeContour)
#[derive(Debug, Clone, Default)]
pub struct NodeContour {
    pub pts: Vec<crate::Point>,
    pub widths: Vec<crate::Coord>,
    pub is_loop: bool,
}

/// Node for loop hierarchy
/// ExtrusionEntity.hpp:28-42 (LoopNode)
#[derive(Debug, Clone)]
pub struct LoopNode {
    pub node_id: usize,
    pub loop_id: usize,
    pub bbox: BoundingBox,
    pub merged_id: Option<usize>,
    pub upper_node_ids: Vec<usize>,
    pub lower_node_ids: Vec<usize>,
    pub node_contour: NodeContour,
}

impl LoopNode {
    pub fn new(node_id: usize, loop_id: usize, bbox: BoundingBox) -> Self {
        Self {
            node_id,
            loop_id,
            bbox,
            merged_id: None,
            upper_node_ids: Vec::new(),
            lower_node_ids: Vec::new(),
            node_contour: NodeContour::default(),
        }
    }
}

impl Default for LoopNode {
    fn default() -> Self {
        Self::new(0, 0, BoundingBox::empty())
    }
}

/// Represents a layer in the print
/// Layer.hpp:150
#[derive(Debug, Clone)]
pub struct Layer {
    /// Layer.hpp:152
    id: usize,
    /// Object ID this layer belongs to
    /// Layer.hpp:153
    object_id: usize,

    // Layer connectivity
    /// Layer.hpp:156-157
    pub upper_layer_id: Option<usize>,
    pub lower_layer_id: Option<usize>,

    /// Layer.hpp:160
    pub slicing_errors: bool,

    // Z coordinates
    /// Layer.hpp:163
    pub slice_z: f64,
    /// Layer.hpp:166
    pub print_z: f64,
    /// Layer.hpp:169
    pub height: f64,

    /// Layer.hpp:172-174
    pub sharp_tails: Polygons,
    pub cantilevers: Polygons,
    pub sharp_tails_height: f64,

    // Slices
    /// Layer.hpp:178
    pub lslices: ExPolygons,
    /// Layer.hpp:181
    pub lslices_extrudable: ExPolygons,
    /// Layer.hpp:184
    pub lslices_bboxes: Vec<BoundingBox>,

    /// Layer.hpp:168-170. R631: these were `Polygons`/`Vec<(Polygon, u32)>`;
    /// C++ declares `ExPolygons` and `vector<pair<ExPolygon, int>>`, and the
    /// overhang region has holes that matter to `is_through_overhang`.
    pub loverhangs: ExPolygons,
    pub loverhangs_with_type: Vec<(crate::geometry::ExPolygon, i32)>,
    pub loverhangs_bbox: BoundingBox,

    /// Layer.hpp:192
    pub loop_nodes: Vec<LoopNode>,

    // Regions
    /// Layer.hpp:196
    regions: Vec<LayerRegion>,

    /// Support material fills for this layer.
    /// C++ SupportLayer::support_fills
    pub support_fills: Option<ExtrusionEntityCollection>,

    /// Snapshot of the owning PrintObject's config, shared via Arc.
    /// C++ reaches it via the parent pointer: `PrintObject *m_object;`
    /// (Layer.hpp:309) + `object()->config()` (Print.hpp:369). Stamped by
    /// PrintObject::wire_layer_hierarchy at sync points; None until wired.
    /// Present on support layers too (stamped at the end of
    /// generate_support_material).
    pub(crate) object_config: Option<Arc<crate::print_config::PrintObjectConfig>>,

    /// `object()->bounding_box()` (Print.hpp:442), which Fill.cpp:599, 699 hand
    /// the filler. Stamped by PrintObject::wire_layer_hierarchy with
    /// [`print_object_bounding_box`]; None until wired.
    pub(crate) object_bounding_box: Option<crate::geometry::BoundingBox>,

    /// Snapshot of the owning Print's config, shared via Arc.
    /// C++: `object()->print()->config()` (PrintBase.hpp:632 + Print.hpp:885).
    /// Stamped at sync points; None until wired.
    pub(crate) print_config: Option<Arc<crate::print_config::PrintConfig>>,
}

// Marker traits for thread safety
// Layer.hpp:200-201
unsafe impl Send for Layer {}
unsafe impl Sync for Layer {}

impl Layer {
    /// Create a new layer
    /// Layer.cpp:312-345
    pub fn new(id: usize, object_id: usize, height: f64, print_z: f64, slice_z: f64) -> Self {
        Self {
            id,
            object_id,
            upper_layer_id: None,
            lower_layer_id: None,
            slicing_errors: false,
            slice_z,
            print_z,
            height,
            sharp_tails: Polygons::new(),
            cantilevers: Polygons::new(),
            sharp_tails_height: 0.0,
            lslices: ExPolygons::new(),
            lslices_extrudable: ExPolygons::new(),
            lslices_bboxes: Vec::new(),
            loverhangs: ExPolygons::new(),
            loverhangs_with_type: Vec::new(),
            loverhangs_bbox: BoundingBox::empty(),
            loop_nodes: Vec::new(),
            regions: Vec::new(),
            support_fills: None,
            object_config: None,
            object_bounding_box: None,
            print_config: None,
        }
    }

    /// Get layer ID
    /// Layer.hpp:204
    pub fn id(&self) -> usize {
        self.id
    }

    /// Set layer ID
    /// Layer.hpp:205
    pub fn set_id(&mut self, id: usize) {
        self.id = id;
    }

    /// Get object ID
    /// Layer.hpp:206
    pub fn object_id(&self) -> usize {
        self.object_id
    }

    /// Upward view to the owning PrintObject, preserving the C++ call shapes
    /// `layer->object()->config()` and `layer->object()->print()->config()`.
    /// C++: `PrintObject* object() { return m_object; }` (Layer.hpp:139)
    ///
    /// Works on region-less support layers too — they get their config Arcs
    /// stamped at the end of generate_support_material; only
    /// LayerRegion::region() is restricted to true printing regions.
    pub fn object(&self) -> crate::print_object::ObjectRef<'_> {
        crate::print_object::ObjectRef::new(
            self.object_config
                .as_deref()
                .expect("config hierarchy not wired — call wire_layer_hierarchy"),
            self.print_config
                .as_deref()
                .expect("config hierarchy not wired — call wire_layer_hierarchy"),
        )
    }

    /// Stamp the config-hierarchy Arcs onto this layer and its LayerRegions.
    /// Called by PrintObject::wire_layer_hierarchy at sync points (end of
    /// PrintObject::slice; end of generate_support_material for the support
    /// layers created there).
    ///
    /// The region Arc is looked up via each LayerRegion's region id into
    /// `all_regions`, mirroring the C++ ctor wiring
    /// `LayerRegion(Layer *layer, const PrintRegion *region)` (Layer.hpp:125).
    /// Support layers have no LayerRegions, so their `region` stays None.
    ///
    /// INVARIANT: the Arcs are cloned/replaced wholesale — NEVER mutated in
    /// place via Arc::make_mut/get_mut, which would fork the share. Faithful
    /// because C++ configs only mutate inside Print::apply and any diff there
    /// invalidates posSlice.
    pub(crate) fn wire_config_hierarchy(
        &mut self,
        object_config: &Arc<crate::print_config::PrintObjectConfig>,
        print_config: &Arc<crate::print_config::PrintConfig>,
        all_regions: &[Arc<crate::print_region::PrintRegion>],
    ) {
        self.object_config = Some(object_config.clone());
        self.print_config = Some(print_config.clone());
        for layerm in &mut self.regions {
            layerm.region = all_regions.get(layerm.region_id).cloned();
            layerm.object_config = Some(object_config.clone());
            layerm.print_config = Some(print_config.clone());
        }
    }

    /// The configs of the object and of the print a Layer of the pin reaches through `m_object`
    /// (Layer.hpp:139), for a Layer that stands on its own (tests/fill_native_parity.rs): what
    /// `wire_config_hierarchy` stamps on the layer itself. The regions keep what
    /// `LayerRegion::set_config_hierarchy` gave them.
    pub fn set_object_configs(
        &mut self,
        object_config: Arc<crate::print_config::PrintObjectConfig>,
        print_config: Arc<crate::print_config::PrintConfig>,
    ) {
        self.object_config = Some(object_config);
        self.print_config = Some(print_config);
    }

    /// `object()->bounding_box()` (PrintObject.hpp, read at Fill.cpp:689) for a Layer that stands
    /// on its own (tests/fill_native_parity.rs): what print_object.rs stamps on its layers.
    pub fn set_object_bounding_box(&mut self, bbox: Option<crate::geometry::BoundingBox>) {
        self.object_bounding_box = bbox;
    }

    /// Get bottom Z coordinate
    /// Layer.hpp:209
    pub fn bottom_z(&self) -> f64 {
        self.print_z - self.height
    }

    /// Get number of regions
    /// Layer.hpp:212
    pub fn region_count(&self) -> usize {
        self.regions.len()
    }

    /// Get a region by index
    /// Layer.hpp:215
    pub fn get_region(&self, idx: usize) -> Option<&LayerRegion> {
        self.regions.get(idx)
    }

    /// Get a mutable region by index
    /// Layer.hpp:218
    pub fn get_region_mut(&mut self, idx: usize) -> Option<&mut LayerRegion> {
        self.regions.get_mut(idx)
    }

    /// Get all regions
    /// Layer.hpp:221
    pub fn regions(&self) -> &[LayerRegion] {
        &self.regions
    }

    /// Get all regions mutably
    /// Layer.cpp:347
    pub fn regions_mut(&mut self) -> &mut [LayerRegion] {
        &mut self.regions
    }

    /// Add a new region
    /// Layer.cpp:349-355
    pub fn add_region(&mut self, region: LayerRegion) {
        self.regions.push(region);
    }

    /// PrintObjectSlice.cpp:2118-2122 `while (!layer->m_regions.empty()) { delete layer->m_regions.back();
    /// layer->m_regions.pop_back(); }`: what `PrintObject::slice_volumes` does to `m_regions` before it
    /// allocates the LayerRegions of the current PrintRegions.
    pub fn clear_regions(&mut self) {
        self.regions.clear();
    }

    // Test whether whether there are any slices assigned to this layer.
    // Layer.cpp:25
    pub fn empty(&self) -> bool {
        // Layer.cpp:27-31
        // C++: for (const LayerRegion *layerm : m_regions)
        // C++:     if (layerm != nullptr && ! layerm->slices.empty())
        // C++:         // Non empty layer.
        // C++:         return false;
        // C++: return true;
        for layerm in &self.regions {
            if !layerm.slices.is_empty() {
                // Non empty layer.
                return false;
            }
        }
        true
    }

    // If there is any incompatibility, separate LayerRegions have to be created.
    // Layer.cpp:173
    // C++: bool Layer::has_compatible_layer_regions(const PrintRegionConfig &config, const PrintRegionConfig &other_config)
    //
    // Porting notes (Rust PrintRegionConfig field-name mapping):
    //   wall_loops           -> perimeters
    //   inner_wall_speed     -> perimeter_speed (C++ per-extruder .get_at(get_process_config_idx(wall_filament)); Rust scalar)
    //   outer_wall_speed     -> external_perimeter_speed (same per-extruder note)
    //   gap_infill_speed     -> gap_fill_speed (same per-extruder note)
    //   detect_overhang_wall -> overhangs
    //   detect_thin_wall     -> thin_walls
    //   infill_wall_overlap  -> infill_overlap
    //   opt_serialize("inner_wall_line_width"/"outer_wall_line_width") -> direct float compare
    //   override_filament_scarf_seam_setting / seam_slope_* keep their C++ names
    pub fn has_compatible_layer_regions(
        &self,
        config: &PrintRegionConfig,
        other_config: &PrintRegionConfig,
    ) -> bool {
        // Layer.cpp:175
        config.wall_filament == other_config.wall_filament
            // Layer.cpp:176
            && config.perimeters == other_config.perimeters
            // Layer.cpp:177
            && config.wall_sequence == other_config.wall_sequence
            // Layer.cpp:178
            && config.perimeter_speed == other_config.perimeter_speed
            // Layer.cpp:179
            && config.external_perimeter_speed == other_config.external_perimeter_speed
            // Layer.cpp:180
            && config.gap_fill_speed == other_config.gap_fill_speed
            // Layer.cpp:181 `bridge_speed.get_at(get_process_config_idx(wall_filament))`: regions with
            // different bridge speeds are not merged (gate LAYER_COMPATIBLE_BRIDGE_SPEED; off = R927).
            && (!crate::faithful_gate("LAYER_COMPATIBLE_BRIDGE_SPEED") || config.bridge_speed == other_config.bridge_speed)
            // Layer.cpp:182
            && config.overhangs == other_config.overhangs
            // Layer.cpp:183
            && config.filter_out_gap_fill == other_config.filter_out_gap_fill
            // Layer.cpp:184
            && config.inner_wall_line_width == other_config.inner_wall_line_width
            // Layer.cpp:185
            && config.outer_wall_line_width == other_config.outer_wall_line_width
            // Layer.cpp:186
            && config.thin_walls == other_config.thin_walls
            // Layer.cpp:187
            && config.infill_overlap == other_config.infill_overlap
            // Layer.cpp:188
            && config.override_filament_scarf_seam_setting == other_config.override_filament_scarf_seam_setting
            // Layer.cpp:189
            && config.seam_slope_type == other_config.seam_slope_type
            // Layer.cpp:190
            && config.seam_slope_start_height == other_config.seam_slope_start_height
            // Layer.cpp:191
            && config.seam_slope_gap == other_config.seam_slope_gap
            // Layer.cpp:192
            && config.seam_slope_min_length == other_config.seam_slope_min_length
            // Layer.cpp:193
            && config.seam_slope_conditional == other_config.seam_slope_conditional
            // Layer.cpp:194
            && config.seam_slope_entire_loop == other_config.seam_slope_entire_loop
            // Layer.cpp:195
            && config.seam_slope_steps == other_config.seam_slope_steps
            // Layer.cpp:196
            && config.seam_slope_inner_walls == other_config.seam_slope_inner_walls
            // Layer.cpp:197 `(this->id() != 0 || config.initial_layer_flow_ratio == other_config.initial_layer_flow_ratio)`:
            // on the first layer regions with different flow ratios are not merged
            // (gate LAYER_COMPATIBLE_INITIAL_FLOW_RATIO; off = R928, where the option was not compared).
            && (!crate::faithful_gate("LAYER_COMPATIBLE_INITIAL_FLOW_RATIO")
                || self.id() != 0
                || config.initial_layer_flow_ratio == other_config.initial_layer_flow_ratio)
    }

    /// Generate perimeters for all regions
    /// Layer.cpp:201-304
    /// Make perimeters with lower layer data for overhang detection
    pub fn make_perimeters_with_lower(
        &mut self,
        lower_slices: Option<&Vec<crate::geometry::ExPolygon>>,
    ) -> Result<()> {
        self.make_perimeters_with_neighbors(lower_slices, None)
    }

    /// Make perimeters with both lower- and upper-layer slices.
    /// upper_slices feeds the top-surface detection (top_fills, PerimeterGenerator.cpp:1118).
    pub fn make_perimeters_with_neighbors(
        &mut self,
        lower_slices: Option<&Vec<crate::geometry::ExPolygon>>,
        upper_slices: Option<&Vec<crate::geometry::ExPolygon>>,
    ) -> Result<()> {
        self.make_perimeters_with_neighbors_in_object(lower_slices, upper_slices, None)
    }

    /// R935: `make_perimeters_with_neighbors` with the object's layers (`this->object()->get_layer(i)`,
    /// read by Layer.cpp:304-351 for a region with adaptive merge patches); see `make_perimeters_in_object`.
    pub fn make_perimeters_with_neighbors_in_object(
        &mut self,
        lower_slices: Option<&Vec<crate::geometry::ExPolygon>>,
        upper_slices: Option<&Vec<crate::geometry::ExPolygon>>,
        object_layers: Option<&[Layer]>,
    ) -> Result<()> {
        // Store neighbor slices in each region (used by LayerRegion::make_perimeters)
        if let Some(ls) = lower_slices {
            for region in &mut self.regions {
                region.lower_slices = Some(ls.clone());
            }
        }
        if let Some(us) = upper_slices {
            for region in &mut self.regions {
                region.upper_slices = Some(us.clone());
            }
        }
        self.make_perimeters_in_object(object_layers)
    }

    // Here the perimeters are created cummulatively for all layer regions
    // sharing the same parameters influencing the perimeters.
    // Layer.cpp:201
    pub fn make_perimeters(
        &mut self,
        _perimeter_generator_options: Option<&()>,
    ) -> Result<()> {
        self.make_perimeters_in_object(None)
    }

    /// R935: the call of the `generate_region_perimeters` lambda at Layer.cpp:366 / :386. With the gate
    /// LAYER_GENERATE_REGION_PERIMETERS off, or with no object (`object_layers` None), it is the R934 call
    /// `LayerRegion::make_perimeters` (= the lambda's `adaptive_merge_patches.empty()` arm, :287-290).
    #[allow(clippy::too_many_arguments)]
    fn run_region_perimeters(
        lm: &mut LayerRegion,
        src: &SurfaceCollection,
        region_id: usize,
        object_layers: Option<&[Layer]>,
        height: f64,
        id: usize,
        print_z: f64,
    ) -> Result<()> {
        match object_layers {
            Some(layers) if crate::faithful_gate("LAYER_GENERATE_REGION_PERIMETERS") => {
                let get_layer = |i: usize| -> &Layer { &layers[i] };
                generate_region_perimeters(lm, src, region_id, layers.len(), &get_layer, height, id, print_z, &|_| {})
            }
            _ => lm.make_perimeters(src, height, id, print_z),
        }
    }

    /// R935 Layer.cpp:201-400 `Layer::make_perimeters` with the object's layers: `object_layers[i]` is
    /// `this->object()->get_layer(int(i))` and its length `this->object()->layer_count()`, as the slicing
    /// path's PrintObject::make_perimeters hands them (a read-only copy of every layer taken before the
    /// parallel loop: nothing this step writes is read through it). With `None` (a Layer outside an object)
    /// a region's walls run `make_perimeters_with` as before R935. Gates: LAYER_FIRST_BAND_SOURCE (:233-238)
    /// and LAYER_GENERATE_REGION_PERIMETERS (:284-361, 366, 386); with either off (=0) that part is the R934
    /// code. Both change nothing while no region carries adaptive merge patches.
    pub fn make_perimeters_in_object(&mut self, object_layers: Option<&[Layer]>) -> Result<()> {
        let print_z = self.print_z;
        let height = self.height;
        let id = self.id;

        // Layer.cpp:205
        // C++: std::vector<unsigned char> done(m_regions.size(), false);
        let mut done = vec![false; self.regions.len()];

        // Layer.cpp:208-230
        // C++: auto same_merge_patches = [](const LayerRegion *lhs, const LayerRegion *rhs) { ... };
        let same_merge_patches = |lhs: &LayerRegion, rhs: &LayerRegion| -> bool {
            let lhs_merged = !lhs.adaptive_merge_patches.is_empty();
            let rhs_merged = !rhs.adaptive_merge_patches.is_empty();
            if lhs_merged != rhs_merged {
                return false;
            }
            if !lhs_merged {
                return true;
            }
            // Layer.cpp:215-220: a layer may carry both markers of a slab above and a local group top of
            // its own; any extruding group top forbids a shared pass.
            let has_extruding_group_top = |layerm: &LayerRegion| -> bool {
                layerm.adaptive_merge_patches.iter().any(|patch| patch.group_top && !patch.area().is_empty())
            };
            if has_extruding_group_top(lhs) || has_extruding_group_top(rhs) {
                return false;
            }
            // Layer.cpp:223-226
            let a = &lhs.adaptive_merge_patches[0];
            let b = &rhs.adaptive_merge_patches[0];
            if a.bottom_idx != b.bottom_idx || a.top_idx != b.top_idx || a.group_top != b.group_top {
                return false;
            }
            // Layer.cpp:227-229
            !a.group_top
        };

        // Layer.cpp:207-208
        // C++: for (LayerRegionPtrs::iterator layerm = m_regions.begin(); layerm != m_regions.end(); ++ layerm)
        //      if ((*layerm)->slices.empty()) {
        for region_idx in 0..self.regions.len() {
            // Layer.cpp:233-238: a group top whose band starts at object layer 0 runs with no slices of its
            // own (gate LAYER_FIRST_BAND_SOURCE; off = R934, slices.empty() alone).
            let first_band_source =
                crate::faithful_gate("LAYER_FIRST_BAND_SOURCE") && has_first_band_source(&self.regions[region_idx]);
            if self.regions[region_idx].slices.surfaces.is_empty() && !first_band_source {
                // Layer.cpp:209-211
                // C++: (*layerm)->perimeters.clear();
                //      (*layerm)->fills.clear();
                //      (*layerm)->thin_fills.clear();
                self.regions[region_idx].perimeters.entities.clear();
                self.regions[region_idx].perimeter_islands.clear();
                self.regions[region_idx].fills.entities.clear();
                self.regions[region_idx].thin_fills.entities.clear();
            } else {
                // Layer.cpp:213-215
                // C++: size_t region_id = layerm - m_regions.begin();
                //      if (done[region_id]) continue;
                if done[region_idx] {
                    continue;
                }
                // Layer.cpp:217
                // C++: done[region_id] = true;
                done[region_idx] = true;
                // Layer.cpp:218
                // C++: const PrintRegionConfig &config = (*layerm)->region().config();
                let config = self.regions[region_idx].region().config().clone();

                // Layer.cpp:221-222
                // find compatible regions
                // C++: LayerRegionPtrs layerms;  layerms.push_back(*layerm);
                let mut layerms: Vec<usize> = vec![region_idx];

                // Layer.cpp:224
                // C++: PerimeterRegions perimeter_regions;
                // FIDELITY-NOTE: PerimeterRegion::has_compatible_perimeter_regions
                // / merge_compatible_perimeter_regions (Layer.cpp:242-244, 270-272)
                // and the LayerRegion::make_perimeters(slices, perimeter_regions,
                // ...) overload that consumes them are not ported in this crate;
                // perimeter_regions is therefore not accumulated and the reduced
                // LayerRegion::make_perimeters is used.

                // Layer.cpp:225-245
                // C++: for (it = layerm + 1; it != m_regions.end(); ++it) { ... }
                for it in (region_idx + 1)..self.regions.len() {
                    // Layer.cpp:226-227
                    if self.regions[it].slices.surfaces.is_empty() {
                        continue;
                    }
                    // Layer.cpp:231-233
                    let other_config = self.regions[it].region().config().clone();
                    if !self.has_compatible_layer_regions(&config, &other_config) {
                        continue;
                    }
                    // Layer.cpp:264-265 `if ((*layerm)->region().gradient_volume_id() !=
                    // other_layerm->region().gradient_volume_id()) continue;` (gate
                    // LAYER_PERIMETERS_GRADIENT_VOLUME_ID; off = R930, no such condition). Nothing on the
                    // slicing path sets the id, so every region carries the invalid default.
                    if crate::faithful_gate("LAYER_PERIMETERS_GRADIENT_VOLUME_ID")
                        && self.regions[region_idx].region().gradient_volume_id()
                            != self.regions[it].region().gradient_volume_id()
                    {
                        continue;
                    }
                    // Layer.cpp:266-268: a shared perimeter pass can only honor the merge plan of its front
                    // region (gate LAYER_PERIMETERS_SAME_MERGE_PATCHES; off = R928, no such condition).
                    if crate::faithful_gate("LAYER_PERIMETERS_SAME_MERGE_PATCHES")
                        && !same_merge_patches(&self.regions[region_idx], &self.regions[it])
                    {
                        continue;
                    }
                    // Layer.cpp:235-237
                    self.regions[it].perimeters.entities.clear();
                    self.regions[it].perimeter_islands.clear();
                    self.regions[it].fills.entities.clear();
                    self.regions[it].thin_fills.entities.clear();
                    // Layer.cpp:239-240
                    layerms.push(it);
                    done[it] = true;
                    // Layer.cpp:242-244: perimeter_regions.emplace_back — see note above.
                }

                if layerms.len() == 1 {
                    // Layer.cpp:247-252  (optimization)
                    // C++: (*layerm)->fill_surfaces.surfaces.clear();
                    //      (*layerm)->fill_no_overlap_expolygons.clear();
                    //      (*layerm)->make_perimeters((*layerm)->slices, perimeter_regions,
                    //          &(*layerm)->fill_surfaces, &(*layerm)->fill_no_overlap_expolygons, this->loop_nodes);
                    let surface_fill = self.regions[region_idx].slices.clone();
                    self.regions[region_idx].fill_surfaces.surfaces.clear();
                    self.regions[region_idx].fill_no_overlap_expolygons.clear();
                    // Layer.cpp:366 generate_region_perimeters(*layerm, (*layerm)->slices, ...) (R935 gate
                    // LAYER_GENERATE_REGION_PERIMETERS; off = R934, the no-patch arm for every region).
                    Self::run_region_perimeters(
                        &mut self.regions[region_idx], &surface_fill, region_idx, object_layers, height, id, print_z,
                    )?;
                    {
                        // Layer.cpp: make_perimeters pushes into this->loop_nodes;
                        // rebase the region-local node ids onto the layer vec.
                        let base = self.loop_nodes.len();
                        let mut nodes =
                            std::mem::take(&mut self.regions[region_idx].loop_nodes_out);
                        for n in &mut nodes {
                            n.node_id += base;
                        }
                        let (s0, e0) = self.regions[region_idx].perimeters.loop_node_range;
                        self.regions[region_idx].perimeters.loop_node_range =
                            (s0 + base, e0 + base);
                        // R932: the island collections' ranges likewise (PerimeterGenerator.cpp:1336).
                        for (_, coll) in self.regions[region_idx].perimeter_islands.iter_mut() {
                            coll.loop_node_range = (coll.loop_node_range.0 + base, coll.loop_node_range.1 + base);
                        }
                        self.loop_nodes.extend(nodes);
                    }
                    // Layer.cpp:254
                    // C++: (*layerm)->fill_expolygons = to_expolygons((*layerm)->fill_surfaces.surfaces);
                    // Already done inside LayerRegion::make_perimeters.
                } else {
                    // Layer.cpp:254-267
                    // C++: SurfaceCollection new_slices;
                    //      group slices according to number of extra perimeters,
                    //      merge each group with offset_ex(.., ClipperSafetyOffset).
                    let mut new_slices = SurfaceCollection::new();
                    {
                        // Layer.cpp:259-263
                        // C++: std::map<unsigned short, Surfaces> slices;
                        //      for (LayerRegion *layerm : layerms)
                        //          for (const Surface &surface : layerm->slices.surfaces)
                        //              slices[surface.extra_perimeters].emplace_back(surface);
                        use std::collections::BTreeMap;
                        let mut slices: BTreeMap<usize, Vec<Surface>> = BTreeMap::new();
                        for &lid in &layerms {
                            for surface in &self.regions[lid].slices.surfaces {
                                slices
                                    .entry(surface.extra_perimeters)
                                    .or_default()
                                    .push(surface.clone());
                            }
                        }
                        // Layer.cpp:264-266
                        // C++: for (.. surfaces_with_extra_perimeters : slices)
                        //          new_slices.append(offset_ex(surfaces_with_extra_perimeters.second,
                        //              ClipperSafetyOffset), surfaces_with_extra_perimeters.second.front());
                        for (_extra, group) in slices {
                            // offset_ex(surfaces, ClipperSafetyOffset) == union_safety_offset_ex.
                            let group_polys = crate::surface::to_polygons(&group);
                            let merged = crate::clipper_utils::union_safety_offset_ex(&group_polys);
                            // append(expolygons, template surface = group.front())
                            new_slices.append_expolygons_templ(merged, &group[0]);
                        }
                    }

                    // Layer.cpp:269-272: PerimeterRegion merge — see FIDELITY-NOTE above.

                    // Layer.cpp:275-278
                    // C++: SurfaceCollection fill_surfaces;  ExPolygons fill_no_overlap;
                    //      (*layerm)->make_perimeters(new_slices, perimeter_regions,
                    //          &fill_surfaces, &fill_no_overlap, this->loop_nodes);
                    // The reduced LayerRegion::make_perimeters writes into the
                    // region's own fill_surfaces / fill_no_overlap_expolygons; we
                    // snapshot those into the local fill_surfaces / fill_no_overlap
                    // used by the split below.
                    self.regions[region_idx].fill_surfaces.surfaces.clear();
                    self.regions[region_idx].fill_no_overlap_expolygons.clear();
                    // Layer.cpp:386 generate_region_perimeters(*layerm, new_slices, &fill_surfaces, ...) (R935 gate
                    // LAYER_GENERATE_REGION_PERIMETERS; off = R934).
                    Self::run_region_perimeters(
                        &mut self.regions[region_idx], &new_slices, region_idx, object_layers, height, id, print_z,
                    )?;
                    {
                        // Layer.cpp: make_perimeters pushes into this->loop_nodes;
                        // rebase the region-local node ids onto the layer vec.
                        let base = self.loop_nodes.len();
                        let mut nodes =
                            std::mem::take(&mut self.regions[region_idx].loop_nodes_out);
                        for n in &mut nodes {
                            n.node_id += base;
                        }
                        let (s0, e0) = self.regions[region_idx].perimeters.loop_node_range;
                        self.regions[region_idx].perimeters.loop_node_range =
                            (s0 + base, e0 + base);
                        // R932: the island collections' ranges likewise (PerimeterGenerator.cpp:1336).
                        for (_, coll) in self.regions[region_idx].perimeter_islands.iter_mut() {
                            coll.loop_node_range = (coll.loop_node_range.0 + base, coll.loop_node_range.1 + base);
                        }
                        self.loop_nodes.extend(nodes);
                    }
                    let fill_surfaces = self.regions[region_idx].fill_surfaces.clone();
                    let fill_no_overlap = self.regions[region_idx].fill_no_overlap_expolygons.clone();

                    // Layer.cpp:281-290
                    // C++: if (!fill_surfaces.surfaces.empty()) {
                    //          for (l : layerms) {
                    //              ExPolygons expp = intersection_ex(fill_surfaces.surfaces, (*l)->slices.surfaces);
                    //              (*l)->fill_expolygons = expp;
                    //              (*l)->fill_surfaces.set(std::move(expp), fill_surfaces.surfaces.front());
                    //              (*l)->fill_no_overlap_expolygons = intersection_ex((*l)->slices.surfaces, fill_no_overlap);
                    //          }
                    //      }
                    if !fill_surfaces.surfaces.is_empty() {
                        let fs_templ = fill_surfaces.surfaces[0].clone();
                        for &lid in &layerms {
                            let slice_expp =
                                crate::surface::to_expolygons(&self.regions[lid].slices.surfaces);
                            // intersection_ex(fill_surfaces.surfaces, (*l)->slices.surfaces)
                            let expp = crate::clipper_utils::intersection_ex(
                                &fill_surfaces.surfaces,
                                &slice_expp,
                            );
                            // intersection_ex((*l)->slices.surfaces, fill_no_overlap)
                            let no_overlap = crate::clipper_utils::intersection_ex(
                                &self.regions[lid].slices.surfaces,
                                &fill_no_overlap,
                            );
                            self.regions[lid].fill_expolygons = expp.clone();
                            self.regions[lid].fill_surfaces.set_expolygons_templ(expp, &fs_templ);
                            self.regions[lid].fill_no_overlap_expolygons = no_overlap;
                        }
                    }
                }
            }
        }

        Ok(())
    }

    // merge all regions' slices to get islands
    // Layer.cpp:47
    pub fn make_slices(&mut self) {
        // Layer.cpp:49-58
        let slices: ExPolygons = if self.regions.len() == 1 {
            // Layer.cpp:50-52
            // optimization: if we only have one region, take its slices
            // C++: slices = to_expolygons(m_regions.front()->slices.surfaces);
            crate::surface::to_expolygons(&self.regions[0].slices.surfaces)
        } else {
            // Layer.cpp:54-57
            // C++: Polygons slices_p;
            //      for (LayerRegion *layerm : m_regions)
            //          polygons_append(slices_p, to_polygons(layerm->slices.surfaces));
            //      slices = union_safety_offset_ex(slices_p);
            let mut slices_p: Polygons = Polygons::new();
            for layerm in &self.regions {
                slices_p.extend(crate::surface::to_polygons(&layerm.slices.surfaces));
            }
            crate::clipper_utils::union_safety_offset_ex(&slices_p)
        };

        // Layer.cpp:60-61
        // C++: this->lslices.clear();
        //      this->lslices.reserve(slices.size());
        self.lslices.clear();
        self.lslices.reserve(slices.len());

        // Layer.cpp:63-67
        // prepare ordering points
        // C++: Points ordering_points;
        //      ordering_points.reserve(slices.size());
        //      for (const ExPolygon &ex : slices)
        //          ordering_points.push_back(ex.contour.first_point());
        let mut ordering_points: Vec<Point> = Vec::with_capacity(slices.len());
        for ex in &slices {
            ordering_points.push(ex.contour.first_point());
        }

        // Layer.cpp:69-70
        // sort slices
        // C++: std::vector<Points::size_type> order = chain_points(ordering_points);
        // R111: the local `chain_points` (layer.rs) is a simplified NN-from-index-0
        // chain; native's `chain_points` (ShortestPath.cpp:1073) is
        // `chain_segments_greedy` (KD-tree greedy with a specific start). On
        // multi-island layers these produce DIFFERENT lslices orders, and since
        // GCode::process_layer emits islands in lslices order, the island EMISSION
        // order diverges (67/126 multi-island layers; island SETS identical). Route
        // through the faithful port (shortest_path::chain_segments_greedy) under
        // F1_UNION so the island order matches native. Default path keeps the local
        // chain (byte-unchanged).
        let order = if crate::faithful_gate("F1_UNION") {
            crate::shortest_path::chain_points(&ordering_points, None)
        } else {
            chain_points(&ordering_points)
        };

        // R709 — island ORDER, not the island set, is what costs the lines:
        // benchy Outer wall layers whose island order matches C++ score 94.4%
        // in-order, those that differ score 63.1% (ceiling +6,599 lines). Print
        // the ordering points and the resulting order so they can be compared
        // against the same probe in C++ (`Layer.cpp`, `MKSL`).
        if crate::probe_enabled("MKSL") {
            let pts: Vec<String> = ordering_points
                .iter()
                .map(|p| format!("{},{}", p.x, p.y))
                .collect();
            let ord: Vec<String> = order.iter().map(|i| i.to_string()).collect();
            eprintln!(
                "[MKSL] z={:.6} n={} pts={} order={}",
                self.print_z,
                ordering_points.len(),
                pts.join(";"),
                ord.join(",")
            );
        }

        // Layer.cpp:72-74
        // populate slices vector
        // C++: for (size_t i : order)
        //          this->lslices.emplace_back(std::move(slices[i]));
        for i in order {
            self.lslices.push(slices[i].clone());
        }
    }

    // Layer.cpp:77
    // C++: static inline bool layer_needs_raw_backup(const Layer *layer)
    //
    // GATE-NOTE (verified against BambuStudio Layer.cpp:77-82): the
    // elefant_foot_compensation read is COMMENTED OUT in the C++ source —
    // BambuStudio backs up raw slices unconditionally ("BBS: backup raw slice
    // for generating support"). The faithful port therefore keeps `true` as
    // the live return; the commented expression below is the disabled C++
    // line rendered with this crate's config-hierarchy reads.
    fn layer_needs_raw_backup(&self) -> bool {
        // BBS: backup raw slice for generating support
        // Layer.cpp:80 (disabled in C++):
        //return !(self.regions().len() == 1
        //    && (self.id() > 0 || self.object().config().elephant_foot_compensation == 0.0));
        // Layer.cpp:81
        true
    }

    // Layer.cpp:84
    pub fn backup_untyped_slices(&mut self) {
        // Layer.cpp:86 — layer_needs_raw_backup(this) is always true (Layer.cpp:81).
        if self.layer_needs_raw_backup() {
            // Layer.cpp:87-95
            // C++: for (LayerRegion *layerm : m_regions) {
            //          layerm->raw_slices = to_expolygons(layerm->slices.surfaces);
            //          layerm->raw_counter_circle_compensation.clear();
            //          layerm->raw_holes_circle_compensation.clear();
            //          for (Surface &surface : layerm->slices.surfaces) {
            //              layerm->raw_counter_circle_compensation.push_back(surface.counter_circle_compensation);
            //              layerm->raw_holes_circle_compensation.push_back(surface.holes_circle_compensation);
            //          }
            //      }
            // raw_slices stores the untyped expolygons of the current slices.
            for region in &mut self.regions {
                region.raw_slices = region.slices.clone();
                region.raw_counter_circle_compensation.clear();
                region.raw_holes_circle_compensation.clear();
                // FIDELITY-NOTE: Surface in this crate does not model the
                // per-surface counter_circle_compensation (bool) /
                // holes_circle_compensation (Vec<int>) flags (Surface.hpp:41-42);
                // the per-surface flag push_back loop is therefore a no-op here.
            }
        } else {
            // Layer.cpp:96-101
            // C++: assert(m_regions.size() == 1);
            //      m_regions.front()->raw_slices.clear();
            //      m_regions.front()->raw_counter_circle_compensation.clear();
            //      m_regions.front()->raw_holes_circle_compensation.clear();
            debug_assert_eq!(self.regions.len(), 1);
            if let Some(front) = self.regions.first_mut() {
                front.raw_slices.surfaces.clear();
                front.raw_counter_circle_compensation.clear();
                front.raw_holes_circle_compensation.clear();
            }
        }
    }

    // Layer.cpp:104
    pub fn restore_untyped_slices(&mut self) {
        // Layer.cpp:106 — layer_needs_raw_backup(this) is always true.
        if self.layer_needs_raw_backup() {
            // Layer.cpp:107-117
            // C++: for (LayerRegion *layerm : m_regions) {
            //          layerm->slices.set(layerm->raw_slices, stInternal);
            //          ... (per-surface circle_compensation copy)
            //      }
            for region in &mut self.regions {
                let raw = region.raw_slices.to_expolygons();
                region.slices.set(&raw, SurfaceType::Internal);
                // FIDELITY-NOTE: per-surface counter/holes circle_compensation
                // copy (Layer.cpp:110-115) is omitted — Surface does not model
                // those flags in this crate.
            }
        } else {
            // Layer.cpp:118-127
            // C++: assert(m_regions.size() == 1);
            //      m_regions.front()->slices.set(this->lslices, stInternal);
            debug_assert_eq!(self.regions.len(), 1);
            let lslices = self.lslices.clone();
            if let Some(front) = self.regions.first_mut() {
                front.slices.set(&lslices, SurfaceType::Internal);
            }
        }
    }

    // Similar to Layer::restore_untyped_slices()
    // Layer.cpp:134
    pub fn restore_untyped_slices_no_extra_perimeters(&mut self) {
        // Layer.cpp:136 — layer_needs_raw_backup(this) is always true.
        if self.layer_needs_raw_backup() {
            // Layer.cpp:137-140
            // C++: for (LayerRegion *layerm : m_regions)
            //          //if (! layerm->region().config().extra_perimeters.value)  // always false
            //              layerm->slices.set(layerm->raw_slices, stInternal);
            for region in &mut self.regions {
                let raw = region.raw_slices.to_expolygons();
                region.slices.set(&raw, SurfaceType::Internal);
            }
        } else {
            // Layer.cpp:141-146
            // C++: assert(m_regions.size() == 1);
            //      LayerRegion *layerm = m_regions.front();
            //      //if (! layerm->region().config().extra_perimeters.value)
            //          layerm->slices.set(this->lslices, stInternal);
            debug_assert_eq!(self.regions.len(), 1);
            let lslices = self.lslices.clone();
            if let Some(front) = self.regions.first_mut() {
                front.slices.set(&lslices, SurfaceType::Internal);
            }
        }
    }

    // Layer.cpp:150-170
    // C++: ExPolygons Layer::merged(float offset_scaled) const
    // The pin's function with its float argument: `offset(const Surfaces &, float)` gives Polygons
    // (ClipperUtils.cpp:563), `union_ex` of those, then `offset_ex(ExPolygons, float)` back when no
    // offset was asked for. EPSILON is rounded to a float both ways (Layer.cpp:156-157).
    pub fn merged_f32(&self, offset_scaled: f32) -> ExPolygons {
        use crate::support::support_common::cu;
        // Layer.cpp:152
        debug_assert!(offset_scaled >= 0.0);
        // Layer.cpp:153-158
        // If no offset is set, apply EPSILON offset before union, and revert it afterwards.
        let mut offset_scaled = offset_scaled;
        let mut offset_scaled2: f32 = 0.0;
        if offset_scaled == 0.0 {
            offset_scaled = crate::libslic3r::EPSILON as f32;
            offset_scaled2 = (-crate::libslic3r::EPSILON) as f32;
        }
        // Layer.cpp:159-165
        let mut polygons: Polygons = Polygons::new();
        for layerm in &self.regions {
            let config = layerm.region().config();
            // Our users learned to bend Slic3r to produce empty volumes to act as subtracters. Only add the region if it is non-empty.
            if config.bottom_solid_layers > 0
                || config.top_solid_layers > 0
                || config.fill_density > 0.0
                || config.perimeters > 0
            {
                let surf_ex = crate::surface::to_expolygons(&layerm.slices.surfaces);
                polygons.extend(cu::offset_expolygons(
                    &surf_ex,
                    offset_scaled,
                    cu::DEFAULT_JOIN_TYPE,
                    cu::DEFAULT_MITER_LIMIT,
                ));
            }
        }
        // Layer.cpp:166
        let mut out = cu::union_ex(&polygons);
        // Layer.cpp:167-168
        if offset_scaled2 != 0.0 {
            out = cu::offset_ex_expolygons(&out, offset_scaled2, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);
        }
        // Layer.cpp:169
        out
    }

    // Layer.cpp:150-170
    // C++: ExPolygons Layer::merged(float offset_scaled) const
    // The argument is a float at the pin; a double given here is rounded to one as a C++ caller's is.
    pub fn merged(&self, offset_scaled: f64) -> ExPolygons {
        self.merged_f32(offset_scaled as f32)
    }

    // The body `merged` had before R928 (double EPSILON, an offset per region into ExPolygons); nothing
    // calls it. Kept for comparison; `merged` above is the pin's function.
    pub fn merged_legacy(&self, offset_scaled: f64) -> ExPolygons {
        // Layer.cpp:152
        debug_assert!(offset_scaled >= 0.0);
        // Layer.cpp:153-158
        // If no offset is set, apply EPSILON offset before union, and revert it afterwards.
        let mut offset_scaled = offset_scaled;
        let mut offset_scaled2 = 0.0_f64;
        if offset_scaled == 0.0 {
            offset_scaled = crate::libslic3r::EPSILON;
            offset_scaled2 = -crate::libslic3r::EPSILON;
        }
        // Layer.cpp:159-165
        // C++: Polygons polygons;
        //      for (LayerRegion *layerm : m_regions) {
        //          const PrintRegionConfig &config = layerm->region().config();
        //          if (config.bottom_shell_layers > 0 || config.top_shell_layers > 0 ||
        //              config.sparse_infill_density > 0. || config.wall_loops > 0)
        //              append(polygons, offset(layerm->slices.surfaces, offset_scaled));
        //      }
        let mut polygons: ExPolygons = Vec::new();
        for layerm in &self.regions {
            let config = layerm.region().config();
            if config.bottom_solid_layers > 0
                || config.top_solid_layers > 0
                || config.fill_density > 0.0
                || config.perimeters > 0
            {
                let surf_ex = crate::surface::to_expolygons(&layerm.slices.surfaces);
                polygons.extend(crate::clipper_utils::offset_expolygons(
                    &surf_ex,
                    offset_scaled,
                    OffsetJoinType::Miter,
                ));
            }
        }
        // Layer.cpp:166
        // C++: ExPolygons out = union_ex(polygons);
        let mut out = union_ex(&polygons);
        // Layer.cpp:167-168
        // C++: if (offset_scaled2 != 0.f)
        //          out = offset_ex(out, offset_scaled2);
        if offset_scaled2 != 0.0 {
            out = crate::clipper_utils::offset_expolygons(&out, offset_scaled2, OffsetJoinType::Miter);
        }
        // Layer.cpp:169
        out
    }

    // Layer.cpp:39
    pub fn apply_auto_circle_compensation(&mut self) {
        // Layer.cpp:41-43
        // C++: for (LayerRegion *layerm : m_regions) {
        //          layerm->auto_circle_compensation(layerm->slices,
        //              this->object()->get_auto_circle_compenstaion_params(),
        //              scale_(this->object()->config().circle_compensation_manual_offset));
        //      }
        // FIDELITY-NOTE: LayerRegion::auto_circle_compensation,
        // PrintObject::get_auto_circle_compenstaion_params, and the
        // circle_compensation_manual_offset object-config field are not yet
        // ported (LayerRegion.cpp / PrintObject), so the per-region call body
        // is a no-op. The region-iteration skeleton mirrors C++ Layer.cpp:41.
        for _layerm in &mut self.regions {
            // layerm.auto_circle_compensation(...);  // not ported
        }
    }

    // C++: void Layer::calculate_perimeter_continuity(std::vector<LoopNode> &prev_nodes)
    // Layer.cpp:353-431. Links this layer's LoopNodes to the previous layer's
    // when a stretch of the previous contour of accumulated length >=
    // Continuitious_length (scale_(0.01) = 1000) stays within the width limit
    // of this node's contour. Distance = unsigned distance to the nearest
    // segment (native ContinuitiousDistancer signs it by side, but the
    // consumer only tests |dist| < width). NOTE native
    // `static const int dist_scale_threshold = 1.2` truncates to int 1 — the
    // width limit is exactly the previous width; ported faithfully.
    pub fn calculate_perimeter_continuity(&mut self, prev_nodes: &mut [LoopNode]) {
        // LAYER_CONTINUITY_PIN (on): Layer.cpp:468-547 as the pin has it, see
        // `calculate_perimeter_continuity_pin`. =0 keeps the body below.
        if crate::faithful_gate("LAYER_CONTINUITY_PIN") {
            return self.calculate_perimeter_continuity_pin(prev_nodes);
        }
        const CONTINUITIOUS_LENGTH: f64 = 1000.0; // scale_(0.01)

        // Layer.cpp:342-351 get_node_continuity_rang_limit (the ×1.2 truncates
        // away through the int constant).
        fn width_limit(widths: &[crate::Coord], idx: usize) -> f64 {
            if idx != 0 && idx < widths.len() {
                widths[idx] as f64
            } else {
                widths.first().copied().unwrap_or(0) as f64
            }
        }

        // Unsigned distance from `p` to the nearest segment of `pts`
        // (ContinuitiousDistancer, Layer.cpp:298-340; sign dropped — the only
        // consumer tests the absolute band).
        fn distance_from_perimeter(pts: &[crate::Point], p: &crate::Point) -> f64 {
            let (px, py) = (p.x as f64, p.y as f64);
            let mut best = f64::MAX;
            for i in 1..pts.len() {
                let (ax, ay) = (pts[i - 1].x as f64, pts[i - 1].y as f64);
                let (bx, by) = (pts[i].x as f64, pts[i].y as f64);
                let (dx, dy) = (bx - ax, by - ay);
                let l2 = dx * dx + dy * dy;
                let d2 = if l2 == 0.0 {
                    (px - ax).powi(2) + (py - ay).powi(2)
                } else {
                    let t = (((px - ax) * dx + (py - ay) * dy) / l2).clamp(0.0, 1.0);
                    (px - (ax + t * dx)).powi(2) + (py - (ay + t * dy)).powi(2)
                };
                if d2 < best {
                    best = d2;
                }
            }
            if best == f64::MAX {
                f64::MAX
            } else {
                best.sqrt()
            }
        }

        fn polyline_length(pts: &[crate::Point]) -> f64 {
            let mut len = 0.0;
            for i in 1..pts.len() {
                let dx = (pts[i].x - pts[i - 1].x) as f64;
                let dy = (pts[i].y - pts[i - 1].y) as f64;
                len += (dx * dx + dy * dy).sqrt();
            }
            len
        }

        // Layer.cpp:355
        for node_pos in 0..self.loop_nodes.len() {
            let node_pts = self.loop_nodes[node_pos].node_contour.pts.clone();
            let node_bbox = self.loop_nodes[node_pos].bbox.clone();
            let node_id = self.loop_nodes[node_pos].node_id;
            for prev_node in prev_nodes.iter_mut() {
                // Layer.cpp:361-363 — no overlap
                if !node_bbox.intersects(&prev_node.bbox) {
                    continue;
                }

                // Layer.cpp:366-371
                let mut continuitious_pl: Vec<crate::Point> = Vec::new();
                let mut flag = false;
                let mut end: i64 = prev_node.node_contour.pts.len() as i64 - 1;

                // Layer.cpp:373-395 — loop contour: walk backwards from the end
                if prev_node.node_contour.is_loop {
                    while end >= 0 {
                        if polyline_length(&continuitious_pl) >= CONTINUITIOUS_LENGTH {
                            self.loop_nodes[node_pos].lower_node_ids.push(prev_node.node_id);
                            prev_node.upper_node_ids.push(node_id);
                            flag = true;
                            break;
                        }
                        let pt = prev_node.node_contour.pts[end as usize];
                        let dist = distance_from_perimeter(&node_pts, &pt);
                        let width = width_limit(&prev_node.node_contour.widths, end as usize);
                        if dist < width && dist > -width {
                            continuitious_pl.insert(0, pt); // append_before
                        } else {
                            break;
                        }
                        end -= 1;
                    }
                    if flag || end < 0 {
                        continue;
                    }
                }

                // Layer.cpp:397-401
                let mut last_pt_idx = end;
                if !prev_node.node_contour.is_loop {
                    last_pt_idx += 1;
                }

                // Layer.cpp:403-424 — forward walk (continuitious_pl carries
                // over from the backward walk, per native).
                let mut start: i64 = 0;
                while start < last_pt_idx {
                    let pt = prev_node.node_contour.pts[start as usize];
                    let dist = distance_from_perimeter(&node_pts, &pt);
                    let width = width_limit(&prev_node.node_contour.widths, start as usize);
                    if dist < width && dist > -width {
                        continuitious_pl.push(pt);
                        start += 1;
                        continue;
                    }
                    if continuitious_pl.is_empty()
                        || polyline_length(&continuitious_pl) < CONTINUITIOUS_LENGTH
                    {
                        continuitious_pl.clear();
                        start += 1;
                        continue;
                    }
                    self.loop_nodes[node_pos].lower_node_ids.push(prev_node.node_id);
                    prev_node.upper_node_ids.push(node_id);
                    continuitious_pl.clear();
                    break;
                }

                // Layer.cpp:426-429
                if polyline_length(&continuitious_pl) >= CONTINUITIOUS_LENGTH {
                    self.loop_nodes[node_pos].lower_node_ids.push(prev_node.node_id);
                    prev_node.upper_node_ids.push(node_id);
                }
            }
        }
    }

    // Layer.cpp:435-460
    // C++: void Layer::recrod_cooling_node_for_each_extrusion()
    // Maps each perimeter entity of every region onto the merged_id of its
    // LoopNode (via the collection's loop_node_range and the nodes' loop_id =
    // entity index), storing it as the entity's cooling_node. The Rust port
    // keeps ONE flat perimeters collection per region (native nests one
    // collection per island), so the range walk runs over that collection.
    pub fn record_cooling_node_for_each_extrusion(&mut self) {
        self.recrod_cooling_node_for_each_extrusion()
    }

    // Layer.cpp:550-574
    // C++: void Layer::recrod_cooling_node_for_each_extrusion()
    // (the body described above; the C++ spells the name this way)
    pub fn recrod_cooling_node_for_each_extrusion(&mut self) {
        for region in &mut self.regions {
            let (range_start, range_end) = region.perimeters.loop_node_range;
            if range_start >= range_end {
                continue;
            }
            // R937 COOLING_NODE_ISLAND_PIN (on unless =0): the pin walks every island collection of
            // `region->perimeters` on its own (Layer.cpp:551-571): the island's walls before its second
            // node's loop take the merged id of the island's FIRST node, also when that node's loop is
            // not the island's first wall (PerimeterGenerator.cpp:648, loop_id = the index of the wall
            // the node is made for); `next_pos` is local to the island. The walk over the one flat
            // collection (=0) left those walls on the previous island's node (Benchy classic layers
            // 54-93: one `; COOLING_NODE: 8` where the pin writes 11, which moves the outer-wall speed
            // smoothing of GCode/Smoothing.cpp). The island's nodes are the consecutive nodes whose
            // loop lies in the island's walls (they are pushed island by island,
            // PerimeterGenerator.cpp:628-829, 1301-1336).
            let n_entities = region.perimeters.entities.len();
            if crate::faithful_gate("COOLING_NODE_ISLAND_PIN")
                && !region.perimeter_islands.is_empty()
                && region.perimeter_islands.iter().map(|(r, _)| r.len()).sum::<usize>() == n_entities
                && region.perimeter_islands.last().map(|(r, _)| r.end) == Some(n_entities)
            {
                let merged = |k: usize| -> i32 {
                    self.loop_nodes[k].merged_id.map(|v| v as i32).unwrap_or(-1)
                };
                region.entity_cooling_nodes = vec![-1; n_entities];
                let mut k = range_start;
                for (range, _) in &region.perimeter_islands {
                    let island_start = k;
                    while k < range_end
                        && self.loop_nodes[k].loop_id >= range.start
                        && self.loop_nodes[k].loop_id < range.end
                    {
                        k += 1;
                    }
                    let end = k;
                    let mut start = island_start;
                    // Layer.cpp:555-556
                    if start >= end {
                        continue;
                    }
                    let base = range.start as i64;
                    // Layer.cpp:558-560
                    let mut cooling_node = merged(start);
                    let mut next_pos: i64 =
                        if start + 1 < end { self.loop_nodes[start + 1].loop_id as i64 - base } else { -1 };
                    // Layer.cpp:561-569
                    for idx in 0..range.len() {
                        if idx as i64 == next_pos && next_pos > 0 {
                            start += 1;
                            cooling_node = merged(start);
                            next_pos =
                                if start + 1 < end { self.loop_nodes[start + 1].loop_id as i64 - base } else { -1 };
                        }
                        region.entity_cooling_nodes[range.start + idx] = cooling_node;
                    }
                }
                continue;
            }
            let mut start = range_start;
            let mut cooling_node: i32 = self.loop_nodes[start]
                .merged_id
                .map(|v| v as i32)
                .unwrap_or(-1);
            let mut next_pos: i64 = if start + 1 < range_end {
                self.loop_nodes[start + 1].loop_id as i64
            } else {
                -1
            };
            region.entity_cooling_nodes = vec![-1; region.perimeters.entities.len()];
            for idx in 0..region.perimeters.entities.len() {
                if idx as i64 == next_pos && next_pos > 0 {
                    start += 1;
                    cooling_node = self.loop_nodes[start]
                        .merged_id
                        .map(|v| v as i32)
                        .unwrap_or(-1);
                    next_pos = if start + 1 < range_end {
                        self.loop_nodes[start + 1].loop_id as i64
                    } else {
                        -1
                    };
                }
                region.entity_cooling_nodes[idx] = cooling_node;
            }
        }
    }

    // Layer.cpp:579
    // C++: coordf_t Layer::get_sparse_infill_max_void_area()
    pub fn get_sparse_infill_max_void_area(&self) -> Result<f64> {
        use crate::print_config::InfillPattern as Ip;
        // Layer.cpp:581
        let mut max_void_area = 0.0_f64;
        // Layer.cpp:582
        // C++: for (auto layerm : m_regions) {
        for layerm in &self.regions {
            // Layer.cpp:583
            // C++: Flow flow = layerm->flow(frInfill);  (uses m_layer->height)
            let flow = layerm.flow(FlowRole::Infill, self.height)?;
            // Layer.cpp:584
            // C++: float density = layerm->region().config().sparse_infill_density;
            let density = layerm.region().config().fill_density;
            // Layer.cpp:585
            // C++: InfillPattern pattern = layerm->region().config().sparse_infill_pattern;
            let pattern = layerm.region().config().fill_pattern;
            // Layer.cpp:586-587
            // C++: if (density == 0.) return -1;
            if density == 0.0 {
                return Ok(-1.0);
            }

            // Layer.cpp:589-590
            // BBS: rough estimation and need to be optimized
            // C++: double spacing = flow.scaled_spacing() * (100 - density) / density;
            let spacing = flow.scaled_spacing() as f64 * (100.0 - density) / density;
            // Layer.cpp:591-619
            match pattern {
                // Layer.cpp:592-603
                Ip::Concentric
                | Ip::Rectilinear
                | Ip::Line
                | Ip::Gyroid
                | Ip::AlignedRectilinear
                | Ip::OctagramSpiral
                | Ip::HilbertCurve
                | Ip::Honeycomb3D
                | Ip::ArchimedeanChords
                | Ip::Lattice2D => {
                    max_void_area = max_void_area.max(spacing * spacing);
                }
                // Layer.cpp:604-608
                Ip::Grid | Ip::Honeycomb | Ip::Lightning => {
                    max_void_area = max_void_area.max(4.0 * spacing * spacing);
                }
                // Layer.cpp:609-615
                Ip::Cubic | Ip::AdaptiveCubic | Ip::Triangles | Ip::Stars | Ip::SupportCubic => {
                    max_void_area = max_void_area.max(4.5 * spacing * spacing);
                }
                // Layer.cpp:616-618
                _ => {
                    max_void_area = max_void_area.max(spacing * spacing);
                }
            }
        }
        // Layer.cpp:621
        Ok(max_void_area)
    }

    /// Generate fill patterns for all regions
    /// Fill.cpp:586-768
    pub fn make_fills(
        &mut self,
        lower_internal_areas: &[crate::geometry::ExPolygon],
        lower_sparse_polys: &[crate::geometry::Polygon],
        lower_unsupport_faithful: Option<&[crate::geometry::ExPolygon]>,
    ) -> Result<()> {
        // Fill.cpp:228-229 — `layer.object()->layers().empty() ? 0. : ...front()->bottom_z()`:
        // a caller without the object's layer list gets the empty-list arm.
        self.make_fills_at_object_base(lower_internal_areas, lower_sparse_polys, lower_unsupport_faithful, 0.)
    }

    /// `make_fills` (Fill.cpp:586-768) with `object_base_z` of Fill.cpp:228-229
    /// (`layer.object()->layers().front()->bottom_z()`) handed over by the caller that
    /// holds the object's layers; `group_fills` reads it behind FILL_GROUP_PIN.
    pub fn make_fills_at_object_base(
        &mut self,
        lower_internal_areas: &[crate::geometry::ExPolygon],
        lower_sparse_polys: &[crate::geometry::Polygon],
        // Fill.cpp:734-745, built by the caller as C++ builds it
        // (FFC_LOWER_UNSUPPORT); None keeps the earlier construction below.
        lower_unsupport_faithful: Option<&[crate::geometry::ExPolygon]>,
        object_base_z: f64,
    ) -> Result<()> {
        // Fill.cpp:600
        // C++: const auto resolution = this->object()->print()->config().resolution.value;
        // Read up front through the layer's own print-config Arc (stamped by
        // wire_config_hierarchy) so no shared borrow overlaps the region
        // mutation below. Still underscore-bound: the downstream consumer
        // (params.resolution, Fill.cpp:705, used for path simplification) is
        // not ported yet.
        let resolution = self.object().print().config().resolution;

        // Fill.cpp:611 — f->layer_id = this->id() - this->object()->get_layer(0)->id().
        // Captured up front so no borrow of `self` overlaps the region mutation
        // below; consumed by the concentric fillers (FillBase.hpp:111).
        let fill_layer_id = self.id();

        // Fill.cpp:588-590
        // C++: for (LayerRegion *layerm : m_regions)
        //          layerm->fills.clear();
        for region in &mut self.regions {
            region.fills.entities.clear();
        }

        // Fill.cpp:594-596
        // C++: LockRegionParam lock_param;
        //      set_outlook_range(lock_param);
        //      std::vector<SurfaceFill> surface_fills = group_fills(*this, lock_param);
        let mut lock_param = crate::fill::LockRegionParam::default();
        // Fill.cpp:661-676, 687 — `set_outlook_range(lock_param)`; it reads only
        // regions whose sparse infill pattern is ipLockedZag. FILL_LOCKED_ZAG=0
        // leaves it out.
        if crate::faithful_gate("FILL_LOCKED_ZAG") {
            self.set_outlook_range(&mut lock_param);
        }
        let __gf_t = if crate::probe_enabled("FILLPROF") { Some(std::time::Instant::now()) } else { None };
        // FILL_GROUP_PIN: the list of the port of Fill.cpp:193-639 (fill/fill.rs, equal to the
        // pin on the vectors of tests/fill_native_parity.rs) in the consumers' form;
        // FILL_GROUP_PIN=0 is the earlier grouping of fill/mod.rs. R926: the three fixtures
        // come out with the same bytes in both arms.
        let surface_fills = if crate::faithful_gate("FILL_GROUP_PIN") {
            crate::fill::fill::group_fills_for_consumers(self, lower_internal_areas, &mut lock_param, object_base_z)?
        } else {
            crate::fill::group_fills(self, lower_internal_areas, &mut lock_param)?
        };
        // `PP=1`: the surface fills in the order make_fills walks them, `PPG ...`.
        if crate::stage_dump::pp_enabled() {
            for (k, sf) in surface_fills.iter().enumerate() {
                for (e, ex) in sf.expolygons.iter().enumerate() {
                    let (mut sx, mut sy, mut np) = (0i64, 0i64, ex.contour.points.len());
                    for p in &ex.contour.points {
                        sx += p.x;
                        sy += p.y;
                    }
                    for h in &ex.holes {
                        np += h.points().len();
                        for p in h.points() {
                            sx += p.x;
                            sy += p.y;
                        }
                    }
                    let f = ex.contour.points.first();
                    eprintln!(
                        "PPG L{} fill={} ex={} type={:?} pattern={:?} np={} sx={} sy={} first=({},{})",
                        self.id, k, e, sf.surface.surface_type, sf.params.pattern, np, sx, sy,
                        f.map_or(0, |p| p.x), f.map_or(0, |p| p.y)
                    );
                }
            }
        }
        if let Some(t) = __gf_t {
            FILLPROF_GROUP_NS.fetch_add(
                t.elapsed().as_nanos() as usize,
                std::sync::atomic::Ordering::Relaxed,
            );
        }

        // R714 — the fill ASSIGNMENT census, against C++ `Fill/Fill.cpp`'s
        // generation loop (FILLASN). R711/R712/R713 cleared the fill pattern's
        // parameters, the fill region and the connection structure, so the
        // remaining question is whether the two engines even hand the same
        // surfaces to the same fillers. Until R926 this crate reached
        // `crate::fill::group_fills` = `fill/mod.rs`, a DIVERGENT
        // reimplementation; since R926 the list is the port's of `fill/fill.rs`
        // (FILL_GROUP_PIN). `Layer::make_fills` itself is not a line-by-line port.
        if crate::probe_enabled("FILLASN") {
            for (i, sf) in surface_fills.iter().enumerate() {
                let area: f64 = sf.expolygons.iter().map(|e| e.area()).sum();
                // R715 — `regions` and `nov` matter for the concentric family:
                // `FillConcentricInternal` iterates `no_overlap_expolygons`, and
                // C++ `Fill.cpp:354` UNIONS that set across every region a fill
                // spans. Ours keeps only the first region's (a TODO at
                // `fill/mod.rs:942`), so a fill with regions>1 is where the two
                // engines can hand the filler different geometry.
                eprintln!(
                    "[FILLASN] z={:.3} i={} pat={:?} role={:?} n={} area={:.0} spacing={} angle={:.4} regions={} nov={}",
                    self.print_z,
                    i,
                    sf.params.pattern,
                    sf.params.extrusion_role,
                    sf.expolygons.len(),
                    area,
                    sf.params.spacing as i64,
                    sf.params.angle,
                    sf.region_id_group.len(),
                    sf.no_overlap_expolygons.len()
                );
                // R926 (diagnostics only): the fields of the params the consumers below read
                // and the first line does not show; FILL_GROUP_PIN on against off.
                eprintln!(
                    "[FILLASN2] z={:.3} i={} st={:?} th={:?} dens={:?} bridge={} ba={:?} al={:?} alm={:?} fw={:?} fh={:?} fs={:?} fb={} sp={:?} sym={} ml={} bas={:?}",
                    self.print_z,
                    i,
                    sf.surface.surface_type,
                    sf.surface.thickness,
                    sf.params.density,
                    sf.params.bridge,
                    sf.params.bridge_angle,
                    sf.params.anchor_length,
                    sf.params.anchor_length_max,
                    sf.params.flow.width(),
                    sf.params.flow.height(),
                    sf.params.flow.spacing(),
                    sf.params.flow.is_bridge(),
                    sf.params.sparse_pattern,
                    sf.params.symmetric_infill_y_axis,
                    sf.params.multiline,
                    sf.surface.bridge_angle
                );
            }
        }
        // Fill.cpp:597-598
        // C++: const Slic3r::BoundingBox bbox = this->object()->bounding_box();
        //      const auto resolution = this->object()->print()->config().resolution.value;
        let _bbox = crate::geometry::BoundingBox::empty(); // TODO: get from object

        // Fill.cpp:605-750
        // C++: for (SurfaceFill &surface_fill : surface_fills)
        let __fl_t = if crate::probe_enabled("FILLPROF") { Some(std::time::Instant::now()) } else { None };
        for surface_fill in surface_fills {
            // R778 — per-surface_fill census: role/pattern/expolygon count.
            if crate::probe_enabled("FILLSF") {
                eprintln!(
                    "FILLSF z={:.2} role={:?} pattern={:?} nexp={} bridge={}",
                    self.print_z,
                    surface_fill.params.extrusion_role,
                    surface_fill.params.pattern,
                    surface_fill.expolygons.len(),
                    surface_fill.params.bridge
                );
            }
            // Fill.cpp:607-632
            // Create the filler object
            // C++: std::unique_ptr<Fill> f = std::unique_ptr<Fill>(Fill::new_from_type(surface_fill.params.pattern));
            // TODO: Implement Fill factory (Fill::new_from_type)

            let fill_pattern = surface_fill.params.pattern;

            // FILL_CONCENTRIC_PIN (on unless =0): ipConcentric as group_fills left it, that is
            // the CONFIGURED pattern of a surface (Fill.cpp:239, 264-282), is filled by
            // FillConcentric through Fill::fill_surface_extrusion with params.use_arachne
            // (Fill.cpp:717-721, 797, 836-846; FillBase.cpp:127-176; FillConcentric.cpp:67-146):
            // one no-sort collection of variable-width paths per expolygon. The block below hands
            // it to FillConcentricInternal as it does ipConcentricInternal; that filler iterates
            // no_overlap_expolygons, which only the narrow block fills (Fill.cpp:599), and emits
            // nothing here.
            if fill_pattern == InfillPattern::Concentric
                && surface_fill.params.pin_pattern
                    == Some(crate::print_config::InfillPattern::Concentric)
                && crate::faithful_gate("FILL_CONCENTRIC_PIN")
            {
                let region_id = surface_fill.region_id;
                // Fill.cpp:787  f->loop_clipping = coord_t(scale_(nozzle_diameter) * LOOP_CLIPPING_LENGTH_OVER_NOZZLE_DIAMETER)
                let loop_clipping = ((surface_fill.params.flow.nozzle_diameter()
                    / crate::libslic3r::SCALING_FACTOR)
                    * 0.15) as crate::Coord;
                // Fill.cpp:789-803
                let mut fp = crate::fill::FillParams::new();
                fp.density = fill_params_density(surface_fill.params.density);
                fp.multiline = surface_fill.params.multiline;
                fp.dont_adjust = false;
                fp.anchor_length = surface_fill.params.anchor_length as f64;
                fp.anchor_length_max = surface_fill.params.anchor_length_max as f64;
                fp.use_arachne = true; // Fill.cpp:797
                fp.layer_height = self.height;
                fp.flow = surface_fill.params.flow.clone();
                fp.extrusion_role = surface_fill.params.extrusion_role;
                fp.using_internal_flow =
                    !surface_fill.surface.is_solid() && !surface_fill.params.bridge;
                let print_config = self
                    .print_config
                    .as_deref()
                    .expect("config hierarchy not wired — call wire_layer_hierarchy");
                let print_object_config = self
                    .object_config
                    .as_deref()
                    .expect("config hierarchy not wired — call wire_layer_hierarchy");
                let mut out_entities: Vec<crate::extrusion_entity::ExtrusionEntityType> =
                    Vec::new();
                // Fill.cpp:835-847
                for expoly in &surface_fill.expolygons {
                    let mut filler = crate::fill::fill_concentric::FillConcentric {
                        // Fill.cpp:844: the spacing is reset for each expolygon
                        spacing: surface_fill.params.spacing,
                        loop_clipping,
                        print_config: Some(print_config),
                        print_object_config: Some(print_object_config),
                    };
                    let mut surface = surface_fill.surface.clone();
                    surface.expolygon = expoly.clone();
                    // Fill::overlap stays 0 here (FillBase.hpp; only make_ironing sets it)
                    filler.fill_surface_extrusion(&surface, &fp, 0.0, &mut out_entities);
                }
                for ent in out_entities {
                    self.regions[region_id].fills.entities.push(ent);
                }
                continue;
            }

            // ---------------------------------------------------------------
            // Concentric (Arachne) fillers — dedicated variable-width path.
            //
            // ipConcentricInternal (-> InfillPattern::Concentric here, via the
            // config->fill mapping at fill/mod.rs:283) and ipFloatingConcentric
            // (-> InfillPattern::FloatingConcentric) are routed by
            // `detect_narrow_internal_solid_infill` (fill/mod.rs:777) onto the
            // narrow vertical-shell InternalSolid strips. In C++ these are filled
            // by FillConcentricInternal / FillFloatingConcentric, whose
            // `fill_surface_extrusion` produces VARIABLE-WIDTH Arachne beads
            // DIRECTLY (WallToolPaths), bypassing the single-width polyline path
            // (Fill.cpp:706,738-751). The generic polyline path below cannot
            // represent those beads, so handle these patterns here and `continue`.
            //
            // FloatingConcentric (ipFloatingConcentric) is filled by the faithful
            // FillFloatingConcentric, whose resplit_order_loops -> detect_floating_line
            // (Z-clipper) prunes/re-seeds the bead set vs the plain concentric path;
            // it needs the lower layer's unsupport areas + sparse anchor polygons
            // (Fill.cpp:638-675). Both are now ported.
            if matches!(
                fill_pattern,
                InfillPattern::Concentric | InfillPattern::FloatingConcentric
            ) {
                use crate::fill::fill_concentric_internal::FillConcentricInternal;
                use crate::fill::fill_floating_concentric::FillFloatingConcentric;

                let region_id = surface_fill.region_id;
                let is_floating = fill_pattern == InfillPattern::FloatingConcentric;

                // Fill.cpp:695 — f->loop_clipping = coord_t(scale_(nozzle_diameter)
                //   * LOOP_CLIPPING_LENGTH_OVER_NOZZLE_DIAMETER), const = 0.15
                //   (libslic3r.h:62).
                let loop_clipping =
                    crate::scale(surface_fill.params.flow.nozzle_diameter() * 0.15);

                // Fill.cpp:700-712 — the FillParams the filler reads.
                let mut fp = crate::fill::FillParams::new();
                fp.density = fill_params_density(surface_fill.params.density);
                fp.flow = surface_fill.params.flow.clone();
                fp.extrusion_role = surface_fill.params.extrusion_role;
                fp.use_arachne = true; // Fill.cpp:706
                fp.layer_height = self.height; // m_regions[..]->layer()->height
                fp.using_internal_flow =
                    !surface_fill.surface.is_solid() && !surface_fill.params.bridge;

                let print_config = self
                    .print_config
                    .as_deref()
                    .expect("config hierarchy not wired — call wire_layer_hierarchy");
                let print_object_config = self
                    .object_config
                    .as_deref()
                    .expect("config hierarchy not wired — call wire_layer_hierarchy");

                // Fill.cpp:642-675 — the floating filler's lower-layer data.
                // lower_layer_unsupport_areas = shrink_ex(union of lower
                // stInternal/stInternalVoid, SCALED_EPSILON). lower_internal_areas
                // is already that un-shrunk union (gathered by the caller).
                let lower_unsupport: ExPolygons = if let (true, Some(faithful)) =
                    (is_floating, lower_unsupport_faithful)
                {
                    faithful.to_vec()
                } else if is_floating && !lower_internal_areas.is_empty() {
                    let unioned = crate::clipper_utils::union_ex(lower_internal_areas);
                    crate::clipper_utils::shrink(
                        &unioned,
                        crate::libslic3r::SCALED_EPSILON / crate::SCALING_FACTOR,
                        crate::clipper_utils::OffsetJoinType::Miter,
                    )
                } else {
                    Vec::new()
                };
                let lower_sparse: Vec<crate::geometry::Polygon> = if is_floating {
                    lower_sparse_polys.to_vec()
                } else {
                    Vec::new()
                };

                // Placeholder surface — the fillers do not read the surface (they
                // iterate no_overlap_expolygons).
                let surface_for_fill = surface_fill.surface.clone();

                // Fill.cpp:738-749 — PER expoly: intersect no_overlap with that
                // single expoly, reset spacing, then fill_surface_extrusion.
                let mut out_entities: Vec<crate::extrusion_entity::ExtrusionEntityType> =
                    Vec::new();
                let fvs_dbg = crate::probe_enabled("FVS_DEBUG");
                if fvs_dbg {
                    FVS_EXPOLYS.fetch_add(
                        surface_fill.expolygons.len(),
                        std::sync::atomic::Ordering::Relaxed,
                    );
                }
                for expoly in &surface_fill.expolygons {
                    if expoly.contour.points.is_empty() {
                        continue;
                    }
                    // Fill.cpp:740 — f->no_overlap_expolygons =
                    //   intersection_ex(surface_fill.no_overlap_expolygons,
                    //                   {expoly}, ApplySafetyOffset::Yes).
                    // Both FillConcentricInternal (FillConcentricInternal.cpp:18) and
                    // FillFloatingConcentric (FillFloatingConcentric.cpp:941) iterate
                    // `no_overlap_expolygons` and nothing else, so an empty intersection
                    // means C++ emits NOTHING for this expolygon. R474: we used to
                    // substitute the whole (overlap-expanded) expoly instead, which
                    // manufactured floating-vertical-shell material C++ never lays.
                    // FVS_NO_OVERLAP_FAITHFUL=0 restores the old fallback.
                    let no_overlap = if surface_fill.no_overlap_expolygons.is_empty() {
                        if crate::faithful_gate("FVS_NO_OVERLAP_FAITHFUL") {
                            if fvs_dbg {
                                FVS_EMPTY_NOOVERLAP
                                    .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                            }
                            continue;
                        }
                        vec![expoly.clone()]
                    } else if crate::faithful_gate("FILL_NOOVERLAP_SAFETY") {
                        // Fill.cpp:740 passes ApplySafetyOffset::Yes; the plain
                        // `intersection` below drops it. R721 measured the fill
                        // input as bit-identical to C++ on only 2 of 391 records
                        // — the vertex counts and areas agree, the coordinates
                        // do not — which is what a missing safety offset on the
                        // clip paths looks like.
                        //
                        // PARKED DEFAULT-OFF (R721). This arm is the faithful
                        // one, but it scores in-order benchy −4 / arachne +12 /
                        // cube 0 / majora −72 = net −64 matched lines. A change
                        // that provably matches C++ and still scores negative
                        // means something downstream compensates for the missing
                        // offset; shipping it now would mask that. Turn on with
                        // FILL_NOOVERLAP_SAFETY=1 once the compensator is found.
                        crate::clipper_utils::intersection_clib_safety(
                            &surface_fill.no_overlap_expolygons,
                            std::slice::from_ref(expoly),
                        )
                    } else {
                        crate::clipper_utils::intersection(
                            &surface_fill.no_overlap_expolygons,
                            std::slice::from_ref(expoly),
                        )
                    };
                    if no_overlap.is_empty() {
                        continue;
                    }

                    let __fp = crate::probe_enabled("FILLPROF");
                    let __fp_t = if __fp { Some(std::time::Instant::now()) } else { None };
                    let __fp_v: usize = if __fp {
                        no_overlap.iter().map(|e| e.contour.points.len()
                            + e.holes.iter().map(|h| h.points.len()).sum::<usize>()).sum()
                    } else { 0 };
                    if is_floating {
                        let mut filler = FillFloatingConcentric {
                            spacing: surface_fill.params.spacing,
                            loop_clipping,
                            no_overlap_expolygons: no_overlap,
                            lower_layer_unsupport_areas: lower_unsupport.clone(),
                            lower_sparse_polys: lower_sparse.clone(),
                            print_config: Some(print_config),
                            print_object_config: Some(print_object_config),
                        };
                        filler.fill_surface_extrusion(&surface_for_fill, &fp, &mut out_entities);
                    } else {
                        let mut filler = FillConcentricInternal {
                            // Fill.cpp:611 — f->layer_id = this->id() - object->get_layer(0)->id()
                            layer_id: fill_layer_id,
                            // Fill.cpp:747 — f->spacing = surface_fill.params.spacing (mm)
                            spacing: surface_fill.params.spacing,
                            loop_clipping,
                            no_overlap_expolygons: no_overlap,
                            print_config: Some(print_config),
                            print_object_config: Some(print_object_config),
                        };
                        filler.fill_surface_extrusion(&surface_for_fill, &fp, &mut out_entities);
                    }
                    if let Some(t) = __fp_t {
                        use std::sync::atomic::Ordering::Relaxed;
                        let ns = t.elapsed().as_nanos() as usize;
                        if is_floating {
                            FILLPROF_FVS_NS.fetch_add(ns, Relaxed);
                            FILLPROF_FVS_N.fetch_add(1, Relaxed);
                            FILLPROF_FVS_V.fetch_add(__fp_v, Relaxed);
                        } else {
                            FILLPROF_CI_NS.fetch_add(ns, Relaxed);
                            FILLPROF_CI_N.fetch_add(1, Relaxed);
                            FILLPROF_CI_V.fetch_add(__fp_v, Relaxed);
                        }
                    }
                }

                for ent in out_entities {
                    self.regions[region_id].fills.entities.push(ent);
                }
                continue;
            }

            // Fill.cpp:678-693
            // Calculate spacing and link_max_length
            // C++: bool using_internal_flow = ! surface_fill.surface.is_solid() && ! surface_fill.params.bridge;
            let _using_internal_flow =
                !surface_fill.surface.is_solid() && !surface_fill.params.bridge;

            // C++: double link_max_length = 0.;
            // Fill.cpp:683-695 computes link_max_length in mm, then
            // f->link_max_length = scale_(link_max_length).
            let link_max_length_mm = if !surface_fill.params.bridge {
                // Fill.cpp:687-693
                if surface_fill.params.density > 80.0 {
                    // C++: link_max_length = 3. * f->spacing;
                    // LINK_MAX_LENGTH_FRESH_FILL (on unless =0): `f` was made by
                    // `Fill::new_from_type` a few lines up (Fill.cpp:699) and its
                    // `spacing` is still the constructor's 0. (FillBase.hpp:194);
                    // `f->spacing = surface_fill.params.spacing` comes after, at
                    // Fill.cpp:844. So the product is 0 and no link between two
                    // infill lines is ever too long. =0 takes the surface's spacing.
                    if crate::faithful_gate("LINK_MAX_LENGTH_FRESH_FILL") {
                        0.0
                    } else {
                        3.0 * surface_fill.params.spacing
                    }
                } else {
                    0.0
                }
            } else {
                0.0
            };
            let link_max_length = crate::scale(link_max_length_mm);

            let density = fill_params_density(surface_fill.params.density);
            let is_grid = fill_pattern == InfillPattern::Grid;
            // C++: FillParams::dont_connect() == (anchor_length_max < 0.05f)
            // (FillBase.hpp). The infill traversal connects lines whenever
            // dont_connect() is false; link_max_length only bounds the length
            // of an individual perimeter link, it does NOT disable connection.
            // Solid/bridge infill has anchor_length_max == 1000 (never
            // disconnected) and sparse infill uses infill_anchor_max (default
            // 12mm), so both connect here. Patterns like Line set
            // anchor_length_max == 0 to force dont_connect == true.
            let dont_connect = surface_fill.params.anchor_length_max < 0.05;
            // C++ make_fills leaves the main filler's `f->overlap` at its default 0
            // (Fill.cpp; only ironing sets it). The raster clip uses
            // `scale_(this->overlap - (0.5 - 0.45)*spacing)` with overlap==0
            // (FillRectilinear.cpp:2847). region_config.infill_overlap flows only
            // into `no_extrusion_overlap` (Fill.cpp:713), NOT this geometric offset.
            let infill_overlap = 0.0; // C++ Fill::overlap=0 for the main filler (FillBase.hpp:183, Fill.cpp:995/1007); infill_overlap flows only into no_extrusion_overlap (Fill.cpp:713), NOT this raster offset. rust's prior spacing*0.15 over-extended every infill line.
            let zag = zag_fill(&surface_fill.params, self.object_bounding_box, fill_layer_id, true);
            let infill_config = InfillConfig {
                pattern: fill_pattern,
                line_spacing: surface_fill.params.spacing,
                angle: surface_fill.params.angle as f64,
                angle_increment: 90.0,
                // R599: C++ Fill::_infill_direction (FillBase.cpp:224) uses the
                // surface's bridge angle when set. Radians, -1 == none. Populated
                // in the LIVE `group_fills` at fill/mod.rs:844 -- R599's comment
                // here named fill.rs:606, which belongs to the dead twin module.
                bridge_angle: surface_fill.params.bridge_angle as f64,
                density,
                extrusion_width: surface_fill.params.spacing,
                overlap: infill_overlap,
                connect_infill: !dont_connect,
                link_max_length,
                dont_adjust: false,
                consistent_pattern: zag.consistent_pattern,
                object_bounding_box: zag.object_bounding_box,
                horiz_move: zag.horiz_move,
                zigzag_rotate_step: zag.zigzag_rotate_step,
                symmetric_y_axis: zag.symmetric_y_axis,
                locked_zag: false,
            };

            // Fill.cpp:735-747
            // C++: LayerRegion* layerm = this->m_regions[surface_fill.region_id];
            //      for (ExPolygon& expoly : surface_fill.expolygons)
            let region_id = surface_fill.region_id;

            // FillMonotonicLineWGapFill (ipMonotonicLine) lays the monotonic
            // lines and then medial-axis gap-fills the band the lines left
            // uncovered (FillRectilinear.cpp:3260-3320). Capture the per-
            // surface no_overlap area and accumulate the footprint of the laid
            // lines so the gap-fill can run once, after the expoly loop.
            let is_monotonic_line = fill_pattern == InfillPattern::MonotonicLine;
            // C++ (Fill.cpp:740) intersects no_overlap with EACH expoly before
            // filling: f->no_overlap = intersection_ex(surface_fill.no_overlap, {expoly}).
            // Aggregate that here by intersecting with the surface's expolygons.
            let mono_no_overlap: ExPolygons = if is_monotonic_line
                && !surface_fill.no_overlap_expolygons.is_empty()
                && !surface_fill.expolygons.is_empty()
            {
                // Native intersection_ex runs ClipperLib @1e5; the geo variant
                // grids the clip contour to 1um and the raster endpoints clip
                // against it (R100 class). Gated full-res.
                if crate::faithful_gate("TOPFILL_FAITHFUL") {
                    crate::clipper_utils::intersection_clib(
                        &surface_fill.no_overlap_expolygons,
                        &surface_fill.expolygons,
                    )
                } else {
                    crate::clipper_utils::intersection(
                        &surface_fill.no_overlap_expolygons,
                        &surface_fill.expolygons,
                    )
                }
            } else {
                Vec::new()
            };
            // FILL_MONOTONIC_ROUTE_PIN (on unless =0): FillMonotonicLineWGapFill::
            // fill_surface_extrusion (FillRectilinear.cpp:3641-3753) per expoly, the gap fills
            // planned into the order of the lines (plan_monotonic_route / append_monotonic_route)
            // in the expoly's one no-sort collection, instead of one gap collection per region.
            // The islands of an expoly that laid no line keep the tail below (route_no_lines_areas).
            let route_gate = is_monotonic_line && crate::faithful_gate("FILL_MONOTONIC_ROUTE_PIN");
            let mut route_no_lines_areas: ExPolygons = Vec::new();
            let mono_flow = surface_fill.params.flow.clone();
            let mono_density = surface_fill.params.density;
            // Footprint (covered-by-spacing) of all monotonic lines emitted for
            // this surface_fill — C++ coll_nosort->polygons_covered_by_spacing(10).
            let mut mono_covered: Vec<crate::geometry::Polygon> = Vec::new();

            // R456: per-expolygon accounting for sparse (InternalInfill) fills — how much
            // Internal AREA is handed to the filler, and how much of it yields no
            // polylines at all. `FILL_SURFACE_DEBUG=1`.
            let fsdbg = crate::probe_enabled("FILL_SURFACE_DEBUG")
                && surface_fill.params.extrusion_role
                    == crate::extrusion_entity::ExtrusionRole::InternalInfill;
            // FillBase.cpp:97 — Fill::fill_surface() shrinks the surface before handing
            // it to _fill_surface_single:
            //     expp = offset_ex(surface->expolygon, scale_(this->overlap - 0.5 * this->spacing))
            // and then fills EACH resulting island separately. FillRectilinear bypasses
            // this (it re-offsets `surface->expolygon` itself in
            // ExPolygonWithOffset, FillRectilinear.cpp:2847), which is why our
            // rectilinear path already matches without it -- but FillGyroid does NOT,
            // so its fill region was a full half-spacing too wide and every boundary
            // arc Fill::connect_infill walks was correspondingly too long.
            // R473. Set GYROID_FILL_SHRINK=0 to restore the unshrunk region.
            // FILL_GYROID_SPACING_PIN (on unless =0): `f->spacing = surface_fill.params.spacing`
            // (Fill.cpp:844), the spacing group_fills set (for a sparse surface the flow's at the
            // object's layer height, Fill.cpp:305-328), not the spacing of the surface's own flow.
            // Only where group_fills of the pin made the list (pin_pattern set).
            let gyroid_spacing: f64 = if surface_fill.params.pin_pattern.is_some()
                && crate::faithful_gate("FILL_GYROID_SPACING_PIN")
            {
                surface_fill.params.spacing
            } else {
                surface_fill.params.flow.spacing()
            };
            let surface_expolygons = if fill_pattern == InfillPattern::Gyroid
                && crate::faithful_gate("GYROID_FILL_SHRINK")
            {
                // FillBase.cpp:102 float(scale_(this->overlap - 0.5 * this->spacing)), overlap 0 (Fill.cpp
                // never sets f->overlap): the division by 0.00001 and the float, behind faithful_gate
                // FILL_SCALE_DIV_PIN; off: the multiplication by 1e5 in double.
                let delta = if crate::faithful_gate("FILL_SCALE_DIV_PIN") {
                    (((-0.5f64).mul_add(gyroid_spacing, 0.0) / crate::libslic3r::SCALING_FACTOR) as f32) as f64
                } else {
                    -0.5 * gyroid_spacing * crate::SCALING_FACTOR
                };
                crate::clipper_utils::offset_expolygons_clib_scaled(
                    &surface_fill.expolygons,
                    delta,
                    crate::clipper_utils::OffsetJoinType::Miter,
                )
            } else {
                surface_fill.expolygons
            };
            for expoly in surface_expolygons {
                let dbg_area = if fsdbg {
                    expoly.area() / (crate::SCALING_FACTOR * crate::SCALING_FACTOR)
                } else {
                    0.0
                };
                if fsdbg {
                    use std::sync::atomic::Ordering::Relaxed;
                    crate::fill::SPARSE_IN_N.fetch_add(1, Relaxed);
                    crate::fill::SPARSE_IN_AREA.fetch_add((dbg_area * 1000.0) as usize, Relaxed);
                }
                // Skip empty polygons
                if expoly.contour.points.is_empty() {
                    if fsdbg {
                        use std::sync::atomic::Ordering::Relaxed;
                        crate::fill::SPARSE_EMPTY_N.fetch_add(1, Relaxed);
                    }
                    continue;
                }

                // Fill.cpp:838-841:
                //     if (params.symmetric_infill_y_axis) {
                //         params.symmetric_y_axis = f->extended_object_bounding_box().center().x();
                //         expoly.symmetric_y(params.symmetric_y_axis);
                //     }
                let mut expoly = expoly;
                if let Some(symmetric_y_axis) = zag.symmetric_y_axis {
                    expolygon_symmetric_y(&mut expoly, symmetric_y_axis);
                }

                // FillRectilinear.cpp:4007-4050 — `FillLockedZag::fill_surface_extrusion`:
                // the skin and the skeleton lines, each clipped to the regions of
                // its flows, in one collection that is sorted
                // (`eec->no_sort = this->no_sort()`, false).
                if zag.locked_zag {
                    let region_config = self.regions[region_id].region().config();
                    let locked = crate::fill::fill_rectilinear::fill_surface_locked_zag(
                        &lock_param,
                        region_config.locked_skin_infill_pattern,
                        region_config.locked_skeleton_infill_pattern,
                        &expoly,
                        &infill_config,
                        fill_layer_id,
                    );
                    if let Some(multi_width_polyline) = locked {
                        let mut collection = ExtrusionEntityCollection::new();
                        for (polylines, flow) in multi_width_polyline {
                            // calculate actual flow from spacing (which might have
                            // been adjusted by the infill pattern generator)
                            let mut flow_mm3_per_mm = flow.mm3_per_mm()?;
                            let mut flow_width = flow.width();
                            if !_using_internal_flow {
                                // FillRectilinear.cpp:4033-4035 — `this->spacing` is
                                // the locked zag's own, which no nested filler adjusts.
                                if let Ok(new_flow) = flow.with_spacing(surface_fill.params.spacing) {
                                    flow_mm3_per_mm = new_flow.mm3_per_mm()?;
                                    flow_width = new_flow.width();
                                }
                            }
                            extrusion_entities_append_paths(
                                &mut collection.entities,
                                polylines,
                                surface_fill.params.extrusion_role,
                                flow_mm3_per_mm,
                                flow_width as f32,
                                flow.height() as f32,
                            );
                        }
                        // FillRectilinear.cpp:4018-4021 — the collection is pushed
                        // where `multi_width_polyline` is not empty, with or
                        // without paths.
                        self.regions[region_id].fills.entities.push(
                            crate::extrusion_entity::ExtrusionEntityType::Collection(Box::new(
                                collection,
                            )),
                        );
                        continue;
                    }
                }

                // Save contour for connect_infill boundary before expoly may be moved
                let boundary_contour = expoly.contour.clone();
                let expoly_for_connect = expoly.clone();

                // Faithful FillGrid = combined raw multilines (make_fill_lines,
                // both sweeps over the shared offset base) + a single anchor-aware
                // connect_infill (fill_base, FillBase.cpp:1501) — C++
                // fill_surface_by_multilines. Only the GRID 2-direction path uses
                // this; single-direction rectilinear keeps fill_surface_by_lines.
                let emitter_pair_grid = is_grid;
                // FILL_MULTILINES_PIN (on unless =0): a region whose sparse pattern is ipTriangles,
                // ipStars or ipCubic is filled by FillTriangles / FillStars / FillCubic
                // (FillBase.cpp:46-48; FillRectilinear.cpp:3133-3165: three sweeps 60 degrees
                // apart, FillCubic's shifted by sqrt(0.5) * z). =0 lays the two sweeps of
                // FillGrid, which fill/mod.rs maps the three patterns to.
                let multilines_pattern: Option<crate::print_config::InfillPattern> =
                    match surface_fill.params.sparse_pattern {
                        Some(
                            p @ (crate::print_config::InfillPattern::Triangles
                            | crate::print_config::InfillPattern::Stars
                            | crate::print_config::InfillPattern::Cubic),
                        ) if is_grid && crate::faithful_gate("FILL_MULTILINES_PIN") => Some(p),
                        _ => None,
                    };
                let mut pair_boundary: Vec<crate::geometry::Polygon> = Vec::new();

                // R791 (gate GYROID_FSS) — the faithful FillGyroid::
                // _fill_surface_single: rotate the expolygon by −(angle +
                // CorrectionAngle), generate + clip + prune + CONNECT in the
                // rotated frame, rotate back (FillGyroid.cpp:151-210). The
                // legacy arm below generated and connected UNROTATED, which
                // moved every boundary-anchor vertex (R789/R790).
                let mut fss_connected = false;

                // FillRectilinear.cpp:3075-3081 — `FillRectilinear::fill_surface`:
                //     if (params.conformal) {
                //         Polylines conformal_out;
                //         if (FillConformal::fill_surface(*this, surface, params, conformal_out))
                //             return conformal_out;
                //     }
                // The fillers that run that method (it is virtual, and FillMonotonic,
                // FillGrid, FillSupportBase ... override it) are FillRectilinear,
                // FillAlignedRectilinear, FillZigZag and FillCrossZag
                // (FillBase.cpp:41-42, 62-63). `conformal_infill` is only set for
                // surfaces that are not solid (Fill.cpp:240), whose pattern is the
                // region's sparse infill pattern (Fill.cpp:239).
                // FIDELITY-NOTE: the skin pattern of ipLockedZag also goes through
                // FillRectilinear::fill_surface with `conformal` set
                // (FillRectilinear.cpp:3942-3967); `fill_surface_locked_zag` of this
                // crate lays the skin as lines.
                let mut conformal_polylines: Option<Vec<crate::geometry::Polyline>> =
                    conformal_fill_surface(
                        self.regions[region_id].region().config().fill_pattern,
                        &surface_fill.params,
                        &surface_fill.surface,
                        &expoly,
                        region_id,
                        fill_layer_id,
                        self.print_z,
                        density,
                        link_max_length,
                        resolution,
                        Some(_using_internal_flow),
                        &zag,
                    );

                // FILL_MONOTONIC_ROUTE_PIN: the no-overlap islands of this expoly and where
                // each island's lines end (FillRectilinear.cpp:3654-3668), set by the MonotonicLine arm.
                let route_pin = route_gate;
                let mut route_areas: ExPolygons = Vec::new();
                let mut route_island_end: Vec<usize> = Vec::new();
                let mut route_scan_angle: f32 = 0.0;
                let generated = match fill_pattern {
                    // FillRectilinear.cpp:3080 — `return conformal_out;`: the paths
                    // are final, nothing connects them again.
                    _ if conformal_polylines.is_some() => {
                        fss_connected = true;
                        conformal_polylines
                            .take()
                            .unwrap_or_default()
                            .into_iter()
                            .map(InfillPath::Line)
                            .collect()
                    }
                    InfillPattern::Gyroid if crate::faithful_gate("GYROID_FSS") => {
                        // R456: rust SurfaceFillParams.spacing carries the flow
                        // WIDTH here; native f->spacing receives the flow
                        // SPACING — flow.spacing() is the faithful source.
                        // Native f->angle is RADIANS (Fill.cpp deg2rad of
                        // infill_direction); rust SurfaceFillParams.angle is
                        // DEGREES (group_fills copies fill_angle raw).
                        let mut fill = crate::fill::fill_gyroid::FillGyroid::new(
                            (surface_fill.params.angle as f64).to_radians() as f32,
                            self.print_z,
                            gyroid_spacing,
                        );
                        let mut fp = crate::fill::FillParams::new();
                        fp.density = density;
                        fp.anchor_length = surface_fill.params.anchor_length as f64;
                        fp.anchor_length_max = surface_fill.params.anchor_length_max as f64;
                        let mut out: Vec<crate::geometry::Polyline> = Vec::new();
                        fill.fill_surface_single(
                            &fp,
                            1,
                            &(0.0f32, crate::geometry::Point::new(0, 0)),
                            expoly.clone(),
                            &mut out,
                        );
                        fss_connected = true;
                        if let Ok(v) = std::env::var("FSSOUT") {
                            if let Ok(zq) = v.parse::<f64>() {
                                if (self.print_z - zq).abs() < 1e-6 {
                                    for (pi, pl) in out.iter().enumerate() {
                                        let pts: Vec<String> = pl
                                            .points
                                            .iter()
                                            .map(|p| format!("({:.2},{:.2})", p.x as f64 * crate::libslic3r::SCALING_FACTOR, p.y as f64 * crate::libslic3r::SCALING_FACTOR))
                                            .collect();
                                        eprintln!("FSSOUT z={} pl={} n={} {}", zq, pi, pl.points.len(), pts.join(" "));
                                    }
                                }
                            }
                        }
                        out.into_iter().map(InfillPath::Line).collect()
                    }
                    _ if emitter_pair_grid => {
                        let (raw, boundary) = match multilines_pattern.and_then(|p| {
                            crate::fill::fill_rectilinear::generate_multilines_raw_lines(
                                &expoly,
                                &infill_config,
                                self.id as usize,
                                p,
                                self.print_z,
                            )
                        }) {
                            Some(lines_and_boundary) => lines_and_boundary,
                            None => crate::fill::fill_rectilinear::generate_grid_raw_lines(
                                &expoly,
                                &infill_config,
                                self.id as usize,
                            ),
                        };
                        pair_boundary = boundary;
                        raw.into_iter().map(InfillPath::Line).collect()
                    }
                    // FILL_HONEYCOMB_PIN (on unless =0): ipHoneycomb as group_fills left it is filled
                    // by FillHoneycomb through Fill::fill_surface (FillBase.cpp:99-113,
                    // FillHoneycomb.cpp:9-84) with the FillParams of Fill.cpp:789-797; =0 takes the
                    // generate_infill helper (density 0.2, no connection, no offset).
                    InfillPattern::Honeycomb
                        if surface_fill.params.pin_pattern
                            == Some(crate::print_config::InfillPattern::Honeycomb)
                            && crate::faithful_gate("FILL_HONEYCOMB_PIN") =>
                    {
                        fss_connected = true;
                        crate::fill::fill_rectilinear::generate_fill_honeycomb_pin(
                            &[expoly],
                            &infill_config,
                            self.id as usize,
                            density as f64,
                            surface_fill.params.multiline,
                            surface_fill.params.anchor_length as f64,
                            surface_fill.params.anchor_length_max as f64,
                        )
                    }
                    // FILL_LINE_PIN (on unless =0): ipLine as group_fills left it is filled by
                    // FillLine (FillBase.cpp:44; Fill::fill_surface FillBase.cpp:99-113,
                    // FillLine.cpp:10-120), not by the rectilinear filler fill/mod.rs maps it to.
                    InfillPattern::Rectilinear
                        if surface_fill.params.pin_pattern
                            == Some(crate::print_config::InfillPattern::Line)
                            && crate::fill::fill_line::fill_line_pin() =>
                    {
                        fss_connected = true;
                        crate::fill::fill_rectilinear::generate_fill_line_pin(
                            &[expoly],
                            &infill_config,
                            self.id as usize,
                        )
                    }
                    InfillPattern::Rectilinear | InfillPattern::Grid => {
                        // FillRectilinear.cpp:3074-3092 — `FillRectilinear::fill_surface`
                        // returns what `fill_surface_by_lines` laid: the lines are
                        // already linked along the contours
                        // (connect_segment_intersections_by_contours /
                        // traverse_graph_generate_polylines, FillRectilinear.cpp:2905,
                        // 2948), and `Fill::connect_infill` (FillBase.cpp:1501) is not
                        // run on them. RECT_FILL_NO_RECONNECT=0 runs it again.
                        if crate::faithful_gate("RECT_FILL_NO_RECONNECT") {
                            fss_connected = true;
                        }
                        generate_fill_rectilinear(
                            &[expoly],
                            &infill_config,
                            self.id as usize,
                            is_grid,
                        )
                    }
                    // Monotonic top/bottom/solid surfaces: FillMonotonic /
                    // FillMonotonicLine (FillRectilinear.hpp:47-62; their
                    // fill_surface, FillRectilinear.cpp:3090-3109). Both set
                    // params.monotonic = true; FillMonotonicLine *also* sets
                    // anchor_length_max = 0 => dont_connect()==true, so its lines
                    // are emitted SEPARATELY (native travels between them), whereas
                    // FillMonotonic keeps the perimeter links (anchor default).
                    // no_sort()==true for both => we skip the post connect_infill.
                    InfillPattern::Monotonic | InfillPattern::MonotonicLine => {
                        // FillMonotonicLine forces dont_connect (anchor_length_max=0).
                        let mut mono_config = infill_config.clone();
                        if fill_pattern == InfillPattern::MonotonicLine {
                            mono_config.connect_infill = false;
                            // Native FillMonotonicLineWGapFill (FillRectilinear.cpp:
                            // 3245-3247): params2.dont_adjust = true — nominal flow
                            // spacing + align_to_grid raster. Gated.
                            if crate::faithful_gate("TOPFILL_FAITHFUL") {
                                mono_config.dont_adjust = true;
                            }
                        }
                        // Native rasters over no_overlap_expolygons, NOT the surface
                        // expolygon (FillRectilinear.cpp:3253-3256); the WGapFill tail
                        // covers the unextruded band. Gated.
                        let mono_areas: Vec<crate::geometry::ExPolygon> =
                            if fill_pattern == InfillPattern::MonotonicLine
                                && !mono_no_overlap.is_empty()
                                && crate::faithful_gate("TOPFILL_FAITHFUL")
                            {
                                if crate::faithful_gate("FILL_NO_OVERLAP_SAFETY_PIN") {
                                    // Fill.cpp:837 `f->no_overlap_expolygons =
                                    // intersection_ex(surface_fill.no_overlap_expolygons,
                                    // ExPolygons() = {expoly}, ApplySafetyOffset::Yes)`: one
                                    // intersection of the group's no-overlap area with this
                                    // expoly, the clip grown by ClipperSafetyOffset first.
                                    crate::clipper_utils::intersection_clib_safety(
                                        &surface_fill.no_overlap_expolygons,
                                        &[expoly.clone()],
                                    )
                                } else {
                                    crate::clipper_utils::intersection_clib(
                                        &mono_no_overlap,
                                        &[expoly.clone()],
                                    )
                                }
                            } else if fill_pattern == InfillPattern::MonotonicLine
                                && mono_no_overlap.is_empty()
                                && crate::faithful_gate("TOPFILL_FAITHFUL")
                                && crate::faithful_gate("FILL_MONOLINE_NO_OVERLAP")
                            {
                                // FILL_MONOLINE_NO_OVERLAP (on unless =0):
                                // FillMonotonicLineWGapFill::fill_surface_extrusion iterates
                                // no_overlap_expolygons and nothing else
                                // (FillRectilinear.cpp:3657-3668): with none it lays no line
                                // (and no gap fill, 3693). The arm below rasters the surface's
                                // expolygon instead.
                                Vec::new()
                            } else {
                                vec![expoly.clone()]
                            };
                        if route_pin {
                            // FillRectilinear.cpp:3657-3668: the lines island by island, the
                            // invalid ones erased per island, `island_end` after each.
                            let mut out = Vec::new();
                            for area in &mono_areas {
                                out.extend(generate_fill_rectilinear_monotonic(
                                    std::slice::from_ref(area),
                                    &mono_config,
                                    self.id as usize,
                                    is_grid,
                                    true,
                                ));
                                route_island_end.push(out.len());
                            }
                            route_areas = mono_areas;
                            // FillRectilinear.cpp:3735 scan_angle = _infill_direction(surface).first
                            route_scan_angle = crate::fill::fill_rectilinear::live_monotonic_line_scan_angle(
                                &expoly,
                                &mono_config,
                                self.id as usize,
                            );
                            out
                        } else {
                            generate_fill_rectilinear_monotonic(
                                &mono_areas,
                                &mono_config,
                                self.id as usize,
                                is_grid,
                                true,
                            )
                        }
                    }
                    InfillPattern::Gyroid => {
                        // Use gyroid fill — generate paths for bounding box then clip to fill area
                        use crate::fill::fill_gyroid::{generate_gyroid_infill, GyroidConfig};
                        use crate::geometry::BoundingBox;
                        let mut bb = BoundingBox::empty();
                        for pt in &expoly.contour.points {
                            bb.merge_point(*pt);
                        }
                        // FillGyroid.cpp:166,177 — `this->spacing` is the flow SPACING,
                        // not its width (Fill::spacing is set from params.spacing, which
                        // Fill.cpp:281 derives via flow.spacing()). Passing the width made
                        // `distance = scale_(spacing)/density_adjusted` ~17% too large, so
                        // the gyroid waves were spaced too far apart (R456).
                        let raw_line_spacing = surface_fill.params.flow.spacing();
                        let gyroid_config = GyroidConfig {
                            z: self.print_z,
                            spacing: raw_line_spacing,
                            density: density,
                            angle: surface_fill.params.angle as f64,
                        };
                        let raw_polylines = generate_gyroid_infill(&gyroid_config, bb.min, bb.max);
                        // FillGyroid.cpp:186 — polylines = intersection_pl(polylines, expolygon).
                        // The previous hand-rolled clip tested `contains_point` on each
                        // polyline VERTEX: it ignored the expolygon's HOLES entirely and
                        // truncated every crossing at the last inside vertex instead of at
                        // the true boundary crossing. Gyroid waves are coarsely sampled, so
                        // that discarded a large share of each wave's length.
                        let clipped_pls =
                            crate::clipper_utils::intersection_pl(&raw_polylines, &[expoly.clone()]);
                        // R461 (GYROID_ENDPOINT_DEBUG=1): how far are the clipped
                        // endpoints from the expolygon they were clipped against?
                        // Computed directly against every edge, independent of EdgeGrid,
                        // to discriminate "the clip polygon differs from the one we hand
                        // the connector" from "EdgeGrid::closest_point cannot find them".
                        if crate::probe_enabled("GYROID_ENDPOINT_DEBUG") {
                            use std::sync::atomic::Ordering::Relaxed;
                            let mut edges: Vec<(crate::geometry::Point, crate::geometry::Point)> =
                                Vec::new();
                            let mut push_ring = |ring: &crate::geometry::Polygon| {
                                let pts = &ring.points;
                                for i in 0..pts.len() {
                                    edges.push((pts[i], pts[(i + 1) % pts.len()]));
                                }
                            };
                            push_ring(&expoly.contour);
                            for h in &expoly.holes {
                                push_ring(h);
                            }
                            // R471: also report WHICH edge is nearest and whether the
                            // point coincides with a ring vertex, so a far "crossing"
                            // can be identified as mis-classified rather than displaced.
                            let ring_pts: std::collections::HashSet<(i64, i64)> = edges
                                .iter()
                                .map(|(a, _)| (a.x, a.y))
                                .collect();
                            let nearest_edge = |p: &crate::geometry::Point| -> (f64, (i64,i64), (i64,i64)) {
                                let mut best = f64::MAX;
                                let mut be = ((0i64,0i64),(0i64,0i64));
                                for (a, b) in &edges {
                                    let (ax, ay) = (a.x as f64, a.y as f64);
                                    let (bx, by) = (b.x as f64, b.y as f64);
                                    let (px, py) = (p.x as f64, p.y as f64);
                                    let (dx, dy) = (bx - ax, by - ay);
                                    let l2 = dx * dx + dy * dy;
                                    let t = if l2 <= 0.0 { 0.0 } else { (((px-ax)*dx + (py-ay)*dy)/l2).clamp(0.0,1.0) };
                                    let (qx, qy) = (ax + t*dx, ay + t*dy);
                                    let d = ((px-qx).powi(2) + (py-qy).powi(2)).sqrt();
                                    if d < best { best = d; be = ((a.x,a.y),(b.x,b.y)); }
                                }
                                (best, be.0, be.1)
                            };
                            let dist_to_edges = |p: &crate::geometry::Point| -> f64 {
                                let mut best = f64::MAX;
                                for (a, b) in &edges {
                                    let (ax, ay) = (a.x as f64, a.y as f64);
                                    let (bx, by) = (b.x as f64, b.y as f64);
                                    let (px, py) = (p.x as f64, p.y as f64);
                                    let (dx, dy) = (bx - ax, by - ay);
                                    let l2 = dx * dx + dy * dy;
                                    let t = if l2 <= 0.0 {
                                        0.0
                                    } else {
                                        (((px - ax) * dx + (py - ay) * dy) / l2).clamp(0.0, 1.0)
                                    };
                                    let (qx, qy) = (ax + t * dx, ay + t * dy);
                                    let d = ((px - qx).powi(2) + (py - qy).powi(2)).sqrt();
                                    if d < best {
                                        best = d;
                                    }
                                }
                                best
                            };
                            // Was this endpoint an ORIGINAL wave vertex (so legitimately
                            // interior at the very start/end of a wave), or a crossing the
                            // clip produced (which MUST lie on the boundary)?
                            let raw_pts: std::collections::HashSet<(i64, i64)> = raw_polylines
                                .iter()
                                .flat_map(|pl| pl.points.iter())
                                .map(|p| (p.x, p.y))
                                .collect();
                            for pl in clipped_pls.iter() {
                                if pl.points.len() < 2 {
                                    continue;
                                }
                                // A correctly merged run spans many wave vertices; a
                                // 2-point output means the per-segment runs were never
                                // rejoined.
                                let b = match pl.points.len() {
                                    2 => 0,
                                    3..=4 => 1,
                                    5..=8 => 2,
                                    9..=16 => 3,
                                    _ => 4,
                                };
                                crate::fill::GEP_PTS_HIST[b]
                                    .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                                for p in [pl.points[0], *pl.points.last().unwrap()] {
                                    let is_raw = raw_pts.contains(&(p.x, p.y));
                                    if is_raw {
                                        crate::fill::GEP_RAWVERT.fetch_add(
                                            1,
                                            std::sync::atomic::Ordering::Relaxed,
                                        );
                                    }
                                    let d_um = dist_to_edges(&p) / crate::SCALING_FACTOR * 1000.0;
                                    crate::fill::GEP_N.fetch_add(1, Relaxed);
                                    crate::fill::GEP_SUM_UM.fetch_add(d_um as usize, Relaxed);
                                    // R470: split the distance by CLASS. A clip CROSSING
                                    // must lie on the boundary; an original raw-wave vertex
                                    // legitimately need not. If crossings measure ~0 the clip
                                    // is exact and only the raw-vertex endpoints are far.
                                    if is_raw {
                                        crate::fill::GEP_RAW_SUM_UM
                                            .fetch_add(d_um as usize, Relaxed);
                                    } else {
                                        crate::fill::GEP_CROSS_N.fetch_add(1, Relaxed);
                                        crate::fill::GEP_CROSS_SUM_UM
                                            .fetch_add(d_um as usize, Relaxed);
                                        let cb = if d_um < 1.0 { 0 } else if d_um < 10.0 { 1 }
                                            else if d_um < 100.0 { 2 } else { 3 };
                                        crate::fill::GEP_CROSS_HIST[cb].fetch_add(1, Relaxed);
                                        if d_um > 10.0
                                            && crate::fill::GEP_DUMPED.fetch_add(1, Relaxed) < 16
                                        {
                                            let (d, ea, eb) = nearest_edge(&p);
                                            eprintln!(
                                                "GEP_BAD pt=({},{}) dist={:.2}um nearest_edge=({},{})->({},{}) on_ring_vertex={} edge_len={:.1}um",
                                                p.x, p.y, d / crate::SCALING_FACTOR * 1000.0,
                                                ea.0, ea.1, eb.0, eb.1,
                                                ring_pts.contains(&(p.x, p.y)),
                                                (((eb.0-ea.0) as f64).hypot((eb.1-ea.1) as f64))
                                                    / crate::SCALING_FACTOR * 1000.0,
                                            );
                                        }
                                    }
                                    let b = if d_um < 1.0 {
                                        0
                                    } else if d_um < 10.0 {
                                        1
                                    } else if d_um < 100.0 {
                                        2
                                    } else if d_um < 1000.0 {
                                        3
                                    } else {
                                        4
                                    };
                                    crate::fill::GEP_HIST[b].fetch_add(1, Relaxed);
                                }
                            }
                        }
                        // FillGyroid.cpp:188-195 — drop very small bits, but keep lines that
                        // connect thin walls: minlength = scale_(0.8 * this->spacing).
                        let minlength = crate::scale(0.8 * raw_line_spacing);
                        clipped_pls
                            .into_iter()
                            .filter(|pl| pl.length() >= minlength as f64)
                            .map(InfillPath::Line)
                            .collect()
                    }
                    // All other patterns: dispatch through generate_infill which
                    // routes to the correct fill implementation
                    _ => {
                        let polylines = generate_infill(
                            fill_pattern,
                            &[expoly.clone()],
                            infill_config.line_spacing,
                            infill_config.angle,
                        )?;
                        polylines.into_iter().map(InfillPath::Line).collect()
                    }
                };

                if generated.is_empty() {
                    // FILL_ROUTE_NO_LINES_PIN (on unless =0): an expoly that laid no line
                    // still runs FillMonotonicLineWGapFill::fill_surface_extrusion's gap part
                    // (FillRectilinear.cpp:3686-3687 unextruded_areas = no_overlap_expolygons,
                    // new_flow = params.flow; 3695-3752): its gaps go into its own no-sort
                    // collection in the planner's leftover order, as the pin pushes one
                    // collection per expoly (Fill.cpp:837-841). =0 hands its islands to the
                    // region tail below.
                    if route_pin && crate::faithful_gate("FILL_ROUTE_NO_LINES_PIN") {
                        if !route_areas.is_empty() {
                            let mut collection = ExtrusionEntityCollection::new();
                            collection.no_sort = crate::faithful_gate("FILL_NO_SORT")
                                && !surface_fill.params.bridge;
                            let new_flow = surface_fill.params.flow.clone();
                            let line_mm3 = new_flow.mm3_per_mm().unwrap_or(0.0);
                            let gap_comp =
                                (self.object().config().monotonic_travel_into_wall * 0.01) as f32;
                            let mut no_lines: Vec<crate::geometry::Polyline> = Vec::new();
                            Self::monotonic_route_gap_fill(
                                &mut collection,
                                &route_areas,
                                &mut no_lines,
                                &route_island_end,
                                route_scan_angle,
                                &new_flow,
                                line_mm3,
                                surface_fill.params.flow.height() as f32,
                                mono_density,
                                resolution,
                                surface_fill.params.extrusion_role,
                                &surface_fill.params.flow,
                                gap_comp,
                            );
                            if crate::probe_enabled("FILL_ROUTE_NO_LINES_PROBE") {
                                eprintln!(
                                    "ROUTE_NO_LINES z={:.3} areas={} entities={}",
                                    self.print_z,
                                    route_areas.len(),
                                    collection.entities.len()
                                );
                            }
                            if !collection.entities.is_empty() {
                                self.regions[region_id].fills.entities.push(
                                    crate::extrusion_entity::ExtrusionEntityType::Collection(
                                        Box::new(collection),
                                    ),
                                );
                            }
                        }
                    } else {
                        route_no_lines_areas.append(&mut route_areas);
                    }
                    if fsdbg {
                        use std::sync::atomic::Ordering::Relaxed;
                        crate::fill::SPARSE_NOGEN_N.fetch_add(1, Relaxed);
                        crate::fill::SPARSE_NOGEN_AREA
                            .fetch_add((dbg_area * 1000.0) as usize, Relaxed);
                    }
                    continue;
                }

                let mut polylines = Vec::new();
                for path in generated {
                    match path {
                        InfillPath::Line(pl) => polylines.push(pl),
                        InfillPath::Loop(poly) => polylines.push(poly.split_at_first_point()),
                    }
                }

                if polylines.is_empty() {
                    // FILL_ROUTE_NO_LINES_PIN (on unless =0): an expoly that laid no line
                    // still runs FillMonotonicLineWGapFill::fill_surface_extrusion's gap part
                    // (FillRectilinear.cpp:3686-3687 unextruded_areas = no_overlap_expolygons,
                    // new_flow = params.flow; 3695-3752): its gaps go into its own no-sort
                    // collection in the planner's leftover order, as the pin pushes one
                    // collection per expoly (Fill.cpp:837-841). =0 hands its islands to the
                    // region tail below.
                    if route_pin && crate::faithful_gate("FILL_ROUTE_NO_LINES_PIN") {
                        if !route_areas.is_empty() {
                            let mut collection = ExtrusionEntityCollection::new();
                            collection.no_sort = crate::faithful_gate("FILL_NO_SORT")
                                && !surface_fill.params.bridge;
                            let new_flow = surface_fill.params.flow.clone();
                            let line_mm3 = new_flow.mm3_per_mm().unwrap_or(0.0);
                            let gap_comp =
                                (self.object().config().monotonic_travel_into_wall * 0.01) as f32;
                            let mut no_lines: Vec<crate::geometry::Polyline> = Vec::new();
                            Self::monotonic_route_gap_fill(
                                &mut collection,
                                &route_areas,
                                &mut no_lines,
                                &route_island_end,
                                route_scan_angle,
                                &new_flow,
                                line_mm3,
                                surface_fill.params.flow.height() as f32,
                                mono_density,
                                resolution,
                                surface_fill.params.extrusion_role,
                                &surface_fill.params.flow,
                                gap_comp,
                            );
                            if crate::probe_enabled("FILL_ROUTE_NO_LINES_PROBE") {
                                eprintln!(
                                    "ROUTE_NO_LINES z={:.3} areas={} entities={}",
                                    self.print_z,
                                    route_areas.len(),
                                    collection.entities.len()
                                );
                            }
                            if !collection.entities.is_empty() {
                                self.regions[region_id].fills.entities.push(
                                    crate::extrusion_entity::ExtrusionEntityType::Collection(
                                        Box::new(collection),
                                    ),
                                );
                            }
                        }
                    } else {
                        route_no_lines_areas.append(&mut route_areas);
                    }
                    if fsdbg {
                        use std::sync::atomic::Ordering::Relaxed;
                        crate::fill::SPARSE_NOGEN_N.fetch_add(1, Relaxed);
                        crate::fill::SPARSE_NOGEN_AREA
                            .fetch_add((dbg_area * 1000.0) as usize, Relaxed);
                    }
                    continue;
                }
                if fsdbg {
                    use std::sync::atomic::Ordering::Relaxed;
                    let l: f64 =
                        polylines.iter().map(|p| p.length()).sum::<f64>() / crate::SCALING_FACTOR;
                    crate::fill::SPARSE_OK_N.fetch_add(1, Relaxed);
                    crate::fill::SPARSE_OK_AREA.fetch_add((dbg_area * 1000.0) as usize, Relaxed);
                    crate::fill::SPARSE_OK_LEN.fetch_add((l * 1000.0) as usize, Relaxed);
                }

                // Connect separate infill lines into continuous paths
                // C++: Fill::connect_infill() - called after fill generation.
                // Monotonic fillers (FillMonotonic / FillMonotonicLine) set
                // no_sort()==true and emit their lines already in the final
                // sweep order WITHOUT cross-line connection; running
                // connect_infill on them would re-chain the lines into long
                // polylines (the divergence from native, which travels between
                // monotonic lines). Skip the connect step for those patterns.
                let is_monotonic = matches!(
                    fill_pattern,
                    InfillPattern::Monotonic | InfillPattern::MonotonicLine
                );
                if emitter_pair_grid {
                    // Faithful FillBase::connect_infill (FillBase.cpp:1501-1733):
                    // anchor-aware BoundaryInfillGraph + take_limited, run ONCE on
                    // the combined raw multilines, against polygons_outer (proven
                    // byte-exact vs C++ in the connect-replay unit case).
                    let mut fp = crate::fill::FillParams::new();
                    fp.density = density;
                    fp.anchor_length = surface_fill.params.anchor_length as f64;
                    fp.anchor_length_max = surface_fill.params.anchor_length_max as f64;
                    fp.dont_sort = false;
                    let bbox = crate::geometry::BoundingBox::from_points(&boundary_contour.points);
                    // FillRectilinear.cpp:3064-3069 — `if (params.dont_connect() || fill_lines.size() <= 1)`
                    // the lines are appended as they are: one line alone gets no anchors.
                    // FILL_MULTILINES_ONE_LINE=0 hands it to connect_infill as well.
                    // FIDELITY-NOTE: the `params.dont_connect()` arm with several lines
                    // (chain_polylines, FillRectilinear.cpp:3065-3066) is not taken here.
                    if !(polylines.len() <= 1 && crate::faithful_gate("FILL_MULTILINES_ONE_LINE")) {
                        let mut connected = Vec::new();
                        crate::fill::fill_base::connect_infill_polygons(
                            polylines,
                            &pair_boundary,
                            &bbox,
                            &mut connected,
                            infill_config.line_spacing,
                            &fp,
                        );
                        polylines = connected;
                    }
                    // R814 GRID_ODD_REVERSE (faithful ON): FillGrid::fill_surface
                    // reverses every polyline on odd layers
                    // (FillRectilinear.cpp:3127-3129).
                    // (FillGrid only: FillTriangles, FillStars and FillCubic return the lines
                    // as connected, FillRectilinear.cpp:3133-3165.)
                    if (self.id as usize) % 2 == 1 && multilines_pattern.is_none() && crate::faithful_gate("GRID_ODD_REVERSE") {
                        for pl in polylines.iter_mut() {
                            pl.points.reverse();
                        }
                    }
                } else if infill_config.connect_infill && !is_monotonic && !fss_connected {
                    let mut connected = Vec::new();
                    if crate::faithful_gate("FILL_CONNECT_FAITHFUL") {
                        // C++ Fill::fill_surface_single ends by calling
                        // Fill::connect_infill (FillBase.cpp:1501-1733), which builds a
                        // BoundaryInfillGraph over the FULL ExPolygon (contour + holes),
                        // snaps every infill end point onto the boundary and chains
                        // consecutive lines along it, honouring the configured anchors.
                        // R472: this faithful port is now the default for the
                        // non-monotonic fillers; set FILL_CONNECT_FAITHFUL=0 to fall
                        // back to the endpoint-merge stub.
                        let mut fp = crate::fill::FillParams::new();
                        fp.density = density;
                        fp.anchor_length = surface_fill.params.anchor_length as f64;
                        fp.anchor_length_max = surface_fill.params.anchor_length_max as f64;
                        fp.dont_sort = false;
                        let mut boundary_all = vec![expoly_for_connect.contour.clone()];
                        boundary_all.extend(expoly_for_connect.holes.iter().cloned());
                        let mut bb = crate::geometry::BoundingBox::empty();
                        for p in boundary_all.iter().flat_map(|g| g.points.iter()) {
                            bb.merge_point(*p);
                        }
                        crate::fill::fill_base::connect_infill_polygons(
                            polylines,
                            &boundary_all,
                            &bb,
                            &mut connected,
                            infill_config.line_spacing,
                            &fp,
                        );
                    } else {
                        let boundary = vec![boundary_contour.clone()];
                        let fill_params = crate::fill::FillParams::new();
                        crate::fill::connect_infill(
                            polylines,
                            &boundary,
                            infill_config.line_spacing,
                            &fill_params,
                            &mut connected,
                        );
                    }
                    polylines = connected;
                }

                if polylines.is_empty() {
                    continue;
                }

                // Convert to extrusion paths
                // R257 (gated): native fill_surface_extrusion recomputes flow
                // from the ADJUSTED solid spacing (`this->spacing`, written back
                // by _adjust_solid_spacing — FillBase.cpp:138-149 with_spacing).
                // Without it every adjusted solid fill extrudes a constant
                // ~0.5% low (the ×1.005 E class on XY-matched ISI rasters).
                let (fill_flow_eff, mm3_per_mm) = {
                    let adj = crate::fill::fill_rectilinear::ADJUSTED_SPACING_MM
                        .with(|c| c.get());
                    // FillBase.cpp:140-149 — the recompute is skipped ONLY when
                    // `params.using_internal_flow`, i.e. for sparse infill:
                    //   if (params.using_internal_flow) { /* ignore f->spacing */ }
                    //   else { Flow new_flow = params.flow.with_spacing(this->spacing); }
                    // using_internal_flow is `!surface.is_solid() && !params.bridge`, so
                    // C++ DOES recompute for bridges. We excluded them with
                    // `&& !params.bridge`, which is why our Bridge flow is bimodal (the
                    // two unadjusted branch constants) where C++'s is a continuum —
                    // `this->spacing` is adjusted per patch to fit a whole number of
                    // lines, so the recomputed width varies patch to patch (R492).
                    // BRIDGE_WITH_SPACING=0 restores the old exclusion.
                    let using_internal_flow =
                        !surface_fill.surface.is_solid() && !surface_fill.params.bridge;
                    let bridge_ok = !surface_fill.params.bridge
                        || crate::faithful_gate("BRIDGE_WITH_SPACING");
                    match adj {
                        Some(sp_mm)
                            if crate::faithful_gate("GAPTRIM_PRETAIL")
                                && !using_internal_flow
                                && bridge_ok =>
                        {
                            let nf = surface_fill
                                .params
                                .flow
                                .with_spacing(sp_mm)
                                .unwrap_or_else(|_| surface_fill.params.flow.clone());
                            let m = nf.mm3_per_mm().unwrap_or(0.0);
                            (nf, m)
                        }
                        _ => (
                            surface_fill.params.flow.clone(),
                            surface_fill.params.flow.mm3_per_mm()?,
                        ),
                    }
                };
                // FillRectilinear.cpp:3688-3691: the lines are handed to the planner as laid
                // (extrusion_entities_append_paths_with_wipe takes a copy).
                let mut route_lines: Vec<crate::geometry::Polyline> =
                    if route_pin { polylines.clone() } else { Vec::new() };
                let mut collection = ExtrusionEntityCollection::new();
                // R779 — faithful filler no_sort (FillBase.hpp:149 + overrides):
                // Concentric/ConcentricInternal/Monotonic/MonotonicLine(WGapFill)
                // return true; the rectilinear base (which native uses for BRIDGE
                // fills via ipRectilinear) returns false. The flag decides whether
                // GCode Region::append flattens the collection for the nearest-
                // neighbor fill chain (ISLAND_FLATTEN).
                // R780 — flipped default ON: without the flag, the emission-side
                // chained_path_from re-chained INSIDE concentric/monotonic
                // collections whose internal order native preserves (no_sort=true
                // fillers, FillBase.hpp:149 overrides). A/B: benchy-arachne content
                // +5,892 / in-order +1,481; Majora and benchy classic neutral.
                collection.no_sort = crate::faithful_gate("FILL_NO_SORT")
                    && !surface_fill.params.bridge
                    && matches!(
                        fill_pattern,
                        InfillPattern::Concentric
                            | InfillPattern::FloatingConcentric
                            | InfillPattern::Monotonic
                            | InfillPattern::MonotonicLine
                    );
                // FillBase.cpp:167-170 (and FillRectilinear.cpp, the paths with wipe) — the
                // height of the paths is `params.flow.height()`, the flow BEFORE
                // `with_spacing`: for a bridge `Flow::with_spacing` sets width and height to
                // the new diameter, and the pin hands over new_flow's width beside the
                // unadjusted height. FILL_PATH_HEIGHT_PIN=0 is the adjusted flow's height.
                let path_height = if crate::faithful_gate("FILL_PATH_HEIGHT_PIN") {
                    surface_fill.params.flow.height() as f32
                } else {
                    fill_flow_eff.height() as f32
                };
                if is_monotonic_line && crate::faithful_gate("TOPFILL_FAITHFUL") {
                    // Native FillMonotonicLineWGapFill emits via
                    // extrusion_entities_append_paths_with_wipe (FillRectilinear.cpp:
                    // 3274): short hops become non-extruding wipe connectors that
                    // (a) appear in the gcode as E-less G1s and (b) count in the
                    // coverage that shapes the WGapFill band (R146).
                    // Fill.cpp:233/636: gap_compensation_ratio =
                    // monotonic_travel_into_wall * 0.01 (profile sets 45%).
                    // Native ConfigOptionPercent stores the raw percent number
                    // (profile "45.0"); Fill.cpp:636 multiplies by 0.01.
                    let gap_comp =
                        (self.object().config().monotonic_travel_into_wall * 0.01) as f32;
                    crate::extrusion_entity::extrusion_entities_append_paths_with_wipe(
                        &mut collection.entities,
                        polylines,
                        surface_fill.params.extrusion_role,
                        mm3_per_mm,
                        fill_flow_eff.width() as f32,
                        path_height,
                        gap_comp,
                    );
                } else {
                    extrusion_entities_append_paths(
                        &mut collection.entities,
                        polylines,
                        surface_fill.params.extrusion_role,
                        mm3_per_mm,
                        fill_flow_eff.width() as f32,
                        path_height,
                    );
                }

                // Fill.cpp:832-833, FillBase.cpp:172-175 — the paths of a grid
                // may not be reversed when the fills are chained
                // (`params.can_reverse = false`, then `set_reverse()` on every
                // path); FillGrid turns them around itself on odd layers.
                // FILL_GRID_NO_REVERSE=0 leaves them reversible.
                if fill_pattern == InfillPattern::Grid
                    && crate::faithful_gate("FILL_GRID_NO_REVERSE")
                {
                    for ent in collection.entities.iter_mut() {
                        if let crate::extrusion_entity::ExtrusionEntityType::Path(path) = ent {
                            path.set_reverse();
                        }
                    }
                }

                // FillMonotonicLineWGapFill: record the footprint of the laid
                // monotonic lines so the post-loop gap-fill can subtract it from
                // the no_overlap area (FillRectilinear.cpp:3279
                // coll_nosort->polygons_covered_by_spacing(10)).
                if is_monotonic_line {
                    for ent in &collection.entities {
                        if let crate::extrusion_entity::ExtrusionEntityType::Path(path) = ent {
                            path.polygons_covered_by_spacing(&mut mono_covered, 10.0);
                        } else if let crate::extrusion_entity::ExtrusionEntityType::MultiPath(mp) =
                            ent
                        {
                            // ExtrusionEntity.hpp ExtrusionMultiPath::polygons_covered_by_spacing:
                            // every path in order (the multipaths of
                            // extrusion_entities_append_paths_with_wipe, EXTRUSION_PATHS_WITH_WIPE_PIN).
                            for path in &mp.paths {
                                path.polygons_covered_by_spacing(&mut mono_covered, 10.0);
                            }
                        }
                    }
                }

                // FILL_MONOTONIC_ROUTE_PIN: FillRectilinear.cpp:3692-3746 for this expoly.
                if route_pin && !route_areas.is_empty() {
                    let gap_comp = (self.object().config().monotonic_travel_into_wall * 0.01) as f32;
                    Self::monotonic_route_gap_fill(
                        &mut collection,
                        &route_areas,
                        &mut route_lines,
                        &route_island_end,
                        route_scan_angle,
                        &fill_flow_eff,
                        mm3_per_mm,
                        path_height,
                        mono_density,
                        resolution,
                        surface_fill.params.extrusion_role,
                        &surface_fill.params.flow,
                        gap_comp,
                    );
                }

                if !collection.entities.is_empty() {
                    self.regions[region_id].fills.entities.push(
                        crate::extrusion_entity::ExtrusionEntityType::Collection(Box::new(
                            collection,
                        )),
                    );
                }
            }

            // FillMonotonicLineWGapFill gap-fill tail (FillRectilinear.cpp:3260-3320).
            // After the monotonic lines are laid, medial-axis gap-fill the band of
            // the no_overlap area the lines left uncovered, emitting variable-width
            // erGapFill runs. These curved runs are what the arc-fitter turns into
            // G2/G3 on the top surface (native: 113 arcs; without this Rust got 4).
            if is_monotonic_line && std::env::var("NOOVDBG").is_ok() {
                let a_no: f64 = mono_no_overlap.iter().map(|e| e.area().abs()).sum::<f64>()
                    / (crate::SCALING_FACTOR * crate::SCALING_FACTOR);
                eprintln!(
                    "NOOVDBG-R z={:.2} n_no={} a_no={:.4}",
                    self.print_z,
                    mono_no_overlap.len(),
                    a_no
                );
            }
            // FILL_MONOTONIC_ROUTE_PIN: the tail runs only over the islands that laid no line.
            let mono_no_overlap = if route_gate { route_no_lines_areas } else { mono_no_overlap };
            if route_gate {
                mono_covered.clear();
            }
            if is_monotonic_line && mono_density >= 1.0 && !mono_no_overlap.is_empty() {
                use crate::clipper_utils::{
                    difference_clib, intersection_ex_expolygons_polygons, offset2_clib,
                    union_polygons_ex, OffsetJoinType,
                };
                use crate::extrusion_entity::{ExtrusionEntityType, ExtrusionRole};
                use crate::perimeter_generator::convert_thin_walls_to_extrusion_paths;

                // C++: ExPolygons unextruded_areas = diff_ex(no_overlap,
                //          union_ex(coll_nosort->polygons_covered_by_spacing(10)));
                // diff_ex via the vertex-exact vendored ClipperLib (difference_clib).
                let unextruded_areas: ExPolygons = if mono_covered.is_empty() {
                    mono_no_overlap.clone()
                } else {
                    let covered_ex = union_polygons_ex(&mono_covered);
                    difference_clib(&mono_no_overlap, &covered_ex)
                };

                // C++: gapfill_areas = union_ex(unextruded_areas);
                //      gapfill_areas = intersection_ex(gapfill_areas, no_overlap);
                let gapfill_areas = if crate::faithful_gate("TOPFILL_FAITHFUL") {
                    // Native intersection_ex @1e5 (mixed-grid guard, see above).
                    crate::clipper_utils::intersection_clib(&unextruded_areas, &mono_no_overlap)
                } else {
                    intersection_ex_expolygons_polygons(
                        &unextruded_areas,
                        &mono_no_overlap
                            .iter()
                            .flat_map(|ex| {
                                std::iter::once(ex.contour.clone())
                                    .chain(ex.holes.iter().cloned())
                            })
                            .collect::<Vec<_>>(),
                    )
                };

                if !gapfill_areas.is_empty() {
                    // C++: new_flow = params.flow.with_spacing(this->spacing) for
                    // solid fill; the top surface uses the surface flow directly.
                    let new_flow = mono_flow.clone();
                    // C++ works in SCALED units (new_flow.scaled_spacing()); the Rust
                    // clipper_utils offset/opening helpers operate in mm, so use the
                    // UNSCALED spacing here (mirrors the perimeter gap-fill block above,
                    // layer.rs:566-567 which uses flow.width()/spacing() in mm).
                    let spacing_mm = new_flow.spacing();
                    // C++: min = 0.2 * scaled_spacing * (1 - INSET_OVERLAP_TOLERANCE)
                    //      max = 2. * scaled_spacing   (INSET_OVERLAP_TOLERANCE = 0.4,
                    //      libslic3r.h:72 — 0.45 was WRONG here; a smaller min kept
                    //      thinner slivers alive and fragmented the gap band, R143).
                    //      Gated: default keeps the legacy 0.45 (byte-locked).
                    let min = if crate::faithful_gate("F1_UNION") {
                        0.2 * spacing_mm * (1.0 - 0.4)
                    } else {
                        0.2 * spacing_mm * (1.0 - 0.45)
                    };
                    let max = 2.0 * spacing_mm;
                    const CLIPPER_SAFETY_OFFSET: f64 = 0.0001;

                    // C++: gaps_ex = diff_ex(opening_ex(gapfill_areas, min/2),
                    //          offset2_ex(gapfill_areas, -max/2, max/2 + ClipperSafetyOffset));
                    // opening_ex(g, d) == offset2_ex(g, -d, +d); both opening_ex and
                    // offset2_ex here take no explicit join type, so DefaultJoinType =
                    // jtMiter, DefaultMiterLimit = 3.0 (ClipperUtils.hpp:31,37). Route
                    // the offsets + diff_ex through the vertex-exact vendored ClipperLib
                    // (offset2_clib / difference_clib) so the gap-band contours feeding
                    // medial_axis are not over-segmented (vs geo-clipper @ 1µm).
                    // R278: same native band-delta quantization as the perimeter
                    // gap block (doubles from trunc(spacing/1e-5)).
                    let (bmin, bmax) = if crate::faithful_gate("F1_UNION") {
                        let s_sc = (new_flow.spacing() / 0.00001).trunc();
                        (
                            0.2 * s_sc * (1.0 - 0.4) / crate::SCALING_FACTOR,
                            2.0 * s_sc / crate::SCALING_FACTOR,
                        )
                    } else {
                        (min, max)
                    };
                    let opened_min =
                        offset2_clib(&gapfill_areas, bmin / 2.0, bmin / 2.0, OffsetJoinType::Miter);
                    let wide_part = offset2_clib(
                        &gapfill_areas,
                        bmax / 2.0,
                        bmax / 2.0 + CLIPPER_SAFETY_OFFSET,
                        OffsetJoinType::Miter,
                    );
                    let mut gaps_ex = difference_clib(&opened_min, &wide_part);

                    if !gaps_ex.is_empty() {
                        // C++: ex.douglas_peucker(SCALED_RESOLUTION * 0.1) then
                        //      ex.medial_axis(min, max, &polylines).
                        let scaled_resolution = crate::scale(resolution) as f64;
                        let simplify_resolution = scaled_resolution * 0.1;
                        let mut polylines: crate::geometry::ThickPolylines = Vec::new();
                        for ex in &mut gaps_ex {
                            ex.douglas_peucker(simplify_resolution);
                            if crate::faithful_gate("MEDIALAXIS_NATIVE") {
                                // Same scaled-domain recipe as the perimeter gap
                                // block: doubles from trunc(spacing/1e-5).
                                let s_sc = (new_flow.spacing() / 0.00001).trunc();
                                ex.medial_axis_scaled(
                                    0.2 * s_sc * (1.0 - 0.4),
                                    2.0 * s_sc,
                                    &mut polylines,
                                );
                            } else {
                                ex.medial_axis(min, max, &mut polylines);
                            }
                        }

                        if !polylines.is_empty() {
                            // C++: variable_width(polylines, erGapFill, params.flow, ...)
                            let paths = convert_thin_walls_to_extrusion_paths(
                                &polylines,
                                ExtrusionRole::GapFill,
                                &mono_flow,
                            );
                            if !paths.is_empty() {
                                let mut coll = ExtrusionEntityCollection::new();
                                coll.no_sort = true;
                                for path in paths {
                                    coll.entities.push(ExtrusionEntityType::Path(path));
                                }
                                self.regions[region_id]
                                    .fills
                                    .entities
                                    .push(ExtrusionEntityType::Collection(Box::new(coll)));
                            }
                        }
                    }
                }
            }
        }
        if let Some(t) = __fl_t {
            FILLPROF_LOOP_NS.fetch_add(
                t.elapsed().as_nanos() as usize,
                std::sync::atomic::Ordering::Relaxed,
            );
        }


        // Fill.cpp:753-763
        // R778 FILLEEC — mirror of the cpp probe: per fills EEC, first role + count.
        if crate::probe_enabled("FILLEEC") {
            for region in &self.regions {
                for ent in &region.fills.entities {
                    if let crate::extrusion_entity::ExtrusionEntityType::Collection(c) = ent {
                        let role = c
                            .entities
                            .first()
                            .map(|e| format!("{:?}", crate::gcode::exporter::get_entity_role(e)))
                            .unwrap_or_default();
                        eprintln!("FILLEEC z={:.2} role={} n={}", self.print_z, role, c.entities.len());
                    }
                }
            }
        }

        // Add thin fill regions
        // C++: for (LayerRegion *layerm : m_regions)
        //          for (const ExtrusionEntity *thin_fill : layerm->thin_fills.entities)
        // Faithful Fill.cpp:757-762 — wrap each thin_fill (gap-fill) in its own
        // EEC and push it into `fills`, so gap-fill flows through the normal infill
        // island-assignment + chaining + emission path (interleaved per-island with
        // the other fills), instead of a separate batched thin_fills block.
        // (The previous code pushed an EMPTY collection — the thin_fill clone was
        // missing — a no-op that kept gap batched at 584 vs native 816.)
        // THIN_FILLS_CLONE (on unless =0): Fill.cpp:856-859 pushes
        // `thin_fill->clone()` and leaves `thin_fills` as it was. The earlier
        // arm moved them out, which the export (print.rs) relied on to not emit
        // them twice; with the gate on the export reads `fills` only, as
        // GCode.cpp does. =0 moves them.
        let thin_fills_clone = crate::faithful_gate("THIN_FILLS_CLONE");
        for region in &mut self.regions {
            let thins = if thin_fills_clone {
                region.thin_fills.entities.clone()
            } else {
                std::mem::take(&mut region.thin_fills.entities)
            };
            for thin_fill in thins {
                let mut collection = ExtrusionEntityCollection::new();
                collection.entities.push(thin_fill);
                region.fills.entities.push(
                    crate::extrusion_entity::ExtrusionEntityType::Collection(Box::new(collection)),
                );
            }
        }

        Ok(())
    }

    /// FillRectilinear.cpp:3670-3746, the part of
    /// `FillMonotonicLineWGapFill::fill_surface_extrusion` after the lines are laid into
    /// `collection`: the gap fills of the unextruded part of `route_areas` (the expoly's
    /// no-overlap islands) planned into the order of the lines (`plan_monotonic_route`) and
    /// the collection rebuilt in that order (`append_monotonic_route`). `new_flow` is the
    /// flow of 3672 (`params.flow`, or `params.flow.with_spacing(this->spacing)` when lines
    /// were laid, 3674-3685), `line_mm3_per_mm` and its width the line flow's; `gap_flow`
    /// is `params.flow`. With no line laid (`route_lines` empty, `island_end` all zero) the
    /// gaps are the planner's leftover, in its nearest-neighbour order (3446-3482).
    #[allow(clippy::too_many_arguments)]
    fn monotonic_route_gap_fill(
        collection: &mut ExtrusionEntityCollection,
        route_areas: &ExPolygons,
        route_lines: &mut Vec<crate::geometry::Polyline>,
        route_island_end: &[usize],
        route_scan_angle: f32,
        new_flow: &Flow,
        line_mm3_per_mm: f64,
        path_height: f32,
        density: f32,
        resolution: f64,
        extrusion_role: crate::extrusion_entity::ExtrusionRole,
        gap_flow: &Flow,
        gap_comp: f32,
    ) {
        use crate::clipper_utils::{difference_clib, offset2_clib, union_polygons_ex};
        use crate::extrusion_entity::ExtrusionEntityType;
        // FillRectilinear.cpp:3692 unextruded_areas = diff_ex(no_overlap_expolygons,
        //     union_ex(coll_nosort->polygons_covered_by_spacing(10))), 3694-3695.
        let mut covered: Vec<crate::geometry::Polygon> = Vec::new();
        for ent in &collection.entities {
            if let ExtrusionEntityType::Path(path) = ent {
                path.polygons_covered_by_spacing(&mut covered, 10.0);
            } else if let ExtrusionEntityType::MultiPath(mp) = ent {
                for path in &mp.paths {
                    path.polygons_covered_by_spacing(&mut covered, 10.0);
                }
            }
        }
        let unextruded_areas: ExPolygons = if covered.is_empty() {
            route_areas.clone()
        } else {
            difference_clib(route_areas, &union_polygons_ex(&covered))
        };
        // FillRectilinear.cpp:3699-3701.
        let gapfill_areas = crate::clipper_utils::intersection_clib(&unextruded_areas, route_areas);
        let mut gap_polylines: crate::geometry::ThickPolylines = Vec::new();
        if !gapfill_areas.is_empty() && density >= 1.0 {
            // FillRectilinear.cpp:3703-3707, min / max from new_flow.scaled_spacing()
            // (the doubles of trunc(spacing / 1e-5)).
            let s_sc = (new_flow.spacing() / 0.00001).trunc();
            let min_sc = 0.2 * s_sc * (1.0 - 0.4);
            let max_sc = 2.0 * s_sc;
            const CLIPPER_SAFETY_OFFSET: f64 = 0.0001;
            let opened_min = offset2_clib(
                &gapfill_areas,
                min_sc / 2.0 / crate::SCALING_FACTOR,
                min_sc / 2.0 / crate::SCALING_FACTOR,
                OffsetJoinType::Miter,
            );
            let wide_part = offset2_clib(
                &gapfill_areas,
                max_sc / 2.0 / crate::SCALING_FACTOR,
                max_sc / 2.0 / crate::SCALING_FACTOR + CLIPPER_SAFETY_OFFSET,
                OffsetJoinType::Miter,
            );
            let mut gaps_ex = difference_clib(&opened_min, &wide_part);
            // FillRectilinear.cpp:3708-3717: the gaps ordered by chain_points of their
            // contours' first points.
            let ordering_points: Vec<crate::geometry::Point> =
                gaps_ex.iter().map(|ex| ex.contour.points[0]).collect();
            let order = crate::shortest_path::chain_points(&ordering_points, None);
            let mut gaps_ex_sorted: ExPolygons = Vec::with_capacity(gaps_ex.len());
            for i in order {
                gaps_ex_sorted.push(std::mem::take(&mut gaps_ex[i]));
            }
            // FillRectilinear.cpp:3719-3723
            let simplify_resolution = crate::scale(resolution) as f64 * 0.1;
            for ex in &mut gaps_ex_sorted {
                ex.douglas_peucker(simplify_resolution);
                // ExPolygon::medial_axis (ExPolygon.cpp:263-371): the C++ `width` array kept
                // as it is, two per segment in scaled units, in `widths` (the pair layout
                // ThickPolyline::thicklines and reverse() read), which
                // thick_polyline_to_extrusion_paths_2 takes (VariableWidth.cpp:103-211).
                let mut pp: Vec<crate::geometry::MedialAxisThickPolyline> = Vec::new();
                ex.medial_axis_thick_polylines(min_sc, max_sc, &mut pp);
                for pl in pp {
                    if pl.points.len() < 2 || pl.width.is_empty() {
                        continue;
                    }
                    let mut tp = crate::geometry::ThickPolyline::with_capacity(pl.points.len());
                    tp.points = pl.points;
                    tp.widths = pl.width;
                    tp.endpoints = [pl.endpoints.0, pl.endpoints.1];
                    gap_polylines.push(tp);
                }
            }
            // FillRectilinear.cpp:3725-3734: tiny gaps dropped (filter_out_gap_fill is never
            // set by Fill.cpp: 0), none for a bridge.
            if !gap_polylines.is_empty() && !crate::extrusion_entity::is_bridge(extrusion_role) {
                // libslic3r.h SCALED_EPSILON = scale_(EPSILON), EPSILON = 1e-4.
                let scaled_epsilon = 1e-4 / 0.00001;
                gap_polylines.retain(|p| !(p.length() <= scaled_epsilon || p.length() < 0.0));
            } else {
                gap_polylines.clear();
            }
        }
        // FillRectilinear.cpp:3736-3746
        if !gap_polylines.is_empty() {
            collection.entities.clear();
            let flow_width = new_flow.width();
            let link_max = 3.0 * (((flow_width as f32) as f64 / 0.00001) as i64) as f64;
            let island_end: Vec<usize> = if route_island_end.last() == Some(&route_lines.len()) {
                route_island_end.to_vec()
            } else {
                vec![route_lines.len()]
            };
            let route = crate::fill::fill_rectilinear::plan_monotonic_route(
                route_lines,
                &mut gap_polylines,
                link_max,
                route_scan_angle,
                &island_end,
            );
            crate::fill::fill_rectilinear::append_monotonic_route(
                &mut collection.entities,
                &route,
                route_lines,
                &gap_polylines,
                extrusion_role,
                line_mm3_per_mm,
                flow_width as f32,
                path_height,
                crate::extrusion_entity::ExtrusionRole::GapFill,
                gap_flow,
                gap_comp,
            );
        }
    }

    /// Generate the sparse-infill polylines used to anchor internal bridges.
    ///
    /// 1:1 (faithful) port of
    /// `Layer::generate_sparse_infill_polylines_for_anchoring`
    /// (Fill/Fill.cpp:772-875). Consumed by `PrintObject::bridge_over_infill`
    /// (PrintObject.cpp:2391-2401 producer, :2845 consumer) where the returned
    /// polylines become the anchor lines that bound internal-bridge expansion.
    ///
    /// This is a const operation (it does not mutate `fill_surfaces` or
    /// `fills`): it re-derives the surface-fill groups via `group_fills`,
    /// processes ONLY `stInternal` surfaces, runs the same per-pattern fill
    /// dispatch as `make_fills`, and returns the raw polylines (no
    /// extrusion-entity conversion, no thin fills).
    ///
    /// Adaptive-cubic / lightning patterns are not ported; for those patterns
    /// no anchor lines are produced (the C++ filler would consult an octree /
    /// lightning generator we do not have). For the common grid / rectilinear /
    /// line / concentric / gyroid sparse-infill patterns this produces the same
    /// anchor polylines as native.
    pub fn generate_sparse_infill_polylines_for_anchoring(&self) -> Result<Vec<Polyline>> {
        // Fill.cpp:228-229: the empty-list arm for a caller without the object's layers.
        self.generate_sparse_infill_polylines_for_anchoring_at_object_base(0.)
    }

    /// `generate_sparse_infill_polylines_for_anchoring` (Fill.cpp:772-875) with `object_base_z`
    /// of Fill.cpp:228-229 handed over by the caller; `group_fills` reads it behind FILL_GROUP_PIN.
    pub fn generate_sparse_infill_polylines_for_anchoring_at_object_base(
        &self,
        object_base_z: f64,
    ) -> Result<Vec<Polyline>> {
        // NOTE: `surface_fill.params.pattern` is `crate::fill::InfillPattern`
        // (group_fills' type), which is the `InfillPattern` already imported at
        // the top of this module (layer.rs:23). Do NOT import
        // `crate::print_config::InfillPattern` here — it is a distinct enum.
        use crate::surface::SurfaceType;

        // Fill.cpp:774-775
        // C++: LockRegionParam skin_inner_param;
        //      std::vector<SurfaceFill> surface_fills = group_fills(*this, skin_inner_param);
        // group_fills only consults `lower_internal_areas` for stInternalSolid
        // (narrow-solid -> ipConcentricInternal) surfaces; this function emits
        // only stInternal surfaces, so an empty slice is faithful here.
        // R738: this generator's ONLY output comes from SurfaceFills whose
        // `surface_type == stInternal` (the `continue` just below, Fill.cpp:782).
        // `group_fills` never CREATES an Internal surface — the single
        // surface_type assignment in fill/mod.rs (:1143) converts Internal ->
        // InternalSolid for narrow areas, so Internal can only DECREASE. Hence a
        // layer with no Internal fill surface provably yields no anchor lines,
        // and the whole `group_fills` call is dead work.
        //
        // That is not a corner case: on benchy, layer 2 (the bottom solid shell)
        // has no Internal surface, and layer 3's snapshot pass ran this on it for
        // **0.1011 s of a 1.25 s slice (8%)** — 0.3645 s under CLIPPER_UNION_GEO,
        // where it was 57% of that gate's entire +0.636 s `fill_loop` cost — and
        // threw every bit of it away (lines=0).
        //
        // Provably-empty early-out, so bit-identical (R732). ANCHOR_GEN_EARLYOUT=0
        // restores the unconditional call.
        if crate::faithful_gate("ANCHOR_GEN_EARLYOUT")
            && !self.regions().iter().any(|lm| {
                lm.fill_surfaces
                    .surfaces
                    .iter()
                    .any(|s| s.surface_type == SurfaceType::Internal)
            })
        {
            return Ok(Vec::new());
        }

        let mut lock_param = crate::fill::LockRegionParam::default();
        // FILL_GROUP_PIN: as in make_fills.
        let surface_fills = if crate::faithful_gate("FILL_GROUP_PIN") {
            crate::fill::fill::group_fills_for_consumers(self, &[], &mut lock_param, object_base_z)?
        } else {
            crate::fill::group_fills(self, &[], &mut lock_param)?
        };

        // Fill.cpp:779
        // C++: Polylines sparse_infill_polylines{};
        let mut sparse_infill_polylines: Vec<Polyline> = Vec::new();

        // Fill.cpp:781
        // C++: for (SurfaceFill &surface_fill : surface_fills)
        for surface_fill in surface_fills {
            // Fill.cpp:782-784
            // C++: if (surface_fill.surface.surface_type != stInternal) continue;
            if surface_fill.surface.surface_type != SurfaceType::Internal {
                continue;
            }

            let fill_pattern = surface_fill.params.pattern;

            // Fill.cpp:786-812
            // C++: switch over pattern; ipCount / ipSupportBase -> continue.
            // The remaining patterns all break (fall through to generation).
            // Adaptive-cubic / support-cubic / lightning need an octree or
            // lightning generator that is not ported here, so they cannot
            // produce anchor lines -> skip them (no anchors, faithful to
            // "no adaptive/lightning" Benchy case). `fill::InfillPattern`
            // collapses ipAdaptiveCubic/ipSupportCubic into `Adaptive`.
            match fill_pattern {
                InfillPattern::Adaptive | InfillPattern::Lightning => continue,
                _ => {}
            }

            // Fill.cpp:826-836
            // C++: link_max_length = 0; if (!bridge && density > 80%) link_max_length = 3*spacing
            let link_max_length_mm = if !surface_fill.params.bridge {
                if surface_fill.params.density > 80.0 {
                    // LINK_MAX_LENGTH_FRESH_FILL: Fill.cpp:931 reads `f->spacing`
                    // before Fill.cpp:968 assigns it, as at Fill.cpp:779.
                    if crate::faithful_gate("LINK_MAX_LENGTH_FRESH_FILL") {
                        0.0
                    } else {
                        3.0 * surface_fill.params.spacing
                    }
                } else {
                    0.0
                }
            } else {
                0.0
            };
            let link_max_length = crate::scale(link_max_length_mm);

            // Fill.cpp:846-854 — FillParams. params.density = 0.01 * density.
            let density = fill_params_density(surface_fill.params.density);
            let is_grid = fill_pattern == InfillPattern::Grid;
            // Mirror make_fills' dont_connect derivation
            // (anchor_length_max < 0.05 disables connection; FillBase.hpp).
            let dont_connect = surface_fill.params.anchor_length_max < 0.05;
            let infill_overlap = 0.0; // C++ Fill::overlap=0 for the main filler (FillBase.hpp:183, Fill.cpp:995/1007); infill_overlap flows only into no_extrusion_overlap (Fill.cpp:713), NOT this raster offset. rust's prior spacing*0.15 over-extended every infill line.
            let zag = zag_fill(&surface_fill.params, self.object_bounding_box, self.id(), false);
            let infill_config = InfillConfig {
                pattern: fill_pattern,
                line_spacing: surface_fill.params.spacing,
                angle: surface_fill.params.angle as f64,
                angle_increment: 90.0,
                // R599: C++ Fill::_infill_direction (FillBase.cpp:224) uses the
                // surface's bridge angle when set. Radians, -1 == none. Populated
                // in the LIVE `group_fills` at fill/mod.rs:844 -- R599's comment
                // here named fill.rs:606, which belongs to the dead twin module.
                bridge_angle: surface_fill.params.bridge_angle as f64,
                density,
                extrusion_width: surface_fill.params.spacing,
                overlap: infill_overlap,
                connect_infill: !dont_connect,
                link_max_length,
                dont_adjust: false,
                consistent_pattern: zag.consistent_pattern,
                object_bounding_box: zag.object_bounding_box,
                horiz_move: zag.horiz_move,
                zigzag_rotate_step: zag.zigzag_rotate_step,
                symmetric_y_axis: zag.symmetric_y_axis,
                locked_zag: false,
            };

            // Fill.cpp:862-871
            // C++: for (ExPolygon &expoly : surface_fill.expolygons)
            //          polylines = f->fill_surface(&surface_fill.surface, params);
            //          sparse_infill_polylines.insert(..., polylines);
            // We mirror make_fills' polyline dispatch exactly, but accumulate
            // the polylines instead of converting them to extrusion entities.
            for expoly in surface_fill.expolygons {
                if expoly.contour.points.is_empty() {
                    continue;
                }

                let boundary_contour = expoly.contour.clone();

                // Fill.cpp:971 — `f->fill_surface(&surface, params)` is the very
                // filler make_fills uses, so a grid goes through
                // FillGrid::fill_surface here too: both sweeps as raw lines, one
                // connect_infill over the shared boundary, reversed on odd
                // layers (FillRectilinear.cpp:3115-3131). ANCHOR_GRID_FAITHFUL=0
                // keeps the separate rectilinear generator.
                if is_grid && crate::faithful_gate("ANCHOR_GRID_FAITHFUL") {
                    // FILL_ANCHOR_MULTILINES_PIN (on unless =0): Fill.cpp:912 creates the
                    // region's own filler, so ipTriangles / ipStars / ipCubic are filled by
                    // FillTriangles / FillStars / FillCubic (FillRectilinear.cpp:3133-3165,
                    // three sweeps 60 degrees apart, FillCubic shifted by sqrt(0.5) * z),
                    // returned as connected (no odd-layer reversal, FillGrid only), as
                    // make_fills does behind FILL_MULTILINES_PIN. =0 lays FillGrid's two
                    // sweeps, which fill/mod.rs maps the three patterns to.
                    let anchor_multilines: Option<crate::print_config::InfillPattern> =
                        match surface_fill.params.sparse_pattern {
                            Some(
                                p @ (crate::print_config::InfillPattern::Triangles
                                | crate::print_config::InfillPattern::Stars
                                | crate::print_config::InfillPattern::Cubic),
                            ) if crate::faithful_gate("FILL_MULTILINES_PIN")
                                && crate::faithful_gate("FILL_ANCHOR_MULTILINES_PIN") =>
                            {
                                Some(p)
                            }
                            _ => None,
                        };
                    let (raw, pair_boundary) = match anchor_multilines.and_then(|p| {
                        crate::fill::fill_rectilinear::generate_multilines_raw_lines(
                            &expoly,
                            &infill_config,
                            self.id as usize,
                            p,
                            self.print_z,
                        )
                    }) {
                        Some(lines_and_boundary) => lines_and_boundary,
                        None => crate::fill::fill_rectilinear::generate_grid_raw_lines(
                            &expoly,
                            &infill_config,
                            self.id as usize,
                        ),
                    };
                    // FillRectilinear.cpp:3064-3069: one line alone is appended as it is.
                    // FILL_ANCHOR_ONE_LINE_PIN (on unless =0); =0 connects it as before.
                    if raw.len() <= 1 && crate::faithful_gate("FILL_ANCHOR_ONE_LINE_PIN") {
                        let mut raw = raw;
                        if (self.id as usize) % 2 == 1
                            && anchor_multilines.is_none()
                            && crate::faithful_gate("GRID_ODD_REVERSE")
                        {
                            for pl in raw.iter_mut() {
                                pl.points.reverse();
                            }
                        }
                        sparse_infill_polylines.extend(raw);
                        continue;
                    }
                    let mut fp = crate::fill::FillParams::new();
                    fp.density = density;
                    fp.anchor_length = surface_fill.params.anchor_length as f64;
                    fp.anchor_length_max = surface_fill.params.anchor_length_max as f64;
                    fp.dont_sort = false;
                    let bbox = crate::geometry::BoundingBox::from_points(&boundary_contour.points);
                    let mut connected = Vec::new();
                    crate::fill::fill_base::connect_infill_polygons(
                        raw,
                        &pair_boundary,
                        &bbox,
                        &mut connected,
                        infill_config.line_spacing,
                        &fp,
                    );
                    if (self.id as usize) % 2 == 1
                        && anchor_multilines.is_none()
                        && crate::faithful_gate("GRID_ODD_REVERSE")
                    {
                        for pl in connected.iter_mut() {
                            pl.points.reverse();
                        }
                    }
                    sparse_infill_polylines.extend(connected);
                    continue;
                }

                // FVS_ANCHOR_GYROID_FSS (on): Fill.cpp:969-971 runs the real filler,
                // `f->fill_surface(&surface, params)`: Fill::fill_surface shrinks the
                // surface by half a spacing (FillBase.cpp, `offset_ex(.., overlap -
                // 0.5 * spacing)`) and FillGyroid::_fill_surface_single generates,
                // clips and CONNECTS the waves in the rotated frame — the same path
                // make_fills' gyroid takes (GYROID_FILL_SHRINK, GYROID_FSS). The arm
                // below connected unshrunk, unrotated lines with default anchor
                // parameters, so the floating vertical shell's anchor polygons
                // covered 61% of the oracle's. =0 keeps that arm.
                if fill_pattern == InfillPattern::Gyroid
                    && crate::faithful_gate("FVS_ANCHOR_GYROID_FSS")
                {
                    // FILL_ANCHOR_GYROID_SPACING_PIN (R939): Fill.cpp:968 `f->spacing =
                    // surface_fill.params.spacing` (for a sparse surface the flow's at the object's
                    // layer height, Fill.cpp:305-328) and FillBase.cpp:102 `float(scale_(this->overlap -
                    // 0.5 * this->spacing))` (a division by 0.00001, then float), as make_fills' gyroid
                    // takes them (FILL_GYROID_SPACING_PIN, FILL_SCALE_DIV_PIN); =0 keeps the surface
                    // flow's spacing and the multiplication by 1e5.
                    let anchor_gyroid_pin = crate::faithful_gate("FILL_ANCHOR_GYROID_SPACING_PIN");
                    let spacing = if anchor_gyroid_pin && surface_fill.params.pin_pattern.is_some() {
                        surface_fill.params.spacing
                    } else {
                        surface_fill.params.flow.spacing()
                    };
                    let delta = if anchor_gyroid_pin {
                        (((-0.5f64).mul_add(spacing, 0.0) / crate::libslic3r::SCALING_FACTOR) as f32) as f64
                    } else {
                        -0.5 * spacing * crate::SCALING_FACTOR
                    };
                    let islands = crate::clipper_utils::offset_expolygons_clib_scaled(
                        std::slice::from_ref(&expoly),
                        delta,
                        crate::clipper_utils::OffsetJoinType::Miter,
                    );
                    let mut fp = crate::fill::FillParams::new();
                    fp.density = density;
                    fp.anchor_length = surface_fill.params.anchor_length as f64;
                    fp.anchor_length_max = surface_fill.params.anchor_length_max as f64;
                    for island in islands {
                        let mut fill = crate::fill::fill_gyroid::FillGyroid::new(
                            (surface_fill.params.angle as f64).to_radians() as f32,
                            self.print_z,
                            spacing,
                        );
                        let mut out: Vec<crate::geometry::Polyline> = Vec::new();
                        fill.fill_surface_single(
                            &fp,
                            1,
                            &(0.0f32, crate::geometry::Point::new(0, 0)),
                            island,
                            &mut out,
                        );
                        sparse_infill_polylines.extend(out);
                    }
                    continue;
                }

                // Fill.cpp:911-971 — the same filler as make_fills, so
                // FillRectilinear::fill_surface takes the conformal fill here too
                // (FillRectilinear.cpp:3077-3081; Fill.cpp:915, 952-958).
                if let Some(conformal_out) = conformal_fill_surface(
                    self.regions[surface_fill.region_id].region().config().fill_pattern,
                    &surface_fill.params,
                    &surface_fill.surface,
                    &expoly,
                    surface_fill.region_id,
                    self.id(),
                    self.print_z,
                    density,
                    link_max_length,
                    self.object().print().config().resolution,
                    None,
                    &zag,
                ) {
                    sparse_infill_polylines.extend(conformal_out);
                    continue;
                }

                let mut rect_connected = false;
                let generated = match fill_pattern {
                    InfillPattern::Rectilinear | InfillPattern::Grid => {
                        // FillRectilinear.cpp:3074-3092 — `FillRectilinear::fill_surface`
                        // returns what `fill_surface_by_lines` laid: the lines are
                        // already linked along the contours
                        // (connect_segment_intersections_by_contours /
                        // traverse_graph_generate_polylines, FillRectilinear.cpp:2905,
                        // 2948), and `Fill::connect_infill` (FillBase.cpp:1501) is not
                        // run on them. RECT_FILL_NO_RECONNECT=0 runs it again.
                        if crate::faithful_gate("RECT_FILL_NO_RECONNECT") {
                            rect_connected = true;
                        }
                        generate_fill_rectilinear(
                            &[expoly],
                            &infill_config,
                            self.id as usize,
                            is_grid,
                        )
                    }
                    InfillPattern::Gyroid => {
                        use crate::fill::fill_gyroid::{generate_gyroid_infill, GyroidConfig};
                        use crate::geometry::BoundingBox;
                        let mut bb = BoundingBox::empty();
                        for pt in &expoly.contour.points {
                            bb.merge_point(*pt);
                        }
                        // R798 (gate FVS_ANCHOR_GYROID): the anchor lines must be
                        // the SAME gyroid the real sparse fill lays down — native's
                        // anchoring generator runs the actual filler (Fill.cpp:869).
                        // This arm had BOTH known traps: spacing fed the flow WIDTH
                        // (waves ~7% off pitch, R456) and the clip was the old
                        // per-vertex contains_point walk (ignores holes, truncates
                        // at the last inside vertex — replaced in the live arm by
                        // R792's exact clib clip). Wrong anchors made nearly every
                        // FVS line "unsupported": floating thicklines 46,374 vs
                        // native 347 (R797 FVSPROBE).
                        let faithful_anchor = crate::faithful_gate("FVS_ANCHOR_GYROID");
                        let raw_line_width = if faithful_anchor {
                            surface_fill.params.flow.spacing()
                        } else {
                            surface_fill.params.flow.width()
                        };
                        let gyroid_config = GyroidConfig {
                            z: self.print_z,
                            spacing: raw_line_width,
                            density,
                            angle: surface_fill.params.angle as f64,
                        };
                        let raw_polylines = generate_gyroid_infill(&gyroid_config, bb.min, bb.max);
                        if faithful_anchor {
                            crate::clipper_utils::intersection_pl(
                                &raw_polylines,
                                std::slice::from_ref(&expoly),
                            )
                            .into_iter()
                            .map(InfillPath::Line)
                            .collect()
                        } else {
                        let mut clipped = Vec::new();
                        for pl in raw_polylines {
                            let mut current_segment = Vec::new();
                            for pt in &pl.points {
                                if expoly.contour.contains_point(pt) {
                                    current_segment.push(*pt);
                                } else {
                                    if current_segment.len() >= 2 {
                                        clipped.push(InfillPath::Line(
                                            crate::geometry::Polyline::from_points(current_segment),
                                        ));
                                    }
                                    current_segment = Vec::new();
                                }
                            }
                            if current_segment.len() >= 2 {
                                clipped.push(InfillPath::Line(
                                    crate::geometry::Polyline::from_points(current_segment),
                                ));
                            }
                        }
                        clipped
                        }
                    }
                    _ => {
                        let polylines = generate_infill(
                            fill_pattern,
                            &[expoly.clone()],
                            infill_config.line_spacing,
                            infill_config.angle,
                        )?;
                        polylines.into_iter().map(InfillPath::Line).collect()
                    }
                };

                if generated.is_empty() {
                    continue;
                }

                let mut polylines = Vec::new();
                for path in generated {
                    match path {
                        InfillPath::Line(pl) => polylines.push(pl),
                        InfillPath::Loop(poly) => polylines.push(poly.split_at_first_point()),
                    }
                }

                if polylines.is_empty() {
                    continue;
                }

                // C++: Fill::connect_infill() inside fill_surface joins lines.
                if infill_config.connect_infill && !rect_connected {
                    let boundary = vec![boundary_contour.clone()];
                    let fill_params = crate::fill::FillParams::new();
                    let mut connected = Vec::new();
                    crate::fill::connect_infill(
                        polylines,
                        &boundary,
                        infill_config.line_spacing,
                        &fill_params,
                        &mut connected,
                    );
                    polylines = connected;
                }

                sparse_infill_polylines.extend(polylines);
            }
        }

        // Fill.cpp:874
        // C++: return sparse_infill_polylines;
        Ok(sparse_infill_polylines)
    }

    // Layer::make_ironing (Fill.cpp:983-1250) is in fill/fill.rs, the file paired with Fill.cpp.

    /// Generate solid infill for a surface (helper)
    /// Layer.cpp:1252-1260
    fn generate_solid_infill(&mut self, _region_idx: usize, _surface: &Surface) -> Result<()> {
        // This is now handled by make_fills()
        Ok(())
    }

    /// Generate sparse infill for a surface (helper)
    /// Layer.cpp:1262-1270
    fn generate_sparse_infill(&mut self, _region_idx: usize, _surface: &Surface) -> Result<()> {
        // This is now handled by make_fills()
        Ok(())
    }

    // Additional helper methods

    /// Create new layer (floating point variant)
    /// Layer.cpp:1272-1285
    pub fn new_f(
        id: usize,
        object_id: usize,
        height_mm: f64,
        print_z_mm: f64,
        slice_z_mm: f64,
    ) -> Self {
        Self::new(id, object_id, height_mm, print_z_mm, slice_z_mm)
    }

    /// Get bottom Z in millimeters
    /// Layer.cpp:1287-1292
    pub fn bottom_z_mm(&self) -> f64 {
        self.bottom_z()
    }

    /// Get height in millimeters
    /// Layer.cpp:1294-1299
    pub fn height_mm(&self) -> f64 {
        self.height
    }

    /// Set lower layer reference
    /// Layer.cpp:1301-1306
    pub fn set_lower_layer(&mut self, lower_layer_id: Option<usize>) {
        self.lower_layer_id = lower_layer_id;
    }

    /// Set upper layer reference
    /// Layer.cpp:1308-1313
    pub fn set_upper_layer(&mut self, upper_layer_id: Option<usize>) {
        self.upper_layer_id = upper_layer_id;
    }

    /// Get all slices (combined)
    /// Layer.cpp:1315-1325
    pub fn all_slices(&self) -> ExPolygons {
        let result = self.lslices.clone();
        result
    }
}

impl LayerRegion {
    /// Set slices for this region
    /// Layer.cpp:1327-1332
    pub fn set_slices(&mut self, slices: SurfaceCollection) {
        self.slices = slices;
    }

    /// The region and the configs of the object and of the print a LayerRegion of the pin reaches through
    /// `m_region` and `m_layer->object()` (Layer.hpp:38, 130-131): what `Layer::wire_config_hierarchy`
    /// stamps, for a LayerRegion that stands on its own.
    pub fn set_config_hierarchy(
        &mut self,
        region: Arc<crate::print_region::PrintRegion>,
        object_config: Arc<crate::print_config::PrintObjectConfig>,
        print_config: Arc<crate::print_config::PrintConfig>,
    ) {
        self.region = Some(region);
        self.object_config = Some(object_config);
        self.print_config = Some(print_config);
    }
}

// NOTE: the canonical PrintObject lives in crate::print_object; a dead local
// placeholder struct (empty bounding_box stub) was removed here.

// Layer.cpp:11 `static const int Continuitious_length = scale_(0.01);`: 0.01 / 0.00001 is
// 999.9999999999999 in double, 999 as an int.
const CONTINUITIOUS_LENGTH_PIN: i32 = (0.01 / crate::libslic3r::SCALING_FACTOR) as i32;
// Layer.cpp:12 `static const int dist_scale_threshold = 1.2;`: 1 as an int.
#[allow(non_upper_case_globals)]
const dist_scale_threshold: i32 = 1.2 as i32;

/// Layer.cpp:412-455 `class ContinuitiousDistancer`
//BBS: use aabbtree to get distance
pub struct ContinuitiousDistancer {
    /// `std::vector<Linef> lines`: the end points are whole numbers, held here as the `Line`s the
    /// tree functions take (they read them as doubles).
    lines: Vec<crate::geometry::Line>,
    tree: crate::aabb_tree_lines::tree2d::Tree,
}

impl ContinuitiousDistancer {
    /// Layer.cpp:418-425 `ContinuitiousDistancer(const Points &pts)`
    pub fn new(pts: &[Point]) -> Self {
        let pt_to_lines = Self::to_lines(pts);
        let mut lines: Vec<crate::geometry::Line> = Vec::new();
        for line in &pt_to_lines {
            lines.push(crate::geometry::Line::new(line.a, line.b));
        }

        let tree = crate::aabb_tree_lines::build_aabb_tree_over_indexed_lines(&lines);
        ContinuitiousDistancer { lines, tree }
    }

    /// Layer.cpp:427-443 `float distance_from_perimeter(const Vec2f &point) const`
    pub fn distance_from_perimeter(&self, point: [f32; 2]) -> f32 {
        let p = crate::geometry::PointF::new(point[0] as f64, point[1] as f64);
        let mut hit_idx_out: usize = 0;
        let mut hit_point_out = crate::geometry::PointF::new(0.0, 0.0);
        let mut distance: f64 = crate::aabb_tree_lines::squared_distance_to_indexed_lines(
            &self.lines,
            &self.tree,
            p,
            &mut hit_idx_out,
            &mut hit_point_out,
            f64::INFINITY,
        );
        if distance < 0.0 {
            return f32::MAX;
        }

        distance = distance.sqrt();
        let line = &self.lines[hit_idx_out];
        let v1 = ((line.b.x - line.a.x) as f64, (line.b.y - line.a.y) as f64);
        let v2 = (p.x - line.a.x as f64, p.y - line.a.y as f64);
        if v1.0.mul_add(v2.1, -(v1.1 * v2.0)) > 0.0 {
            distance *= -1.0;
        }
        distance as f32
    }

    /// Layer.cpp:445-454 `Lines to_lines(const Points &pts)`
    pub fn to_lines(pts: &[Point]) -> Vec<crate::geometry::Line> {
        let mut lines: Vec<crate::geometry::Line> = Vec::new();
        if pts.len() >= 2 {
            lines.reserve(pts.len() - 1);
            for it in 0..pts.len() - 1 {
                lines.push(crate::geometry::Line::new(pts[it], pts[it + 1]));
            }
        }
        lines
    }
}

/// Layer.cpp:458-467 `static double get_node_continuity_rang_limit(const std::vector<coord_t> &Prev_node_widths, int prev_pt_idx)`
#[allow(non_snake_case)]
fn get_node_continuity_rang_limit(Prev_node_widths: &[crate::Coord], prev_pt_idx: i32) -> f64 {
    let width: f64;
    let mut prev_width: f64 = Prev_node_widths[0] as f64;
    if prev_pt_idx != 0 && (prev_pt_idx as usize) < Prev_node_widths.len() {
        prev_width = Prev_node_widths[prev_pt_idx as usize] as f64;
    }

    width = prev_width * dist_scale_threshold as f64;
    width
}

/// Layer.hpp:411-419 `template<typename LayerContainer> inline std::vector<float> zs_from_layers(const LayerContainer &layers)`
pub fn zs_from_layers(layers: &[Layer]) -> Vec<f32> {
    let mut zs: Vec<f32> = Vec::new();
    zs.reserve(layers.len());
    for l in layers {
        zs.push(l.slice_z as f32);
    }
    zs
}

impl Layer {
    /// Layer.hpp:210-213 `template <class T> bool any_internal_region_slice_contains(const T &item) const`
    pub fn any_internal_region_slice_contains<T: crate::surface_collection::ExPolygonContains>(&self, item: &T) -> bool {
        for layerm in &self.regions {
            if layerm.slices.any_internal_contains(item) {
                return true;
            }
        }
        false
    }

    /// Layer.hpp:214-217 `template <class T> bool any_bottom_region_slice_contains(const T &item) const`
    pub fn any_bottom_region_slice_contains<T: crate::surface_collection::ExPolygonContains>(&self, item: &T) -> bool {
        for layerm in &self.regions {
            if layerm.slices.any_bottom_contains(item) {
                return true;
            }
        }
        false
    }

    /// Layer.hpp:239 `virtual bool has_extrusions() const`
    // Is there any valid extrusion assigned to this LayerRegion?
    pub fn has_extrusions(&self) -> bool {
        for layerm in &self.regions {
            if layerm.has_extrusions() {
                return true;
            }
        }
        false
    }

    /// Layer.hpp:242-249 `virtual coordf_t effective_print_height() const`
    // BBS multi-diameter adaptive layer height: the maximum Z span covered by any group-top patch.
    pub fn effective_print_height(&self) -> f64 {
        let mut h: f64 = self.height;
        for layerm in &self.regions {
            for patch in &layerm.adaptive_merge_patches {
                if patch.group_top && patch.merged_height > h {
                    h = patch.merged_height;
                }
            }
        }
        h
    }

    /// Layer.hpp:256-263 `bool is_adaptive_merged_out() const`
    // BBS multi-diameter adaptive layer height: true when this layer takes part in adaptive merging and
    // carries no extrusions of its own. What it would have printed either moved up into the group-top layer
    // or the group had nothing printable to begin with (a merged span that is internal void everywhere still
    // owns its patches but extrudes nothing, and a group top whose slices moved to another region ends up
    // empty as well). In none of those cases is the empty Z span a printability gap the merge planner left
    // behind, so the layer above must not be reported as unsupported.
    pub fn is_adaptive_merged_out(&self) -> bool {
        if self.regions.is_empty() || self.has_extrusions() {
            return false;
        }
        for layerm in &self.regions {
            if !layerm.adaptive_merge_patches.is_empty() {
                return true;
            }
        }
        false
    }

    /// Layer.hpp:266 `void simplify_wall_extrusion_path()`
    //BBS
    pub fn simplify_wall_extrusion_path(&mut self) {
        for layerm in &mut self.regions {
            layerm.simplify_wall_extrusion_entity();
        }
    }

    /// Layer.hpp:267 `void simplify_infill_extrusion_path()`
    pub fn simplify_infill_extrusion_path(&mut self) {
        for layerm in &mut self.regions {
            layerm.simplify_infill_extrusion_entity();
        }
    }

    /// Layer.hpp:277-304 `template<typename IteratorType, typename IndexType, typename FN_HIGHER_EQUAL> static IndexType idx_higher_or_equal(IteratorType begin, IteratorType end, IndexType idx, FN_HIGHER_EQUAL fn_higher_equal)`
    ///
    /// `begin..end` is `items`; `IndexType(-1)` is `usize::MAX`.
    // FN_HIGHER_EQUAL: the provided object pointer has a Z value >= of an internal threshold.
    // Find the first item with Z value >= of an internal threshold of fn_higher_equal.
    // If no vec item with Z value >= of an internal threshold of fn_higher_equal is found, return vec.size()
    // If the initial idx is size_t(-1), then use binary search.
    // Otherwise search linearly upwards.
    pub fn idx_higher_or_equal<T, F: Fn(&T) -> bool>(items: &[T], mut idx: usize, fn_higher_equal: F) -> usize {
        let size = items.len() as i32;
        if size == 0 {
            idx = 0;
        } else if idx == usize::MAX {
            // First of the batch of layers per thread pool invocation. Use binary search.
            let mut idx_low: i32 = 0;
            let mut idx_high: i32 = 0.max(size - 1);
            while idx_low + 1 < idx_high {
                let idx_mid = (idx_low + idx_high) / 2;
                if fn_higher_equal(&items[idx_mid as usize]) {
                    idx_high = idx_mid;
                } else {
                    idx_low = idx_mid;
                }
            }
            idx = if fn_higher_equal(&items[idx_low as usize]) {
                idx_low as usize
            } else if fn_higher_equal(&items[idx_high as usize]) {
                idx_high as usize
            } else {
                size as usize
            };
        } else {
            // For the other layers of this batch of layers, search incrementally, which is cheaper than the binary search.
            while (idx as i32) < size && !fn_higher_equal(&items[idx]) {
                idx += 1;
            }
        }
        idx
    }

    /// Layer.cpp:468-547 `void Layer::calculate_perimeter_continuity(std::vector<LoopNode> &prev_nodes)`
    /// at the pin: the distance of a point of the lower contour is that of the point as floats, to the
    /// nearest line of the node's contour through the AABB tree, as a float; a run counts from
    /// `Continuitious_length`, the int 999.
    fn calculate_perimeter_continuity_pin(&mut self, prev_nodes: &mut [LoopNode]) {
        // `Polyline::length()`
        fn polyline_length(pts: &[Point]) -> f64 {
            let mut l: f64 = 0.0;
            for i in 1..pts.len() {
                let dx = (pts[i].x - pts[i - 1].x) as f64;
                let dy = (pts[i].y - pts[i - 1].y) as f64;
                l += (dx * dx + dy * dy).sqrt();
            }
            l
        }
        // `BoundingBoxBase::overlap` (BoundingBox.hpp:57-60)
        fn overlap(a: &BoundingBox, b: &BoundingBox) -> bool {
            !(a.max.x < b.min.x || a.min.x > b.max.x || a.max.y < b.min.y || a.min.y > b.max.y)
        }

        for node_pos in 0..self.loop_nodes.len() {
            let mut width: f64;
            let node_distancer = ContinuitiousDistancer::new(&self.loop_nodes[node_pos].node_contour.pts);
            for prev_pos in 0..prev_nodes.len() {
                let prev_node = &mut prev_nodes[prev_pos];
                let node = &mut self.loop_nodes[node_pos];

                // no overlap or has diff speed
                if !overlap(&node.bbox, &prev_node.bbox) {
                    continue;
                }

                //calculate dist, checkout the continuity
                let mut continuitious_pl: Vec<Point> = Vec::new();
                //check start pt
                let mut start: usize = 0;
                let mut conntiouitious_flag = false;
                let mut end: i32 = prev_node.node_contour.pts.len() as i32 - 1;

                //if the countor is loop
                if prev_node.node_contour.is_loop {
                    while end >= 0 {
                        if polyline_length(&continuitious_pl) >= CONTINUITIOUS_LENGTH_PIN as f64 {
                            node.lower_node_ids.push(prev_node.node_id);
                            prev_node.upper_node_ids.push(node.node_id);
                            conntiouitious_flag = true;
                            break;
                        }

                        let pt = prev_node.node_contour.pts[end as usize];
                        let dist: f32 = node_distancer.distance_from_perimeter([pt.x as f32, pt.y as f32]);
                        // get corr width
                        width = get_node_continuity_rang_limit(&prev_node.node_contour.widths, end);

                        if (dist as f64) < width && (dist as f64) > -width {
                            continuitious_pl.insert(0, pt);
                        } else {
                            break;
                        }
                        end -= 1;
                    }

                    if conntiouitious_flag || end < 0 {
                        continue;
                    }
                }

                let mut last_pt_idx: i32 = end;
                // line need to check end point
                if !prev_node.node_contour.is_loop {
                    last_pt_idx += 1;
                }

                // `start < last_pt_idx` compares a size_t with an int: the int converts.
                while start < last_pt_idx as isize as usize {
                    let pt = prev_node.node_contour.pts[start];
                    let dist: f32 = node_distancer.distance_from_perimeter([pt.x as f32, pt.y as f32]);
                    //get corr width
                    width = get_node_continuity_rang_limit(&prev_node.node_contour.widths, start as i32);

                    if (dist as f64) < width && (dist as f64) > -width {
                        continuitious_pl.push(pt);
                        start += 1;
                        continue;
                    }

                    if continuitious_pl.is_empty() || polyline_length(&continuitious_pl) < CONTINUITIOUS_LENGTH_PIN as f64 {
                        continuitious_pl.clear();
                        start += 1;
                        continue;
                    }

                    node.lower_node_ids.push(prev_node.node_id);
                    prev_node.upper_node_ids.push(node.node_id);
                    continuitious_pl.clear();
                    break;
                }

                if polyline_length(&continuitious_pl) >= CONTINUITIOUS_LENGTH_PIN as f64 {
                    node.lower_node_ids.push(prev_node.node_id);
                    prev_node.upper_node_ids.push(node.node_id);
                }
            }
        }
    }

    /// Layer.cpp:576-593 `void Layer::export_region_slices_to_svg(const char *path) const`
    pub fn export_region_slices_to_svg(&self, path: &str) {
        let mut bbox = BoundingBox::empty();
        for region in &self.regions {
            for surface in &region.slices.surfaces {
                bbox.merge(&crate::ex_polygon::get_extents_expoly(&surface.expolygon));
            }
        }
        let legend_size = crate::surface::export_surface_type_legend_to_svg_box_size();
        let legend_pos = Point::new(bbox.min.x, bbox.max.y);
        bbox.merge_point(Point::new((bbox.min.x + legend_size.x).max(bbox.max.x), bbox.max.y + legend_size.y));

        let mut svg = crate::svg::SVG::new_bbox_default(path, &bbox);
        let transparency: f32 = 0.5;
        for region in &self.regions {
            for surface in &region.slices.surfaces {
                svg.draw_expolygon(&surface.expolygon, crate::surface::surface_type_to_color_name(surface.surface_type), transparency);
            }
        }
        crate::surface::export_surface_type_legend_to_svg(&mut svg, &legend_pos);
        svg.close();
    }

    /// Layer.cpp:596-600 `void Layer::export_region_slices_to_svg_debug(const char *name) const`
    // Export to "out/LayerRegion-name-%d.svg" with an increasing index with every export.
    pub fn export_region_slices_to_svg_debug(&self, name: &str) {
        static IDX: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let idx = IDX.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        self.export_region_slices_to_svg(&crate::utils::debug_out_path(&format!("Layer-slices-{}-{}.svg", name, idx)));
    }

    /// Layer.cpp:602-619 `void Layer::export_region_fill_surfaces_to_svg(const char *path) const`
    pub fn export_region_fill_surfaces_to_svg(&self, path: &str) {
        let mut bbox = BoundingBox::empty();
        for region in &self.regions {
            for surface in &region.slices.surfaces {
                bbox.merge(&crate::ex_polygon::get_extents_expoly(&surface.expolygon));
            }
        }
        let legend_size = crate::surface::export_surface_type_legend_to_svg_box_size();
        let legend_pos = Point::new(bbox.min.x, bbox.max.y);
        bbox.merge_point(Point::new((bbox.min.x + legend_size.x).max(bbox.max.x), bbox.max.y + legend_size.y));

        let mut svg = crate::svg::SVG::new_bbox_default(path, &bbox);
        let transparency: f32 = 0.5;
        for region in &self.regions {
            for surface in &region.slices.surfaces {
                svg.draw_expolygon(&surface.expolygon, crate::surface::surface_type_to_color_name(surface.surface_type), transparency);
            }
        }
        crate::surface::export_surface_type_legend_to_svg(&mut svg, &legend_pos);
        svg.close();
    }

    /// The `print_config` values the `simplify_support_*` functions read (Layer.cpp:640-643):
    /// `spiral_mode`, `enable_arc_fitting`, `scaled<double>(resolution)`.
    fn simplify_support_print_params(&self) -> (bool, bool, f64) {
        let print_config = self.print_config.as_deref().expect("config hierarchy not wired — call wire_layer_hierarchy");
        (print_config.spiral_vase, print_config.arc_fitting_enabled, print_config.resolution / crate::libslic3r::SCALING_FACTOR)
    }

    /// Layer.cpp:622-636 `void Layer::simplify_support_entity_collection(ExtrusionEntityCollection* entity_collection)`
    //BBS: method to simplify support path
    pub fn simplify_support_entity_collection(&self, entity_collection: &mut ExtrusionEntityCollection) {
        use crate::extrusion_entity::ExtrusionEntityType;
        for i in 0..entity_collection.entities.len() {
            match &mut entity_collection.entities[i] {
                ExtrusionEntityType::Collection(collection) => self.simplify_support_entity_collection(collection),
                ExtrusionEntityType::Path(path) => self.simplify_support_path(path),
                ExtrusionEntityType::MultiPath(multipath) => self.simplify_support_multi_path(multipath),
                ExtrusionEntityType::Loop(loop_) => self.simplify_support_loop(loop_),
            }
        }
    }

    /// Layer.cpp:638-651 `void Layer::simplify_support_path(ExtrusionPath * path)`
    //BBS: method to simplify support path
    pub fn simplify_support_path(&self, path: &mut crate::extrusion_entity::ExtrusionPath) {
        let (spiral_mode, enable_arc_fitting, scaled_resolution) = self.simplify_support_print_params();

        if enable_arc_fitting && !spiral_mode {
            path.simplify_by_fitting_arc(SCALED_SUPPORT_RESOLUTION);
        } else {
            path.simplify(scaled_resolution);
        }
    }

    /// Layer.cpp:653-668 `void Layer::simplify_support_multi_path(ExtrusionMultiPath* multipath)`
    //BBS: method to simplify support path
    pub fn simplify_support_multi_path(&self, multipath: &mut crate::extrusion_entity::ExtrusionMultiPath) {
        let (spiral_mode, enable_arc_fitting, scaled_resolution) = self.simplify_support_print_params();

        for i in 0..multipath.paths.len() {
            if enable_arc_fitting && !spiral_mode {
                multipath.paths[i].simplify_by_fitting_arc(SCALED_SUPPORT_RESOLUTION);
            } else {
                multipath.paths[i].simplify(scaled_resolution);
            }
        }
    }

    /// Layer.cpp:670-685 `void Layer::simplify_support_loop(ExtrusionLoop* loop)`
    //BBS: method to simplify support path
    pub fn simplify_support_loop(&self, loop_: &mut crate::extrusion_entity::ExtrusionLoop) {
        let (spiral_mode, enable_arc_fitting, scaled_resolution) = self.simplify_support_print_params();

        for i in 0..loop_.paths.len() {
            if enable_arc_fitting && !spiral_mode {
                loop_.paths[i].simplify_by_fitting_arc(SCALED_SUPPORT_RESOLUTION);
            } else {
                loop_.paths[i].simplify(scaled_resolution);
            }
        }
    }

    /// Layer.cpp:688-692 `void Layer::export_region_fill_surfaces_to_svg_debug(const char *name) const`
    // Export to "out/LayerRegion-name-%d.svg" with an increasing index with every export.
    pub fn export_region_fill_surfaces_to_svg_debug(&self, name: &str) {
        static IDX: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let idx = IDX.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        self.export_region_fill_surfaces_to_svg(&crate::utils::debug_out_path(&format!("Layer-fill_surfaces-{}-{}.svg", name, idx)));
    }
}

/// libslic3r.h:68-69 `#define SCALED_SUPPORT_RESOLUTION (SUPPORT_RESOLUTION / SCALING_FACTOR)`, with
/// `SUPPORT_RESOLUTION = 0.0375`.
const SCALED_SUPPORT_RESOLUTION: f64 = 0.0375 / crate::libslic3r::SCALING_FACTOR;

impl crate::print::faithful::SupportLayer {
    /// Layer.hpp:374 `void simplify_support_extrusion_path() { this->simplify_support_entity_collection(&support_fills); }`
    pub fn simplify_support_extrusion_path(&mut self) {
        let mut support_fills = std::mem::take(&mut self.support_fills);
        self.layer.simplify_support_entity_collection(&mut support_fills);
        self.support_fills = support_fills;
    }
}

/// Chain points by finding nearest neighbors
/// Layer.cpp:1334-1385
fn chain_points(points: &[Point]) -> Vec<usize> {
    if points.is_empty() {
        return Vec::new();
    }

    let mut result = Vec::with_capacity(points.len());
    let mut used = vec![false; points.len()];

    // Start with first point
    result.push(0);
    used[0] = true;

    // Chain remaining points
    for _ in 1..points.len() {
        let last_idx = *result.last().unwrap();
        let last_pt = &points[last_idx];

        // Find nearest unused point
        let mut nearest_idx = 0;
        let mut nearest_dist = f64::MAX;

        for (i, pt) in points.iter().enumerate() {
            if !used[i] {
                let dx = (pt.x - last_pt.x) as f64;
                let dy = (pt.y - last_pt.y) as f64;
                let dist = dx * dx + dy * dy;

                if dist < nearest_dist {
                    nearest_dist = dist;
                    nearest_idx = i;
                }
            }
        }

        result.push(nearest_idx);
        used[nearest_idx] = true;
    }

    result
}

/// Layer.cpp:750-770: what the two overloads of `get_extents` take (a LayerRegion, the regions of a layer).
pub trait LayerGetExtents {
    fn layer_get_extents(&self) -> BoundingBox;
}
impl LayerGetExtents for LayerRegion {
    // Layer.cpp:750-759
    fn layer_get_extents(&self) -> BoundingBox {
        get_extents_layer_region(self)
    }
}
impl LayerGetExtents for [LayerRegion] {
    // Layer.cpp:761-770
    fn layer_get_extents(&self) -> BoundingBox {
        get_extents_layer_regions(self)
    }
}

// Layer.cpp:750-770
// C++: BoundingBox get_extents(const LayerRegion &layer_region)
// C++: BoundingBox get_extents(const LayerRegionPtrs &layer_regions)
pub fn get_extents<T: LayerGetExtents + ?Sized>(what: &T) -> BoundingBox {
    what.layer_get_extents()
}

// Layer.cpp:750-759
// C++: BoundingBox get_extents(const LayerRegion &layer_region)
pub fn get_extents_layer_region(layer_region: &LayerRegion) -> BoundingBox {
    use crate::geometry::get_extents_expoly;
    // Layer.cpp:637
    let mut bbox = BoundingBox::empty();
    // Layer.cpp:638-642
    // C++: if (!layer_region.slices.surfaces.empty()) {
    // C++:     bbox = get_extents(layer_region.slices.surfaces.front());
    // C++:     for (auto it = layer_region.slices.surfaces.cbegin() + 1; it != layer_region.slices.surfaces.cend(); ++it)
    // C++:         bbox.merge(get_extents(*it));
    // C++: }
    if !layer_region.slices.surfaces.is_empty() {
        // get_extents(const Surface&) returns the extents of surface.expolygon.
        bbox = get_extents_expoly(&layer_region.slices.surfaces[0].expolygon);
        for surface in &layer_region.slices.surfaces[1..] {
            bbox.merge(&get_extents_expoly(&surface.expolygon));
        }
    }
    // Layer.cpp:643
    bbox
}

// Layer.cpp:761-770
// C++: BoundingBox get_extents(const LayerRegionPtrs &layer_regions)
pub fn get_extents_layer_regions(layer_regions: &[LayerRegion]) -> BoundingBox {
    // Layer.cpp:648
    let mut bbox = BoundingBox::empty();
    // Layer.cpp:649-654
    // C++: if (!layer_regions.empty()) {
    // C++:     bbox = get_extents(*layer_regions.front());
    // C++:     for (auto it = layer_regions.begin() + 1; it != layer_regions.end(); ++it)
    // C++:         bbox.merge(get_extents(**it));
    // C++: }
    if !layer_regions.is_empty() {
        bbox = get_extents_layer_region(&layer_regions[0]);
        for layer_region in &layer_regions[1..] {
            bbox.merge(&get_extents_layer_region(layer_region));
        }
    }
    // Layer.cpp:655
    bbox
}

/// R555: what `LayerRegion::make_perimeters` receives, mirroring `[CPP-MPPROBE]`.
/// Counts surfaces, holes and points per (layer, region) so the 1.5x surface gap
/// can be localised rather than only seen in aggregate.
fn mpprobe(layer_id: i32, region_id: i32, surfaces: usize, holes: usize, points: usize) {
    use std::sync::Mutex;
    static ACC: Mutex<(usize, usize, usize, usize)> = Mutex::new((0, 0, 0, 0));
    let Ok(mut g) = ACC.lock() else { return };
    g.0 += 1;
    g.1 += surfaces;
    g.2 += holes;
    g.3 += points;
    if g.0 == 1 || g.0 % 200 == 0 {
        eprintln!(
            "[MPPROBE] calls={} | surfaces={} holes={} points={} | \
             last(layer={layer_id} region={region_id} s={surfaces} h={holes})",
            g.0, g.1, g.2, g.3
        );
    }
}
