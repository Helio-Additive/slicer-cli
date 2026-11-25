//! Port of `PrintObjectSlice.cpp` (BambuStudio libslic3r at the pin), over the faithful
//! `crate::print::faithful::{Print, PrintObject}` and `crate::layer::{Layer, LayerRegion}`.
//!
//! C++ reference: src/libslic3r/PrintObjectSlice.cpp. Every function carries the lines it ports.
//!
//! What differs in shape from the C++:
//!   * A `PrintObject` has no pointer to its `Print`: the functions take the print beside the
//!     object. The object is one taken out of `Print::m_objects` (or a copy of it) while the step
//!     runs, as the callers of `crate::print_object::faithful` do.
//!   * A `LayerRegion` has no pointer to its `PrintRegion`: it is `all_regions[region_id]` of the
//!     object's shared regions, which is what `PrintObject::slice_volumes` builds (one LayerRegion
//!     per PrintRegion, in the order of `all_regions`).
//!   * `ModelVolume*` is the index of the volume in its ModelObject, `VolumeSlices*` the index in
//!     the vector of slices.
//!   * The `tbb::parallel_for` loops run in order.
//!   * The units this file calls that have no port over the faithful Print (the multi-material and
//!     the fuzzy skin segmentation by painting, the interlocking generator) are the methods of
//!     [`SliceVolumesCores`], handed in by the caller.

#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]

use crate::clipper as cl;
use crate::clipper_utils as cu;
use crate::config::{ConfigOptionFloat, ConfigOptionResolver};
use crate::flow::{Flow, FlowRole};
use crate::geometry::geometry::{eigen, Transform3d};
use crate::geometry::{BoundingBox, ExPolygon, ExPolygons, Point, Polygon, Polygons};
use crate::layer::{zs_from_layers, Layer, LayerRegion};
use crate::libslic3r::{EPSILON, SCALING_FACTOR};
use crate::model::{ModelVolume, ModelVolumeType};
use crate::normal_utils::indexed_triangle_set;
use crate::object_id::ObjectID;
use crate::print::faithful::{
    groupedVolumeSlices, AlignedBox3f, LayerRangeRegions, Print, PrintObject, PrintObjectRegions, PrintObjectStep, PrintRegion,
    VolumeSlices,
};
use crate::print_base::{CanceledException, SlicingStatus, WarningLevel};
use crate::print_config::config_def::DynamicPrintConfig;
use crate::surface::{Surface, SurfaceType};
use crate::triangle_mesh_slicer::{slice_mesh_ex_its, MeshSlicingParamsEx as SlicerParamsEx, SlicingMode as MeshSlicingMode};
use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicBool, Ordering};

// PrintObjectSlice.cpp:34-35
/// `bool PrintObject::clip_multipart_objects = true;`
pub static clip_multipart_objects: AtomicBool = AtomicBool::new(true);
/// `bool PrintObject::infill_only_where_needed = false;`
pub static infill_only_where_needed: AtomicBool = AtomicBool::new(false);

/// What the C++ of this unit throws.
#[derive(Debug, Clone)]
pub enum PrintObjectSliceError {
    /// `CanceledException` of `Print::throw_if_canceled()`
    Canceled(CanceledException),
    /// `Slic3r::SlicingError`
    SlicingError(String),
    /// `Slic3r::InvalidArgument` (PrintRegion::flow, Flow::new_from_config_width)
    InvalidArgument(String),
}

impl From<CanceledException> for PrintObjectSliceError {
    fn from(e: CanceledException) -> Self {
        PrintObjectSliceError::Canceled(e)
    }
}

impl std::fmt::Display for PrintObjectSliceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PrintObjectSliceError::Canceled(e) => write!(f, "{}", e),
            PrintObjectSliceError::SlicingError(s) | PrintObjectSliceError::InvalidArgument(s) => write!(f, "{}", s),
        }
    }
}

/// `const std::function<void()> &throw_on_cancel_callback`: the callback that throws
/// `CanceledException`.
pub type ThrowOnCancel<'a> = &'a dyn Fn() -> Result<(), CanceledException>;

/// The units `PrintObject::slice_volumes` calls that have no port over the faithful Print yet.
pub trait SliceVolumesCores {
    /// MultiMaterialSegmentation.cpp `std::vector<std::vector<ExPolygons>>
    /// multi_material_segmentation_by_painting(const PrintObject &print_object, const std::function<void()> &throw_on_cancel_callback)`
    fn multi_material_segmentation_by_painting(&mut self, print: &Print, print_object: &PrintObject) -> Vec<Vec<ExPolygons>>;
    /// MultiMaterialSegmentation.cpp `std::vector<std::vector<ExPolygons>>
    /// fuzzy_skin_segmentation_by_painting(const PrintObject &print_object, const std::function<void()> &throw_on_cancel_callback)`
    fn fuzzy_skin_segmentation_by_painting(&mut self, print: &Print, print_object: &PrintObject) -> Vec<Vec<ExPolygons>>;
    /// Interlocking/InterlockingGenerator.cpp `InterlockingGenerator::generate_interlocking_structure(PrintObject *print_object)`
    fn generate_interlocking_structure(&mut self, print: &Print, print_object: &mut PrintObject);
    /// Interlocking/InterlockingGenerator.cpp `InterlockingGenerator::generate_embedding_wall(PrintObject *print_object)`
    fn generate_embedding_wall(&mut self, print: &Print, print_object: &mut PrintObject);
}

/// The cores of an object that is not painted and has no interlocking: nothing to segment.
pub struct NoSliceVolumesCores;

impl SliceVolumesCores for NoSliceVolumesCores {
    fn multi_material_segmentation_by_painting(&mut self, _print: &Print, _print_object: &PrintObject) -> Vec<Vec<ExPolygons>> {
        panic!("multi_material_segmentation_by_painting: no port over the faithful Print was handed in")
    }
    fn fuzzy_skin_segmentation_by_painting(&mut self, _print: &Print, _print_object: &PrintObject) -> Vec<Vec<ExPolygons>> {
        panic!("fuzzy_skin_segmentation_by_painting: no port over the faithful Print was handed in")
    }
    fn generate_interlocking_structure(&mut self, _print: &Print, _print_object: &mut PrintObject) {}
    fn generate_embedding_wall(&mut self, _print: &Print, _print_object: &mut PrintObject) {}
}

// ---------------------------------------------------------------------------------------------
// Configuration reads
// ---------------------------------------------------------------------------------------------

fn cfg_int(config: &DynamicPrintConfig, key: &str) -> i32 {
    config.option(key).and_then(|o| o.get_int().ok()).unwrap_or_else(|| panic!("option {key}"))
}
fn cfg_float(config: &DynamicPrintConfig, key: &str) -> f64 {
    config.option(key).and_then(|o| o.get_float().ok()).unwrap_or_else(|| panic!("option {key}"))
}
fn cfg_bool(config: &DynamicPrintConfig, key: &str) -> bool {
    config.option(key).and_then(|o| o.get_bool().ok()).unwrap_or_else(|| panic!("option {key}"))
}
fn cfg_floats(config: &DynamicPrintConfig, key: &str) -> Vec<f64> {
    crate::gcode::wipe_tower::cfg_floats(config, key)
}

// ---------------------------------------------------------------------------------------------
// ClipperUtils.cpp, the overloads this file calls. `delta` is the `const float` of the C++.
// ---------------------------------------------------------------------------------------------

/// ClipperUtils.hpp:37 `DefaultMiterLimit`
const DEFAULT_MITER_LIMIT: f64 = 3.;
/// ClipperUtils.hpp:31 `DefaultJoinType`
const DEFAULT_JOIN_TYPE: cl::JoinType = cl::JoinType::jtMiter;

fn path_of(points: &[Point]) -> cl::Path {
    points.iter().map(|p| cl::IntPoint::new(p.x as i32, p.y as i32)).collect()
}
/// `ClipperUtils::PolygonsProvider`
fn polygons_provider(polygons: &[Polygon]) -> cl::Paths {
    polygons.iter().map(|p| path_of(&p.points)).collect()
}
/// `ClipperUtils::ExPolygonsProvider`: the contour, then the holes, of each expolygon.
fn expolygons_provider(expolygons: &[ExPolygon]) -> cl::Paths {
    let mut out = cl::Paths::new();
    for e in expolygons {
        out.push(path_of(&e.contour.points));
        for hole in &e.holes {
            out.push(path_of(&hole.points));
        }
    }
    out
}
/// `ClipperUtils::SurfacesProvider`: as the expolygons of the surfaces.
fn surfaces_provider(surfaces: &[Surface]) -> cl::Paths {
    let mut out = cl::Paths::new();
    for s in surfaces {
        out.push(path_of(&s.expolygon.contour.points));
        for hole in &s.expolygon.holes {
            out.push(path_of(&hole.points));
        }
    }
    out
}
/// An expolygon as the contour and holes `offset_expolygon_inner` reads.
fn expolygon_rings(e: &ExPolygon) -> (cl::Path, cl::Paths) {
    (path_of(&e.contour.points), e.holes.iter().map(|h| path_of(&h.points)).collect())
}
/// `to_polygons(ClipperLib::Paths)`
fn to_polygons_paths(paths: &cl::Paths) -> Polygons {
    cu::clib_polygons_from_paths(paths)
}
/// `float(delta)` as the double ClipperOffset reads.
fn fl(delta: f64) -> f64 {
    delta as f32 as f64
}

/// ClipperUtils.cpp:737-739 `_clipper_ex(clipType, subject, clip, ApplySafetyOffset::No)`
fn clipper_ex(clip_type: cl::ClipType, subject: &cl::Paths, clip: &cl::Paths) -> ExPolygons {
    cu::_clipper_ex(clip_type, subject, clip, cu::ApplySafetyOffset::No, cl::PolyFillType::pftNonZero)
}
/// ClipperUtils.cpp:755-756 `diff_ex(const ExPolygons &subject, const ExPolygons &clip)`
fn diff_ex(subject: &[ExPolygon], clip: &[ExPolygon]) -> ExPolygons {
    clipper_ex(cl::ClipType::ctDifference, &expolygons_provider(subject), &expolygons_provider(clip))
}
/// ClipperUtils.cpp:741-742 `diff_ex(const Polygons &subject, const Polygons &clip)`
fn diff_ex_polygons(subject: &[Polygon], clip: &[Polygon]) -> ExPolygons {
    clipper_ex(cl::ClipType::ctDifference, &polygons_provider(subject), &polygons_provider(clip))
}
/// ClipperUtils.cpp:695-696 `diff(const Polygons &subject, const ExPolygons &clip)`
fn diff_polygons_expolygons(subject: &[Polygon], clip: &[ExPolygon]) -> Polygons {
    to_polygons_paths(&cu::clipper_do::<cl::Paths>(
        cl::ClipType::ctDifference,
        &polygons_provider(subject),
        &expolygons_provider(clip),
        cl::PolyFillType::pftNonZero,
    ))
}
/// ClipperUtils.cpp:802-803 `intersection_ex(const ExPolygons &subject, const ExPolygons &clip)`
fn intersection_ex(subject: &[ExPolygon], clip: &[ExPolygon]) -> ExPolygons {
    clipper_ex(cl::ClipType::ctIntersection, &expolygons_provider(subject), &expolygons_provider(clip))
}
/// ClipperUtils.cpp:796-797 `intersection_ex(const ExPolygons &subject, const Polygons &clip)`
fn intersection_ex_polygons(subject: &[ExPolygon], clip: &[Polygon]) -> ExPolygons {
    clipper_ex(cl::ClipType::ctIntersection, &expolygons_provider(subject), &polygons_provider(clip))
}
/// ClipperUtils.cpp:806-807 `intersection_ex(const Surfaces &subject, const ExPolygons &clip)`
fn intersection_ex_surfaces(subject: &[Surface], clip: &[ExPolygon]) -> ExPolygons {
    clipper_ex(cl::ClipType::ctIntersection, &surfaces_provider(subject), &expolygons_provider(clip))
}
/// ClipperUtils.cpp:813-814 `union_ex(const Polygons &subject, pftNonZero)`
fn union_ex(subject: &[Polygon]) -> ExPolygons {
    clipper_ex(cl::ClipType::ctUnion, &polygons_provider(subject), &cl::Paths::new())
}
/// ClipperUtils.cpp:815-816 `union_ex(const ExPolygons &subject)`
fn union_ex_expolygons(subject: &[ExPolygon]) -> ExPolygons {
    cu::poly_tree_to_ex_polygons(&cu::clipper_do_polytree(
        cl::ClipType::ctUnion,
        &expolygons_provider(subject),
        &cl::Paths::new(),
        cl::PolyFillType::pftNonZero,
    ))
}
/// ClipperUtils.cpp:721-722 `union_(const Polygons &subject)`
fn union_(subject: &[Polygon]) -> Polygons {
    to_polygons_paths(&cu::clipper_do::<cl::Paths>(
        cl::ClipType::ctUnion,
        &polygons_provider(subject),
        &cl::Paths::new(),
        cl::PolyFillType::pftNonZero,
    ))
}
/// ClipperUtils.cpp:410-411 `offset(const Polygon &polygon, const float delta)`
fn offset_polygon(polygon: &Polygon, delta: f64) -> Polygons {
    to_polygons_paths(&cu::raw_offset(
        &vec![path_of(&polygon.points)],
        fl(delta),
        DEFAULT_JOIN_TYPE,
        DEFAULT_MITER_LIMIT,
        cl::EndType::etClosedPolygon,
    ))
}
/// ClipperUtils.cpp:563-564 `offset(const Surfaces &surfaces, const float delta)`
fn offset_surfaces(surfaces: &[Surface], delta: f64) -> Polygons {
    let rings: Vec<(cl::Path, cl::Paths)> = surfaces.iter().map(|s| expolygon_rings(&s.expolygon)).collect();
    to_polygons_paths(&cu::expolygons_offset(&rings, fl(delta), DEFAULT_JOIN_TYPE, DEFAULT_MITER_LIMIT))
}
/// ClipperUtils.cpp:567-569 `offset_ex(const ExPolygon &expolygon, const float delta)`
fn offset_ex_expolygon(expolygon: &ExPolygon, delta: f64) -> ExPolygons {
    let (contour, holes) = expolygon_rings(expolygon);
    cu::offset_ex_expolygon_paths(&contour, &holes, fl(delta), DEFAULT_JOIN_TYPE, DEFAULT_MITER_LIMIT)
}
/// ClipperUtils.cpp:570-571 `offset_ex(const ExPolygons &expolygons, const float delta)`
fn offset_ex(expolygons: &[ExPolygon], delta: f64) -> ExPolygons {
    let rings: Vec<(cl::Path, cl::Paths)> = expolygons.iter().map(expolygon_rings).collect();
    cu::offset_ex_expolygons_paths(&rings, fl(delta), DEFAULT_JOIN_TYPE, DEFAULT_MITER_LIMIT)
}
/// ClipperUtils.hpp:412-413 `closing_ex(const ExPolygons &polygons, const float delta)` =
/// `offset2_ex(polygons, delta, - delta)` (ClipperUtils.cpp:581-584)
fn closing_ex(expolygons: &[ExPolygon], delta: f64) -> ExPolygons {
    let rings: Vec<(cl::Path, cl::Paths)> = expolygons.iter().map(expolygon_rings).collect();
    cu::poly_tree_to_ex_polygons(&cu::offset_paths::<cl::PolyTree>(
        &cu::expolygons_offset(&rings, fl(delta), DEFAULT_JOIN_TYPE, DEFAULT_MITER_LIMIT),
        fl(-fl(delta)),
        DEFAULT_JOIN_TYPE,
        DEFAULT_MITER_LIMIT,
    ))
}
/// ClipperUtils.cpp:619-624 `opening(const ExPolygons &expolygons, const float delta1, const float delta2)`
fn opening(expolygons: &[ExPolygon], delta1: f64, delta2: f64) -> Polygons {
    to_polygons_paths(&cu::expand_paths::<cl::Paths>(
        &cu::shrink_paths::<cl::Paths>(&expolygons_provider(expolygons), fl(delta1), DEFAULT_JOIN_TYPE, DEFAULT_MITER_LIMIT),
        fl(delta2),
        DEFAULT_JOIN_TYPE,
        DEFAULT_MITER_LIMIT,
    ))
}

/// ExPolygon.hpp `to_polygons(const ExPolygons &)`: the contour and the holes of every expolygon.
fn to_polygons(expolygons: &[ExPolygon]) -> Polygons {
    let mut out = Polygons::new();
    for e in expolygons {
        out.push(e.contour.clone());
        for hole in &e.holes {
            out.push(hole.clone());
        }
    }
    out
}

/// Polygon.cpp `get_extents(const Polygon &)`: the bounding box of the points.
fn get_extents_points(points: &[Point]) -> BoundingBox {
    let mut bbox = BoundingBox::new();
    for p in points {
        bbox.merge_point(*p);
    }
    bbox
}
/// ExPolygon.cpp:490-493 `get_extents(const ExPolygon &expolygon)`: the box of the contour.
fn get_extents_expolygon(expolygon: &ExPolygon) -> BoundingBox {
    get_extents_points(&expolygon.contour.points)
}
/// ExPolygon.cpp:495-504 `get_extents(const ExPolygons &expolygons)`
fn get_extents(expolygons: &[ExPolygon]) -> BoundingBox {
    let mut bbox = BoundingBox::new();
    for e in expolygons {
        if !e.contour.points.is_empty() {
            bbox.merge(&get_extents_expolygon(e));
        }
    }
    bbox
}
/// Surface.cpp:12-21 `get_extents(const Surfaces &surfaces)`
fn get_extents_surfaces(surfaces: &[Surface]) -> BoundingBox {
    let mut bbox = BoundingBox::new();
    for s in surfaces {
        bbox.merge(&get_extents_expolygon(&s.expolygon));
    }
    bbox
}
/// BoundingBox.hpp:57-60 `bool BoundingBoxBase::overlap(const BoundingBoxBase &other) const`
fn bbox_overlap(l: &BoundingBox, r: &BoundingBox) -> bool {
    !(l.max.x < r.min.x || l.min.x > r.max.x || l.max.y < r.min.y || l.min.y > r.max.y)
}

/// Point.hpp:199 `Point& operator*=(const double &rhs)` over the points of an expolygon:
/// ExPolygon.cpp:17-22 `void ExPolygon::scale(double factor)` (MultiPoint.cpp:6-10).
fn expolygon_scale(expolygon: &mut ExPolygon, factor: f64) {
    for p in expolygon.contour.points.iter_mut() {
        p.x = (p.x as f64 * factor) as i64;
        p.y = (p.y as f64 * factor) as i64;
    }
    for hole in expolygon.holes.iter_mut() {
        for p in hole.points.iter_mut() {
            p.x = (p.x as f64 * factor) as i64;
            p.y = (p.y as f64 * factor) as i64;
        }
    }
}

/// ExPolygon.cpp:69-74 `void ExPolygon::douglas_peucker(double tolerance)`
fn expolygon_douglas_peucker(expolygon: &mut ExPolygon, tolerance: f64) {
    expolygon.contour.douglas_peucker(tolerance);
    for poly in expolygon.holes.iter_mut() {
        poly.douglas_peucker(tolerance);
    }
}

/// ExPolygon.cpp:109-119 `bool ExPolygon::contains(const Point &point, bool border_result = true) const`
fn expolygon_contains(expolygon: &ExPolygon, point: &Point, border_result: bool) -> bool {
    if !crate::geometry::polygon::contains_polygon(&expolygon.contour, point, border_result) {
        // Outside the outer contour, not on the contour boundary.
        return false;
    }
    for hole in &expolygon.holes {
        if crate::geometry::polygon::contains_polygon(hole, point, !border_result) {
            // Inside a hole, not on the hole boundary.
            return false;
        }
    }
    true
}

/// ExPolygon.cpp:158-184 `bool ExPolygon::overlaps(const ExPolygon &other) const`
fn expolygon_overlaps(this: &ExPolygon, other: &ExPolygon) -> bool {
    if this.contour.points.is_empty() || other.contour.points.is_empty() {
        return false;
    }
    // Polylines pl_out = intersection_pl(to_polylines(other), *this);
    // ExPolygon.hpp:214-229 to_polylines: every ring closed by its first point.
    let mut subject = cl::Paths::new();
    let mut pl = path_of(&other.contour.points);
    pl.push(pl[0]);
    subject.push(pl);
    for hole in &other.holes {
        let mut pl = path_of(&hole.points);
        pl.push(pl[0]);
        subject.push(pl);
    }
    // ClipperUtils.cpp `intersection_pl(const Polylines &subject, const ExPolygon &clip)`:
    // _clipper_pl_open(ctIntersection, PolylinesProvider(subject), ExPolygonProvider(clip))
    let pl_out = cu::_clipper_pl_open(cl::ClipType::ctIntersection, &subject, &expolygons_provider(std::slice::from_ref(this)));
    // See unit test SCENARIO("Clipper diff with polyline", "[Clipper]")
    // for in which case the intersection_pl produces any intersection.
    !pl_out.is_empty() ||
        // If *this is completely inside other, then pl_out is empty, but the expolygons overlap. Test for that situation.
        expolygon_contains(other, &this.contour.points[0], true)
}

/// ExPolygon.cpp:186-195 `bool overlaps(const ExPolygons& expolys1, const ExPolygons& expolys2)`
fn overlaps(expolys1: &[ExPolygon], expolys2: &[ExPolygon]) -> bool {
    for expoly1 in expolys1 {
        for expoly2 in expolys2 {
            if expolygon_overlaps(expoly1, expoly2) {
                return true;
            }
        }
    }
    false
}

/// ExPolygon.hpp `to_expolygons(Surfaces &&)`: the expolygons of the surfaces.
fn to_expolygons(surfaces: &[Surface]) -> ExPolygons {
    surfaces.iter().map(|s| s.expolygon.clone()).collect()
}

// ---------------------------------------------------------------------------------------------
// What this file reads of Layer.cpp, LayerRegion.cpp and PrintRegion.cpp through the pointers a
// Layer and a LayerRegion have in C++.
// ---------------------------------------------------------------------------------------------

/// PrintRegion.cpp:21-59 `Flow PrintRegion::flow(const PrintObject &object, FlowRole role, double
/// layer_height, bool first_layer) const`, over the faithful Print.
fn print_region_flow(
    print: &Print,
    object: &PrintObject,
    region: &PrintRegion,
    role: FlowRole,
    layer_height: f64,
    first_layer: bool,
) -> Result<Flow, PrintObjectSliceError> {
    use crate::print_config::config_def::{multi_nozzle_diameter_mode_enabled, primary_nozzle_diameter};
    let print_config = print.config();
    let m_config = region.config();
    // Get extrusion width from configuration.
    // (an absolute value authored for the process preset, or zero for auto)
    // The infill of the initial layer has its own line width, which takes over the responsibility of
    // "initial_layer_line_width" for the infill roles only. Zero means to keep following "initial_layer_line_width".
    let infill_role = role == FlowRole::Infill || role == FlowRole::SolidInfill || role == FlowRole::TopSolidInfill;
    let mut config_width: f64 = if first_layer && infill_role && cfg_float(print_config, "initial_layer_infill_line_width") > 0. {
        cfg_float(print_config, "initial_layer_infill_line_width")
    } else if first_layer && cfg_float(print_config, "initial_layer_line_width") > 0. {
        cfg_float(print_config, "initial_layer_line_width")
    } else if role == FlowRole::ExternalPerimeter {
        cfg_float(m_config, "outer_wall_line_width")
    } else if role == FlowRole::Perimeter {
        cfg_float(m_config, "inner_wall_line_width")
    } else if role == FlowRole::Infill {
        cfg_float(m_config, "sparse_infill_line_width")
    } else if role == FlowRole::SolidInfill {
        cfg_float(m_config, "internal_solid_infill_line_width")
    } else if role == FlowRole::TopSolidInfill {
        cfg_float(m_config, "top_surface_line_width")
    } else {
        return Err(PrintObjectSliceError::InvalidArgument("Unknown role".to_string()));
    };

    if config_width == 0. {
        config_width = cfg_float(object.config(), "line_width");
    }

    // Get the configured nozzle_diameter for the extruder that this role's filament is grouped to.
    let nozzle_diameter = crate::print::faithful::nozzle_diameter_for_filament(print_config, region.extruder(role));
    crate::support::support_parameters::flow_new_from_config_width(
        role,
        &ConfigOptionFloat::with_value(config_width),
        primary_nozzle_diameter(print.full_print_config()) as f32,
        nozzle_diameter,
        layer_height as f32,
        multi_nozzle_diameter_mode_enabled(print.full_print_config()),
    )
    .map_err(|e| PrintObjectSliceError::InvalidArgument(format!("{:?}", e)))
}

/// LayerRegion.cpp:22-30 `Flow LayerRegion::flow(FlowRole role) const`: `m_region->flow(*m_layer->object(),
/// role, m_layer->height, m_layer->id() == 0)` of the region `region_id` of `layer`.
fn layer_region_flow(print: &Print, object: &PrintObject, layer: &Layer, region_id: usize, role: FlowRole) -> Result<Flow, PrintObjectSliceError> {
    let region = object.printing_region(layer.regions()[region_id].region_id());
    let region = region.borrow();
    print_region_flow(print, object, &region, role, layer.height, layer.id() == 0)
}

/// Layer.cpp:150-170 `ExPolygons Layer::merged(float offset_scaled) const`, the region
/// configurations read from the object's shared regions.
fn layer_merged(object: &PrintObject, layer: &Layer, offset_scaled: f32) -> ExPolygons {
    debug_assert!(offset_scaled >= 0.);
    let mut offset_scaled = offset_scaled;
    // If no offset is set, apply EPSILON offset before union, and revert it afterwards.
    let mut offset_scaled2: f32 = 0.;
    if offset_scaled == 0. {
        offset_scaled = EPSILON as f32;
        offset_scaled2 = -EPSILON as f32;
    }
    let mut polygons = Polygons::new();
    for layerm in layer.regions() {
        let region = object.printing_region(layerm.region_id());
        let region = region.borrow();
        let config = region.config();
        // Our users learned to bend Slic3r to produce empty volumes to act as subtracters. Only add the region if it is non-empty.
        if cfg_int(config, "bottom_shell_layers") > 0
            || cfg_int(config, "top_shell_layers") > 0
            || cfg_float(config, "sparse_infill_density") > 0.
            || cfg_int(config, "wall_loops") > 0
        {
            polygons.extend(offset_surfaces(&layerm.slices.surfaces, offset_scaled as f64));
        }
    }
    let mut out = union_ex(&polygons);
    if offset_scaled2 != 0. {
        out = offset_ex(&out, offset_scaled2 as f64);
    }
    out
}

/// ExPolygon.hpp:451-458 `ExPolygons expolygons_simplify(const ExPolygons &expolys, double tolerance)`:
/// `ExPolygon::simplify(tolerance)` of every expolygon (ExPolygon.cpp:231-261), which is
/// `union_ex(simplify_p(tolerance))`, `simplify_p` being the rings after `MultiPoint::_douglas_peucker`
/// (closed by their first point) put through `simplify_polygons` (ClipperUtils.cpp:1026-1041,
/// `ClipperLib::SimplifyPolygons(pftNonZero)`).
fn expolygons_simplify(expolys: &[ExPolygon], tolerance: f64) -> ExPolygons {
    let mut out = ExPolygons::with_capacity(expolys.len());
    for exp in expolys {
        let pp = exp.simplify_p_dp_rings_faithful(tolerance);
        let simplified = to_polygons_paths(&cl::simplify_polygons(&polygons_provider(&pp), cl::PolyFillType::pftNonZero));
        out.extend(union_ex(&simplified));
    }
    out
}

/// ElephantFootCompensation.cpp:620-635 `ExPolygons elephant_foot_compensation(const ExPolygons &input, const
/// Flow &external_perimeter_flow, const double compensation)`: the expolygons simplified by SCALED_EPSILON,
/// each compensated with `min_contour_width = double(flow.width() + flow.spacing())` (a sum of floats).
fn elephant_foot_compensation(input: &ExPolygons, external_perimeter_flow: &Flow, compensation: f64) -> ExPolygons {
    let simplified_exps = expolygons_simplify(input, EPSILON / SCALING_FACTOR);
    let mut out = ExPolygons::with_capacity(simplified_exps.len());
    // The contour shall be wide enough to apply the external perimeter plus compensation on both sides.
    let min_contour_width = (external_perimeter_flow.width() as f32 + external_perimeter_flow.spacing() as f32) as f64;
    for expoly in &simplified_exps {
        out.push(crate::elephant_foot_compensation::elephant_foot_compensation_with_width(expoly, min_contour_width, compensation));
    }
    out
}

/// PrintBase.hpp `PrintObjectBaseWithState::active_step_add_warning(warning_level, message, message_id = 0)`
fn active_step_add_warning(print: &Print, object: &mut PrintObject, warning_level: WarningLevel, message: &str) {
    let (step, update) = object.m_state.active_step_add_warning(warning_level, message, 0, print.base.base.state_mutex());
    if update {
        print.base.base.status_update_warnings(
            step as i32,
            warning_level,
            message,
            Some(object.id()),
            crate::print_base::SlicingNotificationType::SlicingDefaultNotification,
        );
    }
}

// ---------------------------------------------------------------------------------------------
// PrintObjectSlice.cpp
// ---------------------------------------------------------------------------------------------

/// PrintObjectSlice.cpp:37-60 `LayerPtrs new_layers(PrintObject *print_object, const std::vector<coordf_t>
/// &object_layers)`. `object_layers`: object layers (pairs of bottom/top Z coordinate), without the raft.
pub fn new_layers(print_object: &PrintObject, object_layers: &[f64]) -> Vec<Layer> {
    let mut out: Vec<Layer> = Vec::with_capacity(object_layers.len());
    let mut id = print_object.slicing_parameters().raft_layers() as i32;
    let zmin = print_object.slicing_parameters().object_print_z_min;
    // Layer *prev = nullptr: the layers link by index.
    let mut i_layer = 0;
    while i_layer < object_layers.len() {
        let lo = object_layers[i_layer];
        let hi = object_layers[i_layer + 1];
        let slice_z = 0.5 * (lo + hi);
        let mut layer = Layer::new(id as usize, print_object.m_model_object, hi - lo, hi + zmin, slice_z);
        id += 1;
        let k = out.len();
        if k > 0 {
            out[k - 1].upper_layer_id = Some(k);
            layer.lower_layer_id = Some(k - 1);
        }
        out.push(layer);
        i_layer += 2;
    }
    out
}

/// TriangleMeshSlicer.hpp:11-47 `struct MeshSlicingParamsEx`, as this file fills it. The slicer of
/// this crate reads the transformation as `frame_trafo`; `trafo` is kept here in full.
#[derive(Clone, Debug)]
pub struct MeshSlicingParamsEx {
    pub mode: MeshSlicingMode,
    /// For vase mode: below this layer a different slicing mode will be used.
    pub slicing_mode_normal_below_layer: usize,
    /// Mode to apply below slicing_mode_normal_below_layer.
    pub mode_below: MeshSlicingMode,
    /// Transforming faces during the slicing.
    pub trafo: Transform3d,
    /// Morphological closing operation when creating output expolygons, unscaled.
    pub closing_radius: f32,
    /// Positive offset applied when creating output expolygons, unscaled.
    pub extra_offset: f32,
    /// Resolution for contour simplification, unscaled. 0 = don't simplify.
    pub resolution: f64,
}

impl Default for MeshSlicingParamsEx {
    fn default() -> Self {
        MeshSlicingParamsEx {
            mode: MeshSlicingMode::Regular,
            slicing_mode_normal_below_layer: 0,
            mode_below: MeshSlicingMode::Regular,
            trafo: eigen::identity(),
            closing_radius: 0.,
            extra_offset: 0.,
            resolution: 0.,
        }
    }
}

/// TriangleMeshSlicer.cpp `std::vector<ExPolygons> slice_mesh_ex(const indexed_triangle_set &mesh, const
/// std::vector<float> &zs, const MeshSlicingParamsEx &params, std::function<void()> throw_on_cancel)`: the
/// slicer of this crate with `params.trafo` as its `frame_trafo`.
fn slice_mesh_ex(mesh: &indexed_triangle_set, zs: &[f32], params: &MeshSlicingParamsEx) -> Vec<ExPolygons> {
    let mut p = SlicerParamsEx::default();
    p.base.mode = params.mode;
    p.base.slicing_mode_normal_below_layer = params.slicing_mode_normal_below_layer;
    p.base.mode_below = params.mode_below;
    let mut t16 = [0f64; 16];
    for r in 0..4 {
        for c in 0..4 {
            t16[r * 4 + c] = params.trafo[(r, c)];
        }
    }
    p.base.frame_trafo = Some(t16);
    p.closing_radius = params.closing_radius;
    p.extra_offset = params.extra_offset;
    p.resolution = params.resolution;
    slice_mesh_ex_its(mesh, zs, &p, &|| {})
}

/// PrintObjectSlice.cpp:62-129 `static std::vector<ExPolygons> slice_volume(const ModelVolume &volume, const
/// std::vector<float> &zs, const MeshSlicingParamsEx &params, const std::function<void()> &throw_on_cancel_callback)`:
/// slice single triangle mesh.
pub fn slice_volume(
    volume: &ModelVolume,
    zs: &[f32],
    params: &MeshSlicingParamsEx,
    throw_on_cancel_callback: ThrowOnCancel,
) -> Result<Vec<ExPolygons>, CanceledException> {
    let mut layers: Vec<ExPolygons> = Vec::new();
    if !zs.is_empty() {
        let mesh = volume.mesh();
        let mut its: &indexed_triangle_set = &mesh.its;
        let mut its_flipped;
        if !its.indices.is_empty() {
            let mut params2 = params.clone();
            params2.trafo = eigen::transform_mul(&params2.trafo, &volume.get_matrix(false, false, false, false));
            // params2.trafo.rotation().determinant() < 0.: Transform::rotation() is the rotation of
            // computeRotationScaling().
            if eigen::determinant3(&eigen::compute_rotation_scaling(&eigen::linear(&params2.trafo)).0) < 0. {
                // 仅当模型存在镜像时才复制its并进行翻转
                its_flipped = mesh.its.clone();
                crate::triangle_mesh::its_flip_triangles(&mut its_flipped);
                its = &its_flipped;
            }
            // Only slice Z planes that hit this volume. Lightbox-style parts are ~1mm tall
            // while the object may have many empty layers above them.
            // zs是整个object的高度范围，可以比当前volume的高度范围更大，只对当前volume高度范围内的zs进行切片可以加速
            let tm = &params2.trafo;
            let mut zmin = f32::MAX;
            let mut zmax = f32::MIN;
            for v in &its.vertices {
                // float(tm(2, 0) * double(v.x()) + tm(2, 1) * double(v.y()) + tm(2, 2) * double(v.z()) + tm(2, 3))
                let z = (tm[(2, 2)].mul_add(v.z as f64, tm[(2, 0)].mul_add(v.x as f64, tm[(2, 1)] * v.y as f64)) + tm[(2, 3)]) as f32;
                zmin = zmin.min(z);
                zmax = zmax.max(z);
            }
            let mut zs_hit: Vec<f32> = Vec::with_capacity(zs.len());
            let mut zs_idx: Vec<usize> = Vec::with_capacity(zs.len());
            for i in 0..zs.len() {
                if zs[i] >= zmin - EPSILON as f32 && zs[i] <= zmax + EPSILON as f32 {
                    zs_hit.push(zs[i]);
                    zs_idx.push(i);
                }
            }

            if zs_hit.is_empty() {
                layers = vec![ExPolygons::new(); zs.len()];
            } else if zs_hit.len() == zs.len() {
                layers = slice_mesh_ex(its, zs, &params2);
            } else {
                // slicing_mode_normal_below_layer is an index into the full zs array (vase mode).
                // Remap it onto the zs_hit subset so the bottom-shell mode does not shift.
                let mut params_hit = params2.clone();
                if params_hit.slicing_mode_normal_below_layer > 0 {
                    let mut below_cnt: usize = 0;
                    while below_cnt < zs_idx.len() && zs_idx[below_cnt] < params2.slicing_mode_normal_below_layer {
                        below_cnt += 1;
                    }
                    params_hit.slicing_mode_normal_below_layer = below_cnt;
                }
                // scatter_slices
                let mut sliced = slice_mesh_ex(its, &zs_hit, &params_hit);
                layers = vec![ExPolygons::new(); zs.len()];
                let n = zs_idx.len().min(sliced.len());
                for i in 0..n {
                    layers[zs_idx[i]] = std::mem::take(&mut sliced[i]);
                }
            }
            throw_on_cancel_callback()?;
        }
    }
    Ok(layers)
}

/// PrintObjectSlice.cpp:131-170 `static std::vector<ExPolygons> slice_volume(const ModelVolume &volume, const
/// std::vector<float> &z, const std::vector<t_layer_height_range> &ranges, const MeshSlicingParamsEx &params, const
/// std::function<void()> &throw_on_cancel_callback)`: slice single triangle mesh. Filter the zs not inside
/// the ranges. The ranges are closed at the bottom and open at the top, they are sorted lexicographically
/// and non overlapping.
pub fn slice_volume_ranges(
    volume: &ModelVolume,
    z: &[f32],
    ranges: &[(f64, f64)],
    params: &MeshSlicingParamsEx,
    throw_on_cancel_callback: ThrowOnCancel,
) -> Result<Vec<ExPolygons>, CanceledException> {
    let mut out: Vec<ExPolygons> = Vec::new();
    if !z.is_empty() && !ranges.is_empty() {
        if ranges.len() == 1 && z[0] as f64 >= ranges[0].0 && (z[z.len() - 1] as f64) < ranges[0].1 {
            // All layers fit into a single range.
            out = slice_volume(volume, z, params, throw_on_cancel_callback)?;
        } else {
            let mut z_filtered: Vec<f32> = Vec::with_capacity(z.len());
            let mut n_filtered: Vec<(usize, usize)> = Vec::with_capacity(2 * ranges.len());
            let mut i: usize = 0;
            for range in ranges {
                while i < z.len() && (z[i] as f64) < range.0 {
                    i += 1;
                }
                let first = i;
                while i < z.len() && (z[i] as f64) < range.1 {
                    z_filtered.push(z[i]);
                    i += 1;
                }
                if i > first {
                    n_filtered.push((first, i));
                }
            }
            if !n_filtered.is_empty() {
                let mut layers = slice_volume(volume, &z_filtered, params, throw_on_cancel_callback)?;
                out = vec![ExPolygons::new(); z.len()];
                i = 0;
                for span in &n_filtered {
                    for j in span.0..span.1 {
                        out[j] = std::mem::take(&mut layers[i]);
                        i += 1;
                    }
                }
            }
        }
    }
    Ok(out)
}

/// PrintObjectSlice.cpp:171-175 `static inline bool model_volume_needs_slicing(const ModelVolume &mv)`
pub fn model_volume_needs_slicing(mv: &ModelVolume) -> bool {
    let type_ = mv.type_();
    type_ == ModelVolumeType::ModelPart || type_ == ModelVolumeType::NegativeVolume || type_ == ModelVolumeType::ParameterModifier
}

/// Model.hpp:1434-1437 `model_volumes_sort_by_id(ModelVolumePtrs &model_volumes)` over the copy of the
/// pointers the C++ functions take: the indices of the volumes, in the order of their ids.
fn model_volumes_sort_by_id(model_volumes: &[ModelVolume]) -> Vec<usize> {
    let mut order: Vec<usize> = (0..model_volumes.len()).collect();
    order.sort_by(|&l, &r| model_volumes[l].id().cmp(&model_volumes[r].id()));
    order
}

/// PrintObjectSlice.cpp:177-290 `static std::vector<VolumeSlices> slice_volumes_inner(const PrintConfig
/// &print_config, const PrintObjectConfig &print_object_config, const Transform3d &object_trafo, ModelVolumePtrs
/// model_volumes, const std::vector<PrintObjectRegions::LayerRangeRegions> &layer_ranges, const std::vector<float>
/// &zs, const std::function<void()> &throw_on_cancel_callback, const Print *print = nullptr)`.
///
/// Slice printable volumes, negative volumes and modifier volumes, sorted by ModelVolume::id().
/// Apply closing radius.
/// Apply positive XY compensation to ModelVolumeType::MODEL_PART and ModelVolumeType::PARAMETER_MODIFIER, not to ModelVolumeType::NEGATIVE_VOLUME.
/// Apply contour simplification.
#[allow(clippy::too_many_arguments)]
pub fn slice_volumes_inner(
    print_config: &DynamicPrintConfig,
    print_object_config: &DynamicPrintConfig,
    object_trafo: &Transform3d,
    model_volumes: &[ModelVolume],
    layer_ranges: &[LayerRangeRegions],
    zs: &[f32],
    throw_on_cancel_callback: ThrowOnCancel,
    print: Option<&Print>,
) -> Result<Vec<VolumeSlices>, CanceledException> {
    let sorted = model_volumes_sort_by_id(model_volumes);

    let mut out: Vec<VolumeSlices> = Vec::with_capacity(model_volumes.len());

    let mut slicing_ranges: Vec<(f64, f64)> = Vec::new();
    if layer_ranges.len() > 1 {
        slicing_ranges.reserve(layer_ranges.len());
    }

    let mut params_base = MeshSlicingParamsEx::default();
    params_base.closing_radius = cfg_float(print_object_config, "slice_closing_radius") as f32;
    params_base.extra_offset = 0.;
    params_base.trafo = *object_trafo;
    //BBS: 0.0025mm is safe enough to simplify the data to speed slicing up for high-resolution model.
    //Also has on influence on arc fitting which has default resolution 0.0125mm.
    params_base.resolution = if cfg_float(print_config, "resolution") <= 0.001 { 0.0f32 as f64 } else { 0.0025 };
    // PrintConfig.hpp:157-165 `enum class SlicingMode { Regular, EvenOdd, CloseHoles }`
    match cfg_int(print_object_config, "slicing_mode") {
        0 => params_base.mode = MeshSlicingMode::Regular,
        1 => params_base.mode = MeshSlicingMode::EvenOdd,
        2 => params_base.mode = MeshSlicingMode::Positive,
        _ => {}
    }

    params_base.mode_below = params_base.mode;

    // BBS
    let num_extruders = cfg_floats(print_config, "filament_diameter").len();
    let _is_mm_painted = num_extruders > 1 && model_volumes.iter().any(|mv| mv.is_mm_painted());
    // BBS: don't do size compensation when slice volume.
    // Will handle contour and hole size compensation seperately later.
    //const auto   extra_offset  = is_mm_painted ? 0.f : std::max(0.f, float(print_object_config.xy_contour_compensation.value));
    let extra_offset: f32 = 0.;

    struct SliceVolumeJob {
        model_volume: usize,
        params: MeshSlicingParamsEx,
        slicing_ranges: Vec<(f64, f64)>,
        use_ranges: bool,
    }
    let mut jobs: Vec<SliceVolumeJob> = Vec::with_capacity(model_volumes.len());

    for &mv_idx in &sorted {
        let model_volume = &model_volumes[mv_idx];
        if model_volume_needs_slicing(model_volume) {
            let mut params = params_base.clone();
            if !model_volume.is_negative_volume() {
                params.extra_offset = extra_offset;
            }
            if layer_ranges.len() == 1 {
                let layer_range = &layer_ranges[0];
                if layer_range.has_volume(model_volume.id()) {
                    if model_volume.is_model_part() && cfg_bool(print_config, "spiral_mode") {
                        let it = layer_range.volume_regions.iter().find(|slice| slice.model_volume == Some(mv_idx)).expect("the volume region of a model part");
                        params.mode = MeshSlicingMode::PositiveLargestContour;
                        // Slice the bottom layers with SlicingMode::Regular.
                        // This needs to be in sync with LayerRegion::make_perimeters() spiral_mode!
                        let region = it.region.as_ref().expect("the region of a model part").borrow();
                        let region_config = region.config();
                        params.slicing_mode_normal_below_layer = cfg_int(region_config, "bottom_shell_layers") as usize;
                        while params.slicing_mode_normal_below_layer < zs.len()
                            && (zs[params.slicing_mode_normal_below_layer] as f64) < cfg_float(region_config, "bottom_shell_thickness") - EPSILON
                        {
                            params.slicing_mode_normal_below_layer += 1;
                        }
                    }
                    jobs.push(SliceVolumeJob { model_volume: mv_idx, params, slicing_ranges: Vec::new(), use_ranges: false });
                }
            } else {
                debug_assert!(!cfg_bool(print_config, "spiral_mode"));
                slicing_ranges.clear();
                for layer_range in layer_ranges {
                    if layer_range.has_volume(model_volume.id()) {
                        slicing_ranges.push(layer_range.layer_height_range);
                    }
                }
                if !slicing_ranges.is_empty() {
                    jobs.push(SliceVolumeJob { model_volume: mv_idx, params, slicing_ranges: slicing_ranges.clone(), use_ranges: true });
                }
            }
        }
    }

    let mut results: Vec<VolumeSlices> = (0..jobs.len()).map(|_| VolumeSlices::default()).collect();
    let mut done_count: i32 = 0;
    // tbb::parallel_for over the jobs, in order here.
    for i in 0..jobs.len() {
        let job = &jobs[i];
        let model_volume = &model_volumes[job.model_volume];
        let slices = if job.use_ranges {
            slice_volume_ranges(model_volume, zs, &job.slicing_ranges, &job.params, throw_on_cancel_callback)?
        } else {
            slice_volume(model_volume, zs, &job.params, throw_on_cancel_callback)?
        };
        results[i] = VolumeSlices { volume_id: model_volume.id(), slices };
        done_count += 1;
        let done = done_count;
        if let Some(print) = print {
            if done == jobs.len() as i32 || (done & 31) == 0 {
                let pct = 5 + 5 * done / (jobs.len() as i32).max(1);
                print.base.base.set_status(pct.min(10), "Slicing mesh", SlicingStatus::DEFAULT, -1);
            }
        }
    }
    for vs in results {
        if !vs.slices.is_empty() {
            out.push(vs);
        }
    }

    Ok(out)
}

/// PrintObjectSlice.cpp:292-297 `static inline VolumeSlices& volume_slices_find_by_id(std::vector<VolumeSlices>
/// &volume_slices, const ObjectID id)`: the index of the slices of the volume.
pub fn volume_slices_find_by_id(volume_slices: &[VolumeSlices], id: ObjectID) -> usize {
    let it = volume_slices.partition_point(|vs| vs.volume_id < id);
    debug_assert!(it != volume_slices.len() && volume_slices[it].volume_id == id);
    it
}

/// PrintObjectSlice.cpp:299-303 `static inline bool overlap_in_xy(const PrintObjectRegions::BoundingBox &l, const
/// PrintObjectRegions::BoundingBox &r)`
pub fn overlap_in_xy(l: &AlignedBox3f, r: &AlignedBox3f) -> bool {
    !(l.max[0] < r.min[0] || l.min[0] > r.max[0] || l.max[1] < r.min[1] || l.min[1] > r.max[1])
}

/// PrintObjectSlice.cpp:305-317 `static std::vector<PrintObjectRegions::LayerRangeRegions>::const_iterator
/// layer_range_first(const std::vector<PrintObjectRegions::LayerRangeRegions> &layer_ranges, double z)`: the
/// index of the range.
pub fn layer_range_first(layer_ranges: &[LayerRangeRegions], z: f64) -> usize {
    // lower_bound_by_predicate
    let mut it = layer_ranges.partition_point(|lr| lr.layer_height_range.1 < z && (lr.layer_height_range.1 - z).abs() > EPSILON);
    debug_assert!(it != layer_ranges.len() && layer_ranges[it].layer_height_range.0 <= z && z <= layer_ranges[it].layer_height_range.1);
    if z == layer_ranges[it].layer_height_range.1 {
        let it_next = it + 1;
        if it_next != layer_ranges.len() && layer_ranges[it_next].layer_height_range.0 == z {
            it = it_next;
        }
    }
    debug_assert!(it != layer_ranges.len() && layer_ranges[it].layer_height_range.0 <= z && z <= layer_ranges[it].layer_height_range.1);
    it
}

/// PrintObjectSlice.cpp:319-328 `static std::vector<PrintObjectRegions::LayerRangeRegions>::const_iterator
/// layer_range_next(const std::vector<PrintObjectRegions::LayerRangeRegions> &layer_ranges,
/// std::vector<PrintObjectRegions::LayerRangeRegions>::const_iterator it, double z)`
pub fn layer_range_next(layer_ranges: &[LayerRangeRegions], it: usize, z: f64) -> usize {
    let mut it = it;
    while layer_ranges[it].layer_height_range.1 <= z + EPSILON {
        it += 1;
        debug_assert!(it != layer_ranges.len());
    }
    debug_assert!(it != layer_ranges.len() && layer_ranges[it].layer_height_range.0 <= z && z < layer_ranges[it].layer_height_range.1);
    it
}

/// `region.bbox`: the bounding box of the volume of a VolumeRegion.
fn volume_region_bbox<'a>(layer_range: &'a LayerRangeRegions, idx_region: usize) -> &'a AlignedBox3f {
    &layer_range.volumes[layer_range.volume_regions[idx_region].bbox.expect("the bounding box of a volume region")].bbox
}

/// `region.region->print_object_region_id()` of a VolumeRegion, -1 without a region.
fn volume_region_id(layer_range: &LayerRangeRegions, idx_region: usize) -> i32 {
    match &layer_range.volume_regions[idx_region].region {
        Some(r) => r.borrow().print_object_region_id(),
        None => -1,
    }
}

/// PrintObjectSlice.cpp:330-642 `static std::vector<std::vector<ExPolygons>> slices_to_regions(ModelVolumePtrs
/// model_volumes, const PrintObjectRegions &print_object_regions, const std::vector<float> &zs,
/// std::vector<VolumeSlices> &&volume_slices, const bool clip_multipart_objects, const bool
/// order_independent_overlap_carving, const std::function<void()> &throw_on_cancel_callback)`.
///
/// `clip_multipart_objects`: if clipping is disabled, then ExPolygons produced by different volumes will
/// never be merged, thus they will be allowed to overlap. It is up to the model designer to handle these
/// overlaps.
#[allow(clippy::too_many_arguments)]
pub fn slices_to_regions(
    model_volumes: &[ModelVolume],
    print_object_regions: &PrintObjectRegions,
    zs: &[f32],
    volume_slices: Vec<VolumeSlices>,
    clip_multipart_objects_: bool,
    order_independent_overlap_carving: bool,
    throw_on_cancel_callback: ThrowOnCancel,
) -> Result<Vec<Vec<ExPolygons>>, CanceledException> {
    // model_volumes_sort_by_id(model_volumes): the volumes are reached through the regions below.
    let mut volume_slices = volume_slices;
    let mv = |layer_range: &LayerRangeRegions, idx_region: usize| -> &ModelVolume {
        &model_volumes[layer_range.volume_regions[idx_region].model_volume.expect("the model volume of a volume region")]
    };

    let mut slices_by_region: Vec<Vec<ExPolygons>> = vec![vec![ExPolygons::new(); zs.len()]; print_object_regions.all_regions.len()];

    // First shuffle slices into regions if there is no overlap with another region possible, collect zs of the complex cases.
    let mut zs_complex: Vec<(usize, f32)> = Vec::new();
    {
        let mut z_idx: usize = 0;
        for layer_range in &print_object_regions.layer_ranges {
            while z_idx < zs.len() && (zs[z_idx] as f64) < layer_range.layer_height_range.0 {
                z_idx += 1;
            }
            if layer_range.volume_regions.is_empty() {
            } else if layer_range.volume_regions.len() == 1 {
                let model_volume = mv(layer_range, 0);
                if model_volume.is_model_part() {
                    let src = volume_slices_find_by_id(&volume_slices, model_volume.id());
                    let dst = volume_region_id(layer_range, 0) as usize;
                    while z_idx < zs.len() && (zs[z_idx] as f64) < layer_range.layer_height_range.1 {
                        slices_by_region[dst][z_idx] = std::mem::take(&mut volume_slices[src].slices[z_idx]);
                        z_idx += 1;
                    }
                }
            } else {
                zs_complex.reserve(zs.len());
                while z_idx < zs.len() && (zs[z_idx] as f64) < layer_range.layer_height_range.1 {
                    let z = zs[z_idx];
                    let mut idx_first_printable_region: i32 = -1;
                    let mut complex = false;
                    let mut printable_region_ids: Vec<i32> = Vec::new();
                    for idx_region in 0..layer_range.volume_regions.len() as i32 {
                        let bbox = volume_region_bbox(layer_range, idx_region as usize);
                        if bbox.min[2] <= z && bbox.max[2] >= z {
                            let is_model_part = mv(layer_range, idx_region as usize).is_model_part();
                            if is_model_part {
                                printable_region_ids.push(idx_region);
                            }
                            if idx_first_printable_region == -1 && is_model_part {
                                idx_first_printable_region = idx_region;
                            } else if idx_first_printable_region != -1 {
                                // Test for overlap with some other region.
                                for idx_region2 in idx_first_printable_region..idx_region {
                                    let bbox2 = volume_region_bbox(layer_range, idx_region2 as usize);
                                    if bbox2.min[2] <= z && bbox2.max[2] >= z && overlap_in_xy(bbox, bbox2) {
                                        complex = true;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                    if complex {
                        zs_complex.push((z_idx, z));
                    } else if idx_first_printable_region >= 0 {
                        for &printable_region_id in &printable_region_ids {
                            let rid = volume_region_id(layer_range, printable_region_id as usize) as usize;
                            let src = volume_slices_find_by_id(&volume_slices, mv(layer_range, printable_region_id as usize).id());
                            let moved = std::mem::take(&mut volume_slices[src].slices[z_idx]);
                            slices_by_region[rid][z_idx].extend(moved);
                        }
                    }
                    z_idx += 1;
                }
            }
            throw_on_cancel_callback()?;
        }
    }

    // Second perform region clipping and assignment in parallel.
    if !zs_complex.is_empty() {
        let mut layer_ranges_regions_to_slices: Vec<Vec<usize>> = vec![Vec::new(); print_object_regions.layer_ranges.len()];
        for (lr_idx, layer_range) in print_object_regions.layer_ranges.iter().enumerate() {
            let layer_range_regions_to_slices = &mut layer_ranges_regions_to_slices[lr_idx];
            layer_range_regions_to_slices.reserve(layer_range.volume_regions.len());
            for idx_region in 0..layer_range.volume_regions.len() {
                layer_range_regions_to_slices.push(volume_slices_find_by_id(&volume_slices, mv(layer_range, idx_region).id()));
            }
        }
        // tbb::parallel_for(tbb::blocked_range<size_t>(0, zs_complex.size()), ...): one range here.
        {
            let range = (0usize, zs_complex.len());
            let z = zs_complex[range.0].1;
            let mut it_layer_range = layer_range_first(&print_object_regions.layer_ranges, z as f64);
            // Per volume_regions slices at this Z height.
            struct RegionSlice {
                expolygons: ExPolygons,
                // Identifier of this region in PrintObjectRegions::all_regions
                region_id: i32,
                #[allow(dead_code)]
                volume_id: ObjectID,
            }
            // bool operator<(const RegionSlice &rhs) const
            fn region_slice_lt(this: &RegionSlice, rhs: &RegionSlice) -> bool {
                let this_empty = this.region_id < 0 || this.expolygons.is_empty();
                let rhs_empty = rhs.region_id < 0 || rhs.expolygons.is_empty();
                // Sort the empty items to the end of the list.
                // Sort by region_id & volume_id lexicographically.
                // (`volume_id < volume_id` of the C++ compares the member with itself.)
                !this_empty && (rhs_empty || (this.region_id < rhs.region_id || (this.region_id == rhs.region_id && false)))
            }

            let mut temp_slices: Vec<RegionSlice> = Vec::new();
            let mut slice_bboxes: Vec<BoundingBox> = Vec::new();
            for zs_complex_idx in range.0..range.1 {
                let (z_idx, z) = zs_complex[zs_complex_idx];
                it_layer_range = layer_range_next(&print_object_regions.layer_ranges, it_layer_range, z as f64);
                let layer_range = &print_object_regions.layer_ranges[it_layer_range];
                {
                    let layer_range_regions_to_slices = &layer_ranges_regions_to_slices[it_layer_range];
                    // Per volume_regions slices at this Z height.
                    temp_slices.clear();
                    temp_slices.reserve(layer_range.volume_regions.len());
                    for (idx_region, &slices) in layer_range_regions_to_slices.iter().enumerate() {
                        temp_slices.push(RegionSlice {
                            expolygons: std::mem::take(&mut volume_slices[slices].slices[z_idx]),
                            region_id: volume_region_id(layer_range, idx_region),
                            volume_id: mv(layer_range, idx_region).id(),
                        });
                    }
                }
                let mut has_modifier = false;
                let mut n_model_parts: i32 = 0;
                let mut single_region_id: i32 = -1;
                let mut single_region = true;
                for idx_region in 0..layer_range.volume_regions.len() {
                    if temp_slices[idx_region].expolygons.is_empty() {
                        continue;
                    }
                    let m = mv(layer_range, idx_region);
                    if m.is_modifier() {
                        has_modifier = true;
                    } else if m.is_model_part() {
                        n_model_parts += 1;
                        let region_id = temp_slices[idx_region].region_id;
                        if single_region_id == -1 {
                            single_region_id = region_id;
                        } else if single_region_id != region_id {
                            single_region = false;
                        }
                    }
                }

                let bbox_volume = |b: &AlignedBox3f| -> f64 {
                    // const auto sz = b.sizes();
                    let sz = [b.max[0] - b.min[0], b.max[1] - b.min[1], b.max[2] - b.min[2]];
                    sz[0] as f64 * sz[1] as f64 * sz[2] as f64
                };

                // Fast path for lithophane / lightbox style models: thousands of same-material
                // islands. All parts share a single PrintRegion here, so clipping them against
                // each other is a no-op once they get unioned; only the negative volumes still
                // have to be subtracted. Restricted to one region on purpose: across regions a
                // later volume carves the earlier ones, and merging by region would replace that
                // per-volume order with the order the regions happen to appear in.
                if !has_modifier && clip_multipart_objects_ && n_model_parts >= 2 && single_region && single_region_id >= 0 {
                    let mut parts = ExPolygons::new();
                    let mut n_appended: i32 = 0;
                    for idx_region in 0..layer_range.volume_regions.len() {
                        if temp_slices[idx_region].expolygons.is_empty() {
                            continue;
                        }
                        let m = mv(layer_range, idx_region);
                        if m.is_negative_volume() {
                            // A negative volume only subtracts from the volumes preceding it,
                            // matching the generic path below.
                            if !parts.is_empty() {
                                parts = diff_ex(&parts, &temp_slices[idx_region].expolygons);
                            }
                            continue;
                        }
                        if !m.is_model_part() {
                            continue;
                        }
                        if parts.is_empty() {
                            parts = std::mem::take(&mut temp_slices[idx_region].expolygons);
                        } else {
                            parts.extend(std::mem::take(&mut temp_slices[idx_region].expolygons));
                        }
                        n_appended += 1;
                    }
                    if n_appended > 1 {
                        parts = closing_ex(&parts, EPSILON / SCALING_FACTOR);
                    }
                    slices_by_region[single_region_id as usize][z_idx] = parts;
                    throw_on_cancel_callback()?;
                    continue;
                }

                // Bounding box of what each volume actually contributes to this layer. The volume
                // bbox spans its whole 3D extent, so it lets through many pairs whose slices at
                // this Z cannot possibly intersect. Computed once, before any clipping shrinks the
                // slices, so it stays a conservative filter.
                slice_bboxes.clear();
                slice_bboxes.resize(temp_slices.len(), BoundingBox::new());
                for i in 0..temp_slices.len() {
                    if !temp_slices[i].expolygons.is_empty() {
                        slice_bboxes[i] = get_extents(&temp_slices[i].expolygons);
                    }
                }

                for idx_region in 0..layer_range.volume_regions.len() {
                    if !temp_slices[idx_region].expolygons.is_empty() {
                        let region = &layer_range.volume_regions[idx_region];
                        let model_volume = mv(layer_range, idx_region);
                        if model_volume.is_modifier() {
                            debug_assert!(region.parent > -1);
                            let next_region_same_modifier = idx_region + 1 < temp_slices.len()
                                && layer_range.volume_regions[idx_region + 1].model_volume == region.model_volume;
                            let parent = region.parent as usize;
                            let source = std::mem::take(&mut temp_slices[idx_region].expolygons);
                            let skip_periodic = match &region.region {
                                Some(r) => {
                                    let r = r.borrow();
                                    cfg_bool(r.config(), "periodic_modifier")
                                        && !crate::print_object::faithful::periodic_modifier_active(r.config(), z_idx as i32)
                                }
                                None => false,
                            };
                            if skip_periodic || temp_slices[parent].expolygons.is_empty() {
                                temp_slices[idx_region].expolygons.clear();
                            } else {
                                temp_slices[idx_region].expolygons = intersection_ex(&temp_slices[parent].expolygons, &source);
                                temp_slices[parent].expolygons = diff_ex(&temp_slices[parent].expolygons, &source);
                            }
                            if next_region_same_modifier {
                                // To be used in the following iteration.
                                temp_slices[idx_region + 1].expolygons = source;
                            }
                        } else if (model_volume.is_model_part() && clip_multipart_objects_) || model_volume.is_negative_volume() {
                            // Clip every non-zero region preceding it.
                            let current_is_negative = model_volume.is_negative_volume();
                            let current_vol = if current_is_negative { 0.0 } else { bbox_volume(volume_region_bbox(layer_range, idx_region)) };
                            for idx_region2 in 0..idx_region {
                                if !temp_slices[idx_region2].expolygons.is_empty() {
                                    // Nothing left to carve with, and an empty subtrahend would be a no-op anyway.
                                    if temp_slices[idx_region].expolygons.is_empty() {
                                        break;
                                    }
                                    // Skip trim_overlap for now, because it slow down the performace so much for some special cases
                                    let model_volume2 = mv(layer_range, idx_region2);
                                    if !model_volume2.is_negative_volume()
                                        && overlap_in_xy(volume_region_bbox(layer_range, idx_region), volume_region_bbox(layer_range, idx_region2))
                                    {
                                        // Two model parts sharing a PrintRegion are unioned by the merge step below,
                                        // so whichever way they carve each other the union is the same. Skipping the
                                        // clip is exact, and on multi-material lithophane style models it removes the
                                        // bulk of the Boolean operations.
                                        // Disabled when a modifier is present: the modifier reads the parent slice
                                        // before the union, so it must observe the carved parent.
                                        if !has_modifier
                                            && !current_is_negative
                                            && model_volume2.is_model_part()
                                            && temp_slices[idx_region].region_id == temp_slices[idx_region2].region_id
                                        {
                                            continue;
                                        }
                                        if !bbox_overlap(&slice_bboxes[idx_region], &slice_bboxes[idx_region2]) {
                                            continue;
                                        }
                                        let current_carves = !order_independent_overlap_carving
                                            || current_is_negative
                                            || current_vol <= bbox_volume(volume_region_bbox(layer_range, idx_region2));
                                        if current_carves {
                                            temp_slices[idx_region2].expolygons =
                                                diff_ex(&temp_slices[idx_region2].expolygons, &temp_slices[idx_region].expolygons);
                                        } else {
                                            temp_slices[idx_region].expolygons =
                                                diff_ex(&temp_slices[idx_region].expolygons, &temp_slices[idx_region2].expolygons);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                // Sort by region_id, push empty slices to the end.
                // std::sort(temp_slices.begin(), temp_slices.end()): the permutation libc++ makes.
                {
                    let mut order: Vec<usize> = (0..temp_slices.len()).collect();
                    crate::clipper::libcxx_sort(&mut order, |&a, &b| region_slice_lt(&temp_slices[a], &temp_slices[b]));
                    let mut taken: Vec<Option<RegionSlice>> = temp_slices.drain(..).map(Some).collect();
                    for &i in &order {
                        temp_slices.push(taken[i].take().expect("a permutation"));
                    }
                }
                // Remove the empty slices.
                if let Some(pos) = temp_slices.iter().position(|slice| slice.region_id == -1 || slice.expolygons.is_empty()) {
                    temp_slices.truncate(pos);
                }
                // Merge slices and store them to the output.
                let mut i: usize = 0;
                while i < temp_slices.len() {
                    // Find a range of temp_slices with the same region_id.
                    let mut j = i;
                    let mut merged = false;
                    let mut expolygons = std::mem::take(&mut temp_slices[i].expolygons);
                    j += 1;
                    while j < temp_slices.len() && temp_slices[i].region_id == temp_slices[j].region_id {
                        if !temp_slices[j].expolygons.is_empty() {
                            let expolygons2 = std::mem::take(&mut temp_slices[j].expolygons);
                            if expolygons.is_empty() {
                                expolygons = expolygons2;
                            } else {
                                expolygons.extend(expolygons2);
                                merged = true;
                            }
                        }
                        j += 1;
                    }
                    // Don't unite the regions if ! clip_multipart_objects. In that case it is user's responsibility
                    // to handle region overlaps. Indeed, one may intentionally let the regions overlap to produce crossing perimeters
                    // for example.
                    if merged && clip_multipart_objects_ {
                        expolygons = closing_ex(&expolygons, EPSILON / SCALING_FACTOR);
                    }
                    slices_by_region[temp_slices[i].region_id as usize][z_idx] = expolygons;
                    i = j;
                }
                throw_on_cancel_callback()?;
            }
        }
    }

    Ok(slices_by_region)
}

/// PrintObjectSlice.cpp:644-862 `bool groupingVolumes(std::vector<VolumeSlices> objSliceByVolume,
/// std::vector<groupedVolumeSlices>& groups, double resolution, int firstLayerReplacedBy)`.
///
/// BBS: grouping the volumes of an object according to their connection relationship.
/// Connectivity is evaluated across ALL layers (two volumes are connected when their grown
/// slices overlap on any shared layer, with a +/-1 layer tolerance), which restores the
/// original full-layer semantics after the first-layer-only optimization. A spatial hash grid
/// built on the full (all-layer) bounding boxes keeps the broad phase near-linear so thousands
/// of islands stay fast; the exact per-layer overlap test only runs for spatially near pairs.
/// The grouped slices that feed brim generation are still taken from the first printed layer.
pub fn groupingVolumes(objSliceByVolume: Vec<VolumeSlices>, groups: &mut Vec<groupedVolumeSlices>, resolution: f64, firstLayerReplacedBy: i32) -> bool {
    let n = objSliceByVolume.len() as i32;
    groups.clear();
    if n == 0 {
        return true;
    }
    let n = n as usize;

    let offset_value: f64 = 0.05 / SCALING_FACTOR;
    let layer_idx = firstLayerReplacedBy.min(0.max(objSliceByVolume[0].slices.len() as i32 - 1));

    // Grow every layer of every volume (Douglas-Peucker + 0.05mm offset) and record the full XY
    // bounding box plus the occupied layer range. Growing all layers is what preserves cross-layer
    // connectivity; historically the bottleneck was the O(n^2) pairwise test below, not this step.
    let mut grown: Vec<Vec<ExPolygons>> = vec![Vec::new(); n];
    let mut lbbox: Vec<Vec<BoundingBox>> = vec![Vec::new(); n];
    let mut bboxes: Vec<BoundingBox> = vec![BoundingBox::new(); n];
    let mut layer_lo: Vec<i32> = vec![0; n];
    let mut layer_hi: Vec<i32> = vec![-1; n];
    let mut alive: Vec<bool> = vec![false; n];
    for i in 0..n {
        let src = &objSliceByVolume[i].slices;
        let nl = src.len() as i32;
        grown[i].resize(nl as usize, ExPolygons::new());
        lbbox[i].resize(nl as usize, BoundingBox::new());
        let mut bb = BoundingBox::new();
        let mut bb_valid = false;
        let (mut lo, mut hi) = (nl, -1i32);
        for k in 0..nl {
            let ku = k as usize;
            if src[ku].is_empty() {
                continue;
            }
            let mut simplified = src[ku].clone();
            if resolution > 0. {
                for poly_ex in simplified.iter_mut() {
                    expolygon_douglas_peucker(poly_ex, resolution);
                }
            }
            grown[i][ku] = offset_ex(&simplified, offset_value);
            if grown[i][ku].is_empty() {
                continue;
            }
            let lbb = get_extents(&grown[i][ku]);
            lbbox[i][ku] = lbb;
            if !bb_valid {
                bb = lbb;
                bb_valid = true;
            } else {
                bb.merge(&lbb);
            }
            if k < lo {
                lo = k;
            }
            if k > hi {
                hi = k;
            }
        }
        if bb_valid {
            bboxes[i] = bb;
            layer_lo[i] = lo;
            layer_hi[i] = hi;
            alive[i] = true;
        }
    }

    let mut parent: Vec<usize> = (0..n).collect();
    fn find_root(parent: &mut [usize], mut x: usize) -> usize {
        while parent[x] != x {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        x
    }
    fn unite(parent: &mut [usize], a: usize, b: usize) {
        let a = find_root(parent, a);
        let b = find_root(parent, b);
        if a != b {
            parent[b] = a;
        }
    }

    // Narrow phase: exact all-layer overlap test with +/-1 layer tolerance (the original per-layer
    // overlap semantics). Restricted to the first volume's occupied layer range.
    let volumes_connected = |i: usize, j: usize| -> bool {
        let a = &grown[i];
        let b = &grown[j];
        let ba = &lbbox[i];
        let bb = &lbbox[j];
        let nlb = b.len() as i32;
        if a.len() as i32 != nlb {
            return false;
        }
        let lo = 0.max(layer_lo[i]);
        let hi = layer_hi[i].min(nlb - 1);
        let mut k = lo;
        while k <= hi {
            let ku = k as usize;
            if a[ku].is_empty() {
                k += 1;
                continue;
            }
            let bak = &ba[ku];
            // Per-layer bbox pre-check avoids the expensive Clipper overlaps() for the common case
            // where two volumes share an XY column but not this particular layer.
            if !b[ku].is_empty() && bbox_overlap(bak, &bb[ku]) && overlaps(&a[ku], &b[ku]) {
                return true;
            }
            if k + 1 < nlb && !b[ku + 1].is_empty() && bbox_overlap(bak, &bb[ku + 1]) && overlaps(&a[ku], &b[ku + 1]) {
                return true;
            }
            if k - 1 >= 0 && !b[ku - 1].is_empty() && bbox_overlap(bak, &bb[ku - 1]) && overlaps(&a[ku], &b[ku - 1]) {
                return true;
            }
            k += 1;
        }
        false
    };

    // Broad phase: spatial hash on full (all-layer) bboxes so we only narrow-test near pairs.
    let cell_size: i64 = 1i64.max((5. / SCALING_FACTOR) as i64);
    // std::unordered_map<uint64_t, std::vector<int>> grid: the candidates it yields are sorted below,
    // the order of its iteration does not show.
    let mut grid: HashMap<u64, Vec<usize>> = HashMap::with_capacity(n * 2);
    let cell_key = |cx: i32, cy: i32| -> u64 { ((cx as u32 as u64) << 32) | (cy as u32 as u64) };
    for i in 0..n {
        if !alive[i] {
            continue;
        }
        let bb = &bboxes[i];
        let x0 = (bb.min.x / cell_size) as i32;
        let y0 = (bb.min.y / cell_size) as i32;
        let x1 = (bb.max.x / cell_size) as i32;
        let y1 = (bb.max.y / cell_size) as i32;
        for cx in x0..=x1 {
            for cy in y0..=y1 {
                grid.entry(cell_key(cx, cy)).or_default().push(i);
            }
        }
    }

    let pair_key = |a: usize, b: usize| -> u64 {
        let (a, b) = if a > b { (b, a) } else { (a, b) };
        ((a as u32 as u64) << 32) | (b as u32 as u64)
    };

    // Collect the unique near candidate pairs (cheap bbox / layer-range filters only). The expensive
    // per-layer overlap test is deferred so it can run in parallel below.
    let mut candidates: Vec<(usize, usize)> = Vec::new();
    {
        let mut seen: HashSet<u64> = HashSet::with_capacity(n * 4);
        for ids in grid.values() {
            for a in 0..ids.len() {
                for b in a + 1..ids.len() {
                    let i = ids[a];
                    let j = ids[b];
                    if !bbox_overlap(&bboxes[i], &bboxes[j]) {
                        continue;
                    }
                    // Layer ranges must be within +/-1 to have any chance of touching.
                    if layer_hi[i] < layer_lo[j] - 1 || layer_hi[j] < layer_lo[i] - 1 {
                        continue;
                    }
                    if seen.insert(pair_key(i, j)) {
                        candidates.push((i, j));
                    }
                }
            }
        }
    }

    // Narrow phase: the connectivity test is read-only and independent per pair, so evaluate all
    // candidates in parallel. Candidates are sorted for deterministic union order (grid iteration
    // order is not stable); connectivity itself is order independent.
    for p in candidates.iter_mut() {
        if p.0 > p.1 {
            std::mem::swap(&mut p.0, &mut p.1);
        }
    }
    candidates.sort();
    let mut connected: Vec<bool> = vec![false; candidates.len()];
    for c in 0..candidates.len() {
        connected[c] = volumes_connected(candidates[c].0, candidates[c].1);
    }
    for c in 0..candidates.len() {
        if connected[c] {
            unite(&mut parent, candidates[c].0, candidates[c].1);
        }
    }

    // Assign every geometry-bearing volume to its group (slices are collected below only when needed).
    // groupId is the minimum volume index in the group for stable slice-cache JSON output.
    let mut root_to_group: HashMap<usize, usize> = HashMap::new();
    let mut group_min: Vec<usize> = Vec::new();
    groups.reserve(n);
    for i in 0..n {
        if !alive[i] {
            continue;
        }
        let root = find_root(&mut parent, i);
        let gidx;
        match root_to_group.get(&root) {
            None => {
                gidx = groups.len();
                root_to_group.insert(root, gidx);
                let mut gvs = groupedVolumeSlices::default();
                gvs.groupId = i as i32;
                groups.push(gvs);
                group_min.push(i);
            }
            Some(&g) => {
                gidx = g;
                if i < group_min[gidx] {
                    group_min[gidx] = i;
                    groups[gidx].groupId = i as i32;
                }
            }
        }
        groups[gidx].volume_ids.push(objSliceByVolume[i].volume_id);
    }

    // Collect the grown first-layer slices, union them and shrink back to the real outline. Using
    // grown[.] here (then shrinking by offsetValue) reproduces the original morphological closing
    // between volumes.
    for i in 0..n {
        if !alive[i] {
            continue;
        }
        let gidx = root_to_group[&find_root(&mut parent, i)];
        if layer_idx < grown[i].len() as i32 {
            groups[gidx].slices.extend(grown[i][layer_idx as usize].iter().cloned());
        }
    }
    for gi in 0..groups.len() {
        groups[gi].slices = offset_ex(&union_ex_expolygons(&groups[gi].slices), -offset_value);
        for poly_ex in groups[gi].slices.iter_mut() {
            expolygon_douglas_peucker(poly_ex, resolution);
        }
    }
    true
}

/// PrintObjectSlice.cpp:864-878 `std::vector<VolumeSlices> findPartVolumes(const std::vector<VolumeSlices>&
/// objSliceByVolume, ModelVolumePtrs model_volumes)`.
///
/// BBS: filter the members of "objSliceByVolume" such that only "model_part" are included
pub fn findPartVolumes(objSliceByVolume: &[VolumeSlices], model_volumes: &[ModelVolume]) -> Vec<VolumeSlices> {
    let mut part_ids: HashSet<usize> = HashSet::with_capacity(model_volumes.len());
    for mv in model_volumes {
        if mv.is_model_part() {
            part_ids.insert(mv.id().id);
        }
    }
    let mut out_put: Vec<VolumeSlices> = Vec::with_capacity(objSliceByVolume.len());
    for vs in objSliceByVolume {
        if part_ids.contains(&vs.volume_id.id) {
            out_put.push(vs.clone());
        }
    }
    out_put
}

/// PrintObjectSlice.cpp:880-897 `void applyNegtiveVolumes(ModelVolumePtrs model_volumes, const
/// std::vector<VolumeSlices>& objSliceByVolume, std::vector<groupedVolumeSlices>& groups, double resolution)`
pub fn applyNegtiveVolumes(model_volumes: &[ModelVolume], objSliceByVolume: &[VolumeSlices], groups: &mut [groupedVolumeSlices], resolution: f64) {
    let mut neg_total = ExPolygons::new();
    for vs in objSliceByVolume {
        for mv in model_volumes {
            if vs.volume_id == mv.id() && mv.is_negative_volume() {
                if !vs.slices.is_empty() {
                    neg_total.extend(vs.slices[0].iter().cloned());
                }
            }
        }
    }

    for g in groups.iter_mut() {
        g.slices = diff_ex(&g.slices, &neg_total);
        for poly_ex in g.slices.iter_mut() {
            expolygon_douglas_peucker(poly_ex, resolution);
        }
    }
}

/// PrintObjectSlice.cpp:899-943 `void reGroupingLayerPolygons(std::vector<groupedVolumeSlices>& gvss, ExPolygons
/// &eps, double resolution)`
pub fn reGroupingLayerPolygons(gvss: &mut [groupedVolumeSlices], eps: &ExPolygons, resolution: f64) {
    let mut eps_index: Vec<i32> = vec![-1; eps.len()];

    let mut gvssc: Vec<groupedVolumeSlices> = gvss.to_vec();
    let mut epsc: ExPolygons = eps.clone();

    for poly_ex in epsc.iter_mut() {
        expolygon_douglas_peucker(poly_ex, resolution);
    }

    for g in gvssc.iter_mut() {
        for poly_ex in g.slices.iter_mut() {
            expolygon_douglas_peucker(poly_ex, resolution);
        }
    }

    for ie in 0..epsc.len() {
        if epsc[ie].area() <= 0. {
            continue;
        }

        let mut min_area = epsc[ie].area();
        for iv in 0..gvssc.len() {
            // diff_ex(const ExPolygon &subject, const ExPolygons &clip)
            let cliped_ex_polys = diff_ex(std::slice::from_ref(&epsc[ie]), &gvssc[iv].slices);
            let mut area: f64 = 0.;
            for ce in &cliped_ex_polys {
                area += ce.area();
            }
            if area < min_area {
                min_area = area;
                eps_index[ie] = iv as i32;
            }
        }
    }

    for g in gvss.iter_mut() {
        g.slices.clear();
    }

    for ie in 0..eps.len() {
        if eps_index[ie] >= 0 {
            gvss[eps_index[ie] as usize].slices.push(eps[ie].clone());
        }
    }
}

/// PrintObjectSlice.cpp:945-1065 `std::string fix_slicing_errors(PrintObject* object, LayerPtrs &layers, const
/// std::function<void()> &throw_if_canceled, int &firstLayerReplacedBy)`. `print` is `object->print()`.
pub fn fix_slicing_errors(
    print: &Print,
    object: &PrintObject,
    layers: &mut Vec<Layer>,
    throw_if_canceled: ThrowOnCancel,
    firstLayerReplacedBy: &mut i32,
) -> Result<String, PrintObjectSliceError> {
    let mut error_msg = String::new(); //BBS

    if layers.is_empty() {
        return Ok(error_msg);
    }

    // Collect layers with slicing errors.
    // These layers will be fixed in parallel.
    let mut buggy_layers: Vec<usize> = Vec::with_capacity(layers.len());
    // BBS: get largest external perimenter width of all layers
    let get_ext_peri_width = |layer: &Layer| -> Result<i64, PrintObjectSliceError> {
        if layer.regions().is_empty() {
            Ok(0)
        } else {
            Ok(layer_region_flow(print, object, layer, 0, FlowRole::ExternalPerimeter)?.scaled_width())
        }
    };
    // std::max_element: the first of the largest.
    let mut it = 0usize;
    let mut it_width = get_ext_peri_width(&layers[0])?;
    for i in 1..layers.len() {
        let w = get_ext_peri_width(&layers[i])?;
        if it_width < w {
            it = i;
            it_width = w;
        }
    }
    let _ = it;
    let thresh: i64 = (it_width as f64 * 0.5) as i64; // half of external perimeter width  // 0.5 * scale_(this->config().line_width);
    for idx_layer in 0..layers.len() {
        // BBS: detect empty layers (layers with very small regions) and mark them as problematic, then these layers will copy the nearest good layer
        let layer = &mut layers[idx_layer];
        let mut lslices = ExPolygons::new();
        for region_id in 0..layer.regions().len() {
            let layerm = &layer.regions()[region_id];
            for surface in &layerm.slices.surfaces {
                let expoly = offset_ex_expolygon(&surface.expolygon, -(thresh as f64));
                lslices.splice(0..0, expoly);
            }
        }
        if lslices.is_empty() {
            layer.slicing_errors = true;
        }

        if layers[idx_layer].slicing_errors {
            buggy_layers.push(idx_layer);
        } else {
            break; // only detect empty layers near bed
        }
    }

    let mut is_replaced = false;
    for buggy_layer_idx in 0..buggy_layers.len() {
        throw_if_canceled()?;
        let idx_layer = buggy_layers[buggy_layer_idx];
        // BBS: only replace empty layers lower than 1mm
        let thresh_empty_layer_height: f64 = 1.;
        if layers[idx_layer].print_z >= thresh_empty_layer_height {
            continue;
        }
        debug_assert!(layers[idx_layer].slicing_errors);
        // Try to repair the layer surfaces by merging all contours and all holes from neighbor layers.
        for region_id in 0..layers[idx_layer].region_count() {
            // Find the first valid layer below / above the current layer.
            let mut upper_surfaces: Option<usize> = None;
            let mut lower_surfaces: Option<usize> = None;
            //BBS: only repair empty layers lowers than 1mm
            for j in idx_layer + 1..layers.len() {
                if !layers[j].slicing_errors {
                    upper_surfaces = Some(j);
                    break;
                }
                if layers[j].print_z >= thresh_empty_layer_height {
                    break;
                }
            }
            let mut j = idx_layer as i32 - 1;
            while j >= 0 {
                let ju = j as usize;
                if layers[ju].print_z >= thresh_empty_layer_height {
                    j -= 1;
                    continue;
                }
                if !layers[ju].slicing_errors {
                    lower_surfaces = Some(ju);
                    break;
                }
                j -= 1;
            }
            // Collect outer contours and holes from the valid layers above & below.
            let mut expolys = ExPolygons::new();
            if let Some(j) = upper_surfaces {
                for surface in &layers[j].regions()[region_id].slices.surfaces {
                    expolys.push(surface.expolygon.clone());
                }
            }
            if let Some(j) = lower_surfaces {
                for surface in &layers[j].regions()[region_id].slices.surfaces {
                    expolys.push(surface.expolygon.clone());
                }
            }
            if !expolys.is_empty() {
                //BBS
                is_replaced = true;
                layers[idx_layer].regions_mut()[region_id].slices.set(&union_ex_expolygons(&expolys), SurfaceType::Internal);
            }
        }
        // Update layer slices after repairing the single regions.
        layers[idx_layer].make_slices();
    }
    throw_if_canceled()?;

    if is_replaced {
        error_msg = "Empty layers around bottom are replaced by nearest normal layers.".to_string();
    }

    // remove empty layers from bottom
    while !layers.is_empty() && (layers[0].lslices.is_empty() || layers[0].empty()) {
        layers.remove(0);
        // layers.front()->lower_layer = nullptr: the links are indices, which move down with the layers.
        for layer in layers.iter_mut() {
            layer.lower_layer_id = layer.lower_layer_id.and_then(|id| id.checked_sub(1));
            layer.upper_layer_id = layer.upper_layer_id.and_then(|id| id.checked_sub(1));
        }
        for i in 0..layers.len() {
            let id = layers[i].id();
            layers[i].set_id(id.wrapping_sub(1));
        }
    }

    //BBS
    if error_msg.is_empty() && !buggy_layers.is_empty() {
        error_msg = "The model has too many empty layers.".to_string();
    }

    // BBS: first layer slices are sorted by volume group, if the first layer is empty and replaced by the 2nd layer
    // the later will be stored in "object->firstLayerObjGroupsMod()"
    if !buggy_layers.is_empty() && buggy_layers[0] == 0 && layers.len() > 1 {
        *firstLayerReplacedBy = 1;
    }

    Ok(error_msg)
}

/// PrintObjectSlice.cpp:1067-1076 `void groupingVolumesForBrim(PrintObject* object, LayerPtrs& layers, int
/// firstLayerReplacedBy)`. `layers` are `object.m_layers`.
pub fn groupingVolumesForBrim(print: &Print, object: &mut PrintObject, firstLayerReplacedBy: i32) {
    // scaled<double>(object->print()->config().resolution.value)
    let scaled_resolution = cfg_float(print.config(), "resolution") / SCALING_FACTOR;
    let model_volumes = &object.model_object(print).volumes;
    let parts_obj_slice_by_volume = findPartVolumes(&object.firstLayerObjSliceByVolume, model_volumes);
    let mut groups = std::mem::take(&mut object.firstLayerObjSliceByGroups);
    groupingVolumes(parts_obj_slice_by_volume, &mut groups, scaled_resolution, firstLayerReplacedBy);
    applyNegtiveVolumes(model_volumes, &object.firstLayerObjSliceByVolume, &mut groups, scaled_resolution);

    // BBS: the actual first layer slices stored in layers are re-sorted by volume group and will be used to generate brim
    reGroupingLayerPolygons(&mut groups, &object.m_layers[0].lslices, scaled_resolution);
    object.firstLayerObjSliceByGroups = groups;
}

/// PrintObjectSlice.cpp:1851-1977 `template<typename ThrowOnCancel> static inline void
/// apply_mm_segmentation(PrintObject &print_object, ThrowOnCancel throw_on_cancel)`. The segmentation by
/// painting comes from `cores`.
pub fn apply_mm_segmentation(
    print: &Print,
    print_object: &mut PrintObject,
    cores: &mut dyn SliceVolumesCores,
    throw_on_cancel: ThrowOnCancel,
) -> Result<(), CanceledException> {
    // Returns MMU segmentation based on painting in MMU segmentation gizmo
    let mut segmentation: Vec<Vec<ExPolygons>> = cores.multi_material_segmentation_by_painting(print, print_object);
    debug_assert!(segmentation.len() == print_object.layer_count());
    normalize_mm_segmentation_to_layer_bands(print, print_object, &mut segmentation, throw_on_cancel)?;
    if segmentation.is_empty() {
        return Ok(());
    }
    let shared = print_object.m_shared_regions.clone().expect("the shared regions of the object");
    let shared = shared.borrow();
    let layer_ranges = &shared.layer_ranges;
    // tbb::parallel_for over the layers, one range here.
    let z = print_object.m_layers[0].slice_z;
    let mut it_layer_range = layer_range_first(layer_ranges, z);
    // BBS
    let num_extruders = cfg_floats(print.config(), "filament_diameter").len();
    #[derive(Clone)]
    struct ByExtruder {
        expolygons: ExPolygons,
        bbox: BoundingBox,
    }
    impl Default for ByExtruder {
        fn default() -> Self {
            ByExtruder { expolygons: ExPolygons::new(), bbox: BoundingBox::new() }
        }
    }
    #[derive(Default, Clone)]
    struct ByRegion {
        expolygons: ExPolygons,
        needs_merge: bool,
    }
    for layer_id in 0..segmentation.len() {
        throw_on_cancel()?;
        let slice_z = print_object.m_layers[layer_id].slice_z;
        it_layer_range = layer_range_next(layer_ranges, it_layer_range, slice_z);
        let layer_range = &layer_ranges[it_layer_range];
        let layer = &mut print_object.m_layers[layer_id];
        // Gather per extruder expolygons.
        let mut by_extruder: Vec<ByExtruder> = vec![ByExtruder::default(); num_extruders];
        let mut by_region: Vec<ByRegion> = vec![ByRegion::default(); layer.region_count()];
        let mut layer_split = false;
        for extruder_id in 0..num_extruders {
            let region = &mut by_extruder[extruder_id];
            region.expolygons.extend(std::mem::take(&mut segmentation[layer_id][extruder_id]));
            if !region.expolygons.is_empty() {
                region.bbox = get_extents(&region.expolygons);
                layer_split = true;
            }
        }
        if !layer_split {
            continue;
        }
        // Split LayerRegions by by_extruder regions.
        // layer_range.painted_regions are sorted by extruder ID and parent PrintObject region ID.
        let mut it_painted_region: usize = 0;
        let painted_parent_region_id = |it: usize| -> i32 { volume_region_id(layer_range, layer_range.painted_regions[it].parent as usize) };
        for region_id in 0..layer.region_count() as i32 {
            if layer.regions()[region_id as usize].slices.surfaces.is_empty() {
                continue;
            }
            let layerm_region = shared.all_regions[layer.regions()[region_id as usize].region_id()].clone();
            let bbox = get_extents_surfaces(&layer.regions()[region_id as usize].slices.surfaces);
            debug_assert!(it_painted_region < layer_range.painted_regions.len());
            // Find the first it_painted_region which overrides this region.
            while painted_parent_region_id(it_painted_region) < region_id {
                it_painted_region += 1;
                debug_assert!(it_painted_region != layer_range.painted_regions.len());
            }
            // 1-based extruder ID
            let self_trimmed = false;
            let mut self_extruder_id: i32 = -1;
            for extruder_id in 1..=by_extruder.len() as i32 {
                let segmented = &by_extruder[extruder_id as usize - 1];
                if segmented.bbox.is_defined() && bbox_overlap(&bbox, &segmented.bbox) {
                    // Find the target region.
                    while (layer_range.painted_regions[it_painted_region].extruder_id as i32) < extruder_id {
                        it_painted_region += 1;
                        debug_assert!(it_painted_region != layer_range.painted_regions.len());
                    }
                    let painted = &layer_range.painted_regions[it_painted_region];
                    let painted_region = painted.region.as_ref().expect("the region of a painted region");
                    //FIXME Don't trim by self, it is not reliable.
                    if std::rc::Rc::ptr_eq(&layerm_region, painted_region) {
                        self_extruder_id = extruder_id;
                        continue;
                    }
                    // Steal from this region.
                    let target_region_id = painted_region.borrow().print_object_region_id();
                    let stolen = intersection_ex_surfaces(&layer.regions()[region_id as usize].slices.surfaces, &segmented.expolygons);
                    if !stolen.is_empty() {
                        let dst = &mut by_region[target_region_id as usize];
                        if dst.expolygons.is_empty() {
                            dst.expolygons = stolen;
                        } else {
                            dst.expolygons.extend(stolen);
                            dst.needs_merge = true;
                        }
                    }
                }
            }
            if !self_trimmed {
                // Trim slices of this LayerRegion with all the MMU regions.
                let mut mine: Polygons = crate::surface::to_polygons(&layer.regions()[region_id as usize].slices.surfaces);
                layer.regions_mut()[region_id as usize].slices.surfaces.clear();
                for (idx, segmented) in by_extruder.iter().enumerate() {
                    if idx as i32 + 1 != self_extruder_id && segmented.bbox.is_defined() && bbox_overlap(&bbox, &segmented.bbox) {
                        mine = diff_polygons_expolygons(&mine, &segmented.expolygons);
                        if mine.is_empty() {
                            break;
                        }
                    }
                }
                // Filter out unprintable polygons produced by subtraction multi-material painted regions from layerm.region().
                // ExPolygon returned from multi-material segmentation does not precisely match ExPolygons in layerm.region()
                // (because of preprocessing of the input regions in multi-material segmentation). Therefore, subtraction from
                // layerm.region() could produce a huge number of small unprintable regions for the model's base extruder.
                // This could, on some models, produce bulges with the model's base color (#7109).
                if !mine.is_empty() {
                    mine = opening(&union_ex(&mine), 5. * EPSILON / SCALING_FACTOR, 5. * EPSILON / SCALING_FACTOR);
                }
                if !mine.is_empty() {
                    let dst = &mut by_region[layerm_region.borrow().print_object_region_id() as usize];
                    if dst.expolygons.is_empty() {
                        dst.expolygons = union_ex(&mine);
                    } else {
                        dst.expolygons.extend(union_ex(&mine));
                        dst.needs_merge = true;
                    }
                }
            }
        }
        // Re-create Surfaces of LayerRegions.
        for region_id in 0..layer.region_count() {
            let src = &mut by_region[region_id];
            if src.needs_merge {
                // Multiple regions were merged into one.
                src.expolygons = closing_ex(&src.expolygons, 10. * EPSILON / SCALING_FACTOR);
            }
            layer.regions_mut()[region_id].slices.set(&std::mem::take(&mut src.expolygons), SurfaceType::Internal);
        }
    }
    Ok(())
}

/// PrintObjectSlice.cpp:1979-2084 `template<typename ThrowOnCancel> void apply_fuzzy_skin_segmentation(PrintObject
/// &print_object, ThrowOnCancel throw_on_cancel)`. The segmentation by painting comes from `cores`.
pub fn apply_fuzzy_skin_segmentation(
    print: &Print,
    print_object: &mut PrintObject,
    cores: &mut dyn SliceVolumesCores,
    throw_on_cancel: ThrowOnCancel,
) -> Result<(), CanceledException> {
    // Returns fuzzy skin segmentation based on painting in the fuzzy skin painting gizmo.
    let segmentation: Vec<Vec<ExPolygons>> = cores.fuzzy_skin_segmentation_by_painting(print, print_object);
    debug_assert!(segmentation.len() == print_object.layer_count());
    if segmentation.is_empty() {
        return Ok(());
    }

    #[derive(Default, Clone)]
    struct ByRegion {
        expolygons: ExPolygons,
        needs_merge: bool,
    }

    let shared = print_object.m_shared_regions.clone().expect("the shared regions of the object");
    let shared = shared.borrow();
    let layer_ranges = &shared.layer_ranges;
    // tbb::parallel_for over the layers, one range here.
    let mut it_layer_range = layer_range_first(layer_ranges, print_object.m_layers[0].slice_z);

    for layer_idx in 0..segmentation.len() {
        throw_on_cancel()?;

        let layer = &mut print_object.m_layers[layer_idx];
        it_layer_range = layer_range_next(layer_ranges, it_layer_range, layer.slice_z);
        let layer_range = &layer_ranges[it_layer_range];

        //assert(segmentation[layer_idx].size() == 1);
        let fuzzy_skin_segmentation = &segmentation[layer_idx][1];
        let fuzzy_skin_segmentation_bbox = get_extents(fuzzy_skin_segmentation);
        if fuzzy_skin_segmentation.is_empty() {
            continue;
        }

        // Split LayerRegions by painted fuzzy skin regions.
        // layer_range.fuzzy_skin_painted_regions are sorted by parent PrintObject region ID.
        let mut by_region: Vec<ByRegion> = vec![ByRegion::default(); layer.region_count()];
        let mut it_fuzzy_skin_region_begin: usize = 0;
        for parent_layer_region_idx in 0..layer.region_count() {
            if it_fuzzy_skin_region_begin == layer_range.fuzzy_skin_painted_regions.len() {
                continue;
            }

            let parent_layer_region = &layer.regions()[parent_layer_region_idx];
            let parent_print_region = shared.all_regions[parent_layer_region.region_id()].clone();
            let parent_print_region_id = parent_print_region.borrow().print_object_region_id();
            debug_assert!(parent_print_region_id == parent_layer_region_idx as i32);
            if parent_layer_region.slices.surfaces.is_empty() {
                continue;
            }

            // Find the first FuzzySkinPaintedRegion, which overrides the parent PrintRegion.
            let it_fuzzy_skin_region = (it_fuzzy_skin_region_begin..layer_range.fuzzy_skin_painted_regions.len())
                .find(|&i| layer_range.fuzzy_skin_painted_regions[i].parent_print_object_region_id(layer_range) == parent_print_region_id);

            let Some(it_fuzzy_skin_region) = it_fuzzy_skin_region else {
                continue; // This LayerRegion isn't overrides by any FuzzySkinPaintedRegion.
            };

            // Update the beginning FuzzySkinPaintedRegion iterator for the next iteration.
            it_fuzzy_skin_region_begin = it_fuzzy_skin_region + 1;

            let fuzzy_region = layer_range.fuzzy_skin_painted_regions[it_fuzzy_skin_region].region.as_ref().expect("the region of a fuzzy skin painted region");
            let parent_layer_region_bbox = get_extents_surfaces(&parent_layer_region.slices.surfaces);
            let mut layer_region_remaining_polygons: Polygons = crate::surface::to_polygons(&parent_layer_region.slices.surfaces);
            // Don't trim by self, it is not reliable.
            if bbox_overlap(&parent_layer_region_bbox, &fuzzy_skin_segmentation_bbox) && !std::rc::Rc::ptr_eq(fuzzy_region, &parent_print_region) {
                // Steal from this region.
                let target_region_id = fuzzy_region.borrow().print_object_region_id();
                let stolen = intersection_ex_surfaces(&parent_layer_region.slices.surfaces, fuzzy_skin_segmentation);
                if !stolen.is_empty() {
                    let dst = &mut by_region[target_region_id as usize];
                    if dst.expolygons.is_empty() {
                        dst.expolygons = stolen;
                    } else {
                        dst.expolygons.extend(stolen);
                        dst.needs_merge = true;
                    }
                }

                // Trim slices of this LayerRegion by the fuzzy skin region.
                layer_region_remaining_polygons = diff_polygons_expolygons(&layer_region_remaining_polygons, fuzzy_skin_segmentation);

                // Filter out unprintable polygons. Detailed explanation is inside apply_mm_segmentation.
                if !layer_region_remaining_polygons.is_empty() {
                    layer_region_remaining_polygons =
                        opening(&union_ex(&layer_region_remaining_polygons), 5. * EPSILON / SCALING_FACTOR, 5. * EPSILON / SCALING_FACTOR);
                }
            }

            if !layer_region_remaining_polygons.is_empty() {
                let dst = &mut by_region[parent_print_region_id as usize];
                if dst.expolygons.is_empty() {
                    dst.expolygons = union_ex(&layer_region_remaining_polygons);
                } else {
                    dst.expolygons.extend(union_ex(&layer_region_remaining_polygons));
                    dst.needs_merge = true;
                }
            }
        }

        // Re-create Surfaces of LayerRegions.
        for region_id in 0..layer.region_count() {
            let src = &mut by_region[region_id];
            if src.needs_merge {
                // Multiple regions were merged into one.
                src.expolygons = closing_ex(&src.expolygons, 10. * EPSILON / SCALING_FACTOR);
            }

            layer.regions_mut()[region_id].slices.set(&std::mem::take(&mut src.expolygons), SurfaceType::Internal);
        }
    }
    Ok(())
}

/// PrintObjectSlice.cpp:2086-2098 `static bool should_skip_filament_shrink(const PrintObject &object)`: MMU
/// cannot apply filament shrinkage compensation.
pub fn should_skip_filament_shrink(print: &Print, object: &PrintObject) -> bool {
    if cfg_floats(print.config(), "filament_diameter").len() <= 1 || !object.is_mm_painted(print) {
        return false;
    }

    let filament_shrink = cfg_floats(print.config(), "filament_shrink");
    let object_filaments = object.object_extruders(print);
    object_filaments.iter().any(|&extruder| {
        (extruder as usize) < filament_shrink.len() && filament_shrink[extruder as usize] != 0. && filament_shrink[extruder as usize] != 100.
    })
}

impl PrintObject {
    /// PrintObjectSlice.cpp:1078-1157 `void PrintObject::slice()`. `self` is an object of `print`, taken out
    /// of `Print::m_objects` while the step runs.
    ///
    /// Called by make_perimeters()
    /// 1) Decides Z positions of the layers,
    /// 2) Initializes layers and their regions
    /// 3) Slices the object meshes
    /// 4) Slices the modifier meshes and reclassifies the slices of the object meshes by the slices of the modifier meshes
    /// 5) Applies size compensation (offsets the slices in XY plane)
    /// 6) Replaces bad slices by the slices reconstructed from the upper/lower layer
    /// Resulting expolygons of layer regions are marked as Internal.
    pub fn slice(&mut self, print: &Print, cores: &mut dyn SliceVolumesCores) -> Result<(), PrintObjectSliceError> {
        let throw_if_canceled = || print.base.base.throw_if_canceled();
        if !self.m_state.set_started(PrintObjectStep::posSlice, print.base.base.state_mutex(), throw_if_canceled)? {
            return Ok(());
        }

        //BBS: add flag to reload scene for shell rendering
        print.base.base.set_status(5, "Slicing mesh", SlicingStatus::RELOAD_SCENE, -1);
        let mut layer_height_profile: Vec<f64> = Vec::new();
        let mut nozzle_range_reset = false;
        PrintObject::update_layer_height_profile(self.model_object(print), &self.m_slicing_params, &mut layer_height_profile, &mut nozzle_range_reset);
        if nozzle_range_reset {
            active_step_add_warning(
                print,
                self,
                WarningLevel::NonCritical,
                "The variable layer height profile has been reset because some layer heights exceed the allowed range of the current nozzle.",
            );
        }
        throw_if_canceled()?;
        self.m_typed_slices = false;
        // this->clear_layers();
        self.m_layers.clear();
        self.m_layers = new_layers(
            self,
            &crate::slicing::generate_object_layers(&self.m_slicing_params, &layer_height_profile, cfg_bool(&self.m_config, "precise_z_height")),
        );
        self.slice_volumes(print, cores)?;
        throw_if_canceled()?;
        let mut first_layer_replaced_by: i32 = 0;

        // Fix the model.
        //FIXME is this the right place to do? It is done repeateadly at the UI and now here at the backend.
        let mut layers = std::mem::take(&mut self.m_layers);
        let warning = fix_slicing_errors(print, self, &mut layers, &throw_if_canceled, &mut first_layer_replaced_by);
        self.m_layers = layers;
        let _warning = warning?;
        throw_if_canceled()?;
        //BBS: send warning message to slicing callback
        // This warning is inaccurate, because the empty layers may have been replaced, or the model has supports.

        // BBS: the actual first layer slices stored in layers are re-sorted by volume group and will be used to generate brim
        print.base.base.set_status(12, "Slicing mesh", SlicingStatus::DEFAULT, -1);
        // groupingVolumesForBrim reads layers.front(): without layers the C++ dereferences the end of an
        // empty vector before it reaches the "No layers were detected" error below.
        if !self.m_layers.is_empty() {
            groupingVolumesForBrim(print, self, first_layer_replaced_by);
        }
        print.base.base.set_status(14, "Slicing mesh", SlicingStatus::DEFAULT, -1);

        // Update bounding boxes, back up raw slices of complex models.
        for layer_idx in 0..self.m_layers.len() {
            throw_if_canceled()?;
            let layer = &mut self.m_layers[layer_idx];
            layer.lslices_bboxes.clear();
            layer.lslices_bboxes.reserve(layer.lslices.len());
            for expoly in &layer.lslices {
                layer.lslices_bboxes.push(get_extents_expolygon(expoly));
            }
            layer.backup_untyped_slices();
        }

        if self.m_layers.is_empty() {
            return Err(PrintObjectSliceError::SlicingError(
                "No layers were detected. You might want to repair your STL file(s) or check their size or thickness and retry.\n".to_string(),
            ));
        }

        // BBS
        self.m_state.set_done(PrintObjectStep::posSlice, print.base.base.state_mutex(), throw_if_canceled)?;
        Ok(())
    }

    /// PrintObjectSlice.cpp:2100-2413 `void PrintObject::slice_volumes(long long *region_split_ms_out, long long
    /// *mm_segment_ms_out)` (the two timers have no counterpart).
    ///
    /// 1) Decides Z positions of the layers,
    /// 2) Initializes layers and their regions
    /// 3) Slices the object meshes
    /// 4) Slices the modifier meshes and reclassifies the slices of the object meshes by the slices of the modifier meshes
    /// 5) Applies size compensation (offsets the slices in XY plane)
    /// 6) Replaces bad slices by the slices reconstructed from the upper/lower layer
    /// Resulting expolygons of layer regions are marked as Internal.
    ///
    /// this should be idempotent
    pub fn slice_volumes(&mut self, print: &Print, cores: &mut dyn SliceVolumesCores) -> Result<(), PrintObjectSliceError> {
        let throw_on_cancel_callback = || print.base.base.throw_if_canceled();
        let shared = self.m_shared_regions.clone().expect("the shared regions of the object");

        // Clear old LayerRegions, allocate for new PrintRegions.
        {
            let n_regions = shared.borrow().all_regions.len();
            for layer in self.m_layers.iter_mut() {
                //BBS: should delete all LayerRegionPtr to avoid memory leak
                layer.clear_regions();
                let layer_id = layer.id();
                for region_id in 0..n_regions {
                    layer.add_region(LayerRegion::new(layer_id, region_id));
                }
            }
        }

        let slice_zs: Vec<f32> = zs_from_layers(&self.m_layers);
        let mut obj_slice_by_volume: Vec<VolumeSlices> = Vec::new();
        if !slice_zs.is_empty() {
            obj_slice_by_volume = slice_volumes_inner(
                print.config(),
                self.config(),
                &self.trafo_centered(),
                &self.model_object(print).volumes,
                &shared.borrow().layer_ranges,
                &slice_zs,
                &throw_on_cancel_callback,
                Some(print),
            )?;
        }

        //BBS: "model_part" volumes are grouded according to their connections
        self.firstLayerObjSliceByVolume = obj_slice_by_volume.clone();

        print.base.base.set_status(10, "Slicing mesh", SlicingStatus::DEFAULT, -1);
        let mut region_slices: Vec<Vec<ExPolygons>> = slices_to_regions(
            &self.model_object(print).volumes,
            &shared.borrow(),
            &slice_zs,
            obj_slice_by_volume,
            clip_multipart_objects.load(Ordering::Relaxed),
            cfg_bool(print.config(), "enable_order_independent_overlap_carving"),
            &throw_on_cancel_callback,
        )?;

        for region_id in 0..region_slices.len() {
            let by_layer = &mut region_slices[region_id];
            for layer_id in 0..by_layer.len() {
                self.m_layers[layer_id].regions_mut()[region_id].slices.append(std::mem::take(&mut by_layer[layer_id]), SurfaceType::Internal);
            }
        }
        region_slices.clear();

        // "Slicing volumes - removing top empty layers"
        while !self.m_layers.is_empty() {
            let layer = &self.m_layers[self.m_layers.len() - 1];
            if !layer.empty() {
                break;
            }
            self.m_layers.pop();
        }
        if let Some(back) = self.m_layers.last_mut() {
            back.upper_layer_id = None;
        }
        throw_on_cancel_callback()?;

        let skip_filament_shrink = should_skip_filament_shrink(print, self);

        if skip_filament_shrink {
            active_step_add_warning(print, self, WarningLevel::Critical, "Shrinkage compensation does not take effect on color-painted models.");
        }

        // Is any ModelVolume MMU painted?
        if cfg_floats(print.config(), "filament_diameter").len() > 1 // BBS
            && self.model_object(print).volumes.iter().any(|v| v.is_model_part() && !v.mmu_segmentation_facets.is_empty())
        {
            // If XY Size compensation is also enabled, notify the user that XY Size compensation
            // would not be used because the object is multi-material painted.
            if cfg_float(&self.m_config, "xy_hole_compensation") != 0. || cfg_float(&self.m_config, "xy_contour_compensation") != 0. {
                active_step_add_warning(
                    print,
                    self,
                    WarningLevel::Critical,
                    "An object's XY size compensation will not be used because it is also color-painted.\nXY Size compensation can not be combined with color-painting.",
                );
            }

            // "Slicing volumes - MMU segmentation"
            apply_mm_segmentation(print, self, cores, &throw_on_cancel_callback)?;
        }

        // Is any ModelVolume fuzzy skin painted?
        if self.model_object(print).is_fuzzy_skin_painted() {
            // If XY Size compensation is also enabled, notify the user that XY Size compensation
            // would not be used because the object has custom fuzzy skin painted.
            if cfg_float(&self.m_config, "xy_hole_compensation") != 0. || cfg_float(&self.m_config, "xy_contour_compensation") != 0. {
                let message = format!(
                    "An object has enabled XY Size compensation which will not be used because it is also fuzzy skin painted.\nXY Size compensation cannot be combined with fuzzy skin painting.\nObject name: {}",
                    self.model_object(print).name
                );
                active_step_add_warning(print, self, WarningLevel::Critical, &message);
            }

            // "Slicing volumes - Fuzzy skin segmentation"
            apply_fuzzy_skin_segmentation(print, self, cores, &throw_on_cancel_callback)?;
        }

        cores.generate_interlocking_structure(print, self);
        throw_on_cancel_callback()?;

        cores.generate_embedding_wall(print, self);
        throw_on_cancel_callback()?;

        // TODO(filament-shrink): refactor — apply per role/region in perimeter & infill stages instead of baking
        //   into posSlice geometry. Current impl scales whole region by the wall_filament's shrink, which:
        //     (1) forces posSlice invalidation whenever wall_filament changes (see PrintObject::invalidate_state_by_config_options),
        //     (2) ignores per-role shrink (infill/solid_infill filament may differ from wall_filament).
        // SuperSlicer: filament shrink. Color-painted objects use the same multi-material gate as
        // XY compensation and skip scaling when any filament used by this object enables shrinkage.
        if !skip_filament_shrink {
            let shrink = cfg_floats(print.config(), "filament_shrink");
            for layer in self.m_layers.iter_mut() {
                for i in 0..layer.region_count() {
                    let region_ptr = shared.borrow().all_regions[layer.regions()[i].region_id()].clone();
                    let region = &mut layer.regions_mut()[i];
                    let mut ex_polys = to_expolygons(&region.slices.surfaces);
                    // extruder() returns 0 for a region that inherits the object's filament, and the
                    // per-filament vector can be shorter than the number of filaments the object uses,
                    // so both ends have to be range checked (same guard as support_shrinkage_scale()).
                    let filament_id = region_ptr.borrow().extruder(FlowRole::Perimeter) as i32 - 1;
                    let mut scale: f64 = if filament_id < 0 || filament_id >= shrink.len() as i32 { 1. } else { shrink[filament_id as usize] * 0.01 };
                    if scale != 1. && scale != 0. {
                        scale = 1. / scale;
                        for poly in ex_polys.iter_mut() {
                            expolygon_scale(poly, scale);
                        }
                    }

                    region.slices.set(&ex_polys, SurfaceType::Internal);
                }
            }
            // firstLayerObjSlice is captured from raw volume slices before this compensation.
            // Brim grouping matches those slices against compensated lslices; scale them by the
            // same factor or thin-walled rings can stop overlapping (GitHub #12186, feishu issue-7112906345).
            let shrink_scale = self.support_shrinkage_scale(print);
            if shrink_scale != 1. {
                for vs in self.firstLayerObjSliceByVolume.iter_mut() {
                    for expolys in vs.slices.iter_mut() {
                        for poly in expolys.iter_mut() {
                            expolygon_scale(poly, shrink_scale);
                        }
                    }
                }
            }
        }

        // "Slicing volumes - make_slices in parallel - begin"
        {
            // Compensation value, scaled. Only applying the negative scaling here, as the positive scaling has already been applied during slicing.
            let num_extruders = cfg_floats(print.config(), "filament_diameter").len();
            let mm_painted = num_extruders > 1 && self.is_mm_painted(print);
            // scaled<float>(value) = float(value / SCALING_FACTOR)
            let xy_hole_scaled: f32 = if mm_painted { 0. } else { (cfg_float(&self.m_config, "xy_hole_compensation") / SCALING_FACTOR) as f32 };
            let xy_contour_scaled: f32 = if mm_painted { 0. } else { (cfg_float(&self.m_config, "xy_contour_compensation") / SCALING_FACTOR) as f32 };
            let elephant_foot_compensation_scaled: f32 = if cfg_int(&self.m_config, "raft_layers") == 0 {
                // Only enable Elephant foot compensation if printing directly on the print bed.
                (cfg_float(&self.m_config, "elefant_foot_compensation") / SCALING_FACTOR) as f32
            } else {
                0.
            };
            // Uncompensated slices for the first layer in case the Elephant foot compensation is applied.
            let mut lslices_1st_layer = ExPolygons::new();
            //BBS: this part has been changed a lot to support seperated contour and hole size compensation
            for layer_id in 0..self.m_layers.len() {
                throw_on_cancel_callback()?;
                // Apply size compensation and perform clipping of multi-part objects.
                let elfoot: f32 = if layer_id == 0 { elephant_foot_compensation_scaled } else { 0. };
                if self.m_layers[layer_id].regions().len() == 1 {
                    // Optimized version for a single region layer.
                    // Single region, growing or shrinking.
                    if elfoot > 0. {
                        // Apply the elephant foot compensation and store the 1st layer slices without the Elephant foot compensation applied.
                        lslices_1st_layer = to_expolygons(&self.m_layers[layer_id].regions()[0].slices.surfaces);
                        self.m_layers[layer_id].regions_mut()[0].slices.surfaces.clear();
                        if xy_contour_scaled > 0. || xy_hole_scaled > 0. {
                            lslices_1st_layer = self._shrink_contour_holes(0f32.max(xy_contour_scaled) as f64, 0f32.max(xy_hole_scaled) as f64, &lslices_1st_layer);
                        }
                        if xy_contour_scaled < 0. || xy_hole_scaled < 0. {
                            lslices_1st_layer = self._shrink_contour_holes(0f32.min(xy_contour_scaled) as f64, 0f32.min(xy_hole_scaled) as f64, &lslices_1st_layer);
                        }
                        let flow = layer_region_flow(print, self, &self.m_layers[layer_id], 0, FlowRole::ExternalPerimeter)?;
                        // unscale<double>(elfoot)
                        let compensated = elephant_foot_compensation(&lslices_1st_layer, &flow, elfoot as f64 * SCALING_FACTOR);
                        self.m_layers[layer_id].regions_mut()[0].slices.set(&union_ex_expolygons(&compensated), SurfaceType::Internal);
                    } else {
                        // Apply the XY contour and hole size compensation.
                        if xy_contour_scaled != 0.0 || xy_hole_scaled != 0.0 {
                            let mut expolygons = to_expolygons(&self.m_layers[layer_id].regions()[0].slices.surfaces);
                            self.m_layers[layer_id].regions_mut()[0].slices.surfaces.clear();
                            if xy_contour_scaled > 0. || xy_hole_scaled > 0. {
                                expolygons = self._shrink_contour_holes(0f32.max(xy_contour_scaled) as f64, 0f32.max(xy_hole_scaled) as f64, &expolygons);
                            }
                            if xy_contour_scaled < 0. || xy_hole_scaled < 0. {
                                expolygons = self._shrink_contour_holes(0f32.min(xy_contour_scaled) as f64, 0f32.min(xy_hole_scaled) as f64, &expolygons);
                            }
                            self.m_layers[layer_id].regions_mut()[0].slices.set(&expolygons, SurfaceType::Internal);
                        }
                    }
                } else {
                    let max_growth: f32 = (-xy_hole_scaled).max(xy_contour_scaled);
                    let min_growth: f32 = (-xy_hole_scaled).min(xy_contour_scaled);
                    // for hole, positive xy_hole means increase hole and shrink expolygon
                    if max_growth > 0. {
                        //BBS: merge polygons because region can cut "holes".
                        //Then, cut them to give them again later to their region
                        let mut merged_poly_for_holes_growing = layer_merged(self, &self.m_layers[layer_id], (EPSILON / SCALING_FACTOR) as f32);
                        merged_poly_for_holes_growing = self._shrink_contour_holes(
                            0f32.max(xy_contour_scaled) as f64,
                            0f32.min(xy_hole_scaled) as f64, // negetative offset expand expolygons
                            &union_ex_expolygons(&merged_poly_for_holes_growing),
                        );

                        // BBS: clipping regions, priority is given to the first regions.
                        let mut processed = Polygons::new();
                        let n_regions = self.m_layers[layer_id].regions().len();
                        for region_id in 0..n_regions {
                            let mut slices = to_expolygons(&self.m_layers[layer_id].regions()[region_id].slices.surfaces);
                            self.m_layers[layer_id].regions_mut()[region_id].slices.surfaces.clear();
                            if max_growth > 0. {
                                slices = intersection_ex(&offset_ex(&slices, max_growth as f64), &merged_poly_for_holes_growing);
                            }

                            //BBS: Trim by the slices of already processed regions.
                            if region_id > 0 {
                                slices = diff_ex_polygons(&to_polygons(&slices), &processed);
                            }
                            if region_id + 1 < n_regions {
                                // Collect the already processed regions to trim the to be processed regions.
                                processed.extend(to_polygons(&slices));
                            }
                            self.m_layers[layer_id].regions_mut()[region_id].slices.set(&slices, SurfaceType::Internal);
                        }
                    }
                    if min_growth < 0. || elfoot > 0. {
                        // Apply the negative XY compensation. (the ones that is <0)
                        let mut trimming;
                        // static const float eps = float(scale_(m_config.slice_closing_radius.value) * 1.5);
                        static EPS: std::sync::OnceLock<f32> = std::sync::OnceLock::new();
                        let eps = *EPS.get_or_init(|| (cfg_float(&self.m_config, "slice_closing_radius") / SCALING_FACTOR * 1.5) as f32);
                        if elfoot > 0. {
                            lslices_1st_layer = offset_ex(&layer_merged(self, &self.m_layers[layer_id], eps), -eps as f64);
                            let flow = layer_region_flow(print, self, &self.m_layers[layer_id], 0, FlowRole::ExternalPerimeter)?;
                            trimming = elephant_foot_compensation(&lslices_1st_layer, &flow, elfoot as f64 * SCALING_FACTOR);
                        } else {
                            trimming = layer_merged(self, &self.m_layers[layer_id], (EPSILON / SCALING_FACTOR) as f32);
                        }
                        if min_growth < 0.0 {
                            trimming = self._shrink_contour_holes(
                                0f32.min(xy_contour_scaled) as f64,
                                0f32.max(xy_hole_scaled) as f64, //positive xy_hole means shrink
                                &trimming,
                            );
                        }
                        //BBS: trim surfaces
                        for region_id in 0..self.m_layers[layer_id].regions().len() {
                            // BBS: split trimming result by region
                            let contour_exp = to_expolygons(&self.m_layers[layer_id].regions()[region_id].slices.surfaces);

                            self.m_layers[layer_id].regions_mut()[region_id]
                                .slices
                                .set(&intersection_ex_polygons(&contour_exp, &to_polygons(&trimming)), SurfaceType::Internal);
                        }
                    }
                }
                // Merge all regions' slices to get islands, chain them by a shortest path.
                if cfg_bool(self.config(), "enable_circle_compensation") {
                    // PrintObjectSlice.cpp:2386-2387 `layer->apply_auto_circle_compensation();`, which reads
                    // `get_auto_circle_compenstaion_params()` (set from the PrintConfig, Print.cpp:2334, 2345,
                    // 2482) and `scale_(config().circle_compensation_manual_offset)` passed as a float
                    // (Layer.cpp:41-43). AUTO_CIRCLE_COMPENSATION_PIN (layer.rs, on unless =0) keeps the
                    // earlier no-op when 0.
                    let params = crate::layer_region::AutoContourHolesCompensationParams::new_from_config_values(
                        |key| cfg_floats(print.config(), key),
                    );
                    let manual_offset = (cfg_float(self.config(), "circle_compensation_manual_offset")
                        / crate::libslic3r::SCALING_FACTOR) as f32;
                    self.m_layers[layer_id].apply_auto_circle_compensation_with(&params, manual_offset);
                }
                self.m_layers[layer_id].make_slices();
            }
            if elephant_foot_compensation_scaled > 0. && !self.m_layers.is_empty() {
                // The Elephant foot has been compensated, therefore the 1st layer's lslices are shrank with the Elephant foot compensation value.
                // Store the uncompensated value there.
                debug_assert!(self.m_layers[0].id() == 0);
                //BBS: sort the lslices_1st_layer according to shortest path before saving
                //Otherwise the travel of first layer would be mess.
                let mut ordering_points: Vec<Point> = Vec::with_capacity(lslices_1st_layer.len());
                for ex in &lslices_1st_layer {
                    ordering_points.push(ex.contour.points[0]);
                }
                let order = crate::shortest_path::chain_points(&ordering_points, None);
                let mut lslices_1st_layer_sorted: ExPolygons = Vec::with_capacity(lslices_1st_layer.len());
                for i in order {
                    lslices_1st_layer_sorted.push(std::mem::take(&mut lslices_1st_layer[i]));
                }

                self.m_layers[0].lslices = lslices_1st_layer_sorted;
            }
        }

        throw_on_cancel_callback()?;
        // "Slicing volumes - make_slices in parallel - end"
        Ok(())
    }

    /// PrintObjectSlice.cpp:2415-2447 `ExPolygons PrintObject::_shrink_contour_holes(double contour_delta, double
    /// hole_delta, const ExPolygons& polys) const`.
    ///
    /// BBS: this function is used to offset contour and holes of expolygons seperately by different value
    pub fn _shrink_contour_holes(&self, contour_delta: f64, hole_delta: f64, polys: &ExPolygons) -> ExPolygons {
        let mut new_ex_polys = ExPolygons::new();
        for ex_poly in polys {
            let mut contours = Polygons::new();
            let mut holes = Polygons::new();
            //BBS: modify hole
            for hole in &ex_poly.holes {
                if hole_delta != 0. {
                    for mut new_hole in offset_polygon(hole, -hole_delta) {
                        new_hole.make_counter_clockwise();
                        holes.push(new_hole);
                    }
                } else {
                    holes.push(hole.clone());
                    holes.last_mut().unwrap().make_counter_clockwise();
                }
            }
            //BBS: modify contour
            if contour_delta != 0. {
                let new_contours = offset_polygon(&ex_poly.contour, contour_delta);
                if new_contours.is_empty() {
                    continue;
                }
                contours.extend(new_contours);
            } else {
                contours.push(ex_poly.contour.clone());
            }
            let temp = diff_ex_polygons(&union_(&contours), &union_(&holes));
            new_ex_polys.extend(temp);
        }
        union_ex_expolygons(&new_ex_polys)
    }

    /// PrintObjectSlice.cpp:2449-2477 `double PrintObject::support_shrinkage_scale() const`
    pub fn support_shrinkage_scale(&self, print: &Print) -> f64 {
        if self.num_printing_regions() == 0 {
            return 1.;
        }
        // Keep supports aligned with object contours whenever object shrinkage is skipped.
        if should_skip_filament_shrink(print, self) {
            return 1.;
        }
        let shrink = cfg_floats(print.config(), "filament_shrink");
        if shrink.is_empty() {
            return 1.;
        }

        let mut shrink_percent: f64 = 0.;
        let mut found = false;
        for i in 0..self.num_printing_regions() {
            let filament_id = self.printing_region(i).borrow().extruder(FlowRole::Perimeter) as i32 - 1;
            if filament_id < 0 || filament_id >= shrink.len() as i32 {
                continue;
            }
            if !found {
                shrink_percent = shrink[filament_id as usize];
                found = true;
            } else if shrink[filament_id as usize] != shrink_percent {
                break;
            }
        }
        if !found || shrink_percent == 0. || shrink_percent == 100. {
            return 1.;
        }
        100. / shrink_percent // == 1 / (shrink_percent * 0.01)
    }

    /// PrintObjectSlice.cpp:2479-2537 `std::vector<Polygons> PrintObject::slice_support_volumes(const ModelVolumeType
    /// model_volume_type) const`
    pub fn slice_support_volumes(&self, print: &Print, model_volume_type: ModelVolumeType) -> Result<Vec<Polygons>, CanceledException> {
        let volumes = &self.model_object(print).volumes;
        let mut it_volume = 0usize;
        let it_volume_end = volumes.len();
        while it_volume != it_volume_end && volumes[it_volume].type_() != model_volume_type {
            it_volume += 1;
        }
        let mut slices: Vec<Polygons> = Vec::new();
        if it_volume != it_volume_end {
            // Found at least a single support volume of model_volume_type.
            let zs: Vec<f32> = zs_from_layers(self.layers());
            let mut merge_layers: Vec<bool> = Vec::new();
            let mut merge = false;
            let throw_on_cancel_callback = || print.base.base.throw_if_canceled();
            let mut params = MeshSlicingParamsEx::default();
            params.trafo = self.trafo_centered();
            while it_volume != it_volume_end {
                if volumes[it_volume].type_() == model_volume_type {
                    let slices2 = slice_volume(&volumes[it_volume], &zs, &params, &throw_on_cancel_callback)?;
                    if slices.is_empty() {
                        slices.reserve(slices2.len());
                        for src in &slices2 {
                            slices.push(to_polygons(src));
                        }
                    } else if !slices2.is_empty() {
                        if merge_layers.is_empty() {
                            merge_layers = vec![false; zs.len()];
                        }
                        for i in 0..zs.len() {
                            if slices[i].is_empty() {
                                slices[i] = to_polygons(&slices2[i]);
                            } else if !slices2[i].is_empty() {
                                slices[i].extend(to_polygons(&slices2[i]));
                                merge_layers[i] = true;
                                merge = true;
                            }
                        }
                    }
                }
                it_volume += 1;
            }
            if merge {
                for i in 0..zs.len() {
                    if merge_layers[i] {
                        slices[i] = union_(&slices[i]);
                    }
                }
            }

            // Align the support-volume slices with the filament_shrink-compensated object contours.
            let shrink_scale = self.support_shrinkage_scale(print);
            if shrink_scale != 1. {
                for polys in slices.iter_mut() {
                    for poly in polys.iter_mut() {
                        for p in poly.points.iter_mut() {
                            p.x = (p.x as f64 * shrink_scale) as i64;
                            p.y = (p.y as f64 * shrink_scale) as i64;
                        }
                    }
                }
            }
        }
        Ok(slices)
    }
}

/// PrintObjectSlice.cpp:1159-1849 `static void normalize_mm_segmentation_to_layer_bands(PrintObject
/// &print_object, std::vector<std::vector<ExPolygons>> &segmentation, const std::function<void()>
/// &throw_on_cancel)`. `print` is `print_object.print()`. The `tbb::parallel_for` loops over the bands
/// and over the layers run in order: a band reads and writes only the layers it covers, a layer only
/// itself. What the C++ only logs (`[mm_band]`) is left out.
///
/// A nozzle coarser than the finest one on the plate does not print every layer: it covers a whole band of
/// them with a single extrusion shaped like the band's top layer. Painting, however, is done in object space
/// and knows nothing about that. Paint that never reaches a band top would be decimated away together with
/// the layers it sits on, leaving a void that nobody fills, while paint that does reach one silently owns
/// the layers underneath, where a finer nozzle would otherwise lay a second colour into the same Z.
///
/// Both are settled here, before the paint is cut into regions, by making each coarse extruder's paint
/// constant across its own bands: empty on the band top means the paint is dropped and the area falls back
/// to whoever owned it before painting, which is what never having painted it looks like. A coarse colour
/// left thinner than its nozzle can extrude is finally handed to a neighbouring colour.
pub fn normalize_mm_segmentation_to_layer_bands(
    print: &Print,
    print_object: &PrintObject,
    segmentation: &mut Vec<Vec<ExPolygons>>,
    throw_on_cancel: ThrowOnCancel,
) -> Result<(), CanceledException> {
    use crate::adaptive_layer_merge::{
        adaptive_layer_band_size, adaptive_layer_height_range, adaptive_printable_pieces, build_adaptive_layer_bands,
        pull_adaptive_band_top_to_material, AdaptiveLayerBand, AdaptiveLayerHeightRange,
    };
    use crate::print::faithful::nozzle_diameter_for_filament;

    // ExPolygon.hpp:488 `area(const ExPolygons &polys)`
    let area_mm2 = |expolygons: &[ExPolygon]| -> f64 {
        let mut s: f64 = 0.;
        for p in expolygons {
            s += p.area();
        }
        s * SCALING_FACTOR * SCALING_FACTOR
    };
    let printable_pieces = |pieces: ExPolygons, diameter: f64| -> ExPolygons { adaptive_printable_pieces(pieces, diameter) };

    let layer_count = segmentation.len();
    // Enter only when the print uses the adaptive diameter ladder. A single coarse filament may pass this
    // gate, but without a second painted diameter there is no cross-diameter segmentation to normalize.
    if layer_count < 2 || !print.uses_multiple_nozzle_diameters() {
        return Ok(());
    }
    let extruder_count = segmentation[0].len();
    if extruder_count < 2 {
        return Ok(());
    }
    for layer in segmentation.iter() {
        if layer.len() != extruder_count {
            return Ok(());
        }
    }

    let print_config = print.config();
    // `layerm->region().config().wall_filament` of a LayerRegion of the object.
    let wall_filament = |layerm: &LayerRegion| -> u32 {
        let region = print_object.printing_region(layerm.region_id());
        let region = region.borrow();
        cfg_int(region.config(), "wall_filament") as u32
    };
    // Only the extruders the paint actually hands area to take part. The region set carries an entry per
    // filament of the plate, and an entry this object never prints has no filament to read a diameter from,
    // so it would report a diameter the plate never uses.
    let mut extruder_nozzles: Vec<f64> = vec![0.; extruder_count];
    let mut min_nozzle: f64 = 0.;
    let mut max_nozzle: f64 = 0.;
    for extruder_id in 0..extruder_count {
        let mut prints_anything = false;
        for layer in segmentation.iter() {
            if !layer[extruder_id].is_empty() {
                prints_anything = true;
                break;
            }
        }
        if !prints_anything {
            continue;
        }
        let diameter: f64 = nozzle_diameter_for_filament(print_config, extruder_id as u32 + 1) as f64;
        if diameter <= EPSILON {
            continue;
        }
        extruder_nozzles[extruder_id] = diameter;
        if min_nozzle <= EPSILON || diameter < min_nozzle {
            min_nozzle = diameter;
        }
        if diameter > max_nozzle {
            max_nozzle = diameter;
        }
    }
    // Whatever the paint leaves alone keeps its region's own filament. That area is absent from
    // `segmentation`, yet it is extruded in bands like everything else, so its nozzle counts as in use too -
    // without it an object painted with a single colour looks like a single nozzle here and nothing below
    // is normalized at all.
    let mut base_nozzles: Vec<f64> = Vec::new();
    for layer_idx in 0..layer_count {
        for layerm in print_object.get_layer(layer_idx as i32).regions() {
            if layerm.slices.surfaces.is_empty() {
                continue;
            }
            let diameter: f64 = nozzle_diameter_for_filament(print_config, wall_filament(layerm)) as f64;
            if diameter <= EPSILON {
                continue;
            }
            if !base_nozzles.iter().any(|&known| (known - diameter).abs() <= EPSILON) {
                base_nozzles.push(diameter);
            }
            if min_nozzle <= EPSILON || diameter < min_nozzle {
                min_nozzle = diameter;
            }
            if diameter > max_nozzle {
                max_nozzle = diameter;
            }
        }
    }
    let _ = max_nozzle;
    if min_nozzle <= EPSILON {
        return Ok(());
    }
    // Bands are counted off the grid the plate was sliced on, not off the nozzles it happens to use, so that
    // this grid is the one compute_adaptive_merge_groups() decimates on. Both the reference diameter and the
    // base height must come from the sliced layer stack: the process preset's layer height is only the
    // authored value, which SlicingParameters may have subdivided to resolve a finer nozzle.
    let primary: f64 = crate::print_config::config_def::layer_grid_nozzle_diameter(print.full_print_config());
    let reference_nozzle: f64 = if primary > EPSILON { primary } else { min_nozzle };
    let base_layer_height: f64 = print_object.slicing_parameters().layer_height;
    let uses_merged_ladder =
        |diameter: f64| -> bool { diameter > EPSILON && adaptive_layer_band_size(diameter, reference_nozzle, base_layer_height) >= 2 };
    let has_merged_ladder = extruder_nozzles.iter().any(|&d| uses_merged_ladder(d)) || base_nozzles.iter().any(|&d| uses_merged_ladder(d));
    if !has_merged_ladder {
        return Ok(());
    }

    let mut layer_heights: Vec<f64> = vec![0.; layer_count];
    for layer_idx in 0..layer_count {
        layer_heights[layer_idx] = print_object.get_layer(layer_idx as i32).height;
    }

    // Keep every nozzle on its own fixed ladder. Colour ownership is normalized to the band's anchor
    // contour below; colour starts, stops and layer-to-layer footprint changes must not split the ladder.
    let band_tops: Vec<u8> = Vec::new();

    // A coarse colour discarded below its band top may have overpainted another colour there. Restore
    // that colour from the immutable top-layer ownership before normalizing any extruder; otherwise the
    // vacated area is absent from segmentation altogether and falls through to the model's base filament.
    // Doing this as a pre-pass also lets a restored coarse owner pass through its own normalization below.
    let original_segmentation: Vec<Vec<ExPolygons>> = segmentation.clone();
    // The ladder belongs to the nozzle diameter, not to a colour. Colours sharing one diameter must
    // use the same phase; ownership changes only at the common band's anchor layer.
    let build_painted_bands = |band_size: usize, range: &AdaptiveLayerHeightRange| -> Vec<AdaptiveLayerBand> {
        build_adaptive_layer_bands(&layer_heights, band_size, range, &band_tops)
    };
    // Two ladders of incommensurate band sizes (a 0.6 and a 0.8 on the same grid) make a colour's band top
    // land on a layer the other colour owns over and over. Anchoring on that layer would release the whole
    // band, so the top is pulled down to the last layer this colour really owns - the same shortening
    // compute_adaptive_merge_groups() applies, which is what keeps the two passes describing one band.
    let pull_painted_band = |src: &AdaptiveLayerBand, extruder_id: usize, range: &AdaptiveLayerHeightRange, out: &mut AdaptiveLayerBand| -> bool {
        *out = *src;
        if src.bottom_idx == 0 {
            return true;
        }
        let mut has_material: Vec<u8> = vec![0; src.top_idx + 1];
        let mut layer_idx = src.bottom_idx;
        while layer_idx <= src.top_idx && layer_idx < original_segmentation.len() {
            has_material[layer_idx] = (!original_segmentation[layer_idx][extruder_id].is_empty()) as u8;
            layer_idx += 1;
        }
        pull_adaptive_band_top_to_material(out, &layer_heights, range, &has_material)
    };
    // Ownership on a band's anchor layer is a real extrusion. Protect it from another nozzle whose
    // differently aligned band merely passes through that layer on the way to its own anchor.
    let mut painted_anchor_claims: Vec<Vec<ExPolygons>> = vec![vec![ExPolygons::new(); extruder_count]; layer_count];
    for extruder_id in 0..extruder_count {
        let nozzle = extruder_nozzles[extruder_id];
        if nozzle <= EPSILON {
            continue;
        }
        let band_size = adaptive_layer_band_size(nozzle, reference_nozzle, base_layer_height);
        if band_size < 2 {
            // An unmerged nozzle anchors a real extrusion on every physical layer.
            for layer_idx in 0..layer_count {
                painted_anchor_claims[layer_idx][extruder_id] = original_segmentation[layer_idx][extruder_id].clone();
            }
            continue;
        }
        let range = adaptive_layer_height_range(nozzle);
        for band in &build_painted_bands(band_size, &range) {
            let mut live = AdaptiveLayerBand::default();
            if !pull_painted_band(band, extruder_id, &range, &mut live) {
                continue;
            }
            let anchor_idx = if band.bottom_idx == 0 { 0 } else { live.top_idx };
            painted_anchor_claims[anchor_idx][extruder_id] = original_segmentation[anchor_idx][extruder_id].clone();
        }
    }
    let other_anchor_claims = |layer_idx: usize, extruder_id: usize| -> ExPolygons {
        let mut protected_area = Polygons::new();
        for other_id in 0..extruder_count {
            if other_id != extruder_id {
                protected_area.extend(to_polygons(&painted_anchor_claims[layer_idx][other_id]));
            }
        }
        if protected_area.is_empty() {
            ExPolygons::new()
        } else {
            union_ex(&protected_area)
        }
    };
    let all_anchor_claims = |layer_idx: usize| -> ExPolygons {
        let mut protected_area = Polygons::new();
        for extruder_id in 0..extruder_count {
            protected_area.extend(to_polygons(&painted_anchor_claims[layer_idx][extruder_id]));
        }
        if protected_area.is_empty() {
            ExPolygons::new()
        } else {
            union_ex(&protected_area)
        }
    };
    for extruder_id in 0..extruder_count {
        let nozzle = extruder_nozzles[extruder_id];
        if nozzle <= EPSILON {
            continue;
        }
        let band_size = adaptive_layer_band_size(nozzle, reference_nozzle, base_layer_height);
        if band_size < 2 {
            continue;
        }
        let range = adaptive_layer_height_range(nozzle);
        let bands = build_painted_bands(band_size, &range);
        for band_idx in 0..bands.len() {
            throw_on_cancel()?;
            let band = &bands[band_idx];
            let mut live = AdaptiveLayerBand::default();
            if !pull_painted_band(band, extruder_id, &range, &mut live) {
                continue;
            }
            let anchor_idx = if band.bottom_idx == 0 { 0 } else { live.top_idx };
            for layer_idx in live.bottom_idx..=live.top_idx {
                if layer_idx == anchor_idx {
                    continue;
                }
                let vacated = diff_ex(&original_segmentation[layer_idx][extruder_id], &original_segmentation[anchor_idx][extruder_id]);
                if vacated.is_empty() {
                    continue;
                }
                for other_id in 0..extruder_count {
                    if other_id == extruder_id || original_segmentation[anchor_idx][other_id].is_empty() {
                        continue;
                    }
                    let restored = intersection_ex(&vacated, &original_segmentation[anchor_idx][other_id]);
                    if restored.is_empty() {
                        continue;
                    }
                    let mut combined = to_polygons(&segmentation[layer_idx][other_id]);
                    combined.extend(to_polygons(&restored));
                    segmentation[layer_idx][other_id] = union_ex(&combined);
                }
            }
        }
    }

    // Ascending extruder id, the order painted_regions are applied in, so a contested area goes to the
    // same extruder here as it would in apply_mm_segmentation() below.
    let mut claimed: Vec<ExPolygons> = vec![ExPolygons::new(); layer_count];
    for extruder_id in 0..extruder_count {
        let nozzle = extruder_nozzles[extruder_id];
        if nozzle <= EPSILON {
            continue;
        }
        let band_size = adaptive_layer_band_size(nozzle, reference_nozzle, base_layer_height);
        if band_size < 2 {
            continue;
        }
        let range = adaptive_layer_height_range(nozzle);
        let bands = build_painted_bands(band_size, &range);
        for band_idx in 0..bands.len() {
            throw_on_cancel()?;
            let band = &bands[band_idx];
            let mut live = AdaptiveLayerBand::default();
            if !pull_painted_band(band, extruder_id, &range, &mut live) {
                continue;
            }
            // The band shows one face over its whole Z range and takes its ownership from that face: the
            // top layer normally, the object base for the band that starts at layer 0. That band is the
            // bottom of the print, so paint reaching it only down a vertical wall must not claim it.
            let anchored_on_base = band.bottom_idx == 0;
            let anchor_idx = if anchored_on_base { 0 } else { live.top_idx };
            let retained: ExPolygons = segmentation[anchor_idx][extruder_id].clone();
            for layer_idx in live.bottom_idx..=live.top_idx {
                // The band lays the top contour down and nothing else, so a footprint this extruder painted
                // further down is not extruded at all. It still belongs to this nozzle: releasing it lets the
                // base filament print it layer by layer inside the Z range the band already owns, which comes
                // out as a rim of the model colour tracing every band step - a colour change on every layer
                // instead of the step the decimation implies. Only what another colour holds on the band top
                // is given up, and the restore pass above has moved exactly that to its owner already.
                let mut own: ExPolygons = retained.clone();
                // Never on a base-anchored band: what it would recover there is exactly the side paint the
                // first band must not carry, and layer 0 has already said who owns the bottom face.
                if !retained.is_empty() && !anchored_on_base && layer_idx != anchor_idx {
                    let mut orphaned = diff_ex(&original_segmentation[layer_idx][extruder_id], &original_segmentation[anchor_idx][extruder_id]);
                    let mut other_id = 0;
                    while other_id < extruder_count && !orphaned.is_empty() {
                        if other_id != extruder_id && !original_segmentation[anchor_idx][other_id].is_empty() {
                            orphaned = diff_ex(&orphaned, &original_segmentation[anchor_idx][other_id]);
                        }
                        other_id += 1;
                    }
                    if !orphaned.is_empty() {
                        let mut combined = to_polygons(&own);
                        combined.extend(to_polygons(&orphaned));
                        own = union_ex(&combined);
                    }
                }
                let protected_area = other_anchor_claims(layer_idx, extruder_id);
                if !own.is_empty() && !protected_area.is_empty() {
                    own = diff_ex(&own, &protected_area);
                }
                let area: ExPolygons = if own.is_empty() || claimed[layer_idx].is_empty() { own } else { diff_ex(&own, &claimed[layer_idx]) };
                if !area.is_empty() {
                    // One extrusion fills the band's whole Z range, so no other extruder can keep a share
                    // of these layers.
                    for other_id in 0..extruder_count {
                        if other_id != extruder_id && !segmentation[layer_idx][other_id].is_empty() {
                            // Preserve the colour ownership even when the remainder is narrow. Returning
                            // such a boundary strip to the base region inserts a third colour between the
                            // two painted colours; the wall generator is the right place to decide whether
                            // it becomes a variable-width wall or gap fill.
                            segmentation[layer_idx][other_id] = diff_ex(&segmentation[layer_idx][other_id], &area);
                        }
                    }
                    let mut combined = to_polygons(&claimed[layer_idx]);
                    combined.extend(to_polygons(&area));
                    claimed[layer_idx] = union_ex(&combined);
                }
                segmentation[layer_idx][extruder_id] = area;
            }
        }
    }

    // The unpainted footprint prints in bands as well, and the band carrying the object's top surface has
    // to own its whole Z range. A colour painted on a side wall that keeps extruding on the layers inside
    // that band leaves the merged slab nowhere to go: compute_adaptive_merge_groups() trims the slab
    // against what the finer nozzle already laid down, so the top face comes out short of the model
    // outline by that strip and stops being the shape the user drew.
    let base_at = |layer_idx: usize, diameter: f64| -> ExPolygons {
        let mut base = Polygons::new();
        for layerm in print_object.get_layer(layer_idx as i32).regions() {
            if !layerm.slices.surfaces.is_empty()
                && (nozzle_diameter_for_filament(print_config, wall_filament(layerm)) as f64 - diameter).abs() <= EPSILON
            {
                base.extend(crate::surface::to_polygons(&layerm.slices.surfaces));
            }
        }
        if base.is_empty() {
            ExPolygons::new()
        } else {
            union_ex(&base)
        }
    };
    let unpainted_at = |segmentation: &Vec<Vec<ExPolygons>>, layer_idx: usize, diameter: f64| -> ExPolygons {
        let base = base_at(layer_idx, diameter);
        if base.is_empty() {
            return ExPolygons::new();
        }
        let mut painted = Polygons::new();
        for extruder_id in 0..extruder_count {
            painted.extend(to_polygons(&segmentation[layer_idx][extruder_id]));
        }
        if painted.is_empty() {
            return base;
        }
        // Only what this nozzle could actually lay down may evict paint. A layer the paint covers entirely
        // still differs from its own slice by a hairline, and keeping that hairline would slit the wider
        // layers below this one in two - it falls inside their paint, not outside it - and the wall
        // generator would then follow the slit instead of the model outline.
        let raw = diff_ex(&base, &union_ex(&painted));
        printable_pieces(raw, diameter)
    };
    let mut base_claimed: Vec<ExPolygons> = vec![ExPolygons::new(); layer_count];
    for &diameter in &base_nozzles {
        let band_size = adaptive_layer_band_size(diameter, reference_nozzle, base_layer_height);
        if band_size < 2 {
            continue;
        }
        let range = adaptive_layer_height_range(diameter);
        let bands = build_adaptive_layer_bands(&layer_heights, band_size, &range, &band_tops);
        for band_idx in 0..bands.len() {
            throw_on_cancel()?;
            let band = &bands[band_idx];
            let mut live = *band;
            // A band carries one contour over its whole Z range, and that contour is taken from the face
            // the band shows: the top layer for an ordinary band, the object base for the first one. The
            // first band is the bottom face of the print, so a colour painted on a vertical wall must not
            // decide its ownership - it starts above the band, exactly as a side colour ends below the
            // band carrying the top face.
            let anchored_on_base = band.bottom_idx == 0;
            if !anchored_on_base && unpainted_at(segmentation, band.top_idx, diameter).is_empty() {
                let mut has_material: Vec<u8> = vec![0; band.top_idx + 1];
                for layer_idx in band.bottom_idx..=band.top_idx {
                    has_material[layer_idx] = (!unpainted_at(segmentation, layer_idx, diameter).is_empty()) as u8;
                }
                if !pull_adaptive_band_top_to_material(&mut live, &layer_heights, &range, &has_material) {
                    continue;
                }
            }
            let anchor_idx = if anchored_on_base { 0 } else { live.top_idx };
            let retained: ExPolygons = unpainted_at(segmentation, anchor_idx, diameter);
            if retained.is_empty() {
                continue;
            }
            for layer_idx in live.bottom_idx..=live.top_idx {
                let mut area: ExPolygons = retained.clone();
                let protected_area = all_anchor_claims(layer_idx);
                if !protected_area.is_empty() {
                    area = diff_ex(&area, &protected_area);
                }
                if area.is_empty() {
                    continue;
                }
                let mut claimed_polygons = to_polygons(&base_claimed[layer_idx]);
                claimed_polygons.extend(to_polygons(&area));
                base_claimed[layer_idx] = union_ex(&claimed_polygons);
                for other_id in 0..extruder_count {
                    if !segmentation[layer_idx][other_id].is_empty() {
                        segmentation[layer_idx][other_id] =
                            printable_pieces(diff_ex(&segmentation[layer_idx][other_id], &area), extruder_nozzles[other_id]);
                    }
                }
            }
        }
    }

    // The handovers above cannot be made to agree on their own. An extruder decides what to give up from
    // the paint of its own band top, while what the receiver actually keeps is decided by the receiver's
    // band top and by every extruder that takes a share of that layer after it. Where the two disagree the
    // footprint is left owned by nobody, and nobody means the region's own filament: that is the finest
    // nozzle of the plate, the one nozzle that prints every layer, so a strip too narrow for any painted
    // nozzle still reaches the plate - as a line of the model colour drawn between two painted ones.
    //
    // Rather than chase every handover, hold the invariant that matters: paint that went into this pass
    // comes out of it painted. The colour holding most of a lost piece takes it back, which is the colour
    // that was drawn there. Pieces too small for even the reference nozzle are left alone - they never
    // reach an extruder, and re-uniting the fringe of every colour boundary would cost far more than it
    // could ever change.
    let reclaim_min_area: f64 = 0.25 * reference_nozzle * reference_nozzle;
    for layer_idx in 0..layer_count {
        throw_on_cancel()?;
        let mut before = Polygons::new();
        let mut after = Polygons::new();
        for extruder_id in 0..extruder_count {
            before.extend(to_polygons(&original_segmentation[layer_idx][extruder_id]));
            after.extend(to_polygons(&segmentation[layer_idx][extruder_id]));
        }
        if before.is_empty() {
            continue;
        }
        let mut lost_areas: ExPolygons = if after.is_empty() { union_ex(&before) } else { diff_ex(&union_ex(&before), &union_ex(&after)) };
        if !base_claimed[layer_idx].is_empty() {
            lost_areas = diff_ex(&lost_areas, &base_claimed[layer_idx]);
        }
        let mut reclaimed: Vec<Polygons> = vec![Polygons::new(); extruder_count];
        for piece in &lost_areas {
            let one: ExPolygons = vec![piece.clone()];
            let lost = area_mm2(&one);
            if lost < reclaim_min_area {
                continue;
            }
            // get_extents(piece).inflated(SCALED_EPSILON): BoundingBox.cpp `offset(coordf_t delta)` makes
            // the Point(delta, delta), which rounds.
            let mut piece_bbox = get_extents_expolygon(piece);
            piece_bbox.expand((EPSILON / SCALING_FACTOR).round_ties_even() as i64);
            let mut owner = extruder_count;
            let mut held: f64 = 0.;
            for extruder_id in 0..extruder_count {
                let paint = &original_segmentation[layer_idx][extruder_id];
                if paint.is_empty() {
                    continue;
                }
                let clip: Polygons = cu::clip_clipper_polygons_with_subject_bbox_expolygons(paint, &piece_bbox, false);
                if clip.is_empty() {
                    continue;
                }
                let painted = area_mm2(&intersection_ex_polygons(&one, &clip));
                if painted > held {
                    held = painted;
                    owner = extruder_id;
                }
            }
            if owner == extruder_count {
                continue;
            }
            reclaimed[owner].extend(to_polygons(&one));
        }
        for owner in 0..extruder_count {
            if reclaimed[owner].is_empty() {
                continue;
            }
            let mut all = std::mem::take(&mut reclaimed[owner]);
            all.extend(to_polygons(&segmentation[layer_idx][owner]));
            segmentation[layer_idx][owner] = union_ex(&all);
        }
    }

    // A coarse colour holding a footprint for fewer layers than its nozzle's minimum height cannot extrude
    // it: the band would have to reach into the colours above and below, and a fine colour there prints every
    // layer and cannot give way. Typical cases are a 0.1 mm stroke of a 0.8 colour on its band's bottom layer,
    // or a 0.8 body that 0.2 paint above and below leaves 0.1 mm of. Left alone, the merge planner has no
    // same-colour slab to fold it into, and the step stays empty across the whole footprint.
    //
    // Such a piece takes the colour standing on it one layer up, and that colour plans it like any other
    // footprint it owns: a fine nozzle prints it as sliced, a coarse one merges it with its own slab above.
    // This is what single-diameter printing does with paint that misses a layer's sampling height - the colour
    // drawn there is not the colour printed. What no other colour stands on goes to the colour below, and what
    // has neither keeps its colour. The first band of each ladder rests on the bed and keeps its own rule.
    {
        #[derive(Clone, Copy)]
        struct Owner {
            // The unpainted footprint of the regions carrying this nozzle, or a painted extruder.
            base: bool,
            extruder_id: usize,
            nozzle: f64,
            range: AdaptiveLayerHeightRange,
            // Last layer of the first band; layers up to it are left alone. -1 when this nozzle prints every
            // layer and nothing of it can be thin.
            first_band_top: i64,
        }
        let mut owners: Vec<Owner> = Vec::new();
        {
            let mut add_owner = |base: bool, extruder_id: usize, nozzle: f64| {
                let band_size = adaptive_layer_band_size(nozzle, reference_nozzle, base_layer_height);
                let range = adaptive_layer_height_range(nozzle);
                let mut first_band_top: i64 = -1;
                if band_size >= 2 {
                    let bands = build_adaptive_layer_bands(&layer_heights, band_size, &range, &band_tops);
                    first_band_top = if bands.is_empty() || bands[0].bottom_idx != 0 { 0 } else { bands[0].top_idx as i64 };
                }
                owners.push(Owner { base, extruder_id, nozzle, range, first_band_top: if band_size >= 2 { first_band_top } else { -1 } });
            };
            for extruder_id in 0..extruder_count {
                if extruder_nozzles[extruder_id] > EPSILON {
                    add_owner(false, extruder_id, extruder_nozzles[extruder_id]);
                }
            }
            for &diameter in &base_nozzles {
                add_owner(true, 0, diameter);
            }
        }

        // The unpainted footprint is recomputed only for a layer whose paint changed.
        let mut unpainted_cache: Vec<Vec<ExPolygons>> = vec![vec![ExPolygons::new(); layer_count]; owners.len()];
        let mut unpainted_valid: Vec<Vec<u8>> = vec![vec![0; layer_count]; owners.len()];
        let owned = |segmentation: &Vec<Vec<ExPolygons>>,
                     unpainted_cache: &mut Vec<Vec<ExPolygons>>,
                     unpainted_valid: &mut Vec<Vec<u8>>,
                     owner_idx: usize,
                     layer_idx: usize|
         -> ExPolygons {
            let owner = &owners[owner_idx];
            if !owner.base {
                return segmentation[layer_idx][owner.extruder_id].clone();
            }
            if unpainted_valid[owner_idx][layer_idx] == 0 {
                let mut base = base_at(layer_idx, owner.nozzle);
                if !base.is_empty() {
                    let mut painted = Polygons::new();
                    for extruder_id in 0..extruder_count {
                        painted.extend(to_polygons(&segmentation[layer_idx][extruder_id]));
                    }
                    if !painted.is_empty() {
                        base = diff_ex(&base, &union_ex(&painted));
                    }
                }
                unpainted_cache[owner_idx][layer_idx] = base;
                unpainted_valid[owner_idx][layer_idx] = 1;
            }
            unpainted_cache[owner_idx][layer_idx].clone()
        };
        let span_height = |bottom_idx: usize, top_idx: usize| -> f64 {
            let mut height: f64 = 0.;
            for layer_idx in bottom_idx..=top_idx {
                height += layer_heights[layer_idx];
            }
            height
        };

        // Bottom to top, so a piece given to the colour above already lengthens that colour's run when the
        // layers above are judged.
        for layer_idx in 0..layer_count {
            throw_on_cancel()?;
            for owner_idx in 0..owners.len() {
                let owner = owners[owner_idx];
                if owner.first_band_top < 0 || layer_idx as i64 <= owner.first_band_top {
                    continue;
                }
                let here: ExPolygons = owned(segmentation, &mut unpainted_cache, &mut unpainted_valid, owner_idx, layer_idx);
                if here.is_empty() {
                    continue;
                }

                // Whatever this colour holds through some run of layers around this one that reaches its
                // minimum height is printable. Only the shortest such runs need checking: for each bottom, the
                // lowest top that reaches the height; from the bottom where this layer alone reaches it on, a
                // lower bottom only gives a longer run.
                let mut printable = Polygons::new();
                // for (size_t bottom_idx = layer_idx + 1; bottom_idx-- > 0;)
                let mut bottom_idx = layer_idx + 1;
                while bottom_idx > 0 {
                    bottom_idx -= 1;
                    let mut top_idx = layer_idx;
                    while top_idx + 1 < layer_count && span_height(bottom_idx, top_idx) < owner.range.min_height - EPSILON {
                        top_idx += 1;
                    }
                    if span_height(bottom_idx, top_idx) >= owner.range.min_height - EPSILON {
                        let mut run: ExPolygons = here.clone();
                        let mut idx = bottom_idx;
                        while idx <= top_idx && !run.is_empty() {
                            if idx != layer_idx {
                                run = intersection_ex(&run, &owned(segmentation, &mut unpainted_cache, &mut unpainted_valid, owner_idx, idx));
                            }
                            idx += 1;
                        }
                        printable.extend(to_polygons(&run));
                    }
                    if span_height(bottom_idx, layer_idx) >= owner.range.min_height - EPSILON {
                        break;
                    }
                }
                let mut thin: ExPolygons = if printable.is_empty() { here.clone() } else { diff_ex(&here, &union_ex(&printable)) };
                // Pieces even the finest nozzle could not print - hairlines between two polygonisations - are
                // left where they are.
                thin = printable_pieces(thin, reference_nozzle);
                if thin.is_empty() {
                    continue;
                }

                // for (size_t neighbour_idx : { layer_idx + 1, layer_idx - 1 })
                for neighbour_idx in [layer_idx + 1, layer_idx.wrapping_sub(1)] {
                    if thin.is_empty() || neighbour_idx >= layer_count {
                        continue;
                    }
                    for receiver_idx in 0..owners.len() {
                        if thin.is_empty() {
                            break;
                        }
                        let receiver = owners[receiver_idx];
                        if receiver_idx == owner_idx || (receiver.base && owner.base) {
                            continue;
                        }
                        let mut taken =
                            intersection_ex(&thin, &owned(segmentation, &mut unpainted_cache, &mut unpainted_valid, receiver_idx, neighbour_idx));
                        // Removing the paint hands the footprint to whichever region holds it on this layer, which
                        // is the receiver only where that region is there too: two parts may meet at this layer.
                        if receiver.base && !taken.is_empty() {
                            taken = intersection_ex(&taken, &base_at(layer_idx, receiver.nozzle));
                        }
                        if taken.is_empty() {
                            continue;
                        }
                        // Painted area leaves its extruder; the unpainted footprint is whatever no paint covers,
                        // so it is given up by painting it and taken back by removing the paint.
                        if !owner.base {
                            segmentation[layer_idx][owner.extruder_id] = diff_ex(&segmentation[layer_idx][owner.extruder_id], &taken);
                        }
                        if !receiver.base {
                            let mut combined = to_polygons(&segmentation[layer_idx][receiver.extruder_id]);
                            combined.extend(to_polygons(&taken));
                            segmentation[layer_idx][receiver.extruder_id] = union_ex(&combined);
                        }
                        // paint_changed(layer_idx)
                        for valid in unpainted_valid.iter_mut() {
                            valid[layer_idx] = 0;
                        }
                        thin = diff_ex(&thin, &taken);
                    }
                }
            }
        }
    }
    Ok(())
}
