//! Support material generation.
//!
//! C++ Reference:
//! - Support/SupportMaterial.hpp
//! - Support/SupportMaterial.cpp
//!
//! Port of `Support/SupportMaterial.{hpp,cpp}` at the pin (line numbers are the pin's).
//!
//! - The constants, the allocator helpers, the slice collectors and `merge_contact_layers` are at the top of
//!   the file.
//! - `material`: `PrintObjectSupportMaterial` and its stages (`generate`, `buildplate_covered`,
//!   `top_contact_layers`, `bottom_contact_layers_and_layer_support_areas`,
//!   `trim_top_contacts_by_bottom_contacts`, `raft_and_intermediate_support_layers`, `generate_base_layers`,
//!   `trim_support_layers_by_object`), the namespace `SupportMaterialInternal` and the free helpers
//!   (`detect_overhangs`, `detect_contacts`, `sync_gap_with_object_layer`, `new_contact_layer`,
//!   `fill_contact_layer`, `OverhangCluster`, `add_overhang`, `detect_bottom_contacts`,
//!   `project_support_to_grid`), over the Print classes of `crate::print::faithful`.
//! - `grid`: the rasterizer, `contours_simplified`, `SupportGridParams` and `SupportGridPattern`
//!   (`#define SUPPORT_USE_AGG_RASTERIZER`, over the crate's port of AGG).
//! - `debug`: the SVG exports of `#ifdef SLIC3R_DEBUG`, which the engine does not compile.
//!
//! Compared with the pin by tests/support_material_native_parity.rs (tests/data/support_material/gen.cpp).
//! `SupportGridPattern::serialize`, `deserialize` and `deserialize_` exist only with `SLIC3R_DEBUG` and
//! without `SUPPORT_USE_AGG_RASTERIZER` (they read and write the EdgeGrid branch of the class): the engine
//! does not compile them; they are ported and round-trip in tests/support_material_native_parity.rs, not
//! compared with the pin.
//!
//! coord_t -> i64, coordf_t -> f64 per crate conventions.

use crate::geometry::Polygons;
use crate::libslic3r::EPSILON;
use crate::slicing::SlicingParams as SlicingParameters;
use crate::support::support_layer::{
    SupporLayerType, SupportGeneratorLayer, SupportGeneratorLayerStorage, SupportGeneratorLayersPtr,
};
use crate::surface::SurfaceType;

// SupportMaterial.cpp:56-59
// how much we extend support around the actual contact area
//FIXME this should be dependent on the nozzle diameter!
// BBS: change from 1.5 to 1.2
pub const SUPPORT_MATERIAL_MARGIN: f64 = 1.2;

// SupportMaterial.cpp:61-62
// Increment used to reach MARGIN in steps to avoid trespassing thin objects
pub const NUM_MARGIN_STEPS: usize = 3;

// SupportMaterial.cpp:64-66
// Dimensions of a tree-like structure to save material
pub const PILLAR_SIZE: f64 = 2.5;
pub const PILLAR_SPACING: f64 = 10.0;

// SupportMaterial.cpp:68-70
//#define SUPPORT_SURFACES_OFFSET_PARAMETERS ClipperLib::jtMiter, 3.
//#define SUPPORT_SURFACES_OFFSET_PARAMETERS ClipperLib::jtMiter, 1.5
// #define SUPPORT_SURFACES_OFFSET_PARAMETERS ClipperLib::jtSquare, 0.
// NOTE: in C++ this is a token macro expanding to two ClipperLib args
// (ClipperLib::jtSquare, 0.). Represented at call sites in the crate by
// OffsetJoinType::Square with a 0. miter limit.

// SupportMaterial.cpp:72
pub const SUPPORT_WITH_SHEATH: bool = false;

// ============================================================================
// SupportMaterial.cpp:344-365 — deque allocator helpers.
//
// In C++ these `layer_allocate` overloads push a fresh `SupportGeneratorLayer`
// into a `std::deque` (used as a stable-address allocator) and return a
// reference to it. The Rust port routes this through
// `SupportGeneratorLayerStorage`, which models the deque + the `tbb::spin_mutex`
// guarded variant (single-threaded here, so both overloads are equivalent).
// ============================================================================

// SupportMaterial.cpp:344-352
// inline SupportGeneratorLayer& layer_allocate(
//     std::deque<SupportGeneratorLayer> &layer_storage,
//     SupporLayerType      layer_type)
// {
//     layer_storage.push_back(SupportGeneratorLayer());
//     layer_storage.back().layer_type = layer_type;
//     return layer_storage.back();
// }
pub fn layer_allocate(
    layer_storage: &mut SupportGeneratorLayerStorage,
    layer_type: SupporLayerType,
) -> &mut SupportGeneratorLayer {
    layer_storage.allocate_unguarded(layer_type)
}

// SupportMaterial.cpp:354-365
// inline SupportGeneratorLayer& layer_allocate(
//     std::deque<SupportGeneratorLayer> &layer_storage,
//     tbb::spin_mutex                                 &layer_storage_mutex,
//     SupporLayerType      layer_type)
// {
//     layer_storage_mutex.lock();
//     layer_storage.push_back(SupportGeneratorLayer());
//     SupportGeneratorLayer *layer_new = &layer_storage.back();
//     layer_storage_mutex.unlock();
//     layer_new->layer_type = layer_type;
//     return *layer_new;
// }
//
// The `tbb::spin_mutex` is not portable and is unnecessary in the single-threaded
// port; `allocate()` mirrors the guarded variant.
pub fn layer_allocate_guarded(
    layer_storage: &mut SupportGeneratorLayerStorage,
    layer_type: SupporLayerType,
) -> &mut SupportGeneratorLayer {
    layer_storage.allocate(layer_type)
}

// SupportMaterial.cpp:367-370
// inline void layers_append(SupportGeneratorLayersPtr &dst, const SupportGeneratorLayersPtr &src)
// {
//     dst.insert(dst.end(), src.begin(), src.end());
// }
pub fn layers_append(dst: &mut SupportGeneratorLayersPtr, src: &SupportGeneratorLayersPtr) {
    dst.extend_from_slice(src);
}

// SupportMaterial.cpp:372-375
// Support layer that is covered by some form of dense interface.
// static constexpr const std::initializer_list<SupporLayerType> support_types_interface {
//     SupporLayerType::sltRaftInterface, SupporLayerType::sltBottomContact,
//     SupporLayerType::sltBottomInterface, SupporLayerType::sltTopContact,
//     SupporLayerType::sltTopInterface
// };
pub const SUPPORT_TYPES_INTERFACE: [SupporLayerType; 5] = [
    SupporLayerType::SltRaftInterface,
    SupporLayerType::SltBottomContact,
    SupporLayerType::SltBottomInterface,
    SupporLayerType::SltTopContact,
    SupporLayerType::SltTopInterface,
];

// ============================================================================
// SupportMaterial.cpp:634-662 — Layer slice collectors.
// ============================================================================

// SupportMaterial.cpp:634-651
// Collect all polygons of all regions in a layer with a given surface type.
// Polygons collect_region_slices_by_type(const Layer &layer, SurfaceType surface_type)
pub fn collect_region_slices_by_type(
    layer: &crate::layer::Layer,
    surface_type: SurfaceType,
) -> Polygons {
    // SupportMaterial.cpp:637-642
    // 1) Count the new polygons first.
    let mut n_polygons_new: usize = 0;
    for region in layer.regions() {
        for surface in &region.slices.surfaces {
            if surface.surface_type == surface_type {
                n_polygons_new += surface.expolygon.holes.len() + 1;
            }
        }
    }
    // SupportMaterial.cpp:643-649
    // 2) Collect the new polygons.
    let mut out: Polygons = Polygons::new();
    out.reserve(n_polygons_new);
    for region in layer.regions() {
        for surface in &region.slices.surfaces {
            if surface.surface_type == surface_type {
                crate::geometry::polygons_append_expoly(&mut out, &surface.expolygon);
            }
        }
    }
    out
}

// SupportMaterial.cpp:653-662
// Collect outer contours of all slices of this layer.
// This is useful for calculating the support base with holes filled.
// Polygons collect_slices_outer(const Layer &layer)
pub fn collect_slices_outer(layer: &crate::layer::Layer) -> Polygons {
    let mut out: Polygons = Polygons::new();
    // SupportMaterial.cpp:657
    // out.reserve(out.size() + layer.lslices.size());
    out.reserve(out.len() + layer.lslices.len());
    // SupportMaterial.cpp:658-660
    // for (const ExPolygon &expoly : layer.lslices)
    //     out.emplace_back(expoly.contour);
    for expoly in &layer.lslices {
        out.push(expoly.contour.clone());
    }
    out
}

// ============================================================================
// SupportMaterial.cpp:1397-1402 — BBS well-supported / sharp-tail constants.
// ============================================================================

// SupportMaterial.cpp:1398
// static const double length_thresh_well_supported = scale_(6);  // min: 6mm
pub fn length_thresh_well_supported() -> f64 {
    // libslic3r.h:81 `scale_(val)`: `val / SCALING_FACTOR`
    6. / crate::libslic3r::SCALING_FACTOR
}
// SupportMaterial.cpp:1399
// static const double area_thresh_well_supported = SQ(length_thresh_well_supported);  // min: 6x6=36mm^2
pub fn area_thresh_well_supported() -> f64 {
    let l = length_thresh_well_supported();
    l * l
}
// SupportMaterial.cpp:1400
// static const double sharp_tail_xy_gap = 0.2f;
pub const SHARP_TAIL_XY_GAP: f64 = 0.2;
// SupportMaterial.cpp:1401
// static const double no_overlap_xy_gap = 0.2f;
pub const NO_OVERLAP_XY_GAP: f64 = 0.2;
// SupportMaterial.cpp:1402
// static const double sharp_tail_max_support_height = 16.f;
pub const SHARP_TAIL_MAX_SUPPORT_HEIGHT: f64 = 16.0;

// ============================================================================
// SupportMaterial.cpp:2038-2081 — merge_contact_layers.
//
// Merge close contact layers conservatively: If two layers are closer than the
// minimum allowed print layer height (the min_layer_height parameter), the top
// contact layer is merged into the bottom contact layer.
//
// Operates purely on `SupportGeneratorLayer` + `SlicingParameters`; the
// `SupportGeneratorLayersPtr` is a `Vec<usize>` of indices into `layer_storage`,
// so the function takes the storage to resolve the "pointers".
// ============================================================================

// SupportMaterial.cpp:2040
// static void merge_contact_layers(const SlicingParameters &slicing_params, double support_layer_height_min, SupportGeneratorLayersPtr &layers)
#[allow(unused_assignments)] // C++ `i = j;` is the last statement of the for-loop body (SupportMaterial.cpp:2077); kept verbatim.
pub fn merge_contact_layers(
    slicing_params: &SlicingParameters,
    support_layer_height_min: f64,
    layers: &mut SupportGeneratorLayersPtr,
    layer_storage: &mut SupportGeneratorLayerStorage,
) {
    // SupportMaterial.cpp:2042-2043
    // Sort the layers, as one layer may produce bridging and non-bridging contact layers with different print_z.
    // std::sort(layers.begin(), layers.end(), [](const SupportGeneratorLayer *l1, const SupportGeneratorLayer *l2) { return l1->print_z < l2->print_z; });
    // (libc++'s std::sort: layers of one print_z stay in the order it leaves them)
    crate::clipper::libcxx_sort(layers, |&l1, &l2| layer_storage[l1].print_z < layer_storage[l2].print_z);

    // SupportMaterial.cpp:2045-2046
    let mut i: i32 = 0;
    let mut k: i32 = 0;
    {
        // SupportMaterial.cpp:2048-2050
        // Find the span of layers, which are to be printed at the first layer height.
        let mut j: i32 = 0;
        while (j as usize) < layers.len()
            && layer_storage[layers[j as usize]].print_z
                < slicing_params.first_print_layer_height + support_layer_height_min - EPSILON
        {
            j += 1;
        }
        // SupportMaterial.cpp:2051
        if j > 0 {
            // SupportMaterial.cpp:2053-2055
            // Merge the layers layers (0) to (j - 1) into the layers[0].
            // SupportGeneratorLayer &dst = *layers.front();
            // for (int u = 1; u < j; ++ u)
            //     dst.merge(std::move(*layers[u]));
            // (`SupportGeneratorLayer::merge` on ClipperLib, SupportLayer.hpp:75-88)
            let dst_idx = layers[0];
            for u in 1..j {
                crate::support::support_common::support_generator_layer_merge(layer_storage, dst_idx, layers[u as usize]);
            }
            // SupportMaterial.cpp:2057-2059
            // Snap the first layer to the 1st layer height.
            // dst.print_z  = slicing_params.first_print_layer_height;
            // dst.height   = slicing_params.first_print_layer_height;
            // dst.bottom_z = 0;
            {
                let dst = &mut layer_storage[dst_idx];
                dst.print_z = slicing_params.first_print_layer_height;
                dst.height = slicing_params.first_print_layer_height;
                dst.bottom_z = 0.0;
            }
            // SupportMaterial.cpp:2060
            k += 1;
        }
        // SupportMaterial.cpp:2062
        i = j;
    }
    // SupportMaterial.cpp:2064
    while i < layers.len() as i32 {
        // SupportMaterial.cpp:2066-2068
        // Find the span of layers closer than m_support_layer_height_min.
        let mut j: i32 = i + 1;
        let zmax = layer_storage[layers[i as usize]].print_z + support_layer_height_min + EPSILON;
        while (j as usize) < layers.len() && layer_storage[layers[j as usize]].print_z < zmax {
            j += 1;
        }
        // SupportMaterial.cpp:2069
        if i + 1 < j {
            // SupportMaterial.cpp:2071-2073
            // Merge the layers layers (i + 1) to (j - 1) into the layers[i].
            // SupportGeneratorLayer &dst = *layers[i];
            // for (int u = i + 1; u < j; ++ u)
            //     dst.merge(std::move(*layers[u]));
            let dst_idx = layers[i as usize];
            for u in (i + 1)..j {
                crate::support::support_common::support_generator_layer_merge(layer_storage, dst_idx, layers[u as usize]);
            }
        }
        // SupportMaterial.cpp:2075-2076
        if k < i {
            layers[k as usize] = layers[i as usize];
        }
        // SupportMaterial.cpp:2077
        i = j;
        // SupportMaterial.cpp:2064 (++ k)
        k += 1;
    }
    // SupportMaterial.cpp:2079-2080
    if (k as usize) < layers.len() {
        // layers.erase(layers.begin() + k, layers.end());
        layers.truncate(k as usize);
    }
}

// SupportMaterial.hpp:18-21
// inline double layer_z(const SlicingParameters& slicing_params, const size_t layer_idx)
// {
//     return slicing_params.object_print_z_min + slicing_params.first_object_layer_height + layer_idx * slicing_params.layer_height;
// }
pub fn layer_z(slicing_params: &SlicingParameters, layer_idx: usize) -> f64 {
    slicing_params.object_print_z_min
        + slicing_params.first_object_layer_height
        + layer_idx as f64 * slicing_params.layer_height
}

// SupportMaterial.hpp:23-34
// inline SupportGeneratorLayer& layer_initialize(
//     SupportGeneratorLayer& layer_new,
//     const SupporLayerType    layer_type,
//     const SlicingParameters& slicing_params,
//     const size_t             layer_idx)
// {
//     layer_new.layer_type = layer_type;
//     layer_new.print_z = layer_z(slicing_params, layer_idx);
//     layer_new.height = layer_idx == 0 ? slicing_params.first_object_layer_height : slicing_params.layer_height;
//     layer_new.bottom_z = layer_idx == 0 ? slicing_params.object_print_z_min : layer_new.print_z - layer_new.height;
//     return layer_new;
// }
pub fn layer_initialize(
    layer_new: &mut SupportGeneratorLayer,
    layer_type: SupporLayerType,
    slicing_params: &SlicingParameters,
    layer_idx: usize,
) {
    // SupportMaterial.hpp:29
    layer_new.layer_type = layer_type;
    // SupportMaterial.hpp:30
    layer_new.print_z = layer_z(slicing_params, layer_idx);
    // SupportMaterial.hpp:31
    layer_new.height = if layer_idx == 0 {
        slicing_params.first_object_layer_height
    } else {
        slicing_params.layer_height
    };
    // SupportMaterial.hpp:32
    layer_new.bottom_z = if layer_idx == 0 {
        slicing_params.object_print_z_min
    } else {
        layer_new.print_z - layer_new.height
    };
}

// SupportMaterial.hpp:36-46
// Using the std::deque as an allocator.
// inline SupportGeneratorLayer& layer_allocate(
//     std::deque<SupportGeneratorLayer>& layer_storage,
//     SupporLayerType                    layer_type,
//     const SlicingParameters& slicing_params,
//     size_t                             layer_idx)
// {
//     //FIXME take raft into account.
//     layer_storage.push_back(SupportGeneratorLayer());
//     return layer_initialize(layer_storage.back(), layer_type, slicing_params, layer_idx);
// }
//
// Returns the index ("pointer") of the freshly-allocated layer in `layer_storage`.
pub fn layer_allocate_initialized(
    layer_storage: &mut SupportGeneratorLayerStorage,
    layer_type: SupporLayerType,
    slicing_params: &SlicingParameters,
    layer_idx: usize,
) -> usize {
    //FIXME take raft into account.
    let idx = layer_storage.len();
    let layer = layer_storage.allocate_unguarded(layer_type);
    layer_initialize(layer, layer_type, slicing_params, layer_idx);
    idx
}

// ============================================================================
// SupportMaterial.cpp:334-632, 1173-3263 at the pin, over the Print classes of `crate::print::faithful`:
// PrintObjectSupportMaterial and its stages, the namespace SupportMaterialInternal and the free helpers.
//
// A `SupportGeneratorLayer*` is the index of the layer in the `SupportGeneratorLayerStorage`, a `Layer*`
// the index of the layer in `PrintObject::layers()` (`Layer::lower_layer_id` / `upper_layer_id`).
// `Layer::sharp_tails`, `Layer::sharp_tails_height` and `Layer::cantilevers` (Layer.hpp:172-174, `mutable`,
// written by detect_overhangs() and top_contact_layers()) are kept by PrintObjectSupportMaterial, per
// object layer: the crate's Layer declares them as `Polygons` and one `f64`.
// ============================================================================

pub use debug::{
    export_print_z_polygons_and_extrusions_to_svg, export_print_z_polygons_to_svg, export_support_surface_type_legend_to_svg,
    export_support_surface_type_legend_to_svg_box_size, support_surface_type_to_color_name,
};
pub use material::*;

/// SupportMaterial.cpp:74-169, the SVG exports of `#ifdef SLIC3R_DEBUG`: the engine does not compile them, so
/// they are ported and not compared.
pub mod debug {
    use crate::extrusion_entity::{ExtrusionEntityCollection, ExtrusionEntityType};
    use crate::geometry::{Point, Polygons};
    use crate::libslic3r::SCALING_FACTOR;
    use crate::print::faithful as pf;
    use crate::support::support_common::{cu, cu_polygon, get_extents_polygons_bbox, BBox};
    use crate::support::support_layer::{SupporLayerType, SupportGeneratorLayerStorage};
    use crate::svg::SVG;

    /// libslic3r.h:81 `#define scale_(val) ((val) / SCALING_FACTOR)`
    #[inline]
    fn scale_(val: f64) -> f64 {
        val / SCALING_FACTOR
    }

    /// SupportMaterial.cpp:75-88
    /// C++: `const char* support_surface_type_to_color_name(const SupporLayerType surface_type)`
    pub fn support_surface_type_to_color_name(surface_type: SupporLayerType) -> &'static str {
        match surface_type {
            SupporLayerType::SltTopContact => "rgb(255,0,0)",          // "red";
            SupporLayerType::SltTopInterface => "rgb(0,255,0)",        // "green";
            SupporLayerType::SltBase => "rgb(0,0,255)",                // "blue";
            SupporLayerType::SltBottomInterface => "rgb(255,255,128)", // yellow
            SupporLayerType::SltBottomContact => "rgb(255,0,255)",     // magenta
            SupporLayerType::SltRaftInterface => "rgb(0,255,255)",
            SupporLayerType::SltRaftBase => "rgb(128,128,128)",
            SupporLayerType::SltUnknown => "rgb(128,0,0)", // maroon
            _ => "rgb(64,64,64)",
        }
    }

    /// SupportMaterial.cpp:90-93
    /// C++: `Point export_support_surface_type_legend_to_svg_box_size()`
    pub fn export_support_surface_type_legend_to_svg_box_size() -> Point {
        // `Point(double, double)` rounds (Point.hpp:181)
        Point::new(scale_(1. + 10. * 8.).round_ties_even() as i64, scale_(3.).round_ties_even() as i64)
    }

    /// SupportMaterial.cpp:95-121
    /// C++: `void export_support_surface_type_legend_to_svg(SVG &svg, const Point &pos)`
    pub fn export_support_surface_type_legend_to_svg(svg: &mut SVG, pos: &Point) {
        // 1st row
        // (`coord_t = coord_t + double`: the sum truncated)
        let pos_x0: i64 = (pos.x as f64 + scale_(1.)) as i64;
        let mut pos_x: i64 = pos_x0;
        let mut pos_y: i64 = (pos.y as f64 + scale_(1.5)) as i64;
        let step_x: i64 = scale_(10.) as i64;
        svg.draw_legend(&Point::new(pos_x, pos_y), "top contact", support_surface_type_to_color_name(SupporLayerType::SltTopContact));
        pos_x += step_x;
        svg.draw_legend(&Point::new(pos_x, pos_y), "top iface", support_surface_type_to_color_name(SupporLayerType::SltTopInterface));
        pos_x += step_x;
        svg.draw_legend(&Point::new(pos_x, pos_y), "base", support_surface_type_to_color_name(SupporLayerType::SltBase));
        pos_x += step_x;
        svg.draw_legend(&Point::new(pos_x, pos_y), "bottom iface", support_surface_type_to_color_name(SupporLayerType::SltBottomInterface));
        pos_x += step_x;
        svg.draw_legend(&Point::new(pos_x, pos_y), "bottom contact", support_surface_type_to_color_name(SupporLayerType::SltBottomContact));
        // 2nd row
        pos_x = pos_x0;
        pos_y = (pos.y as f64 + scale_(2.8)) as i64;
        svg.draw_legend(&Point::new(pos_x, pos_y), "raft interface", support_surface_type_to_color_name(SupporLayerType::SltRaftInterface));
        pos_x += step_x;
        svg.draw_legend(&Point::new(pos_x, pos_y), "raft base", support_surface_type_to_color_name(SupporLayerType::SltRaftBase));
        pos_x += step_x;
        svg.draw_legend(&Point::new(pos_x, pos_y), "unknown", support_surface_type_to_color_name(SupporLayerType::SltUnknown));
        pos_x += step_x;
        svg.draw_legend(&Point::new(pos_x, pos_y), "intermediate", support_surface_type_to_color_name(SupporLayerType::SltIntermediate));
    }

    /// The box of the layers with the legend above it (SupportMaterial.cpp:125-130, 147-152), and where the legend goes.
    fn bbox_with_legend(layers: &[usize], layer_storage: &SupportGeneratorLayerStorage) -> (BBox, Point) {
        let mut bbox = BBox::new();
        for &layer in layers {
            bbox.merge(&get_extents_polygons_bbox(&layer_storage[layer].polygons));
        }
        let legend_size = export_support_surface_type_legend_to_svg_box_size();
        let legend_pos = Point::new(bbox.min.x, bbox.max.y);
        // bbox.merge(Point(std::max(bbox.min(0) + legend_size(0), bbox.max(0)), bbox.max(1) + legend_size(1)));
        // (BoundingBox.cpp `merge(const PointClass &point)`)
        let point = Point::new((bbox.min.x + legend_size.x).max(bbox.max.x), bbox.max.y + legend_size.y);
        if bbox.defined {
            bbox.min = Point::new(bbox.min.x.min(point.x), bbox.min.y.min(point.y));
            bbox.max = Point::new(bbox.max.x.max(point.x), bbox.max.y.max(point.y));
        } else {
            bbox.min = point;
            bbox.max = point;
            bbox.defined = true;
        }
        (bbox, legend_pos)
    }

    /// The layers drawn (SupportMaterial.cpp:132-136, 154-158).
    fn draw_layers(svg: &mut SVG, layers: &[usize], layer_storage: &SupportGeneratorLayerStorage) {
        let transparency: f32 = 0.5;
        for &layer in layers {
            let l = &layer_storage[layer];
            svg.draw_expolygons(&cu::union_ex(&l.polygons), support_surface_type_to_color_name(l.layer_type), transparency);
        }
        for &layer in layers {
            let l = &layer_storage[layer];
            svg.draw_polylines(&cu_polygon::to_polylines(&l.polygons), support_surface_type_to_color_name(l.layer_type), 0.);
        }
    }

    /// SupportMaterial.cpp:123-139
    /// C++: `void export_print_z_polygons_to_svg(const char *path, SupportGeneratorLayer ** const layers, size_t n_layers)`
    pub fn export_print_z_polygons_to_svg(path: &str, layers: &[usize], layer_storage: &SupportGeneratorLayerStorage) {
        let (bbox, legend_pos) = bbox_with_legend(layers, layer_storage);
        let mut svg = SVG::new_bbox_default(path, &bbox.to_bounding_box());
        draw_layers(&mut svg, layers, layer_storage);
        export_support_surface_type_legend_to_svg(&mut svg, &legend_pos);
        svg.close();
    }

    /// ExtrusionEntityCollection.hpp `polygons_covered_by_width(Polygons &out, const float scaled_epsilon)`: of
    /// every entity of the collection.
    fn collection_polygons_covered_by_width(collection: &ExtrusionEntityCollection, out: &mut Polygons, scaled_epsilon: f32) {
        for entity in &collection.entities {
            match entity {
                ExtrusionEntityType::Path(path) => path.polygons_covered_by_width(out, scaled_epsilon),
                ExtrusionEntityType::MultiPath(multipath) => multipath.polygons_covered_by_width(out, scaled_epsilon),
                ExtrusionEntityType::Loop(loop_) => loop_.polygons_covered_by_width(out, scaled_epsilon),
                ExtrusionEntityType::Collection(c) => collection_polygons_covered_by_width(c, out, scaled_epsilon),
            }
        }
    }

    /// SupportMaterial.cpp:141-168
    /// C++: `void export_print_z_polygons_and_extrusions_to_svg(const char *path, SupportGeneratorLayer ** const layers,
    /// size_t n_layers, SupportLayer &support_layer)`
    pub fn export_print_z_polygons_and_extrusions_to_svg(path: &str, layers: &[usize], layer_storage: &SupportGeneratorLayerStorage, support_layer: &pf::SupportLayer) {
        let (bbox, legend_pos) = bbox_with_legend(layers, layer_storage);
        let mut svg = SVG::new_bbox_default(path, &bbox.to_bounding_box());
        draw_layers(&mut svg, layers, layer_storage);

        let mut polygons_support: Polygons = Vec::new();
        let polygons_interface: Polygons = Vec::new();
        // support_layer.support_fills.polygons_covered_by_width(polygons_support, float(SCALED_EPSILON));
        collection_polygons_covered_by_width(&support_layer.support_fills, &mut polygons_support, scale_(crate::libslic3r::EPSILON) as f32);
        //    support_layer.support_interface_fills.polygons_covered_by_width(polygons_interface, SCALED_EPSILON);
        svg.draw_expolygons(&cu::union_ex(&polygons_support), "brown", 1.);
        svg.draw_expolygons(&cu::union_ex(&polygons_interface), "black", 1.);

        export_support_surface_type_legend_to_svg(&mut svg, &legend_pos);
        svg.close();
    }
}

/// SupportMaterial.cpp:334-632, 1173-3263
pub mod material {
    use super::grid::{SupportGridParams, SupportGridPattern, SMS_SNUG};
    use super::{merge_contact_layers, SUPPORT_MATERIAL_MARGIN};
    use crate::clipper::JoinType;
    use crate::config::{ConfigOptionFloat, ConfigOptionResolver};
    use crate::extrusion_entity::{ExtrusionEntityCollection, ExtrusionEntityType, ExtrusionLoop, ExtrusionPath, ExtrusionRole};
    use crate::flow::{Flow, FlowRole};
    use crate::geometry::{ExPolygon, ExPolygons, Point, Polygon, Polygons, Polyline, Polylines};
    use crate::layer::{Layer, LayerRegion};
    use crate::libslic3r::{EPSILON, SCALING_FACTOR};
    use crate::print::faithful as pf;
    use crate::print_config::config_def::DynamicPrintConfig;
    use crate::print_config::enums::SupportType;
    use crate::slicing::SlicingParams as SlicingParameters;
    use crate::support::support_common::{cu, cu_expolygon, safe_offset_inc, BBox, NULL_LAYER};
    use crate::support::support_layer::{SupporLayerType, SupportGeneratorLayerStorage, SupportGeneratorLayersPtr};
    use crate::support::support_parameters::{flow_new_from_config_width, support_nozzle_diameter, SupportParameters};
    use crate::support::tree_support::CustomFacetsProjector;
    use crate::surface::SurfaceType;
    use cu::ApplySafetyOffset;
    use std::collections::{BTreeMap, BTreeSet};

    /// SupportMaterial.cpp:70 `#define SUPPORT_SURFACES_OFFSET_PARAMETERS ClipperLib::jtSquare, 0.`
    const SSOP_JT: JoinType = JoinType::jtSquare;
    const SSOP_ML: f64 = 0.;

    /// libslic3r.h:91 `static constexpr bool g_config_support_sharp_tails = true;`
    const G_CONFIG_SUPPORT_SHARP_TAILS: bool = true;

    /// libslic3r.h:81 `#define scale_(val) ((val) / SCALING_FACTOR)`
    #[inline]
    fn scale_(val: f64) -> f64 {
        val / SCALING_FACTOR
    }
    /// libslic3r.h `SCALED_EPSILON`: `scale_(EPSILON)`
    #[inline]
    fn scaled_epsilon() -> f64 {
        scale_(EPSILON)
    }

    // SupportMaterial.cpp:1398-1402
    /// `static const double length_thresh_well_supported = scale_(6);`  // min: 6mm
    #[inline]
    fn length_thresh_well_supported() -> f64 {
        scale_(6.)
    }
    /// `static const double area_thresh_well_supported = SQ(length_thresh_well_supported);`  // min: 6x6=36mm^2
    #[inline]
    fn area_thresh_well_supported() -> f64 {
        length_thresh_well_supported() * length_thresh_well_supported()
    }
    /// `static const double sharp_tail_xy_gap = 0.2f;`
    const SHARP_TAIL_XY_GAP: f64 = 0.2f32 as f64;
    /// `static const double no_overlap_xy_gap = 0.2f;`
    const NO_OVERLAP_XY_GAP: f64 = 0.2f32 as f64;
    /// `static const double sharp_tail_max_support_height = 16.f;`
    const SHARP_TAIL_MAX_SUPPORT_HEIGHT: f64 = 16.;

    fn cfg_float(config: &DynamicPrintConfig, key: &str) -> f64 {
        config.option(key).and_then(|o| o.get_float().ok()).unwrap_or(0.0)
    }
    fn cfg_int(config: &DynamicPrintConfig, key: &str) -> i32 {
        config.option(key).and_then(|o| o.get_int().ok()).unwrap_or(0)
    }
    fn cfg_bool(config: &DynamicPrintConfig, key: &str) -> bool {
        config.option(key).and_then(|o| o.get_bool().ok()).unwrap_or(false)
    }

    /// The overloads of ClipperUtils.cpp that `support_common::cu` does not have.
    mod cx {
        use crate::clipper::{ClipType, IntPoint, JoinType, Path, Paths, PolyFillType};
        use crate::clipper_utils as u;
        use crate::geometry::{ExPolygon, ExPolygons, Point, Polygon, Polygons};
        use crate::support::support_common::cu;

        fn path_of(points: &[Point]) -> Path {
            points.iter().map(|p| IntPoint::new(p.x as _, p.y as _)).collect()
        }
        fn expolygon_paths(expolygons: &[ExPolygon]) -> Vec<(Path, Paths)> {
            expolygons.iter().map(|e| (path_of(&e.contour.points), e.holes.iter().map(|h| path_of(&h.points)).collect())).collect()
        }
        /// ClipperUtils.cpp:577-580 `Polygons offset2(const ExPolygons &, float delta1, float delta2, JoinType, double miterLimit)`
        pub fn offset2(expolygons: &[ExPolygon], delta1: f32, delta2: f32, join_type: JoinType, miter_limit: f64) -> Polygons {
            cu::to_polygons(&u::offset_paths::<Paths>(
                &u::expolygons_offset(&expolygon_paths(expolygons), delta1 as f64, join_type, miter_limit),
                delta2 as f64,
                join_type,
                miter_limit,
            ))
        }
        /// ClipperUtils.cpp:705-706 `Polygons intersection(const ExPolygons &, const Polygons &, ApplySafetyOffset)`
        pub fn intersection_ep(subject: &[ExPolygon], clip: &[Polygon]) -> Polygons {
            cu::clipper_paths(ClipType::ctIntersection, &cu::expolygons_provider(subject), &cu::polygons_provider(clip), cu::ApplySafetyOffset::No)
        }
        /// ClipperUtils.cpp:798-803 `ExPolygons intersection_ex(const ExPolygon(s) &, const ExPolygons &, ApplySafetyOffset)`
        pub fn intersection_ex_ee(subject: &[ExPolygon], clip: &[ExPolygon]) -> ExPolygons {
            u::_clipper_ex(
                ClipType::ctIntersection,
                &cu::expolygons_provider(subject),
                &cu::expolygons_provider(clip),
                cu::ApplySafetyOffset::No,
                PolyFillType::pftNonZero,
            )
        }
    }

    /// ExPolygon.hpp:295-309 `Polygons to_polygons(const ExPolygons &src)`: of every expolygon its contour
    /// (an empty one too), then its holes.
    fn to_polygons(src: &[ExPolygon]) -> Polygons {
        let mut polygons: Polygons = Vec::with_capacity(src.iter().map(|e| e.holes.len() + 1).sum());
        for e in src {
            polygons.push(e.contour.clone());
            polygons.extend(e.holes.iter().cloned());
        }
        polygons
    }

    /// ExPolygon.cpp:495-504 `BoundingBox get_extents(const ExPolygons &expolygons)`
    fn get_extents_expolygons(expolygons: &[ExPolygon]) -> BBox {
        let mut bbox = BBox::new();
        for expolygon in expolygons {
            if !expolygon.contour.points.is_empty() {
                bbox.merge(&BBox::from_points(&expolygon.contour.points));
            }
        }
        bbox
    }
    /// BoundingBox.hpp `PointClass size() const`: `max - min`
    fn bbox_size(bbox: &BBox) -> Point {
        Point::new(bbox.max.x - bbox.min.x, bbox.max.y - bbox.min.y)
    }

    /// MultiPoint.hpp:53-74 `double MultiPoint::distance_to(const Point &point) const`: the distance of the
    /// closest point of the contour (`closest_point_index`), not of its closest edge.
    fn multipoint_distance_to(points: &[Point], point: &Point) -> f64 {
        // `(a - b).cast<double>().norm()`: the squares are summed by Eigen's reduction, nothing is contracted.
        let norm = |a: &Point, b: &Point| -> f64 {
            let x = (a.x - b.x) as f64;
            let y = (a.y - b.y) as f64;
            (x * x + y * y).sqrt()
        };
        let mut idx = 0usize;
        let mut dist_min = norm(point, &points[0]);
        for (i, p) in points.iter().enumerate().skip(1) {
            let d = norm(p, point);
            if d < dist_min {
                dist_min = d;
                idx = i;
            }
        }
        norm(&points[idx], point)
    }

    /// PrintRegion.cpp:21-58 `Flow PrintRegion::flow(const PrintObject &object, FlowRole role, double layer_height,
    /// bool first_layer) const` over the faithful classes (the object reaches its Print through `m_print`).
    pub fn print_region_flow(
        region: &pf::PrintRegion,
        object: &pf::PrintObject,
        print: &pf::Print,
        role: FlowRole,
        layer_height: f64,
        first_layer: bool,
    ) -> crate::Result<Flow> {
        use crate::print_config::config_def::{multi_nozzle_diameter_mode_enabled, primary_nozzle_diameter};
        let print_config = print.config();
        let m_config = region.config();
        // Get extrusion width from configuration.
        // (an absolute value authored for the process preset, or zero for auto)
        // The infill of the initial layer has its own line width, which takes over the responsibility of
        // "initial_layer_line_width" for the infill roles only. Zero means to keep following "initial_layer_line_width".
        let infill_role = matches!(role, FlowRole::Infill | FlowRole::SolidInfill | FlowRole::TopSolidInfill);
        let mut config_width: f64 = if first_layer && infill_role && cfg_float(print_config, "initial_layer_infill_line_width") > 0. {
            cfg_float(print_config, "initial_layer_infill_line_width")
        } else if first_layer && cfg_float(print_config, "initial_layer_line_width") > 0. {
            cfg_float(print_config, "initial_layer_line_width")
        } else {
            match role {
                FlowRole::ExternalPerimeter => cfg_float(m_config, "outer_wall_line_width"),
                FlowRole::Perimeter => cfg_float(m_config, "inner_wall_line_width"),
                FlowRole::Infill => cfg_float(m_config, "sparse_infill_line_width"),
                FlowRole::SolidInfill => cfg_float(m_config, "internal_solid_infill_line_width"),
                FlowRole::TopSolidInfill => cfg_float(m_config, "top_surface_line_width"),
                _ => return Err(crate::Error::InvalidInput("Unknown role".to_string())),
            }
        };

        if config_width == 0. {
            config_width = cfg_float(object.config(), "line_width");
        }

        // Get the configured nozzle_diameter for the extruder that this role's filament is grouped to.
        let nozzle_diameter = pf::nozzle_diameter_for_filament(print_config, region.extruder(role));
        Ok(flow_new_from_config_width(
            role,
            &ConfigOptionFloat::with_value(config_width),
            primary_nozzle_diameter(print.full_print_config()) as f32,
            nozzle_diameter,
            layer_height as f32,
            multi_nozzle_diameter_mode_enabled(print.full_print_config()),
        )?)
    }

    /// PrintRegion.cpp:60-65 `coordf_t PrintRegion::nozzle_dmr_avg(const PrintConfig &print_config) const`: the three
    /// floats summed as floats, the sum divided as a double.
    fn print_region_nozzle_dmr_avg(region: &pf::PrintRegion, print_config: &pf::PrintConfig) -> f64 {
        let m_config = region.config();
        let sum: f32 = pf::nozzle_diameter_for_filament(print_config, cfg_int(m_config, "wall_filament") as u32)
            + pf::nozzle_diameter_for_filament(print_config, cfg_int(m_config, "sparse_infill_filament") as u32)
            + pf::nozzle_diameter_for_filament(print_config, cfg_int(m_config, "solid_infill_filament") as u32);
        sum as f64 / 3.
    }

    /// PrintRegion.cpp:67-70 `coordf_t PrintRegion::bridging_height_avg(const PrintConfig &print_config) const`
    fn print_region_bridging_height_avg(region: &pf::PrintRegion, print_config: &pf::PrintConfig) -> f64 {
        print_region_nozzle_dmr_avg(region, print_config) * cfg_float(region.config(), "bridge_flow").sqrt()
    }

    /// LayerRegion.cpp:21-29 `Flow LayerRegion::flow(FlowRole role) const`: `m_region->flow(*m_layer->object(), role,
    /// m_layer->height, m_layer->id() == 0)`. The region of a LayerRegion is the printing region of its index.
    fn layer_region_flow(object: &pf::PrintObject, print: &pf::Print, layer: &Layer, layerm: &LayerRegion, role: FlowRole) -> crate::Result<Flow> {
        let region = object.printing_region(layerm.region_id());
        let region = region.borrow();
        print_region_flow(&region, object, print, role, layer.height, layer.id() == 0)
    }

    /// LayerRegion.cpp:31-46 `Flow LayerRegion::bridging_flow(FlowRole role, bool thick_bridge) const`
    fn layer_region_bridging_flow(
        object: &pf::PrintObject,
        print: &pf::Print,
        layer: &Layer,
        layerm: &LayerRegion,
        role: FlowRole,
        thick_bridge: bool,
    ) -> crate::Result<Flow> {
        let region_ptr = object.printing_region(layerm.region_id());
        let region = region_ptr.borrow();
        let bridge_flow = cfg_float(region.config(), "bridge_flow");
        if thick_bridge {
            // The old Slic3r way (different from all other slicers): Use rounded extrusions.
            // Get the configured nozzle_diameter for the extruder that this role's filament is grouped to.
            let nozzle_diameter: f32 = pf::nozzle_diameter_for_filament(print.config(), region.extruder(role));
            // Applies default bridge spacing.
            // Flow::bridging_flow(float(sqrt(region_config.bridge_flow)) * nozzle_diameter, nozzle_diameter)
            Ok(Flow::bridging_flow(((bridge_flow.sqrt() as f32) * nozzle_diameter) as f64, nozzle_diameter as f64))
        } else {
            // The same way as other slicers: Use normal extrusions. Apply bridge_flow while maintaining the original spacing.
            Ok(print_region_flow(&region, object, print, role, layer.height, layer.id() == 0)?.with_flow_ratio(bridge_flow)?)
        }
    }

    /// ExtrusionEntity.hpp:192 `Polylines ExtrusionEntity::as_polylines() const` of a collection: a path its
    /// polyline (ExtrusionEntity.hpp:346), a multi path and a loop their paths joined into one polyline
    /// (`as_polyline()`); the empty ones are left out.
    fn collect_polylines(entities: &[ExtrusionEntityType], dst: &mut Polylines) {
        fn joined(paths: &[ExtrusionPath]) -> Polyline {
            let mut out = Polyline::from_points(Vec::new());
            if !paths.is_empty() {
                out.points = paths[0].polyline.points.clone();
                for path in &paths[1..] {
                    if path.polyline.points.len() > 1 {
                        out.points.extend_from_slice(&path.polyline.points[1..]);
                    }
                }
            }
            out
        }
        for entity in entities {
            match entity {
                ExtrusionEntityType::Path(path) => {
                    if !path.polyline.points.is_empty() {
                        dst.push(path.polyline.clone());
                    }
                }
                ExtrusionEntityType::MultiPath(multipath) => {
                    let pl = joined(&multipath.paths);
                    if !pl.points.is_empty() {
                        dst.push(pl);
                    }
                }
                ExtrusionEntityType::Loop(loop_) => {
                    let pl = joined(&loop_.paths);
                    if !pl.points.is_empty() {
                        dst.push(pl);
                    }
                }
                ExtrusionEntityType::Collection(collection) => collect_polylines(&collection.entities, dst),
            }
        }
    }

    // ------------------------------------------------------------------------------------------------
    // SupportMaterial.cpp:1173-1342 `namespace SupportMaterialInternal`
    // ------------------------------------------------------------------------------------------------

    /// SupportMaterial.cpp:1174-1180
    /// C++: `static inline bool has_bridging_perimeters(const ExtrusionLoop &loop)`
    pub fn has_bridging_perimeters_loop(loop_: &ExtrusionLoop) -> bool {
        for ep in &loop_.paths {
            if ep.role == ExtrusionRole::OverhangPerimeter && !ep.polyline.points.is_empty() {
                return ep.polyline.points.len() as i32 >= (if ep.polyline.is_closed() { 3 } else { 2 });
            }
        }
        false
    }

    /// SupportMaterial.cpp:1181-1195
    /// C++: `static bool has_bridging_perimeters(const ExtrusionEntityCollection &perimeters)`
    pub fn has_bridging_perimeters(perimeters: &ExtrusionEntityCollection) -> bool {
        for ee in &perimeters.entities {
            match ee {
                ExtrusionEntityType::Collection(collection) => {
                    for ee2 in &collection.entities {
                        debug_assert!(!matches!(ee2, ExtrusionEntityType::Collection(_)));
                        if let ExtrusionEntityType::Loop(loop_) = ee2 {
                            if has_bridging_perimeters_loop(loop_) {
                                return true;
                            }
                        }
                    }
                }
                ExtrusionEntityType::Loop(loop_) => {
                    if has_bridging_perimeters_loop(loop_) {
                        return true;
                    }
                }
                _ => {}
            }
        }
        false
    }

    /// SupportMaterial.cpp:1196-1208
    /// C++: `static bool has_bridging_fills(const ExtrusionEntityCollection &fills)`
    pub fn has_bridging_fills(fills: &ExtrusionEntityCollection) -> bool {
        for ee in &fills.entities {
            // assert(ee->is_collection());
            if let ExtrusionEntityType::Collection(collection) = ee {
                for ee2 in &collection.entities {
                    // assert(! ee2->is_collection()); assert(! ee2->is_loop());
                    let role = match ee2 {
                        ExtrusionEntityType::Path(path) => path.role,
                        ExtrusionEntityType::MultiPath(multipath) => multipath.role(),
                        ExtrusionEntityType::Loop(loop_) => loop_.role(),
                        // ExtrusionEntityCollection.hpp `role()`: the role of its entities, erMixed when they differ.
                        ExtrusionEntityType::Collection(c) => c.role(),
                    };
                    if role == ExtrusionRole::BridgeInfill {
                        return true;
                    }
                }
            }
        }
        false
    }

    /// SupportMaterial.cpp:1209-1218
    /// C++: `static bool has_bridging_extrusions(const Layer &layer)`
    pub fn has_bridging_extrusions(layer: &Layer) -> bool {
        for region in layer.regions() {
            if has_bridging_perimeters(&region.perimeters) {
                return true;
            }
            if region.fill_surfaces.has(SurfaceType::BottomBridge) && has_bridging_fills(&region.fills) {
                return true;
            }
        }
        false
    }

    /// SupportMaterial.cpp:1220-1245
    /// C++: `static inline void collect_bridging_perimeter_areas(const ExtrusionLoop &loop, const float expansion_scaled, Polygons &out)`
    pub fn collect_bridging_perimeter_areas_loop(loop_: &ExtrusionLoop, expansion_scaled: f32, out: &mut Polygons) {
        debug_assert!(expansion_scaled >= 0.);
        for ep in &loop_.paths {
            if ep.role == ExtrusionRole::OverhangPerimeter && !ep.polyline.points.is_empty() {
                // float exp = 0.5f * (float)scale_(ep.width) + expansion_scaled;  (`float width`)
                let exp: f32 = 0.5f32.mul_add(scale_((ep.width as f32) as f64) as f32, expansion_scaled);
                if ep.polyline.is_closed() {
                    if ep.polyline.points.len() >= 3 {
                        // This is a complete loop.
                        // Add the outer contour first.
                        let mut poly = Polygon::default();
                        poly.points = ep.polyline.points.clone();
                        poly.points.pop();
                        if poly.area() < 0. {
                            poly.reverse();
                        }
                        out.extend(cu::offset_polygon(&poly, exp, SSOP_JT, SSOP_ML));
                        let mut holes: Polygons = cu::offset_polygon(&poly, -exp, SSOP_JT, SSOP_ML);
                        crate::geometry::polygon::polygons_reverse(&mut holes);
                        out.extend(holes);
                    }
                } else if ep.polyline.points.len() >= 2 {
                    // Offset the polyline.
                    out.extend(cu::offset_polyline(&ep.polyline, exp, SSOP_JT, SSOP_ML, cu::DEFAULT_END_TYPE));
                }
            }
        }
    }

    /// SupportMaterial.cpp:1246-1258
    /// C++: `static void collect_bridging_perimeter_areas(const ExtrusionEntityCollection &perimeters, const float expansion_scaled, Polygons &out)`
    pub fn collect_bridging_perimeter_areas(perimeters: &ExtrusionEntityCollection, expansion_scaled: f32, out: &mut Polygons) {
        for ee in &perimeters.entities {
            match ee {
                ExtrusionEntityType::Collection(collection) => {
                    for ee2 in &collection.entities {
                        debug_assert!(!matches!(ee2, ExtrusionEntityType::Collection(_)));
                        if let ExtrusionEntityType::Loop(loop_) = ee2 {
                            collect_bridging_perimeter_areas_loop(loop_, expansion_scaled, out);
                        }
                    }
                }
                ExtrusionEntityType::Loop(loop_) => collect_bridging_perimeter_areas_loop(loop_, expansion_scaled, out),
                _ => {}
            }
        }
    }

    /// SupportMaterial.cpp:1260-1341
    /// C++: `static void remove_bridges_from_contacts(const PrintConfig &print_config, const Layer &lower_layer, const
    /// Polygons &lower_layer_polygons, const LayerRegion &layerm, float fw, Polygons &contact_polygons)`
    ///
    /// `layer` is `layerm.layer()`, `object` its object and `print` the object's Print.
    #[allow(clippy::too_many_arguments)]
    pub fn remove_bridges_from_contacts(
        object: &pf::PrintObject,
        print: &pf::Print,
        print_config: &pf::PrintConfig,
        lower_layer: &Layer,
        lower_layer_polygons: &Polygons,
        layer: &Layer,
        layerm: &LayerRegion,
        fw: f32,
        contact_polygons: &mut Polygons,
    ) -> crate::Result<()> {
        // compute the area of bridging perimeters
        let mut bridges: Polygons = Vec::new();
        {
            // Surface supporting this layer, expanded by 0.5 * nozzle_diameter, as we consider this kind of overhang to be sufficiently supported.
            let wall_filament = {
                let region = object.printing_region(layerm.region_id());
                let region = region.borrow();
                cfg_int(region.config(), "wall_filament") as u32
            };
            let lower_grown_slices: Polygons = cu::expand(
                lower_layer_polygons,
                //FIXME to mimic the decision in the perimeter generator, we should use half the external perimeter width.
                0.5f32 * (scale_(pf::nozzle_diameter_for_filament(print_config, wall_filament) as f64) as f32),
                SSOP_JT,
                SSOP_ML,
            );
            // Collect perimeters of this layer.
            //FIXME split_at_first_point() could split a bridge mid-way
            let mut perimeters: Polylines = Vec::new();
            collect_polylines(&layerm.perimeters.entities, &mut perimeters);
            let mut overhang_perimeters: Polylines = cu::diff_pl(&perimeters, &lower_grown_slices);

            // only consider straight overhangs
            // only consider overhangs having endpoints inside layer's slices
            // convert bridging polylines into polygons by inflating them with their thickness
            // since we're dealing with bridges, we can't assume width is larger than spacing,
            // so we take the largest value and also apply safety offset to be ensure no gaps
            // are left in between
            // BBS
            let object_config = object.config();
            let perimeter_bridge_flow = layer_region_bridging_flow(object, print, layer, layerm, FlowRole::Perimeter, cfg_bool(object_config, "thick_bridges"))?;
            //FIXME one may want to use a maximum of bridging flow width and normal flow width, as the perimeters are calculated using the normal flow
            // and then turned to bridging flow, thus their centerlines are derived from non-bridging flow and expanding them by a bridging flow
            // may not expand them to the edge of their respective islands.
            // const float w = float(0.5 * std::max(scaled_width(), scaled_spacing())) + scaled<float>(0.001);
            let w: f32 = (0.5 * perimeter_bridge_flow.scaled_width().max(perimeter_bridge_flow.scaled_spacing()) as f64) as f32 + scale_(0.001) as f32;
            for polyline in overhang_perimeters.iter_mut() {
                if polyline.is_straight() {
                    // This is a bridge
                    polyline.extend_start(fw as f64);
                    polyline.extend_end(fw as f64);
                    // Is the straight perimeter segment supported at both sides?
                    let pts = [polyline.first_point(), polyline.last_point()];
                    let mut supported = [false, false];
                    let mut i = 0;
                    while i < lower_layer.lslices.len() && !(supported[0] && supported[1]) {
                        for j in 0..2 {
                            // `lower_layer.lslices_bboxes[i].contains(pts[j])`
                            let b = &lower_layer.lslices_bboxes[i];
                            let in_bbox = pts[j].x >= b.min.x && pts[j].x <= b.max.x && pts[j].y >= b.min.y && pts[j].y <= b.max.y;
                            if !supported[j] && in_bbox && cu_expolygon::contains(&lower_layer.lslices[i], &pts[j], true) {
                                supported[j] = true;
                            }
                        }
                        i += 1;
                    }
                    if supported[0] && supported[1] {
                        // Offset a polyline into a thick line.
                        bridges.extend(cu::offset_polyline(polyline, w, cu::DEFAULT_LINE_JOIN_TYPE, cu::DEFAULT_LINE_MITER_LIMIT, cu::DEFAULT_END_TYPE));
                    }
                }
            }
            bridges = cu::union_(&bridges);
        }
        // remove the entire bridges and only support the unsupported edges
        //FIXME the brided regions are already collected as layerm.bridged. Use it?
        for surface in &layerm.fill_surfaces.surfaces {
            // `surface.bridge_angle != -1`: an angle that is not set is -1 in the C++.
            if surface.surface_type == SurfaceType::BottomBridge && surface.bridge_angle.map(|a| a != -1.).unwrap_or(false) {
                crate::geometry::polygons_append_expoly(&mut bridges, &surface.expolygon);
            }
        }
        //FIXME add the gap filled areas. Extrude the gaps with a bridge flow?
        // Remove the unsupported ends of the bridges from the bridged areas.
        //FIXME add supports at regular intervals to support long bridges!
        bridges = cu::diff(
            &bridges,
            // Offset unsupported edges into polygons.
            &cu::offset_polylines(&layerm.unsupported_bridge_edges, scale_(SUPPORT_MATERIAL_MARGIN) as f32, SSOP_JT, SSOP_ML, cu::DEFAULT_END_TYPE),
            ApplySafetyOffset::No,
        );
        // Remove bridged areas from the supported areas.
        *contact_polygons = cu::diff(contact_polygons, &bridges, ApplySafetyOffset::Yes);
        Ok(())
    }

    // ------------------------------------------------------------------------------------------------
    // SupportMaterial.cpp:1371-1395
    // ------------------------------------------------------------------------------------------------

    /// SupportMaterial.cpp:1371-1386 `struct SupportAnnotations`
    pub struct SupportAnnotations<'a> {
        pub enforcers_layers: Vec<Polygons>,
        pub blockers_layers: Vec<Polygons>,
        pub buildplate_covered: &'a [Polygons],
    }

    impl<'a> SupportAnnotations<'a> {
        /// SupportMaterial.cpp:1373-1381
        /// C++: `SupportAnnotations(const PrintObject &object, const std::vector<Polygons> &buildplate_covered)`
        ///
        /// `projector` is `PrintObject::project_and_append_custom_facets` (PrintObject.cpp:6726-6765, see
        /// `CustomFacetsProjector`); without one the object counts as not painted.
        pub fn new(
            object: &pf::PrintObject,
            print: &pf::Print,
            buildplate_covered: &'a [Polygons],
            projector: Option<&dyn CustomFacetsProjector>,
        ) -> crate::Result<Self> {
            use crate::model::ModelVolumeType;
            use crate::triangle_selector::EnforcerBlockerType;
            // Print.hpp:544-545 `slice_support_enforcers()` / `slice_support_blockers()`
            let mut enforcers_layers: Vec<Polygons> =
                object.slice_support_volumes(print, ModelVolumeType::SupportEnforcer).map_err(|e| crate::Error::Slicing(e.what().to_string()))?;
            let mut blockers_layers: Vec<Polygons> =
                object.slice_support_volumes(print, ModelVolumeType::SupportBlocker).map_err(|e| crate::Error::Slicing(e.what().to_string()))?;
            // Append custom supports.
            if let Some(projector) = projector {
                projector.project_and_append_custom_facets(object, print, false, EnforcerBlockerType::ENFORCER, &mut enforcers_layers, None);
                projector.project_and_append_custom_facets(object, print, false, EnforcerBlockerType::BLOCKER, &mut blockers_layers, None);
            }
            Ok(SupportAnnotations { enforcers_layers, blockers_layers, buildplate_covered })
        }
    }

    /// SupportMaterial.cpp:1388-1395 `struct SlicesMarginCache`
    #[derive(Debug, Clone)]
    pub struct SlicesMarginCache {
        pub offset: f32,
        // Trimming polygons, including possibly the "build plate only" mask.
        pub polygons: Polygons,
        // Trimming polygons, without the "build plate only" mask. If empty, use polygons.
        pub all_polygons: Polygons,
    }

    impl Default for SlicesMarginCache {
        fn default() -> Self {
            SlicesMarginCache { offset: -1., polygons: Vec::new(), all_polygons: Vec::new() }
        }
    }

    // ------------------------------------------------------------------------------------------------
    // SupportMaterial.cpp:1404-1735
    // ------------------------------------------------------------------------------------------------

    /// SupportMaterial.cpp:1406-1595
    /// C++: `static inline ExPolygons detect_overhangs(const Layer &layer, const size_t layer_id, Polygons
    /// &lower_layer_polygons, const PrintConfig &print_config, const PrintObjectConfig &object_config,
    /// SupportAnnotations &annotations, const double gap_xy)`
    ///
    /// `sharp_tails`, `sharp_tails_height` and `cantilevers` are the mutable members of `layer` the function
    /// writes (Layer.hpp:172-174); `object` and `print` are what the layer reaches through its pointers.
    #[allow(clippy::too_many_arguments)]
    pub fn detect_overhangs(
        object: &pf::PrintObject,
        print: &pf::Print,
        layer: &Layer,
        layer_id: usize,
        lower_layer_polygons: &Polygons,
        print_config: &pf::PrintConfig,
        object_config: &pf::PrintObjectConfig,
        annotations: &SupportAnnotations,
        _gap_xy: f64,
        sharp_tails: &mut ExPolygons,
        sharp_tails_height: &mut Vec<f32>,
        cantilevers: &mut ExPolygons,
    ) -> crate::Result<ExPolygons> {
        // Snug overhang polygons.
        let mut overhang_polygons: Polygons = Vec::new();

        // BBS.
        let auto_normal_support = cfg_int(object_config, "support_type") == SupportType::StNormalAuto as i32;
        let buildplate_only = !annotations.buildplate_covered.is_empty();
        // If user specified a custom angle threshold, convert it to radians.
        // Zero means automatic overhang detection.
        // +1 makes the threshold inclusive
        let support_threshold_angle = cfg_int(object_config, "support_threshold_angle");
        let mut thresh_angle: f64 = if support_threshold_angle > 0 { (support_threshold_angle + 1) as f64 } else { 0. };
        thresh_angle = thresh_angle.min(89.); // BBS should be smaller than 90
        // Geometry.hpp:295 `deg2rad(angle)`: `T(PI) * angle / T(180.0)`
        let threshold_rad: f64 = std::f64::consts::PI * thresh_angle / 180.0;
        let _max_bridge_length: f64 = scale_(cfg_float(object_config, "max_bridge_length"));
        let bridge_no_support = cfg_bool(object_config, "bridge_no_support");
        let xy_expansion: f64 = scale_(cfg_float(object_config, "support_expansion"));
        let mut lower_layer_offset: f32 = 0.;

        if layer_id == 0 {
            // Don't fill in the holes. The user may apply a higher raft_expansion if one wants a better 1st layer adhesion.
            overhang_polygons = to_polygons(&layer.lslices);

            for slice in &layer.lslices {
                // `get_extents(slice).size()`: the box of the contour.
                let bbox_size = bbox_size(&BBox::from_points(&slice.contour.points));
                if G_CONFIG_SUPPORT_SHARP_TAILS
                    && !(bbox_size.x as f64 > length_thresh_well_supported() && bbox_size.y as f64 > length_thresh_well_supported())
                {
                    sharp_tails.push(slice.clone());
                    // `std::vector<float> sharp_tails_height`
                    sharp_tails_height.push(layer.height as f32);
                }
            }
        } else if !layer.regions().is_empty() {
            // Generate overhang / contact_polygons for non-raft layers.
            let lower_layer: &Layer = &object.layers()[layer.lower_layer_id.expect("detect_overhangs: a layer above the first has a lower layer")];
            // Can't directly use lower_layer.lslices, or we'll miss some very sharp tails.
            // Filter out areas whose diameter that is smaller than extrusion_width. Do not use offset2() for this purpose!
            // FIXME if there are multiple regions with different extrusion width, the following code may not be right.
            let fw: f32 = layer_region_flow(object, print, layer, &layer.regions()[0], FlowRole::ExternalPerimeter)?.scaled_width() as f32;
            let mut lower_layer_expolys: ExPolygons = Vec::new();
            for expoly in &lower_layer.lslices {
                if !cu_expolygon::offset_ex(expoly, -fw / 2., cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT).is_empty() {
                    lower_layer_expolys.push(expoly.clone());
                }
            }

            for layerm in layer.regions() {
                // Extrusion width accounts for the roundings of the extrudates.
                // It is the maximum widh of the extrudate.
                let fw: f32 = layer_region_flow(object, print, layer, layerm, FlowRole::ExternalPerimeter)?.scaled_width() as f32;
                // `layer_id < (size_t)object_config.enforce_support_layers.value`
                lower_layer_offset = if (layer_id as u64) < (cfg_int(object_config, "enforce_support_layers") as i64 as u64) {
                    // Enforce a full possible support, ignore the overhang angle.
                    0.
                } else if threshold_rad > 0. {
                    // Overhang defined by an angle.
                    scale_(lower_layer.height / threshold_rad.tan()) as f32
                } else {
                    // Overhang defined by half the extrusion width.
                    0.5f32 * fw
                };
                // Overhang polygons for this layer and region.
                let mut diff_polygons: Polygons = Vec::new();
                let layerm_polygons: Polygons = crate::surface::to_polygons(&layerm.slices.surfaces);
                if lower_layer_offset == 0. {
                    // Support everything.
                    diff_polygons = cu::diff(&layerm_polygons, lower_layer_polygons, ApplySafetyOffset::No);
                    if buildplate_only {
                        // Don't support overhangs above the top surfaces.
                        // This step is done before the contact surface is calculated by growing the overhang region.
                        diff_polygons = cu::diff(&diff_polygons, &annotations.buildplate_covered[layer_id], ApplySafetyOffset::No);
                    }
                } else if auto_normal_support {
                    // Get the regions needing a suport, collapse very tiny spots.
                    //FIXME cache the lower layer offset if this layer has multiple regions.
                    diff_polygons =
                        cu::diff(&layerm_polygons, &cu::offset(lower_layer_polygons, lower_layer_offset, SSOP_JT, SSOP_ML), ApplySafetyOffset::No);
                    if buildplate_only && !annotations.buildplate_covered[layer_id].is_empty() {
                        // Don't support overhangs above the top surfaces.
                        // This step is done before the contact surface is calculated by growing the overhang region.
                        diff_polygons = cu::diff(&diff_polygons, &annotations.buildplate_covered[layer_id], ApplySafetyOffset::No);
                    }
                    if !diff_polygons.is_empty() {
                        // Offset the support regions back to a full overhang, restrict them to the full overhang.
                        // This is done to increase size of the supporting columns below, as they are calculated by
                        // propagating these contact surfaces downwards.
                        diff_polygons = cu::diff(
                            &cu::intersection(&cu::offset(&diff_polygons, lower_layer_offset, SSOP_JT, SSOP_ML), &layerm_polygons, ApplySafetyOffset::No),
                            lower_layer_polygons,
                            ApplySafetyOffset::No,
                        );
                    }
                    //FIXME add user defined filtering here based on minimal area or minimum radius or whatever.

                    // BBS
                    if G_CONFIG_SUPPORT_SHARP_TAILS {
                        // `ExPolygons raw_slices` (Layer.hpp:44); the crate keeps them as surfaces.
                        for raw_slice in &layerm.raw_slices.surfaces {
                            let expoly: &ExPolygon = &raw_slice.expolygon;
                            // `-0.5 * fw`, `0.5 * fw`, `-0.1 * fw`, `0.05 * fw`: doubles, handed over as floats
                            if cu_expolygon::offset_ex(expoly, (-0.5 * fw as f64) as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT).is_empty() {
                                continue;
                            }
                            let mut is_sharp_tail = false;
                            let accum_height: f32 = layer.height as f32;

                            // 1. nothing below
                            // Check whether this is a sharp tail region.
                            // Should use lower_layer_expolys without any offset. Otherwise, it may missing sharp tails near the main body.
                            if !cu_expolygon::overlaps_expolygons(
                                &cu_expolygon::offset_ex(expoly, (0.5 * fw as f64) as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
                                &lower_layer_expolys,
                            ) {
                                is_sharp_tail = expoly.area() < area_thresh_well_supported()
                                    && !cu_expolygon::offset_ex(expoly, (-0.1 * fw as f64) as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT).is_empty();
                            }

                            if is_sharp_tail {
                                let mut overhang: ExPolygons = cu::diff_ex_ee(std::slice::from_ref(expoly), &lower_layer_expolys, ApplySafetyOffset::No);
                                sharp_tails.push(expoly.clone());
                                sharp_tails_height.push(accum_height);
                                overhang = cu::offset_ex_expolygons(&overhang, (0.05 * fw as f64) as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);
                                diff_polygons.extend(to_polygons(&overhang));
                            }
                        }
                    }
                }

                if diff_polygons.is_empty() {
                    continue;
                }

                // Apply the "support blockers".
                if !annotations.blockers_layers.is_empty() && !annotations.blockers_layers[layer_id].is_empty() {
                    // Expand the blocker a bit. Custom blockers produce strips
                    // spanning just the projection between the two slices.
                    // Subtracting them as they are may leave unwanted narrow
                    // residues of diff_polygons that would then be supported.
                    let blocker =
                        cu::expand(&cu::union_(&annotations.blockers_layers[layer_id]), (1000. * scaled_epsilon()) as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);
                    diff_polygons = cu::diff(&diff_polygons, &blocker, ApplySafetyOffset::No);
                    *sharp_tails = cu::diff_ex_ep(sharp_tails, &blocker, ApplySafetyOffset::No);
                }

                if bridge_no_support {
                    //FIXME Expensive, potentially not precise enough. Misses gap fill extrusions, which bridge.
                    remove_bridges_from_contacts(object, print, print_config, lower_layer, lower_layer_polygons, layer, layerm, fw, &mut diff_polygons)?;
                }

                if diff_polygons.is_empty() || cu::offset(&diff_polygons, (-0.1 * fw as f64) as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT).is_empty() {
                    continue;
                }

                if xy_expansion > 0. {
                    // Step-wise expand and diff against the lower layer outline so that the expansion
                    // cannot bridge over thin model walls into regions that don't actually need support.
                    // (the doubles truncated to coord_t)
                    diff_polygons = safe_offset_inc(&diff_polygons, xy_expansion as i64, lower_layer_polygons, scale_(0.5) as i64, 0, 1);
                } else if xy_expansion < 0. {
                    // `expand()` with a negative delta: its assert is not compiled in, it is the offset.
                    diff_polygons = cu::offset(&diff_polygons, xy_expansion as f32, SSOP_JT, SSOP_ML);
                }

                overhang_polygons.extend(diff_polygons);
            } // for each layer.region
        }

        let overhang_areas: ExPolygons = cu::union_ex(&overhang_polygons);
        // check cantilever
        if let Some(lower_layer_idx) = layer.lower_layer_id {
            let lower_layer = &object.layers()[lower_layer_idx];
            for poly in &overhang_areas {
                let fw: f32 = layer_region_flow(object, print, layer, &layer.regions()[0], FlowRole::ExternalPerimeter)?.scaled_width() as f32;
                // `std::max(fw, lower_layer_offset) + scale_(0.1)`: a double, handed over as a float
                let cluster_boundary_ex: ExPolygons = cx::intersection_ex_ee(
                    std::slice::from_ref(poly),
                    &cu::offset_ex_expolygons(
                        &lower_layer.lslices,
                        (fw.max(lower_layer_offset) as f64 + scale_(0.1)) as f32,
                        cu::DEFAULT_JOIN_TYPE,
                        cu::DEFAULT_MITER_LIMIT,
                    ),
                );
                let cluster_boundary: Polygons = to_polygons(&cluster_boundary_ex);
                if cluster_boundary.is_empty() {
                    continue;
                }
                let mut dist_max: f64 = 0.;
                for pt in &poly.contour.points {
                    let mut dist_pt: f64 = f64::MAX;
                    for ply in &cluster_boundary {
                        let d = multipoint_distance_to(&ply.points, pt);
                        dist_pt = dist_pt.min(d);
                    }
                    dist_max = dist_max.max(dist_pt);
                }
                if dist_max > scale_(3.) {
                    // is cantilever if the farmost point is larger than 3mm away from base
                    cantilevers.push(poly.clone());
                }
            }
        }

        Ok(overhang_areas)
    }

    /// SupportMaterial.cpp:1599-1735
    /// C++: `static inline std::tuple<Polygons, Polygons, double> detect_contacts(const Layer &layer, const size_t
    /// layer_id, Polygons &overhang_polygons, Polygons &lower_layer_polygons, const PrintConfig &print_config, const
    /// PrintObjectConfig &object_config, SupportAnnotations &annotations, SlicesMarginCache &slices_margin, const
    /// double gap_xy)`
    ///
    /// Tuple: contact_polygons, enforcer_polygons, no_interface_offset
    /// no_interface_offset: minimum of external perimeter widths
    #[allow(clippy::too_many_arguments)]
    pub fn detect_contacts(
        object: &pf::PrintObject,
        print: &pf::Print,
        layer: &Layer,
        layer_id: usize,
        overhang_polygons: &mut Polygons,
        lower_layer_polygons: &Polygons,
        _print_config: &pf::PrintConfig,
        object_config: &pf::PrintObjectConfig,
        annotations: &SupportAnnotations,
        slices_margin: &mut SlicesMarginCache,
        gap_xy: f64,
    ) -> crate::Result<(Polygons, Polygons, f64)> {
        // Expanded for stability, trimmed by gap_xy.
        let mut contact_polygons: Polygons = Vec::new();
        // Enforcers projected to overhangs, trimmed
        let mut enforcer_polygons: Polygons = Vec::new();

        // BBS.
        let buildplate_only = !annotations.buildplate_covered.is_empty();
        let mut no_interface_offset: f32 = 0.;

        if layer_id == 0 {
            // Expand for better stability.
            let raft_expansion = cfg_float(object_config, "raft_expansion");
            contact_polygons = if raft_expansion > 0. {
                cu::expand(overhang_polygons, scale_(raft_expansion) as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT)
            } else {
                overhang_polygons.clone()
            };
        } else if !layer.regions().is_empty() {
            // Generate overhang / contact_polygons for non-raft layers.
            let lower_layer: &Layer = &object.layers()[layer.lower_layer_id.expect("detect_contacts: a layer above the first has a lower layer")];
            let has_enforcer = !annotations.enforcers_layers.is_empty() && !annotations.enforcers_layers[layer_id].is_empty();

            // Cache support trimming polygons derived from lower layer polygons, possible merged with "on build plate only" trimming polygons.
            let slices_margin_update = |slices_margin: &mut SlicesMarginCache, slices_margin_offset: f32, no_interface_offset: f32| {
                if slices_margin.offset != slices_margin_offset {
                    slices_margin.offset = slices_margin_offset;
                    slices_margin.polygons = if slices_margin_offset == 0. {
                        lower_layer_polygons.clone()
                    } else {
                        // What is the purpose of no_interface_offset? Likely to not trim the contact layer by lower layer regions that are too thin to extrude?
                        cx::offset2(
                            &lower_layer.lslices,
                            -no_interface_offset * 0.5f32,
                            no_interface_offset.mul_add(0.5f32, slices_margin_offset),
                            SSOP_JT,
                            SSOP_ML,
                        )
                    };
                    if buildplate_only && !annotations.buildplate_covered[layer_id].is_empty() {
                        if has_enforcer {
                            // Make a backup of trimming polygons before enforcing "on build plate only".
                            slices_margin.all_polygons = slices_margin.polygons.clone();
                        }
                        // Trim the inflated contact surfaces by the top surfaces as well.
                        slices_margin.polygons = cu::union_2(&slices_margin.polygons, &annotations.buildplate_covered[layer_id]);
                    }
                }
            };

            // std::accumulate(regions, FLT_MAX, min(acc, float(layerm->flow(frExternalPerimeter).scaled_width())))
            no_interface_offset = f32::MAX;
            for layerm in layer.regions() {
                let w = layer_region_flow(object, print, layer, layerm, FlowRole::ExternalPerimeter)?.scaled_width() as f32;
                // std::min(acc, w): `w < acc ? w : acc`
                no_interface_offset = if w < no_interface_offset { w } else { no_interface_offset };
            }

            let lower_layer_offset: f32 = 0.;
            let xy_expansion: f64 = scale_(cfg_float(object_config, "support_expansion"));
            let expand_and_trim = |src: &Polygons, trimming: &Polygons| -> Polygons {
                if xy_expansion > 0. {
                    return safe_offset_inc(src, xy_expansion as i64, trimming, scale_(0.5) as i64, 0, 1);
                }
                if xy_expansion < 0. {
                    return cu::diff(&cu::offset(src, xy_expansion as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT), trimming, ApplySafetyOffset::No);
                }
                cu::diff(src, trimming, ApplySafetyOffset::No)
            };
            // std::min(lower_layer_offset, float(scale_(gap_xy))): `b < a ? b : a`
            let gap_xy_scaled: f32 = scale_(gap_xy) as f32;
            let margin_offset: f32 = if gap_xy_scaled < lower_layer_offset { gap_xy_scaled } else { lower_layer_offset };
            for layerm in layer.regions() {
                let layerm_polygons: Polygons = crate::surface::to_polygons(&layerm.slices.surfaces);

                // Overhang polygons for this layer and region.
                let mut diff_polygons: Polygons = cu::intersection(overhang_polygons, &layerm_polygons, ApplySafetyOffset::No);
                if diff_polygons.is_empty() {
                    continue;
                }

                // Let's define the required contact area by using a max gap of half the upper
                // extrusion width and extending the area according to the configured margin.
                // We increment the area in steps because we don't want our support to overflow
                // on the other side of the object (if it's very thin).
                {
                    //FIMXE 1) Make the offset configurable, 2) Make the Z span configurable.
                    //FIXME one should trim with the layer span colliding with the support layer, this layer
                    // may be lower than lower_layer, so the support area needed may need to be actually bigger!
                    // For the same reason, the non-bridging support area may be smaller than the bridging support area!
                    slices_margin_update(slices_margin, margin_offset, no_interface_offset);
                    // Offset the contact polygons outside.
                    // Step-wise expansion prevents crossing thin model walls into regions that do not need support.
                    diff_polygons = expand_and_trim(&diff_polygons, &slices_margin.polygons);
                }
                contact_polygons.extend(diff_polygons);
            } // for each layer.region

            if has_enforcer {
                let enforcer_polygons_src: &Polygons = &annotations.enforcers_layers[layer_id];
                if !enforcer_polygons_src.is_empty() {
                    // Enforce supports (as if with 90 degrees of slope) for the regions covered by the enforcer meshes.
                    enforcer_polygons = cu::diff(
                        &cx::intersection_ep(&layer.lslices, enforcer_polygons_src),
                        // Inflate just a tiny bit to avoid intersection of the overhang areas with the object.
                        &cu::offset(lower_layer_polygons, 0.05f32 * no_interface_offset, SSOP_JT, SSOP_ML),
                        ApplySafetyOffset::No,
                    );
                    if !enforcer_polygons.is_empty() {
                        slices_margin_update(slices_margin, margin_offset, no_interface_offset);
                        let enforcer_slices_margin: &Polygons =
                            if slices_margin.all_polygons.is_empty() { &slices_margin.polygons } else { &slices_margin.all_polygons };
                        enforcer_polygons = expand_and_trim(&enforcer_polygons, enforcer_slices_margin);
                        overhang_polygons.extend(enforcer_polygons.iter().cloned());
                        contact_polygons.extend(enforcer_polygons.iter().cloned());
                    }
                }
            }
        }

        Ok((contact_polygons, enforcer_polygons, no_interface_offset as f64))
    }

    /// SupportMaterial.cpp:1737-1779
    /// C++: `Layer* sync_gap_with_object_layer(const Layer& layer, const coordf_t gap_support_object, bool is_top_contact)`
    ///
    /// find the object layer that is closest to the {layer.bottom_z-gap_support_object} for top contact,
    /// or {layer.print_z+gap_object_support} for bottom contact
    ///
    /// The layers are the indices of `layers`; the C++ dereferences `last_valid_gap_layer`, which is null when
    /// the layer has no neighbour on that side.
    pub fn sync_gap_with_object_layer(layers: &[Layer], layer: &Layer, gap_support_object: f64, is_top_contact: bool) -> Option<usize> {
        // sync gap with the object layer height
        let mut gap_synced: f32 = 0.;
        if is_top_contact {
            let mut lower_layer: Option<usize> = layer.lower_layer_id;
            let mut last_valid_gap_layer: Option<usize> = layer.lower_layer_id;
            while let Some(ll) = lower_layer {
                if !((gap_synced as f64) < gap_support_object) {
                    break;
                }
                last_valid_gap_layer = Some(ll);
                gap_synced = (gap_synced as f64 + layers[ll].height) as f32;
                lower_layer = layers[ll].lower_layer_id;
            }
            // maybe gap_synced is too large, find the nearest object layer (one layer above may be better)
            let mut last = last_valid_gap_layer.expect("sync_gap_with_object_layer: no layer below");
            if (gap_synced as f64 - layers[last].height - gap_support_object).abs() < (gap_synced as f64 - gap_support_object).abs() {
                // gap_synced -= last_valid_gap_layer->height;  (not read again)
                last = layers[last].upper_layer_id.expect("sync_gap_with_object_layer: no layer above");
            }
            // layer just below the last valid gap layer
            let mut lower_layer = Some(last);
            if layers[last].lower_layer_id.is_some() {
                lower_layer = layers[last].lower_layer_id;
            }
            lower_layer
        } else {
            let mut upper_layer: Option<usize> = layer.upper_layer_id;
            let mut last_valid_gap_layer: Option<usize> = layer.upper_layer_id;
            while let Some(ul) = upper_layer {
                if !((gap_synced as f64) < gap_support_object) {
                    break;
                }
                last_valid_gap_layer = Some(ul);
                gap_synced = (gap_synced as f64 + layers[ul].height) as f32;
                upper_layer = layers[ul].upper_layer_id;
            }
            // maybe gap_synced is too large, find the nearest object layer (one layer above may be better)
            let mut last = last_valid_gap_layer.expect("sync_gap_with_object_layer: no layer above");
            if (gap_synced as f64 - layers[last].height - gap_support_object).abs() < (gap_synced as f64 - gap_support_object).abs() {
                // gap_synced -= last_valid_gap_layer->height;  (not read again)
                last = layers[last].lower_layer_id.expect("sync_gap_with_object_layer: no layer below");
            }
            if gap_support_object > 0. {
                // layer just above the last valid gap layer
                upper_layer = Some(last);
                if layers[last].upper_layer_id.is_some() {
                    upper_layer = layers[last].upper_layer_id;
                }
            }
            upper_layer
        }
    }

    /// SupportMaterial.cpp:1781-1885
    /// C++: `static inline std::pair<SupportGeneratorLayer*, SupportGeneratorLayer*> new_contact_layer(const PrintConfig
    /// &print_config, const PrintObjectConfig &object_config, const SlicingParameters &slicing_params, const coordf_t
    /// support_layer_height_min, const Layer &layer, SupportGeneratorLayerStorage &layer_storage)`
    ///
    /// Allocate one, possibly two support contact layers.
    /// For "thick" overhangs, one support layer will be generated to support normal extrusions, the other to support the "thick" extrusions.
    #[allow(clippy::too_many_arguments)]
    pub fn new_contact_layer(
        object: &pf::PrintObject,
        print_config: &pf::PrintConfig,
        object_config: &pf::PrintObjectConfig,
        slicing_params: &SlicingParameters,
        support_layer_height_min: f64,
        layer: &Layer,
        layer_storage: &mut SupportGeneratorLayerStorage,
    ) -> (Option<usize>, Option<usize>) {
        let layers = object.layers();
        let print_z: f64;
        let mut bottom_z: f64;
        let mut height: f64;
        let mut print_z_mut: f64;
        let mut bridging_layer: Option<usize> = None;
        debug_assert!(layer.id() >= slicing_params.raft_layers());
        let layer_id: usize = layer.id() - slicing_params.raft_layers();
        let independent_support_layer_height = cfg_bool(print_config, "independent_support_layer_height");

        if layer_id == 0 {
            // This is a raft contact layer sitting directly on the print bed.
            debug_assert!(slicing_params.has_raft());
            print_z_mut = slicing_params.raft_contact_top_z;
            bottom_z = slicing_params.raft_interface_top_z;
            height = slicing_params.contact_raft_layer_height;
        } else if slicing_params.soluble_interface {
            // Align the contact surface height with a layer immediately below the supported layer.
            // Interface layer will be synchronized with the object.
            let lower_layer = &layers[layer.lower_layer_id.expect("new_contact_layer: no lower layer")];
            print_z_mut = layer.bottom_z();
            height = lower_layer.height;
            bottom_z = if layer_id == 1 {
                slicing_params.object_print_z_min
            } else {
                layers[lower_layer.lower_layer_id.expect("new_contact_layer: no layer two below")].print_z
            };
        } else {
            // BBS: need to consider adaptive layer heights
            if independent_support_layer_height {
                print_z_mut = layer.bottom_z() - slicing_params.gap_support_object;
                height = 0.;
            } else {
                let synced_layer = &layers[sync_gap_with_object_layer(layers, layer, slicing_params.gap_support_object, true).expect("new_contact_layer: no synced layer")];
                print_z_mut = synced_layer.print_z;
                height = synced_layer.height;
            }
            bottom_z = print_z_mut - height;
            // Ignore this contact area if it's too low.
            // Don't want to print a layer below the first layer height as it may not stick well.
            //FIXME there may be a need for a single layer support, then one may decide to print it either as a bottom contact or a top contact
            // and it may actually make sense to do it with a thinner layer than the first layer height.
            if print_z_mut < slicing_params.first_print_layer_height - EPSILON {
                // This contact layer is below the first layer height, therefore not printable. Don't support this surface.
                return (None, None);
            }
            let has_raft = slicing_params.raft_layers() > 1;
            let min_print_z: f64 = if has_raft { slicing_params.raft_contact_top_z } else { slicing_params.first_print_layer_height };
            if print_z_mut < min_print_z + support_layer_height_min {
                // Align the layer with the 1st layer height or the raft contact layer.
                // With raft active, any contact layer below the raft_contact_top_z will be brought to raft_contact_top_z to extend the raft area.
                print_z_mut = min_print_z;
                bottom_z = if has_raft { slicing_params.raft_interface_top_z } else { 0. };
                height = if has_raft { slicing_params.contact_raft_layer_height } else { min_print_z };
            } else {
                // Don't know the height of the top contact layer yet. The top contact layer is printed with a normal flow and
                // its height will be set adaptively later on.
            }

            // Contact layer will be printed with a normal flow, but
            // it will support layers printed with a bridging flow.
            if cfg_bool(object_config, "thick_bridges") && has_bridging_extrusions(layer) {
                let mut bridging_height: f64 = 0.;
                for region in layer.regions() {
                    let print_region = object.printing_region(region.region_id());
                    bridging_height += print_region_bridging_height_avg(&print_region.borrow(), print_config);
                }
                bridging_height /= layer.regions().len() as f64;
                // BBS: align bridging height
                let object_layer_height = cfg_float(object_config, "layer_height");
                if !independent_support_layer_height {
                    bridging_height = (bridging_height / object_layer_height - EPSILON).ceil() * object_layer_height;
                }
                let mut bridging_print_z: f64 = layer.print_z - bridging_height - slicing_params.gap_support_object;
                if bridging_print_z >= min_print_z {
                    // Not below the first layer height means this layer is printable.
                    if print_z_mut < min_print_z + support_layer_height_min {
                        // Align the layer with the 1st layer height or the raft contact layer.
                        bridging_print_z = min_print_z;
                    }
                    if bridging_print_z < print_z_mut - EPSILON {
                        // Allocate the new layer.
                        let idx = layer_storage.len();
                        let bl = layer_storage.allocate(SupporLayerType::SltTopContact);
                        bl.idx_object_layer_above = layer_id;
                        bl.print_z = bridging_print_z;
                        if bridging_print_z == slicing_params.first_print_layer_height {
                            bl.bottom_z = 0.;
                            bl.height = slicing_params.first_print_layer_height;
                        } else {
                            // BBS: if independent_support_layer_height is not enabled, the support layer_height should be the same as layer height.
                            // Note that for this case, adaptive layer height must be disabled.
                            bl.height = if independent_support_layer_height { 0. } else { object_layer_height };
                            // Don't know the height yet.
                            bl.bottom_z = bridging_print_z - bl.height;
                        }
                        bridging_layer = Some(idx);
                    }
                }
            }
        }
        print_z = print_z_mut;

        let idx = layer_storage.len();
        let new_layer = layer_storage.allocate(SupporLayerType::SltTopContact);
        new_layer.idx_object_layer_above = layer_id;
        new_layer.print_z = print_z;
        new_layer.bottom_z = bottom_z;
        new_layer.height = height;
        (Some(idx), bridging_layer)
    }

    /// SupportMaterial.cpp:1887-2036
    /// C++: `static inline void fill_contact_layer(SupportGeneratorLayer &new_layer, size_t layer_id, const
    /// SlicingParameters &slicing_params, const PrintObjectConfig &object_config, const SlicesMarginCache
    /// &slices_margin, const Polygons &overhang_polygons, const Polygons &contact_polygons, const Polygons
    /// &enforcer_polygons, const Polygons &lower_layer_polygons, const Flow &support_material_flow, float
    /// no_interface_offset)`
    #[allow(clippy::too_many_arguments)]
    pub fn fill_contact_layer(
        new_layer: &mut crate::support::support_layer::SupportGeneratorLayer,
        layer_id: usize,
        slicing_params: &SlicingParameters,
        object_config: &pf::PrintObjectConfig,
        slices_margin: &SlicesMarginCache,
        overhang_polygons: &Polygons,
        contact_polygons: &Polygons,
        enforcer_polygons: &Polygons,
        lower_layer_polygons: &Polygons,
        support_material_flow: &Flow,
        no_interface_offset: f32,
    ) {
        let grid_params = SupportGridParams::new(object_config, support_material_flow);

        let mut lower_layer_polygons_for_dense_interface_cache: Polygons = Vec::new();
        let mut lower_layer_polygons_for_dense_interface = || -> Polygons {
            if lower_layer_polygons_for_dense_interface_cache.is_empty() {
                lower_layer_polygons_for_dense_interface_cache =
                    //FIXME no_interface_offset * 0.6f offset is not quite correct, one shall derive it based on an angle thus depending on layer height.
                    cu::opening(lower_layer_polygons, no_interface_offset * 0.5f32, no_interface_offset * (0.6f32 + 0.5f32), SSOP_JT, SSOP_ML);
            }
            lower_layer_polygons_for_dense_interface_cache.clone()
        };

        // Stretch support islands into a grid, trim them.
        let support_grid_pattern = SupportGridPattern::new(contact_polygons, &slices_margin.polygons, &grid_params);
        // 1) Contact polygons will be projected down. To keep the interface and base layers from growing, return a contour a tiny bit smaller than the grid cells.
        new_layer.contact_polygons = Some(support_grid_pattern.extract_support(grid_params.expansion_to_propagate, true));
        // 2) infill polygons, expand them by half the extrusion width + a tiny bit of extra.
        let reduce_interfaces = cfg_int(object_config, "support_style") != SMS_SNUG && layer_id > 0 && !slicing_params.soluble_interface;
        if reduce_interfaces {
            // Reduce the amount of dense interfaces: Do not generate dense interfaces below overhangs with 60% overhang of the extrusions.
            let mut dense_interface_polygons: Polygons = cu::diff(overhang_polygons, &lower_layer_polygons_for_dense_interface(), ApplySafetyOffset::No);
            if !dense_interface_polygons.is_empty() {
                dense_interface_polygons = cu::diff(
                    // Regularize the contour.
                    &cu::expand(&dense_interface_polygons, no_interface_offset * 0.1f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
                    &slices_margin.polygons,
                    ApplySafetyOffset::No,
                );
                // Support islands, to be stretched into a grid.
                //FIXME The regularization of dense_interface_polygons above may stretch dense_interface_polygons outside of the contact polygons,
                // thus some dense interface areas may not get supported. Trim the excess with contact_polygons at the following line.
                // See for example GH #4874.
                let dense_interface_polygons_trimmed: Polygons =
                    cu::intersection(&dense_interface_polygons, new_layer.contact_polygons.as_ref().unwrap(), ApplySafetyOffset::No);
                // Stretch support islands into a grid, trim them.
                let support_grid_pattern = SupportGridPattern::new(&dense_interface_polygons_trimmed, &slices_margin.polygons, &grid_params);
                new_layer.polygons = support_grid_pattern.extract_support(grid_params.expansion_to_slice, false);
            }
        } else {
            new_layer.polygons = support_grid_pattern.extract_support(grid_params.expansion_to_slice, true);
        }

        if !enforcer_polygons.is_empty() && !slices_margin.all_polygons.is_empty() && layer_id > 0 {
            // Support enforcers used together with support enforcers. The support enforcers need to be handled separately from the rest of the support.

            let support_grid_pattern = SupportGridPattern::new(enforcer_polygons, &slices_margin.all_polygons, &grid_params);
            // 1) Contact polygons will be projected down. To keep the interface and base layers from growing, return a contour a tiny bit smaller than the grid cells.
            new_layer.enforcer_polygons = Some(support_grid_pattern.extract_support(grid_params.expansion_to_propagate, true));
            let mut new_polygons: Polygons = Vec::new();
            let needs_union = !new_layer.polygons.is_empty();
            if reduce_interfaces {
                // 2) infill polygons, expand them by half the extrusion width + a tiny bit of extra.
                // Reduce the amount of dense interfaces: Do not generate dense interfaces below overhangs with 60% overhang of the extrusions.
                let mut dense_interface_polygons: Polygons = cu::diff(enforcer_polygons, &lower_layer_polygons_for_dense_interface(), ApplySafetyOffset::No);
                if !dense_interface_polygons.is_empty() {
                    dense_interface_polygons = cu::diff(
                        // Regularize the contour.
                        &cu::expand(&dense_interface_polygons, no_interface_offset * 0.1f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
                        &slices_margin.all_polygons,
                        ApplySafetyOffset::No,
                    );
                    // Support islands, to be stretched into a grid.
                    //FIXME The regularization of dense_interface_polygons above may stretch dense_interface_polygons outside of the contact polygons,
                    // thus some dense interface areas may not get supported. Trim the excess with contact_polygons at the following line.
                    // See for example GH #4874.
                    let dense_interface_polygons_trimmed: Polygons =
                        cu::intersection(&dense_interface_polygons, new_layer.enforcer_polygons.as_ref().unwrap(), ApplySafetyOffset::No);
                    let support_grid_pattern = SupportGridPattern::new(&dense_interface_polygons_trimmed, &slices_margin.all_polygons, &grid_params);
                    // Extend the polygons to extrude with the contact polygons of support enforcers.
                    new_polygons = support_grid_pattern.extract_support(grid_params.expansion_to_slice, false);
                }
            } else {
                new_polygons = support_grid_pattern.extract_support(grid_params.expansion_to_slice, true);
            }
            new_layer.polygons.extend(new_polygons);
            if needs_union {
                new_layer.polygons = cu::union_(&new_layer.polygons);
            }
        }

        // Even after the contact layer was expanded into a grid, some of the contact islands may be too tiny to be extruded.
        // Remove those tiny islands from new_layer.polygons and new_layer.contact_polygons.

        // Store the overhang polygons.
        // The overhang polygons are used in the path generator for planning of the contact loops.
        // if (this->has_contact_loops()). Compared to "polygons", "overhang_polygons" are snug.
        new_layer.overhang_polygons = Some(overhang_polygons.clone());
        if !enforcer_polygons.is_empty() {
            new_layer.enforcer_polygons = Some(enforcer_polygons.clone());
        }
    }

    // ------------------------------------------------------------------------------------------------
    // SupportMaterial.cpp:2084-2150
    // ------------------------------------------------------------------------------------------------

    /// SupportMaterial.cpp:2084-2133 `struct OverhangCluster`
    ///
    /// An `ExPolygon*` of `layer_overhangs` is the pair (layer, index in `overhangs_per_layers[layer]`), which is
    /// also the order of the addresses within one layer.
    #[derive(Debug, Clone)]
    pub struct OverhangCluster {
        pub layer_overhangs: BTreeMap<i32, Vec<(usize, usize)>>,
        pub merged_overhangs_dilated: ExPolygons,
        pub min_layer: i32,
        pub max_layer: i32,
        pub offset_scaled: f64,
        pub is_cantilever: bool,
        pub is_sharp_tail: bool,
        pub is_small_overhang: bool,
    }

    impl OverhangCluster {
        /// SupportMaterial.cpp:2094-2097
        /// C++: `OverhangCluster(ExPolygon* overhang, int layer_nr, coordf_t offset_scaled)`
        pub fn new(overhang: (usize, usize), overhang_new: &ExPolygon, layer_nr: i32, offset_scaled: f64) -> Self {
            let mut this = OverhangCluster {
                layer_overhangs: BTreeMap::new(),
                merged_overhangs_dilated: Vec::new(),
                min_layer: 1e7 as i32,
                max_layer: 0,
                offset_scaled,
                is_cantilever: false,
                is_sharp_tail: false,
                is_small_overhang: false,
            };
            this.insert(overhang, overhang_new, layer_nr);
            this
        }

        /// SupportMaterial.cpp:2099-2111
        /// C++: `void insert(ExPolygon* overhang_new, int layer_nr)`
        pub fn insert(&mut self, overhang: (usize, usize), overhang_new: &ExPolygon, layer_nr: i32) {
            self.layer_overhangs.entry(layer_nr).or_default().push(overhang);
            let overhang_dilated: ExPolygons = if self.offset_scaled > EPSILON {
                cu_expolygon::offset_ex(overhang_new, self.offset_scaled as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT)
            } else {
                vec![overhang_new.clone()]
            };
            if !overhang_dilated.is_empty() {
                self.merged_overhangs_dilated = cu_expolygon::union_ex_2(&self.merged_overhangs_dilated, &overhang_dilated);
            }
            self.min_layer = self.min_layer.min(layer_nr);
            self.max_layer = self.max_layer.max(layer_nr);
        }

        /// SupportMaterial.cpp:2113-2115
        /// C++: `int height()`
        pub fn height(&self) -> i32 {
            self.max_layer - self.min_layer + 1
        }

        /// SupportMaterial.cpp:2117-2132
        /// C++: `bool intersects(const ExPolygon& overhang_new, int layer_nr)`
        pub fn intersects(&self, overhang_new: &ExPolygon, layer_nr: i32) -> bool {
            if layer_nr < 1 {
                return false;
            }

            if layer_nr < self.min_layer - 1 || layer_nr > self.max_layer + 1 {
                return false;
            }
            let overhang_dilated: ExPolygons = cu_expolygon::offset_ex(overhang_new, self.offset_scaled as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);
            cu_expolygon::overlaps_expolygons(&overhang_dilated, &self.merged_overhangs_dilated)
        }
    }

    /// SupportMaterial.cpp:2135-2150
    /// C++: `static OverhangCluster* add_overhang(std::vector<OverhangCluster>& clusters, ExPolygon* overhang, int layer_nr, coordf_t offset_scaled)`
    ///
    /// The cluster it returns is its index in `clusters`.
    pub fn add_overhang(clusters: &mut Vec<OverhangCluster>, overhang: (usize, usize), overhang_new: &ExPolygon, layer_nr: i32, offset_scaled: f64) -> usize {
        let mut cluster: Option<usize> = None;
        for i in 0..clusters.len() {
            let cluster_i = &mut clusters[i];
            if cluster_i.intersects(overhang_new, layer_nr) {
                cluster_i.insert(overhang, overhang_new, layer_nr);
                cluster = Some(i);
                break;
            }
        }
        match cluster {
            Some(i) => i,
            None => {
                clusters.push(OverhangCluster::new(overhang, overhang_new, layer_nr, offset_scaled));
                clusters.len() - 1
            }
        }
    }

    // ------------------------------------------------------------------------------------------------
    // SupportMaterial.hpp:56-133, SupportMaterial.cpp:334-341 `class PrintObjectSupportMaterial`
    // ------------------------------------------------------------------------------------------------

    /// SupportMaterial.hpp:56-133 `class PrintObjectSupportMaterial`
    ///
    /// This class manages raft and supports for a single PrintObject.
    ///
    /// The C++ keeps pointers to the object, to the configuration of its Print and to its own configuration;
    /// here the Print is borrowed, the configuration of the object is copied (generate() takes the object
    /// mutably) and the stages take the object as their parameter, as the C++ ones do.
    pub struct PrintObjectSupportMaterial<'a> {
        /// `object->print()`: `m_print_config` is its `config()`.
        pub m_print: &'a pf::Print,
        /// `const PrintObjectConfig *m_object_config`
        pub m_object_config: pf::PrintObjectConfig,
        // Pre-calculated parameters shared between the object slicer and the support generator,
        // carrying information on a raft, 1st layer height, 1st object layer height, gap between the raft and object etc.
        pub m_slicing_params: SlicingParameters,
        // Various precomputed support parameters to be shared with external functions.
        pub m_support_params: SupportParameters,
        /// Layer.hpp:172-174 `mutable ExPolygons sharp_tails; mutable ExPolygons cantilevers; mutable std::vector<float>
        /// sharp_tails_height;` of every object layer, by its index. The C++ never clears them: a caller that
        /// generates the supports of an object again hands the tables of the earlier run in.
        pub layer_sharp_tails: Vec<ExPolygons>,
        pub layer_sharp_tails_height: Vec<Vec<f32>>,
        pub layer_cantilevers: Vec<ExPolygons>,
        /// `PrintObject::project_and_append_custom_facets` (see `SupportAnnotations::new`).
        pub custom_facets_projector: Option<&'a dyn CustomFacetsProjector>,
    }

    impl<'a> PrintObjectSupportMaterial<'a> {
        /// SupportMaterial.cpp:334-341
        /// C++: `PrintObjectSupportMaterial(const PrintObject *object, const SlicingParameters &slicing_params)`
        pub fn new(object: &pf::PrintObject, print: &'a pf::Print, slicing_params: &SlicingParameters) -> crate::Result<Self> {
            Ok(PrintObjectSupportMaterial {
                m_print: print,
                m_object_config: object.config().clone(),
                m_slicing_params: slicing_params.clone(),
                m_support_params: SupportParameters::new(object, print)?,
                layer_sharp_tails: Vec::new(),
                layer_sharp_tails_height: Vec::new(),
                layer_cantilevers: Vec::new(),
                custom_facets_projector: None,
            })
        }

        /// SupportMaterial.hpp:62  Is raft enabled?
        pub fn has_raft(&self) -> bool {
            self.m_slicing_params.has_raft()
        }
        /// SupportMaterial.hpp:64  Has any support?
        pub fn has_support(&self) -> bool {
            cfg_bool(&self.m_object_config, "enable_support") || cfg_int(&self.m_object_config, "enforce_support_layers") != 0
        }
        /// SupportMaterial.hpp:65
        pub fn build_plate_only(&self) -> bool {
            self.has_support() && cfg_bool(&self.m_object_config, "support_on_build_plate_only")
        }
        /// SupportMaterial.hpp:67  BBS
        pub fn synchronize_layers(&self) -> bool {
            /*m_slicing_params.soluble_interface && */
            !cfg_bool(self.m_print.config(), "independent_support_layer_height")
        }
        /// SupportMaterial.hpp:68
        pub fn has_contact_loops(&self) -> bool {
            cfg_bool(&self.m_object_config, "support_interface_loop_pattern")
        }

        /// `object.print()->canceled()`
        fn canceled(&self) -> bool {
            self.m_print.base.base.canceled()
        }

        /// The tables of `Layer::sharp_tails` and its companions, one entry per object layer.
        fn resize_layer_tables(&mut self, layer_count: usize) {
            if self.layer_sharp_tails.len() < layer_count {
                self.layer_sharp_tails.resize(layer_count, Vec::new());
            }
            if self.layer_sharp_tails_height.len() < layer_count {
                self.layer_sharp_tails_height.resize(layer_count, Vec::new());
            }
            if self.layer_cantilevers.len() < layer_count {
                self.layer_cantilevers.resize(layer_count, Vec::new());
            }
        }

        /// SupportMaterial.cpp:1344-1369
        /// C++: `std::vector<Polygons> PrintObjectSupportMaterial::buildplate_covered(const PrintObject &object) const`
        pub fn buildplate_covered(&self, object: &pf::PrintObject) -> Vec<Polygons> {
            // Build support on a build plate only? If so, then collect and union all the surfaces below the current layer.
            // Unfortunately this is an inherently serial process.
            let buildplate_only = self.build_plate_only();
            let mut buildplate_covered: Vec<Polygons> = Vec::new();
            if buildplate_only {
                buildplate_covered = vec![Vec::new(); object.layers().len()];
                //FIXME prefix sum algorithm, parallelize it! Parallelization will also likely be more numerically stable.
                for layer_id in 1..object.layers().len() {
                    let lower_layer = &object.layers()[layer_id - 1];
                    // Merge the new slices with the preceding slices.
                    // Apply the safety offset to the newly added polygons, so they will connect
                    // with the polygons collected before,
                    // but don't apply the safety offset during the union operation as it would
                    // inflate the polygons over and over.
                    let mut covered: Polygons = buildplate_covered[layer_id - 1].clone();
                    covered.extend(cu::offset_expolygons(&lower_layer.lslices, scale_(0.01) as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT));
                    buildplate_covered[layer_id] = cu::union_(&covered);
                }
            }
            buildplate_covered
        }

        /// SupportMaterial.cpp:2152-2430
        /// C++: `SupportGeneratorLayersPtr PrintObjectSupportMaterial::top_contact_layers(const PrintObject &object, const
        /// std::vector<Polygons> &buildplate_covered, SupportGeneratorLayerStorage &layer_storage) const`
        ///
        /// Generate top contact layers supporting overhangs.
        /// For a soluble interface material synchronize the layer heights with the object, otherwise leave the layer height undefined.
        /// If supports over bed surface only are requested, don't generate contact layers over an object.
        ///
        /// The `tbb::parallel_for` over the layers runs in their order (a layer writes its own `sharp_tails` and
        /// `cantilevers` and reads the slices of the layer below).
        pub fn top_contact_layers(
            &mut self,
            object: &pf::PrintObject,
            buildplate_covered: &[Polygons],
            layer_storage: &mut SupportGeneratorLayerStorage,
        ) -> crate::Result<SupportGeneratorLayersPtr> {
            use crate::print_config::config_def::{is_auto, multi_nozzle_diameter_mode_enabled, primary_nozzle_diameter};
            self.resize_layer_tables(object.layer_count());
            let support_type = cfg_int(&self.m_object_config, "support_type");
            // BBS: tree support is selected so normal supports need not be generated.
            // Note we still need to go through the following steps if support is disabled but raft is enabled.
            if cfg_bool(&self.m_object_config, "enable_support")
                && (support_type != SupportType::StNormalAuto as i32 && support_type != SupportType::StNormal as i32)
            {
                return Ok(Vec::new());
            }

            // Slice support enforcers / support blockers.
            let annotations = SupportAnnotations::new(object, self.m_print, buildplate_covered, self.custom_facets_projector)?;

            // Output layers, sorted by top Z.
            let mut contact_out: SupportGeneratorLayersPtr;

            // Determine top contact areas.
            // If generating raft only (no support), only calculate top contact areas for the 0th layer.
            // If having a raft, start with 0th layer, otherwise with 1st layer.
            // Note that layer_id < layer->id when raft_layers > 0 as the layer->id incorporates the raft layers.
            // So layer_id == 0 means first object layer and layer->id == 0 means first print layer if there are no explicit raft layers.
            let num_layers: usize = if self.has_support() { object.layer_count() } else { 1 };
            // For each overhang layer, two supporting layers may be generated: One for the overhangs extruded with a bridging flow,
            // and the other for the overhangs extruded with a normal flow.
            contact_out = vec![NULL_LAYER; num_layers * 2];

            let print_config = self.m_print.config();

            let mut overhangs_per_layers: Vec<ExPolygons> = vec![Vec::new(); num_layers];
            let layer_id_start: usize = if self.has_raft() { 0 } else { 1 };
            // main part of overhang detection can be parallel
            for layer_id in layer_id_start..num_layers {
                let layer = &object.layers()[layer_id];
                let lower_layer_polygons: Polygons = if layer_id == 0 { Vec::new() } else { to_polygons(&object.layers()[layer_id - 1].lslices) };

                overhangs_per_layers[layer_id] = detect_overhangs(
                    object,
                    self.m_print,
                    layer,
                    layer_id,
                    &lower_layer_polygons,
                    print_config,
                    &self.m_object_config,
                    &annotations,
                    self.m_support_params.gap_xy,
                    &mut self.layer_sharp_tails[layer_id],
                    &mut self.layer_sharp_tails_height[layer_id],
                    &mut self.layer_cantilevers[layer_id],
                )?;

                if self.canceled() {
                    break;
                }
            }

            if self.canceled() {
                return Ok(Vec::new());
            }

            // check if the sharp tails should be extended higher
            let detect_first_sharp_tail_only = false;
            let full_print_config = self.m_print.full_print_config();
            let extrusion_width: f64 = Flow::extrusion_width_for_nozzle(
                &ConfigOptionFloat::with_value(cfg_float(&self.m_object_config, "line_width")),
                primary_nozzle_diameter(full_print_config) as f32,
                support_nozzle_diameter(self.m_print, cfg_int(&self.m_object_config, "support_filament") as u32),
                multi_nozzle_diameter_mode_enabled(full_print_config),
            );
            let extrusion_width_scaled: f64 = scale_(extrusion_width);
            if is_auto(support_type_from_i32(support_type)) && G_CONFIG_SUPPORT_SHARP_TAILS && !detect_first_sharp_tail_only {
                for layer_nr in layer_id_start..num_layers {
                    if self.canceled() {
                        break;
                    }

                    let layer = object.get_layer(layer_nr as i32);
                    let Some(lower_layer_idx) = layer.lower_layer_id else {
                        continue;
                    };
                    let lower_layer = &object.layers()[lower_layer_idx];

                    // BBS detect sharp tail
                    let lower_layer_sharptails: ExPolygons = self.layer_sharp_tails[lower_layer_idx].clone();
                    let lower_layer_sharptails_height: Vec<f32> = self.layer_sharp_tails_height[lower_layer_idx].clone();
                    for expoly in &layer.lslices {
                        let mut is_sharp_tail = false;
                        let mut accum_height: f32 = layer.height as f32;
                        #[allow(clippy::never_loop)]
                        loop {
                            // 2. something below
                            // check whether this is above a sharp tail region.

                            // 2.1 If no sharp tail below, this is considered as common region.
                            let supported_by_lower: ExPolygons = cx::intersection_ex_ee(std::slice::from_ref(expoly), &lower_layer_sharptails);
                            if supported_by_lower.is_empty() {
                                is_sharp_tail = false;
                                break;
                            }

                            // 2.2 If sharp tail below, check whether it support this region enough.
                            let bbox = get_extents_expolygons(&supported_by_lower);
                            let size = bbox_size(&bbox);
                            if size.x as f64 > length_thresh_well_supported() && size.y as f64 > length_thresh_well_supported() {
                                is_sharp_tail = false;
                                break;
                            }

                            // 2.3 check whether sharp tail exceed the max height
                            for i in 0..lower_layer_sharptails_height.len() {
                                // `lower_layer_sharptails[i]`: the two vectors have the same length unless a blocker
                                // took a sharp tail away (the C++ then reads past the end of the shorter one).
                                let Some(tail) = lower_layer_sharptails.get(i) else {
                                    break;
                                };
                                if cu_expolygon::overlaps(tail, expoly) {
                                    accum_height += lower_layer_sharptails_height[i];
                                    break;
                                }
                            }
                            if accum_height as f64 > SHARP_TAIL_MAX_SUPPORT_HEIGHT {
                                is_sharp_tail = false;
                                break;
                            }

                            // 2.4 if the area grows fast than threshold, it get connected to other part or
                            // it has a sharp slop and will be auto supported.
                            let new_overhang_expolys: ExPolygons = cu::diff_ex_ee(std::slice::from_ref(expoly), &lower_layer_sharptails, ApplySafetyOffset::No);
                            let a = bbox_size(&get_extents_expolygons(&new_overhang_expolys));
                            let b = bbox_size(&get_extents_expolygons(&lower_layer_sharptails));
                            let size_diff = Point::new(a.x - b.x, a.y - b.y);
                            // `Point(scale_(5), scale_(5))` rounds (Point.hpp:181)
                            let five = scale_(5.).round_ties_even() as i64;
                            if (size_diff.x > five && size_diff.y > five)
                                || !cu::offset_ex_expolygons(
                                    &new_overhang_expolys,
                                    (-5.0 * extrusion_width_scaled) as f32,
                                    cu::DEFAULT_JOIN_TYPE,
                                    cu::DEFAULT_MITER_LIMIT,
                                )
                                .is_empty()
                            {
                                is_sharp_tail = false;
                                break;
                            }

                            // 2.5 mark the expoly as sharptail
                            is_sharp_tail = true;
                            break;
                        }

                        if is_sharp_tail {
                            let overhang: ExPolygons = cu::diff_ex_ee(std::slice::from_ref(expoly), &lower_layer.lslices, ApplySafetyOffset::No);
                            self.layer_sharp_tails[layer_nr].push(expoly.clone());
                            self.layer_sharp_tails_height[layer_nr].push(accum_height);
                            overhangs_per_layers[layer_nr].extend(overhang);
                        }
                    }
                }
            }

            if self.canceled() {
                return Ok(Vec::new());
            }

            // BBS group overhang clusters
            let config_remove_small_overhangs = cfg_bool(&self.m_object_config, "support_remove_small_overhang");
            if config_remove_small_overhangs {
                let mut clusters: Vec<OverhangCluster> = Vec::new();
                let fw_scaled: f64 = scale_(Flow::extrusion_width_for_nozzle(
                    &ConfigOptionFloat::with_value(cfg_float(&self.m_object_config, "line_width")),
                    primary_nozzle_diameter(full_print_config) as f32,
                    support_nozzle_diameter(self.m_print, cfg_int(&self.m_object_config, "support_filament") as u32),
                    multi_nozzle_diameter_mode_enabled(full_print_config),
                ));
                let mut removed_overhang: BTreeSet<(usize, usize)> = BTreeSet::new();

                for layer_id in layer_id_start..num_layers {
                    for poly_idx in 0..overhangs_per_layers[layer_id].len() {
                        let overhang = &overhangs_per_layers[layer_id][poly_idx];
                        let cluster = add_overhang(&mut clusters, (layer_id, poly_idx), overhang, layer_id as i32, fw_scaled);
                        if cu_expolygon::overlaps_expolygons(std::slice::from_ref(overhang), &self.layer_cantilevers[layer_id]) {
                            clusters[cluster].is_cantilever = true;
                        }
                    }
                }

                for cluster in clusters.iter_mut() {
                    // 3. check whether the small overhang is sharp tail
                    cluster.is_sharp_tail = false;
                    // `for (size_t layer_id = cluster.min_layer; layer_id <= cluster.max_layer; layer_id++)`
                    let mut layer_id = cluster.min_layer as usize;
                    while layer_id <= cluster.max_layer as usize {
                        if cu_expolygon::overlaps_expolygons(&self.layer_sharp_tails[layer_id], &cluster.merged_overhangs_dilated) {
                            cluster.is_sharp_tail = true;
                            break;
                        }
                        layer_id += 1;
                    }

                    if !cluster.is_sharp_tail && !cluster.is_cantilever {
                        // 2. check overhang cluster size is small
                        cluster.is_small_overhang = false;
                        let erode1 = cu::offset_ex_expolygons(&cluster.merged_overhangs_dilated, (-1.0 * fw_scaled) as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);
                        let bbox_sz = bbox_size(&get_extents_expolygons(&erode1));
                        if (bbox_sz.x as f64) < 2. * fw_scaled || (bbox_sz.y as f64) < 2. * fw_scaled {
                            cluster.is_small_overhang = true;
                        }
                    }

                    // 5. remove small overhangs
                    if cluster.is_small_overhang {
                        for overhangs in cluster.layer_overhangs.values() {
                            for poly in overhangs {
                                removed_overhang.insert(*poly);
                            }
                        }
                    }
                }

                for layer_id in layer_id_start..num_layers {
                    let layer_overhangs = &mut overhangs_per_layers[layer_id];
                    if layer_overhangs.is_empty() {
                        continue;
                    }

                    for poly_idx in 0..layer_overhangs.len() {
                        if removed_overhang.contains(&(layer_id, poly_idx)) {
                            // overhang->clear();
                            layer_overhangs[poly_idx].contour.points.clear();
                            layer_overhangs[poly_idx].holes.clear();
                        }
                    }
                }
            }

            if self.canceled() {
                return Ok(Vec::new());
            }

            for layer_id in layer_id_start..num_layers {
                let layer = &object.layers()[layer_id];
                let mut overhang_polygons: Polygons = to_polygons(&overhangs_per_layers[layer_id]);
                let lower_layer_polygons: Polygons = if layer_id == 0 { Vec::new() } else { to_polygons(&object.layers()[layer_id - 1].lslices) };
                let mut slices_margin = SlicesMarginCache::default();

                let (contact_polygons, enforcer_polygons, no_interface_offset) = detect_contacts(
                    object,
                    self.m_print,
                    layer,
                    layer_id,
                    &mut overhang_polygons,
                    &lower_layer_polygons,
                    print_config,
                    &self.m_object_config,
                    &annotations,
                    &mut slices_margin,
                    self.m_support_params.gap_xy,
                )?;

                // Now apply the contact areas to the layer where they need to be made.
                if !contact_polygons.is_empty() || !overhang_polygons.is_empty() {
                    // Allocate the two empty layers.
                    let (new_layer, bridging_layer) = new_contact_layer(
                        object,
                        print_config,
                        &self.m_object_config,
                        &self.m_slicing_params,
                        self.m_support_params.support_layer_height_min,
                        layer,
                        layer_storage,
                    );
                    if let Some(new_layer) = new_layer {
                        // Fill the non-bridging layer with polygons.
                        fill_contact_layer(
                            &mut layer_storage[new_layer],
                            layer_id,
                            &self.m_slicing_params,
                            &self.m_object_config,
                            &slices_margin,
                            &overhang_polygons,
                            &contact_polygons,
                            &enforcer_polygons,
                            &lower_layer_polygons,
                            &self.m_support_params.support_material_flow,
                            no_interface_offset as f32,
                        );
                        // Insert new layer even if there is no interface generated: Likely the support angle is not steep enough to require dense interface,
                        // however generating a sparse support will be useful for the object stability.
                        // if (! new_layer->polygons.empty())
                        contact_out[layer_id * 2] = new_layer;
                        if let Some(bridging_layer) = bridging_layer {
                            let (polygons, contact, overhang, enforcer) = {
                                let l = &layer_storage[new_layer];
                                (l.polygons.clone(), l.contact_polygons.clone(), l.overhang_polygons.clone(), l.enforcer_polygons.clone())
                            };
                            let bl = &mut layer_storage[bridging_layer];
                            bl.polygons = polygons;
                            bl.contact_polygons = contact;
                            bl.overhang_polygons = overhang;
                            if enforcer.is_some() {
                                bl.enforcer_polygons = enforcer;
                            }
                            contact_out[layer_id * 2 + 1] = bridging_layer;
                        }
                    }
                }
            }

            // Compress contact_out, remove the nullptr items.
            contact_out.retain(|&l| l != NULL_LAYER);

            // Merge close contact layers conservatively: If two layers are closer than the minimum allowed print layer height (the min_layer_height parameter),
            // the top contact layer is merged into the bottom contact layer.
            merge_contact_layers(&self.m_slicing_params, self.m_support_params.support_layer_height_min, &mut contact_out, layer_storage);

            Ok(contact_out)
        }
    }

    // ------------------------------------------------------------------------------------------------
    // SupportMaterial.cpp:2432-2648
    // ------------------------------------------------------------------------------------------------

    /// SupportMaterial.cpp:2432-2568
    /// C++: `static inline SupportGeneratorLayer* detect_bottom_contacts(const SlicingParameters &slicing_params, const
    /// SupportParameters &support_params, const PrintObject &object, const Layer &layer, const
    /// SupportGeneratorLayersPtr &top_contacts, size_t contact_idx, SupportGeneratorLayerStorage &layer_storage,
    /// std::vector<Polygons> &layer_support_areas, const Polygons &supports_projected)`
    ///
    /// Find the bottom contact layers above the top surfaces of this layer.
    /// `print_config` is `object.print()->config()`. `contact_idx` is the `int` the caller hands over (-1 when no
    /// top contact is left; the C++ converts it to `size_t` and back to `int` for the `std::max`).
    #[allow(clippy::too_many_arguments)]
    pub fn detect_bottom_contacts(
        slicing_params: &SlicingParameters,
        support_params: &SupportParameters,
        object: &pf::PrintObject,
        print_config: &pf::PrintConfig,
        layer: &Layer,
        // Existing top contact layers, to which this newly created bottom contact layer will be snapped to guarantee a minimum layer height.
        top_contacts: &SupportGeneratorLayersPtr,
        // First top contact layer index overlapping with this new bottom interface layer.
        contact_idx: i32,
        // To allocate a new layer from.
        layer_storage: &mut SupportGeneratorLayerStorage,
        // To trim the support areas above this bottom interface layer with this newly created bottom interface layer.
        layer_support_areas: &mut [Polygons],
        // Support areas projected from top to bottom, starting with top support interfaces.
        supports_projected: &Polygons,
    ) -> Option<usize> {
        let top: Polygons = super::collect_region_slices_by_type(layer, SurfaceType::Top);

        // Now find whether any projection of the contact surfaces above layer.print_z not yet supported by any
        // top surfaces above layer.print_z falls onto this top surface.
        // Touching are the contact surfaces supported exclusively by this top surfaces.
        // Don't use a safety offset as it has been applied during insertion of polygons.
        if top.is_empty() {
            return None;
        }

        let mut touching: Polygons = cu::intersection(&top, supports_projected, ApplySafetyOffset::No);
        if touching.is_empty() {
            return None;
        }

        debug_assert!(layer.id() >= slicing_params.raft_layers());
        let layer_id: usize = layer.id() - slicing_params.raft_layers();
        let layers = object.layers();

        // Allocate a new bottom contact layer.
        let layer_new_idx = layer_storage.len();
        layer_storage.allocate_unguarded(SupporLayerType::SltBottomContact);
        // Grow top surfaces so that interface and support generation are generated
        // with some spacing from object - it looks we don't need the actual
        // top shapes so this can be done here
        let height: f64;
        let print_z: f64;
        if cfg_bool(print_config, "independent_support_layer_height") {
            let upper_layer = layer.upper_layer_id.map(|i| &layers[i]);
            // If the layer is extruded with no bridging flow, support just the normal extrusions.
            height = if slicing_params.soluble_interface {
                // Align the interface layer with the object's layer height.
                upper_layer.expect("detect_bottom_contacts: no upper layer").height
            } else {
                // Place a bridge flow interface layer or the normal flow interface layer over the top surface.
                // (`float Flow::height()`)
                support_params.support_material_bottom_interface_flow.height() as f32 as f64
            };
            print_z = if slicing_params.soluble_interface {
                upper_layer.expect("detect_bottom_contacts: no upper layer").print_z
            } else {
                layer.print_z + height + slicing_params.gap_object_support
            };
        } else {
            let upper_layer = &layers[sync_gap_with_object_layer(layers, layer, slicing_params.gap_object_support, false).expect("detect_bottom_contacts: no synced layer")];
            height = upper_layer.height;
            print_z = upper_layer.print_z;
        }
        let polygons = cu::expand(&touching, support_params.support_material_flow.scaled_width() as f32, SSOP_JT, SSOP_ML);
        {
            let layer_new = &mut layer_storage[layer_new_idx];
            layer_new.height = height;
            layer_new.print_z = print_z;
            layer_new.bottom_z = layer.print_z;
            layer_new.idx_object_layer_below = layer_id;
            layer_new.bridging = !slicing_params.soluble_interface && cfg_bool(object.config(), "thick_bridges");
            //FIXME how much to inflate the bottom surface, as it is being extruded with a bridging flow? The following line uses a normal flow.
            layer_new.polygons = polygons;
        }

        if !slicing_params.soluble_interface {
            // Walk the top surfaces, snap the top of the new bottom surface to the closest top of the top surface,
            // so there will be no support surfaces generated with thickness lower than m_support_layer_height_min.
            let mut top_idx: usize = contact_idx.max(0) as usize;
            while top_idx < top_contacts.len()
                && layer_storage[top_contacts[top_idx]].print_z < layer_storage[layer_new_idx].print_z + support_params.support_layer_height_min + EPSILON
            {
                let top_print_z = layer_storage[top_contacts[top_idx]].print_z;
                let layer_new = &mut layer_storage[layer_new_idx];
                if top_print_z > layer_new.print_z - support_params.support_layer_height_min - EPSILON {
                    // A top layer has been found, which is close to the new bottom layer.
                    let diff: f64 = layer_new.print_z - top_print_z;
                    debug_assert!(diff.abs() <= support_params.support_layer_height_min + EPSILON);
                    if diff > 0. {
                        if layer_new.height - diff > support_params.support_layer_height_min {
                            // The top contact layer is below this layer. Make the bridging layer thinner to align with the existing top layer.
                            layer_new.print_z = top_print_z;
                            layer_new.height -= diff;
                        } else {
                            // BBS: The trimmed layer height is smaller than support_layer_height_min. Walk to the next top contact layer.
                            top_idx += 1;
                            continue;
                        }
                    } else {
                        // The top contact layer is above this layer. One may either make this layer thicker or thinner.
                        // By making the layer thicker, one will decrease the number of discrete layers with the price of extruding a bit too thick bridges.
                        // By making the layer thinner, one adds one more discrete layer.
                        layer_new.print_z = top_print_z;
                        layer_new.height -= diff;
                    }
                    break;
                }
                top_idx += 1;
            }
        }

        // Trim the already created base layers above the current layer intersecting with the new bottom contacts layer.
        //FIXME Maybe this is no more needed, as the overlapping base layers are trimmed by the bottom layers at the final stage?
        touching = cu::expand(&touching, scaled_epsilon() as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);
        let layer_new_print_z = layer_storage[layer_new_idx].print_z;
        // `for (int layer_id_above = layer_id + 1; layer_id_above < int(object.total_layer_count()); ++ layer_id_above)`
        let mut layer_id_above: i32 = layer_id as i32 + 1;
        while layer_id_above < object.total_layer_count() as i32 {
            let layer_above = &layers[layer_id_above as usize];
            if layer_above.print_z > layer_new_print_z - EPSILON {
                break;
            }
            let above = &mut layer_support_areas[layer_id_above as usize];
            if !above.is_empty() {
                *above = cu::diff(above, &touching, ApplySafetyOffset::No);
            }
            layer_id_above += 1;
        }

        Some(layer_new_idx)
    }

    /// SupportMaterial.cpp:2570-2648
    /// C++: `static inline std::pair<Polygons, Polygons> project_support_to_grid(const Layer &layer, const
    /// SupportGridParams &grid_params, const Polygons &overhangs, Polygons *layer_buildplate_covered)`
    ///
    /// Returns polygons to print + polygons to propagate downwards.
    /// Called twice: First for normal supports, possibly trimmed by "on build plate only", second for support enforcers not trimmed by "on build plate only".
    ///
    /// The two tasks of the inner `tbb::task_group` write one member of the pair each.
    pub fn project_support_to_grid(
        layer: &Layer,
        grid_params: &SupportGridParams,
        overhangs: &Polygons,
        layer_buildplate_covered: Option<&mut Polygons>,
    ) -> (Polygons, Polygons) {
        // Remove the areas that touched from the projection that will continue on next, lower, top surfaces.
        let trimming: Polygons = match layer_buildplate_covered {
            Some(covered) => std::mem::take(covered),
            None => cu::offset_expolygons(&layer.lslices, scaled_epsilon() as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
        };
        let mut overhangs_projection: Polygons = cu::diff(overhangs, &trimming, ApplySafetyOffset::No);

        crate::geometry::polygon::remove_sticks_polygons(&mut overhangs_projection);
        crate::geometry::polygon::remove_degenerate(&mut overhangs_projection);

        let support_grid_pattern = SupportGridPattern::new(&overhangs_projection, &trimming, grid_params);

        // 1) Cache the slice of a support volume. The support volume is expanded by 1/2 of support material flow spacing
        // to allow a placement of suppot zig-zag snake along the grid lines.
        let first = support_grid_pattern.extract_support(grid_params.expansion_to_slice, true);

        // 2) Support polygons will be projected down. To keep the interface and base layers from growing, return a contour a tiny bit smaller than the grid cells.
        let second = support_grid_pattern.extract_support(grid_params.expansion_to_propagate, true);

        (first, second)
    }

    impl<'a> PrintObjectSupportMaterial<'a> {
        /// SupportMaterial.cpp:2650-2793
        /// C++: `SupportGeneratorLayersPtr PrintObjectSupportMaterial::bottom_contact_layers_and_layer_support_areas(const
        /// PrintObject &object, const SupportGeneratorLayersPtr &top_contacts, std::vector<Polygons>
        /// &buildplate_covered, SupportGeneratorLayerStorage &layer_storage, std::vector<Polygons>
        /// &layer_support_areas) const`
        ///
        /// Generate bottom contact layers supporting the top contact layers.
        /// For a soluble interface material synchronize the layer heights with the object,
        /// otherwise set the layer height to a bridging flow of a support interface nozzle.
        ///
        /// The tasks of the `tbb::task_group` of a layer (the bottom contacts, the projection of the supports, the
        /// projection of the enforcers) write different things, so they run one after the other here.
        pub fn bottom_contact_layers_and_layer_support_areas(
            &self,
            object: &pf::PrintObject,
            top_contacts: &SupportGeneratorLayersPtr,
            buildplate_covered: &mut Vec<Polygons>,
            layer_storage: &mut SupportGeneratorLayerStorage,
            layer_support_areas: &mut Vec<Polygons>,
        ) -> SupportGeneratorLayersPtr {
            if top_contacts.is_empty() {
                return Vec::new();
            }

            //FIXME higher expansion_to_slice here? why?
            //const auto   expansion_to_slice = m_support_material_flow.scaled_spacing() / 2 + 25;
            let grid_params = SupportGridParams::new(&self.m_object_config, &self.m_support_params.support_material_flow);
            let buildplate_only = !buildplate_covered.is_empty();

            // Allocate empty surface areas, one per object layer.
            *layer_support_areas = vec![Vec::new(); object.total_layer_count()];

            // find object top surfaces
            // we'll use them to clip our support and detect where does it stick
            let mut bottom_contacts: SupportGeneratorLayersPtr = Vec::new();

            // There is some support to be built, if there are non-empty top surfaces detected.
            // Sum of unsupported contact areas above the current layer.print_z.
            let mut overhangs_projection: Polygons = Vec::new();
            // Sum of unsupported enforcer contact areas above the current layer.print_z.
            // Only used if "supports on build plate only" is enabled and both automatic and support enforcers are enabled.
            let mut enforcers_projection: Polygons = Vec::new();
            // Last top contact layer visited when collecting the projection of contact areas.
            let mut contact_idx: i32 = top_contacts.len() as i32 - 1;
            let mut layer_id: i32 = object.total_layer_count() as i32 - 2;
            while layer_id >= 0 {
                let layer = object.get_layer(layer_id);
                // Collect projections of all contact areas above or at the same level as this top surface.
                while contact_idx >= 0 && layer_storage[top_contacts[contact_idx as usize]].print_z > layer.print_z - EPSILON {
                    let top_contact = &mut layer_storage[top_contacts[contact_idx as usize]];
                    // Contact surfaces are expanded away from the object, trimmed by the object.
                    // Use a slight positive offset to overlap the touching regions.
                    // Consume the contact_polygons. The contact polygons are already expanded into a grid form, and they are a tiny bit smaller
                    // than the grid cells.
                    // polygons_append(polygons_new, std::move(*top_contact.contact_polygons)): the vector it is moved from is left empty.
                    let mut polygons_new: Polygons =
                        std::mem::take(top_contact.contact_polygons.as_mut().expect("bottom_contact_layers_and_layer_support_areas: a top contact has contact polygons"));
                    let mut enforcers_new: Polygons = Vec::new();
                    if let Some(enforcer_polygons) = top_contact.enforcer_polygons.as_mut() {
                        enforcers_new = std::mem::take(enforcer_polygons);
                    }
                    // These are the overhang surfaces. They are touching the object and they are not expanded away from the object.
                    // Use a slight positive offset to overlap the touching regions.
                    polygons_new.extend(cu::expand(
                        top_contact.overhang_polygons.as_ref().expect("bottom_contact_layers_and_layer_support_areas: a top contact has overhang polygons"),
                        scaled_epsilon() as f32,
                        cu::DEFAULT_JOIN_TYPE,
                        cu::DEFAULT_MITER_LIMIT,
                    ));
                    overhangs_projection.extend(cu::union_(&polygons_new));
                    enforcers_projection.extend(enforcers_new);
                    contact_idx -= 1;
                }
                if overhangs_projection.is_empty() && enforcers_projection.is_empty() {
                    layer_id -= 1;
                    continue;
                }

                // Overhangs_projection will be filled in asynchronously, move it away.
                // (`union_(std::move(...))` takes a const reference: nothing is moved.)
                let overhangs_projection_raw: Polygons = cu::union_(&overhangs_projection);
                let enforcers_projection_raw: Polygons = cu::union_(&enforcers_projection);

                let overhangs_for_bottom_contacts: &Polygons = if buildplate_only { &enforcers_projection_raw } else { &overhangs_projection_raw };
                if !overhangs_for_bottom_contacts.is_empty() {
                    // Find the bottom contact layers above the top surfaces of this layer.
                    let layer_new = detect_bottom_contacts(
                        &self.m_slicing_params,
                        &self.m_support_params,
                        object,
                        self.m_print.config(),
                        layer,
                        top_contacts,
                        contact_idx,
                        layer_storage,
                        layer_support_areas,
                        overhangs_for_bottom_contacts,
                    );
                    if let Some(layer_new) = layer_new {
                        bottom_contacts.push(layer_new);
                    }
                }

                let layer_buildplate_covered: Option<&mut Polygons> =
                    if buildplate_covered.is_empty() { None } else { Some(&mut buildplate_covered[layer_id as usize]) };
                // Filtering the propagated support columns to two extrusions, overlapping by maximum 20%.
                {
                    // buildplate_covered[layer_id] will be consumed here.
                    let (first, second) = project_support_to_grid(layer, &grid_params, &overhangs_projection_raw, layer_buildplate_covered);
                    layer_support_areas[layer_id as usize] = first;
                    // (the enforcers below are tested on what `enforcers_projection` holds, not on this)
                    overhangs_projection = second;
                    // When propagating support areas downwards, stop propagating the support column if it becomes too thin to be printable.
                    //overhangs_projection = opening(overhangs_projection, column_propagation_filtering_radius);
                }

                let mut layer_support_area_enforcers: Polygons = Vec::new();
                if !enforcers_projection.is_empty() {
                    // Project the enforcers polygons downwards, don't trim them with the "buildplate only" polygons.
                    let (first, second) = project_support_to_grid(layer, &grid_params, &enforcers_projection_raw, None);
                    layer_support_area_enforcers = first;
                    enforcers_projection = second;
                }

                if !layer_support_area_enforcers.is_empty() {
                    let layer_support_area = &mut layer_support_areas[layer_id as usize];
                    if layer_support_area.is_empty() {
                        *layer_support_area = layer_support_area_enforcers;
                    } else {
                        *layer_support_area = cu::union_2(layer_support_area, &layer_support_area_enforcers);
                    }
                }
                layer_id -= 1;
            } // over all layers downwards

            bottom_contacts.reverse();
            self.trim_support_layers_by_object(
                object,
                &bottom_contacts,
                self.m_slicing_params.gap_support_object,
                self.m_slicing_params.gap_object_support,
                self.m_support_params.gap_xy,
                layer_storage,
            );
            bottom_contacts
        }

        /// SupportMaterial.cpp:2795-2819
        /// C++: `void PrintObjectSupportMaterial::trim_top_contacts_by_bottom_contacts(const PrintObject &object, const
        /// SupportGeneratorLayersPtr &bottom_contacts, SupportGeneratorLayersPtr &top_contacts) const`
        ///
        /// Trim the top_contacts layers with the bottom_contacts layers if they overlap, so there would not be enough vertical space for both of them.
        ///
        /// The `tbb::parallel_for` starts every sub-range with the binary search of idx_lower_or_equal(); the index it
        /// finds is the one the walk down from the layer above reaches, so the layers run as one range here.
        pub fn trim_top_contacts_by_bottom_contacts(
            &self,
            _object: &pf::PrintObject,
            bottom_contacts: &SupportGeneratorLayersPtr,
            top_contacts: &SupportGeneratorLayersPtr,
            layer_storage: &mut SupportGeneratorLayerStorage,
        ) {
            use crate::support::support_common::idx_lower_or_equal;
            let mut idx_bottom_overlapping_first: isize = -2;
            // For all top contact layers, counting downwards due to the way idx_higher_or_equal caches the last index to avoid repeated binary search.
            for idx_top in (0..top_contacts.len()).rev() {
                let layer_top = top_contacts[idx_top];
                let layer_top_bottom_z = layer_storage[layer_top].bottom_z;
                // Find the first bottom layer overlapping with layer_top.
                idx_bottom_overlapping_first = idx_lower_or_equal(bottom_contacts, idx_bottom_overlapping_first, |&layer_bottom| {
                    layer_storage[layer_bottom].bottom_print_z() - EPSILON <= layer_top_bottom_z
                });
                // For all top contact layers overlapping with the thick bottom contact layer:
                let mut idx_bottom_overlapping = idx_bottom_overlapping_first;
                while idx_bottom_overlapping >= 0 {
                    let layer_bottom = bottom_contacts[idx_bottom_overlapping as usize];
                    debug_assert!(layer_storage[layer_bottom].bottom_print_z() - EPSILON <= layer_storage[layer_top].bottom_z);
                    if layer_storage[layer_top].print_z < layer_storage[layer_bottom].print_z + EPSILON {
                        // Layers overlap. Trim layer_top with layer_bottom.
                        let trimmed = cu::diff(&layer_storage[layer_top].polygons, &layer_storage[layer_bottom].polygons, ApplySafetyOffset::No);
                        layer_storage[layer_top].polygons = trimmed;
                    } else {
                        break;
                    }
                    idx_bottom_overlapping -= 1;
                }
            }
        }

        /// SupportMaterial.cpp:2821-3007
        /// C++: `SupportGeneratorLayersPtr PrintObjectSupportMaterial::raft_and_intermediate_support_layers(const
        /// PrintObject &object, const SupportGeneratorLayersPtr &bottom_contacts, const SupportGeneratorLayersPtr
        /// &top_contacts, SupportGeneratorLayerStorage &layer_storage) const`
        ///
        /// Generate raft layers and the intermediate support layers between the bottom contact and top contact surfaces.
        pub fn raft_and_intermediate_support_layers(
            &self,
            object: &pf::PrintObject,
            bottom_contacts: &SupportGeneratorLayersPtr,
            top_contacts: &SupportGeneratorLayersPtr,
            layer_storage: &mut SupportGeneratorLayerStorage,
        ) -> SupportGeneratorLayersPtr {
            let mut intermediate_layers: SupportGeneratorLayersPtr = Vec::new();
            let sp = &self.m_slicing_params;
            let support_layer_height_min = self.m_support_params.support_layer_height_min;

            // Collect and sort the extremes (bottoms of the top contacts and tops of the bottom contacts).
            let mut extremes: SupportGeneratorLayersPtr = Vec::with_capacity(top_contacts.len() + bottom_contacts.len());
            // Bottoms of the top contact layers. In case of non-soluble supports,
            // the top contact layer thickness is not known yet.
            extremes.extend_from_slice(top_contacts);
            // Tops of the bottom contact layers.
            extremes.extend_from_slice(bottom_contacts);
            if extremes.is_empty() {
                return intermediate_layers;
            }

            // `auto layer_extreme_lower = [](const SupportGeneratorLayer *l1, const SupportGeneratorLayer *l2)`
            crate::clipper::libcxx_sort(&mut extremes, |&l1, &l2| {
                let z1 = layer_storage[l1].extreme_z();
                let z2 = layer_storage[l2].extreme_z();
                // If the layers are aligned, return the top contact surface first.
                z1 < z2
                    || (z1 == z2
                        && layer_storage[l1].layer_type == SupporLayerType::SltTopContact
                        && layer_storage[l2].layer_type == SupporLayerType::SltBottomContact)
            });

            let synchronize = self.synchronize_layers();

            // The new intermediate layer: its place in the storage.
            let allocate = |layer_storage: &mut SupportGeneratorLayerStorage, bottom_z: f64, print_z: f64, height: f64| -> usize {
                let idx = layer_storage.len();
                let layer_new = layer_storage.allocate(SupporLayerType::SltIntermediate);
                layer_new.bottom_z = bottom_z;
                layer_new.print_z = print_z;
                layer_new.height = height;
                idx
            };

            // Generate intermediate layers.
            // The first intermediate layer is the same as the 1st layer if there is no raft,
            // or the bottom of the first intermediate layer is aligned with the bottom of the raft contact layer.
            // Intermediate layers are always printed with a normal extrusion flow (non-bridging).
            let layers = object.layers();
            let mut idx_layer_object: usize = 0;
            let mut idx_extreme_first: usize = 0;
            if !extremes.is_empty() && (layer_storage[extremes[0]].extreme_z() - sp.raft_interface_top_z).abs() < EPSILON {
                // This is a raft contact layer, its height has been decided in this->top_contact_layers().
                // Ignore this layer when calculating the intermediate support layers.
                idx_extreme_first += 1;
            }
            for idx_extreme in idx_extreme_first..extremes.len() {
                let extr2 = extremes[idx_extreme];
                let mut extr2z: f64 = layer_storage[extr2].extreme_z();
                if (extr2z - sp.first_print_layer_height).abs() < EPSILON {
                    // This is a bottom of a synchronized (or soluble) top contact layer, its height has been decided in this->top_contact_layers().
                    if intermediate_layers.is_empty() || layer_storage[*intermediate_layers.last().unwrap()].print_z < sp.first_print_layer_height {
                        let layer_new = allocate(layer_storage, 0., sp.first_print_layer_height, sp.first_print_layer_height);
                        intermediate_layers.push(layer_new);
                    }
                    continue;
                }
                let extr1: Option<usize> = if idx_extreme == idx_extreme_first { None } else { Some(extremes[idx_extreme - 1]) };
                // Fuse a support layer firmly to the raft top interface (not to the raft contacts).
                let mut extr1z: f64 = match extr1 {
                    None => sp.raft_interface_top_z,
                    Some(e) => layer_storage[e].extreme_z(),
                };
                if extr1z.abs() < EPSILON {
                    // This layer interval starts with the 1st layer. Print the 1st layer using the prescribed 1st layer thickness.
                    // At this point only layers above first_print_layer_heigth + EPSILON are expected as the other cases were captured earlier.
                    // Generate a new intermediate layer.
                    extr1z = sp.first_print_layer_height;
                    let layer_new = allocate(layer_storage, 0., extr1z, extr1z);
                    intermediate_layers.push(layer_new);
                    // Continue printing the other layers up to extr2z.
                }
                let mut dist: f64 = extr2z - extr1z;
                if dist == 0. {
                    continue;
                }
                // The new layers shall be at least m_support_layer_height_min thick.
                if synchronize {
                    // Emit support layers synchronized with the object layers.
                    // Find the first object layer, which has its print_z in this support Z range.
                    while idx_layer_object < layers.len() && layers[idx_layer_object].print_z < extr1z + EPSILON {
                        idx_layer_object += 1;
                    }
                    if idx_layer_object == 0 && extr1z == sp.raft_interface_top_z {
                        // Insert one base support layer below the object.
                        let print_z = sp.object_print_z_min;
                        let bottom_z = sp.raft_interface_top_z;
                        let layer_new = allocate(layer_storage, bottom_z, print_z, print_z - bottom_z);
                        intermediate_layers.push(layer_new);
                    }
                    // Emit all intermediate support layers synchronized with object layers up to extr2z.
                    while idx_layer_object < layers.len() && layers[idx_layer_object].print_z < extr2z + EPSILON {
                        let print_z = layers[idx_layer_object].print_z;
                        let height = layers[idx_layer_object].height;
                        let bottom_z = if idx_layer_object > 0 { layers[idx_layer_object - 1].print_z } else { print_z - height };
                        let layer_new = allocate(layer_storage, bottom_z, print_z, height);
                        intermediate_layers.push(layer_new);
                        idx_layer_object += 1;
                    }
                } else {
                    // Insert intermediate layers.
                    let mut n_layers_extra: usize = (dist / sp.max_suport_layer_height).ceil() as usize;
                    let mut step: f64 = dist / n_layers_extra as f64;
                    if let Some(e1) = extr1 {
                        let (e1_type, e1_print_z, e1_bottom_z, e1_height) = {
                            let l = &layer_storage[e1];
                            (l.layer_type, l.print_z, l.bottom_z, l.height)
                        };
                        if e1_type == SupporLayerType::SltTopContact && e1_print_z + support_layer_height_min > e1_bottom_z + step {
                            // The bottom extreme is a bottom of a top surface. Ensure that the gap
                            // between the 1st intermediate layer print_z and extr1->print_z is not too small.
                            // Generate the first intermediate layer.
                            extr1z = e1_print_z;
                            let layer_new = allocate(layer_storage, e1_bottom_z, extr1z, e1_height);
                            intermediate_layers.push(layer_new);
                            dist = extr2z - extr1z;
                            n_layers_extra = (dist / sp.max_suport_layer_height).ceil() as usize;
                            if n_layers_extra == 0 {
                                continue;
                            }
                            // Continue printing the other layers up to extr2z.
                            step = dist / n_layers_extra as f64;
                        }
                    }
                    if !sp.soluble_interface && layer_storage[extr2].layer_type == SupporLayerType::SltTopContact {
                        // This is a top interface layer, which does not have a height assigned yet. Do it now.
                        let l = &mut layer_storage[extr2];
                        l.height = step;
                        extr2z = l.print_z - step;
                        l.bottom_z = extr2z;
                        n_layers_extra -= 1;
                        if n_layers_extra == 0 {
                            continue;
                        }
                    }
                    let extr2z_large_steps: f64 = extr2z;
                    // Take the largest allowed step in the Z axis until extr2z_large_steps is reached.
                    for i in 0..n_layers_extra {
                        let (bottom_z, print_z, height);
                        if i + 1 == n_layers_extra {
                            // Last intermediate layer added. Align the last entered layer with extr2z_large_steps exactly.
                            bottom_z = if i == 0 { extr1z } else { layer_storage[*intermediate_layers.last().unwrap()].print_z };
                            print_z = extr2z_large_steps;
                            height = print_z - bottom_z;
                        } else {
                            // Intermediate layer, not the last added.
                            height = step;
                            // layer_new.bottom_z = extr1z + i * step;
                            bottom_z = (i as f64).mul_add(step, extr1z);
                            print_z = bottom_z + step;
                        }
                        let layer_new = allocate(layer_storage, bottom_z, print_z, height);
                        intermediate_layers.push(layer_new);
                    }
                }
            }

            intermediate_layers
        }

        /// SupportMaterial.cpp:3009-3162
        /// C++: `void PrintObjectSupportMaterial::generate_base_layers(const PrintObject &object, const
        /// SupportGeneratorLayersPtr &bottom_contacts, const SupportGeneratorLayersPtr &top_contacts,
        /// SupportGeneratorLayersPtr &intermediate_layers, const std::vector<Polygons> &layer_support_areas) const`
        ///
        /// At this stage there shall be intermediate_layers allocated between bottom_contacts and top_contacts, but they have no polygons assigned.
        /// Also the bottom/top_contacts shall have a layer thickness assigned already.
        ///
        /// The `tbb::parallel_for` starts every sub-range with the binary searches of idx_lower_or_equal(); the indices
        /// they find are the ones the walk down from the layer above reaches, so the layers run as one range here.
        pub fn generate_base_layers(
            &self,
            object: &pf::PrintObject,
            bottom_contacts: &SupportGeneratorLayersPtr,
            top_contacts: &SupportGeneratorLayersPtr,
            intermediate_layers: &SupportGeneratorLayersPtr,
            layer_support_areas: &[Polygons],
            layer_storage: &mut SupportGeneratorLayerStorage,
        ) {
            use crate::support::support_common::idx_lower_or_equal;
            if top_contacts.is_empty() {
                // No top contacts -> no intermediate layers will be produced.
                return;
            }

            {
                // index -2 means not initialized yet, -1 means intialized and decremented to 0 and then -1.
                let mut idx_top_contact_above: isize = -2;
                let mut idx_bottom_contact_overlapping: isize = -2;
                let mut idx_object_layer_above: isize = -2;
                // Counting down due to the way idx_lower_or_equal caches indices to avoid repeated binary search over the complete sequence.
                for idx_intermediate in (0..intermediate_layers.len()).rev() {
                    let layer_intermediate = intermediate_layers[idx_intermediate];
                    let (li_print_z, li_bottom_z, li_bottom_print_z) = {
                        let l = &layer_storage[layer_intermediate];
                        (l.print_z, l.bottom_z, l.bottom_print_z())
                    };

                    // Find a top_contact layer touching the layer_intermediate from above, if any, and collect its polygons into polygons_new.
                    // New polygons for layer_intermediate.
                    let mut polygons_new: Polygons;

                    // Use the precomputed layer_support_areas. "idx_object_layer_above": above means above since the last iteration, not above after this call.
                    idx_object_layer_above = idx_lower_or_equal(object.layers(), idx_object_layer_above, |layer: &Layer| layer.print_z <= li_print_z + EPSILON);

                    // Polygons to trim polygons_new.
                    let mut polygons_trimming: Polygons = Vec::new();

                    // Trimming the base layer with any overlapping top layer.
                    // Following cases are recognized:
                    // 1) top.bottom_z >= base.top_z -> No overlap, no trimming needed.
                    // 2) base.bottom_z >= top.print_z -> No overlap, no trimming needed.
                    // 3) base.print_z > top.print_z  && base.bottom_z >= top.bottom_z -> Overlap, which will be solved inside generate_toolpaths() by reducing the base layer height where it overlaps the top layer. No trimming needed here.
                    // 4) base.print_z > top.bottom_z && base.bottom_z < top.bottom_z -> Base overlaps with top.bottom_z. This must not happen.
                    // 5) base.print_z <= top.print_z  && base.bottom_z >= top.bottom_z -> Base is fully inside top. Trim base by top.
                    idx_top_contact_above = idx_lower_or_equal(top_contacts, idx_top_contact_above, |&layer| layer_storage[layer].bottom_z <= li_print_z - EPSILON);
                    // Collect all the top_contact layer intersecting with this layer.
                    let mut idx_top_contact_overlapping = idx_top_contact_above;
                    while idx_top_contact_overlapping >= 0 {
                        let layer_top_overlapping = &layer_storage[top_contacts[idx_top_contact_overlapping as usize]];
                        if layer_top_overlapping.print_z < li_bottom_z + EPSILON {
                            break;
                        }
                        // Base must not overlap with top.bottom_z.
                        if li_print_z <= layer_top_overlapping.print_z + EPSILON && li_bottom_z >= layer_top_overlapping.bottom_z - EPSILON {
                            // Base is fully inside top. Trim base by top.
                            polygons_trimming.extend(layer_top_overlapping.polygons.iter().cloned());
                        }
                        idx_top_contact_overlapping -= 1;
                    }

                    if idx_object_layer_above < 0 {
                        // layer_support_areas are synchronized with object layers and they contain projections of the contact layers above them.
                        // This intermediate layer is not above any object layer, thus there is no information in layer_support_areas about
                        // towers supporting contact layers intersecting the first object layer. Project these contact layers now.
                        polygons_new = layer_support_areas[0].clone();
                        let first_layer_z: f64 = object.layers()[0].print_z;
                        let mut i = idx_top_contact_above + 1;
                        while i < top_contacts.len() as isize {
                            let contacts = &layer_storage[top_contacts[i as usize]];
                            if contacts.print_z > first_layer_z + EPSILON {
                                break;
                            }
                            polygons_new.extend(contacts.polygons.iter().cloned());
                            i += 1;
                        }
                    } else {
                        polygons_new = layer_support_areas[idx_object_layer_above as usize].clone();
                    }

                    // Trimming the base layer with any overlapping bottom layer.
                    // Following cases are recognized:
                    // 1) bottom.bottom_z >= base.top_z -> No overlap, no trimming needed.
                    // 2) base.bottom_z >= bottom.print_z -> No overlap, no trimming needed.
                    // 3) base.print_z > bottom.bottom_z && base.bottom_z < bottom.bottom_z -> Overlap, which will be solved inside generate_toolpaths() by reducing the bottom layer height where it overlaps the base layer. No trimming needed here.
                    // 4) base.print_z > bottom.print_z  && base.bottom_z >= bottom.print_z -> Base overlaps with bottom.print_z. This must not happen.
                    // 5) base.print_z <= bottom.print_z && base.bottom_z >= bottom.bottom_z -> Base is fully inside top. Trim base by top.
                    idx_bottom_contact_overlapping =
                        idx_lower_or_equal(bottom_contacts, idx_bottom_contact_overlapping, |&layer| layer_storage[layer].bottom_print_z() <= li_print_z - EPSILON);
                    // Collect all the bottom_contacts layer intersecting with this layer.
                    let mut i = idx_bottom_contact_overlapping;
                    while i >= 0 {
                        let layer_bottom_overlapping = &layer_storage[bottom_contacts[i as usize]];
                        if layer_bottom_overlapping.print_z < li_bottom_print_z + EPSILON {
                            break;
                        }
                        // Base must not overlap with bottom.top_z.
                        if li_print_z <= layer_bottom_overlapping.print_z + EPSILON && li_bottom_z >= layer_bottom_overlapping.bottom_print_z() - EPSILON {
                            // Base is fully inside bottom. Trim base by bottom.
                            polygons_trimming.extend(layer_bottom_overlapping.polygons.iter().cloned());
                        }
                        i -= 1;
                    }

                    // Trim the polygons, store them.
                    let l = &mut layer_storage[layer_intermediate];
                    if polygons_trimming.is_empty() {
                        l.polygons = polygons_new;
                    } else {
                        // safety offset to merge the touching source polygons
                        l.polygons = cu::diff(&polygons_new, &polygons_trimming, ApplySafetyOffset::Yes);
                    }
                    l.layer_type = SupporLayerType::SltBase;
                }
            }

            self.trim_support_layers_by_object(
                object,
                intermediate_layers,
                self.m_slicing_params.gap_support_object,
                self.m_slicing_params.gap_object_support,
                self.m_support_params.gap_xy,
                layer_storage,
            );
        }

        /// SupportMaterial.cpp:3164-3263
        /// C++: `void PrintObjectSupportMaterial::trim_support_layers_by_object(const PrintObject &object,
        /// SupportGeneratorLayersPtr &support_layers, const coordf_t gap_extra_above, const coordf_t gap_extra_below,
        /// const coordf_t gap_xy) const`
        ///
        /// Trim support layers by an object to leave a defined gap between
        /// the support volume and the object.
        ///
        /// The `tbb::parallel_for` starts every sub-range with the binary search of Layer::idx_higher_or_equal(); the
        /// index it finds is the one the walk up from the layer below reaches, so the layers run as one range here.
        pub fn trim_support_layers_by_object(
            &self,
            object: &pf::PrintObject,
            support_layers: &SupportGeneratorLayersPtr,
            gap_extra_above: f64,
            gap_extra_below: f64,
            gap_xy: f64,
            layer_storage: &mut SupportGeneratorLayerStorage,
        ) {
            let gap_xy_scaled: f32 = scale_(gap_xy) as f32;
            let print_config = self.m_print.config();
            let layers = object.layers();

            // Collect non-empty layers to be processed in parallel.
            // This is a good idea as pulling a thread from a thread pool for an empty task is expensive.
            let mut nonempty_layers: SupportGeneratorLayersPtr = Vec::with_capacity(support_layers.len());
            for &support_layer in support_layers {
                let l = &layer_storage[support_layer];
                if !l.polygons.is_empty() && l.print_z >= self.m_slicing_params.raft_contact_top_z + EPSILON {
                    // Non-empty support layer and not a raft layer.
                    nonempty_layers.push(support_layer);
                }
            }

            // For all intermediate support layers:
            let mut idx_object_layer_overlapping: usize = usize::MAX;

            // `auto is_layers_overlap = [](const SupportGeneratorLayer& support_layer, const Layer& object_layer, coordf_t bridging_height = 0.f) -> bool`
            let is_layers_overlap = |sl_print_z: f64, sl_bottom_z: f64, object_layer: &Layer, bridging_height: f64| -> bool {
                if (sl_print_z - object_layer.print_z).abs() < EPSILON {
                    return true;
                }

                let object_lh: f64 = if bridging_height > EPSILON { bridging_height } else { object_layer.height };
                if sl_print_z < object_layer.print_z && sl_print_z > object_layer.print_z - object_lh {
                    return true;
                }

                if sl_print_z > object_layer.print_z && sl_bottom_z < object_layer.print_z - EPSILON {
                    return true;
                }

                false
            };
            let no_tails: ExPolygons = Vec::new();
            for idx_layer in 0..nonempty_layers.len() {
                let support_layer = nonempty_layers[idx_layer];
                let (sl_print_z, sl_bottom_z, sl_bottom_print_z) = {
                    let l = &layer_storage[support_layer];
                    (l.print_z, l.bottom_z, l.bottom_print_z())
                };
                // Find the overlapping object layers including the extra above / below gap.
                let z_threshold: f64 = sl_bottom_print_z - gap_extra_below + EPSILON;
                idx_object_layer_overlapping = Layer::idx_higher_or_equal(layers, idx_object_layer_overlapping, |layer: &Layer| layer.print_z >= z_threshold);
                // Collect all the object layers intersecting with this layer.
                let mut polygons_trimming: Polygons = Vec::new();
                let mut i: usize = idx_object_layer_overlapping;
                while i < layers.len() {
                    let object_layer = &layers[i];
                    if object_layer.bottom_z() > sl_print_z + gap_extra_above - EPSILON {
                        break;
                    }

                    let is_overlap = is_layers_overlap(sl_print_z, sl_bottom_z, object_layer, 0.);
                    // `object_layer.sharp_tails`
                    let sharp_tails: &ExPolygons = self.layer_sharp_tails.get(i).unwrap_or(&no_tails);
                    for expoly in &object_layer.lslices {
                        // BBS
                        let is_sharptail = cu_expolygon::overlaps_expolygons(std::slice::from_ref(expoly), sharp_tails);
                        // coordf_t trimming_offset = is_sharptail ? scale_(sharp_tail_xy_gap) : is_overlap ? gap_xy_scaled : scale_(no_overlap_xy_gap);
                        let trimming_offset: f64 = if is_sharptail {
                            scale_(SHARP_TAIL_XY_GAP)
                        } else if is_overlap {
                            gap_xy_scaled as f64
                        } else {
                            scale_(NO_OVERLAP_XY_GAP)
                        };
                        polygons_trimming.extend(cu::offset_expolygon(expoly, trimming_offset as f32, SSOP_JT, SSOP_ML));
                    }
                    i += 1;
                }
                if !self.m_slicing_params.soluble_interface && cfg_bool(&self.m_object_config, "thick_bridges") {
                    // Collect all bottom surfaces, which will be extruded with a bridging flow.
                    while i < layers.len() {
                        let object_layer = &layers[i];
                        let mut some_region_overlaps = false;
                        for region in object_layer.regions() {
                            let print_region = object.printing_region(region.region_id());
                            let print_region = print_region.borrow();
                            let bridging_height: f64 = print_region_bridging_height_avg(&print_region, print_config);
                            if object_layer.print_z - bridging_height > sl_print_z + gap_extra_above - EPSILON {
                                break;
                            }
                            some_region_overlaps = true;

                            let is_overlap = is_layers_overlap(sl_print_z, sl_bottom_z, object_layer, bridging_height);
                            let trimming_offset: f64 = if is_overlap { gap_xy_scaled as f64 } else { scale_(NO_OVERLAP_XY_GAP) };
                            // offset(region->fill_surfaces.filter_by_type(stBottomBridge), trimming_offset, SUPPORT_SURFACES_OFFSET_PARAMETERS)
                            let bottom_bridges: ExPolygons =
                                region.fill_surfaces.filter_by_type(SurfaceType::BottomBridge).iter().map(|s| s.expolygon.clone()).collect();
                            polygons_trimming.extend(cu::offset_expolygons(&bottom_bridges, trimming_offset as f32, SSOP_JT, SSOP_ML));
                            if cfg_bool(print_region.config(), "detect_overhang_wall") {
                                // Add bridging perimeters.
                                collect_bridging_perimeter_areas(&region.perimeters, gap_xy_scaled, &mut polygons_trimming);
                            }
                        }
                        if !some_region_overlaps {
                            break;
                        }
                        i += 1;
                    }
                }

                // $layer->slices contains the full shape of layer, thus including
                // perimeter's width. $support contains the full shape of support
                // material, thus including the width of its foremost extrusion.
                // We leave a gap equal to a full extrusion width.
                let trimmed = cu::diff(&layer_storage[support_layer].polygons, &polygons_trimming, ApplySafetyOffset::No);
                layer_storage[support_layer].polygons = trimmed;
            }
        }
    }

    /// `make_support_merge_plan(object)` (SupportMergePlan.cpp:254-403, called by merge_support_layers_multi_diameter(),
    /// SupportCommon.cpp:1547): what the function reads of the object and of its print, handed to the crate's
    /// `make_support_merge_plan` (see `SupportMergePlanObject`). The regions are the printing regions of the
    /// faithful object.
    pub fn make_support_merge_plan_of_object(object: &pf::PrintObject, print: &pf::Print) -> crate::support_merge_plan::SupportMergePlan {
        use crate::support_merge_plan::{make_support_merge_plan, SupportMergePlan, SupportMergePlanLayer, SupportMergePlanObject, SupportMergePlanRegion};
        // SupportMergePlan.cpp:259-260: nothing of the object is read off the mode of several nozzle diameters.
        if !print.uses_multiple_nozzle_diameters() {
            return SupportMergePlan::default();
        }
        let print_config = print.config();
        // SupportMergePlan.cpp:341-363 `layers[layer_idx]->regions()`, `layers[layer_idx]->empty()`
        let layers: Vec<SupportMergePlanLayer> = object
            .layers()
            .iter()
            .map(|layer| SupportMergePlanLayer {
                print_z: layer.print_z,
                height: layer.height,
                empty: layer.empty(),
                regions: layer
                    .regions()
                    .iter()
                    .map(|layerm| {
                        let slices_empty = layerm.slices.is_empty();
                        if slices_empty {
                            // `if (layerm->slices.empty()) continue;`: nothing else of the region is read
                            return SupportMergePlanRegion { slices_empty, ..Default::default() };
                        }
                        let print_region = object.printing_region(layerm.region_id());
                        let print_region = print_region.borrow();
                        let m_config = print_region.config();
                        let (wall_filament, sparse_infill_filament, solid_infill_filament) =
                            (cfg_int(m_config, "wall_filament"), cfg_int(m_config, "sparse_infill_filament"), cfg_int(m_config, "solid_infill_filament"));
                        // `LayerTools::filament_slot_by_role(*fill, region_config)` (ToolOrdering.cpp:176-217) reads
                        // the three filaments of the region configuration.
                        let mut region_config = crate::region_config::PrintRegionConfig::default();
                        region_config.wall_filament = wall_filament as usize;
                        region_config.sparse_infill_filament = sparse_infill_filament as usize;
                        region_config.solid_infill_filament = solid_infill_filament as usize;
                        SupportMergePlanRegion {
                            slices_empty,
                            perimeters_empty: layerm.perimeters.is_empty(),
                            wall_filament,
                            sparse_infill_filament,
                            solid_infill_filament,
                            fill_filament_slots: layerm
                                .fills
                                .entities
                                .iter()
                                .filter_map(|ee| match ee {
                                    ExtrusionEntityType::Collection(fill) if !fill.is_empty() => {
                                        Some(crate::gcode::tool_ordering::LayerTools::filament_slot_by_role(fill, &region_config) as i32)
                                    }
                                    _ => None,
                                })
                                .collect(),
                        }
                    })
                    .collect(),
            })
            .collect();
        // PrintConfig.cpp:454-469 `nozzle_diameter_for_filament(print_config, filament_id)`
        let nozzle_diameter_for_filament = |filament_id: u32| -> f32 { pf::nozzle_diameter_for_filament(print_config, filament_id) };
        // `print_config.filament_soluble.get_at(idx)`
        let soluble: Vec<bool> = match print_config.option("filament_soluble") {
            Some(opt) => match opt.as_any().downcast_ref::<crate::config::ConfigOptionBools>() {
                Some(o) => o.values.iter().map(|v| *v != 0).collect(),
                None => Vec::new(),
            },
            None => Vec::new(),
        };
        let filament_soluble = |idx: usize| -> bool {
            if idx < soluble.len() {
                soluble[idx]
            } else {
                soluble.first().copied().unwrap_or(false)
            }
        };
        // SupportMergePlan.cpp:320-327: the filaments of the model parts of the object
        let filament_count = print_config.option("filament_diameter").and_then(|o| o.as_vector_base()).map(|v| v.size()).unwrap_or(0);
        let global_features = crate::model::GlobalFeatureFilaments::from_config(print.default_region_config(), filament_count);
        let model_object = object.model_object(print);
        let model_part_extruders: Vec<Vec<i32>> =
            model_object.volumes.iter().filter(|volume| volume.is_model_part()).map(|volume| volume.get_extruders(model_object, &global_features)).collect();
        let record = SupportMergePlanObject {
            uses_multiple_nozzle_diameters: print.uses_multiple_nozzle_diameters(),
            layers,
            grid_nozzle: crate::print_config::config_def::layer_grid_nozzle_diameter(print.full_print_config()),
            base_height: object.slicing_parameters().layer_height,
            nozzle_diameter_for_filament: &nozzle_diameter_for_filament,
            filament_soluble: &filament_soluble,
            model_part_extruders,
            support_filament: cfg_int(object.config(), "support_filament"),
            support_interface_filament: cfg_int(object.config(), "support_interface_filament"),
        };
        make_support_merge_plan(&record)
    }

    impl<'a> PrintObjectSupportMaterial<'a> {
        /// SupportMaterial.cpp:376-632
        /// C++: `void PrintObjectSupportMaterial::generate(PrintObject &object)`
        ///
        /// Generate support material for the object.
        /// New support layers will be added to the object,
        /// with extrusion paths and islands filled in for each support layer.
        ///
        /// `fill_factory` is `Fill::new_from_type()` of generate_support_toolpaths() (see `SupportFillFactory`).
        pub fn generate(&mut self, object: &mut pf::PrintObject, fill_factory: &dyn crate::support::support_common::SupportFillFactory) -> crate::Result<()> {
            use crate::support::support_common::{
                generate_interface_layers, generate_raft_base, generate_support_layers, generate_support_toolpaths, merge_support_layers_multi_diameter,
            };
            use crate::support_merge_plan::Candidate;
            use ordered_float::OrderedFloat;

            let mut _max_object_layer_height: f64 = 0.;
            for i in 0..object.layer_count() {
                _max_object_layer_height = _max_object_layer_height.max(object.layers()[i].height);
            }

            // Layer instances will be allocated by std::deque and they will be kept until the end of this function call.
            // The layers will be referenced by various LayersPtr (of type std::vector<Layer*>)
            let mut layer_storage = SupportGeneratorLayerStorage::new();

            let support_detect_begin_time = std::time::Instant::now();

            // Per object layer projection of the object below the layer into print bed.
            let mut buildplate_covered: Vec<Polygons> = self.buildplate_covered(object);

            // Determine the top contact surfaces of the support, defined as:
            // contact = overhangs - clearance + margin
            // This method is responsible for identifying what contact surfaces
            // should the support material expose to the object in order to guarantee
            // that it will be effective, regardless of how it's built below.
            // If raft is to be generated, the 1st top_contact layer will contain the 1st object layer silhouette without holes.
            let mut top_contacts: SupportGeneratorLayersPtr = self.top_contact_layers(object, &buildplate_covered, &mut layer_storage)?;
            object.m_support_stage_times.detect += support_detect_begin_time.elapsed().as_millis() as i64;
            if top_contacts.is_empty() {
                // Nothing is supported, no supports are generated.
                return Ok(());
            }

            if self.canceled() {
                return Ok(());
            }

            // Determine the bottom contact surfaces of the supports over the top surfaces of the object.
            // Depending on whether the support is soluble or not, the contact layer thickness is decided.
            // layer_support_areas contains the per object layer support areas. These per object layer support areas
            // may get merged and trimmed by this->generate_base_layers() if the support layers are not synchronized with object layers.
            let mut layer_support_areas: Vec<Polygons> = Vec::new();
            let mut bottom_contacts: SupportGeneratorLayersPtr =
                self.bottom_contact_layers_and_layer_support_areas(object, &top_contacts, &mut buildplate_covered, &mut layer_storage, &mut layer_support_areas);

            if self.canceled() {
                return Ok(());
            }

            // Allocate empty layers between the top / bottom support contact layers
            // as placeholders for the base and intermediate support layers.
            // The layers may or may not be synchronized with the object layers, depending on the configuration.
            // For example, a single nozzle multi material printing will need to generate a waste tower, which in turn
            // wastes less material, if there are as little tool changes as possible.
            let mut intermediate_layers: SupportGeneratorLayersPtr =
                self.raft_and_intermediate_support_layers(object, &bottom_contacts, &top_contacts, &mut layer_storage);

            self.trim_support_layers_by_object(
                object,
                &top_contacts,
                self.m_slicing_params.gap_support_object,
                self.m_slicing_params.gap_object_support,
                self.m_support_params.gap_xy,
                &mut layer_storage,
            );

            // Fill in intermediate layers between the top / bottom support contact layers, trim them by the object.
            self.generate_base_layers(object, &bottom_contacts, &top_contacts, &intermediate_layers, &layer_support_areas, &mut layer_storage);

            // Because the top and bottom contacts are thick slabs, they may overlap causing over extrusion
            // and unwanted strong bonds to the object.
            // Rather trim the top contacts by their overlapping bottom contacts to leave a gap instead of over extruding
            // top contacts over the bottom contacts.
            self.trim_top_contacts_by_bottom_contacts(object, &bottom_contacts, &top_contacts, &mut layer_storage);

            // Propagate top / bottom contact layers to generate interface layers
            // and base interface layers (for soluble interface / non souble base only)
            let support_interface_begin_time = std::time::Instant::now();
            // (the C++ hands one empty vector over twice)
            let mut empty_layers: SupportGeneratorLayersPtr = Vec::new();
            let mut empty_layers_2: SupportGeneratorLayersPtr = Vec::new();
            let (mut interface_layers, mut base_interface_layers) = generate_interface_layers(
                &self.m_object_config,
                &self.m_support_params,
                &bottom_contacts,
                &top_contacts,
                &mut empty_layers,
                &mut empty_layers_2,
                &intermediate_layers,
                &mut layer_storage,
            );
            object.m_support_stage_times.interface_generate += support_interface_begin_time.elapsed().as_millis() as i64;

            // If raft is to be generated, the 1st top_contact layer will contain the 1st object layer silhouette with holes filled.
            // There is also a 1st intermediate layer containing bases of support columns.
            // Inflate the bases of the support columns and create the raft base under the object.
            let raft_layers: SupportGeneratorLayersPtr = generate_raft_base(
                object,
                &self.m_support_params,
                &self.m_slicing_params,
                &top_contacts,
                &interface_layers,
                &base_interface_layers,
                &intermediate_layers,
                &mut layer_storage,
            );

            if self.canceled() {
                return Ok(());
            }

            // BBS multi-diameter: merge the support areas into slabs a coarse nozzle can extrude. This has to run
            // before generate_support_layers() installs them, because that is where the Z ladder is frozen.
            // print_z -> filament the merged slab at that height was planned for.
            let mut planned_filaments: BTreeMap<OrderedFloat<f64>, Candidate> = BTreeMap::new();
            let mut planned_interface_filaments: BTreeMap<OrderedFloat<f64>, Candidate> = BTreeMap::new();
            let mut planned_base_heights: BTreeMap<OrderedFloat<f64>, f64> = BTreeMap::new();
            let mut planned_interface_heights: BTreeMap<OrderedFloat<f64>, f64> = BTreeMap::new();
            let plan = make_support_merge_plan_of_object(object, self.m_print);
            merge_support_layers_multi_diameter(
                object,
                &plan,
                &self.m_support_params,
                &self.m_slicing_params,
                &raft_layers,
                &mut bottom_contacts,
                &mut top_contacts,
                &mut intermediate_layers,
                &mut interface_layers,
                &mut base_interface_layers,
                &mut planned_filaments,
                &mut planned_interface_filaments,
                &mut planned_base_heights,
                &mut planned_interface_heights,
                None,
                None,
                None,
                -1.,
                &mut layer_storage,
            );

            generate_support_layers(
                object,
                &raft_layers,
                &bottom_contacts,
                &top_contacts,
                &intermediate_layers,
                &interface_layers,
                &base_interface_layers,
                &mut layer_storage,
            );

            // BBS multi-diameter: hand the planned filament over to the support layers the object owns now. Matching by
            // print_z is safe because generate_support_layers() gives every support layer a unique print_z.
            for support_layer in object.m_support_layers.iter_mut() {
                let print_z = support_layer.layer.print_z;
                // `map.lower_bound(print_z - EPSILON)`, then `it->first <= print_z + EPSILON`
                if let Some((z, candidate)) = planned_filaments.range(OrderedFloat(print_z - EPSILON)..).next() {
                    if z.0 <= print_z + EPSILON {
                        support_layer.planned_filament_id = candidate.filament_id;
                        support_layer.planned_nozzle_diameter = candidate.nozzle_diameter;
                    }
                }
                if let Some((z, candidate)) = planned_interface_filaments.range(OrderedFloat(print_z - EPSILON)..).next() {
                    if z.0 <= print_z + EPSILON {
                        support_layer.planned_interface_filament_id = candidate.filament_id;
                        support_layer.planned_interface_nozzle_diameter = candidate.nozzle_diameter;
                    }
                }
                if let Some((z, height)) = planned_base_heights.range(OrderedFloat(print_z - EPSILON)..).next() {
                    if z.0 <= print_z + EPSILON {
                        support_layer.base_slab_height = *height;
                    }
                }
                if let Some((z, height)) = planned_interface_heights.range(OrderedFloat(print_z - EPSILON)..).next() {
                    if z.0 <= print_z + EPSILON {
                        support_layer.interface_slab_height = *height;
                    }
                }
            }

            // Generate the actual toolpaths and save them into each layer.
            let support_toolpath_begin_time = std::time::Instant::now();
            generate_support_toolpaths(
                &mut object.m_support_layers,
                &self.m_object_config,
                &self.m_support_params,
                &self.m_slicing_params,
                &raft_layers,
                &bottom_contacts,
                &top_contacts,
                &intermediate_layers,
                &interface_layers,
                &base_interface_layers,
                &[],
                None,
                None,
                &mut layer_storage,
                fill_factory,
            )?;
            object.m_support_stage_times.toolpath_generate += support_toolpath_begin_time.elapsed().as_millis() as i64;

            Ok(())
        }
    }

    /// PrintConfig.hpp `enum SupportType`: stNormalAuto, stTreeAuto, stNormal, stTree
    fn support_type_from_i32(v: i32) -> SupportType {
        match v {
            0 => SupportType::StNormalAuto,
            1 => SupportType::StTreeAuto,
            2 => SupportType::StNormal,
            _ => SupportType::StTree,
        }
    }
}

// ============================================================================
// SupportMaterial.cpp:171-332, 664-1171 — the support grid: the rasterizer, the contours of the raster,
// SupportGridParams and SupportGridPattern.
//
// `#define SUPPORT_USE_AGG_RASTERIZER` (SupportMaterial.cpp:22) is set, so the branch of the Anti-Grain
// rasterizer is the one ported, over the crate's port of AGG (`crate::sla::agg`). The code under
// `SLIC3R_DEBUG` (the PNG dumps; `serialize` / `deserialize`, which also need the EdgeGrid branch, are
// ported at the end of the class) is not compiled into the engine.
// ============================================================================

pub use grid::{contours_simplified, rasterize_polygons, SupportGridParams, SupportGridPattern};

/// SupportMaterial.cpp:171-332, 664-1171
pub mod grid {
    use crate::clipper::JoinType;
    use crate::config::ConfigOptionResolver;
    use crate::flow::Flow;
    use crate::geometry::{ExPolygon, ExPolygons, Point, Polygon, Polygons};
    use crate::libslic3r::SCALING_FACTOR;
    use crate::print_config::config_def::DynamicPrintConfig;
    use crate::sla::agg::color_gray::Gray8;
    use crate::sla::agg::path_storage::PathStorage;
    use crate::sla::agg::pixfmt_gray::PixfmtGray8;
    use crate::sla::agg::rasterizer_scanline_aa::RasterizerScanlineAa;
    use crate::sla::agg::renderer_base::RendererBase;
    use crate::sla::agg::renderer_scanline::{render_scanlines, RendererScanlineAaSolid};
    use crate::sla::agg::rendering_buffer::RenderingBuffer;
    use crate::sla::agg::scanline_p::ScanlineP8;
    use crate::support::support_common::{cu, get_extents_polygons_bbox, BBox};

    /// PrintConfig.hpp:174-176 `enum SupportMaterialStyle`, the values of the `support_style` option.
    pub const SMS_DEFAULT: i32 = 0;
    pub const SMS_GRID: i32 = 1;
    pub const SMS_SNUG: i32 = 2;
    pub const SMS_TREE_SLIM: i32 = 3;
    pub const SMS_TREE_STRONG: i32 = 4;
    pub const SMS_TREE_HYBRID: i32 = 5;
    pub const SMS_TREE_ORGANIC: i32 = 6;

    /// SupportMaterial.cpp:69 `#define SUPPORT_SURFACES_OFFSET_PARAMETERS ClipperLib::jtSquare, 0.`
    pub const SUPPORT_SURFACES_OFFSET_JOIN_TYPE: JoinType = JoinType::jtSquare;
    pub const SUPPORT_SURFACES_OFFSET_MITER_LIMIT: f64 = 0.;

    /// libslic3r.h:81 `#define scale_(val) ((val) / SCALING_FACTOR)`
    #[inline]
    fn scale_(val: f64) -> f64 {
        val / SCALING_FACTOR
    }

    /// `Vec2i`
    pub type Vec2i = (i32, i32);

    /// Point.hpp:250-253 `operator<(const Point &l, const Point &r)`
    #[inline]
    fn point_less(l: &Point, r: &Point) -> bool {
        l.x < r.x || (l.x == r.x && l.y < r.y)
    }

    /// MultiPoint.cpp `void MultiPoint::rotate(double angle)` over the polygons (Polygon.hpp:165-170
    /// `polygons_rotate`): `sin` and `cos` of one argument as the build takes them, the products contracted.
    fn polygons_rotate(polys: &mut Polygons, angle: f64) {
        let (s, c) = crate::clipper::sin_cos(angle);
        for poly in polys.iter_mut() {
            for pt in poly.points.iter_mut() {
                let cur_x = pt.x as f64;
                let cur_y = pt.y as f64;
                // coord_t(round(c * cur_x - s * cur_y)), coord_t(round(c * cur_y + s * cur_x))
                pt.x = c.mul_add(cur_x, -(s * cur_y)).round() as i32 as i64;
                pt.y = c.mul_add(cur_y, s * cur_x).round() as i32 as i64;
            }
        }
    }

    /// SupportMaterial.cpp:172-203
    /// C++: `static std::vector<unsigned char> rasterize_polygons(const Vec2i &grid_size, const double pixel_size, const Point &left_bottom, const Polygons &polygons)`
    pub fn rasterize_polygons(grid_size: Vec2i, pixel_size: f64, left_bottom: &Point, polygons: &Polygons) -> Vec<u8> {
        let mut data: Vec<u8> = vec![0u8; (grid_size.0 * grid_size.1) as usize];
        {
            let rendering_buffer = RenderingBuffer::new(&mut data, grid_size.0 as u32, grid_size.1 as u32, grid_size.0);
            let pixel_renderer = PixfmtGray8::new(rendering_buffer);
            let mut raw_renderer = RendererBase::new(pixel_renderer);
            raw_renderer.clear(&Gray8::new(0));
            let mut renderer = RendererScanlineAaSolid::new(&mut raw_renderer);
            renderer.color(&Gray8::new(255));

            let mut scanline = ScanlineP8::new();
            let mut rasterizer = RasterizerScanlineAa::new();

            let convert_pt = |pt: &Point| -> (f64, f64) {
                (
                    ((pt.x as i32).wrapping_sub(left_bottom.x as i32)) as f64 / pixel_size,
                    ((pt.y as i32).wrapping_sub(left_bottom.y as i32)) as f64 / pixel_size,
                )
            };
            rasterizer.reset();
            for polygon in polygons {
                // The C++ dereferences `polygon.points.begin()`: a polygon has points.
                let Some(first) = polygon.points.first() else {
                    continue;
                };
                let mut path = PathStorage::new();
                let pt_front = convert_pt(first);
                path.move_to(pt_front.0, pt_front.1);
                for it in &polygon.points[1..] {
                    let pt = convert_pt(it);
                    path.line_to(pt.0, pt.1);
                }
                path.line_to(pt_front.0, pt_front.1);
                rasterizer.add_path(&mut path, 0);
            }
            render_scanlines(&mut rasterizer, &mut scanline, &mut renderer);
        }
        data
    }

    /// SupportMaterial.cpp:205-331
    /// C++: `static Polygons contours_simplified(const Vec2i &grid_size, const double pixel_size, Point left_bottom, const std::vector<unsigned char> &grid, coord_t offset, bool fill_holes)`
    ///
    /// Grid has to have the boundary pixels unset.
    pub fn contours_simplified(grid_size: Vec2i, pixel_size: f64, left_bottom: Point, grid: &[u8], offset: i32, fill_holes: bool) -> Polygons {
        debug_assert!(((2 * offset).abs() as f64) < pixel_size - 10.);
        let (gx, gy) = (grid_size.0, grid_size.1);

        // Fill in empty cells, which have a left / right neighbor filled.
        // Fill in empty cells, which have the top / bottom neighbor filled.
        let mut cell_inside_data: Vec<u8> = Vec::new();
        if fill_holes {
            cell_inside_data = grid.to_vec();
            for r in 1..gy - 1 {
                for c in 1..gx - 1 {
                    let addr = (r * gx + c) as usize;
                    if (grid[addr - 1] != 0 && grid[addr + 1] != 0) || (grid[addr - gx as usize] != 0 && grid[addr + gx as usize] != 0) {
                        cell_inside_data[addr] = 1;
                    }
                }
            }
        }
        let cell_inside: &[u8] = if fill_holes { &cell_inside_data } else { grid };

        // 1) Collect the lines.
        let mut lines: Vec<(Point, Point)> = Vec::new();
        let mut start_point_to_line_idx: Vec<(Point, i32)> = Vec::new();
        for r in 1..gy {
            for c in 1..gx {
                let addr = (r * gx + c) as usize;
                let left = cell_inside[addr - 1] != 0;
                let top = cell_inside[addr - gx as usize] != 0;
                let current = cell_inside[addr] != 0;
                let (c, r) = (c as i64, r as i64);
                if left != current {
                    lines.push(if left { (Point::new(c, r + 1), Point::new(c, r)) } else { (Point::new(c, r), Point::new(c, r + 1)) });
                    start_point_to_line_idx.push((lines[lines.len() - 1].0, lines.len() as i32 - 1));
                }
                if top != current {
                    lines.push(if top { (Point::new(c, r), Point::new(c + 1, r)) } else { (Point::new(c + 1, r), Point::new(c, r)) });
                    start_point_to_line_idx.push((lines[lines.len() - 1].0, lines.len() as i32 - 1));
                }
            }
        }
        // std::sort: the lines that start at one point (two of them at a corner where two contours touch)
        // stay in the order libc++ leaves them.
        crate::clipper::libcxx_sort(&mut start_point_to_line_idx, |l, r| point_less(&l.0, &r.0));

        // 2) Chain the lines.
        let mut line_processed: Vec<bool> = vec![false; lines.len()];
        let mut out: Polygons = Vec::new();
        for i_candidate in 0..lines.len() as i32 {
            if line_processed[i_candidate as usize] {
                continue;
            }
            let mut poly = Polygon::default();
            line_processed[i_candidate as usize] = true;
            poly.points.push(lines[i_candidate as usize].1);
            let mut i_line_current = i_candidate;
            'chain: loop {
                // std::equal_range by the start point
                let key = lines[i_line_current as usize].1;
                let first = start_point_to_line_idx.partition_point(|e| point_less(&e.0, &key));
                let second = start_point_to_line_idx.partition_point(|e| !point_less(&key, &e.0));
                // The interval has to be non empty, there shall be at least one line continuing the current one.
                debug_assert!(first != second);
                let mut i_next: i32 = -1;
                for it in &start_point_to_line_idx[first..second] {
                    if it.1 == i_candidate {
                        // closing the loop.
                        break 'chain;
                    }
                    if line_processed[it.1 as usize] {
                        continue;
                    }
                    if i_next == -1 {
                        i_next = it.1;
                    } else {
                        // This is a corner, where two lines meet exactly. Pick the line, which encloses a smallest angle with
                        // the current edge.
                        let line_current = &lines[i_line_current as usize];
                        let line_next = &lines[it.1 as usize];
                        let v1 = (line_current.1.x - line_current.0.x, line_current.1.y - line_current.0.y);
                        let v2 = (line_next.1.x - line_next.0.x, line_next.1.y - line_next.0.y);
                        let cross: i64 = v1.0 * v2.1 - v2.0 * v1.1;
                        if cross > 0 {
                            // This has to be a convex right angle. There is no better next line.
                            i_next = it.1;
                            break;
                        }
                    }
                }
                assert!(i_next >= 0, "contours_simplified: a line of the raster has no continuation");
                line_processed[i_next as usize] = true;
                i_line_current = i_next;
                poly.points.push(lines[i_line_current as usize].1);
            }
            out.push(poly);
        }

        // 3) Scale the polygons back into world, shrink slightly and remove collinear points.
        for poly in out.iter_mut() {
            for p in poly.points.iter_mut() {
                // `p *= pixel_size` (Point.hpp:199: `coord_t(x * rhs)`, truncated), `p += left_bottom`
                p.x = ((p.x as f64 * pixel_size) as i32).wrapping_add(left_bottom.x as i32) as i64;
                p.y = ((p.y as f64 * pixel_size) as i32).wrapping_add(left_bottom.y as i32) as i64;
            }
            // Shrink the contour slightly, so if the same contour gets discretized and simplified again, one will get the same result.
            // Remove collinear points.
            let n = poly.points.len();
            let mut pts: Vec<Point> = Vec::with_capacity(n);
            for j in 0..n {
                let j0 = if j == 0 { n - 1 } else { j - 1 };
                let j2 = if j + 1 == n { 0 } else { j + 1 };
                let v = (poly.points[j2].x - poly.points[j0].x, poly.points[j2].y - poly.points[j0].y);
                if v.0 != 0 && v.1 != 0 {
                    // This is a corner point. Copy it to the output contour.
                    let mut p = poly.points[j];
                    p.y += if v.0 < 0 { -(offset as i64) } else { offset as i64 };
                    p.x += if v.1 > 0 { -(offset as i64) } else { offset as i64 };
                    pts.push(p);
                }
            }
            poly.points = pts;
        }
        out
    }

    /// SupportMaterial.cpp:664-682 `struct SupportGridParams`
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct SupportGridParams {
        /// `SupportMaterialStyle`, see `SMS_*`
        pub style: i32,
        pub grid_resolution: f64,
        pub support_angle: f64,
        pub extrusion_width: f64,
        pub support_closing_radius: f64,
        pub expansion_to_slice: i32,
        pub expansion_to_propagate: i32,
    }

    impl SupportGridParams {
        /// SupportMaterial.cpp:665-673
        /// C++: `SupportGridParams(const PrintObjectConfig &object_config, const Flow &support_material_flow)`
        pub fn new(object_config: &DynamicPrintConfig, support_material_flow: &Flow) -> Self {
            let opt_float = |key: &str| object_config.option(key).and_then(|o| o.get_float().ok()).unwrap_or(0.0);
            let style = object_config.option("support_style").and_then(|o| o.get_int().ok()).unwrap_or(0);
            // `Flow::spacing()` is a float.
            let spacing = support_material_flow.spacing() as f32 as f64;
            SupportGridParams {
                style,
                grid_resolution: opt_float("support_base_pattern_spacing") + spacing,
                // Geometry.hpp:295 `deg2rad(angle)`: `T(PI) * angle / T(180.0)`
                support_angle: std::f64::consts::PI * opt_float("support_angle") / 180.0,
                extrusion_width: spacing,
                //support_closing_radius(object_config.support_closing_radius.value),
                support_closing_radius: 2.0,
                expansion_to_slice: (support_material_flow.scaled_spacing() / 2 + 5) as i32,
                expansion_to_propagate: -3,
            }
        }
    }

    /// SupportMaterial.cpp:684-1171 `class SupportGridPattern`
    ///
    /// The C++ keeps pointers to the support and trimming polygons of its caller (or to its rotated copies);
    /// here it keeps copies of them.
    #[derive(Debug, Clone)]
    pub struct SupportGridPattern {
        m_style: i32,
        m_support_polygons: Polygons,
        m_trimming_polygons: Polygons,
        // Angle in radians, by which the whole support is rotated.
        m_support_angle: f64,
        // X spacing of the support lines parallel with the Y axis.
        m_support_spacing: f64,
        m_extrusion_width: f64,
        // For snug supports: Morphological closing of support areas.
        m_support_material_closing_radius: f64,

        m_grid_size: Vec2i,
        m_pixel_size: f64,
        m_bbox: BBox,
        m_grid2: Vec<u8>,
        /// `Slic3r::EdgeGrid::Grid m_grid`: the member of the branch without `SUPPORT_USE_AGG_RASTERIZER`
        /// (SupportMaterial.cpp:1162-1164); only `deserialize_` fills it.
        m_grid: crate::edge_grid::EdgeGrid,
    }

    impl SupportGridPattern {
        /// SupportMaterial.cpp:687-768
        /// C++: `SupportGridPattern(const Polygons *support_polygons, const Polygons *trimming_polygons, const SupportGridParams &params)`
        ///
        /// `support_polygons`: Support islands, to be stretched into a grid. Already trimmed with
        /// min(lower_layer_offset, m_gap_xy). `trimming_polygons`: Trimming polygons, to trim the stretched
        /// support islands. support_polygons were already trimmed with trimming_polygons.
        pub fn new(support_polygons: &Polygons, trimming_polygons: &Polygons, params: &SupportGridParams) -> Self {
            let mut this = SupportGridPattern {
                m_style: params.style,
                m_support_polygons: support_polygons.clone(),
                m_trimming_polygons: trimming_polygons.clone(),
                m_support_spacing: params.grid_resolution,
                m_support_angle: params.support_angle,
                m_extrusion_width: params.extrusion_width,
                m_support_material_closing_radius: params.support_closing_radius,
                m_grid_size: (0, 0),
                m_pixel_size: 0.,
                m_bbox: BBox::new(),
                m_grid2: Vec::new(),
                m_grid: crate::edge_grid::EdgeGrid::new(),
            };
            if this.m_style == SMS_DEFAULT {
                this.m_style = SMS_GRID;
            }
            if matches!(this.m_style, SMS_TREE_SLIM | SMS_TREE_STRONG | SMS_TREE_HYBRID | SMS_TREE_ORGANIC) {
                this.m_style = SMS_GRID;
            }
            match this.m_style {
                SMS_GRID => {
                    // Prepare the grid data, it will be reused when extracting support structures.
                    if this.m_support_angle != 0. {
                        // Create a copy of the rotated contours.
                        polygons_rotate(&mut this.m_support_polygons, -params.support_angle);
                        polygons_rotate(&mut this.m_trimming_polygons, -params.support_angle);
                    }

                    // Resolution of the sparse support grid.
                    let grid_resolution: i32 = scale_(this.m_support_spacing) as i32;
                    let mut bbox: BBox = get_extents_polygons_bbox(&this.m_support_polygons);
                    bbox.offset(20.);
                    // Align the bounding box with the sparse support grid.
                    // BoundingBox.cpp:257-263 `BoundingBox::align_to_grid(const coord_t cell_size)`
                    if bbox.defined {
                        bbox.min.x = crate::geometry::align_to_grid(bbox.min.x, grid_resolution as i64);
                        bbox.min.y = crate::geometry::align_to_grid(bbox.min.y, grid_resolution as i64);
                    }

                    this.m_bbox = bbox;
                    // Oversample the grid to avoid leaking of supports through or around the object walls.
                    let extrusion_width_scaled: i32 = scale_(params.extrusion_width) as i32;
                    let oversampling: i32 = ((scale_(this.m_support_spacing) / (extrusion_width_scaled + 100) as f64) as i32).clamp(1, 8);
                    // std::max<double>(extrusion_width_scaled + 21, scale_(m_support_spacing / oversampling))
                    let a = (extrusion_width_scaled + 21) as f64;
                    let b = scale_(this.m_support_spacing / oversampling as f64);
                    this.m_pixel_size = if a < b { b } else { a };
                    // Add one empty column / row boundaries.
                    this.m_bbox.offset(this.m_pixel_size);
                    // Grid size fitting the support polygons plus one pixel boundary around the polygons.
                    let size_x = (this.m_bbox.max.x as i32).wrapping_sub(this.m_bbox.min.x as i32);
                    let size_y = (this.m_bbox.max.y as i32).wrapping_sub(this.m_bbox.min.y as i32);
                    let grid_size_raw: Vec2i = ((size_x as f64 / this.m_pixel_size).ceil() as i32, (size_y as f64 / this.m_pixel_size).ceil() as i32);
                    // Overlay macro blocks of (oversampling x oversampling) over the grid.
                    let grid_blocks: Vec2i = ((grid_size_raw.0 + oversampling - 1 - 2) / oversampling, (grid_size_raw.1 + oversampling - 1 - 2) / oversampling);
                    // and resize the grid to fit the macro blocks + one pixel boundary.
                    this.m_grid_size = (grid_blocks.0 * oversampling + 2, grid_blocks.1 * oversampling + 2);
                    debug_assert!(this.m_grid_size.0 >= grid_size_raw.0);
                    debug_assert!(this.m_grid_size.1 >= grid_size_raw.1);
                    this.m_grid2 = rasterize_polygons(this.m_grid_size, this.m_pixel_size, &this.m_bbox.min, &this.m_support_polygons);

                    let trimming = Self::dilate_trimming_region(
                        &rasterize_polygons(this.m_grid_size, this.m_pixel_size, &this.m_bbox.min, &this.m_trimming_polygons),
                        this.m_grid_size,
                    );
                    Self::seed_fill_block(&mut this.m_grid2, this.m_grid_size, &trimming, grid_blocks, oversampling);
                }
                // case smsSnug: default:
                _ => {
                    // nothing to prepare
                }
            }
            this
        }

        /// SupportMaterial.cpp:770-908
        /// C++: `Polygons extract_support(const coord_t offset_in_grid, bool fill_holes)`
        ///
        /// Extract polygons from the grid, offsetted by offset_in_grid,
        /// and trim the extracted polygons by trimming_polygons.
        /// Trimming by the trimming_polygons may split the extracted polygons into pieces.
        /// Remove all the pieces, which do not contain any of the island_samples.
        pub fn extract_support(&self, offset_in_grid: i32, fill_holes: bool) -> Polygons {
            use cu::ApplySafetyOffset::No;
            match self.m_style {
                SMS_TREE_SLIM | SMS_TREE_STRONG | SMS_TREE_HYBRID | SMS_TREE_ORGANIC => {
                    debug_assert!(false);
                    Vec::new()
                }
                SMS_GRID => {
                    let support_polygons_simplified: Polygons =
                        contours_simplified(self.m_grid_size, self.m_pixel_size, self.m_bbox.min, &self.m_grid2, offset_in_grid, fill_holes);

                    let mut islands: ExPolygons = cu::diff_ex(&support_polygons_simplified, &self.m_trimming_polygons, No);

                    // Extract polygons, which contain some of the island_samples.
                    let mut out: Polygons = Vec::new();

                    // Sample a single point per input support polygon, keep it as a reference to maintain corresponding
                    // polygons if ever these polygons get split into parts by the trimming polygons.
                    // As offset_in_grid may be negative, m_support_polygons may stick slightly outside of islands.
                    // Trim ti with islands.
                    let samples: Vec<Point> = Self::island_samples(&if offset_in_grid > 0 {
                        // Expanding, thus m_support_polygons are all inside islands.
                        cu::union_ex(&self.m_support_polygons)
                    } else {
                        // Shrinking, thus m_support_polygons may be trimmed a tiny bit by islands.
                        cu::intersection_ex_pe(&self.m_support_polygons, &islands, No)
                    });

                    let mut samples_inside: Vec<(Point, bool)> = Vec::new();
                    for island in islands.iter_mut() {
                        // `get_extents(island.contour)`: the box of the points of the contour.
                        let bbox = BBox::from_points(&island.contour.points);
                        // Samples are sorted lexicographically.
                        let lo = Point::new(bbox.min.x - 1, bbox.min.y - 1);
                        let hi = Point::new(bbox.max.x + 1, bbox.max.y + 1);
                        let it_lower = samples.partition_point(|s| point_less(s, &lo));
                        let it_upper = samples.partition_point(|s| !point_less(&hi, s));
                        samples_inside.clear();
                        for it in &samples[it_lower..it_upper.max(it_lower)] {
                            // BoundingBox.hpp `contains(const PointClass &point)`
                            if it.x >= bbox.min.x && it.x <= bbox.max.x && it.y >= bbox.min.y && it.y <= bbox.max.y {
                                samples_inside.push((*it, false));
                            }
                        }
                        if !samples_inside.is_empty() {
                            // For all samples_inside count the boundary crossing.
                            for i_contour in 0..=island.holes.len() {
                                let contour: &Polygon = if i_contour == 0 { &island.contour } else { &island.holes[i_contour - 1] };
                                let n = contour.points.len();
                                if n == 0 {
                                    continue;
                                }
                                let mut j = n - 1;
                                for i in 0..n {
                                    let pi = contour.points[i];
                                    let pj = contour.points[j];
                                    //FIXME this test is not numerically robust. Particularly, it does not handle horizontal segments at y == point(1) well.
                                    // Does the ray with y == point(1) intersect this line segment?
                                    for sample_inside in samples_inside.iter_mut() {
                                        if (pi.y > sample_inside.0.y) != (pj.y > sample_inside.0.y) {
                                            let x1 = sample_inside.0.x as f64;
                                            let x2 = pi.x as f64 + (pj.x - pi.x) as f64 * (sample_inside.0.y - pi.y) as f64 / (pj.y - pi.y) as f64;
                                            if x1 < x2 {
                                                sample_inside.1 = !sample_inside.1;
                                            }
                                        }
                                    }
                                    j = i;
                                }
                            }
                            // If any of the sample is inside this island, add this island to the output.
                            for sample_inside in &samples_inside {
                                if sample_inside.1 {
                                    // polygons_append(out, std::move(island)); island.clear();
                                    let island_taken: ExPolygon = std::mem::take(island);
                                    out.push(island_taken.contour);
                                    out.extend(island_taken.holes);
                                    break;
                                }
                            }
                        }
                    }

                    if self.m_support_angle != 0. {
                        polygons_rotate(&mut out, self.m_support_angle);
                    }
                    out
                }
                // case smsSnug:
                _ => {
                    // Merge the support polygons by applying morphological closing and inwards smoothing.
                    // `scaled<float>(v)`: `float(v / SCALING_FACTOR)`
                    let closing_distance: f32 = scale_(self.m_support_material_closing_radius) as f32;
                    let smoothing_distance: f32 = scale_(self.m_extrusion_width) as f32;
                    //FIXME do we want to trim with the object here? On one side the columns will be thinner, on the other side support interfaces may disappear for snug supports.
                    // return diff(smooth_outward(closing(*m_support_polygons, closing_distance, SUPPORT_SURFACES_OFFSET_PARAMETERS), smoothing_distance), *m_trimming_polygons);
                    crate::mutable_polygon::smooth_outward_polygons(
                        cu::closing(&self.m_support_polygons, closing_distance, closing_distance, SUPPORT_SURFACES_OFFSET_JOIN_TYPE, SUPPORT_SURFACES_OFFSET_MITER_LIMIT),
                        smoothing_distance as i32 as i64,
                    )
                }
            }
        }

        /// SupportMaterial.cpp:1026-1044
        /// C++: `static std::vector<unsigned char> dilate_trimming_region(const std::vector<unsigned char> &trimming, const Vec2i &grid_size)`
        ///
        /// Dilate the trimming region (unmask the boundary pixels).
        pub fn dilate_trimming_region(trimming: &[u8], grid_size: Vec2i) -> Vec<u8> {
            let mut dilated: Vec<u8> = vec![0u8; trimming.len()];
            let gx = grid_size.0 as usize;
            for r in 1..grid_size.1 - 1 {
                for c in 1..grid_size.0 - 1 {
                    //int addr = c + r * m_grid_size.x();
                    // 4-neighborhood is not sufficient.
                    // 8-neighborhood
                    let mut addr = c as usize + (r as usize - 1) * gx;
                    let mut b = trimming[addr - 1] != 0 && trimming[addr] != 0 && trimming[addr + 1] != 0;
                    addr += gx;
                    b = b && trimming[addr - 1] != 0 && trimming[addr] != 0 && trimming[addr + 1] != 0;
                    addr += gx;
                    b = b && trimming[addr - 1] != 0 && trimming[addr] != 0 && trimming[addr + 1] != 0;
                    dilated[addr - gx] = b as u8;
                }
            }
            dilated
        }

        /// SupportMaterial.cpp:1046-1086
        /// C++: `static void seed_fill_block(std::vector<unsigned char> &grid, Vec2i grid_size, const std::vector<unsigned char> &trimming, const Vec2i &grid_blocks, int oversampling)`
        ///
        /// Seed fill each of the (oversampling x oversampling) block up to the dilated trimming region.
        pub fn seed_fill_block(grid: &mut [u8], grid_size: Vec2i, trimming: &[u8], grid_blocks: Vec2i, oversampling: i32) {
            let size = oversampling;
            let stride = grid_size.0;
            for block_r in 0..grid_blocks.1 {
                for block_c in 0..grid_blocks.0 {
                    // Propagate the support pixels over the macro cell up to the trimming mask.
                    let base: i32 = block_c * size + 1 + (block_r * size + 1) * stride;
                    // `PROPAGATION_STEP(offset)`
                    let mut propagation_step = |r: i32, c: i32, offset: i32| {
                        let addr = (base + r * stride + c) as usize;
                        let addr2 = (base + r * stride + c + offset) as usize;
                        if grid[addr2] != 0 && trimming[addr] == 0 && trimming[addr2] == 0 {
                            grid[addr] = 1;
                        }
                    };
                    // Top to bottom propagation.
                    for r in 0..size {
                        if r > 0 {
                            for c in 0..size {
                                propagation_step(r, c, -stride);
                            }
                        }
                        for c in 1..size {
                            propagation_step(r, c, -1);
                        }
                        for c in (0..=size - 2).rev() {
                            propagation_step(r, c, 1);
                        }
                    }
                    // Bottom to top propagation.
                    for r in (0..=size - 2).rev() {
                        for c in 0..size {
                            propagation_step(r, c, stride);
                        }
                        for c in 1..size {
                            propagation_step(r, c, -1);
                        }
                        for c in (0..=size - 2).rev() {
                            propagation_step(r, c, 1);
                        }
                    }
                }
            }
        }

        /// SupportMaterial.cpp:1089-1113 (inside `#if 0`: the engine does not compile it, island_samples() takes
        /// its samples from an offset instead)
        /// C++: `static Point island_sample(const ExPolygon &expoly)`
        ///
        /// Get some internal point of an expolygon, to be used as a representative
        /// sample to test, whether this island is inside another island.
        /// FIXME this was quick, but not sufficiently robust.
        pub fn island_sample(expoly: &ExPolygon) -> Point {
            let points = &expoly.contour.points;
            // Find the lowest point lexicographically.
            let mut pt_min: usize = 0;
            for i in 1..points.len() {
                if point_less(&points[i], &points[pt_min]) {
                    pt_min = i;
                }
            }

            // Lowest corner will always be convex, in worst case denegenerate with zero angle.
            let p1 = if pt_min == 0 { points[points.len() - 1] } else { points[pt_min - 1] };
            let p2 = points[pt_min];
            let p3 = if pt_min == points.len() - 1 { points[0] } else { points[pt_min + 1] };

            let v = ((p3.x - p2.x) + (p1.x - p2.x), (p3.y - p2.y) + (p1.y - p2.y));
            // double l2 = double(v(0))*double(v(0))+double(v(1))*double(v(1));
            let l2: f64 = (v.0 as f64).mul_add(v.0 as f64, v.1 as f64 * v.1 as f64);
            if l2 == 0. {
                return p2;
            }
            let coef: f64 = 20. / l2.sqrt();
            // Point(p2(0) + coef * v(0), p2(1) + coef * v(1)): `Point(double, double)` rounds (Point.hpp:181)
            Point::new(coef.mul_add(v.0 as f64, p2.x as f64).round_ties_even() as i64, coef.mul_add(v.1 as f64, p2.y as f64).round_ties_even() as i64)
        }

        /// SupportMaterial.cpp:1115-1141
        /// C++: `static Points island_samples(const ExPolygons &expolygons)`
        ///
        /// Sample one internal point per expolygon.
        /// FIXME this is quite an overkill to calculate a complete offset just to get a single point, but at least it is robust.
        pub fn island_samples(expolygons: &ExPolygons) -> Vec<Point> {
            let mut pts: Vec<Point> = Vec::with_capacity(expolygons.len());
            for expoly in expolygons {
                if expoly.contour.points.len() > 2 {
                    let polygons: Polygons = cu::offset_expolygon(expoly, -20., cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);
                    for poly in &polygons {
                        if !poly.points.is_empty() {
                            // Take a small fixed number of samples of this polygon for robustness.
                            let num_points = poly.points.len() as i32;
                            let num_samples = num_points.min(4);
                            let stride = num_points / num_samples;
                            let mut i = 0;
                            while i < num_points {
                                pts.push(poly.points[i as usize]);
                                i += stride;
                            }
                            break;
                        }
                    }
                }
            }
            // Sort the points lexicographically, so a binary search could be used to locate points inside a bounding box.
            pts.sort_by(|a, b| (a.x, a.y).cmp(&(b.x, b.y)));
            pts
        }

        /// SupportMaterial.cpp:912-942 (`#if defined(SLIC3R_DEBUG) && ! defined(SUPPORT_USE_AGG_RASTERIZER)`: not
        /// compiled into the engine, ported for completeness)
        /// C++: `void serialize(const std::string &path)`
        ///
        /// The file: the spacing and the angle (8 bytes each), then the support polygons and the trimming
        /// polygons, each a count (4 bytes) followed per polygon by its number of points (4 bytes) and the
        /// points (`sizeof(coord_t)` = 8 bytes a coordinate), in the byte order of the machine.
        pub fn serialize(&self, path: &str) -> std::io::Result<()> {
            use std::io::Write;
            // FILE *file = ::fopen(path.c_str(), "wb");
            let mut file = std::io::BufWriter::new(std::fs::File::create(path)?);
            file.write_all(&self.m_support_spacing.to_ne_bytes())?;
            file.write_all(&self.m_support_angle.to_ne_bytes())?;
            let mut n_polygons: u32 = self.m_support_polygons.len() as u32;
            file.write_all(&n_polygons.to_ne_bytes())?;
            for i in 0..n_polygons {
                let poly = &self.m_support_polygons[i as usize];
                let n_points: u32 = poly.points.len() as u32;
                file.write_all(&n_points.to_ne_bytes())?;
                for j in 0..n_points {
                    let pt = &poly.points[j as usize];
                    file.write_all(&pt.x.to_ne_bytes())?;
                    file.write_all(&pt.y.to_ne_bytes())?;
                }
            }
            n_polygons = self.m_trimming_polygons.len() as u32;
            file.write_all(&n_polygons.to_ne_bytes())?;
            for i in 0..n_polygons {
                let poly = &self.m_trimming_polygons[i as usize];
                let n_points: u32 = poly.points.len() as u32;
                file.write_all(&n_points.to_ne_bytes())?;
                for j in 0..n_points {
                    let pt = &poly.points[j as usize];
                    file.write_all(&pt.x.to_ne_bytes())?;
                    file.write_all(&pt.y.to_ne_bytes())?;
                }
            }
            // ::fclose(file);
            file.flush()
        }

        /// SupportMaterial.cpp:944-949 (same `#if`), with the private `SupportGridPattern() {}` of
        /// SupportMaterial.cpp:1022, which leaves the members as they are (here: zero and empty).
        /// C++: `static SupportGridPattern deserialize(const std::string &path, int which = -1)`
        pub fn deserialize(path: &str, which: i32) -> Self {
            let mut out = SupportGridPattern {
                m_style: 0,
                m_support_polygons: Vec::new(),
                m_trimming_polygons: Vec::new(),
                m_support_angle: 0.,
                m_support_spacing: 0.,
                m_extrusion_width: 0.,
                m_support_material_closing_radius: 0.,
                m_grid_size: (0, 0),
                m_pixel_size: 0.,
                m_bbox: BBox::new(),
                m_grid2: Vec::new(),
                m_grid: crate::edge_grid::EdgeGrid::new(),
            };
            out.deserialize_(path, which);
            out
        }

        /// SupportMaterial.cpp:951-1013 (same `#if`)
        /// C++: `bool deserialize_(const std::string &path, int which = -1)`
        ///
        /// Deserialization constructor. `m_support_polygons` and `m_trimming_polygons` point to
        /// `m_support_polygons_deserialized` and `m_trimming_polygons_deserialized` from here on: the two
        /// members this struct owns anyway.
        pub fn deserialize_(&mut self, path: &str, which: i32) -> bool {
            // FILE *file = ::fopen(path.c_str(), "rb"); if (file == nullptr) return false;
            let Ok(data) = std::fs::read(path) else {
                return false;
            };
            // ::fread leaves its target as it is at the end of the file; here a short file reads zeros.
            let mut pos: usize = 0;
            let mut take = |n: usize| -> [u8; 8] {
                let mut b = [0u8; 8];
                if pos + n <= data.len() {
                    b[..n].copy_from_slice(&data[pos..pos + n]);
                }
                pos += n;
                b
            };
            let read_u32 = |take: &mut dyn FnMut(usize) -> [u8; 8]| -> u32 {
                let b = take(4);
                u32::from_ne_bytes([b[0], b[1], b[2], b[3]])
            };

            self.m_support_polygons.clear();
            self.m_trimming_polygons.clear();

            self.m_support_spacing = f64::from_ne_bytes(take(8));
            self.m_support_angle = f64::from_ne_bytes(take(8));
            //FIXME
            //m_support_spacing *= 0.01 / 2;
            let mut n_polygons: u32 = read_u32(&mut take);
            self.m_support_polygons.reserve(n_polygons as usize);
            let scale: i32 = 1;
            for i in 0..n_polygons {
                let mut poly = Polygon::default();
                let n_points: u32 = read_u32(&mut take);
                poly.points.reserve(n_points as usize);
                for _j in 0..n_points {
                    let x = i64::from_ne_bytes(take(8));
                    let y = i64::from_ne_bytes(take(8));
                    poly.points.push(Point::new(x * scale as i64, y * scale as i64));
                }
                // if (which == -1 || which == i): `which` is converted to unsigned for the comparison
                if which == -1 || which as u32 == i {
                    // emplace_back(std::move(poly)): the polygon is empty afterwards
                    self.m_support_polygons.push(std::mem::take(&mut poly));
                }
                // printf("Polygon %d, area: %lf\n", i, area(poly.points)); MultiPoint.hpp:144-149
                let mut area: f64 = 0.;
                let n = poly.points.len();
                if n > 0 {
                    let mut j = n - 1;
                    for k in 0..n {
                        area += (poly.points[k].x + poly.points[j].x) as f64 * (poly.points[k].y - poly.points[j].y) as f64;
                        j = k;
                    }
                }
                println!("Polygon {}, area: {:.6}", i as i32, area);
            }
            n_polygons = read_u32(&mut take);
            self.m_trimming_polygons.reserve(n_polygons as usize);
            for _i in 0..n_polygons {
                let mut poly = Polygon::default();
                let n_points: u32 = read_u32(&mut take);
                poly.points.reserve(n_points as usize);
                for _j in 0..n_points {
                    let x = i64::from_ne_bytes(take(8));
                    let y = i64::from_ne_bytes(take(8));
                    poly.points.push(Point::new(x * scale as i64, y * scale as i64));
                }
                self.m_trimming_polygons.push(poly);
            }
            // ::fclose(file);

            // simplify_polygons(m_support_polygons_deserialized, false), ClipperUtils.cpp:1026-1041:
            // ClipperLib::SimplifyPolygons(subject, pftNonZero)
            self.m_support_polygons = cu::to_polygons(&crate::clipper::simplify_polygons(
                &cu::polygons_provider(&self.m_support_polygons),
                crate::clipper::PolyFillType::pftNonZero,
            ));
            //m_support_polygons_deserialized = to_polygons(union_ex(m_support_polygons_deserialized, false));

            // Create an EdgeGrid, initialize it with projection, initialize signed distance field.
            let grid_resolution: i64 = scale_(self.m_support_spacing) as i64;
            let mut bbox: BBox = get_extents_polygons_bbox(&self.m_support_polygons);
            bbox.offset(20.);
            // BoundingBox.cpp:257-263 `BoundingBox::align_to_grid(const coord_t cell_size)`
            if bbox.defined {
                bbox.min.x = crate::geometry::align_to_grid(bbox.min.x, grid_resolution);
                bbox.min.y = crate::geometry::align_to_grid(bbox.min.y, grid_resolution);
            }
            self.m_grid.set_bbox(bbox.to_bounding_box());
            self.m_grid.create_from_polygons(&self.m_support_polygons, grid_resolution);
            self.m_grid.calculate_sdf();
            true
        }

        /// SupportMaterial.cpp:1015 (same `#if`) `const Polygons& support_polygons() const`
        pub fn support_polygons(&self) -> &Polygons {
            &self.m_support_polygons
        }
        /// SupportMaterial.cpp:1016 (same `#if`) `const Polygons& trimming_polygons() const`
        pub fn trimming_polygons(&self) -> &Polygons {
            &self.m_trimming_polygons
        }
        /// SupportMaterial.cpp:1017 (same `#if`) `const EdgeGrid::Grid& grid() const`
        pub fn grid(&self) -> &crate::edge_grid::EdgeGrid {
            &self.m_grid
        }

        /// `m_style`: the style the constructor settled on (see `SMS_*`).
        pub fn style(&self) -> i32 {
            self.m_style
        }

        /// The members the grid is made of, for the tests: the size of the raster, the size of a pixel, the
        /// corners of its box and the raster.
        pub fn grid_state(&self) -> (Vec2i, f64, Point, Point, &[u8]) {
            (self.m_grid_size, self.m_pixel_size, self.m_bbox.min, self.m_bbox.max, &self.m_grid2)
        }
    }
}
