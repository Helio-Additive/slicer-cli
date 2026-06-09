//! Support material generation.
//!
//! C++ Reference:
//! - Support/SupportMaterial.hpp
//! - Support/SupportMaterial.cpp
//!
//! Faithful 1:1 line-by-line port of the self-contained, leaf-level symbols from
//! `Support/SupportMaterial.{hpp,cpp}`. These are the constants, the deque
//! allocator helpers, the `Layer`-only / `SupportGeneratorLayer`-only geometry
//! collectors, and the conservative contact-layer merge routine — i.e. the parts
//! that do NOT pull in the still-unported support pipeline graph
//! (`PrintObject::print()`, the `Layer`/`LayerRegion` pointer graph,
//! `ExtrusionEntityCollection` role traversal, `Flow::bridging_flow`, the AGG
//! native rasterizer, `EdgeGrid` SDF, TBB).
//!
//! coord_t -> i64, coordf_t -> f64 per crate conventions; scale_() -> scale().
//!
//! Blocked symbols (genuinely require not-yet-ported / native deps; see notes):
//! - `PrintObjectSupportMaterial::generate` and all of its member methods
//!   (`buildplate_covered`, `top_contact_layers`, `bottom_contact_layers_and_layer_support_areas`,
//!   `trim_top_contacts_by_bottom_contacts`, `raft_and_intermediate_support_layers`,
//!   `generate_base_layers`, `trim_support_layers_by_object`)
//!   — need `PrintObject::print()->config()`/`canceled()`,
//!     `object.slice_support_enforcers()/slice_support_blockers()`,
//!     `object.project_and_append_custom_facets()`, `object.total_layer_count()`,
//!     `object.get_layer()`, and the `Layer`/`LayerRegion` C++ pointer-graph API
//!     (`Layer::lower_layer`/`upper_layer` pointers, `Layer::sharp_tails` as
//!     `ExPolygons` with `sharp_tails_height` vector, `LayerRegion::flow()`,
//!     `LayerRegion::bridging_flow()`, `LayerRegion::region()`), none of which
//!     are available with compatible signatures in the current Rust crate.
//! - The AGG rasterizer path (`rasterize_polygons`, `contours_simplified`,
//!   `SupportGridPattern`, `dilate_trimming_region`, `seed_fill_block`,
//!   `island_samples`) — `#define SUPPORT_USE_AGG_RASTERIZER` selects the native
//!   `agg/*` rasterizer (wasm-unsafe, not present); the non-AGG branch needs the
//!   `EdgeGrid` SDF API which is likewise not wired for this use.
//! - `detect_overhangs`, `detect_contacts`, `sync_gap_with_object_layer`,
//!   `new_contact_layer`, `fill_contact_layer`, `OverhangCluster`/`add_overhang`,
//!   `detect_bottom_contacts`, `project_support_to_grid`, `SupportAnnotations`,
//!   `SlicesMarginCache`, `SupportMaterialInternal::*` — all transitively depend
//!   on the blocked `Layer`/`LayerRegion`/`PrintObject` graph and/or the AGG grid.
//! - `SupportGridParams` — needs `PrintObjectConfig::support_angle` (absent in the
//!   Rust `PrintObjectConfig`) and `support_style` typed as `SupportMaterialStyle`
//!   (the Rust field is `TreeSupportStyle`); cannot be faithfully reproduced yet.
//! - `export_print_z_polygons*_to_svg` — SVG debug helpers (SLIC3R_DEBUG only).

use crate::geometry::Polygons;
use crate::libslic3r::EPSILON;
use crate::slicing::SlicingParams as SlicingParameters;
use crate::support::support_layer::{
    SupporLayerType, SupportGeneratorLayer, SupportGeneratorLayerStorage, SupportGeneratorLayersPtr,
};
use crate::surface::SurfaceType;

// SupportMaterial.cpp:54-57
// how much we extend support around the actual contact area
//FIXME this should be dependent on the nozzle diameter!
// BBS: change from 1.5 to 1.2
pub const SUPPORT_MATERIAL_MARGIN: f64 = 1.2;

// SupportMaterial.cpp:59-60
// Increment used to reach MARGIN in steps to avoid trespassing thin objects
pub const NUM_MARGIN_STEPS: usize = 3;

// SupportMaterial.cpp:62-64
// Dimensions of a tree-like structure to save material
pub const PILLAR_SIZE: f64 = 2.5;
pub const PILLAR_SPACING: f64 = 10.0;

// SupportMaterial.cpp:66-68
//#define SUPPORT_SURFACES_OFFSET_PARAMETERS ClipperLib::jtMiter, 3.
//#define SUPPORT_SURFACES_OFFSET_PARAMETERS ClipperLib::jtMiter, 1.5
// #define SUPPORT_SURFACES_OFFSET_PARAMETERS ClipperLib::jtSquare, 0.
// NOTE: in C++ this is a token macro expanding to two ClipperLib args
// (ClipperLib::jtSquare, 0.). Represented at call sites in the crate by
// OffsetJoinType::Square with a 0. miter limit.

// SupportMaterial.cpp:70
pub const SUPPORT_WITH_SHEATH: bool = false;

// ============================================================================
// SupportMaterial.cpp:341-362 — deque allocator helpers.
//
// In C++ these `layer_allocate` overloads push a fresh `SupportGeneratorLayer`
// into a `std::deque` (used as a stable-address allocator) and return a
// reference to it. The Rust port routes this through
// `SupportGeneratorLayerStorage`, which models the deque + the `tbb::spin_mutex`
// guarded variant (single-threaded here, so both overloads are equivalent).
// ============================================================================

// SupportMaterial.cpp:341-349
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

// SupportMaterial.cpp:351-362
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

// SupportMaterial.cpp:364-367
// inline void layers_append(SupportGeneratorLayersPtr &dst, const SupportGeneratorLayersPtr &src)
// {
//     dst.insert(dst.end(), src.begin(), src.end());
// }
pub fn layers_append(dst: &mut SupportGeneratorLayersPtr, src: &SupportGeneratorLayersPtr) {
    dst.extend_from_slice(src);
}

// SupportMaterial.cpp:369-372
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
// SupportMaterial.cpp:587-615 — Layer slice collectors.
// ============================================================================

// SupportMaterial.cpp:587-604
// Collect all polygons of all regions in a layer with a given surface type.
// Polygons collect_region_slices_by_type(const Layer &layer, SurfaceType surface_type)
pub fn collect_region_slices_by_type(
    layer: &crate::layer::Layer,
    surface_type: SurfaceType,
) -> Polygons {
    // SupportMaterial.cpp:590-595
    // 1) Count the new polygons first.
    let mut n_polygons_new: usize = 0;
    for region in layer.regions() {
        for surface in &region.slices.surfaces {
            if surface.surface_type == surface_type {
                n_polygons_new += surface.expolygon.holes.len() + 1;
            }
        }
    }
    // SupportMaterial.cpp:596-602
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

// SupportMaterial.cpp:606-615
// Collect outer contours of all slices of this layer.
// This is useful for calculating the support base with holes filled.
// Polygons collect_slices_outer(const Layer &layer)
pub fn collect_slices_outer(layer: &crate::layer::Layer) -> Polygons {
    let mut out: Polygons = Polygons::new();
    // SupportMaterial.cpp:610
    // out.reserve(out.size() + layer.lslices.size());
    out.reserve(out.len() + layer.lslices.len());
    // SupportMaterial.cpp:611-613
    // for (const ExPolygon &expoly : layer.lslices)
    //     out.emplace_back(expoly.contour);
    for expoly in &layer.lslices {
        out.push(expoly.contour.clone());
    }
    out
}

// ============================================================================
// SupportMaterial.cpp:1350-1355 — BBS well-supported / sharp-tail constants.
// ============================================================================

// SupportMaterial.cpp:1351
// static const double length_thresh_well_supported = scale_(6);  // min: 6mm
pub fn length_thresh_well_supported() -> f64 {
    crate::scale(6.0) as f64
}
// SupportMaterial.cpp:1352
// static const double area_thresh_well_supported = SQ(length_thresh_well_supported);  // min: 6x6=36mm^2
pub fn area_thresh_well_supported() -> f64 {
    let l = length_thresh_well_supported();
    l * l
}
// SupportMaterial.cpp:1353
// static const double sharp_tail_xy_gap = 0.2f;
pub const SHARP_TAIL_XY_GAP: f64 = 0.2;
// SupportMaterial.cpp:1354
// static const double no_overlap_xy_gap = 0.2f;
pub const NO_OVERLAP_XY_GAP: f64 = 0.2;
// SupportMaterial.cpp:1355
// static const double sharp_tail_max_support_height = 16.f;
pub const SHARP_TAIL_MAX_SUPPORT_HEIGHT: f64 = 16.0;

// ============================================================================
// SupportMaterial.cpp:1973-2016 — merge_contact_layers.
//
// Merge close contact layers conservatively: If two layers are closer than the
// minimum allowed print layer height (the min_layer_height parameter), the top
// contact layer is merged into the bottom contact layer.
//
// Operates purely on `SupportGeneratorLayer` + `SlicingParameters`; the
// `SupportGeneratorLayersPtr` is a `Vec<usize>` of indices into `layer_storage`,
// so the function takes the storage to resolve the "pointers".
// ============================================================================

// SupportMaterial.cpp:1975
// static void merge_contact_layers(const SlicingParameters &slicing_params, double support_layer_height_min, SupportGeneratorLayersPtr &layers)
#[allow(unused_assignments)] // C++ `i = j;` is the last statement of the for-loop body (SupportMaterial.cpp:2012); kept verbatim.
pub fn merge_contact_layers(
    slicing_params: &SlicingParameters,
    support_layer_height_min: f64,
    layers: &mut SupportGeneratorLayersPtr,
    layer_storage: &mut SupportGeneratorLayerStorage,
) {
    // SupportMaterial.cpp:1977-1978
    // Sort the layers, as one layer may produce bridging and non-bridging contact layers with different print_z.
    // std::sort(layers.begin(), layers.end(), [](const SupportGeneratorLayer *l1, const SupportGeneratorLayer *l2) { return l1->print_z < l2->print_z; });
    layers.sort_by(|&l1, &l2| {
        layer_storage[l1]
            .print_z
            .partial_cmp(&layer_storage[l2].print_z)
            .unwrap()
    });

    // SupportMaterial.cpp:1980-1981
    let mut i: i32 = 0;
    let mut k: i32 = 0;
    {
        // SupportMaterial.cpp:1983-1985
        // Find the span of layers, which are to be printed at the first layer height.
        let mut j: i32 = 0;
        while (j as usize) < layers.len()
            && layer_storage[layers[j as usize]].print_z
                < slicing_params.first_print_layer_height + support_layer_height_min - EPSILON
        {
            j += 1;
        }
        // SupportMaterial.cpp:1986
        if j > 0 {
            // SupportMaterial.cpp:1988-1990
            // Merge the layers layers (0) to (j - 1) into the layers[0].
            // SupportGeneratorLayer &dst = *layers.front();
            // for (int u = 1; u < j; ++ u)
            //     dst.merge(std::move(*layers[u]));
            let dst_idx = layers[0];
            for u in 1..j {
                let src = std::mem::take(&mut layer_storage[layers[u as usize]]);
                layer_storage[dst_idx].merge(src);
            }
            // SupportMaterial.cpp:1992-1994
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
            // SupportMaterial.cpp:1995
            k += 1;
        }
        // SupportMaterial.cpp:1997
        i = j;
    }
    // SupportMaterial.cpp:1999
    while i < layers.len() as i32 {
        // SupportMaterial.cpp:2001-2003
        // Find the span of layers closer than m_support_layer_height_min.
        let mut j: i32 = i + 1;
        let zmax = layer_storage[layers[i as usize]].print_z + support_layer_height_min + EPSILON;
        while (j as usize) < layers.len() && layer_storage[layers[j as usize]].print_z < zmax {
            j += 1;
        }
        // SupportMaterial.cpp:2004
        if i + 1 < j {
            // SupportMaterial.cpp:2006-2008
            // Merge the layers layers (i + 1) to (j - 1) into the layers[i].
            // SupportGeneratorLayer &dst = *layers[i];
            // for (int u = i + 1; u < j; ++ u)
            //     dst.merge(std::move(*layers[u]));
            let dst_idx = layers[i as usize];
            for u in (i + 1)..j {
                let src = std::mem::take(&mut layer_storage[layers[u as usize]]);
                layer_storage[dst_idx].merge(src);
            }
        }
        // SupportMaterial.cpp:2010-2011
        if k < i {
            layers[k as usize] = layers[i as usize];
        }
        // SupportMaterial.cpp:2012
        i = j;
        // SupportMaterial.cpp:1999 (++ k)
        k += 1;
    }
    // SupportMaterial.cpp:2014-2015
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
