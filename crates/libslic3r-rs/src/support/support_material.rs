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

// ============================================================================
// SupportMaterial.cpp:171-332, 664-1171 — the support grid: the rasterizer, the contours of the raster,
// SupportGridParams and SupportGridPattern.
//
// `#define SUPPORT_USE_AGG_RASTERIZER` (SupportMaterial.cpp:22) is set, so the branch of the Anti-Grain
// rasterizer is the one ported, over the crate's port of AGG (`crate::sla::agg`). The code under
// `SLIC3R_DEBUG` (SVG and PNG dumps, `serialize` / `deserialize`, which also need the EdgeGrid branch)
// is not compiled into the engine and is not ported.
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
