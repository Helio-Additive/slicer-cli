//! Faithful 1:1 port of `PrintObjectSlice.cpp` (BambuStudio libslic3r).
//!
//! C++ Reference:
//! - src/libslic3r/PrintObjectSlice.cpp
//!
//! PORTING STATUS: partial.
//!
//! Most of `PrintObjectSlice.cpp` depends on infrastructure that does not yet
//! exist in this Rust crate, so those symbols are intentionally NOT faked here.
//! The blocked symbols, and the precise reason each is blocked, are:
//!
//!   * `slice_volume` (both overloads)    — needs `ModelVolume`, `MeshSlicingParamsEx`
//!                                          (trafo / closing_radius / mode / resolution /
//!                                          extra_offset) and `slice_mesh_ex`. The crate
//!                                          only has `triangle_mesh_slicer::slice_mesh(mesh, zs)`
//!                                          (no params struct, no per-volume transform).
//!   * `model_volume_needs_slicing`       — needs `ModelVolume` / `ModelVolumeType`.
//!                                          The crate's `Model`/`ModelObject` carry a single
//!                                          mesh and have no `ModelVolume` concept.
//!   * `slice_volumes_inner`              — needs `ModelVolume`, `PrintObjectRegions::
//!                                          LayerRangeRegions`, `VolumeSlices`, spiral_mode
//!                                          config threading, `MeshSlicingParamsEx`.
//!   * `volume_slices_find_by_id`         — needs `VolumeSlices` / `ObjectID`.
//!   * `overlap_in_xy`                    — needs `PrintObjectRegions::BoundingBox`.
//!   * `layer_range_first` / `layer_range_next`
//!                                        — need `PrintObjectRegions::LayerRangeRegions`.
//!   * `slices_to_regions`                — needs `VolumeSlices`, `PrintObjectRegions`,
//!                                          `VolumeRegion`, region clipping infra (tbb).
//!   * `doesVolumeIntersect`              — needs `VolumeSlices` + `overlaps`.
//!   * `groupingVolumes`                  — needs `VolumeSlices` / `groupedVolumeSlices`.
//!   * `findPartVolumes`                  — needs `VolumeSlices` + `ModelVolume`.
//!   * `applyNegtiveVolumes`              — needs `VolumeSlices` / `groupedVolumeSlices`.
//!   * `reGroupingLayerPolygons`          — needs `groupedVolumeSlices`.
//!   * `groupingVolumesForBrim`           — needs `firstLayerObjSliceByVolume/ByGroups`.
//!   * `PrintObject::slice`               — already implemented (simplified) in
//!                                          `print_object.rs`; the faithful version depends on
//!                                          `slice_volumes` and `groupingVolumesForBrim`
//!                                          (`new_layers` and `fix_slicing_errors` are now
//!                                          ported below).
//!   * `apply_mm_segmentation`            — needs `multi_material_segmentation_by_painting`,
//!                                          `PrintObjectRegions::painted_regions`.
//!   * `apply_fuzzy_skin_segmentation`    — needs `fuzzy_skin_segmentation_by_painting`,
//!                                          `PrintObjectRegions::fuzzy_skin_painted_regions`.
//!   * `PrintObject::slice_volumes`       — needs `ModelVolume`, `VolumeSlices`,
//!                                          `slices_to_regions`, MMU / fuzzy-skin segmentation,
//!                                          `InterlockingGenerator`, multi-region per-layer
//!                                          XY-size / elephant-foot compensation pipeline.
//!   * `PrintObject::slice_support_volumes`
//!                                        — needs `ModelVolume` / `MeshSlicingParamsEx`.
//!
//! Ported faithfully below:
//!   * `PrintObject::_shrink_contour_holes` — pure ExPolygon/Polygon geometry helper.
//!   * `new_layers`                         — unblocked by the config-hierarchy wiring
//!                                            (`PrintObject::slicing_parameters()` accessor).
//!   * `fix_slicing_errors`                 — unblocked by the config-hierarchy wiring
//!                                            (`LayerRegion::flow` over the stored Arcs).

use crate::clipper_utils::{
    difference, offset_expolygon, offset_polygon, union_ex, union_polygons_ex, OffsetJoinType,
};
use crate::flow::FlowRole;
use crate::geometry::{ExPolygons, Polygon, Polygons};
use crate::layer::Layer;
use crate::print_object::PrintObject;
use crate::surface::SurfaceType;
use crate::{Coord, CoordF, Result};

/// Initialize the raft-offset object layers from the (bottom, top) Z pairs
/// produced by `generate_object_layers`.
/// PrintObjectSlice.cpp:23-46
/// C++: LayerPtrs new_layers(PrintObject *print_object,
///                           const std::vector<coordf_t> &object_layers)
///
/// Porting notes:
///   - C++ links layers through `upper_layer` / `lower_layer` pointers; the
///     crate's `Layer` carries `upper_layer_id` / `lower_layer_id` Vec indices
///     instead, so the `prev` pointer dance becomes index links into `out`.
///   - C++ passes the `print_object` parent pointer to the `Layer` ctor
///     (Layer.hpp); the crate's `Layer::new` takes an `object_id` placeholder
///     in that slot — `0`, matching `slicer::Slicer::build_layers`. The parent
///     reach-through (`layer->object()`) is provided separately by
///     `PrintObject::wire_layer_hierarchy` at sync points.
pub fn new_layers(
    print_object: &PrintObject,
    // Object layers (pairs of bottom/top Z coordinate), without the raft.
    object_layers: &[CoordF],
) -> Vec<Layer> {
    // PrintObjectSlice.cpp:28-29
    let mut out: Vec<Layer> = Vec::with_capacity(object_layers.len());
    // PrintObjectSlice.cpp:30
    // C++: auto id = int(print_object->slicing_parameters().raft_layers());
    let mut id = print_object.slicing_parameters().raft_layers();
    // PrintObjectSlice.cpp:31
    // C++: coordf_t zmin = print_object->slicing_parameters().object_print_z_min;
    let zmin: CoordF = print_object.slicing_parameters().object_print_z_min;
    // PrintObjectSlice.cpp:32: Layer *prev = nullptr; (index links below)
    // PrintObjectSlice.cpp:33
    let mut i_layer = 0;
    while i_layer < object_layers.len() {
        // PrintObjectSlice.cpp:34
        let lo = object_layers[i_layer];
        // PrintObjectSlice.cpp:35
        let hi = object_layers[i_layer + 1];
        // PrintObjectSlice.cpp:36
        let slice_z = 0.5 * (lo + hi);
        // PrintObjectSlice.cpp:37
        // C++: Layer *layer = new Layer(id ++, print_object, hi - lo, hi + zmin, slice_z);
        let mut layer = Layer::new(id, 0, hi - lo, hi + zmin, slice_z);
        id += 1;
        // PrintObjectSlice.cpp:39-42
        // C++: if (prev != nullptr) { prev->upper_layer = layer; layer->lower_layer = prev; }
        let k = out.len();
        if k > 0 {
            out[k - 1].set_upper_layer(Some(k));
            layer.set_lower_layer(Some(k - 1));
        }
        // PrintObjectSlice.cpp:38
        out.push(layer);
        // PrintObjectSlice.cpp:43: prev = layer; (implicit in the index links)
        i_layer += 2;
    }
    // PrintObjectSlice.cpp:45
    out
}

/// Replace bad slices by slices reconstructed from the upper/lower layer and
/// drop empty layers from the bottom of the stack.
/// PrintObjectSlice.cpp:650-770
/// C++: std::string fix_slicing_errors(PrintObject* object, LayerPtrs &layers,
///          const std::function<void()> &throw_if_canceled, int &firstLayerReplacedBy)
///
/// Porting notes:
///   - `object` is never read by the C++ body either; kept for signature parity.
///   - C++ `LayerRegion::flow(frExternalPerimeter)` reads `m_layer->height`
///     through the parent pointer; the crate's `LayerRegion::flow` threads the
///     layer height explicitly and returns `Result` (C++ throws), so this
///     function returns `Result<String>` and the `get_ext_peri_width` /
///     `max_element` pair is evaluated as one width-per-layer pass.
///   - The C++ repair loop is a `tbb::parallel_for` over `buggy_layers`
///     (PrintObjectSlice.cpp:688-744); ported sequentially like the rest of
///     the crate, which also collapses `std::atomic<bool> is_replaced` to a
///     plain `bool`. The `const Surfaces*` upper/lower pointers become source
///     layer indices whose expolygons are cloned (the C++ `emplace_back`
///     copies them too) before the buggy layer is mutated.
///   - C++ links layers through pointers, which survive `layers.erase`; the
///     crate's Vec-index links must be shifted down by one after each front
///     removal, and the new front's lower link becomes `None` — exactly the
///     C++ `layers.front()->lower_layer = nullptr` (PrintObjectSlice.cpp:755).
pub fn fix_slicing_errors<F: Fn() -> Result<()>>(
    _object: &PrintObject,
    layers: &mut Vec<Layer>,
    throw_if_canceled: F,
    first_layer_replaced_by: &mut i32,
) -> Result<String> {
    // PrintObjectSlice.cpp:652
    let mut error_msg = String::new(); //BBS

    // PrintObjectSlice.cpp:654
    if layers.is_empty() {
        return Ok(error_msg);
    }

    // Collect layers with slicing errors.
    // These layers will be fixed in parallel.
    // PrintObjectSlice.cpp:658-659
    let mut buggy_layers: Vec<usize> = Vec::with_capacity(layers.len());
    // BBS: get largest external perimenter width of all layers
    // PrintObjectSlice.cpp:661
    // C++: auto get_ext_peri_width = [](Layer* layer) {return layer->m_regions.empty() ? 0 :
    //          layer->m_regions[0]->flow(frExternalPerimeter).scaled_width(); };
    let mut ext_peri_widths: Vec<Coord> = Vec::with_capacity(layers.len());
    for layer in layers.iter() {
        ext_peri_widths.push(if layer.regions().is_empty() {
            0
        } else {
            layer.regions()[0]
                .flow(FlowRole::ExternalPerimeter, layer.height)?
                .scaled_width()
        });
    }
    // PrintObjectSlice.cpp:662
    // C++: auto it = std::max_element(layers.begin(), layers.end(), ...);
    let max_ext_peri_width = ext_peri_widths.iter().copied().max().unwrap_or(0);
    // PrintObjectSlice.cpp:663
    // C++: coord_t thresh = get_ext_peri_width(*it) * 0.5; (int * double, truncated back)
    let thresh: Coord = (max_ext_peri_width as f64 * 0.5) as Coord; // half of external perimeter width  // 0.5 * scale_(this->config().line_width);
    // PrintObjectSlice.cpp:664
    for idx_layer in 0..layers.len() {
        // BBS: detect empty layers (layers with very small regions) and mark them as problematic, then these layers will copy the nearest good layer
        // PrintObjectSlice.cpp:666
        let layer = &mut layers[idx_layer];
        // PrintObjectSlice.cpp:667
        let mut lslices: ExPolygons = ExPolygons::new();
        // PrintObjectSlice.cpp:668
        for region_id in 0..layer.region_count() {
            // PrintObjectSlice.cpp:669
            let layerm = layer.get_region(region_id).unwrap();
            // PrintObjectSlice.cpp:670
            for surface in &layerm.slices.surfaces {
                // PrintObjectSlice.cpp:671
                // C++: auto expoly = offset_ex(surface.expolygon, -thresh);
                let expoly =
                    offset_expolygon(&surface.expolygon, -(thresh as CoordF), OffsetJoinType::Miter);
                // PrintObjectSlice.cpp:672
                // C++: lslices.insert(lslices.begin(), expoly.begin(), expoly.end());
                lslices.splice(0..0, expoly);
            }
        }
        // PrintObjectSlice.cpp:675-677
        if lslices.is_empty() {
            layer.slicing_errors = true;
        }

        // PrintObjectSlice.cpp:679-680
        if layers[idx_layer].slicing_errors {
            buggy_layers.push(idx_layer);
        } else {
            // PrintObjectSlice.cpp:683
            break; // only detect empty layers near bed
        }
    }

    // PrintObjectSlice.cpp:686: "Slicing objects - fixing slicing errors in parallel - begin"
    // PrintObjectSlice.cpp:687
    let mut is_replaced = false;
    // PrintObjectSlice.cpp:688-744: tbb::parallel_for over buggy_layers, sequential here.
    for buggy_layer_idx in 0..buggy_layers.len() {
        // PrintObjectSlice.cpp:692
        throw_if_canceled()?;
        // PrintObjectSlice.cpp:693
        let idx_layer = buggy_layers[buggy_layer_idx];
        // BBS: only replace empty layers lower than 1mm
        // PrintObjectSlice.cpp:695
        let thresh_empty_layer_height: CoordF = 1.0;
        // PrintObjectSlice.cpp:696-698
        if layers[idx_layer].print_z >= thresh_empty_layer_height {
            continue;
        }
        // PrintObjectSlice.cpp:699
        debug_assert!(layers[idx_layer].slicing_errors);
        // Try to repair the layer surfaces by merging all contours and all holes from neighbor layers.
        // BOOST_LOG_TRIVIAL(trace) << "Attempting to repair layer" << idx_layer;
        // PrintObjectSlice.cpp:702
        for region_id in 0..layers[idx_layer].region_count() {
            // Find the first valid layer below / above the current layer.
            // PrintObjectSlice.cpp:705-706
            let mut upper_surfaces: Option<usize> = None;
            let mut lower_surfaces: Option<usize> = None;
            //BBS: only repair empty layers lowers than 1mm
            // PrintObjectSlice.cpp:708-714
            for j in idx_layer + 1..layers.len() {
                if !layers[j].slicing_errors {
                    upper_surfaces = Some(j);
                    break;
                }
                if layers[j].print_z >= thresh_empty_layer_height {
                    break;
                }
            }
            // PrintObjectSlice.cpp:715-721
            for j in (0..idx_layer).rev() {
                if layers[j].print_z >= thresh_empty_layer_height {
                    continue;
                }
                if !layers[j].slicing_errors {
                    lower_surfaces = Some(j);
                    break;
                }
            }
            // Collect outer contours and holes from the valid layers above & below.
            // PrintObjectSlice.cpp:723-726
            let mut expolys: ExPolygons = ExPolygons::new();
            // PrintObjectSlice.cpp:727-730
            if let Some(j) = upper_surfaces {
                for surface in &layers[j].regions()[region_id].slices.surfaces {
                    expolys.push(surface.expolygon.clone());
                }
            }
            // PrintObjectSlice.cpp:731-734
            if let Some(j) = lower_surfaces {
                for surface in &layers[j].regions()[region_id].slices.surfaces {
                    expolys.push(surface.expolygon.clone());
                }
            }
            // PrintObjectSlice.cpp:735-739
            if !expolys.is_empty() {
                //BBS
                is_replaced = true;
                // C++: layerm->slices.set(union_ex(expolys), stInternal);
                layers[idx_layer]
                    .get_region_mut(region_id)
                    .unwrap()
                    .slices
                    .set_expolygons(union_ex(&expolys), SurfaceType::Internal);
            }
        }
        // Update layer slices after repairing the single regions.
        // PrintObjectSlice.cpp:742
        layers[idx_layer].make_slices();
    }
    // PrintObjectSlice.cpp:745
    throw_if_canceled()?;
    // PrintObjectSlice.cpp:746: "Slicing objects - fixing slicing errors in parallel - end"

    // PrintObjectSlice.cpp:748-749
    if is_replaced {
        error_msg = "Empty layers around bottom are replaced by nearest normal layers.".to_string();
    }

    // remove empty layers from bottom
    // PrintObjectSlice.cpp:752
    while !layers.is_empty() && (layers[0].lslices.is_empty() || layers[0].empty()) {
        // PrintObjectSlice.cpp:753-754
        // C++: delete layers.front(); layers.erase(layers.begin());
        layers.remove(0);
        // PrintObjectSlice.cpp:755
        // C++: layers.front()->lower_layer = nullptr;
        // The crate's upper/lower links are Vec indices standing in for the
        // C++ layer pointers (which survive the erase); shift every remaining
        // index down by one — the new front's lower link saturates to None,
        // which is exactly the C++ nulling above.
        for layer in layers.iter_mut() {
            layer.lower_layer_id = layer.lower_layer_id.and_then(|id| id.checked_sub(1));
            layer.upper_layer_id = layer.upper_layer_id.and_then(|id| id.checked_sub(1));
        }
        // PrintObjectSlice.cpp:756-757
        // C++: for (size_t i = 0; i < layers.size(); ++ i) layers[i]->set_id(layers[i]->id() - 1);
        for i in 0..layers.len() {
            let id = layers[i].id();
            // size_t arithmetic in C++ wraps on underflow.
            layers[i].set_id(id.wrapping_sub(1));
        }
    }

    //BBS
    // PrintObjectSlice.cpp:761-762
    if error_msg.is_empty() && !buggy_layers.is_empty() {
        error_msg = "The model has too many empty layers.".to_string();
    }

    // BBS: first layer slices are sorted by volume group, if the first layer is empty and replaced by the 2nd layer
    // the later will be stored in "object->firstLayerObjGroupsMod()"
    // PrintObjectSlice.cpp:766-767
    if !buggy_layers.is_empty() && buggy_layers[0] == 0 && layers.len() > 1 {
        *first_layer_replaced_by = 1;
    }

    // PrintObjectSlice.cpp:769
    Ok(error_msg)
}

impl PrintObject {
    //BBS: this function is used to offset contour and holes of expolygons seperately by different value
    // PrintObjectSlice.cpp:1348
    pub fn _shrink_contour_holes(
        &self,
        contour_delta: f64,
        hole_delta: f64,
        polys: &ExPolygons,
    ) -> ExPolygons {
        // PrintObjectSlice.cpp:1350
        let mut new_ex_polys: ExPolygons = Vec::new();
        // PrintObjectSlice.cpp:1351
        for ex_poly in polys {
            // PrintObjectSlice.cpp:1352
            let mut contours: Polygons = Vec::new();
            // PrintObjectSlice.cpp:1353
            let mut holes: Polygons = Vec::new();
            //BBS: modify hole
            // PrintObjectSlice.cpp:1355
            for hole in &ex_poly.holes {
                // PrintObjectSlice.cpp:1356
                if hole_delta != 0.0 {
                    // PrintObjectSlice.cpp:1357: for (Polygon& newHole : offset(hole, -hole_delta))
                    // C++ `offset(const Polygon&, delta)` returns Polygons; the crate's
                    // `offset_polygon` returns ExPolygons whose contours are those Polygons.
                    for new_hole_ex in offset_polygon(hole, -hole_delta, OffsetJoinType::Miter) {
                        let mut new_hole: Polygon = new_hole_ex.contour;
                        // PrintObjectSlice.cpp:1358
                        new_hole.make_counter_clockwise();
                        // PrintObjectSlice.cpp:1359
                        holes.push(new_hole);
                    }
                } else {
                    // PrintObjectSlice.cpp:1362
                    holes.push(hole.clone());
                    // PrintObjectSlice.cpp:1363
                    holes.last_mut().unwrap().make_counter_clockwise();
                }
            }
            //BBS: modify contour
            // PrintObjectSlice.cpp:1367
            if contour_delta != 0.0 {
                // PrintObjectSlice.cpp:1368
                let new_contours = offset_polygon(&ex_poly.contour, contour_delta, OffsetJoinType::Miter);
                // PrintObjectSlice.cpp:1369
                if new_contours.is_empty() {
                    // PrintObjectSlice.cpp:1370
                    continue;
                }
                // PrintObjectSlice.cpp:1371
                contours.extend(new_contours.into_iter().map(|e| e.contour));
            } else {
                // PrintObjectSlice.cpp:1373
                contours.push(ex_poly.contour.clone());
            }
            // PrintObjectSlice.cpp:1375: ExPolygons temp = diff_ex(union_(contours), union_(holes));
            // C++ `union_(Polygons)` returns Polygons; here we union at the ExPolygon level
            // (the crate's `union_polygons_ex`, which performs the same ClipperLib non-zero
            // union) and take the boolean difference of the two resulting ExPolygon sets.
            let union_contours: ExPolygons = union_polygons_ex(&contours);
            let union_holes: ExPolygons = union_polygons_ex(&holes);
            let temp: ExPolygons = difference(&union_contours, &union_holes);
            // PrintObjectSlice.cpp:1376
            new_ex_polys.extend(temp.into_iter());
        }
        // PrintObjectSlice.cpp:1378
        union_ex(&new_ex_polys)
    }
}
