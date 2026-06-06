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
//!   * `new_layers`                       — needs `PrintObject::slicing_parameters()`
//!                                          raft_layers / object_print_z_min and the
//!                                          `Layer` (id, print_z, slice_z, upper/lower
//!                                          links) constructor wiring used here. The
//!                                          crate's `Layer` exists but layer creation is
//!                                          driven by `slicer::Slicer`, not `new_layers`.
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
//!   * `fix_slicing_errors`               — needs `Layer::slicing_errors`, `LayerRegion::flow`,
//!                                          `Surfaces` repair + `make_slices`; partially exists
//!                                          inline in `print_object::slice()` but not as a
//!                                          faithful standalone (depends on multi-region layers
//!                                          which the slicer does not yet produce).
//!   * `groupingVolumesForBrim`           — needs `firstLayerObjSliceByVolume/ByGroups`.
//!   * `PrintObject::slice`               — already implemented (simplified) in
//!                                          `print_object.rs`; the faithful version depends on
//!                                          all of the above (`new_layers`, `slice_volumes`,
//!                                          `fix_slicing_errors`, `groupingVolumesForBrim`).
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
//! The single symbol that is fully tractable today is
//! `PrintObject::_shrink_contour_holes`, a pure ExPolygon/Polygon geometry helper.
//! It is ported faithfully below.

use crate::clipper_utils::{difference, offset_polygon, union_ex, union_polygons_ex, OffsetJoinType};
use crate::geometry::{ExPolygons, Polygon, Polygons};
use crate::print_object::PrintObject;

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
