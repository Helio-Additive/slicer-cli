//! Faithful 1:1 port of BambuStudio `src/libslic3r/LayerRegion.cpp`.
//!
//! C++ Reference:
//! - LayerRegion.cpp
//! - Layer.hpp (declares `class LayerRegion` and the free `expand_*` helpers)
//!
//! # Organization note (why this is not a single self-contained module)
//!
//! In C++, `LayerRegion.cpp` defines:
//!   1. `LayerRegion::` *member* methods (`flow`, `bridging_flow`,
//!      `slices_to_fill_surfaces_clipped`, `auto_circle_compensation`,
//!      `make_perimeters`, `process_external_surfaces`, `prepare_fill_surfaces`,
//!      `infill_area_threshold`, `trim_surfaces`,
//!      `elephant_foot_compensation_step`, `export_region_*`,
//!      `simplify_*`).
//!   2. File-local free functions used by `process_external_surfaces`
//!      (`fill_surfaces_extract_expolygons`, `group_id`, `get_grouped_bridges`,
//!      `detect_bridge_directions`, `merge_bridges`, `expand_expolygons`,
//!      `expand_bridges_detect_orientations`, `expand_merge_surfaces`) plus the
//!      `Bridge` / `ExpansionResult` helper structs.
//!
//! In this crate the `LayerRegion` *struct* and its member methods already live
//! in [`crate::layer`] (its natural home next to `Layer`), and the free
//! expansion helpers in (2) were ported into [`crate::region_expansion`]
//! (matching `Layer.hpp`'s declaration of `expand_bridges_detect_orientations`
//! and `expand_merge_surfaces`). To avoid duplicating those definitions (which
//! would not compile), this module:
//!   - owns the file-level constants `max_deviation` / `max_variance`
//!     (`LayerRegion.cpp:16-17`),
//!   - re-exports the ported free helpers so callers can reach them through the
//!     `layer_region` path that mirrors the C++ filename, and
//!   - ports the self-contained `LayerRegion::simplify_*` member logic as free
//!     functions operating on `ExtrusionEntityCollection` (the struct method
//!     wrappers `simplify_wall_extrusion_entity` / `simplify_infill_extrusion_entity`
//!     in `Layer.hpp` just forward to these).

use crate::extrusion_entity::{
    ExtrusionEntityCollection, ExtrusionEntityType, ExtrusionLoop, ExtrusionPath, ExtrusionRole,
};

// Re-export the free expansion helpers that originate in `LayerRegion.cpp`
// (declared in `Layer.hpp`) but are ported in `region_expansion.rs`.
// LayerRegion.cpp:430-516 / Layer.hpp:366-382
pub use crate::region_expansion::{
    expand_bridges_detect_orientations, expand_merge_surfaces, ExpansionZone,
};
// Re-export the orchestrator that the C++ exposes as
// `LayerRegion::process_external_surfaces` (LayerRegion.cpp:518-640); ported in
// `surface.rs` and driven from `print_object.rs`.
pub use crate::surface::process_external_surfaces;

// LayerRegion.cpp:16
// static const double max_deviation = scale_(0.5);
pub const MAX_DEVIATION: f64 = 0.5 * crate::SCALING_FACTOR;

// LayerRegion.cpp:17
// static const double max_variance  = 5 * scale_(0.01) * scale_(0.01);
pub const MAX_VARIANCE: f64 =
    5.0 * (0.01 * crate::SCALING_FACTOR) * (0.01 * crate::SCALING_FACTOR);

/// Sparse-infill simplify resolution used by the arc-fitting branch of the
/// `simplify_*` family. Kept here mirroring the C++ `SCALED_SPARSE_INFILL_RESOLUTION`
/// constant (PerimeterGenerator.hpp). In this crate, simplify operates on scaled
/// integer tolerances; the value is `scaled<double>(0.05)`.
const SCALED_SPARSE_INFILL_RESOLUTION: f64 = 0.05 * crate::SCALING_FACTOR;

/// Recursively simplify every extrusion entity in a collection.
///
/// LayerRegion.cpp:770-784
/// C++: void LayerRegion::simplify_entity_collection(ExtrusionEntityCollection* entity_collection)
///
/// `spiral_mode`, `enable_arc_fitting` and `scaled_resolution` correspond to the
/// values read from `print()->config()` inside each C++ `simplify_*` member; they
/// are threaded in explicitly here since this crate has no `Print` back-pointer on
/// the entity collection.
pub fn simplify_entity_collection(
    entity_collection: &mut ExtrusionEntityCollection,
    spiral_mode: bool,
    enable_arc_fitting: bool,
    scaled_resolution: f64,
) {
    // LayerRegion.cpp:772
    // for (size_t i = 0; i < entity_collection->entities.size(); i++) {
    for entity in entity_collection.entities.iter_mut() {
        match entity {
            // LayerRegion.cpp:773-774
            // if (ExtrusionEntityCollection* collection = dynamic_cast<...>(...))
            //     this->simplify_entity_collection(collection);
            ExtrusionEntityType::Collection(collection) => {
                simplify_entity_collection(
                    collection,
                    spiral_mode,
                    enable_arc_fitting,
                    scaled_resolution,
                );
            }
            // LayerRegion.cpp:775-776
            // else if (ExtrusionPath* path = dynamic_cast<...>(...))
            //     this->simplify_path(path);
            ExtrusionEntityType::Path(path) => {
                simplify_path(path, spiral_mode, enable_arc_fitting, scaled_resolution);
            }
            // LayerRegion.cpp:777-780
            // else if (ExtrusionMultiPath* multipath = ...) this->simplify_multi_path(multipath);
            // else if (ExtrusionLoop* loop = ...) this->simplify_loop(loop);
            //
            // NOTE: this crate has no `ExtrusionMultiPath` variant (only Path/Loop/Collection),
            // so the multipath branch is folded away; the loop branch is handled below.
            ExtrusionEntityType::Loop(loop_) => {
                simplify_loop(loop_, spiral_mode, enable_arc_fitting, scaled_resolution);
            }
        }
    }
}

/// Simplify a single extrusion path.
///
/// LayerRegion.cpp:786-802
/// C++: void LayerRegion::simplify_path(ExtrusionPath* path)
pub fn simplify_path(
    path: &mut ExtrusionPath,
    spiral_mode: bool,
    enable_arc_fitting: bool,
    scaled_resolution: f64,
) {
    // LayerRegion.cpp:788-791 (print_config / spiral_mode / enable_arc_fitting /
    // scaled_resolution are passed in by the caller instead of read from
    // this->layer()->object()->print()->config()).

    // LayerRegion.cpp:793-794
    // if (enable_arc_fitting && !spiral_mode) {
    if enable_arc_fitting && !spiral_mode {
        // LayerRegion.cpp:795-798
        // if (path->role() == erInternalInfill)
        //     path->simplify_by_fitting_arc(SCALED_SPARSE_INFILL_RESOLUTION);
        // else
        //     path->simplify_by_fitting_arc(scaled_resolution);
        //
        // BLOCKED: ExtrusionPath::simplify_by_fitting_arc (arc fitting) is not yet
        // ported. We fall back to the plain douglas-peucker simplify at the same
        // tolerance so the path is still reduced; this differs from C++ only in
        // that emitted arcs are not fitted (the simplification points are identical).
        let tol = if path.role == ExtrusionRole::InternalInfill {
            SCALED_SPARSE_INFILL_RESOLUTION
        } else {
            scaled_resolution
        };
        path.polyline.simplify(tol as crate::Coord);
    } else {
        // LayerRegion.cpp:800
        // path->simplify(scaled_resolution);
        path.polyline.simplify(scaled_resolution as crate::Coord);
    }
}

/// Simplify a closed extrusion loop (each of its paths).
///
/// LayerRegion.cpp:824-842
/// C++: void LayerRegion::simplify_loop(ExtrusionLoop* loop)
pub fn simplify_loop(
    loop_: &mut ExtrusionLoop,
    spiral_mode: bool,
    enable_arc_fitting: bool,
    scaled_resolution: f64,
) {
    // LayerRegion.cpp:831
    // for (size_t i = 0; i < loop->paths.size(); ++i) {
    for path in loop_.paths.iter_mut() {
        // LayerRegion.cpp:832-833
        // if (enable_arc_fitting && !spiral_mode) {
        if enable_arc_fitting && !spiral_mode {
            // LayerRegion.cpp:834-837
            // if (loop->paths[i].role() == erInternalInfill)
            //     loop->paths[i].simplify_by_fitting_arc(SCALED_SPARSE_INFILL_RESOLUTION);
            // else
            //     loop->paths[i].simplify_by_fitting_arc(scaled_resolution);
            //
            // BLOCKED: simplify_by_fitting_arc not ported; see simplify_path().
            let tol = if path.role == ExtrusionRole::InternalInfill {
                SCALED_SPARSE_INFILL_RESOLUTION
            } else {
                scaled_resolution
            };
            path.polyline.simplify(tol as crate::Coord);
        } else {
            // LayerRegion.cpp:839
            // loop->paths[i].simplify(scaled_resolution);
            path.polyline.simplify(scaled_resolution as crate::Coord);
        }
    }
}

// LayerRegion.cpp:804-822
// void LayerRegion::simplify_multi_path(ExtrusionMultiPath* multipath)
//
// BLOCKED: this crate has no `ExtrusionMultiPath` extrusion-entity variant
// (`ExtrusionEntityType` is Path | Loop | Collection). The body is identical to
// `simplify_loop` applied over `multipath->paths`; once a multipath type exists it
// should be ported here verbatim.
