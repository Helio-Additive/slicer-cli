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
//! In this crate the `LayerRegion` *struct* and most of its member methods
//! already live in [`crate::layer`] (its natural home next to `Layer`), and
//! the free expansion helpers in (2) were ported into
//! [`crate::region_expansion`] (matching `Layer.hpp`'s declaration of
//! `expand_bridges_detect_orientations` and `expand_merge_surfaces`). To avoid
//! duplicating those definitions (which would not compile), this module:
//!   - owns the file-level constants `max_deviation` / `max_variance`
//!     (`LayerRegion.cpp:16-17`),
//!   - re-exports the ported free helpers so callers can reach them through the
//!     `layer_region` path that mirrors the C++ filename, and
//!   - hosts the `LayerRegion::simplify_*` member methods (mirroring the C++
//!     file split: they are defined in `LayerRegion.cpp`, declared in
//!     `Layer.hpp:113-119), reading spiral/arc-fitting/resolution off the
//!     print-config Arc stamped by `wire_layer_hierarchy` — the Rust stand-in
//!     for the C++ `this->layer()->object()->print()->config()` read.

use crate::extrusion_entity::{
    ExtrusionEntityCollection, ExtrusionEntityType, ExtrusionLoop, ExtrusionMultiPath,
    ExtrusionPath, ExtrusionRole,
};
use crate::layer::LayerRegion;

// Re-export the free expansion helpers that originate in `LayerRegion.cpp`
// (declared in `Layer.hpp`) but are ported in `region_expansion.rs`.
// LayerRegion.cpp:430-516 / Layer.hpp:366-382
pub use crate::region_expansion::{
    expand_bridges_detect_orientations, expand_merge_surfaces, ExpansionZone,
};
// `LayerRegion::process_external_surfaces` (LayerRegion.cpp:518-640) is now a
// faithful member method on `LayerRegion` (see `crate::layer`), driving the
// wave-expansion port in `crate::region_expansion`; it no longer needs a free-
// function re-export here.

// LayerRegion.cpp:16
// static const double max_deviation = scale_(0.5);
pub const MAX_DEVIATION: f64 = 0.5 * crate::SCALING_FACTOR;

// LayerRegion.cpp:17
// static const double max_variance  = 5 * scale_(0.01) * scale_(0.01);
pub const MAX_VARIANCE: f64 =
    5.0 * (0.01 * crate::SCALING_FACTOR) * (0.01 * crate::SCALING_FACTOR);

/// Sparse-infill simplify resolution used by the arc-fitting branch of the
/// `simplify_*` family. Kept here mirroring the C++ `SCALED_SPARSE_INFILL_RESOLUTION`
/// constant (libslic3r.h:65-66):
///   `static constexpr double SPARSE_INFILL_RESOLUTION = 0.04;`
///   `#define SCALED_SPARSE_INFILL_RESOLUTION (SPARSE_INFILL_RESOLUTION / SCALING_FACTOR)`
/// In C++ `SCALING_FACTOR = 0.00001` and `scale_/scaled = v / SCALING_FACTOR`, so this
/// equals `0.04 / 0.00001 = 4000` scaled integer units. This crate inverts the constant
/// (`SCALING_FACTOR = 100_000` and `scale = v * SCALING_FACTOR`), so the faithful value
/// is `0.04 * crate::SCALING_FACTOR = 4000`.
const SCALED_SPARSE_INFILL_RESOLUTION: f64 = 0.04 * crate::SCALING_FACTOR;

// `LayerRegion::simplify_*` member methods, defined here to mirror the C++
// file split (bodies in LayerRegion.cpp, declarations in Layer.hpp:113-119).
// The struct itself lives in `crate::layer`.
impl LayerRegion {
    /// Layer.hpp:113
    /// C++: void simplify_infill_extrusion_entity() { simplify_entity_collection(&fills); }
    ///
    /// `fills` is detached for the duration of the call because the recursion
    /// reads the print-config Arc off `&self` (C++ aliases freely through
    /// `this`).
    pub fn simplify_infill_extrusion_entity(&mut self) {
        let mut fills = std::mem::take(&mut self.fills);
        self.simplify_entity_collection(&mut fills);
        self.fills = fills;
    }

    /// Layer.hpp:114
    /// C++: void simplify_wall_extrusion_entity() { simplify_entity_collection(&perimeters); }
    pub fn simplify_wall_extrusion_entity(&mut self) {
        let mut perimeters = std::mem::take(&mut self.perimeters);
        self.simplify_entity_collection(&mut perimeters);
        self.perimeters = perimeters;
    }

    /// Recursively simplify every extrusion entity in a collection.
    ///
    /// LayerRegion.cpp:770-784
    /// C++: void LayerRegion::simplify_entity_collection(ExtrusionEntityCollection* entity_collection)
    pub fn simplify_entity_collection(&self, entity_collection: &mut ExtrusionEntityCollection) {
        // LayerRegion.cpp:772
        // for (size_t i = 0; i < entity_collection->entities.size(); i++) {
        for entity in entity_collection.entities.iter_mut() {
            match entity {
                // LayerRegion.cpp:773-774
                // if (ExtrusionEntityCollection* collection = dynamic_cast<...>(...))
                //     this->simplify_entity_collection(collection);
                ExtrusionEntityType::Collection(collection) => {
                    self.simplify_entity_collection(collection);
                }
                // LayerRegion.cpp:775-776
                // else if (ExtrusionPath* path = dynamic_cast<...>(...))
                //     this->simplify_path(path);
                ExtrusionEntityType::Path(path) => {
                    self.simplify_path(path);
                }
                // LayerRegion.cpp:777-780
                // else if (ExtrusionMultiPath* multipath = ...) this->simplify_multi_path(multipath);
                // else if (ExtrusionLoop* loop = ...) this->simplify_loop(loop);
                //
                // NOTE: this crate has no `ExtrusionMultiPath` variant (only Path/Loop/Collection),
                // so the multipath branch is folded away; the loop branch is handled below.
                ExtrusionEntityType::Loop(loop_) => {
                    self.simplify_loop(loop_);
                }
            }
        }
    }

    /// Simplify a single extrusion path.
    ///
    /// LayerRegion.cpp:786-802
    /// C++: void LayerRegion::simplify_path(ExtrusionPath* path)
    pub fn simplify_path(&self, path: &mut ExtrusionPath) {
        // LayerRegion.cpp:788-791
        // C++: const auto print_config = this->layer()->object()->print()->config();
        //      const bool spiral_mode = print_config.spiral_mode;
        //      const bool enable_arc_fitting = print_config.enable_arc_fitting;
        //      const auto scaled_resolution = scaled<double>(print_config.resolution.value);
        // Read off the stored print-config Arc (field mapping:
        // spiral_mode -> spiral_vase, enable_arc_fitting -> arc_fitting_enabled).
        let (spiral_mode, enable_arc_fitting, scaled_resolution) = self.simplify_print_params();

        // LayerRegion.cpp:793-794
        // if (enable_arc_fitting && !spiral_mode) {
        if enable_arc_fitting && !spiral_mode {
            // LayerRegion.cpp:795-798
            // if (path->role() == erInternalInfill)
            //     path->simplify_by_fitting_arc(SCALED_SPARSE_INFILL_RESOLUTION);
            // else
            //     path->simplify_by_fitting_arc(scaled_resolution);
            if path.role == ExtrusionRole::InternalInfill {
                path.simplify_by_fitting_arc(SCALED_SPARSE_INFILL_RESOLUTION);
            } else {
                path.simplify_by_fitting_arc(scaled_resolution);
            }
        } else {
            // LayerRegion.cpp:800
            // path->simplify(scaled_resolution);
            path.simplify(scaled_resolution);
        }
    }

    /// Simplify a closed extrusion loop (each of its paths).
    ///
    /// LayerRegion.cpp:824-842
    /// C++: void LayerRegion::simplify_loop(ExtrusionLoop* loop)
    pub fn simplify_loop(&self, loop_: &mut ExtrusionLoop) {
        // LayerRegion.cpp:826-829 — same print-config reads as simplify_path.
        let (spiral_mode, enable_arc_fitting, scaled_resolution) = self.simplify_print_params();

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
                if path.role == ExtrusionRole::InternalInfill {
                    path.simplify_by_fitting_arc(SCALED_SPARSE_INFILL_RESOLUTION);
                } else {
                    path.simplify_by_fitting_arc(scaled_resolution);
                }
            } else {
                // LayerRegion.cpp:839
                // loop->paths[i].simplify(scaled_resolution);
                path.simplify(scaled_resolution);
            }
        }
    }

    /// Simplify each path of an extrusion multi-path.
    ///
    /// LayerRegion.cpp:804-822
    /// C++: void LayerRegion::simplify_multi_path(ExtrusionMultiPath* multipath)
    ///
    /// NOTE: `ExtrusionEntityType` (Path | Loop | Collection) has no `MultiPath`
    /// variant, so `simplify_entity_collection`'s dispatch never reaches a
    /// multipath. This method is still ported faithfully against the existing
    /// `ExtrusionMultiPath` struct so it is wired up correctly once such a variant
    /// is added.
    pub fn simplify_multi_path(&self, multipath: &mut ExtrusionMultiPath) {
        // LayerRegion.cpp:806-809 — same print-config reads as simplify_path.
        let (spiral_mode, enable_arc_fitting, scaled_resolution) = self.simplify_print_params();

        // LayerRegion.cpp:811
        // for (size_t i = 0; i < multipath->paths.size(); ++i) {
        for path in multipath.paths.iter_mut() {
            // LayerRegion.cpp:812-813
            // if (enable_arc_fitting && !spiral_mode) {
            if enable_arc_fitting && !spiral_mode {
                // LayerRegion.cpp:814-817
                // if (multipath->paths[i].role() == erInternalInfill)
                //     multipath->paths[i].simplify_by_fitting_arc(SCALED_SPARSE_INFILL_RESOLUTION);
                // else
                //     multipath->paths[i].simplify_by_fitting_arc(scaled_resolution);
                if path.role == ExtrusionRole::InternalInfill {
                    path.simplify_by_fitting_arc(SCALED_SPARSE_INFILL_RESOLUTION);
                } else {
                    path.simplify_by_fitting_arc(scaled_resolution);
                }
            } else {
                // LayerRegion.cpp:819
                // multipath->paths[i].simplify(scaled_resolution);
                path.simplify(scaled_resolution);
            }
        }
    }

    /// The three print-config reads shared by every C++ `simplify_*` member
    /// (LayerRegion.cpp:788-791 / 806-809 / 826-829):
    /// `(spiral_mode, enable_arc_fitting, scaled<double>(resolution))`.
    fn simplify_print_params(&self) -> (bool, bool, f64) {
        let print_config = self
            .print_config
            .as_deref()
            .expect("config hierarchy not wired — call wire_layer_hierarchy");
        (
            print_config.spiral_vase,
            print_config.arc_fitting_enabled,
            print_config.resolution * crate::SCALING_FACTOR,
        )
    }
}
