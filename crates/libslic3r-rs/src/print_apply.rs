//! Print::apply() - Master orchestration for model/config change synchronization
//!
//! C++ Reference:
//! - PrintApply.cpp (1,903 lines)
//!
//! ⚠️ **WARNING: CRITICAL COMPLEXITY - DO NOT START YET** ⚠️
//!
//! **STATUS:** ❌ NOT PORTED (0% - Empty Stub)
//! **PRIORITY:** 🔴 P0 CRITICAL (but many prerequisites required first)
//! **ESTIMATED EFFORT:** 13-18 weeks total (including all prerequisites)
//!
//! ## What This Does
//!
//! Implements `Print::apply()` - the entry point called on EVERY model or config change.
//! This is the master orchestrator that determines what needs to be re-sliced vs. reused.
//!
//! **Key Responsibilities:**
//! - Differential updates (detect what changed)
//! - PrintObject lifecycle (create/delete/reuse)
//! - PrintObjectRegions synchronization (multi-material system)
//! - Config validation and normalization
//! - Cache management
//! - Step invalidation (mark what needs re-processing)
//!
//! ## Why This Is Complex (1,903 Lines!)
//!
//! 1. **10 Major Processing Phases:**
//!    - Config normalization (filament mapping, extruder types)
//!    - Config diff computation (what changed?)
//!    - Support/raft handling
//!    - ModelObject status tracking (new/old/moved/deleted)
//!    - PrintObject reuse analysis
//!    - PrintObject creation/deletion
//!    - Region synchronization (MOST COMPLEX - 400+ lines)
//!    - Region invalidation
//!    - Status return
//!
//! 2. **PrintObjectRegions System:**
//!    - Multi-material painted regions
//!    - Modifier volumes
//!    - Per-layer-range configs
//!    - Geometric intersection tests
//!    - Incremental validation logic
//!
//! 3. **Deep Dependencies:**
//!    - Model/ModelObject/ModelVolume (needs full port - currently 15-20%)
//!    - Config normalization system (not ported)
//!    - PrintObjectRegions (complex, not designed yet)
//!    - Transform comparison utilities
//!
//! ## BLOCKERS - Must Complete First
//!
//! ❌ **DO NOT START** until ALL of these are complete:
//!
//! 1. ✅ complete (Sessions 91-93)
//!    - Wire Rayon parallelism
//!    - Switch CLI to Print::process()
//!    - Delete old pipeline/ module
//!
//! 2. 🔲 Port Model/ModelObject fully (3-4 weeks)
//!    - Volume management
//!    - Transformation tracking
//!    - Painting support (seam, support, MMU segmentation)
//!
//! 3. 🔲 Port Config normalization (1-2 weeks)
//!    - normalize_fdm_1(), normalize_fdm_2()
//!    - Config diff computation
//!    - Per-extruder handling
//!
//! 4. 🔲 Design PrintObjectRegions system (1 week)
//!    - Define Rust API
//!    - Plan incremental implementation
//!
//! 5. 🔲 Implement PrintObjectRegions (4-5 weeks)
//!    - Basic region generation
//!    - Multi-material support
//!    - Painted region handling
//!
//! ## Port Strategy (After Blockers Complete)
//!
//! **** Core infrastructure (Week 1-2, ~200 lines)
//! - ApplyStatus enum, status tracking structs
//! - Basic Print::apply() skeleton with logging
//!
//! **** Simple differential logic (Week 3-4, ~300 lines)
//! - ModelObject tracking (new/deleted)
//! - PrintObject creation/deletion
//! - Basic geometry change detection
//!
//! **** Config diff system (Week 5, ~250 lines)
//! - Config diff computation
//! - Step invalidation based on diffs
//!
//! **** Basic regions (Week 6-7, ~400 lines)
//! - Single-material region generation
//! - Layer height ranges
//!
//! **** Advanced regions (Week 8-10, ~500 lines)
//! - Multi-material support
//! - Painted regions
//! - Modifier volumes
//!
//! **** Testing & polish (Week 11-12, ~253 lines)
//! - Comprehensive tests
//! - Edge case handling
//! - Optimization
//!
//! ## Timeline
//!
//! | Task | Effort | Status |
//! |------|--------|--------|
//! | Prerequisites | 9-12 weeks | 🔲 Not started |
//! | PrintApply port | 6-8 weeks | 🔲 Not started |
//! | **TOTAL** | **15-20 weeks** | **~4-5 months** |
//!
//! **Realistic Start Date:** Q2 2025 (-5 complete)
//!
//! ## Documentation
//!
//! See `SESSION_PRINTAPPLY_INSPECTION.md` (536 lines) for:
//! - Complete algorithm breakdown (10 phases explained)
//! - Line-by-line C++ structure analysis
//! - Detailed port strategy
//! - Testing plan
//! - Dependency tree
//! - Cross-references
//!
//! ## PORT STATUS: PARTIAL
//!
//! The bulk of `PrintApply.cpp` depends on Rust-side infrastructure that does
//! not yet exist (see "BLOCKED SYMBOLS" below). The pure, self-contained leaf
//! helpers that depend only on already-ported primitives have been translated
//! faithfully (1:1) below. The remainder is documented as blocked.
//!
//! ### Ported (faithful, builds):
//! - `transform3d_lower`   — PrintApply.cpp:107
//! - `transform3d_equal`   — PrintApply.cpp:121
//! - `custom_per_printz_gcodes_tool_changes_differ` — PrintApply.cpp:189
//!     (depends only on the already-ported `custom_g_code::Item`/`Type`)
//! - `trafos_differ_in_rotation_by_z_and_mirroring_by_xy_only` — PrintApply.cpp:558
//!     (pure Transform3d math; depends only on already-ported primitives)
//!
//! ### BLOCKED SYMBOLS (genuinely unportable until prerequisites land)
//!
//! Every other symbol in `PrintApply.cpp` references types/methods that the
//! current Rust crate does not expose. Listing the dependency that blocks each:
//!
//! - `model_volume_list_update_supports`, `model_volume_list_copy_configs`
//!     -> needs `ModelObject::volumes` / `ModelVolume` (id, type,
//!        is_support_modifier, supported_facets, fuzzy_skin_facets, seam_facets,
//!        mmu_segmentation_facets, config, get_transformation). Rust `ModelObject`
//!        only carries a single `mesh` and has no `ModelVolume` concept.
//! - `layer_height_ranges_copy_configs`, `layer_height_ranges_equal`,
//!   `LayerRanges`
//!     -> needs `t_layer_config_ranges` / `ModelConfig` / `t_layer_height_range`
//!        (`ModelObject::layer_config_ranges`). Not present.
//! - `print_objects_from_model_object`, `PrintObjectTrafoAndInstances`
//!     -> needs `ModelInstance::is_printable`/`get_matrix` and `PrintInstance`.
//!        Rust `Instance` lacks matrices, printable/print_volume_state, and IDs.
//! - `print_config_diffs`, `full_print_config_diffs`
//!     -> needs `PrintConfig::keys()`, `DynamicPrintConfig::option`,
//!        `print_config_def.extruder_retract_keys()`,
//!        `compute_filament_override_value`, per-plate wipe_tower handling.
//! - `is_printable_filament_changed`
//!     -> needs `DynamicPrintConfig` option access + clipper `diff`/`intersection`
//!        plumbed against `printable_area`/`extruder_printable_area`/`filament_map_mode`.
//! - `ModelObjectStatus(DB)`, `PrintObjectStatus(DB)`
//!     -> needs `ObjectID`-keyed `PrintObject`/`ModelObject` plus
//!        `PrintObjectRegions` ref-counting and `print_object->trafo()`.
//! - `trafo_for_bbox`, `transformed_its_bbox2d`, `transformed_its_bboxes_in_z_ranges`
//!     -> needs `PrintObjectRegions::BoundingBox` (an `Eigen::AlignedBox3f`
//!        equivalent over `Vec3f`) and `indexed_triangle_set`. The Rust
//!        `PrintObjectRegions` only stores `all_regions`; no bbox type exists.
//!     (`trafos_differ_in_rotation_by_z_and_mirroring_by_xy_only` is pure
//!      Transform3d math and HAS been ported above.)
//! - `model_volume_solid_or_modifier`
//!     -> needs `ModelVolumeType`. Not present.
//! - `print_objects_regions_invalidate_keep_some_volumes`, `find_volume_extents`,
//!   `find_modifier_volume_extents`, `update_volume_bboxes`,
//!   `verify_update_print_object_regions`, `generate_print_object_regions`
//!     -> needs the full `PrintObjectRegions` nested system
//!        (`LayerRangeRegions`, `VolumeRegion`, `PaintedRegion`,
//!        `FuzzySkinPaintedRegion`, `VolumeExtents`, `cached_volume_ids`,
//!        `trafo_bboxes`) plus `region_config_from_model_volume`. The current
//!        Rust struct is a placeholder with only `all_regions` + a refcount.
//! - `Print::apply` (the orchestrator)
//!     -> needs essentially all of the above plus `Print` fields
//!        (`m_config`, `m_full_print_config`, `m_default_object_config`,
//!        `m_default_region_config`, `m_objects`, `m_model`,
//!        `m_placeholder_parser`), `normalize_fdm_1/2`,
//!        `invalidate_state_by_config_options`, `invalidate_step(s)`,
//!        `update_filament_self_index_cache`, mixed-filament expansion, the
//!        extruder/variant config machinery, etc.
//!
//! These are tracked for follow-up; none are stubbed here (a fake would corrupt
//! the byte-exact G-code parity goal). When the Model/ModelVolume,
//! `t_layer_config_ranges`, and full `PrintObjectRegions` ports land, the
//! blocked symbols can be translated against them.

use crate::custom_g_code::{Item as CustomGCodeItem, Type as CustomGCodeType};
use crate::geometry::geometry::{Transform3d, Vec3d};
use crate::libslic3r::EPSILON;
use nalgebra::Matrix3;

// PrintApply.cpp:107
// static inline bool transform3d_lower(const Transform3d &lhs, const Transform3d &rhs)
//
// Eigen's `Transform3d` is a homogeneous 4x4 affine matrix stored column-major;
// `Transform3d::data()` walks the 16 scalars in that column-major order. The
// Rust alias `Transform3d = nalgebra::Matrix4<f64>` is also column-major and
// `Matrix4::as_slice()` yields the same 16-element column-major view, so the
// element-by-element lexicographic comparison is byte-for-byte equivalent.
#[inline]
pub fn transform3d_lower(lhs: &Transform3d, rhs: &Transform3d) -> bool {
    // typedef Transform3d::Scalar T;
    // const T *lv = lhs.data();
    // const T *rv = rhs.data();
    let lv = lhs.as_slice();
    let rv = rhs.as_slice();
    // for (size_t i = 0; i < 16; ++ i, ++ lv, ++ rv) {
    for i in 0..16 {
        // if (*lv < *rv)
        if lv[i] < rv[i] {
            // return true;
            return true;
        // else if (*lv > *rv)
        } else if lv[i] > rv[i] {
            // return false;
            return false;
        }
    }
    // return false;
    false
}

// PrintApply.cpp:121
// static inline bool transform3d_equal(const Transform3d &lhs, const Transform3d &rhs)
#[inline]
pub fn transform3d_equal(lhs: &Transform3d, rhs: &Transform3d) -> bool {
    // typedef Transform3d::Scalar T;
    // const T *lv = lhs.data();
    // const T *rv = rhs.data();
    let lv = lhs.as_slice();
    let rv = rhs.as_slice();
    // for (size_t i = 0; i < 16; ++ i, ++ lv, ++ rv)
    for i in 0..16 {
        // if (*lv != *rv)
        if lv[i] != rv[i] {
            // return false;
            return false;
        }
    }
    // return true;
    true
}

// PrintApply.cpp:189
// Returns true if va == vb when all CustomGCode items that are not ToolChangeCode are ignored.
// static bool custom_per_printz_gcodes_tool_changes_differ(const std::vector<CustomGCode::Item> &va, const std::vector<CustomGCode::Item> &vb)
//
// The C++ walks two iterators, skipping non-ToolChange items in either list,
// and compares the surviving ToolChange items pairwise. Modelled here with two
// running slice indices.
pub fn custom_per_printz_gcodes_tool_changes_differ(
    va: &[CustomGCodeItem],
    vb: &[CustomGCodeItem],
) -> bool {
    // auto it_a = va.begin();
    // auto it_b = vb.begin();
    let mut ia = 0usize;
    let mut ib = 0usize;
    // while (it_a != va.end() || it_b != vb.end()) {
    while ia != va.len() || ib != vb.len() {
        // if (it_a != va.end() && it_a->type != CustomGCode::ToolChange) {
        if ia != va.len() && va[ia].gcode_type != CustomGCodeType::ToolChange {
            // Skip any CustomGCode items, which are not tool changes.
            // ++ it_a;
            ia += 1;
            // continue;
            continue;
        }
        // if (it_b != vb.end() && it_b->type != CustomGCode::ToolChange) {
        if ib != vb.len() && vb[ib].gcode_type != CustomGCodeType::ToolChange {
            // Skip any CustomGCode items, which are not tool changes.
            // ++ it_b;
            ib += 1;
            // continue;
            continue;
        }
        // if (it_a == va.end() || it_b == vb.end())
        if ia == va.len() || ib == vb.len() {
            // va or vb contains more Tool Changes than the other.
            // return true;
            return true;
        }
        // assert(it_a->type == CustomGCode::ToolChange);
        debug_assert!(va[ia].gcode_type == CustomGCodeType::ToolChange);
        // assert(it_b->type == CustomGCode::ToolChange);
        debug_assert!(vb[ib].gcode_type == CustomGCodeType::ToolChange);
        // if (*it_a != *it_b)
        if va[ia] != vb[ib] {
            // The two Tool Changes differ.
            // return true;
            return true;
        }
        // ++ it_a;
        ia += 1;
        // ++ it_b;
        ib += 1;
    }
    // There is no change in custom Tool Changes.
    // return false;
    false
}

// PrintApply.cpp:558
// static inline bool trafos_differ_in_rotation_by_z_and_mirroring_by_xy_only(const Transform3d &t1, const Transform3d &t2)
//
// Eigen `Transform3d::translation()` is column 3 of the 4x4 (rows 0..2); in
// nalgebra column-major `Matrix4<f64>` that is `m[(row, 3)]`. `t.matrix().block<3,3>(0,0)`
// is the linear part -> `m.fixed_view::<3,3>(0,0)`. `m.block<3,1>(0,c)` is column `c`
// of the 3x3 -> `m.column(c)`. `squaredNorm()` -> `norm_squared()`.
pub fn trafos_differ_in_rotation_by_z_and_mirroring_by_xy_only(t1: &Transform3d, t2: &Transform3d) -> bool {
    // if (std::abs(t1.translation().z() - t2.translation().z()) > EPSILON)
    if (t1[(2, 3)] - t2[(2, 3)]).abs() > EPSILON {
        // One of the object is higher than the other above the build plate (or below the build plate).
        // return false;
        return false;
    }
    // Matrix3d m1 = t1.matrix().block<3, 3>(0, 0);
    let m1: Matrix3<f64> = t1.fixed_view::<3, 3>(0, 0).into();
    // Matrix3d m2 = t2.matrix().block<3, 3>(0, 0);
    let m2: Matrix3<f64> = t2.fixed_view::<3, 3>(0, 0).into();
    // Matrix3d m = m2.inverse() * m1;
    let m: Matrix3<f64> = m2.try_inverse().unwrap_or_else(Matrix3::identity) * m1;
    // Vec3d    z = m.block<3, 1>(0, 2);
    let z: Vec3d = m.column(2).into();
    // if (std::abs(z.x()) > EPSILON || std::abs(z.y()) > EPSILON || std::abs(z.z() - 1.) > EPSILON)
    if z.x.abs() > EPSILON || z.y.abs() > EPSILON || (z.z - 1.0).abs() > EPSILON {
        // Z direction or length changed.
        // return false;
        return false;
    }
    // Z still points in the same direction and it has the same length.
    // Vec3d    x = m.block<3, 1>(0, 0);
    let x: Vec3d = m.column(0).into();
    // Vec3d    y = m.block<3, 1>(0, 1);
    let y: Vec3d = m.column(1).into();
    // if (std::abs(x.z()) > EPSILON || std::abs(y.z()) > EPSILON)
    if x.z.abs() > EPSILON || y.z.abs() > EPSILON {
        // return false;
        return false;
    }
    // double   lx2 = x.squaredNorm();
    let lx2 = x.norm_squared();
    // double   ly2 = y.squaredNorm();
    let ly2 = y.norm_squared();
    // if (lx2 - 1. > EPSILON * EPSILON || ly2 - 1. > EPSILON * EPSILON)
    if lx2 - 1.0 > EPSILON * EPSILON || ly2 - 1.0 > EPSILON * EPSILON {
        // return false;
        return false;
    }
    // Verify whether the vectors x, y are still perpendicular.
    // double   d   = x.dot(y);
    let d = x.dot(&y);
    // return std::abs(d * d) < EPSILON * lx2 * ly2;
    (d * d).abs() < EPSILON * lx2 * ly2
}


// ============================================================================
// PrintApply.cpp over the Print.hpp classes of `crate::print::faithful`.
// ============================================================================

use crate::config::{ConfigBase, ConfigOption, ConfigOptionResolver};
use crate::model::{Model, ModelObject, ModelVolume, ModelVolumeType, TLayerConfigRanges};
use crate::object_id::ObjectID;
use crate::print::faithful::{
    AlignedBox3f, FuzzySkinPaintedRegion, FuzzySkinParentType, LayerRangeRegions, PaintedRegion, PrintInstance, PrintInstances,
    PrintObjectRegions, PrintRegion, PrintRegionConfig, PrintRegionPtr, VolumeExtents, VolumeRegion,
};
use crate::print_config::config_def::DynamicPrintConfig;
use crate::geometry::{Point, Polygon};
use std::cell::RefCell;
use std::rc::Rc;

/// `Transform3f`: Eigen `Transform<float, 3, Affine>`, its 4x4 matrix.
pub type Transform3f = nalgebra::Matrix4<f32>;

/// The indices of a ModelObject's volumes sorted by their ObjectID (`model_volumes_sort_by_id` of a
/// `ModelVolumePtrs` copy, Model.hpp:1434-1437).
fn volume_indices_sorted_by_id(model_object: &ModelObject) -> Vec<usize> {
    let mut idx: Vec<usize> = (0..model_object.volumes.len()).collect();
    idx.sort_by(|&l, &r| model_object.volumes[l].id().cmp(&model_object.volumes[r].id()));
    idx
}

/// PrintApply.cpp:14-58 `void model_volume_list_update_supports(ModelObject &model_object_dst, const
/// ModelObject &model_object_new)`: add or remove support modifier ModelVolumes from
/// model_object_dst to match the ModelVolumes of model_object_new in the exact order and with the
/// same IDs. It is expected, that the model_object_dst already contains the non-support volumes of
/// model_object_new in the correct order.
pub fn model_volume_list_update_supports(model_object_dst: &mut ModelObject, model_object_new: &ModelObject) {
    // typedef std::pair<const ModelVolume*, bool> ModelVolumeWithStatus;
    let mut old_volumes: Vec<(Option<ModelVolume>, ObjectID, bool)> =
        std::mem::take(&mut model_object_dst.volumes).into_iter().map(|mv| {
            let id = mv.id();
            (Some(mv), id, false)
        }).collect();
    // std::sort(old_volumes, model_volume_lower): the ids are unique
    old_volumes.sort_by(|a, b| a.1.cmp(&b.1));
    model_object_dst.volumes.reserve(model_object_new.volumes.len());
    for model_volume_src in &model_object_new.volumes {
        let key = model_volume_src.id();
        let it = old_volumes.partition_point(|mv| mv.1 < key);
        if it != old_volumes.len() && old_volumes[it].1 == key {
            // The volume was found in the old list. Just copy it.
            debug_assert!(!old_volumes[it].2); // not consumed yet
            old_volumes[it].2 = true;
            let mut model_volume_dst = old_volumes[it].0.take().expect("not consumed yet");
            // For support modifiers, the type may have been switched from blocker to enforcer and vice versa.
            debug_assert!(
                (model_volume_dst.is_support_modifier() && model_volume_src.is_support_modifier())
                    || model_volume_dst.type_() == model_volume_src.type_()
            );
            if model_volume_dst.is_support_modifier() {
                model_volume_dst.set_type(model_volume_src.type_());
                model_volume_dst.set_transformation(model_volume_src.get_transformation());
            }
            model_object_dst.volumes.push(model_volume_dst);
        } else {
            // The volume was not found in the old list. Create a new copy.
            debug_assert!(model_volume_src.is_support_modifier());
            model_object_dst.volumes.push(model_volume_src.clone());
        }
    }
    // Release the non-consumed old volumes (those were deleted from the new list): dropped here.
}

/// PrintApply.cpp:60-92 `static inline void model_volume_list_copy_configs(ModelObject &model_object_dst,
/// const ModelObject &model_object_src, const ModelVolumeType type)`
pub fn model_volume_list_copy_configs(model_object_dst: &mut ModelObject, model_object_src: &ModelObject, type_: ModelVolumeType) {
    let mut i_src = 0usize;
    let mut i_dst = 0usize;
    while i_src < model_object_src.volumes.len() && i_dst < model_object_dst.volumes.len() {
        let mv_src = &model_object_src.volumes[i_src];
        if mv_src.type_() != type_ {
            i_src += 1;
            continue;
        }
        if model_object_dst.volumes[i_dst].type_() != type_ {
            i_dst += 1;
            continue;
        }
        let mv_dst = &mut model_object_dst.volumes[i_dst];
        debug_assert!(mv_src.id() == mv_dst.id());
        // Copy the ModelVolume data.
        mv_dst.name = mv_src.name.clone();
        mv_dst.config.assign_config(&mv_src.config);
        mv_dst.supported_facets.assign(&mv_src.supported_facets);
        mv_dst.fuzzy_skin_facets.assign(&mv_src.fuzzy_skin_facets);
        mv_dst.seam_facets.assign(&mv_src.seam_facets);
        mv_dst.mmu_segmentation_facets.assign(&mv_src.mmu_segmentation_facets);
        //FIXME what to do with the materials?
        i_src += 1;
        i_dst += 1;
    }
}

/// PrintApply.cpp:94-106 `static inline void layer_height_ranges_copy_configs(t_layer_config_ranges &lr_dst,
/// const t_layer_config_ranges &lr_src)`
pub fn layer_height_ranges_copy_configs(lr_dst: &mut TLayerConfigRanges, lr_src: &TLayerConfigRanges) {
    debug_assert!(lr_dst.len() == lr_src.len());
    let mut it_src = lr_src.iter();
    for (_, kvp_dst) in lr_dst.iter_mut() {
        let (_, kvp_src) = it_src.next().expect("lr_dst.size() == lr_src.size()");
        // Layer heights are allowed do differ in case the layer height table is being overriden by the smooth profile.
        *kvp_dst = kvp_src.clone();
    }
}

/// PrintApply.cpp:132-137 `struct PrintObjectTrafoAndInstances`
#[derive(Debug, Clone)]
pub struct PrintObjectTrafoAndInstances {
    pub trafo: Transform3d,
    pub instances: PrintInstances,
}

/// PrintApply.cpp:140-169 `static std::vector<PrintObjectTrafoAndInstances>
/// print_objects_from_model_object(const ModelObject &model_object)`: a list of trafos and XY
/// offsets for instances of a ModelObject.
pub fn print_objects_from_model_object(model_object: &ModelObject) -> Vec<PrintObjectTrafoAndInstances> {
    // std::set<PrintObjectTrafoAndInstances> ordered by transform3d_lower(trafo)
    let mut trafos: Vec<PrintObjectTrafoAndInstances> = Vec::new();
    for (model_instance_idx, model_instance) in model_object.instances.iter().enumerate() {
        if model_instance.is_printable(model_object) {
            let mut trafo = model_instance.get_matrix(false, false, false, false);
            // trafo.data()[12], [13]: the X and Y translation (column-major)
            // Point::new_scale: `coord_t(scale_(x))`, truncated.
            let shift = Point::new(
                (trafo[(0, 3)] / crate::libslic3r::SCALING_FACTOR) as i64,
                (trafo[(1, 3)] / crate::libslic3r::SCALING_FACTOR) as i64,
            );
            // Reset the XY axes of the transformation.
            trafo[(0, 3)] = 0.0;
            trafo[(1, 3)] = 0.0;
            // Search or insert a trafo.
            let pos = trafos.partition_point(|t| transform3d_lower(&t.trafo, &trafo));
            let it = if pos < trafos.len() && !transform3d_lower(&trafo, &trafos[pos].trafo) {
                pos
            } else {
                trafos.insert(pos, PrintObjectTrafoAndInstances { trafo, instances: Vec::new() });
                pos
            };
            trafos[it].instances.push(PrintInstance { print_object: usize::MAX, model_instance: model_instance_idx, shift, id: 0 });
        }
    }
    trafos
}

/// PrintApply.cpp:171-188 `static bool layer_height_ranges_equal(const t_layer_config_ranges &lr1, const
/// t_layer_config_ranges &lr2, bool check_layer_height)`: compare just the layer ranges and their
/// layer heights, not the associated configs.
pub fn layer_height_ranges_equal(lr1: &TLayerConfigRanges, lr2: &TLayerConfigRanges, check_layer_height: bool) -> bool {
    if lr1.len() != lr2.len() {
        return false;
    }
    let mut it2 = lr2.iter();
    for (k1, v1) in lr1.iter() {
        let (k2, v2) = it2.next().unwrap();
        if !v2.has("layer_height") || !v1.has("layer_height") {
            return false;
        }
        let lh = |v: &crate::model::ModelConfig| v.option("layer_height").and_then(|o| o.get_float().ok()).unwrap_or(0.0);
        if (k1.0 .0 - k2.0 .0).abs() > EPSILON
            || (k1.1 .0 - k2.1 .0).abs() > EPSILON
            || (check_layer_height && (lh(v1) - lh(v2)).abs() > EPSILON)
        {
            return false;
        }
    }
    true
}

/// `*opt_new != *opt_old` of two ConfigOptions.
fn options_differ(opt_new: &dyn ConfigOption, opt_old: &dyn ConfigOption) -> bool {
    opt_new.operator_ne(opt_old).unwrap_or(true)
}

/// The values of a `ConfigOptionFloats`.
fn floats_values(opt: &dyn ConfigOption) -> Vec<f64> {
    opt.as_any().downcast_ref::<crate::config::ConfigOptionFloats>().map(|o| o.values.clone()).unwrap_or_default()
}

/// PrintApply.cpp:220-270 `static t_config_option_keys print_config_diffs(const PrintConfig
/// &current_config, const DynamicPrintConfig &new_full_config, DynamicPrintConfig
/// &filament_overrides, int plate_index, std::vector<int>& filament_maps)`: changes to the print
/// config, accounting for overrides of extruder retract values by filament presets.
pub fn print_config_diffs(
    current_config: &DynamicPrintConfig,
    new_full_config: &DynamicPrintConfig,
    filament_overrides: &mut DynamicPrintConfig,
    plate_index: i32,
    filament_maps: &[i32],
) -> Vec<String> {
    let extruder_retract_keys = crate::print_config::config_def::print_config_def().extruder_retract_keys().clone();
    let filament_prefix = "filament_";
    let mut print_diff: Vec<String> = Vec::new();
    for opt_key in current_config.keys() {
        let opt_old = current_config.option(&opt_key).expect("opt_old != nullptr");
        let Some(opt_new) = new_full_config.option(&opt_key) else {
            //FIXME This may happen when executing some test cases.
            continue;
        };
        let opt_new_filament = if extruder_retract_keys.binary_search(&opt_key).is_ok() {
            new_full_config.option(&format!("{}{}", filament_prefix, opt_key))
        } else {
            None
        };

        if let Some(opt_new_filament) = opt_new_filament {
            let mut filament_map_indices: Vec<i32> = filament_maps.iter().map(|&m| m - 1).collect();
            let _ = crate::print_config::config_def::compute_filament_override_value(
                &opt_key,
                opt_old,
                opt_new,
                opt_new_filament,
                new_full_config,
                &mut print_diff,
                filament_overrides,
                &mut filament_map_indices,
            );
        } else if options_differ(opt_new, opt_old) {
            //BBS: add plate_index logic for wipe_tower_x/wipe_tower_y
            if opt_key == "wipe_tower_x" || opt_key == "wipe_tower_y" {
                let option_new = floats_values(opt_new);
                let option_old = floats_values(opt_old);
                let pi = plate_index as usize;
                if plate_index >= 0 && pi < option_new.len() && pi < option_old.len() {
                    let value_new = option_new[pi] as f32;
                    let value_old = option_old[pi] as f32;
                    if value_old != value_new {
                        print_diff.push(opt_key);
                    }
                } else if plate_index >= 0 && (pi < option_new.len() || pi < option_old.len()) {
                    print_diff.push(opt_key);
                }
            } else {
                print_diff.push(opt_key);
            }
        }
    }
    print_diff
}

/// PrintApply.cpp:272-298 `static t_config_option_keys full_print_config_diffs(const
/// DynamicPrintConfig &current_full_config, const DynamicPrintConfig &new_full_config, int
/// plate_index)`
pub fn full_print_config_diffs(current_full_config: &DynamicPrintConfig, new_full_config: &DynamicPrintConfig, plate_index: i32) -> Vec<String> {
    let mut full_config_diff: Vec<String> = Vec::new();
    for opt_key in new_full_config.keys() {
        let opt_old = current_full_config.option(&opt_key);
        let opt_new = new_full_config.option(&opt_key).expect("opt_new");
        if opt_old.is_none() || options_differ(opt_new, opt_old.unwrap()) {
            //BBS: add plate_index logic for wipe_tower_x/wipe_tower_y
            if opt_old.is_some() && (opt_key == "wipe_tower_x" || opt_key == "wipe_tower_y") {
                let option_new = floats_values(opt_new);
                let option_old = floats_values(opt_old.unwrap());
                let pi = plate_index as usize;
                if plate_index >= 0 && pi < option_new.len() && pi < option_old.len() {
                    let value_new = option_new[pi] as f32;
                    let value_old = option_old[pi] as f32;
                    if value_old != value_new {
                        full_config_diff.push(opt_key);
                    }
                } else if plate_index >= 0 && (pi < option_new.len() || pi < option_old.len()) {
                    full_config_diff.push(opt_key);
                }
            } else {
                full_config_diff.push(opt_key);
            }
        }
    }
    full_config_diff
}

/// PrintApply.cpp:300-347 `static bool is_printable_filament_changed(const DynamicPrintConfig&
/// new_full_config, const Polygon& old_poly, const Polygon& new_poly)`
pub fn is_printable_filament_changed(new_full_config: &DynamicPrintConfig, old_poly: &Polygon, new_poly: &Polygon) -> bool {
    use crate::clipper_utils::{diff_polygons, intersection_polygons_raw_clib as intersection_polygons};
    if old_poly.points != new_poly.points {
        if let Some(map_mode_opt) = new_full_config.option("filament_map_mode") {
            if map_mode_opt.get_int().ok() == Some(crate::print_config::enums::FilamentMapMode::FmmManual as i32) {
                return false;
            }
        }

        let printable_area = new_full_config
            .option("printable_area")
            .and_then(|o| o.as_any().downcast_ref::<crate::config::ConfigOptionPoints>())
            .map(|o| o.values.clone())
            .unwrap_or_default();
        let extruder_areas = new_full_config
            .option("extruder_printable_area")
            .and_then(|o| o.as_any().downcast_ref::<crate::config::ConfigOptionPointsGroups>())
            .map(|o| o.values.clone())
            .unwrap_or_default();
        // Point(scale_(x), scale_(y)): the scaled double through Point(double, double), rounded.
        let pt = |p: &crate::geometry::Vec2d| {
            Point::new((p.x / crate::libslic3r::SCALING_FACTOR).round_ties_even() as i64, (p.y / crate::libslic3r::SCALING_FACTOR).round_ties_even() as i64)
        };
        let printable_poly = Polygon::from_points(printable_area.iter().map(pt).collect());

        let mut extruder_polys: Vec<Polygon> = Vec::new();
        for extruder_area in &extruder_areas {
            extruder_polys.push(Polygon::from_points(extruder_area.iter().map(pt).collect()));
        }

        let mut split_polys: Vec<Polygon> = Vec::new();
        for poly in &extruder_polys {
            let res = diff_polygons(std::slice::from_ref(&printable_poly), std::slice::from_ref(poly));
            if !res.is_empty() {
                split_polys.push(res[0].clone());
            }
        }

        let all_extruder_polys = intersection_polygons(std::slice::from_ref(&printable_poly), &extruder_polys);
        if !all_extruder_polys.is_empty() {
            split_polys.push(all_extruder_polys[0].clone());
        }

        let find_intersections = |poly: &Polygon, contours: &[Polygon]| -> std::collections::BTreeSet<i32> {
            let mut result = std::collections::BTreeSet::new();
            for (i, c) in contours.iter().enumerate() {
                if !intersection_polygons(std::slice::from_ref(poly), std::slice::from_ref(c)).is_empty() {
                    result.insert(i as i32);
                }
            }
            result
        };

        let old_poly_ids = find_intersections(old_poly, &split_polys);
        let new_poly_ids = find_intersections(new_poly, &split_polys);

        return old_poly_ids != new_poly_ids;
    }
    false
}

/// PrintApply.cpp:351-357 `struct LayerRange`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LayerRange {
    pub layer_height_range: (f64, f64),
    /// `const DynamicPrintConfig* config`: the key of the range in the ModelObject's
    /// `layer_config_ranges` that owns the config.
    pub config: Option<(f64, f64)>,
}

/// PrintApply.cpp:349-407 `class LayerRanges`: repository for solving partial overlaps of
/// ModelObject::layer_config_ranges.
#[derive(Debug, Clone, Default)]
pub struct LayerRanges {
    m_ranges: Vec<LayerRange>,
}

impl LayerRanges {
    /// PrintApply.cpp:360 `LayerRanges(const t_layer_config_ranges &in) { this->assign(in); }`
    pub fn new(input: &TLayerConfigRanges) -> Self {
        let mut r = LayerRanges::default();
        r.assign(input);
        r
    }

    /// PrintApply.cpp:362-386 `void assign(const t_layer_config_ranges &in)`: convert input config
    /// ranges into continuous non-overlapping sorted vector of intervals and their configs.
    pub fn assign(&mut self, input: &TLayerConfigRanges) {
        self.m_ranges.clear();
        self.m_ranges.reserve(input.len());
        // Input ranges are sorted lexicographically. First range trims the other ranges.
        let mut last_z: f64 = 0.0;
        for (range_first, _) in input {
            let (lo, hi) = (range_first.0 .0, range_first.1 .0);
            if hi > last_z {
                let min_z = lo.max(0.0);
                if min_z > last_z + EPSILON {
                    self.m_ranges.push(LayerRange { layer_height_range: (last_z, min_z), config: None });
                    last_z = min_z;
                }
                if hi > last_z + EPSILON {
                    self.m_ranges.push(LayerRange { layer_height_range: (last_z, hi), config: Some((lo, hi)) });
                    last_z = hi;
                }
            }
        }
        if self.m_ranges.is_empty() {
            self.m_ranges.push(LayerRange { layer_height_range: (0.0, f64::MAX), config: None });
        } else if self.m_ranges.last().unwrap().config.is_none() {
            self.m_ranges.last_mut().unwrap().layer_height_range.1 = f64::MAX;
        } else {
            let s = self.m_ranges.last().unwrap().layer_height_range.1;
            self.m_ranges.push(LayerRange { layer_height_range: (s, f64::MAX), config: None });
        }
    }

    /// PrintApply.cpp:388-399 `const DynamicPrintConfig* config(const t_layer_height_range &range) const`
    pub fn config(&self, range: (f64, f64)) -> Option<(f64, f64)> {
        // std::lower_bound by LayerRange::operator< (the std::pair <)
        let key = (range.0 - EPSILON, range.1 - EPSILON);
        let it = self.m_ranges.partition_point(|l| {
            l.layer_height_range.0 < key.0 || (!(key.0 < l.layer_height_range.0) && l.layer_height_range.1 < key.1)
        });
        if it == self.m_ranges.len()
            || (self.m_ranges[it].layer_height_range.0 - range.0).abs() > EPSILON
            || (self.m_ranges[it].layer_height_range.1 - range.1).abs() > EPSILON
        {
            return None; // desired range doesn't found
        }
        self.m_ranges[it].config
    }

    /// PrintApply.cpp:401
    pub fn begin(&self) -> std::slice::Iter<'_, LayerRange> {
        self.m_ranges.iter()
    }
    /// PrintApply.cpp:402
    pub fn end(&self) -> usize {
        self.m_ranges.len()
    }
    /// PrintApply.cpp:403
    pub fn size(&self) -> usize {
        self.m_ranges.len()
    }
}

/// PrintApply.cpp:411-418 `enum Status`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModelObjectStatusStatus {
    Unknown,
    Old,
    New,
    Moved,
    Deleted,
}

/// PrintApply.cpp:420-424 `enum class PrintObjectRegionsStatus`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PrintObjectRegionsStatus {
    Invalid,
    Valid,
    PartiallyValid,
}

/// PrintApply.cpp:409-443 `struct ModelObjectStatus`: to track Model / ModelObject updates between the
/// front end and back end.
#[derive(Debug, Clone)]
pub struct ModelObjectStatus {
    /// Key of the set.
    pub id: ObjectID,
    /// Status of this ModelObject with id on apply().
    pub status: ModelObjectStatusStatus,
    /// PrintObjects to be generated for this ModelObject including their base transformation.
    pub print_instances: Vec<PrintObjectTrafoAndInstances>,
    /// Regions shared by the associated PrintObjects.
    pub print_object_regions: Option<Rc<RefCell<PrintObjectRegions>>>,
    /// Status of the above.
    pub print_object_regions_status: PrintObjectRegionsStatus,
}

impl ModelObjectStatus {
    /// PrintApply.cpp:426 `ModelObjectStatus(ObjectID id, Status status = Unknown)`
    pub fn new(id: ObjectID, status: ModelObjectStatusStatus) -> Self {
        ModelObjectStatus {
            id,
            status,
            print_instances: Vec::new(),
            print_object_regions: None,
            print_object_regions_status: PrintObjectRegionsStatus::Invalid,
        }
    }
}

/// PrintApply.cpp:427 `~ModelObjectStatus() { if (print_object_regions) print_object_regions->ref_cnt_dec(); }`
impl Drop for ModelObjectStatus {
    fn drop(&mut self) {
        if let Some(r) = &self.print_object_regions {
            if let Ok(mut r) = r.try_borrow_mut() {
                r.ref_cnt_dec();
            }
        }
    }
}

/// PrintApply.cpp:445-475 `struct ModelObjectStatusDB`
#[derive(Debug, Default)]
pub struct ModelObjectStatusDB {
    /// `std::set<ModelObjectStatus> db`, ordered by id.
    pub db: Vec<ModelObjectStatus>,
}

impl ModelObjectStatusDB {
    fn find(&self, id: ObjectID) -> Result<usize, usize> {
        self.db.binary_search_by(|s| s.id.cmp(&id))
    }

    /// PrintApply.cpp:447-450 `void add(const ModelObject &model_object, const ModelObjectStatus::Status status)`
    pub fn add(&mut self, model_object: &ModelObject, status: ModelObjectStatusStatus) {
        let pos = self.find(model_object.id());
        debug_assert!(pos.is_err());
        let pos = pos.unwrap_or_else(|p| p);
        self.db.insert(pos, ModelObjectStatus::new(model_object.id(), status));
    }

    /// PrintApply.cpp:452-459 `bool add_if_new(const ModelObject &model_object, const ModelObjectStatus::Status status)`
    pub fn add_if_new(&mut self, model_object: &ModelObject, status: ModelObjectStatusStatus) -> bool {
        match self.find(model_object.id()) {
            Err(pos) => {
                self.db.insert(pos, ModelObjectStatus::new(model_object.id(), status));
                true
            }
            Ok(_) => false,
        }
    }

    /// PrintApply.cpp:461-465 `const ModelObjectStatus& get(const ModelObject &model_object)`
    pub fn get(&self, model_object: &ModelObject) -> &ModelObjectStatus {
        let it = self.find(model_object.id()).expect("it != db.end()");
        &self.db[it]
    }

    /// The status of an object, to update it (the C++ `const_cast`s the set element).
    pub fn get_mut(&mut self, model_object: &ModelObject) -> &mut ModelObjectStatus {
        let it = self.find(model_object.id()).expect("it != db.end()");
        &mut self.db[it]
    }

    /// PrintApply.cpp:467-471 `const ModelObjectStatus& reuse(const ModelObject &model_object)`
    pub fn reuse(&self, model_object: &ModelObject) -> &ModelObjectStatus {
        let result = self.get(model_object);
        debug_assert!(result.status != ModelObjectStatusStatus::Deleted);
        result
    }
}

/// PrintApply.cpp:478-483 `enum Status`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PrintObjectStatusStatus {
    Unknown,
    Deleted,
    Reused,
    New,
}

/// PrintApply.cpp:477-504 `struct PrintObjectStatus`
#[derive(Debug, Clone)]
pub struct PrintObjectStatus {
    /// ID of the ModelObject & PrintObject
    pub id: ObjectID,
    /// Pointer to the old PrintObject: its index in `Print::m_objects`.
    pub print_object: Option<usize>,
    /// Trafo generated with model_object->world_matrix(true)
    pub trafo: Transform3d,
    pub status: PrintObjectStatusStatus,
}

/// PrintApply.cpp:506-544 `class PrintObjectStatusDB`
#[derive(Debug, Default)]
pub struct PrintObjectStatusDB {
    /// `std::multiset<PrintObjectStatus> m_db`, ordered by id; equal ids in the order of insertion.
    m_db: Vec<PrintObjectStatus>,
}

impl PrintObjectStatusDB {
    /// PrintApply.cpp:511-514 `PrintObjectStatusDB(const PrintObjectPtrs &print_objects)`
    pub fn new(print: &crate::print::faithful::Print) -> Self {
        let mut db = PrintObjectStatusDB::default();
        for (i, print_object) in print.m_objects.iter().enumerate() {
            let status = PrintObjectStatus {
                id: print_object.model_object(print).id(),
                print_object: Some(i),
                trafo: *print_object.trafo(),
                status: PrintObjectStatusStatus::Unknown,
            };
            // multiset::emplace inserts at the upper bound of the equal range
            let pos = db.m_db.partition_point(|s| s.id <= status.id);
            db.m_db.insert(pos, status);
        }
        db
    }

    /// PrintApply.cpp:522-524 `iterator_range get_range(const ModelObject &model_object) const`: the
    /// index range of the statuses of the object.
    pub fn get_range(&self, model_object: &ModelObject) -> std::ops::Range<usize> {
        self.get_range_by_id(model_object.id())
    }

    /// PrintApply.cpp:526-528 `iterator_range get_range(const ModelObjectStatus &model_object_status) const`
    pub fn get_range_by_id(&self, id: ObjectID) -> std::ops::Range<usize> {
        let lo = self.m_db.partition_point(|s| s.id < id);
        let hi = self.m_db.partition_point(|s| s.id <= id);
        lo..hi
    }

    /// PrintApply.cpp:530-532 `size_t count(const ModelObject &model_object)`
    pub fn count(&self, model_object: &ModelObject) -> usize {
        self.get_range(model_object).len()
    }

    /// PrintApply.cpp:534-535 `begin()` / `end()`: the statuses.
    pub fn statuses(&self) -> &[PrintObjectStatus] {
        &self.m_db
    }
    /// The statuses, to update them (the C++ `const_cast`s the multiset elements).
    pub fn statuses_mut(&mut self) -> &mut [PrintObjectStatus] {
        &mut self.m_db
    }

    /// PrintApply.cpp:537-539
    pub fn clear(&mut self) {
        self.m_db.clear();
    }
}

/// PrintApply.cpp:545-549 `static inline bool model_volume_solid_or_modifier(const ModelVolume &mv)`
pub fn model_volume_solid_or_modifier(mv: &ModelVolume) -> bool {
    let type_ = mv.type_();
    type_ == ModelVolumeType::ModelPart || type_ == ModelVolumeType::NegativeVolume || type_ == ModelVolumeType::ParameterModifier
}

/// PrintApply.cpp:551-557 `static inline Transform3f trafo_for_bbox(const Transform3d &object_trafo, const Transform3d &volume_trafo)`
pub fn trafo_for_bbox(object_trafo: &Transform3d, volume_trafo: &Transform3d) -> Transform3f {
    let mut m = crate::model::eigen_mul_affine(object_trafo, volume_trafo);
    m[(0, 3)] = 0.0;
    m[(1, 3)] = 0.0;
    m.map(|v| v as f32)
}

/// Eigen `Transform3f * Vec3f`: the linear part times the vector, plus the translation, in float
/// (each row a product sum `((m0*x + m1*y) + m2*z) + t`, contracted to FMAs as clang does).
fn transform3f_mul(m: &Transform3f, v: &crate::triangle_mesh::Vec3f) -> [f32; 3] {
    let mut out = [0.0f32; 3];
    for r in 0..3 {
        let p = m[(r, 1)].mul_add(v.y, m[(r, 0)] * v.x);
        let p = m[(r, 2)].mul_add(v.z, p);
        out[r] = p + m[(r, 3)];
    }
    out
}

/// PrintApply.cpp:585-600 `static PrintObjectRegions::BoundingBox transformed_its_bbox2d(const
/// indexed_triangle_set &its, const Transform3f &m, float offset)`
pub fn transformed_its_bbox2d(its: &crate::normal_utils::indexed_triangle_set, m: &Transform3f, offset: f32) -> AlignedBox3f {
    if its.indices.is_empty() || its.vertices.is_empty() {
        return AlignedBox3f::default();
    }
    let first = transform3f_mul(m, &its.vertices[its.indices[0][0] as usize]);
    let mut bbox = AlignedBox3f::new(first, first);
    for tri in &its.indices {
        for i in 0..3 {
            bbox.extend(transform3f_mul(m, &its.vertices[tri[i] as usize]));
        }
    }
    let eps = EPSILON as f32;
    bbox.min = [bbox.min[0] - offset, bbox.min[1] - offset, bbox.min[2] - eps];
    bbox.max = [bbox.max[0] + offset, bbox.max[1] + offset, bbox.max[2] + eps];
    bbox
}

/// PrintApply.cpp:602-670 `static void transformed_its_bboxes_in_z_ranges(const indexed_triangle_set &its,
/// const Transform3f &m, const std::vector<t_layer_height_range> &z_ranges,
/// std::vector<std::pair<PrintObjectRegions::BoundingBox, bool>> &bboxes, const float offset)`
pub fn transformed_its_bboxes_in_z_ranges(
    its: &crate::normal_utils::indexed_triangle_set,
    m: &Transform3f,
    z_ranges: &[(f64, f64)],
    bboxes: &mut Vec<(AlignedBox3f, bool)>,
    offset: f32,
) {
    bboxes.clear();
    bboxes.resize(z_ranges.len(), (AlignedBox3f::default(), false));
    for tri in &its.indices {
        let pts = [
            transform3f_mul(m, &its.vertices[tri[0] as usize]),
            transform3f_mul(m, &its.vertices[tri[1] as usize]),
            transform3f_mul(m, &its.vertices[tri[2] as usize]),
        ];
        for irange in 0..z_ranges.len() {
            let z_range = z_ranges[irange];
            let bbox = &mut bboxes[irange];
            let mut bbox_extend = |p: [f32; 3]| {
                if bbox.1 {
                    bbox.0.extend(p);
                } else {
                    bbox.0.min = p;
                    bbox.0.max = p;
                    bbox.1 = true;
                }
            };
            // The float z compared with the double range limits, as the C++ promotes it.
            let (zlo, zhi) = (z_range.0, z_range.1);
            let mut iprev = 2usize;
            for iedge in 0..3usize {
                let mut p1 = pts[iprev];
                let mut p2 = pts[iedge];
                // Sort the edge points by Z.
                if p1[2] > p2[2] {
                    std::mem::swap(&mut p1, &mut p2);
                }
                if (p2[2] as f64) <= zlo || (p1[2] as f64) >= zhi {
                    // Out of this slab.
                } else if (p1[2] as f64) < zlo {
                    if (p2[2] as f64) > zhi {
                        // Two intersections.
                        let zspan = p2[2] - p1[2];
                        let t1 = ((zlo - p1[2] as f64) as f32) / zspan;
                        let t2 = ((zhi - p1[2] as f64) as f32) / zspan;
                        let v = [p2[0] - p1[0], p2[1] - p1[1]];
                        bbox_extend([v[0].mul_add(t1, p1[0]), v[1].mul_add(t1, p1[1]), zlo as f32]);
                        bbox_extend([v[0].mul_add(t2, p1[0]), v[1].mul_add(t2, p1[1]), zhi as f32]);
                    } else {
                        // Single intersection with the lower limit.
                        let t = ((zlo - p1[2] as f64) as f32) / (p2[2] - p1[2]);
                        let v = [p2[0] - p1[0], p2[1] - p1[1]];
                        bbox_extend([v[0].mul_add(t, p1[0]), v[1].mul_add(t, p1[1]), zlo as f32]);
                        bbox_extend(p2);
                    }
                } else if (p2[2] as f64) > zhi {
                    // Single intersection with the upper limit.
                    let t = ((zhi - p1[2] as f64) as f32) / (p2[2] - p1[2]);
                    let v = [p2[0] - p1[0], p2[1] - p1[1]];
                    bbox_extend([v[0].mul_add(t, p1[0]), v[1].mul_add(t, p1[1]), zhi as f32]);
                    bbox_extend(p1);
                } else {
                    // Both points are inside.
                    bbox_extend(p1);
                    bbox_extend(p2);
                }
                iprev = iedge;
            }
        }
    }

    let eps = EPSILON as f32;
    for bbox in bboxes.iter_mut() {
        bbox.0.min = [bbox.0.min[0] - offset, bbox.0.min[1] - offset, bbox.0.min[2] - eps];
        bbox.0.max = [bbox.0.max[0] + offset, bbox.0.max[1] + offset, bbox.0.max[2] + eps];
    }
}

/// PrintApply.cpp:672-703 `void print_objects_regions_invalidate_keep_some_volumes(PrintObjectRegions
/// &print_object_regions, ModelVolumePtrs old_volumes, ModelVolumePtrs new_volumes)`: keep
/// print_object_regions, but delete those volumes, which were either removed from new_volumes, or
/// which rotated or scaled, so they need their bounding boxes to be recalculated.
pub fn print_objects_regions_invalidate_keep_some_volumes(
    print_object_regions: &mut PrintObjectRegions,
    old_object: &ModelObject,
    new_object: &ModelObject,
) {
    print_object_regions.all_regions.clear();

    let old_volumes = volume_indices_sorted_by_id(old_object);
    let new_volumes = volume_indices_sorted_by_id(new_object);

    let mut i_cached_volume = 0usize;
    let mut last_cached_volume = 0usize;
    let mut i_old = 0usize;
    for &i_new in &new_volumes {
        let nv = &new_object.volumes[i_new];
        if model_volume_solid_or_modifier(nv) {
            while i_old < old_volumes.len() {
                if old_object.volumes[old_volumes[i_old]].id() >= nv.id() {
                    break;
                }
                i_old += 1;
            }
            if i_old != old_volumes.len() && old_object.volumes[old_volumes[i_old]].id() == nv.id() {
                let ov = &old_object.volumes[old_volumes[i_old]];
                if eigen_is_approx_mat4(&ov.get_matrix(false, false, false, false), &nv.get_matrix(false, false, false, false)) {
                    // Reuse the volume.
                    while print_object_regions.cached_volume_ids[i_cached_volume] < ov.id() {
                        i_cached_volume += 1;
                    }
                    print_object_regions.cached_volume_ids[last_cached_volume] = print_object_regions.cached_volume_ids[i_cached_volume];
                    last_cached_volume += 1;
                    i_cached_volume += 1;
                }
                // else: don't reuse the volume.
            }
        }
    }
    print_object_regions.cached_volume_ids.truncate(last_cached_volume);
}

/// Eigen `a.isApprox(b)` for a 4x4 matrix with the default precision (1e-12).
fn eigen_is_approx_mat4(a: &Transform3d, b: &Transform3d) -> bool {
    let prec = 1e-12;
    let (mut dd, mut aa, mut bb) = (0.0, 0.0, 0.0);
    for c in 0..4 {
        for r in 0..4 {
            let d = a[(r, c)] - b[(r, c)];
            dd += d * d;
            aa += a[(r, c)] * a[(r, c)];
            bb += b[(r, c)] * b[(r, c)];
        }
    }
    dd <= prec * prec * aa.min(bb)
}

/// PrintApply.cpp:705-710 `const PrintObjectRegions::BoundingBox* find_volume_extents(const
/// PrintObjectRegions::LayerRangeRegions &layer_range, const ModelVolume &volume)`: the index of the
/// volume's extents in the layer range.
pub fn find_volume_extents(layer_range: &LayerRangeRegions, volume: &ModelVolume) -> Option<usize> {
    let it = layer_range.volumes.partition_point(|l| l.volume_id < volume.id());
    if it != layer_range.volumes.len() && layer_range.volumes[it].volume_id == volume.id() {
        Some(it)
    } else {
        None
    }
}

/// PrintApply.cpp:712-731 `PrintObjectRegions::BoundingBox find_modifier_volume_extents(const
/// PrintObjectRegions::LayerRangeRegions &layer_range, const int this_region_id)`: a bounding box of
/// a topmost printable volume referenced by this modifier given this_region_id.
pub fn find_modifier_volume_extents(layer_range: &LayerRangeRegions, this_region_id: i32, model_object: &ModelObject) -> AlignedBox3f {
    // Find the top-most printable volume of this modifier, or the printable volume itself.
    let this_region = &layer_range.volume_regions[this_region_id as usize];
    let this_volume = &model_object.volumes[this_region.model_volume.unwrap()];
    let this_extents = find_volume_extents(layer_range, this_volume).expect("this_extents");
    let mut out = layer_range.volumes[this_extents].bbox;
    if !this_volume.is_model_part() {
        let mut parent_region_id = this_region.parent;
        loop {
            debug_assert!(parent_region_id >= 0);
            let parent_region = &layer_range.volume_regions[parent_region_id as usize];
            let parent_volume = &model_object.volumes[parent_region.model_volume.unwrap()];
            let parent_extents = find_volume_extents(layer_range, parent_volume).expect("parent_extents");
            let pb = layer_range.volumes[parent_extents].bbox;
            out.extend(pb.min);
            out.extend(pb.max);
            if parent_volume.is_model_part() {
                break;
            }
            parent_region_id = parent_region.parent;
        }
    }
    out
}

/// PrintApply.cpp:736 `void print_region_ref_inc(PrintRegion &r) { ++ r.m_ref_cnt; }`
pub fn print_region_ref_inc(r: &mut PrintRegion) {
    r.m_ref_cnt += 1;
}
/// PrintApply.cpp:737
pub fn print_region_ref_reset(r: &mut PrintRegion) {
    r.m_ref_cnt = 0;
}
/// PrintApply.cpp:738
pub fn print_region_ref_cnt(r: &PrintRegion) -> i32 {
    r.m_ref_cnt
}



/// PrintApply.cpp:740-896 `bool verify_update_print_object_regions(ModelVolumePtrs model_volumes, const
/// PrintRegionConfig &default_region_config, size_t num_extruders, const std::vector<unsigned int>
/// &painting_extruders, PrintObjectRegions &print_object_regions, const std::function<void(const
/// PrintRegionConfig&, const PrintRegionConfig&, const t_config_option_keys&)> &callback_invalidate,
/// std::vector<int>& variant_index)`: verify whether the PrintRegions of a PrintObject are still
/// valid, possibly after updating the region configs. Before region configs are updated,
/// callback_invalidate() is called to possibly stop background processing. Returns false if this
/// object needs to be resliced because regions were merged or split.
///
/// `model_object` holds the `model_volumes`. FIDELITY-NOTE: a VolumeRegion names its volume by its
/// index in the ModelObject; the C++ looks the volume up by id in the list sorted by id, which is
/// the same volume.
pub fn verify_update_print_object_regions(
    model_object: &ModelObject,
    model: Option<&Model>,
    default_region_config: &PrintRegionConfig,
    num_extruders: usize,
    _painting_extruders: &[u32],
    print_object_regions: &mut PrintObjectRegions,
    callback_invalidate: &mut dyn FnMut(&PrintRegionConfig, &PrintRegionConfig, &[String]),
    variant_index: &mut Vec<i32>,
) -> bool {
    use crate::print::faithful::static_config_equal;
    for region in &print_object_regions.all_regions {
        print_region_ref_reset(&mut region.borrow_mut());
    }

    let layer_range_config = |key: Option<(f64, f64)>| -> Option<&DynamicPrintConfig> {
        let (lo, hi) = key?;
        model_object.layer_config_ranges.get(&(ordered_float::OrderedFloat(lo), ordered_float::OrderedFloat(hi))).map(|c| c.get())
    };

    // Change the config of a region referenced for the first time, or tell that it splits.
    let mut update_region = |region: &PrintRegionPtr, cfg: PrintRegionConfig, callback_invalidate: &mut dyn FnMut(&PrintRegionConfig, &PrintRegionConfig, &[String])| -> bool {
        let changed = !static_config_equal(&cfg, region.borrow().config());
        if changed {
            // Region configuration changed.
            if print_region_ref_cnt(&region.borrow()) == 0 {
                // Region is referenced for the first time. Just change its parameters.
                // Stop the background process before assigning new configuration to the regions.
                let diff = region.borrow().config().diff(&cfg).unwrap_or_default();
                callback_invalidate(region.borrow().config(), &cfg, &diff);
                region.borrow_mut().config_apply_only(&cfg, &diff, false);
            } else {
                // Region is referenced multiple times, thus the region is being split. We need to reslice.
                return false;
            }
        }
        print_region_ref_inc(&mut region.borrow_mut());
        true
    };

    // Verify and / or update PrintRegions produced by ModelVolumes, layer range modifiers, modifier volumes.
    for ilr in 0..print_object_regions.layer_ranges.len() {
        let layer_range = &print_object_regions.layer_ranges[ilr];
        // Remember whether a given modifier ModelVolume was visited already.
        let mut it_model_volume_modifier_last: Option<ObjectID> = None;
        for iregion in 0..layer_range.volume_regions.len() {
            let region = &layer_range.volume_regions[iregion];
            let volume = &model_object.volumes[region.model_volume.unwrap()];
            if !(volume.is_model_part() || volume.is_modifier()) {
                continue;
            }
            if volume.is_modifier() && Some(volume.id()) != it_model_volume_modifier_last {
                // A modifier ModelVolume is visited for the first time.
                it_model_volume_modifier_last = Some(volume.id());
                let mut next_region_id = iregion as i32;
                let bbox = layer_range.volumes[find_volume_extents(layer_range, volume).expect("bbox")].bbox;
                let mut parent_region_id = next_region_id - 1;
                while parent_region_id >= 0 {
                    let parent_region = &layer_range.volume_regions[parent_region_id as usize];
                    let parent_volume = &model_object.volumes[parent_region.model_volume.unwrap()];
                    if parent_volume.is_model_part() || parent_volume.is_modifier() {
                        // volume_regions are produced in decreasing order of parent volume_regions ids.
                        if (next_region_id as usize) < layer_range.volume_regions.len()
                            && layer_range.volume_regions[next_region_id as usize].model_volume == region.model_volume
                            && layer_range.volume_regions[next_region_id as usize].parent == parent_region_id
                        {
                            // A parent region is already overridden.
                            next_region_id += 1;
                        } else if find_modifier_volume_extents(layer_range, parent_region_id, model_object).intersects(&bbox) {
                            // Such parent region does not exist. If it is needed, then we need to reslice.
                            // Only create new region for a modifier, which actually modifies config of it's parent.
                            let parent_config = parent_region.region.as_ref().unwrap().borrow().config().clone();
                            let config = region_config_from_model_volume(&parent_config, None, volume, model_object, model, num_extruders, variant_index);
                            if !static_config_equal(&config, &parent_config) {
                                // This modifier newly overrides a region, which it did not before. We need to reslice.
                                return false;
                            }
                        }
                    }
                    parent_region_id -= 1;
                }
            }
            let cfg = if region.parent == -1 {
                region_config_from_model_volume(default_region_config, layer_range_config(layer_range.config), volume, model_object, model, num_extruders, variant_index)
            } else {
                let parent_config = layer_range.volume_regions[region.parent as usize].region.as_ref().unwrap().borrow().config().clone();
                region_config_from_model_volume(&parent_config, None, volume, model_object, model, num_extruders, variant_index)
            };
            let r = region.region.clone().unwrap();
            if !update_region(&r, cfg, callback_invalidate) {
                return false;
            }
        }
    }

    // Verify and / or update PrintRegions produced by color painting.
    for layer_range in &print_object_regions.layer_ranges {
        for region in &layer_range.painted_regions {
            let parent_region = &layer_range.volume_regions[region.parent as usize];
            let mut cfg = parent_region.region.as_ref().unwrap().borrow().config().clone();
            *cfg.opt_int_mut("wall_filament") = region.extruder_id as i32;
            *cfg.opt_int_mut("solid_infill_filament") = region.extruder_id as i32;
            *cfg.opt_int_mut("sparse_infill_filament") = region.extruder_id as i32;
            if !update_region(region.region.as_ref().unwrap(), cfg, callback_invalidate) {
                return false;
            }
        }
    }

    // Verify and / or update PrintRegions produced by fuzzy skin painting.
    let disabled_fuzzy = crate::print_config::enums::FuzzySkinType::Disabled_fuzzy as i32;
    let all_fuzzy = crate::print_config::enums::FuzzySkinType::All as i32;
    for layer_range in &print_object_regions.layer_ranges {
        for region in &layer_range.fuzzy_skin_painted_regions {
            let parent_print_region = region.parent_print_object_region(layer_range).expect("parent region");
            let mut cfg = parent_print_region.borrow().config().clone();
            let v = cfg.option("fuzzy_skin").and_then(|o| o.get_int().ok()).unwrap_or(0);
            if v != disabled_fuzzy {
                if let Ok(Some(o)) = cfg.option_mut("fuzzy_skin", false) {
                    let _ = o.set_int(all_fuzzy);
                }
            }
            if !update_region(region.region.as_ref().unwrap(), cfg, callback_invalidate) {
                return false;
            }
        }
    }

    // Lastly verify, whether some regions were not merged.
    {
        let mut regions: Vec<PrintRegionPtr> = Vec::with_capacity(print_object_regions.all_regions.len());
        for region in &print_object_regions.all_regions {
            debug_assert!(print_region_ref_cnt(&region.borrow()) > 0);
            regions.push(region.clone());
        }
        // std::sort by config_hash: which region of a run of equal hashes comes first does not
        // change the result, every pair of the run is compared.
        regions.sort_by(|l, r| l.borrow().config_hash().cmp(&r.borrow().config_hash()));
        for i in 0..regions.len() {
            let hash = regions[i].borrow().config_hash();
            let mut j = i + 1;
            while j < regions.len() && regions[j].borrow().config_hash() == hash {
                if static_config_equal(regions[i].borrow().config(), regions[j].borrow().config())
                    && regions[i].borrow().gradient_volume_id() == regions[j].borrow().gradient_volume_id()
                {
                    // Regions were merged. We need to reslice.
                    return false;
                }
                j += 1;
            }
        }
    }

    true
}

/// PrintApply.cpp:898-971 `void update_volume_bboxes(std::vector<PrintObjectRegions::LayerRangeRegions>
/// &layer_ranges, std::vector<ObjectID> &cached_volume_ids, ModelVolumePtrs model_volumes, const
/// Transform3d &object_trafo, const float offset)`
pub fn update_volume_bboxes(
    layer_ranges: &mut Vec<LayerRangeRegions>,
    cached_volume_ids: &mut Vec<ObjectID>,
    model_object: &ModelObject,
    object_trafo: &Transform3d,
    offset: f32,
) {
    // output will be sorted by the order of model_volumes sorted by their ObjectIDs.
    let model_volumes = volume_indices_sorted_by_id(model_object);

    if layer_ranges.len() == 1 {
        let layer_range = &mut layer_ranges[0];
        let volumes_old: Vec<VolumeExtents> = std::mem::take(&mut layer_range.volumes);
        layer_range.volumes.reserve(model_volumes.len());
        for &iv in &model_volumes {
            let model_volume = &model_object.volumes[iv];
            if model_volume_solid_or_modifier(model_volume) {
                if cached_volume_ids.binary_search(&model_volume.id()).is_ok() {
                    let it = volumes_old.partition_point(|l| l.volume_id < model_volume.id());
                    if it != volumes_old.len() && volumes_old[it].volume_id == model_volume.id() {
                        layer_range.volumes.push(volumes_old[it]);
                    }
                } else {
                    let its = &model_volume.mesh().its;
                    if its.indices.is_empty() || its.vertices.is_empty() {
                        continue;
                    }
                    layer_range.volumes.push(VolumeExtents {
                        volume_id: model_volume.id(),
                        bbox: transformed_its_bbox2d(its, &trafo_for_bbox(object_trafo, &model_volume.get_matrix(false, false, false, false)), offset),
                    });
                }
            }
        }
    } else {
        let mut volumes_old: Vec<Vec<VolumeExtents>> = Vec::new();
        if cached_volume_ids.is_empty() {
            for layer_range in layer_ranges.iter_mut() {
                layer_range.volumes.clear();
            }
        } else {
            volumes_old.reserve(layer_ranges.len());
            for layer_range in layer_ranges.iter_mut() {
                volumes_old.push(std::mem::take(&mut layer_range.volumes));
            }
        }

        let mut bboxes: Vec<(AlignedBox3f, bool)> = Vec::new();
        let mut ranges: Vec<(f64, f64)> = Vec::with_capacity(layer_ranges.len());
        for layer_range in layer_ranges.iter() {
            let mut r = layer_range.layer_height_range;
            r.0 -= EPSILON;
            r.1 += EPSILON;
            ranges.push(r);
        }
        for &iv in &model_volumes {
            let model_volume = &model_object.volumes[iv];
            if model_volume_solid_or_modifier(model_volume) {
                if cached_volume_ids.binary_search(&model_volume.id()).is_ok() {
                    for (ilr, layer_range) in layer_ranges.iter_mut().enumerate() {
                        let vold = &volumes_old[ilr];
                        let it = vold.partition_point(|l| l.volume_id < model_volume.id());
                        if it != vold.len() && vold[it].volume_id == model_volume.id() {
                            layer_range.volumes.push(vold[it]);
                        }
                    }
                } else {
                    transformed_its_bboxes_in_z_ranges(
                        &model_volume.mesh().its,
                        &trafo_for_bbox(object_trafo, &model_volume.get_matrix(false, false, false, false)),
                        &ranges,
                        &mut bboxes,
                        offset,
                    );
                    for (ilr, layer_range) in layer_ranges.iter_mut().enumerate() {
                        let bbox = &bboxes[ilr];
                        if bbox.1 {
                            layer_range.volumes.push(VolumeExtents { volume_id: model_volume.id(), bbox: bbox.0 });
                        }
                    }
                }
            }
        }
    }

    cached_volume_ids.clear();
    cached_volume_ids.reserve(model_volumes.len());
    for &iv in &model_volumes {
        let v = &model_object.volumes[iv];
        if model_volume_solid_or_modifier(v) {
            cached_volume_ids.push(v.id());
        }
    }
}

/// PrintObject.cpp:3611-3616 `static void clamp_exturder_to_default(ConfigOptionInt &opt, size_t num_extruders)`
fn clamp_exturder_to_default(config: &mut DynamicPrintConfig, key: &str, num_extruders: usize) {
    let v = config.opt_int(key);
    if v > num_extruders as i32 {
        // assign the default extruder
        *config.opt_int_mut(key) = 1;
    }
}

/// PrintObject.cpp:3617-3625 `static void clamp_exturder_to_default_protect0(ConfigOptionInt &opt, size_t num_extruders)`
fn clamp_exturder_to_default_protect0(config: &mut DynamicPrintConfig, key: &str, num_extruders: usize) {
    let v = config.opt_int(key);
    if v > num_extruders as i32 {
        *config.opt_int_mut(key) = 1;
    } else if v < 1 {
        *config.opt_int_mut(key) = 1;
    }
}

/// PrintObject.cpp:3627-3639 `PrintObjectConfig PrintObject::object_config_from_model_object(const
/// PrintObjectConfig &default_object_config, const ModelObject &object, size_t num_extruders,
/// std::vector<int>& variant_index)`, which Print::apply calls.
pub fn object_config_from_model_object(
    default_object_config: &DynamicPrintConfig,
    object: &ModelObject,
    num_extruders: usize,
    variant_index: &mut Vec<i32>,
) -> DynamicPrintConfig {
    let mut config = default_object_config.clone();
    {
        let mut src_normalized = object.config.get().clone();
        let _ = src_normalized.normalize_fdm();
        let _ = crate::print_config::config_def::update_static_print_config_from_dynamic(
            &mut config,
            &src_normalized,
            variant_index.clone(),
            &crate::print_config::config_def::print_options_with_variant,
            1,
        );
    }
    // Clamp invalid extruders to the default extruder (with index 1).
    clamp_exturder_to_default(&mut config, "support_filament", num_extruders);
    clamp_exturder_to_default(&mut config, "support_interface_filament", num_extruders);
    config
}

/// PrintObject.cpp:3641-3642 `keys_extruders`
const KEYS_EXTRUDERS: [&str; 3] = ["sparse_infill_filament", "solid_infill_filament", "wall_filament"];

/// The `ConfigOptionInt` of a key, None when the option is missing or of another class
/// (`DynamicConfig::opt<ConfigOptionInt>`).
fn opt_config_int(config: &DynamicPrintConfig, key: &str) -> Option<i32> {
    config.option(key).and_then(|o| o.as_any().downcast_ref::<crate::config::ConfigOptionInt>()).map(|o| o.value)
}

/// PrintObject.cpp:3644-3678 `static void apply_to_print_region_config(PrintRegionConfig &out, const
/// DynamicPrintConfig &in, std::vector<int>& variant_index)`
fn apply_to_print_region_config(out: &mut DynamicPrintConfig, input: &DynamicPrintConfig, variant_index: &mut Vec<i32>) {
    // 1) Copy the "extruder key to sparse_infill_filament and wall_filament.
    if let Some(extruder) = opt_config_int(input, "extruder") {
        if extruder != 0 {
            // Not a default extruder.
            *out.opt_int_mut("sparse_infill_filament") = extruder;
            *out.opt_int_mut("solid_infill_filament") = extruder;
            *out.opt_int_mut("wall_filament") = extruder;
        }
    }
    // 2) Copy the rest of the values.
    for key in input.keys() {
        if key == "extruder" {
            continue;
        }
        let src = input.option(&key).expect("key of the config");
        let Ok(Some(my_opt)) = out.option_mut(&key, false) else {
            continue;
        };
        if KEYS_EXTRUDERS.contains(&key.as_str()) {
            // Ignore "default" extruders.
            // BBS: 2025-10-20 skip these filament settings, they will be processed out of this func
        } else if my_opt.operator_ne(src).unwrap_or(true) {
            if my_opt.is_scalar() || variant_index.is_empty() || !crate::print_config::config_def::print_options_with_variant.contains(&key) {
                let _ = my_opt.set(src);
            } else {
                if let Some(opt_vec_src) = my_opt.as_vector_base_mut() {
                    let _ = opt_vec_src.set_to_index(src, variant_index, 1);
                }
            }
        }
    }
}

/// PrintObject.cpp:3681-3722 `static const char *s_modifier_ignored_infill_keys[]`: internal infill
/// keys a modifier may set that should be dropped when modifier_ignore_infill is on.
const S_MODIFIER_IGNORED_INFILL_KEYS: [&str; 40] = [
    "sparse_infill_density",
    "sparse_infill_pattern",
    "conformal_infill",
    "conformal_stagger",
    "conformal_link_keep_layers",
    "conformal_link_flip_layers",
    "conformal_pole",
    "conformal_ray_count",
    "conformal_hub_radius",
    "infill_direction",
    "symmetric_infill_y_axis",
    "infill_shift_step",
    "sparse_infill_lattice_angle_1",
    "sparse_infill_lattice_angle_2",
    "infill_rotate_step",
    "skeleton_infill_density",
    "skin_infill_density",
    "fill_multiline",
    "infill_lock_depth",
    "skin_infill_depth",
    "locked_skin_infill_pattern",
    "locked_skeleton_infill_pattern",
    "sparse_infill_filament",
    "sparse_infill_line_width",
    "skin_infill_line_width",
    "skeleton_infill_line_width",
    "infill_wall_overlap",
    "sparse_infill_speed",
    "infill_combination",
    "minimum_sparse_infill_area",
    "internal_solid_infill_pattern",
    "internal_solid_infill_line_width",
    "internal_solid_infill_speed",
    "solid_infill_filament",
    "sparse_infill_anchor",
    "sparse_infill_anchor_max",
    "detect_narrow_internal_solid_infill",
    "embedding_wall_into_infill",
    "infill_instead_top_bottom_surfaces",
    "bridge_angle",
];

/// PrintObject.cpp:3724-3730 `static bool volume_ignores_modifier_infill(const ModelVolume &volume)`
fn volume_ignores_modifier_infill(volume: &ModelVolume) -> bool {
    if !volume.is_modifier() {
        return false;
    }
    volume
        .config
        .get()
        .option("modifier_ignore_infill")
        .and_then(|o| o.as_any().downcast_ref::<crate::config::ConfigOptionBool>())
        .map(|o| o.value)
        .unwrap_or(false)
}

/// PrintObject.cpp:3732-3740 `static void restore_parent_internal_infill(PrintRegionConfig &config, const PrintRegionConfig &parent)`
fn restore_parent_internal_infill(config: &mut DynamicPrintConfig, parent: &DynamicPrintConfig) {
    for key in S_MODIFIER_IGNORED_INFILL_KEYS {
        let Some(src) = parent.option(key) else {
            continue;
        };
        if let Ok(Some(dst)) = config.option_mut(key, false) {
            let _ = dst.set(src);
        }
    }
}

/// PrintObject.cpp:3742-3818 `PrintRegionConfig region_config_from_model_volume(const PrintRegionConfig
/// &default_or_parent_region_config, const DynamicPrintConfig *layer_range_config, const ModelVolume
/// &volume, size_t num_extruders, std::vector<int>& variant_index)`, which PrintApply.cpp declares
/// (PrintApply.cpp:733) and calls. `object` is `volume.get_object()`, `model` its model (for the
/// volume's material).
pub fn region_config_from_model_volume(
    default_or_parent_region_config: &PrintRegionConfig,
    layer_range_config: Option<&DynamicPrintConfig>,
    volume: &ModelVolume,
    object: &ModelObject,
    model: Option<&Model>,
    num_extruders: usize,
    variant_index: &mut Vec<i32>,
) -> PrintRegionConfig {
    use crate::config::ConfigOptionBool;
    let mut config = default_or_parent_region_config.clone();
    if volume.is_model_part() {
        // default_or_parent_region_config contains the Print's PrintRegionConfig.
        // Override with ModelObject's PrintRegionConfig values.
        apply_to_print_region_config(&mut config, object.config.get(), variant_index);
    }
    // else: default_or_parent_region_config contains parent PrintRegion config, which already contains ModelVolume's config.
    let ignore_infill = volume_ignores_modifier_infill(volume);
    apply_to_print_region_config(&mut config, volume.config.get(), variant_index);
    if !volume.material_id().is_empty() {
        if let Some(material) = model.and_then(|m| volume.material(m)) {
            apply_to_print_region_config(&mut config, material.config.get(), variant_index);
        }
    }
    if let Some(layer_range_config) = layer_range_config {
        // Not applicable to modifiers.
        debug_assert!(volume.is_model_part());
        apply_to_print_region_config(&mut config, layer_range_config, variant_index);
    }
    if ignore_infill {
        restore_parent_internal_infill(&mut config, default_or_parent_region_config);
    }
    // Do not inherit this flag onto child modifiers; it only describes this volume.
    config.set_key_value("modifier_ignore_infill", Box::new(ConfigOptionBool::with_value(ignore_infill)));

    {
        //over write the seprate filament for features config
        let resolve_filament_value = |key: &str, default_or_parent: i32, previous_value: i32| -> i32 {
            let mut filament_value_temp = 0;
            let opt_vol = opt_config_int(volume.config.get(), key);
            let opt_obj = opt_config_int(object.config.get(), key);
            if let Some(v) = opt_vol {
                // deside use which value
                filament_value_temp = v;
            } else if let Some(v) = opt_obj {
                filament_value_temp = v;
            }
            let mut has_local_value = opt_vol.is_some() || opt_obj.is_some();

            if let Some(lrc) = layer_range_config {
                if volume.is_model_part() {
                    let opt = opt_config_int(lrc, key);
                    if let Some(v) = opt {
                        filament_value_temp = v;
                    }
                    has_local_value = has_local_value || opt.is_some();
                    // Filament-by-height selects the whole band, above the global feature filament.
                    if let Some(e) = opt_config_int(lrc, "extruder") {
                        if e > 0 {
                            has_local_value = true;
                        }
                    }
                }
            }

            if filament_value_temp > 0 {
                return filament_value_temp;
            }
            // Nothing set locally: a global feature filament overrides the part extruder.
            if !has_local_value && volume.is_model_part() && default_or_parent > 0 {
                return default_or_parent;
            }
            previous_value
        };
        let wall = resolve_filament_value(
            "wall_filament",
            default_or_parent_region_config.opt_int("wall_filament"),
            config.opt_int("wall_filament"),
        );
        *config.opt_int_mut("wall_filament") = wall;
        if !ignore_infill {
            let sparse = resolve_filament_value(
                "sparse_infill_filament",
                default_or_parent_region_config.opt_int("sparse_infill_filament"),
                config.opt_int("sparse_infill_filament"),
            );
            *config.opt_int_mut("sparse_infill_filament") = sparse;
            let solid = resolve_filament_value(
                "solid_infill_filament",
                default_or_parent_region_config.opt_int("solid_infill_filament"),
                config.opt_int("solid_infill_filament"),
            );
            *config.opt_int_mut("solid_infill_filament") = solid;
        }
    }

    // Clamp invalid extruders to the default extruder (with index 1).
    clamp_exturder_to_default_protect0(&mut config, "sparse_infill_filament", num_extruders);
    clamp_exturder_to_default_protect0(&mut config, "wall_filament", num_extruders);
    clamp_exturder_to_default_protect0(&mut config, "solid_infill_filament", num_extruders);
    {
        // `config.sparse_infill_density.value`: a ConfigOptionPercent.
        let density = *config.opt_float("sparse_infill_density");
        let value = if density < 0.00011f32 as f64 {
            // Switch of infill for very low infill rates, also avoid division by zero in infill generator for these very low rates.
            0.0
        } else {
            density.min(100.0)
        };
        if let Ok(Some(o)) = config.option_mut("sparse_infill_density", false) {
            if let Some(p) = o.as_any_mut().downcast_mut::<crate::config::ConfigOptionPercent>() {
                p.value = value;
            } else if let Some(f) = o.as_any_mut().downcast_mut::<crate::config::ConfigOptionFloat>() {
                f.value = value;
            }
        }
    }
    // Very small fuzzy_skin_point_distance value may cause infinite recursion, and a near-zero thickness doesn't generate anything anyway.
    let disabled_fuzzy = crate::print_config::enums::FuzzySkinType::Disabled_fuzzy as i32;
    let fuzzy_skin = config.option("fuzzy_skin").and_then(|o| o.get_int().ok()).unwrap_or(0);
    if fuzzy_skin != disabled_fuzzy
        && (*config.opt_float("fuzzy_skin_point_distance") < 0.01 || *config.opt_float("fuzzy_skin_thickness") < 0.001)
    {
        if let Ok(Some(o)) = config.option_mut("fuzzy_skin", false) {
            let _ = o.set_int(disabled_fuzzy);
        }
    }
    config
}

/// PrintConfig.hpp:977-982 `bool operator<(const CLASS_NAME &rhs) const`: the first member that
/// differs decides.
///
/// FIDELITY-NOTE: the `<` of each option class is not on the ConfigOption trait; the serialized
/// values are compared. It orders regions only between configs of the same hash.
fn static_config_lower(lhs: &DynamicPrintConfig, rhs: &DynamicPrintConfig) -> bool {
    for (_, key) in crate::print_config::config_def::static_print_config_members("PrintRegionConfig") {
        let (Some(a), Some(b)) = (lhs.option(key), rhs.option(key)) else {
            continue;
        };
        if a.operator_ne(b).unwrap_or(false) {
            return a.serialize().unwrap_or_default() < b.serialize().unwrap_or_default();
        }
    }
    false
}

/// PrintApply.cpp:973-1185 `static PrintObjectRegions* generate_print_object_regions(PrintObjectRegions
/// *print_object_regions_old, const ModelVolumePtrs &model_volumes, const LayerRanges
/// &model_layer_ranges, const PrintRegionConfig &default_region_config, const Transform3d &trafo,
/// size_t num_extruders, const float xy_contour_compensation, const std::vector<unsigned int>
/// &painting_extruders, std::vector<int> &variant_index, const bool has_painted_fuzzy_skin, const
/// std::vector<bool> &slot_per_part_enabled)`: either a fresh PrintObject, or PrintObject regions
/// were invalidated (merged, split). Generate PrintRegions from scratch.
///
/// `model_object` holds the `model_volumes` (its volumes, in their order).
pub fn generate_print_object_regions(
    print_object_regions_old: Option<Rc<RefCell<PrintObjectRegions>>>,
    model_object: &ModelObject,
    model: Option<&Model>,
    model_layer_ranges: &LayerRanges,
    default_region_config: &PrintRegionConfig,
    trafo: &Transform3d,
    num_extruders: usize,
    xy_contour_compensation: f32,
    painting_extruders: &[u32],
    variant_index: &mut Vec<i32>,
    has_painted_fuzzy_skin: bool,
    slot_per_part_enabled: &[bool],
) -> Rc<RefCell<PrintObjectRegions>> {
    // Reuse the old object or generate a new one.
    let reuse_old = print_object_regions_old.as_ref().map(|o| !o.borrow().layer_ranges.is_empty()).unwrap_or(false);
    let out_rc = print_object_regions_old.unwrap_or_else(|| Rc::new(RefCell::new(PrintObjectRegions::default())));
    let mut out_guard = out_rc.borrow_mut();
    let out: &mut PrintObjectRegions = &mut out_guard;

    out.all_regions.clear();

    let layer_range_config = |key: Option<(f64, f64)>| -> Option<&DynamicPrintConfig> {
        let (lo, hi) = key?;
        model_object.layer_config_ranges.get(&(ordered_float::OrderedFloat(lo), ordered_float::OrderedFloat(hi))).map(|c| c.get())
    };

    if reuse_old {
        // Reuse old bounding boxes of some ModelVolumes and their ranges.
        debug_assert!(model_layer_ranges.size() == out.layer_ranges.len());
        for (i, range) in model_layer_ranges.begin().enumerate() {
            let r = &mut out.layer_ranges[i];
            debug_assert!(range.layer_height_range == r.layer_height_range);
            // If model::assign_copy() is called, layer_ranges_regions is copied thus the pointers to configs are lost.
            r.config = range.config;
            r.volume_regions.clear();
            r.painted_regions.clear();
            r.fuzzy_skin_painted_regions.clear();
        }
    } else {
        out.trafo_bboxes = *trafo;
        out.layer_ranges.reserve(model_layer_ranges.size());
        for range in model_layer_ranges.begin() {
            out.layer_ranges.push(LayerRangeRegions { layer_height_range: range.layer_height_range, config: range.config, ..Default::default() });
        }
    }

    let is_mm_painted = num_extruders > 1 && model_object.volumes.iter().any(|mv| mv.is_mm_painted());
    let trafo_bboxes = out.trafo_bboxes;
    update_volume_bboxes(
        &mut out.layer_ranges,
        &mut out.cached_volume_ids,
        model_object,
        &trafo_bboxes,
        if is_mm_painted { 0.0 } else { 0.0f32.max(xy_contour_compensation) },
    );

    let mut region_set: Vec<PrintRegionPtr> = Vec::new();
    // Look up or create a PrintRegion.
    let mut get_create_region = |all_regions: &mut Vec<PrintRegionPtr>, config: PrintRegionConfig, volume_tag: ObjectID| -> PrintRegionPtr {
        let hash = crate::print::faithful::static_config_hash(&config, "PrintRegionConfig");
        let it = region_set.partition_point(|l| {
            let l = l.borrow();
            if l.config_hash() != hash {
                return l.config_hash() < hash;
            }
            if !crate::print::faithful::static_config_equal(l.config(), &config) {
                return static_config_lower(l.config(), &config);
            }
            l.gradient_volume_id() < volume_tag
        });
        if it != region_set.len() {
            let r = region_set[it].borrow();
            if r.config_hash() == hash && crate::print::faithful::static_config_equal(r.config(), &config) && r.gradient_volume_id() == volume_tag {
                drop(r);
                return region_set[it].clone();
            }
        }
        // Insert into a sorted array, it has O(n) complexity.
        let region = Rc::new(RefCell::new(PrintRegion::with_hash(config, hash, all_regions.len() as i32, volume_tag)));
        all_regions.push(region.clone());
        region_set.insert(it, region.clone());
        region
    };

    // Per-part gradient: count how many printable model-part volumes in this object would
    // be tagged for each per-part-enabled gradient slot.
    let mut per_part_volume_users: Vec<i32> = Vec::new();
    if !slot_per_part_enabled.is_empty() {
        per_part_volume_users = vec![0; slot_per_part_enabled.len()];
        for mv in &model_object.volumes {
            if !mv.is_model_part() {
                continue;
            }
            let range_cfg = if out.layer_ranges.is_empty() { None } else { layer_range_config(out.layer_ranges[0].config) };
            let vol_cfg = region_config_from_model_volume(default_region_config, range_cfg, mv, model_object, model, num_extruders, variant_index);
            for s_1based in [vol_cfg.opt_int("wall_filament"), vol_cfg.opt_int("sparse_infill_filament"), vol_cfg.opt_int("solid_infill_filament")] {
                let s = s_1based as u32;
                if s >= 1 && ((s - 1) as usize) < slot_per_part_enabled.len() && slot_per_part_enabled[(s - 1) as usize] {
                    per_part_volume_users[(s - 1) as usize] += 1;
                }
            }
        }
    }
    let compute_volume_tag = |cfg: &PrintRegionConfig, mv: &ModelVolume| -> ObjectID {
        if per_part_volume_users.is_empty() {
            return ObjectID::default_invalid();
        }
        let qualifies = |s_1based: u32| {
            s_1based >= 1
                && ((s_1based - 1) as usize) < slot_per_part_enabled.len()
                && slot_per_part_enabled[(s_1based - 1) as usize]
                && per_part_volume_users[(s_1based - 1) as usize] >= 2
        };
        if qualifies(cfg.opt_int("wall_filament") as u32)
            || qualifies(cfg.opt_int("sparse_infill_filament") as u32)
            || qualifies(cfg.opt_int("solid_infill_filament") as u32)
        {
            return mv.id();
        }
        ObjectID::default_invalid()
    };

    // Chain the regions in the order they are stored in the volumes list.
    for volume_id in 0..model_object.volumes.len() {
        let volume = &model_object.volumes[volume_id];
        if model_volume_solid_or_modifier(volume) {
            for ilr in 0..out.layer_ranges.len() {
                let Some(bbox) = find_volume_extents(&out.layer_ranges[ilr], volume) else {
                    continue;
                };
                if volume.is_model_part() {
                    // Add a model volume, assign an existing region or generate a new one.
                    let vol_cfg = region_config_from_model_volume(
                        default_region_config,
                        layer_range_config(out.layer_ranges[ilr].config),
                        volume,
                        model_object,
                        model,
                        num_extruders,
                        variant_index,
                    );
                    let volume_tag = compute_volume_tag(&vol_cfg, volume);
                    let region = get_create_region(&mut out.all_regions, vol_cfg, volume_tag);
                    out.layer_ranges[ilr].volume_regions.push(VolumeRegion {
                        model_volume: Some(volume_id),
                        parent: -1,
                        region: Some(region),
                        bbox: Some(bbox),
                        prev_same_region: None,
                    });
                } else if volume.is_negative_volume() {
                    // Add a negative (subtractor) volume. Such volume has neither region nor parent volume assigned.
                    out.layer_ranges[ilr].volume_regions.push(VolumeRegion {
                        model_volume: Some(volume_id),
                        parent: -1,
                        region: None,
                        bbox: Some(bbox),
                        prev_same_region: None,
                    });
                } else {
                    debug_assert!(volume.is_modifier());
                    // Modifiers may be chained one over the other. Check for overlap, merge DynamicPrintConfigs.
                    let mut added = false;
                    let mut parent_model_part_id: i32 = -1;
                    let bbox_box = out.layer_ranges[ilr].volumes[bbox].bbox;
                    let mut parent_region_id = out.layer_ranges[ilr].volume_regions.len() as i32 - 1;
                    while parent_region_id >= 0 {
                        let (parent_is_part, parent_is_modifier, parent_region) = {
                            let pr = &out.layer_ranges[ilr].volume_regions[parent_region_id as usize];
                            let pv = &model_object.volumes[pr.model_volume.unwrap()];
                            (pv.is_model_part(), pv.is_modifier(), pr.region.clone())
                        };
                        if parent_is_part || parent_is_modifier {
                            let parent_bbox = find_modifier_volume_extents(&out.layer_ranges[ilr], parent_region_id, model_object);
                            if parent_bbox.intersects(&bbox_box) {
                                let parent_config = parent_region.as_ref().unwrap().borrow().config().clone();
                                // Only create new region for a modifier, which actually modifies config of it's parent.
                                let config =
                                    region_config_from_model_volume(&parent_config, None, volume, model_object, model, num_extruders, variant_index);
                                if !crate::print::faithful::static_config_equal(&config, &parent_config) {
                                    added = true;
                                    let region = get_create_region(&mut out.all_regions, config, ObjectID::default_invalid());
                                    out.layer_ranges[ilr].volume_regions.push(VolumeRegion {
                                        model_volume: Some(volume_id),
                                        parent: parent_region_id,
                                        region: Some(region),
                                        bbox: Some(bbox),
                                        prev_same_region: None,
                                    });
                                } else if parent_model_part_id == -1 && parent_is_part {
                                    parent_model_part_id = parent_region_id;
                                }
                            }
                        }
                        parent_region_id -= 1;
                    }
                    if !added && parent_model_part_id >= 0 {
                        // This modifier does not override any printable volume's configuration, however it may in the future.
                        let region = out.layer_ranges[ilr].volume_regions[parent_model_part_id as usize].region.clone();
                        out.layer_ranges[ilr].volume_regions.push(VolumeRegion {
                            model_volume: Some(volume_id),
                            parent: parent_model_part_id,
                            region,
                            bbox: Some(bbox),
                            prev_same_region: None,
                        });
                    }
                }
            }
        }
    }

    // Finally add painting regions.
    for ilr in 0..out.layer_ranges.len() {
        for &painted_extruder_id in painting_extruders {
            for parent_region_id in 0..out.layer_ranges[ilr].volume_regions.len() {
                let (is_part_or_modifier, parent_region) = {
                    let pr = &out.layer_ranges[ilr].volume_regions[parent_region_id];
                    let pv = &model_object.volumes[pr.model_volume.unwrap()];
                    (pv.is_model_part() || pv.is_modifier(), pr.region.clone())
                };
                if is_part_or_modifier {
                    let mut cfg = parent_region.as_ref().unwrap().borrow().config().clone();
                    *cfg.opt_int_mut("wall_filament") = painted_extruder_id as i32;
                    *cfg.opt_int_mut("solid_infill_filament") = painted_extruder_id as i32;
                    *cfg.opt_int_mut("sparse_infill_filament") = painted_extruder_id as i32;
                    let region = get_create_region(&mut out.all_regions, cfg, ObjectID::default_invalid());
                    out.layer_ranges[ilr].painted_regions.push(PaintedRegion {
                        extruder_id: painted_extruder_id,
                        parent: parent_region_id as i32,
                        region: Some(region),
                    });
                }
            }
        }
        // Sort the regions by parent region::print_object_region_id() and extruder_id to help the slicing algorithm when applying MMU segmentation.
        let ids: Vec<i32> = out.layer_ranges[ilr]
            .volume_regions
            .iter()
            .map(|vr| vr.region.as_ref().map(|r| r.borrow().print_object_region_id()).unwrap_or(-1))
            .collect();
        {
            // std::sort of the regions; libc++'s algorithm over their indices gives the same order.
            let regions = std::mem::take(&mut out.layer_ranges[ilr].painted_regions);
            let mut order: Vec<usize> = (0..regions.len()).collect();
            crate::clipper::libcxx_sort(&mut order, |&li, &ri| {
                let (l, r) = (&regions[li], &regions[ri]);
                let lid = ids[l.parent as usize];
                let rid = ids[r.parent as usize];
                lid < rid || (lid == rid && l.extruder_id < r.extruder_id)
            });
            out.layer_ranges[ilr].painted_regions = order.into_iter().map(|i| regions[i].clone()).collect();
        }
    }

    if has_painted_fuzzy_skin {
        let disabled_fuzzy = crate::print_config::enums::FuzzySkinType::Disabled_fuzzy as i32;
        let all_fuzzy = crate::print_config::enums::FuzzySkinType::All as i32;
        let fuzzy_all = |cfg: &mut PrintRegionConfig| {
            let v = cfg.option("fuzzy_skin").and_then(|o| o.get_int().ok()).unwrap_or(0);
            if v != disabled_fuzzy {
                if let Ok(Some(o)) = cfg.option_mut("fuzzy_skin", false) {
                    let _ = o.set_int(all_fuzzy);
                }
            }
        };
        for ilr in 0..out.layer_ranges.len() {
            // FuzzySkinPaintedRegion can override different parts of the Layer than PaintedRegions,
            // so FuzzySkinPaintedRegion has to point to both VolumeRegion and PaintedRegion.
            for parent_volume_region_id in 0..out.layer_ranges[ilr].volume_regions.len() {
                let (is_part_or_modifier, parent_region) = {
                    let pr = &out.layer_ranges[ilr].volume_regions[parent_volume_region_id];
                    let pv = &model_object.volumes[pr.model_volume.unwrap()];
                    (pv.is_model_part() || pv.is_modifier(), pr.region.clone())
                };
                if is_part_or_modifier {
                    let mut cfg = parent_region.as_ref().unwrap().borrow().config().clone();
                    fuzzy_all(&mut cfg);
                    let region = get_create_region(&mut out.all_regions, cfg, ObjectID::default_invalid());
                    out.layer_ranges[ilr].fuzzy_skin_painted_regions.push(FuzzySkinPaintedRegion {
                        parent_type: FuzzySkinParentType::VolumeRegion,
                        parent: parent_volume_region_id as i32,
                        region: Some(region),
                    });
                }
            }

            for parent_painted_regions_id in 0..out.layer_ranges[ilr].painted_regions.len() {
                let parent_region = out.layer_ranges[ilr].painted_regions[parent_painted_regions_id].region.clone();
                let mut cfg = parent_region.as_ref().unwrap().borrow().config().clone();
                fuzzy_all(&mut cfg);
                let region = get_create_region(&mut out.all_regions, cfg, ObjectID::default_invalid());
                out.layer_ranges[ilr].fuzzy_skin_painted_regions.push(FuzzySkinPaintedRegion {
                    parent_type: FuzzySkinParentType::PaintedRegion,
                    parent: parent_painted_regions_id as i32,
                    region: Some(region),
                });
            }

            // Sort the regions by parent region::print_object_region_id() to help the slicing algorithm when applying fuzzy skin segmentation.
            let lr_snapshot = out.layer_ranges[ilr].clone();
            {
                let regions = std::mem::take(&mut out.layer_ranges[ilr].fuzzy_skin_painted_regions);
                let mut order: Vec<usize> = (0..regions.len()).collect();
                crate::clipper::libcxx_sort(&mut order, |&li, &ri| {
                    regions[li].parent_print_object_region_id(&lr_snapshot) < regions[ri].parent_print_object_region_id(&lr_snapshot)
                });
                out.layer_ranges[ilr].fuzzy_skin_painted_regions = order.into_iter().map(|i| regions[i].clone()).collect();
            }
        }
    }

    // Save the slot_per_part_enabled bit vector that produced these regions.
    out.last_slot_per_part_enabled = slot_per_part_enabled.to_vec();
    drop(out_guard);
    out_rc
}

/// PrintApply.cpp:1187-1193 `static bool same_ams_grouping(const std::vector<int> &old_map, const
/// std::vector<int> &new_map, int ams_idx)`: whether every filament of the two nozzle maps still
/// belongs to the same group.
pub fn same_ams_grouping(old_map: &[i32], new_map: &[i32], ams_idx: i32) -> bool {
    old_map.len() == new_map.len() && old_map.iter().zip(new_map).all(|(&o, &n)| (o == ams_idx) == (n == ams_idx))
}

/// PrintApply.cpp:1195-1240 `static std::vector<SupportMaterialProperties>
/// collect_model_filament_materials(const ModelObject &model_object, const DynamicPrintConfig
/// &full_config)`: the materials printed by this object.
pub fn collect_model_filament_materials(
    model_object: &ModelObject,
    full_config: &DynamicPrintConfig,
) -> Vec<crate::support_recommended_params::SupportMaterialProperties> {
    use crate::support_recommended_params::{support_recommended_material_name, SupportMaterialProperties};
    let mut out = Vec::new();
    let mut filament_ids: std::collections::BTreeSet<i32> = std::collections::BTreeSet::new();
    let strings = |key: &str| -> Option<Vec<String>> {
        full_config.option(key).and_then(|o| o.as_any().downcast_ref::<crate::config::ConfigOptionStrings>()).map(|o| o.values.clone())
    };
    let types = strings("filament_type");
    let names = strings("filament_settings_id");
    let global_features = crate::model::GlobalFeatureFilaments::from_full_config(full_config);
    let collect_explicit_filaments = |config: &DynamicPrintConfig, filament_ids: &mut std::collections::BTreeSet<i32>| {
        for key in ["extruder", "wall_filament", "sparse_infill_filament", "solid_infill_filament"] {
            if config.has(key) {
                let filament_id = config.option(key).and_then(|o| o.get_int().ok()).unwrap_or(0);
                if filament_id > 0 {
                    filament_ids.insert(filament_id);
                }
            }
        }
    };
    for mv in &model_object.volumes {
        if mv.is_model_part() {
            for filament_id in mv.get_extruders(model_object, &global_features) {
                if filament_id > 0 {
                    filament_ids.insert(filament_id);
                }
            }
        } else if mv.is_modifier() {
            collect_explicit_filaments(mv.config.get(), &mut filament_ids);
        }
    }
    for (_, layer_range) in &model_object.layer_config_ranges {
        collect_explicit_filaments(layer_range.get(), &mut filament_ids);
    }
    out.reserve(filament_ids.len());
    for filament_id in filament_ids {
        let idx = (filament_id - 1) as usize;
        let mut material = SupportMaterialProperties::default();
        if let Some(t) = &types {
            if idx < t.len() {
                material.r#type = t[idx].clone();
            }
        }
        if let Some(n) = &names {
            if idx < n.len() {
                material.name = support_recommended_material_name(n[idx].clone());
            }
        }
        // Keep unknown materials: they must fail name/type rules instead of being omitted from "all materials".
        out.push(material);
    }
    out
}

/// PrintApply.cpp:1242-1246 `struct MultiNozzleSupportCombo`
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct MultiNozzleSupportCombo {
    pub zero_top: bool,
    pub zero_bottom: bool,
}

/// PrintApply.cpp:1248-1294 `static MultiNozzleSupportCombo multi_nozzle_support_combo(const
/// DynamicPrintConfig &full_config, const PrintObjectConfig &object_config, const ModelObject
/// &model_object)`
pub fn multi_nozzle_support_combo(full_config: &DynamicPrintConfig, object_config: &DynamicPrintConfig, model_object: &ModelObject) -> MultiNozzleSupportCombo {
    use crate::support_recommended_params::{evaluate_support_material_combination, support_recommended_material_name, SupportMaterialProperties};
    let mut combo = MultiNozzleSupportCombo::default();
    let interface = object_config.opt_int("support_interface_filament");
    // Without top interface layers there is no support contact surface to classify.
    if object_config.opt_int("support_interface_top_layers") <= 0 || interface <= 0 {
        return combo;
    }

    let model_materials = collect_model_filament_materials(model_object, full_config);
    if model_materials.is_empty() {
        return combo;
    }

    let strings = |key: &str| -> Option<Vec<String>> {
        full_config.option(key).and_then(|o| o.as_any().downcast_ref::<crate::config::ConfigOptionStrings>()).map(|o| o.values.clone())
    };
    let bools = |key: &str| -> Option<Vec<u8>> {
        full_config.option(key).and_then(|o| o.as_any().downcast_ref::<crate::config::ConfigOptionBools>()).map(|o| o.values.clone())
    };
    let types = strings("filament_type");
    let names = strings("filament_settings_id");
    let soluble = bools("filament_soluble");
    let support = bools("filament_is_support");
    let string_at = |opt: &Option<Vec<String>>, id: i32| -> String {
        match opt {
            Some(v) if id > 0 && (id as usize) <= v.len() => v[(id - 1) as usize].clone(),
            _ => String::new(),
        }
    };
    let bool_at = |opt: &Option<Vec<u8>>, id: i32| -> bool {
        match opt {
            Some(v) if id > 0 && (id as usize) <= v.len() => v[(id - 1) as usize] != 0,
            _ => false,
        }
    };
    let interface_material = SupportMaterialProperties {
        name: support_recommended_material_name(string_at(&names, interface)),
        r#type: string_at(&types, interface),
        soluble: bool_at(&soluble, interface),
        support: bool_at(&support, interface),
    };

    let printer_model = full_config
        .option("printer_model")
        .and_then(|o| o.as_any().downcast_ref::<crate::config::ConfigOptionString>())
        .map(|o| o.value.clone())
        .unwrap_or_default();

    combo.zero_top = true;
    combo.zero_bottom = true;
    for material in &model_materials {
        let material_result = evaluate_support_material_combination(&interface_material, material, &printer_model);
        combo.zero_top &= material_result.zero_top;
        combo.zero_bottom &= material_result.zero_bottom;
        if !combo.zero_top && !combo.zero_bottom {
            break;
        }
    }
    combo
}


impl crate::print::faithful::Print {
    /// PrintApply.cpp:2275-2292, the end of `Print::apply` when the regions were reshuffled: update
    /// `Print::m_print_regions` from the objects, a region of the same config (hash and config)
    /// taking the id of the first one.
    pub fn update_print_regions_from_objects(&mut self) {
        // std::set<const PrintRegion*, cmp> region_set
        let region_set: Vec<PrintRegionPtr> = Vec::new();
        self.m_print_regions.clear();
        let mut print_object_regions: Option<Rc<RefCell<PrintObjectRegions>>> = None;
        for oi in 0..self.m_objects.len() {
            let shared = self.m_objects[oi].m_shared_regions.clone();
            let same = match (&print_object_regions, &shared) {
                (Some(a), Some(b)) => Rc::ptr_eq(a, b),
                (None, None) => true,
                _ => false,
            };
            if !same {
                print_object_regions = shared.clone();
                let Some(por) = &print_object_regions else { continue };
                let all_regions = por.borrow().all_regions.clone();
                for print_region in all_regions {
                    // The C++ never inserts into region_set, so the lookup finds nothing and every
                    // region gets an id of its own.
                    let found = region_set.iter().find(|r| *r.borrow() == *print_region.borrow()).cloned();
                    match found {
                        None => {
                            let print_region_id = self.m_print_regions.len() as i32;
                            self.m_print_regions.push(print_region.clone());
                            print_region.borrow_mut().m_print_region_id = print_region_id;
                        }
                        Some(it) => {
                            let id = it.borrow().print_region_id();
                            print_region.borrow_mut().m_print_region_id = id;
                        }
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::geometry::Transform3d;

    #[test]
    fn transform3d_equal_identity() {
        let a = Transform3d::identity();
        let b = Transform3d::identity();
        assert!(transform3d_equal(&a, &b));
        assert!(!transform3d_lower(&a, &b));
    }

    #[test]
    fn transform3d_lower_orders_by_first_differing_scalar() {
        // Column-major: index 12 is the X translation (m[(0,3)]).
        let a = Transform3d::identity();
        let mut b = Transform3d::identity();
        b[(0, 3)] = 5.0;
        // a's scalar at column-major index 12 (0.0) < b's (5.0) -> a < b.
        assert!(transform3d_lower(&a, &b));
        assert!(!transform3d_lower(&b, &a));
        assert!(!transform3d_equal(&a, &b));
    }

    #[test]
    fn trafos_differ_z_rotation_mirror_only() {
        // Identical trafos differ only trivially -> true.
        let a = Transform3d::identity();
        let b = Transform3d::identity();
        assert!(trafos_differ_in_rotation_by_z_and_mirroring_by_xy_only(&a, &b));

        // A pure rotation about Z keeps Z fixed and X,Y orthonormal -> true.
        let theta = 0.7_f64;
        let mut rz = Transform3d::identity();
        rz[(0, 0)] = theta.cos();
        rz[(0, 1)] = -theta.sin();
        rz[(1, 0)] = theta.sin();
        rz[(1, 1)] = theta.cos();
        assert!(trafos_differ_in_rotation_by_z_and_mirroring_by_xy_only(&a, &rz));

        // Different Z translation -> false.
        let mut tz = Transform3d::identity();
        tz[(2, 3)] = 5.0;
        assert!(!trafos_differ_in_rotation_by_z_and_mirroring_by_xy_only(&a, &tz));

        // Z scaling changes Z length -> false.
        let mut sz = Transform3d::identity();
        sz[(2, 2)] = 2.0;
        assert!(!trafos_differ_in_rotation_by_z_and_mirroring_by_xy_only(&a, &sz));
    }

    #[test]
    fn custom_gcode_tool_changes_differ_ignores_non_toolchange() {
        let tc = |z: f64, e: i32| CustomGCodeItem::new(z, CustomGCodeType::ToolChange, e, String::new());
        let color = CustomGCodeItem::new(1.0, CustomGCodeType::ColorChange, 1, String::new());

        // Identical tool-change sequences (with a non-toolchange interleaved) -> no diff.
        let a = vec![tc(1.0, 2), color.clone(), tc(2.0, 3)];
        let b = vec![color.clone(), tc(1.0, 2), tc(2.0, 3)];
        assert!(!custom_per_printz_gcodes_tool_changes_differ(&a, &b));

        // One side has an extra tool change -> differ.
        let c = vec![tc(1.0, 2)];
        let d = vec![tc(1.0, 2), tc(2.0, 3)];
        assert!(custom_per_printz_gcodes_tool_changes_differ(&c, &d));

        // Tool changes differ in extruder -> differ.
        let e = vec![tc(1.0, 2)];
        let f = vec![tc(1.0, 4)];
        assert!(custom_per_printz_gcodes_tool_changes_differ(&e, &f));

        // Only non-toolchange items, both sides empty of tool changes -> no diff.
        let g = vec![color.clone()];
        let h = vec![color.clone(), color.clone()];
        assert!(!custom_per_printz_gcodes_tool_changes_differ(&g, &h));
    }
}
