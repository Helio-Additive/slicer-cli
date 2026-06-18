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
