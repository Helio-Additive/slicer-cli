//! Tests for the flush-into-object eligibility predicates
//! (`WipingExtrusions::is_*_overriddable`, C++ ToolOrdering.cpp:2645-2690).
//! These decide whether an object extrusion may absorb part of a tool change's
//! purge instead of the wipe tower.

use slicer::extrusion_entity::ExtrusionRole as R;
use slicer::gcode::tool_ordering::{is_obj_overriddable, is_overriddable, is_support_overriddable};

#[test]
fn soluble_filament_is_never_overriddable() {
    // cpp:2647 — soluble filament cannot be wiped into the object at all,
    // even with flush_into_objects on.
    assert!(!is_overriddable(R::InternalInfill, true, true, true));
    assert!(!is_overriddable(R::Perimeter, true, true, true));
}

#[test]
fn flush_into_objects_allows_any_role() {
    // cpp:2650 — with flush_into_objects, every (non-soluble) role qualifies.
    for role in [R::InternalInfill, R::Perimeter, R::ExternalPerimeter, R::SolidInfill] {
        assert!(is_overriddable(role, false, true, false), "{role:?}");
    }
}

#[test]
fn flush_into_infill_allows_only_internal_infill() {
    // cpp:2653 — without flush_into_objects, ONLY erInternalInfill qualifies.
    assert!(is_overriddable(R::InternalInfill, false, false, true));
    for role in [R::Perimeter, R::ExternalPerimeter, R::SolidInfill, R::TopSolidInfill] {
        assert!(!is_overriddable(role, false, false, true), "{role:?}");
    }
}

#[test]
fn nothing_overriddable_when_all_flush_flags_off() {
    for role in [R::InternalInfill, R::Perimeter, R::SolidInfill] {
        assert!(!is_overriddable(role, false, false, false), "{role:?}");
    }
}

#[test]
fn obj_overriddable_matches_cpp() {
    // cpp:2658-2668
    assert!(is_obj_overriddable(R::Perimeter, true, false));
    assert!(is_obj_overriddable(R::InternalInfill, false, true));
    assert!(!is_obj_overriddable(R::Perimeter, false, true));
    assert!(!is_obj_overriddable(R::InternalInfill, false, false));
}

#[test]
fn support_overriddable_matches_cpp() {
    // cpp:2670-2688 — requires flush_into_support, and the relevant filament
    // setting must be 0 ("use current filament").
    assert!(!is_support_overriddable(R::SupportMaterial, false, 0, 0));
    assert!(is_support_overriddable(R::SupportMaterial, true, 0, 1));
    assert!(!is_support_overriddable(R::SupportMaterial, true, 2, 0));
    // SupportTransition is grouped with SupportMaterial (cpp:2679).
    assert!(is_support_overriddable(R::SupportTransition, true, 0, 1));
    // Interface keys off support_interface_filament.
    assert!(is_support_overriddable(R::SupportMaterialInterface, true, 1, 0));
    assert!(!is_support_overriddable(R::SupportMaterialInterface, true, 0, 2));
    // Mixed qualifies if EITHER is 0.
    assert!(is_support_overriddable(R::Mixed, true, 0, 5));
    assert!(is_support_overriddable(R::Mixed, true, 5, 0));
    assert!(!is_support_overriddable(R::Mixed, true, 5, 5));
    // Unrelated roles never qualify.
    assert!(!is_support_overriddable(R::InternalInfill, true, 0, 0));
}
