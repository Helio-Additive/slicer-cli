//! Faithful 1:1 port of `ByObjectPrintData.cpp` / `ByObjectPrintData.hpp`.
//!
//! C++ source:
//!   - src/libslic3r/ByObjectPrintData.cpp
//!   - src/libslic3r/ByObjectPrintData.hpp
//!
//! This is the per-object (sequential / "by object") print-data builder. It is a
//! thin orchestrator that, before tool ordering is built, generates the filament
//! map: it computes the print-instance order, the per-object tool ordering, and
//! the layered nozzle-group result, threading everything back into `Print`.
//!
//! PORT STATUS: partial.
//!
//! The struct skeleton and `clear()` are ported faithfully (see below). The two
//! substantive functions `build()` and `collect_filament_data()` are BLOCKED on a
//! body of not-yet-ported sequential-print scheduling and PrintObject-driven
//! tool-ordering machinery. They are intentionally NOT stubbed (no fake logic).
//!
//! AUDIT 2026-06-13: several dependencies originally listed as blockers now exist
//! and are no longer the reason for blocking:
//!   - `has_any_mixed_filament` / `expand_mixed_filaments` — ported in
//!     `filament_mixer.rs`.
//!   - `collect_sorted_used_filaments` — ported in `filament_group.rs`.
//!   - `MultiNozzleUtils::LayeredNozzleGroupResult` with `create`,
//!     `get_layer_filament_nozzle_maps`, `get_used_filaments`,
//!     `is_support_dynamic_nozzle_map`, `get_extruder_map`, `get_volume_map`,
//!     `get_nozzle_map` — ported in `multi_nozzle_utils.rs` (note: the
//!     map-getters take a `layer_id` arg in Rust, vs. the whole-result form
//!     `get_extruder_map(false)` / `get_volume_map()` / `get_nozzle_map()` that
//!     `build()` calls — a signature mismatch that must be reconciled when the
//!     blocking deps below land).
//!   - `FilamentGroupUtils::update_used_filament_values` — ported in
//!     `filament_group_utils.rs`.
//! See the `build` / `collect_filament_data` documentation blocks for the
//! remaining, genuinely-missing blocking dependencies.

use std::collections::HashMap;

use crate::gcode::tool_ordering::ToolOrdering;
use crate::print::PrintObject;

// ByObjectPrintData.hpp:8 — `struct PrintInstance;` (forward declaration).
//
// The C++ `PrintInstance` (declared in Print.hpp) carries a `print_object`
// back-pointer and is what `sort_object_instances_by_model_order` returns. The
// Rust crate does not yet have a `PrintInstance` with this shape (the only
// `PrintInstance` present, in `shortest_path.rs`, is an unrelated geometry type),
// so we forward-declare an opaque placeholder here purely to type the
// `print_instance_order` field. It is never constructed by ported code.
//
// BLOCKED: the real `PrintInstance` requires the sequential-print scheduling
// subsystem from Print.hpp/Print.cpp to be ported first.
pub enum PrintInstance {}

/// `ByObjectPrintData` — ByObjectPrintData.hpp:11
pub struct ByObjectPrintData {
    /// print instance的打印顺序
    /// (print order of the print instances) — ByObjectPrintData.hpp:13
    pub print_instance_order: Vec<*const PrintInstance>,
    /// 每个instance对应的tool ordering
    /// (the tool ordering corresponding to each instance) — ByObjectPrintData.hpp:15
    ///
    /// C++: `std::unordered_map<const PrintObject*, ToolOrdering>`. The raw-pointer
    /// key mirrors the C++ pointer-identity keying exactly.
    pub object_tool_ordering_map: HashMap<*const PrintObject, ToolOrdering>,
    /// object的打印顺序
    /// (print order of the objects) — ByObjectPrintData.hpp:17
    pub print_object_order: Vec<*const PrintObject>,
}

impl ByObjectPrintData {
    // ByObjectPrintData.cpp:144
    pub fn clear(&mut self) {
        // ByObjectPrintData.cpp:146
        self.print_instance_order.clear();
        // ByObjectPrintData.cpp:147
        self.object_tool_ordering_map.clear();
        // NOTE: C++ `clear()` deliberately does NOT clear `print_object_order`.
    }

    // ByObjectPrintData.cpp:12 — collect_filament_data
    //
    // BLOCKED: faithful port not possible yet. Requires the following genuinely
    // not-yet-ported dependencies (verified absent 2026-06-13):
    //   - `ToolOrdering::new(*const PrintObject, /*last_filament_id*/ u32::MAX)`:
    //     the C++ `ToolOrdering(const PrintObject&, unsigned int)` constructor that
    //     builds `layer_tools` from a PrintObject. The Rust `ToolOrdering` is a
    //     fundamentally different, config-driven design (`new(ToolOrderingConfig)`)
    //     with no PrintObject-driven build path; `tool_ordering.rs` contains zero
    //     references to `PrintObject`.
    //   - `Print::config().filament_is_mixed` / `filament_mixed_components`: not
    //     present as `PrintConfig` fields (only as preset key strings in
    //     `preset_bundle.rs`), so `print->config().filament_is_mixed.values` is
    //     not yet readable.
    // The mixer helpers `has_any_mixed_filament(...)` / `expand_mixed_filaments(...)`
    // (filament_mixer.rs) ARE now available and would be reused once the two
    // blockers above land.
    // See ByObjectPrintData.cpp:12-35.

    // ByObjectPrintData.cpp:37 — build
    //
    // BLOCKED: faithful port not possible yet. In addition to the
    // `collect_filament_data` dependencies above, `build()` requires the following
    // genuinely not-yet-ported symbols (each verified to have ZERO definitions in
    // the crate, 2026-06-13):
    //   - `Print::is_sequential_print()` — not ported.
    //   - `sort_object_instances_by_model_order(const Print&)` — not ported.
    //   - `PrintInstance::print_object` back-pointer — `PrintInstance` not ported
    //     with this shape (the only `PrintInstance` is the unrelated geometry type
    //     in `shortest_path.rs`).
    //   - `Print::get_physical_unprintable_filaments(...)`,
    //     `Print::get_geometric_unprintable_filaments()`,
    //     `Print::get_filament_unprintable_flow(...)` — not ported.
    //   - `Print::is_dynamic_group_reorder()`, `Print::get_filament_map_mode()` —
    //     not ported.
    //   - `ToolOrdering::get_recommended_filament_maps(...)` — not ported.
    //   - `Print::set_nozzle_group_result(...)`,
    //     `Print::get_layered_nozzle_group_result()`,
    //     `Print::update_filament_maps_to_config(...)`,
    //     `Print::update_to_config_by_nozzle_group_result(...)` — not ported.
    //   - `GroupReorder::build_filament_group_context(...)` — not ported.
    //   - `ToolOrdering(const PrintObject&, last_filament_id)` constructor,
    //     `ToolOrdering::sort_and_build_data(*const PrintObject, last_filament_id)`,
    //     `ToolOrdering::last_extruder()` (PrintObject-driven),
    //     `ToolOrdering::get_layered_nozzle_group_result()` — not ported (the Rust
    //     `ToolOrdering` is config-driven and has no PrintObject build path).
    //
    // AVAILABLE now (would be reused once the above land, but do not unblock alone):
    //   - `collect_sorted_used_filaments(...)` (filament_group.rs).
    //   - `MultiNozzleUtils::LayeredNozzleGroupResult` with `create`,
    //     `get_layer_filament_nozzle_maps`, `get_used_filaments`,
    //     `is_support_dynamic_nozzle_map` (multi_nozzle_utils.rs). NOTE: the
    //     `get_extruder_map` / `get_volume_map` / `get_nozzle_map` getters take a
    //     `layer_id` argument in Rust, whereas `build()` calls the whole-result
    //     forms `get_extruder_map(false)` / `get_volume_map()` / `get_nozzle_map()`
    //     — that signature gap must also be reconciled.
    //   - `FilamentGroupUtils::update_used_filament_values(...)`
    //     (filament_group_utils.rs).
    // See ByObjectPrintData.cpp:37-140.
}
