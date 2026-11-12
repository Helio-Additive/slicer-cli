# libslic3r C++ → Rust port — handoff

**Goal:** faithful, line-by-line 1:1 port of C++ `libslic3r` (BambuStudio) to Rust in
`crates/libslic3r-rs`, so the Rust engine produces G-code **byte-identical** to the C++
slicer. No shortcuts: same functions, same order, same names (snake_case), same control
flow / constants / rounding / locations. wasm-safe where possible (the four `*-sys` shims
are the exception). How to build, run and score it in this repository:
`docs/rust-engine/README.md`.

## Where things stand

- **Parity (R854), against this repository's `slicer_cli` (Bambu Studio 02.09.01.70):**
  Benchy classic 183,740 lines in order (95.89%), Benchy arachne 236,625 (97.13%), Majora
  2,747,545 (96.17%), prime tower 99.5%. `docs/rust-engine/README.md` has what the numbers
  rest on and where each fixture stands; `docs/rust-engine/parity/rounds-R809-on.md` is the
  round log from R809, `docs/rust-engine/PARITY_STATUS.md` (to R471) and
  `docs/rust-engine/parity/rounds-R472-R808.md` the rounds against the 02.06.00.51 engine.
- **What is ported:** `docs/rust-engine/FILE_MAP.md`, regenerated every round by
  `python3 scripts/parity/file_map.py`. It replaces the hand-kept ledger counts. `PORT_LEDGER.json` remains
  the place to mark a unit `partial` with what blocks it. BLOCKED markers remain in about 75 files; re-check
  each against the current code before believing it.
- lib and bins build; the test targets compile (see "Test status").

## Source of truth

- `PORT_LEDGER.json` — array of units `{cpp,hpp,rust,area,loc,status}`,
  `status ∈ {pending,partial,deferred,done}`. `PORT_LEDGER.md` is the dashboard made from it.
- `PROGRESS.md`, `PARITY.md` — history of the systematic port (to June 2026).
- `docs/rust-engine/PARITY_STATUS.md` and `docs/rust-engine/parity/` — the parity rounds.

## Operating loop

1. Build: `cargo build --workspace` from the repository root — lib and bins must exit 0.
2. When a config struct grows (for example `PrintRegionConfig`), update the struct literals in
   `src/bin/slicer-cli.rs` (`create_default_region_config`).
3. Parity check, which must not regress: run with debug assertions off, which is what the C++
   release build does with `assert()`. The workspace's dev profile already does this for the
   `slicer` package (tests keep them on). Then follow "Parity workflow" in
   `docs/rust-engine/README.md` (`slicer-parity compare`, then `scripts/parity/seq_parity.py`)
   on Benchy classic, Benchy arachne and Majora.
4. Commit as `R<round>: <what>` with the scores before and after.

## Hard rules / gotchas

- Build with the compiler `rust-toolchain.toml` names, and with the Eigen and Boost headers
  `docs/rust-engine/README.md` names when a score is compared with a recorded one: the
  shims' output depends on them.
- coord_t→i64, coordf_t→f64. Reuse existing crate primitives (grep before adding). No
  stubs or fakes — mark the unit `partial` and say what blocks it.
- Read `docs/rust-engine/PORTING_PATTERNS.md` before touching geometry.

## Test status

`cargo test -p slicer --no-run --all-targets` exits 0. Running the tests does not: the list below is what fails, as of the commit that made
them compile again. None of these was edited to pass.

Totals, with debug assertions on (the workspace's test profile): lib 1843 passed / 45 failed / 1 hangs of 1889; integration tests
136 passed / 41 failed / 4 ignored over 25 targets. With `--release`
(debug assertions off) `cut_surface::tests::its_mask_size_mismatch_returns_empty` and nine of
the ten `tree_support_integration` failures pass.

**Disabled with `#[cfg(any())]` (their subject no longer exists):** 30 in `src/surface.rs`
(the former free surface functions are private `PrintObject`/`LayerRegion` methods now); 6 in
`tests/benchy_integration.rs`, 8 in `tests/cube_integration.rs` and 3 in
`tests/tree_support_integration.rs` (the deleted `PrintPipeline` and `profiles` modules); 2 in
`tests/tree_support_integration.rs` (`TreeSupport3D::generate_with_mesh`,
`generate_with_organic_smoothing`); 3 in `tests/layer_methods_test.rs` (bodies were corrupted).

**Need a fixture that is not in the repository (18):** 14 in `benchy_integration` and 4 in
`cube_integration` read `data/test_stls/` and `data/reference_gcodes/` under this crate.

**Hang:** `internal_bridge_detector::tests::test_detect_angle_no_anchors` (every profile);
`tree_support_integration::test_tree_support_many_layers` (debug assertions off).

**Fail in the lib (44 in every profile):**
- Stale expectations of the 1e6 scaling (the engine is 1e5 units/mm): `tests::test_scaling`,
  `flow::tests::test_scaled_values`; probably `arachne::wall_tool_paths::tests::test_meshfix_constants`.
- Stale number format (`X10.000` expected, the writer trims): `gcode::tests::{test_linear_move,
  test_rapid_move}`, `gcode::writer::tests::{test_writer_extrude_relative_e,
  test_writer_extrude_absolute_e, test_writer_travel}`.
- Panics in engine code on degenerate input: `arc_fitter::tests::test_arc_fitting_empty`
  (`points.len() - 1` on an empty list), `fill::fill3_d_honeycomb::tests::test_fill_surface_single_runs`
  (division by zero at spacing 0), `fill::fill_plane_path::tests::{test_fill_surface_single_hilbert,
  test_fill_surface_single_archimedean_solid}` (`assert!(queue.is_empty())` in `shortest_path.rs`),
  `support::tree_model_volumes::tests::test_is_safe_position` (coordinates outside the i32 range).
- Assertion mismatches: `any_ptr::tests::test_get_shared_cpy`;
  `arachne::skeletal_trapezoidation_edge::tests::test_transition_ends`;
  `arachne::utils::extrusion_line::tests::test_area_calculation`;
  `arachne::utils::linear_alg2d::tests::{test_get_angle_left_pointy, test_get_angle_left_full_circle,
  test_is_inside_corner_acute, test_is_inside_corner_obtuse, test_is_inside_corner_simple}`;
  `arachne::utils::polyline_stitcher::tests::test_polyline_stitcher_two_lines`;
  `arc_fitter::tests::test_arc_fitting_quarter_circle`;
  `csg_mesh::model_to_csg_mesh::tests::test_model_to_csgmesh_do_splits`;
  `ex_polygon_serialize::tests::serialize_polygon_matches_derive`;
  `flow::tests::{test_parity_bridge, test_parity_typical_perimeter}`;
  `format::bbs_3mf::tests::test_zip_unicode_path_extra_field_roundtrip`;
  `gcode::cooling::tests::{test_multi_extruder_slowdown, test_slowdown_calculation_slowdown_needed}`;
  `gcode::exporter::tests::{test_extrude_path_cumulative_e, test_extrude_path_multi_segment,
  test_extrude_path_single_segment, test_feature_comment_on_role_change, test_set_extruder_with_retraction}`;
  `geometry::circle::tests::test_ray_circle_intersections`;
  `intersection_points::tests::test_simple_intersection`; `nsvg_utils::tests::to_coor_rounds`;
  `png_read_write::tests::{roundtrip_gray, roundtrip_rgb}`;
  `sla::hollowing::tests::{drain_hole_get_intersections_wall_ray, drain_hole_to_mesh_rotated_and_translated}`;
  `triangulation::tests::test_has_self_intersection`; `utils::tests::test_split_string`.

**Fail in the integration tests (23, default profile):**
- `change_filament_context`: `small_purge_uses_single_segment`,
  `flush_length_split_matches_cpp_formula` (4 decimals emitted, 6 expected).
- `layer_methods_test::test_make_fills_adds_thin_fills`.
- `multi_material_integration::test_wipe_tower_bounds_to_polygon`.
- `print_process_test`: `test_print_cancellation`, `test_status_callback`,
  `test_printobject_cancellation`, `test_print_object_state_tracking`, `test_state_persistence`.
- `support_necessity_test::test_cancellation_during_support_generation`.
- `wall_gap_geometry`: `add_extra_point_targets_the_bottom_edge` (`gaps_actually_remove_length` and
  `two_skip_points_open_two_gaps` pass since R838, which scales the tower's points in float).
- `tree_support_integration` (10): all stop on the `debug_assert` in `clipper_utils.rs` that
  guards the i32 ClipperLib path. Tree-support collision geometry reaches scaled coordinates
  near 8e9, which the assertion catches and a release build truncates silently. This looks like
  an engine bug, so do not turn debug assertions off for tests to hide it.

**Other:** three doc tests do not compile (`gcode/gcode_template.rs`, two in `enum_bitmask.rs`),
so a plain `cargo test` ends non-zero. `clipper2-z-sys`'s `version_links` expects Clipper2 1.5.4
and the vendored source reports 1.5.2. Cancellation is not honoured by `Print::process`,
`PrintObject::ironing` and `generate_support_material`.

## Current track (config threading wired 2026-06-12)
1. **Config-hierarchy threading** (DONE — steps 1–8 committed 2026-06-12) — Arc-distributed
   config snapshots wired through Print→PrintObject→Layer→LayerRegion. Canonical mapping:
   C++ `this->layer()->object()->print()->config()` == Rust `layer.object().print().config()`;
   `layerm->region().config()` == `layer_region.region().config()` (view structs
   `PrintRef<'_>`/`ObjectRef<'_>` in `print_object.rs`). `region_configs` param threading and
   the `flow_with_config` shim are gone crate-wide (grep-verified). INVARIANT: replace config
   Arcs wholesale at sync points (`Print::add_object`, `Print::process`,
   `wire_config_hierarchy`/`wire_layer_hierarchy`); never `Arc::make_mut`/`get_mut`.
   Full record: PARITY.md "Config-hierarchy threading — WIRED" + PARITY.json `configThreading`.
   Gate held: Benchy byte-identical, filament 3818.67 mm.
2. **Retry worklist (NEXT)** — re-attempt the 110 `partial` units (now unblocked on threading),
   Benchy-path-first, → `done`. Note: some `partial`s remain blocked on non-threading deps
   (Z-clipper, octree, missing `PrintRegionConfig` seam_slope_* fields) — see PARITY.md
   remaining-items list.
3. `top_fills` coverage fix (gated behind env `TOP_FILLS`) + faithful `discover_vertical_shells` (gated `VSHELL_FAITHFUL`) — see `PROGRESS.md`.
4. Native-lib-blocked symbols (OpenVDB/CGAL/OCCT/boost-Voronoi) — vendored minimal Rust replacements, only the functions actually used.
