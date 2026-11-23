# libslic3r C++ → Rust port — handoff

**Goal:** faithful, line-by-line 1:1 port of C++ `libslic3r` (BambuStudio) to Rust in
`crates/libslic3r-rs`, so the Rust engine produces G-code **byte-identical** to the C++
slicer. No shortcuts: same functions, same order, same names (snake_case), same control
flow / constants / rounding / locations. wasm-safe where possible (the four `*-sys` shims
are the exception). How to build, run and score it in this repository:
`docs/rust-engine/README.md`.

## Where things stand

- **Parity (R934), against this repository's `slicer_cli` (Bambu Studio 02.09.01.70):**
  Benchy classic 187,447 lines in order (98.19%), Benchy arachne 240,669 (98.90%), Majora
  2,840,386 (98.97%), prime tower 99.6%. `docs/rust-engine/README.md` has what the numbers
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

As of R934 every engine test passes: `cargo test -p slicer --lib --tests --no-fail-fast -- --skip
test_tree_support_many_layers` gives 2,570 passed, 0 failed (debug profile), and `cargo test -p
slicer-cli-rust` 9 passed. R913 settled the 67 that failed in R912 one by one against the pin (a
standalone generator calling the pin's function): most expected values of an older, invented API
and were corrected to the C++ result with a comment saying where it comes from; the rest were
engine code that departed from the C++ and was fixed (behind faithful gates where the slicing
path reaches it: `FLOW_FMA`, `RAY_CIRCLE_FMA`, `AABB_LINES_PARTITION`,
`AABB_LINES_CENTROID_TRUNC`, `PRINT_OBJECT_STARTED_CANCEL`, `THIN_FILLS_CLONE`,
`TREE_MODEL_VOLUMES_OFFSET`).

- **Skipped:** `tree_support_integration::test_tree_support_many_layers` (slow; hangs with debug
  assertions off). The former hang `internal_bridge_detector::tests::test_detect_angle_no_anchors`
  was a millimetre-vs-scaled delta and runs since R910.
- **Fixture-dependent:** `benchy_integration` reads the documented, not redistributed
  `tests/rust-engine/fixtures/3DBenchy.stl` and says `SKIPPED` when it is missing;
  `cube_integration` writes its own 2.792 mm cube; `arachne_infill` reads the committed
  `tests/rust-engine/fixtures/Cube_25.6.stl`.
- **Disabled with `#[cfg(any())]` (their subject no longer exists):** 30 in `src/surface.rs`, and
  some in `tests/tree_support_integration.rs` and `tests/layer_methods_test.rs` (deleted
  `PrintPipeline`/`profiles` modules, corrupted bodies).
- **Other:** three doc tests do not compile (`gcode/gcode_template.rs`, two in
  `enum_bitmask.rs`), so a plain `cargo test` that includes doc tests ends non-zero.

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
