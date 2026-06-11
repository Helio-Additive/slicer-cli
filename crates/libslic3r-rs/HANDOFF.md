# libslic3r C++ → Rust port — session handoff

**Goal:** faithful, line-by-line 1:1 port of C++ `libslic3r` (BambuStudio) to Rust in
`crates/libslic3r-rs`, so the Rust engine produces **byte-identical 3DBenchy G-code** to the
C++ slicer. No shortcuts: same functions, same order, same names (snake_case), same control
flow / constants / rounding / locations. wasm-safe (no system/dylib deps).

## Where things stand (committed @ branch `alex/libslic3r-parity-engine`)
- **Ledger: 164 done / 110 partial / 4 deferred / 0 pending of 278 units — pending drained 2026-06-11.**
- lib **and** bin build green; 3DBenchy slices to **filament 3818.67 mm** (golden 3858.97, ~0.99×), 240 layers, top=5.
- **2026-06-10 regression fixed:** the faithful TriangleMeshSlicer port (checkpoint 54792d7) exploded
  gcode 13× because `src/libslic3r.rs` had `SCALING_FACTOR = 0.000001` (PrusaSlicer value; BambuStudio
  libslic3r.h:58 is `0.00001`) → `scaled_f32` made all XY geometry 10× too large. Fixed the constant,
  made `scaled_f32` a plain f32 division (C++ `scaled<float>`, Point.hpp:529 — no +0.5/floor), and fixed
  layer-0 `slice_z` to mid-plane (PrintObjectSlice.cpp:36). Filament 570,107 → 3818.67.
- **`partial` (110)** = faithfully ported except symbols blocked on the **config-hierarchy
  threading** (Print→PrintObject→Layer→PrintRegion) — **now wired (2026-06-12, see below)** —
  a native lib (OpenVDB/CGAL/OCCT/boost-Voronoi), or a not-yet-ported dep. BLOCKED markers
  remain in ~75 files; the threading-blocked subset is the retry worklist (active track).

## Source of truth + dashboard
- `crates/libslic3r-rs/PORT_LEDGER.json` — array of units `{cpp,hpp,rust,area,loc,status}`.
  `status ∈ {pending,partial,deferred,done}`. **This drives the workflow.**
- `crates/libslic3r-rs/PORT_LEDGER.md` — human dashboard (X/278 + per-area table).
- `crates/libslic3r-rs/PROGRESS.md` + memory `project_benchy_parity_gap` — parity history/insights.

## How to keep porting (the workflow)
Saved script (self-contained, resumable, pending-driven):
`/Users/alex/.claude/projects/-Users-alex-Code-Helio-Additive-worktrees-slicer-cli-lofty-dawn-slicer-cli/a86c8d24-2124-4cc9-be67-a594321213dd/workflows/scripts/libslic3r-systematic-port-v2.js`

Invoke it with the **Workflow tool**: `Workflow({scriptPath: "<that path>"})`.
It reads the ledger, ports each `pending` unit one-phase-per-file (faithful, build-gated,
restore-on-fail), updates the ledger, and **git-commits a green checkpoint every 8 files**.
It runs for hours; a fresh `Workflow({scriptPath})` call always resumes from the ledger
(no resumeFromRunId needed — that's session-bound; the ledger is the durable state).

## Per-run operating loop (do this each time a run completes/fails)
1. `devbox run cargo build --manifest-path crates/libslic3r-rs/Cargo.toml` (lib) and `… --bin slicer-cli` (bin) — **both must be exit 0.**
2. If a porter left a **broken tail** (it happens — the commit-agent refuses to commit red), fix it. Recurring kinds, all mechanical:
   - a config struct grew (e.g. `PrintRegionConfig`) → update struct literals in `src/bin/slicer-cli.rs` (`create_default_region_config`) to add the new fields;
   - module path / `pub use` visibility (e.g. `crate::geometry::geometry::is_approx`; `pub use … indexed_triangle_set`);
   - missing `#[derive(Copy)]` on trivial index handles.
   If unfixable after real effort: `git checkout -- <file>` and set that unit back to `pending` in the ledger.
3. **Parity check (must not regress):** build + run the **parity profile** binary (debug-assertions
   off = C++ release semantics; the golden gcode comes from a release build where `assert()` is a noop,
   and faithful `debug_assert!`s — e.g. the degenerate-slice-line assert at triangle_mesh_slicer.rs:294 —
   legitimately fire on Benchy in the dev profile):
   `devbox run cargo build --manifest-path crates/libslic3r-rs/Cargo.toml --profile parity --bin slicer-cli`
   `crates/libslic3r-rs/target/parity/slicer-cli slice -i examples/3DBenchy.stl --settings examples/out/resolved-config.json -o /tmp/rb.gcode` → check `; total filament length` ≈ **3816–3820** (golden 3858.97), 240 layers, gcode lines ≈ 114k.
4. Commit `crates/` with a `Systematic port: …` message, regenerate `PORT_LEDGER.md`.
5. Re-fire the workflow until `pending == 0`.

## Hard rules / gotchas
- **ALL builds via `devbox run …`** — never bare cargo/cmake (devbox provides the toolchain).
- The crate's **test target has ~150 pre-existing unrelated compile errors** (known harness breakage). Only the **lib + bin** builds must be green; ignore `cargo test` failures unless they reference a file you just touched.
- coord_t→i64, coordf_t→f64. Reuse existing crate primitives (grep before adding). No stubs/fakes — block honestly as `partial`.
- 3DBenchy parity is currently a near-match by *volume* but **not byte-identical**; remaining gap is feature distribution (Top surface 5 vs 142, Bridge/Floating-shell missing), all traced to the config-threading blocker + an unsolved `top_fills` coverage issue. See `project_benchy_parity_gap` memory and `PROGRESS.md`.

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
3. `top_fills` coverage fix (gated behind env `TOP_FILLS`) + faithful `discover_vertical_shells` (gated `VSHELL_FAITHFUL`) — see memory.
4. Native-lib-blocked symbols (OpenVDB/CGAL/OCCT/boost-Voronoi) — vendored minimal Rust replacements, only the functions actually used.
