# libslic3r-rs ↔ BambuStudio G-code Parity — Status

Goal: the Rust slicer (`crates/libslic3r-rs`, the `slicer` crate) produces G-code
**byte-identical** to C++ BambuStudio for the same job. Branch:
`alex/libslic3r-parity-engine`.

## How to measure

The two-engine compare harness runs the *same* job through C++ BambuStudio
(subprocess) and the Rust engine (in-process) and diffs the G-code:

```
COMPARE_KEEP_DIR=/tmp/cmp devbox run -- \
  target/debug/slicer-cli compare --config tests/configs/stl-inline-config.jsonnet
```

- Dumps `native.gcode` / `rust.gcode` to `$COMPARE_KEEP_DIR`.
- A clean Benchy slice takes **~17 s**. If it takes minutes, suspect orphaned
  slicer processes saturating CPU (`pkill -f slicer-cli`).
- **Per-feature MATERIAL** must be measured with `/tmp/feat_e2.py` (counts E only
  on moves with real XY motion). The naive sum (`feat_e.py`) is **contaminated by
  deretraction-priming** moves and inflates native — do not trust it.
- Track the **header filament length** and **per-feature material dE** (rust−native),
  not feature *counts* (counts are a feature-run-segmentation artifact).

## Current parity (at `f67f3f4`, ROUND 48 — see memory `project_benchy_parity_gap.md` for the round-by-round log)

- **Material: 0.99771×** — header filament rust 3850.13 / native 3858.97 mm (XY-gated `feat_e2.py`).
- **Time estimate: CONVERGED** — native 43m0s / rust 43m21s (the old "1h29m vs 43m"
  line-3 divergence is **RESOLVED**; the overhang/speed trio below is landed).
- **Byte-identical: NO**, but the structural subsystems now match or are faithful:
  outer-wall vertex density matches native (offset rerouted to clipper-z-sys),
  gap-fill 81% closed, seam ~89% byte-exact at established layers, arc-fitter /
  simplification / medial-axis (boostvoronoi) / chaining / retraction all proven faithful.
- **Remaining residual is surface CLASSIFICATION** (not geometry/units): rust
  over-fills **bridge-infill** (moves 1286→2346) and under-blocks **internal-solid**
  (389→134 blocks), which also drives the systematic extrusion-arc over (+~1250,
  a downstream symptom of fragmentation — the arc-fitter itself is byte-faithful).

## What's done (verified, on the branch)

- Surface classification: removed a spurious mesh-slicer `detect_surfaces_type`
  that fragmented surfaces (3/layer → 37-44) — **Top surface → parity**.
- Gap-fill: fixed a `variable_width` mm/scaled units bug + a missing
  `douglas_peucker` pre-simplify (gap-fill is at parity; the apparent gap was a
  priming-measurement artifact).
- Arc-fitting: wired `ArcFitter` into G-code export — **0 → ~12k G2/G3 moves**
  (native ~12k); also fixed a missing arc filament-length accumulation.
- Time estimator: ran the (faithful) `GCodeProcessor` and wired its accel-aware
  time into the header (correct format `; estimated printing time (normal mode) =`).
- Per-segment **speed modulation** (overhang speed + smooth-speed): toolpath
  density now matches native (outer-wall ~40k moves, ~6.7k distinct feedrates).
- **`crates/clipper-z-sys`**: vendored BambuStudio `ClipperLib_Z` (clipper.cpp +
  `CLIPPERLIB_USE_XYZ`) via a C-ABI shim; `clip_extrusion` validated. Portable
  binary (static C++; only libc++/libstdc++ residual). Wraps in
  `crates/libslic3r-rs/src/clipper_z.rs`.

## Remaining levers (all foundational/large — tackle as separate scoped efforts)

1. **Overhang trio + time-estimate — RESOLVED (ROUND 48).** `overhang_degree` is now
   `f64`, `merge_same_speed_paths` and `detect_bridge_wall` are ported + called, the
   speed interpolation is faithful, and the **time estimate converged (43m0s vs 43m21s)**.
   Overhang-wall feature matches (90 vs 91). This whole lever is done — do not re-attempt.
2. **Internal-solid UNDER-fill via region-PARTITIONING divergence (PRIMARY lever).**
   Material (feat_e2): internal-solid 494→414 (delta -79.95, UNDER), sparse 531→598
   (+66.13, OVER), floating-vertical-shell 171→128 (-42.42). Solid that native fills is
   routed to sparse in rust. **ROUND 49 pinned the chain decisively** (see
   `/tmp/class_findings.md`):
   - Layer→Z mapping was off-by-one in prior notes: gcode "layer num N" == internal
     li N == Z=N*0.2. The pinned divergent layer is **gcode layer 71 == li 71 == Z14.4**.
     Memory's fill mm² ("3.51/2.86/0.61") was also off 100× — actual 351/285.7/61.1 mm².
   - `discover_horizontal_shells` is a **CORRECT no-op** (ensure_vertical_shell_thickness
     = enabled → the per-region `continue` fires in BOTH C++ and rust; before==after on
     every layer). NOT the bug. The old "horizontal-shells offset" lead is a **dead end**.
   - Proximate cause: `detect_narrow_internal_solid_infill` (Fill.cpp:453-546,
     fill/mod.rs:777) reclassifies ALL of rust's InternalSolid at Z14.4 (3 expolys) as
     `narrow_floating` → stFloatingVerticalShell → 0 "Internal solid infill" emitted
     (native emits 2). But this classifier is **FAITHFUL** to C++.
   - **DECISIVE TEST — geo-vs-clib offset DISPROVEN here**: A/B rerouting the two
     `offset_expolygon` calls (is_narrow_infill_area shrink, floating-test grow) to
     `shrink_clib`/`grow_clib` gives BYTE-IDENTICAL narrow/floating counts. The
     geo-clipper over-segmentation hypothesis does NOT apply to this gate.
   - **ROOT**: the InternalSolid/Bridge **region geometry diverges upstream**, already at
     Z14.2 (where gcode COUNTS coincidentally match but XY bboxes differ completely):
     native internal-solid = small slivers near walls (perimeter shadow, x[4.25,5.69]);
     rust = one large central region (x[4.99,16.81]); native vs rust bridge at entirely
     different XY. The `surfaces_covered`/`has_voids` omission in
     `process_external_surfaces` is **NOT** the cause for Benchy (fill_density=0.15 ⇒
     `has_voids=false` ⇒ C++ also passes nullptr — exact). Root is in
     `LayerRegion::process_external_surfaces` (region_expansion wave) region partitioning
     and/or `bridge_over_infill` unsupported_area. DEEP multi-pass region-geometry rework,
     NOT a clean single offset reroute — correctly bailed (no regression shipped).
   - SCOPED PLAN (next): instrument `process_external_surfaces_wave` output by region at
     Z14.2/14.4 vs native; verify discover_vertical_shells produces native's sliver
     pattern; check bridge_over_infill unsupported_area XY placement. Re-assess Arachne
     narrow-fill residual (ROUND 47 `/tmp/solid_findings.md`) only after this lands.
3. **Clipper coordinate byte-exactness (F1).** The live clipper backend is
   `geo-clipper` at scale 1000 (1 µm grid) fed via an mm float round-trip, vs C++
   ClipperLib at scale 100000. For byte-exact coordinates, feed the C++ clipper the
   same i64 inputs (raw FFI / vendor BambuStudio's exact `clipper.cpp` as a `-sys`
   crate). NOTE: F1 is *not* the cause of the toolpath-density gap (verified —
   bumping the scale changed nothing).
4. **Seam / toolpath ordering** — perimeter/seam emission order differs; needed
   before the two G-code streams are 1:1 alignable past the headers.
5. **Byte-exact bridges/overhang** would use **Clipper2Lib_Z** (`SetZCallback` +
   Z-preserving Clipper2 offset), a later `clipper-z-sys` extension.

## Notes

- `cargo test` for the `slicer` lib is **pre-existing-broken** (unrelated
  arachne/surface/fill errors) — only `cargo build` gates are authoritative on this
  branch. `clipper-z-sys` tests pass.
- Many compiler warnings are intentional: `///` C++-reference doc comments and
  faithfully-ported-but-gated/unwired code.
