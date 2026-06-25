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

## Current parity (at `dd93fc4`, ROUND 62 — see memory `project_benchy_parity_gap.md` for the full round-by-round log)

- **Material: 0.99771×** — header filament rust 3850.13 / native 3858.97 mm (XY-gated `feat_e2.py`).
- **Time estimate: CONVERGED** — native 43m0s / rust 43m21s (the old "1h29m vs 43m"
  line-3 divergence is **RESOLVED**; the overhang/speed trio is landed).
- **Byte-identical: NO**, but the structural subsystems now match or are faithful:
  outer-wall vertex density matches native (offset rerouted to clipper-z-sys),
  gap-fill 81% closed, seam ~89% byte-exact at established layers, arc-fitter /
  simplification / medial-axis (boostvoronoi) / chaining / retraction / overhang-trio
  all proven faithful. `lslices` and `detect_surfaces_type` are faithful **given inputs**;
  the **mesh slicer is NOT bit-faithful at the Benchy hull bottom** (see below — R62's
  "slicer faithful" claim was overturned by R63's both-engine A/B).
- **The remaining residual is ONE lever — ROUND 63 (both-engine A/B) RELOCATED it from the fill
  stage back to the MESH SLICER (F2), overturning R62's "fill reclassification / slicer ruled out"
  framing.** R62 inferred slicer-faithfulness from code reading and never measured native's slice. R63
  instrumented BOTH engines through the cascade: at **layer 1 (pz=0.4) rust's slice carries 8 spurious
  ~10mm² holes (~86mm² total) that C++ does not** → li=2 over-classifies ~290mm² as BottomBridge → steals
  from InternalSolid → the ISI leftover fragments into narrow slivers → `FillConcentric` starves them.
  internal-solid −60..−80, floating −64, bridge +15 ALL fall out of this one slicer divergence;
  `detect_narrow`/`FillConcentric`/`lslices` are faithful given inputs. **R63.5 correction:** the
  `make_expolygons:1312-1313 scale()` suspect is a runtime NO-OP (closing_radius=0 → pure union) and the
  `closing_radius=0.049` lever is refuted by magnitude (10mm² holes can't be sealed by a 0.049mm close).
  Real root = **F2 mesh-slicer on-plane facet classification** at the near-horizontal cabin floor
  (exact-f32 z==slice_z) — VINDICATES R61. NO faithful fill-stage fix converges it. Next step: dump raw
  loops before union at z=0.3 in both engines. See the R63/63.5 round-log + `docs/parity/R63_reclass_findings.md`
  (branch `L74-reclass`).

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
2. **THE FILL-SURFACE RECLASSIFICATION (the one remaining cascade root — coupled, needs a holistic fix).**
   Material (feat_e2): internal-solid 494→414 (−80, UNDER), sparse 531→598 (+66, OVER),
   floating-vertical-shell 171→107 (−64). Cascades into the systematic extrusion-arc over (+~1250).
   **ROUND 58-62 definitively RULED OUT everything upstream** (do NOT re-chase these):
   the mesh slicer (`slice_facet`/`make_loops` bit-faithful; slice grid bit-identical;
   cabin-floor facets at z≈0.3001 → cavity open at li=1 / closed at li=2 in BOTH engines —
   rust closes it correctly), `lslices` (byte-identical to `slices`), `detect_surfaces_type`
   (rust creates the CORRECT bottom-bridge at li=2, 91.7mm²), `discover_horizontal_shells`
   (no-op), `has_voids`/`surfaces_covered` (Benchy fill_density=0.15 → C++ also nullptr),
   the geo-clipper offsets in the narrow gate (A/B clib reroute byte-identical), and
   `clip_fill_surfaces` (dead code: `infill_only_where_needed` static-false).
   **ROOT (ROUND 62, definitive):** the correctly-born li=2 bottom-bridge/internal-solid is
   **reclassified DOWNSTREAM in the fill stage** — `detect_narrow_internal_solid_infill`
   (Fill.cpp:453-546, `fill/mod.rs`) narrow-detects it → routes to Concentric/floating →
   the **`FillConcentric` no-boundary-loop bug** (`fill_concentric.rs`; C++ seeds
   `loops=to_polygons(expolygon)` at FillConcentric.cpp:30) emits ~0 for sub-spacing strips.
   Native keeps/fills it (bridge or rectilinear); rust narrow-floats + starves it.
   **WHY IT'S NOT YET FIXED — it's COUPLED (3 pieces must land together or it regresses):**
   (a) **R53 gap-fill subtraction reorder** (branch `L74-fill`) — faithful, but alone un-masks
       the deficit (material 3850→3828); (b) **`FillConcentric` boundary-loop seed** (in
       `/tmp/vshell_findings.md`) — alone OVERSHOOTS +106mm (fills mis-sized regions); (c) the
       **reclassification correction** — why rust narrow-floats what native keeps as
       bridge/rectilinear (the unresolved knot). HOLISTIC NEXT STEP: land (a)+(b)+(c) on one
       branch off `L74-fill`, verifying **per-feature** convergence (internal-solid→494,
       floating→170, bridge→237, total→3859) NOT the coincidental aggregate, and guarding the
       byte-matched outer wall (G1 ~22087/native 22053 — the rejected `slicer-fix` regressed it to +371).
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

## WIP branches (preserved on GitHub — building blocks for the holistic fill-reclassification fix)

All are pushed; none merged to `alex/libslic3r-parity-engine` (each is a real fix or diagnosis held back to avoid regressing parity). The holistic fix (lever #2) should branch off `L74-fill` and combine the faithful pieces, verifying per-feature.

| branch | holds | status |
|--------|-------|--------|
| `L74-fill` | R53: gap-fill subtraction reorder (C++ order: subtract before infill opening) — **faithful** | use as the BASE; alone it un-masks the deficit (3850→3828) |
| `vshell-fix` | diagnosis: the `FillConcentric` no-boundary-loop bug + analysis (`/tmp/vshell_findings.md`) | the FillConcentric fix; alone OVERSHOOTS +106mm |
| `f1-fill` / `void-clamp` / `bottom-surface` / `slicer-fix` / `lslices-phase2` / `slice-facet` | diagnosis trail that RULED OUT slicer / lslices / clamp / region-partitioning (with the data) | reference only — do NOT re-chase these dead ends |

Measurement (unchanged): `COMPARE_KEEP_DIR=/tmp/cmp devbox run -- target/debug/slicer-cli compare --config tests/configs/stl-inline-config.jsonnet`; per-feature material via `/tmp/feat_e2.py`. The merged units-fix (`make_expolygons` mm-not-scaled, `dd93fc4`) is a no-op today but required the moment any nonzero `closing_radius` is plumbed.
