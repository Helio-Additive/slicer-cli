# R64 — raw-loops-before-union measurement: F2 pinned to slice_facet/make_loops, union EXONERATED

Branch: f2-rawloops (off the parity branch alex/libslic3r-parity-engine — has 6529a52, unlike L74-reclass).
Both engines instrumented to dump the RAW LOOPS (`layers_p[layer_id]`, the slice_facet→make_loops output)
BEFORE `make_expolygons`/union, at the Benchy cabin-floor layers (z<0.55). Env-gated `F2RAW=1`.
- rust: crates/libslic3r-rs/src/triangle_mesh_slicer.rs slice_mesh_ex_its, before make_expolygons.
- C++ : references .../TriangleMeshSlicer.cpp slice_mesh_ex, before Slic3r::make_expolygons (REVERTED after).

## THE DECISIVE NUMBERS (F2RAW compare, deterministic)
| layer | z(slice) | C++ nloops | RUST nloops |
|-------|----------|-----------|-------------|
| li=0  | 0.1      | 10        | 10  (MATCH — cavity open at the very bottom in BOTH) |
| **li=1** | **0.3** | **1**   | **10**  ← THE DIVERGENCE |
| li=2  | 0.5      | 1         | 1   (MATCH — cavity closed in BOTH) |

RUST li=1 (z=0.3) raw loops:
  loop0  CCW 545.950mm²  (outer contour — BYTE-IDENTICAL area to C++ li=1)
  loop1..8  CW  9.007 / 23.523 / 8.194 / 11.164 / 9.283 / 10.798 / 11.569 / 3.137 mm²  (8 holes, ~86.7mm²)
  loop9  CCW 8.569mm²  (stray detached island)
C++  li=1 (z=0.3): ONE loop, CCW 545.950mm², 0 holes (solid floor).

## WHAT THIS PROVES (answers the open question F1-union vs F2-facet)
1. **The union / make_expolygons is EXONERATED.** The 10-vs-1 loop divergence exists in the RAW loops
   BEFORE any union or offset. (make_expolygons is also a no-op here: closing_radius=0 → pure union — R63.5.)
2. **The OUTER contour is faithful in both** (loop0 = 545.950mm² to 3dp in both; the only delta is a uniform
   +0.83mm X-translate — a known placement offset, shape-identical, NOT the bug).
3. **The bug is the CAVITY-INTERIOR loops.** Rust's 8 hole-loops at z=0.3 are the SAME geometry (same areas,
   same +0.83mm X-translate) as C++'s holes at z=0.1 (both engines: li=0 = 10 loops). I.e. rust slices z=0.3
   as if it were the lower (open) geometry — the cabin-floor cavity closes ONE SLICE LATE in rust
   (C++ closes between z=0.1→0.3; rust between z=0.3→0.5).
4. Root locus = **F2: slice_facet / make_loops on-plane cap-facet classification at z=0.3.** The near-horizontal
   floor-cap facets (R62 measured vertices ≈z=0.3001) are treated by C++ as AT/above the z=0.3 plane (interior
   contours vanish → solid floor) and by rust as below (interior contours persist → 8 holes). This VINDICATES
   R61 ("mesh slicer on-plane facet classification") and confirms R62's "slicer ruled out" was wrong.

## THE CASCADE (now fully chained, R63 + R64)
rust make_loops emits 8 phantom holes at li=1 (z=0.3) → li=1 region.slices has them → li=2 bottom-bridge
support diff reads them unsupported → over-classifies ~290mm² stBottomBridge → steals from stInternalSolid →
ISI fragments into 244 narrow slivers → detect_narrow routes to Concentric → FillConcentric starves them.
ISI −60..−80, Bridge +15, floating −64 all originate at the slicer loop assembly.

## NEXT FAITHFUL STEP (the fix — NOT taken here; user scoped this to the measurement only)
Split slice_facet-classification vs make_loops-chaining: dump the IntersectionLines count + the cap facets'
exact f32 vertex z at z=0.3 in BOTH engines.
  (a) If C++ slice_facet emits FEWER intersection lines at z=0.3 (the cap facets aren't cut) → the divergence
      is the facet/plane side test (slice_facet, exact-f32 z==slice_z handling of near-horizontal/on-plane
      facets) = deepest F2 / Coord-precision.
  (b) If both emit the same lines but make_loops chains them into holes (rust) vs absorbs them (C++) → it's
      make_loops loop-assembly / orientation.
Most likely (a) per R62's z≈0.3001 facet-vertex measurement. Base any fix on the PARITY branch (not L74-reclass).
GUARDRAILS for the eventual fix: the slice change touches all 240 layers — outer-wall G1 ~22087, time ~43m,
gap-fill ~231, build green; verify per-feature (ISI→494, floating→170, bridge→237) not a coincidental total.

## INSTRUMENTATION STATUS
rust F2RAW probe: KEPT on branch f2-rawloops (env-gated, build-green, re-pins in ~17s via `F2RAW=1 ... compare`).
C++ F2RAW probe: REVERTED (git checkout of the references tree) + slicer_cli rebuilt clean.
