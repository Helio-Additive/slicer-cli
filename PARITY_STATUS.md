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

## Current parity (ROUND 80 — see memory `project_byteparity_admesh.md` + `project_benchy_parity_gap.md`)

- **★ R80 — ADMESH REPAIR LANDED (the +93-vert mesh root SOLVED; material-neutral, faithful).** R79h's
  "2284-line bail" was OVER-scoped: PHASE-0 ground truth showed benchy is MANIFOLD after the exact check
  (conn3 = number_of_facets) → C++ SKIPS the bit-sensitive nearby-merge entirely (the guard
  `connected_facets_3_edge < number_of_facets` is false). So the +93 decomposed into just (1) 552 degenerate
  facets removed + (2) topology-based shared-vertex generation — ~270 DETERMINISTIC lines, no
  tolerance/order ambiguity. Ported `stl_repair.rs` (faithful admesh: degenerate-facet removal + exact-edge
  neighbor graph [HashEdge byte-key, −0/+0 normalized] + `stl_generate_shared_vertices` fan traversal),
  now the DEFAULT binary-STL path, replacing the exact-f32-bit HashMap dedup that wrongly kept 552
  degenerate facets + skipped shared-vertex gen (correct-beyond-parity — rust was genuinely wrong vs C++
  `from_stl(repair=true)`). RESULT (default, re-verified): **vert count 112569 EXACT** (was 112662 — the
  +93 GONE), facets 225154, manifold — all EXACT vs C++; **material-neutral** (per-feature stable: ISI
  −30.5, sparse −12.48, floating +31.55, outer +2.12; total −0.42 ≈ baseline noise); R65 floor intact
  (li=1 loops==1, repair is pre-quantize); exact-identical moves 11964→11971; build green. Landed parity
  @0aad342. The mesh is now FAITHFUL. REMAINING byte-parity blocker = the X-FRAME centering (separable last
  sub-lever): REPAIR alone leaves the ~0.8245mm slice-frame X-offset (first-div native X.936 / rust X3.146);
  REPAIR+FRAME_PAIR regresses material +9.92 because frame-pair's PRE-SLICE per-vertex mesh-translate
  perturbs the f32 slice geometry (+ R65-quantize interaction). The faithful X-frame needs either (a)
  centering INSIDE the slice-time transform (C++ make_trafo_for_slicing fused op), or (b) the export-origin
  ALONE without shifting slice geometry. frame-pair foundation is on parity but GATED (FRAME_PAIR, default-off).
- **★ R79h — (resolved by R80) suspected the admesh REPAIR subsystem (2284 lines) as a likely bail.** Scoped the
  +93-vert root. Per-stage bisect: benchy binary STL = 225706 facets → 677118 raw verts. rust dedups by
  EXACT f32 bit-key (`[x.to_bits(),y.to_bits(),z.to_bits()]` HashMap, stl.rs:158-180) — NO tolerance, NO
  repair → 112662. C++ `ReadSTLFile(repair=true)` → `trianglemesh_repair_on_import` (admesh) →
  `stl_generate_shared_vertices` → 112569. The +93 enters at the REPAIR stage rust SKIPS ENTIRELY (stl.rs:82-85
  comment admits repair + shared-vertex generation are not reproduced). C++ repair (TriangleMesh.cpp:79-160):
  stl_check_facets_exact → **stl_check_facets_nearby(tolerance)** → stl_remove_degenerate →
  stl_fix_normal_directions, then stl_generate_shared_vertices builds the index from the repaired TOPOLOGY
  (neighbor graph, not a hash). The 93 = near verts merged by stl_check_facets_nearby (tolerance =
  stl.stats.shortest_edge, ITERATED 2× with increment — DATA-DEPENDENT, not a fixed epsilon) + degenerate
  facets removed, that rust's exact-bit dedup keeps. SIZED: NOT a bounded one-tolerance tweak — the faithful
  fix needs the admesh subsystem bit-for-bit (~2284 lines: connect.cpp 743 facet-connectivity graph, shared.cpp
  263 topology traversal, stlinit 389, util 399, normals 239, stl_io 251), and matching the EXACT 93 merges
  needs admesh's exact edge-matching + union-find ORDER + iterative tolerance — hits 2 of 3 bail triggers
  (whole subsystem + order/tolerance-sensitive). Does NOT touch R65 (repair is pre-quantize). **VERDICT:
  BAIL/BANK. Material parity (1.000×) is the achieved + landed goal; byte-identical gcode is gated SOLELY on a
  full faithful admesh-repair port — a large (multi-session), bit-sensitive lever with uncertain byte-payoff
  (porting it may still not reproduce exactly 93). Every other layer end-to-end is faithful.** frame-pair
  @9892a5d banks the faithful f64 centering (gated, NOT merged). branch stl-load (scope-only, no edits).
- **R79g — SLICE ARITHMETIC EXONERATED; true root = STL-LOAD MESH (+93 verts), a discrete root
  (corrects R79f).** Funded the slice-intersection lever — it OVERTURNED R79f's "slice-intersection is the
  wall". C++ slice_facet interpolation (TriangleMeshSlicer.cpp:261-280): `t=(double(slice_z)−double(b.z))/
  (double(a.z)−double(b.z))`, `x=coord_t(floor(double(b.x)+(double(a.x)−double(b.x))·t+0.5))`, on-vertex →
  `a.x` passthrough. rust slice_facet (triangle_mesh_slicer.rs:331-358) is ALREADY BIT-IDENTICAL (same f64
  t, same floor(+0.5), same passthrough) → **slice-intersection arithmetic is FAITHFUL.** The loop-point
  bit-check is MOOT because the INPUT mesh differs: a full-mesh FNV bit-hash over ALL transformed verts
  shows **C++ n=112569 vs rust n=112662 — +93 VERTICES** (FRAME_PAIR-independent — same count with centering
  off). So rust's STL-load / mesh-repair (vertex merge/dedup tolerance or degenerate-facet/edge collapse)
  keeps 93 verts C++ merges → different triangulation → all downstream byte-divergence inherits from this.
  **STRATEGIC: every downstream layer is now EXONERATED** — slice arithmetic, centering/frame (verts that
  exist bit-match), seam placer, perimeter-gen, entity order, coords-format all proven faithful. The mesh is
  the SOLE remaining upstream root. So the +93-vert STL-load fix is a BOUNDED discrete lever (NOT the f32
  rabbit hole) and is potentially the LAST keystone — if the mesh is made identical, byte-parity could
  cascade out. NEXT LEVER (if byte-parity pursued): scope why rust keeps 93 verts C++ merges — STL-load
  vertex-merge tolerance / degenerate handling (TriangleMesh repair/its_merge_vertices). Material parity
  (1.000×) remains the achieved + landed goal. frame-pair @9892a5d banks the faithful f64 centering (gated,
  NOT merged); default unchanged (−0.44), R65 li=1 loops==1 intact, build green.
- **★ R79f — FLOOR REFINED (corrects R79e): centering is FAITHFUL (verts bit-match C++); the wall is the
  slice-INTERSECTION f32, one layer deeper (F-class; BAILED CLEAN).** Funded the fused-matmul port — it
  OVERTURNED R79e's "matmul wall" framing. C++ instrumentation (`transform_mesh_vertices_for_slicing`,
  TriangleMeshSlicer.cpp:1840): the slice tf is essentially PURE ×1e5 scale (tf[0][3]≈0.008 negligible) — the
  centering is NOT in the slice matrix, it's baked into the MESH VERTICES (C++ vert.x 5.7975 vs rust raw
  6.622, Δ=0.8245 exact; scale step byte-identical). Build = -O3 arm64 (ffp-contract=on). FIX = clean
  f64-subtract of the exact truncated-grid center_offset (NO FMA matmul needed — R79e's regression was just
  double-rounding + a spurious export-origin). RESULT: **rust verts now BIT-MATCH C++ EXACTLY** (in_bits
  40b9851f == C++, scaled out 579750 == C++), R65 floor intact (li=1 loops==1) → the centering/frame
  arithmetic is FULLY FAITHFUL. **But the SLICES STILL DIVERGE on bit-identical verts** — material +10.22
  (broad: ISI −21/sparse −6/bridge +5/top +3), seam 0%, first-div native X.936 vs rust X2.32. Same input →
  different loops ⇒ the wall is the slice-INTERSECTION f32 edge-interpolation (`slice_facet_at_zs` line-plane
  crossing), non-faithful vs C++ and entangled with the R65 quantize hack. BAILED CLEAN: byte-identical
  perimeter coords need bit-exact f32 edge-interpolation across the whole slicer — the open-ended precision
  rabbit hole (each f32 layer reveals the next). **Material parity (1.000×) is the achieved + landed goal.**
  frame-pair branch now holds the FAITHFUL gated centering (verts bit-match) + export-origin, banked for any
  future slice-f32 work (NOT merged — regresses default w/o the deeper f32 match). (Caveat: 10 verts
  bit-checked; the broad shift points to intersection arithmetic, not incomplete centering.)
- **R79e — (SUPERSEDED by R79f) suspected the fused f32 slice-transform matrix as the wall.** Built
  `slice_center_xy` per-vertex XY −center (Z untouched, R65-safe), (2) `GCodeWriter.gcode_origin` +
  `set_gcode_origin` subtracting m_origin from absolute coords at the writer chokepoint (I/J left relative,
  correct), threaded app_slice→export. Net = raw − 2·center_offset (derived: slicer_cli `set_instances` does
  shift += center_offset so m_origin = center_offset, doubling). RESULT: the FRAME IS CORRECT — slices now
  match C++ to 0.001mm (Δx 0.8245→0), export shift lands (first-div 3.146→1.495 = −2·center) — BUT material
  REGRESSES +9.76→+9.98, seam-match stays 0%. Applied BOTH exactness fixes (exact center_offset =
  unscale(trunc(center/SCALING_FACTOR)·SCALING_FACTOR), C++ truncation not rust's round; single f64-subtract
  → one f32 cast, not double-f32): bit-check UNCHANGED → the residual is NOT the center value or rounding
  sequence. ROOT (definitive): C++ `make_trafo_for_slicing` (TriangleMeshSlicer.cpp:1827-1862) FUSES the
  −center translation AND the ×1e5 scale into ONE f32 matrix-multiply per vertex (`tf=t.cast<float>(); v=tf*v`);
  rust does TWO separate f32 ops (center-subtract in mm, then ×1e5 scale) → sub-ULP drift per vertex → facet
  on-plane classification re-quantizes (the +9.98, R65-family) → loops/seam don't byte-align. The bbox
  matches to 0.001mm; the gap is SUB-ULP. **BAILED CLEAN** per the pre-set criterion: matching it requires
  reproducing C++'s fused `make_trafo_for_slicing` Eigen-f32 matmul (FMA rounding) bit-for-bit across the
  slicer — the F-class precision port scoped as bail-worthy. **VERDICT: material parity (1.000×) is the
  achieved + landed goal; BYTE-identical gcode is gated on the fused-f32 slice-matrix port (deferred).** The
  frame+export-origin mechanism is correct/gated/reusable (banked on branch frame-pair, NOT merged — it
  regresses default without the f32 port).
- **★ R79d — ROOT PINNED: constant ~0.8245mm X PLACEMENT offset (R65-XY family, BOUNDED; diagnosis-only).**
  Scoped the perimeter-geometry rung — 3 probes. DECISIVE bisect: the RAW SLICES fed into PerimeterGenerator
  ALREADY differ — at L100 every input-slice bbox is X-shifted by a dead-constant +0.8235..0.8254mm (both
  min AND max corners), dy ±0.0009 ≈ 0; n_slices match (5=5). A pure RIGID X TRANSLATION → NOT F1 geo-clipper
  (which would give variable per-loop deltas + Y noise), NOT a PerimeterGenerator bug. ROOT = `app_slice.rs`
  mesh placement: it does `mesh.translate(0,0,dz)` with the comment *"no XY centering, matching C++
  slicer_cli"* — but C++ slices land ~0.8245mm offset in X that rust never applies. This is the **R65
  family**: R65's `quantize_f32_center_roundtrip` handles only center-**Z** (Benchy center_z=24); the C++
  X/Y placement/centering round-trip (volume mesh stored bbox-centered + instance trafo re-place, in X+Y) is
  NOT replicated. Same shape as the gap-interleave empty-collection bug: a comment-asserted no-op that isn't
  faithful. (Probe-3 note: rust raw slice CONTOURS carry ~4× points [C++ 96 / rust 389] — a separate
  densification; but matched output perimeter loops are 32=32 pts, so perimeter-gen normalizes it — the
  densification is a candidate FOLLOW-ON if arcs don't converge after the placement fix.) VERDICT: BOUNDED —
  fund a faithful XY placement fix in app_slice.rs (match C++'s 0.8245mm). Expected cascade: slices →
  perimeters → seam (proven faithful) → likely shrinks the arc gap. NOT the F1 rabbit hole.
- **R79c — SEAM EXONERATED; root descends to PERIMETER GEOMETRY (diagnosis-only).** Scoped + debugged the
  seam rung: it is NOT a missing subsystem — `gcode/seam_placer.rs` is a full 3165-line SeamPlacer (aligned
  mode, wired, running). Both-engine unit-case on one matched 32-pt external loop at L100 (both finalized):
  the seam placer is FAITHFUL — both pick the SAME candidate index (k=6), SAME local_ccw_angle (−1.51531 vs
  −1.51518), SAME visibility rank (0.166/0.169), SAME comparator decision, SAME npts (32). The map's
  "12-14mm" was index-misalignment; the true order-independent per-seam delta is ~0.825mm — and it's a
  UNIFORM shift of the WHOLE candidate set (every point x ≈ rust_x − 0.825), so the seam lands on the
  identical vertex of a SHIFTED loop. NEITHER engine has the other's exact X → not a comparator tie-break.
  ROOT = the PERIMETER GEOMETRY fed into the placer: (1) a uniform ~0.825mm X offset of the loop, and (2)
  loop-SPLITTING — rust feeds 10 external perimeters at L100 vs C++ 5 (rust splits some walls into 2: C++
  npts=93 ↔ rust 65+92; C++ 32 ↔ rust 22+32). Both are PerimeterGenerator wall-placement divergences,
  BEFORE the seam placer. NOTE: material is at-native (R77) and a pure ~0.8mm translation preserves loop
  length → consistent. **The seam placer would byte-match if fed identical perimeters.** REVISED CHAIN:
  PERIMETER-GEOMETRY (offset ~0.825mm + loop-split; possibly F1 geo-clipper precision) → seam [faithful,
  waits on geometry] → arcs (partly follows wall geometry) → coords [done].
- **R79b — STRUCTURAL MAP (diagnosis-only; SEAM is the gating rung, coords already exact).** With order
  converged (R78/R79), mapped the remaining structural divergences. **FIRST body divergence = move 13**, the
  first perimeter loop's START point (native `G1 X.936 Y2.228` vs rust `G1 X3.146 Y.502`) — a SEAM
  divergence; prelude (first 12 moves) is byte-identical. **COORDINATE FORMAT IS ALREADY BYTE-EXACT** where
  geometry aligns (same decimals/precision; 11964 moves exact-match any-order) → **no F1 coordinate rung
  needed.** SEAM: loop counts match (outer 770/772, inner 667 EXACT) but only **9.2%** of native outer
  seams have a rust loop starting within 50µm (6.6% within 0.6mm) — ~84% start at a different seam point,
  median delta ~12-14mm → a real Seam.cpp PLACEMENT-algorithm divergence (nearest/aligned/rear), gating
  because it cascades into loop direction + arc segmentation. ARCS: both engines arc-fit
  (enable_arc_fitting=1); native 11906 G2/G3 / rust 14960 (+25.7%, ~3000 extra) — an arc-fit FIDELITY
  divergence (rust over-segments), SECOND to seam (partly follows from seam's loop start/direction). Body
  lines native 139498 / rust 148631 (+6.5%). **Recommended rung order: SEAM → ARCS → (coords done).**
- **★ R79 — GAP-FILL INTERLEAVE LANDED (order rung 2; material byte-unchanged).** Next divergence after
  island grouping = gap-fill EMISSION position. Native interleaves gap-fill per-island (C++ Fill.cpp:757-762
  wraps each thin_fill in its own EEC and PUSHES it into `layerm->fills`); rust's port had the loop but
  pushed an EMPTY collection — `collection.entities.push(thin_fill)` was MISSING (latent no-op), so gap
  stayed batched in thin_fills (584 blocks vs native 816). FIX (two faithful parts): (1) actually move each
  thin_fill into `fills` so gap rides the normal infill island-assignment + chaining + per-island emission;
  (2) persistent FEATURE-role dedup — the `;FEATURE` marker used a per-CALL local role; C++
  `m_last_extrusion_role` (GCode.hpp:538) is a PERSISTENT member, so consecutive same-role entities across
  separate extrude calls don't re-emit the marker — added `GCodeWriter::last_extrusion_role` (always-on,
  independently material-neutral). RESULT: **gap blocks 584→830** (native 816, ~97%; inner-wall 667 EXACT,
  outer 770/772); **MATERIAL BYTE-UNCHANGED** (feat_e2 XY-gated: total −0.43, gap +1.28, ISI −30.53,
  sparse −12.48 — all stable vs R77/R78). Localization verdict: divergence was the GENERATOR entity-tree
  placement (make_fills), NOT the emission iteration. Un-gated → default, landed parity @15bd4f7, build
  green. RESIDUAL = the ISI-grouping rung (Internal-solid blocks ~237 vs native 389 under, Floating ~210 vs
  122 over — the near-cancel ISI/floating split, now an emission/grouping difference distinct from gap).
  Dependency chain: island-order [R78] → gap-interleave [R79] → ISI-grouping → seam → arcs (G2/G3) → F1.
- **★ R78 — ISLAND-GROUPED LAYER EMISSION LANDED (byte-parity phase opens; material byte-unchanged).**
  Material is at native (R77); the remaining gap to byte-identical is STRUCTURAL (entity emission order →
  seam → arcs → coordinates). First divergence localized = entity emission ORDER. Ported C++
  `GCode::process_layer` island grouping (GCode.cpp:4340-4392): layer extrusions grouped by island
  (`lslices` by bbox area), per-island perimeters→infill, replacing the flat per-region loop (print.rs).
  Added `extrude_{perimeters,infill}_entities` (subset emit) in exporter.rs; fixed a latent bug —
  `layer.lslices_bboxes` was never populated by the rust port (the island grouping was a silent no-op
  without it). RESULT: order converges at the ISLAND level — **outer-wall blocks 746→770 (native 772,
  near-exact); gap-fill 279→584 (native 816, ~60%)**; **MATERIAL BYTE-UNCHANGED** (feat_e2 XY-gated: total
  −0.37, sparse −12.48, ISI −30.50 — stable vs R77; the raw move-set differs ~15% but that's
  deretraction-prime re-ordering + loop re-seeding from re-chaining the re-grouped entities, NOT material).
  Time 43m28s, build green. Un-gated → default (landed @2113bbc). RESIDUAL = the INTRA-region per-perimeter
  gap-fill interleave (native emits gap ~1:1 after each perimeter LOOP, 816≈772; rust still batches gap
  per-island, 584) — a perimeter-generator/emission ordering inside each island, the NEXT sub-lever toward
  full order convergence → then SEAM becomes cleanly measurable. Dependency chain: island-order [LANDED] →
  per-loop gap interleave → seam → arcs (G2/G3) → F1 coordinate byte-exactness.
- **★ R77 — MATERIAL AGGREGATE AT NATIVE (1.0000×). The emitter-pair landed.** Ported faithful FillGrid
  `fill_surface_by_multilines` (combined two-direction sweep over a SHARED copy-rotated offset base +
  `make_fill_lines_raw` + grid-align) and wired the already-ported (never-called, proven-faithful)
  `fill_base::connect_infill` once on the combined set. The blocker was NOT connect_infill (a both-engine
  UNIT-CASE replay proved rust's connect byte-exact vs C++); it was the RAW-LINE input — rust's
  rotate-then-offset put endpoints off the shared `polygons_outer` so the connect couldn't snap them. Fix =
  copy-rotate the offset base (FillRectilinear.cpp:501) + faithful `make_fill_lines` + `align_to_grid`.
  RESULT: **sparse +55→−12.48** (519.2/531.6, 78% closed); **TOTAL +67→−0.09 (3858.88 / native 3858.97 =
  0.99998×)**; time 44m22s→43m53s. Blast radius PERFECTLY contained — ISI/bridge/top/bottom/gap/walls all
  BYTE-UNCHANGED (single-direction rectilinear path untouched). Build green. The biggest single converging
  fix of the run; the connect_infill stub is retired for grid. REMAINING (all per-feature, aggregate is
  done): the **ISI −30 / floating +31.5 split** (near-cancel material; classification/attribution — rust
  narrow-floats regions native keeps internal-solid; entangled w/ the gated fragmentation work), and small
  bridge +3 / top +2 / gap +1. The remaining path to BYTE-identical is structural (seam/toolpath ordering,
  G2/G3, coordinate byte-exactness), not material.
- **R75 — infill raster `overlap=0` LANDED (faithful, biggest single material move yet).** rust passed a
  spurious `overlap = spacing*0.15` into the raster offset (layer.rs) over-extending EVERY infill line; C++
  `Fill::overlap=0` for the main filler (FillBase.hpp:183, Fill.cpp:995/1007 — `infill_overlap` flows only
  into `no_extrusion_overlap`, NOT the raster geometry). Fix = `overlap=0`. **Total +114.83→+67.33 (−47.5;
  3926.30 / native 3858.97 = 1.017×, was 1.030×)**; sparse +68→**+55**, bridge +10→**+3**, top +15→**+2** (all
  toward native); walls/gap unchanged; time 44m23s; build green. Honestly **unmasked the ISI deficit**
  (−23→**−30**) — the spurious overlap was a compensating bug inflating solid lines (completeness-over-aggregate
  per the playbook). REMAINING material levers (post-overlap): **sparse +55** (still the biggest over — overlap
  was only −13 of it; ~+10.5% line excess remains, beyond the raster offset → re-localize the grid emitter),
  **floating +31.56 / ISI −30.28** (near-cancel net; the concentric/floating emitter mis-split, R69 lever).

- **R74 — group_fills post-loop + Ord fix LANDED (faithful, −6.87 material, no regression).** Ported the
  missing C++ Fill.cpp:361-373 post-loop (`union_safety_offset_ex` + `diff_ex` vs accumulated groups) + fixed
  a real `SurfaceFillParams::Ord` defect (rust omitted C++'s first sort key — decreasing bridge_angle,
  "bridges first"). Effect: the union merges near-touching **bridge** fragments → bridge +16.82→**+10.07**;
  total 3973.85→**3967.08**. DECISIVE NEGATIVE: union is **MOOT for sparse** (599.85 unchanged) — the grid
  emitter already unions internally, so **sparse +68 is NOT a group_fills problem**; it's in the grid
  emitter's line generation on the already-unioned area. Walls/gap/top/bottom/ISI/floating all unchanged.
  NEXT material targets (now precisely localized to the EMITTERS): **sparse +68** = grid emitter line-gen
  (spacing/boundary/connect on the unioned area); **ISI −23 / floating +32** = concentric/floating emitters.

- **Material: per-feature is the metric, NOT the aggregate.** Three big subsystem fixes landed (R65 slicer +
  R67/R68 Arachne + R69 floating). Current rust 3973.85 / native 3858.97 (aggregate +115, time 45m0s vs 43m).
  The aggregate is temporarily OVER because the Arachne port CORRECTED the under-features (the Arachne pipeline
  now produces beads where it produced 0), which UN-MASKED the pre-existing **sparse +68 / bridge +17**
  over-production. Per the playbook "completeness > coincidental aggregate closeness".
- **THE ISI/FLOATING SPLIT IS ONE FRAGMENTATION ISSUE, NOT TWO DEFICITS (R69, both-engine proven).** ISI −23
  (470.9/494.1) and floating +32 (202.7/170.8) look like independent under/over, but **COMBINED ISI+floating =
  rust 673.6 / native 664.9 = +8.75, near parity**. R69 ported the faithful `FillFloatingConcentric` (Z-clipper
  `detect_floating_line` — was thought blocked, actually fine via `clipper-z-sys`/`cz_clip_extrusion`) and
  measured: native's floating filler does NOT prune bead material (both engines emit the SAME WallToolPaths
  beads); `detect_floating_line`/`resplit_order_loops` only re-tag/re-seed. The split is a DOWNSTREAM
  consequence of rust **over-fragmenting the narrow-solid fill regions ~3× (FLOATCLASS_DBG: rust 4237 fragments
  vs native 1369)** — the same surface-classification/slicing fragmentation lineage — which `detect_narrow`
  then classifies differently against `lower_internal_areas`. The faithful floating port is a real fidelity win
  (genuine floating detection + seam, deretraction-prime 186→138) and material-neutral / no-regression, landed.
  **The real lever for floating→170 AND ISI→494 is reducing narrow-solid fragmentation (group_fills surface
  merge / the slicer fragment lineage), upstream of the fill stage.** See `docs/parity/R69_floating.md`.
  REMAINING per-feature levers: **sparse +68** (biggest single material gap, pre-existing), **bridge +17**
  (pre-existing). Walls + gap-fill at parity (untouched by the infill changes).
- **R70–R73 — THE FRAGMENTATION THEORY WAS DISPROVEN FOR MATERIAL (a multi-round investigation, banked).**
  R70/R71 traced the ISI/floating/sparse material gap to rust over-fragmenting `fill_surfaces` ~2–3× (R71:
  the explosion is inside `process_external_surfaces` — shells enter clean at 404, exit at 1929 vs native 605).
  We funded the faithful fix to de-fragment: built a reusable **Clipper2-Z engine shim** (`crates/clipper2-z-sys`,
  vendored Clipper2 + USINGZ, ODR-namespaced + symbol/full-link verified), ported the faithful `wave_seeds`
  (R72), and the faithful Miter/ClipperLib **closing** (R73, which collapses the fragmentation 1911→707).
  **BUT both-engine A/B proved the surface FRAGMENT COUNT does NOT drive the material**: the closing fix moves
  the gcode by ~0 (ISI/floating/sparse unchanged) because the fill is computed on the **unioned area** (already
  identical between engines, 404 entering, area matches). So R69–R73's "fragmentation is the lever" framing is
  a RED HERRING for material. (Three roots refuted by cheap assess-first measurement before any expensive fix
  shipped: wave_seeds-approx→units-bug-artifact, F1-difference→clib made it worse, fragment-count→gcode~0.)
  **CORRECTED next-session target:** the real ISI −23 / sparse +68 / floating +32 material gap is DOWNSTREAM in
  **FILL-PATH GENERATION on the (already-correct, unioned) surfaces** — `group_fills` + the grid/concentric/
  floating emitters — NOT in process_external surface classification. Fresh both-engine localization needed
  (no current hypothesis). The Clipper2-Z shim + faithful wave_seeds + faithful closing are PRESERVED gated on
  branch `wave-seeds` (pushed, no-regression, env-gated REGION_EXPANSION_FAITHFUL/CLOSE_CLIB) — correct fidelity
  foundations to revive when the real material lever is found. Docs: `docs/parity/R70_sparse.md`,
  `R71_defrag.md`, `R72_wave_seeds.md`, `R73`.
- **THE ARACHNE PIPELINE IS NOW LIVE (R67/R68).** The keystone `SkeletalTrapezoidation` VD→half-edge graph
  builder (`construct_from_polygons` + make_node/transfer_edge/discretize/compute_point_cell_range, ~415
  lines, SkeletalTrapezoidation.cpp:92-504) was ported against the `bv::Diagram` index API and wired into
  `WallToolPaths::generate` (replacing the stub). This unblocks the ENTIRE Arachne pipeline (concentric
  infill here + the Arachne perimeter path elsewhere). Two latent bugs fixed (surfaced now the graph is
  non-empty): `collapse_small_edges` use-after-free (LinkedList rebuild moved payloads → dangled the
  raw-pointer graph; fix = `LinkedList<Box<STHalfEdge>>` for stable payload addresses) + `generate_junctions`
  size_t underflow. See `docs/parity/R67_arachne.md`.
- **Time estimate: CONVERGED** — native 43m0s / rust 43m21s (the old "1h29m vs 43m"
  line-3 divergence is **RESOLVED**; the overhang/speed trio is landed).
- **Byte-identical: NO**, but the structural subsystems now match or are faithful:
  outer-wall vertex density matches native (offset rerouted to clipper-z-sys),
  gap-fill 81% closed, seam ~89% byte-exact at established layers, arc-fitter /
  simplification / medial-axis (boostvoronoi) / chaining / retraction / overhang-trio
  all proven faithful. `lslices` and `detect_surfaces_type` are faithful **given inputs**; the mesh slicer
  was NOT bit-faithful at the Benchy hull bottom (R62's "slicer faithful" claim was overturned by R63's
  both-engine A/B) — **R65 ROOT-CAUSED + FIXED it** (the f32 center round-trip below; floor now slices
  exactly as C++).
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
  (exact-f32 z==slice_z) — VINDICATES R61. NO faithful fill-stage fix converges it. **R64 took the decisive
  raw-loops-before-union measurement (both engines, branch `f2-rawloops`): at z=0.3/li=1 C++ `make_loops`
  emits 1 clean loop, rust emits 10 (outer 545.95mm² byte-identical + 8 phantom holes ~87mm²). The UNION is
  EXONERATED (the split is in the raw loops); the bug is F2 `slice_facet`/`make_loops` on-plane cap-facet
  classification — rust's cavity closes one slice late.** **R65 ROOT-CAUSED AND FIXED (LANDED, this branch).**
  Both-engine F2TRAFO dump showed the divergence is a COORDINATE-FRAME / f32-precision gap, not the facet
  logic: C++ stores the ModelVolume mesh **f32-centered** on its bbox and re-places it via the instance
  trafo (`trafo_centered() * volume.get_matrix()`, PrintObjectSlice.cpp:60 — identity + Z translate of
  exactly +24 for the Benchy). That f32 round trip QUANTIZES geometry sitting exactly on a layer-midpoint
  slice plane OFF the plane (f32 is ~21 ULPs coarser near center_z=24): the cabin floor f32(0.3)=0.300000012
  → `f32(f32(0.3−24)+24)` = 0.299999237 (7.75e-7 below slice plane zs[1]) → clean floor. Rust stores
  vertices f64 and only casts to f32 at slice time, so it kept exact f32(0.3) == slice_z → bit-coincident →
  degenerate. **FIX = `TriangleMesh::quantize_f32_center_roundtrip()`** (bakes the f32 round trip into the
  f64 vertices before slicing; app_slice.rs). RESULT (clean A/B): li=1 raw loops **10→1** (matches C++);
  layer-3 cabin-floor ISI **13.07→38.19** (native 38.08, parity); outer-wall G1 **+212→+158** and inner
  **+578→+554** (toward native); wall+ISI material all toward native; time 42m49s; gap-fill parity; build
  green; NO guardrail regression (outer-wall G1 IMPROVED — opposite of the rejected R59 slicer-fix at +371).
  The cabin-floor cascade is resolved; the REMAINING ISI deficit (−76) is the SEPARATE distributed
  **FillConcentric no-boundary-loop starvation** (body+top narrow vertical-shell strips) — that is the next
  lever. See `docs/parity/R65_floor_z.md`, R63/R64 docs, and the R63-R65 round-log.

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
