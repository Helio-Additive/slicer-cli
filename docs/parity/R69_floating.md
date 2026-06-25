# R69 — Faithful FillFloatingConcentric LANDED; the floating "+34" is upstream narrow-solid fragmentation, NOT a filler problem

Branch: `floating-fill` (off the parity tip `alex/libslic3r-parity-engine` @ b84c8d8).
Job: Benchy, `tests/configs/stl-inline-config.jsonnet`.
Metric: `/tmp/feat_e2.py` (per-`;FEATURE` E over XY-moving extrusion).

## TL;DR

Lever (a) was "port the faithful FillFloatingConcentric (Z-clipper) to prune the
floating +34 interim overshoot to native (~170)." **The whole faithful port is
done, builds green, and runs end-to-end** (the Z-clipper, has_intersecting_edges,
detect_floating_line, resplit_order_loops, and FillFloatingConcentric are all
ported and wired). But it **does NOT move the floating material** (204.76 → 202.66,
native 170.76) — which **disproves the premise**: native's
`detect_floating_line`/`resplit_order_loops` do NOT prune bead material, they only
re-tag floating segments and re-seed the loop start. Both engines emit the same
WallToolPaths bead set per region.

Both-engine instrumentation (FLOATCLASS_DBG at `detect_narrow_internal_solid_infill`)
pins the real root: rust's narrow-solid fill regions are **~3× more fragmented**
than native's, and the Concentric-vs-FloatingConcentric split lands very
differently:

| narrow-solid classification | NATIVE (C++) | RUST |
|-----------------------------|--------------|------|
| floating area               | 6534 mm²     | 2723 mm² |
| internal (plain) area       | 4480 mm²     | 6989 mm² |
| n_float fragments           | 387          | 234  |
| n_internal fragments        | 982          | **4003** |
| total narrow fragments      | 1369         | 4237 |

Rust splits the same narrow-solid area into **4237 fragments vs native's 1369**.
The per-feature E split (floating +32, ISI −23) is a downstream consequence of this
fragmentation + the resulting classification, NOT the FloatingConcentric filler.
The COMBINED narrow-solid material (ISI + floating) is near parity: native 664.84 /
rust 673.68 = **+8.84**.

## What landed (faithful, committed on `floating-fill`, all pushed)

1. `6e623cd` — `cz_detect_floating` shim (clipper-z-sys) + `clipper_z::detect_floating`
   wrapper: the detect_floating_line Z-clipper (ClipperLib_Z, dual ctIntersection/
   ctDifference under the negative-hash ZFillFunction). The Z-clipper was NOT
   blocked — ClipperLib_Z + ZFillFunction was already proven by `cz_clip_extrusion`;
   this is a second filler with detect_floating_line's hash semantics. (The "blocked
   on Clipper2, no Z-aware clipper" notes in fill_floating_concentric.rs:905 and
   overhang_detector.rs:381 were STALE.)
2. `9b686f0` — `EdgeGrid::has_intersecting_edges` (EdgeGrid.cpp:1452).
3. `a807fa3` — `detect_floating_line` (FillFloatingConcentric.cpp:389-489).
4. `2db0b2f` — `FillFloatingConcentric` struct + resplit_order_loops +
   _fill_surface_single + fill_surface_arachne_floating + fill_surface_extrusion
   (FillFloatingConcentric.cpp:682-1000) + FloatingThickPolyline::clip_end/is_valid.
5. `3c1467b` — wire FloatingConcentric → FillFloatingConcentric in make_fills, with
   lower_layer_unsupport_areas = shrink_ex(union of lower stInternal/stInternalVoid)
   and lower_sparse_polys = union_(offset(lower
   generate_sparse_infill_polylines_for_anchoring, internal_infill_width/2)),
   threaded from the print_object.rs caller.

## Per-feature (native / rust)

| FEATURE | native | before (interim) | NOW (faithful) | dE |
|---|---|---|---|---|
| Internal solid infill | 494.08 | 471.05 | 471.02 | −23.06 |
| Floating vertical shell | 170.76 | 204.76 | 202.66 | +31.90 |
| Sparse infill | 531.63 | 599.85 | 599.85 | +68.22 |
| Outer wall | 1003.08 | 1005.21 | 1005.21 | +2.13 (guardrail OK) |
| Inner wall | 995.23 | 997.98 | 997.98 | +2.75 (guardrail OK) |
| Gap infill | 230.58 | 230.79 | 230.79 | +0.20 (guardrail OK) |
| TOTAL | 3858.97 | 3975.57 | 3973.93 | +114.96 |

The faithful port is material-neutral vs the interim (floating −2.1) but CORRECT:
it produces real floating-segment detection + seam re-seeding (floating prime
deretraction dropped 175 → 138). No guardrail regression; ISI/walls/gap unchanged.

## Why the premise was wrong (the disproof)

`floating_thick_polyline_to_extrusion_paths` (FillFloatingConcentric.cpp:61-203,
already ported) splits the thick polyline at floating/non-floating transitions and
tags the floating runs with `CustomizeFlag::FloatingVerticalShell` — it emits ALL
segments, dropping none. `resplit_order_loops` only re-orders loops and picks a
better loop start (get_best_loop_start) for the floating regions. So the faithful
floating path produces the SAME WallToolPaths bead material as the plain-concentric
interim — confirmed: 204.76 → 202.66 (−2.1). There is no bead pruning to recover
the +32.

## The real root (the next lever) — upstream narrow-solid fragmentation

Rust produces 4237 narrow-solid fragments vs native's 1369 (~3×). This is the SAME
surface-classification / slicing-fidelity lineage tracked in earlier rounds (the
fill_surfaces are over-fragmented). `detect_narrow_internal_solid_infill`
(fill/mod.rs:777, faithful) then classifies each fragment Concentric vs
FloatingConcentric by whether it overlaps `lower_internal_areas` (the lower layer's
stInternal/stInternalVoid). With 3× more, smaller fragments, rust's overlap test
lands differently — far more fragments fall on the plain-Concentric side (internal
6989 mm² vs native 4480), yet rust's floating filler still produces more material
per classified area.

NEXT LEVER (to actually move floating → 170): reduce the narrow-solid fragmentation
so rust's fill_surfaces match native's fragment count/areas, which will realign the
Concentric/FloatingConcentric split. This is upstream of the fill stage (surface
classification / make_fills `group_fills` surface merging, or the slicer fragment
lineage) — NOT the FloatingConcentric filler (now faithful and correct).

## Verdict

The faithful FillFloatingConcentric (and its full Z-clipper + EdgeGrid +
detect_floating_line dependency chain) is LANDED, correct, and a real fidelity
improvement (it unblocks the last blocked floating symbols and produces correct
floating-segment detection). It does not close the floating +34, because that
overshoot is an upstream fragmentation/classification artifact, not a filler issue —
disproven by direct measurement. No regression; guardrails intact. Recommend landing
the faithful port and redirecting the floating-parity effort to the narrow-solid
fragmentation root.
