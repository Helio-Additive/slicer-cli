# R71 — Fill-surface over-fragmentation localized to process_external_surfaces; root is the BLOCKED wave_seeds Clipper2-Z backend (foundational, but maybe unblockable via clipper-z-sys)

Branch: `defrag-surfaces` (off parity tip `alex/libslic3r-parity-engine` @ c2acf5c).
Job: Benchy, `tests/configs/stl-inline-config.jsonnet`.
Method: per-stage fill_surface fragment-count, both engines (DEFRAG_DBG / DEFRAG2_DBG, all reverted).

## TL;DR / verdict (diagnosis + scope; no code change)

The fill-surface over-fragmentation that drives BOTH lever (a) (floating/ISI split,
R69) and lever (b) (sparse +68, R70) is introduced in **`process_external_surfaces`**.
Per-stage tally (stInternal `n_int` / stInternalSolid `n_solid`, summed over all
layers, native vs rust):

| stage | native n_int | rust n_int | native n_solid | rust n_solid |
|-------|--------------|------------|----------------|--------------|
| detect_surfaces_type      | 648 | 657 | 0   | 0    |
| prepare_fill_surfaces     | 209 | 205 | 439 | 452  |
| discover_vertical_shells  | 388 | **651** | 692 | 587 |
| **process_external_surfaces** | **169** | **367** | **605** | **1929** |
| discover_horizontal_shells| 169 | 367 | 605 | 1929 |

Two stages diverge; `process_external_surfaces` is the dominant one:
- C++ `process_external_surfaces` MERGES/REDUCES: solid 692 → **605**.
- rust `process_external_surfaces` EXPLODES: solid 587 → **1929** (~3.2×).
- (`discover_vertical_shells` is the secondary contributor: rust internal 205→651
  vs native 209→388.)

The 1929 rust solid surfaces are GENUINELY DISJOINT, not collapsible slivers:
re-unioning the solid zone before emission changes it by only −10 (1746 → 1736 in
the per-region measurement). So this is real geometric over-fragmentation, not a
missing final union.

## Root cause — the BLOCKED wave_seeds Clipper2-Z backend (region_expansion.rs:25-56)

rust's `Layer::process_external_surfaces` → `region_expansion::process_external_surfaces_wave`
→ `expand_merge_surfaces` → `propagate_waves` → **`wave_seeds`**. The faithful
`wave_seeds()` (RegionExpansion.cpp:278-389) and its whole wave-propagation chain
(`wavefront_initial/step/clip`, the Z-tagged open-path offset `expolygons_to_zpaths64_expanded_opened`,
`merge_splits`) are **BLOCKED** on the Clipper2 *Z* engine: `Clipper2Lib_Z::Clipper64::SetZCallback`
+ a Z-preserving `ClipperOffset` for *open* paths — which the crate's `clipper2c-sys`
backend does not expose. rust falls back to `wave_seeds_polygon_based`, documented as
"NOT byte-equivalent to the C++ Z-callback path."

That approximate wave produces differently-shaped `expanded` regions; when those are
`difference`'d out of the solid/sparse zones (expand_merge_surfaces, the carve at
region_expansion.rs:637) and `closing`-rounded, the solid zone is carved into ~3×
more disjoint pieces than C++'s clean wave-expanded geometry. Hence solid 587→1929 in
rust vs 692→605 in C++.

This is the SINGLE root of both levers: the over-fragmented fill_surfaces feed
group_fills → (a) detect_narrow classifies 3× more narrow-solid fragments
(floating/ISI split) and (b) the grid filler rasterizes ~2× more sparse islands
(sparse +68).

## Tractability — foundational, but possibly unblockable via the clipper-z-sys shim

The brief asked: tractable or foundational (F1/F2/F3)? It is **foundational** — the
wave_seeds Clipper2-Z backend gap (an F1-class clipper-backend blocker). BUT it may be
unblockable with the infrastructure built for lever (a): `crates/clipper-z-sys` now
wraps **ClipperLib_Z** with a working `ZFillFunction`/SetZCallback (cz_clip_extrusion,
cz_detect_floating) and Z-aware boolean ops. wave_seeds needs:
1. a Z-tagged offset of OPEN paths (etOpenRound) preserving Z — ClipperLib_Z's
   `ClipperOffset` supports Z; the shim would need a `cz_offset_open_z` entry, AND
2. a Z-aware boolean Execute returning open + closed segments with a SetZCallback that
   records intersection provenance into an `Intersections` table (RegionExpansion.cpp:
   278-389) — the same ZFillFunction pattern cz_detect_floating already uses.

C++ uses Clipper2Lib_Z; clipper-z-sys provides ClipperLib_Z (Clipper1). The wave_seeds
algorithm is engine-agnostic in principle (both support Z-callbacks + open-path
offset), so a faithful port onto the ClipperLib_Z shim is plausible — but it is a
substantial, multi-function port (RegionExpansion.cpp:83-465: the Z-path builders,
merge_splits ×2, wave_seeds, wavefront_*) with its own verification burden. It is the
right next investment IF the team wants to close the ~115mm total infill gap to
near-zero, since it fixes (a)+(b) together.

## Scope estimate for the faithful wave_seeds port

- New shim entries: `cz_offset_open_z` (Z-preserving open-path round offset) + a
  `cz_wave_clip` (Z-callback boolean returning open+closed Z-segments). ~2 C-ABI fns,
  mirroring the cz_detect_floating pattern.
- Rust ports: expolygons_to_zpaths(64)_expanded_opened, merge_splits ×2, wave_seeds,
  wavefront_initial/step/clip, propagate_wave_from_boundary, and re-route
  propagate_waves/expand_expolygons/expand_merge_expolygons through them (RegionExpansion.cpp:
  83-465, ~380 lines).
- Then process_external_surfaces_wave already calls expand_merge_surfaces, so the
  switch is localized; re-measure the per-stage tally (target: rust solid → ~605,
  internal → ~169 at process_external) and the per-feature table (floating→170,
  sparse→531, ISI→494 all move together).

## Verdict

The over-fragmentation root is pinned to `process_external_surfaces`, caused by the
BLOCKED wave_seeds Clipper2-Z backend (the polygon approximation over-carves the solid
zone 3×). It is foundational (clipper-Z backend), shared by levers (a) and (b), and
the only change that moves floating/ISI/sparse to parity together. It is plausibly
unblockable via the clipper-z-sys ClipperLib_Z shim built for lever (a) — a bounded
but non-trivial wave_seeds port (~380 rust lines + ~2 shim fns). Recommend the team
decide whether to fund the faithful wave_seeds port; no fill-stage or surface-stage
patch fixes this without it (proven — the fragments are real, union-stable geometry
from the approximate wave). No code changed; both trees clean; instrumentation
reverted.
