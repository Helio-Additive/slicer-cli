# R72 — Faithful wave_seeds (Clipper2-Z) ported: seed generation FIXES the solid over-fragmentation (1929→628≈605); wavefront propagation over-expands (gated, not default)

Branch: `wave-seeds` (off parity tip `alex/libslic3r-parity-engine` @ 82b9521).
Job: Benchy, `tests/configs/stl-inline-config.jsonnet`.

## TL;DR

The R71 root — `process_external_surfaces` over-fragmenting the solid zone (rust
1929 vs native 605) because `wave_seeds` was a non-faithful polygon approximation —
is **resolved at the seed-generation level**. I built a Clipper2-Z shim (USINGZ,
ODR-isolated) and ported the faithful `wave_seeds` chain (RegionExpansion.cpp:108-389)
against it. DECISIVE A/B (DEFRAG_DBG):

| process_external_surfaces | n_solid | n_internal |
|---------------------------|---------|------------|
| native (C++)              | **605** | 169 |
| rust legacy (polygon approx) | 1929 | 367 |
| rust FAITHFUL (Clipper2-Z wave_seeds) | **628** | 367 |

The faithful seed generation collapses the solid fragmentation **1929 → 628**, almost
exactly native's 605. The keystone defect is fixed for the solid zone.

BUT it is NOT yet landable as default, for two reasons, so it is gated behind
`REGION_EXPANSION_FAITHFUL=1` (default keeps the legacy approximation — no material
regression):
1. The wavefront PROPAGATION (Phase 2c, geo-clipper open-round offset) **over-expands**
   material vs C++'s ClipperOffset.
2. The sparse/internal zone is **unaffected** (n_internal still 367 vs native 169).

## What landed (committed on `wave-seeds`, pushed)

PHASE 1 — `crates/clipper2-z-sys` (commit 65d62d6): vendored Clipper2 v1.5.4 built
`-DUSINGZ`, namespace-renamed `Clipper2Lib → Clipper2ZSys` for ODR isolation from
clipper2c-sys's non-Z Clipper2 (symbol-verified: 2685 Clipper2ZSys::, 0 Clipper2Lib::).
Two C-ABI fns matching wave_seeds' ops: `cz2_offset_z` (Z-preserving ClipperOffset)
and `cz2_intersect_open_z` (Clipper64 + SetZCallback = the Clipper2ZIntersectionVisitor
+ AddOpenSubject + Execute returning closed+open Z-segments + the intersections table).
3 green standalone tests; links cleanly alongside clipper2c-sys.

PHASE 2 (b330ebf, 6657677, 739090f): ported RegionExpansion.cpp:108-465 into
region_expansion.rs against the shim —
- `expolygons_to_zpaths64` / `_expanded_opened`, `merge_splits` + `polylines_merge_z`,
  `wave_seeds_faithful` (the Z intersection + intersections-table lookup + AABB
  boundary sampling), `WaveSeed.path`.
- `offset_polylines_round` (geo-clipper etOpenRound/etClosedLine) + `wavefront_clip`
  + `propagate_wave_from_boundary` + `propagate_waves_from_seeds`.
- `propagate_waves` re-routed (gated `REGION_EXPANSION_FAITHFUL`).

## The two remaining gaps (where the over-expansion comes from)

### 1. Wavefront propagation over-expands (the material regression)

Per-feature, faithful vs native (REGION_EXPANSION_FAITHFUL=1):

| FEATURE | native | legacy | FAITHFUL |
|---|---|---|---|
| Internal solid infill | 494.08 | 470.98 (−23) | **533.18 (+39 OVER)** |
| Floating vertical shell | 170.76 | 202.69 (+32) | 213.57 (+43) |
| combined ISI+floating | 664.84 | 673.7 (+8.8) | **746.8 (+81.9 OVER)** |
| Sparse infill | 531.63 | 599.85 | 599.85 (unchanged) |

The faithful path's bigger, merged solid surfaces (628 ≈ 605) now carry MORE ISI
material than native's 494 — the wavefront propagation expands the (correctly merged)
solid zone ~12% too far. Likely causes (next debug): (a) `wavefront_clip` unions ALL
offset waves before clipping (vs C++'s per-step pftPositive), (b) geo-clipper's round
offset over-shoots vs ClipperLib's `ShortestEdgeLength`-decimated offset (geo-clipper
does not expose ShortestEdgeLength), or (c) the open-round seed cap inflation extends
past C++'s. The seed COUNT is right (628), so it's the propagation step geometry, not
the seeds.

### 2. The sparse zone is untouched

n_internal stays 367 (vs native 169). `process_external_surfaces` carves the SPARSE
zone (`expansion_zones[1]`) by `difference` too, and the wave_seeds fix only addressed
the SOLID zone's `expand_merge_surfaces` expansions. The sparse fragmentation has a
separate source (the difference-carving of the sparse zone, or a missing union of the
sparse zone before emission) — so sparse +68 needs its own follow-up even after the
propagation is fixed.

## Verdict

The keystone is HALF-LANDED and proven: the Clipper2-Z `wave_seeds` is faithful and
collapses the solid over-fragmentation 1929→628 (native 605). Two bounded gaps remain
before it can be the default: (1) the wavefront-propagation over-expansion (a
geo-clipper-offset fidelity gap, +81 combined material — the immediate next step), and
(2) the sparse-zone fragmentation (separate, untouched by the solid wave fix). The
faithful path is gated `REGION_EXPANSION_FAITHFUL=1`; the default is unchanged (no
regression). The Clipper2-Z shim (Phase 1) is a clean, reusable, ODR-safe foundation
regardless.

RECOMMENDATION: keep the gated faithful path; next debug the wavefront over-expansion
(instrument the per-step wave area faithful-vs-native on a sample layer) — that closes
the ISI/floating gap. The sparse-zone fragmentation is a separate follow-up.
