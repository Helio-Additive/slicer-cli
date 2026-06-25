# R70 — Sparse +68 is NOT a density/spacing issue; it's the SAME fill-surface fragmentation root as the floating/ISI split (R69)

Branch: `sparse-fill` (off parity tip `alex/libslic3r-parity-engine` @ e8445e7).
Job: Benchy, `tests/configs/stl-inline-config.jsonnet`.
Metric: `/tmp/feat_e2.py` + targeted both-engine instrumentation.

## TL;DR / verdict (diagnosis-only; no code change; clean)

Lever (b): sparse infill over-produces — native 531.63 / rust 599.85 (dE +68.22).
**Pinned with both-engine instrumentation: this is NOT density, spacing, or pattern.
It is the SAME upstream fill-surface FRAGMENTATION that drives the floating/ISI split
(R69).** Rust carves the internal-infill region into ~1.8–2.3× more fragments than
native; grid infill on many small fragments lays more line length per unit area
(boundary losses: partial lines + per-fragment anchoring) → +12.6% sparse line length
→ +68 E. No fill-stage fix applies; the lever is reducing fragmentation upstream of
the fill stage. No code changed; both trees clean; C++ instrumentation reverted+rebuilt.

## The measurements (decisive)

### 1. E/mm is identical — rules out density/flow/spacing

| sparse | native | rust |
|--------|--------|------|
| XY length | 16019 mm | 18036 mm (**+12.6%**) |
| E | 531.63 | 599.85 |
| **E/mm** | **0.03319** | **0.03326** (+0.2%) |

The per-mm extrusion rate matches to 0.2%. Config matches exactly
(sparse_infill_density=15%, pattern=grid, infill_direction=45). So the overshoot is
purely **+12.6% more sparse line length** — i.e. more line laid over essentially the
same area.

### 2. Sparse fill AREA is the same (even slightly less in rust) — rules out over-classification of sparse

At the fill stage (per density=15 SurfaceFill, summed over all layers):

| | native | rust |
|---|---|---|
| sparse fill area | 37619 mm² | 36901 mm² (−2%) |
| sparse SurfaceFills | **6** | **120** (20×) |
| sparse expolygons | **201** | **472** (2.3×) |

Same sparse area, but rust splits it into 20× more SurfaceFill groups / 2.3× more
expolygons.

### 3. The fragmentation is already present in `fill_surfaces` entering `group_fills`

stInternal surfaces entering group_fills (summed over all layers):

| | native | rust |
|---|---|---|
| n stInternal surfaces | **572** | **1004** (+76%) |
| stInternal area | 100005 mm² | 80298 mm² (−20%) |

Rust enters group_fills with **76% more stInternal fragments** (and slightly LESS
area — so it is NOT over-classifying area as internal; it is over-FRAGMENTING the
internal area). group_fills then groups native's 572 into 6 sparse SurfaceFills but
rust's 1004 into 120 — the fragmentation survives grouping and reaches the grid
filler as many small islands.

## Why fragments inflate grid line length

A grid/rectilinear filler on one large region lays long, efficient parallel lines.
Split the same area into N small islands and each island independently rasterizes the
grid against its own bbox/clip: partial boundary lines, per-island infill-direction
reference, and connect_infill anchor segments along each island's perimeter all add
line length that the single-region case amortizes away. 2.3× more islands → +12.6%
length at equal area and equal E/mm. (The grid line E/mm is unchanged because spacing
is set by density, which is identical.)

## This is ONE root with lever (a)

R69 found the floating/ISI per-feature split is driven by rust producing ~3× more
narrow-solid fragments (4237 vs 1369). R70 finds sparse +68 is driven by rust
producing ~1.8× more stInternal fragments (1004 vs 572). **Same defect: rust's
`fill_surfaces` are fragmented far more than native's.** Fixing the fragmentation
would move BOTH the floating/ISI split AND the sparse over toward parity in one
change — it is the single highest-value remaining infill lever.

## Where the fragmentation comes from (next step, not taken)

The fragmentation is present in `layerm.fill_surfaces` BEFORE group_fills, so it
originates upstream in the surface pipeline: the slices → fill_surfaces lineage and/or
the surface-classification stages (detect_surfaces_type / discover_*_shells /
process_external_surfaces / prepare_fill_surfaces), which split a region's internal
area into many typed surfaces. This is the same surface-classification/slicing lineage
documented in rounds 1-65 (the long-standing over-fragmentation). NEXT: instrument the
fill_surfaces fragment count per stage (raw slices → detect_surfaces_type →
process_external → entering group_fills) on both engines to localize WHICH stage
introduces the extra splits, then fix that stage faithfully (likely a missing
union/merge of co-typed adjacent surfaces, or a clipper-precision over-segmentation).

## Verdict

Sparse +68 is a measured consequence of upstream fill-surface fragmentation (rust
1004 vs native 572 stInternal surfaces), NOT a fill-stage density/spacing/pattern
issue (E/mm and config are identical; area matches). It shares its root with the
lever-(a) floating/ISI split. No fill-stage change can fix it; the actionable lever is
de-fragmenting `fill_surfaces` upstream, which would address (a) and (b) together.
Recommend redirecting to the fill-surface fragmentation root (per-stage fragment-count
localization) rather than a fill-stage sparse patch.
