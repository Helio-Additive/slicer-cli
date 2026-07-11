# C++ → Rust Porting Patterns (byte-parity fidelity classes)

Modeled on the `PORTING.md` pattern-mapping approach from Bun's Zig→Rust rewrite
(https://bun.com/blog/bun-in-rust): every recurring semantic mismatch discovered
by the two-engine byte-compare campaign (R100–R142, `PARITY_STATUS.md`) is
documented here as a mechanical mapping, so new code never reintroduces a class
and audits can grep for anti-patterns instead of rediscovering them by bisection.

The oracle is stronger than a test suite: the native binary itself, byte-compared
(`compare --config tests/configs/stl-inline-config.jsonnet`, content-multiset-
unmatched as the primary metric). LOCALIZE BEFORE FIXING: dump matched
intermediates in both engines and find the FIRST diverging stage; "FP noise /
library limit / floor" verdicts have been refuted 9 times.

## Class 1 — geo-clipper 1µm gridding (≥16 confirmed hits; the dominant class)
`geo-clipper` runs at `GEO_CLIPPER_SCALE = 1000` (1µm); native ClipperLib runs at
1e5 (10nm). EVERY geo boolean/offset silently snaps output vertices to a 1µm grid.
- ANTI-PATTERN: `union_ex / union_polygons_ex / intersection / difference /
  offset_expolygons / offset2 / grow / shrink / closing / opening_ex` (geo
  variants) anywhere on a byte-relevant geometry path.
- MAPPING: route through the clib shims — `union_ex_clib(polys, 1 /*NonZero*/)`,
  `intersection_clib`, `difference_clib`, `offset_expolygons_clib`,
  `offset2_ex_clib`, `shrink_clib`/`grow_clib`, `union_safety_offset_ex_clib`.
- MIXED-GRID COROLLARY (R74, R142): never diff/intersect a full-res operand
  against a gridded one — it fragments bands into slivers. Un-grid whole chains,
  not single ops.

## Class 2 — ApplySafetyOffset semantics (R114/R116/R139)
Native `diff_ex(a, b, ApplySafetyOffset::Yes)` applies a RAW +10-scaled-unit
ClipperOffset (jtMiter/ML3, orientation-aware, NO union/reconstruction) to the
CLIP paths before ctDifference; `union_safety_offset_ex` safety-offsets the
SUBJECT before the two-pass NonZero union.
- ANTI-PATTERN: ignoring the `_safety_offset` argument (the geo wrappers did,
  silently, at every call site) or emulating ::Yes via union→offset→diff.
- MAPPING: `difference_clib_safety` / `union_safety_offset_ex_clib`
  (shims `cz_difference_closed_safety`, `cz_union_ex_safety`).

## Class 3 — scale_ / coordinate-cast semantics (R101/R109/R117/R122-125/R140)
Four DISTINCT native behaviors; do not conflate:
| Native construct | Behavior | Rust mapping |
|---|---|---|
| `scale_(v)` = `v / SCALING_FACTOR` (f64 literal 0.00001, slightly > 1e-5) then `coord_t()` | divide-then-TRUNCATE; `0.5→49999`, `0.42→41999` | `scale_faithful(mm) = trunc(f32(mm)/0.00001)` (clipper_utils); NEVER `mm * 1e5` for native-matching deltas |
| `Point(double,double)` | `lrint` = ROUND-to-nearest, ties-even | `round_ties_even()`; NEVER `as i64` (truncate) and NEVER `.round()` (half-away-from-zero) |
| Eigen `.cast<coord_t>()` | static_cast = TRUNCATE | `as i64` is CORRECT here — most rust casts are this class; audit before "fixing" |
| `offset(paths, float delta)` | f64→f32 TRUNCATION at the call boundary | truncate deltas through f32 before use (`((d as f32) as f64)`) |
Flow widths are stored f32 (`Flow::m_width`); scaled widths inherit both the f32
store and the scale_ truncation (`0.42 → f32 0.41999998 → 41999`).

## Class 4 — DP simplification (R122, R140)
Native `douglas_peucker` uses the PURE-DOUBLE perpendicular distance and a SCALED
tolerance (`m_scaled_resolution`).
- ANTI-PATTERN: crate `Line::distance_to_squared` (integer-ROUNDED projection —
  masks tolerance bugs by accidentally deleting near-collinear points) fed an
  UNSCALED mm tolerance (≈1e5× too small, near no-op).
- MAPPING: `simplify_p_dp_rings_faithful(tolerance_scaled)` /
  `douglas_peucker_faithful`; convert mm→scaled via `/0.00001`.

## Class 5 — op-structure substitutions (R88/R97/R111/R119)
Byte-parity requires the SAME operation structure, not an equivalent one:
- `union_ex` is TWO passes (flat-Paths union, then a second union into a
  PolyTree); single-pass PolyTree union gives different vertices (R88).
- PolyTree→ExPolygons emits each contour's holes BEFORE recursing into nested
  contours (R97).
- `A ∩ B` must be single-op ctIntersection; `A − (A − B)` inserts near-collinear
  vertices on ~2/3 of real inputs (R119; the "proven equal on one layer"
  generalization trap).
- `chain_points` = KD-tree greedy `chain_segments_greedy`, not NN-from-index-0
  (R111 — island ORDER depends on it).
- Result-STRUCT merges: when adding a field to a per-surface result, grep every
  merge site; rust merges silently drop new fields (R141 top_band).

## Class 6 — fill emitters are coupled clusters (R113-R120, R134-R141)
Angle, spacing adjust, raster anchor, fill area, and band cover are ONE cluster
(`FillMonotonicLineWGapFill` etc.). Landing any subset churns; land whole
clusters behind one gate with per-stage byte oracles (dump matched intermediates
at each stage on a chosen layer). Specifics already fixed: `_infill_direction`
adds an UNCONDITIONAL +90° and the per-layer alternation is parity `(idx&1)`,
mod-360 (not `90·idx` mod-180); top-surface MonotonicLine sets
`dont_adjust=true`; the align_to_grid refpt is exactly (0,0) (origin-centered
object bbox); rasters run over `no_overlap_expolygons`.

## Class 7 — output formatters are fidelity surfaces (R160)
A correct VALUE behind a lossy output format is still a 100% byte mismatch.
`writer.set_speed` formatted F as `{:.0}` while native `GCodeG1Formatter::emit_f`
prints 3 decimals with trailing zeros/dot trimmed — every fractional-F line
(smooth ramps, overhang interpolation, volumetric caps) was unmatchable even
where the computed speed was already byte-correct; fixing the FORMAT alone
closed 16.3k lines (the largest single win of the campaign).
- ANTI-PATTERN: any `format!("{:.0}")`/rounded emission on a value native prints
  with `XYZF_EXPORT_DIGITS`/`m_gcode_precision_xyz`-style precision.
- MAPPING: replicate the native formatter constants exactly (3-decimal + trim
  for F; check E/XYZ precisions against GCodeFormatter) BEFORE chasing value
  divergence — cheap to audit, catastrophic to miss.
- COROLLARY: post-processors may RE-format some lines (native cooling rewrites
  slowed F as `int(floor(60·f+0.5))`) — match the format PER EMISSION SITE.

## Process rules (what actually worked for 40+ rounds)
1. Byte-locked default: every output-changing fix behind an env gate; default
   output checksum-verified after EVERY change (147987 lines / sha 7adae05c).
2. Dual-engine stage dumps, reverted from BOTH trees before landing.
3. One oracle layer per subsystem (e.g. L23 for top-fill), stage-split to the
   first diverging intermediate, then walk up.
4. Determinism check: two runs must be byte-identical.
5. Treat remaining divergence as a work queue grouped by CLASS (grep the
   anti-patterns above), not only by feature.
