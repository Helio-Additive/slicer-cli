# R78 — Byte-parity scope (diagnosis-only): the first structural blocker is ISLAND-based extrusion ordering (GCode.cpp:4340-4392)

Branch: `byteparity-scope` (off parity @2efc482, no commits — diagnosis only).
Job: Benchy, `tests/configs/stl-inline-config.jsonnet`. native.gcode vs rust.gcode.
Material is at native aggregate (0.99998×); this scopes the path toward BYTE-identical.

## TL;DR

The g-code streams are NOT 1:1 alignable past the header, and the FIRST + most
gating structural divergence is **entity emission ORDER**: C++ groups a layer's
extrusions by **ISLAND** (GCode.cpp:4340-4392 — slices sorted by bbox size, each
ExtrusionEntityCollection assigned to its island, then per-island perimeters→infill),
whereas rust emits **flat per-REGION** (print.rs:596 `for region in layer.regions()`
→ all of a region's perimeters then all its infill). Everything downstream (seam,
arcs, coords) is unalignable until this is fixed. **Recommended next lever: port the
island-grouped layer emission (GCode.cpp `ObjectByExtruder`/island map).**

## First structural divergence (after the header)

The header diffs are config-comment formatting (time/total lines, `0.3,0.5` vs
`0.3x0.5`, `100%` vs `100`) — not toolpath. The first TOOLPATH divergence is at
layer 1's wall emission:
- native: `; FEATURE: Outer wall` → Inner → Outer → Inner …
- rust:   `; FEATURE: Inner wall` → Outer → Inner → Outer …

…and the per-feature block grouping diverges immediately after.

## Per-candidate numbers

### ORDER (the root) — island grouping
- FEATURE-block counts (emission blocks per role):
  | role | native | rust |
  |---|---|---|
  | Gap infill | **816** | **279** |
  | Outer wall | 772 | 746 |
  | Inner wall | 667 | 667 |
  | Internal solid infill | **389** | **254** |
  | Sparse infill | 193 | 187 |
  Gap-fill: native 816 ≈ its 772 outer-wall blocks → native interleaves gap-fill
  **per-island, between perimeters**; rust batches it (279). ISI 389 vs 254 likewise.
  This is the island-vs-region grouping signature.
- Per-layer FIRST wall: native 202 inner-first / 38 outer-first; rust 218 / 22.
  Both mostly inner-first (config `wall_sequence = inner wall/outer wall`), but the
  per-island first-wall choice + per-loop interleave differ (~16-layer gap + within-
  layer reversal), a downstream symptom of the island ordering, not a separate knob.

### SEAM (gated by order — not independently measurable yet)
- Outer-wall loop START points, index-aligned: 0/746 exact (<1µm), 12 close (<0.5mm).
- The 0% is because the loops aren't 1:1 (772 native vs 746 rust, different order).
  Seam fidelity CANNOT be measured until the entity order + loop counts converge.
  Seam is a follow-on lever, not the first one.

### ARCS (separate, later lever)
- G2/G3 extruding arcs: native 10164 vs rust 11013 (+8.4%); total G2/G3 11906 vs
  14949 (+26%). A real arc-fit divergence, but it does not gate alignment the way
  order does — defer until order+seam land.

### COORDINATES (F1, deepest)
- Not separately measured (unalignable streams). The geo-clipper scale-1000 vs
  ClipperLib 1e5 coordinate-exactness is the final byte lever after order/seam/arcs.

## Root (code)

- C++ `GCode::process_layer` (GCode.cpp:4340-4392): builds a per-layer
  `by_region`/`ObjectByExtruder::Island` map — `layer.lslices` sorted by bbox area
  (`slices_test_order`), each EEC assigned to the island whose contour contains its
  first point; then per island, per extruder, emits perimeters then infill
  (`extrude_perimeters`/`extrude_infill`), unless `infill_first`.
- rust `print.rs:596`: flat `for region in ltp.layer.regions()` →
  `extrude_perimeters(region)` then `extrude_infill(region)`. No island assignment,
  no bbox-sorted slice traversal → entities come out region-grouped, not
  island-grouped → different order, different gap/ISI interleave.

## Recommended next lever (tractability read)

**Port the island-grouped layer emission** (GCode.cpp:4340-4392 + the
`ObjectByExtruder::Island::Region` map). It is:
- The FIRST divergence in every layer's toolpath → it GATES byte-diffing everything
  after it (seam, arcs, coords can't be aligned until entities are in native order).
- Self-contained to the gcode-export layer loop (print.rs:596 + exporter
  extrude_perimeters/extrude_infill already exist) — it's a re-ordering port, not a
  geometry change, so it should be MATERIAL-NEUTRAL (same extrusions, different
  emission order) — low regression risk, and directly verifiable by the FEATURE-block
  counts converging (gap 279→816, ISI 254→389) and the first-wall pattern matching.
- Tractable: the pieces (lslices, bboxes, per-region perimeters/fills EECs, the
  point-in-island test) all exist in rust; the work is the bbox-sorted island
  assignment + the per-island emit loop.

After island ordering lands, SEAM becomes measurable (then fixable), then ARCS, then
F1 coordinates — the rough dependency order from PARITY_STATUS.

## State

Diagnosis-only: no code change, branch `byteparity-scope` clean (0 commits), default
unchanged, both trees clean, build green.
