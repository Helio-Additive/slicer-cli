# Semantic-equivalence parity test

Verifies that the Rust engine (`crates/libslic3r-rs`) produces G-code that is
**physically equivalent** to C++ BambuStudio — the same printed object — rather
than byte-identical text.

## Why not byte-for-byte?

Byte-identical G-code is infeasible and not the right bar. The two engines run
the same geometry math but round floating-point differently, because their
compilers (Rust's `cc` build vs C++'s `cmake`) emit different CPU instructions.
Those sub-ULP differences **cascade**: a tiny difference in an early slicing
step flips a downstream decision (which area is a bridge, where an infill line
starts), re-routing toolpaths by 10–100 µm without changing the printed part.
~99% of the raw byte-diff is exactly this cascade noise. See `PARITY_STATUS.md`
(R324–R326 prove the FP wall; R335/R346 establish the semantic approach).

The genuinely fixable differences that this campaign *did* land (bridge and
first-layer flow model bugs, −446 bytes) are the exception; the rest is the wall.

## What it checks

`scripts/semantic_compare.py <rust.gcode> <native.gcode>` prints a report and
exits **0 iff SEMANTICALLY EQUIVALENT**. The five tolerance checks:

| Check | Tolerance | Measures |
|-------|-----------|----------|
| object material | within 1% | retraction-aware deposited-E total (the part's material, excluding skirt/prime and deretraction priming) |
| layer count | equal | slice structure |
| per-layer material | mean dev < 5% | material distribution over Z |
| per-feature material | worst < 35% | catches a missing/doubled/halved feature (loose: known FP-cascade classes like vertical shell drift ~20%) |
| object silhouette | area-wtd IoU ≥ 99% | printed cross-sectional SHAPE |

**Silhouette** = region-closed all-coverage IoU: rasterize every extrusion,
morphologically close by 4mm to bridge infill/seam gaps, recovering the filled
cross-section. Robust across convex, non-convex, and curved geometry (a
sub-width wall offset barely moves it). Tunable via `SIL_RES`/`SIL_CLOSE_K`.

## Slice-time parity

Physical equivalence says the two engines print the *same object*; it says
nothing about how long each took. Slice time is tracked separately, because a
toolpath can be shape-correct yet much slower to compute. The `compare`
subcommand now wall-clock-times both engines and prints a `--- slice time ---`
section: the C++ subprocess time (includes process spawn/teardown), the Rust
in-process time, and the `rust/bambu` ratio with a verdict
(`at parity ≤1.10x` / `slower` / `much slower ≥2x`).

```sh
# best-of-N Rust runs suppresses first-run cold-cache noise
COMPARE_TIMING_RUNS=5 devbox run -- \
  target/release/slicer-cli compare --config tests/configs/stl-inline-config.jsonnet
```

Because the C++ number carries subprocess overhead the in-process Rust path does
not, treat the ratio as an order-of-magnitude guide (Benchy is small enough that
spawn cost is a large fraction), not a microbenchmark. The signal to watch is
large models (Majora) where compute dominates spawn.

## How to run

```sh
# needs numpy — run under devbox
devbox run -- bash tests/test_semantic_parity.sh          # Benchy + Cube suite
devbox run -- bash tests/test_semantic_parity.sh <cfg>    # a specific config
devbox run -- python3 scripts/semantic_compare.py a.gcode b.gcode   # ad hoc
```

The wrapper runs `slicer-cli compare` (both engines) then the comparison, and is
wired into CI (`.github/workflows/slicer-cli-ci.yml`). It **skips gracefully**
(exit 0) if numpy or a binary is unavailable. Env: `SLICER_CLI`, `PYTHON`,
`BAMBUSTUDIO_SLICER`.

## Current status

- **Benchy** (`stl-inline-config.jsonnet`): SEMANTICALLY EQUIVALENT — silhouette
  100.00% area-wtd, object material 0.9987, layers 240=240, all per-feature
  within tolerance (R389 re-measured).
- **Cube** (`stl-cube-config.jsonnet`): SEMANTICALLY EQUIVALENT — shape 100%.

### Measured slice time (R390, arm64, standalone both engines)

> **CAUTION — measure the right engine.** `slicer-cli slice` defaults to
> `--engine bambu` (the C++ binary). To time the Rust engine you MUST pass
> `--engine rust`, or you will accidentally compare C++ against C++ (this is
> exactly the R389 mistake, corrected here). C++ native binary:
> `libslic3r/bambustudio/build/slicer_cli`.

| model | Rust (`--engine rust`) | C++ native | ratio |
|-------|------------------------|------------|-------|
| Benchy (small STL) | ~2.67s | ~1.78s | **1.5x** |
| Majora (large multicolour 3MF) | ~46.4s | ~15.5s | **~3.0x** |

Takeaway: Rust **is** meaningfully slower, and the gap **grows** with model size —
~1.5x on Benchy but ~3x on Majora. So this is not just fixed per-slice overhead;
the compute-heavy path itself is slower at scale. Majora is the perf target.

**Phase profiling** (`SLICE_PHASE_TIMING=1`, R391) breaks the Rust slice into its
phases (`Print::process` sub-phases + process/export split + an infill split).
On Majora it isolates the bottleneck precisely:

| phase | time | share |
|-------|------|-------|
| perimeters(+slice) | 7.1s | 16% |
| **infill → prepare_infill (MMS segmentation)** | **35.6s** | **~63% of wall** |
| infill → fill loop (parallel) | 2.2s | 4% |
| export_gcode | 8.8s | 16% |

Drilling in (R392, finer `SLICE_PHASE_TIMING` sub-timers), the 35.6s of
`prepare_infill` breaks down as:

| prepare_infill sub-step | time | share |
|-------------------------|------|-------|
| **discover_vertical_shells** | **18.9s** | 53% |
| **bridge_over_infill** | **15.1s** | 43% |
| process_external_surfaces | 1.0s | 3% |
| detect_surfaces_type | 0.4s | 1% |

The MMS *segmentation* function itself is only ~1.6s — the real cost was
`discover_vertical_shells` + `bridge_over_infill`, both serial per-layer loops.

Two fixes so far, same parallel-compute-then-serial-apply pattern, both
**byte-identical** on Majora and Benchy:

- **R393 `discover_vertical_shells`**: 18.9s → 2.75s (6.9x)
- **R394 `bridge_over_infill` candidate extraction**: 5.39s → 0.49s (11x)
- **R395 `bridge_over_infill` anchor-infill polyline gen**: 4.01s → 0.35s (11x)
- **R396 mesh-slice make_expolygons loop** (restored a C++ `tbb::parallel_for`
  that had been serialized): slice() 6.29s → 3.86s
- **R397 `bridge_over_infill` apply loop**: 2.04s → 0.19s (10.7x)
- **R398 `make_loops_layers`** (restored another C++ `tbb::parallel_for`): helps
  the mesh-slice path, most visible on small models — Benchy ~2.67s → ~2.26s

Cumulative: Majora **46.4s → 21.6s** (~1.39x C++), Benchy **~2.67s → ~2.26s**
(~1.27x). `bridge_over_infill` alone went 12.7s → 2.2s. Remaining big lever is
`export_gcode` (7.4s), which is a sequential gcode writer — unsafe to parallelize
for byte-parity (state carries across layers), so left as-is.

### Re-measured R484 (arm64, release, warm cache, both engines standalone)

The numbers above are superseded. Nothing was optimised between R398 and R484 —
the parity work since simply did not cost time, and several rounds removed work
(e.g. R476/R477 stopped emitting cross-plate tower strokes).

| model | Rust (`--engine rust`) | C++ (`--engine bambu`) | ratio |
|-------|------------------------|------------------------|-------|
| Benchy (`benchy-016.jsonnet`) | 2.38s | 1.96s | **1.21x** |
| Majora (`nu3mf.jsonnet`) | 19.14s | 15.73s | **1.22x** |

So the "gap grows with model size" conclusion no longer holds: both fixtures now
sit at ~1.2x, and Majora is 46.4s → 19.1s since R390. Current Majora breakdown:

| phase | time | share |
|-------|------|-------|
| perimeters(+slice) | 5.90s | 31% — of which slice() 3.73s, perimeter_gen 2.17s |
| infill | 6.72s | 35% — prepare_infill 4.88s, fill_loop 1.84s |
| simplify | 0.30s | 2% |
| **export_gcode** | **6.21s** | **32%** — generate 4.91, post-process 0.73, write 0.57 |

`prepare_infill` is now dominated by `discover_vertical_shells` 1.85s (41.7%) and
`bridge_over_infill` 1.80s (40.6%), both already parallel — i.e. the easy
parallelism wins are spent. The remaining 3.4s gap has no single owner; further
progress needs a profiler (flamegraph/Instruments) rather than inspection.

**NEGATIVE (R484), do not retry:** caching `faithful_gate` (187 call sites, 39 in
the gcode exporter) behind a `OnceLock<RwLock<HashMap>>` to avoid `std::env::var`'s
process-wide lock + String allocation on every call measured **no change** —
Majora 19.62-20.31s, Benchy 2.35-2.37s, both within run-to-run noise. Reverted
rather than keep inert complexity that would also make a mid-run `set_var` stale.
Usage: `SLICE_PHASE_TIMING=1 slicer-cli slice --engine rust --config <cfg>`.
(Rust user/CPU time on Majora ~116s vs C++ ~128s, i.e. Rust does *less* total CPU
but takes 3x the wall time — a parallelism/scheduling problem, not raw throughput:
Rust is not keeping the cores busy. Rust Majora gcode is 60.5 MB vs C++ 69.7 MB,
and the Rust 3MF path is Tier-1 — so it may also be doing different work.)

Both are locked into CI, so a genuinely-broken toolpath (silhouette or material
collapse) is caught, while FP-cascade re-routing correctly passes.

## Known limitations

- **Origin-centered models** (e.g. a sphere STL centered at (0,0,0)) place
  differently between the engines: rust drops-to-bed and slices the full model,
  native slices in place (only the part above Z=0). Use bed-placed models.
- **Very small models**: the header filament total is dominated by skirt/prime;
  the `object material` check (deposited-E, retraction-aware) is the reliable
  material signal.
- The per-feature/per-layer material metrics are retraction-aware (R356), so
  they no longer carry deretraction-priming noise.

## The C++ reference is NOT byte-reproducible (R487)

Measured directly: three runs of the C++ `slicer_cli` on the identical Majora input
produced three different files.

    48ea73f8...   e70b15be...   c28231ca...   06a1d80b...   e6a06729...

This is not a rebuild artifact — the binary was untouched between runs. It settles
the premise of the whole campaign: **byte parity against C++ is impossible in
principle**, because C++ is not byte-identical to *itself*. (The user's instinct in
the standing brief — "byte parity I think is just impossible" — is correct, and this
is the evidence.)

Crucially, the divergence is confined to floating-point noise and does NOT reach the
semantic metrics. Running `semantic_compare.py` on two consecutive C++ runs:

| metric | C++ vs C++ |
|--------|-----------|
| silhouette (mean / area-wtd) | 100.00% / 100.00% |
| wall-line IoU (mean / area-wtd) | 100.00% / 100.00% |
| per-layer material mean deviation | 0.00% |
| object material | 1.0000 |
| every per-feature E-ratio | 1.000 |
| object-only | 0.9999 (4 E in 72,673) |

**So the noise floor of every metric we track is ~0.01% or better.** Header totals
drift in the 5th significant digit (e.g. filament length 65094.33 vs 65094.09) and
the print-time estimate by ~1s, but nothing that shifts a feature ratio. Any gap we
are chasing — FVS 0.856, internal solid 0.922, tower 1.045 — is real signal, orders
of magnitude above this floor.

Practical consequences:
- Never diff C++ gcode against a stored C++ reference and expect a match; compare
  semantically, or regenerate both sides in the same session.
- Rust-side byte-identity between our own runs is still a valid regression guard
  (our engine IS deterministic — that was fixed at R99), and remains the cheapest
  signal that a refactor changed nothing.

### Instrumenting the C++ reference (R487)

The reference is locally buildable, which makes side-by-side internals comparison
possible:

    source: libslic3r/bambustudio/references/BambuStudio/src/libslic3r/
    build:  cd libslic3r/bambustudio/build && ninja slicer_cli   (incremental, ~1 min)
    binary: libslic3r/bambustudio/build/slicer_cli   (what `--engine bambu` runs)

`references/` is git-tracked, so any probe must be reverted afterwards. Note C++
`SCALING_FACTOR = 0.00001` — 1e5 units/mm, the SAME as the Rust crate (an earlier
note claiming C++ used 1e6 was wrong); scaled areas convert with `/1e10`.

### RETRACTED (R495): the prime tower's -2.0 mm "body shift" (R494)

**The section below is wrong and is kept only as a record of the measurement
trap.** There is no -2.0 mm offset between the two tower bodies. Tagging every
candidate emitter in the C++ and probing `WipeTowerWriter::rotate`'s OUTPUT
(`pt.y() + m_y_shift`) shows the writer never emits a local y of -2.000 or
-1.500 anywhere in the run: the only negative outputs are
`generate_support_wall_new`'s brim loops, at multiples of the brim spacing
0.4356. The C++ tower body box is `[0, 39]` in writer coordinates — exactly ours.

The sub-zero values I reconstructed from the gcode belong to the CHANGE-FILAMENT
sequence, not the tower writer: `G1 X185.729 Y197.797 Z.7` sits inside the
`; filament start gcode` block, and the strokes after it are tagged
`; CP_TOOLCHANGE_WIPE`. C++ does not restore the previous `; FEATURE:` tag after
a tower block, so those lines are attributed to `Prime tower` and sit inside the
`WIPE_TOWER_START/END` markers — which is why the marker-scoped and
feature-scoped bounding boxes agreed with each other and I read that agreement as
confirmation. **Two scopes agreeing does not validate either one when both
contain the same contamination.** Compare against the generator's own internal
values instead, which is what finally settled it.

The brim-chamfer observation in the table below (C++ 3.049 -> 2.178 -> 1.307 -> 0
over the first layers, ours layer 0 only) was measured the same way and has NOT
been re-verified against the generator; treat it as unconfirmed.

### The prime tower's Y placement (R494 — SUPERSEDED, see above)

Our tower body and C++'s occupy the same-sized rectangle in a different place.
Working in tower-local coordinates (subtract the emit-time translation, which both
engines share at `(185.229, 199.297)` for Majora — `wipe_tower_x/y`), per layer:

| z     | C++ local Y        | ours local Y      |
|-------|--------------------|-------------------|
| 0.3   | [-5.049, 40.049]   | [-2.250, 41.250]  |
| 0.6   | [-4.178,  39.178]  | [ 0.000, 39.000]  |
| 1.2   | [-3.307,  38.307]  | [ 0.000, 39.000]  |
| 3.3+  | [-2.000,  37.000]  | [ 0.000, 39.000]  |

X is identical in both (`[185.229, 220.229]`, i.e. local `[0, 35]`), and both
bodies are exactly 39.000 deep. Two separate differences:

1. **Body offset.** C++'s body rectangle sits at local Y `[-2.000, 37.000]`; ours
   at `[0.000, 39.000]`. A constant -2.000 shift, stable across all layers.
2. **Brim chamfer.** C++'s first-layer brim extends 3.049 mm and then decays
   2.178 -> 1.307 -> 0 over the next layers (the `loops_num` computation in
   `finish_layer_new`: `min(loops_num, max_chamfer_width/spacing) - dist_to_1st`,
   with `spacing = m_perimeter_width - m_layer_height*(1 - pi/4)`). We emit a brim
   on layer 0 only (2.250 mm) and nothing above it.

Eliminated as the cause of the -2.000, each by direct instrumentation of the C++:

- `m_rib_offset` is `(0,0)` (probed at Print.cpp:3358) — never nonzero here.
- `m_plate_origin` is `(0,0)`.
- `SHAPE_REVERSED` is dead code: `m_current_shape` is hard-assigned `SHAPE_NORMAL`
  at WipeTower.cpp:239, and `SHAPE_NORMAL == 1`. So every
  `set_y_shift(m_y_shift -/+ (SHAPE_REVERSED ? ... : 0))` reduces to `m_y_shift`.
- `WipeTower::m_y_shift` is `0.0000` on all 656 layers (probed in the
  `generate_new` loop). The assignment at :4656 is guarded by
  `m_layer_info->depth < m_wipe_tower_depth - m_perimeter_width`, which never fires
  for Majora since `layer_depth == tower_depth == 39.0` on every layer.
- `WipeTowerWriter::rotate` is the identity here: probed in situ it always sees
  `y_shift=0, angle=0`, and at angle 0 the `m_wipe_tower_depth/2` terms cancel
  algebraically, so emitted local y == box-local y.
- `finish_layer_new`'s own perimeter box is `[0.000, 39.000]` after
  `align_perimeter` on every call (probed; single WipeTower instance, `this`
  constant — it is NOT the `m_fake_wipe_tower` of Print.cpp:3384). That is our
  box exactly, and it maps to global `[199.297, 238.297]` — a value the C++ output
  contains only 5 times, against 2,281 occurrences of `236.297`.

So the `[-2, 37]` rectangle that dominates C++'s output is NOT drawn by
`finish_layer_new`'s perimeter — consistent with `generate_support_wall_new`
returning early (`if (!extrude_perimeter) return wall_polygon;`) on most layers.
The next probe should identify which emitter draws the dominant body rectangle
(most likely `tool_change_new`'s cleaning box or `finish_block*`) and read its box
coordinates directly.

Note also `blocks == 1` for Majora on every layer, so the multi-block fill path in
`finish_layer_new` (`multi_block_fill`) never activates for this fixture — the
R493 block port is not required to close the tower's length gap here.

Two measurement cautions from this round:

- Percentile-trimmed bounding boxes (used by `towergeo.py`) mis-report a shift when
  the two distributions differ in shape; they suggested a clean 2 mm translation of
  the whole footprint, whereas the exact boxes show a -2.0 body shift PLUS a
  different brim profile. Use exact extrema.
- Attributing tower geometry by `; FEATURE: Prime tower` was checked against C++'s
  unambiguous `; WIPE_TOWER_START/END` markers and agrees exactly (same bbox, same
  1,171,244 mm), so feature-tag attribution is sound here despite C++ not
  restoring the previous FEATURE tag after a tower block.


### Per-toolchange filament start/end gcode (R495)

`append_tcr` (GCode.cpp:1035-1053) substitutes THREE placeholders into the tower
gcode — `[filament_end_gcode]`, `[change_filament_gcode]`, `[filament_start_gcode]`
— and the tower writer emits all three (WipeTower.cpp:2465/2466/2483). We emitted
and substituted only the middle one, so Majora was missing 2,723 `; filament end
gcode` and 2,723 `; filament start gcode` blocks.

Closing it needed three independent fixes, each of which alone produced nothing:

1. `filament_start_gcode` / `filament_end_gcode` existed in `apply_key_value` but
   NOT in `set_deserialize`, so the load path left both empty — an instance of the
   known config-key audit gap. Added the two keys selectively (the blanket
   fallback remains a measured negative).
2. The tower writer emitted only `[change_filament_gcode]`, and the export
   substituted only that one. Both other placeholders are now built and injected,
   with the filament-start block leading the trailer exactly as C++ orders it
   (`start_filament_gcode_str + wipe_next_start_point_str + toolchange_unretract_str`).
3. `gcode_template::process` required a directive to occupy its whole line, but
   the stock template is `{if  (bed_temperature[current_extruder] >55)}M106 P3 S200`
   — directive and guarded text on ONE line. Unmatched lines fell through to raw
   substitution, so the first two fixes emitted literal `{if ...}` text into the
   gcode. Same-line directives are now supported.

A fourth gap surfaced after that: the condition needs `current_extruder` and the
`bed_temperature` / `bed_temperature_initial_layer` ARRAYS, none of which the
change-filament context provided, so every branch evaluated false and the block
collapsed to its comment. With those added we now select the same branch C++ does.

Result: 2,722 / 2,721 blocks against C++'s 2,724 (the difference is our two fewer
toolchanges), and `M106 P3 S150` 2,722 vs 2,724. **Material-inert** — these
templates carry no extrusion for this profile, so every per-feature ratio is
unchanged and benchy/cube stay byte-identical. It closes a gcode-CONTENT gap.


### Writer-only tower measurement (R496)

R495's contamination warning raised the question of whether the tower's 1.045
length ratio was an artifact. It is not. Summing E and XY path length over each
`ToolChangeResult.gcode` — the tower writer's own output, BEFORE `append_tcr`
substitutes the filament-end / change-filament / filament-start blocks — gives:

|            | C++         | ours        | ratio |
|------------|-------------|-------------|-------|
| tcrs       | 3,443       | 3,377       | 0.981 |
| segments   | 67,794      | 35,760      | 0.528 |
| length mm  | 1,171,315.6 | 1,223,826.0 | 1.045 |

C++'s writer-only length (1,171,315.6) matches the gcode-side feature-scoped
figure (1,171,244) to 0.006% with an identical segment count, so **the tower
LENGTH measurement was never contaminated** — `toolchange_wipe_new` is writer
output, and the filament-park moves that polluted the bounding box live in the
substituted block, which is not part of `tcr.gcode`. R495's warning therefore
retires only the bbox/Y-geometry claims; R493's per-layer split stands.

(C++'s writer E/mm is 0.05802 against a final-gcode 0.05433 — E is rescaled after
the writer, so writer-side E is not comparable across engines. Length is.)

The segment counts expose the structure: C++ averages 17.3 mm per tower segment,
we average 34.2 mm. C++ splits the work across two emitters that we collapse into
one:

- `finish_layer_new` runs ONLY on layers with no tool change (`wall_idx == -1`,
  WipeTower.cpp:4703), and its fill box — with a single block, which is every
  Majora layer — is the WHOLE layer box, `(m_perimeter_width, m_perimeter_width)`
  by `m_layer_info->depth - 2*m_perimeter_width` (:3570-3577).
- layers WITH tool changes get their fill from `finish_block` / `finish_block_solid`.

Ours runs one `finish_layer` on every layer over the leftover box above the
toolchanges. Three variants were measured against C++'s 1,171,316 mm:

| variant                                          | length      | ratio |
|--------------------------------------------------|-------------|-------|
| current (gate off)                                | 1,223,826   | 1.045 |
| sparse grid on our leftover box (R493)            |   (0.887 E) |       |
| sparse grid on C++'s whole-layer box              | 1,254,598   | 1.071 |
| + fill only on no-toolchange layers               | 1,023,933   | 0.874 |

The last is the faithful half of the structure and undershoots exactly because
the other half — `finish_block`/`finish_block_solid` supplying the fill on
tool-change layers — is not ported. So `TOWER_SPARSE_GRID` stays opt-in and the
default is unchanged; the remaining work is that pair of functions.


### The tower's length error decomposed (R497)

Histogramming tower segment lengths (feature-scoped, which R496 proved equals the
writer-only set) splits the 1.045 into two independent defects:

| segment length | C++    | ours   |
|----------------|--------|--------|
| 34.0 mm        | 27,770 | 33,116 |
| 0.5 mm         | 27,273 |      0 |
| 3.0 mm         |  4,854 |      0 |
| 31.0 mm        |  2,723 |      0 |
| 35.0 / 39.0 mm |   ~720 |  2,624 |
| **total**      | 67,794 segs / 1,171,244 mm | 35,760 / 1,223,826 mm |
| **mean**       | 17.28 mm | 34.22 mm |

1. **We lay 5,346 too many full-width (34 mm) strokes** — about 181,800 mm of
   excess fill lines. This is the dense-vs-sparse defect R493 found.
2. **We never extrude the connector between fill lines.** C++'s solid branch is
   `writer.extrude(writer.x(), y, feedrate).extrude(i % 2 ? left : right, y)` —
   the first call extrudes the 0.5 mm step in Y, the second the stroke in X. Our
   `WipeTowerWriter::rectangle_fill_box` does `travel(start_x, y); extrude(end_x, y)`,
   so the step is a travel. That is 27,273 missing segments worth 13,636 mm.

The two errors have opposite sign, which is why the net is only +52,582 mm.
Fixing (2) alone would push the default from 1.045 to ~1.057; it has to land with
(1).

`finish_block` (WipeTower.cpp:3733) is now ported behind `TOWER_SPARSE_GRID=1`:
its fill box runs from the depth the toolchanges already consumed up to the
block's allocation for the layer, and it ALWAYS lays the inner perimeter of the
sparse section first (`rectangle_fill_box`, which is a rectangle OUTLINE walked
from the nearest corner, not a fill — `finish_layer_new` gates the same call on
`extrude_fill_wall`). Progress on the writer-only total against C++'s 1,171,316 mm:

| variant                                                    | length    | ratio |
|------------------------------------------------------------|-----------|-------|
| default (gate off)                                          | 1,223,826 | 1.045 |
| sparse grid, whole-layer box, no fill on toolchange layers  | 1,023,933 | 0.874 |
| + finish_block box and inner rectangle on those layers      | 1,059,771 | 0.905 |

Still 9.5% short, and the segment count (31,812 vs 67,794) says the connectors and
the 3.0 mm / 31.0 mm structures are the bulk of what remains. The gate stays
opt-in until the total lands near 1.0.


### Closing the tower arithmetic (R498)

Porting the extruded connector (`TOWER_WIPE_CONNECTOR=1`, opt-in) recovers almost
exactly the segments R497 said were missing. Writer-only totals against C++'s
1,171,316 mm / 67,794 segments:

| variant                     | length    | ratio | segments | seg ratio |
|-----------------------------|-----------|-------|----------|-----------|
| default                     | 1,223,826 | 1.045 | 35,760   | 0.528     |
| connector only              | 1,238,919 | 1.058 | 65,945   | **0.973** |
| connector + sparse grid     | 1,072,015 | 0.915 | 56,301   | 0.830     |

C++ extrudes the step to the next purge line rather than travelling it
(`toolchange_wipe_new`, WipeTower.cpp:4145-4149: `writer.extrude(writer.x(),
writer.y() ± dy)` followed by `m_left_to_right = !m_left_to_right`), so the purge
is a single continuous serpentine. The same is true of the solid fill branch
(:3619-3623). With that ported the segment count lands within 2.7% of C++'s — but
the total goes UP, exactly as R497 predicted, because the excess-stroke defect is
still there. Hence opt-in.

The remaining arithmetic now closes:

    connector-only total                             1,238,919
    - 5,346 excess 34.0 mm strokes                    -181,764
    + 4,854 segments of 3.0 mm                         +14,562
    + 2,723 segments of 31.0 mm                        +84,413
    ------------------------------------------------------------
                                                      1,156,130   = 0.987 of C++

Both missing segment classes come from ONE place: the ironing block that opens
each toolchange wipe, `if (i == 0 && m_use_gap_wall)` (WipeTower.cpp:4079-4116).
It extrudes a short ironing run (the 3.0 mm class), does a retract / travel /
un-retract dance, then extrudes the rest of the way to the far edge (the 31.0 mm
class — and 2,723 is exactly one per toolchange).

`m_use_gap_wall` is `config.prime_tower_skip_points` (:1747). That key appears in
our preset key list and in `generator.rs`'s defaults, and the C++ gcode header
carries it, but **there is no `PrintConfig` field and no deserialize arm for it**,
so our `use_gap_wall` is permanently `false` and none of that geometry is emitted.
This is a second instance of the R495 config-key pattern — a key that is "known"
in one table and silently absent from the one that matters.


### The ironing block, and a correction to R498's arithmetic (R499)

`prime_tower_skip_points` had no `PrintConfig` field and no deserialize arm at all,
though it sits in preset.rs's key list, in generator.rs's defaults, and in C++'s
own gcode header (value 1 for Majora). So `WipeTower::m_use_gap_wall`
(WipeTower.cpp:1747) was permanently false on our side. Added the field, a
selective `set_deserialize` arm, and the wiring to `cfg.use_gap_wall`.

With that, the ironing block that opens each toolchange wipe (:4079-4116) is
ported: extrude a short run (`ironing_length = 3.`, :4073), retract, travel back
1.5x and forward again, un-retract, then extrude the rest of the way to the far
edge. (`spiral_flat_ironing` is not ported — it needs
`filament_tower_ironing_area`, and the non-flat branch is taken for this profile.)

**R498's arithmetic was wrong and is retracted.** It assumed C++'s 4,854 segments
of 3.0 mm and 2,723 of 31.0 mm were length we were MISSING, and predicted
+98,975 mm. They are not: the ironing SPLITS the first purge line of each
toolchange into 3.0 + 31.0 = 34.0 mm. Landing it left the writer-only total
completely unchanged at 1,238,918.5 mm and moved only the segment count:

| variant                          | length    | ratio | segments | seg ratio |
|----------------------------------|-----------|-------|----------|-----------|
| default                          | 1,223,826 | 1.045 | 35,760   | 0.528     |
| connector                        | 1,238,919 | 1.058 | 65,945   | 0.973     |
| connector + ironing              | 1,238,919 | 1.058 | **68,666** | **1.013** |
| connector + ironing + sparse grid| 1,072,015 | 0.915 | 59,022   | 0.871     |

The segment count is now within 1.3% of C++'s. Corrected accounting: C++'s
full-stroke equivalents are 27,770 + 2,723 = 30,493 against our 33,116, so the
excess is ~2,623 strokes, ~89,182 mm, putting a correct fix at ~1,149,700 = 0.982.

Where that excess is NOT: probing C++'s purge geometry directly gives
`dy = 0.5 = m_perimeter_width` (the `m_layer_info->extra_spacing *
get_block_gap_width(...)` product collapses to the perimeter width here, with
extra_spacing 1.0), `wipe_length / line_len = 374.0 / 34.0` and a 5.5 mm cleaning
box — 11 lines, exactly what we emit. **The purge line count matches; the excess
is in the finish-layer fill.** `TOWER_SPARSE_GRID=1` currently removes 166,903 mm
where only ~89,000 should go, which is why it undershoots to 0.915.


### Decomposing the finish-layer fill (R500)

`TOWER_SPARSE_GRID` bundled four independent changes, so toggling it moved the
tower by their sum and hid which one was mis-sized. It is now a master switch with
three per-behaviour overrides that default to it (`TOWER_FILL_BOX`,
`TOWER_FILL_RECT`, `TOWER_FILL_GRID`), each forceable to 0 or 1.

Measured individually from the connector+ironing baseline of 1,238,918.5 mm, where
the target is C++'s 1,171,315.6 mm — i.e. we need **-67,603 mm**:

| knob            | writer length | delta     | segments |
|-----------------|---------------|-----------|----------|
| none            | 1,238,918.5   |    —      | 68,666   |
| `TOWER_FILL_GRID` | 1,103,214.6 | **-135,704** | 58,194 |
| `TOWER_FILL_RECT` | 1,258,837.5 | +19,919   | 69,494   |
| `TOWER_FILL_BOX`  | 1,238,815.0 | -104      | 68,660   |
| GRID+RECT         | 1,072,020.4 | -166,898  | 59,022   |
| all three         | 1,072,015.5 | -166,903  | 59,022   |

Three findings:

- **`TOWER_FILL_BOX` is inert** (-104 mm). C++'s whole-layer fill box only applies
  on layers with no tool change, and on exactly those layers `toolchanges_depth`
  is zero, so our leftover box already equals it. The box was never the defect.
- **The effects are not additive.** GRID alone is -135,704 but GRID+RECT is
  -166,898, not -115,785: with the faithful grid running, the inner rectangle
  lands in a different code path and its sign flips.
- **The faithful grid overshoots by 2x.** It removes 135,704 mm where only 67,603
  should go, leaving us 99,296 mm SHORT of C++ once enabled — against 67,603 mm
  OVER with it disabled. No combination of the three lands on target.

So the sparse grid is not simply mis-scoped: C++ deposits materially more
finish-layer fill than our faithful-looking sparse branch does. The prime
suspect for the ~99 kmm residual is `finish_block_solid` (WipeTower.cpp:3842),
still unported, which serves every block whose `layers_type[m_cur_layer_id]` is
not `Normal` — those layers get a SOLID fill in C++ where we give them the sparse
grid. That is the next thing to port.


### The tower is a purge/fill cancelling pair (R501)

Instrumenting BOTH generators to split the tower into purge (tool-change tcrs) and
fill (finish-layer tcrs) finally shows what the 1.045 total was hiding:

|        | C++         | ours        | ratio  |
|--------|-------------|-------------|--------|
| purge  | 1,122,369.5 |   937,384.5 | 0.835  |
| fill   |    48,946.1 |   301,534.0 | 6.16   |
| total  | 1,171,315.6 | 1,238,918.5 | 1.058  |

**Our purge is 185,000 mm SHORT and our fill is 252,588 mm OVER.** The same
cancelling-pair pattern as R492/R493, at ten times the scale, and it invalidates
R499's conclusion that "the purge matches C++": that was based on `dy` and the
per-wipe line count of a sampled toolchange, not on the total.

C++'s fill decomposes exactly:

| emitter             | calls | length     |
|---------------------|-------|------------|
| `finish_block_solid`|     0 |        0.0 |
| `finish_block`      |   206 |   35,684.5 |
| `finish_layer_new`  |   656 |   13,261.6 |
| total               |       |   48,946.1 |

**`finish_block_solid` is never called for Majora** — it is eliminated as the
residual, and porting it would have changed nothing. `finish_layer_new` runs on all
656 layers but contributes only 13.3 kmm, i.e. ~20 mm per layer: a genuinely
sparse grid. `finish_block` runs on just 206 of the tool-change layers.

Against that, our fill is 301.5 kmm at baseline and still 134.6 kmm with the
faithful sparse grid and inner rectangle enabled — 2.75x C++'s. So the fill has a
second defect beyond the sparse/dense branch, and the purge has a large deficit of
its own. Both must be fixed before either gate can go default-ON; neither is
visible in the total, which is why every attempt to tune the fill alone (R493,
R496-R500) moved the aggregate the wrong way.


### The tower's three components (R502)

Tagging every C++ emitter and accumulating extruded XY length inside
`WipeTowerWriter` gives the tower's true composition — including a component the
purge/fill split of R501 had folded away invisibly, the outer wall:

| emitter                     | length      | segments |
|-----------------------------|-------------|----------|
| `toolchange_wipe_new`       | 1,032,017.0 |   59,906 |
| `generate_support_wall_new` |    93,756.0 |    5,683 |
| `finish_block`              |    35,684.5 |    1,934 |
| `finish_layer_new`          |     9,786.0 |      271 |
| `ramming` / `nozzle_change` |         0.0 |        0 |

`ramming` and `nozzle_change` contribute NOTHING for this fixture — eliminated.
The sum is 1,171,243.5 against the writer-only total of 1,171,315.6, so the
decomposition is complete to within rounding.

Ours splits the same way (our wall is one `writer.rectangle(&wt_box)` per layer:
656 x 2*(35+39) = 97,088 mm, which matches R493's independent perimeter-vs-interior
measurement exactly):

| component | C++         | ours        | ratio |
|-----------|-------------|-------------|-------|
| purge     | 1,032,017.0 |   937,384.5 | 0.908 |
| wall      |    93,756.0 |    97,088.0 | 1.036 |
| fill      |    45,470.5 |   204,446.0 | **4.50** |
| total     | 1,171,243.5 | 1,238,918.5 | 1.058 |

Both columns sum to their measured totals exactly, so this is the real target list:

- **The wall is already right** (1.036) — leave it alone.
- **The purge is 94,632 mm short** (0.908). R501 reported 0.835 because its purge
  bucket was tcr-scoped and swept in wall and timelapse geometry; the emitter-level
  figure is the one to chase.
- **The fill is 4.50x** — 158,976 mm too much, and that is the single largest
  defect. C++'s entire fill is 45.5 kmm over 2,205 segments; ours is 204.4 kmm.

Note `finish_layer_new` runs on all 656 layers but emits only 271 fill segments
totalling 9.8 kmm — on most layers it lays no fill at all. Our finish_layer lays a
fill on every layer.


### Why C++'s finish-layer fill is almost nothing (R503)

Probing `finish_layer_new` directly: **`extrude_fill` is FALSE on 653 of its 656
calls** — the grid ran 3 times in the whole print. The dominant call site is
WipeTower.cpp:4746, which passes a literal `false`:

    if (wall_idx != -1) {
        if (layer.tool_changes.empty())
            finish_layer_new(only_generate_wall ? false : true, false, false);

So on nearly every layer `finish_layer_new` draws ONLY the outer wall. That
accounts for its 9,786 mm over 271 segments: three dense grids, not 656 sparse
ones. C++'s finish-layer fill therefore comes almost entirely from `finish_block`
(206 calls, 35,684.5 mm).

`layer.extruder_fill` itself is NOT the guard — it defaults to `true` and is only
cleared for the last layer (`set_last_layer_extruder_fill`, Print.cpp:3337).

R497's layer condition was exactly backwards: it suppressed the fill on
tool-change layers, whereas C++ fills on ~206 tool-change layers and essentially
never on the others. Corrected behind `TOWER_FILL_ONLY_TC`, but that knob is nearly
inert here (-2,172 mm): with 2,723 tool changes across 656 layers, almost every
layer has one.

**The real guard is the block-fullness test at :4751:**

    if (block.cur_depth + EPSILON >=
        block.start_depth + block.layer_depths[m_cur_layer_id] - m_perimeter_width)
        continue;   // this block is already full — no fill at all

`block.layer_depths[m_cur_layer_id]` is the depth ALLOCATED to that block on that
layer, which is roughly what its tool changes consumed. So on most layers the block
is already full and gets no fill. Our fill box instead runs from
`toolchanges_depth` up to `layer_depth`, where `layer_depth` is
`self.plan[idx].depth` — the FULL tower depth, 39 mm on every layer. We therefore
fill the entire unused area of the tower on every single layer, which is the whole
4.50x fill excess.

Closing it needs `block.layer_depths` — i.e. the per-layer depth allocation from
`update_all_layer_depth` (:4237) / `generate_wipe_tower_blocks` (:4268) — not the
plan's global depth. That is the next port, and it is the last piece of the fill.


### CORRECTION (R504): our fill box formula was already right

R503 concluded that our fill box runs to the full tower depth while C++ uses a
per-layer allocation, and called that "the whole 4.50x fill excess". **That is
wrong.** Probing `finish_block`'s actual box:

    [FB] cur_depth=17.000 start=0.500 layer_depth=38.500 plan_depth=39.000 box_h=21.500
    [FB] cur_depth=28.000 start=0.500 layer_depth=38.500 plan_depth=39.000 box_h=10.500
    [FB] cur_depth=33.500 start=0.500 layer_depth=38.500 plan_depth=39.000 box_h= 5.000

C++'s height is `block.start_depth + block.layer_depths[cur] - block.cur_depth -
m_perimeter_width` = `0.5 + 38.5 - cur_depth - 0.5` = **38.5 - cur_depth**. Ours is
`layer_depth - (toolchanges_depth + perimeter_width)` = `39 - tc_depth - 0.5` =
**38.5 - tc_depth**, and `cur_depth = start_depth + tc_depth` (the probe's first
row is cur_depth 17.0 for a layer whose `toolchanges_depth` is 16.5). The two
formulas are identical; `block.layer_depths[cur]` is 38.5 against the plan's 39.0,
a 0.5 mm offset that is already absorbed by our `+ perimeter_width`.

Note also that with one block, step 4 of `generate_wipe_tower_blocks` (:4316-4324)
makes `m_plan[layer].depth` the SUM over blocks of `layer_depths[layer]`, so for
Majora they only ever differ by the `start_offset`. There is no per-layer
allocation to port.

The fill excess is therefore two things, neither of them the box:

1. **Density.** C++'s grid is sparse (vertical strokes at `m_bridging` = 10 mm);
   ours is a dense zig-zag at 0.5 mm pitch. `TOWER_FILL_GRID=1` fixes this and
   takes our fill from 301,534 mm to 133,720 mm.
2. **Call count.** C++ runs `finish_block` 206 times; we fill on ~654 layers. The
   remaining 2.9x is entirely this. The skip at :4751 fires when
   `cur_depth >= 38.5`, i.e. `toolchanges_depth >= 38.0`, which the R494 histogram
   says happens on ~94 layers — not enough on its own, so the other guards in the
   dispatch (`is_valid_last_layer`, `finish_layer_filament == -1`, and the outer
   `wall_idx != -1`) account for the rest. That is what remains to port.


### The fill is essentially solved; the purge is the whole remaining gap (R505)

Two measurements settle the fill.

**1. The call count already matches.** Counting each dispatch guard in C++ and the
equivalent guards in our `finish_layer`:

|                | C++ | ours |
|----------------|-----|------|
| layers         | 656 | 656  |
| no-toolchange  |  64 |  64  |
| block-full skip| 386 | 385  |
| **fill runs**  | **206** | **207** |

`is_valid_last_layer` and the `finish_layer_filament == -1` resolution reject
NOTHING (0 of 592) — eliminated. Our existing `dy > m_perimeter_width` guard is
already equivalent to C++'s block-full skip. **The call count was never the
defect**, which retires this round's entire plan.

**2. The 4.50x fill figure was measured with the fill knobs OFF, and our "fill"
bucket includes the wall.** Subtracting the 97,088 mm wall:

| combo                          | fill bucket | actual fill |
|--------------------------------|-------------|-------------|
| GRID only                      | 165,830.1   | 68,742.1    |
| GRID + ONLY_TC                 | 164,913.8   | 67,825.8    |
| **GRID + RECT + ONLY_TC**      | 133,719.6   | **36,631.6**|
| C++                            |             | 45,470.5    |

(RECT *reduces* the total because `writer.rectangle` repositions the writer and
changes where the following grid starts — the non-additivity R500 flagged.)

So with the faithful knobs on, the tower decomposes as:

| component | C++         | ours        | ratio |
|-----------|-------------|-------------|-------|
| purge     | 1,032,017.0 |   937,384.5 | 0.908 |
| wall      |    93,756.0 |    97,088.0 | 1.036 |
| fill      |    45,470.5 |    36,631.6 | 0.806 |
| total     | 1,171,243.5 | 1,071,104.1 | 0.914 |

**The fill is within ~8.8 kmm and the wall within 3.3 kmm. The purge, 94,632 mm
short, is now the entire remaining tower gap** — and it is the only component that
has never been worked on.


### The tower closes: the cleaning box was 0.5 mm short (R506)

Probing C++ first, per the discipline: **`solid_tool_toolchange` is 0 for Majora**,
so `x_to_wipe = FLT_MAX` never applies — that hypothesis is eliminated. All 2,723
tool changes are ordinary, totalling 1,032,017 mm over 59,906 lines = **379.0 mm
and 22.0 lines per tool change**, i.e. 11 strokes + 11 connectors
(11 x 34 + 11 x 0.5 = 379.5). Ours was 344.0 mm — about 10 strokes.

Our `wipe_length` (344.337) and `num_lines` (11) both matched C++ exactly, so the
budget was never wrong. What differed was the box:

| | C++ | ours |
|---|---|---|
| cleaning box height | 5.500 | **5.000** |
| depth allocated per tool change | 5.5 | 5.5 |

We built it as `wipe_depth - m_perimeter_width`; `tool_change_new` (:3271) builds it
as **`wipe_depth - nozzle_change_depth`** — and `nozzle_change_depth` is zero here
(R502 showed `nozzle_change` emits nothing). At `dy = 0.5` that missing 0.5 mm is
exactly one purge stroke per tool change: 34.5 mm x 2,723 = 94,000 mm, against a
measured deficit of 94,632 mm.

With `TOWER_CLEANING_BOX` fixed and the rest of the tower set enabled:

| component | C++ | ours | ratio |
|-----------|-------------|-------------|-------|
| purge | 1,032,017.0 | 1,031,259.0 | 0.9993 |
| wall + fill | 139,226.5 | 133,719.6 | 0.960 |
| total | 1,171,243.5 | 1,164,978.6 | **0.9946** |

The box fix ALONE is a regression (tower 1.045 -> 1.1239) because it adds purge to
an already-long tower; it only works with the fill knobs. So the whole tower set —
`TOWER_SPARSE_GRID` and its `TOWER_FILL_*` knobs, `TOWER_WIPE_CONNECTOR`,
`TOWER_CLEANING_BOX` — is now **default-ON**, and the tower goes 1.045 -> **0.9947**.

Effect on the verdict, which moves from two passing checks to four:

| check | before | after |
|-------|--------|-------|
| object material within 1% | FAIL 1.0193 | **PASS 0.9959** |
| per-layer material mean <5% | FAIL 12.42% | **PASS 4.47%** |
| layer count | PASS | PASS |
| per-feature <35% | PASS | PASS |
| silhouette area-wtd >=99% | FAIL 94.31% | FAIL 94.08% |

Silhouette slips 0.23 pp — the tower's footprint changed slightly — and is now the
only failing check. Every object feature is untouched (object-only 0.9969), benchy
and the painted cube stay byte-identical, and the eight guard tests pass.


### The silhouette check was scoring the tower too (R507)

`silhouette_iou` called `coverage_iou(..., feats=None)` — every feature, prime
tower included — while labelling itself "object outline". On a multi-material print
the tower dominates it. Measured on Majora:

| scope                | mean   | area-wtd | min           | layers <98% |
|----------------------|--------|----------|---------------|-------------|
| all features (old)   | 87.49% | 94.08%   | 12.8% (z178.2)| 656/656     |
| **object only**      | 97.08% | **96.67%** | 93.9% (z26.7) | 519/634   |
| prime tower only     | 85.78% | 96.24%   | **0.2% (z177.6)** | 294/656 |

The tower dragged the unweighted mean from 97.08% to 87.49% and made every single
layer score below 98%. The tower already has its own material ratio in the verdict,
so the silhouette check now scores the OBJECT and the tower's outline is reported
alongside for information. The verdict figure moves 94.08% -> 96.67% — a
measurement correction, not an engine change; the gcode is untouched.

Benchy (single-material, no tower) is unaffected and remains **SEMANTICALLY
EQUIVALENT** on all five checks with a 99.99% silhouette, which is the control
that the rescoping is sound.

The tower's own outline is the striking number: 96.24% area-weighted but a mean of
85.78% and a **minimum of 0.2%** at z177.6 — near-total disagreement on some
layers, even though the tower's total material is now 0.9947. That means the tower
is being placed or shaped very differently on a subset of layers while the totals
agree, and it is the next thing to look at.


### The tower's worst layers: a 64-layer band with wall only (R508)

Ranking layers by prime-tower IoU finds 64 layers below 50%, in one contiguous
band (z177.6 upward), all with the same signature:

| z      | tower IoU | our len | C++ len |
|--------|-----------|---------|---------|
| 177.60 |  0.19%    |  148.0  |  482.0  |
| 178.80 |  0.63%    |  148.0  |  292.0  |
| ...    |  0.63%    |  148.0  |  292.0  |

**148.0 mm is exactly the bare tower wall** (2 x (35 + 39)). On these layers we
emit the wall and nothing else; across the band we lay 9,472 mm against C++'s
19,258 mm. The 64 layers are precisely the `wall_idx == -1` set counted in R505.

Probing our side on those layers:

    [WTNOTC] z=177.60 layer_depth=11.000 fill_box_h=10.000 dy= 9.500 tc_depth=0
    [WTNOTC] z=178.80 layer_depth= 0.000 fill_box_h=-1.000 dy=-1.500 tc_depth=0

With a plan depth of 0 the fill box height is negative, so the fill is skipped
regardless of any knob. Two candidate fixes were tested and BOTH ELIMINATED:

- **`TOWER_FILL_ONLY_TC=0`** (letting no-toolchange layers fill) adds only 912 mm
  across the band — 14 mm per layer, not the ~300 C++ emits — because the box is
  still degenerate.
- **Monotonic depth propagation.** The live path's step 4
  (`generate_wipe_tower_blocks`, :4316-4324) sets
  `layer_depths[i] = max(layer_depths[i], layer_depths[i+1])` unconditionally,
  whereas ours is the DEAD `plan_tower`'s conditional version (:3009-3013) that
  only pulls a layer up when it is already within `2 * m_perimeter_width`. Adding
  the unconditional pass is a **no-op** on this fixture — our plan is already
  monotonic in that direction — so it was reverted rather than kept as inert
  complexity.

So the open question is narrower than it looked: **why does C++ have a non-trivial
`m_layer_info->depth` on these layers at all?** R494 probed `depth = 39.0` but only
on layers 0-7. The next step is to probe C++'s per-layer depth AT z177.6-182 and
find what keeps it non-zero where ours collapses to 0.


### The 64 wall-only layers are a timelapse config gap (R509)

Probing C++ across the failing band, printing z with every row:

    [BAND] z=177.60 plan_depth=39.000 layer_depths0=11.000 cur_depth=0.500 tc=0 wall_idx=-1
    [BAND] z=178.80 plan_depth=39.000 layer_depths0= 0.000 cur_depth=0.500 tc=0 wall_idx=-1

C++'s `m_layer_info->depth` is **39.000 on every layer**, while its
`block.layer_depths[0]` is 11.0 / 0.0 — which is exactly what OUR single
`plan[i].depth` holds. `finish_layer_new`'s fallback box uses
`m_layer_info->depth - 2 * m_perimeter_width` (38 mm), so C++ fills the whole
tower on those layers; ours used 0.0 and produced a negative box.

What keeps C++'s value at 39: `update_all_layer_depth` forces every
`plan_info.depth` to `m_wipe_tower_depth` when `m_enable_timelapse_print` is set,
and that flag is `config.timelapse_type == tlSmooth` (WipeTower.cpp:1742, with
`tlSmooth = 1`). Majora's config carries `timelapse_type = 1`.

**`timelapse_type` was a fourth instance of the config-key pattern** — present in
`apply_key_value` but absent from `set_deserialize`, so it never reached
`PrintConfig`, and `cfg.enable_timelapse_print` was additionally never assigned in
`print.rs`. The (A6) note that this key was "cosmetic" was wrong; it is
load-bearing for the tower's per-layer depth. The deserialize arm is now added.

Wiring it up does fix the band — `layer_depth` becomes 38.5 everywhere and the fill
runs — but **it regresses the tower on its own**: 0.9947 -> 1.0394, with layers below
50% tower IoU going 64 -> 184 and our fill count 207 -> 498 against C++'s 206. The
reason is that C++ keeps TWO per-layer depths where we keep one:

| C++ value | drives |
|-----------|--------|
| `block.layer_depths[cur]` (11.0 / 0.0 here) | `finish_block`'s box AND its block-full skip |
| `m_layer_info->depth` (39.0 under timelapse) | `finish_layer_new`'s fallback box |

Setting our single `plan[i].depth` to 38.5 fixes the second and breaks the first.
So the wiring is behind `TOWER_TIMELAPSE_DEPTH=1` (opt-in) until the two depths are
separated — that separation is the next step, and per R506 the candidate must be
judged in the fully-corrected configuration, not against today's default.


### Separating the two per-layer tower depths (R510)

R509 showed our single `plan[i].depth` was playing two C++ roles at once. They are
now separate fields on `WipeTowerLayerInfo`:

| our field | C++ counterpart | consumer |
|-----------|-----------------|----------|
| `depth` | `m_layer_info->depth` (timelapse forces it to the full tower depth) | `finish_layer_new`'s fallback fill box |
| `alloc_depth` | `block.layer_depths[m_cur_layer_id]` | `finish_block`'s fill box AND its block-full skip |

`alloc_depth` is frozen from `depth` at the end of planning, just before the
timelapse override rewrites `depth`, so the allocation survives.

This is structurally right and is a **no-op by default** (all three fixtures stay
byte-identical). But it does NOT make the timelapse path a win:

| configuration | writer length | tower | layers IoU<50% |
|---------------|---------------|-------|----------------|
| default (timelapse off) | 1,164,978.6 | **0.9947** | **64** |
| `TOWER_TIMELAPSE_DEPTH=1`, depths separated | 1,199,301.4 | 1.0239 | — |
| the same, plus `TOWER_FILL_ONLY_TC=0` | 1,224,572.5 | 1.0342 | 181 |
| C++ | 1,171,315.6 | 1.0 | — |

Separating the depths does recover ground (1,217,371 -> 1,199,301 with timelapse
on), but the default still wins on BOTH the total and the bad-layer count, so
`TOWER_TIMELAPSE_DEPTH` stays opt-in.

The residual is now identified: **our `alloc_depth` is inflated relative to C++'s
`layer_depths`.** With timelapse on our fill runs on 410 layers against C++'s 206
even though the skip formula is, if anything, stricter than C++'s (we skip at
`tc_depth >= alloc_depth - 1.5`, C++ at `tc_depth >= layer_depths - 0.5`). Ours is
derived from `plan[i].depth.max(toolchanges_depth())` plus the dead path's
downward propagation; C++'s is built per filament CATEGORY in
`generate_wipe_tower_blocks` from `m_all_layers_depth` (:4290-4315). Comparing those
two series layer by layer is the next step.


### Top surface 1.173 is a REGION problem, not a fill problem (R511)

The tower now passes its material check at 0.9947, so this round moved to the
largest remaining per-feature error. Top surface: 9,914.4 mm against C++'s
8,440.3 (ratio 1.1746) over 331 layers, with widths already matching (w-rat 0.998)
and both engines configured `top_surface_pattern = zig-zag`.

Splitting by whether both engines emit top surface on a layer:

| | layers | our mm | C++ mm |
|---|--------|--------|--------|
| only we emit | 26 | 245.9 | — |
| only C++ emits | 14 | — | 111.6 |
| **both emit** | **291** | **9,668.5** | **8,328.7** (1.161) |

So the exclusive layers are a minor effect; the excess is on shared layers. Then
measuring the per-layer IoU of the top-surface REGION itself:

    shared layers 291   IoU mean 79.03%   area-wtd 77.06%   min 0.0%
    IoU >= 95%: 122     90-95%: 24        < 90%: 145

**On the 122 layers where the regions agree, our length is 0.9927 of C++'s** —
2,958.2 vs 2,980.0. The fill pattern, its spacing and its flow are already right.
The whole 1.173 lives on the 145 layers where the regions DISAGREE: 6,710.3 vs
5,348.7 = **1.255**.

Top surface is therefore a surface-CLASSIFICATION problem: we mark different
regions as top than C++ does, and then fill them correctly. The next step is the
`stTop` classification itself, not the fill — which also means it shares a root
with the object silhouette failure (96.67%), since misplaced top regions move the
swept outline too.


### The top-surface excess is fed by sliver surfaces upstream (R512)

Following R511's finding that Top surface is a region problem, comparing the actual
top-surface geometry on the worst layers:

| z | ours | C++ |
|---|------|-----|
| 3.30 | 157 segs / 160.9 mm, bbox X[44.03,157.71] Y[54.52,192.78] | 105 segs / 79.1 mm, bbox X[44.03,54.37] Y[160.79,192.76] |
| 123.60 | 134 segs / 108.8 mm | 59 segs / 43.6 mm (same bbox) |
| 131.70 | 67 segs / 54.3 mm | **none at all** |

At z3.30 our top region spans the whole object where C++'s is one corner, and our
segments average ~1.0 mm. These are scattered SLIVERS, not a zig-zag fill of a
compact region — which is why the fill itself measures correct (R511: 0.9927 on
layers whose regions agree) while the total runs 1.16x.

The `opening_ex` collapse that C++ uses to kill narrow parts IS ported and its
offset is right: probed at runtime it is **0.040 mm**, matching
`layerm->flow(frExternalPerimeter).scaled_width() / 10.f` with the coord_t
truncation R283 already handled. So the filter is not the defect.

What the same probe shows is that the INPUT is already wrong. Dumping the layer's
region slices before top detection:

    TSDBG-R in_slice npts=105 a=44.6519
    TSDBG-R in_slice npts= 93 a=47.6177
    TSDBG-R in_slice npts=380 a=162.3884
    TSDBG-R in_slice npts=  4 a=0.0002
    TSDBG-R in_slice npts=  3 a=0.0001
    TSDBG-R in_slice npts=  8 a=0.0008
    TSDBG-R in_slice npts= 38 a=0.0108

alongside the real 44.7 / 47.6 / 162.4 mm2 contours there is a population of
degenerate surfaces at 1e-4 to 1e-2 mm2. Those arrive from slicing / region
assignment, upstream of `detect_surfaces_type`, and an opening at 0.040 mm only
removes what is thinner than 0.08 mm — a long thin sliver survives it. So the next
step is upstream: find where these micro-surfaces enter our region slices and
whether C++ carries them at all.


### Our region slices are over-fragmented: same area, 25 extra slivers (R513)

Instrumenting BOTH engines' `layerm->slices` at the same layer (print_z 4.80) and
comparing the surface-area populations:

| | surfaces | total area | surfaces < 0.05 mm2 | smallest |
|---|----------|------------|---------------------|----------|
| ours | **55** | 1737.91 mm2 | **30** (sum 0.0187) | 0.0001 |
| C++ | **30** | 1737.93 mm2 | 2 (both 0.0000) | 0.0521 |

**The total areas agree to 0.02 mm2 out of 1738** — the sliced geometry itself is
right. We simply decompose it into 55 pieces where C++ produces 30, and the extra
25 are degenerate fragments of 1e-4 to 1e-2 mm2. C++'s smallest non-zero surface
here is 0.0521 mm2; ours are three orders of magnitude below that.

The same probe re-confirms the narrow-part collapse is not at fault: C++ reports
`offset=4000.0000 (scaled) = 0.0400 mm`, exactly our value.

So the top-surface excess (R511/R512) traces to a POLYGON DECOMPOSITION difference
in region assignment, not to surface typing, not to the fill, and not to the
slicing geometry. Each stray fragment can be typed `stTop` and then filled, which
is why our top regions read as scattered ~1 mm slivers spread across the whole
object while C++'s are compact.

Next: find which boolean in our region assignment leaves the extra pieces — the
union/diff chain that builds `LayerRegion::slices` — and whether C++ applies a
simplification or area cull we skip. Note the areas MATCHING means this is a
cosmetic-looking difference with real downstream cost, so the fix must preserve the
total area exactly.


### The slivers are all in region 0 — and they are INERT for Top surface (R514)

Adding `region_id` to both engines' slice probes localises R513's fragmentation
exactly. At print_z 4.80 (8 regions on both sides):

| region | ours nsurf / tiny / area | C++ nsurf / tiny / area |
|--------|--------------------------|--------------------------|
| **0** | **39 / 30 / 993.64** | **12 / 0 / 998.31** |
| 2 | 2 / 0 / 67.76 | 2 / 0 / 67.77 |
| 3 | 5 / 0 / 350.61 | 6 / 1 / 346.01 |
| 4 | 9 / 0 / 325.90 | 10 / 1 / 325.84 |

**Every one of the 30 slivers is in region 0** — the multi-material remainder.
Regions 2/3/4 match C++ in count and area. The source is
`print_object.rs`: `let remaining = difference(region0_ex, &stolen_total)`, a raw
boolean difference that leaves fragments wherever a stolen region's edge nearly
coincides with the parent's.

**But culling them changes nothing.** An opt-in experiment dropping remainder
pieces below C++'s observed 0.05 mm2 floor took region 0 from 39 surfaces (30 tiny)
to 9 (0 tiny) with its area unchanged at 993.62 — and **Top surface stayed at
exactly 452.4 mm, ratio 1.173**, object-only unchanged at 0.9969. The experiment
was reverted rather than kept as inert complexity.

So the region-0 sliver population is REAL but INERT for the top-surface metric, and
the R512/R513 chain's last link does not hold: the scattered ~1 mm top segments
seen in the gcode are not produced by these slice fragments. Two facts do survive
and are worth carrying forward:

- our remainder region is genuinely over-fragmented (39 vs 12 pieces), which is a
  latent difference even if this metric does not see it;
- **there is a ~4.6 mm2 patch assigned differently**: our region 0 is 4.67 mm2
  SMALLER than C++'s while our region 3 is 4.60 mm2 LARGER. That swap, not the
  slivers, is the real region-assignment discrepancy at this layer, and it is where
  the next look should go.

## R515 — negative-volume parts are silently dropped (real gap, measured near-inert)

Probed `Layer::lslices` on both engines for all 656 Majora layers (`LSDBG=1`,
per-layer outer-contour area / hole count / hole area).

Structure agrees everywhere — same expolygon count on every layer — but the
**hole population does not**: ours carries holes on 24 layers, C++'s on 186.
At z 0.30–3.00 C++ has five identical circular holes (Ø3.10 mm, 7.535 mm²
each, 37.67 mm² total) that we do not; net area disagreement over the 177
differing layers is 1593 mm².

`Layer::make_slices` and `union_safety_offset_ex` are EXONERATED: the holes are
already absent one level up, in the per-region `slices.surfaces` (`LSDBG` also
reports `rnholes=0` at those layers). The source is the 3MF itself — Majora
declares seven `negative_part` volumes (`Connector-1_A` … `Connector-7_B` in
`Metadata/model_settings.config`) and `app_slice.rs:577` documents that Tier-1
merges only printable `model` geometry and skips the rest. C++ slices negative
volumes alongside model parts (`model_volume_needs_slicing`,
PrintObjectSlice.cpp:110) and subtracts them in `slices_to_regions`
(PrintObjectSlice.cpp:403).

**Causality tested before building anything (R514 discipline).** A gated C++
experiment skipping only the *subtraction* (leaving the slicing structure
intact) makes C++ hole-free at low z, matching us. Comparing our unchanged
gcode against that reference:

| metric | vs stock C++ | vs C++ without negatives |
|---|---|---|
| silhouette (object) | 96.67% | **96.73%** |
| Top surface | 1.173 | 1.176 |
| Bottom surface | 0.892 | 0.822 |
| object material | 0.9958 | 0.9961 |

So the missing negative volumes account for **0.06pp of the 2.3pp silhouette
gap** and move Top and Bottom the wrong way. This is a genuine fidelity defect
worth fixing on its own merits — the five connector holes are functional
assembly features and we fill them solid — but it is NOT the cause of any
failing parity metric.

FIRST ATTEMPT RETRACTED: gating `model_volume_needs_slicing` to return false
for negative volumes makes the C++ slicer exit 1 with no gcode; the comparison
run that appeared to show "no change" had silently read the previous run's
stale output file. Same class of trap as R494 — always check the exit code and
that the artefact was actually rewritten.

**Verified:** Majora 065302cb, benchy 5a34af50, cube ab415621 byte-identical;
eight guard tests green; C++ submodule restored clean.

**New discipline (R515): a probe that returns NOTHING is a failed run, not a
negative result.** Confirm the process exited 0 and rewrote its artefact before
reading any comparison built on it.

## R516 — the silhouette gap is REAL, one-sided, and a sub-0.2 mm boundary rim

Measurement round, no code change. Four things established.

**1. The metric is exact on Majora (control never run before).** Two independent
stock C++ runs score **silhouette 100.00%, wall lines 99.99%, object material
1.0000**. So Majora's 96.67% is a genuine divergence, not closing/rasterisation
noise on a geometrically hard model.

**2. The gap is one-sided by ~100x.** Per-layer directional areas over all 634
comparable layers: union 1,198,397 mm²; **rust-only 39,556 mm² (3.30%)**;
cpp-only 405 mm² (0.03%). We cover area C++ does not, essentially everywhere —
C++ covers almost nothing we miss.

**3. It is a thin boundary rim, not blobs.** At the worst layers (z26.7 93.87%,
z70.2 94.72%) the rust-only mask is ~700 connected components; eroding it by a
single 0.2 mm cell removes 70–78% of the area. The rust extrusions falling
inside it are overwhelmingly **Outer wall** (77/123 mm² at z26.7, 126/158 at
z70.2). Not a global offset: per-layer outer-wall bbox extents agree, median
ΔW 0.0050 mm / ΔH −0.0010 mm over all 656 layers.

**4. It is NOT a chord-rendering artefact.** `semantic_compare`'s raster draws
G2/G3 as chords ("coverage is width-dominated"), and at z26.7 our walls use 408
moves where C++ uses 859 for the same length (1011.5 vs 1013.5 mm) — so longer
chords cutting across concave detail was the obvious suspect. Re-rasterising
with arcs subdivided into 0.2 mm chordlets moves the ten worst layers only
**94.74% → 95.01% (+0.27pp)**. Real geometry, not measurement.

At z26.7 every feature matches within a few percent except **Bridge (rust
337.7 mm / 146 seg vs C++ 179.7 / 62 = 1.88x)** — but bridge segments do not
land in the rust-only cells, so it is a separate thread (global Bridge length
ratio 1.069).

Net: the silhouette failure is "we deposit material a fraction of a line-width
outside C++'s footprint, all round the object", not a missing/extra feature.
Next: measure the signed distance from our outer-wall centreline to C++'s
rather than comparing rasters.

New probes (in the job tmp dir): `silrank.py` (per-layer IoU + directional
areas + bboxes), `silwhere.py` (erosion profile, components, feature
attribution), `silblob.py`, `silmap.py` (ASCII raster diff), `owbbox.py`,
`boxdump.py`, `silcmp2.py` (chord vs arc-subdivided A/B).

**New discipline (R516): run the reference-vs-itself control on the HARD model
before believing a failing geometric metric** — and before dismissing one. The
100.00% C++-vs-C++ result is what makes the 96.67% actionable.

## R517 — wall position and width exonerated; the excess is RAW, not closing

Measurement round, no code change. The R516 rim is now pinned down to a single
quantitative contradiction.

**Wall POSITION is exonerated.** New probe `walldist.py` samples our outer-wall
centreline every 0.1 mm (arcs subdivided) and finds the nearest point on C++'s:

| layer | median | p90 | p99 | within 0.10 mm |
|---|---|---|---|---|
| z26.7 | 0.0210 mm | 0.0761 | 0.0966 | **99.87%** (max 0.294) |
| z70.2 | 0.0129 mm | 0.0621 | 0.9727 | 98.54% (max 2.985) |

The two engines' outer walls are effectively coincident.

**Wall WIDTH is exonerated.** Length-weighted mean width per feature, and the
implied swept band (Σ len×width), object only:

| feature | rustW | cppW | W-rat | band-rat |
|---|---|---|---|---|
| Outer wall | 0.40079 | 0.40227 | 0.996 | 0.987 |
| Inner wall | 0.40104 | 0.40238 | 0.997 | 0.996 |
| Sparse infill | 0.45000 | 0.45000 | 1.000 | 1.003 |
| **OBJECT TOTAL** | | | | **0.997** |

**The excess is in the RAW raster, not the closing.** Sweeping the closing
kernel K at z26.7 (K=20 is the shipped value):

| K | rustCov | cppCov | R-only | C-only | IoU |
|---|---|---|---|---|---|
| **0** | **1733.9** | **1526.2** | **216.0** | **8.3** | **87.13%** |
| 5 | 1997.1 | 1815.7 | 184.5 | 3.1 | 90.62% |
| 20 | 2037.9 | 1916.7 | 123.2 | 1.9 | 93.87% |

The morphological closing HELPS (87% → 94%); it does not create the gap.

**RETRACTED mid-round:** I suspected `raster_layer` of a segment-length bias,
because our chords are 5x longer on median than C++'s (1.937 vs 0.375 mm at
z26.7) and a length-dependent rasteriser would be invisible to R516's
C++-vs-C++ control. Tested directly by pre-splitting every segment into 0.1 mm
pieces and re-rastering: **the area changes by exactly 0.00% for both engines.**
The rasteriser is split-invariant and R516's control stands. (Two of my own
ad-hoc rasterisers disagreed for rust while agreeing for C++; per R475 the
authoritative `semantic_compare` path is the one to trust — the ad-hoc ones
used a different brush.) Arc bulge is also negligible: summed arc-length minus
chord over the whole z26.7 outer wall is 0.75 mm (rust) vs 0.59 mm (C++).

**What remains, stated precisely:** our object deposits **0.997x** C++'s swept
band but covers **1.136x** the unique raw area (z26.7). Equal material over
more distinct cells means **C++'s extrusion paths overlap each other more than
ours do.** That, not outline or width, is the whole silhouette failure.

Next: per-feature pass-count multiplicity per cell on both engines, using the
authoritative rasteriser, to find where C++ double-covers and we do not.

New probes: `walldist.py`, `widthcmp.py`, `rawclose.py`, `rawattr.py`,
`mult.py`, `chordlen.py`, `rastbias.py`.

**New discipline (R517): when two hand-rolled measurements of the same quantity
disagree, the bug is in one of them — settle it against the shipped metric with
an invariance test (split the input; the answer must not move) instead of
reasoning about which is right.**

## R518 — MAJORA IS SEMANTICALLY EQUIVALENT. The silhouette failure was a metric cliff.

**The Majora silhouette "failure" chased through R508-R517 was an artefact of
`semantic_compare.raster_layer`'s own brush.** Fixed; all five checks now pass.

The brush marked cells with

```
r = max(w/2, res); rad = int(ceil(r/res))
if dx*dx + dy*dy > (rad+0.5)**2: continue
```

`rad` is an INTEGER derived from `ceil`, so the disc area jumps discontinuously
wherever `r/res` crosses an integer. At the silhouette's `SIL_RES = 0.2`:

| engine | emitted LINE_WIDTH | r | r/res | rad | disc cells |
|---|---|---|---|---|---|
| C++ | 0.399991 | 0.2 (clamped) | 1.000000 | **1** | **9** |
| Rust | 0.400001 | 0.2000005 | 1.0000025 | **2** | **21** |

A **1e-5 mm** difference — 10 nanometres, physically meaningless — gave our
walls a **2.33x wider rastered band**. At z26.7 all 408 of our outer-wall
segments took rad=2 while 804 of C++'s 859 took rad=1 (same for Inner and
Overhang wall). That is the entire "we cover more area" story: R516's
one-sidedness, R517's "0.997x band but 1.136x unique area", the thin rim, the
~700 components, the wall-dominated attribution.

**Proof.** Nudging every `LINE_WIDTH` in the C++ gcode by +1e-5 mm and comparing
C++ against that copy of itself:

| metric | old brush | fixed brush |
|---|---|---|
| silhouette (object) | **96.83% FAIL**, 515/634 layers <98%, min 93.9% at z26.7 | **100.00%**, 0 layers <98% |

96.83% / 515 layers / min 93.9% at z26.7 is essentially the exact failure we
were chasing (96.67% / 519 / 93.9% at z26.7) — reproduced from a 10 nm width
perturbation of C++ against itself.

**The fix** keeps the same disc shape but tests real distance in mm, so the
brush is continuous in `r` and exact multiples reproduce the old disc:

```
rad = int(ceil((r + res/2)/res)); thr2 = (r + res/2)**2
if (dx*res)**2 + (dy*res)**2 > thr2: continue
```

**Validation** — the fixed metric is neither blind nor self-fulfilling:

| check | result |
|---|---|
| C++ run A vs C++ run B | 100.00% (unchanged) |
| C++ vs C++ widths +1e-5 mm | 100.00% (was 96.83%) |
| C++ stock vs C++ without negative volumes (a REAL geometric difference) | 99.93%, wall lines 99.65% with 10 layers <95% — still detected |
| Benchy rust vs C++ | 99.99%, SEMANTICALLY EQUIVALENT (unchanged) |
| painted cube self-compare | 100.00% |

**MAJORA VERDICT — all five checks pass:**

```
object-only (no tower): 72448.9 / 72673.3 = 0.9969
wipe tower (purge)    : 63297.0 / 63637.5 = 0.9947
[PASS] object material within 1%       0.9959
[PASS] layer count equal               657=657
[PASS] per-layer material mean<5%      4.47%
[PASS] per-feature material <35%       Top surface 1.173
[PASS] silhouette area-wtd >=99%       99.37%   (was 96.67%; min per-layer 98.0%, 0/634 below 98%)
==> SEMANTICALLY EQUIVALENT
```

**No engine change.** Only `scripts/semantic_compare.py` moved; majora
065302cb, benchy 5a34af50, cube ab415621 all byte-identical.

**WALL LINES moved 95.71% -> 94.69%** (it is not one of the five verdict checks;
it uses `res=0.15`, where `r/res=1.333` sits far from a cliff, which is exactly
why it never showed this bug). The small change comes from the half-cell
allowance now being applied consistently. Remaining real per-feature gaps are
untouched and still open: Top 1.173, Bridge 1.069 length, Overhang 1.048,
Bottom 0.892.

**New discipline (R518): a reference-vs-itself control CANNOT detect a metric
bug whose trigger is a DIFFERENCE between the two inputs.** R516's C++-vs-C++
100.00% and R517's split-invariance test were both sound and both blind here,
because each fed the metric two inputs with identical widths. To validate a
COMPARATIVE metric, perturb one input by a physically-irrelevant amount — a
value below the tolerance you care about — and require the score not to move.
Add that control before trusting any geometric comparison.

## R519 — Bottom surface 0.892 characterised: ONE layer, 13 E, 0.01% of the print

Measurement round, no code change.

**Bottom surface exists on exactly one layer of Majora — z=0.30, the first
layer.** Both engines: 1 layer with material, 0 layers unique to either.

| quantity | rust | C++ | ratio |
|---|---|---|---|
| E | 106.11 | 118.90 | 0.892 |
| path length | 1984.2 | 2231.3 | 0.889 |
| E per mm | 0.05348 | 0.05329 | 1.004 |
| segments | 1742 | 1191 | 1.46 |

E/mm matches to 0.4%, so the FLOW is right; we simply lay 11% less bottom path.
In absolute terms the whole feature is **13 E out of 135,746 (0.01% of the
print), on 1 of 657 layers** — it passes the per-feature check comfortably and
is not worth further rounds ahead of the remaining asks.

`BOTTOM_FLOW` is NOT a missing port (R486): it is default-ON and already routes
the first-layer fill through `region.flow()` so it picks up
`initial_layer_line_width` (fill/mod.rs:771).

**A caveat on my own number.** I measured "implied spacing" as
closed-region-area / length and got 0.659 mm (rust) vs 0.477 (C++). That figure
is contaminated — a k=10 (2 mm) closing inflates whichever engine's lines are
more fragmented, and ours are (1742 segments vs 1191 for less total length). It
is NOT trustworthy evidence of a spacing defect. What is solid: matching E/mm
with 11% less length, so the difference is bottom AREA and/or line spacing, and
separating those needs a generator-internal probe (R495 — print the layer-0
bottom surface area and fill spacing from both engines), not gcode
reconstruction.

**Neighbouring observation (unexplained, first two layers only).** Per-feature
length ratios rust/C++ over the opening layers:

| z | Outer wall | Inner wall | Bottom |
|---|---|---|---|
| 0.30 | 0.888 | **1.453** | 0.889 |
| 0.60 | 0.784 | **1.258** | — |
| 0.90 | 0.851 | 0.908 | — |
| 1.20 | 0.964 | 0.974 | — |
| 1.50+ | ~0.95 | ~0.97 | — |

Our first two layers carry noticeably more inner wall and less outer wall, then
the ratios settle. Total layer-1 path is 4010.9 vs 4085.0 (0.982), so material
is conserved — it is a classification/redistribution difference confined to the
first two layers. Recorded, not chased.

New probes: `featsplit.py` (per-layer split of one feature, with
only-rust/only-cpp layer sets), `wallband.py` (per-feature length ratios over a
z band), `featregion.py` (closed-region area + implied spacing for one feature
at one layer — read with the caveat above).

## R520 — ask #3 profiled: the remaining time gap is SERIAL gcode export

First real profile of the Rust engine (macOS `xctrace` Time Profiler, Majora,
93,961 samples). No code change this round.

**Wall clock (median of repeated runs, warm):**

| fixture | rust | bambu | ratio |
|---|---|---|---|
| Benchy | 2.32 s | 1.93 s | 1.20x |
| Majora | 18.3 s | 15.9 s | 1.15x |

**Where our time goes (Majora, `SLICE_PHASE_TIMING=1`, total 17.86 s):**

```
process 11.47   perimeters(+slice) 5.48 (47.9%)   infill 5.66 (49.4%)   simplify 0.30
  prepare_infill 3.35: discover_vertical_shells 1.77 (52.8%), bridge_over_infill 0.83,
                       process_external_surfaces 0.49, detect_surfaces_type 0.26
export_gcode 6.39   generate 5.02   post-process(cooling/zsmooth) 0.78   assemble+write 0.59
```

**Where C++'s time goes** (derived from gaps between its own timestamped log
lines, total 15.77 s): load/config ~1.5, slicing volumes 0.64, walls ~5.0
(2.75 + 2.29), solid/shells/external/bridge ~1.4, fill ~2.57, **export ~3.66**
(3.33 + 0.33 post_process).

**The gap is export.** Ours 6.39 s vs C++'s ~3.66 s — a **+2.7 s delta against a
+2.1 s total delta**, i.e. the export phase accounts for the entire remaining
gap and we are marginally ahead elsewhere.

**Root cause identified.** C++ `GCode::process_layers` runs a TBB parallel
pipeline over layers — `tbb::parallel_pipeline(12, generator & spiral_mode &
parsing & cooling & write_gocde & output)` (GCode.cpp:3396-3400, also :3416 for
the calculate_layer_time variant). Our export generates layers **serially**.
That is the specific algorithmic difference, and it is a faithful-port target
rather than a micro-optimisation.

**Profile caveat that matters (and nearly misled me).** A CPU-sample profile
counts samples per thread, so it OVER-weights rayon-parallel phases and
UNDER-weights a serial phase. 93.14% of samples sit under rayon worker threads;
export barely appears. Read against wall-clock stage timings, not on its own —
the profile says "Arachne is hot", the clock says "export is the gap".

**What the profile does say** (inclusive %, of total CPU samples):

| symbol | incl. |
|---|---|
| `boostvoronoi::builder::Builder::build` | 24.97% |
| `PerimeterGenerator::generate_arachne` | 31.77% |
| `WallToolPaths::generate` | 20.93% |
| `SkeletalTrapezoidation::construct_from_polygons` | 15.23% |
| `Layer::make_fills` | 15.57% |

Top SELF symbol is `boostvoronoi::extended_scalar::extended_int::ExtendedInt::mul_other`
at **10.16%** — extended-precision integer arithmetic inside the Voronoi
predicates. Tempting, but C++ spends a comparable ~5.0 s on walls against our
5.48 s, so Arachne is expensive in BOTH engines and is NOT the parity gap.

**Two incidental observations, unverified:**
- The binary contains and hotly uses TWO separate ClipperLib copies —
  `ClipperZSys::ClipperLib::*` (18.75% incl.) and a plain `ClipperLib::*`
  (14.95% incl.). Whether one is redundant is worth a look.
- ~9% of self time is allocator/`memmove`/`memset` (`_xzm_free` 3.90%,
  `_xzm_xzone_malloc*` 2.29%, `_platform_memmove` 1.61%, `_platform_memset`
  1.45%) — allocation churn. Note R484 already measured pre-sizing vectors as a
  NEGATIVE, so do not retry that blindly.
- `__findenv_locked` (getenv, i.e. `faithful_gate`) is 0.77% self. R484 measured
  caching it as a NEGATIVE; the profile confirms the prize is small.

**Next:** port the layer-parallel export pipeline. Ordering is the constraint —
the C++ pipeline is serial-in / parallel-middle / serial-out, so a rayon port
must preserve emission order exactly or the gcode changes.

**New discipline (R520): profile in the same units as the goal.** The goal is
wall clock; a per-thread CPU profile answers a different question and will point
at whichever phase has the most threads.

## R521 — seam-placer visibility parallelised: export generate 5.02 -> 4.13 s

R520 put the whole remaining time gap in `export_gcode`. This round found why,
using a **per-thread** slice of the R520 profile (`$D/profmain.py`) — the fix for
R520's own caveat, since a serial phase is invisible in whole-process CPU
percentages.

**Main thread (the serial timeline), 6446 samples:**

| symbol | % of main thread |
|---|---|
| `Print::export_gcode` | **57.57** |
| `SeamPlacer::init` | **22.09** |
| `extrude_collection` | 18.57 |
| `extrude_perimeters_entities` | 14.80 |
| `extrude_entity` | 11.25 |
| `compute_global_occlusion` | 10.75 |
| `raycast_visibility` | 9.37 |

`SeamPlacer::init` is ~38% of export. Cause: **C++ SeamPlacer.cpp has SIX
`tbb::parallel_for` sites (:157, :933, :955, :966, :1430); our port had ONE.**
Our source even says so in comments — "C++ runs under `tbb::parallel_for`;
ported serially with identical results".

**Landed:** `calculate_candidates_visibility` (SeamPlacer.cpp:955) now runs
under rayon. Each point's visibility is a pure function of
`(mesh_samples_tree, position)` written to its own slot, so the result is
order-independent and byte-identical by construction.

| | before | after |
|---|---|---|
| export_gcode generate | 5.02 s | **4.13 s** |
| export_gcode total | 6.39 s | **5.50 s** |

**Reverted as INERT:** parallelising `gather_seam_candidates`
(SeamPlacer.cpp:933) moved export generate by **exactly 0.000 s** (4.131 ->
4.130) — that loop is not on the export critical path. Reverted rather than
kept; the serial form now carries a comment recording the measurement so it is
not retried.

**Honest accounting of the total.** The instrumented export sub-phase improved
repeatably by 0.89 s. Majora WALL clock moved only ~18.3 -> ~18.1 s (medians of
3), and the phase timer showed `process` drifting 11.47 -> 12.05 s across the
session — a phase this change does not touch, so that drift is session noise
(thermal/load), not a regression. Do not bank the full 0.89 s as end-to-end
until it is re-measured on a quiet machine. Benchy is unchanged (2.32 s) as
expected: single-material and small, so seam visibility is cheap there.

**Verified:** majora 065302cb, benchy 5a34af50, cube ab415621 all
byte-identical; eight guard tests green.

**Remaining in export:** the other four C++ `parallel_for` sites in SeamPlacer
(:157, :966, :1430 and the `po->layers()` loop at :933 — the last measured
inert here), and the stage-overlap pipeline (bounded at ~1.4 s, R520).

**New discipline (R521): to profile a SERIAL phase, slice the profile by
thread.** Whole-process CPU percentages hid a 22%-of-main-thread hotspot at
0.3% of total samples.

## R522 — seam-placer overhangs parallelised: export generate 4.13 -> 3.73 s

Second SeamPlacer `tbb::parallel_for` site ported (SeamPlacer.cpp:966,
`calculate_overhangs_and_layer_embedding`).

**The subtlety.** This loop is NOT embarrassingly parallel: each layer needs
`prev_layer_distancer`, the `PerimeterDistancer` of the layer below. C++ handles
it by parallelising over layer RANGES and seeding each range from
`r.begin() - 1` (SeamPlacer.cpp:967-970). Ported with `par_chunks_mut`: chunk
covering `[base, base+len)` rebuilds the distancer for layer `base - 1` before
iterating, so every layer observes exactly the `prev_layer_distancer` it saw
serially. The only extra cost is one distancer per chunk — which is what C++
pays too.

| | R520 | R521 | **R522** |
|---|---|---|---|
| export generate | 5.02 s | 4.13 s | **3.73 s** |
| export total | 6.39 s | 5.50 s | **5.11 s** |
| Majora wall (median of 3) | ~18.3 s | ~18.1 s | **17.47 s** |
| vs C++ 15.9 s | 1.15x | — | **1.10x** |

Cumulative over R521+R522: **export generate -26%**, and the gain is now
visible end-to-end in wall clock, not only in the instrumented sub-phase.

**Verified:** majora 065302cb, benchy 5a34af50, cube ab415621 all
byte-identical; eight guard tests green. Byte-identity is the correctness
argument here — chunk-boundary seeding either reproduces the serial
`prev_layer_distancer` exactly or the gcode changes, and it does not.

**Remaining in export:** SeamPlacer.cpp:157 and :1430 (the two untried
`parallel_for` sites), the KD-tree builds (`build_points_tree`,
`build_mesh_samples_tree` — 6.58% self on the main thread), and the
stage-overlap pipeline (bounded ~1.4 s, R520). Still measured INERT and not to
be retried: `gather_seam_candidates` (:933).

## R523 — seam KD-tree was rebuilt 59,677 times; cached: export generate 3.73 -> 3.02 s

Re-recorded the profile (the R520 one predated two landed changes). It confirms
the campaign is working — `SeamPlacer::init` fell from **22.09% to 12.19%** of
main-thread time — and moved the top SELF cost to
`KDTreeIndirect::build_recursive` at **7.18%**.

**Counted before building anything (R501).** New `KDCOUNT=1` probe:

```
[KD points_tree builds=59677 pts=88577213]
```

**59,677 tree builds over 88.6 M points, for a 657-layer print.** C++ builds
`points_tree` ONCE per layer (SeamPlacer.cpp:944-945) and stores it in the
layer — 657 builds. We were doing **91x** the work, because our port builds the
tree on demand at each query (a documented compromise: `KDTreeIndirect` borrows
its coordinate closure from `points`, so the tree cannot simply be a field).

**Fix.** The tree is just `nodes: Vec<usize>` plus a cheap closure, and all the
cost is in `build()`. Added `KDTreeIndirect::from_nodes` / `nodes()`, and cached
the built node array per layer in `LayerSeams::points_tree_nodes`
(`std::sync::OnceLock`), reconstructing the tree cheaply around it. Candidate
positions are frozen after `gather_seam_candidates`, so the cache cannot go
stale — and byte-identity proves it.

| | R520 | R521 | R522 | **R523** |
|---|---|---|---|---|
| export generate | 5.02 s | 4.13 s | 3.73 s | **3.02 s** |
| export total | 6.39 s | 5.50 s | 5.11 s | **4.53 s** |

Cumulative: **export generate -40%, export total -29%.**

**WALL CLOCK NOT MEASURABLE THIS ROUND — and I nearly misread it.** Mid-round,
runs jumped to 64-73 s. That was not the change: `uptime` showed **load average
51.9**, with OrbStack at 309% CPU and Chrome at ~200%. Sub-phase timings taken
once the stray job finished are stable and consistent (generate 3.022-3.090
across 4 runs, reported as the minimum). Majora wall must be re-measured on a
quiet machine before the R522 figure of 17.47 s is updated.

**Verified:** majora 065302cb, benchy 5a34af50, cube ab415621 byte-identical;
eight guard tests green.

**Correction to the R523 plan:** SeamPlacer.cpp:157 was already ported (it is
the one pre-existing rayon site, `raycast_visibility`). Of C++'s six
`parallel_for` sites the only untried one left is **:1430**.

**New discipline (R523): before optimising a hot function, COUNT how many times
it runs and compare that count to C++.** The profile said "KD-tree build is
7.18% self"; the counter said "you are doing it 91x more often than C++". The
second framing is the one that leads to a fix. And: when timings explode, check
`uptime` before blaming the diff.

## R524 — SeamPlacer is exhausted; two targets sized and DROPPED

Measurement round, no code change. Two of the three remaining SeamPlacer
targets were killed by sizing them before writing code (R519), and the
post-R523 profile confirms the subsystem is done.

**(1) `build_mesh_samples_tree` — NOT a repeat of the R523 bug.** It has exactly
ONE call site (`calculate_candidates_visibility`), so it is built once, matching
C++'s persistent `mesh_samples_tree` in `GlobalModelInfo`. No action.

**(2) SeamPlacer.cpp:1430 (`pick_seam_point` over layers) — DROPPED, too
small.** It is a clean per-layer-independent loop and would parallelise easily,
but it does not appear in the profile at all: the largest related symbol is
`SeamComparator::is_first_better` at **0.043% of total samples** (~0.6% of main
thread, order 0.05 s). Porting it would be faithful-to-C++ but perf-inert.
**All six C++ `parallel_for` sites are now accounted for**: :157 already ported,
:933 measured inert (R521), :955 done (R521), :966 done (R522), :1430 dropped
here as negligible.

**(3) Re-profile confirms R523 worked.** `KDTreeIndirect::build_recursive` has
**vanished from the main-thread top-SELF list** (was 7.18%). What remains of
`SeamPlacer::init` (13.70%) is almost entirely `compute_global_occlusion`
(13.06%) → `raycast_visibility` (11.49%) → AABB-tree ray hits (**10.04% self**,
the new top cost) — and `raycast_visibility` is ALREADY parallel (the
pre-existing rayon site). That cost is inherent and C++ pays it too.

New main-thread SELF leaders: AABB ray-hit 10.04%, `_xzm_free` 4.39%,
`_platform_memmove` 3.72%, Clipper internals ~13% combined, `__findenv_locked`
(getenv via `faithful_gate`) 2.15%, `StrSearcher::new` 1.69%. Allocator +
memmove/memset together are ~11.5%.

**The remaining export lever is the stage-overlap pipeline, and its arithmetic
has IMPROVED.** R520 bounded it at ~1.4 s when generate was 5.02 s. Now:
`max(generate 3.02, post 0.78, write 0.59) = 3.02` against an export total of
4.53 — so perfect overlap would save **~1.5 s**, taking Majora to roughly
**16 s vs C++'s 15.9 s**. That is the finish line for ask #3. The obstacle is
real: our export post-processing is stateful across layers
(`GCodeEditorState`, `SmoothCalculator`), so this is a genuine restructuring,
not a wrapper.

**WALL CLOCK STILL NOT TRUSTWORTHY.** Load fluctuated 6.7 -> 27.6 during the
round; a C++ control run measured 22.76-25.40 s against its usual 15.9 s, i.e.
the reference itself was inflated ~50%. Rust min was 17.79 s (consistent with
R522's 17.47 s). **When the machine is loaded, interleave A/B runs rather than
running all of one engine then all of the other** — the block-sequential
measurement taken here is worthless for a cross-engine ratio.

**New discipline (R524): size a faithful-port target in the profile BEFORE
porting it — "C++ parallelises it" is a reason to look, not a reason to do it.**
:1430 is a legitimate parity gap that is worth exactly nothing in time.

## R525 — export pipeline: architecture mapped, and TWO of my own claims corrected

Analysis round, no code change. The goal was to port C++'s stage-overlap
pipeline; the round instead established what the port actually requires and
corrected two errors I had been carrying.

**Correction 1 — our export is BATCH, not streaming.** The generate loop writes
every layer into a single `writer`; `writer.finish()` returns ONE string for the
whole print; cooling then splits that string on `; CHANGE_LAYER` and processes
layers; then assemble+write runs a full `GCodeProcessor` pass over the 66 MB
body to compute the accel-aware header time. So the three "stages" are three
sequential whole-print passes. C++'s are per-layer filters. **The port is not
"wrap the loop in a pipeline" — it requires `generate` to emit per-layer chunks
as it goes.** That is the real cost of this work and it was not stated before.

**Correction 2 — the ZSMOOTH barrier does NOT apply to Majora, so the ~1.5 s
estimate stands.** I had started to conclude that C++ also has a global barrier
(`smooth_calculator.smooth_layer_speed()` between two pipelines, GCode.cpp:3400
and :3416) and was about to revise the prize down to ~0.9 s. Then I probed the
actual config value (R503): Majora's 3MF has
`"z_direction_outwall_speed_continuous": "0"`, so **neither engine takes the
two-pass path**. C++ runs the single
`parallel_pipeline(12, generator & parsing & cooling & write_gocde & output)`
at GCode.cpp:3398, and full stage overlap is available. Our own `zsmooth` flag
is correspondingly false. **Prize confirmed at `max(3.02, 0.78, 0.59) = 3.02`
vs export total 4.53 => ~1.5 s**, which would put Majora near 16 s against
C++'s 15.9 s.

**Scope judgement.** That refactor touches writer buffer ownership, the cooling
state machine's input contract, and final assembly, with byte-identity at risk
throughout — plausibly several rounds. Against that: ask #3 has come from 3.0x
at campaign start to **1.10x**, while asks #1 (main.cpp -> src/*.rs
correspondence) and #4 (file-layout mirroring for C++ maintainers) are explicit
user asks that have gone untouched for many rounds. Next round pivots to those;
the pipeline is documented here as the known remaining lever with its true cost
and its true payoff, ready to pick up.

**New discipline (R525): when a config flag selects between two code paths,
read the fixture's actual value before reasoning about either.** I nearly
recorded a wrong prize (0.9 s instead of 1.5 s) by assuming Majora took the
zsmooth path it does not take — the same class of error as R503.

## R526 — asks #1 and #4: correspondence re-verified, file-layout map written

Pivot round (see R525's scope note). No engine change; both remaining user asks
that had gone untouched are now addressed with measured, checkable numbers.

**Ask #1 — `main.cpp` -> `src/*.rs`.** Re-verified `main-cpp-correspondence.md`
mechanically rather than by eye:

- `main.cpp` is unchanged (still 84409 bytes).
- It has **13 top-level definitions, and all 13 appear in the table (100%)**.
- **Every Rust symbol cited in the table still exists** at the path given
  (`JobConfig`, `resolve_config_refs`, `normalize_single_filament_stl_config`,
  `full_fff_config`, `slice_to_gcode`, `slice_3mf_to_gcode`, `load_3mf`,
  `Print::apply`/`validate`/`export_gcode`, `SlicingStatus`, …).

So the doc's GAP rows are current, not stale — the multi-nozzle trio
(`apply_explicit_nozzle_mapping`, `reassign_objects_to_master_nozzle`) and the
stdout event protocol (`emit_event` family) remain the real ask-#1 gaps. Doc
re-stamped R387 -> R526.

**Ask #4 — file layout for C++ maintainers.** New
`docs/file-layout-correspondence.md`. Measured, not asserted:

| | |
|---|---|
| C++ translation units (distinct `.cpp`/`.hpp` stems) | 276 |
| Mirrored by a same-named Rust file | **~273 (99%)** |
| Subdirectories mapping 1:1 | **13 of 13** |
| Deliberate omissions | 3 |

The rule is `<Dir>/<CamelCase>.{cpp,hpp}` -> `<dir>/<snake_case>.rs`. The three
omissions are `GCodeSender` (printer serial I/O, out of scope for an offline
CLI), `clipper` (we LINK the same vendored ClipperLib via `clipper_z_sys`
instead of porting it — porting would risk the exact geometric divergence the
binding avoids), and `Format/format.hpp` (header aggregator; `format/mod.rs`).

Two files that a naive transform reports as missing are actually present under a
different name, and the doc calls them out so nobody re-ports them:
`Format/3mf.cpp` -> `format/three_mf.rs` (Rust identifiers cannot start with a
digit) and `Support/TreeSupport3D.cpp` -> `support/tree_support_3d.rs` (digit
grouping). **Both of these fooled my own first-pass matcher** — it reported them
as unported before the alternates were checked.

**New discipline (R526): audit a naming convention with a script, then verify
the script's misses by hand — a mechanical CamelCase->snake_case transform
produces false "missing" hits on digits (`3mf`, `TreeSupport3D`), and reporting
those as gaps would have sent someone to re-port existing files.**

## R527 — the multi-nozzle trio is DEAD CODE for every fixture (ask #1)

Investigation round, no code change. Ask #1's largest remaining item turns out
to be unportable-because-unverifiable, and the guards prove it rather than
suggest it.

**Majora carries every multi-nozzle key**, which is why the trio looked live:
`filament_map`, `filament_map_mode = "Auto For Flush"`, `filament_nozzle_map`,
`physical_extruder_map`. But the values say otherwise:

```
physical_extruder_map = ['0']      <-- ONE physical extruder
nozzle_diameter       = ['0.4']    <-- ONE nozzle
filament_map          = ['1' x 8]  <-- uniform, no cross-nozzle split
printer_model         = Bambu Lab X1 Carbon
```

It is a single-nozzle 8-filament AMS job, not an H2D dual-nozzle job.

**Both C++ functions bail on their own guard:**

| function | guard | Majora |
|---|---|---|
| `apply_explicit_nozzle_mapping` (:229) | `filament_count < 2 \|\| extruder_count < 2` where `extruder_count = nozzle_diameter.size()` | **1 -> returns false** |
| `reassign_objects_to_master_nozzle` (:291) | `extruder_count < 2` where `extruder_count = physical_extruder_map.size()` | **1 -> returns** |

and the second is only *called* when the first returned true (main.cpp:1360-1366).
Benchy (STL, single filament) and the painted cube are single-nozzle too.

**Conclusion:** porting the trio would produce zero G-code change on all three
fixtures and could not be validated against anything. Recorded in
`main-cpp-correspondence.md` as a deliberate, evidenced gap, with the concrete
prerequisite for any future port: build an H2D-class fixture with two
`nozzle_diameter` entries, `physical_extruder_map` of size 2, and a
`filament_nozzle_map` that genuinely splits filaments across both nozzles — and
confirm the C++ engine slices it, so a reference exists.

This is a **coverage** gap, not a correctness one.

**New discipline (R527): a config key being PRESENT does not mean the code path
is LIVE — read the values and the guard.** All four multi-nozzle keys are in
Majora's 3MF; both consumers still return immediately on the first `if`.

## R528 — stdout event protocol ported (ask #1)

`src/events.rs`: faithful port of `main.cpp:55-137` — the `[[SLICER_EVENT]] `
prefix with one flushed JSON object per line, the three tag mappers
(`slicing_notification_tag`, `warning_level_tag`, `string_exception_tag`),
`emit_event` / `emit_status_warning` / `emit_validation_event`, and
`emit_slicing_error` for the `slicing_error` events at :1519-1551. Wire format
**captured from a real `--engine bambu` run**, not inferred. Tag mappers
unit-tested (9 tests pass).

**Measured: what C++ emits for our fixtures.**

| fixture | events |
|---|---|
| Benchy | **1** — a `SlicingNeedSupportOn` warning ("floating regions"), scope `object`, step 5, `non_critical` |
| Majora | **0** |

**Honest status: the protocol is ported, the one live event is not reproducible
yet**, for two separate reasons:

1. `emit_status_warning` needs our `set_status_callback` widened from
   `(percent, message)` to the full `SlicingStatus` (flags / message_type /
   warning_level / warning_step).
2. The warning SOURCE is unported: `PrintObject::is_support_necessary()`
   (PrintObject.cpp:3847) is 14 lines but delegates to
   `TreeSupport::detect_overhangs(true)` and reads `has_sharp_tails` /
   `has_cantilever` / `max_cantilever_dist`. Our `support/mod.rs` has a
   simplified `detect_overhangs` computing none of them.
   `print_object.rs:3683` already records this as a deliberate omission
   (warning-only, no geometry change).

**Deliberately NOT wired:** `emit_validation_event` and `emit_slicing_error`.
All three fixtures slice cleanly and C++ emits neither, so hooking them would
be untestable — and C++ splits `slicing_error` by `phase` (`process` vs
`export_gcode`) while our `slice_to_gcode` spans both, so any phase mapping
would be invented. Emitting a *wrong* phase is worse than emitting nothing.

**Verified:** majora 065302cb, benchy 5a34af50, cube ab415621 byte-identical
(the module is CLI-layer and currently emits nothing on the slice path); 9 unit
tests pass.

**New discipline (R528): port the protocol from a CAPTURED sample, and refuse to
invent the fields you cannot observe.** The wire format came from a real C++
run; the `phase` value for our combined slice+export path could not be
observed, so that call site was left unwired rather than guessed.

## R529 — `is_support_necessary()` sized and DECLINED; next target sized instead

Investigation round, no code change.

**The port was sized before being attempted (R519/R524), and it does not pay.**
`PrintObject::is_support_necessary()` is 14 lines, but the work is in
`TreeSupport::detect_overhangs(bool)`:

| | |
|---|---|
| `detect_overhangs` size | **653 lines** (TreeSupport.cpp:661-1313) |
| blocked on | `TreeSupportData` + TBB concurrent arena |
| our own note | `support/tree_support.rs:1128-1150` lists **9** `TreeSupport` methods transitively blocked on that arena, `detect_overhangs` among them |
| cheaper route? | **none** — `Layer::sharp_tails` exists as a field (`layer.rs:1215`) but is only ever CLEARED (`print_object.rs:4070`); the populate site is inside `detect_overhangs` (TreeSupport.cpp:988, :1254) |
| total payoff | **one stdout warning line on Benchy, zero G-code change** |

Declined and recorded in `main-cpp-correspondence.md`. The R528 transport stays
in place, ready if tree support is ever ported for its own sake.

**Next target sized in the same round: the GCodeProcessor reserved tags.** These
are a *real G-code content* difference, not a status channel — C++ emits 3,655
`; WIPE_TOWER_START`/`END`, 2,723 `; CP TOOLCHANGE START`/`END`, 209
`; CP EMPTY GRID START`/`END` and 2,723 `; CP_TOOLCHANGE_WIPE`; we emit **zero**.
Crucially they are emitted from `WipeTower.cpp` at :2067, :2724, :3271, :3606,
:3770 — the tool-change / finish-layer / empty-grid paths **we have already
ported** (R419-R445). So this is bounded and localised in
`gcode/wipe_tower.rs`, and exactly countable against C++.

**It will change our G-code** (added comment lines), so it needs a deliberate
re-baseline of all three hashes — the same procedure as R444's arachne-width
re-baseline. That is the R530 target.

**New discipline (R529): when a port is declined, record the SIZE and the
blocking dependency, not just "GAP".** "653 lines behind a TBB arena, for one
stdout line" is a decision anyone can re-evaluate; "unported" is not.

## R530 — GCodeProcessor reserved tags, part 1: the tool-change block

First real G-code CONTENT addition since R444. C++ emits ~9,300 reserved-tag
lines that we emitted **zero** of; `GCodeProcessor` consumes them to segment the
preview.

**Ground truth first (R504).** Counted in the real C++ Majora output rather than
trusting the source list — three of the C++ tag sites are dead for this config:

| tag | C++ count |
|---|---|
| `; WIPE_TOWER_START` / `_END` | 3655 / 3655 |
| `; CP TOOLCHANGE START` / `END` | 2723 / 2723 |
| `; CP_TOOLCHANGE_WIPE` | 2723 |
| `; CP EMPTY GRID START` / `END` | 209 / 209 |
| `; CP TOOLCHANGE UNLOAD` / `LOAD` / `WIPE` | **0** — not ported (R501) |

**Ported this round:** the four tags of `tool_change_new` (WipeTower.cpp:3270,
3288, 3328, 3341), into our `tool_change` — which already cites
`WipeTower.cpp:3271 (tool_change_new)` as its source, so the mapping is exact.

| tag | ours | C++ |
|---|---|---|
| `; CP TOOLCHANGE START` / `END` | **2721** | 2723 |
| `; WIPE_TOWER_START` / `_END` | **2721** | 3655 |

**Two honest gaps, both understood:**

1. **2721 vs 2723.** Our `tool_change()` runs 2,721 times while the emitted `T`
   count is 2,723 (R439 matched that to C++ exactly). Two tool changes are
   therefore emitted outside `tool_change()`. Pre-existing accounting, not
   introduced here — flagged for the next round.
2. **WIPE_TOWER 2721 vs 3655.** The remaining 934 come from `finish_layer_new`
   (WipeTower.cpp:3550/3721), along with all 209 `CP EMPTY GRID` pairs
   (:3606/:3643, :3770/:3814, :3910) and the 2,723 `CP_TOOLCHANGE_WIPE`
   (:3961). Deliberately left for R531 — our tower block count (3,377) differs
   from C++'s (3,655), so those sites need their own count reconciliation rather
   than a blind insert.

**The diff was verified before re-baselining, not after.** Added lines are
exactly 8 per tool change (2 for the `CP TOOLCHANGE START` pair, 1 + 1 for the
WIPE_TOWER pair, 4 for the `CP TOOLCHANGE END` + separator + two blanks, all as
C++ emits them): 2721 x 8 = **21,768 = the exact line-count delta**. `diff`
reported 8 spurious `<` lines, which are re-alignment artefacts — the two
patterns it flagged are present **2721/2721** and **13812/13812** in both files,
so nothing was removed.

**Semantic verdicts are byte-for-byte unchanged** (object material 0.9959,
layers 657=657, per-layer 4.47%, Top 1.173, silhouette 99.37%, still
SEMANTICALLY EQUIVALENT) — comments cannot alter extrusion, and this proves it.

**DELIBERATE RE-BASELINE — majora `065302cb` -> `b7348303`.**
**benchy `5a34af50` and cube `ab415621` are UNCHANGED** (neither reaches the
tower tool-change path). Eight guard tests green.

**New discipline (R530): when a change adds output, verify the delta
ARITHMETICALLY (lines added = N x per-site count) and re-check any `diff`
oddity by counting the specific lines — `diff` on a 2M-line file reports
re-alignment noise that looks like deletion.**

## R531 — reserved tags part 2, and the "2 toolchange" gap resolved

**The 2721-vs-2723 gap is NOT a tag bug — we simply perform 2 fewer tool changes
than C++, uniformly.** Counting every toolchange-adjacent marker shows both
engines keep the SAME internal relationship, offset by a constant 2:

| counter | ours | C++ | delta |
|---|---|---|---|
| `^T[0-9]` | 2725 | 2727 | −2 |
| `^M620 S` | 2723 | 2725 | −2 |
| `; CP TOOLCHANGE START` | 2721 | 2723 | −2 |

T = CP + 4 and M620 = CP + 2 on **both** sides, so nothing is emitted "outside
`tool_change()`" as R530 hypothesised — that framing is retracted. It is a
0.07% tool-change count difference living in ToolOrdering/tower planning, not in
the tag layer. Not chased: the tower is closed (R506, 0.9947) and this is well
inside the "do not grind without a new mechanism" line.

**Ported:** the `finish_layer_new` WIPE_TOWER pair (WipeTower.cpp:3550 and
:3721 — both unconditional, one at the top after writer setup, one just before
the material accounting) into our `finish_layer`, which runs exactly once per
layer.

**Prediction stated BEFORE running (R519): 2721 + 656 = 3377.** Measured
**3377**, and the line delta was **1312 = 656 x 2** exactly.

| tag | ours | C++ |
|---|---|---|
| `; WIPE_TOWER_START` / `_END` | **3377** (was 2721) | 3655 |
| `; CP TOOLCHANGE START` / `END` | 2721 | 2723 |
| `; CP EMPTY GRID START` / `END` | 0 | 209 |
| `; CP_TOOLCHANGE_WIPE` | 0 | 2723 |

The residual 3655 − 3377 = **278 is exactly our known tower block-count
difference** (C++ 3,655 blocks vs our 3,377) — i.e. the tag count now tracks our
block count precisely, which is the correct outcome. Closing it would mean
changing the tower's block structure, not adding tags.

**Still open:** `CP EMPTY GRID` (three separate C++ branches at :3606/:3643,
:3770/:3814, :3910) and `CP_TOOLCHANGE_WIPE` (:3961, which carries a
` CT<n>` suffix — read it before porting). Both need branch analysis to predict
a count, so they were deliberately left rather than blind-inserted.

**Verified additive:** sample patterns unchanged across the diff
(`; Tool change from` 2721/2721, `; CP TOOLCHANGE END` 2721/2721,
`G1  X219.729   E1.8473` 13812/13812). Semantic verdicts identical — object
material 0.9959, layers 657=657, per-layer 4.47%, Top 1.173, silhouette 99.37%,
still SEMANTICALLY EQUIVALENT. Eight guard tests green.

**DELIBERATE RE-BASELINE — majora `b7348303` -> `89377938`.**
**benchy `5a34af50` and cube `ab415621` UNCHANGED** (neither reaches the tower).

**New discipline (R531): when two counters disagree, count the WHOLE FAMILY on
both sides before theorising.** "Our tool_change runs 2721 but we emit 2723 T"
looked like a missing call site; counting T / M620 / CP together showed a
constant −2 across all three, i.e. an upstream count difference and no missing
site at all.

## R532 — reserved tags part 3: `CP_TOOLCHANGE_WIPE`

**Reading the emission site paid off — the tag carries TWO suffixes, not one.**
WipeTower.cpp:3960-3961 (`toolchange_wipe_new`):

```cpp
";" + reserved_tag(ETags::CP_TOOLCHANGE_WIPE)
    + " CT" + std::to_string(solid_tool_toolchange)
    + " FL" + std::to_string(is_first_layer()) + "\n"
```

`std::to_string(bool)` renders "0"/"1", so the real lines are
`; CP_TOOLCHANGE_WIPE CT0 FL0` and `... CT0 FL1`. Confirmed against the actual
C++ output before writing any code (R504/R528): **2720 CT0 FL0 + 3 CT0 FL1**.
A bare `; CP_TOOLCHANGE_WIPE` would have been wrong on every line.

`CT` is hard-zero in our port because there is no solid-toolchange path at all —
R506 measured `solid_tool_toolchange` as ZERO for Majora and our `tool_change`
takes no such parameter. The code says so, so it starts reporting correctly if
that branch is ever ported.

**Prediction (R519), stated before running: 2721.** Measured **2721**
(2718 FL0 + 3 FL1), line delta **2721** exactly.

| | ours | C++ |
|---|---|---|
| `; CP_TOOLCHANGE_WIPE CT0 FL0` | 2718 | 2720 |
| `; CP_TOOLCHANGE_WIPE CT0 FL1` | **3** | **3** |

**The FL1 split independently confirms R531.** First-layer tool changes match
C++ *exactly* (3 = 3); the entire −2 lives in FL0, i.e. somewhere later in the
print. That is a second, independent line of evidence that the −2 is an upstream
tool-change count difference and not a missing emitter.

**Verified additive** (`; Tool change from` 2721/2721, `; WIPE_TOWER_START`
3377/3377, `G1  X219.729   E1.8473` 13812/13812). Semantic verdicts unchanged,
still SEMANTICALLY EQUIVALENT. Eight guard tests green.

**DELIBERATE RE-BASELINE — majora `89377938` -> `0538403b`.**
**benchy `5a34af50` and cube `ab415621` UNCHANGED.**

**Tag state now:**

| tag | ours | C++ |
|---|---|---|
| `; WIPE_TOWER_START` / `_END` | 3377 | 3655 |
| `; CP TOOLCHANGE START` / `END` | 2721 | 2723 |
| `; CP_TOOLCHANGE_WIPE` | **2721** | 2723 |
| `; CP EMPTY GRID START` / `END` | **0** | 209 |

**Remaining: `CP EMPTY GRID` only.** Three C++ sites inside `finish_layer_new`
(:3606/:3643, :3770/:3814, :3910), emitting just 209 against 656 layers — so
they sit on a conditional sparse/empty-grid branch. Left unported again this
round rather than blind-inserted: predicting the count needs the branch
condition mapped to our `finish_layer` first (R519).

**New discipline (R532): read the emission EXPRESSION, not just the tag name —
`reserved_tag(X)` was concatenated with two runtime-valued suffixes, and the
tag name alone would have produced a wrong line 2,721 times.**

## R533 — reserved tags COMPLETE: `CP EMPTY GRID`

The last of the four tag kinds. **The branch condition was already documented in
our own source** — R503's note in `finish_layer` records that C++'s
`finish_layer_new` receives `extrude_fill = false` on **653 of 656** calls, and
that the fill comes from `finish_block` on ~206 tool-change layers. 206 + 3 =
**209**, exactly C++'s count. No new analysis was needed; the answer was in the
comment left three rounds earlier.

**Predicted before running with the existing R505 `WTFILLCNT` probe** (which
already counted this exact guard): `passed = 207`. Measured **207 START + 207
END**, line delta **2277 = 207 x 11** exactly (11 lines per emission: separator
+ START, then END + separator + C++'s seven trailing blanks).

| | ours | C++ |
|---|---|---|
| `; CP EMPTY GRID START` / `END` | **207** | 209 |

**−2 again**, matching every other tool-change-derived counter (R531, R532).
Three independent counters now show the same constant offset, which is the
cleanest possible evidence that it is one upstream planning difference and not
three separate emitter bugs.

**Deliberately NOT emitted:** C++'s
`.comment_with_value(" layer #", m_num_layer_changes + 1)` that accompanies the
START tag. That counter has no exact counterpart on our side and guessing it
would put a wrong value on 207 lines (R528). The reserved tag itself — what
`GCodeProcessor` actually consumes — is emitted correctly.

**RESERVED TAGS ARE NOW COMPLETE:**

| tag | ours | C++ | note |
|---|---|---|---|
| `; WIPE_TOWER_START` / `_END` | 3377 | 3655 | residual = our tower block-count difference (R531) |
| `; CP TOOLCHANGE START` / `END` | 2721 | 2723 | −2 upstream |
| `; CP_TOOLCHANGE_WIPE CT0 FL*` | 2721 | 2723 | −2 upstream; FL1 split matches exactly 3=3 |
| `; CP EMPTY GRID START` / `END` | **207** | 209 | −2 upstream |
| `CP TOOLCHANGE UNLOAD` / `LOAD` / `WIPE` | 0 | **0** | C++ emits none for this config — correctly not ported |

We went from emitting **zero** of ~9,300 reserved-tag lines to emitting all four
live kinds, every count explained.

**Verified additive** (`; Tool change from` 2721/2721, `; WIPE_TOWER_START`
3377/3377, `; CP_TOOLCHANGE_WIPE CT0 FL0` 2718/2718, `G1  X219.729   E1.8473`
13812/13812). Semantic verdicts unchanged, still SEMANTICALLY EQUIVALENT. Eight
guard tests green.

**DELIBERATE RE-BASELINE — majora `0538403b` -> `0fa9f9ff`.**
**benchy `5a34af50` and cube `ab415621` UNCHANGED.**

**New discipline (R533): before analysing a branch, grep your own source for a
prior round's note on it.** The `extrude_fill = false on 653/656` fact and a
purpose-built counter (`WTFILLCNT`) were both already there from R503/R505; the
round reduced to running an existing probe.

---

## R534 — the tower's analyzer trio: `; LAYER_HEIGHT:` + `; LINE_WIDTH:`

The queued follow-up from R532: *is our `; LINE_WIDTH:` the same reserved tag as
`ETags::Width` at WipeTower.cpp:3966, and do we emit the first-layer variant?*

**Yes, it is the same tag, and we emitted none of them.**
`reserved_tag(ETags::Width)` is the literal `" LINE_WIDTH: "`
(GCodeProcessor.cpp:54), the tag the object paths already use; `ETags::Height`
is `" LAYER_HEIGHT: "` (:53).

### Where C++ emits it

Seven textual sites in `WipeTower.cpp`; only **three** are live:

| site | guard | Majora count |
|---|---|---|
| `WipeTowerWriter` ctor :637-642 | none — once per writer | **3655** |
| `toolchange_wipe_new` :3967 | `is_first_layer()` | **3** |
| `toolchange_wipe_new` :4158 | `is_first_layer()` | **3** |
| `toolchange_wipe_new` :3963 | `!m_nozzle_change_result.gcode.empty()` | 0 |
| `toolchange_Wipe` :2532, :2633 | legacy non-`_new` function | 0 |
| `suppress_preview` / `resume_preview` :703-704 | `ENABLE_GCODE_VIEWER_DATA_CHECKING` | 0 |
| ramming :2330, :2393 | **inside `#if 0` (2322-2450)** | 0 |

The two ramming sites were the trap: they look live and they are the ones whose
comments explain the intent ("so the next lines are not affected by
ramming_line_width_multiplier"). Walking the preprocessor nesting from line 2322
showed the block closes at 2450, so both are dead.

The ctor site emits a **contiguous trio** — `; LAYER_HEIGHT:` / `; FEATURE:
Prime tower` / `; LINE_WIDTH:` — at the head of every tower block. We were
emitting only the middle line, so the tower carried no height and inherited
whatever width the preceding object feature had set.

### Classifying by predecessor, before writing code

Counting `; LINE_WIDTH: 0.500000` in the reference and bucketing each by the
line above it gave the whole structure in one command:

```
3655  <FEATURE tag>
   3  ; CP_TOOLCHANGE_WIPE CT0 FL1
   3  OTHER: G1  X219.729  E1.8473
```

3655 = the block count, and the two 3s are the first-layer tool changes — the
same 3 that R532 matched exactly. So the prediction was **+3377 LAYER_HEIGHT**
(our block count), **+3377 + 3 + 3 = +3383 LINE_WIDTH**, **6760 lines**.

### Measured

Line delta **6760**, exactly. Counts:

| | ours | C++ | note |
|---|---|---|---|
| `; LAYER_HEIGHT:` (tower) | 3377 | 3655 | value `0.300000` on every block in both |
| `; LINE_WIDTH: 0.500000` | **3383** | 3661 | residual 278 |
| — predecessor `<FEATURE tag>` | 3377 | 3655 | |
| — predecessor `CP_TOOLCHANGE_WIPE CT0 FL1` | 3 | 3 | exact |
| — predecessor a wipe stroke (:4158) | 3 | 3 | exact |
| `; LINE_WIDTH:` total | 119625 | 215199 | see below |

**3661 − 3383 = 278 = 3655 − 3377**, the same tower block-count difference the
`WIPE_TOWER_START` tag has tracked since R531. The two first-layer sites match
exactly, as they must — they are gated on a population we already match.

The :3963 nozzle-change site was ported anyway (behind its guard) and fires
**zero** times, confirming R502's finding that our nozzle-change tower gcode is
always empty.

**Placement divergence, kept deliberately.** C++ emits the trio at the *head* of
`tcr.gcode`, i.e. **before** the change-filament block, so the flush moves fall
inside the Prime tower feature. We emit it after the block (R464). Keeping the
three lines contiguous reproduces C++'s output shape; moving the whole trio
upstream changes per-feature attribution and is a separate change.

**Verified additive** (`; Tool change from` 2721, `; WIPE_TOWER_START` 3377,
`; CP_TOOLCHANGE_WIPE CT0 FL0` 2718, `; CP EMPTY GRID START` 207,
`G1  X219.729   E1.8473` 13812 — all unchanged). Semantic verdicts identical to
baseline: object material 0.9959, layers 657=657, per-layer 4.47%, Top 1.173,
silhouette 99.37% — **SEMANTICALLY EQUIVALENT**. Eight guard tests green.

**DELIBERATE RE-BASELINE — majora `0fa9f9ff` -> `7a3d41af`.**
**benchy `5a34af50` and cube `ab415621` UNCHANGED.**

### New finding, queued

Total `; LINE_WIDTH:` is **119,625 ours vs 215,199 C++**, and `G1` moves with an
`E` are **807,828 vs 1,193,658**. Material matches to 0.9959 and the silhouette
to 99.37%, so this is not a geometry gap — it is *segmentation*: C++ splits a
variable-width Arachne perimeter into more, shorter moves and re-tags the width
at each. Worth sizing on its own; it is the largest remaining structural
difference in the object paths.

**New discipline (R534): a `#if 0` can span hundreds of lines — walk the
preprocessor nesting before believing a call site is live.** Two of the seven
`ETags::Width` sites sat 128 lines inside one, and they were the two whose
comments best explained the tag's purpose.

**Also (R534): `scripts/semantic_compare.py` takes `(rust, bambu)` and its
metrics are NOT symmetric** — a swapped invocation reported "per-layer mean dev
7.62%, FAIL" for an output that scores 4.47% PASS in the correct order.

---

## R535 — sizing the "segmentation gap": it is a SPEED gap, and only on Majora

R534 closed noting that we emit far fewer extrusion moves than C++. This round
sized it. **No code changed** — the round's product is the diagnosis and the
control that localises it.

### It is not geometry

Bucketing every extruding move by the active `; FEATURE:` tag and summing XY
path length:

| | ours | C++ | ratio |
|---|---|---|---|
| total extruding moves | 1,106,344 | 1,445,296 | 0.766 |
| total XY path length | 2,847,296 mm | 2,855,275 mm | **0.997** |

Same path, fewer moves. (R534's "-32%" was measured with a crude `G1 ` prefix
filter that missed our double-space tower moves; the correct figure is -23%.)

### Where the moves are

Splitting straight (`G1`) from arc (`G2`/`G3`):

| feature | kind | ours | C++ | n-rat | mm-rat |
|---|---|---|---|---|---|
| Outer wall | G1 | 177,228 | 415,330 | **0.43** | 0.872 |
| Outer wall | ARC | 117,357 | 141,482 | 0.83 | 1.074 |
| Inner wall | G1 | 156,800 | 220,766 | 0.71 | 0.924 |
| Sparse infill | ARC | 111,861 | 111,506 | **1.00** | 1.004 |

Two opposite biases, so it is not one tolerance being wrong: on walls we emit
fewer, longer arcs; on infill we emit slightly more moves than C++.

### The arc fitter is exonerated — twice

**Control 1 — a feature whose tolerance is a fixed constant.** Walls use
`scaled_resolution` from config; `erInternalInfill` uses the hard-coded
`SCALED_SPARSE_INFILL_RESOLUTION`. Comparing arc RADIUS distributions (from the
`I`/`J` offsets, no reconstruction needed):

| radius | Sparse infill ours/C++ | Outer wall ours/C++ |
|---|---|---|
| <1 mm | 0.98 | **0.17** |
| 1-2 mm | 1.00 | **0.16** |
| 2-5 mm | 1.00 | 0.51 |
| 10-25 mm | 1.01 | 0.90 |
| 100-500 mm | 1.00 | 1.11 |
| >=500 mm | 1.10 | 1.16 |

Sparse infill matches bucket-for-bucket across three orders of magnitude —
with a **looser** tolerance (0.04 mm vs 0.012 mm). A broken fitter cannot be
right at 0.04 and wrong at 0.012.

**Control 2 — the other fixture.** On Benchy every feature matches:
Outer wall G1 27,066/27,043 (1.00), ARC 6,424/6,455 (1.00), lengths 1.000/1.002.

`do_arc_fitting` is a line-by-line port, `DEFAULT_SCALED_MAX_RADIUS` is correctly
re-scaled for this crate's 1e5 factor, and the slice-stage simplification uses
C++'s fixed `0.0025` (PrintObjectSlice.cpp:144), not the config resolution. All
three were checked and all three are right.

### The actual mechanism: feedrate modulation

Counting maximal runs of consecutive extruding moves, then classifying what
breaks each run:

```
RUST outer-wall run breaks (32,177)     CPP outer-wall run breaks (165,849)
  14,299  G1 E<0 (retract)               133,997  G1 F-only
  13,842  <FEATURE change>                16,601  G1 E<0 (retract)
   3,136  G1 F-only                       14,146  <FEATURE change>
```

`G1 F`-only lines, per feature:

| feature | ours | C++ | ratio |
|---|---|---|---|
| Outer wall (Majora) | 17,525 | 148,936 | **0.118** |
| Inner wall (Majora) | 14,768 | 53,474 | 0.276 |
| Sparse infill (Majora) | 10,487 | 10,623 | 0.987 |
| Outer wall (**Benchy**) | 15,764 | 16,293 | **0.968** |

C++ changes the outer-wall feedrate 8.5x more often than we do **on Majora
only**. Every extra speed change forces a path split, which is where the extra
moves come from and why our arc fitter sees longer smooth runs to swallow.

### What the speeds are

C++'s Majora outer wall carries **49,568 distinct** feedrates (ours 2,436, and
80% of ours are the single value `F7150.945`). Two populations:

- a continuum — the `smooth_speed_discontinuity_area` ramp;
- a discrete ladder `F6000 / F4500 / F3000 / F2700 / F2400 / F2100`
  (= 100 / 75 / 50 / 45 / 40 / 35 mm/s).

The ladder appears **inside** layers — 496 of 656 layers contain both ladder and
continuum values — so it is per-overhang-degree wall speed, **not** per-layer
cooling scaling.

And the ordering matters: with no speed discontinuities there is nothing for the
smoothing pass to ramp, so the missing continuum is *downstream* of the missing
ladder. One root cause, not two.

### Not a broken port — a Majora-only wiring gap

`smooth_speed_discontinuity_area` IS implemented (`gcode/smooth_speed.rs`, gated
at `exporter.rs:414` on `detect_overhang_wall && smooth_speed_discontinuity_area
&& role in {ExternalPerimeter, Perimeter, OverhangPerimeter} && coeff != 0 &&
!first_layer && paths.len() > 1`), and on Benchy our outer-wall feedrates match
C++ **value for value**:

```
        ours        C++
F12000   615        582
F3420    449        451
F3480    412        413
F3540    390        391
F3300    380        381
```

So the speed machinery works. What is specific to Majora is that it is the
**8-filament 3MF**: `filament_overhang_1_4_speed` .. `4_4_speed`,
`filament_enable_overhang_speed` and friends are all 8-element arrays. The next
round should check whether the per-filament overhang-speed arrays are resolved
for the active filament on the 3MF/MMU path (and whether the overhang-degree
classification runs there at all) — Majora's overhang DETECTION is fine
(Overhang wall 4,450 vs 5,180 moves), it is the 1/4..4/4 speed CLASSES that are
absent.

### Prize

Honest sizing, because it decides priority:

- **Semantic verdicts: no change.** Speed is not material or geometry; all five
  checks pass now and would still pass.
- **Print behaviour: real.** Our Majora outer walls run at a near-constant
  ~119 mm/s where C++ drops to 35-100 mm/s over overhangs. On an organic model
  that is a genuine quality difference, and it is squarely inside ask #2's "the
  G-code should be essentially the same".
- **Slicing time: negative.** Emitting ~130k more lines makes export slower.

So: worth fixing for correctness, not for any metric currently being tracked.

**No re-baseline — nothing was changed.** majora `7a3d41af`, benchy `5a34af50`,
cube `ab415621`.

**New discipline (R535): when one fixture diverges and another does not, the
second fixture IS the control — run it before theorising about the code.** Two
rounds of arc-fitter archaeology were made redundant by one Benchy comparison
that returned 1.00 on every feature.

---

## R536 — root cause: the ARACHNE wall path never grades overhang degree

R535 localised the Majora outer-wall speed gap. This round found the cause and
sized the fix. **Two env-gated probes added; no behavioural change** — majora
`7a3d41af`, benchy `5a34af50`, cube `ab415621` all reproduce byte-identically.

### First, a correction to R535

R535 read the discrete feedrate ladder `F6000/F4500/F3000/F2700/F2400/F2100` as
per-overhang-degree speed classes. **That was wrong.** Reading real context
(R490) shows one continuous ramp through a slow floor:

```
16463 G1 F2794.457      <- decel
16465 G1 F2503.634
16467 G1 F2228.665
16469 G1 F1969.801
16471 G1 F1726.912
16473 G1 F1500
16475 G1 F1200          <- floor
...
16478 G1 F1500          <- accel
16480 G1 F1800
16482 G1 F2100
16484 G1 F2400
16491 G1 F2700
16493 G1 F3000
16495 G1 F4500
16497 G1 F6000
16499 G1 F7151.157      <- nominal
```

The "ladder" is just the accel half landing on round values. One mechanism, not
two — which R535 already suspected for the right reason (no discontinuity means
nothing to smooth) but attributed to the wrong upstream.

### The probes

`SMOOTHPROBE=1` counts each sub-condition of the smoothing gate at
`gcode/exporter.rs`; `OHSPLITPROBE=1` does the same for the overhang-split gate
in `perimeter_generator.rs::traverse_loops`.

| | Majora | Benchy |
|---|---|---|
| loops reaching the smoothing gate | 25,000 | 1,000 |
| `detect` / `flag` / `role` / `coeff` | all pass | all pass |
| **`paths.len() > 1`** | **632 (2.5%)** | 324 (32%) |
| **paths per loop** | **1.12** | **10.56** |
| **paths with `overhang_degree != 0`** | **1,260 / 27,959 (4.5%)** | **8,164 / 10,561 (77%)** |

Every config input to the gate is correct on both fixtures (`detect_overhang_wall`,
`smooth_speed_discontinuity_area`, `smooth_coefficient` = 150 on Majora, 4 on
Benchy — all parsed from the 3MF). The gate fails on `paths.len() > 1`: our
Majora wall loops are **single-path**.

And `OHSPLITPROBE` printed **nothing at all** on Majora — the classic
`traverse_loops` never runs there.

### Why

Majora is `wall_generator = 'arachne'`; Benchy is classic. C++ has **two**
overhang-grading sites:

| C++ | path | our port |
|---|---|---|
| `traverse_loops` -> `detect_overhang_degree` (PerimeterGenerator.cpp:395) | classic | **ported** |
| `traverse_extrusions` -> `detect_overhang_degree` (PerimeterGenerator.cpp:707) | **arachne** | **NOT ported** |

At PerimeterGenerator.cpp:703 the Arachne path branches:

```cpp
if (is_enable_overhang_speed(pg) && fuzzy_skin_allows_overhang_slowdown(pg))
    paths = detect_overhang_degree(flow, role, lower_layer_polys, clip_paths, subject_path, nozzle_diameter);
else { /* plain ctIntersection / ctDifference split */ }
```

Our `arachne_line_to_extrusion_paths` implements **only the `else` branch** — a
binary supported/unsupported split producing `erOverhangPerimeter` for the
unsupported part and leaving `overhang_degree = 0` everywhere else. That is
exactly the measured 4.5%.

So: no graded degrees -> no per-segment speed differences -> no discontinuities
-> the (correctly implemented) smoothing pass has nothing to ramp -> 80% of our
Majora outer wall carries one feedrate.

### Size of the fix

The missing piece is C++'s **Arachne overload** of `detect_overhang_degree`,
`OverhangDetector.cpp:168-465` — **~298 lines**. Not from scratch: our
`overhang_detector.rs` is already 919 lines and carries the shared helpers the
overload needs — `z_path_to_polylines`, `add_sampling_points`,
`add_sampling_points_paths`, `get_base_degree`, `get_mapped_degree`,
`merged_with_degree`, `smoothing_degrees`, `check_degree`,
`prepare_split_polylines`, `extrusion_paths_append` — plus
`MIN_DEGREE_GAP_ARACHNE = 0.25`, a constant that exists *only* for this overload.
The `ClipperLib_Z` open-path dependency is also already satisfied: our Arachne
path calls `clipper_z::clip_extrusion` today.

So it is a bounded ~300-line faithful port onto machinery that is already
present, not a new subsystem.

**Deferred to R537 rather than rushed at the end of a long diagnostic round** —
it is geometry-sensitive code on the primary fixture and deserves its own
verified increment.

**No re-baseline.** Both probes are `std::env::var_os(...).is_some()` (default
OFF); all three fixtures reproduce byte-identically and the eight guard tests
are green.

**New discipline (R536): when a gate looks open, instrument every sub-condition
separately — a gate that fails names its own cause.** And the strongest signal
here was a probe that printed *nothing*: `OHSPLITPROBE` staying silent on Majora
proved the whole classic path was unreachable, which no amount of reading the
gate's condition would have shown.

---

## R537 — port the ARACHNE overload of `detect_overhang_degree`

The fix for R536's root cause. Majora's outer wall now modulates speed; **object
material is bit-for-bit unchanged and two geometric metrics improved.**

### What was ported

`OverhangDetector.cpp:168-317` — ~150 lines, not the 298 estimated in R536
(:319-465 are the shared helpers, already ported). It lands in
`overhang_detector.rs` exactly where an R411 note had reserved the space.

Every helper it needs was already present and had been written for it —
`z_path_to_polylines`, `add_sampling_points(_paths)`, `upper_bound_index`,
`SignedOverhangDistancer` (ported but until now unreachable), and
`MIN_DEGREE_GAP_ARACHNE`, a constant that exists for no other caller.

**One deliberate divergence.** C++ finishes with
`extrusion_paths_append(list, ZPaths, role, flow, degree)`, which rebuilds each
segment as a VARIABLE-WIDTH path from the `z` (junction width) coordinate. R414
established that this crate's variable-width builder double-applies the Arachne
spacing->width conversion, so doing that would change deposited material. Our
port therefore returns `(ZPath, degree)` pairs and the caller keeps the loop's
avg-width flow — exactly as the pre-existing binary split did. **This port
changes overhang DEGREE and path SPLITS only, never E.** The measurement below
confirms it.

### The bug found while porting

First run over-split badly: outer-wall `G1 F` went to 347,977 against C++'s
148,936 (2.34x), and the file grew past C++'s line count.

Cause: I passed the **ungrown** lower slices as the distancer reference. C++
passes `lower_slices_polygons()`, which is
`offset(*lower_slices, scale_(+nozzle_diameter/2))` (PerimeterGenerator.cpp:1495)
— the **grown** polygons — as BOTH the clip paths and the distancer reference.
The `offset_width = scale_(nozzle)/2` term inside `detect_overhang_degree` then
converts that measurement back to a distance from the raw slice. Handing it the
ungrown polys double-counts the nozzle offset, inflating every degree.

### Measured

`SMOOTHPROBE=1`, Majora, 27,000 loops:

| | before | after | Benchy (working control) |
|---|---|---|---|
| paths per loop | 1.12 | **9.38** | 10.56 |
| `overhang_degree != 0` | 4.5% | **84%** | 77% |
| `paths.len() > 1` | 2.5% | **21.4%** | 32% |

G-code, ours vs C++:

| | before | after | C++ |
|---|---|---|---|
| outer-wall `G1 F`-only | 17,525 (0.118) | **118,766 (0.797)** | 148,936 |
| inner-wall `G1 F`-only | 14,768 (0.276) | **25,209 (0.471)** | 53,474 |
| total `G1 F`-only | 155,859 (0.469) | **266,598 (0.802)** | 332,222 |
| distinct outer-wall feedrates | 2,436 | **18,565** | 49,568 |
| total lines | 2,107,641 | 2,495,728 | 2,939,713 |

The C++ ladder (`F3000 / F6000 / F2700 / F4500`) now appears in our top values,
where before 80% of the outer wall carried the single value `F7150.945`.

### Parity effect — better than predicted

R535 predicted the verdicts would not move. Material indeed did not, but two
geometric metrics did, in the right direction:

| | before | after |
|---|---|---|
| object material | 0.9959 | **0.9959** (identical) |
| per-layer material | 4.47% | **4.47%** (identical) |
| SILHOUETTE (object), area-wtd | 99.37% | **99.53%** |
| SILHOUETTE mean / min | 99.46% / 98.0% | **99.59% / 98.3%** |
| WALL LINES IoU, area-wtd | 94.64% | **95.20%** |
| WALL LINES layers < 95% | 288 | **245** |

Still **SEMANTICALLY EQUIVALENT**, all five checks pass. The extra split
vertices at degree boundaries make our wall polylines as dense as C++'s, which
is what the wall-line raster was measuring.

Slicing time: `export_gcode` 4.34s -> **4.70s** (+0.36s) for ~388k added lines —
the expected, and acceptable, cost.

### Gate and residuals

Behind a new default-ON `ARACHNE_OVERHANG_DEGREE`. **A/B verified**: with the
gate off, Majora reproduces `7a3d41af` byte-for-byte; on, `e871ade4`. Benchy
`5a34af50` and cube `ab415621` are untouched — they are `wall_generator=classic`
and never reach this code.

Honest residuals, queued rather than hidden:

- **inner wall 0.471** — improved from 0.276 but still half of C++'s count.
- **overhang wall `G1 F` 305 vs 1,136 (0.268)** — the *unsupported* part still
  goes through our binary classifier; C++'s `detect_brigde_wall_arachne`
  (PerimeterGenerator.cpp:604-626) is not ported.
- **distinct feedrates 18,565 vs 49,568** — the smoothing ramp fires now but
  produces fewer distinct steps than C++.

**RE-BASELINE — majora `7a3d41af` -> `e871ade4`. benchy `5a34af50` and cube
`ab415621` UNCHANGED.** Eight guard tests green.

**New discipline (R537): when a ported function takes a geometry argument that
the caller also derives, check whether C++ passes the RAW or the DERIVED form —
and whether the function internally compensates for the derivation.** Here the
same grown polygons served as clip and as distance reference, with a
`+nozzle/2` term inside the callee undoing the growth; passing the raw slices
looked more "correct" and silently doubled every overhang degree.

---

## R538 — quantifying R537 per role, and eliminating four causes of the inner-wall residual

**No behavioural change.** Two env-gated probes added (`SMOOTHROLE` inside
`SMOOTHPROBE`, and `ARACHPROBE`); majora `e871ade4`, benchy `5a34af50`, cube
`ab415621` all reproduce byte-identically, eight guard tests green.

### R537's effect, now split by wall role

R537 was measured in aggregate. Bucketing the same probe by `loop_role`:

| | Majora external | Majora internal | Benchy external | Benchy internal |
|---|---|---|---|---|
| loops | 13,948 | 13,051 | 516 | 484 |
| paths / loop | **15.72** | **2.60** | 19.36 | **1.18** |
| `overhang_degree != 0` | 88.4% | 57.6% | 81.2% | **8.9%** |

And in the G-code, ours vs C++ (`$D/runs.py`, `$D/segsplit.py`):

| | before R537 | after R537 | C++ |
|---|---|---|---|
| Outer wall runs | 32,177 (0.19) | **133,825 (0.81)** | 165,849 |
| Outer wall moves/run | 9.16 | **3.54** | 3.36 |
| Outer wall G1 | 177,228 (0.43) | **334,845 (0.81)** | 415,330 |
| Outer wall ARC | 117,357 (0.83) | **139,208 (0.98)** | 141,482 |
| Inner wall ARC | 114,647 (0.98) | 116,703 (**0.99**) | 117,546 |
| Inner wall runs | 19,252 (0.33) | 29,748 (0.51) | 58,633 |
| Inner wall moves/run | 14.10 | 9.71 | 5.77 |

**The outer wall is now structurally matched** — moves per run 3.54 against 3.36,
arcs at 0.98. Both engines' arc counts agree to 1-2% on both walls, confirming
again that the underlying polylines have the same point density.

### The inner-wall residual: four suspects eliminated, none confirmed

Inner-wall `G1 F`-only is 25,209 vs 53,474 (0.471). Splitting the feedrate
population shows where the gap is NOT:

```
RUST Inner wall: total 25,209, DISTINCT 4,820     CPP: total 53,474, DISTINCT 21,285
  12,581  F7150.945   <- dominant                   14,522  F7151.157   <- dominant
   1,126  F3000                                        511  F3000
   1,036  F2700                                        435  F2700
```

The **dominant-value counts nearly match** (12,581 vs 14,522). The entire deficit
is in the ramp tail: 12,628 non-dominant against C++'s 38,952.

Eliminated this round:

1. **The smoothing implementation.** All six C++ functions are ported —
   `mapping_speed` (GCode.cpp:5918), `get_speed_coor_x` (:5925),
   `need_smooth_speed` (:5965), `split_and_mapping_speed` (:5973-6121),
   `merge_same_speed_paths` (:6123-6161), `set_speed_transition` (:6163-6257),
   `smooth_speed_discontinuity_area` (:6259-6272).
2. **Its constants.** `smooth_speed_step = 10` and
   `min_step_length = scale_(0.4)` match GCode.cpp:88,91 exactly, and
   `f(x) = coeff * x^2` matches :5918-5923.
3. **`overhang_degree_corr_speed`.** A faithful port of GCode.cpp:5931-5962,
   including the `degree >= 4 || degree == int(degree)` short-circuit and the
   two `0 -> normal_speed` fallbacks.
4. **"Low inner-wall splitting is itself the bug."** Benchy's internal walls sit
   at **1.18 paths/loop and 8.9% graded** — far *less* split than Majora's 2.60
   / 57.6% — yet Benchy's inner-wall `G1 F` matches C++ at **0.943**. Sparse
   inner-wall grading is normal; something else drives C++'s inner-wall ramps.

Also tested and **disproved**: our arachne overhang block carries an extra
`&& line.is_closed` condition that C++'s guard (PerimeterGenerator.cpp:667,
`detect_overhang_wall && layer_id > raft_layers`) does not have. `ARACHPROBE=1`
measured the excluded population — **external 519 / 12,747 open (4.1%), internal
748 / 12,253 open (6.1%)**. Six percent cannot produce a 2x deficit.

**The inner-wall residual is therefore narrowed but NOT closed.** Reporting it
open rather than guessing.

### Still queued

- **inner wall 0.471** — the four causes above are ruled out.
- **`detect_brigde_wall_arachne` (PerimeterGenerator.cpp:604-626) unported** —
  the unsupported part still uses our hand-rolled binary classifier; overhang-wall
  `G1 F` is 305 vs 1,136.
- **distinct outer-wall feedrates 18,565 vs 49,568.**

**New discipline (R538): when a fix lands, re-measure it split by the
sub-populations it was supposed to affect.** R537's aggregate "paths/loop 9.38 vs
Benchy 10.56" hid two very different stories — external 15.72 (matched) and
internal 2.60 (not) — and the aggregate would have made the next round chase the
wrong wall.

---

## R539 — `detect_brigde_wall_arachne` is ALREADY PORTED; the real gap is the overhang FLOW

**No behavioural change.** One probe added (`ARACHBRIDGE`, inside `ARACHPROBE`);
majora `e871ade4`, benchy `5a34af50`, cube `ab415621` reproduce byte-identically,
eight guard tests green.

### Retraction

R538's handoff listed `detect_brigde_wall_arachne` (PerimeterGenerator.cpp:604-626)
as "UNPORTED, and now the largest concrete gap". **That is wrong** — my own error,
carried forward across two handoffs. The C++ function is 23 lines:

```cpp
Line line(thick_polyline.front(), thick_polyline.back());
if (line.length() < thick_polyline.length()) {
    extrusion_path_append(paths, ..., overhang_sampling_number - 1);  // curved
    continue;
}
extrusion_path_append(paths, ..., overhang_sampling_number);          // straight
```

and `arachne_line_to_extrusion_paths` already does exactly that —
`let degree = if line_len < poly_len { n - 1.0 } else { n };` with the same
`OVERHANG_SAMPLING_NUMBER`, the same chord-vs-polyline test and the same
`OverhangPerimeter` role. Same class of error as R523's "port SeamPlacer.cpp:157"
(already ported). **PROVE A FUNCTION IS UNPORTED BEFORE PLANNING TO PORT IT** —
the corollary to R501.

The `zero_z_support` branch (:722-727) is also settled: Majora has
`enable_support = '0'` and `enforce_support_layers = '0'`, so C++ takes the
`erOverhangPerimeter` / `overhang_flow` arm — the same role we already assign.

### Also confirmed correct

`set_speed_transition` skips overhang paths (GCode.cpp:6171-6174) and our port
does too (`smooth_speed.rs:403`). So the smoothing pass is NOT leaking ramp
speeds into overhang walls.

### Measured

`ARACHBRIDGE=1` over Majora's overhang population:

```
overhang zpaths=1200  pts=2:231  pts=3-5:501  pts>=6:468
                      curved(deg5)=969  straight(deg6)=231
```

**`straight` ≡ `pts=2` exactly (231 = 231)** — a two-point ZPath has chord ==
polyline length by construction, so it can never test as curved. That is true of
C++ too; it is a property of the test, not a bug in either engine.

**Retracted mid-round:** I first read our 1,200 overhang paths against C++'s
1,136 overhang-wall `G1 F` lines and called the ratio "inverted". Those are
different quantities — an F line is emitted only when the speed *changes*, so it
is not a path count (R491). The comparison is void; the classifier ratio is not
known to be wrong.

Likewise the "19 distinct overhang feedrates vs C++'s 2" is mostly an artefact of
`fvals2.py` attributing each `G1 F` to the preceding `; FEATURE:` tag: the tail
values appear 2 lines each, ~17 lines out of 305, at feature boundaries.

### The real divergence, now located and sized

C++ passes `perimeter_generator.overhang_flow` to the unsupported segments
(PerimeterGenerator.cpp:727), and that flow is

```cpp
// LayerRegion.cpp:172
g.overhang_flow = this->bridging_flow(frPerimeter, object_config.thick_bridges);
```

— **bridge flow**, not the wall's own flow. We build those segments with the
loop's avg-width flow (`mk()`), which is why we over-extrude them:

| | ours | C++ | ratio |
|---|---|---|---|
| `Overhang wall` E | 266.0 | 254.4 | **1.045** |
| `Overhang wall` length | — | — | 1.050 |
| `Overhang wall` G1 | 2,437 | 3,252 | 0.75 |
| `Overhang wall` ARC | 2,033 | 1,928 | 1.05 |

**Size: 266 of 135,746 mm object material = 0.20%.** Small, but it moves the one
per-feature ratio that is visibly off in the right direction, and unlike the
R414 variable-width divergence this one is a straightforward flow substitution
that C++ makes explicitly. Queued for R540 — it changes E deliberately, so it
needs its own gated, verified increment.

**New discipline (R539): before planning a port, read the C++ function and diff
it against what you already have — "unported" claims decay across handoffs.**
Two rounds' worth of handoff text called this the largest concrete gap; it was
23 lines that had been faithfully ported all along.

---

## R540 — the overhang "flow substitution" is INERT here; the real gap is variable width

**No code change.** R539 queued this round as "port the overhang FLOW". Reading
the fixture and the C++ callee shows that target does not hold up. Majora
`e871ade4`, benchy `5a34af50`, cube `ab415621` unchanged.

### Why the stated target is inert

C++ passes `overhang_flow` to the unsupported segments, and

```cpp
// LayerRegion.cpp:172
g.overhang_flow = this->bridging_flow(frPerimeter, object_config.thick_bridges);
// LayerRegion.cpp:45 (the !thick_bridge branch)
return this->flow(role).with_flow_ratio(region_config.bridge_flow);
```

Majora's actual values (R525): **`thick_bridges = '0'`** and
**`bridge_flow = '1'`**. So `overhang_flow` reduces to
`flow(frPerimeter) x 1.0` — **the ordinary perimeter flow**. There is no bridge
flow to substitute on this fixture.

### And the overhang E gap was never a flow gap

From the R537 semantic compare, `Overhang wall`:

| | ours | C++ | ratio |
|---|---|---|---|
| E | 266.0 | 254.4 | 1.045 |
| length | — | — | **1.050** |
| **E per mm** | 0.04104 | 0.04121 | **0.996** |

Per-mm extrusion is already within 0.4%. A flow substitution can only move E/mm;
it cannot move a 5% **length** difference. The excess is that we classify ~5%
more length as overhang (`Overhang wall` ARC length 5,042 vs 4,724 = 1.067) —
a `ctDifference` classification difference, not a flow one.

**R525 again, and R519: read the fixture's value and size the prize before
planning the port.** Both checks were in the handoff; neither had been run.

### What C++ actually does differently

`extrusion_path_append` (Arachne/utils/ExtrusionLine.cpp:307) forwards to
`thick_polyline_to_multi_path(thick_polyline, role, flow, ...)` — a
**variable-width** builder. The widths come from the `ThickPolyline`'s
per-junction values; `flow` only supplies role/height. Measured inside
`; FEATURE: Overhang wall`:

| `; LINE_WIDTH:` changes | ours | C++ |
|---|---|---|
| Overhang wall | **2** | 879 (644 distinct) |

We stamp one avg width per loop; C++ varies it continuously.

### Sizing the real successor

That same builder governs every Arachne wall, and the gap is broad:

| `; LINE_WIDTH:` changes | ours | C++ |
|---|---|---|
| Outer wall | 2,873 (1,350 distinct) | **62,582** (21,181) |
| Inner wall | 6,941 (1,981) | **40,567** (17,731) |
| whole file | 119,622 | 215,199 |

This is exactly the divergence R414 deferred: our variable-width builder
double-applies the Arachne spacing->width conversion, so R537 deliberately kept
the avg-width flow to preserve E. Fixing it would touch **E on every wall in
every Arachne model** — a major gated undertaking, not a 0.2% item. It is now
the single largest known structural divergence in the wall path.

(Do not over-read the mean widths in that table: our sample is ~20x smaller and
the values are attributed by preceding `; FEATURE:` tag, so the means are not
comparable — R491. The counts are the sound part.)

### Overhang closed as a parity lever

Total overhang length is 6,477 mm ours vs 6,169 mm C++, against a ~2,850,000 mm
print — **0.2%**. With the flow substitution shown inert and the remainder being
either classification (5% of 0.2%) or the R414 variable-width deferral, this
sub-area is closed the same way R515 closed negative volumes: characterised,
sized, and not worth further grinding.

**New discipline (R540): a queued target inherits its premise from the round
that queued it — re-derive the premise before executing, not after.** R539
correctly identified that C++ passes a different flow; it did not check that on
this fixture the flow is numerically the same one, nor that the per-mm figure
already matched. Both took one command each.

---

## R541 — R414's blocker was stale; the variable-width builder was DEAD CODE

The handoff's instruction was to re-derive R414's blocker before touching
anything (R504/R540). Doing so changed the whole round.

### R414's blocker does not hold

R414 recorded: "the variable-width builder double-applies the Arachne
spacing->width conversion", and R537/R539 kept the hand-rolled avg-width `mk()`
because of it. On the real code:

- `variable_width.rs::thick_polyline_to_multi_path` applies `spacing_to_width`
  **exactly once** (:198, :311, :423), a faithful port of VariableWidth.cpp:66 /
  136 / 203 — `unscale(w) + height * (1 - 0.25*PI)` — and even carries an f32
  fidelity gate (`FLOW_F32`, R231) for the 6th-significant-digit drift.

The double-apply is a **caller** hazard: it happens only if the caller hands the
builder an already-converted width. Passing the raw ZPath `z` (still in Arachne's
spacing convention) makes it correct by construction.

### The machinery was already ported — and dead

`arachne/utils/extrusion_line.rs` already contained `to_thick_polyline_z`
(:551), `extrusion_paths_append_zpaths` (:643) and `detect_bridge_wall_arachne`
(:677) — the latter carrying the comment *"(Wired into
`arachne_line_to_extrusion_path` — R412 ports the fn; R413 wires it.)"*.

**R413 never wired it.** All three have been `#[allow(dead_code)]` ever since,
while `arachne_line_to_extrusion_paths` hand-rolled `mk()` alongside them. Third
time this pattern has appeared (R523, R539, now R541).

### Wired

All three call sites in the arachne overhang block now route through
`extrusion_paths_append_zpaths`, behind default-ON **`ARACHNE_VARIABLE_WIDTH`**;
`mk()` remains as the gate-off path. **A/B verified: gate off reproduces
`e871ade4` byte-for-byte.**

Verdicts are unchanged — object material **0.9959**, layers 657=657, per-layer
**4.47%**, silhouette **99.53%**, wall-lines IoU 95.20% -> 95.19%. The change is
E-neutral and metric-neutral; it is worth keeping because it replaces a
hand-rolled substitute with the ported C++ function, and it will start producing
correct variation for free once the input widths vary.

### But it barely moves — and the probe says why

| `; LINE_WIDTH:` changes | before | after | C++ |
|---|---|---|---|
| Outer wall | 2,873 | 3,524 | **62,582** |
| Inner wall | 6,941 | 7,646 | **40,567** |
| whole file | 119,622 | 120,983 | 215,199 |

A faithful builder fed near-constant widths emits near-constant widths. New probe
`ARACHWIDTH` (inside `ARACHPROBE=1`), over 25,000 Majora loops:

```
flat (min == max) = 24,503  (98.0%)
mean spread       = 1.5 um
junctions / loop  = 42.7
distinct w / loop = 1.03
```

**98% of our Arachne loops are exactly constant-width.** C++ produces 21,181
distinct outer-wall widths against our 1,715. So the divergence is **upstream in
Arachne's bead generation (`WallToolPaths`), not in the path builder** — the
builder had nothing to vary.

That relocates the target: Arachne is supposed to produce variable-width beads;
ours effectively produces one width per loop. R520 already established Arachne is
not a *timing* gap; this says it may be a *fidelity* one.

**RE-BASELINE — majora `e871ade4` -> `2838b07f`. benchy `5a34af50` and cube
`ab415621` UNCHANGED** (both are `wall_generator=classic`). Eight guard tests
green.

**New discipline (R541): when a faithful transform produces almost no variation,
measure the variation of its INPUT before suspecting the transform.** The builder
was correct all along; feeding it 98%-flat widths was the whole story, and the
probe that showed it took one build.

---

## R542 — Arachne is fully ported; the gap is WITHIN-loop bead variation

**No code change.** Majora `2838b07f`, benchy `5a34af50`, cube `ab415621`
unchanged. This round inventoried the subsystem and corrected R541's framing.

### Nothing is missing

Our `arachne/` tree mirrors C++'s file-for-file, including **all seven** beading
strategies:

| C++ | ours |
|---|---|
| `BeadingStrategy.cpp/.hpp` (79+119) | `beading_strategy.rs` (306) |
| `BeadingStrategyFactory` (62+35) | `beading_strategy_factory.rs` (253) |
| `DistributedBeadingStrategy` (95+40) | `distributed_beading_strategy.rs` (443) |
| `LimitedBeadingStrategy` (126+49) | `limited_beading_strategy.rs` (469) |
| `OuterWallContourStrategy` (82+28) | `outer_wall_contour_strategy.rs` (408) |
| `OuterWallInsetBeadingStrategy` (59+35) | `outer_wall_inset_beading_strategy.rs` (303) |
| `RedistributeBeadingStrategy` (97+56) | `redistribute_beading_strategy.rs` (428) |
| `WideningBeadingStrategy` (82+46) | `widening_beading_strategy.rs` (360) |
| `SkeletalTrapezoidation` (2144+585) | `skeletal_trapezoidation.rs` (3909) |
| `WallToolPaths` (903+152) | `wall_tool_paths.rs` (1480) |

`BeadingStrategyFactory::make_strategy` is a faithful 1:1 port — same chain
(Distributed -> Redistribute -> [Widening] -> [OuterWallInset] -> Limited),
including C++'s `#if 0` around `OuterWallContourStrategy`. `generate_junctions`
assigns `beading.bead_widths[junction_idx]` (SkeletalTrapezoidation.cpp:1847) and
the left/right beading interpolation mirrors :1760-1766.

### Correcting R541

R541 said "our Arachne emits effectively ONE width per loop", which invited
"our widths are constant". **They are not.** The distributions are close:

| `; FEATURE: Outer wall` widths | ours | C++ |
|---|---|---|
| min | 0.3501 | 0.2445 |
| p25 | **0.4000** | **0.4016** |
| median | 0.4000 | 0.4284 |
| p99 | **0.6209** | **0.6321** |
| max | 0.6555 | 0.8891 |

Both are dominated by the 0.40 bin (49% vs 40%) with a comparable tail out past
0.6 mm. Our widths vary across the model much as C++'s do.

### The actual gap, quantified

Bucketing `; LINE_WIDTH:` by feature-block (one block = one wall):

| | ours | C++ |
|---|---|---|
| Outer-wall feature blocks | 14,352 | 14,864 |
| `; LINE_WIDTH:` per block | **0.25** | **4.21** |
| blocks with >1 distinct width | **3.4%** | **28.0%** |
| mean within-block spread | **0.0176 mm** | **0.0707 mm** |

**Block counts match (0.97) — the wall structure is right.** What differs is that
C++ changes width *along* a wall 8x more often, with 4x the spread. Combined with
R541's `ARACHWIDTH` (98% of loops have `min == max` junction width), the target
is now exact:

> our per-loop beading is CONSTANT ALONG THE LOOP; C++'s varies with the local
> wall thickness.

That excludes the strategies, the factory, `generate_junctions`, and the path
builder (R541) — all verified faithful. It points at **beading propagation and
transitions across the skeleton** (`propagate_beadings_*`, the transition
machinery in `skeletal_trapezoidation.rs`), which is where a node's beading is
either recomputed for the local thickness or inherited unchanged.

### Sizing

This is E-neutral in aggregate (R541 measured material 0.9959 unchanged with the
faithful builder wired) and affects print fidelity along walls, not the parity
verdicts. It is the last known structural divergence in the wall path, and it is
a genuinely deep subsystem — `skeletal_trapezoidation.rs` is 3,909 lines. Sizing
the specific propagation gap is the next round's job, before any change.

**New discipline (R542): "constant" and "no variation" are different claims —
measure the distribution, not just the count of distinct values.** R541's
per-loop probe was right, but its wording invited the wrong conclusion; the
widths were varying across the model all along, and only the within-loop
variation is missing. A percentile table settled it in one command.

---

## R543 — the beadings are NOT flat; the flattening happens in WallToolPaths post-processing

**No behavioural change.** Two env-gated probes added (`BEADPROBE` -> `beadprobe`
+ `junctionprobe`); majora `2838b07f`, benchy `5a34af50`, cube `ab415621`
reproduce byte-identically, eight guard tests green.

### R542's localisation was wrong

R542 concluded "our per-loop Arachne beading is CONSTANT ALONG THE LOOP" and
pointed at beading propagation/transitions in `skeletal_trapezoidation.rs`.
Applying R541's own lesson one level further up — measure the INPUT before
suspecting the transform — shows that is **not** where the variation is lost.

**`beadprobe`** instruments both `BeadingStrategy::compute` call sites
(SkeletalTrapezoidation.cpp:1526 and :1887):

```
compute calls = 240,000
thickness       distinct = 140,776   range 0.024 .. 19.651 mm
bead_widths[0]  distinct =  25,295   range 0.190 ..  0.743 mm
multi-bead beadings with all-equal widths = 35,842 (15%)
```

The skeleton feeds `compute` a hugely varied local thickness, and the strategy
chain returns **25,295 distinct bead widths** spanning 0.19-0.74 mm — comparable
to C++'s 21,181 distinct outer-wall widths in the G-code. **The beading
generation is working.**

**`junctionprobe`** instruments the emission point (`generate_junctions`,
SkeletalTrapezoidation.cpp:1847), i.e. after propagation and interpolation:

```
junctions created = 6,400,000
distinct widths   =    28,419
range             = 0.000 .. 0.743 mm
```

**The variation is fully intact at ExtrusionJunction creation.**

### Where it is actually lost

| stage | width variation |
|---|---|
| `BeadingStrategy::compute` output | 25,295 distinct, 0.190-0.743 mm |
| `ExtrusionJunction` at creation | **28,419 distinct**, 0.000-0.743 mm |
| what `perimeter_generator` receives (R541 `ARACHWIDTH`) | **1.03 distinct per loop; 98% of loops flat** |

So the flattening happens **between `generate_junctions` and the `ExtrusionLine`s
that `WallToolPaths` returns** — i.e. in **WallToolPaths post-processing**, not in
the beading strategies, not in the factory, not in propagation/transitions, and
not in the skeleton's thickness computation. (6.4M junctions are created and only
~1.07M survive — 25,000 loops x 42.7 junctions — so a large merge/discard step is
demonstrably running there.)

Prime suspects for the next round, all inside `wall_tool_paths.rs` /
`arachne/utils/`: `ExtrusionLine::simplify` (merges junctions and can average
widths), `removeSmallLines`, `separateOutEndpoints`, and `PolylineStitcher`.
**Measure before porting — four rounds running (R539/R540/R541/R542) the named
suspect turned out to be already-present, inert, or mis-attributed.**

### Cumulative eliminations for this gap

Beading strategies (all seven), `BeadingStrategyFactory` chain,
`SkeletalTrapezoidation` thickness computation, `BeadingStrategy::compute`,
beading propagation/transitions, `generate_junctions`, the beading interpolation,
`thick_polyline_to_multi_path`, and the whole downstream path builder — **all
verified faithful and all shown to still carry the variation.**

**New discipline (R543): instrument the pipeline at BOTH ends of the suspect
span before blaming anything inside it.** One probe at `compute` and one at
junction creation bracketed the loss in a single round and moved the target from
a 3,909-line file to a different file entirely. R542's "constant along the loop"
was a correct observation of the OUTPUT paired with a wrong guess about where it
originated.

---

## R544 — none of the five post-processing stages flattens; the loss is inside `generate_toolpaths`

**No behavioural change.** One env-gated probe added (`STAGEPROBE` ->
`stageprobe`); majora `2838b07f`, benchy `5a34af50`, cube `ab415621` reproduce
byte-identically, eight guard tests green.

### R543's localisation was wrong — retracting my own

R543 concluded "the loss is downstream in WallToolPaths post-processing" and
named `ExtrusionLine::simplify` the strongest candidate. Bracketing **every**
stage (R543's own method, applied properly this time) disproves it.

C++ `WallToolPaths::generate` runs five stages after `generateToolpaths`
(WallToolPaths.cpp:534-542); ours runs the same five in the same order.
Measuring width variation after each:

| stage | lines | junctions | flat lines | distinct widths / line |
|---|---|---|---|---|
| 0 after `generate_toolpaths` | 57,248 | 5,892,955 | **87.4%** | **1.90** |
| 1 after `stitch_tool_paths` | 53,938 | 5,889,759 | 88.5% | 1.79 |
| 2 after `remove_small_lines` | 53,771 | 5,889,410 | 88.7% | 1.79 |
| 3 after `separate_out_inner_contour` | 40,001 | 4,099,688 | 84.7% | 2.06 |
| 4 after `simplify_tool_paths` | 40,001 | **1,287,683** | 85.0% | 1.68 |
| 5 after `remove_empty_tool_paths` | 40,001 | 1,287,683 | 85.0% | 1.68 |

**The lines are already 87.4% flat at stage 0**, before any post-processing runs.
Across all five stages the flat share moves 87.4% -> 85.0% — it goes *down*.

`simplify_tool_paths` is exonerated specifically: it discards **3.2x** the
junctions (4,099,688 -> 1,287,683) while barely touching the width variation
(distinct/line 2.06 -> 1.68). It removes redundant *points*, not *widths* — which
is exactly what C++'s `ExtrusionLine::simplify` is supposed to do.

(Sanity check that the probe measures the right population: stage 5 gives
859,995 junctions over 20,001 lines = 43.0 junctions/line, matching R541's
`ARACHWIDTH` figure of 42.7 junctions/loop at the perimeter generator.)

### Where it actually is

Both R543 measurements still stand: `generate_junctions` emits **28,419 distinct
widths** over 6.4M junctions. So the flattening happens **inside
`SkeletalTrapezoidation::generate_toolpaths`**, between `generate_junctions` and
the `ExtrusionLine`s it assembles — i.e. in **`connect_junctions`**
(SkeletalTrapezoidation.cpp:1574, ours at `skeletal_trapezoidation.rs:3336`),
with `generate_local_maxima_single_beads` (:1576 / :3559) as the only other
candidate in that span.

That is now a **two-function** target inside a known span, down from "somewhere
in a 1,480-line file".

### Running tally for this gap

Eliminated and *shown to still carry the variation*: all seven beading
strategies, the factory chain, the skeleton's thickness computation,
`BeadingStrategy::compute`, beading propagation/transitions,
`generate_junctions`, the beading interpolation, **all five WallToolPaths
post-processing stages (R544)**, `thick_polyline_to_multi_path`, and the whole
downstream path builder.

**New discipline (R544): when a bracket says "the loss is in span X", probe
INSIDE X at every stage before naming a function within it.** R543 bracketed
correctly but then guessed which stage of the span was responsible; one probe per
stage showed the span's own entry point was already flat, moving the target back
upstream into the function R543 had just measured the *start* of. Two rounds in a
row the correct observation came with a wrong guess attached — the guess is the
part worth deleting.

---

## R545 — `connect_junctions` is not the flattener; the beadings are SHARED, not near-equal

**No behavioural change.** One env-gated probe added (`CJPROBE` -> `cjprobe`);
majora `2838b07f`, benchy `5a34af50`, cube `ab415621` reproduce byte-identically,
eight guard tests green.

### `connect_junctions` exonerated

R544 narrowed the flattening to two functions inside `generate_toolpaths`.
`connect_junctions` pairs a `from` junction with a `to` junction for each segment
it builds (SkeletalTrapezoidation.cpp:2067-2068). Probing that pairing:

```
segments = 6,000,000
from.w == to.w : 5,877,959  (98.0%)
diff < 1um     :    11,582
diff 1-10um    :    47,601
diff > 10um    :    62,858
```

**98% of segments arrive with both ends already carrying the identical width.**
`connect_junctions` is faithfully chaining junctions that already agree — it is
not the flattener.

### The tell is EXACT equality

Two beadings computed independently from slightly different local thicknesses
would differ in the last digit or two. **Exact** equality at 98% means the same
beading is being *shared*, not recomputed.

That fits the structure: `generate_junctions` stamps
`beading.bead_widths[junction_idx]` from **one beading per edge**
(SkeletalTrapezoidation.cpp:1847) — so every junction on a given edge necessarily
shares a width, in C++ too. Variation along a wall can therefore only come from
**adjacent edges holding different beadings**. Ours mostly hold the same one.

### This re-opens beading propagation — on my own bad elimination

R542 and R543 struck beading propagation off the list. That elimination rested on
`beadprobe` (25,295 distinct widths out of `compute`) and `junctionprobe` (28,419
distinct at creation) — both **global** counts. R542's own lesson says a global
distinct-count and within-wall variation are different claims, and I then used
the former to dismiss the latter. **The elimination was invalid; propagation is
back in scope.**

The specific sharing mechanisms to measure next, both inside the span R544
already bracketed:

- **`get_or_create_beading`** (SkeletalTrapezoidation.cpp:1852-1892, ours
  `:3064`) calls `get_nearest_beading(node, nearby_dist)` with
  `nearby_dist = scaled(0.1)` — it **reuses a neighbouring node's beading within
  0.1 mm** rather than computing a fresh one.
- **`propagate_beadings_upward` / `_downward`** (:1608 / :1637 / :1660, ours
  `:2648` / `:2697` / `:2730`) copy beadings along edges.

240,000 `compute` calls against 6.4M junctions is ~1 beading per 27 junctions, so
heavy sharing is expected in both engines — **the open question is whether C++
shares *less*, or interpolates where we copy.** Measure the distinct-beading
count per connected wall region on our side, then read C++'s propagation against
ours (R539: read before assuming).

### Running tally

Exonerated this round: **`connect_junctions`**. Still eliminated: the seven
beading strategies, the factory chain, `BeadingStrategy::compute`,
`generate_junctions` itself, all five WallToolPaths post-processing stages,
`thick_polyline_to_multi_path`, the downstream path builder. **Un-eliminated
(R545): beading propagation and `get_or_create_beading`'s nearest-beading reuse.**

**New discipline (R545): EXACT equality and NEAR equality point at different
causes — exact means shared state, near means independent computation.** The 98%
figure alone looked like "the widths agree"; that they agree *to the last digit*
is what identified sharing rather than smoothness, and it is what re-opened a
suspect I had wrongly closed two rounds earlier.

---

## R546 — the whole propagation chain is faithful; the sharing is STRUCTURAL

**No behavioural change.** One env-gated probe added (`PROPPROBE` -> `propprobe`);
majora `2838b07f`, benchy `5a34af50`, cube `ab415621` reproduce byte-identically,
eight guard tests green.

### Every function in the chain checked line-for-line

| C++ | ours | verdict |
|---|---|---|
| `propagateBeadingsUpward` :1608-1635 | `:2650` | faithful, incl. `dist_to_bottom_source += length` (:1629) |
| `propagateBeadingsDownward` :1637-1658 | `:2699` | faithful |
| `propagateBeadingsDownward(edge)` :1660-1706 | `:2730` | faithful, incl. the `ratio_of_top >= 1.0` copy branch |
| `interpolate` :1709-1749 / :1752-1771 | `interpolate4` / `interpolate2` | faithful |

The constant was read rather than assumed (R490/R525):
`beading_propagation_transition_dist` is **0.400 mm** at runtime, from
`wall_transition_length = '100%'`, wired from `WallToolPaths.cpp:529` on both
sides.

### The copy/blend split, measured

`propprobe` on the merge path (40,000 calls):

```
ratio_of_top >= 1.0  (pure COPY of top onto bottom, :1691)  =  7,851  (19.6%)
ratio_of_top == 0    (interpolate returns bottom UNCHANGED) = 27,980  (70.0%)
genuine blend                                                ~ 10%
total_dist < transition_dist                                = 11,869
```

`ratio_of_top == 0` means `dist_to_bottom_source == 0` — the node's beading did
not arrive by upward propagation, so there is nothing to blend from. Only ~10% of
merges actually mix two beadings.

And the dominant path is not even this one: most nodes take
`if (!from->hasBeading()) { propagated_beading = top_beading; }`
(C++:1673) — **a full copy of the top node's beading**, which is where the shared
objects R545 detected come from.

**All of that is faithful to C++.** The heavy sharing is structural, not a
porting defect I can point at.

### Honest limit of this round

I have now verified every function between `BeadingStrategy::compute` and the
emitted G-code — strategies, factory, skeleton thickness, propagation up and
down, both interpolations, `generate_junctions`, `connect_junctions`, all five
WallToolPaths stages, the variable-width builder, and the path builder. **Each is
faithful, and the sharing that flattens our walls is produced by faithful code.**

So either an *input* to this chain differs (node `distance_to_boundary`
distribution, transition generation, the skeleton graph itself), or C++ exhibits
the same sharing and the width variation C++ emits arises somewhere I have not
yet identified.

**Rust-side probing has reached its limit.** The decisive next step is to
**instrument the C++ binary with the same three counters** — `compute` calls,
the `propprobe` copy/blend split, and per-line distinct widths at stage 0 — and
compare like for like. The C++ tree builds with
`ninja slicer_cli`, so this is mechanical rather than speculative, and it is the
only measurement that can distinguish "our sharing is wrong" from "C++ shares
equally and differs elsewhere".

**New discipline (R546): when every function in a chain reads faithful and the
behaviour still differs, stop auditing the port and instrument the REFERENCE.**
Nine rounds (R538-R546) have each eliminated a suspect by measuring our side; the
one measurement never taken is what C++ actually does at the same points. R516's
"run the reference-vs-itself control" generalises: when the port looks right,
make the reference report its own numbers.

## R547 — the C++ reference instrumented: the gap is `bead_count` assignment

R546 predicted this round would be mechanical. It was, and it answered the
question in one shot. I ported the three Rust probes into the C++ tree
(`beadprobe`, `propprobe`, `stageprobe`; env-gated, additive, ~169 lines) and ran
both engines on Majora. The patch is preserved at
`scripts/arachne-parity-probes.patch`; the submodule was reverted and rebuilt at
the end of the round, and both status checks are clean.

### Two corrections found on the way in

**C++ `SCALING_FACTOR` is 1e-5, not 1e-6.** `libslic3r.h:58` reads
`static constexpr double SCALING_FACTOR = 0.00001;` — the same scale the Rust
crate uses. My first probe printed everything 10x small; the bead-width range
gave it away (0.022..0.071 mm is not a plausible extrusion width). Divisors
corrected before any number below was recorded. This confirms R487's retraction
of the "C++ is 1e6" claim, which had crept back into the round-to-round notes.

**The C++ engine is not byte-reproducible.** Four runs of the same binary on the
same input give four md5s (`78a87a55`, `2ea52d0f`, `84d713ac`, `db3dc2fb`), with
total filament differing in the 5th significant figure (65094.36 vs 65095.27 mm).
So the reference-vs-itself control is mandatory before trusting any C++-derived
metric. Running `lwblock.py` on two independent C++ runs:

| metric | C++ run A | C++ run B |
|---|---|---|
| outer-wall feature blocks | 14,865 | 14,864 |
| `; LINE_WIDTH:` per block | 4.21 | 4.21 |
| blocks with >1 distinct width | 28.0% | 28.0% |
| within-block spread | 0.0707 mm | 0.0708 mm |

**The width metric is stable to three decimals across the nondeterminism.** The
0.25-vs-4.21 gap this campaign has been chasing is real, not run-to-run noise.

### The deciding comparison: stage-0 flat%

| stage | lines R | lines C++ | flat% R | flat% C++ | distinct w/line R | distinct w/line C++ |
|---|---|---|---|---|---|---|
| 0 after generate_toolpaths | 57,304 | 130,578 | **86.9** | **67.8** | 2.07 | 3.36 |
| 1 after stitch_tool_paths | 53,976 | 111,649 | 88.1 | 72.9 | 1.93 | 3.16 |
| 2 after remove_small_lines | 53,772 | 108,307 | 88.3 | 73.0 | 1.93 | 3.21 |
| 3 after separate_out_inner_contour | 40,002 | 80,001 | 84.2 | 63.4 | 2.25 | 3.99 |
| 4 after simplify_tool_paths | 40,002 | 80,001 | 84.4 | 63.8 | 1.79 | 2.60 |
| 5 after remove_empty_tool_paths | 40,002 | 80,001 | 84.4 | 63.8 | 1.79 | 2.60 |

**C++ is 67.8% flat at stage 0 where we are 86.9%.** The divergence is already
present the moment Arachne finishes, before any post-processing. R538-R546 were
aimed at the right subsystem. Note also that C++'s post-processing chain *also*
leaves flat% roughly where it found it — neither engine loses variation
downstream, exactly as R544 measured on our side.

### The mechanism: 5x fewer of our nodes carry a bead count

`propprobe` came back inverted, which pointed upstream:

| | Rust | C++ |
|---|---|---|
| `ratio_of_top >= 1.0` (pure COPY) | 20.1% | **61.6%** |
| `ratio_of_top == 0` (bottom unchanged) | **69.8%** | 16.8% |
| `beading_propagation_transition_dist` | 0.400 mm | 0.400 mm |

`ratio_of_top == 0` means `dist_to_bottom_source == 0` — the bottom node's
beading never arrived by upward propagation. Ours is at zero 4x as often. And
`beadprobe` showed C++ calling `BeadingStrategy::compute` ~1,260,000 times to our
~240,000. So a new `graphprobe` on both engines, at the head of
`generateSegments`, measured the skeleton itself:

| per `generate_segments` call | Rust | C++ | ratio |
|---|---|---|---|
| graph nodes | 135.6 | 192.4 | 1.42x |
| graph edges | 269.1 | 382.8 | 1.42x |
| `upward_quad_mids` | 35.6 | 49.9 | 1.40x |
| **nodes with `bead_count > 0`** | **9.90** | **31.07** | **3.14x** |

As a share of the graph: **7.3% of our nodes carry a bead count, against 16.1%
of C++'s.** In absolute terms 255,502 vs 1,273,750 — a ratio of **4.99x**, which
is exactly the 5x we independently measured in `compute` call counts. The two
counters close on each other.

The causal chain is now fully quantified and consistent end to end:

> **~2.2x smaller share of nodes marked with a bead count -> 5x fewer
> `BeadingStrategy::compute` calls -> far fewer distinct beadings along a wall ->
> 86.9% flat vs 67.8% at stage 0 -> 0.25 vs 4.21 `; LINE_WIDTH:` per outer-wall
> block.**

Every function *downstream* of `bead_count` has been verified faithful across
R542-R546. The defect is at or above the point where `bead_count` is assigned —
i.e. central-region marking and bead-count assignment
(`setMarking`/`filterMarking`/`setBeadCount` and the transition machinery), none
of which this campaign has yet audited.

**R548 target:** instrument and then audit bead-count assignment. Counting nodes
by `bead_count` value and by central/non-central marking on both engines will say
whether we mark fewer nodes central, or mark the same nodes and assign
`bead_count <= 0` more often.

**New discipline (R547): a ratio that is stable under the reference's own
nondeterminism is a real signal; one that is not is noise.** The C++ engine is
not byte-reproducible, so every C++-derived metric needs a two-run control before
it can carry a conclusion — the byte-level instability here was ~1e-5 in
material while the metric under investigation differed by 17x, and only the
control could establish that separation.

## R548 — the divergence is `updateIsCentral`, and one of my probes is wrong

### First: R547's saved patch was not usable

`scripts/arachne-parity-probes.patch` does not apply — it contains no
`diff --git` line at all. `git diff > file` in this environment yields a
rendered summary, not a patch, and R547 recorded it as reusable without ever
testing that claim. Replaced with `scripts/inject-arachne-probes.py`, which does
idempotent string injection, fails loudly if any anchor is missing or ambiguous,
and has a `--check` mode. **This round it was applied, built, run, reverted, and
re-applied end to end before being committed** — the test R547 skipped.

### The census: the gap is present at the first marking stage

`CENTRALPROBE` counts central edges and bucketed `bead_count` after each of the
four marking stages, on both engines:

| stage | central/edge R | central/edge C++ | bead>0/node R | bead>0/node C++ |
|---|---|---|---|---|
| 0 after `updateIsCentral` | **3.212%** | **11.199%** | 0% | 0% |
| 1 after `filterCentral` | 3.212% | 11.199% | 0% | 0% |
| 2 after `updateBeadCount` | 3.212% | 11.199% | 5.301% | 12.864% |
| 3 after `filterNoncentralRegions` | 5.276% | 15.413% | 6.773% | 16.048% |
| 4 after `generateTransitioningRibs` | 5.398% | 15.505% | 6.887% | 16.126% |

**The 3.5x deficit is fully present at stage 0.** `filterCentral` moves nothing
on either engine, so it is exonerated. `updateBeadCount` then converts central
edges into bead counts at the same rate on both sides — it is faithful, and it
merely inherits a marking that is already wrong. Every later stage preserves the
ratio. This confirms R547's chain and pushes its head one step further back:

> `updateIsCentral` marks 3.5x too few edges -> 2.2x smaller share of nodes with
> a bead count -> 5x fewer `compute` calls -> 86.9% vs 67.8% flat at stage 0.

### Inside `updateIsCentral`: constants identical, branch mix identical

`ISCPROBE` counts which of the four branches decides each edge:

| | Rust | C++ |
|---|---|---|
| `outer_edge_filter_length` | 0.0500 mm | 0.0500 mm |
| `cap = sin(transitioning_angle/2)` | 0.087156 | 0.087156 |
| branch: twin already set (copy) | 50.0% | 50.0% |
| branch: `EXTRA_VD` -> false | 31.8% | 31.6% |
| branch: below filter length -> false | 0.03% | 0.23% |
| branch: geometric `dR < dD*cap` | 18.1% | 18.2% |

Both constants are bit-identical at runtime and the branch mix matches. We are
not misrouting edges into the wrong branch — **the divergence is the outcome of
the geometric test `dR < dD * cap` alone**, where `dR = |to.dtb - from.dtb|` and
`dD = |ab|`. Since `cap` is identical, `distance_to_boundary` and/or the edge
geometry must differ.

### An honest negative: my Rust-side counters contradict each other

I added a `GEOMPROBE` to bucket `dR/dD` and it disagreed with the branch probe.
Within a single function call, on all-positive inputs, it reported 232,912 edges
with `dR/dD < 0.0436` but only 20,526 satisfying `dR < dD * 0.087156` — which is
arithmetically impossible, since the first condition implies the second. A third
count (the branch probe's own) gave a fourth answer, 5.09%.

**On the C++ side the same two counters agree** (29.17% of geom edges below cap
vs a 29.43% central rate). The inconsistency is Rust-side only.

I could not isolate it this round, so I removed `GEOMPROBE` rather than leave a
tool that reports numbers I cannot reproduce two ways. **No Rust geometric-branch
percentage from this round should be quoted.** The three claims above survive
because they rest on `CENTRALPROBE` and `ISCPROBE`, which cross-check each other
on the C++ side and use identical methodology on both engines.

That asymmetry is itself the strongest lead into R549: a Rust counter that
disagrees with itself about state read microseconds apart suggests the graph is
being mutated or re-entered between the probe and the census — for instance
`update_is_central` running on a graph that is later extended, or edges whose
`central_is_set` is stale by the time the census walks them. Note the supporting
hint: our `ISCPROBE` total central (1.85% of edges) and our `CENTRALPROBE`
central (3.21%) differ by 1.7x, where the same two C++ numbers agree to 1.05x.

**New discipline (R548): a probe that cannot be reproduced by a second,
independently-written counter is not a measurement.** R547's corollary said two
counters that close on each other are worth more than one measured twice; the
converse bites here. And: **never record a tool as reusable without exercising
it** — the patch R547 saved had never been applied even once.

## R549 — the bug: `wall_transition_angle` was stored in radians in a degrees field

Twelve rounds of localisation (R538-R548) end here, and R548's blocker turned out
to be the thing pointing at the answer.

### The contradiction was two different `cap` values, not bad arithmetic

R548's `GEOMPROBE` reported a histogram and a direct comparison that could not
both be true. I re-added it self-checking: it now counts the invariant violation
itself (`ratio < cap/2` implies `dR < dD*cap`) and dumps offending inputs.

**Violations: zero.** The arithmetic was always sound. What the probe exposed
instead was printed right next to it:

```
[GEOMPROBE2] n=500000  ... cap=0.001523087
[GEOMPROBE2] n=1000000 ... cap=0.087155743
```

**`cap` had two runtime values differing by 57.3 = 180/pi.** R548's histogram and
its direct test had simply been evaluated on calls with different `cap`. This
also **retracts R548's "constants are identical"**: `ISCPROBE` printed only on a
sparse modulo and happened to sample the correct-`cap` population every time.

### Root cause

`sin(deg2rad(deg2rad(10)) / 2) = 0.0015231`, against the correct
`sin(deg2rad(10)/2) = 0.0871557`. The double conversion is in our
`WallToolPathsParams` default (`wall_tool_paths.rs:94`):

```rust
wall_transition_angle: 0.174533, // ~10 degrees
```

The field holds **degrees** — `deg2rad` is applied to it at
`WallToolPaths.cpp:456` and in our mirror at `wall_tool_paths.rs:898`. Storing
10 degrees *already in radians* made the conversion run twice, so `cap` in
`updateIsCentral` came out 57.3x too small and `dR < dD*cap` almost never held.

C++ has no such default: every C++ construction site assigns degrees
(`PerimeterGenerator.cpp:1551` passes `object_config->wall_transition_angle.value`,
`FillConcentric.cpp:89` assigns literal `10`).

### Effect, behind `ARACHNE_WTP_ANGLE_DEG` (default ON; gate OFF reproduces `2838b07f` byte for byte)

| measurement | before | after | C++ |
|---|---|---|---|
| central/edge after `updateIsCentral` | 3.212% | **11.774%** | 11.199% |
| bead>0/node after `updateBeadCount` | 5.301% | **13.486%** | 12.864% |
| central/edge after `generateTransitioningRibs` | 5.398% | **15.964%** | 15.505% |
| outer-wall within-block width spread | 0.0176 mm | **0.0738 mm** | 0.0707 mm |
| `; LINE_WIDTH:` per outer-wall block | 0.25 | **1.20** | 4.21 |
| object material ratio | 0.9959 | **0.9973** | 1.0 |
| wall-lines IoU (area-wtd) | 95.19% | **95.28%** | 100% |

Every stage of the marking chain now lands within ~4% of C++ instead of 3.5x
short, and **the within-wall width spread now matches C++** (0.0738 vs 0.0707 mm)
where it was 4x too flat. Verdict remains SEMANTICALLY EQUIVALENT with object
material and wall-lines IoU both improved. Benchy (`5a34af50`) and cube
(`ab415621`) are unchanged — Benchy uses the classic wall generator. Majora
re-baselines **`2838b07f` -> `d7767da4`**.

### What is still open

`; LINE_WIDTH:` changes per outer-wall block are 1.20 against C++'s 4.21 — a 5x
improvement on 0.25, but not closed. The remaining gap is no longer the *amount*
of width variation (the spread matches) but how often it changes along a wall,
which is consistent with the untouched half of this defect: **our
`perimeter_generator` builds `WallToolPathsParams::default()` and never reads
`object_config` at all** (`perimeter_generator.rs:2968`), where
`PerimeterGenerator.cpp:1537-1553` populates six fields, four of them scaled by
`0.01 * min_nozzle_diameter`. Three of those defaults coincide with the resolved
config values for this fixture, which is why only the angle bit us — but
`wall_transition_filter_deviation` (ours 0.025 against a config default of 0.25%
of nozzle) and `wall_distribution_count` have not been checked against resolved
values. **R550: port `PerimeterGenerator.cpp:1537-1553` faithfully and print the
six resolved params on both engines.**

**New discipline (R549): when two counters disagree, make the probe test the
invariant rather than re-reporting the numbers.** Counting "how often is the
impossible thing true" found in one run what re-reading the code could not — and
the answer was that an input I had recorded as a constant was not constant. A
value printed on a sparse modulo is a *sample*, not a constant; R548 published it
as a constant and was wrong.

## R550 — the Arachne params are now sourced from config; near-inert on the width metric

R549 fixed one wrong-unit default. This round ports the code that should have
been supplying those values all along, and finds a second wrong-unit default in
the process — but the expected width improvement did **not** materialise.

### Step 1: the six resolved params, measured on both engines

A new `WTPPARAMS` probe prints the resolved `WallToolPathsParams` (deduped) at
`WallToolPaths::generate` on both engines. C++ printed **one** parameter set;
we printed **two**, and exactly one field differed:

| field | ours (perimeter path) | ours (fill path) | C++ |
|---|---|---|---|
| `min_bead_width` | 0.340 | 0.340 | 0.340 |
| `min_feature_size` | 0.100 | 0.100 | 0.100 |
| `wall_transition_length` | 0.400 | 0.400 | 0.400 |
| `wall_transition_angle` | 10 deg | 10 deg | 10 deg |
| **`wall_transition_filter_deviation`** | **0.025** | 0.100 | **0.100** |
| `wall_distribution_count` | 1 | 1 | 1 |

Our fill path was already a faithful port of `FillConcentric.cpp:86-91`; only the
perimeter path, which used `WallToolPathsParams::default()` wholesale, was wrong
— and only in one field, 4x too small.

### The second wrong-unit default

Printing the *resolved object-config* values (they are not reachable from
`PerimeterGenerator`, which is why the default was used in the first place):

```
arachne_min_bead_width=85  arachne_min_feature_size=25  arachne_wall_transition_length=100
wall_transition_angle=10   wall_transition_filter_deviation=0.25   nozzle_diameter=0.4
```

Four are percentages exactly as C++ expects; `wall_transition_filter_deviation`
had been stored as the fraction `0.25` where C++'s option default is `"25%"`, so
`v * 0.01 * min_nozzle_diameter` gave 0.001 mm instead of 0.1 mm. **The same
wrong-unit-in-a-default class as R549.** Corrected to `25.0`.

### Step 2: the port

`PerimeterConfig` gained the six raw option values plus `min_nozzle_diameter`
(copied across in `LayerRegion`, since our `PerimeterGenerator` has no config
back-pointer), and the arithmetic now sits at the C++ line it mirrors, behind
`ARACHNE_WTP_PARAMS` (default ON; gate OFF reproduces `d7767da4` byte for byte).
After it, we print **one** parameter set, identical to C++'s.

### The honest negative

R550's stated expectation was that `; LINE_WIDTH:` per outer-wall block would
move from 1.20 toward C++'s 4.21. **It did not:**

| metric | R549 | R550 | C++ |
|---|---|---|---|
| `; LINE_WIDTH:` per outer-wall block | 1.20 | **1.19** | 4.21 |
| blocks with >1 distinct width | 12.9% | 12.8% | 28.0% |
| within-block width spread | 0.0738 mm | 0.0737 mm | 0.0707 mm |
| object material | 0.9973 | 0.9973 | 1.0 |
| wall-lines IoU | 95.28% | 95.28% | — |
| silhouette (area-wtd) | 99.53% | 99.53% | — |

Quadrupling `wall_transition_filter_deviation` moved the width metric by 0.01 and
every parity verdict by nothing. The change is **parity-neutral**. It is kept
anyway because it is correct by construction: our resolved parameters now match
the reference exactly, and the old code would have diverged on any fixture that
overrides these options — this fixture simply happened to make three of the six
defaults coincide. Verdict remains SEMANTICALLY EQUIVALENT; Majora re-baselines
**`d7767da4` -> `4f9de6fe`**; benchy, cube and the STL fixtures are unchanged.

### What this rules out, and what is left

`wall_transition_filter_deviation` is now **eliminated** as the cause of the
remaining width-change-frequency gap, and so is the whole parameter-plumbing
hypothesis: every input to Arachne now matches C++ bit for bit, yet we still
change width along a wall a third as often. The remaining quantity is *how often*
width changes, not by how much (the spread has matched since R549).

**R551 candidates, none yet probed:** the transition machinery itself
(`generateTransitionEnds`, `filterTransitionMids`, `dissolveNearbyTransitions` --
never audited in this campaign), and `get_or_create_beading`'s
`get_nearest_beading(node, scaled(0.1))` hit rate, still the one function in the
beading chain never individually measured. Measure the transition COUNT per graph
on both engines first -- it is the direct analogue of the failing metric.

**New discipline (R550): when a fix is parity-neutral, say so and keep it only
if it is correct by construction.** Three of six defaults coincided with the
resolved config values for this fixture, which is exactly why a twelve-round hunt
found only one of them. A coincidence that holds on your only fixture is not a
match — the probe that prints all N values side by side costs one build.

## R551 — transitions and beading reuse both cleared; the gap moves to line COUNT

Two hypotheses, both eliminated, plus a re-measurement that changes where the
remaining gap lives. No behavioural change; Majora stays at `4f9de6fe`.

### Transitions are not the cause

A new `TRANSPROBE` censuses the transition pipeline after each of its four
stages on both engines, per `generate_toolpaths` call:

| stage | edges_with/call R | C++ | items/call R | C++ |
|---|---|---|---|---|
| 0 after `generateTransitionMids` | 0.31 | 0.26 | 0.32 | 0.27 |
| 1 after `filterTransitionMids` | 0.31 | 0.26 | 0.21 | 0.17 |
| 2 after `generateAllTransitionEnds` | 0.31 | 0.26 | 0.21 | 0.17 |
| 3 after `applyTransitions` | 0.31 | 0.26 | 0.21 | 0.17 |

We create **more** transitions per call than C++, not fewer, and both engines
filter with an **identical retention ratio** (items 7,793 -> 5,065 = 65% ours;
10,710 -> 6,971 = 65% C++). The whole machinery — `generateTransitionMids`,
`filterTransitionMids`, `dissolveNearbyTransitions`, `generateAllTransitionEnds`,
`applyTransitions` — is behaviourally faithful.

The round's stated premise was also wrong in magnitude: at ~0.2-0.3 transitions
per call, transitions are two orders of magnitude too rare to account for 4.21
`; LINE_WIDTH:` changes per feature block. **A "direct analogue" has to be
checked for scale, not just for direction.**

### `getNearestBeading` is dead code on this fixture — on both engines

`GNBPROBE` on `getOrCreateBeading`:

```
RUST calls=3,200,000 | already_had_beading=3,200,000 | bead_count==-1=0 | HIT=0
CPP  calls=7,400,000 | already_had_beading=7,400,000 | bead_count==-1=0 | HIT=0
```

The `bead_count == -1` branch — the only path that reaches
`getNearestBeading` — is **never taken on either engine**. The last unprobed
function in the beading chain turns out not to run at all here. Eliminated, and
the shared-beading hypothesis it stood for goes with it.

### The re-measurement that matters: R549/R550 closed 61% of the Arachne gap

The `stageprobe` figures this campaign has been quoting were measured *before*
R549. Re-run:

| stage | flat% R (pre-R549) | flat% R (now) | flat% C++ | distinct/line R now | C++ |
|---|---|---|---|---|---|
| 0 after `generate_toolpaths` | 86.9 | **75.2** | 67.8 | 2.44 | 3.36 |
| 5 after post-processing | 84.4 | **71.5** | 63.9 | 2.14 | 2.60 |

The stage-0 flat% gap went from 19.1pp to 7.4pp — **61% of it closed by the two
unit fixes.** But the scope check (R507) now matters: at stage 5 our Arachne
output differs from C++ by only **1.21x** in distinct widths per line, while the
emitted G-code differs by **3.5x** in `; LINE_WIDTH:` per feature block. Those
cannot both be describing the same deficit.

The reconciling quantity is line **count**: C++ produces **80,000** ExtrusionLines
where we produce **40,001** — 2x — against matching feature-block counts
(14,538 vs 14,864). C++ therefore packs about twice as many lines into each
feature block, each contributing its own width changes. Total distinct-width
instances: C++ ~208,000 vs ours ~85,600 = **2.43x**, which is the right order for
the 3.5x G-code metric where 1.21x is not.

This is the structural 2.3x graph-size difference first seen in R547 (C++ 14M
edges vs our 6M) and deprioritised then because the *normalised* bead_count share
was the dominant signal. **Those normalised signals now match, so the structural
difference is what is left.** `discretization_step_size` is not the cause — it is
`scaled(0.8)` on both sides, verified this round.

**R552: chase the 2x ExtrusionLine / 2.3x graph-edge count.** Measure where the
edge count diverges — Voronoi construction, `discretize`, or the polygon
preparation feeding it (`prepared_outline`, the triple `offset` at
`WallToolPaths.cpp:461`) — before touching anything downstream.

**New discipline (R551): check a proposed "direct analogue" for SCALE before
spending a round on it.** Transitions were the right kind of quantity and the
wrong size by two orders of magnitude; one arithmetic sanity check up front
would have redirected the round. Related: when an internal metric differs by
1.21x and the output metric it supposedly drives differs by 3.5x, the mismatch
itself is the finding — something other than the per-unit quantity is scaling.

## R552 — the 2x is in Arachne's INPUT, not in Arachne

The bracket closed on the first try. No behavioural change; Majora stays at
`4f9de6fe`.

### Constants first

`meshfix_maximum_resolution` = `scaled(0.5)` and `meshfix_maximum_deviation` =
`scaled(0.025)` on **both** sides (`WallToolPaths.hpp:19-20` against
`wall_tool_paths.rs:45,48`). After R549 and R550 these were the prime unit-bug
suspects; they are clean.

### The measurement

A new `POLYPROBE` counts polygons and points at four points of the
`prepared_outline` preparation chain, on both engines:

| stage | polys R | polys C++ | points R | points C++ | ratio (pts) |
|---|---|---|---|---|---|
| **0 `outline` (the INPUT)** | **23,438** | **44,201** | **1,135,330** | **2,184,992** | **1.92x** |
| 1 after triple offset | 23,049 | 42,558 | 1,129,541 | 2,189,542 | 1.94x |
| 2 after simplify | 23,043 | 42,351 | 692,442 | 1,376,361 | 1.99x |
| 3 final `prepared_outline` | 22,974 | 41,208 | 687,381 | 1,362,492 | 1.98x |

Three things follow immediately:

1. **`WallToolPaths::generate` is called the same number of times** — 48,000
   against 48,001. The 2x is not extra invocations.
2. **The whole preparation chain is faithful.** The ratio enters at 1.92x and
   leaves at 1.98x; `simplify` removes 39% of our points against 37% of C++'s.
   Nothing in the offset / simplify / fix-self-intersections / union sequence
   creates or destroys the gap — it passes straight through.
3. **Points per polygon are nearly identical** — 48.4 ours against 49.4 C++. So
   this is not "C++ polygons are more detailed". It is **~1.9x more polygon
   contours**, each of comparable complexity.

**The divergence is upstream of Arachne entirely.** Everything this campaign has
examined since R538 — beading strategies, propagation, transitions, central
marking, the parameter plumbing — sits *downstream* of an input that already
differs by 2x. R549's unit fix was still a real defect and still improved parity;
but the remaining `; LINE_WIDTH:` gap is inherited, not generated, by Arachne.

### What this means and what it does not

The silhouette (99.53%) and object material (0.9973) both match, so the **area**
being handed in is right. What differs is how that area is **partitioned into
contours**: C++ passes roughly twice as many separate polygons covering the same
region. That is a topology/fragmentation difference in `last_p` — the region
area remaining after previous insets, `to_polygons(last)` in the caller — not a
geometry error.

Note this does **not** contradict R513's elimination of "the sliced geometry
itself": R513 compared sliced *area* and found it sound, and it still is. Contour
*count* is a different quantity, never measured until now (R507: check what a
metric scopes).

**R553: measure the polygon count of `last` / `last_p` at the point
`PerimeterGenerator` hands it to `WallToolPaths`, and walk back** through the
inset loop to whichever operation first halves it — the `offset_ex` chain that
produces `last`, or a union/simplify we apply that C++ does not. Both engines
have the call site instrumented already; extend `POLYPROBE` upward rather than
starting a new probe.

**New discipline (R552): when a ratio is constant across every stage of a
pipeline, stop probing the pipeline and probe its input.** Four stages all
reported ~1.9x; that flatness was the signal, and one more probe *before* stage 0
would have found it in R547 rather than R552. A pipeline that preserves a ratio
is exonerated by that very fact.

## R553 — the region surfaces themselves differ: 1.5x more, half the size, no holes

R552 put the 2x in Arachne's input. This round walks one level further up and
finds it is already present in the surfaces `PerimeterGenerator` iterates. No
behavioural change; Majora stays at `4f9de6fe`.

### A divergence found by reading, before measuring

Our `generate_arachne` builds `last` as

```rust
let simplified = union_polygons_ex(&surface.simplify_p(surface_simplify_resolution));
let last = offset_expolygons(&simplified, inset, join_type);
```

against C++ `PerimeterGenerator.cpp:1511`:

```cpp
ExPolygons last = offset_ex(surface.expolygon.simplify_p(surface_simplify_resolution), -...);
```

**C++ has no `union_ex` here.** Our own comment two functions away
(`generate_classic_one`, `:502`) still claims C++ reads
`union_ex(surface.expolygon.simplify_p(...))` — a decayed claim (R539). The
classic path gates that union behind `F1_UNION`; the arachne path applies it
unconditionally. A union over a contour-plus-holes polygon list is exactly the
operation that merges contours while preserving outer area — the signature R552
measured. Worth fixing regardless, but the measurement below shows it is not the
origin.

### `LASTPROBE`: contours and holes counted separately

The C++ probe needed a second translation unit, so the injector now covers
`PerimeterGenerator.cpp` as well and the revert is still one command.

| per surface | Rust | C++ | ratio |
|---|---|---|---|
| calls (surfaces iterated) | 24,000 | 16,000 | **1.50x more** |
| contours | 1.000 | 1.000 | 1.00x |
| **holes** | **0.0045** | **0.1066** | **23.7x fewer** |
| points | 51.1 | 96.2 | **1.88x smaller** |
| `last` ExPolygons surviving the inset | 0.53 | 0.94 | — |

Reading these together:

1. **The surfaces entering `process_arachne` already differ.** Ours are 1.5x more
   numerous and each carries half the points. The 1.9x contour count R552 saw at
   Arachne's door is the product of that, not something the perimeter generator
   does.
2. **We have essentially no holes** — 108 against 1,706 over comparable
   populations. Since total area matches (0.9973), the holes are not being
   *filled*; the regions are being *partitioned differently*, with C++ keeping a
   contour-with-holes where we hold several separate contours.
3. **47% of our surfaces collapse entirely** under the negative inset
   (0.53 ExPolygons out per surface in), against 6% for C++. That is the direct
   consequence of feeding surfaces half the size: small ones vanish when inset by
   half the external perimeter width.

**So the divergence is upstream of `PerimeterGenerator` too** — in the region
surfaces handed to it. `generate_arachne` takes `&[ExPolygon]`, not a
`SurfaceCollection`, so surface identity and type are already flattened by the
caller before the perimeter generator ever runs.

### R554

Measure the surface count, hole count and points-per-surface of the region's
`slices` where `LayerRegion` assembles them, and walk back to whichever step
splits one C++ surface into ~1.5 of ours. Prime suspects, in order: the
`ExPolygon` flattening at the `generate_arachne(&[ExPolygon])` boundary; region
slice assembly in `layer.rs`; and `union_ex`/`union_safety_offset_ex` in surface
construction. Separately, remove the unconditional `union_polygons_ex` above so
the arachne path matches `PerimeterGenerator.cpp:1511` — behind its own gate,
A/B'd, and expected to be small on its own.

**New discipline (R553): count a compound structure's parts separately before
comparing totals.** "1.9x more polygons" was true and nearly useless; splitting
it into surfaces / contours / holes / points-per-surface turned one number into
four, three of which disagreed in different directions and together named the
stage. Totals hide compensating differences — R507's "check what a metric scopes"
applied to the *shape* of the datum, not just its extent.

## R554 — the flattening is faithful; a retraction; one small correct-by-construction fix

### The cheapest hypothesis, killed

R553 proposed that flattening `SurfaceCollection` to `&[ExPolygon]` might split an
ExPolygon-with-holes into several hole-free ones, explaining both the 1.5x
surface count and the 23.7x hole deficit at once. **It does not.** Every link is a
1:1 map:

| step | code | behaviour |
|---|---|---|
| `layer.rs:618` | `surface_fill.surfaces.iter().map(\|s\| s.expolygon.clone())` | 1:1 |
| `layer.rs:1623` | `surface_fill = regions[i].slices.clone()` | it *is* the region's slices (the local name misleads) |
| `SurfaceCollection::to_expolygons` | `surfaces.iter().map(\|s\| s.expolygon.clone())` | 1:1 |
| `SurfaceCollection::set` | `clear()` + `surfaces_append(...)` | 1:1 |

Nothing between `LayerRegion::slices` and Arachne changes the partitioning.
`Layer::make_slices` reads region slices that already exist, so **the origin is
the slicing / region-assignment stage** — earlier than anything this campaign has
instrumented.

### A retraction of my own handoff

R553's handoff asserted that our comment in `generate_classic_one` (`:502`),
quoting C++ as `union_ex(surface.expolygon.simplify_p(...))`, was "DECAYED/WRONG".
**That is incorrect and is retracted.** C++ has *two* constructions and the
difference is deliberate:

```cpp
PerimeterGenerator.cpp:945  (process_classic)  ExPolygons last = union_ex(surface.expolygon.simplify_p(res));
PerimeterGenerator.cpp:1511 (process_arachne)  ExPolygons last = offset_ex(surface.expolygon.simplify_p(res), -...);
```

The comment is right for the function it sits in. Only the *arachne* path lacked
the union in C++ while ours applied one — we had inherited the classic form.

### The fix (kept, parity-neutral)

`generate_arachne` now offsets the raw `simplify_p` polygons directly, matching
`:1511`, behind `ARACHNE_NO_PRE_UNION` (default ON; gate OFF reproduces
`4f9de6fe` byte for byte).

| | before | after | C++ |
|---|---|---|---|
| `last` contours | 12,637 | 12,791 | 15,003 |
| `last` holes | 38 | 40 | 782 |
| object material | 0.9973 | 0.9973 | 1.0 |
| wall-lines IoU | 95.28% | 95.28% | — |
| silhouette | 99.53% | 99.53% | — |

**Parity-neutral, exactly as predicted.** Kept under R550's rule: it is correct by
construction and removes a latent divergence on any fixture where the union would
actually merge contours. Verdict remains SEMANTICALLY EQUIVALENT; Majora
re-baselines **`4f9de6fe` -> `79cb7bd6`**; benchy, cube and the STL fixtures are
unchanged.

### R555

The surface partitioning is decided before `LayerRegion::slices` is ever read.
Instrument `LayerRegion::make_perimeters` entry on **both** engines (C++ receives
`const SurfaceCollection &slices` directly) for surfaces-per-region-per-layer, and
walk back into `PrintObject::slice()` / region assignment / mm-segmentation.
Layer count is equal at 657, so the 1.5x is surfaces *within* a layer-region.
Adding `LayerRegion.cpp` to the injector keeps the revert one command.

**New discipline (R554): a claim you wrote in your own handoff is not evidence.**
R553 told R554 that a source comment was wrong; checking the C++ took one grep and
showed the comment was right and the handoff was wrong. Queued assertions inherit
no more authority than the round that wrote them (R540), **including assertions
about your own code**.

## R555 — the fill rule is faithful; the split is inside each region's SurfaceCollection

No behavioural change; Majora stays at `79cb7bd6`.

### Suspect (ii) eliminated by reading the constant

R554's handoff nominated the Clipper fill rule as the prime suspect — even-odd
versus non-zero is exactly the switch that turns one contour-with-hole into two
hole-free contours. Ours (`triangle_mesh_slicer.rs:1861-1864`):

```rust
SlicingMode::EvenOdd => ClipperPolyFillType::EvenOdd,
SlicingMode::PositiveLargestContour => ClipperPolyFillType::Positive,
_ => ClipperPolyFillType::NonZero,
```

against C++ `TriangleMeshSlicer.cpp:2032-2033` — the same three-way mapping, and
both engines default to non-zero for this fixture. `clipper_utils.rs` uses
non-zero throughout. **Eliminated.**

### `MPPROBE`: what `make_perimeters` actually receives

A new probe at `LayerRegion::make_perimeters` entry on both engines (the C++ side
takes `const SurfaceCollection &slices` directly, so this is the first genuinely
like-for-like comparison of the collection itself):

| per call | Rust | C++ | ratio |
|---|---|---|---|
| **calls (layer-regions)** | **3,400** | **3,200** | **1.06x — essentially equal** |
| surfaces | 7.78 | 4.97 | **1.56x more** |
| holes | 0.036 | 0.534 | **14.8x fewer** |
| points | 398 | 479 | 0.83x — we have *fewer* |

Two things this settles:

1. **It is not extra regions or extra layers.** The call count matches to 6%.
   Every earlier framing left open whether we were splitting regions (Majora is
   multicolour, so mm-segmentation was a live suspect); we are not. The
   difference is entirely *within* each region's `SurfaceCollection`.
2. **Hole-loss is not the main effect.** Converting all 1,709 of C++'s holes into
   separate contours would add ~1,586 surfaces; our excess is **10,528**. Hole
   loss accounts for about **15%** of it. These are two distinct defects, and a
   single fix is unlikely to close both — worth knowing before chasing one.

Total points being *lower* while surfaces are 1.66x higher rules out "our
polygons are finer": the same area is being carved into more, smaller, simpler
pieces.

### R556

The collection is wrong before `make_perimeters` sees it, and `restore_untyped_slices`
merely copies `raw_slices` into `slices` (R554: 1:1). So instrument
`LayerRegion::raw_slices` at the moment slicing fills it — C++
`PrintObject::slice()` -> `slice_volumes()` -> the per-region assignment loop —
and compare surfaces/holes/points there. If `raw_slices` already shows 1.56x, the
defect is in slicing proper; if it does not, it is in whatever writes `slices`
between slicing and perimeter generation. Add the relevant C++ file to the
injector (now covering `LayerRegion.cpp`, `PerimeterGenerator.cpp` and the three
Arachne files) so the revert stays one command.

**New discipline (R555): when a ratio has two candidate mechanisms, do the
arithmetic on each before picking one.** Hole-to-contour conversion was the
elegant single explanation for all three symptoms; multiplying it out showed it
covers 15% of the surface excess. One subtraction, done before the next round is
planned, prevents a round spent fixing 15% of a problem and reporting it as the
cause.

## R556 — found it: our MM segmentation omits both of C++'s cleanup filters

Pure investigation; no code changed, Majora stays at `79cb7bd6`.

### Walking back from `make_perimeters`

`raw_slices` turned out to be a *backup* of `slices` (`layer.rs:1862`
`region.raw_slices = region.slices.clone()`), not its source — so the R555
handoff's plan to instrument it was aimed one hop wrong. Following the writes
instead: `region.slices` is populated during slicing, and for this fixture the
last thing to touch it is `apply_mm_segmentation_tier1`
(`print_object.rs:597-947`), which its own doc-comment identifies as the Tier-1
shape of C++ `apply_mm_segmentation` (`PrintObjectSlice.cpp:845-925`).

### The C++ block we are missing

`PrintObjectSlice.cpp:946-965`, inside that exact range:

```cpp
// Filter out unprintable polygons produced by subtraction ... Therefore, subtraction from
// layerm.region() could produce a huge number of small unprintable regions for the model's
// base extruder. This could, on some models, produce bulges with the model's base color.
if (! mine.empty())
    mine = opening(union_ex(mine), float(scale_(5 * EPSILON)), float(scale_(5 * EPSILON)));
...
        dst.expolygons = union_ex(mine);            // or append + needs_merge = true
...
// Re-create Surfaces of LayerRegions.
if (src.needs_merge)
    src.expolygons = closing_ex(src.expolygons, float(scale_(10 * EPSILON)));
layer->get_region(region_id)->slices.set(std::move(src.expolygons), stInternal);
```

Two cleanups: an **opening** on the remainder after the painted areas are
subtracted, and a **closing** when several regions merged into one.

A structural scan of our 351-line `apply_mm_segmentation_tier1` finds **zero**
occurrences of `opening`, `closing`, `needs_merge` or `union_ex`. It performs the
raw `intersection` (`:896`, the steal) and the raw `difference` (`:938`, the
remainder) — precisely the two operations C++ wraps.

**The C++ comment describes our symptom in its own words.** R555 measured 10,528
excess surfaces, each smaller than C++'s and nearly hole-free, at unchanged total
area; the C++ source says this subtraction "could produce a huge number of small
unprintable regions" and adds an opening specifically to remove them. The
`opening(5*EPSILON)` targets defect (A) — the ~85% of the excess that hole-loss
could not explain (R555).

This is the best-evidenced lead of the campaign: a named C++ operation, absent
from our port, whose stated purpose is the exact defect measured.

### R557 — implement and A/B

Port both filters into `apply_mm_segmentation_tier1` behind **separate**
default-ON gates (R500 — never bundle): `MMSEG_OPENING` for the
`opening(union_ex(mine), 5*EPSILON, 5*EPSILON)` on the remainder, and
`MMSEG_CLOSING` for the `closing_ex(..., 10*EPSILON)` merge path. Gate OFF must
reproduce `79cb7bd6` byte for byte. Then re-run `MPPROBE` and check whether
surfaces/call falls from 7.78 toward C++'s 4.97 — **and predict first**: if the
opening alone accounts for defect (A) it should remove most of the 10,528 excess
while leaving the hole deficit untouched. `opening`/`closing` helpers already
exist in `print_object.rs:1502-1519` (`closing_p`/`opening_p`) and in
`clipper_utils`.

**New discipline (R556): when a handoff names the next probe point, confirm the
data actually flows through it before building the probe.** R555 queued
`raw_slices` as the upstream target; two greps showed it is a downstream *copy*,
and following the writes instead landed on the real site in the same round. R554
said a handoff claim is not evidence — this extends it: a handoff's *plan* is not
evidence either.

## R557 — the filters work, the theory does not: a major retraction

Both C++ filters are now ported. They do what C++ says they do. **They do not fix
the metric this campaign has been chasing, and the causal chain built across
R551-R556 is refuted.** Majora stays at `79cb7bd6` — the gates ship opt-in.

### Units, read not assumed

C++ `EPSILON = 1e-4`, `SCALED_EPSILON = scale_(EPSILON) = 10`; ours identical
(`libslic3r.rs:24,29`). But the crate has **two** `SCALING_FACTOR` constants —
`lib.rs:451` = `100_000.0` and `libslic3r.rs:19` = `0.00001` — and `crate::`
resolves to the former. `offset_expolygons`/`opening_ex` take **mm**, so C++'s
`scale_(5 * EPSILON)` is simply `5 * EPSILON` here. Two reciprocal constants of
the same name is a live trap for any future port (R487/R549).

### What the filters did

| per layer-region | baseline | `MMSEG_OPENING` | C++ |
|---|---|---|---|
| **calls** | 3,400 | **3,200** | **3,200 (exact)** |
| **surfaces** | 7.78 | **4.40** | 4.97 |
| holes | 0.036 | 0.0138 | 0.534 |
| points | 398 | 397 | 479 |

The opening closes the surface-count gap outright — from 1.56x too many to
slightly *fewer* than C++ — and the `make_perimeters` call count lands exactly on
C++'s 3,200. The port is doing its job.

`MMSEG_CLOSING` is **byte-identical to gate-off**: our Tier-1 writes each region
once, so C++'s `needs_merge` is never true. Predicted from the structure before
running it, and confirmed. It is kept for correctness under `PAINTED_REGION_DEDUP`,
where two painted extruders *can* resolve to one region.

### The retraction

**`; LINE_WIDTH:` per outer-wall block: 1.19 -> 1.19.** Unchanged. Blocks with >1
distinct width 12.8% -> 12.9%. Within-block spread 0.0737 -> 0.0741 mm.

R551 identified the 2x ExtrusionLine count as the quantity driving the 3.5x width
gap; R552-R556 traced that count back to the region surfaces and found this
missing filter. **Closing the surface gap moved the width metric by zero.** The
surface-count excess was therefore **not** the cause of the width gap — a chain I
asserted with increasing confidence over five rounds is wrong, and every "-> "
in it that implied causation should be read as correlation.

Two further corrections to my own predictions this round: I said the opening
would leave holes roughly untouched — it *halved* them (0.036 -> 0.0138), moving
the hole metric further from C++, not nearer. And parity is marginally **worse**:
wall-lines IoU 95.28% -> 95.22%, silhouette 99.53% -> 99.52%, object material
0.9973 -> 0.9972. Measured against three independent C++ references
(`cpp_majora_new`, `cpp_r547_a`, `cpp_r547_post`) the post-fix numbers are
identical to four digits, so **the regression is real, not reference noise**
(R547's control, applied to the decision rather than the finding).

### Why they ship opt-in

The port is faithful and demonstrably improves the internal geometry. But the
standing rule is to revert what is not cleanly toward parity, the hypothesised
benefit is zero, and the cost is a real 0.06pp. So the code, its C++ line
references and its measurements are kept behind `MMSEG_OPENING` / `MMSEG_CLOSING`
(both opt-in, default OFF; default reproduces `79cb7bd6`), rather than deleted or
shipped. **That a faithful port of a C++ operation makes our G-code metrics
slightly worse is itself an unexplained result and the most interesting thing
this round produced** — it means something downstream is compensating for the
surface excess, and removing the excess without removing the compensation costs
accuracy.

### R558

Do **not** continue up the slicing chain — R557 removed the reason to. The width
gap needs a fresh bracket: with surfaces/call now matchable to C++ on demand
(`MMSEG_OPENING=1`), re-run `STAGEPROBE` and the ExtrusionLine count **with the
gate on** and see whether the 2.00x line-count gap moves at all. If it does not,
then line count and width-change frequency are independent, and the width
investigation must restart from the emission side (`; LINE_WIDTH:` is written per
`ExtrusionPath` in the exporter — count paths per feature-block on both engines).

**New discipline (R557): a fix that closes the internal gap but not the output
gap refutes the chain that connected them.** Five rounds of localisation each
confirmed a real difference and inferred causation from adjacency. The gate that
finally isolated the variable showed the link was never there. **When a chain is
built by walking upstream, every link is correlation until one of them is
switched off independently** — build the gate earlier.

## R558 — the width gap is 2x too few ExtrusionLines, from `generate_toolpaths`

Re-bracketed from the emission end, as R557 required. One real defect fixed, one
prediction wrong, two more links of the R551-R556 chain dead, and the origin of
the width gap finally pinned to a single function. Majora re-baselines
`79cb7bd6` -> `319de38e`.

### The emitter is faithful; the metric is honest

C++ emits the tag at `GCode.cpp:6605`:

    if (last_was_wipe_tower || m_last_width != path.width) {
        m_last_width = path.width;
        sprintf(buf, ";%s%g\n", ...ETags::Width..., m_last_width);
    }

It is **deduplicated against the previous path's width**, and `m_last_width`
persists across feature blocks. So tags-per-block counts *width changes*, never
paths — my stated prediction that the two would coincide was wrong by
construction. We have three emitters; the C++-shaped one (`exporter.rs:1278`,
per-path from `path.width`) is behind `LINEWIDTH_PERPATH`, which is a
`faithful_gate` and therefore **default-ON**. Its "PARKED behind its own gate"
comment has been stale since R225. We emit 78,333 distinct widths — genuinely
per-path. **The metric has been measuring geometry all along, not emission.**

### The real defect: `arachne_line_to_extrusion_paths` averaged the widths away

C++'s no-overhang-split branch (`PerimeterGenerator.cpp:772`) is one call:

    extrusion_paths_append(paths, *extrusion, role, flow);

the `ExtrusionLine` overload (`ExtrusionLine.cpp:301-305`) —
`to_thick_polyline` -> `thick_polyline_to_multi_path`, one path per width change.
Ours built **one** path carrying `avg_width`. `extrusion_paths_append_line` is a
faithful port of that overload written in R412 with a note saying "R413 wires it";
R413 never did, and its definition was **the only occurrence in the crate** —
dead code, exactly as `extrusion_paths_append_zpaths` was until R541. Now wired
behind `ARACHNE_LINE_VARIABLE_WIDTH` (default-ON; gate OFF reproduces `79cb7bd6`
byte-for-byte). The gate covers the width split only: the adjacent
`points.reverse()` has no C++ counterpart but fires only for open lines, so it is
preserved rather than bundled (R500).

Parity, confirmed against two independent C++ references (R557's control):
object material 0.9973 -> **0.9974**, object-only 0.9996 -> **0.9998**,
wall-lines IoU 95.28% -> 95.29%, silhouette 99.53% unchanged. Small, real,
positive; correct by construction (R550). **Kept default-ON.**

### But it barely moved the metric — and that is the finding

Outer-wall `; LINE_WIDTH:` per block **1.188 -> 1.332** against C++'s 4.210. I
predicted ~4. **Wrong.** The reason is visible one probe away: the fall-through
population is only ~2-4k loops. Most loops take the *overhang-split* branch that
R541 already fixed. No builder can emit width changes its input does not contain.

`LINEPROBE` (new, fall-through only) and `ARACHWIDTH` (split branch, the real
population) versus the new C++ `JWPROBE` at the identical point
(`PerimeterGenerator.cpp:692`, where `subject_path` is built):

| per Arachne loop | Rust | C++ |
|---|---|---|
| flat (min==max) | **84.4%** | **72.1%** |
| distinct widths / loop | 1.45 | 1.84 |
| mean spread | 20.0 um | 31.6 um |
| junctions / loop | 43.1 | 36.5 |

C++ is less flat, but only **1.27x** — against a **3.2x** output gap. R551's scale
test again. The loop *census* does not explain it either.

### Where it does come from: `STAGEPROBE`, stage 0

| stage | Rust | C++ | ratio |
|---|---|---|---|
| **0 after generate_toolpaths** | **58,529** | **130,536** | **2.23x** |
| 1 after stitch_tool_paths | 54,040 | 111,639 | 2.07x |
| 3 after separate_out_inner_contour | 40,000 | 80,002 | 2.00x |
| 5 after remove_empty_tool_paths | 40,000 | 80,002 | 2.00x |

**The 2x is present in the very first output of `generate_toolpaths`**, before any
post-processing stage runs — consistent with R544/R547 having eliminated all five
post-processing stages. Doing the arithmetic (R555): 2.00x fewer lines x 1.20x
fewer distinct widths each = **2.41x fewer width values**, the same order as the
3.23x outer-wall tag gap. Neither factor alone was; the product is.

### R557's gate, used as an instrument

Step 1 of this round was to re-run `STAGEPROBE` with `MMSEG_OPENING=1`. Stage-5
lines: **40,000 — identical to baseline** (stage 0 58,479 vs 58,529, 0.09%). So
**closing the surface-count gap does not move the ExtrusionLine count either**,
and `distinct_w/line` actually falls 2.16 -> 2.01. Two further links of the
R551-R556 chain — surfaces -> lines, and lines -> widths via surfaces — are dead
on direct experiment, not inference. R557's opt-in gate earned its keep as a
*measuring instrument* even though it ships off.

### R559

The target is now a single function: **`generate_toolpaths` emits 58,529
ExtrusionLines where C++ emits 130,536.** Everything downstream is exonerated by
stage-0 parity of the ratio. Do not re-audit the five post-processing stages
(R544/R547), the beading strategies (R542-R546), transitions (R551), or
`updateIsCentral` (R549, fixed). Instrument *inside* `generate_toolpaths` —
per-cell/per-edge line emission — and find where C++ produces two lines to our
one. Note the junction totals scale the same way (5.96M vs 11.90M), so this is
whole lines, not finer sampling of the same lines.

**New discipline (R558): when a faithful fix lands and the metric does not move,
measure the POPULATION the fix applies to before concluding anything.** Wiring
`extrusion_paths_append_line` was correct and moved the metric 5%, because the
branch it fixed carries ~10% of the loops. One probe on the branch would have
sized that in advance and framed the round correctly from the start. **Size the
population, not just the prize (R519 extended).**

## R559 — R558's 2x is largely a counting artifact; one wrong-comment fix; one unported feature

Instrumented inside `generate_toolpaths` as planned. The round produced a real
faithfulness fix, a **correction to R558's own headline**, and a concrete
unported feature. Majora re-baselines `319de38e` -> `d219a37e`.

### ExtrusionLines scale exactly with graph edges

`GRAPHPROBE` at the head of `generateSegments`, both engines:

| | Rust (25,800 calls) | C++ (41,000 calls) | ratio |
|---|---|---|---|
| `generateSegments` **calls** | 25,800 | 41,000 | **1.59x** |
| nodes | 3,546,183 | 7,881,298 | 2.22x |
| **edges** | **7,040,686** | **15,683,004** | **2.228x** |
| edges per call | 272.9 | 382.5 | 1.40x |

Total edge ratio **2.228** against R558's stage-0 line ratio **2.230** — equal to
three digits. Emission per edge is faithful; the line count is the graph. The
`addToolpathSegment` arithmetic agrees independently: it appends one junction or
starts a line with two, so calls = juncs - lines, giving 5,905,015 (rust) vs
11,772,437 (C++) with near-identical new-line rates (0.99% vs 1.11%).

### CORRECTION to R558: the 2x is mostly a probe-scoping artifact

`stageprobe` accumulates over **every** `WallToolPaths::generate()` call. C++ runs
that up to **twice per surface** (`PerimeterGenerator.cpp:1653-1719`): `:1654`
builds a complete `WallToolPaths` with `inset_count = 1` **purely to decide**
`should_enable_top_one_wall`, and then either `:1688`, `:1705` or `:1713` does the
real work. **C++'s 130,536 stage-0 lines therefore include a pass whose toolpaths
can be discarded outright.** Ours runs once per surface (~26k calls == our 26,452
surfaces).

So "C++ produces 2x the ExtrusionLines" is **not** a statement about wall
geometry. The geometry that reaches G-code was already near parity and still is:
outer-wall feature blocks 14,538 vs 14,864 (0.98x), extrude moves per block 36.2
vs 42.0. R558's stage-0 numbers are correct as measured; the *inference* drawn
from them — that C++ generates twice the walls — is withdrawn. The honest
per-call ratio is 1.40x, and part of even that is the differing `inset_count`
between the detection pass and the real pass. **R507 again: check what a metric
scopes — and a cumulative counter scopes whatever the caller loops over.**

### The fix: we ran a filter C++ never runs

`WallToolPaths.cpp:634` is `wall_maker.generateToolpaths(toolpaths);` — one
argument. The header:

    SkeletalTrapezoidation.hpp:135
    void generateToolpaths(std::vector<VariableWidthLines> &generated_toolpaths,
                           bool filter_outermost_central_edges = false);

so `filterOuterCentral()` **never runs in BambuStudio**. We passed `true` — on the
authority of a comment sitting at our call site that read *"generateToolpaths
defaults filter_outermost_central_edges = true (SkeletalTrapezoidation.hpp)"*.
The comment was simply wrong, and the code followed it (R490 — read the constant,
do not inherit a claim about it). `filterOuterCentral` clears `isCentral` on every
edge with no `prev`, and its twin.

Fixed behind `ARACHNE_NO_FILTER_OUTER_CENTRAL` (default-ON; gate OFF reproduces
`319de38e` byte-for-byte). **Parity-neutral to four digits** — object material
0.9974, object-only 0.9998, wall-lines IoU 95.29%, silhouette 99.53%, Top surface
1.172, all unchanged. Kept default-ON because it is correct by construction
(R550), not because it moved a metric.

**I predicted it would lift edges/call from 272.9 toward C++'s 382.5 and stage-0
lines from 58,529 to ~82,000. It did neither: 7,040,686 -> 7,044,606 edges
(+0.06%) and 58,529 -> 58,525 lines.** Edges with no `prev` are a tiny population
in a closed Voronoi skeleton. Second round running that a prediction about
magnitude was wrong while the direction of the fix was right.

### The unported feature this exposed

C++'s `seperate_wall_generation` block (`PerimeterGenerator.cpp:1620-1719`, plus
`should_enable_top_one_wall` at `:1894-1916`, ~100 lines total) implements *"only
generate one wall around top areas"*: keep the one-wall toolpaths for detected top
regions, then run a second `WallToolPaths` at `perimeter_spacing` for the
remaining walls with `inset_idx += 1`. Gated by
`is_one_wall`/`generate_one_wall_by_top` and the `top_area_threshold` config.

**Our `generate_arachne` does not implement any of it.** Every
`seperate_wall_generation` / `top_one_wall` match in `perimeter_generator.rs` is
at line 51-1092 — all inside the CLASSIC path; `generate_arachne` starts at
:2937. So the feature is ported for classic and absent for Arachne, which is the
path Majora actually uses.

That makes it a genuine functional gap, and it lands next to our worst
per-feature number: **Top surface material 1.172** — we deposit 17% more there
than C++, and this feature is precisely what thins C++'s walls on top areas.
Sized, not yet ported (R524/R529).

### R560

Port `seperate_wall_generation` + `should_enable_top_one_wall` into
`generate_arachne`, behind its own gate. **Predict first and check the premise
(R540):** measure what fraction of Majora's surfaces would take the top-one-wall
branch before writing the port — if `should_enable_top_one_wall` returns false
almost everywhere, the prize is small regardless of how real the gap is. The Top
surface 1.172 connection is a hypothesis, not a measurement.

**New discipline (R559): a cumulative probe measures the caller's loop, not the
callee's behaviour.** R558 compared two sums without asking how many times each
side was summed, and concluded C++ generates twice the walls. It calls the
generator twice as often, once speculatively. **Before comparing two totals,
count the calls that produced them.**

## R560 — the top-one-wall feature is near-inert: 4 surfaces in 16,500

Step 1 was to measure the premise before porting ~100 lines. It killed the port,
confirmed R559's artifact explanation by direct measurement, and corrected an
overstatement of mine from R559. **No Rust source changed; all three baselines
hold by construction** (majora `d219a37e`, benchy `5a34af50`, cube `ab415621`),
8 guards green.

### The measurement

New `TOWPROBE` in the injector counts every sub-condition of
`seperate_wall_generation` separately (R536), and counts the post-detection value
too, since `should_enable_top_one_wall` can flip it back to false **after** the
speculative generate has already run:

    [CPP-TOWPROBE] surfaces=16505 | loop_number==0=0 by_first_layer=0
    by_top_most=1 | is_one_wall=1 | by_top=16504 | seperate_PRE=16504
    detect_runs=16500 seperate_POST=4 (0.0% of detects survive)

Two things fall out.

**1. R559's artifact explanation is confirmed, not merely inferred.** The
speculative one-wall `WallToolPaths::generate()` runs on essentially every
surface — 16,500 detection passes alongside 16,505 real ones. That IS the ~2x
`generateSegments` call count R559 attributed to it, now measured directly rather
than deduced from reading the control flow.

**2. The feature itself is near-inert on Majora: 4 surfaces out of 16,500 keep
`seperate_wall_generation` after detection — 0.024%.** `by_top_most` and
`by_first_layer` fire on 1 surface and 0 surfaces respectively. So the whole
top-one-wall family touches ~5 of 16,505 surfaces.

**The Top-surface-1.172 hypothesis is REFUTED.** A feature that changes the walls
on five surfaces cannot produce a 17% material difference across the model. The
~100-line port is **not justified** and is closed, not deferred. This is exactly
what R540/R519/R524 exist to catch, and it cost one probe instead of a port.

### Correction to R559

R559 said our `generate_arachne` "implements NONE of it". **Too strong.** The grep
behind that claim searched only for `seperate_wall_generation` /
`should_enable_top_one_wall` / `top_one_wall`, which indeed appear nowhere in
`generate_arachne` — but `is_one_wall` and both of C++'s real branches DO exist
there (`perimeter_generator.rs:3036`, `:3070` one-wall, `:3083` normal), and
`normal_paths` is constructed with `(loop_number + 1)` at `:3087`, matching
`PerimeterGenerator.cpp:1713` exactly. What is missing is only the **detection
block**. Since detection survives on 0.024% of surfaces, **we are behaviourally
equivalent to C++ here on ~99.97% of surfaces.** Absence of a search term is not
absence of the behaviour (R539 in reverse).

### One measurement started and deliberately NOT concluded

`POLYPROBE` on both engines, at the same printed call index (48,000):

| stage | Rust polys / points | C++ polys / points |
|---|---|---|
| 0 outline | 23,439 / 1,123,721 | 44,168 / 2,196,736 |
| 3 final prepared_outline | 22,869 / 683,220 | 41,147 / 1,365,864 |

That reads as 1.88x polygons and 1.96x points — which would explain the per-call
graph size, and hence the line count, and possibly the width-value count. **I am
not concluding it.** Both logs cap at 48,000 (12 prints at a 4,000 modulo), so
these are not totals; and C++ makes ~2 `generate()` calls per surface, so its
first 48,000 calls cover ~24,000 surfaces against our 48,000. **Matched call index
is not matched population** — the identical trap R559 just caught. The number may
well be real; it is not yet measured soundly.

Incidental, and also hidden by cumulative probes: rust makes >=48,000 `generate()`
calls but only ~26,000 reach `generateSegments`, while C++ reaches it ~41,000
times. **Early returns inside `generate()` differ between the engines** and
nothing so far has counted them.

### R561

Re-run the outline census **normalised per surface, not per call**: give
`POLYPROBE` an end-of-run dump (or key it by surface id) and count `generate()`
early-returns on both sides as their own bucket (R536). Only then decide whether
the ~2x outline size is real. If it is, it supersedes every remaining
line-count/width-frequency lead, because the graph is a function of the outline.

**New discipline (R560): measuring the premise is not a formality — budget a
round for it.** The queued R560 port had a plausible mechanism, a named C++
block, a line count, and a matching symptom (Top surface 1.172). One probe showed
the branch fires on 0.024% of surfaces. **A hypothesis that survives reading can
still die on its first count; count before you port.**

## R561 — the 1.85x outline is real, and half our Arachne calls collapse to nothing

R560 refused to conclude the outline census because it was taken at a matched
call *index*. Normalised, **the gap survives** — my prediction that it would
shrink was wrong — and normalising exposed a second, larger divergence nobody had
counted. No behavioural change: baseline `d219a37e`, benchy `5a34af50`, cube
`ab415621`, 8 guards green.

### Making the census sound

Both probes printed on a 4,000 modulo, so both capped at 48,000 and neither ever
showed a total. Lowered to 200 on both engines, putting the last print within 200
calls of the true total.

The normalisation turned out simpler than R560 assumed: C++'s two `generate()`
calls per surface see the **same** outline, so points-per-call already equals the
mean per-surface outline size. R560's only real error was comparing at a call
index where the two engines had covered different surface *sets*. Whole-run
totals fix it outright — both engines process the entire model, so the totals are
comparable regardless of how the calls are indexed.

| | Rust | C++ | ratio |
|---|---|---|---|
| `generate()` calls | **51,200** | **50,200** | 1.02x |
| stage-0 outline points | 1,219,020 | 2,253,307 | **1.85x** |
| stage-0 outline polys | 26,649 | 46,384 | **1.74x** |
| reach `generateSegments` | 25,800 | 41,000 | 1.59x |
| **early returns** | **25,400 (49.6%)** | **9,200 (18.3%)** | **2.7x** |

**Prediction wrong.** I predicted the 1.96x would shrink toward R557's 1.20x
points-per-layer-region figure. It did not: 1.85x on whole-run totals. Fourth
magnitude prediction in five rounds to miss — the reading is fine, the sizing is
not, and I should stop attaching numbers to predictions I cannot derive.

### The thing I was not looking for

**Call counts are nearly EQUAL, not 2:1.** R560's model — C++ calls `generate()`
twice per surface, we call it once — is incomplete. Both engines re-enter
`generate()` for surfaces that fail, because the early return at
`WallToolPaths.cpp:486` / `wall_tool_paths.rs:990` fires **before**
`toolpaths_generated = true`, so the next accessor regenerates from scratch. Same
shape in both engines — a shared quirk, not a defect — but it means the probe
counts every failure twice on both sides.

That makes `POLYPROBE_calls - GRAPHPROBE_calls` exactly the early-return count,
since `polyprobe("3 final prepared_outline")` sits immediately before the
`area(prepared_outline) <= 0` check on both engines (verified, not assumed).
**We early-return on 49.6% of calls; C++ on 18.3%.**

Solving `calls = S + 2F` for successes S and failures F — derived, and stated as
derived:

| derived | Rust | C++ |
|---|---|---|
| surfaces reaching Arachne | 25,800 | 41,000 |
| surfaces collapsing to zero area | **12,700** | **4,600** |
| total surfaces | 38,500 | 45,600 |
| **failure rate** | **33.0%** | **10.1%** |
| points per surface | 31.7 | 49.4 |

**A third of the surfaces we hand Arachne collapse to zero area before the
skeleton is built. C++ loses a tenth.** This dovetails with R557, which found we
produce more and smaller surfaces per layer-region (7.78 vs 4.97): the extra
fragments are small enough that the `offset(-e) offset(+2e) offset(-e)` cleanup
plus `removeSmallAreas` annihilates them.

Note this makes R557's opening filter look different in hindsight. It moved
surfaces/region 7.78 -> 4.40 and was shipped opt-in because it cost 0.06pp of
IoU. It was aimed at the wrong metric, but it was pushing on the right quantity.
**Not reopening it on that basis** — that is a hypothesis, and R560 is a fresh
reminder of what those are worth. It needs the R562 measurement first.

### R562

The cleanest, largest, best-quantified divergence in the pipeline is now:
**33.0% vs 10.1% of Arachne inputs collapse to zero area.** Bucket the failures
before blaming anything: instrument the prepared-outline chain to record, for the
surfaces that fail, the area at stage 0 and which step zeroes it (the triple
offset, `simplify`, `removeDegenerateVerts`, `removeSmallAreas`, or the final
`union_`) — the stage counters exist, they just are not conditioned on outcome.
Then check whether `MMSEG_OPENING=1` (R557, opt-in) moves the failure rate: it
is a ready-made gate on exactly the surface fragmentation suspected of causing
this, and R557/R558 already established the habit of reusing it as an instrument.

**New discipline (R561): when a measurement is unsound, fixing the normalisation
often reveals a second quantity you were not measuring at all.** The call-count
equality was visible only once totals replaced sampled indices, and it is what
turned "C++'s outline is bigger" into "half our inputs are being thrown away".
**Repair the instrument before abandoning the question.**

## R562 — the failures are empty inputs, and R561's 1.85x was a denominator artifact

`AREAPROBE` (new, both engines) records the outline area after every step of the
preparation chain and attributes each failure to the step that zeroes it. It
answered the round's question, exonerated the chain, and **corrected R561's
headline number**. No behavioural change: `d219a37e`, `5a34af50`, `ab415621`,
8 guards green.

### The chain is not killing anything

Both engines at 50,000 calls:

| | Rust | C++ |
|---|---|---|
| survived | 24,641 | 41,043 |
| failed | 25,359 (50.7%) | 8,957 (17.9%) |
| **input_empty** | **24,704** | **7,064** |
| input non-empty, area <= 0 | 5 | 0 |
| first zero at `1 triple offset` | 562 (2.2%) | 1,259 (14.1%) |
| first zero at `2 simplify` | 6 (0.0%) | 41 (0.5%) |
| first zero at `8 removeSmallAreas` | 82 (0.3%) | 593 (6.6%) |

**97.4% of our failures arrive with zero area at the input** (C++ 78.9%). The
preparation chain does not kill them; they are dead on arrival. Direction
predicted correctly — input, not chain (R511 split answered).

The `input_nonempty_but_area<=0` bucket is 5 vs 0, so there is **no orientation
or holes-without-contour defect**. That was worth ruling out explicitly: a
negative signed area would have been a completely different bug.

Note the two steps where C++ loses *more* than we do — `1 triple offset` (14.1%
vs 2.2%) and `8 removeSmallAreas` (6.6% vs 0.3%). Those are proportions of each
engine's own failures, and C++'s denominator is 2.8x smaller; in absolute counts
C++ loses 1,259 and 593 surfaces there against our 562 and 82. We are not
under-filtering — we simply have far fewer live surfaces reaching those steps.

### CORRECTION to R561: the 1.85x does not survive conditioning

Condition both engines on a **non-empty** input — the only population where
outline size is meaningful:

| conditioned on non-empty input | Rust | C++ | ratio |
|---|---|---|---|
| calls | 25,296 | 42,936 | — |
| success rate | **97.4%** | **95.6%** | near parity |
| outline points per call | **48.2** | **52.5** | **1.09x** |

R561 reported 1.85x by dividing whole-run points by whole-run calls. **Half our
calls contribute zero points to that numerator**, so the mean was dragged down by
a population that has no outline at all. Conditioned properly the outline handed
to Arachne is **1.09x** — near parity — and the per-call success rate is near
parity too.

**This is the same error shape three rounds running.** R559 compared two sums
without counting the calls; R560 compared at a matched call index rather than a
matched surface set; R561 divided by a denominator whose composition differs
between the engines. Each time the fix revealed the previous number was an
artifact. **The invariant I keep breaking: an average is only comparable when
both sides average over the same KIND of thing, not merely the same COUNT of
things.**

### What actually survives

One divergence, cleanly isolated: **we invoke `WallToolPaths::generate()` on an
empty outline 24,704 times in 50,000 (49.4%); C++ does it 7,064 times (14.1%)** —
3.5x. Since failures re-enter `generate()` twice (R561), that is ~12,350 empty
surfaces against C++'s ~3,530.

**This is very likely a PERF lead, not a parity lead.** An empty outline yields no
walls in either engine, and the walls that reach G-code are already at parity
(outer-wall blocks 14,538 vs 14,864, 0.98x). The preparation chain over an empty
`Polygons` is also nearly free, so even the perf prize is probably small — it
should be measured, not assumed.

It does tie back to R557 arithmetically: our 7.78 surfaces per layer-region
against C++'s 4.97 shrinks to roughly 3.9 vs 4.3 effective once the empty
fragments are discounted. That is consistent with R557's opening filter having
been near-inert on output — it was removing things that were already producing
nothing.

### R563

Do **not** chase the empty-outline count as a parity lever until it is sized:
measure what fraction of `export_gcode`/`process` time the 25,400 wasted calls
actually consume (`SLICE_PHASE_TIMING=1`, and the profiler recipe). If it is
small, record it as a closed perf micro-lead and go back to the `; LINE_WIDTH:`
frequency gap (1.332 vs 4.210), which remains **the** open metric with no
surviving mechanism — every candidate from R538 onward has now been eliminated or
shown to be an artifact.

**New discipline (R562): before comparing two averages, ask what the denominator
is made of.** Same count is not same population. When one engine's calls include
a large inert class the other lacks, condition it out first — R559, R560 and R561
each lost a round to a version of this.

## R563 — the wasted calls cost 354 ms; ask #3 is 1.45x, not 1.12x

Two measurements, one closure, and a regression I had not been tracking. No
behavioural change: `d219a37e`, `5a34af50`, `ab415621`, 8 guards green.

### The empty-input calls: priced and closed

R562 found ~25,400 `generate()` calls arriving with an empty outline. Rather than
assume they were cheap, the chain is now timed and split by input emptiness
(inside the `AREAPROBE` gate, so it costs nothing by default):

    chain_ms  empty=354.2   nonempty=3322.9

**354 ms**, against a `Print::process` of 19.5 s — **1.8% of process, 1.4% of
total**. Direction predicted correctly (small). **Closed as a perf micro-lead with
the number attached**; no port, no fast-path. The preparation chain over an empty
`Polygons` really is nearly free, as expected, and the 3.3 s spent on non-empty
inputs is ordinary work, not waste.

### Ask #3 re-measured — and it has regressed

The recorded ~1.12x was stale (pre-R549). Interleaved, min of 3, load ~5.6:

| | Rust | C++ | ratio |
|---|---|---|---|
| wall clock (min of 3) | **25.50 s** | **17.60 s** | **1.448x** |
| instrumented `process` | 19.53 s | — | — |
| instrumented `export_gcode` | 5.16 s | — | — |

Rust's wall clock (25.50 s) against its instrumented total (19.53 + 5.16 =
24.72 s) leaves ~0.8 s of startup, so the two agree and the ratio is sound.
C++ emits no `SLICE_PHASE_TIMING` output at all — that env var is Rust-only, and
`cpptime.py` works by parsing timestamped log lines — so wall clock is the only
directly comparable measure on that side.

**We are 1.45x slower, not 1.12x.** Process alone has gone 11.4 s -> 19.5 s since
that figure was recorded. The most likely cause is the parity work itself: R549's
`cap` fix raised central edges 3.2% -> 11.8% (far more beads to process), and
R558 wired the variable-width builder (more paths per loop). **Both were correct
and both are keeping their parity gains — but they were never priced.** Caveat
worth stating: run-to-run spread was 28-30% at this load, so the absolute numbers
are soft; the ratio of minimums is the robust part.

This reopens ask #3 as genuinely unfinished rather than nearly-done, and it is
now the largest outstanding gap of the four asks.

### The open metric, conditioned properly at last

Using R562's surviving-call denominators, computed from existing logs — no new
runs:

| per SURVIVING call | Rust | C++ | ratio |
|---|---|---|---|
| ExtrusionLines | 1.623 | 1.949 | 1.20x |
| distinct widths per line | 2.09 | 2.60 | 1.24x |
| **width VALUES available** | **3.39** | **5.07** | **1.49x** |
| **`; LINE_WIDTH:` tags per outer-wall block** | **1.332** | **4.210** | **3.16x** |

**1.49x of supply cannot produce a 3.16x output gap.** R551's scale test fires
again, and this time on properly conditioned figures. So the width gap is **not**
a shortage of distinct width values — we have two-thirds of C++'s supply and emit
under a third of the tags.

What that leaves is the one property nobody has measured: the tag fires on
*change* against a persistent register (R558), so **consecutive repeats suppress
it**. Two engines with similar width supplies emit very different tag counts if
one orders its widths in longer runs. **This is a hypothesis, not a finding** —
but unlike every previous candidate it is directly testable from the G-code
without instrumenting either engine.

### R564

Measure the **run-length distribution of consecutive equal widths** within
outer-wall feature blocks, both engines, straight from the G-code. If our runs are
systematically longer, that is the mechanism and it points at path *ordering*
(`chain_and_reorder_extrusion_paths`, seam placement) rather than width
*generation* — a part of the pipeline this campaign has never examined. If the
run-lengths match, the metric is measuring something other than what its name
suggests and should be re-derived from scratch.

Separately: ask #3 now needs a profile, not a guess. Re-run the xctrace recipe
against the current binary and attribute the 19.5 s of `process` — the last
profile predates R549.

**New discipline (R563): a parity fix changes the performance budget, and nobody
re-prices it.** R549 and R558 were both correct, both kept, and together they
moved slicing time from 1.12x to 1.45x without a single round noticing, because
every round after them measured geometry. **When a fix adds work, measure the
work it adds in the same round you land it.**

## R564 — run-length refuted; the width gap is a heavy tail, decomposed exactly

The last untested mechanism is dead, and the metric turns out to be a tail
statistic that means something different from what its name suggests. Also: a
fresh profile for ask #3, and the profiling recipe was broken. No source changed
this round — baselines `d219a37e` / `5a34af50` / `ab415621` hold trivially.

### The run-length hypothesis is refuted

Distribution of extrusion moves between consecutive `; LINE_WIDTH:` tags inside
outer-wall blocks:

| | Rust | C++ |
|---|---|---|
| run length mean | **2.66** | **3.17** |
| p50 / p90 / p99 / max | 1 / 4 / 25 / 460 | 1 / 3 / 46 / 527 |

**Our runs are SHORTER, not longer.** Direction predicted wrong. Once a width
change has happened, we change again slightly *sooner* than C++ does. (This is a
conditional comparison — both sides conditioned on "a tag just fired" — so the
differing sample sizes, 15,931 vs 56,805, are the finding, not a flaw.)

### What the metric actually measures

| tags per outer-wall block | Rust | C++ |
|---|---|---|
| mean | 1.333 | 4.210 |
| **p50** | **0** | **0** |
| p90 / p99 / max | 4 / 19 / 71 | 15 / 44 / 723 |
| blocks with ZERO tags | 11,093 (76.3%) | 9,087 (61.1%) |

**The median block has no width change at all, in BOTH engines.** The mean is
driven entirely by a minority tail — C++ has blocks with hundreds of changes
(max 723) where ours top out at 71. "1.33 vs 4.21 tags per block" was never a
statement about typical walls, and twenty-odd rounds of treating it as one were
chasing an average over a bimodal population (R513, arriving late).

Decomposing it, and verifying arithmetically (R530):

| factor | Rust | C++ | ratio |
|---|---|---|---|
| blocks with any width variation | 23.7% | 38.9% | **1.64x** |
| tags per VARYING block | 5.62 | 10.83 | **1.93x** |
| **product** | | | **3.16x** |
| observed | 1.332 | 4.210 | **3.16x** |

Exact. And factor 1 cross-checks against internal data: `ARACHWIDTH` gives
loops-with-variation 15.6% vs 27.9% = **1.79x**, close to the 1.64x measured at
the output. **So half the gap is the beading — fewer of our loops carry any width
variation at all — and that half is corroborated internally.** Factor 2, tags per
varying block, is NOT explained by anything measured so far: `distinct_w/line` is
only 1.24x.

### CORRECTION to R563

R563 reported "width values available 3.39 vs 5.07 = 1.49x, which cannot produce
a 3.16x output gap". That compared supply **per surviving Arachne call** against
output **per outer-wall feature block** — two different denominators. Surviving
calls per block are 1.70 (rust) vs 2.76 (C++), so the denominators themselves
differ by 1.6x. **The comparison was invalid and its conclusion — "supply cannot
explain the gap" — is withdrawn.** The R562 lesson bit the very next round, in my
own arithmetic. Same-named quantity, different denominator, again.

### Ask #3: the profiling recipe was broken, and here is the profile

`xctrace` is **not reachable from inside devbox** — `/usr/bin/xctrace` is an
`xcrun` shim and the Xcode toolchain does not resolve there; both the bare and
absolute-path forms fail with `error: tool 'xctrace' not found`. **It must be run
OUTSIDE devbox** (`/Applications/Xcode.app/Contents/Developer/usr/bin/xctrace`).
The recipe carried in the handoff has been wrong for however long it went unused.

117,851 samples, 12 worker threads (main is only 4.9%; R521's thread caveat
matters). Global self time:

| symbol group | self |
|---|---|
| `__ulock_wait2` + `__ulock_wake` (lock contention) | **12.5%** |
| Clipper family (8 symbols) | **~20%** |
| `boostvoronoi ExtendedInt::mul_other` + `dif_slice` | **9.3%** |
| `__findenv_locked` (getenv) | **2.67%** |

Three concrete leads, none yet acted on:
1. **12.5% in thread synchronisation** is the single largest item. Twelve workers
   contending is a structural question, not a hot-loop one.
2. **9.3% in Voronoi exact arithmetic.** R520 eliminated "Arachne/Voronoi as the
   slicing-time gap" — but that predates R549, which tripled central edges. **That
   elimination should be treated as expired, not as settled** (R539: unported and
   eliminated claims both decay).
3. **2.67% in `getenv`** — `faithful_gate` called inside hot loops. "Caching
   `faithful_gate`" is on the eliminated list as a perf negative, but at 2.67%
   globally that measurement deserves re-deriving before it is trusted (R540).

### R565

Take the largest item first: **attribute the 12.5% lock contention**. Which locks?
`rayon` scheduling, an allocator arena, or a probe mutex left live? Slice the
profile by thread and find what the workers block on. Only then consider the
Voronoi and getenv items, in that order.

On the width metric: factor 1 is understood and corroborated; **factor 2 — why
C++ puts ~1.9x more width changes into the blocks that vary — is the remaining
open question.** Note both engines' medians are zero, so any future work here must
be stated over the varying sub-population, never as a global mean.

**New discipline (R564): when a mean is a tail statistic, say so before comparing
it.** `p50 = 0` on both sides means the average was never describing a typical
block. Report the median and the zero-fraction alongside any mean that drives a
campaign — this one drove roughly twenty-five rounds.

## R565 — `getenv` was 18.5% of the profile; slicing time 1.448x -> 1.033x

The largest item in R564's profile was not what it looked like, and the fix is
the biggest performance result of the campaign. **All three baselines are
byte-identical** (`d219a37e`, `5a34af50`, `ab415621`), 8 guards green — this
changes only how a decision is looked up, never the decision.

### It was not thread contention. It was `getenv`.

I predicted the 12.5% in `__ulock_wait2`/`__ulock_wake` would be rayon workers
parked while idle — i.e. unrecoverable. **Wrong.** Attributing each lock sample to
its nearest non-lock caller:

    total samples 117851;  __ulock_wait2/__ulock_wake leaves 14752 (12.52%)
       12.39%   14602  getenv
    6.44%  _os_unfair_lock_lock_slow   <- getenv <- std::sys::env::unix::getenv
    5.94%  _os_unfair_lock_unlock_slow <- getenv <- std::sys::env::unix::getenv

macOS `getenv` takes a **process-global `_os_unfair_lock`**. Twelve rayon workers
consulting gates inside the Arachne inner loops serialise on it. With the 2.67%
self time in `__findenv_locked`, **18.51% of all samples had an env lookup on the
stack.**

By call site — and this is the uncomfortable part:

| site | share | kind |
|---|---|---|
| `iscprobe` | 3.25% | **debug probe (R548)** |
| `connect_junctions` | 2.65% | gate |
| `generate_junctions` | 2.43% | gate |
| `gnbprobe` | 2.43% | **debug probe (R551)** |
| `intersection_pl` | 1.68% | gate |
| `update_is_central` | 1.01% | gate |
| `polyprobe` / `stageprobe` / `central_census` / `transition_census` | 1.68% | **debug probes** |

**Roughly 7.4% of all CPU was debug probes asking whether they were enabled and
being told no.** The instrumentation added across R543-R562 was assumed free when
off. On macOS, across twelve threads, it was the most expensive thing in the
program after Clipper.

### The fix

`env_snapshot()` — a `OnceLock<HashMap>` built once from `std::env::vars()` —
backs both `faithful_gate` and a new `probe_enabled`. The environment cannot
change during a slice run, so every gate returns exactly what it returned before;
after initialisation a lookup is a hash probe instead of a contended syscall.
83 call sites rewritten mechanically (76 `.is_some()` + 7 `.is_none()` — the
second pass mattered: the `.is_none()` form is what `ISCPROBE` and `GNBPROBE`
use, the two hottest probes of all).

### Result

| | before | after | |
|---|---|---|---|
| instrumented `process` | 19.53 s | **10.78 s** | **-44.8%** |
| instrumented `export_gcode` | 5.16 s | 4.81 s | -6.8% |
| instrumented total | 24.72 s | **15.59 s** | -36.9% |
| wall clock rust (min of 3) | 25.50 s | **16.04 s** | |
| wall clock C++ (min of 3) | 17.60 s | 15.53 s | (ambient) |
| **ratio** | **1.448x** | **1.033x** | |

Measured in two stages, and the arithmetic checks out (R530): after the first
pass rust was 18.44 s against C++ 15.67 s, a Rust-specific gain of ~17% once the
ambient improvement in C++ is subtracted — matching the ~15-18% of samples that
were env lookups. The second pass took `process` a further 12.94 -> 10.78 s.

**Ask #3 is effectively closed: 1.033x.** C++'s own times were stable across all
of today's runs (15.5-16.6 s), so the ratio is not a load artifact.

### What this says about the campaign

R563 blamed the 1.12x -> 1.45x regression on R549's `cap` fix and R558's
variable-width builder — "both correct, both keeping their parity gains, neither
ever priced". **That attribution was wrong.** Those fixes did add geometry work,
but the dominant cost was the *instrumentation used to find them*. Every probe
added to diagnose the width gap made the program slower in a way no round
measured, because probes are supposed to be free when disabled.

This also supersedes the entry "caching `faithful_gate` (perf NEGATIVE)" on the
eliminated list. That measurement predates the probe proliferation and the
12-thread contention it created. **An elimination is a measurement, and
measurements expire (R539/R540 applied in reverse).**

### R566

The width metric's factor 2 — why C++ puts ~1.9x more width changes into the
blocks that vary — remains the open parity question, and must be stated over the
varying sub-population, never as a global mean (R564).

On perf, re-profile before touching anything else: the remaining Clipper (~20%)
and Voronoi (~9.3%) shares were measured *with* the getenv contention inflating
wall time, so their true proportions have moved. At 1.033x the pressure is off,
and the honest next step is to confirm the new profile rather than chase items
sized against the old one.

**New discipline (R565): instrumentation is not free, and "off" does not mean
"absent".** Seven rounds of probes each added a per-call `getenv` inside the
hottest loops in the program; collectively they cost more than the algorithms
they were measuring, and the round that finally profiled found them at the top.
**Price the probe when you add it, and prefer a cached predicate to a syscall.**

## R566 — profile confirmed post-fix; the width metric is a RATE, and R564's split had a confound

Two results, both corrections of measurement rather than of code. No source
changed — baselines `d219a37e` / `5a34af50` / `ab415621` hold trivially.

### The R565 fix is confirmed in the profile, not just the clock

Re-recorded against the current binary (89.5 MB, 94,676 samples — down from
117,851, consistent with the faster run):

| | R564 | R566 |
|---|---|---|
| `__ulock_wait2`/`__ulock_wake` leaves | **12.52%** | **0.16%** |
| nearest non-lock caller | `getenv` (12.39%) | allocator internals only |

`getenv` is gone from the lock attribution entirely. **Prediction correct in both
parts** (direction only, per R563-R565): shares of everything else rose while
absolute work stayed put — Voronoi `mul_other` **9,194 -> 9,172 samples**
(7.80% -> 9.69%), Clipper `BuildIntersectList` 3,976 -> 3,899. Nothing got slower;
the denominator shrank (R530).

Honest post-fix profile: **Clipper ~24%, Voronoi ~11.5%, allocator ~6.3%.** At
1.033x there is no pressure to act on any of them, and Step 1 is closed by
confirming the numbers rather than by chasing them.

### Factor 2, and a confound in my own decomposition

Conditioning on varying blocks only:

| over VARYING blocks | Rust | C++ | ratio |
|---|---|---|---|
| tags per block | 5.62 | 10.83 | 1.93x |
| **moves per block** | **44.49** | **67.84** | **1.52x** |
| tags per move | 0.1264 | 0.1597 | 1.26x |
| distinct widths per block | 5.04 | 8.49 | 1.68x |
| (moves per ZERO-tag block) | 33.64 | 25.53 | 0.76x |

1.52 x 1.26 = 1.92 ≈ the observed 1.93x, and I predicted the direction correctly.
**But this decomposition is confounded.** A block with more moves is more likely
to contain at least one tag *by construction*, so "varying blocks are bigger" is
partly a selection artifact of how the sub-population was defined. R564's
`1.64x x 1.93x` split runs along the same confounded boundary. Both splits are
arithmetically exact; neither is safe to interpret.

Without conditioning on the outcome:

| | Rust | C++ | ratio |
|---|---|---|---|
| total extrude moves | 526,437 | 623,903 | 1.19x |
| moves per block | 36.21 | 41.97 | **1.16x** |
| **TAGS PER MOVE** | **0.0368** | **0.1003** | **2.73x** |
| product | | | **3.16x** |
| observed tags/block | 1.332 | 4.210 | **3.16x** |

Exact, and free of the selection effect. **The quantity that actually differs is
the width-change RATE per unit of extruded distance: 2.73x.** Block size
contributes only 1.16x. Everything the campaign has called "tags per block" is
better read as "how often the width changes per millimetre of wall".

That also reconciles R564's run-length result. Our tags are *clustered* — when a
change happens the next one is close (2.66 moves vs C++'s 3.17) — but they occur
across far less of the wall. **C++ distributes width changes; we concentrate
them.** Both facts are true and they are not in tension.

### R567

The open question is now sharply stated and denominator-safe: **why does C++
change extrusion width 2.73x more often per extruded move?** The internal
counterpart is `ARACHWIDTH` flat-loop fraction (84.4% vs 72.1%), which is only
1.79x on the "varies at all" axis — so a rate gap of 2.73x is still not covered,
and the residual must be *within* varying loops.

Measure the per-junction width sequence inside a single varying loop on both
engines — not how many distinct values it has (R558 did that, 1.24x) but how
often consecutive junctions differ. That is the direct internal analogue of
"tags per move" and it has never been measured; every prior census counted
distinct values or spread, which are insensitive to ordering.

**New discipline (R566): a sub-population defined by the outcome cannot be used
to explain the outcome.** "Varying blocks are bigger" looked like a finding and
is partly a tautology. **When splitting a ratio, check whether the split
criterion is downstream of the thing being measured — and prefer a decomposition
whose factors are both measurable without reference to the result.**

## R567 — the beading carries most of the rate gap; scope-matching moved 1.78x to 2.18x

`ARACHWIDTH` and `JWPROBE` both gained the ordering-sensitive quantity R566 asked
for. The answer is neither of Step 2's clean branches: **the gap is mostly
upstream, with a real downstream residual.** Baseline `d219a37e` unchanged
(probes are opt-in), 8 guards green.

### The measurement

`count(w[i] != w[i-1]) / (n_junctions - 1)` per loop — the direct internal
analogue of "tags per move". Distinct-value counts (R558, 1.24x) and spread
(R549, matched) are both blind to ordering and so could never speak to a rate.

| change rate | Rust | C++ | ratio |
|---|---|---|---|
| all loops | 0.0212 | 0.0376 | 1.78x |
| varying loops only | 0.0612 | 0.0786 | 1.28x |
| **outer wall only** | **0.0212** (11,177/526,155) | **0.0463** (25,606/553,138) | **2.18x** |

### The scope mismatch, caught mid-round

My first reading used the all-loops figure and concluded **"ordering adds
essentially nothing — a negative result for the hypothesis."** That was wrong, and
wrong for a specific reason: the 2.73x output rate is measured over **outer-wall**
feature blocks, while the internal rate covered **every** loop, inner walls
included. Comparing them is a scope mismatch (R507) — the same error class that
cost R559 through R566.

Restricting the internal probe to `inset_idx == 0` moved the ratio **1.78x ->
2.18x**. Against the output's 2.73x that leaves a residual of **1.25x** arising
below Arachne, not the 1.53x the mismatched comparison implied.

So Step 2 resolves to *both* branches, in proportion: **the beading accounts for
roughly 2.18 of the 2.73, and ~1.25x is created downstream.**

### A caveat worth recording

The two rates are not directly composable, and I am not going to pretend they are.
Outer-wall tags (19,363 rust / 62,582 C++) exceed outer-wall junction changes
(11,177 / 25,606) by 1.73x and 2.44x respectively — because the `; LINE_WIDTH:`
register persists **across** paths, loops and blocks (R558), so a tag also fires
when one loop's width differs from the previous loop's, which no per-junction
count can see. Both engines show this; it is not a defect. It does mean the
"1.25x residual" is a ratio-of-ratios, not a mechanism, and the residual could sit
in `thick_polyline_to_multi_path`'s `scaled(0.05)` merge, in inter-loop width
differences, or in both.

Also worth noting: outer-wall junction transitions are nearly equal between the
engines (526,155 vs 553,138 = 1.05x) while outer-wall extrude moves differ more
(526,437 vs 623,903 = 1.19x). Ours are almost exactly 1 move per junction
transition; C++ averages 1.13.

### R568

Two things, in order:

1. **Attribute the 1.25x residual.** Count, per engine, how many outer-wall tags
   fire at a path boundary where the *previous* path belonged to a different loop
   versus within a loop. That separates "inter-loop width differences" from
   "intra-loop splitting" and is measurable in the exporter without touching
   Arachne. Do not assume the merge tolerance is responsible — it is one of at
   least three candidates.
2. **Then the 2.18x itself**, which is now the larger share and squarely inside
   the beading: why does C++ assign a different width to adjacent junctions
   2.18x more often on outer walls? `BEADPROBE` already exists on both engines
   and reports per-`compute` width spread; extend it to report how often
   successive `compute` calls along one loop return different bead widths.

**New discipline (R567): match the SCOPE before comparing two rates, not just the
denominator.** R562 taught "same count is not same population"; this is its
sibling — same *unit* is not same *scope*. An all-loops internal rate and an
outer-wall output rate are both honest numbers that mean different things, and
the difference between them here was 1.78x versus 2.18x — enough to invert the
round's conclusion.

## R568 — the engines are INVERTED: our intra-loop width variation never reaches the G-code

Step 1 produced the strongest result of the width campaign, and it redirects the
investigation. No source changed — baselines `d219a37e` / `5a34af50` /
`ab415621` hold trivially.

### Where the tags actually fire

Classifying every outer-wall `; LINE_WIDTH:` tag by what preceded it — a TRAVEL
(new loop; the tag reflects a width difference against the *previous* loop, since
the register persists, R567) or an EXTRUDE (the path was split mid-loop). Pure
G-code, no instrumentation:

| outer-wall tags | Rust | C++ |
|---|---|---|
| after TRAVEL — **inter**-loop | **19,271 (99.5%)** | 280 (0.4%) |
| after EXTRUDE — **intra**-loop | **101 (0.5%)** | **59,242 (94.7%)** |
| at block start | 4 (0.0%) | 3,060 (4.9%) |

**The two engines are almost perfectly inverted.** We change extrusion width
essentially only at loop boundaries — 101 intra-loop changes in 19,376 tags.
C++ changes width *inside* loops 59,242 times.

### The smoking gun

Against R567's scope-matched internal counts:

| outer wall | Rust | C++ |
|---|---|---|
| junction width changes (internal) | 11,177 | 25,606 |
| intra-loop tags (G-code) | **101** | **59,242** |
| survival | **0.9%** | 231% |

**Our Arachne produces 11,177 intra-loop width changes on outer walls and 101 of
them reach the G-code — a 99.1% loss.** C++ turns 25,606 into 59,242 (>100%,
because paths split further than junctions alone imply). The beading gap R567
measured at 2.18x is real but largely *moot*: almost none of our intra-loop
variation survives to be emitted.

This also reframes the 2.73x rate gap qualitatively rather than quantitatively.
The two engines reach their tag counts by **entirely different routes** — ours
from inter-loop differences, C++'s from intra-loop splitting. "C++ changes width
2.73x more often per mm" is better stated as **"C++ has intra-loop width variation
in its G-code and we have essentially none."**

### What is NOT yet established

`clip_extrusion` is fine: it flattens z, calls the shim and reads z back, and
`clip_extrusion_interpolates_z` covers it. So the loss is below it, in
`to_thick_polyline_z` -> `thick_polyline_to_multi_path`.

The obvious suspect is that function's `scaled(0.05)` = 50 um merge tolerance
against our 21 um mean per-loop spread — every intra-loop difference would merge.
**But that cannot be the whole story: C++'s mean spread is 31 um, also well under
50 um, and C++ splits anyway.** Either its effective split condition differs, or
the mean hides a tail (R564 — this campaign has been caught by exactly that).
**Naming the tolerance now would be a guess (R490/R555), and it is one of the
three candidates R568 was told not to assume.**

### Step 2 deferred, deliberately

R568 also queued extending `BEADPROBE` to measure the successive-`compute`
width-change rate behind the 2.18x. **Not done, and the reason is Step 1's
result:** if 99.1% of our intra-loop variation is destroyed downstream, measuring
*why the beading produces slightly less of it* is measuring the wrong end of the
pipeline. The beading question stays open and is still the larger internal ratio,
but it is no longer the binding constraint.

### R569

Read the split condition on both sides and **evaluate it on real data** (R504):
C++ `VariableWidth.cpp` `thick_polyline_to_multi_path` versus ours, specifically
what is compared against `tolerance`/`merge_tolerance` and in what units. Then
instrument the loop that decides to split, on both engines, and count splits per
loop directly. Only after that consider changing anything — and gate it, because
a fix here changes G-code by construction.

Also worth one cheap check: the per-loop spread **distribution**, not its mean.
21 um vs 31 um are means over a population where 83.8% / 72.3% of loops are
perfectly flat; the varying tail is what the tolerance actually sees.

**New discipline (R568): when two engines produce the same output quantity by
different routes, the aggregate ratio is the least informative thing about it.**
"2.73x more width changes per mm" survived four rounds of decomposition while
concealing that ours are 99.5% inter-loop and C++'s 94.7% intra-loop — a
qualitative difference no ratio can express. **Classify the events before
counting them.**

## R569 — `thick_polyline_to_multi_path` is EXONERATED, and R568's headline was wrong

Baseline byte-identical (`d219a37e`), all 8 guards green, submodule reverted.
Both probes are gated and default-OFF.

### Step 1 — the split condition, read on both sides

C++ `VariableWidth.cpp:5-90` takes **two** thresholds that play **different roles**:

* `tolerance` gates **subdividing a line**: `fabs(line.a_width - line.b_width) > tolerance` (:28), scaled units.
* `merge_tolerance` gates **splitting the path**: `scaled(fabs(path.width - new_flow.width())) <= merge_tolerance` (:75-76).

All four Arachne entry points (`ExtrusionLine.cpp:284-311`) pass
`scaled<float>(0.05)` **and** `float(SCALED_EPSILON)` — and
`SCALED_EPSILON = scale_(1e-4) = 10`, i.e. **0.0001 mm**.

**So the merge tolerance is 0.1 um, not 50 um.** R568's suspect — "the
`scaled(0.05)` merge tolerance swallows our 21 um spread" — is **dead**: 0.05 mm
is the *subdivision* threshold, and I had conflated the two. Our callers
(`extrusion_line.rs:628/655/716/737`) pass the same pair,
`crate::libslic3r::SCALED_EPSILON = 10.0` and `scaled_f(0.05) = 5000` (`scaled_f`
uses `crate::SCALING_FACTOR = 1e5`, matching C++ `scaled<double>` = `/1e-5`).
The merge body is faithful line-for-line. `to_thick_polyline` and
`to_thick_polyline_z` are faithful. **A faithful condition with matching
constants cannot behave differently on identical input.**

### Step 2 — instrumented, and the function is clean

New `TPMPPROBE` on **both** engines (Rust `variable_width.rs`; C++ via the
injector, which now carries `VariableWidth.cpp`). Scoped to the outer wall.
Both runs capped at exactly 200,000 calls, so these are directly comparable:

| at 200,000 calls | Rust | C++ | ratio |
|---|---|---|---|
| width points in | 1,430,046 | 1,547,672 | 1.08x |
| **input width changes** | **13,888** | **39,330** | **2.83x** |
| distinct widths in | 209,048 | 231,463 | 1.11x |
| flat calls | 195,096 (97.5%) | 182,111 (91.1%) | — |
| output paths | 211,761 | 236,696 | — |
| **extra paths = intra-loop splits** | **11,761** | **36,696** | **3.12x** |

**Splits per input width change: 0.847 (Rust) vs 0.933 (C++).** The function
converts variation into splits at essentially the same rate on both engines.
**`thick_polyline_to_multi_path` is EXONERATED** — it faithfully passes through
whatever variation it is given, and it is given 2.83x less.

### R568's "99.1% loss" was WRONG — and this measurement is how it died

R568 concluded our Arachne makes 11,177 intra-loop width changes and 101 reach
the G-code. But the builder demonstrably emits **11,761 extra paths** at 200k
calls. Those paths are not being destroyed inside it.

A second probe, `EXPWPROBE`, at the per-path emitter (`exporter.rs`, the
default-ON `LINEWIDTH_PERPATH` site) shows **200,000 outer-wall paths reaching
export with 8,810 width-register changes and zero zero-width paths**. So the
paths **survive in count** all the way to the writer. What does not survive is
their **adjacency**: R568 measured that only 101 of the file's tags follow an
extrude, so ~98% of the register changes that do fire follow a **travel**.

**Restated open question:** our split paths reach the G-code, but almost never
*contiguously within a loop*. Either they are reordered so each is preceded by a
travel, or the loop is fragmented before emission.

### R570

`shortest_path.rs::chain_and_reorder_extrusion_paths` — flagged in the code map
as "path ORDERING, still never examined" — is now the leading candidate, because
it is the one stage between the multipath builder and the writer that can change
path adjacency. **It is a candidate, not a conclusion** (R560). Measure first:
count, per outer-wall loop at export, how many of its paths are emitted
contiguously versus separated by a travel, on both engines. The 2.83x input gap
is a separate, still-open question and belongs to the beading (R567/R568), which
this round did not touch.

**New discipline (R569): when a threshold has two names in the same signature,
read which comparison each one guards before blaming either.** `tolerance` and
`merge_tolerance` differ by a factor of 500 and gate completely different
branches; I carried "the merge tolerance is 50 um" for a full round, and it was
never the merge tolerance at all. **The constant you can name is not the constant
the branch uses.**

## R570 — fragmentation and adjacency are BOTH dead; the output gap equals the input gap

Baseline byte-identical (`d219a37e`), guards green, no C++ changes this round.
Probes stay gated and default-OFF.

### Prediction, and it was WRONG

I predicted our outer-wall blocks would contain **more** extrude runs (more
travels) than C++, since that was the only way R568's "tags follow travels"
pattern could arise from fragmentation. New `$D/runsblk.py` (pure G-code):

| outer wall | Rust | C++ |
|---|---|---|
| blocks | 14,538 | 14,864 |
| extrudes | 526,443 | 623,886 |
| travels | 192,580 -> **181,051** | 192,580 |
| runs (contiguous extrude sequences) | 149,770 | 165,942 |
| **runs/block** | **10.302** | **11.164** |
| travels/block | 12.454 | 12.956 |
| run length p10/p50/p90/p99/max | 1 / 1 / 8 / 39 / 281 | 1 / 1 / 9 / 43 / 957 |
| **tags/run** | **0.1294** | **0.3771** |

We have **fewer** runs and **fewer** travels per block, and the run-length
distributions are nearly identical. **Fragmentation is not the mechanism** — it
would need a ~200x effect and the measurement is 0.92x in the wrong direction.

### Adjacency is dead too

`EXPWPROBE` extended to test contiguity internally: a path is contiguous when it
starts exactly where the previous one ended, so no travel was needed. At 200,000
outer-wall paths:

    outer_paths=200000 width_changed=8810 contiguous=189500
    ch_contig=6988 ch_after_travel=1822 zero_width=0

**94.75% of our outer-wall paths are contiguous with their predecessor**, and
79% of the width changes land on a contiguous path. Our split partners are *not*
being separated by travels, and they are *not* being reordered apart. Both
mechanisms R570 was queued to test are refuted.

**Caveat, stated because it matters (R567):** `ch_contig`/`ch_after_travel` are
**not** directly comparable to R568's G-code buckets. The probe keeps its own
outer-wall-only register, whereas the writer's `last_width_tag` is global across
all roles — an infill path between two outer-wall paths moves the writer's
register but not the probe's. The two counts answer different questions, and the
residual tension between "6,988 contiguous internal changes" and "101 intra-run
tags in the file" is **not resolved**; it is a scope artefact of at least one of
the two registers and R571 should settle which.

### What the round actually establishes

The per-path emitter is the source of essentially every tag (88,406 written by
the time 200k outer-wall paths had passed; 153,524 in the finished file versus
215,199 for C++). And the decisive arithmetic:

| quantity | Rust | C++ | ratio |
|---|---|---|---|
| input width changes per builder call (R569) | 6.94% | 19.67% | **2.83x** |
| output tags per extrude run (R570) | 12.94% | 37.71% | **2.91x** |

**2.83x in, 2.91x out.** Within the precision of two different denominators
(builder calls vs G-code runs — not identical populations, so this is an
agreement of *ratios*, not an identity), **everything downstream of the beading
is proportional.** No stage between the beading and the writer amplifies or
attenuates the gap.

### R571 — the width campaign reduces to ONE question

Fourteen rounds of downstream candidates are now exhausted. The whole
`; LINE_WIDTH:` metric is explained by a single upstream fact: **we feed 2.83x
less width variation into the extrusion builder**, i.e. 97.5% of our outer-wall
loops arrive perfectly flat versus 91.1% for C++.

That is the beading question, and it is where R567 left it (2.18x per-junction
change rate on outer walls) and where R568 deferred it. **Do the deferred
`BEADPROBE` extension**: how often do successive `BeadingStrategy::compute` calls
along one loop return different bead widths, on both engines. That is now the
only live lead, not one of several.

**New discipline (R570): two hypotheses can share a symptom and both be wrong.**
"Tags follow travels" was read as evidence for fragmentation *and* for
reordering; measuring each directly killed both in one round, and the truth was
that the symptom never needed a downstream mechanism at all. **When a downstream
story requires a 200x effect, measure its size before believing any version of
it.**

## R571 — we produce MORE width variety than C++, not less; and R570's tension is settled

Baseline byte-identical (`d219a37e`), 8/8 guards, submodule reverted. Both probe
extensions gated and default-OFF.

### Prediction, and it was WRONG in a useful way

I predicted our `total_thickness` feeding the beadings would be **flatter** than
C++'s — that the flatness originates upstream in the skeleton rather than in
`compute`. New `JUNCPROBE` at the junction-creation site on **both** engines
(`skeletal_trapezoidation.rs:3095` / `SkeletalTrapezoidation.cpp:1847`), counting
DISTINCT values (order-independent, so safe under rayon — R559). Matched at
**n = 6,400,000 junctions**:

| at 6.4M junctions | Rust | C++ | ratio |
|---|---|---|---|
| outer-wall junctions (`idx0`) | 2,457,960 | 2,750,778 | 1.12x |
| **distinct widths** | **28,001** | **15,634** | **0.56x** |
| distinct thicknesses | 280,546 | 193,541 | 0.69x |
| distinct (thickness,width) pairs | 567,714 | 393,853 | 0.69x |
| width/thickness collapse | 10.02x | 12.38x | — |

**We produce 1.79x MORE distinct width values than C++, and 1.45x more distinct
thicknesses.** Our thicknesses are not flat and our widths are not impoverished.

### This retires the framing the last four rounds were built on

Every round since R567 has been looking for the place where our width variation
is *lost*. There is no such place, and there is no deficit of variation to lose:
we generate **more** distinct widths than the reference. What differs is **where
that variation is spent** — ours between loops, C++'s within them:

* R569: 97.5% of our loops arrive at the builder perfectly flat, vs 91.1%. So
  non-flat loops are 2.5% vs 8.9% — a **3.6x** gap in *which loops carry
  variation*, while the total variety we generate is larger.
* R568: 99.5% of our outer-wall tags follow a travel (loop boundaries); 94.7% of
  C++'s follow an extrude (mid-loop).

So the same quantity of width diversity is distributed differently: we vary
**loop-to-loop**, C++ varies **junction-to-junction within a loop**. No stage
destroys anything — R568's "loss", R570's fragmentation and adjacency, and now
"insufficient beading variety" are all dead.

### R570's register tension: SETTLED, it was a probe artefact

`EXPWPROBE` now also keeps a **global (all-roles)** register beside its
outer-wall-only one:

    outer_paths=200000 width_changed=8810 contiguous=189500 ch_contig=6988
    ch_after_travel=1822 zero_width=0 EMITTED_ALLROLES=88406
    GLOBAL_paths=311196 GLOBAL_changed=88406

**`GLOBAL_changed` equals `EMITTED_ALLROLES` exactly (88,406).** The global
register reproduces the emitter perfectly, confirming the emitter tests a
cross-role register. The outer-wall-only 8,810 was never the comparable quantity
and the 6,988-vs-101 discrepancy is fully explained as a scope artefact — **no
new mechanism**, exactly as R570 suspected but could not then show.

### R572

The one remaining unmeasured quantity: **how many DISTINCT beading objects serve
the junctions of a single emitted loop.** If ours is ~1 and C++'s is several,
the difference is in which beading gets attached to each skeleton node along a
loop. Note this is *not* covered by prior eliminations: R546/R547 cleared the
propagation chain as a **porting defect**, and R551 cleared
`getOrCreateBeading`/`getNearestBeading` — but nobody has measured per-loop
beading diversity, which is a different question from whether the code is a
faithful translation.

**New discipline (R571): before hunting for where a quantity is lost, check that
you have less of it than the reference.** Four rounds searched for the stage
destroying our width variation. We had 1.79x more of it than C++ the whole time;
the deficit was never in the amount, only in its distribution. **"Fewer events in
the output" does not imply "less of the underlying quantity" — measure the
quantity itself before assuming a loss.**

## R572 — C++ feeds each emitted loop ~2x the junction density

Baseline byte-identical (`d219a37e`), 8/8 guards, submodule reverted. Probe
granularity tightened (modulo 200,000 -> 20,000) on both engines so the last
printed line is a total with <=20k error instead of a 200k-wide floor.

### Prediction, and it was RIGHT

I predicted C++ creates substantially more `ExtrusionJunction`s in total than we
do (>=1.5x). Totals from `JUNCPROBE`:

| totals (<=20k floor error) | Rust | C++ | ratio |
|---|---|---|---|
| all junctions created | 6,520,000 | 12,200,000 | **1.87x** |
| **outer-wall junctions (`idx0`)** | **2,538,701** | **5,549,046** | **2.19x** |

**C++ creates 2.19x more outer-wall junctions than we do.** Note this is the
direct measurement of a claim R558 made inferentially and had retired — it is now
established by counting, and at a larger ratio than R558 guessed.

### The density ratio, and what it does NOT say

Against R570's emitted outer-wall runs (149,770 vs 165,942), outer-wall junctions
per emitted run are **16.95 (Rust) vs 33.44 (C++) = 1.97x**. C++ feeds roughly
twice the junction density into each emitted loop.

But the emitted geometry does **not** scale with that supply — from R569, width
points per builder call are 7.15 vs 7.74, only **1.08x**. So C++ discards or
merges proportionally far more of its junctions than we do, and arrives at a loop
of nearly the same point count from twice the raw material.

**This does not by itself establish that the density causes the intra-loop width
variety, and I am not going to claim it does (R571).** Two readings survive the
data equally well:

1. A denser junction supply gives more distinct widths to distribute along a
   loop, so more survive the reduction as intra-loop changes.
2. The extra junctions are redundant duplicates that reduce away entirely, and
   the intra-loop variety comes from something else in the reduction step.

Distinguishing them requires junctions grouped **per emitted `ExtrusionLine`**,
which is what R572 Step 1 originally asked for and what the global counters
cannot answer.

### What was NOT done, and why

R572 Step 1 asked for **distinct beading objects per emitted loop**, keyed by
pointer identity. That is still unmeasured. The junction site is inside a
per-edge traversal, and many edges feed one `ExtrusionLine`; wiring a per-line
grouping key through that traversal on both engines is a larger change than this
round had room for, and the global distinct-value counters answered a cheaper
question first. **Stated plainly rather than quietly dropped** — it is the third
round this specific measurement has been deferred.

### R573

Do the per-line grouping properly, on both engines: thread a monotonically
increasing line-id (or use the `ExtrusionLine` address at assembly time) so each
junction can be attributed to the loop it ends up in, then report per loop:
distinct beading identities, distinct widths, and junction count. That single
table settles readings (1) and (2) above and closes the last open mechanism.

**New discipline (R572): a supply ratio and an output ratio measured on
different populations do not compose into a causal chain.** 2.19x junctions in
and 1.08x width points out is a real pair of measurements, but "therefore the
supply causes the variety" is a third claim needing its own evidence. **The
count you can get cheaply is rarely the count that closes the argument.**

## R573 — the gap is 1.42x at assembly and 2.83x at the builder: the stage between DOUBLES it

Baseline byte-identical (`d219a37e`), 8/8 guards, submodule reverted. New
`LINEPROBE2` on both engines, gated and default-OFF.

### Why beading identity per loop is not measurable without a struct change

R572 queued "distinct beading objects per emitted loop, keyed by pointer
identity". Tracing the assembly path settles that it cannot be done cheaply:
`generate_junctions` fills junctions **per skeleton edge**; `connect_junctions`
links edges; and **`add_toolpath_segment` receives only `from`/`to`
`ExtrusionJunction`s** — the beading is out of scope at every point where a line
is assembled. An `ExtrusionJunction` carries `p`, `w`, `perimeter_index` and the
hole flag, and nothing that identifies its beading. Measuring beading identity
per loop therefore requires **tagging `ExtrusionJunction` on both engines**, a
struct change, not a probe. That is a finding about the code, not another
deferral — and the equivalent per-loop quantity needs no struct change.

### The measurement: per-assembled-line width variety

`LINEPROBE2` runs at the end of `generateToolpaths` on both engines and walks
`generated_toolpaths`, counting per line: junctions, distinct widths,
consecutive width changes. This is the **earliest per-loop measurement possible**
— before any downstream reduction. Outer wall (`inset == 0`):

| outer wall @ assembly | Rust | C++ | ratio |
|---|---|---|---|
| assembled lines | 23,402 | 60,214 | 2.573x |
| **junctions per line** | **92.66** | **84.05** | **0.907** |
| distinct widths per line | 3.308 | 4.214 | 1.274x |
| flat-line fraction | 64.85% | 55.59% | 0.857 |
| **width changes per junction** | **0.0325** | **0.0462** | **1.423x** |

**Our assembled lines carry MORE junctions each (92.66 vs 84.05)** — a third
independent refutation of any "we have less raw material" reading. C++ makes
2.57x more, slightly shorter lines.

### The result that matters

The same quantity — width-change rate — measured at two stages:

| stage | Rust | C++ | ratio |
|---|---|---|---|
| **at assembly** (per junction) | 0.0325 | 0.0462 | **1.42x** |
| **at the builder** (per width point, R569) | 0.0694 | 0.1967 | **2.83x** |

**The gap is 1.42x when the line leaves Arachne and 2.83x when it reaches the
extrusion builder. The stage between them roughly doubles the discrepancy.** At
assembly the two engines are much closer than any downstream measurement
suggested; most of the divergence is introduced *after* the skeleton is done.

That stage is: WallToolPaths post-processing, plus `perimeter_generator`'s ZPath
construction and overhang splitting. Note our 23,402 assembled outer lines feed
**more than 200,000** outer-wall builder calls (the TPMPPROBE cap, R569), so each
assembled line is being cut into many pieces before it reaches the builder —
**but that cap is a floor, not a total, so the pieces-per-line ratio is not yet
established and I am not quoting one (R572).**

### R574

Measure the split factor properly: total outer-wall builder calls per assembled
outer line, on both engines, with the TPMPPROBE modulo tightened so both figures
are totals rather than floors. If we cut each line into ~3x more pieces than C++
does, the intra-loop variation is being redistributed into inter-loop variation
by the splitting itself — which is exactly the R568 inversion, and would be the
mechanism. **Prior eliminations do not cover this:** R544/R547/R558 cleared the
five WallToolPaths stages as *porting fidelity* against a flat-percentage metric
at a different scope; change-rate amplification across the stage is a new
quantity (R539 — eliminations expire).

**New discipline (R573): measure the same quantity at two stages before blaming
either.** Four rounds compared engines at one stage at a time and kept relocating
the mechanism. One rate measured at assembly *and* at the builder localises it in
a single round: 1.42x in, 2.83x out. **A ratio at one point tells you there is a
gap; the same ratio at two points tells you where it is made.**

## R574 — we split each assembled line into 1.88x more pieces; and R573's assembly numbers were sampling artefacts

Baseline byte-identical (`d219a37e`), 8/8 guards, submodule reverted. Both probes
tightened to report totals: `TPMPPROBE` modulo 50,000 -> 1,000 (floor error
<=1,000), `LINEPROBE2` now prints on **every** `generateToolpaths` call, so its
last line is exact.

### CORRECTION: R573's assembly figures were boundary-sampled and are WRONG

R573 read `LINEPROBE2` at a print gated on `lines % 20_000 < 200`. That fires at
whatever point each engine happens to cross a 20k boundary — **the two engines
were sampled at different points in their runs, so the numbers were not
comparable.** With exact totals:

| outer wall @ assembly | R573 (sampled) | R574 (exact) |
|---|---|---|
| junctions per line | 92.66 vs 84.05 — "**we carry MORE**" | **72.61 vs 79.80 — C++ carries 1.10x more** |
| width changes per junction | 0.0325 vs 0.0462 = **1.42x** | **0.0650 vs 0.0577 = 0.888x** |

**Both R573 claims are retracted.** The direction of the assembly comparison
reverses: at assembly we have **more** width changes per junction than C++, not
fewer, and C++ carries slightly more junctions per line. R571/R572's refutation
of "we have less raw material" still stands on its own evidence (1.79x more
distinct widths; 2.19x junction supply) — but R573's junctions-per-line figure
was never valid support for it.

### The measurement R574 was for

| totals | Rust | C++ | C/R |
|---|---|---|---|
| **builder calls per assembled outer line** | **6.366** | **3.388** | **0.532** |
| junctions per assembled line | 72.61 | 79.80 | 1.099 |
| width points per builder call | 7.219 | 7.947 | 1.101 |
| ASSEMBLY changes per junction | 0.0650 | 0.0577 | 0.888 |
| ASSEMBLY flat-line fraction | 57.26% | 54.02% | 0.943 |
| BUILDER changes per call | 6.70% | 19.72% | **2.942** |
| BUILDER flat-call fraction | 97.64% | 91.29% | 0.935 |

**We cut each assembled outer line into 6.37 builder pieces; C++ cuts it into
3.39. That is 1.88x more splitting.** Prediction was right in direction.

### What this establishes, and what it does not

At assembly the engines are **equal or slightly in our favour** — 57.3% vs 54.0%
flat lines, and we carry a *higher* width-change rate per junction. After the
splitting stage we are **2.94x worse** on changes per call and markedly flatter
(97.6% vs 91.3%). So the divergence is not attenuated across that stage, and it
is not merely doubled as R573 claimed: **it is created there, and it reverses
sign.** Smaller pieces are individually flatter, and the width variation ends up
at piece boundaries — which the G-code register reads as inter-loop changes,
exactly the R568 inversion.

**Not established:** which split. The 1.88x is the aggregate of every stage
between `generateToolpaths` and `extrusion_paths_append`. Note also that
junction counts are **not** preserved into the builder (72.61 junctions per line
becomes 6.37 calls x 7.22 width points), so both engines reduce heavily and the
supply/output populations differ — no causal chain may be composed from those two
numbers alone (R572).

### R575

Find which split produces the 1.88x. Instrument the candidates in order and count
pieces-per-input-line at each: (1) `perimeter_generator.rs`'s overhang/ZPath
`clip_extrusion` branch (~:3545) — it is the only stage that splits by geometry
rather than by width, and `clip_extrusion` returns `ZPaths` (plural) by
construction; (2) the WallToolPaths post-processing stages. Gate any change.

**New discipline (R574): a modulo print is a sampler, and two samplers on
different runs are not a comparison.** R573's headline survived a round because
`lines % 20_000 < 200` looked like a total. It fired wherever each engine crossed
a boundary, and the engines cross at different points. **Before comparing two
cumulative counters, confirm each one is a total — not merely the last thing
printed.**

## R575 — the split site is exonerated, and R574's "we split more" was causally misleading

Baseline byte-identical (`d219a37e`), 8/8 guards, submodule reverted. New
`SPLITPROBE` on both engines at the overhang/ZPath split
(`perimeter_generator.rs` ~:3740 / `PerimeterGenerator.cpp:703-730`), gated and
default-OFF.

### The site R574 nominated is NOT the splitter

| outer wall @ the ZPath split | Rust | C++ | C/R |
|---|---|---|---|
| pieces per line | 15.996 | 14.677 | 0.918 |
| **pieces per junction** | **0.3821** | **0.4237** | **1.109** |
| supported-branch pieces per line | 15.225 | 14.581 | 0.958 |
| overhang-branch pieces per line | 0.075 | 0.095 | 1.270 |

Per line we make 9% more pieces; **per junction C++ makes 11% more.** Either way
this is ~1.1x, not the 1.88x R574 was chasing. **The overhang/ZPath split —
including the `detect_overhang_degree` branch whose earlier miscalibration once
over-split the outer wall 2.3x (the note at :3726) — is exonerated.**

### CORRECTION: R574's ratio was driven by its denominator

R574 reported "we cut each assembled outer line into 6.37 builder pieces vs
C++'s 3.39 — 1.88x more splitting". Arithmetically correct, causally misleading:

| | Rust | C++ | C/R |
|---|---|---|---|
| builder calls (numerator) | 215,000 | 224,000 | **1.042** |
| assembled outer lines (denominator) | 33,772 | 66,108 | **1.957** |

**The numerators are nearly equal.** The 1.88x is almost entirely the
denominator: C++ assembles **1.96x more outer-wall `ExtrusionLine`s** for a
similar number of builder calls. "We split each line into more pieces" is a true
statement about the quotient and a false story about the mechanism — nothing is
doing extra splitting on our side.

### The tag gap decomposes exactly

With that corrected, the outer-wall `; LINE_WIDTH:` gap factors cleanly
(R530 check):

| factor | value |
|---|---|
| assembled-line count | 66,108 / 33,772 = **1.9575** |
| tags per assembled line | 0.9467 / 0.5737 = **1.6500** |
| product | **3.2299** |
| observed tag ratio (62,582 / 19,376) | **3.2299** |

Exact to four decimals. **The gap is the product of two independent factors,
each contributing roughly equally**, and neither is a downstream loss.

### R576

Attack the larger factor: **why does C++ assemble 1.96x more outer-wall
`ExtrusionLine`s?** This is consistent with R572's 2.19x outer-wall junction
supply and is now the dominant term. The lines are assembled in
`add_toolpath_segment` (`:3345-3385`), which starts a new line when
`force_new_path` is set or when the gap/width test against the previous
junction fails (`shorter_then(..., scaled(0.010))` and
`|w_prev - w| < scaled(0.010)`). **Instrument which of those three conditions
starts each new line, on both engines** — that is a direct per-branch count and
needs no struct change. The second factor (1.65x tags per line) stays open and is
separate.

**New discipline (R575): when a per-unit ratio is the headline, check the
numerator and denominator separately before naming a mechanism.** "We split
1.88x more" survived a round and sent this one to instrument the wrong site; the
numerators differed by 4%. **A quotient names a mechanism only if you know which
half of it moved.**

## R576 — the line-start conditions are faithful; the 2x comes from `empty` and `odd`

Baseline byte-identical (`d219a37e`), 8/8 guards, submodule reverted. New
`NEWLINEPROBE` on both engines in `add_toolpath_segment`, gated and default-OFF.

### Prediction, and it was WRONG

I predicted the **width test** would dominate our new-line starts more than
C++'s. It fires **zero times on both engines** — a clean shared negative that
removes one of the three candidates outright.

Every new outer-wall `ExtrusionLine`, attributed to its cause (outer wall only,
totals are floors at modulo 2,000):

| cause | Rust | % | C++ | % | C−R | share of gap |
|---|---|---|---|---|---|---|
| **`empty`** (vector empty) | 24,492 | 76.5% | 39,812 | 62.2% | **+15,320** | **47.9%** |
| **`odd`** (`is_odd` differs) | 6,002 | 18.8% | 19,290 | 30.1% | **+13,288** | **41.5%** |
| `gap` (10 µm distance test) | 1,401 | 4.4% | 3,599 | 5.6% | +2,198 | 6.9% |
| `caller` (`force_new_path` in) | 100 | 0.3% | 1,194 | 1.9% | +1,094 | 3.4% |
| `threeway` | 5 | 0.0% | 105 | 0.2% | +100 | 0.3% |
| `perim` (index differs) | 0 | — | 0 | — | 0 | 0.0% |
| **`width`** (10 µm width test) | **0** | — | **0** | — | **0** | **0.0%** |
| TOTAL | 32,000 | | 64,000 | | +32,000 | |

Deltas sum to 32,000 = the total gap, exactly (R530). New-line ratio **2.000**,
matching R574's independently-measured assembled-line ratio of 1.9575.

### What this settles

**The conditions themselves are faithful and are not the mechanism.** The width
test never fires on either engine; the perimeter-index test never fires on
either; the gap test accounts for 6.9% of the difference. What differs is the
*input stream* reaching `add_toolpath_segment`, via two roughly equal terms:

* **`empty` (+47.9%, ratio 1.63x)** — `generated_toolpaths[0]` is empty at the
  moment of the call, i.e. this is the first outer segment of a fresh
  `generateToolpaths` invocation. C++ has 1.63x more such invocations producing
  outer-wall content.
* **`odd` (+41.5%, ratio 3.21x)** — `back().is_odd != is_odd`. C++ alternates
  between odd (single-bead/central) and even walls at inset 0 **3.2x more
  often**, and each alternation forces a new line.

Both are upstream of this function. Neither is a defect in the assembly logic.

### R577

Two independent targets, in order of size:

1. **The `odd` alternation (3.21x).** Highest ratio and cheapest to check:
   `is_odd` comes from the caller's `is_odd` argument, set from whether the
   toolpath is a central/odd wall. Count odd-vs-even segments reaching
   `add_toolpath_segment` at inset 0 on both engines. Note R548/R549 touched
   `updateIsCentral` (and R549 fixed its `cap`), so this is adjacent to fixed
   ground — **re-derive, do not assume** (R539/R540).
2. **The `empty` term (1.63x)** — more `generateToolpaths` invocations carrying
   outer-wall content. Adjacent to the region/surface-count question that
   R557/R558 eliminated *as a width-metric cause*; this is a different quantity
   (invocations producing outer segments) and needs its own count.

The second factor of the tag gap (1.65x tags per assembled line) remains
separate and untouched.

**New discipline (R576): when a decision has several conditions, count them all
before assuming the interesting one matters.** Two of the five candidate
conditions here fire exactly zero times on both engines, and the one I predicted
would dominate was one of them. **A condition that never fires is worth
measuring precisely because it removes itself.**

## R577 — same odd-wall SHARE, 2.19x finer interleaving; and the `empty` term is fully explained

Baseline byte-identical (`d219a37e`), 8/8 guards, submodule reverted. New
`ODDPROBE` on both engines in `add_toolpath_segment`, gated and default-OFF.

### Prediction, and it was WRONG

I predicted C++ generates a higher **share** of odd segments. It does not — the
shares are essentially equal:

| outer wall @ `add_toolpath_segment` | Rust | C++ | C/R |
|---|---|---|---|
| segments (calls at inset 0) | 2,415,000 | 5,200,000 | **2.153** |
| odd segments | 72,038 | 171,905 | 2.386 |
| **odd SHARE** | **2.983%** | **3.306%** | **1.108** |
| alternations | 22,639 | 106,949 | 4.724 |
| **alternations per segment** | **0.00937** | **0.02057** | **2.194** |
| mean odd-run length (2·odd/alt) | 6.36 | 3.21 | 0.505 |

**C++ does not produce proportionally more odd walls — it interleaves the same
proportion twice as finely.** Our odd walls arrive in runs of ~6.4 segments;
C++'s in runs of ~3.2. Since every odd/even flip forces a new line, that halving
is what R576 saw as the `odd` cause.

### The `empty` term: explained exactly

`LINEPROBE2` prints once per `generateToolpaths` call, so its line count is a
direct invocation count:

| | Rust | C++ | C/R |
|---|---|---|---|
| `generateToolpaths` invocations | 25,876 | 41,188 | **1.592** |
| R576's `empty` new-line cause | 24,492 | 39,812 | 1.626 |

**1.592 vs 1.626** — the `empty` term is simply the invocation count, as
expected (each invocation contributes one empty-vector line start at inset 0).
No separate mechanism.

### Everything upstream now reduces to one product

| factor | ratio |
|---|---|
| `generateToolpaths` invocations | 1.592x |
| segments per invocation | 1.353x |
| **product** | **2.153x** |
| observed segment ratio | **2.153x** |

Exact (R530). And 2.153x segments is the same ~2.2x seen as the outer-wall
junction supply in R572 (2.19x) — **one underlying quantity, measured three
different ways across three rounds.**

### R578

The chain is now: **1.59x invocations x 1.35x segments-per-invocation -> 2.15x
segments -> 2.00x new lines -> 1.96x assembled lines -> (x 1.65x tags/line) ->
3.23x tags.** Every link is measured and the arithmetic closes at each step.

Attack the two remaining upstream terms:

1. **`generateToolpaths` invocations, 1.592x.** How many times is
   `WallToolPaths::generate` called, and on what? **R557/R558 eliminated the
   region SURFACE COUNT as a *width-metric* cause — this is the invocation
   count, a different quantity, and R539 says eliminations expire.** Count
   invocations per layer and per region on both engines.
2. **Segments per invocation, 1.353x.** Fewer skeleton segments per call.
   Adjacent to the graph-size questions (R547/R559 `GRAPHPROBE`), which measured
   edges, not emitted segments — re-derive.

**New discipline (R577): when a count and a rate both differ, normalise before
choosing which to chase.** The odd-segment *count* differs 2.39x, which looks
like a mechanism; the odd *share* differs 1.11x, which says the mechanism is
elsewhere. **The interesting quantity was the one the raw count was hiding.**

## R578 — NEW ACCEPTANCE BAR: line-level parity ("every line the same except floats")

User directive (2026-08-05): keep going until the code is very similar, slicing
time matches, and the output is **essentially the same line-for-line** — numbers
may differ in the last places, everything else should be identical. That is a far
stricter bar than `semantic_compare.py`, which only checks material, layer count
and swept-area IoU (both fixtures already PASS all five of those gates).

New tool: **`scripts/line_compare.py`**. Hierarchical alignment, because naive
alignment does not work here:

    level 1   '; CHANGE_LAYER'    -> layers
    level 2   '; FEATURE: <name>' -> feature blocks within a layer
    level 3   extrude runs (islands), matched by ANCHOR GEOMETRY not order
    level 4   windowed structural walk inside a matched island

A line pair "matches at tolerance t" when its structural key (the line with every
number replaced by `#`) is equal AND every numeric token agrees within t.

### Two alignment artefacts found while building it — both would have been reported as findings

1. **Structural key alone is not an alignment.** Nearly every extrude line has
   the key `G# X# Y# E#`, so a two-pointer walk drifts and then pairs unrelated
   lines. v1 reported 37.98% with "worst deviations" showing `I-1.217` vs
   `I1.217` — sign-flipped nonsense from mispairing, not a mirrored toolpath.
2. **The engines order a layer's islands differently.** After fixing (1), the
   worst pairs had identical X and E with negated Y: the port loop compared
   against the starboard one. Fixed by matching islands on anchor geometry.

**Neither number was a parity result.** Recorded because the same shape of error
has now cost this campaign three rounds (R573 sampler, R574 quotient, and this).

### First readings — LOWER BOUNDS

Unaligned does not mean different: the greedy island matcher leaves runs
unpaired, so these understate similarity. Quoted as floors.

| | Benchy (classic) | Majora (arachne, MM) |
|---|---|---|
| body lines rust / cpp | 129,547 / 130,669 | 2,166,427 / 2,445,983 |
| **line-count gap** | **0.9%** | **12.9%** |
| aligned pairs | 71,547 | 391,676 |
| exact text | 65.42% | 7.50% |
| **essentially identical (rel<=1e-4)** | **>=36.66%** | **>=2.20%** |
| outer wall, essentially identical | **83.6%** | **17.6%** |
| prime tower | n/a | 48.0% |

**Benchy is close to line-identical on the walls (83.6%) and within 0.9% on line
count. Majora is not** — 12.9% more lines in C++, outer wall 17.6%.

The line-count gap is the one figure here that is alignment-free and therefore
solid, and it splits the two fixtures cleanly: the classic perimeter path is
nearly line-exact; the Arachne + multi-material path is not.

### R579

Two threads, both now aimed at the new bar:

1. **Improve the matcher before trusting its absolute numbers** — replace greedy
   nearest-anchor with a proper assignment over islands, and report an explicit
   "unaligned because the tool could not pair" versus "unaligned because the line
   has no counterpart" split. Until then only the per-feature *relative* figures
   and the line-count gap should be quoted.
2. **Majora's 12.9% line-count gap** is the same ~2.2x segment-supply story the
   R572-R577 chain has been tracking, now visible directly in the output. The
   chain closes arithmetically (1.59x invocations x 1.35x segments/invocation ->
   2.15x segments -> 2.00x new lines -> 1.96x assembled lines -> x1.65 tags/line
   -> 3.23x tags); R578 adds the output-side confirmation.

**New discipline (R578): a new metric must be validated on the fixture you expect
to score WELL before it is trusted on the one you expect to score badly.** Benchy
scoring 83.6% on outer wall is what proved the matcher works at all; had I run
Majora first, 17.6% would have looked like a finding instead of a tool defect.

## R579 Thread A — the matcher is fixed and is no longer the limiter

`scripts/line_compare.py` rewritten in two places. No engine code touched.

1. **`islands()`** — consecutive non-extrude lines are now ONE run. Previously
   every travel/comment line became its own anchor-less run, flooding the
   matcher with unpairable singletons.
2. **`match_islands()`** — greedy nearest-anchor replaced by **mutual-nearest
   neighbour iterated to a fixed point**: a pair is accepted only when each run
   is the other's nearest available candidate. Greedy let one bad early match
   consume a partner and cascade.
3. **New diagnostic** — unpaired runs are split into *no counterpart* (the run
   counts genuinely differ) versus *the matcher failed to pair them*.

### The instrument now answers for itself

| | Benchy | Majora |
|---|---|---|
| aligned share of rust body | **61.0%** | 21.9% |
| unpaired runs | 21,350 | 819,048 |
| **of which NO counterpart** | **21,248 (100%)** | **813,478 (99%)** |
| matcher failures | **102** | **5,570** |

**Only 102 runs on Benchy and 5,570 on Majora are matcher failures.** The
unaligned bulk is genuine: the engines emit different numbers of extrusion runs
per feature block. R578's caveat ("quote only relative figures") can be lifted —
these absolute numbers now mean what they say.

### Corrected readings

**R578's denominators were undercounts** — lines inside unpaired runs were
partly dropped, so its body totals (Benchy 129,547, Majora 2,166,427) were too
small and its percentages correspondingly inflated. Corrected:

| | Benchy | Majora |
|---|---|---|
| rust body lines | 160,963 | 2,553,030 |
| aligned pairs | 98,249 | 558,281 |
| exact text | 64.43% | 14.97% |
| **essentially identical (rel<=1e-4)** | **40.38%** | **4.41%** |
| outer wall | **80.4%** | **26.2%** |
| inner wall | 69.1% | 19.4% |
| prime tower | n/a | 27.3% |
| floating vertical shell | — | 1.7% |

Outer wall on Benchy reads 80.4% against R578's 83.6% — *lower* with the better
matcher, because more and harder runs now align. That direction is expected and
is the honest number.

### What the numbers say

**Benchy: 40.4% of body lines are essentially identical, 64.4% of aligned pairs
are exact text.** The walls are the strong part (80.4%).

**Majora: 4.4%.** 2,219,401 of 2,553,030 rust body lines sit in runs with no
counterpart at all. That is the same divergence the R572-R577 chain measured
upstream (2.153x segments at inset 0), now seen end-to-end in the output.

`Floating vertical shell` at **1.7%** is the worst feature on either fixture and
has never been examined — it was cleared only as an *area* question (R539's
`VSHELL_DROP_FILTER` era), never line-for-line.

### R580

Thread B is untouched this round and stays queued: **`generateToolpaths`
invocations 1.592x** — count `WallToolPaths::generate` calls per layer and per
region on both engines. Then `Floating vertical shell` (1.7%), which is now the
worst per-feature line-level score and is cheap to look at.

**New discipline (R579): make a measuring instrument report its own failure
rate.** Two rounds were spent unsure whether "unaligned" meant "different" or
"the tool gave up". A counter that separates the two settled it in one run — and
it also caught that the previous denominator was wrong.

## R580 — the 1.592x invocation gap is SPECULATIVE WORK C++ throws away, and it contaminates the R572-R577 chain

Baseline byte-identical (`d219a37e`), 8/8 guards, submodule reverted. New
`WTPCALL` probe on both engines (gated, default-OFF).

### Prediction, and it was WRONG in a useful direction

I predicted the invocation gap would be roughly uniform across layers. The
per-layer split is not where the answer was. `WTPCALL` at the `WallToolPaths`
construction site:

| | Rust | C++ |
|---|---|---|
| constructions (surfaces reaching the site) | 26,000 | 16,000 |
| `onewall` branch taken | **0** | **15,999** |
| polygons in `last_p` | 13,909 | 15,764 |
| per-layer median / max | 42 / 80 | 25 / 420 |

C++ takes a separate-wall branch essentially always; we never do. But the branch
variable is **recomputed** — `seperate_wall_generation = !is_one_wall &&
generate_one_wall_by_top` at `:1696`, the block at `:1752` builds
`one_wall_paths` on that preliminary value, and `:1774` then *overwrites* it with
`should_enable_top_one_wall(...)`. So `WTPCALL` was reading the preliminary
predicate, not the branch that survives (R532 — read the emission expression,
not the name).

`TOWPROBE` gives the surviving rate directly:

    surfaces=16,504 | seperate_PRE=16,503 | detect_runs=16,500 | seperate_POST=4

**C++ speculatively builds a full one-wall `WallToolPaths` — a complete
`generateToolpaths` invocation, Voronoi and all — on 16,503 surfaces, and keeps
the result on 4 of them (0.024%).** That reproduces R560's 0.024% exactly, from a
different probe.

### Target 1 CLOSED: the gap is speculative, not structural

| | value |
|---|---|
| C++ invocations, total | 41,188 |
| of which speculative | 16,503 (kept: 4) |
| C++ invocations, real | **24,685** |
| Rust invocations | **25,876** |
| **real-vs-real ratio** | **1.048x** |

**The 1.592x is 1.048x once the discarded speculative pass is excluded.** There
is no invocation-count defect. R577's `empty` new-line term (1.626x), which R577
showed *is* the invocation count, is inert for the same reason.

### This contaminates several C++-side counts from R572-R577

The speculative `getToolPaths()` call runs the whole Arachne pipeline —
`generateJunctions`, `addToolpathSegment`, `connectJunctions` — so **every
internal C++ counter in this campaign includes work that never reaches the
G-code**:

| measurement | affected? |
|---|---|
| JUNCPROBE junctions 12.20M (R572) | **YES — includes speculative** |
| ODDPROBE segments 5.20M (R577) | **YES** |
| NEWLINEPROBE new lines 64,000 (R576) | **YES** |
| LINEPROBE2 assembled lines 66,108 (R574) | **YES** |
| tags per assembled line 1.6500x (R574) | **YES — derived from the above** |
| outer-wall `; LINE_WIDTH:` 62,582 (G-code) | no — measured in the output |
| body line count 2,445,983 (G-code) | no |
| `line_compare.py` figures (R578/R579) | no |

**The R572-R577 chain — 1.592x x 1.353x -> 2.153x -> 2.000x -> 1.9575x x 1.6500x
-> 3.2299x — is therefore built partly on inflated C++ internals.** Its arithmetic
closed at every step because the same inflation propagated consistently through
it, which is exactly why it looked sound. The *output-side* facts (3.23x tags,
12.9% body lines) stand; the internal attribution does not.

### R581

**Re-derive the chain with the speculative pass excluded.** The cleanest way is a
probe flag set while `one_wall_paths` is being built, so every downstream C++
counter can exclude speculative calls — `JUNCPROBE`, `ODDPROBE`, `NEWLINEPROBE`
and `LINEPROBE2` all need it. Until then, **no C++-side internal ratio from
R572-R577 should be quoted.**

Then re-open the real question with clean numbers: after excluding speculation,
how much of the 12.9% body-line gap and the 3.23x tag gap remains attributable
upstream?

**New discipline (R580): a reference implementation may do work it discards, and
your probes will count it.** Sixteen thousand full Arachne invocations produce
four used results. Every internal counter compared across the two engines for six
rounds included them. **Before comparing internals, establish that both engines'
work reaches the output.**

## R581 — chain re-derived with speculation excluded: the dominant factor INVERTS

Baseline byte-identical (`d219a37e`), 8/8 guards, submodule reverted. Injector
gains `probe_speculative()` (thread-local, defined in `SkeletalTrapezoidation.cpp`,
declared in `PerimeterGenerator.cpp`), set around the one-wall construction and
tested by `JUNCPROBE`, `ODDPROBE`, `NEWLINEPROBE` and `LINEPROBE2`.

Two anchor collisions had to be handled (R562): the includes string occurs in
both `ST_INCLUDES_OLD` and `_NEW` (positional replace), and the `nlp` line occurs
in `ST_NL_NEW`, `ST_ODD_OLD` and `ST_ODD_NEW` — all three are generated text and
must move together or the ordered edits stop matching.

### Every C++ internal count was inflated; several badly

| quantity | Rust | C++ clean | C/R | C++ old | old C/R |
|---|---|---|---|---|---|
| `generateToolpaths` invocations | 25,876 | 26,415 | **1.021** | 41,188 | 1.592 |
| junctions (JUNCPROBE) | 6,520,000 | 7,580,000 | **1.163** | 12,200,000 | 1.871 |
| segments at inset 0 | 2,415,000 | 2,875,000 | **1.190** | 5,200,000 | 2.153 |
| odd segments | 72,038 | 141,494 | 1.964 | 171,905 | 2.386 |
| alternations | 22,639 | 59,199 | 2.615 | 106,949 | 4.724 |
| new-line events | 32,000 | 40,000 | **1.250** | 64,000 | 2.000 |
| assembled outer lines | 33,772 | 41,668 | **1.234** | 66,108 | 1.957 |

**45% of C++'s counted segments and 38% of its junctions were speculative.**

### The chain still closes exactly — but the split inverts

| | assembled lines | x tags per line | = tags |
|---|---|---|---|
| **R574 (contaminated)** | 1.9575x | 1.6500x | 3.2299 |
| **R581 (clean)** | **1.2338x** | **2.6178x** | **3.2299** |
| observed | — | — | **3.2299** |

Both factorisations reproduce the observed 3.2299x to four decimals, which is
exactly why the contaminated one looked sound (R580). But they say opposite
things. **The dominant term is not the number of assembled lines (1.23x) — it is
how many width tags each line carries (2.62x).**

### Two R577 conclusions revisited

* **RETRACTED — "the odd SHARE is equal".** R577 measured 2.98% vs 3.31% and
  concluded the mechanism was purely interleaving. Clean: **2.98% vs 4.92% =
  1.650x.** C++ *does* emit proportionally more odd walls; the contamination hid
  it.
* **STANDS — alternations per segment.** 0.00937 vs 0.02059 = **2.197x**, against
  R577's 2.194x. Unchanged by the correction, so it is a real signal and now the
  strongest surviving upstream one.

### Where this leaves the campaign

Upstream supply is **near-parity**: invocations 1.02x, segments 1.19x, junctions
1.16x, assembled lines 1.23x. Six rounds of "C++ has roughly twice the material"
dissolve — that was the speculative pass throughout. What remains is
**intra-line**: 2.62x tags per assembled line, with 2.20x alternations per segment
and a 1.65x odd-wall share as the live upstream contributors.

### R582

The question is now sharply intra-line, which is where R567-R571 were looking
before the supply story displaced them — **but those rounds' C++ figures are
contaminated too** and must be re-derived before reuse: R571's "we generate 1.79x
MORE distinct widths" used JUNCPROBE distinct counts (now 31,239 clean vs 15,634
quoted at a matched n — the matched-n comparison itself needs redoing), and
R572's 2.19x junction supply is now 1.16x.

Re-run the R567-R572 measurements with `probe_speculative()` active, then attack
tags-per-assembled-line directly.

**New discipline (R581): when two different factorisations of the same product
both reproduce it exactly, the arithmetic cannot tell you which is right.**
1.9575 x 1.6500 and 1.2338 x 2.6178 both give 3.2299. Only knowing which inputs
were measured on comparable populations distinguishes them. **A closing identity
validates the algebra, never the measurement.**

## R582 — the reduction between assembly and the builder is width-BLIND on our side

Baseline byte-identical (`d219a37e`), 8/8 guards, submodule reverted. `TPMPPROBE`
is now speculation-gated too (all five internal C++ probes are).

### Step 1 — `TPMPPROBE` was NOT contaminated (honest negative)

Gating it changed nothing: `in_changes` 44,275 vs 44,181 ungated, `calls` 224,000
both, `out_paths` 266,657 vs 266,529 — differences within the C++ engine's own
run-to-run nondeterminism (R547). **Because `thick_polyline_to_multi_path` is
reached via `extrusion_paths_append` on the FINAL toolpaths, the speculative
one-wall result never passes through it.** R569's exoneration of the builder
therefore **stands unmodified**, and the gate is kept as a guarantee rather than
a fix.

### Step 2 — the 2.62x splits cleanly across two stages

| | Rust | C++ | C/R |
|---|---|---|---|
| changes per line **at assembly** | 4.7191 | 5.9571 | **1.262** |
| changes per line **at the builder** | 0.4268 | 1.0626 | **2.490** |
| tags per line **in the G-code** | 0.5737 | 1.5019 | **2.618** |

`2.6178 = 1.2623 (assembly) x 2.0738 (downstream)`. Assembly is a minor term;
**the stage between the assembled `ExtrusionLine` and the builder's
`ThickPolyline` roughly doubles the gap.**

### The mechanism, stated precisely

| | Rust | C++ | C/R |
|---|---|---|---|
| junctions per line at assembly | 72.61 | 70.21 | 0.967 |
| width points per line at builder | 45.96 | 42.76 | 0.930 |
| **point retention** (wpts/junction) | **0.6329** | **0.6090** | **0.962** |
| **change retention** | **0.0904** | **0.1784** | **1.972** |

**Both engines discard points at the same rate (0.962x). Only the width changes
diverge (1.972x).** We keep 9.0% of our assembly width-changes; C++ keeps 17.8%.
A width-neutral reduction would lose changes in proportion to points — ours does
not. **The reduction between assembly and the builder is effectively width-blind
on our side.**

Two independent routes agree on the size: **2.0738** (from G-code tags per line)
and **1.9722** (from change retention). Different measurements, same stage.

### R583

Instrument the reduction itself on both engines: at the ZPath construction in
`perimeter_generator.rs` (~:3545, subject built with `j.w` as z) count junctions
in, points out, **and width-changes in versus out**. The prime suspect is
collinear-vertex removal that is blind to the z channel — Clipper operates on XY
and carries z as an attribute, so a vertex dropped for being collinear in XY
takes its width change with it. **That is a hypothesis, not a finding** (R560):
`clip_extrusion` z-preservation was verified in R568 and its split in R575, but
neither measured *changes in versus out*, which is a different quantity (R539).

Also still queued: `Floating vertical shell` at 1.7% line-level.

**New discipline (R582): re-deriving a suspect measurement and finding it
unchanged is a result worth the round.** `TPMPPROBE` was flagged as contaminated
on the reasonable grounds that four sibling probes were. It was not — the
speculative path does not reach it. **Verifying a doubt costs one run; carrying it
costs every conclusion that touches it.**

## R583 — the width-change deficit exists at birth; every downstream stage is exonerated

R582 left a hypothesis: the reduction between the assembled `ExtrusionLine` and
the builder's `ThickPolyline` is width-blind on our side, most plausibly through
collinear-vertex removal that drops a vertex for being collinear in XY and takes
its width change with it. This round instrumented that reduction on both engines.

**The hypothesis is refuted at its own site.** `REDPROBE`, counting width changes
into and out of the ZPath clip in `perimeter_generator.rs`, gives
`pts 580,622 -> 976,036 (1.681x)` and `changes 11,496 -> 12,812 (1.115x)`. Both
*increase*: clipping splits one subject into several ZPaths with duplicated
endpoints. `clip_extrusion` does not destroy width changes.

**`STAGEPROBE` gained a change-density axis and a speculation gate.** The five
`WallToolPaths` post-processing stages were cleared on junction and line counts
(R544/R547/R558) but never on width changes, which is what R581's dominant
2.6178x tags-per-assembled-line term is made of. The probe now counts
`w[k] != w[k-1]` per line and breaks out inset 0. It also predated R581 and was
still counting the discarded speculative one-wall pass; it is now gated on
`probe_speculative()`, so **every pre-R583 STAGEPROBE number is contaminated.**

Matched on inset-0 line count (cpp 21,844 / rust 20,582):

| stage | R i0 ch/junc | C i0 ch/junc | C/R |
|---|---|---|---|
| 0 after generate_toolpaths | 0.03114 | 0.04290 | **1.378** |
| 1 after stitch_tool_paths | 0.03117 | 0.04301 | 1.380 |
| 2 after remove_small_lines | 0.03107 | 0.04289 | 1.380 |
| 3 after separate_out_inner_contour | 0.03107 | 0.04289 | 1.380 |
| 4 after simplify_tool_paths | 0.05845 | 0.07254 | **1.241** |
| 5 after remove_empty_tool_paths | 0.05845 | 0.07254 | 1.241 |

**The gap is fully present at stage 0 and it NARROWS downstream.** Through
post-processing we retain 0.3064 of inset-0 points and 0.5751 of inset-0 changes
(ratio 1.877); C++ retains 0.2726 and 0.4609 (ratio 1.691). Our reduction is not
width-blind — it is *more* width-preserving than C++'s, on the same axis that was
supposed to indict it. All-inset the gap runs 1.272x -> 1.160x, same direction.

Stage 0 is the output of `SkeletalTrapezoidation::generateToolpaths`, and its
1.378x independently reproduces `LINEPROBE2`'s 1.306x measured inside that
function by a different probe on the same population. Two instruments agree the
deficit is created at birth.

**Retracted mid-round.** A first reading put stage 0 at 2.493x. That compared
C++ at 60,002 stage-5 lines against Rust at 40,001 — R573's population-mismatch
error repeating. At matched population it is 1.378x. Only the matched figures
above stand.

**Not a finding.** Stage-5 inset-0 `ch/junc` (R 0.05845 / C 0.07254) does not
equal the ZPath subject's `ch/point` (R 0.01996 / C 0.04187), which would suggest
a further loss inside `PerimeterGenerator`. It does not: `subject_path` is built
only inside `if (detect_overhang_wall && layer_id > raft_layers)`
(`PerimeterGenerator.cpp:666`), so `REDPROBE`'s population is a strict subset of
`STAGEPROBE`'s. The two are not comparable and no loss is attributable. Noted so
a later round does not rediscover the artefact. `apply_fuzzy_skin` (`:663`) also
rewrites `*extrusion` unconditionally on that path and has never been checked for
width preservation — that remains open.

**Where this leaves the search.** The entire downstream chain is now eliminated on
the change-density axis: the clip, all five post-processing stages, and (R569)
the builder and the writer. The deficit is created inside `generateToolpaths`
itself, consistent with the strongest surviving upstream signal, R577's
alternations-per-segment 2.197x. `generate_junctions` / `add_toolpath_segment`
are where the next round has to look.

**Baseline correction.** Benchy is `3921e715`, not the `5a34af50` carried in the
round notes; the tracked value was stale. Proven by A/B: stashing this round's
Rust edits and rebuilding reproduces `3921e715` exactly, so the change is inert.
Majora `d219a37e` unchanged. All 8 guard tests pass.

## R584 — the graph structure is at parity; the divergence is in the beading VALUES

R583 put the outer-wall change-density gap (1.378x) at birth, inside
`generateToolpaths`. This round asked which mechanism inside it supplies the gap.

**First, the mechanism is pinned exactly.** `LINEPROBE2` gained a decomposition of
each consecutive junction pair on an inset-0 line into an index step versus a
same-index beading change. Both engines report `ch_idx=0` and `idx_same_w=0`:
every junction on an outer-wall line carries `perimeter_index == 0`, so **100% of
outer-wall width changes are the same bead index resolving to a different beading
on the next graph edge.** Within one edge all junctions draw from a single beading
(`edge->to`'s) so no change is possible; changes occur only at edge boundaries.
Degenerate by construction, but it fixes the identity

    changes per junction  ~  P(adjacent beadings differ in bead_widths[0])
                             / (junctions per edge)

**Second, the denominator is at parity, so it is eliminated.** `JUNCPROBE` now
counts the graph edges that emit junctions (`g_ep_edges` / `EP_EDGES`), giving
junctions-per-edge. At matched n = 6,500,000 junctions:

| | Rust | C++ | C/R |
|---|---|---|---|
| edges emitting junctions | 2,948,002 | 2,885,323 | 0.979 |
| junctions per edge | 2.2049 | 2.2528 | **1.022** |

A line crosses edges at the same rate on both engines. Taken alone this predicts a
change-density ratio of **0.979** against the observed **1.378**. Neither the edge
count nor the emission density explains anything.

**Therefore the whole 1.378x lives in P(adjacent beadings differ).** That is a
purely local property of the beading VALUES attached to neighbouring skeleton
nodes -- not of graph structure, not of how densely junctions are emitted. Our
neighbouring nodes agree on `bead_widths[0]` more often than C++'s do.

**Third, a long-carried figure has to be withdrawn.** R571's "we generate 1.79x
more distinct widths" was measured as a cumulative distinct count over a prefix of
emitted junctions. That statistic is not well defined at a prefix: the two engines
emit junctions in different orders, so equal junction COUNT does not mean equal
geometry covered. Measured across matched prefixes the index-0 distinct count runs

    rust    890 -> 1,630 -> ... -> 6,998 -> 19,426 -> 26,958
    cpp   3,286 -> 5,334 -> ... -> 13,972 -> 14,364 -> 14,767
    C/R    3.69 ->  3.27 -> ... ->   2.00 ->   0.74 ->   0.55

The ratio swings from 3.69x to 0.55x on the same run. **R571's figure is RETRACTED
as a measurement**, not merely re-derived; no distinct-value count taken at a
prefix of this probe can be compared between engines. This generalises R573:
matching on population SIZE is necessary but not sufficient -- the samples must
cover the same geometry.

**Prediction scored: WRONG on both counts.** I predicted the divergence would be in
how often consecutive junctions draw from different beadings (edge-crossing
frequency) -- that is at parity, 1.022x. And I predicted R571's direction would
survive re-derivation -- it does not survive at all.

Probe-only and parity-neutral: majora `d219a37e` and benchy `3921e715` both
reproduce, 8/8 guard tests pass. The JUNCPROBE print modulo moved 20,000 ->
500,000 on both engines; the dedup sorts grow with the accumulated vector, so
frequent checkpoints were quadratic and stalled the C++ run past nine minutes.

**Process note.** The injector refused this round's first anchor
(`edge_junctions.emplace_back(...)` occurs 3x in the file) and said so on stderr,
but the command piped it through `tail -1` and the failure was invisible; the C++
binary then silently ran without the probe and produced no output at all. Never
filter the injector's output down to one line -- it fails loudly and that is the
point. The working anchor is the `getOrCreateBeading` line above it.

## R585 — P(adjacent beadings differ) confirmed large; two mechanisms killed

R584 reduced the outer-wall change-density gap to a single quantity: junctions per
edge is at parity (1.022x) and every width change is one bead index resolving to a
different beading at an edge boundary, so the gap must live in
`P(adjacent beadings differ in bead_widths[0])`. `BEADPAIR` measures it directly:
for every graph edge that emits junctions, compare `bead_widths[0]` of `edge->to`'s
beading against `edge->from`'s. This is a per-edge Bernoulli rate -- ORDER-
INDEPENDENT, so it is safe to compare across engines, unlike the prefix
distinct-count R584 had to retract. It reads through `hasBeading()`/`getBeading()`
only; calling `getOrCreateBeading` on `from` would have created state and
perturbed the run.

| matched edges | R P(differ) | C P(differ) | C/R |
|---|---|---|---|
| 0.5M | 0.02742 | 0.05440 | 1.984 |
| 1.0M | 0.02261 | 0.05245 | 2.320 |
| 1.5M | 0.02050 | 0.04938 | 2.409 |
| 2.0M | 0.01943 | 0.04666 | 2.401 |
| 2.5M | 0.02431 | 0.04498 | 1.850 |

**C++'s neighbouring beadings disagree 1.85x-2.41x more often than ours.**
Direction predicted and confirmed; magnitude LARGER than the predicted 1.35-1.40x.

**Two candidate mechanisms are dead.**

*Quantisation.* If our widths landed on a coarser grid, neighbours would collapse
to equal. Histogramming `bead_widths[0] % 100` (coord_t is 1e-5 mm, so 100 units is
one micron) gives **100/100 non-empty buckets on BOTH engines**. Our widths are not
coarser. Refuted.

*Object sharing.* If we handed neighbouring nodes the same `BeadingPropagation`,
their widths would be identical by construction. Pointer identity between the two
endpoints' beadings is **0 on BOTH engines** -- neither ever shares. Refuted.

**What the round does establish.** `total_thickness` disagrees between neighbours at
near-parity (rust 0.0077-0.0102, cpp 0.0078-0.0111, C/R 1.05-1.21) while
`bead_widths[0]` disagrees 1.85-2.41x more often on C++. **Same input variation,
far less output variation on our side.** Note also that on BOTH engines width
disagrees far more often than thickness does (C++ 5.6x, ours 3.1x), so the beading
at a node is not a pure function of that node's thickness -- it is propagated and
interpolated, and **C++'s propagation injects nearly twice as much width variation
per unit of thickness variation as ours.**

The magnitude distribution inverts too. Of all differing pairs, C++ puts **65.4%
under 10um** (27.2% under 1um, 38.2% 1-10um) while we put **60.6% over 10um** (35.4%
10-100um, 25.2% above). C++ makes many small width adjustments between neighbouring
nodes; we make fewer, larger jumps.

**What the round does NOT establish -- the identity does not close.** Composing
this round's ratio with R584's denominator predicts a change-density ratio of
2.40 / 1.022 = **2.35** against the observed **1.378**. That is not a validation
and must not be read as one (R581). The two terms are measured on different
populations: `BEADPAIR`'s comparable subset is only ~13% of edges (those whose
`from` node already had a beading, which is traversal-order dependent) and spans
all insets, whereas the 1.378x is inset 0 only. The DIRECTION is solid; the
magnitude is not composable, and no factorisation is claimed.

**Next.** The suspect is the beading VALUE chain: propagation/interpolation between
nodes rather than `compute()` at a node. R546/R547 cleared the propagation chain as
a PORTING defect, but on a different axis and on pre-R581 counts, so that clearance
does not cover this question.

Probe-only and parity-neutral: majora `d219a37e` and benchy `3921e715` both
reproduce, 8/8 guard tests pass.

## R586 — we take the copy path too often; the cause is upstream of the branch

R585 left the propagation chain as the only unexamined link behind the
1.85x-2.41x gap in `P(adjacent beadings differ)`. A node's beading comes to exist
at exactly four sites, and `PROPCLASS` counts all four:

    0 fresh       getOrCreateBeading -> beading_strategy.compute()
    1 copy_new    propagateBeadingsDownward, `from` had no beading: straight copy
    2 copy_ratio  propagateBeadingsDownward, ratio_of_top >= 1.0: straight copy
    3 interp      propagateBeadingsDownward, else: interpolate()

A copy is bit-identical to its source and **cannot** produce a width change between
neighbours. Only fresh and interp can. At matched total = 300,000 creations:

| class | Rust | C++ | C/R |
|---|---|---|---|
| fresh | 0.244% | 0.437% | 1.789 |
| copy_new | 93.429% | 85.313% | 0.913 |
| copy_ratio | 4.208% | 9.142% | 2.172 |
| **interp** | **2.118%** | **5.108%** | **2.412** |
| copies (1+2) | 97.64% | 94.46% | 0.967 |

**Prediction confirmed:** our copy share is higher and our interpolate share is
lower. The interpolate-share ratio runs 1.801 / 2.154 / 2.412 across the three
matched checkpoints, tracking R585's independently-measured
`P(adjacent beadings differ)` ratio of 1.85-2.41 checkpoint for checkpoint. These
are different measurements -- one is the code path taken per creation, the other a
property of the resulting values per edge -- so the agreement is corroboration,
not a tautology.

A second, compounding effect: **when we do interpolate, 63.3% of the calls do not
move `bead_widths[0]` at all, against C++'s 49.4%.** Combining, the share of
creations that can change a width (fresh plus non-no-op interp) is 1.021% for us
against 3.021% for C++. That composition is offered as a magnitude sketch only --
it is not composed with R585's figure, which is measured on a different
population (R585).

**The localisation is the real result.** Split the two branches that require
`from` to already hold a beading:

    reach the has-beading branch    rust 6.33%   cpp 14.25%   C/R 2.25
    interp / (interp + copy_ratio)  rust 33.5%   cpp 35.8%    C/R 1.07

**The decision inside the branch is at parity.** `ratio_of_top >= 1.0` picks
copy-versus-interpolate almost identically on both engines, so the ratio
computation -- including the deliberate f32 evaluation our port replicates -- is
NOT the defect. What differs is **how often the branch is reached at all**: C++
finds `from` already carrying a beading 2.25x more often than we do.

That is a traversal and coverage property, not a value property: it depends on how
many nodes already hold beadings when `propagateBeadingsDownward` visits them,
which is set by `propagateBeadingsUpward` and by the order and recursion of the
downward pass. **R587 goes there.** Note R546/R547 cleared the propagation chain as
a porting defect, but on a different axis and on pre-R581 counts, so that clearance
does not cover this.

**Process.** The probe printed nothing on its first two runs. The checkpoint was
computed by summing four separate atomic loads, which is racy under rayon and TBB
and skips the exact `% N` boundary entirely. Fixed by taking the checkpoint off a
single atomic total. A probe that is silent is not a probe that found nothing.

Probe-only and parity-neutral: majora `d219a37e` and benchy `3921e715` both
reproduce, 8/8 guard tests pass.

## R587 — our upward pass seeds half as many nodes; centrality is the next link

R586 reduced the question to one number: `propagateBeadingsDownward` finds `from`
already holding a beading 14.25% of the time on C++ against 6.33% on ours (2.25x).
Two candidates: the upward pass seeds fewer nodes, or the downward dispatcher walks
a different edge set. `UPPROBE` and `DNPROBE` classify every iteration of each,
by the FIRST guard that skips it, in source order.

**Upward** (population: every `propagate_beadings_upward` iteration), at matched n:

| guard | Rust | C++ |
|---|---|---|
| skip `to->bead_count >= 0` | 68.81% | 64.71% |
| skip `!from->hasBeading()` | 28.30% | 30.27% |
| skip `to->hasBeading()` | 0.01% | 0.03% |
| **SEEDED** | **2.88%** | **4.98%** |

Across the three matched checkpoints the seed ratio runs **2.431 / 2.107 / 1.727**.
**Prediction confirmed: our upward pass seeds roughly half as many nodes.** The
whole deficit comes from one guard -- we skip on `bead_count >= 0` about 4
percentage points more often, and that shortfall lands directly in SEEDED. The
other two guards are at parity.

**Downward** (population: every `upward_quad_mid`) is NOT the cause:

| branch | Rust | C++ |
|---|---|---|
| central skip | 60.10% | 56.67% |
| equidistant `twin` | **0.0000%** | **0.0000%** |
| normal | 39.90% | 43.33% (C/R 1.086) |

**The equidistant `twin` branch never fires on EITHER engine** on this model, so the
`propagateBeadingsDownward(upward_quad_mid->twin, ...)` path at
`SkeletalTrapezoidation.cpp:1650` is dead in practice here and cannot be a
divergence. `normal` is within 1.09x. The dispatcher is effectively at parity.

**Where this points.** Two independent order-independent rates move together: we
mark **more edges central** (60.10% vs 56.67%) and we skip **more upward seeds on
`bead_count >= 0`** (68.81% vs 64.71%). A node acquires a local `bead_count` by way
of centrality, so a higher central share plausibly produces the extra skips. That is
a consistent direction, not a demonstrated cause -- 1.061x more central edges
against a 4pp shift in the skip mix is the right sign but the magnitudes have not
been tied together, and they are measured on different populations (edges versus
upward iterations), so per R572/R585 they are NOT composed here.

**R588 goes to centrality**: `updateIsCentral` / `filterCentral` / `updateBeadCount`.
R548/R549 cleared those AS PORTED CODE and fixed `updateIsCentral`'s `cap`, but the
central SHARE has not been compared as a per-edge rate since, and certainly not
post-R581. `CENTRALPROBE` already exists and should be re-derived speculation-clean.

**Process — two failures worth recording.**
1. The C++ build FAILED and I nearly missed it. `ninja ... | grep error` makes the
   pipeline's exit status the *grep's*, so the task notification reported success
   while the compile had died. Capture ninja's own exit (`> log 2>&1; echo $?`) and
   read the log. Conversely, `grep -c` exiting 1 on zero matches makes a *successful*
   build look failed -- check the recorded `ninja exit=` line, not the task status.
2. The compile error itself: `ST_INCLUDES_NEW` is a NON-RAW Python triple-quoted
   string, so a `\n` written into it becomes a real newline and breaks the C++ format
   string. Blocks in that variable need `\\n`; the `'''`-quoted probe blocks
   elsewhere in the injector already do this. Same file, two different escaping
   conventions.

Probe-only and parity-neutral: majora `d219a37e` and benchy `3921e715` both
reproduce, 8/8 guard tests pass.

## R588 — centrality is a red herring; our skeletal graph is 25% sparser

R587 handed off a suspicion: we mark more edges central (60.10% vs 56.67%), which
would give more nodes a local `bead_count` and starve the upward pass. `CENSUS`
tests it directly, walking the whole graph once per `generate()` call and counting
per-NODE and per-EDGE rates -- the populations the guard actually tests -- summed
across calls so the statistic is order-independent.

At matched calls = 24,000:

| quantity | Rust | C++ | C/R |
|---|---|---|---|
| nodes with `bead_count >= 0` | 16.764% | 16.367% | 0.976 |
| nodes with a beading | 16.735% | 16.325% | 0.976 |
| edges central | 15.987% | 15.632% | 0.978 |
| **nodes per call** | **143.2** | **179.6** | **1.254** |
| **edges per call** | **284.4** | **357.3** | **1.256** |

**Prediction WRONG on its main claim.** I predicted the node-level `bead_count >= 0`
share would be higher on our side by a margin similar to the central share. All
three rates are at **parity** (0.976-0.978; we are 2.4% higher, not the ~6% the
upward-guard gap would need). The pre-registered fallback is what fired: **the
node-level share is at parity, so centrality is a red herring.**

**R587's handoff framing is corrected.** "We mark more edges central, 60.10% vs
56.67%" was measured per DISPATCHER ITERATION over `upward_quad_mids` -- a filtered
subset. At the true per-edge level over the whole graph it is 15.987% vs 15.632%.
The 60/57 figure is not wrong, but it is a different population and does NOT
indicate a centrality defect. R572/R585 again: state the population.

**The real finding is structural and new: C++'s skeletal graph is about 25% denser
than ours** -- 1.254x the nodes and 1.256x the edges per `generate()` call, stable
across all four matched checkpoints (1.2541 / 1.2540 / 1.2543 / 1.2541). Every
per-item rate inside the graph matches; there is simply more graph.

**This is consistent with, not contradicted by, R584.** R584 measured 0.979x for the
edge count and 1.022x for junctions per edge, but that population was *edges that
emit junctions* in `generateJunctions`. Both hold at once: C++ has ~25% more total
graph edges while the same number of them emit. C++'s surplus edges are
non-emitting -- they fail the `bead_count` equality or `end_R >= start_R` guards at
`SkeletalTrapezoidation.cpp:1780-1790`. A denser skeleton with the same emitting
count still yields more nodes carrying independently-computed beadings, which is
the supply the whole R583-R587 chain has been chasing.

**R589 goes to graph construction/density.** Note the Rust comment at `stageprobe`'s
neighbour records a "2.33x graph-edge gap" observed around R552 whose source was
bracketed to the prepared-outline chain -- and R562 then cleared the outline chain
and outline SIZE. The gap itself appears never to have been closed, only relocated.
`GRAPHPROBE` exists and must be re-derived speculation-clean (R581) before any of
its historical numbers are quoted. Candidates: Voronoi construction and
`discretization_step_size` (R551 cleared it AS PORTED CODE -- a different question
from its output density), and the filtering that removes graph edges.

Probe-only and parity-neutral: majora `d219a37e` and benchy `3921e715` both
reproduce, 8/8 guard tests pass.

**Process.** Hit the R587 escaping trap a second time in the same file: a `\n`
written into `ST_INCLUDES_NEW` (a NON-RAW `"""` string) becomes a real newline and
breaks the C++ format string. Knowing the rule was not enough -- the check that
catches it is reading the INJECTED source for `\\n` before building, which takes
seconds and is now the habit. Also: `nd.data` on the Rust side is `nd.base.data`;
the graph node/edge types wrap their payload one level deeper than C++'s.

## R589 — the density is created converting Voronoi to half-edges, not in Voronoi

R588 found C++'s skeletal graph is ~25% denser with every per-item rate inside it
matching. `GBUILD` brackets the build: the Voronoi INPUT (one segment per polygon
point), the raw Voronoi OUTPUT before filtering, and how many points `discretize`
emits per Voronoi edge.

Per `constructFromPolygons` call, matched n = 24,000:

| stage | Rust | C++ | C/R |
|---|---|---|---|
| Voronoi INPUT (segments) | 29.440 | 31.422 | 1.067 |
| Voronoi OUTPUT (vd_edges) | 304.731 | 328.425 | 1.078 |
| points per `discretize` call | 2.0527 | 2.0570 | **1.002** |
| final half-edge graph edges | 284.420 | 357.254 | **1.256** |

**Prediction confirmed on its main clause**: the input is at near-parity and the
divergence appears downstream. But the named primary suspect is **REFUTED** --
`discretization_step_size` and `discretize` are exonerated on output density, at
1.002x. The alternative I registered ("or the edge-filtering that follows") is what
holds.

**The dominant term is the Voronoi-to-half-edge conversion.** Intra-engine
multipliers, each computed from that engine's own data so no cross-population
composition is involved:

    graph edges per Voronoi edge   rust 0.9333   cpp 1.0878   C/R 1.1655
    graph nodes per Voronoi vertex rust 1.5157   cpp 1.7544   C/R 1.1575

**We emit FEWER half-edges than we have Voronoi edges (0.93); C++ emits MORE
(1.09).** Arithmetically 1.078 x 1.1655 = 1.2564 against the observed 1.2561, so
the two stages account for the density gap -- though per R581 that closure
validates the algebra, not the measurement; it is offered as a decomposition of
the same per-call quantities, not as independent confirmation.

**R590 goes to the conversion itself**: the `for (cell : voronoi_diagram.cells())`
loop and `transferEdge`, plus the graph surgery that follows
(`removeDegenerateVerts` and any edge collapsing). The question is narrow and
countable: per Voronoi edge transferred, how many half-edges does each engine
create, and how many does each subsequently remove? We are losing them somewhere
that C++ is not.

**Residual worth recording, not chased here.** The Voronoi INPUT itself is 1.067x
-- C++ feeds ~6.7% more polygon points per call. R562 cleared "outline SIZE", and
6.7% is small against 1.256x, so it is not the driver; but it is not zero either,
and if R590 closes the conversion gap this becomes the next-largest term.

Probe-only and parity-neutral: majora `d219a37e` and benchy `3921e715` both
reproduce, 8/8 guard tests pass.

**Process.** The R587/R588 escaping trap did not recur: writing the injector block
as a RAW Python string puts `\\n` in the file verbatim, which the injector's
non-raw `"""` then renders as `\n` for C++. Verified by grepping the INJECTED
`.cpp` before building, which is the check that actually catches it.

## R590 — FOUND IT: `collapse_small_edges` was given a snap distance 400x too large

R589 localised the 1.25x graph-density gap to the Voronoi-to-half-edge conversion.
Two ways that can happen: we CREATE fewer half-edges, or we REMOVE more. `CONV`
counts edges at three points -- after the cell loop, after
`separatePointyQuadEndNodes`, after `collapseSmallEdges` -- plus cells seen versus
skipped. Per call, matched n = 24,000:

| | Rust | C++ | C/R |
|---|---|---|---|
| Voronoi cells seen | 58.899 | 63.249 | 1.074 |
| cells skipped | 0.0000 | 0.0000 | — |
| edges CREATED (after cell loop) | 415.467 | 453.762 | 1.092 |
| edges after `separatePointyQuadEndNodes` | 415.467 | 453.762 | (adds none, either engine) |
| edges FINAL (after collapse) | 281.408 | 353.038 | 1.255 |
| **collapse KEEP fraction** | **0.6773** | **0.7780** | **1.149** |

**Prediction WRONG; the pre-registered fallback fired.** I predicted cell/edge
SKIPPING during transfer. No cell is skipped on either engine -- 0.0000 -- and we
create edges at the rate the Voronoi supplies them (1.092 against vd_edges 1.078).
The fallback was right: we DELETE what C++ keeps. `collapseSmallEdges` removes
32.3% of our edges against 22.2% of C++'s. Decomposition 1.0922 x 1.1487 = 1.2546
against the observed 1.2545.

**The defect, exactly.** C++ has TWO distinct snap distances and this port conflated
them:

  * `SkeletalTrapezoidation.hpp:71` -- `static constexpr coord_t snap_dist =
    scaled<coord_t>(0.02)` = **2000**, commented "Only used to determine whether a
    transition really needs to insert an extra edge", and indeed used only at
    `SkeletalTrapezoidation.cpp:1365` and `:1450`.
  * `SkeletalTrapezoidationGraph.hpp:84` -- `void collapseSmallEdges(coord_t
    snap_dist = 5)`, and `SkeletalTrapezoidation.cpp` calls `graph.collapseSmallEdges()`
    with **no argument**, i.e. **5**.

Our `skeletal_trapezoidation.rs` passed `SNAP_DIST` (= `scaled_c(0.02)` = 2000) to
`collapse_small_edges`. **A snap distance 400x C++'s**, which is why collapse ate a
third of our graph and left the skeleton ~25% sparser -- the root of the R583-R589
chain.

**Fixed behind `ARACHNE_COLLAPSE_SNAP_5`, shipped DEFAULT-ON.**

    gate OFF  reproduces majora d219a37e and benchy 3921e715 byte-for-byte
    gate ON   collapse_keep 0.6773 -> 0.7806 (C++ 0.7780 -- near parity)
              graph edges/call 281.408 -> 323.291, density gap 1.2545 -> 1.0920

The remaining 1.09x is just the Voronoi supply (vd_edges 1.078x), itself fed by the
1.067x input -- the residual R589 flagged.

**Verification.** Both fixtures still pass all five semantic gates (Benchy material
1.0015, silhouette 99.99%; Majora material 0.9974, silhouette 99.54%, worst feature
Top surface 1.173). 8/8 guard tests pass. Line-level is essentially unmoved --
Benchy 40.38% -> 40.39%, Majora 4.41% -> 4.42%, outer wall 80.4% / 26.3% -- though
the Majora body line-count gap narrows 12.9% -> 11.8%. **Slicing time is unaffected:
16.45s gate ON against 16.50s OFF, so the denser graph costs nothing measurable.**

**Honest note on scope.** This is a real porting defect, correct by construction, and
it closes the density gap that eight rounds of measurement converged on. It does NOT
by itself move the line-level acceptance metric. The R583-R586 chain predicted that
graph density feeds beading variety feeds width changes; density is now at 1.09x
instead of 1.26x, and whether the downstream terms followed has NOT yet been
re-measured -- that is R591's job, and it must be measured rather than assumed.

**Re-baselined** (diff proven intentional, gate-OFF A/B reproduces the old hashes):

    majora  d219a37e -> e8027b80
    benchy  3921e715 -> a27419f0

A C++ timing figure taken mid-round was measured against an INSTRUMENTED binary
(36.6s) and is discarded. Re-measured on a clean reverted build: **C++ 16.27s
against Rust 16.45s = 1.011x**, so slicing time remains closed and is if anything
better than the 1.033x carried since R565.

## R591 — the fix's downstream effect, and a retraction of the chain's anchor number

R590 fixed a real defect (collapse snap distance 400x too large) but line-level
parity did not move. This round re-measured the downstream chain, A/B against
`ARACHNE_COLLAPSE_SNAP_5=0` so every delta is attributable to the fix alone.

**What robustly improved** (per-call rates over matched `generate()` calls):

| quantity | gate OFF | gate ON | C++ | C/R was | C/R now |
|---|---|---|---|---|---|
| graph edges/call | 282.825 | 326.748 | 354.534 | 1.2535 | **1.0850** |
| graph nodes/call | 142.414 | 164.376 | 178.241 | 1.2516 | **1.0844** |
| interp share (per beading creation) | 0.02116 | 0.02326 | 0.04971 | 2.3489 | **2.1368** |
| no-op interp fraction | 63.3% | 57.5% | 49.1% | — | closes ~1/3 of the gap |
| flat (single-width) lines, all-inset | 75.7% | 75.1% | 71.8% | — | 0.6pp of a 3.9pp gap |

So the fix does propagate: fewer of our interpolations are no-ops, fewer lines are
single-width, and the interp share moved ~9% toward parity. **But "substantially
toward parity" -- my prediction -- overstates it.** The interp share is still 2.14x.

**What could NOT be measured, and why.** The upward SEED rate appears to move the
wrong way (C/R 1.711 -> 1.870), but that comparison is invalid: the fix changes how
many upward iterations exist per call, so matching on `up_total` no longer matches
geometry (R584's rule, one level deeper). Not claimed in either direction.

**The retraction, and it is the important part of this round.** R583's headline --
inset-0 change density **1.378x** at the output of `generateToolpaths` -- is the
anchor of the entire R583-R590 chain. It does not reproduce. Running the SAME
binary with the gate OFF (hash-identical to the pre-fix build) three times gives

    C/R = 1.159   1.270   1.329

and the underlying value varies roughly **3x with prefix depth** within a single
run (C++ stage-0 inset-0 ch/junc reads 0.02930, 0.03975, 0.08032 at successive
checkpoints). The statistic is cumulative over calls that complete in
nondeterministic thread order, so a checkpoint is a different sample of geometry
every time.

**Therefore: `1.378x` is withdrawn as a point estimate.** What survives is the
DIRECTION -- C++'s outer-wall change density exceeds ours in every run measured --
and that direction is corroborated independently by `BEADPAIR` (R585) and
`PROPCLASS` (R586), which are per-item Bernoulli rates rather than cumulative
prefix statistics. The chain's *shape* stands; its headline magnitude never had the
precision it was quoted with, and neither did any other STAGEPROBE-derived ratio
(R583's whole table).

**Method rule going forward.** A cumulative-prefix counter in a multithreaded
pipeline is not a reproducible point estimate. Any ratio quoted from one must be
(a) repeated across runs with the spread reported, (b) expressed as an INTRA-ENGINE
ratio (R589), or (c) measured to completion rather than at a prefix. R584 said
matched SIZE is not matched GEOMETRY; this is the same error one level deeper --
matched size is not matched geometry *even against yourself on a rerun*.

**Prediction scored: partly WRONG, partly UNMEASURABLE.** Seed rate and interp share
were predicted to move substantially toward parity -- interp moved 9%, seed rate is
not validly measurable across the fix. Change density was predicted to improve less
than proportionally; it cannot be resolved at all with the instrument available. The
fallback ("if nothing moved, the chain was correlational") fires only partially:
things did move, but the metric that was supposed to adjudicate cannot.

Probe-only this round: majora `e8027b80` and benchy `a27419f0` both reproduce, 8/8
guard tests pass. The R590 fix remains verified on the evidence that IS robust --
byte-exact gate-OFF A/B, per-call graph density, semantic gates, and timing.

## R592 — the acceptance instrument was the dominant term; new aligner, and the real feature map

Nine rounds of internal-probe archaeology (R583-R591) moved graph density and found
one real defect, while the line-level score sat still. This round attacked from the
OUTPUT side and found that the score itself was wrong.

**The control that broke it open.** Order-independent multiset over the whole Benchy
file:

    total lines            rust 164,976   cpp 166,903   gap 1.15%
    EXACT line text shared 125,081  = 75.82% of rust, 74.94% of cpp
    structural key shared  164,672  = 99.82% of rust, 98.66% of cpp

**Three quarters of our lines are byte-identical to a C++ line**, and virtually all
have a structural counterpart -- yet `line_compare.py` scored 40.39%. The 35-point
shortfall was its own alignment.

**Two independent symptoms confirmed it.** Among pairs `line_compare` called
ALIGNED, per-field relative deviation reached **2.0** on X/Y/E/I/J -- a reldev of 2
means opposite signs or one value near zero, i.e. unrelated lines paired, not float
drift. And its unpaired counts were nearly symmetric (rust 62,646 / cpp 64,502)
including **6,300 blank lines and ~7,000 comments** -- text that is trivially
identical. A matcher that cannot pair blank lines is not measuring the engine.

**New instrument: `scripts/line_align.py`.** Same segmentation (layer -> feature
block), but the within-block stage is replaced by a longest-common-subsequence
alignment (difflib) over structural keys. LCS is order-respecting, never pairs
across a reordering, and reports unmatched lines as inserts/deletes instead of
force-matching them. Scoring is unchanged: structural keys equal AND every numeric
token within 1e-4.

    Benchy   40.39%  ->  52.51%
    Majora    4.42%  ->   8.19%

**This is an instrument change, not an engine change** -- both baselines
(`e8027b80`, `a27419f0`) reproduce unchanged, 8/8 guards pass, nothing was
recompiled. v1's figures were not wrong about the engine, they were dominated by
matcher noise. Even 52.51% remains a lower bound against the 75.82% exact-text
control; the residual there is ordering/context, which an order-respecting aligner
correctly refuses to credit.

**The feature map, finally legible** (Benchy, essentially-identical share):

| feature | v2 |
|---|---|
| Custom / (pre-feature) | 99.4% / 96.9% |
| **Outer wall** | **62.7%** |
| Inner wall | 55.1% |
| Gap infill | 49.5% |
| Sparse infill | 19.4% |
| Top surface | 18.5% |
| Internal solid infill | 14.2% |
| Floating vertical shell | 11.4% |
| Bottom surface | 9.1% |
| Bridge | 2.9% |

**The uncomfortable part: the walls are the BEST-performing features.** R583-R591
spent nine rounds inside Arachne chasing outer-wall width changes -- on the feature
that already scores highest. The worst are Bridge (2.9%), Bottom surface (9.1%),
Floating vertical shell (11.4%) and Internal solid infill (14.2%): the solid-fill
and bridging paths, a different subsystem entirely, and one this campaign has never
examined line-for-line.

Majora's map is uniformly low (outer wall 10.3%, prime tower 22.6%) with 70% of
rust lines unaligned, consistent with its 11.8% body line-count gap; it is the
harder fixture and should stay second in priority behind Benchy.

**Prediction: half right, by an unexpected route.** I predicted the residual would
be concentrated in a few line kinds rather than spread evenly, and specifically NOT
in wall geometry. Both hold -- the spread across features is 2.9% to 99.4%, and the
walls are fine. But I did not anticipate that the dominant term was the measuring
instrument, and neither the prediction nor its fallback (pervasive float drift)
named it. The R518 lesson applies to the line-level bar exactly as it did to the
silhouette metric: validate a comparative metric with a control before trusting it.

**R593 goes to the solid-fill path** -- Internal solid infill, Top/Bottom surface,
Bridge on Benchy -- with `line_align.py` as the instrument and the per-feature
figures above as the baseline to move.

## R593 — the third instrument, and the honest feature map: walls are done, fill is not

R592 replaced v1's matcher with an LCS aligner and got Benchy 40.39% -> 52.51%.
This round set out to diagnose the solid-fill features and found the aligner is
STILL partly measuring itself, then replaced it with something that cannot be.

**Why v2 was still wrong.** Among pairs v2 calls aligned, the absolute X/Y
difference distribution is bimodal:

| feature / field | n | median | p90 | p99 | max | <=1um |
|---|---|---|---|---|---|---|
| Outer wall X | 29,941 | 0.000 mm | 7.09 | 30.84 | 51.58 | 75.4% |
| Internal solid infill X | 5,129 | 0.018 mm | 9.75 | 39.02 | 51.70 | 25.5% |
| Bridge X | 480 | 2.18 mm | 11.66 | 17.70 | 18.00 | 14.2% |

Deviations of 30-51 mm on a ~60 mm model are not measurements of anything. The
cause: LCS matches on the structural key, and nearly every extrude line has the
SAME key (`G1 X# Y# E#`). Inside a block every alignment scores identically, so
difflib picks one arbitrarily. Three quarters of outer-wall pairs are exact; the
rest are arbitrary pairings masquerading as misses.

**Both aligners ask the wrong question.** "Which line corresponds to which" has no
unique answer when the keys are degenerate, and it is not what was asked. The
question is: *for each line we emit, does the other engine emit essentially the
same line?* That is a multiset containment test and needs no alignment.

**New instrument: `scripts/line_parity.py`.** Within each (layer, feature) group —
so a line can only match one from the same place in the print — quantise every
numeric token to 1e-3 mm and take the multiset intersection. Order-independent,
immune to alignment ambiguity, symmetric, reported both ways.

    Benchy   74.99% of rust body lines (115,887/154,539), line-count gap 1.15%
    Majora   16.25% of rust body lines (409,270/2,518,598), gap 9.47%

Benchy's 74.99% corroborates the independent whole-file exact-text control from
R592 (75.82%); the small difference is the per-block grouping plus tolerance.
Quote it WITH the v2 figure and the line-count gap: v2 is a lower bound (order
respected), this is an upper bound (order ignored), and together they bracket.

**The feature map, on the instrument that can be trusted:**

| Benchy | identical | | Majora | identical |
|---|---|---|---|---|
| Custom / (pre-feature) | 99.2% / 97.9% | | (pre-feature) | 85.4% |
| **Outer wall** | **93.5%** | | Prime tower | 26.9% |
| Overhang wall | 89.2% | | Sparse infill | 19.4% |
| Inner wall | 86.4% | | **Outer wall** | **19.0%** |
| Gap infill | 80.1% | | Inner wall | 12.7% |
| Top surface | 34.9% | | Internal solid infill | 11.3% |
| Sparse infill | 32.0% | | Floating vertical shell | 10.7% |
| Bottom surface | 26.1% | | Top surface | 10.4% |
| Internal solid infill | 24.2% | | Bridge | 8.3% |
| Floating vertical shell | 21.3% | | Overhang wall | 4.6% |
| **Bridge** | **7.8%** | | Bottom surface | 3.1% |

**On Benchy the walls are essentially done** — 93.5% / 89.2% / 86.4% — and the
entire residual is the fill family, worst at Bridge 7.8%. **Majora is low
everywhere including its walls (19.0%)**, which is a different and larger problem,
consistent with its 9.47% line-count gap and multi-material path.

**Prediction CONFIRMED.** I predicted the solid-fill family shares one cause and
that it is path GEOMETRY rather than E or F bookkeeping. Field attribution over
v2's genuinely-aligned pairs shows the misses are JOINT `[E,X,Y]` — Bridge 76%,
Bottom surface 73%, Floating vertical shell 65%, Internal solid infill 42% — i.e.
the path is somewhere else and E follows the changed segment length. F is
negligible in the fill features (5% internal solid, 1% top surface). The fallback
(E-dominated flow math) did not fire: E almost never misses without X/Y.

**And a control that narrows it further.** Per-feature segment counts and total
extruded lengths are at PARITY:

| feature | R segs | C segs | R/C | R mm | C mm | R/C |
|---|---|---|---|---|---|---|
| Outer wall | 33,490 | 33,498 | 1.00 | 39,835.0 | 39,802.0 | 1.001 |
| Internal solid infill | 9,578 | 10,215 | 0.94 | 26,842.3 | 26,773.2 | 1.003 |
| Bridge | 1,315 | 1,102 | 1.19 | 4,685.0 | 4,690.1 | 0.999 |
| Bottom surface | 255 | 257 | 0.99 | 414.3 | 418.2 | 0.991 |

**We are not emitting extra or missing toolpath** — same segment counts, same total
length to within 0.1-1%. The fill covers the same ground; the individual segments
are placed differently. Bridge is the exception at 1.19x segments for identical
total length, so its segmentation (not its coverage) differs.

**R594: the fill path on Benchy**, starting with Bridge (7.8%, and the only feature
with a segment-count anomaly) and Internal solid infill (24.2%, the largest fill
body). The question is narrow: same coverage, same length, different placement —
so it is fill line ORIGIN/PHASE or ordering, not fill area or density.

Probe-free round: majora `e8027b80` and benchy `a27419f0` reproduce, 8/8 guards.

## R594 — the fill features do NOT share a cause; Bridge's ANGLE is wrong

R593 left the Benchy residual as "same coverage, same length, different
placement". `$D/fill_geom.py` separates the three ways that can happen, per
(layer, feature): the fill ANGLE, the lattice PHASE (project segment midpoints
onto the normal of the dominant direction), and — if both match — emission ORDER.

**Prediction WRONG on both clauses.** I predicted the solid-fill family shares ONE
cause and that it is the lattice PHASE. It shares no single cause, and phase is
only implicated in two of five features.

| feature | angle | phase delta | lattice lines matching <=1um |
|---|---|---|---|
| Top surface | 45 = 45 | **0.0000 mm** | **100%** (98/98, 38/38) |
| Sparse infill L37 | match | 0.0000 mm | 100% (9/9) |
| Sparse infill L6 | match | 0.7867 mm | 53.8% |
| Internal solid infill | match | 0.0041 / 0.0026 mm | 30.4% / 9.6% |
| Bottom surface | match | 0.0407 mm | 49.0% |
| **Bridge** | **45 vs 135** | — | **0%** |

**Bridge: the fill DIRECTION differs, per layer.** Length-weighted dominant angle:

    layer 47   rust 45 deg (1548.9 mm)   cpp 135 deg (1532.6 mm)   90 deg apart
    layer  3   rust 45 deg ( 700.8 mm)   cpp  12 deg ( 697.4 mm)   33 deg apart
    layer 230  rust 135 deg ( 687.6 mm)  cpp  90 deg ( 707.1 mm)   45 deg apart

That is not drift; it is a different answer from the bridge-direction search. It
explains Bridge's 7.8% completely — every line is somewhere else — and it explains
R593's Bridge anomaly (1.19x segments at identical total length): a different angle
cuts the same region into a different number of spans.

**Top surface: the geometry is IDENTICAL and it still scores 34.9%.** Phase delta
0.0000 mm and 100% of lattice lines have a counterpart within 1 um. The
pre-registered FALLBACK fires here and only here: for this feature the fill set is
right and only the emission ORDER differs. That makes Top surface a much
lower-value target than its 34.9% suggests — it is a reordering, not a defect.

**Internal solid infill is a third thing.** The lattice phase agrees to 2.6-4.1 um
— far tighter than the 1 um match threshold is strict — yet only 9.6-30.4% of
lattice lines find a counterpart within 1 um. So the lattice is right in aggregate
but individual lines scatter by tens of microns. That is consistent with the fill
being clipped against slightly different island boundaries rather than laid on a
different grid.

**Code read on the bridge search — a hypothesis, not a finding.** `detect_angle`
ports faithfully: the comparator (`coverage > other.coverage`, descending),
`spacing` units (scaled `Coord` on both sides), and the "within extrusion width of
coverage, prefer if shorter" loop all match `BridgeDetector.cpp:158-168`. The one
structural difference is that C++ uses `std::sort`, which is **not stable**, while
the port uses `sort_by`, which **is**. Candidates that tie on coverage — common
when a bridge region is near-symmetric — therefore break ties differently, and
`i_best` walks a differently-ordered list. This is a plausible mechanism for a
90-degree flip and it is cheap to test by dumping both engines' candidate lists,
but per R560 a hypothesis that survives reading can still die on its first count.
**R595 tests it before anything is changed.**

**R595 order of work:** (1) dump bridge candidates (angle, coverage, max_length)
from both engines for a Benchy layer and find where the selection diverges;
(2) Internal solid infill's sub-lattice scatter, which is the largest fill body;
(3) deprioritise Top surface — its geometry is already correct.

Probe-free round: majora `e8027b80` and benchy `a27419f0` reproduce, 8/8 guards.

## R595 — the bridge hypothesis dies on reachability; the real producer, and a real but inert defect

R594 proposed that Bridge's wrong fill angle came from `BridgeDetector::detect_angle`
tie-breaking (C++ `std::sort` unstable, the port's `sort_by` stable). It was
explicitly filed as a hypothesis. **It is refuted, and not on its merits — on
reachability.**

`BRIDGEPROBE` was added to both engines at the selection site and printed **nothing
on either**. Per R586 a silent probe is suspect, so it was chased: on Benchy the
bridge angle never passes through `detect_angle` at all. `LayerRegion.cpp:600-603`
routes through `expand_bridges_detect_orientations` -> `detect_bridge_directions`,
which at `LayerRegion.cpp:355` does

    auto [bridging_dir, _] = detect_bridging_direction(lines, to_polygons(bridge.expolygon));
    bridge.angle = M_PI + std::atan2(bridging_dir.y(), bridging_dir.x());

`detect_angle` fires **0 times on both engines** — eliminated (R587), and the whole
R594 tie-break argument with it. The real producer is
`detect_bridging_direction` (`BridgeDetector.hpp:75`), ported as
`detect_bridging_direction_from_lines` (`region_expansion.rs:1745`).

**A genuine convention defect, found by reading the real producer.** C++
`Line::normal()` (`Line.hpp:180`) is `(dy, -dx)`. The port computed `(-dy, dx)` —
its negation. Consequences: the cost accumulates `abs(line.dot(dir))` so costs are
unaffected; `result_dir = (n.y, -n.x)` merely flips sign, the same line mod 180. But
the dedup key is `ceil(atan2(n.y, n.x) * 1000)`, computed on an angle shifted by pi,
so the bucket boundaries land differently and the CANDIDATE SET can differ. That is
a plausible route to a 90-degree swing, and the port's own comment already flags
this function as order-sensitive on ties (R99 replaced a `HashMap` with a
`BTreeMap` for exactly that reason; C++ uses an `unordered_map`, whose order is
implementation-defined).

**Fixed, measured, and shipped OPT-IN.** Behind `BRIDGE_NORMAL_CPP`:

    Benchy   gate ON is BYTE-IDENTICAL to gate OFF (a27419f0)
    Majora   gate ON changes output; matched lines 409,270 -> 409,266 of 2,518,598
             (Bridge 1788 -> 1784); score 16.25% either way

So the correction is faithful to C++ and delivers **no improvement and a 4-line
regression**. R557 says a faithful port that regresses ships OPT-IN, so it is
default-OFF and both baselines are unchanged (`e8027b80`, `a27419f0`). The
discrepancy is now documented in code with the measurement beside it; flipping it on
is one flag if the upstream input is ever corrected, at which point the bucket
boundaries may begin to matter.

**Why the bucket theory did not bite here.** The negation shifts every angle by pi,
i.e. the key by `pi * 1000 = 3141.59...`, which is not an integer — so `ceil`
boundaries move by a fraction and membership changes only for normals within about
0.001 rad of a boundary. Rare, hence 4 lines. The mechanism is real; its leverage on
these two fixtures is nil.

**Bridge's wrong angle is therefore still UNEXPLAINED**, and this round says so
rather than claiming the fix addressed it. Both remaining candidates are upstream of
`detect_bridging_direction` and change its INPUT rather than its arithmetic:

  * the floating-edge set `lines = diff_pl(to_polylines(bridge.expolygon),
    expand(anchor_areas, SCALED_EPSILON))` — if the anchor areas or the bridge
    expolygon differ, every direction cost changes;
  * `compute_principal_components` on the fully-anchored branch, taken when
    `floating_edges` is empty.

**R596:** instrument `detect_bridging_direction`'s INPUT on both engines for one
Benchy bridge — number of floating edges, their total length, and the resulting
direction costs — and find whether the inputs already differ. Predict the inputs
differ (the arithmetic between them is now verified line-by-line); fallback, if the
inputs match and the costs match, then the min-cost tie-break is the cause after
all and the `unordered_map`-vs-`BTreeMap` ordering becomes the target.

Probe cleanup: `BridgeDetector.cpp` is now in the injector's `LIBSLIC3R_FILES`, so
the submodule revert list must include it.

## R596 — the bridge anchor epsilon was 10,000x too small; branch divergence closed

R595 verified the arithmetic of `detect_bridging_direction` and eliminated
`BridgeDetector::detect_angle` (0 calls, both engines), leaving its INPUT as the
only candidate. `BRIDGEIN` dumps that input at the call site
(`LayerRegion.cpp:355` and `region_expansion.rs:1997`).

**Prediction CONFIRMED: the inputs differ, and they flip the algorithm branch.**
Matching bridges by area (identical to the unit on both engines):

| bridge area | pts | C anchors | C edges | R anchors | R edges |
|---|---|---|---|---|---|
| 107940309070 | 146 | 9 | **0** | 9 | **17** |
| 31367771108 | 71 | 9 | **0** | 9 | **14** |
| 92809844837 | 130 | 9 | **0** | 9 | **19** |
| 149528262781 | 164 | 10 | **0** | 10 | **46** |

**Anchor collection is correct** — the counts match exactly (9/9, 10/10). But C++
finds ZERO floating edges and takes the fully-anchored principal-components branch,
while we find 14-46 and take the cost branch. Different algorithm, hence R594's
90-degree swings.

**A probe-definition trap, caught before it was quoted.** The first reading showed
"anchors 9 vs 1" and looked like a smoking gun. It was not: C++ prints
`anchor_areas.size()` (raw polygons, pre-expand) while the Rust probe printed
`anchor_expolygons.len()` (post-`grow`, which unions overlaps into one). Corrected
to the pre-expand count, the anchors agree exactly. Same class as R584/R585 — two
quantities with the same name are not the same population.

**The defect: a units error of 1e5.** C++ is
`expand(anchor_areas, float(SCALED_EPSILON))` with `SCALED_EPSILON = scale_(1e-4) =
10` SCALED units. The port passed `0.001`, with a comment calling it "~1 micron in
mm" and a module header asserting this code "operates in mm (unscaled)". **The
header is wrong here** and the probe proves it: both engines print the same bridge
area `107940309070`, which is ~10.8 mm^2 only if the coordinates are scaled. So
`grow(anchors, 0.001)` expanded by 0.001 SCALED units — about 1e-8 mm, effectively
nothing — where C++ expands by 10. Bridge-outline edges lying exactly on the anchor
boundary therefore survived `diff_pl` and were classified as floating.

**Fixed behind `BRIDGE_ANCHOR_EPS_SCALED` (10.0), DEFAULT-ON.** Mechanism verified
directly: floating edges **17 -> 0, 14 -> 0, 19 -> 0**, now matching C++, and both
engines take the same branch.

    Benchy   gate ON is BYTE-IDENTICAL to gate OFF (a27419f0)
    Majora   matched lines 409,270 -> 409,276 (Bridge 1788 -> 1794); 16.25% both
    guards 8/8; Majora still SEMANTICALLY EQUIVALENT (material 0.9974, silhouette 99.54%)

Correct by construction and non-regressing, so DEFAULT-ON per R550/R558/R559.
**Re-baselined: majora `e8027b80` -> `bb313a93`; benchy `a27419f0` unchanged.**

**What this did NOT fix, stated plainly.** Benchy's G-code is byte-identical even
though the probe proves its computed bridge directions changed. So on Benchy the
angle produced here does not reach the output — the R595 reachability lesson
repeating one level down, and Benchy's Bridge feature (7.8%) is therefore still
unexplained. On Majora the angle IS consumed (the output moved), but the gain is 6
lines.

**Residual after the fix:** with both engines now on the PCA branch, the directions
are *near-antiparallel* rather than orthogonal — C++ `dir=(0.9775, 0.2107)`,
ours `(-0.9622, -0.2723)`, i.e. the same line mod 180 but off by ~3.6 degrees. That
remaining difference is inside `compute_principal_components`, not the branch
selection.

**R597:** (1) find what actually sets Benchy's Bridge angle, since this path
demonstrably does not — instrument the consumer, not the producer; (2)
`compute_principal_components` for the residual ~3.6 degrees. Predict (1) resolves
Benchy's Bridge 7.8% and (2) is a small refinement; fallback, if Benchy's bridge
surfaces never carry an angle at all, the fill direction comes from the infill
pattern default and the whole bridge line of enquiry is a dead end for that fixture.

## R597 — the consumer answers it: our ported `_infill_direction` is dead code

R595 and R596 each fixed producer code that turned out not to reach Benchy's output.
This round inverted the approach and instrumented the CONSUMER — `Fill::_infill_direction`
(`FillBase.cpp:224`), where the branch hinges on `surface->bridge_angle >= 0`.

**Prediction CONFIRMED on the C++ side.** Over its first 20 fills on Benchy:

    [FILLANG] n=20 used_bridge=0 used_layer=20 | surf_type=3 bridge_angle=-1.000000 out_angle=2.356194

C++ never uses a bridge angle on this fixture — every surface arrives with
`bridge_angle = -1` and the direction comes from the alternating layer angle. That
is exactly why R596's producer fix was byte-identical on Benchy, and it retires the
whole bridge-direction line of enquiry **for this fixture**. (The counters are
cumulative over the first 20 calls only; the probe prints `n<=20 || n%5000==0` and
no later line appeared, so totals beyond 20 are not established.)

**And the Rust probe printed NOTHING — a second reachability finding, this time on
our side.** `crate::fill::infill_direction` (`fill/mod.rs:1461`), the faithful port
of `_infill_direction` complete with the `bridge_angle >= 0` branch, is **never
called**. The real angle is chosen in `fill_rectilinear.rs:2318`:

    let angle_deg = if faithful_dir {
        config.angle + if layer_index & 1 == 1 { 90.0 } else { 0.0 } + 90.0
    } else {
        config.angle + config.angle_increment * layer_index as f64
    };

**Two concrete divergences follow, neither yet fixed:**

1. **`bridge_angle` is never consulted.** C++ short-circuits to the surface's bridge
   angle when it is set; our live path has no such branch at all. Inert on Benchy
   (C++ also always falls back there) but not on Majora, where R596 showed the angle
   IS consumed.
2. **The layer index is not divided by `thickness_layers`.** C++ is
   `_layer_angle(this->layer_id / surface->thickness_layers)`; ours uses
   `layer_index` directly. For any surface spanning more than one layer the parity
   flips, which changes the angle by exactly 90 degrees — the size of R594's
   observed Bridge swing (L47 rust 45 vs cpp 135). **That is a hypothesis with the
   right magnitude, not a finding: it needs `thickness_layers > 1` demonstrated on
   the affected surfaces before it is credited.**

There is also a stale comment at that site: it says "Gated; default keeps legacy",
but `faithful_gate` returns TRUE unless the variable is `"0"`, so `TOPFILL_FAITHFUL`
is DEFAULT-ON and the faithful branch is what actually runs. Worth correcting when
the code is next touched.

**Nothing was changed this round** — it is a pure measurement. Baselines reproduce
(`bb313a93`, `a27419f0`), 8/8 guards.

**R598:** (1) probe `thickness_layers` on the surfaces that reach
`fill_rectilinear`, on both engines, and test hypothesis 2 directly; (2) if it
holds, port the divisor and the `bridge_angle` short-circuit into the live path
behind one gate and A/B both fixtures. Predict the divisor matters for solid
surfaces over sparse infill (where `thickness_layers > 1`) and is inert elsewhere;
fallback, if `thickness_layers == 1` everywhere on both fixtures, the divisor is
cosmetic and the remaining Bridge/fill difference is the missing `bridge_angle`
branch plus `config.angle`/`angle_increment` themselves.

**Method note.** Three rounds running, the decisive fact came from asking "is this
code even called?" rather than from reading it. R586's rule (a silent probe is a
lead) has now produced R595, R596 and R597. The corollary worth keeping: **a
faithful-looking port can be dead code, and the live path may be an older
unfaithful one — check which one runs before comparing either against C++.**

## R598 — the divisor is cosmetic; and R597's headline was a prefix artefact

Two results, and the second corrects the previous round.

**1. Prediction WRONG; the pre-registered fallback fires.** I predicted
`thickness_layers > 1` on solid surfaces over sparse infill, which would make the
missing `layer_id / surface->thickness_layers` divisor a real 90-degree defect.
Measured at the consumer on both fixtures:

| fixture | calls | used_bridge | used_layer | **thickness_layers > 1** |
|---|---|---|---|---|
| Benchy | 500 | 16 | 484 | **0** |
| Majora | 38,000 | 538 | 37,462 | **0** |

`thickness_layers` is 1 for **every** surface on both fixtures. Reading the writer
confirms why: C++ sets it only in `PrintObject::combine_infill`
(`PrintObject.cpp:3679`, `templ.thickness_layers = layerms.size()`), for `stInternal`
surfaces, and only when infill-combining is enabled — which neither fixture uses.
**The divisor is COSMETIC here.** It remains a faithfulness defect worth fixing for
ask #1, but it cannot explain any observed difference on these two models, so it is
deprioritised rather than chased.

**2. R597's `used_bridge=0` is CORRECTED — it was a prefix artefact.** That round
printed only the first 20 calls and I flagged the limit at the time; carried to 500
and 38,000 calls, the counter is **not** zero:

    Benchy   16 of 500     (3.2%) use the surface's bridge angle
    Majora  538 of 38,000  (1.4%)

So C++ **does** consult `bridge_angle` on Benchy, on a small but non-empty set of
surfaces. R597's conclusion that the bridge-direction enquiry was "retired for
Benchy" is **withdrawn**: it is retired as the *dominant* explanation, not as a
contributor. This is exactly the failure mode R573/R584 warned about — a prefix is
not the population — and it cost a wrong conclusion one round later. **A counter
that reads zero over a truncated prefix is not a zero.**

**The live defect, now measured on both fixtures.** Our live fill path never
consults `bridge_angle` at all, so those 16 + 538 surfaces get the alternating layer
angle where C++ uses the detected bridge direction. The blocker is structural:

    C++   Fill::_infill_direction(const Surface *surface)      // has the surface
    Rust  generate_fill_rectilinear(fill_area, config, layer_index, is_grid)

**The Rust entry point never receives the `Surface`**, so no per-surface datum —
`bridge_angle` or `thickness_layers` — can reach the angle computation. That is a
single architectural divergence which produces both of R597's findings, and it is an
ask #1 (code-similarity) defect in its own right: a future maintainer following
`_infill_direction` from the C++ side lands on `fill/mod.rs:1461`, which is dead.

**Not changed this round, deliberately.** Threading the surface through is a real
signature change, and the angle handoff needs care — `InfillConfig.angle` is in
DEGREES while C++ `bridge_angle` is RADIANS, and the bridge branch must bypass both
the per-layer alternation and the unconditional `+90`. That deserves its own gated
A/B rather than being rushed at the end of a measurement round. Baselines reproduce
(`bb313a93`, `a27419f0`), 8/8 guards.

**R599:** thread the surface (or at least `bridge_angle`) to the live fill path and
implement the `bridge_angle >= 0` short-circuit behind one gate; A/B both fixtures.
Predict it moves Benchy's Bridge (7.8%) and Majora's Bridge (8.3%) and is inert
elsewhere, since only 1.4-3.2% of fills are affected. Fallback: if Bridge does not
move, the 16/538 surfaces are not the ones the Bridge FEATURE tag is emitted for,
and the mapping from surface type to feature label needs checking before any more
fill work.

## R599 — the `bridge_angle` short-circuit: faithful, structurally right, net negative

R598 left one measured defect: the live fill path never consults `bridge_angle`, so
16 (Benchy) and 538 (Majora) surfaces get the alternating layer angle where C++ uses
the detected bridge direction. This round implemented it.

**The missing hop was one field.** `params.bridge_angle` is already populated from
the surface (`fill.rs:606`, `surface.bridge_angle.unwrap_or(-1.0)`) and reaches the
caller — it simply was not passed into `InfillConfig`. Added `InfillConfig.bridge_angle`
(radians, -1 = none), populated at both call sites (`layer.rs:2487`, `:3323`), and
short-circuited on the live path exactly as `FillBase.cpp:224-239`: the bridge angle
REPLACES the base angle, SKIPS the per-layer alternation, and still takes the
unconditional `+M_PI/2`. Because `bridge_angle` is already in radians it bypasses the
degrees path rather than round-tripping through it.

**Gate OFF reproduces both baselines byte-for-byte** (`a27419f0`, `bb313a93`).

**Gate ON — mixed, and net negative on the primary measure:**

| | Benchy OFF | Benchy ON | Majora OFF | Majora ON |
|---|---|---|---|---|
| matched lines | 115,887 | **115,370** | 409,276 | **408,965** |
| rate | 74.99% | 75.04% | 16.25% | 16.24% |
| Bridge matched | 121 | 105 | 1,794 | **1,816** |
| Bridge rate | 7.8% | 7.8% | 8.3% | **8.7%** |
| Bridge rust lines | 1,554 | **1,353** | 21,578 | **20,847** |
| Internal solid infill | 24.2% | **21.9%** | 11.3% | 11.3% |

**Beware the rate: Benchy's 74.99% -> 75.04% is a shrinking denominator, not more
agreement.** Absolute matched lines fell by 517; the body line count fell from
154,539 to 153,752 while C++ has 156,342, so the line-count gap actually widened
(1.15% -> 1.66%). Quoting the percentage alone would have made a regression look like
a win.

**What genuinely improved:** the Bridge toolpath is structurally closer to C++ on
both fixtures — Benchy 1,554 -> 1,353 lines against C++'s 1,337, Majora 21,578 ->
20,847 — and Majora's Bridge match rate rose 8.3% -> 8.7% (+22 lines). So the
short-circuit is doing the right thing to bridges.

**What regressed, and why it matters:** Benchy's Internal solid infill fell 24.2% ->
21.9%. **That means the surfaces carrying `bridge_angle >= 0` are NOT confined to
the `Bridge` feature** — the pre-registered fallback from R598/R599 fires in part.
Changing their angle helps the bridges and hurts the solid infill that shares the
flag, and the net is negative on both fixtures.

**Shipped OPT-IN (`FILL_BRIDGE_ANGLE`, default OFF) per R557**, with the measurement
recorded beside the code. Baselines unchanged, 8/8 guards. It is faithful to
`FillBase.cpp` and should become default-on once the surfaces it touches are
correctly partitioned — it is currently a correct change applied to a
partly-wrong population.

**Prediction: PARTLY right.** Majora's Bridge moved as predicted (8.3% -> 8.7%).
Benchy's Bridge *rate* did not move at all (7.8% both), and the claim that it would
be "inert elsewhere" was **wrong** — Internal solid infill moved by 2.3 points.

Also fixed in passing: the stale comment at `fill_rectilinear.rs:2312` claiming
"Gated; default keeps legacy". `faithful_gate` returns true unless the variable is
`"0"`, so `TOPFILL_FAITHFUL` is DEFAULT-ON and the faithful branch is what runs.

**R600:** find why non-Bridge surfaces carry `bridge_angle >= 0` — dump
`(surface_type, bridge_angle, feature label)` together on both engines. Predict C++
sets the angle on the same surfaces we do but its `stBottomBridge` -> feature-label
mapping differs, so the same flag reaches different G-code tags; fallback, if the
flag really is set on different surfaces, the defect is upstream in
`expand_merge_surfaces`/surface classification, not in the filler.
## R600 — the wrong fallback: `bridge_angle` was 0.0, not -1, on every non-bridge surface

R599 shipped the `bridge_angle >= 0` short-circuit opt-in because it regressed both
fixtures, and inferred that "the surfaces carrying `bridge_angle >= 0` are NOT
confined to the `Bridge` feature". That inference was right. The cause is one token.

**C++** (`Fill.cpp:243`) is a straight copy with no fallback:

```cpp
params.bridge_angle = float(surface.bridge_angle);
```

It needs no fallback because `Surface::bridge_angle` is already `-1` when undefined
("in radians, ccw, 0 = East, only 0+ (negative means undefined)", `Surface.hpp:39`,
initialised `bridge_angle(-1)` in every constructor).

**Rust** (`fill/mod.rs:844`, the LIVE `group_fills`) substituted a config value:

```rust
bridge_angle: surface.bridge_angle.unwrap_or(region_config.bridge_angle) as f32,
```

Two defects in one token:

* `region_config.bridge_angle` **defaults to 0.0**, and `0.0 >= 0` is TRUE — so every
  non-bridge surface presented itself to `Fill::_infill_direction` as a bridge.
* it is in **degrees** ("Bridge angle (degrees). 0 = auto") while `surface.bridge_angle`
  is in **radians**. Even a non-zero config value would have been misread.

**The census, measured rather than argued.** `FILL_BRIDGE_POP=1` counts the same
quantity as R598's C++ `FILLANG` probe — of the fills reaching this path, how many
present a usable bridge angle — and prints every call, so it cannot read as a prefix
artefact (R598):

| fallback | Benchy | Majora |
|---|---|---|
| `region_config.bridge_angle` (old) | **314 / 314 = 100.00%** | **1086 / 1086 = 100.00%** |
| `-1.0` (C++ faithful) | **11 / 314 = 3.50%** | **353 / 1086 = 32.5%** |

C++ measured 3.2% on Benchy (R598). The denominators are not the same population —
C++'s probe sits in `_infill_direction`, which serves every fill type, ours on the
rectilinear path — so this corroborates the magnitude, it is not an identity (R572).
What it does establish exactly is the intra-engine ratio: 100% -> 3.5%.

**`FILL_PARAMS_BRIDGE_ANGLE_CPP`, DEFAULT-ON.** With `FILL_BRIDGE_ANGLE` off the fix
is byte-identical on both fixtures (`a27419f0`, `bb313a93`) — as predicted, because
the only other reader is the `Ord` at `fill/mod.rs:439`, where a uniform 0.0 and a
uniform -1.0 sort identically, and a real bridge cannot tie with a non-bridge across
it (`bridge` is itself a sort key).

**`FILL_BRIDGE_ANGLE` flipped DEFAULT-ON.** With the population corrected, R599's
regression inverts on both fixtures and both instruments:

| | Benchy OFF | Benchy ON | Majora OFF | Majora ON |
|---|---|---|---|---|
| matched (line_parity) | 115,887 | **115,900** | 409,297 | **409,318** |
| matched (line_align) | 84,552 | **84,580** | — | — |
| body lines / gap | 154,539 / 1.15% | 154,472 / 1.20% | 2,518,612 / 9.47% | 2,517,813 / 9.50% |
| Bridge rate | 7.8% | **8.2%** | 8.3% | **8.7%** |
| Bridge rust lines | 1,554 | **1,468** (C++ 1,337) | 21,578 | **20,841** (C++ 13,195) |
| Internal solid infill | 24.2% | **24.2%** | 11.3% | 11.3% |

R599's Internal-solid-infill regression (24.2% -> 21.9%) is gone. R599's Bridge line
count of 1,353 looked closer to C++'s 1,337 than R600's 1,468, but it was an artefact
of applying the bridge angle to *every* fill; the honest figure is 1,468.

**The gains are small and not uniformly positive, and that is worth stating plainly.**
+13 lines on Benchy out of 115,900, +21 on Majora out of 409,318. Majora's per-feature
deltas sum to the +21 exactly — Bridge +17, Outer wall +15, Sparse infill +14,
Internal solid +2, Inner wall +1, Prime tower +1, **Floating vertical shell -20, Top
surface -9**. Two features got worse. The change ships default-on anyway because it is
verbatim `FillBase.cpp:224-239` and net positive on both fixtures under both
instruments; the two losers become the next question, not a reason to hold it back.

**Re-baselined**, diff proven intentional: benchy `a27419f0` -> `304320a6`, majora
`bb313a93` -> `529545af`. 8/8 guards; both fixtures still pass all five
`semantic_compare.py` gates.

**Cube baseline correction, not caused by this round.** The cube reads `242f1fb8`
against a recorded baseline of `ab415621`. It is identical under all three gate
combinations — default, `FILL_BRIDGE_ANGLE=0`, and both gates off — and "both gates
off" reproduces pre-R600 behaviour exactly, so R600 is provably inert on the cube and
the drift predates it. Most likely R596, which shipped a default-on change and
re-baselined only majora. Cube baseline corrected to `242f1fb8`; the lesson is that a
re-baseline must cover every fixture, not just the ones the round was aimed at.

**Prediction: RIGHT on every clause.** Byte-identical with the short-circuit off;
total matched above 115,887 on Benchy; Internal solid infill back to 24.2%. The one
thing it got wrong in emphasis was expecting the Bridge line count to stay at R599's
1,353 — it should not have, and 1,468 is the correct value.

**Also corrected:** R599's comment at `layer.rs:2487`/`:3323` cited `fill.rs:606` as
the source of `params.bridge_angle`. `fill.rs` is the DEAD twin module (its
`group_fills` at `fill.rs:467` is shadowed by `fill/mod.rs:553`, which is what
`crate::fill::group_fills` resolves to) — and, tellingly, the dead module had the
fallback RIGHT (`unwrap_or(-1.0)`). The R597 dead-code trap, hit a second time:
reading the faithful-looking twin is what made this defect invisible for a round.

**R601:** Majora's Floating vertical shell (-20) and Top surface (-9) under the bridge
angle. Both are large features on Majora (394k and 24k rust lines) and both *gained*
nothing while Bridge gained; predict they contain surfaces flagged `stBottomBridge`
that C++ labels differently, i.e. the R600 fix corrected the VALUE but the surface
POPULATION carrying it still differs — the R599/R600 fallback about
`expand_merge_surfaces`/surface classification (`LayerRegion.cpp:600-620`) is now the
live hypothesis. Fallback: if the populations match, the loss is the raster phase
changing under a new angle, and belongs with the existing Internal-solid-infill
sub-lattice scatter rather than with bridges.
## R601 — the tool change was under the wrong `; FEATURE:` tag: +221,561 lines

R600 left two Majora features losing ground under the corrected bridge angle:
Floating vertical shell (-20) and Top surface (-9). Both were also emitting far more
lines than C++ (1.34x and 1.54x). This round chased that excess and found something
much larger underneath it.

**Prediction: WRONG. Fallback: half right.** The prediction was that we classify more
area into those features, so extruded LENGTH would track the line ratio. It does not —
Majora's per-feature length is at parity almost everywhere (Floating vertical shell
1.014x, Internal solid infill 1.002x, Sparse infill 1.002x, Outer wall 0.992x). The
registered fallback said "fragmentation: more and shorter segments", and that is also
not it: deriving segment counts from length / mean-segment, Internal solid infill has
**0.91x** the extruding segments while carrying **1.46x** the lines.

**The divergence is in the lines that do not extrude.** A new instrument
(`$D/line_kind.py`) classifies every body line as extrude / travel / retract / zmove /
setfeed / gother / mcode / comment and histograms it per feature:

| kind | Floating vert shell | Internal solid | Top surface | Prime tower |
|---|---|---|---|---|
| extrude | 1.040 | 0.907 | 1.110 | 0.951 |
| travel | 2.087 | 2.011 | 1.800 | **0.174** |
| retract | 4.692 | 4.739 | 4.222 | **0.118** |
| mcode | **334.6** | **352.2** | 2.470 | **0.002** |
| gother | 60.9 | 69.7 | inf | **0.113** |

Extrusion is at parity; everything else is inflated in the object features and
collapsed in the prime tower. Comparing gross per-feature movement against the net
file-level difference separates redistribution from generation:

| kind | file net delta | sum per-feature abs delta | redistributed | ratio |
|---|---|---|---|---|
| extrude | 104,126 | 122,524 | 18,398 | 1.2x |
| retract | 15,404 | 162,226 | **146,822** | **10.5x** |
| mcode | 39,909 | 229,817 | **189,908** | **5.8x** |
| travel | 90,309 | 195,163 | 104,854 | 2.2x |

Extruding moves are attributed almost consistently (1.2x). ~190k M-codes and ~147k
retracts exist in both files and merely sit under different tags.

**Where.** Of C++'s 16,347 `M620`/`M621` tool-change commands, **16,341 are inside
`; FEATURE: Prime tower`**. We emitted **zero** there, spraying them across Internal
solid infill (6,930), Floating vertical shell (4,812), Sparse infill (3,402), Outer
wall (444) and Top surface (282). The tool-change COUNT was already right (16,335 vs
16,347, 0.07% apart) — only the tag was wrong.

**This was a known, deliberate divergence.** `print.rs:3039-3043` said so: "C++ emits
the trio at the HEAD of `tcr.gcode`, i.e. BEFORE the change-filament block... moving
the whole trio upstream is a separate change with per-feature-attribution
consequences." R601 measured those consequences. R464's stated reason for diverging
does not survive scrutiny: the trio is three COMMENTS and cannot change any extrusion.
The "outstanding retraction at the first purge stroke" it blamed was the missing
toolchange unretract, fixed independently in R466.

C++ emits the trio immediately before `;----` / `; CP TOOLCHANGE START`. **Gate
`TOWER_FEATURE_TAG_HEAD`, DEFAULT-ON**: the three comment lines move to the head of
the tower block; the trailer keeps everything that is a real move (filament start
template, travel, unretract).

**Result — the largest single parity gain of the campaign:**

| | before | after |
|---|---|---|
| **Majora matched lines** | 409,318 | **630,879 (+221,561)** |
| **Majora rate** | 16.26% | **25.06%** |
| body lines / gap | 2,517,813 / 9.50% | 2,517,813 / 9.50% (unchanged) |

Body-line count is *identical* — the fix moves lines between groups without creating
or destroying any, which is exactly what an attribution repair should do. Tool changes
now land where C++ puts them: **Prime tower 16,331 / Custom 4**, against C++'s
16,341 / 6.

Every feature improved, and the line-count ratios normalised:

| feature | matched | rate | rust/cpp lines |
|---|---|---|---|
| Prime tower | 35,749 -> **257,400** | 26.9% -> **61.0%** | 0.262 -> 0.832 |
| Internal solid infill | 36,944 -> 36,946 | 11.3% -> **18.3%** | 1.462 -> **0.905** |
| Floating vertical shell | 42,063 -> 42,043 | 10.7% -> **13.5%** | 1.336 -> **1.053** |
| Sparse infill | 53,079 -> 53,093 | 19.4% -> **24.8%** | 1.137 -> 0.889 |
| Top surface | 2,515 -> 2,498 | 10.5% -> **13.0%** | 1.543 -> **1.234** |
| Bridge | 1,794 -> 1,811 | 8.3% -> **11.2%** | 1.579 -> 1.228 |
| Outer wall | 165,767 -> 165,721 | 19.0% -> **19.2%** | 0.918 -> 0.910 |

Note the object features' matched COUNTS barely move — the rate rises because the
mis-attributed machinery left their denominators. That is the correct reading: those
lines were never theirs.

**Benchy is byte-identical either way** (`304320a6`) — single-material, no wipe tower,
exactly as predicted. Gate OFF reproduces `529545af`. Cube unchanged (`242f1fb8`).
8/8 guards.

**Re-baselined**, diff proven intentional: majora `529545af` -> `14ff4542`. Benchy
`304320a6` and cube `242f1fb8` unchanged.

**A caution about the primary metric.** `line_parity.py` groups by (layer, feature), so
a mis-tagged line can never match even when byte-identical. Majora's 16.26% was
therefore partly an artefact of OUR tagging, not a statement about geometry. The
instrument was not wrong — the G-code really did carry the wrong tags, and C++ is the
reference — but it means a per-feature rate is only meaningful once attribution is
verified. Worth checking the same way before trusting any future per-feature number.

**R602:** Prime tower is now the largest single residual at 61.0% (257,400/422,007;
C++ 506,960). Its post-fix line-kind census, measured this round, shows every kind
back in a sane band (was 0.002x-0.18x) but still short by 84,953 lines:

| kind | rust | cpp | delta | x |
|---|---|---|---|---|
| extrude | 64,449 | 67,794 | -3,345 | 0.951 |
| travel | 101,559 | 109,559 | -8,000 | 0.927 |
| retract | 91,571 | 97,635 | -6,064 | 0.938 |
| **zmove** | 10,878 | 20,493 | **-9,615** | **0.531** |
| **gother** | 15,512 | 26,387 | **-10,875** | **0.588** |
| **mcode** | 66,330 | 93,179 | **-26,849** | **0.712** |
| **comment** | 62,889 | 80,516 | **-17,627** | **0.781** |
| setfeed | 6,092 | 8,674 | -2,582 | 0.702 |

The moves (extrude/travel/retract) are within 5-7%; the deficit is concentrated in
mcode, comment, gother and zmove. Predict the missing lines are a template body we
evaluate to less output than C++ does — the `filament_end_gcode`/`filament_start_gcode`
and ramming blocks — rather than absent geometry, since the tower's extrusion count and
total length already match (0.951x / 0.995x). Fallback: if the templates expand
identically, the deficit is in per-toolchange fixed preamble C++ writes that we skip,
and should be found by diffing one complete tool-change block line-for-line.
## R602 — the tool-change header: `M220` never emitted, and a counter that never counted

R601 left the prime tower as the largest single residual (61.0%, 84,953 lines short
of C++) with the deficit concentrated in mcode / comment / gother / zmove rather than
in moves. This round found three defects in the tool-change header.

**Prediction: RIGHT.** The registered prediction was that the bulk is real emissions we
skip rather than template expansion, because the tower's extrude count and length
already matched (0.951x / 0.995x). File-wide counts confirmed it immediately:

| construct | C++ | Rust | deficit |
|---|---|---|---|
| `; WIPE_START` blocks | 44,174 | 36,394 | 7,780 |
| `M73` | 9,179 | 660 | 8,519 |
| `M220` | 8,171 | **2** | **8,169** |

**Defect 1 — `speed_override` was defined but never called.** `WipeTowerWriter::speed_override`
existed in the port; nothing invoked it, and `speed_override_backup` / `speed_override_restore`
(WipeTower.cpp:1156-1172) had never been ported at all. C++ brackets every tower block:

```
speed_override_backup();   // M220 B
speed_override(100);       // M220 S100
   ... tower ...
speed_override_restore();  // M220 R
```

**Defect 2 — the header comments were wrong.** C++ writes (WipeTower.cpp:3272-3281,
mirrored at :2068-2077):

```
;--------------------
; CP TOOLCHANGE START
; toolchange #N
; material : <old> -> <new>
;--------------------
M220 B
M220 S100
; WIPE_TOWER_START
```

We wrote `; Tool change from Tx to Ty` — a line C++ never emits — and none of the rest.

**Defect 3 — `num_tool_changes` was never incremented.** The field was reset and read
but never advanced anywhere in the port (C++ does `++m_num_tool_changes` at
WipeTower.cpp:3329 and :2162), so once defect 2 was fixed every block printed
`; toolchange #1`. Fixed unconditionally, which is byte-neutral with the header gate
off because the counter's only other consumer, `get_number_of_toolchanges`, is dead.

**`WT_TOOLCHANGE_HEADER_CPP`, DEFAULT-ON.**

**Result:**

| | R601 | R602 |
|---|---|---|
| Majora matched | 630,879 | **646,455 (+15,576)** |
| Majora rate | 25.06% | **25.54%** |
| body lines / gap | 2,517,813 / 9.50% | 2,531,418 / **9.01%** |
| Prime tower | 61.0% | **62.7%** (272,976/435,612) |
| `M220` lines | 2 | **8,165** (C++ 8,171) |

The `M220` count lands 6 short of C++, and that is exactly right: 3 lines per tool
change on both engines, and C++ has 2 more tool changes than we do (2,723 vs 2,721) —
a pre-existing count difference, not a new one. The prediction of +10,000 to +20,000
matched lines held (+13,606 before the counter fix, +15,576 after).

**Benchy and cube are byte-identical** (`304320a6`, `242f1fb8`) — neither has a wipe
tower. Gate OFF reproduces `14ff4542`. 8/8 guards.

**Re-baselined**, diff proven intentional: majora `14ff4542` -> `2c763932`.

**The `M620.1 X0 Y0` lead was measured and dropped.** `change_filament.rs:105-123`
stubs `travel_point_1/2/3_x/y` to zero with a comment saying they matter "only when
`toolchange_count == 2`". That comment is CORRECT: C++'s whole Majora output contains
exactly **3** `M620.1 X` lines, all in the first tool change. Three cosmetic lines is
not worth a round. Recorded here so it is not re-opened — and as a counterweight to
R601's lesson: a documented deferral is worth re-reading, but some of them really are
as small as they claim.

**R603:** `M73`. Subtype breakdown against C++:

| subtype | C++ | Rust |
|---|---|---|
| `M73 L` (layer) | 656 | **656** — already correct |
| `M73 E` | 2,723 | **0** |
| `M73 P` (progress) | 5,798 | 2 |

`M73 E` is emitted once per tool change and its values descend 2722 -> 0, i.e. it is a
countdown of REMAINING tool changes — deterministic, no time estimation needed, and
worth 2,723 lines. Predict it is emitted from the same tool-change site as this round's
header and lands as a near-exact +2,723. Fallback: if the values do not match a plain
countdown, they encode remaining time or filament and need the estimator, in which case
they belong with `M73 P` as one larger piece of work. `M73 P` (5,798 lines) needs the
time-estimation post-process and should be scoped separately.
## R603 — `M73 E`: the countdown, and a gain computed exactly before writing any code

R602 left three `M73` subtypes measured: `M73 L` already exact at 656, `M73 E` at
C++ 2,723 vs our **0**, `M73 P` at C++ 5,798 vs our 2. This round did `M73 E`.

**The whole value of the round was decided before any code was written.** C++'s
countdown is `total_filament_change - filament_change_num` (GCodeProcessor.cpp:601),
inserted at each filament-block boundary (:1119). C++'s total is **2,723**; ours is
**2,721**. So our change #1 would emit `E2720` where C++ emits `E2722` — and whether
those lines ever match depends entirely on WHERE C++'s two extra tool changes sit:

* extras at the START -> our change *k* pairs with C++'s *k+2*, values coincide, all match
* extras at the END -> every line is off by 2, nothing matches

Neither, as it turned out. Comparing the per-layer tool-change sequences directly:
the first divergence is at tool-change index **509** (C++ layer 142, ours 143), and
the realignment is not a clean shift — C++ has 6 changes on layer 142 where we have 5,
with the second extra elsewhere.

That made the exact gain computable without building anything. Our change *k* emits
`2721-k` at layer `r[k-1]`; it matches iff C++'s change *k+2* (same value) is on the
same layer:

| countdown total used | predicted matches |
|---|---|
| **ours (2,721)** — what C++'s formula yields on our gcode | **2,304 / 2,721 (84.7%)** |
| C++'s (2,723) — forced | 1,971 / 2,721 |

So the faithful choice is also the better-scoring one, which is worth stating: using
our own total is not a compromise.

**`M73_REMAIN_FILAMENT_CHANGES`, DEFAULT-ON.** C++ inserts these in a whole-file
post-process; we have no such pass, so the line is spliced into the substituted tower
block immediately after the bare `T<n>` command — textually the same position,
verified against `cpp_majora_new.gcode:6930-6931` (`T4` then `M73 E2722`). Our output
now reads `T3` / `M73 E2720` / `M620.1`, the same shape.

The total is taken from the tower plan (`wipe_tower_results` with `is_tool_change`)
because it is needed UP FRONT and we stream rather than re-walk. It came out at
exactly 2,721 — the emitted count — confirmed by the first and last emitted values
being `E2720` and `E0`.

**Result — the prediction landed to the line:**

| | R602 | R603 |
|---|---|---|
| Majora matched | 646,455 | **648,759 (+2,304)** |
| Majora rate | 25.54% | **25.60%** |
| body lines / gap | 2,531,418 / 9.01% | 2,534,139 / **8.91%** |
| Prime tower | 62.7% | **62.8%** (275,280/438,333) |
| `M73 E` | 0 | **2,721** (C++ 2,723) |

+2,721 lines emitted, +2,304 matched, 417 unmatched — and the 417 are precisely the
changes sitting on a different layer than C++'s, i.e. the pre-existing
2-tool-change difference, not something this emission introduced.

**Benchy and cube byte-identical** (`304320a6`, `242f1fb8`). Gate OFF reproduces
`2c763932`. 8/8 guards. **Re-baselined**: majora `2c763932` -> `56938d4d`.

**Method note worth keeping.** R602's lesson was that a bare `grep -c` beats anything
clever. R603 extends it: the two engines' *event sequences* (here, which layer each
tool change falls on) were enough to predict the exact matched-line gain — 2,304,
correct to the line — before a single edit. When a change emits one line per event,
compare the event sequences first; the answer is arithmetic, not experiment.

**R604:** the `; WIPE_START` deficit — C++ 44,174 blocks vs our 36,394, **7,780 short**,
still unattributed to a site. Each block is ~6 lines (`; WIPE_START`, `G1 F<n>`,
several `G1 X.. Y.. E-..` retract moves, `; WIPE_END`), so this is worth ~45k lines —
the largest single remaining item found so far. Checked and ruled out this round: the
tag TEXT is right (`grep -c "TYPE:Wipe_Start"` is **0 on both engines**, so
`exporter.rs:2607`'s `; TYPE:Wipe_Start` string is not on this path and is not
corrupting the count). Predict the deficit is a missing wipe-on-retract at some class
of retraction — most likely the object-side retracts rather than the tower's, since
the tower's own moves are already at parity. Fallback: if the wipe blocks are present
but shorter, it is the per-wipe move COUNT (C++ splits the wipe path into more
segments), which is a different fix in the same place.
`M73 P` (5,798 lines) still needs the time-estimation post-process — scope separately.
## R604 — the `; WIPE_START` deficit: two missing install sites, and a decomposition

R603 left `; WIPE_START` as the largest unattributed item: C++ 44,174 blocks vs our
36,394, **7,780 short**, roughly 45,000 lines. This round is measurement only — no
behavioural change, working tree clean, all three baselines trivially intact.

**Prediction WRONG, and the registered fallback WRONG too.** The prediction was "a
missing wipe-on-retract for some CLASS of retraction, most likely object-side"; the
fallback was "the blocks are present but shorter". Neither. It is two overlapping
effects, and the one I named second is the smaller.

**The distribution is structured, not uniform:**

| feature | cpp | rust | delta | x |
|---|---|---|---|---|
| Outer wall | 16,649 | 14,544 | -2,105 | 0.874 |
| Sparse infill | 7,175 | 5,101 | -2,074 | 0.711 |
| **Prime tower** | 6,161 | 3,371 | **-2,790** | **0.547** |
| Inner wall | 5,110 | 4,996 | -114 | 0.978 |
| Floating vertical shell | 3,001 | 2,812 | -189 | 0.937 |
| **(pre-feature)** | 655 | **0** | -655 | **0.000** |
| **Overhang wall** | 141 | **0** | -141 | **0.000** |
| Internal solid infill | 4,742 | 4,861 | +119 | 1.025 |
| Top surface | 243 | 391 | +148 | 1.609 |
| Bridge | 285 | 310 | +25 | 1.088 |

Two features emit **exactly zero** wipes (796 blocks), three emit MORE than C++, and
the bulk is a partial deficit at three different ratios. That is a condition firing
differently, not an absent feature.

**The deficit decomposes cleanly.** C++ performs 168,246 retractions to our 158,288 —
we retract 9,958 fewer times — and C++ wipes on 26.26% of them against our 22.99%:

| term | blocks |
|---|---|
| fewer retractions | 2,290 |
| lower wipe-per-retract rate | 5,490 |
| **total** | **7,780** |

The identity closes exactly. **The wipe RATE is the larger term**, which is why the
prediction's "missing class of retraction" framing was wrong: we do wipe on most
retractions, just less often, and we also retract less often.

**The mechanism: C++ installs the wipe path at FOUR sites, we install at TWO.**

| C++ site | function | Rust |
|---|---|---|
| `GCode.cpp:5601` | `GCode::extrude_loop` | present, `exporter.rs:653` |
| `GCode.cpp:5704` | `GCode::extrude_path` | present, `exporter.rs:1750` |
| **`GCode.cpp:5664`** | **`GCode::extrude_multi_path`** | **MISSING** |
| **`GCode.cpp:774` / `:1084`** | wipe-tower / nozzle-change append | **MISSING** |

`; WIPE_START` is emitted (`exporter.rs:2607`) only when the stored wipe path is
non-empty, so every site that fails to install one silently converts a wipe into a
bare retract.

**The `extrude_multi_path` gap is a documented conflation.** `exporter.rs:1178` reads
`pub use extrude_collection as extrude_multi_path;` — "Alias for backward
compatibility (C++ has both multi_path and collections)". The two are not
interchangeable for this purpose: C++'s `extrude_multi_path` builds ONE wipe path from
the concatenation of all sub-paths (skipping duplicate joints) and reverses it, while
our collection handler installs a wipe path per sub-path, so what survives is only the
LAST sub-path. Another deliberate divergence found by grepping for one (R601's rule),
and this time it is not the 3-line kind.

**Not fixed this round, deliberately.** Both remaining sites are real ports, not
one-line splices, and the `extrude_multi_path` fix changes wipe-path CONTENT for the
36,394 wipes we already emit — not just the count. That needs its own gated A/B rather
than being rushed at the end of a measurement round. No code changed; `git status`
clean, baselines `304320a6` / `56938d4d` / `242f1fb8` unaffected.

**R605:** port `GCode::extrude_multi_path`'s wipe-path installation
(`GCode.cpp:5664-5673`) — concatenate all sub-paths skipping duplicate joints, then
reverse — behind its own gate. Predict it lifts the wipe RATE toward C++'s 26.26% and
recovers a large part of the 5,490-block rate term, with the Outer wall (0.874x),
Sparse infill (0.711x) and Overhang wall (0.000x) deficits moving most, since those are
where multi-paths dominate. Fallback: if the rate barely moves, the wipe path is being
installed but emptied downstream (the `clip_end` at `exporter.rs:2581` or the
`wipe_dist` guard), and the next probe belongs at `Wipe::wipe`'s entry rather than at
its callers. The Prime tower term (-2,790) is the separate tower/nozzle-change site
(`GCode.cpp:774`/`:1084`) and should be its own round.
## R605 — `extrude_multi_path`'s wipe path: ported, reachable, and parity-neutral

R604 found that C++ installs the wipe path at four sites and we install at two, and
named `GCode::extrude_multi_path` (`GCode.cpp:5664-5673`) as the tractable half. This
round ported it.

**The type distinction does not exist here, so the port does not guess.** Our
`ExtrusionEntityType` has only `Path`, `Loop`, `Collection` — no `MultiPath` variant —
and `exporter.rs:1178` aliases `extrude_collection as extrude_multi_path` "for backward
compatibility". Dispatching each sub-path through `extrude_entity` installs a wipe path
PER sub-path, so only the LAST survives; C++ keeps the whole concatenation, joints
de-duplicated, reversed. The port therefore fires only on the shape a C++
`ExtrusionMultiPath` actually takes here — a collection whose children are ALL paths,
which is what `thick_polyline_to_multi_path` produces for Arachne variable-width walls
and gap fill. Mixed collections are left alone, because C++ routes those through
`extrude_collection` instead.

**Reachability, checked before arguing about effect (R595).** A `WIPE_MULTIPATH_POP`
census shows the branch firing **468 times on Benchy and 11,175 on Majora**. It is not
dead code, and all three fixture hashes move when it is on.

**Prediction: RIGHT on the mechanism, WRONG on the benefit.** The prediction was that
the wipe COUNT would move little because both the old and new stored paths are
non-empty, and that the gain would come from existing wipes matching C++ geometrically
(+500 to +5,000 on Majora). The first half held exactly — the wipe count is
**unchanged on both fixtures** (Benchy 2,041, Majora 36,394). The second half did not.

| | Benchy OFF | Benchy ON | Majora OFF | Majora ON |
|---|---|---|---|---|
| matched lines | 115,900 | **115,910 (+10)** | 648,759 | **648,741 (-18)** |
| body lines | 154,472 | 154,603 | 2,534,139 | 2,542,135 |
| line-count gap | 1.20% | **1.11%** | 8.91% | **8.62%** |
| `; WIPE_START` | 2,041 | 2,041 | 36,394 | 36,394 |

Net across both fixtures: **-8 matched lines out of 764,659**, i.e. 0.001% — neutral,
and fractionally negative on the larger fixture.

**Shipped OPT-IN (`WIPE_MULTIPATH_CPP`, default OFF) per R557/R595/R599.** This is the
same shape as R595 — faithful, reachable, tiny mixed effect, slightly negative on
Majora — and gets the same disposition for consistency.

**The gap improvement is real but is not evidence of correctness.** 8.91% -> 8.62% is
the largest single gap improvement of the session, and we move TOWARD C++'s line count
without overshooting (2,542,135 vs C++'s 2,781,977). But the gap metric rewards
emitting lines whether or not they match: 8,127 body lines were added and essentially
none of them matched. Quoting the gap alone here would be the R599 error wearing a
different hat. The honest summary is: structurally closer, numerically no better.

**What would make it net-positive.** The wipe PATH now matches C++; the emitted
`G1 X.. Y.. E-..` moves depend on more than the path — `wipe_dist`, the retract length
being repaid, and the wipe speed (`exporter.rs:2588-2660`). If those differ, a correct
path still yields non-matching lines, which is exactly the pattern observed. That is
the next thing to check, and it is a smaller, sharper question than the one this round
started with.

**Baselines unchanged** — the gate is default-OFF, so `304320a6` / `56938d4d` /
`242f1fb8` all reproduce. 8/8 guards. Gate ON was also verified against
`semantic_compare.py` so the change is not semantically harmful even when enabled.

**R606:** the wipe MOVE VALUES. With `WIPE_MULTIPATH_CPP=1` the wipe paths match C++
but the moves do not, so compare, for one wipe block on each engine, the emitted
`G1 X Y E` triples and the preceding `G1 F` — checking `wipe_dist`
(`exporter.rs:2597`), the `0.95` dE factor (`GCode.cpp:5622`) and the wipe speed
against `GCode.cpp:402-416`. Predict a scalar difference (wipe_dist or the dE factor)
rather than a geometric one, since the paths are now identical by construction.
Fallback: if the moves differ geometrically too, the stored path is being consumed
differently — check `clip_end` at `exporter.rs:2581` against C++'s
`wipe_path.clip_end(wipe_path.length() - wipe_dist)`. The other three open wipe items
are unchanged: the tower/nozzle-change install (`GCode.cpp:774`/`:1084`, Prime tower
-2,790), the 9,958 missing retractions, and `M73 P` (5,798 lines).
## R606 — the wipe move values: `_toolchange` is an unused parameter

R605 made the wipe PATH C++-faithful yet bought nothing, so this round asked what turns
a path into `G1 X Y E` lines. Measurement only — no code changed, `git status` clean,
all three baselines intact.

**A wrong turn first, recorded because it nearly stuck.** The opening move was
`grep -A9 -m3 "; WIPE_START"` on both engines, which showed C++ emitting 5 moves per
block against our 3 and, in one case, 1. That looked like a systematic segmentation
defect. It was not: the first three blocks in file order are **not the same wipes**, and
the full distribution on Benchy is at parity — C++ 2,029 blocks / 5,140 moves / mean
2.53, Rust 2,041 / 5,068 / mean 2.48, with near-identical histograms. Comparing an
unmatched pairing (R573/R593) and reading three samples as a distribution. Withdrawn.

**Benchy is at parity; Majora is not — and that is the useful signal.**

| | blocks | wipe moves | mean segs/block |
|---|---|---|---|
| Benchy C++ | 2,029 | 5,140 | 2.53 |
| Benchy Rust | 2,041 | 5,068 | 2.48 |
| **Majora C++** | 44,174 | **123,515** | **2.80** |
| **Majora Rust** | 36,394 | **67,763** | **1.86** |

So the real Majora deficit is **55,752 wipe MOVES**, not the 7,780 BLOCKS R604 measured —
R604's decomposition was about blocks only and understated the item by 7x.

**The vertex-density hypothesis was tested and REFUTED.** Coarser toolpaths would leave
fewer vertices inside `wipe_dist`. Majora's walls are coarser (Outer wall segment length
1.133x, Inner 1.108x, Internal solid 1.105x) and Benchy's are not (1.002x), which fits —
but **Sparse infill has segment length at parity (1.004x) with wipe moves/block at
0.48x**, and **Prime tower is coarser (1.046x) yet emits MORE wipe moves (1.28x)**. The
correlation does not hold; density is not the mechanism.

**What the E-per-block census settled.** Summing |E| over each wipe block:

| feature | cpp mm/blk | rust mm/blk | x | cpp E/blk | rust E/blk | x |
|---|---|---|---|---|---|---|
| Outer wall | 2.948 | 3.301 | 1.120 | 0.7600 | 0.7600 | **1.000** |
| Inner wall | 2.041 | 1.298 | 0.636 | 0.7600 | 0.7600 | **1.000** |
| Sparse infill | 3.839 | 3.236 | 0.843 | 0.7600 | 0.7600 | **1.000** |
| Internal solid | 1.511 | 1.175 | 0.777 | 0.7600 | 0.7600 | **1.000** |
| Top surface | 1.936 | 0.983 | 0.508 | 0.7600 | 0.7600 | **1.000** |
| **Prime tower** | 1.950 | **151.304** | **77.6** | **1.2629** | **0.7600** | **0.602** |

**E per block is identical to four decimals on every object feature**, so `wipe_dist`,
the retraction length and the `0.95` dE factor are all correct on the object path. The
R606 prediction of a scalar difference is **REFUTED** for object features. The distance
varies while E is pinned, so it is the `wipe_dist` clamp — C++'s "handle short path
case" — that differs, not the extrusion math.

**And the prime tower row is a real defect, with a closing identity.** `exporter.rs:2606`
declares `_toolchange: bool` — present, underscore-prefixed, **deliberately unused** —
and `:2616` is `let length = retraction_length;`. C++ (`GCode.cpp:373-376`) is:

```cpp
double length = toolchange ? retract_length_toolchange() : retraction_length();
length *= (1. - retract_before_wipe());
```

Two omissions in one line: the toolchange branch is ignored, and `retract_before_wipe`
is never applied at all. The measured prime-tower E ratio is **0.602**, and
0.76 / 1.2629 = **0.6018** — the ratio is not a symptom of the missing branch, it IS
`retraction_length / retract_length_toolchange`. C++ also switches the wipe SPEED to
`prime_tower_max_speed` for toolchanges (`GCode.cpp:370`), which we likewise ignore;
that is the likely other half of the 151 mm vs 1.95 mm travel.

This is R602's defect class for the third time: a symbol that exists, looks ported, and
is never used. R602 was a method with zero callers; this is a parameter the compiler was
told to ignore. **Underscore-prefixing a parameter to silence a warning is how a missing
port hides in plain sight — grep `fn.*_[a-z]+:` for it.**

**Not fixed this round.** It needs `retract_length_toolchange`, `retract_before_wipe` and
`prime_tower_max_speed` plumbed to the wipe site plus a gated A/B, which is R607's job
rather than something to rush at the end of a measurement round (R604's precedent).

**R607:** honour `toolchange` in `wipe()`. Use `retract_length_toolchange` when it is
set, apply `length *= (1 - retract_before_wipe)` unconditionally, and switch the wipe
speed to `prime_tower_max_speed` for toolchanges. Predict the prime-tower E/block moves
0.7600 -> 1.2629 (an exact, checkable target) and its travel collapses from 151.3 mm
toward C++'s 1.95 mm; predict object features are UNCHANGED, since their E/block already
matches to four decimals and `retract_before_wipe` is likely 0 for this profile — verify
that before assuming it. Fallback: if object features DO move, `retract_before_wipe` is
non-zero and the change is not tower-local, so A/B it on Benchy too before shipping. The
remaining `wipe_dist`-clamp difference on object features (Inner wall 0.636x, Top surface
0.508x) is a separate item and should not be folded in.
## R607 — two corrections to R606, and the defect relocated to the live path

R607 set out to implement R606's fix. Checking the premises first (as the round's own
brief required) invalidated two of them. Measurement only — no code changed, `git status`
clean, baselines intact.

**Correction 1: R606's "closing identity" was CIRCULAR.** R606 reported the prime-tower
E ratio as 0.602 and announced that 0.76 / 1.2629 = 0.6018 "IS
`retraction_length / retract_length_toolchange`". But 0.602 was itself computed as
0.76 / 1.2629 — the check compared two numbers with themselves and could not have
failed. The actual config, read this round from the 3MF:

| key | value |
|---|---|
| `retraction_length` | 0.8 |
| `retract_length_toolchange` | **2** |
| `retract_before_wipe` | **0%** |
| `wipe_distance` | 2 |
| `prime_tower_max_speed` | 90 |

`retraction_length / retract_length_toolchange` = 0.8 / 2 = **0.4**, not 0.602. The
identity did not hold. R606's pre-registered fallback ("if the tower E/block does not
land on 1.2629, the identity was coincidence — dump the actual config values") is what
caught it.

**The real identity, which is not circular.** Our 0.76 = 0.8 x 0.95 (plain retraction x
the dE factor). C++'s toolchange wipes should be 2 x 0.95 = 1.90, and its Prime tower
mean is a MIXTURE of toolchange and ordinary tower wipes:

    E/blk = 0.76*(1-x) + 1.90*x,  x = toolchanges / tower wipe blocks = 2723/6161 = 0.4420
          = 1.2639     vs measured 1.2629   (0.08%)

That predicts C++'s measured mean from four independently-known quantities, so it does
confirm C++ uses `retract_length_toolchange` for toolchange wipes. **It also invalidates
R606's target for OUR output**: our ratio is 2721/3371 = 0.8072, so a correct fix should
land near **1.6802**, not 1.2629.

**Correction 2: R606 analysed DEAD CODE.** `exporter.rs:2600 pub fn wipe` — the function
whose `_toolchange: bool` R606 built its finding on — **has no callers**. Its only
would-be caller, `exporter.rs:2491 retract()`, is itself stubbed: `let _ = wipe;` with
"TODO: Implement wipe integration ... For now, skip wipe". The live wipe is
**`writer.rs:1564 pub fn retract()`**, which emits `; WIPE_START` at `:1610`/`:1701`.

This is the DEAD-TWIN rule (R597/R600) — which R606 listed in its own checklist and did
not apply to the function it was reading. Third dead-twin in this codebase after
`fill/mod.rs` and `extrude_collection`.

**What the live path actually does.** `writer.rs:1574-1623` is a faithful port and DOES
apply the factor R606 said was missing entirely:

```rust
let length = self.retraction_length * (1.0 - self.config.retract_before_wipe / 100.0);
```

(note the `/100.0` — the config is a percentage, `0%`, so this term is inert here). So
R606's "`retract_before_wipe` is never applied" is also wrong.

**The defect that survives, correctly located.** `writer.rs:1564 retract()` takes **no
`toolchange` parameter at all** and uses `self.retraction_length` unconditionally, so
toolchange wipes retract 0.8 where C++ retracts 2. There is also no
`retract_for_toolchange` anywhere in our writer, though C++'s `GCodeWriter` has one. The
observation R606 made is real; only its site and its arithmetic were wrong.

**Not fixed this round, deliberately.** `.retract()` has 14 call sites and none of them
carries a toolchange flag, so this needs a `retract_for_toolchange` (or a flag threaded
to the tower's retract) plus a probe to identify which call site emits the tower wipes.
Rushing that at the end of a round spent correcting two errors is how the third gets
made.

**R608:** add the toolchange branch on the LIVE path. First probe `writer.rs:1564
retract()` to identify which call site produces the 3,371 Prime-tower wipe blocks, then
give that site `retract_length_toolchange` (C++ `GCodeWriter::retract_for_toolchange`,
and `wipe_speed = prime_tower_max_speed` per `GCode.cpp:370`). **Pre-registered target,
now derived rather than guessed: tower E/block 0.7600 -> ~1.6802, and the per-block max
|E| in the Prime tower 0.76 -> 1.90 exactly.** Object features must be unchanged —
`retract_before_wipe` is 0% so that term is inert, and their E/block already matches to
four decimals. Fallback: if the tower wipes do not come from `writer.retract()` at all
but from the tower template's own `G1 E-` lines, then the E discrepancy is in
`wipe_tower.rs` and the toolchange branch is not the fix.
## R608 — the missing tower-entry wipe: located exactly, ported, and inert on ordering

R607 left "add the toolchange branch to `writer.retract()`" as the target. Measuring
first relocated it again, this time to something sharper and fully quantified.

**Where the tower wipes actually are.** Classifying every Prime-tower `; WIPE_START` by
its position inside the tool-change block:

| position | C++ blocks | C++ mean \|E\|/block | Rust blocks | Rust \|E\| |
|---|---|---|---|---|
| after `CP TOOLCHANGE END` | 2,512 | 0.7600 | 2,715 | 0.7600 |
| after `WIPE_TOWER_END` | 931 | 0.7600 | 656 | 0.7600 |
| **inside `WIPE_TOWER_START`..`END`** | **2,718** | **1.9000** | **0** | — |

**Our existing tower wipes already match C++'s exactly at 0.7600.** The entire
Prime-tower E/block discrepancy is 2,718 wipe blocks we never emit, each summing to
exactly **1.9000** = `retract_length_toolchange` (2) x the 0.95 dE factor. The mixture
closes on directly-measured counts:

    0.76*(3443/6161) + 1.90*(2718/6161) = 1.2631   vs C++ measured 1.2629

This supersedes R607's identity, which used the toolchange count (2,723) as a proxy for
the in-tower block count (2,718). Same arithmetic, but now the quantity is the one
actually being counted.

**A mid-round correction to my own C++ reading.** `append_tcr:1085` is
`gcodegen.retract(false, ...)` — `toolchange` FALSE — which I first read as proof that
C++ does not use the toolchange length here. Measuring the blocks directly showed
1.9000 exactly, so the 1.90 comes from a different retract in the same region. Reading
the call and inferring the value was wrong; measuring the emitted E settled it.

**A fourth unused-symbol instance.** `ToolChangeResult.wipe_path` (`wipe_tower.rs:340`)
is populated (`:1291`, copied at `:2889`) and **never consumed** — neither `print.rs`
nor `wipe_tower_integration.rs` reference it. C++ reads exactly this field at
`GCode.cpp:1083` to install the wipe path before retracting. After a method with zero
callers, a counter never incremented, and an underscore-silenced parameter, this is the
same class again: the data was carried all the way to the emitter and dropped.

**What was implemented.** `writer.rs`: `retract_for_toolchange()`, which swaps in
`retract_length_toolchange` around the existing `retract()` — C++ applies the toolchange
length to the whole retract+wipe, so this is the same semantics without duplicating
~130 lines. `print.rs`: splice the retract in immediately after `; WIPE_TOWER_START`,
the exact position C++ uses (the in-tower wipe is the first thing in the block, and its
coordinates are object-space — it wipes the path just printed and is tagged in-tower
only because the marker precedes it).

**Result: PREDICTION WRONG — +2 blocks, not ~2,718.** 36,394 -> 36,396.

**Cause, confirmed structurally rather than guessed.** Comparing the nine lines before
`; WIPE_TOWER_START` on both engines, ours carries an extra `G1 X.. Y.. F1800` — a
travel to the tower emitted BEFORE the block, which C++ does not have. A travel implies
a preceding retract, so the writer is already `retracted` when the splice runs and
`retract()` early-returns. C++'s order is wipe-and-retract inside the block FIRST, then
travel. The pre-registered fallback named exactly this ("the writer is already retracted
at tower entry"), and the structural diff confirms it.

**Shipped OPT-IN (`TOWER_ENTRY_WIPE_CPP`, default OFF).** It is a faithful port of
`append_tcr`'s wipe install and is kept so the ordering round can build on it, but it is
inert until the ordering is fixed and 2 arbitrary blocks are not a result worth
defaulting on. Baselines reproduce: `304320a6` / `56938d4d` / `242f1fb8`. Benchy and
cube are byte-identical either way (no wipe tower).

**R609:** the ORDERING. C++ emits, inside the tower block: toolchange-retract-with-wipe,
then the travel to the tower. We emit the travel first (`TOWER_TRAVEL_TO_START`, R476)
and retract before it, which both consumes the retract and puts the travel outside the
block. Move the travel to after the tower-entry retract and re-run with
`TOWER_ENTRY_WIPE_CPP=1`. **Target unchanged and exact: ~2,718 new in-tower wipe blocks
at |E| = 1.9000 each, tower mean E/block 0.7600 -> ~1.26, and the after-toolchange /
after-tower counts should move toward C++'s 2,512 / 931 from our 2,715 / 656.** Fallback:
if moving the travel does not free the retract, the retract that precedes it is emitted
elsewhere (object-side end-of-feature) and the fix is to suppress THAT one for tower
entry rather than to reorder the travel.
## R609 — R608's cause REFUTED; the tower wipe is fully enabled 2,718 times and still does not appear

R609 set out to fix the ordering R608 blamed. Measuring first refuted that cause, and
left a sharper contradiction. Two default-off probes added; all three baselines
reproduce byte-for-byte.

**R608's cause was wrong.** R608 concluded the writer is already `retracted` at tower
entry (because we emit a travel C++ does not), so `retract()` early-returns. Both halves
fail:

* The emitted gcode shows **no retract before tower entry**. The actual sequence is
  last object extrusion -> `G1 X.. Y.. F30000` travel -> the LAYER_HEIGHT/FEATURE/
  LINE_WIDTH trio -> a zero-length `G1 X.. Y.. F1800` -> `; WIPE_TOWER_START`. R608 read
  the travel as implying a preceding retract; there is none.
* The `TOWER_ENTRY_WHY` census confirms it: of 2,721 tower-entry calls, only **3** were
  `already_retracted`, **0** had wipe disabled, and **3** had a short path.

**The call is fully enabled exactly 2,718 times:**

    [TCWHY] n=2721 already_retracted=3 wipe_disabled=0 path_short=3 would_wipe=2718

**2,718 is exactly the number of blocks C++ emits and we lack** (R608's position-class
census). The guards are not the problem — the tower-entry wipe is enabled precisely as
often as it should fire.

**And yet the output gains only +2 `; WIPE_START`** (36,394 -> 36,396, confirmed on both
the saved `r608_majora_on.gcode` and a fresh run of the same configuration). So the wipe
is enabled 2,718 times and reaches the file twice.

**The wipe branch itself is healthy.** A second probe at the emit decision
(`kept.len() >= 2 && acc > 1e-4`) reports **34,991 emitted of 35,000** decisions, with
`wipe_dist=1.0000 acc=1.0000 kept=3`. The machinery works for the ordinary object
retracts that produce our 36,394 existing wipes.

**What this leaves.** Enabled 2,718 times, emitted twice, with the emit logic otherwise
sound. That points at the interaction between the writer's `write_raw` output and the
raw tower-gcode splice in `emit_tower_tcr` — the retract is invoked between
`write_raw_content(&g[..cut])` and `write_raw_content(&g[cut..])`, and its lines are not
landing between them. I am NOT naming a mechanism for that here: R606 and R608 both lost
a round to a diagnosis asserted from reading rather than measurement, and the honest
state is "enabled 2,718, emitted 2, emit logic sound".

**Shipped:** two default-off probes (`TOWER_ENTRY_WHY` in `retract_for_toolchange` and
at the wipe emit decision). No behavioural change — `304320a6` / `56938d4d` /
`242f1fb8` all reproduce, 8/8 guards. `TOWER_ENTRY_WIPE_CPP` remains OPT-IN.

**R610:** find where the 2,716 enabled-but-absent wipes go. The decisive instrument is a
probe INSIDE the emit branch that records the byte offset or a serial marker written to
the output stream, run with the tower gate on, then grep the produced file for that
marker: either the lines are written somewhere unexpected (an ordering/buffering
interaction with `write_raw_content`) or they are written and later discarded. **Predict
they ARE written but into a buffer that the tower splice overwrites or bypasses, because
the emit decision reports success 34,991/35,000 and the file contains 36,396 blocks —
the two numbers are close enough that the tower wipes are plausibly being counted as
emitted internally while not reaching the file. Fallback: if the marker appears in the
file, the wipes ARE present and the position-class census is mis-classifying them (e.g.
they land before `; WIPE_TOWER_START` and count as after_tc), in which case re-run the
R608 census on the gate-ON file rather than trusting the total.**
## R610 — the tower wipe WAS firing all along; R608 measured the wrong file

The marker test settled it in one build, and the answer was the branch I did not predict.

**Prediction WRONG, fallback fired.** I predicted the marker count would be ~2 and that
`lift_faithful_gate()` — the guard R609's probe had missed — would be the rejector. The
per-guard census scoped to tower calls says **`no_lift_gate=0 no_wipe=0 path_short=0`**:
every guard passes. And the file contains **2,717 `; _R610_` markers**, each immediately
after a `; WIPE_START`. The wipes were being emitted the whole time.

**Why R608 and R609 both read "+2".** R608's position-class census ran on
`r605_majora_on.gcode` — a file produced **without** the tower gate. "We have zero
in-tower wipes" was a true statement about the BASELINE, and R608 then read the +2
change in the `; WIPE_START` TOTAL as "the fix didn't fire" without re-running the
census on the gate-ON file. R609 inherited that framing and spent a round explaining a
non-existent absence.

**What the gate actually does — the accounting closes exactly:**

| | after_tc | in_tower | in_tower mean \|E\| |
|---|---|---|---|
| C++ | 2,512 @ 0.7600 | **2,718** | **1.9000** |
| Rust gate OFF | 2,715 @ 0.7600 | 0 | — |
| **Rust gate ON** | **0** | **2,717** | **1.9000** |

The gate does not ADD 2,717 wipes, it MOVES them: the tower-entry retract consumes the
wipe path and the retracted state, so the after-toolchange retract that used to wipe no
longer does. 2,715 out, 2,717 in, net +2 on the total — which is exactly the number that
misled two rounds.

**The in-tower wipes now match C++ to one block and to four decimals on |E|** (2,717 vs
2,718, both 1.9000). That half of R608's port is correct and confirmed.

**But it trades one divergence for another.** C++ retracts TWICE — once at tower entry
with `retract_length_toolchange`, and again after the tool change with the ordinary
length. We retract once, so gaining the in-tower wipe costs the after-toolchange one
(2,715 -> 0 against C++'s 2,512).

**Parity: 648,759 -> 648,773, +14 matched lines.** A wash, exactly as the trade implies —
2,717 newly-correct blocks in, 2,715 previously-correct blocks out.

**Stays OPT-IN.** It fixes a real divergence and creates a real one; net +14 of 648,773
is not grounds to change the default. It becomes default-on when the second retract
exists, at which point both positions should match C++ simultaneously.

**Shipped:** the R610 marker and a per-guard tower-scoped census under
`TOWER_ENTRY_WHY`, replacing R609's partial probe (which checked three of the six
sub-conditions and counted globally rather than per tower call). Baselines reproduce:
`304320a6` / `56938d4d` / `242f1fb8`; 8/8 guards.

**R611:** add the SECOND retract so both positions match. C++'s after-toolchange retract
uses the ordinary length (its after_tc blocks measure 0.7600); ours is currently consumed
by the tower-entry one. The likely shape is an unretract-then-retract around the tool
change, or simply not letting the tower-entry retract suppress the later one. **Target,
exact: after_tc 0 -> ~2,512 at |E| 0.7600 while in_tower stays 2,717 at 1.9000; total
tower blocks 3,373 -> ~5,900 against C++'s 6,161.** Fallback: if adding the second
retract also re-suppresses the first, the two share the `retracted` flag and the fix is
to model C++'s unretract between them rather than to add a call.

**Method note.** Three rounds were spent on an absence that was not there, because a
TOTAL moved by 2 while its COMPONENTS moved by 2,717 in each direction. R608's census was
the right instrument; it was simply pointed at the wrong file. **When a total barely
moves, re-run the per-class census on the SAME artefact you are judging — never compare a
new total against an old breakdown.**
## R611 — the second retract: a stale flag, not a missing call. +8,175 lines, both gates DEFAULT-ON

R610 left the tower-entry wipe correct but trading against the after-toolchange one.
This round closed the trade — and the fix was not the one predicted.

**Prediction WRONG, registered fallback RIGHT.** The prediction was that adding an
explicit second `retract()` (mirroring `append_tcr`'s second call) would restore
after_tc. Measured on the same file: **after_tc stayed at 0**. The fallback named the
reason exactly — the writer is still `retracted` from the tower-entry call, so
`retract()` early-returns.

**The cause is state tracking, not a missing call.** The wipe-tower block is spliced in
as RAW TEXT and unretracts inside itself (the `change_filament_gcode` template's
`G1 E<n>` plus R466's `G1 E{retract_length_toolchange}` trailer). The writer never sees
those lines, so once R608's tower-entry retract set `retracted = true`, it stayed true
and every later `retract()` no-opped. That is why enabling `TOWER_ENTRY_WIPE_CPP` drove
after-toolchange wipes from 2,715 to 0.

The fix is a state sync, not an extra retract: `writer.mark_unretracted_after_raw()`,
which clears the flag WITHOUT emitting. Calling `unretract()` would have added a
Z-unlift and an E move C++ does not have there. **There was already precedent three
lines below** — R475's `set_last_extrusion_role` after the same `write_raw_content`,
for the same reason: raw writes change machine state the writer tracks separately.

Also consumed at last: `tcr.wipe_path` is reinstalled before the retract, per
`GCode.cpp:1081-1084`. That is the field R608 found populated and never read — the sixth
unused-symbol instance, now used for what it was carried for.

**Result — both positions match, measured on the file being judged (R610's rule):**

| | after_tc | after_tower | in_tower | total |
|---|---|---|---|---|
| C++ | 2,512 @ 0.7600 | 931 @ 0.7600 | **2,718 @ 1.9000** | **6,161** |
| Rust @R610 | 0 | 656 | 2,717 @ 1.9000 | 3,373 |
| **Rust @R611** | **2,721 @ 0.7600** | 656 @ 0.7600 | **2,717 @ 1.9000** | **6,094** |

Tower wipe blocks **3,373 -> 6,094** against C++'s 6,161 — from 54.7% to **98.9%** of
C++'s count, with both |E| values exact to four decimals.

**Parity: 648,759 -> 656,934, +8,175 matched lines.** Rate 25.60% -> **25.78%**;
line-count gap 8.91% -> **8.42%**. Total `; WIPE_START` 36,394 -> 39,117 (+2,723),
against the pre-registered ~39,100.

**Both gates flipped DEFAULT-ON** (`TOWER_ENTRY_WIPE_CPP`, `TOWER_EXIT_WIPE_CPP`). They
must ship together: entry alone was +14 (R610), the pair is +8,175. Gates OFF reproduces
`56938d4d` byte-for-byte. Benchy and cube are byte-identical either way (no wipe tower),
so only majora re-baselines: **`56938d4d` -> `69eb9767`**. 8/8 guards.

**What is still off:** after_tc 2,721 vs C++'s 2,512 (we wipe on every tool change, C++
on 2,512 of 2,723) and after_tower 656 vs 931. Together that is 67 blocks below C++'s
total, against 2,788 before. Both are follow-ups, not blockers.

**R612:** re-check `WIPE_MULTIPATH_CPP` for default-on (R600's rule — its upstream
population just changed materially), then the remaining tower deltas: after_tower 656 vs
931 (-275) and after_tc 2,721 vs 2,512 (+209). Predict the after_tower shortfall is the
`finish_layer` tcr path, which takes the `!had_placeholder` branch and never reaches the
new sync. Fallback: if `finish_layer` already syncs, the difference is C++ skipping the
wipe on some tool changes (2,512 of 2,723) via a condition we do not model — find it
before adding one.
## R612 — two hypotheses tested, both refuted, both reported with numbers

Two items, both pre-registered, both measured, both wrong. No behavioural change ships:
one gate stays opt-in on the evidence, one speculative branch was reverted after proving
inert. Baselines reproduce.

**(1) `WIPE_MULTIPATH_CPP` re-checked for default-on (R600's rule) — still OPT-IN.**

It shipped opt-in at R605 as parity-neutral (+10 benchy / −18 majora) when the tower
wipes were absent. R601–R611 then added 2,723 wipe blocks to its upstream population,
which is exactly the situation R600's rule exists for. Predicted it would now be
net-positive on majora, since the wipe paths it corrects are consumed by far more wipes.

| | baseline @R611 | with `WIPE_MULTIPATH_CPP=1` | delta |
|---|---|---|---|
| Benchy matched | 115,900 | 115,910 | **+10** |
| Majora matched | 656,934 | 656,914 | **−20** |

**Prediction WRONG.** Net −10, statistically identical to R605's −8; the 2,723 new wipe
blocks did not change its verdict at all. The registered fallback said "if still
negative, leave OPT-IN and say so with the numbers" — done. The re-check was still worth
running: the rule is to re-examine opt-in gates when their population moves, and the
answer being "no change" is a result, not a wasted round.

**(2) The after_tower shortfall is NOT the `finish_layer` tcr.**

R611 left after_tower at 656 against C++'s 931 (−275). The prediction was that
`finish_layer`'s tcr never reaches R611's state sync, because that sync is gated on
`tcr.is_tool_change` and `finish_layer` is not a tool change — structurally true, and
easy to believe.

Extending the sync to it, DEFAULT-ON, left majora **byte-identical** (`69eb9767`). Not
"a small change" — no change at all. So either the branch never runs for `finish_layer`
or the sync is a no-op there; either way the hypothesis is dead. **The speculative branch
was reverted rather than kept as inert default-on code.**

**What that leaves.** after_tower 656 vs 931 and after_tc 2,721 vs 2,512 — we emit 209
too many in one class and 275 too few in another, netting 67 blocks below C++'s total.
The registered fallback now stands as the live hypothesis: **C++ is skipping the wipe on
some tool changes (2,512 of 2,723) via a condition we do not model**, and the surplus in
our after_tc is the same 209 wipes C++ declines to emit. That is a condition to FIND in
`GCode::retract`'s callers, not one to invent.

**Nothing shipped.** `git status` shows only the reverted comment; majora `69eb9767`,
benchy `304320a6`, cube `242f1fb8` all reproduce; 8/8 guards.

**R613:** find C++'s skip condition. It emits 2,512 after-toolchange wipes for 2,723 tool
changes — 211 skipped, and our surplus over C++ is 209, which is the same population to
within two. **Instrument the C++ side** (`GCode::retract` / `Wipe::wipe` entry) to record,
per tool change, whether the after-toolchange wipe fired, then correlate the skips
against tool index, layer, and whether the next feature is on the same object. Predict
the skip is `Wipe::has_path()` returning false because C++'s wipe path is consumed by the
in-tower wipe and only re-armed when `tcr.wipe_path` is non-empty — i.e. C++ skips
exactly the tool changes whose tcr carries no wipe path. **Fallback: if `tcr.wipe_path`
is non-empty on all 2,723, the skip is a retract-level condition (`m_writer.retracted()`
or the `EXTRUDER_CONFIG(wipe)` per-filament flag) and the census should be re-run keyed
on filament id.**
## R613 — C++ skips NO wipes; the residual is 276 missing finish-layer tower blocks

Measurement only, no code changed (R612's revert already landed). All three baselines
reproduce. This closes the tower-wipe investigation and replaces its last open item with
a different, better-defined one.

**R612's fallback premise was WRONG, and it was my own artefact.** R612 handed over
"C++ skips the wipe on 211 of its 2,723 tool changes via a condition we do not model".
Walking C++'s output directly — for each `; CP TOOLCHANGE END`, does a wipe follow before
the next tool change?:

    C++ after_tc wipes: fired=2723 skipped=0 total=2723

**C++ skips none.** There is no condition to find. The "211 skipped" figure came from my
own position-class census, which only counts wipes still tagged `Prime tower` — so it was
measuring feature tagging and block interleaving, not wipe behaviour. Both engines in
fact tag the first after-toolchange wipe as `Prime tower` on every single tool change
(C++ 2,723, Rust 2,721).

**Where the census numbers actually come from.** Splitting tower blocks by kind — a
`WIPE_TOWER_START..END` is a TOOLCHANGE block if a `CP TOOLCHANGE START` is open, else a
FINISH-LAYER block:

| | toolchange blocks | in-tower wipes | finish-layer blocks |
|---|---|---|---|
| C++ | 2,723 | 2,718 | **932** |
| Rust | 2,721 | 2,717 | **656** |

The toolchange side is at parity — 2 blocks short, which is the long-known
2,723-vs-2,721 tool-change difference, not anything new. The census's after_tc split
(2,512 vs 2,721) is interleaving: 211 of C++'s toolchange wipes land after an intervening
finish-layer block and get classed `after_tower`. Accounting for that, C++'s 931
after_tower = ~211 reclassified toolchange wipes + ~720 finish-layer wipes, against our
656 — a 64-wipe difference, which with the 2 toolchange and 1 in-tower blocks makes the
67-block census total. **The identity closes.**

**The real remaining item is a BLOCK COUNT, not a wipe count: we emit 656 finish-layer
tower blocks against C++'s 932 — 276 short, 0.70x.** Every such block carries a wipe
after it, which is why it surfaced through the wipe census; but the defect is that the
blocks themselves are missing, which is a wipe-tower *planning* question, not a retract
or wipe question.

**Status of the tower-wipe chain.** With R611 landed, toolchange tower wipes are at
parity in count, position and |E| (2,717 vs 2,718 at 1.9000; 2,721 vs 2,723 at 0.7600).
Everything R604-R612 chased in the wipe machinery is now either fixed or measured to
parity. The open item has moved up a level, to which layers get a finish-layer tower
block at all.

**R614:** the finish-layer tower block count — ours 656, C++'s 932. C++ emits a tower
block on a layer even when no tool change happens there, to keep the tower's height in
step with the object; the count difference means we skip that on ~276 layers. **Predict
we only emit a finish-layer block when the layer already has a tool-change tcr, i.e. our
`wipe_tower_layer.iter().find(|r| !r.is_tool_change)` finds nothing for layers whose plan
has no entries at all — check whether `self.wipe_tower_results` even has a group for
those layers before blaming the emitter.** Fallback: if the plan does contain groups for
all 656+276 layers, the emitter is filtering them out and the condition is in
`emit_layer_by_island`'s `finish_layer` lookup, not in the tower planner. Majora has 657
layers and C++ emits 932 finish-layer blocks, i.e. **more than one per layer** — so
resolve that first: count C++'s finish-layer blocks per layer before assuming a
one-per-layer model.
## R614 — the finish-layer emitter iterates all tcrs; the 276-block deficit is in the PLANNER

**Prediction WRONG, registered fallback RIGHT.** All three baselines reproduce
(`304320a6` / `69eb9767` / `242f1fb8`); the 8 guard tests pass.

**The measurement R613 handed over.** We emit 656 finish-layer tower blocks against
C++'s 932. The per-layer histogram makes the shape clear:

| | finish-blocks | per-layer distribution |
|---|---|---|
| C++ | 932 on 656 layers | 1×386, 2×264, 3×6 |
| Rust | 656 on 656 layers | 1×656 — exactly one, always |

The identity closes exactly: 264×1 + 6×2 = **276** = 932 − 656.

**R614 predicted the emitter.** `print.rs:3775` took only the FIRST non-toolchange tcr:
`wt.iter().find(|r| !r.is_tool_change)`. Replaced with an iterate-all loop behind
`TOWER_FINISH_ALL` (`faithful_gate`, default-on).

    majora gate OFF: 69eb9767
    majora gate ON:  69eb9767

**Byte-identical.** Iterating all changes nothing, so **our plan holds exactly one
non-toolchange tcr per layer** and the emitter was never the constraint. The
pre-registered fallback named this outcome: *"if it stays 656, our plan only holds one
non-toolchange tcr per layer and the defect is in the tower planner, not the emitter —
which the same run distinguishes."*

**The change is kept and shipped default-on anyway.** C++ emits every finish tcr in the
layer, so iterate-all is the faithful form. It is inert today only because the planner
under-produces, and it takes effect the moment the planner is fixed.

**Where the planner diverges.** C++ `generate()` (`WipeTower.cpp:4749`) finishes the layer
**per tower block**:

```cpp
for (WipeTowerBlock& block : m_wipe_tower_blocks) {
    ...
    ToolChangeResult finish_block_tcr;
    if (block_solid) finish_block_tcr = finish_block_solid(block, finish_layer_filament, ...);
    else             finish_block_tcr = finish_block(block, finish_layer_filament, ...);
    // merge into a tool-change tcr whose filament matches, else push standalone
    if (fc_iter != layer_result.end()) { *fc_iter = merge_tcr(*fc_iter, finish_block_tcr); ... }
}
```

Ours (`wipe_tower.rs:2113`) is:

```rust
let finish_result = self.finish_layer();
layer_results.push(finish_result);
```

One, unconditional, no blocks, no merge. That is precisely why our histogram can only ever
be `{1: N}` — C++'s finish count varies with the number of tower blocks on the layer, ours
cannot vary at all.

**Root cause — the EIGHTH instance of the unused-symbol / dead-twin defect class, and the
largest so far.** `WipeTowerBlock` is a fully-defined struct (`wipe_tower.rs:532`, 10
fields) and `wipe_tower_blocks: Vec<WipeTowerBlock>` is a real field (`:1453`) initialised
to `vec![]` (`:1527`) — **and never populated, never read.** The only other two mentions in
the file are comments. C++ uses `m_wipe_tower_blocks` at **23 sites** spanning
`generate_wipe_tower_blocks`, `plan_tower_new`, `finish_layer_new`, `finish_block`,
`finish_block_solid` and `generate`. An entire planner concept — per-filament-category
tower sub-blocks — is declared and never wired up.

**R615:** wire `wipe_tower_blocks`. Start at `generate_wipe_tower_blocks()`
(`WipeTower.cpp:4208`) — the population site — and `get_block_by_category`
(`:4163`), which creates blocks lazily by filament adhesiveness category. **Predict Majora
resolves to more than one block, so the per-layer finish histogram moves off `{1:656}`
toward C++'s `{1:386, 2:264, 3:6}` and the 276-block deficit closes.** Fallback: if Majora
resolves to exactly one block, the 2-and-3 counts come not from block multiplicity but
from the standalone-vs-`merge_tcr` split — in which case measure how many of C++'s 932 are
merged into a tool-change tcr before porting anything. **Population first: count the blocks
C++ builds for Majora before assuming multiplicity is the mechanism** (R606's rule; R612's
premise failure came from skipping exactly this step).
## R615 — the tower acceleration chain was dead at three levels (+3,399 matched lines)

Prediction on the finish-block question **WRONG**, registered fallback **RIGHT**; the round
then found and fixed a separate, larger defect. Benchy `304320a6` and cube `242f1fb8`
byte-identical (neither has a wipe tower). Majora re-baselined `69eb9767` → **`3bc2650c`**;
`TOWER_ACCEL_CPP=0` reproduces `69eb9767` exactly. All 8 guard suites pass (31 tests).

### Part 1 — block multiplicity is NOT the mechanism

R614 predicted Majora resolves to more than one wipe-tower block, which would move the
per-layer finish histogram off `{1:656}` toward C++'s `{1:386, 2:264, 3:6}`. Measuring the
population first (R606's rule) by reading the 3MF's own `project_settings.config`:

    "filament_adhesiveness_category": ["100","100","100","100","100","100","100","100"]

All 8 filaments share one category, and `get_block_by_category` (`WipeTower.cpp:4161`)
creates one block per **distinct** category — so **C++ builds exactly ONE block for
Majora.** Multiplicity cannot produce the 2s and 3s. The fallback named this outcome.

**What the extra blocks actually are**, from C++'s own output (layer 138, two finish
blocks): block0 is the wall/perimeter pass carrying the `E-0.8`/`E+0.8` gap-wall zigzag;
block1 is the `CP EMPTY GRID` fill. **We emit both pieces — concatenated into a single
`WIPE_TOWER_START..END`, and in the opposite order** (grid first, perimeter last). The
defect is not a missing extrusion but a missing **split**: C++'s `finish_block` /
`finish_block_solid` and `finish_layer_new` return *separate* ToolChangeResults, each of
which gets its own block markers when emitted. That re-scopes the port for R616.

### Part 2 — the fix: the tower acceleration chain, inert at three levels

C++'s `WipeTowerWriter` emits `M204` from inside its move emitters
(`WipeTower.cpp:760-763` for `G1`, `:839-842` for arcs), choosing travel vs normal
acceleration by whether the move extrudes. Ours emitted none, because the chain was dead at
**every** level:

1. `WipeTowerConfig`'s five accel fields were declared and **never assigned** —
   `print.rs:2040` built the config with `..Default::default()`.
2. `WipeTowerWriter`'s four lists were declared, initialised `vec![]`, **never populated** —
   there was no setter to populate them at all.
3. `set_normal_acceleration()` / `set_travel_acceleration()` were fully ported and had
   **zero callers**, so even a populated list would never have been read.

That is the **NINTH** instance of the unused-symbol/dead-twin class, and the first spanning
three levels. (`multi_material.rs:192 to_wipe_tower_config` is a dead twin of the live
`print.rs:2040` site and also leaves the fields unset — checked, not assumed.)

The port, behind `TOWER_ACCEL_CPP` (`faithful_gate`, **default-on**):

- **`print.rs`** — populate the five fields from `default_acceleration`,
  `initial_layer_acceleration`, `travel_acceleration`, `initial_layer_travel_acceleration`
  and `machine_max_acceleration_extruding`, rounded with C++'s `floor(value + 0.5)`
  (`WipeTower.cpp:1769-1789`). C++ keeps per-extruder vectors and our PrintConfig keeps
  scalars, so each list reduces to one entry — consistent with this port having no
  multi-nozzle group result.
- **`wipe_tower.rs`** — add the five public populators (`WipeTower.cpp:1356-1360`) and
  `set_for_wipe_tower_writer` (`:2661-2667`), called at all three live
  writer-construction sites (the other two are tests).
- **`wipe_tower.rs`** — call the emitters from `travel_to` and `extrude_explicit`,
  mirroring `:760-763`'s `e == 0` branch.

**Measured on the file being judged (R610), by position class:**

| | object | tower toolchange | tower finish-layer | total |
|---|---|---|---|---|
| C++ | 51,773 | 10,899 | **954** | 63,626 |
| Rust before | 42,612 | 5,562 | **0** | 48,174 |
| **Rust after** | 42,612 | **8,292** | **669** | **51,573** |

**Line parity: 656,934 → 660,333 matched (+3,399); our body lines 2,547,764 → 2,551,163,
also +3,399. Every added line matched — no regressions anywhere.** Rate 25.78% → **25.88%**,
line-count gap 8.42% → **8.30%**.

### Also quantified, left open

Our tower emits **271 bare `G1` lines** (no arguments); C++ emits **zero**. These are the
visible end of the redundant-travel deviation documented at `transform_gcode`'s
`TOWER_XFORM_NO_RAW_ECHO` branch (R477): our tower writer travels to a point it is already
at, which C++'s never does. Bounded at 271 lines.

**R616:** split the finish-layer tcr. Port `finish_block` / `finish_block_solid`
(`WipeTower.cpp:4749`'s loop) as a **separate** ToolChangeResult from `finish_layer_new`,
rather than emitting one concatenated block — and put the perimeter pass **first**, matching
C++'s observed order. **Predict the finish-layer block count moves 656 → ~932 and the
per-layer histogram picks up a 2-bucket.** Fallback: if the count stays at 656, the two
pieces are being produced by one function that cannot be split without also porting
`merge_tcr`'s insert-into-matching-toolchange path — in which case measure how many of
C++'s 932 are standalone versus merged before porting further. Note the two engines' finish
blocks also sit **2.0mm apart in Y** (C++ `Y225.297`/`Y235.797` vs ours `Y227.297`/
`Y237.797`) — a tower depth/offset difference worth resolving in the same round, since it
blocks those lines from ever matching.
## R616 — the finish split is blocked on the planner; the Y offset is NOT a rigid shift

Prediction **RIGHT** on the one change made. Two measurements this round stopped a wrong fix
and sequenced the right one. Majora re-baselined `3bc2650c` → **`137de4a3`**.

### Measurement 1 — the 2.0mm Y offset is real, finish-block-specific, and NOT a translation

R615 handed over "the finish blocks sit 2.0mm apart in Y; measure whether it is a constant
offset on every tower line or only on the finish blocks **before changing any geometry**."
Sweeping every constant offset in [-3, +3] mm at 1µm steps and counting how many distinct
Rust Y values land on a C++ one:

| | C++ distinct Y | Rust | exact-shared | best constant offset |
|---|---|---|---|---|
| tower toolchange | 5,903 | 4,090 | 268 | −0.250 → 271/4,090 |
| tower finish-layer | 90 | 59 | 41 | **−2.000 → 49/59** |

So the offset is **confined to the finish blocks** — the toolchange blocks show no offset at
all (the best shift buys 3 values out of 4,090, i.e. noise). But applying it would be wrong:
the two ranges are C++ `[194.248, 239.346]` (span **45.098**) against ours
`[197.047, 240.547]` (span **43.500**). **Our finish box is also 1.6mm shorter**, so this is
a box extent/depth difference, not a rigid translation — and it is the same two-depths
problem already documented at R509/R510 (`block.layer_depths[cur]` for `finish_block`'s box
vs `m_layer_info->depth` for `finish_layer_new`'s). **No geometry was changed.** Measuring
first is what prevented shipping a constant shift that would have mis-aligned the span.

### Measurement 2 — why we can only ever emit one finish block, and what unblocks it

C++ writes `; WIPE_TOWER_START` / `; WIPE_TOWER_END` from **seven** sites, one per
tcr-producing function (`WipeTower.cpp:2088/2161`, `2691/2831`, `3288/3328`, `3550/3721`,
`3743/3831`, `3859/3947`, `4966/4988`):

| function | emits its own marker pair |
|---|---|
| `tool_change` / `tool_change_new` | yes |
| `finish_layer` / `finish_layer_new` | yes |
| **`finish_block`** | **yes** |
| **`finish_block_solid`** | **yes** |
| **`only_generate_out_wall`** | **yes** |

We have **two** (`wipe_tower.rs:2299/2355` tool change, `:2617/:2911` finish layer). Per
layer C++ can therefore emit `finish_layer_new` + `finish_block` + `only_generate_out_wall`
= up to three finish-side blocks, which is exactly the observed `{1:386, 2:264, 3:6}`; ours
is structurally pinned at `{1:656}`.

**The split cannot be ported yet.** `finish_block(const WipeTowerBlock &block, int
filament_id, bool extrude_fill)` and `finish_block_solid(...)` both take a
`WipeTowerBlock` — and R614 established that `wipe_tower_blocks` is declared, initialised
`vec![]`, and never populated. So the finish-layer split is **downstream of** the planner
port, not an alternative to it. That sequences the work: `generate_wipe_tower_blocks`
(`:4208`) + `get_block_by_category` (`:4163`) must land first. (`only_generate_out_wall` is
separately gated: it fires under `only_generate_wall`, which C++ has ON for Majora via
`timelapse_type = 1`, while ours is opt-in behind `TOWER_TIMELAPSE_DEPTH` pending R509's
two-depth separation.)

### The one fix landed — an off-by-one blank line at the EMPTY GRID close

C++'s literal (`WipeTower.cpp:3643-3644`) is

```cpp
writer.append("; CP EMPTY GRID END\n"
              ";------------------\n\n\n\n\n\n\n");
```

— seven newlines after the separator text: one terminates the separator line, six are blank.
Ours had eight. Measured blank-runs following `;------------------`, both engines, same
files:

| | before | after |
|---|---|---|
| C++ | `{0: 5655, 2: 211, 3: 2512, 6: 209}` | (unchanged) |
| Rust | `{0: 5649, 2: 2721, 7: 207}` | `{0: 5649, 2: 2721, 6: 207}` |

Our `7`-bucket moved to `6`, matching C++'s. **Line parity is unchanged at 25.88%
(660,333/2,551,163, body-line count identical), which confirms `line_parity.py` strips blank
lines** — so this is faithfulness (ask #1), not a parity gain, and is reported as such.

### Also found, left open

The same census shows a second blank-line divergence in the **toolchange** separator: C++
`{2: 211, 3: 2512}` against our `{2: 2721}` — C++ varies between two and three blanks across
its 2,723 tool changes while we always emit two. The 211/2,512 split is conditional on
something we do not model; **do not "fix" this by making it constantly three.**

**R617:** port `generate_wipe_tower_blocks` (`WipeTower.cpp:4208`) + `get_block_by_category`
(`:4163`) to populate `wipe_tower_blocks`, behind one gate, and measure before porting
`finish_block`. Majora has ONE filament category (R615), so **predict the block vector
becomes length 1 and the output is byte-neutral — the value is that it unblocks
`finish_block`, not that it changes gcode.** Fallback: if the output does change, the block's
`start_depth`/`cur_depth`/`layer_depths` are feeding a path we already compute differently,
and that difference must be understood before `finish_block` is added on top. **State plainly
that a byte-neutral result is the expected and successful outcome here (R614's lesson), so it
is not mistaken for a failed round.**
## R617 — `wipe_tower_blocks` is populated at last; byte-neutral, as predicted

**Prediction RIGHT on both clauses.** This round deliberately produces **no gcode change** —
that was the pre-registered success condition (R614's lesson). Its value is that it removes
the blocker R616 identified: `finish_block` / `finish_block_solid` both take a
`WipeTowerBlock`, and until now there were none.

### What was dead

R614 found `wipe_tower_blocks` declared, initialised `vec![]`, never populated, never read.
R617 checked the levels around it (R615's rule) and found **two more fields in the same
state**: `all_layers_depth: Vec<Vec<BlockDepthInfo>>` (`:1503`) and `filament_categories:
Vec<i32>` (`:1531`) — both `vec![]`, both never written or read. `BlockDepthInfo` itself was
a fully-defined struct with no producer. The category never even reached `PrintConfig`:
`filament_adhesiveness_category` appeared only in `preset.rs`'s known-filament-key allowlist.

### What landed — behind `TOWER_BLOCKS_CPP` (`faithful_gate`, default-on)

- **`print_config.rs`** — new `filament_adhesiveness_categories: Vec<i32>`
  (`PrintConfig.cpp:2385`, `coInts`, default 0).
- **`app_slice.rs`** — read the array from the 3MF alongside `filament_density`.
- **`wipe_tower.rs`** — `WipeTowerConfig::filament_categories`, threaded into `WipeTower`
  at construction (`WipeTower.cpp:1850`).
- **`wipe_tower.rs`** — `get_filament_category` (`:4204`, including C++'s out-of-range → 0
  fallback), `get_block_by_category` (`:4161`, create-on-miss; returns an index rather than
  a pointer so the borrow checker is satisfied), `add_depth_to_block` (`:4182`),
  `generate_wipe_tower_blocks` steps 1–3 (`:4268-4315`), `reset_block_status` (`:4219`).
- **Call sites** — `generate_wipe_tower_blocks` from `plan_tower` (mirroring
  `plan_tower_new:4483/4494`), `reset_block_status` at the top of each layer in `generate`
  (`:4652`).

**Deliberately NOT ported, and why.** C++'s step 4 (`:4316-4323`) then *rewrites*
`m_plan[i].depth` from the blocks using a reverse-cumulative max over `layer_depths`. That is
a real geometry change, entangled with the two-depth problem (R509/R510) that also owns the
1.6mm finish-box span gap measured in R616 — so it is held back rather than smuggled into a
round advertised as byte-neutral. The `add_solid_flag` classification pass (`:4335-4390`) is
also out: it assigns `WipeTowerLayerType`, an enum this port lacks (we carry
`solid_infill: Vec<bool>`), so it needs the enum first.

### Measured

    TOWER_BLOCKS: layers=656 blocks=1 categories=[100] depths=[38.5]
    majora: 137de4a3   (unchanged)

Exactly as predicted: **one** block, because Majora's eight filaments all carry category
**100** — and the probe reports the real value `100`, not the `0` fallback, so the config
path works end to end rather than accidentally agreeing. The block depth **38.5**
independently matches the plate's known max toolchange depth of 38.50 (recorded at
`print.rs:2082`), which is a cross-check that steps 1–3 compute the same quantity C++ does.

Benchy `304320a6`, cube `242f1fb8`, Majora `137de4a3` all unchanged; `TOWER_BLOCKS_CPP=0`
reproduces the same Majora hash (the gate is inert by construction this round); 31 guard
tests pass. Line parity unchanged at **25.88% (660,333/2,551,163)**.

### Why a byte-neutral round was worth spending

Three rounds have now been shaped by one missing vector. R614 could not split the
finish-layer tcr; R616 established the split needs `finish_block`; `finish_block` needs a
block. That chain is now cut. The next round can port `finish_block` against real data
instead of a stub.

**R618:** port `finish_block` (`WipeTower.cpp:3743-3831`) and its `; WIPE_TOWER_START/END`
pair, emitted as a **separate** ToolChangeResult ahead of `finish_layer_new`'s — R614's
`TOWER_FINISH_ALL` iterate-all emitter is already default-on and waiting for a second
non-toolchange tcr. **Predict the finish-layer block count moves 656 → ~920 and the
per-layer histogram gains a 2-bucket (C++: `{1:386, 2:264, 3:6}`); the remaining ~12 are
`only_generate_out_wall`, which is separately gated behind `TOWER_TIMELAPSE_DEPTH`.**
Fallback: if the count moves but matched lines do NOT rise, the new block's geometry is
wrong rather than its existence — compare the emitted block against C++'s layer-138 block0
with `$D/r615_dump.py` before touching anything else, and expect the 1.6mm span gap to be
the reason (R616), which means step 4 of `generate_wipe_tower_blocks` is the real next
target.

## R618 — the `finish_block` skip is an identity without step 4; census 0 → 210

Prediction **WRONG** on the census clause, then **right in kind** after the diagnosis; the
registered fallback named the outcome both times. Byte-neutral throughout — Majora stays
`137de4a3`, benchy `304320a6`, cube `242f1fb8`. This round deliberately spent one build on a
census instead of ~90 lines of emitter, and that is what caught the problem.

### Reading first: `finish_block` needs two things we did not have

`finish_block` (`WipeTower.cpp:3743-3831`) and `finish_layer_new` (`:3550-3721`) are
structurally near-identical — both do `rectangle_fill_box` + `CP EMPTY GRID` + outer
perimeter. The difference is the fill box: `finish_block` spans `block.cur_depth` →
`block.start_depth + block.layer_depths[cur]`, i.e. only the depth this layer's tool changes
left over. Two prerequisites follow, neither of which existed:

1. **`cur_depth` must be advanced by the tool changes** — C++ does it at `:3333`
   (`+= wipe_depth - nozzle_change_depth`, sets `last_filament_change_id`) and `:3479`
   (`+= nozzle_change_depth`, sets `last_nozzle_change_id`). R617 added only
   `reset_block_status`, which *rewinds* `cur_depth`; nothing advanced it.
2. **`start_depth` must be laid out** — `update_all_layer_depth` (`:4237`) walks the blocks
   assigning `start_depth`, starting from `m_perimeter_width`. Without it every block kept
   `start_depth = 0`, so the rewind went to 0 rather than to the block's origin.

And generate() `:4751` **skips** the block entirely when the tool changes already filled it:

```cpp
if (block.cur_depth + EPSILON >= block.start_depth + block.layer_depths[m_cur_layer_id] - m_perimeter_width) continue;
```

That skip — not any property of the emitter — is what produces C++'s `{1:386, 2:264, 3:6}`.

### Landed (both `faithful_gate`, default-on, both byte-neutral)

`TOWER_BLOCKS_CPP` gains `update_all_layer_depth` (block side only) and
`charge_tool_change_to_block`; a new `TOWER_BLOCK_LAYER_DEPTH_MAX` carries step 4's
depth propagation. Plus `block_needs_finish` (the `:4751` predicate) and a
`TOWER_FINISH_BLOCK_CENSUS` probe.

### The measurement that changed the round

With prerequisites 1 and 2 in place the census reported **0 emissions on all 656 layers**.
Dumping the terms rather than guessing:

    layer=0   start=0.500 cur=17.000 layer_depth=16.500 pw=0.500 slack=-0.500
    layer=139 start=0.500 cur=28.000 layer_depth=27.500 pw=0.500 slack=-0.500
    layer=299 start=0.500 cur=33.500 layer_depth=33.000 pw=0.500 slack=-0.500
    layer=654 start=0.500 cur= 0.500 layer_depth= 0.000 pw=0.500 slack=-0.500

**The slack is exactly −`perimeter_width` on every single layer — an algebraic identity, not
a data error.** `layer_depths[i]` is the sum of layer i's `required_depth`s (that is what
`add_depth_to_block` accumulates) and `cur_depth` advances by exactly `required_depth` per
change, so `cur_depth ≡ start_depth + layer_depths[i]` and the predicate can never fire.
Layer 0 checks out to the digit: 0.5 + 16.5 = 17.0.

C++ escapes the identity through **step 4** (`:4316-4323`), the half R617 deliberately held
back, which raises each layer's depth to the running max from the layers **above**:

```cpp
block.layer_depths[layer_id] = max(block.layer_depths[layer_id], block.layer_depths[layer_id + 1]);
```

A sparse layer inherits a taller block from above, and that inherited slack is exactly what
`finish_block` fills. Porting only that half — byte-neutral, since nothing but the block
machinery reads `layer_depths` — moved the census:

| | total | per-layer |
|---|---|---|
| before | 0 | `{0: 656}` |
| **after** | **210** | `{0: 446, 1: 210}` |
| C++ (extra blocks) | 276 | `{0: 386, 1: 264, 2: 6}` |

**210 of C++'s 264 one-extra layers (79.5%)**, and none of the 6 two-extra layers — those are
`only_generate_out_wall`, which is separately gated behind `TOWER_TIMELAPSE_DEPTH`. So the
mechanism is confirmed and the remaining 54 belong to step 4's *other* half, the
`m_plan[i].depth` rewrite that is entangled with the two-depth problem (R509/R510).

C++ folds both statements into one loop; only the `layer_depths` half is ported here,
because the plan-depth half changes geometry and this round is byte-neutral by construction.

### Why the census was worth a round

Writing `finish_block` first would have produced an emitter that fires zero times, and the
zero would have looked like a broken port rather than an unfired predicate. The census cost
one build and established the target count **in advance**: R619 should see the finish-block
count go 656 → **866**, not 932.

**R619:** port `finish_block` (`WipeTower.cpp:3743-3831`) with its own
`; WIPE_TOWER_START/END` pair, emitted as a separate ToolChangeResult; `TOWER_FINISH_ALL`
(R614) is default-on and waiting. **Predict the finish-block count moves 656 → 866 (the
census's 210, now validated) and Majora matched lines rise.** Fallback: if the count reaches
866 but matched lines do not rise, the geometry is wrong rather than the count — dump against
C++'s layer-138 block1 with `$D/r615_dump.py` and expect R616's 1.6mm span gap, which points
back at step 4's plan-depth half as the next target either way.

## R619 — `finish_block` is already emitted; the defect is packaging, not absence

**Measurement only, no code changed.** R619's planned port would have been a double-emit, and
the premise check caught it before a line was written (R607's rule). Baselines untouched:
benchy `304320a6`, majora `137de4a3`, cube `242f1fb8`.

### The premise was wrong: our `finish_layer` is a MERGED emitter

R618 handed over "port `finish_block` as a separate ToolChangeResult; validated target
656 → 866". Reading our `finish_layer` first shows it already serves **both** C++ functions,
selected per layer (`wipe_tower.rs`, the `TOWER_FILL_BOX` knob):

```rust
let fill_box = if fill_box_faithful && !layer_has_toolchange {
    // finish_layer_new's whole-layer box
    BoxCoordinates::new(pw, pw, width - 2*pw, layer_depth - 2*pw)
} else {
    // finish_block's box (:3751), measured against the block's ALLOCATION
    BoxCoordinates::new(pw, fill_box_y, width - 2*pw, alloc_depth - fill_box_y)
};
```

The code's own R496/R503 notes say it outright — *"layers WITH tool changes are finished by
`finish_block` (:3733), whose fill box runs from the depth already consumed by the
toolchanges up to the block's allocation"* and *"C++'s finish-layer fill therefore comes
almost entirely from `finish_block`"*. **So `finish_block`'s content is already being
emitted** — inside `finish_layer`'s single `; WIPE_TOWER_START/END` pair. Adding a second
emitter would have laid the fill twice on exactly the 210 layers R618 identified.

This also explains R615's layer-138 dump precisely: ours is `[rect+grid][perimeter tail]`
carrying E-values `1.9016`/`2.1190`, and C++'s block0 carries `2.1190`/`1.9017` — **the same
content, in the opposite order, in one block instead of two.**

### Sizing the real work

Splitting our single block at `CP EMPTY GRID END` and comparing against C++'s two blocks on
its 264 two-block layers:

| | C++ | Rust |
|---|---|---|
| layers | 264 (2-block) | 207 with a grid (449 without) |
| fill/grid part | 5,659 lines (mean 21.4) | 4,010 (mean 19.4) |
| perimeter/wall part | 4,798 lines (mean 18.2) | 2,733 (mean 13.2) |

Two things follow. First, **207 independently corroborates R618's census of 210** — the grid
appears on essentially the layers where `block_needs_finish` is true, from a completely
different measurement path. Second, **both halves are already short**: −1,649 lines of
fill/grid and −2,065 of perimeter/wall. The packaging fix (two tcrs instead of one) is worth
207 extra marker pairs = 414 lines; the *content* gap is 3,714 lines and dominates it.

The perimeter shortfall is the gap-wall zigzag visible in C++'s block0 (`E-0.8`/`E+0.8`
strokes from `use_gap_wall` / `prime_tower_skip_points`, R499) which our tail lacks; the fill
shortfall is R616's 1.6mm span gap, owned by step 4's plan-depth half.

### Why no code shipped

The correct change is a **restructure** of a ~300-line emitter that R500/R506 record as
having been tuned to land the tower at 0.9947 — splitting it into two tcrs, reordering
perimeter before fill. That is not a change to make speculatively at the end of a round with
no measurement of the intermediate state. R618's own lesson applies in reverse: a census
before a big port is worth a round, and so is stopping one that would have been wrong.

**R620:** split `finish_layer` into two ToolChangeResults **without changing any geometry** —
tcr A = the outer perimeter, tcr B = the fill/grid — emitted in C++'s order (perimeter
first), each with its own `; WIPE_TOWER_START/END`, on the 207 layers that currently carry a
grid. `TOWER_FINISH_ALL` (R614) is default-on and will pick up the second tcr. **Predict the
finish-block count moves 656 → 863 and matched lines rise by roughly the 414 marker lines,
NOT by the full 3,714 — the content gap is a separate, larger item.** Fallback: if matched
lines move by materially more than ~414, the split is also changing the emitted geometry
(check the E-values against `$D/r615_dump.py` on layer 138), which would mean the two halves
were sharing writer state and the restructure must preserve it explicitly (R611's
raw-splice/state lesson applies to writer position and `left_to_right` here).

## R620 — the gap-wall skip points are entirely unported; ~16,200 missing lines

**Prediction RIGHT on its first clause.** Measurement and scoping only, no code changed;
baselines untouched. The item found is **four times** the size of the one R619 sized, and a
cheap config check removed its worst dependency.

### The zigzag is absent, not mis-sized

R619 named C++ block0's `E-0.8`/`E+0.8` strokes as the cause of a 2,065-line perimeter
shortfall. Counting them: **C++ 5,399, Rust 0.** Attributing by position class (R601):

| | tower toolchange | tower finish | total |
|---|---|---|---|
| C++ | **2,723** | 2,676 | 5,399 |
| Rust | 0 | 0 | **0** |

C++'s toolchange count is **exactly 2,723 — one per tool change**. This is not a
finish-block detail: the gap wall is missing from *both* block kinds, and R619's 2,065-line
figure saw only its finish-block share.

### Full pattern cost

Each gap is retract / travel / unretract:

| line kind | C++ | Rust | Δ |
|---|---|---|---|
| `G1 E-0.8000 F1800` | 5,399 | 0 | −5,399 |
| `G1 E0.8000 F1800` | 5,399 | 0 | −5,399 |
| `F600` travels | 7,884 | 2,449 | −5,435 |

**≈16,200 lines, ~2.5% of Majora's body — the largest single remaining tower item**, and 4×
R619's estimate for the entire finish-block content gap.

### Tenth unused-symbol instance

`wall_skip_points: Vec<Vec2f>` (`wipe_tower.rs:1504`) is declared, initialised `vec![]`
(`:1579`), and never populated or read. C++ fills it from `get_all_wall_skip_points` →
`get_wall_skip_points` (`:3138-3145`), **live** at `:3140` (the `:4661` call is the
commented-out one), and consumes it at `:5067`/`:5138`/`:5158` via
`contrust_gap_for_skip_points` / `remove_points_from_segment`.

### Scoping — Majora's own config removed the worst dependency

From the 3MF's `project_settings.config`:

    "enable_tower_interface_features": "0"
    "prime_tower_skip_points": "1"

The first kills **both** conditional branches inside `get_wall_skip_points`. Since
`block.layers_type` is read only inside `solid_toolchange && m_enable_tower_interface_features`,
**the `WipeTowerLayerType` enum — the dependency that has blocked `finish_block_solid` since
R616 — is not needed for this port at all.** The second confirms `m_use_gap_wall` is true, so
the consumer path is live.

For Majora the producer reduces to: per tool change, track `process_depth` per category and
push **one** point chosen by a 4-way switch on `layer_id % 4`. Remaining dependencies:
`get_block_by_category` (ported R617), `is_need_ramming` (present), `get_block_gap_width`,
`is_valid_last_layer`, and a per-layer `extra_spacing` (we hold a global one).

**R621:** port the producer `get_wall_skip_points` (`:3145-3186`, Majora-reduced form) behind
one gate, and **measure the point count before porting the consumer** — R618's census lesson.
**Predict ~2,723 points across 656 layers (one per tool change), and byte-neutral output,
since nothing consumes them yet.** Fallback: if the count is far from 2,723, `process_depth`
or the `% 4` switch is mis-derived — dump per-layer counts against C++'s per-layer
`E-0.8000` tally before writing the consumer. Then R622 ports
`remove_points_from_segment` + the wall-emission branches (`:5138`/`:5158`), which is where
the ~16,200 lines are actually realised.

## R621 — the wall skip-point producer is ported; 2,721 points, byte-neutral

**Prediction RIGHT on both clauses.** The tenth dead symbol is now populated. Output is
byte-identical by design (nothing consumes the points yet) — the pre-registered success
condition (R614/R617).

    TOWER_SKIP_POINTS: layers=656 with_points=592 total=2721
    majora: 137de4a3   (unchanged)

**Two independent cross-checks land exactly.** `total = 2721` is one point per tool change
(we plan 2,721; C++ plans 2,723, the long-known 2-toolchange difference). `with_points = 592`
matches R613's separately-measured "2,721 toolchange blocks on 592 layers" — a figure from a
different round and a different instrument.

### What was ported

`get_all_wall_skip_points` (`WipeTower.cpp:3135-3142`) and `get_wall_skip_points`
(`:3145-3186`), behind `TOWER_SKIP_POINTS_CPP` (`faithful_gate`, default-on), called from
`plan_tower` under `use_gap_wall` exactly as C++ calls it from `plan_tower_new:4559` — right
after `update_all_layer_depth`, because the points are measured from each block's
`start_depth`.

`wall_skip_points` was also the **wrong shape**: `Vec<Vec2f>` against C++'s
`vector<vector<Vec2f>>` (`WipeTower.hpp:524`). Being never populated, the wrong type had gone
unnoticed. Now per-layer, as C++ has it.

### Every simplification is measured, not assumed

Three of C++'s terms collapse on this plate, and each was pinned before being dropped:

- **The `enable_tower_interface_features` branches** (`:3190` and the per-block pass after
  it) — the 3MF sets `"enable_tower_interface_features": "0"` (R620). These are also the
  **only** readers of `block.layers_type`, which is why this port did not need the
  `WipeTowerLayerType` enum that still blocks `finish_block_solid`.
- **The `is_valid_last_layer` zeroing** (`:3159`) — probed `sum(nozzle_change_depth) = 0.00`
  across all 2,721 tool changes, so it is a no-op. The ramming structure itself is kept so
  the shape still matches C++.
- **`infill_gap_width`** — `get_block_gap_width(new_filament, false)` (`:5226`) is
  `extra_width + m_perimeter_width` for a no-ramming block (`:4439`), with
  `extra_width = (m_extra_spacing - 1) * m_perimeter_width` (`:4433`). Majora's
  `extra_spacing` is exactly **1.0** (probed `WT_PLAN`), so `extra_width = 0` and the gap
  width is **exactly** `perimeter_width` — which is also C++'s own fallback when the category
  is absent from the map (`:5232`). No approximation. The same 1.0 collapses
  `m_plan[layer_id].extra_spacing * infill_gap_width`; we hold a global `extra_spacing` rather
  than a per-layer one, so that factor is documented in place instead of faked as a field.

Benchy `304320a6`, cube `242f1fb8`, Majora `137de4a3` all unchanged; `TOWER_SKIP_POINTS_CPP=0`
reproduces the same hash; 31 guard tests pass. Line parity unchanged at **25.88%
(660,333/2,551,163)**.

**R622:** port the consumer — `remove_points_from_segment` and the wall-emission branches at
`:5138`/`:5158`, plus `contrust_gap_for_skip_points` (`:5067`) if the rib-wall path needs it.
That is where the ~16,200 lines are realised. **Predict the three line kinds move together
toward C++'s counts — `G1 E-0.8000 F1800` 0 → ~5,399, its unretract likewise, and `F600`
travels 2,449 → ~7,884 — and Majora matched lines rise by several thousand.** Fallback: if
the retracts appear but the counts land near 2,721 rather than 5,399, only the toolchange
blocks are being broken and the finish-block wall (C++'s other 2,676) uses a second emission
site — attribute with `$D/r620_attr.py` before touching the geometry, since R620 showed the
two block kinds carry almost equal shares.

## R622 — the consumer plan named the wrong function twice; plus 2,719 missing tower arcs

**Measurement and correction only, no code changed.** R622 was slated to ship the consumer
port. Checking which twin is live *before* coding (R600/R607/R615) showed the handed-over
plan named the wrong function **twice**, and the live path carries a dependency that makes it
a multi-round port. Shipping the named port would have replaced a correct wall with a wrong
one.

### Correction 1 — `generate_support_wall` (`:5081`) is DEAD

Both its call sites (`:3661`, `:4977`) are commented out. The live emitter is
`generate_support_wall_new` (`:5030`). The `remove_points_from_segment` helper and the
`:5138`/`:5158` branches the plan named belong to the **dead** twin.

### Correction 2 — the live helper is `remove_points_from_polygon`

`:510` (84 lines), reached via `contrust_gap_for_skip_points` (`:595`). The plan said to use
`contrust_gap_for_skip_points` "only if the rib-wall path needs it" — backwards. `rib_wall`
only selects the polygon **shape** (`generate_rib_polygon` vs `generate_rectange_polygon`),
and Majora sets `"prime_tower_rib_wall": "0"`, so the shape is a plain rectangle — but the
gap construction runs regardless, gated on `skip_points` (= `m_use_gap_wall` =
`"prime_tower_skip_points": "1"`).

### The live chain, pinned end to end

    generate_support_wall_new (:5030)
      -> contrust_gap_for_skip_points (:595)
        -> remove_points_from_polygon (:510)
      -> writer.generate_path (:1249)

and `generate_path` is where the missing lines are emitted, per gap (`:1302-1305`):

```cpp
retract(retract_length, retract_speed);
travel(segments[i].start, 600.);
retract(-retract_length, retract_speed);
```

**That triple is exactly the pattern R620 counted** — `G1 E-0.8000 F1800`, an `F600` travel,
`G1 E0.8000 F1800`. The mechanism was right; only its location was wrong.

### The dependency that makes this multi-round

`generate_path` branches on `m_enable_arc_fitting`, and Majora sets
`"enable_arc_fitting": "1"`, so it takes `simplify_by_fitting_arc` and builds its segment
list from `Polyline::fitting_result`, emitting arcs via `extrude_arc`. Not inert here —
measured:

| G2/G3 | object | tower toolchange | tower finish | total |
|---|---|---|---|---|
| C++ | 432,594 | **2,719** | 0 | 435,313 |
| Rust | 462,017 | **0** | 0 | 462,017 |

C++ emits **2,719 arcs inside toolchange tower blocks** — again one per tool change — from
the fillet-rounded wall (`"prime_tower_fillet_wall": "1"`). We emit zero in the tower.
Porting `generate_path` without arc fitting would add the retract triples **while introducing
a new geometry divergence** (straight corners where C++ emits arcs). That is why no code
shipped rather than a partial port.

### Separate finding, previously unrecorded

We emit **462,017 object arcs against C++'s 432,594 — 29,423 more.** Opposite sign to the
tower gap and unrelated to the tower; logged as its own item rather than folded in.

**R623 sequencing:** (a) port `remove_points_from_polygon` (`:510`) and
`generate_rectange_polygon` (`:610`) — pure geometry, byte-neutral, verifiable by point
count; **predict the gap construction yields 2,721 gaps matching R621's point count.**
(b) port `generate_path`'s **linear** branch plus the retract triple behind a gate, and
measure whether the tower's straight segments match before touching arcs; (c) arc fitting
last — it is worth a bounded 2,719 lines against the ~16,200 of (b). Fallback for (a): if the
gap count differs from 2,721, the `2.5 * m_perimeter_width` range is merging or splitting
gaps — dump the per-layer gap count against the per-layer skip-point count before proceeding.

## R623 — the wall-gap leaf helpers, ported with tests; one test caught my own error

**Code shipped**, ending the three-measurement-rounds-in-four streak. Three leaf functions of
C++'s live wall-gap chain are ported as pure functions with **11 unit tests, all passing**.
Byte-neutral by construction — nothing calls them yet — so all three baselines are unchanged.

### Why helpers first, and why tests

R622 mapped the live chain:

    generate_support_wall_new (:5030)
      -> contrust_gap_for_skip_points (:595)
        -> remove_points_from_polygon (:510)
      -> WipeTowerWriter::generate_path (:1249)

Sizing `remove_points_from_polygon` gave **267 lines** once its four helpers are counted
(84 main + 90 `add_extra_point` + 61 `move_point_along_polygon` + 17 `ray_intersetion_line` +
15 `insert_points`), plus a `Polygon::closest_point_index` this crate lacks. Writing all of
that in one round and validating it only by a gap count would repeat the mistake of the last
three rounds in a new form. **Since nothing consumes the output yet, gcode parity cannot check
this code at all** — unit tests are the only available oracle, so the leaves were ported and
tested first.

Ported: `ray_intersetion_line` (`:230`), `move_point_along_polygon` (`:337`), `insert_points`
(`:399`), plus the `PointWithFlag` / `IntersectionInfo` carriers. Left for next round:
`add_extra_point` (`:417`), `closest_point_index`, and the main function.

### The test that earned its keep

`move_backward_across_a_corner` **failed on the first run** — and the bug was in my
*expectation*, not the port. Walking 5mm backwards from (2,0) on a CCW 10mm square: 2mm
reaches the origin, then C++ (`:381-383`) computes the remainder from the edge's **far** end,

```cpp
pos = points[i+1] - ratio * (points[i+1] - points[i])
    = (0,0) - 0.3 * ((0,0) - (0,10)) = (0, 3)
```

so the answer is 3mm **above** the origin. I had asserted `(0, 7)`, measuring from the top
corner. The Rust port already produced `(0, 3)`, matching C++ exactly. The assertion was
corrected and the reasoning written into the test, since it is the natural mistake to make.

That is the point of testing a component nothing consumes: the only error this round could
surface was a silent one, and it surfaced.

The backward branch is **not** a mirror of the forward branch in C++ — it measures
`dis_from_idx` as `segmentLength - remainingDistance` and steps from `points[(i+1) % mod]`,
where forward measures `remainingDistance` from `points[i]`. Both are reproduced literally
rather than folded together, and both directions are covered by tests.

### Note on the test harness

The tests live in `crates/libslic3r-rs/tests/wall_gap_geometry.rs` rather than a
`#[cfg(test)]` module because **`cargo test --lib` does not compile in this crate** —
pre-existing and unrelated (unresolved `print_object` imports, `CSGType`, `CoolingConfig`,
`InfillPattern`). That is why all eight guard suites are integration tests; this is now the
ninth.

Benchy `304320a6`, cube `242f1fb8`, Majora `137de4a3` unchanged; the 8 existing guard suites
pass. Line parity unchanged at **25.88% (660,333/2,551,163)**.

**R624:** port `add_extra_point` (`:417`) and `Polygon::closest_point_index`, then
`remove_points_from_polygon` (`:510`) itself, still emitting nothing. **Predict the gap
construction yields 2,721 gap polylines across 592 layers, matching R621's skip-point count,
and byte-neutral output.** Fallback: if the count differs, the `2.5 * m_perimeter_width` range
is merging or splitting gaps — dump per-layer gap counts against `TOWER_SKIP_POINTS_CENSUS`
before proceeding. Extend the same test file as each piece lands; it is the only oracle until
`generate_path` is wired (R625).

## R624 — the gap constructor is ported; a test caught a real port bug this time

**Code shipped, byte-neutral, 20 tests passing** (up from 11). Unlike R623 — where the failing
test was my own wrong expectation — this round's failure was **a genuine defect in the port**.

### What landed

- **`Polygon::closest_point_index`** (`geometry/polygon.rs`) — only `split_at_index` had been
  ported; BambuStudio uses the pair together (find the index, rotate the ring to start there),
  so the lookup half was missing.
- **`add_extra_point`** (`WipeTower.cpp:417-506`) — splices `offset_to_a`, `mid`, `offset_to_b`
  into the edge whose midpoint is nearest the anchor `(bbox centre x, bbox min y)`, with the
  range clamped to `0.9 × the shorter half-edge` (`:471`).
- **`remove_points_from_polygon`** (`:510-593`) — densify, rotate to the anchor, cast one
  horizontal ray per skip point, walk `range` either way from the hit to get the gap
  boundaries, splice them in as tagged vertices, then walk the ring emitting runs and jumping
  across each tagged pair.
- **`contrust_gap_for_skip_points`** (`:595`) and **`generate_rectange_polygon`** (`:610`).
- **`to_polyline`** (`Polygon.hpp:224`) — see below.

Nothing calls any of it yet, so all three baselines are unchanged.

### The bug the test caught

`gaps_actually_remove_length` failed with

    gapped wall (144.500) should be shorter than whole (108.500)

The gapped figure is exactly right: the 35 × 38.5 wall's closed perimeter is
2 × (35 + 38.5) = **147**, and 147 − 2 × 1.25 (one gap, `range` either side of the hit) =
**144.5**. The *baseline* was wrong — 108.5 = 35 + 38.5 + 35, i.e. **three sides**.

Cause: `Polygon.hpp:224`'s `to_polyline` pushes the first point again to close the ring, and
my port omitted it, so an ungapped wall came back one side short. Fixed by porting
`to_polyline` properly and using it at both degenerate sites. The test now also pins the
ungapped perimeter at exactly 147mm, so the same omission cannot return silently.

This is the second round running where the only available oracle was a unit test, and the
second time it found something. Here it found a real defect that byte-neutrality could never
have surfaced.

### The other failure was an expectation, and the code was right

`two_skip_points_open_two_gaps` expected 2 runs and got 3. Three is correct: the walk
(`:559-588`) starts at the ring vertex nearest the anchor, which sits **mid-arc**, so that arc
is emitted as a head run and a tail run. Two cuts in a ring give two arcs, but one is split by
the start position. C++ does the same. The expectation was corrected to 3 with the reasoning
recorded.

### Coverage

20 tests: 4 on the ray intersection (hit, behind, parallel, past-the-end), 4 on the polygon
walk (forward/backward × within-edge/across-corner), 3 on vertex insertion (tag-current,
tag-next, splice), and 9 new ones on the constructor — rectangle winding, the three-vertex
splice, which edge it targets, range clamping, the empty case, one gap, two gaps, the length
identity, and the degenerate path.

Benchy `304320a6`, cube `242f1fb8`, Majora `137de4a3` unchanged; the 8 guard suites pass.
Line parity unchanged at **25.88% (660,333/2,551,163)**.

**R625:** wire it up — `generate_support_wall_new`'s live path calls
`contrust_gap_for_skip_points` and hands the runs to `WipeTowerWriter::generate_path`
(`:1249`), whose per-gap triple `retract / travel(start, 600.) / retract(-…)` is the ~16,200
missing lines. Port `generate_path`'s **linear** branch first and measure before touching arc
fitting (R622: `enable_arc_fitting` is on and C++ emits 2,719 tower arcs, so the arc branch is
its own round). **Predict `G1 E-0.8000 F1800` moves 0 → ~5,399 with its unretract and the
`F600` travels tracking it, and matched lines rise by several thousand.** Fallback: if the
retracts appear but the wall geometry regresses, the runs are being emitted in the wrong order
— `generate_path` picks its start segment by proximity (`get_closet_idx`), which this port
must reproduce rather than emitting runs in list order.

## R625 — the gap wall is wired up: +7,825 matched lines, and an 11th unpopulated field

Prediction **right** on the retract count and the line gain, **short** on the unretract and
`F600` counts; the residual is localised and named below. Majora re-baselined
`137de4a3` → **`4ffc6a0d`**. Benchy `304320a6` and cube `242f1fb8` unchanged; 8 guard suites
and the 20 wall-gap tests pass.

### Landed

`WipeTowerWriter::generate_path` (`WipeTower.cpp:1249-1309`), **linear branch**, plus the call
site in `finish_layer` mirroring `generate_support_wall_new` (`:3664`/`:5030`): build the wall
rectangle, break it at this layer's skip points via `contrust_gap_for_skip_points`, emit the
runs through `generate_path`. Behind `TOWER_WALL_GAPS_CPP` (`faithful_gate`, default-on). The
start segment is chosen by **proximity** (`:1251-1264`, `get_closet_idx`), not list order.

### An eleventh unpopulated field, found by the measurement

The retract triple fired immediately, but a grep for `G1 E-0.8000 F1800` returned **0** —
because we were writing **`F2100`**. Cause: `print.rs` built `FilamentParameters` with *only*
`nozzle_diameter` plus `..Default::default()`, so `retract_length` sat at the struct default
0.8 (which happens to match Majora) and `retract_speed` at **35**, against the 3MF's
`"retraction_speed": 30`. Every tower retract carried the wrong feedrate. Now populated from
config (`WipeTower.cpp:1793-1800`).

That fix is deliberately **ungated** — a config-plumbing correction independent of the gap
feature — so `TOWER_WALL_GAPS_CPP=0` yields `2c993879` rather than the old `137de4a3`. C++
writes `F1800`; we now do too, gaps or no gaps.

### Measured, on the file being judged

| | C++ | before | after |
|---|---|---|---|
| `G1 E-0.8000 F1800` | 5,399 | 0 | **5,280** |
| `G1 E0.8000 F1800` | 5,399 | 0 | 2,562 |
| `F600` travels | 7,884 | 2,449 | 5,011 |

**Line parity 660,333 → 668,158 matched (+7,825)**; 25.88% → **26.02%**; line-count gap
8.30% → **7.71%**. Body lines grew 2,551,163 → 2,567,537 — **+16,374** emitted against R620's
~16,200 estimate, of which about half match so far.

### The residual, named rather than hidden (R622's rule)

1. **The unretract asymmetry**, 2,562 against 5,280 retracts. C++'s `travel(start, 600.)`
   writes the feedrate **on the travel line**; this port calls `feedrate(600)` separately, so
   on roughly 2,718 gaps the following unretract omits its ` F1800` because the current
   feedrate already matches. That also explains `F600` landing at 5,011 rather than 7,884.
   An emission-shape difference, not a missing behaviour.
2. **Arc fitting is not ported** (deliberately, R622): `enable_arc_fitting` is `"1"` and C++
   emits 2,719 tower arcs against our 0, so our wall corners are straight where C++'s are
   rounded.

**R626:** fix (1) first — give the writer a `travel_to_with_feedrate(pos, f)` that writes `F`
inline as C++ does, so the unretract keeps its feedrate. **Predict `G1 E0.8000 F1800` moves
2,562 → ~5,280 and `F600` 5,011 → ~7,700, with matched lines up a few thousand more.**
Fallback: if the unretracts rise but `F600` does not, our travels are already carrying F600 on
a separate line that C++ folds in — compare layer 138 with `$D/r615_dump.py` before changing
the writer further. Then take arc fitting (2,719 lines) as its own round.

## R626 — fold the gap travel's feedrate inline: faithful, and parity-neutral

Prediction **right and exact** on the mechanism; the parity effect is **flat** and reported as
such. Majora re-baselined `4ffc6a0d` → **`bacdfefb`**. Benchy `304320a6` and cube `242f1fb8`
unchanged; 8 guard suites and the 20 wall-gap tests pass.

### R625's handover reasoning was wrong, and measuring first caught it

R625 blamed the unretract asymmetry on our gap travels leaving the following unretract without
its feedrate. Counting the actual line shapes:

| | C++ | Rust @R625 |
|---|---|---|
| `G1 E0.8000 F1800` | 5,399 | 2,562 |
| bare `G1 E0.8000` | **0** | **2,721** |
| standalone `G1 F600` | 2,485 | 5,011 |

**2,721 is exactly our tool-change count**, so the F-less unretracts are the pre-existing
R608/R611 tower wipe — not the new gap code, which was already emitting its 2,562 correctly.
And C++ *does* emit standalone `G1 F600` (2,485), so "C++ always folds F into the move" was
wrong too.

### What was actually wrong, and is now fixed

Each gap travel emitted a standalone `G1 F600` line **and** a bare `G1 X.. Y..` — two lines
where C++'s `travel(dest, 600.)` (`WipeTower.cpp:884`, via `extrude_explicit`'s
`set_format_F`) writes one. Added `travel_to_f`, which appends the feedrate to the move line,
and used it in `generate_path` (`:1304`).

**Measured: standalone `G1 F600` 5,011 → 2,449** — exactly our pre-existing count, so all
2,562 gap travels now fold their feedrate inline (2,449 + 2,562 = 5,011 F600-bearing lines,
unchanged). C++ has 2,485 standalone; we are 36 short, a separate small item.

### Parity effect, stated plainly

**Matched lines 668,158 → 668,151 — that is −7, i.e. flat.** Our body lines fell
2,567,537 → 2,564,969 as the 2,568 spurious lines went away. The rate ticks 26.02% → **26.05%**
but that is **denominator-driven, not a gain** (R599/R605), and the line-count gap actually
**widens** 7.71% → 7.80%, because we already had fewer lines than C++.

Kept anyway: it is the faithful emission shape — proven by the count landing exactly on the
pre-existing 2,449 — and it removes 2,568 lines that could never have matched. But it is not a
parity win and is not counted as one.

**R627:** the 2,721 bare `G1 E0.8000` unretracts in the pre-existing R608/R611 wipe path.
C++ has **zero** bare ones; every unretract carries `F1800`. Find the call that passes a
feedrate equal to the writer's current one (so `load`'s `f != m_current_feedrate` test
suppresses it) and give it C++'s value. **Predict bare `G1 E0.8000` 2,721 → 0, `G1 E0.8000
F1800` 2,562 → ~5,283, and matched lines up by roughly 2,700 — a genuine gain this time, since
these are existing lines gaining a correct suffix rather than new lines being added.**
Fallback: if the bare count does not move, the emitter is not `load`/`retract` but a direct
`append`, so grep for the literal before changing the writer.

## R627 — the gap-wall ironing travels lost their feedrates: +2,715 matched lines

Prediction **right on all three counts, two of them exact**. A genuine gain this time: body
lines are unchanged, so the rise is matched lines rather than a shrinking denominator. Majora
re-baselined `bacdfefb` → **`92c93130`**. Benchy `304320a6`, cube `242f1fb8` unchanged; 8 guard
suites and the 20 wall-gap tests pass.

### Located by attribution, not by guessing

All 2,721 bare `G1 E0.8000` sat in tower **toolchange** blocks (C++ has zero anywhere), and the
surrounding travels carried the trailing-space coordinates that mark tower gcode after
`transform_gcode`. That pinned the emitter to the tower writer rather than the object one, and
led to `wipe_tower.rs:3588` — our port of C++'s gap-wall ironing at `WipeTower.cpp:4085-4094`
(the `prime_tower_skip_points` ironing, R499).

### The defect

C++ gives both ironing travels an explicit feedrate:

```cpp
writer.retract(retract_length, retract_speed);
writer.travel(writer.x() - 1.5 * ironing_length, writer.y(), 600.);
writer.travel(writer.x() + 1.5 * ironing_length, writer.y(), 240.);
writer.retract(-retract_length, retract_speed);
```

We passed none. With no feedrate written, `m_current_feedrate` stayed at `retract_speed`, so
the **closing** retract's `f != m_current_feedrate` test suppressed its suffix — one bare
`G1 E0.8000` per tool change. Fixed by using R626's `travel_to_f` with 600 and 240.

### Measured, on the file being judged

| | C++ | before | after |
|---|---|---|---|
| bare `G1 E0.8000` | 0 | 2,721 | **0** |
| `G1 E0.8000 F1800` | 5,399 | 2,562 | **5,283** |
| `F240` travels | 2,723 | 0 | **2,721** |

The `F240` count lands 2 short of C++ — exactly the long-known 2-tool-change difference
(2,723 vs 2,721), not a new discrepancy.

**Parity: matched 668,151 → 670,866, i.e. +2,715** against a predicted ~2,700. Body lines
2,564,969 → 2,564,963 (−6), so this is a **real gain** — existing lines acquiring a correct
suffix — and not the denominator effect that made R626 flat. Rate 26.05% → **26.15%**;
line-count gap unchanged at 7.80%.

**R628:** tower arc fitting. `enable_arc_fitting` is `"1"` and C++ emits **2,719** `G2`/`G3`
inside tower toolchange blocks against our **0** (R622), because the fillet-rounded wall
(`prime_tower_fillet_wall: "1"`) is fitted to arcs by `simplify_by_fitting_arc` before
`generate_path` builds its segment list. **Predict tower `G2`/`G3` 0 → ~2,719 and matched lines
up a couple of thousand.** Fallback: if the arcs appear but matched lines fall, our arc
parameters (`I`/`J` centre offsets) differ from C++'s even where the endpoints agree — compare
one block with `$D/r615_dump.py` before tuning, and remember we already emit **29,423 more**
object arcs than C++ (R622), so an arc-fitting change may move that too; attribute with
`$D/r622_arc.py` before attributing any delta to the tower.

## R628 — the tower "arcs" are spiral Z-lifts, not wall arcs

Prediction **wrong on both clauses**, and the measurement refuted the premise this round
**inherited from R622**. Arc fitting is ported but left **opt-in** because it pays nothing
here; all three baselines are byte-identical to R627, 8 guard suites and the 20 wall-gap tests
pass.

### What R622 assumed, and what is actually there

R622 recorded "C++ emits 2,719 tower arcs from the fillet-rounded wall, fitted by
`simplify_by_fitting_arc`". Dumping one:

```
; WIPE_TOWER_START
G1 E-2 F1800
G17
G3 Z.7 I1.217 J0 P1  F5400
```

That is a **spiral Z-lift** in the toolchange's filament-change gcode — one per tool change —
not a wall arc. Two further facts confirm the wall cannot be the source: `prime_tower_rib_wall`
is `"0"`, and C++'s own comment in `generate_support_wall_new` (`:5059`) reads
*"rectangle_wall do nothing"* — the fillet rounding is **skipped** for a rectangular wall.
C++'s tower wall is a plain rectangle, exactly like ours.

### What was ported anyway

`WipeTowerWriter::extrude_arc` (`WipeTower.cpp:798-872` via `:894`) emitting
`G2`/`G3 X Y I J E [F]`, with the centre as an **I/J offset from the current position** and E
from the **arc's** length rather than the chord's; plus `generate_path`'s `fitting_result` walk
(`:1274-1289`). The Rust arc machinery already existed (`ArcFitter`, `PathFittingData`,
`ArcSegment`, `Polyline::simplify_by_fitting_arc`) — only the tower writer's `extrude_arc` was
missing, which is why the port was small.

### Why it is opt-in

With the gate on, measured on Majora: tower `G2`/`G3` stayed at **zero** (nothing to fit on a
rectangle) while body lines fell 2,564,963 → 2,563,779 — **−1,184 lines** — and matched lines
were **670,866, identical to R627**. A pure loss for zero gain; the 26.15% → 26.17% tick was
denominator-driven again.

The **whole** simplification step is gated, not just the arc branch: the linear branch
(`simplify` + `reset_to_linear_move`) also removed lines for no gain, and flipping only the arc
flag left the hash at `16ced1d9` rather than R627's `92c93130`. C++ runs one branch or the
other unconditionally (`:1265-1272`), so running neither is a **known deviation, recorded
rather than hidden**. The likely cause is the tolerance: C++'s `SCALED_WIPE_TOWER_RESOLUTION`
is `0.1 / SCALING_FACTOR` with `SCALING_FACTOR = 1e-6`, while ours scales by `1e5`, so the
constant is not transferable as a literal (R596/R600's units warning).

**R629:** the spiral Z-lift, now correctly identified. C++ emits `G17` then
`G3 Z<z> I<i> J0 P1 F<f>` at the head of every tower toolchange (2,719 of them, one per tool
change); we emit a plain lift. **Our object writer already produces exactly this shape**
(`writer.rs:1190`, `_spiral_travel_to_z`), so the work is reusing it in the tower's toolchange
rather than porting anything new. **Predict tower `G2`/`G3` 0 → ~2,719, plus ~2,719 `G17`
lines, and matched lines up several thousand.** Fallback: if the lift is emitted by the
filament-change TEMPLATE rather than the writer, it will be in the change_filament gcode — grep
the template for `G17`/`spiral` before touching the writer, since R426/R427 established that
template lands as raw text.

## R629 — the spiral lift fires ~6× too often; cause localised to one guard

**Measurement and localisation only, no code changed.** The round found a defect an order of
magnitude larger than the one it was sent to fix, and localised it to a single predicate whose
correct implementation already exists in a dead file.

### The cheap check came back with the opposite sign

R629 was to add the tower's 2,719 missing spiral lifts. Counting `G17` in both engines first
(R627's rule) instead showed:

| `G17` | C++ | Rust |
|---|---|---|
| object | 6,062 | **36,407** |
| tower toolchange | **2,719** | 0 |
| total | 8,781 | 36,407 |

**+30,345 excess object `G17`** — about eleven times the tower deficit — tracking R622's
separately-measured +29,423 excess object arcs, since each spiral lift is a `G17` + `G3` pair.
R622 logged that excess as an open item; this round identifies its cause.

### What is *not* the cause

Majora sets `"z_hop_types": ["Auto Lift", "Auto Lift"]`, which looked like a mis-mapping on our
side — but C++ maps `zhtAuto` to `LiftType::SpiralLift` outright (`GCode.cpp:743-744`,
`:4095-4096`), so "Auto" means spiral there too. Our enum mapping is correct.

### The cause

C++ gates the spiral on **two** conditions (`GCodeWriter.cpp:478`, `:537`):

```cpp
if (m_to_lift_type == LiftType::SpiralLift && this->is_current_position_clear())
```

Our **live** writer (`gcode/writer.rs:1183`) checks:

```rust
if self.m_to_lift_type == 1 && self.position_known
```

`position_known` is set true at the first move (`:569`, `:576`) and **never cleared**, so the
guard is effectively always satisfied. C++'s `m_is_current_pos_clear` is set *and cleared* as
the toolpath moves, which is what holds its spiral count to 6,062.

### Twelfth dead-twin instance — and this one has the correct logic

`g_code_writer.rs:1873` defines `is_current_position_clear()` over an `m_is_current_pos_clear`
field and uses it in the faithful form at `:1096`. **Nothing imports that module**; the live
path is `gcode/writer.rs`, which lacks the field entirely. The right implementation was
already written, in the file that is not used.

No code shipped: making the live writer honour a flag it does not have means porting the
set/clear sites too, and doing that half-way would change 36,407 lines on a guess.

**R630:** port `m_is_current_pos_clear` into the live writer — the field plus every site that
sets or clears it (`GCodeWriter.cpp`, and `GCode.cpp`'s callers), then use it in the spiral
guard alongside `m_to_lift_type`. **Predict object `G17` 36,407 → ~6,062 and object `G3`
462,017 → ~432,594, with matched lines up several thousand** (these are existing lines being
*removed* where C++ has none, so also check body-line count — R626's lesson). Fallback: if the
counts drop but overshoot below C++'s, our clear-sites fire more eagerly than C++'s — attribute
with `$D/r629_attr.py` (G17) and `$D/r622_arc.py` (arcs) before adjusting. Only then add the
tower's 2,719 lifts, which is the smaller half of this item.

## R630 — R629's localisation was the smaller half: the lift TYPE is chosen per travel, not once

**Two faithful ports shipped, both OPT-IN, both measured. The round's value is the correction:
`m_is_current_pos_clear` accounts for 656 of the ~29,700-line spiral-lift excess. The other
~29,000 come from a hardcoded lift type.**

### What R629 handed over, and what was wrong with it

R629 measured Majora object `G17` at 36,407 against C++'s 6,062 and localised the cause to one
guard — our live writer's spiral test used `position_known` (never cleared) where C++ uses
`is_current_position_clear()`. That reading was correct as far as it went. R630 ported the flag
faithfully: the field (`GCodeWriter.hpp:164`), the four true-sites (`GCodeWriter.cpp:410`,
`:582`, `:593`, `:622`), the clear-sites after our raw-gcode splices (`GCode.cpp:945`/`:7480`
change_filament, `:4538` timelapse; `:2729` is the default-false constructor), the `eager_lift`
guard (`:478`) and the two `travel_to_xyz` reads we had been running with the wrong flag (`:520`,
`:546`) — including C++'s "force to move xy first then z after filament change" split at `:568`
and `:606`, which we had never had at all.

**It bought 656 `G17`.** 36,407 → 35,751. The predicted ~30,345 did not arrive.

### The rest of it — `travel_lift_type`

C++ does NOT resolve `zhtAuto` to one lift type. It resolves it **per call site**:

| site | Auto resolves to |
|---|---|
| layer change (`GCode.cpp:5283`) | `SpiralLift`, forced |
| layer-change lift setup (`:4094-4096`) | `SpiralLift` |
| **every travel** (`:7046`, `:7089`, in `needs_retraction`) | **`is_through_overhang(clipped_travel) ? SpiralLift : SlopeLift`** |

Our writer passed a literal `1` (SpiralLift) at both `lazy_lift_faithful` call sites, from an R206
reading — and R206 measured **Benchy**, so the constant looked right. Both fixtures are actually
`"Auto Lift"`, and on Majora C++ takes the *slope* branch for most travels. The object-travel line
census shows it directly:

| Majora, object only | C++ | Rust |
|---|---|---|
| `G1 X Y Z` | 58,937 | 35,758 |
| `G17` | 6,062 | 35,751 |

Our `G1 X Y Z` count tracks our `G17` count exactly — every one of ours is the move after a spiral.
C++'s 52,875 surplus is the slope pre-move plus its combined move, on travels we spiral instead.

### Both halves measured, both left OFF, and why

Taking the Slope branch unconditionally under Auto — the naive completion — **regresses both
fixtures**, because `is_through_overhang` is not a detail of the rule, it IS the rule:

| gate | Majora matched | Benchy matched | Majora `G17` | Benchy `G17` |
|---|---|---|---|---|
| both off (shipped) | **670,865** | **115,901** | 35,751 | 2,040 |
| `WRITER_POS_CLEAR_CPP` | 669,555 | 115,901 | 35,095 | — |
| + `LIFT_TYPE_AUTO_CPP` | 665,793 | 112,680 | 0 | 299 |
| C++ | — | — | 6,062 | 2,029 |

Benchy is the clearest signal: C++ emits 2,029 spirals there against 300 layers, so ~1,729 of its
travels DO cross an overhang and C++ promotes every one of them. Forcing Slope drops us to 299 —
the layer-change lifts alone. Majora goes to 0 for the same reason. The demotion without the
promotion removes the right lifts in the wrong places, so both gates ship default-OFF with the
measurements recorded at the gate.

### Thirteenth dead-symbol instance

`Layer::loverhangs`, `loverhangs_with_type` and `loverhangs_bbox` are declared at `layer.rs:1274-1276`
and initialised empty at `:1327-1329`. **Nothing else in the crate mentions them** — the producer,
`PrintObject::detect_overhangs_for_lift` (`PrintObject.cpp:814-852`), was never ported. So R631 is
two ports, not one: the producer, then the predicate that reads it.

### Readings

Majora **26.15%** (670,865/2,564,962), Benchy **75.03%** (115,901/154,471) — both level with R629
(−1 and +1 line). Baselines re-taken: majora `92c93130` → **`98c75afb`**, benchy `304320a6` →
**`c93f963f`**; the drift is the `eager_lift` position-clear guard on the print's very first lift,
which is now genuinely unknown as C++ has it. Cube unchanged.

**R631:** port `PrintObject::detect_overhangs_for_lift` (`PrintObject.cpp:814-852` —
`diff_ex(layer.lslices, offset_ex(lower.lslices, scale_(line_width * 0.3)))` then
`offset2_ex(±0.1 * line_width)`, plus the support-island append and the bbox), then
`is_through_overhang` (`GCode.cpp:6972-7027`), then enable **both** gates together and re-measure.
**Predict Majora object `G17` 35,751 → ~6,062 and Benchy 2,040 → ~2,029 with matched lines UP on
both.** **Quote matched lines AND body lines — this removes ~29,000 spirals and adds ~29,000 slope
moves, so the counts move in both directions.** Fallback 1: if `loverhangs` comes back empty, the
`lslices` we diff are not populated at that stage — dump `layer.lslices.len()` before theorising.
Fallback 2: if `G17` lands near 6,062 on Majora but Benchy collapses again, our overhang polygons
are too small — check the `offset2_ex` sign convention and `line_width` units before touching the
predicate.

### Tooling correction

`benchy_integration` and `cube_integration` do **not compile** (E0282/E0432/E0433) — the same
pre-existing breakage as `cargo test --lib`, so they are not part of the runnable guard set.
`multi_material_integration::test_wipe_tower_bounds_to_polygon` fails on HEAD `de150aa` too,
verified by stashing. The suites that do run and pass: `multi_material_integration` (25/26),
`painted_cube_e2e`, `three_mf_parse`, `gcode_template`, `gcode_template_majora`, `arachne_infill`,
`wall_gap_geometry` (20).

## R631 — the overhang chain is ported end to end, and it is starved at the source

**Producer, predicate and z-window all shipped and wired; both gates stay OFF. The rate on
Majora RISES to 26.54% and that is exactly the trap — matched lines FALL on both fixtures.**

### What landed

- **`PrintObject::detect_overhangs_for_lift`** (`PrintObject.cpp:814-853`) and
  `clear_overhangs_for_lift` (`:801-810`), wired at both C++ call sites (`Print.cpp:2092`, `:2019`),
  which had been TODO comments. `Layer::loverhangs` was the thirteenth dead symbol — declared,
  initialised empty, read by nothing.
- **`Layer::loverhangs` retyped** `Polygons` → `ExPolygons` and `loverhangs_with_type`
  `Vec<(Polygon, u32)>` → `Vec<(ExPolygon, i32)>`, matching `Layer.hpp:168-170`. The overhang
  region has holes and they reach the predicate.
- **`is_through_overhang`** (`GCode.cpp:6972-7027`) and the travel clip at `:7028-7042`
  (`max_z_hop / tan(slope_threshold)` = 7.63mm here), resolved inside `needs_retraction_faithful`
  at both of C++'s decision points (`:7046`, `:7089`) into `m_pending_travel_lift_type` — C++'s
  by-reference out-param.
- **The 0.4mm z-window** (`:6977-6981`): `is_through_overhang` consults every layer whose print_z
  falls in `[print_z - 0.4, print_z]`, not just the current one. At 0.16mm layers that is three
  layers. The writer now keeps a pruned deque instead of a single layer's polygons.

### It is starved, and the census says where

Benchy, 299 layers: **107 have any overhang at all, 275 polygons in total.** Most layers produce
1-9 raw overhang slivers and the `offset2_ex(-0.1·lw, +0.1·lw)` opening takes them to zero. The
predicate consequently fires on 357 travels against C++'s 2,029.

| | C++ | off (shipped) | + `LIFT_TYPE_AUTO_CPP` | + both gates |
|---|---|---|---|---|
| Majora `G17` | 6,062 | 36,406 | 1,234 | 579 |
| Majora matched | — | **670,865** | 667,490 | 666,180 |
| Majora body | 2,827,544 | 2,564,962 | 2,514,656 | 2,514,001 |
| Benchy `G17` | 2,029 | 2,040 | 357 | 357 |
| Benchy matched | — | **115,901** | 112,756 | 112,756 |

Majora's rate reads 26.15% → **26.54%** under the gate. That number is worthless: matched lines
fell by 3,375 while the denominator fell by 50,306. Benchy has no such cover and shows the same
sign, −3,145. Both gates stay off; baselines `98c75afb` / `c93f963f` reproduce byte-exactly with
them off, which is also the regression check that the new producer is inert.

### Eliminated: the clipper backend

The obvious suspect for a too-small offset result was the `geo` clipper path. Re-running the
producer through the C++-exact `difference_clib` / `offset_expolygons_clib` / `offset2_ex_clib`
gives **byte-identical census output** — nonempty=107, sum_opened=275, same per-layer raw counts.
The backend is not the cause. (The `_clib` calls were kept: they are the faithful ones regardless.)

### What that leaves

The arithmetic is C++'s, the units are right (`line_width` reads 0.4200, and
`offset_expolygons` takes unscaled mm — confirmed against `bridge_detector.rs:656-662`, which
divides C++'s `scale_`d delta by `SCALING_FACTOR`), and the clipper backend is exonerated. So
either our `lslices` differ from C++'s at this stage, or C++'s hits come from a few large
overhang regions we are also finding but failing to intersect. Distinguishing those needs the
C++ side instrumented, not more guessing on ours.

**R632:** instrument the C++ `detect_overhangs_for_lift` to dump per-layer overhang polygon count
and total area for Benchy, and compare against the same dump from ours (`OVERHANG_LIFT_CENSUS=1`,
extended to report area). **Predict our total overhang area is < 20% of C++'s** — if so the defect
is in `lslices` or the diff, upstream of everything R631 touched. Fallback: if the areas MATCH,
the producer is right and the defect is in the predicate — compare the actual travel polylines
that C++ promotes against ours for one layer, since our clip length and bbox reject are the only
places left to differ.

## R632 — the producer was never wrong; `z_hop_type` was never read

**Instrumented the C++ producer as planned. It agrees with ours to 100.0% of area on all 299
Benchy layers. The prediction was wrong, the fallback fired, and the real defect turned out to be
a config-resolution bug that had been silently shaping the last three rounds.**

### The C++ dump settles the producer

`PrintObject::detect_overhangs_for_lift` was instrumented in the submodule to print, per layer,
`lslices` count and area, the raw `diff_ex` overhangs and their area, and the post-`offset2_ex`
result and its area. Against our `OVERHANG_LIFT_CENSUS` on the same fixture:

| Benchy, 299 layers | C++ | Rust |
|---|---|---|
| `lslices` area | 96,572.0 mm² | 96,572.0 mm² |
| raw overhang area | 421.02 mm² | 421.02 mm² |
| opened overhang area | 395.45 mm² | 395.45 mm² |

**100.0%.** Layer for layer. R631's port is exact, and R631's own framing of the result was
wrong twice over: the opening is not destructive (it keeps 93.9% of the area — the layers that
came back empty had almost no area to begin with), and the region is not too small.

The instrumentation has been reverted inside the submodule and the stock engine rebuilt; both
status checks are clean.

### The actual defect

While the C++ build ran, checking Benchy's profile chain turned up this:

```
machine  "Bambu Lab H2D 0.4 nozzle.json"  z_hop_types          = ["Auto Lift", ...]
filament "Bambu PLA Basic @BBL H2D.json"  filament_z_hop_types = ["Spiral Lift", "Spiral Lift"]
```

C++ reads `ZHopType(FILAMENT_CONFIG(z_hop_types))`, and `FILAMENT_CONFIG(OPT)` is
`m_config.OPT.get_at(get_filament_config_index(...))` (`GCode.cpp:1272`) — the filament-resolved
value. **On Benchy that is `Spiral Lift`, so C++ never calls `is_through_overhang` there at all.**
Every Benchy travel is a plain SpiralLift. That is why its 2,029 `G17` spread evenly across all
300 layers while our overhang area sits in a handful of them — a mismatch I had been reading as
evidence about the geometry.

On our side `config.z_hop_type` was **never read from the config at all**. It sat at its
`ZHopType::Auto` default (`print_config.rs:1041`). Majora happens to be "Auto Lift" with
`filament_z_hop_types` all `nil`, so it was accidentally right; Benchy was accidentally wrong.

Fixed in `apply_filament_overrides`: resolve `z_hop_types` from the config, filament override
first, machine second, mapping the four `PrintConfig.cpp:475-480` key strings. Also added
`filament_z_hop_types` to `patch_filament_overrides_in_json` so the template-visible value is
merged the way C++ merges it.

### What that buys

**Benchy is now immune to `LIFT_TYPE_AUTO_CPP` — identical hash with the gate on and off.** The
"Benchy regression" that kept both R630's and R631's gates switched off was never about
overhangs; it was this bug letting Benchy into a branch C++ never enters.

| | off (shipped) | + `LIFT_TYPE_AUTO_CPP` |
|---|---|---|
| Benchy matched | 115,900 | **115,900** (was 112,756 before this fix) |
| Benchy `G17` | 2,040 | **2,040** (C++ 2,029) |
| Majora matched | 670,865 | 667,490 |
| Majora `G17` | 36,406 | 1,234 (C++ 6,062) |

Majora is unchanged — it was always Auto — so the Auto path still loses there and the gates stay
off. But the loss is now isolated to one fixture and one mechanism.

### Fourteenth dead branch

`use_g1_travel_with_z` (`gcode/writer.rs`) tests `matches!(z_hop_type, ZHopType::Spiral)`, so with
`z_hop_type` pinned at `Auto` it had **never once been true**. Fixing the config woke it and cost
82 matched lines. It is also not a port of anything — C++ has no z_hop_type gate there; the
XY-with-Z travel comes from `travel_to_xyz`'s combined move (`GCodeWriter.cpp:565-580`), which we
already emit. Left behind `TRAVEL_G1_WITH_Z`, default off, rather than deleted.

### Readings

Benchy **75.03%** (115,900/154,472), Majora **26.15%** (670,865/2,564,962) — flat, ±1 line.
Baselines re-taken: benchy `c93f963f` → **`2a5ec3d6`**, cube `242f1fb8` → **`7497af44`** (same
filament, same fix); majora **`98c75afb`** unchanged.

**R633:** Majora is now the only fixture in the Auto branch, and its predicate under-fires 5×
(1,234 against C++'s 6,062). The producer is proven exact, so the gap is in the predicate's
INPUTS, and the prime suspect is object scope: C++'s `is_through_overhang` walks
`m_curr_print->layers_sorted_for_object(z_range...)` over **every object and every instance**
(`GCode.cpp:6984`), translating each layer's overhangs by `objects_instances_shift`, while our
writer is handed only the current object's layer. **Majora is multi-object; Benchy is not — which
is exactly why this never showed up before.** Port the object/instance loop, then re-enable both
gates and re-measure. **Predict Majora `G17` 1,234 → ~6,062 with matched lines UP.** Fallback: if
it barely moves, dump for one Majora layer the travels C++ promotes against ours — the clip
length (7.63mm) and the bbox reject are the only remaining places to differ.

## R633 — the assigned target was moot; four suspects eliminated, and the predicate still under-fires

**No parity change. The round's product is a set of eliminations backed by measurement, five new
tests, and a reusable probe. The assigned cause was refuted in the first two minutes.**

### The object/instance loop was a dead end

R632 handed over "port `layers_sorted_for_object` and the instance loop — Majora is multi-object".
It is not. Neither C++'s output nor ours contains a single `M624` or
`; start printing object, unique label id:` line, and C++ emits those whenever
`m_enable_label_object` is set, which it sets exactly when `num_object_instances > 1`. **Majora is
single-instance**, so C++'s object/instance loop collapses to one object with a zero shift — which
is what we already do. Nothing to port.

### Where the gap actually sits

| Majora, object travels | C++ | Rust |
|---|---|---|
| retractions | 41,434 | 36,406 |
| of which spiral (`G17`) | 6,062 (**14.6%**) | 1,234 (**3.4%**) |

The retraction *count* is within 12%; the *promotion rate* is 4.3× low. So the defect is in the
predicate, not in how often it is consulted.

### Four suspects eliminated, each by measurement

1. **The region does not reach the predicate** — refuted. The Majora census (the first time it was
   run on Majora at all; R631/R632 only ever ran it on Benchy) shows 655 layers, 446 with
   substantial overhang, **5,579 mm²** of opened area. A new `OVERHANG_PRED_CENSUS` probe confirms
   29,732 of 35,754 predicate calls receive a non-empty region, and the z-window holds 2 layers as
   C++'s 0.4mm `protect_z` requires at Majora's 0.3mm layer height.
2. **Coordinate frames disagree** — refuted. Travel bbox and overhang bbox coincide
   (±103mm vs ±98mm over the whole print), and **50.9%** of calls with a non-empty region have
   bbox overlap. C++'s plate-frame translation at `GCode.cpp:7038` has no analogue to miss here.
3. **`intersection_pl` mishandles a contained path** — refuted, by five new tests in
   `tests/overhang_predicate.rs`. A travel entirely inside the overhang, one crossing the boundary,
   one clear of it, one inside a hole, and a sub-millimetre one all behave correctly. This was the
   strongest hypothesis: a fully-contained open path returning empty would have produced exactly
   this signature.
4. **The clip length and min-travel threshold are wrong** — refuted. The probe reports
   `min_travel=1.0000` and `lift=0.4000`, both correct, and the clipped segments run 1.0mm to
   exactly 7.63mm — the `max_z_hop / tan(3°)` threshold, hit precisely.

**A correction on that last one:** mid-round I read the probe's scaled coordinates as 1e6-scaled
and reported that every segment was 10× under the minimum-travel threshold. `SCALING_FACTOR` in
this crate is **1e5** (`lib.rs:489`), so that was a division error in the analysis script, not a
defect in the code. The bbox-overlap and frame conclusions above are unaffected — those are ratios.

### What is left

Every input to the predicate now checks out, and it still fires 4.3× too rarely. The remaining
structural difference is the travel geometry itself: C++ tests
`Polyline(travel.points[0], travel.points[1])` where `travel` is the **avoid-crossing-perimeters
path**, which can have many points and can be routed far from the straight line; ours is always a
straight two-point segment between the last position and the target. Same first segment only when
the router did not deflect.

**R634:** instrument C++'s `is_through_overhang` to print, per call, the clipped segment endpoints
and the boolean result, and diff against the same dump from ours for one Majora layer — R632
proved this is the fast path, and every cheaper hypothesis is now spent. **Predict the two dumps
disagree on WHICH SEGMENTS are tested, not on the verdicts for shared segments** — i.e. C++'s
`travel.points[1]` differs from our target point on a large fraction of calls. Fallback: if the
segments match and the verdicts differ, the divergence is inside the overhang polygons after all
and the next step is a per-layer polygon diff, not a per-call one.

### Shipped

`tests/overhang_predicate.rs` (5 tests) and the `OVERHANG_PRED_CENSUS` probe. Both gates remain
off; all three baselines byte-identical.

## R634 — the prediction died before the build; the shortfall is the predicate's own verdict

**No parity change. The round refuted its own assigned hypothesis on a config read, eliminated the
producer on the fixture that matters, and finally measured the thing four rounds had inferred
around: the predicate's RESULT.**

### The assigned prediction was wrong, and cost nothing to find out

R633 handed over: "C++ tests the avoid-crossing-perimeters route; ours is a straight segment; the
dumps will disagree on WHICH SEGMENTS are tested." Majora's config says

    reduce_crossing_wall = 0
    max_travel_detour_distance = 0

so C++'s `m_avoid_crossing_perimeters` never engages and its travel is the same straight two-point
segment as ours. The hypothesis was dead before `devbox run bambu:build` was worth starting. (Our
side has the router as an explicit TODO at `exporter.rs:2433` with the flag defaulted false — so
even where it does matter, we are knowingly out of that branch.)

### The producer matches on Majora too

R632 proved the producer exact on Benchy; it had never been checked on the fixture that actually
enters the Auto branch. Re-instrumented C++ and ran Majora:

| Majora, 655 layers | C++ | Rust |
|---|---|---|
| `lslices` area | 1,187,681.4 mm² | 1,189,237.0 mm² |
| raw overhang area | 6,645.11 mm² | 6,589.28 mm² |
| opened overhang area | 5,635.21 mm² | **5,579.42 mm² (99.0%)** |
| overhang polygons | 2,740 | 2,681 (97.8%) |

627 of 655 layers agree within 1%. One layer (10) differs materially — 32.15 mm² against 0.36 —
and 27 differ by a sliver. Not a 4.3× effect. Eliminated.

The instrumentation is reverted inside the submodule; both status checks clean.

### The measurement that was missing

R633 logged the predicate's *inputs* and eliminated four suspects without ever logging its
*output*. Doing that:

| Majora, `LIFT_TYPE_AUTO_CPP=1` | count | rate |
|---|---|---|
| `is_through_overhang` calls | 35,754 | — |
| returning **true** | **611** | **1.7%** |
| returning false | 35,143 | 98.3% |
| consumer reads `lift_type=1` | 662 | — |
| consumer reads `lift_type=2` | 38,466 | — |
| **C++ spiral promotions** | **6,062 / 41,434** | **14.6%** |

The consumer is faithful — 611 trues in, 662 spirals out (the extra 51 are the non-travel retract
sites reading the pending value, which is correct behaviour). **The verdict is not being lost
downstream. The predicate genuinely answers false about 8.5× too often**, on inputs that have now
each been independently verified: correct region (99.0% of C++'s area), correct frame (bboxes
coincide), correct clip (1.0–7.63 mm), correct threshold (`min_travel=1.0000`), and a
geometry primitive that passes five shaped tests.

That is a real narrowing: every *input* is exonerated and the disagreement is now provably in the
test itself, on a specific and reproducible set of 35,143 calls.

### R635

Instrument C++'s `is_through_overhang` to print its per-call verdict AND the segment, run Majora,
and join the two dumps on the segment endpoints. With the router disengaged the segments are
directly comparable, so the join is exact rather than approximate. **Predict a large set of
segments where C++ says true and we say false, concentrated on the layers where C++'s bbox-reject
at `GCode.cpp:7013-7017` passes and ours rejects** — i.e. the per-polygon bbox test, the one part
of the predicate that has never been checked in isolation (R633 checked the AGGREGATE bbox overlap,
which is a different quantity). Fallback: if the two agree per-segment wherever both are called,
then C++ is being CALLED on segments we never test — count C++'s calls (not its trues) and compare
against our 35,754 before looking anywhere else.

### Shipped

`OHRESOLVE` / `OHCONSUME` logging under the existing `OVERHANG_PRED_CENSUS` probe. Both gates
remain off; all three baselines byte-identical.

## R635 — the overhang predicate was never the main mechanism

**The per-call join reframes five rounds of work. C++'s `is_through_overhang` returns true only
1,478 times on Majora — it cannot be the source of 6,062 object spirals. Roughly three quarters of
them come from a mechanism with no overhang test at all.**

### The dumps

Instrumented both of C++'s `is_through_overhang` call sites (`GCode.cpp:7046`, `:7089`) to print
the clipped segment and the verdict, and matched our `OHRESOLVE` line to the same fields and units
(C++ scales at 1e-6, we scale at 1e5, so raw coordinates are not comparable — both now print mm).

| Majora | C++ | Rust |
|---|---|---|
| predicate **calls** | **51,669** | 35,754 |
| predicate returns **true** | **1,478** (2.9%) | 611 (1.7%) |
| object `G17` emitted | **6,062** | 1,234 |

**1,478 ≠ 6,062.** The predicate accounts for at most a quarter of C++'s object spirals. Every
round from R630 onward — including this one's assigned prediction — has been treating it as the
whole mechanism.

### Where the rest come from

`GCode.cpp:4094-4096`:

```cpp
ZHopType z_hope_type = ZHopType(FILAMENT_CONFIG(z_hop_types));
LiftType auto_lift_type = LiftType::NormalLift;
if (z_hope_type == zhtAuto || z_hope_type == zhtSpiral || z_hope_type == zhtSlope)
    auto_lift_type = LiftType::SpiralLift;
```

`auto_lift_type` is then passed to **fifteen** retract sites — `:742`, `:744`, `:747`, `:775`,
`:1085`, `:4582`, `:4679`, `:4688`, `:4702`, `:4710`, `:4908`, `:4929`, `:5078`, `:5089`, `:5189`
— covering layer change, timelapse, object change, the wipe-tower entry and more. Every one of
them is an **unconditional spiral** under Auto. No overhang test, no travel geometry, no
`is_through_overhang` call. That is consistent with R629's old observation that 5,882 of C++'s
6,062 object `G17` sit nowhere near a layer change: they are scattered through the layer because
these fifteen sites are scattered through the layer.

### On the round's own prediction

Predicted: "a large set of segments where C++ says true and we say false, concentrated in the
per-polygon bbox reject." **Wrong** — the per-segment verdict gap is only 1,478 vs 611, far too
small to matter. The pre-registered fallback was right in direction: C++ *is* called on more
segments (51,669 vs 35,754, 1.45×), because those extra calls come from retract sites we never
reach. But even the fallback framed the predicate as the target, and it is not.

The remaining predicate gap (1,478 vs 611) stays open and is now correctly sized: worth roughly
870 spirals, not 4,800.

### What this costs and what it buys

Five rounds (R630-R635) were spent porting and then validating the overhang chain — the producer,
the predicate, the z-window, the clip. That work is correct and stays: the producer matches C++ to
99.0-100.0% of area, and the predicate is a faithful port. It was simply aimed at the smaller half
of the problem, and no round before this one measured C++'s spiral sources by mechanism rather
than by count.

The lesson is specific enough to be worth stating: **counting a mechanism's OUTPUT on our side and
comparing it to a TOTAL on C++'s side attributes the whole total to that mechanism.** R629's
"36,407 vs 6,062" framed the entire object-`G17` deficit as one guard's fault, and every round
since inherited it.

### R636

Port `auto_lift_type` (`GCode.cpp:4094-4096`) and the fifteen retract sites that consume it. Our
equivalents are the layer-change and splice-adjacent `retract()` calls in `print.rs` and
`exporter.rs` — all of which currently take the writer's default lift path rather than a forced
SpiralLift. **Predict Majora object `G17` 1,234 → ~5,000+ with matched lines UP; Benchy unchanged
(its filament is "Spiral Lift", so `auto_lift_type` is SpiralLift there too and the sites already
agree).** Fallback: if `G17` rises but matched lines do not, the spirals are landing at the right
count in the wrong places — attribute with `$D/r629_attr.py` before adjusting, and check the
`; WIPE_TOWER` position class, since four of the fifteen sites are tower-adjacent.

### Shipped

`OHRUST` segment logging under `OVERHANG_PRED_CENSUS`. Both gates remain off; all three baselines
byte-identical; submodule reverted and stock engine rebuilt.

## R636 — the fifteen sites are three, and they are not the answer either

**No parity change. The round built the mechanism, then measured the sites exactly and found
R635's attribution was wrong too. Object spirals remain mostly unexplained — but the unexplained
share is now pinned at ~3,900 with three candidates eliminated by count.**

### The premise check that mattered

R635 concluded "~4,584 object spirals come from `auto_lift_type`'s fifteen retract sites". Mapping
each site to its guard before wiring anything showed four of them (`:4908`, `:4929`, `:5078`,
`:5089`) are gated on `!has_wipe_tower` — and **Majora has a tower**, so they cannot fire. That
alone broke the arithmetic, so instead of porting on a guess I tagged all thirteen call sites with
a per-site counter and ran Majora:

| C++ site | calls | region |
|---|---|---|
| `:747` wipe-tower toolchange retract | 3,443 | tower |
| `:1085` wipe-tower retract | 3,443 | tower |
| `:4582` timelapse / layer retract | **656** | object |
| the other ten | **0** | — |
| **total forced-lift retracts** | **7,542** | |

**Ten of the thirteen never fire.** The object-side contribution is 656 — one per layer — not
4,584. R635's attribution is corrected: `auto_lift_type` is real and worth porting, but it is a
656-line item on the object side, not a 4,800-line one.

### The object-spiral budget, as far as it is now known

| source | C++ object `G17` |
|---|---|
| overhang predicate (R635, measured) | 1,478 |
| `auto_lift_type` site `:4582` (R636, measured) | 656 |
| **still unattributed** | **~3,928** |
| total | 6,062 |

Two thirds of C++'s object spirals still have no measured source. Candidates not yet counted:
`GCode.cpp:5283` (the layer-change retract, which forces SpiralLift for `zhtAuto` **without** going
through the `auto_lift_type` variable, so it was invisible to this round's grep) and `:3039`
(`retract(..., SpiralLift, true)`). Neither was tagged here.

### What shipped

`GCodeWriter::auto_lift_type()` (`GCode.cpp:4092-4096`) and `retract_with_lift_type()`, mirroring
C++'s explicit `LiftType` argument to `GCode::retract(bool, bool, LiftType, bool)`, plus an
`m_forced_lift_type` override that `retract()` consults before falling back to the travel
predicate. The mechanism is correct and compiles clean, but is **deliberately not wired to any
call site yet** — the census says only one object-side site matters and it is worth 656 lines, so
wiring it blind ahead of the missing two thirds would be guesswork of exactly the kind the last
four rounds have been paying for.

### On the method

This is the third round in a row where the handover's target was wrong, and the second where the
error was mine from the previous round. R635 applied "count by mechanism" to the predicate and got
a real answer, then immediately attributed the *remainder* to a mechanism it had not counted. The
rule needs its second half stated: **counting mechanism A does not license attributing the
remainder to mechanism B — count B too.** The per-site census here cost one C++ build and settled
it in one run.

**R637:** tag `GCode.cpp:5283` and `:3039` the same way (per-site counter, one build), and add a
catch-all counter inside `GCodeWriter::_spiral_travel_to_z` — the single funnel every spiral passes
through — so the per-site counts can be checked against the total and any remaining source shows
up as the unexplained balance. **Predict `:5283` accounts for ~655 (one per layer) and the funnel
total reconciles to 6,062, leaving a named residual.** Fallback: if the tagged sites still do not
sum to 6,062, the residual is being emitted from `travel_to_xyz`'s spiral branch under a lift type
set somewhere not yet inspected — dump `m_to_lift_type` at the branch instead of tagging more call
sites.

### Shipped

`auto_lift_type()` + `retract_with_lift_type()` (unwired). Both gates remain off; all three
baselines byte-identical; submodule reverted and stock engine rebuilt.
## R637 — the budget closes: 85% of C++'s spirals are EAGER, and we have one eager call site

**The funnel counter reconciles. After eight rounds of partial attribution, every spiral C++ emits
now has a named source, and the answer is a path we barely use.**

### The funnel

A counter inside `GCodeWriter::_spiral_travel_to_z` — the single function every spiral passes
through — plus tags on its two callers, run on Majora:

| source | spirals | share |
|---|---|---|
| `eager_lift` (`GCodeWriter.cpp:484`) | **7,538** | **85%** |
| `travel_to_xyz` (`GCodeWriter.cpp:542`) | 1,307 | 15% |
| **funnel total** | **8,845** | |
| gcode `G17` (cross-check) | 8,781 | |

The 64-line difference between the funnel and the gcode is spirals built into strings that are
discarded or re-emitted; it does not affect the split.

**And it reconciles with R636.** The forced-lift retract sites measured last round totalled
**7,542** calls (`:747` 3,443 + `:1085` 3,443 + `:4582` 656) — against `eager_lift`'s **7,538**.
Those sites pass `apply_instantly = true`, which is exactly what routes C++ to `eager_lift`
(immediate emission) instead of the lazy path. The two independent censuses agree to four calls.

### What that means for us

`retract(..., LiftType, apply_instantly = true)` → `eager_lift` → immediate spiral. That is C++'s
dominant spiral mechanism, and **we call our equivalent from exactly one place**:

```
crates/libslic3r-rs/src/print.rs:663:   writer.eager_spiral_lift();
```

Everything else we emit goes through `lazy_lift_faithful` and only becomes a spiral if the next
`travel_to_xyz` decides to. So we are not missing a predicate, a geometry test, or a config value —
we are missing ~7,000 invocations of the *eager* path at retract sites that C++ marks
`apply_instantly`.

### The full attribution, at last

| mechanism | C++ Majora spirals | our equivalent |
|---|---|---|
| `eager_lift` from forced/`apply_instantly` retracts | 7,538 | 1 call site |
| `travel_to_xyz` lazy path (predicate-gated) | 1,307 | 1,234 — **already close** |
| total | 8,845 | ~1,234 |

The travel path we spent R630-R635 porting is **within 6% of C++ already** (1,234 vs 1,307). It was
never the deficit. The deficit is entirely the eager path.

### Method note

This is what the "count every mechanism, and make the tally sum" rule buys. R629 attributed the
whole deficit to one guard; R635 attributed the remainder to a mechanism it had not counted; R636
counted that mechanism and found it worth 656. Only a counter at the **funnel** — the one place
everything must pass — closed the books, and it cost a single build. When a budget refuses to sum
after two attempts, instrument the funnel rather than the next candidate.

### R638

Wire `retract_with_lift_type()` (shipped R636) to route through the **eager** lift when the caller
is one of C++'s `apply_instantly = true` sites, and call it at our equivalents of `:747`/`:1085`
(wipe-tower toolchange, ~6,886 calls, tower region) and `:4582`/`:5283` (timelapse and layer
change, 656 each, object region). Our `eager_spiral_lift()` already exists and is faithful
(`GCodeWriter.cpp:456-495`); it needs the `to_lift < EPSILON` early-out honoured so it no-ops where
C++ does — that is why 7,542 retracts yield 7,538 spirals and not more.
**PREDICT Majora `G17` 1,234 → ~8,000 (tower ~2,700 + object ~5,300) with matched lines UP
substantially; Benchy unchanged.** **Quote matched AND body lines — this ADDS ~14,000 lines
(G17+G3 pairs), so the denominator moves too.** Fallback: if `G17` overshoots, our eager path is
missing C++'s `to_lift = target_lift - m_lifted; if (to_lift < EPSILON) return;` guard — check
`m_lifted` bookkeeping before adjusting call sites.

### Shipped

The funnel/caller instrumentation is reverted inside the submodule (both status checks clean, stock
engine rebuilt). No Rust change this round; both gates remain off; all three baselines
byte-identical.

## R638 — the eager path lands: +2,718 matched, and the tower's missing spirals are closed

**First parity gain since R627. Majora 26.15% → 26.26% (670,865 → 673,583 matched) with the
denominator UNCHANGED at 2,564,962 — a clean gain, not a shrinking-denominator artefact.**

### The mechanism, and why it was structural rather than fifteen call sites

R637 localised the deficit to `eager_lift`. Reading `GCode::retract` (`GCode.cpp:7097-7128`) showed
why we never reached it:

```cpp
gcode += toolchange ? m_writer.retract_for_toolchange() : m_writer.retract();  // MAY BE EMPTY
gcode += m_writer.reset_e();
if (m_writer.filament()->retraction_length() > 0 || m_config.use_firmware_retraction) {
    if (apply_instantly) gcode += m_writer.eager_lift(lift_type, toolchange);
    else                 gcode += m_writer.lazy_lift(lift_type, ...);
}
```

**The lift is not guarded by whether the retraction emitted anything.** C++'s writer-level
`retract()` returns an empty string when the filament is already retracted — and the lift still
runs. Ours began `if self.retracted { return; }`, skipping the lift entirely. This is also the
exact shape R611 hit and worked around: "adding an explicit second retract produced ZERO extra
wipes because `retract()` early-returns".

### What shipped

- **`emit_lift_after_retract()`** — the lift half of `GCode::retract` factored out of both retract
  paths, so it can run independently of the retraction early-out.
- **`m_apply_lift_instantly`** — C++'s `apply_instantly` argument; true routes to `eager_lift`
  (immediate), false to `lazy_lift` (deferred).
- **`eager_lift(lift_type)`** — `GCodeWriter.cpp:456-495`, including the `to_lift < EPSILON`
  early-out at `:459` that is why C++'s 7,542 forced retracts yield 7,538 spirals and not more.
- **`retract_for_toolchange_with_lift()` / `retract_with_lift_type(lift_type, apply_instantly)`**.
- The wipe-tower toolchange site wired to `GCode.cpp:747`'s
  `retract(..., auto_lift_type, /*apply_instantly=*/true)`.
- All behind `RETRACT_LIFT_ALWAYS` (default-ON); with it off the output is byte-identical to the
  R637 baselines, which is the regression check.

### The measurement

| Majora `G17` | C++ | before | after |
|---|---|---|---|
| object | 6,062 | 36,406 | 33,688 |
| **tower_tc** | **2,719** | **0** | **2,718** |
| total | 8,781 | 36,406 | 36,406 |

**The tower's 2,719 missing spiral lifts — open since R628 — are now 2,718, off by one.** Note the
total did not change: 2,718 spirals *moved* from the object region into the tower block, which is
where C++ emits them. That is the pre-registered fallback scenario ("if `G17` lands right but
matched lines do not rise, the spirals are in the wrong position class") resolved in the positive
direction — they moved into the right class AND matched.

| | matched | body | rate |
|---|---|---|---|
| off | 670,865 | 2,564,962 | 26.15% |
| **on** | **673,583** | **2,564,962** | **26.26%** |

Benchy 115,900 and cube `7497af44` both unchanged, as predicted — Benchy's filament resolves to
Spiral so its sites already agreed.

### On the round's prediction

Predicted `G17` 1,234 → ~8,000. **Wrong in shape**: the count did not rise at all, because the
spirals we were already emitting in the object region were the same ones, mis-placed. The gain came
from *relocation*, not addition. The prediction assumed our object spirals and C++'s were disjoint
populations; they overlap.

An intermediate step also measured as a no-op and is worth recording: restructuring `retract()`
alone (the early-out fix) left both fixtures byte-identical, because nothing set `apply_instantly`
— the lift still routed to the lazy path. The restructure was necessary but not sufficient, and
only wiring a call site made it observable.

### R639

The object region still shows 33,688 spirals against C++'s 6,062 — a 27,626 excess, now the
largest single item on this path and unchanged by this round. With the tower closed and the eager
mechanism in place, the remaining object excess is the `LIFT_TYPE_AUTO_CPP` gate's territory
(measured R631 at 1,234 when enabled, i.e. it removes ~32,000 of the 33,688). **Re-run the
three-way gate A/B (`$D/r631.sh`) now that the eager path exists** — the gate was measured as a
loss in R631 against a writer that had no eager path, so its earlier verdict is stale.
**Predict enabling `LIFT_TYPE_AUTO_CPP` on top of R638 now GAINS rather than loses, because the
object spirals it removes are replaced by correctly-placed eager ones.** Fallback: if it still
loses, attribute the removed lines with `$D/r627_attr.py` before adjusting — the slope moves it
substitutes may be the mismatch rather than the spirals it deletes.

## R639 — the gate still loses, but 4.8x less, and the residual is now exactly sized

**No parity change; gates stay off. The prediction was wrong — but the re-measurement was worth
doing, because it converts the last open item from "27,626 excess spirals" into a named,
quantified mechanism.**

### The re-measurement

R631 measured `LIFT_TYPE_AUTO_CPP` against a writer with no eager path. R638 added one, so the
verdict was re-taken on top of it:

| variant | matched | body | rate | object `G17` | tower `G17` |
|---|---|---|---|---|---|
| **base (shipped)** | **673,583** | 2,564,962 | 26.26% | 33,688 | 2,718 |
| `+LIFT_TYPE_AUTO_CPP` | 672,885 | 2,517,535 | **26.73%** | 1,193 | 2,718 |
| `+both gates` | 671,575 | 2,516,880 | 26.68% | 538 | 2,718 |
| C++ | — | — | — | 6,062 | 2,719 |

**Predicted the gate would now gain. It does not — matched falls 698.** The 26.26% → 26.73% rate
rise is the R631 trap exactly: the denominator drops 47,427 while matched drops 698.

But the loss is **4.8x smaller than R631's −3,375**, which is the eager path doing real work: a
large share of what the gate used to destroy was spirals that R638 has since relocated correctly.

### What the gate exposes

With the gate on, object `G17` collapses to **1,193** against C++'s **6,062** — a 4,869 shortfall.
R637's funnel gives the expected value directly: `eager_lift` fires 7,538 times total, the tower
takes 2,719, so **object-region eager spirals ≈ 4,819**. The two numbers agree to within 1%.

So the picture is now fully decomposed:

| Majora object `G17` = 6,062 | C++ | ours (gate on) |
|---|---|---|
| travel / lazy path | ~1,243 | 1,193 — **matches** |
| **eager path** | **~4,819** | **0** |

Our travel path is right. **We emit zero object-region eager spirals**, and that single gap is
worth ~4,819 — the entire remaining object deficit, and the reason the gate cannot pay yet: it
correctly deletes ~32,000 wrong spirals but there is nothing correct to replace them with.

### Where those 4,819 come from

R636's per-site census found only three live sites: `:747` (3,443), `:1085` (3,443), `:4582` (656).
The two tower sites total 6,886 retracts but yield only 2,719 in-tower spirals — so roughly 4,167
of them emit **outside** the `; WIPE_TOWER` markers and land in the object region. That, plus
`:4582`'s 656, accounts for the ~4,819. The sites are already identified; what is missing is that
our equivalents fire only inside the tower block.

### R640

Wire the object-region half of the tower toolchange retracts. R638 wired the `WT_START` splice
path (`print.rs:3283`), which is why our 2,718 land in-tower; C++'s `:747`/`:1085` also fire on
tool changes whose lift is emitted before the tower markers. **Check the `else` branch at
`print.rs:3285` — the non-`WT_START` path — and `:4582`'s timelapse retract, which our
`print.rs:663` eager lift may or may not already cover** (its comment cites `:3039`, which R637
measured at ONE call, so the reference is stale even though the behaviour fires per layer).
**Predict object `G17` 1,193 → ~6,000 with the gate ON, and the gate flipping from −698 to a gain
of several thousand.** Fallback: if the new spirals land in-tower again, the position class is
decided by where the splice writes them relative to `WT_START` — dump one tool change with
`$D/r615_dump.py` before moving call sites.

## R640 — the object-side eager spirals are not the tower's, and the prediction was wrong again

**No parity change; the wiring measured as a no-gain and ships opt-in. The round's value is that it
replaced an INFERENCE with a MEASUREMENT and killed a plausible-but-wrong target.**

### The premise was an inference; measuring it changed the plan

R639 handed over: "the two tower sites total 6,886 retracts but yield only 2,719 in-tower spirals,
so roughly 4,167 emit outside the markers." That was arithmetic, not observation. Classifying every
C++ object-region `G17` by its nearest preceding marker:

| after | C++ | ours (gate ON, pre-R640) | gap |
|---|---|---|---|
| `; WIPE_END` | 4,751 | 538 | −4,213 |
| `M622 J1` (timelapse, `:4582`) | **656** | **0** | −656 |
| `; update layer progress` (layer change, `:5283`) | 654 | **655** | **already correct** |

Two things fall straight out. **The layer-change site is already right** — `print.rs:663` covers it,
so R639's suspicion about it is closed. And the **timelapse retract emits nothing at all** on our
side: a clean, exactly-sized 656-line target that no previous round had isolated.

### The change, and why it does not pay

C++ has two tower toolchange retracts: `:747`, whose `toolchange_retract_str` is **prepended** to
the tower gcode (so its wipe + spiral land before `; WIPE_TOWER_START`, in the object region), and
`:1085` in `append_tcr`, after the tower's moves. R638 wired only the second. Adding the first:

| | matched | body | object `G17` | tower `G17` |
|---|---|---|---|---|
| off (shipped) | **673,583** | 2,564,962 | 33,688 | **2,718** |
| on | **673,583** | 2,562,245 | 36,406 | **0** |

**Matched is identical to the line.** The rate reads 26.26% → 26.29%, which is entirely the
denominator dropping 2,717 — the trap, for the third round running.

Worse, it **undoes R638**: the pre-tower retract fires first and lifts before `WT_START`, and then
the post-tower retract's `to_lift < EPSILON` guard suppresses the spiral R638 had correctly placed
in-tower. Tower `G17` 2,718 → 0. The call moves spirals rather than creating them, and moves them
the wrong way. Shipped behind `TOWER_PRE_RETRACT`, default **off**, with the measurement recorded at
the call site.

### What this rules out

C++'s 4,751 object spirals after `; WIPE_END` do **not** come from a second tower-toolchange
retract — we now have that call and it produces none of them. They come from ordinary object
retracts that wipe and then lift eagerly, at sites still unidentified. R639's "~4,167 outside the
markers" arithmetic was wrong: the tower sites' 6,886 retracts do not split 2,719/4,167 across the
marker, because most of them hit the `to_lift < EPSILON` guard and emit no spiral at all.

### R641

Two targets, both now exactly sized and independent:

1. **The timelapse retract, 656 lines.** `GCode.cpp:4582` —
   `if (retract_when_changing_layer) gcode += retract(false,false,auto_lift_type,true)` immediately
   before `insert_timelapse_gcode()`. Our timelapse splice (`print.rs:668-703`) has no retract at
   all. Small, isolated, and the marker classifier (`M622 J1`) verifies it directly.
2. **The 4,213 after `; WIPE_END`.** Do NOT guess the site again — instrument C++'s `eager_lift`
   with a caller tag (extend R637's `SPIRAL_FUNNEL` to print the return address or add a tag
   argument at each `GCode::retract` call site) and count which callers land outside the tower
   markers. R637's funnel proved this class of instrument settles the question in one build; R639's
   and R640's arithmetic guesses have now failed twice in a row.

**Predict the timelapse retract adds ~656 matched lines with the denominator up ~1,300 (`G17`+`G3`
pairs), and `M622 J1`-preceded spirals go 0 → ~656.** Fallback: if it emits nothing, our timelapse
splice runs when the writer is already retracted AND already lifted — check `m_lifted` at that
point, since `eager_lift`'s EPSILON guard is exactly what suppressed R640's call.

## R641 — the "timelapse retract" was a template branch: +6,560 matched

**Majora 26.26% → 26.45% (673,583 → 680,143 matched) with the denominator UP 6,558 — a genuine
addition, the largest gain since R601's chain and second only to R638 in this stretch.**

### The handover said "add a retract". It was wrong, and one dump showed why

R640 measured 656 C++ object spirals preceded by `M622 J1` and attributed them to
`GCode.cpp:4582`'s `retract(false, false, auto_lift_type, true)` before `insert_timelapse_gcode()`.
Dumping the actual C++ lines around one of them instead of trusting the attribution:

```
M622 J1
 ; timelapse with wipe tower
G92 E0
G1 X65 Y245 F20000 ; move to safe pos
G17
G3 Z.7 I1.217 J0 P1  F30000
```

The `G17` is **literal text inside the `time_lapse_gcode` template**, not emitted by
`GCode::retract` at all. The template branches:

```
{if timelapse_type == 0} ; timelapse without wipe tower
...
{elsif timelapse_type == 1} ; timelapse with wipe tower
G92 E0
G1 X65 Y245 F20000 ; move to safe pos
G17
G2 Z{layer_z} I0.86 J0.86 P1 F20000
...
```

Counting which branch each side takes settled it in one grep: C++ emits **657**
`; timelapse with wipe tower` blocks, we emitted **1**.

### The defect

`print.rs:689` hardcoded `tl_settings["timelapse_type"] = json!(0)` — the "without wipe tower"
branch. `timelapse_type` is a real config option (C++ `m_config.timelapse_type`), **Majora's is 1**,
and our own code already reads it correctly 1,400 lines away at `print.rs:2072`
(`cfg.enable_timelapse_print = self.config.timelapse_type == 1`). The template substitution simply
never asked. Fixed to `json!(self.config.timelapse_type)`.

This is the third instance of the R632 class — a config value that exists, is resolved, and is
ignored at the point of use. R632's `z_hop_type` was never read at all; R634's
`retract_when_changing_layer` is mapped to the wrong field; this one is hardcoded past a correct
value. **The standing sweep for config fields never read or mis-mapped is now overdue by three
confirmed instances.**

### Measurement

| | matched | body | rate | object `G17` | `; timelapse with wipe tower` |
|---|---|---|---|---|---|
| base | 673,583 | 2,564,962 | 26.26% | 33,688 | 1 |
| **R641** | **680,143** | **2,571,520** | **26.45%** | 34,344 | **657** (C++ 657) |

**+6,560 matched with the denominator also up 6,558** — the opposite of the shrinking-denominator
trap that produced three false positives in R631/R639/R640. The gain is ~10x the predicted 656
because the whole branch body lands, not just the spiral. Benchy `2a5ec3d6` and cube `7497af44`
byte-identical; suites unchanged.

Majora re-baselined `d16f15e6` → **`88e956a4`**.

### R642

**Target 2 from R641 is untouched and unchanged: the ~4,213 object spirals after `; WIPE_END`.**
Do NOT guess a site — R639's and R640's arithmetic guesses both failed, and R641's assigned target
turned out not to be a retract at all. Extend R637's `SPIRAL_FUNNEL` to tag WHICH `GCode::retract`
caller reaches `eager_lift` (tag argument or `__builtin_return_address(0)`) and count callers
landing outside the `; WIPE_TOWER` markers. One build.
**Also worth one cheap grep first, in the spirit of what just worked:** count
`{if ...}`/`{elsif ...}` branches across ALL templates in Majora's config
(`machine_start_gcode`, `change_filament_gcode`, `layer_change_gcode`,
`wrapping_detection_gcode`) and compare each branch marker's occurrence count between the two
outputs. R641 found a 6,560-line defect that way in about two minutes, and the same class of bug
may sit in another template.

## R642 — the template sweep finds a 12,238-line defect: `;` is only a comment if preceded by a space

**No code change; this is a localisation round. The sweep predicted in R641's handover fired
immediately and led, in four measurements, to a precise one-place root cause worth roughly twice
R641's gain.**

### Step 1: the branch sweep (as predicted)

Extracting every `{if}`/`{elsif}` branch from Majora's eight gcode templates and counting each
branch's first body line in both outputs — 24 branches checked, **6 mismatches**:

| template branch | body line | C++ | ours |
|---|---|---|---|
| `{if flush_length_2/3/4 > 1}` | `G1 X3 F12000; move aside to extrude` | **3,308** | **3** |
| `{elsif timelapse_type == 1}` | `G92 E0` | 9,498 | 6,771 |
| `{if flush_length_1 > 1}` | `; FLUSH_START` | 8,756 | 8,988 |
| `{if long_retractions_when_cut[…]}` | `M620.11 S0` | 5,448 | 5,444 |

### The `flush_length` branches are NOT a wrong-branch bug

The obvious reading — "we take the wrong branch, as in R641" — is wrong, and the discipline caught
it. `flush_count`'s formula and all three constants (`g_min_purge_volume` 100,
`g_purge_volume_one_time` 135, `g_max_flush_count` 4) are **identical** to
`GCode.cpp:918-933`. And the branches demonstrably DO fire: `; FLUSH_START` is 8,988 on our side
against C++'s 8,756, well above the 2,730 that branch 1 alone could produce.

Dumping both sides at the same point:

```
C++:  G91 / 'G1 X3 F12000; move aside to extrude' / G90 / M83
ours: G91 / 'G1 X3 F12000'                        / G90 / M83
```

We emit the branch, the `G91`, the `G90`, the `M83` and every flush move. **We drop the inline
comment**, so the line does not match.

### Root cause: the comment delimiter needs leading whitespace

Comparing lines whose comments survive against those that vanish:

| template line | our output |
|---|---|
| `G1 X165 F15000; wipe and shake` — **no space** before `;` | comment **LOST** |
| `G1 X3 F12000; move aside to extrude` — **no space** | comment **LOST** |
| `G1 X80 ; shake to put down garbage` — **space** | comment **KEPT** |

Our line handling only treats `;` as a comment delimiter when it follows whitespace; otherwise the
`;` and everything after it are swallowed as part of the preceding token (`F12000;`).

### Scope

Command lines carrying an inline comment, whole Majora file:

| | C++ | ours |
|---|---|---|
| `^[GM]\d+…;…` | **19,731** | **7,493** |

**A 12,238-line gap**, every one of which differs from C++ by a stripped comment suffix and
therefore cannot match. The biggest single contributors are `;move aside to extrude` (3,305 → 1),
`;wipe and shake` (2,724 → 0), `;do not need pulsatile flushing for start` (2,723 → 0) and
`;Compensate for filament spillage during…` (2,723 → 0) — all no-space forms. The space-form
`;shake to put down garbage` survives at 2,722 vs 2,724, confirming the discriminator.

### R643

Find the parser that splits a gcode line at `;` and make it match C++'s
(`GCodeReader`/`check_add_eol` path treat `;` as a delimiter regardless of preceding whitespace),
then re-measure. **Predict Majora matched +8,000 to +12,000 with the denominator roughly flat** —
these are lines we already emit, so the fix changes their content rather than their count; the
body-line delta should be near zero while matched rises sharply. That shape is the opposite of
R631/R639/R640's false positives and a strong check that the fix is real.
**Fallback: if matched rises far less than the 12,238 line gap, the surviving lines differ from C++
in more than the comment — diff ten of them character-by-character before widening the fix.**
**Check both other fixtures: Benchy and cube use the same templates and should also gain.**

## R643 — the comment truncation was in the cooling buffer, not the parser: +12,045

**Majora 26.45% → 26.92% (680,143 → 692,188 matched) with the body-line count BYTE-IDENTICAL at
2,571,520. Largest single gain since R601's chain.**

### R642's root cause was wrong; the discriminator is `F`, not whitespace

R642 concluded we treat `;` as a comment delimiter only when it follows whitespace, from three
examples. Splitting C++'s inline-comment command lines by whether the line carries an `F`
parameter refutes that in one measurement:

| inline-comment command lines | C++ | ours (pre-fix) |
|---|---|---|
| **no** `F` param | 7,482 | 7,475 — **fine** |
| **has** `F` param | **12,249** | **18** |

The whitespace hypothesis was a coincidence of the samples: all three no-space examples happened to
carry an `F`, and the space example did not. The real rule is that only F-bearing lines lose their
comments — and those are exactly the lines the cooling buffer rewrites.

### The defect

`gcode/cooling.rs`, two sites on the feedrate-rewrite path, both truncating at `end_pos` (the `;`):

```rust
let rest = &after_f[f_end..end_pos.saturating_sub(fpos)];   // comment excluded
...
// Not slowed, different feedrate — emit without comments
let trimmed = sline[..end_pos].trim_end();
```

The second even documented the behaviour in its own comment. C++'s cooling buffer rewrites the `F`
value in place and keeps the rest of the line; the only thing it strips is its own `;_` markers.
Both sites now keep the remainder, still passing it through `strip_cooling_markers`.

### Measurement

| | matched | body | rate |
|---|---|---|---|
| base | 680,143 | 2,571,520 | 26.45% |
| **R643** | **692,188** | **2,571,520** | **26.92%** |

**+12,045 matched, denominator unchanged to the line.** This was the pre-registered falsification
shape — the fix alters line CONTENT, not count — and is the exact inverse of the
shrinking-denominator false positives from R631/R639/R640. It also lands inside the predicted
8,000–12,000 band.

Benchy `2a5ec3d6` and cube `7497af44` are byte-identical, and Benchy's parity is unchanged at
115,900 — no regression, but no gain either: those fixtures evidently have no F-bearing template
lines with inline comments. Majora re-baselined `88e956a4` → **`430d7a2b`**.

### One thing this opened

We now emit **15,755** F-bearing inline-comment lines against C++'s **12,249** — a 3,506
overshoot. Some of those comments belong to lines C++ removes outright, or C++ strips a comment we
keep. Worth a round on its own; it is a smaller and opposite-signed error to the one just fixed.

### R644

1. **The 3,506 overshoot.** Classify our surplus F-comment lines by comment text and compare
   against C++'s, the way R642's sweep did — if a whole comment class appears only on our side,
   C++ is dropping those lines and the cooling buffer's *removal* predicate differs.
2. **Re-run the R642 branch sweep** — the other five mismatches (`timelapse_type == 1` body
   `G92 E0` 9,498 vs 6,771; `; FLUSH_START` 8,756 vs 8,988; `M620.11 S0` 5,448 vs 5,444) were
   measured before this fix and may have moved.
**Predict the sweep's `; FLUSH_START` and `M620.11` gaps are now smaller or gone, and the `G92 E0`
gap persists** (it is a bare command with no comment, so this fix cannot touch it). Fallback: if
every branch count is unchanged, they are independent defects and the sweep should be treated as
four separate open items rather than one.

## R644 — the overshoot was a comment C++ never writes; the sweep's other items are independent

**No parity change (matched flat at 692,188). Two measurements, one small faithfulness fix, and a
prediction that was half right.**

### Step 1: the 3,506 overshoot is one class

Classifying our surplus F-bearing inline-comment lines by comment text:

| comment | C++ | ours |
|---|---|---|
| `;Travel to a Wipe Tower` | **0** | **3,377** |
| `;move aside to extrude` | 3,305 | 3,541 (+236) |
| `;_EXTRUDE_SET_SPEED` | **172** | **0** |

C++ *does* pass `"Travel to a Wipe Tower"` to `travel_to` (`GCode.cpp:698-701`) — but
`GCodeWriter::emit_comment` is gated on `full_gcode_comment`, which is **off** on both fixtures, so
the string appears **zero** times in its output. We emitted it unconditionally on every tower
travel. **We do not model `full_gcode_comment` at all**, which means writer-generated comments must
simply not be written; the comments that legitimately appear in C++'s output come from gcode
TEMPLATES, which are verbatim text and unaffected.

Emitting the travel bare brings F-comment lines **15,755 → 12,378** against C++'s **12,249** — the
overshoot is now 129 rather than 3,506.

**But matched is unchanged at 692,188.** Those 3,377 lines still do not match, so they differ from
C++ in more than the comment — coordinates, feedrate, or C++ does not emit that travel at all.
This is a faithfulness fix with zero parity effect, and it is worth stating plainly rather than
banking the improved comment count as a win.

The `;_EXTRUDE_SET_SPEED` row is the opposite defect: C++ keeps that marker in its final output 172
times and we strip all of them.

### Step 2: the sweep re-run, prediction half right

| branch body | C++ | ours | vs R642 |
|---|---|---|---|
| `G92 E0` (timelapse) | 9,498 | 6,771 | **unchanged, −2,727** |
| `; FLUSH_START` | 8,756 | 8,988 | **unchanged, +232** |
| `G1 X3 F12000; move aside…` | 3,308 | 3,544 | was 3 → **R643 fixed it**, now +236 |
| `M620.11 S0` | 5,448 | 5,444 | **unchanged, −4** |

**Predicted `; FLUSH_START` and `M620.11` would shrink or vanish — they did not.** Only `G92 E0`
behaved as predicted (it persists, being a bare command R643's fix cannot touch). The pre-registered
fallback applies: these are **independent defects**, and the sweep should be tracked as four
separate open items rather than one.

### R645

**`G92 E0` is the largest at −2,727** and sits in the timelapse "with wipe tower" branch R641
enabled. We emit 6,771 against C++'s 9,498. `G92 E0` appears in that branch AND elsewhere, so
**classify by preceding marker first** (the R640 instrument) rather than assuming it is the
timelapse one — R641's `; timelapse with wipe tower` count is already exact at 657, so the branch
fires the right number of times and the deficit is likely elsewhere. **Predict the missing `G92 E0`
are NOT in the timelapse block.** Fallback: if they are, the branch body is being truncated after
its first lines — dump one whole block and compare line-for-line.
Also still open from step 1: **`;_EXTRUDE_SET_SPEED` (C++ 172, ours 0)** — we strip a marker C++
keeps, a 172-line item in `strip_cooling_markers`.

## R645 — the toolchange trailer was documented and never emitted: +5,442

**Majora 26.92% → 27.04% (692,188 → 697,630 matched). The prediction was right, and the marker
classifier pointed straight at the site.**

### The prediction held

R644 handed over `G92 E0` at −2,727 with a warning not to assume it was the timelapse block.
Classifying every `G92 E0` by its nearest preceding marker:

| preceding marker | C++ | ours |
|---|---|---|
| **`; WIPE_TOWER_END`** | **2,723** | **0** |
| `M622 J1` / timelapse | 656 | 656 — **exact** |
| `M620.11 S0`, `; FLUSH_END` | 2,723 each | 2,721 each |

**The entire deficit sits after `; WIPE_TOWER_END`**, and the timelapse block matches to the line —
confirming R641's fix and the prediction that the gap was elsewhere.

### It was three lines, not one

Dumping both sides after the marker:

```
C++:  ; WIPE_TOWER_END / M220 R / G1 F30000 / G4 S0 / G92 E0 / ; CP TOOLCHANGE END
ours: ; WIPE_TOWER_END / M220 R /                              ; CP TOOLCHANGE END
```

`WipeTower.cpp:2171-2177` is a five-call chain:

```cpp
writer.speed_override_restore();            // M220 R      <- we had this
writer.feedrate(m_travel_speed * 60.f)      // G1 F30000   <- missing
      .flush_planner_queue()                // G4 S0       <- missing
      .reset_extruder()                     // G92 E0      <- missing
      .append("; CP TOOLCHANGE END\n" ...);
```

We called only the first. **The comment already sitting above our call site named the missing
trailer verbatim** — "`G1 F30000` / `G4 S0` / `G92 E0` trailer, verified against the reference
output at cpp_majora_new.gcode:7094-7099" — so this was documented, verified against C++, and then
not written. `flush_planner_queue` did not exist on our writer at all; added it.

### Measurement

| | matched | body | rate |
|---|---|---|---|
| base | 692,188 | 2,571,520 | 26.92% |
| **R645** | **697,630** | 2,579,683 | **27.04%** |

**+5,442 matched** on +8,163 body lines (3 × 2,721 tool changes) — a 67% match rate on the new
lines, so `G92 E0` and `G4 S0` land while `G1 F30000` often does not, presumably a feedrate value
difference. Worth a follow-up but the net is clearly positive. Benchy `2a5ec3d6` and cube
`7497af44` byte-identical; suites unchanged. Majora re-baselined `3f9e7fd3` → **`cad562e8`**.

### R646

1. **The `G1 F30000` remainder (~2,721 unmatched).** Compare our emitted feedrate against C++'s at
   that site — `m_travel_speed * 60` on both, so if they differ the travel-speed value or its
   formatting differs. One grep of the two lines side by side.
2. **`;_EXTRUDE_SET_SPEED` (−172)** — we strip a marker C++ keeps 172 times. Check WHICH markers
   C++ keeps before widening `strip_cooling_markers`; it clearly keeps some and strips others.
3. **A comment in the tree that names an unimplemented behaviour is a lead, not documentation.**
   R645's target was described precisely in our own source and never acted on. **Grep the gcode
   modules for comments naming C++ line ranges next to code that does not implement them** — the
   same shape may sit elsewhere, and it is a cheap sweep of exactly the kind that paid in R642.

## R646 — the tower's travel speed was never read: +2,722, and a fourth config-read defect

**Majora 27.04% → 27.15% (697,630 → 700,352 matched) with the denominator flat (+2). Prediction
held for the second round running.**

### Step 1: the `G1 F30000` remainder

R645 added three lines per tool change and only ~67% matched. Dumping the block side by side:

```
C++:  ; WIPE_TOWER_END / M220 R / G1 F30000 / G4 S0 / G92 E0 / ; CP TOOLCHANGE END
ours: ; WIPE_TOWER_END / M220 R / G1 F9000  / G4 S0 / G92 E0 / ; CP TOOLCHANGE END
```

**Predicted the VALUE would differ, not the format — it does.** 30000 vs 9000, i.e. travel speeds
of 500 vs 150 mm/s. `WipeTower.cpp:1739` sets `m_travel_speed(config.travel_speed.get_at(...))`,
Majora's `travel_speed` is **500**, and our `WipeTowerConfig.travel_speed` was **never assigned
anywhere** — it kept `Default`'s hardcoded `150.0`.

**This is the FOURTH instance of the R632 class**, after `z_hop_type` (never read), 
`retract_when_changing_layer` (mapped to the wrong field) and `timelapse_type` (hardcoded past a
correct value). The pattern is now unmistakable: our config plumbing has a systematic hole where
a struct field is declared, defaulted, and never connected to the resolved config.

| | matched | body | rate |
|---|---|---|---|
| R645 | 697,630 | 2,579,683 | 27.04% |
| **R646** | **700,352** | 2,579,685 | **27.15%** |

**+2,722 matched — exactly one per tool change** — with the denominator flat, the R643 shape.
Benchy `2a5ec3d6` and cube `7497af44` byte-identical; suites unchanged. Majora re-baselined
`cad562e8` → **`af309663`**.

Note the tower's `travel_speed` also feeds `wipe_tower.rs:3441` and `:3824`, so other tower
feedrates were wrong by the same 150-vs-500 factor and are now corrected too — part of the +2,722.

### R647

**THE CONFIG-FIELD SWEEP IS NOW FOUR-FOR-FOUR AND MUST BE THE NEXT ROUND.** Four separate defects
of one shape have each been found by accident while chasing something else, and each was worth
thousands of lines. Do it systematically instead:

1. For every field of `WipeTowerConfig`, `PrintConfig` and the other config structs, grep for an
   assignment other than in `Default`/`new`. **Any field never assigned outside its default is a
   candidate** — that is exactly how `travel_speed` (150 vs 500) and `z_hop_type` presented.
2. For each candidate, find the C++ field it mirrors and read the value out of Majora's
   `project_settings.config`. **Report every field whose default differs from the fixture's
   configured value, with both numbers, before changing anything.**

**Predict the sweep finds at least three more fields whose default silently overrides a configured
value.** Fallback: if every field is either assigned or matches its default, the hole is narrower
than it looks — say so, and move to `;_EXTRUDE_SET_SPEED` (−172) and the remaining sweep items
(`; FLUSH_START` +232, `G1 X3 F12000; move aside…` +236).

## R647 — the config-field sweep: one struct never populated, 78,621 spurious lines removed

**Majora `af309663` → `92d8bb20`: matched 700,352 → 700,354 (+2), our body 2,579,685 → 2,501,064
(−78,621). Benchy 115,900 → 115,961 (+61). This round is a PRECISION fix, not a recall gain —
say so plainly: the rate moved 27.15% → 28.00% almost entirely because the denominator shrank,
which is exactly the trap R631/R639/R640 warn about. What makes it real is WHAT shrank: 78,621
`M106` lines C++ never emits.**

### The sweep

Four defects of one shape had each been found by accident (R632 `z_hop_type`, R634
`retract_when_changing_layer`, R641 `timelapse_type`, R646 `travel_speed`). The method: for every
field of every config struct, grep for an assignment outside `Default`/`new`; any field never
assigned elsewhere is a candidate; then read the mirrored key out of Majora's
`project_settings.config` and compare against our default.

Fields never assigned outside `Default`, by struct:

| struct | fields | never assigned | live path? |
|---|---|---|---|
| `PrintConfig` | 202 | 18 | yes |
| `PrintObjectConfig` | 179 | 2 | yes |
| `PrintRegionConfig` | 89 | **0** | yes |
| `PerimeterConfig` | 42 | **0** | yes |
| `InfillConfig` | 11 | **0** | yes |
| `GCodeConfig` | 4 | **0** | yes |
| `WipeTowerConfig` | 43 | 14 | yes |
| `CoolingConfig` | 14 | 2 | — |
| `TravelConfig` | 5 | 3 | **DEAD** |
| `MultiMaterialConfig` | 24 | 5 | **DEAD** (`to_wipe_tower_config`) |
| `ToolOrderingConfig` | 19 | 4 | **DEAD** (`ToolOrdering::new` — tests + the dead coordinator only) |
| `SeamPlacerConfig` | 3 | 1 | fallback only |

**The object-side geometry configs are clean** — `PrintRegionConfig`, `PerimeterConfig`,
`InfillConfig`, `FuzzySkinConfig`, `MedialAxisConfig`, `ExternalSurfaceConfig` have zero
never-assigned fields between them. The hole is entirely on the gcode-emission side.

Checking each live candidate against Majora's config, almost all of the WipeTower ones turn out
inert, and the honest tally is smaller than the field counts suggest:

- `no_sparse_layers` (0=false), `use_rib_wall` (0), `extra_rib_length` (0), `tower_framework` (0),
  `flat_ironing` (0), `physical_extruder_map` (`["0"]`), `first_layer_flow_ratio`
  (`initial_layer_flow_ratio`=1) — **default already equals the configured value.**
- `rib_width` (0 vs **8**) and `use_fillet` (false vs **1**) — differ, but `reads=0`: unported
  features, not config-read defects.
- `filament_change_length`/`_nc` (20 vs **10**) — differ and are read, but only under
  `is_need_ramming`, which is false for Majora once `set_filament_map(vec![0; n])` makes
  `is_same_extruder` true. **Inert.**

### The real find: `PerExtruderCoolingConfig` is never populated

`PrintConfig::per_extruder_cooling` is declared, defaulted to `Vec::new()` — and **assigned
nowhere**. So `export_gcode` always takes the fallback branch, which builds a single entry from
seven scalar fields and leaves the other **nine at `Default`**. C++'s `EXTRUDER_CONFIG` macro
(GCodeEditor.cpp:402-460) reads all sixteen.

Two of the nine differ from Majora's config:

| field | our default | Majora | read at |
|---|---|---|---|
| `reduce_fan_stop_start_freq` | `false` | **`1`** | cooling.rs:2335 |
| `additional_cooling_fan_speed` | `0` | **`70`** | cooling.rs:2340 |

Neither key existed on `PrintConfig` at all, so no `set_deserialize` handler could ever have been
written for them. Added all nine as fields + handlers, plus `machine_max_acceleration_travel` /
`_retracting`, which had the same gap.

### What `reduce_fan_stop_start_freq` was actually costing

`false` makes the base fan speed 0, so `overhang_fan_control = overhang_fan_speed(100) >
fan_speed_new(0)` is **true** and every `;_OVERHANG_FAN_START/END` pair emitted a pair of `M106`
lines. With the configured `true`, the base is `fan_min_speed`=100, `100 > 100` is false, and the
overhang fan control switches off — as it does in C++. The four-way A/B isolates it exactly:

| run | hash | matched | our body | `M106` |
|---|---|---|---|---|
| both gates off | **`af309663`** (reproduces R646) | 700,352 | 2,579,685 | 89,529 |
| cooling only | `98485914` | 700,354 | 2,501,064 | 10,908 |
| accel only | `b911c6d2` | 700,352 | 2,579,685 | 89,529 |
| both | **`92d8bb20`** | 700,354 | 2,501,064 | 10,908 |

**We were emitting 89,529 `M106` lines against C++'s 16,364 — a 73,165-line over-emission.** That
is why matched barely moved: those lines never matched anything. Note the base run reproduces
R646's hash byte-for-byte, which is what makes the other three rows trustworthy.

**The accel handlers are inert**: `accel` differs from `base` in exactly one line — the
`; estimated printing time` header (2d 12h 7m 39s → 2d 12h 10m 20s). C++ writes `M204 P20000 R5000
T20000`, i.e. the *extruding* value, because `gcode_flavor == gcfMarlinLegacy` takes that branch
(GCode.cpp:3601-3603) and `machine_max_acceleration_travel` is never consulted. Kept anyway — the
config value should reach the struct — but scored as **zero**.

Also fixed: `format_set_additional_fan` rounded where GCodeWriter.cpp:907 truncates
(`(int)(255.0 * speed / 100.0)`), so 70% gave `S179` against C++'s `S178`. Worth exactly +1 here,
but it would have poisoned every P2 line the moment R648 lands.

### R648 — the marker we classify and never emit

The P2 census after the fix is the tell: **1** `M106 P2 S178` against C++'s **2,721**. The cause is
localised and exact. C++ appends `;_FORCE_RESUME_FAN_SPEED` at GCode.cpp:944 and :7479, right
before `set_current_position_clear(false)`; `GCodeEditor.cpp:556-561` turns that marker into a
forced re-emission of *both* fans. We have the marker constant (`TYPE_FORCE_RESUME_FAN`,
cooling.rs:1404), the classifier (cooling.rs:2256) and the emitter (cooling.rs:2551-2567) — and
**we never write the marker**. Total `M106` deficit is now 16,364 − 10,908 = **5,456 ≈ 2 × 2,728
tool changes**, which is exactly two lines per tool change: the `M106 S<current>` and the
`M106 P2 S178`.

Our `print.rs:3363` already cites "GCode.cpp:945 and :7480" and quotes C++'s comment from that very
block — while omitting the line C++ writes there. **This is R645's lesson for the third time: a
comment naming a C++ line range next to code that does not implement it is a lead.**

**Predict R648 recovers ~5,456 `M106` lines and ~4,000-5,400 matched.** Fallback: if the marker
fires but the emitter stays silent, the state machine's `m_set_fan_changing_filament_start` is
never set — check the `;_SET_FAN_CHANGING_FILAMENT` producer next, same shape, same file.

## R648 — the marker we classify and never write: +5,442, every added line matches

**Majora `92d8bb20` → `50d674f9`: matched 700,354 → 705,796 (+5,442), body 2,501,064 → 2,506,506
(+5,442). Body and matched moved by the SAME amount — every line this round adds is a line C++
also has. That is a recall gain, not the precision gain R647 was. Benchy `4d8dd7ad` and cube
`ebda7d03` byte-identical.**

### The three-stage check, generalised

R647's residual pointed at `;_FORCE_RESUME_FAN_SPEED`. Before porting it, run the check over the
whole marker set — for each string the cooling classifier recognises, does anything outside
`cooling.rs`/`g_code_editor.rs` write it?

| marker | producers outside the classifier |
|---|---|
| `;_EXTRUDE_END` | 1 |
| `;_OVERHANG_FAN_START` / `_END` | 2 / 2 |
| `;_SET_FAN_SPEED_CHANGING_LAYER` | 4 |
| `; COOLING_NODE:` | 2 |
| **`;_FORCE_RESUME_FAN_SPEED`** | **0** |
| **`;set fan changing filament`** | **0** |
| **`; Slow Down Start` / `End`** | **0** |

**Three markers we classify and never produce.** The second turned out to be a non-issue and the
check is why I know that rather than guessing: `;set fan changing filament` has no producer in C++
libslic3r either — it would come from a printer profile's template, and Majora's config contains
the string nowhere. Both sides default `m_set_fan_changing_filament_start = true`
(GCodeEditor.hpp:483, cooling.rs:1820), so the gate on the P2 emission was already satisfied and
R648's pre-registered fallback was ruled out before writing any code.

### The fix

C++ appends the marker immediately after the tool-change template — GCode.cpp:944 for
`toolchange_gcode_str`, :7479 for the `set_extruder` path — and `GCodeEditor.cpp:556-561` expands
it into a forced re-emission of both fans. Dumping C++'s output around the third `M621 S4A` shows
exactly where the pair lands:

```
M621 S4A          <- the template's last line
M106 S255         <- FORCE_RESUME, main fan
M106 P2 S178      <- FORCE_RESUME, auxiliary fan
G1 X180.18 Y208.797 F30000
```

That is the head of `emit_tower_tcr`'s trailer, so the producer is one line there. Our
`print.rs:3363` had cited "GCode.cpp:945 and :7480" and quoted C++'s comment from that block for
eighteen rounds while omitting the line C++ writes at it.

| run | hash | matched | body | `M106` | `M106 P2 S178` |
|---|---|---|---|---|---|
| `FORCE_RESUME_FAN_MARKER=0` | **`92d8bb20`** (reproduces R647) | 700,354 | 2,501,064 | 10,905 | 1 |
| **`=1`** | **`50d674f9`** | **705,796** | **2,506,506** | **16,347** | **2,719** |
| C++ | | | 2,781,977 | **16,364** | **2,721** |

Predicted "~5,456 `M106` lines and ~4,000-5,400 matched": actual **+5,442 and +5,442**, at the top
of the range. The `M106` deficit closes from 5,459 to **17** and the P2 deficit from 2,720 to
**2** — and 2 is the known 2-tool-change difference, i.e. this class is now exhausted.

### R649 — the ordering the same dump hands us

The side-by-side after the fix shows the pair landing correctly and one difference remaining:

```
C++:   M621 S4A / M106 S255 / M106 P2 S178 / G1 X180.18 Y208.797 F30000 / G1 Z1.5 / (blank) / ; filament start gcode / M106 P3 S150
ours:  M621 S4A / M106 S255 / M106 P2 S178 /                                       ; filament start gcode / M106 P3 S150 / G1 X185.729 Y199.297 F30000
```

**C++ emits the travel BEFORE the filament-start template; we emit it after** — and C++ has a
`G1 Z1.5` we do not emit at all. Our trailer is built `{fil_start}{travel_to_start}…` on the
strength of a comment citing GCode.cpp:1051; the observed output contradicts that reading, so
**check which C++ path actually produces this block before reordering** (R642's lesson: a root
cause from a code comment is a hypothesis). At ~2,723 tool changes, the misordered travel plus the
missing `G1 Z` is worth up to ~5,400 lines.

**Predict R649 recovers ~2,700 (the travel alone, if order is what blocks the match) to ~5,400
(travel + the missing `G1 Z`).** Fallback: if reordering moves matched by less than 500, the
grouping in `line_parity.py` is order-insensitive within a feature and the real defect is the
absent `G1 Z1.5` — port that instead and re-measure. Second lead from the same sweep:
`; Slow Down Start`/`End` (GCode.cpp:6597/6768) has no producer, so `not_join_cooling` is never
set and we slow down paths C++ leaves alone — check what drives `use_seperate_speed` first, since
it is the circle-compensation path and may be inert on both fixtures.

## R649 — the inherited ordering premise was WRONG; the real defect was a second `flush_planner_queue`

**Majora `50d674f9` → `dee06c25`: matched 705,796 → 708,517 (+2,721), body 2,506,506 → 2,509,227
(+2,721). Recall again — body and matched move together. Benchy `4d8dd7ad` and cube `ebda7d03`
byte-identical.**

### The premise, refuted

R648's handoff claimed: "C++ emits `travel_to_start` BEFORE the filament-start template; we emit it
after." **That is wrong, and reading the C++ before touching the code is what caught it.**

`GCode.cpp:1051` really does assemble `start_filament_gcode_str + wipe_next_start_point_str +
toolchange_unretract_str` — template, then travel, then unretract — which is exactly our
`{fil_start}{travel_to_start}G1 E…`. **Our ordering already matched C++.** A longer dump confirms
it: C++ has a travel on *both* sides of `; filament start gcode`, and we match the one after it.

```
C++:   M621 S4A / M106 S255 / M106 P2 S178 / G1 X180.18 Y208.797 F30000 / G1 Z1.5 /
       ; filament start gcode / M106 P3 S150 / G1 X185.729 Y208.797 Z1.9 / G1 Z1.5 /
       G1 E2 F1800 / G4 S0 / ; CP_TOOLCHANGE_WIPE
ours:  M621 S4A / M106 S255 / M106 P2 S178 /
       ; filament start gcode / M106 P3 S150 / G1 X185.729 Y199.297 F30000 /
       G1 E2.0000 F1800 /        / ; CP_TOOLCHANGE_WIPE
```

The travel C++ has *before* the template is a different thing entirely: `travel_to_wipe_tower_gcode`
(GCode.cpp:1002-1015), the intermediate points of the avoid-crossing-perimeters detour, appended to
`toolchange_gcode_str` in the `is_used_travel_avoid_perimeter` branch — a path we do not implement
at all. Reordering our trailer would have moved a correct line to a wrong place.

I also checked and discarded a second hypothesis on the way: that the travel came from the tower's
own gcode between `[change_filament_gcode]` and `[filament_start_gcode]`. WipeTower.cpp:2465-2483
shows those two placeholders are adjacent — the travel block between them is `#if 0`'d out with the
comment "BBS: do travel in GCode::append_tcr() for lazy_lift".

### What the dump actually showed

One line in C++'s block had no counterpart anywhere in ours: **`G4 S0`**, between the unretract and
`; CP_TOOLCHANGE_WIPE`. A census sized it immediately:

| class | C++ | ours @R648 |
|---|---|---|
| **`G4 S0`** | **5,446** | **2,721** |
| `G1 Z<only>` | 47,528 | 36,410 |
| `; CP_TOOLCHANGE_WIPE` | 2,723 | 2,721 |

5,446 = 2 × 2,723; we had exactly one per tool change. `WipeTower::toolchange_Change` closes with

```cpp
writer.append("[filament_start_gcode]\n");
writer.flush_planner_queue();          // WipeTower.cpp:2485
```

**R645 ported the *other* `flush_planner_queue`** — WipeTower.cpp:2173/3339, the `; WIPE_TOWER_END`
trailer — and this one stayed missing. C++ has five call sites; we had one live.

| run | hash | matched | body | `G4 S0` |
|---|---|---|---|---|
| `TOOLCHANGE_FLUSH_QUEUE=0` | **`50d674f9`** (reproduces R648) | 705,796 | 2,506,506 | 2,721 |
| **`=1`** | **`dee06c25`** | **708,517** | **2,509,227** | **5,442** |
| C++ | | | 2,781,977 | **5,446** |

+2,721 matched, +2,721 body — the residual 4 is the known 2-tool-change difference, so this class
is exhausted too. The predicted range (~2,700–5,400) contained the answer, but **the mechanism was
not the predicted one**: the number landed inside only because both candidate defects happen to be
one line per tool change.

### R650

The same dump leaves two sized, unexplained differences in this block:

1. **`G1 Z<only>` — C++ 47,528, ours 36,410, deficit 11,118 ≈ 4 × 2,723.** In C++'s toolchange block
   the travel carries a Z (`G1 X185.729 Y208.797 Z1.9`) and is followed by a bare `G1 Z1.5`; ours
   emits `G1 X… Y… F30000` with no Z at all. That is the lazy-lift restore. **Census the deficit by
   region (tower vs object) before porting — 4× per tool change is suspiciously neat and may be two
   separate causes.**
2. **The avoid-crossing-perimeters detour** (GCode.cpp:966-1017) — unimplemented, and now confirmed
   to produce real output lines rather than being inert. This is the `exporter.rs:2433` TODO with a
   measured consequence for the first time.

**Predict R650's Z census splits the 11,118 into a tower-block term of ~5,446 (two per tool change,
the travel's Z rider plus the following bare `G1 Z`) and an object-side remainder of ~5,700.**
Fallback: if the tower term is not ~5,446, the deficit is not the toolchange lift and the census
tells you where it actually is — follow that, do not port the lift on faith.

## R650 — the Z census: prediction held on both terms, and the tower half is now closed

**Majora `dee06c25` → `c00edd1d`: matched 708,517 → 711,537 (+3,020), body 2,509,227 → 2,512,604
(+3,377). Benchy `4d8dd7ad` and cube `ebda7d03` byte-identical.**

### The census (the whole point of the round)

Splitting `G1 Z<only>` by region — inside `; WIPE_TOWER_START`…`_END` versus everywhere else:

| region | C++ | ours @R649 | deficit |
|---|---|---|---|
| tower | 10,892 | 5,442 | **5,450** |
| object | 42,095 | 36,422 | **5,673** |

**Predicted "a tower-block term of ~5,446 and an object-side remainder of ~5,700" — measured 5,450
and 5,673.** Both terms held, so the deficit really is two separate causes and porting one blind
would have been half a fix.

Splitting again by whether the line carries an `F` sharpened it to a single class:

| | C++ | ours |
|---|---|---|
| tower, `hasF` | 5,446 | 5,442 |
| tower, **`noF`** | **5,446** | **0** |

The F-bearing tower Z lines (after `M204` and after `M400`) already matched. **Every missing tower
Z line was a bare `G1 Z<z>` following a travel** — C++'s `travel_to` is an XY move *followed by* a
`travel_to_z` (GCodeWriter.cpp:626), and we emitted only the XY half. Two per tool change: one for
the `wipe_next_start_point` travel, one for the avoid-perimeter detour travel we do not emit at all.

### The fix

One line at `emit_tower_tcr`'s `travel_to_start`, formatted with `format_gcode_value(_, 3)` so
`1.5` → `1.5` and `0.3` → `.3` — verified against C++'s `G1 Z.3` at `cpp_majora_new.gcode:8582`.

| run | hash | matched | body | bare `G1 Z` |
|---|---|---|---|---|
| `TOWER_TRAVEL_Z=0` | **`dee06c25`** (reproduces R649) | 708,517 | 2,509,227 | 36,410 |
| **`=1`** | **`c00edd1d`** | **711,537** | **2,512,604** | **39,787** |

+3,377 lines emitted, **+3,020 matched** — 89%. The tower `noF` class goes 0 → 2,721 (one per tool
change, exactly as predicted) and 656 more land in the once-per-layer finish-tower blocks, which
also match. Note the honest shortfall: 357 of the added lines do **not** match, a small precision
cost against a clear recall win.

### What remains, sized

Re-censusing after the fix:

| class | C++ | ours @R650 | remaining |
|---|---|---|---|
| tower `noF` | 5,446 | 2,721 | **2,725** — the detour travel |
| object `noF` | 42,089 | 37,073 | **5,016** |
| **`M204`** | **63,626** | **51,567** | **12,059** (newly sized) |

Two object-side sub-classes are already localised by preceding line: **656** C++ Z lines follow
`; SKIPPABLE_END` (the timelapse block — exactly the layer count, and C++ also emits an `M204
S10000` there that we lack), and **211** follow an `M73`, which are ordinary travel-then-drop pairs
with a progress line interleaved.

### R651

**The `M204` deficit (12,059) is now the largest single sized class and has never been examined.**
Census it the same way — by region and by preceding line — before touching anything. **Predict it
splits with a per-layer term near 656 (the `; SKIPPABLE_END` site found above) and a much larger
per-feature term**, because C++ resets acceleration per extrusion role and we appear to emit
`M204` only at block boundaries. Fallback: if the census shows the deficit concentrated in the
tower instead, it is the same detour-travel gap as the remaining tower Z lines — port the detour
(GCode.cpp:966-1017) and both classes close together.

## R651 — the M204 census, a correction to my own reading, and proof the primary metric is order-blind

**Majora `c00edd1d` → `3d741dde`: matched 711,537 → 711,537, body 2,512,604 → 2,512,604. ZERO on
both.** Benchy 115,961/154,472 unchanged; all three hashes changed (`4d8dd7ad` → `248ff22a`,
`ebda7d03` → `14566293`) so lines *moved* on every fixture and nothing matched differently.

### The census

`M204` by region and by immediately-preceding line:

| | C++ | ours | delta |
|---|---|---|---|
| tower | 11,853 | 8,955 | −2,898 |
| object | 51,774 | 42,613 | −9,161 |

Preceding-line breakdown surfaced the striking rows — C++ has **16,932** `M204` whose previous line
is `; WIPE_START` and we had **zero**; C++ has 2,720 after `; CP_TOOLCHANGE_WIPE` and we had zero.
Against that, *we* had thousands C++ lacks: `; LINE_WIDTH`-preceded 4,491 (C++ 38), `G2`-preceded
9,165 (C++ 3,273), `G3`-preceded 2,566 (C++ 1,273).

**That pattern is a position difference, not a presence difference — and I should have read it that
way immediately.** A preceding-line census measures *where* lines sit, not *whether* they exist. The
16,932 was never 16,932 missing lines.

The C++ contract is exact: 16,931 of the 16,932 are `M204 S10000` = `default_acceleration`, emitted
between the `; WIPE_START` tag (GCode.cpp:408) and the wipe's first `extrude_to_xy`. Our
`writer.rs:2148` had flushed the pending acceleration one line **earlier** — R227 had the mechanism
right and the side of the marker wrong.

### The fix, and its zero

Moving `flush_pending_accel()` to after the marker put the lines where C++ has them: **0 → 15,081**
(C++ 16,932). The `M204` total is unchanged at 51,567, as it must be — no line was created or
destroyed.

| run | hash | matched | body | `M204` | after `; WIPE_START` |
|---|---|---|---|---|---|
| `WIPE_ACCEL_AFTER_MARKER=0` | **`c00edd1d`** (reproduces R650) | 711,537 | 2,512,604 | 51,567 | 0 |
| `=1` | `3d741dde` | **711,537** | **2,512,604** | 51,567 | **15,081** |

**Kept** — it is verifiably what C++ does — but scored honestly as **zero**, the R644 outcome.

### The finding that actually matters

`scripts/line_parity.py` groups by `(layer, feature)` and matches by content, so **it cannot see
intra-group ordering at all**. 15,081 lines moved to their correct C++ position and the primary
metric did not register a single one. R649's fallback hypothesised this; R651 proves it.

That is a hole in the measurement, not just in the port, and it matters directly for the standing
bar ("all lines are essentially exactly the same"): a file could reach a high line-parity rate with
every feature's internals shuffled. **Every ordering defect found so far has been invisible to the
number we have been steering by.**

### R652 — build the order-sensitive metric first

Before chasing another line class, add a **sequence metric**: within each `(layer, feature)` group,
compare the two line sequences with an LCS/diff rather than a multiset, and report
`in-order-matched / matched`. Run it against the current baselines to size how much of the existing
28.32% is order-correct. **Predict the sequence rate comes in materially below 28.32%** — R650's
`G1 Z` and R651's `M204` both landed as content matches whose positions were only sometimes right.
Fallback: if the sequence rate is within a point of the content rate, ordering is already fine
almost everywhere, this round's fix was worth even less than it looks, and the remaining 12,059
`M204` deficit is genuinely missing lines — go back to the census and find their producer.

The M204 deficit itself is untouched and still unexplained: −2,898 tower and −9,161 object, now
known **not** to be the wipe ordering.

## R652 — the order-sensitive metric: 243,000 Majora lines are right but in the wrong place

**New instrument `scripts/seq_parity.py`. Both clauses of the prediction held, and the control
retroactively re-scores R651 from zero to +7,740.**

### What it is

v3 `line_parity.py` matches by tolerant **multiset** intersection inside each `(layer, feature)`
group — its own docstring calls that an upper bound, but the practical consequence went unnoticed
for fifty rounds: it cannot see intra-group ordering at all. v2 `line_align.py` does respect order
but aligns on *structural keys* (`G1 X# Y# E#`), which are degenerate, so its pairs mix true
matches with coin flips.

`seq_parity.py` keeps v3's grouping **and** v3's quantisation — a match is still "the same line to
1e-3 mm", never an arbitrary pairing — and replaces the multiset intersection with a
longest-matching-block walk (`difflib`, `autojunk=False`) over the two sequences. A line counts only
if it is essentially identical **and** reachable in order, so `in_order <= matched` always and the
gap between them *is* the ordering defect. difflib's walk is a heuristic rather than a strict LCS,
so `in_order` is a lower bound — it never over-reports, which is the safe direction. Groups are
small (largest observed 7,360 lines), nothing was skipped, and the cap that would report a skip is
in the code.

### The readings

| fixture | content | **in order** | misordered |
|---|---|---|---|
| Benchy | 75.07% (115,961/154,472) | **63.61% (98,259)** | **17,702 — 15.27% of content matches** |
| Majora | 28.32% (711,537/2,512,604) | **18.65% (468,570)** | **242,967 — 34.15% of content matches** |

**A third of every Majora line we get right is in the wrong place.** Per feature, in-order share of
our own lines:

| feature | ours | content-matched | in order |
|---|---|---|---|
| Outer wall | 796,792 | 165,733 | **71,753 (9.0%)** |
| Prime tower | 488,204 | 330,542 | 269,323 (55.2%) |
| Inner wall | 434,746 | 56,821 | 31,232 (7.2%) |
| Floating vertical shell | 310,855 | 42,120 | 15,341 (4.9%) |
| (pre-feature) | 22,205 | 20,789 | 20,787 (93.6%) |

The wipe-tower work of the last fifty rounds shows up exactly where it was done: Prime tower is the
only large feature above 50% in order. The object-side walls are the opposite — Outer wall matches
20.8% of its lines by content but only 9.0% in order.

### The control, and R651 re-scored

R651's `WIPE_ACCEL_AFTER_MARKER` moved 15,081 lines into their C++ position and scored **zero** on
the content metric. Under the new one:

| run | content | in order |
|---|---|---|
| `WIPE_ACCEL_AFTER_MARKER=0` | 711,537 | 460,830 (18.34%) |
| `=1` | 711,537 | **468,570 (18.65%)** |

**+7,740 in-order lines.** The metric registers precisely the change the old one could not see,
which is the validation this instrument needed: it was not tuned to produce a number, it was checked
against a known-good change whose size was established independently. **R651 is re-scored from ZERO
to +7,740 in-order.**

`$D/r643_m.sh` now prints both rates on every round.

### R653

The ordering loss is 242,967 lines and it is concentrated in the object walls, not the tower.
**Localise it the way R650 localised the Z deficit: pick the worst feature (Outer wall, 9.0% in
order against 20.8% by content) and dump the first diverging block against C++** — `seq_parity`'s
matching blocks give the exact indices where the sequences part company, so extend it with a
`--dump-first-divergence <feature>` mode rather than eyeballing.

**Predict the Outer wall divergence is a systematic per-loop emission-order difference (seam/segment
start, or the `; LINE_WIDTH:`/`M204` interleave R651 exposed), not scattered noise** — 9.0% in-order
against 20.8% content-matched is far too structured to be random. Fallback: if the diverging
positions are scattered with no repeating shape, the cause is upstream geometry (segment
subdivision) rather than emission order, and the right target is the Arachne/fill path instead —
say so and go back to the M204 producer hunt (−2,898 tower / −9,161 object).

## R653 — the ordering loss has a shape: 148,548 inverted `; LINE_WIDTH:` / `G1 F` pairs

**Prediction held. The divergence is systematic, not scattered — and both worst Outer wall groups
part company at index 0, the very first line.** No engine code changed; all three hashes unchanged.

### The tool

`scripts/seq_parity.py --dump-divergence "<feature>" [n]` — for the n worst groups of a feature, it
uses difflib's matching blocks to find the first index our sequence stops tracking C++'s and prints
both neighbourhoods side by side with indices. No eyeballing: the blocks already know where the
sequences part.

### What it showed

```
=== layer 163  ours 2801 lines, cpp 3184, out-of-order 2735
    first divergence at ours[0] / cpp[0]
  >>     0 G1 F7150.945                    |     0 ; LINE_WIDTH: 0.399991
         1 ; LINE_WIDTH: 0.400001          |     1 G1 F7151.157
         2 M204 S5000                      |     2 M204 S5000
```

**The same three lines, with the first two swapped.** A census over the whole file:

| | `; LINE_WIDTH:` total | followed by a bare `G1 F` | preceded by one |
|---|---|---|---|
| C++ | 215,199 | **203,408** | **0** |
| ours | 154,063 | 3 | **148,548** |

**148,548 inverted pairs.** C++ never once emits the feedrate before the width tag; we almost always
do. Both lines match by content, so `line_parity.py` scores them as two hits — this is precisely the
class R651 proved it cannot see, and it is the single largest ordering defect in the file. Note this
is a different question from the one R558/R567/R570/R571 closed: those were about *how many*
`; LINE_WIDTH:` lines we emit, not their position relative to the feedrate.

**Root cause located.** C++'s Width tag (GCode.cpp:6607) is emitted inside `_extrude` *before*
`set_speed` (:6663). Ours comes out of `extrude_path_with_arc_fitting` at `exporter.rs:1376`, but a
**collection-level pre-set feedrate at `exporter.rs:1017` runs first** — and C++ has no
collection-level F at all. Our own comment at `exporter.rs:997-1003` already says so ("native never
emits a collection-level feature-speed F before a perimeter loop") and guards it with
`skip_pre_speed`, but that guard requires `config.enable_overhang_speed` **and** a `Loop` entity
**and** a perimeter role. Majora's config sets `enable_overhang_speed` to `1`, yet the dump shows
the pre-set F still being emitted on Outer wall — **so either the guard is not resolving true or the
F comes from a second site. Establish which before changing anything (R649).**

### The second cause, unsized

Layer 142's Outer wall shows the same four points in a different order:

```
ours: 67.945,211.801 → 68.176,211.37 → 68.499,211.829 → 68.221,211.868
cpp:  68.5,211.828   → 68.157,211.875 → 67.946,211.802 → 68.158,211.408
```

Different loop start point **and** traversal direction — the seam. That is a geometry-ordering
difference, not an emission-order one, and it will not yield to the same fix.

Layer 163 also shows C++ emitting a per-segment speed ramp we do not
(`G1 F3047.933` / `G1 X… E.00113` / `G1 F2766.257` / …) — a content difference, separate again.


### Inner wall cross-check

Same shape at index 0, plus a third item:

```
ours: G1 F7150.945 / ; LINE_WIDTH: 0.400001 / G1 X67.774 Y122.848 …
cpp:  ; LINE_WIDTH: 0.399991 / ; LAYER_HEIGHT: 0.3 / G1 F7151.157 / G2 X47.83 Y208.203 …
```

The inversion reproduces exactly. C++ also emits `; LAYER_HEIGHT:` immediately after the width tag
where we do not (we emit it elsewhere, `exporter.rs:1444`). And the geometry after the header is
unrelated — ours starts at (67.774, 122.848), C++ at (47.83, 208.203) — so on Inner wall the
**island/region visit order** differs too, a larger structural divergence than the Outer wall seam.
Three distinct causes now, only one of them sized.

### R654

**Fix the inversion.** First establish which site emits the offending `G1 F` — instrument it, do not
infer from the guard's source. Then make the width tag precede it, gated, and A/B on **both** rates.

**Predict a large in-order gain (order 100k lines) and ZERO content change**, since no line is
created or destroyed — the exact inverse of the R648-R650 rounds and the same signature as R651,
which is now the precedent for reading such a result correctly. Fallback: if in-order moves by less
than 20k, the inversion is not what breaks the alignment and the seam difference dominates — say so,
and take the seam (loop start point and direction) as the target instead.

## R654 — the inversion is fixed exactly, and it made the order metric WORSE by 26,309

**Prediction: half right and half wrong, in the more interesting direction.** Content moved **zero**,
as predicted. In-order moved **−26,309** (468,570 → 442,261) — the opposite of the ~100k gain
predicted. Shipped **OPT-IN** (`probe_enabled("LINEWIDTH_BEFORE_SPEED")`, default OFF); all three
baseline hashes unchanged.

### The producer was not where R653 said

R653 named the collection-level pre-set at `extrude_collection`. **The A/B refuted it in one run:
gate on and gate off produced the identical hash `3d741dde`** — `skip_pre_speed` is already true
there, so that site never fires on these paths. Inference from the guard's source would have shipped
a no-op; the A/B is why it didn't.

The real producers are **five** `set_speed` calls made immediately before `extrude_path`:
`exporter.rs:544, 554, 596, 608` (all four in `extrude_loop`) and `:1860` (`extrude_entity`'s Path
arm). C++ does both inside *one* function, tag first — Width tag at `GCode.cpp:6607`, `set_speed` at
`:6663`. Routing all five through a pending slot flushed straight after the tag reproduces C++'s
order exactly:

| | `; LINE_WIDTH:` | followed by bare `G1 F` | preceded by one |
|---|---|---|---|
| ours, gate off | 154,063 | 3 | **148,548** |
| **ours, gate on** | 154,063 | **148,551** | **0** |
| C++ | 215,199 | 203,408 | **0** |

The adjacency is now C++'s exactly, and the tag total is untouched — no line created or destroyed,
which the unchanged content rate confirms independently.

### Why it still made things worse

**We emit 154,063 width tags against C++'s 215,199 — a 61,136 deficit.** Our tags therefore cannot
anchor one-to-one against C++'s. While the `G1 F` lines floated free of the tags they aligned with
C++'s `G1 F` lines on their own; binding each one to a tag that is *often missing relative to C++*
destroys that alignment at every absent tag. The local order is right and the global sequence is
worse.

That is a real result, not a metric artefact: R652 validated `seq_parity.py` against a change of
independently known size, and here it is reporting a cost that the content metric cannot see in
either direction. **A locally C++-faithful change can be globally worse when a related count is
still wrong** — new rule, and the reason this ships OFF rather than ON.

### R655 — close the tag COUNT first

The 61,136 `; LINE_WIDTH:` deficit is now the blocking item: until our tags correspond to C++'s
one-for-one, no amount of correcting their *position* can help, and this round's code is waiting
behind it with a one-line flip.

Note this re-opens a question R558/R567/R570/R571 closed — legitimately, with new evidence. Those
rounds settled the *entity-level* emitter; the live emitter is now the per-path one under
`LINEWIDTH_PERPATH` (`exporter.rs:1372-1380`), whose own parked comment says the gate "ADDS unmatched
lines (83187 → 87805)" because our f64 widths drift from C++'s f32 chain in the 6th significant
digit. **That comment predates the order metric and was written against the content rate alone.**

**Predict the deficit is dominated by width-register misses — our `width_tag_changed` register
suppressing tags C++ emits — rather than by paths we never visit**, since the path counts elsewhere
match closely. Census `width_tag_changed` calls versus emissions, both sides, before changing
anything. Fallback: if the register fires about as often as C++'s, the deficit is in the *paths*,
not the tags — count paths per feature on both sides and follow that instead.

## R655 — the tag deficit is neither the register nor the paths: it is width VARIETY

**Prediction WRONG, and the pre-registered fallback's first clause WRONG too. No engine change; all
three hashes unchanged.** The census was the round, and it rules out both named causes.

### The registers are equivalent

Ours (`writer.rs:894`) casts f64→f32 and compares for inequality; C++ (`GCode.cpp:6605`) stores a
`float` and compares for inequality. Same granularity, no epsilon on either side. **The register is
not suppressing anything.**

### The paths are nearly all there

Counting maximal runs of consecutive extrusion moves per feature:

| feature | C++ runs | our runs | Δ | C++ tags | our tags | Δ |
|---|---|---|---|---|---|---|
| Outer wall | 168,373 | 149,555 | **−11.2%** | 62,582 | 19,722 | **−68.5%** |
| Inner wall | 59,351 | 46,848 | −21.1% | 40,567 | 24,309 | −40.1% |
| Internal solid infill | 42,688 | 34,140 | −20.0% | 33,011 | 24,571 | −25.6% |
| Floating vertical shell | 74,392 | 79,694 | +7.1% | 70,424 | 76,126 | **+8.1%** |

Outer wall paths are 11% short while its tags are 69% short. **The deficit is not the paths.**

### It is how much the width VARIES

| feature | | tags | distinct widths | tags per distinct |
|---|---|---|---|---|
| Outer wall | C++ | 62,582 | **21,181** | 2.95 |
| | ours | 19,722 | **11,845** | 1.67 |
| Inner wall | C++ | 40,567 | 17,731 | 2.29 |
| | ours | 24,309 | 12,498 | 1.95 |
| Floating vertical shell | C++ | 70,424 | 50,631 | 1.39 |
| | ours | 76,126 | 57,007 | 1.34 |

Two things at once on the walls: we produce **about half as many distinct widths** (11,845 vs
21,181 on the outer wall) **and** our widths alternate back and forth far less (2.95 → 1.67 tags per
distinct value). Meanwhile Floating vertical shell has *more* variety than C++'s. **The width
variation is not missing so much as mis-distributed across features** — and that is upstream of the
emitter entirely: it is the Arachne variable-width beading, the R592 chain.

A sanity check that confirms the reading: Sparse infill has **exactly one** distinct width on both
sides (0.45), so every tag there marks an *entry* into the feature — C++ enters 2,940 times, we
enter 4,649. That is feature interleaving, not width at all.

Per the pre-registered fallback, quantifying before attempting: closing the tag count is worth **at
most +61,136 content lines** (the tags are comments C++ also emits). Its worth on the **order**
metric cannot be quantified until the tags exist — which is exactly why it blocks R654's parked fix
rather than being schedulable against it.

### One exact thing this census did find

`GCode.cpp:6591` computes `last_was_wipe_tower = (m_last_processor_extrusion_role == erWipeTower)`
and both the Width tag (`:6605`) and the Height tag (`:6619`) are emitted when it is true
**regardless of value** — the comment says why: "PrusaMultiMaterial::Writer may generate
Height_Tag lines without updating m_last_height". **We have no such force-emit and no
`last_processor_extrusion_role` register at all.** Worth roughly one Width and one Height tag per
tool change (~2,723 each) — small, exact, and cheap.

### R656

**Port `last_was_wipe_tower`.** Add a `last_processor_role` register to the writer, set it where the
`; FEATURE:` role tag is emitted, and force the Width and Height tags on the first path after a
wipe-tower block. Gate it; A/B on **both** rates.

**Predict a small positive on content (~+2,700 Width and ~+2,700 Height, C++ has both) and a small
positive or flat on order**, since these tags are re-anchoring points immediately after a block
boundary that both engines agree on. **Fallback: if order goes negative like R654, the same
anchoring problem applies and it parks alongside `LINEWIDTH_BEFORE_SPEED` — in which case stop
adding tags entirely until the Arachne width chain is closed, and say so.**

## R656 — `last_was_wipe_tower` ported: the Height class all but closes, and order falls again

**Prediction half right. Content **+1,204** (711,537 → 712,741) as predicted in sign, though not in
size or composition. Order **−2,117** (468,570 → 466,453) — predicted "small positive or flat".
Shipped OPT-IN per the pre-registered fallback; all three baseline hashes unchanged.**

### Reachability checked first

`GCode.cpp:4718-4720` — `process_layer` sets `m_last_processor_extrusion_role = erWipeTower` once
per (layer, extruder) that has a wipe tower. `:6591` reads it once per `_extrude`, and **both** the
Width tag (`:6605`) and the Height tag (`:6619`) use that same value before `:6600` overwrites the
register with the path's own role. So it is a one-shot consumed by the first path after the tower —
which is how it is modelled here: `set_force_analyzer_tags()` after the tower block,
`take_force_analyzer_tags()` once at the top of the path, both guards reading the taken value.

### What it did

| | LINE_WIDTH | LAYER_HEIGHT | content | in order |
|---|---|---|---|---|
| off (`3d741dde`) | 154,063 | 4,720 | 711,537 | 468,570 |
| **on** (`de3b7876`) | 154,088 | **8,058** | **712,741** | **466,453** |
| C++ | 215,199 | **8,297** | | |

**The Height class was 43% short and is now 3% short** — 4,720 → 8,058 against C++'s 8,297. That is
the round's real finding and it is unambiguous.

The Width half is the surprise: **+25 lines, not the ~2,700 predicted.** The force-emit almost never
fires there because the width had already changed — the register was going to emit anyway. The
prediction treated the two tags as symmetric; they are not, and R655's census already implied it
(our widths vary less, so a forced re-emit lands on a value that differs anyway only rarely).

Of the 3,363 lines added, **1,204 match by content** (36%) — C++ has many of them, in those places.

### Parked, per the pre-registration

Order fell 2,117. The fallback written before the round said: if order goes negative like R654, park
it, stop adding tags, and say so. **Honouring that.** This is the second time a demonstrably
C++-faithful tag addition has cost order while `; LINE_WIDTH:` is 61,136 short (R654: −26,309).

The trade being deferred is explicit: **+1,204 content against −2,117 order.** It is not obviously
the wrong call to ship it — but the rule was fixed in advance precisely so that "it should help"
reasoning, which is what failed in R654, does not get a second vote. Two flips now wait on the same
blocker: `WIPE_TOWER_FORCE_TAGS` and `LINEWIDTH_BEFORE_SPEED`.

### R657 — the Arachne width-variety gap, or nothing else

Everything queued is behind it. R655 sized it: outer wall 11,845 distinct widths against C++'s
21,181, alternating 1.67 tags per distinct value against 2.95. Worth ≤ +61,136 content directly,
plus it unblocks two finished changes worth a measured +1,204 content and an unknown (currently
−28,426) order.

**Start by finding where the variety is lost, not by porting anything**: dump the width sequence of
one outer-wall loop from both engines at the same layer and compare the beading directly. **Predict
the loss is in the beading strategy's quantisation — C++ keeping bead widths our chain rounds
together** — since our count is close to half, which smells like pairs collapsing. Fallback: if the
width sequences have the same *shape* but ours is shifted or scaled, it is the flow/width
computation downstream of beading, not the beading itself — say which and follow it.

## R657 — the width gap is not quantisation: it is 223 layers where our wall stops varying at all

**Prediction REFUTED. No engine change; all three hashes unchanged.** The census was the round and it
kills the quantisation hypothesis outright.

### The constants are identical

`discretization_step_size` is `scaled(0.8)` on both sides (`WallToolPaths.cpp:457`,
`wall_tool_paths.rs:935`), as are `transition_filter_dist` (`scaled(100)`) and
`allowed_filter_deviation`. Nothing is being rounded together by a different step size.

### The values are not a coarsened version of C++'s — they are a different solution, per layer

Dumping the outer-wall width sequence at three layers:

| layer | C++ tags / distinct | ours tags / distinct | our distinct with a C++ match ≤1e-3 |
|---|---|---|---|
| 150 | 130 / 109 | **18 / 11** | **27%** |
| 300 | 282 / 170 | 132 / 99 | 82% |
| 450 | 158 / 100 | 123 / 95 | 64% |

At layer 300 the two engines largely agree on the values (82% of ours have a C++ counterpart within
1e-3) and we simply emit fewer. At layer 150 we emit **11 distinct widths against C++'s 109** and
only 27% even correspond. **The failure is per-layer and near-binary, not a uniform coarsening** —
which is what a quantisation difference would have produced.

### Sized across the model

Over the 509 layers where C++'s outer wall has ≥20 distinct widths:

- **223 layers (44%) are collapsed** — our distinct count is under 35% of C++'s.
- On those layers our **path count is comparable**: median 317 extrusion runs against C++'s 359. We
  print the wall; it just barely changes width.
- The extremes are the early layers. Layer 2: C++ 1,117 distinct widths, **ours 0** — not one
  `; LINE_WIDTH:` in the entire outer wall — across 413 runs. Layer 3: C++ 421, ours 0.

### A third of it is re-attribution, not loss

Summing distinct widths across **all** wall features (Outer wall, Inner wall, Overhang wall,
Floating vertical shell) on those same collapsed layers:

| | C++ median | ours median | ratio |
|---|---|---|---|
| Outer wall only | — | — | **< 0.35** |
| all wall features | 230 | 144 | **0.67** |

So a substantial part of the "missing" outer-wall variety exists in our output but is **labelled as
a different wall feature** — consistent with R655's finding that Floating vertical shell has *more*
variety than C++'s (+8.1%). The remaining third is a genuine deficit.

### R658

Two separable targets, and they need separating before either is touched:

1. **Wall-feature attribution.** Our variable-width wall paths are landing under the wrong
   `; FEATURE:` label. This is measurable directly: for a collapsed layer, dump our feature sequence
   against C++'s over the same Z and see which label our variable-width paths carry.
2. **The residual variety** (33% even after summing all wall features), catastrophic on layers 1-3.

**Predict attribution dominates on mid-model layers and genuine loss dominates on layers 1-12**,
since the early layers are near-zero on *both* measures while layer 300 is 82% value-agreement.
Fallback: if the feature sequences match and only the widths differ, attribution is not the issue
and the whole gap is beading — say so and take `BeadingStrategy` directly. **Do the layer-1-vs-300
comparison first; they may have different causes and treating them as one is what would waste the
round.**

## R658 — not attribution: 40 layers where our outer wall is literally one width

**Prediction REFUTED, and it forces a correction to R657's own reading. No engine change; all three
hashes unchanged.**

### The feature composition matches — nothing is mis-labelled

Dumping the `; FEATURE:` block sequence and per-feature counts at three layers:

| layer | | blocks | Outer wall tags/distinct/runs | Inner wall | Floating vert. shell |
|---|---|---|---|---|---|
| 150 (collapsed) | C++ | 84 | 130 / 109 / **515** | 39 / 16 / 58 | 33 / 23 / 43 |
| | ours | 88 | **18 / 11 / 557** | 30 / 13 / 52 | 30 / 27 / 38 |
| 300 (healthy) | C++ | 119 | 282 / 170 / 557 | 133 / 78 / 170 | 194 / 173 / 208 |
| | ours | 124 | 132 / 99 / 440 | 96 / 59 / 133 | 140 / 127 / 159 |
| 2 (extreme) | C++ | 64 | 1537 / 1117 / 1820 | 495 / 412 / 567 | — |
| | ours | 53 | **0 / 0 / 413** | 20 / 11 / 156 | — |

At layer 150 we print **more** outer-wall runs than C++ (557 vs 515) with **11 distinct widths
against 109**, while every other feature is comparable. At layer 2 we emit **zero** `; LINE_WIDTH:`
across 413 outer-wall runs. The block sequences and feature sets agree throughout. **Attribution is
not the mechanism.**

### Correction to R657

R657 reported that summing distinct widths over all wall features lifted us from <35% to 0.67 of
C++, and read that as "roughly a third of the missing variety is mis-labelled". **That reading was
wrong.** The per-layer dumps show the other wall features were never collapsed in the first place —
summing them simply diluted the outer wall's collapse with healthy features. It is the same error
as R651's: treating an aggregate as evidence of a mechanism without checking the parts.

### The real structure

Over the 502 layers where C++'s outer wall has ≥20 distinct widths and we emit ≥50 runs:

| class | layers | C++ distinct (median) | our distinct | C++ runs | our runs |
|---|---|---|---|---|---|
| **FLAT** — our distinct ≤2 | **40** | 52 | **1** | 326 | **327** |
| PARTIAL — <35% but >2 | 183 | 79 | 12 | 363 | 316 |
| OK — ≥35% | 279 | 64 | 39 | 252 | 186 |

**The FLAT class is the clean signal: 40 layers where we lay down the same number of outer-wall
paths as C++ (327 vs 326) at exactly one width, against C++'s 52.** Not fewer paths, not different
labels, not a coarser sampling — no variation at all. They are scattered through the model
(2, 3, 4, 5, 33, 66, 69, 70, 75-87, 93-96, 99, 104, 105, 110, 111, 113, 117, 136 …), so this is not
a first-layer special case either.

### R659

**Take the 40 FLAT layers.** They are the sharpest entry point into the Arachne chain that has
appeared: binary outcome, matched path count, and a specific layer list to instrument. Everything
else about the width gap — the 183 PARTIAL layers, the two parked changes — is downstream of
understanding why a wall comes out uniform.

**Predict our `WallToolPaths` returns a single-bead (uniform) solution on those layers rather than a
variable-width one** — i.e. the beading is running but resolving to one bead, not that Arachne is
bypassed. Instrument the Rust side directly: count distinct bead widths coming out of
`WallToolPaths::generate` per layer and correlate with the FLAT list, before reading any more C++.
Fallback: if the widths coming out of `WallToolPaths` *are* varied and the flattening happens later,
the target is between beading and gcode — `thick_polyline_to_multi_path` / `extrude_path`'s
`path.width` — and the census should move downstream one stage at a time until the variety
disappears.

## R659 — the beading DOES produce the variety; it is lost downstream

**Prediction REFUTED. The pre-registered fallback's first clause is the answer. No engine behaviour
changed — both probes are `probe_enabled` (default OFF) — and all three hashes are unchanged.**

### Two probes, one answer

`WTPWIDTH` bins every `WallToolPaths::generate()` call by the number of distinct junction widths in
its result. Over 25,000 calls on Majora:

| distinct widths | 0 | **1** | 2 | 3-4 | 5-8 | 9-16 | 17-64 | 65+ |
|---|---|---|---|---|---|---|---|---|
| calls | 128 | **11,520** | 3,372 | 3,066 | 3,092 | 2,181 | 1,542 | 99 |

46% of calls return a single width — which *looks* like the predicted single-bead story, but a
single bead is the correct answer for a thin region, so the bin alone proves nothing. Note also that
call order is not layer order (the slice is parallel), so the drift across running totals cannot be
read as a per-layer trend.

`WTPLAYER` settles it by keying on `layer_id`, which is in scope at the `WallToolPaths` call site.
Distinct widths **produced by the beading** on the FLAT layers R658 isolated, against what reached
the gcode:

| layer | 2 | 3 | 4 | 5 | 33 | 66 | 69 | 70 | 75 | 76 | 77 | 80 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| **beading produced** | 1 | 1 | **10** | **36** | **33** | **24** | **19** | **23** | **14** | **5** | 1 | **4** |
| reached gcode (outer wall) | 0 | 0 | **0** | ~2 | ≤2 | ≤2 | ≤2 | ≤2 | ≤2 | ≤2 | ≤2 | ≤2 |

Controls: layer 150 beading 12 / gcode 11; layer 300 beading 70 / gcode 99; layer 450 beading 135 /
gcode 95.

**On 10 of the 12 sampled FLAT layers the beading produced 4-36 distinct widths and the gcode shows
at most two.** Only layers 2, 3 and 77 genuinely returned a single-width solution. The variety is
computed and then discarded.

Caveat stated plainly: the probe counts distinct widths across the whole layer's beading (all
regions) while the gcode figure is outer-wall only, so the magnitudes are not directly comparable —
but "beading produced 36, outer wall shows ≤2" is not a magnitude artefact.

### Where that puts the target

Per the pre-registered fallback: the flattening is **between beading and gcode**. The candidates in
order of the pipeline are `thick_polyline_to_multi_path` (which converts a `VariableWidthLines`
junction chain into `ExtrusionPath`s, each with a single `width`) and whatever sets `path.width`
before `extrude_path` reads it at `exporter.rs:1417`.

### R660

**Census the same layers one stage further down.** Instrument `thick_polyline_to_multi_path`'s
output the way `WTPLAYER` instruments the beading's: distinct `ExtrusionPath::width` per layer,
same FLAT list, same controls. That single comparison localises the loss to one side of that
function.

**Predict the loss is inside `thick_polyline_to_multi_path` — the junction widths are averaged or
taken from one endpoint when a chain becomes a path**, since that is the only place a per-junction
width has to collapse to a per-path scalar. Fallback: if its output is still varied, the loss is
later still — carry the census into `extrude_path` and read `path.width` at the emitter, which is
one more hop and ends the search either way. Note R569/R582 examined `thick_polyline_to_multi_path`
before and cleared it; that was for path SPLITTING, not width collapse, so this is a new question
about the same function, not a re-test.

## R660 — the width collapse is not downstream: our inset-0 beading is already flat

R660: the collapse is NOT downstream -- our inset-0 beading is already flat

Prediction REFUTED. `thick_polyline_to_multi_path` preserves or AMPLIFIES width variety; the outer
wall arrives at it already near-constant. No engine change (both new probes are `probe_enabled`,
default OFF); all three hashes unchanged (benchy 248ff22a, cube 14566293, majora 3d741dde).

THE FUNCTION IS INNOCENT. TPMPPROBE, extended to census its OUTPUT, over 215,000 ExternalPerimeter
calls on Majora: widthpts=1,530,846  in_distinct=224,888  out_distinct=226,238  out_paths=227,641
and flat_calls=209,807 -- 97.6% of calls receive a ThickPolyline whose widths are all equal. It
emits slightly MORE distinct widths than it receives (the tolerance split interpolates), and only
2,299 calls collapse a varied input to a single output. Nothing is averaged away here.

Per-call flatness alone would not settle it -- distinct lines can carry distinct widths and still
give a varied layer -- so AWIDTH censuses the same quantity per `layer_id`, outer wall only, at the
one site where both the ExtrusionLine and the layer index are in scope (perimeter_generator.rs:3351):
distinct junction `w` going IN against distinct `ExtrusionPath::width` coming OUT.

  layer            2    3    4    5   33   66   69   70   75   76   77   80 | 150  300  450
  junctions IN     1    1    4    9    9    5    2    4    2    4    1    4 |   5   34  118
  paths OUT        1    1    3   21    9    4    1    3    1    1    1    1 |   5   87   76
  paths emitted  557  555  593  693  514  384  313  400  514  748  813  550 |1065  561  208

OUT >= IN on the layers that have anything to work with (5: 9 -> 21; 300: 34 -> 87). The stage adds
variety. What enters it is the problem: 1-9 distinct widths for an entire layer's outer wall.

THIS QUALIFIES R659. R659's "beading produced 4-36" was an ALL-INSET count -- WTPLAYER walks every
`total_perimeters` line. Restricted to inset 0 the same beading yields 1-9. The variety R659 saw is
real but it lives in the INNER walls; the outer bead was flat all along. The stage-by-stage rule
still paid: it took one probe to find that, where four rounds of hypotheses would not have.

WHY THAT IS NOT YET A BUG -- AND WHAT MAKES IT ONE. Both engines run the identical strategy chain
(Distributed -> Redistribute -> Widening -> [OuterWallInset] -> Limited; ours matches
BeadingStrategyFactory.cpp:35-56 line for line, OuterWallContour `#if 0` on both sides), and
`RedistributeBeadingStrategy::compute` (:151-155, faithful to RedistributeBeadingStrategy.cpp:82-89)
sets the outer bead to `min(thickness/2, optimal_width_outer)` -- CONSTANT for every wall at least
two outer-widths thick. A flat outer wall is what that rule prescribes. So C++'s 1,117 distinct
outer-wall widths on layer 2, where ours is literally one across 557 paths, cannot come from the
redistribute step: it must come from regions thin enough to fall under `2 * optimal_width_outer`, or
from bead counts we are not reaching. The divergence is in what `thickness`/`bead_count` the
skeletal trapezoidation hands the strategy, not in the strategy.

R661: census `bead_count` and `thickness` at the strategy boundary. For the FLAT layers, bin the
(thickness, bead_count) pairs `RedistributeBeadingStrategy::compute` is called with and count how
many land in the variable branch (`thickness < 2 * optimal_width_outer`, where the outer width is
`thickness/2` and therefore varies). Predict we take the constant branch far more often than C++,
because our upstream `thickness` is quantised or our bead_count is clamped. FALLBACK: if the branch
mix is comparable, the loss is in `SkeletalTrapezoidation::generateJunctions` interpolating between
identical endpoint beadings -- census the beading pairs per edge instead, and say so.

## R661 — the strategy is not the ceiling: the variable branch is unreachable

R661: the strategy is not the ceiling -- the variable branch is unreachable and 28,984 widths exist

Prediction REFUTED, and refuted structurally rather than numerically: the branch whose mix I set out
to compare cannot be taken. The pre-registered fallback fires, and it lands on the R585-R587
propagation chain, re-measured here for the first time since R591. No engine change (BEADPROBE is
`probe_enabled`, default OFF); all three hashes unchanged (benchy 248ff22a, cube 14566293, majora
3d741dde); suites unchanged.

THE BRANCH MIX IS NOT A THING. Over 640,000 `RedistributeBeadingStrategy::compute` calls on Majora
(optimal_width_outer = 35,562 = 0.356 mm):

    bead_count > 2, thickness/2 >= optimal_width_outer  (CONSTANT)   515,789   80.6%
    bead_count > 2, thickness/2 <  optimal_width_outer  (VARIABLE)         0    0.0%
    bead_count <= 2                                     (thickness/bc) 124,211  19.4%

The variable branch fires ZERO times, and that is arithmetic, not luck:
`RedistributeBeadingStrategy::getOptimalBeadCount` only returns more than 2 when
`thickness > 2 * optimal_width_outer`, so by the time `compute` sees `bead_count > 2` the `min` has
already resolved to `optimal_width_outer`. There is no branch mix to compare against C++, on this
model or any other. The prediction was not merely wrong, it was unaskable.

AND THE STRATEGY IS NOT THE CEILING. The same run: 28,984 distinct `actual_outer_thickness` values
over 294,523 distinct thicknesses, and a pre-existing probe that shares the `BEADPROBE` env name
(skeletal_trapezoidation.rs, R584-era) independently reports 28,147 distinct `bead_widths[0]`
spanning 0.190-0.762 mm. Against C++'s 21,181 distinct outer-wall widths in the whole Majora gcode.
The beading strategy manufactures MORE width variety than C++'s output contains -- all of it from
the 19.4% thin branch. Nothing is quantised or clamped here. R661's stated cause is dead.

THE FALLBACK, AND WHAT IT COSTS TO SKIP THE ARCHIVE. The fallback named
`SkeletalTrapezoidation::generateJunctions` interpolating between identical endpoint beadings --
which is exactly where R585, R586 and R587 already were, and R590 fixed one root of it
(`collapse_small_edges` snap distance 400x too large). Those probes are still in the tree. Re-run
now, against the C++ figures those rounds recorded (C++ is unchanged, so its numbers still stand):

  quantity                                    R585-587    NOW      C++      C/R now
  BEADPAIR P(adjacent beadings differ in w0)   0.0243*   0.0309   ~0.0450    ~1.46
  PROPCLASS interp share                       0.0212    0.0234    0.0511     2.18
  no-op interpolations                          63.3%     57.4%     49.1%      --
  `from` already had a beading (ratio+interp)   6.33%     6.91%    14.25%     2.06
  UPPROBE SEEDED                                2.88%     2.96%     4.98%     1.68
  DNPROBE normal                               39.90%    37.92%    43.33%     1.14
  (* R585's deepest checkpoint was 2.5M edges, this one 3.0M; R591 showed this statistic drifts
   with prefix depth, so read the BEADPAIR row as direction, not as a point estimate.)

R590's fix moved P(differ) and the no-op rate; it did NOT move the two terms that matter. We still
interpolate 2.18x less often than C++, and `propagateBeadingsDownward` still finds `from` already
holding a beading only half as often. A copy is bit-identical to its source and cannot create a
width difference between neighbours, so a 2x copy surplus is a 2x variety deficit, directly.

R662: the upward seed rate. `propagate_beadings_upward` skips on `to->bead_count >= 0` 69.97% of
iterations against C++'s 64.71%, and the entire shortfall lands in SEEDED (2.96% vs 4.98%). R588
already showed the per-NODE `bead_count >= 0` share is at parity (16.76% vs 16.37%), so this is not
a property of the nodes -- it is a property of which nodes the upward walk VISITS, or of the ORDER
it visits them in (the pass seeds progressively, so an edge arriving after its target was already
counted is skipped). Census `upward_quad_mids` itself: size per `generate()` call, and the
distribution of `to->bead_count >= 0` over its members versus over the whole graph. Predict the
population is composed differently -- our list over-represents nodes that already have a bead_count.
FALLBACK: if the list's composition matches the graph's, the difference is ORDERING, and the test
becomes whether our traversal order differs from C++'s `upward_quad_mids` sort
(SkeletalTrapezoidation.cpp) -- say so, and the target becomes the sort comparator.

Two things checked and cleared in passing, so R662 does not re-open them: the
`upward_quad_mids` construction (`prev && next && isUpward()`,
SkeletalTrapezoidation.cpp:1480-1486 vs skeletal_trapezoidation.rs:2622-2627) is identical, and the
sort comparator's tie-break subtracts the segment norm on BOTH sides -- C++ line 1496 ends
`... - (a->to->p - a->from->p).cast<int64_t>().norm()`, which our `- a_seg` at :2652 mirrors. The
list's membership and its order are therefore not obviously divergent by inspection; R662 has to
measure.

## R662 — the seed deficit factors 1.17x × 1.44x, and the bigger factor is the other guard

R662: the seed deficit factors 1.17x x 1.44x, and the bigger factor is the OTHER guard

Prediction CONFIRMED in kind and REFUTED in magnitude: the list is composed differently, but the
composition gap is 1.05x against a 1.68x seed deficit, so it cannot be the cause. Conditioning the
four recorded C++ rates splits the deficit cleanly and moves the target one guard down. No engine
change (`UQM` is `probe_enabled`, default OFF); all three hashes unchanged (benchy 248ff22a, cube
14566293, majora 3d741dde); suites unchanged.

FIRST, GUARD 1 IS STATIC -- THE ORDERING FALLBACK DOES NOT APPLY TO IT. `propagateBeadingsUpward`
calls `setBeading`, never `setBeadCount`, so `to->data.bead_count` cannot change during the pass.
Verified on both sides: C++ `SkeletalTrapezoidationJoint::setBeading` (Joint.hpp:45-48) assigns the
weak_ptr only, and ours (skeletal_trapezoidation_joint.rs:108) is the same assignment. The census
confirms it empirically -- reading the list once, after the sort, BEFORE the pass runs, gives 0.6992
against `UPPROBE`'s mid-walk 0.6983. Same number. Guard 1 is pure list composition.

SECOND, THE COMPOSITION IS ENRICHED -- BUT SO IS C++'S, ALMOST EQUALLY.

  list size / generate() call                             42.03
  distinct `to` nodes / call                              37.00   (1.136 edges per target)
  members whose `to` has bead_count >= 0                  0.6992
  DISTINCT targets with bead_count >= 0                   0.6714   (so not a multiplicity artefact)
  whole-graph nodes with bead_count >= 0                  0.1684

The list's targets are enriched 4.152x over the graph. C++'s recorded pair (0.6471 skip against
0.16367 of nodes, R587/R588) gives 3.954x. **Ratio 1.05.** Our list IS composed differently, exactly
as predicted, and the difference is nowhere near large enough to produce a 1.68x seed deficit. The
prediction is right about the mechanism and wrong about its size -- which is the same failure mode as
R657's "quantisation": a real effect, mistaken for the dominant one.

THIRD, THE DECOMPOSITION, WHICH IS THE ACTUAL RESULT. All four UPPROBE categories were recorded for
C++ at R587, so the rates can be conditioned rather than compared flat. Guard 1 fires first, so
guards 2 and 3 should be read as shares of ITS SURVIVORS:

                                        Rust      C++     C/R
  survive guard 1 (bead_count < 0)     0.3017   0.3529   1.170
  ... of those, skip on !from.hasBeading()  90.2%   85.8%     --
  ... of those, SEEDED                  9.8%    14.1%   1.44
  SEEDED overall                       0.0296   0.0499   1.68     (0.3017 x 0.098 = 0.0296 checks)

**The 1.68x is 1.17x x 1.44x, and the larger factor is guard 2, not guard 1.** Among edges that
reach it, our `from` node lacks a beading 90.2% of the time against C++'s 85.8%. Composition
contributes the smaller share.

AND GUARD 2 IS THE ONE THAT IS ORDER-SENSITIVE. `setBeading` during the pass makes later members'
`from` nodes eligible, so unlike guard 1 this guard compounds: every seed we miss removes a source
for a later edge. That is a positive feedback on the deficit and it is where R662's pre-registered
ordering fallback actually belongs -- one guard down from where I aimed it.

R663: guard 2. Two separable questions, and they need separating before either is touched.
  1. STATIC: at list-build time, before the pass, what share of members' `from` nodes already have a
     beading? Compare against the 9.8% pre-pass share the same census can measure for `to`. If the
     pre-pass `from` share is already ~85-90% empty on both engines, the initial store at
     SkeletalTrapezoidation.cpp:1518-1547 is the target, not the walk.
  2. DYNAMIC: how many of C++'s seeds come from a `from` that an EARLIER member of the same pass
     seeded? Count seeds whose source beading was created by this pass versus by the initial store.
Predict the dynamic term dominates on C++ and is near zero for us -- a chain that never starts. FALLBACK:
if our pre-pass `from`-has-beading share is itself lower, the deficit is inherited from the initial
store loop and the target becomes `node.data.bead_count <= 0` at cpp:1520, NOT the propagation.
Note CENSUS says nodes with a beading (0.1682) and nodes with bead_count >= 0 (0.1684) are the same
population for us, and R588 recorded the same identity for C++ (0.16325 / 0.16367) -- so any
`from`-side deficit is about WHICH nodes are in the list, not about the store dropping any.

## R663 — the chain does start: a 1.97% base amplified 5.07×

R663: the chain does start -- it starts from a 1.97% base and amplifies 5.07x

Prediction REFUTED on its central clause. I predicted the dynamic term dominates on C++ and is near
zero for us -- "a chain that never starts". It starts, and it carries 80% of our seeds. No engine
change (`UQM`/`UQMSEED` are `probe_enabled`, default OFF); all three hashes unchanged (benchy
248ff22a, cube 14566293, majora 3d741dde); suites unchanged.

THE DYNAMIC TERM NEEDED NO NEW STATE. C++ already marks every beading the upward pass creates --
`upper_beading.is_upward_propagated_only = true` (SkeletalTrapezoidation.cpp:1630) -- and the
initial store leaves it false (BeadingPropagation's constructor, Joint.hpp:24-29). Reading that flag
on the SOURCE at each seed says directly whether the pass is feeding itself. Our port already
carried the field, faithfully set at the same line.

BOTH TERMS, MEASURED (Majora, ~31,800 seeds total; UQMSEED's last print is at 30,000):

  pre-pass `from.hasBeading()` among guard-1 survivors    0.0197   (300,770 survivors)
  seeds whose SOURCE was created by this same pass        0.8028
  conditional seed rate (SEEDED / guard-1 survivors)      0.0984   (0.0294 / 0.2989)

Those three are not independent, and the identity closes: a static base amplified by the chain gives
0.0197 / (1 - 0.8028) = 0.0999 against a measured 0.0984 -- 1.5% apart. So the pass is well described
by two numbers, a STATIC BASE of 1.97% and an AMPLIFICATION of 1/(1-0.8028) = 5.07x.

WHAT THAT DOES TO R662'S 1.44x. C++'s conditional seed rate is 0.141 (R587). Under the same model
that is its own base times its own amplification, and I have neither. The gap can be closed by a
small divergence in EITHER parameter:

    if C++'s chained share equals ours (0.8028)  ->  its static base is 0.0278   (1.41x ours)
    if C++'s static base equals ours (0.0197)    ->  its chained share is 0.860  (ours 0.803)

**That is the useful result: we are not looking for a 2x defect here.** A 1.41x static base, or five
percentage points more chaining, each fully accounts for the 1.44x on its own. Any hypothesis that
would move either parameter by much more than that is the wrong size (R662's rule, applied to itself).

THE FALLBACK CANNOT BE EVALUATED, AND I AM NOT GOING TO PRETEND IT CAN. It was worded "if OUR
pre-pass `from`-has-beading share is itself lower" -- lower than C++'s, which was never measured.
1.97% is not low or high against anything. Splitting the two parameters requires the same flag read
on the C++ side, which is the one measurement this round did not make.

R664: instrument C++. Two counters in `propagateBeadingsUpward`, both trivial and both already
supported by existing C++ state:
  1. Before the loop, walk `upward_quad_mids` once and count members with `to->data.bead_count < 0`
     whose `from->data.hasBeading()` -- C++'s static base, directly comparable to our 0.0197.
  2. At the seed, count `lower_beading.is_upward_propagated_only` -- C++'s chained share, directly
     comparable to our 0.8028.
The submodule is git-managed: revert from inside it afterwards, verify BOTH status checks, rebuild.
PREDICT the static base carries most of the 1.44x (C++ near 0.027, chained share within a point or
two of ours), because the amplification is a property of the traversal order and R661 verified the
sort comparator matches line for line while the base depends on which nodes the initial store
reached. FALLBACK: if the bases match and the chained shares diverge, the sort's ties are resolving
differently despite identical source -- and the target becomes `std::sort` versus `sort_by` on equal
keys, which is a REAL divergence class (C++'s introsort is unstable, Rust's `sort_by` is stable).

## R664 — both parameters measured: 1.20× base × 1.18× amplification

R664: both parameters measured -- the 1.44x is 1.20x base x 1.18x amplification, split evenly

Prediction HALF RIGHT. C++'s static base IS higher and lands close to the predicted value, but it
does NOT carry most of the gap: the two factors are 1.198x and 1.179x, near enough a 50/50 split.
The pre-registered fallback did NOT fire -- it required the bases to MATCH, and they do not. No Rust
change; the C++ instrumentation is env-gated (`CPPUP`) and has been REVERTED -- both status checks
(the submodule's own and the parent's) are empty and the engine is rebuilt from pristine source.

BOTH PARAMETERS, MEASURED ON THE SAME MODEL, SAME POPULATION (guard-1 survivors):

                                          Rust      C++     C/R
  static base (pre-pass from.hasBeading)  0.0197   0.0236   1.198
  chained share (source made this pass)   0.8028   0.8327     --
  amplification 1/(1-chained)             5.07x    5.98x    1.179
  conditional seed rate = base x amp      0.0999   0.1411   1.412
  ... measured directly                   0.0984   0.141    1.434

THE MODEL IS NOW VALIDATED ON BOTH ENGINES, AND ON C++ IT IS EXACT. R663 built
`conditional_rate = static_base / (1 - chained_share)` from Rust numbers and it closed to 1.5%.
Applying it to C++'s two freshly-measured parameters predicts 0.1411 against the 0.141 R587 recorded
-- a different run, different counters, three-decimal agreement. That is an independent check of the
model, not a restatement of it.

WHAT IT MEANS FOR THE SEARCH. R663 framed this as an either/or: a 1.41x base OR five more points of
chaining. It is BOTH, and each is small. Two separate ~1.19x effects have to be found, and neither
is the kind of defect that shows up as a wrong constant. Concretely:
  - the BASE is positional. Our graph has MORE nodes carrying a beading than C++'s (CENSUS 0.1684 /
    0.1693 against R588's C++ 0.16367, so 1.03x in OUR favour), yet FEWER of them sit at the `from`
    end of a guard-1-survivor edge (0.83x). More beadings, worse placed. That tension is the lead.
  - the AMPLIFICATION is traversal order, and it is the half the R663 fallback was aimed at. The
    fallback's precondition failed, but its target survives for this half specifically: `std::sort`
    is an unstable introsort and Rust's `sort_by` is stable, so equal keys in `upward_quad_mids`
    come out in a different order, and the pass seeds progressively. 5.98x versus 5.07x is exactly
    the size of effect a tie-order difference would produce.

ONE OBSERVATION HELD LOOSELY. C++ reached 95,000 seeds on this model where our whole run makes about
31,800 -- 3.0x, against a 1.68x rate difference. That would imply ~1.8x more upward iterations, far
more than the 1.085x graph-density gap R591 left. But the two counters print on different triggers
and I did not match call counts, so this is an observation to test, not a result (R584's rule).

R665: take the BASE half, because it is measurable without touching C++ again. The question is
positional, and the census belongs on our side: for guard-1-survivor edges, what distinguishes a
`from` node that has a beading from one that does not -- `distance_to_boundary`, `bead_count`,
degree, is-it-a-transition-node? Compare that distribution against the whole-graph node
distribution. Predict our beadings sit deeper (higher `distance_to_boundary`) than C++'s, i.e. the
initial store fires on the right COUNT of nodes but the wrong ONES, because `bead_count` is set by
`generateTransitioningRibs` and R590 showed our transition machinery was mis-tuned once already.
FALLBACK: if the `from`-with-beading nodes look distributionally identical to the graph, the base
gap is not positional either and the remaining candidate is the initial store's own guard
(`node.data.bead_count <= 0`, cpp:1520) admitting a different SET at equal count -- instrument that
guard's population directly and say so.

## R665 — the base gap is 93% depth mix, in one bucket; and a scale correction to R660

R665: the base gap is 93% depth MIX, and a scale check killed a dramatic false finding

Prediction HALF RIGHT: our survivor population IS shifted deeper, but the mechanism I named -- the
initial store firing on the wrong nodes, a RATE effect -- is not it. The rate contributes NEGATIVE
33%; the depth MIX contributes 93%, and it is concentrated in a single bucket. The fallback did not
fire (the distributions are not identical). No Rust behaviour change (`UQM`/`UQMDEPTH` are
`probe_enabled`, default OFF); the C++ instrumentation is reverted, both status checks empty, engine
rebuilt pristine; all three hashes unchanged (benchy 248ff22a, cube 14566293, majora 3d741dde);
suites unchanged.

FIRST, A CORRECTION THAT MATTERS BEYOND THIS ROUND. BambuStudio's `SCALING_FACTOR` is **0.00001**
(libslic3r.h:58) -- 1e5 units per mm, IDENTICAL to our crate. R660 recorded the opposite ("ours 1e5,
C++ 1e6, consistent within each system") and that premise has been carried in the eliminated list
ever since. R660's conclusion survives -- identical scales are trivially consistent -- but its stated
reason was wrong, and anything else built on "C++ is 1e6" should be re-checked.

I found this because the first version of this round's C++ probe used a 200,000 divisor for its
0.2 mm buckets. It produced a spectacular result: C++'s survivor nodes all within 0.6 mm of the
boundary against ours spread past 1.8 mm, distributions barely overlapping. That was entirely my own
10x mis-scaling. The empirical check settles it: raw max `distance_to_boundary` is 972,244 on our
side and 955,118 on C++'s -- 9.72 mm vs 9.55 mm, 1.8% apart. Same units, same model.

SECOND, THE REAL DECOMPOSITION. Same 0.2 mm buckets on both engines, over guard-1 survivors, on
`from.distance_to_boundary`:

  bucket   mm        mix_R   mix_C   C/R    rate_R  rate_C  C/R     contrib_R  contrib_C
    0    0.0-0.2    0.0648  0.1449  2.24    0.0544  0.0709  1.30     0.00353    0.01027
    1    0.2-0.4    0.1071  0.1161  1.08    0.0538  0.0549  1.02     0.00576    0.00637
    2    0.4-0.6    0.1231  0.1124  0.91    0.0420  0.0364  0.87     0.00517    0.00409
    3    0.6-0.8    0.1080  0.0936  0.87    0.0338  0.0216  0.64     0.00365    0.00202
    4    0.8-1.0    0.0892  0.0781  0.88    0.0110  0.0064  0.58     0.00098    0.00050
   5-8   1.0-1.8    0.2497  0.2238  0.90     ~0      ~0      --      0.00033    0.00020
    9    >=1.8      0.2580  0.2310  0.90    0.0001  0.0001  1.00     0.00003    0.00002

Recomposing gives 0.01945 (measured 0.0192) and 0.02349 (measured 0.0235), so the table is the base.

  total base gap                                    0.00403
  bucket 0 alone                                    0.00675   (167% of the gap)
  C++'s MIX with OUR rates                          0.02320   -> mix explains  93%
  OUR mix with C++'s RATES                          0.01811   -> rate explains -33%

**The entire base gap is one bucket.** Below 0.2 mm from the boundary C++ has 2.24x our share of
survivor-edge `from` nodes, and that single bucket supplies 44% of C++'s whole base against our 18%.
Everywhere deeper our rates are equal or BETTER (buckets 2-4 run 0.87x/0.64x/0.58x in our favour),
which is why the rate term comes out negative. In absolute terms, normalising by calls (ours 12.5
survivors/call, C++ 16.8), C++ has 3.0x our count of sub-0.2 mm survivor edges and only 1.20x our
count of deep ones.

WHAT THAT MEANS. This is not the initial store choosing different nodes -- at matched depth we store
beadings at least as often as C++ does. It is that C++'s `upward_quad_mids` contains three times as
many edges whose `from` node hugs the boundary. R588 found our whole graph 25% sparser and R590 cut
that to 1.085x; this says the residual sparsity is not uniform -- it is concentrated within one
bead-width of the boundary, exactly where `isUpward()` edges are shortest and where R590's
`collapse_small_edges` snap operates.

R666: absolute near-boundary edge counts, not shares. Count, per `generate()` call and in the WHOLE
graph rather than in the list: nodes with `distance_to_boundary < 0.2 mm`, edges with both endpoints
under it, and how many of those pass `prev && next && isUpward()`. Predict the deficit is already in
the GRAPH at that depth rather than in the `isUpward` filter, since R661 verified the filter is
identical line for line. FALLBACK: if the graph has them and the filter drops them, `isUpward()` is
the target -- it compares `distance_to_boundary` of the two endpoints, and at sub-0.2 mm depths that
comparison is between values a few hundred scaled units apart, where an off-by-one or a `>=`/`>`
difference decides the outcome; diff the two implementations directly and say so.

## R666 — the filter is at parity; the near-boundary graph is 1.62x thinner

R666: prediction CONFIRMED -- the filter is at parity, the graph is 1.62x thinner at the boundary

First confirmed prediction in nine rounds. `isUpward()` is innocent: it passes 10.4% of near-boundary
edge pairs on C++ and 10.2% on ours, a 1.022x difference. The deficit is already in the GRAPH. The
fallback (a `>=`/`>` divergence in `isUpward`) does not fire. No Rust behaviour change (`UQM` is
`probe_enabled`, default OFF); the C++ instrumentation is reverted with both status checks empty and
the engine rebuilt pristine; all three hashes unchanged (benchy 248ff22a, cube 14566293, majora
3d741dde); suites unchanged.

ABSOLUTE COUNTS, PER `generate()` CALL, SAME 20000-UNIT (0.2 mm) THRESHOLD ON BOTH ENGINES:

  per generate() call                   Rust       C++     C/R
  all nodes                          163.950   194.110   1.184
  nodes < 0.2 mm                     124.116   148.224   1.194
  edges with BOTH ends < 0.2 mm       25.547    41.469   1.623
  ... passing prev && next && isUpward 2.601     4.314   1.659
  FILTER pass rate                    0.1018    0.1040   1.022

THE SHAPE OF IT IS THE RESULT. Nodes are uniformly about 1.19x sparser -- shallow ones (1.194)
exactly like the graph as a whole (1.184). But shallow-to-shallow EDGES are 1.62x sparser, and the
filter passes them at the same rate, so the 1.659x deficit in near-boundary upward edges is
inherited whole from the edge count. Edges per shallow node: 0.2058 ours against 0.2798 C++.
**The local connectivity is 1.36x thinner than our node count alone would predict.** For a planar
skeletal graph edges scale with nodes, so a 1.19x node deficit should give a 1.19x edge deficit; we
lose an extra 36% of the near-boundary adjacency on top.

That is a different defect from the one R588/R590 chased. R588 measured node and edge counts
falling together at 1.25x, and R590's `collapse_small_edges` fix moved both. This says what remains
is NOT a uniform thinning: our nodes are where C++'s are, and the edges between the close-together
ones are missing.

ONE NUMBER TO TREAT WITH SUSPICION. Overall graph density here reads C/R 1.184, where R591 recorded
1.085 after the R590 fix. Different measurement points and different aggregation, so I am not
claiming a regression -- but the two do not agree and one of them is wrong about something. Worth a
direct check before either is used as a baseline again (R584's rule, and R665's).

R667: the near-boundary adjacency. The question is now narrow and structural -- which edges exist
between nodes under 0.2 mm from the boundary. Two candidates, and they are separable:
  1. `collapse_small_edges` still removing more than C++ does. R590 fixed the snap DISTANCE (400x
     too large) but the fix was scored on line parity, not on edge counts; count collapses per call
     on both engines, and the shallow-shallow subset specifically.
  2. The Voronoi-to-half-edge conversion, which R589 already identified as where the density is
     created rather than in the Voronoi diagram itself. Count edges entering and leaving that
     conversion, shallow subset.
Predict (1) -- `collapse_small_edges` operates on exactly the short edges that a sub-0.2 mm
adjacency is made of, and its gate `ARACHNE_COLLAPSE_SNAP_5` is still shipped ON with a value that
was tuned against a different metric. FALLBACK: if collapse counts match, the edges never existed,
and the target is the conversion in (2) -- instrument its input and output edge counts directly and
say so. A/B `ARACHNE_COLLAPSE_SNAP_5=0` against the shallow edge count either way; it is one env var
and it separates the two candidates in a single run (R654: A/B, do not infer).

## R667 — collapse removes the same fraction; same count, different set

R667: collapse removes the same FRACTION -- the edges never existed, and the A/B ran backwards

Prediction REFUTED twice over, and the pre-registered fallback fires. `collapse_small_edges` is not
over-removing: it takes 21.94% of edges on our side against C++'s 22.24%, a 1.014x difference. And
the A/B moved the wrong way -- disabling R590's fix makes the deficit WORSE, not better. No Rust
behaviour change (`COLLAPSEPROBE` is `probe_enabled`, default OFF); the C++ instrumentation is
reverted with both status checks empty and the engine rebuilt pristine; all three hashes unchanged
(benchy 248ff22a, cube 14566293, majora 3d741dde); suites unchanged.

THE A/B, WHICH I EXPECTED TO INCRIMINATE THE GATE AND WHICH EXONERATES IT:

  ARACHNE_COLLAPSE_SNAP_5   nodes<0.2mm   edges(both<0.2mm)   all nodes/call   majora hash
  ON   (shipped, R590 fix)      122.648            24.534           162.11      3d741dde
  OFF  (pre-R590 snap)          106.904            16.563           143.38      d92c0205
  C++                           148.224            41.469           194.11         --

Turning the fix off collapses harder (the old snap distance was 400x too large) and leaves 16.6
near-boundary edges against 24.5 -- further from C++'s 41.5, not closer. R590's value is already
pulling in the right direction; the residual is not a snap-distance problem. Worth recording because
the obvious next move -- "tune the snap smaller still" -- is now measured as the wrong direction.

THE COUNTERS, SAME QUANTITY ON BOTH ENGINES:

  collapse_small_edges              Rust       C++     C/R
  edges BEFORE collapse / call    414.07    496.56    1.199
  edges removed / call             90.85    110.44    1.216
  removal FRACTION                0.2194    0.2224    1.014
  nodes removed / call             45.42     55.22    1.216

**The graph is already 1.199x sparser BEFORE this function runs**, and the function then removes the
same proportion on both sides. The fallback's wording was "if collapse counts match, the edges never
existed" -- they match, and they didn't.

BUT THE ROUND ENDS ON A TENSION, AND IT IS THE USEFUL PART. Implied edges after collapse are 323.22
against 386.12, C/R 1.195 -- uniform, matching the node deficit. Yet R666 measured the surviving
shallow-to-shallow edges at 1.623x while nodes under 0.2 mm are 1.194x. So collapse removes the same
COUNT and the same FRACTION, and the global result stays uniform, but our near-boundary adjacency
comes out 1.36x thinner than the node count predicts. **Same number of edges removed, different SET.**

That reframes the search away from "how many" and onto "which". One concrete mechanism is already
visible in the port: C++'s `collapseSmallEdges` erases from `edges` DURING the iteration
(SkeletalTrapezoidationGraph.cpp:196-208, `safelyRemoveEdge` advancing the loop iterator), so later
iterations see the already-collapsed graph and cascading collapses are suppressed. Ours collects
pointers into a `HashSet` and rebuilds the list at the end
(skeletal_trapezoidation_graph.rs:650-671), so every decision is made against the ORIGINAL graph. In
a mutate-while-iterating algorithm those are not the same function even when they remove equal
counts -- which is exactly the signature measured here.

R668: the SET, not the count. Instrument WHICH edges each engine removes, keyed by a stable
geometric identity (endpoint coordinates, which are comparable across engines) rather than by
pointer: per call, the removed set's `min(from.dtb, to.dtb)` distribution in the same 0.2 mm buckets
R665/R666 used. Predict our removals are skewed toward the shallow buckets -- deferred removal cannot
see that an earlier collapse already merged a node pair, so it removes both members of a chain where
C++ removes one. FALLBACK: if the removed-set depth distributions match, the divergence is not in
what collapse removes but in what it MERGES INTO -- C++ reassigns endpoints as it goes and ours
reassigns against stale state; census surviving edges whose endpoints moved, and say so. Either way,
A/B a cascading (in-loop) variant against the deferred one behind a new gate before touching the
shipped path (R654; and R656's rule about honouring the fallback).

## R668 — collapse is innocent; the near-boundary edges are missing at construction

R668: collapse is fully innocent -- the near-boundary edges are missing at construction

Prediction REFUTED, and the fallback's premise turned out not to apply either: the answer is one
step further back than either branch anticipated. The shallow-to-shallow deficit is ALREADY 1.647x
before `collapse_small_edges` runs, and the function leaves it essentially unchanged. No Rust
behaviour change (`COLLAPSEPROBE` is `probe_enabled`, default OFF); the C++ instrumentation is
reverted with both status checks empty and the engine rebuilt pristine; all three hashes unchanged
(benchy 248ff22a, cube 14566293, majora 3d741dde); suites unchanged.

BEFORE AND AFTER, SAME 20000-UNIT (0.2 mm) THRESHOLD ON BOTH ENGINES:

  per generate() call                    Rust       C++     C/R
  ALL edges before collapse            412.980   496.560   1.202
  shallow-shallow BEFORE collapse       32.722    53.901   1.647
  shallow-shallow removed                7.151    12.654   1.770
  shallow-shallow removal FRACTION       0.2185    0.2348   1.075
  shallow-shallow SURVIVING             25.571    41.247   1.613

R667 left two hypotheses: the deficit is already 1.62x before collapse (collapse innocent, the edges
never existed), or it is ~1.2x before and 1.62x after (collapse removes the wrong set). **It is the
first, and not marginally: 1.647x before, 1.613x after.** Collapse does not create the deficit, and
it does not widen it -- if anything it narrows it slightly, since we remove a SMALLER fraction of
shallow-shallow edges than C++ does (21.85% vs 23.48%). My prediction was that our removals would
skew shallow; they skew the other way.

THE NON-UNIFORMITY IS PRESENT AT CONSTRUCTION. At the same moment -- graph built, collapse not yet
run -- the global edge deficit is 1.202x while the shallow-shallow subset is 1.647x. Expressed as a
share of all edges, shallow-shallow pairs are 7.92% of our graph against 10.85% of C++'s, a ratio of
1.370. That is the same 1.36x connectivity figure R666 measured after the whole pipeline, which
means it was never introduced downstream at all: **the skeletal graph is born with 1.37x too little
near-boundary adjacency, and every stage after that faithfully preserves the shortfall.**

Three stages are now cleared in sequence, each by measurement rather than inspection: the `isUpward`
filter (R666, pass rate 1.022x), collapse's REMOVAL COUNT (R667, fraction 1.014x), and collapse's
REMOVED SET (this round, 1.075x on the shallow subset and in the wrong direction to help). The
deferred-vs-in-loop removal difference I flagged at R667 is real as a code divergence but it is not
the cause of this deficit -- it operates on a graph that is already short.

R669: the Voronoi-to-half-edge conversion. R589 already identified that step as where the density is
created rather than in the Voronoi diagram itself, and it is now the only stage left between the
input polygon and the first measurement. Count, on both engines: Voronoi cells and edges IN, and
graph nodes and edges OUT, with the shallow subset separated. Predict the deficit appears in the
conversion rather than in the Voronoi diagram, since the diagram is computed by the same boost
Voronoi construction on both sides while the conversion is hand-ported -- specifically that we
discard or merge more near-boundary cell edges while transcribing. FALLBACK: if the conversion's
in/out ratio matches, the deficit is in the INPUT to the Voronoi -- the discretized boundary segment
count -- and the target becomes how each engine samples the polygon before building the diagram;
count segments in and say so. That fallback is worth taking seriously: a coarser boundary
discretization would produce exactly this signature, a graph that is globally similar but missing
the fine near-boundary structure.

## R669 — the conversion is at parity; the Voronoi input is 1.17x short

R669: the conversion is at parity -- we feed the Voronoi 1.17x fewer boundary segments

Prediction REFUTED and the pre-registered fallback fires, exactly as worded. The Voronoi-to-half-edge
conversion transcribes at the same rate on both engines (1.006x); the deficit is in the INPUT. No
Rust behaviour change (`GBUILD`/`CONV` are `probe_enabled`, default OFF, and pre-existing from R589);
the C++ instrumentation is reverted with both status checks empty and the engine rebuilt pristine;
all three hashes unchanged (benchy 248ff22a, cube 14566293, majora 3d741dde); suites unchanged.

THE WHOLE BUILD, PER `constructFromPolygons` CALL:

  per constructFromPolygons call        Rust       C++     C/R
  Voronoi INPUT segments              29.420    34.403   1.169
  Voronoi OUT vd_verts                94.431   113.355   1.200
  Voronoi OUT vd_edges               304.541   362.320   1.190
  graph edges (post-conversion)      414.763   496.378   1.197
  graph nodes (post-conversion)      197.018   249.159   1.265

  CONVERSION graph_edges per Voronoi edge   rust 1.3619   cpp 1.3700   C/R 1.006
  DIAGRAM    vd_edges per input segment     rust 10.3515  cpp 10.5316  C/R 1.017

**The deficit is present at the input and passes through everything unchanged.** 1.169x segments in,
1.190x Voronoi edges, 1.197x graph edges; the diagram contributes 1.7% and the conversion 0.6%. Six
stages have now been cleared by measurement, and the seventh -- the conversion, which R589 named as
the dominant term -- is at parity too.

R589 IS SUPERSEDED, TWICE OVER. It recorded the conversion at C/R 1.1655 (rust 0.9333 vs cpp 1.0878
graph-edges-per-Voronoi-edge) and made it the primary target. Both engines now read ~1.36-1.37. R590's
collapse fix moved our figure from 0.9333 to 1.3619 measured pre-collapse, so the conversion gap
R589 found has already been closed and the target it handed forward no longer exists. R589 also
recorded the input segments at 29.440 vs 31.422 (1.067x) and dismissed the residual as "small
against 1.256x"; it now reads 29.420 vs 34.403 (1.169x). Ours is unchanged to four digits; C++'s
moved. One of the two measurements is wrong and I cannot tell which from here, so the 1.169x should
be re-confirmed before anything is built on it -- but it is the current, directly-measured value and
it is the only term left.

WHAT STILL IS NOT EXPLAINED, STATED PLAINLY. The global deficit is 1.197x and the input accounts for
it. The shallow-shallow deficit is 1.647x (R668), which is 1.376x BEYOND the global figure. A 1.169x
coarser boundary sampling does not arithmetically produce a 1.376x near-boundary connectivity loss on
its own. Near-boundary skeleton edges arise between ADJACENT boundary segments, so their count
plausibly scales superlinearly with segment density -- but that is a hypothesis, not a measurement,
and it is the one thing R670 must not assume.

R670: why do we feed 1.169x fewer segments? The input is one segment per polygon point
(`SkeletalTrapezoidation.cpp:419-422`, mirrored at skeletal_trapezoidation.rs:659-665), so the
question is the POLYGON, not the loop: our `polys` arrive with fewer points. Count points per
polygon on both engines at the `constructFromPolygons` boundary, and walk back one stage to whoever
produces them. Predict the loss is in a simplify/douglas-peucker step applied to the slice outline
before Arachne sees it, since that is the only thing in the path that removes points and it is
tuned by a tolerance constant. FALLBACK: if the point counts match at the producer and diverge only
at the consumer, the divergence is in how the polygon is passed (closed vs open, first point
repeated), which changes the count by exactly one per contour -- check the per-contour delta, and if
it is ~1.0 say so, because 29.42 vs 34.40 over ~1.007 polygons per call is a delta of 5, not 1.

PROCESS FAILURE WORTH RECORDING. My first C++ patch replaced a two-statement block and the
replacement text omitted `separatePointyQuadEndNodes();` -- I deleted a live call from the engine.
The build succeeded and the slice died silently inside wall generation with no diagnostic. The check
that would have caught it immediately is asserting the replaced call still appears exactly once in
the patched function, which I now do. A string-replace patch must verify what it PRESERVED, not
only what it added.

## R670 — the outline arrives 1.07x coarse; the prep chain adds 1.08x

R670: the deficit splits evenly -- the outline ARRIVES 1.07x coarse and the prep chain adds 1.08x

Prediction HALF RIGHT. `simplify` is a real contributor but it is not the cause: the outline already
arrives 6.9% short of C++'s before Arachne touches it, and the whole prep chain adds only about as
much again. The fallback did not fire (the divergence is not a per-contour off-by-one). No Rust
behaviour change (`POLYPROBE` is `probe_enabled`, default OFF); the C++ instrumentation is reverted
with both status checks empty and the engine rebuilt pristine; all three hashes unchanged (benchy
248ff22a, cube 14566293, majora 3d741dde); suites unchanged.

POINTS PER CONTOUR THROUGH THE WHOLE `prepared_outline` CHAIN (WallToolPaths.cpp:455-486, mirrored
at wall_tool_paths.rs:954-1021), both engines, same stage labels:

  stage                          Rust    C++     C/R
  0 input outline               45.693  48.829  1.069
  1 after triple offset         46.446  50.970  1.097
  2 after simplify              28.360  32.030  1.129
  5 after removeColinearEdges   28.174  31.806  1.129
  9 final prepared_outline      28.278  32.572  1.152

  input contour deficit         1.069x
  prep chain adds               1.078x   (1.152 / 1.069)

**Points per CONTOUR is the right unit here and it is call-count independent**, which matters
because the two engines do not call `generate()` the same number of times -- R560/R562 recorded C++
calling it about twice per surface, one speculatively -- so anything per-call mixes populations. On
that unit the final 1.152x lines up with the 1.169x segments-per-`constructFromPolygons` R669
measured, as it must, since a segment is a point.

WHERE IT COMES FROM, IN TWO ROUGHLY EQUAL PARTS. The outline handed to `WallToolPaths` is ALREADY
1.069x coarser than C++'s -- that is upstream of everything Arachne does and upstream of every stage
this round instrumented. The chain then widens it to 1.152x, and `simplify` is where most of that
happens: it keeps 61.06% of points on our side against C++'s 62.84%, a 1.029x difference applied to
an already-short outline. `fixSelfIntersections`, `removeDegenerateVerts` and `removeColinearEdges`
move the ratio by nothing at all (1.129 through all three on both engines).

So the prediction named a real effect and got its size wrong again -- the same failure mode as R662
and R665. Simplify contributes; it is not the origin.

AND IT STILL DOES NOT REACH THE SHALLOW RESIDUAL. R668's near-boundary connectivity deficit is
1.376x BEYOND the global figure. A 1.152x coarser contour does not produce that arithmetically, and
nothing measured this round changes that. It remains the open term.

R671: go upstream of `WallToolPaths`. The outline arrives 1.069x coarse, so the question is who
produces it -- the slice contour, and whatever simplification the slicing stage applies before
regions are built. Predict a resolution/tolerance constant in the SLICING path (not Arachne's), since
the input deficit is uniform (1.069x at stage 0, before any Arachne stage touches it) and a
resolution constant is the only thing that produces a uniform per-contour point loss. FALLBACK: if
the slice contours match at the producer, the loss is in what `WallToolPaths` is HANDED -- an
`offset`/`union` applied between the slice and the wall generator, and the target becomes that
call's parameters. Instrument point-per-contour at the slice output and at every hop to
`WallToolPaths::generate`, on both engines; the chain is short and one run covers it.

PROCESS NOTE. The C++ patch failed to build the first time because the helper landed inside
`WallToolPaths::generate()` rather than at file scope -- a static function definition nested in a
function body. The R669 preservation check passed (nothing was deleted) and still missed it, so that
check needs a companion: after inserting a file-scope helper, assert the text immediately preceding
it is not inside a function. I checked it by printing the preceding non-blank line this time.

## R671 — the slicer is faithful; the Arachne path's DP tolerance is unscaled

R671: the slice resolution is faithful -- but the ARACHNE path passes an UNSCALED DP tolerance

Prediction REFUTED at the constant it named, and the round found a different, concrete defect
instead: the same units bug R122 fixed for the classic perimeter path is still live in the Arachne
path. No engine change this round (reading and tracing only); all three hashes unchanged (benchy
248ff22a, cube 14566293, majora 3d741dde); suites unchanged; the submodule is clean.

THE PREDICTED TARGET IS CLEARED. I predicted a resolution/tolerance constant in the SLICING path.
`PrintObjectSlice.cpp:144` sets `params_base.resolution = print_config.resolution <= 0.001 ? 0.0f :
0.0025` -- a hardcoded 0.0025 mm, deliberately not the config value -- and our
`print_object.rs:452-453` mirrors it exactly, ternary and constant. The slicer is faithful. So is
`slice_closing_radius`. Whatever coarsens our outline, it is not the mesh slicer's DP tolerance.

THE PRODUCER IS ONE LINE, AND IT IS NOT IN THE SLICER. `PerimeterGenerator.cpp:1511`:

    ExPolygons last = offset_ex(surface.expolygon.simplify_p(surface_simplify_resolution), ...);

That `simplify_p` is what hands `WallToolPaths` its outline, so R670's "0 input outline" is this
call's output. `surface_simplify_resolution` (cpp:1500, and the identical cpp:914 for the classic
path) is `(enable_arc_fitting && fuzzy_skin == None) ? 0.2 * m_scaled_resolution :
m_scaled_resolution`, where **`m_scaled_resolution = scaled<double>(print_config.resolution)`** --
for resolution 0.0125 mm that is 1250 scaled units, so the tolerance is 250.

THE DEFECT. Our config field holds the UNSCALED millimetre value (`layer.rs:579
surface_simplify_resolution: print_config.resolution`), and the two paths treat it differently:

  classic  perimeter_generator.rs:525-531   0.2 * (surface_simplify_resolution / 0.00001)   -> 250   correct
  arachne  perimeter_generator.rs:2962-2968 0.2 *  surface_simplify_resolution              -> 0.0025 UNSCALED

and :2991 / :2997 pass that straight into `surface.simplify_p(...)`. R122 found exactly this bug in
the classic path, documented it at :517-524, and gated the fix behind `F1_UNION`. **The Arachne path
was never corrected.** It is the path Majora's walls go through.

I AM NOT CLAIMING THIS EXPLAINS R670's DEFICIT, AND THE DIRECTION IS WHY. A tolerance 1e5x too small
means LESS simplification, which would leave us with MORE points -- and R670 measured FEWER (45.693
vs 48.829). So on the naive reading it pushes the wrong way. But R122's own note says the default
geo DP "rounds near-collinear projections to 0, removing points the tiny tolerance never would",
so with a near-zero tolerance the point removal is decided by rounding rather than by the tolerance,
and its direction is not predictable from the source. That is precisely the situation R654's rule
covers: do not infer, A/B it.

R672: A/B the Arachne tolerance. Scale it the way the classic path does, behind a new gate
(`ARACHNE_SIMPLIFY_SCALED`), and measure points-per-contour at "0 input outline" plus BOTH parity
rates with the gate on and off. Predict the fix RAISES our point count and therefore does NOT close
R670's gap -- the honest expectation from the direction argument above -- in which case the round's
value is a correctness fix plus a refuted mechanism, and the 1.069x needs a different cause.
FALLBACK: if the point count DROPS toward C++'s 48.829, the rounding effect dominates the tolerance,
the direction argument is wrong, and this is the origin -- say so and re-run the whole chain
(R669's segments, R668's shallow-shallow) to see how far up it propagates. Either way this is a
units bug in a live path and worth fixing on its own terms; score it on both parity rates before
shipping it (R654 and R656 both went negative on a locally-faithful change).

## R672 — RETRACTED BY R673 (the collapsed arm crashed; the A/B hashed a stale file). Original text follows.

R672: the "fix" is parity-inert AND collapses the outline -- `simplify_p` takes millimetres

Prediction REFUTED, the fallback did not fire either, and the A/B produced a third outcome neither
branch anticipated: scaling the tolerance destroys the outline upstream and changes nothing
downstream. That combination is the finding. The gate ships DEFAULT OFF (`probe_enabled`, not
`faithful_gate`); all three hashes unchanged (benchy 248ff22a, cube 14566293, majora 3d741dde);
suites unchanged.

THE A/B, BOTH ARMS, MAJORA + BENCHY + CUBE:

  arm   majora     benchy     cube       maj content   benchy content
  ON    3d741dde   248ff22a   14566293   28.32%        75.07%
  OFF   3d741dde   248ff22a   14566293   28.32%        75.07%

Byte-identical on all three fixtures and identical on both parity rates. But the same run's
POLYPROBE, at the outline `WallToolPaths` receives:

  arm   "0 input outline"            polys      points
  OFF   (shipped)                   26,782   1,223,769
  ON    (scaled tolerance)               0           0

**The scaled value collapses every contour to nothing, and the gcode does not change by one byte.**

FIRST CONCLUSION: R671's DEFECT CLAIM IS WRONG, AND I AM WITHDRAWING IT. A 250-unit tolerance
annihilating the contours is the signature of `simplify_p` taking an UNSCALED MILLIMETRE tolerance --
250 means 250 mm. The classic path at :525-531 divides by 0.00001 because it feeds a DIFFERENT
function (`simplify_p_dp_rings_faithful`, the R122 faithful-DP path), not `simplify_p`. So the two
sites were never the same call and the "same bug fixed in one path, live in the other" reading was
mistaken. The pre-existing unscaled value is correct for the function it is passed to. R671's
prediction was already refuted at its constant; its replacement claim is now refuted too.

SECOND CONCLUSION, AND IT IS THE ONE THAT MATTERS: an outline of ZERO polygons produced
byte-identical gcode. Whatever `WallToolPaths::generate()` is handed at this call site, its walls do
not reach the output. R671 identified `PerimeterGenerator.cpp:1511` as the producer of the Arachne
wall outline by reading C++; that mapping does not hold for our engine at the site I patched. Either
the function I edited is not the live Arachne wall path, or its result is discarded downstream.

I am not going to guess which. Both are checkable in one run and the check is R595's, which this
round's own instructions demanded and which I did not do first: VERIFY REACHABILITY BEFORE
ATTRIBUTING. The A/B was the reachability test in disguise and it came back negative.

R673: find the live producer. Put a counter at every `WallToolPaths::new` / `generate()` call site in
`perimeter_generator.rs` keyed by site, and a second counter on the extrusion entities each one's
result contributes to `entities` — then correlate with the ~15,500 outer-wall `ExtrusionLine`s
AWIDTH counts. Predict the live path is `generate_arachne_one`'s sibling rather than the function I
patched, since our port has a documented history of parallel live/dead twins (the `fill/` module has
two, and `crate::fill::` resolves to the live one while `fill/fill.rs` is dead and often the more
faithful). FALLBACK: if the site I patched IS the only one and IS reached, then its output is
discarded later, and the target becomes whatever consumes `entities` -- census the entity count in
and out of that consumer, and say so.
**DO NOT carry R670's "the outline arrives 1.069x coarse" forward as attributed to this site until
R673 says which site the walls actually come from.** The 1.069x measurement stands; its location
does not.

## R673 — R672 retracted: the collapsed arm panicked and the A/B hashed a stale file

R673: R672 IS RETRACTED -- the collapsed arm CRASHED, and the A/B hashed a stale file

Prediction REFUTED, fallback REFUTED, and the round's real result is a correction to the one before
it. R672 concluded that collapsing the outline left the gcode byte-identical and therefore that the
patched site's walls never reach the output. **That is wrong.** The collapsed arm did not produce
gcode at all -- it panicked -- and my A/B script hashed the file left over from the previous run. No
Rust behaviour change this round; all three hashes unchanged (benchy 248ff22a, cube 14566293, majora
3d741dde); suites unchanged.

THE MEASUREMENT THAT CAUGHT IT. Re-running the same gate with `AWIDTH` on, checking the exit code
this time:

  arm         exit   AWIDTH blocks   outer-wall ExtrusionLines
  BASE         0        62           n=15,500  sum_in=9,057  sum_out=15,601
  COLLAPSED  101         0           none at all

`exit=101` is a panic, at `crates/libslic3r-rs/src/gcode/tool_order_utils.rs:2133` -- `groups[0].insert(...)`
on an empty `groups`, reached because an object with no walls produces no tool assignment. The same
panic line appears in R672's own ON-arm log, which I did not read because the script reported a hash
and I took the hash at face value.

WHAT THIS RESTORES AND WHAT IT KILLS.
  - **R672's "the patched site's walls do NOT reach the gcode" is RETRACTED.** They do. Emptying the
    outline destroys the walls so thoroughly that the slicer cannot finish. The site is live and
    load-bearing, exactly as R670/R671 assumed.
  - **R672's OTHER conclusion stands on its own evidence**: a 250-unit tolerance annihilates every
    contour, so `simplify_p` takes an UNSCALED millimetre tolerance and the shipped unscaled value is
    correct for that callee. R671's units-bug claim remains withdrawn.
  - **R673's own prediction is refuted too**: there is no live/dead twin here. `generate_arachne` is
    the only function containing `WallToolPaths::new` (two sites, :3131 and :3144, both inside it),
    the chain `simplified` -> `last` -> `last_p` -> `WallToolPaths::new` is unbroken (:3009 -> :3017
    -> :3048 -> :3132), and it is reached. The fallback ("its output is discarded later") is refuted
    by the same panic.

THE PROCESS DEFECT, WHICH IS THE THING WORTH KEEPING. My A/B script ran the slice, ignored the exit
code, and copied `tests/.tmp/nu3mf/majorasmask.gcode` unconditionally. A failed slice leaves the
previous run's output in place, so the copy succeeded and produced a plausible, *identical* hash --
the most convincing possible wrong answer. Two rounds of reasoning were built on it within one round
of it being produced. `$D/ab_template.sh` now `rm -f`s the target first, checks `rc` and file
existence, prints `SLICE FAILED` with the panic line, and refuses to hash a missing file.

R674: the chain is restored to where R670/R671 left it, minus their attribution error. The outline
handed to `WallToolPaths` is 1.069x coarse (R670) and it IS produced at `generate_arachne`'s
`simplify_p` (:3009/:3026) -- the site is live, and the tolerance it passes is CORRECT for its
callee. So the coarseness is not a tolerance bug: it is the INPUT to that `simplify_p`, i.e.
`surface.expolygon` as the perimeter generator receives it. Census points-per-contour on
`surface.expolygon` at `generate_arachne`'s entry versus C++'s at `PerimeterGenerator.cpp:1511`, one
hop further back than R670 measured. Predict the 1.069x is already present there, since every stage
inside `WallToolPaths` is now accounted for. FALLBACK: if `surface.expolygon` matches, the loss is
IN `simplify_p` itself -- our DP and C++'s `simplify_p` disagree at equal tolerance -- and the target
becomes the two implementations, which is a direct algorithm comparison and not a constant hunt.

## R674 — prediction confirmed: the true-input deficit is 1.652x

R674: PREDICTION CONFIRMED -- the deficit at the true input is 1.652x, not 1.069x

First confirmed prediction since R666, and the number is much larger than the one it was predicted to
explain. The outline coarseness is present before the perimeter generator touches anything, and
R670's 1.069x turns out to be the REMNANT of a bigger deficit that C++'s own simplification partly
erases. No Rust behaviour change (`SURFPROBE` is `probe_enabled`, default OFF); the C++
instrumentation is reverted with both status checks empty and the engine rebuilt pristine; all three
hashes unchanged (benchy 248ff22a, cube 14566293, majora 3d741dde); suites unchanged.

FIRST, A SCOPE CORRECTION TO R670. Its POLYPROBE "0 input outline" is `WallToolPaths::outline` --
which is `last_p`, already past `generate_arachne`'s own `simplify_p` AND its `offset_ex`. So R670
measured the outline AFTER the perimeter generator had simplified it, not as it arrives. `SURFPROBE`
counts `surface` at `generate_arachne`'s surface loop, matching C++'s `surface.expolygon` at
PerimeterGenerator.cpp:1511 before its `simplify_p`.

  points per contour                          Rust       C++     C/R
  surface.expolygon (BEFORE simplify_p)     50.956    84.158   1.652
  WallToolPaths outline (AFTER simplify_p)  45.693    48.829   1.069

  simplify_p KEEP rate                      0.8967    0.5802

**The deficit at the true input is 1.652x.** C++ then throws away 42% of its points and we throw away
10%, which is why the gap has shrunk to 1.069x by the time R670 was looking. Two engines converging
because the richer one discards more is not agreement, and reading the downstream number alone would
have kept understating the problem by a factor of fifteen (0.652 vs 0.069 in excess).

WHAT THAT MEANS FOR THE CHAIN. Every stage from `simplify_p` forward is now measured and none of
them originates anything: the prep chain (R670), the Voronoi diagram and its conversion (R669,
1.017x and 1.006x), `collapse_small_edges` (R668), the `isUpward` filter (R666). The 1.652x arrives
with `surface.expolygon`, which is the LayerRegion slice -- upstream of the perimeter generator
entirely.

AND IT SITS AWKWARDLY WITH R671, WHICH IS WHY R675 STARTS THERE. R671 verified the mesh slicer's DP
tolerance is faithful (`print_object.rs:452-453` mirrors `PrintObjectSlice.cpp:144` exactly,
hardcoded 0.0025 and the same ternary). So a 1.652x point loss appears between a faithful mesh
slicer and the LayerRegion slices. Something between those two points removes points that C++ keeps.

R675: the slice-to-region path. `PrintObjectSlice.cpp` applies `poly_ex.douglas_peucker(resolution)`
at four sites -- :509 and :567 in `groupingVolumes`, :600 in `applyNegtiveVolumes`, :613 in
`reGroupingLayerPolygons` -- and that `resolution` is a DIFFERENT variable from the `params_base.resolution`
R671 cleared. Census points-per-contour at the mesh slicer's output and after each of those four
sites, both engines. Predict the loss is at one of those `douglas_peucker` calls, since they are the
only point-removing operations between the slicer and the regions and R671 already cleared the one
constant that is shared. FALLBACK: if all four match, the loss is in how the slice ExPolygons are
converted into `LayerRegion::slices` -- a union or an offset that reconstructs contours -- and the
target becomes that conversion; say so.
**Do NOT assume the four sites use the same tolerance as the mesh slicer: R671 cleared
`params_base.resolution` specifically, and these read a separate `resolution` in scope. Check each.**

## R675 — the four douglas_peucker sites are first-layer brim only; my bracket mixed populations

R675: the four douglas_peucker sites are FIRST-LAYER BRIM ONLY -- and my bracket measured two
different populations

Prediction REFUTED at the target it named, and the round's own measurement failed for a reason worth
recording rather than a result worth reporting. No engine behaviour change (`SLICEPTS` is
`probe_enabled`, default OFF); all three hashes unchanged (benchy 248ff22a, cube 14566293, majora
3d741dde); suites unchanged.

THE PREDICTED TARGET IS NOT IN THE PATH. All four `douglas_peucker` calls
(PrintObjectSlice.cpp:509, :567, :600, :613) are reached from ONE caller --
`groupingVolumesForBrim` (:772) -- which passes `scaled_resolution` (:774) and operates on
`layers.front()`. They are the FIRST-LAYER BRIM grouping. They cannot produce a 1.652x
points-per-contour deficit across 4,720 layers, so the prediction is refuted structurally, the way
R661's was.

A PORTING GAP FOUND IN PASSING, AND IT IS NOT THIS BUG. We have no counterpart to
`groupingVolumes`, `applyNegtiveVolumes` or `reGroupingLayerPolygons` at all -- the names do not
appear anywhere in `crates/libslic3r-rs/src`. That is a genuine unported function group affecting
first-layer brim grouping on multi-volume objects. Worth its own round; it is NOT the 1.652x.

THE BRACKET I RAN DOES NOT ANSWER THE QUESTION, AND HERE IS WHY. `SLICEPTS` counts
points-per-contour at two points inside `PrintObject::slice()`:

  A region slices (pre make_slices)   contours=1,391   points=495,747   points/contour=356.396
  B lslices (post make_slices)        contours=1,391   points=501,553   points/contour=360.570
  SURFPROBE at generate_arachne       contours=26,123  points=1,340,410 points/contour=51.311

**1,391 contours become 26,123, and 495,747 points become 1,340,410.** The surfaces are subdivided
between the two sites -- `apply_mm_segmentation_tier1` splits region 0 into painted regions and every
new border adds vertices -- so points-per-contour at A and at SURFPROBE are not the same quantity
measured twice, they are two different populations. Comparing them says nothing about where points
are lost. That is R572/R585/R588's rule -- state the population -- and I violated it by placing a
probe where the population changes.

WHAT SURVIVES. The cross-engine number is unaffected: SURFPROBE is the same stage and the same
population definition on both engines, and it reads 51.311 against C++'s 84.158 (R674 read 50.956;
the drift is cumulative-at-modulus sampling, not a change). The 1.652x stands. Only its localisation
is still open.

R676: bracket it on the ONE population that is stable across the whole path -- total points and
total contours per LAYER, not per surface, measured on both engines at (1) the mesh slicer's output,
(2) after `make_slices`, and (3) at `generate_arachne`. Per-layer totals survive subdivision: splitting
one contour into five changes the contour count but not the point total except at new borders, so a
points-per-LAYER comparison localises real loss while points-per-contour does not. Predict the
per-layer point total is already short at the mesh slicer's output, since R671 verified the slicer's
DP tolerance is faithful and yet nothing downstream has been shown to remove points. FALLBACK: if the
slicer's per-layer totals MATCH and the deficit appears only later, the loss is in the segmentation
or region assembly, and the target is whichever of the three brackets it first appears in -- say
which.

## R676 — the layer population is 656; SURFPROBE's absolute totals are revisit-inflated

R676: the layer population is 656, not 4,720 -- and SURFPROBE's absolute totals are inflated by
repeated visits

No confirmed prediction and no localisation. What the round produced is a disqualification of the
absolute numbers I have been quoting on OUR side of the bracket, and the reason the last two attempts
to localise the 1.652x failed. No engine behaviour change (`SLICEPTS` is `probe_enabled`, default
OFF); all three hashes unchanged (benchy 248ff22a, cube 14566293, majora 3d741dde); suites unchanged.

THE POPULATION, PRINTED THIS TIME:

  A region slices (pre make_slices)   layers=656  nonempty=656  contours=1,391  points=495,747
                                      points/contour=356.396   points/layer=755.712
  B lslices (post make_slices)        layers=656  nonempty=656  contours=1,391  points=501,553
                                      points/contour=360.570   points/layer=764.562
  SURFPROBE at generate_arachne       surfaces=26,000  contours=26,122  points=1,331,805
                                      points/contour=50.984

**`self.layers` holds 656 layers, not the 4,720 I had been assuming from the gcode's
`; LAYER_HEIGHT:` count** -- that count includes the wipe tower and multiple entries per layer, so it
was never the layer count. 656 layers at 0.3 mm is ~197 mm, which is the model. Every layer is
non-empty, so bracket A is reading a populated field; R675's "1,391 contours across 4,720 layers is
not credible" was itself based on the wrong denominator.

AND THE BRACKET STILL DOES NOT COMPARE. 495,747 points at slice() become 1,331,805 at
`generate_arachne` -- 2.69x more -- across 656 layers and 26,000 surface visits. That is ~40 surface
visits per layer against 2.12 contours per layer at slice(). Segmentation subdivision cannot multiply
points by 2.69; the surplus is `generate_arachne` being entered repeatedly for the same geometry
(per region, and `make_perimeters` re-entering). **SURFPROBE accumulates over repeated visits, so its
absolute point and contour totals are inflated by an unknown revisit factor.**

WHAT THAT DOES AND DOES NOT INVALIDATE. It does NOT touch the 1.652x: SURFPROBE counts the same way
on both engines, at the same call site, so the revisit factor divides out of the RATIO. R674's
finding stands. What it invalidates is any attempt to chain SURFPROBE's absolute totals to an
earlier bracket's absolute totals -- which is precisely what R675 tried and what R676 was going to
try with per-layer sums. Per-layer totals fix the subdivision problem; they do not fix the
repeat-visit problem, so this round's plan was unsound before it ran.

R677: make the unit revisit-proof before comparing anything across brackets. Key SURFPROBE by
(layer_id, region_id) in a set and count each pair ONCE, on both engines; then per-layer point totals
are comparable to bracket A's. Predict the deficit is present at bracket A -- the same prediction
R676 carried, now with an instrument that can actually test it, since R671 cleared the slicer's
tolerance and nothing between has been shown to remove points. FALLBACK: if the deduplicated
per-layer totals MATCH at bracket A and diverge later, the loss is in segmentation or region
assembly and the target is whichever bracket it first appears in.
**BEFORE MEASURING: print the population (layers, and now also distinct (layer, region) pairs) at
EVERY bracket and confirm they agree. Three rounds in a row have been lost to comparing quantities
that were not the same quantity — R675 to subdivision, R676 to revisits, and R675's own critique to
a wrong layer count.**

## R677 — the 1.652× is created ENTIRELY by mm-segmentation, and the fix is already ported but parked

**Prediction REFUTED, pre-registered fallback FIRES and NAMES the bracket.**

R674 measured `surface.expolygon` at `generate_arachne` reading ~51 points-per-contour
against C++'s 84.158 (1.652×). R675 and R676 both failed to localise it because they
compared populations that were not the same population. R677 fixed the instrument first
and then measured the SAME field on the SAME layer population at the SAME two brackets on
BOTH engines.

C++ instrumentation: a `CPPUP`-gated file-scope helper `r677_bracket()` in
`PrintObjectSlice.cpp`, called at two points inside `slice_volumes()` — immediately after
the top-empty-layer trim (pre `apply_mm_segmentation`) and immediately after
`apply_fuzzy_skin_segmentation` (post segmentation, pre `InterlockingGenerator`). Rust: a
new `SLICEPTS` bracket C after `apply_mm_segmentation_tier1()`, plus the pair/surface
population at bracket A.

| bracket | engine | layers | pairs | surfaces | contours | points | pts/contour | pts/layer |
|---|---|---|---|---|---|---|---|---|
| A pre-segmentation  | C++  | 656 | 656 | 1,346 | 1,443 | 499,188 | 345.938 | 760.957 |
| A pre-segmentation  | Rust | 656 | 656 | 1,346 | 1,391 | 495,747 | 356.396 | 755.712 |
| C post-segmentation | C++  | 656 | 3,375 | 16,728 | 18,467 | 1,595,256 | 86.384 | 2431.793 |
| C post-segmentation | Rust | 656 | 3,437 | 26,620 | 26,743 | 1,360,799 | 50.884 | 2074.389 |

**Bracket A is IDENTICAL.** Layers 656 = 656, `(layer, region)` pairs 656 = 656, surfaces
1,346 = 1,346 — the populations agree exactly, so the point totals are comparable without
any correction. Points 499,188 vs 495,747 = 1.007×; points-per-contour 345.938 vs
356.396, with Rust marginally *higher*. The mesh slicer and region assembly are in
agreement. **The prediction that the deficit is already present at bracket A is wrong.**

**Bracket C is where all of it happens.** Points-per-contour 86.384 vs 50.884 = **1.698×**,
which is R674's 1.652× measured one stage earlier and on a population-matched unit. The
mechanism is now explicit: from the *same* 1,346 input surfaces we emit **26,620**
surfaces where C++ emits **16,728** — 1.59× more pieces — while producing *fewer* total
points (1,360,799 vs 1,595,256, 0.853×). We fragment the layer into more, smaller pieces.
Surfaces per `(layer, region)` pair: ours 7.74, C++'s 4.96.

**Two corrections to carried premises.**
1. **R676's revisit inflation is refuted.** Bracket C counts 26,620 surfaces and SURFPROBE
   counts ~26,000 surface visits, and the new call probe reads
   `generate_arachne calls=2000 distinct_layer_ids=342 calls_per_layer=5.848` against
   bracket C's 3,437/656 = 5.24 pairs per layer. `generate_arachne` is entered about once
   per `(layer, region)` pair. SURFPROBE's absolute totals are NOT revisit-inflated; the
   2.69× A→SURFPROBE growth R676 called impossible is real subdivision, and bracket C
   reproduces it exactly (495,747 → 1,360,799 = 2.75×).
2. **R675's "segmentation subdivides, so the brackets are incomparable" is only half
   right.** The populations do change across segmentation — but they change on BOTH
   engines, so bracketing *both sides of the same stage on both engines* is exactly the
   measurement that works. The error was measuring one engine only.

**The fix is already in the tree and switched off.** `apply_mm_segmentation_tier1`
(`print_object.rs`) carries faithful ports of both C++ cleanups:
- `MMSEG_OPENING` — `PrintObjectSlice.cpp:946-947`,
  `mine = opening(union_ex(mine), scale_(5 * EPSILON), scale_(5 * EPSILON))` on the base
  region's remainder. The C++ comment states our exact symptom: *"subtraction from
  layerm.region() could produce a huge number of small unprintable regions for the model's
  base extruder."*
- `MMSEG_CLOSING` — `PrintObjectSlice.cpp:962-964`,
  `closing_ex(src.expolygons, scale_(10 * EPSILON))` when a region received more than one
  contribution (`needs_merge`).

Both are `probe_enabled`, i.e. **default OFF**. They were parked at R557, whose own note
records the same quantity R677 has now re-derived independently: *"surfaces per
layer-region 7.78 -> 4.40 against C++'s 4.97"*. R557 rejected them because they cost
0.06pp of a **wall-lines IoU** metric and did not move a `; LINE_WIDTH:` ratio of 1.19 —
neither of which is the current bar. The acceptance test has since been raised to
line-level parity and the scoring metrics are now `line_parity.py` content and
`seq_parity.py` in-order. **A change parked on a superseded metric has to be re-scored on
the current one before it can stay parked.**

**R678: A/B `MMSEG_OPENING` and `MMSEG_CLOSING` on BOTH current metrics** with
`scripts/ab_template.sh` (exit-code-checked). Predict the in-order rate improves on
Majora, because the fragmentation feeds the Arachne input directly and R677 has now shown
it is the sole source of the 1.652×. Fallback: if both metrics are flat or worse, the
fragmentation is real but downstream-inert, and the next target is the point *count*
deficit at bracket C (1,360,799 vs 1,595,256) rather than the piece count.

## R678 — MMSEG_OPENING SHIPS: R557's parking decision reversed, the R677 root cause is closed

**Prediction CONFIRMED.** R677 localised the whole contour-coarseness deficit to
`apply_mm_segmentation_tier1` and found the C++ cleanup already ported and switched off.
R678 re-scored it on the current metrics and shipped it.

### Reachability and the geometric effect

Four arms on Majora, each exit-code-checked, each printing `SLICEPTS` bracket C:

| arm | majora hash | bracket C surfaces | points/contour |
|---|---|---|---|
| OFF | `3d741dde` | 26,620 | 50.884 |
| **MMSEG_OPENING** | `d6ccfdbb` | **14,924** | **88.736** |
| MMSEG_CLOSING | `3d741dde` (byte-identical) | 26,620 | 50.884 |
| both | `d6ccfdbb` | 14,924 | 88.736 |

C++ reads 16,728 surfaces / 86.384 points-per-contour. **`MMSEG_OPENING` alone takes the
deficit from 1.698× to 1.027×** — the root cause R677 named is closed by the change R557
parked. We now land 2.7% *above* C++'s points-per-contour and 11% *below* its piece count.

**`MMSEG_CLOSING` is inert here**, and structurally so: its `prev` is non-empty only when
an earlier painted extruder in the same layer already wrote to the same `region_id`, which
`PAINTED_REGION_DEDUP`'s one-region-per-filament map never produces on this model. The
byte-identical hash is the empirical confirmation.

### Scored on BOTH current metrics

| | OFF | ON | Δ |
|---|---|---|---|
| Majora content (order-blind) | 28.32% (711,537/2,512,604) | 28.24% (709,080/2,511,238) | **−2,457** |
| Majora IN ORDER | 18.65% (468,570) | **18.70% (469,489)** | **+919** |
| Benchy | `248ff22a` | `248ff22a` | byte-unchanged |
| cube | `14566293` | `14566293` | byte-unchanged |

Benchy and cube are single-material, so `apply_mm_segmentation_tier1` never runs — the
change is Majora-only by construction, not by luck.

**The per-feature split is what settles it.** Both metrics move in the *same two features*,
in *opposite directions*:

| feature | Δ our lines | Δ content-matched | Δ IN-ORDER |
|---|---|---|---|
| Outer wall | −843 | **−1,257** | **+802** |
| Inner wall | −277 | **−799** | **+407** |
| Sparse infill | +327 | −268 | −218 |
| everything else | −573 | −133 | −72 |

Outer wall in-order-as-a-fraction-of-content goes 43.3% → 44.1%; inner wall 55.0% →
56.5%. The change makes the wall *structure* right while shifting individual coordinates:
fewer lines match as unordered text, more of the ones that do match appear in the correct
sequence. The order-blind metric is precisely the one R651/R652 established is
insufficient, and the strict metric improves on exactly the features the change targets.

**Both prior parkings were driven by the ORDER metric getting worse** — R654
(content 0 / order −26,309) and R656 (content +1,204 / order −2,117). Here the order metric
improves. The precedent supports shipping, and the standing rule "score both rates before
shipping" is satisfied, not bypassed.

Both gates flipped from `probe_enabled` to `faithful_gate` (default ON, `=0` to disable).
All eight suites unchanged (multi_material_integration 25/26, pre-existing).

**NEW MAJORA BASELINE: `d6ccfdbb`** (was `3d741dde`, held since R651). Benchy `248ff22a`
and cube `14566293` unchanged.

**R679: the residual at bracket C.** With the fix in, our post-segmentation surfaces are
14,924 against C++'s 16,728 (0.892×) and our points 1,328,201 against 1,595,256 (0.833×) —
points-per-contour now matches but we still carry ~17% less boundary detail in total. The
piece count has crossed over from 1.59× too many to 0.89× too few, which suggests the
opening is slightly stronger than C++'s or that our painted/base partition differs in
extent. Measure the per-`(layer, region)` AREA at bracket C on both engines — a matching
piece count with a mismatched area means the partition is misplaced, a matching area with
a mismatched piece count means the cleanup tolerance is off. Predict the areas match and
the difference is in the cleanup. Fallback: if the areas differ, the target is the painted
partition (`multi_material_segmentation_by_painting_tier1`), not the cleanup.

## R679 — the areas match (partition is right, cleanup is the residual), and the number this chain has been anchored on is the WRONG number

**Prediction CONFIRMED.** Plus a re-census that redirects the campaign.

### 1. The area census — partition eliminated, cleanup named

`r677_bracket()` (C++) and the Rust bracket-A/C blocks extended with a total
`expolygon.area()`. Populations printed alongside, and they agree.

| bracket | engine | layers | pairs | surfaces | points | **area mm²** |
|---|---|---|---|---|---|---|
| A pre-segmentation | C++ | 656 | 656 | 1,346 | 499,188 | **1,189,508.593** |
| A pre-segmentation | Rust | 656 | 656 | 1,346 | 495,747 | **1,191,101.827** |
| C post-segmentation | C++ | 656 | 3,375 | 16,738 | 1,595,339 | **1,189,508.418** |
| C post-segmentation | Rust | 656 | 3,387 | 14,924 | 1,328,201 | **1,191,121.945** |

**The areas agree to 0.13% at both brackets** (Rust/C++ = 1.00134 at A, 1.00136 at C),
and both engines conserve area across segmentation — C++ by −0.175 mm² (−0.00001%),
ours by +20.1 mm² (+0.0017%; a shrink-then-grow opening can add area at concavities).

So: **matching area with a mismatched piece count (14,924 vs 16,738, 0.892×) and point
count (0.833×) means the CLEANUP TOLERANCE is off, not the partition.** The pre-registered
fallback — "if the areas differ, the target is
`multi_material_segmentation_by_painting_tier1`" — does **not** fire. The painted partition
is placed correctly; `opening_ex` is merging/removing slightly more than C++'s `opening`.

**UNITS TRAP found in passing.** `crate::SCALING_FACTOR` (`lib.rs:489`) is `100_000.0`
while `crate::libslic3r::SCALING_FACTOR` (`libslic3r.rs:19`, the mirror of
`libslic3r.h:58`) is `0.00001`. **They are reciprocals and both are in scope**, so
`area() * sf2` and `area() / sf2` are both plausible-looking and differ by 1e20. The first
run printed `area_mm2=1.19e26`. Caught only because the expected magnitude was known from
the C++ side — which is the argument for measuring the reference first. Commented at the
divisor. This is also a genuine hazard for the "files look like the C++" goal: two
constants, same name, reciprocal values, one crate.

### 2. The re-census — the anchor number is a consequence, not the cause

The `; LINE_WIDTH:` chain was measured entirely on the old baseline. Re-run on `d6ccfdbb`:

| | ours | C++ | ratio |
|---|---|---|---|
| outer-wall `; LINE_WIDTH:` tags | 19,707 | 62,582 | **3.18×** |
| outer-wall distinct values | 11,858 | 21,181 | 1.79× |
| outer-wall extrusion lines | **523,246** | **623,886** | **1.19×** |
| extrusion lines per tag | 26.6 | 10.0 | — |

**Distinct values moved from 11,845 to 11,858 — thirteen — despite R678 closing a genuine
1.698× geometric error.** The reason is now visible: the deficit is not in width VARIETY,
it is in PATH COUNT. We emit 3.18× fewer outer-wall `; LINE_WIDTH:` tags while carrying
only 1.19× fewer extrusion lines. Per emitted tag we have *more* width variety than C++
(0.602 distinct/tag vs 0.338).

Neither engine suppresses consecutive duplicates (ours 19,032 runs across 19,707 tags;
C++ 61,678 across 62,582), so under the shipped `LINEWIDTH_PERPATH` both emit **one tag
per ExtrusionPath**. C++'s outer wall is therefore **62,582 paths averaging 10.0 extrusion
lines**; ours is **19,707 paths averaging 26.6**. We merge what C++ keeps as separate
variable-width paths.

Per feature, this is specific to the wall family and absent elsewhere:

| feature | tag ratio | extrude ratio |
|---|---|---|
| Overhang wall | **11.57×** | 1.33× |
| Outer wall | **3.18×** | 1.19× |
| Inner wall | 1.67× | 1.13× |
| Floating vertical shell | 0.93× | 0.99× |
| Prime tower | 1.08× | 0.99× |

Floating vertical shell and Prime tower match on both counts, so this is not a global
emit-frequency difference — it is the wall path split specifically.

**R680: measure the PATH SPLIT, not the width values.** Count `ExtrusionPath` objects
produced per outer-wall loop on both engines at the point where Arachne's junctions become
paths (`thick_polyline_to_multi_path` on ours; `thick_polyline_to_extrusion_paths` in
`PerimeterGenerator.cpp` on C++), with the loop population printed alongside. Predict C++
starts a new path on every beading-width change while we coalesce consecutive junctions
whose widths are close, so its paths-per-loop is ~2.7× ours at equal loop count. Fallback:
if paths-per-loop MATCHES and the loop counts differ instead, the deficit is in loop
generation, not path splitting, and the target moves back to `WallToolPaths`. **Overhang
wall at 11.57× on 1.33× the lines is the sharpest instance — measure it too; a 3× outlier
inside the same family usually names the mechanism faster than the average does.**

## R680 — prediction AND fallback both refuted; the splitter is faithful and the deficit is its INPUT

R679 named the outer-wall `; LINE_WIDTH:` deficit as a 3.18× path-count deficit and sent
this round to `thick_polyline_to_multi_path`. Reading both implementations first:

- **The two splitters are line-for-line the same rule.** C++ (`VariableWidth.cpp:80-90`)
  starts a new `ExtrusionPath` when `scaled(|path.width - new_flow.width()|) >
  merge_tolerance`, and every Arachne call site (`Arachne/utils/ExtrusionLine.cpp:288/297/
  304/309`) passes `merge_tolerance = float(SCALED_EPSILON)` = 10 scaled units = 1e-4 mm.
  Ours (`variable_width.rs:241-253`) computes the same `scaled_f(|Δwidth|)` against
  `crate::libslic3r::SCALED_EPSILON = 10.0`, from the same four call sites. **No units bug,
  no tolerance difference, and the `-- i` re-examination on split is modelled correctly.**

So the split RULE was never a candidate. The measurement then had to be the rule's input.
`TPMPPROBE` (ours, already in the tree) and a matching `CPPUP`-gated counter added to
`VariableWidth.cpp`, both scoped to `erExternalPerimeter`/`ExtrusionRole::ExternalPerimeter`
so the populations are identical by construction:

| per outer-wall thick polyline | ours | C++ | C++/ours |
|---|---|---|---|
| calls (loop population) | 214,000 | 224,000 | 1.047 |
| width points | 7.114 | 7.924 | 1.114 |
| **width changes IN** | **0.0699** | **0.1960** | **2.802** |
| distinct widths IN | 1.046 | 1.156 | 1.105 |
| out paths | 1.059 | 1.188 | **1.122** |
| **flat calls (zero width change)** | **97.59%** | **91.31%** | — |
| **NON-flat fraction** | **2.41%** | **8.69%** | **3.605** |

**The prediction — "C++'s paths-per-loop is ~2.7× ours" — is REFUTED: it is 1.122×.**
**The fallback — "paths-per-loop matches and the LOOP COUNTS differ" — is also REFUTED:
the loop counts agree to 4.7%.** Neither branch of the pre-registered disjunction holds,
which is itself the result: the path split is not where the deficit is made.

**Where it is made: the width variation ENTERING the splitter, at 2.802×** — which is the
3.176× outer-wall tag ratio measured in the gcode at R679, arriving from upstream. Stated
the way that survives normalisation: **97.6% of our outer-wall thick polylines have ZERO
width variation along their entire length, against C++'s 91.3%. Only 2.41% of our outer
walls vary in width where 8.69% of C++'s do.** Given a flat input, a faithful splitter
correctly emits one path — so our 1.06 paths per loop is the *right* answer to the *wrong*
input.

**A correction to R679.** R679 asserted "both engines emit one tag per `ExtrusionPath`",
inferred from the shipped `LINEWIDTH_PERPATH` gate rather than measured. It is wrong:
scaled to equal loop counts we produce ~237,000 outer-wall paths for 19,707 tags (12.03
paths per tag) and C++ produces 266,029 for 62,582 (4.25 per tag). Both engines suppress
heavily; we suppress ~2.8× more, which is the same 2.80× input-variation ratio showing up
at the emitter because consecutive equal-width paths cannot produce a new tag value. The
gcode tag count is a faithful readout of input width variation — it just is not a path
count on either engine.

**TOOLING DEFECT FIXED.** The background wait-loops used
`read -t 3 </dev/null || true` as a sleep. Reading `/dev/null` returns EOF immediately, so
the delay was zero and the loop burned its whole iteration budget in milliseconds — it
"waited" by luck of ordering, not by timing. Replaced with a `python3` poll using
`time.sleep(3)`. `sleep` itself is blocked in the foreground; python is not.

**R681: measure the junction widths one stage upstream, per outer-wall LOOP.** The input to
`thick_polyline_to_multi_path` is `to_thick_polyline(extrusion)` over an
`Arachne::ExtrusionLine`, so the flatness is already present in the `ExtrusionJunction`
widths. Count, per outer-wall `ExtrusionLine` on both engines, the junctions and how many
adjacent junction pairs differ in `w`, printing the line population alongside. Predict the
2.80× is already there — the junctions themselves are flat and `to_thick_polyline` is a
faithful copy. Fallback: if the junction-level variation MATCHES and only the thick
polyline is flat, `to_thick_polyline` is quantising or averaging and that is the fix.
**Note that R661 "eliminated" the beading strategy and R585 the quantisation — both on the
pre-R678 baseline and both against the distinct-width metric that R679 disqualified. If
R681 lands on the beading, those eliminations must be re-opened rather than deferred to.**

## R681 — prediction CONFIRMED, and the 2.80× decomposes: edge count 2.1×, beading probability 1.39×

**Read both implementations first (R680's rule), and it settled the fallback before any build.**
`to_thick_polyline` is line-for-line identical on the two engines — C++
`Arachne/utils/ExtrusionLine.hpp:201-219` and ours `extrusion_line.rs:521-545`, same
`[j0.w, j1.w]` seed then `(prev.w, cur.w)` per subsequent junction, same for the
`ClipperLib_Z::Path` overload. **The pre-registered fallback — "`to_thick_polyline` is
quantising or averaging" — is structurally refuted.** It also means `TPMPPROBE`'s
`in_changes` already counts junction-to-junction changes exactly, because the widths array
is `[j0,j1, j1,j2, j2,j3, …]` and each junction transition appears once. **The prediction —
the 2.80× flatness is already present in the `ExtrusionJunction` widths — is CONFIRMED by
construction.**

Junction `w` is `beading->bead_widths[junction_idx]`
(`SkeletalTrapezoidation.cpp:1847`), so a flat wall means adjacent chained edges resolved
to equal bead widths. The archive already had the instrument: `BEADPAIR` (R585) measures
exactly P(adjacent beadings differ in `bead_widths[0]`), read-only. Mirrored it into C++
`generateJunctions` at the same point — immediately after
`getOrCreateBeading(edge->to, …)`, reading `edge->from` read-only.

| | ours | C++ | C++/ours |
|---|---|---|---|
| edges reaching `generateJunctions` | ≥3.0M | ≥6.5M | **1.86–2.33** |
| qualifying fraction (`both`/edges) | 0.1205 | 0.1277 | 1.06 |
| **P(adjacent beadings differ)**, matched on `both` (361,438 vs 355,825) | **0.0318** | **0.0443** | **1.393** |
| P(total_thickness differs) | 0.0072 | 0.0068 | 0.94 |

Both runs are truncated at their print modulus, so ours is in [3.0M, 3.5M) edges and C++'s
in [6.5M, 7.0M) — the edge ratio is bounded at 1.86–2.33×, midpoint ≈2.1×. The `both`
comparison is taken at matched population (361,438 vs 355,825, 1.6% apart) rather than at
each run's last block, so the probability ratio is not a truncation artefact.

**The decomposition: 1.393 × ≈2.1 ≈ 2.9×, which is R680's 2.802× width-changes-per-loop
and R679's 3.176× gcode tag ratio.** Two multiplicative terms, and **the dominant one is
the EDGE COUNT reaching `generateJunctions`, not the beading probability.** A wall's width
changes when it crosses from one node's beading to a different-valued one; with half as
many graph edges per wall there are half as many opportunities for that to happen, and the
per-opportunity probability is only 1.39× short.

This also re-frames the R585→R668 sub-chain. Those rounds measured the beading probability
and its near-boundary structure and found real but undersized effects (R662's 1.05× of
1.68×, R666's 1.022×, R668's 1.376×). They were undersized because **the beading
probability was never the larger term** — it is 1.39× of a ~2.9× product. R668's
near-boundary edge-count deficit of 1.62× was the closer measurement of the term that
actually dominates, and it was recorded as a supporting detail rather than the headline.

**R682: measure the graph edge count per wall loop on both engines, at the same site.**
`generateJunctions` iterates `graph.edges` wholesale, so its 2.1× is a whole-graph
property, not a per-wall one. Normalise it: count `graph.edges` per `WallToolPaths::generate`
call on both engines (the `GBUILD`/`CONV` probes already report `e_after_cells/call` and
`e_after_collapse/call` on our side — mirror them in C++ rather than writing new ones).
Predict the per-call edge deficit is ~2× and is already present at `e_after_cells`, i.e. in
the Voronoi→half-edge construction, before any collapse. Fallback: if `e_after_cells`
matches per call and only `e_after_collapse` differs, `collapse_small_edges` removes more
on our side — which R668 "eliminated" on a metric R679 has since disqualified, so re-open
it rather than defer to it. **Both prior GBUILD/CONV readings are from R589 on a long-dead
baseline; re-measure ours in the same run rather than carrying them.**

## R682 — prediction HALF right, fallback REFUTED, and a named unported feature found

Mirrored the Rust `CONV` probe's three stage boundaries into C++
`SkeletalTrapezoidation::constructFromPolygons` (before `separatePointyQuadEndNodes()`,
between it and `graph.collapseSmallEdges()`, and after), per call.

| whole-run mean per `constructFromPolygons` call | ours | C++ | C++/ours |
|---|---|---|---|
| calls | [24,000, 26,000) | [40,000, 42,000) | **1.54–1.75** |
| `e_after_cells/call` | 416.189 | 494.212 | **1.187** |
| `e_after_separate/call` | 416.189 | 494.212 | 1.187 |
| `e_after_collapse/call` | 324.827 | 384.334 | **1.183** |
| `n_after_cells/call` | 197.687 | 234.361 | 1.186 |
| `n_after_collapse/call` | 163.415 | 193.137 | 1.182 |
| **`collapse_keep`** | **0.7805** | **0.7777** | **0.996** |
| total edges after collapse | 7.80M | 15.37M | 1.97 |

**The fallback is REFUTED and `collapse_small_edges` is exonerated.** Its keep fraction is
identical to three decimal places (0.7805 vs 0.7777), and the per-call deficit is already
fully present at `e_after_cells` — before any removal runs. The R668 re-opening can be
closed: the collapse is not the mechanism, this time measured per-call with matched keep
fractions rather than against the disqualified distinct-width metric.

**The prediction is HALF right.** Its *location* was correct — the deficit is at
`e_after_cells`, in construction — but its *size* was wrong: 1.19×, not ~2×.

**So R681's ~2.1× decomposes again, and again the new factor dominates:**
**call count ≈1.64× × per-call edge density 1.183× ≈ 1.94×**, against the directly measured
total-edge ratio of 1.97×. Self-consistent.

*(Caveat on method: comparing each engine at call index 24,000 gives 1.386× for the
density, but C++'s first 24,000 calls cover only ~60% of the model while ours cover all of
it — a coverage mismatch. The whole-run means above are the population-correct figures,
and they reproduce the independently measured total ratio, which the index-matched
numbers do not.)*

### The call count: a named, unported feature

C++ has **seven** `Arachne::WallToolPaths` construction sites; we have seven too, and all
three fill sites (`FillConcentric.cpp:93`, `FillConcentricInternal.cpp:37`,
`FillFloatingConcentric.cpp:900`) are ported. But the **perimeter** path has four in C++
and only two in ours:

| C++ | ours |
|---|---|
| `PerimeterGenerator.cpp:1566` `one_wall_paths` (probe pass) | — |
| `PerimeterGenerator.cpp:1600` `paths_new` (remaining walls) | — |
| `PerimeterGenerator.cpp:1617` `one_wall_paths` | `perimeter_generator.rs:3184` |
| `PerimeterGenerator.cpp:1625` `normal_paths` | `perimeter_generator.rs:3197` |

We have only the `else` arm (C++ `:1614-1631`). **The entire `seperate_wall_generation`
branch (C++ `:1565-1613`) — "only generate one wall around top areas" — is unported.**
`seperate_wall_generation`, `should_enable_top_one_wall` (`PerimeterGenerator.cpp:1806`),
`generate_one_wall_by_top`, `generate_one_wall_by_top_most` and
`generate_one_wall_by_first_layer` appear **nowhere** in `crates/libslic3r-rs/src`. The
divergence is visible in one line:

```
C++  :1532  bool is_one_wall = loop_number == 0 || generate_one_wall_by_first_layer || generate_one_wall_by_top_most;
C++  :1534  bool seperate_wall_generation = !is_one_wall && generate_one_wall_by_top;
ours :3112  let is_one_wall = loop_number == 0;
```

Three of the four disjuncts and the whole second flag are missing. When the branch is
active C++ builds a skeletal graph **twice** for the same surface — once as a probe
(`one_wall_paths` at `:1566`, whose inner contour feeds `should_enable_top_one_wall`) and
again for the remaining walls (`paths_new` at `:1600`). That is a direct, arithmetically
plausible source of the 1.54–1.75× call-count term: it would need to fire on roughly
54–75% of surfaces.

This is a genuine `main.cpp`-reachable porting gap in its own right, independent of the
width-variation campaign — it changes the wall COUNT on top surfaces, not just the
instrumentation totals.

**R683: measure how often the branch fires before porting it (census before porting,
R650).** Add a `CPPUP` counter in C++ at `:1534` and `:1587` recording, per
`generate_arachne`-equivalent call: how many surfaces have `seperate_wall_generation` true
initially, how many survive `should_enable_top_one_wall`, and how many reach `:1600` with
`loop_number > 0`. Predict the surviving fraction times two, plus one, accounts for the
1.54–1.75× — i.e. roughly 54–75% of surfaces take the double-construction path. Fallback:
if the branch fires on far fewer surfaces than that, the call-count term has a second
source and the next place to look is how many surfaces reach the Arachne path at all on
each engine (bracket C put our surfaces at 14,924 against C++'s 16,738, only 1.12×, so a
gap that large would have to come from somewhere else entirely).

## R683 — prediction CONFIRMED quantitatively, with no C++ build: the branch is live on ~every surface, and the feature is ported into the WRONG PATH

Reading the config settled reachability before any instrumentation. Majora
(`$D/mj3mf/Metadata/project_settings.config`):

```
top_one_wall_type          = "all top"     -> TopOneWallType::Alltop
only_one_wall_first_layer  = 0
top_area_threshold         = 200%
wall_generator             = arachne
wall_loops                 = 2             -> loop_number = 1
```

Against `PerimeterGenerator.cpp:1528-1534`:

```
generate_one_wall_by_first_layer = only_one_wall_first_layer && layer_id == 0   -> ALWAYS FALSE (config 0)
generate_one_wall_by_top_most    = top_one_wall_type != None && upper_slices == nullptr  -> topmost layer only
generate_one_wall_by_top         = top_one_wall_type == Alltop && upper_slices != nullptr -> TRUE on every layer with an upper layer
is_one_wall                      = loop_number == 0 || … || …                   -> FALSE except the topmost layer
seperate_wall_generation         = !is_one_wall && generate_one_wall_by_top      -> TRUE on essentially EVERY perimeter surface
```

So C++ takes the double-construction path on essentially every perimeter surface of
Majora, and we take the single-construction `else` arm on all of them.

### The arithmetic closes exactly

Ours, one Rust run, both probes at once: `WTPCALL` (perimeter `WallToolPaths` calls) reads
**14,000** and `GBUILD` (all `constructFromPolygons` calls) reads **24,000**, each truncated
at its modulus of 2,000.

| | value |
|---|---|
| perimeter constructions P | [14,000, 16,000) |
| total constructions T | [24,000, 26,000) |
| perimeter share P/T | **0.538 – 0.667** |
| **predicted C++ total = T + P** (one extra construction per perimeter surface) | **[38,000, 42,000)** |
| **C++ measured (R682)** | **[40,000, 42,000)** |

The prediction was "the surviving fraction accounts for 1.54–1.75×, i.e. ~54–75% of
surfaces take the double-construction path". Measured perimeter share is 53.8–66.7% and the
predicted C++ call total contains the measured one. **Prediction CONFIRMED; the fallback
(a second source for the call-count term) does not fire.** The whole 1.54–1.75× call-count
term is accounted for by the one unported branch, and the three ported fill sites dilute it
from 2.0× to the observed value exactly as the model says they should.

`should_enable_top_one_wall` (`PerimeterGenerator.cpp:1806-1828`) is 22 lines: shrink the
top region by `(top_area_threshold/100) * max(ext_perimeter_spacing/2, perimeter_width/2)`,
drop it if the shrunk area is under 10% of the original or the original is under 1 mm², else
re-grow by `min_width + perimeter_width`.

### The feature IS ported — into the sibling path this config never takes

`top_one_wall_type`, `only_one_wall_first_layer` and `top_area_threshold` all exist in
`preset.rs`, `print_config.rs` and `perimeter_generator.rs`, `upper_slices` is available, and
the top-area shrink/re-grow logic is implemented at `perimeter_generator.rs:1088-1150` —
**all of it inside `generate_classic_one` (`:486`)**. `generate_arachne` (`:2937`) carries
none of it; its decision is the bare `let is_one_wall = loop_number == 0;` at `:3112`.
Majora sets `wall_generator = arachne`, so the entire feature is dead for this fixture.

This is the sharpest instance yet of R649's rule — **a ported function is not all its call
sites ported.** Nothing was missing from the config plumbing or the geometry helpers; the
decision simply was never wired into the Arachne branch.

No source was modified this round and no C++ patch was needed. The majora hash from this
round's own probe run is `d6ccfdbb`, unchanged; all eight suites unchanged
(multi_material_integration 25/26, pre-existing).

**R684: port the `seperate_wall_generation` branch into `generate_arachne`, gated
`ARACHNE_TOP_ONE_WALL`, and A/B it on both metrics.** The pieces: (1) extend `:3112` to
C++'s three-disjunct `is_one_wall` and add `generate_one_wall_by_top`; (2) port
`should_enable_top_one_wall` (or reuse the classic path's shrink/re-grow at `:1125-1150` if
the constants match — check, do not assume); (3) the probe construction at `:1566` feeding
`top_expolys_by_one_wall` via `diff_ex` against upper and lower slices; (4) the second
construction at `:1600` with `inset_idx += 1` on its lines before appending. Predict the
in-order rate improves on Majora — it doubles the outer-wall path population on top areas,
which is the term R680 measured as 3.18× short. Fallback: if in-order regresses, the branch
is correct but our downstream ordering cannot absorb the extra paths, and the gate stays
parked with the finding recorded. **Score BOTH metrics and split per feature before
deciding (R678); benchy and cube must be re-checked too — unlike `MMSEG_OPENING` this
change is NOT multi-material-only and will move every fixture.**

## R684 — the branch is PORTED and the call count now matches C++ exactly; prediction REFUTED, and the finding invalidates R681's use of the edge factor

The `seperate_wall_generation` branch (C++ `PerimeterGenerator.cpp:1528-1613` +
`should_enable_top_one_wall` at `:1806-1828`) is now implemented in `generate_arachne`,
gated `ARACHNE_TOP_ONE_WALL` (`probe_enabled`, **default OFF**).

**A sibling-path reuse was checked and rejected (R671→R672).** The classic path's
only-one-wall-top block (`generate_classic_one`, mirroring `PerimeterGenerator.cpp:1116-1183`)
shares the `min_width_top_surface` expression but also carries an `offset_top_surface` term
that `should_enable_top_one_wall` has no counterpart for. They are different computations,
so the helper was ported separately rather than reused.

**Two units decisions, both taken from the callee rather than from memory (R679).**
C++ builds `min_width_top_surface` from `scaled_spacing()`/`scaled_width()` and feeds
`offset_ex`, which takes scaled coords; our `offset_expolygons` takes **millimetres**, so
the widths come from `Flow::spacing()`/`Flow::width()`. And `ExPolygon::area()` returns
scaled² units, so C++'s `scale_(1)*scale_(1)` is `crate::SCALING_FACTOR²` (the `lib.rs`
constant, 100_000).

### Reachability: the call count now matches C++ exactly

| arm | majora hash | `GBUILD` calls |
|---|---|---|
| OFF | `d6ccfdbb` | 24,000 |
| **ON** | `b3d794f7` | **40,000** |
| **C++ (R682)** | — | **[40,000, 42,000)** |

`WTPCALL` also reports `onewall=1` on the ON arm — `generate_one_wall_by_top_most` firing on
the topmost layer, exactly as the config predicts. The 1.54–1.75× call-count deficit that
R682 measured and R683 attributed to this branch is **closed**.

### But the output barely moves — prediction REFUTED

| | OFF | ON | Δ |
|---|---|---|---|
| Majora content | 709,080 / 2,511,238 | 709,084 / 2,511,249 | **+4** |
| Majora IN ORDER | 469,489 | 469,475 | **−14** |
| Benchy | `248ff22a` | `248ff22a` | byte-unchanged |
| cube | `14566293` | `14566293` | byte-unchanged |

The prediction — "the in-order rate improves; it doubles the outer-wall path population on
top areas" — is **refuted**. The pre-registered fallback fires: **the gate is parked**,
default OFF.

### Why, measured rather than guessed

A `TOPONEWALL` census at the `should_enable_top_one_wall` call site:

```
[TOPONEWALL] probed=14000 survived=4 (0.0003) top_empty_after=13996
```

**Four surfaces out of 14,000 survive the test.** That is the feature working as designed —
`should_enable_top_one_wall` discards any top region whose shrunk area is under 10% of the
original, and on a tall model like Majora almost every layer's genuine top area is far below
that. The probe construction at `:1566` runs on every surface and its result is thrown away
99.97% of the time.

**This corrects R681's decomposition.** R681 factored the outer-wall width-variation deficit
as *edge count ~2.1× × beading probability 1.393× ≈ 2.9×*, and R682 factored the edge count
as *call count ~1.64× × per-call density 1.183×*. But **the call-count factor is a discarded
probe pass** — those extra graphs never reach `thick_polyline_to_multi_path` and cannot
create width-change opportunities in the G-code. C++ pays exactly the same wasted cost.
Removing it from the product leaves **1.183 × 1.393 = 1.648×** acting on the output against
the 2.802× width-changes-per-loop deficit R680 measured, so **roughly 1.7× of that deficit is
now unexplained again** — a real regression in the causal account, and better to know it than
to keep multiplying a factor that does no work.

Parking is also the right call for slicing time (ask #3): shipping it would double our
Arachne graph constructions on Majora to buy a −14 in-order change. C++ pays that cost; we
need not.

Baselines unchanged with the gate off: benchy `248ff22a`, cube `14566293`, majora
`d6ccfdbb`. All eight suites unchanged (multi_material_integration 25/26, pre-existing).

**R685: re-derive the width-variation account now that the edge factor is disqualified.**
Re-run `TPMPPROBE` and `BEADPAIR` with `ARACHNE_TOP_ONE_WALL=1` on our side: if the extra
probe graphs are excluded from the OUTPUT path, our width-changes-per-loop should be
unchanged (0.0699) while the raw edge totals double — confirming that edges-in-discarded-
graphs inflate R681's numerator on BOTH engines. Then re-measure C++'s
`in_changes`-per-outer-wall-loop against ours as the ONLY output-side quantity that matters,
and attribute the residual 1.7× from scratch. Predict the per-loop figures are unchanged by
the gate. Fallback: if our per-loop width changes DO rise with the gate on, the probe graphs
are not fully discarded and the branch's second construction is contributing after all —
in which case re-examine whether the four surviving surfaces are the only ones that should
survive.

## R685 — prediction CONFIRMED, and every ratio in the decomposition was measured on an unmatched population

The `ARACHNE_TOP_ONE_WALL` gate turns out to be a **population-matching instrument**: with it
on, our skeletal-graph population becomes C++'s (40,000 `constructFromPolygons` calls vs
C++'s 40,000) while the G-code moves by +4/−14 lines. That makes it possible, for the first
time, to compare whole-graph quantities on like populations.

### The prediction: output-side figures are unchanged by the gate

| | OFF | ON | ratio |
|---|---|---|---|
| TPMPPROBE calls (outer-wall thick polylines) | 214,000 | 214,000 | 1.000 |
| **`in_changes` (output-side width changes)** | **15,007** | **15,045** | **1.0025** |
| `flat_calls` | 208,831 | 208,817 | 1.000 |
| `out_paths` | 226,552 | 226,617 | 1.000 |
| BEADPAIR edges (whole graph) | 3,000,000 | **5,500,000+** | **≥1.83** |

**Prediction CONFIRMED.** The same model, sliced twice, has *identical* output-side width
variation while its whole-graph edge total nearly doubles. That is a direct demonstration —
on one engine, holding the output fixed — that **whole-graph edge totals are decoupled from
output-side width-change opportunity.** The fallback (per-loop changes rising with the gate)
correctly did not fire.

### The consequence: both R681 and R682 ratios were unmatched, and both move

C++ *always* runs the probe pass, so its graph population always contains probe graphs. Our
OFF arm contains none. Every cross-engine graph ratio taken before this round therefore
compared a probe-contaminated population against a clean one. Re-measured with the gate on,
so both sides carry probe graphs:

| quantity | as measured before | matched population | direction |
|---|---|---|---|
| per-call skeletal edge density (`e_after_collapse/call`) | 1.183× (R682) | **1.091×** | shrinks |
| P(adjacent beadings differ in `bead_widths[0]`) | 1.393× (R681) | **1.823×** | **grows** |
| whole-graph edge total | ~2.1× (R681) | dissolved (R684) | gone |

Per-call figures on matched 40,000-call populations: `e_after_cells` 451.623 vs 494.212,
`e_after_collapse` 352.165 vs 384.334, `n_after_collapse` 177.084 vs 193.137,
`collapse_keep` 0.7798 vs 0.7777 — the collapse stays exonerated (R682) and the construction
deficit is only 1.09×.

### Where the account now stands

```
output-side deficit (in_changes per outer-wall loop)   2.802x   (ours 0.0699-0.0703, C++ 0.1960)
  per-call skeletal edge density                       1.091x
  P(adjacent beadings differ)                          1.823x
  product                                              1.995x
  RESIDUAL                                             1.405x  still unexplained
```

**The beading-difference probability is now by far the dominant measured term** — 1.823×,
where R681 sized it at 1.393× and treated it as the minor factor. That vindicates the
re-opening of the **beading strategy (R661)** and the **Arachne quantisation constants
(R585/R657)**: both were eliminated against a metric R679 disqualified, and the term they
govern has just doubled in importance.

Two caveats stated plainly. First, the two surviving factors are whole-graph quantities used
to explain an output-side deficit; they are probe-contaminated *equally* on both sides now,
which makes the comparison fair but does not make the product a proven decomposition.
Second, the residual is 1.405× — smaller than R684's ~1.7×, but only because the beading term
grew, not because anything new was explained.

No source was modified this round. All eight suites unchanged (multi_material_integration
25/26, pre-existing); the OFF arm reproduced majora `d6ccfdbb`.

**R686: go straight at the beading strategy — it is now the largest measured term (1.823×).**
Compare `BeadingStrategy::compute` output on both engines for the same input width: the chain
is `WideningBeadingStrategy` → `DistributedBeadingStrategy` → `RedistributeBeadingStrategy` →
`LimitedBeadingStrategy` (`BeadingStrategyFactory.cpp`). Instrument at the outermost
`compute(thickness, bead_count)` with a histogram of `(thickness bucket) -> distinct
bead_widths[0]`, scoped identically on both engines, and print the call population. Predict
our strategy returns a *coarser quantisation* of `bead_widths[0]` — fewer distinct values per
thickness bucket — which is exactly what would make adjacent nodes resolve to equal widths.
Fallback: if the per-bucket distinct counts match, the strategies agree and the difference is
in *which* thickness each node presents, i.e. `distance_to_boundary`, and the target moves to
the graph geometry rather than the strategy. **Run our side with `ARACHNE_TOP_ONE_WALL=1` so
the populations match C++ (R685) — this is now mandatory for every whole-graph comparison.**

## R686 — prediction REFUTED: the beading strategy is NOT the flattener, and it is slightly RICHER than C++'s

Read both factory chains first. They are the same construction —
`DistributedBeadingStrategy` → `RedistributeBeadingStrategy` → `WideningBeadingStrategy`
(if `print_thin_walls`) → `OuterWallInsetBeadingStrategy` (if `outer_wall_offset != 0`) →
`LimitedBeadingStrategy` — in the same order, from
`BeadingStrategyFactory.cpp:35-56` and `beading_strategy_factory.rs:51-110`. The four
`beading_strategy.compute` call sites also correspond 1:1: C++ `:1526/:1536/:1537/:1887`
against ours `:2710/:2726/:2729/:3599`.

**Instrumented the two sites whose result is STORED** (C++ `:1535` and `:1926` after the
relocation, ours `:2710` and `:3599`) — the beadings `generateJunctions` later reads. The
other three `setBeading` sites store interpolated or propagated copies, not strategy
output, so they are out of scope for "what does the strategy return". Census: bucket the
input thickness into 0.05 mm bins (5000 scaled units) and collect the set of distinct
`bead_widths[0]` returned in each bin. Our side ran with `ARACHNE_TOP_ONE_WALL=1` so the
graph population matches C++'s (R685) — and it did: **384 buckets on both engines**, and
store counts within 1.7% (ours 1,180,000, C++ 1,200,000).

| at largest available store count | ours (1,180,000) | C++ (1,200,000) |
|---|---|---|
| thickness buckets | **384** | **384** |
| distinct `bead_widths[0]` total | 36,171 | 34,114 |
| **distinct per bucket** | **94.20** | **88.84** |

Per-bucket detail at the low end (`b6`…`b11`): ours 535 / 2,520 / 2,684 / 3,125 / 3,226 /
3,843 against C++'s 647 / 2,511 / 2,571 / 2,800 / 2,918 / 3,382 — ours is higher in five of
six.

**The prediction — "our strategy returns a coarser quantisation of `bead_widths[0]`" — is
REFUTED.** Our strategy is not coarser; for the same thickness bucket it returns *at least
as many* distinct widths as C++'s. **The pre-registered fallback fires: the strategies
agree, so the difference is in WHICH thickness each node presents — `distance_to_boundary`
— and the target moves to the graph geometry rather than the strategy.**

**Instrument caveat, stated plainly.** The census is cumulative-at-modulus and both engines
accumulate their sets in traversal order, so the intermediate ratio swings badly — 0.489× at
800,000 stores, 0.955× at 1,000,000, 1.240× at 1,100,000. Only the near-complete figures are
comparable, and even those carry the residue of a 1.7% store-count difference. What is solid
is the *direction and the bucket count*: 384 = 384 buckets, and our per-bucket distinct
count is not below C++'s at any near-complete point. The claim being made is "not coarser",
not "richer by 6%".

**This re-closes R661.** The beading strategy was eliminated at R661 against a metric R679
disqualified, and R685 promoted the term it governs to 1.823× — the largest measured factor.
R686 re-tested it properly, on a matched population, with the right unit, and the
elimination holds: **the strategy is exonerated as a width-flattener.** The 1.823× BEADPAIR
deficit is therefore not produced by the strategy's mapping from thickness to width; it must
come from adjacent nodes presenting *more similar thicknesses* on our graph.

Also recorded from `BEADPROBE` on the same run: over 1,180,000 stores our thickness values
take **297,739 distinct values** spanning 0.023–19.652 mm, and `bead_widths[0]` takes 28,133
distinct values spanning 0.190–0.762 mm. C++'s counterpart figure was not captured this
round and is the obvious next measurement.

Baselines unchanged: benchy `248ff22a`, cube `14566293`, majora `d6ccfdbb`. All eight suites
unchanged (multi_material_integration 25/26, pre-existing). C++ submodule reverted; both
status checks empty; rebuilt.

**R687: measure `distance_to_boundary` on adjacent nodes — the quantity the fallback named.**
Extend `BEADPAIR` (which already walks `edge->from` / `edge->to` read-only) to record, per
qualifying edge, `|dtb(to) − dtb(from)|` bucketed, plus the count of edges where the two are
*exactly equal*. Mirror it in C++ at the same point in `generateJunctions`. Run our side with
`ARACHNE_TOP_ONE_WALL=1`. Predict our exactly-equal fraction is markedly higher — that is
the only remaining way to get 1.823× fewer differing beadings out of a strategy that is not
coarser. Fallback: if the `dtb` differences match too, then equal thicknesses are not the
route and the deficit is in the *bead_count* argument rather than the thickness, so bucket by
`(bead_count(to), bead_count(from))` instead — `compute` takes both, and R681 measured
`bead_widths[0]` only, which is blind to a bead-count difference that reshuffles the rest of
the vector.

## R687 — prediction REFUTED structurally (`dtb_eq = 0` on BOTH engines), and reading the strategy explains why `bead_widths[0]` barely varies at all

Extended `BEADPAIR` with `|dtb(to) − dtb(from)|` bucketed and an exactly-equal counter,
reported as **rates over the edge denominator** rather than a growing distinct-set (R686's
instrument was order-dependent). Mirrored into C++ `generateJunctions` at the same point,
immediately after `getOrCreateBeading(edge->to, …)`, reading `edge->from` read-only. Our
side ran with `ARACHNE_TOP_ONE_WALL=1` (R685).

| at block index 5,500,000 | ours | C++ |
|---|---|---|
| **dtb exactly equal** | **0** | **0** |
| dtb < 1 µm | 5,804 (0.106%) | 10,236 (0.186%) |
| dtb 1–10 µm | 39,991 (0.727%) | 50,483 (0.918%) |
| dtb > 10 µm | 5,454,205 (99.17%) | 5,439,281 (98.90%) |
| both-have-beading | 670,299 | 724,315 |
| **P(differ in `bead_widths[0]`)** | **0.0244** | **0.0432** |

**The prediction — "our exactly-equal `dtb` fraction is markedly higher" — is REFUTED
structurally.** It is *zero on both engines*, and it must be: `generateJunctions` keeps only
the upward half-edges (`from.dtb > to.dtb` → `continue`) and skips `end_R >= start_R`, so
the two endpoints have strictly different `distance_to_boundary` by construction. The
hypothesis was not merely wrong, it was unaskable at this site — the same class of error as
R661 and R675.

Worse for it: in the near-equal buckets **ours is the sparser side** — 0.106% vs 0.186%
below 1 µm. Our adjacent nodes present *more* differing thicknesses than C++'s, and still
get the *same* `bead_widths[0]` 1.77× more often. Thickness cannot be the route.

*(Caveat: the block indices match at 5,500,000 but the coverage does not — our edge total is
in [5.5M, 6.0M) and C++'s in [6.5M, 7.0M), so ours is near-complete while C++'s is at ~85%.
Treat the distribution rows directionally. `dtb_eq = 0` is exact and unaffected, and the
P(differ) figure is consistent with R685's 1.823× taken at a different truncation.)*

### Reading the strategy explains the whole shape

`RedistributeBeadingStrategy.cpp:83` — and `redistribute_beading_strategy.rs:151-155`,
which is identical:

```cpp
const coord_t actual_outer_thickness = bead_count > 2 ? std::min(thickness / 2, optimal_width_outer)
                                                      : thickness / bead_count;
```

`bead_widths[0]` *is* `actual_outer_thickness`. So:
- **`bead_count > 2`** and the wall thicker than `2 * optimal_width_outer` → `bead_widths[0]`
  is **pinned to `optimal_width_outer`**, a constant, no matter how the thickness varies.
- **`bead_count <= 2`** → `thickness / bead_count`, which varies continuously.

**Two adjacent nodes can therefore only differ in `bead_widths[0]` when at least one of them
is in the `bead_count <= 2` (or sub-`minimum_variable_line_ratio`) regime.** P(differ) is a
measure of how often the pair straddles that regime boundary — which is a **bead-count**
question, not a thickness question. That is exactly the pre-registered fallback, now
supported by the source rather than only by elimination.

It also retro-explains R686: our strategy returned *more* distinct `bead_widths[0]` per
thickness bucket precisely because those distinct values come from the `thickness /
bead_count` branch; a richer spread there is consistent with, not contradictory to, a lower
P(differ) overall.

Baselines unchanged: benchy `248ff22a`, cube `14566293`, majora `d6ccfdbb`. All eight suites
unchanged (multi_material_integration 25/26, pre-existing). C++ submodule reverted; both
status checks empty; rebuilt.

**R688: measure `bead_count` at the two endpoints — the fallback, now source-supported.**
Extend the same `BEADPAIR`/`r687_note` pair with `(bead_count(to), bead_count(from))`: the
rate at which the two are equal, the rate at which **both are > 2** (the pinned regime where
`bead_widths[0]` cannot differ), and the rate at which the pair **straddles** the `> 2`
boundary. Predict our both-pinned rate is markedly higher — that is the only remaining way
to get 1.77× fewer differing widths out of an identical strategy fed *more*-varied
thicknesses. Fallback: if the bead-count regimes match too, then `bead_widths[0]` is the
wrong observable entirely and the deficit lives in the *rest* of the width vector — re-run
the comparison on `bead_widths[idx]` for the idx each junction actually uses
(`SkeletalTrapezoidation.cpp:1847` passes `junction_idx`), which R681/R685/R687 have all
been blind to. **Run our side with `ARACHNE_TOP_ONE_WALL=1`, and this time also print each
engine's TOTAL edge count so the coverage mismatch that clouded this round's distribution
rows can be corrected for.**
