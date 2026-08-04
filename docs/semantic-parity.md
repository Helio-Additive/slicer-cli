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
