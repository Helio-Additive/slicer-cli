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
