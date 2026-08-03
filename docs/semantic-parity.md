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
