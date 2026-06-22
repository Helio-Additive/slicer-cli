# Cooling Slowdown Parity — Findings

Branch: `cooling-slowdown` (from `alex/libslic3r-parity-engine` @ 65e4500)
Material: Bambu PLA Basic @BBL H2D; Process: 0.20mm Standard @BBL H2D
Native config: `slow_down_layer_time = 4` s, `slow_down_min_speed = 20` mm/s, `slow_down_for_layer_cooling = 1`.

## M1 — DATA: native vs rust per-layer slowdown

### Top-line
- Native estimated print time: **43m 0s**. Rust: **33m 14s**. Gap ~9m45s.
- Both emit 240 `; CHANGE_LAYER` markers (layer count converged).

### Per-layer extrusion feedrate (mm/min), small upper Benchy layers
(min / max / avg over extruding moves)

| layer | nat_min | nat_max | nat_avg | rust_min | rust_max | rust_avg |
|------:|--------:|--------:|--------:|---------:|---------:|---------:|
| 195 | 2405 | 18424 | 8369 | 12000 | 21000 | 14521 |
| 196 | 2262 | 11213 | 9931 | 12000 | 18000 | 14257 |
| 197 | 2112 | 9706 | 8597 | 12000 | 18000 | 14419 |
| 198 | 1956 | 8097 | 6717 | 12000 | 18000 | 14241 |
| 199 | 1798 | 6269 | 5189 | 12000 | 18000 | 14296 |
| 200 | 1640 | 4947 | 4132 | 12000 | 18000 | 14045 |
| 201 | 1494 | 2943 | 2557 | 12000 | 15000 | 13696 |
| 202 | 1200 | 1200 | 1200 | 1200 | 1200 | 1200 |

Native progressively slows ALL features on small upper layers (toward `slow_down_min_speed`=20 mm/s=1200 mm/min for the outer wall) to hit the 4 s minimum layer time. Rust applies **ZERO slowdown** on 195–201 — feedrates stay at nominal (outer 12000, inner 18000, infill 15000).

Per-feature, native layer 198 vs rust layer 198 (F in mm/min):
- Native: Inner 8097, Outer ~1970, Internal solid 8097, Top 8097, Gap 8097 (everything slowed to ~135 mm/s, outer wall to ~32 mm/s).
- Rust:   Inner 18000, Outer 12000, Top 12000, Gap 15000 (all nominal — NOT slowed).

### Config wiring — OK (not the bug)
Runtime instrumentation (COOLDBG) of the live `calculate_layer_slowdown_postproc`:
```
L198 et0=0 nadj_total=46 enabled=[true] sdlt=[4.0] sdms=[20.0]
     tier0 total=2.222 <= sdlt=4.004 max_time=22.824 stretch=1.782 logic=0
```
- `slow_down_layer_time`=4.0, `slow_down_min_speed`=20.0 DO reach the live CoolingBuffer.
- `slow_down_layer_time`/`slow_down_min_speed` are handled in BOTH `PrintConfig::set_deserialize` (live path, line ~2336) AND `apply_key_value` (line ~4552); both write the same scalar fields. No M204-style wiring gap.
- The decision logic correctly ENTERS the slowdown branch with a positive `time_stretch` (e.g. 1.782 s on L198) and calls `non_proportional_slowdown` (logic=0).
- `; CHANGE_LAYER` layer split processes the final layer too (sentinel push + trailing append). Final layer is NOT skipped.

### ROOT CAUSE — f32 EPSILON too small in `non_proportional_slowdown`
File: `crates/libslic3r-rs/src/gcode/cooling.rs:13`
```rust
const EPSILON: f32 = 1e-6;
```
C++ uses `static constexpr double EPSILON = 1e-4;` (libslic3r.h:52).

The non-proportional span-finding loop (cooling.rs ~2854):
```rust
while adj.idx_line_end < adj.n_lines_adjustable
    && adj.lines[adj.idx_line_end].feedrate > feedrate - EPSILON
{ adj.idx_line_end += 1; }
```
With f32 EPSILON=1e-6 and feedrate ~300 mm/s: `300.0_f32 - 1e-6 == 300.0_f32` (1e-6 is below f32 ULP at 300, ~3e-5). So `300.0 > 300.0` = **false** — the span body never runs, `idx_line_end` stays 0, `feedrate_next == feedrate` (300), `feedrate_limit=300`, `time_stretch_max=0`, the loop makes no progress and spins to the 1000-iteration guard, slowing **zero** lines (`nslowed=0`, layer time unchanged).

Verified: `300.0_f32 > 300.0 - 1e-6` → false; `300.0_f32 > 300.0 - 1e-4` → true (299.9999).

COOLDBG3 trace (L198, stretch=1.782):
```
loop=1..1000 tier=0 feedrate=300 feedrate_next=300 feedrate_limit=300 tsm=0.0000 stretch=1.7820 ibeg=0 iend=0 f[0]=300 f[1]=250 f[iend]=300
```
`iend` stuck at 0 → infinite no-op loop → no slowdown.

### Fix (M2)
Set `EPSILON` to `1e-4` to match C++ (`libslic3r.h:52`). This affects the feedrate-tier comparisons in `non_proportional_slowdown` / `consistent_surface_slowdown` and the debug asserts. After fix, the span will advance, tiers will be processed, and `slow_down_to_feedrate` will slow lines toward `slow_down_min_speed` to meet `slow_down_layer_time`.

Note: there is also a structural divergence — `slow_down_to_feedrate` / `time_stretch_when_slowing_down_to_feedrate` iterate `0..n_lines_adjustable` (matches C++ header), good. But the early "merge travel into previous modifier" in parse is a separate fidelity item; not the cause of zero-slowdown.

## M2 — FIX APPLIED

`crates/libslic3r-rs/src/gcode/cooling.rs:13`: `const EPSILON: f32 = 1e-6` -> `1e-4`
(matches C++ `libslic3r.h:52`). Cite CoolingBuffer.cpp:178,210,244 — span-find and
tier-skip comparisons `feedrate > feedrate - EPSILON` / `slow_down_min_speed > ... - EPSILON`.

### Per-layer extrusion F (avg mm/min): before -> after vs native
| layer | native_avg | rust_before | rust_after |
|------:|-----------:|------------:|-----------:|
| 195 | 8369 | 14521 | 10690 |
| 197 | 8597 | 14419 | 9384 |
| 198 | 6717 | 14241 | 7106 |
| 200 | 4132 | 14045 | 4406 |
| 201 | 2557 | 13696 | 2508 |

Rust now applies min-layer-time slowdown; per-layer avg F tracks native closely.
COOLDBG AFTER fix: L198 nslowed=46, slowed to ~118 mm/s, layer time 2.222 -> 4.047 s
(target 4.0). Layers 195-201 all reach ~4.0-4.5 s.

### Preserved invariants
- Material: TOTAL 3858.97 native vs 3852.96 rust = **0.9984x** (still 0.998x parity).
- Move count: cooling only rewrites/removes F tokens, never adds/removes G-moves.
- Rust header estimated time 33m14s -> 35m11s (rust's own estimator; native 43m).

### Residual cooling gap (separate parity item, NOT CoolingBuffer math)
Native slows the OUTER WALL to ~1956-1640 mm/min on these layers — this is the
small-perimeter/overhang speed limit applied PRE-cooling (native L188 with NO cooling
slowdown already has outer wall at 1440-3385), not cooling. Rust's outer wall is full
nominal speed pre-cooling, so cooling equalization lands at one common feedrate (7106)
vs native's two tiers (8097 non-outer / 1956 outer). Closing this needs the
small-perimeter outer-wall speed reduction (separate gap).
