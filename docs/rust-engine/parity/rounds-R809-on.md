# Rounds from R809: against this repository's `slicer_cli`

From R809 the oracle is this repository's `slicer_cli` (Bambu Studio
02.09.01.70, `references/BambuStudio` at `56a57d0`), not the 02.06.00.51 engine
the earlier rounds were scored against. Scores are in-order matched lines from
`scripts/parity/seq_parity.py`, on the job configs in
`tests/rust-engine/configs/` (`benchy-016`, `benchy-016-arachne`, `nu3mf` for
Majora), with both programs built on the same machine against the same Eigen
(5.0.1) and Boost (1.90) headers.

## How the scores are taken

`slicer_cli` is not reproducible run to run (about 900 lines on Majora), so from
R810 each fixture's oracle G-code is sliced once and kept, and every round's Rust
G-code is scored against that same file. The earlier state of the Rust engine is
re-measured against it through the gates (`NAME=0`), which is where the "before"
numbers come from. From R810 the STL jobs' settings bundle also keeps
`printer_model`, so `slicer_cli` treats the printer as the Bambu Lab printer it
is (main.cpp 11010-11023); without it the oracle left out the Bambu-only lines.

## Baseline

The Rust engine as it came from R808, against the kept oracle files:

| Fixture | In order | Content |
|-|-|-|
| Benchy classic | 30,486 (15.72%) | 41,423 (21.36%) |
| Benchy arachne | 32,769 (13.68%) | 55,458 (23.15%) |
| Majora | 1,512,438 (55.88%) | 1,745,378 (64.49%) |
| Majora, prime tower | 83.5% | |

## Rounds

| Round | What | Gates | Benchy classic | Benchy arachne | Majora |
|-|-|-|-|-|-|
| R809 | STL placed on the plate as `slicer_cli` does (`desktop_place_on_bed`, cli_model_load.cpp 280-378): mesh stored minus its bounding-box centre in f32, volume offset re-centred twice in f64, instance at the bed centre with a truncating `new_scale` shift (160 mm scales to 15999999), sliced through the exact composed f32 transform | `STL_DESKTOP_PLACE` (on) | 30,486 → 50,316 (25.91%) | 32,769 → 67,675 (28.27%) | unchanged (3MF path) |
| R810 | (1) Perimeter simplification read `enable_arc_fitting` as always on; the 02.09.01.70 H2D process profiles turn it off, and walls and gap fills are then simplified at the full `resolution` (PerimeterGenerator.cpp 924, 1521). (2) The 02.09.01.70 machine G-code placeholders `initial_no_support_hotend`, `current_hotend`, `initial_nozzle_id`, `nozzle_diameter_at_nozzle_id`, `long_retraction_when_ec`, `retraction_distance_when_ec` (GCode.cpp 2918-2937). (3) `slicer_cli`'s alias of `initial_no_support_filament_id` (main.cpp 486-580). (4) What `_do_export` writes after the last layer: power-loss line, final retract with a spiral lift, fans off, `M981 S0`, the role tag, `filament_end_gcode` through `writeln` (GCode.cpp 3548-3627). (5) `is_BBL_Printer` by `slicer_cli`'s rule. (6) Probe `PP=1`: per-layer checksums at the step boundaries, with a C++ twin kept out of the repository | `PG_ARC_FITTING_CFG`, `START_GCODE_2091`, `GCODE_TOKEN_ALIAS`, `END_SEQUENCE_CPP`, `BBL_PRINTER_RULE` (all on) | 50,316 → 124,330 (66.46%) | 67,675 → 126,359 (53.35%) | 1,512,438 → 1,512,509 |
| R811 | (1) The seam placer's occlusion mesh is built from the volume's own f32 mesh, through the volume matrix before the decimation and `trafo_centered()` after it (SeamPlacer.cpp 577-604); rounding the plate-frame mesh lost the low bits and flipped the seam between symmetric candidates. (2) In a wipe the pending acceleration comes after the wipe's speed line (GCode.cpp 496-510) | `SEAM_OCCL_LOCAL`, `WIPE_ACCEL_AFTER_SPEED` (on) | 124,330 → 137,832 (73.77%) | 126,359 → 136,442 (57.48%) | 1,512,509 → 1,512,563 |
| R812 | Farthest-point timelapse, new in 02.09: `compute_farthest_point` (GCode.cpp 4582-4733), the inline photo block of `generate_timelapse_gcode` (4735-4840), `check_and_insert_timelapse` after each extruded segment (7946-7973), no timelapse at the layer change (5604). `process_text`: template processing that keeps the whitespace around `{if}` tags as the C++ parser does | `FARTHEST_TIMELAPSE` (on) | 137,832 → 140,456 (76.41%) | 136,442 → 139,009 (59.31%) | unchanged (the X1C settings do not turn it on) |
| R813 | `;VT0 H-1`: the hotend id GCode.cpp 3235-3236 writes after the filament start G-code | `VT_HOTEND` (on) | 140,456 → 140,457 | 139,009 → 139,010 | 1,512,563 → 1,512,564 |

## Findings

- R809: after placement, straight-wall coordinates agree to the last digit, so
  the frame is exact.
- R810, from the `PP` checksums on Benchy classic: the slices of both engines
  are identical on 299 of 300 layers (layer 76 differs; layer 0 differs in the
  region slices, the elephant-foot contour). After the fix the perimeters are
  identical on 298 of 300 layers and the gap fills on 248 of 254. The fill
  surfaces left by the perimeter step agree on 77 of 204 layers; where they
  differ it is by a few scaled units on single points. After `prepare_infill`
  almost no layer's internal or solid surfaces agree, and the fills agree on
  109 of 289 layers. That is where the next rounds go.
- R810: the new `stSubTop` surface type (the `sub_top_surface_pattern` setting)
  appears on three Benchy layers in the oracle and is not ported.
- R810, open after it (the seam and the wipe order were closed by R811): feed
  rates differ on walls whose geometry agrees. In the start G-code `{max(...)}` is not evaluated,
  `first_layer_print_min` is 144.9995 where the oracle prints 145, `;VT0` lacks
  ` H-1`, and the `M73` progress lines are fewer.
- R812: the 300 inline timelapse blocks are in the same place and have the same
  text as the oracle's. Six of them are followed or preceded by an extrusion
  whose E differs in the fifth decimal (`E.03564` against `E.03563`).
- R812, from `PP` checksums after each step of `prepare_infill` on Benchy
  classic: the typed region slices agree on 286 of 298 layers (internal) and
  168 of 169 (top). The fill surfaces already differ before `prepare_infill`,
  on 127 of 204 layers, by a few scaled units on a few points: on layer 3 four
  corner points are 10 units off (a safety offset applied on one side only, in
  the top-one-wall branch); on layer 29 three points are 1 to 4 units off.
  `discover_vertical_shells` then leaves 2 of 142 layers' internal surfaces
  equal. `combine_infill` and the new `discover_sub_top_surfaces` are not
  ported (the oracle has `stSubTop` surfaces on layers 38, 51 and 126).
- R813, looked at and left open:
  - Gap fill rings come out in the other direction and from another start point
    than the oracle's, so their lines carry other E values (Benchy layer 244).
  - `needs_retraction` did not change between 02.06 and 02.09; the oracle's
    extra wipes inside gap fill follow from the ring order above.
  - Majora, layer 300: the merged slice itself differs (1,386 points against
    1,367) and the region slices are 54 pieces against 47, so the wall
    differences there start before the wall generator. On layers whose slices
    agree, arachne junctions are 1 to 20 scaled units apart.
  - Benchy arachne, layer 100: the same paths, widths and flows as the oracle
    with coordinates 1 to 133 scaled units apart; the arachne input preparation
    is the first suspect.
