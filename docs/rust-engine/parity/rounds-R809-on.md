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
| R814 | Fills, Benchy first. (1) A fill's non-extruding connectors (`ExtrusionPath` with `no_extrusion`, the hops of a monotonic-line top surface) are E-less moves on the linear branch of `_extrude` too (GCode.cpp 7993-7996); they were extruded. (2) `_extrude` does not simplify: with arc fitting off the one simplification is `simplify_path`'s at the print resolution (LayerRegion.cpp 1053-1069), also for sparse infill. (3) Top-one-wall: `fill_clip` is offset on the scaled value through the `float` parameter, and `top_polygons = diff_ex(fill_clip, inner_polygons, ApplySafetyOffset::Yes)` runs on the ClipperLib grid with the safety offset (PerimeterGenerator.cpp 1152, 1176, 1187). (4) `discover_sub_top_surfaces` and `stSubTop` with `sub_top_surface_pattern` (PrintObject.cpp 5839-5920, Fill.cpp 273-274, 556-558, 600). (5) Bridges of one overlap group that run 10 degrees or more apart become separate surfaces and share out their anchors: `separate_bridge_directions`, `partition_bridge_anchors`, the new `merge_bridges` (LayerRegion.cpp 372-637). (6) `polygons_covered_by_width` takes its delta in `float` (ExtrusionEntity.cpp); the fill boundary runs `simplify_polygons` after the Douglas-Peucker pass (`ExPolygon::simplify_p`); `Point::new_scale` truncation at the seam (SeamPlacer.cpp); the `float` radii of `discover_vertical_shells`; `FillGrid` reverses its polylines on odd layers (FillRectilinear.cpp 3127-3129); `intersection_ex(…, ApplySafetyOffset::Yes)` for a fill's no-overlap area (Fill.cpp 837). (7) Four arms that were parked because they lowered the score against the 02.06 oracle now raise it and are on | new, on: `NO_EXTRUSION_LINEAR`, `EXPORT_NO_RESIMPLIFY`, `TOPFILL_CLIP_FAITHFUL`, `SUB_TOP_SURFACES`, `BRIDGE_DIR_SPLIT`, `COVERED_WIDTH_F32`, `TAIL_SIMPLIFY_POLYGONS`, `SEAM_NEW_SCALE_TRUNC`, `GRID_ODD_REVERSE`; turned on: `VSHELL_REG_QUANT`, `FILL_NOOVERLAP_SAFETY`, `ARACHNE_SIMPLIFY_FAITHFUL`, `WIPE_TOWER_FORCE_TAGS`, `LINEWIDTH_BEFORE_SPEED`, `NO_COLLECTION_PRESPEED` | 140,457 → 148,085 (79.65%) | 139,010 → 143,944 (60.82%) | 1,512,564 → 1,524,497 (56.18%) |

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
- R814, what each part moved on Benchy classic / arachne, measured one at a
  time from the R813 state plus the gates of (6): E-less connectors +2,861 /
  +2,014; no second simplification +2,904 / +1,479; sub-top surfaces +76 /
  +256; the top-one-wall clip +303 / 0; the bridge split +20 / -21. The gates
  of (6) together +1,231 / +1,195, of which `FILL_NOOVERLAP_SAFETY` +825 / +786.
- R814, the parked gates, each alone against the new oracle (Benchy classic /
  arachne / Majora): `LINEWIDTH_BEFORE_SPEED` +215 / +14 / -91;
  `NO_COLLECTION_PRESPEED` +203 / +3 / 0; `WIPE_TOWER_FORCE_TAGS` 0 / 0 / +118;
  `ARACHNE_SIMPLIFY_FAITHFUL` 0 / 0 / +3,969. Left off: `GEO_SCALE_COORDT`
  (+335 / +876 / +1,713, but the 1e5 geo-clipper grid makes round offsets take
  minutes in the unit tests; the call sites have to move to the ClipperLib
  helpers instead), `TOWER_PRE_RETRACT` (Majora -14,271), `ISLAND_FLATTEN`
  (-25,803 / -4,104 / -84,842), `WRITER_POS_CLEAR_CPP` (Majora -1,311),
  `RASTER_QUANT` (-74 / +151 / +3), `MMS_PAINT_FMA` and `MMSEG_DIFF_CLIB` (no
  change).
- R814, from the `PP` and `FBIS` probes on Benchy classic after the round: the
  fill surfaces the perimeter step leaves agree on 197 of 204 layers (from 77),
  the top surfaces on 48 of 48 at every step of `prepare_infill`, the sub-top
  surfaces on 3 of 3. Still different: the internal bridges of
  `bridge_over_infill` (11 surfaces against 10, none equal), the bottom bridge
  of layer 2, and the fills on 155 of 289 layers.
- R814, looked at and left: `FillGrid::_layer_angle` is 0 in C++
  (FillRectilinear.hpp 75) and the port turns the grid by 90 degrees on odd
  layers; dropping the turn, or always turning, moves the scores by less than
  the noise between runs (-117 to +118 lines), because both sweeps of a grid
  are 90 degrees apart anyway. Sparse infill is 48% in order: on the layers
  looked at the polylines are the oracle's, started from the other end and
  taken in another order. With the same end point of the previous fill the
  oracle starts at the farther end of the first polyline, so the order comes
  from the chaining of the fill collections, not from the grid.

