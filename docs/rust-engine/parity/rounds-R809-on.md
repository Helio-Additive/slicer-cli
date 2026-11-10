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
| R815 | Bridges, from the probes of `expand_bridges_detect_orientations` on Benchy layer 2 until every stage agrees. (1) The floating edges of a bridge: closed polylines, the anchors grown as raw polygons with the miter join, one open-path ClipperLib difference (LayerRegion.cpp 353; ClipperUtils.cpp 841-849, 911). (2) `Line::normal()` is `(dy, -dx)` (Line.hpp 180), and the direction candidates are walked in the order of the `std::unordered_map` C++ keeps them in (BridgeDetector.hpp 89-113): two edges with opposite normals cost the same, and the order picks the sign. (3) `compute_principal_components` with the fused multiply-adds clang emits for its scalar `float` expressions on arm64 (PrincipalComponents2D.cpp). (4) `RegionExpansionParameters::build` in its own `float` scaled values, handed to the ClipperLib calls as they are (RegionExpansion.cpp 32-79). (5) `polylines_merge` swaps instead of reversing both and keeps the merge point twice (Polyline.hpp 236-246). (6) `closing_ex(Polygons)` over the raw polygons (ClipperUtils.cpp 598-602). (7) `diff_ex`, `intersection_ex` and `offset_ex(ExPolygons)` rebuild their expolygons with one union into a PolyTree (ClipperUtils.cpp 552-557, 641-654); the port ran `union_ex`, two unions, behind every one of them. (8) The pattern of a solid surface in the order of Fill.cpp 268-281: a bridge is rectilinear unless the top pattern is monotonic; it took the bottom pattern | new, on: `BRIDGE_FLOATING_CLIB`, `BRIDGE_DIR_HASH_ORDER`, `PCA_FP_CONTRACT`, `REGION_EXPANSION_PARAMS_F32`, `POLYLINES_MERGE_FAITHFUL`, `BRIDGE_CLOSING_FAITHFUL`, `CLIB_POLYTREE_ONE_PASS`, `FILL_PATTERN_CHAIN`; turned on: `BRIDGE_NORMAL_CPP` | 148,085 → 148,122 (79.66%) | 143,944 → 143,955 (60.81%) | 1,524,497 → 1,525,490 (56.27%) |
| R816 | `bridge_over_infill`, which did not change between 02.06 and 02.09 but ran on the 1 um geo backend. (1) The `Polygons` overloads it is written in — `diff`, `diff_ex`, `intersection`, `intersection_ex`, `expand`, `shrink`, `opening`, `closing`, `union_` — on ClipperLib over the raw paths (ClipperUtils.cpp 413-415, 592-613, 669-680, 741-742, 802). (2) Every delta from the truncated `scaled_spacing()`, as PrintObject.cpp 2794 and 3376 take it; `0.3 * flow.spacing()` stays the unscaled number C++ passes (3458). (3) `to_polylines(Polygons)` as closed rings for the boundary lines (3456-3460); `closing_ex` and raw `diff` in `gather_areas_w_depth` (3073-3077); the apply step's `diff_ex` and `intersection_ex` (3560-3587). (4) The anchoring lines come from the filler `make_fills` uses (Fill.cpp 912-971): a grid goes through both sweeps and one `connect_infill`. (5) The grid's sweep angles are formed in `float` and sit at +90 and +180 degrees on every layer: `FillGrid::_layer_angle` is 0 (FillBase.cpp 204-244; FillRectilinear.hpp 75; FillRectilinear.cpp 3050-3054). (6) `bridge_extrusion_spacing` adds the double 0.05 to a `float` and rounds back (Flow.cpp 229-232): 0.45000002 for a 0.4 nozzle, 45000 scaled where the port had 44999 | new, on: `POLYGONS_OPS_CLIB`, `ANCHOR_GRID_FAITHFUL`, `GRID_ANGLE_F32`, `BRIDGE_SPACING_F32` | 148,122 → 149,309 (80.28%) | 143,955 → 145,799 (61.61%) | 1,525,490 → 1,527,592 (56.34%) |

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
- R815: on Benchy layer 2 the wave seeds, the expansions, the bridge
  directions, the direction groups, every step of `partition_bridge_anchors`
  and the 104 bridge surfaces that come out are now the oracle's, and the
  bottom bridges agree on 4 of 4 layers after `process_external_surfaces`.
  Bridge lines in order: Benchy classic 110 → 383 of 1,587, arachne 206 → 500,
  Majora 2,351 → 3,574. The content match rises by 552 / 576 / 1,173 lines;
  the in-order score rises less because the fills printed after a bridge now
  start from another point (`FILL_PATTERN_CHAIN` alone: -56 / -77 in order,
  +388 / +414 content on the two Benchy jobs).
- R815: two results of the oracle depend on how it was built, and the port
  follows the build the scores are taken against (macOS, arm64, libc++). The
  sign of a bridge direction with tied costs follows the iteration order of
  libc++'s `std::unordered_map<double, …>`; libstdc++ orders it differently.
  The principal components of a fully anchored bridge are computed with fused
  multiply-adds; an x86-64 build without FMA rounds each product. Both are
  behind gates (`BRIDGE_DIR_HASH_ORDER`, `PCA_FP_CONTRACT`), so a Linux or
  x86-64 oracle can be matched by turning them off or by adding its arm.
- R815, open: `bridge_over_infill` (internal bridges on 11 layers against 10,
  none equal), the fill surfaces of layers 1, 71, 74, 99, 117, 125 (they follow
  the gap fills, which differ on layers 0, 1, 71, 72, 117, 196), and the fills,
  identical on 135 of 289 layers.
- R816, from the `BOI` probe on Benchy classic: the 20 bridge candidates, the
  anchoring lines of all 17 layers, and the internal bridges of all 11 layers
  are the oracle's. `prepare_infill` as a whole no longer adds a difference:
  what differs after it (internal surfaces on layers 4, 74, 93, solid ones on
  1, 4, 71, 93, 99, 117, 125) already differs before it, in the fill surfaces
  the perimeter step leaves and in `discover_vertical_shells`. The fills are
  identical on 173 of 289 layers (from 135). Bridge lines in order: 383 → 869
  of 1,604 on Benchy classic.
- R816: the grid layer angle that R814 left as noise was half of a real
  difference. With the angles also formed in `float` the anchoring lines agree
  to the unit; in double one point in twenty lands one unit off.
- R816, open: two layers' deep-infill areas (33, 46) are 70 to 90 units off on
  one vertex and one bridging angle differs in the tenth digit; sparse infill
  is still only 47% in order although its lines agree, because the fills
  printed before it end elsewhere.

