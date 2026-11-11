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
| R817 | Fills whose input surfaces already agree. Merged `origin/main` `a1a1fc5` (a test script). (1) `make_fill_lines` takes the bounding box of the rotated source polygon and drops the lines within the scaled spacing plus `SCALED_EPSILON` of its left and right side (FillRectilinear.cpp 2993-3030); the grid kept them. (2) Rectilinear and monotonic fills form their angle in `float`, bridge angle included (FillBase.cpp 204-244), so a rotated end point no longer lands one unit off | new, on: `GRID_FILL_LINES_FAITHFUL`, `RECT_ANGLE_F32` | 149,309 → 149,451 (80.36%) | 145,799 → 145,850 (61.64%) | 1,527,592 → 1,527,621 (56.34%) |
| R818 | Arachne, from a probe on both sides of every `WallToolPaths` call (`WTP`): only 21 of 2,721 calls on Benchy arachne received the outline the oracle's call received. (1) The wall inset is `-float(ext_perimeter_width / 2 - ext_perimeter_spacing / 2)`, two coord_t divisions (PerimeterGenerator.cpp 1506-1508); halving in double moved every outline by half a unit: 1,466 calls agree. (2) `add_infill_contour_for_arachne` in coord_t: integer halves through `float`, `insert` truncated after the overlap percentage, each expolygon simplified once and the rings united once (1463-1484): 2,695 calls agree, the rest are the fills of layers that differ before. (3) `fixSelfIntersections` calls `ClipperLib::SimplifyPolygons` and drops its result (WallToolPaths.cpp 225, 265; clipper.hpp 559-566), and `union_()` is one raw union (ClipperUtils.cpp 721-722); neither moved a line here | new, on: `ARACHNE_INSET_INT`, `ARACHNE_TAIL_SCALED`, `ARACHNE_FIXSELF_DISCARD`, `ARACHNE_UNION_FLAT` | unchanged (149,451) | 145,850 → 158,538 (67.00%) | 1,527,621 → 1,796,767 (66.27%) |
| R819 | The order the islands of a layer are printed in. `process_layer` puts every perimeter and fill collection into `islands[i]`, `i` being the lslice its first point lies in, and prints that vector front to back (GCode.cpp 5437-5527, 5922). The port printed the islands in the order they first appeared among the perimeter entities, which is the order the perimeter generator visited its surfaces in | new, on: `ISLAND_LSLICE_ORDER` | 149,451 → 164,807 (88.62%) | 158,538 → 204,849 (86.58%) | 1,796,767 → 1,847,143 (68.13%) |
| R820 | The order of the fills inside an island. (1) `Region::append` takes the entities out of every sortable fill collection (GCode.cpp 9125-9133), so `extrude_infill` chains the single paths of a region, not its collections (6983-7009). The port had this parked as `ISLAND_FLATTEN` because it scored far below zero: a path taken out of its collection was printed without its feature tag, width and speed. (2) Each thin fill sits in `fills` in a collection of its own (end of `make_fills`), so gap fills are chained with the other fills instead of following them. (3) The paths of a grid may not be reversed by the chain (`params.can_reverse = false`, Fill.cpp 832-833; FillBase.cpp 172-175) | on: `ISLAND_FLATTEN` (was parked), `FILL_GRID_NO_REVERSE` (new) | 164,807 → 171,768 (92.33%) | 204,849 → 212,289 (89.65%) | 1,847,143 → 1,847,625 (68.24%) |
| R821 | No engine change. The source map: `scripts/parity/file_map.py` writes `docs/rust-engine/FILE_MAP.md`, every libslic3r unit against the Rust file of the same name, function by function. It replaces the hand-kept counts of `PORT_LEDGER.md` | none | unchanged (171,768) | unchanged (212,289) | unchanged (1,847,625) |
| R822 | From the map: `GCode/CoolingBuffer` had no Rust file; its five functions (`calculate_layer_slowdown`, `new_feedrate_to_reach_time_stretch` and the three `extruder_range_slow_down_*`) move out of `gcode/cooling.rs` into `gcode/cooling_buffer.rs` under their C++ names, output unchanged. From the G-code: wipe into infill. `WipingExtrusions::mark_wiping_extrusions` and `ensure_perimeters_infills_order` (ToolOrdering.cpp 3906-4117), driven as `Print::_make_wipe_tower` drives them (Print.cpp 4266-4329): the sparse infill marked for a tool change is taken off the volume purged on the tower, a region asks for its filament only for what cannot be overridden (ToolOrdering.cpp 939-1017), and the overridden fills print first after the change, closed by `; PURGING FINISHED` (GCode.cpp 5826-5828, 9046-9101) | on: `WIPE_INTO_INFILL` (new) | unchanged (171,768) | unchanged (212,289) | 1,847,625 → 1,954,872 (71.90%) |
| R823 | The Elephant foot compensation of a first layer with several regions. `slice_volumes` compensates after `apply_mm_segmentation` (PrintObjectSlice.cpp 2202), so a painted first layer takes the multi-region branch (2331-2378): the merged layer, closed by 1.5 × `slice_closing_radius`, is compensated once, every region is cut by the result, and the layer keeps the uncompensated outline as its `lslices`, chained by `chain_points` (2391-2408). The port compensated each region's slices by themselves in `make_perimeters`, after the untyped slices were backed up | on: `L0_EFC_MULTI_REGION` (new) | unchanged (171,768) | unchanged (212,289) | 1,954,872 → 1,959,722 (72.07%) |
| R824 | From the map: the exporter the engine writes G-code with moves from `gcode/exporter.rs` into `g_code.rs`, the file named after GCode.cpp (`crate::gcode::exporter` stays as a name for it); output unchanged. From the G-code: (1) `unscale<float>(v)` is `float(v) * float(SCALING_FACTOR)` (libslic3r.h 112), and the float nearest 1e-5 is 2.5e-8 below it, so the width of a variable-width path (VariableWidth.cpp 66, 136, 203; FillFloatingConcentric.cpp 72) is one float step lower than a division by 1e5 gives on about a third of the paths, which moves the volumetric speed cap by 0.001 mm/min. (2) The vertical-shell cache over all regions of a multi-material print (PrintObject.cpp 2351-2386) computes its offsets in `float` over the scaled integer widths, and the perimeter shadow is one `offset2` of the lslices. (3) `apply_mm_segmentation` intersects the region's slices (subject) with the segmentation (clip), PrintObjectSlice.cpp 1921; the other way round Clipper rounds the crossings differently | on: `UNSCALE_FLOAT`, `VSHELL_ALL_REGIONS_F32`, `MMSEG_STOLEN_SUBJECT` (new) | 171,768 → 171,788 (92.34%) | 212,289 → 213,070 (89.98%) | 1,959,722 → 1,994,413 (73.35%) |
| R825 | From the map: `gcode/generator.rs` (the G-code text, its header and config block) moves into `g_code.rs`; output unchanged. From the G-code: (1) the nearest seam is picked from the nozzle position, `pick_nearest_seam_point_index(.., unscaled<float>(last_pos))` (SeamPlacer.cpp 1591); the port took the loop's first vertex. (2) The end of `append_tcr` (GCode.cpp 1355-1373): the position after a tower visit is the tower's end position, the wipe path is the tower's own, and the retract there wipes along it and lifts at once at `prime_tower_lift_speed`; the port kept the last object position and wiped toward the object. (3) `extrude_loop` handed the seam placer the position times 1e6 instead of scaled | on: `SEAM_NEAREST_LAST_POS`, `TOWER_LEAVE_CPP`, `SEAM_LAST_POS_SCALED` (new) | unchanged (171,788) | unchanged (213,070) | 1,994,413 → 2,066,495 (75.89%) |
| R826 | The floating vertical shell. `slicer_cli` compiles the override layer's `Fill/FillFloatingConcentric.cpp`, not the engine's file. The override keeps the points of the self-intersection grid alive, so `detect_floating_line` runs; the port forced the detection off (`FVS_FORCE_NO_DETECT`, R801) to mirror the engine's file reading freed memory. With the override followed: `rebase_at` writes the closure flag at index n-1, and a finished path is flagged cfFloatingVerticalShell by the run it closes. In the exporter a flagged path takes the bridge speed and is bracketed by `; Slow Down Start` / `; Slow Down End` (`extrude_path` and `_extrude`, GCode.cpp 6930-6931, 7738-7741, 7874-7875, 8083-8085). From the map: `file_map.py` reads the override layer's checked-in files in place of the engine's | on: `FFC_OVERRIDE`, `FVS_SEPARATE_SPEED` (new) | 171,788 → 174,014 (93.36%) | 213,070 → 218,088 (91.87%) | 2,066,495 → 2,251,712 (81.17%) |
| R827 | The gyroid sparse infill. (1) `params.density = float(0.01 * surface_fill.params.density)` (Fill.cpp 944): the product is taken in double and rounded to float once; multiplied in float, 10 % gives 0.099999994 where C++ has 0.1f. (2) `FillGyroid::_fill_surface_single` (FillGyroid.cpp 169, 175-176): the grid module `Point(2*M_PI*distance, ..)` is rounded by `lrint` (Point.hpp 179), and `ceil(bb.size()(0) / distance) + 1.` divides two `coord_t`, so the `ceil` does nothing; the port truncated the module and divided in double, which made one wave more per axis on a grid shifted by a few units. From the map: `gcode/smooth_speed.rs` (the smooth-speed helpers of GCode.cpp) moves into `g_code.rs`, output unchanged | on: `FILL_DENSITY_DOUBLE`, `GYROID_BBOX_CPP` (new) | unchanged (174,014) | unchanged (218,088) | 2,251,712 → 2,308,349 (83.21%) |
| R828 | The cooling markers of a path in a slow-down block. (1) `GCodeEditor` removes `;_EXTRUDE_SET_SPEED`, `;_EXTERNAL_PERIMETER` and `;_WIPE` only from a line of type ADJUSTABLE, EXTERNAL_PERIMETER or WIPE (GCodeEditor.cpp 672-684); any other line with an F word keeps the rest of the source line. Between `; Slow Down Start` and `; Slow Down End` a line is not adjustable (216, 298-301), so its marker stays, with the F word when the feedrate changes and alone when it does not. (2) `GCodeWriter::set_speed` puts the marker right after the F word. (3) `; Slow Down Start` precedes the role tag of the path (GCode.cpp 7874-7875) | on: `EDITOR_KEEP_FOREIGN_COMMENT`, `SET_SPEED_MARKER_NOSPACE`, `SLOW_DOWN_BEFORE_ROLE_TAG` (new) | 174,014 → 174,107 (93.40%) | 218,088 → 218,262 (91.92%) | 2,308,349 → 2,329,271 (83.42%) |
| R829 | The tower wall under smooth timelapse. `generate_new` draws the outer wall as a result of its own, `only_generate_out_wall(true)` (WipeTower.cpp 7818, 7854-7856, 8215-8280), puts it in front of the layer's results (8072-8073), and the finish of the layer draws no wall (7821); `WipeTowerIntegration::tool_change` prints it before anything else of the layer (GCode.cpp 1508-1512). The wipe path of that block is taken from the wall (`add_wipe_path`, WipeTower.cpp 1368-1379). After a tower block the nominal Z is the tower layer's, so the lift on leaving targets the right height. From the map: `WipeTower::only_generate_out_wall` now exists in `gcode/wipe_tower.rs` | on: `TOWER_TIMELAPSE_WALL` (new) | unchanged (174,107) | unchanged (218,262) | 2,329,271 → 2,343,292 (83.63%) |
| R830 | The seam placer's occlusion mesh. `compute_global_occlusion` (SeamPlacer.cpp 577-606) puts every MODEL_PART and NEGATIVE_VOLUME through its own matrix (`its_transform`, admesh/stl.h 372-380), merges parts and negative volumes into two sets, decimates each (`its_short_edge_collpase`), appends the negative set and applies `trafo_centered()`, whose Z is `trafo()`'s own. The port decimated the placed mesh of the parts in bed coordinates and had no negative volumes in it. The 3MF loader now hands the volumes over one by one | on: `SEAM_OCCL_VOLUMES` (new) | unchanged (174,107) | unchanged (218,262) | 2,343,292 → 2,361,380 (84.28%) |
| R831 | Painted seams. The 3MF loader reads `paint_seam` (bbs_3mf.cpp 296) into the same storage as `ModelVolume::seam_facets`. `gather_enforcers_blockers` (SeamPlacer.cpp 645-668): the ENFORCER and BLOCKER facets of the volume, taken from the triangle selector in the volume's own coordinates, go through `trafo_centered() * get_matrix()` and into two AABB trees. `GlobalModelInfo::is_enforced` and `is_blocked` (287-299) ask `is_any_triangle_in_radius`; they returned false before. `process_perimeter_polygon` (467-485) and `SeamComparator` were already written against them. From the map: the two placeholders of `gcode/seam_placer.rs` are real functions and `gather_enforcers_blockers` exists | on: `SEAM_PAINTING` (new) | unchanged (174,107) | unchanged (218,262) | 2,361,380 → 2,558,898 (91.21%) |
| R832 | (1) `WipingExtrusions::ensure_perimeters_infills_order` (ToolOrdering.cpp 4093-4094) calls `lt.is_extruder_order(lt.wall_filament(region), last_nonsoluble_extruder \|\| ! lt.has_extruder(..))`: the `\|\|` stands inside the argument list, so the second argument is 0 or 1. An infill of the last tool's own region is thereby overridden to that tool and prints before its walls. The port had the two conditions apart, as the comments beside them read. (2) `generateLocalMaximaSingleBeads` (SkeletalTrapezoidation.cpp 2131-2132): the six points of the circle go through `Point(double, double)`, which rounds (Point.hpp 179); the port truncated | on: `WIPE_ORDER_PAREN_CPP`, `SINGLE_BEAD_CIRCLE_LRINT` (new) | 174,107 → 174,109 (93.41%) | 218,262 → 218,276 (91.92%) | 2,558,898 → 2,564,965 (91.38%) |
| R833 | Lines the in-order score does not count. (1) `sprintf(buf, "; Z_HEIGHT: %g\n", print_z)` (GCode.cpp 5002): six significant digits; the port printed the double in full (`20.840000000000014`). (2) The tool-change block: the placeholder parser removes an `{if}` tag and nothing else, so a line that held only a tag stays as an empty line; the result gets `check_add_eol` and the `; FilamentChangeGCodeStart` / `; FilamentChangeGCodeEnd` markers (GCode.cpp 1122-1128); the filament-end text keeps its trailing blank and its empty lines. New: `scripts/parity/exact_lines.py`, every body line as written | on: `Z_HEIGHT_G6`, `TOOLCHANGE_TEMPLATE_TEXT` (new) | unchanged (174,109) | unchanged (218,276) | 2,564,965 → 2,570,411 (91.39%) |
| R834 | (1) Retraction inside infill. `GCode::needs_retraction` (GCode.cpp 8465-8486) skips the retraction of a travel only when `travel_inside_internal_regions_no_wall_crossing` holds, and that function has a third condition (RetractWhenCrossingPerimeters.cpp 113-198): the travel must not run within `wall_proximity_distance` (the inner wall's line width, or the nozzle diameter) of a wall for `retraction_minimum_travel` or longer. The port had the first two conditions only, so travels along a wall went without retraction and wipe. From the map: `travel_near_perimeters_too_long` was the one function missing in `gcode/retract_when_crossing_perimeters.rs`. (2) `; OBJECT_ID` is written for every instance in every extruder pass (GCode.cpp 5843), before the wipe pass and again after `; PURGING FINISHED`; the port wrote it once a layer. (3) `m_writer.reset_last_acceleration()` before and after a tower block (GCode.cpp 743, 1377), so the acceleration is written again after the tower. (4) The id of the instance of a job loaded from one STL is 28 in the oracle, 13 in the port. 28 is read off the oracle's output and not derived: the id is a process-wide counter of created objects, which the port does not keep | on: `RETRACT_NEAR_WALL`, `OBJECT_ID_PER_PASS`, `TOWER_RESET_ACCEL`, `STL_OBJECT_ID_28` (new) | 174,109 → 180,394 (94.27%) | 218,276 → 226,152 (92.95%) | 2,570,411 → 2,593,339 (91.42%) |

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
- R817: of the 119 Benchy classic layers whose fills differed after R816, 107
  had identical grouped fill inputs. After the round the fills agree on 185 of
  289 layers. What is left on the layers looked at is the two fills that go
  through the arachne wall generator: the floating vertical shell
  (`FillFloatingConcentric`, 8 paths against 14 with other widths on layer 20)
  and the narrow internal solid fill (`FillConcentricInternal`, widths 0.66176
  against 0.66177, a ring started at the mirrored point on layer 22). They
  share the cause with the arachne walls of the other Benchy job.
- R818: with the same outline the port's `WallToolPaths` returns the oracle's
  toolpaths: of 2,680 calls with equal input, 2,694 prepared outlines agree to
  the point and 2,412 final toolpath sets agree to the unit. The other 275
  have the same lines and junction counts, with junction sums one or two
  units off (10 to 20 nm), which the G-code's three decimals do not show.
  The earlier finding that "junctions are 1 to 130 units apart" was the input,
  not the wall generator.
- R818: content match is now 88.5% on Benchy arachne and 77.1% on Majora, the
  in-order score 67.0% and 66.3%: a quarter of the matching wall lines come in
  another order. The order in which arachne walls are emitted is the next
  thing to look at.
- R819: with the islands in lslice order the in-order score is within two
  points of the content score on both Benchy jobs (88.6% of 90.7%, 86.6% of
  88.7%); walls are 97% in order on Benchy classic. What still comes in
  another order is the fills of one island: on Benchy classic layer 101 the
  oracle prints floating vertical shell, sparse infill, gap infill where the
  port prints sparse, gap, floating shell (`extrude_infill`,
  GCode.cpp 6983-7009).
- R819, Majora: content 77.1%, in order 68.1%. Floating vertical shell (25%
  in order), sparse infill (35%), top surface (50%) and bridges (42%) are the
  weak features; outer and inner walls are at 73% and 76%.
- R820, Benchy classic by feature, in order: walls 97%, sparse infill 91%
  (from 48%), solid infill 91%, bridges 92%, top surface 92%, gap infill 83%,
  floating vertical shell 75%. The floating shell is the one fill that still
  starts elsewhere: on layer 101 its first run begins at the mirrored point.
- R820: Majora gains 482 lines in order and loses 1,704 in content; its
  floating vertical shell (25% in order) and sparse infill (35%) did not move
  with Benchy's, so they differ for another reason.
- R821, the first source map: 310 C++ units, 276 with a Rust file of the same name, 34 without (6 of them
  on the FFF path: `Fill/FillRadialZigZag`, `Fill/FillConformalChart`, `Fill/FillConformal`,
  `ColorDecomposeRecipe`, `GCode/CoolingBuffer`, `GCode/BedExcludeChecker`). Of 9,059 named C++ functions
  3,662 have a Rust function of the same name in the paired file (3,084 estimated faithful, 357 partial, 190
  divergent, 31 stubs); of the 4,724 that do not, 1,633 exist under that name in another Rust file. 51 Rust
  files are named after no C++ file, among them the live G-code writer (`gcode/exporter.rs`,
  `gcode/writer.rs`, `gcode/cooling.rs`, `gcode/generator.rs`), and 7 names are held by two Rust files.
- R822, Majora: prime tower 83.6% → 93.5% in order, sparse infill 35% → 51%, content 77.2% → 80.1%. The
  number of `; PURGING FINISHED` lines agrees on every layer both engines print. The port takes wipe into
  infill for one object, with `flush_into_infill` and without `flush_into_objects`; wiping into supports
  and into whole objects, `flush_multiplier` and `grab_length` (1 and 0 on Majora) are not read yet.
- R822, found and not fixed: the tool order of the first two Majora layers differs (oracle T0, T4, T2, T3
  on the first layer; the port ends the layer with T0), every later layer has the oracle's order.
- R822, the source map against R821: units with a Rust file of the same name 276 → 277, without 34 → 33;
  functions matched 3,662 → 3,671 (faithful 3,084 → 3,089, partial 357 → 361), missing in the pair
  4,724 → 4,720, in a unit with no Rust file 673 → 668, Rust only 4,395 → 4,399.
- R823: the first-layer tool order of Majora is the oracle's (T0, T4, T2, T3). It is read from the
  compensated `raw_slices` (ToolOrdering.cpp 642-728); uncompensated, a 0.27 mm piece of the first filament
  survived the shrink and put that filament last. On layer 0 the top surfaces now agree with the oracle
  exactly, the region slices are 792 pieces against 788 (815 before), `lslices` 1,927 points against 1,928.
- R823, found and not fixed: on Majora layers 1 and 2 the `lslices` differ before any compensation (same
  point count with other sums on layer 1; 6 outlines against 7 on layer 2), so slicing or the segmentation.
- R823, the source map: headline unchanged against R822; the round's code is part of `PrintObject::slice_volumes`
  (2,266 tokens, listed as missing), which the port keeps inside `PrintObject::slice`.
  `GCode/BedExcludeChecker` (no Rust file) was read: it checks a finished `GCodeProcessorResult` against the
  exclude areas and writes no G-code, so it waits behind the files that do.
- R824, the oracle is not reproducible in the multi-material segmentation: over six runs of `slicer_cli` the
  output of `multi_material_segmentation_by_painting` differs on 83 of Majora's 656 layers and the region
  slices after `apply_mm_segmentation` on 63. What R823 listed as a difference on layers 1 and 2 is this:
  the port equals some runs and not others. The kept oracle G-code is one sample, so these layers bound
  the Majora score. Against the six runs the port's region slices equal none on 4 layers (175, 300, 493
  of the stable ones), down from 49 before the operands of the intersection were put in the C++ order.
- R824, Majora stage by stage after the round (layers equal to one probed oracle run): region slices 618
  of 656, fill surfaces from the perimeter generator 611 of 651, internal fill surfaces after
  `discover_vertical_shells` 585 of 635 (154 before), after `bridge_over_infill` 523 of 631 (151 before).
  The extrusions themselves still differ on most layers: walls on 576 of 656, fills on 634 of 650.
- R824, Majora walls by line: 86% of the wall extrusion lines of a layer are in the oracle, evenly over
  the layers. Of the standalone `G1 F` lines 220,019 of 318,505 matched before `UNSCALE_FLOAT`. What is
  left in walls is mostly structure: where an open wall starts, and the wipe at its end (the oracle
  retracts in one move, the port in several along the path).
- R824, by feature in order on Majora: outer wall 74.2%, inner wall 77.6%, floating vertical shell 32.1%,
  sparse infill 51.4%, prime tower 93.6%.
- R824, the source map against R823: Rust files named after no C++ file 51 → 50 (`gcode/exporter.rs` is
  gone); functions matched 3,671 → 3,684 (faithful 3,089 → 3,090, partial 361 → 365, divergent 190 → 198:
  the exporter's functions are now measured against `GCode.cpp`, and several are free functions that cover
  part of a method); missing in the pair 4,720 → 4,707; Rust only 4,399 → 4,438. The five exporter tests
  that fail are the same five, under `g_code::exporter_tests`.
- R825, Majora by feature in order: outer wall 76.3% (74.2), inner wall 77.7%, sparse infill 68.8% (51.4),
  floating vertical shell 32.3%, prime tower 94.6% (93.6). Extrusion runs that start at the oracle's point:
  outer wall 72.5% (56.0), inner wall 73.0% (53.3), sparse infill 83%, floating vertical shell 56.5%.
- R825, found and not fixed: on 353 layers the first wall after a tower visit still starts elsewhere,
  because the nozzle leaves the tower from another place. `WipeTower::generate_new` (WipeTower.cpp 7760)
  puts the finish of a tower layer (`finish_layer_new`, merged by `merge_tcr`) into one of the layer's
  tool-change results, which `WipeTowerIntegration::tool_change` prints before the objects of that tool
  (GCode.cpp 1489-1522); the port's tower is the earlier `generate`/`finish_layer` shape and prints the
  finish at the end of the layer. The map lists `generate_new`, `finish_layer_new`, `tool_change_new`,
  `merge_tcr` and `only_generate_out_wall` as missing.
- R825, the source map against R824: Rust files named after no C++ file 50 → 49 (`gcode/generator.rs`);
  functions matched 3,684 → 3,685; divergent 198 → 199; missing in the pair 4,707 → 4,706; Rust only
  4,438 → 4,478 (the header and config-block helpers now counted against `GCode.cpp`).
- R826, floating vertical shell in order: Majora 32.3% → 79.9%, Benchy classic 74.7% → 95.5%, Benchy
  arachne 66.6% → 93.7%. `; Slow Down Start` lines: Majora 22,630 against 22,753, Benchy classic 82 against
  83, Benchy arachne 164 against 165.
- R826, Majora by feature in order: outer wall 76.4%, inner wall 77.7%, prime tower 94.7%, floating
  vertical shell 79.9%, sparse infill 69.0%, internal solid infill 97.4%. The walls are now the largest
  part of what does not match; the tower's finish-layer placement (R825 note) is the known cause of a part
  of it.
- R826, the source map against R825: with `Fill/FillFloatingConcentric.cpp`, `LogSink.cpp` and `Thread.cpp`
  read from the override layer, functions matched 3,685 → 3,682 (faithful 3,090 → 3,088, partial 365 →
  364), missing in the pair 4,706 → 4,702, Rust only 4,478 → 4,482. The other headline rows are unchanged.
- R827, Majora sparse infill in order 69.0% → 92.7%. Stage by stage against one probed oracle run, of the
  port's 5,935 gyroid calls: rotated outline equal 5,378, waves 5,903 (0 before), clipped 5,754, connected
  5,496 of 5,814.
- R827, what is left on Majora by feature in order: outer wall 76.4%, inner wall 77.7%, top surface 66.2%,
  bridge 54.0%, bottom surface 17.0% (1,362 lines), floating vertical shell 80%, prime tower 94.7%.
- R827, the source map against R826: Rust files named after no C++ file 49 → 48 (`gcode/smooth_speed.rs`);
  functions matched 3,682 → 3,689 (faithful 3,088 → 3,094, partial 364 → 365); missing in the pair
  4,702 → 4,695; Rust only 4,482 → 4,488.
- R828, `G1 F3000;_EXTRUDE_SET_SPEED` lines on Majora: 4,370 against the oracle's 4,819; the marker alone
  18,246 against 17,934. No separate item from the source map this round, and its headline is unchanged.
- R828, seen and left: on Benchy arachne the outer wall's standalone `G1 F` lines match on 5,919 of 12,781.
  The first of a ramp is the layer's cooling slowdown (`F8570` against `F8564`), so the estimated layer
  time differs by about 0.07%; the steps of the ramp (`smooth_speed_discontinuity_area`) differ in the
  third decimal.
- R829, prime tower in order 94.8% → 95.7%. Not yet as the oracle: the wall's gaps at the skip points (the
  oracle breaks the right edge where the tool changes enter, the port splits the bottom edge), the start
  corner of the wall, and the finish blocks, which `generate_new` merges into a tool-change result
  (`finish_block`, `merge_tcr`) and the port prints at the end of the layer.
- R829, why 27% of Majora's wall runs still start elsewhere: for the nearest seam the candidates of a
  perimeter agree with the oracle in position, angle and embedded distance, and differ in visibility
  (0.614 against 0.687 on one vertex), so two similar corners rank the other way. The oracle's visibility
  is the same from run to run. Stage by stage in `compute_global_occlusion`: both start from 799,380
  triangles; after `its_short_edge_collpase` the oracle has 15,958 and the port 15,952 (the port decimates
  in bed coordinates, the oracle in the volume's own); the oracle then merges 10,812 triangles of negative
  volumes (26,770 in all, lowest z -0.05) and the port none.
- R829, the source map against R828: functions matched 3,689 → 3,690 (partial 365 → 366), missing in the
  pair 4,695 → 4,694, Rust only 4,488 → 4,489.
- R830, checked with the same checksum lines in both engines: the vertices of the set that is sampled
  (26,770 triangles) and the sum of the 30,000 sample visibilities (21227.115779) are now the oracle's.
  Wall runs that start at the oracle's point: outer 72.5% → 79.5%, inner 73.1% → 78.8%.
- R830, found and not fixed: Majora's 3MF paints seams (`paint_seam` on 28,792 triangles). In the oracle
  `process_perimeter_polygon` oversamples a perimeter inside an enforcer and `SeamComparator` ranks enforced
  and blocked points first (SeamPlacer.cpp 467-485, 645-668, 287-299); one perimeter has 71 candidates there
  and 55 in the port. The port's loader drops the attribute and `is_enforced` / `is_blocked` return false.
- R830, the source map against R829: matched 3,690 (unchanged), faithful 3,094 → 3,093, divergent 199 → 200
  (`compute_global_occlusion` now carries both arms), Rust only 4,489 → 4,491.
- R831, Majora by feature in order: outer wall 77.6% → 89.5%, inner wall 78.9% → 93.9%, floating vertical
  shell 82.5%, sparse infill 94.0%, internal solid infill 97.9%, prime tower 95.9%, overhang wall 83.9%,
  top surface 68.4%, bridge 55.0%. Wall runs starting at the oracle's point: outer 88.8%, inner 90.4%.
  The model has 53,179 enforcer triangles after the selector's splits and no blockers; the perimeter that
  had 55 seam candidates against the oracle's 71 has 71 at the same positions.
- R831, the AABB tree of the crate works in double where the oracle's enforcer tree works in float; the
  float vertices and the float radius squared are handed over as they are. Not seen to matter.
- R831, the source map against R830: functions matched 3,690 → 3,691 (faithful 3,093 → 3,095, partial
  366 → 367, divergent 200 → 198), missing in the pair 4,694 → 4,693, Rust only 4,491 → 4,492.
- R832, arachne: with the same input, `WallToolPaths` toolpaths after generation equal the oracle's on
  40,642 of 40,732 calls (38,094 before the circle was rounded); 49,596 of about 50,150 calls have the same
  input.
- R832, the nearest seam: where the nozzle position at `place_seam` is the oracle's, the port picks the same
  point in 25,626 cases and another in 10. In 2,412 of the oracle's 28,238 calls the nozzle is elsewhere,
  which is what is left of the wall start points (outer 88.8%, inner 90.4% at the oracle's point).
- R832, Majora sparse infill in order 94.0% → 95.4%; 52 layers have the oracle's sequence of extrusion runs
  from first to last (29 before).
- R832, seen and left: the steps of a speed ramp (`split_and_mapping_speed`) differ in the third decimal of
  F where a point of the split overhang path is one unit off; neighbours then differ by +d and -d.
- R832, the source map: headline unchanged against R831.
- R833, lines of the oracle's body the port has verbatim (`exact_lines.py`), before → after: Benchy classic
  181,421 → 181,709 of 196,495; Benchy arachne 224,205 → 224,493 of 248,190; Majora 2,634,431 → 2,706,453
  of 3,024,752. `; Z_HEIGHT` differed on 288 of 300 Benchy layers and 641 of 656 Majora layers, now on none.
- R833, what is missing verbatim on Majora, by shape: `; LINE_WIDTH` 20,308 (22,776 the other way), empty
  lines 12,103, `M<n> S<n>` 6,847, `; OBJECT_ID` 6,063, `M73 P R` 5,802, `; WIPE_START` / `; WIPE_END` 2,285
  each, `; set nozzle temperature` 655. On Benchy classic: `; OBJECT_ID` has another value on all 300
  layers (28 against 13), 440 wipe marker pairs and 131 `M73` lines are missing.
- R833, found: the oracle writes `; OBJECT_ID` for every instance in every extruder pass
  (GCode.cpp 5843), the port once a layer; and between the end of the tool-change block and the tower's
  first move it travels along `generate_path_to_wipe_tower` before the filament-start text, which the
  port writes first.
- R833, the source map: headline unchanged.
- R834, lines of the oracle's body the port has verbatim, before → after: Benchy classic 181,709 → 187,738
  of 196,495 (95.5%); Benchy arachne 224,493 → 232,419 of 248,190 (93.6%); Majora 2,706,453 → 2,730,277 of
  3,024,752 (90.3%).
- R834, wipe marker pairs, port against oracle, before → after: Benchy classic 2,111 → 2,667 against 2,551;
  Benchy arachne 2,041 → 2,571 against 2,530; Majora 43,339 → 44,799 against 45,624. By feature on Benchy
  classic after the round: gap infill 1,223 against 1,207 (834 before), sparse infill 161 and floating
  vertical shell 91 on both sides, outer wall 707 against 665, inner wall 117 against 87. On Majora the port
  is short on overhang wall (6 against 141), floating vertical shell (3,677 against 3,935) and internal solid
  infill (4,982 against 5,197), and over on top surface (373 against 298).
- R834, the source map against R833: functions matched 3,691 → 3,692 (faithful 3,095 → 3,096), missing in
  the pair 4,693 → 4,692, Rust only 4,492 → 4,494. `gcode/retract_when_crossing_perimeters.rs` has every
  function of its C++ file.
