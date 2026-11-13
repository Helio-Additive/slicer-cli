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
| R835 | The travel to the prime tower after the change-filament template. `append_tcr` (GCode.cpp:1131-1144, 1177-1253, 1294): the template leaves the position unknown and its last Z current; with `prime_tower_skip_points` the travel goes from the template's last position along `generate_path_to_wipe_tower` (623-739, ported to `g_code.rs`) around the tower box grown by 2 mm, each point a `GCode::travel_to` with its lift; every point but the last ends the change-filament text, the last one and `GCode::unretract()` follow the filament-start text as parsed; both texts end with a newline before their line's own. The port travelled once, to the tower start without the extruder offset (2 mm off in Y on Majora), and unretracted by a written-out length. `Extruder::unretract` gives back what was retracted, the tool-change length after `retract_for_toolchange`. The tower box is `WipeTower::get_bbx` (the first layer's wall and brim), so the brim is drawn as `finish_layer_new` draws it (4333-4371): `prime_tower_brim_width` (3, a key the port did not read), loops offset by the extrusion spacing from the aligned wall box, one loop less on every layer above the first (the chamfer, layers 2-6), each loop recorded with the wall. From the map: `generate_path_to_wipe_tower`, `get_bbx`, `align_perimeter`, `brim_chamfer_loops_num`, `extrusion_spacing_for`, `is_first_layer_for_slab`, `first_printed_layer_idx`, both `calc_tower_feedrate` | on: `TOWER_TRAVEL_PATH_CPP`, `TOWER_BRIM_CHAMFER_CPP`, `UNRETRACT_RETRACTED_LENGTH` (new) | unchanged (180,394) | unchanged (226,152) | 2,593,339 → 2,605,718 (91.54%) |
| R836 | The layer structure of the prime tower as `generate_new` makes it (WipeTower.cpp:7760-8106). The wall filament of the tower (`get_wall_filament_for_all_layer`) and of each layer (`get_wall_filament_for_layer`) decide where the finish of a layer with tool changes goes: `finish_layer_new` without fill, before the layer's first tool change when the layer starts with the wall filament, else after the tool change that loads it. What the tool changes left of each block is filled by `finish_block` with the block's filament (`get_block_fill_filament`) and merged into that filament's tool change (`merge_tcr`: a travel at the tower's maximum speed unless the second part starts where its tool change starts, then the second writer's analyzer lines and G-code). Only a layer without tool changes keeps a finish of its own, with its rectangle and grid. The finish and fill begin at the fill box's right or left upper corner, write `; layer #N` with the grid, and end with a wipe to the next corner or along the last grid line. On the G-code side a result that begins with the finish is reached as `append_tcr` reaches one with `is_finish_first` (GCode.cpp:787-795: retract with a lazy spiral lift, travel, unretract), and the tool change's retract goes after the `; WIPE_TOWER_START` of the tool change, not of the finish. The port finished every layer at its end, after the objects, with fill, wall and brim | on: `TOWER_GENERATE_NEW_CPP` (new) | unchanged (180,394) | unchanged (226,152) | 2,605,718 → 2,610,010 (91.81%) |
| R837 | The head of every Majora layer. (1) The timelapse template: the placeholder parser writes a double through an ostringstream, six significant digits (PlaceholderParser.cpp:271-283), so `{layer_z}` is `15`, not `15.000000000000014`; and when the template leaves the head at another Z, `G1 Z%.3f` with the Z from before it follows, the writer's position left alone (GCode.cpp:4783-4806). Its own feedrate is current after it, as the cooling buffer reads the text. (2) A tower result that is not a tool change (the timelapse wall, the finish of a layer without tool changes) is reached as `append_tcr` reaches any `is_finish_first` result (GCode.cpp:787-795): lazy spiral retract, `GCode::travel_to` (extruder offset), unretract; the port wrote a raw move to it. (3) `only_generate_out_wall` (WipeTower.cpp:8215-8280) starts at the fill box's upper corner of the layer depth as C++ reads it, uses `calc_tower_feedrate` and writes no feedrate line of its own; `generate_path` writes the feedrate on each extrusion (WipeTower.cpp:1295, 1307), not on a `G1 F` line | on: `TIMELAPSE_LAYER_Z_G6`, `TIMELAPSE_Z_RESTORE`, `TIMELAPSE_F_NOTED`, `TOWER_FINISH_FIRST_TRAVEL_CPP`, `TOWER_OUT_WALL_CPP`, `TOWER_PATH_F_INLINE` (new) | unchanged (180,394) | unchanged (226,152) | 2,610,010 → 2,614,706 (91.95%) |
| R838 | The prime tower's wall gaps and flow. (1) `scaled(const Vec2f&)` divides the float coordinate by the scaling factor as a float (35.0f / 1e-5f rounds to exactly 3500000 in float; corrected in R842, this row first said 1e-6f); the port divided in double and truncated, which put the wall's right edge at 34.99999 and every skip point at x = 35 to the right of it, so the ray of `remove_points_from_polygon` met no edge and that side of the wall had no gaps (layers with `layer_id % 4` of 1 or 2). (2) `generate_path` simplifies the polylines first, arc fitting when `enable_arc_fitting` (WipeTower.cpp:1265-1272), which drops the collinear points `add_extra_point` inserts; the port had it as a diagnostic switch, measured before the gaps worked. (3) The tower's flow divides by `filament_area` as the print fills it, `float((M_PI/4.f) * pow(d, 2))` (WipeTower.cpp:2047); the port's `PI * 0.875 * 0.875` in float is one unit in the last place above, which makes a 35 mm wall line E1.9016 against E1.9017. (4) `post_process_wipe_tower_moves` reads one more, empty line after a text that ends with a newline and writes it: every tower block ends with one empty line more (3,443 on Majora) | on: `TOWER_SCALED_VEC2F`, `TOWER_ARC_FITTING_CPP` (was a probe), `TOWER_FILAMENT_AREA_CPP`, `TOWER_XFORM_TRAILING_BLANK` (new) | unchanged (180,394) | unchanged (226,152) | 2,614,706 → 2,621,490 (92.06%) |
| R839 | (1) The tower's grid. `WipeTowerInfo::extruder_fill` defaults to true (WipeTower.hpp:916); under smooth timelapse the print clears it on the layers whose `wipe_tower_partitions` is 0 (Print.cpp:4331-4335), and the partitions are propagated down from the layers above (ToolOrdering.cpp:1213-1228), so only the layers above the last tool change lose it. The port's default was false, which left every `finish_block` without its grid (209 on Majora). (2) The layer head. The processed layer-change template is followed by "\n" (GCode.cpp:5094-5096): an empty line only when the template ends with a newline (Benchy's does, Majora's does not; the port always wrote one). Then, before `change_layer` lifts: on the second layer the power-loss recovery and, with `scan_first_layer` (a key the loader did not read), the scan between a retract and an unretract; from then on the nozzle temperature of the loaded filament when it is not its first-layer one, `M104 M S200 ; set nozzle temperature` (GCode.cpp:5138-5202, GCodeWriter.cpp:136-171) | on: `TOWER_EXTRUDER_FILL_CPP`, `LAYER_CHANGE_TEMPLATE_TEXT`, `SECOND_LAYER_THINGS_CPP` (new) | 180,394 → 180,395 (94.27%) | 226,152 → 226,153 (92.95%) | 2,621,490 → 2,626,178 (92.10%) |
| R840 | (1) The wipe of a tool change's retraction (`Wipe::wipe` with `toolchange`, GCode.cpp:450-453, 469-472) runs at `prime_tower_wipe_speed`, or `prime_tower_max_speed` when that is 0 (F5400 on Majora), over `prime_tower_wipe_distance` when set; `set_speed` makes that the writer's current speed (GCodeWriter.cpp:450), which the wipe when leaving the tower then takes (`role_base_wipe_speed`). The port wiped at the current extrusion speed both times. Two keys the loader did not read. (2) `generate_new` calls `set_layer` with `is_first_layer` false on every layer (WipeTower.cpp:7787), so the counter of the grid's `; layer #N` is 1 on the first layer | on: `TOOLCHANGE_WIPE_SPEED`, `TOWER_LAYER_COUNT_CPP` (new) | unchanged (180,395) | unchanged (226,153) | 2,626,178 → 2,631,613 (92.21%) |
| R841 | (1) The flush lengths of the change-filament template are float in C++ (GCode.cpp:985-987, 1104-1108: `float purge_volume`, `float new_filament_area`, `float purge_length`, `float flush_unit`, rounding with float constants) and the template reads them as doubles; the port computed in double, which moved the sixth digit of about 2,700 flush lines (`G1 E5.41813` against `E5.41814`). (2) From the map, a structural move with no change in output: `WipeTowerIntegration::append_tcr` (the port's `emit_tower_tcr` in `print.rs`) and `post_process_wipe_tower_moves` (`transform_gcode` in `gcode/wipe_tower_integration.rs`) now live in `g_code.rs`, the file named after GCode.cpp, under those names; `gcode/wipe_tower_integration.rs` is gone. Also: a citation in `writer.rs` corrected (`GCode::needs_retraction` is GCode.cpp:8363-8496) | on: `FLUSH_LENGTH_FLOAT` (new) | unchanged (180,395) | unchanged (226,153) | 2,631,613 → 2,631,617 (92.21%) |
| R842 | The wipe on the scaled polyline. `Wipe::wipe` (GCode.cpp:474-505) clips `[last_pos] + path[1..]` with `clip_end(length() - scale_(wipe_distance))`, which puts the new end at `(last + v * (d / |v|)).cast<coord_t>()`, truncated (Polyline.cpp:52-72); each segment's dE is `length * (line.length() / wipe_dist) * 0.95` in scaled units; and the retraction after the wipe is `length - m_retracted`, the sum of those dE (Extruder.cpp:71-130). The port clipped in floating millimetres and took the rest as `0.05 * length`, so a one-unit shorter or longer end never showed: the oracle's tool-change wipes summing to 1.90001 and the `G1 E-.09999` after them. Also: two statements of R838 and R841 corrected (see the findings) | on: `WIPE_CLIP_SCALED` (new) | 180,395 → 180,416 (94.28%) | 226,153 → 226,166 (92.95%) | 2,631,617 → 2,631,744 (92.21%) |
| R843 | (1) E per move. `_extrude` gives each linear move `e_per_mm * line_length` with `line_length = line.length() * SCALING_FACTOR` in double (GCode.cpp:7987, 8021); the port truncated the distance to whole units before scaling, so its E was one unit short in the last digit on 3% of Benchy's moves and 3% of Majora's, nearly always on the low side. (2) Floating-point contraction. The oracle's libslic3r is built with Apple clang -O3, whose default `-ffp-contract=on` fuses `p*q + r` inside an expression; a twin probe of `Circle::try_create_circle` (Circle.cpp:15-55) built the same way shows LLVM fusing the left product of an addition with the right product as addend and each further product onto that. With those fusions the port's a, b, c, centre and radius equal the C++ bit for bit on 3,988 of 3,988 triples (62 centres without). The centre is rounded to an integer point before use, so no byte of the fixtures changes | on: `LINE_LENGTH_DOUBLE`, `CIRCLE_FMA` (new) | 180,416 → 180,427 (94.28%) | 226,166 → 226,193 (92.96%) | 2,631,744 → 2,631,876 (92.21%) |
| R844 | The layer change as 02.09 writes it. (1) `GCode::change_layer` (GCode.cpp:6473-6503), called right after the layer head (:5066): with `retract_when_changing_layer` and a Z change it retracts, wiping, and asks for a lazy lift (SpiralLift when `z_hop_types` is Auto), then sets `m_need_change_layer_lift_z`, so the layer's first travel lifts with a spiral pointed along the travel and a `G1 X Y Z` at the lifted Z (:7611-7618; GCodeWriter.cpp `travel_to_xyz`). The port retracted at the end of the previous layer, before the head, and lifted with a static spiral (`I1.217 J0`) after the layer-change G-code. `change_layer` in `g_code.rs` is now that function and the one `export_gcode` calls; nothing sets the travel acceleration at the layer start any more, the travel does. (2) Before the timelapse block at the layer start, `retract(false, false, auto_lift_type, true)` (:5604-5607), an eager lift: the static spiral Majora's layers have, or, where the head's position is unknown, a plain `G1 Z` after `set_travel_acceleration()` (`_travel_to_z`). (3) With a prime tower `_do_export` moves to max(first layer height + z_hop, 0.4) before the first layer (:3482-3490). (4) A retraction without a wipe leaves its F as the current feedrate (`G1 Z1.1 F30000` after `G1 E-.8 F1800`). (5) `extrude_perimeters` writes `; COOLING_NODE` before the entity's travel (:6968-6976); the port wrote it after the bucket's first travel | on: `CHANGE_LAYER_RETRACT_CPP`, `FIRST_LAYER_Z_MOVE_CPP`, `RETRACT_F_NOTED`, `EAGER_LIFT_TRAVEL_ACCEL`, `COOLING_NODE_BEFORE_TRAVEL` (new) | 180,427 → 183,712 (95.96%) | 226,193 → 229,892 (94.44%) | 2,631,876 → 2,635,432 (92.32%) |
| R845 | (1) The cooling editor parses an F-only line with the position before it, E included, and resets only the current E in relative mode (GCodeEditor.cpp:180, 233-258), so after any extrusion such a line has a length; at the current feedrate and not adjustable it goes the remove-F way, where the bare `G1` is not kept but the rest of the line, its newline, is (:618-664): an empty line. The port dropped the line. (2) `thick_polyline_to_multi_path` gives every finished path the overhang degree (VariableWidth.cpp:86), not only the last (:94); an arachne overhang wall split by width printed its earlier paths at the bridge speed (F3000) where the oracle has the overhang speed (F600). (3) `extrude_loop` hands every path to `_extrude`, which sets that path's acceleration (GCode.cpp:7638-7666; an overhang wall takes the outer wall's, the inner wall after it the default): `M204 S5000` before an overhang wall inside an inner-wall loop and `M204 S10000` after it. From the map: `GCode::to_lift_type` (GCode.cpp:8348-8361) in `g_code.rs` with the C++ `LiftType` names; the writer's copy calls it | on: `EDITOR_BARE_F_LINE`, `MULTI_PATH_OVERHANG_DEGREE`, `LOOP_PATH_ACCEL` (new) | 183,712 → 183,736 (95.96%) | 229,892 → 230,206 (94.55%) | 2,635,432 → 2,637,145 (92.38%) |
| R846 | The processor's time, from the map. (1) `process_tags` (GCodeProcessor.cpp:3381-3632) was a stub: roles, `; Z_HEIGHT`, wipe and wipe-tower tags, skippable blocks, flush tags, width and height tags and the layer change now reach the processor; the role tag `Custom` before the machine start G-code opens the prepare stage (:3393), whose blocks are the prepare time. (2) `process_G4` (:5304-5312) dwells; `process_G29` (:5315-5324) takes `machine_prepare_compensation_time` (260 s) inside an `M622 J1` block, which `process_M622` / `process_M623` (:5776-5791) open and close; the config key now reaches `PrintConfig`. (3) `get_time` / `get_prepare_time` (:2888-2896) and the header of 02.09: `; BambuStudio 02.09.01.70` (`header_slic3r_generated`, version.inc:16) and, on a Bambu Lab printer, `; model printing time: ..; total estimated time: ..` (:711-716). From the G-code: `_spiral_travel_to_z` writes `set_travel_acceleration()` before `G17` (GCodeWriter.cpp:800) | on: `PROCESS_TAGS`, `PROCESS_G4_G29`, `HEADER_0209`, `SPIRAL_LIFT_ACCEL` (new) | 183,736 → 183,738 (95.96%) | 230,206 → 230,208 (94.55%) | 2,637,145 → 2,637,152 (92.38%) |
| R847 | The time of a tool change, from the map. `process_T` and `process_M1020` were no-ops (GCodeProcessor.cpp:5909-5984); they now hand the filament to `process_filament_change` (:6054-6125), which takes the target nozzle from the nozzle group result and the nozzle status recorder, adds the extruder change, unload and load times (`get_extruder_change_time`, `get_filament_unload_time`, `get_filament_load_time`, :6357-6373) to a `Tool_change` block and synchronises. The port's print has no nozzle group result, so the processor builds C++'s fallback, one nozzle per extruder of `filament_map` (`ensure_nozzle_group_result`, :5993-6052), which for these single-nozzle jobs is the same. `apply_config` now reads the filament count, the per-filament vectors, `nozzle_volume`, `handle_hotend_as_extruder`, the legacy load and unload times through `resolve_ams_load_unload` (MultiNozzleUtils.cpp:1152-1176, `default_ams_type` -1), `machine_switch_extruder_time` and `filament_map` (:1964-2079). The start G-code's `T0` costs one filament load, as in the oracle | on: `PROCESS_FILAMENT_CHANGE` (new) | 183,738, unchanged | 230,208, unchanged | 2,637,152, unchanged |
| R848 | The processor's machine envelope, from the map. C++ copies the machine limits from the config (GCodeProcessor.cpp:2030-2036); the port's `PrintConfig::set_deserialize` did not read `machine_max_speed_*`, `machine_max_jerk_*`, the per-axis accelerations or the minimum rates, so the processor ran with defaults (Benchy: 500 mm/s on X and Y against 1000). And `process_M201`, `process_M203`, `process_M205` and `process_M566` (:5600-5649, 5680-5708, 5857-5872) were empty, so the start G-code's `M203 X1000 Y1000 Z30 E50` did nothing; they now set the Normal mode's limits | on: `MACHINE_LIMITS` (new) | 183,738, unchanged | 230,208, unchanged | 2,637,152, unchanged |
| R849 | The split points of an arachne wall's overhang degrees, localized with the `PP` probe (perimeters differed on 577 of the 618 Majora layers whose slices agree; `WallToolPaths` agreed on 99.8% of its 40,760 calls) and a twin probe of `detect_overhang_degree` (`OVD`). (1) `get_split_points` places a split at `pa + (pb - pa) * t` (OverhangDetector.cpp:216-228): `pb - pa` is an Eigen expression, not a `Point`, so the member `Point::operator*(const double&)`, which rounds through `Point(double, double)`, does not bind; the free `operator*(const Point&, const double&)` does, through the converting constructor, and truncates (Point.hpp:200, 255-258). The port rounded, so every split point with a negative fraction moved one unit and the walls' smooth speeds, arc centres and wipes moved with it. (2) `coord_t offset_width = scale_(nozzle_diameter) / 2` (:236): the nozzle diameter is a float (PrintConfig.hpp:2292), 0.4f scales to 20000.0003 and `coord_t` truncates it to 20000; the port kept the fraction in every degree | on: `SPLIT_POINT_TRUNC`, `OVERHANG_OFFSET_COORD` (new) | 183,738, unchanged | 230,208 → 236,621 (97.18%) | 2,637,152 → 2,698,052 (94.51%) |
| R850 | (1) The G-code origin of a 3MF object. `process_layer` sets `m_origin = unscale(shift)` (GCode.cpp:5873), where an instance's `shift` is its translation `Point::new_scale`d (PrintApply.cpp:150) plus `m_center_offset` (PrintObject.cpp:114), two truncations added as integers. The port took the truncated centre of the placed mesh, one truncation of the sum, which on Majora sat one unit off in X: every X moved by 1e-5 mm and about 1% of them printed a thousandth higher. The 3MF loader now records the instance's integer shift and the export uses it. (2) `traverse_extrusions` clips each grown lower polygon to the bounding box of the extrusion's junctions before the overhang clip and the distancer (PerimeterGenerator.cpp:675-692, `clip_clipper_polygon_with_subject_bbox(.., true)`); the box is not inflated, the `inflated()` at :684 returns a copy that is dropped. (3) `point_to_gcode` evaluates `unscale(point) + m_origin - extruder_offset` left to right (GCode.cpp:9029-9033); the port subtracted their sum. No byte of the fixtures moves with (3) | on: `GCODE_ORIGIN_INSTANCE_SHIFT`, `ARACHNE_CLIP_BBOX`, `POINT_TO_GCODE_ORDER` (new) | 183,738, unchanged | 236,621, unchanged | 2,698,052 → 2,712,617 (95.02%) |
| R851 | The anchor lines of the floating vertical shell, localized with the `PP` probe (on a Majora layer every stage of `prepare_infill` agreed and the fills did not) and a twin probe of the floating filler's inputs (`FFC`): every call received other `lower_sparse_polys` than the oracle's (22 polygons of 5.3e11 where the oracle has 9 of 8.7e11). `Layer::generate_sparse_infill_polylines_for_anchoring` runs the real filler, `f->fill_surface(&surface, params)` (Fill.cpp:869-977): `Fill::fill_surface` shrinks the surface by half a spacing and `FillGyroid::_fill_surface_single` generates, clips and connects the waves in the rotated frame. The port's anchoring arm for gyroid connected unshrunk, unrotated waves with default anchor lengths, a path `make_fills` left in R473 and R791. It now takes the path `make_fills` takes | on: `FVS_ANCHOR_GYROID_FSS` (new) | 183,738, unchanged | 236,621, unchanged | 2,712,617 → 2,747,529 (96.25%) |
| R852 | The `M73 P.. R..` progress lines, from the map (`TimeProcessor::post_process` was missing). Before every G1, G2, G3 and `;VG1` line of the finished G-code `process_line_move` writes the pair <percent, remaining minutes> when it changed (GCodeProcessor.cpp:816-885, 1019-1032), from the planner's `g1_times_cache`; its ids follow `m_g1_line_id`, which G0 and G28 advance too, while the text counter counts G1/G2/G3 lines only, as in C++. The first line's `M73 P0 R..` takes `time_in_minutes(machine.time)` (:589-592, 683-688); the port wrote the writer's crude time there (R15 for a 48-minute print). Ported: the M73 part of the function, the stop-time branch included; not the placeholders, the filament and extruder blocks or the pre-heating injection | on: `M73_LINES` (new) | 183,738 → 183,740 | 236,621 → 236,625 | 2,747,529 → 2,747,545 |
| R853 | (1) The Width tag after a prime tower visit. GCode.cpp:7886 writes it once per `_extrude`, `if (last_was_wipe_tower \|\| m_last_width != path.width)`; the port evaluates that statement at two places and after a tower visit wrote the tag at both, 3,339 times on Majora (the oracle has the same tag twice within four lines 2 times, the port had 3,341 and now has 2). (2) The time estimator, localized by running the port's processor over the oracle's Benchy G-code (`TIMEPROC_REPLAY`) and a twin dump of every planner block (`TIMEDBG_BLOCKS`): of 131,695 blocks 4,042 differed in acceleration, 2,549 in move type and 27 were missing or extra. Three causes, each fixed behind an opt-in gate. `gcode_flavor` was not read: "marlin" is gcfMarlinLegacy (PrintConfig.cpp:545-559), for which the travel limit is the extruding one (GCodeProcessor.cpp:2032-2035), so travels plan at 10000 mm/s2, not 9000. The `absolute_position` lambda of `process_G2_G3` returns a double (:4912-4935); returning f32 gave the E of a spiral lift a delta, and the arc was typed Extrude. `process_G28` asks `line.has('X')`, the `char` overload that finds a letter without a value (:5365-5376; GCodeReader.cpp:225-245): `G28 Z P0 T250` homed X and Y too. With the three on, the replay's total is the oracle's (2851.220 s against 2851.21997), every per-line elapsed time is within 0.0006 s, and all blocks agree but 251 in the cruise feedrate and the last in its exit feedrate. They are opt-in: on the port's own G-code every combination of the three moves `M73` lines and takes 1 to 3 in-order lines off one Benchy fixture (all on: 183,741 / 236,623 / 2,747,551) | on: `WIDTH_TAG_FORCE_ONCE` (new); opt-in: `GCODE_FLAVOR_KEY`, `ARC_ABS_POS_DOUBLE`, `G28_HAS_CHAR` (new) | 183,740, unchanged | 236,625, unchanged | 2,747,545, unchanged (96.06% → 96.17%: 3,339 lines fewer in the port's body) |
| R854 | The time estimator, finished against the Benchy replay of R853, with the planner blocks now compared bit for bit. (1) The centripetal limit of `process_G1`, `float sin_theta_2 = sqrt((1.0f - cos(angle)) * 0.5f)` (GCodeProcessor.cpp:4339): `angle` is a float and libc++ declares `float cos(float)` in the global namespace, so the subtraction cancels in single precision; the port took the cosine in double. That was the 251 blocks. (2) The arc branch (`process_G2_G3`) never assigns `curr.axis_feedrate[E]`, it multiplies what the last move left (:5135), and never resets `curr.safe_feedrate`, it only lowers it (:5173-5179): a spiral lift after `G1 E-.02 F1800` inherits the retract's E feedrate and safe feedrate 2.5. That was the last block's exit. (3) Single against double: `AxisCoords` is `std::array<double, 4>` (GCodeProcessor.hpp:548) and both `delta_pos` and `State::axis_feedrate` are of it (:639-640), so `curr.feedrate * delta_pos[a] * inv_distance`, the feedrate factor `std::min<float>(f, max / abs)` and the axis acceleration limits (:4347-4376, :5131-5165) are evaluated in double and rounded once; `2 * PI * norm` of a `P1` arc (:4999), `sqrt(sqr(arc_length) + sqr(delta_pos[Z]))` (:5023) and `2 * M_PI + radian` (Circle.cpp:496-498) likewise. (4) clang on arm64 contracts `a * b + c` written in one expression: `speed_from_distance`, `max_allowable_speed`, `intersection_distance` (:126-143) and the dot and cross product of the centripetal angle (:4336-4337) are fused multiply-adds. With the five opt-in gates on (`GCODE_FLAVOR_KEY`, `ARC_ABS_POS_DOUBLE`, `G28_HAS_CHAR` of R853, `TIME_FLOAT_MODEL`, `TIME_ARC_STALE_STATE`) every field of all 131,695 Benchy blocks and every elapsed time is the same float as in C++, and the replay over each kept oracle writes every `M73` line of the oracle at its place with its values: Benchy 133 of 133, arachne 132 of 132, Majora 5,804 of 5,804 (total 345,347 s, the oracle's 3d 23h 55m 47s). The lines the replay cannot write are the first `M73 P0`, which is the G-code writer's placeholder, and the `M73 P100 R0` of the machine end G-code. Gates off the output is R853's byte for byte on all three; with the five on the bodies without `M73` lines are still those, and the in-order counts are 183,741 / 236,623 / 2,747,551, so they stay opt-in for the two arachne lines. Suite: front end 28 pass; engine 1980 pass, the same 84 fail. | opt-in: `TIME_FLOAT_MODEL`, `TIME_ARC_STALE_STATE` (new) | 183,740, unchanged | 236,625, unchanged | 2,747,545, unchanged |
| R855 | The inputs of the floating concentric fill, from the Majora diff (Floating vertical shell was the largest loss, 27,692 lines over 610 of 629 layers). The `FFC` twin probe of R851 showed Arachne's output equal on all 5,157 calls and both lower-layer inputs different on all of them, so `resplit_order_loops` split the loops at other points. (1) `lower_layer_unsupport_areas` (Fill.cpp:734-745): per region of the lower layer `union_ex` of its stInternal and stInternalVoid fill surfaces, accumulated with `union_ex(acc, region)` (ClipperUtils.cpp:825-831), then `shrink_ex(acc, SCALED_EPSILON)`, which offsets each expolygon on its own and unites the paths through a PolyTree (ClipperUtils.hpp:392, ClipperUtils.cpp:552-571). The port united all regions at once and shrank with the geo-clipper offset, whose grid is 1 um. Equal to C++ on 4,659 calls after (0 before); the rest sit on layers whose fill surfaces differ upstream. (2) `lower_sparse_polys` (Fill.cpp:756-762): `float internal_infill_width`, `offset(Polylines, width / 2)` as one ClipperOffset per path and one `clipper_union<Paths>` (ClipperUtils.cpp:420-421, :273-300), then `union_` (:721-722). The port used the geo-clipper polyline offset. Equal on 2,959 calls after (0 before). The fill's output is the same on 4,339 calls (1,770 before). Also measured this round: the oracle's multi-material segmentation differs between runs on 80 of Majora's 656 layers (layer 0 gave 4 results in 5 runs), and the port differs from it on 21 layers where 5 runs agree (175, 211-235, 250, 259, 459, 460, 493). Suite: front end 28 pass; engine 1980 pass, the same 84 fail. | on: `FFC_LOWER_UNSUPPORT`, `FFC_LOWER_SPARSE_CLIB` (new) | 183,740 → 184,212 (95.89% → 96.14%) | 236,625 → 237,255 (97.13% → 97.39%) | 2,747,545 → 2,782,052 (96.17% → 97.37%); Floating vertical shell 90.3% → 99.3% |
| R856 | The helpers that still computed on the geo-clipper grid (1 um, where C++ computes at coord_t, 10 nm), found with a new census: `GEOCALLS=1` counts, per call site, the calls that reach one of them and prints the table at the end of the export. After the routing gates of earlier rounds (`CLIPPER_INT`, `CLIPPER_DIFF_GEO`, `CLIPPER_UNION_GEO`) seven call sites were left on the three fixtures. (1) Six in the Arachne branch's top-one-wall detection: the upper and lower slices are now clipped to the bounding box of the one-wall infill contour before the three `diff_ex` (PerimeterGenerator.cpp:1618-1634), `should_enable_top_one_wall` takes `perimeter_width` and `ext_perimeter_spacing` as coord_t and offsets with a float delta at coord_t (:1853-1864), and `last`, the united infill contour and its intersection (:1640-1656) go through ClipperLib. (2) One in `group_fills`: `offset_ex(expoly, SCALED_EPSILON)` of the floating-shell test (Fill.cpp:570) is ten coord_t units, a tenth of the geo grid step. The census is empty on all three fixtures after the round. Suite: front end 28 pass; engine 1980 pass, the same 84 fail. | on: `ARACHNE_TOP_ONE_WALL_CLIB`, `FVS_DETECT_OFFSET_CLIB` (new) | 184,212, unchanged | 237,255 → 238,663 (97.39% → 97.97%) | 2,782,052 → 2,782,529 (97.37% → 97.39%) |
| R857 | The five estimator gates turned on by default: `GCODE_FLAVOR_KEY`, `ARC_ABS_POS_DOUBLE`, `G28_HAS_CHAR` (R853), `TIME_FLOAT_MODEL`, `TIME_ARC_STALE_STATE` (R854) go from `opt_in_gate` to `faithful_gate`. No code changes besides. The estimator is exact over each oracle's own G-code (R854); over the port's G-code the times differ because the moves do, and the `M73` lines that move take 2 in-order lines off Benchy arachne. Accepted by the maintainer as an exception to the no-lowering rule; the lines return when that fixture's moves agree. All five `=0` reproduce the R856 files byte for byte. `M73 P` lines, port against oracle: Benchy 136/136, arachne 137/135, Majora 5,808/5,806. | `GCODE_FLAVOR_KEY`, `ARC_ABS_POS_DOUBLE`, `G28_HAS_CHAR`, `TIME_FLOAT_MODEL`, `TIME_ARC_STALE_STATE` (now on) | 184,212 → 184,213 (96.14% → 96.15%) | 238,663 → 238,661 (97.97%) | 2,782,529 → 2,782,538 (97.39%) |
| R858 | Closed gap fills, from the Benchy diff (Gap infill was its largest loss, 1,917 lines; the chimney layers 244 to 290 lost a whole ring each). (1) `variable_width` (VariableWidth.cpp:220-230) keeps the paths of one thick polyline together and appends them as one `ExtrusionLoop` when the first point of the first equals the last point of the last; `LayerRegion`'s gap fill appended every path on its own. (2) `GCode::extrude_loop` (GCode.cpp:6679-6684) gives only loops with the description "perimeter" to the seam placer and splits every other loop at the point nearest the nozzle, `loop.split_at(last_pos, false)`; the port sent every loop to the seam placer. The port does not carry the description, so the role decides: anything but the three perimeter roles. With both, a closed gap fill starts next to where the wall before it ended, without a retraction, runs counter-clockwise and is clipped at its end, as in the oracle. Benchy gap infill lines missing 1,917 → 401. No effect on arachne or Majora (no closed gap fill there). Seen and left: `G1 F1166` where the oracle has `F1165` on the same layers. | `GAPFILL_LOOP`, `LOOP_SPLIT_NONPERIMETER` | 184,213 → 185,972 (96.15% → 97.35%) | 238,661, unchanged | 2,782,538, unchanged |
| R859 | Two findings from the Benchy queue. (1) `G1 F1166` where the oracle has `F1165` (one line): GCodeEditor.cpp:620 writes a slowed line's feedrate as `int(floor(60. * line->feedrate + 0.5))`, in double; the port multiplied in f32. (2) Internal solid infill of Benchy layers 48 and 51 (564 of its 863 missing lines): the same scan lines, linked differently, with retractions where the oracle follows the contour. Stage checksums (`PP=1`) agree up to the fill's input, so a twin probe went into `fill_surface_by_lines` (`RECT=1`: sums after `slice_region_by_vertical_lines`, after `connect_segment_intersections_by_contours`, and of the polylines). The intersections agreed; the link qualities did not: C++ ran with `link_max_length` 0, the port with three spacings. Fill.cpp:772-784 computes `3. * f->spacing` for a density above 80%, but `f` is the object `Fill::new_from_type` made at Fill.cpp:699 and its `spacing` is still the constructor's 0 (FillBase.hpp:194); the assignment is at Fill.cpp:844 (the same at 931 and 968). So in `make_fills` no link is ever too long. Only ironing (Fill.cpp:1191-1193) sets the spacing first. `RECT` stays as a probe. The C++ probe also showed 48 calls of this function against the port's 123: the port sends fills through its rectilinear filler that C++ gives to other classes; not looked at yet. | `COOLING_F_DOUBLE`, `LINK_MAX_LENGTH_FRESH_FILL` | 185,972 → 187,118 (97.35% → 98.02%) | 238,661 → 240,025 (97.97% → 98.59%) | 2,782,538 → 2,786,069 (97.39% → 97.54%) |
| R860 | Majora walls (Outer wall 19,357 and Inner wall 11,101 lines missing, spread over 575 layers). Layer 64 showed short wall pieces the oracle prints as `Overhang wall` at `F600` and the port as `Outer wall` at the wall speed. A twin probe at PerimeterGenerator.cpp:690 (`ARBB`, removed again) printed, for every extrusion of at most 8 junctions, the lower polygons left by the bounding-box clip: the same polygon on both sides, but 8,650 events in C++ against the port's 3,437, the difference being every open extrusion. (1) The guard of PerimeterGenerator.cpp:667-668 is `detect_overhang_wall && layer_id > raft_layers && !starts_at_object_base`; the port added `&& line.is_closed`, so an open Arachne extrusion never met the lower slices: no overhang pieces, no overhang degrees. With it goes the start point of PerimeterGenerator.cpp:728-757: an end of the chain, a non-overhang one if there is one, found by walking a `std::unordered_map<Point, PointInfo, PointHash>`; the libc++ container's iteration order is emulated as `region_expansion.rs` does for `double` keys. (2) `calc_speed_by_role` of `_extrude` (GCode.cpp:7702-7732) for a path on its own: a wall path takes the speed of its overhang degree, an overhang wall of degree 5 `overhang_totally_speed`, any other overhang wall the bridge speed; the port's single-path branch gave all three the wall speed. Majora after: Outer wall 9,786, Inner wall 7,358 lines missing. Seen and left for R861: C++ appends an open extrusion as one `ExtrusionMultiPath` (PerimeterGenerator.cpp:807-823), the port as separate paths, so the wipe after it covers only the last piece (`extrude_multi_path`, GCode.cpp:6884-6903). | `ARACHNE_OVERHANG_OPEN`, `PATH_OVERHANG_SPEED` | 187,118, unchanged | 240,025 → 240,150 (98.59% → 98.64%) | 2,786,069 → 2,817,342 (97.54% → 98.41%) |
| R861 | `ExtrusionMultiPath` as an entity. C++ appends an open Arachne extrusion as one multi-path, starting a new one where two consecutive paths do not meet (PerimeterGenerator.cpp:807-823), and `GCode::extrude_multi_path` (GCode.cpp:6884-6903) sends every path through `_extrude` and then installs one wipe path for the whole of it: the paths' points in order, a joint written once, reversed. The port's `ExtrusionEntityType` had `Path`, `Loop` and `Collection` only (the struct was ported, unused), so an open extrusion was a run of separate paths and the wipe after it covered the last piece. The enum gains `MultiPath`; its 33 match sites follow the C++ dispatch of each (`ConflictChecker.cpp:154-155`, `LayerRegion.cpp:1044-1045` `simplify_multi_path`, `PrintExtents.cpp:91-93`, `ShortestPath.cpp` through `first_point`/`last_point`/`can_reverse`/`reverse`). The gate is at the one place that builds them. Left on Majora layer 64: an empty line the oracle has after some `; LINE_WIDTH:` tags of overhang walls (a feedrate line the cooling pass emptied), speed-only `G1 F` lines the port has in the middle of a path, arc centres one unit off in the third decimal, the start of some tiny loops. | `ARACHNE_MULTIPATH` | 187,118, unchanged | 240,150 → 240,269 (98.64% → 98.70%) | 2,817,342 → 2,830,440 (98.41% → 98.60%) |
| R862 | No code. A tracked list of what is not ported, `parity/not-ported.md`: the 28 + 6 + 16 functions of the shim crates with their call sites (ClipperLib and the boost Voronoi medial axis through `clipper-z-sys`, Clipper2 through `clipper2-z-sys`, Eigen transforms, Douglas-Peucker, edge collapse, sampling and the seam placer's rays through `eigen-transform-sys`; `boost-voronoi-sys` is called by one example only), the child processes, the places where the port's structure differs from the C++ on the fixtures' path (found in R858 to R861), and the library units a slicing job can reach that have no Rust file or are `partial`. Rounds that close an item move it to the list's last section. | none | 187,118, unchanged | 240,269, unchanged | 2,830,440, unchanged |
| R863 | Tests only. Eight of the 84 failing engine tests had expectations of an older engine and not of the C++: the 1e6 scaling (`tests::test_scaling`; libslic3r.h:58 has `SCALING_FACTOR = 0.00001`), rounded where C++ truncates (`flow::tests::test_scaled_values`, Flow.hpp:62 and :68 `coord_t(scale_(float))`; `test_meshfix_constants`, WallToolPaths.hpp:19-21 `scaled<coord_t>(0.5)` is 49,999 and `scaled<coord_t>(2.)` 199,999), untrimmed numbers (`gcode::tests::{test_linear_move, test_rapid_move}`, `gcode::writer::tests::test_writer_travel`), and the third argument of `extrude_to` read as a target where it is dE (`test_writer_extrude_{relative,absolute}_e`; Extruder.cpp:71-88). Corrected to what the C++ computes. Engine tests: 1,988 pass, 76 fail (were 1,980 and 84); no test newly fails. `arc_fitter::tests::test_arc_fitting_empty` looked at and left: C++ returns one segment ending at `size() - 1` of an empty vector, the test wants an error or nothing. | none | 187,118, unchanged | 240,269, unchanged | 2,830,440, unchanged |
| R864 | Arc centres. The missing wall lines of Majora, sorted by kind (`G1` or arc, with or without a port line ending at the same XY): 2,653 arcs whose I or J differ by one in the third decimal, 2,008 moves whose E differs by one in the fifth. GCode.cpp:8041 computes the offset as `point_to_gcode(arc.center) - point_to_gcode(arc.start_point)`, and `point_to_gcode` (9029-9033) adds the origin and subtracts the extruder offset in each term; the port subtracted the unscaled coordinates, on the grounds that the origin cancels. It does not in double, and one arc in a hundred sits on a half of the third decimal. A twin probe (`ARCP=1`, kept: start, end, centre and radius of every arc written) then showed the arcs themselves agree: 421,971 of Majora's 426,993 are identical, and only 167 have the same ends and another centre. The I/J differences left belong to arcs that start somewhere else, and the E differences (215 on Benchy with both ends equal to three decimals) to points that differ below a micron, most on layer 0; neither is a rounding rule. | `ARC_CENTER_OFFSET_ORDER` | 187,118, unchanged | 240,269, unchanged | 2,830,440 → 2,832,110 (98.60% → 98.66%) |
| R865 | Nothing landed; logged so it is not chased again. Majora's Top surface (3,387 lines missing over 298 layers, about ten a layer): the same fill, with line ends and the arcs fitted over the connecting pieces a micron apart (`Y117.041` for `Y117.04`, `I2.036 J2.568` for `I2.03 J2.562`). The stage checksums against the kept C++ dump say the filler's input is the same and its output is not: `prep_top` agrees on 305 of 306 layers, `fills` on 222 of 650. Tried: clang for arm64 contracts `cos * x - sin * y` of `Point::rotate` (Point.hpp:226-231) and `MultiPoint::rotate` (MultiPoint.cpp:27-35) into a fused multiply-add of the left product; with `mul_add` in both the three G-code files are byte-identical to R864's, so the rotation is not where a coordinate goes the other way, and the change was taken out again. Next to look at: the fill of the top surface itself (the monotonic-line filler and what it does at the ends of its lines), with a `RECT`-style twin probe on Majora layer 464. | none | 187,118, unchanged | 240,269, unchanged | 2,832,110, unchanged |
| R866 | No engine code. The port checklist, `docs/parity/PORT_CHECKLIST.md`, from `scripts/parity/port_checklist.py`: all 310 C++ units of `libslic3r`, each with its classes and its 9,052 named functions, the Rust file each maps to and a status. It uses the file map's parser, pairing and estimate; *ported* is a function in the paired Rust file whose body is estimated faithful, *partial* one that is elsewhere, of another overload, or estimated partial, divergent or a stub. *Verified* needs evidence kept by hand in `scripts/parity/port_checklist_verified.json`; it starts with the 13 estimator functions the R854 replay proved bit for bit and `ArcFitter::do_arc_fitting` (R864). At the start: 34.5% ported or verified (3,123 functions; 23.3% by body size), 30.4% partial, 35.1% not started; 43 units fully ported, 33 without a Rust file; 36 shim functions called by the engine. Work is chosen from this list from here on (AGENTS.md). | none | 187,118, unchanged | 240,269, unchanged | 2,832,110, unchanged |
| R867 | First round chosen from the checklist: a shim function off the engine's path. `eigen-transform-sys::douglas_peucker_keep` stood for `MultiPoint::_douglas_peucker` (MultiPoint.cpp:179-230) over `line_alg::distance_to_squared` (Line.hpp:43-68); R806 routed `geometry::simplify::douglas_peucker` to it on the belief that clang's fused multiply-add ranked two near-duplicate vertices the other way. A search over 54 ways to evaluate the distance (the squared norm, the dot product, `t * v - va` and the last norm, each plain or fused) against the shim, on inputs where rounding decides, left nine, all with the last two steps unfused; the nine agree with each other because the products of scaled coordinates are exact in double. So the plain formula is the C++, and it was already ported as `multi_point::douglas_peucker_faithful`; what had differed from the shim was the other algorithm in `simplify.rs` (a stack over index pairs with the rounded projection of `Line::distance_to_squared`). `simplify::douglas_peucker` now calls the port; `tests/douglas_peucker_native_parity.rs` holds it equal to the C++ on 400,000 polylines (families of points at exactly the same distance from the line, near-duplicates one unit apart, degenerate inputs). The three G-code files are byte-identical with the gate on and off. Left: three Douglas-Peucker functions in the port (`multi_point::douglas_peucker`, `douglas_peucker_faithful`, `simplify::douglas_peucker`) where C++ has one; to fold into `multi_point::douglas_peucker` behind a gate, since the first uses the rounded distance and has callers of its own. | `DP_RUST` | 187,118, unchanged | 240,269, unchanged | 2,832,110, unchanged |
| R868 | From the checklist, `MultiPoint::_douglas_peucker` (partial): the function of that name in the port, `multi_point::douglas_peucker`, measured each point against `Line::distance_to_squared`, the distance to the projection rounded to integer coordinates, where `line_alg::distance_to_squared` (Line.hpp:43-68) returns the squared distance to the segment in double. Up to a unit off, enough to keep or drop a point at the tolerance. It has the callers C++ gives to `_douglas_peucker`: the arc fitter, `Polygon::douglas_peucker`, `Polyline::simplify`. It now uses the distance verified in R867, and `tests/douglas_peucker_native_parity.rs` checks it against the C++ as well. With the gate on, `douglas_peucker` and `douglas_peucker_faithful` are the same function; the second goes when the gate is retired. | `DP_TRUE_DISTANCE` | 187,118 → 187,343 (98.02% → 98.14%) | 240,269 → 240,533 (98.70% → 98.85%) | 2,832,110 → 2,832,318 (98.66% → 98.67%) |
| R869 | From the checklist, unit `Line` (partial; `distance_to_squared` and `distance_to_infinite_squared` read as divergent). The port's `Line::distance_to_squared` measured to the projection rounded to integer coordinates, and `distance_to_infinite_squared` divided the squared cross product, taken in exact integers, by the squared length; `line_alg::distance_to_squared` (Line.hpp:43-68) and `line_alg::distance_to_infinite_squared` (Line.hpp:87-104) compute `(t * v - va).squaredNorm()` in double. Both are ported as written, with no fused step (R867). The three G-code files are byte-identical with the gate on and off: the callers on the fixtures' path (Arachne's graph and tool paths, `simplify`) do not sit on a value where the two differ. The checklist script now lets recorded evidence outrank the mechanical estimate, so the three estimator and Douglas-Peucker functions whose bodies carry both gate arms count as verified. Checklist: 3,126 of 9,052 ported or verified (34.5%), 17 of them verified. The two functions of this round still read as partial or divergent to the estimate, which judges a gated body with both arms in it: a port made behind a gate moves the percentage only when the gate is retired and the old arm removed. | `LINE_ALG_DOUBLE` | 187,343, unchanged | 240,533, unchanged | 2,832,318, unchanged |
| R870 | One Douglas-Peucker, as in C++, and the gates of R867 to R869 retired. `multi_point::douglas_peucker` (`MultiPoint::_douglas_peucker`) calls `Line::distance_to_squared`, which is `line_alg::distance_to_squared` without an older arm; `douglas_peucker_faithful` and its private distance are gone, their callers in `geometry/expolygon.rs` use the one function; `geometry::simplify::douglas_peucker` is a call to it with the tolerance scaled, without the shim arm and without its own index-pair algorithm. `eigen-transform-sys::douglas_peucker_keep` is called by the test only. `DP_SHIM`, `DP_RUST`, `DP_TRUE_DISTANCE` and `LINE_ALG_DOUBLE` no longer exist: the three G-code files are byte-identical to R869's, which is the check a retirement has in place of a gates-off run. Shim functions on the engine's path: 36 → 35. | none (four retired) | 187,343, unchanged | 240,533, unchanged | 2,832,318, unchanged |
| R871 | From the checklist, the Arachne units one function short. `toString` of the beading strategies (BeadingStrategy.cpp:51-54 and the five overrides: `"LimitedBeadingStrategy+"`, `"OuterWallContourStrategy+"`, `"OuterWallOfsetBeadingStrategy+"`, `"RedistributeBeadingStrategy+"`, `"Widening+"` before the parent's) existed in the port as `name()`; it is `to_string()` now, the trait method and its seven implementations, so the name is the C++ one. No output changes. The checklist gains a second hand-kept list, `scripts/parity/port_checklist_reviewed.json`: functions the mechanical estimate calls partial or divergent that a round has read side by side with the C++ and found to be the same logic under other identifiers (`upgrade()` for `use_count()`/`lock()`, scalars for an Eigen vector). They count as ported and say so in their note. First entries: `hasTransitions`, `hasTransitionEnds`, `hasExtrusionJunctions` of `SkeletalTrapezoidationEdge` (hpp:74-103) and `line_alg::distance_to_infinite_squared`. Checklist: 3,126 → 3,135 (34.6%). | none | 187,343, unchanged | 240,533, unchanged | 2,832,318, unchanged |
| R872 | From the checklist, unit `Line`, read function by function against Line.cpp and Line.hpp. Three differed. `Line::normal` (Line.hpp:180) is `(b.y - a.y, -(b.x - a.x))`, the vector turned clockwise; the port turned it counter-clockwise, the opposite sign. `Line::extend` (Line.cpp:121-126) scales `vector().normalized()` by the offset, so the division by the norm comes first; the port multiplied first. `line_alg::intersection` (Line.hpp:123-148) works in double and casts the point to `coord_t`, which truncates; the port took the cross products in exact integers and rounded the point. All three are ported as written behind one gate. `intersection_infinite`, `perp_distance_to`, `parallel_to(Line)`, `perpendicular_to(Line)` and `clip_with_bbox` were found to be the same logic and go on the reviewed list. The three G-code files are byte-identical with the gate on and off: none of the three sits on the fixtures' path with a value that differs. Left in the unit: C++ `Line::direction()` is the angle and `Line::vector()` the difference, where the port's `direction()` is the difference and `direction_angle()` the angle, a rename with many callers; the `Linef3` and `Line3` members (`intersect_plane`, `unit_vector`); the overloads taking an angle, which are `parallel_to_angle` and `perpendicular_to_angle` here. | `LINE_CPP_METHODS` | 187,343, unchanged | 240,533, unchanged | 2,832,318, unchanged |
| R873 | From the checklist's shim list: `eigen-transform-sys::project_line_on_line`, which stood for `project_line_on_line` (MultiMaterialSegmentation.cpp:493-514). R806 routed the port to it on the belief that clang fused the dot products. Nine ways to evaluate it (the squared norm and the two dot products, each plain or fused either way) all equal the shim on 900,000 line pairs at three magnitudes, because the products of coordinate differences are exact in double; the endpoints are `a + (t * v1)` truncated. The port's function now computes that in plain double, written out in scalars, and `tests/project_line_on_line_native_parity.rs` holds it equal to the C++. The three G-code files, Majora with its painted model among them, are byte-identical with the gate on and off. Not found: what made the earlier Rust arm differ on 6 of 625 painted lines in R806; it went through `PointF` helpers, which this version does not use. | `MMS_PROJECT_RUST` | 187,343, unchanged | 240,533, unchanged | 2,832,318, unchanged |
| R874 | From the checklist's shim list: the three vertex transforms of `eigen-transform-sys` the engine calls (`transform_verts_unified`, `transform_verts_for_slicing`, `transform_verts_affine_f32_pair`), which stood for `make_trafo_for_slicing` (TriangleMeshSlicer.cpp:1945-1951), the `tf * p` it is used for (:2007-2008) and the f32 product of MultiMaterialSegmentation.cpp:2303. Unlike the 2D double code of R867 and R873, this single-precision code is fused, so the order had to be found: of 16 ways to evaluate a row of `Transform3f * Vec3f` exactly one equals the shim bit for bit, `fma(m2, v2, fma(m1, v1, m0 * v0)) + t` (the columns of the linear part accumulated with fused multiply-adds, the translation added last); of 53 ways to sum the four products of an element of `Transform3f * Transform3f`, the unfused `((p1 + p2) + p0) + p3`. `prescale` multiplies the rows in double before the cast. The port has `make_trafo_for_slicing`, `transform3f_mul_vec3f`, `transform3f_mul_transform3f` and the three transforms in `triangle_mesh_slicer.rs`; `tests/vertex_transform_native_parity.rs` holds them equal to the C++ bit for bit on 60,000 transforms of 20 vertices each. The three G-code files are byte-identical with the gate on and off. Shim functions on the engine's path with default gates: 7 of `eigen-transform-sys` → 3 (`short_edge_collapse`, `sample_uniform`, `raycast_visibility_native`). | `XF_RUST` | 187,343, unchanged | 240,533, unchanged | 2,832,318, unchanged |
| R875 | The gates of R872 to R874 retired, each shown byte-identical on the three fixtures with it on and off: `LINE_CPP_METHODS` (`Line::normal`, `Line::extend` and `Line::intersection` are the C++ only), `MMS_PROJECT_RUST` (`project_line_on_line` has no shim arm) and `XF_RUST` (the engine calls `transform_verts_unified`, `transform_verts_for_slicing` and `transform_verts_affine_f32_pair` of `triangle_mesh_slicer.rs` directly). The three G-code files are byte-identical to R874's, the check a retirement has in place of a gates-off run. `eigen-transform-sys` functions called by the engine: `short_edge_collapse`, `sample_uniform` and `raycast_visibility_native`; the other five are called by their tests only, as the reference. | none (three retired) | 187,343, unchanged | 240,533, unchanged | 2,832,318, unchanged |
| R876 | From the checklist's shim list: `eigen-transform-sys::sample_uniform`, which stood for `sample_its_uniform_parallel` (TriangleSetSampling.cpp:9-70, the seam placer's surface samples). The port had the whole function in Rust already, with the engine's `mt19937_64` and `uniform_real_distribution`; R188 sent it to the shim because nalgebra's `cross` and `norm` were an ulp off Eigen's. A search against the shim, each quantity on its own, leaves one way for each: a component of `Vec3f::cross` is `fma(a, b, -(c * d))` (one expression, contracted), `squaredNorm` is `(x * x + y * y) + z * z` unfused, `normalized()` divides by the square root, and the position blend is `(a * w0 + b * w1) + c * w2` unfused with the weights cast to float. The port's function uses those (`eigen_cross3f`, `eigen_squared_norm3f`); `tests/triangle_set_sampling_native_parity.rs` holds the total area, every sample's triangle, position and normal equal to the C++ bit for bit on 400 random meshes of 300 samples. The three G-code files are byte-identical with the gate on and off. | `SAMPLE_RUST` | 187,343, unchanged | 240,533, unchanged | 2,832,318, unchanged |
| R877 | From the checklist's shim list: `eigen-transform-sys::short_edge_collapse`, which stands for `its_short_edge_collpase` (ShortEdgeCollapse.cpp:11-185, the seam placer's decimated mesh). Not finished. The port has the whole function in Rust; R188 sent it to the shim. Its float operations, and those of `NormalUtils` it calls (NormalUtils.cpp:12-13, 60-63, 110-112), now follow the C++ build as found in R876: `eigen_cross3f`, `eigen_squared_norm3f`, and two more next to them, `eigen_normalized3f` (divide by the square root of the squared norm when positive) and `eigen_dot3f` (`(x + y) + z`, unfused, assumed from the squared norm and not searched on its own). With that `tests/short_edge_collapse_native_parity.rs` holds the Rust arm equal to the C++, vertices bit for bit and faces, on 40 bumpy spheres decimated to a half to a fifth. On the fixtures it is not equal: with the Rust arm the scores fall to 175,533 / 225,436 / 2,803,285, the seams moving with the other mesh. So the arm is opt-in (`SEC_RUST=1`) and the shim stays; the default output is byte-identical to R876's. Next for this item: put the Benchy mesh into the test and compare stage by stage (the shim also exports the per-vertex dot products, `min_vertex_dots`), starting with `eigen_dot3f` and the `acos` of `indice_angle`. | `SEC_RUST` (opt-in) | 187,343, unchanged | 240,533, unchanged | 2,832,318, unchanged |
| R878 | `SAMPLE_RUST` retired: `sample_its_uniform_parallel` has no shim arm, R876 having shown the three G-code files byte-identical with the gate on and off. They are byte-identical to R877's. `eigen-transform-sys` functions called by the engine: `short_edge_collapse` (its Rust arm is opt-in, R877) and `raycast_visibility_native`; `sample_uniform` is called by its test only. Shim functions the engine calls: 30. | none (one retired) | 187,343, unchanged | 240,533, unchanged | 2,832,318, unchanged |
| R879 | From the checklist, a unit with no Rust file: `GCode/BedExcludeChecker` (.cpp 198 lines, .hpp 21). `gcode/bed_exclude_checker.rs` ports it line for line: `scaled_point`, `is_valid_extrusion_move`, `visit_extrusion_segments` (the arc's interpolation points included), `clip_segment_to_bbox` (Liang-Barsky, the clipped ends truncated), `segment_intersects_exclude_grid` (an `EdgeGrid` of the exclude polygons at 2 mm, visited with `need_consider_eps`), `toolpath_intersects_bed_exclude_area_2d` and `toolpath_exceeds_boundary_2d` (the `#if 1` arm; the brute-force arm kept for timing in the C++ is not ported). `MoveVertex` gains `is_arc_move_with_interpolation_points` (GCodeProcessor.hpp:239-241). The open-path difference of the concave case goes through `diff_pl_clib`, so through the ClipperLib shim. Its callers in C++ are in the GUI (`PartPlate.cpp`, `BaseRenderer.cpp`), not in `libslic3r`, so nothing in the engine calls it and no output changes; two unit tests cover a path through an exclude area and a path leaving a boundary. Units without a Rust file: 33 → 32. | none | 187,343, unchanged | 240,533, unchanged | 2,832,318, unchanged |

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
- R835, lines of the oracle's body the port has verbatim, before → after: Majora 2,730,277 → 2,750,295 of
  3,024,752 (90.9%); the Benchy jobs have no tower and are unchanged. Majora prime tower in order 95.8% → 96.4%.
- R835, the text between `; FilamentChangeGCodeEnd` and the `G4 S0` that ends the filament-start text is the
  oracle's in 2,685 of 2,723 tool changes (none before). Of the rest, 24 have other numbers, 7 differ only by
  an `M73` line (the time estimate, which the port does not write) and 7 otherwise, among them an `M204` in
  another place: the oracle's `travel_to_xyz` builds the XY move and the `travel_to_z` that follows before
  it adds `set_travel_acceleration()` (GCodeWriter.cpp:743), so the `M204` of the Z move lands between the
  two; and an `M204 S6000` the port writes before the first travel of the first layer's tool changes,
  which the oracle writes with the spiral lift of the entry retract (see the next item).
- R835, tried and left out: `_spiral_travel_to_z` begins with `set_travel_acceleration()` (GCodeWriter.cpp:800).
  Written so in the port it adds an `M204` before 656 `G17` lines where the oracle has 8: the port keeps
  `last_travel_accel` apart from the extrusion acceleration, the C++ writer one `m_last_acceleration`.
  Neutral on the scores; not committed.
- R835, the brim and chamfer loops are the oracle's on layers 1-7 (7, 5, 4, 3, 2, 1, 0 loops, the same
  coordinates). The first layer's brim starts at another corner and a chamfer loop carries an `F` the oracle
  drops: the port's finish of the layer still draws the fill first and is not merged into a tool change
  (`generate_new`, `finish_block`, `merge_tcr`).
- R835, the source map against R834: functions matched 3,692 → 3,700 (faithful 3,096 → 3,101, partial
  367 → 369, divergent 198 → 199), missing in the pair 4,692 → 4,684, Rust only 4,494 → 4,499.
  `gcode/wipe_tower.rs` matched 70 → 77.
- R836, Majora lines verbatim 2,750,295 → 2,752,858 of 3,024,752; prime tower in order 96.4% → 97.5%. The
  fill blocks after the last tool change of a layer are the oracle's text (layer 50: rectangle, grid, wipe).
- R836, what still differs in the tower, by layer: the timelapse wall at the head of every layer — its skip
  points (the oracle breaks the right side, the port the lower side), the travel to it (`append_tcr`'s
  `is_finish_first` travel through `GCode::travel_to`: extruder offset, unretract; the port writes a raw
  move), the writer's start (`only_generate_out_wall` starts at the fill box's upper corner of the full
  timelapse depth), `generate_path` writing the feedrate on the move, not on a `G1 F` line of its own;
  and the feedrates of the tool-change wipe (`G1 F4989.605` where the oracle has `G1 F5400`).
- R836, `GCode.cpp:4783-4800`: when the timelapse template moves Z, the oracle appends `G1 Z%.3f` with the Z
  from before it (656 lines on Majora); the port does not.
- R836, the source map against R835: functions matched 3,700 → 3,706 (faithful 3,101 → 3,102, partial
  369 → 374), missing in the pair 4,684 → 4,678, Rust only 4,499 → 4,505. `gcode/wipe_tower.rs` matched
  77 → 83 (`merge_tcr`, `finish_layer_new`, `finish_block`, `get_wall_filament_for_all_layer`,
  `get_wall_filament_for_layer`, `get_block_fill_filament`).
- R837, Majora lines verbatim 2,752,858 → 2,758,231 of 3,024,752; prime tower in order 97.5% → 98.5%. By step,
  in order: timelapse wall start, feedrate and inline F +2,263; `layer_z` with six digits and the Z restore
  +641 (656 `G1 Z%.3f` lines, the oracle's count); the travel to non-tool-change results +1,186; the
  template's feedrate +592.
- R837, left in the timelapse wall: the skip points. On the oracle's second layer the wall breaks on the
  right side at y 5.5, 11 and 16.5 (`layer_id % 4 == 1`, `process_depth + wipe_depth - gap`); the port's
  wall has split points around x 17.5 on the lower side and no gaps there.
- R837, the source map: headline unchanged.
- R838, Majora lines verbatim 2,758,231 → 2,769,131 of 3,024,752; prime tower in order 98.5% → 99.0%. By step,
  in order: the gaps +5,600; the simplification +1,184. Verbatim: the filament area +673, the empty
  line +3,443.
- R838, two tests that failed since they were written pass now: `wall_gap_geometry::gaps_actually_remove_length`
  and `two_skip_points_open_two_gaps`. `wipe_tower_transform::preserves_trailing_newline_structure`
  asserted the old end of the text and now asserts the C++ one. Suite: 84 failures (86 before), 1,979 pass.
- R838, left in the tower group: the wipe after a tower block (`G1 F5400`, the feedrate the oracle's text
  had last, where the port writes the cooled object speed), and how the entry wipe of a tool change shares
  its 1.9 mm of retraction along the path (`G1 X208.897 Y59.355 E-1.17414` against `E-.51398`).
- R838, the source map against R837: functions matched 3,706 → 3,707 (faithful 3,102 → 3,103), missing in
  the pair 4,678 → 4,677, Rust only 4,505 → 4,506 (`WipeTower::filament_area`).
- R839, Majora lines verbatim 2,769,131 → 2,774,529 of 3,024,752. The grid adds 5,396; the layer head adds
  655 `M104 M` lines and takes 656 empty lines out that matched empty lines elsewhere in the oracle (the
  verbatim count is a multiset), so it moves the verbatim count by 2 and the in-order score by 660.
- R839, the second layer's head is the oracle's but for `G1 E.8 F1800` against `G1 E.8`: the oracle's editor
  drops a feedrate the text already has, the port's writer decides it when writing E moves.
- R839, the source map against R838: functions matched 3,707 → 3,708 (faithful 3,103 → 3,104), missing in
  the pair 4,677 → 4,676, Rust only 4,506 → 4,508 (`WipeTower::set_last_layer_extruder_fill`).
- R840, Majora lines verbatim 2,774,529 → 2,779,643 of 3,024,752; prime tower in order 99.0% → 99.6%. The wipe
  speed adds 5,226 in order, the layer counter 209.
- R840, what is left in the tower group on a layer is a handful of lines: the first wipe segment where the
  last object path ends elsewhere (geometry), flush lines of the change-filament template (`G1 E5.41814`
  against `E5.41813`, and one pair of flush lines in another place) and the retraction after a wipe
  (`G1 E-.09999` against `E-.1`).
- R840, the source map: headline unchanged.
- R841, Majora lines verbatim 2,779,643 → 2,781,551 of 3,024,752 (the flush lines; the in-order score rounds to
  1e-3 and moves by 4).
- R841, how much of what is left differs only in the third decimal: Majora in order at 3 decimals 92.21%,
  at 2 decimals 93.81% (+45,884 lines), at 1 decimal 95.04%; Benchy arachne 92.95%, 93.93%, 95.80%.
  Outer-wall arcs that end at the oracle's point differ in 32,165 of 139k cases, almost all in the centre
  (I/J) by a micron or, on long nearly straight arcs, by more. The wipe of a tool change sums to 1.90001 in
  the oracle (`G1 E-.09999` after it, 187 lines) and to 1.9 in the port. (Corrected in R842: this item first
  put these down to a finer coordinate grid in C++; Bambu Studio's `SCALING_FACTOR` is 0.00001
  (libslic3r.h:58), the port's too, so they are differences of arithmetic still to be found.)
- R841, looked at and left, all following geometry: the bottom surface of Majora's first layer has the
  oracle's strokes in another order, because the wall before it ends at another point; bridges are the same
  round 0.4 mm extrusion on both sides where the oracle's E per mm varies along the bridge (R490/R491);
  `M204` lines differ by values that follow which paths exist (Benchy's line up); `; Slow Down Start/End`
  blocks differ by one to three per layer over many layers.
- R841, `M73 P R` (5,802 lines on Majora, 131 on Benchy) needs the processor's time estimate: the port
  estimates Benchy at 43m 27s against the oracle's 47m 31s and Majora at 2d 5h against 3d 23h 55m, so
  the estimator has to reach the oracle's before `TimeProcessor::post_process` (GCodeProcessor.cpp:571-) is
  worth porting.
- R841, the source map against R840: Rust files without a C++ name 48 → 47; functions matched 3,708 → 3,710
  (divergent 199 → 201), missing in the pair 4,676 → 4,674, Rust only 4,508 → 4,509.
- R842, lines verbatim, before → after: Benchy classic 187,738 → 187,844; Benchy arachne 232,419 → 232,655;
  Majora 2,781,551 → 2,786,734. `G1 E-.09999` on Majora 0 → 175 (oracle 187), `G1 E-.03999` 0 → 19 (21).
- R842, corrections. R838's row said `35.0f / 1e-6f`; Bambu Studio's `SCALING_FACTOR` is 0.00001
  (libslic3r.h:58), so it is `35.0f / 1e-5f`, which a float rounds to exactly 3500000 where the double
  quotient truncates to 3499999 — the fix stands, the arithmetic in the row was wrong. R841's finding put the
  third-decimal differences down to a finer grid in C++; the grids are the same, so they are differences of
  arithmetic. R842's wipe is one; another candidate is clang contracting `a*b + c` into fused
  multiply-adds on arm64 in the arc fitter's circle formula (Circle.cpp:29-43), which Rust never does —
  to be checked with a twin probe.
- R842, the source map: headline unchanged.
- R843, lines verbatim, before → after: Benchy classic 187,844 → 188,987 (96.2%); Benchy arachne 232,655 →
  234,065 (94.3%); Majora 2,786,734 → 2,800,654 (92.6%). The in-order score rounds E to 1e-3, so it moves little.
  Moves with the oracle's X and Y and E one unit off: Benchy 1,427 → 285, Majora 24,894 → 11,013; what is
  left is as often above as below (widths of variable-width paths), not one-sided.
- R843, the outer-wall arcs whose centre differs by a micron: the circle is taken through the path's first,
  middle and last points (Circle.cpp:57-95) and its centre rounded to an integer point, so these follow the
  path's points, not the circle arithmetic.
- R843, contraction applies wherever the C++ sums inexact products in double; the port has no general
  treatment of it. A place where a last-bit difference survives into the output is a candidate.
- R843, the source map: headline unchanged.
- R844, lines verbatim, before → after: Benchy classic 188,987 → 189,743 (96.56%); Benchy arachne 234,065 →
  234,761 (94.59%); Majora 2,800,654 → 2,801,056 (92.60%). The in-order score moves more than the verbatim
  count because the wipe and lift lines were already in the oracle, in another place.
- R844, left at the layer change: where the head's position is unknown after a custom block, C++ writes the
  `M204` of `_travel_to_z` between the XY move and the Z move (`travel_to_xyz` builds the string with the
  inner `_travel_to_z` before it calls the outer `set_travel_acceleration`), 5 times on Majora. The spiral
  of an eager lift tries four directions to stay on the bed (GCodeWriter.cpp `spiral_arc_within_bed`); the
  port always takes +X, which is the oracle's on every layer of the fixtures.
- R844, the source map against R843: `GCode::change_layer` divergent (17%) → partial (30%); estimate
  divergent 201 → 200, partial 374 → 375. Suite: 84 failures, the same as R843; 1,980 pass.
- R845, lines verbatim, before → after: Benchy classic 189,743 → 189,787; Benchy arachne 234,761 → 235,040;
  Majora 2,801,056 → 2,802,824. Empty lines the oracle has and the port not, Majora 1,129 → 396.
- R845, Majora's `M204 S10000` was already 41 over the oracle's count outside the overhang walls (215 over
  outside the contexts this round touched); `S2000` is 810 against 632 and `S6000` 0 against 111. Overhang
  walls: 1,122 features against the oracle's 1,276 (detection, geometry).
- R845, correction: R844's `change_layer` comment cited `to_lift_type` at GCode.cpp:8370-8383; it is at
  :8348-8361.
- R845, the source map against R844: functions matched 3,710 → 3,711 (faithful 3,104 → 3,105), missing in the
  pair 4,674 → 4,673, Rust only 4,509 → 4,511 (`lift_type_code`, `loop_path_acceleration`). Suite: 84 failures, the same as R844.
- R846, lines verbatim: Benchy classic 189,787 and Benchy arachne 235,040 unchanged; Majora 2,802,824 →
  2,802,827 (the `M204` before a spiral lift: 8 on Majora, as in the oracle). The header is not in the body.
- R846, the header's time line, port against oracle: Benchy classic model 42m 46s / total 47m 39s against
  42m 13s / 47m 31s; Benchy arachne 42m 55s / 47m 48s against 42m 36s / 47m 53s; Majora 2d 4h 58m / 2d 5h 5m
  against 3d 23h 48m / 3d 23h 55m. The port's processor over the oracle's own Benchy G-code: total 2,835.8 s
  against 2,851 s, prepare 293.3 s against 318 s. Majora's gap is the time of its 2,723 tool changes:
  `process_filament_change` (GCodeProcessor.cpp:6054-6125) adds unload plus load time (28 s + 29 s) to a
  `Tool_change` block per change, about 43 h; it needs the nozzle group result and the nozzle status
  recorder (`MultiNozzleUtils`), which the port does not have yet, and `process_T` is a no-op.
- R846, `M73 P R` needs `TimeProcessor::post_process` (GCodeProcessor.cpp:571-, missing in the port), and the
  processor's time per layer to be the oracle's first.
- R846, `PrintConfig::set_deserialize` hands only a vetted list of keys to `apply_key_value`; the machine limits
  (`machine_max_acceleration_*`, `machine_max_speed_*`, `machine_max_jerk_*`) are read only there, so the
  processor runs with their defaults. To check against the oracle's model time next.
- R846, the source map against R845: functions matched 3,711 → 3,717 (faithful 3,105 → 3,113), stubs 31 → 30,
  divergent 200 → 198, missing in the pair 4,673 → 4,667; `GCode/GCodeProcessor` faithful 83 → 91,
  `process_tags` stub → partial. Suite: 84 failures, the same as R845.
- R847, the header's time line, port against oracle: Benchy classic model 42m 46s / total 48m 5s against 42m 13s /
  47m 31s; Benchy arachne 42m 55s / 48m 14s against 42m 36s / 47m 53s; Majora 3d 23h 52m 49s / 4d 0h 0m 15s
  against 3d 23h 48m 22s / 3d 23h 55m 47s (was 2d 5h). The prepare time is the oracle's to a second on Benchy
  (5m 19s against 5m 18s); what is left is model time, +33 s on Benchy, +4.5 min on Majora. Only the header's
  time line changes; the body is R846's byte for byte.
- R847, the source map against R846: functions matched 3,717 → 3,723 (faithful 3,113 → 3,120), stubs 30 → 29,
  divergent 198 → 196, missing in the pair 4,667 → 4,661; `GCode/GCodeProcessor` faithful 91 → 97,
  `MultiNozzleUtils` 38 → 39. Suite: 84 failures, the same as R846.
- R848, the header's time line: Benchy classic 42m 44s / 48m 2s (was 42m 46s / 48m 5s; oracle 42m 13s / 47m 31s),
  Benchy arachne 42m 52s / 48m 11s; Majora unchanged (its limits are the defaults: 500 mm/s, travel 9000, which
  another path already read). What is left: about +8 s on the oracle's own Benchy G-code, +31 s on the port's,
  so most of the rest follows the G-code's differences (slow-down blocks, extra wipes and travels).
- R848, the source map against R847: stubs 29 → 25, faithful 3,120 → 3,121, partial 378 → 381;
  `GCode/GCodeProcessor` stubs 7 → 3. Suite: 84 failures, the same as R847.
- R849, lines verbatim, before → after: Benchy classic 189,787 unchanged; Benchy arachne 235,040 → 241,689 (97.38%);
  Majora 2,802,827 → 2,871,150 (94.92%). Majora outer wall in order 89.7% → 95.6%; Benchy arachne walls 98.7% and
  98.8%. The split-point truncation alone gives +6,340 / +58,619 in order, the offset +71 / +2,281 on top of it.
- R849, the `PP` probe on Majora after the round: perimeters agree on 121 of 656 layers (79 before). In the probed
  region every point's distance, degree and mapped degree now equal the oracle's to the bit. What is left there:
  the open-path clip of a wall against the grown lower slices (`clip_extrusion`, ClipperLib_Z) keeps a few points
  the port's clip does not (a zero-length spike on the boundary), and the bridge role of some overhang pieces
  (`detect_brigde_wall_arachne`) follows from that.
- R849, looked at and left: contracting `low_idx * (1 - t) + t * high_idx` (:265) and the sampling interpolation
  (:130-132) into fused multiply-adds changed nothing measurable here, so neither is in the port.
- R849, the README's Majora summary is brought up to this round's `PP` run.
- R849, the source map against R848: Rust only 4,515 → 4,517 (the `OVD` probe's helpers); headline otherwise unchanged. Suite: 84 failures, the same as R848.
- R850, lines verbatim: Benchy classic 189,787 and Benchy arachne 241,689 unchanged; Majora 2,871,150 → 2,884,786
  (95.37%). Majora outer wall in order 95.6% → 96.3%, inner wall 95.1% → 95.8%. The origin carries +14,361 in order,
  the bbox clip +204.
- R850, what is left on Majora's walls, sampled over 31 layers: no longer last-digit X or speed differences, but
  blocks of different length (paths split differently), travel moves and wipes. Floating vertical shell 82.9%,
  top surface 69%, bridges 57%, bottom surface 18% are the largest losses by share.
- R850, the instance shift is recorded on the 3MF path that carries the paint frame (Majora's); a 3MF without it
  keeps the old origin.
- R850, the source map: headline unchanged. Suite: 84 failures, the same as R849.
- R851, lines verbatim: Benchy classic 189,787 and Benchy arachne 241,689 unchanged (their sparse infill is grid);
  Majora 2,884,786 → 2,905,284 (96.05%). Majora floating vertical shell in order 82.9% → 90.3%, bridges 57.0% → 92.5%
  (the same strokes, now in the oracle's place), prime tower 99.6% → 99.8%.
- R851, the `FFC` probe after the round: for the same fill expolygon the anchor polygons are within 0.01% to 2% of the
  oracle's area but not equal (7 polygons of 8.92e11 against 9 of 8.74e11), and the unsupported areas differ in the
  fourth digit (4.107866e12 against 4.107637e12); 15 of 5,157 calls receive the oracle's inputs exactly. Next: the
  polyline offset (`offset(Polylines, float delta)` is ClipperLib with a square join on scaled integers; the port
  offsets in millimetres through another backend) and the union and shrink of the lower layer's internal surfaces.
- R851, the source map: headline unchanged. Suite: 84 failures, the same as R850.
- R852, lines verbatim, before → after: Benchy classic 189,787 → 189,897 (96.64%); Benchy arachne 241,689 → 241,808
  (97.43%); Majora 2,905,284 → 2,910,852 (96.23%). Of the oracle's `M73 P R` lines the port now has all but 21 of 136
  on Benchy classic, 11 on Benchy arachne and 234 of 5,802 on Majora; the rest differ by a minute or a percent where
  the estimates differ (Benchy 48m 2s against 47m 31s, Majora 4d 0h 0m against 3d 23h 55m).
- R852, the in-order score does not match `M73` lines but counts them in the body, so the percentage of the port's
  body falls a little while the matched count rises: Benchy classic 95.96% → 95.89% (183,740 of 191,616), Benchy
  arachne 97.18% → 97.13%, Majora 96.25% → 96.06%. The oracle's body has the same lines.
- R852, the source map against R851: functions matched 3,723 → 3,724 (`TimeProcessor::post_process`, which the
  map rates divergent, 196 → 197: the body is the M73 part of a function of 730 lines), missing in the pair
  4,661 → 4,660.
- R853, open in the estimator after the three fixes, from the Benchy replay: 251
  blocks differ in the cruise feedrate alone (242.84 against 242.72 on a 0.035 mm
  move, so no time moves with it); the last block's exit feedrate is 2.5 in C++
  and 3.0 in the port; the per-line elapsed times agree to 0.0006 s, which is two
  float steps at 2851 s and puts 4 of the replay's 133 `M73` lines one move early
  or late. The replay does not write the first `M73 P0` and the last `M73 P100`.
- R853: `seq_parity.py` counts `M73` lines in the port's body and matches the ones
  whose text and place agree, so a change of the estimator moves the in-order
  count by a few lines in either direction while the moves are byte for byte the
  same. Compare bodies without `M73` lines before reading such a change as a gain
  or a loss.
- R854: the estimator's open items of R853 are closed (see the round). What is left of the `M73` lines on
  the port's own G-code is the moves themselves: the port's Benchy takes 47m 54s against the oracle's 47m 31s,
  arachne 48m 3s against 47m 53s, Majora 3d 23h 57m 7s against 3d 23h 55m 47s, each from the 3 to 4% of lines
  that still differ. `process_VG1` has the same single-against-double arithmetic as `process_G1` and is not
  touched: no fixture writes `;VG1` lines.
- R854, a pattern: a float expression that mixes in an `AxisCoords` element, `M_PI`, `PI` or a `0.5` literal is
  evaluated in double and rounded once; an unqualified `cos`/`sqrt` of a float is the float overload on libc++.
- R855, Majora layer by layer against the kept oracle (lines of the port's body the oracle does not have, as
  written): Floating vertical shell 27,692 before the round, then Outer wall 19,529, Inner wall 11,140, Top
  surface 5,083, Sparse infill 2,139, Internal solid infill 1,349, Prime tower 1,219, Bridge 1,126, Bottom
  surface 1,117. The wall losses are spread: half of the Outer wall loss is in 133 layers.
- R855, Bottom surface (17.9%) is one fill on layer 0. `MONO=1` prints the monotonic regions and the chosen
  path of every `chain_monotonic_regions` call (C++ twin: one `fprintf` after each of the two calls in
  `FillRectilinear.cpp`). On Benchy the path is the same in every call whose regions are the same (19 of 80;
  78 have the same topology and differ in `len1`/`len2`, the float lengths of
  `montonous_region_path_length`). On Majora the region boundaries differ by one scan line because the
  layer-0 slices do, and layer 0 is one of the layers the oracle's segmentation does not reproduce. Five
  oracle runs agree with each other on the Bottom surface to 99.2% and more, so the port's difference there
  is its own: open.
- R855, `seq_parity.py` of one oracle run against another: 99.87% and 99.92% in order on Majora. That is the
  ceiling of the Majora score against one kept sample.
- R856: `GEO_SCALE_COORDT=1` (the geo backend on the coord_t grid) priced the class before the round at 0
  lines on Benchy classic, +1,103 on arachne and +347 on Majora; the exact ports gave 0, +1,408 and +477. A
  finer grid is not the same computation as ClipperLib, so the pricing run is a lower bound.
- R856: the helpers of `clipper_utils.rs` without `_clib` in the name are the geo backend unless a routing
  gate sends them to ClipperLib (`union_ex`, `union_polygons_ex`, `intersection`, `difference` are routed;
  `offset_*`, `shrink`, `grow`, `union`, `xor`, `offset_polyline*` are not). A new fixture should be run with
  `GEOCALLS=1` first: every line it prints is a call site to port.
