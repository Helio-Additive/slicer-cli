# libslic3r-rs Port Progress

**Goal:** faithful 1:1 C++→Rust port of all of libslic3r (Bun-style), file by file / function by
function, until the Rust engine produces **byte-identical G-code** to native BambuStudio — the
integration-test SHA (`tests/integration-tests.test.ts`: 3DBenchy, base64 job sha `f420aee5…`,
3MF job `6c0ed1df…`).

> Methodology: port faithfully bottom-up (primitives → slicing → paths → support → gcode →
> orchestration → formats); each file build-gated + adversarially fidelity-reviewed vs C++;
> regressions auto-revert. G-code SHA is the final **validation**, not the driver.

## Overall completion (faithful-symbol coverage)

_Baseline from the MAP ledger (`PARITY.json`), Σ ported / Σ total public symbols across 280 C++ files._

| Metric | Value |
|---|---|
| **Symbols ported** | **1 101 / 3 073 ≈ 35%** |
| **LOC-weighted** | **≈ 30%** of 243 785 C++ LOC |
| Files: done / partial / stub / missing | 68 / 117 / 87 / 8 (of 280) |
| FFF-relevant (excl. SLA resin, 0/107) | ≈ 37% |

> These are MAP estimates and currently **stale** (pre-cleanup + pre-fixes — real value is a bit
> higher after the recent ports). Re-MAP refreshes them; see "Cadence" below.

## By priority (criticality to the FFF slice→gcode path)

| Priority | Symbol coverage | Files done |
|---|---|---|
| critical | 390/1048 (37%) | 10/44 |
| high | 379/870 (43%) | 25/81 |
| medium | 138/429 (32%) | 13/58 |
| low | 194/726 (26%) | 20/97 |

## By area

| Area | Coverage | Files |
|---|---|---|
| Execution | 87% | 3 |
| Interlocking | 73% | 2 |
| Arachne | 64% | 26 |
| Format | 62% | 10 |
| Optimize | 60% | 3 |
| CSGMesh | 55% | 7 |
| Geometry | 50% | 10 |
| GCode | 40% | 17 |
| Algorithm | 40% | 1 |
| Fill | 38% | 18 |
| Support | 37% | 8 |
| (root) | 30% | 153 |
| SLA (resin — out of FFF scope) | 0% | 21 |
| Shape | 0% | 1 |

## Validation status (G-code vs native BambuStudio, same config)

- **Cube** (proxy, fast): moves within 6% of native (6 280 vs 6 655); G0/templates/header/coords matched.
- **Benchy** (the real SHA target): slices end-to-end (~20s), but Rust 129 594 moves / 4.52 MB vs
  native 54 837 / 3.02 MB (2.36×). Biggest gaps: arc-fitting on curves (497 vs 11 923 arcs),
  gap-infill (+35 446), solid-infill, missing bridge + floating-vertical-shell. → bytes far from SHA.

## How the % is computed

`Σ keySymbols.ported / Σ keySymbols.total` over `PARITY.json` records (symbol %), and
`Σ cppLoc·(ported/total) / Σ cppLoc` (LOC-weighted). Refresh by re-running the `libslic3r-parity`
workflow (`args:{phase:"map"}`) to recount per-file coverage, then regenerate this file.

## Cadence (built to run for weeks)

- Driver: `libslic3r-systematic-port` workflow over the dependency-ordered queue (next files via
  `args.files`). Per file: faithful full port → devbox build → fidelity review → keep/revert.
- After each batch: update this file's "Recent batches" log; every ~1–2 weeks (or N batches) re-MAP
  to true-up the %.
- Final gate: re-slice Benchy → compare SHA.

## True-up log

- **2026-05-31 (after 8 files):** Benchy re-slice unchanged — 172 141 lines / 129 594 moves / 4.52 MB vs ref 139 498 / 54 837 / 3.02 MB; byte-diff ~284 k; not byte-identical. **No regression.**
- **Key insight:** bottom-up foundational ports (primitives) raise *symbol coverage* but do **not** move the Benchy **SHA** until the *consuming* higher-level code (perimeter/fill/gcode/orchestration) is ported to use the faithful primitives. So % and SHA-distance are decoupled during the foundation phase — expect the SHA to start converging only once the mid/high tiers land.
- Full symbol re-MAP (the 2M-token workflow) deferred — too heavy to run every 8 files; will run it less frequently / on request.

## Recent batches

| Date | Batch | Files | Result |
|---|---|---|---|
| 2026-05-31 | batch #15 | CSGMeshCopy + IntersectionPoints + TriangleMeshAdapter (verify) | Verified faithful: CSGMeshCopy (shallow/deep copy + is_same), IntersectionPoints (brute-force get_intersections_* — TODO 'use AABBTreeLines' is PERF-only, non-divergent), TriangleMeshAdapter (trivial bare-mesh→CSGPart: Union/identity). Deferred SlicesToTriangleMesh (cap triangulation stubbed → blocked on Tesselate; +diff_ex TODO; 2D→3D, off FFF path). build green. |
| 2026-05-31 | batch #14 (back-to-back) | clipper.cpp + ModelToCSGMesh + 2 defers | clipper.cpp = done-by-replacement (build-shim compiling ClipperLib into Slic3r:: ns; Rust uses geo_clipper/clipper2 backend). ModelToCSGMesh: FIXED dropped transform composition (was combined=*trafo; now vol.matrix.then(trafo) = trafo*vol.matrix, ModelToCSGMesh.hpp:66); gap: volume splitting (its_split, deferred w/ TriangleMesh) — off benchy path. build green. Deferred PerformCSGMeshBooleans (CGAL/mcut MeshBoolean backend) + AABBTreeLines (2D-tree design mismatch vs C++ Tree<2> free-fn API). |
| 2026-05-31 | sched #13 | AABBTreeIndirect.hpp (14→16/16) | Ported is_any_triangle_in_radius + get_candidate_idxs (AABBTreeIndirect.hpp:822,882). get_candidate_idxs exact 1:1 bbox-traversal; is_any_triangle_in_radius result-faithful (closest-dist comparison vs C++ pruned recursion). build green. Rest (build/ray/squared_distance/closest_point) already present. |
| 2026-06-04 | FIX #4 | process_external_surfaces min_area units bug | Per-stage Top-count audit (with TOP_FILLS) localized the Top-dropper: detect 581 -> clip 42 -> process_external **2** (vshell preserves 42; process_external deletes them). Root: surface.rs process_external_surfaces had `min_area_scaled = min_area_mm2 * 1e12` but the crate SCALING_FACTOR is 1e5, so 1mm²=1e10 — the threshold was 100x too large (50mm² instead of 0.5mm²), deleting nearly every Top/Bottom surface. FIX: min_area_mm2 * SCALING_FACTOR². RESULT (default): filament 3742->3820 (0.97x->0.99x, closer to golden 3858), Top features 1->5. With TOP_FILLS: Top 2->40 (top now survives process_external, 42->38). TOP_FILLS still gated (filament 4428, vshell over-solidifies around top — next). build green. |
| 2026-06-04 | PORT (gated) | faithful discover_vertical_shells | Implemented PrintObject::discover_vertical_shells as a 1:1 port of C++ PrintObject.cpp:1739-2110 (single-region path): cache top/bottom from SLICES (flow*0.05) + holes from fill_expolygons, shell projection (windows + anchor case), trim, regularize w/ the erase-remove predicate, keep_types + reassign. Added SurfaceCollection::filter_by_types/keep_types (committed a87fedc). Enabling it REGRESSES (3742->6031, internal_solid 166->462) because it faithfully reads SLICES carrying the over-classified BottomBridge=603 (golden ~38) — divergent reimpl read post-clip fill_surfaces (=2) so masked it. Gated behind env VSHELL_FAITHFUL; default = divergent reimpl (3742.67, no regression). NEW ROOT: detect_surfaces_type over-classifies BottomBridge. Dependency chain now: detect bottom fix -> enable faithful vshell -> top coverage. |
| 2026-06-04 | FIX #3 | gap-fill medial-axis (dominant over-extrusion) | Per-feature E-volume analysis (rust 4807 vs golden 3849) found Gap infill was 1064.8 vs 230.6 = **4.62x over** (87% of total excess). Walls were ~1.05x (flow fine). Root cause: layer.rs traced each gap polygon's CONTOUR at full perimeter width (~2x length, full vs thin width) AND the max_gap_area filter was declared-but-unused. FIX: faithful port of PerimeterGenerator.cpp:1327-1364 — collapse gaps to the thin band via difference(opening_ex(gaps,min/2), offset2(gaps,max/2,max/2)), medial_axis(min,max) per region, then convert_thin_walls_to_extrusion_paths (variable width). RESULT: **filament 4807.50 -> 3742.67 (1.25x -> 0.97x)**, size 4.21MB -> 2.90MB (golden 3.02MB), Gap infill E 1064.8 -> 80.0. Made convert_thin_walls_to_extrusion_paths pub(crate). build green. NOTE: gap now slightly UNDER (80 vs golden 515) — masked in total by sparse over (922 vs 562); revisit when top-surface fix lands (top areas mis-filled as sparse). Still open: Top 1 vs 142, Bridge 0 vs 38, Floating shell 0. |
| 2026-06-04 | ATTEMPT (reverted) | top_fills port #1 | Threaded upper-layer slices into the perimeter pipeline (PerimeterConfig/LayerRegion.upper_slices + make_perimeters_with_neighbors + PrintObject call site — KEPT, inert) and ported C++ only_one_wall_top/top_fills (PerimeterGenerator.cpp:1116-1183 + 1407-1413) into generate_classic_one. REGRESSED: filament 4807->5098, Top 1->2. Two bugs: (a) top_fills geometry over-large (fill_expolygons bloated past slice; inner collapsed to ~0 on top layers — partial-top split wrong); (b) NEW FINDING: clip now KEEPS top on ~30 layers (up to 86%) but only 2 Top features emit → a DOWNSTREAM stage re-types kept stTop (discover_vertical_shells reimpl / process_external_surfaces). Full fix is multi-part. Reverted to known-good 4807.50 (top=1,sparse=198); TODO left at perimeter_generator.rs. Saved to memory. |
| 2026-06-04 | DIAGNOSIS | Top-surface 1-vs-142 root cause (workflow + empirical) | Approved discover_vertical_shells re-port; 3-agent Understand workflow + empirical instrumentation PROVED it would NOT fix Top surface. Real cause: `slices_to_fill_surfaces_clipped` (layer.rs:205) clips typed top/bottom slices by `fill_expolygons`, which (=perimeter-gen `result.infill_area=last`, perimeter_generator.rs:743) does NOT cover top/bottom SKIN regions (esp. thin sloped-prow top strips). C++ covers them via PerimeterGenerator top detection vs upper layer (`top_fills`, PerimeterGenerator.cpp:1409-1413) — UNPORTED (TODO perimeter_generator.rs:583). Winding ruled out (all CCW). DECISIVE EXPERIMENT (NOCLIP_SKIN): clip skin by own area → Top 1→119 (golden 142), confirming the clip deletes them; but bottom 6→278 + filament worse, so raw-unclip isn't the fix. Upper slices ARE available (all_lslices[idx+1] in PrintObject::make_perimeters; only lower currently threaded). FIX = port top_fills (thread upper into perimeter gen + diff-vs-upper top detection + union into infill_area). All scaffolding removed; tree green; filament 4807.50 unchanged. Saved to memory project_benchy_parity_gap. |
| 2026-06-02 | FIX #2 | ensure_vertical_shell_thickness gate (over-extrusion) | Found + fixed the DOMINANT over-extrusion via toggle experiment (skip vshell/hshell). Root cause: Rust ran BOTH discover_vertical_shells AND discover_horizontal_shells; C++ runs only ONE — `discover_horizontal_shells` does `if region_config.ensure_vertical_shell_thickness != evtDisabled continue;` (PrintObject.cpp:3398), and the C++ DEFAULT is evtEnabled (PrintConfig.cpp:1804), key absent from config → C++ skips horizontal shells. Rust lacked the field. FIX: added EnsureVerticalThicknessLevel enum + PrintRegionConfig field (default Enabled) + set_deserialize("ensure_vertical_shell_thickness") + the skip-guard. RESULT: filament 7433.88→**4807.50mm (1.93x→1.25x)**, Sparse infill 29→**198 (≈golden 193)** ✅. Removed temp debug scaffolding (FILL_DEBUG, SKIP_VSHELL/HSHELL). build green. Remaining 1.25x gap: Top surface 1 vs golden 142 (discover_vertical_shells is a divergent reimpl, under-classifies top) + residual flow/perimeter over-extrusion. |
| 2026-06-01 | MEASURE | native-vs-Rust 3DBenchy differential | Rust engine runs end-to-end (exit 0, 240 layers == golden 240). GAP: rust 4,517,956 B vs golden 3,022,221 B; **filament 7433.88mm vs 3858.97mm (1.93x over-extrusion)** = the #1 parity blocker. ROOT CAUSE (feature-tag breakdown): rust emits Sparse infill 29 vs 193, and ZERO Top surface (golden 142) / Floating vertical shell (122) / Bridge (38) → those regions over-filled as solid. = incomplete surface classification (prepare_fill_surfaces/process_external_surfaces/discover_* — the config-coupled fns the threading track targets). Also fixed: hardcoded gcode version string 02.05.01.52 → 02.06.00.51 (generator.rs:287, matches golden header). Tracking metric going forward: filament-length ratio → 1.0. build green. |
| 2026-06-01 | config-thread #1 | discover_horizontal_shells region config | First config-threading increment (approved track). PrintObject::discover_horizontal_shells now reads the real PrintRegionConfig (fetched per region_id via shared_regions, the established no-back-pointer pattern) instead of hardcodes: num_solid_layers = top/bottom_solid_layers (was 3/3), top/bottom_shell_thickness = top/bottom_solid_min_thickness (was 0.6/0.6), sparse_infill_density-zero check = fill_density (was 20.0; Rust stores 0-1 fraction, C++ percent — zero-check is unit-safe). G-code-affecting (solid shell layer counts). build green. STILL hardcoded in this fn: flow().scaled_width() margins (lines ~1539/1581) + ensure_vertical_shell_thickness (field absent from Rust PrintRegionConfig). |
| 2026-06-01 | review #14 | Layer.cpp (0→4/29) + 2 bug-fixes | Review-gated port of the tractable non-config group: FIXED Layer::empty() (was checking has_extrusions() — wrong semantic — now slices.is_empty(), Layer.cpp:25-32); FIXED layer_needs_raw_backup() (was false → now true matching C++, Layer.cpp:77-82); added get_extents_layer_region + get_extents_layer_regions (Layer.cpp:635-655, renamed for no-overloading). build green, all divergences info-level. 25/29 blocked on the Print→PrintObject→Layer→PrintRegion config hierarchy → user approved PIVOT to config-threading track (unblocks make_perimeters/merged/has_compatible_layer_regions/etc. + downstream files). |
| 2026-06-01 | review #13 | Slicing.cpp (3→11/21) + SlicingParams cleanup | Review-gated workflow ported the config-independent group: smooth_height_profile (gaussian blur+kernel), adjust_layer_height_profile (+LayerHeightEditActionType), adjust_layer_series_to_align_object_height, generate_object_layers, check_object_layers_fixed, HeightProfileSmoothingParams, object_print_z_height(). THEN (user-approved cleanup) made SlicingParams 1:1 with C++ SlicingParameters: dropped non-C++ fields mode/closing_radius/extra_offset/resolution (those are MeshSlicingParams) + legacy first_layer_height (→first_print_layer_height, 3 callers repointed), replaced is_valid() method with `valid:bool` field (Slicing.hpp:21), removed builder chain, rewrote equal_layering exact (==, full field set, debug_assert valid; Slicing.hpp:103-131). build green. Blocked-on-config: create_from_config, min/max_layer_height_from_nozzle, layer_height_profile_from_ranges/adaptive, generate_layer_height_texture. |
| 2026-05-31 | sched #12 | AABBMesh (verify) + TriangleMeshSlicer/TriangleMesh (defer) | VERIFIED AABBMesh: query_ray_hit/hits + squared_distance present & faithful (delegate to AABB tree intersect_ray/squared_distance_to_its = C++). Gap: filter_hits (hole/neg-volume ray entry/exit, SLA-adjacent, off FFF path). Deferred TriangleMeshSlicer (SHA-critical; needs dedicated faithful rewrite of slice_facet+triangle-connectivity chaining+close_gaps) and TriangleMesh (its_face_neighbors/edge_ids co-dependent with that rewrite; core ops present as methods). |
| 2026-05-31 | sched #11 | SliceCSGMesh.hpp (fix stub) + VoronoiUtilsCgal (defer) | FIXED: merge_slices Difference/Intersection were silent no-op stubs (marked '3/3 done' but wrong) — now call clipper_utils::difference/intersection (= C++ diff_ex/intersection_ex). collect_nonempty_indices + Union already faithful. build green. Deferred VoronoiUtilsCgal (CGAL exact-predicates/arrangement). |
| 2026-05-31 | sched #10 | ClipperUtils.cpp (verify) + MutablePolygon (defer) | VERIFIED: common boolean ops (union/intersection/difference) use NonZero via geo_clipper (matches C++ pftNonZero, 25 sites) — faithful. GAP flagged: C++ also uses pftEvenOdd/Positive/Negative (8 each); geo_clipper high-level API is NonZero-only → those specific variants diverge, need geo_clipper's fill-rule-parameterized API. Deferred MutablePolygon (linked-list/iterator structural port). |
| 2026-05-31 | sched #9 | Geometry/VoronoiUtils.cpp (discretize_parabola) | Ported discretize_parabola (VoronoiUtils.cpp:107-197) faithfully: exact integer-division parabola math, perp=(-y,x), rotate_by_cos_sin. build green, review-faithful (UNVERIFIED; pxx/norm rounding caveats noted). Remaining (get_source_*, compute_segment_cell_range) need boostvoronoi cell accessors + serve the deferred SkeletalTrapezoidation. |
| 2026-05-31 | sched #8 | Geometry/Voronoi.cpp (replacement) | Done-by-replacement: the boost::polygon voronoi_diagram wrapper + degeneracy-repair (detect_known_issues/try_to_repair) is N/A — Rust uses the boostvoronoi pure-Rust backend (the wasm-safe native-dep substitution); annotation in voronoi_annotation.rs. PARITY RISK: boostvoronoi may differ numerically from boost::polygon on Voronoi-dependent features (medial axis/gap fill). No code change. |
| 2026-05-31 | sched #7 | MultiPoint.cpp (1→~13/17) | Ported the transform/query group as free fns over &[Point] (no C++ inheritance): scale, scale_xy, translate, rotate(cos,sin), rotate(angle,center), length, find_point x2, bounding_box, has_duplicate_points, remove_duplicate_points, remove_colinear_points (MultiPoint.cpp:6-134). build green, review-faithful (UNVERIFIED). Remaining: intersection family, visivalingam, concave_hull_2d, symmetric_y, has_boundary_point. Deferred LineSegmentation (ClipperLib_Z, off benchy path). |
| 2026-05-31 | sched #6 | Geometry/MedialAxis.cpp (verify) | VERIFIED faithful: uses boostvoronoi (pure-Rust, wasm-safe) backend; validate_edge + process_edge_neighbors (twin/rot_next active-neighbor walk) + build chaining match MedialAxis.cpp. Adaptation: per-vertex widths vs C++ 2*(N-1) edge-end array (documented). Skipped dump_voronoi_to_svg (debug). NOT Voronoi-blocked — boostvoronoi already wired. No code change. |
| 2026-05-31 | in-session #5 | Geometry.cpp (defer) + ConvexHull.cpp (verify) | Deferred Geometry.cpp (Transformation/transform cluster needs dedicated nalgebra-backed port; 5 standalone helpers already done). VERIFIED ConvexHull faithful: convex_hull_points (Andrew monotone chain) is exact 1:1 (is_ccw sign matches Geometry::orient via cyclic equivalence). Known divergence: convex_polygons_intersect uses SAT vs C++ rotating-calipers (same result, revisit). No code change. |
| 2026-05-31 | in-session #4 | EdgeGrid.cpp (signed distance) | Ported signed_distance_edges + signed_distance (EdgeGrid.cpp:1178-1281) faithfully + added Contour::segment_prev. Exact cell-window edge iteration, convex/reflex vertex sign, on-segment flag. build green. Remaining: contours_simplified, intersecting_edges (resurface on re-MAP) |
| 2026-05-31 | in-session #3 | Geometry.cpp (helpers) + Line.cpp (finish) | Added to geometry/mod: directions_parallel, directions_perpendicular, rad2deg_dir, linint, liang_barsky_line_clipping (Geometry.cpp:29-73 + Geometry.hpp). **Completed Line.cpp**: parallel_to_angle, perpendicular_to_angle, clip_with_bbox now use them — Line.cpp fully faithful. VoronoiOffset deferred (Voronoi infra). build green |
| 2026-05-31 | in-session #2 | Line.cpp | **KEPT** faithful: +vector, perp_distance_to, orientation, parallel_to, perpendicular_to, overlap, extend, get_extents; fixed intersection_infinite to C++-exact (cross2+EPSILON+overflow). 3 left (parallel_to/perpendicular_to(angle), clip_with_bbox) pending Geometry.cpp. build green |
| 2026-05-30 | foundational #1 | ExtrusionEntity, Polyline, Circle | **Circle KEPT** (0→16/18 sym, fidelity .95); ExtrusionEntity & Polyline reverted (built but fidelity .35/.28 — workflow upgraded with 2-pass fix iteration to retry) |
