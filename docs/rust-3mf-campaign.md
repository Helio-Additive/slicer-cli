# Rust 3MF campaign — Arachne infill, negative parts, multi-color

## H — CONVERGENCE LOOP  [STATUS: active /loop, started 2026-07-27]

Charter (user): iterate until rust produces pretty much exactly the same
G-Code as C++ AND similar execution time. Fixtures: Benchy (stl-inline) +
Majora (nu3mf) ONLY. Scoreboard lives here — update every round.

| metric                     | current           | target        |
|----------------------------|-------------------|---------------|
| benchy diff-lines          | 139,717 (R385)    | ~0 structural |
| benchy semantic            | EQUIVALENT (R385) | keep          |
| benchy semantic material   | 0.9972 PASS (R385)| within 1%     |
| benchy silhouette          | 99.83%            | >=99.9%       |
| benchy rust time           | 2.63s (R385)      | bambu 2.32s — AT PARITY (1.13x) |
| majora rust time           | 44.2s (R385)      | bambu 15.5s (2.8x; Tier-1 vs full MC, not comparable) |
| majora semantic            | blocked on Tier-2 (wipe tower/ToolOrdering) | EQUIVALENT |

NOTE (R385 correction): the 12.6s/90min figures were STALE — R382's
faithful-gate default-on had already routed most clipper to integer.
Benchy is seam-placer bound now (~60% raycast_visibility), not clipper.

Known ceiling: perfect byte-parity is blocked by the compiler-FP wall
(R324-326 proof); "pretty much exactly" = eliminate all STRUCTURAL diff
classes, leave only sub-ULP FP scatter.

Work queue (leverage order):
H1 = task #17 double-bridge bottom-shell fix (material FAIL + diff class). DONE R383.
H1b = task #19 real-bridge over-widening. DONE R384 (benchy now EQUIVALENT).
H1c = anchor-polyline over-segmentation (rust anchors_pts 134 vs bambu 16 at
      z=37.8; skews determine_bridging_angle histogram → rust π vs bambu 3π/4
      → z=37.8 bridge 226.67 vs 115.30, ~1.97x). Root upstream of
      bridge_over_infill (generate_sparse_infill_polylines_for_anchoring or
      intersection_pl fragmentation). MINOR — semantic already passes.
H2 = task #18 frame-gate generalization (benchy −36k, multicolour-safe).
     DESIGN DONE (R384-era, vshell-hunter analysis; implement after H3):
     * General transform = C++ `trafo_centered() * volume.get_matrix()`
       (PrintObjectSlice.cpp:1395,:60; Print.hpp:375-376 trafo_centered =
       trafo() pretranslated by -unscale(m_center_offset).xy, Z=0;
       PrintObject.cpp:88 center = instance-transformed bbox XY center;
       applied f32 per-vertex, TriangleMeshSlicer.cpp:1827-1861). The benchy
       hardcode (Z+24/voff 0.8245) is just this chain evaluated for benchy —
       Z terms cancel; the load-bearing part is the f32 store-centered/
       place-back quantization (must stay on the Eigen FFI shim, R85 1-ULP).
       STL interim: derive voff = mesh bbox center from compute_bounding_box
       (no hardcode); real chain arrives with G-pipeline (#16).
     * center_offset→MMS thread: add scaled `center_offset: Point` param to
       multi_material_segmentation_by_painting_tier1 (mms.rs:2739), apply
       `line_to_test.translate(-center_offset)` at :2899-2903 (C++ MMS.cpp:
       2291; painted facets transformed by trafo()*get_matrix() at :2233,
       :2245), caller print_object.rs:624 passes Point::new_scale of
       slice_center_offset (set :464-474); update tests/mms_by_painting.rs:50.
     * COUPLING: slice mesh + painted mesh MUST get the identical frame or
       MMU silently dies (painted lines miss slices → 0 toolchanges).
       painted_cube_e2e (>=10 T1) is the guard. Sequence: (i) thread
       center_offset (gate-off = byte-identical), (ii) generalize transform
       for BOTH meshes, (iii) default-ON both gates, (iv) 3MF/multi-volume
       after G-pipeline. Risks: f32 fidelity (pure-f64 loses the floor-hole
       fix), units (m_center_offset is SCALED, new_scale truncs), per-volume
       matrices for multi-volume painted objects, instance shift
       (PrintObject.cpp:108) + gcode origin (print.rs:362-368) consistency.
H3 = campaign E integer-clipper routing (majora 10-50x, benchy ~2-5x).
H4 = G1-G6 main.cpp pipeline mirror (config parity; tasks #12-16).
H5 = re-census remaining diff classes post H1-H4; iterate.

Round log:
- R385 (H3/campaign E): CLIPPER_INT umbrella gate (default-on) routes
  union_safety_offset_ex(+_expolygons) and intersection(ExPolygons) through
  vendored integer ClipperLib (cz_union_ex_safety / cz_intersection_closed),
  matching ClipperUtils.hpp:372/ClipperUtils.cpp:803 exactly at 10nm; old
  geo path (1µm re-grid + unfaithful shrink-back) behind CLIPPER_INT=0.
  Profile-driven: Majora was 36% in geo execute_offset_operation via
  union_safety_offset_ex (bridge_over_infill); benchy has ZERO clipper in
  profile (seam-placer bound). Gates: semantic EQUIVALENT (material 0.9972,
  slightly better), diff-lines 139,137→139,717 (+580 precision churn),
  benchy 2.63s ≈ geo 2.67s, majora 53.85s→44.22s wall (1.29x CPU), suites
  green. Deferred (documented in clipper_utils.rs): offset_expolygons
  family (~40 byte-tuned call sites, little perf left), union_polygons_ex
  (entangled with F1_UNION=0 fallback), xor (no shim), open-path ops.
- R384 (H1b): BENCHY SEMANTICALLY EQUIVALENT — first time all checks pass.
  Root cause of ~1.8x bridge over-widening: construct_anchored_polygon
  (print_object.rs, port of PrintObject.cpp:2584-2752) had BOTH upper_bound
  predicates negated in the section-anchor extension (PrintObject.cpp:
  2637-2653) — `if !(section.a.y > ai.y)` picked the nearest anchor on the
  WRONG side (above instead of below and vice versa), extending bridge
  sections the wrong way. worth_bridging candidates verified byte-identical
  between engines (z=14.2: 18.84 = 18.84); divergence was purely anchoring
  (260.66 vs 103.65 post-anchor at z=14.2). Fix: drop both `!` (+14/-3).
  Gates: z=5.8 bridge 840→459.03 (bambu 459.35), z=14.2 383→218.28 (210.32),
  z=37.8 539→226.67 (115.30, residual = H1c); material 1.0147→0.9969 PASS;
  Bridge E-ratio 1.259→0.971; per-layer mean 0.78%, max dev 80.99→18.79%;
  diff-lines 140,297→139,137; suites green. Verdict: SEMANTICALLY EQUIVALENT.
- R383 (H1): phantom-bridge root cause was NOT discover_vertical_shells /
  bridge_over_infill (both at parity). region_expansion.rs
  process_external_surfaces_wave had an early `continue` on layers with no
  top/bottom/bridge surfaces, skipping the minimum_sparse_infill_area
  sparse→solid promotion (LayerRegion.cpp:597-614; C++ 518-640 has no such
  early-out). idx71 kept an 8.2mm² sparse island (rust ISI 83.15mm² vs bambu
  91.36mm²) → idx72 read as unsupported → phantom internal bridge at z=14.6.
  Fix: remove the early-return (one file, +9/-14). Gates: phantom gone
  (z=14.6 Bridge 17.6mm→0; z=14.2 real bridge stays), per-feature Bridge
  1.353→1.259 (check FLIPPED to PASS), ISI 0.975→1.002, vshell 1.068→1.044,
  material 1.0186→1.0147 (still FAIL), diff-lines 140,621→140,297 (fresh
  baseline), suites green (arachne 1, mms 1, painted_cube 1, 3mf 7).
  Caveat: z=37.8 locally worse (147→539mm, redistribution vs unfixed H1b
  over-widening); per-layer MEAN 1.65% still PASS.
  Ops note: BambuStudio/libnoise submodule gitdirs went dangling (modules/
  missing under .git/worktrees/slicer-cli3) — repaired by re-init + pinned
  fetch in-place; both submodules verified clean at pinned SHAs.

## G — Mirror main.cpp's drive pipeline in src/*.rs  [STATUS: in progress, 2026-07-26]

User direction: the rust slice initiated via src/main.rs must include the
custom pipeline logic of libslic3r/bambustudio/main.cpp (the engineer's
harness) so rust behaves like the actual C++ slice. Gaps (main.cpp refs):
- G1: set_default_config (337-604, full BBS defaults FIRST) +
      ensure_vector_config_sizes (605-817).
- G2: config layering order — defaults → 3MF/bundle → machine → process →
      filament → CLI overrides (1208-1231, 1414-1430).
- G3: PresetBundle-style rebuild for 3MF (1017-1178): resolve presets named by
      printer/print/filament_settings_id via src/profiles.rs inheritance,
      full-config analog, overlay flat 3MF on top; fall back to flat.
- G4: vector padding to extruder_count + master_extruder_id clamp (1233-1314),
      plate filament_maps → filament_map/_2 (1320-1358), prime-tower
      auto-disable (1381-1412).
- G5: validate() port + plate selection/translation/seq-print (939-1015),
      set_BBL_Printer / set_plate_origin.
- G6: wire into src/commands.rs rust path (STL + 3MF), app_slice consumes the
      pipeline-built config instead of raw flat JSON.
Related in-tree change (uncommitted at campaign start): faithful_gate()
default-ON rewrite of the 14 byte-parity gates (150 call sites) — measured
gates-on diff vs bambu 105,396 lines vs 241,115 gates-off (metric:
diff -a | grep -c '^[<>]' on stl-inline benchy); semantic: silhouette 100.00%,
material 1.0188 (bridge over-detection — separate finding below).
★ Bridge finding (2026-07-26): rust lays a SECOND solid/bridge layer 2 layers
above real bridges (z=14.6 X[7.7,25.1]: rust 17.6mm Bridge where bambu keeps
sparse-only; both bridge at z=14.2). Bottom-shell accounting divergence
(discover_horizontal_shells count-above-bridge) — explains Bridge 1.326 +
vshell 1.119 vs ISI 0.959 signature. Fix candidate after G.

## D — Rayon parallelization  [STATUS: in progress, started 2026-07-23]

Goal: close the 349x nu3mf gap (info.json 2026-07-22_22-46-20: bambu 15.76s vs
rust 5506.9s). C++ runs per-layer stages under tbb::parallel_for; the port made
them sequential. CONSTRAINT (user): parallel code must stay visually close to
the C++ — map `tbb::parallel_for(blocked_range(0,n), λ)` to rayon
`(0..n).into_par_iter()` / `par_iter_mut()` at the same loop sites, keeping the
C++ line-ref comments. Determinism: order-indexed collects only (no reduction
reordering); gates = stl-inline rust byte-identical 3097916 + semantic parity +
all crate integration tests + painted-cube toolchanges.
Hot spots (measured): bridge_over_infill (dominant on Majora), prepare_infill
stages, make_perimeters, make_fills, detect_surfaces_type, MMS per-layer loop
(13s sequential — minor), slicer layer conversion. rayon already a dep
(triangle_set_sampling uses par_iter).
Round plan: D1 survey C++ parallel_for sites ↔ rust loops, convert
make_perimeters/make_fills/detect_surfaces + MMS layer loop; D2
bridge_over_infill; D3 measure nu3mf, iterate.

OUTCOME (2026-07-23, R378-R380): ALL major C++ tbb::parallel_for sites
converted to rayon in C++-mirroring form (make_perimeters, infill fills
two-phase, MMS layer loop, bridge_over_infill clusters via explicit
ownership partition, detect_surfaces_type two-phase + clipping pass).
Byte-identical maintained throughout (stl-inline 3097916 exact on every
conversion). Gains: stl-inline 14.57→12.58s (~14%); benchy-class models
parallelize across layers.
★ D3 FINDING — Majora wall ~89min vs 92min baseline (≈flat): its cost is
CONCENTRATED, not spread — bridge candidates stack vertically into few huge
clusters (cluster parallelism ≈ nil for this geometry) and the giant
fragmented-layer clipper ops run 1-core (observed 1.4-2.4 core average).
The 349x gap vs bambu (15.76s) is therefore dominated by the geo-clipper
float path (fixed scale 1000, re-gridding per op) vs C++ integer ClipperLib
+ nested TBB. NEXT CAMPAIGN (E): route hot clipper ops (union_safety_offset,
offset_expolygons, diff/intersect in bridge/infill paths) through the
integer `_clib` ClipperLib bindings that already exist in clipper_utils —
expected the true 10-50x lever. Optional E2: nested parallelism inside
per-cluster candidate processing.

Working file for the /loop campaign started 2026-07-21. Full diagnosis in
`~/.claude/plans/golden-seeking-newt.md`; condensed here so any session can resume.

Evidence base: `tests/.tmp/2026-07-21_08-31-23/nu3mf/{rust,bambu}.gcode`
(Majora's Mask 3MF, `tests/28_MAJORASMASK_FULLCOLOUR_Makerworld_1plate_multicol.3mf`).

## A — Arachne infill bug  [STATUS: DONE — R362]

Symptom: rust slice with `wall_generator=arachne` emits walls only — zero sparse /
solid / top / bottom / bridge fill (all `; FEATURE:` fill counts 0 vs bambu 2940/3835/534/…).

CONFIRMED root cause locus: Arachne perimeter path. Same mesh+config with only
`wall_generator→classic` restores fill (sparse 0→1361, solid 0→664, top 0→755).

Chain: `generate_arachne` (perimeter_generator.rs:2875) → `WallToolPaths::get_inner_contour()`
→ `add_infill_contour_for_arachne` → `result.infill_area` (perimeter_generator.rs:3201) →
`layer.rs:561-574` fills `fill_surfaces` SOLELY from it. Empty every layer ⇒ no fill anywhere.

Prime suspect: `WallToolPaths::separate_out_inner_contour`
(arachne/wall_tool_paths.rs:1187-1271) — unions inner contour with NON-ZERO winding
approximation instead of ClipperLib EVEN-ODD; flagged in-code `FIDELITY-NOTE(F1)` at
:1264-1268. Concave self-overlapping mask contours ⇒ collapse ⇒ empty inner contour.
(`add_infill_contour_for_arachne` offset math checked and ruled out.)

Plan:
1. Instrument `generate_arachne`: log `infill_area` len per layer; verify empty; check whether
   `get_inner_contour()` is empty before or after the union. (Temp logging, not shipped.)
2. Fix: even-odd union in `separate_out_inner_contour` matching C++ `ClipperLib::Union`
   (reuse/extend `clipper_utils.rs`; scope to this union only — not full F1 geo-clipper swap).
3. Regression test (integration target; lib test target pre-broken): concave fixture sliced
   with arachne asserts `; FEATURE: Sparse infill` present. Keep classic tests green.
4. Verify: `just slice-configs nu3mf` → rust sparse/solid counts non-zero, roughly track bambu.

## C — negative parts merged as positive  [STATUS: DONE — R361 (skip, not subtract)]

`parse_3mf_model_xml` (app_slice.rs) ignores object `type="other"` /
model_settings `subtype="negative_part"` — Majora's 7 connector volumes union as positive.
Fix: parse types; exclude negative volumes from the merged mesh (true boolean subtract needs
Tier-2 ModelVolume; excluding negatives is the Tier-1-honest step — note in doc comment).
Verify: slice nu3mf; connector regions no longer produce stray solids.

## B — multi-color  [STATUS: in progress, phased]

Tier-1 merges everything to one mesh/material; `; filament: 1` hardcoded (generator.rs:350);
`num_extruders=1` hardcoded (print.rs:1874-1906). Ported-but-unwired: MMU segmentation core
(multi_material_segmentation.rs, entry BLOCKED at :1695), wipe tower, tool ordering,
toolchange emitter `set_extruder()` (exporter.rs:2529, called only from #[cfg(test)]).
Faithful importer parses `slic3rpe:mmu_segmentation` (three_mf.rs:2627-2643) but drops it
(:3264-3270, no ModelVolume/FacetsAnnotation).

Dependency chain (C++ refs: PrintApply.cpp:1060 generate_print_object_regions,
MultiMaterialSegmentation.cpp, PrintObjectSlice.cpp:845 apply_mm_segmentation, ToolOrdering,
WipeTower):
1. [DONE R364] FacetsAnnotation ported (model.rs, Model.cpp:4267/4292 hex codec,
   wraps TriangleSelector SerializedData). Pragmatic reader captures per-triangle
   `paint_color`/`mmu_segmentation` attrs → Parsed3mfModel/Loaded3mf.mmu_facets
   (merged-mesh triangle order). slice_3mf_to_gcode logs painted-facet count.
   NOTE: skipped full ModelVolume — merged-mesh + parallel annotation is the
   Tier-1-consistent shape; revisit if layer 4 needs per-volume separation.
2. [DONE R365] Per-extruder PrintConfig — ADDITIVE vectors (filament_colours/
   filament_diameters/filament_densities; scalars stay = filament 0), populated
   by apply_filament_arrays from the raw settings JSON; num_filaments();
   `; filament: N` header + all_extruders() driven by it (N=1 single-material,
   byte-neutral there). Bin's own load_bambustudio_settings copy NOT updated
   (single-material; multicolour targets the library path).
3. [DONE R366] Painted-region generation: TriangleSelector::used_states()
   (painted extruder slots from the annotation), Print::install_painted_regions
   (PrintApply.cpp:1062-1078 shape, single-parent collapse), add_object now
   shares ALL print_regions into PrintObjectRegions. slice_3mf_to_gcode decodes
   mmu_facets → painting_extruders → installs regions. Layers still carry only
   LayerRegion 0 until layer 4 splits surfaces.
4. Unblock multi_material_segmentation_by_painting + port apply_mm_segmentation.
   ★ RESCOPED R6 (read the C++ body, MMS.cpp:2095-2400): the main path does NOT
   use slice_mesh_slabs! It projects each painted facet to a slice-plane LINE
   inline (~40 lines, MMS.cpp:2244-2311) and feeds PaintedLineVisitor → the
   PORTED chain (post_process_painted_lines → colorize_contours →
   has_layer_only_one_color → build_graph → remove_multiple_edges_in_vertices →
   extract_colored_segments(graph) → cut/merge_segmented_layers).
   MISSING pieces only:
   (a) [DONE R369=commit R368] fn build_graph + append_voronoi_vertices +
       is_edge_attach/connecting helpers + clip_(in)finite_edge +
       mark_processed — ported (agent-assisted), full chain callable;
       tests/mms_build_graph.rs green.
   (b) [DONE R369] by_painting_tier1 orchestrator ported (agent-assisted;
       sequential; FIDELITY-NOTEs: simplify steps omitted, EdgeGrid bbox from
       contours, no consider-eps band). Output = faithful merge shape
       [layer][num_extruders] 0-based (default color dropped by merge).
   (c) [DONE R369] apply_mm_segmentation_tier1 in print_object.rs (single-
       parent collapse); painted_submeshes extracted post-centering in
       app_slice. Painted LayerRegions now carry real surfaces.
   (d) top/bottom propagation (mmu_segmentation_top_and_bottom_layers) is the
       ONLY slice_mesh_slabs consumer → STUB empty for Tier-1 (horizontal
       painted-surface propagation missing; contour painting — the dominant
       Majora signal — unaffected). Port slabs later for fidelity.
5. [DONE R373] Region→tool via region config wall_filament in the layer emit.
6. [DONE R373 Tier-1] emit_layer_by_island extruder-major + exporter::set_extruder
   bare T-commands (~2 changes/layer on the painted cube; single-tool layers
   byte-identical — stl-inline 3097916 exact). NOT yet ported: WipeTower purging,
   full ToolOrdering (cross-layer optimization), filament start/end gcode.

STATUS after R372: layers 1-4 VALIDATED end-to-end on the committed
painted_cube.3mf fixture (<1s, `; filament: 2`, e2e test green). Majora
(8-colour mosaic, 377k painted facets): segmentation 13s/656 layers but
bridge_over_infill >50min on thousands of genuine painted islands →
reclassified as PERF milestone (sequential geo-clipper vs C++ TBB+integer
clipper), not a correctness gate. Runs were timeout-killed with exit code
laundered to 0 via caffeinate/timeout — always check for the gcode file, not
the exit code. Next: layers 5-6 against the painted-cube fixture.

Verify: nu3mf rust header `; filament: N>1`, toolchanges present, per-filament length list
comparable to bambu.

## Round log
- R1 (2026-07-21): campaign file created; starting A step 1 (instrument + pinpoint).
- R1 cont: ROOT CAUSE FOUND + FIXED. Not the even-odd union (F1 exonerated —
  instrumentation showed inner_contour EMPTY pre-union, and convex cube+arachne
  also had zero fill). Actual cause: `PolylineStitcher::stitch` for the
  VariableWidthLines instantiation was a BLOCKED stub in stitch_tool_paths —
  ST's open wall-contour segments were never chained/closed, so the 0-width
  marker loop stayed `is_closed=false` and separate_out_inner_contour dropped it.
  Fix: ported `stitch_extrusion` (PolylineStitcher.hpp:53-217) with an
  `ExtrusionPointIndex` grid element (endpoint copy, avoids PathsPointIndex
  genericity refactor); rewired stitch_tool_paths to call it. Cube+arachne:
  sparse 0→120, solid 0→5, top/bottom restored. Regression test
  `tests/arachne_infill.rs` + fixture `tests/data/cube_arachne_settings.json`
  (both green). three_mf_parse 3/3 green. Pending: Majora nu3mf verify (bg),
  bun suite classic/sha guard (bg). A status → verify.
- R2 (2026-07-21): A VERIFIED on Majora (rust: sparse 1337 / solid 547 / top 627 /
  bridge 276, 657 layers, 26MB — was 0/0/0/0 hollow 10MB). Bambu's higher counts
  = per-colour region splits (Tier-1 single-material expected delta). Bun suite
  9/9 (classic + sha locks safe). C implemented same round: type="other" objects
  skipped in parse_3mf_model_xml + regression (three_mf_parse 4/4). Committed
  R359-R363, pushed. CAUTION noted: a stale bambu gcode at tests/.tmp/nu3mf/
  nearly read as the rust result — always verify header engine identity.
  Next: B layer 1 (ModelVolume + FacetsAnnotation storage).
- R3 (2026-07-21): B layer 1 DONE (R364). FacetsAnnotation in model.rs;
  paint_color captured through parse_3mf_model_xml → Loaded3mf.mmu_facets;
  codec round-trip + capture tests (three_mf_parse 6/6); arachne_infill still
  green; helio CLI compiles. Next: B layer 2 — per-extruder PrintConfig
  (filament vectors, filament_colour, num_extruders from filament_diameter.len()),
  then `; filament: N` header + all_extruders().
- R4 (2026-07-21): B layer 2 DONE (R365, additive vectors — byte-neutral for
  single-material, verified stl-inline bytes identical with/without WIP).
  ★ BYTE-LOCK SUPERSEDED: discovered R362 moved the historical default lock
  147987/7adae05c → 147761 lines. Route: concentric/floating fills construct
  WallToolPaths even under classic walls → the now-real stitcher changes their
  output. VERDICT: accepted — the lock had frozen the stub's behavior; the
  official gate since R348 is semantic parity, and it PASSES post-R362:
  benchy EQUIVALENT (filament 0.9974, silhouette 99.77% — better than the
  99.29% at lock time), cube EQUIVALENT (1.0035, 100.00%). Also fixed R360
  fallout: test_semantic_parity.sh expected native.gcode, compare now writes
  bambu.gcode (gate silently SKIPped). Next: B layer 3 — painted-region
  generation (PrintApply.cpp:1060 → painted_regions, >1 region/object).
- R5 (2026-07-21): B layer 3 implemented (used_states + install_painted_regions
  + share-all-regions + 3MF wiring; three_mf_parse 7/7). Majora smoke running
  (decode 377k facets + regions installed, single-material toolpaths expected
  unchanged). Commit as R366 after smoke green. Layer 4 scoped: slice_mesh_slabs
  = 100-line body + ~1150 lines support (TriangleMeshSlicer.cpp:908-2158) —
  the deliberate-exclusion wall; plan = port slab machinery faithfully across
  rounds, then unblock multi_material_segmentation_by_painting (cpp:2095) and
  port apply_mm_segmentation (PrintObjectSlice.cpp:845).
- R5 cont (R366 committed+pushed): first Majora run PANICKED
  (print_object.rs:2233 — region loops index layer.regions()[region_id]);
  fixed by padding layers to num_printing_regions() with empty LayerRegions
  in PrintObject::slice. Re-smoke GREEN: '; filament: 8', 9 regions, 656
  layers (== bambu now), fill intact (sparse 1341/solid 557). PERF NOTE:
  Majora debug wall ~7→~28min with 9 regions (bridge_over_infill clipper
  hot; sampled JoinCommonEdges/FixupFirstLefts2) — revisit after layer 4.
  Next: layer 4 — port slab support machinery (TriangleMeshSlicer.cpp:908+:
  slice_facet_at_zs variant, make_slab_loops, slice_mesh_slabs body), then
  multi_material_segmentation_by_painting + apply_mm_segmentation.
