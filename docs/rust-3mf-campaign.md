# Rust 3MF campaign — Arachne infill, negative parts, multi-color

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
3. Painted-region generation (painted_regions; >1 region/object in print.rs:207-233).
4. slice_mesh_slabs port; unblock multi_material_segmentation_by_painting; port
   apply_mm_segmentation (print_object_slice.rs:40) to split LayerRegions per extruder.
5. Drive region_extruder (print_region.rs:246) from multiple regions.
6. Wire ToolOrdering + set_extruder + WipeTower into export_gcode.

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
