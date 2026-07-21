# Rust 3MF campaign — Arachne infill, negative parts, multi-color

Working file for the /loop campaign started 2026-07-21. Full diagnosis in
`~/.claude/plans/golden-seeking-newt.md`; condensed here so any session can resume.

Evidence base: `tests/.tmp/2026-07-21_08-31-23/nu3mf/{rust,bambu}.gcode`
(Majora's Mask 3MF, `tests/28_MAJORASMASK_FULLCOLOUR_Makerworld_1plate_multicol.3mf`).

## A — Arachne infill bug  [STATUS: in progress]

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

## C — negative parts merged as positive  [STATUS: todo]

`parse_3mf_model_xml` (app_slice.rs) ignores object `type="other"` /
model_settings `subtype="negative_part"` — Majora's 7 connector volumes union as positive.
Fix: parse types; exclude negative volumes from the merged mesh (true boolean subtract needs
Tier-2 ModelVolume; excluding negatives is the Tier-1-honest step — note in doc comment).
Verify: slice nu3mf; connector regions no longer produce stray solids.

## B — multi-color  [STATUS: todo, phased]

Tier-1 merges everything to one mesh/material; `; filament: 1` hardcoded (generator.rs:350);
`num_extruders=1` hardcoded (print.rs:1874-1906). Ported-but-unwired: MMU segmentation core
(multi_material_segmentation.rs, entry BLOCKED at :1695), wipe tower, tool ordering,
toolchange emitter `set_extruder()` (exporter.rs:2529, called only from #[cfg(test)]).
Faithful importer parses `slic3rpe:mmu_segmentation` (three_mf.rs:2627-2643) but drops it
(:3264-3270, no ModelVolume/FacetsAnnotation).

Dependency chain (C++ refs: PrintApply.cpp:1060 generate_print_object_regions,
MultiMaterialSegmentation.cpp, PrintObjectSlice.cpp:845 apply_mm_segmentation, ToolOrdering,
WipeTower):
1. ModelVolume + FacetsAnnotation storage; importer keeps segmentation strings.
2. Per-extruder PrintConfig (filament vectors, filament_colour, real num_extruders).
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
