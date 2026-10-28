# Checkpoints

## Checkpoint 1 — 2026-10-28 20:50 — PASSED
Rule: stock Orca program builds and slices a U1 test model.
Result: build-orca built in 5 min (0 failures); work/models/cube20.stl sliced with "Snapmaker U1 (0.4 nozzle)" + "0.20 Standard @Snapmaker U1 (0.4 nozzle)" → work/out-orca/cube20.gcode (exit 0). Defaults seen: High Temp Plate, Generic ABS (matches the gap report).
Decision: continue to Phase 1.

## Checkpoint 2 — 2026-10-28 21:05 — PASSED
Rule: Snapmaker engine library compiles and the CLI has <=150 places to fix.
Result: engine library compiles fully (after D-004). Front end: 129 error lines at 97 distinct places in 6 files: main.cpp 77, cli_load_settings.cpp 44, desktop_presets.cpp 3, cli_assemble.cpp 3, cli_run_steps.cpp 1, layout_plan.cpp 1. Nearly all are Orca 2.4-era APIs the fork predates (filament maps, extruder/nozzle variants, gcode check results, wipe tower type, a few CLI error codes). Full list: decision-log/phase1-errors.txt.
Decision: continue to Phase 2.

## Checkpoint 3 — 2026-10-28 21:31 — PASSED
Rule: slicer_cli-snapmaker slices the single-colour and 4-colour U1 tests; Bambu and Orca programs still pass their tests.
Result: snapmaker build passes 8/8 U1 cases (work/u1-tests/out-snap). Orca: test_diagnostic_events + test_excluded_features pass (21 passed, 0 failed, 2 skipped). Bambu: not built yet; the frontend helper proved by preprocessing (unifdef) that Bambu/Orca code is unchanged; Bambu build is scheduled for Phase 5. Snapmaker build itself fails 10 regression cases (3 diagnostic-events, 3 legacy-token, 4 calibration); assigned for investigation.
Early Phase 4 result: vs Snapmaker Orca 2.4.1 own CLI — four_colour, four_colour_gep, four_colour_highflow, two_plate plate 1: IDENTICAL; two_plate plate 2: 1 move differs by 0.001 mm; mixed_filament: different (theirs no tower); single: their CLI segfaults on a flattened process JSON.
Decision: continue.

## Checkpoint 4 — 2026-10-28 22:56 — reported (no gate)
Parity vs Snapmaker Orca 2.4.1 own CLI, 10 G-code files from 9 U1 cases: 6 IDENTICAL (four_colour, four_colour_gep, four_colour_highflow, single_project, two-plate plate 1 x2). 2 differ by one coordinate (0.001 mm, plate 2 of the two-plate projects). 2 differ by decision: mixed_filament (D-142), bare STL (D-007). Independent review: Bambu/Orca unchanged (unifdef), 1 crash + 5 issues fixed (review-fixes), 4 decision items to QUESTIONS.md.

## Checkpoint 5 — 2026-10-28 22:56 — FINISHED
All three programs build (Snapmaker on CGAL 5.4, D-161/D-162). Tests: diagnostic-events 8/8 on all three; excluded-features snapmaker 21/0/2, orca 21/0/2, bambu 22/0/0; nozzle-map bambu 32/0; U1 suite 9/9; Orca U1 G-code identical to the pre-change baseline. Package pkg-local/release/*.tar.gz (112 MB) validated on an extracted copy (3 programs, clean-host smoke, CI package tests); packaged slicer_cli-snapmaker four_colour IDENTICAL to Snapmaker CLI. Nothing pushed (push URL disabled; 10 local commits).
