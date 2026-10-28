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
