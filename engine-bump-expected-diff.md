# Engine bump: expected diff

This bump moves the Bambu Studio engine to the current public release. The
Orca and Snapmaker engines stay where they are (OrcaSlicer 2.5.0 was at
release candidate when this was written).

| engine | was | is |
|-|-|-|
| BambuStudio | `926a719` = v02.08.02.61 (2.8.2) | **`56a57d0`** = upstream **v02.09.01.70** (2.9.1, 3 Nov 2026) |
| OrcaSlicer | `8500fcd` = v2.4.2 | unchanged |
| Snapmaker Orca | `bac6fd5` = 2.4.1 | unchanged |

Upstream: 1,566 commits, 217 `libslic3r` files (+36,554 / -5,339), 53 setting
keys added, `nozzle_type` dropped, the Bambu Lab H3C (a four-slot nozzle
rack), mixed nozzle diameters, conformal infill, alternating-layer modifiers,
a separate sub-top-surface pattern, and a new mesh-boolean backend
(OpenMeshCraft). Every claim below names the run or the source line it comes
from. A version bump is expected to change G-code everywhere; this file
records what the change *is*, so a reviewer can tell expected drift from a
defect.

## Why now

A 3MF saved by Bambu Studio 2.9.1 carries a newer major.minor than the 2.8.2
engine, which refuses it (`CLI_FILE_VERSION_NOT_SUPPORTED`, -24) unless
`--allow-newer-file` is passed. Proven on the Linux box with a project the
2.9.1 build wrote (`--export-3mf` of a sliced cube, then `--slice 1` on each
binary; the suite's "a project written by this engine slices again" case
repeats the new-binary half):

| binary | result |
|-|-|
| 2.8.2, no flag | refused, rc 232 (-24), `FileVersionNewerThanEngine` |
| 2.8.2 with `--allow-newer-file` | rc 0 |
| 2.9.1 | rc 0 |

## What the front end had to change

Every item is a port of the official command line at `v02.09.01.70`
(`src/BambuStudio.cpp`) or of the desktop, cited in the code.

| Change | Where | Why |
|-|-|-|
| Boost.Log's header included before any engine header | `main.cpp` | 2.9.1's `libslic3r/Log.hpp` replaces `BOOST_LOG_TRIVIAL` with its own logger and occupies boost's include guard; the engine-log bridge still works, as `LogStream` hands every record to Boost.Log (`Log.cpp:300-313`) |
| A headless `LogSink.cpp` in the override layer | `libslic3r/bambustudio/libslic3r/LogSink.cpp` | `Log.cpp` links the desktop's log-file sink, which needs the desktop's HTTP client and AES code for encrypted log files; the copy writes plain text and the CLI never opens a log file; the three `LogSinkBackend` stubs `standalone_stubs.cpp` carried for the same reason are gone (MSVC refused the duplicate definitions; GNU ld had silently picked one) |
| `MeshBooleanOpenMeshCraft.cpp` excluded | `CMakeLists.txt` | the OpenMeshCraft backend needs GMP/MPFR and the OpenMeshCraft package; `MeshBoolean.cpp` keeps MCUT when `SLIC3R_ENABLE_OPENMESHCRAFT_BOOLEAN` is unset, as upstream's own `LINUX_CLI_COMPILE` does |
| `get_extruders(...)` takes the global feature filaments | `cli_assemble.cpp`, `main.cpp` | the plate's filament list passes `GlobalFeatureFilaments::from_full_config` and reads `wall_filament` / `sparse_infill_filament` / `solid_infill_filament` from layer ranges too, as `PartPlate::get_extruders_under_cli` does (`PartPlate.cpp 1321, 1336, 1343`); the assemble path passes none, as the official does (process config not loaded yet) |
| `--allow-mix-temp` has no engine hook on Bambu | `main.cpp` | 2.9.1 removed the plate-level high/low temperature mix check and `Print::set_check_multi_filaments_compatibility`; the official CLI parses the flag and no longer uses it |
| `Print::validate` returns a list of warnings | `main.cpp` | joined with newlines as the official does (`BambuStudio.cpp 7531-7536`), so `validation_warning` keeps its shape |
| Nozzle inventory on a printer pick | `desktop_presets.cpp` | `PresetBundle::extruder_nozzle_stat` is gone; the desktop calls `NozzleInventory::select_printer` (`Plater.cpp 2179`, `Tab.cpp 6155`), which writes `extruder_nozzle_stats(_new)` and the per-slot keys into the project config |
| Nozzle statistics for settings-file input | `main.cpp` | the official CLI's 2.9.1 route: Hybrid slots take the preset's per-slot defaults (`4544-4553`), the per-slot nozzle keys are derived from the counts when the input lacks them (`7170-7191`), for carried stats too (`extruder_nozzle_stats_new` read first, `4496, 4555-4559`): a 2.8.x H2D/H2C project carries the stats but none of the per-slot keys. Carried stats are dropped on a printer change, as the official does (`4530`) |
| Mixed nozzle diameters | `cli_nozzle_diameter.{cpp,hpp}` (new), `cli_load_settings.cpp`, `main.cpp` | `filament_diameter_map` filled from the printer when not given (`2673-2849`); `multi_nozzle_diameter_mode` gate (`3289-3401`); a single-diameter plate refused with -108 (`5066-5086`); `--downward-check` fails a printer that mounts fewer nozzles than the diameters used (`5218-5258`) |
| The system-preset `AppConfig` outlives the bundles | `main.cpp` | `load_presets` now stores a pointer to it as the nozzle inventory's history (`PresetBundle.cpp 580, 2382`); a local one left `select_printer` reading a dead object (an access violation on Windows, found on the Windows box; silent on Linux) |
| `FillFloatingConcentric.cpp` override regenerated | `libslic3r/bambustudio/libslic3r/Fill/` | upstream changed the file (Arachne thresholds follow the fill's nozzle); the override is the 2.9.1 copy with the five fixes re-applied, now with LF line endings like the engine file |

Not ported, on purpose:

- The per-slot nozzle keys given as flags (`extruder_nozzle_count`,
  `extruder_nozzle_volume_type`, `extruder_nozzle_diameter`,
  `extruder_nozzle_slot`, `ams_nozzle_idx`): the first two were already
  refused as flags (the engine derives them); the other three are now
  refused the same way (`BambuStudio.cpp 7104-7172`).
- The single-diameter plate refusal (-108) runs where slicer-cli checks a
  plate: before any plate is sliced for `--slice 0` (the pre-check pass
  visits every plate first), on the one plate for `--slice N`. The
  official CLI checks every plate of the project before slicing even when
  one plate is asked for (`5066-5086`).
- `check_preset` (the official's compatible_printers validation of loaded
  filaments): it existed at 2.8.2 and was not ported then either.
- The import/prepare/export timings the official CLI now records: nothing
  reads them.

## What stays the same

- The three engines' settings, flags and event shapes: every existing event
  keeps its kind and tag. New: `config_refused` / `PlateSingleNozzleDiameter`,
  `warning` / `PrinterNozzleCountTooSmall`, `warning` / `AllowMixTempIgnored`
  (Bambu build, when the flag is given), the `MultiNozzleDiameters` entry of
  `upward_compatibility_taint`, return code -108.
- PR #55's height-range fill: the unchecked `layer_height` read is still
  there at the tag (`Slicing.cpp 229, 251`).
- The Linux, macOS and Windows dependency pins (Boost 1.84, CGAL 5.4 with
  Bambu's patch, OpenCASCADE 7.6.0): the upstream deps changes are
  Windows-ARM64 and macOS-GMP work that this build does not use.

## Expected G-code drift

Old (2.8.2, package `fix55b`) against new (2.9.1, package `fix291`) on every
Bambu-printer project in the suite corpus, plate 1 (`--slice 1` with each
binary, `rc` and layer count read from the G-code header, times from
`result.json`):

| project printer | rc old / new | layers old / new | time old | time new |
|-|-|-|-|-|
| A1 mini 0.4 | 0 / 0 | 68 / 68 | 26m 59s | 26m 52s |
| A1 0.6 | 0 / 0 | 646 / 646 | 10h 55m 30s | 9h 59m 16s |
| X2D 0.4 | 0 / 0 | 1242 / 1242 | 1d 20h 31m | 1d 20h 30m |
| H2D 0.4 | 0 / 0 | 512 / 512 | 4h 29m 48s | 4h 36m 25s |
| P2S 0.4 | 0 / 0 | 799 / 799 | 2h 47m 5s | 2h 47m 13s |
| A1 mini 0.4 | 0 / 0 | 870 / 870 | 5h 52m 32s | 5h 49m 6s |
| P1S 0.4 | 0 / 0 | 236 / 236 | 2h 22m 57s | 2h 23m 48s |
| X1 Carbon 0.4 | 0 / 0 | 108 / 108 | 10h 31m 30s | 10h 34m 32s |

Every project slices on both engines with the same layer count. The time
moves because of upstream changes to the same input: the top-surface
monotonic travel optimisation and the sub-top-surface pattern default
(monotonic, was rectilinear) on every printer, and the multi-nozzle rework
(`PrintObject.cpp`, `ToolOrdering.cpp`, `WipeTower.cpp`, `TreeSupport.cpp`)
on the H2D. The largest moves are the A1 0.6 project (9% faster) and the H2D
project (2.5% slower).

On every project saved by 2.8.x the new engine reports `nozzle_type` as an
unknown key (`config_unknown_keys` / `UnrecognizedConfigKeys`): 2.9.1 ignores
the key on load (`PrintConfig.cpp 8482`).

## Test results

Both boxes ran the final head (the binaries carry the `AppConfig` lifetime fix and the removed LogSink stubs).

Linux box, all three engines built from this branch (Orca and Snapmaker on their unchanged pins):

| Check | Result |
|-|-|
| engine entry points (bambustu_main, orcaslicer_main, Snapmaker_Orca_main) | one each |
| `tests/test-packaged-slice.sh` (Bambu, Orca and Snapmaker packages; three of the six cases are the Snapmaker's) | rc 0, 6 PASS |
| `tests/test-complete-slice-path.sh` (takes the Bambu and Orca binaries; its catalog cases also look for the Snapmaker binary beside them) | rc 0, 325 PASS, 0 FAIL |
| `tests/test_list_settings.sh` on the Bambu package | PASS |

The four new Bambu cases all pass on Linux: re-slice of a project this engine wrote without `--allow-newer-file`, the Bambu Lab H3C on its named preset with nozzle statistics, conformal infill and the alternating-layer modifier as object settings, and the mixed-diameter block (map filled from the printer, mode off checks nothing, single-diameter map and plate refused with -108).

The mixed-diameter block also asserts that `--extruder-nozzle-diameter` is refused by name and that `--allow-mix-temp` reports `AllowMixTempIgnored`.

Windows box (VS 2022 x64, vcpkg, Bambu 2.9.1 build): the same cases pass, `FAILS=0`. The first Windows run crashed the named-preset route with an access violation; that run is what found the `AppConfig` lifetime bug.

The review-fix pass (plate filament list with the global feature filaments, per-slot derivation for carried stats, the taint entry, the refused per-slot flags, the `--allow-mix-temp` warning) was rebuilt and rerun on both boxes with the same results: Linux 325 PASS / 0 FAIL (the two suites above, over the three rebuilt engines), Windows `FAILS=0`. One regression surfaced and was fixed on the way: re-serialising carried `extruder_nozzle_stats` unconditionally (as the official does) rewrote a project's own value, so the write-back now happens only when the stats were computed or derived.
