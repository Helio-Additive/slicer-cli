# Engine bump: expected diff

This bump moves the OrcaSlicer engine to the current public release. The
Bambu Studio and Snapmaker Orca engines stay where they are: each is already
on its project's latest release.

| engine | was | is |
|-|-|-|
| OrcaSlicer | `8500fcd` = v2.4.2 | **`f92a7c0`** = upstream **v2.5.2** (21 Nov 2026) |
| BambuStudio | `56a57d0` = v02.09.01.70 (2.9.1) | unchanged |
| Snapmaker Orca | `bac6fd5` = 2.4.1 | unchanged |

Upstream: 3,704 commits, 582 `libslic3r` files (+99,521 / -15,297), and the
official command line `src/OrcaSlicer.cpp` itself +1,502 / -415 in 120 hunks.
OrcaSlicer 2.5 takes over Bambu Studio's multi-nozzle engine (nozzle
statistics, nozzle and volume maps, a nozzle rack), adds mixed filaments, belt
printers, bed exclusion volumes, a mesh repair step and eight command-line
flags. Every claim below names the run or the source line it comes from. A
version bump is expected to change G-code everywhere; this file records what
the change *is*, so a reviewer can tell expected drift from a defect.

## Why now

Two reasons.

1. The pins are meant to be on the latest release of each project.
2. OrcaSlicer 2.4.2 leaves two members of `Arachne::WallToolPathsParams`
   unset in its concentric fills (`FillConcentric.cpp`,
   `FillConcentricInternal.cpp`), and `WallToolPaths::removeSmallLines` reads
   them. Leftover stack bytes then decide which short lines of a concentric
   solid fill are dropped, so one input could give different G-code from run
   to run. Upstream gave the two members defaults in `e2fd46f82`, which
   v2.5.2 has. Snapmaker Orca 2.4.1 has the same two files without the fix:
   two files in our override folder (`libslic3r/snapmaker/libslic3r/Fill/`)
   set the two members to upstream's defaults. No file under `references/`
   changes; the pin move changes only the submodule pointer.

## What the front end had to change

Every item is a port of the official command line at v2.5.2
(`src/OrcaSlicer.cpp`) or of the desktop app, cited in the code at `f92a7c0`.
All 120 hunks of the upstream command-line diff were read; each is ported,
was already covered, does not apply to this front end, or is one of the
recorded differences below. New code is for the OrcaSlicer build only
(`ENGINE_ORCA` without `ENGINE_SNAPMAKER`); the Snapmaker Orca build keeps
the 2.4 paths.

| Change | Where | Why |
|-|-|-|
| The multi-nozzle paths now run for OrcaSlicer | `cli_load_settings.cpp`, `cli_nozzle_options.cpp`, `cli_filament_map.cpp`, `cli_plate_print.cpp`, `cli_result_json.cpp` | 2.5 has Bambu Studio's nozzle statistics, per-nozzle volume types and maps; `Print::validate` returns a list of warnings |
| Nozzle statistics in OrcaSlicer's own format | `cli_load_settings.cpp`, `cli_load_project_archive.cpp` | `extruder_nozzle_stats` is `Standard#1` per extruder, written by the app after a device sync and read by the command line (`OrcaSlicer.cpp` 6865); a project's own value is kept across the app-style load |
| Changed-settings column against the shipped parent | `cli_load_settings.cpp` | the official command line records each setting file's difference to its parent preset |
| Process compatibility through the engine's condition | `cli_load_settings.cpp` | `is_compatible_with_printer` with the process and printer settings, as the 2.5 command line calls it |
| Filament values per nozzle variant | `cli_load_settings.cpp`, `cli_filament_config.cpp` | `normalize_filament_values_to_variants`, `filament_dev_options` rebuilt with `set_filament_dev_options` |
| Mixed filaments | `cli_plate_settings.cpp`, `cli_plate_print.cpp`, `cli_filament_map.cpp`, `cli_plate_report.cpp`, `cli_slice_export.cpp`, `cli_load_project_archive.cpp` | a slot that mixes two filaments; refused (-69) when a mixed slot has no filament of its own or mixes two material types; the exported project carries `mixed_filaments_info` |
| Bambu filament ids in the sliced file | `cli_plate_report.cpp` | OrcaSlicer 2.5 names filaments `OF…`; for a Bambu Lab printer the sliced file gets Bambu's `GF…` id from `resources/printers/bambu_filament_ids.json` (`OrcaSlicer.cpp` 7407-7447) |
| Prime tower | `cli_plate_arrange.cpp`, `cli_plate_plan.cpp`, `cli_slice_plate.cpp`, `cli_arrange_geometry.cpp` | the tower is moved inside the shared printable area (`WipeTower::move_box_inside_polygon`), the arrange reserves room per plate from that plate's filament count, a belt printer has no tower |
| Plate filament list | `cli_arrange_geometry.cpp` | `PartPlate::get_extruders_under_cli` at 2.5.2: a top or bottom surface filament left at "Default" no longer takes the internal solid filament |
| Bed exclusion volumes in the arrange | `cli_arrange_geometry.cpp` | new setting; no shipped printer states one at this pin |
| Default plate of a printer | `desktop_presets.cpp` | `Preset::get_default_bed_type` reads a plate **name** from 2.5 on (`Preset.cpp` 1140-1171); Snapmaker Orca still reads a number |
| Colours of filament files without colours | `cli_load_settings.cpp` | the 2.5 command line starts from an empty list: every filament white, flush matrix rebuilt |
| Seven new flags | `cli_run_steps.cpp`, `cli_flag_support.cpp`, `cli_options.*` | `--strict`, `--ground-largest-face`, `--ground-face-normal`, `--ground-face-point`, `--repair`, `--align-to-y-axis`, `--export-statistics` |
| `result.json` | `cli_result_json.cpp`, `schemas/` | `strict_mode` and `warnings`, as upstream writes them |
| Arrange refusal sentence | `cli_plate_arrange.cpp` | a part that fits the bed only turned 90 degrees names `--allow-rotations` (this sentence changes on all three builds) |
| `Thread.cpp` override regenerated | `libslic3r/orcaslicer/libslic3r/Thread.cpp` | upstream changed the file; the override is the 2.5.2 copy with the same fix |
| Build | `CMakeLists.txt` | the bundled `clipper` folder is optional (2.5 has only Clipper2), `libslic3r/CAD/` is left out (the parametric design tab), `CGAL_LINKED_WITH_TBB` as upstream sets it, the OpenCASCADE toolkit list follows the version found |

Not ported, on purpose:

- **Where a settings file's parent comes from.** Upstream resolves `inherits`
  only for a preset of the loaded bundle or a file inside a vendor tree
  (`PresetBundle::resolve_preset_config`, `PresetBundle.cpp` 556-650).
  slicer-cli resolves it by name from the shipped profiles for any file,
  which covers upstream's cases and more.
- **Project refresh.** The 2.5 command line refreshes a project's printer,
  process and filaments from the current system presets when the project
  lists changed keys or `--uptodate` is given (`OrcaSlicer.cpp` 2800-2843,
  3189-3225). slicer-cli keeps its two documented loads: the app-style load
  by default, the stored values with `--project-settings-as-saved`.
- **A replaced setting value.** Upstream warns and slices. slicer-cli prints
  the same warning and refuses (-18) unless `--allow-substitution` is given,
  as it does on the Bambu Studio build.
- **The engine's final filament maps in an exported project.** Bambu Studio's
  command line writes them too and slicer-cli never mirrored that; the same
  choice is kept for OrcaSlicer.
- **`--inspect-mesh`, `--inspect-paint`.** Their reports are made with app
  code that slicer-cli does not build. Both are refused by name.
- **`--export-settings -`** (settings to stdout). stdout carries this
  binary's events, so `-` is refused with a sentence; a file name works.
- **`src/libslic3r/CAD/`** (the parametric design tab) and OpenCASCADE's
  ModelingAlgorithms module: nothing in the command line uses them, and
  upstream's own dependency build leaves the module off without the tab.

Two values in the tests are read from the 2.5.2 code and not measured against
an official build: the flush multiplier and matrix of a model file loaded
with two filament files (`OrcaSlicer.cpp` 3874-3886, 3973-4047), and the
prime tower entry of a project whose list is longer than its plate count
(6495-6526).

## Dependencies

OrcaSlicer 2.5.2 pins newer libraries than 2.4.2
(docs/engine-dependency-contract.md):

| Library | 2.4.2 build | 2.5.2 build |
|-|-|-|
| CGAL | 5.6.3 (Linux, macOS CI), vcpkg 6.1.1 (Windows) | 6.2.1 everywhere: the archive and SHA-256 of `deps/CGAL/CGAL.cmake`; on Windows through the overlay port `ci/vcpkg-overlays-orca/cgal` |
| OpenCASCADE | 7.6.0 static (Linux); vcpkg 7.9.3 (Windows); Homebrew (macOS) | 8.0.1 static with OrcaSlicer's `0001-BRepMesh-seam-pcurve-selection.patch` on Linux, macOS and Windows |

The source needs CGAL 6.2: `MeshBoolean.cpp` includes
`CGAL/boost/graph/border.h`, which 6.1.1 does not have (found on the Windows
box: error C1083). `ci/patches/orca-cgal6-extract-boundary-cycles.patch` is
deleted: 2.5.2 calls the CGAL 6 function itself and no step applied the file.
Bambu Studio and Snapmaker Orca keep CGAL 5.4 and OpenCASCADE 7.6.0.

On Windows the OrcaSlicer build sets the MSVC option `/Zc:lambda`, as
OrcaSlicer's own `CMakeLists.txt` does (391-405 at `f92a7c0`). 2.5 reads
constexpr constants inside lambdas that do not capture them
(`FillRectilinear.cpp` 2416, `GCodeProcessor.cpp` 7548); MSVC under C++17
refuses that without the option (error C3493). It needs Visual Studio 2019
16.8 or newer. It also sets `/bigobj` (the same file, 406-407): `MeshRepair.cpp`, new in
2.5, has more sections than the default object format holds (C1128).

On the Linux box the same OrcaSlicer front end was built against both
kernels and given upstream's own STEP test files (`tests/data`):

| STEP file | on OpenCASCADE 7.6 | on OpenCASCADE 8.0.1 with the patch |
|-|-|-|
| `cone_tilted_seam_pcurve.step` | not sliced ("No layers were detected") | sliced, 125 layers |
| `utf8_part_names.step` | sliced, 30 layers | sliced, 30 layers |
| `security_classification.step` | sliced, 47 layers | sliced, 47 layers |

The first file is the case upstream's patch is for (a cone whose seam curve
is slightly tilted).

The OrcaSlicer build on Windows and macOS uses the same static OpenCASCADE
8.0.1 with the patch (Windows: the overlay port
`ci/vcpkg-overlays-orca/opencascade`; macOS: `install_deps.sh`). On Windows
the OrcaSlicer program slices the three files with 125, 30 and 47 layers; an
OrcaSlicer program built against vcpkg's 7.9.3 refused the first file (-100).
On macOS the first file is a case of `tests/test-complete-slice-path.sh`.

## What stays the same

- **Bambu Studio and Snapmaker Orca output.** Old package (PR head before
  this bump) against new, same inputs, first plate: every preset case and
  every fixture project gives the same layer count, time, filament length,
  move count and settings. On ten maker projects the Bambu Studio build gives
  the same layers and settings; five of them differ in a few moves, and so do
  three runs of the *old* binary alone on one of those files (343,947,
  343,951 and 343,953 moves): that engine is not byte-deterministic.
- The events, tags and return codes of the Bambu Studio and Snapmaker Orca
  builds. The one sentence that changes is the arrange refusal above.
- The Snapmaker Orca front end: every 2.5 path is closed to it.

## Expected G-code drift (OrcaSlicer)

Old (2.4.2) against new (2.5.2), a 30 x 30 x 50 mm post on the named printer
with nothing else given, first plate; layer count and filament length from the
G-code header:

| printer | layers old / new | time old | time new | filament mm old / new | moves old / new |
|-|-|-|-|-|-|
| Bambu Lab A1 mini 0.4 | 250 / 250 | 38m 0s | 34m 29s | 5098 / 5182 | 35,445 / 16,628 |
| Bambu Lab X1 Carbon 0.4 | 250 / 250 | 26m 14s | 23m 25s | 5182 / 5267 | 34,219 / 15,625 |
| Bambu Lab H2D 0.4 | 250 / 250 | 27m 41s | 23m 53s | 5093 / 5167 | 34,078 / 15,272 |
| Creality K2 Plus 0.4 | 312 / 312 | 28m 10s | 1h 19m 20s | 5117 / 5117 | 19,145 / 19,136 |
| Prusa MK4 0.4 | 250 / 250 | 1h 10m 39s | 47m 37s | 5738 / 5749 | 36,424 / 35,342 |
| Snapmaker U1 0.4 | 624 / 250 | 56m 22s | 25m 30s | 5003 / 4847 | 79,108 / 39,718 |

Every case slices on both engines. The causes, read from the settings each
G-code states:

- **Bambu printers: about half the moves.** The shipped process profiles now
  give `sparse_infill_pattern = grid` where 2.4.2 gave `crosshatch`. Their
  printer profiles no longer state `nozzle_type` (it reads `undefine`), and
  `bed_temperature_formula` is `by_highest_temp`.
- **Snapmaker U1: 250 layers, was 624.** Its default process is now
  `0.20 Standard @Snapmaker U1 (0.4 nozzle)`; it was
  `0.08 Extra Fine @Snapmaker U1 (0.4 nozzle)`.
- **Prusa MK4: a shorter time estimate.** The settings are the same but for
  the default filament's new name (`Generic PLA @Prusa MK4`); the layers and
  the machine limits in the G-code are the same. The difference is in the
  engine and was not traced further.
- **Creality K2 Plus 0.4: an ABS filament by default.** The printer's default
  filament, `Generic PLA @Creality K2-all`, does not list this printer as
  compatible in the shipped profiles, so the first compatible filament is
  used, the app's own rule for a newly picked printer
  (`PresetBundle::load_selections`, `PresetBundle.cpp` 4069-4082 and
  4186-4193): `Generic ABS @K2 Plus-all` (260 °C, slower cooling).
  The run says so (`presets_resolved`, `defaults_replaced`). Name a filament
  with `--filament-preset` on such a printer. A read of the profile files
  finds a default filament that does not list its own printer on 191 printer
  presets at 2.5.2 (172 at 2.4.2); among Creality's it is 41, was 14.
- **Bambu Lab H2D with no filament named: one filament, was two.** The engine
  now starts a printer with one filament when it is
  `single_extruder_multi_material`, else one per extruder
  (`PresetBundle::min_filament_count`, `PresetBundle.cpp` 5419-5424); 2.4.2
  gave one per extruder whatever that setting. A two-cube assemble list that
  asks for filaments 1 and 2 on the H2D printed with two filaments and a
  prime tower on 2.4.2 (2h 45m, 16.4 m of filament). With one filament the
  engine printed both cubes with it and said nothing, on every printer that
  starts with one filament. slicer_cli now refuses that list (-2) on all
  three binaries: name each filament with `--filament-preset`.

**Preset names.** A read of the shipped profile files (selectable presets,
every vendor):

| kind | 2.4.2 | 2.5.2 | names that no longer exist | of those, with a successor in `renamed_from` | with none |
|-|-|-|-|-|-|
| printers | 1,001 | 1,194 | 1 | 1 | 0 |
| processes | 2,881 | 3,259 | 22 | 21 | 1 |
| filaments | 5,952 | 7,637 | 789 | 748 | 41 |

An old name with a successor still works: `--filament-preset "Prusa Generic
PLA @MK4"` on the Prusa MK4 slices and reports `Generic PLA @Prusa MK4`
(run on the Linux box). The 41 filament names with no successor are SeeMeCNC
(21), Creality (18, for example `CR-PLA @Ender-5 Max-all`), Dremel (1) and one
Bambu-folder name whose spelling was corrected; such a name is refused
(-5) with the close names. Third-party filaments for Bambu printers moved
from the Bambu vendor folder to OrcaSlicer's filament library and keep their
names: the Bambu Lab X1 Carbon 0.4 lists 439 filaments, was 366, and lost
two names.

A project saved by OrcaSlicer 2.4.2 slices on 2.5.2 (the two 2.4.2 fixture
projects: 100 layers each). A project saved by 2.5.2 is refused by the 2.4.2
engine as newer (-24).

Other behaviour that changed with the engine, each in
docs/cli-reference.md under "Known upstream behaviour":

- `--arrange 1` now places a part on the Construct 1 and the Folgertech i3
  0.6 (2.4.2: -21). The Anycubic Kobra 3 still fails.
- A fixed layout with Bambu PLA Basic and Bambu PETG HF on the X1 Carbon
  slices (2.4.2: -104, the prime tower past the bed edge).
- The Snapmaker U1 refuses 42 of its 93 filaments on its default plate,
  which is now Textured PEI.

## Test results

See the pull request for the runs on its final head.
