# AGENTS.md

For coding agents working in this repository. Short and exact; the linked files
are the authority.

## What this is

A command-line slicer: one front end, three slicing engines. It ships three
programs, `slicer_cli` (Bambu Studio engine, source pin `56a57d0`),
`slicer_cli-orcaslicer` (OrcaSlicer engine, pin `8500fcd`) and
`slicer_cli-snapmaker` (Snapmaker Orca engine, a Snapmaker fork of OrcaSlicer,
pin `bac6fd5`), for Linux x86_64,
Linux arm64, macOS arm64 and Windows x86_64. Flags that share a name with the desktop apps'
own command lines behave the same way.

Read first, and change here rather than copy: [README.md](README.md),
[CONTRIBUTING.md](CONTRIBUTING.md), [docs/cli-reference.md](docs/cli-reference.md),
[docs/building.md](docs/building.md), `.github/workflows/slicer-cli-ci.yml`
(the full, tested recipe), `CMakeLists.txt`.

## Layout

| Path | What it is |
|-|-|
| `src/` | Command-line sources and headers, listed below |
| `libslic3r/bambustudio/`, `libslic3r/orcaslicer/`, `libslic3r/snapmaker/` | The override layer: files that replace engine source of the same relative path |
| `references/BambuStudio`, `references/OrcaSlicer`, `references/SnapmakerOrca` | The pinned engine submodules. Read-only, upstream as-is |
| `tests/` | The test scripts, run against built or packaged programs |
| `docs/` | `cli-reference.md` (every flag, outputs, `result.json`, event lines, return codes), `building.md` |
| `ci/` | vcpkg overlay port and patches the build applies |
| `.github/workflows/slicer-cli-ci.yml` | Build, test, package, release |

The build picks its engine with the CMake cache variable `ENGINE`
(`-DENGINE=bambu` default, `-DENGINE=orca`, `-DENGINE=snapmaker`;
`CMakeLists.txt`), which defines `ENGINE_BAMBU`, `ENGINE_ORCA`, or
`ENGINE_ORCA` and `ENGINE_SNAPMAKER` together for the front-end blocks: the
Snapmaker build is an OrcaSlicer build, and `ENGINE_SNAPMAKER` marks what only
it does. The Bambu Studio and OrcaSlicer engines build a program called
`slicer_cli`, and packaging renames the OrcaSlicer one; the Snapmaker engine
builds `slicer_cli-snapmaker`.

## Where things live

The slice driver owns one `RunState`. Loading fills `LoadedProject` (model, project facts and plate data) and `RunSettings` (the merged base and flag overrides).
`RunPlatePlan` holds grid origins and instance membership. Preparation saves its configuration and carried outcome in `PlatePreparation`.
`PlateWarnings` holds diagnostics across the check pass. `RunPipeProgress` tracks the progress announcements already sent.
`RunActionOutputs` tracks actions run and output paths reserved. Each plate has its own working configuration and `PlateOutcome` for reporting.

| File in `src/` | Responsibility |
|---|---|
| `main.cpp` | Dispatch startup, parsing, validation and run modes |
| `cli_command_line.cpp` | Parse arguments, build the flag table and print help |
| `cli_command_line.hpp` | Parser entry points and mode arguments |
| `cli_options.hpp` | Parsed flags, input files, requested actions and compatibility switches |
| `cli_flag_support.cpp` | Select the refusal sentence for an unsupported flag |
| `cli_flag_support.hpp` | Engine-specific flag support lookup |
| `cli_list_flags.cpp` | Write the flag catalog to stdout |
| `cli_list_flags.hpp` | Flag catalog command |
| `cli_list_settings.cpp` | Write the engine's setting definitions to stdout |
| `cli_list_settings.hpp` | Settings catalog command |
| `cli_gcode_timing.cpp` | Read existing G-code and report the engine's time estimates |
| `cli_gcode_timing.hpp` | G-code timing command and result contract |
| `cli_file_info.cpp` | Read a model for the file information command |
| `cli_file_info.hpp` | File information entry point |
| `cli_engine_info.cpp` | Inspect project object IDs and report the engine information |
| `cli_engine_info.hpp` | Engine information and object ID lookups |
| `calib_args.cpp` | Parse calibration arguments and build pressure advance pattern geometry |
| `calib_args.hpp` | Calibration mode and pattern interface |
| `cli_layout_command.cpp` | Read a layout request and dispatch the layout command |
| `cli_layout_command.hpp` | Layout command entry points |
| `layout_plan.cpp` | Solve layout plans and serialize placement candidates |
| `layout_plan.hpp` | Versioned layout request, capabilities and candidate types |
| `layout_spike.cpp` | Run the standalone layout experiment |
| `arrange_harness.cpp` | Build the standalone arrange experiment from engine profiles |
| `startup_logging.cpp` | Silence engine startup logs before command dispatch |
| `standalone_stubs.cpp` | Supply headless implementations of engine GUI hooks |
| `cli_slice_run.cpp` | Drive the prepare, check and slice passes and collect their outcomes |
| `cli_slice_run.hpp` | Slice run entry point |
| `cli_run_context.hpp` | Run-wide options, preset cache, assemble list, downward checks and staging paths |
| `cli_slice_state.hpp` | Ownership of the project, settings, plan, preparation, warnings, pipe and action outputs |
| `cli_load_project_archive.cpp` | Read the project archive, rebase desktop presets and recover its plate layout |
| `cli_load_project_archive.hpp` | Archive load inputs, project facts and plate layout outputs |
| `cli_load_slice_inputs.cpp` | Load models and projects, merge settings and validate inputs before preparation |
| `cli_load_slice_inputs.hpp` | Input classification and per-load settings and substitutions |
| `cli_project_archive.cpp` | Read ZIP members, count plates and check file versions |
| `cli_project_archive.hpp` | Archive member and version checks |
| `cli_project_settings.cpp` | Read raw project, object and part settings from the archive |
| `cli_project_settings.hpp` | Raw settings types and project readers |
| `cli_project_values.cpp` | Translate project values and record warnings or refusals |
| `cli_project_values.hpp` | Value conversion results for one settings scope |
| `cli_project_load.cpp` | Restore the project's prime tower placement and describe replaced settings |
| `cli_project_load.hpp` | Desktop project load helpers |
| `cli_model_load.cpp` | Read geometry files and place loaded objects on the bed |
| `cli_model_load.hpp` | File classification, load result and placement interface |
| `cli_assemble.cpp` | Read the assemble list and build its plates and models |
| `cli_assemble.hpp` | Assemble list, plate and object types |
| `cli_load_settings.cpp` | Merge machine, process and filament files and updated presets |
| `cli_load_settings.hpp` | Project facts and the settings merge result |
| `cli_named_presets.cpp` | Resolve named presets, apply file inheritance and cache preset settings |
| `cli_named_presets.hpp` | Named preset and file inheritance entry points |
| `desktop_presets.cpp` | Apply desktop printer and filament selection rules to the preset bundle |
| `desktop_presets.hpp` | Desktop preset selection interface |
| `cli_printer_catalog.cpp` | Look up printers across the three engines' packaged profiles |
| `cli_printer_catalog.hpp` | Engine fit and printer model lookup types |
| `cli_preset_staging.cpp` | Stage profile trees, mark their ownership and configure engine resources |
| `cli_preset_staging.hpp` | Preset staging and engine resource interface |
| `cli_json_config.cpp` | Read JSON overrides and detect explicit nozzle maps in input files |
| `cli_json_config.hpp` | JSON loading and nozzle map provenance queries |
| `cli_percent_widths.cpp` | Convert percentage line widths in an input 3MF for the Bambu Studio engine |
| `cli_percent_widths.hpp` | Temporary rewritten project and refusal details |
| `cli_filament_config.cpp` | Build filament defaults and align configuration vectors with the roster |
| `cli_filament_config.hpp` | Filament configuration and nozzle mapping helpers |
| `cli_filament_map.cpp` | Validate the command-line filament-to-nozzle mapping |
| `cli_filament_map.hpp` | Filament map validation result |
| `cli_nozzle_diameter.cpp` | Populate and validate nozzle diameter choices per filament |
| `cli_nozzle_diameter.hpp` | Mixed nozzle diameter helpers |
| `cli_nozzle_options.cpp` | Apply nozzle and filament grouping options |
| `cli_nozzle_options.hpp` | Nozzle and grouping flag interface |
| `cli_setting_overrides.cpp` | Apply command-line values to the active configuration |
| `cli_setting_overrides.hpp` | Setting override interface |
| `cli_legacy_gcode.cpp` | Normalize legacy placeholders and identify unsupported custom G-code tokens |
| `cli_legacy_gcode.hpp` | Placeholder conversion and lookup interface |
| `cli_layer_ranges.cpp` | Fill missing layer heights for model height ranges |
| `cli_layer_ranges.hpp` | Layer range normalization interface |
| `cli_plate_settings.cpp` | Overlay plate and flag settings, nozzle maps and retained tower positions |
| `cli_plate_settings.hpp` | Per-plate configuration and manual map checks |
| `cli_check_downward.cpp` | Check plate sizes and printer compatibility for the downward check |
| `cli_check_downward.hpp` | Downward-check inputs and plate outcome |
| `cli_slice_plate.cpp` | Apply plate settings, transform, arrange and check objects before printing |
| `cli_slice_plate.hpp` | Single-plate preparation and slicing entry point |
| `cli_plate_plan.cpp` | Build the grid, instance membership and wipe tower positions across plates |
| `cli_plate_plan.hpp` | Project plan, scene bounds and grid origin interface |
| `cli_arrange_geometry.cpp` | Build exclusion polygons and derive extruder use for arrangement |
| `cli_arrange_geometry.hpp` | Geometry and extruder queries used by arrangement |
| `cli_plate_arrange.cpp` | Check bed bounds, move objects to a new bed and arrange plate instances and towers |
| `cli_plate_arrange.hpp` | Plate fit, clearance, arrangement and tower sizing interface |
| `cli_plate_scope.cpp` | Temporarily expose one plate's model instances in plate coordinates |
| `cli_plate_scope.hpp` | Scoped plate membership and coordinate restoration |
| `cli_run_steps.cpp` | Run ordered transforms and model actions and validate their arguments |
| `cli_run_steps.hpp` | Action phases, step outcomes and slice cache paths |
| `cli_simplify.cpp` | Reduce meshes and report simplification warnings |
| `cli_simplify.hpp` | Mesh simplification entry points |
| `cli_repetitions.cpp` | Duplicate and arrange the requested plate copies |
| `cli_repetitions.hpp` | Repetition result and arrangement callback |
| `cli_snapmaker.cpp` | Apply Snapmaker Orca printer switch, filament, paint and tower rules |
| `cli_snapmaker.hpp` | Snapmaker Orca profile and plate rule interface |
| `cli_plate_print.cpp` | Apply the prepared plate to the engine, validate, process and export G-code |
| `cli_plate_print.hpp` | Prepared plate inputs, action checks and pipe progress steps |
| `cli_pipe.cpp` | Send progress and warnings to the named Linux pipe |
| `cli_pipe.hpp` | Progress pipe lifecycle and messages |
| `cli_engine_events.cpp` | Translate engine substitutions, warnings and validation results into events |
| `cli_engine_events.hpp` | Engine diagnostic event adapters |
| `cli_events.hpp` | Structured event writer |
| `diagnostic_output.hpp` | Write each diagnostic record as one synchronized stream operation |
| `cli_failure_reason.cpp` | Build the explanation for a failed plate |
| `cli_failure_reason.hpp` | Plate failure reason helpers |
| `cli_plate_report.cpp` | Collect plate statistics and data for project export |
| `cli_plate_report.hpp` | Plate statistics and export capture interface |
| `cli_slice_export.cpp` | Check output path conflicts and write the sliced project archive |
| `cli_slice_export.hpp` | Output reservation and sliced project export interface |
| `cli_result_json.cpp` | Serialize run and plate outcomes and protect the result path |
| `cli_result_json.hpp` | Plate outcome, result codes and result writer interface |
| `cli_run_cleanup.cpp` | Create and detach model backup paths and sweep stale staging folders |
| `cli_run_cleanup.hpp` | Backup path and scoped detachment interface |
| `cli_staging_folders.cpp` | Track and remove a run's staging folders |
| `cli_staging_folders.hpp` | Run-owned staging folder inventory |

## Build and test

One engine per build tree; build three times for all three. Build per operating system and
engine by [docs/building.md](docs/building.md): its "Dependencies" and "Configure
and build" sections carry the pinned versions, the per-platform recipe and the
prefix each engine needs. The tested recipe is the `Build & Test` job of
`.github/workflows/slicer-cli-ci.yml`, in steps `Install dependencies (macOS)`,
`Install dependencies (Linux)`, `Install dependencies (Windows)`, `Configure
(macOS / Linux)`, `Configure (Windows)` and `Build`.

The job and step names quoted below are those in that workflow, with the
`${{ matrix.* }}` suffix left off where a name is parameterised. Every build and
test job is skipped on a draft pull request (`github.event.pull_request.draft ==
false`), so mark a PR ready for review before reading its checks. A change that
touches only markdown other than `README.md`, or files under `docs/`, starts no
CI run: both triggers carry an ordered `paths` filter that excludes `**/*.md`
and `docs/**` and re-includes `README.md`, which the packages, CPack and the
install rules copy. `LICENSE` and `NOTICE` are never excluded, for the same
reason.

Tests. The build-tree tests take the program the build produced; CI runs them on
Linux and macOS, never on Windows, in steps `Run structured-diagnostics event
tests`, `Run feature-exclusion regression tests`, `Test diagnostic transport`,
`Run nozzle-map provenance regression tests (bambu)`, `Run --filament-map tests`
and `Run official command-line parity check`:

```sh
bash tests/test_diagnostic_events.sh build/slicer_cli        # engine read from the binary
bash tests/test_excluded_features.sh build/slicer_cli
python3 tests/test_diagnostic_transport.py                   # no arguments; compiles its own probe
bash tests/test_nozzle_map_provenance.sh build/slicer_cli    # Bambu build only
bash tests/test_filament_map_flag.sh build/slicer_cli        # engine read from the binary
python3 tests/test_official_cli_parity.py build/slicer_cli   # any build; reads references/
```

The package tests take the packaged programs, from the `Package` job (macOS and
Windows) and the `Package - Linux-x86_64` and `Package - Linux-arm64` jobs. Linux keeps them in `bin/` (step
`Assemble and validate Linux package`); macOS (step `Assemble + validate package
(macOS arm64)`) and Windows (step `Assemble + validate package (Windows x86_64)`)
put them at the package root, with `.exe` on Windows:

```sh
# every OS, the first two programs, in that order
bash tests/test-complete-slice-path.sh <slicer_cli> <slicer_cli-orcaslicer>
# macOS and Windows; the second and third programs are optional
bash tests/test-packaged-slice.sh <slicer_cli> [<slicer_cli-orcaslicer> [<slicer_cli-snapmaker>]]
# macOS only; both programs required
bash tests/test_list_settings.sh <slicer_cli> <slicer_cli-orcaslicer>
# Linux package only; takes the package root and checks the bin/ layout
bash tests/test-portable-bundle.sh <package root>
```

`tests/test-complete-slice-path.sh` covers the Bambu Studio and OrcaSlicer engines: the flags, plate type
rules, model formats, `result.json` and exported projects. Flags and outputs are
documented in [docs/cli-reference.md](docs/cli-reference.md); a first build
compiles about 1,500 engine files.

## Rules

- **Never edit anything under `references/`.** The pinned engine sources stay
  byte-identical upstream so an engine bump stays easy. A `libslic3r` fix goes
  in the override layer — a file at the same relative path under
  `libslic3r/bambustudio/`, `libslic3r/orcaslicer/` or `libslic3r/snapmaker/`, which CMake compiles in
  place of the engine's file ([CONTRIBUTING.md](CONTRIBUTING.md), "Engine
  source"; docs/building.md, "The override layer") — or in the front end.
  An override freezes its file at the commit it was taken from; a bump has to
  re-apply the change to the new file.
- **Port, don't invent.** Behaviour follows the desktop app's code at the
  pinned references. A commit that ports behaviour cites the upstream file and
  line numbers (`BambuStudio.cpp 1234`, `OrcaSlicer.cpp 1234`).
- **All engines, all three operating systems, every change.** No single-OS or
  single-engine code path. Where the engines differ, follow each engine's own
  code and say which one a line applies to; engine-only code is guarded by
  `#ifdef ENGINE_ORCA` / `#ifndef ENGINE_ORCA`, and Snapmaker-only code by
  `#ifdef ENGINE_SNAPMAKER` (in `src/cli_snapmaker.cpp` where it is more than a
  branch). A change for the Snapmaker build must leave the other two programs'
  output as it was.
- **New behaviour gets a new flag; an existing flag keeps its meaning.** What
  the program prints — messages, event kinds and tags, `result.json` keys,
  return codes — is read by other programs: add, never change.
- **A change to what an existing call does is allowed when it is right, and
  it does not go alone.** A defect is fixed without a flag. When a fix, or a
  decision of the owner, changes the result of a call that worked before, the
  PR says so under "Changed on purpose", gives the flag that brings the old
  result back when there is one, and states that the programs that call
  slicer-cli get their own PRs for it before they use the release that
  carries the change.
- **A project loads the way its desktop app loads it, on all three programs.**
  Each app keeps the settings the maker changed and takes every other value
  from the preset it has today, for its own projects too.
  `--project-settings-as-saved` slices the saved copy. A step an app does on
  load is checked in all three apps' source before it is ported for one.
- **slicer-cli takes what the official programs take, and what their apps'
  screens choose.** On an engine update, list every setting each official
  command line reads from the flags (`m_extra_config`) and every choice the
  app's screens offer in the area that changed, and port each one, for all
  three engines. A setting an official command line handles may be refused as
  a flag only with the repository owner's agreement, listed with its reason in
  `tests/official-cli-refused-settings.json`;
  `tests/test_official_cli_parity.py` fails on any other such refusal. "The
  engine derives it" is not a reason.
- **Refusals are clear sentences that say what to do**, and name the program
  that fits when the other engine does (`src/cli_flag_support.cpp`): a flag a
  program does not take is refused by name, before anything loads, and points
  at `slicer_cli`, `slicer_cli-orcaslicer` or `slicer_cli-snapmaker` as appropriate.
- **A test proves its fix:** it fails before the change and passes after.
- **A pull request says what it covers** (`.github/pull_request_template.md`):
  the engine versions it was tested on, one line for each of the three engines
  (changed, refused or not relevant, and why) and one for each system (tested
  or not, and where). The `PR description` check fails when a section is missing.
- **No machine-specific paths, hostnames or internal names** in code, docs or
  commits. This repository is public.
- C++17, the engine's conventions. New command-line code goes in its own
  `src/cli_*.cpp`, not in `src/main.cpp`.

## Releases

Tags are `slicer-cli/v<version>`; the `.github/workflows/slicer-cli-ci.yml` job
`Publish slicer-cli release assets` publishes them. Contributors do not tag or
publish.

Everything else — the CLA, what is and is not accepted, style — is in
[CONTRIBUTING.md](CONTRIBUTING.md).
