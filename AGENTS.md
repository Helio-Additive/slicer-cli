# AGENTS.md

For coding agents working in this repository. Short and exact; the linked files
are the authority.

## What this is

A command-line slicer: one front end, three slicing engines. It ships three
programs, `slicer_cli` (Bambu Studio engine, source pin `926a719`),
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
| `main.cpp` | The front end: option parsing, run modes, the engine-specific blocks (`ENGINE_BAMBU` / `ENGINE_ORCA` / `ENGINE_SNAPMAKER`) |
| `cli_snapmaker.cpp` | The Snapmaker Orca build's desktop rules (mixed filaments, paint remap, pre-slice checks, the prime tower) and its profile lookup |
| `cli_command_line.cpp`, `cli_flag_support.cpp` | Flag table, `--help`, the refusal sentences for flags a program does not take |
| `cli_load_settings.cpp`, `desktop_presets.cpp` | Preset/profile resolution, `inherits` chains, desktop-equivalent selection |
| `cli_model_load.cpp`, `cli_assemble.cpp`, `cli_run_steps.cpp`, `cli_repetitions.cpp` | Model and 3MF loading, plate assembly, the run itself, repeated plates |
| `cli_pipe.cpp` | The official Linux progress pipe |
| `layout_plan.cpp`, `calib_args.cpp`, `startup_logging.cpp` | `--layout` planning, calibration arguments, startup log |
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
tests`, `Run feature-exclusion regression tests`, `Test diagnostic transport`
and `Run nozzle-map provenance regression tests (bambu)`:

```sh
bash tests/test_diagnostic_events.sh build/slicer_cli        # engine read from the binary
bash tests/test_excluded_features.sh build/slicer_cli
python3 tests/test_diagnostic_transport.py                   # no arguments; compiles its own probe
bash tests/test_nozzle_map_provenance.sh build/slicer_cli    # Bambu build only
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
  `#ifdef ENGINE_SNAPMAKER` (in `cli_snapmaker.cpp` where it is more than a
  branch). A change for the Snapmaker build must leave the other two programs'
  output as it was.
- **New behaviour gets a new flag; an existing flag keeps its meaning.** What
  the program prints — messages, event kinds and tags, `result.json` keys,
  return codes — is read by other programs: add, never change.
- **Refusals are clear sentences that say what to do**, and name the program
  that fits when the other engine does (`cli_flag_support.cpp`): a flag a
  program does not take is refused by name, before anything loads, and points
  at `slicer_cli`, `slicer_cli-orcaslicer` or `slicer_cli-snapmaker` as appropriate.
- **A test proves its fix:** it fails before the change and passes after.
- **No machine-specific paths, hostnames or internal names** in code, docs or
  commits. This repository is public.
- C++17, the engine's conventions. New command-line code goes in its own
  `cli_*.cpp`, not in `main.cpp`.

## Releases

Tags are `slicer-cli/v<version>`; the `.github/workflows/slicer-cli-ci.yml` job
`Publish slicer-cli release assets` publishes them. Contributors do not tag or
publish.

Everything else — the CLA, what is and is not accepted, style — is in
[CONTRIBUTING.md](CONTRIBUTING.md).
