# AGENTS.md

For coding agents working in this repository. Short and exact; the linked files
are the authority.

## What this is

A command-line slicer: one front end, two slicing engines. It ships two
programs, `slicer_cli` (Bambu Studio engine, source pin `926a719`) and
`slicer_cli-orcaslicer` (OrcaSlicer engine, pin `8500fcd`), for Linux x86_64,
macOS arm64 and Windows x86_64. Flags that share a name with the desktop apps'
own command lines behave the same way.

Read first, and change here rather than copy: [README.md](README.md),
[CONTRIBUTING.md](CONTRIBUTING.md), [docs/cli-reference.md](docs/cli-reference.md),
[docs/building.md](docs/building.md), `.github/workflows/slicer-cli-ci.yml`
(the full, tested recipe), `CMakeLists.txt`.

## Layout

| Path | What it is |
|-|-|
| `main.cpp` | The front end: option parsing, run modes, the engine-specific blocks (`ENGINE_BAMBU` / `ENGINE_ORCA`) |
| `cli_command_line.cpp`, `cli_flag_support.cpp` | Flag table, `--help`, the refusal sentences for flags a program does not take |
| `cli_load_settings.cpp`, `desktop_presets.cpp` | Preset/profile resolution, `inherits` chains, desktop-equivalent selection |
| `cli_model_load.cpp`, `cli_assemble.cpp`, `cli_run_steps.cpp`, `cli_repetitions.cpp` | Model and 3MF loading, plate assembly, the run itself, repeated plates |
| `cli_pipe.cpp` | The official Linux progress pipe |
| `layout_plan.cpp`, `calib_args.cpp`, `startup_logging.cpp` | `--layout` planning, calibration arguments, startup log |
| `libslic3r/bambustudio/`, `libslic3r/orcaslicer/` | The override layer: files that replace engine source of the same relative path |
| `references/BambuStudio`, `references/OrcaSlicer` | The pinned engine submodules. Read-only, upstream as-is |
| `tests/` | The test scripts, run against built or packaged programs |
| `docs/` | `cli-reference.md` (every flag, outputs, `result.json`, event lines, return codes), `building.md` |
| `ci/` | vcpkg overlay port and patches the build applies |
| `.github/workflows/slicer-cli-ci.yml` | Build, test, package, release |

The build picks its engine with the CMake cache variable `ENGINE`
(`-DENGINE=bambu` default, `-DENGINE=orca`; `CMakeLists.txt:99`), which defines
`ENGINE_BAMBU` or `ENGINE_ORCA` (`CMakeLists.txt:107,113`). Both engines build a
program called `slicer_cli`; packaging renames the OrcaSlicer one.

## Build and test

One engine per build tree; build twice for both. Per-OS instructions, the pinned
dependency versions and the CGAL prefixes are in
[docs/building.md](docs/building.md). Line numbers below are in
`.github/workflows/slicer-cli-ci.yml`. Every build and test job is skipped on a
draft pull request (`github.event.pull_request.draft == false`), so mark a PR
ready for review before reading its checks.

Dependencies — Linux and macOS; on Windows they come from vcpkg
(docs/building.md, "Windows"):

```sh
ENGINE=bambu ./install_deps.sh     # docs/building.md; CI runs it on macOS: slicer-cli-ci.yml:306,883
ENGINE=orca  ./install_deps.sh
```

Configure and build — Linux (`:629,677`) and macOS (`:1214,1262`):

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DENGINE=bambu \
  -DCGAL_DIR=<CGAL 5.4 prefix>/lib/cmake/CGAL      # -DENGINE=orca uses the 5.6.3 prefix
cmake --build build --config Release --parallel 4
```

Windows, from a Visual Studio developer prompt (`:1245,1265`); the toolchain,
CGAL and libnoise paths come from vcpkg:

```powershell
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release `
  -DENGINE=bambu `
  -DCMAKE_TOOLCHAIN_FILE="$env:VCPKG_INSTALLATION_ROOT\scripts\buildsystems\vcpkg.cmake" `
  -DCGAL_DIR="$env:VCPKG_INSTALLATION_ROOT\installed\x64-windows\share\cgal" `
  -DCMAKE_PREFIX_PATH="$env:LIBNOISE_ROOT"
cmake --build build --config Release --parallel 16
```

Tests. The build-tree tests take the program the build produced; CI runs them on
Linux (`:702-718`) and macOS (`:1287-1305`), not on Windows:

```sh
bash tests/test_diagnostic_events.sh build/slicer_cli        # engine read from the binary
bash tests/test_excluded_features.sh build/slicer_cli
python3 tests/test_diagnostic_transport.py                   # no arguments; compiles its own probe
bash tests/test_nozzle_map_provenance.sh build/slicer_cli    # Bambu build only
```

The package tests take the packaged programs. Linux keeps them in `bin/`
(`:1850-1851`); macOS (`:1534-1535`) and Windows (`:1681-1682`) put them at the
package root, with `.exe` on Windows:

```sh
# every OS, both programs, in that order
bash tests/test-complete-slice-path.sh <slicer_cli> <slicer_cli-orcaslicer>   # :1888, :1585, :1705
# macOS (:1582) and Windows (:1703); the second program is optional
bash tests/test-packaged-slice.sh <slicer_cli> [<slicer_cli-orcaslicer>]
# macOS only (:1587); both programs required
bash tests/test_list_settings.sh <slicer_cli> <slicer_cli-orcaslicer>
# Linux package only (:1882); takes the package root and checks the bin/ layout
bash tests/test-portable-bundle.sh <package root>
```

`tests/test-complete-slice-path.sh` covers both engines: the flags, plate type
rules, model formats, `result.json` and exported projects. Flags and outputs are
documented in [docs/cli-reference.md](docs/cli-reference.md); a first build
compiles about 1,500 engine files (`:674-676`).

## Rules

- **Never edit anything under `references/`.** The pinned engine sources stay
  byte-identical upstream so an engine bump stays easy. A `libslic3r` fix goes
  in the override layer — a file at the same relative path under
  `libslic3r/bambustudio/` or `libslic3r/orcaslicer/`, which CMake compiles in
  place of the engine's file ([CONTRIBUTING.md](CONTRIBUTING.md), "Engine
  source"; docs/building.md, "The override layer") — or in the front end.
  An override freezes its file at the commit it was taken from; a bump has to
  re-apply the change to the new file.
- **Port, don't invent.** Behaviour follows the desktop app's code at the
  pinned references. A commit that ports behaviour cites the upstream file and
  line numbers (`BambuStudio.cpp 1234`, `OrcaSlicer.cpp 1234`).
- **Both engines, all three operating systems, every change.** No single-OS or
  single-engine code path. Where the engines differ, follow each engine's own
  code and say which one a line applies to; engine-only code is guarded by
  `#ifdef ENGINE_ORCA` / `#ifndef ENGINE_ORCA`.
- **New behaviour gets a new flag; an existing flag keeps its meaning.** What
  the program prints — messages, event kinds and tags, `result.json` keys,
  return codes — is read by other programs: add, never change.
- **Refusals are clear sentences that say what to do**, and name the program
  that fits when the other engine does (`cli_flag_support.cpp`): a flag a
  program does not take is refused by name, before anything loads, and points
  at `slicer_cli` and `slicer_cli-orcaslicer` as appropriate.
- **A test proves its fix:** it fails before the change and passes after.
- **No machine-specific paths, hostnames or internal names** in code, docs or
  commits. This repository is public.
- C++17, the engine's conventions. New command-line code goes in its own
  `cli_*.cpp`, not in `main.cpp`.

## Releases

Tags are `slicer-cli/v<version>`; the workflow publishes the release assets
(`.github/workflows/slicer-cli-ci.yml`, `release` job, line 1929). Contributors
do not tag or publish.

Everything else — the CLA, what is and is not accepted, style — is in
[CONTRIBUTING.md](CONTRIBUTING.md).
