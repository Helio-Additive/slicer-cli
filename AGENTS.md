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
(`-DENGINE=bambu` default, `-DENGINE=orca`; `CMakeLists.txt`), which defines
`ENGINE_BAMBU` or `ENGINE_ORCA` for the front-end blocks. Both engines build a
program called `slicer_cli`; packaging renames the OrcaSlicer one.

## Build and test

One engine per build tree; build twice for both. Per-OS instructions, the pinned
dependency versions and the CGAL prefixes are in
[docs/building.md](docs/building.md). The job and step names quoted below are
those in `.github/workflows/slicer-cli-ci.yml`, with the `${{ matrix.* }}`
suffix left off where a name is parameterised. Every build and test job is
skipped on a draft pull request (`github.event.pull_request.draft == false`), so
mark a PR ready for review before reading its checks. A change that touches only
`*.md`, `docs/**`, `LICENSE` or `NOTICE` runs no CI at all (`paths-ignore` on
both triggers).

Dependencies — Linux and macOS; on Windows they come from vcpkg
(docs/building.md, "Windows"):

```sh
ENGINE=bambu ./install_deps.sh     # docs/building.md; step "Install dependencies (macOS)"
ENGINE=orca  ./install_deps.sh
```

Configure and build — steps `Configure (macOS / Linux)` and `Build` of the
`Build & Test` job:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DENGINE=bambu \
  -DCGAL_DIR=<CGAL 5.4 prefix>/lib/cmake/CGAL      # -DENGINE=orca uses the 5.6.3 prefix
cmake --build build --config Release --parallel 4
```

Windows builds follow [docs/building.md](docs/building.md), "Windows": vcpkg
gives the dependencies, libnoise comes from the engine's pinned fork and must be
built first, and its prefix goes in `-DCMAKE_PREFIX_PATH`. CI builds Windows in
steps `Configure (Windows)` and `Build (Windows)` of the same job.

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
Windows) and the `Package - Linux x86_64` job. Linux keeps them in `bin/` (step
`Assemble and validate Linux package`); macOS (step `Assemble + validate package
(macOS arm64)`) and Windows (step `Assemble + validate package (Windows x86_64)`)
put them at the package root, with `.exe` on Windows:

```sh
# every OS, both programs, in that order
bash tests/test-complete-slice-path.sh <slicer_cli> <slicer_cli-orcaslicer>
# macOS and Windows; the second program is optional
bash tests/test-packaged-slice.sh <slicer_cli> [<slicer_cli-orcaslicer>]
# macOS only; both programs required
bash tests/test_list_settings.sh <slicer_cli> <slicer_cli-orcaslicer>
# Linux package only; takes the package root and checks the bin/ layout
bash tests/test-portable-bundle.sh <package root>
```

`tests/test-complete-slice-path.sh` covers both engines: the flags, plate type
rules, model formats, `result.json` and exported projects. Flags and outputs are
documented in [docs/cli-reference.md](docs/cli-reference.md); a first build
compiles about 1,500 engine files.

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

Tags are `slicer-cli/v<version>`; the `.github/workflows/slicer-cli-ci.yml` job
`Publish slicer-cli release assets` publishes them. Contributors do not tag or
publish.

Everything else — the CLA, what is and is not accepted, style — is in
[CONTRIBUTING.md](CONTRIBUTING.md).
