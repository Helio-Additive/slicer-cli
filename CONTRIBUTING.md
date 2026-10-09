# Contributing to slicer-cli

Thank you for contributing. Before opening a PR, please read the sections below.

## CLA

All contributions require signing the Helio Additive CLA. This preserves
dual-licence optionality and is required before any external PR is merged.
The CLA bot will comment on your first PR with instructions.

> **Why a CLA for an AGPL project?** The AGPL covers distribution and network
> use. The CLA additionally grants Helio a non-exclusive licence to use
> contributions in commercial builds that run `slicer-cli` as a separate
> program.

## What to contribute

Welcome:
- **Bug fixes**, for either engine. A fix to slicing behaviour should match
  what the desktop app (Bambu Studio or OrcaSlicer) does at the pinned
  version, and cite the lines it follows.
- **Command-line parity**: a flag of Bambu Studio's or OrcaSlicer's own
  command line that slicer-cli refuses, ported so it behaves the same way.
- **Dependency updates**: when a dependency (Boost, Eigen, CGAL, OpenCASCADE)
  changes, the build in `CMakeLists.txt`, `install_deps.sh` and the workflow
  may need updating.
- **Platform fixes**: a PR that restores a broken release target is very
  welcome.

Not accepted for now:
- Changes to the engine source under `references/`. It tracks the pinned
  engine commits as they are; see "Engine source" below.
- Printer, filament or process profile changes. Profiles come from the engines;
  send them to Bambu Studio or OrcaSlicer, and they arrive here with the next
  pin.
- Changes that break the release packages or their tests on any platform.

## Both engines

Every change to how slicing behaves must work in both programs, `slicer_cli`
(Bambu Studio engine) and `slicer_cli-orcaslicer` (OrcaSlicer engine), on
Linux, macOS and Windows. Where the engines differ, follow each engine's own
code and say which one a line applies to. Code that only one engine compiles
goes under `#ifdef ENGINE_ORCA` / `#ifndef ENGINE_ORCA`.

Text that slicer-cli prints while it runs (stdout, stderr, event lines,
`result.json`) is read by other programs. Do not change existing messages,
event kinds and tags, `result.json` keys or return codes in a fix; add new
ones instead.

## Engine source

The engine source (`references/BambuStudio`, `references/OrcaSlicer`) is not
edited. To change a `libslic3r` file for the command-line build, add a file
at the same relative path under `libslic3r/bambustudio/libslic3r/` or
`libslic3r/orcaslicer/libslic3r/`; CMake compiles it in place of the engine's
file. See "The override layer" in [docs/building.md](docs/building.md).

## CI requirements

PRs must build and pass the tests on all three platforms, for both engines:
- Linux x86_64
- macOS arm64
- Windows x86_64

The workflow is `.github/workflows/slicer-cli-ci.yml`. To build locally, see
[docs/building.md](docs/building.md).

## Style

C++17, the same conventions as the engine source. New command-line code goes
in its own files (`cli_*.cpp`), not in `main.cpp`. Comments cite the engine
lines a behaviour comes from as `BambuStudio.cpp 1234` or `OrcaSlicer.cpp
1234`, at the pinned commits.
