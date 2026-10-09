# Building slicer-cli

slicer-cli builds one engine per build tree. Build it twice to get both
programs:

| CMake option | Program | Engine source |
|-|-|-|
| `-DENGINE=bambu` (default) | `slicer_cli` | `references/BambuStudio` |
| `-DENGINE=orca` | `slicer_cli-orcaslicer` | `references/OrcaSlicer` |

The release builds run in GitHub Actions
(`.github/workflows/slicer-cli-ci.yml`) for Linux x86_64, macOS arm64 and
Windows x86_64, both engines each. That workflow is the full, tested recipe;
this page is the short form.

## Get the source

```sh
git clone --recurse-submodules https://github.com/Helio-Additive/slicer-cli.git
cd slicer-cli
```

The two engines are git submodules, pinned to one commit each:

| Engine | Submodule | Pin | Version |
|-|-|-|-|
| Bambu Studio | `references/BambuStudio` | `5873b5f` | 02.08.01.55 |
| OrcaSlicer | `references/OrcaSlicer` | `31f6803` | 2.4.0-alpha |

Both submodules point at Helio-Additive forks of the upstream projects. The
engine source is used as it is (see "The override layer" below).

## Dependencies

Each engine keeps the dependency versions its own project pins, where the
platform allows it:

| Dependency | Bambu Studio engine | OrcaSlicer engine |
|-|-|-|
| CGAL | 5.4, commit `c58ac97e`, with Bambu's `0001-clang19.patch` | 5.6.3 (Linux, macOS); vcpkg CGAL 6 on Windows |
| OpenCASCADE | 7.6.0 static on Linux; vcpkg 7.9.3 on Windows | the same |
| libnoise | bambulab/libnoise `7e7c98c0` | SoftFever/Orca-deps-libnoise `f25d5331` |
| Boost | 1.84.0 static on Linux, with Bambu's process patch | the same |
| Assimp | required (model import) | not used |

The CGAL 5.4 headers need one Boost MPL include that newer Boost no longer
pulls in; `libslic3r/bambustudio/cgal_54_compat.hpp` adds it without changing
the engine source. The Windows OrcaSlicer build applies
`ci/patches/orca-cgal6-extract-boundary-cycles.patch` for the CGAL 6 API, and
the Windows Bambu build takes CGAL 5.4 from the overlay port in
`ci/vcpkg-overlays/cgal`.

### Linux (Ubuntu 22.04) and macOS

`install_deps.sh` installs the system packages (apt or Homebrew) and builds
the pinned CGAL, OpenCASCADE (Linux) and libnoise for one engine:

```sh
ENGINE=bambu ./install_deps.sh
ENGINE=orca ./install_deps.sh
```

On Linux the release build also links a static Boost 1.84.0; the
`linux-boost` job of the workflow shows how it is built (the Boost source,
its checksum and the Bambu patch it takes).

### Windows

Dependencies come from vcpkg at the commit the workflow pins
(`de03680510e75aa77d07750159adac4e570a6a81`):

```powershell
vcpkg install tbb eigen3 boost cgal "opencv[core]" assimp opencascade qhull cereal nlopt `
  openssl libpng zlib expat freetype libjpeg-turbo --triplet x64-windows
# Bambu Studio engine: add --overlay-ports=ci/vcpkg-overlays for CGAL 5.4
```

libnoise is built from the engine's pinned source, as in the workflow's
"libnoise" step.

## Configure and build

Linux and macOS:

```sh
cmake -S . -B build-bambu -G Ninja -DCMAKE_BUILD_TYPE=Release -DENGINE=bambu \
  -DCGAL_DIR=<the CGAL 5.4 prefix>/lib/cmake/CGAL
cmake --build build-bambu --target slicer_cli

cmake -S . -B build-orca -G Ninja -DCMAKE_BUILD_TYPE=Release -DENGINE=orca \
  -DCGAL_DIR=<the CGAL 5.6.3 prefix>/lib/cmake/CGAL
cmake --build build-orca --target slicer_cli
```

Add the prefixes of anything you built yourself (Boost, libnoise, qhull) to
`-DCMAKE_PREFIX_PATH`.

Windows (from a Visual Studio developer prompt):

```powershell
cmake -S . -B build -G Ninja -DENGINE=bambu `
  -DCMAKE_TOOLCHAIN_FILE="$env:VCPKG_ROOT\scripts\buildsystems\vcpkg.cmake" `
  -DCGAL_DIR="$env:VCPKG_ROOT\installed\x64-windows\share\cgal" `
  -DCMAKE_PREFIX_PATH="<libnoise prefix>"
cmake --build build --config Release --target slicer_cli
```

Both trees build a program named `slicer_cli`; the package step renames the
OrcaSlicer one to `slicer_cli-orcaslicer`. A first build compiles the whole
engine library (about 1,500 files) and takes a while.

## Packages

`scripts/bundle-linux.sh` and `scripts/bundle-macos.sh` put a program, its
libraries and the engine's `resources` (profiles and engine data) into the
layout the release archives use. The program looks for its profiles in
`resources` beside it (`../resources` on Linux, where it sits in `bin/`).
`scripts/smoke-clean-host.sh` checks a package on a machine without the build
tools.

## Tests

The tests run against packaged programs:

```sh
tests/test-complete-slice-path.sh <package>/bin/slicer_cli <package>/bin/slicer_cli-orcaslicer
tests/test_diagnostic_events.sh <package>/bin/slicer_cli
tests/test_excluded_features.sh <package>/bin/slicer_cli
tests/test-packaged-slice.sh ...
```

`test-complete-slice-path.sh` covers both engines: the flags, the plate type
rules, model files other than STL, the result document and the exported
projects.

## The override layer

The engine source under `references/` is never edited. When a libslic3r file
has to differ for a command-line build, a file with the same relative path
goes under `libslic3r/bambustudio/` or `libslic3r/orcaslicer/`, and CMake
compiles it in place of the engine's file (the "override layer" section of
`CMakeLists.txt`). Each override says what it changes and why, citing the
engine lines it replaces.

The files the build leaves out of each engine, and why:

| File | Why |
|-|-|
| `PostProcessor.cpp` | runs user scripts on the G-code; neither official command line runs them |
| `GCodeSender.cpp` | sends G-code to a printer over the network |
| `OpenVDBUtils.cpp`, `SLA/Hollowing.cpp` | SLA only; needs OpenVDB |
| `CutSurface.cpp` | mesh cutting; needs the CGAL 5 API on every platform |
| `PressureEqualizer.cpp` | not used by the FDM profiles |
| `LogSink.cpp` | logs into a GUI widget |
| `NSVGUtils.cpp` (Bambu) | replaced by the override without the wxWidgets image path |

## Moving an engine pin

Moving a submodule to a newer engine commit can change the dependencies and
the override layer. Check the engine's own `deps` folder for version changes,
rebuild both engines on all three systems, and run the tests. The workflow's
caches are keyed by the pinned versions.
