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
| Bambu Studio | `references/BambuStudio` | `926a719` | 02.08.02.61 |
| OrcaSlicer | `references/OrcaSlicer` | `8500fcd` | 2.4.2 |

Both submodules point at the upstream projects' official repositories and are
pinned to the commit of the release tag: `bambulab/BambuStudio` tag
`v02.08.02.61` and `OrcaSlicer/OrcaSlicer` tag `v2.4.2`. The engine source is
used as it is (see "The override layer" below); no Helio commit lives inside
`references/`.

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

An override is a whole-file copy, so it also freezes that file at the commit
it was taken from: an engine bump does not reach it. Regenerate it from the
new pinned file with the same change re-applied, or the engine's own fixes to
that file are lost.

### Files that shadow engine source

| File | What it changes | In the current pin? | Can it move to the front end? |
|-|-|-|-|
| `libslic3r/bambustudio/libslic3r/Fill/FillFloatingConcentric.cpp` | Five crash/UB fixes: `is_floating` is kept the same length as `points` in both `rebase_at` (write the closure flag in place, never append); the floating-vertical-shell flag is read from the surviving line rather than one just erased; the extracted line's float flags are captured before `erase`/`insert` invalidate it; `prev`/`next` fall back to `curr` at the path ends instead of reading `path[-1]`/`path[size]`; the grid point buffer is owned by a vector that outlives the `EdgeGrid::Grid` query. Plus the engine's local `"../X.hpp"` includes rewritten as `<libslic3r/X.hpp>`. | No. The file is byte-identical between the previous pin and v02.08.02.61, and between v02.08.02.61 and the newest tag v02.08.04.61; no upstream commit has touched it since 2025-05-17. | No. All five are inside the fill path, and there is no front-end call site: the corrupting reads happen while a contour is resampled. The motivating input is `LV_nano_bag_rel_multipart.3mf` plates 2 and 3 (PR #25), where Windows PageHeap reported heap failure `0xC0000374`; the two use-after-frees were measured on a build whose only difference from the pin was elsewhere in the front end. |

The rest of the layer is dependency compatibility, not engine logic:
`libslic3r/bambustudio/libigl/` (Eigen `DynamicSparseMatrix` API replacements)
and `libslic3r/bambustudio/cgal_54_compat.hpp` (the Boost.MPL include CGAL 5.4
used to get transitively).

### The Print.cpp overrides are gone

Both engines' `Print.cpp` used to be shadowed for one hunk in
`Print::get_physical_unprintable_filaments`, which reads
`m_config.filament_printable.values[filament_idx]` with no bounds check
(OrcaSlicer v2.4.2 `Print.cpp:3229`, BambuStudio v02.08.02.61 `Print.cpp:3091`)
while walking every used filament. The vector is short or empty only on this
build's flat-config path; the desktop sizes it before slicing.

`filament_printable` is a filament option (`Preset.cpp:1310` at v2.4.2,
`Preset.cpp:1088` at v02.08.02.61), so the desktop's own rule reaches it:
`Preset::normalize()` sizes every filament vector to the filament count,
filling a slot it has to add with that option's definition default
(`Preset.cpp:444-486` at v2.4.2, `442-487` at v02.08.02.61), and
`PresetBundle::full_fff_config` then assembles the slicing config out of the
normalized presets. No vector the engine slices with is ever shorter than the
roster.

The front end now does the same, so neither override is needed:

| Engine | Where |
|-|-|
| Bambu | `align_per_filament_config_vectors()` in `main.cpp`, called before `print.apply()`. It sizes every key `is_per_filament_config_key()` names — `filament_printable` included — to the roster, and `ensure_vector_config_sizes()` gives a missing or empty `filament_printable` its definition default first. |
| Orca | `normalize_filament_config_vectors()` in `main.cpp`, called immediately before `print.apply()`. It applies the `Preset::normalize` rule over the engine's own `Preset::filament_options()` list. Only a vector shorter than the roster is resized; one already at the roster length keeps every value it holds, so a project's own `filament_printable` is never overwritten. |

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
