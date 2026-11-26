# Engine dependency contract

The BambuStudio, OrcaSlicer and Snapmaker Orca engines are compiled as separate binaries from
unchanged pinned references. A dependency that is part of an engine's source
contract must retain that engine's upstream version and provenance.

## Aligned dependencies

- BambuStudio uses CGAL 5.4 plus its upstream Clang 19 patch.
- OrcaSlicer uses CGAL 6.2.1 on Linux and in macOS CI builds (the archive and
  hash of deps/CGAL/CGAL.cmake at `f92a7c0`).
- Snapmaker Orca uses the CGAL 5.4 archive and the `0001-clang19.patch` that
  BambuStudio pins (deps/CGAL at `bac6fd5`), and its own bundled Eigen 3.3.7.
- Linux uses a static OpenCASCADE per engine, each matching its pinned recipe:
  7.6.0 for BambuStudio and Snapmaker Orca, and 8.0.1 with the
  `0001-BRepMesh-seam-pcurve-selection.patch` OrcaSlicer applies (deps/OCCT at
  `f92a7c0`; its second patch is for MSVC on ARM64 only).
- OrcaSlicer uses that static OpenCASCADE 8.0.1 with the patch on macOS and
  Windows too: `install_deps.sh` builds it on macOS, and on Windows the overlay
  port `ci/vcpkg-overlays-orca/opencascade` builds it as static libraries
  (`ci/vcpkg-triplets-orca`). OrcaSlicer's own Windows build makes shared
  libraries; this package cannot, because its one folder already holds the
  7.9.3 DLLs of the same names for the other two engines.
- Each engine uses its own pinned libnoise source: BambuStudio v1.0.0 commit
  `7e7c98c06a67d5203dd780b45e9a25d3ec930fd8`; OrcaSlicer and Snapmaker Orca
  commit `f25d5331570ae109f0e645cb729ecab155612714` (tag 1.0 of
  SoftFever/Orca-deps-libnoise).

## Compatibility exceptions

The CLI does not yet reproduce each upstream project's complete private
dependency prefix. Ubuntu, Homebrew, and the pinned Windows vcpkg baseline
provide non-core dependencies. For BambuStudio and Snapmaker Orca, Windows
uses vcpkg OpenCASCADE 7.9.3 rather than upstream OpenCASCADE 7.6.0, and macOS
takes Homebrew's OpenCASCADE. The Windows OrcaSlicer CGAL is the pinned 6.2.1,
through the overlay port `ci/vcpkg-overlays-orca/cgal`.
Those combinations are compatibility-tested by CI but are not upstream parity.

Local macOS Orca setup (`ENGINE=orca ./install_deps.sh`) installs Homebrew's
current CGAL. The Apple Orca CMake path uses `find_package(CGAL REQUIRED)`;
macOS CI explicitly supplies `CGAL_DIR` for its pinned 6.2.1 prefix. The local
Homebrew setup therefore does not reproduce that CI pin by default.

Removing these exceptions requires per-engine dependency prefixes built from
the pinned upstream `deps` projects. That migration must preserve the existing
headless target, package contract, and all three platform builds; it must not
modify either reference or copy engine source into this repository.

## Distribution qualification

Linux archives target Ubuntu 22.04/glibc 2.35, carry their non-glibc dynamic
dependency closure under `lib/`, and are tested in a network-disabled minimal
Ubuntu 22.04 container with a Bambu 3MF slice (including preset resolution) and
an Orca STL slice and a Snapmaker STL slice using packaged profiles. Linux
executables live in `bin/` so Bambu can find `../resources/profiles`,
including the `BBL.json` vendor index; the published Linux entry points are
`bin/slicer_cli`, `bin/slicer_cli-orcaslicer` and `bin/slicer_cli-snapmaker`
(named by `bin/manifest.json`), and the archive
carries no root-level launchers, symlinks or hardlinks.
macOS bundling fails on unresolved non-system dylibs or
remaining Homebrew/build paths. Release publication requires every supported
package job to succeed.
