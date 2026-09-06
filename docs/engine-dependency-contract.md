# Engine dependency contract

The BambuStudio and OrcaSlicer engines are compiled as separate binaries from
unchanged pinned references. A dependency that is part of an engine's source
contract must retain that engine's upstream version and provenance.

## Aligned dependencies

- BambuStudio uses CGAL 5.4 plus its upstream Clang 19 patch.
- OrcaSlicer uses CGAL 5.6.3 on Linux and in macOS CI builds.
- Linux uses static OpenCASCADE 7.6.0, matching both pinned engine recipes.
- Each engine uses its own pinned libnoise source: BambuStudio v1.0.0 commit
  `7e7c98c06a67d5203dd780b45e9a25d3ec930fd8`; OrcaSlicer commit
  `f25d5331570ae109f0e645cb729ecab155612714`.

## Compatibility exceptions

The CLI does not yet reproduce each upstream project's complete private
dependency prefix. Ubuntu, Homebrew, and the pinned Windows vcpkg baseline
provide non-core dependencies. Windows currently uses vcpkg CGAL 6.1.1 and
OpenCASCADE 7.9.3 rather than Orca CGAL 5.6.3 and upstream OpenCASCADE 7.6.0.
Those combinations are compatibility-tested by CI but are not upstream parity.

Local macOS Orca setup (`ENGINE=orca ./install_deps.sh`) installs Homebrew's
current CGAL. The Apple Orca CMake path uses `find_package(CGAL REQUIRED)`;
macOS CI explicitly supplies `CGAL_DIR` for its pinned 5.6.3 prefix. The local
Homebrew setup therefore does not reproduce that CI pin by default.

Removing these exceptions requires per-engine dependency prefixes built from
the pinned upstream `deps` projects. That migration must preserve the existing
headless target, package contract, and all three platform builds; it must not
modify either reference or copy engine source into this repository.

## Distribution qualification

Linux archives target Ubuntu 22.04/glibc 2.35, carry their non-glibc dynamic
dependency closure under `lib/`, and are tested in a network-disabled minimal
Ubuntu 22.04 container with a Bambu 3MF slice (including preset resolution) and
an Orca STL slice using packaged profiles. Linux executables live in `bin/`
so Bambu can find `../resources/profiles`, including the `BBL.json` vendor
index; relative symlinks retain the published package entry points.
macOS bundling fails on unresolved non-system dylibs or
remaining Homebrew/build paths. Release publication requires every supported
package job to succeed.
