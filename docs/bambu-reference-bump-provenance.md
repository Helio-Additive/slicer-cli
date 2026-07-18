# BambuStudio reference bump provenance

## Source pair

- Previous locked BambuStudio reference: `b506005bc4ee62124e24bf00e0f58656db3646a6`.
- Bump candidate: `ba049f6a2e08c3b6033660bb84da80c08722974b`.
- The candidate includes the post-`2f014ce1` consolidated filament-grouping
  implementation being evaluated by the Wave 5 acceptance suite.

## Build-dependency delta

The reference source now requires Assimp through `libslic3r` (`find_package(assimp
REQUIRED)` and `assimp::assimp`). The CLI owns that dependency rather than relying
on BambuStudio's private dependency build.

| Platform | Provisioning change |
| --- | --- |
| macOS | Homebrew `assimp` in `install_deps.sh`; Homebrew cache/repair marker advanced to v6. |
| Debian/Ubuntu | `libassimp-dev` in `install_deps.sh` and the Linux CI install step. |
| Windows | `assimp:x64-windows` in the pinned-vcpkg CI install step; vcpkg cache advanced to v4. |

`libnoise` is **not** part of this bump's dependency delta. It was already built
from Bambu's fork by `install_deps.sh` and all three CI platforms before this
reference change.

The bump candidate also incorporates the stale-Homebrew-cache repair from
`b82aeee`: skip the installed-dependents walk and remove formula records whose
Cellar keg is missing before the declared dependency set is installed. This is
CI cache hardening, not an engine build dependency.

## CGAL compatibility pin

BambuStudio's own `deps/CGAL/CGAL.cmake` pins CGAL v5.4 plus its
`0001-clang19.patch`. Its current `MeshBoolean.cpp` still calls
`CGAL::Polygon_mesh_processing::extract_boundary_cycles`, which CGAL 6 moved
to the top-level `CGAL` namespace. The CLI therefore installs the exact
hash-pinned, Bambu-patched CGAL 5.4 source instead of Homebrew/vcpkg's current
6.x package. The pinned 5.4 headers also rely on `boost::mpl::if_c` arriving
transitively; `cgal_54_compat.hpp` makes that Boost 1.90 dependency explicit
for both `libslic3r_cgal` and `libslic3r_core` without modifying the Bambu
submodule source.

| Platform | Bambu-patched CGAL 5.4 provisioning |
| --- | --- |
| macOS | Project-managed Cellar keg at `$(brew --cellar)/cgal@5/5.4`, passed as `CGAL_DIR`. |
| Debian/Ubuntu | Hash-verified source install at `/opt/slicer-cli/cgal-5.4`, passed as `CGAL_DIR`. |
| Windows | Bambu-patched CGAL 5.4 vcpkg overlay; the existing vcpkg baseline remains pinned for OCCT 7.9.3. |

The additional dependency delta for this pin is `gmp`, `mpfr`, and `unzip` on
Debian/Ubuntu; macOS already provides archive extraction and installs Homebrew
`gmp` and `mpfr`. Windows receives the same `gmp`/`mpfr` dependency graph from
the versioned vcpkg overlay.

CGAL 5.4 source SHA-256 (the exact BambuStudio dependency value):
`d7605e0a5a5ca17da7547592f6f6e4a59430a0bc861948974254d0de43eab4c0`.

## Local validation scope

This provenance record tracks a local, unmerged source-bump investigation only.
It does not identify a released engine, alter a runtime lock, or authorize a
remote push. Release metadata and platform binary hashes will be recorded only
after the CLI bump is reviewed, merged by Priyesh, built by macOS and Windows CI,
and published under the normal release process.
