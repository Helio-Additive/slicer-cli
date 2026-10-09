# Bambu Studio engine: dependency notes

These notes record why the Bambu Studio engine build takes the dependencies it
does. [building.md](building.md) has the build steps.

## Assimp

The Bambu Studio source needs Assimp through `libslic3r`
(`find_package(assimp REQUIRED)` and `assimp::assimp`). slicer-cli provides it
itself rather than through Bambu Studio's private dependency build:

| Platform | Provisioning |
|-|-|
| macOS | Homebrew `assimp` in `install_deps.sh` |
| Debian/Ubuntu | `libassimp-dev` in `install_deps.sh` and the Linux CI install step |
| Windows | `assimp:x64-windows` in the pinned-vcpkg CI install step |

libnoise is built from Bambu's fork by `install_deps.sh` and by all three CI
platforms.

## CGAL 5.4

Bambu Studio's own `deps/CGAL/CGAL.cmake` pins CGAL v5.4 plus its
`0001-clang19.patch`. Its `MeshBoolean.cpp` calls
`CGAL::Polygon_mesh_processing::extract_boundary_cycles`, which CGAL 6 moved
to the top-level `CGAL` namespace, so the engine needs CGAL 5.4, not the 6.x
package Homebrew and vcpkg ship. The CGAL 5.4 headers rely on
`boost::mpl::if_c` arriving through another Boost header; `cgal_54_compat.hpp`
includes it for both `libslic3r_cgal` and `libslic3r_core` without changing
the engine source.

| Platform | Bambu-patched CGAL 5.4 provisioning |
|-|-|
| macOS | A project-managed Cellar keg at `$(brew --cellar)/cgal@5/5.4`, passed as `CGAL_DIR` |
| Debian/Ubuntu | A checked source install at `/opt/slicer-cli/cgal-5.4`, passed as `CGAL_DIR` |
| Windows | The Bambu-patched CGAL 5.4 vcpkg overlay (`ci/vcpkg-overlays/cgal`) |

CGAL 5.4 also needs `gmp` and `mpfr` (and `unzip` on Debian/Ubuntu); Windows
gets them through the vcpkg overlay.

The archive checksum in Bambu Studio's manifest no longer matches GitHub's
regenerated tag archive, so the build locks the `v5.4` tag commit instead and
checks it after the fetch on every platform:
`c58ac97e93c838ebfb1e8adaf23ff4fd185dc8e4`.
