#!/bin/bash
# configure.sh <engine> <build-dir> [source-dir]   — local macOS recipe mirroring CI "Configure (macOS / Linux)"
# Per-engine pinned deps in deps-prefix/ (D-001):
#   bambu     → cgal-5.4 (Bambu-patched, CI: $(brew --cellar)/cgal@5/5.4) + libnoise-bambu (bambulab/libnoise 7e7c98c0)
#                built by decision-log/scripts/local-deps-bambu.sh
#   orca      → cgal-5.6.3 (CI: /opt/slicer-cli/cgal-5.6.3) + libnoise-orca (SoftFever/Orca-deps-libnoise f25d5331)
#                built by decision-log/scripts/local-deps.sh
#   snapmaker → same as orca (the fork's libnoise pin is Orca-deps-libnoise tag 1.0 = f25d5331).
#                The fork's deps/CGAL pins CGAL 5.4 + the same clang19 patch as Bambu; set SNAPMAKER_CGAL=5.4
#                to build against deps-prefix/cgal-5.4 instead (see inbox/package.md "Snapmaker CGAL").
set -euo pipefail
ENGINE=$1; B=$2; SRC=${3:-$(cd "$(dirname "$0")/../.." && pwd)}
W="$(cd "$(dirname "$0")/../.." && pwd)"; P="$W/deps-prefix"
case "$ENGINE" in
  bambu)     CGAL_PREFIX="$P/cgal-5.4";   LIBNOISE_PREFIX="$P/libnoise-bambu" ;;
  orca)      CGAL_PREFIX="$P/cgal-5.6.3"; LIBNOISE_PREFIX="$P/libnoise-orca" ;;
  snapmaker) CGAL_PREFIX="$P/cgal-${SNAPMAKER_CGAL:-5.4}"; LIBNOISE_PREFIX="$P/libnoise-orca" ;;
  *) echo "configure.sh: unknown engine '$ENGINE' (bambu|orca|snapmaker)" >&2; exit 2 ;;
esac
[ -f "$CGAL_PREFIX/lib/cmake/CGAL/CGALConfig.cmake" ] || { echo "missing $CGAL_PREFIX (run decision-log/scripts/local-deps*.sh)" >&2; exit 1; }
[ -f "$LIBNOISE_PREFIX/include/libnoise/noise.h" ] || [ -f "$LIBNOISE_PREFIX/include/noise/noise.h" ] \
  || { echo "missing $LIBNOISE_PREFIX (run decision-log/scripts/local-deps*.sh)" >&2; exit 1; }
LIBPNG=$(brew --prefix libpng)
EIGEN3_CMAKE_DIR="$(dirname "$(find /opt/homebrew/opt/eigen/ -name Eigen3Config.cmake | head -1)")"
export CCACHE_BASEDIR="$HOME/Projects/Snapmaker" CCACHE_DIR="$W/deps-prefix/ccache" CCACHE_MAXSIZE=30G
cmake -S "$SRC" -B "$B" -G Ninja -DCMAKE_BUILD_TYPE=Release -DENGINE="$ENGINE" \
  -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
  -DCMAKE_PREFIX_PATH="$LIBNOISE_PREFIX;/opt/homebrew" \
  -DSLICER_CLI_LIBNOISE_PREFIX="$LIBNOISE_PREFIX" \
  -DPNG_PNG_INCLUDE_DIR="$LIBPNG/include" -DPNG_LIBRARY="$LIBPNG/lib/libpng.dylib" \
  -DEigen3_DIR="$EIGEN3_CMAKE_DIR" -DSLICER_CLI_OSX_DEPLOYMENT_TARGET=11.0 -DCGAL_DIR="$CGAL_PREFIX/lib/cmake/CGAL"
