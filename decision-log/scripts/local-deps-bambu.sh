#!/bin/bash
# Local, no-sudo build of the Bambu engine's pinned deps into deps-prefix/ (D-001 pattern).
# Mirrors install_deps.sh install_cgal5 (CGAL5_*) and its libnoise block (ENGINE=bambu),
# with deps-prefix/ in place of $(brew --cellar)/cgal@5/5.4 and /usr/local. Idempotent (marker files).
set -euo pipefail
W="$(cd "$(dirname "$0")/../.." && pwd)"
P="$W/deps-prefix"; S="$W/deps-src"; mkdir -p "$P" "$S"

# CGAL 5.4, Bambu-patched (install_deps.sh:29-32, 46-128).
CGAL5_VERSION=5.4
CGAL5_COMMIT=c58ac97e93c838ebfb1e8adaf23ff4fd185dc8e4
CGAL5_PATCH="$W/references/BambuStudio/deps/CGAL/0001-clang19.patch"
CGAL5_PREFIX="$P/cgal-5.4"
[ -f "$CGAL5_PATCH" ] || { echo "missing $CGAL5_PATCH (git submodule update --init --depth 1 references/BambuStudio)"; exit 1; }
patch_sha256="$(shasum -a 256 "$CGAL5_PATCH" | awk '{print $1}')"
marker="$CGAL5_PREFIX/.slicer-cli-cgal-5.4-complete"
if [ -r "$marker" ] && grep -qx "version=$CGAL5_VERSION" "$marker" && grep -qx "source_commit=$CGAL5_COMMIT" "$marker" \
   && grep -qx "patch_sha256=$patch_sha256" "$marker" && [ -f "$CGAL5_PREFIX/lib/cmake/CGAL/CGALConfig.cmake" ] \
   && [ -f "$CGAL5_PREFIX/lib/cmake/CGAL/CGALConfigVersion.cmake" ] && [ -f "$CGAL5_PREFIX/include/CGAL/version.h" ]; then
  echo "CGAL $CGAL5_VERSION OK ($CGAL5_PREFIX)"
else
  rm -rf "$CGAL5_PREFIX" "$S/cgal-5.4"
  git clone -q --depth=1 --branch "v$CGAL5_VERSION" https://github.com/CGAL/cgal.git "$S/cgal-5.4"
  [ "$(git -C "$S/cgal-5.4" rev-parse HEAD)" = "$CGAL5_COMMIT" ] || { echo "CGAL $CGAL5_VERSION commit mismatch"; exit 1; }
  patch -d "$S/cgal-5.4" -p1 < "$CGAL5_PATCH"
  cmake -S "$S/cgal-5.4" -B "$S/cgal-5.4/build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$CGAL5_PREFIX" \
    -DCGAL_HEADER_ONLY=ON -DCGAL_INSTALL_CMAKE_DIR=lib/cmake/CGAL -DBUILD_TESTING=OFF -DBUILD_DOC=OFF -DCGAL_BUILD_THREE_DOC=OFF
  cmake --install "$S/cgal-5.4/build"
  test -f "$CGAL5_PREFIX/lib/cmake/CGAL/CGALConfig.cmake"
  test -f "$CGAL5_PREFIX/lib/cmake/CGAL/CGALConfigVersion.cmake"
  test -f "$CGAL5_PREFIX/include/CGAL/version.h"
  printf 'version=%s\nsource_commit=%s\npatch_sha256=%s\n' "$CGAL5_VERSION" "$CGAL5_COMMIT" "$patch_sha256" > "$marker"
fi

# libnoise, Bambu's fork (install_deps.sh:41-44, 285-310).
LN_REPO=https://github.com/bambulab/libnoise.git
LN_COMMIT=7e7c98c06a67d5203dd780b45e9a25d3ec930fd8
LN_PREFIX="$P/libnoise-bambu"; LN_SRC="$S/libnoise-bambu"
ln_marker="$LN_PREFIX/.slicer-cli-libnoise-complete"
# The fork installs liblibnoise_static.a (as the Orca fork does); CI's prefix check accepts all three names.
ln_lib_ok() { [ -f "$LN_PREFIX/include/libnoise/noise.h" ] || [ -f "$LN_PREFIX/include/noise/noise.h" ] || return 1
  [ -f "$LN_PREFIX/lib/libnoise.a" ] || [ -f "$LN_PREFIX/lib/libnoise_static.a" ] || [ -f "$LN_PREFIX/lib/liblibnoise_static.a" ]; }
if [ -r "$ln_marker" ] && grep -qx "source_repository=$LN_REPO" "$ln_marker" && grep -qx "source_commit=$LN_COMMIT" "$ln_marker" \
   && ln_lib_ok; then
  echo "libnoise-bambu OK ($LN_PREFIX)"
else
  rm -rf "$LN_PREFIX" "$LN_SRC"
  git init -q "$LN_SRC"; git -C "$LN_SRC" remote add origin "$LN_REPO"
  git -C "$LN_SRC" fetch -q --depth=1 origin "$LN_COMMIT"; git -C "$LN_SRC" checkout -q --detach "$LN_COMMIT"
  [ "$(git -C "$LN_SRC" rev-parse HEAD)" = "$LN_COMMIT" ] || { echo "libnoise commit mismatch"; exit 1; }
  cmake -S "$LN_SRC" -B "$LN_SRC/build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$LN_PREFIX" -DCMAKE_POLICY_VERSION_MINIMUM=3.5
  cmake --build "$LN_SRC/build" --parallel
  cmake --install "$LN_SRC/build"
  ln_lib_ok || { echo "libnoise install incomplete"; exit 1; }
  printf 'source_repository=%s\nsource_commit=%s\n' "$LN_REPO" "$LN_COMMIT" > "$ln_marker"
fi
echo "bambu local deps OK: $P"
