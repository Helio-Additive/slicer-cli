#!/bin/bash
# Local, no-sudo replacements for install_deps.sh's /usr/local and /opt installs (D-001).
set -euo pipefail
W="$(cd "$(dirname "$0")/../.." && pwd)"
P="$W/deps-prefix"; mkdir -p "$P" "$W/deps-src"
# CGAL 5.6.3 (Orca engine pin on macOS CI), header-only install.
if [ ! -f "$P/cgal-5.6.3/lib/cmake/CGAL/CGALConfig.cmake" ]; then
  curl -fsSL -o "$W/deps-src/CGAL-5.6.3.tar.xz" https://github.com/CGAL/cgal/releases/download/v5.6.3/CGAL-5.6.3.tar.xz
  echo "15c743cb395d1a0855b9062525f3ae0cd40486489acfe7ce1457c3710ab34111  $W/deps-src/CGAL-5.6.3.tar.xz" | shasum -a 256 -c -
  tar -xJf "$W/deps-src/CGAL-5.6.3.tar.xz" -C "$W/deps-src"
  cmake -S "$W/deps-src/CGAL-5.6.3" -B "$W/deps-src/CGAL-5.6.3/build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$P/cgal-5.6.3"
  cmake --install "$W/deps-src/CGAL-5.6.3/build"
fi
# libnoise, Orca's pinned fork.
build_libnoise() { # name repo commit
  local d="$W/deps-src/libnoise-$1"
  [ -f "$P/libnoise-$1/lib/libnoise.a" ] || [ -f "$P/libnoise-$1/lib/libnoise.dylib" ] && return 0
  rm -rf "$d"; git init -q "$d"; git -C "$d" remote add origin "$2"
  git -C "$d" fetch -q --depth=1 origin "$3"; git -C "$d" checkout -q --detach "$3"
  cmake -S "$d" -B "$d/build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$P/libnoise-$1" -DCMAKE_POLICY_VERSION_MINIMUM=3.5
  cmake --build "$d/build" --parallel; cmake --install "$d/build"
}
build_libnoise orca https://github.com/SoftFever/Orca-deps-libnoise.git f25d5331570ae109f0e645cb729ecab155612714
echo "local deps OK: $P"
