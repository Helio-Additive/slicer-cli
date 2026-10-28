#!/bin/bash
# compile.sh <snapmaker|orca> <file.cpp> [more.cpp ...]
# Compiles only the named front-end files in that build tree, one build at a time
# (a lock serialises all helpers' and the coordinator's builds). Errors go to stdout.
set -uo pipefail
W="$(cd "$(dirname "$0")/../.." && pwd)"; B="$W/build-$1"; shift
T=(); for f in "$@"; do T+=("CMakeFiles/slicer_cli.dir/$(basename "$f").o"); done
export CCACHE_BASEDIR="$HOME/Projects/Snapmaker" CCACHE_DIR="$W/deps-prefix/ccache"
exec /usr/bin/lockf -t 3600 "$W/deps-prefix/build.lock" ninja -C "$B" -k 0 "${T[@]}"
