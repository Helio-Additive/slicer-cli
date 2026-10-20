#!/usr/bin/env bash
# test_find_nlopt.sh: cmake/find_nlopt.cmake always gives libnest2d NLopt::nlopt.
#
# Usage: tests/test_find_nlopt.sh [path/to/find_nlopt.cmake]
#
# Fixture prefixes hold the two NLoptConfig.cmake packages Ubuntu installs
# (cmake/nlopt -> NLopt::nlopt, cmake/nlopt_cxx -> NLopt::nlopt_cxx), made in
# both orders, so whichever order the filesystem lists them in, one prefix
# puts nlopt_cxx first. NLopt_DIR set to nlopt_cxx must stop configure with a
# sentence naming the package to use.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
MODULE="$(cd "$(dirname "${1:-$SCRIPT_DIR/../cmake/find_nlopt.cmake}")" && pwd -P)/$(basename "${1:-find_nlopt.cmake}")"
W="$(mktemp -d)"
trap 'rm -rf "$W"' EXIT
FAILS=0
fail() { echo "FAIL: $*"; FAILS=$((FAILS + 1)); }

# package DIR NAME: an NLoptConfig.cmake in DIR/lib/cmake/NAME defining NLopt::NAME
package() {
    mkdir -p "$1/lib/cmake/$2"
    printf 'if(NOT TARGET NLopt::%s)\n  add_library(NLopt::%s INTERFACE IMPORTED)\nendif()\n' "$2" "$2" \
        > "$1/lib/cmake/$2/NLoptConfig.cmake"
}

# configure NAME CMAKE-ARGS...: configure a project that includes the module
# and needs NLopt::nlopt in a subdirectory, as libnest2d does; prints the log
configure() {
    local p="$W/proj-$1"; shift
    mkdir -p "$p/sub"
    printf 'cmake_minimum_required(VERSION 3.13)\nproject(t NONE)\ninclude("%s")\nmessage(STATUS "picked ${NLopt_DIR}")\nadd_subdirectory(sub)\n' \
        "$MODULE" > "$p/CMakeLists.txt"
    printf 'if(NOT TARGET NLopt::nlopt)\n  message(FATAL_ERROR "no NLopt::nlopt in a subdirectory")\nendif()\n' \
        > "$p/sub/CMakeLists.txt"
    cmake -S "$p" -B "$p/build" "$@" 2>&1
}

for order in "nlopt_cxx nlopt" "nlopt nlopt_cxx"; do
    prefix="$W/both-${order// /-}"
    for name in $order; do package "$prefix" "$name"; done
    # CMAKE_PREFIX_PATH is searched before the system's own NLopt
    log=$(configure "both-${order// /-}" -DCMAKE_PREFIX_PATH="$prefix")
    if [ $? -ne 0 ]; then
        fail "packages made in the order '$order': configure failed"; echo "$log" | grep -A4 "CMake Error" | head -n 6
    elif ! echo "$log" | grep -q "picked .*/lib/cmake/nlopt$"; then
        fail "packages made in the order '$order': $(echo "$log" | grep -m1 'picked'), not cmake/nlopt"
    else
        echo "PASS: packages made in the order '$order': cmake/nlopt picked, NLopt::nlopt defined"
    fi
done

prefix="$W/cxx-only"
package "$prefix" nlopt_cxx
log=$(configure cxx-only -DNLopt_DIR="$prefix/lib/cmake/nlopt_cxx")
if [ $? -eq 0 ]; then
    fail "NLopt_DIR at cmake/nlopt_cxx: configure passed without NLopt::nlopt"
elif ! echo "$log" | grep -q "has no NLopt::nlopt target"; then
    fail "NLopt_DIR at cmake/nlopt_cxx: configure failed without the sentence naming the package"; echo "$log" | grep -A4 "CMake Error" | head -n 6
else
    echo "PASS: NLopt_DIR at cmake/nlopt_cxx: configure stops and names cmake/nlopt"
fi

[ $FAILS -eq 0 ] || { echo "$FAILS check(s) failed"; exit 1; }
