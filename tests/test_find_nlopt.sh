#!/usr/bin/env bash
# test_find_nlopt.sh: cmake/find_nlopt.cmake always gives libnest2d NLopt::nlopt.
#
# Usage: tests/test_find_nlopt.sh [path/to/find_nlopt.cmake]
#
# Fixture prefixes hold the two NLoptConfig.cmake packages Ubuntu installs
# (cmake/nlopt -> NLopt::nlopt, cmake/nlopt_cxx -> NLopt::nlopt_cxx), made in
# both orders, so whichever order the filesystem lists them in, one prefix
# puts nlopt_cxx first; the caller's own sort settings stay as they were.
# NLopt_DIR set to nlopt_cxx must stop configure with a
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

# configure NAME CALLER CMAKE-ARGS...: configure a project that runs the
# CMake lines CALLER, includes the module and needs NLopt::nlopt in a
# subdirectory, as libnest2d does; prints the log
configure() {
    local p="$W/proj-$1" caller="$2"; shift 2
    mkdir -p "$p/sub"
    printf 'cmake_minimum_required(VERSION 3.13)\nproject(t NONE)\n%s\ninclude("%s")\nmessage(STATUS "picked ${NLopt_DIR}")\nmessage(STATUS "caller sort ${CMAKE_FIND_PACKAGE_SORT_ORDER}/${CMAKE_FIND_PACKAGE_SORT_DIRECTION}")\nadd_subdirectory(sub)\n' \
        "$caller" "$MODULE" > "$p/CMakeLists.txt"
    printf 'if(NOT TARGET NLopt::nlopt)\n  message(FATAL_ERROR "no NLopt::nlopt in a subdirectory")\nendif()\n' \
        > "$p/sub/CMakeLists.txt"
    cmake -S "$p" -B "$p/build" "$@" 2>&1
}

# check NAME CALLER-SORT PREFIX [CALLER]: cmake/nlopt is picked from PREFIX
# (searched before the system's own NLopt) and the caller's sort settings
# read CALLER-SORT afterwards
check() {
    local log
    log=$(configure "$1" "${4:-}" -DCMAKE_PREFIX_PATH="$3")
    if [ $? -ne 0 ]; then
        fail "$1: configure failed"; echo "$log" | grep -A4 "CMake Error" | head -n 6
    elif ! echo "$log" | grep -q "picked .*/lib/cmake/nlopt$"; then
        fail "$1: $(echo "$log" | grep -m1 'picked'), not cmake/nlopt"
    elif ! echo "$log" | grep -q "caller sort $2$"; then
        fail "$1: the caller's sort settings changed to $(echo "$log" | grep -m1 'caller sort' | sed 's/.*caller sort //'), not $2"
    else
        echo "PASS: $1: cmake/nlopt picked, NLopt::nlopt defined, the caller's sort settings kept"
    fi
}

for order in "nlopt_cxx nlopt" "nlopt nlopt_cxx"; do
    prefix="$W/both-${order// /-}"
    for name in $order; do package "$prefix" "$name"; done
    check "packages made in the order '$order'" "/" "$prefix"
done
# A caller sorting by version, descending, would take nlopt_cxx itself.
check "the caller sorts NATURAL/DEC" "NATURAL/DEC" "$W/both-nlopt-nlopt_cxx" \
    'set(CMAKE_FIND_PACKAGE_SORT_ORDER NATURAL)
set(CMAKE_FIND_PACKAGE_SORT_DIRECTION DEC)'

prefix="$W/cxx-only"
package "$prefix" nlopt_cxx
log=$(configure cxx-only "" -DNLopt_DIR="$prefix/lib/cmake/nlopt_cxx")
if [ $? -eq 0 ]; then
    fail "NLopt_DIR at cmake/nlopt_cxx: configure passed without NLopt::nlopt"
elif ! echo "$log" | grep -q "has no NLopt::nlopt target"; then
    fail "NLopt_DIR at cmake/nlopt_cxx: configure failed without the sentence naming the package"; echo "$log" | grep -A4 "CMake Error" | head -n 6
else
    echo "PASS: NLopt_DIR at cmake/nlopt_cxx: configure stops and names cmake/nlopt"
fi

[ $FAILS -eq 0 ] || { echo "$FAILS check(s) failed"; exit 1; }
