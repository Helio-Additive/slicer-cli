#!/usr/bin/env bash
# Slice the Bambu fixture with a PACKAGED bambu engine and require that the
# presets resolve from the package and that no resource read was swallowed.
# Usage: tests/test-packaged-slice.sh /path/to/packaged/slicer_cli[.exe]
set -euo pipefail

BINARY="${1:?usage: $0 /path/to/packaged/slicer_cli [/path/to/packaged/slicer_cli-orcaslicer]}"
ORCA_BINARY="${2:-}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
FIXTURE="$SCRIPT_DIR/fixtures/calib_base.3mf"
WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT

test -f "$FIXTURE"
if ! "$BINARY" "$FIXTURE" -o "$WORKDIR/out.gcode" > "$WORKDIR/slice.log" 2>&1; then
    cat "$WORKDIR/slice.log"
    exit 1
fi
cat "$WORKDIR/slice.log"

# The engine prints the resources root it configured (main.cpp
# configure_engine_resources()); it must be the package's, not "<not found>".
grep -E "^  Engine resources: .*[/\\\\]resources$" "$WORKDIR/slice.log"
# The fixture names all three Bambu presets; the packaged profiles tree and
# its BBL.json vendor index must resolve them (flat-config fallback is a fail).
grep -F "Preset match: printer=1 (resolved='Bambu Lab X1 Carbon 0.4 nozzle') print=1 (resolved='0.20mm Standard @BBL X1C') filament=1 (resolved='Bambu PLA Basic @BBL X1C')" "$WORKDIR/slice.log"
if grep -F 'WARNING: Using flat 3MF config' "$WORKDIR/slice.log"; then
    exit 1
fi
# libslic3r logs a failed resource read and continues on its hardcoded tables
# (Print.cpp get_filament_temp_type: "parse info/filament_info.json got a …
# parse_error"); a swallowed read must fail the package.
if grep -E 'parse (.*[/\\])?(info|flush|filament_mixing)[/\\]|PresetBundle exception' "$WORKDIR/slice.log"; then
    exit 1
fi
test -s "$WORKDIR/out.gcode"
grep -Eq '^G1 .*X.*Y.*E[0-9]' "$WORKDIR/out.gcode"
echo "PASS: packaged slice resolved presets and read resources"

# The Orca engine has its own root (resources/orca): its info/*.json and
# flush/*.txt differ from BambuStudio's. Print::get_hrc_by_nozzle_type reads
# info/nozzle_info.json on every slice, so the root must resolve and no
# read may be swallowed ("parse …/info/nozzle_info.json … unexpected end of input").
if [ -n "$ORCA_BINARY" ]; then
    ORCA_ROOT="$(cd "$(dirname "$ORCA_BINARY")" && pwd -P)"
    cat > "$WORKDIR/model.stl" <<'STL'
solid packaged_test
facet normal 0 0 -1
outer loop
vertex 110 110 0
vertex 110 120 0
vertex 120 110 0
endloop
endfacet
facet normal 0 -1 0
outer loop
vertex 110 110 0
vertex 120 110 0
vertex 110 110 10
endloop
endfacet
facet normal -1 0 0
outer loop
vertex 110 110 0
vertex 110 110 10
vertex 110 120 0
endloop
endfacet
facet normal 1 1 1
outer loop
vertex 120 110 0
vertex 110 120 0
vertex 110 110 10
endloop
endfacet
endsolid packaged_test
STL
    python3 "$SCRIPT_DIR/resolve-orca-profiles.py" "$ORCA_ROOT/resources/profiles-orca/Snapmaker" "$WORKDIR/orca-config.json"
    if ! "$ORCA_BINARY" "$WORKDIR/model.stl" \
        --config "$WORKDIR/orca-config.json" \
        --machine "$ORCA_ROOT/resources/profiles-orca/Snapmaker/machine/Snapmaker U1 (0.4 nozzle).json" \
        --filament "$ORCA_ROOT/resources/profiles-orca/Snapmaker/filament/Snapmaker PLA @U1.json" \
        --process "$ORCA_ROOT/resources/profiles-orca/Snapmaker/process/0.20 Standard @Snapmaker U1 (0.4 nozzle).json" \
        -o "$WORKDIR/orca.gcode" > "$WORKDIR/orca.log" 2>&1; then
        cat "$WORKDIR/orca.log"
        exit 1
    fi
    cat "$WORKDIR/orca.log"
    grep -E "^  Engine resources: .*[/\\\\]resources[/\\\\]orca$" "$WORKDIR/orca.log"
    if grep -E 'parse (.*[/\\])?(info|flush)[/\\]' "$WORKDIR/orca.log"; then
        exit 1
    fi
    test -s "$WORKDIR/orca.gcode"
    # Orca writes extrusion amounts without a leading zero (E.12345).
    if ! grep -Eq '^G1 .*X.*Y.*E([0-9]|\.[0-9])' "$WORKDIR/orca.gcode"; then
        echo 'FAIL: Orca output has no XY extrusion moves'
        head -n 80 "$WORKDIR/orca.gcode"
        exit 1
    fi
    echo "PASS: packaged orca slice resolved its own resources root"
fi
