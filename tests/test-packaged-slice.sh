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

# Staging must also work when the profiles dir and the temp dir are on
# different filesystems. boost::filesystem::copy_file uses the copy_file_range
# syscall on Linux, which reports EXDEV (Invalid cross-device link) when the
# packaged vendor JSONs and TMPDIR live on different filesystems — the Linux
# host that hit this had the profiles tree on btrfs and /tmp on tmpfs. The
# vendor-JSON copy has to fall back to a stream copy (main.cpp stage_file_copy)
# rather than failing the preset load, and a staging failure must never leave
# the slice running on the flat 3MF config.
# /dev/shm is tmpfs on the Linux runners and a different device from the
# checkout the package is unpacked into.
if [ -d /dev/shm ] && [ -w /dev/shm ]; then
    XDEV_DIR="$(mktemp -d /dev/shm/slicer_cli_xdev.XXXXXX)"
    BIN_DEV="$(stat -c '%d' "$(dirname "$BINARY")" 2>/dev/null || stat -f '%d' "$(dirname "$BINARY")")"
    XDEV_DEV="$(stat -c '%d' "$XDEV_DIR" 2>/dev/null || stat -f '%d' "$XDEV_DIR")"
    if [ "$BIN_DEV" = "$XDEV_DEV" ]; then
        echo "SKIP: /dev/shm is on the same filesystem as the packaged engine"
        rm -rf "$XDEV_DIR"
    else
        echo "Presets dir and TMPDIR on different filesystems (devices $BIN_DEV vs $XDEV_DEV)"
        if ! TMPDIR="$XDEV_DIR" "$BINARY" "$FIXTURE" -o "$WORKDIR/xdev.gcode" > "$WORKDIR/xdev.log" 2>&1; then
            cat "$WORKDIR/xdev.log"
            rm -rf "$XDEV_DIR"
            exit 1
        fi
        cat "$WORKDIR/xdev.log"
        grep -F "Preset match: printer=1 (resolved='Bambu Lab X1 Carbon 0.4 nozzle') print=1 (resolved='0.20mm Standard @BBL X1C') filament=1 (resolved='Bambu PLA Basic @BBL X1C')" "$WORKDIR/xdev.log"
        if grep -E 'Invalid cross-device link|PresetBundle exception|Preset staging failed|WARNING: Using flat 3MF config' "$WORKDIR/xdev.log"; then
            rm -rf "$XDEV_DIR"
            exit 1
        fi
        test -s "$WORKDIR/xdev.gcode"
        rm -rf "$XDEV_DIR"
        echo "PASS: presets staged from a different filesystem than TMPDIR"
    fi
else
    echo "SKIP: no writable /dev/shm on this host (cross-filesystem staging case is Linux-only)"
fi

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
