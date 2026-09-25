#!/usr/bin/env bash
# Slice the Bambu fixture with a PACKAGED bambu engine and require that the
# presets resolve from the package and that no resource read was swallowed.
# Usage: tests/test-packaged-slice.sh /path/to/packaged/slicer_cli[.exe]
set -euo pipefail

BINARY="${1:?usage: $0 /path/to/packaged/slicer_cli}"
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

# The fixture names all three Bambu presets; the packaged profiles tree and
# its BBL.json vendor index must resolve them (flat-config fallback is a fail).
grep -F "Preset match: printer=1 (resolved='Bambu Lab X1 Carbon 0.4 nozzle') print=1 (resolved='0.20mm Standard @BBL X1C') filament=1 (resolved='Bambu PLA Basic @BBL X1C')" "$WORKDIR/slice.log"
if grep -F 'WARNING: Using flat 3MF config' "$WORKDIR/slice.log"; then
    exit 1
fi
# libslic3r logs a failed resource read and continues on its hardcoded tables
# (Print.cpp get_filament_temp_type: "parse info/filament_info.json got a …
# parse_error"); a swallowed read must fail the package.
if grep -E 'parse (info|flush|filament_mixing)/|PresetBundle exception' "$WORKDIR/slice.log"; then
    exit 1
fi
test -s "$WORKDIR/out.gcode"
grep -Eq '^G1 .*X.*Y.*E[0-9]' "$WORKDIR/out.gcode"
echo "PASS: packaged slice resolved presets and read resources"
