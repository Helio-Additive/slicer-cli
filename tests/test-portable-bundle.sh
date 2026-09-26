#!/usr/bin/env bash
set -euo pipefail

PACKAGE_ROOT="${1:?Usage: $0 <package-root>}"
PACKAGE_ROOT="$(cd "$PACKAGE_ROOT" && pwd -P)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
FIXTURES="$SCRIPT_DIR/fixtures"

test -x "$PACKAGE_ROOT/bin/slicer_cli"
test -x "$PACKAGE_ROOT/bin/slicer_cli-orcaslicer"
test -f "$PACKAGE_ROOT/bin/manifest.json"
# The archive publishes binaries only under bin/, with no symlinks anywhere.
test ! -e "$PACKAGE_ROOT/slicer_cli"
test ! -e "$PACKAGE_ROOT/slicer_cli-orcaslicer"
test ! -e "$PACKAGE_ROOT/manifest.json"
test -z "$(find "$PACKAGE_ROOT" -type l -print -quit)"
test -d "$PACKAGE_ROOT/resources/profiles/BBL/machine"
test -f "$PACKAGE_ROOT/resources/profiles/BBL.json"
test -d "$PACKAGE_ROOT/resources/profiles-orca/Snapmaker/machine"
test -n "$(find "$PACKAGE_ROOT/THIRD_PARTY_LICENSES" -type f -print -quit)"
test -f "$FIXTURES/calib_base.3mf"

# The CLI reads explicit JSON overrides; it does not walk `inherits`. Resolve
# the packaged Snapmaker parents on the test host, as a calling app must do.
CONFIG_DIR="$(mktemp -d)"
trap 'rm -rf "$CONFIG_DIR"' EXIT
python3 "$SCRIPT_DIR/resolve-orca-profiles.py" "$PACKAGE_ROOT/resources/profiles-orca/Snapmaker" "$CONFIG_DIR/config.json"

# Mount only the archive and test inputs: source-checkout fallbacks must not
# satisfy profile lookup, and network access must not supply missing inputs.
docker run --rm --platform linux/amd64 --network none -i \
    -v "$PACKAGE_ROOT:/package-input:ro" \
    -v "$FIXTURES:/fixtures:ro" \
    -v "$CONFIG_DIR:/resolved:ro" \
    --workdir /tmp \
    ubuntu:22.04 \
    sh -seu <<'CONTAINER'
# Use the container filesystem as an extracted installation would. In
# particular, preset staging must not depend on host-bind copy_file semantics.
cp -R /package-input /package
/package/bin/slicer_cli --help >/dev/null
/package/bin/slicer_cli-orcaslicer --help >/dev/null
test -f /package/bin/manifest.json
test -z "$(find /package -type l -print -quit)"

# This existing 3MF names all three Bambu presets. A successful process alone
# is insufficient: explicitly reject the silent flat-config fallback.
if ! /package/bin/slicer_cli /fixtures/calib_base.3mf -o /tmp/bambu.gcode > /tmp/bambu.log 2>&1; then
    cat /tmp/bambu.log
    exit 1
fi
cat /tmp/bambu.log
grep -F "  Engine resources: /package/resources" /tmp/bambu.log
grep -F "Preset match: printer=1 (resolved='Bambu Lab X1 Carbon 0.4 nozzle') print=1 (resolved='0.20mm Standard @BBL X1C') filament=1 (resolved='Bambu PLA Basic @BBL X1C')" /tmp/bambu.log
if grep -F 'WARNING: Using flat 3MF config' /tmp/bambu.log; then
    exit 1
fi
# libslic3r logs a failed resource read and continues on its hardcoded tables
# (Print.cpp get_filament_temp_type: "parse info/filament_info.json got a …
# parse_error"); a swallowed read must fail the package.
if grep -E 'parse (.*/)?(info|flush|filament_mixing)/|PresetBundle exception' /tmp/bambu.log; then
    exit 1
fi
test -s /tmp/bambu.gcode
grep -Eq '^G1 .*X.*Y.*E[0-9]' /tmp/bambu.gcode

# A closed tetrahedron exercises Orca STL loading with complete settings
# resolved from its packaged profiles, including the parent configurations.
cat > /tmp/model.stl <<'STL'
solid portable_test
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
endsolid portable_test
STL
if ! /package/bin/slicer_cli-orcaslicer /tmp/model.stl \
    --config /resolved/config.json \
    --machine '/package/resources/profiles-orca/Snapmaker/machine/Snapmaker U1 (0.4 nozzle).json' \
    --filament '/package/resources/profiles-orca/Snapmaker/filament/Snapmaker PLA @U1.json' \
    --process '/package/resources/profiles-orca/Snapmaker/process/0.20 Standard @Snapmaker U1 (0.4 nozzle).json' \
    -o /tmp/orca.gcode > /tmp/orca.log 2>&1; then
    cat /tmp/orca.log
    exit 1
fi
cat /tmp/orca.log
# The Orca engine's own root: resources/orca (its info/*.json and flush/*.txt
# are not BambuStudio's bytes). Print::get_hrc_by_nozzle_type reads
# info/nozzle_info.json on every slice; a swallowed read logs "parse …".
grep -F "  Engine resources: /package/resources/orca" /tmp/orca.log
if grep -E 'parse (.*/)?(info|flush)/' /tmp/orca.log; then
    exit 1
fi
test -s /tmp/orca.gcode
if ! grep -Eq '^G1 .*X.*Y.*E([0-9]|\.[0-9])' /tmp/orca.gcode; then
    echo 'FAIL: Orca output has no XY extrusion moves'
    head -n 80 /tmp/orca.gcode
    exit 1
fi
echo 'PASS: both packaged engines sliced in the clean offline container'
CONTAINER
