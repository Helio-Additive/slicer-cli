#!/usr/bin/env bash
# Bambu provenance regressions. Assertions use the production CLI's config
# loading, explicit-map derivation, and object reassignment path.
# Usage: tests/test_nozzle_map_provenance.sh /path/to/bambu/slicer_cli

set -euo pipefail

BINARY="${1:?usage: $0 /path/to/bambu/slicer_cli}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_3MF="$SCRIPT_DIR/fixtures/calib_base.3mf"
WORKDIR="$(mktemp -d "${TMPDIR:-/tmp}/nozzle_map_provenance.XXXXXX")"
trap 'rm -rf "$WORKDIR"' EXIT
PASS=0
FAIL=0

record() {
    local label="$1" expected="$2" output="$3"
    if grep -Fq "$expected" <<<"$output"; then
        PASS=$((PASS + 1)); echo "PASS [$label]"
    else
        FAIL=$((FAIL + 1)); echo "FAIL [$label] expected: $expected"
        echo "  output: $output"
    fi
}

run_case() {
    local label="$1" input="$2" provenance="$3" derivation="$4" reassignment="$5"; shift 5
    set +e
    local output
    output=$("$BINARY" --verbose "$input" "$@" -o "$WORKDIR/$label.gcode" 2>&1)
    local status=$?
    set -e
    if [ "$status" -ne 0 ]; then
        FAIL=$((FAIL + 1)); echo "FAIL [$label] exit=$status"
        echo "  output: $output"
        return
    fi
    record "$label/provenance" "$provenance" "$output"
    record "$label/derivation" "$derivation" "$output"
    if [ -z "$reassignment" ]; then
        if grep -Fq "Nozzle-map reassignment:" <<<"$output"; then
            FAIL=$((FAIL + 1)); echo "FAIL [$label/reassignment] unexpected reassignment"
        else
            PASS=$((PASS + 1)); echo "PASS [$label/reassignment]"
        fi
    else
        record "$label/reassignment" "$reassignment" "$output"
    fi
}

run_validation_failure() {
    local label="$1" input="$2" provenance="$3" derivation="$4" reassignment="$5" validation="$6"; shift 6
    set +e
    local output
    output=$("$BINARY" --verbose "$input" "$@" -o "$WORKDIR/$label.gcode" 2>&1)
    local status=$?
    set -e
    if [ "$status" -ne 1 ]; then
        FAIL=$((FAIL + 1)); echo "FAIL [$label] exit=$status (expected validation exit 1)"
        echo "  output: $output"
        return
    fi
    record "$label/provenance" "$provenance" "$output"
    record "$label/derivation" "$derivation" "$output"
    record "$label/reassignment" "$reassignment" "$output"
    record "$label/validation" "$validation" "$output"
}

# Closed 20 mm cube: 12 triangles, suitable for a successful real slice.
cat > "$WORKDIR/cube.stl" <<'EOF'
solid cube
facet normal 0 0 -1
outer loop
vertex 0 0 0
vertex 20 20 0
vertex 20 0 0
endloop
endfacet
facet normal 0 0 -1
outer loop
vertex 0 0 0
vertex 0 20 0
vertex 20 20 0
endloop
endfacet
facet normal 0 0 1
outer loop
vertex 0 0 20
vertex 20 0 20
vertex 20 20 20
endloop
endfacet
facet normal 0 0 1
outer loop
vertex 0 0 20
vertex 20 20 20
vertex 0 20 20
endloop
endfacet
facet normal 0 -1 0
outer loop
vertex 0 0 0
vertex 20 0 0
vertex 20 0 20
endloop
endfacet
facet normal 0 -1 0
outer loop
vertex 0 0 0
vertex 20 0 20
vertex 0 0 20
endloop
endfacet
facet normal 0 1 0
outer loop
vertex 0 20 0
vertex 0 20 20
vertex 20 20 20
endloop
endfacet
facet normal 0 1 0
outer loop
vertex 0 20 0
vertex 20 20 20
vertex 20 20 0
endloop
endfacet
facet normal -1 0 0
outer loop
vertex 0 0 0
vertex 0 0 20
vertex 0 20 20
endloop
endfacet
facet normal -1 0 0
outer loop
vertex 0 0 0
vertex 0 20 20
vertex 0 20 0
endloop
endfacet
facet normal 1 0 0
outer loop
vertex 20 0 0
vertex 20 20 0
vertex 20 20 20
endloop
endfacet
facet normal 1 0 0
outer loop
vertex 20 0 0
vertex 20 20 20
vertex 20 0 20
endloop
endfacet
endsolid cube
EOF

cat > "$WORKDIR/cross-map.json" <<'EOF'
{
  "filament_nozzle_map": ["2", "1"]
}
EOF

# Make mapless and explicitly mapped copies; the checked-in fixture is untouched.
python3 - "$BASE_3MF" "$WORKDIR/mapless.3mf" "$WORKDIR/native-cross-map.3mf" "$WORKDIR/plate-cross-map.3mf" "$WORKDIR/stl-cross-map.json" <<'PY'
import json
import sys
import zipfile

source, mapless_destination, native_destination, plate_destination, stl_profile_destination = sys.argv[1:]
with zipfile.ZipFile(source) as src:
    members = [(info, src.read(info.filename)) for info in src.infolist()]
for destination, cross_map in ((mapless_destination, False), (native_destination, True)):
    with zipfile.ZipFile(destination, "w") as dst:
        for info, data in members:
            if info.filename == "Metadata/project_settings.config":
                config = json.loads(data)
                config.pop("filament_nozzle_map", None)
                config.update({
                    "filament_map": ["1", "1"],
                    "physical_extruder_map": ["1", "2"],
                    "nozzle_diameter": ["0.4", "0.4"],
                    "filament_volume_map": ["0", "0"],
                    "extruder_nozzle_stats": ["Standard#1", "Standard#1"],
                    "extruder_max_nozzle_count": ["1", "1"],
                    "nozzle_volume_type": ["Standard", "Standard"],
                    "extruder_type": ["Direct Drive", "Direct Drive"],
                    "print_extruder_id": ["1", "2"],
                    "print_extruder_variant": ["Direct Drive Standard", "Direct Drive Standard"],
                    "filament_extruder_variant": ["Direct Drive Standard", "Direct Drive Standard"],
                    "flush_multiplier": ["1", "1"],
                    "flush_multiplier_fast": ["1", "1"],
                    "flush_volumes_matrix": ["0", "0", "0", "0", "0", "0", "0", "0"],
                    "filament_map_mode": "Auto For Flush",
                })
                if cross_map:
                    config["filament_nozzle_map"] = ["2", "1"]
                else:
                    stl_profile = dict(config)
                    stl_profile["filament_nozzle_map"] = ["2", "1"]
                    with open(stl_profile_destination, "w") as profile:
                        json.dump(stl_profile, profile, indent=2)
                data = json.dumps(config, indent=2).encode()
            dst.writestr(info, data)

# Keep a 1-nozzle-per-extruder variant where a plate-level diverse
# filament_maps value drives explicit mapping. This must skip re-derivation and
# disable master-object reassignment.
with zipfile.ZipFile(plate_destination, "w") as dst:
    with zipfile.ZipFile(mapless_destination) as src:
        for info in src.infolist():
            data = src.read(info.filename)
            if info.filename == "Metadata/model_settings.config":
                text = data.decode()
                text = text.replace('key="filament_maps" value="1"', 'key="filament_maps" value="2 1"')
                data = text.encode()
            dst.writestr(info, data)
PY

run_case "seed-only" "$WORKDIR/cube.stl" \
    "Nozzle-map provenance: explicit config map=no" \
    "Nozzle-map derivation: skipped mode=Auto For Flush" \
    ""

run_case "mapless-3mf-default" "$WORKDIR/mapless.3mf" \
    "Nozzle-map provenance: explicit config map=no" \
    "Nozzle-map derivation: skipped mode=Auto For Flush" \
    ""

run_validation_failure "stl-config-cross-map" "$WORKDIR/cube.stl" \
    "Nozzle-map provenance: explicit config map=yes" \
    "Nozzle-map derivation: filament_map=[2,1] mode=Nozzle Manual" \
    "Nozzle-map reassignment: object_extruders=[1]" \
    "Validation error: cube.stl is too close to exclusion area" \
    --config "$WORKDIR/stl-cross-map.json"

# Each CLI overlay must establish provenance after a mapless native 3MF load.
# [2,1] is the helper's derived logical map for physical mapping [1,2]; the
# master physical nozzle is 2 and reassigns the cube to filament slot 1.
for flag in --config --machine --process --filament; do
    label="mapless-3mf-${flag#--}"
    run_case "$label" "$WORKDIR/mapless.3mf" \
        "Nozzle-map provenance: explicit config map=yes" \
        "Nozzle-map derivation: filament_map=[2,1] mode=Nozzle Manual" \
        "Nozzle-map reassignment: object_extruders=[1]" \
        "$flag" "$WORKDIR/cross-map.json"
done

run_case "native-3mf-cross-map" "$WORKDIR/native-cross-map.3mf" \
    "Nozzle-map provenance: explicit config map=yes" \
    "Nozzle-map derivation: filament_map=[2,1] mode=Nozzle Manual" \
    "Nozzle-map reassignment: object_extruders=[1]"

run_case "plate-cross-map-single-nozzle" "$WORKDIR/plate-cross-map.3mf" \
    "Nozzle-map provenance: explicit config map=no" \
    "Nozzle-map derivation: skipped mode=Manual" \
    ""

echo "Results: $PASS passed, $FAIL failed"
[ "$FAIL" -eq 0 ]
