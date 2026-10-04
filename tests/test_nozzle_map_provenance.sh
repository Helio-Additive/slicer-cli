#!/usr/bin/env bash
# Bambu provenance regressions. Assertions use the production CLI's config
# loading and explicit-map derivation. A derived routing is the plate's; no
# object is moved to another filament (Bambu Studio and OrcaSlicer never do).
# Usage: tests/test_nozzle_map_provenance.sh /path/to/bambu/slicer_cli

set -euo pipefail

BINARY="${1:?usage: $0 /path/to/bambu/slicer_cli}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_3MF="$SCRIPT_DIR/fixtures/calib_base.3mf"
WORKDIR="$(mktemp -d "${TMPDIR:-/tmp}/nozzle_map_provenance.XXXXXX")"
trap 'rm -rf "$WORKDIR"' EXIT
PASS=0
FAIL=0

# Records one labeled assertion and marks the suite failed when it does not match.
record() {
    local label="$1" expected="$2" output="$3"
    if grep -Fq "$expected" <<<"$output"; then
        PASS=$((PASS + 1)); echo "PASS [$label]"
    else
        FAIL=$((FAIL + 1)); echo "FAIL [$label] expected: $expected"
        echo "  output: $output"
    fi
}

# Runs a slice case and checks provenance, derivation, and reassignment output.
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

# Runs a derived-routing case: provenance and derivation as stated, no object
# moved to another filament, `absent` (when given) not in the output, and the
# routed plate sliced to G-code.
run_routing_case() {
    local label="$1" input="$2" provenance="$3" derivation="$4" absent="$5"; shift 5
    set +e
    local output
    output=$("$BINARY" --verbose "$input" "$@" -o "$WORKDIR/$label.gcode" 2>&1)
    local status=$?
    set -e
    if [ "$status" -ne 0 ] || [ ! -s "$WORKDIR/$label.gcode" ]; then
        FAIL=$((FAIL + 1)); echo "FAIL [$label] exit=$status (expected a sliced plate)"
        echo "  output: $output"
        return
    fi
    record "$label/provenance" "$provenance" "$output"
    record "$label/derivation" "$derivation" "$output"
    if grep -Fq "Nozzle-map reassignment:" <<<"$output" || { [ -n "$absent" ] && grep -Fq "$absent" <<<"$output"; }; then
        FAIL=$((FAIL + 1)); echo "FAIL [$label/objects-keep-filament]"
        echo "  output: $output"
    else
        PASS=$((PASS + 1)); echo "PASS [$label/objects-keep-filament]"
    fi
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
  "filament_nozzle_map": ["1", "0"]
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
                    "physical_extruder_map": ["0", "1"],
                    # Stop at validation after observing actual reassignment:
                    # slot 1 is printable, slot 2 deliberately is not.
                    "textured_plate_temp": ["55", "0"],
                    "textured_plate_temp_initial_layer": ["55", "0"],
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
                    config["filament_nozzle_map"] = ["1", "0"]
                else:
                    stl_profile = dict(config)
                    stl_profile["filament_nozzle_map"] = ["1", "0"]
                    with open(stl_profile_destination, "w") as profile:
                        json.dump(stl_profile, profile, indent=2)
                data = json.dumps(config, indent=2).encode()
            dst.writestr(info, data)

# A 1-nozzle-per-extruder variant with a plate-level diverse filament_maps
# value, once in the plate's "Auto For Flush" and once with the plate set to
# Manual (only then does the stored map drive the mapping).
for destination, manual in ((plate_destination, False), (plate_destination.replace(".3mf", "-manual.3mf"), True)):
    with zipfile.ZipFile(destination, "w") as dst:
        with zipfile.ZipFile(mapless_destination) as src:
            for info in src.infolist():
                data = src.read(info.filename)
                if info.filename == "Metadata/model_settings.config":
                    text = data.decode()
                    text = text.replace('key="filament_maps" value="1"', 'key="filament_maps" value="2 1"')
                    if manual:
                        text = text.replace('key="filament_map_mode" value="Auto For Flush"', 'key="filament_map_mode" value="Manual"')
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

# One nozzle per extruder (extruder_max_nozzle_count 1,1): the derived map
# takes Manual, as the official CLI refuses Nozzle Manual on such machines.
# The same cube at the bed centre: the profile is the X1 Carbon's, whose
# bed_exclude_area corner (0-18 x 0-28 mm) the origin cube sits in.
python3 - "$WORKDIR/cube.stl" "$WORKDIR/cube-centre.stl" <<'PY'
import re, sys
src, dst = sys.argv[1:]
text = open(src).read()
text = re.sub(r"vertex (\S+) (\S+) (\S+)",
              lambda m: "vertex %g %g %s" % (float(m.group(1)) + 118, float(m.group(2)) + 118, m.group(3)), text)
open(dst, "w").write(text)
PY
run_routing_case "stl-config-cross-map" "$WORKDIR/cube-centre.stl" \
    "Nozzle-map provenance: explicit config map=yes" \
    "Nozzle-map derivation: filament_map=[2,1] mode=Manual" \
    "does not support filament 2" \
    --config "$WORKDIR/stl-cross-map.json"

# Each CLI overlay must establish provenance after a mapless native 3MF load.
# With physical mapping [0,1] and nozzle map [1,0], the derived logical map
# is [2,1]. The object stays on filament slot 1 and the plate slices: slot 2
# is deliberately unprintable on the plate, so an object moved onto it would
# be refused.
for flag in --config --machine --process --filament; do
    label="mapless-3mf-${flag#--}"
    run_routing_case "$label" "$WORKDIR/mapless.3mf" \
        "Nozzle-map provenance: explicit config map=yes" \
        "Nozzle-map derivation: filament_map=[2,1] mode=Manual" \
        "does not support filament 2" \
        "$flag" "$WORKDIR/cross-map.json"
done

# A file's own filament_nozzle_map is what an earlier slice grouped, not a
# request: under "Auto For Flush" the engine groups itself, as the official
# CLI does (it reads a nozzle map only from its command line, in a manual
# mode). No derivation, no reassignment, the object stays on filament 1.
run_case "native-3mf-cross-map" "$WORKDIR/native-cross-map.3mf" \
    "Nozzle-map provenance: explicit config map=no" \
    "Nozzle-map derivation: skipped mode=Auto For Flush" \
    ""

# A plate saved with a mixed map but in "Auto For Flush" stays automatic:
# the stored map is only used when the plate's mode is manual.
run_case "plate-cross-map-auto" "$WORKDIR/plate-cross-map.3mf" \
    "Nozzle-map provenance: explicit config map=no" \
    "Nozzle-map derivation: skipped mode=Auto For Flush" \
    ""

# The same plate map with the plate set to Manual is applied as saved.
run_case "plate-cross-map-manual" "$WORKDIR/plate-cross-map-manual.3mf" \
    "Nozzle-map provenance: explicit config map=no" \
    "Nozzle-map derivation: skipped mode=Manual" \
    ""

# --export-3mf keeps the derived routing on the plate: a mapless project
# sliced with a cross-nozzle --config map exports the derived map and Manual
# on its plate, the project's own mode untouched, and the object on slot 1.
python3 - "$WORKDIR/mapless.3mf" "$WORKDIR/mapless-ok.3mf" <<'PY'
import json, sys, zipfile
source, destination = sys.argv[1:]
with zipfile.ZipFile(source) as src, zipfile.ZipFile(destination, "w") as dst:
    for info in src.infolist():
        data = src.read(info.filename)
        if info.filename == "Metadata/project_settings.config":
            config = json.loads(data)
            config["textured_plate_temp"] = ["55", "55"]
            config["textured_plate_temp_initial_layer"] = ["55", "55"]
            config["extruder_printable_height"] = ["250", "250"]
            data = json.dumps(config, indent=2).encode()
        dst.writestr(info, data)
PY
set +e
export_output=$("$BINARY" --verbose "$WORKDIR/mapless-ok.3mf" --config "$WORKDIR/cross-map.json" \
    --slice 1 --outputdir "$WORKDIR/export-routing" --export-3mf routed.3mf 2>&1)
export_status=$?
set -e
if [ "$export_status" -ne 0 ]; then
    FAIL=$((FAIL + 1)); echo "FAIL [export-derived-routing] exit=$export_status"
    echo "  output: $export_output"
else
    record "export-derived-routing/derivation" "Nozzle-map derivation: filament_map=[2,1] mode=Manual" "$export_output"
    if python3 - "$WORKDIR/export-routing/routed.3mf" <<'PY'
import json, re, sys, zipfile
z = zipfile.ZipFile(sys.argv[1])
project = json.loads(z.read("Metadata/project_settings.config"))
assert project["filament_map_mode"] == "Auto For Flush", project["filament_map_mode"]
model = z.read("Metadata/model_settings.config").decode()
plate = model[model.index("<plate>"):model.index("</plate>")]
assert 'key="filament_map_mode" value="Manual"' in plate, plate[:400]
assert 'key="filament_maps" value="2 1"' in plate, plate[:400]
obj = model[model.index("<object"):model.index("</object>")]
assert not re.search(r'key="extruder" value="2"', obj), obj[:400]
PY
    then PASS=$((PASS + 1)); echo "PASS [export-derived-routing/project]"
    else FAIL=$((FAIL + 1)); echo "FAIL [export-derived-routing/project]"
    fi
fi

echo "Results: $PASS passed, $FAIL failed"
[ "$FAIL" -eq 0 ]
