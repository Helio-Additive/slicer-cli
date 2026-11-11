#!/usr/bin/env bash
# --filament-map on a two-nozzle Bambu printer (the H2C): which nozzle each
# filament prints with, and what result.json reports.
#
# A bare STL and a bare 3MF (the STL saved as a project, never sliced) on
# "Bambu Lab H2C 0.4 nozzle" (extruder_max_nozzle_count 1,6, master extruder 2):
#   - no flag: Auto For Flush, the engine picks the nozzle; result.json states
#     the nozzle the plate's slice_info.config filament_maps states;
#   - --filament-map 1 and --filament-map 2: slice_info filament_maps is that
#     nozzle, the mode is Manual, result.json reports the same;
#   - the 3MF the automatic slice saved (project_settings filament_map and the
#     plate's map as an earlier slice left them), re-sliced with each flag;
#   - refusals before slicing (-2): a wrong count, a nozzle the printer does
#     not have, and a one-nozzle printer.
# Usage: tests/test_filament_map_flag.sh /path/to/bambu/slicer_cli

set -euo pipefail

BINARY="${1:?usage: $0 /path/to/bambu/slicer_cli}"
WORKDIR="$(mktemp -d "${TMPDIR:-/tmp}/filament_map_flag.XXXXXX")"
trap 'rm -rf "$WORKDIR"' EXIT
H2C="Bambu Lab H2C 0.4 nozzle"
ONE_NOZZLE="Bambu Lab A1 mini 0.4 nozzle"
PASS=0
FAIL=0

pass() { PASS=$((PASS + 1)); echo "PASS [$1]"; }
fail() { FAIL=$((FAIL + 1)); echo "FAIL [$1] $2"; }

# A closed 20 mm cube at the bed's middle.
python3 - "$WORKDIR/cube.stl" <<'PY'
import sys
x0, y0, s = 150.0, 150.0, 20.0
v = [(x0 + dx * s, y0 + dy * s, dz * s) for dz in (0, 1) for dy in (0, 1) for dx in (0, 1)]
faces = [(0, 2, 1), (1, 2, 3), (4, 5, 6), (5, 7, 6), (0, 1, 4), (1, 5, 4),
         (2, 6, 3), (3, 6, 7), (0, 4, 2), (2, 4, 6), (1, 3, 5), (3, 7, 5)]
with open(sys.argv[1], "w") as f:
    f.write("solid cube\n")
    for a, b, c in faces:
        f.write("facet normal 0 0 0\nouter loop\n")
        for i in (a, b, c):
            f.write("vertex %g %g %g\n" % v[i])
        f.write("endloop\nendfacet\n")
    f.write("endsolid cube\n")
PY

# Slices `input` into $WORKDIR/<label>/ with a sliced 3MF; prints the exit code.
slice() {
    local label="$1" input="$2"; shift 2
    mkdir -p "$WORKDIR/$label"
    set +e
    "$BINARY" "$input" --printer-preset "$H2C" --slice 1 --outputdir "$WORKDIR/$label" \
        --export-3mf sliced.3mf "$@" >"$WORKDIR/$label/stdout" 2>"$WORKDIR/$label/stderr"
    local status=$?
    set -e
    echo "$status"
}

# Checks one sliced run: the plate's slice_info filament_maps (and, when given,
# the expected nozzle and mode) agree with result.json's filament_maps,
# filament_map_mode and each filament's extruder. Prints the slice_info map.
check_slice() {
    local label="$1" want_map="$2" want_mode="$3"
    python3 - "$WORKDIR/$label" "$want_map" "$want_mode" <<'PY'
import json, re, sys, zipfile
d, want_map, want_mode = sys.argv[1:]
z = zipfile.ZipFile(d + "/sliced.3mf")
info = z.read("Metadata/slice_info.config").decode()
m = re.search(r'key="filament_maps" value="([^"]*)"', info)
assert m, "slice_info.config has no filament_maps"
maps = [int(x) for x in m.group(1).split()]
r = json.load(open(d + "/result.json"))
assert r["return_code"] == 0, r
plate = r["sliced_plates"][0]
assert plate.get("filament_maps") == maps, ("result.json filament_maps", plate.get("filament_maps"), "slice_info", maps)
for f in plate["filaments"]:
    assert f.get("extruder") == maps[f["id"] - 1], ("filament extruder", f, maps)
if want_map:
    assert maps == [int(x) for x in want_map.split(",")], ("slice_info filament_maps", maps, "wanted", want_map)
if want_mode:
    assert plate.get("filament_map_mode") == want_mode, ("filament_map_mode", plate.get("filament_map_mode"))
    plate_cfg = z.read("Metadata/model_settings.config").decode()
    if want_mode == "Manual":
        assert 'key="filament_map_mode" value="Manual"' in plate_cfg, "exported plate is not Manual"
gcode = z.read("Metadata/plate_1.gcode").decode(errors="replace")
g = re.search(r"^; filament_map = (.*)$", gcode, re.M)
if g and want_map:
    assert g.group(1).strip().replace(" ", ",") == want_map, ("G-code filament_map", g.group(1))
print(",".join(map(str, maps)))
PY
}

# Runs one slice that must succeed and checks it.
expect_slice() {
    local label="$1" input="$2" want_map="$3" want_mode="$4"; shift 4
    local status got
    status=$(slice "$label" "$input" "$@")
    if [ "$status" != 0 ]; then
        fail "$label" "exit $status"; sed 's/^/  /' "$WORKDIR/$label/stderr" | tail -n 5; return
    fi
    if got=$(check_slice "$label" "$want_map" "$want_mode" 2>&1); then
        pass "$label: filament_maps $got"
    else
        fail "$label" "$got"
    fi
}

# Runs one slice that must be refused before slicing, with `sentence` in stderr.
expect_refusal() {
    local label="$1" input="$2" printer="$3" sentence="$4"; shift 4
    mkdir -p "$WORKDIR/$label"
    set +e
    "$BINARY" "$input" --printer-preset "$printer" --slice 1 --outputdir "$WORKDIR/$label" "$@" \
        >"$WORKDIR/$label/stdout" 2>"$WORKDIR/$label/stderr"
    local status=$?
    set -e
    # -2 as the shell sees it.
    if [ "$status" != 254 ]; then
        fail "$label" "exit $status, expected -2"; tail -n 5 "$WORKDIR/$label/stderr"; return
    fi
    if ! grep -Fq -- "$sentence" "$WORKDIR/$label/stderr"; then
        fail "$label" "stderr lacks: $sentence"; tail -n 5 "$WORKDIR/$label/stderr"; return
    fi
    if [ -e "$WORKDIR/$label/plate_1.gcode" ]; then
        fail "$label" "a refused run wrote G-code"; return
    fi
    if ! python3 -c 'import json,sys; r=json.load(open(sys.argv[1])); assert r["return_code"]==-2, r' \
        "$WORKDIR/$label/result.json" 2>/dev/null; then
        fail "$label" "result.json does not state -2"; return
    fi
    pass "$label"
}

# Bare STL.
expect_slice "stl-auto" "$WORKDIR/cube.stl" "" "Auto For Flush"
expect_slice "stl-map-1" "$WORKDIR/cube.stl" "1" "Manual" --filament-map 1
expect_slice "stl-map-2" "$WORKDIR/cube.stl" "2" "Manual" --filament-map 2
grep -Fq '"tag":"FilamentMapFromCommandLine"' "$WORKDIR/stl-map-1/stdout" \
    && pass "stl-map-1: FilamentMapFromCommandLine event" \
    || fail "stl-map-1" "no FilamentMapFromCommandLine event"

# A bare 3MF: the project the automatic slice saved, with what slicing adds
# taken out (the plate G-code, slice_info.config, the plate's map and mode):
# slicer_cli writes a 3MF only with --slice.
BARE="$WORKDIR/bare.3mf"
python3 - "$WORKDIR/stl-auto/sliced.3mf" "$BARE" <<'PY'
import re, sys, zipfile
src, dst = sys.argv[1:]
with zipfile.ZipFile(src) as zin, zipfile.ZipFile(dst, "w", zipfile.ZIP_DEFLATED) as zout:
    for info in zin.infolist():
        name = info.filename
        if re.match(r"Metadata/(plate_\d+\.gcode.*|slice_info\.config|plate_\d+\.json|plate_\d+.*\.png|top_\d+\.png|pick_\d+\.png)$", name):
            continue
        data = zin.read(name)
        if name == "Metadata/model_settings.config":
            text = re.sub(r'\s*<metadata key="(filament_maps|filament_map_mode|filament_volume_maps)" value="[^"]*"/>', "", data.decode())
            data = text.encode()
        zout.writestr(info, data)
    names = zout.namelist()
assert "Metadata/slice_info.config" not in names and not any(n.endswith(".gcode") for n in names), names
PY

# Bare 3MF.
expect_slice "3mf-auto" "$BARE" "" "Auto For Flush"
expect_slice "3mf-map-1" "$BARE" "1" "Manual" --filament-map 1
expect_slice "3mf-map-2" "$BARE" "2" "Manual" --filament-map 2

# The project the automatic slice saved, re-sliced with each nozzle: the
# flag wins over the map that slice left on the plate and in the project.
AUTO_3MF="$WORKDIR/stl-auto/sliced.3mf"
if [ -s "$AUTO_3MF" ]; then
    expect_slice "resliced-map-1" "$AUTO_3MF" "1" "Manual" --filament-map 1
    expect_slice "resliced-map-2" "$AUTO_3MF" "2" "Manual" --filament-map 2
fi

# Refusals.
expect_refusal "refuse-count" "$WORKDIR/cube.stl" "$H2C" \
    "--filament-map gives 2 values but the project has 1 filament" --filament-map 1,2
expect_refusal "refuse-nozzle" "$WORKDIR/cube.stl" "$H2C" \
    "--filament-map gives nozzle 3 for filament 1, but this printer" --filament-map 3
expect_refusal "refuse-one-nozzle" "$WORKDIR/cube.stl" "$ONE_NOZZLE" \
    "has one nozzle. Drop the flag to slice." --filament-map 1

echo "filament-map: $PASS passed, $FAIL failed"
[ "$FAIL" -eq 0 ]
