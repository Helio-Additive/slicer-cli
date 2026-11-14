#!/usr/bin/env bash
# --filament-map on a printer with a left and a right extruder: which one each
# filament prints with, and what result.json reports, for slicer_cli and
# slicer_cli-orcaslicer; for slicer_cli-snapmaker the nozzles' flow types on
# the U1. The engine is read from the binary's --list-flags.
#
# One filament, a bare STL and a bare 3MF (the STL saved as a project, never
# sliced), on "Bambu Lab H2C 0.4 nozzle" (extruder_max_nozzle_count 1,6, master
# extruder 2) for slicer_cli and "Bambu Lab H2D 0.4 nozzle" for
# slicer_cli-orcaslicer, whose engine has no H2C:
#   - no flag: Auto For Flush, the engine picks the nozzle; result.json states
#     the nozzle the plate's slice_info.config filament_maps states;
#   - --filament-map 1 and --filament-map 2: slice_info filament_maps is that
#     nozzle, the mode is Manual, result.json reports the same;
#   - the 3MF the automatic slice saved (project_settings filament_map and the
#     plate's map as an earlier slice left them), re-sliced with each flag;
#   - refusals before slicing (-2): a wrong count, a nozzle the printer does
#     not have, a one-nozzle printer, and (slicer_cli) the H3C, one extruder
#     with a nozzle rack;
#   - --export-settings with the flag states Manual.
# Two filaments, one object each, split across the extruders (1,2 and 2,1) on
# every such printer the engine has, and two colours with no flag: the bare
# model's prime tower lands where both nozzles reach.
# Usage: tests/test_filament_map_flag.sh /path/to/slicer_cli[-orcaslicer|-snapmaker]

set -euo pipefail

BINARY="${1:?usage: $0 /path/to/slicer_cli[-orcaslicer]}"
ENGINE="$("$BINARY" --list-flags | python3 -c 'import json,sys; print(json.load(sys.stdin)["engine"])')"
WORKDIR="$(mktemp -d "${TMPDIR:-/tmp}/filament_map_flag.XXXXXX")"
trap 'rm -rf "$WORKDIR"' EXIT
H2C="Bambu Lab H2C 0.4 nozzle"
ONE_NOZZLE="Bambu Lab A1 mini 0.4 nozzle"
H2D="Bambu Lab H2D 0.4 nozzle"
H3C="Bambu Lab H3C 0.4 nozzle"
# The printer of the one-filament cases, and every left/right printer the engine has.
# A bare model on Orca's H2D is a project of two filaments (the preset's two
# slots), on Bambu's H2C of one: L and R send every filament left or right.
if [ "$ENGINE" = orcaslicer ]; then
    ONE="$H2D"; SPLIT=("$H2D" "Bambu Lab H2D Pro 0.4 nozzle" "Bambu Lab X2D 0.4 nozzle")
    L="1,1"; R="2,2"; SHORT="1"; SHORT_SENTENCE="--filament-map gives 1 value but the project has 2 filaments"; BAD="3,1"
else
    ONE="$H2C"; SPLIT=("$H2D" "Bambu Lab H2D Pro 0.4 nozzle" "Bambu Lab X2D 0.4 nozzle" "$H2C")
    L="1"; R="2"; SHORT="1,2"; SHORT_SENTENCE="--filament-map gives 2 values but the project has 1 filament"; BAD="3"
fi
PRINTER="$ONE"   # the printer slice() uses
PASS=0
FAIL=0

pass() { PASS=$((PASS + 1)); echo "PASS [$1]"; }
fail() { FAIL=$((FAIL + 1)); echo "FAIL [$1] $2"; }

# Two 20 mm cubes as a bare 3MF, the first on filament 1, the second on filament 2.
make_two_3mf() {
python3 - "$WORKDIR/two.3mf" <<'PY'
import sys, zipfile
def cube(x0, y0, s=20.0):
    v = [(x0 + dx * s, y0 + dy * s, dz * s) for dz in (0, 1) for dy in (0, 1) for dx in (0, 1)]
    f = [(0, 2, 1), (1, 2, 3), (4, 5, 6), (5, 7, 6), (0, 1, 4), (1, 5, 4),
         (2, 6, 3), (3, 6, 7), (0, 4, 2), (2, 4, 6), (1, 3, 5), (3, 7, 5)]
    return ("<mesh><vertices>" + "".join('<vertex x="%g" y="%g" z="%g"/>' % p for p in v) + "</vertices><triangles>"
            + "".join('<triangle v1="%d" v2="%d" v3="%d"/>' % t for t in f) + "</triangles></mesh>")
model = ('<?xml version="1.0" encoding="UTF-8"?>\n<model unit="millimeter" xml:lang="en-US" '
         'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02"><resources>'
         '<object id="1" type="model">%s</object><object id="2" type="model">%s</object></resources>'
         '<build><item objectid="1"/><item objectid="2"/></build></model>') % (cube(100, 150), cube(180, 150))
settings = ('<?xml version="1.0" encoding="UTF-8"?>\n<config>'
            '<object id="1"><metadata key="name" value="a"/><metadata key="extruder" value="1"/></object>'
            '<object id="2"><metadata key="name" value="b"/><metadata key="extruder" value="2"/></object></config>')
types = ('<?xml version="1.0" encoding="UTF-8"?>\n<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
         '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
         '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>')
rels = ('<?xml version="1.0" encoding="UTF-8"?>\n<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
        '<Relationship Target="/3D/3dmodel.model" Id="rel-1" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
with zipfile.ZipFile(sys.argv[1], "w", zipfile.ZIP_DEFLATED) as z:
    z.writestr("[Content_Types].xml", types)
    z.writestr("_rels/.rels", rels)
    z.writestr("3D/3dmodel.model", model)
    z.writestr("Metadata/model_settings.config", settings)
PY
}

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
    "$BINARY" "$input" --printer-preset "$PRINTER" --slice 1 --outputdir "$WORKDIR/$label" \
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

# Runs one slice that must be refused before slicing, with `sentence` in stderr
# and return code -2 (CODE=<n> for another).
expect_refusal() {
    local label="$1" input="$2" printer="$3" sentence="$4"; shift 4
    mkdir -p "$WORKDIR/$label"
    set +e
    "$BINARY" "$input" --printer-preset "$printer" --slice 1 --outputdir "$WORKDIR/$label" "$@" \
        >"$WORKDIR/$label/stdout" 2>"$WORKDIR/$label/stderr"
    local status=$?
    set -e
    # The refusal's code is read from result.json below: the shell's view of
    # -2 differs by system (254, or 127 under Git Bash on Windows).
    if [ "$status" = 0 ]; then
        fail "$label" "exit 0, expected a refusal"; tail -n 5 "$WORKDIR/$label/stderr"; return
    fi
    if ! grep -Fq -- "$sentence" "$WORKDIR/$label/stderr"; then
        fail "$label" "stderr lacks: $sentence"; tail -n 5 "$WORKDIR/$label/stderr"; return
    fi
    if [ -e "$WORKDIR/$label/plate_1.gcode" ]; then
        fail "$label" "a refused run wrote G-code"; return
    fi
    if ! python3 -c 'import json,sys; r=json.load(open(sys.argv[1])); assert r["return_code"]==int(sys.argv[2]), r' \
        "$WORKDIR/$label/result.json" "${CODE:--2}" 2>/dev/null; then
        fail "$label" "result.json does not state ${CODE:--2}"; return
    fi
    pass "$label"
}

# slicer_cli-snapmaker: the nozzles' flow types and the grouping dialog's
# answer on the U1. The engine has no filament map.
if [ "$ENGINE" = snapmaker ]; then
    U1="Snapmaker U1 (0.4 nozzle)"
    make_two_3mf
    snap() { # label, the G-code line wanted, the event tag wanted ("" for none), flags...
        local label="$1" want="$2" tag="$3"; shift 3
        mkdir -p "$WORKDIR/$label"
        if ! "$BINARY" "$WORKDIR/two.3mf" --printer-preset "$U1" --slice 1 --outputdir "$WORKDIR/$label" "$@" \
                >"$WORKDIR/$label/stdout" 2>"$WORKDIR/$label/stderr"; then
            fail "$label" "the slice failed"; tail -n 3 "$WORKDIR/$label/stderr"; return
        fi
        grep -Fxq -- "; $want" "$WORKDIR/$label/plate_1.gcode" || { fail "$label" "the G-code does not state: $want"; return; }
        if [ -n "$tag" ] && ! grep -Fq "\"tag\":\"$tag\"" "$WORKDIR/$label/stdout"; then
            fail "$label" "no $tag event"; return
        fi
        pass "$label"
    }
    PLA=(--filament-preset "Generic PLA" --filament-preset "Generic PLA")
    WOOD=(--filament-preset "Snapmaker PLA Wood @U1 0.4 nozzle" --filament-preset "Snapmaker PLA Wood @U1 0.4 nozzle")
    snap "all-standard" "filament_volume_type = standard,standard" "" "${PLA[@]}"
    snap "all-high-flow" "filament_volume_type = high_flow,high_flow" "" "${PLA[@]}" \
        --nozzle-volume-type "high_flow,high_flow,high_flow,high_flow"
    snap "mixed-keeps-the-file" "filament_volume_type = standard,standard" "MixedNozzleFlowTypes" "${PLA[@]}" \
        --nozzle-volume-type "high_flow,standard,standard,standard"
    snap "mixed-with-the-answer" "filament_volume_type = high_flow,standard" "" "${PLA[@]}" \
        --nozzle-volume-type "high_flow,standard,standard,standard" --filament-volume-type "high_flow,standard"
    snap "not-recommended-warns" "filament_volume_type = high_flow,high_flow" "HighFlowNotRecommended" "${WOOD[@]}" \
        --nozzle-volume-type "high_flow,high_flow,high_flow,high_flow"
    expect_refusal "refuse-filament-map" "$WORKDIR/two.3mf" "$U1" "--filament-map works in slicer_cli and slicer_cli-orcaslicer" \
        "${PLA[@]}" --filament-map 1,2
    echo "filament-map: $PASS passed, $FAIL failed"
    [ "$FAIL" -eq 0 ]
    exit
fi

# Bare STL.
expect_slice "stl-auto" "$WORKDIR/cube.stl" "" "Auto For Flush"
expect_slice "stl-map-1" "$WORKDIR/cube.stl" "$L" "Manual" --filament-map "$L"
expect_slice "stl-map-2" "$WORKDIR/cube.stl" "$R" "Manual" --filament-map "$R"
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
expect_slice "3mf-map-1" "$BARE" "$L" "Manual" --filament-map "$L"
expect_slice "3mf-map-2" "$BARE" "$R" "Manual" --filament-map "$R"

# The project the automatic slice saved, re-sliced with each nozzle: the
# flag wins over the map that slice left on the plate and in the project.
AUTO_3MF="$WORKDIR/stl-auto/sliced.3mf"
if [ -s "$AUTO_3MF" ]; then
    expect_slice "resliced-map-1" "$AUTO_3MF" "$L" "Manual" --filament-map "$L"
    expect_slice "resliced-map-2" "$AUTO_3MF" "$R" "Manual" --filament-map "$R"
fi

# Refusals.
expect_refusal "refuse-count" "$WORKDIR/cube.stl" "$ONE" \
    "$SHORT_SENTENCE" --filament-map "$SHORT"
expect_refusal "refuse-nozzle" "$WORKDIR/cube.stl" "$ONE" \
    "--filament-map gives nozzle 3 for filament 1, but this printer" --filament-map "$BAD"
expect_refusal "refuse-one-nozzle" "$WORKDIR/cube.stl" "$ONE_NOZZLE" \
    "has one nozzle. Drop the flag to slice." --filament-map 1

# --export-settings with the flag: the saved settings state Manual with the
# map, so a slice from them keeps the grouping.
mkdir -p "$WORKDIR/settings"
if "$BINARY" "$WORKDIR/cube.stl" --printer-preset "$ONE" --slice 1 --outputdir "$WORKDIR/settings" \
        --filament-map "$L" --export-settings "$WORKDIR/settings/settings.json" \
        >"$WORKDIR/settings/stdout" 2>"$WORKDIR/settings/stderr" &&
   got=$(python3 -c 'import json,sys; s=json.load(open(sys.argv[1])); assert s["filament_map_mode"]=="Manual", s["filament_map_mode"]; assert [int(v) for v in s["filament_map"]]==[int(v) for v in sys.argv[2].split(",")], s["filament_map"]' \
        "$WORKDIR/settings/settings.json" "$L" 2>&1); then
    pass "export-settings: Manual with the map"
else
    fail "export-settings" "${got:-the run failed}"; tail -n 3 "$WORKDIR/settings/stderr"
fi

# Two filaments, one object each, as a bare 3MF: each split across the two
# extruders on each printer of SPLIT. The model has no saved prime tower, so
# these also check that the tower lands where both nozzles reach; the last
# case is that check alone (two colours keep the tower on, no flag).
make_two_3mf
for PRINTER in "${SPLIT[@]}"; do
    case "$PRINTER" in
        *"H2D Pro"*) tag=h2dpro; pla="Bambu PLA Basic @BBL H2DP" ;;
        *H2D*)       tag=h2d;    pla="Bambu PLA Basic @BBL H2D" ;;
        *X2D*)       tag=x2d;    pla="Bambu PLA Basic @BBL X2D 0.4 nozzle" ;;
        *)           tag=h2c;    pla="Bambu PLA Basic @BBL H2C" ;;
    esac
    expect_slice "$tag-two-1-2" "$WORKDIR/two.3mf" "1,2" "Manual" \
        --filament-preset "$pla" --filament-preset "$pla" --filament-map 1,2
    expect_slice "$tag-two-2-1" "$WORKDIR/two.3mf" "2,1" "Manual" \
        --filament-preset "$pla" --filament-preset "$pla" --filament-map 2,1
    expect_slice "$tag-two-colours-auto" "$WORKDIR/two.3mf" "" "Auto For Flush" \
        --filament-preset "$pla" --filament-preset "$pla" --filament-colour "#FF0000;#00FF00"
done
PRINTER="$ONE"

# The nozzle and grouping settings of the official command lines, and Bambu
# Studio's rack grouping. Each case slices two.3mf and reads the settings the
# plate's G-code states (and result.json's filament_nozzle_map when given).
expect_header() {
    local label="$1" want="$2" want_nozzles="$3"; shift 3
    local status got
    status=$(slice "$label" "$WORKDIR/two.3mf" "$@")
    if [ "$status" != 0 ]; then
        fail "$label" "exit $status"; sed 's/^/  /' "$WORKDIR/$label/stderr" | tail -n 5; return
    fi
    if got=$(python3 - "$WORKDIR/$label" "$want" "$want_nozzles" 2>&1 <<'PY'
import json, sys, zipfile
d, want, want_nozzles = sys.argv[1:]
z = zipfile.ZipFile(d + "/sliced.3mf")
gcode = z.read([n for n in z.namelist() if n.endswith(".gcode")][0]).decode(errors="ignore")
for line in filter(None, want.split(";;")):
    assert ("\n; " + line + "\n") in gcode, "the G-code does not state: " + line
plate = json.load(open(d + "/result.json"))["sliced_plates"][0]
if want_nozzles:
    assert plate.get("filament_nozzle_map") == [int(v) for v in want_nozzles.split(",")], \
        ("result.json filament_nozzle_map", plate.get("filament_nozzle_map"))
PY
    ); then
        pass "$label"
    else
        fail "$label" "$got"
    fi
}

PRINTER="$H2D"; D2=(--filament-preset "Bambu PLA Basic @BBL H2D" --filament-preset "Bambu PLA Basic @BBL H2D")
# The mode from the command line wins over the plate's: a plate saved in
# Manual slices in the automatic mode again.
status=$(slice "mode-auto-over-manual-plate" "$WORKDIR/h2d-two-1-2/sliced.3mf" --filament-map-mode "Auto For Flush")
if [ "$status" = 0 ] && python3 -c 'import json,sys; p=json.load(open(sys.argv[1]))["sliced_plates"][0]; assert p["filament_map_mode"]=="Auto For Flush", p["filament_map_mode"]' \
        "$WORKDIR/mode-auto-over-manual-plate/result.json" 2>/dev/null; then
    pass "mode-auto-over-manual-plate"
else
    fail "mode-auto-over-manual-plate" "exit $status, or the plate's Manual stayed"
fi
# An object where only the left nozzle reaches (the H2D's right nozzle starts
# at x = 25): the map that names the right extruder is refused before slicing
# (-66, as the official command lines), the left one slices.
python3 - "$WORKDIR/edge.stl" <<'PY'
import sys
x0, y0, s = 2.0, 150.0, 20.0
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
EDGE_L="${L%%,*}"; EDGE_R="${R%%,*}"; [ "$ENGINE" = orcaslicer ] && { EDGE_L="1,1"; EDGE_R="2,2"; }
CODE=-66 expect_refusal "refuse-extruder-cannot-reach" "$WORKDIR/edge.stl" "$H2D" "cannot reach" --arrange 0 --filament-map "$EDGE_R"
status=$(slice "extruder-reaches" "$WORKDIR/edge.stl" --arrange 0 --filament-map "$EDGE_L")
[ "$status" = 0 ] && pass "extruder-reaches" || { fail "extruder-reaches" "exit $status"; tail -n 3 "$WORKDIR/extruder-reaches/stderr"; }

expect_header "flow-type-per-extruder" "nozzle_volume_type = High Flow,Standard" "" "${D2[@]}" \
    --nozzle-volume-type "High Flow,Standard"

if [ "$ENGINE" = orcaslicer ]; then
    expect_refusal "refuse-own-extruder-per-filament" "$WORKDIR/two.3mf" "Prusa XL 5T 0.4 nozzle" \
        "every filament has its own extruder" --filament-map 2,1,3,4,5
else
    expect_header "manual-flow-type-follows-extruder" "filament_map = 1,2;;filament_volume_map = 1,0;;filament_map_mode = Manual" "" \
        "${D2[@]}" --filament-map 1,2 --nozzle-volume-type "High Flow,Standard"
    # A plate that remembers a filament switcher and smart filament
    # assignment: both are off unless a flag states them, as in the official
    # command line.
    SWITCHER="$WORKDIR/switcher.3mf"
    python3 - "$WORKDIR/h2d-two-1-2/sliced.3mf" "$SWITCHER" <<'PY'
import re, sys, zipfile
src, dst = sys.argv[1:]
with zipfile.ZipFile(src) as zin, zipfile.ZipFile(dst, "w", zipfile.ZIP_DEFLATED) as zout:
    for info in zin.infolist():
        data = zin.read(info.filename)
        if info.filename == "Metadata/model_settings.config":
            text = data.decode()
            text, n = re.subn(r'(<plate>\s*)', r'\1<metadata key="enable_filament_dynamic_map" value="true"/>\n    '
                              r'<metadata key="has_filament_switcher" value="true"/>\n    ', text, count=1)
            assert n == 1, "no <plate> in model_settings.config"
            data = text.encode()
        zout.writestr(info, data)
PY
    expect_header_of() { # label, input, wanted G-code lines, flags...
        local label="$1" input="$2" want="$3"; shift 3
        local status; status=$(slice "$label" "$input" "$@")
        [ "$status" = 0 ] || { fail "$label" "exit $status"; tail -n 3 "$WORKDIR/$label/stderr"; return 1; }
        python3 - "$WORKDIR/$label" "$want" <<'PY' && pass "$label" || fail "$label" "the G-code does not state: $want"
import sys, zipfile
d, want = sys.argv[1:]
z = zipfile.ZipFile(d + "/sliced.3mf")
gcode = z.read([n for n in z.namelist() if n.endswith(".gcode")][0]).decode(errors="ignore")
sys.exit(0 if all(("\n; " + line + "\n") in gcode for line in want.split(";;")) else 1)
PY
    }
    expect_header_of "switcher-off-without-the-flag" "$SWITCHER" "enable_filament_dynamic_map = 0;;has_filament_switcher = 0"
    expect_header_of "switcher-on-with-the-flag" "$SWITCHER" "enable_filament_dynamic_map = 1;;has_filament_switcher = 1" \
        --enable-filament-dynamic-map=1 --has-filament-switcher=1

    # Bambu PET-CF prints with the H2D's left extruder only (filament_printable).
    CF=(--filament-preset "Bambu PET-CF @BBL H2D" --filament-preset "Bambu PET-CF @BBL H2D")
    expect_header "printable-on-the-mapped-extruder" "filament_map = 1,1" "" "${CF[@]}" --filament-map 1,1
    CODE=-68 expect_refusal "refuse-extruder-cannot-print" "$WORKDIR/two.3mf" "$H2D" "cannot print with extruder 2 (right)" \
        "${CF[@]}" --filament-map 2,2
    expect_refusal "refuse-nozzle-manual-no-rack" "$WORKDIR/two.3mf" "$H2D" "has no nozzle rack" "${D2[@]}" \
        --filament-map-mode "Nozzle Manual" --filament-map 1,2
    expect_refusal "refuse-quality-same-extruders" "$WORKDIR/two.3mf" "$H2D" "has no two extruders of different types" \
        "${D2[@]}" --filament-map-mode "Auto For Quality"
    PRINTER="Bambu Lab X2D 0.4 nozzle"
    expect_header "quality-mode" "filament_map_mode = Auto For Quality" "" \
        --filament-preset "Bambu PLA Basic @BBL X2D 0.4 nozzle" --filament-preset "Bambu PLA Basic @BBL X2D 0.4 nozzle" \
        --filament-map-mode "Auto For Quality"

    # The H2C: left extruder one nozzle, right extruder a rack.
    PRINTER="$H2C"; C2=(--filament-preset "Bambu PLA Basic @BBL H2C" --filament-preset "Bambu PLA Basic @BBL H2C")
    expect_header "h2c-rack-from-command-line" 'extruder_nozzle_count = 1,3;;extruder_nozzle_stats = Standard#1;"Standard#2|High Flow#1"' "" \
        "${C2[@]}" --filament-map 2,2 --extruder-nozzle-count 1,3 \
        --extruder-nozzle-volume-type "Standard,Standard,High Flow,Standard"
    expect_header "h2c-nozzle-manual" "filament_map_mode = Nozzle Manual;;filament_nozzle_map = 1,3" "1,3" \
        "${C2[@]}" --filament-map-mode "Nozzle Manual" --filament-map 2,2 --filament-volume-map 0,0 \
        --filament-nozzle-map 1,3 --extruder-nozzle-count 1,3 \
        --extruder-nozzle-volume-type "Standard,Standard,Standard,Standard"
    expect_refusal "refuse-nozzle-manual-incomplete" "$WORKDIR/two.3mf" "$H2C" \
        "needs the rack and every filament's place in it" "${C2[@]}" --filament-map-mode "Nozzle Manual" --filament-map 2,2

    # The H3C: one extruder with a rack. --filament-map is Bambu Studio's
    # grouping, 1 = the nozzle the AMS feeds, 2 = the fixed nozzles.
    PRINTER="$H3C"; H2=(--filament-preset "Bambu PLA Basic @BBL H3C" --filament-preset "Bambu PLA Basic @BBL H3C")
    # The preset's ams_nozzle_idx (0) states the AMS, as in the official CLI.
    expect_header "h3c-ams-then-fixed" "filament_map = 1,1;;filament_nozzle_map = 0,1;;filament_map_mode = Manual" "0,1" \
        "${H2[@]}" --filament-map 1,2
    expect_header "h3c-fixed-then-ams" "filament_map = 1,1;;filament_nozzle_map = 1,0;;filament_map_mode = Manual" "1,0" \
        "${H2[@]}" --ams-nozzle-idx 0 --filament-map 2,1
    expect_header "h3c-nozzle-manual" "filament_map_mode = Nozzle Manual;;filament_nozzle_map = 2,4" "2,4" \
        "${H2[@]}" --filament-map-mode "Nozzle Manual" --filament-map 1,1 --filament-volume-map 0,0 \
        --filament-nozzle-map 2,4 --extruder-nozzle-count 6 \
        --extruder-nozzle-volume-type "Standard,Standard,Standard,Standard,Standard,Standard"
    expect_refusal "refuse-h3c-no-ams" "$WORKDIR/two.3mf" "$H3C" "the settings state no AMS on this printer" \
        "${H2[@]}" --ams-nozzle-idx=-1 --filament-map 1,2
    expect_refusal "refuse-h3c-group" "$WORKDIR/two.3mf" "$H3C" "has one extruder with a nozzle rack" \
        "${H2[@]}" --ams-nozzle-idx 0 --filament-map 3,1
fi
PRINTER="$ONE"

echo "filament-map: $PASS passed, $FAIL failed"
[ "$FAIL" -eq 0 ]
