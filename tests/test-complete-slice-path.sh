#!/usr/bin/env bash
# The complete slice path on a PACKAGED engine pair: --slice/--outputdir
# (result.json + progress), --arrange, presets by name, --export-3mf, --info,
# --list-presets, and the default-path checks (printer this engine lacks,
# percentage line widths, file newer than the engine, value ranges).
# Usage: tests/test-complete-slice-path.sh /path/to/slicer_cli[.exe] /path/to/slicer_cli-orcaslicer[.exe]
set -euo pipefail

B="${1:?usage: $0 /path/to/packaged/slicer_cli /path/to/packaged/slicer_cli-orcaslicer}"
O="${2:?usage: $0 /path/to/packaged/slicer_cli /path/to/packaged/slicer_cli-orcaslicer}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
FIXTURE="$SCRIPT_DIR/fixtures/calib_base.3mf"
WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT
# Absolute binaries, then work with relative paths only (Windows python and the
# .exe both read relative paths; MSYS absolute paths they do not).
B="$(cd "$(dirname "$B")" && pwd -P)/$(basename "$B")"
O="$(cd "$(dirname "$O")" && pwd -P)/$(basename "$O")"
cp "$FIXTURE" "$WORKDIR/base.3mf"
FIXTURE=base.3mf
cd "$WORKDIR"

A1M="Bambu Lab A1 mini 0.4 nozzle"
A1M_PROCESS="0.20mm Standard @BBL A1M"
A1M_FILAMENT="Bambu PLA Basic @BBL A1M"

fail() { echo "FAIL: $*"; exit 1; }
# run NAME BINARY ARGS... : stdout/stderr/rc kept under NAME/; never aborts the script.
run() {
    local name=$1; shift
    mkdir -p "$name"
    set +e
    "$@" > "$name/stdout" 2> "$name/stderr"
    echo $? > "$name/rc"
    set -e
}
rc() { cat "$1/rc"; }
show() { tail -n 20 "$1/stderr"; grep '^\[\[SLICER_EVENT\]\]' "$1/stdout" | grep -v engine_log | tail -n 5 || true; }
# py SCRIPT ARGS... : python3 on Windows runners needs native paths, so pass relative ones.
py() { python3 -c "$@"; }

# Two ASCII STLs: a 20 mm cube and a 300 mm box (larger than the A1 mini bed).
py '
import sys
def box(path, s, h):
    v = [(x, y, z) for z in (0, h) for y in (0, s) for x in (0, s)]
    f = [(0,2,1),(1,2,3),(4,5,6),(5,7,6),(0,1,4),(1,5,4),(2,6,3),(3,6,7),(0,4,2),(2,4,6),(1,3,5),(3,7,5)]
    with open(path, "w") as o:
        o.write("solid t\n")
        for a, b, c in f:
            o.write("facet normal 0 0 0\nouter loop\n")
            for i in (a, b, c): o.write("vertex %g %g %g\n" % v[i])
            o.write("endloop\nendfacet\n")
        o.write("endsolid t\n")
box("cube.stl", 20, 20)
box("big.stl", 300, 10)
'

# Copies of the Bambu fixture with one change each.
py '
import json, re, shutil, zipfile
def rewrite(dst, settings=None, app=None):
    with zipfile.ZipFile("'"$FIXTURE"'") as zin, zipfile.ZipFile(dst, "w", zipfile.ZIP_DEFLATED) as zout:
        for item in zin.infolist():
            data = zin.read(item.filename)
            if settings and item.filename == "Metadata/project_settings.config":
                d = json.loads(data); d.update(settings); data = json.dumps(d, indent=4).encode()
            if app and item.filename == "3D/3dmodel.model":
                data = re.sub(rb"(<metadata name=\"Application\">)[^<]*", rb"\g<1>" + app.encode(), data)
            zout.writestr(item, data)
rewrite("pct.3mf", settings={"skin_infill_line_width": "100%", "support_line_width": "105%"})
rewrite("u1.3mf", settings={"printer_model": "Snapmaker U1"})
rewrite("newer.3mf", app="BambuStudio-99.01.00.00")

# Two plates, the cube on plate 2. The desktop lays plates out in a grid:
# 2 plates -> 2 columns, stride = 256 mm bed * 1.2, so plate 2 starts at
# x = 307.2 and the cube saved at (128, 128) on it is stored at (435.2, 128).
with zipfile.ZipFile("base.3mf") as zin, zipfile.ZipFile("two-plates.3mf", "w", zipfile.ZIP_DEFLATED) as zout:
    for item in zin.infolist():
        data = zin.read(item.filename)
        if item.filename == "3D/3dmodel.model":
            data = data.replace(b"127.999992 127.999992 12.8000002", b"435.199992 127.999992 12.8000002")
        if item.filename == "Metadata/model_settings.config":
            text = data.decode()
            start = text.index("  <plate>"); end = text.index("  </plate>") + len("  </plate>\n")
            plate = text[start:end]
            empty = re.sub(r"    <model_instance>.*?</model_instance>\n", "", plate, flags=re.S)
            second = plate.replace("key=\"plater_id\" value=\"1\"", "key=\"plater_id\" value=\"2\"")
            text = text[:start] + empty + second + text[end:]
            data = text.encode()
        zout.writestr(item, data)
'

# --info names the printer and the binaries that have it.
"$B" --info "$FIXTURE" > info.json
py '
import json; d = json.load(open("info.json"))
assert d["printer_model"] == "Bambu Lab X1 Carbon", d
assert "slicer_cli" in d["fits"], d
'
"$B" --info u1.3mf > info-u1.json
py '
import json; d = json.load(open("info-u1.json"))
assert d["fits"] == ["slicer_cli-orcaslicer"], d
'
# The kind comes from the final extension, any case, not from a folder name.
mkdir -p "dir.stl"
cp "$FIXTURE" "dir.stl/Project.3Mf"
"$B" --info "dir.stl/Project.3Mf" > info-ext.json
py '
import json; d = json.load(open("info-ext.json"))
assert d["kind"] == "3mf" and d["printer_model"] == "Bambu Lab X1 Carbon", d
'
echo "PASS: --info names the printer and the engine that fits"

# --slice N --outputdir: one G-code per plate, result.json in the official shape, progress to 100.
run slice "$B" "$FIXTURE" --slice 1 --outputdir slice/out
[ "$(rc slice)" = 0 ] || { show slice; fail "--slice 1 exit $(rc slice)"; }
test -s slice/out/plate_1.gcode || fail "no plate_1.gcode"
py '
import json; d = json.load(open("slice/out/result.json"))
assert d["return_code"] == 0 and d["error_string"] == "Success.", d
p = d["sliced_plates"][0]
assert p["id"] == 1 and p["total_predication"] > 0 and p["objects"], p
'
grep -q '"event":"progress".*"percent":100' slice/stdout || fail "no progress event at 100"
echo "PASS: --slice writes plate_1.gcode, result.json and progress events"

# A result.json that cannot be written fails the run (a directory stands in its place).
mkdir -p noresult/out/result.json
run noresult "$B" "$FIXTURE" --slice 1 --outputdir noresult/out
[ "$(rc noresult)" != 0 ] || fail "--slice exited 0 without writing result.json"
grep -q '"tag":"ResultNotWritten"' noresult/stdout || { show noresult; fail "no ResultNotWritten event"; }
grep -q '"percent":100' noresult/stdout && fail "progress reached 100 on a run without result.json"
echo "PASS: a result.json that cannot be written fails the run"

# Parts stay where the maker put them: the fixture's 25.6 mm cube is saved
# centred at (128, 128), so its box starts at (115.2, 115.2). The plate's own
# plate_1.json describes another part; it must not move the cube.
py '
import json, sys
o = json.load(open(sys.argv[1]))["sliced_plates"][0]["objects"][0]["bbox"]
assert abs(o["x"] - 115.2) < 0.5 and abs(o["y"] - 115.2) < 0.5, o
' slice/out/result.json
run plate2 "$B" two-plates.3mf --slice 2 --outputdir plate2/out
[ "$(rc plate2)" = 0 ] || { show plate2; fail "plate 2 of a two-plate file exit $(rc plate2)"; }
py '
import json, sys
o = json.load(open(sys.argv[1]))["sliced_plates"][0]["objects"][0]["bbox"]
assert abs(o["x"] - 115.2) < 0.5 and abs(o["y"] - 115.2) < 0.5, o
' plate2/out/result.json
echo "PASS: plates keep the maker's positions (plate grid origin, no corner snap)"

# --export-3mf: the sliced project carries the plate G-code.
run export "$B" "$FIXTURE" --slice 1 --outputdir export/out --export-3mf sliced.3mf
[ "$(rc export)" = 0 ] || { show export; fail "--export-3mf exit $(rc export)"; }
py '
import zipfile; n = zipfile.ZipFile("export/out/sliced.3mf").namelist()
assert "Metadata/plate_1.gcode" in n and "Metadata/slice_info.config" in n, n
'
# Progress reaches 100 only after the 3MF is written (official ladder:
# last plate 93, "Exporting 3mf" 97, "All done, Success" 100).
py '
import json
events = []
for line in open("export/stdout", encoding="utf-8", errors="replace"):
    line = line.strip()
    if line.startswith("[[SLICER_EVENT]] "):
        events.append(json.loads(line[len("[[SLICER_EVENT]] "):]))
written = [i for i, e in enumerate(events) if e.get("event") == "exported_3mf"]
progress = [(i, e) for i, e in enumerate(events) if e.get("event") == "progress"]
assert written and progress, (written, len(progress))
assert all(e["percent"] < 100 for i, e in progress if i < written[0]), [e for i, e in progress if e["percent"] >= 100]
last = progress[-1][1]
assert progress[-1][0] > written[0] and last["percent"] == 100 and last["message"] == "All done, Success", last
'
echo "PASS: --export-3mf writes the sliced 3MF; progress reaches 100 after it"

# The exported project states the settings its G-code was sliced with: the
# Bambu build's mm line widths and a command-line override.
run pct-export "$B" pct.3mf --slice 1 --layer-height 0.16 --outputdir pct-export/out --export-3mf sliced.3mf
[ "$(rc pct-export)" = 0 ] || { show pct-export; fail "--export-3mf with converted widths exit $(rc pct-export)"; }
py '
import json, zipfile
d = json.loads(zipfile.ZipFile("pct-export/out/sliced.3mf").read("Metadata/project_settings.config"))
def num(k):
    v = d[k]; v = v[0] if isinstance(v, list) else v
    return float(str(v).rstrip("%"))
assert abs(num("skin_infill_line_width") - 0.4) < 1e-6, d["skin_infill_line_width"]
assert abs(num("support_line_width") - 0.42) < 1e-6, d["support_line_width"]
assert abs(num("layer_height") - 0.16) < 1e-6, d["layer_height"]
'
echo "PASS: the exported project keeps the converted widths and the command-line override"

# ... and the custom G-code as it ran: the legacy placeholder aliased.
py '
import json, zipfile
with zipfile.ZipFile("base.3mf") as zin, zipfile.ZipFile("legacy.3mf", "w", zipfile.ZIP_DEFLATED) as zout:
    for item in zin.infolist():
        data = zin.read(item.filename)
        if item.filename == "Metadata/project_settings.config":
            d = json.loads(data)
            d["machine_start_gcode"] = d["machine_start_gcode"] + "\n; first {initial_no_support_filament_id}\n"
            data = json.dumps(d, indent=4).encode()
        zout.writestr(item, data)
'
run legacy-export "$B" legacy.3mf --slice 1 --outputdir legacy-export/out --export-3mf sliced.3mf
[ "$(rc legacy-export)" = 0 ] || { show legacy-export; fail "--export-3mf with a legacy placeholder exit $(rc legacy-export)"; }
py '
import json, zipfile
g = json.loads(zipfile.ZipFile("legacy-export/out/sliced.3mf").read("Metadata/project_settings.config"))["machine_start_gcode"]
assert "{initial_no_support_extruder}" in g and "initial_no_support_filament_id" not in g, g[-200:]
'
[ "$(grep -c '"tag":"LegacyGcodeTokenAliased"' legacy-export/stdout)" = 1 ] || fail "the placeholder alias was reported more than once"
echo "PASS: the exported project states the custom G-code the slice ran (reported once)"

# --export-3mf may not name the run's own result.json or plate G-code.
for name in result.json PLATE_1.gcode; do
    run clash "$B" "$FIXTURE" --slice 1 --outputdir clash/out --export-3mf "$name"
    [ "$(rc clash)" != 0 ] || fail "--export-3mf $name was accepted"
    grep -q '"tag":"ExportNameTaken"' clash/stdout || { show clash; fail "no ExportNameTaken event for $name"; }
    py '
import json; d = json.load(open("clash/out/result.json"))
assert d["return_code"] != 0 and "export-3mf" in d["error_string"], d
'
    rm -rf clash
done
echo "PASS: --export-3mf refuses the name of a file the run writes"

# A command-line override is range-checked like the file's values, and a run
# that fails leaves no earlier run's project at the --export-3mf name.
mkdir -p badlh/out
echo "stale" > badlh/out/sliced.3mf
run badlh "$B" "$FIXTURE" --slice 1 --layer-height -0.1 --outputdir badlh/out --export-3mf sliced.3mf
[ "$(rc badlh)" != 0 ] || fail "--layer-height -0.1 was sliced"
py '
import json; d = json.load(open("badlh/out/result.json"))
assert d["return_code"] != 0 and "layer_height" in d["error_string"], d
'
[ ! -e badlh/out/sliced.3mf ] || fail "a stale sliced.3mf survived a failed run"
echo "PASS: a bad override is refused before slicing; no stale project is left"

# Only an earlier export (a plain .3mf name inside --outputdir) is ever removed:
# a NAME pointing outside it is never deleted by a run that fails.
mkdir -p keep/out
echo "keep" > keep/important.txt
run keep "$B" "$FIXTURE" --slice 1 --layer-height -0.1 --outputdir keep/out --export-3mf ../important.txt
[ "$(rc keep)" != 0 ] || fail "--layer-height -0.1 was sliced"
[ "$(cat keep/important.txt)" = keep ] || fail "a failed run deleted a file outside --outputdir"
echo "PASS: a failed export run deletes nothing outside --outputdir"

# An earlier export in a subfolder of --outputdir is cleared like one at the top.
mkdir -p nested/out/projects
echo "stale" > nested/out/projects/sliced.3mf
run nested "$B" "$FIXTURE" --slice 1 --layer-height -0.1 --outputdir nested/out --export-3mf projects/sliced.3mf
[ "$(rc nested)" != 0 ] || fail "--layer-height -0.1 was sliced"
[ ! -e nested/out/projects/sliced.3mf ] || fail "a stale nested sliced.3mf survived a failed run"
echo "PASS: a stale export in a subfolder of --outputdir is cleared"

# A 3MF without plate metadata slices as one plate, and its export carries
# that plate's G-code (the official CLI always has plate 1).
py '
import re, zipfile
with zipfile.ZipFile("base.3mf") as zin, zipfile.ZipFile("noplates.3mf", "w", zipfile.ZIP_DEFLATED) as zout:
    for item in zin.infolist():
        data = zin.read(item.filename)
        if item.filename == "Metadata/model_settings.config":
            data = re.sub(r"\s*<plate>.*?</plate>", "", data.decode(), flags=re.S).encode()
            assert b"<plate>" not in data
        zout.writestr(item, data)
'
run noplates "$B" noplates.3mf --slice 1 --outputdir noplates/out --export-3mf sliced.3mf
[ "$(rc noplates)" = 0 ] || { show noplates; fail "plate-less 3MF --export-3mf exit $(rc noplates)"; }
py '
import zipfile; n = zipfile.ZipFile("noplates/out/sliced.3mf").namelist()
assert "Metadata/plate_1.gcode" in n, n
'
echo "PASS: a plate-less 3MF exports with its plate G-code"

# Presets by name with every parent applied (desktop result: 220 C, 200 mm/s
# outer wall) and --arrange 1 centring the cube on the 180 mm A1 mini bed. Both engines.
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run cube-$e "$bin" cube.stl --slice 1 --arrange 1 --printer-preset "$A1M" --process-preset "$A1M_PROCESS" --filament-preset "$A1M_FILAMENT" --outputdir cube-$e/out
    [ "$(rc cube-$e)" = 0 ] || { show cube-$e; fail "$e: named presets + arrange exit $(rc cube-$e)"; }
    py '
import re, sys
g = open(sys.argv[1]).read()
def cfg(k):
    m = re.search(r"^; " + k + r" = (.*)$", g, re.M); return m.group(1).strip() if m else None
assert cfg("nozzle_temperature").split(",")[0] == "220", cfg("nozzle_temperature")
assert float(cfg("outer_wall_speed").split(",")[0]) == 200, cfg("outer_wall_speed")
xs = [float(m) for m in re.findall(r"^G1 [^;\n]*X([0-9.]+)[^;\n]*E[0-9.]", g, re.M)]
assert xs and 70 <= min(xs) and max(xs) <= 110, (min(xs), max(xs))
' "cube-$e/out/plate_1.gcode"
done
echo "PASS: named presets apply their parents and --arrange centres the part (both engines)"

# --list-presets --printer gives the printer default process.
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    "$bin" --list-presets --printer "$A1M" > list-$e.json
    py '
import json, sys; d = json.load(open(sys.argv[1]))
assert d["default_process"] == "0.20mm Standard @BBL A1M", d.get("default_process")
assert "0.20mm Standard @BBL A1M" in d["processes"]
' list-$e.json
done
echo "PASS: --list-presets names the default process (both engines)"

# A printer whose stated default the engine does not ship (Orca pin: the
# Snapmaker U1 0.4 names "0.20mm Standard @Snapmaker"): the listing gives the
# fallback a slice would take, so the listed default is one it accepts.
"$O" --list-presets --printer "Snapmaker U1 (0.4 nozzle)" > list-u1.json
py '
import json; d = json.load(open("list-u1.json"))
assert d["default_process"] in d["processes"], (d["default_process"], d["processes"][:5])
assert all(f in d["filaments"] for f in d["default_filaments"]), d["default_filaments"]
assert any("process" in r for r in d["defaults_replaced"]), d["defaults_replaced"]
'
echo "PASS: --list-presets gives the default a slice takes when the stated one is missing"

# A part larger than the bed: refused with the official -50 code and its size.
run big-bambu "$B" big.stl --slice 1 --arrange 1 --printer-preset "$A1M" --outputdir big-bambu/out
run big-orca "$O" big.stl --slice 1 --arrange 1 --printer-preset "$A1M" --outputdir big-orca/out
for e in bambu orca; do
    [ "$(rc big-$e)" != 0 ] || fail "$e: 300 mm box was sliced"
    py '
import json, sys; d = json.load(open(sys.argv[1]))
assert d["return_code"] == -50, d
assert "300" in d["error_string"], d["error_string"]
' big-$e/out/result.json
done
echo "PASS: a part larger than the bed is refused with its size (both engines)"

# Default path: a printer this engine does not have is refused, naming the binary that has it.
run u1 "$B" u1.3mf --plate 1 -o u1/out.gcode
[ "$(rc u1)" != 0 ] || fail "Bambu build sliced a Snapmaker U1 file"
grep -q 'use slicer_cli-orcaslicer' u1/stderr || { show u1; fail "refusal does not name slicer_cli-orcaslicer"; }
echo "PASS: a printer this engine lacks is refused, naming the engine that has it"

# Default path: percentage line widths on the Bambu build are a share of the 0.4 mm nozzle.
run pct "$B" pct.3mf --plate 1 -o pct/out.gcode
[ "$(rc pct)" = 0 ] || { show pct; fail "percent line widths exit $(rc pct)"; }
tr -d '\r' < pct/out.gcode > pct/out.lf
grep -q '^; skin_infill_line_width = 0.4$' pct/out.lf || fail "100% skin line width is not 0.4 mm"
grep -q '^; support_line_width = 0.42$' pct/out.lf || fail "105% support line width is not 0.42 mm"
grep -q '"tag":"PercentLineWidthConverted"' pct/stdout || fail "no PercentLineWidthConverted event"
echo "PASS: percentage line widths convert to mm on the Bambu build"

# A file newer than the engine: refused unless --allow-newer-file.
run newer "$B" newer.3mf --plate 1 -o newer/out.gcode
[ "$(rc newer)" != 0 ] || fail "a 99.1 file was sliced without --allow-newer-file"
grep -q 'FileVersionNewerThanEngine' newer/stdout || { show newer; fail "no FileVersionNewerThanEngine event"; }
run newer-ok "$B" newer.3mf --plate 1 --allow-newer-file -o newer-ok/out.gcode
[ "$(rc newer-ok)" = 0 ] || { show newer-ok; fail "--allow-newer-file exit $(rc newer-ok)"; }
echo "PASS: a newer file is refused unless --allow-newer-file"

# A Bambu Studio 2.5 file on the Orca build (2.4): refused as newer, naming the Bambu build.
run orca-newer "$O" "$FIXTURE" --plate 1 -o orca-newer/out.gcode
[ "$(rc orca-newer)" != 0 ] || fail "Orca sliced a newer Bambu Studio file"
grep -q 'use slicer_cli (BambuStudio)' orca-newer/stderr || { show orca-newer; fail "refusal does not name slicer_cli"; }
echo "PASS: Orca refuses a newer Bambu Studio file, naming slicer_cli"

# Cross-engine values on the Orca build: out-of-range and unknown values refused in one sentence.
# A G-code left from an earlier run in the same --outputdir must not survive a failed plate.
mkdir -p orca-values/out
echo "; stale" > orca-values/out/plate_1.gcode
run orca-values "$O" "$FIXTURE" --slice 1 --allow-newer-file --outputdir orca-values/out
[ "$(rc orca-values)" != 0 ] || fail "Orca sliced a file with values it does not have"
[ ! -e orca-values/out/plate_1.gcode ] || fail "a stale plate_1.gcode survived a failed plate"
grep -q 'tree_support_wall_count: -1 not in range' orca-values/stderr || { show orca-values; fail "no range refusal"; }
grep -q "'ensure_vertical_shell_thickness' is 'enabled'" orca-values/stderr || { show orca-values; fail "no unknown-value refusal"; }
echo "PASS: Orca refuses out-of-range and unknown values, naming each"
