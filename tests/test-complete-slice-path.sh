#!/usr/bin/env bash
# The complete slice path on a PACKAGED engine pair: --slice/--outputdir
# (result.json + progress), --arrange, presets by name, --export-3mf, --engine-info,
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

# --engine-info names the printer and the binaries that have it.
"$B" --engine-info "$FIXTURE" > info.json
py '
import json; d = json.load(open("info.json"))
assert d["printer_model"] == "Bambu Lab X1 Carbon", d
assert "slicer_cli" in d["fits"], d
'
"$B" --engine-info u1.3mf > info-u1.json
py '
import json; d = json.load(open("info-u1.json"))
assert d["fits"] == ["slicer_cli-orcaslicer"], d
'
# The kind comes from the final extension, any case, not from a folder name.
mkdir -p "dir.stl"
cp "$FIXTURE" "dir.stl/Project.3Mf"
"$B" --engine-info "dir.stl/Project.3Mf" > info-ext.json
py '
import json; d = json.load(open("info-ext.json"))
assert d["kind"] == "3mf" and d["printer_model"] == "Bambu Lab X1 Carbon", d
'
# An unreadable 3MF (not a ZIP, a folder named .3mf, or a model part that
# fails its CRC) is refused (-2).
echo "not a zip" > broken.3mf
mkdir -p folder.3mf
py '
import zipfile
with zipfile.ZipFile("crc.3mf", "w", zipfile.ZIP_STORED) as z:
    z.writestr("3D/3dmodel.model", "<model>" + "x" * 64 + "</model>")
data = bytearray(open("crc.3mf", "rb").read())
at = data.index(b"xxxx")
data[at] = ord("y")
open("crc.3mf", "wb").write(bytes(data))
'
for bad in broken.3mf folder.3mf crc.3mf; do
    if "$B" --engine-info "$bad" > info-bad.json; then fail "--engine-info accepted $bad"; fi
    py '
import json; d = json.load(open("info-bad.json"))
assert "error" in d and "3D/3dmodel.model" in d["error"], d
'
done
# A real STL is inspected; an unreadable one (a folder named .stl, or an
# empty file) is refused (-2).
"$B" --engine-info cube.stl > info-stl.json || fail "--engine-info refused cube.stl"
py '
import json; d = json.load(open("info-stl.json"))
assert d["kind"] == "stl" and "error" not in d, d
'

mkdir -p folder.stl
: > empty.stl
for bad in folder.stl empty.stl; do
    if "$B" --engine-info "$bad" > info-bad.json; then fail "--engine-info accepted $bad"; fi
    py '
import json; d = json.load(open("info-bad.json"))
assert "error" in d and "STL" in d["error"], d
'
done
echo "PASS: --engine-info names the printer and the engine that fits"

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

# A missing input is "not found" (-3), not an unparseable model.
run missing "$B" no-such-file.3mf --slice 1 --outputdir missing/out
[ "$(rc missing)" != 0 ] || fail "a missing input was sliced"
py '
import json; d = json.load(open("missing/out/result.json"))
assert d["return_code"] == -3, d
'
echo "PASS: a missing --slice input is reported as not found"

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
# result.json states each object's box in the scene, as the official
# sliced_info does (object->bounding_box(), BambuStudio.cpp 7353): plate 2 of
# two 256 mm plates sits at (256 * 1.2, 0).
py '
import json, sys
o = json.load(open(sys.argv[1]))["sliced_plates"][0]["objects"][0]["bbox"]
assert abs(o["x"] - (115.2 + 307.2)) < 0.5 and abs(o["y"] - 115.2) < 0.5, o
' plate2/out/result.json
echo "PASS: plates keep the maker's positions (plate grid origin, no corner snap)"

# --export-3mf: the sliced project carries the plate G-code.
run export "$B" "$FIXTURE" --slice 1 --outputdir export/out --export-3mf sliced.3mf
[ "$(rc export)" = 0 ] || { show export; fail "--export-3mf exit $(rc export)"; }
py '
import zipfile; z = zipfile.ZipFile("export/out/sliced.3mf"); n = z.namelist()
assert "Metadata/plate_1.gcode" in n and "Metadata/slice_info.config" in n, n
# The exported plate still lists its object (model_instance), so the project
# reopens with the part on plate 1.
m = z.read("Metadata/model_settings.config").decode()
plate = m[m.index("<plate>"):m.index("</plate>")]
assert "<model_instance>" in plate, plate
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

# ... and a NAME that is a link to one of them (POSIX only: Git Bash on
# Windows makes copies, not links).
case "$(uname -s)" in
MINGW*|MSYS*|CYGWIN*) echo "SKIP: export name links (Windows)";;
*)
    mkdir -p alias/out
    ln -s result.json alias/out/link.3mf
    run alias "$B" "$FIXTURE" --slice 1 --outputdir alias/out --export-3mf link.3mf
    [ "$(rc alias)" != 0 ] || fail "--export-3mf through a link to result.json was accepted"
    grep -q '"tag":"ExportNameTaken"' alias/stdout || { show alias; fail "no ExportNameTaken event for a link"; }
    echo "PASS: --export-3mf refuses a link to the run's result.json";;
esac

# A command-line override is range-checked like the file's values.
run badlh "$B" "$FIXTURE" --slice 1 --layer-height -0.1 --outputdir badlh/out --export-3mf sliced.3mf
[ "$(rc badlh)" != 0 ] || fail "--layer-height -0.1 was sliced"
py '
import json; d = json.load(open("badlh/out/result.json"))
assert d["return_code"] != 0 and "layer_height" in d["error_string"], d
'
echo "PASS: a bad override is refused before slicing"

# A failed run deletes nothing: like the official CLI, --export-3mf only ever
# overwrites on success, and result.json says whether this run worked.
mkdir -p keep/out
echo "keep" > keep/important.txt
run keep "$B" "$FIXTURE" --slice 1 --layer-height -0.1 --outputdir keep/out --export-3mf ../important.txt
[ "$(rc keep)" != 0 ] || fail "--layer-height -0.1 was sliced"
[ "$(cat keep/important.txt)" = keep ] || fail "a failed run deleted a file outside --outputdir"
echo "PASS: a failed export run deletes nothing"


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

# The Snapmaker U1 0.4 names a default process Orca does not ship ("0.20mm
# Standard @Snapmaker", fdm_toolchanger.json) and a default filament it has no
# compatible preset of ("Snapmaker PLA"). A fresh OrcaSlicer 2.4.2 app then
# selects "0.08 Extra Fine @Snapmaker U1 (0.4 nozzle)" and "Generic ABS
# @System"; a slice with only --printer-preset takes the same, and says so.
"$O" --list-presets --printer "Snapmaker U1 (0.4 nozzle)" > list-u1.json
py '
import json; d = json.load(open("list-u1.json"))
assert d["default_process"] == "0.08 Extra Fine @Snapmaker U1 (0.4 nozzle)", d["default_process"]
assert d["default_filaments"] == ["Generic ABS @System"], d["default_filaments"]
assert len(d["processes"]) == 15, len(d["processes"])
assert any(r.startswith("process") for r in d["defaults_replaced"]), d["defaults_replaced"]
assert any(r.startswith("filament") for r in d["defaults_replaced"]), d["defaults_replaced"]
'
echo "PASS: the Snapmaker U1 0.4 defaults are the ones the OrcaSlicer app selects"

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

# Each engine's own X1 Carbon project of the 20 mm cube (base.3mf is newer
# than the OrcaSlicer build reads), and two copies of it: edge-<engine>.3mf
# with the cube moved over the bed's edge (x = 250 on the 256 mm bed), and
# platebed-<engine>.3mf with plate 1 stating its own High Temp Plate.
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run proj-$e "$bin" cube.stl --slice 1 --printer-preset "Bambu Lab X1 Carbon 0.4 nozzle" --outputdir proj-$e/out --export-3mf proj.3mf
    [ "$(rc proj-$e)" = 0 ] || { show proj-$e; fail "$e: X1 Carbon cube project exit $(rc proj-$e)"; }
    py '
import sys, zipfile
def rewrite(dst, name, old, new):
    with zipfile.ZipFile(sys.argv[1]) as zin, zipfile.ZipFile(dst, "w", zipfile.ZIP_DEFLATED) as zout:
        for item in zin.infolist():
            data = zin.read(item.filename)
            if item.filename == name:
                assert old in data, (name, old)
                data = data.replace(old, new, 1)
            zout.writestr(item, data)
e = sys.argv[2]
rewrite("edge-%s.3mf" % e, "3D/3dmodel.model", b" 128 128 10\"", b" 250 128 10\"")
rewrite("platebed-%s.3mf" % e, "Metadata/model_settings.config", b"  <plate>\n",
        b"  <plate>\n    <metadata key=\"bed_type\" value=\"High Temp Plate\"/>\n")
' proj-$e/out/proj.3mf $e
done

# --no-check does not skip the printable-area refusals: the official CLI
# refuses a plate with nothing fully inside (-50, BambuStudio.cpp 6530-6535;
# OrcaSlicer.cpp 5647-5652) and an object over the bed's edge (-52,
# BambuStudio.cpp 6576-6581; OrcaSlicer.cpp 5693-5698) whatever no_check says.
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run bignc-$e "$bin" big.stl --slice 1 --arrange 1 --no-check --printer-preset "$A1M" --outputdir bignc-$e/out
    run edgenc-$e "$bin" edge-$e.3mf --slice 1 --no-check --outputdir edgenc-$e/out
    py '
import json, sys
for path, want in ((sys.argv[1], -50), (sys.argv[2], -52)):
    d = json.load(open(path))
    assert d["return_code"] == want, (path, d)
' bignc-$e/out/result.json edgenc-$e/out/result.json
done
echo "PASS: --no-check keeps the printable-area refusals (both engines)"

# --arrange 1 keeps clear of the bed exclusion area, as the official per-plate
# arrange does (PartPlateList::preprocess_exclude_areas, fixed items plus
# excluded regions). The X1 Carbon excludes its 18 x 28 mm front-left corner;
# a 230 mm square centred on the 256 mm bed spans 13..243 and would cross it,
# so the arrange must put the square at x >= 18 or y >= 28.
X1C="Bambu Lab X1 Carbon 0.4 nozzle"
py '
v = [(x, y, z) for z in (0, 2) for y in (0, 230) for x in (0, 230)]
f = [(0,2,1),(1,2,3),(4,5,6),(5,7,6),(0,1,4),(1,5,4),(2,6,3),(3,6,7),(0,4,2),(2,4,6),(1,3,5),(3,7,5)]
with open("wide.stl", "w") as o:
    o.write("solid t\n")
    for a, b, c in f:
        o.write("facet normal 0 0 0\nouter loop\n")
        for i in (a, b, c): o.write("vertex %g %g %g\n" % v[i])
        o.write("endloop\nendfacet\n")
    o.write("endsolid t\n")
'
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run wide-$e "$bin" wide.stl --slice 1 --arrange 1 --printer-preset "$X1C" --process-preset "0.20mm Standard @BBL X1C" --filament-preset "Bambu PLA Basic @BBL X1C" --outputdir wide-$e/out
    [ "$(rc wide-$e)" = 0 ] || { show wide-$e; fail "$e: X1 Carbon arrange exit $(rc wide-$e)"; }
    py '
import json, sys
events = []
for line in open(sys.argv[1], encoding="utf-8", errors="replace"):
    line = line.strip()
    if line.startswith("[[SLICER_EVENT]] "):
        events.append(json.loads(line[len("[[SLICER_EVENT]] "):]))
arranged = [e for e in events if e.get("event") == "arranged"]
assert arranged, "no arranged event"
o = arranged[-1]["objects"][0]
x0, y0 = o["center_x_mm"] - 115, o["center_y_mm"] - 115
x1, y1 = o["center_x_mm"] + 115, o["center_y_mm"] + 115
assert -0.01 <= x0 and x1 <= 256.01 and -0.01 <= y0 and y1 <= 256.01, ("off the bed", x0, y0, x1, y1)
assert not (x0 < 18 and y0 < 28), ("over the exclusion area", x0, y0)
' wide-$e/stdout
done
echo "PASS: --arrange keeps clear of the bed exclusion area (both engines)"

# Default path: the plate's own settings apply over the project's (official
# new_print_config.apply(plate config)); plate 1 states a Cool Plate.
py '
import zipfile
with zipfile.ZipFile("base.3mf") as zin, zipfile.ZipFile("platebed.3mf", "w", zipfile.ZIP_DEFLATED) as zout:
    for item in zin.infolist():
        data = zin.read(item.filename)
        if item.filename == "Metadata/model_settings.config":
            t = data.decode()
            mark = "<metadata key=\"plater_id\" value=\"1\"/>"
            assert mark in t
            t = t.replace(mark, mark + "\n    <metadata key=\"bed_type\" value=\"Cool Plate\"/>", 1)
            data = t.encode()
        zout.writestr(item, data)
'
run platebed "$B" platebed.3mf --plate 1 -o platebed/out.gcode
[ "$(rc platebed)" = 0 ] || { show platebed; fail "plate bed type exit $(rc platebed)"; }
tr -d '\r' < platebed/out.gcode > platebed/out.lf
grep -q '^; curr_bed_type = Cool Plate$' platebed/out.lf || fail "the plate's Cool Plate was not applied"
grep -q '"tag":"PlateSettingsApplied"' platebed/stdout || fail "no PlateSettingsApplied event"
echo "PASS: the plate's own bed type applies over the project's"

# A plate value out of the engine's range is refused like a project value.
py '
import zipfile
with zipfile.ZipFile("base.3mf") as zin, zipfile.ZipFile("plateseq.3mf", "w", zipfile.ZIP_DEFLATED) as zout:
    for item in zin.infolist():
        data = zin.read(item.filename)
        if item.filename == "Metadata/model_settings.config":
            t = data.decode()
            mark = "<metadata key=\"plater_id\" value=\"1\"/>"
            assert mark in t
            t = t.replace(mark, mark + "\n    <metadata key=\"first_layer_print_sequence\" value=\"99\"/>", 1)
            data = t.encode()
        zout.writestr(item, data)
'
run plateseq "$B" plateseq.3mf --plate 1 -o plateseq/out.gcode
[ "$(rc plateseq)" != 0 ] || fail "a plate value out of range was sliced"
grep -q 'first_layer_print_sequence: 99 not in range' plateseq/stderr || { show plateseq; fail "no range refusal for the plate value"; }
echo "PASS: a plate value out of range is refused"

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

# A project whose plates carry no identify_id (#31): plate membership comes
# from position, as the desktop places every instance, and --arrange moves
# reach the exported project. The cube sits off-centre on plate 2 (plate 2
# starts at x = 307.2), so arranging it moves it to the plate's centre.
py '
import re, zipfile
with zipfile.ZipFile("base.3mf") as zin, zipfile.ZipFile("noid.3mf", "w", zipfile.ZIP_DEFLATED) as zout:
    for item in zin.infolist():
        data = zin.read(item.filename)
        if item.filename == "3D/3dmodel.model":
            assert b"127.999992 127.999992 12.8000002" in data
            data = data.replace(b"127.999992 127.999992 12.8000002", b"347.2 40 12.8000002")
        if item.filename == "Metadata/model_settings.config":
            text = re.sub(r"\s*<metadata key=\"identify_id\" value=\"[0-9]+\"/>", "", data.decode())
            start = text.index("  <plate>"); end = text.index("  </plate>") + len("  </plate>\n")
            plate = text[start:end]
            empty = re.sub(r"    <model_instance>.*?</model_instance>\n", "", plate, flags=re.S)
            second = plate.replace("key=\"plater_id\" value=\"1\"", "key=\"plater_id\" value=\"2\"")
            text = text[:start] + empty + second + text[end:]
            data = text.encode()
        zout.writestr(item, data)
'
run noid "$B" noid.3mf --slice 2 --arrange 1 --outputdir noid/out --export-3mf sliced.3mf
[ "$(rc noid)" = 0 ] || { show noid; fail "identify_id-less --arrange --export-3mf exit $(rc noid)"; }
py '
import re, zipfile
z = zipfile.ZipFile("noid/out/sliced.3mf")
model = z.read("Metadata/model_settings.config").decode()
plates = re.findall(r"<plate>.*?</plate>", model, re.S)
assert len(plates) == 2, len(plates)
assert "<model_instance>" not in plates[0] and "<model_instance>" in plates[1], plates
main = z.read("3D/3dmodel.model").decode()
items = re.findall(r"<item [^>]*transform=\"([^\"]+)\"", main)
assert len(items) == 1, items
x, y = (float(v) for v in items[0].split()[9:11])
assert abs(x - (307.2 + 128)) < 2 and abs(y - 128) < 2, (x, y)
'
echo "PASS: plates without identify_id keep their objects, and --arrange moves reach the export"

# ── Flags, plate types and model files (both engines) ─────────────────────

# events FILE KIND: the [[SLICER_EVENT]] records of one kind, as JSON lines.
events() { grep '^\[\[SLICER_EVENT\]\]' "$1" | sed 's/^\[\[SLICER_EVENT\]\] //' | grep "\"event\":\"$2\"" || true; }
# has_line FILE LINE: FILE (CRLF or LF) holds LINE exactly. No pipe into
# grep -q: with pipefail, grep -q's early exit fails the writer.
has_line() { tr -d '\r' < "$1" > "$1.lf"; grep -qxF -- "$2" "$1.lf"; }
bed_line() { tr -d '\r' < "$1" > "$1.lf"; grep -m1 '^; curr_bed_type' "$1.lf" || true; }

# An STL with only the printer named takes the plate type the desktop app
# picks for that printer, and the exported project keeps the filaments'
# filament_printable (no INT_MAX "nil" marker).
#   BambuStudio: a Bambu Lab printer's model default_bed_type when offered.
#   OrcaSlicer: Preset::get_default_bed_type (a number, else by model id).
bed_case() {  # engine binary printer expected-plate
    local e=$1 bin=$2 printer=$3 want=$4 n
    n="bed-$e-$(printf '%s' "$printer" | tr -c 'A-Za-z0-9' '_')"
    run "$n" "$bin" cube.stl --slice 1 --printer-preset "$printer" --outputdir "$n/out" --export-3mf c.3mf
    [ "$(rc "$n")" = 0 ] || { show "$n"; fail "$e $printer: exit $(rc "$n")"; }
    has_line "$n/out/plate_1.gcode" "; curr_bed_type = $want" ||
        fail "$e $printer: plate is $(bed_line "$n/out/plate_1.gcode"), want $want"
    py '
import json, sys, zipfile
s = zipfile.ZipFile(sys.argv[1]).read("Metadata/project_settings.config").decode()
assert "2147483647" not in s, "INT_MAX in the exported settings"
fp = json.loads(s).get("filament_printable")
assert fp and all(int(v) > 0 for v in fp), fp
' "$n/out/c.3mf"
}
bed_case bambu "$B" "Bambu Lab A1 mini 0.4 nozzle" "Textured PEI Plate"
bed_case bambu "$B" "Bambu Lab X1 Carbon 0.4 nozzle" "Textured PEI Plate"
bed_case bambu "$B" "Bambu Lab H2D 0.4 nozzle" "Textured PEI Plate"
bed_case bambu "$B" "Bambu Lab P1S 0.4 nozzle" "Textured PEI Plate"
bed_case orca "$O" "Snapmaker U1 (0.4 nozzle)" "High Temp Plate"
bed_case orca "$O" "Bambu Lab X1 Carbon 0.4 nozzle" "Cool Plate"
bed_case orca "$O" "Prusa MK4 0.4 nozzle" "High Temp Plate"
bed_case orca "$O" "Creality K1C 0.4 nozzle" "High Temp Plate"
echo "PASS: an STL takes the desktop app's plate type for the printer; no INT_MAX in the export (both engines)"

# --curr-bed-type: the official setting flag. A known plate wins over the
# printer's and over a plate's own (official apply order: plate, then the
# command line). An unknown plate is refused (-2), listing the plates.
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run cbt-$e "$bin" cube.stl --slice 1 --printer-preset "$X1C" --curr-bed-type "High Temp Plate" --outputdir cbt-$e/out
    [ "$(rc cbt-$e)" = 0 ] || { show cbt-$e; fail "$e: --curr-bed-type exit $(rc cbt-$e)"; }
    has_line cbt-$e/out/plate_1.gcode '; curr_bed_type = High Temp Plate' || fail "$e: --curr-bed-type not applied"
    for bad in "Glass Plate" "Default Plate"; do
        run cbtbad-$e "$bin" cube.stl --slice 1 --printer-preset "$X1C" --curr-bed-type "$bad" --outputdir cbtbad-$e/out
        [ "$(rc cbtbad-$e)" != 0 ] || fail "$e: --curr-bed-type $bad was taken"
        py '
import json, sys
d = json.load(open(sys.argv[1]))
assert d["return_code"] == -2, d
assert "Textured PEI Plate" in d["error_string"] and "Cool Plate" in d["error_string"], d["error_string"]
' cbtbad-$e/out/result.json
        rm -rf cbtbad-$e
    done
done
# The fixture with its plate set to the Cool Plate (the plate's own bed_type,
# bbs_3mf.cpp 4563-4567).
py '
import zipfile
with zipfile.ZipFile("base.3mf") as zin, zipfile.ZipFile("platebed.3mf", "w", zipfile.ZIP_DEFLATED) as zout:
    for item in zin.infolist():
        data = zin.read(item.filename)
        if item.filename == "Metadata/model_settings.config":
            data = data.replace(b"  <plate>\n", b"  <plate>\n    <metadata key=\"bed_type\" value=\"Cool Plate\"/>\n", 1)
        zout.writestr(item, data)
'
run platebed "$B" platebed.3mf --plate 1 -o platebed/out.gcode
[ "$(rc platebed)" = 0 ] || { show platebed; fail "a plate with its own bed type exit $(rc platebed)"; }
has_line platebed/out.gcode '; curr_bed_type = Cool Plate' || fail "the plate's own bed type was not used"
run cbtplate "$B" platebed.3mf --plate 1 --curr-bed-type "High Temp Plate" -o cbtplate/out.gcode
[ "$(rc cbtplate)" = 0 ] || { show cbtplate; fail "--curr-bed-type over a plate's bed type exit $(rc cbtplate)"; }
has_line cbtplate/out.gcode '; curr_bed_type = High Temp Plate' || fail "the command line did not win over the plate's bed type"
echo "PASS: --curr-bed-type applies over the printer's and the plate's plate type, and refuses unknown plates (both engines)"

# --export-settings writes the project's settings, not a plate's: the official
# action saves m_print_config before the plate loop lays a plate's own
# settings over it (BambuStudio.cpp 6366-6370 and 6902-6904; OrcaSlicer.cpp
# 5499-5503 and 5905-5907). Plate 1 of platebed-<engine>.3mf (made above) is
# sliced on its High Temp Plate; the file keeps the project's plate.
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    for n in 1 0; do
        run pset$n-$e "$bin" platebed-$e.3mf --slice $n --export-settings pset$n-$e.json --outputdir pset$n-$e/out
        [ "$(rc pset$n-$e)" = 0 ] || { show pset$n-$e; fail "$e: --slice $n --export-settings exit $(rc pset$n-$e)"; }
        has_line pset$n-$e/out/plate_1.gcode '; curr_bed_type = High Temp Plate' || fail "$e: --slice $n did not slice on the plate's own bed type"
        py '
import json, sys, zipfile
want = json.loads(zipfile.ZipFile(sys.argv[2]).read("Metadata/project_settings.config"))["curr_bed_type"]
got = json.load(open(sys.argv[1])).get("curr_bed_type")
assert want != "High Temp Plate" and got == want, ("exported plate type", got, "project", want)
' pset$n-$e.json platebed-$e.3mf
    done
done
echo "PASS: --export-settings with --slice writes the project's settings, not plate 1's (both engines)"

# A 3MF keeps its own filament_printable (H2D: a two-nozzle printer).
run h2d "$B" cube.stl --slice 1 --printer-preset "Bambu Lab H2D 0.4 nozzle" --outputdir h2d/out --export-3mf h2d.3mf
[ "$(rc h2d)" = 0 ] || { show h2d; fail "H2D STL exit $(rc h2d)"; }
py '
import json, zipfile
src = zipfile.ZipFile("h2d/out/h2d.3mf")
with zipfile.ZipFile("h2d-fp.3mf", "w", zipfile.ZIP_DEFLATED) as out:
    for item in src.infolist():
        data = src.read(item.filename)
        if item.filename == "Metadata/project_settings.config":
            d = json.loads(data); d["filament_printable"] = ["1"]; data = json.dumps(d, indent=4).encode()
        if item.filename.endswith(".gcode") or item.filename.endswith(".md5"):
            continue
        out.writestr(item, data)
'
run h2dfp "$B" h2d-fp.3mf --slice 1 --outputdir h2dfp/out --export-3mf again.3mf
[ "$(rc h2dfp)" = 0 ] || { show h2dfp; fail "H2D 3MF exit $(rc h2dfp)"; }
py '
import json, zipfile
d = json.loads(zipfile.ZipFile("h2dfp/out/again.3mf").read("Metadata/project_settings.config"))
assert d["filament_printable"] == ["1"], d["filament_printable"]
'
echo "PASS: a 3MF keeps its own filament_printable"

# The official flags parse with each engine's own definitions; flags that are
# not in this binary refuse by name, never "Unknown option".
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    "$bin" --help > help-$e.txt
    grep -q -- '--curr-bed-type values: ' help-$e.txt || fail "$e: --help lists no plate types"
    grep -q -- '--load-settings' help-$e.txt || fail "$e: --help lists no official flags"
    for args in "--cut 3" "--export-obj" "--no-such-flag 1"; do
        run refuse-$e "$bin" cube.stl $args
        [ "$(rc refuse-$e)" != 0 ] || fail "$e: $args was taken"
        flag=${args%% *}
        grep -q -- "$flag" refuse-$e/stderr || { show refuse-$e; fail "$e: the refusal of $args does not name it"; }
        grep -q 'Unknown option' refuse-$e/stderr && fail "$e: $args gave Unknown option"
        rm -rf refuse-$e
    done
    # --pipe exists on Linux only, as in the official command lines.
    if [ "$(uname -s)" != Linux ]; then
        run pipe-$e "$bin" cube.stl --pipe x
        grep -q -- '--pipe is a flag of' pipe-$e/stderr || { show pipe-$e; fail "$e: --pipe not refused by name"; }
    fi
done
run png-bambu "$B" cube.stl --export-png 1
grep -q -- '--export-png is a flag of' png-bambu/stderr || { show png-bambu; fail "bambu: --export-png not refused by name"; }
run png-orca "$O" cube.stl --export-png 1
grep -q 'works only in slicer_cli' png-orca/stderr || { show png-orca; fail "orca: --export-png does not name the binary that has it"; }
grep -q 'Also: --layer-height' help-bambu.txt || fail "--help does not list the flags shared with a setting"
echo "PASS: unsupported and foreign flags refuse by name (both engines)"

# Any print setting is a flag (m_extra_config), on both engines.
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run setting-$e "$bin" cube.stl --printer-preset "$X1C" --sparse-infill-density 15% --wall-loops 4 -o setting-$e/out.gcode
    [ "$(rc setting-$e)" = 0 ] || { show setting-$e; fail "$e: setting flags exit $(rc setting-$e)"; }
    tr -d '\r' < setting-$e/out.gcode > setting-$e/out.lf
    grep -q '^; sparse_infill_density = 15%$' setting-$e/out.lf || fail "$e: --sparse-infill-density not applied"
    grep -q '^; wall_loops = 4$' setting-$e/out.lf || fail "$e: --wall-loops not applied"
done
echo "PASS: print settings work as flags (both engines)"

# --engine-info is slicer-cli's own report; --info is the official model report.
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run info-$e "$bin" cube.stl --info
    [ "$(rc info-$e)" = 0 ] || { show info-$e; fail "$e: --info exit $(rc info-$e)"; }
    grep -q 'size_x = 20' info-$e/stdout || { show info-$e; fail "$e: --info printed no model size"; }
    test ! -e output.gcode || fail "$e: --info alone sliced"
done
echo "PASS: --info reports the model, and slices nothing (both engines)"

# Model files other than STL: OBJ, AMF, STEP, several at once, and a 3MF that
# holds only geometry (named presets apply; without them it is refused).
py '
v = [(x, y, z) for z in (0, 20) for y in (0, 20) for x in (0, 20)]
f = [(0,2,1),(1,2,3),(4,5,6),(5,7,6),(0,1,4),(1,5,4),(2,6,3),(3,6,7),(0,4,2),(2,4,6),(1,3,5),(3,7,5)]
with open("cube.obj", "w") as o:
    for p in v: o.write("v %g %g %g\n" % p)
    for a, b, c in f: o.write("f %d %d %d\n" % (a + 1, b + 1, c + 1))
with open("cube.amf", "w") as o:
    o.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<amf unit=\"millimeter\">\n<object id=\"0\"><mesh><vertices>\n")
    for p in v: o.write("<vertex><coordinates><x>%g</x><y>%g</y><z>%g</z></coordinates></vertex>\n" % p)
    o.write("</vertices><volume>\n")
    for a, b, c in f: o.write("<triangle><v1>%d</v1><v2>%d</v2><v3>%d</v3></triangle>\n" % (a, b, c))
    o.write("</volume></mesh></object>\n</amf>\n")
import zipfile
model = ["<?xml version=\"1.0\" encoding=\"UTF-8\"?>",
         "<model unit=\"millimeter\" xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\">",
         "<resources><object id=\"1\" type=\"model\"><mesh><vertices>"]
model += ["<vertex x=\"%g\" y=\"%g\" z=\"%g\"/>" % p for p in v]
model += ["</vertices><triangles>"]
model += ["<triangle v1=\"%d\" v2=\"%d\" v3=\"%d\"/>" % t for t in f]
model += ["</triangles></mesh></object></resources><build><item objectid=\"1\"/></build></model>"]
with zipfile.ZipFile("geo.3mf", "w", zipfile.ZIP_DEFLATED) as z:
    z.writestr("[Content_Types].xml", "<?xml version=\"1.0\" encoding=\"UTF-8\"?><Types xmlns=\"http://schemas.openxmlformats.org/package/2006/content-types\"><Default Extension=\"rels\" ContentType=\"application/vnd.openxmlformats-package.relationships+xml\"/><Default Extension=\"model\" ContentType=\"application/vnd.ms-package.3dmanufacturing-3dmodel+xml\"/></Types>")
    z.writestr("_rels/.rels", "<?xml version=\"1.0\" encoding=\"UTF-8\"?><Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\"><Relationship Target=\"/3D/3dmodel.model\" Id=\"rel0\" Type=\"http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel\"/></Relationships>")
    z.writestr("3D/3dmodel.model", "\n".join(model))
'
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    for f in cube.obj cube.amf geo.3mf; do
        n="kind-$e-${f%%.*}-${f##*.}"
        run "$n" "$bin" "$f" --slice 1 --printer-preset "$A1M" --outputdir "$n/out"
        [ "$(rc "$n")" = 0 ] || { show "$n"; fail "$e: $f exit $(rc "$n")"; }
        py '
import json, sys
d = json.load(open(sys.argv[1]))
o = d["sliced_plates"][0]["objects"][0]["bbox"]
# Placed by the desktop rule: centred on the 180 mm A1 mini bed.
assert abs(o["x"] + o["width"] / 2 - 90) < 1 and abs(o["y"] + o["depth"] / 2 - 90) < 1, o
' "$n/out/result.json"
    done
    run multi-$e "$bin" cube.stl cube.obj --slice 1 --printer-preset "$A1M" --outputdir multi-$e/out
    [ "$(rc multi-$e)" = 0 ] || { show multi-$e; fail "$e: two model files exit $(rc multi-$e)"; }
    py '
import json, sys
d = json.load(open(sys.argv[1]))
assert len(d["sliced_plates"][0]["objects"]) == 2, d["sliced_plates"][0]["objects"]
' multi-$e/out/result.json
    # --arrange 2 (any value but 0 and 1) is automatic, as no --arrange: model
    # files with several objects are arranged apart ("0-disable, 1-enable,
    # others-auto", BambuStudio.cpp 5066-5081; OrcaSlicer.cpp 4327-4342).
    for a in none 2; do
        flag=""; [ $a = 2 ] && flag="--arrange 2"
        run arr$a-$e "$bin" cube.stl cube.stl --slice 1 $flag --printer-preset "$A1M" --outputdir arr$a-$e/out
        [ "$(rc arr$a-$e)" = 0 ] || { show arr$a-$e; fail "$e: two cubes with --arrange $a exit $(rc arr$a-$e)"; }
        py '
import json, sys
o = json.load(open(sys.argv[1]))["sliced_plates"][0]["objects"]
assert len(o) == 2, o
a, b = o[0]["bbox"], o[1]["bbox"]
apart = a["x"] + a["width"] <= b["x"] or b["x"] + b["width"] <= a["x"] or a["y"] + a["depth"] <= b["y"] or b["y"] + b["depth"] <= a["y"]
assert apart, (sys.argv[2], a, b)
' arr$a-$e/out/result.json $a
    done
    # A geometry-only 3MF (no project settings) whose model_settings.config
    # still lists two plates is one plate: --slice 0 slices it once.
    py '
import zipfile
src = zipfile.ZipFile("geo.3mf")
with zipfile.ZipFile("geo-plates.3mf", "w", zipfile.ZIP_DEFLATED) as z:
    for item in src.infolist():
        z.writestr(item, src.read(item.filename))
    z.writestr("Metadata/model_settings.config",
               "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<config>\n<plate>\n<metadata key=\"plater_id\" value=\"1\"/>\n</plate>\n"
               "<plate>\n<metadata key=\"plater_id\" value=\"2\"/>\n</plate>\n</config>\n")
'
    run geop-$e "$bin" geo-plates.3mf --slice 0 --printer-preset "$A1M" --outputdir geop-$e/out
    [ "$(rc geop-$e)" = 0 ] || { show geop-$e; fail "$e: a geometry-only 3MF with plate tags exit $(rc geop-$e)"; }
    [ -s geop-$e/out/plate_1.gcode ] && [ ! -e geop-$e/out/plate_2.gcode ] || fail "$e: a geometry-only 3MF was sliced as two plates"
    run geo-$e "$bin" geo.3mf --slice 1 --outputdir geo-$e/out
    # The exit status of -2 reads differently per shell and system; result.json is the record.
    [ "$(rc geo-$e)" != 0 ] || { show geo-$e; fail "$e: a geometry-only 3MF without a printer was sliced"; }
    py '
import json, sys; d = json.load(open(sys.argv[1]))
assert d["return_code"] == -2, d
' geo-$e/out/result.json
    grep -q -- '--printer-preset' geo-$e/stderr || fail "$e: the geometry-only refusal does not name --printer-preset"
    # Model actions only (no --slice) need no printer, as for an STL.
    run geox-$e "$bin" geo.3mf --export-stl --outputdir geox-$e/out
    [ "$(rc geox-$e)" = 0 ] && [ "$(ls geox-$e/out/stl | wc -l | tr -d ' ')" = 1 ] || { show geox-$e; fail "$e: --export-stl on a geometry-only 3MF without a printer exit $(rc geox-$e)"; }
    run geoi-$e "$bin" geo.3mf --info
    [ "$(rc geoi-$e)" = 0 ] || { show geoi-$e; fail "$e: --info on a geometry-only 3MF without a printer exit $(rc geoi-$e)"; }
done
echo "PASS: OBJ, AMF, a geometry-only 3MF and several files at once load and are placed; --arrange 2 is automatic; model actions on a geometry-only 3MF need no printer (both engines)"

# A switch takes no separate value, as on the official command lines
# (DynamicConfig::read_cli, Config.cpp 1719-1726): --normative-check=0 turns
# it off; in "--normative-check 0" the 0 is read as a model file.
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run boolon-$e "$bin" cube.stl --normative-check=0 --slice 1 --printer-preset "$A1M" --outputdir boolon-$e/out
    [ "$(rc boolon-$e)" = 0 ] || { show boolon-$e; fail "$e: --normative-check=0 exit $(rc boolon-$e)"; }
    run boolsep-$e "$bin" cube.stl --normative-check 0 --slice 1 --printer-preset "$A1M" --outputdir boolsep-$e/out
    [ "$(rc boolsep-$e)" != 0 ] || fail "$e: --normative-check 0 took 0 as the switch value"
done
echo "PASS: a switch takes its value only after = (both engines)"

# --engine-info names a model file's kind and, with --printer-preset, the
# printer and the binary that has it.
"$B" --engine-info cube.obj > ei-obj.json
"$B" --engine-info cube.obj --printer-preset "$A1M" > ei-a1m.json
"$B" --engine-info cube.stl --printer-preset "Snapmaker U1 (0.4 nozzle)" > ei-u1.json
py '
import json
d = json.load(open("ei-obj.json"));  assert d["kind"] == "obj", d
d = json.load(open("ei-a1m.json"));  assert d["kind"] == "obj" and d["printer_model"] == "Bambu Lab A1 mini" and "slicer_cli" in d["fits"], d
d = json.load(open("ei-u1.json"));   assert d["printer_model"] == "Snapmaker U1" and d["fits"] == ["slicer_cli-orcaslicer"] and d["recommended"] == "slicer_cli-orcaslicer", d
'
echo "PASS: --engine-info reports model kinds and the printer of --printer-preset"

# Units: a model in meters is reported; --convert-unit scales it.
py '
v = [(x, y, z) for z in (0, 0.02) for y in (0, 0.02) for x in (0, 0.02)]
f = [(0,2,1),(1,2,3),(4,5,6),(5,7,6),(0,1,4),(1,5,4),(2,6,3),(3,6,7),(0,4,2),(2,4,6),(1,3,5),(3,7,5)]
with open("meters.stl", "w") as o:
    o.write("solid t\n")
    for a, b, c in f:
        o.write("facet normal 0 0 0\nouter loop\n")
        for i in (a, b, c): o.write("vertex %g %g %g\n" % v[i])
        o.write("endloop\nendfacet\n")
    o.write("endsolid t\n")
'
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run m-$e "$bin" meters.stl --info
    events m-$e/stdout model_warning | grep ModelUnitsLookWrong >/dev/null || { show m-$e; fail "$e: no units event for a model in meters"; }
    run mc-$e "$bin" meters.stl --convert-unit --info
    grep -q 'size_x = 20' mc-$e/stdout || { show mc-$e; fail "$e: --convert-unit did not scale meters to mm"; }
done
echo "PASS: model units are reported, and --convert-unit converts (both engines)"

# Transforms, in command-line order: --scale, --rotate, --orient, --assemble.
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run scale-$e "$bin" cube.stl --scale 2 --info
    grep -q 'size_x = 40' scale-$e/stdout || { show scale-$e; fail "$e: --scale 2 did not double the cube"; }
    run rot-$e "$bin" wide.stl --rotate-x 90 --info
    grep -q 'size_z = 230' rot-$e/stdout || { show rot-$e; fail "$e: --rotate-x 90 did not stand the plate up"; }
    run asm-$e "$bin" cube.stl cube.obj --assemble --slice 1 --arrange 1 --printer-preset "$A1M" --outputdir asm-$e/out
    [ "$(rc asm-$e)" = 0 ] || { show asm-$e; fail "$e: --assemble exit $(rc asm-$e)"; }
    py '
import json, sys
d = json.load(open(sys.argv[1]))
assert len(d["sliced_plates"][0]["objects"]) == 1, d["sliced_plates"][0]["objects"]
' asm-$e/out/result.json
done
echo "PASS: --scale, --rotate-x and --assemble transform the model (both engines)"

# Actions: --export-stl writes each object; --export-settings writes the settings.
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run stl-$e "$bin" cube.stl cube.obj --export-stl --outputdir stl-$e/out
    [ "$(rc stl-$e)" = 0 ] || { show stl-$e; fail "$e: --export-stl exit $(rc stl-$e)"; }
    [ "$(ls stl-$e/out/stl | wc -l | tr -d ' ')" = 2 ] || fail "$e: --export-stl did not write two STLs"
    run set-$e "$bin" cube.stl --printer-preset "$A1M" --export-settings set-$e.json
    [ "$(rc set-$e)" = 0 ] || { show set-$e; fail "$e: --export-settings exit $(rc set-$e)"; }
    py '
import json, sys; d = json.load(open(sys.argv[1]))
assert d.get("printer_model") == "Bambu Lab A1 mini", d.get("printer_model")
' set-$e.json
done
echo "PASS: --export-stl and --export-settings write their files (both engines)"

# An STL target that exists but cannot be written keeps its old content, and
# the engine's store_stl still reports success: the run fails with the
# official export failure (CLI_EXPORT_STL_ERROR, -11) instead of reporting the
# stale file. Not as root, which writes read-only files.
if [ "$(id -u)" != 0 ]; then
    for e in bambu orca; do
        bin=$B; [ $e = orca ] && bin=$O
        f=$(ls stl-$e/out/stl/*.stl | head -n 1)
        echo stale > "$f"; chmod 444 "$f"
        run stlro-$e "$bin" cube.stl cube.obj --export-stl --outputdir stl-$e/out
        chmod 644 "$f"
        [ "$(rc stlro-$e)" != 0 ] && [ "$(head -c 5 "$f")" = stale ] || { show stlro-$e; fail "$e: --export-stl onto a read-only target exit $(rc stlro-$e)"; }
        if grep '^\[\[SLICER_EVENT\]\]' stlro-$e/stdout | grep StlExported | grep -F "$(basename "$f")" >/dev/null; then
            fail "$e: --export-stl reported the read-only target as exported"
        fi
        grep -q "Writing .*$(basename "$f") failed" stlro-$e/stderr || { show stlro-$e; fail "$e: --export-stl onto a read-only target did not name the file"; }
    done
    echo "PASS: --export-stl onto a target it cannot write fails (both engines)"
fi

# A full disk: the STL writer checks none of its writes, so the file comes
# out short or empty. The run fails with the export failure (-11) and leaves
# no STL. Linux with user namespaces only: a 64 KB tmpfs, filled first.
if [ "$(uname -s)" = Linux ] && unshare -rm true 2>/dev/null; then
    for e in bambu orca; do
        bin=$B; [ $e = orca ] && bin=$O
        mkdir -p full-$e/mnt
        unshare -rm bash -c '
            mount -t tmpfs -o size=64k tmpfs "$1/mnt" || exit 9
            mkdir -p "$1/mnt/out/stl"
            dd if=/dev/zero of="$1/mnt/fill" bs=1k count=1000 2>/dev/null
            "$2" cube.stl --export-stl --outputdir "$1/mnt/out" > "$1/stdout" 2> "$1/stderr"
            echo $? > "$1/rc"
            ls "$1/mnt/out/stl" > "$1/stls"
        ' _ full-$e "$bin" || true
        [ -s full-$e/rc ] && [ "$(rc full-$e)" != 0 ] && [ ! -s full-$e/stls ] || { show full-$e; fail "$e: --export-stl onto a full disk exit $(cat full-$e/rc 2>/dev/null), left: $(cat full-$e/stls 2>/dev/null)"; }
        if grep '^\[\[SLICER_EVENT\]\]' full-$e/stdout | grep StlExported >/dev/null; then
            fail "$e: --export-stl reported an STL written to a full disk"
        fi
    done
    echo "PASS: --export-stl onto a full disk fails and leaves no STL (both engines, Linux)"
fi

# --skip-objects leaves the named object out; skipping all of them is -60.
run skipall "$B" "$FIXTURE" --slice 1 --skip-objects 999999 --outputdir skipall/out
[ "$(rc skipall)" = 0 ] || { show skipall; fail "--skip-objects of an id the file lacks exit $(rc skipall)"; }
ID=$(py '
import re, zipfile
m = zipfile.ZipFile("base.3mf").read("Metadata/model_settings.config").decode()
print(re.search(r"key=\"identify_id\" value=\"([0-9]+)\"", m).group(1))
')
run skipone "$B" "$FIXTURE" --slice 1 --skip-objects "$ID" --outputdir skipone/out
py '
import json; d = json.load(open("skipone/out/result.json"))
assert d["return_code"] == -60, d
'
echo "PASS: --skip-objects skips by the file's object id; nothing left is -60"

# --skip-objects belongs to the slice action (BambuStudio.cpp 6541-6581;
# OrcaSlicer.cpp 5645-5720): a run of model actions only skips nothing, so
# --export-stl writes every object even when all are named.
py '
import json
json.dump({"plates": [{"plate_name": "p", "need_arrange": True, "objects": [{"path": "cube.stl", "count": 2, "filaments": [1]}]}]}, open("skip2.json", "w"))
'
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run skp-$e "$bin" --load-assemble-list skip2.json --slice 1 --printer-preset "Bambu Lab X1 Carbon 0.4 nozzle" \
        --outputdir skp-$e/out --export-3mf skip2.3mf
    [ "$(rc skp-$e)" = 0 ] || { show skp-$e; fail "$e: two-cube project exit $(rc skp-$e)"; }
    ids=$(py '
import re, sys, zipfile
t = zipfile.ZipFile(sys.argv[1]).read("Metadata/model_settings.config").decode()
print(",".join(re.findall(r"key=\"identify_id\" value=\"([0-9]+)\"", t)))
' skp-$e/out/skip2.3mf)
    [ -n "$ids" ] || fail "$e: no object ids in the two-cube project"
    run skpa-$e "$bin" skp-$e/out/skip2.3mf --export-stl --skip-objects "$ids" --outputdir skpa-$e/out
    [ "$(rc skpa-$e)" = 0 ] && [ "$(ls skpa-$e/out/stl | wc -l | tr -d ' ')" = 2 ] || { show skpa-$e; fail "$e: --export-stl with every object in --skip-objects and no --slice exit $(rc skpa-$e)"; }
done
echo "PASS: a run of model actions only skips no object (both engines)"

# --mtcpp: a triangle limit below the cube's 12 is -59.
run mtcpp "$B" "$FIXTURE" --slice 1 --mtcpp 5 --outputdir mtcpp/out
py '
import json; d = json.load(open("mtcpp/out/result.json"))
assert d["return_code"] == -59, d
'
echo "PASS: --mtcpp refuses a plate over the triangle limit"

# --repetitions N: the plate N times in all (N-1 copies), as many as fit.
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run rep-$e "$bin" cube.stl --repetitions 3 --slice 1 --printer-preset "$A1M" --outputdir rep-$e/out
    [ "$(rc rep-$e)" = 0 ] || { show rep-$e; fail "$e: --repetitions exit $(rc rep-$e)"; }
    py '
import json, sys
d = json.load(open(sys.argv[1]))
assert len(d["sliced_plates"][0]["objects"]) == 3, d["sliced_plates"][0]["objects"]
' rep-$e/out/result.json
done
echo "PASS: --repetitions copies the plate (both engines)"

# A project 3MF of each engine's own (the Bambu fixture is too new for the
# OrcaSlicer engine).
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run proj-$e "$bin" cube.stl --slice 1 --printer-preset "$A1M" --outputdir proj-$e/out --export-3mf proj.3mf
    [ "$(rc proj-$e)" = 0 ] || { show proj-$e; fail "$e: project export exit $(rc proj-$e)"; }
    cp proj-$e/out/proj.3mf proj-$e.3mf
    # A bare output file name writes into the current folder.
    run bare-$e "$bin" proj-$e.3mf --plate 1 -o bare-$e.gcode
    [ "$(rc bare-$e)" = 0 ] && [ -s bare-$e.gcode ] || { show bare-$e; fail "$e: -o with a bare file name"; }
done

# --export-slicedata writes the plate's slicing; --load-slicedata slices from it.
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run sdx-$e "$bin" proj-$e.3mf --slice 1 --export-slicedata sd-$e --outputdir sdx-$e/out
    [ "$(rc sdx-$e)" = 0 ] || { show sdx-$e; fail "$e: --export-slicedata exit $(rc sdx-$e)"; }
    [ -n "$(ls sd-$e/1 2>/dev/null)" ] || fail "$e: --export-slicedata wrote nothing under sd-$e/1"
    run sdl-$e "$bin" proj-$e.3mf --slice 1 --load-slicedata sd-$e --outputdir sdl-$e/out
    [ "$(rc sdl-$e)" = 0 ] || { show sdl-$e; fail "$e: --load-slicedata exit $(rc sdl-$e)"; }
    # OrcaSlicer's loader at its pin cannot read what its own exporter writes
    # (Print::load_cached_data, -57), so that engine slices the plate normally
    # and says so, as its own command line falls back (OrcaSlicer.cpp 6067-6090).
    want=SliceDataLoaded; [ $e = orca ] && want=SliceDataNotLoaded
    events sdl-$e/stdout model_loaded | grep $want >/dev/null || { show sdl-$e; fail "$e: --load-slicedata: no $want"; }
    run sdr-$e "$bin" proj-$e.3mf --slice 1 --load-slicedata sd-$e --repetitions 2 --outputdir sdr-$e/out
    py '
import json, sys; d = json.load(open(sys.argv[1]))
assert d["return_code"] == -2, d
' sdr-$e/out/result.json
done
echo "PASS: --export-slicedata and --load-slicedata round-trip; with --repetitions it is refused (both engines)"

# --downward-check with --downward-settings: the printers the plate also fits.
py '
import json
for name, size in (("Roomy Test Printer", 200), ("Tiny Test Printer", 10)):
    json.dump({"type": "machine", "from": "system", "name": name, "instantiation": "true",
               "printable_area": ["0x0", "%dx0" % size, "%dx%d" % (size, size), "0x%d" % size],
               "printable_height": str(size)},
              open(name.split()[0].lower() + ".json", "w"), indent=4)
'
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run dw-$e "$bin" proj-$e.3mf --slice 1 --downward-check --downward-settings roomy.json --downward-settings tiny.json --outputdir dw-$e/out
    [ "$(rc dw-$e)" = 0 ] || { show dw-$e; fail "$e: --downward-check exit $(rc dw-$e)"; }
    py '
import json, sys; d = json.load(open(sys.argv[1]))
assert d.get("downward_compatible_machine") == ["Roomy Test Printer"], d.get("downward_compatible_machine")
' dw-$e/out/result.json
done
run dwstl "$B" cube.stl --slice 1 --printer-preset "$A1M" --downward-check --outputdir dwstl/out
[ "$(rc dwstl)" != 0 ] || fail "--downward-check on an STL was taken"
echo "PASS: --downward-check lists the printers the plate fits (both engines)"

# Without those flags result.json gains no compatibility keys.
for f in sdx-bambu/out/result.json sdx-orca/out/result.json; do
    py '
import json, sys; d = json.load(open(sys.argv[1]))
for k in ("downward_compatible_machine", "upward_compatible_machine", "upward_compatibility_taint"):
    assert k not in d, k
' "$f"
done
echo "PASS: result.json gains no keys unless the flags ask for them"

# --load-assemble-list: plates built from a JSON list. Plate 1 is arranged
# (two copies of the cube); plate 2 keeps the list's positions, merges its two
# copies into one object (assemble_index 1) and has its own settings.
py '
import json
json.dump({"plates": [
    {"plate_name": "arranged", "need_arrange": True,
     "objects": [{"path": "cube.stl", "count": 2, "filaments": [1]}]},
    {"plate_name": "placed", "need_arrange": False,
     "plate_params": {"sparse_infill_density": "25%"},
     "objects": [{"path": "cube.stl", "count": 2, "filaments": [1], "assemble_index": [1],
                  "pos_x": [60, 30], "pos_y": [60], "print_params": {"wall_loops": "3"},
                  "height_ranges": [{"min_z": 0, "max_z": 5, "range_params": {"sparse_infill_density": "50%"}}]}],
     "assembled_params": [{"assemble_index": 1, "print_params": {"wall_loops": "4"}}]}]},
    open("asm.json", "w"))
json.dump({"plates": [{"plate_name": "p", "need_arrange": True,
                       "objects": [{"path": "missing.stl", "count": 1, "filaments": [1]}]}]},
          open("asm-missing.json", "w"))
json.dump({"plates": [{"plate_name": "p", "need_arrange": True,
                       "objects": [{"path": "cube.stl", "count": 2, "filaments": [1, 1, 1]}]}]},
          open("asm-bad.json", "w"))
'
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    # --slice 1: an assemble list slices every plate, as the official does.
    run asml-$e "$bin" --load-assemble-list asm.json --slice 1 --printer-preset "$A1M" --outputdir asml-$e/out --export-3mf asm.3mf
    [ "$(rc asml-$e)" = 0 ] || { show asml-$e; fail "$e: --load-assemble-list exit $(rc asml-$e)"; }
    [ -s asml-$e/out/plate_1.gcode ] && [ -s asml-$e/out/plate_2.gcode ] || fail "$e: --load-assemble-list did not slice both plates"
    events asml-$e/stdout config_normalized | grep AssembleListSlicesEveryPlate >/dev/null || fail "$e: no note that every plate is sliced"
    has_line asml-$e/out/plate_2.gcode '; sparse_infill_density = 25%' || fail "$e: plate 2's own settings were not applied"
    py '
import json, re, sys, zipfile
d = json.load(open(sys.argv[1]))
plates = d["sliced_plates"]
assert [p["id"] for p in plates] == [1, 2], plates
assert len(plates[0]["objects"]) == 2, plates[0]["objects"]
assert [o["name"] for o in plates[1]["objects"]] == ["assemble_1"], plates[1]["objects"]
box = plates[1]["objects"][0]["bbox"]
# The copies stay where the list puts them: x 60..110, y 60..140 on plate 2,
# which sits at (180 * 1.2, 0) in the scene result.json states.
assert abs(box["x"] - (60 + 216)) < 0.5 and abs(box["width"] - 50) < 0.5, box
assert abs(box["y"] - 60) < 0.5 and abs(box["depth"] - 80) < 0.5, box
z = zipfile.ZipFile(sys.argv[2])
model = z.read("Metadata/model_settings.config").decode()
assert len(re.findall(r"<plate>", model)) == 2, model
assert "\"placed\"" in model and "\"arranged\"" in model, "plate names missing"
' asml-$e/out/result.json asml-$e/out/asm.3mf
    # With a model file: -2. A part the list names that does not exist: -3.
    # A per-copy list of the wrong length: -5.
    for c in "files:cube.stl --load-assemble-list asm.json:-2" "missing:--load-assemble-list asm-missing.json:-3" \
             "bad:--load-assemble-list asm-bad.json:-5"; do
        n=${c%%:*}; rest=${c#*:}; args=${rest%:*}; want=${rest##*:}
        run asm$n-$e "$bin" $args --slice 0 --printer-preset "$A1M" --outputdir asm$n-$e/out
        py '
import json, sys; d = json.load(open(sys.argv[1]))
assert d["return_code"] == int(sys.argv[2]), d
' asm$n-$e/out/result.json "$want"
    done
done
echo "PASS: --load-assemble-list builds, arranges and slices every plate of its list (both engines)"

# Coloured OBJs in an assemble list (slicer_cli: their colours become the
# filaments). The second OBJ repeats 5 of the first's 10 colours and adds 5:
# a repeated colour keeps its filament and adds none, so the palette holds
# fewer colours than the capacity check counts; the nearest-colour fallback
# compares only the colours there. Every face's filament is one of the
# project's filaments.
py '
import json
def cubes(path, colours):
    lines, n = [], 0
    for k, (r, g, b) in enumerate(colours):
        x0 = (k % 5) * 12; y0 = (k // 5) * 12
        for z in (0, 10):
            for y in (0, 10):
                for x in (0, 10):
                    lines.append("v %g %g %g %g %g %g" % (x0 + x, y0 + y, z, r, g, b))
        for a, b2, c in ((1,3,2),(2,3,4),(5,6,7),(6,8,7),(1,2,5),(2,6,5),(3,7,4),(4,7,8),(1,5,3),(3,5,7),(2,4,6),(4,8,6)):
            lines.append("f %d %d %d" % (n + a, n + b2, n + c))
        n += 8
    open(path, "w").write("\n".join(lines) + "\n")
V = (0, 0.2, 0.6, 1)
A = [(0, g, b) for g in V for b in V][:10]
B = A[:5] + [(0.2, g, 0) for g in V] + [(0.2, 0, 1)]
cubes("pal-a.obj", A); cubes("pal-b.obj", B)
json.dump({"plates": [{"plate_name": "p", "need_arrange": True,
            "objects": [{"path": "pal-a.obj", "count": 1, "filaments": [1]},
                        {"path": "pal-b.obj", "count": 1, "filaments": [1]}]}]}, open("pal-list.json", "w"))
'
run palset "$B" pal-a.obj --printer-preset "Bambu Lab X1 Carbon 0.4 nozzle" --export-settings pal-all.json
py '
import json
d = json.load(open("pal-all.json"))
f = {k: (v[:1] if isinstance(v, list) else v) for k, v in d.items() if k.startswith("filament_") and k != "filament_settings_id"}
f.update({"type": "filament", "from": "system", "name": "Test Filament A", "filament_id": "GFL99"})
json.dump(f, open("pal-fil.json", "w"), indent=1)
'
run pal "$B" --load-assemble-list pal-list.json --slice 1 --printer-preset "Bambu Lab X1 Carbon 0.4 nozzle" \
    --load-filaments pal-fil.json --outputdir pal/out --export-3mf pal.3mf
[ "$(rc pal)" = 0 ] || { show pal; fail "coloured OBJs with repeated colours exit $(rc pal)"; }
py '
import json, re, sys, zipfile
z = zipfile.ZipFile(sys.argv[1])
d = json.loads(z.read("Metadata/project_settings.config"))
n = len(d["filament_colour"])
assert 10 < n <= 16, d["filament_colour"]
used = set()
for name in z.namelist():
    if name.startswith("3D/") and name.endswith(".model"):
        text = z.read(name).decode(errors="replace")
        used |= {int(v) for v in re.findall(r"<metadata key=\"extruder\" value=\"(\d+)\"", text)}
model = z.read("Metadata/model_settings.config").decode(errors="replace")
used |= {int(v) for v in re.findall(r"key=\"extruder\" value=\"(\d+)\"", model)}
assert all(1 <= u <= n for u in used), (sorted(used), n)
' pal/out/pal.3mf
echo "PASS: coloured OBJs that repeat colours map every colour to a project filament (slicer_cli)"

# --downward-check checks every plate of the project, not only the sliced
# one: the assemble export has a 20 mm plate 1 and a 50 x 80 mm plate 2, so
# a 60 mm printer fails on plate 2 even when only plate 1 is sliced.
py '
import json
json.dump({"type": "machine", "from": "system", "name": "Mid Test Printer", "instantiation": "true",
           "printable_area": ["0x0", "60x0", "60x60", "0x60"], "printable_height": "60"},
          open("mid.json", "w"), indent=4)
'
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    cp asml-$e/out/asm.3mf asm-$e.3mf
    run dwall-$e "$bin" asm-$e.3mf --slice 1 --downward-check --downward-settings roomy.json --downward-settings mid.json --outputdir dwall-$e/out
    [ "$(rc dwall-$e)" = 0 ] || { show dwall-$e; fail "$e: --downward-check on a two-plate project exit $(rc dwall-$e)"; }
    py '
import json, sys; d = json.load(open(sys.argv[1]))
assert d.get("downward_compatible_machine") == ["Roomy Test Printer"], d.get("downward_compatible_machine")
' dwall-$e/out/result.json
done
echo "PASS: --downward-check checks every plate of the project (both engines)"

# A plate 2 with the prime tower and two filaments: its objects and its tower
# are measured at plate 2's own origin (BambuStudio.cpp 4486-4489;
# OrcaSlicer.cpp 3875-3878), so the 200 mm printer fits every plate. The
# merged settings decide the other plates' print sequence: with a process
# that prints by object, plate 2 (two objects) has no tower, so the 60 mm
# printer fits too (BambuStudio.cpp 4785). With --slice 0 every plate is in
# the run's one model (the official loads the whole project once,
# BambuStudio.cpp 1889), so no plate is read from the project again.
py '
import json
json.dump({"plates": [
    {"plate_name": "one", "need_arrange": True, "objects": [{"path": "cube.stl", "count": 1, "filaments": [1]}]},
    {"plate_name": "two", "need_arrange": True, "objects": [{"path": "cube.stl", "count": 2, "filaments": [1, 2]}]}]},
    open("asm-tower.json", "w"))
json.dump({"type": "process", "from": "user", "name": "By Object Test Process", "inherits": "",
           "print_sequence": "by object", "compatible_printers": ["Bambu Lab A1 mini 0.4 nozzle"]},
          open("seq-process.json", "w"), indent=4)
# The same two printers with extruder clearances, which a by-object plate
# needs on the Bambu Studio build (BambuStudio.cpp 4819-4836).
for name, size in (("Roomy Test Printer", 200), ("Mid Test Printer", 60)):
    json.dump({"type": "machine", "from": "system", "name": name, "instantiation": "true",
               "printable_area": ["0x0", "%dx0" % size, "%dx%d" % (size, size), "0x%d" % size],
               "printable_height": str(size), "extruder_clearance_max_radius": "1",
               "extruder_clearance_height_to_rod": "1000", "extruder_clearance_height_to_lid": "1000",
               "extruder_clearance_dist_to_rod": "1"},
              open(name.split()[0].lower() + "-clear.json", "w"), indent=4)
'
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run dwtp-$e "$bin" --load-assemble-list asm-tower.json --slice 1 --printer-preset "$A1M" \
        --filament-preset "Bambu PLA Basic @BBL A1M" --filament-preset "Bambu PLA Matte @BBL A1M" \
        --enable-prime-tower --outputdir dwtp-$e/out --export-3mf tower2.3mf
    [ "$(rc dwtp-$e)" = 0 ] || { show dwtp-$e; fail "$e: two-plate tower project exit $(rc dwtp-$e)"; }
    # Two colours, so the Bambu Studio build keeps the tower for the plate
    # (it turns the tower off when every filament has the same colour).
    py '
import json, sys, zipfile
src = sys.argv[1]; dst = sys.argv[2]
zin = zipfile.ZipFile(src); zout = zipfile.ZipFile(dst, "w", zipfile.ZIP_DEFLATED)
for item in zin.infolist():
    data = zin.read(item.filename)
    if item.filename == "Metadata/project_settings.config":
        d = json.loads(data); d["filament_colour"] = ["#FF0000", "#0000FF"]; data = json.dumps(d, indent=4).encode()
    zout.writestr(item, data)
zout.close()
' dwtp-$e/out/tower2.3mf tower2-$e.3mf
    run dwt-$e "$bin" tower2-$e.3mf --slice 1 --downward-check --downward-settings roomy.json \
        --downward-settings mid.json --outputdir dwt-$e/out
    [ "$(rc dwt-$e)" = 0 ] || { show dwt-$e; fail "$e: --downward-check on the tower project exit $(rc dwt-$e)"; }
    py '
import json, sys; d = json.load(open(sys.argv[1]))
assert d.get("downward_compatible_machine") == ["Roomy Test Printer"], d.get("downward_compatible_machine")
' dwt-$e/out/result.json
    run dwts-$e "$bin" tower2-$e.3mf --slice 1 --downward-check --downward-settings roomy-clear.json \
        --downward-settings mid-clear.json --load-settings seq-process.json --outputdir dwts-$e/out
    [ "$(rc dwts-$e)" = 0 ] || { show dwts-$e; fail "$e: --downward-check with a by-object process exit $(rc dwts-$e)"; }
    py '
import json, sys; d = json.load(open(sys.argv[1]))
assert d.get("downward_compatible_machine") == ["Roomy Test Printer", "Mid Test Printer"], d.get("downward_compatible_machine")
' dwts-$e/out/result.json
    run dwt0-$e "$bin" tower2-$e.3mf --slice 0 --downward-check --downward-settings roomy.json --verbose \
        --outputdir dwt0-$e/out
    [ "$(rc dwt0-$e)" = 0 ] || { show dwt0-$e; fail "$e: --downward-check with --slice 0 exit $(rc dwt0-$e)"; }
    reads=$(grep -c '^--downward-check: read plate' dwt0-$e/stdout || true)
    [ "$reads" = 0 ] || fail "$e: --slice 0 read the other plates $reads times (want 0)"
    py '
import json, sys; d = json.load(open(sys.argv[1]))
assert d.get("downward_compatible_machine") == ["Roomy Test Printer"], d.get("downward_compatible_machine")
assert [p["id"] for p in d["sliced_plates"]] == [1, 2], d["sliced_plates"]
' dwt0-$e/out/result.json
done
echo "PASS: --downward-check measures each plate at its own origin, with the merged print sequence, once per run (both engines)"

# --mtcpp counts every instance inside the bed: three copies of the 12-triangle
# cube are 36 triangles.
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run mtr-$e "$bin" cube.stl --repetitions 3 --slice 1 --mtcpp 30 --printer-preset "$A1M" --outputdir mtr-$e/out
    py '
import json, sys; d = json.load(open(sys.argv[1]))
assert d["return_code"] == -59, d
' mtr-$e/out/result.json
    run mtok-$e "$bin" cube.stl --repetitions 3 --slice 1 --mtcpp 40 --printer-preset "$A1M" --outputdir mtok-$e/out
    [ "$(rc mtok-$e)" = 0 ] || { show mtok-$e; fail "$e: --mtcpp 40 on 36 triangles exit $(rc mtok-$e)"; }
done
echo "PASS: --mtcpp counts the triangles of every instance (both engines)"

# An instance the user made unprintable still counts (only --skip-objects
# instances do not; BambuStudio.cpp 6545-6575, OrcaSlicer.cpp 5657-5690):
# plate 1 of the assemble export holds two cubes, one set unprintable here.
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    py '
import sys, zipfile
src, dst = sys.argv[1], sys.argv[2]
zin = zipfile.ZipFile(src)
zout = zipfile.ZipFile(dst, "w", zipfile.ZIP_DEFLATED)
for item in zin.infolist():
    data = zin.read(item.filename)
    if item.filename == "3D/3dmodel.model":
        text = data.decode()
        assert "printable=\"1\"" in text, "no printable item"
        data = text.replace("printable=\"1\"", "printable=\"0\"", 1).encode()
    zout.writestr(item, data)
zout.close()
' asm-$e.3mf unp-$e.3mf
    run mtu-$e "$bin" unp-$e.3mf --slice 1 --mtcpp 20 --outputdir mtu-$e/out
    py '
import json, sys; d = json.load(open(sys.argv[1]))
assert d["return_code"] == -59, d
' mtu-$e/out/result.json
done
echo "PASS: --mtcpp counts an unprintable instance inside the bed (both engines)"

# --load-filaments with one filament in every slot turns the prime tower off;
# two different filaments keep it.
py '
import json
for name, colour in (("Test Filament A", "#FF0000"), ("Test Filament B", "#0000FF")):
    json.dump({"type": "filament", "from": "system", "name": name, "filament_id": "GFL99",
               "filament_type": ["PLA"], "filament_colour": [colour]},
              open(name.split()[-1].lower() + "-fil.json", "w"), indent=4)
'
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    # A project with the prime tower on whose plate prints two filaments: two
    # copies of the cube, one on each filament. A plate that prints one
    # filament has no tower in either engine (the official legacy check,
    # BambuStudio.cpp 5816-5865; normalize_fdm_2, OrcaSlicer PrintConfig.cpp
    # 8455-8470).
    py '
import json
json.dump({"plates": [{"plate_name": "two", "need_arrange": True,
                       "objects": [{"path": "cube.stl", "count": 2, "filaments": [1, 2]}]}]},
          open("tower-list.json", "w"))
'
    run towp-$e "$bin" --load-assemble-list tower-list.json --slice 1 --printer-preset "$A1M" \
        --filament-preset "Bambu PLA Basic @BBL A1M" --filament-preset "Bambu PLA Matte @BBL A1M" \
        --enable-prime-tower --outputdir towp-$e/out --export-3mf tower.3mf
    [ "$(rc towp-$e)" = 0 ] || { show towp-$e; fail "$e: tower project export exit $(rc towp-$e)"; }
    cp towp-$e/out/tower.3mf tower-$e.3mf
    # Two named filaments give two slots in every per-filament list.
    py '
import json, sys, zipfile
d = json.loads(zipfile.ZipFile(sys.argv[1]).read("Metadata/project_settings.config"))
for k in ("filament_settings_id", "filament_colour", "filament_multi_colour", "filament_colour_type",
          "filament_type", "filament_ids", "filament_map"):
    if k in d:
        assert len(d[k]) == 2, (k, d[k])
assert len(d["flush_volumes_matrix"]) % 4 == 0, d["flush_volumes_matrix"]
' tower-$e.3mf
    run tow1-$e "$bin" tower-$e.3mf --slice 1 --load-filaments "a-fil.json;a-fil.json" --outputdir tow1-$e/out
    [ "$(rc tow1-$e)" = 0 ] || { show tow1-$e; fail "$e: --load-filaments with one filament exit $(rc tow1-$e)"; }
    has_line tow1-$e/out/plate_1.gcode '; enable_prime_tower = 0' || fail "$e: one filament in every slot kept the prime tower"
    events tow1-$e/stdout config_normalized | grep PrimeTowerOffOneFilament >/dev/null || fail "$e: no PrimeTowerOffOneFilament event"
    run tow2-$e "$bin" tower-$e.3mf --slice 1 --load-filaments "a-fil.json;b-fil.json" --outputdir tow2-$e/out
    [ "$(rc tow2-$e)" = 0 ] || { show tow2-$e; fail "$e: --load-filaments with two filaments exit $(rc tow2-$e)"; }
    if events tow2-$e/stdout config_normalized | grep PrimeTowerOffOneFilament >/dev/null; then
        fail "$e: two different filaments turned the prime tower off"
    fi
    has_line tow2-$e/out/plate_1.gcode '; enable_prime_tower = 1' || fail "$e: two filaments lost the prime tower"
done
echo "PASS: one filament in every slot turns the prime tower off (both engines)"

# A printer with more extruders than filaments: OrcaSlicer gives it one
# filament per extruder (update_multi_material_filament_presets), and every
# per-filament list of the project has that many values, as the desktop's
# load_selections sizes them. Bambu Studio adds no filament there; its lists
# match too.
for e in bambu orca; do
    bin=$B; P="Bambu Lab H2D 0.4 nozzle"
    [ $e = orca ] && { bin=$O; P="Lulzbot Taz Pro Dual 0.5 nozzle"; }
    run mtl-$e "$bin" cube.stl --slice 1 --printer-preset "$P" --outputdir mtl-$e/out --export-3mf mt.3mf
    [ "$(rc mtl-$e)" = 0 ] || { show mtl-$e; fail "$e: $P exit $(rc mtl-$e)"; }
    py '
import json, sys, zipfile
d = json.loads(zipfile.ZipFile(sys.argv[1]).read("Metadata/project_settings.config"))
n = len(d["filament_settings_id"])
if sys.argv[2] == "orca":
    assert n == len(d["nozzle_diameter"]), (n, d["nozzle_diameter"])
for k in ("filament_colour", "filament_colour_type", "filament_multi_colour", "filament_is_support",
          "filament_flow_ratio", "filament_max_volumetric_speed", "filament_type"):
    if k in d:
        assert len(d[k]) == n, (k, d[k], n)
if sys.argv[2] == "orca":
    # The added slots take the desktop load_selections values (PresetBundle.cpp 2780-2795).
    assert d["filament_colour"][1:] == ["#26A69A"] * (n - 1), d["filament_colour"]
    assert d["filament_multi_colour"][1:] == ["#26A69A"] * (n - 1), d["filament_multi_colour"]
    assert d["filament_colour_type"][1:] == ["1"] * (n - 1), d["filament_colour_type"]
' mtl-$e/out/mt.3mf $e
done
echo "PASS: every per-filament list matches the filament count on a multi-extruder printer (both engines)"

# Non-ASCII names: a part named "würfel" in a folder "ü-<engine>" slices, and
# --export-stls writes it into a non-ASCII folder (boost::filesystem reads
# UTF-8 paths on Windows too). The export reads the part from the current
# folder: on Windows an STL's object name keeps any folder written with "/"
# (load_stl splits on "\\" only, STL.cpp 10-12 and 33-34 in both engines), and
# that folder is then missing under the export folder.
cp cube.stl würfel.stl
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    mkdir -p "ü-$e"; cp cube.stl "ü-$e/würfel.stl"
    run u8-$e "$bin" "ü-$e/würfel.stl" --slice 1 --printer-preset "$A1M" --outputdir "ü-$e/aus"
    [ "$(rc u8-$e)" = 0 ] && [ -s "ü-$e/aus/plate_1.gcode" ] || { show u8-$e; fail "$e: a non-ASCII path exit $(rc u8-$e)"; }
    run u8s-$e "$bin" würfel.stl --export-stls "ü-$e/stls"
    # A settings file in a non-ASCII folder (--machine reads it through
    # boost::nowide, as the arguments are UTF-8 on Windows).
    py '
import json, sys
json.dump({"type": "machine", "name": "Test Maschine", "printable_area": ["0x0", "200x0", "200x200", "0x200"],
           "printable_height": "200", "before_layer_change_gcode": "G92 E0", "layer_change_gcode": "G92 E0"},
          open(sys.argv[1], "w"))
' "ü-$e/maschine.json"
    run u8m-$e "$bin" würfel.stl --machine "ü-$e/maschine.json" -o "ü-$e/maschine.gcode"
    [ "$(rc u8m-$e)" = 0 ] && [ -s "ü-$e/maschine.gcode" ] || { show u8m-$e; fail "$e: --machine with a non-ASCII path exit $(rc u8m-$e)"; }
    [ "$(rc u8s-$e)" = 0 ] || { show u8s-$e; fail "$e: --export-stls with a non-ASCII name exit $(rc u8s-$e)"; }
    py '
import os, sys
names = os.listdir(sys.argv[1])
assert len(names) == 1 and "würfel" in names[0], names
' "ü-$e/stls"
done
echo "PASS: non-ASCII file names and folders (both engines)"

# A printer change through --load-settings moves the plate onto the new bed
# (translate_models, BambuStudio.cpp 4497-4644; OrcaSlicer.cpp 3885-3980): an
# X1 Carbon project whose cube stands at (220, 220) lands on the A1 mini's
# 180 mm bed. A by-object project whose printer change widens the extruder
# clearance is arranged again (BambuStudio.cpp 5275-5288; OrcaSlicer.cpp
# 4538-4549) instead of failing the clearance check (-63).
py '
import json
json.dump({"plates": [{"plate_name": "p", "need_arrange": False,
            "objects": [{"path": "cube.stl", "count": 1, "filaments": [1], "pos_x": [220], "pos_y": [220]}]}]},
          open("far.json", "w"))
json.dump({"plates": [{"plate_name": "p", "need_arrange": True, "plate_params": {"print_sequence": "by object"},
            "objects": [{"path": "cube.stl", "count": 2, "filaments": [1]}]}]}, open("seq2.json", "w"))
far = {"plate_name": "p", "need_arrange": False,
       "objects": [{"path": "cube.stl", "count": 1, "filaments": [1], "pos_x": [220], "pos_y": [220]}]}
json.dump({"plates": [far, dict(far, plate_name="q")]}, open("far2.json", "w"))
'
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run far-$e "$bin" --load-assemble-list far.json --slice 1 --printer-preset "Bambu Lab X1 Carbon 0.4 nozzle" \
        --outputdir far-$e/out --export-3mf far.3mf
    [ "$(rc far-$e)" = 0 ] || { show far-$e; fail "$e: far-corner X1 Carbon project exit $(rc far-$e)"; }
    run a1mset-$e "$bin" cube.stl --printer-preset "$A1M" --export-settings a1m-all-$e.json
    run p1sset-$e "$bin" cube.stl --printer-preset "Bambu Lab P1S 0.4 nozzle" --export-settings p1s-all-$e.json
    py '
import json, sys
e = sys.argv[1]
d = json.load(open("a1m-all-%s.json" % e))
d.update({"type": "machine", "from": "system", "name": "Bambu Lab A1 mini 0.4 nozzle", "instantiation": "true"})
d.pop("inherits", None)
json.dump(d, open("a1m-%s.json" % e, "w"), indent=1)
d = json.load(open("p1s-all-%s.json" % e))
d.update({"type": "machine", "from": "system", "name": "Bambu Lab P1S 0.4 nozzle", "instantiation": "true"})
d.pop("inherits", None)
d["extruder_clearance_radius" if "extruder_clearance_radius" in d else "extruder_clearance_max_radius"] = "110"
json.dump(d, open("p1s-wide-%s.json" % e, "w"), indent=1)
' $e
    run sw-$e "$bin" far-$e/out/far.3mf --slice 1 --load-settings a1m-$e.json --outputdir sw-$e/out
    [ "$(rc sw-$e)" = 0 ] || { show sw-$e; fail "$e: printer change to the A1 mini exit $(rc sw-$e)"; }
    events sw-$e/stdout arranged | grep MovedToNewBed >/dev/null || fail "$e: no MovedToNewBed event"
    py '
import json, sys
o = json.load(open(sys.argv[1]))["sliced_plates"][0]["objects"]
b = o[0]["bbox"]
assert len(o) == 1 and 0 <= b["x"] and b["x"] + b["width"] <= 180 and 0 <= b["y"] and b["y"] + b["depth"] <= 180, o
' sw-$e/out/result.json
    run seq2-$e "$bin" --load-assemble-list seq2.json --slice 1 --printer-preset "Bambu Lab X1 Carbon 0.4 nozzle" \
        --outputdir seq2-$e/out --export-3mf seq2.3mf
    [ "$(rc seq2-$e)" = 0 ] || { show seq2-$e; fail "$e: by-object X1 Carbon project exit $(rc seq2-$e)"; }
    run clr-$e "$bin" seq2-$e/out/seq2.3mf --slice 1 --load-settings p1s-wide-$e.json --outputdir clr-$e/out
    [ "$(rc clr-$e)" = 0 ] || { show clr-$e; fail "$e: a wider clearance on a printer change exit $(rc clr-$e)"; }
    events clr-$e/stdout arranged | grep ClearanceArrange >/dev/null || fail "$e: no ClearanceArrange event"
done
echo "PASS: a printer change moves the plate onto the new bed, and a wider clearance arranges a by-object plate again (both engines)"

# A printer change moves the plate after the settings given as flags apply
# (BambuStudio.cpp 4091 then 4659; OrcaSlicer.cpp 3542 then 3885): an X1
# Carbon project with two filaments and the prime tower, switched to the A1
# mini with --enable-prime-tower=0, centres its two cubes alone (no tower in
# the box). Its two cubes stand side by side (a copy is the first cube moved
# again by its own entry, BambuStudio.cpp 1253-1259), so their box is
# 50 x 20 mm and lands at 65..115 x 80..100 on the 180 mm bed.
py '
import json
json.dump({"plates": [{"plate_name": "t", "need_arrange": False,
            "objects": [{"path": "cube.stl", "count": 2, "filaments": [1, 2], "pos_x": [195, 30], "pos_y": [220, 0]}]}]},
          open("ftow.json", "w"))
'
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run ftow-$e "$bin" --load-assemble-list ftow.json --slice 1 --printer-preset "Bambu Lab X1 Carbon 0.4 nozzle" \
        --filament-preset "Bambu PLA Basic @BBL X1C" --filament-preset "Bambu PLA Matte @BBL X1C" \
        --enable-prime-tower --wipe-tower-x 20 --wipe-tower-y 20 --outputdir ftow-$e/out --export-3mf ftow.3mf
    [ "$(rc ftow-$e)" = 0 ] || { show ftow-$e; fail "$e: two-filament X1 Carbon project exit $(rc ftow-$e)"; }
    run swt-$e "$bin" ftow-$e/out/ftow.3mf --slice 1 --load-settings a1m-$e.json --enable-prime-tower=0 --outputdir swt-$e/out
    [ "$(rc swt-$e)" = 0 ] || { show swt-$e; fail "$e: printer change with --enable-prime-tower=0 exit $(rc swt-$e)"; }
    py '
import json, sys
o = json.load(open(sys.argv[1]))["sliced_plates"][0]["objects"]
x0 = min(b["bbox"]["x"] for b in o); x1 = max(b["bbox"]["x"] + b["bbox"]["width"] for b in o)
y0 = min(b["bbox"]["y"] for b in o); y1 = max(b["bbox"]["y"] + b["bbox"]["depth"] for b in o)
assert len(o) == 2 and abs(x0 - 65) < 0.5 and abs(x1 - 115) < 0.5 and abs(y0 - 80) < 0.5 and abs(y1 - 100) < 0.5, (x0, x1, y0, y1)
' swt-$e/out/result.json
done
echo "PASS: a printer change moves the plate after the setting flags apply (both engines)"

# A refusal before anything loads still leaves result.json under --slice:
# named presets on a project 3MF, and --plate with --slice (CLI_INVALID_PARAMS).
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run rfp-$e "$bin" ftow-$e/out/ftow.3mf --slice 1 --printer-preset "$A1M" --outputdir rfp-$e/out
    run rfq-$e "$bin" ftow-$e/out/ftow.3mf --slice 1 --plate 1 --outputdir rfq-$e/out
    for n in rfp rfq; do
        [ "$(rc $n-$e)" != 0 ] || { show $n-$e; fail "$e: $n was not refused"; }
        py '
import json, sys; d = json.load(open(sys.argv[1]))
assert d["return_code"] == -2, d
' $n-$e/out/result.json || { show $n-$e; fail "$e: the $n refusal left no result.json with -2"; }
    done
done
echo "PASS: early refusals under --slice leave result.json (both engines)"

# A --slice value that is not a plate (--slice foo, --slice=-1) is refused
# with result.json too (CLI_INVALID_PARAMS, -2); a project 3MF that is not the
# first file is refused with CLI_FILELIST_INVALID_ORDER (-4; BambuStudio.cpp
# 1890-1897; OrcaSlicer.cpp 1572-1579).
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run rsf-$e "$bin" cube.stl --slice foo --outputdir rsf-$e/out
    run rsn-$e "$bin" cube.stl --slice=-1 --outputdir rsn-$e/out
    run rord-$e "$bin" cube.stl ftow-$e/out/ftow.3mf --slice 1 --printer-preset "$A1M" --outputdir rord-$e/out
    run rord2-$e "$bin" ftow-$e/out/ftow.3mf ftow-$e/out/ftow.3mf --slice 1 --outputdir rord2-$e/out
    for n in rsf:-2 rsn:-2 rord:-4 rord2:-4; do
        name=${n%%:*}; want=${n##*:}
        [ "$(rc $name-$e)" != 0 ] || { show $name-$e; fail "$e: $name was not refused"; }
        py '
import json, sys; d = json.load(open(sys.argv[1]))
assert d["return_code"] == int(sys.argv[2]), d
' $name-$e/out/result.json "$want" || { show $name-$e; fail "$e: $name left no result.json with $want"; }
    done
done
echo "PASS: a bad --slice value and a project 3MF that is not first are refused with result.json (both engines)"

# A --slice run never ends without slicing or result.json: --engine-info and
# --list-presets (slicer-cli's own) are refused with it (-2); --help prints
# the flags and the slice goes on, as the official's help action does
# (BambuStudio.cpp 6337-6338; OrcaSlicer.cpp 5471-5472).
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run rei-$e "$bin" --engine-info cube.stl --slice 1 --outputdir rei-$e/out
    run rlp-$e "$bin" --list-presets --slice 1 --outputdir rlp-$e/out
    for n in rei rlp; do
        py '
import json, sys; d = json.load(open(sys.argv[1]))
assert d["return_code"] == -2, d
' $n-$e/out/result.json || { show $n-$e; fail "$e: $n with --slice left no result.json with -2"; }
    done
    run rhp-$e "$bin" cube.stl --help --slice 1 --printer-preset "$A1M" --outputdir rhp-$e/out
    [ "$(rc rhp-$e)" = 0 ] && [ -s rhp-$e/out/plate_1.gcode ] && grep -q -- '--slice' rhp-$e/stdout || { show rhp-$e; fail "$e: --help with --slice did not print the flags and slice"; }
done
echo "PASS: info-only flags with --slice are refused with result.json, and --help with --slice slices (both engines)"

# The tower an arrange sets is the one the project keeps: with --repetitions
# on a printer with another bed, the exported tower is the one the copies were
# arranged with, not the one moved to the new bed (the official exports the
# m_print_config its arrange updated, BambuStudio.cpp 5693-5700 and 8156;
# OrcaSlicer.cpp 4954-4961 and 6985).
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run rpt-$e "$bin" ftow-$e/out/ftow.3mf --slice 1 --load-settings a1m-$e.json --repetitions 2 \
        --outputdir rpt-$e/out --export-3mf rpt.3mf
    [ "$(rc rpt-$e)" = 0 ] || { show rpt-$e; fail "$e: --repetitions on a printer change exit $(rc rpt-$e)"; }
    py '
import json, re, sys, zipfile
d = sys.argv[1]
g = open(d + "/plate_1.gcode", errors="replace").read()
gx = float(re.search(r"^; wipe_tower_x = (.*)$", g, re.M).group(1).split(",")[0])
gy = float(re.search(r"^; wipe_tower_y = (.*)$", g, re.M).group(1).split(",")[0])
s = json.loads(zipfile.ZipFile(d + "/rpt.3mf").read("Metadata/project_settings.config"))
ex, ey = float(s["wipe_tower_x"][0]), float(s["wipe_tower_y"][0])
assert abs(gx - ex) < 0.01 and abs(gy - ey) < 0.01, ("sliced", gx, gy, "exported", ex, ey)
' rpt-$e/out || { show rpt-$e; fail "$e: the exported tower is not the one the copies were arranged with"; }
done
echo "PASS: --repetitions on a printer change exports the tower the copies were arranged with (both engines)"

# --ensure-on-bed lifts a sunk object before the actions given before --slice
# (BambuStudio.cpp 6188-6194 then the action loop at 6336; OrcaSlicer.cpp
# 5443-5455 then 5470): --export-stl writes the lifted objects.
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    py '
import re, sys, zipfile
zi = zipfile.ZipFile(sys.argv[1]); zo = zipfile.ZipFile(sys.argv[2], "w", zipfile.ZIP_DEFLATED)
for it in zi.infolist():
    data = zi.read(it.filename)
    if it.filename == "3D/3dmodel.model":
        def sink(m):
            v = m.group(2).split(); v[11] = "%g" % (float(v[11]) - 5.0)
            return m.group(1) + " ".join(v) + chr(34)
        t, n = re.subn(r"(<item [^>]*transform=\")([^\"]*)\"", sink, data.decode())
        assert n > 0, "no build item"
        data = t.encode()
    zo.writestr(it, data)
zo.close()
' ftow-$e/out/ftow.3mf sunk-$e.3mf
    run eob-$e "$bin" sunk-$e.3mf --ensure-on-bed --export-stl --slice 1 --outputdir eob-$e/out
    [ "$(rc eob-$e)" = 0 ] || { show eob-$e; fail "$e: --ensure-on-bed --export-stl --slice 1 exit $(rc eob-$e)"; }
    py '
import glob, re, struct, sys
zs = []
for f in glob.glob(sys.argv[1] + "/*.stl"):
    d = open(f, "rb").read()
    if d[:5] == b"solid" and b"facet" in d[:300]:
        zs += [float(m.split()[2]) for m in re.findall(rb"vertex\s+(\S+\s+\S+\s+\S+)", d)]
    else:
        n = struct.unpack("<I", d[80:84])[0]
        for i in range(n):
            v = struct.unpack("<12f", d[84 + 50 * i:84 + 50 * i + 48])
            zs += [v[5], v[8], v[11]]
assert zs and abs(min(zs)) < 0.01, ("lowest z", min(zs) if zs else None)
' eob-$e/out/stl || { show eob-$e; fail "$e: --export-stl wrote the sunk object before --ensure-on-bed lifted it"; }
done
echo "PASS: --ensure-on-bed lifts a sunk object before the actions given before --slice (both engines)"

# --slice 0 never arranges a by-object plate again for wider clearances: the
# official does so only for one named plate (plate_to_slice > 0,
# BambuStudio.cpp 5275; OrcaSlicer.cpp 4538).
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run clr0-$e "$bin" seq2-$e/out/seq2.3mf --slice 0 --load-settings p1s-wide-$e.json --outputdir clr0-$e/out
    # Not arranged again, the two cubes stand too close for the wider
    # clearance: the by-object check refuses the plate (-63), as the official
    # does for --slice 0 (CLI_OBJECT_COLLISION_IN_SEQ_PRINT, BambuStudio.cpp
    # 7000-7001; OrcaSlicer.cpp 6007-6008).
    py '
import json, sys; d = json.load(open(sys.argv[1]))
assert d["return_code"] == -63, (d["return_code"], d.get("error_string"))
' clr0-$e/out/result.json || { show clr0-$e; fail "$e: --slice 0 on the by-object project did not end with the clearance refusal (-63), exit $(rc clr0-$e)"; }
    if events clr0-$e/stdout arranged | grep ClearanceArrange >/dev/null; then
        fail "$e: --slice 0 arranged a by-object plate again"
    fi
done
echo "PASS: --slice 0 does not arrange a by-object plate again for wider clearances; the clearance check refuses it (-63) (both engines)"

# A printer change moves the sliced plate onto the new bed (translate_models,
# BambuStudio.cpp 4516-4642; OrcaSlicer.cpp 3901-3977): the cube (saved
# centred at 138, 138) is centred on its plate, a move of -48 mm, and the
# plate goes to its place in the A1 mini's grid (3 plates: 2 columns, stride
# 180 * 1.2 = 216 mm). With --slice N the export holds plate N's objects
# only, as the official loads only that plate (BambuStudio.cpp 1889). The
# tower list holds 2 values for 3 plates (given as flags for --slice 2, in
# the settings file for --slice 3): plate N's entry moves by -48 (plate 3's
# is the first value, as set_at and get_at fill with it, Config.hpp 437).
# The project keeps the moved tower (the official exports m_print_config,
# which holds the flags, 4091, before the move); the other plates' entries
# stay as given (the official moves them by the box of a plate it did not
# load, an empty box).
py '
import json
plate = lambda n: {"plate_name": n, "need_arrange": False,
                   "objects": [{"path": "cube.stl", "count": 1, "filaments": [1], "pos_x": [128], "pos_y": [128]}]}
json.dump({"plates": [plate("a"), plate("b"), plate("c")]}, open("three.json", "w"))
'
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run three-$e "$bin" --load-assemble-list three.json --slice 0 --printer-preset "Bambu Lab X1 Carbon 0.4 nozzle" \
        --outputdir three-$e/out --export-3mf three.3mf
    [ "$(rc three-$e)" = 0 ] || { show three-$e; fail "$e: three-plate X1 Carbon project exit $(rc three-$e)"; }
    py '
import json, sys
d = json.load(open("a1m-%s.json" % sys.argv[1]))
d["wipe_tower_x"] = ["100", "150"]; d["wipe_tower_y"] = ["120", "140"]
json.dump(d, open("a1m-tw-%s.json" % sys.argv[1], "w"), indent=1)
' $e
    for p in 2 3; do
        if [ $p = 2 ]; then tower="--load-settings a1m-$e.json --wipe-tower-x 100,150 --wipe-tower-y 120,140"
        else tower="--load-settings a1m-tw-$e.json"; fi
        run mv$p-$e "$bin" three-$e/out/three.3mf --slice $p $tower --outputdir mv$p-$e/out --export-3mf moved.3mf
        [ "$(rc mv$p-$e)" = 0 ] || { show mv$p-$e; fail "$e: --slice $p after a printer change exit $(rc mv$p-$e)"; }
        py '
import json, re, sys, zipfile
z = zipfile.ZipFile(sys.argv[1])
model = z.read("3D/3dmodel.model").decode()
build = model[model.index("<build"):model.index("</build>")]
centres = sorted((round(float(t[9])), round(float(t[10])))
                 for t in (m.split() for m in re.findall(r"<item [^>]*transform=\"([^\"]+)\"", build)))
d = json.loads(z.read("Metadata/project_settings.config"))
wx = [round(float(v)) for v in d["wipe_tower_x"]]; wy = [round(float(v)) for v in d["wipe_tower_y"]]
p = sys.argv[2]
ok_centres = centres == ([(306, 90)] if p == "2" else [(90, -126)])
ok_tower = (wx, wy) == (([100, 102], [120, 92]) if p == "2" else ([100, 150, 52], [120, 140, 72]))
assert ok_centres and ok_tower, ("centres", centres, "tower", wx, wy)
' mv$p-$e/out/moved.3mf $p
    done
done
echo "PASS: a printer change moves the sliced plate of the exported project, and a short tower list fills with its first value (both engines)"

# --nozzle on a two-extruder printer keeps both extruders: the value fills
# the list at its size (the extruder count is that size).
for e in bambu orca; do
    bin=$B; P="Bambu Lab H2D 0.4 nozzle"; N=0.4
    [ $e = orca ] && { bin=$O; P="Lulzbot Taz Pro Dual 0.5 nozzle"; N=0.5; }
    run noz2-$e "$bin" cube.stl --slice 1 --printer-preset "$P" --nozzle $N --outputdir noz2-$e/out
    [ "$(rc noz2-$e)" = 0 ] || { show noz2-$e; fail "$e: --nozzle on $P exit $(rc noz2-$e)"; }
    has_line noz2-$e/out/plate_1.gcode "; nozzle_diameter = $N,$N" ||
        fail "$e: --nozzle on $P left $(grep -m1 '^; nozzle_diameter' noz2-$e/out/plate_1.gcode.lf)"
done
echo "PASS: --nozzle keeps a two-extruder printer's two nozzles (both engines)"

# --nozzle writes nozzle_diameter as the list it is; --engine-info counts a
# geometry-only 3MF as one plate; --slice 0 --export-stls exports every
# plate's objects once (the official loads the whole project for --slice 0).
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run noz-$e "$bin" cube.stl --printer-preset "$A1M" --nozzle 0.4 -o noz-$e.gcode
    [ "$(rc noz-$e)" = 0 ] && [ -s noz-$e.gcode ] || { show noz-$e; fail "$e: --nozzle 0.4 exit $(rc noz-$e)"; }
    run eig-$e "$bin" --engine-info geo-plates.3mf
    py '
import json, sys
text = open(sys.argv[1]).read()
d = json.loads(text[text.index("{"):])
assert d.get("plates") == 1, d.get("plates")
' eig-$e/stdout
    rm -rf stl0-$e
    run stl0-$e "$bin" asm-$e.3mf --slice 0 --export-stls stl0-$e --outputdir stl0-$e/out
    [ "$(rc stl0-$e)" = 0 ] || { show stl0-$e; fail "$e: --slice 0 --export-stls exit $(rc stl0-$e)"; }
    py '
import os, sys
names = sorted(n for n in os.listdir(sys.argv[1]) if n.endswith(".stl"))
# Plate 1: two cubes; plate 2: one object of two merged copies.
assert len(names) == 3, names
' stl0-$e
    # The actions run in command-line order, and the first that fails stops
    # the rest (one action loop, BambuStudio.cpp 6366-6401; OrcaSlicer.cpp
    # 5499-5534): an --export-stls into a regular file fails with -11 after
    # an --export-settings before it, and before an --export-settings after it.
    rm -rf stlbad-$e ord1-$e ord2-$e ord1-$e.json ord2-$e.json; : > stlbad-$e
    run ord1-$e "$bin" asm-$e.3mf --slice 0 --export-settings ord1-$e.json --export-stls stlbad-$e --outputdir ord1-$e/out
    run ord2-$e "$bin" asm-$e.3mf --slice 0 --export-stls stlbad-$e --export-settings ord2-$e.json --outputdir ord2-$e/out
    for n in ord1 ord2; do
        py '
import json, sys; d = json.load(open(sys.argv[1]))
assert d["return_code"] == -11 and d["plate_index"] == 0, d
' $n-$e/out/result.json || { show $n-$e; fail "$e: --slice 0 with a failing --export-stls did not end with -11 on plate 0"; }
    done
    test -s ord1-$e.json || { show ord1-$e; fail "$e: --export-settings before a failing --export-stls wrote no settings"; }
    test ! -e ord2-$e.json || fail "$e: --export-settings after a failing --export-stls ran"
    # The whole project moves onto a new printer's bed before the actions, as
    # each plate does (translate_models on every plate, BambuStudio.cpp
    # 4645-4659; OrcaSlicer.cpp 3982-3996, before the actions): a two-plate X1
    # Carbon project with a cube at (220, 220) on each plate, on the A1 mini's
    # 180 mm bed. Plate 1's cube is on that bed; plate 2's is on plate 2 of a
    # 180 mm grid (origin x = 180 * 1.2 = 216, compute_origin_using_new_size).
    run far2-$e "$bin" --load-assemble-list far2.json --slice 0 --printer-preset "Bambu Lab X1 Carbon 0.4 nozzle" \
        --outputdir far2-$e/out --export-3mf far2.3mf
    [ "$(rc far2-$e)" = 0 ] || { show far2-$e; fail "$e: two-plate far-corner X1 Carbon project exit $(rc far2-$e)"; }
    rm -rf sw2-$e
    run sw2-$e "$bin" far2-$e/out/far2.3mf --slice 0 --load-settings a1m-$e.json --export-stls sw2-$e/stls --outputdir sw2-$e/out
    [ "$(rc sw2-$e)" = 0 ] || { show sw2-$e; fail "$e: --slice 0 --export-stls on a printer change exit $(rc sw2-$e)"; }
    py '
import os, struct, sys
boxes = []
for n in sorted(n for n in os.listdir(sys.argv[1]) if n.endswith(".stl")):
    data = open(os.path.join(sys.argv[1], n), "rb").read()
    count = struct.unpack_from("<I", data, 80)[0]
    xs, ys = [], []
    for i in range(count):
        v = struct.unpack_from("<12f", data, 84 + 50 * i)
        xs += v[3::3]; ys += v[4::3]
    boxes.append((min(xs), max(xs), min(ys), max(ys)))
boxes.sort()
assert len(boxes) == 2, boxes
p1, p2 = boxes
assert 0 <= p1[0] and p1[1] <= 180 and 0 <= p1[2] and p1[3] <= 180, boxes
assert 216 <= p2[0] and p2[1] <= 396 and 0 <= p2[2] and p2[3] <= 180, boxes
' sw2-$e/stls || { show sw2-$e; fail "$e: --slice 0 --export-stls did not move the plates onto the new bed"; }
done
echo "PASS: --nozzle, --engine-info on a geometry-only 3MF and --slice 0 --export-stls on a two-plate project, in command-line order and on a new printer's bed (both engines)"

# An action given before --slice runs before any plate's bed check, one given
# after it only once every plate has sliced: --slice is one action of the
# official loop, with each plate's bed check inside it (BambuStudio.cpp 6336,
# 6437, 6527-6534; OrcaSlicer.cpp 5470, 5561, 5650). Plate 2 of this list
# stands off the A1 mini's bed, so its bed check fails (-50). --slice 0 on
# several plates checks every plate before it slices any (pre_check,
# BambuStudio.cpp 6441, 7027, 7428; OrcaSlicer.cpp 5565, 6034, 6222), so
# neither order writes a G-code.
py '
import json
json.dump({"plates": [
    {"plate_name": "on", "need_arrange": True, "objects": [{"path": "cube.stl", "count": 1, "filaments": [1]}]},
    {"plate_name": "off", "need_arrange": False,
     "objects": [{"path": "cube.stl", "count": 1, "filaments": [1], "pos_x": [400], "pos_y": [400]}]}]},
    open("offbed.json", "w"))
'
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    rm -rf obA-$e obB-$e
    run obA-$e "$bin" --load-assemble-list offbed.json --export-stls obA-$e/stls --slice 0 --printer-preset "$A1M" --outputdir obA-$e/out
    run obB-$e "$bin" --load-assemble-list offbed.json --slice 0 --export-stls obB-$e/stls --printer-preset "$A1M" --outputdir obB-$e/out
    for n in obA obB; do
        py '
import json, sys; d = json.load(open(sys.argv[1]))
assert d["return_code"] == -50, d
' $n-$e/out/result.json || { show $n-$e; fail "$e: the off-bed plate 2 did not end with -50 ($n)"; }
    done
    py '
import os, sys
a = sorted(n for n in os.listdir(sys.argv[1]) if n.endswith(".stl")) if os.path.isdir(sys.argv[1]) else []
b = sorted(n for n in os.listdir(sys.argv[2]) if n.endswith(".stl")) if os.path.isdir(sys.argv[2]) else []
assert len(a) == 2, ("--export-stls before --slice", a)
assert b == [], ("--export-stls after --slice", b)
' obA-$e/stls obB-$e/stls || fail "$e: --export-stls did not follow its place against --slice"
    for n in obA obB; do
        [ ! -e $n-$e/out/plate_1.gcode ] || fail "$e: --slice 0 sliced plate 1 although plate 2 is off the bed ($n)"
    done
done
echo "PASS: --export-stls before --slice runs before the bed checks, after --slice only once every plate has sliced; --slice 0 checks every plate before slicing any (both engines)"

# --slice 0's check pass sends no event of its own: the run's settings
# events come once (the settings merge builds m_print_config once,
# BambuStudio.cpp 3246, 3384), each plate's own once, from its slice (two
# plates, so twice in all).
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run ev2-$e "$bin" far2-$e/out/far2.3mf --slice 0 --load-settings a1m-$e.json --outputdir ev2-$e/out
    [ "$(rc ev2-$e)" = 0 ] || { show ev2-$e; fail "$e: --slice 0 --load-settings on two plates exit $(rc ev2-$e)"; }
    events ev2-$e/stdout config_normalized > ev2-$e/normalized.jsonl
    py '
import collections, json, sys
c = collections.Counter(json.loads(l)["tag"] for l in open(sys.argv[1]) if l.strip())
assert c.get("SettingsFilesMerged") == 1 and c.get("PlateSettingsApplied") == 2, dict(c)
assert all(n == 2 for t, n in c.items() if t != "SettingsFilesMerged"), dict(c)
' ev2-$e/normalized.jsonl || { show ev2-$e; fail "$e: --slice 0 sent a settings event more than once per plate"; }
done
echo "PASS: --slice 0's check pass sends no event twice (both engines)"

# A project 3MF with model files after it, as the official CLI runs it: one
# model of every file, is_bbl_3mf reset per file so --slice N becomes 0
# (BambuStudio.cpp 1871, 2192-2196; OrcaSlicer.cpp 1553, 1839-1843), and
# every object arranged across the plates (BambuStudio.cpp 3947-3963,
# 5627-5722; OrcaSlicer.cpp 3441-3457, 4887-4983). The model file's part is
# on one plate, once, clear of the project's objects; the whole-project
# actions see it too.
py '
import json
p = {"plate_name": "p", "need_arrange": False, "objects": [{"path": "cube.stl", "count": 1, "filaments": [1], "pos_x": [118], "pos_y": [118]}]}
json.dump({"plates": [p, dict(p, plate_name="q")]}, open("trail2.json", "w"))
v = [(x, y, z) for z in (0, 10) for y in (0, 10) for x in (0, 10)]
f = [(0,2,1),(1,2,3),(4,5,6),(5,7,6),(0,1,4),(1,5,4),(2,6,3),(3,6,7),(0,4,2),(2,4,6),(1,3,5),(3,7,5)]
with open("extra.stl", "w") as o:
    o.write("solid extra\n")
    for t in f:
        o.write("facet normal 0 0 0\nouter loop\n")
        for i in t: o.write("vertex %g %g %g\n" % v[i])
        o.write("endloop\nendfacet\n")
    o.write("endsolid extra\n")
'
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run trp-$e "$bin" --load-assemble-list trail2.json --slice 0 --printer-preset "Bambu Lab X1 Carbon 0.4 nozzle" \
        --outputdir trp-$e/out --export-3mf trail2.3mf
    [ "$(rc trp-$e)" = 0 ] || { show trp-$e; fail "$e: two-plate project exit $(rc trp-$e)"; }
    run tr0-$e "$bin" trp-$e/out/trail2.3mf extra.stl --slice 0 --export-stls tr0-$e/stls --outputdir tr0-$e/out
    run tr2-$e "$bin" trp-$e/out/trail2.3mf extra.stl --slice 2 --outputdir tr2-$e/out
    for n in tr0 tr2; do
        [ "$(rc $n-$e)" = 0 ] || { show $n-$e; fail "$e: project + model file ($n) exit $(rc $n-$e)"; }
        py '
import json, sys
d = json.load(open(sys.argv[1]))
plates = d["sliced_plates"]
names = [o["name"] for p in plates for o in p["objects"]]
assert names.count("extra.stl") == 1, ("the model file part once", names)
assert len(plates) >= 1 and sorted(p["id"] for p in plates) == list(range(1, len(plates) + 1)), plates
for p in plates:
    boxes = [o["bbox"] for o in p["objects"]]
    for i in range(len(boxes)):
        for j in range(i + 1, len(boxes)):
            a, b = boxes[i], boxes[j]
            apart = a["x"] + a["width"] <= b["x"] or b["x"] + b["width"] <= a["x"] or \
                    a["y"] + a["depth"] <= b["y"] or b["y"] + b["depth"] <= a["y"]
            assert apart, ("objects overlap on plate", p["id"], a, b)
' $n-$e/out/result.json || { show $n-$e; fail "$e: project + model file ($n): the part is not on one plate, once, clear of the others"; }
        events $n-$e/stdout arranged | grep -q ProjectArrangedAcrossPlates || fail "$e: $n has no ProjectArrangedAcrossPlates event"
    done
    events tr2-$e/stdout config_normalized | grep -q ProjectWithModelsSlicesEveryPlate || fail "$e: --slice 2 with a model file did not say it slices every plate"
    py '
import os, sys
stls = [n for n in os.listdir(sys.argv[1]) if n.endswith(".stl")]
assert len(stls) == 3 and any("extra" in n for n in stls), stls
' tr0-$e/stls || { show tr0-$e; fail "$e: --export-stls on project + model file missed the model file part"; }
done
echo "PASS: a project 3MF with a model file after it is arranged across the plates, every plate sliced, the part once (both engines)"

# The arrange adds plates when the objects do not fit (create_plate from
# postprocess_bed_index_for_selected, Bambu PartPlate.cpp 6009-6051): a
# one-plate project with two 200 mm parts is sliced on two plates.
py '
import json
def box(path, s, h):
    v = [(x, y, z) for z in (0, h) for y in (0, s) for x in (0, s)]
    f = [(0,2,1),(1,2,3),(4,5,6),(5,7,6),(0,1,4),(1,5,4),(2,6,3),(3,6,7),(0,4,2),(2,4,6),(1,3,5),(3,7,5)]
    with open(path, "w") as o:
        o.write("solid b\n")
        for t in f:
            o.write("facet normal 0 0 0\nouter loop\n")
            for i in t: o.write("vertex %g %g %g\n" % v[i])
            o.write("endloop\nendfacet\n")
        o.write("endsolid b\n")
box("big1.stl", 200, 10); box("big2.stl", 200, 10)
p = {"plate_name": "p", "need_arrange": False, "objects": [{"path": "cube.stl", "count": 1, "filaments": [1], "pos_x": [118], "pos_y": [118]}]}
json.dump({"plates": [p]}, open("trail1.json", "w"))
'
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run tg1-$e "$bin" --load-assemble-list trail1.json --slice 1 --printer-preset "Bambu Lab X1 Carbon 0.4 nozzle" \
        --outputdir tg1-$e/out --export-3mf trail1.3mf
    [ "$(rc tg1-$e)" = 0 ] || { show tg1-$e; fail "$e: one-plate project exit $(rc tg1-$e)"; }
    run tgr-$e "$bin" tg1-$e/out/trail1.3mf big1.stl big2.stl --slice 0 --outputdir tgr-$e/out
    [ "$(rc tgr-$e)" = 0 ] && [ -s tgr-$e/out/plate_2.gcode ] || { show tgr-$e; fail "$e: project + two 200 mm parts exit $(rc tgr-$e), no plate 2"; }
    events tgr-$e/stdout arranged > tgr-$e/arranged.jsonl
    py '
import json, sys
e = [json.loads(l) for l in open(sys.argv[1]) if "ProjectArrangedAcrossPlates" in l][0]
assert e["file_plate_count"] == 1 and e["plate_count"] == 2, e
' tgr-$e/arranged.jsonl || { show tgr-$e; fail "$e: the arrange did not add a plate"; }
done
echo "PASS: the arrange of a project 3MF with model files adds the plates it needs (both engines)"

# One settings base for the run: the G-code header, --export-settings, the
# exported project's settings and result.json read the same settings for a
# plate. The official keeps one m_print_config, which its arrange updates
# (BambuStudio.cpp 5856; OrcaSlicer.cpp 5112) and which --export-settings
# (6366-6370; 5499-5503), result.json (6905-6910; 5908-5913) and the export
# (8156-8157; 6985) read, and slices each plate with a copy that has the
# plate's own settings on top (6902-6904; 5905-5907). Checked: the tower the
# arrange placed (--arrange 1, and --repetitions on a printer change, whose
# copies start the tower at the default corner), the print sequence, the
# filament list and the summary.
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    for c in "gb:--arrange 1" "gbr:--load-settings a1m-$e.json --repetitions 2"; do
    n=${c%%:*}; flags=${c#*:}
    run $n-$e "$bin" ftow-$e/out/ftow.3mf --slice 1 $flags --export-settings $n-$e.json \
        --outputdir $n-$e/out --export-3mf gb.3mf
    [ "$(rc $n-$e)" = 0 ] || { show $n-$e; fail "$e: --slice 1 $flags with --export-settings and --export-3mf exit $(rc $n-$e)"; }
    py '
import json, re, sys, zipfile
g = open(sys.argv[1], errors="replace").read().replace("\r", "")
s = json.load(open(sys.argv[2]))
z = zipfile.ZipFile(sys.argv[3])
p = json.loads(z.read("Metadata/project_settings.config"))
ms = z.read("Metadata/model_settings.config").decode()
r = json.load(open(sys.argv[4]))
hv = lambda k: re.search(r"^; " + k + r" = (.*)$", g, re.M).group(1)
items = lambda v: [x.strip().strip("\"") for x in re.split(r"[;,]", v)] if isinstance(v, str) else [str(x) for x in v]
close = lambda a, b: abs(float(a) - float(b)) < 0.01
# The tower the arrange placed (plate 1: entry 0).
tower = [(items(hv(k))[0], items(s[k])[0], items(p[k])[0]) for k in ("wipe_tower_x", "wipe_tower_y")]
assert all(close(a, b) and close(a, c) for a, b, c in tower), ("tower: G-code, --export-settings, 3MF", tower)
# The print sequence: the plate setting, else the project setting.
own = re.search(r"<plate>(?:(?!</plate>).)*?key=\"print_sequence\" value=\"([^\"]+)\"", ms, re.S)
want = own.group(1) if own else s["print_sequence"]
assert hv("print_sequence") == want and p["print_sequence"] == s["print_sequence"], (hv("print_sequence"), want, p["print_sequence"], s["print_sequence"])
# The filament list.
for k in ("filament_settings_id", "filament_colour", "filament_type"):
    assert items(hv(k)) == items(s[k]) == items(p[k]), (k, hv(k), s[k], p[k])
# The summary result.json states.
assert close(r["layer_height"], s["layer_height"]) and r["wall_loops"] == int(s["wall_loops"]), (r["layer_height"], r["wall_loops"], s["layer_height"], s["wall_loops"])
assert close(r["sparse_infill_density"], str(s["sparse_infill_density"]).rstrip("%")), (r["sparse_infill_density"], s["sparse_infill_density"])
' $n-$e/out/plate_1.gcode $n-$e.json $n-$e/out/gb.3mf $n-$e/out/result.json || { show $n-$e; fail "$e: $flags: the G-code header, --export-settings, the exported project and result.json disagree"; }
    done
done
echo "PASS: the G-code header, --export-settings, the exported project's settings and result.json agree on the plate's tower, print sequence, filaments and summary (both engines)"

# The PR #35 findings F1-F10 (and F7o), each a check of what the official
# command line does, on both engines: tests/test-pr35-findings.sh runs every
# block and lists each result.
if bash "$SCRIPT_DIR/test-pr35-findings.sh" "$B" "$O" > findings.log 2>&1; then
    sed -n '/^== summary/,$p' findings.log | grep -E '^(PASS|SKIP)'
else
    sed -n '/^== summary/,$p' findings.log
    fail "PR #35 findings (tests/test-pr35-findings.sh)"
fi

# The product's contracts a G-code compare cannot see (--layout-plan exit
# codes and JSON, the by-object collision line): tests/test-product-invariants.sh.
if bash "$SCRIPT_DIR/test-product-invariants.sh" "$B" "$O" > invariants.log 2>&1; then
    grep -E '^(PASS|SKIP)' invariants.log
else
    cat invariants.log
    fail "product invariants (tests/test-product-invariants.sh)"
fi

# --pipe (Linux only): one JSON line per progress step into the named pipe.
if [ "$(uname -s)" = Linux ]; then
    for e in bambu orca; do
        bin=$B; [ $e = orca ] && bin=$O
        rm -f pipe-$e.fifo; mkfifo pipe-$e.fifo
        timeout 120 cat pipe-$e.fifo > pipe-$e.lines & reader=$!
        run pipe-$e "$bin" cube.stl --slice 1 --printer-preset "$A1M" --outputdir pipe-$e/out --pipe pipe-$e.fifo
        wait $reader || true
        [ "$(rc pipe-$e)" = 0 ] || { show pipe-$e; fail "$e: --pipe exit $(rc pipe-$e)"; }
        py '
import json, sys
lines = [json.loads(l) for l in open(sys.argv[1]) if l.strip()]
assert lines, "nothing came through the pipe"
# The writer sends the latest step when it wakes, as the official one does,
# so a fast slice may skip steps; the last one is always the run end.
last = lines[-1]
assert last.get("message") == "All done, Success" and last["total_percent"] == 100, last
assert all(set(l) >= {"plate_index", "plate_count", "plate_percent", "total_percent"} for l in lines), lines
totals = [l["total_percent"] for l in lines]
assert totals == sorted(totals), totals
# Each record fits the official 512-byte buffer with its newline.
raw = open(sys.argv[1], "rb").read().split(b"\n")
assert all(len(r) + 1 <= 511 for r in raw if r), max(len(r) for r in raw)
' pipe-$e.lines
        # A reader that leaves after the first byte: the slice goes on and
        # finishes (SIGPIPE is ignored; the official OrcaSlicer CLI ignores
        # it for every run, OrcaSlicer.cpp 7464-7468).
        rm -f pipeq-$e.fifo; mkfifo pipeq-$e.fifo
        timeout 120 head -c 1 pipeq-$e.fifo > /dev/null & reader=$!
        run pipeq-$e "$bin" cube.stl --slice 1 --printer-preset "$A1M" --outputdir pipeq-$e/out --pipe pipeq-$e.fifo
        wait $reader || true
        [ "$(rc pipeq-$e)" = 0 ] && [ -s pipeq-$e/out/plate_1.gcode ] || { show pipeq-$e; fail "$e: --pipe with a reader that left exit $(rc pipeq-$e)"; }
        # Steps 2 and 3 come before any plate is checked (BambuStudio.cpp
        # 4923-4926, 6455-6458; OrcaSlicer.cpp 4185, 5580): --slice 0 refused
        # by its check pass (plate 2 off the bed) ends on step 3.
        rm -f pipeo-$e.fifo; mkfifo pipeo-$e.fifo
        timeout 120 cat pipeo-$e.fifo > pipeo-$e.lines & reader=$!
        run pipeo-$e "$bin" --load-assemble-list offbed.json --slice 0 --printer-preset "$A1M" --outputdir pipeo-$e/out --pipe pipeo-$e.fifo
        wait $reader || true
        py '
import json, sys
lines = [json.loads(l) for l in open(sys.argv[1]) if l.strip()]
assert lines and lines[-1].get("message") == "Prepare slicing" and lines[-1]["total_percent"] == 3, lines
' pipeo-$e.lines || { show pipeo-$e; fail "$e: --slice 0 refused by its check pass did not send pipe step 3"; }
    done
    echo "PASS: --pipe writes the official progress lines, steps 2 and 3 before any plate check, and a reader that leaves does not stop the slice (both engines, Linux)"
fi

# ---------------------------------------------------------------- Stage D
# Differences from the desktop app, each checked on both engines. Settings
# files are the engine's own system presets, flattened (inherits walked) from
# the package's resources, as the official CLIs read them.
RES="$(cd "$(dirname "$B")/.." && pwd -P)/resources"
# flat_presets ENGINE TAG PRINTER: TAG-ENGINE-machine.json, -process.json and
# -filament.json (the printer's default process and filament).
flat_presets() {
    local vendor="$RES/profiles/BBL"
    [ "$1" = orca ] && vendor="$RES/profiles-orca/BBL"
    py '
import json, os, sys
vendor, tag, printer = sys.argv[1:4]
def load(kind):
    out = {}
    for root, _, files in os.walk(os.path.join(vendor, kind)):
        for f in files:
            if f.endswith(".json"):
                try:
                    d = json.load(open(os.path.join(root, f), encoding="utf-8"))
                except Exception:
                    continue
                out[d.get("name", f[:-5])] = d
    return out
P = {k: load(k) for k in ("machine", "process", "filament")}
def resolve(kind, name, seen=()):
    d = P[kind][name]
    r = resolve(kind, d["inherits"], seen + (name,)) if d.get("inherits") and name not in seen else {}
    r.update(d); r.pop("inherits", None)
    return r
first = lambda v: v[0] if isinstance(v, list) else v
m = resolve("machine", printer)
proc = first(m.get("default_print_profile")); fil = first(m.get("default_filament_profile"))
for kind, name, d in (("machine", printer, m), ("process", proc, resolve("process", proc)), ("filament", fil, resolve("filament", fil))):
    d = dict(d); d.update({"type": kind, "from": "system", "name": name, "instantiation": "true"})
    json.dump(d, open("%s-%s.json" % (tag, kind), "w", encoding="utf-8"), indent=1)
' "$vendor" "$2-$1" "$3"
}

# D1: the Bambu printer features (M981 spaghetti detection, M1003 power-loss
# recovery) follow the printer's vendor, as the desktop decides
# (BackgroundSlicingProcess.cpp:205 is_bbl_vendor_preset) and the official CLI
# (printer_model, else the printer name: BambuStudio.cpp 7055-7070). An STL
# with the A1 mini's settings files gets the named preset's M981/M1003, through
# --load-settings and through the product's --machine/--filament/--process.
# The Orca build already decides by printer_model (OrcaSlicer.cpp 5972-5986):
# its --load-settings run is the control (its --machine route refuses these
# flattened presets for relative E without G92 E0, which is not this item).
d1_ok=1
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    flat_presets $e d1a1m "$A1M"
    run d1p-$e "$bin" cube.stl --printer-preset "$A1M" --slice 1 --outputdir d1p-$e/out
    [ "$(rc d1p-$e)" = 0 ] || { show d1p-$e; fail "D1 $e: the named A1 mini preset exit $(rc d1p-$e)"; d1_ok=0; continue; }
    want="M981=$(grep -c '^M981' d1p-$e/out/plate_1.gcode || true) M1003=$(grep -c '^M1003' d1p-$e/out/plate_1.gcode || true)"
    [ $e = orca ] || [ "$want" != "M981=0 M1003=0" ] || { fail "D1 $e: the named A1 mini preset has no M981/M1003"; d1_ok=0; }
    run d1l-$e "$bin" cube.stl --load-settings "d1a1m-$e-machine.json;d1a1m-$e-process.json" \
        --load-filaments d1a1m-$e-filament.json --slice 1 --outputdir d1l-$e/out
    routes="l"
    if [ $e = bambu ]; then
        run d1m-$e "$bin" cube.stl -o d1m-$e.gcode --machine d1a1m-$e-machine.json \
            --filament d1a1m-$e-filament.json --process d1a1m-$e-process.json
        routes="l m"
    fi
    for n in $routes; do
        g=d1l-$e/out/plate_1.gcode; [ $n = m ] && g=d1m-$e.gcode
        [ "$(rc d1$n-$e)" = 0 ] && [ -s $g ] || { show d1$n-$e; fail "D1 $e: settings files (route $n) exit $(rc d1$n-$e)"; d1_ok=0; continue; }
        got="M981=$(grep -c '^M981' $g || true) M1003=$(grep -c '^M1003' $g || true)"
        [ "$got" = "$want" ] || { fail "D1 $e: settings files (route $n) give $got, the named preset $want"; d1_ok=0; }
    done
done
[ $d1_ok = 1 ] && echo "PASS: D1 the Bambu printer features follow the printer's vendor, also for settings files (both engines)"

# D2: no per-filament setting is padded with 0 on a printer with two
# extruders. Which keys are per filament is the engine's own list of filament
# settings (Preset::filament_options(), Preset.cpp 1087-1140), not a
# hand-written one. A one-filament cube on the H2D: every list setting of the
# filament preset holds only the preset's values, in --export-settings and in
# the G-code header (on 2a12432 the Bambu build wrote hot_plate_temp = 60,0,
# fan_max_speed = 100,0 and ~45 more). The Orca build pads no per-filament
# setting (control).
d2_ok=1
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    prof="$RES/profiles/BBL"; [ $e = orca ] && prof="$RES/profiles-orca/BBL"
    run d2-$e "$bin" cube.stl --printer-preset "Bambu Lab H2D 0.4 nozzle" --slice 1 --export-settings d2-$e.json --outputdir d2-$e/out
    [ "$(rc d2-$e)" = 0 ] || { show d2-$e; fail "D2 $e: H2D cube exit $(rc d2-$e)"; d2_ok=0; continue; }
    py '
import json, os, re, sys
s = json.load(open(sys.argv[1])); vendor = sys.argv[2]; g = open(sys.argv[3], errors="replace").read()
profiles = {}
for root, _, files in os.walk(os.path.join(vendor, "filament")):
    for f in files:
        if f.endswith(".json"):
            try:
                d = json.load(open(os.path.join(root, f), encoding="utf-8"))
            except Exception:
                continue
            profiles[d.get("name", f[:-5])] = d
def resolve(n, seen=()):
    d = profiles.get(n)
    if d is None or n in seen: return {}
    r = resolve(d.get("inherits", ""), seen + (n,)) if d.get("inherits") else {}
    r.update(d); return r
fil = s["filament_settings_id"][0]
p = resolve(fil)
assert p, "no filament preset " + fil
skip = {"name", "inherits", "from", "type", "instantiation", "setting_id", "filament_id", "compatible_printers",
        "compatible_printers_condition", "compatible_prints", "compatible_prints_condition", "version", "description"}
# filament_extruder_variant is the variant-slot table, sized by the extruders
# of the printer on purpose (see the padding in main.cpp), not one value per filament.
skip.add("filament_extruder_variant")
bad = []
for k, v in p.items():
    if k in skip or k.endswith("_settings_id") or not isinstance(v, list) or not isinstance(s.get(k), list): continue
    if all(x == "" for x in v): continue
    # Padding: more entries than the one filament has, the extra ones not in the preset.
    if len(s[k]) > len(v) and [x for x in s[k] if x not in v]:
        bad.append("export %s = %s (preset %s)" % (k, ",".join(s[k]), ",".join(v)))
    # The header writes number lists comma-separated; only those are compared.
    if not all(re.fullmatch(r"-?[0-9.]+%?", x) for x in v): continue
    m = re.search(r"^; " + re.escape(k) + r" = (.*)$", g, re.M)
    hv = m.group(1).split(",") if m else []
    if len(hv) > len(v) and [x for x in hv if x not in v]:
        bad.append("G-code %s = %s (preset %s)" % (k, m.group(1), ",".join(v)))
assert not bad, "; ".join(bad[:12])
' d2-$e.json "$prof" d2-$e/out/plate_1.gcode || { fail "D2 $e: per-filament settings padded on the H2D"; d2_ok=0; }
done
[ $d2_ok = 1 ] && echo "PASS: D2 no per-filament setting is padded with 0 on the H2D (both engines)"

# D3: the Orca build gives automatic grouping the same estimated AMS slots as
# the official OrcaSlicer CLI (extruder_ams_count / set_extruder_filament_info,
# OrcaSlicer.cpp 5914-5951). N cubes in a row, cube k on filament k, auto
# grouping: the filament_map equals the official CLI's on its own presets
# (OrcaSlicer 2.4.0-alpha, the version of the pin 31f6803, measured on the
# Linux AppImage; the 8-filament projects the official refuses are not
# listed). The Bambu build runs BambuStudio.cpp's own step (6911-6950), so
# this item is checked on the Orca build only.
d3_ok=1
while IFS='|' read -r printer fils official; do
    n=$(echo "$fils" | tr ';' '\n' | wc -l | tr -d ' ')
    t="d3-$(printf '%s' "$printer-$n" | tr -c 'A-Za-z0-9' '_')"
    py '
import json, sys
n = int(sys.argv[1])
json.dump({"plates": [{"plate_name": "d3", "need_arrange": False,
            "objects": [{"path": "cube.stl", "count": n, "filaments": list(range(1, n + 1)),
                         "pos_x": [100 + 40 * i for i in range(n)], "pos_y": [150] * n}]}]}, open(sys.argv[2], "w"))
' $n $t.json
    fargs=(); IFS=';' read -ra fl <<< "$fils"; for f in "${fl[@]}"; do fargs+=(--filament-preset "$f"); done
    run $t "$O" --load-assemble-list $t.json --slice 1 --printer-preset "$printer" "${fargs[@]}" --outputdir $t/out
    [ "$(rc $t)" = 0 ] || { show $t; fail "D3 orca: $printer, $n filaments exit $(rc $t)"; d3_ok=0; continue; }
    ours=$(sed -n 's/^; filament_map = //p' $t/out/plate_1.gcode | head -n 1)
    [ "$ours" = "$official" ] || { fail "D3 orca: $printer, $n filaments: filament_map $ours, the official CLI $official"; d3_ok=0; }
done <<'D3EOF'
Bambu Lab H2D 0.4 nozzle|Bambu PLA Basic @BBL H2D;Bambu PLA Matte @BBL H2D|2,1
Bambu Lab H2D 0.4 nozzle|Bambu PLA Basic @BBL H2D;Bambu PLA Matte @BBL H2D;Bambu PETG HF @BBL H2D 0.4 nozzle;Bambu ABS @BBL H2D|1,1,1,2
Bambu Lab H2D Pro 0.4 nozzle|Bambu PLA Basic @BBL H2DP;Bambu PLA Matte @BBL H2DP|2,1
Bambu Lab H2D Pro 0.4 nozzle|Bambu PLA Basic @BBL H2DP;Bambu PLA Matte @BBL H2DP;Bambu PETG HF @BBL H2DP 0.4 nozzle;Bambu ABS @BBL H2DP|1,1,1,2
Bambu Lab X2D 0.4 nozzle|Bambu PLA Basic @BBL X2D 0.4 nozzle;Bambu PLA Matte @BBL X2D 0.4 nozzle|1,1
Bambu Lab X2D 0.4 nozzle|Bambu PLA Basic @BBL X2D 0.4 nozzle;Bambu PLA Matte @BBL X2D 0.4 nozzle;Bambu PETG HF @BBL X2D 0.4 nozzle;Bambu ABS @BBL X2D 0.4 nozzle|1,1,1,1
D3EOF
[ $d3_ok = 1 ] && echo "PASS: D3 the Orca build's automatic grouping equals the official OrcaSlicer CLI's on the H2D, H2D Pro and X2D"

# D4 (Bambu build; OrcaSlicer 31f6803 has no such setting): input without
# extruder_nozzle_stats gets the official CLI's value (BambuStudio.cpp
# 4141-4155, 6668-6692): an H2D STL with settings files that lack it, through
# --load-settings and through --machine/--filament/--process, gets what the
# named preset carries (on 2a12432: ";"). A 3MF that carries a value keeps it
# byte for byte, here one no computation gives.
d4_ok=1
hdr() { py '
import re, sys
for line in open(sys.argv[1], errors="replace"):
    m = re.match(r"^; " + re.escape(sys.argv[2]) + r" = (.*)$", line.rstrip("\r\n"))
    if m:
        print(m.group(1).replace("\"", "")); break
' "$1" "$2"; }
flat_presets bambu d4h2d "Bambu Lab H2D 0.4 nozzle"
py '
import json
for k in ("machine", "process", "filament"):
    d = json.load(open("d4h2d-bambu-%s.json" % k)); d.pop("extruder_nozzle_stats", None)
    json.dump(d, open("d4h2d-bambu-%s.json" % k, "w"), indent=1)
'
run d4p "$B" cube.stl --printer-preset "Bambu Lab H2D 0.4 nozzle" --slice 1 --outputdir d4p/out --export-3mf d4.3mf
cp d4p/out/d4.3mf d4.3mf 2>/dev/null || true
want=$(hdr d4p/out/plate_1.gcode extruder_nozzle_stats)
[ "$(rc d4p)" = 0 ] && [ -n "$want" ] && [ "$want" != ";" ] || { show d4p; fail "D4 bambu: the named H2D preset gives extruder_nozzle_stats '$want' (exit $(rc d4p))"; d4_ok=0; }
run d4l "$B" cube.stl --load-settings "d4h2d-bambu-machine.json;d4h2d-bambu-process.json" \
    --load-filaments d4h2d-bambu-filament.json --slice 1 --outputdir d4l/out --export-settings d4l.json
run d4m "$B" cube.stl -o d4m.gcode --machine d4h2d-bambu-machine.json --filament d4h2d-bambu-filament.json \
    --process d4h2d-bambu-process.json
for n in l m; do
    g=d4l/out/plate_1.gcode; [ $n = m ] && g=d4m.gcode
    [ "$(rc d4$n)" = 0 ] && [ -s $g ] || { show d4$n; fail "D4 bambu: settings files (route $n) exit $(rc d4$n)"; d4_ok=0; continue; }
    got=$(hdr $g extruder_nozzle_stats)
    [ "$got" = "$want" ] || { fail "D4 bambu: settings files (route $n) give extruder_nozzle_stats '$got', the named preset '$want'"; d4_ok=0; }
done
py '
import json, sys
s = json.load(open("d4l.json"))["extruder_nozzle_stats"]
assert ";".join(s) == sys.argv[1], (s, sys.argv[1])
' "$want" || { fail "D4 bambu: --export-settings does not hold the computed extruder_nozzle_stats"; d4_ok=0; }
# The keep guard: the project's own value, which no computation gives.
py '
import json, zipfile
with zipfile.ZipFile("d4.3mf") as zin, zipfile.ZipFile("d4keep.3mf", "w", zipfile.ZIP_DEFLATED) as zout:
    for item in zin.infolist():
        data = zin.read(item.filename)
        if item.filename == "Metadata/project_settings.config":
            d = json.loads(data); d["extruder_nozzle_stats"] = ["Standard#1|High Flow#0", "Standard#1"]
            data = json.dumps(d, indent=4).encode()
        zout.writestr(item, data)
' || { fail "D4 bambu: no exported H2D project for the keep check"; d4_ok=0; }
run d4k "$B" d4keep.3mf --plate 1 -o d4k.gcode
got=$(hdr d4k.gcode extruder_nozzle_stats)
[ "$(rc d4k)" = 0 ] && [ "$got" = "Standard#1|High Flow#0;Standard#1" ] || { show d4k; fail "D4 bambu: a 3MF's own extruder_nozzle_stats became '$got' (exit $(rc d4k))"; d4_ok=0; }
[ $d4_ok = 1 ] && echo "PASS: D4 input without extruder_nozzle_stats gets the official CLI's value, a 3MF keeps its own (Bambu build)"

# D5: each plate prints its own per-layer custom G-code. The loader keys
# them by plate (bbs_3mf.cpp 3446/3474) and the Print reads the model's
# current plate (Print.cpp 517-518), which the official CLI sets per plate
# (BambuStudio.cpp 6493; OrcaSlicer.cpp 5617). A two-plate project with a
# custom line on each plate: the product's call (--plate N -o) and --slice 0
# print plate N's line on plate N only (on 2a12432 plate 2 printed plate 1's).
d5_ok=1
py '
import json
P = lambda n: {"plate_name": n, "need_arrange": False,
               "objects": [{"path": "cube.stl", "count": 1, "filaments": [1], "pos_x": [118], "pos_y": [118]}]}
json.dump({"plates": [P("p"), P("q")]}, open("d5.json", "w"))
'
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run d5mk-$e "$bin" --load-assemble-list d5.json --slice 0 --printer-preset "Bambu Lab X1 Carbon 0.4 nozzle" \
        --outputdir d5mk-$e/out --export-3mf d5.3mf
    [ "$(rc d5mk-$e)" = 0 ] || { show d5mk-$e; fail "D5 $e: two-plate project exit $(rc d5mk-$e)"; d5_ok=0; continue; }
    py '
import sys, zipfile
src, dst = sys.argv[1], sys.argv[2]
xml = """<?xml version="1.0" encoding="utf-8"?>
<custom_gcodes_per_layer>
<plate>
<plate_info id="1"/>
<layer top_z="5" type="4" extruder="1" color="" extra="M117 D5 plate one" gcode="custom"/>
<mode value="SingleExtruder"/>
</plate>
<plate>
<plate_info id="2"/>
<layer top_z="10" type="4" extruder="1" color="" extra="M117 D5 plate two" gcode="custom"/>
<mode value="SingleExtruder"/>
</plate>
</custom_gcodes_per_layer>
"""
with zipfile.ZipFile(src) as zin, zipfile.ZipFile(dst, "w", zipfile.ZIP_DEFLATED) as zout:
    for item in zin.infolist():
        if item.filename == "Metadata/custom_gcode_per_layer.xml":
            continue
        zout.writestr(item, zin.read(item.filename))
    zout.writestr("Metadata/custom_gcode_per_layer.xml", xml)
' d5mk-$e/out/d5.3mf d5-$e.3mf
    for p in 1 2; do
        run d5p$p-$e "$bin" d5-$e.3mf --plate $p -o d5p$p-$e.gcode
    done
    run d5s-$e "$bin" d5-$e.3mf --slice 0 --outputdir d5s-$e/out
    for g in d5p1-$e.gcode:1 d5p2-$e.gcode:2 d5s-$e/out/plate_1.gcode:1 d5s-$e/out/plate_2.gcode:2; do
        f=${g%:*}; p=${g##*:}
        [ -s $f ] || { fail "D5 $e: no G-code $f"; d5_ok=0; continue; }
        own=one; other=two; [ $p = 2 ] && { own=two; other=one; }
        grep -q "^M117 D5 plate $own" $f || { fail "D5 $e: $f lacks plate $p's own custom G-code"; d5_ok=0; }
        ! grep -q "^M117 D5 plate $other" $f || { fail "D5 $e: $f prints the other plate's custom G-code"; d5_ok=0; }
    done
done
[ $d5_ok = 1 ] && echo "PASS: D5 each plate prints its own per-layer custom G-code, with --plate N and --slice 0 (both engines)"

# ---------------------------------------------------------------- Stage E
# E4: a setting with one value per plate or per nozzle is not padded with a 0
# to the extruder count (the official leaves each as given; get_at() reads a
# short list's first value, Config.hpp 681-685). A two-plate, two-filament H2D
# project whose wipe_tower_x/y hold one value: plate 2's tower stands where
# the official CLI puts it, not at (0, 0) (on 933ff72: wipe_tower_x = 165,0
# and the tower's first move near X21 Y20; the official near X40 Y244).
e4_ok=1
py '
import json
P = lambda n: {"plate_name": n, "need_arrange": True, "objects": [{"path": "cube.stl", "count": 2, "filaments": [1, 2]}]}
json.dump({"plates": [P("p"), P("q")]}, open("e4.json", "w"))
'
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run e4mk-$e "$bin" --load-assemble-list e4.json --slice 0 --printer-preset "Bambu Lab H2D 0.4 nozzle" \
        --filament-preset "Bambu PLA Basic @BBL H2D" --filament-preset "Bambu PLA Matte @BBL H2D" \
        --filament-colour "#FF0000;#00FF00" --outputdir e4mk-$e/out --export-3mf e4.3mf
    [ "$(rc e4mk-$e)" = 0 ] && [ -s e4mk-$e/out/e4.3mf ] || { show e4mk-$e; fail "E4 $e: two-plate H2D project exit $(rc e4mk-$e)"; e4_ok=0; continue; }
    py '
import json, sys, zipfile
with zipfile.ZipFile(sys.argv[1]) as zin, zipfile.ZipFile(sys.argv[2], "w", zipfile.ZIP_DEFLATED) as zout:
    for item in zin.infolist():
        data = zin.read(item.filename)
        if item.filename == "Metadata/project_settings.config":
            d = json.loads(data); d.update({"wipe_tower_x": ["165"], "wipe_tower_y": ["250"]})
            data = json.dumps(d, indent=4).encode()
        if item.filename.startswith("Metadata/plate_") and item.filename.endswith((".gcode", ".md5")):
            continue
        zout.writestr(item, data)
' e4mk-$e/out/e4.3mf e4-$e.3mf
    run e4p2-$e "$bin" e4-$e.3mf --plate 2 -o e4p2-$e.gcode
    [ "$(rc e4p2-$e)" = 0 ] && [ -s e4p2-$e.gcode ] || { show e4p2-$e; fail "E4 $e: plate 2 exit $(rc e4p2-$e)"; e4_ok=0; continue; }
    py '
import re, sys
g = open(sys.argv[1], errors="replace").read()
for k in ("wipe_tower_x", "wipe_tower_y"):
    for v in re.findall(r"^; " + k + r" = (.*)$", g, re.M):
        assert "," not in v, (k, v)
m = re.search(r"; FEATURE: (?:Prime|Wipe) tower\n(?:.*\n)*?G1 +X([0-9.]+) Y([0-9.]+)", g)
assert m and float(m.group(2)) > 100, ("tower first point", m.groups() if m else None)
' e4p2-$e.gcode || { fail "E4 $e: plate 2 tower or padded header (one value per plate)"; e4_ok=0; }
done
[ $e4_ok = 1 ] && echo "PASS: E4 one-value settings are not padded with 0; plate 2 tower stands where the official CLI puts it (both engines)"
