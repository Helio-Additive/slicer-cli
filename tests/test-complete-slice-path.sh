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
    if "$B" --info "$bad" > info-bad.json; then fail "--info accepted $bad"; fi
    py '
import json; d = json.load(open("info-bad.json"))
assert "error" in d and "3D/3dmodel.model" in d["error"], d
'
done
# A real STL is inspected; an unreadable one (a folder named .stl, or an
# empty file) is refused (-2).
"$B" --info cube.stl > info-stl.json || fail "--info refused cube.stl"
py '
import json; d = json.load(open("info-stl.json"))
assert d["kind"] == "stl" and "error" not in d, d
'

mkdir -p folder.stl
: > empty.stl
for bad in folder.stl empty.stl; do
    if "$B" --info "$bad" > info-bad.json; then fail "--info accepted $bad"; fi
    py '
import json; d = json.load(open("info-bad.json"))
assert "error" in d and "STL" in d["error"], d
'
done
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
