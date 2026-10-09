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
# The package's resources: beside the binary (the macOS and Windows release
# packages) or one level up (a bin/ layout, the box and Linux packages).
RES="$(cd "$(dirname "$B")" && pwd -P)/resources"
[ -d "$RES/profiles" ] || RES="$(cd "$(dirname "$B")/.." && pwd -P)/resources"
cp "$FIXTURE" "$WORKDIR/base.3mf"
FIXTURE=base.3mf
cd "$WORKDIR"

A1M="Bambu Lab A1 mini 0.4 nozzle"
A1M_PROCESS="0.20mm Standard @BBL A1M"
A1M_FILAMENT="Bambu PLA Basic @BBL A1M"

FAILS=0
# fail MESSAGE... : a failed check. The run stops there, unless E2E_CONTINUE=1,
# which records it and goes on -- for a corpus run that must not hide every
# check behind one known failure.
fail() { echo "FAIL: $*"; FAILS=$((FAILS + 1)); [ "${E2E_CONTINUE:-0}" = "1" ] || exit 1; }
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

# A 3MF member with a rooted path is skipped by the run's staging code: no
# folder is made for it (boost's path::operator/ drops the run's folder for a
# rooted member, so one would land outside it), an event names it, and the load
# goes on, as the desktop's does: both loaders extract only members under
# Metadata/ and Auxiliaries/ (bbs_3mf.cpp 2036-2054 at 5873b5f), and the
# OrcaSlicer one also skips an absolute path (is_path_within_root, 104-118 at
# 31f6803). After the '\' -> '/' replace, "C:/x", "//server/x" and "/x" are the
# shapes; a hostile archive is the point, so the members are crafted.
py '
import zipfile
def add(src, dst, name):
    with zipfile.ZipFile(src) as zin, zipfile.ZipFile(dst, "w", zipfile.ZIP_DEFLATED) as zout:
        for item in zin.infolist():
            zout.writestr(item, zin.read(item.filename))
        zout.writestr(name, "<x/>")
add("base.3mf", "rooted-drive.3mf", "C:/slicer-created/x")
add("base.3mf", "rooted-unc.3mf", "//server/share/x")
add("base.3mf", "rooted-root.3mf", "/slicer-created-root/x")
'
rooted_before=$FAILS
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    for pair in "drive:C:/slicer-created/x" "unc://server/share/x" "root:/slicer-created-root/x"; do
        kind=${pair%%:*}; member=${pair#*:}
        n=rooted-$kind-$e
        run $n "$bin" rooted-$kind.3mf --slice 1 --outputdir $n/out
        [ "$(rc $n)" = 0 ] && [ -s $n/out/plate_1.gcode ] || { show $n; fail "$e: the 3MF with the rooted member $member did not slice"; }
        # The member is named inside python from its kind: on Windows git-bash
        # rewrites a "/x" argument into C:/Program Files/Git/x before python
        # sees it.
        py '
import json, sys
member = {"drive": "C:/slicer-created/x", "unc": "//server/share/x", "root": "/slicer-created-root/x"}[sys.argv[2]]
events = [json.loads(l[len("[[SLICER_EVENT]] "):]) for l in open(sys.argv[1], errors="replace") if l.startswith("[[SLICER_EVENT]] ")]
assert any(e.get("tag") == "ThreeMfMemberSkipped" and e.get("member") == member for e in events), member
' $n/stdout "$kind" || { show $n; fail "$e: no ThreeMfMemberSkipped event naming $member"; }
        py '
import os, sys, tempfile
bad = []
for root, depth in ((".", 6), (tempfile.gettempdir(), 3)):
    base = os.path.abspath(root)
    for dirpath, dirnames, _ in os.walk(base):
        if dirpath[len(base):].count(os.sep) > depth:
            dirnames[:] = []
            continue
        for d in dirnames:
            if d.lower() in ("slicer-created", "slicer-created-root", "c:", "server"):
                bad.append(os.path.join(dirpath, d))
for p in ("/slicer-created-root", "C:/slicer-created", "C:/slicer-created-root"):
    if os.path.exists(p):
        bad.append(p)
assert not bad, bad
' || fail "$e: a folder of the rooted member $member was made"
    done
done
[ "$FAILS" = "$rooted_before" ] && echo "PASS: a rooted 3MF member is skipped, named in an event, and the project still slices (both engines)"

# A member whose folder name only holds two dots (Metadata/rev..1/data) stays
# inside the run's folder: only a ".." component leaves it, as the OrcaSlicer
# loader's is_path_within_root reads it (bbs_3mf.cpp 112-117 at 31f6803). Its
# folder is made like any other, and the project slices.
py '
import zipfile
with zipfile.ZipFile("base.3mf") as zin, zipfile.ZipFile("dots.3mf", "w", zipfile.ZIP_DEFLATED) as zout:
    for item in zin.infolist():
        zout.writestr(item, zin.read(item.filename))
    zout.writestr("Metadata/rev..1/data", "<x/>")
'
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run dots-$e "$bin" dots.3mf --slice 1 --outputdir dots-$e/out
    [ "$(rc dots-$e)" = 0 ] && [ -s dots-$e/out/plate_1.gcode ] || { show dots-$e; fail "$e: the 3MF with the member Metadata/rev..1/data did not slice"; }
    ! grep -q '"tag":"ThreeMfMemberSkipped"' dots-$e/stdout || fail "$e: Metadata/rev..1/data was named as skipped"
done
echo "PASS: a 3MF member whose folder name holds two dots loads (both engines)"

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

# A project that states no filament ids at all. filament_ids' own default is an
# empty list (PrintConfig.cpp 2851-2852 at 5873b5f, `new ConfigOptionStrings()`)
# and BambuStudio saves every project without it, so the option is there with no
# entry at all; the export step reads it at each used filament's index, and
# get_at() then reads values.front() of an empty vector (Config.hpp 681-684) —
# a null dereference. The desktop never meets that shape (PresetBundle fans
# every per-filament array out to the filament count), BambuStudio's own CLI
# segfaults on the crashing corpus project the same way. The four filament
# lists are that project's own, cut down to this fixture.
py '
import json, zipfile
with zipfile.ZipFile("base.3mf") as zin, zipfile.ZipFile("no-filament-ids.3mf", "w", zipfile.ZIP_DEFLATED) as zout:
    for item in zin.infolist():
        data = zin.read(item.filename)
        if item.filename == "Metadata/project_settings.config":
            d = json.loads(data)
            d.pop("filament_ids", None)
            d.update({"filament_colour": ["#3A3A3A", "#212721", "#212721", "#FFFFFF"],
                      "filament_type": ["PLA"] * 4,
                      "filament_settings_id": ["Bambu PLA Basic @BBL X1C"] * 4,
                      "filament_vendor": ["Bambu Lab"] * 4,
                      "filament_is_support": ["0"] * 4,
                      "filament_map": [1] * 4})
            data = json.dumps(d, indent=4).encode()
        zout.writestr(item, data)
'
run noids "$B" no-filament-ids.3mf --slice 1 --outputdir noids/out --export-3mf sliced.3mf
[ "$(rc noids)" = 0 ] || { show noids; fail "--export-3mf without filament_ids exit $(rc noids)"; }
# The plate's filament entry is the step that crashed: the project's own colour
# and type, and no tray id at all (the project states none).
py '
import re, zipfile
s = zipfile.ZipFile("noids/out/sliced.3mf").read("Metadata/slice_info.config").decode()
entry = re.search(r"<filament [^>]*/>", s)
assert entry, s
entry = entry.group(0)
assert "color=\"#3A3A3A\"" in entry, entry
assert "type=\"PLA\"" in entry, entry
assert "tray_info_idx=\"\"" in entry, entry
'
echo "PASS: --export-3mf survives a project that states no filament ids"

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

# --export-3mf may not name the run's own result.json or plate G-code, nor a
# name that differs from one only by letter case (on every system: two
# outputs of one run are not left to the file system's case rule).
run caseclash "$B" "$FIXTURE" --slice 1 --outputdir caseclash/out --export-3mf PLATE_1.gcode
[ "$(rc caseclash)" != 0 ] || fail "--export-3mf PLATE_1.gcode beside plate_1.gcode was accepted"
py '
import json; d = json.load(open("caseclash/out/result.json"))
assert d["return_code"] == -2 and "differs only by letter case from the run'"'"'s own plate_1.gcode" in d["error_string"], d
' || { show caseclash; fail "PLATE_1.gcode: no letter-case refusal"; }
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

# ... or the file another action of the run writes: --export-settings' file.
run setclash "$B" "$FIXTURE" --slice 1 --outputdir setclash/out --export-settings setclash/out/settings.json \
    --export-3mf settings.json
[ "$(rc setclash)" != 0 ] || fail "--export-3mf onto the --export-settings file was accepted"
grep -q '"tag":"ExportNameTaken"' setclash/stdout || { show setclash; fail "no ExportNameTaken event for the --export-settings file"; }
py '
import json; d = json.load(open("setclash/out/result.json"))
assert d["return_code"] == -2 and "--export-settings" in d["error_string"], d
'
echo "PASS: --export-3mf refuses the file --export-settings writes"

# Every output of a run is checked against the others, whether or not
# --export-3mf is given: --export-settings onto the run's own result.json
# (which the run writes last, over the settings) is refused before slicing.
run setres "$B" "$FIXTURE" --slice 1 --outputdir setres/out --export-settings setres/out/result.json
[ "$(rc setres)" != 0 ] || fail "--export-settings onto the run's own result.json was accepted"
[ ! -e setres/out/plate_1.gcode ] || fail "the run sliced before refusing --export-settings onto result.json"
py '
import json; d = json.load(open("setres/out/result.json"))
assert d["return_code"] == -2 and "--export-settings setres/out/result.json is the run'"'"'s own result.json in --outputdir" in d["error_string"], d
' || { show setres; fail "--export-settings onto result.json: no ExportNameTaken refusal naming both"; }
grep -q '"tag":"ExportNameTaken"' setres/stdout || { show setres; fail "no ExportNameTaken event for --export-settings onto result.json"; }
echo "PASS: --export-settings onto the run's own result.json is refused without --export-3mf"

# --export-slicedata into the folder --load-slicedata reads (or one inside the
# other) would write over the cache the run loads from; --export-stl and
# --export-stls into one folder write the same object STL files, the later
# action over the earlier. Both flags name a parent folder with one folder per
# plate under it: --export-slicedata sdsame/cache/1 writes plate 1 to
# sdsame/cache/1/1, inside sdsame/cache/1 that --load-slicedata reads.
run sdsame "$B" "$FIXTURE" --slice 1 --outputdir sdsame/out --load-slicedata sdsame/cache --export-slicedata sdsame/cache/1
[ "$(rc sdsame)" != 0 ] || fail "--export-slicedata inside the --load-slicedata folder was accepted"
grep -q '"tag":"ExportOverwritesInput"' sdsame/stdout || { show sdsame; fail "no ExportOverwritesInput event for --export-slicedata inside --load-slicedata"; }
grep -q 'would overwrite the folder --load-slicedata' sdsame/stderr || { show sdsame; fail "the refusal does not name --load-slicedata"; }
run stlboth "$B" cube.stl --printer-preset "$A1M" --outputdir stlboth --export-stl --export-stls stlboth/stl
[ "$(rc stlboth)" != 0 ] || fail "--export-stl and --export-stls into one folder were accepted"
grep -q '"tag":"ExportNameTaken"' stlboth/stdout || { show stlboth; fail "no ExportNameTaken event for --export-stl with --export-stls"; }
grep -q -- '--export-stl and --export-stls stlboth/stl write the same object STL file' stlboth/stderr ||
    { show stlboth; fail "the refusal does not name both STL flags"; }
echo "PASS: --export-slicedata over the --load-slicedata folder, and --export-stl with --export-stls into one folder, are refused"

# The outputs are checked once the model is loaded and before the first one is
# written: an action given before --slice that would write over the input
# project is refused with the project untouched, and the object STL files are
# the loaded objects' own names (a one-object cube writes only obj_1_*.stl, so
# another file in that folder is no clash; obj_1_cube.stl is).
cp "$FIXTURE" pre.3mf
py '
import hashlib; print(hashlib.md5(open("pre.3mf", "rb").read()).hexdigest())
' > pre.md5
run preset "$B" pre.3mf --export-settings pre.3mf --slice 1 --outputdir preset/out
[ "$(rc preset)" != 0 ] || fail "--export-settings onto the input project before --slice was accepted"
grep -q '"tag":"ExportOverwritesInput"' preset/stdout || { show preset; fail "no ExportOverwritesInput event for --export-settings onto the input project"; }
py '
import hashlib; assert hashlib.md5(open("pre.3mf", "rb").read()).hexdigest() == open("pre.md5").read().strip(), "pre.3mf changed"
' || fail "--export-settings before --slice wrote over the input project before refusing"
run stlnote "$B" cube.stl --printer-preset "$A1M" --outputdir stlnote --export-stl --export-settings stlnote/stl/obj_999_notes.stl
[ "$(rc stlnote)" = 0 ] || { show stlnote; fail "--export-stl with --export-settings stl/obj_999_notes.stl exit $(rc stlnote)"; }
ls stlnote/stl/obj_1_*.stl > /dev/null 2>&1 && [ -s stlnote/stl/obj_999_notes.stl ] ||
    fail "--export-stl and --export-settings stl/obj_999_notes.stl did not both write"
run stlclash "$B" cube.stl --printer-preset "$A1M" --outputdir stlclash --export-stl --export-settings stlclash/stl/obj_1_cube.stl
[ "$(rc stlclash)" != 0 ] || fail "--export-settings onto the object STL --export-stl writes was accepted"
grep -q '"tag":"ExportNameTaken"' stlclash/stdout || { show stlclash; fail "no ExportNameTaken event for the object STL clash"; }
[ ! -e stlclash/stl/obj_1_cube.stl ] || fail "the object STL clash was written before it was refused"
echo "PASS: the outputs are checked before the first write, with the loaded objects' own STL names"

# The STL export stages each file under a name of its own (a random part,
# created exclusively): a file kept beside the target under the old fixed
# staging name (<target>.writing) is neither taken nor removed.
mkdir -p stlkeep/stl
printf 'keep me\n' > stlkeep/stl/obj_1_cube.stl.writing
run stlkeep "$B" cube.stl --printer-preset "$A1M" --outputdir stlkeep --export-stl
[ "$(rc stlkeep)" = 0 ] && [ -s stlkeep/stl/obj_1_cube.stl ] || { show stlkeep; fail "--export-stl beside a kept .writing file exit $(rc stlkeep)"; }
[ "$(cat stlkeep/stl/obj_1_cube.stl.writing 2>/dev/null)" = "keep me" ] || fail "--export-stl took or removed the file kept as obj_1_cube.stl.writing"
[ "$(ls stlkeep/stl | grep -c '\.writing$')" = 1 ] || fail "--export-stl left a staging file behind: $(ls stlkeep/stl)"
echo "PASS: --export-stl stages under its own name and leaves a kept .writing file alone"

# The check before the first write reserves every plate G-code the run could
# write: for --slice 0, every plate of the project before its plan is final,
# so an action given before --slice that names one is refused before it runs.
run preplate "$B" "$FIXTURE" --export-settings preplate/out/plate_1.gcode --slice 0 --outputdir preplate/out
[ "$(rc preplate)" != 0 ] || fail "--export-settings onto plate_1.gcode with --slice 0 was accepted"
grep -q '"tag":"ExportNameTaken"' preplate/stdout || { show preplate; fail "no ExportNameTaken event for --export-settings onto plate_1.gcode"; }
[ ! -e preplate/out/plate_1.gcode ] || fail "--export-settings wrote plate_1.gcode before the refusal"
echo "PASS: --slice 0 reserves every plate's G-code before the first action writes"

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

# --export-3mf may not name a file the run reads: the official joins the name
# onto --outputdir (export_3mf_file = outfile_dir + "/" + export_3mf_file,
# BambuStudio.cpp 7508-7510 at 5873b5f; OrcaSlicer.cpp 6302-6304 at 31f6803) and
# writes wherever that lands, so `--export-3mf ../project.3mf` under a
# --outputdir inside the input's folder writes over the input project once the
# plate is sliced. That one name is refused; a name that lands on some other
# file is left as the official leaves it.
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    mkdir -p esc-$e/out/sub
    cp "$FIXTURE" esc-$e/project.3mf
    py '
import hashlib, sys; print(hashlib.md5(open(sys.argv[1], "rb").read()).hexdigest())
' esc-$e/project.3mf > esc-$e/md5.before
    run escin-$e "$bin" esc-$e/project.3mf --slice 1 --outputdir esc-$e/out --export-3mf ../project.3mf
    [ "$(rc escin-$e)" != 0 ] || fail "$e: --export-3mf ../project.3mf was accepted"
    [ ! -e esc-$e/out/plate_1.gcode ] || fail "$e: the run sliced a plate before refusing the export"
    grep -q '"tag":"ExportOverwritesInput"' escin-$e/stdout || { show escin-$e; fail "$e: no ExportOverwritesInput event"; }
    py '
import json, sys
d = json.load(open(sys.argv[1]))
assert d["return_code"] == -2, d
assert "would overwrite the input file" in d["error_string"], d
assert "esc-%s/project.3mf" % sys.argv[2] in d["error_string"], d
' esc-$e/out/result.json "$e" || { show escin-$e; fail "$e: no input-overwrite refusal in result.json"; }
    py '
import hashlib, sys
assert hashlib.md5(open(sys.argv[1], "rb").read()).hexdigest() == open(sys.argv[2]).read().strip()
' esc-$e/project.3mf esc-$e/md5.before || fail "$e: the refused run wrote over the input project"
    # A name outside --outputdir that is not an input still writes where the
    # official writes it (beside the folder it was given).
    run escout-$e "$bin" esc-$e/project.3mf --slice 1 --outputdir esc-$e/out --export-3mf ../escape.3mf
    [ "$(rc escout-$e)" = 0 ] || { show escout-$e; fail "$e: --export-3mf ../escape.3mf exit $(rc escout-$e)"; }
    py '
import sys, zipfile
n = zipfile.ZipFile(sys.argv[1]).namelist()
assert "Metadata/plate_1.gcode" in n, n
' esc-$e/escape.3mf || fail "$e: escape.3mf is not the sliced project"
    # ... and a name inside --outputdir still writes.
    run escok-$e "$bin" esc-$e/project.3mf --slice 1 --outputdir esc-$e/out --export-3mf sub/ok.3mf
    [ "$(rc escok-$e)" = 0 ] || { show escok-$e; fail "$e: --export-3mf sub/ok.3mf exit $(rc escok-$e)"; }
    py '
import sys, zipfile
n = zipfile.ZipFile(sys.argv[1]).namelist()
assert "Metadata/plate_1.gcode" in n, n
' esc-$e/out/sub/ok.3mf || fail "$e: sub/ok.3mf is not the sliced project"
    # The parts an assemble list names are inputs too: the list's loader read
    # them, and an export over one would replace the user's part. A real list
    # naming esc-<e>/part.stl; the export onto that part is refused, its md5
    # unchanged, and an export onto any other name still writes.
    cp cube.stl esc-$e/part.stl
    py '
import json, sys
json.dump({"plates": [{"plate_name": "p", "need_arrange": True,
                       "objects": [{"path": sys.argv[1], "count": 1, "filaments": [1]}]}]},
          open(sys.argv[2], "w"))
' esc-$e/part.stl esc-$e/list.json
    py '
import hashlib, sys; print(hashlib.md5(open(sys.argv[1], "rb").read()).hexdigest())
' esc-$e/part.stl > esc-$e/part.md5
    run escpart-$e "$bin" --load-assemble-list esc-$e/list.json --slice 1 --printer-preset "$A1M" \
        --outputdir esc-$e/out --export-3mf ../part.stl
    [ "$(rc escpart-$e)" != 0 ] || fail "$e: --export-3mf onto the assemble list's part was accepted"
    py '
import hashlib, json, sys
d = json.load(open(sys.argv[1]))
assert d["return_code"] == -2 and "would overwrite the input file" in d["error_string"] and "part.stl" in d["error_string"], d
assert hashlib.md5(open(sys.argv[2], "rb").read()).hexdigest() == open(sys.argv[3]).read().strip(), "the part changed"
' esc-$e/out/result.json esc-$e/part.stl esc-$e/part.md5 || { show escpart-$e; fail "$e: the export onto the assemble list's part was not refused, or the part changed"; }
    run escpartok-$e "$bin" --load-assemble-list esc-$e/list.json --slice 1 --printer-preset "$A1M" \
        --outputdir esc-$e/out --export-3mf ../listed.3mf
    [ "$(rc escpartok-$e)" = 0 ] && [ -s esc-$e/listed.3mf ] || { show escpartok-$e; fail "$e: an export beside the assemble list's part exit $(rc escpartok-$e)"; }
    # ../PART.STL is the part itself where the file system ignores case
    # (Windows, macOS) and another file where it does not (Linux).
    run escpartcase-$e "$bin" --load-assemble-list esc-$e/list.json --slice 1 --printer-preset "$A1M" \
        --outputdir esc-$e/out --export-3mf ../PART.STL
    case "$(uname -s)" in
    MINGW*|MSYS*|CYGWIN*|Darwin)
        [ "$(rc escpartcase-$e)" != 0 ] || fail "$e: --export-3mf ../PART.STL onto part.stl was accepted"
        grep -q '"tag":"ExportOverwritesInput"' escpartcase-$e/stdout || { show escpartcase-$e; fail "$e: ../PART.STL: no ExportOverwritesInput event"; };;
    *)
        [ "$(rc escpartcase-$e)" = 0 ] || { show escpartcase-$e; fail "$e: --export-3mf ../PART.STL beside part.stl on a case-sensitive file system exit $(rc escpartcase-$e)"; }
        py '
import sys, zipfile
assert "Metadata/plate_1.gcode" in zipfile.ZipFile(sys.argv[1]).namelist()
' esc-$e/PART.STL || fail "$e: ../PART.STL is not the sliced project";;
    esac
    py '
import hashlib, sys
assert hashlib.md5(open(sys.argv[1], "rb").read()).hexdigest() == open(sys.argv[2]).read().strip(), "part.stl changed"
' esc-$e/part.stl esc-$e/part.md5 || fail "$e: ../PART.STL changed the assemble list's part.stl"
    # The files a loader reads beside a model are inputs too: the .mtl an
    # OBJ's mtllib names (load_obj, both engines), for an OBJ model file and
    # for an OBJ the assemble list names. The export onto it is refused and
    # the .mtl is left as it was.
    mkdir -p esc-$e/lp/out
    py '
import sys
v = [(x, y, z) for x in (0, 20) for y in (0, 20) for z in (0, 20)]
f = [(1,3,4),(1,4,2),(5,6,8),(5,8,7),(1,2,6),(1,6,5),(3,7,8),(3,8,4),(1,5,7),(1,7,3),(2,4,8),(2,8,6)]
for out in sys.argv[1:]:
    with open(out, "w") as o:
        o.write("mtllib colors.mtl\n")
        for p in v: o.write("v %d %d %d\n" % p)
        for t in f: o.write("f %d %d %d\n" % t)
' esc-$e/top.obj esc-$e/lp/part.obj
    for m in esc-$e/colors.mtl esc-$e/lp/colors.mtl; do
        printf 'newmtl red\nKd 1 0 0\n' > $m
        py '
import hashlib, sys; print(hashlib.md5(open(sys.argv[1], "rb").read()).hexdigest())
' $m > $m.md5
    done
    py '
import json, sys
json.dump({"plates": [{"plate_name": "p", "need_arrange": True,
                       "objects": [{"path": sys.argv[1], "count": 1, "filaments": [1]}]}]},
          open(sys.argv[2], "w"))
' esc-$e/lp/part.obj esc-$e/lp/list.json
    run escmtl-$e "$bin" esc-$e/top.obj --slice 1 --printer-preset "$A1M" --outputdir esc-$e/out --export-3mf ../colors.mtl
    run escmtllist-$e "$bin" --load-assemble-list esc-$e/lp/list.json --slice 1 --printer-preset "$A1M" \
        --outputdir esc-$e/lp/out --export-3mf ../colors.mtl
    for c in escmtl:esc-$e escmtllist:esc-$e/lp; do
        n=${c%%:*}-$e; d=${c#*:}; m=$d/colors.mtl
        [ "$(rc $n)" != 0 ] || fail "$e: $n: --export-3mf onto $m was accepted"
        grep -q '"tag":"ExportOverwritesInput"' $n/stdout || { show $n; fail "$e: $n: no ExportOverwritesInput event"; }
        py '
import hashlib, json, sys
d = json.load(open(sys.argv[1]))
assert d["return_code"] == -2 and "would overwrite the input file" in d["error_string"] and "colors.mtl" in d["error_string"], d
assert hashlib.md5(open(sys.argv[2], "rb").read()).hexdigest() == open(sys.argv[2] + ".md5").read().strip(), "colors.mtl changed"
' $d/out/result.json $m || { show $n; fail "$e: $n: the export onto $m was not refused, or the .mtl changed"; }
    done
    # BambuStudio reads a glTF through Assimp, which opens the buffer file the
    # glTF names: that file is an input as well.
    if [ $e = bambu ]; then
        mkdir -p esc-$e/gl/out
        py '
import json, struct, sys
v = [(x, y, z) for x in (0, 20) for y in (0, 20) for z in (0, 20)]
f = [(0,2,3),(0,3,1),(4,5,7),(4,7,6),(0,1,5),(0,5,4),(2,6,7),(2,7,3),(0,4,6),(0,6,2),(1,3,7),(1,7,5)]
pos = b"".join(struct.pack("<3f", *p) for p in v)
idx = b"".join(struct.pack("<3H", *t) for t in f)
open(sys.argv[2], "wb").write(pos + idx)
json.dump({"asset": {"version": "2.0"}, "scene": 0, "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0}],
           "meshes": [{"primitives": [{"attributes": {"POSITION": 0}, "indices": 1}]}],
           "buffers": [{"uri": "mesh.bin", "byteLength": len(pos) + len(idx)}],
           "bufferViews": [{"buffer": 0, "byteOffset": 0, "byteLength": len(pos)},
                           {"buffer": 0, "byteOffset": len(pos), "byteLength": len(idx)}],
           "accessors": [{"bufferView": 0, "componentType": 5126, "count": 8, "type": "VEC3",
                          "min": [0, 0, 0], "max": [20, 20, 20]},
                         {"bufferView": 1, "componentType": 5123, "count": 36, "type": "SCALAR"}]},
          open(sys.argv[1], "w"))
print(__import__("hashlib").md5(pos + idx).hexdigest())
' esc-$e/gl/cube.gltf esc-$e/gl/mesh.bin > esc-$e/gl/mesh.bin.md5
        run escbin-$e "$bin" esc-$e/gl/cube.gltf --slice 1 --printer-preset "$A1M" --outputdir esc-$e/gl/out --export-3mf ../mesh.bin
        [ "$(rc escbin-$e)" != 0 ] || fail "$e: --export-3mf onto the glTF's buffer was accepted"
        py '
import hashlib, json, sys
d = json.load(open(sys.argv[1]))
assert d["return_code"] == -2 and "would overwrite the input file" in d["error_string"] and "mesh.bin" in d["error_string"], d
assert hashlib.md5(open(sys.argv[2], "rb").read()).hexdigest() == open(sys.argv[2] + ".md5").read().strip(), "mesh.bin changed"
' esc-$e/gl/out/result.json esc-$e/gl/mesh.bin || { show escbin-$e; fail "$e: the export onto the glTF's buffer was not refused, or the buffer changed"; }
    fi
    # Names that are no input still write as the official writes them,
    # whatever their extension.
    for x in x.3MF out.gcode; do
        run escname-$e "$bin" esc-$e/project.3mf --slice 1 --outputdir esc-$e/out --export-3mf ../$x
        [ "$(rc escname-$e)" = 0 ] || { show escname-$e; fail "$e: --export-3mf ../$x exit $(rc escname-$e)"; }
        py '
import sys, zipfile
assert "Metadata/plate_1.gcode" in zipfile.ZipFile(sys.argv[1]).namelist()
' esc-$e/$x || fail "$e: ../$x is not the sliced project"
    done
done
echo "PASS: --export-3mf refuses a name that would overwrite an input, the parts an assemble list names and the .mtl an OBJ reads included; other names write as the official writes them (both engines)"

# The plates a run writes are only known after the global arrange of
# --slice 0 --arrange 1, which can add one (BambuStudio.cpp 5627-5722;
# OrcaSlicer.cpp 4887-4983): the collision check must read the final plate
# plan, or --export-3mf plate_3.gcode passes on a 2-plate project that
# arranges into three plates and the export overwrites the plate_3.gcode the
# slice pass wrote (the run's own G-code replaced by 3MF bytes).
py '
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
box("part150.stl", 150, 20)
import json
json.dump({"plates": [{"plate_name": "p%d" % i, "need_arrange": True,
                       "objects": [{"path": "part150.stl", "count": 1, "filaments": [1]}]}
                      for i in (1, 2, 3)]}, open("grow3.json", "w"))
'
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    # One 150 mm part per plate on the A1 mini: the arrange cannot fit two on
    # one 180 mm bed, so it keeps three plates.
    run growfx-$e "$bin" --load-assemble-list grow3.json --slice 0 --printer-preset "$A1M" \
        --outputdir growfx-$e/out --export-3mf grow3.3mf
    [ "$(rc growfx-$e)" = 0 ] || { show growfx-$e; fail "$e: the three-plate fixture exit $(rc growfx-$e)"; }
    # The same project with the third plate's instance moved onto plate 2: two
    # plates on paper, three after the arrange.
    py '
import re, sys, zipfile
zin = zipfile.ZipFile(sys.argv[1])
s = zin.read("Metadata/model_settings.config").decode()
plates = re.findall(r"  <plate>.*?  </plate>\n", s, re.S)
assert len(plates) == 3, len(plates)
moved = re.findall(r"    <model_instance>.*?    </model_instance>\n", plates[2], re.S)
two = s.replace(plates[1], plates[1].replace("  </plate>\n", "".join(moved) + "  </plate>\n")).replace(plates[2], "")
assert len(re.findall(r"<plate>", two)) == 2 and len(re.findall(r"<model_instance>", two)) == 3, two
with zipfile.ZipFile(sys.argv[2], "w", zipfile.ZIP_DEFLATED) as zout:
    for item in zin.infolist():
        if item.filename == "Metadata/model_settings.config": data = two.encode()
        elif item.filename == "Metadata/plate_3.json": continue
        else: data = zin.read(item.filename)
        zout.writestr(item, data)
' growfx-$e/out/grow3.3mf grow2-$e.3mf || fail "$e: the 2-plate fixture could not be built"
    run grow2-$e "$bin" grow2-$e.3mf --slice 0 --arrange 1 --outputdir grow2-$e/out --export-3mf plate_3.gcode
    [ "$(rc grow2-$e)" != 0 ] || fail "$e: --export-3mf plate_3.gcode was accepted after the arrange grew the project to three plates"
    grep -q '"tag":"ExportNameTaken"' grow2-$e/stdout || { show grow2-$e; fail "$e: no ExportNameTaken event"; }
    py '
import json, sys; d = json.load(open(sys.argv[1]))
assert d["return_code"] == -2 and "plate_3.gcode" in d["error_string"], d
' grow2-$e/out/result.json || { show grow2-$e; fail "$e: the refusal left no result.json with -2"; }
    [ ! -e grow2-$e/out/plate_3.gcode ] || fail "$e: the refused run still wrote a plate_3.gcode"
    grep -q '"plate_count":3' grow2-$e/stdout || { show grow2-$e; fail "$e: the arrange did not grow the 2-plate project to three plates"; }
done
echo "PASS: --export-3mf refuses a plate the global arrange adds (both engines)"

# A command-line override is range-checked like the file's values.
run badlh "$B" "$FIXTURE" --slice 1 --layer-height -0.1 --outputdir badlh/out --export-3mf sliced.3mf
[ "$(rc badlh)" != 0 ] || fail "--layer-height -0.1 was sliced"
py '
import json; d = json.load(open("badlh/out/result.json"))
assert d["return_code"] != 0 and "layer_height" in d["error_string"], d
'
echo "PASS: a bad override is refused before slicing"

# A refusal writes result.json only for a --slice the parse itself would read:
# not after the "--" terminator, not as the value of another flag.
run preok "$B" --bad --slice=1 --outputdir preok/out
[ "$(rc preok)" != 0 ] || fail "--bad was accepted"
[ -f preok/out/result.json ] || { show preok; fail "a refused --slice run wrote no result.json"; }
run preend "$B" --bad -- --slice=1 --outputdir preend/out
[ "$(rc preend)" != 0 ] || fail "--bad was accepted"
[ ! -e preend/out/result.json ] || fail "--slice after the -- terminator wrote result.json"
run preval "$B" --bad --export-settings --slice=1 --outputdir preval/out
[ "$(rc preval)" != 0 ] || fail "--bad was accepted"
[ ! -e preval/out/result.json ] || fail "--slice given as --export-settings' value wrote result.json"
echo "PASS: a refusal reads --slice and --outputdir as the parse does"

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

# A Bambu Studio file is compared with the Bambu base this Orca release is built
# on (SLIC3R_VERSION, 02.06.00.51), as the desktop does (Plater.cpp 6094-6157),
# so a Bambu Studio 2.5 project is not "newer" here and slices; only a file
# OrcaSlicer made is compared with this engine's own version.
run orca-newer "$O" "$FIXTURE" --plate 1 -o orca-newer/out.gcode
[ "$(rc orca-newer)" = 0 ] || { show orca-newer; fail "Orca refused a Bambu Studio file that is not newer than its Bambu base"; }
echo "PASS: a Bambu Studio project is measured against the Bambu base, not against the Orca version"

# Cross-engine values on the Orca build: a value this engine has no meaning for
# is refused, naming it and what to use instead.
py '
import json, zipfile
with zipfile.ZipFile("base.3mf") as zin, zipfile.ZipFile("partial.3mf", "w", zipfile.ZIP_DEFLATED) as zout:
    for item in zin.infolist():
        data = zin.read(item.filename)
        if item.filename == "Metadata/project_settings.config":
            d = json.loads(data)
            d["ensure_vertical_shell_thickness"] = "partial"
            d["different_settings_to_system"] = ["ensure_vertical_shell_thickness", "", ""]
            data = json.dumps(d, indent=4).encode()
        zout.writestr(item, data)
' || fail "orca: the partial-shell fixture could not be written"
mkdir -p orca-values/out
echo "; stale" > orca-values/out/plate_1.gcode
run orca-values "$O" partial.3mf --slice 1 --outputdir orca-values/out
[ "$(rc orca-values)" != 0 ] || fail "Orca sliced a file with a value it does not have"
# A G-code left from an earlier run in the same --outputdir must not survive a failed plate.
[ ! -e orca-values/out/plate_1.gcode ] || fail "a stale plate_1.gcode survived a failed plate"
grep -q 'ensure_vertical_shell_thickness' orca-values/stderr || { show orca-values; fail "no refusal naming the value"; }
grep -qi 'pick the shell coverage' orca-values/stderr || { show orca-values; fail "the refusal does not say what to do"; }
echo "PASS: Orca refuses a value it has no meaning for, naming it and what to use instead"

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

# --repetitions on one plate of a multi-plate project. The search for a count
# that fits retries, and a retry used to put the saved model back with a Model
# assignment: that frees every object and instance the model holds, while the
# plate's own scope (main.cpp PlateScope) and the run's plates still name them —
# the scope wrote through the freed objects on its way out, and the other
# plates were left with no members. The search now runs on a copy of the model
# (cli_repetitions.cpp), so nothing the run still names is freed.
#
# A real multi-plate project, external like the other corpus case: the run goes
# where it is, and skips where it is not.
REPS="${SLICER_CLI_CORPUS:-$HOME/engcheck-plugin/corpus}/feb807b138a8ddfa.3mf"
if [ ! -f "$REPS" ]; then
    echo "SKIP: --repetitions on a plate of a multi-plate project (no $REPS)"
else
    for e in bambu orca; do
        bin=$B; [ $e = orca ] && bin=$O
        # Plate 3 of the project's three (3 objects, with 22 and 6 on the other
        # two, so the plate is not the whole model and the count cannot fit.
        # The project states raft_first_layer_expansion: -1, which the OrcaSlicer
        # engine refuses as out of range (-18, naming the flag); the override is
        # the flag it asks for.
        run multirep-$e "$bin" "$REPS" --slice 3 --repetitions 60 --allow-newer-file \
            --raft-first-layer-expansion 0 --outputdir multirep-$e/out
        [ "$(rc multirep-$e)" = 0 ] || { show multirep-$e; fail "$e: --repetitions 60 on plate 3 of a three-plate project exit $(rc multirep-$e)"; }
        grep -q '"tag":"RepetitionsPlaced"' multirep-$e/stdout || { show multirep-$e; fail "$e: no RepetitionsPlaced event"; }
        py '
import json, sys
d = json.load(open(sys.argv[1]))
assert d["return_code"] == 0, d
plates = d["sliced_plates"]
assert len(plates) == 1 and plates[0]["id"] == 3, plates
assert plates[0]["objects"], plates[0]
' multirep-$e/out/result.json || { show multirep-$e; fail "$e: the plate came out of the repetitions search empty"; }
    done
    echo "PASS: --repetitions on one plate of a multi-plate project leaves it whole (both engines)"
fi

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

# A fixed-layout assemble-list plate of two filaments gets the desktop's own
# tower position: PartPlateList::set_default_wipe_tower_pos_for_plate clamps
# the default corner inside the plate, narrowed by the extruder areas, for a
# tower sized for two extruders (Orca PartPlate.cpp 4115-4190; Bambu
# PartPlate.cpp 4615-4702). The official CLI's corner alone leaves it at
# (165, 250) and the tower's own moves go past the 256 bed: the Orca run is
# refused -104 "Found G-code outside of the printable area ... It comes from
# the prime tower".
py '
import json
json.dump({"plates": [{"plate_name": "tower", "need_arrange": False,
                       "objects": [{"path": "cube.stl", "count": 1, "filaments": [1], "pos_x": [100], "pos_y": [150]},
                                   {"path": "cube.stl", "count": 1, "filaments": [2], "pos_x": [140], "pos_y": [150]}]}]},
          open("altow.json", "w"))
'
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    # Two runs: the command with no tower flag at all, and the same with the
    # tower asked for, because the Bambu build prints no tower for two
    # filaments of one colour (PrimeTowerOffOneFilament) and its tower is the
    # geometry this check measures.
    for c in "plain:" "tower:--enable-prime-tower"; do
    n=${c%%:*}; flag=${c#*:}
    run altow$n-$e "$bin" --load-assemble-list altow.json --slice 1 --printer-preset "$X1C" \
        --filament-preset "Bambu PLA Basic @BBL X1C" --filament-preset "Bambu PLA Basic @BBL X1C" \
        $flag --outputdir altow$n-$e/out
    [ "$(rc altow$n-$e)" = 0 ] || { show altow$n-$e; fail "$e ($n): two filaments on a fixed plate exit $(rc altow$n-$e)"; }
    [ -s altow$n-$e/out/plate_1.gcode ] || fail "$e ($n): the fixed-layout plate produced no G-code"
    py '
import re, sys
g = open(sys.argv[1], errors="replace").read().replace("\r", "")
x = float(re.search(r"^; wipe_tower_x = ([\d.eE+-]+)", g, re.M).group(1))
y = float(re.search(r"^; wipe_tower_y = ([\d.eE+-]+)", g, re.M).group(1))
assert 0 <= x < 256 and 0 <= y < 256, ("tower corner off the 256 bed", x, y)
# The tower and its depth, and every object, inside the 256 mm bed: the
# extrusion moves the G-code prints are the geometry itself.
moves = [(float(a), float(b)) for a, b in re.findall(r"^G1 X([\d.-]+) Y([\d.-]+) E", g, re.M)]
assert moves, "no extrusion moves"
xs = [p[0] for p in moves]; ys = [p[1] for p in moves]
assert max(xs) <= 256 and max(ys) <= 256, ("printed outside the bed", max(xs), max(ys))
assert min(xs) >= 0 and min(ys) >= 0, ("printed outside the bed", min(xs), min(ys))
if re.search(r"^; enable_prime_tower = 1", g, re.M):
    # The fixture objects stop at y 170, so the moves above it are the tower.
    tower = [v for v in ys if v > 200]
    assert tower and min(tower) >= y, ("tower not at its corner", y, min(tower))
' altow$n-$e/out/plate_1.gcode || { show altow$n-$e; fail "$e ($n): the fixed-layout plate tower is not inside the 256 bed"; }
    done
done
echo "PASS: an assemble-list plate with a fixed layout puts its tower inside the bed (both engines)"

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

# The Orca build's printer change takes the process the DESKTOP selects for the
# new printer, not the project's own: OrcaSlicer ships no process_full folder
# for the branch the official CLI takes here (OrcaSlicer.cpp 3003-3011 reads
# resources/orca/profiles/BBL/process_full/), while the desktop switches
# printers from the presets its package ships (PresetBundle::update_compatible,
# PresetBundle.cpp 5295-5330; Tab::select_preset, Tab.cpp 6130-6140 at
# 31f6803). The Bambu build keeps the official path: its package ships
# process_full and no value of its runs is checked here.
ORCA_PROFILES="$(cd "$(dirname "$O")" && pwd -P)/resources/profiles-orca"
[ -d "$ORCA_PROFILES/BBL/process" ] || ORCA_PROFILES="$(cd "$(dirname "$O")/.." && pwd -P)/resources/profiles-orca"
# The shipped presets flattened over their inherits chains, for the EXPECTED
# values below only: the runs themselves pass the files the package ships, or
# the preset names, never a flattened copy (a shipped file holds only its
# differences from its parents, and the engines read it over them). The oracle
# is the two processes the checks below expect.
A1M_MACHINE="$ORCA_PROFILES/BBL/machine/Bambu Lab A1 mini 0.4 nozzle.json"
[ -f "$A1M_MACHINE" ] || fail "orca: the package ships no A1 mini machine preset at $A1M_MACHINE"
py '
import json, os, sys
vendor = sys.argv[1]
def flat(kind, name):
    profiles = {}
    for root, _, files in os.walk(os.path.join(vendor, kind)):
        for f in files:
            if f.endswith(".json"):
                d = json.load(open(os.path.join(root, f), encoding="utf-8"))
                profiles[d.get("name", f[:-5])] = d
    def resolve(n, seen=()):
        d = profiles[n]
        r = resolve(d["inherits"], seen + (n,)) if d.get("inherits") and n not in seen else {}
        r.update({k: v for k, v in d.items() if k != "inherits"})
        return r
    return resolve(name)
for out, name in (("a1m-p020.json", "0.20mm Standard @BBL A1M"), ("a1m-p016.json", "0.16mm Optimal @BBL A1M")):
    json.dump(flat("process", name), open(out, "w", encoding="utf-8"), indent=1)
' "$ORCA_PROFILES/BBL" || fail "orca: the shipped A1 mini processes could not be flattened"

# The fifteen printer-change cases above switch with an --export-settings dump
# as the machine file; the process must still be the A1 mini's, not the X1
# Carbon project's.
py '
import json, re, sys
want = json.load(open(sys.argv[2])); g = open(sys.argv[1], errors="replace").read()
def got(k):
    m = re.search(r"^; " + k + r" = (.*)$", g, re.M)
    return None if m is None else m.group(1).split(",")[0].strip()
def first(v):
    return re.sub(r"\"", "", str(v[0] if isinstance(v, list) else v))
bad = []
for k in ("layer_height", "wall_loops", "sparse_infill_density", "travel_speed",
          "default_acceleration", "bridge_speed", "elefant_foot_compensation"):
    if got(k) != first(want[k]):
        bad.append("%s: G-code %r, the A1 mini process %r" % (k, got(k), first(want[k])))
assert not bad, "; ".join(bad)
' sw-orca/out/plate_1.gcode a1m-p020.json || fail "orca: the printer change did not slice with the A1 mini process settings the package ships"
echo "PASS: the Orca printer change slices with the process the desktop selects for the new printer"

# The desktop's keep-or-replace choice reads the printers the project's process
# says it suits, which a project 3MF carries as print_compatible_printers
# (full_fff_config erases compatible_printers, PresetBundle.cpp 4084 and 4129,
# and load_config_file_config puts that list back, 4390-4393 at 31f6803), and it
# then takes the preset whose alias is the current preset's before any other
# (PreferedProfileMatch, PresetBundle.cpp 5196-5207; a system preset's alias is
# its name up to the "@", 5013-5027), so its layer height survives. An X1
# Carbon project at 0.16mm Optimal moved to the A1 mini with a machine file
# holding printer keys only becomes 0.16mm Optimal @BBL A1M: not the printer's
# default 0.20mm Standard, and not the 0.16mm High Quality the vendor list
# holds before it.
run sw16p "$O" cube.stl --slice 1 --printer-preset "Bambu Lab X1 Carbon 0.4 nozzle" \
    --process-preset "0.16mm Optimal @BBL X1C" --outputdir sw16p/out --export-3mf sw16.3mf
[ "$(rc sw16p)" = 0 ] || { show sw16p; fail "orca: the 0.16mm Optimal X1 Carbon project exit $(rc sw16p)"; }
run sw16 "$O" sw16p/out/sw16.3mf --slice 1 --load-settings "$A1M_MACHINE" --outputdir sw16/out
[ "$(rc sw16)" = 0 ] || { show sw16; fail "orca: the 0.16mm printer change with a printer-keys-only machine file exit $(rc sw16)"; }
py '
import json, re, sys
want = json.load(open(sys.argv[2])); g = open(sys.argv[1], errors="replace").read()
def got(k):
    m = re.search(r"^; " + k + r" = (.*)$", g, re.M)
    return None if m is None else m.group(1).split(",")[0].strip()
def first(v):
    return re.sub(r"\"", "", str(v[0] if isinstance(v, list) else v))
bad = []
for k in ("layer_height", "wall_loops", "sparse_infill_density", "travel_speed",
          "default_acceleration", "bridge_speed", "elefant_foot_compensation"):
    if got(k) != first(want[k]):
        bad.append("%s: G-code %r, the A1 mini process %r" % (k, got(k), first(want[k])))
# The selected preset names itself (full_fff_config, PresetBundle.cpp 4106).
if got("print_settings_id") != "0.16mm Optimal @BBL A1M":
    bad.append("print_settings_id: G-code %r" % got("print_settings_id"))
assert not bad, "; ".join(bad)
' sw16/out/plate_1.gcode a1m-p016.json || fail "orca: the printer change did not take the alias-matching A1 mini process"
echo "PASS: the Orca printer change keeps the project's process recipe (0.16mm Optimal) and names the preset it took"

# The process list to search is the NEW printer's vendor's: the desktop tags
# every loaded preset with the vendor bundle it came from (PresetBundle.cpp
# 4992), a printer model resolves to its vendor through that bundle's
# machine_list (PresetBundle.cpp 617-622), and PresetBundle::find_preset_vendor
# reads the same list (OrcaSlicer PresetBundle.cpp 244-318 at 31f6803). With
# the project's vendor passed instead, an X1 Carbon project switched to a
# Snapmaker machine preset never looks at the Snapmaker process list, finds no
# process that suits "Snapmaker U1 (0.4 nozzle)" and keeps the BBL process.
py '
import json, os, sys
vendor = sys.argv[1]
META = {"name", "inherits", "include", "from", "type", "instantiation",
        "setting_id", "filament_id", "description"}
def load(kind):
    out = {}
    for root, _, files in os.walk(os.path.join(vendor, kind)):
        for f in files:
            if f.endswith(".json"):
                d = json.load(open(os.path.join(root, f), encoding="utf-8"))
                out[d.get("name", f[:-5])] = d
    return out
P = {k: load(k) for k in ("machine", "process", "filament")}
def flat(kind, name, seen=()):
    d = P[kind][name]
    r = flat(kind, d["inherits"], seen + (name,)) if d.get("inherits") and name not in seen else {}
    for inc in d.get("include", []) or []:
        t = flat(kind, inc, seen + (name,))
        r.update({k: v for k, v in t.items() if k not in META})
    r.update({k: v for k, v in d.items() if k != "include"})
    r.pop("inherits", None)
    return r
m = flat("machine", "Snapmaker U1 (0.4 nozzle)")
m.update({"type": "machine", "from": "system", "name": "Snapmaker U1 (0.4 nozzle)", "instantiation": "true"})
assert "layer_height" not in m, sorted(m)
# The pick the desktop makes: first_compatible_idx over the vendor list with
# PreferedPrintProfileMatch (Preset.hpp 686-709, PresetBundle.cpp 5196-5247),
# skipping presets a user cannot instantiate (Preset.cpp 1620). The project
# process is 0.20mm Standard @BBL X1C, layer height 0.2; no Snapmaker process
# carries that alias, so the first process of the list that suits the U1 at
# this layer height wins.
def first(v):
    return v[0] if isinstance(v, list) else v
order = json.load(open(os.path.join(vendor, "..", "Snapmaker.json")))["process_list"]
best, bestq = None, -1
for entry in order:
    n = entry["name"]
    d = P["process"].get(n)
    if not d or d.get("instantiation") == "false":
        continue
    c = flat("process", n)
    if "Snapmaker U1 (0.4 nozzle)" not in (c.get("compatible_printers") or []):
        continue
    q = 10 ** 9 if n.split("@")[0].rstrip() == "0.20mm Standard" else 2
    lh = c.get("layer_height")
    if q < 10 ** 9 and lh is not None and abs(float(first(lh)) - 0.2) < 0.0005:
        q *= 10
    if q > bestq:
        bestq, best = q, n
json.dump({"name": best, "config": flat("process", best)}, open("u1-process-orca.json", "w", encoding="utf-8"), indent=1)
print(best)
' "$ORCA_PROFILES/Snapmaker" > u1-pick.txt || fail "orca: the Snapmaker presets could not be flattened"
U1_PICK="0.20 Bambu Support W @Snapmaker U1 (0.4 nozzle)"
[ "$(cat u1-pick.txt)" = "$U1_PICK" ] || { cat u1-pick.txt; fail "orca: the desktop pick for the Snapmaker U1 was not $U1_PICK"; }
# The file the PACKAGE ships, not a flattened copy of it: it holds only its
# differences from its parents (74 keys of a machine preset's several hundred),
# and both engines read it over its parents out of their own profiles tree.
U1_MACHINE="$ORCA_PROFILES/Snapmaker/machine/Snapmaker U1 (0.4 nozzle).json"
[ -f "$U1_MACHINE" ] || fail "orca: the package ships no Snapmaker U1 machine preset at $U1_MACHINE"
run u1p "$O" cube.stl --slice 1 --printer-preset "Bambu Lab X1 Carbon 0.4 nozzle" \
    --process-preset "0.20mm Standard @BBL X1C" --outputdir u1p/out --export-3mf u1.3mf
[ "$(rc u1p)" = 0 ] || { show u1p; fail "orca: the X1 Carbon project for the Snapmaker switch exit $(rc u1p)"; }
run u1s "$O" u1p/out/u1.3mf --slice 1 --load-settings "$U1_MACHINE" --outputdir u1s/out
[ "$(rc u1s)" = 0 ] || { show u1s; fail "orca: the Snapmaker printer change exit $(rc u1s)"; }
py '
import json, re, sys
want = json.load(open(sys.argv[2])); g = open(sys.argv[1], errors="replace").read()
def got(k):
    m = re.search(r"^; " + k + r" = (.*)$", g, re.M)
    return None if m is None else m.group(1).split(",")[0].strip()
def first(v):
    return re.sub(r"\"", "", str(v[0] if isinstance(v, list) else v))
bad = []
for k in ("layer_height", "wall_loops", "sparse_infill_density", "travel_speed",
          "default_acceleration", "bridge_speed", "elefant_foot_compensation",
          "enable_support", "ooze_prevention", "prime_volume"):
    if k in want["config"] and got(k) != first(want["config"][k]):
        bad.append("%s: G-code %r, the Snapmaker U1 process %r" % (k, got(k), first(want["config"][k])))
if got("print_settings_id") != want["name"]:
    bad.append("print_settings_id: G-code %r" % got("print_settings_id"))
if got("printer_settings_id") != "Snapmaker U1 (0.4 nozzle)":
    bad.append("printer_settings_id: G-code %r" % got("printer_settings_id"))
assert not bad, "; ".join(bad)
' u1s/out/plate_1.gcode u1-process-orca.json || fail "orca: the Snapmaker printer change did not take the Snapmaker process the desktop selects"
echo "PASS: the Orca printer change reads the new printer's own vendor process list"

# ── The same switch, asked for as a real user or agent would ─────────────────
# Two ways a caller names a printer, and both must slice the project the same
# way the desktop app does when its printer list is used:
#   * --printer-preset "NAME" on a project 3MF (the desktop's printer pick:
#     PresetBundle::update_compatible re-selects the process, PresetBundle.cpp
#     5295-5330 at 31f6803), and
#   * --load-settings "<the file the package ships>", which holds only its
#     differences from its parents and is now read over them, as the desktop
#     loads a preset (load_vendor_configs_from_json: `config = *default_config;
#     config.apply(config_src)`, PresetBundle.cpp 4894).
run u1n "$O" u1p/out/u1.3mf --slice 1 --printer-preset "Snapmaker U1 (0.4 nozzle)" --outputdir u1n/out
[ "$(rc u1n)" = 0 ] || { show u1n; fail "orca: --printer-preset on a project 3MF exit $(rc u1n)"; }
py '
import json, re, sys
want = json.load(open(sys.argv[1])); pick = open(sys.argv[2]).read().strip()
KEYS = ("printable_area", "nozzle_diameter", "printer_settings_id", "print_settings_id", "layer_height",
        "wall_loops", "sparse_infill_density", "travel_speed", "default_acceleration", "outer_wall_speed",
        "enable_support", "prime_volume", "retraction_length", "machine_max_acceleration_x",
        "gcode_flavor", "single_extruder_multi_material", "printer_agent", "bed_mesh_max", "bed_mesh_min",
        "scan_first_layer", "bed_exclude_area", "default_print_profile", "default_filament_profile")
def head(path):
    g = open(path, errors="replace").read()
    def got(k):
        m = re.search(r"^; " + k + r" = (.*)$", g, re.M)
        return None if m is None else m.group(1).strip()
    return g, {k: got(k) for k in KEYS}
def first(v):
    return re.sub(r"\"", "", str(v[0] if isinstance(v, list) else v))
bad = []
gfile, a = head("u1s/out/plate_1.gcode")
gname, b = head("u1n/out/plate_1.gcode")
if a != b:
    bad.append("the shipped file and --printer-preset sliced differently: " +
               "; ".join("%s: %r vs %r" % (k, a[k], b[k]) for k in KEYS if a[k] != b[k]))
for k in ("layer_height", "wall_loops", "sparse_infill_density", "travel_speed", "default_acceleration",
          "outer_wall_speed", "enable_support", "prime_volume"):
    if k in want["config"] and b[k] != first(want["config"][k]):
        bad.append("%s: G-code %r, the flattened Snapmaker U1 process %r" % (k, b[k], first(want["config"][k])))
if b["print_settings_id"] != pick:
    bad.append("print_settings_id: G-code %r, the pick the desktop makes %r" % (b["print_settings_id"], pick))
if b["printer_settings_id"] != "Snapmaker U1 (0.4 nozzle)":
    bad.append("printer_settings_id: G-code %r" % b["printer_settings_id"])
# The keys the shipped file leaves to its parents, and what the file read as
# it stands put there instead: the own dialect of the printer and its tool
# changer. The U1 is a Klipper tool changer (fdm_U1: gcode_flavor klipper,
# single_extruder_multi_material 0); the Bambu project values are marlin and 1,
# which a hybrid slice writes as the printer own.
for key, want in (("gcode_flavor", "klipper"), ("single_extruder_multi_material", "0")):
    if b[key] != want:
        bad.append("%s: G-code %r, the value of the printer own %r" % (key, b[key], want))
# The U1 has four nozzles on a 0.5..270.5 by 1..271 bed: both come from the
# parents of the machine preset (fdm_U1 / fdm_toolchanger / fdm_klipper), which
# the shipped file leaves out.
if (b["nozzle_diameter"] or "").count(",") != 3:
    bad.append("nozzle_diameter: G-code %r, the U1 has four nozzles" % b["nozzle_diameter"])
pts = [tuple(float(v) for v in p.lower().split("x")) for p in (b["printable_area"] or "").split(",")]
if len(pts) < 4 or (min(p[0] for p in pts), max(p[0] for p in pts), min(p[1] for p in pts),
                    max(p[1] for p in pts)) != (0.5, 270.5, 1.0, 271.0):
    bad.append("printable_area: G-code %r, the U1 bed is 0.5..270.5 by 1..271" % b["printable_area"])
else:
    out = []
    for line in gname.splitlines():
        if not line.startswith("G1") or " E" not in line:
            continue
        mx = re.search(r" X([-\d.]+)", line); my = re.search(r" Y([-\d.]+)", line)
        if mx and not (0.5 <= float(mx.group(1)) <= 270.5):
            out.append("X" + mx.group(1))
        if my and not (1.0 <= float(my.group(1)) <= 271.0):
            out.append("Y" + my.group(1))
    if out:
        bad.append("%d extrusion coordinate(s) outside the U1 bed: %s" % (len(out), ", ".join(out[:5])))
assert not bad, "; ".join(bad)
' u1-process-orca.json u1-pick.txt || fail "orca: --printer-preset on a project 3MF did not slice the Snapmaker U1 the desktop way"
echo "PASS: a project 3MF switched to the Snapmaker U1 by name and by the shipped file slices the same"

# An unknown name refuses with the close ones, on this path too.
run u1bad "$O" u1p/out/u1.3mf --slice 1 --printer-preset "Snapmaker U1" --outputdir u1bad/out
[ "$(rc u1bad)" != 0 ] || fail "orca: an unknown printer name was taken on a project 3MF"
grep -q "has no printer preset named 'Snapmaker U1'.*Close names:.*Snapmaker U1 (0.4 nozzle)" u1bad/stderr ||
    { show u1bad; fail "orca: the unknown-name refusal names no close preset"; }
py '
import json, sys; d = json.load(open(sys.argv[1]))
assert d["return_code"] == -5, d
' u1bad/out/result.json || { show u1bad; fail "orca: the unknown-name refusal left no result.json with -5"; }

# A partial preset whose parent this engine does not ship refuses before
# slicing, and says which preset to pass instead.
py '
import json, sys
d = json.load(open(sys.argv[1], encoding="utf-8"))
d["inherits"] = "fdm_no_such_parent"
json.dump(d, open("u1-noparent.json", "w"), indent=1)
' "$U1_MACHINE"
run u1np "$O" u1p/out/u1.3mf --slice 1 --load-settings u1-noparent.json --outputdir u1np/out
[ "$(rc u1np)" != 0 ] || fail "orca: a partial preset with a parent this engine lacks was sliced"
grep -qF "u1-noparent.json is a partial preset that inherits 'fdm_no_such_parent'; pass --printer-preset \"Snapmaker U1 (0.4 nozzle)\" instead" u1np/stderr ||
    { show u1np; fail "orca: the partial-preset refusal does not name the parent and the preset to pass"; }
echo "PASS: a partial preset whose parent is missing refuses with what to do instead"

# The Bambu engine ships the BBL tree, so the same call works there: an A1
# mini project switched to an X1 Carbon by name. The machine values are the
# ones the engine's own --printer-preset resolves for that printer (the
# oracle), and the bed is the X1 Carbon's.
run bambuproj "$B" cube.stl --slice 1 --printer-preset "$A1M" --outputdir bambuproj/out --export-3mf a1m.3mf
[ "$(rc bambuproj)" = 0 ] || { show bambuproj; fail "the A1 mini project fixture exit $(rc bambuproj)"; }
run x1cset "$B" cube.stl --slice 1 --printer-preset "Bambu Lab X1 Carbon 0.4 nozzle" --export-settings x1c-all.json
[ "$(rc x1cset)" = 0 ] || { show x1cset; fail "the X1 Carbon oracle run exit $(rc x1cset)"; }
run x1csw "$B" bambuproj/out/a1m.3mf --slice 1 --printer-preset "Bambu Lab X1 Carbon 0.4 nozzle" --outputdir x1csw/out
[ "$(rc x1csw)" = 0 ] || { show x1csw; fail "bambu: the A1 mini project switched to the X1 Carbon exit $(rc x1csw)"; }
py '
import json, re, sys
def first(v):
    return re.sub(r"\"", "", str(v[0] if isinstance(v, list) else v))
want = json.load(open(sys.argv[1]))
g = open("x1csw/out/plate_1.gcode", errors="replace").read()
def got(k):
    m = re.search(r"^; " + k + r" = (.*)$", g, re.M)
    return None if m is None else m.group(1).strip()
def got_first(k):
    v = got(k)
    return None if v is None else v.split(",")[0].strip()
bad = []
if got("printer_settings_id") != "Bambu Lab X1 Carbon 0.4 nozzle":
    bad.append("printer_settings_id: G-code %r" % got("printer_settings_id"))
for k in ("printable_area", "printable_height", "nozzle_diameter", "machine_max_acceleration_x",
          "machine_max_speed_x", "retraction_length", "extruder_clearance_radius"):
    if k not in want:
        continue
    # One entry of a vector key at most: the header writes the engine count of
    # entries, which is the extruder number, not the file one.
    if got_first(k) != first(want[k]):
        bad.append("%s: G-code %r, the printer preset %r" % (k, got(k), first(want[k])))
# The X1 Carbon bed of 256 mm, not the 180 mm one of the A1 mini.
pts = [tuple(float(v) for v in p.lower().split("x")) for p in (got("printable_area") or "").split(",")]
if len(pts) < 4 or max(p[0] for p in pts) != 256.0 or max(p[1] for p in pts) != 256.0:
    bad.append("printable_area: G-code %r, the X1 Carbon bed is 256 x 256" % got("printable_area"))
assert not bad, "; ".join(bad)
' x1c-all.json || fail "bambu: the project switched to the X1 Carbon did not take the printer preset's own values"
echo "PASS: a Bambu A1 mini project switched to the X1 Carbon by name (Bambu engine)"

# ── The other flags that read a settings file ───────────────────────────────
# --load-filaments, the legacy --machine / --process / --filament (which take
# any settings file) and --downward-settings read their files the same way now,
# and the check below is the point of it: every key a shipped preset leaves to
# its parents must reach the slice as the parent's value, which is what
# tests/tools/check_preset_gcode.py measures (the file's own keys taken out of
# the flattened preset, and the rest read out of the G-code header).
U1_PROCESS="$ORCA_PROFILES/Snapmaker/process/0.20 Standard @Snapmaker U1 (0.4 nozzle).json"
U1_FILAMENT="$ORCA_PROFILES/Snapmaker/filament/Snapmaker PLA @U1.json"

# The switch through --load-settings, checked key by key against the parents.
python3 "$SCRIPT_DIR/tools/check_preset_gcode.py" "$ORCA_PROFILES/Snapmaker" machine \
    "Snapmaker U1 (0.4 nozzle)" u1s/out/plate_1.gcode u1n/out/plate_1.gcode ||
    fail "orca: the shipped Snapmaker U1 machine file did not arrive with its parents' values"

# An STL through the three legacy flags, with the files the package ships, and
# the filament a U1 user has: "Snapmaker PLA @U1". KNOWN FAILURE, kept as the
# user's own call -- the trio leaves curr_bed_type at the config default (Cool
# Plate; the printer's own Textured PEI Plate refuses the same way), and the
# shipped Snapmaker PLA @U1 chain defines no temperature for either plate type
# (only hot_plate_temp 55), so the engine refuses -103, "Found some filament
# unprintable at first layer on current Plate". That is audit row 18 (an STL
# plus settings files gets a plate type that is not the printer's default);
# lane-shorthand owns it. The rest of this case is what passes today: the same
# trio with a system filament that names every plate slices, and the preset
# path picks a plate the shipped filament has a temperature for.
run u1leg "$O" cube.stl --slice 1 --machine "$U1_MACHINE" --process "$U1_PROCESS" --filament "$U1_FILAMENT" \
    --outputdir u1leg/out
if [ "$(rc u1leg)" != 0 ]; then
    show u1leg
    fail "orca: the shipped Snapmaker trio through --machine/--process/--filament exit $(rc u1leg) (row 18: the plate type, not this lane)"
else
    python3 "$SCRIPT_DIR/tools/check_preset_gcode.py" "$ORCA_PROFILES/Snapmaker" machine \
        "Snapmaker U1 (0.4 nozzle)" u1leg/out/plate_1.gcode ||
        fail "orca: --machine did not read the shipped file over its parents"
    python3 "$SCRIPT_DIR/tools/check_preset_gcode.py" "$ORCA_PROFILES/Snapmaker" process \
        "0.20 Standard @Snapmaker U1 (0.4 nozzle)" u1leg/out/plate_1.gcode ||
        fail "orca: --process did not read the shipped file over its parents"
    python3 "$SCRIPT_DIR/tools/check_preset_gcode.py" "$ORCA_PROFILES/Snapmaker" filament \
        "Snapmaker PLA @U1" u1leg/out/plate_1.gcode ||
        fail "orca: --filament did not read the shipped file over its parents"
fi

# --load-filaments with a shipped filament, on the printer it belongs to.
run u1fil "$O" cube.stl --slice 1 --printer-preset "Snapmaker U1 (0.4 nozzle)" \
    --load-filaments "$U1_FILAMENT" --outputdir u1fil/out
[ "$(rc u1fil)" = 0 ] || { show u1fil; fail "orca: the shipped Snapmaker filament through --load-filaments exit $(rc u1fil)"; }
python3 "$SCRIPT_DIR/tools/check_preset_gcode.py" "$ORCA_PROFILES/Snapmaker" filament \
    "Snapmaker PLA @U1" u1fil/out/plate_1.gcode ||
    fail "orca: --load-filaments did not read the shipped filament over its parents"
echo "PASS: the legacy flags and --load-filaments read the shipped Snapmaker presets over their parents"

# The Bambu engine's own filament, whose chain includes a template file
# ("include", the BBL tree's own spelling): the same check, and the include is
# part of what the parents hold.
A1M_FILE="$RES/profiles/BBL/filament/Bambu PLA Basic @BBL A1M.json"
[ -f "$A1M_FILE" ] || fail "the package ships no Bambu PLA Basic @BBL A1M filament at $A1M_FILE"
run filb "$B" cube.stl --slice 1 --printer-preset "$A1M" --load-filaments "$A1M_FILE" --outputdir filb/out
[ "$(rc filb)" = 0 ] || { show filb; fail "bambu: the shipped BBL filament through --load-filaments exit $(rc filb)"; }
python3 "$SCRIPT_DIR/tools/check_preset_gcode.py" "$RES/profiles/BBL" filament \
    "Bambu PLA Basic @BBL A1M" filb/out/plate_1.gcode ||
    fail "bambu: --load-filaments did not read the shipped filament over its parents and its include"
echo "PASS: --load-filaments reads a shipped BBL filament over its parents and its include (Bambu engine)"

# --downward-check reads the machine file the same way: the answer must be the
# one the engine's own view of the preset gives (--export-settings of the same
# printer by name), not the one a 62-key file alone would give.
run x1cdwset "$B" cube.stl --slice 1 --printer-preset "Bambu Lab X1 Carbon 0.4 nozzle" --export-settings x1c-all.json
py '
import json
d = json.load(open("x1c-all.json"))
d.update({"type": "machine", "from": "system", "name": "Bambu Lab X1 Carbon 0.4 nozzle", "instantiation": "true"})
json.dump(d, open("x1c-machine.json", "w"), indent=1)
'
X1C_FILE="$RES/profiles/BBL/machine/Bambu Lab X1 Carbon 0.4 nozzle.json"
[ -f "$X1C_FILE" ] || fail "the package ships no X1 Carbon machine preset at $X1C_FILE"

# ── The owner's case: a Bambu Studio project for a Bambu printer, on the Orca
# build, retargeted to a Snapmaker U1 ────────────────────────────────────────
# Three of its parts are the desktop's, and none of them was ported:
#   * BambuStudio writes its filament indices from 0 (its own range starts
#     there, BambuStudio PrintConfig.cpp 4359-4361 at 5873b5f) while this
#     engine's start at 1, so the value check refuses the file -18. The desktop
#     only warns (Plater.cpp 6259-6272) and slices from full_fff_config, which
#     resets those keys into [1, N] (PresetBundle.cpp 4090-4105; N =
#     filament_presets.size(), 3866). That clamp is this command line's own:
#     the CLI path never runs full_fff_config.
#   * tree_support_wall_count is one setting in two encodings, and the engine
#     already converts it on every config load -- PrintConfigDef::handle_legacy,
#     ported from OrcaSlicer 434ff3011f77 in the override layer of
#     libslic3r/orcaslicer/libslic3r/PrintConfig.cpp.
#   * BambuStudio writes no print_compatible_printers, and the desktop does not
#     read the missing list as "suits every printer": the project's process is
#     loaded over the system preset its print_settings_id names and keeps that
#     preset's compatible_printers (load_external_preset, Preset.cpp
#     2446-2500), so a printer outside them re-selects a process
#     (PresetBundle.cpp 5295-5330).
# The fixture is a real Bambu Studio X1 Carbon project (base.3mf) with the
# project_settings a Bambu Studio X1C project carries: its filament indices
# from 0, tree_support_wall_count -1, no print_compatible_printers. Two of its
# Bambu-only spellings that neither the desktop nor this engine converts are
# left out (the -1 "auto" of raft_first_layer_expansion and the 'enabled' of
# ensure_vertical_shell_thickness), so the runs below test those three
# behaviours and not those refusals.
py '
import json, re, zipfile
def build(src, dst, settings, app=None):
    with zipfile.ZipFile(src) as zin, zipfile.ZipFile(dst, "w", zipfile.ZIP_DEFLATED) as zout:
        for item in zin.infolist():
            data = zin.read(item.filename)
            if item.filename == "Metadata/project_settings.config":
                d = json.loads(data)
                for k in ("raft_first_layer_expansion", "ensure_vertical_shell_thickness"):
                    d.pop(k, None)
                d.pop("print_compatible_printers", None)
                d.update(settings)
                data = json.dumps(d, indent=4).encode()
            if app and item.filename == "3D/3dmodel.model":
                data = re.sub(rb"(<metadata name=\"Application\">)[^<]*", rb"\g<1>" + app.encode(), data)
            zout.writestr(item, data)
bambu = {"wall_filament": "0", "sparse_infill_filament": "0", "solid_infill_filament": "0",
         "tree_support_wall_count": "-1"}
build("base.3mf", "bblx1c.3mf", bambu)
# The same values from a file OrcaSlicer made: handle_legacy converts by VALUE
# (PrintConfig.cpp 7942, OrcaSlicer 434ff3011f77), not by the maker of the file.
build("base.3mf", "orcamade.3mf", bambu, app="OrcaSlicer-2.4.0")
with zipfile.ZipFile("bblx1c.3mf") as z:
    d = json.loads(z.read("Metadata/project_settings.config"))
    assert "print_compatible_printers" not in d and d["tree_support_wall_count"] == "-1", sorted(d)
    assert d["print_settings_id"] == "0.20mm Standard @BBL X1C" and d["layer_height"] == "0.2", d
' || fail "orca: the Bambu Studio X1C project fixtures could not be written"
run dwship "$B" bblx1c.3mf --slice 1 --allow-newer-file --downward-check --downward-settings "$X1C_FILE" --outputdir dwship/out
[ "$(rc dwship)" = 0 ] || { show dwship; fail "bambu: --downward-settings with the shipped machine file exit $(rc dwship)"; }
run dwfull "$B" bblx1c.3mf --slice 1 --allow-newer-file --downward-check --downward-settings x1c-machine.json --outputdir dwfull/out
[ "$(rc dwfull)" = 0 ] || { show dwfull; fail "bambu: --downward-settings with the engine's own machine file exit $(rc dwfull)"; }
py '
import json, sys
a = json.load(open(sys.argv[1])); b = json.load(open(sys.argv[2]))
ka = {k: v for k, v in a.items() if k.startswith("downward")}
kb = {k: v for k, v in b.items() if k.startswith("downward")}
assert ka == kb, (ka, kb)
assert ka, "no downward answer to compare"
' dwship/out/result.json dwfull/out/result.json ||
    fail "bambu: the shipped machine file and the engine's own preset gave different --downward-check answers"
echo "PASS: --downward-settings reads the shipped machine file over its parents (Bambu engine)"
run bblu1 "$O" bblx1c.3mf --slice 1 --allow-newer-file --load-settings "$U1_MACHINE" --outputdir bblu1/out
[ "$(rc bblu1)" = 0 ] || { show bblu1; fail "orca: a Bambu Studio X1C project on the Snapmaker U1 exit $(rc bblu1)"; }
[ -s bblu1/out/plate_1.gcode ] || { show bblu1; fail "orca: the retargeted Bambu Studio project wrote no G-code"; }
# The project with no printer switch slices with its own settings, so the
# converted value is readable in the G-code.
run bbldirect "$O" bblx1c.3mf --slice 1 --allow-newer-file --outputdir bbldirect/out
[ "$(rc bbldirect)" = 0 ] || { show bbldirect; fail "orca: a Bambu Studio X1C project with no printer switch exit $(rc bbldirect)"; }
grep -q "^; tree_support_wall_count = 0$" bbldirect/out/plate_1.gcode || {
    grep -m1 "tree_support_wall_count" bbldirect/out/plate_1.gcode
    fail "orca: the BambuStudio -1 of tree_support_wall_count did not reach the slice as this engine's 0"; }
run orcamade "$O" orcamade.3mf --slice 1 --allow-newer-file --outputdir orcamade/out
[ "$(rc orcamade)" = 0 ] || { show orcamade; fail "orca: a file OrcaSlicer made with tree_support_wall_count -1 exit $(rc orcamade)"; }
grep -q "^; tree_support_wall_count = 0$" orcamade/out/plate_1.gcode || {
    grep -m1 "tree_support_wall_count" orcamade/out/plate_1.gcode
    fail "orca: the ported legacy conversion skipped a file OrcaSlicer made (it converts by value)"; }
# The clamp is the fallback for a project this bundle cannot load over its own
# presets: the file's own settings are sliced, so the 0-based indices reach the
# engine and the clamp is what carries them. It is reported as its own event and
# names only the keys it reset: tree_support_wall_count is converted by the
# engine's own handle_legacy, not here. A project whose preset this bundle does
# ship takes the preset's own values instead, and reports no reset.
py '
import json, zipfile
with zipfile.ZipFile("bblx1c.3mf") as zin, zipfile.ZipFile("bblnosuch.3mf", "w", zipfile.ZIP_DEFLATED) as zout:
    for item in zin.infolist():
        data = zin.read(item.filename)
        if item.filename == "Metadata/project_settings.config":
            d = json.loads(data)
            d["print_settings_id"] = "0.20mm Standard @BBL NoSuchProfile"
            data = json.dumps(d, indent=4).encode()
        zout.writestr(item, data)
' || fail "orca: the no-system-preset fixture could not be written"
run bblflat "$O" bblnosuch.3mf --slice 1 --allow-newer-file --outputdir bblflat/out
[ "$(rc bblflat)" = 0 ] || { show bblflat; fail "orca: a project with no shipped system preset exit $(rc bblflat)"; }
grep -q 'ProjectPresetRebaseSkipped' bblflat/stdout || { show bblflat; fail "the file's own settings were not kept for a project the bundle cannot load over"; }
py '
import json, sys
ev = [json.loads(l.split("]] ", 1)[1]) for l in open(sys.argv[1], errors="replace") if "[[SLICER_EVENT]]" in l]
ev = [e for e in ev if e.get("tag") == "FilamentIndexOutOfRangeReset"]
assert len(ev) == 1, [e.get("tag") for e in ev]
e = ev[0]
assert e["filament_count"] == 1, e
assert sorted(e["reset"]) == ["solid_infill_filament", "sparse_infill_filament", "wall_filament"], e
assert all(c["from"] == 0 and c["to"] == 1 for c in e["reset"].values()), e
assert e["clamped"] == {}, e
for k in ("wall_filament", "solid_infill_filament", "sparse_infill_filament"):
    assert k in e["message"], e["message"]
assert "tree_support_wall_count" not in e["message"], e["message"]
' bblflat/stdout || fail "orca: the 3MF filament-index reset was not reported as its own event"
py '
import json, sys
ev = [json.loads(l.split("]] ", 1)[1]) for l in open(sys.argv[1], errors="replace") if "[[SLICER_EVENT]]" in l]
assert not [e for e in ev if e.get("tag") == "FilamentIndexOutOfRangeReset"], "a rebased run reset an index"
' bblu1/stdout || fail "orca: a project loaded over its shipped presets reported an index reset"
echo "PASS: the Orca 3MF load resets a Bambu Studio project's 0-based filament indices into this engine's range, and reports it"
# The printer change takes the U1's process, and the values, the pick and the
# bed are the ones the desktop's retarget gives (u1-process-orca.json is the
# process the desktop selects, flattened from the shipped presets).
py '
import json, re, sys
want = json.load(open(sys.argv[2])); pick = open(sys.argv[3]).read().strip()
g = open(sys.argv[1], errors="replace").read()
def got(k):
    m = re.search(r"^; " + k + r" = (.*)$", g, re.M)
    return None if m is None else m.group(1).split(",")[0].strip()
def raw(k):
    m = re.search(r"^; " + k + r" = (.*)$", g, re.M)
    return None if m is None else m.group(1).strip()
def first(v):
    return re.sub(r"\"", "", str(v[0] if isinstance(v, list) else v))
bad = []
for k in ("layer_height", "wall_loops", "sparse_infill_density", "travel_speed",
          "default_acceleration", "outer_wall_speed"):
    if k in want["config"] and got(k) != first(want["config"][k]):
        bad.append("%s: G-code %r, the flattened Snapmaker U1 process %r" % (k, got(k), first(want["config"][k])))
if got("print_settings_id") != pick:
    bad.append("print_settings_id: G-code %r, the pick the desktop makes %r" % (got("print_settings_id"), pick))
if got("printer_settings_id") != "Snapmaker U1 (0.4 nozzle)":
    bad.append("printer_settings_id: G-code %r" % got("printer_settings_id"))
# Every extrusion is inside the U1 plate the printer preset states: the objects
# of the project were laid out for an X1 Carbon bed and moved.
area = raw("printable_area")
pts = [tuple(float(v) for v in p.lower().split("x")) for p in area.split(",")] if area else []
xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
if not pts:
    bad.append("no printable_area in the G-code")
else:
    out = []
    for line in g.splitlines():
        if not line.startswith("G1") or " E" not in line:
            continue
        mx = re.search(r" X([-\d.]+)", line); my = re.search(r" Y([-\d.]+)", line)
        if mx and not (min(xs) - 0.001 <= float(mx.group(1)) <= max(xs) + 0.001):
            out.append("X" + mx.group(1))
        if my and not (min(ys) - 0.001 <= float(my.group(1)) <= max(ys) + 0.001):
            out.append("Y" + my.group(1))
    if out:
        bad.append("%d extrusion coordinate(s) outside the printer bed %r: %s" % (len(out), area, ", ".join(out[:5])))
assert not bad, "; ".join(bad)
' bblu1/out/plate_1.gcode u1-process-orca.json u1-pick.txt || fail "orca: the Bambu Studio project on the Snapmaker U1 did not slice with the desktop's retarget"
echo "PASS: a Bambu Studio project slices on the Snapmaker U1 with the desktop's process, values and bed"

# The switch onto a printer with a different nozzle count is the desktop's own:
# the H2D project states two nozzles, the Snapmaker U1 has four, and the desktop
# selects the preset and reconciles the config with it (Plater.cpp 9451-9613 at
# 31f6803: the printer tab's select_preset, then on_config_change(full_config);
# Plater.cpp 16527-16610: a changed nozzle count runs update_flush_volume_matrix;
# PresetBundle.cpp 5111-5160: update_multi_material_filament_presets grows the
# filament presets to the new extruder count and rebuilds the flush matrix).
# Slicing it needs memory in proportion to the model, and a slice that runs out
# of memory reports the official out-of-memory outcome (CLI_OUT_OF_MEMORY -14,
# Utils.hpp:39) with its sentence (OrcaSlicer.cpp 126), never the slicing
# failure -100. The corpus project is 370 MB and external: the case runs where it
# is, and skips where it is not.
H2D="${SLICER_CLI_CORPUS:-$HOME/engcheck-plugin/corpus}/13ab99508c843d4d.3mf"
case "$(uname -s)" in
MINGW*|MSYS*|CYGWIN*)
    echo "SKIP: the H2D project switched to the Snapmaker U1 (Windows has no ulimit cap)";;
*)
if [ ! -f "$H2D" ]; then
    echo "SKIP: the H2D project switched to the Snapmaker U1 (no $H2D)"
else
    # A hard cap makes the outcome deterministic: the load fits, the slice does
    # not, so the engine raises std::bad_alloc inside its slicing step.
    run h2du1 bash -c 'ulimit -v 6291456; exec "$@"' _ "$O" "$H2D" --slice 1 --allow-newer-file \
        --load-settings "$U1_MACHINE" --outputdir h2du1/out
    case "$(rc h2du1)" in
    0)   echo "PASS: the H2D project switched to the Snapmaker U1 slices (this machine had the memory)";;
    242) # 256 - 14: CLI_OUT_OF_MEMORY
         grep -q 'Out of memory during slicing' h2du1/stderr || { show h2du1; fail "orca: the out-of-memory run has no sentence"; }
         grep -q '"return_code": *-14' h2du1/out/result.json || { show h2du1; fail "orca: the out-of-memory run did not record -14"; }
         echo "PASS: a slice that ran out of memory is reported as such, with what to do";;
    *)   show h2du1; fail "orca: the H2D project on the Snapmaker U1 exit $(rc h2du1)";;
    esac
    if [ -f h2du1/out/result.json ] && grep -q '"return_code": *-100' h2du1/out/result.json; then
        fail "orca: -100 reached the user for the H2D project on the Snapmaker U1"
    fi
fi
;; 
esac

# The same conversions apply to an object's own settings and to a part's, which
# the file keeps in Metadata/model_settings.config (bbs_3mf loads them over the
# same legacy pass, bbs_3mf.cpp 2130, 5115, 5263 at 31f6803). They are matched
# by the names the file states, because the 3MF's numeric ids are not kept on
# the loaded objects.
py '
import json, zipfile
with zipfile.ZipFile("base.3mf") as zin, zipfile.ZipFile("objover.3mf", "w", zipfile.ZIP_DEFLATED) as zout:
    for item in zin.infolist():
        data = zin.read(item.filename)
        if item.filename == "Metadata/project_settings.config":
            d = json.loads(data)
            for k in ("raft_first_layer_expansion", "ensure_vertical_shell_thickness"):
                d.pop(k, None)
            d.update({"wall_filament": "0", "sparse_infill_filament": "0", "solid_infill_filament": "0"})
            data = json.dumps(d, indent=4).encode()
        if item.filename == "Metadata/model_settings.config":
            text = data.decode()
            assert text.count("<metadata key=\"name\" value=\"Cube\"/>") >= 1, text[:200]
            # The object own settings: a BambuStudio word and an automatic value.
            text = text.replace("<metadata key=\"name\" value=\"Cube\"/>",
                                "<metadata key=\"name\" value=\"Cube\"/>\n"
                                "    <metadata key=\"ensure_vertical_shell_thickness\" value=\"enabled\"/>\n"
                                "    <metadata key=\"tree_support_wall_count\" value=\"-1\"/>", 1)
            data = text.encode()
        zout.writestr(item, data)
' || fail "orca: the object-override fixture could not be written"
run objconv "$O" objover.3mf --slice 1 --allow-newer-file --outputdir objconv/out
[ "$(rc objconv)" = 0 ] || { show objconv; fail "orca: a project whose object states BambuStudio values exit $(rc objconv)"; }
py '
import json, sys
ev = [json.loads(l.split("]] ", 1)[1]) for l in open(sys.argv[1], errors="replace") if "[[SLICER_EVENT]]" in l]
def one(tag):
    found = [e for e in ev if e.get("tag") == tag and e.get("object") == "Cube"]
    assert len(found) == 1, (tag, [e.get("tag") for e in ev])
    return found[0]
shell = one("VerticalShellThicknessWordConverted")
assert shell["from"] == "enabled" and shell["to"] == "ensure_all", shell
wall = one("TreeSupportWallCountAutoConverted")
assert wall["from"] == "-1" and str(wall["to"]) == "0", wall
' objconv/stdout || fail "orca: the object own settings were not converted with the same events as the project's"
echo "PASS: an object's own tree_support_wall_count -1 and ensure_vertical_shell_thickness \"enabled\" convert like the project's"

# ── The desktop's and upstream's conversions the owner ruled on (G2-G8) ─────
# Each case states the BambuStudio value on purpose (the maker's own key list,
# different_settings_to_system), so the rebase keeps it and only the conversion
# can carry it across.
py '
import json, re, zipfile
def proj(dst, settings=None, diff=None, app=None):
    with zipfile.ZipFile("base.3mf") as zin, zipfile.ZipFile(dst, "w", zipfile.ZIP_DEFLATED) as zout:
        for item in zin.infolist():
            data = zin.read(item.filename)
            if item.filename == "Metadata/project_settings.config":
                d = json.loads(data)
                for k in ("raft_first_layer_expansion", "ensure_vertical_shell_thickness", "top_one_wall_type",
                          "tree_support_wall_count", "support_type"):
                    d.pop(k, None)
                d.update(settings or {})
                if diff:
                    d["different_settings_to_system"] = [diff, "", ""]
                data = json.dumps(d, indent=4).encode()
            if app and item.filename == "3D/3dmodel.model":
                data = re.sub(rb"(<metadata name=\"Application\">)[^<]*", rb"\g<1>" + app.encode(), data)
            zout.writestr(item, data)
proj("mkr.3mf", {"raft_first_layer_expansion": "-1", "top_one_wall_type": "not apply",
                 "ensure_vertical_shell_thickness": "disabled", "support_type": "normal(auto)"},
     "raft_first_layer_expansion;top_one_wall_type;ensure_vertical_shell_thickness;support_type")
proj("mkrtree.3mf", {"raft_first_layer_expansion": "-1", "support_type": "tree(auto)"},
     "raft_first_layer_expansion;support_type")
proj("mkrg4.3mf", {"tree_support_wall_count": "0", "support_type": "tree(auto)"}, "tree_support_wall_count;support_type")
proj("mkrpartial.3mf", {"ensure_vertical_shell_thickness": "partial", "support_type": "normal(auto)"},
     "ensure_vertical_shell_thickness;support_type")
proj("neworca.3mf", {}, None, app="OrcaSlicer-99.01.00.00")
json.dump({"type": "machine", "name": "conv-machine", "from": "system", "instantiation": "true",
           "use_relative_e_distances": "0"},
          open("g7-machine.json", "w"))
json.dump({"type": "filament", "name": "conv-filament", "from": "system", "instantiation": "true"},
          open("g7-filament.json", "w"))
json.dump({"type": "process", "name": "conv-process", "from": "system", "instantiation": "true",
           "wall_infill_order": "infill/outer wall/inner wall", "support_type": "hybrid(auto)"},
          open("g7-process.json", "w"))
json.dump({"type": "machine", "name": "conv-bad", "from": "system", "instantiation": "true",
           "machine_start_gcode": "G28\n{if nozzle_diameter_at_nozzle_id[initial_nozzle_id] > 0.4}M900\n{endif}"},
          open("g8-bad.json", "w"))
' || fail "orca: the conversion fixtures could not be written"

# G2: a Bambu Studio file newer than the Bambu base this engine is built on
# slices, with a warning that names the version and the ignored settings; a file
# OrcaSlicer made and newer than this engine is still refused.
run g2warn "$O" newer.3mf --slice 1 --outputdir g2warn/out
[ "$(rc g2warn)" = 0 ] || { show g2warn; fail "orca: a Bambu Studio file newer than the Bambu base exit $(rc g2warn)"; }
grep -q 'FileNewerThanEngineBase' g2warn/stdout || { show g2warn; fail "no FileNewerThanEngineBase warning"; }
run g2ref "$O" neworca.3mf --slice 1 --outputdir g2ref/out
[ "$(rc g2ref)" != 0 ] || fail "orca: an OrcaSlicer file newer than this engine was sliced"
grep -q 'FileVersionNewerThanEngine' g2ref/stdout || { show g2ref; fail "no FileVersionNewerThanEngine refusal"; }
echo "PASS: the Bambu base decides for a Bambu Studio file, and this engine's version for an OrcaSlicer one"

# --engine-info's file_newer_than_engine is the slice gate's answer: true only
# when this binary refuses the file. On the Orca build the real BambuStudio
# 02.05 fixture is older than the 02.06 Bambu base (it slices, no warning), and
# a BambuStudio 99.1 project slices with the warning above, so both are false;
# an OrcaSlicer project newer than this engine is refused, so true. The real
# OrcaSlicer 2.4.0-alpha fixture is not newer. The Bambu build compares every
# file with its own version, as before.
"$O" --engine-info "$FIXTURE" > ei-g2base.json
"$O" --engine-info newer.3mf > ei-g2warn.json
"$O" --engine-info neworca.3mf > ei-g2ref.json
cp "$SCRIPT_DIR/fixtures/calib_base_orca.3mf" g2orca.3mf
"$O" --engine-info g2orca.3mf > ei-g2orca.json
"$B" --engine-info "$FIXTURE" > ei-bbase.json
"$B" --engine-info newer.3mf > ei-bnewer.json
run g2base "$O" "$FIXTURE" --slice 1 --outputdir g2base/out
[ "$(rc g2base)" = 0 ] || { show g2base; fail "orca: the BambuStudio 02.05 fixture exit $(rc g2base)"; }
! grep -q 'FileNewerThanEngineBase\|FileVersionNewerThanEngine' g2base/stdout || fail "orca: the BambuStudio 02.05 fixture was called newer"
py '
import json
def newer(f): return json.load(open(f))["this_engine_reads"]["file_newer_than_engine"]
want = {"ei-g2base.json": False, "ei-g2warn.json": False, "ei-g2ref.json": True, "ei-g2orca.json": False,
        "ei-bbase.json": False, "ei-bnewer.json": True}
bad = {f: newer(f) for f in want if newer(f) is not want[f]}
assert not bad, bad
' || fail "--engine-info file_newer_than_engine does not match what the slice does"
echo "PASS: --engine-info calls a file newer only when the slice refuses it (both engines)"

# G3, G5, G6 in one run: the maker own words.
run g356 "$O" mkr.3mf --slice 1 --outputdir g356/out
[ "$(rc g356)" = 0 ] || { show g356; fail "orca: a project with the maker own values exit $(rc g356)"; }
py '
import json, re, sys
g = open(sys.argv[1], errors="replace").read()
ev = [json.loads(l.split("]] ", 1)[1]) for l in open(sys.argv[2], errors="replace") if "[[SLICER_EVENT]]" in l]
def header(key):
    m = re.search(r"^; " + key + r" = (.*)$", g, re.M)
    return None if m is None else m.group(1).strip()
bad = []
assert [e for e in ev if e.get("tag") == "RaftAutoExpansionConverted"], [e.get("tag") for e in ev]
assert [e for e in ev if e.get("tag") == "TopOneWallTypeNotApplyConverted"], [e.get("tag") for e in ev]
assert [e for e in ev if e.get("tag") == "VerticalShellThicknessWordConverted"], [e.get("tag") for e in ev]
if header("raft_first_layer_expansion") != "2":
    bad.append("raft_first_layer_expansion: G-code %r" % header("raft_first_layer_expansion"))
if header("only_one_wall_top") != "0":
    bad.append("only_one_wall_top: G-code %r" % header("only_one_wall_top"))
if header("ensure_vertical_shell_thickness") != "none":
    bad.append("ensure_vertical_shell_thickness: G-code %r" % header("ensure_vertical_shell_thickness"))
assert not bad, "; ".join(bad)
' g356/out/plate_1.gcode g356/stdout || fail "orca: the maker own BambuStudio values did not reach the slice converted"
echo "PASS: raft_first_layer_expansion -1, top_one_wall_type \"not apply\" and ensure_vertical_shell_thickness \"disabled\" convert"

# G3 for tree support: this engine has no automatic expansion there, and the
# refusal says what to set instead.
run g3tree "$O" mkrtree.3mf --slice 1 --outputdir g3tree/out
[ "$(rc g3tree)" != 0 ] || fail "orca: an automatic expansion was sliced for tree support"
grep -q 'BambuValueNotCarryable' g3tree/stdout || { show g3tree; fail "no tree-support refusal"; }
grep -q 'with tree support' g3tree/stderr || { show g3tree; fail "the refusal does not say why tree support has no automatic value"; }
grep -q 'Set it to the expansion you want in mm' g3tree/stderr || { show g3tree; fail "the refusal does not say what to do"; }
echo "PASS: an automatic first-layer expansion with tree support is refused with what to set instead"

# G4: BambuStudio 0 (infill-only walls) is this engine auto, kept and warned
# about where tree support is in play.
run g4 "$O" mkrg4.3mf --slice 1 --outputdir g4/out
[ "$(rc g4)" = 0 ] || { show g4; fail "orca: a project with tree_support_wall_count 0 exit $(rc g4)"; }
grep -q 'TreeSupportWallCountZeroIsAuto' g4/stdout || { show g4; fail "no warning for the 0 the file means as infill-only"; }
grep -q '^; tree_support_wall_count = 0$' g4/out/plate_1.gcode || { show g4; fail "the value did not stay 0"; }
echo "PASS: BambuStudio's tree_support_wall_count 0 is kept and warned about"

# G6 for the word with no twin: refused, with the choice to make.
run g6ref "$O" mkrpartial.3mf --slice 1 --outputdir g6ref/out
[ "$(rc g6ref)" != 0 ] || fail "orca: \"partial\" was sliced"
grep -q 'BambuValueNotCarryable' g6ref/stdout || { show g6ref; fail "no refusal for partial"; }
grep -qi 'pick the shell coverage' g6ref/stderr || { show g6ref; fail "the refusal does not say what to do"; }
echo "PASS: ensure_vertical_shell_thickness \"partial\" is refused with the choice to make"

# G7: a --process file runs the file-level conversions the engines run when they
# read a settings file (Config.cpp 931-948).
run g7 "$O" cube.stl -o g7.gcode --machine g7-machine.json --filament g7-filament.json --process g7-process.json
[ "$(rc g7)" = 0 ] || { show g7; fail "orca: a --process file with old words exit $(rc g7)"; }
grep -q '^; is_infill_first = 1$' g7.gcode || { grep -m1 is_infill_first g7.gcode; fail "wall_infill_order was not converted"; }
grep -q '^; support_style = tree_hybrid$' g7.gcode || { grep -m1 support_style g7.gcode; fail "support_type hybrid(auto) was not converted"; }
echo "PASS: a --process file's own words convert the way a settings file's do"

# G8: a Bambu Studio placeholder in the machine's own start G-code is refused
# before the run slices, naming it and the value to use instead.
run g8 "$O" cube.stl -o g8.gcode --machine g8-bad.json
[ "$(rc g8)" != 0 ] || fail "orca: a Bambu-only G-code placeholder was sliced"
grep -q 'BambuOnlyGcodePlaceholder' g8/stdout || { show g8; fail "no refusal for the placeholder"; }
grep -q 'nozzle_diameter_at_nozzle_id' g8/stderr || { show g8; fail "the refusal does not name the placeholder"; }
grep -q '{nozzle_diameter\[initial_extruder\]}' g8/stderr || { show g8; fail "the refusal does not say what to use instead"; }
echo "PASS: a Bambu-only G-code placeholder is refused before slicing, naming what to use instead"

# The engine's 3MF loader extracts Metadata/project_settings.config and the
# embedded presets as <backup>/_temp_3.config / _temp_2.config and parses them
# back (bbs_3mf.cpp _extract_project_config_from_archive 2636 and
# _extract_project_embedded_presets_from_archive 2665 at 31f6803). The shared
# <tmp>/slicer_cli_backup name let parallel runs read each other's half-written
# file - "load_from_json: parse <tmp>/slicer_cli_backup/_temp_3.config got a
# parse_error" - and then slice with the other project's settings; each run
# stages in a folder of its own now (run_backup_path), as f747602 does for the
# --export-3mf staging folder.
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    case "$(uname -s)" in
    MINGW*|MSYS*|CYGWIN*) echo "SKIP: the private-TMPDIR backup check (Windows)";;
    *)
        # A run whose TMPDIR is its own keeps it clear: no shared
        # slicer_cli_backup, and the folder the load stages in is the run's own
        # and goes.
        mkdir -p bkdir-$e/tmp
        TMPDIR="$PWD/bkdir-$e/tmp" "$bin" cube.stl --slice 1 --printer-preset "$A1M" \
            --outputdir bkdir-$e/out > bkdir-$e/stdout 2>&1 || { tail -n 3 bkdir-$e/stdout; fail "$e: a run with its own TMPDIR failed"; }
        [ -s bkdir-$e/out/plate_1.gcode ] || fail "$e: a run with its own TMPDIR wrote no G-code"
        [ ! -e bkdir-$e/tmp/slicer_cli_backup ] || fail "$e: the 3MF load staged in the shared slicer_cli_backup folder"
        [ -z "$(ls -A bkdir-$e/tmp)" ] || { ls -A bkdir-$e/tmp; fail "$e: the run left folders in the temp dir it was given"; };;
    esac
    # 3 workers x 20 slices of one project 3MF on the default temp dir.
    run bkproj-$e "$bin" cube.stl --slice 1 --printer-preset "$A1M" --outputdir bkproj-$e/out --export-3mf bk.3mf
    [ "$(rc bkproj-$e)" = 0 ] || { show bkproj-$e; fail "$e: the parallel-slice fixture exit $(rc bkproj-$e)"; }
    rm -rf bkrace-$e-failed bkrace-$e-w
    bkr() {
        local n=$1 i
        for i in $(seq 1 20); do
            rm -rf "bkrace-$e-w$n-$i"; mkdir -p "bkrace-$e-w$n-$i"
            "$bin" "bkproj-$e/out/bk.3mf" --slice 1 --outputdir "bkrace-$e-w$n-$i" > "bkrace-$e-$n-$i.log" 2>&1 \
                || echo 1 >> "bkrace-$e-failed"
            rm -rf "bkrace-$e-w$n-$i"
        done
    }
    for n in 1 2 3; do bkr "$n" & done
    wait
    [ ! -e bkrace-$e-failed ] || { tail -n 3 bkrace-$e-*.log; fail "$e: a parallel slice failed"; }
    if grep -q "parse_error" bkrace-$e-*.log 2>/dev/null; then
        grep -l "parse_error" bkrace-$e-*.log | head -2 | xargs -r tail -n 3
        fail "$e: a parallel run parsed another run's half-written backup config"
    fi
    rm -f bkrace-$e-*
done
echo "PASS: 3 parallel workers x 20 slices share no 3MF-load backup folder (both engines)"

# A refusal before anything loads still leaves result.json under --slice:
# an unknown preset name (CLI_CONFIG_FILE_ERROR, -5), and --plate with --slice
# (CLI_INVALID_PARAMS, -2). A project 3MF with a preset name this engine has is
# no longer refused: it is the desktop app's printer switch.
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run rfp-$e "$bin" ftow-$e/out/ftow.3mf --slice 1 --printer-preset "No Such Printer 0.4 nozzle" --outputdir rfp-$e/out
    run rfq-$e "$bin" ftow-$e/out/ftow.3mf --slice 1 --plate 1 --outputdir rfq-$e/out
    for n in rfp:-5 rfq:-2; do
        name=${n%%:*}; want=${n##*:}
        [ "$(rc $name-$e)" != 0 ] || { show $name-$e; fail "$e: $name was not refused"; }
        py '
import json, sys; d = json.load(open(sys.argv[1]))
assert d["return_code"] == int(sys.argv[2]), d
' $name-$e/out/result.json "$want" || { show $name-$e; fail "$e: the $name refusal left no result.json with $want"; }
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

# The two layout modes arrange and return before the slice loop: --layout-plan
# (the versioned contract) and --layout (the older form) are refused with
# --slice (-2), in either flag order, so a --slice run never ends without
# slicing or result.json.
for e in bambu orca; do
    bin=$B; prof="$RES/profiles"
    [ $e = orca ] && { bin=$O; prof="$RES/profiles-orca"; }
    py '
import json, sys
w, e, prof = sys.argv[1], sys.argv[2], sys.argv[3]
machine = "BBL/machine/Bambu Lab X1 Carbon 0.4 nozzle.json"
json.dump({"schemaVersion": 1, "engine": e, "profilesDir": prof,
           "profiles": {"machine": machine}, "spacing": {"minObjectDistanceMm": 10.0},
           "models": [{"id": "a", "path": w + "/cube.stl"}]}, open("plan-%s.json" % e, "w"))
json.dump({"profilesDir": prof, "profiles": {"machine": machine},
           "objects": [{"stl": w + "/cube.stl"}]}, open("old-layout-%s.json" % e, "w"))
' "$WORKDIR" "$e" "$prof"
    run rlpa-$e "$bin" --slice 1 --outputdir rlpa-$e/out --layout-plan --input plan-$e.json
    run rlpb-$e "$bin" --layout-plan --input plan-$e.json --slice 1 --outputdir rlpb-$e/out
    run rold-$e "$bin" --slice 1 --outputdir rold-$e/out --layout old-layout-$e.json
    # Both flags at once is a refusal of the same kind: under --slice it still
    # leaves result.json (-2) like each flag alone, not a bare exit 1.
    run rboth-$e "$bin" --slice 1 --outputdir rboth-$e/out --layout-plan --input plan-$e.json --layout old-layout-$e.json
    run rbothb-$e "$bin" --layout-plan --input plan-$e.json --layout old-layout-$e.json --slice 1 --outputdir rbothb-$e/out
    for n in rlpa rlpb rold rboth rbothb; do
        [ -f $n-$e/out/result.json ] || { show $n-$e; fail "$e: $n with --slice wrote no result.json"; }
        py '
import json, sys; d = json.load(open(sys.argv[1]))
assert d["return_code"] == -2, d
' $n-$e/out/result.json || { show $n-$e; fail "$e: $n with --slice left no result.json with -2"; }
    done
    # Without --slice the sentence and exit 1 are unchanged.
    run rbothn-$e "$bin" --layout-plan --input plan-$e.json --layout old-layout-$e.json
    [ "$(rc rbothn-$e)" = 1 ] && grep -q -- "--layout-plan and --layout are mutually exclusive" rbothn-$e/stderr \
        || { show rbothn-$e; fail "$e: the mutual exclusion without --slice changed"; }
done
echo "PASS: --layout-plan and --layout with --slice are refused with result.json, in either flag order and together (both engines)"

# A named preset never decides how a 3MF that cannot be read is refused: a
# missing or unreadable .3mf gives the same code and sentence with and
# without one (CLI_FILE_NOTFOUND with "No such file: ...", CLI_DATA_FILE_ERROR
# from the 3MF loader). Reading an unreadable 3MF as a project made the
# preset refusal answer first (CLI_INVALID_PARAMS, "a 3MF carries its own
# settings"), which the official command line cannot do: it checks every
# input's existence before it loads one and leaves a file it cannot open to
# the loader (BambuStudio.cpp 1855-1860; OrcaSlicer.cpp 1537-1542).
py 'open("corrupt.3mf", "wb").write(b"PK\x03\x04 this is not a zip archive\n")'
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run rmf-$e "$bin" missing.3mf --printer-preset "$A1M" --slice 1 --outputdir rmf-$e/out
    run rmfn-$e "$bin" missing.3mf --slice 1 --outputdir rmfn-$e/out
    run rcf-$e "$bin" corrupt.3mf --printer-preset "$A1M" --slice 1 --outputdir rcf-$e/out
    run rcfn-$e "$bin" corrupt.3mf --slice 1 --outputdir rcfn-$e/out
    grep -q 'No such file: missing.3mf' rmf-$e/stderr \
        || { show rmf-$e; fail "$e: a missing 3MF with a named preset does not name the file"; }
    # result.json is the record, as for every other refusal: the shell's own view
    # of a native status is not portable (Git Bash reports -3, 0xFFFFFFFD, as 127
    # where a POSIX shell reports 253), so the 0..255 view is compared only where
    # the shell is a POSIX one.
    case "$(uname -s)" in
    MINGW*|MSYS*|CYGWIN*) ;;
    *) [ "$(rc rmf-$e)" = 253 ] || { show rmf-$e; fail "$e: a missing 3MF with a named preset exit $(rc rmf-$e), want 253 (-3)"; }
       [ "$(rc rcf-$e)" = 250 ] || { show rcf-$e; fail "$e: an unreadable 3MF with a named preset exit $(rc rcf-$e), want 250 (-6)"; };;
    esac
    py '
import json, sys
m, mn, c, cn = [json.load(open(p)) for p in sys.argv[1:5]]
assert m["return_code"] == mn["return_code"] == -3, ("missing", m["return_code"], mn["return_code"])
assert m["error_string"] == mn["error_string"], (m["error_string"], mn["error_string"])
assert c["return_code"] == cn["return_code"] == -6, ("corrupt", c["return_code"], cn["return_code"])
assert c["error_string"] == cn["error_string"], (c["error_string"], cn["error_string"])
' rmf-$e/out/result.json rmfn-$e/out/result.json rcf-$e/out/result.json rcfn-$e/out/result.json \
        || { show rcf-$e; fail "$e: an unreadable 3MF with a named preset is not the no-preset refusal"; }
done
echo "PASS: a missing or unreadable 3MF gives the same code and sentence with and without a named preset (both engines)"

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

# A second --export-stl / --export-stls into the same target replaces the
# first: the STL is written beside the target and renamed over it, and the
# rename replaces an existing file on every OS (boost::filesystem::rename:
# rename(2) on POSIX, MoveFileExW with MOVEFILE_REPLACE_EXISTING on Windows,
# boost 1.90 operations.cpp 236). The second run scales the cube, so its file
# differs from the first.
reexp_before=$FAILS
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    for mode in stl stls; do
        d=reexp-$mode-$e; rm -rf $d
        for s in 1 2; do
            if [ $mode = stl ]; then
                run $d-$s "$bin" cube.stl --printer-preset "$A1M" --scale $s --export-stl --outputdir $d
            else
                run $d-$s "$bin" cube.stl --printer-preset "$A1M" --scale $s --export-stls $d/stls --outputdir $d
            fi
            [ "$(rc $d-$s)" = 0 ] || { show $d-$s; fail "$e: --export-$mode run $s into the same target exit $(rc $d-$s)"; }
            py '
import hashlib, os, sys
files = [os.path.join(r, n) for r, _, ns in os.walk(sys.argv[1]) for n in ns if n.endswith(".stl")]
left = [os.path.join(r, n) for r, _, ns in os.walk(sys.argv[1]) for n in ns if n.endswith(".writing")]
assert len(files) == 1 and not left, (files, left)
open(sys.argv[2], "w").write(hashlib.sha1(open(files[0], "rb").read()).hexdigest())
' $d $d-$s.sha || fail "$e: --export-$mode run $s did not leave exactly one STL"
        done
        [ -s $d-1.sha ] && [ -s $d-2.sha ] && ! cmp -s $d-1.sha $d-2.sha ||
            fail "$e: the second --export-$mode did not replace the first STL"
    done
done
[ "$FAILS" = "$reexp_before" ] && echo "PASS: a second --export-stl or --export-stls into the same target replaces the first (both engines)"

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

# Named presets give each filament its own preset's values, as the desktop's
# full_fff_config does (OrcaSlicer Plater.cpp 7959-7963 slices with
# full_config(false)): an H2D with four named filaments prints each with its
# own nozzle temperature. Each filament's value comes from a run of that
# filament alone.
F4N=("Bambu PLA Basic @BBL H2D" "Bambu PLA Matte @BBL H2D" "Bambu PETG HF @BBL H2D 0.4 nozzle" "Bambu ABS @BBL H2D")
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    want=""
    for f in "${F4N[@]}"; do
        run e1one-$e "$bin" cube.stl --printer-preset "Bambu Lab H2D 0.4 nozzle" --filament-preset "$f" --export-settings e1one-$e.json
        want="$want,$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["nozzle_temperature"][0])' e1one-$e.json)"
    done
    want=${want#,}
    args=(); for f in "${F4N[@]}"; do args+=(--filament-preset "$f"); done
    run e1four-$e "$bin" cube.stl --slice 1 --printer-preset "Bambu Lab H2D 0.4 nozzle" "${args[@]}" --outputdir e1four-$e/out
    [ "$(rc e1four-$e)" = 0 ] || { show e1four-$e; fail "$e: H2D with four named filaments exit $(rc e1four-$e)"; }
    got=$(sed -n 's/^; nozzle_temperature = //p' e1four-$e/out/plate_1.gcode)
    [ "$got" = "$want" ] || fail "$e: four named filaments: nozzle_temperature $got, each filament's own $want"
done
echo "PASS: named filament presets keep each filament's own values on a two-extruder printer (both engines)"

# The PR #35 findings F1-F10 (and F7o), each a check of what the official
# command line does, on both engines: tests/test-pr35-findings.sh runs every
# block and lists each result.
if bash "$SCRIPT_DIR/test-pr35-findings.sh" "$B" "$O" > findings.log 2>&1; then
    sed -n '/^== summary/,$p' findings.log | grep -E '^(PASS|SKIP)'
else
    sed -n '/^== summary/,$p' findings.log
    fail "PR #35 findings (tests/test-pr35-findings.sh)"
fi

# Every refusal sentence reworded for the owner rule (the sentence says what
# is wrong AND what to do) carries its "do" part in result.json's
# error_string: tests/test_refusal_sentences.sh.
if bash "$SCRIPT_DIR/test_refusal_sentences.sh" "$B" "$O" > refusals.log 2>&1; then
    grep -E '^(PASS|SKIP)' refusals.log
else
    grep -E '^FAIL' refusals.log
    fail "refusal sentences (tests/test_refusal_sentences.sh)"
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
# The package's resources: beside the binary (the macOS and Windows release
# packages, slicer_cli and resources/ side by side) or one level up (a bin/
# layout, the box packages and the Linux release package). Same resolution as
# test-pr35-findings.sh and test-product-invariants.sh.
# flat_presets ENGINE TAG PRINTER [VENDOR]: TAG-ENGINE-machine.json,
# -process.json and -filament.json (the printer's default process and
# filament). VENDOR (a directory holding machine/, process/, filament/) picks
# another vendor's tree than BBL, e.g. $RES/profiles-orca/Snapmaker.
flat_presets() {
    local vendor="$RES/profiles/BBL"
    [ "$1" = orca ] && vendor="$RES/profiles-orca/BBL"
    [ -z "${4:-}" ] || vendor="$4"
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
# E3: an H2D STL with the printer's settings files gets the official CLI's
# extruder variants, flush settings and filament map. Measured with the
# official CLIs (BambuStudio, and OrcaSlicer 2.4.0-alpha, the pins' versions)
# on the same files (--load-settings):
#  - the --machine/--process/--filament loader joined a string list with ','
#    (one variant of seven; the second extruder found none: nozzle_volume
#    130,130); the engines' loader writes a ';' list (Config.cpp 1022-1040);
#  - the flush volumes were recomputed for the engine's default colour, which
#    the official m_print_config does not hold for model files: the flush
#    matrix became 0,0 (official: the printer's own), and the single filament
#    went to extruder 2 (official 1) (BambuStudio.cpp 3760-3771, 4115).
# Official values: Bambu filament_map 1, Orca 2; nozzle_volume 130,145; the
# printer's flush_volumes_matrix (16 values); flush_multiplier one value
# (Bambu 1, Orca 0.3); wipe_tower_x/y and the print sequences one value.
e3_ok=1
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    flat_presets $e e3h2d "Bambu Lab H2D 0.4 nozzle"
    run e3l-$e "$bin" cube.stl --load-settings "e3h2d-$e-machine.json;e3h2d-$e-process.json" \
        --load-filaments e3h2d-$e-filament.json --slice 1 --outputdir e3l-$e/out
    run e3m-$e "$bin" cube.stl -o e3m-$e.gcode --machine e3h2d-$e-machine.json \
        --filament e3h2d-$e-filament.json --process e3h2d-$e-process.json
    for n in l m; do
        g=e3l-$e/out/plate_1.gcode; [ $n = m ] && g=e3m-$e.gcode
        [ "$(rc e3$n-$e)" = 0 ] && [ -s $g ] || { show e3$n-$e; fail "E3 $e: H2D STL with settings files (route $n) exit $(rc e3$n-$e)"; e3_ok=0; continue; }
        py '
import re, sys
g = open(sys.argv[1], errors="replace").read(); e = sys.argv[2]
def hdr(k):
    return re.findall(r"^; " + re.escape(k) + r" = (.*)$", g, re.M)
want = {"filament_map": "1" if e == "bambu" else "2", "nozzle_volume": "130,145",
        "flush_multiplier": "1" if e == "bambu" else "0.3"}
bad = ["%s = %s (official %s)" % (k, hdr(k), v) for k, v in want.items() if hdr(k)[-1:] != [v]]
for k in ("wipe_tower_x", "wipe_tower_y", "flush_multiplier_fast", "first_layer_print_sequence", "other_layers_print_sequence"):
    if any("," in v for v in hdr(k)):
        bad.append("%s = %s (official: one value)" % (k, hdr(k)))
m = hdr("flush_volumes_matrix")
if not m or len(m[-1].split(",")) != 16:
    bad.append("flush_volumes_matrix = %s (official: the printer 16 values)" % m)
assert not bad, "; ".join(bad)
' $g $e || { fail "E3 $e: route $n differs from the official CLI"; e3_ok=0; }
    done
done
[ $e3_ok = 1 ] && echo "PASS: E3 an H2D STL with settings files gets the official CLI's variants, flush settings and filament map (both engines)"

# E4: a setting with one value per plate or per nozzle is not padded with a 0
# to the extruder count (the official leaves each as given; get_at() reads a
# short list's first value, Config.hpp 681-685). A three-plate, two-filament
# H2D project whose wipe_tower_x/y hold ONE value: every plate's tower stands
# where the official CLI puts it, not at (0, 0) and not at the one value the
# list carries. A plate the list holds no entry for stands on the option's own
# default, wipe_tower_x 15 / wipe_tower_y 220 (PrintConfig.cpp 5979-5993), the
# front-left corner of the tower, while a plate the list does name uses that
# value - here 165/250, the arrange default our own export writes per plate
# (WIPE_TOWER_DEFAULT_X_POS / _Y_POS, PartPlate.cpp 68-69).
# Measured with the official CLI (BambuStudio 02.08.01.55) on the two-plate
# form of this fixture with wipe_tower_x/y = [100]/[120]: its plate-2 tower's
# first layer runs X13.609..193.359 Y248.179..277.006, the corner of 15/220
# (X13.62 Y218.14), not of 100/120 (which would put it near X98.6 Y118.6) and
# not of a padded 0 (near X0 Y0). Our build's same corner: X13.624 Y218.139.
# So the check derives the tower's own footprint offset (its brim) from the
# full-list control of the same plate, and requires the one-value corner to
# land on the option default minus that offset. On the pre-E4 build (933ff72)
# the padded 0 put it at (0.028, 0.046) instead, which is what this fails on.
e4_ok=1
py '
import json
P = lambda n: {"plate_name": n, "need_arrange": True, "objects": [{"path": "cube.stl", "count": 2, "filaments": [1, 2]}]}
json.dump({"plates": [P("p"), P("q"), P("r")]}, open("e4.json", "w"))
'
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run e4mk-$e "$bin" --load-assemble-list e4.json --slice 0 --printer-preset "Bambu Lab H2D 0.4 nozzle" \
        --filament-preset "Bambu PLA Basic @BBL H2D" --filament-preset "Bambu PLA Matte @BBL H2D" \
        --filament-colour "#FF0000;#00FF00" --outputdir e4mk-$e/out --export-3mf e4.3mf
    [ "$(rc e4mk-$e)" = 0 ] && [ -s e4mk-$e/out/e4.3mf ] || { show e4mk-$e; fail "E4 $e: three-plate H2D project exit $(rc e4mk-$e)"; e4_ok=0; continue; }
    # The control, the project's own full per-plate list, for the same plates.
    for p in 2 3; do
        run e4c$p-$e "$bin" e4mk-$e/out/e4.3mf --plate $p -o e4c$p-$e.gcode
        [ "$(rc e4c$p-$e)" = 0 ] && [ -s e4c$p-$e.gcode ] || { show e4c$p-$e; fail "E4 $e: control plate $p exit $(rc e4c$p-$e)"; e4_ok=0; }
    done
    py '
import json, sys, zipfile
with zipfile.ZipFile(sys.argv[1]) as zin, zipfile.ZipFile(sys.argv[2], "w", zipfile.ZIP_DEFLATED) as zout:
    for item in zin.infolist():
        data = zin.read(item.filename)
        if item.filename == "Metadata/project_settings.config":
            d = json.loads(data); d.update({"wipe_tower_x": ["100"], "wipe_tower_y": ["120"]})
            data = json.dumps(d, indent=4).encode()
        if item.filename.startswith("Metadata/plate_") and item.filename.endswith((".gcode", ".md5")):
            continue
        zout.writestr(item, data)
' e4mk-$e/out/e4.3mf e4-$e.3mf
    for p in 2 3; do
        run e4s$p-$e "$bin" e4-$e.3mf --plate $p -o e4s$p-$e.gcode
        [ "$(rc e4s$p-$e)" = 0 ] && [ -s e4s$p-$e.gcode ] || { show e4s$p-$e; fail "E4 $e: plate $p exit $(rc e4s$p-$e)"; e4_ok=0; }
    done
    [ $e4_ok = 1 ] || continue
    py '
import re, sys
def tower_corner(path):
    g = open(path, errors="replace").read()
    m = re.search(r"; FEATURE: (?:Prime|Wipe) tower\n", g)
    assert m, ("no tower", path)
    j = g.find("\n; FEATURE:", m.end())
    blk = g[m.end(): j if j > 0 else len(g)]
    xs = [float(v) for v in re.findall(r"\bX([0-9.]+)", blk)]
    ys = [float(v) for v in re.findall(r"\bY([0-9.]+)", blk)]
    assert xs and ys, ("empty tower", path)
    return min(xs), min(ys)
c2, c3, s2, s3 = [tower_corner(p) for p in sys.argv[1:5]]
bad = []
for n, c, s in (("2", c2, s2), ("3", c3, s3)):
    if abs(c[0] - 163.62) > 1 or abs(c[1] - 248.14) > 1:
        bad.append("plate %s: the full-list control is not the 165/250 default, corner (%.3f, %.3f)" % (n, c[0], c[1]))
        continue
    bx, by = 165.0 - c[0], 250.0 - c[1]          # the tower footprint offset (brim)
    ex, ey = 15.0 - bx, 220.0 - by               # the option default, minus that offset
    if abs(s[0] - ex) > 1 or abs(s[1] - ey) > 1:
        bad.append("plate %s: one-value corner (%.3f, %.3f), want the option default 15/220 minus the brim (%.3f, %.3f)" % (n, s[0], s[1], ex, ey))
g = open(sys.argv[3], errors="replace").read()
for k in ("wipe_tower_x", "wipe_tower_y"):
    for v in re.findall(r"^; " + k + r" = (.*)$", g, re.M):
        if "," in v:
            bad.append("%s = %s in the header (one value per plate)" % (k, v))
assert not bad, "; ".join(bad)
' e4c2-$e.gcode e4c3-$e.gcode e4s2-$e.gcode e4s3-$e.gcode || { fail "E4 $e: a plate without its own wipe_tower entry does not stand on the option's default corner"; e4_ok=0; }
done
[ $e4_ok = 1 ] && echo "PASS: E4 one-value settings are not padded with 0; a plate without its entry stands on the option's default tower corner (both engines)"

# inherits_group and different_settings_to_system carry one entry per preset in
# the project's order - process, filament 1 .. filament N, printer - or are
# absent; the desktop writes the key only when some entry is non-empty
# (PresetBundle::full_config, PresetBundle.cpp 3490-3523, add_if_some_non_empty).
# The official CLI reads the list as N + 2 entries: it sizes the filament system
# names to size - 2 and takes each filament's from it (BambuStudio.cpp 2001-2046),
# so a two-entry list is an out-of-bounds read - SIGSEGV (rc 139) on every
# --slice N of the exported project. The command line's named-preset path (no
# settings file and no model file to name the filaments) left the settings merge
# with filament_count 0 and wrote exactly that two-entry list; on the pre-fix
# build this fails with inherits_group and different_settings_to_system both
# ['', ''].
inh_ok=1
py '
import json
json.dump({"plates": [{"plate_name": "p", "need_arrange": True,
                       "objects": [{"path": "cube.stl", "count": 2, "filaments": [1, 2]}]}]}, open("inh.json", "w"))
'
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run inh-$e "$bin" --load-assemble-list inh.json --slice 0 --printer-preset "Bambu Lab H2D 0.4 nozzle" \
        --filament-preset "Bambu PLA Basic @BBL H2D" --filament-preset "Bambu PLA Matte @BBL H2D" \
        --filament-colour "#FF0000;#00FF00" --outputdir inh-$e/out --export-3mf inh-$e.3mf
    [ "$(rc inh-$e)" = 0 ] && [ -s inh-$e/out/inh-$e.3mf ] || { show inh-$e; fail "inherits $e: two-filament H2D project exit $(rc inh-$e)"; inh_ok=0; continue; }
    py '
import json, sys, zipfile
ps = json.loads(zipfile.ZipFile(sys.argv[1]).read("Metadata/project_settings.config"))
n = len(ps["filament_settings_id"])
sizes, bad = {}, []
for k in ("inherits_group", "different_settings_to_system"):
    v = ps.get(k)
    if v is None:
        continue
    sizes[k] = len(v)
    if len(v) != n + 2:
        bad.append("%s has %d entries, want %d (process, %d filament(s), printer)" % (k, len(v), n + 2, n))
    elif any(entry != "" for entry in v):
        bad.append("%s = %r for system presets" % (k, v))
if len(sizes) == 2 and sizes["inherits_group"] != sizes["different_settings_to_system"]:
    bad.append("the two lists differ in length: %r" % sizes)
assert not bad, "; ".join(bad)
' inh-$e/out/inh-$e.3mf || { fail "inherits $e: the exported project's inherits_group/different_settings_to_system are not one entry per preset"; inh_ok=0; }
done
[ $inh_ok = 1 ] && echo "PASS: a two-filament H2D project exports inherits_group and different_settings_to_system as one entry per preset, or absent (both engines)"

# ── A model file loaded after a non-empty plate keeps off that plate ──────
# The desktop's plate-empty test is the whole model
# (partplate_list.get_curr_plate()->empty(), BambuStudio Plater.cpp 9646), and
# the cell a new object gets is empty for EVERY object of the model
# (GLCanvas3D::get_empty_cells walks m_model->objects, GLCanvas3D.cpp
# 6744-6778) — the project's own, placed by the file, included. The command
# line's placement kept only the objects it gave an instance to, so an STL
# after a project 3MF was placed as if the bed were empty, on the bed centre,
# on top of the project's geometry: the plate's printed footprint was the
# project's alone. Both engines.
occ_ok=1
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run occ1-$e "$bin" base.3mf --plate 1 -o occ1-$e.gcode
    [ "$(rc occ1-$e)" = 0 ] || { show occ1-$e; fail "$e: the project alone exit $(rc occ1-$e)"; occ_ok=0; continue; }
    run occ2-$e "$bin" base.3mf cube.stl --plate 1 -o occ2-$e.gcode
    [ "$(rc occ2-$e)" = 0 ] || { show occ2-$e; fail "$e: project + STL exit $(rc occ2-$e)"; occ_ok=0; continue; }
    py '
import re, sys
def box(path):
    g = open(path, errors="replace").read()
    xs = [float(v) for v in re.findall(r"^G1 [^;\n]*X(-?[0-9.]+)[^;\n]*E[0-9.]", g, re.M)]
    ys = [float(v) for v in re.findall(r"^G1 [^;\n]*Y(-?[0-9.]+)[^;\n]*E[0-9.]", g, re.M)]
    assert xs and ys, ("no extrusion in " + path)
    return min(xs), max(xs), min(ys), max(ys)
one, two = box(sys.argv[1]), box(sys.argv[2])
grew = max(one[0] - two[0], two[1] - one[1], one[2] - two[2], two[3] - one[3])
assert grew >= 5.0, ("the STL did not move the plate footprint: project %r, with the STL %r (grew %.2f mm)" % (one, two, grew))
' occ1-$e.gcode occ2-$e.gcode || { fail "$e: the STL loaded after the project 3MF is placed on the project's own geometry"; occ_ok=0; }
done
if [ $occ_ok = 1 ]; then echo "PASS: a model file after a project 3MF is placed clear of the project's objects (both engines)"; fi

# ── The sweep reads the marker beside the folder, rules unchanged ─────────
# The desktop judges a staging folder stale from a PID written in the folder
# itself (has_restore_data, bbs_3mf.cpp 9447-9466 / OrcaSlicer 9013-9031);
# this program writes that PID into <folder>.owner, beside the folder, and the
# sweep reads only that — the folder's own lock.txt is the name the engine's
# backup lock uses (Model::set_backup_path writes it for a folder it creates,
# Model.cpp 1123-1130 / OrcaSlicer 986-993, and the restore path rewrites it,
# bbs_3mf.cpp 1463-1471). The three rules lane-tmpkill proved hold on the new
# marker: a dead PID's folder goes, a damaged marker counts as not-an-owner,
# and a folder with no marker at all is left alone inside the age window.
# A Windows build takes its temp folder from TMP/TEMP (GetTempPathW, which
# boost's temp_directory_path calls), not TMPDIR, and wants it in D:\ form.
winpath() { case "$(uname -s)" in MINGW*|MSYS*|CYGWIN*) cygpath -w "$1";; *) printf '%s' "$1";; esac; }
RACE=race-rules; rm -rf "$RACE"; mkdir -p "$RACE"
sweep_before=$FAILS
mkfolder() {  # mkfolder NAME [MARKER]
    mkdir -p "$RACE/slicer_cli_load-$1"
    : > "$RACE/slicer_cli_load-$1/plate_1.gcode"
    if [ $# -ge 2 ]; then printf '%s' "$2" > "$RACE/slicer_cli_load-$1.owner"; fi
    return 0
}
mkfolder dead 999999        # a PID no process of this program has
mkfolder damaged not-a-pid  # a marker that does not parse
mkfolder young              # no marker at all, made now
# Every other temp folder the run makes (make_owned_temp_dir: the Bambu
# build's percent line-width copy, the preset stagings, --list-presets) carries
# the same marker, so a killed run's leftover of each goes too.
for p in percent named plate list project presets; do
    mkdir -p "$RACE/slicer_cli_$p-dead"
    : > "$RACE/slicer_cli_$p-dead/x.3mf"
    printf '%s' 999999 > "$RACE/slicer_cli_$p-dead.owner"
done
# A run killed between writing its marker and renaming it into place leaves
# <folder>.owner.tmp and no marker: the age rule takes the folder, and the
# half-written marker with it. Two hours old, past the one-hour window.
mkdir -p "$RACE/slicer_cli_load-halfmark"
printf '%s' 999999 > "$RACE/slicer_cli_load-halfmark.owner.tmp"
py '
import os, sys, time
t = time.time() - 7200
for p in sys.argv[1:]:
    os.utime(p, (t, t))
' "$RACE/slicer_cli_load-halfmark" "$RACE/slicer_cli_load-halfmark.owner.tmp"
run race-rules env TMPDIR="$RACE" TMP="$(winpath "$PWD/$RACE")" TEMP="$(winpath "$PWD/$RACE")" "$B" cube.stl --slice 1 --printer-preset "$A1M" --outputdir race-rules/out
[ "$(rc race-rules)" = 0 ] || { show race-rules; fail "a run beside the stale folders exit $(rc race-rules)"; }
[ ! -d "$RACE/slicer_cli_load-dead" ] || fail "the sweep kept a folder whose marker names a dead PID"
[ ! -e "$RACE/slicer_cli_load-dead.owner" ] || fail "the sweep kept the marker of a folder it removed"
[ ! -d "$RACE/slicer_cli_load-damaged" ] || fail "the sweep kept a folder whose marker does not parse"
[ -d "$RACE/slicer_cli_load-young" ] || fail "the sweep removed a folder with no marker inside the age window"
for p in percent named plate list project presets; do
    [ ! -e "$RACE/slicer_cli_$p-dead" ] && [ ! -e "$RACE/slicer_cli_$p-dead.owner" ] ||
        fail "the sweep kept a slicer_cli_$p- folder whose marker names a dead PID"
done
[ ! -e "$RACE/slicer_cli_load-halfmark" ] && [ ! -e "$RACE/slicer_cli_load-halfmark.owner.tmp" ] ||
    fail "the sweep kept an old folder whose marker was left half-written (.owner.tmp)"
[ "$FAILS" = "$sweep_before" ] && echo "PASS: the sweep reads the marker beside the folder (dead PID and damaged marker go, a marker-less folder stays; every temp folder prefix; a half-written marker goes with its old folder)"

# ── Runs at once in one temp folder ───────────────────────────────────────
# Every run sweeps the shared temp folder as it starts, while the others make
# their staging folders (load-, presets-, named-, plate-) and write their
# markers. A marker is written whole (<folder>.owner.tmp renamed onto
# <folder>.owner), so a sweep never reads a live run's marker half-written and
# removes its folder. Six runs at once, twice: every one slices, and nothing of
# theirs is left behind.
HAM=hammer-tmp; rm -rf "$HAM"; mkdir -p "$HAM"
ham_before=$FAILS
for round in 1 2; do
    pids=""
    for k in 1 2 3 4 5 6; do
        n=ham-$round-$k
        mkdir -p $n
        if [ $((k % 2)) = 1 ]; then
            env TMPDIR="$HAM" TMP="$(winpath "$PWD/$HAM")" TEMP="$(winpath "$PWD/$HAM")" "$B" base.3mf --slice 1 --outputdir $n/out > $n/stdout 2> $n/stderr &
        else
            env TMPDIR="$HAM" TMP="$(winpath "$PWD/$HAM")" TEMP="$(winpath "$PWD/$HAM")" "$B" cube.stl --slice 1 --printer-preset "$A1M" --process-preset "$A1M_PROCESS" --filament-preset "$A1M_FILAMENT" --outputdir $n/out > $n/stdout 2> $n/stderr &
        fi
        pids="$pids $!"
    done
    k=0
    for p in $pids; do
        k=$((k + 1)); n=ham-$round-$k
        r=0; wait $p || r=$?
        [ "$r" = 0 ] && [ -s $n/out/plate_1.gcode ] || { show $n; fail "run $k of $round in the shared temp folder exit $r"; }
    done
done
left=$(ls "$HAM" 2>/dev/null | grep '^slicer_cli_' || true)
[ -z "$left" ] || fail "runs at once left in their temp folder: $left"
[ "$FAILS" = "$ham_before" ] && echo "PASS: six runs at once in one temp folder all slice and leave nothing behind, twice (Bambu build)"

# ── A live run's ownership marker is out of the extraction folder ─────────
# A second run sweeping while the first is alive must leave the live run's
# folder alone. The marker sat at <folder>/lock.txt, inside the folder the 3MF
# is extracted into: a marker there is only as trustworthy as every writer of
# that path. The loader's extraction is name-filtered — Metadata/*.gcode,
# *.png, *.md5, the project/model config, Auxiliaries/… (bbs_3mf.cpp
# 1933-2053 at 5873b5f) — so no archive member reaches <folder>/lock.txt at
# these pins; this case writes the dead PID itself, standing in for the writer
# of that path, and holds the first run while the second one starts.
case "$(uname -s)" in
MINGW*|MSYS*|CYGWIN*) echo "SKIP: the ownership marker race (Windows)";;
*)
py '
import zipfile
with zipfile.ZipFile("base.3mf") as zin, zipfile.ZipFile("tainted.3mf", "w", zipfile.ZIP_DEFLATED) as zout:
    for item in zin.infolist():
        zout.writestr(item, zin.read(item.filename))
    zout.writestr("lock.txt", "999999")   # the PID a writer of <folder>/lock.txt leaves
'
RACE=race-tmp; rm -rf "$RACE"; mkdir -p "$RACE" race-a
py '
v = [(x, y, z) for z in (0, 20) for y in (0, 20) for x in (0, 20)]
f = [(0,2,1),(1,2,3),(4,5,6),(5,7,6),(0,1,4),(1,5,4),(2,6,3),(3,6,7),(0,4,2),(2,4,6),(1,3,5),(3,7,5)]
with open("objfeed.obj", "w") as o:
    for p in v: o.write("v %g %g %g\n" % p)
    for a, b, c in f: o.write("f %d %d %d\n" % (a + 1, b + 1, c + 1))
'
# The first run holds on a FIFO it reads as its second model file: it has made
# its staging folder and read its project by then, and it stays alive until
# the test feeds it one. An OBJ, not an STL: the STL reader opens the file
# more than once, and the second open reads the tail of the first feed.
# A model read from a pipe is not a path a user's file takes: if run A ever
# fails here inside the OBJ reader on a partial read of the feed, that is this
# harness, not the sweep. CI 7c59ddc saw run A exit 139 once, right after it
# loaded the feed; the case passed 70 of 70 times on the Linux box, and the
# package job now prints the backtrace of any crashed run (slicer-cli-ci.yml).
mkfifo "$RACE/slow.obj"
TMPDIR="$RACE" TMP="$(winpath "$PWD/$RACE")" TEMP="$(winpath "$PWD/$RACE")" "$B" tainted.3mf "$RACE/slow.obj" --plate 1 -o race-a/out.gcode > race-a/stdout 2> race-a/stderr &
apid=$!
folder=""
for i in $(seq 1 200); do
    folder=$(ls -d "$RACE"/slicer_cli_load-*/ 2>/dev/null | head -1 || true)
    folder=${folder%/}
    if [ -n "$folder" ]; then break; fi
    sleep 0.05
done
[ -n "$folder" ] || { kill -KILL $apid 2>/dev/null || true; fail "run A made no staging folder"; }
[ -f "$folder.owner" ] || { kill -KILL $apid 2>/dev/null || true; fail "run A keeps no ownership marker beside its staging folder"; }
echo 999999 > "$folder/lock.txt"   # what the marker was, and who else writes it
run race-b env TMPDIR="$RACE" TMP="$(winpath "$PWD/$RACE")" TEMP="$(winpath "$PWD/$RACE")" "$B" cube.stl --slice 1 --printer-preset "$A1M" --outputdir race-b/out
[ "$(rc race-b)" = 0 ] || { kill -KILL $apid 2>/dev/null || true; show race-b; fail "run B exit $(rc race-b)"; }
if [ ! -d "$folder" ]; then
    kill -KILL $apid 2>/dev/null || true
    fail "the second run's sweep removed the staging folder of the live run"
fi
# Let the first run finish: every open of the FIFO is served, so it ends the
# way an ordinary run does — its folder intact, its G-code written. A python
# feeder, not a shell loop: it dies on the first signal, so no feeder of this
# case can outlive the script and hold its output pipe open. A reader that
# closes while the feeder still writes (the loader reads its feeds, the next
# write finds it gone) is a broken pipe, not the end: run A opens the OBJ again
# (the output check reads its mtllib names), and that open must be served too.
py '
import sys
data = open("objfeed.obj", "rb").read()
while True:
    try:
        with open(sys.argv[1], "wb") as f:
            f.write(data)
    except BrokenPipeError:
        continue
    except OSError:
        break
' "$RACE/slow.obj" &
feeder=$!
arc=hung
for i in $(seq 1 150); do
    if ! kill -0 $apid 2>/dev/null; then arc=0; wait $apid || arc=$?; break; fi
    sleep 0.2
done
if [ "$arc" = hung ]; then kill -KILL $apid 2>/dev/null || true; wait $apid 2>/dev/null || true; fi
kill $feeder 2>/dev/null || true; wait $feeder 2>/dev/null || true
# A run's own cleanup on the way out is the private-TMPDIR case's business
# ("the run left folders in the temp dir it was given") and leakcheck's; this
# case is about the folder the second run must leave alone.
[ "$arc" = 0 ] || { show race-a; fail "run A exit $arc after the second run's sweep"; }
[ -s race-a/out.gcode ] || fail "run A wrote no G-code after the second run's sweep"
echo "PASS: a live run's ownership marker is beside the folder, out of the extraction root (both engines)"
;;
esac
# ── A live run of the other engine owns its folder too ────────────────────
# The desktop's staleness test compares the folder owner's process name with
# its own (has_restore_data, bbs_3mf.cpp 9447-9466 / OrcaSlicer 9013-9031):
# one program. This package ships two binaries — slicer_cli for the Bambu
# engine and slicer_cli-orcaslicer for OrcaSlicer (the names the package and
# the updater's engine convention give them, slicer-cli-ci.yml 1390,
# 1438-1439) — and one host may run both against the same TMPDIR. Comparing
# the two names alone made a run of one call the other's LIVE folder stale and
# remove it, with the loader still holding it: it then read back a settings
# file the sweep had taken away ("load_from_json: parse
# <tmp>/slicer_cli_load-.../_temp_3.config got a parse_error", rc 255). A live
# pid of EITHER binary is an owner now.
case "$(uname -s)" in
MINGW*|MSYS*|CYGWIN*) echo "SKIP: the cross-engine owner check (Windows)";;
*)
py '
v = [(x, y, z) for z in (0, 20) for y in (0, 20) for x in (0, 20)]
f = [(0,2,1),(1,2,3),(4,5,6),(5,7,6),(0,1,4),(1,5,4),(2,6,3),(3,6,7),(0,4,2),(2,4,6),(1,3,5),(3,7,5)]
with open("objfeed2.obj", "w") as o:
    for p in v: o.write("v %g %g %g\n" % p)
    for a, b, c in f: o.write("f %d %d %d\n" % (a + 1, b + 1, c + 1))
'
RACE=race-engine; rm -rf "$RACE"; mkdir -p "$RACE" race-orca
# The OrcaSlicer run holds on a FIFO it reads as its second model file: its
# staging folder and its marker are there while the Bambu run sweeps.
mkfifo "$RACE/slow.obj"
TMPDIR="$RACE" TMP="$(winpath "$PWD/$RACE")" TEMP="$(winpath "$PWD/$RACE")" "$O" base.3mf "$RACE/slow.obj" --slice 1 --outputdir race-orca/out > race-orca/stdout 2> race-orca/stderr &
opid=$!
folder=""
for i in $(seq 1 200); do
    folder=$(ls -d "$RACE"/slicer_cli_load-*/ 2>/dev/null | head -1 || true)
    folder=${folder%/}
    if [ -n "$folder" ]; then break; fi
    sleep 0.05
done
[ -n "$folder" ] || { kill -KILL $opid 2>/dev/null || true; fail "the OrcaSlicer run made no staging folder"; }
run race-engine "$B" cube.stl --slice 1 --printer-preset "$A1M" --outputdir race-engine/out
if [ "$(rc race-engine)" != 0 ]; then kill -KILL $opid 2>/dev/null || true; show race-engine; fail "the Bambu run exit $(rc race-engine)"; fi
if [ ! -d "$folder" ] || [ ! -f "$folder.owner" ]; then
    kill -KILL $opid 2>/dev/null || true
    fail "the Bambu run's sweep removed the live OrcaSlicer run's staging folder"
fi
py '
import sys
data = open("objfeed2.obj", "rb").read()
while True:
    try:
        with open(sys.argv[1], "wb") as f:
            f.write(data)
    except OSError:
        break
' "$RACE/slow.obj" &
feeder=$!
arc=hung
for i in $(seq 1 150); do
    if ! kill -0 $opid 2>/dev/null; then arc=0; wait $opid || arc=$?; break; fi
    sleep 0.2
done
if [ "$arc" = hung ]; then kill -KILL $opid 2>/dev/null || true; wait $opid 2>/dev/null || true; fi
kill $feeder 2>/dev/null || true; wait $feeder 2>/dev/null || true
[ "$arc" = 0 ] || { show race-orca; fail "the OrcaSlicer run exit $arc beside the Bambu run"; }
[ -s race-orca/out/plate_1.gcode ] || fail "the OrcaSlicer run wrote no G-code after the Bambu run's sweep"
echo "PASS: a live run of the other engine keeps its staging folder (both engines)"
;;
esac
# ---------------------------------------------------------------- Stage F
# Shorthand flags that must do what the desktop app does, and refusals that
# name the real cause instead of a symptom (input-path audit: SILENT-WRONG
# 18-22, BAD-REFUSAL 3, 7, 8, 9). The settings files are the package's own
# system presets, flattened over their "inherits" chain (flat_presets above),
# as the files a user exports are.

# F18: an STL with settings files takes the plate type of the printer preset
# those settings belong to, as the desktop app does when a printer is picked
# (Sidebar::update_all_preset_comboboxes -> set_bed_type_accord_combox,
# BambuStudio Plater.cpp 3340-3414 and 3703-3715 at 5873b5f; OrcaSlicer
# Plater.cpp 2527-2556 and 2798-2806 at 31f6803). On 2a12432 the engine's own
# default was used instead (curr_bed_type's ConfigDef default, btPC "Cool
# Plate", BambuStudio PrintConfig.cpp 1162 at 5873b5f), which is no printer's.
f18_ok=1
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    flat_presets $e f18 "$A1M"
    run f18n-$e "$bin" cube.stl --slice 1 --printer-preset "$A1M" --outputdir f18n-$e/out
    [ "$(rc f18n-$e)" = 0 ] || { show f18n-$e; fail "F18 $e: the named printer preset exit $(rc f18n-$e)"; f18_ok=0; continue; }
    want=$(hdr f18n-$e/out/plate_1.gcode curr_bed_type)
    [ -n "$want" ] || { fail "F18 $e: the named printer preset states no plate type"; f18_ok=0; continue; }
    # the file route the product uses
    run f18l-$e "$bin" cube.stl --slice 1 --load-settings "f18-$e-machine.json;f18-$e-process.json" \
        --load-filaments f18-$e-filament.json --outputdir f18l-$e/out
    [ "$(rc f18l-$e)" = 0 ] && [ -s f18l-$e/out/plate_1.gcode ] ||
        { show f18l-$e; fail "F18 $e: settings files exit $(rc f18l-$e)"; f18_ok=0; continue; }
    got=$(hdr f18l-$e/out/plate_1.gcode curr_bed_type)
    [ "$got" = "$want" ] || { fail "F18 $e: an STL with settings files slices on '$got', the printer preset is '$want'"; f18_ok=0; }
    if [ $e = bambu ]; then
        run f18m-$e "$bin" cube.stl -o f18m-$e.gcode --machine f18-$e-machine.json \
            --filament f18-$e-filament.json --process f18-$e-process.json
        got=$(hdr f18m-$e.gcode curr_bed_type)
        [ "$(rc f18m-$e)" = 0 ] && [ "$got" = "$want" ] ||
            { show f18m-$e; fail "F18 bambu: --machine/--process/--filament slice on '$got', the printer preset is '$want' (exit $(rc f18m-$e))"; f18_ok=0; }
    fi
done
[ $f18_ok = 1 ] && echo "PASS: F18 an STL with settings files takes the plate type of the printer preset those settings belong to (both engines)"

# F19: --nozzle changes nozzle_diameter alone, which no desktop flow does: the
# desktop changes a printer's nozzle by choosing the printer variant, and the
# line widths, the process and the start G-code come from that variant's
# preset (Sidebar::update_all_preset_comboboxes). A nozzle the printer preset
# is not built for is refused, naming the preset to choose instead; the
# nozzle it already has is what it already is.
f19_ok=1
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run f19no-$e "$bin" cube.stl --slice 1 --printer-preset "$X1C" --outputdir f19no-$e/out
    [ "$(rc f19no-$e)" = 0 ] || { show f19no-$e; fail "F19 $e: the plain run exit $(rc f19no-$e)"; f19_ok=0; continue; }
    run f19bad-$e "$bin" cube.stl --slice 1 --printer-preset "$X1C" --nozzle 0.6 --outputdir f19bad-$e/out
    [ "$(rc f19bad-$e)" != 0 ] || { fail "F19 $e: --nozzle 0.6 on a 0.4 printer sliced with 0.4 line widths"; f19_ok=0; continue; }
    py '
import json, sys
d = json.load(open(sys.argv[1]))
assert d["return_code"] == -2, d
s = d["error_string"]
assert "--printer-preset" in s and "0.6" in s, s
' f19bad-$e/out/result.json || { show f19bad-$e; fail "F19 $e: the --nozzle refusal does not name --printer-preset and the nozzle asked for"; f19_ok=0; }
    # The nozzle the printer already has: nothing changes, and the run slices
    # exactly as it does without the flag.
    run f19ok-$e "$bin" cube.stl --slice 1 --printer-preset "$X1C" --nozzle 0.4 --outputdir f19ok-$e/out
    [ "$(rc f19ok-$e)" = 0 ] || { show f19ok-$e; fail "F19 $e: --nozzle 0.4 on a 0.4 printer exit $(rc f19ok-$e)"; f19_ok=0; continue; }
    for k in nozzle_diameter line_width curr_bed_type; do
        a=$(hdr f19ok-$e/out/plate_1.gcode $k); b=$(hdr f19no-$e/out/plate_1.gcode $k)
        [ "$a" = "$b" ] || { fail "F19 $e: --nozzle 0.4 changed $k to '$a', without it '$b'"; f19_ok=0; }
    done
done
[ $f19_ok = 1 ] && echo "PASS: F19 --nozzle refuses a nozzle that is not the printer preset's, and is a no-op for the one it has (both engines)"

# F20: --bed-temp sets the bed temperature the print runs at. The bed heat
# commands do not read bed_temperature: they read the active plate type's own
# key (GCode::get_bed_temperature -> get_bed_temp_key, GCode.cpp 3866-3876 at
# 5873b5f; Print.cpp 1245-1260 at 31f6803), and the start G-code's
# {bed_temperature} and {bed_temperature_initial_layer} placeholders are
# overwritten with the same two keys (GCode.cpp 2789-2793 at 5873b5f; the same
# pair at 31f6803). On 2a12432 the flag left M140/M190 at the plate's old
# value.
f20_ok=1
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run f20-$e "$bin" cube.stl --slice 1 --printer-preset "$A1M" --bed-temp 70 --outputdir f20-$e/out
    [ "$(rc f20-$e)" = 0 ] && [ -s f20-$e/out/plate_1.gcode ] ||
        { show f20-$e; fail "F20 $e: --bed-temp 70 exit $(rc f20-$e)"; f20_ok=0; continue; }
    py '
import re, sys
text = open(sys.argv[1], errors="replace").read()
vals = [int(m.group(1)) for m in re.finditer(r"^M1[49]0 S(\d+)", text, re.M)]
assert vals, "no M140/M190 in the G-code"
assert 70 in vals, vals
bad = sorted({v for v in vals if v not in (0, 70)})
assert not bad, "the bed is still heated to %s" % bad
' f20-$e/out/plate_1.gcode || { show f20-$e; fail "F20 $e: --bed-temp 70 does not reach the bed heat commands"; f20_ok=0; }
done
[ $f20_ok = 1 ] && echo "PASS: F20 --bed-temp sets the bed temperature the slice runs at, in the G-code (both engines)"

# F21: --temp on a printer with several extruders sets every filament slot the
# run uses. The Snapmaker U1 has four, and on 2a12432 the flag left
# nozzle_temperature at 230,0,0,0: three slots printed at no temperature at
# all.
f21_ok=1
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    # Four extruders on the U1; two on the H2D, whose two filaments are both
    # used so the check covers both slots as well.
    printer="Snapmaker U1 (0.4 nozzle)"
    fargs=()
    if [ $e = bambu ]; then
        printer="Bambu Lab H2D 0.4 nozzle"
        fargs=(--filament-preset "Bambu PLA Basic @BBL H2D" --filament-preset "Bambu PLA Matte @BBL H2D")
    fi
    run f21-$e "$bin" cube.stl --slice 1 --printer-preset "$printer" "${fargs[@]}" --temp 230 --outputdir f21-$e/out
    [ "$(rc f21-$e)" = 0 ] && [ -s f21-$e/out/plate_1.gcode ] ||
        { show f21-$e; fail "F21 $e: --temp 230 on $printer exit $(rc f21-$e)"; f21_ok=0; continue; }
    py '
import re, sys
m = re.search(r"^; nozzle_temperature = (.*)$", open(sys.argv[1], errors="replace").read(), re.M)
assert m, "no nozzle_temperature in the G-code header"
vals = [v.strip() for v in m.group(1).split(",")]
assert len(vals) >= 2, vals
assert all(v == "230" for v in vals), vals
' f21-$e/out/plate_1.gcode || { show f21-$e; fail "F21 $e: --temp leaves slots at another temperature"; f21_ok=0; }
done
[ $f21_ok = 1 ] && echo "PASS: F21 --temp sets every filament slot of a printer with several extruders (both engines)"

# F22: --uptodate updates a Bambu Studio project to this engine's own system
# presets, from machine_full/, process_full/ and filament_full/ (BambuStudio.cpp
# 2638-2786 at 5873b5f; OrcaSlicer.cpp 2258-2420 at 31f6803). This package
# ships none of those folders and no other input has anything to update, so
# both cases say so instead of exiting 0 with nothing done.
f22_ok=1
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run f22a-$e "$bin" base.3mf --slice 1 --uptodate --outputdir f22a-$e/out
    run f22b-$e "$bin" cube.stl --slice 1 --uptodate --outputdir f22b-$e/out
    for n in a b; do
        [ "$(rc f22$n-$e)" != 0 ] || { fail "F22 $e: --uptodate succeeded ($n) with nothing updated"; f22_ok=0; continue; }
        py '
import json, sys
d = json.load(open(sys.argv[1]))
assert d["return_code"] == -2, d
s = d["error_string"]
assert "uptodate" in s and "--load-settings" in s, s
' f22$n-$e/out/result.json || { show f22$n-$e; fail "F22 $e: the --uptodate refusal ($n) does not name the flags that work"; f22_ok=0; }
    done
done
[ $f22_ok = 1 ] && echo "PASS: F22 --uptodate says what to give instead of exiting 0 with nothing updated (both engines)"

# BR3: a machine settings file that states no layer_change_gcode, and has no
# parent that defines one, makes the merge write that key empty
# (load_default_gcodes_to_config, BambuStudio.cpp 685-745 at 5873b5f;
# OrcaSlicer.cpp 548-604 at 31f6803), and the engine then refuses the print for
# relative extruder addressing (Print.cpp 1676-1690 at 31f6803; 1636-1652 at
# 5873b5f: the check takes a Marlin flavour, relative E, and neither layer
# G-code resetting the extruder). The refusal names that cause, not only the
# symptom.
#
# Two files. The one the package ships, the Snapmaker U1 0.6 machine preset,
# states no layer_change_gcode of its own — it is its parents' — and a settings
# file is read over them (`config = *default_config; config.apply(config_src)`,
# PresetBundle.cpp 5087 at 5873b5f / OrcaSlicer PresetBundle.cpp 4894 at
# 31f6803, on the preset the file names). So it slices (rc 0), the first check
# below. The refusal needs a file with no parent left to restore the key: a
# machine preset flattened over its parents with the key taken out and no
# "inherits". It is a Marlin one: the U1 is a Klipper tool changer
# (fdm_klipper), and the check above skips a Klipper flavour — measured on this
# build, the flattened U1 slices with both layer G-codes taken out. The Prusa
# MK4 0.4 nozzle is Marlin and its own before_layer_change_gcode holds the bare
# "G92 E0.0" line the engine's regex reads as a reset (Print.cpp 1259), so both
# keys are the ones taken out.
br3_ok=1
u106="$RES/profiles-orca/Snapmaker/machine/Snapmaker U1 (0.6 nozzle).json"
[ -f "$u106" ] || { fail "BR3: the package ships no U1 0.6 machine preset at $u106"; br3_ok=0; }
if [ $br3_ok = 1 ]; then
    flat_presets orca br3mk4 "Prusa MK4 0.4 nozzle" "$RES/profiles-orca/Prusa"
    py '
import json
p = "br3mk4-orca-machine.json"
d = json.load(open(p))
assert "inherits" not in d, "the flattened preset still inherits"
for key in ("layer_change_gcode", "before_layer_change_gcode"):
    assert key in d, "the parents define no " + key
    d.pop(key)
json.dump(d, open(p, "w"), indent=1)
' || { fail "BR3: the MK4 preset flattened over its parents is not one file"; br3_ok=0; }
fi
for e in orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run br3s-$e "$bin" base.3mf --slice 1 --load-settings "$u106" --outputdir br3s-$e/out
    [ "$(rc br3s-$e)" = 0 ] || { show br3s-$e; fail "BR3 $e: the shipped U1 0.6 machine preset no longer slices (exit $(rc br3s-$e))"; br3_ok=0; }
    run br3-$e "$bin" base.3mf --slice 1 --load-settings br3mk4-orca-machine.json --outputdir br3-$e/out
    [ "$(rc br3-$e)" != 0 ] || { fail "BR3 $e: a machine file with no layer_change_gcode sliced"; br3_ok=0; continue; }
    py '
import json, sys
d = json.load(open(sys.argv[1]))
s = d["error_string"]
assert d["return_code"] == -51, d
assert "Relative extruder addressing" in s, s
assert "layer_change_gcode" in s, s
' br3-$e/out/result.json || { show br3-$e; fail "BR3 $e: the refusal does not name the empty layer_change_gcode"; br3_ok=0; }
done
[ $br3_ok = 1 ] && echo "PASS: BR3 the relative-extruder refusal names the empty layer_change_gcode (Orca build)"

# BR7: --estimate-mode takes each filament from filament_full (BambuStudio.cpp
# 2408-2444), and this package ships no such folder. The refusal names the
# flags the user gave (--estimate-mode, --load-settings), never
# --load-defaultfila, which was not given.
br7_ok=1
flat_presets bambu e7 "$X1C"
run br7 "$B" base.3mf --slice 1 --estimate-mode --load-settings "e7-bambu-machine.json;e7-bambu-process.json" --outputdir br7/out
[ "$(rc br7)" != 0 ] || { show br7; fail "BR7: --estimate-mode without filament_full succeeded"; br7_ok=0; }
py '
import json, sys
d = json.load(open(sys.argv[1]))
s = d["error_string"]
assert d["return_code"] == -5, d
assert "--estimate-mode" in s and "--load-filaments" in s, s
assert "--load-defaultfila" not in s, s
' br7/out/result.json || { show br7; fail "BR7: the --estimate-mode refusal names a flag the user never gave"; br7_ok=0; }
[ $br7_ok = 1 ] && echo "PASS: BR7 the --estimate-mode refusal names the flags actually given (Bambu build)"

# BR8: --load-filament-ids counts the filaments the run has, which on this
# command line are the --filament-preset presets as well as the
# --load-filaments files (the official CLI counts m_load_filaments alone,
# BambuStudio.cpp 2078-2130 at 5873b5f; it has no preset path). On 2a12432 two
# presets with "1,2" were refused as "past the 0 filament(s) loaded".
br8_ok=1
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    # Two presets of one temperature series: the pair the audit used (PLA Basic
    # + PETG HF) trips the engine's own "Selected nozzle temperatures are
    # incompatible" on the Orca build once the count check no longer stops the
    # run first, which is a different refusal and not this row.
    run br8-$e "$bin" cube.stl cube.stl --slice 1 --printer-preset "$X1C" \
        --filament-preset "Bambu PLA Basic @BBL X1C" --filament-preset "Bambu PLA Matte @BBL X1C" \
        --load-filament-ids 1,2 --outputdir br8-$e/out
    [ "$(rc br8-$e)" = 0 ] || { show br8-$e; fail "BR8 $e: two presets with --load-filament-ids 1,2 exit $(rc br8-$e)"; br8_ok=0; continue; }
    py '
import re, sys
text = open(sys.argv[1], errors="replace").read()
m = re.search(r"^; filament_settings_id = (.*)$", text, re.M)
assert m, "no filament_settings_id in the G-code header"
for name in ("Bambu PLA Basic @BBL X1C", "Bambu PLA Matte @BBL X1C"):
    assert name in m.group(1), (name, m.group(1))
' br8-$e/out/plate_1.gcode || { show br8-$e; fail "BR8 $e: the two objects did not take the two preset filaments"; br8_ok=0; }
done
[ $br8_ok = 1 ] && echo "PASS: BR8 --load-filament-ids counts the --filament-preset filaments (both engines)"

# BR9: a printer preset this engine has not got, which the other engine of the
# package has, is refused naming that binary, as the 3MF engine-fit refusal
# does (engine_mismatch_sentence).
br9_ok=1
run br9 "$B" cube.stl --slice 1 --printer-preset "Snapmaker U1 (0.4 nozzle)" --outputdir br9/out
[ "$(rc br9)" != 0 ] || { show br9; fail "BR9: slicer_cli took a Snapmaker printer"; br9_ok=0; }
py '
import json, sys
d = json.load(open(sys.argv[1]))
s = d["error_string"]
assert "BambuStudio has no printer preset named" in s, s
assert "slicer_cli-orcaslicer" in s, s
' br9/out/result.json || { show br9; fail "BR9: the refusal of a Snapmaker printer does not name slicer_cli-orcaslicer"; br9_ok=0; }
[ $br9_ok = 1 ] && echo "PASS: BR9 a printer preset of the other engine is refused naming that binary (Bambu build)"

# BR10: the plate/filament refusal (-61) names the plates the filament does
# support, not only the one it does not: the engine's own sentence stops at
# "Plate 1: Cool Plate does not support filament 2" (Print.cpp 1700-1726 at
# 31f6803; 1650-1672 at 5873b5f), from a check that reads the plate's own bed
# temperature key and reads 0 as "not this filament's plate". Two filaments
# that share no other plate, on the plate given on the command line: Bambu
# PETG HF @BBL X1C has cool_plate_temp 0, so the run is refused -61 for it.
# --allow-mix-temp=1 gets past the temperature check to this one
# (BambuStudio.cpp 6979-6982; OrcaSlicer.cpp 5968-5971). The plates the
# sentence names must be the ones it can actually print on: the run it
# suggests slices. Two filaments on one plate want a prime tower, and this
# pair's tower is too big for the plate: the desktop's own clamp puts the
# tower's corner at (165, 227.972) on the X1 Carbon (the fixed-layout tower
# check above), and the tower's own G-code still reaches y 258.884 — past the
# 2 mm the engine's upload check tolerates (GCodeProcessor.cpp 1861-1883: the
# printable box offset by 2 mm, extrude moves only), so the run is refused
# -104 on the Orca build. The official CLI is deliberately more conservative
# than the desktop here, its own sentence says so, so this run gives the
# tower a position.
br10_ok=1
py '
import json
json.dump({"plates": [{"plate_name": "br10", "need_arrange": False,
                       "objects": [{"path": "cube.stl", "count": 1, "filaments": [1], "pos_x": [100], "pos_y": [150]},
                                   {"path": "cube.stl", "count": 1, "filaments": [2], "pos_x": [140], "pos_y": [150]}]}]},
          open("br10.json", "w"))
'
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run br10-$e "$bin" --load-assemble-list br10.json --slice 1 --printer-preset "$X1C" \
        --filament-preset "Bambu PLA Basic @BBL X1C" --filament-preset "Bambu PETG HF @BBL X1C" \
        --allow-mix-temp=1 --curr-bed-type "Cool Plate" --wipe-tower-x 30 --wipe-tower-y 220 --outputdir br10-$e/out
    [ "$(rc br10-$e)" != 0 ] || { show br10-$e; fail "BR10 $e: two filaments that share no Cool Plate sliced"; br10_ok=0; continue; }
    suggested=$(py '
import json, re, sys
d = json.load(open(sys.argv[1]))
s = d["error_string"]
assert d["return_code"] == -61, d
assert "does not support filament" in s, s
m = re.search(r"Pick a plate this filament supports: --curr-bed-type \"([^\"]+)\"", s)
assert m, s
print(m.group(1))
' br10-$e/out/result.json) || { show br10-$e; fail "BR10 $e: the plate/filament refusal names no plate to give instead"; br10_ok=0; continue; }
    run br10r-$e "$bin" --load-assemble-list br10.json --slice 1 --printer-preset "$X1C" \
        --filament-preset "Bambu PLA Basic @BBL X1C" --filament-preset "Bambu PETG HF @BBL X1C" \
        --allow-mix-temp=1 --curr-bed-type "$suggested" --wipe-tower-x 30 --wipe-tower-y 220 --outputdir br10r-$e/out
    [ "$(rc br10r-$e)" = 0 ] && [ -s br10r-$e/out/plate_1.gcode ] ||
        { show br10r-$e; fail "BR10 $e: the plate the refusal names ($suggested) exit $(rc br10r-$e)"; br10_ok=0; }
done
[ $br10_ok = 1 ] && echo "PASS: BR10 the plate/filament refusal names a plate the run slices on (both engines)"

# BR11: the two bed refusals say what to do, not only what is wrong. An object
# crossing the bed edge is refused -52 with "Object 'X' (…) crosses the edge of
# the 180 x 180 x 180 mm bed." (the official's partly-inside gate, BambuStudio.cpp
# 6527-6567 at 5873b5f; OrcaSlicer.cpp 5645-5697 at 31f6803), and a plate whose
# objects cannot share it is refused -21 with "These objects do not fit on the
# 180 x 180 x 180 mm bed together: 'X'." (the arrange's own verdict,
# BambuStudio.cpp 5936-5945 / OrcaSlicer.cpp 5196-5205). Each now ends with the
# way out. The plate: a 145 mm box clear of the X1 Carbon's exclusion area and
# a 40 mm box near the A1 mini's 180 mm edge, both inside the X1 Carbon's bed,
# exported from that printer as a project — so the plate fits one printer and
# the other. --arrange 1 cannot place both on the A1 mini (145 + 40 > 180 in
# both directions), which is the -21 half. The third run is the same gate on a
# plate the run has already re-arranged: --arrange 1 on the assemble list is
# accepted, the arrange runs, and an object still crosses — the sentence there
# must not name --arrange 1 again.
br11_ok=1
py '
def box(path, sx, sy, h):
    v = [(x, y, z) for z in (0, h) for y in (0, sy) for x in (0, sx)]
    tri = [(0,2,1),(1,2,3),(4,5,6),(5,7,6),(0,1,4),(1,5,4),(2,6,3),(3,6,7),(0,4,2),(2,4,6),(1,3,5),(3,7,5)]
    with open(path, "w") as o:
        o.write("solid t\n")
        for a, b, c in tri:
            o.write("facet normal 0 0 0\nouter loop\n")
            for i in (a, b, c): o.write("vertex %g %g %g\n" % v[i])
            o.write("endloop\nendfacet\n")
        o.write("endsolid t\n")
box("br11big.stl", 145, 145, 10)
box("br11edge.stl", 40, 40, 10)
import json
json.dump({"plates": [{"plate_name": "bed", "need_arrange": False, "objects": [
    {"path": "br11big.stl", "count": 1, "filaments": [1], "pos_x": [20], "pos_y": [32]},
    {"path": "br11edge.stl", "count": 1, "filaments": [1], "pos_x": [170], "pos_y": [90]}]}]},
    open("br11.json", "w"))
'
for e in bambu orca; do
    bin=$B; [ $e = orca ] && bin=$O
    run br11p-$e "$bin" --load-assemble-list br11.json --slice 1 --printer-preset "$X1C" \
        --outputdir br11p-$e/out --export-3mf br11proj.3mf
    [ "$(rc br11p-$e)" = 0 ] && [ -s br11p-$e/out/br11proj.3mf ] ||
        { show br11p-$e; fail "BR11 $e: the two-object X1 Carbon project did not export (exit $(rc br11p-$e))"; br11_ok=0; continue; }
    run br11a-$e "$bin" br11p-$e/out/br11proj.3mf --slice 1 --printer-preset "$A1M" --outputdir br11a-$e/out
    [ "$(rc br11a-$e)" != 0 ] || { fail "BR11 $e: an object crossing the bed edge sliced"; br11_ok=0; continue; }
    py '
import json, sys
d = json.load(open(sys.argv[1]))
s = d["error_string"]
assert d["return_code"] == -52, d
assert "crosses the edge of the 180 x 180 x 180 mm bed." in s, s
assert s.endswith(" Give --arrange 1 to re-arrange the plate on this bed, or use a printer with a bigger bed."), s
' br11a-$e/out/result.json || { show br11a-$e; fail "BR11 $e: the bed-edge refusal does not say what to do"; br11_ok=0; }
    run br11b-$e "$bin" br11p-$e/out/br11proj.3mf --slice 1 --printer-preset "$A1M" --arrange 1 --outputdir br11b-$e/out
    [ "$(rc br11b-$e)" != 0 ] || { fail "BR11 $e: objects that do not fit the A1 mini together sliced"; br11_ok=0; continue; }
    py '
import json, sys
d = json.load(open(sys.argv[1]))
s = d["error_string"]
assert d["return_code"] == -21, d
assert "do not fit on the 180 x 180 x 180 mm bed together" in s, s
assert s.endswith(" Move some objects to another plate in the file, or use a printer with a bigger bed."), s
' br11b-$e/out/result.json || { show br11b-$e; fail "BR11 $e: the arrange refusal does not say what to do"; br11_ok=0; }
    # The same gate after the run has already re-arranged the plate: --arrange 1
    # on the assemble list is accepted by the Bambu build, the arrange runs,
    # and an object can still cross this bed — so the refusal must not send the
    # run back to a flag it was already given. (The Orca CLI refuses --arrange
    # with --load-assemble-list: -2, "give no model files and no transforms with
    # it", so this run is the Bambu build's.)
    if [ $e = bambu ]; then
        run br11c-$e "$bin" --load-assemble-list br11.json --slice 1 --printer-preset "$A1M" --arrange 1 --outputdir br11c-$e/out
        [ "$(rc br11c-$e)" != 0 ] || { fail "BR11 $e: --arrange 1 on the assemble list sliced"; br11_ok=0; continue; }
        py '
import json, sys
d = json.load(open(sys.argv[1]))
s = d["error_string"]
assert d["return_code"] == -52, d
assert "crosses the edge of the 180 x 180 x 180 mm bed." in s, s
assert s.endswith(" Use a printer with a bigger bed."), s
' br11c-$e/out/result.json || { show br11c-$e; fail "BR11 $e: the refusal after --arrange 1 names the flag again"; br11_ok=0; }
    fi
done
[ $br11_ok = 1 ] && echo "PASS: BR11 the bed refusals say what to do (both engines; the re-arranged plate on the Bambu build)"

# F18b: the same defect on the real Snapmaker U1 files, where it decides
# whether the run works at all: on 2a12432 an STL with the shipped U1 machine,
# process and filament files sliced on the engine's own default Cool Plate
# (M140/M190 S35), which "Snapmaker PLA @U1" cannot print on — the run is
# refused -103 "Found some filament unprintable at first layer on current
# Plate". The U1's own plate (the one the desktop app picks for the printer)
# slices it. Both routes are checked: the product's --machine/--process/
# --filament, and --printer-preset/--filament-preset (which already takes it).
f18b_ok=1
u1m="$RES/profiles-orca/Snapmaker/machine/Snapmaker U1 (0.4 nozzle).json"
u1p="$RES/profiles-orca/Snapmaker/process/0.20 Standard @Snapmaker U1 (0.4 nozzle).json"
u1f="$RES/profiles-orca/Snapmaker/filament/Snapmaker PLA @U1.json"
for f in "$u1m" "$u1p" "$u1f"; do
    [ -f "$f" ] || { fail "F18b: the package ships no $f"; f18b_ok=0; }
done
if [ $f18b_ok = 1 ]; then
    run f18bn "$O" cube.stl --slice 1 --printer-preset "Snapmaker U1 (0.4 nozzle)" \
        --process-preset "0.20 Standard @Snapmaker U1 (0.4 nozzle)" --filament-preset "Snapmaker PLA @U1" \
        --outputdir f18bn/out
    run f18bf "$O" cube.stl --slice 1 --machine "$u1m" --process "$u1p" --filament "$u1f" --outputdir f18bf/out
    for n in f18bn f18bf; do
        [ "$(rc $n)" = 0 ] && [ -s $n/out/plate_1.gcode ] ||
            { show $n; fail "F18b orca: $n exit $(rc $n)"; f18b_ok=0; }
    done
    want=$(hdr f18bn/out/plate_1.gcode curr_bed_type)
    got=$(hdr f18bf/out/plate_1.gcode curr_bed_type)
    [ -n "$want" ] && [ "$want" != "Cool Plate" ] ||
        { fail "F18b orca: the printer preset slices on '$want', the engine's own default"; f18b_ok=0; }
    [ "$got" = "$want" ] ||
        { fail "F18b orca: the shipped U1 files slice on '$got', the printer preset on '$want'"; f18b_ok=0; }
    py '
import re, sys
def beds(path):
    t = open(path, errors="replace").read()
    return sorted({int(m.group(1)) for m in re.finditer(r"^M1[49]0 S(\d+)", t, re.M) if int(m.group(1))})
a, b = beds(sys.argv[1]), beds(sys.argv[2])
assert a, "no M140/M190 in the printer preset run"
assert a == b, (a, b)
' f18bn/out/plate_1.gcode f18bf/out/plate_1.gcode ||
        { show f18bf; fail "F18b orca: the shipped U1 files do not heat the bed as the printer preset does"; f18b_ok=0; }
fi
[ $f18b_ok = 1 ] && echo "PASS: F18b the shipped U1 files slice on the U1's own plate with its bed temperature, through both routes (Orca build)"
# With E2E_CONTINUE=1 the run reaches here even after a FAIL (see fail()): say
# how many there were, so the exit status still tells the truth.
if [ "${E2E_CONTINUE:-0}" = "1" ] && [ "$FAILS" -gt 0 ]; then
    echo "E2E: $FAILS failed check(s)"
    exit 1
fi
