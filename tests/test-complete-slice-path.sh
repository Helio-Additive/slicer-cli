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

# --export-3mf: the sliced project carries the plate G-code.
run export "$B" "$FIXTURE" --slice 1 --outputdir export/out --export-3mf sliced.3mf
[ "$(rc export)" = 0 ] || { show export; fail "--export-3mf exit $(rc export)"; }
py '
import zipfile; n = zipfile.ZipFile("export/out/sliced.3mf").namelist()
assert "Metadata/plate_1.gcode" in n and "Metadata/slice_info.config" in n, n
'
echo "PASS: --export-3mf writes the sliced 3MF"

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
run orca-values "$O" "$FIXTURE" --slice 1 --allow-newer-file --outputdir orca-values/out
[ "$(rc orca-values)" != 0 ] || fail "Orca sliced a file with values it does not have"
grep -q 'tree_support_wall_count: -1 not in range' orca-values/stderr || { show orca-values; fail "no range refusal"; }
grep -q "'ensure_vertical_shell_thickness' is 'enabled'" orca-values/stderr || { show orca-values; fail "no unknown-value refusal"; }
echo "PASS: Orca refuses out-of-range and unknown values, naming each"
