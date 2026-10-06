#!/usr/bin/env bash
# test-pr35-findings.sh: the PR #35 findings F1-F10, each a check of what the
# official command line does (BambuStudio.cpp at 5873b5f, OrcaSlicer.cpp at
# 31f6803), on both engines. Every block FAILS on ba0dcfb for the reason in
# its comment and passes once fixed. Every block runs; the script fails at the
# end when any did. Run by tests/test-complete-slice-path.sh.
#
# Usage: tests/test-pr35-findings.sh SLICER_CLI SLICER_CLI_ORCASLICER [BLOCK...]
#   BLOCK: F1..F10, F7o (default: all).
# Env: KEEP=<dir> keeps the work folder there.
set -uo pipefail

B="${1:?usage: $0 slicer_cli slicer_cli-orcaslicer [BLOCK...]}"
O="${2:?usage: $0 slicer_cli slicer_cli-orcaslicer [BLOCK...]}"
shift 2
BLOCKS=("$@"); [ ${#BLOCKS[@]} = 0 ] && BLOCKS=(F1 F2 F3 F4 F5 F6 F7 F7o F8 F9 F10)
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
B="$(cd "$(dirname "$B")" && pwd -P)/$(basename "$B")"
O="$(cd "$(dirname "$O")" && pwd -P)/$(basename "$O")"
# The package's resources: beside the binary (the release packages) or one
# level up (a bin/ layout).
RES="$(cd "$(dirname "$B")" && pwd -P)/resources"
[ -d "$RES/profiles" ] || RES="$(cd "$(dirname "$B")/.." && pwd -P)/resources"
if [ -n "${KEEP:-}" ]; then WORKDIR="$KEEP"; mkdir -p "$WORKDIR"
else WORKDIR="$(mktemp -d)"; trap 'rm -rf "$WORKDIR"' EXIT; fi
cp "$HERE/fixtures/calib_base.3mf" "$WORKDIR/base.3mf"
cp "$HERE/tools/fhelp.py" "$HERE/tools/resolve_preset.py" "$WORKDIR/"
cd "$WORKDIR"

A1M="Bambu Lab A1 mini 0.4 nozzle"
A1M_Q="\"$A1M\""
X1C="Bambu Lab X1 Carbon 0.4 nozzle"
RESULTS=()
# report ID ENGINE STATUS TEXT
report() { local line="$3: $1 $2: $4"; echo "$line"; RESULTS+=("$line"); }
# run NAME BINARY ARGS... : stdout/stderr/rc kept under NAME/; never aborts.
run() {
    local name=$1; shift
    rm -rf "$name"; mkdir -p "$name"
    "$@" > "$name/stdout" 2> "$name/stderr" < /dev/null
    echo $? > "$name/rc"
}
rc() { cat "$1/rc" 2>/dev/null || echo none; }
why() { echo "exit $(rc "$1"); $(grep -h '^Error' "$1/stderr" "$1/stdout" 2>/dev/null | tail -n 1 | cut -c1-200)"; }
fh() { python3 fhelp.py "$@"; }
py() { python3 -c "$@"; }
events() { grep '^\[\[SLICER_EVENT\]\]' "$1" | sed 's/^\[\[SLICER_EVENT\]\] //' | grep "\"event\":\"$2\"" || true; }
bin_of() { [ "$1" = orca ] && echo "$O" || echo "$B"; }

# ---------------------------------------------------------------- fixtures
py '
def box(path, sx, sy, sz):
    v = [(x, y, z) for z in (0, sz) for y in (0, sy) for x in (0, sx)]
    f = [(0,2,1),(1,2,3),(4,5,6),(5,7,6),(0,1,4),(1,5,4),(2,6,3),(3,6,7),(0,4,2),(2,4,6),(1,3,5),(3,7,5)]
    with open(path, "w") as o:
        o.write("solid t\n")
        for a, b, c in f:
            o.write("facet normal 0 0 0\nouter loop\n")
            for i in (a, b, c): o.write("vertex %g %g %g\n" % v[i])
            o.write("endloop\nendfacet\n")
        o.write("endsolid t\n")
box("cube.stl", 20, 20, 20)
box("cube2.stl", 20, 20, 20)
box("extra.stl", 10, 10, 10)
box("slab.stl", 30, 20, 10)
box("tiny.stl", 1.5, 1.5, 1.5)
import math
def prism(path, profile, depth):
    n = len(profile)
    front = [(x, 0, z) for x, z in profile]; back = [(x, depth, z) for x, z in profile]
    tris = []
    for i in range(1, n - 1):
        tris.append((front[0], front[i + 1], front[i])); tris.append((back[0], back[i], back[i + 1]))
    for i in range(n):
        j = (i + 1) % n
        tris.append((front[i], front[j], back[j])); tris.append((front[i], back[j], back[i]))
    with open(path, "w") as o:
        o.write("solid p\n")
        for t in tris:
            o.write("facet normal 0 0 0\nouter loop\n")
            for v in t: o.write("vertex %g %g %g\n" % v)
            o.write("endloop\nendfacet\n")
        o.write("endsolid p\n")
# A block leaning 50 degrees: its sides overhang between the default 30 degree
# threshold and the 85 degree one of lean-process.json.
dx = 30 * math.tan(math.radians(50))
prism("lean.stl", [(0, 0), (20, 0), (20 + dx, 30), (dx, 30)], 20)
import json
P = lambda name, objs, **kw: dict({"plate_name": name, "need_arrange": False, "objects": objs}, **kw)
C = lambda path, n=1, fil=None, **kw: dict({"path": path, "count": n, "filaments": fil or [1] * n}, **kw)
json.dump({"plates": [P("p", [C("cube.stl", 2)], need_arrange=True)]}, open("skip2.json", "w"))
json.dump({"plates": [P("p", [C("cube.stl", pos_x=[118], pos_y=[118])]), P("q", [C("cube.stl", pos_x=[118], pos_y=[118])])]}, open("trail2.json", "w"))
json.dump({"plates": [P("p", [C("lean.stl", pos_x=[100], pos_y=[100])]), P("q", [C("lean.stl", pos_x=[100], pos_y=[100])])]}, open("lean2.json", "w"))
json.dump({"plates": [P(n, [C("cube.stl", pos_x=[128], pos_y=[128])]) for n in "abc"]}, open("three.json", "w"))
json.dump({"plates": [P("t", [C("cube.stl", 2, [1, 2], pos_x=[195, 30], pos_y=[220, 0])])]}, open("ftow.json", "w"))
json.dump({"plates": [P("pp", [C("cube.stl")], need_arrange=True,
                        plate_params={"layer_height": "0.28", "wall_loops": "5", "sparse_infill_density": "35%"})]},
          open("pparams.json", "w"))
'

# proj-<e>.3mf: a 20 mm cube on the A1 mini (the 8b8614a fixture).
fx_proj() {
    [ -s proj-$1.3mf ] && return 0
    run fxproj-$1 "$(bin_of $1)" cube.stl --slice 1 --printer-preset "$A1M" --outputdir fxproj-$1/out --export-3mf proj.3mf
    cp fxproj-$1/out/proj.3mf proj-$1.3mf 2>/dev/null
}
# slab-<e>.3mf: a 30 x 20 x 10 box, tiny-<e>.3mf: a 1.5 mm cube (looks like inches).
fx_named() {  # fx_named ENGINE NAME STL
    [ -s $2-$1.3mf ] && return 0
    run fx$2-$1 "$(bin_of $1)" $3 --slice 1 --printer-preset "$A1M" --outputdir fx$2-$1/out --export-3mf $2.3mf
    cp fx$2-$1/out/$2.3mf $2-$1.3mf 2>/dev/null
}
# skip2-<e>.3mf: one cube object with two instances on an X1 Carbon plate.
fx_skip2() {
    [ -s skip2-$1.3mf ] && return 0
    run fxskip2-$1 "$(bin_of $1)" --load-assemble-list skip2.json --slice 1 --printer-preset "$X1C" --outputdir fxskip2-$1/out --export-3mf skip2.3mf
    cp fxskip2-$1/out/skip2.3mf skip2-$1.3mf 2>/dev/null
}
# inst2-<e>.3mf: ONE cube object with two instances (skip2's second object made an
# instance of the first: one build item more for object 2, as the desktop
# writes an object's instances).
fx_inst2() {
    fx_skip2 $1
    [ -s inst2-$1.3mf ] && return 0
    [ -s skip2-$1.3mf ] || return 0
    py '
import re, sys, zipfile
src, dst = sys.argv[1], sys.argv[2]
with zipfile.ZipFile(src) as zin, zipfile.ZipFile(dst, "w", zipfile.ZIP_DEFLATED) as zout:
    for item in zin.infolist():
        data = zin.read(item.filename)
        if item.filename == "3D/3dmodel.model":
            t = data.decode()
            t = re.sub(r"\s*<object id=\"3\".*?</object>", "", t, count=1, flags=re.S)
            t = t.replace("<item objectid=\"3\"", "<item objectid=\"2\"")
            data = t.encode()
        elif item.filename == "Metadata/model_settings.config":
            t = data.decode()
            t = re.sub(r"  <object id=\"3\">.*?</object>\n", "", t, count=1, flags=re.S)
            t = re.sub(r"(<metadata key=\"object_id\" value=\")3(\"/>\s*<metadata key=\"instance_id\" value=\")0",
                       r"\g<1>2\g<2>1", t)
            data = t.encode()
        elif item.filename.startswith("Metadata/plate_") and item.filename.endswith((".gcode", ".md5")):
            continue
        zout.writestr(item, data)
' skip2-$1.3mf inst2-$1.3mf
}
# two-<e>.3mf: two cube objects side by side on the A1 mini.
fx_two() {
    [ -s two-$1.3mf ] && return 0
    run fxtwo-$1 "$(bin_of $1)" cube.stl cube2.stl --slice 1 --arrange 1 --printer-preset "$A1M" --outputdir fxtwo-$1/out --export-3mf two.3mf
    cp fxtwo-$1/out/two.3mf two-$1.3mf 2>/dev/null
}
fx_list() {  # fx_list ENGINE NAME LIST [extra args]: an X1 Carbon project from an assemble list, every plate sliced
    local e=$1 n=$2 l=$3; shift 3
    [ -s $n-$e.3mf ] && return 0
    run fx$n-$e "$(bin_of $e)" --load-assemble-list $l --slice 0 --printer-preset "$X1C" "$@" --outputdir fx$n-$e/out --export-3mf $n.3mf
    cp fx$n-$e/out/$n.3mf $n-$e.3mf 2>/dev/null
}
fx_ftow() {
    [ -s ftow-$1.3mf ] && return 0
    run fxftow-$1 "$(bin_of $1)" --load-assemble-list ftow.json --slice 1 --printer-preset "$X1C" \
        --filament-preset "Bambu PLA Basic @BBL X1C" --filament-preset "Bambu PLA Matte @BBL X1C" \
        --enable-prime-tower --wipe-tower-x 20 --wipe-tower-y 20 --outputdir fxftow-$1/out --export-3mf ftow.3mf
    cp fxftow-$1/out/ftow.3mf ftow-$1.3mf 2>/dev/null
}
# Settings files: the engine's own system presets for PRINTER, flattened (inherits walked), as
# the official CLIs read them: <tag>-<e>-machine.json, -process.json, -filament.json.
fx_settings() {  # fx_settings ENGINE TAG PRINTER
    local e=$1 t=$2 p=$3 v="$RES/profiles/BBL"
    [ $e = orca ] && v="$RES/profiles-orca/BBL"
    [ -s $t-$e-machine.json ] && return 0
    # A copy in the work folder: python reads it by a relative path (Windows
    # python cannot read an MSYS absolute path).
    [ -d vendor-$e ] || cp -R "$v" vendor-$e
    python3 resolve_preset.py vendor-$e "$p" $t-$e > /dev/null
}

want() { for b in "${BLOCKS[@]}"; do [ "$b" = "$1" ] && return 0; done; return 1; }

# ---------------------------------------------------------------- F1
# --export-3mf keeps --skip-objects: the skipped instance is written not
# printable, as the official marks it on the model it exports (BambuStudio.cpp
# 6545-6551 then export_project 7537/8156; OrcaSlicer.cpp 5663-5668, 6331).
# ba0dcfb reloads the file for the export, so the instance is printable again
# (main.cpp 8296-8316) and a re-slice of the export prints it.
F1() {
    for e in bambu orca; do
        fx_skip2 $e; bin=$(bin_of $e)
        [ -s skip2-$e.3mf ] || { report F1 $e FAIL "fixture skip2 project: $(why fxskip2-$e)"; continue; }
        id1=$(fh items skip2-$e.3mf | py 'import json,sys; print(json.load(sys.stdin)[0]["identify_id"])')
        run f1-$e "$bin" skip2-$e.3mf --slice 1 --skip-objects "$id1" --outputdir f1-$e/out --export-3mf x.3mf
        [ "$(rc f1-$e)" = 0 ] || { report F1 $e FAIL "the skip run: $(why f1-$e)"; continue; }
        labels=$(fh labels f1-$e/out/plate_1.gcode)
        printable=$(fh items f1-$e/out/x.3mf | py '
import json, sys; print(json.dumps([i["printable"] for i in json.load(sys.stdin) if i["identify_id"] == sys.argv[1]]))' "$id1")
        run f1r-$e "$bin" f1-$e/out/x.3mf --plate 1 -o f1r-$e.gcode
        relabels=$(fh labels f1r-$e.gcode 2>/dev/null || echo '?')
        if grep -qw "$id1" <<<"$labels"; then report F1 $e FAIL "the G-code printed the skipped id $id1 ($labels)"
        elif [ "$printable" = "[false]" ] && [ "$relabels" = "$labels" ]; then
            report F1 $e PASS "the exported 3MF marks id $id1 not printable; its re-slice prints $relabels as the run did"
        else
            report F1 $e FAIL "the G-code skipped id $id1 ($labels) but the exported 3MF has it printable=$printable; re-slice prints $relabels"
        fi
    done
}

# ---------------------------------------------------------------- F2
# --export-stls sees the scene, also with --slice N: the official
# loads plate N's objects at their scene place (BambuStudio.cpp 1889, 6506;
# its actions 6390-6398 read that model). ba0dcfb shifts plate N's objects to
# the plate's local frame before the actions. (--info is not checked: the
# engine's print_info min_x is the object's own mesh box, the same either way.)
F2() {
    for e in bambu orca; do
        fx_list $e trail2 trail2.json; bin=$(bin_of $e)
        [ -s trail2-$e.3mf ] || { report F2 $e FAIL "fixture two-plate project: $(why fxtrail2-$e)"; continue; }
        run f2a-$e "$bin" trail2-$e.3mf --slice 2 --export-stls f2a-$e/stls --outputdir f2a-$e/out
        run f2b-$e "$bin" trail2-$e.3mf --slice 0 --export-stls f2b-$e/stls --outputdir f2b-$e/out
        [ "$(rc f2a-$e)" = 0 ] && [ "$(rc f2b-$e)" = 0 ] || { report F2 $e FAIL "runs: --slice 2 $(why f2a-$e); --slice 0 $(why f2b-$e)"; continue; }
        verdict=$(py '
import json, subprocess, sys
e = sys.argv[1]
fh = lambda *a: json.loads(subprocess.check_output(["python3", "fhelp.py"] + list(a)))
a = fh("stl_boxes", "f2a-%s/stls" % e); b = fh("stl_boxes", "f2b-%s/stls" % e)
# Plate 2 sits right of plate 1 in the scene (grid column 2).
b2 = [x for x in b if x[0] > 256]
ok = len(a) == 1 and len(b2) == 1 and a == b2
print(("PASS" if ok else "FAIL") + " --export-stls box with --slice 2: %s; with --slice 0, plate 2: %s" % (a, b2))
' $e)
        report F2 $e "${verdict%% *}" "${verdict#* }"
    done
}

# ---------------------------------------------------------------- F3
# --slice 0 --arrange 1 arranges all plates together and recycles the empty
# plates (global arrange, BambuStudio.cpp 5628-5728, rebuild_plates_after_
# arrangement 6003 -> PartPlate.cpp 6850-6867; OrcaSlicer.cpp 4889-4990): two
# 20 mm cubes, one per plate, end on one plate. ba0dcfb arranges each plate
# alone and keeps both plates.
F3() {
    for e in bambu orca; do
        fx_list $e trail2 trail2.json; bin=$(bin_of $e)
        [ -s trail2-$e.3mf ] || { report F3 $e FAIL "fixture two-plate project: $(why fxtrail2-$e)"; continue; }
        run f3-$e "$bin" trail2-$e.3mf --slice 0 --arrange 1 --outputdir f3-$e/out
        [ "$(rc f3-$e)" = 0 ] || { report F3 $e FAIL "run: $(why f3-$e)"; continue; }
        verdict=$(py '
import json, sys
d = json.load(open(sys.argv[1]))
plates = [(p["id"], len(p["objects"])) for p in d["sliced_plates"]]
print(("PASS" if plates == [(1, 2)] else "FAIL") + " sliced plates (id, objects): %s; want [(1, 2)]" % plates)
' f3-$e/out/result.json)
        report F3 $e "${verdict%% *}" "${verdict#* }"
    done
}

# ---------------------------------------------------------------- F4
# Model actions without --slice see the arranged model: the official arranges
# model files before its action loop (need_arrange for model files,
# BambuStudio.cpp 2077, arrange 5558, actions 6336; OrcaSlicer.cpp 1724, 4817,
# 5470). Two cubes given as files must not overlap in --export-stls.
F4() {
    for e in bambu orca; do
        bin=$(bin_of $e)
        run f4-$e "$bin" cube.stl cube2.stl --printer-preset "$A1M" --export-stls f4-$e/stls --outputdir f4-$e/out
        [ "$(rc f4-$e)" = 0 ] || { report F4 $e FAIL "run: $(why f4-$e)"; continue; }
        verdict=$(py '
import json, subprocess, sys
b = json.loads(subprocess.check_output(["python3", "fhelp.py", "stl_boxes", sys.argv[1]]))
over = len(b) == 2 and b[0][0] < b[1][1] and b[1][0] < b[0][1] and b[0][2] < b[1][3] and b[1][2] < b[0][3]
print(("FAIL" if over or len(b) != 2 else "PASS") + " STL boxes %s%s" % (b, " overlap" if over else ""))
' f4-$e/stls)
        report F4 $e "${verdict%% *}" "${verdict#* }"
    done
}

# ---------------------------------------------------------------- F5
# --export-settings writes the settings after the arrange, so it holds the
# tower the arrange set (BambuStudio.cpp 5856 then the action 6366-6370;
# OrcaSlicer.cpp 5116 then 5499-5503). ba0dcfb snapshots the settings before
# the arrange. (a) --repetitions on a printer change, (b) --arrange 1.
F5() {
    for e in bambu orca; do
        fx_ftow $e; bin=$(bin_of $e)
        [ -s a1m-all-$e.json ] || run f5set-$e "$bin" cube.stl --printer-preset "$A1M" --export-settings a1m-all-$e.json
        [ -s ftow-$e.3mf ] || { report F5 $e FAIL "fixture tower project: $(why fxftow-$e)"; continue; }
        py '
import json, sys
d = json.load(open("a1m-all-%s.json" % sys.argv[1]))
d.update({"type": "machine", "from": "system", "name": "Bambu Lab A1 mini 0.4 nozzle", "instantiation": "true"}); d.pop("inherits", None)
json.dump(d, open("a1m-%s.json" % sys.argv[1], "w"), indent=1)
' $e
        bad=""
        for c in "rep:--load-settings a1m-$e.json --repetitions 2" "arr:--arrange 1"; do
            n=${c%%:*}; flags=${c#*:}
            run f5$n-$e "$bin" ftow-$e.3mf --slice 1 $flags --export-settings f5$n-$e.json --outputdir f5$n-$e/out
            [ "$(rc f5$n-$e)" = 0 ] || { bad="$bad $n: $(why f5$n-$e);"; continue; }
            r=$(py '
import json, re, sys
g = open(sys.argv[1], errors="replace").read()
gx = float(re.search(r"^; wipe_tower_x = (.*)$", g, re.M).group(1).split(",")[0])
gy = float(re.search(r"^; wipe_tower_y = (.*)$", g, re.M).group(1).split(",")[0])
s = json.load(open(sys.argv[2]))
ex, ey = float(s["wipe_tower_x"][0]), float(s["wipe_tower_y"][0])
print("ok" if abs(gx - ex) < 0.01 and abs(gy - ey) < 0.01 else "sliced tower (%g, %g), --export-settings (%g, %g)" % (gx, gy, ex, ey))
' f5$n-$e/out/plate_1.gcode f5$n-$e.json)
            [ "$r" = ok ] || bad="$bad $n: $r;"
        done
        if [ -z "$bad" ]; then report F5 $e PASS "--export-settings holds the tower the slice used (repetitions and --arrange 1)"
        else report F5 $e FAIL "$bad"; fi
    done
}

# ---------------------------------------------------------------- F6
# A plate's print sequence is its own print_sequence or the project's
# (get_print_sequence, BambuStudio.cpp 4403-4416). plate_N.json is a slice
# output; ba0dcfb reads its is_seq_print and forces "by object" (main.cpp
# 5893-5937). The default product call is the one checked.
F6() {
    for e in bambu orca; do
        fx_proj $e; bin=$(bin_of $e)
        [ -s proj-$e.3mf ] || { report F6 $e FAIL "fixture project: $(why fxproj-$e)"; continue; }
        py '
import json, zipfile, sys
src, dst = sys.argv[1], sys.argv[2]
with zipfile.ZipFile(src) as zin, zipfile.ZipFile(dst, "w", zipfile.ZIP_DEFLATED) as zout:
    names = zin.namelist()
    for item in zin.infolist():
        data = zin.read(item.filename)
        if item.filename == "Metadata/plate_1.json":
            d = json.loads(data); d["is_seq_print"] = True; data = json.dumps(d).encode()
        zout.writestr(item, data)
    if "Metadata/plate_1.json" not in names:
        zout.writestr("Metadata/plate_1.json", json.dumps({"is_seq_print": True}))
' proj-$e.3mf seqjson-$e.3mf
        proj_seq=$(py 'import json,zipfile,sys; print(json.loads(zipfile.ZipFile(sys.argv[1]).read("Metadata/project_settings.config")).get("print_sequence"))' proj-$e.3mf)
        run f6-$e "$bin" seqjson-$e.3mf --plate 1 -o f6-$e.gcode
        [ "$(rc f6-$e)" = 0 ] || { report F6 $e FAIL "run: $(why f6-$e)"; continue; }
        got=$(fh hdr f6-$e.gcode print_sequence)
        if [ "$got" = "$proj_seq" ]; then report F6 $e PASS "print_sequence = $got, the project's"
        else report F6 $e FAIL "plate_1.json is_seq_print made the plate print '$got'; the project (and its plate) say '$proj_seq'"; fi
    done
}

# ---------------------------------------------------------------- F7
# The model actions see the objects as the slice oriented them: the official
# orients each object once, with the loaded process settings
# (BambuStudio.cpp 5213-5234: o->config.assign_config(load_process_config),
# then orientation::orient reads support_threshold_angle, Orient.cpp 667-671).
# ba0dcfb's `whole` (--slice 0, several plates, a model action) runs the
# transforms with a nullptr process config, so --export-stls gets the 30
# degree default while the slice used the file's angle. The two plates hold a
# block leaning 50 degrees and the process file says 85. Bambu engine only:
# OrcaSlicer's CLI never assigns the process to the object (OrcaSlicer.cpp
# 4474-4495) and its orient() takes no angle.
F7() {
    report F7 orca SKIP "not applicable: OrcaSlicer.cpp 4474-4495 orients without the process settings"
    local e=bambu; bin=$B
    fx_list $e lean2 lean2.json; fx_settings $e x1c "$X1C"
    [ -s lean2-$e.3mf ] || { report F7 $e FAIL "fixture two-plate project: $(why fxlean2-$e)"; return; }
    py '
import json
d = json.load(open("x1c-bambu-process.json")); d["support_threshold_angle"] = "85"; d["name"] = "Lean Test Process"; d["from"] = "user"
json.dump(d, open("lean-process.json", "w"), indent=1)
'
    run f7-$e "$bin" lean2-$e.3mf --slice 0 --orient 1 --load-settings "x1c-$e-machine.json;lean-process.json" \
        --export-stls f7-$e/stls --outputdir f7-$e/out
    if [ "$(rc f7-$e)" != 0 ]; then report F7 $e FAIL "run: $(why f7-$e) (rc 139 = the engine's orient crash, see README)"; return; fi
    verdict=$(py '
import json, subprocess, sys
b = json.loads(subprocess.check_output(["python3", "fhelp.py", "stl_boxes", sys.argv[1]]))
d = json.load(open(sys.argv[2]))
sliced = sorted(round(o["bbox"]["height"], 1) for p in d["sliced_plates"] for o in p["objects"])
stl = sorted(round(x[5] - x[4], 1) for x in b)
print(("PASS" if stl == sliced and len(stl) == 2 else "FAIL") + " STL heights %s, sliced heights %s" % (stl, sliced))
' f7-$e/stls f7-$e/out/result.json)
    report F7 $e "${verdict%% *}" "${verdict#* }"
}

# ---------------------------------------------------------------- F8
# --export-3mf writes the model the run transformed and sliced (the official
# exports m_models[0] after its transforms: BambuStudio.cpp 4928-5109 then
# export_project 8156-8157; OrcaSlicer.cpp 4189-4370, 6985). ba0dcfb reloads
# the file, losing --scale, --rotate, --rotate-x, --rotate-y, --orient,
# --assemble, --convert-unit and --repetitions. Cases xsc/xrp/xs0 are 8b8614a's.
# Each case: the exported build items (world mesh size, rounded) sitting where
# the slice printed them (the official lifts nothing after --scale or
# --rotate-x, BambuStudio.cpp 5084-5109, ensure_on_bed only with
# --ensure-on-bed, 6188-6194; the engine prints only above the bed), then a
# re-slice of the export must print the same objects as the run.
F8() {
    for e in bambu orca; do
        bin=$(bin_of $e)
        fx_proj $e; fx_named $e slab slab.stl; fx_named $e tiny tiny.stl; fx_named $e lean lean.stl; fx_two $e; fx_skip2 $e; fx_inst2 $e
        fx_list $e three three.json
        id1=$(fh items skip2-$e.3mf 2>/dev/null | py 'import json,sys; print(json.load(sys.stdin)[0]["identify_id"])' 2>/dev/null)
        # A project with one object instanced on two plates (inst2 with instance 2 moved to plate 2).
        py '
import re, sys, zipfile
src, dst = sys.argv[1], sys.argv[2]
with zipfile.ZipFile(src) as zin, zipfile.ZipFile(dst, "w", zipfile.ZIP_DEFLATED) as zout:
    for item in zin.infolist():
        data = zin.read(item.filename)
        if item.filename == "3D/3dmodel.model":
            t = data.decode()
            b = t.index("<build")
            items = list(re.finditer(r"<item [^>]*/>", t[b:]))
            it = items[1].group(0)
            m = re.search(r"transform=\"([^\"]+)\"", it)
            v = m.group(1).split(); v[9] = "%g" % (float(v[9]) + 307.2)
            t = t[:b] + t[b:].replace(it, it.replace(m.group(1), " ".join(v)), 1)
            data = t.encode()
        if item.filename == "Metadata/model_settings.config":
            t = data.decode()
            s = t.index("  <plate>"); e2 = t.index("  </plate>") + len("  </plate>\n")
            plate = t[s:e2]
            insts = re.findall(r"    <model_instance>.*?</model_instance>\n", plate, re.S)
            p1 = plate.replace(insts[1], "")
            p2 = plate.replace(insts[0], "").replace("key=\"plater_id\" value=\"1\"", "key=\"plater_id\" value=\"2\"")
            t = t[:s] + p1 + p2 + t[e2:]
            data = t.encode()
        zout.writestr(item, data)
' inst2-$e.3mf split-$e.3mf 2>/dev/null
        bad=""
        # name : input : --slice : flags : wanted sorted item sizes
        for c in "xsc:proj-$e.3mf:1:--scale 2:[[40, 40, 40]]" \
                 "xrp:proj-$e.3mf:1:--repetitions 2:[[20, 20, 20], [20, 20, 20]]" \
                 "xs0:three-$e.3mf:0:--scale 2:[[40, 40, 40], [40, 40, 40], [40, 40, 40]]" \
                 "xrz:proj-$e.3mf:1:--rotate 45:[[28, 28, 20]]" \
                 "xrx:slab-$e.3mf:1:--rotate-x 90:[[30, 10, 20]]" \
                 "xry:slab-$e.3mf:1:--rotate-y 90:[[10, 20, 30]]" \
                 "xcu:tiny-$e.3mf:1:--convert-unit:[[38, 38, 38]]" \
                 "xas:two-$e.3mf:1:--assemble --arrange 1:sliced1" \
                 "xor:lean-$e.3mf:1:--orient 1:sliced" \
                 "xsp:split-$e.3mf:0:--scale 2:[[40, 40, 40], [40, 40, 40]]" \
                 "xsk:skip2-$e.3mf:1:--scale 2 --skip-objects $id1:[[40, 40, 40], [40, 40, 40]]"; do
            IFS=: read -r n in plate flags wantsz <<< "$c"
            [ -s "$in" ] || { bad="$bad $n: no input $in;"; continue; }
            run f8$n-$e "$bin" "$in" --slice $plate $flags --outputdir f8$n-$e/out --export-3mf x.3mf
            [ "$(rc f8$n-$e)" = 0 ] || { bad="$bad $n: $(why f8$n-$e);"; continue; }
            run f8r$n-$e "$bin" f8$n-$e/out/x.3mf --slice $plate --outputdir f8r$n-$e/out
            r=$(py '
import json, subprocess, sys, glob
n, d, rd, want, skip = sys.argv[1:6]
items = json.loads(subprocess.check_output(["python3", "fhelp.py", "items", d + "/x.3mf"]))
sizes = sorted(i["size"] for i in items)
msg = []
# What the run sliced (result.json object boxes) is what the export must hold.
res = json.load(open(d + "/result.json"))
sliced = sorted([round(o["bbox"]["width"]), round(o["bbox"]["depth"]), round(o["bbox"]["height"])]
                for p in res["sliced_plates"] for o in p["objects"])
printed = sorted(i["size"] for i in items if i["printable"])
if want.startswith("sliced"):
    if want == "sliced1" and len(items) != 1: msg.append("%d exported objects, want 1 (assembled)" % len(items))
    want = json.dumps(sliced)
elif sizes != json.loads(want): msg.append("exported sizes %s, want %s" % (sizes, want))
# (xsp: result.json has one entry per plate; xsk: ba0dcfb result.json also lists the skipped object.)
if printed != sliced and n not in ("xsp", "xsk"): msg.append("exported printable sizes %s, sliced %s" % (printed, sliced))
# The export sits as the slice printed it: the top of the highest printable
# object is the last Z of the G-code (the engine slices only above the bed, so an
# object the transforms left partly below it prints shorter).
import re as _re
tops = []
for g in sorted(glob.glob(d + "/plate_*.gcode")):
    zs = [float(z) for z in _re.findall(r"^; Z_HEIGHT: ([0-9.]+)", open(g, errors="replace").read(), _re.M)]
    if zs: tops.append(max(zs))
ptop = max((i["min"][2] + i["size"][2] for i in items if i["printable"]), default=None)
if tops and ptop is not None and abs(max(tops) - ptop) > 0.6:
    msg.append("exported top %.1f, G-code top Z %.1f" % (ptop, max(tops)))
if n == "xsp" and len({i["objectid"] for i in items}) != 1: msg.append("the two-plate object was split into %d objects" % len({i["objectid"] for i in items}))
if n == "xsk":
    p = [i["printable"] for i in items if i["identify_id"] == skip]
    if p != [False]: msg.append("skipped id %s printable=%s" % (skip, p))
def labels(dirname):
    out = []
    for g in sorted(glob.glob(dirname + "/plate_*.gcode")):
        out += json.loads(subprocess.check_output(["python3", "fhelp.py", "labels", g]))
    return sorted(out)
la, lb = labels(d), labels(rd)
if la != lb: msg.append("labels sliced %s, re-slice of the export %s" % (la, lb))
if n == "xrp":
    ids = sorted(int(i["identify_id"]) for i in items if i["identify_id"] and i["printable"])
    if ids != la: msg.append("exported identify_ids %s, sliced label ids %s" % (ids, la))
print("; ".join(msg) if msg else "ok")
' $n f8$n-$e/out f8r$n-$e/out "$wantsz" "$id1")
            [ "$r" = ok ] || bad="$bad $n ($flags): $r;"
        done
        if [ -z "$bad" ]; then report F8 $e PASS "every transform and repetition is in the exported 3MF, and its re-slice prints the same objects"
        else report F8 $e FAIL "$bad"; fi
    done
}

# ---------------------------------------------------------------- F9
# --repetitions with --skip-objects skips per instance (Codex P2, comment
# 4191597494; PartPlate::duplicate_all_instance walks every (object, instance)
# pair and skips by the instance's loaded_id, PartPlate.cpp 2820-2855). One
# cube object with two instances, --repetitions 2 (one copy): skipping an
# instance leaves the other one and its copy, 2 printed. The engine's 3MF
# loader gives an identify_id to one instance per object only, the one the
# plate lists first (bbs_3mf.cpp 4896: obj_inst_map.emplace keyed by
# object_id), so each case lists the skipped instance first: instance 1 in
# inst2, instance 2 in inst2r. ba0dcfb checks only instances.front() and
# copies the whole object: skipping instance 2 prints 4.
fx_inst2r() {
    fx_inst2 $1
    [ -s inst2r-$1.3mf ] && return 0
    [ -s inst2-$1.3mf ] || return 0
    py '
import re, sys, zipfile
src, dst = sys.argv[1], sys.argv[2]
with zipfile.ZipFile(src) as zin, zipfile.ZipFile(dst, "w", zipfile.ZIP_DEFLATED) as zout:
    for item in zin.infolist():
        data = zin.read(item.filename)
        if item.filename == "Metadata/model_settings.config":
            t = data.decode()
            b = re.findall(r"    <model_instance>.*?</model_instance>\n", t, flags=re.S)
            t = t.replace(b[0] + b[1], b[1] + b[0])
            data = t.encode()
        zout.writestr(item, data)
' inst2-$1.3mf inst2r-$1.3mf
}
F9() {
    for e in bambu orca; do
        fx_inst2r $e; bin=$(bin_of $e)
        [ -s inst2r-$e.3mf ] || { report F9 $e FAIL "fixture inst2 project: $(why fxskip2-$e)"; continue; }
        bad=""; got=""
        for k in 1 2; do
            f=inst2-$e.3mf; [ $k = 2 ] && f=inst2r-$e.3mf
            # The id of instance k: the one identify_id the loader keeps.
            id=$(fh items $f | py '
import json, sys
k = int(sys.argv[1]) - 1
print([i["identify_id"] for i in json.load(sys.stdin) if i["instance"] == k][0])' $k)
            run f9s$k-$e "$bin" $f --slice 1 --repetitions 2 --skip-objects "$id" --outputdir f9s$k-$e/out
            [ "$(rc f9s$k-$e)" = 0 ] || { bad="$bad skip instance $k: $(why f9s$k-$e);"; continue; }
            l=$(fh labels f9s$k-$e/out/plate_1.gcode)
            cnt=$(fh label_count f9s$k-$e/out/plate_1.gcode)
            got="$got skip instance $k (id $id) printed $cnt $l;"
            if [ "$cnt" != 2 ] || grep -qw "$id" <<<"$l"; then bad="$bad skip instance $k (id $id): printed $cnt objects $l, want 2 without $id;"; fi
        done
        if [ -z "$bad" ]; then report F9 $e PASS "$got"; else report F9 $e FAIL "$bad"; fi
    done
}

# ---------------------------------------------------------------- F7o
# --orient 1 on a model file: the official loads model files with their
# default instance (read_from_file(..., AddDefaultInstances), BambuStudio.cpp
# 1880-1889; OrcaSlicer.cpp 1562-1571), so orient() measures a mesh. ba0dcfb
# orients the object before the placement gives it an instance: an empty mesh,
# and a crash in AutoOrienter::get_features (rc 139) on every run. The official
# CLIs (BambuStudio 02.08.01.55, OrcaSlicer 2.4.0-alpha) orient the same files.
F7o() {
    for e in bambu orca; do
        bin=$(bin_of $e); bad=""
        for c in "info:--info" "stls:--export-stls f7o-$e-stls" "slice:--slice 1 --printer-preset $A1M_Q --outputdir f7o-$e-out"; do
            n=${c%%:*}; flags=${c#*:}
            eval "run f7o$n-$e \"\$bin\" lean.stl --orient 1 $flags"
            [ "$(rc f7o$n-$e)" = 0 ] || bad="$bad $n: $(why f7o$n-$e);"
        done
        if [ -z "$bad" ]; then
            h=$(fh stl_boxes f7o-$e-stls | py 'import json,sys; b=json.load(sys.stdin)[0]; print(round(b[5]-b[4],1))')
            report F7o $e PASS "--orient 1 on a model file: --info, --export-stls (height $h) and --slice 1 run"
        else report F7o $e FAIL "$bad"; fi
    done
}

# ---------------------------------------------------------------- F10
# result.json states the run's settings from the same config as
# --export-settings (sliced_info from m_print_config, BambuStudio.cpp
# 6905-6910; OrcaSlicer.cpp 5908-5913), not the plate's. An assemble list's
# plate_params are plate settings (BambuStudio.cpp 1071-1077): the G-code uses
# them, result.json and --export-settings state the project's. ba0dcfb reads
# the plate-overlaid config (main.cpp 4124-4127, 7764).
F10() {
    for e in bambu orca; do
        bin=$(bin_of $e)
        run f10-$e "$bin" --load-assemble-list pparams.json --slice 1 --printer-preset "$X1C" --export-settings f10-$e.json --outputdir f10-$e/out
        [ "$(rc f10-$e)" = 0 ] || { report F10 $e FAIL "run: $(why f10-$e)"; continue; }
        verdict=$(py '
import json, re, sys
r = json.load(open(sys.argv[1])); s = json.load(open(sys.argv[2])); g = open(sys.argv[3], errors="replace").read()
gv = lambda k: re.search(r"^; " + k + r" = (.*)$", g, re.M).group(1)
res = (round(r["layer_height"], 3), r["wall_loops"], round(r["sparse_infill_density"], 1))
exp = (round(float(s["layer_height"]), 3), int(s["wall_loops"]), round(float(s["sparse_infill_density"].rstrip("%")), 1))
gc = (gv("layer_height"), gv("wall_loops"), gv("sparse_infill_density"))
plate = gc == ("0.28", "5", "35%")
ok = res == exp and plate
print(("PASS" if ok else "FAIL") + " result.json %s, --export-settings %s, G-code %s (plate settings %s)" % (res, exp, gc, "applied" if plate else "NOT applied"))
' f10-$e/out/result.json f10-$e.json f10-$e/out/plate_1.gcode)
        report F10 $e "${verdict%% *}" "${verdict#* }"
    done
}

for blk in "${BLOCKS[@]}"; do
    case "$blk" in F1|F2|F3|F4|F5|F6|F7|F7o|F8|F9|F10) "$blk" ;; *) echo "unknown block $blk" ;; esac
done

echo
echo "== summary ($(basename "$B"), $(basename "$O"))"
printf '%s\n' "${RESULTS[@]}" | cut -c1-240
nfail=$(printf '%s\n' "${RESULTS[@]}" | grep -c '^FAIL' || true)
echo "FAIL=$nfail PASS=$(printf '%s\n' "${RESULTS[@]}" | grep -c '^PASS' || true) SKIP=$(printf '%s\n' "${RESULTS[@]}" | grep -c '^SKIP' || true)"
exit $(( nfail > 0 ? 1 : 0 ))
