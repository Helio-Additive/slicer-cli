#!/usr/bin/env bash
# test_refusal_sentences.sh: every refusal sentence reworded for the owner
# rule ("the sentence says what is wrong AND what to do instead") carries its
# "do" part in result.json's error_string — read from result.json, never from
# the exit status. The sentences checked here are the ones PR #35 added; text
# that existed on main e5027ad is not reworded.
#
# Usage: tests/test_refusal_sentences.sh SLICER_CLI SLICER_CLI_ORCASLICER
# Env: KEEP=<dir> keeps the work folder there.
#
# A sentence with no reachable trigger is listed under SKIPPED with the reason
# (nothing in the binary reaches it), so the gap is visible rather than
# silently untested.
set -uo pipefail

B="${1:?usage: $0 slicer_cli slicer_cli-orcaslicer}"
O="${2:?usage: $0 slicer_cli slicer_cli-orcaslicer}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
B="$(cd "$(dirname "$B")" && pwd -P)/$(basename "$B")"
O="$(cd "$(dirname "$O")" && pwd -P)/$(basename "$O")"
# The package's resources: beside the binary (the release packages) or one
# level up (a bin/ layout).
RES="$(cd "$(dirname "$B")" && pwd -P)/resources"
[ -d "$RES/profiles" ] || RES="$(cd "$(dirname "$B")/.." && pwd -P)/resources"
if [ -n "${KEEP:-}" ]; then WORKDIR="$KEEP"; mkdir -p "$WORKDIR"
else WORKDIR="$(mktemp -d)"; trap 'rm -rf "$WORKDIR"' EXIT; fi
cp "$HERE/tools/fhelp.py" "$HERE/tools/resolve_preset.py" "$WORKDIR/"
cd "$WORKDIR"

A1M="Bambu Lab A1 mini 0.4 nozzle"
X1C="Bambu Lab X1 Carbon 0.4 nozzle"
RESULTS=()
# report NAME ENGINE STATUS TEXT
report() { local line="$3: $1 [$2] $4"; echo "$line"; RESULTS+=("$line"); }
# run NAME BINARY ARGS... : stdout/stderr/rc kept under NAME/; never aborts.
run() {
    local name=$1; shift
    rm -rf "$name"; mkdir -p "$name"
    "$@" > "$name/stdout" 2> "$name/stderr" < /dev/null
    echo $? > "$name/rc"
}
why() { echo "exit $(cat "$1/rc" 2>/dev/null); $(grep -h '^Error' "$1/stderr" "$1/stdout" 2>/dev/null | tail -n 1 | cut -c1-200)"; }
bin_of() { [ "$1" = orca ] && echo "$O" || echo "$B"; }

# ---------------------------------------------------------------- fixtures
python3 - <<'PY'
import json, struct

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
box("big.stl", 300, 10)          # larger than the A1 mini bed in both directions
box("big200.stl", 200, 20)       # fits the bed alone, not two of them together
open("empty.stl", "w").write("solid t\nendsolid t\n")
open("zero.stl", "wb").write(b"\0" * 80 + struct.pack("<I", 0))
open("garbage.json", "w").write("not json at all")
open("junk.obj", "w").write("this is not an obj\n")

def lst(path, obj, arrange=False):
    json.dump({"plates": [{"plate_name": "p", "need_arrange": arrange, "objects": [obj]}]}, open(path, "w"))

lst("nopath.json", {"path": "missing.stl", "count": 1, "filaments": [1]})
lst("emptylist.json", {"path": "cube.stl", "count": 1, "filaments": [1]})
lst("nofil.json", {"path": "cube.stl", "count": 1, "filaments": []})
lst("readfail.json", {"path": "empty.stl", "count": 1, "filaments": [1]})
lst("junkobj.json", {"path": "junk.obj", "count": 1, "filaments": [1]})
# Two 200 mm cubes on one plate of the 180 mm A1 mini bed: each fits alone,
# the two do not fit together.
json.dump({"plates": [{"plate_name": "p", "need_arrange": True,
                       "objects": [{"path": "big200.stl", "count": 2, "filaments": [1, 1]}]}]},
          open("pair.json", "w"))
# Two cubes, one filament each, for the plate/filament refusal: PLA Basic and
# PETG HF share no plate the run would print both on at the same temperature.
json.dump({"plates": [{"plate_name": "p", "need_arrange": False,
                       "objects": [{"path": "cube.stl", "count": 1, "filaments": [1], "pos_x": [100], "pos_y": [150]},
                                   {"path": "cube.stl", "count": 1, "filaments": [2], "pos_x": [140], "pos_y": [150]}]}]},
          open("twomix.json", "w"))
# A plate the A1 mini cannot hold: a 145 mm box clear of the X1 Carbon's
# exclusion area (18 x 28 mm at the origin) and a 40 mm box near the A1 mini's
# 180 mm edge, both inside the X1 Carbon's bed, for the -52 and -21 refusals.
box("bedbig.stl", 145, 10)
box("bededge.stl", 40, 10)
json.dump({"plates": [{"plate_name": "p", "need_arrange": False,
                       "objects": [{"path": "bedbig.stl", "count": 1, "filaments": [1], "pos_x": [20], "pos_y": [32]},
                                   {"path": "bededge.stl", "count": 1, "filaments": [1], "pos_x": [170], "pos_y": [90]}]}]},
          open("bedproj.json", "w"))
json.dump({"plates": []}, open("noplates.json", "w"))
PY
# An assemble list whose plate holds no object.
python3 -c '
import json
json.dump({"plates": [{"plate_name": "p", "need_arrange": False, "objects": []}]}, open("noobj.json", "w"))
'

# Settings files: the engine's own system presets for PRINTER, flattened
# (inherits walked), as the official CLIs read them: <tag>-<e>-machine.json,
# -process.json, -filament.json.
fx_settings() {  # fx_settings ENGINE TAG PRINTER
    local e=$1 t=$2 p=$3 v="$RES/profiles/BBL"
    [ "$e" = orca ] && v="$RES/profiles-orca/BBL"
    [ -s "$t-$e-machine.json" ] && return 0
    [ -d "vendor-$e" ] || cp -R "$v" "vendor-$e"
    python3 resolve_preset.py "vendor-$e" "$p" "$t-$e" > /dev/null
}
# A copy of a preset file the package ships, with its "inherits" renamed to a
# parent this engine does not have.
fx_partial() {  # fx_partial ENGINE PRESET
    python3 - "$1" "$2" <<'PY'
import json, sys
engine, name = sys.argv[1], sys.argv[2]
d = json.load(open("vendor-%s/machine/%s.json" % (engine, name), encoding="utf-8"))
d["inherits"] = "fdm_no_such_parent"
json.dump(d, open("partial-%s.json" % engine, "w"), indent=1)
PY
}

# proj-<e>.3mf / x1c-proj-<e>.3mf: a 20 mm cube as the engine's own project.
fx_proj() {  # fx_proj ENGINE NAME PRESET
    [ -s "$2-$1.3mf" ] && return 0
    run "fx$2-$1" "$(bin_of "$1")" cube.stl --slice 1 --printer-preset "$3" \
        --outputdir "fx$2-$1/out" --export-3mf "$2.3mf"
    cp "fx$2-$1/out/$2.3mf" "$2-$1.3mf" 2>/dev/null
}
# bedproj-<e>.3mf: bedproj.json as that engine's own project, made on the X1
# Carbon (both objects are inside its bed), to slice on the A1 mini.
fx_bedproj() {  # fx_bedproj ENGINE
    [ -s "bedproj-$1.3mf" ] && return 0
    run "fxbed-$1" "$(bin_of "$1")" --load-assemble-list bedproj.json --slice 1 \
        --printer-preset "$X1C" --outputdir "fxbed-$1/out" --export-3mf bedproj.3mf
    cp "fxbed-$1/out/bedproj.3mf" "bedproj-$1.3mf" 2>/dev/null || true
}
# A copy of the engine's own project with the given project_settings.config
# keys replaced.
fx_rewrite() {  # fx_rewrite SRC DST KEY=VALUE...
    python3 - "$@" <<'PY'
import json, sys, zipfile
src, dst, pairs = sys.argv[1], sys.argv[2], [a.split("=", 1) for a in sys.argv[3:]]
with zipfile.ZipFile(src) as zin, zipfile.ZipFile(dst, "w", zipfile.ZIP_DEFLATED) as zout:
    for item in zin.infolist():
        data = zin.read(item.filename)
        if item.filename == "Metadata/project_settings.config":
            d = json.loads(data)
            for k, v in pairs:
                d[k] = json.loads(v)
            data = json.dumps(d, indent=4).encode()
        zout.writestr(item, data)
PY
}

# ---------------------------------------------------------------- the checks
# check NAME ENGINE WANT ARGS...: run ARGS on that engine's binary and require
# result.json's error_string to match WANT (a shell pattern, so "*a*b*" checks
# two parts of one sentence).
check() {
    local n=$1 e=$2 want=$3; shift 3
    local bin; bin=$(bin_of "$e")
    run "c-$n-$e" "$bin" "$@" --outputdir "c-$n-$e/out"
    local got
    got=$(python3 -c 'import json,sys
try: print(json.load(open(sys.argv[1]))["error_string"])
except Exception: print("")' "c-$n-$e/out/result.json" 2>/dev/null)
    if [ -z "$got" ]; then
        report "$n" "$e" FAIL "no result.json; $(why "c-$n-$e")"
    else
        case "$got" in
        $want) report "$n" "$e" PASS "error_string carries: $want" ;;
        *)     report "$n" "$e" FAIL "error_string does not match '$want': $got" ;;
        esac
    fi
}
both() { check "$1" bambu "$2" "${@:3}"; check "$1" orca "$2" "${@:3}"; }

# check_slices NAME ENGINE KEY VALUE ARGS...: the run must SUCCEED (result.json
# return_code 0) and the G-code header must carry KEY = VALUE. It is what the
# hint on the range refusal promises: the flag it names, with an in-range
# value, gets past the check and reaches the slice.
check_slices() {
    local n=$1 e=$2 key=$3 val=$4; shift 4
    local bin; bin=$(bin_of "$e")
    run "c-$n-$e" "$bin" "$@" --outputdir "c-$n-$e/out"
    local code got
    code=$(python3 -c 'import json,sys
try: print(json.load(open(sys.argv[1]))["return_code"])
except Exception: print("none")' "c-$n-$e/out/result.json" 2>/dev/null)
    got=$(sed -n "s/^; $key = //p" "c-$n-$e/out/plate_1.gcode" 2>/dev/null | head -n 1)
    if [ "$code" = 0 ] && [ "$got" = "$val" ]; then
        report "$n" "$e" PASS "the hinted override slices: return_code 0, $key = $got"
    else
        report "$n" "$e" FAIL "the hinted override did not slice: return_code '$code', $key '$got'; $(why "c-$n-$e")"
    fi
}

# ---------------------------------------------------------------- setup
for e in bambu orca; do
    fx_settings "$e" a1m "$A1M"
    fx_settings "$e" x1c "$X1C"
    fx_proj "$e" proj "$A1M"
    fx_proj "$e" x1c-proj "$X1C"
    fx_proj "$e" p02 "Bambu Lab A1 mini 0.2 nozzle"
    fx_proj "$e" p08 "Bambu Lab A1 mini 0.8 nozzle"
    [ -s "proj-$e.3mf" ] || { echo "FAIL: fixture proj-$e.3mf: $(why "fxproj-$e")"; exit 1; }
    [ -s "p02-$e.3mf" ] || { echo "FAIL: fixture p02-$e.3mf: $(why "fxp02-$e")"; exit 1; }
    [ -s "p08-$e.3mf" ] || { echo "FAIL: fixture p08-$e.3mf: $(why "fxp08-$e")"; exit 1; }
    [ -s "x1c-proj-$e.3mf" ] || { echo "FAIL: fixture x1c-proj-$e.3mf: $(why "fxx1c-proj-$e")"; exit 1; }
done
for e in bambu orca; do
    fx_rewrite "proj-$e.3mf" "post-$e.3mf" 'post_process=["/bin/true"]'
    fx_rewrite "proj-$e.3mf" "mixed-$e.3mf" 'filament_is_mixed=["1"]'
    fx_rewrite "proj-$e.3mf" "range-$e.3mf" 'support_threshold_angle="999"'
    fx_rewrite "proj-$e.3mf" "spiral-$e.3mf" 'spiral_mode="1"'
    fx_rewrite "proj-$e.3mf" "spiralr-$e.3mf" 'spiral_mode="1"' 'support_threshold_angle="999"'
done
# The 0.8 nozzle projects with a line width that fits 0.8 and not 0.4.
fx_rewrite "p08-orca.3mf" "p08w-orca.3mf" 'bridge_line_width="0.6"'
fx_rewrite "p08-bambu.3mf" "p08w-bambu.3mf" 'outer_wall_line_width="1.6"'
# The 0.2 nozzle projects with a line width already too wide for 0.2.
fx_rewrite "p02-orca.3mf" "p02w-orca.3mf" 'bridge_line_width="0.3"'
fx_rewrite "p02-bambu.3mf" "p02w-bambu.3mf" 'outer_wall_line_width="0.8"'

# ---------------------------------------------------------------- option combinations
both assemble-clone           "*give one of them, not both*" \
     cube.stl --slice 1 --assemble --clone-objects 2
check slicedata-both orca "*give one of them: load a saved slicing, or write one*" cube.stl --slice 1 --load-slicedata d --export-slicedata e
both slicedata-rep            "*drop --repetitions, or slice without --load-slicedata*" \
     cube.stl --slice 1 --load-slicedata d --repetitions 2
both layout-plan-layout       "*give one of them*" \
     cube.stl --slice 1 --layout-plan --layout x.json
both plate-not-number         "*give 0 for every plate, or a plate number*" \
     cube.stl --slice 1 --plate abc
both load-filament-ids        "*give --load-filaments with at least 3 file(s)*" \
     cube.stl --slice 1 --load-filament-ids 3
both clone-objects-max        "*copies or fewer*" \
     cube.stl --slice 1 --clone-objects 9999
both object-larger-than-bed   "*Scale the object down, split it across plates, or use a bigger printer.*" \
     big.stl --slice 1 --arrange 1 --printer-preset "$A1M"
both assemble-no-fit          "*Give the plate fewer objects or smaller copies, or use a bigger printer.*" \
     --load-assemble-list pair.json --slice 1 --printer-preset "$A1M"

# ---------------------------------------------------------------- the run's project
# Everything below is an input the engine itself made, so the project settings are
# the ones that engine accepts.
for e in bambu orca; do
    p="proj-$e.3mf"          # a 20 mm cube on the A1 mini
    x="x1c-proj-$e.3mf"      # the same cube on the X1 Carbon
    check mtcpp-limit              "$e" "*raise --mtcpp above 12, or split the plate*" \
          "$p" --slice 1 --mtcpp 1
    check settings-two-machines    "$e" "*give one machine file*" \
          "$p" --slice 1 --load-settings "a1m-$e-machine.json;a1m-$e-machine.json"
    check settings-two-processes   "$e" "*give one process file*" \
          "$p" --slice 1 --load-settings "a1m-$e-process.json;a1m-$e-process.json"
    check uptodate-wrong-type      "$e" "*give a machine or process file*" \
          "$p" --slice 1 --uptodate --uptodate-settings "a1m-$e-filament.json"
    check uptodate-machine         "$e" "*give the project's own machine file*" \
          "$p" --slice 1 --uptodate --uptodate-settings "x1c-$e-machine.json"
    check uptodate-process         "$e" "*give the project's own process file*" \
          "$p" --slice 1 --uptodate --uptodate-settings "x1c-$e-process.json"
    # A shipped preset file with its parent renamed to one this engine does not
    # ship: the file holds its differences from a preset that is not there, and
    # reading it as it stands would put those keys on the project and leave the
    # rest of the printer at the old one's. It refuses before slicing, and says
    # which preset to pass by name.
    fx_partial "$e" "$A1M"
    check partial-preset           "$e" "*is a partial preset that inherits 'fdm_no_such_parent'; pass --printer-preset \"$A1M\" instead*" \
          "$p" --slice 1 --load-settings "partial-$e.json"
    check uptodate-fila-count      "$e" "*give one file per filament, in the project's order*" \
          "$p" --slice 1 --uptodate --uptodate-filaments "a1m-$e-filament.json;a1m-$e-filament.json"
    check uptodate-fila-mismatch   "$e" "*give the project's own filament files, in the project's order*" \
          "$p" --slice 1 --uptodate --uptodate-filaments "x1c-$e-filament.json"
    check load-custom-gcodes       "$e" "*check the file's path*" \
          "$p" --slice 1 --load-custom-gcodes /nope.json
    check downward-settings        "$e" "*give a system machine preset*" \
          "$p" --slice 1 --downward-check --downward-settings "a1m-$e-filament.json"
    check move-to-printer          "$e" "*give a process that printer suits*" \
          "$x" --slice 1 --load-settings "a1m-$e-machine.json;x1c-$e-process.json"
    check process-not-suit         "$e" "*give a process that printer suits*" \
          "$x" --slice 1 --load-settings "x1c-$e-machine.json;a1m-$e-process.json"
    check post-process             "$e" "*clear post_process in the file, or slice it in the desktop app*" \
          "post-$e.3mf" --slice 1
    # A finding only the CLI check gives is refused, with the flags to give:
    # spiral vase mode with more than one wall (under_cli only; the desktop
    # asks the user in a dialog there). A value out of range in the file is
    # only noted (the desktop app's notice on load) and the run slices with it.
    check value-out-of-range       "$e" "*wall_loops: Invalid value when spiral vase mode is enabled*--wall-loops a value in range to override it.*" \
          "spiral-$e.3mf" --slice 1
    check_slices value-out-of-range-file "$e" support_threshold_angle 999 \
          "range-$e.3mf" --slice 1
    if grep '"tag":"InvalidValuesNoted"' "c-value-out-of-range-file-$e/stdout" | grep -q support_threshold_angle; then
        report value-out-of-range-noted "$e" PASS "InvalidValuesNoted names support_threshold_angle"
    else
        report value-out-of-range-noted "$e" FAIL "no InvalidValuesNoted warning naming support_threshold_angle"
    fi
    # The hint the warning gives: the flag it names, with a value in range,
    # slices with that value.
    check_slices value-out-of-range-retry "$e" support_threshold_angle 45 \
          "range-$e.3mf" --slice 1 --support-threshold-angle 45
    # A flag whose value the command line alone accepts (the check against
    # the default 0.4 nozzle) but the project's 0.2 nozzle does not: a
    # command-line value, so refused with its hint, not noted. The same
    # project without the flag slices.
    if [ "$e" = orca ]; then
        check flag-over-nozzle     "$e" "*bridge_line_width: Bridge line width must not exceed nozzle diameter*Give --bridge-line-width a value in range to override it.*" \
              "p02-$e.3mf" --slice 1 --bridge-line-width 0.3
    else
        check flag-over-nozzle     "$e" "*outer_wall_line_width: too large line width*Give --outer-wall-line-width a value in range to override it.*" \
              "p02-$e.3mf" --slice 1 --outer-wall-line-width 0.8
    fi
    check_slices flag-over-nozzle-without "$e" nozzle_diameter 0.2 "p02-$e.3mf" --slice 1
    # A flag that makes another setting of the file out of range: the 0.8
    # nozzle project's line width fits 0.8, and --nozzle-diameter 0.4 makes it
    # too wide. The command line brought the finding, so it is refused; the
    # same project without the flag slices.
    if [ "$e" = orca ]; then
        check flag-cross-key       "$e" "*bridge_line_width: Bridge line width must not exceed nozzle diameter*" \
              "p08w-$e.3mf" --slice 1 --nozzle-diameter 0.4
        check_slices flag-cross-key-without "$e" bridge_line_width 0.6 "p08w-$e.3mf" --slice 1
    else
        check flag-cross-key       "$e" "*outer_wall_line_width: too large line width*" \
              "p08w-$e.3mf" --slice 1 --nozzle-diameter 0.4
        check_slices flag-cross-key-without "$e" outer_wall_line_width 1.6 "p08w-$e.3mf" --slice 1
    fi
    # The file's value is already out of range for its nozzle, and the flag
    # gives the same setting the same value: the command line's value, so
    # refused by the settings check. The file alone is only noted there (the
    # slice's own check, Print::validate, then gates it as the desktop does).
    if [ "$e" = orca ]; then
        check flag-same-as-file    "$e" "*bridge_line_width: Bridge line width must not exceed nozzle diameter*Give --bridge-line-width a value in range to override it.*" \
              "p02w-$e.3mf" --slice 1 --bridge-line-width 0.3
    else
        check flag-same-as-file    "$e" "*outer_wall_line_width: too large line width*Give --outer-wall-line-width a value in range to override it.*" \
              "p02w-$e.3mf" --slice 1 --outer-wall-line-width 0.8
    fi
    run "c-flag-same-as-file-without-$e" "$(bin_of "$e")" "p02w-$e.3mf" --slice 1 --outputdir "c-flag-same-as-file-without-$e/out"
    if grep -q '"tag":"InvalidValuesNoted"' "c-flag-same-as-file-without-$e/stdout"; then
        report flag-same-as-file-without "$e" PASS "the file's own value is noted, not refused by the settings check"
    else
        report flag-same-as-file-without "$e" FAIL "no InvalidValuesNoted for the file's own value; $(why "c-flag-same-as-file-without-$e")"
    fi
    # A refused finding (spiral vase with two walls) beside a noted one (a
    # value out of range in the file): refused, and the noted one is not
    # announced as sliced.
    check refused-with-noted       "$e" "*wall_loops: Invalid value when spiral vase mode is enabled*" \
          "spiralr-$e.3mf" --slice 1
    if grep -q '"tag":"InvalidValuesNoted"' "c-refused-with-noted-$e/stdout"; then
        report refused-with-noted-quiet "$e" FAIL "InvalidValuesNoted announced on a refused run"
    else
        report refused-with-noted-quiet "$e" PASS "no InvalidValuesNoted on a refused run"
    fi
done

# A settings file of the wrong kind, and --load-defaultfila with no usable file.
for e in bambu orca; do
    check load-filaments-wrong     "$e" "*is not a filament file; give it with --load-filaments*" \
          cube.stl --slice 1 --load-filaments "a1m-$e-machine.json"
    check load-defaultfila         "$e" "*check the paths given to --load-filaments*" \
          cube.stl --slice 1 --load-defaultfila --load-filaments ";"
done

# The mixed-filament check is BambuStudio's (the OrcaSlicer CLI has none).
check mixed-filament bambu "*give the project one filament per extruder, or slice it in the desktop app*" \
      "mixed-bambu.3mf" --slice 1

# The plate/filament refusal (-61) names the plates the filaments do share, not
# only the one they do not. Two filaments of very different temperatures on the
# Cool Plate, allowed by --allow-mix-temp, are refused for it — Bambu PETG HF
# @BBL X1C has cool_plate_temp 0 (Print.cpp 1700-1726 at 31f6803; 1650-1672 at
# 5873b5f). The plates the sentence names are the ones both filaments have a bed
# temperature on. Both runs place the two-filament prime tower by hand: at the
# desktop's own spot for it this pair's tower runs past the engine's G-code
# check (-104, GCodeProcessor.cpp 1861-1883; the tower-outside case below),
# which is not this sentence's subject.
both plate-filament-hint  "*Pick a plate this filament supports: --curr-bed-type \"High Temp Plate\" (or*" \
     --load-assemble-list twomix.json --slice 1 --printer-preset "$X1C" \
     --filament-preset "Bambu PLA Basic @BBL X1C" --filament-preset "Bambu PETG HF @BBL X1C" \
     --allow-mix-temp=1 --curr-bed-type "Cool Plate" --wipe-tower-x 30 --wipe-tower-y 220
# The retry the hint promises: the plate it names first takes both filaments.
check_slices plate-filament-retry bambu curr_bed_type "High Temp Plate" \
     --load-assemble-list twomix.json --slice 1 --printer-preset "$X1C" \
     --filament-preset "Bambu PLA Basic @BBL X1C" --filament-preset "Bambu PETG HF @BBL X1C" \
     --allow-mix-temp=1 --curr-bed-type "High Temp Plate" --wipe-tower-x 30 --wipe-tower-y 220
check_slices plate-filament-retry-orca orca curr_bed_type "High Temp Plate" \
     --load-assemble-list twomix.json --slice 1 --printer-preset "$X1C" \
     --filament-preset "Bambu PLA Basic @BBL X1C" --filament-preset "Bambu PETG HF @BBL X1C" \
     --allow-mix-temp=1 --curr-bed-type "High Temp Plate" --wipe-tower-x 30 --wipe-tower-y 220

# The prime-tower refusal (-104) says how to move the tower or turn it off. On
# the Orca build the tower stands where the desktop puts it for a new plate
# (set_default_wipe_tower_pos_for_plate, PartPlate.cpp 4115-4190), but this
# pair's tower is deeper than that spot's estimate and runs past the 2 mm the
# engine's G-code check allows (GCodeProcessor.cpp 1861-1883). Each flag the
# sentence names gets the same plate to slice.
TOWER=(--load-assemble-list twomix.json --slice 1 --printer-preset "$X1C"
       --filament-preset "Bambu PLA Basic @BBL X1C" --filament-preset "Bambu PETG HF @BBL X1C"
       --allow-mix-temp=1 --curr-bed-type "Textured PEI Plate")
check tower-outside orca \
      "*It comes from the prime tower*Move the prime tower with --wipe-tower-x and --wipe-tower-y, or turn it off with --enable-prime-tower=0." \
      "${TOWER[@]}"
check_slices tower-moved orca wipe_tower_y 200.000 "${TOWER[@]}" --wipe-tower-x 30 --wipe-tower-y 200
check_slices tower-off orca enable_prime_tower 0 "${TOWER[@]}" --enable-prime-tower=0

# The two bed refusals say what to do. A project made on the X1 Carbon (both
# objects inside its bed) sliced on the A1 mini: one object crosses its 180 mm
# edge (-52, the official's partly-inside gate at BambuStudio.cpp 6527-6567 /
# OrcaSlicer.cpp 5645-5697), and with --arrange 1 the two cannot share that bed
# (-21, the arrange's own verdict at BambuStudio.cpp 5936-5945 /
# OrcaSlicer.cpp 5196-5205).
for e in bambu orca; do
    fx_bedproj "$e"
    check bed-edge-refusal      "$e" \
          "*crosses the edge of the 180 x 180 x 180 mm bed.*Give --arrange 1 to re-arrange the plate on this bed, or use a printer with a bigger bed.*" \
          "bedproj-$e.3mf" --slice 1 --printer-preset "$A1M"
    check bed-arrange-refusal   "$e" \
          "*do not fit on the 180 x 180 x 180 mm bed together:*Move some objects to another plate in the file, or use a printer with a bigger bed.*" \
          "bedproj-$e.3mf" --slice 1 --printer-preset "$A1M" --arrange 1
done

# ---------------------------------------------------------------- the assemble list
both assemble-list-file-missing "*does not exist; check the path given to --load-assemble-list*" \
     --load-assemble-list nolist.json --slice 1 --printer-preset "$A1M"
both assemble-list-missing    "*which does not exist; check the path*" \
     --load-assemble-list nopath.json --slice 1 --printer-preset "$A1M"
both assemble-list-empty      "*give the plate at least one object*" \
     --load-assemble-list noobj.json --slice 1 --printer-preset "$A1M"
both assemble-list-badjson    "*could not be read*; check the file*" \
     --load-assemble-list garbage.json --slice 1 --printer-preset "$A1M"
both assemble-no-filament     "*give a filament for it*" \
     --load-assemble-list nofil.json --slice 1 --printer-preset "$A1M"
both assemble-stl-read        "*could not be read; check the file*" \
     --load-assemble-list readfail.json --slice 1 --printer-preset "$A1M"
both assemble-obj-mesh        "*holds no usable mesh*; check the file*" \
     --load-assemble-list junkobj.json --slice 1 --printer-preset "$A1M"

# ---------------------------------------------------------------- summary
echo
PASSED=$(printf '%s\n' ${RESULTS[@]+"${RESULTS[@]}"} | grep -c '^PASS:')
FAILED=$(printf '%s\n' ${RESULTS[@]+"${RESULTS[@]}"} | grep -c '^FAIL:')
echo
echo "Passed: $PASSED"
echo "Failed: $FAILED"
echo "Skipped: 10 (no input reaches the sentence):"
echo "  --<flag> is not implemented (cli_run_steps.cpp); 'The STL ... holds no mesh';"
echo "  'The STL ... could not be added'; 'The OBJ ... could not be added';"
echo "  'The colours of the OBJ ... could not be turned into filaments';"
echo "  'The assemble list's parts could not be built'; 'The model files hold no object to print';"
echo "  'The new printer names no default process'; '<f> is not a system process file';"
echo "  'The OBJ ... uses the material ...' (this input crashes the engine, rc 139)."
[ "$FAILED" -eq 0 ]
