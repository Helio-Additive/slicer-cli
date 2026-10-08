#!/usr/bin/env bash
# The settings files a real user or agent has, read the way the desktop reads
# them: one case per row of the input-path audit whose cause is a settings file
# read without its parents (SILENT-WRONG 1-17) or a refusal a real file must not
# get (BAD-REFUSAL 1, 2, 4, 5, 6).
#
# Every case runs the file the PACKAGE SHIPS, or one the desktop itself wrote (a
# user preset, an "Export current configs" file), never a flattened copy. The
# expected values come from the vendor tree: tools/check_preset_gcode.py takes
# the file's own keys out of the preset's flattened chain and requires every
# other key in the G-code header to be that parent's value. The pre-fix build
# slices with the old printer's or the engine's default for those keys, so every
# parents() check fails there.
#
# Usage: tests/test-inherits-inputs.sh /path/to/slicer_cli /path/to/slicer_cli-orcaslicer
set -uo pipefail

B="${1:?usage: $0 /path/to/packaged/slicer_cli /path/to/packaged/slicer_cli-orcaslicer}"
O="${2:?usage: $0 /path/to/packaged/slicer_cli /path/to/packaged/slicer_cli-orcaslicer}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
CHECK="$SCRIPT_DIR/tools/check_preset_gcode.py"
WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT
B="$(cd "$(dirname "$B")" && pwd -P)/$(basename "$B")"
O="$(cd "$(dirname "$O")" && pwd -P)/$(basename "$O")"
RES="$(cd "$(dirname "$B")" && pwd -P)/resources"
[ -d "$RES/profiles" ] || RES="$(cd "$(dirname "$B")/.." && pwd -P)/resources"
BBL="$RES/profiles/BBL"
OCA="$RES/profiles-orca"
[ -d "$OCA/Snapmaker" ] || { echo "SKIP: no OrcaSlicer profiles tree beside $O"; exit 0; }
cd "$WORKDIR"

PASS=0; FAILED=0
fail() { echo "FAIL: $*"; FAILED=$((FAILED + 1)); }
run() {  # run NAME BINARY ARGS...
    local name=$1; shift
    mkdir -p "$name"
    timeout 1800 "$@" > "$name/stdout" 2> "$name/stderr"
    echo $? > "$name/rc"
}
rc() { cat "$1/rc"; }
why() { tail -n 2 "$1/stderr" | tr '\n' '|' | cut -c1-220; }
gcode() { ls "$1"/out/plate_*.gcode 2>/dev/null | head -1; }
# parents NAME VENDOR KIND LEAF...: every key the leaf leaves to its parents,
# read out of the G-code header and compared with the flattened chain.
parents() {
    local name=$1 vendor=$2 kind=$3; shift 3
    local g; g=$(gcode "$name")
    if [ -z "$g" ]; then fail "$name: no G-code"; return; fi
    local out
    if out=$(python3 "$CHECK" "$vendor" "$kind" "$@" "$g" 2>&1); then
        echo "   ${out%%$'\n'*}"
    else
        echo "$out"; fail "$name: the $kind file was not read over its parents"
    fi
}
# header NAME KEY: the G-code header value.
header() { sed -n "s/^; $2 = //p" "$(gcode "$1")" 2>/dev/null | head -n 1; }
# expect NAME KEY VALUE WHY
expect() {
    local got; got=$(header "$1" "$2")
    [ "$got" = "$3" ] || fail "$4: $2 is '$got', want '$3'"
}
# case_done BEFORE DESCRIPTION
case_done() { [ "$FAILED" = "$1" ] && { echo "PASS: $2"; PASS=$((PASS + 1)); }; }

# ── fixtures ───────────────────────────────────────────────────────────────
# A 20 mm cube, and the engine's own projects for the printers the rows switch
# to (the audit's PA and H2D projects are the same shape of input).
python3 - <<'PY'
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
PY
# A1M.3MF: an A1 mini project (the audit's project is an A1 mini project), and
# H2D.3MF: the two-nozzle project row 11 switches. --export-3mf writes into
# --outputdir, so the project is taken from there.
run fxa1m "$B" cube.stl --slice 1 --printer-preset "Bambu Lab A1 mini 0.4 nozzle" \
    --outputdir fxa1m/out --export-3mf a1m.3mf
run fxh2d "$B" cube.stl --slice 1 --printer-preset "Bambu Lab H2D 0.4 nozzle" \
    --outputdir fxh2d/out --export-3mf h2d.3mf
cp fxa1m/out/a1m.3mf . 2>/dev/null
cp fxh2d/out/h2d.3mf . 2>/dev/null
[ -s a1m.3mf ] && [ -s h2d.3mf ] || { echo "FAIL: the project fixtures could not be made ($(why fxa1m))"; exit 1; }

# The files the desktop writes: a user preset (from "User", the keys it changed,
# the system preset it was saved over), with and without the "type" the desktop
# leaves out, and a printer preset the same way.
python3 - <<'PY'
import json
def write(path, meta, keys):
    d = dict(meta); d.update(keys)
    json.dump(d, open(path, "w"), indent=1)
write("user-petg-typed.json", {"type": "filament", "name": "My PETG 250", "from": "User", "version": "2.0.0.0"},
      {"inherits": "Bambu PETG HF @BBL A1M", "nozzle_temperature": ["250"],
       "filament_settings_id": ["My PETG 250"], "is_custom_defined": "1"})
write("user-pla-untyped.json", {"name": "My PLA 225", "from": "User", "version": "2.0.0.0"},
      {"inherits": "Generic PLA @BBL A1M", "nozzle_temperature": ["225"],
       "filament_settings_id": ["My PLA 225"], "is_custom_defined": "1"})
write("user-x1c-typed.json", {"type": "machine", "name": "My X1C", "from": "User", "version": "2.0.0.0"},
      {"inherits": "Bambu Lab X1 Carbon 0.4 nozzle", "retraction_length": ["1.2"],
       "printer_settings_id": ["My X1C"], "is_custom_defined": "1"})
PY

# ── rows 1-4: the settings-file flags, STL ─────────────────────────────────
# Row 1 (B) and row 4 (B, O): the shipped partial trio.
before=$FAILED
run r1 "$B" cube.stl --slice 1 \
    --machine "$BBL/machine/Bambu Lab X1 Carbon 0.4 nozzle.json" \
    --process "$BBL/process/0.20mm Standard @BBL X1C.json" \
    --filament "$BBL/filament/Bambu PLA Basic @BBL X1C.json" --outputdir r1/out
[ "$(rc r1)" = 0 ] || { echo "   $(why r1)"; fail "row 1: exit $(rc r1)"; }
parents r1 "$BBL" machine "Bambu Lab X1 Carbon 0.4 nozzle"
parents r1 "$BBL" process "0.20mm Standard @BBL X1C"
parents r1 "$BBL" filament "Bambu PLA Basic @BBL X1C"
case_done "$before" "row 1: --machine/--process/--filament read the shipped BBL files over their parents"

before=$FAILED
run r2 "$O" cube.stl --slice 1 \
    --machine "$OCA/Snapmaker/machine/Snapmaker U1 (0.4 nozzle).json" \
    --process "$OCA/Snapmaker/process/0.20 Standard @Snapmaker U1 (0.4 nozzle).json" \
    --filament "$OCA/OrcaFilamentLibrary/filament/Generic PLA @System.json" --outputdir r2/out
[ "$(rc r2)" = 0 ] || { echo "   $(why r2)"; fail "row 2: exit $(rc r2)"; }
parents r2 "$OCA/Snapmaker" machine "Snapmaker U1 (0.4 nozzle)"
parents r2 "$OCA/Snapmaker" process "0.20 Standard @Snapmaker U1 (0.4 nozzle)"
parents r2 "$OCA/OrcaFilamentLibrary" filament "Generic PLA @System"
case_done "$before" "row 2: the same three flags read the shipped Snapmaker files over their parents"

before=$FAILED
run r3 "$B" cube.stl --slice 1 --config "$BBL/machine/Bambu Lab X1 Carbon 0.4 nozzle.json" --outputdir r3/out
[ "$(rc r3)" = 0 ] || { echo "   $(why r3)"; fail "row 3: exit $(rc r3)"; }
parents r3 "$BBL" machine "Bambu Lab X1 Carbon 0.4 nozzle"
case_done "$before" "row 3: --config reads the shipped machine file over its parents"

before=$FAILED
run r4b "$B" cube.stl --slice 1 \
    --load-settings "$BBL/machine/Bambu Lab X1 Carbon 0.4 nozzle.json;$BBL/process/0.20mm Standard @BBL X1C.json" \
    --load-filaments "$BBL/filament/Bambu PLA Basic @BBL X1C.json" --outputdir r4b/out
[ "$(rc r4b)" = 0 ] || { echo "   $(why r4b)"; fail "row 4 (bambu): exit $(rc r4b)"; }
parents r4b "$BBL" machine "Bambu Lab X1 Carbon 0.4 nozzle"
parents r4b "$BBL" process "0.20mm Standard @BBL X1C"
parents r4b "$BBL" filament "Bambu PLA Basic @BBL X1C"
run r4o "$O" cube.stl --slice 1 \
    --load-settings "$OCA/Snapmaker/machine/Snapmaker U1 (0.4 nozzle).json;$OCA/Snapmaker/process/0.20 Standard @Snapmaker U1 (0.4 nozzle).json" \
    --load-filaments "$OCA/OrcaFilamentLibrary/filament/Generic PLA @System.json" --outputdir r4o/out
[ "$(rc r4o)" = 0 ] || { echo "   $(why r4o)"; fail "row 4 (orca): exit $(rc r4o)"; }
parents r4o "$OCA/Snapmaker" machine "Snapmaker U1 (0.4 nozzle)"
parents r4o "$OCA/Snapmaker" process "0.20 Standard @Snapmaker U1 (0.4 nozzle)"
parents r4o "$OCA/OrcaFilamentLibrary" filament "Generic PLA @System"
case_done "$before" "row 4: --load-settings and --load-filaments read the shipped files over their parents"

# ── rows 5-8: a printer change onto a project ──────────────────────────────
before=$FAILED
run r5 "$B" a1m.3mf --slice 1 \
    --load-settings "$BBL/machine/Bambu Lab X1 Carbon 0.4 nozzle.json;$BBL/process/0.20mm Standard @BBL X1C.json" \
    --load-filaments "$BBL/filament/Bambu PLA Basic @BBL X1C.json" --outputdir r5/out
[ "$(rc r5)" = 0 ] || { echo "   $(why r5)"; fail "row 5: exit $(rc r5)"; }
parents r5 "$BBL" machine "Bambu Lab X1 Carbon 0.4 nozzle"
expect r5 printer_settings_id "Bambu Lab X1 Carbon 0.4 nozzle" "row 5"
run r6 "$O" a1m.3mf --slice 1 \
    --load-settings "$OCA/Snapmaker/machine/Snapmaker U1 (0.4 nozzle).json;$OCA/Snapmaker/process/0.20 Standard @Snapmaker U1 (0.4 nozzle).json" \
    --load-filaments "$OCA/OrcaFilamentLibrary/filament/Generic PLA @System.json" --outputdir r6/out
[ "$(rc r6)" = 0 ] || { echo "   $(why r6)"; fail "row 6: exit $(rc r6)"; }
parents r6 "$OCA/Snapmaker" machine "Snapmaker U1 (0.4 nozzle)"
expect r6 gcode_flavor "klipper" "row 6"
run r7 "$O" a1m.3mf --slice 1 \
    --load-settings "$OCA/Snapmaker/machine/Snapmaker U1 (0.4 nozzle).json" --outputdir r7/out
[ "$(rc r7)" = 0 ] || { echo "   $(why r7)"; fail "row 7: exit $(rc r7)"; }
parents r7 "$OCA/Snapmaker" machine "Snapmaker U1 (0.4 nozzle)"
run r8 "$O" a1m.3mf --slice 1 \
    --load-settings "$OCA/BBL/machine/Bambu Lab X1 Carbon 0.4 nozzle.json" --outputdir r8/out
[ "$(rc r8)" = 0 ] || { echo "   $(why r8)"; fail "row 8: exit $(rc r8)"; }
parents r8 "$OCA/BBL" machine "Bambu Lab X1 Carbon 0.4 nozzle"
expect r8 printable_area "0x0,256x0,256x256,0x256" "row 8"
case_done "$before" "rows 5-8: a project moves to the file's printer, with its parents' values"

# ── rows 9-11: a nozzle change with the shipped file ───────────────────────
before=$FAILED
# The oracle: the same printer named, which resolves it from the tree.
run n06 "$B" cube.stl --slice 1 --printer-preset "Bambu Lab A1 mini 0.6 nozzle" --outputdir n06/out
[ "$(rc n06)" = 0 ] || { echo "   $(why n06)"; fail "rows 9-11 oracle: exit $(rc n06)"; }
run r9 "$B" a1m.3mf --slice 1 --load-settings "$BBL/machine/Bambu Lab A1 mini 0.6 nozzle.json" --outputdir r9/out
[ "$(rc r9)" = 0 ] || { echo "   $(why r9)"; fail "row 9: exit $(rc r9)"; }
parents r9 "$BBL" machine "Bambu Lab A1 mini 0.6 nozzle"
expect r9 nozzle_diameter "$(header n06 nozzle_diameter)" "row 9"
expect r9 line_width "$(header n06 line_width)" "row 9"
run nx06 "$B" cube.stl --slice 1 --printer-preset "Bambu Lab X1 Carbon 0.6 nozzle" --outputdir nx06/out
[ "$(rc nx06)" = 0 ] || { echo "   $(why nx06)"; fail "rows 9-11 oracle: exit $(rc nx06)"; }
run r10 "$B" a1m.3mf --slice 1 --load-settings "$BBL/machine/Bambu Lab X1 Carbon 0.6 nozzle.json" --outputdir r10/out
[ "$(rc r10)" = 0 ] || { echo "   $(why r10)"; fail "row 10: exit $(rc r10)"; }
parents r10 "$BBL" machine "Bambu Lab X1 Carbon 0.6 nozzle"
expect r10 printable_area "$(header nx06 printable_area)" "row 10"
run nh06 "$B" cube.stl --slice 1 --printer-preset "Bambu Lab H2D 0.6 nozzle" --outputdir nh06/out
[ "$(rc nh06)" = 0 ] || { echo "   $(why nh06)"; fail "rows 9-11 oracle: exit $(rc nh06)"; }
run r11 "$B" h2d.3mf --slice 1 --load-settings "$BBL/machine/Bambu Lab H2D 0.6 nozzle.json" --outputdir r11/out
[ "$(rc r11)" = 0 ] || { echo "   $(why r11)"; fail "row 11: exit $(rc r11)"; }
parents r11 "$BBL" machine "Bambu Lab H2D 0.6 nozzle"
expect r11 nozzle_flush_dataset "$(header nh06 nozzle_flush_dataset)" "row 11"
expect r11 machine_start_gcode "$(header nh06 machine_start_gcode)" "row 11"
case_done "$before" "rows 9-11: a nozzle change slices as the printer named by preset does"

# ── rows 12-17: the process and filament files ─────────────────────────────
before=$FAILED
run r12 "$B" a1m.3mf --slice 1 --load-settings "$BBL/process/0.20mm Standard @BBL A1M.json" --outputdir r12/out
[ "$(rc r12)" = 0 ] || { echo "   $(why r12)"; fail "row 12: exit $(rc r12)"; }
parents r12 "$BBL" process "0.20mm Standard @BBL A1M"
for name in r13 r13d; do
    extra=""
    [ "$name" = r13d ] && extra="--load-defaultfila"
    run "$name" "$B" a1m.3mf --slice 1 $extra --load-filaments "$BBL/filament/Bambu PETG HF @BBL A1M.json" --outputdir "$name/out"
    [ "$(rc "$name")" = 0 ] || { echo "   $(why "$name")"; fail "row 13 ($name): exit $(rc "$name")"; }
    parents "$name" "$BBL" filament "Bambu PETG HF @BBL A1M"
    expect "$name" filament_type "PETG" "row 13 ($name)"
done
run r14 "$O" a1m.3mf --slice 1 \
    --load-filaments "$OCA/OrcaFilamentLibrary/filament/Generic PLA @System.json" --outputdir r14/out
[ "$(rc r14)" = 0 ] || { echo "   $(why r14)"; fail "row 14: exit $(rc r14)"; }
parents r14 "$OCA/OrcaFilamentLibrary" filament "Generic PLA @System"
case_done "$before" "rows 12-14: a process or filament file alone is read over its parents"

before=$FAILED
run r15 "$B" a1m.3mf --slice 1 --load-filaments user-petg-typed.json --outputdir r15/out
[ "$(rc r15)" = 0 ] || { echo "   $(why r15)"; fail "row 15: exit $(rc r15)"; }
parents r15 "$BBL" filament user-petg-typed.json
expect r15 filament_type "PETG" "row 15"
expect r15 nozzle_temperature "250" "row 15"
# BAD 4: the same file without the "type" the desktop leaves out.
run r15u "$B" a1m.3mf --slice 1 --load-filaments user-pla-untyped.json --outputdir r15u/out
if [ "$(rc r15u)" = 0 ]; then
    parents r15u "$BBL" filament user-pla-untyped.json
    expect r15u nozzle_temperature "225" "BAD 4"
else
    echo "   $(why r15u)"; fail "BAD 4: the desktop's own user preset without a type was refused"
fi
# Row 16: a user printer preset over the X1 Carbon.
run r16 "$B" a1m.3mf --slice 1 --load-settings user-x1c-typed.json --outputdir r16/out
[ "$(rc r16)" = 0 ] || { echo "   $(why r16)"; fail "row 16: exit $(rc r16)"; }
parents r16 "$BBL" machine user-x1c-typed.json
expect r16 printable_area "0x0,256x0,256x256,0x256" "row 16"
case_done "$before" "rows 15-16, BAD 4: the desktop's own user presets load over their system parent"

# ── BAD 5: the desktop's "Export current configs" form ─────────────────────
before=$FAILED
run export "$B" cube.stl --slice 1 --printer-preset "Bambu Lab X1 Carbon 0.4 nozzle" \
    --export-settings export-x1c.json --outputdir export/out
[ "$(rc export)" = 0 ] || { echo "   $(why export)"; fail "the export fixture: exit $(rc export)"; }
python3 - <<'PY'
import json
d = json.load(open("export-x1c.json"))
d.pop("type", None)      # the desktop export states no "type" ...
d["from"] = ""           # ... and an empty "from"
d["name"] = "Bambu Lab X1 Carbon 0.4 nozzle"
json.dump(d, open("export-x1c-desktop.json", "w"), indent=1)
PY
run rbad5 "$B" a1m.3mf --slice 1 --load-settings export-x1c-desktop.json --outputdir rbad5/out
if [ "$(rc rbad5)" = 0 ]; then
    # The export is flattened (no "inherits"): there is no parent chain to
    # check, only that it loaded and took effect.
    expect rbad5 printable_area "0x0,256x0,256x256,0x256" "BAD 5"
    expect rbad5 printer_settings_id "Bambu Lab X1 Carbon 0.4 nozzle" "BAD 5"
else
    echo "   $(why rbad5)"; fail "BAD 5: the desktop's own export was refused"
fi
case_done "$before" "BAD 5: the desktop's exported settings file loads"

# ── BAD 6: --downward-settings with the shipped partial files ──────────────
before=$FAILED
unset x
run rbad6 "$B" a1m.3mf --slice 1 --downward-check \
    --downward-settings "$BBL/machine/Bambu Lab X1 Carbon 0.4 nozzle.json;$BBL/machine/Bambu Lab A1 mini 0.6 nozzle.json" \
    --outputdir rbad6/out
if [ "$(rc rbad6)" = 0 ] && [ -s rbad6/out/result.json ]; then
    python3 - rbad6/out/result.json <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))
assert "downward_compatible_machine" in d, sorted(d)
for want in ("Bambu Lab X1 Carbon 0.4 nozzle", "Bambu Lab A1 mini 0.6 nozzle"):
    assert want in d["downward_compatible_machine"], (want, d["downward_compatible_machine"])
PY
    [ $? = 0 ] || fail "BAD 6: the two machine files were not both read"
else
    echo "   $(why rbad6)"; fail "BAD 6: exit $(rc rbad6), result.json $( [ -s rbad6/out/result.json ] && echo present || echo missing)"
fi
case_done "$before" "BAD 6: --downward-settings reads the shipped partial machine files, no crash"

# ── BAD 1 and BAD 2: the two refusals a real file must not get ─────────────
before=$FAILED
run rbad1 "$O" a1m.3mf --slice 1 --printer-preset "Snapmaker U1 (0.4 nozzle)" --outputdir rbad1/out
if [ "$(rc rbad1)" = 0 ]; then
    expect rbad1 printer_settings_id "Snapmaker U1 (0.4 nozzle)" "BAD 1"
    parents rbad1 "$OCA/Snapmaker" machine "Snapmaker U1 (0.4 nozzle)"
else
    echo "   $(why rbad1)"; fail "BAD 1: --printer-preset on a project 3MF was refused"
fi
run rbad2 "$B" a1m.3mf --slice 1 --load-settings "$BBL/machine/Bambu Lab X1 Carbon 0.4 nozzle.json" --outputdir rbad2/out
if [ "$(rc rbad2)" = 0 ]; then
    expect rbad2 printer_settings_id "Bambu Lab X1 Carbon 0.4 nozzle" "BAD 2"
    parents rbad2 "$BBL" machine "Bambu Lab X1 Carbon 0.4 nozzle"
else
    echo "   $(why rbad2)"; fail "BAD 2: a machine-only switch was refused"
fi
case_done "$before" "BAD 1-2: a printer switch by name, and a machine-only switch, both slice"

# SILENT-WRONG 17 (the legacy --filament flag with a file of another vendor) is
# NOT in this script: its root is not the inherits chain but the vector growth
# of the legacy apply (a one-value list onto a four-nozzle config gave
# nozzle_temperature 220,220,0,0) and the missing compatibility check. See the
# lane report.
echo
echo "test-inherits-inputs: $PASS passed, $FAILED failed"
[ "$FAILED" = 0 ]
