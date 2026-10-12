#!/usr/bin/env bash
# test-product-invariants.sh: contracts the product reads that a G-code compare
# cannot see, on both engines. Each passes on ba0dcfb and must keep passing.
#   I1 --layout-plan: exit 4 (UNFITTABLE) with the candidate as the one JSON
#      document on stdout and the error JSON on stderr.
#   I2 --layout-plan: exit 5 (CANCELLED) on SIGINT while it waits for its
#      input, nothing on stdout (Linux and macOS: needs a FIFO).
#   I3 --layout-plan: exit 0 with stdout one JSON document (the JSON is the
#      last thing printed, after any diagnostics).
#   I6 --layout-plan schemaVersion 3: "beds": "all" places items on every bed
#      it fills (bed_idx, beds_used), clear of the primeTower on every bed;
#      v2 keeps the single-bed answer; v3 fields in a v2 request are refused.
#   I4 the product's default call on a by-object plate whose objects are too
#      close: a nonzero exit and the stderr line
#      "Validation error: ... collisions may be caused".
#
# Usage: tests/test-product-invariants.sh SLICER_CLI SLICER_CLI_ORCASLICER
set -uo pipefail

B="${1:?usage: $0 slicer_cli slicer_cli-orcaslicer}"
O="${2:?usage: $0 slicer_cli slicer_cli-orcaslicer}"
B="$(cd "$(dirname "$B")" && pwd -P)/$(basename "$B")"
O="$(cd "$(dirname "$O")" && pwd -P)/$(basename "$O")"
# The package's resources: beside the binary or one level up.
RES="$(cd "$(dirname "$B")" && pwd -P)/resources"
[ -d "$RES/profiles" ] || RES="$(cd "$(dirname "$B")/.." && pwd -P)/resources"
WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT
cd "$WORKDIR"

X1C="Bambu Lab X1 Carbon 0.4 nozzle"
FAILED=0
pass() { echo "PASS: $*"; }
fail() { echo "FAIL: $*"; FAILED=1; }
run() {
    local name=$1; shift
    mkdir -p "$name"
    "$@" > "$name/stdout" 2> "$name/stderr" < /dev/null
    echo $? > "$name/rc"
}
rc() { cat "$1/rc"; }
# native PATH: the path as the engine's own loader must see it. Git Bash
# converts MSYS paths (/d/..., /tmp/...) in command-line arguments only, never
# inside a file, so a path written into a request JSON has to be converted by
# hand (cygpath -m gives D:/..., which the Windows binary opens; the argument
# form was already proven by the orcaswitch checks). Elsewhere: unchanged.
native() { if command -v cygpath > /dev/null 2>&1; then cygpath -m "$1"; else printf '%s' "$1"; fi; }
one_json() { python3 -c 'import json,sys; json.loads(open(sys.argv[1]).read())' "$1" 2>/dev/null; }

python3 - <<'PY'
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
box("big.stl", 300, 300, 10)
box("tall.stl", 20, 20, 60)
import json
json.dump({"plates": [{"plate_name": "p", "need_arrange": True, "objects": [
    {"path": "tall.stl", "count": 2, "filaments": [1, 1]}]}]}, open("close.json", "w"))
PY

for e in bambu orca; do
    bin=$B; prof="$RES/profiles"
    [ $e = orca ] && { bin=$O; prof="$RES/profiles-orca"; }
    machine="BBL/machine/$X1C.json"
    problem() {  # problem FILE MODEL
        cat > "$1" <<EOF
{
  "schemaVersion": 1,
  "engine": "$e",
  "profilesDir": "$(native "$prof")",
  "profiles": { "machine": "$machine" },
  "spacing": { "minObjectDistanceMm": 10.0 },
  "models": [ { "id": "a", "path": "$(native "$WORKDIR/$2")" } ]
}
EOF
    }

    # I3: a cube fits: exit 0, stdout is one JSON document.
    problem fit-$e.json cube.stl
    run i3-$e "$bin" --layout-plan --input fit-$e.json
    if [ "$(rc i3-$e)" = 0 ] && one_json i3-$e/stdout; then pass "I3 $e: --layout-plan exit 0, stdout one JSON document"
    else fail "I3 $e: exit $(rc i3-$e), stdout one JSON: $(one_json i3-$e/stdout && echo yes || echo no)"; fi

    # I1: a 300 mm box does not fit the X1 Carbon: exit 4, the candidate on
    # stdout, UNFITTABLE naming the model on stderr.
    problem big-$e.json big.stl
    run i1-$e "$bin" --layout-plan --input big-$e.json
    if [ "$(rc i1-$e)" = 4 ] && one_json i1-$e/stdout && python3 - i1-$e/stderr <<'PY'
import json, sys
lines = [l for l in open(sys.argv[1]) if l.strip().startswith("{")]
err = json.loads(lines[-1])["error"]
assert err["code"] == "UNFITTABLE" and "a" in err.get("objectIds", err.get("object_ids", [])), err
PY
    then pass "I1 $e: --layout-plan exit 4, candidate JSON on stdout, UNFITTABLE on stderr"
    else fail "I1 $e: exit $(rc i1-$e) (want 4); stderr: $(tail -c 300 i1-$e/stderr)"; fi

    # I5: every spelling the parser takes for the flag (one dash, an inline
    # =value) gives the same streams: stderr holds JSON documents only (the
    # resources line is left out), the last one UNFITTABLE.
    for spelling in -layout-plan --layout-plan=1; do
        n=i5$(printf '%s' "$spelling" | tr -c 'a-z0-9' '_')-$e
        run $n "$bin" "$spelling" --input big-$e.json
        if [ "$(rc $n)" = 4 ] && python3 - $n/stderr <<'PY'
import json, sys
lines = [l for l in open(sys.argv[1]) if l.strip()]
assert lines, "stderr is empty"
docs = [json.loads(l) for l in lines]
assert docs[-1]["error"]["code"] == "UNFITTABLE", docs[-1]
PY
        then pass "I5 $e: $spelling gives stderr of JSON documents only, UNFITTABLE"
        else fail "I5 $e: $spelling exit $(rc $n) (want 4); stderr: $(head -c 300 $n/stderr)"; fi
    done

    # I6 (schemaVersion 3): six 110 mm squares need two X1 Carbon beds.
    #  - "beds": "first" (today's single-bed answer, v2) leaves the ones past
    #    bed 0 unplaced: exit 4, UNFITTABLE.
    #  - "beds": "all" places every one, on bed 0 or 1 (beds_used 2), each fully
    #    on its bed, and none overlapping the prime tower given in the request,
    #    which stands on every bed.
    #  - "beds" in a v2 request is refused; capabilities say v3, cross_plate
    #    and prime_tower.
    six() {  # six FILE SCHEMA EXTRA
        python3 - "$1" "$2" "$3" "$e" "$(native "$prof")" "$machine" "$(native "$WORKDIR/sq110.stl")" <<'PY'
import json, sys
path, schema, extra, engine, prof, machine, model = sys.argv[1:]
req = {"schemaVersion": int(schema), "engine": engine, "profilesDir": prof, "profiles": {"machine": machine},
       "spacing": {"minObjectDistanceMm": 10.0, "allowRotations": False},
       "models": [{"id": "s%d" % i, "path": model} for i in range(6)]}
req.update(json.loads(extra))
json.dump(req, open(path, "w"))
PY
    }
    [ -f sq110.stl ] || python3 -c '
v=[(x,y,z) for z in (0,10) for y in (0,110) for x in (0,110)]
f=[(0,2,1),(1,2,3),(4,5,6),(5,7,6),(0,1,4),(1,5,4),(2,6,3),(3,6,7),(0,4,2),(2,4,6),(1,3,5),(3,7,5)]
o=open("sq110.stl","w"); o.write("solid t\n")
for a,b,c in f:
    o.write("facet normal 0 0 0\nouter loop\n")
    for i in (a,b,c): o.write("vertex %g %g %g\n" % v[i])
    o.write("endloop\nendfacet\n")
o.write("endsolid t\n")'
    TOWER='{"x_mm": 15, "y_mm": 206.4, "width_mm": 35, "depth_mm": 37.4, "brim_mm": 3}'
    six i6first-$e.json 2 '{}'
    run i6first-$e "$bin" --layout-plan --input i6first-$e.json
    if [ "$(rc i6first-$e)" = 4 ] && grep -q '"UNFITTABLE"' i6first-$e/stderr; then
        pass "I6 $e: v2 (one bed) leaves the squares past bed 0 unplaced: exit 4, UNFITTABLE"
    else fail "I6 $e: v2 exit $(rc i6first-$e) (want 4); stderr: $(tail -c 200 i6first-$e/stderr)"; fi

    six i6all-$e.json 3 "{\"beds\": \"all\", \"primeTower\": $TOWER}"
    run i6all-$e "$bin" --layout-plan --input i6all-$e.json
    if [ "$(rc i6all-$e)" = 0 ] && python3 - i6all-$e/stdout <<'PY'
import json, sys
doc = json.loads(open(sys.argv[1]).read())
pl = doc["placements"]
# Every square placed, on beds 0..beds_used-1 with none skipped. How many
# beds is each engine's own packing: two at least (four 110 mm squares do not
# fit one bed beside the tower).
beds = {p["bed_idx"] for p in pl}
assert len(pl) == 6 and all(p["bed_idx"] >= 0 for p in pl), pl
assert doc["beds_used"] >= 2 and beds == set(range(doc["beds_used"])), doc
# The tower box grown by its brim: x 12..53, y 203.4..246.8. A square's box
# (centre +- 55) must not overlap it, on any bed; and each square stays on
# the 256 x 256 bed.
# The X1 Carbon's fixed areas, on every bed: its bed_exclude_area corner
# (0,0)-(18,28) and, as it scans its first layer, the calibration strip
# (18,0)-(240,15).
fixed = [(12, 203.4, 53, 246.8, "tower"), (0, 0, 18, 28, "exclusion corner"), (18, 0, 240, 15, "calibration strip")]
for p in pl:
    x0, y0, x1, y1 = p["x_mm"] - 55, p["y_mm"] - 55, p["x_mm"] + 55, p["y_mm"] + 55
    assert x0 >= -0.01 and y0 >= -0.01 and x1 <= 256.01 and y1 <= 256.01, p
    for fx0, fy0, fx1, fy1, what in fixed:
        assert x1 <= fx0 or x0 >= fx1 or y1 <= fy0 or y0 >= fy1, ("on the " + what, p)
PY
    then pass "I6 $e: v3 beds all places all six across beds_used beds (none skipped), clear of the tower, exclusion corner and calibration strip on every bed"
    else fail "I6 $e: v3 exit $(rc i6all-$e) (want 0); stdout: $(head -c 400 i6all-$e/stdout); stderr: $(tail -c 200 i6all-$e/stderr)"; fi

    # The tower stands on EVERY bed: a tower across the back of the bed
    # (x 8-248, y 108-248) leaves one row of 60 mm squares per bed in front,
    # so nine need three beds or more. With the tower on bed 0 only, bed 1
    # would take the other six and two beds would do.
    [ -f sq60.stl ] || python3 -c '
v=[(x,y,z) for z in (0,10) for y in (0,60) for x in (0,60)]
f=[(0,2,1),(1,2,3),(4,5,6),(5,7,6),(0,1,4),(1,5,4),(2,6,3),(3,6,7),(0,4,2),(2,4,6),(1,3,5),(3,7,5)]
o=open("sq60.stl","w"); o.write("solid t\n")
for a,b,c in f:
    o.write("facet normal 0 0 0\nouter loop\n")
    for i in (a,b,c): o.write("vertex %g %g %g\n" % v[i])
    o.write("endloop\nendfacet\n")
o.write("endsolid t\n")'
    python3 - i6wall-$e.json "$e" "$(native "$prof")" "$machine" "$(native "$WORKDIR/sq60.stl")" <<'PY'
import json, sys
path, engine, prof, machine, model = sys.argv[1:]
json.dump({"schemaVersion": 3, "engine": engine, "profilesDir": prof, "profiles": {"machine": machine},
           "spacing": {"minObjectDistanceMm": 10.0, "allowRotations": False},
           "models": [{"id": "w%d" % i, "path": model} for i in range(9)],
           "beds": "all", "primeTower": {"x_mm": 8, "y_mm": 108, "width_mm": 240, "depth_mm": 140}},
          open(path, "w"))
PY
    run i6wall-$e "$bin" --layout-plan --input i6wall-$e.json
    if [ "$(rc i6wall-$e)" = 0 ] && python3 - i6wall-$e/stdout <<'PY'
import json, sys
doc = json.loads(open(sys.argv[1]).read())
pl = doc["placements"]
assert len(pl) == 9 and all(p["bed_idx"] >= 0 for p in pl), pl
assert doc["beds_used"] >= 3, doc
for p in pl:
    assert p["y_mm"] + 30 <= 108.01, ("into the tower", p)
PY
    then pass "I6 $e: the tower stands on every bed: nine squares in front of it take $(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["beds_used"])' i6wall-$e/stdout) beds"
    else fail "I6 $e: tower on every bed: exit $(rc i6wall-$e); stdout: $(head -c 400 i6wall-$e/stdout); stderr: $(tail -c 200 i6wall-$e/stderr)"; fi

    # A locked square standing on the tower is UNFITTABLE: two fixed items are
    # never moved apart, so v3 reports it rather than answer an overlap. The
    # same request without the tower places it.
    for tw in with without; do
        python3 - i6lock$tw-$e.json "$e" "$(native "$prof")" "$machine" "$(native "$WORKDIR/sq60.stl")" "$tw" <<'PY'
import json, sys
path, engine, prof, machine, model, tw = sys.argv[1:]
req = {"schemaVersion": 3, "engine": engine, "profilesDir": prof, "profiles": {"machine": machine},
       "models": [{"id": "lk", "path": model, "locked": True, "transform": {"x_mm": 40, "y_mm": 185}}], "beds": "all"}
if tw == "with":
    req["primeTower"] = {"x_mm": 15, "y_mm": 206.4, "width_mm": 35, "depth_mm": 37.4, "brim_mm": 3}
json.dump(req, open(path, "w"))
PY
        run i6lock$tw-$e "$bin" --layout-plan --input i6lock$tw-$e.json
    done
    if [ "$(rc i6lockwith-$e)" = 4 ] && grep -q '"UNFITTABLE"' i6lockwith-$e/stderr && grep -q '"lk"' i6lockwith-$e/stderr \
       && [ "$(rc i6lockwithout-$e)" = 0 ]; then
        pass "I6 $e: a locked square on the prime tower is UNFITTABLE (placed when there is no tower)"
    else fail "I6 $e: locked on tower exit $(rc i6lockwith-$e) (want 4), without tower exit $(rc i6lockwithout-$e) (want 0); stderr: $(tail -c 200 i6lockwith-$e/stderr)"; fi

    # The same check counts each item's inflation (half the 10 mm spacing),
    # as the nester does: a tower 3 mm right of the locked square overlaps it
    # once both are grown; 25 mm away it does not. The square's place on the
    # bed is read from the run without a tower.
    for gap in 3 25; do
        python3 - i6lockgap$gap-$e.json "$e" "$(native "$prof")" "$machine" "$(native "$WORKDIR/sq60.stl")" i6lockwithout-$e/stdout "$gap" <<'PY'
import json, sys
path, engine, prof, machine, model, placed, gap = sys.argv[1:]
p = json.loads(open(placed).read())["placements"][0]
right = p["x_mm"] + 30 + float(gap)
json.dump({"schemaVersion": 3, "engine": engine, "profilesDir": prof, "profiles": {"machine": machine},
           "spacing": {"minObjectDistanceMm": 10.0},
           "models": [{"id": "lk", "path": model, "locked": True, "transform": {"x_mm": 40, "y_mm": 185}}], "beds": "all",
           "primeTower": {"x_mm": right, "y_mm": p["y_mm"] - 30, "width_mm": 20, "depth_mm": 60}}, open(path, "w"))
PY
        run i6lockgap$gap-$e "$bin" --layout-plan --input i6lockgap$gap-$e.json
    done
    if [ "$(rc i6lockgap3-$e)" = 4 ] && grep -q '"lk"' i6lockgap3-$e/stderr && [ "$(rc i6lockgap25-$e)" = 0 ]; then
        pass "I6 $e: a locked square 3 mm from the tower is UNFITTABLE once both are inflated (placed at 25 mm)"
    else fail "I6 $e: inflated locked check: 3 mm exit $(rc i6lockgap3-$e) (want 4), 25 mm exit $(rc i6lockgap25-$e) (want 0); stderr: $(tail -c 200 i6lockgap3-$e/stderr)"; fi

    # A tower that is not on the bed is refused, not reserved off the bed.
    python3 - i6off-$e.json "$e" "$(native "$prof")" "$machine" "$(native "$WORKDIR/sq60.stl")" <<'PY'
import json, sys
path, engine, prof, machine, model = sys.argv[1:]
json.dump({"schemaVersion": 3, "engine": engine, "profilesDir": prof, "profiles": {"machine": machine},
           "models": [{"id": "a", "path": model}], "beds": "all",
           "primeTower": {"x_mm": 165, "y_mm": 250, "width_mm": 35, "depth_mm": 37}}, open(path, "w"))
PY
    run i6off-$e "$bin" --layout-plan --input i6off-$e.json
    if [ "$(rc i6off-$e)" = 3 ] && grep -q "must lie inside the printer's printable_area" i6off-$e/stderr; then
        pass "I6 $e: a prime tower off the bed is refused (exit 3)"
    else fail "I6 $e: off-bed tower exit $(rc i6off-$e) (want 3); stderr: $(tail -c 200 i6off-$e/stderr)"; fi

    six i6old-$e.json 2 '{"beds": "all"}'
    run i6old-$e "$bin" --layout-plan --input i6old-$e.json
    if [ "$(rc i6old-$e)" = 3 ] && grep -q "require schemaVersion 3" i6old-$e/stderr; then
        pass "I6 $e: beds in a v2 request is refused (exit 3)"
    else fail "I6 $e: v2 with beds exit $(rc i6old-$e) (want 3); stderr: $(tail -c 200 i6old-$e/stderr)"; fi

    run i6caps-$e "$bin" layout capabilities --json
    if python3 - i6caps-$e/stdout <<'PY'
import json, sys
doc = json.loads(open(sys.argv[1]).read())
c = doc["capabilities"]
assert doc["max_schema_version"] == 3 and c["cross_plate"] is True and c["prime_tower"] is True, doc
PY
    then pass "I6 $e: capabilities: max_schema_version 3, cross_plate, prime_tower"
    else fail "I6 $e: capabilities: $(head -c 300 i6caps-$e/stdout)"; fi

    # I2: SIGINT while it waits for a writer on its --input FIFO: exit 5,
    # CANCELLED on stderr, nothing on stdout.
    if [ "$(uname -s)" = Linux ] || [ "$(uname -s)" = Darwin ]; then
        rm -f in-$e.fifo; mkfifo in-$e.fifo; mkdir -p i2-$e
        "$bin" --layout-plan --input in-$e.fifo > i2-$e/stdout 2> i2-$e/stderr < /dev/null &
        pid=$!
        sleep 2
        kill -INT $pid
        wait $pid; code=$?
        if [ "$code" = 5 ] && [ ! -s i2-$e/stdout ] && grep -q '"CANCELLED"' i2-$e/stderr; then
            pass "I2 $e: --layout-plan exit 5 on SIGINT, CANCELLED on stderr, stdout empty"
        else fail "I2 $e: exit $code (want 5); stdout $(wc -c < i2-$e/stdout) bytes; stderr: $(tail -c 200 i2-$e/stderr)"; fi
    else
        echo "SKIP: I2 $e: needs a FIFO (Linux, macOS)"
    fi

    # I4: two 60 mm tall boxes arranged side by side for a by-layer print, then
    # the project set to print by object (the gap is far below the extruder
    # clearance), sliced by the product's default call: refused with the
    # engine's collision sentence on stderr (Print::validate, Print.cpp 700 at
    # 5873b5f / 691 at 31f6803).
    run i4p-$e "$bin" --load-assemble-list close.json --slice 1 --printer-preset "$X1C" \
        --outputdir i4p-$e/out --export-3mf close.3mf
    if [ ! -s i4p-$e/out/close.3mf ]; then fail "I4 $e: fixture project: exit $(rc i4p-$e)"; continue; fi
    python3 - i4p-$e/out/close.3mf close-$e.3mf <<'PY'
import json, sys, zipfile
src, dst = sys.argv[1], sys.argv[2]
with zipfile.ZipFile(src) as zin, zipfile.ZipFile(dst, "w", zipfile.ZIP_DEFLATED) as zout:
    for item in zin.infolist():
        data = zin.read(item.filename)
        if item.filename == "Metadata/project_settings.config":
            d = json.loads(data); d["print_sequence"] = "by object"; data = json.dumps(d, indent=4).encode()
        elif item.filename.startswith("Metadata/plate_") and item.filename.endswith((".gcode", ".md5")):
            continue
        zout.writestr(item, data)
PY
    run i4-$e "$bin" close-$e.3mf --plate 1 -o i4-$e.gcode
    if [ "$(rc i4-$e)" != 0 ] && grep -q '^Validation error: .*collisions may be caused' i4-$e/stderr; then
        pass "I4 $e: by-object collision exits $(rc i4-$e) with 'Validation error: ... collisions may be caused' on stderr"
    else fail "I4 $e: exit $(rc i4-$e); stderr: $(grep -m1 -i 'validation\|error' i4-$e/stderr | cut -c1-200)"; fi
done

exit $FAILED
