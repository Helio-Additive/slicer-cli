#!/usr/bin/env bash
# test-product-invariants.sh: contracts the product reads that a G-code compare
# cannot see, on both engines. Each passes on ba0dcfb and must keep passing.
#   I1 --layout-plan: exit 4 (UNFITTABLE) with the candidate as the one JSON
#      document on stdout and the error JSON on stderr.
#   I2 --layout-plan: exit 5 (CANCELLED) on SIGINT while it waits for its
#      input, nothing on stdout (Linux and macOS: needs a FIFO).
#   I3 --layout-plan: exit 0 with stdout one JSON document (the JSON is the
#      last thing printed, after any diagnostics).
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
