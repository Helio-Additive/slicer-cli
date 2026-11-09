#!/usr/bin/env bash
# --simplify is the desktop Simplify tool, headless, on every engine build.
# Usage: tests/test_simplify.sh --engine bambu|orca|snapmaker BIN
#    or: tests/test_simplify.sh BAMBU_BIN [ORCA_BIN [SNAPMAKER_BIN]]
#
# The meshes are generated here (UV spheres with a known triangle count), the
# 3MF is the calibration fixture the other tests slice. Nothing is hand-made.
set -euo pipefail

declare -a BINARIES LABELS
if [ "$#" = 3 ] && [ "$1" = "--engine" ]; then
    case "$2" in bambu|orca|snapmaker) ;; *) echo "invalid engine: $2" >&2; exit 64 ;; esac
    BINARIES=("$3")
    LABELS=("$2")
elif [ "$#" -ge 1 ] && [ "$#" -le 3 ]; then
    BINARIES=("$@")
    LABELS=(bambu orca snapmaker)
else
    echo "usage: $0 --engine bambu|orca|snapmaker BIN" >&2
    echo "   or: $0 BAMBU_BIN [ORCA_BIN [SNAPMAKER_BIN]]" >&2
    exit 64
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
WORKDIR="$(mktemp -d "${TMPDIR:-/tmp}/simplify.XXXXXX")"
trap 'rm -rf "$WORKDIR"' EXIT
A1M="Bambu Lab A1 mini 0.4 nozzle"

# sphere <path> <slices> <stacks>: a binary STL UV sphere, radius 20 mm, with
# exactly 2 * slices * (stacks - 1) triangles. Prints that count.
sphere() {
    python3 - "$1" "$2" "$3" <<'PY'
import math, struct, sys
path, L, S = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
R = 20.0
ring = lambda i: [(R*math.sin(math.pi*i/S)*math.cos(2*math.pi*j/L), R*math.sin(math.pi*i/S)*math.sin(2*math.pi*j/L), R*math.cos(math.pi*i/S)) for j in range(L)]
top, bottom = (0.0, 0.0, R), (0.0, 0.0, -R)
rings = [ring(i) for i in range(1, S)]
tris = []
for j in range(L):
    tris.append((top, rings[0][j], rings[0][(j+1) % L]))
for i in range(S - 2):
    a, b = rings[i], rings[i+1]
    for j in range(L):
        k = (j+1) % L
        tris.append((a[j], b[j], b[k]))
        tris.append((a[j], b[k], a[k]))
for j in range(L):
    tris.append((bottom, rings[-1][(j+1) % L], rings[-1][j]))
with open(path, "wb") as f:
    f.write(b"\0" * 80 + struct.pack("<I", len(tris)))
    for t in tris:
        f.write(struct.pack("<3f", 0.0, 0.0, 0.0))
        for p in t:
            f.write(struct.pack("<3f", *p))
        f.write(b"\0\0")
print(len(tris))
PY
}

# stl_triangles <path>: the triangle count of a binary or ASCII STL.
stl_triangles() {
    python3 - "$1" <<'PY'
import struct, sys
data = open(sys.argv[1], "rb").read()
if data[:5] == b"solid" and b"facet normal" in data[:400]:
    print(data.count(b"facet normal"))
else:
    print(struct.unpack("<I", data[80:84])[0])
PY
}

# events <stdout> <tag>: the [[SLICER_EVENT]] records with that tag, one JSON per line.
events() {
    grep '^\[\[SLICER_EVENT\]\]' "$1" | sed 's/^\[\[SLICER_EVENT\]\] //' | python3 -c '
import json, sys
tag = sys.argv[1]
for line in sys.stdin:
    try:
        e = json.loads(line)
    except ValueError:
        continue
    if e.get("tag") == tag:
        print(json.dumps(e))
' "$2"
}

# field <json line> <key>
field() { python3 -c 'import json, sys; print(json.loads(sys.argv[1])[sys.argv[2]])' "$1" "$2"; }

# run <label> <stdout> <stderr> -- <command...>: runs, keeps rc in $RC.
run() {
    local out="$1" err="$2"; shift 2
    set +e
    "$@" > "$out" 2> "$err"
    RC=$?
    set -e
}

# Relative names: a Windows build given a Unix-style absolute path keeps the
# whole converted path as the object's name, and the STL export then has no
# usable file name. A real Windows caller passes Windows paths.
cd "$WORKDIR"
SPHERE=sphere.stl
SPHERE_TRIANGLES="$(sphere "$SPHERE" 500 201)"   # 200,000
test "$SPHERE_TRIANGLES" = 200000
BIG=big.stl
BIG_TRIANGLES="$(sphere "$BIG" 1000 501)"        # 1,000,000
test "$BIG_TRIANGLES" = 1000000
HASHES_BEFORE="$(python3 -c 'import hashlib, sys; [print(hashlib.sha256(open(p, "rb").read()).hexdigest()) for p in sys.argv[1:]]' "$SPHERE" "$BIG")"

index=0
for binary in "${BINARIES[@]}"; do
    label="${LABELS[$index]}"
    case "$label" in
        bambu) fixture="$SCRIPT_DIR/fixtures/calib_base.3mf" ;;
        orca) fixture="$SCRIPT_DIR/fixtures/calib_base_orca.3mf" ;;
        snapmaker) fixture="$SCRIPT_DIR/fixtures/calib_base_snapmaker.3mf" ;;
    esac
    d="$label"
    mkdir -p "$d"

    # A detail level reduces the part and reports both counts.
    run "$d/level.out" "$d/level.err" "$binary" "$SPHERE" --printer-preset "$A1M" --info --simplify medium
    test "$RC" = 0
    ev="$(events "$d/level.out" MeshSimplified)"
    test "$(printf '%s\n' "$ev" | wc -l | tr -d ' ')" = 1
    test "$(field "$ev" triangles_before)" = "$SPHERE_TRIANGLES"
    test "$(field "$ev" mode)" = level
    test "$(field "$ev" level)" = medium
    LEVEL_AFTER="$(field "$ev" triangles_after)"
    test "$LEVEL_AFTER" -lt "$SPHERE_TRIANGLES"
    test "$LEVEL_AFTER" -gt 0
    # A coarser level keeps fewer triangles than a finer one.
    run "$d/low.out" "$d/low.err" "$binary" "$SPHERE" --printer-preset "$A1M" --info --simplify extra-low
    test "$RC" = 0
    test "$(field "$(events "$d/low.out" MeshSimplified)" triangles_after)" -lt "$LEVEL_AFTER"
    run "$d/high.out" "$d/high.err" "$binary" "$SPHERE" --printer-preset "$A1M" --info --simplify extra-high
    test "$RC" = 0
    test "$(field "$(events "$d/high.out" MeshSimplified)" triangles_after)" -gt "$LEVEL_AFTER"

    # The decimate ratio keeps the share the dialog's rule keeps: 90% removed
    # of 200,000 leaves 20,000. The collapse removes two triangles a step and
    # stops at or just under the count (QuadricEdgeCollapse.cpp 228, 346).
    run "$d/ratio.out" "$d/ratio.err" "$binary" "$SPHERE" --printer-preset "$A1M" --info --simplify-ratio 90
    test "$RC" = 0
    ev="$(events "$d/ratio.out" MeshSimplified)"
    test "$(field "$ev" target_triangles)" = 20000
    after="$(field "$ev" triangles_after)"
    test "$after" -le 20000
    test "$after" -ge 19998

    # A wanted count, directly.
    run "$d/count.out" "$d/count.err" "$binary" "$SPHERE" --printer-preset "$A1M" --info --simplify-triangles 5000
    test "$RC" = 0
    after="$(field "$(events "$d/count.out" MeshSimplified)" triangles_after)"
    test "$after" -le 5000
    test "$after" -ge 4998

    # A part already at or under the count is left as it is, and says so.
    run "$d/skip.out" "$d/skip.err" "$binary" "$SPHERE" --printer-preset "$A1M" --info --simplify-triangles 200000
    test "$RC" = 0
    test -z "$(events "$d/skip.out" MeshSimplified)"
    test "$(field "$(events "$d/skip.out" SimplifySkipped)" triangles_after)" = "$SPHERE_TRIANGLES"

    # The slice and the exports see the reduced mesh: result.json's count and
    # the exported STL's count are the event's count.
    mkdir -p "$d/slice"
    run "$d/slice.out" "$d/slice.err" "$binary" "$SPHERE" --printer-preset "$A1M" --slice 1 --outputdir "$d/slice" \
        --simplify-triangles 20000 --export-stl
    test "$RC" = 0
    after="$(field "$(events "$d/slice.out" MeshSimplified)" triangles_after)"
    test "$(python3 -c 'import json, sys; d = json.load(open(sys.argv[1])); print(d["sliced_plates"][0]["objects"][0]["triangle_count"])' "$d/slice/result.json")" = "$after"
    exported="$(find "$d/slice" -name '*.stl' | head -1)"
    test -s "$exported"
    test "$(stl_triangles "$exported")" = "$after"
    test -s "$(find "$d/slice" -name '*.gcode' | head -1)"

    # The desktop's suggestion: a single-part object of a million triangles
    # or more gets the warning; simplified under that, it does not.
    run "$d/big.out" "$d/big.err" "$binary" "$BIG" --printer-preset "$A1M" --info
    test "$RC" = 0
    ev="$(events "$d/big.out" LargeMesh)"
    test "$(field "$ev" event)" = warning
    test "$(field "$ev" triangle_count)" = "$BIG_TRIANGLES"
    run "$d/bigsimp.out" "$d/bigsimp.err" "$binary" "$BIG" --printer-preset "$A1M" --info --simplify low
    test "$RC" = 0
    test -z "$(events "$d/bigsimp.out" LargeMesh)"
    test "$(field "$(events "$d/bigsimp.out" MeshSimplified)" triangles_before)" = "$BIG_TRIANGLES"

    # --simplify-objects takes the 3MF's object ids: an id the file has not is
    # refused with the ids it has; one it has is simplified.
    mkdir -p "$d/obj"
    run "$d/badid.out" "$d/badid.err" "$binary" "$fixture" --slice 1 --outputdir "$d/obj" --simplify medium --simplify-objects 999999
    test "$RC" != 0
    grep -q "no object has id 999999; the file's object ids are " "$d/badid.err"
    id="$(sed -n "s/.*the file's object ids are \([0-9]*\).*/\1/p" "$d/badid.err" | head -1)"
    test -n "$id"
    mkdir -p "$d/obj2"
    run "$d/goodid.out" "$d/goodid.err" "$binary" "$fixture" --slice 1 --outputdir "$d/obj2" --simplify medium --simplify-objects "$id"
    test "$RC" = 0
    # The fixture's part is a 12-triangle cube: nothing collapses at this
    # level, so the selected part reports a skip; either way it was reached.
    test -n "$(events "$d/goodid.out" MeshSimplified)$(events "$d/goodid.out" SimplifySkipped)"

    # Refused combinations and values: a sentence on stderr, no run.
    refuse() {
        local want="$1"; shift
        run "$d/refuse.out" "$d/refuse.err" "$binary" "$SPHERE" --printer-preset "$A1M" --info "$@"
        test "$RC" != 0
        grep -q -- "$want" "$d/refuse.err"
        test -z "$(events "$d/refuse.out" MeshSimplified)"
    }
    refuse "mutually exclusive" --simplify medium --simplify-ratio 50
    refuse "mutually exclusive" --simplify-ratio 50 --simplify-triangles 100
    refuse "needs --simplify" --simplify-objects 1
    refuse "needs --simplify" --simplify-keep-paint
    refuse "expects one of extra-high, high, medium, low, extra-low" --simplify huge
    refuse "above 0 and below 100" --simplify-ratio 0
    refuse "above 0 and below 100" --simplify-ratio 100
    refuse "expects a number" --simplify-ratio half
    refuse "at least 1" --simplify-triangles 0
    refuse "expects an integer" --simplify-triangles many
    refuse "object ids separated by commas" --simplify-objects 1,x
    refuse "needs a value" --simplify
    if [ "$label" = snapmaker ]; then
        refuse "not available in slicer_cli-snapmaker" --simplify medium --simplify-keep-paint
    else
        # Accepted; an unpainted part has nothing to keep.
        run "$d/keep.out" "$d/keep.err" "$binary" "$SPHERE" --printer-preset "$A1M" --info --simplify medium --simplify-keep-paint
        test "$RC" = 0
        test "$(field "$(events "$d/keep.out" MeshSimplified)" paint_kept)" = False
    fi

    echo "PASS: $label: --simplify reduces by level, ratio and count like the desktop tool; the slice, result.json and exports carry the reduced mesh; the 1M-triangle suggestion and the refusals are as documented"
    index=$((index + 1))
done

test "$(python3 -c 'import hashlib, sys; [print(hashlib.sha256(open(p, "rb").read()).hexdigest()) for p in sys.argv[1:]]' "$SPHERE" "$BIG")" = "$HASHES_BEFORE"
echo "PASS: --simplify leaves its input files unchanged"
