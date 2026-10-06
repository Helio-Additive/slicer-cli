#!/usr/bin/env bash
# Per-plate custom G-code on a real project (D5): each plate's G-code carries
# that plate's own pauses, at their layers, and no other plate's.
#
# The loader keys a project's per-layer custom G-code by plate (plate_id - 1,
# bbs_3mf.cpp 3446/3474) and the Print reads the model's current plate
# (Print.cpp 517-518); the official CLI sets it per plate (BambuStudio.cpp
# 6493; OrcaSlicer.cpp 5617). The product's call is used: <3mf> --plate N -o.
#
# Usage: tests/test_plate_custom_gcode.sh /path/to/slicer_cli[-orcaslicer] /path/to/project.3mf
# The project must carry Metadata/custom_gcode_per_layer.xml with pauses (for
# example the DiceTower_WirelessRift_V4a.3mf maker file: one M400 U1 pause on
# each of plates 4-8). A binary that refuses the file (another engine's file)
# is reported as SKIP.
set -uo pipefail

BIN="${1:?usage: $0 slicer_cli project.3mf}"
PROJECT="${2:?usage: $0 slicer_cli project.3mf}"
WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT

# plate -> [(top_z, type, extra)], the plate count, and the pause G-code.
python3 - "$PROJECT" > "$WORKDIR/expect.json" <<'PY' || { echo "FAIL: cannot read $PROJECT"; exit 1; }
import json, re, sys, zipfile
z = zipfile.ZipFile(sys.argv[1])
xml = z.read("Metadata/custom_gcode_per_layer.xml").decode()
items = {}
for plate in re.findall(r"<plate>.*?</plate>", xml, re.S):
    pid = int(re.search(r'<plate_info id="(\d+)"', plate).group(1))
    for m in re.finditer(r"<layer ([^>]*)/>", plate):
        a = dict(re.findall(r'(\w+)="([^"]*)"', m.group(1)))
        items.setdefault(pid, []).append([float(a["top_z"]), int(a.get("type", "1")), a.get("extra", "")])
plates = sorted({int(p) for p in re.findall(r'key="plater_id" value="(\d+)"', z.read("Metadata/model_settings.config").decode())})
settings = json.loads(z.read("Metadata/project_settings.config"))
pause = (settings.get("machine_pause_gcode") or "M400 U1").splitlines()[0].strip()
json.dump({"items": items, "plates": plates, "pause": pause}, sys.stdout)
PY

plates=$(python3 -c 'import json,sys; print(" ".join(map(str, json.load(open(sys.argv[1]))["plates"])))' "$WORKDIR/expect.json")
fails=0; checked=0
for p in $plates; do
    g="$WORKDIR/plate_$p.gcode"
    "$BIN" "$PROJECT" --plate "$p" -o "$g" > "$WORKDIR/out-$p.txt" 2>&1
    rc=$?
    if [ $rc != 0 ] || [ ! -s "$g" ]; then
        if grep -q '"tag":"FileVersionNewerThanEngine"\|cannot print this file' "$WORKDIR/out-$p.txt"; then
            echo "SKIP: $(basename "$BIN") refuses $(basename "$PROJECT") (another engine's file)"; exit 0
        fi
        echo "FAIL: plate $p exit $rc"; fails=$((fails + 1)); continue
    fi
    checked=$((checked + 1))
    python3 - "$WORKDIR/expect.json" "$g" "$p" <<'PY' || fails=$((fails + 1))
import json, re, sys
e = json.load(open(sys.argv[1])); g = sys.argv[2]; p = sys.argv[3]
want = [i for i in e["items"].get(p, []) if i[1] == 1]      # PausePrint
pause = e["pause"]
z = None; got = []
for line in open(g, errors="replace"):
    m = re.match(r"^; Z_HEIGHT: ([0-9.]+)", line) or re.match(r"^;Z:([0-9.]+)", line)
    if m:
        z = float(m.group(1))
    elif line.strip() == pause:
        got.append(z)
ok = len(got) == len(want)
# Each pause on the first layer at or above its height (within one layer, 0.4 mm).
for (top_z, _, _), at in zip(sorted(want), sorted(x for x in got if x is not None)):
    ok = ok and at is not None and -0.01 <= at - top_z <= 0.4
print("%s: plate %s pauses wanted at %s, G-code has %d at %s" % ("PASS" if ok else "FAIL", p,
      [w[0] for w in sorted(want)] or "none", len(got), got or "none"))
sys.exit(0 if ok else 1)
PY
done
[ $checked -gt 0 ] || { echo "FAIL: no plate sliced"; exit 1; }
if [ $fails = 0 ]; then echo "PASS: every plate of $(basename "$PROJECT") carries its own pauses at their layers ($checked plates)"; exit 0; fi
echo "FAIL: $fails plate(s) of $(basename "$PROJECT")"; exit 1
