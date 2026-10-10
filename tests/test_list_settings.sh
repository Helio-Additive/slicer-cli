#!/usr/bin/env bash
# --list-settings: this engine's settings table (PrintConfigDef) as one JSON
# document — every key with what it means, its type, its default, its bounds
# and the values it takes — so an agent driving this binary finds a setting
# ("infill" -> sparse_infill_density, 0-100 %) without reading the engine's
# source. The document is on stdout, warnings on stderr, exit 0; like
# --list-presets it never slices, so --slice with it is refused.
#
# Usage: tests/test_list_settings.sh SLICER_CLI SLICER_CLI_ORCASLICER
set -uo pipefail

B="${1:?usage: $0 slicer_cli slicer_cli-orcaslicer}"
O="${2:?usage: $0 slicer_cli slicer_cli-orcaslicer}"
B="$(cd "$(dirname "$B")" && pwd -P)/$(basename "$B")"
O="$(cd "$(dirname "$O")" && pwd -P)/$(basename "$O")"
WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT
fails=0

for e in bambu orca; do
    bin=$B
    [ "$e" = orca ] && bin=$O
    doc="$WORKDIR/list-$e.json"
    if ! "$bin" --list-settings > "$doc" 2> "$WORKDIR/list-$e.err"; then
        echo "FAIL: --list-settings [$e] exit $?; $(tail -n 1 "$WORKDIR/list-$e.err")"
        fails=$((fails + 1))
        continue
    fi
    python3 - "$doc" "$e" <<'PY' || fails=$((fails + 1))
import json, sys
path, engine = sys.argv[1], sys.argv[2]
doc = json.load(open(path))
want = "orcaslicer" if engine == "orca" else "bambustudio"
assert doc["engine"] == want, doc["engine"]
assert doc["engine_version"], "no engine_version"
assert doc["slicer_cli_version"], "no slicer_cli_version"
settings = doc["settings"]
by = {s["key"]: s for s in settings}
assert len(by) == len(settings), "duplicate keys"
assert len(settings) > 600, "only %d settings" % len(settings)
assert [s["key"] for s in settings] == sorted(by), "not sorted by key"
for s in settings:
    assert s["key"] and s["type"] and s["mode"], s
    assert s["mode"] in ("simple", "advanced", "develop", "expert"), s["mode"]
    assert isinstance(s["vector"], bool), s
    assert "scope" not in s or s["scope"] in ("process", "filament", "printer"), s
    assert all(isinstance(s[b], (int, float)) for b in ("min", "max") if b in s), s
    assert "enum" not in s or all(isinstance(v, dict) and v["value"] and v["label"] for v in s["enum"]), s
# A scalar of the engine's own value type, and its plural as a vector.
lh = by["layer_height"]
assert lh["type"] == "float" and lh["vector"] is False, lh
nd = by["nozzle_diameter"]
assert nd["type"] == "floats" and nd["vector"] is True, nd
# The setting behind "infill": key, meaning, type, bounds, unit, preset.
d = by["sparse_infill_density"]
assert d["type"] == "percent" and d["vector"] is False, d
assert d["min"] == 0 and d["max"] == 100, d
assert d["scope"] == "process" and d["unit"] == "%", d
assert d["label"] == "Sparse infill density" and d["category"] == "Strength", d
assert d["default"] == "20%", d["default"]
# An enum's values come with their labels.
p = by["sparse_infill_pattern"]
assert p["type"] == "enum" and p["vector"] is False, p
assert len(p["enum"]) > 5, p
assert all(v["value"] and v["label"] for v in p["enum"]), p["enum"][:3]
assert "grid" in [v["value"] for v in p["enum"]], p["enum"]
# A definition with no bound carries none: the engine's sentinel ("unbounded")
# is not a bound.
assert "min" not in by["enable_support"] and "max" not in by["enable_support"], by["enable_support"]
print("PASS: --list-settings [%s]: %d settings, sparse_infill_density 0-100 %%, "
      "sparse_infill_pattern %d values" % (engine, len(settings), len(p["enum"])))
PY

    # --slice with it is refused like --list-presets, and the refusal leaves
    # result.json in --outputdir naming the reason.
    out="$WORKDIR/slice-$e/out"
    "$bin" --list-settings --slice 1 --outputdir "$out" > "$WORKDIR/slice-$e.out" 2>&1
    rc=$?
    if [ "$rc" = 0 ]; then
        echo "FAIL: --list-settings with --slice was accepted [$e]"
        fails=$((fails + 1))
    elif ! python3 -c '
import json, sys
d = json.load(open(sys.argv[1]))
assert d["return_code"] != 0 and "lists this engine\x27s settings and slices nothing" in d["error_string"], d
' "$out/result.json"; then
        echo "FAIL: --list-settings with --slice [$e] left no result.json naming the refusal"
        fails=$((fails + 1))
    fi
done

if [ "$fails" = 0 ]; then
    echo "PASS: --list-settings prints this engine's settings table on both builds"
    exit 0
fi
echo "FAIL: $fails check(s) of --list-settings"
exit 1
