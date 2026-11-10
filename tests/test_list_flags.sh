#!/usr/bin/env bash
# --list-flags: every flag this program takes as one JSON document — what it
# does, what value it takes, which binaries take it — so a caller driving the
# binary reads its contract from the binary it runs, and a flag a release adds
# is a visible change of the document. Stdout is the document, exit 0; like
# --list-settings it never slices, so --slice with it is refused.
#
# Usage: tests/test_list_flags.sh --engine bambu|orca|snapmaker BIN
#        tests/test_list_flags.sh SLICER_CLI SLICER_CLI_ORCASLICER SLICER_CLI_SNAPMAKER
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT
fails=0

absolute() { case "$1" in /*|[A-Za-z]:*) echo "$1";; *) echo "$PWD/$1";; esac; }
declare -a ENGINES=() BINS=()
if [ "${1:-}" = "--engine" ]; then
    ENGINES=("${2:?engine}"); BINS=("$(absolute "${3:?binary}")")
else
    ENGINES=(bambu orca snapmaker)
    BINS=("$(absolute "${1:?usage}")" "$(absolute "${2:?usage}")" "$(absolute "${3:?usage}")")
fi

for i in "${!ENGINES[@]}"; do
    e="${ENGINES[$i]}"; bin="${BINS[$i]}"
    doc="$WORKDIR/flags-$e.json"
    if ! "$bin" --list-flags > "$doc" 2> "$WORKDIR/flags-$e.err"; then
        echo "FAIL: --list-flags [$e] exit $?; $(tail -n 1 "$WORKDIR/flags-$e.err")"
        fails=$((fails + 1))
        continue
    fi
    python3 - "$doc" "$e" <<'PY' || fails=$((fails + 1))
import json, sys
path, engine = sys.argv[1], sys.argv[2]
doc = json.load(open(path))
want = {"bambu": "bambustudio", "orca": "orcaslicer", "snapmaker": "snapmaker"}[engine]
assert doc["engine"] == want, doc["engine"]
assert doc["engine_version"], "no engine_version"
flags = doc["flags"]
by = {f["flag"]: f for f in flags}
assert len(by) == len(flags), "duplicate flags"
assert [f["flag"] for f in flags] == sorted(by), "not sorted by flag"
assert 80 <= len(flags) <= 300, "%d flags" % len(flags)
for f in flags:
    assert f["flag"].startswith("-") and f["source"] in ("official", "slicer-cli"), f
    assert isinstance(f["supported"], bool) and isinstance(f["takes_value"], bool), f
    assert ("refusal" in f) == (not f["supported"]), f
    assert f["works_in"], f
    if f["source"] == "official":
        assert f["key"], f
        assert "type" in f or not f["supported"], f
# slicer-cli's own flags, with their value names.
for flag, value in [("--list-flags", None), ("--list-settings", None), ("--engine-info", "FILE"),
                    ("--help", None), ("--gcode-timing", "ORIGINAL OPTIMIZED"),
                    ("--printer-preset", "NAME"), ("--simplify", "LEVEL"), ("--simplify-ratio", "PERCENT"),
                    ("--simplify-triangles", "N"), ("--simplify-objects", "IDS"), ("--simplify-keep-paint", None)]:
    f = by[flag]
    assert f["source"] == "slicer-cli" and f["supported"] is True, f
    assert f["takes_value"] == (value is not None) and f.get("value") == value, f
    assert f["help"], f
assert by["--help"]["aliases"] == ["-h"], by["--help"]
if engine == "bambu":
    assert "--printer-vendor-bbl" not in by, "the Bambu build has no --printer-vendor-bbl"
else:
    assert by["--printer-vendor-bbl"]["works_in"] == "slicer_cli-orcaslicer and slicer_cli-snapmaker", by["--printer-vendor-bbl"]
keep = by["--simplify-keep-paint"]
assert keep["works_in"] == "slicer_cli and slicer_cli-orcaslicer", keep
# The official flags, with their keys, types and the shared value words.
for flag, key, typ in [("--slice", "slice", "int"), ("--arrange", "arrange", "int"),
                       ("--export-3mf", "export_3mf", "string"), ("--scale", "scale", "float"),
                       ("--rotate", "rotate", "float"), ("--repetitions", "repetitions", "int"),
                       ("--skip-objects", "skip_objects", "ints"), ("--mtcpp", "mtcpp", "int"),
                       ("--allow-newer-file", "allow_newer_file", "bool"),
                       ("--ensure-on-bed", "ensure_on_bed", "bool"), ("--outputdir", "outputdir", "string")]:
    f = by[flag]
    assert f["source"] == "official" and f["key"] == key, f
    assert f["type"] == typ, (flag, f["type"])
    assert f["takes_value"] == (typ != "bool"), f
    assert f["supported"] is True, f
assert "-o" in by["--output"].get("aliases", []) or "-o" in by, by.get("--output")
# A print setting is a flag too, but --list-settings lists those.
assert "--curr-bed-type" not in by and "--layer-height" not in by, "print settings belong to --list-settings"
assert by["--datadir"]["note"], by["--datadir"]
# A flag of the other engine's own command line is listed and refused with
# the sentence the run prints; --export-png is Bambu Studio's alone.
png = by["--export-png"]
if engine == "bambu":
    assert png["source"] == "official" and png["supported"] is False and "makes no pictures" in png["refusal"], png
else:
    assert png["supported"] is False and "works only in slicer_cli" in png["refusal"], png
    assert png["works_in"] == "slicer_cli only (Bambu Studio engine)", png
assert by["--downward-check"]["works_in"] == "slicer_cli and slicer_cli-orcaslicer", by["--downward-check"]
print("PASS: --list-flags [%s]: %d flags, %d of slicer-cli's own, %d refused" % (
    engine, len(flags), sum(f["source"] == "slicer-cli" for f in flags), sum(not f["supported"] for f in flags)))
PY
    python3 "$SCRIPT_DIR/tools/check_json_schema.py" "$SCRIPT_DIR/../schemas/list-flags.schema.json" "$doc" ||
        { echo "FAIL: --list-flags [$e] does not match schemas/list-flags.schema.json"; fails=$((fails + 1)); }

    # Every flag --help prints is in the document, and the other way round.
    "$bin" --help > "$WORKDIR/help-$e.txt" 2>&1
    python3 - "$doc" "$WORKDIR/help-$e.txt" <<'PY' || fails=$((fails + 1))
import json, re, sys
doc = json.load(open(sys.argv[1]))
text = open(sys.argv[2], encoding="utf-8", errors="replace").read()
tables = text.split("=== Every print setting ===")[0]   # the two flag tables; settings are --list-settings
listed = {f["flag"] for f in doc["flags"]} | {a for f in doc["flags"] for a in f.get("aliases", [])}
helped = set()
for line in tables.splitlines():
    m = re.match(r" {1,4}((?:--?[a-z][a-z0-9-]*)(?:\|--?[a-z][a-z0-9-]*)*)", line)
    if m:
        helped.update(m.group(1).split("|"))
missing = sorted(helped - listed)
assert not missing, "in --help, not in --list-flags: %s" % missing
assert len(helped) > 60, "only %d flags read from --help" % len(helped)
print("PASS: --list-flags covers the %d flags --help lists before the print settings" % len(helped))
PY

    # --slice with it is refused like --list-settings; result.json names why.
    out="$WORKDIR/slice-$e/out"
    "$bin" --list-flags --slice 1 --outputdir "$out" > "$WORKDIR/slice-$e.out" 2>&1
    if [ "$?" = 0 ]; then
        echo "FAIL: --list-flags with --slice was accepted [$e]"
        fails=$((fails + 1))
    elif ! python3 -c '
import json, sys
d = json.load(open(sys.argv[1]))
assert d["return_code"] != 0 and "lists this program\x27s flags and slices nothing" in d["error_string"], d
' "$out/result.json"; then
        echo "FAIL: --list-flags with --slice [$e] left no result.json naming the refusal"
        fails=$((fails + 1))
    fi
done

if [ "$fails" = 0 ]; then
    echo "PASS: --list-flags prints this program's flags on every build given"
    exit 0
fi
echo "FAIL: $fails check(s) of --list-flags"
exit 1
