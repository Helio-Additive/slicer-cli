#!/usr/bin/env bash
# Existing-G-code timing is a strict one-document command on all engine builds.
# Usage: tests/test_gcode_timing.sh --engine BAMBU|ORCA|SNAPMAKER BIN
#    or: tests/test_gcode_timing.sh BAMBU [ORCA [SNAPMAKER]]
set -euo pipefail

declare -a BINARIES LABELS
if [ "$#" = 3 ] && [ "$1" = "--engine" ]; then
    case "$2" in BAMBU|ORCA|SNAPMAKER) ;; *) echo "invalid engine: $2" >&2; exit 64 ;; esac
    BINARIES=("$3")
    LABELS=("$2")
elif [ "$#" -ge 1 ] && [ "$#" -le 3 ]; then
    BINARIES=("$@")
    LABELS=(BAMBU ORCA SNAPMAKER)
else
    echo "usage: $0 --engine BAMBU|ORCA|SNAPMAKER BIN" >&2
    echo "   or: $0 BAMBU [ORCA [SNAPMAKER]]" >&2
    exit 64
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
ORIGINAL="$SCRIPT_DIR/fixtures/gcode_timing_original.gcode"
OPTIMIZED="$SCRIPT_DIR/fixtures/gcode_timing_optimized.gcode"
WORKDIR="$(mktemp -d "${TMPDIR:-/tmp}/gcode_timing.XXXXXX")"
trap 'rm -rf "$WORKDIR"' EXIT

# The schema accepts every finite positive estimate, rejects zero/negative
# estimates, and never accepts a response mixing error and success fields.
python3 - "$SCRIPT_DIR" <<'PY_SCHEMA'
import importlib.util, json, pathlib, sys
base = pathlib.Path(sys.argv[1])
spec = importlib.util.spec_from_file_location("schema_check", base / "tools/check_json_schema.py")
validator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(validator)
schema = json.loads((base.parent / "schemas/gcode-timing.schema.json").read_text())
validator.walk(schema, "$")
def valid(payload):
    return not validator.errors(payload, schema, schema, "$")
base_doc = {"schema_version": 1, "engine": "BAMBU", "engine_revision": "test"}
for seconds in (1e-12, 1.0, 10000.5):
    assert valid(dict(base_doc, original_seconds=seconds, optimized_seconds=seconds))
for seconds in (0, -1):
    assert not valid(dict(base_doc, original_seconds=seconds, optimized_seconds=1))
assert valid(dict(base_doc, error_code="TIMING_UNAVAILABLE"))
assert not valid(dict(base_doc, original_seconds=1, optimized_seconds=1, error_code="TIMING_UNAVAILABLE"))
PY_SCHEMA

python3 - "$ORIGINAL" "$OPTIMIZED" > "$WORKDIR/hashes.before" <<'PY'
import hashlib, pathlib, sys
for name in sys.argv[1:]:
    print(hashlib.sha256(pathlib.Path(name).read_bytes()).hexdigest())
PY

run_timing() {
    local binary="$1" label="$2" original="$3" optimized="$4" stdout="$5" stderr="$6"
    if [ "$label" != BAMBU ]; then
        "$binary" --gcode-timing "$original" "$optimized" --printer-vendor-bbl false > "$stdout" 2> "$stderr"
    else
        "$binary" --gcode-timing "$original" "$optimized" > "$stdout" 2> "$stderr"
    fi
}

run_timing_with_timeout() {
    local binary="$1" label="$2" original="$3" optimized="$4" stdout="$5" stderr="$6"
    python3 - "$binary" "$label" "$original" "$optimized" "$stdout" "$stderr" <<'PY_TIMEOUT'
import pathlib, subprocess, sys
binary, label, original, optimized, stdout, stderr = sys.argv[1:]
command = [binary, "--gcode-timing", original, optimized]
if label != "BAMBU":
    command += ["--printer-vendor-bbl", "false"]
try:
    result = subprocess.run(command, text=True, capture_output=True, timeout=10)
except subprocess.TimeoutExpired as error:
    raise SystemExit(f"timing command did not return within 10 seconds: {error}")
pathlib.Path(stdout).write_text(result.stdout)
pathlib.Path(stderr).write_text(result.stderr)
raise SystemExit(result.returncode)
PY_TIMEOUT
}

assert_schema() {
    python3 "$SCRIPT_DIR/tools/check_json_schema.py" "$SCRIPT_DIR/../schemas/gcode-timing.schema.json" "$1"
}

index=0
for binary in "${BINARIES[@]}"; do
    label="${LABELS[$index]}"
    stdout="$WORKDIR/$index.stdout"
    stderr="$WORKDIR/$index.stderr"
    run_timing "$binary" "$label" "$ORIGINAL" "$OPTIMIZED" "$stdout" "$stderr"
    test ! -s "$stderr"
    test "$(wc -l < "$stdout" | tr -d ' ')" = 1
    assert_schema "$stdout"
    python3 - "$stdout" "$label" <<'PY'
import json, math, sys
d = json.load(open(sys.argv[1]))
assert d["engine"] == sys.argv[2], d
assert d["engine_revision"] != "unknown", d
assert math.isfinite(d["original_seconds"]) and d["original_seconds"] > 0, d
assert math.isfinite(d["optimized_seconds"]) and d["optimized_seconds"] > 0, d
# The paired fixtures differ only in their G1 feed rates. This validates that
# the selected native processor read both files; it is not desktop parity proof.
assert d["optimized_seconds"] < d["original_seconds"], d
PY
    # Paths are UTF-8 on every platform, including the Windows wide argv path.
    unicode_dir="$WORKDIR/模型 with spaces"
    mkdir -p "$unicode_dir"
    cp "$ORIGINAL" "$unicode_dir/original.gcode"
    cp "$OPTIMIZED" "$unicode_dir/optimized.gcode"
    run_timing "$binary" "$label" "$unicode_dir/original.gcode" "$unicode_dir/optimized.gcode" "$stdout.unicode" "$stderr.unicode"
    test ! -s "$stderr.unicode"
    cmp "$stdout" "$stdout.unicode"
    # Missing, empty, directory, and FIFO inputs must fail as JSON rather than
    # crashing a native importer or blocking on a special file.
    empty="$WORKDIR/empty-$index.gcode"
    : > "$empty"
    inputs=("$empty" "$WORKDIR" "$WORKDIR/missing-$index.gcode")
    fifo="$WORKDIR/$label.fifo"
    if python3 - "$fifo" <<'PY_FIFO'
import os, sys
if hasattr(os, "mkfifo"):
    os.mkfifo(sys.argv[1])
else:
    raise SystemExit(1)
PY_FIFO
    then
        inputs+=("$fifo")
    fi
    for input in "${inputs[@]}"; do
        set +e
        run_timing_with_timeout "$binary" "$label" "$input" "$OPTIMIZED" "$stdout.input" "$stderr.input"
        rc=$?
        set -e
        test "$rc" -eq 1
        test ! -s "$stderr.input"
        assert_schema "$stdout.input"
        python3 - "$stdout.input" "$input" <<'PY_INPUT'
import json, sys
payload = json.load(open(sys.argv[1]))
assert payload["error_code"] == "TIMING_UNAVAILABLE", payload
assert sys.argv[2] not in open(sys.argv[1]).read()
PY_INPUT
    done
    "$binary" --help > "$stdout.help" 2> "$stderr.help"
    grep -q -- '--gcode-timing' "$stdout.help"
    # Identical imports must produce identical normal-mode estimates.
    run_timing "$binary" "$label" "$ORIGINAL" "$ORIGINAL" "$stdout.same" "$stderr.same"
    test ! -s "$stderr.same"
    python3 - "$stdout.same" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))
assert d["original_seconds"] == d["optimized_seconds"], d
PY
    # Swapping the pair must exchange the two reported native estimates.
    run_timing "$binary" "$label" "$OPTIMIZED" "$ORIGINAL" "$stdout.swap" "$stderr.swap"
    test ! -s "$stderr.swap"
    python3 - "$stdout" "$stdout.swap" <<'PY'
import json, math, sys
forward, swapped = (json.load(open(path)) for path in sys.argv[1:])
assert math.isclose(forward["original_seconds"], swapped["optimized_seconds"], rel_tol=0, abs_tol=1e-9), (forward, swapped)
assert math.isclose(forward["optimized_seconds"], swapped["original_seconds"], rel_tol=0, abs_tol=1e-9), (forward, swapped)
PY
    # A timing command must refuse all slice/output/probe combinations with
    # the same one-document contract and no leaked file names.
    set +e
    "$binary" --gcode-timing "$ORIGINAL" "$OPTIMIZED" --slice 1 > "$stdout.conflict" 2> "$stderr.conflict"
    rc=$?
    set -e
    test "$rc" -ne 0
    test ! -s "$stderr.conflict"
    assert_schema "$stdout.conflict"
    python3 - "$stdout.conflict" <<'PY'
import json, sys
assert json.load(open(sys.argv[1]))["error_code"] == "CONFLICTING_ARGUMENTS"
PY
    # A timing-looking word consumed as an option value, or after --, stays on
    # the normal parser path. A genuine mixed timing option remains a strict
    # timing refusal even when another option appears before it.
    for args in '--printer-preset --gcode-timing' '-- --gcode-timing'; do
        set +e
        # shellcheck disable=SC2086
        "$binary" $args > "$stdout.literal" 2> "$stderr.literal"
        rc=$?
        set -e
        test "$rc" -ne 64
        ! grep -q '"error_code"' "$stdout.literal"
    done
    set +e
    if [ "$label" != BAMBU ]; then
        "$binary" --slice 1 --gcode-timing "$ORIGINAL" "$OPTIMIZED" --printer-vendor-bbl false > "$stdout.mixed" 2> "$stderr.mixed"
    else
        "$binary" --slice 1 --gcode-timing "$ORIGINAL" "$OPTIMIZED" > "$stdout.mixed" 2> "$stderr.mixed"
    fi
    rc=$?
    set -e
    test "$rc" -eq 64
    test ! -s "$stderr.mixed"
    assert_schema "$stdout.mixed"
    python3 - "$stdout.mixed" <<'PY'
import json, sys
assert json.load(open(sys.argv[1]))["error_code"] == "CONFLICTING_ARGUMENTS"
PY
    # A Python exec harness closes both stdin and stderr in the executable
    # process. This exercises CRT descriptor allocation on Windows too, unlike
    # a shell redirection which can leave a Windows child with a replacement
    # standard handle.
    python3 - "$binary" "$label" "$ORIGINAL" "$OPTIMIZED" "$stdout.closed-stderr" <<'PY_CLOSED_FDS'
import os, pathlib, subprocess, sys
binary, label, original, optimized, stdout = sys.argv[1:]
command = [binary, "--gcode-timing", original, optimized]
if label != "BAMBU":
    command += ["--printer-vendor-bbl", "false"]
launcher = """import os, sys
for fd in (0, 2):
    try:
        os.close(fd)
    except OSError:
        pass
os.execv(sys.argv[1], sys.argv[1:])
"""
result = subprocess.run([sys.executable, "-c", launcher] + command,
                        text=True, capture_output=True, timeout=10)
pathlib.Path(stdout).write_text(result.stdout)
assert result.returncode == 1, result.returncode
assert not result.stderr, result.stderr
PY_CLOSED_FDS
    assert_schema "$stdout.closed-stderr"
    python3 - "$stdout.closed-stderr" <<'PY'
import json, sys
assert json.load(open(sys.argv[1]))["error_code"] == "UNAVAILABLE"
PY
    if [ "$label" = BAMBU ]; then
        set +e
        "$binary" --gcode-timing "$ORIGINAL" "$OPTIMIZED" --printer-vendor-bbl false > "$stdout.context-foreign" 2> "$stderr.context-foreign"
        rc=$?
        set -e
        test "$rc" -eq 64
        test ! -s "$stderr.context-foreign"
        assert_schema "$stdout.context-foreign"
        python3 - "$stdout.context-foreign" <<'PY'
import json, sys
assert json.load(open(sys.argv[1]))["error_code"] == "CONFLICTING_ARGUMENTS"
PY
    fi
    missing="$WORKDIR/not-a-gcode-$index"
    set +e
    if [ "$label" != BAMBU ]; then
        "$binary" --gcode-timing "$missing" "$OPTIMIZED" --printer-vendor-bbl false > "$stdout.error" 2> "$stderr.error"
    else
        "$binary" --gcode-timing "$missing" "$OPTIMIZED" > "$stdout.error" 2> "$stderr.error"
    fi
    rc=$?
    set -e
    test "$rc" -ne 0
    test ! -s "$stderr.error"
    ! grep -F "$missing" "$stdout.error"
    assert_schema "$stdout.error"
    python3 - "$stdout.error" <<'PY'
import json, sys
assert json.load(open(sys.argv[1]))["error_code"] == "TIMING_UNAVAILABLE"
PY
    if [ "$label" != BAMBU ]; then
        set +e
        "$binary" --gcode-timing "$ORIGINAL" "$OPTIMIZED" > "$stdout.context-required" 2> "$stderr.context-required"
        rc=$?
        set -e
        test "$rc" -eq 64
        test ! -s "$stderr.context-required"
        assert_schema "$stdout.context-required"
        python3 - "$stdout.context-required" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))
assert d["error_code"] == "PRINTER_CONTEXT_REQUIRED", d
assert "printer_vendor_bbl" not in d, d
PY
        set +e
        "$binary" --gcode-timing "$ORIGINAL" "$OPTIMIZED" --printer-vendor-bbl maybe > "$stdout.context-invalid" 2> "$stderr.context-invalid"
        rc=$?
        set -e
        test "$rc" -eq 64
        test ! -s "$stderr.context-invalid"
        assert_schema "$stdout.context-invalid"
        python3 - "$stdout.context-invalid" <<'PY'
import json, sys
assert json.load(open(sys.argv[1]))["error_code"] == "CONFLICTING_ARGUMENTS"
PY
        set +e
        "$binary" --gcode-timing "$ORIGINAL" "$OPTIMIZED" --printer-vendor-bbl > "$stdout.context-missing" 2> "$stderr.context-missing"
        rc=$?
        set -e
        test "$rc" -eq 64
        test ! -s "$stderr.context-missing"
        assert_schema "$stdout.context-missing"
        python3 - "$stdout.context-missing" <<'PY'
import json, sys
assert json.load(open(sys.argv[1]))["error_code"] == "CONFLICTING_ARGUMENTS"
PY
        set +e
        "$binary" --gcode-timing "$ORIGINAL" "$OPTIMIZED" --printer-vendor-bbl true --slice 1 > "$stdout.context-extra" 2> "$stderr.context-extra"
        rc=$?
        set -e
        test "$rc" -eq 64
        test ! -s "$stderr.context-extra"
        assert_schema "$stdout.context-extra"
        python3 - "$stdout.context-extra" <<'PY'
import json, sys
assert json.load(open(sys.argv[1]))["error_code"] == "CONFLICTING_ARGUMENTS"
PY
        "$binary" --gcode-timing "$ORIGINAL" "$OPTIMIZED" --printer-vendor-bbl true > "$stdout.context-true" 2> "$stderr.context-true"
        test ! -s "$stderr.context-true"
        assert_schema "$stdout.context-true"
        python3 - "$stdout.context-true" <<'PY'
import json, sys
assert json.load(open(sys.argv[1]))["original_seconds"] > 0
PY
        # Metadata-free imports must use the selected executable's resources,
        # never an arbitrary CWD info/nozzle_info.json.
        clean_dir="$WORKDIR/$label-clean"
        poison_dir="$WORKDIR/$label-poison"
        mkdir -p "$clean_dir" "$poison_dir/info"
        printf '%s\n' '{"nozzle_hrc":{"undefine":"invalid"}}' > "$poison_dir/info/nozzle_info.json"
        binary_dir="$(cd "$(dirname "$binary")" && pwd -P)"
        binary_from_cwd="$binary_dir/$(basename "$binary")"
        (cd "$clean_dir" && "$binary_from_cwd" --gcode-timing "$ORIGINAL" "$OPTIMIZED" --printer-vendor-bbl false > "$stdout.cwd-clean" 2> "$stderr.cwd-clean")
        (cd "$poison_dir" && "$binary_from_cwd" --gcode-timing "$ORIGINAL" "$OPTIMIZED" --printer-vendor-bbl false > "$stdout.cwd-poison" 2> "$stderr.cwd-poison")
        test ! -s "$stderr.cwd-clean"
        test ! -s "$stderr.cwd-poison"
        assert_schema "$stdout.cwd-clean"
        assert_schema "$stdout.cwd-poison"
        python3 - "$stdout.cwd-clean" "$stdout.cwd-poison" <<'PY'
import json, sys
clean, poison = (json.load(open(path)) for path in sys.argv[1:])
assert clean["original_seconds"] == poison["original_seconds"], (clean, poison)
assert clean["optimized_seconds"] == poison["optimized_seconds"], (clean, poison)
PY
    fi
    if [ "$label" = ORCA ]; then
        # Orca's desktop importer gives non-BBL G29 a 260-second allowance.
        # Omitting the selected preset context silently loses that time.
        python3 - "$binary" "$ORIGINAL" "$WORKDIR" <<'PY_ORCA_CONTEXT'
import json, math, pathlib, subprocess, sys
binary, original, directory = sys.argv[1:]
fixture = pathlib.Path(directory) / "orca-g29.gcode"
fixture.write_text(pathlib.Path(original).read_text() + "G29\n")
seconds = {}
for bbl in ("true", "false"):
    result = subprocess.run([binary, "--gcode-timing", str(fixture), str(fixture), "--printer-vendor-bbl", bbl],
                            capture_output=True, text=True, check=True, timeout=10)
    assert not result.stderr
    data = json.loads(result.stdout)
    assert data["original_seconds"] == data["optimized_seconds"]
    seconds[bbl] = data["original_seconds"]
assert math.isclose(seconds["false"] - seconds["true"], 260.0, rel_tol=0, abs_tol=1e-4), seconds
PY_ORCA_CONTEXT
    fi
    index=$((index + 1))
done

python3 - "$ORIGINAL" "$OPTIMIZED" > "$WORKDIR/hashes.after" <<'PY'
import hashlib, pathlib, sys
for name in sys.argv[1:]:
    print(hashlib.sha256(pathlib.Path(name).read_bytes()).hexdigest())
PY
cmp "$WORKDIR/hashes.before" "$WORKDIR/hashes.after"
echo "PASS: existing-G-code timing is native, strict JSON, and leaves input fixtures unchanged"
