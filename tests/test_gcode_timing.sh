#!/usr/bin/env bash
# Existing-G-code timing is a strict one-document command on all engine builds.
# Usage: tests/test_gcode_timing.sh --engine bambu|orca|snapmaker BIN
#    or: tests/test_gcode_timing.sh BAMBU_BIN [ORCA_BIN [SNAPMAKER_BIN]]
#
# The inputs are real engine output: each binary slices its own calibration
# fixture (tests/fixtures/calib_base*.3mf, the files the other tests slice) and
# the G-code it wrote is timed. Nothing here is hand-written G-code.
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
SCHEMA="$SCRIPT_DIR/../schemas/gcode-timing.schema.json"
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
base_doc = {"schema_version": 1, "engine": "bambustudio", "engine_revision": "test"}
for seconds in (1e-12, 1.0, 10000.5):
    assert valid(dict(base_doc, original_seconds=seconds, optimized_seconds=seconds))
for seconds in (0, -1):
    assert not valid(dict(base_doc, original_seconds=seconds, optimized_seconds=1))
assert valid(dict(base_doc, error_code="TIMING_UNAVAILABLE"))
assert not valid(dict(base_doc, original_seconds=1, optimized_seconds=1, error_code="TIMING_UNAVAILABLE"))
assert not valid(dict(base_doc, engine="BAMBU", error_code="TIMING_UNAVAILABLE"))
PY_SCHEMA

# json_name <label> -> the engine name the document carries (result.json's).
json_name() {
    case "$1" in bambu) echo bambustudio ;; orca) echo orcaslicer ;; snapmaker) echo snapmaker ;; esac
}

# run_timing <bin> <label> <original> <optimized> <stdout> <stderr> [extra...]
# The calibration fixtures are Bambu Lab X1 Carbon projects, a BBL printer.
run_timing() {
    local binary="$1" label="$2" original="$3" optimized="$4" stdout="$5" stderr="$6"
    shift 6
    if [ "$label" != bambu ]; then
        "$binary" --gcode-timing "$original" "$optimized" --printer-vendor-bbl true "$@" > "$stdout" 2> "$stderr"
    else
        "$binary" --gcode-timing "$original" "$optimized" "$@" > "$stdout" 2> "$stderr"
    fi
}

# expect_error <bin> <label> <rc> <code> <args...>: the refusal is one schema-valid
# document with that code and nothing on stderr, and the process exits <rc>.
expect_error() {
    local binary="$1" label="$2" want_rc="$3" code="$4"
    shift 4
    local out="$WORKDIR/err.$code.$RANDOM"
    set +e
    "$binary" "$@" > "$out" 2> "$out.stderr"
    local rc=$?
    set -e
    test "$rc" -eq "$want_rc"
    test ! -s "$out.stderr"
    test "$(wc -l < "$out" | tr -d ' ')" = 1
    python3 "$SCRIPT_DIR/tools/check_json_schema.py" "$SCHEMA" "$out" >&2
    python3 - "$out" "$code" "$(json_name "$label")" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))
assert d["error_code"] == sys.argv[2], d
assert d["engine"] == sys.argv[3], d
PY
    echo "$out"
}

index=0
for binary in "${BINARIES[@]}"; do
    label="${LABELS[$index]}"
    case "$label" in
        bambu) fixture="$SCRIPT_DIR/fixtures/calib_base.3mf" ;;
        orca) fixture="$SCRIPT_DIR/fixtures/calib_base_orca.3mf" ;;
        snapmaker) fixture="$SCRIPT_DIR/fixtures/calib_base_snapmaker.3mf" ;;
    esac
    engine_dir="$WORKDIR/$label"
    mkdir -p "$engine_dir/slice"

    # Real G-code from this binary. The embedded estimate is what the slice
    # path's processor computed over the same bytes the importer now reads.
    "$binary" "$fixture" --slice 1 --outputdir "$engine_dir/slice" > "$engine_dir/slice.log" 2>&1
    ORIGINAL="$(find "$engine_dir/slice" -name '*.gcode' | head -1)"
    test -s "$ORIGINAL"
    # The optimized file is the same real G-code with every feed rate doubled:
    # derived from engine output, never typed in.
    OPTIMIZED="$engine_dir/optimized.gcode"
    python3 - "$ORIGINAL" "$OPTIMIZED" <<'PY'
import re, sys
src, dst = sys.argv[1:]
with open(src, "rb") as f:
    lines = f.read().split(b"\n")
out = []
for line in lines:
    if line.startswith((b"G1 ", b"G0 ")):
        line = re.sub(rb"\bF(\d+(?:\.\d+)?)", lambda m: b"F%d" % (int(float(m.group(1))) * 2), line)
    out.append(line)
with open(dst, "wb") as f:
    f.write(b"\n".join(out))
PY
    python3 - "$ORIGINAL" "$OPTIMIZED" > "$engine_dir/hashes.before" <<'PY'
import hashlib, pathlib, sys
for name in sys.argv[1:]:
    print(hashlib.sha256(pathlib.Path(name).read_bytes()).hexdigest())
PY

    stdout="$engine_dir/stdout"
    stderr="$engine_dir/stderr"
    run_timing "$binary" "$label" "$ORIGINAL" "$OPTIMIZED" "$stdout" "$stderr"
    test ! -s "$stderr"
    test "$(wc -l < "$stdout" | tr -d ' ')" = 1
    python3 "$SCRIPT_DIR/tools/check_json_schema.py" "$SCHEMA" "$stdout"
    python3 - "$stdout" "$(json_name "$label")" <<'PY'
import json, math, sys
d = json.load(open(sys.argv[1]))
assert d["engine"] == sys.argv[2], d
assert d["engine_revision"] != "unknown", d
assert math.isfinite(d["original_seconds"]) and d["original_seconds"] > 0, d
assert math.isfinite(d["optimized_seconds"]) and d["optimized_seconds"] > 0, d
# Doubled feed rates print faster; the native processor read both files.
assert d["optimized_seconds"] < d["original_seconds"], d
PY

    # Parity with the slice path: the same file as both inputs gives the same
    # two numbers, and they agree with the estimate the engine wrote into the
    # G-code when it sliced ("total estimated time" on Bambu and Orca,
    # "estimated printing time (normal mode)" on Snapmaker). The header is in
    # whole seconds; the Snapmaker pin's import reads about 2 s over its own
    # header, so the bound is half a percent.
    run_timing "$binary" "$label" "$ORIGINAL" "$ORIGINAL" "$stdout.same" "$stderr.same"
    test ! -s "$stderr.same"
    python3 - "$stdout.same" "$ORIGINAL" <<'PY'
import json, re, sys
d = json.load(open(sys.argv[1]))
assert d["original_seconds"] == d["optimized_seconds"], d
header = None
with open(sys.argv[2], "rb") as f:
    for line in f:
        if not line.startswith(b";"):
            continue
        m = re.search(rb"(?:total estimated time|estimated printing time \(normal mode\))\s*[=:]\s*([0-9dhms ]+)", line)
        if m:
            header = m.group(1).decode().strip()
            break
assert header, "no estimated time header in the sliced G-code"
seconds = 0
for value, unit in re.findall(r"(\d+)\s*([dhms])", header):
    seconds += int(value) * {"d": 86400, "h": 3600, "m": 60, "s": 1}[unit]
assert seconds > 0, header
assert abs(d["original_seconds"] - seconds) <= max(1.0, 0.005 * seconds), (d["original_seconds"], header)
PY

    # Swapping the pair swaps the numbers: neither import leaks into the other.
    run_timing "$binary" "$label" "$OPTIMIZED" "$ORIGINAL" "$stdout.swapped" "$stderr.swapped"
    python3 - "$stdout" "$stdout.swapped" <<'PY'
import json, sys
a = json.load(open(sys.argv[1])); b = json.load(open(sys.argv[2]))
assert a["original_seconds"] == b["optimized_seconds"] and a["optimized_seconds"] == b["original_seconds"], (a, b)
PY

    # Paths are UTF-8 on every platform, including the Windows wide argv path.
    unicode_dir="$engine_dir/模型 with spaces"
    mkdir -p "$unicode_dir"
    cp "$ORIGINAL" "$unicode_dir/original.gcode"
    cp "$OPTIMIZED" "$unicode_dir/optimized.gcode"
    run_timing "$binary" "$label" "$unicode_dir/original.gcode" "$unicode_dir/optimized.gcode" "$stdout.unicode" "$stderr.unicode"
    test ! -s "$stderr.unicode"
    cmp "$stdout" "$stdout.unicode"

    # Missing, empty, directory, and FIFO inputs are refused as INPUT_UNREADABLE
    # before any native importer sees them, and the path never leaks.
    context=()
    [ "$label" != bambu ] && context=(--printer-vendor-bbl true)
    empty="$engine_dir/empty.gcode"; : > "$empty"
    missing="$engine_dir/not-a-gcode"
    for bad in "$empty" "$engine_dir" "$missing"; do
        out="$(expect_error "$binary" "$label" 1 INPUT_UNREADABLE --gcode-timing "$bad" "$OPTIMIZED" "${context[@]}")"
        if grep -qF "$bad" "$out"; then echo "path leaked into the response"; exit 1; fi
    done
    # Windows has no FIFOs; Git Bash's mkfifo makes an emulated file that the
    # native program sees as an ordinary (and sometimes readable) file.
    if command -v mkfifo > /dev/null && [[ "$(uname -s)" != MINGW* && "$(uname -s)" != MSYS* && "$(uname -s)" != CYGWIN* ]]; then
        fifo="$engine_dir/input.fifo"; mkfifo "$fifo"
        expect_error "$binary" "$label" 1 INPUT_UNREADABLE --gcode-timing "$fifo" "$OPTIMIZED" "${context[@]}" > /dev/null
    fi

    # A timing-looking word consumed as an option value, or after --, stays on
    # the normal parser path and gets that path's own refusal, never this
    # mode's document.
    set +e
    "$binary" --printer-preset --gcode-timing > "$stdout.literal-preset" 2> "$stderr.literal-preset"
    rc=$?
    set -e
    test "$rc" -eq 1
    if grep -q '"error_code"' "$stdout.literal-preset"; then echo "timing document on the parser path"; exit 1; fi
    grep -Fq "has no printer preset named '--gcode-timing'" "$stderr.literal-preset"
    set +e
    "$binary" -- --gcode-timing > "$stdout.literal-after-double-dash" 2> "$stderr.literal-after-double-dash"
    rc=$?
    set -e
    test "$rc" -eq 1
    if grep -q '"error_code"' "$stdout.literal-after-double-dash"; then echo "timing document on the parser path"; exit 1; fi
    grep -Fq 'Unsupported file format.' "$stderr.literal-after-double-dash"

    # With stdin and stderr closed by the launcher the saved streams must not
    # land on fd 2; the command answers UNAVAILABLE as one document, exit 1.
    # On Windows the CRT's execv is a spawn, so the launcher waits on the
    # child itself and relays its status.
    python3 - "$binary" "$ORIGINAL" "$OPTIMIZED" "$stdout.closed-fds" "${context[@]}" <<'PY_CLOSED_FDS'
import pathlib, subprocess, sys
binary, original, optimized, stdout, *context = sys.argv[1:]
launcher = """import os, sys
for fd in (0, 2):
    try:
        os.close(fd)
    except OSError:
        pass
if os.name == "nt":
    import _winapi
    handle = os.spawnv(os.P_NOWAIT, sys.argv[1], sys.argv[1:])
    try:
        if _winapi.WaitForSingleObject(handle, 60000) == _winapi.WAIT_TIMEOUT:
            _winapi.TerminateProcess(handle, 124)
            _winapi.WaitForSingleObject(handle, _winapi.INFINITE)
            status = 124
        else:
            status = _winapi.GetExitCodeProcess(handle)
    finally:
        _winapi.CloseHandle(handle)
    os._exit(status if 0 <= status <= 255 else 126)
os.execv(sys.argv[1], sys.argv[1:])
"""
result = subprocess.run([sys.executable, "-c", launcher, binary, "--gcode-timing", original, optimized, *context],
                        text=True, capture_output=True, timeout=120)
pathlib.Path(stdout).write_text(result.stdout)
assert result.returncode == 1, result.returncode
assert not result.stderr, result.stderr
PY_CLOSED_FDS
    python3 "$SCRIPT_DIR/tools/check_json_schema.py" "$SCHEMA" "$stdout.closed-fds"
    python3 - "$stdout.closed-fds" <<'PY'
import json, sys
assert json.load(open(sys.argv[1]))["error_code"] == "UNAVAILABLE"
PY

    # Any other argument is a conflict: the two paths must come first and alone.
    expect_error "$binary" "$label" 64 CONFLICTING_ARGUMENTS --gcode-timing "$ORIGINAL" "$OPTIMIZED" "${context[@]}" --slice 1 > /dev/null
    expect_error "$binary" "$label" 64 CONFLICTING_ARGUMENTS --slice 1 --gcode-timing "$ORIGINAL" "$OPTIMIZED" "${context[@]}" > /dev/null
    expect_error "$binary" "$label" 64 CONFLICTING_ARGUMENTS --gcode-timing "$ORIGINAL" > /dev/null
    if [ "$label" = bambu ]; then
        expect_error "$binary" "$label" 64 CONFLICTING_ARGUMENTS --gcode-timing "$ORIGINAL" "$OPTIMIZED" --printer-vendor-bbl true > /dev/null
    else
        out="$(expect_error "$binary" "$label" 64 PRINTER_CONTEXT_REQUIRED --gcode-timing "$ORIGINAL" "$OPTIMIZED")"
        if grep -q printer_vendor_bbl "$out"; then echo "unexpected field in the refusal"; exit 1; fi
        expect_error "$binary" "$label" 64 CONFLICTING_ARGUMENTS --gcode-timing "$ORIGINAL" "$OPTIMIZED" --printer-vendor-bbl maybe > /dev/null
        expect_error "$binary" "$label" 64 CONFLICTING_ARGUMENTS --gcode-timing "$ORIGINAL" "$OPTIMIZED" --printer-vendor-bbl > /dev/null
        # The non-BBL context is accepted and times the same real file.
        "$binary" --gcode-timing "$ORIGINAL" "$ORIGINAL" --printer-vendor-bbl false > "$stdout.nonbbl" 2> "$stderr.nonbbl"
        test ! -s "$stderr.nonbbl"
        python3 "$SCRIPT_DIR/tools/check_json_schema.py" "$SCHEMA" "$stdout.nonbbl"
        # The import reads the executable's resources (info/nozzle_info.json via
        # Print::get_hrc_by_nozzle_type), never the working directory's.
        clean_dir="$engine_dir/clean"; poison_dir="$engine_dir/poison"
        mkdir -p "$clean_dir" "$poison_dir/info"
        printf '%s\n' '{"nozzle_hrc":{"undefine":"invalid"}}' > "$poison_dir/info/nozzle_info.json"
        binary_abs="$(cd "$(dirname "$binary")" && pwd -P)/$(basename "$binary")"
        (cd "$clean_dir" && "$binary_abs" --gcode-timing "$ORIGINAL" "$OPTIMIZED" --printer-vendor-bbl false > "$stdout.cwd-clean" 2> "$stderr.cwd-clean")
        (cd "$poison_dir" && "$binary_abs" --gcode-timing "$ORIGINAL" "$OPTIMIZED" --printer-vendor-bbl false > "$stdout.cwd-poison" 2> "$stderr.cwd-poison")
        test ! -s "$stderr.cwd-clean"
        test ! -s "$stderr.cwd-poison"
        python3 "$SCRIPT_DIR/tools/check_json_schema.py" "$SCHEMA" "$stdout.cwd-clean"
        cmp "$stdout.cwd-clean" "$stdout.cwd-poison"
    fi
    if [ "$label" = orca ]; then
        # Orca's desktop importer gives non-BBL G29 a 260-second allowance
        # (GCodeProcessor.cpp 4859-4870 at the pin). The selected preset
        # context decides it; omitting it would silently lose that time. The
        # file's own printer_model would refine the context (GCodeProcessor.cpp
        # 2632), so the probe is the real G-code without that one line.
        python3 - "$binary" "$ORIGINAL" "$engine_dir" <<'PY_ORCA_CONTEXT'
import json, math, pathlib, subprocess, sys
binary, original, directory = sys.argv[1:]
fixture = pathlib.Path(directory) / "orca-g29.gcode"
lines = [l for l in pathlib.Path(original).read_bytes().split(b"\n") if not l.startswith(b"; printer_model")]
fixture.write_bytes(b"\n".join(lines) + b"\nG29\n")
seconds = {}
for bbl in ("true", "false"):
    result = subprocess.run([binary, "--gcode-timing", str(fixture), str(fixture), "--printer-vendor-bbl", bbl],
                            capture_output=True, text=True, check=True, timeout=60)
    assert not result.stderr
    data = json.loads(result.stdout)
    assert data["original_seconds"] == data["optimized_seconds"]
    seconds[bbl] = data["original_seconds"]
assert math.isclose(seconds["false"] - seconds["true"], 260.0, rel_tol=0, abs_tol=1e-4), seconds
PY_ORCA_CONTEXT
    fi

    python3 - "$ORIGINAL" "$OPTIMIZED" > "$engine_dir/hashes.after" <<'PY'
import hashlib, pathlib, sys
for name in sys.argv[1:]:
    print(hashlib.sha256(pathlib.Path(name).read_bytes()).hexdigest())
PY
    cmp "$engine_dir/hashes.before" "$engine_dir/hashes.after"
    echo "PASS: $label: existing-G-code timing is native, strict JSON, matches the slice estimate and leaves its inputs unchanged"
    index=$((index + 1))
done
echo "PASS: existing-G-code timing is native, strict JSON, and leaves input files unchanged"
