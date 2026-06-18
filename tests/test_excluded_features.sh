#!/usr/bin/env bash
# Regression tests for excluded features.
#
# Pins the user-visible behaviour of every source file excluded from the
# slicer_cli build. Excluded features must fail loudly — not silently
# mis-process. This script runs as part of CI after the build succeeds.
#
# Usage:
#   cli/tests/test_excluded_features.sh [path-to-slicer_cli]
#
# Defaults to slicer_cli on PATH; set $1 to an explicit binary path.
#
# Exit code: 0 = all tests passed. Non-zero = one or more failures.

set -euo pipefail

BINARY="${1:-slicer_cli}"

PASS=0
FAIL=0

check() {
    local LABEL="$1"; shift
    local EXPECTED_EXIT="$1"; shift
    local EXPECTED_OUTPUT="$1"; shift

    set +e
    OUTPUT=$("$BINARY" "$@" 2>&1)
    ACTUAL_EXIT=$?
    set -e

    local STATUS="PASS"
    if [ "$ACTUAL_EXIT" -eq "$EXPECTED_EXIT" ] && echo "$OUTPUT" | grep -q "$EXPECTED_OUTPUT"; then
        PASS=$((PASS + 1))
    else
        STATUS="FAIL"
        FAIL=$((FAIL + 1))
        echo "FAIL [$LABEL]"
        echo "  Expected exit=$EXPECTED_EXIT, got $ACTUAL_EXIT"
        echo "  Expected output containing: $EXPECTED_OUTPUT"
        echo "  Actual output: $OUTPUT"
        return
    fi
    echo "PASS [$LABEL]"
}

# Resolve fixtures dir relative to this script so the tests work from any CWD.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FIXTURES="$SCRIPT_DIR/fixtures"

# run_slice <out.gcode> <args...> → captures combined output in LAST_OUTPUT and
# exit code in LAST_EXIT. Never aborts the script (set +e around the call).
LAST_OUTPUT=""
LAST_EXIT=0
run_slice() {
    local OUT="$1"; shift
    set +e
    LAST_OUTPUT=$("$BINARY" "$@" -o "$OUT" 2>&1)
    LAST_EXIT=$?
    set -e
}

# Portable temp G-code path. mktemp requires the X's to be the LAST characters
# of the template on both BSD (macOS) and GNU (Linux) — a ".gcode" suffix after
# the X's is treated literally and collides on the second call.
mktmp_gcode() { mktemp "${TMPDIR:-/tmp}/calib_cli.XXXXXX"; }

# pass/fail bookkeeping for the gcode-content assertions.
record() {
    local LABEL="$1"; local OK="$2"; local DETAIL="${3:-}"
    if [ "$OK" = "1" ]; then
        PASS=$((PASS + 1)); echo "PASS [$LABEL]"
    else
        FAIL=$((FAIL + 1)); echo "FAIL [$LABEL]"; [ -n "$DETAIL" ] && echo "  $DETAIL"
    fi
}

# ── Binary launches ─────────────────────────────────────────────────────────
check "binary-launches"           0 "Usage"    --help
# --version is not yet implemented; the binary exits non-zero with "Unknown option".
# Test that it at least doesn't segfault (any clean exit is fine).
check "unknown-flag-no-crash"     1 ""         --version

# ── Unsupported file formats (SLA / other) ──────────────────────────────────
SL1_TMP=$(mktemp /tmp/test_XXXXXX.sl1)
echo "fake SLA file" > "$SL1_TMP"
check "sl1-rejected"          1 "Unsupported file format"   "$SL1_TMP"
rm -f "$SL1_TMP"

XYZ_TMP=$(mktemp /tmp/test_XXXXXX.xyz)
echo "fake xyz" > "$XYZ_TMP"
check "xyz-rejected"          1 "Unsupported file format"   "$XYZ_TMP"
rm -f "$XYZ_TMP"

# ── Missing-input rejection ─────────────────────────────────────────────────
check "no-input"              1 ""   # no args → non-zero exit, message doesn't matter

# ── Excluded flags (PostProcessor, GCodeSender) ─────────────────────────────
# These flags do not exist. The binary must reject them, not silently ignore.
check "no-post-process-flag"  1 ""   --post-process /tmp/fake_script.sh /dev/null
check "no-send-to-printer"    1 ""   --send-to-printer 192.168.1.1 /dev/null

# ── #4: unbound `initial_no_support_filament_id` placeholder normalization ───
# Neither BambuStudio nor OrcaSlicer bind this token (only initial_no_support_
# {tool,extruder,hotend}). A 3MF whose custom machine_start_gcode references it
# makes the PlaceholderParser throw at export. The driver aliases it to the
# bound `initial_no_support_extruder` (always on; --no-normalize-legacy-gcode
# opts out).
TOKEN="initial_no_support_filament_id"
TOKEN_3MF="$FIXTURES/legacy_token.3mf"
BASE_3MF="$FIXTURES/calib_base.3mf"

if [ -f "$TOKEN_3MF" ]; then
    # (1) Default: slice succeeds, an audit notice is emitted, and the literal
    #     token is gone from the exported G-code.
    GC=$(mktmp_gcode)
    run_slice "$GC" "$TOKEN_3MF"
    if [ "$LAST_EXIT" -eq 0 ] \
       && echo "$LAST_OUTPUT" | grep -q "LegacyGcodeTokenAliased" \
       && ! grep -q "$TOKEN" "$GC"; then
        record "legacy-token-normalized" 1
    else
        record "legacy-token-normalized" 0 "exit=$LAST_EXIT (want 0); notice/token check failed"
    fi
    rm -f "$GC"

    # (2) Opt-out reproduces the original failure: PlaceholderParser rejects the
    #     unbound token and export aborts.
    GC=$(mktmp_gcode)
    run_slice "$GC" "$TOKEN_3MF" --no-normalize-legacy-gcode
    if [ "$LAST_EXIT" -ne 0 ] && echo "$LAST_OUTPUT" | grep -q "Not a variable name"; then
        record "legacy-token-reproduces-without-flag" 1
    else
        record "legacy-token-reproduces-without-flag" 0 "exit=$LAST_EXIT (want non-zero); parser error not seen"
    fi
    rm -f "$GC"
else
    echo "SKIP [legacy-token-*] fixture missing: $TOKEN_3MF"
fi

if [ -f "$BASE_3MF" ]; then
    # (3) Guard: a config WITHOUT the legacy token is untouched — no rewrite
    #     notice, clean slice. Proves normalization is whole-token and inert
    #     when the token is absent (e.g. the separately-bound initial_filament_id
    #     is never rewritten).
    GC=$(mktmp_gcode)
    run_slice "$GC" "$BASE_3MF"
    if [ "$LAST_EXIT" -eq 0 ] && ! echo "$LAST_OUTPUT" | grep -q "LegacyGcodeTokenAliased"; then
        record "legacy-token-absent-untouched" 1
    else
        record "legacy-token-absent-untouched" 0 "exit=$LAST_EXIT (want 0); unexpected rewrite notice"
    fi
    rm -f "$GC"
else
    echo "SKIP [legacy-token-absent-untouched] fixture missing: $BASE_3MF"
fi

# ── #5: --calib-mode calibration flags ──────────────────────────────────────
# The engine provides CalibMode/Calib_Params/Print::set_calib_params and the
# per-layer emission; the driver only parses flags, sets params, and (for the
# pattern mode) ports the GUI-free geometry generator. Each mode slices the base
# 3MF and greps the exported G-code for its signature.
# NB: swallow grep's exit 1 on zero matches BEFORE the pipe, or `set -o pipefail`
# would make `N=$(distinct_count …)` abort the whole script on a real regression
# (zero matches) instead of letting the assertion record a FAIL.
distinct_count() { { grep -oE "$2" "$1" 2>/dev/null || true; } | sort -u | wc -l | tr -d ' '; }
# `local n=$(…)` masks grep's exit-1-on-zero-match (local's own status is 0, so
# set -e doesn't abort), yielding a single clean integer ("0" on no match) —
# unlike grep -c's "0" plus a `|| echo 0` which would print "0\n0".
total_count()    { local n="$(grep -c "$2" "$1" 2>/dev/null)"; echo "${n:-0}"; }

# Negative: an unknown mode must fail fast WITH the mode-rejection message, so
# the test can't pass for an unrelated reason (e.g. a missing fixture).
GC=$(mktmp_gcode)
run_slice "$GC" "$BASE_3MF" --calib-mode bogus
if [ "$LAST_EXIT" -ne 0 ] && echo "$LAST_OUTPUT" | grep -q "calib-mode"; then
    record "calib-bogus-rejected" 1
else
    record "calib-bogus-rejected" 0 "exit=$LAST_EXIT (want non-zero + 'calib-mode' in output)"
fi
rm -f "$GC"

if [ -f "$BASE_3MF" ]; then
    # temp_tower: stepped set-temperatures (M104/M109) descending across the sweep.
    GC=$(mktmp_gcode)
    run_slice "$GC" "$BASE_3MF" --calib-mode temp_tower --calib-start 240 --calib-end 190 --calib-step 5
    N=$(distinct_count "$GC" "M10[49] S[0-9]+")
    [ "$LAST_EXIT" -eq 0 ] && [ "${N:-0}" -ge 3 ] \
        && record "calib-temp-tower" 1 \
        || record "calib-temp-tower" 0 "exit=$LAST_EXIT distinct-temps=$N (want exit 0, >=3)"
    rm -f "$GC"

    # retraction_tower: the engine emits a per-Z marker comment.
    GC=$(mktmp_gcode)
    run_slice "$GC" "$BASE_3MF" --calib-mode retraction_tower --calib-start 0 --calib-end 2 --calib-step 0.1
    N=$(total_count "$GC" "Calib_Retraction_tower: Z_HEIGHT")
    [ "$LAST_EXIT" -eq 0 ] && [ "${N:-0}" -ge 1 ] \
        && record "calib-retraction-tower" 1 \
        || record "calib-retraction-tower" 0 "exit=$LAST_EXIT markers=$N (want exit 0, >=1)"
    rm -f "$GC"

    # pressure_advance_tower: varying pressure-advance (M900 K) up the tower.
    GC=$(mktmp_gcode)
    run_slice "$GC" "$BASE_3MF" --calib-mode pressure_advance_tower --calib-start 0 --calib-end 0.1 --calib-step 0.002
    N=$(distinct_count "$GC" "M900 K[0-9.]+")
    [ "$LAST_EXIT" -eq 0 ] && [ "${N:-0}" -ge 3 ] \
        && record "calib-pa-tower" 1 \
        || record "calib-pa-tower" 0 "exit=$LAST_EXIT distinct-M900=$N (want exit 0, >=3)"
    rm -f "$GC"

    # pressure_advance_line: engine builds the line test internally (M900 sweep).
    GC=$(mktmp_gcode)
    run_slice "$GC" "$BASE_3MF" --calib-mode pressure_advance_line --calib-start 0 --calib-end 0.1 --calib-step 0.002
    N=$(distinct_count "$GC" "M900 K[0-9.]+")
    [ "$LAST_EXIT" -eq 0 ] && [ "${N:-0}" -ge 2 ] \
        && record "calib-pa-line" 1 \
        || record "calib-pa-line" 0 "exit=$LAST_EXIT distinct-M900=$N (want exit 0, >=2)"
    rm -f "$GC"

    # pressure_advance_pattern: driver-generated geometry. The loaded model is
    # replaced by the synthesized handle cube; config comes from the 3MF. Look for
    # the pattern layer marker (calib.cpp) + M900.
    GC=$(mktmp_gcode)
    run_slice "$GC" "$BASE_3MF" --calib-mode pressure_advance_pattern --calib-start 0 --calib-end 0.08 --calib-step 0.005
    M=$(total_count "$GC" "start pressure advance pattern for layer")
    K=$(total_count "$GC" "M900")
    [ "$LAST_EXIT" -eq 0 ] && [ "${M:-0}" -ge 1 ] && [ "${K:-0}" -ge 1 ] \
        && record "calib-pa-pattern" 1 \
        || record "calib-pa-pattern" 0 "exit=$LAST_EXIT pattern-markers=$M M900=$K (want exit 0, both >=1)"
    rm -f "$GC"

    # Negative: a reversed PA sweep (start > end) must be rejected before slicing
    # — the engine's unsigned pattern-count loop would otherwise wrap unbounded.
    GC=$(mktmp_gcode)
    run_slice "$GC" "$BASE_3MF" --calib-mode pressure_advance_pattern --calib-start 0.1 --calib-end 0 --calib-step 0.005
    if [ "$LAST_EXIT" -ne 0 ] && echo "$LAST_OUTPUT" | grep -q "greater than"; then
        record "calib-pa-reversed-rejected" 1
    else
        record "calib-pa-reversed-rejected" 0 "exit=$LAST_EXIT (want non-zero + ascending-sweep error)"
    fi
    rm -f "$GC"

    # temp_tower descends from start (engine hook), so an ascending range is a
    # misconfiguration and must be rejected.
    GC=$(mktmp_gcode)
    run_slice "$GC" "$BASE_3MF" --calib-mode temp_tower --calib-start 190 --calib-end 240 --calib-step 5
    if [ "$LAST_EXIT" -ne 0 ] && echo "$LAST_OUTPUT" | grep -q "descends"; then
        record "calib-temp-ascending-rejected" 1
    else
        record "calib-temp-ascending-rejected" 0 "exit=$LAST_EXIT (want non-zero + descends error)"
    fi
    rm -f "$GC"

    # --calib-extruder-id is only honored by pressure_advance_pattern; a nonzero
    # id on another mode must be rejected (it would calibrate the wrong extruder).
    GC=$(mktmp_gcode)
    run_slice "$GC" "$BASE_3MF" --calib-mode temp_tower --calib-start 240 --calib-end 190 --calib-step 5 --calib-extruder-id 1
    if [ "$LAST_EXIT" -ne 0 ] && echo "$LAST_OUTPUT" | grep -q "only honored"; then
        record "calib-extruder-id-rejected-non-pattern" 1
    else
        record "calib-extruder-id-rejected-non-pattern" 0 "exit=$LAST_EXIT (want non-zero + only-honored error)"
    fi
    rm -f "$GC"

    # retraction_tower ascends from start (engine hook), so a descending range is
    # a misconfiguration and must be rejected.
    GC=$(mktmp_gcode)
    run_slice "$GC" "$BASE_3MF" --calib-mode retraction_tower --calib-start 2 --calib-end 0 --calib-step 0.1
    if [ "$LAST_EXIT" -ne 0 ] && echo "$LAST_OUTPUT" | grep -q "ascends"; then
        record "calib-retraction-descending-rejected" 1
    else
        record "calib-retraction-descending-rejected" 0 "exit=$LAST_EXIT (want non-zero + ascends error)"
    fi
    rm -f "$GC"

    # pressure_advance_pattern requires a .3mf --input (it discards the model but
    # needs the embedded config + plate setup). An STL must be rejected.
    STL_TMP=$(mktemp "${TMPDIR:-/tmp}/calib_stl.XXXXXX"); mv "$STL_TMP" "$STL_TMP.stl"; STL_TMP="$STL_TMP.stl"
    printf 'solid x\nendsolid x\n' > "$STL_TMP"
    GC=$(mktmp_gcode)
    run_slice "$GC" "$STL_TMP" --calib-mode pressure_advance_pattern --calib-start 0 --calib-end 0.08 --calib-step 0.005
    if [ "$LAST_EXIT" -ne 0 ] && echo "$LAST_OUTPUT" | grep -q "requires a .3mf"; then
        record "calib-pa-pattern-stl-rejected" 1
    else
        record "calib-pa-pattern-stl-rejected" 0 "exit=$LAST_EXIT (want non-zero + 3mf-required error)"
    fi
    rm -f "$GC" "$STL_TMP"
else
    echo "SKIP [calib-*] fixture missing: $BASE_3MF"
fi

# ── Summary ─────────────────────────────────────────────────────────────────
echo ""
echo "Results: $PASS passed, $FAIL failed"
[ $FAIL -eq 0 ]   # exit 0 iff all passed
