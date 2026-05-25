#!/usr/bin/env bash
# Regression tests for excluded features.
#
# Pins the user-visible behaviour of every source file excluded from the
# slicer_cli build. Excluded features must fail loudly — not silently
# mis-process. This script runs as part of CI after the build succeeds.
#
# Usage:
#   tests/test_excluded_features.sh [path-to-slicer_cli]
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

# ── Summary ─────────────────────────────────────────────────────────────────
echo ""
echo "Results: $PASS passed, $FAIL failed"
[ $FAIL -eq 0 ]   # exit 0 iff all passed
