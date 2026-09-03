#!/usr/bin/env bash
# Regression tests for the structured-diagnostics event stream.
#
# Pins two halves of one contract:
#
#   1. The SLICE path emits `[[SLICER_EVENT]] {json}` lines for engine
#      diagnostics that were previously plain text (or nothing at all), and
#      emitting them does NOT change the exit code — a run that warned and
#      still produced G-code keeps exiting 0. Warnings are an information
#      stream, never a refusal.
#   2. The strict-JSON paths (`layout capabilities`, `--layout-plan`) still
#      write ONE JSON document to stdout and nothing else. The diagnostics
#      bridge is installed after those early-returns for exactly this reason;
#      a single event line prepended to that document breaks every caller
#      that parses stdout as one document.
#
# Usage:
#   tests/test_diagnostic_events.sh [path-to-slicer_cli]
#
# Defaults to slicer_cli on PATH; set $1 to an explicit binary path.
#
# Exit code: 0 = all tests passed. Non-zero = one or more failures.

set -euo pipefail

BINARY="${1:-slicer_cli}"

PASS=0
FAIL=0

record() {
    local LABEL="$1"; local OK="$2"; local DETAIL="${3:-}"
    if [ "$OK" = "1" ]; then
        PASS=$((PASS + 1)); echo "PASS [$LABEL]"
    else
        FAIL=$((FAIL + 1)); echo "FAIL [$LABEL]"; [ -n "$DETAIL" ] && echo "  $DETAIL"
    fi
    return 0
}

# Same mktemp caveat as test_excluded_features.sh: the X's must be last on
# both BSD (macOS) and GNU (Linux).
mktmp_gcode() { mktemp "${TMPDIR:-/tmp}/diag_cli.XXXXXX"; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FIXTURES="$SCRIPT_DIR/fixtures"
BASE_3MF="$FIXTURES/calib_base.3mf"

# run <stdout-file> <stderr-file> <args...> — never aborts the script.
LAST_EXIT=0
run() {
    local OUT="$1"; shift
    local ERR="$1"; shift
    set +e
    "$BINARY" "$@" > "$OUT" 2> "$ERR"
    LAST_EXIT=$?
    set -e
}

# has_event <file> <event-name>  → grep for one event kind on the event stream.
has_event() { grep -q "\[\[SLICER_EVENT\]\].*\"event\":\"$2\"" "$1"; }
# no_events <file> → the file carries no event lines at all.
no_events()  { ! grep -q "\[\[SLICER_EVENT\]\]" "$1"; }

OUT=$(mktemp "${TMPDIR:-/tmp}/diag_out.XXXXXX")
ERR=$(mktemp "${TMPDIR:-/tmp}/diag_err.XXXXXX")
trap 'rm -f "$OUT" "$ERR"' EXIT

# ── Slice path: engine log records reach the event stream ───────────────────
# A missing input makes libslic3r log `[error] Unable to open the file ...`
# through BOOST_LOG_TRIVIAL, which had no structured representation at all.
# The driver's own load failure becomes an event too. Exit code unchanged (1).
run "$OUT" "$ERR" "$FIXTURES/does_not_exist.3mf" -o /dev/null
if [ "$LAST_EXIT" -eq 1 ] \
   && has_event "$OUT" "engine_log" \
   && grep -q "\"severity\":\"error\"" "$OUT" \
   && has_event "$OUT" "load_error"; then
    record "slice-missing-input-emits-engine-log-and-load-error" 1
else
    record "slice-missing-input-emits-engine-log-and-load-error" 0 \
        "exit=$LAST_EXIT (want 1); engine_log/load_error events not both present"
fi

if [ -f "$BASE_3MF" ]; then
    # ── A profile that fails to load must be reported, not swallowed ────────
    # The slice continues on the settings already resolved (no hard block) and
    # still exits 0 — the agent learns from the event that its chosen filament
    # profile never took effect.
    GC=$(mktmp_gcode)
    run "$OUT" "$ERR" "$BASE_3MF" --filament "$FIXTURES/does_not_exist.json" -o "$GC"
    if [ "$LAST_EXIT" -eq 0 ] \
       && has_event "$OUT" "config_load_failed" \
       && grep -q "\"kind\":\"filament\"" "$OUT"; then
        record "profile-load-failure-is-an-event-not-a-refusal" 1
    else
        record "profile-load-failure-is-an-event-not-a-refusal" 0 \
            "exit=$LAST_EXIT (want 0); config_load_failed/kind=filament not found"
    fi
    rm -f "$GC"

    # ── A rejected command-line override must be reported ───────────────────
    # The value cannot be parsed, so the override silently had no effect. The
    # slice still runs to completion with the unmodified setting.
    GC=$(mktmp_gcode)
    run "$OUT" "$ERR" "$BASE_3MF" --layer-height not-a-number -o "$GC"
    if [ "$LAST_EXIT" -eq 0 ] \
       && has_event "$OUT" "override_rejected" \
       && grep -q "\"opt_key\":\"layer_height\"" "$OUT"; then
        record "rejected-override-is-an-event" 1
    else
        record "rejected-override-is-an-event" 0 \
            "exit=$LAST_EXIT (want 0); override_rejected/layer_height not found"
    fi
    rm -f "$GC"

    # ── --plate out of range keeps its exit code and gains an event ─────────
    run "$OUT" "$ERR" "$BASE_3MF" --plate 99 -o /dev/null
    if [ "$LAST_EXIT" -eq 1 ] \
       && has_event "$OUT" "input_error" \
       && grep -q "\"tag\":\"PlateOutOfRange\"" "$OUT"; then
        record "plate-out-of-range-is-an-event" 1
    else
        record "plate-out-of-range-is-an-event" 0 \
            "exit=$LAST_EXIT (want 1); input_error/PlateOutOfRange not found"
    fi
else
    echo "SKIP [slice-path event assertions] fixture missing: $BASE_3MF"
fi

# ── Strict-JSON path 1: layout capabilities ─────────────────────────────────
# Exactly one JSON document line on stdout, and no event line beside it.
run "$OUT" "$ERR" layout capabilities --json
DOC_LINES=$(grep -c '^{' "$OUT" || true)
if [ "$LAST_EXIT" -eq 0 ] && no_events "$OUT" && [ "$DOC_LINES" = "1" ]; then
    record "layout-capabilities-stdout-stays-one-json-document" 1
else
    record "layout-capabilities-stdout-stays-one-json-document" 0 \
        "exit=$LAST_EXIT (want 0); json-document lines=$DOC_LINES (want 1); or an event line leaked"
fi

# ── Strict-JSON path 2: --layout-plan ───────────────────────────────────────
# This is the case the install point exists to protect, and the assertion is
# only meaningful if engine code actually logs on this path — so the test
# proves that first: the run must produce warning/error BOOST_LOG_TRIVIAL text
# on stdout (model load, backup-path setup) AND still carry no event line.
# Install the bridge any earlier and those records become [[SLICER_EVENT]]
# lines inside a document every caller parses as one.
PROFILES_DIR="$SCRIPT_DIR/../references/BambuStudio/resources/profiles"
MACHINE_PROFILE="BBL/machine/Bambu Lab X1 Carbon 0.4 nozzle.json"
if [ -f "$BASE_3MF" ] && [ -f "$PROFILES_DIR/$MACHINE_PROFILE" ]; then
    PROBLEM=$(mktemp "${TMPDIR:-/tmp}/diag_problem.XXXXXX")
    cat > "$PROBLEM" <<EOF
{
  "schemaVersion": 1,
  "engine": "bambu",
  "profilesDir": "$PROFILES_DIR",
  "profiles": { "machine": "$MACHINE_PROFILE" },
  "spacing": { "min_object_distance_mm": 10.0 },
  "models": [ { "id": "a", "path": "$BASE_3MF" } ]
}
EOF
    run "$OUT" "$ERR" --layout-plan --input "$PROBLEM"
    LAYOUT_EXIT=$LAST_EXIT
    ENGINE_LOGGED=0
    grep -qE "\[(warning|error)\]" "$OUT" && ENGINE_LOGGED=1
    if [ "$ENGINE_LOGGED" = "1" ] && no_events "$OUT"; then
        record "layout-plan-stdout-carries-no-events-while-engine-logs" 1
    elif [ "$ENGINE_LOGGED" = "0" ]; then
        record "layout-plan-stdout-carries-no-events-while-engine-logs" 0 \
            "the run logged nothing at warning+ (exit=$LAYOUT_EXIT), so the no-event assertion proves nothing — fix the fixture"
    else
        record "layout-plan-stdout-carries-no-events-while-engine-logs" 0 \
            "an event line leaked into the --layout-plan stdout document (exit=$LAYOUT_EXIT)"
    fi
    rm -f "$PROBLEM"
else
    echo "SKIP [layout-plan-stdout-carries-no-events-while-engine-logs] fixture or profile missing"
fi

echo
echo "── diagnostic-event tests ──"
echo "Passed: $PASS"
echo "Failed: $FAIL"
[ "$FAIL" -eq 0 ]
