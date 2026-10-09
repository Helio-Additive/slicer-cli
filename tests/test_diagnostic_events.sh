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
# Runs on both engine binaries (slicer_cli and slicer_cli-orcaslicer). The
# engine is read from `layout capabilities --json`; each engine slices its own
# fixture (see tests/test_excluded_features.sh for how calib_base_orca.3mf was
# made), because a binary refuses a project made by the other engine's newer
# desktop app. The layout checks use that engine's profiles and engine name.
#
# Exit code: 0 = all tests passed. Non-zero = one or more failures.

set -euo pipefail

BINARY="${1:-slicer_cli}"

PASS=0
FAIL=0
SKIP=0

record() {
    local LABEL="$1"; local OK="$2"; local DETAIL="${3:-}"
    if [ "$OK" = "1" ]; then
        PASS=$((PASS + 1)); echo "PASS [$LABEL]"
    else
        FAIL=$((FAIL + 1)); echo "FAIL [$LABEL]"; [ -n "$DETAIL" ] && echo "  $DETAIL"
    fi
    return 0
}
skip() { SKIP=$((SKIP + 1)); echo "SKIP [$1] $2"; }
# native PATH: the path as the engine's own loader must see it. Git Bash
# converts MSYS paths (/d/..., /tmp/...) in command-line arguments only, never
# inside a file, so a path written into a request JSON has to be converted by
# hand (cygpath -m gives D:/...). Elsewhere: unchanged.
native() { if command -v cygpath > /dev/null 2>&1; then cygpath -m "$1"; else printf '%s' "$1"; fi; }

# Same mktemp caveat as test_excluded_features.sh: the X's must be last on
# both BSD (macOS) and GNU (Linux).
mktmp_gcode() { mktemp "${TMPDIR:-/tmp}/diag_cli.XXXXXX"; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FIXTURES="$SCRIPT_DIR/fixtures"

# The engine of this binary, from its strict-JSON capabilities document.
ENGINE=$("$BINARY" layout capabilities --json 2>/dev/null \
    | python3 -c 'import json,sys; print(json.load(sys.stdin)["engine"])' 2>/dev/null || true)
case "$ENGINE" in
    bambu) BASE_3MF="$FIXTURES/calib_base.3mf"
           DEFAULT_PROFILES_DIR="$SCRIPT_DIR/../references/BambuStudio/resources/profiles" ;;
    orca)  BASE_3MF="$FIXTURES/calib_base_orca.3mf"
           DEFAULT_PROFILES_DIR="$SCRIPT_DIR/../references/OrcaSlicer/resources/profiles" ;;
    *) echo "FAIL [engine-detect] 'layout capabilities --json' gave no engine (got '$ENGINE')"; exit 1 ;;
esac
echo "Engine: $ENGINE"

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
LAYOUT_DIR=$(mktemp -d "${TMPDIR:-/tmp}/diag_layout.XXXXXX")
trap 'rm -f "$OUT" "$ERR"; rm -rf "$LAYOUT_DIR"' EXIT

# ── Slice path: engine log records reach the event stream ───────────────────
# A missing input makes libslic3r log `[error] Unable to open the file ...`
# through BOOST_LOG_TRIVIAL, which had no structured representation at all.
# The driver's own load failure becomes an event too. Exit code unchanged (1).
run "$OUT" "$ERR" "$FIXTURES/does_not_exist.3mf" -o /dev/null
if [ "$LAST_EXIT" -eq 1 ] \
   && has_event "$OUT" "engine_log" \
   && grep -q "\"severity\":\"error\"" "$OUT" \
   && grep -Eq '^\[[^]]+\] \[[^]]+\] \[error\].*Unable to open' "$OUT" \
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
    run "$OUT" "$ERR" "$BASE_3MF" --verbose --filament "$FIXTURES/does_not_exist.json" -o "$GC"
    if [ "$LAST_EXIT" -eq 0 ] \
       && has_event "$OUT" "config_load_failed" \
       && grep -q "\"kind\":\"filament\"" "$OUT"; then
        record "profile-load-failure-is-an-event-not-a-refusal" 1
    else
        record "profile-load-failure-is-an-event-not-a-refusal" 0 \
            "exit=$LAST_EXIT (want 0); config_load_failed/kind=filament not found"
    fi
    if grep -Eq '^\[[^]]+\] \[[^]]+\] \[(info|debug|trace)\]' "$OUT"; then
        record "verbose-preserves-plain-engine-diagnostics" 1
    else
        record "verbose-preserves-plain-engine-diagnostics" 0
    fi
    rm -f "$GC"

    # ── A rejected command-line override must be reported ───────────────────
    # The value cannot be parsed, so the override silently had no effect. The
    # slice still runs to completion with the unmodified setting.
    GC=$(mktmp_gcode)
    run "$OUT" "$ERR" "$BASE_3MF" --infill not-a-number -o "$GC"
    if [ "$LAST_EXIT" -eq 0 ] \
       && has_event "$OUT" "override_rejected" \
       && grep -q "\"opt_key\":\"fill_density\"" "$OUT"; then
        record "rejected-override-is-an-event" 1
    else
        record "rejected-override-is-an-event" 0 \
            "exit=$LAST_EXIT (want 0); override_rejected/fill_density not found"
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
    skip "slice-path event assertions" "fixture missing: $BASE_3MF"
fi

# ── Strict-JSON path 1: layout capabilities ─────────────────────────────────
# Exactly one JSON document line on stdout, and no event line beside it.
run "$OUT" "$ERR" layout capabilities --json
valid_json() { python3 -c 'import json,sys; assert isinstance(json.load(sys.stdin), dict)' < "$1"; }
if [ "$LAST_EXIT" -eq 0 ] && no_events "$OUT" && valid_json "$OUT"; then
    record "layout-capabilities-stdout-stays-one-json-document" 1
else
    record "layout-capabilities-stdout-stays-one-json-document" 0 \
        "exit=$LAST_EXIT (want 0); stdout must parse as exactly one JSON object with no event lines"
fi

# ── Strict-JSON path 2: --layout-plan ───────────────────────────────────────
# Require successful planning and parse the entire stdout, not a matching line.
# SLICER_TEST_PROFILES_DIR must hold THIS engine's profiles (a package has
# resources/profiles for slicer_cli and resources/profiles-orca for Orca).
PROFILES_DIR="${SLICER_TEST_PROFILES_DIR:-$DEFAULT_PROFILES_DIR}"
MACHINE_PROFILE="BBL/machine/Bambu Lab X1 Carbon 0.4 nozzle.json"
if [ -f "$PROFILES_DIR/$MACHINE_PROFILE" ]; then
    # The generic layout reader does not import Bambu project components.
    # A watertight cube exercises actual model loading and arrangement.
    python3 - "$LAYOUT_DIR/cube.stl" <<'PY'
import sys
vertices = [(0,0,0), (10,0,0), (10,10,0), (0,10,0),
            (0,0,10), (10,0,10), (10,10,10), (0,10,10)]
faces = [(0,2,1),(0,3,2),(4,5,6),(4,6,7),(0,1,5),(0,5,4),
         (1,2,6),(1,6,5),(2,3,7),(2,7,6),(3,0,4),(3,4,7)]
with open(sys.argv[1], "w") as f:
    f.write("solid cube\n")
    for face in faces:
        f.write("facet normal 0 0 0\nouter loop\n")
        for index in face:
            f.write("vertex %s %s %s\n" % vertices[index])
        f.write("endloop\nendfacet\n")
    f.write("endsolid cube\n")
PY
    PROBLEM=$(mktemp "${TMPDIR:-/tmp}/diag_problem.XXXXXX")
    cat > "$PROBLEM" <<EOF
{
  "schemaVersion": 1,
  "engine": "$ENGINE",
  "profilesDir": "$(native "$PROFILES_DIR")",
  "profiles": { "machine": "$MACHINE_PROFILE" },
  "spacing": { "minObjectDistanceMm": 10.0 },
  "models": [ { "id": "a", "path": "$(native "$LAYOUT_DIR/cube.stl")" } ]
}
EOF
    run "$OUT" "$ERR" --layout-plan --input "$PROBLEM"
    LAYOUT_EXIT=$LAST_EXIT
    if [ "$LAYOUT_EXIT" -eq 0 ] && no_events "$OUT" && valid_json "$OUT"; then
        record "layout-plan-stdout-stays-one-json-document" 1
    else
        record "layout-plan-stdout-stays-one-json-document" 0 \
            "exit=$LAYOUT_EXIT (want 0); stdout must parse as exactly one JSON object"
        cat "$ERR"
    fi
    # Legacy --layout permits engine text before the placements document.
    # Preserve that contract without adding structured slicing events.
    cat > "$PROBLEM" <<EOF
{
  "profilesDir": "$(native "$PROFILES_DIR")",
  "profiles": { "machine": "$MACHINE_PROFILE" },
  "objects": [ { "stl": "$(native "$LAYOUT_DIR/cube.stl")" } ]
}
EOF
    run "$OUT" "$ERR" --layout "$PROBLEM"
    if [ "$LAST_EXIT" -eq 0 ] && no_events "$OUT" && python3 - "$OUT" "$ENGINE" <<'PY'
import json
import sys
with open(sys.argv[1]) as f:
    lines = [line for line in f if line.strip()]
result = json.loads(lines[-1])
assert result["engine"] == sys.argv[2]
assert len(result["placements"]) == 1
PY
    then
        record "legacy-layout-keeps-placement-result-without-events" 1
    else
        record "legacy-layout-keeps-placement-result-without-events" 0 \
            "exit=$LAST_EXIT (want 0); missing placements or slicing events leaked"
    fi
    rm -f "$PROBLEM"
else
    skip "layout-plan-stdout-stays-one-json-document" "profile missing: $PROFILES_DIR/$MACHINE_PROFILE"
    skip "legacy-layout-keeps-placement-result-without-events" "profile missing: $PROFILES_DIR/$MACHINE_PROFILE"
fi

echo
echo "── diagnostic-event tests ($ENGINE) ──"
echo "Passed: $PASS"
echo "Failed: $FAIL"
echo "Skipped: $SKIP"
[ "$FAIL" -eq 0 ]
