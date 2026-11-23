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
# Runs on both engine binaries (slicer_cli and slicer_cli-orcaslicer). The
# engine is read from `layout capabilities --json`; each engine slices its own
# fixtures, because a binary refuses a project made by the other engine's newer
# desktop app (the cross-engine refusal). A check whose feature exists on one
# engine only prints SKIP with the reason.
#
# Exit code: 0 = all tests passed. Non-zero = one or more failures.

set -euo pipefail

BINARY="${1:-slicer_cli}"

PASS=0
FAIL=0
SKIP=0

check() {
    local LABEL="$1"; shift
    local EXPECTED_EXIT="$1"; shift
    local EXPECTED_OUTPUT="$1"; shift

    set +e
    OUTPUT=$("$BINARY" "$@" 2>&1)
    ACTUAL_EXIT=$?
    set -e

    local STATUS="PASS"
    if [ "$ACTUAL_EXIT" -eq "$EXPECTED_EXIT" ] && grep -q "$EXPECTED_OUTPUT" <<<"$OUTPUT"; then
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
    return 0
}
skip() { SKIP=$((SKIP + 1)); echo "SKIP [$1] $2"; }

# The engine of this binary, from its strict-JSON capabilities document.
ENGINE=$("$BINARY" layout capabilities --json 2>/dev/null \
    | python3 -c 'import json,sys; print(json.load(sys.stdin)["engine"])' 2>/dev/null || true)
case "$ENGINE" in
    bambu|orca|snapmaker) echo "Engine: $ENGINE" ;;
    *) echo "FAIL [engine-detect] 'layout capabilities --json' gave no engine (got '$ENGINE')"; exit 1 ;;
esac

# Per-engine fixtures. calib_base.3mf / legacy_token.3mf were saved by Bambu
# Studio 02.05.01.52 (X1 Carbon 0.4, Bambu PLA Basic); slicer_cli-orcaslicer
# refuses them as newer Bambu Studio files. The _orca twins are the same
# printer, process and filament, saved by slicer_cli-orcaslicer (engine f92a7c0,
# OrcaSlicer 2.5.2) from a 20 mm cube:
#   slicer_cli-orcaslicer cube.stl --slice 1 --arrange 1 \
#     --printer-preset "Bambu Lab X1 Carbon 0.4 nozzle" \
#     --process-preset "0.20mm Standard @BBL X1C" \
#     --filament-preset "Bambu PLA Basic @BBL X1C" --export-3mf calib_base_orca.3mf
# then stripped of the plate G-code (Metadata/plate_1.gcode, its .md5 and rels,
# the gcode_file key, the slice_info plate block). legacy_token_orca.3mf adds the
# same two `{initial_no_support_filament_id}` lines as legacy_token.3mf to
# machine_start_gcode and filament_start_gcode.
# The _snapmaker twins are made the same way by slicer_cli-snapmaker (engine
# bac6fd5) with --printer-preset "Snapmaker U1 (0.4 nozzle)" --process-preset
# "0.20mm Standard @Snapmaker U1 (0.4 nozzle)" --filament-preset "Generic PLA"
# --export-3mf calib_base_snapmaker.3mf, with the same stripping and token lines.
case "$ENGINE" in
    orca)     FIXTURE_SUFFIX=_orca ;;
    snapmaker) FIXTURE_SUFFIX=_snapmaker ;;
    *)        FIXTURE_SUFFIX= ;;
esac

# Pressure-advance command in the sliced G-code. The Snapmaker fixture's
# printer (U1) is a Klipper machine, so the engine writes SET_PRESSURE_ADVANCE
# there instead of M900.
PA_RE="M900 K[0-9.]+"
[ "$ENGINE" = snapmaker ] && PA_RE="SET_PRESSURE_ADVANCE ADVANCE=[0-9.]+"

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
# The refusal names the kinds this engine loads.
check "xyz-names-kinds"       1 "This engine loads: stl, obj"   "$XYZ_TMP"
rm -f "$XYZ_TMP"

# ── Missing-input rejection ─────────────────────────────────────────────────
check "no-input"              1 ""   # no args → non-zero exit, message doesn't matter

# ── Excluded flags (PostProcessor, GCodeSender) ─────────────────────────────
# These flags do not exist. The binary must reject them, not silently ignore.
check "no-post-process-flag"  1 ""   --post-process /tmp/fake_script.sh /dev/null
check "no-send-to-printer"    1 ""   --send-to-printer 192.168.1.1 /dev/null

# ── #4: `initial_no_support_filament_id` compatibility ──────────────────────
# Old BambuStudio releases did not bind this token, so the driver normalizes it
# to `initial_no_support_extruder` by default. The bumped reference now binds
# the token natively (BambuStudio 862dd5e16), and the opt-out must therefore
# emit a resolved G-code body without the driver's normalization notice.
TOKEN="initial_no_support_filament_id"
TOKEN_3MF="$FIXTURES/legacy_token$FIXTURE_SUFFIX.3mf"
BASE_3MF="$FIXTURES/calib_base$FIXTURE_SUFFIX.3mf"

if [ -f "$TOKEN_3MF" ]; then
    # (1) Default: slice succeeds, an audit notice is emitted, and the literal
    #     token is gone from the exported G-code.
    #     The token is in the file's own saved start G-code, so the file's
    #     values are sliced (--project-settings-as-saved): by default a Bambu
    #     Studio project takes today's preset G-code, as the desktop app does.
    GC=$(mktmp_gcode)
    run_slice "$GC" "$TOKEN_3MF" --project-settings-as-saved
    if [ "$LAST_EXIT" -eq 0 ] \
       && grep -q "LegacyGcodeTokenAliased" <<<"$LAST_OUTPUT" \
       && ! grep -q "$TOKEN" "$GC"; then
        record "legacy-token-normalized" 1
    else
        record "legacy-token-normalized" 0 "exit=$LAST_EXIT (want 0); notice/token check failed"
    fi
    rm -f "$GC"

    # (2) The bumped engine binds the historical token itself. Opting out keeps
    #     the source spelling, produces no aliasing notice, and must not leave
    #     that token unresolved in executable G-code (metadata comments may
    #     faithfully retain the archived source text).
    #     OrcaSlicer binds it too from 2.5 (GCode.cpp:3784 at f92a7c0).
    #     Snapmaker Orca (bac6fd5) binds only initial_no_support_extruder, so
    #     there the opt-out hands the engine an unknown variable: its placeholder
    #     parser refuses the custom G-code and no G-code is written. That is the
    #     engine's own behaviour.
    #     The file's own start G-code is sliced, as in (1).
    GC=$(mktmp_gcode)
    run_slice "$GC" "$TOKEN_3MF" --no-normalize-legacy-gcode --project-settings-as-saved
    if [ "$ENGINE" = snapmaker ]; then
        if [ "$LAST_EXIT" -ne 0 ] \
           && [[ "$LAST_OUTPUT" != *LegacyGcodeTokenAliased* ]] \
           && grep -q "Not a variable name" <<<"$LAST_OUTPUT" \
           && grep -q "{$TOKEN}" <<<"$LAST_OUTPUT" \
           && [ ! -s "$GC" ]; then
            record "legacy-token-unbound-without-normalization-snapmaker" 1
        else
            record "legacy-token-unbound-without-normalization-snapmaker" 0 "exit=$LAST_EXIT (want non-zero, placeholder error naming $TOKEN, no G-code, no aliasing notice)"
        fi
    elif [ "$LAST_EXIT" -eq 0 ] \
       && [[ "$LAST_OUTPUT" != *LegacyGcodeTokenAliased* ]] \
       && ! awk -v token="$TOKEN" '
            $0 !~ /^[[:space:]]*;/ && index($0, token) { found = 1; exit }
            END { exit !found }
          ' "$GC"; then
        record "legacy-token-native-without-normalization" 1
    else
        record "legacy-token-native-without-normalization" 0 "exit=$LAST_EXIT (want 0); native binding/aliasing check failed"
    fi
    rm -f "$GC"
else
    skip "legacy-token-*" "fixture missing: $TOKEN_3MF"
fi

if [ -f "$BASE_3MF" ]; then
    # (3) Guard: a config WITHOUT the legacy token is untouched — no rewrite
    #     notice, clean slice. Proves normalization is whole-token and inert
    #     when the token is absent (e.g. the separately-bound initial_filament_id
    #     is never rewritten).
    GC=$(mktmp_gcode)
    run_slice "$GC" "$BASE_3MF"
    if [ "$LAST_EXIT" -eq 0 ] && [[ "$LAST_OUTPUT" != *LegacyGcodeTokenAliased* ]]; then
        record "legacy-token-absent-untouched" 1
    else
        record "legacy-token-absent-untouched" 0 "exit=$LAST_EXIT (want 0); unexpected rewrite notice"
    fi
    rm -f "$GC"
else
    skip "legacy-token-absent-untouched" "fixture missing: $BASE_3MF"
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
if [ "$LAST_EXIT" -ne 0 ] && grep -q "calib-mode" <<<"$LAST_OUTPUT"; then
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
    N=$(distinct_count "$GC" "$PA_RE")
    [ "$LAST_EXIT" -eq 0 ] && [ "${N:-0}" -ge 3 ] \
        && record "calib-pa-tower" 1 \
        || record "calib-pa-tower" 0 "exit=$LAST_EXIT distinct-M900=$N (want exit 0, >=3)"
    rm -f "$GC"

    # pressure_advance_line: engine builds the line test internally (M900 sweep).
    GC=$(mktmp_gcode)
    run_slice "$GC" "$BASE_3MF" --calib-mode pressure_advance_line --calib-start 0 --calib-end 0.1 --calib-step 0.002
    N=$(distinct_count "$GC" "$PA_RE")
    [ "$LAST_EXIT" -eq 0 ] && [ "${N:-0}" -ge 2 ] \
        && record "calib-pa-line" 1 \
        || record "calib-pa-line" 0 "exit=$LAST_EXIT distinct-M900=$N (want exit 0, >=2)"
    rm -f "$GC"

    # pressure_advance_pattern: driver-generated geometry. The loaded model is
    # replaced by the synthesized handle cube; config comes from the 3MF. Look for
    # the pattern layer marker (calib.cpp) + M900.
    # The geometry generator is ported for the Bambu Studio engine only
    # (docs/cli-reference.md, --calib-mode pressure_advance_pattern: "slicer_cli
    # only"); slicer_cli-orcaslicer refuses the mode before any input check.
    PA_PATTERN_SKIP="pressure_advance_pattern is slicer_cli only (docs/cli-reference.md); slicer_cli-orcaslicer refuses the mode"
    # Snapmaker Orca is an OrcaSlicer fork and refuses the mode the same way.
    if [ "$ENGINE" != bambu ]; then
        skip "calib-pa-pattern" "$PA_PATTERN_SKIP"
        GC=$(mktmp_gcode)
        run_slice "$GC" "$BASE_3MF" --calib-mode pressure_advance_pattern --calib-start 0 --calib-end 0.08 --calib-step 0.005
        if [ "$LAST_EXIT" -ne 0 ] \
           && grep -q "pressure_advance_pattern is not yet supported on the OrcaSlicer engine" <<<"$LAST_OUTPUT" \
           && [ ! -s "$GC" ]; then
            record "calib-pa-pattern-refused-on-orca" 1
        else
            record "calib-pa-pattern-refused-on-orca" 0 "exit=$LAST_EXIT (want non-zero + OrcaSlicer refusal, no G-code)"
        fi
        rm -f "$GC"
    else
        GC=$(mktmp_gcode)
        run_slice "$GC" "$BASE_3MF" --calib-mode pressure_advance_pattern --calib-start 0 --calib-end 0.08 --calib-step 0.005
        M=$(total_count "$GC" "start pressure advance pattern for layer")
        K=$(total_count "$GC" "M900")
        [ "$LAST_EXIT" -eq 0 ] && [ "${M:-0}" -ge 1 ] && [ "${K:-0}" -ge 1 ] \
            && record "calib-pa-pattern" 1 \
            || record "calib-pa-pattern" 0 "exit=$LAST_EXIT pattern-markers=$M M900=$K (want exit 0, both >=1)"
        rm -f "$GC"
    fi

    # Negative: a reversed PA sweep (start > end) must be rejected before slicing
    # — the engine's unsigned pattern-count loop would otherwise wrap unbounded.
    GC=$(mktmp_gcode)
    run_slice "$GC" "$BASE_3MF" --calib-mode pressure_advance_pattern --calib-start 0.1 --calib-end 0 --calib-step 0.005
    if [ "$LAST_EXIT" -ne 0 ] && grep -q "greater than" <<<"$LAST_OUTPUT"; then
        record "calib-pa-reversed-rejected" 1
    else
        record "calib-pa-reversed-rejected" 0 "exit=$LAST_EXIT (want non-zero + ascending-sweep error)"
    fi
    rm -f "$GC"

    # temp_tower descends from start (engine hook), so an ascending range is a
    # misconfiguration and must be rejected.
    GC=$(mktmp_gcode)
    run_slice "$GC" "$BASE_3MF" --calib-mode temp_tower --calib-start 190 --calib-end 240 --calib-step 5
    if [ "$LAST_EXIT" -ne 0 ] && grep -q "descends" <<<"$LAST_OUTPUT"; then
        record "calib-temp-ascending-rejected" 1
    else
        record "calib-temp-ascending-rejected" 0 "exit=$LAST_EXIT (want non-zero + descends error)"
    fi
    rm -f "$GC"

    # --calib-extruder-id is only honored by pressure_advance_pattern; a nonzero
    # id on another mode must be rejected (it would calibrate the wrong extruder).
    GC=$(mktmp_gcode)
    run_slice "$GC" "$BASE_3MF" --calib-mode temp_tower --calib-start 240 --calib-end 190 --calib-step 5 --calib-extruder-id 1
    if [ "$LAST_EXIT" -ne 0 ] && grep -q "only honored" <<<"$LAST_OUTPUT"; then
        record "calib-extruder-id-rejected-non-pattern" 1
    else
        record "calib-extruder-id-rejected-non-pattern" 0 "exit=$LAST_EXIT (want non-zero + only-honored error)"
    fi
    rm -f "$GC"

    # retraction_tower ascends from start (engine hook), so a descending range is
    # a misconfiguration and must be rejected.
    GC=$(mktmp_gcode)
    run_slice "$GC" "$BASE_3MF" --calib-mode retraction_tower --calib-start 2 --calib-end 0 --calib-step 0.1
    if [ "$LAST_EXIT" -ne 0 ] && grep -q "ascends" <<<"$LAST_OUTPUT"; then
        record "calib-retraction-descending-rejected" 1
    else
        record "calib-retraction-descending-rejected" 0 "exit=$LAST_EXIT (want non-zero + ascends error)"
    fi
    rm -f "$GC"

    # pressure_advance_pattern requires a .3mf --input (it discards the model but
    # needs the embedded config + plate setup). An STL must be rejected.
    if [ "$ENGINE" != bambu ]; then
        skip "calib-pa-pattern-stl-rejected" "$PA_PATTERN_SKIP"
    else
        STL_TMP=$(mktemp "${TMPDIR:-/tmp}/calib_stl.XXXXXX"); mv "$STL_TMP" "$STL_TMP.stl"; STL_TMP="$STL_TMP.stl"
        printf 'solid x\nendsolid x\n' > "$STL_TMP"
        GC=$(mktmp_gcode)
        run_slice "$GC" "$STL_TMP" --calib-mode pressure_advance_pattern --calib-start 0 --calib-end 0.08 --calib-step 0.005
        if [ "$LAST_EXIT" -ne 0 ] && grep -q "requires a .3mf" <<<"$LAST_OUTPUT"; then
            record "calib-pa-pattern-stl-rejected" 1
        else
            record "calib-pa-pattern-stl-rejected" 0 "exit=$LAST_EXIT (want non-zero + 3mf-required error)"
        fi
        rm -f "$GC" "$STL_TMP"
    fi
else
    skip "calib-*" "fixture missing: $BASE_3MF"
fi

# ── A project loads the way its desktop app loads it ────────────────────────
# Each app keeps the settings the maker changed (different_settings_to_system)
# and takes every other value from the preset it has today, for a project it
# made itself too (BambuStudio Plater.cpp 12155 at v02.09.01.70; OrcaSlicer
# Plater.cpp 6063-6110, 6415 and Preset.cpp 2516-2535 at 8500fcd; Snapmaker_Orca
# Plater.cpp 12103 and Preset.cpp 2121-2148 at bac6fd5). The fixture is this
# engine's own project; --project-settings-as-saved slices the file's values.
if [ -f "$BASE_3MF" ]; then
    OWN_3MF=$(mktemp "${TMPDIR:-/tmp}/own_project.XXXXXX"); mv "$OWN_3MF" "$OWN_3MF.3mf"; OWN_3MF="$OWN_3MF.3mf"
    python3 - "$BASE_3MF" "$OWN_3MF" <<'PY'
import json, sys, zipfile
with zipfile.ZipFile(sys.argv[1]) as zin, zipfile.ZipFile(sys.argv[2], "w", zipfile.ZIP_DEFLATED) as zout:
    for item in zin.infolist():
        data = zin.read(item.filename)
        if item.filename == "Metadata/project_settings.config":
            d = json.loads(data)
            d["sparse_infill_density"] = "33%"
            d["wall_loops"] = "5"
            diff = d.get("different_settings_to_system") or [""]
            diff[0] = "wall_loops"
            d["different_settings_to_system"] = diff
            data = json.dumps(d, indent=4).encode()
        zout.writestr(item, data)
PY
    header() { sed -n "s/^; $2 = //p" "$1" | head -n 1 | tr -d '\r'; }
    GC=$(mktmp_gcode)
    run_slice "$GC" "$OWN_3MF"
    DENSITY=$(header "$GC" sparse_infill_density); WALLS=$(header "$GC" wall_loops)
    if [ "$LAST_EXIT" -eq 0 ] && [ -n "$DENSITY" ] && [ "$DENSITY" != "33%" ] && [ "$WALLS" = 5 ] \
       && grep -q "ProjectRebasedOnSystemPresets" <<<"$LAST_OUTPUT"; then
        record "own-project-loads-like-the-app" 1
    else
        record "own-project-loads-like-the-app" 0 "exit=$LAST_EXIT density='$DENSITY' (want the preset's, not 33%) wall_loops='$WALLS' (want 5)"
    fi
    # The run names what it did not keep: the replaced setting is in the
    # event's list, the maker's own change is not.
    if python3 -c '
import json, sys
for line in sys.stdin:
    line = line.strip()
    if line.startswith("[[SLICER_EVENT]] ") and "ProjectRebasedOnSystemPresets" in line:
        replaced = json.loads(line[len("[[SLICER_EVENT]] "):])["replaced_settings"]
        assert "sparse_infill_density" in replaced and "wall_loops" not in replaced, replaced
        break
else:
    raise SystemExit("no ProjectRebasedOnSystemPresets event")
' <<<"$LAST_OUTPUT" 2>/dev/null; then
        record "own-project-names-what-the-presets-replaced" 1
    else
        record "own-project-names-what-the-presets-replaced" 0 "replaced_settings does not hold sparse_infill_density without wall_loops"
    fi
    # result.json names a setting only when the print used a value other than
    # the file's: here the one unlisted edit, nothing else.
    REPLACED_DIR=$(mktemp -d "${TMPDIR:-/tmp}/replaced.XXXXXX")
    set +e
    "$BINARY" "$OWN_3MF" --slice 1 --outputdir "$REPLACED_DIR" > "$REPLACED_DIR.out" 2>&1
    set -e
    REPLACED=$(python3 -c 'import json, sys; r = json.load(open(sys.argv[1])); print(json.dumps(r.get("settings_replaced_by_presets")), json.dumps(r["sliced_plates"][0].get("settings_replaced_by_presets")))' "$REPLACED_DIR/result.json" 2>/dev/null || true)
    # The engine's own project fixtures hold a few values older than today's
    # presets that the maker did not list; those are replaced and printed as
    # the preset's, and are named too.
    WANT_REPLACED='["sparse_infill_density"]'
    # Bambu: the edit moves brim_type off the fixture's own changed list, so
    # the preset's brim type is used, and that is named too.
    [ "$ENGINE" = bambu ] && WANT_REPLACED='["brim_type", "filament_cost", "sparse_infill_density"]'
    if [ "$REPLACED" = "$WANT_REPLACED $WANT_REPLACED" ]; then
        record "own-project-result-names-only-what-printed-differently" 1
    else
        record "own-project-result-names-only-what-printed-differently" 0 "result.json run and plate lists: '$REPLACED' (want $WANT_REPLACED for both)"
    fi
    # A setting a flag gives is the flag's, not the presets': --infill writes
    # sparse_infill_density, so the edit is no longer named.
    set +e
    "$BINARY" "$OWN_3MF" --infill 20% --slice 1 --outputdir "$REPLACED_DIR/f" > "$REPLACED_DIR.f.out" 2>&1
    set -e
    FLAGGED=$(python3 -c 'import json, sys; print(json.dumps(json.load(open(sys.argv[1])).get("settings_replaced_by_presets")))' "$REPLACED_DIR/f/result.json" 2>/dev/null || true)
    WANT_FLAGGED=$(python3 -c 'import json, sys; print(json.dumps([k for k in json.loads(sys.argv[1]) if k != "sparse_infill_density"]))' "$WANT_REPLACED")
    if [ -n "$FLAGGED" ] && [ "$FLAGGED" = "$WANT_FLAGGED" ]; then
        record "own-project-result-leaves-out-what-a-flag-set" 1
    else
        record "own-project-result-leaves-out-what-a-flag-set" 0 "with --infill 20%: '$FLAGGED' (want $WANT_FLAGGED)"
    fi
    rm -rf "$REPLACED_DIR" "$REPLACED_DIR.out" "$REPLACED_DIR.f.out"
    run_slice "$GC" "$OWN_3MF" --project-settings-as-saved
    DENSITY=$(header "$GC" sparse_infill_density)
    if [ "$LAST_EXIT" -eq 0 ] && [ "$DENSITY" = "33%" ]; then
        record "own-project-as-saved" 1
    else
        record "own-project-as-saved" 0 "exit=$LAST_EXIT density='$DENSITY' (want 33%)"
    fi
    rm -f "$GC" "$OWN_3MF"
else
    skip "own-project-loads-like-the-app" "fixture missing: $BASE_3MF"
fi

# ── A printer with nozzle variants ──────────────────────────────────────────
# The presets of an H2D hold one value per nozzle variant, which the print
# narrows to the plate's own; a project exported from them and loaded again
# names only the setting it edited without listing it.
if [ "$ENGINE" = bambu ]; then
    VAR_DIR=$(mktemp -d "${TMPDIR:-/tmp}/variant_project.XXXXXX")
    python3 - "$VAR_DIR/cube.stl" <<'PY'
import sys
v = [(0,0,0),(10,0,0),(10,10,0),(0,10,0),(0,0,10),(10,0,10),(10,10,10),(0,10,10)]
f = [(0,2,1),(0,3,2),(4,5,6),(4,6,7),(0,1,5),(0,5,4),(1,2,6),(1,6,5),(2,3,7),(2,7,6),(3,0,4),(3,4,7)]
with open(sys.argv[1], "w") as o:
    o.write("solid c\n")
    for t in f:
        o.write("facet normal 0 0 0\nouter loop\n" + "".join("vertex %g %g %g\n" % v[i] for i in t) + "endloop\nendfacet\n")
    o.write("endsolid c\n")
PY
    set +e
    "$BINARY" "$VAR_DIR/cube.stl" --printer-preset "Bambu Lab H2D 0.4 nozzle" --filament-preset "Bambu PLA Basic @BBL H2D" \
        --slice 1 --outputdir "$VAR_DIR/a" --export-3mf p.3mf > "$VAR_DIR/a.out" 2>&1; VAR_EXPORT=$?
    set -e
    python3 - "$VAR_DIR/a/p.3mf" "$VAR_DIR/edited.3mf" <<'PY' 2>/dev/null || true
import json, sys, zipfile
with zipfile.ZipFile(sys.argv[1]) as zin, zipfile.ZipFile(sys.argv[2], "w", zipfile.ZIP_DEFLATED) as zout:
    for item in zin.infolist():
        data = zin.read(item.filename)
        if item.filename.startswith("Metadata/plate_") and item.filename.endswith((".gcode", ".md5")):
            continue
        if item.filename == "Metadata/project_settings.config":
            d = json.loads(data)
            d["sparse_infill_density"] = "33%"
            data = json.dumps(d, indent=4).encode()
        zout.writestr(item, data)
PY
    set +e
    "$BINARY" "$VAR_DIR/edited.3mf" --slice 1 --outputdir "$VAR_DIR/b" > "$VAR_DIR/b.out" 2>&1; VAR_EXIT=$?
    set -e
    VAR_REPLACED=$(python3 -c 'import json, sys; print(json.dumps(json.load(open(sys.argv[1])).get("settings_replaced_by_presets")))' "$VAR_DIR/b/result.json" 2>/dev/null || true)
    if [ "$VAR_EXPORT" -eq 0 ] && [ "$VAR_EXIT" -eq 0 ] && [ "$VAR_REPLACED" = '["sparse_infill_density"]' ]; then
        record "variant-printer-project-names-only-what-printed-differently" 1
    else
        record "variant-printer-project-names-only-what-printed-differently" 0 "export exit=$VAR_EXPORT, slice exit=$VAR_EXIT, settings_replaced_by_presets=$VAR_REPLACED (want [\"sparse_infill_density\"])"
    fi
    rm -rf "$VAR_DIR"
fi

# ── Bambu Studio's spelling of the flow type ────────────────────────────────
# Bambu Studio and OrcaSlicer write "Standard" / "High Flow"; Snapmaker Orca has
# "standard" / "high_flow" and its desktop substitutes its default for the other
# words. slicer_cli-snapmaker reads them as the same flow type and says so; any
# other word is still refused with --slice.
if [ "$ENGINE" = snapmaker ] && [ -f "$BASE_3MF" ]; then
    FLOW_DIR=$(mktemp -d "${TMPDIR:-/tmp}/flow_type.XXXXXX")
    flow_file() {
        python3 - "$BASE_3MF" "$1" "$2" <<'PY'
import json, sys, zipfile
with zipfile.ZipFile(sys.argv[1]) as zin, zipfile.ZipFile(sys.argv[2], "w", zipfile.ZIP_DEFLATED) as zout:
    for item in zin.infolist():
        data = zin.read(item.filename)
        if item.filename == "Metadata/project_settings.config":
            d = json.loads(data)
            d["nozzle_volume_type"] = sys.argv[3].split(",")
            data = json.dumps(d, indent=4).encode()
        zout.writestr(item, data)
PY
    }
    flow_file "$FLOW_DIR/bambu_words.3mf" "Standard,High Flow,Standard,Standard"
    flow_file "$FLOW_DIR/unknown_word.3mf" "Standard,Turbo,Standard,Standard"
    GC=$(mktmp_gcode)
    set +e
    "$BINARY" "$FLOW_DIR/bambu_words.3mf" --slice 1 --outputdir "$FLOW_DIR/a" > "$FLOW_DIR/a.out" 2>&1; FLOW_EXIT=$?
    "$BINARY" "$FLOW_DIR/unknown_word.3mf" --slice 1 --outputdir "$FLOW_DIR/b" > "$FLOW_DIR/b.out" 2>&1; UNKNOWN_EXIT=$?
    set -e
    FLOW_HEADER=$(sed -n 's/^; nozzle_volume_type = //p' "$FLOW_DIR/a/plate_1.gcode" 2>/dev/null | head -n 1 | tr -d '\r' || true)
    if [ "$FLOW_EXIT" -eq 0 ] && [ "$FLOW_HEADER" = "standard,high_flow,standard,standard" ] \
       && grep -q 'FlowTypeSpellingConverted' "$FLOW_DIR/a.out"; then
        record "flow-type-bambu-spelling-read-as-snapmaker" 1
    else
        record "flow-type-bambu-spelling-read-as-snapmaker" 0 "exit=$FLOW_EXIT nozzle_volume_type='$FLOW_HEADER' (want standard,high_flow,standard,standard and a FlowTypeSpellingConverted event)"
    fi
    if [ "$UNKNOWN_EXIT" -ne 0 ] && grep -q "'nozzle_volume_type' is 'Standard,Turbo,Standard,Standard'" "$FLOW_DIR/b.out"; then
        record "flow-type-unknown-word-refused" 1
    else
        record "flow-type-unknown-word-refused" 0 "exit=$UNKNOWN_EXIT: want a refusal naming the value"
    fi
    # A settings file that inherits a shipped preset is read twice (its own
    # values, then over the preset it inherits); both reads keep the flow type.
    printf '%s\n' '{"type":"machine","name":"Flow test U1","from":"User","inherits":"Snapmaker U1 (0.4 nozzle)","nozzle_volume_type":["High Flow","Standard","Standard","Standard"]}' > "$FLOW_DIR/machine.json"
    python3 - "$FLOW_DIR/cube.stl" <<'PY'
import sys
v = [(0,0,0),(10,0,0),(10,10,0),(0,10,0),(0,0,10),(10,0,10),(10,10,10),(0,10,10)]
f = [(0,2,1),(0,3,2),(4,5,6),(4,6,7),(0,1,5),(0,5,4),(1,2,6),(1,6,5),(2,3,7),(2,7,6),(3,0,4),(3,4,7)]
with open(sys.argv[1], "w") as o:
    o.write("solid c\n")
    for t in f:
        o.write("facet normal 0 0 0\nouter loop\n" + "".join("vertex %g %g %g\n" % v[i] for i in t) + "endloop\nendfacet\n")
    o.write("endsolid c\n")
PY
    set +e
    "$BINARY" "$FLOW_DIR/cube.stl" --machine "$FLOW_DIR/machine.json" --slice 1 --outputdir "$FLOW_DIR/c" > "$FLOW_DIR/c.out" 2>&1; MACHINE_EXIT=$?
    set -e
    MACHINE_HEADER=$(sed -n 's/^; nozzle_volume_type = //p' "$FLOW_DIR/c/plate_1.gcode" 2>/dev/null | head -n 1 | tr -d '\r' || true)
    if [ "$MACHINE_EXIT" -eq 0 ] && [ "$MACHINE_HEADER" = "high_flow,standard,standard,standard" ]; then
        record "flow-type-bambu-spelling-in-inheriting-settings-file" 1
    else
        record "flow-type-bambu-spelling-in-inheriting-settings-file" 0 "exit=$MACHINE_EXIT nozzle_volume_type='$MACHINE_HEADER' (want high_flow,standard,standard,standard)"
    fi
    # A Bambu Studio placeholder in the printer's start G-code: Snapmaker Orca
    # 2.4.1 binds no value for nozzle_diameter_at_nozzle_id or initial_nozzle_id
    # (Bambu Studio and OrcaSlicer from 2.5 bind both), so the run is refused
    # before it slices, naming the placeholder and what to use instead.
    printf '%s\n' '{"type":"machine","name":"Placeholder U1","from":"User","inherits":"Snapmaker U1 (0.4 nozzle)","machine_start_gcode":"G28\n{if nozzle_diameter_at_nozzle_id[initial_nozzle_id] > 0.4}M900\n{endif}"}' > "$FLOW_DIR/placeholder.json"
    set +e
    "$BINARY" "$FLOW_DIR/cube.stl" --machine "$FLOW_DIR/placeholder.json" --slice 1 --outputdir "$FLOW_DIR/p" > "$FLOW_DIR/p.out" 2>&1; PLACEHOLDER_EXIT=$?
    set -e
    if [ "$PLACEHOLDER_EXIT" -ne 0 ] && grep -q 'BambuOnlyGcodePlaceholder' "$FLOW_DIR/p.out" \
       && grep -q 'nozzle_diameter_at_nozzle_id' "$FLOW_DIR/p.out" \
       && grep -q '{nozzle_diameter\[initial_extruder\]}' "$FLOW_DIR/p.out" \
       && [ ! -s "$FLOW_DIR/p/plate_1.gcode" ]; then
        record "bambu-gcode-placeholder-refused-before-slicing" 1
    else
        record "bambu-gcode-placeholder-refused-before-slicing" 0 "exit=$PLACEHOLDER_EXIT: want a BambuOnlyGcodePlaceholder refusal naming the placeholder and {nozzle_diameter[initial_extruder]}, and no G-code"
    fi
    # --load-settings takes a process with the printer: the engine's own U1
    # 0.20mm Standard process file.
    FLOW_PROCESS="$SCRIPT_DIR/../references/SnapmakerOrca/resources/profiles/Snapmaker/process/0.20mm Standard @Snapmaker U1 (0.4 nozzle).json"
    if [ -f "$FLOW_PROCESS" ]; then
        set +e
        # Git Bash on Windows does not convert the paths of a ';' list.
        case "$(uname -s)" in
            MINGW*|MSYS*|CYGWIN*) FLOW_LIST="$(cygpath -w "$FLOW_DIR/machine.json");$(cygpath -w "$FLOW_PROCESS")";;
            *) FLOW_LIST="$FLOW_DIR/machine.json;$FLOW_PROCESS";;
        esac
        "$BINARY" "$FLOW_DIR/cube.stl" --load-settings "$FLOW_LIST" --slice 1 --outputdir "$FLOW_DIR/d" > "$FLOW_DIR/d.out" 2>&1; LOAD_EXIT=$?
        set -e
        LOAD_HEADER=$(sed -n 's/^; nozzle_volume_type = //p' "$FLOW_DIR/d/plate_1.gcode" 2>/dev/null | head -n 1 | tr -d '\r' || true)
        if [ "$LOAD_EXIT" -eq 0 ] && [ "$LOAD_HEADER" = "high_flow,standard,standard,standard" ]; then
            record "flow-type-bambu-spelling-in-load-settings" 1
        else
            record "flow-type-bambu-spelling-in-load-settings" 0 "exit=$LOAD_EXIT nozzle_volume_type='$LOAD_HEADER' (want high_flow,standard,standard,standard)"
        fi
    else
        skip "flow-type-bambu-spelling-in-load-settings" "engine profiles missing: $FLOW_PROCESS"
    fi
    rm -rf "$GC" "$FLOW_DIR"
fi

# ── Supports set on a part stay on the part ─────────────────────────────────
# The project load replaces project-wide settings the file does not list as
# changed; what is stored with an object (Metadata/model_settings.config) is
# the object's own and is not part of that. A slab on a pillar, exported as a
# project with supports off, then given enable_support on the object itself:
# the project loaded the app's way prints supports, and without the object's
# setting it prints none.
PART_DIR=$(mktemp -d "${TMPDIR:-/tmp}/part_support.XXXXXX")
python3 - "$PART_DIR/slab.stl" <<'PY'
import sys
def box(x0, y0, z0, x1, y1, z1):
    v = [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
    f = [(0,1,3),(0,3,2),(4,6,7),(4,7,5),(0,4,5),(0,5,1),(2,3,7),(2,7,6),(0,2,6),(0,6,4),(1,5,7),(1,7,3)]
    return [[v[i] for i in t] for t in f]
tris = box(12, 12, 0, 18, 18, 12) + box(0, 0, 12, 30, 30, 14)
with open(sys.argv[1], "w") as o:
    o.write("solid slab\n")
    for t in tris:
        o.write("facet normal 0 0 0\nouter loop\n")
        for p in t:
            o.write("vertex %g %g %g\n" % p)
        o.write("endloop\nendfacet\n")
    o.write("endsolid slab\n")
PY
PART_PRINTER="Bambu Lab X1 Carbon 0.4 nozzle"
[ "$ENGINE" = snapmaker ] && PART_PRINTER="Snapmaker U1 (0.4 nozzle)"
set +e
"$BINARY" "$PART_DIR/slab.stl" --printer-preset "$PART_PRINTER" --slice 1 --outputdir "$PART_DIR/a" --export-3mf p.3mf > "$PART_DIR/a.out" 2>&1; PART_EXPORT=$?
set -e
python3 - "$PART_DIR/a/p.3mf" "$PART_DIR/part.3mf" <<'PY' 2>/dev/null || true
import re, sys, zipfile
with zipfile.ZipFile(sys.argv[1]) as zin, zipfile.ZipFile(sys.argv[2], "w", zipfile.ZIP_DEFLATED) as zout:
    for item in zin.infolist():
        data = zin.read(item.filename)
        if item.filename.startswith("Metadata/plate_") and item.filename.endswith((".gcode", ".md5")):
            continue
        if item.filename == "Metadata/model_settings.config":
            # OrcaSlicer 2.5 writes an attribute after the id (auto_drop).
            text, n = re.subn(r"(<object id=\"\d+\"[^>]*>)", r'\1\n    <metadata key="enable_support" value="1"/>', data.decode(), count=1)
            assert n == 1, "no object in model_settings.config"
            data = text.encode()
        zout.writestr(item, data)
PY
support_lines() { grep -c -i -E '^; ?(FEATURE|TYPE): ?Support' "$1" 2>/dev/null || true; }
GC=$(mktmp_gcode)
run_slice "$GC" "$PART_DIR/a/p.3mf"
PART_OFF_EXIT=$LAST_EXIT; PART_OFF=$(support_lines "$GC")
run_slice "$GC" "$PART_DIR/part.3mf"
PART_ON=$(support_lines "$GC")
if [ "$PART_EXPORT" -eq 0 ] && [ "$PART_OFF_EXIT" -eq 0 ] && [ "$LAST_EXIT" -eq 0 ] \
   && [ "${PART_OFF:-0}" -eq 0 ] && [ "${PART_ON:-0}" -gt 0 ] \
   && grep -q "ProjectRebasedOnSystemPresets" <<<"$LAST_OUTPUT"; then
    record "supports-set-on-a-part-are-kept" 1
else
    record "supports-set-on-a-part-are-kept" 0 "export exit=$PART_EXPORT, project exit=$PART_OFF_EXIT with ${PART_OFF:-?} support sections (want 0), with the part's setting exit=$LAST_EXIT with ${PART_ON:-?} (want more than 0)"
fi

# ── --strict (OrcaSlicer from 2.5) ──────────────────────────────────────────
# The slab on a pillar needs support and has none: the engine raises a warning
# it only logs. Without --strict the run slices and result.json lists the
# warning with a class; with --strict the same warning ends the run (-100);
# with --no-check as well the pair is refused (-2), as the official command
# line does (OrcaSlicer.cpp 1422-1429, 7171-7183 at f92a7c0). The single-file
# call (-o) stops on the same warning with --strict and slices without it.
if [ "$ENGINE" = orca ]; then
    set +e
    "$BINARY" "$PART_DIR/slab.stl" --printer-preset "$PART_PRINTER" --slice 1 --outputdir "$PART_DIR/s0" > "$PART_DIR/s0.out" 2>&1; STRICT_OFF=$?
    "$BINARY" "$PART_DIR/slab.stl" --printer-preset "$PART_PRINTER" --slice 1 --strict --outputdir "$PART_DIR/s1" > "$PART_DIR/s1.out" 2>&1; STRICT_ON=$?
    "$BINARY" "$PART_DIR/slab.stl" --printer-preset "$PART_PRINTER" --slice 1 --strict --no-check --outputdir "$PART_DIR/s2" > "$PART_DIR/s2.out" 2>&1; STRICT_PAIR=$?
    # The model is named as in the s0 run: the warning names the object as it was given.
    "$BINARY" "$PART_DIR/slab.stl" --printer-preset "$PART_PRINTER" -o "$PART_DIR/s3.gcode" > "$PART_DIR/s3.out" 2>&1; STRICT_FILE_OFF=$?
    "$BINARY" "$PART_DIR/slab.stl" --printer-preset "$PART_PRINTER" --strict -o "$PART_DIR/s4.gcode" > "$PART_DIR/s4.out" 2>&1; STRICT_FILE_ON=$?
    set -e
    if STRICT_DETAIL=$(python3 - "$PART_DIR" "$STRICT_OFF" "$STRICT_ON" "$STRICT_PAIR" "$STRICT_FILE_OFF" "$STRICT_FILE_ON" <<'PY' 2>&1
import json, os, sys
d, off, on, pair, file_off, file_on = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5]), int(sys.argv[6])
r0 = json.load(open(d + "/s0/result.json")); r1 = json.load(open(d + "/s1/result.json")); r2 = json.load(open(d + "/s2/result.json"))
assert off == 0 and r0["return_code"] == 0 and r0["strict_mode"] is False, (off, r0["return_code"], r0.get("strict_mode"))
w = [x for x in r0["warnings"] if x["class"] == "slicing_warning_non_critical"]
assert w and w[0]["plate_id"] == 1 and w[0]["text"], r0["warnings"]
assert on != 0 and r1["return_code"] == -100 and r1["strict_mode"] is True, (on, r1["return_code"], r1.get("strict_mode"))
assert [x for x in r1["warnings"] if x["class"] == "slicing_warning_non_critical"], r1["warnings"]
assert pair != 0 and r2["return_code"] == -2 and "--strict cannot be combined with --no-check" in json.dumps(r2), (pair, r2["return_code"])
assert file_off == 0 and os.path.getsize(d + "/s3.gcode") > 0, ("-o without --strict", file_off)
assert file_on != 0, "-o with --strict sliced a model that warns"
said = [line for line in open(d + "/s4.out", encoding="utf-8", errors="replace").read().splitlines() if line.startswith("Error: ")]
assert said and w[0]["text"] in said[-1], "-o with --strict does not end with the warning %r: %r" % (w[0]["text"], said[-1:])
PY
    ); then
        record "strict-ends-the-run-on-a-non-critical-warning" 1
    else
        record "strict-ends-the-run-on-a-non-critical-warning" 0 "exits $STRICT_OFF/$STRICT_ON/$STRICT_PAIR/$STRICT_FILE_OFF/$STRICT_FILE_ON: $(tail -n 1 <<<"$STRICT_DETAIL")"
    fi

    # --export-statistics (OrcaSlicer from 2.5): the plate's statistics as
    # JSON next to its G-code and, the same text, inside the exported project
    # (OrcaSlicer.cpp 7228-7237, bbs_3mf.cpp 9080-9085 at f92a7c0). Without the
    # flag neither is written (the s0 run above).
    set +e
    "$BINARY" "$PART_DIR/slab.stl" --printer-preset "$PART_PRINTER" --slice 1 --export-statistics --export-3mf st.3mf --outputdir "$PART_DIR/st" > "$PART_DIR/st.out" 2>&1; STAT_EXIT=$?
    set -e
    if STAT_DETAIL=$(python3 - "$PART_DIR" "$STAT_EXIT" <<'PY' 2>&1
import json, os, sys, zipfile
d, rc = sys.argv[1], int(sys.argv[2])
assert rc == 0, rc
text = open(d + "/st/plate_1.statistics.json").read()
doc = json.loads(text)
assert {"summary", "layers", "objects"} <= set(doc), sorted(doc)
z = zipfile.ZipFile(d + "/st/st.3mf")
assert z.read("Metadata/plate_1.statistics.json").decode() == text, "the project holds another text"
assert not os.path.exists(d + "/s0/plate_1.statistics.json"), "written without the flag"
PY
    ); then
        record "export-statistics-next-to-the-gcode-and-in-the-project" 1
    else
        record "export-statistics-next-to-the-gcode-and-in-the-project" 0 "exit $STAT_EXIT: $(tail -n 1 <<<"$STAT_DETAIL")"
    fi

    # The statistics file is one of the run's outputs. Without --outputdir it
    # is written next to the G-code, in the current folder (the official skips
    # it there, where its G-code is a temporary file). Its name is checked
    # before the run writes: --export-3mf onto it is refused, and so is a run
    # that reads a settings file of that name in its output folder.
    mkdir -p "$PART_DIR/sc" "$PART_DIR/sn" "$PART_DIR/si"
    printf '%s\n' '{"type":"process","name":"Stat process","from":"User","inherits":"0.20mm Standard @BBL X1C"}' > "$PART_DIR/si/plate_1.statistics.json"
    cp "$PART_DIR/si/plate_1.statistics.json" "$PART_DIR/si.kept"
    set +e
    (cd "$PART_DIR/sc" && "$BINARY" ../slab.stl --printer-preset "$PART_PRINTER" --slice 1 --export-statistics > ../sc.out 2>&1); STATC_EXIT=$?
    "$BINARY" "$PART_DIR/slab.stl" --printer-preset "$PART_PRINTER" --slice 1 --export-statistics --export-3mf plate_1.statistics.json --outputdir "$PART_DIR/sn" > "$PART_DIR/sn.out" 2>&1; STATN_EXIT=$?
    (cd "$PART_DIR/si" && "$BINARY" ../slab.stl --printer-preset "$PART_PRINTER" --process plate_1.statistics.json --slice 1 --export-statistics --outputdir . > ../si.out 2>&1); STATI_EXIT=$?
    set -e
    if STATX_DETAIL=$(python3 - "$PART_DIR" "$STATC_EXIT" "$STATN_EXIT" "$STATI_EXIT" <<'PY' 2>&1
import json, os, sys
d = sys.argv[1]; rc_cwd, rc_name, rc_input = (int(x) for x in sys.argv[2:5])
assert rc_cwd == 0, ("no --outputdir", rc_cwd)
assert os.path.exists(d + "/sc/plate_1.gcode"), "no G-code in the current folder"
assert {"summary", "layers", "objects"} <= set(json.load(open(d + "/sc/plate_1.statistics.json"))), "no statistics next to it"
assert rc_name != 0, "--export-3mf onto the statistics file was not refused"
out = open(d + "/sn.out", encoding="utf-8", errors="replace").read()
assert '"tag":"ExportNameTaken"' in out and "plate_1.statistics.json --export-statistics writes" in out, out[-400:]
assert not os.path.exists(d + "/sn/plate_1.gcode"), "the refused run sliced"
assert rc_input != 0, "a settings file named as the statistics file was not protected"
out = open(d + "/si.out", encoding="utf-8", errors="replace").read()
assert '"tag":"ExportOverwritesInput"' in out and "plate_1.statistics.json" in out, out[-400:]
assert open(d + "/si/plate_1.statistics.json").read() == open(d + "/si.kept").read(), "the settings file was overwritten"
PY
    ); then
        record "export-statistics-is-a-checked-output-in-every-output-place" 1
    else
        record "export-statistics-is-a-checked-output-in-every-output-place" 0 "exits $STATC_EXIT/$STATN_EXIT/$STATI_EXIT: $(tail -n 1 <<<"$STATX_DETAIL")"
    fi

    # --align-to-y-axis (OrcaSlicer from 2.5): only when given does it replace
    # the printer-structure default of --arrange (OrcaSlicer.cpp 1631-1634,
    # 5524-5528 at f92a7c0). A 60 x 10 mm bar: the X1 Carbon (CoreXY) leaves
    # it along X and turns it with the flag; the A1 (an i3 bed-slinger) turns
    # it by default and leaves it with --align-to-y-axis=0.
    python3 - "$PART_DIR/bar.stl" <<'PY'
import sys
v = [(0,0,0),(60,0,0),(60,10,0),(0,10,0),(0,0,5),(60,0,5),(60,10,5),(0,10,5)]
f = [(0,2,1),(0,3,2),(4,5,6),(4,6,7),(0,1,5),(0,5,4),(1,2,6),(1,6,5),(2,3,7),(2,7,6),(3,0,4),(3,4,7)]
with open(sys.argv[1], "w") as o:
    o.write("solid bar\n")
    for t in f:
        o.write("facet normal 0 0 0\nouter loop\n" + "".join("vertex %g %g %g\n" % v[i] for i in t) + "endloop\nendfacet\n")
    o.write("endsolid bar\n")
PY
    align_run() { # name printer flags... -> "width depth" of the object on the plate
        local name=$1 printer=$2; shift 2
        "$BINARY" "$PART_DIR/bar.stl" --printer-preset "$printer" --slice 1 --arrange 1 --outputdir "$PART_DIR/$name" "$@" > "$PART_DIR/$name.out" 2>&1 || true
        python3 -c 'import json, sys
b = json.load(open(sys.argv[1]))["sliced_plates"][0]["objects"][0]["bbox"]
print(round(b["width"]), round(b["depth"]))' "$PART_DIR/$name/result.json" 2>/dev/null || echo "no result"
    }
    ALIGN_GOT="$(align_run y0 "Bambu Lab X1 Carbon 0.4 nozzle") | $(align_run y1 "Bambu Lab X1 Carbon 0.4 nozzle" --align-to-y-axis) | $(align_run y2 "Bambu Lab A1 0.4 nozzle") | $(align_run y3 "Bambu Lab A1 0.4 nozzle" --align-to-y-axis=0)"
    if [ "$ALIGN_GOT" = "60 10 | 10 60 | 10 60 | 60 10" ]; then
        record "align-to-y-axis-replaces-the-printer-default-only-when-given" 1
    else
        record "align-to-y-axis-replaces-the-printer-default-only-when-given" 0 "width depth: $ALIGN_GOT (want 60 10 | 10 60 | 10 60 | 60 10)"
    fi

    # --ground-largest-face, --ground-face-normal, --ground-face-point
    # (OrcaSlicer from 2.5; OrcaSlicer.cpp 5105-5160 at f92a7c0): the object
    # is laid on one of the faces the desktop's "Lay on Face" tool offers.
    # The 60 x 10 x 5 mm bar, by its printed height: stood on its side by
    # --rotate-x 90 it is 10 high; the largest face brings it back to 5; the
    # face that looks along +X, or the face that holds a point of it, stands
    # it on end, 60 high. Malformed numbers and a point on no face are -2.
    ground_run() { # name flags... -> "exit height"
        local name=$1; shift
        local rc=0
        "$BINARY" "$PART_DIR/bar.stl" --printer-preset "Bambu Lab X1 Carbon 0.4 nozzle" --slice 1 --outputdir "$PART_DIR/$name" "$@" > "$PART_DIR/$name.out" 2>&1 || rc=$?
        echo "$rc $(sed -n 's/^; max_z_height: \([0-9]*\).*/\1/p' "$PART_DIR/$name/plate_1.gcode" 2>/dev/null | head -n 1)"
    }
    ground_code() { python3 -c 'import json, sys; print(json.load(open(sys.argv[1]))["return_code"])' "$PART_DIR/$1/result.json" 2>/dev/null || echo none; }
    GROUND_GOT="$(ground_run g0 --rotate-x 90) | $(ground_run g1 --rotate-x 90 --ground-largest-face) | $(ground_run g2 --ground-face-normal 1,0,0) | $(ground_run g3 --ground-face-point 60,5,2.5)"
    ground_run g4 --ground-face-normal abc > /dev/null; ground_run g5 --ground-face-point 1000,0,0 > /dev/null
    GROUND_GOT="$GROUND_GOT | $(ground_code g4) $(ground_code g5)"
    if [ "$GROUND_GOT" = "0 10 | 0 5 | 0 60 | 0 60 | -2 -2" ]; then
        record "ground-face-flags-lay-the-object-on-the-named-face" 1
    else
        record "ground-face-flags-lay-the-object-on-the-named-face" 0 "exit height: $GROUND_GOT (want 0 10 | 0 5 | 0 60 | 0 60 | -2 -2)"
    fi

    # --repair (OrcaSlicer from 2.5; OrcaSlicer.cpp 5267-5280 at f92a7c0):
    # the meshes are repaired as the app's Fix Model does. The bar without
    # its top face: with the flag the hole is filled and the run says so;
    # without it, and for the whole bar with it, nothing is repaired.
    python3 - "$PART_DIR/open.stl" <<'PY'
import sys
v = [(0,0,0),(60,0,0),(60,10,0),(0,10,0),(0,0,5),(60,0,5),(60,10,5),(0,10,5)]
f = [(0,2,1),(0,3,2),(0,1,5),(0,5,4),(1,2,6),(1,6,5),(2,3,7),(2,7,6),(3,0,4),(3,4,7)]
with open(sys.argv[1], "w") as o:
    o.write("solid bar\n")
    for t in f:
        o.write("facet normal 0 0 0\nouter loop\n" + "".join("vertex %g %g %g\n" % v[i] for i in t) + "endloop\nendfacet\n")
    o.write("endsolid bar\n")
PY
    repair_run() { # name stl flags... -> "exit repaired-events filled-holes"
        local name=$1 stl=$2; shift 2
        local rc=0
        "$BINARY" "$PART_DIR/$stl" --printer-preset "Bambu Lab X1 Carbon 0.4 nozzle" --slice 1 --outputdir "$PART_DIR/$name" "$@" > "$PART_DIR/$name.out" 2>&1 || rc=$?
        echo "$rc $(grep -c '"tag":"MeshRepaired"' "$PART_DIR/$name.out" || true) $(sed -n 's/.*"filled_holes":\([0-9]*\).*/\1/p' "$PART_DIR/$name.out" | head -n 1)"
    }
    REPAIR_GOT="$(repair_run rp1 open.stl --repair) | $(repair_run rp2 open.stl) | $(repair_run rp3 bar.stl --repair)"
    if [ "$REPAIR_GOT" = "0 1 1 | 0 0  | 0 0 " ]; then
        record "repair-fills-the-hole-only-when-asked" 1
    else
        record "repair-fills-the-hole-only-when-asked" 0 "exit events holes: '$REPAIR_GOT' (want '0 1 1 | 0 0  | 0 0 ')"
    fi
fi
rm -rf "$GC" "$PART_DIR"

# ── A saved prime tower outside the plate ───────────────────────────────────
# Bambu Studio puts a project's saved tower back inside the plate when it opens
# the project (Plater.cpp 12270-12276 at v02.09.01.70); OrcaSlicer and
# Snapmaker Orca keep the saved position. On the plain call and with --slice
# alike; a position given as a flag stays as given; the exported project
# states the position the load gave.
if [ -f "$BASE_3MF" ]; then
    TOWER_DIR=$(mktemp -d "${TMPDIR:-/tmp}/tower_load.XXXXXX")
    python3 - "$BASE_3MF" "$TOWER_DIR/far.3mf" <<'PY'
import json, sys, zipfile
with zipfile.ZipFile(sys.argv[1]) as zin, zipfile.ZipFile(sys.argv[2], "w", zipfile.ZIP_DEFLATED) as zout:
    for item in zin.infolist():
        data = zin.read(item.filename)
        if item.filename == "Metadata/project_settings.config":
            d = json.loads(data)
            d["wipe_tower_x"] = ["400"]
            data = json.dumps(d, indent=4).encode()
        zout.writestr(item, data)
PY
    tower_x() { sed -n 's/^; wipe_tower_x = //p' "$1" | head -n 1 | tr -d '\r'; }
    inside() { python3 -c 'import sys; x = float(sys.argv[1].split(",")[0]); sys.exit(0 if 0 < x < 300 else 1)' "$1" 2>/dev/null; }
    is_value() { python3 -c 'import sys; sys.exit(0 if float(sys.argv[1].split(",")[0]) == float(sys.argv[2]) else 1)' "$1" "$2" 2>/dev/null; }
    GC=$(mktmp_gcode)
    run_slice "$GC" "$TOWER_DIR/far.3mf"
    PLAIN=$(tower_x "$GC")
    set +e
    "$BINARY" "$TOWER_DIR/far.3mf" --slice 1 --outputdir "$TOWER_DIR/s" --export-3mf p.3mf > "$TOWER_DIR/s.out" 2>&1; SLICE_EXIT=$?
    set -e
    SLICED=$(tower_x "$TOWER_DIR/s/plate_1.gcode" 2>/dev/null || true)
    EXPORTED=$(python3 -c '
import json, sys, zipfile
print(json.loads(zipfile.ZipFile(sys.argv[1]).read("Metadata/project_settings.config"))["wipe_tower_x"][0])' "$TOWER_DIR/s/p.3mf" 2>/dev/null || true)
    if [ "$ENGINE" = bambu ]; then
        if [ "$LAST_EXIT" -eq 0 ] && [ "$SLICE_EXIT" -eq 0 ] && inside "$PLAIN" && is_value "$SLICED" "$PLAIN" && is_value "$EXPORTED" "$PLAIN"; then
            record "project-tower-put-inside-plate" 1
        else
            record "project-tower-put-inside-plate" 0 "plain call '$PLAIN' (exit $LAST_EXIT), --slice '$SLICED' (exit $SLICE_EXIT), exported '$EXPORTED': want one position inside the plate"
        fi
        run_slice "$GC" "$TOWER_DIR/far.3mf" --wipe-tower-x 5
        FLAG=$(tower_x "$GC")
        if [ "$LAST_EXIT" -eq 0 ] && is_value "$FLAG" 5; then
            record "project-tower-flag-stays-as-given" 1
        else
            record "project-tower-flag-stays-as-given" 0 "exit=$LAST_EXIT wipe_tower_x='$FLAG' (want 5)"
        fi
    elif [ "$LAST_EXIT" -eq 0 ] && [ "$SLICE_EXIT" -eq 0 ] && is_value "$PLAIN" 400 && is_value "$SLICED" 400; then
        record "project-tower-kept-as-saved" 1
    else
        record "project-tower-kept-as-saved" 0 "plain call '$PLAIN' (exit $LAST_EXIT), --slice '$SLICED' (exit $SLICE_EXIT): want 400, as the app keeps it"
    fi
    rm -rf "$GC" "$TOWER_DIR"
else
    skip "project-tower-*" "fixture missing: $BASE_3MF"
fi

# ── An exported project slices again with the same settings ─────────────────
# The export lists the settings the run changed, as the app lists them when it
# saves (src/desktop_presets.hpp record_changed_settings), so opening the export
# keeps them. Each case slices with --export-3mf, slices the export again with
# no flags, and compares the two G-codes: the changed settings in both
# headers, the same layer count and the same filament use to 1 %. The printed
# lines themselves are not compared: one build slicing one file twice does not
# always write the same lines (the engine's threads), which a Linux arm64 run
# of this check showed as 10137 against 10175 lines for one print.
RT=$(mktemp -d "${TMPDIR:-/tmp}/roundtrip.XXXXXX")
python3 - "$RT/cube.stl" <<'PY'
import sys
v = [(x, y, z) for x in (0, 20) for y in (0, 20) for z in (0, 20)]
f = [(0,1,3),(0,3,2),(4,6,7),(4,7,5),(0,4,5),(0,5,1),(2,3,7),(2,7,6),(0,2,6),(0,6,4),(1,5,7),(1,7,3)]
with open(sys.argv[1], "w") as o:
    o.write("solid cube\n")
    for a, b, c in f:
        o.write("facet normal 0 0 0\nouter loop\n")
        for i in (a, b, c):
            o.write("vertex %d %d %d\n" % v[i])
        o.write("endloop\nendfacet\n")
    o.write("endsolid cube\n")
PY
RT_PRINTER="Bambu Lab X1 Carbon 0.4 nozzle"
[ "$ENGINE" = snapmaker ] && RT_PRINTER="Snapmaker U1 (0.4 nozzle)"
roundtrip() {
    local LABEL="$1"; shift
    local D="$RT/$LABEL"; mkdir -p "$D"
    set +e
    "$BINARY" "$@" --slice 1 --outputdir "$D/a" --export-3mf p.3mf > "$D/a.out" 2>&1; local R1=$?
    "$BINARY" "$D/a/p.3mf" --slice 1 --outputdir "$D/b" > "$D/b.out" 2>&1; local R2=$?
    set -e
    local DETAIL
    DETAIL=$(python3 - "$D/a/plate_1.gcode" "$D/b/plate_1.gcode" 2>&1 <<'PY'
import re, sys
def read(path):
    header, layers, grams = {}, 0, None
    for line in open(path, errors="replace"):
        line = line.rstrip("\r\n")
        m = re.match(r"^; ([a-z_0-9]+) = (.*)$", line)
        if m:
            header.setdefault(m.group(1), m.group(2))
        if re.match(r"^; ?(CHANGE_LAYER|LAYER_CHANGE)", line):
            layers += 1
        m = re.match(r"^; (?:total filament weight \[g\] ?:|filament used \[g\] ?=) *(.*)$", line)
        if m and grams is None:
            grams = sum(float(v) for v in m.group(1).split(","))
    return header, layers, grams
(a, la, ga), (b, lb, gb) = read(sys.argv[1]), read(sys.argv[2])
for name, h in (("the first slice", a), ("the export sliced again", b)):
    assert h.get("wall_loops") == "5" and h.get("sparse_infill_density") == "33%", (name, h.get("wall_loops"), h.get("sparse_infill_density"))
differ = sorted(k for k in a if k in b and a[k] != b[k])
assert la > 0 and la == lb, ("layers", la, lb, "settings that differ", differ)
assert ga and gb and abs(ga - gb) <= 0.01 * ga, ("filament grams", ga, gb, "settings that differ", differ)
PY
    ) && [ "$R1" -eq 0 ] && [ "$R2" -eq 0 ] && record "export-slices-again-$LABEL" 1 \
        || record "export-slices-again-$LABEL" 0 "export exit=$R1 re-slice exit=$R2 $DETAIL"
}
roundtrip "model-with-flags" "$RT/cube.stl" --printer-preset "$RT_PRINTER" --perimeters 5 --infill 33%
if [ -f "$BASE_3MF" ]; then
    roundtrip "project-with-flags" "$BASE_3MF" --perimeters 5 --infill 33%
    roundtrip "project-as-saved" "$BASE_3MF" --project-settings-as-saved --perimeters 5 --infill 33%
fi
rm -rf "$RT"

# ── Summary ─────────────────────────────────────────────────────────────────
echo ""
echo "Results ($ENGINE): $PASS passed, $FAIL failed, $SKIP skipped"
[ $FAIL -eq 0 ]   # exit 0 iff all passed
