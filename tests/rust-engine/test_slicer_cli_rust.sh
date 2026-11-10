#!/usr/bin/env bash
# slicer_cli-rust: the call it takes, the flags it refuses, its return codes,
# and one slice of each input kind.
#
# Usage:  tests/rust-engine/test_slicer_cli_rust.sh <slicer_cli-rust>
set -uo pipefail

BIN="${1:?usage: $0 <slicer_cli-rust>}"
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
DATA="$ROOT/crates/libslic3r-rs/tests/data"
TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT
PASS=0; FAIL=0

ok()   { PASS=$((PASS + 1)); echo "PASS [$1]"; }
fail() { FAIL=$((FAIL + 1)); echo "FAIL [$1] $2"; }

# expect_exit NAME WANT -- command...
expect_exit() {
    local name="$1" want="$2"; shift 3
    "$@" >"$TMP/out" 2>"$TMP/err"; local got=$?
    if [ "$got" -eq "$want" ]; then ok "$name"; else fail "$name" "exit $got, want $want: $(head -c 300 "$TMP/err")"; fi
}

expect_exit help 0 -- "$BIN" --help
grep -q -- '--config FILE' "$TMP/out" && ok help-names-config || fail help-names-config "no --config in --help"

# A flag of slicer_cli that this program does not take: refused by name, with
# CLI_INVALID_PARAMS (-2, exit status 254), pointing at slicer_cli.
for flag in --slice --arrange --load-settings --export-3mf --gcode-timing --simplify --plate --engine-info; do
    expect_exit "refuses$flag" 254 -- "$BIN" "$flag" 1 "$DATA/painted_cube.3mf"
    if grep -q "^$flag is not a flag of slicer_cli-rust" "$TMP/err" && grep -q 'Use slicer_cli ' "$TMP/err"; then
        ok "sentence$flag"
    else
        fail "sentence$flag" "$(cat "$TMP/err")"
    fi
done

# The refusal comes before anything loads: the model file here does not exist.
expect_exit refused-before-loading 254 -- "$BIN" "$TMP/no-such.stl" --orient
expect_exit missing-model 253 -- "$BIN" "$TMP/no-such.stl"
expect_exit no-model 254 -- "$BIN" -o "$TMP/x.gcode"
expect_exit stl-needs-config 254 -- "$BIN" "$ROOT/tests/rust-engine/fixtures/Cube_25.6.stl"
expect_exit step-is-refused 254 -- "$BIN" "$ROOT/README.md"

# A slice of each input kind.
check_gcode() {
    local name="$1" file="$2"
    if [ -s "$file" ] && grep -q '^; FEATURE: Outer wall' "$file" && grep -q '^G1 .*E' "$file"; then
        ok "$name"
    else
        fail "$name" "no usable G-code in $file"
    fi
}
expect_exit slice-3mf 0 -- "$BIN" -o "$TMP/3mf/cube.gcode" "$DATA/painted_cube.3mf"
check_gcode slice-3mf-gcode "$TMP/3mf/cube.gcode"
expect_exit slice-stl 0 -- "$BIN" --config "$DATA/cube_arachne_settings.json" --output="$TMP/stl.gcode" "$ROOT/tests/rust-engine/fixtures/Cube_25.6.stl"
check_gcode slice-stl-gcode "$TMP/stl.gcode"

# The same input gives the same bytes.
"$BIN" -o "$TMP/again.gcode" "$DATA/painted_cube.3mf" >/dev/null 2>&1
if cmp -s "$TMP/3mf/cube.gcode" "$TMP/again.gcode"; then ok deterministic; else fail deterministic "two runs differ"; fi

echo "Results: $PASS passed, $FAIL failed"
[ "$FAIL" -eq 0 ]
