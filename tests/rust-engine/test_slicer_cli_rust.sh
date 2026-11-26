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
for flag in --slice --arrange --load-settings --export-3mf --simplify --plate --engine-info; do
    expect_exit "refuses$flag" 254 -- "$BIN" "$flag" 1 "$DATA/painted_cube.3mf"
    if grep -q "^$flag is not a flag of slicer_cli-rust" "$TMP/err" && grep -q 'Use slicer_cli ' "$TMP/err"; then
        ok "sentence$flag"
    else
        fail "sentence$flag" "$(cat "$TMP/err")"
    fi
done

expect_exit timing-missing-input 1 -- "$BIN" --gcode-timing "$TMP/missing.gcode" "$TMP/missing.gcode"
grep -q '"error_code":"INPUT_UNREADABLE"' "$TMP/out" && test ! -s "$TMP/err" && ok timing-error-json || fail timing-error-json "$(cat "$TMP/out" "$TMP/err")"

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

# A job Print::validate() refuses is refused as slicer_cli refuses it
# (main.cpp:11466-11636): the validation_error event on stdout, "Validation
# error: <sentence>" and the tip on stderr, exit status 1. Here the layer height
# (0.5 mm) is over the nozzle diameter (0.4 mm).
sed 's/"layer_height": "0.2"/"layer_height": "0.5"/' "$DATA/cube_arachne_settings.json" > "$TMP/lh05.json"
expect_exit validation-refused 1 -- "$BIN" --config "$TMP/lh05.json" -o "$TMP/lh05.gcode" "$ROOT/tests/rust-engine/fixtures/Cube_25.6.stl"
if grep -q '^Validation error: Layer height cannot exceed nozzle diameter$' "$TMP/err" \
    && grep -q '^  --machine, --filament, and --process options$' "$TMP/err" \
    && grep -qF '[[SLICER_EVENT]] {"event":"validation_error","message":"Layer height cannot exceed nozzle diameter","opt_key":"layer_height","tag":"STRING_EXCEPT_NOT_DEFINED"}' "$TMP/out"; then
    ok validation-sentence
else
    fail validation-sentence "$(cat "$TMP/out" "$TMP/err")"
fi

# Print::validate checks support_line_width only for an object with support or
# a raft (Print.cpp:1795-1798): slicer_cli slices this job, so does this program.
sed 's/"support_line_width": "[0-9.]*"/"support_line_width": "0.1"/' "$DATA/cube_arachne_settings.json" > "$TMP/slw01.json"
expect_exit support-width-unused 0 -- "$BIN" --config "$TMP/slw01.json" -o "$TMP/slw01.gcode" "$ROOT/tests/rust-engine/fixtures/Cube_25.6.stl"

# Print::validate checks every region's line widths, internal solid infill
# among them (Print.cpp:1796-1800): slicer_cli refuses an internal solid infill
# width of 0.1 mm under a 0.2 mm layer with this event and sentence.
sed 's/"internal_solid_infill_line_width": "[0-9.]*"/"internal_solid_infill_line_width": "0.1"/' "$DATA/cube_arachne_settings.json" > "$TMP/isw01.json"
expect_exit internal-solid-width-refused 1 -- "$BIN" --config "$TMP/isw01.json" -o "$TMP/isw01.gcode" "$ROOT/tests/rust-engine/fixtures/Cube_25.6.stl"
if grep -q '^Validation error: Too small line width$' "$TMP/err" \
    && grep -qF '[[SLICER_EVENT]] {"event":"validation_error","message":"Too small line width","opt_key":"internal_solid_infill_line_width","tag":"STRING_EXCEPT_NOT_DEFINED"}' "$TMP/out"; then
    ok internal-solid-width-sentence
else
    fail internal-solid-width-sentence "$(cat "$TMP/out" "$TMP/err")"
fi

# A filament whose bed temperature for the plate type is 0 cannot print on that
# plate (Print.cpp:1805-1832): slicer_cli refuses the job with the plate, the
# filament and the type STRING_EXCEPT_FILAMENT_NOT_MATCH_BED_TYPE.
sed 's/"textured_plate_temp": \[[^]]*\]/"textured_plate_temp": ["0"]/' "$DATA/cube_arachne_settings.json" > "$TMP/tpt0.json"
expect_exit bed-type-refused 1 -- "$BIN" --config "$TMP/tpt0.json" -o "$TMP/tpt0.gcode" "$ROOT/tests/rust-engine/fixtures/Cube_25.6.stl"
if grep -q '^Validation error: Plate 1: Textured PEI Plate does not support filament 1$' "$TMP/err" \
    && grep -qF '[[SLICER_EVENT]] {"event":"validation_error","message":"Plate 1: Textured PEI Plate does not support filament 1\n","params":["1","Textured PEI Plate","1"],"tag":"STRING_EXCEPT_FILAMENT_NOT_MATCH_BED_TYPE"}' "$TMP/out"; then
    ok bed-type-sentence
else
    fail bed-type-sentence "$(cat "$TMP/out" "$TMP/err")"
fi

# A validate() warning does not stop the slice; slicer_cli emits it as the
# validation event and "Validation warning: <sentence>" on stdout
# (main.cpp:11483-11494, 11528-11531). The precise outer wall warning
# (Print.cpp:1834-1840) leaves is_warning false, so its event is
# "validation_error".
sed '1s/^{/{ "precise_outer_wall": "1", "wall_sequence": "outer wall\/inner wall",/' "$DATA/cube_arachne_settings.json" > "$TMP/pow.json"
expect_exit precise-outer-wall-warned 0 -- "$BIN" --config "$TMP/pow.json" -o "$TMP/pow.gcode" "$ROOT/tests/rust-engine/fixtures/Cube_25.6.stl"
if grep -q '^Validation warning: Precise outer wall will be ignored unless the order of walls is set to inner/outer$' "$TMP/out" \
    && grep -qF '[[SLICER_EVENT]] {"event":"validation_error","message":"Precise outer wall will be ignored unless the order of walls is set to inner/outer","opt_key":"precise_outer_wall","tag":"STRING_EXCEPT_NOT_DEFINED"}' "$TMP/out"; then
    ok precise-outer-wall-warning
else
    fail precise-outer-wall-warning "$(cat "$TMP/out")"
fi

# The bed check reads every filament the print uses, the support filament among
# them (Print::extruders, Print.cpp:523-576): two filaments, support printed
# with filament 2, whose Textured PEI Plate temperature is 0. slicer_cli refuses
# it naming filament 2.
python3 - "$DATA/cube_arachne_settings.json" "$TMP/sup2.json" <<'EOF'
import json, sys
d = json.load(open(sys.argv[1]))
for k, v in list(d.items()):
    if isinstance(v, list) and len(v) == 1 and (k.startswith('filament') or k.startswith('nozzle_temperature') or '_plate_temp' in k):
        d[k] = v * 2
d.update({'filament_colour': ['#FF0000', '#00FF00'], 'textured_plate_temp': ['55', '0'],
          'textured_plate_temp_initial_layer': ['55', '0'], 'enable_support': '1',
          'support_filament': '2', 'support_interface_filament': '2'})
json.dump(d, open(sys.argv[2], 'w'), indent=4)
EOF
expect_exit support-filament-bed-refused 1 -- "$BIN" --config "$TMP/sup2.json" -o "$TMP/sup2.gcode" "$ROOT/tests/rust-engine/fixtures/Cube_25.6.stl"
if grep -qF '[[SLICER_EVENT]] {"event":"validation_error","message":"Plate 1: Textured PEI Plate does not support filament 2\n","params":["1","Textured PEI Plate","2"],"tag":"STRING_EXCEPT_FILAMENT_NOT_MATCH_BED_TYPE"}' "$TMP/out"; then
    ok support-filament-bed-sentence
else
    fail support-filament-bed-sentence "$(cat "$TMP/out" "$TMP/err")"
fi

# The settings' values are checked before anything slices (main.cpp:9438-9593,
# DynamicPrintConfig::validate(true), PrintConfig.cpp:10641-10916): spiral vase
# mode with two walls, 15% infill and five top layers is refused as slicer_cli
# refuses it, with the config_refused event, the sentence on stderr and exit 1.
sed 's/"spiral_mode": "0"/"spiral_mode": "1"/' "$DATA/cube_arachne_settings.json" > "$TMP/spiral.json"
expect_exit spiral-config-refused 1 -- "$BIN" --config "$TMP/spiral.json" -o "$TMP/spiral.gcode" "$ROOT/tests/rust-engine/fixtures/Cube_25.6.stl"
spiral_sentence='BambuStudio cannot print this file as it is: sparse_infill_density: Invalid value when spiral vase mode is enabled: 15.000000; top_shell_layers: Invalid value when spiral vase mode is enabled: 5; wall_loops: Invalid value when spiral vase mode is enabled: 2. Give --sparse-infill-density, --top-shell-layers, --wall-loops a value in range to override it. --allow-substitution lets the engine use its own default for a value it does not know instead.'
if grep -qxF "Param values in 3mf/config error: $spiral_sentence" "$TMP/err" \
    && grep -qxF "[[SLICER_EVENT]] {\"event\":\"config_refused\",\"message\":\"$spiral_sentence\",\"settings\":{\"sparse_infill_density\":\"Invalid value when spiral vase mode is enabled: 15.000000\",\"top_shell_layers\":\"Invalid value when spiral vase mode is enabled: 5\",\"wall_loops\":\"Invalid value when spiral vase mode is enabled: 2\"},\"tag\":\"InvalidValues\",\"unknown_values\":[]}" "$TMP/out"; then
    ok spiral-config-sentence
else
    fail spiral-config-sentence "$(cat "$TMP/out" "$TMP/err")"
fi

# A value out of range that the desktop's own check finds the same way is the
# file's: slicer_cli warns (InvalidValuesNoted) and slices.
sed 's/"tree_support_wall_count": "-1"/"tree_support_wall_count": "-2"/' "$DATA/cube_arachne_settings.json" > "$TMP/tswc.json"
expect_exit out-of-range-noted 0 -- "$BIN" --config "$TMP/tswc.json" -o "$TMP/tswc.gcode" "$ROOT/tests/rust-engine/fixtures/Cube_25.6.stl"
noted_sentence="Invalid values found in the 3MF/config: tree_support_wall_count: -2 not in range [-1.000000,2.000000]. The run slices with them as they are, and the slice's own check of each object against its nozzle and plate still applies. Give --tree-support-wall-count a value in range to change it."
if grep -qxF "Warning: $noted_sentence" "$TMP/err" \
    && grep -qxF "[[SLICER_EVENT]] {\"event\":\"warning\",\"message\":\"$noted_sentence\",\"plate_id\":1,\"settings\":{\"tree_support_wall_count\":\"-2 not in range [-1.000000,2.000000]\"},\"tag\":\"InvalidValuesNoted\"}" "$TMP/out"; then
    ok out-of-range-noted-sentence
else
    fail out-of-range-noted-sentence "$(cat "$TMP/out" "$TMP/err")"
fi

# The same input gives the same bytes.
"$BIN" -o "$TMP/again.gcode" "$DATA/painted_cube.3mf" >/dev/null 2>&1
if cmp -s "$TMP/3mf/cube.gcode" "$TMP/again.gcode"; then ok deterministic; else fail deterministic "two runs differ"; fi

echo "Results: $PASS passed, $FAIL failed"
[ "$FAIL" -eq 0 ]
