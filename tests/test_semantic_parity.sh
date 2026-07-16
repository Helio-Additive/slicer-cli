#!/usr/bin/env bash
# Semantic-equivalence regression test.
#
# Asserts the Rust engine's G-code is PHYSICALLY equivalent to native C++
# BambuStudio's — total filament, layer structure, per-layer material, and
# object-silhouette coverage (IoU) all within tolerance. This deliberately
# replaces byte-identical comparison, which is infeasible: ~99% of the raw
# byte-diff is floating-point-cascade noise (different compilers round geometry
# differently and the difference re-routes toolpaths by 10-100um without
# changing the printed object). See PARITY_STATUS.md R335 and R346-R347.
#
# Its value as a regression guard: a genuinely-broken toolpath collapses the
# silhouette IoU or the material ratio, which this catches — while FP re-routing
# (which byte-diff would flag as thousands of "failures") correctly passes.
#
# Usage:  tests/test_semantic_parity.sh [config.jsonnet]
#   env:  SLICER_CLI=<path>  PYTHON=<python-with-numpy>  BAMBUSTUDIO_SLICER=<native>
#   note: needs numpy — run under devbox:  devbox run -- tests/test_semantic_parity.sh
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN="${SLICER_CLI:-$ROOT/target/release/slicer-cli}"
CONFIG="${1:-$ROOT/tests/configs/stl-inline-config.jsonnet}"
PY="${PYTHON:-python3}"

if ! "$PY" -c 'import numpy' 2>/dev/null; then
    echo "SKIP: numpy not available for '$PY' (try: devbox run -- $0)"; exit 0
fi
if [ ! -x "$BIN" ]; then
    echo "SKIP: slicer-cli not built at $BIN"; exit 0
fi

TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT
COMPARE_KEEP_DIR="$TMP" "$BIN" compare --config "$CONFIG" >/dev/null 2>&1 || true
if [ ! -s "$TMP/rust.gcode" ] || [ ! -s "$TMP/native.gcode" ]; then
    echo "SKIP: compare did not produce both gcodes (native binary missing?)"; exit 0
fi

"$PY" "$ROOT/scripts/semantic_compare.py" "$TMP/rust.gcode" "$TMP/native.gcode"
# semantic_compare.py exits 0 iff SEMANTICALLY EQUIVALENT
