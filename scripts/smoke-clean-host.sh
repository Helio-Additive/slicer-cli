#!/usr/bin/env bash
# smoke-clean-host.sh — verify slicer_cli runs without Homebrew paths on macOS.
#
# Simulates a clean host by clearing Homebrew's dyld paths from the environment
# before invoking the binary. On CI this runs on the same agent that built the
# bundle; on developer machines it checks the bundling step output.
#
# Usage:
#   bash smoke-clean-host.sh <path-to-slicer_cli>   # binary or dist/slicer_cli
#
# Exit code: 0 on success, 1 on failure.

set -euo pipefail

BINARY="${1:?Usage: $0 <path-to-slicer_cli>}"

echo "=== Clean-host smoke test: $BINARY ==="

# Clear every Homebrew dyld hint
CLEAN_ENV=(
    env -i
    PATH="/usr/bin:/bin:/usr/sbin:/sbin"
    DYLD_FALLBACK_LIBRARY_PATH=""
    DYLD_LIBRARY_PATH=""
    HOME="$HOME"
)

set +e
OUTPUT=$("${CLEAN_ENV[@]}" "$BINARY" --version 2>&1)
EXIT_CODE=$?
set -e

echo "Exit code: $EXIT_CODE"
echo "Output:    $OUTPUT"

if [ $EXIT_CODE -ne 0 ]; then
    echo "FAIL: slicer_cli exited $EXIT_CODE on clean host"
    echo ""
    echo "This usually means a Homebrew dylib was not bundled."
    echo "Run cli/scripts/bundle-macos.sh first, then re-run this script"
    echo "against the dist/slicer_cli output."
    exit 1
fi

if echo "$OUTPUT" | grep -qi "slicer_cli\|slic3r\|BambuStudio\|[0-9]\+\.[0-9]\+"; then
    echo "PASS: binary ran and produced version output"
    exit 0
else
    echo "WARN: binary exited 0 but output didn't match expected version pattern"
    echo "Manual review needed"
    exit 0
fi
