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
# Run without arguments — binary prints help and exits non-zero, which is fine.
# We only care that it ran (all dylibs loaded). Crash signals (SIGABRT=134,
# SIGSEGV=139) indicate a missing bundled dylib; any other exit code means success.
OUTPUT=$("${CLEAN_ENV[@]}" "$BINARY" 2>&1)
EXIT_CODE=$?
set -e

echo "Exit code: $EXIT_CODE"
echo "Output:    $OUTPUT"

if [ $EXIT_CODE -eq 134 ] || [ $EXIT_CODE -eq 139 ]; then
    echo "FAIL: slicer_cli crashed (signal, exit $EXIT_CODE) — likely a missing bundled dylib"
    echo ""
    echo "This usually means a Homebrew dylib was not bundled."
    echo "Run scripts/bundle-macos.sh first, then re-run this script."
    exit 1
fi

if echo "$OUTPUT" | grep -qi "Library not loaded\|image not found"; then
    echo "FAIL: dyld error in output"
    exit 1
fi

echo "PASS: binary ran without crash on clean host (exit $EXIT_CODE)"
exit 0
