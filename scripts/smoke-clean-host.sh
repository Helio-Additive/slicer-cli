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

for TARGET in "$BINARY" "$(dirname "$BINARY")"/Frameworks/*.dylib; do
    [ -e "$TARGET" ] || continue
    if otool -L "$TARGET" | grep -E '/opt/homebrew|/usr/local/(Cellar|opt)|/(build|\.cache|_temp)/'; then
        echo "FAIL: non-relocatable dependency in $TARGET"
        exit 1
    fi
done

# Clear every Homebrew dyld hint
CLEAN_ENV=(
    env -i
    PATH="/usr/bin:/bin:/usr/sbin:/sbin"
    DYLD_FALLBACK_LIBRARY_PATH=""
    DYLD_LIBRARY_PATH=""
    HOME="$HOME"
)

set +e
OUTPUT=$("${CLEAN_ENV[@]}" "$BINARY" --help 2>&1)
EXIT_CODE=$?
set -e

echo "Exit code: $EXIT_CODE"
echo "Output:    $OUTPUT"

if [ "$EXIT_CODE" -ne 0 ]; then
    echo "FAIL: slicer_cli --help failed on the clean environment (exit $EXIT_CODE)"
    exit 1
fi

if echo "$OUTPUT" | grep -qi "Library not loaded\|image not found"; then
    echo "FAIL: dyld error in output"
    exit 1
fi

echo "PASS: binary ran successfully on clean host"
exit 0
