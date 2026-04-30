#!/usr/bin/env bash
# bundle-macos.sh — make slicer_cli relocatable on macOS.
#
# After `cmake --build .`, the binary has Homebrew absolute rpaths baked in:
#   @rpath/libTBB.dylib → /opt/homebrew/opt/tbb/lib/libTBB.dylib
#
# This script:
#   1. Copies the external shared dylibs needed at runtime into a Frameworks/
#      directory next to the binary.
#   2. Rewrites the binary's load commands to use @executable_path/../Frameworks/
#      instead of absolute Homebrew paths.
#
# Usage (run from the cli/build directory after cmake --build .):
#   bash ../scripts/bundle-macos.sh slicer_cli [output_dir]
#
# Output: <output_dir>/slicer_cli   (default: dist/)
#                     /Frameworks/  (bundled dylibs)
#
# Verification: run <output_dir>/slicer_cli --version in a subshell that clears
# the Homebrew dyld paths to simulate a clean host.

set -euo pipefail

BINARY="${1:?Usage: $0 <path-to-slicer_cli> [output_dir]}"
OUTPUT="${2:-dist}"
FRAMEWORKS="$OUTPUT/Frameworks"

mkdir -p "$OUTPUT" "$FRAMEWORKS"
cp "$BINARY" "$OUTPUT/slicer_cli"

# ── Collect non-system shared deps ─────────────────────────────────────────
# We recurse through otool -L until no new libs are discovered.
# "Non-system" = anything NOT under /usr/lib or /System/Library.

declare -A SEEN
QUEUE=("$OUTPUT/slicer_cli")

while [ "${#QUEUE[@]}" -gt 0 ]; do
    CURRENT="${QUEUE[0]}"
    QUEUE=("${QUEUE[@]:1}")

    while IFS= read -r LIB; do
        LIB="$(echo "$LIB" | awk '{print $1}')"
        # Skip self, @rpath placeholders, system libs
        [[ "$LIB" == @* ]] && continue
        [[ "$LIB" =~ ^/usr/lib/ ]] && continue
        [[ "$LIB" =~ ^/System/ ]] && continue
        [[ -z "$LIB" ]] && continue
        [[ -n "${SEEN[$LIB]+x}" ]] && continue
        SEEN["$LIB"]=1
        # Resolve @rpath to actual absolute path if needed
        RESOLVED="$LIB"
        if [[ ! -f "$RESOLVED" ]]; then
            # Try resolving via DYLD_FALLBACK_LIBRARY_PATH
            for DIR in /opt/homebrew/lib /usr/local/lib; do
                CANDIDATE="$DIR/$(basename "$LIB")"
                if [[ -f "$CANDIDATE" ]]; then
                    RESOLVED="$CANDIDATE"
                    break
                fi
            done
        fi
        [[ ! -f "$RESOLVED" ]] && { echo "WARNING: cannot resolve $LIB"; continue; }
        DEST="$FRAMEWORKS/$(basename "$RESOLVED")"
        [[ -f "$DEST" ]] && continue
        echo "Bundling: $RESOLVED → $DEST"
        cp "$RESOLVED" "$DEST"
        QUEUE+=("$DEST")
    done < <(otool -L "$CURRENT" 2>/dev/null | tail -n +2)
done

# ── Rewrite load commands ───────────────────────────────────────────────────
rewrite_refs() {
    local TARGET="$1"
    chmod +w "$TARGET"
    for SRC_LIB in "${!SEEN[@]}"; do
        local BASENAME
        BASENAME="$(basename "$SRC_LIB")"
        install_name_tool -change "$SRC_LIB" \
            "@executable_path/../Frameworks/$BASENAME" \
            "$TARGET" 2>/dev/null || true
    done
    # Also fix @rpath references
    while IFS= read -r RPATH_LIB; do
        RPATH_LIB="$(echo "$RPATH_LIB" | awk '{print $1}')"
        [[ "$RPATH_LIB" != @rpath/* ]] && continue
        local BASENAME="${RPATH_LIB#@rpath/}"
        [[ -f "$FRAMEWORKS/$BASENAME" ]] && \
            install_name_tool -change "$RPATH_LIB" \
                "@executable_path/../Frameworks/$BASENAME" \
                "$TARGET" 2>/dev/null || true
    done < <(otool -L "$TARGET" 2>/dev/null | tail -n +2)
    # Remove all rpaths that point to Homebrew
    while IFS= read -r RPATH; do
        RPATH="${RPATH//\(.*\)/}"
        RPATH="${RPATH//[[:space:]]/}"
        [[ "$RPATH" =~ homebrew|Homebrew ]] && \
            install_name_tool -delete_rpath "$RPATH" "$TARGET" 2>/dev/null || true
    done < <(otool -l "$TARGET" 2>/dev/null | grep -A2 'LC_RPATH' | grep 'path' | awk '{print $2}')
}

rewrite_refs "$OUTPUT/slicer_cli"
for DYLIB in "$FRAMEWORKS"/*.dylib; do
    rewrite_refs "$DYLIB"
done

echo ""
echo "Relocatable bundle written to $OUTPUT/"
echo ""
echo "Verify on a clean host:"
echo "  env -i PATH=/usr/bin:/bin DYLD_FALLBACK_LIBRARY_PATH= $OUTPUT/slicer_cli --version"
