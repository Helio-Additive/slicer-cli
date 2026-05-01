#!/usr/bin/env bash
# bundle-macos.sh — make slicer_cli relocatable on macOS.
#
# After `cmake --build .`, the binary has Homebrew absolute rpaths baked in:
#   @rpath/libTBB.dylib → /opt/homebrew/opt/tbb/lib/libTBB.dylib
#
# This script:
#   1. Copies the external shared dylibs needed at runtime into a Frameworks/
#      directory next to the binary.
#   2. Rewrites the binary's load commands to use @executable_path/Frameworks/
#      instead of absolute Homebrew paths.
#
# Usage:
#   bash scripts/bundle-macos.sh build/slicer_cli [output_dir]
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

# ── Global rpath pool ───────────────────────────────────────────────────────
# Collect ALL absolute LC_RPATH entries from the main binary up-front.
# These are used as a fallback when per-file @rpath resolution fails (e.g.
# transitive GCC runtime libs whose copies in Frameworks/ have @loader_path
# rpaths that become meaningless after the copy).
declare -a GLOBAL_RPATH_DIRS=()
while IFS= read -r RP; do
    RP="${RP//\(.*\)/}"
    RP="${RP//[[:space:]]/}"
    [[ "$RP" == @* ]] && continue
    [[ -d "$RP" ]] && GLOBAL_RPATH_DIRS+=("$RP")
done < <(otool -l "$OUTPUT/slicer_cli" 2>/dev/null | grep -A2 'LC_RPATH' | grep 'path' | awk '{print $2}')

# ── Collect non-system shared deps ─────────────────────────────────────────
# Recurse through otool -L output until no new deps are found.
# "Non-system" = anything NOT under /usr/lib or /System/Library.

declare -A SEEN
QUEUE=("$OUTPUT/slicer_cli")

resolve_rpath() {
    local BASENAME="$1" CURRENT_FILE="$2"
    # 1. Try absolute LC_RPATH entries of the current file
    while IFS= read -r RP; do
        RP="${RP//\(.*\)/}"
        RP="${RP//[[:space:]]/}"
        [[ "$RP" == @* ]] && continue
        [[ -f "$RP/$BASENAME" ]] && echo "$RP/$BASENAME" && return 0
    done < <(otool -l "$CURRENT_FILE" 2>/dev/null | grep -A2 'LC_RPATH' | grep 'path' | awk '{print $2}' || true)
    # 2. Fall back to global rpath pool (main binary's absolute rpaths)
    for DIR in "${GLOBAL_RPATH_DIRS[@]}"; do
        [[ -f "$DIR/$BASENAME" ]] && echo "$DIR/$BASENAME" && return 0
    done
    return 0  # not found — caller checks for empty FOUND
}

while [ "${#QUEUE[@]}" -gt 0 ]; do
    CURRENT="${QUEUE[0]}"
    QUEUE=("${QUEUE[@]:1}")

    while IFS= read -r LIB; do
        LIB="$(echo "$LIB" | awk '{print $1}')"
        [[ -z "$LIB" ]] && continue
        # Resolve @rpath/X: first try per-file LC_RPATH, then global pool.
        if [[ "$LIB" == @rpath/* ]]; then
            BASENAME="${LIB#@rpath/}"
            FOUND="$(resolve_rpath "$BASENAME" "$CURRENT")"
            [[ -z "$FOUND" ]] && continue
            LIB="$FOUND"
        fi
        # Skip remaining @-prefixed refs, system libs
        [[ "$LIB" == @* ]] && continue
        [[ "$LIB" =~ ^/usr/lib/ ]] && continue
        [[ "$LIB" =~ ^/System/ ]] && continue
        [[ -n "${SEEN[$LIB]+x}" ]] && continue
        SEEN["$LIB"]=1
        [[ ! -f "$LIB" ]] && { echo "WARNING: cannot find $LIB"; continue; }
        DEST="$FRAMEWORKS/$(basename "$LIB")"
        [[ -f "$DEST" ]] && continue
        echo "Bundling: $LIB → $DEST"
        cp "$LIB" "$DEST"
        QUEUE+=("$DEST")
    done < <(otool -L "$CURRENT" 2>/dev/null | tail -n +2)
done

# ── Rewrite load commands ───────────────────────────────────────────────────
rewrite_refs() {
    local TARGET="$1"
    chmod +w "$TARGET"
    # Rewrite absolute Homebrew paths
    for SRC_LIB in "${!SEEN[@]}"; do
        local BASENAME
        BASENAME="$(basename "$SRC_LIB")"
        install_name_tool -change "$SRC_LIB" \
            "@executable_path/Frameworks/$BASENAME" \
            "$TARGET" 2>/dev/null || true
    done
    # Rewrite @rpath/X references for bundled libs
    while IFS= read -r RPATH_LIB; do
        RPATH_LIB="$(echo "$RPATH_LIB" | awk '{print $1}')"
        [[ "$RPATH_LIB" != @rpath/* ]] && continue
        local BASENAME="${RPATH_LIB#@rpath/}"
        [[ -f "$FRAMEWORKS/$BASENAME" ]] && \
            install_name_tool -change "$RPATH_LIB" \
                "@executable_path/Frameworks/$BASENAME" \
                "$TARGET" 2>/dev/null || true
    done < <(otool -L "$TARGET" 2>/dev/null | tail -n +2)
    # Remove Homebrew rpaths (now replaced with @executable_path refs)
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
