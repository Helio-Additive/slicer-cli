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
declare -a GLOBAL_RPATH_DIRS=("__slicer_cli_rpath_sentinel__")
while IFS= read -r RP; do
    RP="${RP//\(.*\)/}"
    RP="${RP//[[:space:]]/}"
    [[ "$RP" == @* ]] && continue
    [[ -d "$RP" ]] && GLOBAL_RPATH_DIRS+=("$RP")
done < <(otool -l "$OUTPUT/slicer_cli" 2>/dev/null | grep -A2 'LC_RPATH' | grep 'path' | awk '{print $2}')

# ── Collect non-system shared deps ─────────────────────────────────────────
# Recurse through otool -L output until no new deps are found.
# "Non-system" = anything NOT under /usr/lib or /System/Library.

declare -a SEEN=("__slicer_cli_seen_sentinel__")
declare -a SRC_KEYS=("__slicer_cli_source_sentinel__")
declare -a SRC_VALUES=("")
QUEUE=("$OUTPUT/slicer_cli")

record_homebrew_provenance() {
    local SOURCE_PATH="$1" RELATIVE FORMULA DESTINATION
    case "$SOURCE_PATH" in
        /opt/homebrew/opt/*)
            RELATIVE="${SOURCE_PATH#/opt/homebrew/opt/}"
            ;;
        /usr/local/opt/*)
            RELATIVE="${SOURCE_PATH#/usr/local/opt/}"
            ;;
        /opt/homebrew/Cellar/*)
            RELATIVE="${SOURCE_PATH#/opt/homebrew/Cellar/}"
            ;;
        /usr/local/Cellar/*)
            RELATIVE="${SOURCE_PATH#/usr/local/Cellar/}"
            ;;
        *)
            echo "ERROR: no provenance collector for bundled library $SOURCE_PATH" >&2
            return 1
            ;;
    esac
    FORMULA="${RELATIVE%%/*}"
    DESTINATION="$OUTPUT/THIRD_PARTY_LICENSES/homebrew"
    mkdir -p "$DESTINATION"
    if [ ! -s "$DESTINATION/$FORMULA.json" ]; then
        brew info --json=v2 "$FORMULA" > "$DESTINATION/$FORMULA.json"
    fi
    if [ ! -s "$DESTINATION/$FORMULA.rb" ]; then
        brew cat "$FORMULA" > "$DESTINATION/$FORMULA.rb"
    fi
}

already_seen() {
    local CANDIDATE="$1" ITEM
    for ITEM in "${SEEN[@]}"; do
        [[ "$ITEM" == "$CANDIDATE" ]] && return 0
    done
    return 1
}

source_for() {
    local CANDIDATE="$1" INDEX
    for ((INDEX=0; INDEX<${#SRC_KEYS[@]}; INDEX++)); do
        if [[ "${SRC_KEYS[$INDEX]}" == "$CANDIDATE" ]]; then
            echo "${SRC_VALUES[$INDEX]}"
            return 0
        fi
    done
    return 0
}

resolve_rpath() {
    local BASENAME="$1" CURRENT_FILE="$2"
    # 1. Try absolute LC_RPATH entries of the current file
    while IFS= read -r RP; do
        RP="${RP//\(.*\)/}"
        RP="${RP//[[:space:]]/}"
        [[ "$RP" == @* ]] && continue
        [[ -f "$RP/$BASENAME" ]] && echo "$RP/$BASENAME" && return 0
    done < <(otool -l "$CURRENT_FILE" 2>/dev/null | grep -A2 'LC_RPATH' | grep 'path' | awk '{print $2}' || true)
    # 2. Resolve @loader_path via original source directory (copies in
    #    dist/Frameworks/ lose their @loader_path context after being moved).
    local ORIG
    ORIG="$(source_for "$CURRENT_FILE")"
    if [[ -n "$ORIG" ]]; then
        local ORIG_DIR
        ORIG_DIR="$(dirname "$ORIG")"
        [[ -f "$ORIG_DIR/$BASENAME" ]] && echo "$ORIG_DIR/$BASENAME" && return 0
    fi
    # 3. Fall back to global rpath pool (main binary's absolute rpaths)
    for DIR in "${GLOBAL_RPATH_DIRS[@]}"; do
        [[ -f "$DIR/$BASENAME" ]] && echo "$DIR/$BASENAME" && return 0
    done
    return 0  # caller treats an empty result as fatal
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
            [[ -z "$FOUND" ]] && { echo "ERROR: cannot resolve $LIB for $CURRENT" >&2; exit 1; }
            LIB="$FOUND"
        fi
        # Skip remaining @-prefixed refs, system libs
        [[ "$LIB" == @* ]] && continue
        [[ "$LIB" =~ ^/usr/lib/ ]] && continue
        [[ "$LIB" =~ ^/System/ ]] && continue
        already_seen "$LIB" && continue
        SEEN+=("$LIB")
        [[ ! -f "$LIB" ]] && { echo "ERROR: cannot find $LIB" >&2; exit 1; }
        DEST="$FRAMEWORKS/$(basename "$LIB")"
        [[ -f "$DEST" ]] && continue
        echo "Bundling: $LIB → $DEST"
        cp "$LIB" "$DEST"
        record_homebrew_provenance "$LIB"
        SRC_KEYS+=("$DEST")
        SRC_VALUES+=("$LIB")
        QUEUE+=("$DEST")
    done < <(otool -L "$CURRENT" 2>/dev/null | tail -n +2)
done

# ── Rewrite load commands ───────────────────────────────────────────────────
rewrite_refs() {
    local TARGET="$1"
    chmod +w "$TARGET"
    if [[ "$TARGET" == "$FRAMEWORKS/"* ]]; then
        install_name_tool -id "@rpath/$(basename "$TARGET")" "$TARGET"
    fi
    # Rewrite absolute Homebrew paths
    for SRC_LIB in "${SEEN[@]}"; do
        local BASENAME
        BASENAME="$(basename "$SRC_LIB")"
        if otool -L "$TARGET" | tail -n +2 | awk '{print $1}' | grep -Fxq "$SRC_LIB"; then
            install_name_tool -change "$SRC_LIB" \
                "@executable_path/Frameworks/$BASENAME" \
                "$TARGET"
        fi
    done
    # Rewrite @rpath/X references for bundled libs
    while IFS= read -r RPATH_LIB; do
        RPATH_LIB="$(echo "$RPATH_LIB" | awk '{print $1}')"
        [[ "$RPATH_LIB" != @rpath/* ]] && continue
        local BASENAME="${RPATH_LIB#@rpath/}"
        if [[ -f "$FRAMEWORKS/$BASENAME" ]]; then
            install_name_tool -change "$RPATH_LIB" \
                "@executable_path/Frameworks/$BASENAME" \
                "$TARGET"
        else
            echo "ERROR: unresolved bundled dependency $RPATH_LIB in $TARGET" >&2
            exit 1
        fi
    done < <(otool -L "$TARGET" 2>/dev/null | tail -n +2)
    # Remove Homebrew rpaths (now replaced with @executable_path refs)
    while IFS= read -r RPATH; do
        RPATH="${RPATH//\(.*\)/}"
        RPATH="${RPATH//[[:space:]]/}"
        if [[ "$RPATH" =~ homebrew|Homebrew ]]; then
            install_name_tool -delete_rpath "$RPATH" "$TARGET"
        fi
    done < <(otool -l "$TARGET" 2>/dev/null | grep -A2 'LC_RPATH' | grep 'path' | awk '{print $2}')
}

rewrite_refs "$OUTPUT/slicer_cli"
shopt -s nullglob
DYLIBS=("$FRAMEWORKS"/*.dylib)
for DYLIB in "${DYLIBS[@]}"; do
    rewrite_refs "$DYLIB"
done

for TARGET in "$OUTPUT/slicer_cli" "${DYLIBS[@]}"; do
    if otool -L "$TARGET" | grep -E '/opt/homebrew|/usr/local/(Cellar|opt)|/(build|\.cache|_temp)/'; then
        echo "ERROR: non-relocatable dependency remains in $TARGET" >&2
        exit 1
    fi
done

# install_name_tool mutates Mach-O load commands after the linker creates the
# original ad-hoc signatures. macOS 15 can then kill the downloaded binary at
# dyld load time with "Code Signature Invalid". Re-sign every copied dylib first
# and the launcher last so the package remains runnable after extraction.
if command -v codesign >/dev/null 2>&1; then
    for DYLIB in "${DYLIBS[@]}"; do
        codesign --force --sign - "$DYLIB"
    done
    codesign --force --sign - "$OUTPUT/slicer_cli"
fi

echo ""
echo "Relocatable bundle written to $OUTPUT/"
echo ""
echo "Verify on a clean host:"
echo "  env -i PATH=/usr/bin:/bin DYLD_FALLBACK_LIBRARY_PATH= $OUTPUT/slicer_cli --version"
