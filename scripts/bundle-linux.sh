#!/usr/bin/env bash
set -euo pipefail

PACKAGE_ROOT="${1:?Usage: $0 <package-root> <binary> [binary ...]}"
shift
[ "$#" -gt 0 ] || { echo "At least one binary is required" >&2; exit 2; }

LIB_DIR="$PACKAGE_ROOT/lib"
LICENSE_DIR="$PACKAGE_ROOT/THIRD_PARTY_LICENSES/dpkg"
mkdir -p "$LIB_DIR"
mkdir -p "$LICENSE_DIR"
declare -A COPIED
QUEUE=("$@")

copy_dpkg_copyright() {
    local LIBRARY="$1" PACKAGE COPYRIGHT DESTINATION
    PACKAGE="$(dpkg-query -S "$LIBRARY" 2>/dev/null | head -n 1 | cut -d: -f1 || true)"
    if [ -z "$PACKAGE" ]; then
        PACKAGE="$(dpkg-query -S "$(readlink -f "$LIBRARY")" 2>/dev/null | head -n 1 | cut -d: -f1 || true)"
    fi
    [ -n "$PACKAGE" ] || { echo "No dpkg owner for bundled library: $LIBRARY" >&2; exit 1; }
    COPYRIGHT="/usr/share/doc/$PACKAGE/copyright"
    [ -r "$COPYRIGHT" ] || { echo "Missing copyright record for bundled package: $PACKAGE" >&2; exit 1; }
    DESTINATION="$LICENSE_DIR/$PACKAGE.txt"
    if [ -e "$DESTINATION" ]; then
        cmp -s "$COPYRIGHT" "$DESTINATION" || {
            echo "Copyright record collision: $PACKAGE" >&2
            exit 1
        }
    else
        cp "$COPYRIGHT" "$DESTINATION"
    fi
}

is_glibc_core() {
    case "$(basename "$1")" in
        ld-linux*.so*|libc.so.*|libm.so.*|libpthread.so.*|librt.so.*|libdl.so.*|libresolv.so.*|libutil.so.*)
            return 0 ;;
    esac
    return 1
}

while [ "${#QUEUE[@]}" -gt 0 ]; do
    CURRENT="${QUEUE[0]}"
    QUEUE=("${QUEUE[@]:1}")
    while IFS= read -r LINE; do
        if [[ "$LINE" == *"=> not found"* ]]; then
            echo "Unresolved dependency for $CURRENT: $LINE" >&2
            exit 1
        fi
        RESOLVED="$(awk '/=> \/|^[[:space:]]*\// { for (i=1; i<=NF; i++) if ($i ~ /^\//) { print $i; exit } }' <<<"$LINE")"
        [ -n "$RESOLVED" ] || continue
        [ -f "$RESOLVED" ] || continue
        is_glibc_core "$RESOLVED" && continue
        BASENAME="$(basename "$RESOLVED")"
        DESTINATION="$LIB_DIR/$BASENAME"
        if [[ -n "${COPIED[$BASENAME]+x}" ]]; then
            cmp -s "$RESOLVED" "$DESTINATION" || {
                echo "Runtime library basename collision: $BASENAME" >&2
                exit 1
            }
            continue
        fi
        COPIED["$BASENAME"]=1
        cp -L "$RESOLVED" "$DESTINATION"
        copy_dpkg_copyright "$RESOLVED"
        chmod u+w "$DESTINATION"
        QUEUE+=("$DESTINATION")
    done < <(ldd "$CURRENT")
done

[ -n "$(find "$LICENSE_DIR" -type f -print -quit)" ] || {
    echo "No third-party dependency records were collected" >&2
    exit 1
}

for BINARY in "$@"; do
    patchelf --set-rpath '$ORIGIN/lib' "$BINARY"
done
for LIBRARY in "$LIB_DIR"/*; do
    [ -e "$LIBRARY" ] || continue
    patchelf --set-rpath '$ORIGIN' "$LIBRARY"
done

for BINARY in "$@"; do
    if ! LD_LIBRARY_PATH="$LIB_DIR" ldd "$BINARY" | awk -v root="$PACKAGE_ROOT" '
        /=> not found/ { bad=1 }
        /=> \// {
            path=$3
            if (index(path, root) != 1 && path !~ /\/(ld-linux|libc\.so|libm\.so|libpthread\.so|librt\.so|libdl\.so|libresolv\.so|libutil\.so)/) bad=1
        }
        END { exit bad }
    '; then
        echo "Packaged dependency escaped the bundle: $BINARY" >&2
        LD_LIBRARY_PATH="$LIB_DIR" ldd "$BINARY" >&2
        exit 1
    fi
done
