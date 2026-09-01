#!/usr/bin/env bash
set -euo pipefail

PACKAGE_ROOT="${1:?Usage: $0 <package-root>}"
PACKAGE_ROOT="$(cd "$PACKAGE_ROOT" && pwd -P)"

test -x "$PACKAGE_ROOT/slicer_cli"
test -x "$PACKAGE_ROOT/slicer_cli-orcaslicer"
test -d "$PACKAGE_ROOT/resources/profiles/BBL/machine"
test -d "$PACKAGE_ROOT/resources/profiles-orca/Snapmaker/machine"
test -n "$(find "$PACKAGE_ROOT/THIRD_PARTY_LICENSES" -type f -print -quit)"

docker run --rm --network none \
    -v "$PACKAGE_ROOT:/package:ro" \
    ubuntu:22.04 \
    sh -ceu '/package/slicer_cli --help >/dev/null; /package/slicer_cli-orcaslicer --help >/dev/null'
