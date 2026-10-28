#!/usr/bin/env bash
# package-local-macos.sh — assemble a local macOS arm64 package with all three engine programs.
#
# Mirrors the CI steps "Bundle for macOS (arm64 only)", "Stage engine binary (macOS / Linux)" and
# "Assemble + validate package (macOS arm64)" (.github/workflows/slicer-cli-ci.yml), with a third
# program built with -DENGINE=snapmaker (Snapmaker Orca, references/SnapmakerOrca):
#
#   slicer_cli              ← build-bambu/slicer_cli      (BambuStudio)
#   slicer_cli-orcaslicer   ← build-orca/slicer_cli       (OrcaSlicer)
#   slicer_cli-snapmaker    ← build-snapmaker/slicer_cli  (Snapmaker Orca)
#
# Usage:
#   bash scripts/package-local-macos.sh <out_dir>
#
# Output (everything under <out_dir> is rebuilt from scratch on every run):
#   <out_dir>/slicer-cli/                                 the package tree
#   <out_dir>/release/slicer-cli-v<ver>-macos-arm64.tar.gz (+ .sha256, engine-platform-macos-arm64.json)
#
# Environment:
#   BAMBU_BIN / ORCA_BIN / SNAPMAKER_BIN   override the input binaries
#   PKG_NO_LOCK=1    don't take deps-prefix/build.lock while copying the binaries
#                    (set it when the caller already holds that lock)
#   RUN_TESTS=1      also run the CI package tests on the extracted copy
#                    (test-packaged-slice, test-complete-slice-path, test_list_settings)
#
# The validation always runs on a copy extracted OUTSIDE the repository: a package placed inside the
# checkout (e.g. pkg-local/slicer-cli) can resolve ../../references/* from the binary's directory
# (main.cpp engine_profiles_dir / orca_profiles_dir / engine_resources_root), which would hide a
# missing package resource.
set -euo pipefail

OUT_ARG="${1:?Usage: $0 <out_dir>}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
mkdir -p "$OUT_ARG"
OUT="$(cd "$OUT_ARG" && pwd -P)"
BASH5=/opt/homebrew/bin/bash   # bundle-macos.sh is run with Homebrew bash, as in CI
[ -x "$BASH5" ] || BASH5="$(command -v bash)"
export COPYFILE_DISABLE=1      # no AppleDouble files in the tarball

BAMBU_BIN="${BAMBU_BIN:-$REPO/build-bambu/slicer_cli}"
ORCA_BIN="${ORCA_BIN:-$REPO/build-orca/slicer_cli}"
SNAPMAKER_BIN="${SNAPMAKER_BIN:-$REPO/build-snapmaker/slicer_cli}"
for b in "$BAMBU_BIN" "$ORCA_BIN" "$SNAPMAKER_BIN"; do
    [ -x "$b" ] || { echo "missing engine binary: $b" >&2; exit 1; }
done
for d in BambuStudio OrcaSlicer SnapmakerOrca; do
    [ -d "$REPO/references/$d/resources/profiles" ] || { echo "missing references/$d (submodule not checked out)" >&2; exit 1; }
done

GIT_SHA="$(git -C "$REPO" rev-parse HEAD 2>/dev/null || echo unknown)"
VERSION="0.0.0-local-${GIT_SHA:0:12}"
PLATFORM="macos-arm64"
ASSET="slicer-cli-v${VERSION}-${PLATFORM}.tar.gz"
STAGE="$OUT/.stage"
P="$OUT/slicer-cli"
REL="$OUT/release"
rm -rf "$STAGE" "$P" "$REL"
mkdir -p "$STAGE" "$P/resources/profiles" "$REL"

# ── 1. Copy the binaries (under the build lock, so a link in progress is never picked up) ──
copy_bins() {
    mkdir -p "$STAGE/in"
    cp "$BAMBU_BIN" "$STAGE/in/bambu"
    cp "$ORCA_BIN" "$STAGE/in/orca"
    cp "$SNAPMAKER_BIN" "$STAGE/in/snapmaker"
}
if [ "${PKG_NO_LOCK:-0}" = 1 ]; then
    copy_bins
else
    mkdir -p "$REPO/deps-prefix"
    /usr/bin/lockf -t 3600 "$REPO/deps-prefix/build.lock" /bin/bash -c "
        set -e; mkdir -p '$STAGE/in'
        cp '$BAMBU_BIN' '$STAGE/in/bambu'; cp '$ORCA_BIN' '$STAGE/in/orca'; cp '$SNAPMAKER_BIN' '$STAGE/in/snapmaker'"
fi

# ── 2. Bundle each engine (CI "Bundle for macOS") and stage it (CI "Stage engine binary") ──
# engine key → in-package file name
pkg_name() {
    case "$1" in
        bambu) echo slicer_cli ;;
        orca) echo slicer_cli-orcaslicer ;;
        snapmaker) echo slicer_cli-snapmaker ;;
    esac
}
ENGINES=(bambu orca snapmaker)
for e in "${ENGINES[@]}"; do
    "$BASH5" "$REPO/scripts/bundle-macos.sh" "$STAGE/in/$e" "$STAGE/dist-$e" > "$STAGE/bundle-$e.log"
    bash "$REPO/scripts/smoke-clean-host.sh" "$STAGE/dist-$e/slicer_cli" > "$STAGE/smoke-$e.log"
    S="$STAGE/engine-$e"; mkdir -p "$S"
    cp "$STAGE/dist-$e/slicer_cli" "$S/$(pkg_name "$e")"
    [ -d "$STAGE/dist-$e/Frameworks" ] && cp -R "$STAGE/dist-$e/Frameworks" "$S/Frameworks"
    if [ -d "$STAGE/dist-$e/THIRD_PARTY_LICENSES" ]; then
        mkdir -p "$S/THIRD_PARTY_LICENSES/$e"
        cp -R "$STAGE/dist-$e/THIRD_PARTY_LICENSES/." "$S/THIRD_PARTY_LICENSES/$e/"
    fi
done

# ── 3. Assemble (CI "Assemble + validate package (macOS arm64)") ──
for e in "${ENGINES[@]}"; do
    n="$(pkg_name "$e")"
    cp "$STAGE/engine-$e/$n" "$P/$n"
    chmod +x "$P/$n"
done
# Union every engine's bundled Frameworks; the same name must be the same bytes.
for e in "${ENGINES[@]}"; do
    d="$STAGE/engine-$e"
    [ -d "$d/Frameworks" ] || continue
    mkdir -p "$P/Frameworks"
    for framework in "$d"/Frameworks/*; do
        [ -e "$framework" ] || continue
        destination="$P/Frameworks/$(basename "$framework")"
        if [ -e "$destination" ] && ! cmp -s "$framework" "$destination"; then
            echo "Framework collision between engine bundles: $(basename "$framework")" >&2
            exit 1
        fi
        cp -R "$framework" "$destination"
    done
done
mkdir -p "$P/THIRD_PARTY_LICENSES"
for e in "${ENGINES[@]}"; do
    d="$STAGE/engine-$e"
    [ -d "$d/THIRD_PARTY_LICENSES" ] || continue
    while IFS= read -r license; do
        relative="${license#"$d/THIRD_PARTY_LICENSES/"}"
        destination="$P/THIRD_PARTY_LICENSES/$relative"
        mkdir -p "$(dirname "$destination")"
        if [ -e "$destination" ] && ! cmp -s "$license" "$destination"; then
            echo "Third-party record collision: $relative" >&2
            exit 1
        fi
        cp "$license" "$destination"
    done < <(find "$d/THIRD_PARTY_LICENSES" -type f -print)
done
test -n "$(find "$P/THIRD_PARTY_LICENSES" -type f -print -quit)"

R="$REPO/references"
# Bambu: BBL profiles + vendor index; resources root = resources/ (main.cpp engine_resources_root, Bambu branch).
cp -R "$R/BambuStudio/resources/profiles/BBL" "$P/resources/profiles/BBL"
cp "$R/BambuStudio/resources/profiles/BBL.json" "$P/resources/profiles/BBL.json"
cp -R "$R/BambuStudio/resources/info" "$P/resources/info"
cp -R "$R/BambuStudio/resources/flush" "$P/resources/flush"
cp -R "$R/BambuStudio/resources/filament_mixing" "$P/resources/filament_mixing"
# Orca: full profile tree + its own resources root (main.cpp orca_profiles_dir / engine_resources_root, ENGINE_ORCA).
cp -R "$R/OrcaSlicer/resources/profiles" "$P/resources/profiles-orca"
mkdir -p "$P/resources/orca"
cp -R "$R/OrcaSlicer/resources/info" "$P/resources/orca/info"
cp -R "$R/OrcaSlicer/resources/flush" "$P/resources/orca/flush"
# Snapmaker Orca: the fork's own profile tree and resources root (main.cpp orca_profiles_dir /
# engine_resources_root, ENGINE_SNAPMAKER: resources/profiles-snapmaker, resources/snapmaker).
# The fork reads info/*.json (Print.cpp 3081, 3116). It ships no resources/flush folder
# (its libslic3r has no FlushVolPredictor), so flush/ is copied only if a later fork adds one.
cp -R "$R/SnapmakerOrca/resources/profiles" "$P/resources/profiles-snapmaker"
mkdir -p "$P/resources/snapmaker"
cp -R "$R/SnapmakerOrca/resources/info" "$P/resources/snapmaker/info"
[ -d "$R/SnapmakerOrca/resources/flush" ] && cp -R "$R/SnapmakerOrca/resources/flush" "$P/resources/snapmaker/flush"

cp "$REPO/LICENSE" "$REPO/NOTICE" "$REPO/README.md" "$P/"
cp "$R/BambuStudio/LICENSE" "$P/BAMBUSTUDIO-LICENSE.txt"
cp "$R/OrcaSlicer/LICENSE.txt" "$P/ORCASLICER-LICENSE.txt"
if [ -f "$R/SnapmakerOrca/LICENSE.txt" ]; then
    cp "$R/SnapmakerOrca/LICENSE.txt" "$P/SNAPMAKER-ORCA-LICENSE.txt"
else
    cp "$R/SnapmakerOrca/LICENSE" "$P/SNAPMAKER-ORCA-LICENSE.txt"
fi
cat > "$P/manifest.json" <<EOF
{
  "version": "${VERSION}",
  "gitSha": "${GIT_SHA}",
  "platform": "${PLATFORM}",
  "executablePath": "slicer-cli/slicer_cli",
  "profilesPath": "slicer-cli/resources/profiles/BBL",
  "orcaProfilesPath": "slicer-cli/resources/profiles-orca",
  "snapmakerProfilesPath": "slicer-cli/resources/profiles-snapmaker",
  "engines": [
    { "engine": "bambustudio", "path": "slicer-cli/slicer_cli" },
    { "engine": "orcaslicer",  "path": "slicer-cli/slicer_cli-orcaslicer" },
    { "engine": "snapmaker",   "path": "slicer-cli/slicer_cli-snapmaker" }
  ]
}
EOF
python3 -I -c 'import json,sys; json.load(open(sys.argv[1]))' "$P/manifest.json"

tar -C "$OUT" -czf "$REL/$ASSET" slicer-cli
HASH="$(shasum -a 256 "$REL/$ASSET" | awk '{print $1}')"
printf '%s  %s\n' "$HASH" "$ASSET" > "$REL/$ASSET.sha256"
cat > "$REL/engine-platform-${PLATFORM}.json" <<EOF
{
  "platform": "${PLATFORM}",
  "assetFilename": "${ASSET}",
  "sha256": "${HASH}",
  "executablePath": "slicer-cli/slicer_cli",
  "profilesPath": "slicer-cli/resources/profiles/BBL"
}
EOF

# ── 4. Validate the extracted tarball, outside the repository ──
VERIFY="$(mktemp -d "${TMPDIR:-/tmp}/slicer-cli-verify.XXXXXX")"
trap 'rm -rf "$VERIFY"' EXIT
tar -C "$VERIFY" -xzf "$REL/$ASSET"
V="$VERIFY/slicer-cli"
test -x "$V/slicer_cli"
test -x "$V/slicer_cli-orcaslicer"
test -x "$V/slicer_cli-snapmaker"
for t in machine process filament; do
    test -d "$V/resources/profiles/BBL/$t"
    test -d "$V/resources/profiles-orca/Snapmaker/$t"
    test -d "$V/resources/profiles-snapmaker/Snapmaker/$t"
done
test -f "$V/resources/info/filament_info.json"
test -f "$V/resources/info/nozzle_info.json"
test -f "$V/resources/info/nozzle_incompatibles.json"
test -f "$V/resources/flush/flush_data_material_pair.json"
test -f "$V/resources/flush/flush_data_standard.txt"
test -f "$V/resources/filament_mixing/standard_color_recipes.json"
test -f "$V/resources/orca/info/nozzle_info.json"
test -f "$V/resources/orca/info/nozzle_incompatibles.json"
test -f "$V/resources/orca/flush/flush_data_standard.txt"
test -f "$V/resources/profiles/BBL.json"
test -f "$V/resources/profiles-snapmaker/Snapmaker.json"
test -f "$V/resources/snapmaker/info/filament_info.json"
test -f "$V/resources/snapmaker/info/nozzle_info.json"
test -f "$V/SNAPMAKER-ORCA-LICENSE.txt"
"$V/slicer_cli" --help >/dev/null
"$V/slicer_cli-orcaslicer" --help >/dev/null
"$V/slicer_cli-snapmaker" --help >/dev/null
for n in slicer_cli slicer_cli-orcaslicer slicer_cli-snapmaker; do
    bash "$REPO/scripts/smoke-clean-host.sh" "$V/$n" > "$STAGE/smoke-pkg-$n.log"
done
# The Snapmaker program must configure the package's own resources root.
"$V/slicer_cli-snapmaker" --help 2>&1 | grep -E "^  Engine resources: .*/slicer-cli/resources/snapmaker$" >/dev/null \
    || { echo "slicer_cli-snapmaker does not use the package's resources/snapmaker root" >&2; "$V/slicer_cli-snapmaker" --help 2>&1 | grep -i "resources" >&2; exit 1; }

if [ "${RUN_TESTS:-0}" = 1 ]; then
    bash "$REPO/tests/test-packaged-slice.sh" "$V/slicer_cli" "$V/slicer_cli-orcaslicer"
    bash "$REPO/tests/test-complete-slice-path.sh" "$V/slicer_cli" "$V/slicer_cli-orcaslicer"
    bash "$REPO/tests/test_list_settings.sh" "$V/slicer_cli" "$V/slicer_cli-orcaslicer"
fi

rm -rf "$STAGE/in"
echo "Package: $P"
echo "Tarball: $REL/$ASSET ($HASH)"
echo "Validated an extracted copy outside the repository: 3 programs run, resources present, clean-host smoke passed."
