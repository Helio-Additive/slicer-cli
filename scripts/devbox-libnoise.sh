#!/usr/bin/env bash
# Build Bambu's libnoise fork into $LIBNOISE_PREFIX on first devbox activation.
# Idempotent — skips if the static lib already exists.
#
# nixpkgs ships an unrelated `libnoise`; BambuStudio needs the Bambu fork at
# https://github.com/bambulab/libnoise. We build it once into the devbox state
# directory so it stays inside the project (no /usr/local pollution, no sudo).

set -euo pipefail

# `init_hook` runs before devbox.json's `env` block is exported, so derive
# the prefix from DEVBOX_PROJECT_ROOT (which devbox always sets).
ROOT="${DEVBOX_PROJECT_ROOT:?DEVBOX_PROJECT_ROOT not set — run inside devbox shell}"
PREFIX="$ROOT/.devbox/state/libnoise"

# Already built? bail. Bambu fork installs liblibnoise_static.a (yes, double prefix).
if [ -f "$PREFIX/lib/liblibnoise_static.a" ]; then
    exit 0
fi

echo "[devbox] Building libnoise (Bambu fork) into $PREFIX ..."

TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

git clone --depth=1 https://github.com/bambulab/libnoise.git "$TMP/src"
cmake \
    -S "$TMP/src" \
    -B "$TMP/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$PREFIX" \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build "$TMP/build" --parallel
cmake --install "$TMP/build"

echo "[devbox] libnoise installed."
