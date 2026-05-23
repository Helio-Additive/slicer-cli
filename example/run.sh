#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

STL_URL="https://helioadditive-public.s3.ap-east-1.amazonaws.com/3DBenchy.stl"
STL_PATH="$SCRIPT_DIR/3DBenchy.stl"
JOBSPEC_PATH="$SCRIPT_DIR/benchy-jobspec.json"
OUTPUT_PATH="$SCRIPT_DIR/benchy.gcode"

echo "Run locally or via docker? [l/d]"
read -r mode

if [[ ! -f "$STL_PATH" ]]; then
    echo "Downloading 3DBenchy STL..."
    curl -fsSL "$STL_URL" -o "$STL_PATH"
else
    echo "3DBenchy STL already cached at $STL_PATH"
fi

echo "Building jobspec from jsonnet..."
jsonnet -J "$ROOT_DIR" \
    --ext-str input_path="$STL_PATH" \
    --ext-str output_path="$OUTPUT_PATH" \
    "$SCRIPT_DIR/jobspec.jsonnet" > "$JOBSPEC_PATH"

if [[ "$mode" == "d" ]]; then
    echo "Running via Docker..."
    docker run --rm \
        -v "$STL_PATH":/data/3DBenchy.stl \
        -v "$JOBSPEC_PATH":/data/jobspec.json \
        -v "$SCRIPT_DIR":/data/out \
        slicer-cli:latest slice /data/jobspec.json
else
    echo "Running locally..."
    "$ROOT_DIR/target/debug/slicer-cli" slice "$JOBSPEC_PATH"
fi
