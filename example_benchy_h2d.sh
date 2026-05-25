#!/bin/bash
# Example: Slice 3DBenchy for Bambu Lab H2D with PLA Basic filament
# Demonstrates using slicer_cli with BambuStudio JSON config profiles

set -e

GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║  Slicing 3DBenchy with Bambu Lab H2D + PLA Basic         ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Paths — everything is relative to this script at the repo root.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$SCRIPT_DIR"
BUILD_DIR="$REPO_ROOT/libslic3r/bambustudio/build"
SLICER="$BUILD_DIR/slicer_cli"

# BambuStudio profiles ship inside the vendored source tree
PROFILES_DIR="$REPO_ROOT/libslic3r/bambustudio/references/BambuStudio/resources/profiles/BBL"

# Output
OUTPUT_GCODE="$SCRIPT_DIR/3DBenchy_H2D_PLA.gcode"

# Config files
MACHINE_CONFIG="$PROFILES_DIR/machine/Bambu Lab H2D 0.4 nozzle.json"
FILAMENT_CONFIG="$PROFILES_DIR/filament/Bambu PLA Basic @BBL H2D.json"
PROCESS_CONFIG="$PROFILES_DIR/process/0.20mm Standard @BBL H2D.json"

# ── Pre-flight checks ───────────────────────────────────────────────────────

if [ ! -f "$SLICER" ]; then
    echo -e "${RED}Error: slicer_cli not found at $SLICER${NC}"
    echo ""
    echo "Build it first:"
    echo "  devbox run setup"
    echo "  devbox run native:build"
    exit 1
fi

# Resolve input STL — accept argument, or look for a bundled copy
INPUT_STL=""
if [ -n "$1" ]; then
    INPUT_STL="$1"
elif [ -f "$REPO_ROOT/libslic3r/bambustudio/references/BambuStudio/resources/model/3DBenchy.stl" ]; then
    INPUT_STL="$REPO_ROOT/libslic3r/bambustudio/references/BambuStudio/resources/model/3DBenchy.stl"
fi

if [ -z "$INPUT_STL" ] || [ ! -f "$INPUT_STL" ]; then
    echo -e "${RED}Error: No input STL found${NC}"
    echo ""
    echo "Usage: $0 [path/to/model.stl]"
    exit 1
fi

# Check config files
for pair in "Machine:$MACHINE_CONFIG" "Filament:$FILAMENT_CONFIG" "Process:$PROCESS_CONFIG"; do
    label="${pair%%:*}"
    path="${pair#*:}"
    if [ ! -f "$path" ]; then
        echo -e "${RED}Error: $label config not found:${NC}"
        echo "  $path"
        echo ""
        echo "If you don't have BambuStudio resources, copy them into:"
        echo "  $REPO_ROOT/libslic3r/bambustudio/references/BambuStudio/resources/"
        exit 1
    fi
done

# ── Slice ────────────────────────────────────────────────────────────────────

echo -e "${GREEN}Configuration:${NC}"
echo "  Input:    $(basename "$INPUT_STL")"
echo "  Output:   $(basename "$OUTPUT_GCODE")"
echo "  Printer:  Bambu Lab H2D (0.4mm nozzle)"
echo "  Filament: Bambu PLA Basic"
echo "  Process:  0.20mm Standard @BBL H2D"
echo ""
echo -e "${GREEN}Slicing...${NC}"

"$SLICER" "$INPUT_STL" \
    --machine  "$MACHINE_CONFIG" \
    --filament "$FILAMENT_CONFIG" \
    --process  "$PROCESS_CONFIG" \
    -o "$OUTPUT_GCODE"

# ── Result ───────────────────────────────────────────────────────────────────

if [ -f "$OUTPUT_GCODE" ]; then
    FILE_SIZE=$(du -h "$OUTPUT_GCODE" | cut -f1)
    LINE_COUNT=$(wc -l < "$OUTPUT_GCODE")

    echo ""
    echo -e "${GREEN}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║  ✓ SUCCESS!                                               ║${NC}"
    echo -e "${GREEN}╚════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo "  File:  $OUTPUT_GCODE"
    echo "  Size:  $FILE_SIZE"
    echo "  Lines: $LINE_COUNT"
else
    echo ""
    echo -e "${RED}Error: G-code file was not created!${NC}"
    exit 1
fi

# ── Hints ────────────────────────────────────────────────────────────────────

echo ""
echo "────────────────────────────────────────────────────────────"
echo "Override examples:"
echo ""
echo "  Layer height:"
echo "    $SLICER model.stl --machine ... --filament ... --layer-height 0.16 -o out.gcode"
echo ""
echo "  Infill density:"
echo "    $SLICER model.stl --machine ... --filament ... --infill 25 -o out.gcode"
echo "────────────────────────────────────────────────────────────"
