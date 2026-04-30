#!/bin/bash
# Install system dependencies for slicer_cli
# These replace BambuStudio's bundled dep build system entirely.

set -e

GREEN='\033[0;32m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}Installing system dependencies for slicer_cli${NC}"
echo ""

if [[ "$OSTYPE" == "darwin"* ]]; then
    if ! command -v brew &>/dev/null; then
        echo -e "${RED}Homebrew not found. Install it from https://brew.sh${NC}"
        exit 1
    fi

    echo -e "${GREEN}macOS detected — using Homebrew${NC}"
    echo ""

    PACKAGES=(
        cmake
        boost
        tbb
        eigen
        cgal
        libpng
        zlib
        expat
        openssl@3
        opencv
        nlopt
        qhull
        cereal
        opencascade
        freetype
    )

    for pkg in "${PACKAGES[@]}"; do
        if brew list --formula "$pkg" &>/dev/null; then
            echo "  ✓ $pkg (already installed)"
        else
            echo "  Installing $pkg..."
            brew install "$pkg"
        fi
    done

elif [[ -f /etc/debian_version ]]; then
    echo -e "${GREEN}Debian/Ubuntu detected — using apt${NC}"
    echo ""

    PACKAGES=(
        cmake
        build-essential
        libboost-all-dev
        libtbb-dev
        libeigen3-dev
        libcgal-dev
        libpng-dev
        zlib1g-dev
        libexpat1-dev
        libssl-dev
        libopencv-dev
        libnlopt-dev
        libqhull-dev
        libfreetype6-dev
        libfontconfig1-dev
        libocct-modeling-algorithms-dev
        libocct-data-exchange-dev
        libocct-foundation-dev
    )

    echo "  sudo apt update"
    sudo apt update -qq

    for pkg in "${PACKAGES[@]}"; do
        if dpkg -s "$pkg" &>/dev/null 2>&1; then
            echo "  ✓ $pkg"
        else
            echo "  Installing $pkg..."
            sudo apt install -y -qq "$pkg"
        fi
    done

    # cereal is header-only — check manually
    if [ ! -f /usr/include/cereal/cereal.hpp ]; then
        echo "  Installing cereal (header-only)..."
        sudo apt install -y -qq libcereal-dev 2>/dev/null || {
            echo -e "${RED}  cereal not in apt — installing from source${NC}"
            TMP=$(mktemp -d)
            git clone --depth 1 https://github.com/USCiLab/cereal.git "$TMP/cereal"
            sudo cp -R "$TMP/cereal/include/cereal" /usr/local/include/
            rm -rf "$TMP"
        }
    else
        echo "  ✓ cereal"
    fi
else
    echo -e "${RED}Unsupported platform: $OSTYPE${NC}"
    echo ""
    echo "Required libraries:"
    echo "  cmake, boost, tbb, eigen3, cgal, libpng, zlib, expat,"
    echo "  openssl, opencv, nlopt, qhull, cereal, opencascade (occt),"
    echo "  freetype"
    exit 1
fi

echo ""
echo -e "${GREEN}✓ All dependencies installed.${NC}"
echo ""
echo "Next steps:"
echo ""
echo "  cd cli"
echo "  mkdir -p build && cd build"
echo "  cmake .. -DCMAKE_BUILD_TYPE=Release"
echo "  make -j\$(sysctl -n hw.ncpu 2>/dev/null || nproc)"
echo ""
