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
        bash          # macOS /bin/bash is 3.2; scripts/bundle-macos.sh uses declare -A
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

    # Homebrew bash 5 depends on ncurses. On CI runners, bash may be listed as
    # "already installed" but ncurses missing/unlinked, causing bash itself to
    # crash. Force a reinstall to ensure the dylib is present.
    if ! [ -f /opt/homebrew/opt/ncurses/lib/libncursesw.6.dylib ]; then
        echo "  ncurses dylib missing — reinstalling to fix Homebrew bash dep..."
        brew reinstall ncurses bash
    fi

    # Eigen3 cmake config must be present for find_package(Eigen3). Runner images
    # sometimes ship eigen without the cmake config properly linked.
    if ! [ -f /opt/homebrew/opt/eigen/share/eigen3/cmake/Eigen3Config.cmake ]; then
        echo "  Eigen3 cmake config missing — reinstalling..."
        brew reinstall eigen
    fi

    # libnoise (Bambu fork) — not in Homebrew, must build from source
    if ! [ -f /usr/local/lib/libnoise.a ] && ! [ -f /usr/local/lib/libnoise.dylib ]; then
        echo "  Installing libnoise (Bambu fork)..."
        LIBNOISE_TMP=$(mktemp -d)
        git clone --depth=1 https://github.com/bambulab/libnoise.git "$LIBNOISE_TMP"
        cmake -S "$LIBNOISE_TMP" -B "$LIBNOISE_TMP/build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_POLICY_VERSION_MINIMUM=3.5
        cmake --build "$LIBNOISE_TMP/build" --parallel
        sudo cmake --install "$LIBNOISE_TMP/build"
        rm -rf "$LIBNOISE_TMP"
    else
        echo "  ✓ libnoise (already installed)"
    fi

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
