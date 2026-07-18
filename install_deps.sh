#!/bin/bash
# Install system dependencies for slicer_cli
# These replace BambuStudio's bundled dep build system entirely.

set -e

GREEN='\033[0;32m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

# BambuStudio v02.08.01.55 builds CGAL v5.4 from source. CGAL 6 moved
# extract_boundary_cycles out of Polygon_mesh_processing, so the current Bambu
# source cannot compile against it. Homebrew no longer ships cgal@5; use the
# hash-pinned, Bambu-patched CGAL 5.4 source release. The CMake compatibility
# header supplies the legacy Boost MPL include that this exact CGAL release
# formerly received transitively.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CGAL5_VERSION='5.4'
CGAL5_SHA256='d7605e0a5a5ca17da7547592f6f6e4a59430a0bc861948974254d0de43eab4c0'
CGAL5_URL="https://github.com/CGAL/cgal/archive/refs/tags/v${CGAL5_VERSION}.zip"
CGAL5_PATCH="$SCRIPT_DIR/references/BambuStudio/deps/CGAL/0001-clang19.patch"

install_cgal5() {
    local prefix="$1"
    local use_sudo="$2"
    local config="$prefix/lib/cmake/CGAL/CGALConfig.cmake"

    if [ -f "$config" ] && grep -q "${CGAL5_VERSION}" "$config"; then
        echo "  ✓ CGAL ${CGAL5_VERSION} ($prefix)"
        return
    fi

    echo "  Installing CGAL ${CGAL5_VERSION} from the hash-pinned upstream release..."
    local workdir
    workdir=$(mktemp -d)
    local archive="$workdir/cgal-${CGAL5_VERSION}.zip"
    curl --fail --location --silent --show-error "$CGAL5_URL" -o "$archive"
    local actual_sha256
    if command -v shasum >/dev/null 2>&1; then
        actual_sha256=$(shasum -a 256 "$archive" | awk '{print $1}')
    else
        actual_sha256=$(sha256sum "$archive" | awk '{print $1}')
    fi
    if [ "$actual_sha256" != "$CGAL5_SHA256" ]; then
        echo -e "${RED}CGAL ${CGAL5_VERSION} SHA-256 mismatch${NC}"
        exit 1
    fi
    unzip -q "$archive" -d "$workdir"
    patch -d "$workdir/cgal-${CGAL5_VERSION}" -p1 < "$CGAL5_PATCH"
    cmake -S "$workdir/cgal-${CGAL5_VERSION}" -B "$workdir/build" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$prefix" \
        -DCGAL_HEADER_ONLY=ON \
        -DCGAL_INSTALL_CMAKE_DIR=lib/cmake/CGAL \
        -DBUILD_TESTING=OFF \
        -DBUILD_DOC=OFF \
        -DCGAL_BUILD_THREE_DOC=OFF
    if [ "$use_sudo" = 'yes' ]; then
        sudo cmake --install "$workdir/build"
    else
        cmake --install "$workdir/build"
    fi
}

echo -e "${BLUE}Installing system dependencies for slicer_cli${NC}"
echo ""

if [[ "$OSTYPE" == "darwin"* ]]; then
    if ! command -v brew &>/dev/null; then
        echo -e "${RED}Homebrew not found. Install it from https://brew.sh${NC}"
        exit 1
    fi

    echo -e "${GREEN}macOS detected — using Homebrew${NC}"
    echo ""

    # A restored Homebrew cache from an older runner image can leave formulae
    # listed as installed while their Cellar kegs are missing or truncated.
    # Brew's installed-dependents walk then aborts on the broken keg (e.g.
    # "/opt/homebrew/Cellar/cgal/6.2 is not a directory"). Skip that walk —
    # every package we need is (re)installed explicitly below anyway.
    export HOMEBREW_NO_INSTALLED_DEPENDENTS_CHECK=1

    PACKAGES=(
        cmake
        bash          # macOS /bin/bash is 3.2; scripts/bundle-macos.sh uses declare -A
        boost
        tbb
        eigen
        gmp
        mpfr
        libpng
        zlib
        expat
        openssl@3
        opencv
        assimp        # BambuStudio reference bump: libslic3r imports textured-model formats
        nlopt
        qhull
        cereal
        opencascade
        freetype
        jpeg-turbo     # ENGINE=orca: OrcaSlicer's GCode/Thumbnails.cpp needs libjpeg (find_package(JPEG))
    )

    # Purge kegs that brew lists as installed but whose Cellar directory is
    # gone (stale restored cache vs. newer runner image); a plain reinstall
    # of the healthy set cannot repair those.
    CELLAR="$(brew --cellar)"
    for pkg in "${PACKAGES[@]}"; do
        if brew list --formula "$pkg" &>/dev/null && [ ! -d "$CELLAR/$pkg" ]; then
            echo "  ✗ $pkg broken (listed but no Cellar keg) — removing stale record..."
            brew uninstall --force --ignore-dependencies "$pkg" || true
        fi
    done

    for pkg in "${PACKAGES[@]}"; do
        if brew list --formula "$pkg" &>/dev/null; then
            echo "  ✓ $pkg (already installed)"
        else
            echo "  Installing $pkg..."
            brew install "$pkg"
        fi
    done

    # Cold-cache hardening: macOS GHA runner images often ship Homebrew
    # packages "installed" per `brew list` but with missing dylibs (nlopt,
    # opencascade, etc. all observed broken on different runner image versions).
    # Detect cold-cache by checking if a sentinel cache marker exists; if not,
    # force-reinstall every package to guarantee clean state. The Homebrew
    # cache action will save the result, so warm-cache runs skip this entirely.
    if ! [ -f /opt/homebrew/.slicer_cli_v6_warm ]; then
        echo ""
        echo "  Cold cache detected — force-reinstalling all packages for clean state..."
        brew reinstall "${PACKAGES[@]}"
        touch /opt/homebrew/.slicer_cli_v6_warm
        echo "  ✓ All packages reinstalled, marker set"
    else
        echo "  ✓ Cache marker present; skipping reinstall"
    fi

    # This is deliberately a project-managed Cellar keg, not a Homebrew
    # formula: Homebrew removed cgal@5 while this Bambu source remains on its
    # Polygon_mesh_processing namespace API.
    install_cgal5 "$CELLAR/cgal@5/$CGAL5_VERSION" no

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
        unzip
        libtbb-dev
        libeigen3-dev
        libgmp-dev
        libmpfr-dev
        libpng-dev
        zlib1g-dev
        libexpat1-dev
        libssl-dev
        libopencv-dev
        libassimp-dev
        libnlopt-dev
        libqhull-dev
        libfreetype6-dev
        libfontconfig1-dev
        libjpeg-dev    # ENGINE=orca: OrcaSlicer's GCode/Thumbnails.cpp needs libjpeg (find_package(JPEG))
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

    install_cgal5 "/opt/slicer-cli/cgal-$CGAL5_VERSION" yes

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
    echo "  cmake, boost, tbb, eigen3, Bambu-patched CGAL 5.4, libpng, zlib, expat,"
    echo "  openssl, opencv, assimp, nlopt, qhull, cereal, opencascade (occt),"
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
