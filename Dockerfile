# ============================================================================
# Multi-stage Dockerfile for slicer_cli (arm64)
#
# Builds critical dependencies from source at the exact versions BambuStudio
# pins, matching the official Linux Compile Guide:
#   https://github.com/bambulab/BambuStudio/wiki/Linux-Compile-Guide
#
# Key insight: Ubuntu 24.04 system packages (CGAL 6.x, TBB 2021.11,
# Boost 1.83, OCCT 7.6) are ABI/API-incompatible with what BambuStudio
# expects (CGAL 5.4, TBB 2021.5, Boost 1.84, OCCT 7.6.0).  Building
# from source with static linking (like BambuStudio's own deps/ system)
# eliminates the ABI mismatches.
#
# Build (run from repo root so the full source tree is in context):
#   docker build -f cli/Dockerfile -t slicer-cli .
#
# Run:
#   docker run --rm -v $(pwd)/data:/data slicer-cli \
#       /data/3DBenchy.stl \
#       --machine  /data/"Bambu Lab H2D 0.4 nozzle.json" \
#       --filament /data/"Bambu PLA Basic @BBL H2D.json" \
#       --process  /data/"0.20mm Standard @BBL H2D.json" \
#       -o /data/output.gcode
#
# Or use the bundled profiles:
#   docker run --rm -v $(pwd)/data:/data slicer-cli \
#       /data/3DBenchy.stl \
#       --machine  /profiles/BBL/machine/"Bambu Lab H2D 0.4 nozzle.json" \
#       --filament /profiles/BBL/filament/"Bambu PLA Basic @BBL H2D.json" \
#       --process  /profiles/BBL/process/"0.20mm Standard @BBL H2D.json" \
#       -o /data/output.gcode
# ============================================================================

# ---------------------------------------------------------------------------
# Stage 1: Build pinned dependencies from source
# ---------------------------------------------------------------------------
FROM ubuntu:24.04 AS deps

ENV DEBIAN_FRONTEND=noninteractive

# Base build tools + system libraries that are safe to use from the distro
# (pure C libs with stable ABIs, header-only libs, and tools)
RUN apt-get update -qq && apt-get install -y -qq --no-install-recommends \
        build-essential \
        cmake \
        ninja-build \
        pkg-config \
        git \
        ca-certificates \
        curl \
        # Safe system libraries (stable C ABI, no C++ ABI concerns)
        libpng-dev \
        zlib1g-dev \
        libexpat1-dev \
        libssl-dev \
        libfreetype-dev \
        libfontconfig-dev \
        libicu-dev \
        # Header-only (no ABI issues)
        libeigen3-dev \
        # NLopt C++ headers
        libnlopt-dev \
        libnlopt-cxx-dev \
        # Qhull
        libqhull-dev \
        # GMP + MPFR (C libraries, needed by CGAL)
        libgmp-dev \
        libmpfr-dev \
    && rm -rf /var/lib/apt/lists/*

# Cereal — header-only
RUN apt-get update -qq \
    && (apt-get install -y -qq --no-install-recommends libcereal-dev 2>/dev/null \
        || { \
            git clone --depth 1 https://github.com/USCiLab/cereal.git /tmp/cereal \
            && cp -R /tmp/cereal/include/cereal /usr/local/include/ \
            && rm -rf /tmp/cereal; \
        }) \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /deps

# ── TBB 2021.5.0 (exact BambuStudio pin) ───────────────────────────────────
RUN curl -sL https://github.com/oneapi-src/oneTBB/archive/refs/tags/v2021.5.0.tar.gz \
        | tar xz \
    && cmake -S oneTBB-2021.5.0 -B oneTBB-2021.5.0/build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/deps/destdir \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        -DTBB_BUILD_SHARED=OFF \
        -DTBB_BUILD_TESTS=OFF \
        -DTBB_TEST=OFF \
    && cmake --build oneTBB-2021.5.0/build \
    && cmake --install oneTBB-2021.5.0/build \
    && rm -rf oneTBB-2021.5.0

# ── Boost 1.84.0 (exact BambuStudio pin) ───────────────────────────────────
RUN curl -sL https://github.com/boostorg/boost/releases/download/boost-1.84.0/boost-1.84.0.tar.gz \
        | tar xz \
    && cmake -S boost-1.84.0 -B boost-1.84.0/build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/deps/destdir \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        -DBOOST_EXCLUDE_LIBRARIES="contract;fiber;numpy;wave;test" \
        -DBOOST_LOCALE_ENABLE_ICU=OFF \
        -DBUILD_TESTING=OFF \
        -DBUILD_SHARED_LIBS=OFF \
    && cmake --build boost-1.84.0/build \
    && cmake --install boost-1.84.0/build \
    && rm -rf boost-1.84.0

# ── CGAL 5.4 (exact BambuStudio pin) ───────────────────────────────────────
# Header-only for most features; cmake config must be installed so
# find_package(CGAL) works.  BambuStudio uses CGAL 5.x APIs that are
# incompatible with CGAL 6.x (AABB_traits, add_property_map returning pair).
RUN curl -sL https://github.com/CGAL/cgal/archive/refs/tags/v5.4.tar.gz \
        | tar xz \
    && cmake -S cgal-5.4 -B cgal-5.4/build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/deps/destdir \
        -DCMAKE_PREFIX_PATH=/deps/destdir \
    && cmake --install cgal-5.4/build \
    && rm -rf cgal-5.4

# ── OpenCASCADE 7.6.0 (exact BambuStudio pin) ─────────────────────────────
# Headless build: disable GUI/OpenGL/Tk.  Keep DataExchange (STEP import).
RUN curl -sL https://github.com/Open-Cascade-SAS/OCCT/archive/refs/tags/V7_6_0.tar.gz \
        | tar xz \
    && cmake -S OCCT-7_6_0 -B OCCT-7_6_0/build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/deps/destdir \
        -DCMAKE_PREFIX_PATH=/deps/destdir \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        -DBUILD_LIBRARY_TYPE=Static \
        -DUSE_TK=OFF \
        -DUSE_TBB=OFF \
        -DUSE_FFMPEG=OFF \
        -DUSE_VTK=OFF \
        -DUSE_FREEIMAGE=OFF \
        -DUSE_OPENGL=OFF \
        -DUSE_GLES2=OFF \
        -DUSE_XLIB=OFF \
        -DBUILD_DOC_Overview=OFF \
        -DBUILD_MODULE_ApplicationFramework=OFF \
        -DBUILD_MODULE_Draw=OFF \
        -DBUILD_MODULE_Visualization=OFF \
    && cmake --build OCCT-7_6_0/build \
    && cmake --install OCCT-7_6_0/build \
    && rm -rf OCCT-7_6_0

# ── OpenCV 4.6.0 (exact BambuStudio pin, minimal core-only build) ──────────
RUN curl -sL https://github.com/opencv/opencv/archive/refs/tags/4.6.0.tar.gz \
        | tar xz \
    && cmake -S opencv-4.6.0 -B opencv-4.6.0/build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/deps/destdir \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        -DBUILD_SHARED_LIBS=OFF \
        -DBUILD_TESTS=OFF \
        -DBUILD_PERF_TESTS=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_opencv_apps=OFF \
        -DBUILD_opencv_java=OFF \
        -DBUILD_opencv_python2=OFF \
        -DBUILD_opencv_python3=OFF \
        -DBUILD_opencv_highgui=OFF \
        -DBUILD_LIST=core,imgcodecs,imgproc \
        -DBUILD_opencv_world=OFF \
        -DWITH_CUDA=OFF \
        -DWITH_EIGEN=OFF \
        -DWITH_IPP=OFF \
        -DWITH_ITT=OFF \
        -DWITH_FFMPEG=OFF \
        -DWITH_GSTREAMER=OFF \
        -DWITH_GTK_2_X=OFF \
        -DWITH_JASPER=OFF \
        -DWITH_LAPACK=OFF \
        -DWITH_OPENCL=OFF \
        -DWITH_OPENEXR=OFF \
        -DWITH_OPENJPEG=OFF \
        -DWITH_PROTOBUF=OFF \
        -DWITH_QUIRC=OFF \
        -DWITH_VTK=OFF \
        -DWITH_WEBP=OFF \
        -DWITH_ADE=OFF \
        -DENABLE_PRECOMPILED_HEADERS=OFF \
    && cmake --build opencv-4.6.0/build \
    && cmake --install opencv-4.6.0/build \
    && rm -rf opencv-4.6.0

# ---------------------------------------------------------------------------
# Stage 2: Build slicer_cli against the pinned deps
# ---------------------------------------------------------------------------
FROM deps AS builder

WORKDIR /src
COPY cli/         cli/
COPY libslic3r/   libslic3r/
COPY references/BambuStudio/src/libslic3r/ references/BambuStudio/src/libslic3r/

# Build slicer_cli.
# CMAKE_PREFIX_PATH points to our from-source deps so they take priority
# over any system packages.  System paths are fallback for libraries we
# left as system packages (zlib, libpng, expat, etc).
RUN mkdir -p cli/build \
    && cd cli/build \
    && cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_PREFIX_PATH="/deps/destdir;/usr" \
    && make -j"$(nproc)"

# Collect every shared library the binary needs at runtime
RUN mkdir -p /runtime-libs \
    && ldd /src/cli/build/slicer_cli \
        | awk '/=>/ { print $3 }' \
        | sort -u \
        | while read -r lib; do \
            [ -f "$lib" ] && cp --dereference "$lib" /runtime-libs/; \
        done

# ---------------------------------------------------------------------------
# Stage 3: Slim runtime image
# ---------------------------------------------------------------------------
FROM ubuntu:24.04 AS runtime

ENV DEBIAN_FRONTEND=noninteractive

# Minimal runtime packages:
#   - fontconfig + fonts so FreeType/Fontconfig don't error at runtime
#   - locales so Boost.Locale has proper locale data
#   - ca-certificates for any HTTPS calls
RUN apt-get update -qq && apt-get install -y -qq --no-install-recommends \
        fontconfig \
        fonts-dejavu-core \
        ca-certificates \
        locales \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US.UTF-8

ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    LANGUAGE=en_US:en

# Copy the harvested shared libraries and refresh the linker cache
COPY --from=builder /runtime-libs/ /usr/local/lib/
RUN ldconfig

# Copy the compiled binary
COPY --from=builder /src/cli/build/slicer_cli /usr/local/bin/slicer_cli

# Copy BambuStudio printer/filament/process profiles so they are available
# inside the container at /profiles (BBL, Prusa, Creality, Voron, etc.)
COPY libslic3r/bambustudio/resources/profiles/ /profiles/

# Default working directory for input/output via volume mounts
WORKDIR /data

ENTRYPOINT ["slicer_cli"]
CMD ["--help"]
