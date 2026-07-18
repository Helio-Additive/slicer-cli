cmake_minimum_required(VERSION 3.13)
project(slicer_cli VERSION 1.0.0)

# Standalone CLI build of libslic3r — fully self-contained.
#
# Source tree layout:
#
#   Primary source (read-only reference — do not edit):
#     references/BambuStudio/src/libslic3r/
#
#   Override/patch layer (files here shadow the reference 1-for-1):
#     libslic3r/bambustudio/libslic3r/
#
#   To replace a reference file, create a file at the same relative path
#   inside the override layer.  It will be compiled instead of the reference
#   file, and its directory will take priority in #include resolution.
#   Files with no override are compiled directly from the reference tree.
#
#   Bundled sub-libraries — compiled directly from the BambuStudio reference tree:
#     references/BambuStudio/src/admesh/
#     references/BambuStudio/src/clipper/
#     references/BambuStudio/src/clipper2/
#     references/BambuStudio/src/miniz/
#     references/BambuStudio/src/glu-libtess/
#     references/BambuStudio/src/semver/
#     references/BambuStudio/src/libnest2d/
#     references/BambuStudio/src/mcut/
#     references/BambuStudio/src/boost/         — Boost.Nowide
#
#   Bundled sub-libraries — kept under the VENDOR override layer (patched):
#     libslic3r/bambustudio/libigl/             — 4 source files patched to replace
#                                                    removed Eigen::DynamicSparseMatrix API
#                                                    (cat/diag/slice/slice_into).
#                                                    Full tree required: the override works
#                                                    via search-path anchoring — angle-bracket
#                                                    includes like <igl/copyleft/cgal/...>
#                                                    must resolve to VENDOR first so that all
#                                                    subsequent relative ../../ includes stay
#                                                    anchored to the patched tree.
#
# External deps come from system packages (Homebrew / apt).

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# MSVC needs _USE_MATH_DEFINES to expose M_PI etc. from <math.h>/<cmath>;
# NOMINMAX prevents <windows.h> min/max macros from clashing with std::min/max.
if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES NOMINMAX)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(APPLE)
  set(CMAKE_OSX_DEPLOYMENT_TARGET "10.15" CACHE STRING "" FORCE)
endif()

# BambuStudio was developed on x86/macOS where `char` is signed by default.
# On Linux ARM64 (and some other platforms) `char` is unsigned, which breaks
# enums with `: char` underlying type that use negative sentinels like `-1`
# (e.g. BuildVolume::Type::Invalid).  Force signed char globally to match the
# upstream's assumptions without touching any reference source files.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang" AND NOT APPLE)
  add_compile_options(-fsigned-char)
endif()

# Allow old cmake_minimum_required() inside vendored sub-libraries
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

# ─── Paths ──────────────────────────────────────────────────────────────────

# Bundled sub-libraries live here (admesh, clipper, boost/nowide, etc.)
# get_filename_component ABSOLUTE resolves the ".." so that paths returned by
# file(GLOB_RECURSE) — which are always normalized — match the strings we
# construct for list(REMOVE_ITEM) and EXISTS checks.
# ─── Engine selection (bambu | orca) ────────────────────────────────────────
# The CLI driver (main.cpp) compiles against ONE engine's libslic3r at a time:
#   -DENGINE=bambu  (default)  → references/BambuStudio  + libslic3r/bambustudio
#   -DENGINE=orca              → references/OrcaSlicer    + libslic3r/orcaslicer
# Both are PrusaSlicer forks sharing the same public API (Model, Print, etc.).
# Differences captured below:
#   * ENGINE_ROOT — submodule root.  REF_SRC = ENGINE_ROOT/src.
#   * SUBLIB_DIR  — where the bundled sub-libraries live:
#                     BambuStudio: src/        (admesh, clipper, …)
#                     OrcaSlicer:  deps_src/   (admesh, clipper, …)
#   * ENGINE_BAMBU / ENGINE_ORCA compile define → gates the divergent front-end
#     blocks in main.cpp (preset/staging resolution, toolchanger normalizations).
set(ENGINE "bambu" CACHE STRING "Slicing engine source: bambu (BambuStudio) or orca (OrcaSlicer)")
set_property(CACHE ENGINE PROPERTY STRINGS bambu orca)
set(ENGINE_EXTRA_LIBS "")   # engine-specific extra link libraries (populated below)

if(ENGINE STREQUAL "orca")
  get_filename_component(ENGINE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/references/OrcaSlicer"   ABSOLUTE)
  get_filename_component(VENDOR      "${CMAKE_CURRENT_SOURCE_DIR}/libslic3r/orcaslicer"    ABSOLUTE)
  set(SUBLIB_DIR "${ENGINE_ROOT}/deps_src")   # Orca bundles sub-libs under deps_src/
  add_compile_definitions(ENGINE_ORCA)
  message(STATUS "ENGINE = orca (OrcaSlicer)")
elseif(ENGINE STREQUAL "bambu")
  get_filename_component(ENGINE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/references/BambuStudio"  ABSOLUTE)
  get_filename_component(VENDOR      "${CMAKE_CURRENT_SOURCE_DIR}/libslic3r/bambustudio"   ABSOLUTE)
  set(SUBLIB_DIR "${ENGINE_ROOT}/src")        # BambuStudio bundles sub-libs under src/
  add_compile_definitions(ENGINE_BAMBU)
  message(STATUS "ENGINE = bambu (BambuStudio)")
else()
  message(FATAL_ERROR "Unknown ENGINE='${ENGINE}'. Use -DENGINE=bambu or -DENGINE=orca")
endif()

get_filename_component(REF_SRC "${ENGINE_ROOT}/src" ABSOLUTE)

# Primary source: exact engine reference files (do not edit these)
set(LIBSLIC3R_REF_DIR "${REF_SRC}/libslic3r")

# Override/patch layer: place files here to shadow the reference 1-for-1
set(LIBSLIC3R_DIR "${VENDOR}/libslic3r")

# libigl tree.  BambuStudio ships a patched copy under the VENDOR override layer
# (4 files patched for removed Eigen::DynamicSparseMatrix APIs); OrcaSlicer ships
# its own under deps_src/libigl.  Prefer the override copy when present, else the
# engine's bundled copy.
if(EXISTS "${VENDOR}/libigl/CMakeLists.txt")
  set(LIBIGL_DIR "${VENDOR}/libigl")
elseif(EXISTS "${SUBLIB_DIR}/libigl/CMakeLists.txt")
  set(LIBIGL_DIR "${SUBLIB_DIR}/libigl")
else()
  set(LIBIGL_DIR "${REF_SRC}/libigl")
endif()

if(NOT EXISTS "${LIBSLIC3R_REF_DIR}")
  message(FATAL_ERROR
        "Reference libslic3r not found at ${LIBSLIC3R_REF_DIR}\n"
        "Ensure the ${ENGINE} submodule is initialised under ${ENGINE_ROOT}")
endif()

message(STATUS "============================================")
message(STATUS "slicer_cli")
message(STATUS "  reference:  ${LIBSLIC3R_REF_DIR}")
message(STATUS "  override:   ${LIBSLIC3R_DIR}")
message(STATUS "  vendor:     ${VENDOR}")
message(STATUS "============================================")

# ─── Override-aware file resolver ───────────────────────────────────────────
# Resolve a path relative to the libslic3r root, preferring the override layer
# over the reference tree.  Usage:
#   libslic3r_src(MY_VAR "GCode/SomeFile.cpp")
function(libslic3r_src result rel_path)
  if(EXISTS "${LIBSLIC3R_DIR}/${rel_path}")
    set(${result} "${LIBSLIC3R_DIR}/${rel_path}" PARENT_SCOPE)
  else()
    set(${result} "${LIBSLIC3R_REF_DIR}/${rel_path}" PARENT_SCOPE)
  endif()
endfunction()

# ─── System dependencies ────────────────────────────────────────────────────
find_package(Threads REQUIRED)
find_package(TBB     REQUIRED)
find_package(Eigen3  REQUIRED)
find_package(PNG     REQUIRED)
find_package(ZLIB    REQUIRED)
find_package(EXPAT   REQUIRED)
find_package(OpenSSL REQUIRED)

set(Boost_NO_WARN_NEW_VERSIONS ON)
# boost_system is header-only since 1.69 — don't request it as a component.
# Try static first, fall back to shared.
set(Boost_USE_STATIC_LIBS ON)
find_package(Boost 1.73 QUIET COMPONENTS
    filesystem thread log log_setup regex atomic locale)
if(NOT Boost_FOUND)
  set(Boost_USE_STATIC_LIBS OFF)
  find_package(Boost 1.73 REQUIRED COMPONENTS
        filesystem thread log log_setup regex atomic locale)
endif()

set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE ON CACHE BOOL "" FORCE)
find_package(CGAL REQUIRED)

find_package(OpenCV REQUIRED core)

find_package(NLopt QUIET)
if(NLopt_FOUND)
  message(STATUS "NLopt found")
else()
  message(WARNING "NLopt not found — arrangement quality may be reduced")
endif()

# OpenCASCADE — Homebrew installs as "opencascade"
find_package(OpenCASCADE QUIET)
if(NOT OpenCASCADE_FOUND)
  find_package(OpenCASCADE QUIET PATHS
      /opt/homebrew/lib/cmake/opencascade
      /usr/local/lib/cmake/opencascade)
endif()
if(NOT OpenCASCADE_FOUND)
  message(FATAL_ERROR "OpenCASCADE (OCCT) not found. Install: brew install opencascade")
endif()

# Qhull — cmake package ships as "Qhull" with targets like Qhull::qhullcpp
find_package(Qhull QUIET CONFIG)
if(NOT Qhull_FOUND)
  find_library(QHULL_CPP_LIB     qhullcpp)
  find_library(QHULL_STATIC_LIB  qhullstatic_r)
  find_path(QHULL_INCLUDE_DIR    libqhullcpp/Qhull.h)
  if(NOT QHULL_CPP_LIB OR NOT QHULL_STATIC_LIB)
    message(FATAL_ERROR "Qhull not found. Install: brew install qhull")
  endif()
endif()

# Cereal (header-only)
find_path(CEREAL_INCLUDE_DIR cereal/cereal.hpp
    HINTS /opt/homebrew/include /usr/local/include /usr/include)
if(NOT CEREAL_INCLUDE_DIR)
  message(FATAL_ERROR "Cereal not found. Install: brew install cereal")
endif()

# libnoise — required by FuzzySkin.cpp since BBS v02.05.03.
# Source: https://github.com/bambulab/libnoise (tag v1.0.0)
# Build+install libnoise once, then pass CMAKE_PREFIX_PATH=<install-root>
# when configuring. Uses BBS's own Findlibnoise.cmake module.
list(APPEND CMAKE_MODULE_PATH "${REF_SRC}/../cmake/modules")
find_package(libnoise REQUIRED)

message(STATUS "============================================")
message(STATUS "  TBB:            ${TBB_VERSION}")
message(STATUS "  Boost:          ${Boost_VERSION}")
message(STATUS "  Eigen3:         ${EIGEN3_VERSION_STRING}")
message(STATUS "  CGAL:           ${CGAL_VERSION}")
message(STATUS "  OpenCV:         ${OpenCV_VERSION}")
message(STATUS "  OpenCASCADE:    ${OpenCASCADE_INSTALL_PREFIX}")
message(STATUS "  Cereal:         ${CEREAL_INCLUDE_DIR}")
message(STATUS "============================================")

# ─── Stub required by some BambuStudio sub-library CMakeLists ───────────────
function(encoding_check TARGET)
endfunction()

# ─── Version header ─────────────────────────────────────────────────────────
# Pull SLIC3R_VERSION, SLIC3R_APP_NAME, SLIC3R_APP_KEY from the submodule's
# version.inc so bumping the submodule auto-updates the version the CLI
# reports in its gcode headers.
include("${REF_SRC}/../version.inc")
set(SLIC3R_BUILD_ID         "standalone-cli")
set(SLIC3R_BUILD_TIME       "standalone-cli")
set(SLIC3R_COMPILE_VERSION  "standalone-cli")
set(BBL_RELEASE_TO_PUBLIC   1)
set(BBL_INTERNAL_TESTING    0)
# OrcaSlicer's libslic3r_version.h.in references @ORCA_CHECK_GCODE_PLACEHOLDERS@.
# Orca's own CMake sets it to 1 only in debug; left unset, configure_file would
# substitute an empty string → "#define ORCA_CHECK_GCODE_PLACEHOLDERS " → a
# "#if <empty>" preprocessor error in GCode.hpp. Disable it for the release CLI.
set(ORCA_CHECK_GCODE_PLACEHOLDERS 0)

# Use the override version template if present, otherwise fall back to reference.
if(EXISTS "${LIBSLIC3R_DIR}/libslic3r_version.h.in")
  set(_ver_in "${LIBSLIC3R_DIR}/libslic3r_version.h.in")
else()
  set(_ver_in "${LIBSLIC3R_REF_DIR}/libslic3r_version.h.in")
endif()
configure_file("${_ver_in}" "${CMAKE_CURRENT_BINARY_DIR}/libslic3r_version.h" @ONLY)

# ─── Global include directories ─────────────────────────────────────────────
# Listed in priority order — the first matching directory wins for #include.
#
#   1. VENDOR        → override layer: "libslic3r/..." overrides
#   2. VENDOR/libigl → patched libigl tree; must come before REF_SRC/libigl so that
#                      angle-bracket includes like <igl/copyleft/cgal/mesh_boolean.h>
#                      resolve to VENDOR first, anchoring all subsequent relative
#                      ../../ includes to the patched files (e.g. patched slice.cpp).
#   3. REF_SRC       → reference tree: "libslic3r/..." fallback + all other sub-lib headers
#   4. REF_SRC/libigl → fallback (VENDOR tree shadows all files actually used)
#
include_directories(
    "${VENDOR}"                   # override: "libslic3r/..." overrides
    "${LIBIGL_DIR}"               # libigl tree (patched copy for BBS, deps_src for Orca)
    "${REF_SRC}"                  # reference: "libslic3r/..." fallback + (BBS) sub-lib headers
    "${SUBLIB_DIR}"               # sub-lib roots (nanosvg/, etc.) — == REF_SRC for bambu
    "${SUBLIB_DIR}/nanosvg"       # nanosvg bare-include path: #include "nanosvg.h"
    "${CMAKE_CURRENT_BINARY_DIR}" # libslic3r_version.h
)

# Eigen — add the include path globally so sub-libraries (admesh, clipper, etc.)
# that use #include <Eigen/...> can find the headers without each needing to
# link against Eigen3::Eigen explicitly.
# FindEigen3 (module mode, Homebrew) sets EIGEN3_INCLUDE_DIR.
# Eigen3Config (config mode, vcpkg) sets Eigen3::Eigen target but may not set
# the legacy variable. Fall back to extracting from the target.
if(EIGEN3_INCLUDE_DIR)
  include_directories(SYSTEM "${EIGEN3_INCLUDE_DIR}")
elseif(TARGET Eigen3::Eigen)
  get_target_property(_eigen3_incs Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
  if(_eigen3_incs)
    include_directories(SYSTEM ${_eigen3_incs})
  endif()
endif()

# On macOS, Homebrew Eigen may also live here
if(EXISTS /opt/homebrew/opt/eigen/include/eigen3)
  include_directories(SYSTEM /opt/homebrew/opt/eigen/include/eigen3)
endif()

# OpenCV — Homebrew nests headers under opencv4/
if(OpenCV_INCLUDE_DIRS)
  include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
endif()
if(EXISTS /opt/homebrew/opt/opencv/include/opencv4)
  include_directories(SYSTEM /opt/homebrew/opt/opencv/include/opencv4)
endif()

# ─── Sub-libraries ──────────────────────────────────────────────────────────
#
# All sub-libraries are compiled directly from the BambuStudio reference tree.
# The VENDOR path is used only for the libslic3r override layer.
#
add_subdirectory("${SUBLIB_DIR}/admesh"      "${CMAKE_CURRENT_BINARY_DIR}/admesh")
add_subdirectory("${SUBLIB_DIR}/clipper"     "${CMAKE_CURRENT_BINARY_DIR}/clipper")
add_subdirectory("${SUBLIB_DIR}/clipper2"    "${CMAKE_CURRENT_BINARY_DIR}/clipper2")
add_subdirectory("${SUBLIB_DIR}/miniz"       "${CMAKE_CURRENT_BINARY_DIR}/miniz")
add_subdirectory("${SUBLIB_DIR}/glu-libtess" "${CMAKE_CURRENT_BINARY_DIR}/glu-libtess")
add_subdirectory("${SUBLIB_DIR}/semver"      "${CMAKE_CURRENT_BINARY_DIR}/semver")
add_subdirectory("${SUBLIB_DIR}/libnest2d"   "${CMAKE_CURRENT_BINARY_DIR}/libnest2d")
add_subdirectory("${LIBIGL_DIR}"             "${CMAKE_CURRENT_BINARY_DIR}/libigl")
add_subdirectory("${SUBLIB_DIR}/mcut"        "${CMAKE_CURRENT_BINARY_DIR}/mcut")

# OrcaSlicer's GCode/Thumbnails.cpp compresses thumbnails as JPG (<jpeglib.h>) and
# QOI (bundled deps_src/qoi). BambuStudio handles thumbnails differently and needs
# neither. Wire them up only for the Orca engine.
if(ENGINE STREQUAL "orca")
  find_package(JPEG REQUIRED)
  add_subdirectory("${SUBLIB_DIR}/qoi" "${CMAKE_CURRENT_BINARY_DIR}/qoi")
  list(APPEND ENGINE_EXTRA_LIBS JPEG::JPEG qoi)
  message(STATUS "  JPEG:           ${JPEG_LIBRARIES}")
endif()

# Boost.Nowide
if(NOT TARGET boost_headeronly)
  add_library(boost_headeronly INTERFACE)
  target_include_directories(boost_headeronly INTERFACE ${Boost_INCLUDE_DIRS})
endif()
if(NOT TARGET nowide)
  if(WIN32)
    # On Windows, use vcpkg's boost-nowide DLL. Building from the bundled
    # source as a static lib produces LNK2019 unresolved externals on
    # boost::nowide::cerr/cout because consumer TUs see __declspec(dllimport)
    # whenever vcpkg's boost auto-defines BOOST_ALL_DYN_LINK.
    find_package(Boost CONFIG REQUIRED COMPONENTS nowide)
    add_library(nowide INTERFACE)
    target_link_libraries(nowide INTERFACE Boost::nowide)
  elseif(EXISTS "${SUBLIB_DIR}/boost/nowide")
    # BambuStudio bundles Boost.Nowide source under src/boost.
    add_subdirectory("${SUBLIB_DIR}/boost" "${CMAKE_CURRENT_BINARY_DIR}/boost")
  else()
    # OrcaSlicer does NOT bundle Boost.Nowide — use the system Boost component.
    find_package(Boost CONFIG REQUIRED COMPONENTS nowide)
    add_library(nowide INTERFACE)
    target_link_libraries(nowide INTERFACE Boost::nowide)
  endif()
endif()

# Qhull — wrap in a single interface target.
# Homebrew's qhull provides qhullstatic_r; vcpkg's port ships qhull_r only.
add_library(qhull INTERFACE)
if(TARGET Qhull::qhullcpp)
  if(TARGET Qhull::qhullstatic_r)
    target_link_libraries(qhull INTERFACE Qhull::qhullcpp Qhull::qhullstatic_r)
  elseif(TARGET Qhull::qhull_r)
    target_link_libraries(qhull INTERFACE Qhull::qhullcpp Qhull::qhull_r)
  else()
    message(FATAL_ERROR "Qhull found via CMake config but neither Qhull::qhullstatic_r nor Qhull::qhull_r exists")
  endif()
else()
  target_link_libraries(qhull INTERFACE ${QHULL_CPP_LIB} ${QHULL_STATIC_LIB})
  if(QHULL_INCLUDE_DIR)
    target_include_directories(qhull INTERFACE "${QHULL_INCLUDE_DIR}")
  endif()
endif()

# Cereal target
add_library(cereal INTERFACE)
target_include_directories(cereal INTERFACE "${CEREAL_INCLUDE_DIR}")

# ─── Collect libslic3r sources ──────────────────────────────────────────────
#
# Strategy:
#   1. Start with the full reference tree (GLOB_RECURSE on LIBSLIC3R_REF_DIR).
#   2. Collect override files (GLOB_RECURSE on LIBSLIC3R_DIR).
#   3. For each override file, remove the corresponding reference file from the
#      list and append the override in its place.
#
# The result is: every reference file is compiled AS-IS unless an identically-
# named (relative-path-matching) file exists in the override layer, in which
# case the override file is used instead.

# Step 1 — reference tree
file(GLOB_RECURSE LIBSLIC3R_SOURCES
    "${LIBSLIC3R_REF_DIR}/*.cpp"
    "${LIBSLIC3R_REF_DIR}/*.c")

# Step 2 — override layer
file(GLOB_RECURSE _LIBSLIC3R_OVERRIDE_SOURCES
    "${LIBSLIC3R_DIR}/*.cpp"
    "${LIBSLIC3R_DIR}/*.c")

# Step 3 — shadow: for every override file, drop the matching reference file
foreach(_ovr ${_LIBSLIC3R_OVERRIDE_SOURCES})
  file(RELATIVE_PATH _rel "${LIBSLIC3R_DIR}" "${_ovr}")
  list(REMOVE_ITEM LIBSLIC3R_SOURCES "${LIBSLIC3R_REF_DIR}/${_rel}")
  list(APPEND LIBSLIC3R_SOURCES "${_ovr}")
endforeach()

# Exclude files that don't compile / aren't needed in headless mode
list(FILTER LIBSLIC3R_SOURCES EXCLUDE REGEX ".*pchheader\\.cpp$")
list(FILTER LIBSLIC3R_SOURCES EXCLUDE REGEX ".*ExPolygonCollection\\.cpp$")
list(FILTER LIBSLIC3R_SOURCES EXCLUDE REGEX ".*GCodeSender\\.cpp$")
if(ENGINE STREQUAL "bambu")
  # BambuStudio's GCode.cpp does not reference PressureEqualizer; OrcaSlicer's
  # GCode.cpp (_do_export / process_layers) does, so Orca MUST compile it.
  list(FILTER LIBSLIC3R_SOURCES EXCLUDE REGEX ".*PressureEqualizer\\.cpp$")
endif()
list(FILTER LIBSLIC3R_SOURCES EXCLUDE REGEX ".*OpenVDBUtils\\.cpp$")
list(FILTER LIBSLIC3R_SOURCES EXCLUDE REGEX ".*SLA/Hollowing\\.cpp$")
list(FILTER LIBSLIC3R_SOURCES EXCLUDE REGEX ".*TryCatchSignalSEH\\.cpp$")
list(FILTER LIBSLIC3R_SOURCES EXCLUDE REGEX ".*LogSink\\.cpp$")
list(FILTER LIBSLIC3R_SOURCES EXCLUDE REGEX ".*NSVGUtils\\.cpp$")
list(FILTER LIBSLIC3R_SOURCES EXCLUDE REGEX ".*Print\\.cpp\\.ANNOTATED$")
list(FILTER LIBSLIC3R_SOURCES EXCLUDE REGEX ".*PostProcessor\\.cpp$")
# OrcaSlicer-only: Format/DRC.cpp is a Google-Draco (.drc) mesh-compression reader
# pulling <draco/...> (an external find_package(draco) in Orca's build). It is off
# the .3mf→gcode slice/export path, so exclude it rather than add the Draco dep.
list(FILTER LIBSLIC3R_SOURCES EXCLUDE REGEX ".*Format/DRC\\.cpp$")

if(NOT APPLE)
  list(FILTER LIBSLIC3R_SOURCES EXCLUDE REGEX ".*\\.mm$")
endif()

# ─── CGAL-dependent sources — compiled in a separate target ─────────────────
# CutSurface.cpp excluded: uses CGAL 5.x APIs (add_property_map returns pair,
# AABB_traits) that are incompatible with system CGAL 6.x.  Only needed for
# mesh cutting operations, not basic STL→gcode slicing.
libslic3r_src(_ip  "IntersectionPoints.cpp")
libslic3r_src(_mb  "MeshBoolean.cpp")
libslic3r_src(_tcs "TryCatchSignal.cpp")
libslic3r_src(_tri "Triangulation.cpp")
set(LIBSLIC3R_CGAL_SOURCES "${_ip}" "${_mb}" "${_tcs}" "${_tri}")

# Also exclude CutSurface.cpp from the main GLOB (it would have been removed
# anyway, but be explicit since it's no longer in the CGAL list either).
list(FILTER LIBSLIC3R_SOURCES EXCLUDE REGEX ".*CutSurface\\.cpp$")

foreach(src ${LIBSLIC3R_CGAL_SOURCES})
  list(REMOVE_ITEM LIBSLIC3R_SOURCES "${src}")
endforeach()

# ─── nanosvg implementation + stubs ─────────────────────────────────────────
set(NANOSVG_IMPL "${CMAKE_CURRENT_BINARY_DIR}/nanosvg_impl.cpp")
file(WRITE "${NANOSVG_IMPL}" "#define NANOSVG_IMPLEMENTATION\n#include \"nanosvg/nanosvg.h\"\n")

libslic3r_src(_nsvg "NSVGUtils.cpp")
list(APPEND LIBSLIC3R_SOURCES
    "${_nsvg}"
    "${NANOSVG_IMPL}"
    "${CMAKE_CURRENT_SOURCE_DIR}/standalone_stubs.cpp")

# ─── libslic3r_core (main library) ─────────────────────────────────────────
add_library(libslic3r_core ${LIBSLIC3R_SOURCES}
    "${CMAKE_CURRENT_BINARY_DIR}/libslic3r_version.h")

# Generate a compatibility shim for deprecated Boost APIs that were removed in
# newer Boost versions but are still used in the reference source.  By putting
# the shims here (force-included via PCH) we can keep the reference source files
# byte-for-byte identical to the BambuStudio repository.
set(COMPAT_HEADER "${CMAKE_CURRENT_BINARY_DIR}/standalone_compat.hpp")
file(WRITE "${COMPAT_HEADER}" [=[
#pragma once
// ── Boost.Filesystem compatibility shims ─────────────────────────────────────
// Boost 1.85 removed several deprecated filesystem APIs still used by the
// reference source.  Guard each shim so it is only defined when the installed
// Boost no longer provides the symbol natively (avoids ambiguous-overload or
// "referred to as struct" errors on Boost ≤1.84 where they still exist).
#include <boost/version.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#if BOOST_VERSION >= 108500
namespace boost { namespace filesystem {
    // Removed in Boost 1.85 — wrap the modern replace_extension().
    inline path change_extension(path p, const path& new_ext) {
        return p.replace_extension(new_ext);
    }
    // copy_option::overwrite_if_exists → copy_options::overwrite_existing
    struct copy_option {
        static constexpr copy_options overwrite_if_exists = copy_options::overwrite_existing;
    };
} } // namespace boost::filesystem
#endif // BOOST_VERSION >= 108500
]=])

# Match BambuStudio's FORCEINCLUDE of pchheader.hpp — injects the PCH into
# every translation unit, making std::stringstream, ExPolygon, Polygon etc.
# universally available without per-file includes.
# Use override pchheader.hpp if one exists, otherwise use the reference.
if(EXISTS "${LIBSLIC3R_DIR}/pchheader.hpp")
  set(_pch "${LIBSLIC3R_DIR}/pchheader.hpp")
else()
  set(_pch "${LIBSLIC3R_REF_DIR}/pchheader.hpp")
endif()

if(MSVC)
  # The bundled BambuStudio nowide headers (REF_SRC/boost/nowide/) use the old
  # namespace boost::nowide::details (with 's'); vcpkg's Boost.Nowide 1.90 DLL
  # exports boost::nowide::detail (no 's') — causing LNK2019 on cerr/cout.
  # The PCH (pchheader.hpp) includes <boost/nowide/iostream.hpp>, so the old
  # declarations get baked into cmake_pch.pch before any /FI can help.
  # Fix: inject vcpkg's iostream.hpp as the FIRST entry in target_precompile_headers
  # so it is compiled into the PCH before pchheader.hpp sees the bundled headers.
  # The shared include guard (BOOST_NOWIDE_IOSTREAM_HPP_INCLUDED) then prevents
  # the old bundled header from being processed anywhere in the TU.
  set(_nowide_fwd "${CMAKE_CURRENT_BINARY_DIR}/nowide_fwd.hpp")
  file(WRITE "${_nowide_fwd}"
    "#pragma once\n// Compiled first in the PCH to ensure vcpkg Boost.Nowide 1.90\n// (namespace boost::nowide::detail) precedes the bundled BambuStudio headers\n// (namespace boost::nowide::details) in every translation unit.\n#include \"${Boost_INCLUDE_DIRS}/boost/nowide/iostream.hpp\"\n")
  target_precompile_headers(libslic3r_core PRIVATE
      "${_nowide_fwd}"
      "${_pch}"
      "${COMPAT_HEADER}")
else()
  target_precompile_headers(libslic3r_core PRIVATE
      "${_pch}"
      "${COMPAT_HEADER}")
endif()

target_compile_definitions(libslic3r_core PUBLIC
    USE_TBB
    TBB_USE_CAPTURED_EXCEPTION=0)

# Include directories for libslic3r_core, in priority order:
#   1. VENDOR             — override headers: "libslic3r/..." and sub-lib headers
#   2. REF_SRC            — reference headers: "libslic3r/..." fallback
#   3. LIBSLIC3R_DIR      — override headers: bare "Foo.hpp" within libslic3r
#   4. LIBSLIC3R_REF_DIR  — reference headers: bare "Foo.hpp" fallback
target_include_directories(libslic3r_core PUBLIC
    "${VENDOR}"
    "${REF_SRC}"
    "${LIBSLIC3R_DIR}"
    "${LIBSLIC3R_REF_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${EIGEN3_INCLUDE_DIR}"
    "${OpenCASCADE_INCLUDE_DIR}")

target_include_directories(libslic3r_core PRIVATE
    "${EXPAT_INCLUDE_DIRS}"
    "${SUBLIB_DIR}/miniz")

# ─── libslic3r_cgal ────────────────────────────────────────────────────────
add_library(libslic3r_cgal STATIC ${LIBSLIC3R_CGAL_SOURCES})

target_include_directories(libslic3r_cgal PRIVATE
    "${VENDOR}"
    "${REF_SRC}"
    "${LIBSLIC3R_DIR}"
    "${LIBSLIC3R_REF_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${OPENSSL_INCLUDE_DIR}")

target_compile_definitions(libslic3r_cgal PRIVATE
    USE_TBB
    TBB_USE_CAPTURED_EXCEPTION=0)

# Isolate -frounding-math so it doesn't leak into the rest of the build
get_target_property(_cgal_tgt CGAL::CGAL ALIASED_TARGET)
if(NOT TARGET "${_cgal_tgt}")
  set(_cgal_tgt CGAL::CGAL)
endif()
get_target_property(_opts ${_cgal_tgt} INTERFACE_COMPILE_OPTIONS)
if(_opts)
  set(_bad  "${_opts}")
  set(_good "${_opts}")
  list(FILTER _bad  INCLUDE REGEX frounding-math)
  list(FILTER _good EXCLUDE REGEX frounding-math)
  set_target_properties(${_cgal_tgt} PROPERTIES INTERFACE_COMPILE_OPTIONS "${_good}")
  target_compile_options(libslic3r_cgal PRIVATE "${_bad}")
endif()

target_link_libraries(libslic3r_cgal PRIVATE ${_cgal_tgt} libigl mcut)

# ─── OpenCASCADE libraries ─────────────────────────────────────────────────
if(APPLE)
  set(OCCT_STEP_LIB TKDESTEP)
else()
  set(OCCT_STEP_LIB TKXDESTEP)
endif()

# OCCT 7.9+ merged the old TKSTEP/TKSTEPBase/TKSTEPAttr/TKSTEP209 into TKDESTEP + TKDE.
# Detect which layout is available.
find_library(_TKSTEP_LIB TKSTEP HINTS "${OpenCASCADE_INSTALL_PREFIX}/lib" /opt/homebrew/lib)
if(_TKSTEP_LIB)
  # Old OCCT layout (< 7.8)
  set(OCCT_LIBS
        ${OCCT_STEP_LIB}
        TKSTEP TKSTEPBase TKSTEPAttr TKSTEP209
        TKXCAF TKXSBase TKVCAF TKCAF TKLCAF TKCDF
        TKV3d TKService TKMesh TKBO TKPrim TKHLR
        TKShHealing TKTopAlgo TKGeomAlgo TKBRep
        TKGeomBase TKG3d TKG2d TKMath TKernel)
else()
  # New OCCT 7.9+ layout — STEP libs consolidated into TKDESTEP/TKDE
  set(OCCT_LIBS
        TKDESTEP TKDE
        TKXCAF TKXSBase TKVCAF TKCAF TKLCAF TKCDF
        TKV3d TKService TKMesh TKBO TKPrim TKHLR
        TKShHealing TKTopAlgo TKGeomAlgo TKBRep
        TKGeomBase TKG3d TKG2d TKMath TKernel)
endif()

# ─── Link everything ───────────────────────────────────────────────────────
target_link_libraries(libslic3r_core PUBLIC
    noise::noise
    libnest2d
    admesh
    cereal
    libigl
    miniz_static
    Boost::filesystem
    Boost::headers
    Boost::thread
    Boost::log
    Boost::log_setup
    Boost::regex
    Boost::atomic
    Boost::locale
    clipper
    Clipper2
    nowide
    ${EXPAT_LIBRARIES}
    glu-libtess
    qhull
    semver
    TBB::tbb
    TBB::tbbmalloc
    libslic3r_cgal
    ${CMAKE_DL_LIBS}
    PNG::PNG
    ZLIB::ZLIB
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wl,--start-group>
    ${OCCT_LIBS}
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wl,--end-group>
    mcut
    ${OpenCV_LIBS}
    ${ENGINE_EXTRA_LIBS}
    Threads::Threads
    OpenSSL::SSL
    OpenSSL::Crypto)

# Boost::locale static lib depends on ICU on all platforms.
# On macOS, Homebrew installs ICU at a keg-only (unlinked) path.
if(APPLE)
  find_library(ICU_UC_LIB   icuuc
      HINTS /opt/homebrew/opt/icu4c/lib
            /opt/homebrew/opt/icu4c@77/lib
            /opt/homebrew/opt/icu4c@78/lib
            /usr/local/opt/icu4c/lib
      NO_DEFAULT_PATH)
  find_library(ICU_I18N_LIB icui18n
      HINTS /opt/homebrew/opt/icu4c/lib
            /opt/homebrew/opt/icu4c@77/lib
            /opt/homebrew/opt/icu4c@78/lib
            /usr/local/opt/icu4c/lib
      NO_DEFAULT_PATH)
  find_library(ICU_DATA_LIB icudata
      HINTS /opt/homebrew/opt/icu4c/lib
            /opt/homebrew/opt/icu4c@77/lib
            /opt/homebrew/opt/icu4c@78/lib
            /usr/local/opt/icu4c/lib
      NO_DEFAULT_PATH)
  if(ICU_UC_LIB AND ICU_I18N_LIB)
    message(STATUS "ICU uc:   ${ICU_UC_LIB}")
    message(STATUS "ICU i18n: ${ICU_I18N_LIB}")
    message(STATUS "ICU data: ${ICU_DATA_LIB}")
    # Add the ICU lib directory to the linker search path so that icudata
    # (a transitive dep of icuuc) can be resolved even though it's keg-only.
    get_filename_component(ICU_LIB_DIR "${ICU_UC_LIB}" DIRECTORY)
    target_link_directories(libslic3r_core PUBLIC "${ICU_LIB_DIR}")
    target_link_libraries(libslic3r_core PUBLIC "${ICU_UC_LIB}" "${ICU_I18N_LIB}")
    if(ICU_DATA_LIB)
      target_link_libraries(libslic3r_core PUBLIC "${ICU_DATA_LIB}")
    endif()
  else()
    message(FATAL_ERROR "ICU not found in Homebrew keg paths. Run: brew install icu4c")
  endif()
elseif(UNIX)
  target_link_libraries(libslic3r_core PUBLIC icuuc icui18n)
endif()

if(APPLE)
  find_library(FOUNDATION Foundation REQUIRED)
  target_link_libraries(libslic3r_core PUBLIC ${FOUNDATION})
  find_library(MODELIO ModelIO)
  if(MODELIO)
    target_link_libraries(libslic3r_core PUBLIC ${MODELIO})
  endif()
  find_package(Freetype)
  if(FREETYPE_FOUND)
    target_link_libraries(libslic3r_core PUBLIC Freetype::Freetype)
  endif()
elseif(UNIX)
  find_package(Freetype  REQUIRED)
  find_package(Fontconfig REQUIRED)
  target_link_libraries(libslic3r_core PUBLIC Freetype::Freetype Fontconfig::Fontconfig)
elseif(WIN32)
  target_link_libraries(libslic3r_core PUBLIC Psapi.lib)
endif()

set_target_properties(libslic3r_core PROPERTIES
    OUTPUT_NAME slic3r_core
    VERSION     ${PROJECT_VERSION}
    SOVERSION   ${PROJECT_VERSION_MAJOR})

# Alias that libnest2d's CMakeLists.txt expects
add_library(libslic3r ALIAS libslic3r_core)

# ─── CLI executable ────────────────────────────────────────────────────────
add_executable(slicer_cli main.cpp calib_args.cpp)
target_link_libraries(slicer_cli PRIVATE libslic3r_core)
target_include_directories(slicer_cli PRIVATE
    "${VENDOR}"
    "${REF_SRC}"
    "${LIBSLIC3R_DIR}"
    "${LIBSLIC3R_REF_DIR}"
    "${SUBLIB_DIR}/miniz"
    "${OpenCASCADE_INCLUDE_DIR}")

# On macOS, ICU is keg-only — add the lib dir directly to the executable's
# link command so the linker can resolve icudata (a transitive dep of icuuc).
if(APPLE AND ICU_UC_LIB)
  get_filename_component(_icu_lib_dir "${ICU_UC_LIB}" DIRECTORY)
  target_link_directories(slicer_cli PRIVATE "${_icu_lib_dir}")
endif()

message(STATUS "============================================")
message(STATUS "  Library:    libslic3r_core")
message(STATUS "  Executable: slicer_cli")
message(STATUS "============================================")

# ─── Install rules ─────────────────────────────────────────────────────────
include(GNUInstallDirs)
install(TARGETS slicer_cli
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
        COMPONENT Runtime)
install(FILES
          "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE"
          "${CMAKE_CURRENT_SOURCE_DIR}/NOTICE"
          "${CMAKE_CURRENT_SOURCE_DIR}/README.md"
        DESTINATION "${CMAKE_INSTALL_DOCDIR}"
        COMPONENT Runtime)

# ─── CPack metadata ────────────────────────────────────────────────────────
# Set these AFTER all add_subdirectory() calls so that sub-library CPack
# variables (e.g. mcut sets CPACK_PACKAGE_NAME to "MCUT") are overridden.
# CI asserts the produced package identifies as slicer_cli — see
# .github/workflows/slicer-cli-release.yml step "Assert package metadata".
set(CPACK_PACKAGE_NAME             "slicer_cli")
set(CPACK_PACKAGE_VENDOR           "Helio Additive")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY
    "Standalone command-line FDM slicer derived from BambuStudio (AGPLv3)")
set(CPACK_PACKAGE_HOMEPAGE_URL     "https://github.com/helio-additive/slicer-cli")
set(CPACK_RESOURCE_FILE_LICENSE    "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_RESOURCE_FILE_README     "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
set(CPACK_PACKAGE_VERSION_MAJOR    ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR    ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH    ${PROJECT_VERSION_PATCH})
set(CPACK_PACKAGE_FILE_NAME
    "slicer_cli-${PROJECT_VERSION}-${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")
# Include only the Runtime component (the binary + license + readme).
# Development artefacts (libslic3r_core headers, static libs) are not packaged
# for public distribution — consumers should build from source.
set(CPACK_COMPONENTS_ALL           Runtime)
set(CPACK_STRIP_FILES              TRUE)

include(CPack)

# ─── Disposable layout-kernel spike (issue #7) ─────────────────────────────
option(SPIKE_LAYOUT "Build disposable layout spike harness" OFF)
if(SPIKE_LAYOUT)
  add_subdirectory(spike/layout)
endif()
