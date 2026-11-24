# OpenCASCADE 8.0.1 for the OrcaSlicer engine on Windows: vcpkg's own 8.0.1 port
# (microsoft/vcpkg, ports/opencascade at version 8.0.1#1) plus the one patch
# OrcaSlicer v2.5.2 builds its OpenCASCADE with
# (references/OrcaSlicer/deps/OCCT/OCCT.cmake at f92a7c0). The patch is read from
# the engine checkout, so there is one copy of it; its hash is checked here
# because vcpkg does not track a file outside the port folder.
set(ORCA_SEAM_PATCH "${CMAKE_CURRENT_LIST_DIR}/../../../references/OrcaSlicer/deps/OCCT/0001-BRepMesh-seam-pcurve-selection.patch")
if(NOT EXISTS "${ORCA_SEAM_PATCH}")
    message(FATAL_ERROR "The OrcaSlicer checkout is missing: ${ORCA_SEAM_PATCH}")
endif()
file(SHA256 "${ORCA_SEAM_PATCH}" ORCA_SEAM_PATCH_SHA256)
if(NOT ORCA_SEAM_PATCH_SHA256 STREQUAL "652d13689016f44bdde8012d73c885fda4e803a5ee4a6ea2cf0d570d5982cf72")
    message(FATAL_ERROR "OrcaSlicer's OpenCASCADE patch changed (${ORCA_SEAM_PATCH_SHA256}); review it and update this port.")
endif()

string(REPLACE "." "_" VERSION_STR "V${VERSION}")
vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO Open-Cascade-SAS/OCCT
    REF "${VERSION_STR}"
    SHA512 bbe7099071cbf5397940ebc6e66ec05f8023d5e5aae6142870e14b93aa6f8f94c30980ef421e717f0fbfbc23b3520c3ccfe8a939c4caba3ccbf325060e26eb52
    HEAD_REF master
    PATCHES
        0001-cmake-keep-build-use-vcpkg-explicit.patch
        0002-cmake-load-exported-package-dependencies.patch
        0003-image-remove-freeimage-msvc-autolink.patch
        0004-cmake-add-additional-path-extraction-for-OpenCASCADE.patch
        0005-drop-bin-letter.patch
        "${ORCA_SEAM_PATCH}"
)

set(STATIC_FLAG_OPTIONS "")
if (VCPKG_LIBRARY_LINKAGE STREQUAL "dynamic")
    set(BUILD_TYPE "Shared")
else()
    set(BUILD_TYPE "Static")
    if (VCPKG_TARGET_IS_WINDOWS AND NOT VCPKG_TARGET_IS_MINGW)
        # vcpkg compiles with /Z7, which puts the debug information into each
        # object: the static libraries then take several gigabytes. These are
        # vcpkg's release flags (scripts/toolchains/windows.cmake) without /Z7.
        if (VCPKG_CRT_LINKAGE STREQUAL "dynamic")
            set(CRT_FLAG "/MD")
        else()
            set(CRT_FLAG "/MT")
        endif()
        set(STATIC_FLAG_OPTIONS
            "-DCMAKE_C_FLAGS_RELEASE=${CRT_FLAG} /O2 /Oi /Gy /DNDEBUG"
            "-DCMAKE_CXX_FLAGS_RELEASE=${CRT_FLAG} /O2 /Oi /Gy /DNDEBUG"
        )
    endif()
endif()

vcpkg_check_features(OUT_FEATURE_OPTIONS FEATURE_OPTIONS
    FEATURES
        freetype    USE_FREETYPE
        rapidjson   USE_RAPIDJSON
        tbb         USE_TBB
        vtk         USE_VTK
)

# We turn off BUILD_MODULE_Draw as it requires TCL 8.6 and TK 8.6 specifically which conflicts with vcpkg only having TCL 9.0
# And pre-built ActiveTCL binaries are behind a marketing wall :(
# We use the Unix install layout for Windows as it matches vcpkg
vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        ${FEATURE_OPTIONS}
        -DBUILD_LIBRARY_TYPE=${BUILD_TYPE}
        -DBUILD_MODULE_Draw=OFF
        -DBUILD_DOC_Overview=OFF
        -DINSTALL_DIR_LAYOUT=Unix
        -DINSTALL_DIR_DOC=share/trash
        -DINSTALL_DIR_SCRIPT=share/trash # not relocatable
        -DINSTALL_TEST_CASES=OFF
        -DUSE_FREEIMAGE=OFF
        -DUSE_TK=OFF
    OPTIONS_RELEASE
        ${STATIC_FLAG_OPTIONS}
)

vcpkg_cmake_install()
vcpkg_copy_pdbs()
vcpkg_cmake_config_fixup(CONFIG_PATH lib/cmake/opencascade)

#make occt includes relative to source_file
file(GLOB extra_headers
    LIST_DIRECTORIES false
    RELATIVE "${CURRENT_PACKAGES_DIR}/include/opencascade"
    "${CURRENT_PACKAGES_DIR}/include/opencascade/*.h"
)
list(JOIN extra_headers "|" extra_headers)
file(GLOB files "${CURRENT_PACKAGES_DIR}/include/opencascade/*.[hgl]xx")
foreach(file_name IN LISTS files)
    vcpkg_replace_string("${file_name}" "(# *include) <([a-zA-Z0-9_]*[.][hgl]xx|${extra_headers})>" [[\1 "\2"]] REGEX IGNORE_UNCHANGED)
endforeach()

if(VCPKG_LIBRARY_LINKAGE STREQUAL "static")
    vcpkg_replace_string("${CURRENT_PACKAGES_DIR}/include/opencascade/Standard_Macro.hxx" "defined(OCCT_STATIC_BUILD)" "(1)")
endif()

file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/share")
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/share/opencascade/samples/qt")
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/share/trash")

vcpkg_install_copyright(
    FILE_LIST
        "${SOURCE_PATH}/LICENSE_LGPL_21.txt"
        "${SOURCE_PATH}/OCCT_LGPL_EXCEPTION.txt"
)
