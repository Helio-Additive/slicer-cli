# Exact Bambu-patched CGAL 5.4, required by BambuStudio v02.08.01.55.
vcpkg_buildpath_length_warning(37)

vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO CGAL/cgal
    REF v5.4
    SHA512 c9cdacc74844a6eca94980d0350ae6defb99462ef70ddc3e15e825f06b171a21571efd9246a4abac16a6efc350aa9fa79330d2e89dcec24fc6ecff51905efdeb
    HEAD_REF master
    PATCHES
        "${CMAKE_CURRENT_LIST_DIR}/../../../references/BambuStudio/deps/CGAL/0001-clang19.patch"
)

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        -DCGAL_HEADER_ONLY=ON
        -DCGAL_INSTALL_CMAKE_DIR=share/cgal
        -DBUILD_TESTING=OFF
        -DBUILD_DOC=OFF
        -DCGAL_BUILD_THREE_DOC=OFF
)

vcpkg_cmake_install()
vcpkg_cmake_config_fixup()
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug" "${CURRENT_PACKAGES_DIR}/share/doc" "${CURRENT_PACKAGES_DIR}/share/man")

vcpkg_install_copyright(FILE_LIST
    "${SOURCE_PATH}/Installation/LICENSE"
    "${SOURCE_PATH}/Installation/LICENSE.BSL"
    "${SOURCE_PATH}/Installation/LICENSE.RFL"
    "${SOURCE_PATH}/Installation/LICENSE.GPL"
    "${SOURCE_PATH}/Installation/LICENSE.LGPL"
)
