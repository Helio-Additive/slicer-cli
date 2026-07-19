# Exact Bambu-patched CGAL 5.4, required by BambuStudio v02.08.01.55.
vcpkg_buildpath_length_warning(37)

vcpkg_from_git(
    OUT_SOURCE_PATH SOURCE_PATH
    URL https://github.com/CGAL/cgal.git
    REF c58ac97e93c838ebfb1e8adaf23ff4fd185dc8e4
    FETCH_REF v5.4
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
