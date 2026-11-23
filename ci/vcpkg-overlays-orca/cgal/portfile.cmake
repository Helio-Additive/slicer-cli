# CGAL 6.2.1, the release OrcaSlicer v2.5.2 pins: the same archive and SHA-256
# (eebd737d9b7f0199647ba5c1f9f6c7a3d0651aacef9c2fd4dedc52da6c25edbe) as
# references/OrcaSlicer/deps/CGAL/CGAL.cmake at f92a7c0. The vcpkg baseline this
# build pins has CGAL 6.1.1, which lacks CGAL/boost/graph/border.h.
set(VCPKG_BUILD_TYPE release)  # header-only

vcpkg_download_distfile(ARCHIVE
    URLS "https://github.com/CGAL/cgal/releases/download/v6.2.1/CGAL-6.2.1.zip"
    FILENAME "CGAL-6.2.1.zip"
    SHA512 04873e82940b9a14379d2d5aa0185439201db843f3edf7ae1b676786f6334cb223954ea70746f6f0215dbbd05d719a424ca426033df8f182daf34b2cf25f0ac6
)
vcpkg_extract_source_archive(SOURCE_PATH ARCHIVE "${ARCHIVE}")

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        -DCGAL_HEADER_ONLY=ON
        -DCGAL_INSTALL_CMAKE_DIR=share/cgal
        -DBUILD_TESTING=OFF
        -DBUILD_DOC=OFF
        -DCGAL_BUILD_THREE_DOC=OFF
    MAYBE_UNUSED_VARIABLES
        CGAL_BUILD_THREE_DOC
        CGAL_HEADER_ONLY
)

vcpkg_cmake_install()
vcpkg_cmake_config_fixup()
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug" "${CURRENT_PACKAGES_DIR}/bin" "${CURRENT_PACKAGES_DIR}/share/doc" "${CURRENT_PACKAGES_DIR}/share/man")

vcpkg_install_copyright(FILE_LIST
    "${SOURCE_PATH}/LICENSE"
    "${SOURCE_PATH}/LICENSE.BSL"
    "${SOURCE_PATH}/LICENSE.RFL"
    "${SOURCE_PATH}/LICENSE.GPL"
    "${SOURCE_PATH}/LICENSE.LGPL"
)
