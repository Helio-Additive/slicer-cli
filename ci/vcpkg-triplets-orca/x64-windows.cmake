# The x64-windows triplet for the OrcaSlicer build, with one difference:
# OpenCASCADE is a static library. OrcaSlicer's own dependency build makes it
# static on Linux and macOS and shared on Windows
# (references/OrcaSlicer/deps/OCCT/OCCT.cmake). This build cannot use shared
# libraries on Windows: the package holds all three programs in one folder with
# one set of TK*.dll (7.9.3, for Bambu Studio and Snapmaker Orca), so
# OrcaSlicer's 8.0.1 cannot be a second set of DLLs with the same names.
set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE dynamic)
if(PORT STREQUAL "opencascade")
    set(VCPKG_LIBRARY_LINKAGE static)
    # The build links Release only; the debug libraries would double the size.
    set(VCPKG_BUILD_TYPE release)
endif()
