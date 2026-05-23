cmake_minimum_required(VERSION 3.0)
project(libigl)

add_library(libigl INTERFACE)

find_package(libigl QUIET)

if(libigl_FOUND)
    message(STATUS "IGL found, using system version...")
    target_link_libraries(libigl INTERFACE igl::core)
else()
    message(STATUS "IGL NOT found, using bundled version (patched override + reference)...")
    # Override dir first — the 3 patched files shadow the reference equivalents.
    # Everything else resolves from the reference tree.
    target_include_directories(libigl SYSTEM BEFORE INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}"
        "${REF_SRC}/libigl"
    )
endif()
