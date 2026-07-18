# spike/layout/CMakeLists.txt — DISPOSABLE spike harness for issue #7.
# Only reachable via -DSPIKE_LAYOUT=ON; never part of the production build.

add_executable(layout_spike layout_spike.cpp)
target_link_libraries(layout_spike PRIVATE libslic3r_core)
target_include_directories(layout_spike PRIVATE
    "${VENDOR}"
    "${REF_SRC}"
    "${LIBSLIC3R_DIR}"
    "${LIBSLIC3R_REF_DIR}"
    "${SUBLIB_DIR}/miniz"
    "${OpenCASCADE_INCLUDE_DIR}")

# Same ICU keg-only workaround as the slicer_cli target.
if(APPLE AND ICU_UC_LIB)
  get_filename_component(_icu_lib_dir "${ICU_UC_LIB}" DIRECTORY)
  target_link_directories(layout_spike PRIVATE "${_icu_lib_dir}")
endif()
