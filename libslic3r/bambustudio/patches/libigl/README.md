# libigl Patch Overlay

This directory intentionally stores only files that differ from
`references/BambuStudio/src/libigl`.

`libslic3r/bambustudio/CMakeLists.txt` copies the reference libigl tree into the
build directory, overlays these files, and then builds against that generated
tree. This keeps the repository patch small while preserving libigl's relative
include behavior.

The patched files replace removed Eigen 3.4 `DynamicSparseMatrix` usage:

- `igl/cat.cpp`
- `igl/diag.cpp`
- `igl/slice.cpp`
- `igl/slice_into.cpp`

