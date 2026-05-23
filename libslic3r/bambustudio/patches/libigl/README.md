# libigl — Patched Vendor Copy

This directory is a **full copy** of the `libigl` tree from
`references/BambuStudio/src/libigl/`, with four source files patched for
compatibility with Eigen 3.4+.

---

## Why does the full tree live here?

The short answer: **relative `#include` paths make it impossible to override
just the 4 patched files.**

libigl uses a header-only / unity-build pattern where each `.h` file
conditionally includes its corresponding `.cpp` at the bottom:

```cpp
// igl/slice.h (bottom)
#ifndef IGL_STATIC_LIBRARY
#  include "slice.cpp"
#endif
```

The include chain that triggers the problem looks like this:

```
MeshBoolean.cpp
  → #include <igl/copyleft/cgal/mesh_boolean.h>   ← angle-bracket, uses search path
    → #include "mesh_boolean.cpp"                 ← relative to mesh_boolean.h's location
      → #include "../../slice.h"                  ← relative to copyleft/cgal/
        → #include "slice.cpp"                    ← relative to igl/
```

The crucial detail is that `#include <igl/copyleft/cgal/mesh_boolean.h>` uses
**angle brackets**, so the compiler resolves it via the `-I` search path.
Whichever directory appears first in the search path wins.  Once the compiler
finds `mesh_boolean.h` on disk, **all subsequent relative `../../` includes are
anchored to that file's actual location on disk** — the search path is no longer
consulted for them.

This means:

- If `mesh_boolean.h` is found in `references/BambuStudio/src/libigl/`, every
  relative include chains through the **reference tree**, picking up the
  unpatched `slice.cpp`.
- If `mesh_boolean.h` is found in `libslic3r/bambustudio/libigl/`, every
  relative include chains through **this directory**, picking up the patched
  `slice.cpp`.

For the patched files to take effect, `libslic3r/bambustudio/libigl` must appear
**before** `references/BambuStudio/src/libigl` in the compiler's include search
path **and** contain the full tree so that every file in the chain exists at the
expected relative path.

Having only the 4 patched files here does not work: the first relative include
that points to a file missing from this directory causes a compile error (the
compiler does not fall back to the reference tree for relative includes once the
anchor file has been found).

---

## What are the 4 patches?

`Eigen::DynamicSparseMatrix` was removed in Eigen 3.4.  The upstream BambuStudio
libigl vendored copy still uses it in four files.  Each patch replaces it with
the equivalent modern `Eigen::SparseMatrix` API:

| File | Change |
|------|--------|
| `igl/slice.cpp` | `DynamicSparseMatrix` → `SparseMatrix`; `reserve(scalar)` → `reserve(VectorXi)` |
| `igl/slice_into.cpp` | `DynamicSparseMatrix` → `SparseMatrix` |
| `igl/cat.cpp` | `DynamicSparseMatrix` → `SparseMatrix` |
| `igl/diag.cpp` | `DynamicSparseMatrix` → `SparseMatrix`; `reserve(scalar)` → `reserve(VectorXi)` |

---

## Keeping this directory up to date

When the `references/BambuStudio` submodule is updated, re-sync this directory:

```sh
rsync -a --delete references/BambuStudio/src/libigl/ libslic3r/bambustudio/libigl/
```

Then re-apply the four patches (or check whether upstream has fixed the
`DynamicSparseMatrix` usage itself, in which case this entire directory can be
deleted and `cli/CMakeLists.txt` updated to point libigl at `REF_SRC` directly).

---

## Relationship to `cli/CMakeLists.txt`

```cmake
# include path order — VENDOR/libigl must precede REF_SRC/libigl
include_directories(
    "${VENDOR}/libigl"   # patched tree (this directory) — wins the anchor race
    "${REF_SRC}/libigl"  # fallback
    ...
)

# sub-library build — CMakeLists.txt is identical to reference
add_subdirectory("${VENDOR}/libigl" ...)
```
