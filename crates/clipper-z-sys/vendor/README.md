# Vendored ClipperLib / ClipperLib_Z (BambuStudio)

These files are copied verbatim from the BambuStudio source tree in this repo:

| Vendored file | Source path (relative to repo root) |
|---------------|--------------------------------------|
| `clipper.cpp` | `libslic3r/bambustudio/references/BambuStudio/src/clipper/clipper.cpp` |
| `clipper.hpp` | `libslic3r/bambustudio/references/BambuStudio/src/clipper/clipper.hpp` |
| `clipper_z.hpp` | `libslic3r/bambustudio/references/BambuStudio/src/clipper/clipper_z.hpp` |
| `Int128.hpp` | `libslic3r/bambustudio/references/BambuStudio/src/libslic3r/Int128.hpp` |

## Local modification

`clipper.cpp` line 51 was changed from

```cpp
#include <libslic3r/Int128.hpp>
```

to

```cpp
#include "Int128.hpp"
```

so the include closure is satisfied entirely within this `vendor/` directory
(no `-I` into the BambuStudio source tree is required for the clipper TUs).

## Compilation

`build.rs` compiles `clipper.cpp` twice:

* once normally → namespace `ClipperLib` (2D `IntPoint`)
* once with `-DCLIPPERLIB_USE_XYZ` → namespace `ClipperLib_Z` (3D `IntPoint`, Z tags)

`clipper.hpp` defines `CLIPPERLIB_INT32`, so the coordinate type `cInt` and the
Z tag are `int32_t`. `IntPoint` is an `Eigen::Matrix<cInt, 2 or 3, 1, DontAlign>`;
Eigen is header-only and located at build time via `pkg-config eigen3` (provided
by the devbox shell), with a fallback to the BambuStudio-vendored Eigen under
`references/.../src/eigen`.

The C ABI shim (`../shim/clipper_z_shim.{h,cpp}`) wraps these into `extern "C"`
functions; the real primitive `cz_clip_extrusion` is a faithful replica of
`OverhangDetector.cpp` `clip_extrusion`.
