// C ABI shim around BambuStudio's vendored ClipperLib / ClipperLib_Z.
//
// This header is C-callable (extern "C") and exposes:
//  - cz_version():            trivial smoke (M1) — returns CLIPPER_VERSION string.
//  - cz_union_point_count():  trivial smoke (M1) — a closed-path union via the
//                             non-Z ClipperLib, returning the resulting point count.
//                             De-risks linking the normal (non-XYZ) translation unit.
//  - cz_clip_extrusion():     the real primitive (M2) — a faithful replica of
//                             BambuStudio OverhangDetector.cpp `clip_extrusion`,
//                             using ClipperLib_Z with the baked-in ZFillFunction.
//                             Inputs/outputs are flat int32 arrays (x,y,z triples).
//
// All coordinates are ClipperLib's `cInt` which is int32_t here (CLIPPERLIB_INT32
// is defined in clipper.hpp). The Rust side scales libslic3r coords (i64) into the
// int32 range before calling and widens back afterwards.
#ifndef CLIPPER_Z_SHIM_H
#define CLIPPER_Z_SHIM_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// M1 smoke: returns a static, null-terminated version string ("6.2.6").
const char *cz_version(void);

// M1 smoke: union (ctUnion) of a single closed polygon with itself via the
// non-Z ClipperLib. `xy` holds `n` (x,y) int32 pairs (2*n int32s). Returns the
// total number of output points across all solution paths. De-risks the normal TU.
int32_t cz_union_point_count(const int32_t *xy, int32_t n);

// M2 clip_extrusion result. Flat layout:
//   coords:    int32 triples (x,y,z), `total_points` of them (3*total_points int32s)
//   path_lens: int32, one per output path; sum == total_points
//   num_paths: number of output paths
// Ownership: allocated by cz_clip_extrusion, freed by cz_free_zpaths.
typedef struct {
    int32_t *coords;     // 3 * total_points int32s (x,y,z, x,y,z, ...)
    int32_t *path_lens;  // num_paths int32s
    int32_t  num_paths;
    int32_t  total_points;
} CzZPaths;

// M2: faithful replica of OverhangDetector.cpp clip_extrusion.
//   subject_xyz / subject_n: the OPEN subject path (n x,y,z triples).
//   clip_xyz / clip_lens / clip_num: the CLOSED clip paths (flat triples + per-path lengths).
//   clip_type: 0=ctIntersection, 1=ctUnion, 2=ctDifference, 3=ctXor (ClipperLib_Z::ClipType).
// Returns the clipped OPEN paths with interpolated Z (extrusion width) tags.
CzZPaths cz_clip_extrusion(const int32_t *subject_xyz, int32_t subject_n,
                           const int32_t *clip_xyz, const int32_t *clip_lens, int32_t clip_num,
                           int32_t clip_type);

// Free a CzZPaths returned by cz_clip_extrusion.
void cz_free_zpaths(CzZPaths paths);

// ASSESS-ONLY (offset-vtx-assess branch): faithful non-Z ClipperOffset mirroring
// libslic3r raw_offset() (ClipperUtils.cpp:272). jtMiter, MiterLimit=3.0,
// ShortestEdgeLength=|delta|*0.005, etClosedPolygon. Single closed input path
// (`xy` = n int32 x,y pairs). Reorients like Execute (signum reversed for CW).
// Returns the offset output paths flat (CzZPaths reused; z is always 0). Used by
// the vertex-density comparison harness only; NOT a production primitive.
//   join_type: 0=jtMiter, 1=jtRound, 2=jtSquare. delta is in input integer units.
CzZPaths cz_offset_closed(const int32_t *xy, int32_t n, double delta, int32_t join_type);

#ifdef __cplusplus
}
#endif

#endif // CLIPPER_Z_SHIM_H
