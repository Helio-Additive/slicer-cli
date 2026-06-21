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

// ---------------------------------------------------------------------------
// M1 (bridges / wave_seeds): Z-preserving OPEN-PATH offset.
//
// Faithful replica of RegionExpansion.cpp:83-106
// `expolygons_to_zpaths_expanded_opened` (ClipperLib::ClipperOffset +
// ClipperZUtils::to_zpaths<true>). For each input expolygon contour, the
// contour is offset (outer contour by +expansion, holes by -expansion) with
// jtSquare/etClosedPolygon, then each resulting closed offset polygon is
// "opened" (first point repeated at the end) and every vertex is Z-tagged with
// the expolygon's running `base_idx`. `base_idx` increments once per expolygon.
//
// Input layout (closed contours, Z is irrelevant on input — only x,y read):
//   contour_xy:    int32 (x,y) pairs, flat across all contours of all expolygons
//   contour_lens:  int32, one per contour; point count of that contour
//   contour_per_ex:int32, one per expolygon; number of contours in that expolygon
//                  (contour[0] of each expolygon is the outer contour => +expansion)
//   num_ex:        number of expolygons
//   expansion:     offset distance in scaled coords (same units as x,y)
//   shortest_edge_length: ClipperOffset.ShortestEdgeLength (scaled)
//   base_idx_start: starting base_idx (C++ idx_src_end seed); the *next* free
//                   base_idx is returned via base_idx_out.
// Output (in CzZPaths): the opened, Z-tagged offset paths (Z = base_idx).
CzZPaths cz_offset_open(const int32_t *contour_xy, const int32_t *contour_lens,
                        const int32_t *contour_per_ex, int32_t num_ex,
                        double expansion, double shortest_edge_length,
                        int32_t base_idx_start, int32_t *base_idx_out);

// Result of cz_wave_seeds_clip: Z-tagged segments + provenance intersections.
typedef struct {
    // Z-tagged output segments (closed segments first, then open segments).
    int32_t *coords;        // 3 * total_points int32s (x,y,z triples)
    int32_t *path_lens;     // num_paths int32s; sum == total_points
    int32_t  num_paths;
    int32_t  total_points;
    int32_t  num_closed;    // first `num_closed` paths are closed; the rest open
    // Intersections table (ClipperZIntersectionVisitor): pair (first, second) of
    // the two distinct source Z indices for each recorded intersection point. A
    // negative output Z value -k refers to intersections[k-1].
    int32_t *intersections; // 2 * num_intersections int32s (first, second pairs)
    int32_t  num_intersections;
} CzWaveSeeds;

// M1 (bridges / wave_seeds): provenance Z-clip core.
//
// Faithful replica of RegionExpansion.cpp:302-327 (engine-equivalent, ClipperLib_Z
// instead of Clipper2Lib_Z): build a ClipperLib_Z::Clipper with the
// ClipperZIntersectionVisitor ZFillFunction (ClipperZUtils.hpp:125-160), add the
// boundary as CLOSED clip, the offset-opened src as OPEN subject, run
// Execute(ctIntersection, pftNonZero) and return the Z-tagged closed + open
// output segments plus the populated intersections table.
//
//   subj_xyz / subj_lens / subj_num: OPEN subject Z-paths (offset-opened src).
//   clip_xyz / clip_lens / clip_num: CLOSED clip Z-paths (boundary, pre-tagged
//                                    with Z = boundary index in 1..idx_boundary_end).
// Output: CzWaveSeeds (free via cz_free_wave_seeds).
CzWaveSeeds cz_wave_seeds_clip(const int32_t *subj_xyz, const int32_t *subj_lens,
                               int32_t subj_num, const int32_t *clip_xyz,
                               const int32_t *clip_lens, int32_t clip_num);

// Free a CzWaveSeeds returned by cz_wave_seeds_clip.
void cz_free_wave_seeds(CzWaveSeeds seeds);

#ifdef __cplusplus
}
#endif

#endif // CLIPPER_Z_SHIM_H
