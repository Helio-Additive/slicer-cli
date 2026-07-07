// C-ABI shim around the vendored Clipper2 (v1.5.4) built with -DUSINGZ, namespace-
// isolated as `Clipper2ZSys` (see build.rs) so it never collides with
// clipper2c-sys's non-Z `Clipper2Lib`.
//
// Exposes exactly the two Clipper2-Z operations BambuStudio's RegionExpansion.cpp
// `wave_seeds` (RegionExpansion.cpp:278-389) depends on:
//   - cz2_offset_z():        ClipperOffset (JoinType::Square, EndType::Polygon),
//                            Z carried through, used by
//                            expolygons_to_zpaths64_expanded_opened.
//   - cz2_intersect_open_z(): Clipper64 + SetZCallback (the
//                            Clipper2ZIntersectionVisitor logic) + AddClip(closed)
//                            + AddOpenSubject(open) + Execute(Intersection, NonZero)
//                            returning BOTH closed and open Z-paths plus the
//                            recorded Intersections table.
//
// All coordinates are Clipper2 `int64_t` (x, y, z). The Rust side passes libslic3r
// i64 coords straight through (no narrowing — Clipper2 is 64-bit, unlike the int32
// ClipperLib_Z shim).
#ifndef CLIPPER2_Z_SHIM_H
#define CLIPPER2_Z_SHIM_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Flat Z-paths result: `coords` holds `total_points` (x,y,z) int64 triples
// (3*total_points int64s); `path_lens` has `num_paths` entries summing to
// total_points. Allocated by the producing fn, freed by cz2_free_zpaths.
typedef struct {
    int64_t *coords;     // 3 * total_points int64s (x,y,z, ...)
    int32_t *path_lens;  // num_paths int32s
    int32_t  num_paths;
    int32_t  total_points;
} Cz2ZPaths;

// ClipperOffset of a SINGLE closed contour (Clipper2 `ClipperOffset`,
// JoinType::Square, EndType::Polygon) by `delta`, preserving the input Z on every
// output vertex. `contour_xyz` = `n` (x,y,z) int64 triples. `delta` is the offset
// distance in scaled integer units (sign chosen by caller: +expansion for the
// outer contour, -expansion for holes — matching
// expolygons_to_zpaths64_expanded_opened). Returns the offset path(s). Free via
// cz2_free_zpaths.
Cz2ZPaths cz2_offset_z(const int64_t *contour_xyz, int32_t n, double delta);

// Result of cz2_intersect_open_z: the clipped segments (closed first, then open)
// plus the Intersections table the Z-callback recorded. `is_a`/`is_b` are the two
// source-z values of each recorded intersection (parallel arrays of `num_is`).
// The negative Z tags on `segs` index this table as `-z - 1`.
typedef struct {
    Cz2ZPaths segs;        // closed_segs ++ open_segs
    int32_t   num_closed;  // first `num_closed` paths in `segs` are the closed ones
    int64_t  *is_a;        // num_is intersection lower-z values
    int64_t  *is_b;        // num_is intersection upper-z values
    int32_t   num_is;
} Cz2WaveClip;

// Faithful replica of the wave_seeds Clipper64 boolean (RegionExpansion.cpp:301-322):
// Clipper64 with SetZCallback(Clipper2ZIntersectionVisitor) +
// AddClip(boundary_closed) + AddOpenSubject(src_open) +
// Execute(ClipType::Intersection, FillRule::NonZero, closed_segs, open_segs).
//   src_xyz / src_lens / src_num:        OPEN subject Z-paths (flat triples + per-path lens).
//   clip_xyz / clip_lens / clip_num:     CLOSED clip (boundary) Z-paths.
// Returns the closed+open Z-segments and the Intersections table. Free via
// cz2_free_wave_clip.
Cz2WaveClip cz2_intersect_open_z(const int64_t *src_xyz, const int32_t *src_lens,
                                 int32_t src_num, const int64_t *clip_xyz,
                                 const int32_t *clip_lens, int32_t clip_num);

void cz2_free_zpaths(Cz2ZPaths paths);
void cz2_free_wave_clip(Cz2WaveClip wc);

// Smoke: returns the vendored Clipper2 version string ("1.5.4") and proves the
// USINGZ TU links.
const char *cz2_version(void);

#ifdef __cplusplus
}
#endif

#endif // CLIPPER2_Z_SHIM_H
