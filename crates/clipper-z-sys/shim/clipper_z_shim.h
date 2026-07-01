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

// Faithful replica of libslic3r ClipperUtils.cpp `union_ex(const Polygons&,
// PolyFillType)` (ClipperUtils.cpp:813-814) = PolyTreeToExPolygons(
// clipper_do_polytree(ctUnion, PolygonsProvider(subject), Empty, fill_type)).
// This is the union behind make_expolygons (TriangleMeshSlicer.cpp:1819-1823,
// the slice-stage F1 site). Runs the non-Z ClipperLib union over the CLOSED
// input paths, builds the PolyTree, and flattens it via the EXACT
// PolyTreeToExPolygons nesting (contour, its holes, then contours nested in
// holes appended). Coordinates stay native i32 (no float / scale-1000
// re-quantization), making the slice coords byte-exact vs C++.
//
// Layout: `xy` = flat (x,y) int32 pairs; `lens` = per-path point counts;
//   `num` = path count. `fill_type`: 0=EvenOdd, 1=NonZero, 2=Positive, 3=Negative.
// OUTPUT encodes the ExPolygon grouping in each point's Z: a CONTOUR path's
// points carry z=0 (starts a new ExPolygon); a HOLE path's points carry z=1
// (attaches to the most recent contour). Paths are emitted in
// PolyTreeToExPolygons order (each contour immediately followed by its holes).
// Free via cz_free_zpaths.
CzZPaths cz_union_ex(const int32_t *xy, const int32_t *lens, int32_t num,
                     int32_t fill_type);

// Faithful replica of libslic3r ClipperUtils.cpp `offset2_ex(ExPolygons, delta1,
// delta2)` (ClipperUtils.cpp:581) = the post-union morphological close in
// make_expolygons (TriangleMeshSlicer.cpp:1820). Input is the cz_union_ex output
// layout: flat (x,y) int32 pairs, per-path point counts `lens`, per-path `is_hole`
// (0=contour starts a new ExPolygon, 1=hole attaches to current), `num` paths.
// `delta1`/`delta2` are SCALED (1e5) — make_expolygons passes delta1=+scale(r),
// delta2=-scale(r). join_type: 0=jtMiter,1=jtRound,2=jtSquare; miter_limit=3.0.
// OUTPUT uses the same grouped z-encoding as cz_union_ex. Free via cz_free_zpaths.
CzZPaths cz_offset2_ex(const int32_t *xy, const int32_t *lens, const int32_t *is_hole,
                       int32_t num, double delta1, double delta2,
                       int32_t join_type, double miter_limit);

// Faithful ClipperLib::SimplifyPolygons (clipper.hpp:559): ctUnion with
// StrictlySimple(true). Used by ExPolygon::simplify_p's post-DP step
// (ClipperUtils simplify_polygons). Returns flat Paths (z=0); caller re-unions.
// `fill_type`: 0=EvenOdd,1=NonZero,2=Positive,3=Negative.
CzZPaths cz_simplify_polygons(const int32_t *xy, const int32_t *lens, int32_t num,
                              int32_t fill_type);

// Faithful replica of libslic3r ClipperUtils.cpp `offset_expolygon_inner`
// (ClipperUtils.cpp:437-506): offset a SINGLE ExPolygon (contour + holes) by
// `delta` (in scaled integer units) using the vertex-exact ClipperOffset
// (jtMiter, MiterLimit=miter_limit; jtRound uses ArcTolerance=miter_limit;
// ShortestEdgeLength=|delta|*ClipperOffsetShortestEdgeFactor=0.005). For a
// negative offset the offsetted holes are subtracted from the offsetted contour
// (ctDifference, pftNonZero); for a positive offset the reversed holes are just
// appended. The result is the per-ExPolygon offset Paths (NOT unioned across
// ExPolygons — the caller unions, matching expolygons_offset). z is always 0.
//
// Layout: `contour_xy` = `contour_n` int32 (x,y) pairs. `holes_xy`/`hole_lens`/
// `hole_num` describe the holes (flat (x,y) pairs + per-hole point counts).
//   join_type: 0=jtMiter, 1=jtRound, 2=jtSquare. delta is in input integer units.
// Returns the offset paths flat (CzZPaths reused; z always 0). Free via
// cz_free_zpaths. This is the ONLY vertex-generating primitive routed away from
// geo-clipper (the perimeter inner-wall offset density fix); booleans and the
// final path->ExPolygon union stay on geo-clipper.
CzZPaths cz_offset_expolygon(const int32_t *contour_xy, int32_t contour_n,
                             const int32_t *holes_xy, const int32_t *hole_lens,
                             int32_t hole_num, double delta, int32_t join_type,
                             double miter_limit);

// Faithful replica of libslic3r ClipperUtils.cpp `_clipper` /
// `clipper_do<ClipperLib::Paths>(ctDifference, subject, clip, pftNonZero)`
// (ClipperUtils.cpp:309-322, 669-692): a closed-path boolean DIFFERENCE
// (subject - clip) over the non-Z ClipperLib. Both subject and clip are sets of
// CLOSED paths (each ExPolygon contributes its contour + holes as separate paths,
// in their natural orientation — exactly what ClipperUtils::ExPolygonsProvider
// emits). The two pass `pftNonZero / pftNonZero` fill rules, matching
// `_clipper` (ClipperUtils.cpp:672). NO safety offset is applied (the gap-fill
// `diff_ex` call sites use ApplySafetyOffset::No).
//
// Layout (both subject and clip): flat (x,y) int32 pairs + per-path point counts.
//   subject_xy / subject_lens / subject_num
//   clip_xy    / clip_lens    / clip_num
// Returns the raw difference output as flat CLOSED paths (z always 0); the caller
// re-unions them into ExPolygons (NonZero union + PolyTree nesting), which makes
// the whole thing byte-faithful to `diff_ex` =
// PolyTreeToExPolygons(clipper_do_polytree(ctDifference, ..., pftNonZero)).
// Free via cz_free_zpaths.
CzZPaths cz_difference_closed(const int32_t *subject_xy, const int32_t *subject_lens,
                              int32_t subject_num, const int32_t *clip_xy,
                              const int32_t *clip_lens, int32_t clip_num);

// Faithful replica of libslic3r FillFloatingConcentric.cpp `detect_floating_line`
// (FillFloatingConcentric.cpp:431-475): the Z-aware open-path clip used to mark
// which segments of a thick polyline fall in the floating (unsupported) area.
// Runs the ClipperLib_Z Clipper twice on the SAME inputs — ctIntersection and
// ctDifference — under the detect_floating_line ZFillFunction (which tags each
// intersection point with a NEGATIVE hash of the four edge endpoint z-indices,
// or, for a subject-self-intersection, the common subject z). Both passes use
// pftNonZero.
//
//   subject_xyz / subject_n : the OPEN subject path (n x,y,z triples; z is the
//                             polyline vertex index 0..subject_idx_range-1).
//   clip_xyz / clip_lens / clip_num : the CLOSED clip paths (floating-area
//                             polygons; z is a per-vertex index >= subject_idx_range).
//   subject_idx_range       : the z-index boundary (== subject point count) the
//                             ZFillFunction uses to tell subject from clip.
//   out_num_diff_paths      : [out] number of ctDifference paths at the FRONT of
//                             the returned path list (the rest are ctIntersection,
//                             i.e. the FLOATING paths). Mirrors C++ `to_merge =
//                             diff_out ++ intersect_out` with floating_flags set
//                             true for the intersect tail.
// Returns [diff_paths..., intersect_paths...] as flat OPEN ZPaths (x,y,z), z
// carrying the source/hash tags merge_lines() consumes. Free via cz_free_zpaths.
CzZPaths cz_detect_floating(const int32_t *subject_xyz, int32_t subject_n,
                            const int32_t *clip_xyz, const int32_t *clip_lens,
                            int32_t clip_num, int32_t subject_idx_range,
                            int32_t *out_num_diff_paths);

#ifdef __cplusplus
}
#endif

#endif // CLIPPER_Z_SHIM_H
