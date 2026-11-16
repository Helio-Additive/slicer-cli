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
// PolyTreeToExPolygons(clipper_union<PolyTree>(paths, pftNonZero)): one union
// into a PolyTree, grouped like cz_union_ex (ClipperUtils.cpp:339-350).
CzZPaths cz_union_polytree(const int32_t *xy, const int32_t *lens, int32_t num);

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

// R803: faithful `offset(const Polygons&, float delta)` (ClipperUtils.cpp:413)
// — raw_offset + clipper_union<Paths> / shrink_paths<Paths>; flat Paths in
// Clipper's BuildResult order (ring start preserved). join_type 0=Miter,1=Round,2=Square.
CzZPaths cz_offset_paths(const int32_t *xy, const int32_t *lens, int32_t num,
                         double delta, int32_t join_type, double miter_limit);

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
// Faithful replica of RegionExpansion.cpp propagate_wave_from_boundary
// (+ wavefront_initial/step/clip): the ClipperLib wavefront propagation for one
// (boundary, src) seed group. seed_* = open seed polylines; bnd_* = boundary
// ExPolygon as closed paths (contour CCW + holes CW). Returns the expanded closed
// polygons (z=0). Free via cz_free_zpaths.
CzZPaths cz_propagate_wave(const int32_t *seed_xy, const int32_t *seed_lens, int32_t seed_num,
                           const int32_t *bnd_xy, const int32_t *bnd_lens, int32_t bnd_num,
                           double initial_step, double other_step, int32_t num_other_steps,
                           double arc_tolerance, double shortest_edge_length);

// Faithful union_safety_offset_ex(Polygons): subject safety-offset (+10u raw)
// then the two-pass NonZero union. Grouped ExPolygon output like cz_union_ex.
CzZPaths cz_union_ex_safety(const int32_t *xy, const int32_t *lens, int32_t num);

// Faithful Slic3r::offset(Polyline, delta): jtSquare/etOpenButt ClipperOffset +
// NonZero union. xy = n open-path (x,y) pairs; delta scaled. Free via cz_free_zpaths.
CzZPaths cz_offset_polyline(const int32_t *xy, int32_t n, double delta, double miter_limit);

CzZPaths cz_difference_closed(const int32_t *subject_xy, const int32_t *subject_lens,
                              int32_t subject_num, const int32_t *clip_xy,
                              const int32_t *clip_lens, int32_t clip_num);

// diff_ex(subject, clip, ApplySafetyOffset::Yes): applies safety_offset (raw
// +10-unit ClipperOffset, jtMiter/ML3, orientation-aware) to the clip paths
// before the ctDifference. Same marshalling as cz_difference_closed.
CzZPaths cz_difference_closed_safety(const int32_t *subject_xy, const int32_t *subject_lens,
                                     int32_t subject_num, const int32_t *clip_xy,
                                     const int32_t *clip_lens, int32_t clip_num);

// Faithful replica of libslic3r ClipperUtils.cpp
// `intersection_ex(subject, clip)` = `_clipper_ex(ctIntersection, ...,
// ApplySafetyOffset::No)` (ClipperUtils.cpp:802) — a closed-path boolean
// INTERSECTION (subject ∩ clip) over the non-Z ClipperLib, pftNonZero/pftNonZero.
// Same marshalling + caller re-union as cz_difference_closed. This is the FAITHFUL
// replacement for the R118 `A - (A - B)` double-difference intersection, which
// added a near-collinear vertex on some geometries (a second ctDifference pass
// re-processed the intermediate region); a single ctIntersection matches native's
// PolyTreeToExPolygons(clipper_do_polytree(ctIntersection,...)) byte-for-byte.
CzZPaths cz_intersection_closed(const int32_t *subject_xy, const int32_t *subject_lens,
                                int32_t subject_num, const int32_t *clip_xy,
                                const int32_t *clip_lens, int32_t clip_num);

// R792 — _clipper_pl_open(ctIntersection): open subject polylines clipped by
// closed polygons; PolyTree preorder output (ClipperUtils.cpp:841/213).
CzZPaths cz_intersection_pl(const int32_t *subject_xy, const int32_t *subject_lens,
                            int32_t subject_num, const int32_t *clip_xy,
                            const int32_t *clip_lens, int32_t clip_num);

// _clipper_pl_open(ctDifference): the parts of open subject polylines outside
// closed polygons; PolyTree preorder output (ClipperUtils.cpp:841, 909-920).
CzZPaths cz_diff_pl(const int32_t *subject_xy, const int32_t *subject_lens,
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

// R885 — reference exports for the Rust port of clipper.cpp; called by tests only.
double  cz_ref_area(const int32_t *xy, int32_t n);
int32_t cz_ref_point_in_polygon(int32_t px, int32_t py, const int32_t *xy, int32_t n);
int32_t cz_ref_clean_polygon(const int32_t *xy, int32_t n, double distance, int32_t *out_xy);

// Reference export for the Rust port of the Clipper class; called by tests
// only. ClipperLib::Clipper (2D) with every subject and clip added closed through
// AddPath; returns the solution of Execute(clip_type, solution, subj_fill, clip_fill)
// as produced (z always 0) and writes what Execute returned to *succeeded. clip_type:
// 0=ctIntersection, 1=ctUnion, 2=ctDifference, 3=ctXor; fills: 0=pftEvenOdd,
// 1=pftNonZero, 2=pftPositive, 3=pftNegative. Free via cz_free_zpaths.
CzZPaths cz_ref_execute(const int32_t *subj_xy, const int32_t *subj_lens, int32_t subj_num,
                        const int32_t *clip_xy, const int32_t *clip_lens, int32_t clip_num,
                        int32_t clip_type, int32_t subj_fill, int32_t clip_fill,
                        int32_t strictly_simple, int32_t preserve_collinear,
                        int32_t reverse_solution, int32_t *succeeded);

// Reference exports for the Rust port of PolyNode / PolyTree, the PolyTree Execute
// of Clipper, ClipperOffset and SimplifyPolygon(s); called by tests only.
//
// The encoding of a PolyTree: the contours of its nodes as a CzZPaths (z always 0)
// in pre-order (a node, then its children in the order of Childs, each with its
// own descendants; the tree itself is not listed), every contour with the points
// and the start vertex the library produced. For node i of that listing
// node_info[4*i .. 4*i+4] receives: the index in the listing of its Parent (-1 when
// the Parent is the tree itself), IsHole(), IsOpen(), ChildCount(). At most
// node_info_cap nodes are written; *num_nodes receives the number of nodes listed,
// *total what PolyTree::Total() returns.
//
// cz_ref_execute_polytree: Clipper::Execute(clip_type, PolyTree&, subj_fill,
// clip_fill). Subject i is closed when subj_closed[i] != 0, open otherwise; clips
// are closed. add_paths == 0: every path through AddPath, subjects first;
// otherwise three AddPaths calls (closed subjects, open subjects, clips). output:
// 0 = the tree in the encoding above, 1 = PolyTreeToPaths, 2 =
// ClosedPathsFromPolyTree, 3 = OpenPathsFromPolyTree (node_info not written for
// 1-3). Free via cz_free_zpaths.
CzZPaths cz_ref_execute_polytree(const int32_t *subj_xy, const int32_t *subj_lens,
                                 const int32_t *subj_closed, int32_t subj_num,
                                 const int32_t *clip_xy, const int32_t *clip_lens, int32_t clip_num,
                                 int32_t clip_type, int32_t subj_fill, int32_t clip_fill,
                                 int32_t strictly_simple, int32_t preserve_collinear,
                                 int32_t reverse_solution, int32_t add_paths, int32_t output,
                                 int32_t *node_info, int32_t node_info_cap, int32_t *num_nodes,
                                 int32_t *total, int32_t *succeeded);

// cz_ref_offset: ClipperOffset(miter_limit, arc_tolerance, shortest_edge_length);
// path i through AddPath(path, join_types[i], end_types[i]) (join: 0=jtSquare,
// 1=jtRound, 2=jtMiter; end: 0=etClosedPolygon, 1=etClosedLine, 2=etOpenButt,
// 3=etOpenSquare, 4=etOpenRound); then Execute(solution, delta) into a Paths
// (use_polytree == 0, returned as produced) or into a PolyTree (returned in the
// encoding above). Free via cz_free_zpaths.
CzZPaths cz_ref_offset(const int32_t *xy, const int32_t *lens, const int32_t *join_types,
                       const int32_t *end_types, int32_t num, double miter_limit,
                       double arc_tolerance, double shortest_edge_length, double delta,
                       int32_t use_polytree, int32_t *node_info, int32_t node_info_cap,
                       int32_t *num_nodes, int32_t *total);

// cz_ref_simplify_polygons: SimplifyPolygon(first path, fill_type) when which == 0,
// SimplifyPolygons(paths, fill_type) otherwise. Free via cz_free_zpaths.
CzZPaths cz_ref_simplify_polygons(const int32_t *xy, const int32_t *lens, int32_t num,
                                  int32_t fill_type, int32_t which);

// Reference exports for the Rust port of the CLIPPERLIB_USE_XYZ build of clipper.cpp
// (namespace ClipperLib_Z); called by tests only. Paths are flat (x, y, z) triples, in
// and out.
//
// cz_ref_execute_z: ClipperLib_Z::Clipper; subjects, clips, options and add_paths as
// in cz_ref_execute_polytree. z_fill: 0 = no ZFillFunction, 1 = the callback of the
// reference (cz_ref_z_fill in clipper_z_shim.cpp, which documents its formula).
// output: 0 = Execute into a PolyTree, in the encoding above with every point's Z;
// 1 = the same, then PolyTreeToPaths; 2 = Execute into a Paths (closed subjects only).
// The library of this build dies on some inputs (clipper_z_shim.cpp says which): the
// call runs under a guard, and *died receives 0 or the signal that ended it (SIGSEGV,
// SIGBUS, or SIGALRM after one second), in which case nothing is returned. Free via
// cz_free_zpaths.
CzZPaths cz_ref_execute_z(const int32_t *subj_xyz, const int32_t *subj_lens,
                          const int32_t *subj_closed, int32_t subj_num,
                          const int32_t *clip_xyz, const int32_t *clip_lens, int32_t clip_num,
                          int32_t clip_type, int32_t subj_fill, int32_t clip_fill,
                          int32_t strictly_simple, int32_t preserve_collinear,
                          int32_t reverse_solution, int32_t add_paths, int32_t z_fill,
                          int32_t output, int32_t *node_info, int32_t node_info_cap,
                          int32_t *num_nodes, int32_t *total, int32_t *succeeded, int32_t *died);

// cz_clip_extrusion and cz_detect_floating under the guard; *died as above.
CzZPaths cz_ref_clip_extrusion_guarded(const int32_t *subject_xyz, int32_t subject_n,
                                       const int32_t *clip_xyz, const int32_t *clip_lens,
                                       int32_t clip_num, int32_t clip_type, int32_t *died);
CzZPaths cz_ref_detect_floating_guarded(const int32_t *subject_xyz, int32_t subject_n,
                                        const int32_t *clip_xyz, const int32_t *clip_lens,
                                        int32_t clip_num, int32_t subject_idx_range,
                                        int32_t *out_num_diff_paths, int32_t *died);

// cz_ref_offset_z: ClipperLib_Z::ClipperOffset, as cz_ref_offset, with a Clear()
// after the constructor, which leaves m_lowest uninitialized in this build. Free
// via cz_free_zpaths.
CzZPaths cz_ref_offset_z(const int32_t *xyz, const int32_t *lens, const int32_t *join_types,
                         const int32_t *end_types, int32_t num, double miter_limit,
                         double arc_tolerance, double shortest_edge_length, double delta,
                         int32_t use_polytree, int32_t *node_info, int32_t node_info_cap,
                         int32_t *num_nodes, int32_t *total);

// R910 — reference exports for the GEO_CLIPPER_CLIB arms of clipper_utils.rs; called
// by tests only. Each runs the pin's ClipperUtils.cpp function named (copied into
// clipper_z_shim.cpp, namespace cu_pin), with `delta` handed to its `float`
// parameter as the C++ callers hand a double.
//
// ExPolygons in: flat (x,y) int32 pairs, per-path point counts and per-path is_hole
// (0 = a contour, which starts an expolygon; otherwise a hole of the expolygon before
// it). ExPolygons out: grouped like cz_union_ex (z = 0 on a contour, z = 1 on a hole).
// join_type: 0=jtMiter, 1=jtRound, 2=jtSquare. end_type: 0=etClosedPolygon,
// 1=etClosedLine, 2=etOpenButt, 3=etOpenSquare, 4=etOpenRound. Free via cz_free_zpaths.
//
// offset_ex(const Polygons&, float delta, joinType, miterLimit) (ClipperUtils.cpp:415-416).
CzZPaths cz_ref_cu_offset_ex_polygons(const int32_t *xy, const int32_t *lens, int32_t num,
                                      double delta, int32_t join_type, double miter_limit);
// which == 0: offset_ex(const ExPolygon&, ...) (ClipperUtils.cpp:567-569) of the first
// expolygon; otherwise offset_ex(const ExPolygons&, ...) (ClipperUtils.cpp:570-571).
CzZPaths cz_ref_cu_offset_ex_expolygons(const int32_t *xy, const int32_t *lens, const int32_t *is_hole,
                                        int32_t num, double delta, int32_t join_type,
                                        double miter_limit, int32_t which);
// offset(const Polyline&, float delta, joinType, miterLimit, end_type)
// (ClipperUtils.cpp:418-419); flat paths out (z = 0).
CzZPaths cz_ref_cu_offset_polyline(const int32_t *xy, int32_t n, double delta, int32_t join_type,
                                   double miter_limit, int32_t end_type);
// op 0: intersection_ex(const ExPolygons&, const ExPolygons&) (ClipperUtils.cpp:802-803);
// 1: union_ex(const ExPolygons&, const ExPolygons&) (ClipperUtils.cpp:824-831);
// 2: diff_ex(const ExPolygons&, const ExPolygons&) (ClipperUtils.cpp:755-756);
// otherwise xor_ex(const ExPolygons&, const ExPolygons&) (ClipperUtils.cpp:836-838);
// all with ApplySafetyOffset::No.
CzZPaths cz_ref_cu_boolean_ex(const int32_t *subj_xy, const int32_t *subj_lens, const int32_t *subj_is_hole,
                              int32_t subj_num, const int32_t *clip_xy, const int32_t *clip_lens,
                              const int32_t *clip_is_hole, int32_t clip_num, int32_t op);

#ifdef __cplusplus
}
#endif

#endif // CLIPPER_Z_SHIM_H
