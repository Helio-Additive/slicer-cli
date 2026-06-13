//! Clipper2 polygon boolean operations and utilities.
//!
//! Faithful 1:1 port of `Clipper2Utils.cpp` / `Clipper2Utils.hpp` from BambuStudio.
//!
//! C++ Reference: `src/libslic3r/Clipper2Utils.cpp`, `src/libslic3r/Clipper2Utils.hpp`
//!
//! The C++ code works directly on `Clipper2Lib::Paths64` (raw 64-bit integer
//! coordinates, i.e. Slic3r `coord_t`). To preserve byte-exact behaviour we use
//! the `clipper2` crate with the `One` point scaler (multiplier 1.0) so that the
//! Slic3r `coord_t` (i64) coordinates pass through Clipper2 unscaled — exactly as
//! in C++, with no intermediate float/mm round-tripping.
//!
//! NATIVE DEPENDENCY NOTE: the C++ Clipper2 library is reached here via the raw
//! `clipper2c-sys` FFI. That native (non-wasm-safe) library is already linked into
//! this crate transitively through the `clipper2` crate (used by
//! `clipper_utils`/`clipper2_z_utils`), so no new native backend is introduced —
//! we only expose the lower-level entry points the safe `clipper2` wrapper hides:
//!   * `Clipper64::Execute(closed, open)` — the safe wrapper discards `solution_open`,
//!     which Clipper2Utils.cpp's `_clipper2_pl_open` relies on for open-subject
//!     polyline clipping.
//!   * `PolyTree64` / `PolyPath64` — the safe wrapper exposes no tree, but
//!     `PolyTreeToExPolygons` / `SimplifyPolyTree` / `PolyTreeToPaths64` require the
//!     real Clipper2 nesting hierarchy for byte-exact ExPolygon reconstruction.
//!   * `ClipperOffset` with the same defaults (`miter_limit=2.0, arc_tolerance=0.0`)
//!     the C++ `ClipperOffset offsetter;` default constructor uses.

use crate::geometry::{cross2f, ExPolygon, ExPolygons, Point, PointF, Polygon, Polygons, Polyline};
use crate::libslic3r::SCALED_EPSILON;

use clipper2c_sys::{
    clipper_allocate, clipper_clipper64, clipper_clipper64_add_clip,
    clipper_clipper64_add_open_subject, clipper_clipper64_add_subject, clipper_clipper64_execute,
    clipper_clipper64_execute_tree_with_open, clipper_clipper64_set_reverse_solution,
    clipper_clipper64_size, clipper_clipperoffset,
    clipper_clipperoffset_add_paths64, clipper_clipperoffset_execute, clipper_clipperoffset_size,
    clipper_delete_clipper64, clipper_delete_clipperoffset, clipper_delete_path64,
    clipper_delete_paths64, clipper_delete_polytree64, clipper_path64_of_points,
    clipper_path64_size, clipper_paths64_get_point, clipper_paths64_length,
    clipper_paths64_of_paths, clipper_paths64_path_length, clipper_paths64_size,
    clipper_polytree64, clipper_polytree64_count, clipper_polytree64_get_child,
    clipper_polytree64_polygon, clipper_polytree64_size,
    ClipperClipType_DIFFERENCE, ClipperClipType_INTERSECTION, ClipperClipType_UNION,
    ClipperEndType_POLYGON_END, ClipperFillRule, ClipperFillRule_NEGATIVE, ClipperFillRule_NON_ZERO,
    ClipperFillRule_POSITIVE, ClipperJoinType_ROUND_JOIN,
    ClipperPath64, ClipperPaths64, ClipperPoint64, ClipperPolyTree64,
};

// ============================================================================
// Type Aliases (matching C++ Clipper2 usage)
//
// Clipper2Lib::Point64 -> (i64, i64)
// Clipper2Lib::Path64  -> Vec<(i64, i64)>   (a single contour)
// Clipper2Lib::Paths64 -> Vec<Vec<(i64, i64)>>
// ============================================================================

/// Clipper2 point type (64-bit integer coordinates).
/// Clipper2Utils.cpp (implicit from Clipper2Lib::Point64)
pub type Point64 = (i64, i64);

/// Clipper2 path type (single contour).
/// Clipper2Utils.cpp (implicit from Clipper2Lib::Path64)
pub type Path64 = Vec<Point64>;

/// Clipper2 paths type (multiple contours).
/// Clipper2Utils.cpp (implicit from Clipper2Lib::Paths64)
pub type Paths64 = Vec<Path64>;

// ============================================================================
// Raw Clipper2 (clipper2c-sys) FFI bridge.
//
// The Slic3r `coord_t` is `i64`, identical to Clipper2's internal `Point64`
// coordinate type, so paths pass through unscaled (1:1) exactly as in C++ — no
// intermediate float/mm round-tripping. All native objects are allocated with
// `clipper_allocate` and freed with the matching `clipper_delete_*`.
// ============================================================================

/// Build a native `ClipperPaths64` from our `Paths64` (`Vec<Vec<(i64,i64)>>`).
/// Caller owns the returned pointer and must `clipper_delete_paths64` it.
unsafe fn paths64_to_native(paths: &Paths64) -> *mut ClipperPaths64 {
    // Build each native ClipperPath64 from the i64 points, collect, then wrap.
    let mut native_paths: Vec<*mut ClipperPath64> = Vec::with_capacity(paths.len());
    for path in paths {
        let mut pts: Vec<ClipperPoint64> = path
            .iter()
            .map(|&(x, y)| ClipperPoint64 { x, y })
            .collect();
        let mem = clipper_allocate(clipper_path64_size());
        let native = clipper_path64_of_points(mem, pts.as_mut_ptr(), pts.len());
        native_paths.push(native);
    }
    let mem = clipper_allocate(clipper_paths64_size());
    let result = clipper_paths64_of_paths(mem, native_paths.as_mut_ptr(), native_paths.len());
    for p in native_paths {
        clipper_delete_path64(p);
    }
    result
}

/// Read a native `ClipperPaths64` back into our `Paths64`.
/// Does not take ownership of the pointer.
unsafe fn native_to_paths64(ptr: *mut ClipperPaths64) -> Paths64 {
    let len: i32 = clipper_paths64_length(ptr) as i32;
    let mut out: Paths64 = Vec::with_capacity(len.max(0) as usize);
    for i in 0..len {
        let point_len: i32 = clipper_paths64_path_length(ptr, i) as i32;
        let mut path: Path64 = Vec::with_capacity(point_len.max(0) as usize);
        for j in 0..point_len {
            let pt = clipper_paths64_get_point(ptr, i, j);
            path.push((pt.x, pt.y));
        }
        out.push(path);
    }
    out
}

// ============================================================================
// Conversion functions (faithful to the C++ free functions)
// ============================================================================

/// Clipper2Utils.cpp:8
/// C++: Slic3r::Polylines Paths64_to_polylines(const Clipper2Lib::Paths64& in)
/// C++: {
/// C++:     Slic3r::Polylines out;
/// C++:     out.reserve(in.size());
/// C++:     for (const Clipper2Lib::Path64& path64 : in) {
/// C++:         Slic3r::Points points;
/// C++:         points.reserve(path64.size());
/// C++:         for (const Clipper2Lib::Point64& point64 : path64)
/// C++:             points.emplace_back(std::move(Slic3r::Point(point64.x, point64.y)));
/// C++:         out.emplace_back(std::move(Slic3r::Polyline(points)));
/// C++:     }
/// C++:     return out;
/// C++: }
pub fn paths64_to_polylines(in_paths: &Paths64) -> Vec<Polyline> {
    let mut out: Vec<Polyline> = Vec::with_capacity(in_paths.len());
    for path64 in in_paths {
        let mut points: Vec<Point> = Vec::with_capacity(path64.len());
        for &(x, y) in path64 {
            points.push(Point::new(x, y));
        }
        out.push(Polyline::from_points(points));
    }
    out
}

/// Clipper2Utils.cpp:23
/// C++: template <typename T>
/// C++: Clipper2Lib::Paths64 Slic3rPoints_to_Paths64(const std::vector<T>& in)
/// C++: {
/// C++:     Clipper2Lib::Paths64 out;
/// C++:     out.reserve(in.size());
/// C++:     for (const T item: in) {
/// C++:         Clipper2Lib::Path64 path;
/// C++:         path.reserve(item.size());
/// C++:         for (const Slic3r::Point& point : item.points)
/// C++:             path.emplace_back(std::move(Clipper2Lib::Point64(point.x(), point.y())));
/// C++:         out.emplace_back(std::move(path));
/// C++:     }
/// C++:     return out;
/// C++: }
///
/// The C++ template is instantiated for both `Polylines` and `Polygons`
/// (anything exposing a `points` member). We provide two concrete overloads.
fn slic3r_polylines_to_paths64(in_items: &[Polyline]) -> Paths64 {
    let mut out: Paths64 = Vec::with_capacity(in_items.len());
    for item in in_items {
        let mut path: Path64 = Vec::with_capacity(item.points.len());
        for point in &item.points {
            path.push((point.x(), point.y()));
        }
        out.push(path);
    }
    out
}

fn slic3r_polygons_points_to_paths64(in_items: &[Polygon]) -> Paths64 {
    let mut out: Paths64 = Vec::with_capacity(in_items.len());
    for item in in_items {
        let mut path: Path64 = Vec::with_capacity(item.points.len());
        for point in &item.points {
            path.push((point.x(), point.y()));
        }
        out.push(path);
    }
    out
}

/// Clipper2Utils.cpp:38
/// C++: Points Path64ToPoints(const Clipper2Lib::Path64& path64)
/// C++: {
/// C++:     Points points;
/// C++:     points.reserve(path64.size());
/// C++:     for (const Clipper2Lib::Point64 &point64 : path64) points.emplace_back(std::move(Slic3r::Point(point64.x, point64.y)));
/// C++:     return points;
/// C++: }
pub fn path64_to_points(path64: &Path64) -> Vec<Point> {
    let mut points: Vec<Point> = Vec::with_capacity(path64.len());
    for &(x, y) in path64 {
        points.push(Point::new(x, y));
    }
    points
}

/// Clipper2Utils.cpp:90
/// C++: Clipper2Lib::Paths64 Slic3rPolygons_to_Paths64(const Polygons &in)
/// C++: {
/// C++:     Clipper2Lib::Paths64 out;
/// C++:     out.reserve(in.size());
/// C++:     for (const Polygon &poly : in) {
/// C++:         Clipper2Lib::Path64 path;
/// C++:         path.reserve(poly.points.size());
/// C++:         for (const Slic3r::Point &point : poly.points) path.emplace_back(std::move(Clipper2Lib::Point64(point.x(), point.y())));
/// C++:         out.emplace_back(std::move(path));
/// C++:     }
/// C++:     return out;
/// C++: }
pub fn slic3r_polygons_to_paths64(in_polys: &[Polygon]) -> Paths64 {
    let mut out: Paths64 = Vec::with_capacity(in_polys.len());
    for poly in in_polys {
        let mut path: Path64 = Vec::with_capacity(poly.points.len());
        for point in &poly.points {
            path.push((point.x(), point.y()));
        }
        out.push(path);
    }
    out
}

/// Clipper2Utils.cpp:103
/// C++: Clipper2Lib::Paths64 Slic3rExPolygons_to_Paths64(const ExPolygons& in)
/// C++: {
/// C++:     Clipper2Lib::Paths64 out;
/// C++:     out.reserve(in.size());
/// C++:     for (const ExPolygon& expolygon : in) {
/// C++:         for (size_t i = 0; i < expolygon.num_contours(); i++) {
/// C++:             const auto         &poly = expolygon.contour_or_hole(i);
/// C++:             Clipper2Lib::Path64 path;
/// C++:             path.reserve(poly.points.size());
/// C++:             for (const Slic3r::Point &point : poly.points) path.emplace_back(std::move(Clipper2Lib::Point64(point.x(), point.y())));
/// C++:             out.emplace_back(std::move(path));
/// C++:         }
/// C++:     }
/// C++:     return out;
/// C++: }
pub fn slic3r_expolygons_to_paths64(in_expolys: &ExPolygons) -> Paths64 {
    let mut out: Paths64 = Vec::with_capacity(in_expolys.len());
    for expolygon in in_expolys {
        // expolygon.num_contours() == 1 (contour) + holes; contour_or_hole(0) is
        // the contour, contour_or_hole(i>0) is holes[i-1].
        for i in 0..expolygon.num_contours() {
            let poly: &Polygon = if i == 0 {
                &expolygon.contour
            } else {
                &expolygon.holes[i - 1]
            };
            let mut path: Path64 = Vec::with_capacity(poly.points.len());
            for point in &poly.points {
                path.push((point.x(), point.y()));
            }
            out.push(path);
        }
    }
    out
}

// ============================================================================
// PolyTree -> ExPolygons reconstruction
//
// Clipper2Utils.cpp:46
// C++: static ExPolygons PolyTreeToExPolygons(Clipper2Lib::PolyTree64 &&polytree)
//
// Ported faithfully against the native `ClipperPolyTree64` obtained from the
// raw clipper2c-sys FFI, walking the real Clipper2 nesting hierarchy exactly as
// the C++ does (no flat-paths approximation).
// ============================================================================

/// Read a native `ClipperPolyTree64` node's own polygon into our `Path64`.
unsafe fn polynode_polygon(node: *mut ClipperPolyTree64) -> Path64 {
    let mem = clipper_allocate(clipper_path64_size());
    let path = clipper_polytree64_polygon(mem, node);
    // clipper_path64_of_points-style readback; reuse the path-length accessors.
    let len: i32 = clipper2c_sys::clipper_path64_length(path) as i32;
    let mut out: Path64 = Vec::with_capacity(len.max(0) as usize);
    for j in 0..len {
        let pt = clipper2c_sys::clipper_path64_get_point(path, j);
        out.push((pt.x, pt.y));
    }
    clipper_delete_path64(path);
    out
}

/// Number of children of a native PolyTree node.
#[inline]
unsafe fn polynode_count(node: *mut ClipperPolyTree64) -> usize {
    clipper_polytree64_count(node)
}

/// `idx`-th child of a native PolyTree node (mutable pointer).
#[inline]
unsafe fn polynode_child(node: *mut ClipperPolyTree64, idx: usize) -> *mut ClipperPolyTree64 {
    clipper_polytree64_get_child(node, idx) as *mut ClipperPolyTree64
}

/// Clipper2Utils.cpp:64 (Inner::PolyTreeCountExPolygons)
/// C++: static size_t PolyTreeCountExPolygons(const Clipper2Lib::PolyPath64& polynode)
/// C++: {
/// C++:     size_t cnt = 1;
/// C++:     for (size_t i = 0; i < polynode.Count(); ++i) {
/// C++:         for (size_t j = 0; j < polynode.Child(i)->Count(); ++j) cnt += PolyTreeCountExPolygons(*polynode.Child(i)->Child(j));
/// C++:     }
/// C++:     return cnt;
/// C++: }
unsafe fn poly_tree_count_expolygons(polynode: *mut ClipperPolyTree64) -> usize {
    let mut cnt: usize = 1;
    let count = polynode_count(polynode);
    for i in 0..count {
        let child_i = polynode_child(polynode, i);
        let child_i_count = polynode_count(child_i);
        for j in 0..child_i_count {
            cnt += poly_tree_count_expolygons(polynode_child(child_i, j));
        }
    }
    cnt
}

/// Clipper2Utils.cpp:50 (Inner::PolyTreeToExPolygonsRecursive)
/// C++: static void PolyTreeToExPolygonsRecursive(Clipper2Lib::PolyTree64 &&polynode, ExPolygons *expolygons)
/// C++: {
/// C++:     size_t cnt = expolygons->size();
/// C++:     expolygons->resize(cnt + 1);
/// C++:     (*expolygons)[cnt].contour.points = Path64ToPoints(polynode.Polygon());
/// C++:
/// C++:     (*expolygons)[cnt].holes.resize(polynode.Count());
/// C++:     for (int i = 0; i < polynode.Count(); ++i) {
/// C++:         (*expolygons)[cnt].holes[i].points = Path64ToPoints(polynode[i]->Polygon());
/// C++:         // Add outer polygons contained by (nested within) holes.
/// C++:         for (int j = 0; j < polynode[i]->Count(); ++j) PolyTreeToExPolygonsRecursive(std::move(*polynode[i]->Child(j)), expolygons);
/// C++:     }
/// C++: }
unsafe fn poly_tree_to_expolygons_recursive(
    polynode: *mut ClipperPolyTree64,
    expolygons: &mut ExPolygons,
) {
    // size_t cnt = expolygons->size();
    let cnt = expolygons.len();
    // expolygons->resize(cnt + 1);
    expolygons.push(ExPolygon::empty());
    // (*expolygons)[cnt].contour.points = Path64ToPoints(polynode.Polygon());
    expolygons[cnt].contour = Polygon::from_points(path64_to_points(&polynode_polygon(polynode)));

    // (*expolygons)[cnt].holes.resize(polynode.Count());
    let count = polynode_count(polynode);
    expolygons[cnt].holes = vec![Polygon::default(); count];
    for i in 0..count {
        let child_i = polynode_child(polynode, i);
        // (*expolygons)[cnt].holes[i].points = Path64ToPoints(polynode[i]->Polygon());
        expolygons[cnt].holes[i] =
            Polygon::from_points(path64_to_points(&polynode_polygon(child_i)));
        // Add outer polygons contained by (nested within) holes.
        let child_i_count = polynode_count(child_i);
        for j in 0..child_i_count {
            poly_tree_to_expolygons_recursive(polynode_child(child_i, j), expolygons);
        }
    }
}

/// Clipper2Utils.cpp:46
/// C++: static ExPolygons PolyTreeToExPolygons(Clipper2Lib::PolyTree64 &&polytree)
/// C++: {
/// C++:     ... (Inner struct above) ...
/// C++:     ExPolygons retval;
/// C++:     size_t     cnt = 0;
/// C++:     for (int i = 0; i < polytree.Count(); ++i) cnt += Inner::PolyTreeCountExPolygons(*polytree[i]);
/// C++:     retval.reserve(cnt);
/// C++:     for (int i = 0; i < polytree.Count(); ++i) Inner::PolyTreeToExPolygonsRecursive(std::move(*polytree[i]), &retval);
/// C++:     return retval;
/// C++: }
unsafe fn poly_tree_to_expolygons(polytree: *mut ClipperPolyTree64) -> ExPolygons {
    let mut retval: ExPolygons = Vec::new();
    let mut cnt: usize = 0;
    let count = polynode_count(polytree);
    for i in 0..count {
        cnt += poly_tree_count_expolygons(polynode_child(polytree, i));
    }
    retval.reserve(cnt);
    for i in 0..count {
        poly_tree_to_expolygons_recursive(polynode_child(polytree, i), &mut retval);
    }
    retval
}

// ============================================================================
// Native execute / offset helpers (faithful to Clipper2Lib::Clipper64 /
// ClipperOffset usage in Clipper2Utils.cpp).
// ============================================================================

/// Run a `Clipper64` boolean op (closed subject + clip) into a native PolyTree64,
/// matching C++ `c.Execute(ct, fr, solution)` where `solution` is a PolyTree64.
/// Returns the reconstructed ExPolygons via `PolyTreeToExPolygons`.
unsafe fn clipper64_union_tree_to_expolygons(subject: &Paths64) -> ExPolygons {
    let c_mem = clipper_allocate(clipper_clipper64_size());
    let c = clipper_clipper64(c_mem);

    let subject_native = paths64_to_native(subject);
    clipper_clipper64_add_subject(c, subject_native);
    clipper_delete_paths64(subject_native);

    let tree_mem = clipper_allocate(clipper_polytree64_size());
    let tree = clipper_polytree64(tree_mem, std::ptr::null_mut());
    // Open output buffer is required by the C wrapper signature; closed-path
    // unions never emit open paths, mirroring C++ which uses the PolyTree-only
    // Execute overload.
    let open_mem = clipper_allocate(clipper_paths64_size());
    let open = clipper2c_sys::clipper_paths64(open_mem);

    clipper_clipper64_execute_tree_with_open(
        c,
        ClipperClipType_UNION,
        ClipperFillRule_NON_ZERO,
        tree,
        open,
    );

    let result = poly_tree_to_expolygons(tree);

    clipper_delete_paths64(open);
    clipper_delete_polytree64(tree);
    clipper_delete_clipper64(c);
    result
}

/// Run a `ClipperOffset` over `subject` with the given delta into flat Paths64,
/// matching C++ `ClipperOffset offsetter; offsetter.AddPaths(..., Round, Polygon);
/// offsetter.Execute(delta, polytree)` (default miter_limit=2.0, arc_tolerance=0.0).
unsafe fn clipper_offset_execute(subject: &Paths64, delta: f64) -> Paths64 {
    let co_mem = clipper_allocate(clipper_clipperoffset_size());
    // Default ClipperOffset: miter_limit=2.0, arc_tolerance=0.0,
    // preserve_collinear=false, reverse_solution=false.
    let co = clipper_clipperoffset(co_mem, 2.0, 0.0, 0, 0);

    let subject_native = paths64_to_native(subject);
    clipper_clipperoffset_add_paths64(
        co,
        subject_native,
        ClipperJoinType_ROUND_JOIN,
        ClipperEndType_POLYGON_END,
    );
    clipper_delete_paths64(subject_native);

    let res_mem = clipper_allocate(clipper_paths64_size());
    let res = clipper_clipperoffset_execute(res_mem, co, delta);
    let out = native_to_paths64(res);

    clipper_delete_paths64(res);
    clipper_delete_clipperoffset(co);
    out
}

/// Convert the flat offset result paths into a native PolyTree64 (so the same
/// `PolyTreeToExPolygons` reconstruction applies) and reconstruct ExPolygons.
/// The offset C-wrapper returns flat paths only (the C++ `ClipperOffset::Execute`
/// PolyPath64 overload is not exposed), so the nesting hierarchy is recovered by
/// a NonZero union execute-to-tree — Clipper2 builds the identical PolyTree the
/// offsetter would, since both encode containment of the same offset contours.
unsafe fn offset_paths_to_expolygons(offset_paths: &Paths64) -> ExPolygons {
    clipper64_union_tree_to_expolygons(offset_paths)
}

/// Clipper2Utils.cpp:82
/// C++: void SimplifyPolyTree(const Clipper2Lib::PolyPath64 &polytree, double epsilon, Clipper2Lib::PolyPath64 &result)
/// C++: {
/// C++:     for (const auto &child : polytree) {
/// C++:         Clipper2Lib::PolyPath64 *newchild = result.AddChild(Clipper2Lib::SimplifyPath(child->Polygon(), epsilon));
/// C++:         SimplifyPolyTree(*child, epsilon, *newchild);
/// C++:     }
/// C++: }
///
/// We hold the offset result as a native PolyTree (built from the flat offset
/// paths). `SimplifyPolyTree` recursively simplifies every contour in the tree
/// with `Clipper2Lib::SimplifyPath` (closed-path simplification) and rebuilds an
/// identically-structured tree. Since the subsequent `PolyTreeToPaths64` flattens
/// the tree back to paths anyway, this is equivalent to simplifying every path in
/// the flattened tree with the same epsilon — which we do via the native
/// `simplify` on the flat paths.
unsafe fn simplify_poly_tree_paths(paths: &Paths64, epsilon: f64) -> Paths64 {
    // Clipper2Lib::SimplifyPath is closed-path simplification (is_open = false).
    let native = paths64_to_native(paths);
    let result = clipper2c_sys::clipper_paths64_simplify(
        clipper_allocate(clipper_paths64_size()),
        native,
        epsilon,
        0,
    );
    clipper_delete_paths64(native);
    let out = native_to_paths64(result);
    clipper_delete_paths64(result);
    out
}

// ============================================================================
// Boolean Operations (Polylines)
// ============================================================================

/// Clipper2 clip type enumeration (only the variants used here).
/// Mirrors `Clipper2Lib::ClipType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClipType {
    Intersection,
    Difference,
}

/// Clipper2Utils.cpp:119
/// C++: Polylines _clipper2_pl_open(Clipper2Lib::ClipType clipType, const Slic3r::Polylines& subject, const Slic3r::Polygons& clip)
/// C++: {
/// C++:     Clipper2Lib::Clipper64 c;
/// C++:     c.AddOpenSubject(Slic3rPoints_to_Paths64(subject));
/// C++:     c.AddClip(Slic3rPoints_to_Paths64(clip));
/// C++:
/// C++:     Clipper2Lib::ClipType ct = clipType;
/// C++:     Clipper2Lib::FillRule fr = Clipper2Lib::FillRule::NonZero;
/// C++:     Clipper2Lib::Paths64 solution, solution_open;
/// C++:     c.Execute(ct, fr, solution, solution_open);
/// C++:
/// C++:     Slic3r::Polylines out;
/// C++:     out.reserve(solution.size() + solution_open.size());
/// C++:     polylines_append(out, std::move(Paths64_to_polylines(solution)));
/// C++:     polylines_append(out, std::move(Paths64_to_polylines(solution_open)));
/// C++:
/// C++:     return out;
/// C++: }
fn clipper2_pl_open(clip_type: ClipType, subject: &[Polyline], clip: &[Polygon]) -> Vec<Polyline> {
    unsafe {
        // C++: Clipper2Lib::Clipper64 c;
        let c_mem = clipper_allocate(clipper_clipper64_size());
        let c = clipper_clipper64(c_mem);

        // C++: c.AddOpenSubject(Slic3rPoints_to_Paths64(subject));
        let subject_native = paths64_to_native(&slic3r_polylines_to_paths64(subject));
        clipper_clipper64_add_open_subject(c, subject_native);
        clipper_delete_paths64(subject_native);

        // C++: c.AddClip(Slic3rPoints_to_Paths64(clip));
        let clip_native = paths64_to_native(&slic3r_polygons_points_to_paths64(clip));
        clipper_clipper64_add_clip(c, clip_native);
        clipper_delete_paths64(clip_native);

        // C++: Clipper2Lib::ClipType ct = clipType;
        // C++: Clipper2Lib::FillRule fr = Clipper2Lib::FillRule::NonZero;
        // C++: Clipper2Lib::Paths64 solution, solution_open;
        // C++: c.Execute(ct, fr, solution, solution_open);
        //
        // With an OPEN subject clipped against a closed polygon, Clipper2 places
        // the clipped open paths into `solution_open`. We call the raw FFI which
        // exposes BOTH output buffers (the safe `clipper2` wrapper discards
        // `solution_open`), so both sets are recovered exactly as in C++.
        let ct = match clip_type {
            ClipType::Intersection => ClipperClipType_INTERSECTION,
            ClipType::Difference => ClipperClipType_DIFFERENCE,
        };
        let solution_mem = clipper_allocate(clipper_paths64_size());
        let solution = clipper2c_sys::clipper_paths64(solution_mem);
        let solution_open_mem = clipper_allocate(clipper_paths64_size());
        let solution_open = clipper2c_sys::clipper_paths64(solution_open_mem);
        clipper_clipper64_execute(c, ct, ClipperFillRule_NON_ZERO, solution, solution_open);

        // C++: out.reserve(solution.size() + solution_open.size());
        // C++: polylines_append(out, std::move(Paths64_to_polylines(solution)));
        // C++: polylines_append(out, std::move(Paths64_to_polylines(solution_open)));
        let solution64 = native_to_paths64(solution);
        let solution_open64 = native_to_paths64(solution_open);
        let mut out: Vec<Polyline> =
            Vec::with_capacity(solution64.len() + solution_open64.len());
        out.extend(paths64_to_polylines(&solution64));
        out.extend(paths64_to_polylines(&solution_open64));

        clipper_delete_paths64(solution);
        clipper_delete_paths64(solution_open);
        clipper_delete_clipper64(c);
        out
    }
}

/// Clipper2Utils.hpp:10
/// Clipper2Utils.cpp:138
/// C++: Slic3r::Polylines intersection_pl_2(const Slic3r::Polylines& subject, const Slic3r::Polygons& clip)
/// C++:     { return _clipper2_pl_open(Clipper2Lib::ClipType::Intersection, subject, clip); }
pub fn intersection_pl_2(subject: &[Polyline], clip: &[Polygon]) -> Vec<Polyline> {
    clipper2_pl_open(ClipType::Intersection, subject, clip)
}

/// Clipper2Utils.hpp:11
/// Clipper2Utils.cpp:140
/// C++: Slic3r::Polylines  diff_pl_2(const Slic3r::Polylines& subject, const Slic3r::Polygons& clip)
/// C++:     { return _clipper2_pl_open(Clipper2Lib::ClipType::Difference, subject, clip); }
pub fn diff_pl_2(subject: &[Polyline], clip: &[Polygon]) -> Vec<Polyline> {
    clipper2_pl_open(ClipType::Difference, subject, clip)
}

// ============================================================================
// Boolean Operations (Polygons / ExPolygons)
// ============================================================================

/// Clipper2Utils.hpp:12
/// Clipper2Utils.cpp:143
/// C++: ExPolygons union_ex_2(const Polygons& polygons)
/// C++: {
/// C++:     Clipper2Lib::Clipper64 c;
/// C++:     c.AddSubject(Slic3rPolygons_to_Paths64(polygons));
/// C++:
/// C++:     Clipper2Lib::ClipType ct = Clipper2Lib::ClipType::Union;
/// C++:     Clipper2Lib::FillRule fr = Clipper2Lib::FillRule::NonZero;
/// C++:     Clipper2Lib::PolyTree64 solution;
/// C++:     c.Execute(ct, fr, solution);
/// C++:
/// C++:     ExPolygons results = PolyTreeToExPolygons(std::move(solution));
/// C++:
/// C++:     return results;
/// C++: }
pub fn union_ex_2(polygons: &[Polygon]) -> ExPolygons {
    // C++: c.AddSubject(Slic3rPolygons_to_Paths64(polygons));
    // C++: Clipper2Lib::ClipType ct = Clipper2Lib::ClipType::Union;
    // C++: Clipper2Lib::FillRule fr = Clipper2Lib::FillRule::NonZero;
    // C++: Clipper2Lib::PolyTree64 solution;
    // C++: c.Execute(ct, fr, solution);
    // C++: ExPolygons results = PolyTreeToExPolygons(std::move(solution));
    unsafe { clipper64_union_tree_to_expolygons(&slic3r_polygons_to_paths64(polygons)) }
}

/// Clipper2Utils.hpp:13
/// Clipper2Utils.cpp:158
/// C++: ExPolygons union_ex_2(const ExPolygons &expolygons)
/// C++: {
/// C++:     Clipper2Lib::Clipper64 c;
/// C++:     c.AddSubject(Slic3rExPolygons_to_Paths64(expolygons));
/// C++:
/// C++:     Clipper2Lib::ClipType   ct = Clipper2Lib::ClipType::Union;
/// C++:     Clipper2Lib::FillRule   fr = Clipper2Lib::FillRule::NonZero;
/// C++:     Clipper2Lib::PolyTree64 solution;
/// C++:     c.Execute(ct, fr, solution);
/// C++:
/// C++:     ExPolygons results = PolyTreeToExPolygons(std::move(solution));
/// C++:
/// C++:     return results;
/// C++: }
///
/// (C++ overloads `union_ex_2`; Rust has no overloading so the ExPolygons
/// variant is suffixed `_expolygons`.)
pub fn union_ex_2_expolygons(expolygons: &ExPolygons) -> ExPolygons {
    // C++: c.AddSubject(Slic3rExPolygons_to_Paths64(expolygons));
    // C++: Clipper2Lib::ClipType   ct = Clipper2Lib::ClipType::Union;
    // C++: Clipper2Lib::FillRule   fr = Clipper2Lib::FillRule::NonZero;
    // C++: Clipper2Lib::PolyTree64 solution;
    // C++: c.Execute(ct, fr, solution);
    // C++: ExPolygons results = PolyTreeToExPolygons(std::move(solution));
    unsafe { clipper64_union_tree_to_expolygons(&slic3r_expolygons_to_paths64(expolygons)) }
}

// ============================================================================
// Offset Operations
// ============================================================================

/// 对 ExPolygons 进行偏移
/// Clipper2Utils.hpp:14
/// Clipper2Utils.cpp:174
/// C++: ExPolygons offset_ex_2(const ExPolygons &expolygons, double delta)
/// C++: {
/// C++:     Clipper2Lib::Paths64 subject = Slic3rExPolygons_to_Paths64(expolygons);
/// C++:     Clipper2Lib::ClipperOffset offsetter;
/// C++:     offsetter.AddPaths(subject, Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);
/// C++:     Clipper2Lib::PolyPath64 polytree;
/// C++:     offsetter.Execute(delta, polytree);
/// C++:     ExPolygons results = PolyTreeToExPolygons(std::move(polytree));
/// C++:
/// C++:     return results;
/// C++: }
pub fn offset_ex_2(expolygons: &ExPolygons, delta: f64) -> ExPolygons {
    unsafe {
        // C++: Clipper2Lib::Paths64 subject = Slic3rExPolygons_to_Paths64(expolygons);
        let subject = slic3r_expolygons_to_paths64(expolygons);

        // C++: Clipper2Lib::ClipperOffset offsetter;
        // C++: offsetter.AddPaths(subject, Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);
        // C++: Clipper2Lib::PolyPath64 polytree;
        // C++: offsetter.Execute(delta, polytree);
        let polytree = clipper_offset_execute(&subject, delta);

        // C++: ExPolygons results = PolyTreeToExPolygons(std::move(polytree));
        offset_paths_to_expolygons(&polytree)
    }
}

/// Clipper2Utils.hpp:15
/// Clipper2Utils.cpp:186
/// C++: ExPolygons offset2_ex_2(const ExPolygons& expolygons, double delta1, double delta2)
/// C++: {
/// C++:     // 1st offset
/// C++:     Clipper2Lib::Paths64       subject = Slic3rExPolygons_to_Paths64(expolygons);
/// C++:     Clipper2Lib::ClipperOffset offsetter;
/// C++:     offsetter.AddPaths(subject, Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);
/// C++:     Clipper2Lib::PolyPath64 polytree;
/// C++:     offsetter.Execute(delta1, polytree);
/// C++:
/// C++:     // simplify the result
/// C++:     Clipper2Lib::PolyPath64 polytree2;
/// C++:     SimplifyPolyTree(polytree, SCALED_EPSILON, polytree2);
/// C++:
/// C++:     // 2nd offset
/// C++:     offsetter.Clear();
/// C++:     offsetter.AddPaths(Clipper2Lib::PolyTreeToPaths64(polytree2), Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);
/// C++:     polytree.Clear();
/// C++:     offsetter.Execute(delta2, polytree);
/// C++:
/// C++:     // convert back to expolygons
/// C++:     ExPolygons results = PolyTreeToExPolygons(std::move(polytree));
/// C++:
/// C++:     return results;
/// C++: }
pub fn offset2_ex_2(expolygons: &ExPolygons, delta1: f64, delta2: f64) -> ExPolygons {
    unsafe {
        // 1st offset
        // C++: Clipper2Lib::Paths64       subject = Slic3rExPolygons_to_Paths64(expolygons);
        // C++: Clipper2Lib::ClipperOffset offsetter;
        // C++: offsetter.AddPaths(subject, Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);
        // C++: Clipper2Lib::PolyPath64 polytree;
        // C++: offsetter.Execute(delta1, polytree);
        let subject = slic3r_expolygons_to_paths64(expolygons);
        let polytree = clipper_offset_execute(&subject, delta1);

        // simplify the result
        // C++: Clipper2Lib::PolyPath64 polytree2;
        // C++: SimplifyPolyTree(polytree, SCALED_EPSILON, polytree2);
        let polytree2 = simplify_poly_tree_paths(&polytree, SCALED_EPSILON);

        // 2nd offset
        // C++: offsetter.Clear();
        // C++: offsetter.AddPaths(Clipper2Lib::PolyTreeToPaths64(polytree2), Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);
        // C++: polytree.Clear();
        // C++: offsetter.Execute(delta2, polytree);
        let polytree = clipper_offset_execute(&polytree2, delta2);

        // convert back to expolygons
        // C++: ExPolygons results = PolyTreeToExPolygons(std::move(polytree));
        offset_paths_to_expolygons(&polytree)
    }
}

// ============================================================================
// Variable offset (ClipperUtils.cpp). These four `variable_offset_*` entry
// points and their helpers (`mittered_offset_path_scaled`,
// `fix_after_inner_offset`, `fix_after_outer_offset`) live in
// `src/libslic3r/ClipperUtils.cpp` upstream, but they are implemented here so
// they can reuse this module's raw `clipper2c-sys` FFI plumbing
// (`paths64_to_native`/`native_to_paths64`/`poly_tree_to_expolygons`) instead of
// duplicating it against the geo-clipper backend used in `clipper_utils.rs`.
//
// BACKEND NOTE: upstream `ClipperUtils.cpp` uses the legacy Clipper1
// (`ClipperLib::Clipper`) with `ReverseSolution()` + `pftNegative`/`pftPositive`
// fill types. This crate links Clipper2 (no Clipper1 dependency exists, and
// adding one would introduce a new native backend). Clipper2 exposes the same
// winding-rule fill modes (`Positive`/`Negative`/`NonZero`) plus
// `set_reverse_solution`, so the boolean-op semantics are reproduced 1:1 on the
// Clipper2 FFI — matching the crate-wide convention of backing Clipper1-era C++
// with Clipper2 (see the rest of this file). `ClipperLib::Area(path) > 0`
// assertions map onto `Polygon::area() > 0`.
// ============================================================================

/// Internal clipper offset shortest-edge factor.
/// ClipperUtils.hpp:44 — `static constexpr const double ClipperOffsetShortestEdgeFactor = 0.005;`
const CLIPPER_OFFSET_SHORTEST_EDGE_FACTOR: f64 = 0.005;

// Cast a scaled integer Point to a double vector (Eigen `.cast<double>()`).
#[inline]
fn pt_to_d_2(p: Point) -> PointF {
    PointF::new(p.x as f64, p.y as f64)
}

/// Run a `Clipper64` UNION over a single closed subject path into flat Paths64,
/// with the given fill rule and `reverse_solution` flag. Mirrors the
/// `ClipperLib::Clipper` usage in `fix_after_outer_offset` / `fix_after_inner_offset`
/// (`c.AddPath(...); c.ReverseSolution(rev); c.Execute(ctUnion, solution, fill, fill)`).
unsafe fn clipper64_union_paths(
    subject: &Paths64,
    fillrule: ClipperFillRule,
    reverse_solution: bool,
) -> Paths64 {
    let c_mem = clipper_allocate(clipper_clipper64_size());
    let c = clipper_clipper64(c_mem);

    let subject_native = paths64_to_native(subject);
    clipper_clipper64_add_subject(c, subject_native);
    clipper_delete_paths64(subject_native);

    clipper_clipper64_set_reverse_solution(c, reverse_solution as ::std::os::raw::c_int);

    let solution_mem = clipper_allocate(clipper_paths64_size());
    let solution = clipper2c_sys::clipper_paths64(solution_mem);
    let open_mem = clipper_allocate(clipper_paths64_size());
    let open = clipper2c_sys::clipper_paths64(open_mem);
    clipper_clipper64_execute(c, ClipperClipType_UNION, fillrule, solution, open);

    let out = native_to_paths64(solution);

    clipper_delete_paths64(solution);
    clipper_delete_paths64(open);
    clipper_delete_clipper64(c);
    out
}

/// Run a `Clipper64` DIFFERENCE (subject minus clip) into a native PolyTree64 and
/// reconstruct ExPolygons. Mirrors the `ClipperLib::Clipper` difference into a
/// `PolyTree` used by `variable_offset_inner_ex` / `variable_offset_outer_ex`.
unsafe fn clipper64_difference_tree_to_expolygons(
    subject: &Paths64,
    clip: &Paths64,
) -> ExPolygons {
    let c_mem = clipper_allocate(clipper_clipper64_size());
    let c = clipper_clipper64(c_mem);

    let subject_native = paths64_to_native(subject);
    clipper_clipper64_add_subject(c, subject_native);
    clipper_delete_paths64(subject_native);

    let clip_native = paths64_to_native(clip);
    clipper_clipper64_add_clip(c, clip_native);
    clipper_delete_paths64(clip_native);

    let tree_mem = clipper_allocate(clipper_polytree64_size());
    let tree = clipper_polytree64(tree_mem, std::ptr::null_mut());
    let open_mem = clipper_allocate(clipper_paths64_size());
    let open = clipper2c_sys::clipper_paths64(open_mem);
    clipper_clipper64_execute_tree_with_open(
        c,
        ClipperClipType_DIFFERENCE,
        ClipperFillRule_NON_ZERO,
        tree,
        open,
    );

    let result = poly_tree_to_expolygons(tree);

    clipper_delete_paths64(open);
    clipper_delete_polytree64(tree);
    clipper_delete_clipper64(c);
    result
}

/// Run a `Clipper64` DIFFERENCE (subject minus clip) into flat Paths64.
/// Mirrors the `ClipperLib::Clipper` difference into `Paths` used by
/// `variable_offset_inner` / `variable_offset_outer`.
unsafe fn clipper64_difference_paths(subject: &Paths64, clip: &Paths64) -> Paths64 {
    let c_mem = clipper_allocate(clipper_clipper64_size());
    let c = clipper_clipper64(c_mem);

    let subject_native = paths64_to_native(subject);
    clipper_clipper64_add_subject(c, subject_native);
    clipper_delete_paths64(subject_native);

    let clip_native = paths64_to_native(clip);
    clipper_clipper64_add_clip(c, clip_native);
    clipper_delete_paths64(clip_native);

    let solution_mem = clipper_allocate(clipper_paths64_size());
    let solution = clipper2c_sys::clipper_paths64(solution_mem);
    let open_mem = clipper_allocate(clipper_paths64_size());
    let open = clipper2c_sys::clipper_paths64(open_mem);
    clipper_clipper64_execute(
        c,
        ClipperClipType_DIFFERENCE,
        ClipperFillRule_NON_ZERO,
        solution,
        open,
    );

    let out = native_to_paths64(solution);

    clipper_delete_paths64(solution);
    clipper_delete_paths64(open);
    clipper_delete_clipper64(c);
    out
}

/// Build a `Polygon` from a clipper `Path64` (`to_polygons` / `Slic3r::Polygon(Path&&)`).
#[inline]
fn path64_to_polygon(path: &Path64) -> Polygon {
    Polygon::from_points(path64_to_points(path))
}

/// ClipperUtils.cpp:1077 — fix_after_outer_offset
/// Outer offset shall not split the input contour into multiples. It is expected,
/// that the solution will be non empty and it will contain just a single polygon.
/// Defaults: filltype = pftPositive, reverse_result = false.
fn fix_after_outer_offset(
    input: &Path64,
    filltype: ClipperFillRule,
    reverse_result: bool,
) -> Paths64 {
    // ClipperUtils.cpp:1084-1090
    let mut solution: Paths64 = Vec::new();
    if !input.is_empty() {
        // c.AddPath(input, ptSubject, true);
        // c.ReverseSolution(reverse_result);
        // c.Execute(ctUnion, solution, filltype, filltype);
        solution = unsafe { clipper64_union_paths(&vec![input.clone()], filltype, reverse_result) };
    }
    solution
}

/// ClipperUtils.cpp:1095 — fix_after_inner_offset
/// Inner offset may split the source contour into multiple contours, but one
/// resulting contour shall not lie inside the other.
/// Defaults: filltype = pftNegative, reverse_result = true.
fn fix_after_inner_offset(
    input: &Path64,
    filltype: ClipperFillRule,
    reverse_result: bool,
) -> Paths64 {
    // ClipperUtils.cpp:1102-1116
    let mut solution: Paths64 = Vec::new();
    if !input.is_empty() {
        // ClipperLib::IntRect r = clipper.GetBounds();
        // GetBounds() returns the bounding box of all added subject paths; at this
        // point only `input` is added, so the bounds are the min/max over its
        // points — computed directly (Clipper2 exposes no bounds via this FFI).
        let mut left = input[0].0;
        let mut top = input[0].1;
        let mut right = input[0].0;
        let mut bottom = input[0].1;
        for &(x, y) in input.iter() {
            left = left.min(x);
            right = right.max(x);
            top = top.min(y);
            bottom = bottom.max(y);
        }
        // r.left -= 10; r.top -= 10; r.right += 10; r.bottom += 10;
        left -= 10;
        top -= 10;
        right += 10;
        bottom += 10;
        // The bounding rectangle is wound CCW for pftPositive, CW otherwise so the
        // big rectangle is a "hole-killer" with the right winding sign.
        let rect: Path64 = if filltype == ClipperFillRule_POSITIVE {
            vec![(left, bottom), (left, top), (right, top), (right, bottom)]
        } else {
            vec![(left, bottom), (right, bottom), (right, top), (left, top)]
        };
        // c.AddPath(input, ...); c.AddPath(rect, ...);
        // c.ReverseSolution(reverse_result);
        // c.Execute(ctUnion, solution, filltype, filltype);
        let subject: Paths64 = vec![input.clone(), rect];
        solution = unsafe { clipper64_union_paths(&subject, filltype, reverse_result) };
        // if (! solution.empty()) solution.erase(solution.begin());
        if !solution.is_empty() {
            solution.remove(0);
        }
    }
    solution
}

/// ClipperUtils.cpp:1120 — mittered_offset_path_scaled
fn mittered_offset_path_scaled(contour: &[Point], deltas: &[f32], mut miter_limit: f64) -> Path64 {
    debug_assert_eq!(contour.len(), deltas.len());

    #[cfg(debug_assertions)]
    {
        // ClipperUtils.cpp:1124-1134 — deltas are either all positive or all negative.
        let mut positive = false;
        let mut negative = false;
        for &delta in deltas {
            if delta < 0.0 {
                negative = true;
            } else if delta > 0.0 {
                positive = true;
            }
        }
        debug_assert!(!(negative && positive));
    }

    let mut out: Path64 = Vec::new();

    // ClipperUtils.cpp:1138
    if deltas.len() > 2 {
        out.reserve(contour.len() * 2);

        // ClipperUtils.cpp:1143 — Clamp miter limit to 2.
        miter_limit = if miter_limit > 2.0 {
            2.0 / (miter_limit * miter_limit)
        } else {
            0.5
        };

        // perpenduclar vector
        let perp = |v: PointF| -> PointF { PointF::new(v.y, -v.x) };

        // ClipperUtils.cpp:1149-1152 — Add a new point to the output, round to cInt.
        let add_offset_point = |out: &mut Path64, mut pt: PointF| {
            pt = pt
                + PointF::new(
                    0.5 - ((pt.x < 0.0) as i32 as f64),
                    0.5 - ((pt.y < 0.0) as i32 as f64),
                );
            out.push((pt.x as i64, pt.y as i64));
        };

        // ClipperUtils.cpp:1155-1156 — Minimum edge length, squared.
        let lmin = deltas.iter().cloned().fold(f32::MIN, f32::max) as f64
            * CLIPPER_OFFSET_SHORTEST_EDGE_FACTOR;
        let l2min = lmin * lmin;
        // ClipperUtils.cpp:1161 — Implementation equal to Clipper.
        let sin_min_parallel = 1.0_f64;

        // ClipperUtils.cpp:1164-1171 — Find the last point further from pt by l2min.
        let mut pt = pt_to_d_2(contour[0]);
        let mut iprev = contour.len() - 1;
        let mut ptprev = PointF::new(0.0, 0.0);
        while iprev > 0 {
            ptprev = pt_to_d_2(contour[iprev]);
            let d = ptprev - pt;
            if d.x * d.x + d.y * d.y > l2min {
                break;
            }
            iprev -= 1;
        }

        // ClipperUtils.cpp:1173
        if iprev != 0 {
            let ilast = iprev;
            // Normal to the (pt - ptprev) segment.
            let mut nprev = normalize_pf(perp(pt - ptprev));
            let mut i = 0usize;
            loop {
                // ClipperUtils.cpp:1179-1186 — Find the next point further from pt by l2min.
                let mut j = i + 1;
                let mut ptnext = PointF::new(0.0, 0.0);
                while j <= ilast {
                    ptnext = pt_to_d_2(contour[j]);
                    let d = ptnext - pt;
                    let l2 = d.x * d.x + d.y * d.y;
                    if l2 > l2min {
                        break;
                    }
                    j += 1;
                }
                // ClipperUtils.cpp:1187-1192
                if j > ilast {
                    debug_assert!(i <= ilast);
                    // If the last edge is too short, merge it with the previous edge.
                    i = ilast;
                    ptnext = pt_to_d_2(contour[0]);
                }

                // ClipperUtils.cpp:1195 — Normal to the (ptnext - pt) segment.
                let nnext = normalize_pf(perp(ptnext - pt));

                // ClipperUtils.cpp:1197-1199
                let delta = deltas[i] as f64;
                let sin_a = cross2f(nprev, nnext).clamp(-1.0, 1.0);
                let convex = sin_a * delta;
                if convex <= -sin_min_parallel {
                    // ClipperUtils.cpp:1201-1204 — Concave corner.
                    add_offset_point(&mut out, pt + nprev * delta);
                    add_offset_point(&mut out, pt);
                    add_offset_point(&mut out, pt + nnext * delta);
                } else {
                    // ClipperUtils.cpp:1206
                    let dot = nprev.x * nnext.x + nprev.y * nnext.y;
                    if convex < sin_min_parallel && dot > 0.0 {
                        // ClipperUtils.cpp:1207-1209 — Nearly parallel.
                        let nd = nprev.x * nnext.x + nprev.y * nnext.y;
                        add_offset_point(&mut out, if nd > 0.0 { pt + nprev * delta } else { pt });
                    } else {
                        // ClipperUtils.cpp:1211-1226 — Convex corner.
                        let r = 1.0 + dot;
                        if r >= miter_limit {
                            add_offset_point(&mut out, pt + (nprev + nnext) * (delta / r));
                        } else {
                            let dx = (sin_a.atan2(dot) / 4.0).tan();
                            let newpt1 = pt + (nprev - perp(nprev) * dx) * delta;
                            let newpt2 = pt + (nnext + perp(nnext) * dx) * delta;
                            #[cfg(debug_assertions)]
                            {
                                let vedge = (newpt1 + newpt2) * 0.5 - pt;
                                let dist_norm = (vedge.x * vedge.x + vedge.y * vedge.y).sqrt();
                                debug_assert!((dist_norm - delta.abs()).abs() < SCALED_EPSILON);
                            }
                            add_offset_point(&mut out, newpt1);
                            add_offset_point(&mut out, newpt2);
                        }
                    }
                }

                // ClipperUtils.cpp:1230-1236
                if i == ilast {
                    break;
                }
                ptprev = pt;
                nprev = nnext;
                pt = ptnext;
                i = j;
            }
            let _ = ptprev;
        }
    }

    // ClipperUtils.cpp:1257
    out
}

/// Eigen `.normalized()`: divide by Euclidean norm.
#[inline]
fn normalize_pf(v: PointF) -> PointF {
    let n = (v.x * v.x + v.y * v.y).sqrt();
    PointF::new(v.x / n, v.y / n)
}

/// ClipperUtils.cpp:1260 — variable_offset_inner
/// hpp:668 — Polygons variable_offset_inner(const ExPolygon&, const std::vector<std::vector<float>>&, double miter_limit)
pub fn variable_offset_inner(expoly: &ExPolygon, deltas: &[Vec<f32>], miter_limit: f64) -> Polygons {
    #[cfg(debug_assertions)]
    {
        // ClipperUtils.cpp:1262-1267 — deltas are all non positive.
        for ds in deltas {
            for &delta in ds {
                debug_assert!(delta <= 0.0);
            }
        }
        debug_assert_eq!(expoly.holes.len() + 1, deltas.len());
    }

    // ClipperUtils.cpp:1270-1271 — 1) Offset the outer contour.
    let contours = fix_after_inner_offset(
        &mittered_offset_path_scaled(&expoly.contour.points, &deltas[0], miter_limit),
        ClipperFillRule_NEGATIVE,
        true,
    );
    #[cfg(debug_assertions)]
    for c in &contours {
        debug_assert!(path64_to_polygon(c).area() > 0.0);
    }

    // ClipperUtils.cpp:1278-1281 — 2) Offset the holes one by one, collect the results.
    let mut holes: Paths64 = Vec::new();
    holes.reserve(expoly.holes.len());
    for (h, hole) in expoly.holes.iter().enumerate() {
        let mut part = fix_after_outer_offset(
            &mittered_offset_path_scaled(&hole.points, &deltas[1 + h], miter_limit),
            ClipperFillRule_NEGATIVE,
            false,
        );
        holes.append(&mut part);
    }
    #[cfg(debug_assertions)]
    for c in &holes {
        debug_assert!(path64_to_polygon(c).area() > 0.0);
    }

    // ClipperUtils.cpp:1287-1297 — 3) Subtract holes from the contours.
    let output: Paths64 = if holes.is_empty() {
        contours
    } else {
        unsafe { clipper64_difference_paths(&contours, &holes) }
    };

    output.iter().map(path64_to_polygon).collect()
}

/// ClipperUtils.cpp:1302 — variable_offset_outer
/// hpp:669 — Polygons variable_offset_outer(const ExPolygon&, const std::vector<std::vector<float>>&, double miter_limit)
pub fn variable_offset_outer(expoly: &ExPolygon, deltas: &[Vec<f32>], miter_limit: f64) -> Polygons {
    #[cfg(debug_assertions)]
    {
        // ClipperUtils.cpp:1304-1309 — deltas are all non negative.
        for ds in deltas {
            for &delta in ds {
                debug_assert!(delta >= 0.0);
            }
        }
        debug_assert_eq!(expoly.holes.len() + 1, deltas.len());
    }

    // ClipperUtils.cpp:1312-1313 — 1) Offset the outer contour.
    let contours = fix_after_outer_offset(
        &mittered_offset_path_scaled(&expoly.contour.points, &deltas[0], miter_limit),
        ClipperFillRule_POSITIVE,
        false,
    );
    #[cfg(debug_assertions)]
    for c in &contours {
        debug_assert!(path64_to_polygon(c).area() > 0.0);
    }

    // ClipperUtils.cpp:1319-1323 — 2) Offset the holes one by one, collect the results.
    let mut holes: Paths64 = Vec::new();
    holes.reserve(expoly.holes.len());
    for (h, hole) in expoly.holes.iter().enumerate() {
        let mut part = fix_after_inner_offset(
            &mittered_offset_path_scaled(&hole.points, &deltas[1 + h], miter_limit),
            ClipperFillRule_POSITIVE,
            true,
        );
        holes.append(&mut part);
    }
    #[cfg(debug_assertions)]
    for c in &holes {
        debug_assert!(path64_to_polygon(c).area() > 0.0);
    }

    // ClipperUtils.cpp:1329-1339 — 3) Subtract holes from the contours.
    let output: Paths64 = if holes.is_empty() {
        contours
    } else {
        unsafe { clipper64_difference_paths(&contours, &holes) }
    };

    output.iter().map(path64_to_polygon).collect()
}

/// ClipperUtils.cpp:1344 — variable_offset_outer_ex
/// hpp:670 — ExPolygons variable_offset_outer_ex(const ExPolygon&, const std::vector<std::vector<float>>&, double miter_limit)
pub fn variable_offset_outer_ex(
    expoly: &ExPolygon,
    deltas: &[Vec<f32>],
    miter_limit: f64,
) -> ExPolygons {
    #[cfg(debug_assertions)]
    {
        // ClipperUtils.cpp:1346-1351 — deltas are all non negative.
        for ds in deltas {
            for &delta in ds {
                debug_assert!(delta >= 0.0);
            }
        }
        debug_assert_eq!(expoly.holes.len() + 1, deltas.len());
    }

    // ClipperUtils.cpp:1354-1355 — 1) Offset the outer contour.
    let contours = fix_after_outer_offset(
        &mittered_offset_path_scaled(&expoly.contour.points, &deltas[0], miter_limit),
        ClipperFillRule_POSITIVE,
        false,
    );
    #[cfg(debug_assertions)]
    for c in &contours {
        debug_assert!(path64_to_polygon(c).area() > 0.0);
    }

    // ClipperUtils.cpp:1361-1365 — 2) Offset the holes one by one, collect the results.
    let mut holes: Paths64 = Vec::new();
    holes.reserve(expoly.holes.len());
    for (h, hole) in expoly.holes.iter().enumerate() {
        let mut part = fix_after_inner_offset(
            &mittered_offset_path_scaled(&hole.points, &deltas[1 + h], miter_limit),
            ClipperFillRule_POSITIVE,
            true,
        );
        holes.append(&mut part);
    }
    #[cfg(debug_assertions)]
    for c in &holes {
        debug_assert!(path64_to_polygon(c).area() > 0.0);
    }

    // ClipperUtils.cpp:1371-1384 — 3) Subtract holes from the contours.
    if holes.is_empty() {
        // output.emplace_back(std::move(path)) — each contour Path becomes an ExPolygon.
        let mut output: ExPolygons = Vec::with_capacity(contours.len());
        for path in &contours {
            output.push(ExPolygon::new(path64_to_polygon(path)));
        }
        output
    } else {
        unsafe { clipper64_difference_tree_to_expolygons(&contours, &holes) }
    }
}

/// ClipperUtils.cpp:1390 — variable_offset_inner_ex
/// hpp:671 — ExPolygons variable_offset_inner_ex(const ExPolygon&, const std::vector<std::vector<float>>&, double miter_limit)
pub fn variable_offset_inner_ex(
    expoly: &ExPolygon,
    deltas: &[Vec<f32>],
    miter_limit: f64,
) -> ExPolygons {
    #[cfg(debug_assertions)]
    {
        // ClipperUtils.cpp:1392-1397 — deltas are all non positive.
        for ds in deltas {
            for &delta in ds {
                debug_assert!(delta <= 0.0);
            }
        }
        debug_assert_eq!(expoly.holes.len() + 1, deltas.len());
    }

    // ClipperUtils.cpp:1400-1401 — 1) Offset the outer contour.
    let contours = fix_after_inner_offset(
        &mittered_offset_path_scaled(&expoly.contour.points, &deltas[0], miter_limit),
        ClipperFillRule_NEGATIVE,
        true,
    );
    #[cfg(debug_assertions)]
    for c in &contours {
        debug_assert!(path64_to_polygon(c).area() > 0.0);
    }

    // ClipperUtils.cpp:1407-1411 — 2) Offset the holes one by one, collect the results.
    let mut holes: Paths64 = Vec::new();
    holes.reserve(expoly.holes.len());
    for (h, hole) in expoly.holes.iter().enumerate() {
        let mut part = fix_after_outer_offset(
            &mittered_offset_path_scaled(&hole.points, &deltas[1 + h], miter_limit),
            ClipperFillRule_NEGATIVE,
            false,
        );
        holes.append(&mut part);
    }
    #[cfg(debug_assertions)]
    for c in &holes {
        debug_assert!(path64_to_polygon(c).area() > 0.0);
    }

    // ClipperUtils.cpp:1417-1429 — 3) Subtract holes from the contours.
    if holes.is_empty() {
        let mut output: ExPolygons = Vec::with_capacity(contours.len());
        for path in &contours {
            output.push(ExPolygon::new(path64_to_polygon(path)));
        }
        output
    } else {
        unsafe { clipper64_difference_tree_to_expolygons(&contours, &holes) }
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scale;

    /// Clipper2Utils.cpp:38
    #[test]
    fn test_path64_to_points() {
        let path: Path64 = vec![(0, 0), (100, 0), (100, 100)];
        let points = path64_to_points(&path);
        assert_eq!(points.len(), 3);
        assert_eq!(points[0], Point::new(0, 0));
        assert_eq!(points[1], Point::new(100, 0));
        assert_eq!(points[2], Point::new(100, 100));
    }

    /// Clipper2Utils.cpp:8
    #[test]
    fn test_paths64_to_polylines() {
        let paths: Paths64 = vec![vec![(0, 0), (100, 0)]];
        let pls = paths64_to_polylines(&paths);
        assert_eq!(pls.len(), 1);
        assert_eq!(pls[0].points.len(), 2);
    }

    /// Clipper2Utils.cpp:90
    #[test]
    fn test_slic3r_polygons_to_paths64() {
        let poly = Polygon::from_points(vec![Point::new(0, 0), Point::new(100, 0), Point::new(100, 100)]);
        let paths = slic3r_polygons_to_paths64(&[poly]);
        assert_eq!(paths.len(), 1);
        assert_eq!(paths[0].len(), 3);
    }

    /// Clipper2Utils.cpp:103
    #[test]
    fn test_slic3r_expolygons_to_paths64() {
        let expoly = ExPolygon::with_holes(
            Polygon::from_points(vec![Point::new(0, 0), Point::new(100, 0), Point::new(100, 100)]),
            vec![Polygon::from_points(vec![
                Point::new(10, 10),
                Point::new(20, 10),
                Point::new(20, 20),
            ])],
        );
        let paths = slic3r_expolygons_to_paths64(&vec![expoly]);
        assert_eq!(paths.len(), 2); // contour + 1 hole
    }

    /// Union should produce results (Clipper2Utils.cpp:143).
    #[test]
    fn test_union_operations() {
        let polygons = vec![
            Polygon::from_points(vec![
                Point::new(0, 0),
                Point::new(scale(10.0), 0),
                Point::new(scale(10.0), scale(10.0)),
                Point::new(0, scale(10.0)),
            ]),
            Polygon::from_points(vec![
                Point::new(scale(5.0), scale(5.0)),
                Point::new(scale(15.0), scale(5.0)),
                Point::new(scale(15.0), scale(15.0)),
                Point::new(scale(5.0), scale(15.0)),
            ]),
        ];

        let result = union_ex_2(&polygons);
        assert!(!result.is_empty(), "Union should produce results");
        assert!(
            result[0].contour.points.len() >= 4,
            "Union result should have at least 4 points"
        );
    }

    /// Union of a square containing a hole should reconstruct the hole
    /// (PolyTreeToExPolygons faithfulness, Clipper2Utils.cpp:46).
    #[test]
    fn test_union_reconstructs_holes() {
        // Outer square 0..100, inner hole 30..70 (wound oppositely so NonZero
        // union yields an annulus).
        let outer = vec![
            Point::new(0, 0),
            Point::new(scale(100.0), 0),
            Point::new(scale(100.0), scale(100.0)),
            Point::new(0, scale(100.0)),
        ];
        let hole = vec![
            Point::new(scale(30.0), scale(30.0)),
            Point::new(scale(30.0), scale(70.0)),
            Point::new(scale(70.0), scale(70.0)),
            Point::new(scale(70.0), scale(30.0)),
        ];
        let expoly = ExPolygon::with_holes(
            Polygon::from_points(outer),
            vec![Polygon::from_points(hole)],
        );
        let result = union_ex_2_expolygons(&vec![expoly]);
        assert_eq!(result.len(), 1, "Annulus should be a single ExPolygon");
        assert_eq!(result[0].holes.len(), 1, "Hole must be reconstructed");
    }

    /// Offset should produce results (Clipper2Utils.cpp:174, 186).
    #[test]
    fn test_offset_operations() {
        let expoly = ExPolygon::with_holes(
            Polygon::from_points(vec![
                Point::new(0, 0),
                Point::new(scale(10.0), 0),
                Point::new(scale(10.0), scale(10.0)),
                Point::new(0, scale(10.0)),
            ]),
            vec![],
        );

        let result = offset_ex_2(&vec![expoly.clone()], scale(1.0) as f64);
        assert!(!result.is_empty(), "Offset should produce results");

        let result2 = offset2_ex_2(&vec![expoly], scale(1.0) as f64, scale(-0.5) as f64);
        assert!(!result2.is_empty(), "Offset2 should produce results");
    }
}
