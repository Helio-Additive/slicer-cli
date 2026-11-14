//! Clipper2 polygon boolean operations and utilities.
//!
//! Faithful 1:1 port of `Clipper2Utils.cpp` / `Clipper2Utils.hpp` from BambuStudio.
//!
//! C++ Reference: `src/libslic3r/Clipper2Utils.cpp`, `src/libslic3r/Clipper2Utils.hpp`
//!
//! Every function here runs on `crate::clipper2`, the Rust port of the Clipper2 the engine
//! pins (1.5.2), called the way Clipper2Utils.cpp calls the library: the coordinates are
//! the Slic3r `coord_t` (i64), unscaled, as `Clipper2Lib::Paths64` has them.
//! `tests/clipper2_utils_native_parity.rs` holds the functions equal to the calls of
//! Clipper2Utils.cpp made in C++ on the pin's Clipper2.

use crate::geometry::{cross2f, ExPolygon, ExPolygons, Point, PointF, Polygon, Polygons, Polyline};
use crate::libslic3r::SCALED_EPSILON;

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
// `crate::clipper2` (`namespace Clipper2Lib` of the pin).
// ============================================================================

use crate::clipper2 as c2;

/// This file's `Paths64` (pairs) as `Clipper2Lib::Paths64`.
fn paths64_to_c2(paths: &Paths64) -> c2::Paths64 {
    paths.iter().map(|path| path.iter().map(|&(x, y)| c2::Point64::new(x, y)).collect()).collect()
}

/// `Clipper2Lib::Paths64` as this file's `Paths64` (pairs).
fn c2_to_paths64(paths: &c2::Paths64) -> Paths64 {
    paths.iter().map(|path| path.iter().map(|p| (p.x, p.y)).collect()).collect()
}

/// Clipper2Utils.cpp:38-44
/// C++: Points Path64ToPoints(const Clipper2Lib::Path64& path64)
fn path64_to_points_c2(path64: &c2::Path64) -> Vec<Point> {
    let mut points: Vec<Point> = Vec::with_capacity(path64.len());
    for point64 in path64 {
        points.push(Point::new(point64.x, point64.y));
    }
    points
}

/// Clipper2Utils.cpp:8-21
/// C++: Slic3r::Polylines Paths64_to_polylines(const Clipper2Lib::Paths64& in)
fn paths64_to_polylines_c2(in_paths: &c2::Paths64) -> Vec<Polyline> {
    let mut out: Vec<Polyline> = Vec::with_capacity(in_paths.len());
    for path64 in in_paths {
        out.push(Polyline::from_points(path64_to_points_c2(path64)));
    }
    out
}

/// Clipper2Utils.cpp:64-71 (Inner::PolyTreeCountExPolygons)
/// C++: static size_t PolyTreeCountExPolygons(const Clipper2Lib::PolyPath64& polynode)
fn poly_tree_count_expolygons_c2(polynode: &c2::PolyPath64) -> usize {
    let mut cnt: usize = 1;
    for i in 0..polynode.count() {
        for j in 0..polynode.child(i).count() {
            cnt += poly_tree_count_expolygons_c2(polynode.child(i).child(j));
        }
    }
    cnt
}

/// Clipper2Utils.cpp:50-62 (Inner::PolyTreeToExPolygonsRecursive)
/// C++: static void PolyTreeToExPolygonsRecursive(Clipper2Lib::PolyTree64 &&polynode, ExPolygons *expolygons)
fn poly_tree_to_expolygons_recursive_c2(polynode: &c2::PolyPath64, expolygons: &mut ExPolygons) {
    let cnt = expolygons.len();
    expolygons.push(ExPolygon::empty());
    expolygons[cnt].contour = Polygon::from_points(path64_to_points_c2(polynode.polygon()));

    expolygons[cnt].holes = vec![Polygon::default(); polynode.count()];
    for i in 0..polynode.count() {
        expolygons[cnt].holes[i] = Polygon::from_points(path64_to_points_c2(polynode.child(i).polygon()));
        // Add outer polygons contained by (nested within) holes.
        for j in 0..polynode.child(i).count() {
            poly_tree_to_expolygons_recursive_c2(polynode.child(i).child(j), expolygons);
        }
    }
}

/// Clipper2Utils.cpp:46-80
/// C++: static ExPolygons PolyTreeToExPolygons(Clipper2Lib::PolyTree64 &&polytree)
fn poly_tree_to_expolygons_c2(polytree: &c2::PolyTree64) -> ExPolygons {
    let mut retval: ExPolygons = Vec::new();
    let mut cnt: usize = 0;
    for i in 0..polytree.count() {
        cnt += poly_tree_count_expolygons_c2(polytree.child(i));
    }
    retval.reserve(cnt);
    for i in 0..polytree.count() {
        poly_tree_to_expolygons_recursive_c2(polytree.child(i), &mut retval);
    }
    retval
}

/// Clipper2Utils.cpp:82-88
/// C++: void SimplifyPolyTree(const Clipper2Lib::PolyPath64 &polytree, double epsilon, Clipper2Lib::PolyPath64 &result)
fn simplify_poly_tree_c2(polytree: &c2::PolyPath64, epsilon: f64, result: &mut c2::PolyPath64) {
    for child in polytree.iter() {
        // `Clipper2Lib::SimplifyPath(child->Polygon(), epsilon)`: isClosedPath = true.
        let newchild = result.add_child(&c2::simplify_path(child.polygon(), epsilon, true));
        simplify_poly_tree_c2(child, epsilon, newchild);
    }
}

/// The body of the two `union_ex_2` (Clipper2Utils.cpp:143-156 and 158-172 from `c.AddSubject`):
/// C++: Clipper2Lib::Clipper64 c; c.AddSubject(subject);
/// C++: c.Execute(Clipper2Lib::ClipType::Union, Clipper2Lib::FillRule::NonZero, solution);
/// C++: ExPolygons results = PolyTreeToExPolygons(std::move(solution));
fn union_ex_2_c2(subject: &Paths64) -> ExPolygons {
    let mut c = c2::Clipper64::new();
    c.add_subject(&paths64_to_c2(subject));

    let ct = c2::ClipType::Union;
    let fr = c2::FillRule::NonZero;
    let mut solution = c2::PolyTree64::new();
    c.execute_tree(ct, fr, &mut solution);

    poly_tree_to_expolygons_c2(&solution)
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
    // Clipper2Utils.cpp:119-136, on `crate::clipper2`.
    let mut c = c2::Clipper64::new();
    c.add_open_subject(&paths64_to_c2(&slic3r_polylines_to_paths64(subject)));
    c.add_clip(&paths64_to_c2(&slic3r_polygons_points_to_paths64(clip)));

    let ct = match clip_type {
        ClipType::Intersection => c2::ClipType::Intersection,
        ClipType::Difference => c2::ClipType::Difference,
    };
    let fr = c2::FillRule::NonZero;
    let mut solution = c2::Paths64::new();
    let mut solution_open = c2::Paths64::new();
    c.execute_open(ct, fr, &mut solution, &mut solution_open);

    let mut out: Vec<Polyline> = Vec::with_capacity(solution.len() + solution_open.len());
    out.extend(paths64_to_polylines_c2(&solution));
    out.extend(paths64_to_polylines_c2(&solution_open));

    out
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
    union_ex_2_c2(&slic3r_polygons_to_paths64(polygons))
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
    union_ex_2_c2(&slic3r_expolygons_to_paths64(expolygons))
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
    // Clipper2Utils.cpp:174-184, on `crate::clipper2`.
    let subject = paths64_to_c2(&slic3r_expolygons_to_paths64(expolygons));
    let mut offsetter = c2::ClipperOffset::default();
    offsetter.add_paths(&subject, c2::JoinType::Round, c2::EndType::Polygon);
    let mut polytree = c2::PolyPath64::new();
    offsetter.execute_tree(delta, &mut polytree);
    poly_tree_to_expolygons_c2(&polytree)
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
    // Clipper2Utils.cpp:186-208, on `crate::clipper2`.
    // 1st offset
    let subject = paths64_to_c2(&slic3r_expolygons_to_paths64(expolygons));
    let mut offsetter = c2::ClipperOffset::default();
    offsetter.add_paths(&subject, c2::JoinType::Round, c2::EndType::Polygon);
    let mut polytree = c2::PolyPath64::new();
    offsetter.execute_tree(delta1, &mut polytree);

    // simplify the result
    let mut polytree2 = c2::PolyPath64::new();
    simplify_poly_tree_c2(&polytree, SCALED_EPSILON, &mut polytree2);

    // 2nd offset
    offsetter.clear();
    offsetter.add_paths(&c2::poly_tree_to_paths64(&polytree2), c2::JoinType::Round, c2::EndType::Polygon);
    polytree.clear();
    offsetter.execute_tree(delta2, &mut polytree);

    // convert back to expolygons
    poly_tree_to_expolygons_c2(&polytree)
}

// ============================================================================
// Variable offset (ClipperUtils.cpp). These four `variable_offset_*` entry
// points and their helpers (`mittered_offset_path_scaled`,
// `fix_after_inner_offset`, `fix_after_outer_offset`) live in
// `src/libslic3r/ClipperUtils.cpp` upstream, but they are implemented here, on
// `crate::clipper2`.
//
// BACKEND NOTE: upstream `ClipperUtils.cpp` uses the legacy Clipper1
// (`ClipperLib::Clipper`) with `ReverseSolution()` + `pftNegative`/`pftPositive`
// fill types. Here the same calls are made on Clipper2, which has the same
// winding-rule fill modes (`Positive`/`Negative`/`NonZero`) and
// `set_reverse_solution`. `ClipperLib::Area(path) > 0` assertions map onto
// `Polygon::area() > 0`.
// ============================================================================

/// Internal clipper offset shortest-edge factor.
/// ClipperUtils.hpp:44 — `static constexpr const double ClipperOffsetShortestEdgeFactor = 0.005;`
const CLIPPER_OFFSET_SHORTEST_EDGE_FACTOR: f64 = 0.005;

// Cast a scaled integer Point to a double vector (Eigen `.cast<double>()`).
// FIDELITY-NOTE(F2): C++ `contour[i].cast<double>()` casts an `int32_t` coord
// (libslic3r `coord_t`) to double; the crate-wide `Coord` is i64. For the
// in-range scaled slicer coordinates these agree exactly, so the i64->f64 cast
// here is faithful — not narrowing per-file per the F2 directive.
#[inline]
fn pt_to_d_2(p: Point) -> PointF {
    PointF::new(p.x as f64, p.y as f64)
}

/// Run a `Clipper64` UNION over a single closed subject path into flat Paths64,
/// with the given fill rule and `reverse_solution` flag. Mirrors the
/// `ClipperLib::Clipper` usage in `fix_after_outer_offset` / `fix_after_inner_offset`
/// (`c.AddPath(...); c.ReverseSolution(rev); c.Execute(ctUnion, solution, fill, fill)`).
fn clipper64_union_paths(subject: &Paths64, fillrule: c2::FillRule, reverse_solution: bool) -> Paths64 {
    let mut c = c2::Clipper64::new();
    c.add_subject(&paths64_to_c2(subject));
    c.set_reverse_solution(reverse_solution);
    let mut solution = c2::Paths64::new();
    let mut open = c2::Paths64::new();
    c.execute_open(c2::ClipType::Union, fillrule, &mut solution, &mut open);
    c2_to_paths64(&solution)
}

/// Run a `Clipper64` DIFFERENCE (subject minus clip) into a PolyTree64 and
/// reconstruct ExPolygons. Mirrors the `ClipperLib::Clipper` difference into a
/// `PolyTree` used by `variable_offset_inner_ex` / `variable_offset_outer_ex`.
fn clipper64_difference_tree_to_expolygons(subject: &Paths64, clip: &Paths64) -> ExPolygons {
    let mut c = c2::Clipper64::new();
    c.add_subject(&paths64_to_c2(subject));
    c.add_clip(&paths64_to_c2(clip));
    let mut tree = c2::PolyTree64::new();
    let mut open = c2::Paths64::new();
    c.execute_tree_open(c2::ClipType::Difference, c2::FillRule::NonZero, &mut tree, &mut open);
    poly_tree_to_expolygons_c2(&tree)
}

/// Run a `Clipper64` DIFFERENCE (subject minus clip) into flat Paths64.
/// Mirrors the `ClipperLib::Clipper` difference into `Paths` used by
/// `variable_offset_inner` / `variable_offset_outer`.
fn clipper64_difference_paths(subject: &Paths64, clip: &Paths64) -> Paths64 {
    let mut c = c2::Clipper64::new();
    c.add_subject(&paths64_to_c2(subject));
    c.add_clip(&paths64_to_c2(clip));
    let mut solution = c2::Paths64::new();
    let mut open = c2::Paths64::new();
    c.execute_open(c2::ClipType::Difference, c2::FillRule::NonZero, &mut solution, &mut open);
    c2_to_paths64(&solution)
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
fn fix_after_outer_offset(input: &Path64, filltype: c2::FillRule, reverse_result: bool) -> Paths64 {
    // ClipperUtils.cpp:1084-1090
    let mut solution: Paths64 = Vec::new();
    if !input.is_empty() {
        // c.AddPath(input, ptSubject, true);
        // c.ReverseSolution(reverse_result);
        // c.Execute(ctUnion, solution, filltype, filltype);
        solution = clipper64_union_paths(&vec![input.clone()], filltype, reverse_result);
    }
    solution
}

/// ClipperUtils.cpp:1095 — fix_after_inner_offset
/// Inner offset may split the source contour into multiple contours, but one
/// resulting contour shall not lie inside the other.
/// Defaults: filltype = pftNegative, reverse_result = true.
fn fix_after_inner_offset(input: &Path64, filltype: c2::FillRule, reverse_result: bool) -> Paths64 {
    // ClipperUtils.cpp:1102-1116
    let mut solution: Paths64 = Vec::new();
    if !input.is_empty() {
        // ClipperLib::IntRect r = clipper.GetBounds();
        // GetBounds() returns the bounding box of all added subject paths; at this
        // point only `input` is added, so the bounds are the min/max over its
        // points — computed directly.
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
        let rect: Path64 = if filltype == c2::FillRule::Positive {
            vec![(left, bottom), (left, top), (right, top), (right, bottom)]
        } else {
            vec![(left, bottom), (right, bottom), (right, top), (left, top)]
        };
        // c.AddPath(input, ...); c.AddPath(rect, ...);
        // c.ReverseSolution(reverse_result);
        // c.Execute(ctUnion, solution, filltype, filltype);
        let subject: Paths64 = vec![input.clone(), rect];
        solution = clipper64_union_paths(&subject, filltype, reverse_result);
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
        // FIDELITY-NOTE(F2): C++ defines CLIPPERLIB_INT32 (clipper.hpp:83), so
        // `ClipperLib::cInt` is `int32_t` (clipper.hpp:88) — the offset coordinates
        // are truncated to int32 here. Reproduce that truncation locally (`as i32`)
        // even though the crate-wide `Coord` is i64, then widen back to the i64
        // path-coordinate type. For in-range scaled coordinates these agree; this
        // preserves the exact int32 wrap/truncation C++ produces.
        let add_offset_point = |out: &mut Path64, mut pt: PointF| {
            pt = pt
                + PointF::new(
                    0.5 - ((pt.x < 0.0) as i32 as f64),
                    0.5 - ((pt.y < 0.0) as i32 as f64),
                );
            out.push((pt.x as i32 as i64, pt.y as i32 as i64));
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
        c2::FillRule::Negative,
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
            c2::FillRule::Negative,
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
        clipper64_difference_paths(&contours, &holes)
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
        c2::FillRule::Positive,
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
            c2::FillRule::Positive,
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
        clipper64_difference_paths(&contours, &holes)
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
        c2::FillRule::Positive,
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
            c2::FillRule::Positive,
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
        clipper64_difference_tree_to_expolygons(&contours, &holes)
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
        c2::FillRule::Negative,
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
            c2::FillRule::Negative,
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
        clipper64_difference_tree_to_expolygons(&contours, &holes)
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
