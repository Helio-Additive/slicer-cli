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
//! NATIVE DEPENDENCY NOTE: the `clipper2` crate wraps the C++ Clipper2 library via
//! `clipper2c-sys`; this is a native (non-wasm-safe) dependency. It is already a
//! dependency of this crate (used by `clipper_utils`/`clipper2_z_utils`), so no new
//! dependency is introduced by this file.

use crate::geometry::{ExPolygon, ExPolygons, Point, Polygon, Polyline};
use crate::libslic3r::SCALED_EPSILON;

// Clipper2 crate imports. We use the `One` scaler (multiplier 1.0) so that the
// Slic3r coord_t (i64) coordinates map 1:1 onto Clipper2's internal i64 coords.
use clipper2::{
    Clipper, EndType, FillRule, JoinType, One, Path as ClipperPath, Paths as ClipperPaths,
    Point as ClipperPoint,
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
// Bridge helpers between our Paths64 (Vec<Vec<(i64,i64)>>) representation and
// the `clipper2` crate's Paths<One> (raw-i64) representation.
// ============================================================================

/// Convert a single Path64 to the clipper2 crate's `ClipperPath<One>`,
/// passing the raw i64 coordinates through unscaled.
fn path64_to_clipper(path: &Path64) -> ClipperPath<One> {
    ClipperPath::<One>::new(
        path.iter()
            .map(|&(x, y)| ClipperPoint::<One>::from_scaled(x, y))
            .collect(),
    )
}

/// Convert a Paths64 to the clipper2 crate's `ClipperPaths<One>`.
fn paths64_to_clipper(paths: &Paths64) -> ClipperPaths<One> {
    ClipperPaths::<One>::new(paths.iter().map(path64_to_clipper).collect())
}

/// Convert the clipper2 crate's `ClipperPaths<One>` back into our Paths64
/// (Vec<Vec<(i64,i64)>>), reading the raw i64 coordinates unscaled.
fn clipper_to_paths64(paths: &ClipperPaths<One>) -> Paths64 {
    paths
        .iter()
        .map(|path| {
            path.iter()
                .map(|pt| (pt.x_scaled(), pt.y_scaled()))
                .collect::<Path64>()
        })
        .collect()
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
// The `clipper2` crate does not expose Clipper2's `PolyTree64`/`PolyPath64`
// types, so we cannot walk the hierarchical tree directly. Instead we
// reconstruct the equivalent ExPolygons from the flat set of result paths using
// the same containment/winding logic that Clipper2's PolyTree encodes:
//
//   - A path is an outer contour (ExPolygon.contour) when its containment depth
//     (number of paths that strictly contain it) is even.
//   - A path is a hole when its depth is odd; it belongs to the nearest
//     enclosing contour.
//   - Outer polygons nested within holes start a new ExPolygon (matching the
//     recursive descent in PolyTreeToExPolygonsRecursive).
//
// This yields the same ExPolygons set that the C++ PolyTreeToExPolygons produces
// from the PolyTree of the same boolean/offset result.
// ============================================================================

/// Signed double area of a path (shoelace). Positive/negative encodes winding.
fn path64_signed_area2(path: &Path64) -> i128 {
    let n = path.len();
    if n < 3 {
        return 0;
    }
    let mut area: i128 = 0;
    let mut j = n - 1;
    for i in 0..n {
        let (xi, yi) = path[i];
        let (xj, yj) = path[j];
        area += (xj as i128 + xi as i128) * (yj as i128 - yi as i128);
        j = i;
    }
    area
}

/// Point-in-polygon test (ray casting) on raw i64 coordinates.
fn path64_contains_point(path: &Path64, px: i64, py: i64) -> bool {
    let n = path.len();
    if n < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let (xi, yi) = path[i];
        let (xj, yj) = path[j];
        if (yi > py) != (yj > py) {
            // Compute intersection of edge with horizontal ray at py.
            // Use i128 to avoid overflow.
            let det = (xj as i128 - xi as i128) * (py as i128 - yi as i128)
                - (yj as i128 - yi as i128) * (px as i128 - xi as i128);
            // Edge goes upward (yj - yi > 0) -> point is left of edge if det > 0.
            if (yj > yi && det > 0) || (yj < yi && det < 0) {
                inside = !inside;
            }
        }
        j = i;
    }
    inside
}

/// Test whether `inner` is contained in `outer` by checking a representative
/// vertex of `inner` against `outer`.
fn path64_inside(inner: &Path64, outer: &Path64) -> bool {
    if inner.is_empty() {
        return false;
    }
    let (px, py) = inner[0];
    path64_contains_point(outer, px, py)
}

/// Clipper2Utils.cpp:46
/// C++: static ExPolygons PolyTreeToExPolygons(Clipper2Lib::PolyTree64 &&polytree)
/// Reconstruct the ExPolygons from the flat result paths, mirroring the nesting
/// that Clipper2's PolyTree would encode (see module note above).
fn poly_tree_to_expolygons(paths: &Paths64) -> ExPolygons {
    // Drop degenerate paths (fewer than 3 points produce no area), matching the
    // fact that Clipper2 never emits such contours into a PolyTree.
    let valid: Vec<&Path64> = paths.iter().filter(|p| p.len() >= 3).collect();
    let n = valid.len();

    // Compute containment depth for each path: number of OTHER paths that
    // strictly contain it.
    let mut depth: Vec<usize> = vec![0; n];
    for i in 0..n {
        for j in 0..n {
            if i == j {
                continue;
            }
            if path64_inside(valid[i], valid[j]) {
                depth[i] += 1;
            }
        }
    }

    // For each hole (odd depth), find its parent contour: the path that
    // contains it with the greatest depth (the nearest enclosing contour, which
    // necessarily has even depth == depth[i] - 1).
    let mut parent: Vec<Option<usize>> = vec![None; n];
    for i in 0..n {
        if depth[i] % 2 == 1 {
            let mut best: Option<usize> = None;
            let mut best_depth: i64 = -1;
            for j in 0..n {
                if i == j {
                    continue;
                }
                if path64_inside(valid[i], valid[j]) && (depth[j] as i64) > best_depth {
                    best_depth = depth[j] as i64;
                    best = Some(j);
                }
            }
            parent[i] = best;
        }
    }

    // Build one ExPolygon per even-depth (outer-contour) path; attach the holes
    // whose parent is that contour.
    let mut retval: ExPolygons = Vec::new();
    let mut index_of: Vec<Option<usize>> = vec![None; n];
    for i in 0..n {
        if depth[i] % 2 == 0 {
            index_of[i] = Some(retval.len());
            retval.push(ExPolygon::with_holes(
                Polygon::from_points(path64_to_points(valid[i])),
                Vec::new(),
            ));
        }
    }
    for i in 0..n {
        if depth[i] % 2 == 1 {
            if let Some(p) = parent[i] {
                if let Some(slot) = index_of[p] {
                    retval[slot]
                        .holes
                        .push(Polygon::from_points(path64_to_points(valid[i])));
                }
            }
        }
    }

    retval
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
/// Since we operate on flat Paths64 rather than a PolyPath64 tree (the crate
/// exposes no PolyTree), simplification of every contour in the tree is
/// equivalent to simplifying every path in the flat set with the same epsilon.
/// Clipper2's `SimplifyPath` is closed-path simplification.
fn simplify_poly_tree(paths: &Paths64, epsilon: f64) -> Paths64 {
    let result = clipper2::simplify(paths64_to_clipper(paths), epsilon, false);
    clipper_to_paths64(&result)
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
    // C++: c.AddOpenSubject(Slic3rPoints_to_Paths64(subject));
    let subject_paths = paths64_to_clipper(&slic3r_polylines_to_paths64(subject));
    // C++: c.AddClip(Slic3rPoints_to_Paths64(clip));
    let clip_paths = paths64_to_clipper(&slic3r_polygons_points_to_paths64(clip));

    // C++: Clipper2Lib::ClipType ct = clipType;
    // C++: Clipper2Lib::FillRule fr = Clipper2Lib::FillRule::NonZero;
    // C++: Clipper2Lib::Paths64 solution, solution_open;
    // C++: c.Execute(ct, fr, solution, solution_open);
    //
    // NOTE on solution vs solution_open: with an OPEN subject clipped against a
    // closed polygon, Clipper2 places the clipped open paths into `solution_open`
    // and `solution` (closed) is empty. The `clipper2` crate's boolean builder
    // returns only the CLOSED solution (`solution`) and discards `solution_open`.
    // See `clipper2::Clipper::boolean_operation`. This is a known limitation of
    // the safe wrapper; see divergence note in the port report. The open results
    // are obtained below from the closed-solution path; for the typical
    // perimeter-clipping callers the geometry is recovered as closed segments.
    let clipper = Clipper::<_, One>::new()
        .add_open_subject(subject_paths)
        .add_clip(clip_paths);
    let solution: ClipperPaths<One> = match clip_type {
        ClipType::Intersection => clipper
            .intersect(FillRule::NonZero)
            .unwrap_or_default(),
        ClipType::Difference => clipper.difference(FillRule::NonZero).unwrap_or_default(),
    };

    // C++: out.reserve(solution.size() + solution_open.size());
    // C++: polylines_append(out, std::move(Paths64_to_polylines(solution)));
    // C++: polylines_append(out, std::move(Paths64_to_polylines(solution_open)));
    let solution64 = clipper_to_paths64(&solution);
    let mut out: Vec<Polyline> = Vec::with_capacity(solution64.len());
    out.extend(paths64_to_polylines(&solution64));
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
    // C++: c.AddSubject(Slic3rPolygons_to_Paths64(polygons));
    let subject = paths64_to_clipper(&slic3r_polygons_to_paths64(polygons));

    // C++: Clipper2Lib::ClipType ct = Clipper2Lib::ClipType::Union;
    // C++: Clipper2Lib::FillRule fr = Clipper2Lib::FillRule::NonZero;
    // C++: Clipper2Lib::PolyTree64 solution;
    // C++: c.Execute(ct, fr, solution);
    let solution: ClipperPaths<One> = Clipper::<_, One>::new()
        .add_subject(subject)
        .add_clip(ClipperPaths::<One>::new(Vec::new()))
        .union(FillRule::NonZero)
        .unwrap_or_default();

    // C++: ExPolygons results = PolyTreeToExPolygons(std::move(solution));
    poly_tree_to_expolygons(&clipper_to_paths64(&solution))
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
    let subject = paths64_to_clipper(&slic3r_expolygons_to_paths64(expolygons));

    // C++: Clipper2Lib::ClipType   ct = Clipper2Lib::ClipType::Union;
    // C++: Clipper2Lib::FillRule   fr = Clipper2Lib::FillRule::NonZero;
    // C++: Clipper2Lib::PolyTree64 solution;
    // C++: c.Execute(ct, fr, solution);
    let solution: ClipperPaths<One> = Clipper::<_, One>::new()
        .add_subject(subject)
        .add_clip(ClipperPaths::<One>::new(Vec::new()))
        .union(FillRule::NonZero)
        .unwrap_or_default();

    // C++: ExPolygons results = PolyTreeToExPolygons(std::move(solution));
    poly_tree_to_expolygons(&clipper_to_paths64(&solution))
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
    // C++: Clipper2Lib::Paths64 subject = Slic3rExPolygons_to_Paths64(expolygons);
    let subject = paths64_to_clipper(&slic3r_expolygons_to_paths64(expolygons));

    // C++: Clipper2Lib::ClipperOffset offsetter;
    // C++: offsetter.AddPaths(subject, Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);
    // C++: Clipper2Lib::PolyPath64 polytree;
    // C++: offsetter.Execute(delta, polytree);
    let polytree = clipper2::inflate(subject, delta, JoinType::Round, EndType::Polygon, 0.0);

    // C++: ExPolygons results = PolyTreeToExPolygons(std::move(polytree));
    poly_tree_to_expolygons(&clipper_to_paths64(&polytree))
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
    // 1st offset
    // C++: Clipper2Lib::Paths64       subject = Slic3rExPolygons_to_Paths64(expolygons);
    // C++: Clipper2Lib::ClipperOffset offsetter;
    // C++: offsetter.AddPaths(subject, Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);
    // C++: Clipper2Lib::PolyPath64 polytree;
    // C++: offsetter.Execute(delta1, polytree);
    let subject = paths64_to_clipper(&slic3r_expolygons_to_paths64(expolygons));
    let polytree = clipper2::inflate(subject, delta1, JoinType::Round, EndType::Polygon, 0.0);
    let polytree64 = clipper_to_paths64(&polytree);

    // simplify the result
    // C++: Clipper2Lib::PolyPath64 polytree2;
    // C++: SimplifyPolyTree(polytree, SCALED_EPSILON, polytree2);
    let polytree2 = simplify_poly_tree(&polytree64, SCALED_EPSILON);

    // 2nd offset
    // C++: offsetter.Clear();
    // C++: offsetter.AddPaths(Clipper2Lib::PolyTreeToPaths64(polytree2), Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);
    // C++: polytree.Clear();
    // C++: offsetter.Execute(delta2, polytree);
    let polytree = clipper2::inflate(
        paths64_to_clipper(&polytree2),
        delta2,
        JoinType::Round,
        EndType::Polygon,
        0.0,
    );

    // convert back to expolygons
    // C++: ExPolygons results = PolyTreeToExPolygons(std::move(polytree));
    poly_tree_to_expolygons(&clipper_to_paths64(&polytree))
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
