//! Clipper2 polygon boolean operations and utilities
//!
//! This module provides polygon boolean operations and offset operations using
//! the Clipper2 library, which is the successor to the original Clipper library.
//!
//! C++ Reference: `Clipper2Utils.hpp`, `Clipper2Utils.cpp`
//!
//! ## Key Operations
//!
//! - **Boolean operations**: union, intersection, difference
//! - **Offset operations**: inflate/deflate polygons with various join types
//! - **Offset2**: two-stage offset with simplification between stages
//!
//! ## Architecture
//!
//! The module provides:
//! 1. Conversion functions (Slic3r types ↔ Clipper2 types)
//! 2. Boolean operations on polylines and polygons
//! 3. Offset operations with join/end type control
//! 4. PolyTree simplification for multi-stage operations

use crate::geometry::{ExPolygon, ExPolygons, Point, Polygon, Polyline};
use crate::libslic3r::SCALED_EPSILON;
use crate::{scale, unscale};

// Clipper2 crate imports
use clipper2::{Centi, EndType, FillRule, JoinType, Paths as ClipperPaths};

// ============================================================================
// Type Aliases (matching C++ Clipper2 usage)
// ============================================================================

/// Clipper2 point type (64-bit integer coordinates)
/// Clipper2Utils.cpp:8 (implicit from Clipper2Lib::Point64)
pub type Point64 = (i64, i64);

/// Clipper2 path type (single contour)
/// Clipper2Utils.cpp:8 (implicit from Clipper2Lib::Path64)
pub type Path64 = Vec<Point64>;

/// Clipper2 paths type (multiple contours)
/// Clipper2Utils.cpp:8 (implicit from Clipper2Lib::Paths64)
pub type Paths64 = Vec<Path64>;

// ============================================================================
// Conversion: Slic3r → Clipper2
// ============================================================================

/// Convert Slic3r Points to Clipper2 Path64
/// Clipper2Utils.cpp:8-15 (helper pattern)
/// C++: Clipper2Lib::Path64 path;
/// C++: path.reserve(item.size());
/// C++: for (const Slic3r::Point& point : item.points)
/// C++:     path.emplace_back(std::move(Clipper2Lib::Point64(point.x(), point.y())));
fn points_to_path64(points: &[Point]) -> Path64 {
    points.iter().map(|p| (p.x(), p.y())).collect()
}

/// Convert Slic3r Polyline to Clipper2 Path64
/// Clipper2Utils.cpp:23-38
fn polyline_to_path64(polyline: &Polyline) -> Path64 {
    points_to_path64(&polyline.points)
}

/// Convert Slic3r Polylines to Clipper2 Paths64
/// Clipper2Utils.cpp:23-38 (template instantiation)
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
fn polylines_to_paths64(polylines: &[Polyline]) -> Paths64 {
    polylines.iter().map(|pl| polyline_to_path64(pl)).collect()
}

/// Convert Slic3r Polygon to Clipper2 Path64
/// Clipper2Utils.cpp:68-76
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
fn polygon_to_path64(polygon: &Polygon) -> Path64 {
    points_to_path64(&polygon.points)
}

/// Convert Slic3r Polygons to Clipper2 Paths64
/// Clipper2Utils.cpp:68-76
fn polygons_to_paths64(polygons: &[Polygon]) -> Paths64 {
    polygons.iter().map(|p| polygon_to_path64(p)).collect()
}

/// Convert Slic3r ExPolygons to Clipper2 Paths64
/// Clipper2Utils.cpp:78-91
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
fn expolygons_to_paths64(expolygons: &ExPolygons) -> Paths64 {
    let mut out = Vec::new();
    for expoly in expolygons {
        // Add contour
        out.push(points_to_path64(&expoly.contour.points));
        // Add holes
        for hole in &expoly.holes {
            out.push(points_to_path64(&hole.points));
        }
    }
    out
}

// ============================================================================
// Conversion: Clipper2 → Slic3r
// ============================================================================

/// Convert Clipper2 Path64 to Slic3r Points
/// Clipper2Utils.cpp:40-46
/// C++: Points Path64ToPoints(const Clipper2Lib::Path64& path64)
/// C++: {
/// C++:     Points points;
/// C++:     points.reserve(path64.size());
/// C++:     for (const Clipper2Lib::Point64 &point64 : path64) points.emplace_back(std::move(Slic3r::Point(point64.x, point64.y)));
/// C++:     return points;
/// C++: }
fn path64_to_points(path: &Path64) -> Vec<Point> {
    path.iter().map(|&(x, y)| Point::new(x, y)).collect()
}

/// Convert Slic3r points to Clipper2 crate path (f64 coordinates)
fn points_to_clipper_path(points: &[Point]) -> Vec<(f64, f64)> {
    points
        .iter()
        .map(|p| (unscale(p.x()), unscale(p.y())))
        .collect()
}

/// Convert Clipper2 crate path (f64 coordinates) to Slic3r points
fn clipper_path_to_points(path: &[(f64, f64)]) -> Vec<Point> {
    path.iter()
        .map(|&(x, y)| Point::new(scale(x), scale(y)))
        .collect()
}

/// Convert Clipper2 Paths64 to Slic3r Polylines
/// Clipper2Utils.cpp:8-21 (inverse operation)
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
fn paths64_to_polylines(paths: &Paths64) -> Vec<Polyline> {
    paths
        .iter()
        .map(|path| Polyline {
            points: path64_to_points(path),
        })
        .collect()
}

/// Convert Clipper2 Path64 to Slic3r Polygon
/// Clipper2Utils.cpp:40-46 (adapted for Polygon)
fn path64_to_polygon(path: &Path64) -> Polygon {
    Polygon {
        points: path64_to_points(path),
    }
}

// ============================================================================
// PolyTree Conversion (for hierarchical polygon results)
// ============================================================================

// NOTE: Clipper2's PolyTree is a hierarchical structure representing polygon
// containment. The C++ code converts this to ExPolygons by treating alternating
// levels as contours and holes.
//
// Since we don't have direct Clipper2 bindings yet, we'll implement the core
// operations as direct Paths64 → ExPolygons conversions for now.
//
// TODO: Add full PolyTree support when Clipper2 Rust bindings are more mature.

/// Reconstruct ExPolygons from Paths64 (simplified approach)
/// Clipper2Utils.cpp:48-66
/// C++: static ExPolygons PolyTreeToExPolygons(Clipper2Lib::PolyTree64 &&polytree)
/// C++: {
/// C++:     struct Inner { ... };
/// C++:     ExPolygons retval;
/// C++:     size_t cnt = 0;
/// C++:     for (int i = 0; i < polytree.Count(); ++i) cnt += Inner::PolyTreeCountExPolygons(*polytree[i]);
/// C++:     retval.reserve(cnt);
/// C++:     for (int i = 0; i < polytree.Count(); ++i) Inner::PolyTreeToExPolygonsRecursive(std::move(*polytree[i]), &retval);
/// C++:     return retval;
/// C++: }
///
/// This is a simplified version that assumes all paths are contours.
/// A full implementation would require PolyTree access to determine hierarchy.
fn paths64_to_expolygons(paths: &Paths64) -> ExPolygons {
    // Simple conversion: treat each path as a separate expolygon with no holes
    // TODO: Implement proper PolyTree parsing for nested polygons
    paths
        .iter()
        .map(|path| ExPolygon {
            contour: path64_to_polygon(path),
            holes: vec![],
        })
        .collect()
}

// ============================================================================
// Boolean Operations (Polylines)
// ============================================================================

/// Intersection of polylines with polygons (open paths)
/// Clipper2Utils.cpp:93-107
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
///
/// NOTE: Uses clipper2 crate for actual implementation
fn clipper2_pl_open(clip_type: ClipType, subject: &[Polyline], clip: &[Polygon]) -> Vec<Polyline> {
    // Convert subject polylines to clipper2 paths
    // Clipper2Utils.cpp:95-96
    // C++: c.AddOpenSubject(Slic3rPoints_to_Paths64(subject));
    let subject_paths: Vec<Vec<(f64, f64)>> = subject
        .iter()
        .map(|pl| points_to_clipper_path(&pl.points))
        .collect();

    // Convert clip polygons to clipper2 paths
    // Clipper2Utils.cpp:97
    // C++: c.AddClip(Slic3rPoints_to_Paths64(clip));
    let clip_paths: Vec<Vec<(f64, f64)>> = clip
        .iter()
        .map(|poly| points_to_clipper_path(&poly.points))
        .collect();

    // Execute clipper operation
    // Clipper2Utils.cpp:99-102
    // C++: Clipper2Lib::ClipType ct = clipType;
    // C++: Clipper2Lib::FillRule fr = Clipper2Lib::FillRule::NonZero;
    // C++: Clipper2Lib::Paths64 solution, solution_open;
    // C++: c.Execute(ct, fr, solution, solution_open);
    // Convert to ClipperPaths for clipper2 operations
    let subject_clipper_paths = ClipperPaths::<Centi>::from(subject_paths);
    let clip_clipper_paths = ClipperPaths::<Centi>::from(clip_paths);

    let result: ClipperPaths<Centi> = match clip_type {
        ClipType::Intersection => {
            clipper2::intersect(subject_clipper_paths, clip_clipper_paths, FillRule::NonZero)
                .unwrap_or_default()
        }
        ClipType::Difference => {
            clipper2::difference(subject_clipper_paths, clip_clipper_paths, FillRule::NonZero)
                .unwrap_or_default()
        }
    };

    // Convert result back to polylines
    // Clipper2Utils.cpp:104-107
    // C++: Slic3r::Polylines out;
    // C++: out.reserve(solution.size() + solution_open.size());
    // C++: polylines_append(out, std::move(Paths64_to_polylines(solution)));
    // C++: polylines_append(out, std::move(Paths64_to_polylines(solution_open)));
    let result_vec: Vec<Vec<(f64, f64)>> = result.into();
    result_vec
        .iter()
        .map(|path| Polyline {
            points: clipper_path_to_points(path),
        })
        .collect()
}

/// Clipper2 clip type enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ClipType {
    Intersection,
    Difference,
}

/// Intersection of polylines with polygons
/// Clipper2Utils.hpp:8
/// Clipper2Utils.cpp:109-110
/// C++: Slic3r::Polylines intersection_pl_2(const Slic3r::Polylines& subject, const Slic3r::Polygons& clip)
/// C++:     { return _clipper2_pl_open(Clipper2Lib::ClipType::Intersection, subject, clip); }
pub fn intersection_pl_2(subject: &[Polyline], clip: &[Polygon]) -> Vec<Polyline> {
    clipper2_pl_open(ClipType::Intersection, subject, clip)
}

/// Difference of polylines with polygons
/// Clipper2Utils.hpp:9
/// Clipper2Utils.cpp:111-112
/// C++: Slic3r::Polylines diff_pl_2(const Slic3r::Polylines& subject, const Slic3r::Polygons& clip)
/// C++:     { return _clipper2_pl_open(Clipper2Lib::ClipType::Difference, subject, clip); }
pub fn diff_pl_2(subject: &[Polyline], clip: &[Polygon]) -> Vec<Polyline> {
    clipper2_pl_open(ClipType::Difference, subject, clip)
}

// ============================================================================
// Boolean Operations (Polygons)
// ============================================================================

/// Union of polygons
/// Clipper2Utils.hpp:10
/// Clipper2Utils.cpp:114-126
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
///
/// NOTE: Uses clipper2 crate for actual implementation
pub fn union_ex_2(polygons: &[Polygon]) -> ExPolygons {
    if polygons.is_empty() {
        return Vec::new();
    }

    // Convert polygons to clipper2 paths
    // Clipper2Utils.cpp:116
    // C++: c.AddSubject(Slic3rPolygons_to_Paths64(polygons));
    let paths: Vec<Vec<(f64, f64)>> = polygons
        .iter()
        .map(|poly| points_to_clipper_path(&poly.points))
        .collect();

    // Execute union operation
    // Clipper2Utils.cpp:118-121
    // C++: Clipper2Lib::ClipType ct = Clipper2Lib::ClipType::Union;
    // C++: Clipper2Lib::FillRule fr = Clipper2Lib::FillRule::NonZero;
    // C++: Clipper2Lib::PolyTree64 solution;
    // C++: c.Execute(ct, fr, solution);
    let result: ClipperPaths<Centi> = clipper2::union(
        ClipperPaths::<Centi>::from(paths),
        ClipperPaths::<Centi>::from(Vec::<Vec<(f64, f64)>>::new()),
        FillRule::NonZero,
    )
    .unwrap_or_default();

    // Convert result to ExPolygons
    // Clipper2Utils.cpp:123
    // C++: ExPolygons results = PolyTreeToExPolygons(std::move(solution));
    let result_vec: Vec<Vec<(f64, f64)>> = result.into();
    result_vec
        .iter()
        .map(|path| ExPolygon {
            contour: Polygon {
                points: clipper_path_to_points(path),
            },
            holes: vec![],
        })
        .collect()
}

/// Union of expolygons
/// Clipper2Utils.hpp:11
/// Clipper2Utils.cpp:128-140
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
/// NOTE: Uses clipper2 crate for actual implementation
pub fn union_ex_2_expolygons(expolygons: &ExPolygons) -> ExPolygons {
    if expolygons.is_empty() {
        return Vec::new();
    }

    // Convert expolygons to clipper2 paths (flatten contours and holes)
    // Clipper2Utils.cpp:130
    // C++: c.AddSubject(Slic3rExPolygons_to_Paths64(expolygons));
    let mut paths: Vec<Vec<(f64, f64)>> = Vec::new();
    for expoly in expolygons {
        paths.push(points_to_clipper_path(&expoly.contour.points));
        for hole in &expoly.holes {
            paths.push(points_to_clipper_path(&hole.points));
        }
    }

    // Execute union operation
    // Clipper2Utils.cpp:132-135
    // C++: Clipper2Lib::ClipType   ct = Clipper2Lib::ClipType::Union;
    // C++: Clipper2Lib::FillRule   fr = Clipper2Lib::FillRule::NonZero;
    // C++: Clipper2Lib::PolyTree64 solution;
    // C++: c.Execute(ct, fr, solution);
    let result: ClipperPaths<Centi> = clipper2::union(
        ClipperPaths::<Centi>::from(paths),
        ClipperPaths::<Centi>::from(Vec::<Vec<(f64, f64)>>::new()),
        FillRule::NonZero,
    )
    .unwrap_or_default();

    // Convert result to ExPolygons
    // Clipper2Utils.cpp:137
    // C++: ExPolygons results = PolyTreeToExPolygons(std::move(solution));
    let result_vec: Vec<Vec<(f64, f64)>> = result.into();
    result_vec
        .iter()
        .map(|path| ExPolygon {
            contour: Polygon {
                points: clipper_path_to_points(path),
            },
            holes: vec![],
        })
        .collect()
}

// ============================================================================
// Offset Operations
// ============================================================================

/// Offset expolygons by a given delta
/// Clipper2Utils.hpp:12
/// Clipper2Utils.cpp:143-153
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
///
/// NOTE: Uses clipper2 crate for actual implementation
pub fn offset_ex_2(expolygons: &ExPolygons, delta: f64) -> ExPolygons {
    if expolygons.is_empty() {
        return Vec::new();
    }

    // Convert expolygons to clipper2 paths (flatten contours and holes)
    // Clipper2Utils.cpp:145
    // C++: Clipper2Lib::Paths64 subject = Slic3rExPolygons_to_Paths64(expolygons);
    let mut paths: Vec<Vec<(f64, f64)>> = Vec::new();
    for expoly in expolygons {
        paths.push(points_to_clipper_path(&expoly.contour.points));
        for hole in &expoly.holes {
            paths.push(points_to_clipper_path(&hole.points));
        }
    }

    // Execute offset operation
    // Clipper2Utils.cpp:146-149
    // C++: Clipper2Lib::ClipperOffset offsetter;
    // C++: offsetter.AddPaths(subject, Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);
    // C++: Clipper2Lib::PolyPath64 polytree;
    // C++: offsetter.Execute(delta, polytree);
    let delta_mm = unscale(scale(delta)); // Ensure proper scaling
    let result: ClipperPaths<Centi> = clipper2::inflate(
        ClipperPaths::<Centi>::from(paths),
        delta_mm,
        JoinType::Round,
        EndType::Polygon,
        0.0,
    );

    // Convert result to ExPolygons
    // Clipper2Utils.cpp:150
    // C++: ExPolygons results = PolyTreeToExPolygons(std::move(polytree));
    let result_vec: Vec<Vec<(f64, f64)>> = result.into();
    result_vec
        .iter()
        .map(|path| ExPolygon {
            contour: Polygon {
                points: clipper_path_to_points(path),
            },
            holes: vec![],
        })
        .collect()
}

/// Two-stage offset with simplification between stages
/// Clipper2Utils.hpp:13
/// Clipper2Utils.cpp:155-172
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
///
/// NOTE: Uses clipper2 crate for actual implementation
pub fn offset2_ex_2(expolygons: &ExPolygons, delta1: f64, delta2: f64) -> ExPolygons {
    if expolygons.is_empty() {
        return Vec::new();
    }

    // Convert expolygons to clipper2 paths (flatten contours and holes)
    // Clipper2Utils.cpp:158
    // C++: Clipper2Lib::Paths64 subject = Slic3rExPolygons_to_Paths64(expolygons);
    let mut paths: Vec<Vec<(f64, f64)>> = Vec::new();
    for expoly in expolygons {
        paths.push(points_to_clipper_path(&expoly.contour.points));
        for hole in &expoly.holes {
            paths.push(points_to_clipper_path(&hole.points));
        }
    }

    // First offset
    // Clipper2Utils.cpp:159-162
    // C++: Clipper2Lib::ClipperOffset offsetter;
    // C++: offsetter.AddPaths(subject, Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);
    // C++: Clipper2Lib::PolyPath64 polytree;
    // C++: offsetter.Execute(delta1, polytree);
    let delta1_mm = unscale(scale(delta1));
    let intermediate: ClipperPaths<Centi> = clipper2::inflate(
        ClipperPaths::<Centi>::from(paths),
        delta1_mm,
        JoinType::Round,
        EndType::Polygon,
        0.0,
    );

    // Simplify intermediate result
    // Clipper2Utils.cpp:164-165
    // C++: Clipper2Lib::PolyPath64 polytree2;
    // C++: SimplifyPolyTree(polytree, SCALED_EPSILON, polytree2);
    let epsilon_mm = unscale(SCALED_EPSILON as i64);
    let simplified: ClipperPaths<Centi> = clipper2::simplify(intermediate, epsilon_mm, false);

    // Second offset
    // Clipper2Utils.cpp:167-170
    // C++: offsetter.Clear();
    // C++: offsetter.AddPaths(Clipper2Lib::PolyTreeToPaths64(polytree2), Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);
    // C++: polytree.Clear();
    // C++: offsetter.Execute(delta2, polytree);
    let delta2_mm = unscale(scale(delta2));
    let result: ClipperPaths<Centi> = clipper2::inflate(
        simplified,
        delta2_mm,
        JoinType::Round,
        EndType::Polygon,
        0.0,
    );

    // Convert result to ExPolygons
    // Clipper2Utils.cpp:172
    // C++: ExPolygons results = PolyTreeToExPolygons(std::move(polytree));
    let result_vec: Vec<Vec<(f64, f64)>> = result.into();
    result_vec
        .iter()
        .map(|path| ExPolygon {
            contour: Polygon {
                points: clipper_path_to_points(path),
            },
            holes: vec![],
        })
        .collect()
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// Test conversion from Points to Path64
    /// Clipper2Utils.cpp:8-15
    #[test]
    fn test_points_to_path64() {
        let points = vec![Point::new(0, 0), Point::new(100, 0), Point::new(100, 100)];
        let path = points_to_path64(&points);

        assert_eq!(path.len(), 3);
        assert_eq!(path[0], (0, 0));
        assert_eq!(path[1], (100, 0));
        assert_eq!(path[2], (100, 100));
    }

    /// Test conversion from Path64 to Points
    /// Clipper2Utils.cpp:40-46
    #[test]
    fn test_path64_to_points() {
        let path = vec![(0, 0), (100, 0), (100, 100)];
        let points = path64_to_points(&path);

        assert_eq!(points.len(), 3);
        assert_eq!(points[0], Point::new(0, 0));
        assert_eq!(points[1], Point::new(100, 0));
        assert_eq!(points[2], Point::new(100, 100));
    }

    /// Test polyline conversion
    /// Clipper2Utils.cpp:8-21
    #[test]
    fn test_polyline_conversion() {
        let polyline = Polyline {
            points: vec![Point::new(0, 0), Point::new(100, 0)],
        };

        let path = polyline_to_path64(&polyline);
        assert_eq!(path.len(), 2);

        let polylines = vec![polyline.clone()];
        let paths = polylines_to_paths64(&polylines);
        assert_eq!(paths.len(), 1);

        let result = paths64_to_polylines(&paths);
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].points.len(), 2);
    }

    /// Test polygon conversion
    /// Clipper2Utils.cpp:68-76
    #[test]
    fn test_polygon_conversion() {
        let polygon = Polygon {
            points: vec![Point::new(0, 0), Point::new(100, 0), Point::new(100, 100)],
        };

        let path = polygon_to_path64(&polygon);
        assert_eq!(path.len(), 3);

        let result = path64_to_polygon(&path);
        assert_eq!(result.points.len(), 3);
    }

    /// Test expolygon conversion
    /// Clipper2Utils.cpp:78-91
    #[test]
    fn test_expolygon_conversion() {
        let expoly = ExPolygon {
            contour: Polygon {
                points: vec![Point::new(0, 0), Point::new(100, 0), Point::new(100, 100)],
            },
            holes: vec![Polygon {
                points: vec![Point::new(10, 10), Point::new(20, 10), Point::new(20, 20)],
            }],
        };

        let paths = expolygons_to_paths64(&[expoly]);
        assert_eq!(paths.len(), 2); // contour + 1 hole
    }

    /// Test actual Clipper2 operations work correctly
    #[test]
    fn test_union_operations() {
        let polygons = vec![
            Polygon {
                points: vec![
                    Point::new(0, 0),
                    Point::new(scale(10.0), 0),
                    Point::new(scale(10.0), scale(10.0)),
                    Point::new(0, scale(10.0)),
                ],
            },
            Polygon {
                points: vec![
                    Point::new(scale(5.0), scale(5.0)),
                    Point::new(scale(15.0), scale(5.0)),
                    Point::new(scale(15.0), scale(15.0)),
                    Point::new(scale(5.0), scale(15.0)),
                ],
            },
        ];

        let result = union_ex_2(&polygons);
        assert!(!result.is_empty(), "Union should produce results");
        assert!(
            result[0].contour.points.len() >= 4,
            "Union result should have at least 4 points"
        );
    }

    /// Test offset operations
    #[test]
    fn test_offset_operations() {
        let expoly = ExPolygon {
            contour: Polygon {
                points: vec![
                    Point::new(0, 0),
                    Point::new(scale(10.0), 0),
                    Point::new(scale(10.0), scale(10.0)),
                    Point::new(0, scale(10.0)),
                ],
            },
            holes: vec![],
        };

        // Test single offset
        let result = offset_ex_2(&[expoly.clone()], 1.0);
        assert!(!result.is_empty(), "Offset should produce results");

        // Test two-stage offset
        let result2 = offset2_ex_2(&[expoly], 1.0, -0.5);
        assert!(!result2.is_empty(), "Offset2 should produce results");
    }

    /// Test polyline operations
    #[test]
    fn test_polyline_operations() {
        let polylines = vec![Polyline {
            points: vec![
                Point::new(0, 0),
                Point::new(scale(10.0), 0),
                Point::new(scale(10.0), scale(10.0)),
            ],
        }];

        let clip = vec![Polygon {
            points: vec![
                Point::new(scale(5.0), scale(-1.0)),
                Point::new(scale(15.0), scale(-1.0)),
                Point::new(scale(15.0), scale(5.0)),
                Point::new(scale(5.0), scale(5.0)),
            ],
        }];

        let result = intersection_pl_2(&polylines, &clip);
        // Result should contain the portion of polyline inside clip polygon
        assert!(!result.is_empty() || polylines.is_empty());
    }
}
