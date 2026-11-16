//! Intersection point detection for lines and polygons
//!
//! C++ Reference:
//! - IntersectionPoints.hpp (22 lines)
//! - IntersectionPoints.cpp (45 lines)
//!
//! This module provides functionality to detect all intersection points between
//! line segments in various geometric structures (lines, polygons, expolygons).
//! Mirrors the C++ which uses `AABBTreeLines` for efficient intersection detection.

// IntersectionPoints.cpp:1-2
use crate::aabb_tree_lines::{build_aabb_tree_over_indexed_lines, get_intersections_with_line};
use crate::geometry::{
    to_lines as expolygons_to_lines, to_lines_expoly, ExPolygon, ExPolygons, Lines, PointF,
    Polygon, Polygons,
};

/// Intersection information between two line segments
/// IntersectionPoints.hpp:8-12
#[derive(Debug, Clone)]
pub struct IntersectionLines {
    /// Index of the first line
    /// IntersectionPoints.hpp:9
    pub line_index1: u32,

    /// Index of the second line
    /// IntersectionPoints.hpp:10
    pub line_index2: u32,

    /// Intersection point in 2D space
    /// IntersectionPoints.hpp:11
    pub intersection: PointF,
}

/// Collection of intersection line information
/// IntersectionPoints.hpp:13
pub type IntersectionsLines = Vec<IntersectionLines>;

/// Internal function to compute all intersections in a set of lines.
/// IntersectionPoints.cpp:8-36
fn compute_intersections(lines: &Lines) -> IntersectionsLines {
    // IntersectionPoints.cpp:10-11
    if lines.len() < 3 {
        return Vec::new();
    }

    // IntersectionPoints.cpp:13
    let tree = build_aabb_tree_over_indexed_lines(lines);
    // IntersectionPoints.cpp:14
    let mut result: IntersectionsLines = IntersectionsLines::new();
    // IntersectionPoints.cpp:15 — for (uint32_t li = 0; li < lines.size()-1; ++li)
    for li in 0..(lines.len() - 1) {
        // IntersectionPoints.cpp:16
        let l = &lines[li];
        // IntersectionPoints.cpp:17 — get_intersections_with_line<false, Point, Line>
        let intersections = get_intersections_with_line::<false>(lines, &tree, l);
        // IntersectionPoints.cpp:18 — for (const auto &[p, node_index] : intersections)
        for (p, node_index) in &intersections {
            let node_index = *node_index;
            // IntersectionPoints.cpp:19-20 — if (node_index - 1 <= li) continue;
            // node_index is size_t; C++ wraps on node_index == 0. Mirror the unsigned
            // semantics: node_index.wrapping_sub(1) <= li.
            if node_index.wrapping_sub(1) <= li {
                continue;
            }
            // IntersectionPoints.cpp:21-27 — shared-endpoint check (duplicit point)
            let l_ = &lines[node_index];
            if l_.a == l.a || l_.a == l.b || l_.b == l.a || l_.b == l.b {
                // it is duplicit point not intersection
                continue;
            }

            // NOTE: fix AABBTree to compute intersection with double preccission!!
            // IntersectionPoints.cpp:29-30 — Vec2d intersection_point = p.cast<double>();
            let intersection_point = PointF::new(p.x as f64, p.y as f64);

            // IntersectionPoints.cpp:32
            result.push(IntersectionLines {
                line_index1: li as u32,
                line_index2: node_index as u32,
                intersection: intersection_point,
            });
        }
    }
    // IntersectionPoints.cpp:35
    result
}

/// Collect all intersecting points in a set of lines
/// IntersectionPoints.cpp:40 — get_intersections(const Lines &lines)
pub fn get_intersections_lines(lines: &Lines) -> IntersectionsLines {
    compute_intersections(lines)
}

/// Collect all intersecting points in a polygon
/// IntersectionPoints.cpp:41 — get_intersections(const Polygon &polygon)
pub fn get_intersections_polygon(polygon: &Polygon) -> IntersectionsLines {
    // C++ `to_lines(const Polygon&)`; `Polygon::lines()` == `to_lines(*this)`.
    compute_intersections(&polygon.lines())
}

/// Collect all intersecting points in a set of polygons
/// IntersectionPoints.cpp:42 — get_intersections(const Polygons &polygons)
pub fn get_intersections_polygons(polygons: &Polygons) -> IntersectionsLines {
    // C++ `to_lines(const Polygons&)` — concatenate the edges of every polygon.
    let mut lines: Lines = Lines::new();
    for polygon in polygons {
        lines.extend(polygon.lines());
    }
    compute_intersections(&lines)
}

/// Collect all intersecting points in an expolygon
/// IntersectionPoints.cpp:43 — get_intersections(const ExPolygon &expolygon)
pub fn get_intersections_expolygon(expolygon: &ExPolygon) -> IntersectionsLines {
    compute_intersections(&to_lines_expoly(expolygon))
}

/// Collect all intersecting points in a set of expolygons
/// IntersectionPoints.cpp:44 — get_intersections(const ExPolygons &expolygons)
pub fn get_intersections_expolygons(expolygons: &ExPolygons) -> IntersectionsLines {
    compute_intersections(&expolygons_to_lines(expolygons))
}

/// The five `get_intersections` overloads of IntersectionPoints.hpp:15-19, one implementation
/// per argument type (the C++ overload set), so that `get_intersections(&x)` reads as the C++
/// call does.
pub trait GetIntersections {
    fn get_intersections(&self) -> IntersectionsLines;
}

impl GetIntersections for Lines {
    // IntersectionPoints.cpp:40  IntersectionsLines get_intersections(const Lines &lines)           { return compute_intersections(lines); }
    fn get_intersections(&self) -> IntersectionsLines {
        compute_intersections(self)
    }
}

impl GetIntersections for Polygon {
    // IntersectionPoints.cpp:41  IntersectionsLines get_intersections(const Polygon &polygon)       { return compute_intersections(to_lines(polygon)); }
    fn get_intersections(&self) -> IntersectionsLines {
        get_intersections_polygon(self)
    }
}

impl GetIntersections for Polygons {
    // IntersectionPoints.cpp:42  IntersectionsLines get_intersections(const Polygons &polygons)     { return compute_intersections(to_lines(polygons)); }
    fn get_intersections(&self) -> IntersectionsLines {
        get_intersections_polygons(self)
    }
}

impl GetIntersections for ExPolygon {
    // IntersectionPoints.cpp:43  IntersectionsLines get_intersections(const ExPolygon &expolygon)   { return compute_intersections(to_lines(expolygon)); }
    fn get_intersections(&self) -> IntersectionsLines {
        compute_intersections(&to_lines_expoly(self))
    }
}

impl GetIntersections for ExPolygons {
    // IntersectionPoints.cpp:44  IntersectionsLines get_intersections(const ExPolygons &expolygons) { return compute_intersections(to_lines(expolygons)); }
    fn get_intersections(&self) -> IntersectionsLines {
        compute_intersections(&expolygons_to_lines(self))
    }
}

/// IntersectionPoints.hpp:15-19 — `get_intersections(x)` for any of the five argument types.
pub fn get_intersections<T: GetIntersections + ?Sized>(input: &T) -> IntersectionsLines {
    input.get_intersections()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Line, Point};

    #[test]
    fn test_no_intersections_empty() {
        let lines = Lines::new();
        let intersections = get_intersections_lines(&lines);
        assert!(intersections.is_empty());
    }

    #[test]
    fn test_no_intersections_too_few() {
        let lines = vec![
            Line::new(Point::new(0, 0), Point::new(100, 0)),
            Line::new(Point::new(0, 100), Point::new(100, 100)),
        ];
        let intersections = get_intersections_lines(&lines);
        assert!(
            intersections.is_empty(),
            "Need at least 3 lines to detect intersections"
        );
    }

    #[test]
    fn test_simple_intersection() {
        // Two lines that cross in the middle
        let lines = vec![
            Line::new(Point::new(0, 0), Point::new(100, 100)), // Diagonal
            Line::new(Point::new(100, 0), Point::new(0, 100)), // Other diagonal
            Line::new(Point::new(200, 200), Point::new(300, 200)), // Far away (needed for >= 3)
        ];
        let intersections = get_intersections_lines(&lines);

        // IntersectionPoints.cpp:18 skips `node_index - 1 <= li` (size_t): from line 0 the
        // crossing with line 1 is skipped, from line 1 the crossing with line 0 is kept, because
        // 0 - 1 wraps. The pin gives one intersection [1 0 (50, 50)] (a generator calling
        // get_intersections on these lines).
        assert_eq!(intersections.len(), 1, "Should find one intersection");
        assert_eq!(intersections[0].line_index1, 1);
        assert_eq!(intersections[0].line_index2, 0);
        assert_eq!((intersections[0].intersection.x, intersections[0].intersection.y), (50.0, 50.0));
    }

    #[test]
    fn test_no_intersection_parallel() {
        // Three parallel lines - no intersections
        let lines = vec![
            Line::new(Point::new(0, 0), Point::new(100, 0)),
            Line::new(Point::new(0, 50), Point::new(100, 50)),
            Line::new(Point::new(0, 100), Point::new(100, 100)),
        ];
        let intersections = get_intersections_lines(&lines);
        assert!(
            intersections.is_empty(),
            "Parallel lines should not intersect"
        );
    }

    #[test]
    fn test_shared_endpoint_not_intersection() {
        // Lines that share endpoints are not considered intersections
        let lines = vec![
            Line::new(Point::new(0, 0), Point::new(100, 0)),
            Line::new(Point::new(100, 0), Point::new(100, 100)),
            Line::new(Point::new(100, 100), Point::new(0, 100)),
        ];
        let intersections = get_intersections_lines(&lines);
        assert!(
            intersections.is_empty(),
            "Shared endpoints should not count as intersections"
        );
    }

    #[test]
    fn test_polygon_self_intersection() {
        // Create a self-intersecting polygon (figure-8 shape)
        let polygon = Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(100, 100),
            Point::new(100, 0),
            Point::new(0, 100),
        ]);

        let intersections = get_intersections_polygon(&polygon);

        // The two crossing segments should intersect
        assert!(
            !intersections.is_empty(),
            "Self-intersecting polygon should have intersections"
        );
    }

    #[test]
    fn test_intersection_lines_struct() {
        let intersection = IntersectionLines {
            line_index1: 5,
            line_index2: 10,
            intersection: PointF::new(50.5, 75.3),
        };

        assert_eq!(intersection.line_index1, 5);
        assert_eq!(intersection.line_index2, 10);
        assert_eq!(intersection.intersection.x, 50.5);
        assert_eq!(intersection.intersection.y, 75.3);
    }
}
