//! Intersection point detection for lines and polygons
//!
//! C++ Reference:
//! - IntersectionPoints.hpp (22 lines)
//! - IntersectionPoints.cpp (45 lines)
//!
//! This module provides functionality to detect all intersection points between
//! line segments in various geometric structures (lines, polygons, expolygons).
//! Uses AABBTreeLines for efficient intersection detection.

use crate::geometry::{ExPolygon, ExPolygons, Lines, PointF, Polygon, Polygons};

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

/// Convert polygon points to line segments
/// ExPolygon.hpp:122-132
fn polygon_to_lines(polygon: &Polygon) -> Lines {
    let mut lines = Lines::new();
    if polygon.points().is_empty() {
        return lines;
    }

    // Create line segments between consecutive points
    for i in 0..polygon.points().len() - 1 {
        lines.push(crate::geometry::Line::new(
            polygon.points()[i],
            polygon.points()[i + 1],
        ));
    }

    // Close the polygon with a line from last to first point
    if let (Some(&last), Some(&first)) = (polygon.points().last(), polygon.points().first()) {
        lines.push(crate::geometry::Line::new(last, first));
    }

    lines
}

/// Internal function to compute all intersections in a set of lines
/// Uses brute-force for now (TODO: optimize with AABBTreeLines)
/// IntersectionPoints.cpp:6-35
fn compute_intersections(lines: &Lines) -> IntersectionsLines {
    // Early exit if too few lines to intersect
    // IntersectionPoints.cpp:9-10
    if lines.len() < 3 {
        return Vec::new();
    }

    // TODO: Use AABBTreeLines::build_aabb_tree_over_indexed_lines for efficiency
    // For now, use brute force O(n²) approach
    // IntersectionPoints.cpp:12-13
    let mut result = Vec::new();

    // Test each line against all others
    // IntersectionPoints.cpp:14
    for li in 0..(lines.len() - 1) {
        // IntersectionPoints.cpp:15
        let l = &lines[li];

        // TODO: Use AABBTreeLines::get_intersections_with_line for efficiency
        // For now, test against all remaining lines
        // IntersectionPoints.cpp:16
        for lj in (li + 1)..lines.len() {
            let l_ = &lines[lj];

            // Skip if lines share endpoints (not a true intersection)
            // IntersectionPoints.cpp:21-26
            if l_.a == l.a || l_.a == l.b || l_.b == l.a || l_.b == l.b {
                // it is duplicate point not intersection
                continue;
            }

            // Check if lines actually intersect
            if let Some(intersection_point) = line_segment_intersection(l, l_) {
                // NOTE: fix AABBTree to compute intersection with double precision!!
                // IntersectionPoints.cpp:28-29

                // Add to result list
                // IntersectionPoints.cpp:31
                result.push(IntersectionLines {
                    line_index1: li as u32,
                    line_index2: lj as u32,
                    intersection: intersection_point,
                });
            }
        }
    }

    // IntersectionPoints.cpp:34
    result
}

/// Compute intersection point between two line segments
/// Returns Some(point) if segments intersect, None otherwise
fn line_segment_intersection(
    l1: &crate::geometry::Line,
    l2: &crate::geometry::Line,
) -> Option<PointF> {
    let p1 = PointF::new(l1.a.x as f64, l1.a.y as f64);
    let p2 = PointF::new(l1.b.x as f64, l1.b.y as f64);
    let p3 = PointF::new(l2.a.x as f64, l2.a.y as f64);
    let p4 = PointF::new(l2.b.x as f64, l2.b.y as f64);

    let d = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x);

    // Lines are parallel or coincident
    if d.abs() < 1e-10 {
        return None;
    }

    let t = ((p1.x - p3.x) * (p3.y - p4.y) - (p1.y - p3.y) * (p3.x - p4.x)) / d;
    let u = -((p1.x - p2.x) * (p1.y - p3.y) - (p1.y - p2.y) * (p1.x - p3.x)) / d;

    // Check if intersection is within both line segments
    if t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0 {
        let x = p1.x + t * (p2.x - p1.x);
        let y = p1.y + t * (p2.y - p1.y);
        Some(PointF::new(x, y))
    } else {
        None
    }
}

/// Collect all intersecting points in a set of lines
/// IntersectionPoints.cpp:39
pub fn get_intersections_lines(lines: &Lines) -> IntersectionsLines {
    compute_intersections(lines)
}

/// Collect all intersecting points in a polygon
/// IntersectionPoints.cpp:40
pub fn get_intersections_polygon(polygon: &Polygon) -> IntersectionsLines {
    compute_intersections(&polygon_to_lines(polygon))
}

/// Collect all intersecting points in a set of polygons
/// IntersectionPoints.cpp:41
pub fn get_intersections_polygons(polygons: &Polygons) -> IntersectionsLines {
    let mut lines = Lines::new();
    for polygon in polygons {
        lines.extend(polygon_to_lines(polygon));
    }
    compute_intersections(&lines)
}

/// Collect all intersecting points in an expolygon
/// IntersectionPoints.cpp:42
pub fn get_intersections_expolygon(expolygon: &ExPolygon) -> IntersectionsLines {
    let mut lines = polygon_to_lines(&expolygon.contour);
    for hole in &expolygon.holes {
        lines.extend(polygon_to_lines(hole));
    }
    compute_intersections(&lines)
}

/// Collect all intersecting points in a set of expolygons
/// IntersectionPoints.cpp:43
pub fn get_intersections_expolygons(expolygons: &ExPolygons) -> IntersectionsLines {
    let mut lines = Lines::new();
    for expolygon in expolygons {
        lines.extend(polygon_to_lines(&expolygon.contour));
        for hole in &expolygon.holes {
            lines.extend(polygon_to_lines(hole));
        }
    }
    compute_intersections(&lines)
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

        // Should find one intersection between the two diagonals
        assert_eq!(intersections.len(), 1, "Should find one intersection");
        assert_eq!(intersections[0].line_index1, 0);
        assert_eq!(intersections[0].line_index2, 1);
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
        let polygon = Polygon::new(vec![
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
