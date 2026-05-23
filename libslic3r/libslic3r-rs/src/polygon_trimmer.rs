//! Polygon trimming utilities.
//!
//! C++ Reference:
//! - PolygonTrimmer.hpp (32 lines)
//! - PolygonTrimmer.cpp (57 lines)
//!
//! This module provides utilities for trimming polygon loops against an EdgeGrid,
//! detecting and handling intersections between loops and grid edges.

use crate::edge_grid::EdgeGrid;
use crate::geometry::{Point, Polygon, Polygons};
use crate::Coord;

/// A polygon loop that has been trimmed against an EdgeGrid.
///
/// PolygonTrimmer.hpp:13-22
#[derive(Debug, Clone)]
pub struct TrimmedLoop {
    /// The points of the trimmed loop
    /// PolygonTrimmer.hpp:15
    pub points: Vec<Point>,

    /// Number of points per segment.
    /// Empty if the loop is not trimmed (no intersections found).
    /// PolygonTrimmer.hpp:17
    pub segments: Vec<u32>,
}

impl TrimmedLoop {
    /// Create a new empty trimmed loop
    pub fn new() -> Self {
        Self {
            points: Vec::new(),
            segments: Vec::new(),
        }
    }

    /// Check if this loop has been trimmed (has intersection segments)
    ///
    /// PolygonTrimmer.hpp:20
    pub fn is_trimmed(&self) -> bool {
        // PolygonTrimmer.hpp:20
        // C++: bool is_trimmed() const { return ! segments.empty(); }
        !self.segments.is_empty()
    }
}

impl Default for TrimmedLoop {
    fn default() -> Self {
        Self::new()
    }
}

/// Trim a single polygon loop against an EdgeGrid.
///
/// This function visits all cells in the grid that intersect with the loop's edges,
/// checks for segment intersections, and builds a trimmed representation.
///
/// PolygonTrimmer.hpp:24
/// PolygonTrimmer.cpp:7-47
pub fn trim_loop(loop_polygon: &Polygon, grid: &EdgeGrid) -> TrimmedLoop {
    // Ensure the loop has at least 2 points
    // PolygonTrimmer.cpp:9-10
    // C++: assert(! loop.empty());
    // C++: assert(loop.size() >= 2);
    assert!(!loop_polygon.points().is_empty());
    assert!(loop_polygon.len() >= 2);

    // Create output structure
    // PolygonTrimmer.cpp:12
    let out = TrimmedLoop::new();

    // Only process if we have at least 2 points
    // PolygonTrimmer.cpp:14
    if loop_polygon.len() >= 2 {
        // Visitor struct that checks each grid cell for intersections
        // PolygonTrimmer.cpp:16
        // C++: struct Visitor {
        struct Visitor<'a> {
            grid: &'a EdgeGrid,
            pt_prev: &'a Point,
            pt_this: &'a Point,
        }

        impl<'a> Visitor<'a> {
            // Visit a grid cell and check for segment intersections
            // PolygonTrimmer.cpp:19-31
            // C++: bool operator()(coord_t iy, coord_t ix) {
            fn visit(&self, _iy: Coord, _ix: Coord) -> bool {
                // TODO: Implement cell_data_range iteration when EdgeGrid API is complete
                // Get the range of contour segments in this grid cell
                // PolygonTrimmer.cpp:21
                // C++: auto cell_data_range = grid.cell_data_range(iy, ix);
                // let cell_data_range = self.grid.cell_data_range(iy, ix);

                // Iterate over all segments in this cell
                // PolygonTrimmer.cpp:22
                // C++: for (auto it_contour_and_segment = cell_data_range.first; it_contour_and_segment != cell_data_range.second; ++ it_contour_and_segment) {
                // for contour_and_segment in cell_data_range {
                //     // Get the endpoints of the grid segment
                //     // PolygonTrimmer.cpp:24
                //     // C++: auto segment = grid.segment(*it_contour_and_segment);
                //     let segment =
                //         self.grid.contours()[contour_and_segment.0].segment(contour_and_segment.1);
                //     let segment = (segment.a, segment.b);
                //
                //     // Check if the two segments intersect
                //     // PolygonTrimmer.cpp:25
                //     // C++: if (Geometry::segments_intersect(segment.first, segment.second, *pt_prev, *pt_this)) {
                //     if segments_intersect(segment.0, segment.1, *self.pt_prev, *self.pt_this) {
                //         // The two segments intersect
                //         // PolygonTrimmer.cpp:26-27
                //         // C++: // The two segments intersect. Add them to the output.
                //         // TODO: Actually add the intersection to the output
                //         // The C++ code has a comment but no implementation
                //     }
                // }

                // Continue traversing the grid along the edge
                // PolygonTrimmer.cpp:30
                // C++: return true;
                true
            }
        }

        // Initialize visitor with the last point as the starting previous point
        // PolygonTrimmer.cpp:37
        // C++: } visitor(grid, &loop.points.back(), nullptr);
        let mut pt_prev = &loop_polygon.points()[loop_polygon.len() - 1];

        // Iterate over all points in the loop
        // PolygonTrimmer.cpp:39
        // C++: for (const Point &pt_this : loop.points) {
        for pt_this in loop_polygon.points() {
            // Create visitor for this edge
            // PolygonTrimmer.cpp:40
            let visitor = Visitor {
                grid,
                pt_prev,
                pt_this,
            };

            // Visit all grid cells that intersect this edge
            // PolygonTrimmer.cpp:41
            // C++: grid.visit_cells_intersecting_line(*visitor.pt_prev, pt_this, visitor);
            // TODO: Uncomment when EdgeGrid::visit_cells_intersecting_line is implemented
            // grid.visit_cells_intersecting_line(*pt_prev, *pt_this, |iy, ix| visitor.visit(iy, ix));
            let _ = visitor; // Suppress unused warning

            // Move to next edge
            // PolygonTrimmer.cpp:42
            // C++: visitor.pt_prev = &pt_this;
            pt_prev = pt_this;
        }
    }

    // Return the trimmed loop
    // PolygonTrimmer.cpp:46
    out
}

/// Trim multiple polygon loops against an EdgeGrid.
///
/// This is a convenience function that applies trim_loop to each polygon in the input.
///
/// PolygonTrimmer.hpp:25
/// PolygonTrimmer.cpp:49-56
pub fn trim_loops(loops: &Polygons, grid: &EdgeGrid) -> Vec<TrimmedLoop> {
    // Reserve space for output
    // PolygonTrimmer.cpp:51
    // C++: out.reserve(loops.size());
    let mut out = Vec::with_capacity(loops.len());

    // Trim each loop
    // PolygonTrimmer.cpp:52-53
    // C++: for (const Polygon &loop : loops)
    // C++:     out.emplace_back(trim_loop(loop, grid));
    for loop_polygon in loops {
        out.push(trim_loop(loop_polygon, grid));
    }

    // PolygonTrimmer.cpp:54
    out
}

/// Check if two line segments intersect.
///
/// This is a helper function that checks if the line segment from p1 to p2
/// intersects with the line segment from p3 to p4.
///
/// Geometry.hpp:117-156 (segments_intersect function)
fn segments_intersect(p1: Point, p2: Point, p3: Point, p4: Point) -> bool {
    // This is a simplified version of Geometry::segments_intersect
    // The full implementation uses cross products to determine if segments intersect
    //
    // Geometry.hpp:117
    // C++: inline bool segments_intersect(
    // C++:     const Slic3r::Point &ip1, const Slic3r::Point &ip2,
    // C++:     const Slic3r::Point &jp1, const Slic3r::Point &jp2)

    // Helper lambda to check if segments could intersect
    // Geometry.hpp:125-136
    let segments_could_intersect = |ip1: Point, ip2: Point, jp1: Point, jp2: Point| -> (i8, i8) {
        // Cast to i64 for arithmetic to avoid overflow
        let iv_x = (ip2.x - ip1.x) as i64;
        let iv_y = (ip2.y - ip1.y) as i64;
        let vij1_x = (jp1.x - ip1.x) as i64;
        let vij1_y = (jp1.y - ip1.y) as i64;
        let vij2_x = (jp2.x - ip1.x) as i64;
        let vij2_y = (jp2.y - ip1.y) as i64;

        // Cross product to determine orientation
        let tij1 = iv_x * vij1_y - iv_y * vij1_x;
        let tij2 = iv_x * vij2_y - iv_y * vij2_x;

        // Return signs
        (
            if tij1 > 0 {
                1
            } else if tij1 < 0 {
                -1
            } else {
                0
            },
            if tij2 > 0 {
                1
            } else if tij2 < 0 {
                -1
            } else {
                0
            },
        )
    };

    // Check both directions
    // Geometry.hpp:139-140
    let sign1 = segments_could_intersect(p1, p2, p3, p4);
    let sign2 = segments_could_intersect(p3, p4, p1, p2);

    let test1 = sign1.0 * sign1.1;
    let test2 = sign2.0 * sign2.1;

    // Segments intersect if both tests are <= 0 and at least one is != 0
    // Geometry.hpp:143-145
    if test1 <= 0 && test2 <= 0 {
        if test1 != 0 || test2 != 0 {
            // Certainly not collinear, segments intersect
            return true;
        }

        // Handle collinear case
        // Geometry.hpp:148-150
        if sign1.0 == 0 && sign1.1 == 0 {
            // Segments are collinear, need to check overlap
            // Simplified: return false for now
            // Full implementation would check if collinear segments overlap
            return false;
        }
    }

    false
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_trimmed_loop_new() {
        let trimmed = TrimmedLoop::new();
        assert!(trimmed.points.is_empty());
        assert!(trimmed.segments.is_empty());
        assert!(!trimmed.is_trimmed());
    }

    #[test]
    fn test_trimmed_loop_is_trimmed() {
        let mut trimmed = TrimmedLoop::new();
        assert!(!trimmed.is_trimmed());

        trimmed.segments.push(1);
        assert!(trimmed.is_trimmed());
    }

    #[test]
    fn test_segments_intersect_basic() {
        // Two segments that clearly intersect (crossing X)
        let p1 = Point::new(0, 0);
        let p2 = Point::new(10, 10);
        let p3 = Point::new(0, 10);
        let p4 = Point::new(10, 0);
        assert!(segments_intersect(p1, p2, p3, p4));

        // Two segments that don't intersect (parallel)
        let p1 = Point::new(0, 0);
        let p2 = Point::new(10, 0);
        let p3 = Point::new(0, 10);
        let p4 = Point::new(10, 10);
        assert!(!segments_intersect(p1, p2, p3, p4));
    }

    #[test]
    fn test_trim_loop_empty() {
        // Create a simple polygon
        let polygon = Polygon::new(vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 100),
        ]);

        // Create an empty edge grid
        let grid = EdgeGrid::new();

        // Trim the loop
        let result = trim_loop(&polygon, &grid);

        // Result should be empty since grid is empty
        assert!(!result.is_trimmed());
    }

    #[test]
    fn test_trim_loops_multiple() {
        let polygons = vec![
            Polygon::new(vec![
                Point::new(0, 0),
                Point::new(100, 0),
                Point::new(100, 100),
            ]),
            Polygon::new(vec![
                Point::new(200, 200),
                Point::new(300, 200),
                Point::new(300, 300),
            ]),
        ];

        let grid = EdgeGrid::new();
        let results = trim_loops(&polygons, &grid);

        assert_eq!(results.len(), 2);
        assert!(!results[0].is_trimmed());
        assert!(!results[1].is_trimmed());
    }
}
