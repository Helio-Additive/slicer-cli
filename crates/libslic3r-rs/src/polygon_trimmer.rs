//! Polygon trimming utilities.
//!
//! C++ Reference:
//! - PolygonTrimmer.hpp (32 lines)
//! - PolygonTrimmer.cpp (57 lines)
//!
//! 1:1 line-by-line port of `src/libslic3r/PolygonTrimmer.{hpp,cpp}`.

// PolygonTrimmer.cpp:1-3
// #include "PolygonTrimmer.hpp"
// #include "EdgeGrid.hpp"
// #include "Geometry.hpp"
use crate::edge_grid::EdgeGrid;
use crate::geometry::{segments_intersect, Point, Polygon, Polygons};

/// A polygon loop that has been trimmed against an EdgeGrid.
///
/// PolygonTrimmer.hpp:18-25
#[derive(Debug, Clone, Default)]
pub struct TrimmedLoop {
    /// The points of the trimmed loop.
    /// PolygonTrimmer.hpp:20 — std::vector<Point> points;
    pub points: Vec<Point>,

    /// Number of points per segment. Empty if the loop is
    /// PolygonTrimmer.hpp:21-22 — std::vector<unsigned int> segments;
    pub segments: Vec<u32>,
}

impl TrimmedLoop {
    /// Create a new empty trimmed loop.
    pub fn new() -> Self {
        Self {
            points: Vec::new(),
            segments: Vec::new(),
        }
    }

    /// PolygonTrimmer.hpp:24
    /// C++: bool is_trimmed() const { return ! segments.empty(); }
    pub fn is_trimmed(&self) -> bool {
        !self.segments.is_empty()
    }
}

/// PolygonTrimmer.cpp:7
/// TrimmedLoop trim_loop(const Polygon &loop, const EdgeGrid::Grid &grid)
pub fn trim_loop(loop_polygon: &Polygon, grid: &EdgeGrid) -> TrimmedLoop {
    // PolygonTrimmer.cpp:9-10
    // assert(! loop.empty());
    // assert(loop.size() >= 2);
    assert!(!loop_polygon.points().is_empty());
    assert!(loop_polygon.len() >= 2);

    // PolygonTrimmer.cpp:12
    // TrimmedLoop out;
    let out = TrimmedLoop::new();

    // PolygonTrimmer.cpp:14
    // if (loop.size() >= 2) {
    if loop_polygon.len() >= 2 {
        // PolygonTrimmer.cpp:16-36
        // struct Visitor { ... } visitor(grid, &loop.points.back(), nullptr);
        struct Visitor<'a> {
            // PolygonTrimmer.cpp:33-35
            grid: &'a EdgeGrid,
            pt_this: Option<Point>,
            pt_prev: Option<Point>,
        }

        impl Visitor<'_> {
            // PolygonTrimmer.cpp:19
            // bool operator()(coord_t iy, coord_t ix) {
            fn visit(&self, iy: usize, ix: usize) -> bool {
                // Called with a row and colum of the grid cell, which is intersected by a line.
                // PolygonTrimmer.cpp:21
                // auto cell_data_range = grid.cell_data_range(iy, ix);
                let cell_data_range = self.grid.cell_data_range_at(iy, ix);
                // PolygonTrimmer.cpp:22
                // for (auto it_contour_and_segment = cell_data_range.first; it_contour_and_segment != cell_data_range.second; ++ it_contour_and_segment) {
                for it_contour_and_segment in cell_data_range {
                    // End points of the line segment and their vector.
                    // PolygonTrimmer.cpp:24
                    // auto segment = grid.segment(*it_contour_and_segment);
                    let segment = self.grid.segment(*it_contour_and_segment);
                    // PolygonTrimmer.cpp:25
                    // if (Geometry::segments_intersect(segment.first, segment.second, *pt_prev, *pt_this)) {
                    if segments_intersect(
                        segment.a,
                        segment.b,
                        self.pt_prev.unwrap(),
                        self.pt_this.unwrap(),
                    ) {
                        // The two segments intersect. Add them to the output.
                        // PolygonTrimmer.cpp:26-27
                    }
                }
                // Continue traversing the grid along the edge.
                // PolygonTrimmer.cpp:30
                true
            }
        }

        // PolygonTrimmer.cpp:36
        // } visitor(grid, &loop.points.back(), nullptr);
        let mut visitor = Visitor {
            grid,
            pt_this: None,
            pt_prev: Some(*loop_polygon.points().last().unwrap()),
        };

        // PolygonTrimmer.cpp:38
        // for (const Point &pt_this : loop.points) {
        for pt_this in loop_polygon.points() {
            // PolygonTrimmer.cpp:39
            // visitor.pt_this = &pt_this;
            visitor.pt_this = Some(*pt_this);
            // PolygonTrimmer.cpp:40
            // grid.visit_cells_intersecting_line(*visitor.pt_prev, pt_this, visitor);
            grid.visit_cells_intersecting_line(visitor.pt_prev.unwrap(), *pt_this, |iy, ix| {
                visitor.visit(iy, ix);
            });
            // PolygonTrimmer.cpp:41
            // visitor.pt_prev = &pt_this;
            visitor.pt_prev = Some(*pt_this);
        }
    }

    // PolygonTrimmer.cpp:45
    // return out;
    out
}

/// PolygonTrimmer.cpp:48
/// std::vector<TrimmedLoop> trim_loops(const Polygons &loops, const EdgeGrid::Grid &grid)
pub fn trim_loops(loops: &Polygons, grid: &EdgeGrid) -> Vec<TrimmedLoop> {
    // PolygonTrimmer.cpp:50
    // std::vector<TrimmedLoop> out;
    // PolygonTrimmer.cpp:51
    // out.reserve(loops.size());
    let mut out = Vec::with_capacity(loops.len());
    // PolygonTrimmer.cpp:52-53
    // for (const Polygon &loop : loops)
    //     out.emplace_back(trim_loop(loop, grid));
    for loop_polygon in loops {
        out.push(trim_loop(loop_polygon, grid));
    }
    // PolygonTrimmer.cpp:54
    // return out;
    out
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
    fn test_trim_loop_empty_grid() {
        // A simple polygon trimmed against an empty grid produces an untrimmed loop
        // (the C++ never writes into `out`, so it is always default-constructed).
        let polygon = Polygon::new(vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 100),
        ]);
        let grid = EdgeGrid::new();
        let result = trim_loop(&polygon, &grid);
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
