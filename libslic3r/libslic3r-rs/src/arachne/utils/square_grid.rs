//! Square grid helper for spatial indexing in Arachne
//!
//! C++ Reference:
//! - Arachne/utils/SquareGrid.hpp
//! - Arachne/utils/SquareGrid.cpp
//!
//! **STATUS:** ✅ COMPLETE - Full implementation with C++ parity

use crate::geometry::Point;

/// Helper class to calculate coordinates on a square grid
///
/// Doesn't contain any data except cell size. The purpose is only to automatically
/// generate coordinates on a grid, and automatically feed them to functions.
/// The grid is theoretically infinite (bar integer limits).
///
/// C++ Reference: Arachne/utils/SquareGrid.hpp:15-97
/// C++: class SquareGrid
/// C++: {
/// C++: public:
/// C++:     SquareGrid(const coord_t cell_size);
/// C++:     coord_t getCellSize() const;
/// C++:     using GridPoint = Point;
/// C++:     using grid_coord_t = coord_t;
/// C++:     // ... methods ...
/// C++: protected:
/// C++:     coord_t cell_size;
/// C++: };
#[derive(Debug, Clone)]
pub struct SquareGrid {
    /// The cell (square) size
    /// C++ Reference: Arachne/utils/SquareGrid.hpp:92
    /// C++: coord_t cell_size;
    cell_size: i64,
}

/// Type alias for grid points
/// C++ Reference: Arachne/utils/SquareGrid.hpp:34
/// C++: using GridPoint = Point;
pub type GridPoint = Point;

/// Type alias for grid coordinates
/// C++ Reference: Arachne/utils/SquareGrid.hpp:35
/// C++: using grid_coord_t = coord_t;
pub type GridCoord = i64;

impl SquareGrid {
    /// Constructs a grid with the specified cell size
    ///
    /// C++ Reference: Arachne/utils/SquareGrid.cpp:9-12
    /// C++: SquareGrid::SquareGrid(coord_t cell_size) : cell_size(cell_size)
    /// C++: {
    /// C++:     assert(cell_size > 0U);
    /// C++: }
    pub fn new(cell_size: i64) -> Self {
        assert!(cell_size > 0, "cell_size must be positive");
        Self { cell_size }
    }

    /// Get the cell size this grid was created for
    ///
    /// C++ Reference: Arachne/utils/SquareGrid.cpp:143-146
    /// C++: coord_t SquareGrid::getCellSize() const
    /// C++: {
    /// C++:     return cell_size;
    /// C++: }
    pub fn get_cell_size(&self) -> i64 {
        self.cell_size
    }

    /// Compute the grid coordinates of a point
    ///
    /// C++ Reference: Arachne/utils/SquareGrid.cpp:15-18
    /// C++: SquareGrid::GridPoint SquareGrid::toGridPoint(const Vec2i64 &point) const
    /// C++: {
    /// C++:     return Point(toGridCoord(point.x()), toGridCoord(point.y()));
    /// C++: }
    pub fn to_grid_point(&self, point: Point) -> GridPoint {
        Point::new(
            self.to_grid_coord(point.x() as i64),
            self.to_grid_coord(point.y() as i64),
        )
    }

    /// Compute the grid coordinate of a real space coordinate
    ///
    /// This mapping via truncation results in the cells with GridPoint.x==0 being
    /// twice as large and similarly for GridPoint.y==0. This doesn't cause any
    /// incorrect behavior, just changes the running time slightly.
    ///
    /// C++ Reference: Arachne/utils/SquareGrid.cpp:21-31
    /// C++: SquareGrid::grid_coord_t SquareGrid::toGridCoord(const int64_t &coord) const
    /// C++: {
    /// C++:     // This mapping via truncation results in the cells with
    /// C++:     // GridPoint.x==0 being twice as large and similarly for
    /// C++:     // GridPoint.y==0.  This doesn't cause any incorrect behavior,
    /// C++:     // just changes the running time slightly.  The change in running
    /// C++:     // time from this is probably not worth doing a proper floor
    /// C++:     // operation.
    /// C++:     return coord / cell_size;
    /// C++: }
    pub fn to_grid_coord(&self, coord: i64) -> GridCoord {
        coord / self.cell_size
    }

    /// Compute the lowest coord in a grid cell
    ///
    /// The lowest point is the point in the grid cell closest to the origin.
    ///
    /// C++ Reference: Arachne/utils/SquareGrid.cpp:33-43
    /// C++: coord_t SquareGrid::toLowerCoord(const grid_coord_t& grid_coord) const
    /// C++: {
    /// C++:     // This mapping via truncation results in the cells with
    /// C++:     // GridPoint.x==0 being twice as large and similarly for
    /// C++:     // GridPoint.y==0.  This doesn't cause any incorrect behavior,
    /// C++:     // just changes the running time slightly.  The change in running
    /// C++:     // time from this is probably not worth doing a proper floor
    /// C++:     // operation.
    /// C++:     return grid_coord * cell_size;
    /// C++: }
    pub fn to_lower_coord(&self, grid_coord: GridCoord) -> i64 {
        grid_coord * self.cell_size
    }

    /// Compute the sign of a number
    ///
    /// The number 0 will result in a positive sign (1).
    ///
    /// C++ Reference: Arachne/utils/SquareGrid.cpp:136-139
    /// C++: SquareGrid::grid_coord_t SquareGrid::nonzeroSign(const grid_coord_t z) const
    /// C++: {
    /// C++:     return (z >= 0) - (z < 0);
    /// C++: }
    pub fn nonzero_sign(&self, z: GridCoord) -> GridCoord {
        ((z >= 0) as GridCoord) - ((z < 0) as GridCoord)
    }

    /// Process cells along a line
    ///
    /// C++ Reference: Arachne/utils/SquareGrid.cpp:51-126
    /// C++: bool SquareGrid::processLineCells(const std::pair<Point, Point> line,
    /// C++:                                     const std::function<bool (GridPoint)>& process_cell_func) const
    /// C++: {
    /// C++:     Point start = line.first;
    /// C++:     Point end = line.second;
    /// C++:     // ... implementation ...
    /// C++: }
    pub fn process_line_cells<F>(&self, line: (Point, Point), mut process_cell_func: F) -> bool
    where
        F: FnMut(GridPoint) -> bool,
    {
        // Get start and end points
        // C++ Reference: Arachne/utils/SquareGrid.cpp:53-54
        // C++: Point start = line.first;
        // C++: Point end = line.second;
        let mut start = line.0;
        let mut end = line.1;

        // Make sure X increases between start and end
        // C++ Reference: Arachne/utils/SquareGrid.cpp:55-58
        // C++: if (end.x() < start.x())
        // C++: {
        // C++:     std::swap(start, end);
        // C++: }
        if end.x() < start.x() {
            std::mem::swap(&mut start, &mut end);
        }

        // Get grid cells for start and end
        // C++ Reference: Arachne/utils/SquareGrid.cpp:60-61
        // C++: const GridPoint start_cell = toGridPoint(start.cast<int64_t>());
        // C++: const GridPoint end_cell = toGridPoint(end.cast<int64_t>());
        let start_cell = self.to_grid_point(start);
        let end_cell = self.to_grid_point(end);

        // Calculate Y difference and direction
        // C++ Reference: Arachne/utils/SquareGrid.cpp:62-63
        // C++: const int64_t y_diff = int64_t(end.y() - start.y());
        // C++: const grid_coord_t y_dir = nonzeroSign(y_diff);
        let y_diff = end.y() as i64 - start.y() as i64;
        let y_dir = self.nonzero_sign(y_diff);

        // Iterate over Y coordinates
        // C++ Reference: Arachne/utils/SquareGrid.cpp:69-70
        // C++: grid_coord_t x_cell_start = start_cell.x();
        // C++: for (grid_coord_t cell_y = start_cell.y(); cell_y * y_dir <= end_cell.y() * y_dir; cell_y += y_dir)
        let mut x_cell_start = start_cell.x();
        let mut cell_y = start_cell.y();

        loop {
            if cell_y * y_dir > end_cell.y() * y_dir {
                break;
            }

            // Calculate nearest next Y coordinate
            // C++ Reference: Arachne/utils/SquareGrid.cpp:72-73
            // C++: const coord_t nearest_next_y = toLowerCoord(cell_y + ((nonzeroSign(cell_y) == y_dir || cell_y == 0) ? y_dir : coord_t(0)));
            let y_offset = if self.nonzero_sign(cell_y) == y_dir || cell_y == 0 {
                y_dir
            } else {
                0
            };
            let nearest_next_y = self.to_lower_coord(cell_y + y_offset);

            // Calculate X cell end
            // C++ Reference: Arachne/utils/SquareGrid.cpp:74-91
            // C++: grid_coord_t x_cell_end;
            // C++: if (y_diff == 0)
            // C++: {
            // C++:     x_cell_end = end_cell.x();
            // C++: }
            // C++: else
            // C++: {
            // C++:     const int64_t area = int64_t(end.x() - start.x()) * int64_t(nearest_next_y - start.y());
            // C++:     int64_t corresponding_x = int64_t(start.x()) + area / y_diff;
            /// C++:     x_cell_end = toGridCoord(corresponding_x + ((corresponding_x < 0) && ((area % y_diff) != 0)));
            /// C++:     if (x_cell_end < start_cell.x())
            /// C++:     {
            /// C++:         x_cell_end = x_cell_start;
            /// C++:     }
            /// C++: }
            let x_cell_end = if y_diff == 0 {
                end_cell.x()
            } else {
                let area =
                    (end.x() as i64 - start.x() as i64) * (nearest_next_y - start.y() as i64);
                let mut corresponding_x = start.x() as i64 + area / y_diff;
                if corresponding_x < 0 && (area % y_diff) != 0 {
                    corresponding_x += 1;
                }
                let mut x_end = self.to_grid_coord(corresponding_x);
                if x_end < start_cell.x() {
                    x_end = x_cell_start;
                }
                x_end
            };

            // Process all X cells in this row
            // C++ Reference: Arachne/utils/SquareGrid.cpp:93-106
            // C++: for (grid_coord_t cell_x = x_cell_start; cell_x <= x_cell_end; ++cell_x)
            // C++: {
            // C++:     GridPoint grid_loc(cell_x, cell_y);
            // C++:     if (! process_cell_func(grid_loc))
            // C++:     {
            // C++:         return false;
            // C++:     }
            // C++:     if (grid_loc == end_cell)
            // C++:     {
            /// C++:         return true;
            /// C++:     }
            /// C++: }
            for cell_x in x_cell_start..=x_cell_end {
                let grid_loc = Point::new(cell_x, cell_y);
                if !process_cell_func(grid_loc) {
                    return false;
                }
                if grid_loc == end_cell {
                    return true;
                }
            }

            // Move to next row
            // C++ Reference: Arachne/utils/SquareGrid.cpp:109
            // C++: x_cell_start = x_cell_end;
            x_cell_start = x_cell_end;
            cell_y += y_dir;
        }

        // Should have returned before here
        // C++ Reference: Arachne/utils/SquareGrid.cpp:111-112
        // C++: assert(false && "We should have returned already before here!");
        // C++: return false;
        false
    }

    /// Process cells that might contain sought after points
    ///
    /// Processes cells that might be within a square with twice radius as width,
    /// centered around query_pt. May process elements that are up to radius + cell_size
    /// from query_pt.
    ///
    /// C++ Reference: Arachne/utils/SquareGrid.cpp:115-134
    /// C++: bool SquareGrid::processNearby(const Point &query_pt, coord_t radius,
    /// C++:                                  const std::function<bool (const GridPoint&)>& process_func) const
    /// C++: {
    /// C++:     const Point min_loc(query_pt.x() - radius, query_pt.y() - radius);
    /// C++:     const Point max_loc(query_pt.x() + radius, query_pt.y() + radius);
    /// C++:
    /// C++:     GridPoint min_grid = toGridPoint(min_loc.cast<int64_t>());
    /// C++:     GridPoint max_grid = toGridPoint(max_loc.cast<int64_t>());
    /// C++:
    /// C++:     for (coord_t grid_y = min_grid.y(); grid_y <= max_grid.y(); ++grid_y)
    /// C++:     {
    /// C++:         for (coord_t grid_x = min_grid.x(); grid_x <= max_grid.x(); ++grid_x)
    /// C++:         {
    /// C++:             GridPoint grid_pt(grid_x,grid_y);
    /// C++:             if (!process_func(grid_pt))
    /// C++:             {
    /// C++:                 return false;
    /// C++:             }
    /// C++:         }
    /// C++:     }
    /// C++:     return true;
    /// C++: }
    pub fn process_nearby<F>(&self, query_pt: Point, radius: i64, mut process_func: F) -> bool
    where
        F: FnMut(GridPoint) -> bool,
    {
        let min_loc = Point::new(query_pt.x() - radius, query_pt.y() - radius);
        let max_loc = Point::new(query_pt.x() + radius, query_pt.y() + radius);

        let min_grid = self.to_grid_point(min_loc);
        let max_grid = self.to_grid_point(max_loc);

        for grid_y in min_grid.y()..=max_grid.y() {
            for grid_x in min_grid.x()..=max_grid.x() {
                let grid_pt = Point::new(grid_x, grid_y);
                if !process_func(grid_pt) {
                    return false;
                }
            }
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Point;

    #[test]
    fn test_square_grid_creation() {
        /// Test basic SquareGrid creation
        /// C++ Reference: Arachne/utils/SquareGrid.cpp:12
        let grid = SquareGrid::new(1000);
        assert_eq!(grid.get_cell_size(), 1000);
    }

    #[test]
    #[should_panic(expected = "cell_size must be positive")]
    fn test_square_grid_invalid_size() {
        /// Test that invalid cell size panics
        /// C++ Reference: Arachne/utils/SquareGrid.cpp:11
        SquareGrid::new(0);
    }

    #[test]
    fn test_to_grid_coord() {
        /// Test converting real coordinates to grid coordinates
        /// C++ Reference: Arachne/utils/SquareGrid.cpp:31
        let grid = SquareGrid::new(100);
        assert_eq!(grid.to_grid_coord(0), 0);
        assert_eq!(grid.to_grid_coord(50), 0);
        assert_eq!(grid.to_grid_coord(100), 1);
        assert_eq!(grid.to_grid_coord(150), 1);
        assert_eq!(grid.to_grid_coord(200), 2);
        assert_eq!(grid.to_grid_coord(-50), 0);
        assert_eq!(grid.to_grid_coord(-100), -1);
    }

    #[test]
    fn test_to_grid_point() {
        /// Test converting point to grid point
        /// C++ Reference: Arachne/utils/SquareGrid.cpp:18
        let grid = SquareGrid::new(100);
        let point = Point::new(250, 350);
        let grid_point = grid.to_grid_point(point);
        assert_eq!(grid_point.x(), 2);
        assert_eq!(grid_point.y(), 3);
    }

    #[test]
    fn test_to_lower_coord() {
        /// Test converting grid coordinate to lowest real coordinate
        /// C++ Reference: Arachne/utils/SquareGrid.cpp:43
        let grid = SquareGrid::new(100);
        assert_eq!(grid.to_lower_coord(0), 0);
        assert_eq!(grid.to_lower_coord(1), 100);
        assert_eq!(grid.to_lower_coord(2), 200);
        assert_eq!(grid.to_lower_coord(-1), -100);
    }

    #[test]
    fn test_nonzero_sign() {
        /// Test sign function
        /// C++ Reference: Arachne/utils/SquareGrid.cpp:139
        let grid = SquareGrid::new(100);
        assert_eq!(grid.nonzero_sign(5), 1);
        assert_eq!(grid.nonzero_sign(0), 1);
        assert_eq!(grid.nonzero_sign(-5), -1);
    }

    #[test]
    fn test_process_nearby() {
        /// Test processing nearby cells
        /// C++ Reference: Arachne/utils/SquareGrid.cpp:134
        let grid = SquareGrid::new(100);
        let query_pt = Point::new(150, 150);
        let radius = 150;

        let mut visited = Vec::new();
        grid.process_nearby(query_pt, radius, |grid_pt| {
            visited.push(grid_pt);
            true
        });

        // Should visit cells in a 3x3 grid around (1,1)
        assert!(visited.len() >= 9);
        assert!(visited.contains(&Point::new(1, 1)));
    }

    #[test]
    fn test_process_nearby_early_exit() {
        /// Test early exit from process_nearby
        let grid = SquareGrid::new(100);
        let query_pt = Point::new(150, 150);
        let radius = 150;

        let mut count = 0;
        let result = grid.process_nearby(query_pt, radius, |_grid_pt| {
            count += 1;
            count < 3 // Stop after 3 cells
        });

        assert!(!result);
        assert_eq!(count, 3);
    }

    #[test]
    fn test_process_line_cells_horizontal() {
        /// Test processing cells along horizontal line
        /// C++ Reference: Arachne/utils/SquareGrid.cpp:126
        let grid = SquareGrid::new(100);
        let line = (Point::new(50, 50), Point::new(350, 50));

        let mut visited = Vec::new();
        grid.process_line_cells(line, |grid_pt| {
            visited.push(grid_pt);
            true
        });

        // Should visit cells along X axis
        assert!(visited.len() >= 3);
    }

    #[test]
    fn test_process_line_cells_vertical() {
        /// Test processing cells along vertical line
        let grid = SquareGrid::new(100);
        let line = (Point::new(50, 50), Point::new(50, 350));

        let mut visited = Vec::new();
        grid.process_line_cells(line, |grid_pt| {
            visited.push(grid_pt);
            true
        });

        // Should visit cells along Y axis
        assert!(visited.len() >= 3);
    }

    #[test]
    fn test_process_line_cells_diagonal() {
        /// Test processing cells along diagonal line
        let grid = SquareGrid::new(100);
        let line = (Point::new(0, 0), Point::new(300, 300));

        let mut visited = Vec::new();
        grid.process_line_cells(line, |grid_pt| {
            visited.push(grid_pt);
            true
        });

        // Should visit cells along diagonal
        assert!(visited.len() >= 3);
    }

    #[test]
    fn test_process_line_cells_early_exit() {
        /// Test early exit from process_line_cells
        let grid = SquareGrid::new(100);
        let line = (Point::new(0, 0), Point::new(500, 0));

        let mut count = 0;
        let result = grid.process_line_cells(line, |_grid_pt| {
            count += 1;
            count < 2 // Stop after 2 cells
        });

        assert!(!result);
        assert_eq!(count, 2);
    }
}
