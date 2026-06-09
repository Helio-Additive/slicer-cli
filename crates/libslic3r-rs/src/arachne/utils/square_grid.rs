//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.
//
//! Square grid helper for spatial indexing in Arachne.
//!
//! C++ Reference:
//! - Arachne/utils/SquareGrid.hpp
//! - Arachne/utils/SquareGrid.cpp

use crate::geometry::Point;

/// Helper class to calculate coordinates on a square grid, and providing some
/// utility functions to process grids.
///
/// Doesn't contain any data, except cell size. The purpose is only to
/// automatically generate coordinates on a grid, and automatically feed them to
/// functions.
/// The grid is theoretically infinite (bar integer limits).
///
/// SquareGrid.hpp:25
#[derive(Debug, Clone)]
pub struct SquareGrid {
    /// The cell (square) size.
    /// SquareGrid.hpp:95
    cell_size: i64,
}

/// SquareGrid.hpp:38: using GridPoint = Point;
pub type GridPoint = Point;

/// SquareGrid.hpp:39: using grid_coord_t = coord_t;
pub type GridCoord = i64;

impl SquareGrid {
    /// Constructs a grid with the specified cell size.
    /// \param[in] cell_size The size to use for a cell (square) in the grid.
    ///
    /// SquareGrid.cpp:10
    pub fn new(cell_size: i64) -> Self {
        // SquareGrid.cpp:12: assert(cell_size > 0U);
        assert!(cell_size > 0);
        // SquareGrid.cpp:10: : cell_size(cell_size)
        Self { cell_size }
    }

    /// Compute the grid coordinates of a point.
    /// \param point The actual location.
    /// \return The grid coordinates that correspond to \p point.
    ///
    /// SquareGrid.cpp:16
    pub fn to_grid_point(&self, point: Point) -> GridPoint {
        // SquareGrid.cpp:18: return Point(toGridCoord(point.x()), toGridCoord(point.y()));
        Point::new(
            self.to_grid_coord(point.x() as i64),
            self.to_grid_coord(point.y() as i64),
        )
    }

    /// Compute the grid coordinate of a real space coordinate.
    /// \param coord The actual location.
    /// \return The grid coordinate that corresponds to \p coord.
    ///
    /// SquareGrid.cpp:22
    pub fn to_grid_coord(&self, coord: i64) -> GridCoord {
        // This mapping via truncation results in the cells with
        // GridPoint.x==0 being twice as large and similarly for
        // GridPoint.y==0.  This doesn't cause any incorrect behavior,
        // just changes the running time slightly.  The change in running
        // time from this is probably not worth doing a proper floor
        // operation.
        // SquareGrid.cpp:30: return coord / cell_size;
        coord / self.cell_size
    }

    /// Compute the lowest coord in a grid cell.
    /// The lowest point is the point in the grid cell closest to the origin.
    ///
    /// \param grid_coord The grid coordinate.
    /// \return The print space coordinate that corresponds to \p grid_coord.
    ///
    /// SquareGrid.cpp:33
    pub fn to_lower_coord(&self, grid_coord: GridCoord) -> i64 {
        // This mapping via truncation results in the cells with
        // GridPoint.x==0 being twice as large and similarly for
        // GridPoint.y==0.  This doesn't cause any incorrect behavior,
        // just changes the running time slightly.  The change in running
        // time from this is probably not worth doing a proper floor
        // operation.
        // SquareGrid.cpp:41: return grid_coord * cell_size;
        grid_coord * self.cell_size
    }

    /// Process cells along a line indicated by \p line.
    ///
    /// \param line The line along which to process cells.
    /// \param process_cell_func Processes each cell. ``process_cell_func(elem)``
    /// is called for each cell. Processing stops if function returns false.
    /// \return Whether we need to continue processing after this function.
    ///
    /// Note: the C++ non-const overload (SquareGrid.cpp:45-48) merely delegates
    /// to the const overload below; this single Rust method covers both.
    ///
    /// SquareGrid.cpp:51
    pub fn process_line_cells<F>(&self, line: (Point, Point), mut process_cell_func: F) -> bool
    where
        F: FnMut(GridPoint) -> bool,
    {
        // SquareGrid.cpp:53: Point start = line.first;
        // SquareGrid.cpp:54: Point end = line.second;
        let mut start = line.0;
        let mut end = line.1;
        // SquareGrid.cpp:55: if (end.x() < start.x())
        if end.x() < start.x() {
            // make sure X increases between start and end
            // SquareGrid.cpp:57: std::swap(start, end);
            std::mem::swap(&mut start, &mut end);
        }

        // SquareGrid.cpp:60: const GridPoint start_cell = toGridPoint(start.cast<int64_t>());
        let start_cell = self.to_grid_point(start);
        // SquareGrid.cpp:61: const GridPoint end_cell = toGridPoint(end.cast<int64_t>());
        let end_cell = self.to_grid_point(end);
        // SquareGrid.cpp:62: const int64_t y_diff = int64_t(end.y() - start.y());
        let y_diff = (end.y() - start.y()) as i64;
        // SquareGrid.cpp:63: const grid_coord_t y_dir = nonzeroSign(y_diff);
        let y_dir = self.nonzero_sign(y_diff);

        // This line drawing algorithm iterates over the range of Y coordinates, and
        // for each Y coordinate computes the range of X coordinates crossed in one
        // unit of Y. These ranges are rounded to be inclusive, so effectively this
        // creates a "fat" line, marking more cells than a strict one-cell-wide path.
        // SquareGrid.cpp:69: grid_coord_t x_cell_start = start_cell.x();
        let mut x_cell_start = start_cell.x();
        // SquareGrid.cpp:70: for (grid_coord_t cell_y = start_cell.y(); cell_y * y_dir <= end_cell.y() * y_dir; cell_y += y_dir)
        let mut cell_y = start_cell.y();
        while cell_y * y_dir <= end_cell.y() * y_dir {
            // for all Y from start to end
            // nearest y coordinate of the cells in the next row
            // SquareGrid.cpp:73: const coord_t nearest_next_y = toLowerCoord(cell_y + ((nonzeroSign(cell_y) == y_dir || cell_y == 0) ? y_dir : coord_t(0)));
            let nearest_next_y = self.to_lower_coord(
                cell_y
                    + if self.nonzero_sign(cell_y) == y_dir || cell_y == 0 {
                        y_dir
                    } else {
                        0
                    },
            );
            // SquareGrid.cpp:74: grid_coord_t x_cell_end; // the X coord of the last cell to include from this row
            let x_cell_end: GridCoord;
            // SquareGrid.cpp:75: if (y_diff == 0)
            if y_diff == 0 {
                // SquareGrid.cpp:77: x_cell_end = end_cell.x();
                x_cell_end = end_cell.x();
            } else {
                // SquareGrid.cpp:81: const int64_t area = int64_t(end.x() - start.x()) * int64_t(nearest_next_y - start.y());
                let area = (end.x() - start.x()) as i64 * (nearest_next_y - start.y() as i64) as i64;
                // corresponding_x: the x coordinate corresponding to nearest_next_y
                // SquareGrid.cpp:83: int64_t corresponding_x = int64_t(start.x()) + area / y_diff;
                let corresponding_x = start.x() as i64 + area / y_diff;
                // SquareGrid.cpp:84: x_cell_end = toGridCoord(corresponding_x + ((corresponding_x < 0) && ((area % y_diff) != 0)));
                let mut x_end = self.to_grid_coord(
                    corresponding_x + (((corresponding_x < 0) && ((area % y_diff) != 0)) as i64),
                );
                // SquareGrid.cpp:85: if (x_cell_end < start_cell.x())
                if x_end < start_cell.x() {
                    // process at least one cell!
                    // SquareGrid.cpp:87: x_cell_end = x_cell_start;
                    x_end = x_cell_start;
                }
                x_cell_end = x_end;
            }

            // SquareGrid.cpp:91: for (grid_coord_t cell_x = x_cell_start; cell_x <= x_cell_end; ++cell_x)
            let mut cell_x = x_cell_start;
            while cell_x <= x_cell_end {
                // SquareGrid.cpp:93: GridPoint grid_loc(cell_x, cell_y);
                let grid_loc = Point::new(cell_x, cell_y);
                // SquareGrid.cpp:94: if (! process_cell_func(grid_loc))
                if !process_cell_func(grid_loc) {
                    // SquareGrid.cpp:96: return false;
                    return false;
                }
                // SquareGrid.cpp:98: if (grid_loc == end_cell)
                if grid_loc == end_cell {
                    // SquareGrid.cpp:100: return true;
                    return true;
                }
                cell_x += 1;
            }
            // TODO: this causes at least a one cell overlap for each row, which
            // includes extra cells when crossing precisely on the corners
            // where positive slope where x > 0 and negative slope where x < 0
            // SquareGrid.cpp:106: x_cell_start = x_cell_end;
            x_cell_start = x_cell_end;

            // SquareGrid.cpp:70: cell_y += y_dir
            cell_y += y_dir;
        }
        // SquareGrid.cpp:108: assert(false && "We should have returned already before here!");
        // SquareGrid.cpp:109: return false;
        false
    }

    /// Process cells that might contain sought after points.
    ///
    /// Processes cells that might be within a square with twice \p radius as
    /// width, centered around \p query_pt.
    /// May process elements that are up to radius + cell_size from query_pt.
    /// \param query_pt The point to search around.
    /// \param radius The search radius.
    /// \param process_func Processes each cell. ``process_func(loc)`` is called
    /// for each cell coord within range. Processing stops if function returns
    /// ``false``.
    /// \return Whether we need to continue processing after this function.
    ///
    /// SquareGrid.cpp:112
    pub fn process_nearby<F>(&self, query_pt: Point, radius: i64, mut process_func: F) -> bool
    where
        F: FnMut(GridPoint) -> bool,
    {
        // SquareGrid.cpp:119: const Point min_loc(query_pt.x() - radius, query_pt.y() - radius);
        let min_loc = Point::new(query_pt.x() - radius, query_pt.y() - radius);
        // SquareGrid.cpp:120: const Point max_loc(query_pt.x() + radius, query_pt.y() + radius);
        let max_loc = Point::new(query_pt.x() + radius, query_pt.y() + radius);

        // SquareGrid.cpp:122: GridPoint min_grid = toGridPoint(min_loc.cast<int64_t>());
        let min_grid = self.to_grid_point(min_loc);
        // SquareGrid.cpp:123: GridPoint max_grid = toGridPoint(max_loc.cast<int64_t>());
        let max_grid = self.to_grid_point(max_loc);

        // SquareGrid.cpp:125: for (coord_t grid_y = min_grid.y(); grid_y <= max_grid.y(); ++grid_y)
        for grid_y in min_grid.y()..=max_grid.y() {
            // SquareGrid.cpp:127: for (coord_t grid_x = min_grid.x(); grid_x <= max_grid.x(); ++grid_x)
            for grid_x in min_grid.x()..=max_grid.x() {
                // SquareGrid.cpp:129: GridPoint grid_pt(grid_x,grid_y);
                let grid_pt = Point::new(grid_x, grid_y);
                // SquareGrid.cpp:130: if (!process_func(grid_pt))
                if !process_func(grid_pt) {
                    // SquareGrid.cpp:132: return false;
                    return false;
                }
            }
        }
        // SquareGrid.cpp:136: return true;
        true
    }

    /// Compute the sign of a number.
    ///
    /// The number 0 will result in a positive sign (1).
    /// \param z The number to find the sign of.
    /// \return 1 if the number is positive or 0, or -1 if the number is
    /// negative.
    ///
    /// SquareGrid.cpp:139
    pub fn nonzero_sign(&self, z: GridCoord) -> GridCoord {
        // SquareGrid.cpp:141: return (z >= 0) - (z < 0);
        ((z >= 0) as GridCoord) - ((z < 0) as GridCoord)
    }

    /// Get the cell size this grid was created for.
    ///
    /// SquareGrid.cpp:144
    pub fn get_cell_size(&self) -> i64 {
        // SquareGrid.cpp:146: return cell_size;
        self.cell_size
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Point;

    #[test]
    fn test_square_grid_creation() {
        // SquareGrid.cpp:10
        let grid = SquareGrid::new(1000);
        assert_eq!(grid.get_cell_size(), 1000);
    }

    #[test]
    #[should_panic]
    fn test_square_grid_invalid_size() {
        // SquareGrid.cpp:12: assert(cell_size > 0U);
        SquareGrid::new(0);
    }

    #[test]
    fn test_to_grid_coord() {
        // SquareGrid.cpp:30
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
        // SquareGrid.cpp:18
        let grid = SquareGrid::new(100);
        let point = Point::new(250, 350);
        let grid_point = grid.to_grid_point(point);
        assert_eq!(grid_point.x(), 2);
        assert_eq!(grid_point.y(), 3);
    }

    #[test]
    fn test_to_lower_coord() {
        // SquareGrid.cpp:41
        let grid = SquareGrid::new(100);
        assert_eq!(grid.to_lower_coord(0), 0);
        assert_eq!(grid.to_lower_coord(1), 100);
        assert_eq!(grid.to_lower_coord(2), 200);
        assert_eq!(grid.to_lower_coord(-1), -100);
    }

    #[test]
    fn test_nonzero_sign() {
        // SquareGrid.cpp:141
        let grid = SquareGrid::new(100);
        assert_eq!(grid.nonzero_sign(5), 1);
        assert_eq!(grid.nonzero_sign(0), 1);
        assert_eq!(grid.nonzero_sign(-5), -1);
    }

    #[test]
    fn test_process_nearby() {
        // SquareGrid.cpp:112
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
        // SquareGrid.cpp:51
        let grid = SquareGrid::new(100);
        let line = (Point::new(50, 50), Point::new(350, 50));

        let mut visited = Vec::new();
        grid.process_line_cells(line, |grid_pt| {
            visited.push(grid_pt);
            true
        });

        assert!(visited.len() >= 3);
    }

    #[test]
    fn test_process_line_cells_vertical() {
        let grid = SquareGrid::new(100);
        let line = (Point::new(50, 50), Point::new(50, 350));

        let mut visited = Vec::new();
        grid.process_line_cells(line, |grid_pt| {
            visited.push(grid_pt);
            true
        });

        assert!(visited.len() >= 3);
    }

    #[test]
    fn test_process_line_cells_diagonal() {
        let grid = SquareGrid::new(100);
        let line = (Point::new(0, 0), Point::new(300, 300));

        let mut visited = Vec::new();
        grid.process_line_cells(line, |grid_pt| {
            visited.push(grid_pt);
            true
        });

        assert!(visited.len() >= 3);
    }

    #[test]
    fn test_process_line_cells_early_exit() {
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
