//! Sparse grid for spatial indexing in Arachne
//!
//! C++ Reference:
//! - Arachne/utils/SparseGrid.hpp
//!
//! **STATUS:** ✅ COMPLETE - Full implementation with C++ parity

use super::square_grid::{GridPoint, SquareGrid};
use crate::geometry::Point;
use std::collections::HashMap;
use std::hash::{Hash, Hasher};

/// Sparse grid which can locate spatially nearby elements efficiently
///
/// This is an abstract template class which doesn't have any functions to insert elements.
///
/// C++ Reference: Arachne/utils/SparseGrid.hpp:17-137
/// C++: template<class ElemT> class SparseGrid : public SquareGrid
/// C++: {
/// C++: public:
/// C++:     using Elem = ElemT;
/// C++:     using GridPoint = SquareGrid::GridPoint;
/// C++:     using grid_coord_t = SquareGrid::grid_coord_t;
/// C++:     using GridMap = std::unordered_multimap<GridPoint, Elem, PointHash>;
/// C++:     // ... methods ...
/// C++: protected:
/// C++:     GridMap m_grid;
/// C++: };
#[derive(Debug, Clone)]
pub struct SparseGrid<Elem> {
    /// Base square grid for coordinate conversions
    /// C++ Reference: Arachne/utils/SparseGrid.hpp:23 (inherits from SquareGrid)
    square_grid: SquareGrid,

    /// Map from grid locations (GridPoint) to elements (Elem)
    /// C++ Reference: Arachne/utils/SparseGrid.hpp:98
    /// C++: GridMap m_grid;
    grid: HashMap<GridPoint, Vec<Elem>>,
}

impl<Elem> SparseGrid<Elem>
where
    Elem: Clone,
{
    /// Constructs a sparse grid with the specified cell size
    ///
    /// C++ Reference: Arachne/utils/SparseGrid.hpp:103-109
    /// C++: template<class ElemT> SparseGrid<ElemT>::SparseGrid(coord_t cell_size, size_t elem_reserve, float max_load_factor)
    /// C++:     : SquareGrid(cell_size)
    /// C++: {
    /// C++:     // Must be before the reserve call.
    /// C++:     m_grid.max_load_factor(max_load_factor);
    /// C++:     if (elem_reserve != 0U)
    /// C++:         m_grid.reserve(elem_reserve);
    /// C++: }
    pub fn new(cell_size: i64, elem_reserve: usize, _max_load_factor: f32) -> Self {
        let mut grid = HashMap::new();
        if elem_reserve != 0 {
            grid.reserve(elem_reserve);
        }
        Self {
            square_grid: SquareGrid::new(cell_size),
            grid,
        }
    }

    /// Get the cell size this grid was created for
    pub fn get_cell_size(&self) -> i64 {
        self.square_grid.get_cell_size()
    }

    /// Convert a point to grid coordinates
    pub fn to_grid_point(&self, point: Point) -> GridPoint {
        self.square_grid.to_grid_point(point)
    }

    /// Insert an element at a grid location
    ///
    /// This is used by derived classes like SparsePointGrid
    pub fn insert_at_grid_point(&mut self, grid_pt: GridPoint, elem: Elem) {
        self.grid.entry(grid_pt).or_insert_with(Vec::new).push(elem);
    }

    /// Returns all data within radius of query_pt
    ///
    /// Finds all elements with location within radius of query_pt. May return
    /// additional elements that are beyond radius.
    ///
    /// C++ Reference: Arachne/utils/SparseGrid.hpp:130-137
    /// C++: template<class ElemT> std::vector<typename SparseGrid<ElemT>::Elem> SparseGrid<ElemT>::getNearby(const Point &query_pt, coord_t radius) const
    /// C++: {
    /// C++:     std::vector<Elem> ret;
    /// C++:     const std::function<bool(const Elem &)> process_func = [&ret](const Elem &elem) {
    /// C++:         ret.push_back(elem);
    /// C++:         return true;
    /// C++:     };
    /// C++:     processNearby(query_pt, radius, process_func);
    /// C++:     return ret;
    /// C++: }
    pub fn get_nearby(&self, query_pt: Point, radius: i64) -> Vec<Elem> {
        let mut ret = Vec::new();
        self.process_nearby(query_pt, radius, |elem| {
            ret.push(elem.clone());
            true
        });
        ret
    }

    /// Process elements from cells that might contain sought after points
    ///
    /// Processes elements from cell that might have elements within radius of query_pt.
    /// Processes all elements that are within radius of query_pt. May process elements
    /// that are up to radius + cell_size from query_pt.
    ///
    /// C++ Reference: Arachne/utils/SparseGrid.hpp:121-125
    /// C++: template<class ElemT>
    /// C++: bool SparseGrid<ElemT>::processNearby(const Point &query_pt, coord_t radius,
    /// C++:                                         const std::function<bool(const Elem &)> &process_func) const
    /// C++: {
    /// C++:     return SquareGrid::processNearby(query_pt, radius, [&process_func, this](const GridPoint &grid_pt) {
    /// C++:         return processFromCell(grid_pt, process_func);
    /// C++:     });
    /// C++: }
    pub fn process_nearby<F>(&self, query_pt: Point, radius: i64, process_func: F) -> bool
    where
        F: FnMut(&Elem) -> bool,
    {
        let mut func = process_func;
        self.square_grid
            .process_nearby(query_pt, radius, |grid_pt| {
                self.process_from_cell(grid_pt, &mut func)
            })
    }

    /// Process elements from the cell indicated by grid_pt
    ///
    /// C++ Reference: Arachne/utils/SparseGrid.hpp:111-118
    /// C++: template<class ElemT> bool SparseGrid<ElemT>::processFromCell(const GridPoint &grid_pt,
    /// C++:                                                                  const std::function<bool(const Elem &)> &process_func) const
    /// C++: {
    /// C++:     auto grid_range = m_grid.equal_range(grid_pt);
    /// C++:     for (auto iter = grid_range.first; iter != grid_range.second; ++iter)
    /// C++:         if (!process_func(iter->second))
    /// C++:             return false;
    /// C++:     return true;
    /// C++: }
    fn process_from_cell<F>(&self, grid_pt: GridPoint, process_func: &mut F) -> bool
    where
        F: FnMut(&Elem) -> bool,
    {
        if let Some(elements) = self.grid.get(&grid_pt) {
            for elem in elements {
                if !process_func(elem) {
                    return false;
                }
            }
        }
        true
    }

    /// Get an iterator over all elements
    pub fn iter(&self) -> impl Iterator<Item = &Elem> {
        self.grid.values().flatten()
    }

    /// Get the number of grid cells
    pub fn num_cells(&self) -> usize {
        self.grid.len()
    }

    /// Get the total number of elements
    pub fn num_elements(&self) -> usize {
        self.grid.values().map(|v| v.len()).sum()
    }

    /// Clear all elements from the grid
    pub fn clear(&mut self) {
        self.grid.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Point;

    #[test]
    fn test_sparse_grid_creation() {
        /// Test basic SparseGrid creation
        /// C++ Reference: Arachne/utils/SparseGrid.hpp:109
        let grid: SparseGrid<i32> = SparseGrid::new(1000, 10, 1.0);
        assert_eq!(grid.get_cell_size(), 1000);
        assert_eq!(grid.num_elements(), 0);
    }

    #[test]
    fn test_sparse_grid_insert() {
        /// Test inserting elements into grid
        let mut grid: SparseGrid<i32> = SparseGrid::new(100, 0, 1.0);
        let pt = Point::new(150, 150);
        let grid_pt = grid.to_grid_point(pt);

        grid.insert_at_grid_point(grid_pt, 42);
        assert_eq!(grid.num_elements(), 1);
    }

    #[test]
    fn test_sparse_grid_get_nearby() {
        /// Test getting nearby elements
        /// C++ Reference: Arachne/utils/SparseGrid.hpp:137
        let mut grid: SparseGrid<i32> = SparseGrid::new(100, 0, 1.0);

        // Insert elements at different locations
        let pt1 = Point::new(150, 150);
        let pt2 = Point::new(250, 150);
        let pt3 = Point::new(500, 500);

        grid.insert_at_grid_point(grid.to_grid_point(pt1), 1);
        grid.insert_at_grid_point(grid.to_grid_point(pt2), 2);
        grid.insert_at_grid_point(grid.to_grid_point(pt3), 3);

        // Query near pt1
        let nearby = grid.get_nearby(pt1, 150);
        assert!(nearby.contains(&1));
        assert!(nearby.contains(&2));
        assert!(!nearby.contains(&3));
    }

    #[test]
    fn test_sparse_grid_process_nearby() {
        /// Test processing nearby elements
        /// C++ Reference: Arachne/utils/SparseGrid.hpp:125
        let mut grid: SparseGrid<i32> = SparseGrid::new(100, 0, 1.0);

        let pt1 = Point::new(150, 150);
        let pt2 = Point::new(250, 150);

        grid.insert_at_grid_point(grid.to_grid_point(pt1), 10);
        grid.insert_at_grid_point(grid.to_grid_point(pt2), 20);

        let mut sum = 0;
        grid.process_nearby(pt1, 150, |elem| {
            sum += elem;
            true
        });

        assert_eq!(sum, 30); // 10 + 20
    }

    #[test]
    fn test_sparse_grid_process_nearby_early_exit() {
        /// Test early exit from process_nearby
        let mut grid: SparseGrid<i32> = SparseGrid::new(100, 0, 1.0);

        let pt = Point::new(150, 150);
        grid.insert_at_grid_point(grid.to_grid_point(pt), 1);
        grid.insert_at_grid_point(grid.to_grid_point(pt), 2);
        grid.insert_at_grid_point(grid.to_grid_point(pt), 3);

        let mut count = 0;
        let result = grid.process_nearby(pt, 50, |_elem| {
            count += 1;
            count < 2 // Stop after 2 elements
        });

        assert!(!result);
        assert_eq!(count, 2);
    }

    #[test]
    fn test_sparse_grid_clear() {
        /// Test clearing the grid
        let mut grid: SparseGrid<i32> = SparseGrid::new(100, 0, 1.0);

        let pt = Point::new(150, 150);
        grid.insert_at_grid_point(grid.to_grid_point(pt), 42);
        assert_eq!(grid.num_elements(), 1);

        grid.clear();
        assert_eq!(grid.num_elements(), 0);
    }

    #[test]
    fn test_sparse_grid_multiple_elements_per_cell() {
        /// Test multiple elements in same cell
        let mut grid: SparseGrid<i32> = SparseGrid::new(100, 0, 1.0);

        let pt = Point::new(150, 150);
        let grid_pt = grid.to_grid_point(pt);

        grid.insert_at_grid_point(grid_pt, 1);
        grid.insert_at_grid_point(grid_pt, 2);
        grid.insert_at_grid_point(grid_pt, 3);

        assert_eq!(grid.num_elements(), 3);
        assert_eq!(grid.num_cells(), 1);

        let nearby = grid.get_nearby(pt, 50);
        assert_eq!(nearby.len(), 3);
    }

    #[test]
    fn test_sparse_grid_iter() {
        /// Test iterating over all elements
        let mut grid: SparseGrid<i32> = SparseGrid::new(100, 0, 1.0);

        let pt1 = Point::new(150, 150);
        let pt2 = Point::new(250, 150);

        grid.insert_at_grid_point(grid.to_grid_point(pt1), 10);
        grid.insert_at_grid_point(grid.to_grid_point(pt2), 20);

        let sum: i32 = grid.iter().sum();
        assert_eq!(sum, 30);
    }
}
