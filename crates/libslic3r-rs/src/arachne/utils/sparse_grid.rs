//! Sparse grid which can locate spatially nearby elements efficiently.
//!
//! C++ Reference: Arachne/utils/SparseGrid.hpp (header-only template class)
//!
//! Copyright (c) 2016 Scott Lenser
//! Copyright (c) 2018 Ultimaker B.V.
//! CuraEngine is released under the terms of the AGPLv3 or higher.

use super::square_grid::{GridPoint, SquareGrid};
use crate::geometry::Point;
use std::collections::HashMap;

/// Sparse grid which can locate spatially nearby elements efficiently.
///
/// \note This is an abstract template class which doesn't have any functions to insert elements.
/// \see SparsePointGrid
///
/// \tparam ElemT The element type to store.
///
/// C++ Reference: Arachne/utils/SparseGrid.hpp:25-95
/// C++: template<class ElemT> class SparseGrid : public SquareGrid
/// C++: {
/// C++: public:
/// C++:     using Elem = ElemT;                                                  // :28
/// C++:     using GridPoint    = SquareGrid::GridPoint;                          // :30
/// C++:     using grid_coord_t = SquareGrid::grid_coord_t;                       // :31
/// C++:     using GridMap       = std::unordered_multimap<GridPoint, Elem, PointHash>; // :32
/// C++:     using iterator       = typename GridMap::iterator;                   // :34
/// C++:     using const_iterator = typename GridMap::const_iterator;             // :35
/// C++: protected:
/// C++:     GridMap m_grid;                                                      // :94
/// C++: };
///
/// `std::unordered_multimap<GridPoint, Elem, PointHash>` is represented here as
/// `HashMap<GridPoint, Vec<Elem>>`: each grid cell maps to the list of elements
/// hashed into it (a multimap with `GridPoint` keys), preserving insertion order
/// within a cell so iteration matches the C++ `equal_range` traversal.
#[derive(Debug, Clone)]
pub struct SparseGrid<Elem> {
    /// Base square grid for coordinate conversions.
    /// C++ Reference: Arachne/utils/SparseGrid.hpp:25 (`: public SquareGrid`)
    square_grid: SquareGrid,

    /// Map from grid locations (GridPoint) to elements (Elem).
    /// C++ Reference: Arachne/utils/SparseGrid.hpp:94
    /// C++: GridMap m_grid;
    grid: HashMap<GridPoint, Vec<Elem>>,
}

impl<Elem> SparseGrid<Elem>
where
    Elem: Clone,
{
    /// Constructs a sparse grid with the specified cell size.
    ///
    /// \param[in] cell_size The size to use for a cell (square) in the grid.
    ///    Typical values would be around 0.5-2x of expected query radius.
    /// \param[in] elem_reserve Number of elements to research space for.
    /// \param[in] max_load_factor Maximum average load factor before rehashing.
    ///
    /// C++ Reference: Arachne/utils/SparseGrid.hpp:97-103
    /// C++: template<class ElemT> SparseGrid<ElemT>::SparseGrid(coord_t cell_size, size_t elem_reserve, float max_load_factor) : SquareGrid(cell_size)
    /// C++: {
    /// C++:     // Must be before the reserve call.
    /// C++:     m_grid.max_load_factor(max_load_factor);
    /// C++:     if (elem_reserve != 0U)
    /// C++:         m_grid.reserve(elem_reserve);
    /// C++: }
    ///
    /// `max_load_factor` only governs the C++ hash table's rehashing threshold
    /// (a performance knob, not an output-affecting parameter); Rust's `HashMap`
    /// has no equivalent setter, so it is accepted and ignored.
    pub fn new(cell_size: i64, elem_reserve: usize, _max_load_factor: f32) -> Self {
        // Must be before the reserve call.
        let mut grid = HashMap::new();
        if elem_reserve != 0 {
            grid.reserve(elem_reserve);
        }
        Self {
            square_grid: SquareGrid::new(cell_size),
            grid,
        }
    }

    // --- Helpers exposing the protected base / `m_grid` to derived grids ---
    // C++ uses public inheritance from SquareGrid plus a protected `m_grid`
    // member that `SparsePointGrid` / `SparseLineGrid` access directly. Rust has
    // no protected inheritance, so the equivalent base operations are forwarded
    // through these thin wrappers used by the derived grid types.

    /// Get the cell size this grid was created for.
    /// C++ Reference: Arachne/utils/SquareGrid.hpp:36 (inherited `getCellSize`)
    pub fn get_cell_size(&self) -> i64 {
        self.square_grid.get_cell_size()
    }

    /// Compute the grid coordinates of a point.
    /// C++ Reference: Arachne/utils/SquareGrid.hpp:77 (inherited `toGridPoint`)
    pub fn to_grid_point(&self, point: Point) -> GridPoint {
        self.square_grid.to_grid_point(point)
    }

    /// Insert an element at a grid location.
    ///
    /// Mirrors `m_grid.emplace(grid_loc, elem)` as performed by the derived
    /// `SparsePointGrid` / `SparseLineGrid` insert paths.
    /// C++ Reference: Arachne/utils/SparseGrid.hpp:94 (`m_grid`)
    pub fn insert_at_grid_point(&mut self, grid_pt: GridPoint, elem: Elem) {
        self.grid.entry(grid_pt).or_default().push(elem);
    }

    /// Process cells along a line indicated by `line`.
    ///
    /// Inherited from `SquareGrid` in C++. Provided here so that derived
    /// classes such as `SparseLineGrid` can invoke it via the base
    /// `SparseGrid` (mirroring `SparseGrid<ElemT>::processLineCells(...)`).
    ///
    /// C++ Reference: Arachne/utils/SquareGrid.hpp:48-57 (inherited)
    pub fn process_line_cells<F>(&self, line: (Point, Point), process_cell_func: F) -> bool
    where
        F: FnMut(GridPoint) -> bool,
    {
        self.square_grid.process_line_cells(line, process_cell_func)
    }

    /// Returns all data within radius of query_pt.
    ///
    /// Finds all elements with location within radius of \p query_pt.  May
    /// return additional elements that are beyond radius.
    ///
    /// \param[in] query_pt The point to search around.
    /// \param[in] radius The search radius.
    /// \return Vector of elements found
    ///
    /// C++ Reference: Arachne/utils/SparseGrid.hpp:120-129
    /// C++: template<class ElemT> std::vector<typename SparseGrid<ElemT>::Elem> SparseGrid<ElemT>::getNearby(const Point &query_pt, coord_t radius) const
    /// C++: {
    /// C++:     std::vector<Elem>                       ret;
    /// C++:     const std::function<bool(const Elem &)> process_func = [&ret](const Elem &elem) {
    /// C++:         ret.push_back(elem);
    /// C++:         return true;
    /// C++:     };
    /// C++:     processNearby(query_pt, radius, process_func);
    /// C++:     return ret;
    /// C++: }
    pub fn get_nearby(&self, query_pt: Point, radius: i64) -> Vec<Elem> {
        let mut ret: Vec<Elem> = Vec::new();
        let process_func = |elem: &Elem| {
            ret.push(elem.clone());
            true
        };
        self.process_nearby(query_pt, radius, process_func);
        ret
    }

    /// Process elements from cells that might contain sought after points.
    ///
    /// Processes elements from cell that might have elements within \p
    /// radius of \p query_pt.  Processes all elements that are within
    /// radius of query_pt.  May process elements that are up to radius +
    /// cell_size from query_pt.
    ///
    /// \param[in] query_pt The point to search around.
    /// \param[in] radius The search radius.
    /// \param[in] process_func Processes each element.  process_func(elem) is
    ///    called for each element in the cell. Processing stops if function returns false.
    /// \return Whether we need to continue processing after this function
    ///
    /// C++ Reference: Arachne/utils/SparseGrid.hpp:114-118
    /// C++: template<class ElemT>
    /// C++: bool SparseGrid<ElemT>::processNearby(const Point &query_pt, coord_t radius, const std::function<bool(const Elem &)> &process_func) const
    /// C++: {
    /// C++:     return SquareGrid::processNearby(query_pt, radius, [&process_func, this](const GridPoint &grid_pt) { return processFromCell(grid_pt, process_func); });
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

    /// Process elements from the cell indicated by \p grid_pt.
    ///
    /// \param[in] grid_pt The grid coordinates of the cell.
    /// \param[in] process_func Processes each element.  process_func(elem) is
    ///    called for each element in the cell. Processing stops if function returns false.
    /// \return Whether we need to continue processing a next cell.
    ///
    /// C++ Reference: Arachne/utils/SparseGrid.hpp:105-112
    /// C++: template<class ElemT> bool SparseGrid<ElemT>::processFromCell(const GridPoint &grid_pt, const std::function<bool(const Elem &)> &process_func) const
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

    // --- Iteration / introspection helpers ---
    // C++ exposes `begin()`/`end()` over `m_grid` (SparseGrid.hpp:46-49). Rust
    // provides an iterator over the stored elements plus small introspection
    // helpers used by the derived grid types and tests.

    /// Get an iterator over all elements.
    /// C++ Reference: Arachne/utils/SparseGrid.hpp:46-49 (`begin`/`end`)
    pub fn iter(&self) -> impl Iterator<Item = &Elem> {
        self.grid.values().flatten()
    }

    /// Get the number of grid cells.
    pub fn num_cells(&self) -> usize {
        self.grid.len()
    }

    /// Get the total number of elements.
    pub fn num_elements(&self) -> usize {
        self.grid.values().map(|v| v.len()).sum()
    }

    /// Clear all elements from the grid.
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
        // Test basic SparseGrid creation.
        // C++ Reference: Arachne/utils/SparseGrid.hpp:97
        let grid: SparseGrid<i32> = SparseGrid::new(1000, 10, 1.0);
        assert_eq!(grid.get_cell_size(), 1000);
        assert_eq!(grid.num_elements(), 0);
    }

    #[test]
    fn test_sparse_grid_insert() {
        // Test inserting elements into grid.
        let mut grid: SparseGrid<i32> = SparseGrid::new(100, 0, 1.0);
        let pt = Point::new(150, 150);
        let grid_pt = grid.to_grid_point(pt);

        grid.insert_at_grid_point(grid_pt, 42);
        assert_eq!(grid.num_elements(), 1);
    }

    #[test]
    fn test_sparse_grid_get_nearby() {
        // Test getting nearby elements.
        // C++ Reference: Arachne/utils/SparseGrid.hpp:120
        let mut grid: SparseGrid<i32> = SparseGrid::new(100, 0, 1.0);

        let pt1 = Point::new(150, 150);
        let pt2 = Point::new(250, 150);
        let pt3 = Point::new(500, 500);

        grid.insert_at_grid_point(grid.to_grid_point(pt1), 1);
        grid.insert_at_grid_point(grid.to_grid_point(pt2), 2);
        grid.insert_at_grid_point(grid.to_grid_point(pt3), 3);

        let nearby = grid.get_nearby(pt1, 150);
        assert!(nearby.contains(&1));
        assert!(nearby.contains(&2));
        assert!(!nearby.contains(&3));
    }

    #[test]
    fn test_sparse_grid_process_nearby() {
        // Test processing nearby elements.
        // C++ Reference: Arachne/utils/SparseGrid.hpp:114
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
        // Test early exit from process_nearby.
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
        // Test clearing the grid.
        let mut grid: SparseGrid<i32> = SparseGrid::new(100, 0, 1.0);

        let pt = Point::new(150, 150);
        grid.insert_at_grid_point(grid.to_grid_point(pt), 42);
        assert_eq!(grid.num_elements(), 1);

        grid.clear();
        assert_eq!(grid.num_elements(), 0);
    }

    #[test]
    fn test_sparse_grid_multiple_elements_per_cell() {
        // Test multiple elements in same cell.
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
        // Test iterating over all elements.
        let mut grid: SparseGrid<i32> = SparseGrid::new(100, 0, 1.0);

        let pt1 = Point::new(150, 150);
        let pt2 = Point::new(250, 150);

        grid.insert_at_grid_point(grid.to_grid_point(pt1), 10);
        grid.insert_at_grid_point(grid.to_grid_point(pt2), 20);

        let sum: i32 = grid.iter().sum();
        assert_eq!(sum, 30);
    }
}
