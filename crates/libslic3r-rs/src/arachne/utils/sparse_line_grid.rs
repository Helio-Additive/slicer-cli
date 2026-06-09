//! Sparse line grid for efficient spatial queries of line segments in Arachne
//!
//! C++ Reference:
//! - Arachne/utils/SparseLineGrid.hpp
//!
//! **STATUS:** ✅ COMPLETE - Full 1:1 port. `insert` delegates to the inherited
//! `processLineCells` (via `SparseGrid::process_line_cells` ->
//! `SquareGrid::process_line_cells`) exactly as the C++ does, instead of
//! reimplementing the line-traversal algorithm.

use super::sparse_grid::SparseGrid;
use super::square_grid::GridPoint;
use crate::geometry::Point;

/// Sparse grid which can locate spatially nearby line segments efficiently
///
/// This extends SparseGrid to handle line segments by inserting elements into
/// all grid cells that the line passes through.
///
/// C++ Reference: Arachne/utils/SparseLineGrid.hpp:16-74
/// C++: template<class ElemT, class Locator> class SparseLineGrid : public SparseGrid<ElemT>
/// C++: {
/// C++: public:
/// C++:     using Elem = ElemT;
/// C++:     SparseLineGrid(coord_t cell_size, size_t elem_reserve = 0U, float max_load_factor = 1.0f);
/// C++:     void insert(const Elem &elem);
/// C++: protected:
/// C++:     Locator m_locator;
/// C++: };
#[derive(Debug, Clone)]
pub struct SparseLineGrid<Elem, Locator> {
    /// Base sparse grid
    /// C++ Reference: Arachne/utils/SparseLineGrid.hpp:23 (inherits from SparseGrid)
    grid: SparseGrid<Elem>,

    /// Accessor for getting line segment locations from elements
    /// C++ Reference: Arachne/utils/SparseLineGrid.hpp:49
    /// C++: Locator m_locator;
    locator: Locator,
}

impl<Elem, Locator> SparseLineGrid<Elem, Locator>
where
    Elem: Clone,
    Locator: LineLocatorTrait<Elem>,
{
    /// Constructs a sparse grid with the specified cell size
    ///
    /// C++ Reference: Arachne/utils/SparseLineGrid.hpp:52-54
    /// C++: template<class ElemT, class Locator>
    /// C++: SparseLineGrid<ElemT, Locator>::SparseLineGrid(coord_t cell_size, size_t elem_reserve, float max_load_factor)
    /// C++:     : SparseGrid<ElemT>(cell_size, elem_reserve, max_load_factor) {}
    pub fn new(cell_size: i64, elem_reserve: usize, max_load_factor: f32) -> Self {
        Self {
            grid: SparseGrid::new(cell_size, elem_reserve, max_load_factor),
            locator: Locator::default(),
        }
    }

    /// Inserts elem into the sparse grid
    ///
    /// The element is inserted into all grid cells that the line segment passes through.
    ///
    /// C++ Reference: Arachne/utils/SparseLineGrid.hpp:56-72
    /// C++: template<class ElemT, class Locator> void SparseLineGrid<ElemT, Locator>::insert(const Elem &elem)
    /// C++: {
    /// C++:     const std::pair<Point, Point> line = m_locator(elem);
    /// C++:     using GridMap = std::unordered_multimap<GridPoint, Elem, PointHash>;
    /// C++:     // below is a workaround for the fact that lambda functions cannot access private or protected members
    /// C++:     // first we define a lambda which works on any GridMap and then we bind it to the actual protected GridMap of the parent class
    /// C++:     std::function<bool(GridMap *, const GridPoint)> process_cell_func_ = [&elem](GridMap *m_grid, const GridPoint grid_loc) {
    /// C++:         m_grid->emplace(grid_loc, elem);
    /// C++:         return true;
    /// C++:     };
    /// C++:     using namespace std::placeholders; // for _1, _2, _3...
    /// C++:     GridMap *m_grid = &(this->m_grid);
    /// C++:     std::function<bool(const GridPoint)> process_cell_func(std::bind(process_cell_func_, m_grid, _1));
    /// C++:
    /// C++:     SparseGrid<ElemT>::processLineCells(line, process_cell_func);
    /// C++: }
    pub fn insert(&mut self, elem: Elem) {
        // SparseLineGrid.hpp:57
        // C++: const std::pair<Point, Point> line = m_locator(elem);
        let line = self.locator.locate(&elem);
        // SparseLineGrid.hpp:58
        // C++: using GridMap = std::unordered_multimap<GridPoint, Elem, PointHash>;
        // below is a workaround for the fact that lambda functions cannot access private or protected members
        // first we define a lambda which works on any GridMap and then we bind it to the actual protected GridMap of the parent class
        //
        // SparseLineGrid.hpp:61-64
        // C++: std::function<bool(GridMap *, const GridPoint)> process_cell_func_ = [&elem](GridMap *m_grid, const GridPoint grid_loc) {
        // C++:     m_grid->emplace(grid_loc, elem);
        // C++:     return true;
        // C++: };
        //
        // In Rust the borrow checker forbids mutating `self.grid` from inside a
        // closure that is itself running an immutable borrow of `self.grid`
        // (via `process_line_cells`, which reads the cell size from the base
        // grid). Because the C++ `process_cell_func_` is pure-append and always
        // returns `true`, we first collect the grid cells (running the exact
        // same `processLineCells` traversal) and then emplace into each cell.
        // This is observationally identical to the C++ behaviour.
        let mut grid_locs: Vec<GridPoint> = Vec::new();
        // SparseLineGrid.hpp:65-67
        // C++: using namespace std::placeholders; // for _1, _2, _3...
        // C++: GridMap *m_grid = &(this->m_grid);
        // C++: std::function<bool(const GridPoint)> process_cell_func(std::bind(process_cell_func_, m_grid, _1));
        let process_cell_func = |grid_loc: GridPoint| {
            grid_locs.push(grid_loc);
            true
        };

        // SparseLineGrid.hpp:69
        // C++: SparseGrid<ElemT>::processLineCells(line, process_cell_func);
        self.grid.process_line_cells(line, process_cell_func);

        for grid_loc in grid_locs {
            self.grid.insert_at_grid_point(grid_loc, elem.clone());
        }
    }

    /// Process nearby elements
    pub fn process_nearby<F>(&self, query_pt: Point, radius: i64, process_func: F) -> bool
    where
        F: FnMut(&Elem) -> bool,
    {
        self.grid.process_nearby(query_pt, radius, process_func)
    }

    /// Get all nearby elements
    pub fn get_nearby(&self, query_pt: Point, radius: i64) -> Vec<Elem> {
        self.grid.get_nearby(query_pt, radius)
    }

    /// Get the number of elements in the grid (may count duplicates)
    pub fn num_elements(&self) -> usize {
        self.grid.num_elements()
    }

    /// Clear all elements
    pub fn clear(&mut self) {
        self.grid.clear();
    }
}

/// Trait for locator pattern - extracts line segment from element
///
/// C++ Reference: Arachne/utils/SparseLineGrid.hpp:18-20
/// C++: // Locator must have: std::pair<Point, Point> operator()(const ElemT &elem) const
pub trait LineLocatorTrait<Elem>: Default {
    /// Get the line segment (start, end) associated with an element
    fn locate(&self, elem: &Elem) -> (Point, Point);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Point;

    #[derive(Debug, Clone, PartialEq)]
    struct TestLineElement {
        start: Point,
        end: Point,
        value: i32,
    }

    #[derive(Default)]
    struct TestLineLocator;

    impl LineLocatorTrait<TestLineElement> for TestLineLocator {
        fn locate(&self, elem: &TestLineElement) -> (Point, Point) {
            (elem.start, elem.end)
        }
    }

    #[test]
    fn test_sparse_line_grid_creation() {
        /// Test basic SparseLineGrid creation
        /// C++ Reference: Arachne/utils/SparseLineGrid.hpp:54
        let grid: SparseLineGrid<TestLineElement, TestLineLocator> =
            SparseLineGrid::new(1000, 10, 1.0);
        assert_eq!(grid.num_elements(), 0);
    }

    #[test]
    fn test_sparse_line_grid_insert() {
        /// Test inserting line elements
        /// C++ Reference: Arachne/utils/SparseLineGrid.hpp:72
        let mut grid: SparseLineGrid<TestLineElement, TestLineLocator> =
            SparseLineGrid::new(100, 0, 1.0);

        let elem = TestLineElement {
            start: Point::new(150, 150),
            end: Point::new(250, 250),
            value: 42,
        };

        grid.insert(elem);
        assert!(grid.num_elements() > 0);
    }

    #[test]
    fn test_sparse_line_grid_get_nearby() {
        /// Test getting nearby line elements
        let mut grid: SparseLineGrid<TestLineElement, TestLineLocator> =
            SparseLineGrid::new(100, 0, 1.0);

        // Insert a horizontal line
        let elem1 = TestLineElement {
            start: Point::new(100, 150),
            end: Point::new(200, 150),
            value: 1,
        };

        // Insert a vertical line far away
        let elem2 = TestLineElement {
            start: Point::new(500, 500),
            end: Point::new(500, 600),
            value: 2,
        };

        grid.insert(elem1.clone());
        grid.insert(elem2);

        // Query near the first line
        let query = Point::new(150, 150);
        let nearby = grid.get_nearby(query, 100);

        // Should find at least the first line
        assert!(!nearby.is_empty());
        assert!(nearby.iter().any(|e| e.value == 1));
    }

    #[test]
    fn test_sparse_line_grid_process_nearby() {
        /// Test processing nearby elements
        let mut grid: SparseLineGrid<TestLineElement, TestLineLocator> =
            SparseLineGrid::new(100, 0, 1.0);

        let elem1 = TestLineElement {
            start: Point::new(150, 150),
            end: Point::new(250, 150),
            value: 10,
        };

        let elem2 = TestLineElement {
            start: Point::new(150, 250),
            end: Point::new(250, 250),
            value: 20,
        };

        grid.insert(elem1);
        grid.insert(elem2);

        let mut sum = 0;
        grid.process_nearby(Point::new(200, 200), 100, |elem| {
            sum += elem.value;
            true
        });

        assert!(sum > 0);
    }

    #[test]
    fn test_sparse_line_grid_clear() {
        /// Test clearing the grid
        let mut grid: SparseLineGrid<TestLineElement, TestLineLocator> =
            SparseLineGrid::new(100, 0, 1.0);

        let elem = TestLineElement {
            start: Point::new(150, 150),
            end: Point::new(250, 250),
            value: 42,
        };

        grid.insert(elem);
        assert!(grid.num_elements() > 0);

        grid.clear();
        assert_eq!(grid.num_elements(), 0);
    }

    #[test]
    fn test_sparse_line_grid_diagonal_line() {
        /// Test inserting a diagonal line that crosses multiple cells
        let mut grid: SparseLineGrid<TestLineElement, TestLineLocator> =
            SparseLineGrid::new(100, 0, 1.0);

        let elem = TestLineElement {
            start: Point::new(0, 0),
            end: Point::new(300, 300),
            value: 5,
        };

        grid.insert(elem);

        // The line should be findable from multiple points along it
        let queries = vec![
            Point::new(50, 50),
            Point::new(150, 150),
            Point::new(250, 250),
        ];

        for query in queries {
            let nearby = grid.get_nearby(query, 50);
            assert!(!nearby.is_empty(), "Should find line at {:?}", query);
        }
    }

    #[test]
    fn test_sparse_line_grid_horizontal_line() {
        /// Test horizontal line insertion
        let mut grid: SparseLineGrid<TestLineElement, TestLineLocator> =
            SparseLineGrid::new(100, 0, 1.0);

        let elem = TestLineElement {
            start: Point::new(0, 150),
            end: Point::new(400, 150),
            value: 7,
        };

        grid.insert(elem);

        // Should find the line along its length
        let nearby = grid.get_nearby(Point::new(200, 150), 50);
        assert!(!nearby.is_empty());
        assert_eq!(nearby[0].value, 7);
    }

    #[test]
    fn test_sparse_line_grid_vertical_line() {
        /// Test vertical line insertion
        let mut grid: SparseLineGrid<TestLineElement, TestLineLocator> =
            SparseLineGrid::new(100, 0, 1.0);

        let elem = TestLineElement {
            start: Point::new(150, 0),
            end: Point::new(150, 400),
            value: 9,
        };

        grid.insert(elem);

        // Should find the line along its length
        let nearby = grid.get_nearby(Point::new(150, 200), 50);
        assert!(!nearby.is_empty());
        assert_eq!(nearby[0].value, 9);
    }

    #[test]
    fn test_sparse_line_grid_multiple_lines() {
        /// Test multiple intersecting lines
        let mut grid: SparseLineGrid<TestLineElement, TestLineLocator> =
            SparseLineGrid::new(100, 0, 1.0);

        // Horizontal line
        grid.insert(TestLineElement {
            start: Point::new(0, 200),
            end: Point::new(400, 200),
            value: 1,
        });

        // Vertical line
        grid.insert(TestLineElement {
            start: Point::new(200, 0),
            end: Point::new(200, 400),
            value: 2,
        });

        // At the intersection, both lines should be findable
        let nearby = grid.get_nearby(Point::new(200, 200), 50);
        assert!(nearby.len() >= 2);

        let values: Vec<i32> = nearby.iter().map(|e| e.value).collect();
        assert!(values.contains(&1));
        assert!(values.contains(&2));
    }
}
