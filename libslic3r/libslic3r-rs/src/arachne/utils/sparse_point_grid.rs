//! Sparse point grid for efficient spatial queries in Arachne
//!
//! C++ Reference:
//! - Arachne/utils/SparsePointGrid.hpp
//!
//! **STATUS:** ✅ COMPLETE - Full implementation with C++ parity

use super::sparse_grid::SparseGrid;
use crate::geometry::Point;

/// Sparse grid which can locate spatially nearby elements efficiently
///
/// C++ Reference: Arachne/utils/SparsePointGrid.hpp:14-90
/// C++: template<class ElemT, class Locator> class SparsePointGrid : public SparseGrid<ElemT>
/// C++: {
/// C++: public:
/// C++:     using Elem = ElemT;
/// C++:     SparsePointGrid(coord_t cell_size, size_t elem_reserve = 0U, float max_load_factor = 1.0f);
/// C++:     void insert(const Elem &elem);
/// C++:     const ElemT *getAnyNearby(const Point &query_pt, coord_t radius);
/// C++: protected:
/// C++:     Locator m_locator;
/// C++: };
#[derive(Debug, Clone)]
pub struct SparsePointGrid<Elem, Locator> {
    /// Base sparse grid
    /// C++ Reference: Arachne/utils/SparsePointGrid.hpp:21 (inherits from SparseGrid)
    grid: SparseGrid<Elem>,

    /// Accessor for getting locations from elements
    /// C++ Reference: Arachne/utils/SparsePointGrid.hpp:59
    /// C++: Locator m_locator;
    locator: Locator,
}

impl<Elem, Locator> SparsePointGrid<Elem, Locator>
where
    Elem: Clone,
    Locator: LocatorTrait<Elem>,
{
    /// Constructs a sparse grid with the specified cell size
    ///
    /// C++ Reference: Arachne/utils/SparsePointGrid.hpp:64-65
    /// C++: template<class ElemT, class Locator>
    /// C++: SparsePointGrid<ElemT, Locator>::SparsePointGrid(coord_t cell_size, size_t elem_reserve, float max_load_factor)
    /// C++:     : SparseGrid<ElemT>(cell_size, elem_reserve, max_load_factor) {}
    pub fn new(cell_size: i64, elem_reserve: usize, max_load_factor: f32) -> Self {
        Self {
            grid: SparseGrid::new(cell_size, elem_reserve, max_load_factor),
            locator: Locator::default(),
        }
    }

    /// Inserts elem into the sparse grid
    ///
    /// C++ Reference: Arachne/utils/SparsePointGrid.hpp:67-73
    /// C++: template<class ElemT, class Locator>
    /// C++: void SparsePointGrid<ElemT, Locator>::insert(const Elem &elem)
    /// C++: {
    /// C++:     Point     loc      = m_locator(elem);
    /// C++:     GridPoint grid_loc = SparseGrid<ElemT>::toGridPoint(loc.template cast<int64_t>());
    /// C++:
    /// C++:     SparseGrid<ElemT>::m_grid.emplace(grid_loc, elem);
    /// C++: }
    pub fn insert(&mut self, elem: Elem) {
        let loc = self.locator.locate(&elem);
        let grid_loc = self.grid.to_grid_point(loc);
        self.grid.insert_at_grid_point(grid_loc, elem);
    }

    /// Get just any element that's within a certain radius of a point
    ///
    /// Rather than giving a vector of nearby elements, this function just gives
    /// a single element, any element, in no particular order.
    ///
    /// C++ Reference: Arachne/utils/SparsePointGrid.hpp:75-87
    /// C++: template<class ElemT, class Locator>
    /// C++: const ElemT *SparsePointGrid<ElemT, Locator>::getAnyNearby(const Point &query_pt, coord_t radius)
    /// C++: {
    /// C++:     const ElemT                              *ret          = nullptr;
    /// C++:     const std::function<bool(const ElemT &)> &process_func = [&ret, query_pt, radius, this](const ElemT &maybe_nearby) {
    /// C++:         if (shorter_then(m_locator(maybe_nearby) - query_pt, radius)) {
    /// C++:             ret = &maybe_nearby;
    /// C++:             return false;
    /// C++:         }
    /// C++:         return true;
    /// C++:     };
    /// C++:     SparseGrid<ElemT>::processNearby(query_pt, radius, process_func);
    /// C++:
    /// C++:     return ret;
    /// C++: }
    pub fn get_any_nearby(&self, query_pt: Point, radius: i64) -> Option<Elem> {
        let mut ret = None;
        self.grid.process_nearby(query_pt, radius, |maybe_nearby| {
            let loc = self.locator.locate(maybe_nearby);
            let dx = loc.x() - query_pt.x();
            let dy = loc.y() - query_pt.y();
            let dist_sq = (dx as i64 * dx as i64 + dy as i64 * dy as i64) as i64;
            if dist_sq < radius * radius {
                ret = Some(maybe_nearby.clone());
                return false; // Stop searching
            }
            true // Keep searching
        });
        ret
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

    /// Get the number of elements in the grid
    pub fn num_elements(&self) -> usize {
        self.grid.num_elements()
    }

    /// Clear all elements
    pub fn clear(&mut self) {
        self.grid.clear();
    }
}

/// Trait for locator pattern - extracts location from element
///
/// C++ Reference: Arachne/utils/SparsePointGrid.hpp:59
/// C++: Locator m_locator;
/// C++: // Where Locator has: Point operator()(const ElemT &elem) const
pub trait LocatorTrait<Elem>: Default {
    /// Get the location associated with an element
    fn locate(&self, elem: &Elem) -> Point;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Point;

    #[derive(Debug, Clone, PartialEq)]
    struct TestElement {
        pos: Point,
        value: i32,
    }

    #[derive(Default)]
    struct TestLocator;

    impl LocatorTrait<TestElement> for TestLocator {
        fn locate(&self, elem: &TestElement) -> Point {
            elem.pos
        }
    }

    #[test]
    fn test_sparse_point_grid_creation() {
        /// Test basic SparsePointGrid creation
        /// C++ Reference: Arachne/utils/SparsePointGrid.hpp:65
        let grid: SparsePointGrid<TestElement, TestLocator> = SparsePointGrid::new(1000, 10, 1.0);
        assert_eq!(grid.num_elements(), 0);
    }

    #[test]
    fn test_sparse_point_grid_insert() {
        /// Test inserting elements
        /// C++ Reference: Arachne/utils/SparsePointGrid.hpp:73
        let mut grid: SparsePointGrid<TestElement, TestLocator> = SparsePointGrid::new(100, 0, 1.0);

        let elem = TestElement {
            pos: Point::new(150, 150),
            value: 42,
        };

        grid.insert(elem);
        assert_eq!(grid.num_elements(), 1);
    }

    #[test]
    fn test_sparse_point_grid_get_any_nearby() {
        /// Test getting any nearby element
        /// C++ Reference: Arachne/utils/SparsePointGrid.hpp:87
        let mut grid: SparsePointGrid<TestElement, TestLocator> = SparsePointGrid::new(100, 0, 1.0);

        let elem1 = TestElement {
            pos: Point::new(150, 150),
            value: 1,
        };
        let elem2 = TestElement {
            pos: Point::new(250, 150),
            value: 2,
        };
        let elem3 = TestElement {
            pos: Point::new(500, 500),
            value: 3,
        };

        grid.insert(elem1.clone());
        grid.insert(elem2.clone());
        grid.insert(elem3);

        // Query near elem1
        let query = Point::new(150, 150);
        let nearby = grid.get_any_nearby(query, 50);
        assert!(nearby.is_some());
        assert_eq!(nearby.unwrap().value, 1);

        // Query far away
        let far_query = Point::new(1000, 1000);
        let far_nearby = grid.get_any_nearby(far_query, 50);
        assert!(far_nearby.is_none());
    }

    #[test]
    fn test_sparse_point_grid_get_nearby() {
        /// Test getting all nearby elements
        let mut grid: SparsePointGrid<TestElement, TestLocator> = SparsePointGrid::new(100, 0, 1.0);

        let elem1 = TestElement {
            pos: Point::new(150, 150),
            value: 1,
        };
        let elem2 = TestElement {
            pos: Point::new(250, 150),
            value: 2,
        };
        let elem3 = TestElement {
            pos: Point::new(500, 500),
            value: 3,
        };

        grid.insert(elem1);
        grid.insert(elem2);
        grid.insert(elem3);

        // Query near first two elements
        let query = Point::new(200, 150);
        let nearby = grid.get_nearby(query, 150);
        assert!(nearby.len() >= 2);
    }

    #[test]
    fn test_sparse_point_grid_process_nearby() {
        /// Test processing nearby elements
        let mut grid: SparsePointGrid<TestElement, TestLocator> = SparsePointGrid::new(100, 0, 1.0);

        let elem1 = TestElement {
            pos: Point::new(150, 150),
            value: 10,
        };
        let elem2 = TestElement {
            pos: Point::new(250, 150),
            value: 20,
        };

        grid.insert(elem1);
        grid.insert(elem2);

        let mut sum = 0;
        grid.process_nearby(Point::new(200, 150), 150, |elem| {
            sum += elem.value;
            true
        });

        assert!(sum > 0);
    }

    #[test]
    fn test_sparse_point_grid_clear() {
        /// Test clearing the grid
        let mut grid: SparsePointGrid<TestElement, TestLocator> = SparsePointGrid::new(100, 0, 1.0);

        let elem = TestElement {
            pos: Point::new(150, 150),
            value: 42,
        };

        grid.insert(elem);
        assert_eq!(grid.num_elements(), 1);

        grid.clear();
        assert_eq!(grid.num_elements(), 0);
    }

    #[test]
    fn test_sparse_point_grid_multiple_elements() {
        /// Test multiple elements in same cell
        let mut grid: SparsePointGrid<TestElement, TestLocator> = SparsePointGrid::new(100, 0, 1.0);

        // All these will be in the same grid cell
        let elem1 = TestElement {
            pos: Point::new(150, 150),
            value: 1,
        };
        let elem2 = TestElement {
            pos: Point::new(160, 160),
            value: 2,
        };
        let elem3 = TestElement {
            pos: Point::new(170, 170),
            value: 3,
        };

        grid.insert(elem1);
        grid.insert(elem2);
        grid.insert(elem3);

        assert_eq!(grid.num_elements(), 3);

        let nearby = grid.get_nearby(Point::new(150, 150), 50);
        assert!(nearby.len() >= 1);
    }
}
