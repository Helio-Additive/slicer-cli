//! Sparse point grid which can locate spatially nearby elements efficiently.
//!
//! C++ Reference: Arachne/utils/SparsePointGrid.hpp (header-only template class)
//!
//! Copyright (c) 2016 Scott Lenser
//! Copyright (c) 2020 Ultimaker B.V.
//! CuraEngine is released under the terms of the AGPLv3 or higher.

use super::sparse_grid::SparseGrid;
use crate::geometry::{shorter_then, Point};

/// Sparse grid which can locate spatially nearby elements efficiently.
///
/// \tparam ElemT The element type to store.
/// \tparam Locator The functor to get the location from ElemT.  Locator
///    must have: Point operator()(const ElemT &elem) const
///    which returns the location associated with val.
///
/// C++ Reference: Arachne/utils/SparsePointGrid.hpp:16-58
/// C++: template<class ElemT, class Locator> class SparsePointGrid : public SparseGrid<ElemT>
/// C++: {
/// C++: public:
/// C++:     using Elem = ElemT;                                                              // :26
/// C++:     SparsePointGrid(coord_t cell_size, size_t elem_reserve = 0U, float max_load_factor = 1.0f); // :35
/// C++:     void insert(const Elem &elem);                                                   // :41
/// C++:     const ElemT *getAnyNearby(const Point &query_pt, coord_t radius);                // :51
/// C++: protected:
/// C++:     using GridPoint = typename SparseGrid<ElemT>::GridPoint;                         // :54
/// C++:     Locator m_locator;                                                               // :57
/// C++: };
#[derive(Debug, Clone)]
pub struct SparsePointGrid<Elem, Locator> {
    /// Base sparse grid.
    /// C++ Reference: Arachne/utils/SparsePointGrid.hpp:23 (`: public SparseGrid<ElemT>`)
    grid: SparseGrid<Elem>,

    /// Accessor for getting locations from elements.
    /// C++ Reference: Arachne/utils/SparsePointGrid.hpp:57
    /// C++: Locator m_locator;
    locator: Locator,
}

impl<Elem, Locator> SparsePointGrid<Elem, Locator>
where
    Elem: Clone,
    Locator: LocatorTrait<Elem>,
{
    /// Constructs a sparse grid with the specified cell size.
    ///
    /// \param[in] cell_size The size to use for a cell (square) in the grid.
    ///    Typical values would be around 0.5-2x of expected query radius.
    /// \param[in] elem_reserve Number of elements to research space for.
    /// \param[in] max_load_factor Maximum average load factor before rehashing.
    ///
    /// C++ Reference: Arachne/utils/SparsePointGrid.hpp:60-61
    /// C++: template<class ElemT, class Locator>
    /// C++: SparsePointGrid<ElemT, Locator>::SparsePointGrid(coord_t cell_size, size_t elem_reserve, float max_load_factor)
    /// C++:     : SparseGrid<ElemT>(cell_size, elem_reserve, max_load_factor) {}
    pub fn new(cell_size: i64, elem_reserve: usize, max_load_factor: f32) -> Self {
        Self {
            grid: SparseGrid::new(cell_size, elem_reserve, max_load_factor),
            // C++ default-constructs the `Locator m_locator;` member (SparsePointGrid.hpp:57).
            locator: Locator::default(),
        }
    }

    /// Inserts elem into the sparse grid.
    ///
    /// \param[in] elem The element to be inserted.
    ///
    /// C++ Reference: Arachne/utils/SparsePointGrid.hpp:63-70
    /// C++: template<class ElemT, class Locator>
    /// C++: void SparsePointGrid<ElemT, Locator>::insert(const Elem &elem)
    /// C++: {
    /// C++:     Point     loc      = m_locator(elem);
    /// C++:     GridPoint grid_loc = SparseGrid<ElemT>::toGridPoint(loc.template cast<int64_t>());
    /// C++:
    /// C++:     SparseGrid<ElemT>::m_grid.emplace(grid_loc, elem);
    /// C++: }
    pub fn insert(&mut self, elem: Elem) {
        // SparsePointGrid.hpp:66 Point loc = m_locator(elem);
        let loc = self.locator.locate(&elem);
        // SparsePointGrid.hpp:67 GridPoint grid_loc = SparseGrid<ElemT>::toGridPoint(loc.template cast<int64_t>());
        let grid_loc = self.grid.to_grid_point(loc);
        // SparsePointGrid.hpp:69 SparseGrid<ElemT>::m_grid.emplace(grid_loc, elem);
        self.grid.insert_at_grid_point(grid_loc, elem);
    }

    /// Get just any element that's within a certain radius of a point.
    ///
    /// Rather than giving a vector of nearby elements, this function just gives
    /// a single element, any element, in no particular order.
    /// \param query_pt The point to query for an object nearby.
    /// \param radius The radius of what is considered "nearby".
    ///
    /// C++ Reference: Arachne/utils/SparsePointGrid.hpp:72-86
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
    ///
    /// C++ returns a `const ElemT *` into the grid storage; Rust returns a clone of
    /// the found element (`Option<Elem>`) as the borrow cannot escape the closure.
    pub fn get_any_nearby(&self, query_pt: Point, radius: i64) -> Option<Elem> {
        // SparsePointGrid.hpp:75 const ElemT *ret = nullptr;
        let mut ret = None;
        // SparsePointGrid.hpp:76-82 process_func lambda
        let process_func = |maybe_nearby: &Elem| {
            // SparsePointGrid.hpp:77 if (shorter_then(m_locator(maybe_nearby) - query_pt, radius))
            if shorter_then(&(self.locator.locate(maybe_nearby) - query_pt), radius) {
                // SparsePointGrid.hpp:78 ret = &maybe_nearby;
                ret = Some(maybe_nearby.clone());
                // SparsePointGrid.hpp:79 return false;
                return false;
            }
            // SparsePointGrid.hpp:81 return true;
            true
        };
        // SparsePointGrid.hpp:83 SparseGrid<ElemT>::processNearby(query_pt, radius, process_func);
        self.grid.process_nearby(query_pt, radius, process_func);

        // SparsePointGrid.hpp:85 return ret;
        ret
    }

    // --- Publicly inherited SparseGrid<ElemT> interface ---
    // C++ `SparsePointGrid` publicly inherits from `SparseGrid<ElemT>`
    // (SparsePointGrid.hpp:23), exposing its public methods on the derived
    // type. Rust has no inheritance, so the inherited public surface used by
    // callers (PolylineStitcher, WallToolPaths) is forwarded explicitly.

    /// Process elements from cells that might contain sought after points.
    /// C++ Reference: Arachne/utils/SparseGrid.hpp:114-118 (inherited `processNearby`)
    pub fn process_nearby<F>(&self, query_pt: Point, radius: i64, process_func: F) -> bool
    where
        F: FnMut(&Elem) -> bool,
    {
        self.grid.process_nearby(query_pt, radius, process_func)
    }

    /// Returns all data within radius of query_pt.
    /// C++ Reference: Arachne/utils/SparseGrid.hpp:120-129 (inherited `getNearby`)
    pub fn get_nearby(&self, query_pt: Point, radius: i64) -> Vec<Elem> {
        self.grid.get_nearby(query_pt, radius)
    }

    /// Get the total number of elements stored in the grid.
    /// Introspection helper over the inherited `m_grid` (SparseGrid.hpp:94).
    pub fn num_elements(&self) -> usize {
        self.grid.num_elements()
    }

    /// Clear all elements from the grid.
    /// Helper over the inherited `m_grid` (SparseGrid.hpp:94).
    pub fn clear(&mut self) {
        self.grid.clear();
    }
}

/// Trait standing in for the C++ `Locator` template parameter.
///
/// C++ Reference: Arachne/utils/SparsePointGrid.hpp:19-21
/// C++: \tparam Locator The functor to get the location from ElemT.  Locator
/// C++:    must have: Point operator()(const ElemT &elem) const
/// C++:    which returns the location associated with val.
pub trait LocatorTrait<Elem>: Default {
    /// Get the location associated with an element.
    /// C++: Point operator()(const ElemT &elem) const
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
        // Test basic SparsePointGrid creation.
        // C++ Reference: Arachne/utils/SparsePointGrid.hpp:60-61
        let grid: SparsePointGrid<TestElement, TestLocator> = SparsePointGrid::new(1000, 10, 1.0);
        assert_eq!(grid.num_elements(), 0);
    }

    #[test]
    fn test_sparse_point_grid_insert() {
        // Test inserting elements.
        // C++ Reference: Arachne/utils/SparsePointGrid.hpp:63-70
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
        // Test getting any nearby element.
        // C++ Reference: Arachne/utils/SparsePointGrid.hpp:72-86
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

        // Query near elem1.
        let query = Point::new(150, 150);
        let nearby = grid.get_any_nearby(query, 50);
        assert!(nearby.is_some());
        assert_eq!(nearby.unwrap().value, 1);

        // Query far away.
        let far_query = Point::new(1000, 1000);
        let far_nearby = grid.get_any_nearby(far_query, 50);
        assert!(far_nearby.is_none());
    }

    #[test]
    fn test_sparse_point_grid_get_any_nearby_radius_inclusive() {
        // shorter_then (Point.hpp:349-356) uses `<=`: an element exactly at
        // `radius` distance is still considered nearby.
        let mut grid: SparsePointGrid<TestElement, TestLocator> = SparsePointGrid::new(100, 0, 1.0);

        grid.insert(TestElement {
            pos: Point::new(150, 100),
            value: 7,
        });

        // Element lies exactly 50 units away from the query point.
        let nearby = grid.get_any_nearby(Point::new(100, 100), 50);
        assert!(nearby.is_some());
        assert_eq!(nearby.unwrap().value, 7);

        // One unit closer than the element: 50 > 49, no longer nearby.
        let none = grid.get_any_nearby(Point::new(100, 100), 49);
        assert!(none.is_none());
    }

    #[test]
    fn test_sparse_point_grid_get_nearby() {
        // Test getting all nearby elements (inherited SparseGrid::getNearby).
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

        // Query near first two elements.
        let query = Point::new(200, 150);
        let nearby = grid.get_nearby(query, 150);
        assert!(nearby.len() >= 2);
    }

    #[test]
    fn test_sparse_point_grid_process_nearby() {
        // Test processing nearby elements (inherited SparseGrid::processNearby).
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
        // Test clearing the grid.
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
        // Test multiple elements in same cell.
        let mut grid: SparsePointGrid<TestElement, TestLocator> = SparsePointGrid::new(100, 0, 1.0);

        // All these will be in the same grid cell.
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
