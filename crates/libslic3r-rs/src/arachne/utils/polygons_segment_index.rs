//! Segment index for iterating over line segments in polygons for Arachne
//!
//! C++ Reference:
//! - Arachne/utils/PolygonsSegmentIndex.hpp
//!
//! **STATUS:** ✅ COMPLETE - Full implementation with C++ parity

use super::polygons_point_index::{PathsPointIndex, PolygonsPointIndex};
use crate::geometry::Point;

/// A class for iterating over the segments in one of the polygons in a Polygons object
///
/// This extends PolygonsPointIndex to provide segment-based iteration where each
/// index represents a line segment from point N to point N+1.
///
/// C++ Reference: Arachne/utils/PolygonsSegmentIndex.hpp:13-27
/// C++: class PolygonsSegmentIndex : public PolygonsPointIndex
/// C++: {
/// C++: public:
/// C++:     PolygonsSegmentIndex() : PolygonsPointIndex(){};
/// C++:     PolygonsSegmentIndex(const Polygons *polygons, unsigned int poly_idx, unsigned int point_idx)
/// C++:         : PolygonsPointIndex(polygons, poly_idx, point_idx){};
/// C++:
/// C++:     Point from() const { return PolygonsPointIndex::p(); }
/// C++:
/// C++:     Point to() const { return PolygonsSegmentIndex::next().p(); }
/// C++: };
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct PolygonsSegmentIndex<'a> {
    /// The underlying point index
    /// C++ Reference: Arachne/utils/PolygonsSegmentIndex.hpp:15 (inherits from PolygonsPointIndex)
    point_index: PolygonsPointIndex<'a>,
}

impl<'a> PolygonsSegmentIndex<'a> {
    /// Constructs an empty segment index to no polygon
    ///
    /// C++ Reference: Arachne/utils/PolygonsSegmentIndex.hpp:20
    /// C++: PolygonsSegmentIndex() : PolygonsPointIndex(){};
    pub fn new() -> Self {
        Self {
            point_index: PolygonsPointIndex::new(),
        }
    }

    /// Constructs a new segment index to a segment in a polygon
    ///
    /// The segment goes from point_idx to point_idx+1 (wrapping around)
    ///
    /// C++ Reference: Arachne/utils/PolygonsSegmentIndex.hpp:21
    /// C++: PolygonsSegmentIndex(const Polygons *polygons, unsigned int poly_idx, unsigned int point_idx)
    /// C++:     : PolygonsPointIndex(polygons, poly_idx, point_idx){};
    pub fn with_indices(
        polygons: &'a crate::geometry::Polygons,
        poly_idx: usize,
        point_idx: usize,
    ) -> Self {
        Self {
            point_index: PathsPointIndex::with_indices(polygons, poly_idx, point_idx),
        }
    }

    /// Get the start point of this segment
    ///
    /// C++ Reference: Arachne/utils/PolygonsSegmentIndex.hpp:23
    /// C++: Point from() const { return PolygonsPointIndex::p(); }
    pub fn from(&self) -> Point {
        self.point_index.p()
    }

    /// Get the end point of this segment
    ///
    /// C++ Reference: Arachne/utils/PolygonsSegmentIndex.hpp:25
    /// C++: Point to() const { return PolygonsSegmentIndex::next().p(); }
    pub fn to(&self) -> Point {
        self.point_index.next().p()
    }

    /// Get the underlying point index
    pub fn point_index(&self) -> &PolygonsPointIndex<'a> {
        &self.point_index
    }

    /// Get the underlying point index mutably
    pub fn point_index_mut(&mut self) -> &mut PolygonsPointIndex<'a> {
        &mut self.point_index
    }

    /// Check if this segment index is initialized
    pub fn initialized(&self) -> bool {
        self.point_index.initialized()
    }

    /// Get the polygon index
    pub fn poly_idx(&self) -> usize {
        self.point_index.poly_idx
    }

    /// Get the point index (start of segment)
    pub fn point_idx(&self) -> usize {
        self.point_index.point_idx
    }

    /// Move to the next segment
    pub fn next(&self) -> Self {
        Self {
            point_index: self.point_index.next(),
        }
    }

    /// Move to the previous segment
    pub fn prev(&self) -> Self {
        Self {
            point_index: self.point_index.prev(),
        }
    }

    /// Increment to the next segment (modifies self)
    pub fn increment(&mut self) {
        self.point_index.increment();
    }

    /// Decrement to the previous segment (modifies self)
    pub fn decrement(&mut self) {
        self.point_index.decrement();
    }

    /// boost::polygon segment accessor: returns the `to()` endpoint when the
    /// direction is HIGH and the `from()` endpoint when it is LOW.
    ///
    /// This is the Rust equivalent of the `boost::polygon::segment_traits`
    /// specialisation for `PolygonsSegmentIndex`. Rust has no boost.polygon
    /// trait-specialisation system, so the `get(segment, dir)` free function is
    /// expressed as an inherent method.
    ///
    /// C++ Reference: Arachne/utils/PolygonsSegmentIndex.hpp:30-48
    /// C++: namespace boost::polygon {
    /// C++: template<> struct geometry_concept<Slic3r::Arachne::PolygonsSegmentIndex>
    /// C++: {
    /// C++:     typedef segment_concept type;
    /// C++: };
    /// C++:
    /// C++: template<> struct segment_traits<Slic3r::Arachne::PolygonsSegmentIndex>
    /// C++: {
    /// C++:     typedef coord_t       coordinate_type;
    /// C++:     typedef Slic3r::Point point_type;
    /// C++:
    /// C++:     static inline point_type get(const Slic3r::Arachne::PolygonsSegmentIndex &CSegment, direction_1d dir)
    /// C++:     {
    /// C++:         return dir.to_int() ? CSegment.to() : CSegment.from();
    /// C++:     }
    /// C++: };
    /// C++: } // namespace boost::polygon
    pub fn segment_get(&self, dir: Direction1d) -> Point {
        // C++: return dir.to_int() ? CSegment.to() : CSegment.from();
        if dir.to_int() != 0 {
            self.to()
        } else {
            self.from()
        }
    }
}

/// boost::polygon `direction_1d` for a segment endpoint: LOW selects the
/// `from()` endpoint, HIGH selects the `to()` endpoint.
///
/// C++ Reference: Arachne/utils/PolygonsSegmentIndex.hpp:42 (boost::polygon::direction_1d)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Direction1d {
    /// boost::polygon::LOW (to_int() == 0)
    Low,
    /// boost::polygon::HIGH (to_int() == 1)
    High,
}

impl Direction1d {
    /// C++ Reference: boost::polygon::direction_1d::to_int()
    #[inline]
    pub fn to_int(self) -> i32 {
        match self {
            Direction1d::Low => 0,
            Direction1d::High => 1,
        }
    }
}

impl<'a> Default for PolygonsSegmentIndex<'a> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Point, Polygon, Polygons};

    fn create_test_polygons() -> Polygons {
        /// Create a simple test polygon structure
        let mut polygons = Polygons::new();

        // First polygon: square
        let mut poly1 = Polygon::new();
        poly1.points.push(Point::new(0, 0));
        poly1.points.push(Point::new(100, 0));
        poly1.points.push(Point::new(100, 100));
        poly1.points.push(Point::new(0, 100));
        polygons.push(poly1);

        // Second polygon: triangle
        let mut poly2 = Polygon::new();
        poly2.points.push(Point::new(200, 0));
        poly2.points.push(Point::new(300, 0));
        poly2.points.push(Point::new(250, 100));
        polygons.push(poly2);

        polygons
    }

    #[test]
    fn test_polygons_segment_index_creation() {
        /// Test basic PolygonsSegmentIndex creation
        /// C++ Reference: Arachne/utils/PolygonsSegmentIndex.hpp:17
        let index = PolygonsSegmentIndex::new();
        assert!(!index.initialized());
    }

    #[test]
    fn test_polygons_segment_index_with_indices() {
        /// Test PolygonsSegmentIndex with polygon reference
        /// C++ Reference: Arachne/utils/PolygonsSegmentIndex.hpp:19
        let polygons = create_test_polygons();
        let index = PolygonsSegmentIndex::with_indices(&polygons, 0, 0);

        assert!(index.initialized());
        assert_eq!(index.poly_idx(), 0);
        assert_eq!(index.point_idx(), 0);
    }

    #[test]
    fn test_polygons_segment_index_from_to() {
        /// Test getting segment endpoints
        /// C++ Reference: Arachne/utils/PolygonsSegmentIndex.hpp:21-23
        let polygons = create_test_polygons();
        let index = PolygonsSegmentIndex::with_indices(&polygons, 0, 0);

        let from = index.from();
        let to = index.to();

        assert_eq!(from, Point::new(0, 0));
        assert_eq!(to, Point::new(100, 0));
    }

    #[test]
    fn test_polygons_segment_index_segment_get() {
        // Test boost::polygon segment_traits::get equivalent
        // C++ Reference: Arachne/utils/PolygonsSegmentIndex.hpp:42-45
        let polygons = create_test_polygons();
        let index = PolygonsSegmentIndex::with_indices(&polygons, 0, 0);

        // dir.to_int() == 0 (LOW) -> from()
        assert_eq!(index.segment_get(Direction1d::Low), Point::new(0, 0));
        // dir.to_int() != 0 (HIGH) -> to()
        assert_eq!(index.segment_get(Direction1d::High), Point::new(100, 0));
    }

    #[test]
    fn test_polygons_segment_index_wrapping() {
        /// Test segment wrapping at polygon end
        /// C++ Reference: Arachne/utils/PolygonsSegmentIndex.hpp:23
        let polygons = create_test_polygons();
        let index = PolygonsSegmentIndex::with_indices(&polygons, 0, 3);

        let from = index.from();
        let to = index.to();

        // Last segment should wrap back to first point
        assert_eq!(from, Point::new(0, 100));
        assert_eq!(to, Point::new(0, 0));
    }

    #[test]
    fn test_polygons_segment_index_next() {
        /// Test moving to next segment
        let polygons = create_test_polygons();
        let index = PolygonsSegmentIndex::with_indices(&polygons, 0, 0);

        let next = index.next();
        assert_eq!(next.point_idx(), 1);
        assert_eq!(next.from(), Point::new(100, 0));
        assert_eq!(next.to(), Point::new(100, 100));
    }

    #[test]
    fn test_polygons_segment_index_prev() {
        /// Test moving to previous segment
        let polygons = create_test_polygons();
        let index = PolygonsSegmentIndex::with_indices(&polygons, 0, 2);

        let prev = index.prev();
        assert_eq!(prev.point_idx(), 1);
        assert_eq!(prev.from(), Point::new(100, 0));
        assert_eq!(prev.to(), Point::new(100, 100));
    }

    #[test]
    fn test_polygons_segment_index_increment() {
        /// Test incrementing segment index
        let polygons = create_test_polygons();
        let mut index = PolygonsSegmentIndex::with_indices(&polygons, 0, 2);

        index.increment();
        assert_eq!(index.point_idx(), 3);

        index.increment();
        assert_eq!(index.point_idx(), 0); // wraps around
    }

    #[test]
    fn test_polygons_segment_index_decrement() {
        /// Test decrementing segment index
        let polygons = create_test_polygons();
        let mut index = PolygonsSegmentIndex::with_indices(&polygons, 0, 0);

        index.decrement();
        assert_eq!(index.point_idx(), 3); // wraps to end

        index.decrement();
        assert_eq!(index.point_idx(), 2);
    }

    #[test]
    fn test_polygons_segment_index_triangle() {
        /// Test with triangle polygon
        let polygons = create_test_polygons();
        let index = PolygonsSegmentIndex::with_indices(&polygons, 1, 0);

        assert_eq!(index.from(), Point::new(200, 0));
        assert_eq!(index.to(), Point::new(300, 0));

        let next = index.next();
        assert_eq!(next.from(), Point::new(300, 0));
        assert_eq!(next.to(), Point::new(250, 100));
    }

    #[test]
    fn test_polygons_segment_index_equality() {
        /// Test equality comparison
        let polygons = create_test_polygons();
        let index1 = PolygonsSegmentIndex::with_indices(&polygons, 0, 1);
        let index2 = PolygonsSegmentIndex::with_indices(&polygons, 0, 1);
        let index3 = PolygonsSegmentIndex::with_indices(&polygons, 0, 2);

        assert_eq!(index1, index2);
        assert_ne!(index1, index3);
    }

    #[test]
    fn test_polygons_segment_index_all_segments() {
        /// Test iterating through all segments of a polygon
        let polygons = create_test_polygons();
        let mut index = PolygonsSegmentIndex::with_indices(&polygons, 0, 0);

        // Square has 4 segments
        let expected_segments = vec![
            (Point::new(0, 0), Point::new(100, 0)),
            (Point::new(100, 0), Point::new(100, 100)),
            (Point::new(100, 100), Point::new(0, 100)),
            (Point::new(0, 100), Point::new(0, 0)),
        ];

        for expected in expected_segments {
            assert_eq!(index.from(), expected.0);
            assert_eq!(index.to(), expected.1);
            index.increment();
        }

        // Should wrap back to start
        assert_eq!(index.point_idx(), 0);
    }
}
