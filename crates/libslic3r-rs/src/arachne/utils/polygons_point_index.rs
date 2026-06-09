//! Index for iterating over points in polygons for Arachne
//!
//! C++ Reference:
//! - Arachne/utils/PolygonsPointIndex.hpp
//!
//! **STATUS:** ✅ COMPLETE - Full implementation with C++ parity

use crate::geometry::{Point, Polygon, Polygons};
use std::hash::{Hash, Hasher};

/// Identity function for points (used to make templated algorithms work)
///
/// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:17
/// C++: inline const Point &make_point(const Point &p) { return p; }
#[inline]
pub fn make_point(p: &Point) -> Point {
    *p
}

/// A class for iterating over the points in one of the polygons in a Polygons object
///
/// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:22-128
/// C++: template<typename Paths>
/// C++: class PathsPointIndex
/// C++: {
/// C++: public:
/// C++:     const Paths* polygons;
/// C++:     unsigned int poly_idx;
/// C++:     unsigned int point_idx;
/// C++:     // ... methods ...
/// C++: };
#[derive(Debug, Clone, Copy)]
pub struct PathsPointIndex<'a> {
    /// The polygons into which this index is indexing (pointer to const polygons)
    /// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:29
    /// C++: const Paths* polygons;
    pub polygons: Option<&'a Polygons>,

    /// The index of the polygon in polygons
    /// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:31
    /// C++: unsigned int poly_idx;
    pub poly_idx: usize,

    /// The index of the point in the polygon
    /// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:33
    /// C++: unsigned int point_idx;
    pub point_idx: usize,
}

impl<'a> PathsPointIndex<'a> {
    /// Constructs an empty point index to no polygon
    ///
    /// This is used as a placeholder for when there is a zero-construction needed.
    /// Since the `polygons` field is const you can't ever make this initialisation useful.
    ///
    /// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:35-42
    /// C++: PathsPointIndex() : polygons(nullptr), poly_idx(0), point_idx(0) {}
    pub fn new() -> Self {
        Self {
            polygons: None,
            poly_idx: 0,
            point_idx: 0,
        }
    }

    /// Constructs a new point index to a vertex of a polygon
    ///
    /// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:44-50
    /// C++: PathsPointIndex(const Paths *polygons, unsigned int poly_idx, unsigned int point_idx)
    /// C++:     : polygons(polygons), poly_idx(poly_idx), point_idx(point_idx) {}
    pub fn with_indices(polygons: &'a Polygons, poly_idx: usize, point_idx: usize) -> Self {
        Self {
            polygons: Some(polygons),
            poly_idx,
            point_idx,
        }
    }

    /// Get the point this index refers to
    ///
    /// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:57-63
    /// C++: Point p() const
    /// C++: {
    /// C++:     if (!polygons)
    /// C++:         return {0, 0};
    /// C++:
    /// C++:     return make_point((*polygons)[poly_idx][point_idx]);
    /// C++: }
    pub fn p(&self) -> Point {
        match self.polygons {
            // PolygonsPointIndex.hpp:59-60
            None => Point::new(0, 0),
            // PolygonsPointIndex.hpp:62
            Some(polys) => make_point(&polys[self.poly_idx][self.point_idx]),
        }
    }

    /// Returns whether this point is initialised
    ///
    /// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:65-68
    /// C++: bool initialized() const { return polygons; }
    pub fn initialized(&self) -> bool {
        self.polygons.is_some()
    }

    /// Get the polygon to which this index refers
    ///
    /// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:70-73
    /// C++: const Polygon &getPolygon() const { return (*polygons)[poly_idx]; }
    pub fn get_polygon(&self) -> Option<&'a Polygon> {
        self.polygons.map(|polys| &polys[self.poly_idx])
    }

    /// Move the iterator forward (and wrap around at the end)
    ///
    /// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:100-105
    /// C++: PathsPointIndex &operator++()
    /// C++: {
    /// C++:     point_idx = (point_idx + 1) % (*polygons)[poly_idx].size();
    /// C++:     return *this;
    /// C++: }
    pub fn increment(&mut self) {
        if let Some(polys) = self.polygons {
            // PolygonsPointIndex.hpp:103
            self.point_idx = (self.point_idx + 1) % polys[self.poly_idx].len();
        }
    }

    /// Move the iterator backward (and wrap around at the beginning)
    ///
    /// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:107-113
    /// C++: PathsPointIndex &operator--()
    /// C++: {
    /// C++:     if (point_idx == 0)
    /// C++:         point_idx = (*polygons)[poly_idx].size();
    /// C++:     point_idx--;
    /// C++:     return *this;
    /// C++: }
    pub fn decrement(&mut self) {
        if let Some(polys) = self.polygons {
            // PolygonsPointIndex.hpp:109-110
            if self.point_idx == 0 {
                self.point_idx = polys[self.poly_idx].len();
            }
            // PolygonsPointIndex.hpp:111
            self.point_idx -= 1;
        }
    }

    /// Move the iterator forward (and wrap around at the end)
    ///
    /// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:114-120
    /// C++: PathsPointIndex next() const
    /// C++: {
    /// C++:     PathsPointIndex ret(*this);
    /// C++:     ++ret;
    /// C++:     return ret;
    /// C++: }
    pub fn next(&self) -> Self {
        let mut ret = *self;
        ret.increment();
        ret
    }

    /// Move the iterator backward (and wrap around at the beginning)
    ///
    /// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:121-127
    /// C++: PathsPointIndex prev() const
    /// C++: {
    /// C++:     PathsPointIndex ret(*this);
    /// C++:     --ret;
    /// C++:     return ret;
    /// C++: }
    pub fn prev(&self) -> Self {
        let mut ret = *self;
        ret.decrement();
        ret
    }
}

impl<'a> Default for PathsPointIndex<'a> {
    fn default() -> Self {
        Self::new()
    }
}

impl<'a> PartialEq for PathsPointIndex<'a> {
    /// Test whether two iterators refer to the same polygon in the same polygon list
    ///
    /// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:81-88
    /// C++: bool operator==(const PathsPointIndex &other) const
    /// C++: {
    /// C++:     return polygons == other.polygons && poly_idx == other.poly_idx && point_idx == other.point_idx;
    /// C++: }
    /// C++: bool operator!=(const PathsPointIndex &other) const
    /// C++: {
    /// C++:     return !(*this == other);
    /// C++: }
    fn eq(&self, other: &Self) -> bool {
        match (self.polygons, other.polygons) {
            (Some(p1), Some(p2)) if std::ptr::eq(p1, p2) => {
                self.poly_idx == other.poly_idx && self.point_idx == other.point_idx
            }
            (None, None) => self.poly_idx == other.poly_idx && self.point_idx == other.point_idx,
            _ => false,
        }
    }
}

impl<'a> Eq for PathsPointIndex<'a> {}

impl<'a> PartialOrd for PathsPointIndex<'a> {
    /// Compare two point indices by their point coordinates
    ///
    /// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:89-92
    /// C++: bool operator<(const PathsPointIndex &other) const
    /// C++: {
    /// C++:     return this->p() < other.p();
    /// C++: }
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl<'a> Ord for PathsPointIndex<'a> {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Point::operator< (Point.hpp:250-253):
        //   l.x() < r.x() || (l.x() == r.x() && l.y() < r.y())
        // i.e. lexicographic comparison, x first then y.
        let p1 = self.p();
        let p2 = other.p();

        match p1.x().cmp(&p2.x()) {
            std::cmp::Ordering::Equal => p1.y().cmp(&p2.y()),
            other => other,
        }
    }
}

impl<'a> Hash for PathsPointIndex<'a> {
    /// Hash function for PathsPointIndex
    ///
    /// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:168-175
    /// C++: template <>
    /// C++: struct hash<Slic3r::Arachne::PolygonsPointIndex>
    /// C++: {
    /// C++:     size_t operator()(const Slic3r::Arachne::PolygonsPointIndex& lpi) const
    /// C++:     {
    /// C++:         return Slic3r::PointHash{}(lpi.p());
    /// C++:     }
    /// C++: };
    ///
    /// Slic3r::PointHash (Point.hpp:368-372):
    /// C++: struct PointHash {
    /// C++:     size_t operator()(const Vec2crd &pt) const {
    /// C++:         return coord_t((89 * 31 + int64_t(pt.x())) * 31 + pt.y());
    /// C++:     }
    /// C++: };
    fn hash<H: Hasher>(&self, state: &mut H) {
        let p = self.p();
        // coord_t == i64; reproduce the exact PointHash mixing with wrapping i64
        // arithmetic, then feed the resulting size_t to the Rust hasher.
        let h: i64 = (89i64 * 31)
            .wrapping_add(p.x())
            .wrapping_mul(31)
            .wrapping_add(p.y());
        state.write_u64(h as u64);
    }
}

/// Type alias for the common case of PathsPointIndex with Polygons
///
/// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:130
/// C++: using PolygonsPointIndex = PathsPointIndex<Polygons>;
pub type PolygonsPointIndex<'a> = PathsPointIndex<'a>;

/// Locator to extract a line segment out of a PolygonsPointIndex
///
/// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:132-145
/// C++: struct PolygonsPointIndexSegmentLocator
/// C++: {
/// C++:     std::pair<Point, Point> operator()(const PolygonsPointIndex &val) const
/// C++:     {
/// C++:         const Polygon &poly           = (*val.polygons)[val.poly_idx];
/// C++:         Point          start          = poly[val.point_idx];
/// C++:         unsigned int   next_point_idx = (val.point_idx + 1) % poly.size();
/// C++:         Point          end            = poly[next_point_idx];
/// C++:         return std::pair<Point, Point>(start, end);
/// C++:     }
/// C++: };
#[derive(Debug, Clone, Copy)]
pub struct PolygonsPointIndexSegmentLocator;

impl PolygonsPointIndexSegmentLocator {
    /// Get the line segment starting at the indexed point
    pub fn locate(&self, val: &PolygonsPointIndex) -> Option<(Point, Point)> {
        let polys = val.polygons?;
        // PolygonsPointIndex.hpp:139
        let poly = &polys[val.poly_idx];
        // PolygonsPointIndex.hpp:140
        let start = poly[val.point_idx];
        // PolygonsPointIndex.hpp:141
        let next_point_idx = (val.point_idx + 1) % poly.len();
        // PolygonsPointIndex.hpp:142
        let end = poly[next_point_idx];
        // PolygonsPointIndex.hpp:143
        Some((start, end))
    }
}

/// Locator of a PathsPointIndex
///
/// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:147-157
/// C++: template<typename Paths>
/// C++: struct PathsPointIndexLocator
/// C++: {
/// C++:     Point operator()(const PathsPointIndex<Paths>& val) const
/// C++:     {
/// C++:         return make_point(val.p());
/// C++:     }
/// C++: };
#[derive(Debug, Clone, Copy, Default)]
pub struct PathsPointIndexLocator;

impl PathsPointIndexLocator {
    /// Get the point from a PathsPointIndex
    /// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:155
    pub fn locate(&self, val: &PathsPointIndex) -> Point {
        make_point(&val.p())
    }
}

/// Implement LocatorTrait for SparsePointGrid compatibility
/// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:153-156
impl crate::arachne::utils::sparse_point_grid::LocatorTrait<PathsPointIndex<'_>>
    for PathsPointIndexLocator
{
    fn locate(&self, elem: &PathsPointIndex) -> Point {
        make_point(&elem.p())
    }
}

/// Type alias for the common case
///
/// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:159
/// C++: using PolygonsPointIndexLocator = PathsPointIndexLocator<Polygons>;
pub type PolygonsPointIndexLocator = PathsPointIndexLocator;

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
    fn test_paths_point_index_creation() {
        /// Test basic PathsPointIndex creation
        /// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:35
        let index = PathsPointIndex::new();
        assert!(!index.initialized());
        assert_eq!(index.poly_idx, 0);
        assert_eq!(index.point_idx, 0);
    }

    #[test]
    fn test_paths_point_index_with_indices() {
        /// Test PathsPointIndex with polygon reference
        /// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:45
        let polygons = create_test_polygons();
        let index = PathsPointIndex::with_indices(&polygons, 0, 0);

        assert!(index.initialized());
        assert_eq!(index.poly_idx, 0);
        assert_eq!(index.point_idx, 0);
    }

    #[test]
    fn test_paths_point_index_p() {
        /// Test getting point from index
        /// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:58
        let polygons = create_test_polygons();
        let index = PathsPointIndex::with_indices(&polygons, 0, 1);

        let p = index.p();
        assert_eq!(p, Point::new(100, 0));
    }

    #[test]
    fn test_paths_point_index_get_polygon() {
        /// Test getting polygon from index
        /// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:72
        let polygons = create_test_polygons();
        let index = PathsPointIndex::with_indices(&polygons, 0, 0);

        let poly = index.get_polygon().unwrap();
        assert_eq!(poly.points.len(), 4);
    }

    #[test]
    fn test_paths_point_index_increment() {
        /// Test incrementing point index
        /// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:99
        let polygons = create_test_polygons();
        let mut index = PathsPointIndex::with_indices(&polygons, 0, 3);

        index.increment();
        assert_eq!(index.point_idx, 0); // wraps around

        index.increment();
        assert_eq!(index.point_idx, 1);
    }

    #[test]
    fn test_paths_point_index_decrement() {
        /// Test decrementing point index
        /// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:108
        let polygons = create_test_polygons();
        let mut index = PathsPointIndex::with_indices(&polygons, 0, 0);

        index.decrement();
        assert_eq!(index.point_idx, 3); // wraps around to end

        index.decrement();
        assert_eq!(index.point_idx, 2);
    }

    #[test]
    fn test_paths_point_index_next() {
        /// Test next() method
        /// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:115
        let polygons = create_test_polygons();
        let index = PathsPointIndex::with_indices(&polygons, 0, 2);

        let next = index.next();
        assert_eq!(next.point_idx, 3);
        assert_eq!(index.point_idx, 2); // original unchanged
    }

    #[test]
    fn test_paths_point_index_prev() {
        /// Test prev() method
        /// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:122
        let polygons = create_test_polygons();
        let index = PathsPointIndex::with_indices(&polygons, 0, 2);

        let prev = index.prev();
        assert_eq!(prev.point_idx, 1);
        assert_eq!(index.point_idx, 2); // original unchanged
    }

    #[test]
    fn test_paths_point_index_equality() {
        /// Test equality comparison
        /// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:84
        let polygons = create_test_polygons();
        let index1 = PathsPointIndex::with_indices(&polygons, 0, 1);
        let index2 = PathsPointIndex::with_indices(&polygons, 0, 1);
        let index3 = PathsPointIndex::with_indices(&polygons, 0, 2);

        assert_eq!(index1, index2);
        assert_ne!(index1, index3);
    }

    #[test]
    fn test_paths_point_index_ordering() {
        /// Test ordering comparison
        /// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:88
        let polygons = create_test_polygons();
        let index1 = PathsPointIndex::with_indices(&polygons, 0, 0); // (0, 0)
        let index2 = PathsPointIndex::with_indices(&polygons, 0, 1); // (100, 0)

        assert!(index1 < index2);
    }

    #[test]
    fn test_segment_locator() {
        /// Test PolygonsPointIndexSegmentLocator
        /// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:136
        let polygons = create_test_polygons();
        let index = PathsPointIndex::with_indices(&polygons, 0, 0);
        let locator = PolygonsPointIndexSegmentLocator;

        let (start, end) = locator.locate(&index).unwrap();
        assert_eq!(start, Point::new(0, 0));
        assert_eq!(end, Point::new(100, 0));
    }

    #[test]
    fn test_point_locator() {
        /// Test PathsPointIndexLocator
        /// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:147
        let polygons = create_test_polygons();
        let index = PathsPointIndex::with_indices(&polygons, 0, 2);
        let locator = PathsPointIndexLocator;

        let p = locator.locate(&index);
        assert_eq!(p, Point::new(100, 100));
    }

    #[test]
    fn test_make_point() {
        /// Test make_point identity function
        /// C++ Reference: Arachne/utils/PolygonsPointIndex.hpp:14
        let p = Point::new(123, 456);
        let result = make_point(&p);
        assert_eq!(result, p);
    }

    #[test]
    fn test_hash_consistency() {
        /// Test that hash is consistent for equal indices
        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};

        let polygons = create_test_polygons();
        let index1 = PathsPointIndex::with_indices(&polygons, 0, 1);
        let index2 = PathsPointIndex::with_indices(&polygons, 0, 1);

        let mut hasher1 = DefaultHasher::new();
        index1.hash(&mut hasher1);
        let hash1 = hasher1.finish();

        let mut hasher2 = DefaultHasher::new();
        index2.hash(&mut hasher2);
        let hash2 = hasher2.finish();

        assert_eq!(hash1, hash2);
    }
}
