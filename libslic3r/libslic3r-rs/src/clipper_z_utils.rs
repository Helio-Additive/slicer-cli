//! Z-coordinate aware utilities for original Clipper library integration
//!
//! This module provides utilities for working with Clipper paths that include
//! Z-coordinates, enabling tracking of source geometry through boolean operations.
//!
//! C++ Reference: `ClipperZUtils.hpp`
//!
//! ## Key Concepts
//!
//! - **ZPoint/ZPath/ZPaths**: Clipper types with Z-coordinate support
//! - **Z-coordinate tracking**: Each point carries a Z value identifying its source
//! - **Intersection detection**: Track which contours intersect during boolean ops
//! - **Contour preservation**: Maintain source information through transformations
//!
//! ## Architecture
//!
//! The module provides:
//! 1. Conversion functions (Slic3r types ↔ Clipper Z-types)
//! 2. Intersection visitor for detecting edge crossings
//! 3. Helper functions for batch conversions with Z-indexing

use crate::geometry::{ExPolygon, Point};
use std::cmp::Ordering;

// ============================================================================
// Type Aliases (matching C++ exactly)
// ============================================================================

/// Clipper Z-aware point type
/// ClipperZUtils.hpp:14
/// C++: using ZPoint = ClipperLib_Z::IntPoint;
pub type ZPoint = (i64, i64, i64); // (x, y, z)

/// Clipper Z-aware path type (single contour)
/// ClipperZUtils.hpp:15
/// C++: using ZPath = ClipperLib_Z::Path;
pub type ZPath = Vec<ZPoint>;

/// Clipper Z-aware paths type (multiple contours)
/// ClipperZUtils.hpp:16
/// C++: using ZPaths = ClipperLib_Z::Paths;
pub type ZPaths = Vec<ZPath>;

// ============================================================================
// Comparison and Ordering
// ============================================================================

/// Compare two Z-points lexicographically (x, then y, then z)
/// ClipperZUtils.hpp:18-21
/// C++: inline bool zpoint_lower(const ZPoint &l, const ZPoint &r)
/// C++: {
/// C++:     return l.x() < r.x() || (l.x() == r.x() && (l.y() < r.y() || (l.y() == r.y() && l.z() < r.z())));
/// C++: }
#[inline]
pub fn zpoint_lower(l: &ZPoint, r: &ZPoint) -> bool {
    l.0 < r.0 || (l.0 == r.0 && (l.1 < r.1 || (l.1 == r.1 && l.2 < r.2)))
}

/// Ordering implementation for sorting Z-points
/// ClipperZUtils.hpp:18-21 (derived)
#[inline]
pub fn zpoint_cmp(l: &ZPoint, r: &ZPoint) -> Ordering {
    match l.0.cmp(&r.0) {
        Ordering::Equal => match l.1.cmp(&r.1) {
            Ordering::Equal => l.2.cmp(&r.2),
            other => other,
        },
        other => other,
    }
}

// ============================================================================
// Conversion: Slic3r → Clipper Z-types
// ============================================================================

/// Convert a single path to zpath with a given Z coordinate
/// ClipperZUtils.hpp:24-36
/// C++: template<bool Open = false>
/// C++: inline ZPath to_zpath(const Points &path, coord_t z)
/// C++: {
/// C++:     ZPath out;
/// C++:     if (! path.empty()) {
/// C++:         out.reserve((path.size() + Open) ? 1 : 0);
/// C++:         for (const Point &p : path)
/// C++:             out.emplace_back(p.x(), p.y(), z);
/// C++:         if (Open)
/// C++:             out.emplace_back(out.front());
/// C++:     }
/// C++:     return out;
/// C++: }
pub fn to_zpath(path: &[Point], z: i64, open: bool) -> ZPath {
    if path.is_empty() {
        return Vec::new();
    }

    let mut out = Vec::with_capacity(path.len() + if open { 1 } else { 0 });
    for p in path {
        out.push((p.x(), p.y(), z));
    }
    if open && !out.is_empty() {
        out.push(out[0]);
    }
    out
}

/// Convert multiple paths to zpaths with a given Z coordinate
/// ClipperZUtils.hpp:39-48
/// C++: template<bool Open = false>
/// C++: inline ZPaths to_zpaths(const VecOfPoints &paths, coord_t z)
/// C++: {
/// C++:     ZPaths out;
/// C++:     out.reserve(paths.size());
/// C++:     for (const Points &path : paths)
/// C++:         out.emplace_back(to_zpath<Open>(path, z));
/// C++:     return out;
/// C++: }
pub fn to_zpaths(paths: &[Vec<Point>], z: i64, open: bool) -> ZPaths {
    let mut out = Vec::with_capacity(paths.len());
    for path in paths {
        out.push(to_zpath(path, z, open));
    }
    out
}

/// Convert multiple expolygons into zpaths with Z specified by index offset by base_idx
/// ClipperZUtils.hpp:52-66
/// C++: template<bool Open = false>
/// C++: inline ZPaths expolygons_to_zpaths(const ExPolygons &src, coord_t &base_idx)
/// C++: {
/// C++:     ZPaths out;
/// C++:     out.reserve(std::accumulate(src.begin(), src.end(), size_t(0),
/// C++:         [](const size_t acc, const ExPolygon &expoly) { return acc + expoly.num_contours(); }));
/// C++:     for (const ExPolygon &expoly : src) {
/// C++:         out.emplace_back(to_zpath<Open>(expoly.contour.points, base_idx));
/// C++:         for (const Polygon &hole : expoly.holes)
/// C++:             out.emplace_back(to_zpath<Open>(hole.points, base_idx));
/// C++:         ++ base_idx;
/// C++:     }
/// C++:     return out;
/// C++: }
pub fn expolygons_to_zpaths(src: &[ExPolygon], base_idx: &mut i64, open: bool) -> ZPaths {
    // Helper to count total contours
    // ClipperZUtils.hpp:54-55
    fn count_contours(expolygons: &[ExPolygon]) -> usize {
        expolygons.iter().map(|e| e.num_contours()).sum()
    }

    let mut out = Vec::with_capacity(count_contours(src));
    for expoly in src {
        // Add outer contour
        // ClipperZUtils.hpp:61
        // C++: out.emplace_back(to_zpath<Open>(expoly.contour.points, base_idx));
        out.push(to_zpath(&expoly.contour.points, *base_idx, open));

        // Add holes
        // ClipperZUtils.hpp:62-63
        // C++: for (const Polygon &hole : expoly.holes)
        // C++:     out.emplace_back(to_zpath<Open>(hole.points, base_idx));
        for hole in &expoly.holes {
            out.push(to_zpath(&hole.points, *base_idx, open));
        }

        // Increment base index for next expolygon
        // ClipperZUtils.hpp:64
        // C++: ++ base_idx;
        *base_idx += 1;
    }
    out
}

/// Convert multiple expolygons into zpaths with a given Z coordinate
/// ClipperZUtils.hpp:69-82
/// C++: template<bool Open> inline ZPaths expolygons_to_zpaths_with_same_z(const ExPolygons &src, const coord_t z)
/// C++: {
/// C++:     ZPaths out;
/// C++:     out.reserve(std::accumulate(src.begin(), src.end(), size_t(0), [](const size_t acc, const ExPolygon &expoly) {
/// C++:         return acc + expoly.num_contours();
/// C++:     }));
/// C++:     for (const ExPolygon &expoly : src) {
/// C++:         out.emplace_back(to_zpath<Open>(expoly.contour.points, z));
/// C++:         for (const Polygon &hole : expoly.holes) {
/// C++:             out.emplace_back(to_zpath<Open>(hole.points, z));
/// C++:         }
/// C++:     }
/// C++:
/// C++:     return out;
/// C++: }
pub fn expolygons_to_zpaths_with_same_z(src: &[ExPolygon], z: i64, open: bool) -> ZPaths {
    // Helper to count total contours
    // ClipperZUtils.hpp:71-73
    fn count_contours(expolygons: &[ExPolygon]) -> usize {
        expolygons.iter().map(|e| e.num_contours()).sum()
    }

    let mut out = Vec::with_capacity(count_contours(src));
    for expoly in src {
        // Add outer contour with same Z
        // ClipperZUtils.hpp:74
        // C++: out.emplace_back(to_zpath<Open>(expoly.contour.points, z));
        out.push(to_zpath(&expoly.contour.points, z, open));

        // Add holes with same Z
        // ClipperZUtils.hpp:75-77
        // C++: for (const Polygon &hole : expoly.holes) {
        // C++:     out.emplace_back(to_zpath<Open>(hole.points, z));
        // C++: }
        for hole in &expoly.holes {
            out.push(to_zpath(&hole.points, z, open));
        }
    }
    out
}

// ============================================================================
// Conversion: Clipper Z-types → Slic3r
// ============================================================================

/// Convert a zpath back to 2D Points
/// ClipperZUtils.hpp:86-97
/// C++: template<bool Open = false>
/// C++: inline Points from_zpath(const ZPoints &path)
/// C++: {
/// C++:     Points out;
/// C++:     if (! path.empty()) {
/// C++:         out.reserve((path.size() + Open) ? 1 : 0);
/// C++:         for (const ZPoint &p : path)
/// C++:             out.emplace_back(p.x(), p.y());
/// C++:         if (Open)
/// C++:             out.emplace_back(out.front());
/// C++:     }
/// C++:     return out;
/// C++: }
pub fn from_zpath(path: &ZPath, open: bool) -> Vec<Point> {
    if path.is_empty() {
        return Vec::new();
    }

    let mut out = Vec::with_capacity(path.len() + if open { 1 } else { 0 });
    for &(x, y, _z) in path {
        out.push(Point::new(x, y));
    }
    if open && !out.is_empty() {
        out.push(out[0]);
    }
    out
}

/// Convert multiple zpaths back to 2D paths (appending to existing vector)
/// ClipperZUtils.hpp:100-106
/// C++: template<bool Open = false>
/// C++: inline void from_zpaths(const ZPaths &paths, VecOfPoints &out)
/// C++: {
/// C++:     out.reserve(out.size() + paths.size());
/// C++:     for (const ZPoints &path : paths)
/// C++:         out.emplace_back(from_zpath<Open>(path));
/// C++: }
pub fn from_zpaths_append(paths: &ZPaths, out: &mut Vec<Vec<Point>>, open: bool) {
    out.reserve(out.len() + paths.len());
    for path in paths {
        out.push(from_zpath(path, open));
    }
}

/// Convert multiple zpaths back to 2D paths (new vector)
/// ClipperZUtils.hpp:107-112
/// C++: template<bool Open = false>
/// C++: inline VecOfPoints from_zpaths(const ZPaths &paths)
/// C++: {
/// C++:     VecOfPoints out;
/// C++:     from_zpaths(paths, out);
/// C++:     return out;
/// C++: }
pub fn from_zpaths(paths: &ZPaths, open: bool) -> Vec<Vec<Point>> {
    let mut out = Vec::new();
    from_zpaths_append(paths, &mut out, open);
    out
}

// ============================================================================
// Intersection Visitor
// ============================================================================

/// Intersection pair: (source_z_1, source_z_2)
/// ClipperZUtils.hpp:116
/// C++: using Intersection = std::pair<coord_t, coord_t>;
pub type Intersection = (i64, i64);

/// Vector of intersection pairs
/// ClipperZUtils.hpp:117
/// C++: using Intersections = std::vector<Intersection>;
pub type Intersections = Vec<Intersection>;

/// Intersection visitor for Clipper (ZFillCallback)
///
/// Tracks intersections between edges from different source contours during
/// Clipper boolean operations. When two edges intersect, their source Z-values
/// are paired and stored.
///
/// ClipperZUtils.hpp:114-143
/// C++: class ClipperZIntersectionVisitor {
/// C++: public:
/// C++:     using Intersection  = std::pair<coord_t, coord_t>;
/// C++:     using Intersections = std::vector<Intersection>;
/// C++:     ClipperZIntersectionVisitor(Intersections &intersections) : m_intersections(intersections) {}
/// C++:     void reset() { m_intersections.clear(); }
/// C++:     void operator()(const ZPoint &e1bot, const ZPoint &e1top, const ZPoint &e2bot, const ZPoint &e2top, ZPoint &pt) { ... }
/// C++:     ClipperLib_Z::ZFillCallback clipper_callback() { ... }
/// C++:     const std::vector<std::pair<coord_t, coord_t>>& intersections() const { return m_intersections; }
/// C++:
/// C++: private:
/// C++:     std::vector<std::pair<coord_t, coord_t>> &m_intersections;
/// C++: };
pub struct ClipperZIntersectionVisitor {
    // Storage for detected intersections
    // ClipperZUtils.hpp:142
    // C++: std::vector<std::pair<coord_t, coord_t>> &m_intersections;
    intersections: Intersections,
}

impl ClipperZIntersectionVisitor {
    // Create a new intersection visitor
    // ClipperZUtils.hpp:118
    // C++: ClipperZIntersectionVisitor(Intersections &intersections) : m_intersections(intersections) {}
    pub fn new() -> Self {
        Self {
            intersections: Vec::new(),
        }
    }

    // Create with pre-allocated capacity
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            intersections: Vec::with_capacity(capacity),
        }
    }

    // Reset intersection storage
    // ClipperZUtils.hpp:119
    // C++: void reset() { m_intersections.clear(); }
    pub fn reset(&mut self) {
        self.intersections.clear();
    }

    // Process an intersection between two edges
    //
    // When edges from two different source contours intersect, their Z-values
    // (which encode source contour indices) are paired and stored.
    //
    // ClipperZUtils.hpp:120-135
    // C++: void operator()(const ZPoint &e1bot, const ZPoint &e1top, const ZPoint &e2bot, const ZPoint &e2top, ZPoint &pt) {
    // C++:     coord_t srcs[4]{ e1bot.z(), e1top.z(), e2bot.z(), e2top.z() };
    // C++:     coord_t *begin = srcs;
    // C++:     coord_t *end = srcs + 4;
    // C++:     //FIXME bubble sort manually?
    // C++:     std::sort(begin, end);
    // C++:     end = std::unique(begin, end);
    // C++:     if (begin + 1 == end) {
    // C++:         // Self intersection may happen on source contour. Just copy the Z value.
    // C++:         pt.z() = *begin;
    // C++:     } else {
    // C++:         assert(begin + 2 == end);
    // C++:         if (begin + 2 <= end) {
    // C++:             // store a -1 based negative index into the "intersections" vector here.
    // C++:             m_intersections.emplace_back(srcs[0], srcs[1]);
    // C++:             pt.z() = -coord_t(m_intersections.size());
    // C++:         }
    // C++:     }
    // C++: }
    pub fn process_intersection(
        &mut self,
        e1bot: &ZPoint,
        e1top: &ZPoint,
        e2bot: &ZPoint,
        e2top: &ZPoint,
    ) -> ZPoint {
        // Collect all source Z values from edge endpoints
        // ClipperZUtils.hpp:121
        // C++: coord_t srcs[4]{ e1bot.z(), e1top.z(), e2bot.z(), e2top.z() };
        let mut srcs = [e1bot.2, e1top.2, e2bot.2, e2top.2];

        // Sort and deduplicate to find unique source contours
        // ClipperZUtils.hpp:122-126
        // C++: coord_t *begin = srcs;
        // C++: coord_t *end = srcs + 4;
        // C++: //FIXME bubble sort manually?
        // C++: std::sort(begin, end);
        // C++: end = std::unique(begin, end);
        srcs.sort_unstable();
        let mut unique_count = 1;
        for i in 1..4 {
            if srcs[i] != srcs[unique_count - 1] {
                srcs[unique_count] = srcs[i];
                unique_count += 1;
            }
        }

        // Calculate intersection point (midpoint of edge intersections)
        // Not explicitly in C++ (Clipper handles geometry internally)
        let pt_x = (e1bot.0 + e1top.0 + e2bot.0 + e2top.0) / 4;
        let pt_y = (e1bot.1 + e1top.1 + e2bot.1 + e2top.1) / 4;

        // Handle result based on number of unique sources
        // ClipperZUtils.hpp:127-134
        // C++: if (begin + 1 == end) {
        // C++:     // Self intersection may happen on source contour. Just copy the Z value.
        // C++:     pt.z() = *begin;
        // C++: } else {
        // C++:     assert(begin + 2 == end);
        // C++:     if (begin + 2 <= end) {
        // C++:         // store a -1 based negative index into the "intersections" vector here.
        // C++:         m_intersections.emplace_back(srcs[0], srcs[1]);
        // C++:         pt.z() = -coord_t(m_intersections.size());
        // C++:     }
        // C++: }
        let pt_z = if unique_count == 1 {
            // Self-intersection: both edges from same source
            srcs[0]
        } else {
            // True intersection: edges from different sources
            debug_assert!(unique_count == 2, "Expected 1 or 2 unique source contours");
            self.intersections.push((srcs[0], srcs[1]));
            -(self.intersections.len() as i64)
        };

        (pt_x, pt_y, pt_z)
    }

    // Get reference to collected intersections
    // ClipperZUtils.hpp:140
    // C++: const std::vector<std::pair<coord_t, coord_t>>& intersections() const { return m_intersections; }
    pub fn intersections(&self) -> &Intersections {
        &self.intersections
    }

    // Take ownership of collected intersections
    pub fn into_intersections(self) -> Intersections {
        self.intersections
    }
}

impl Default for ClipperZIntersectionVisitor {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // Test Z-point comparison
    // ClipperZUtils.hpp:18-21
    #[test]
    fn test_zpoint_lower() {
        let p1 = (0, 0, 0);
        let p2 = (1, 0, 0);
        let p3 = (0, 1, 0);
        let p4 = (0, 0, 1);

        assert!(zpoint_lower(&p1, &p2)); // x differs
        assert!(zpoint_lower(&p1, &p3)); // y differs
        assert!(zpoint_lower(&p1, &p4)); // z differs
        assert!(!zpoint_lower(&p2, &p1));
    }

    // Test conversion to zpath with closed path
    // ClipperZUtils.hpp:24-36
    #[test]
    fn test_to_zpath_closed() {
        let points = vec![Point::new(0, 0), Point::new(100, 0), Point::new(100, 100)];
        let z = 42;

        let zpath = to_zpath(&points, z, false);
        assert_eq!(zpath.len(), 3);
        assert_eq!(zpath[0], (0, 0, 42));
        assert_eq!(zpath[1], (100, 0, 42));
        assert_eq!(zpath[2], (100, 100, 42));
    }

    // Test conversion to zpath with open path (duplicates first point)
    // ClipperZUtils.hpp:24-36 (Open template parameter)
    #[test]
    fn test_to_zpath_open() {
        let points = vec![Point::new(0, 0), Point::new(100, 0)];
        let z = 10;

        let zpath = to_zpath(&points, z, true);
        assert_eq!(zpath.len(), 3);
        assert_eq!(zpath[0], (0, 0, 10));
        assert_eq!(zpath[1], (100, 0, 10));
        assert_eq!(zpath[2], (0, 0, 10)); // first point duplicated
    }

    // Test expolygons_to_zpaths with index tracking
    // ClipperZUtils.hpp:52-66
    #[test]
    fn test_expolygons_to_zpaths() {
        let expoly1 = ExPolygon {
            contour: Polygon {
                points: vec![Point::new(0, 0), Point::new(100, 0), Point::new(100, 100)],
            },
            holes: vec![],
        };
        let expoly2 = ExPolygon {
            contour: Polygon {
                points: vec![Point::new(200, 0), Point::new(300, 0)],
            },
            holes: vec![Polygon {
                points: vec![Point::new(210, 10), Point::new(220, 10)],
            }],
        };

        let mut base_idx = 100;
        let zpaths = expolygons_to_zpaths(&[expoly1, expoly2], &mut base_idx, false);

        // First expolygon: 1 contour, Z=100
        assert_eq!(zpaths[0].len(), 3);
        assert_eq!(zpaths[0][0].2, 100);

        // Second expolygon: 1 contour + 1 hole, Z=101
        assert_eq!(zpaths[1].len(), 2);
        assert_eq!(zpaths[1][0].2, 101);
        assert_eq!(zpaths[2].len(), 2);
        assert_eq!(zpaths[2][0].2, 101);

        // base_idx should be incremented by 2
        assert_eq!(base_idx, 102);
    }

    // Test from_zpath conversion back to 2D
    // ClipperZUtils.hpp:86-97
    #[test]
    fn test_from_zpath() {
        let zpath = vec![(0, 0, 10), (100, 0, 10), (100, 100, 10)];

        let points = from_zpath(&zpath, false);
        assert_eq!(points.len(), 3);
        assert_eq!(points[0], Point::new(0, 0));
        assert_eq!(points[1], Point::new(100, 0));
        assert_eq!(points[2], Point::new(100, 100));
    }

    // Test intersection visitor
    // ClipperZUtils.hpp:114-143
    #[test]
    fn test_intersection_visitor() {
        let mut visitor = ClipperZIntersectionVisitor::new();

        // Self-intersection (same Z on all endpoints)
        let e1bot = (0, 0, 5);
        let e1top = (100, 100, 5);
        let e2bot = (0, 100, 5);
        let e2top = (100, 0, 5);

        let pt = visitor.process_intersection(&e1bot, &e1top, &e2bot, &e2top);
        assert_eq!(pt.2, 5); // Z unchanged for self-intersection
        assert_eq!(visitor.intersections().len(), 0);

        // True intersection (different Z values)
        visitor.reset();
        let e1bot = (0, 0, 10);
        let e1top = (100, 100, 10);
        let e2bot = (0, 100, 20);
        let e2top = (100, 0, 20);

        let pt = visitor.process_intersection(&e1bot, &e1top, &e2bot, &e2top);
        assert_eq!(pt.2, -1); // Negative index into intersections
        assert_eq!(visitor.intersections().len(), 1);
        assert_eq!(visitor.intersections()[0], (10, 20));
    }

    // Test expolygons_to_zpaths_with_same_z
    // ClipperZUtils.hpp:69-82
    #[test]
    fn test_expolygons_to_zpaths_with_same_z() {
        let expoly1 = ExPolygon {
            contour: Polygon {
                points: vec![Point::new(0, 0), Point::new(100, 0)],
            },
            holes: vec![],
        };
        let expoly2 = ExPolygon {
            contour: Polygon {
                points: vec![Point::new(200, 0), Point::new(300, 0)],
            },
            holes: vec![],
        };

        let zpaths = expolygons_to_zpaths_with_same_z(&[expoly1, expoly2], 42, false);

        // All paths should have Z=42
        assert_eq!(zpaths.len(), 2);
        assert!(zpaths[0].iter().all(|p| p.2 == 42));
        assert!(zpaths[1].iter().all(|p| p.2 == 42));
    }

    // Test multiple path conversion
    // ClipperZUtils.hpp:39-48
    #[test]
    fn test_to_zpaths() {
        let paths = vec![
            vec![Point::new(0, 0), Point::new(100, 0)],
            vec![Point::new(200, 0), Point::new(300, 0)],
        ];

        let zpaths = to_zpaths(&paths, 50, false);
        assert_eq!(zpaths.len(), 2);
        assert!(zpaths[0].iter().all(|p| p.2 == 50));
        assert!(zpaths[1].iter().all(|p| p.2 == 50));
    }

    // Test from_zpaths conversion
    // ClipperZUtils.hpp:107-112
    #[test]
    fn test_from_zpaths() {
        let zpaths = vec![
            vec![(0, 0, 10), (100, 0, 10)],
            vec![(200, 0, 20), (300, 0, 20)],
        ];

        let paths = from_zpaths(&zpaths, false);
        assert_eq!(paths.len(), 2);
        assert_eq!(paths[0].len(), 2);
        assert_eq!(paths[1].len(), 2);
    }
}
