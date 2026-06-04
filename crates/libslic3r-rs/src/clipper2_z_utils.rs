//! Z-coordinate aware utilities for Clipper2 library integration
//!
//! This module provides utilities for working with Clipper2 paths that include
//! Z-coordinates, enabling tracking of source geometry through boolean operations.
//!
//! C++ Reference: `Clipper2ZUtils.hpp`
//!
//! ## Key Concepts
//!
//! - **ZPoint64/ZPath64/ZPaths64**: Clipper2 types with Z-coordinate support
//! - **Z-coordinate tracking**: Each point carries a Z value identifying its source
//! - **Intersection detection**: Track which contours intersect during boolean ops
//! - **Contour preservation**: Maintain source information through transformations
//!
//! ## Architecture
//!
//! The module provides:
//! 1. Conversion functions (Slic3r types ↔ Clipper2 Z-types)
//! 2. Intersection visitor for detecting edge crossings
//! 3. Helper functions for batch conversions with Z-indexing

use crate::geometry::{ExPolygon, Point, Polygon};
use std::cmp::Ordering;

// ============================================================================
// Type Aliases (matching C++ exactly)
// ============================================================================

/// Clipper2 Z-aware point type
/// Clipper2ZUtils.hpp:11
/// C++: using ZPoint64 = Clipper2Lib_Z::Point64;
pub type ZPoint64 = (i64, i64, i64); // (x, y, z)

/// Clipper2 Z-aware path type (single contour)
/// Clipper2ZUtils.hpp:12
/// C++: using ZPath64 = Clipper2Lib_Z::Path64;
pub type ZPath64 = Vec<ZPoint64>;

/// Clipper2 Z-aware paths type (multiple contours)
/// Clipper2ZUtils.hpp:13
/// C++: using ZPaths64 = Clipper2Lib_Z::Paths64;
pub type ZPaths64 = Vec<ZPath64>;

// ============================================================================
// Comparison and Ordering
// ============================================================================

/// Compare two Z-points lexicographically (x, then y, then z)
/// Clipper2ZUtils.hpp:15-17
/// C++: inline bool zpoint64_lower(const ZPoint64 &l, const ZPoint64 &r) {
/// C++:     return l.x < r.x || (l.x == r.x && (l.y < r.y || (l.y == r.y && l.z < r.z)));
/// C++: }
#[inline]
pub fn zpoint64_lower(l: &ZPoint64, r: &ZPoint64) -> bool {
    l.0 < r.0 || (l.0 == r.0 && (l.1 < r.1 || (l.1 == r.1 && l.2 < r.2)))
}

/// Ordering implementation for sorting Z-points
/// Clipper2ZUtils.hpp:15-17 (derived)
#[inline]
pub fn zpoint64_cmp(l: &ZPoint64, r: &ZPoint64) -> Ordering {
    match l.0.cmp(&r.0) {
        Ordering::Equal => match l.1.cmp(&r.1) {
            Ordering::Equal => l.2.cmp(&r.2),
            other => other,
        },
        other => other,
    }
}

// ============================================================================
// Conversion: Slic3r → Clipper2 Z-types
// ============================================================================

/// Convert a single path to zpath with a given Z coordinate
/// Clipper2ZUtils.hpp:20-30
/// C++: template<bool Open = false>
/// C++: inline ZPath64 to_zpath64(const Points &path, int64_t z)
/// C++: {
/// C++:     ZPath64 out;
/// C++:     if (!path.empty()) {
/// C++:         out.reserve(path.size() + (Open ? 1 : 0));
/// C++:         for (const Point &p : path) out.emplace_back(p.x(), p.y(), z);
/// C++:         if (Open) out.emplace_back(out.front());
/// C++:     }
/// C++:     return out;
/// C++: }
pub fn to_zpath64(path: &[Point], z: i64, open: bool) -> ZPath64 {
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
/// Clipper2ZUtils.hpp:48-56
/// C++: template<bool Open = false>
/// C++: inline ZPaths64 to_zpaths64(const VecOfPoints &paths, int64_t z)
/// C++: {
/// C++:     ZPaths64 out;
/// C++:     out.reserve(paths.size());
/// C++:     for (const Points &path : paths) out.emplace_back(to_zpath64<Open>(path, z));
/// C++:     return out;
/// C++: }
pub fn to_zpaths64(paths: &[Vec<Point>], z: i64, open: bool) -> ZPaths64 {
    let mut out = Vec::with_capacity(paths.len());
    for path in paths {
        out.push(to_zpath64(path, z, open));
    }
    out
}

/// Convert multiple expolygons into zpaths with Z specified by index offset by base_idx
/// Clipper2ZUtils.hpp:60-74
/// C++: template<bool Open = false>
/// C++: inline ZPaths64 expolygons_to_zpaths64(const ExPolygons &src, int64_t &base_idx)
/// C++: {
/// C++:     ZPaths64 out;
/// C++:     out.reserve(std::accumulate(src.begin(), src.end(), size_t(0),
/// C++:         [](const size_t acc, const ExPolygon &expoly) { return acc + expoly.num_contours(); }));
/// C++:     for (const ExPolygon &expoly : src) {
/// C++:         out.emplace_back(to_zpath64<Open>(expoly.contour.points, base_idx));
/// C++:         for (const Polygon &hole : expoly.holes)
/// C++:             out.emplace_back(to_zpath64<Open>(hole.points, base_idx));
/// C++:         ++base_idx;
/// C++:     }
/// C++:     return out;
/// C++: }
pub fn expolygons_to_zpaths64(src: &[ExPolygon], base_idx: &mut i64, open: bool) -> ZPaths64 {
    // Helper to count total contours
    // Clipper2ZUtils.hpp:62-63
    fn count_contours(expolygons: &[ExPolygon]) -> usize {
        expolygons.iter().map(|e| e.num_contours()).sum()
    }

    let mut out = Vec::with_capacity(count_contours(src));
    for expoly in src {
        // Add outer contour
        // Clipper2ZUtils.hpp:69
        // C++: out.emplace_back(to_zpath64<Open>(expoly.contour.points, base_idx));
        out.push(to_zpath64(&expoly.contour.points, *base_idx, open));

        // Add holes
        // Clipper2ZUtils.hpp:70-71
        // C++: for (const Polygon &hole : expoly.holes)
        // C++:     out.emplace_back(to_zpath64<Open>(hole.points, base_idx));
        for hole in &expoly.holes {
            out.push(to_zpath64(&hole.points, *base_idx, open));
        }

        // Increment base index for next expolygon
        // Clipper2ZUtils.hpp:72
        // C++: ++base_idx;
        *base_idx += 1;
    }
    out
}

/// Convert multiple expolygons into zpaths with the same Z
/// Clipper2ZUtils.hpp:77-89
/// C++: template<bool Open = false>
/// C++: inline ZPaths64 expolygons_to_zpaths64_with_same_z(const ExPolygons &src, int64_t z)
/// C++: {
/// C++:     ZPaths64 out;
/// C++:     out.reserve(std::accumulate(src.begin(), src.end(), size_t(0),
/// C++:         [](const size_t acc, const ExPolygon &expoly) { return acc + expoly.num_contours(); }));
/// C++:     for (const ExPolygon &expoly : src) {
/// C++:         out.emplace_back(to_zpath64<Open>(expoly.contour.points, z));
/// C++:         for (const Polygon &hole : expoly.holes)
/// C++:             out.emplace_back(to_zpath64<Open>(hole.points, z));
/// C++:     }
/// C++:     return out;
/// C++: }
pub fn expolygons_to_zpaths64_with_same_z(src: &[ExPolygon], z: i64, open: bool) -> ZPaths64 {
    // Helper to count total contours
    // Clipper2ZUtils.hpp:79-80
    fn count_contours(expolygons: &[ExPolygon]) -> usize {
        expolygons.iter().map(|e| e.num_contours()).sum()
    }

    let mut out = Vec::with_capacity(count_contours(src));
    for expoly in src {
        // Add outer contour with same Z
        // Clipper2ZUtils.hpp:83
        // C++: out.emplace_back(to_zpath64<Open>(expoly.contour.points, z));
        out.push(to_zpath64(&expoly.contour.points, z, open));

        // Add holes with same Z
        // Clipper2ZUtils.hpp:84-85
        // C++: for (const Polygon &hole : expoly.holes)
        // C++:     out.emplace_back(to_zpath64<Open>(hole.points, z));
        for hole in &expoly.holes {
            out.push(to_zpath64(&hole.points, z, open));
        }
    }
    out
}

// ============================================================================
// Conversion: Clipper2 Z-types → Slic3r
// ============================================================================

/// Convert a zpath back to 2D Points
/// Clipper2ZUtils.hpp:93-102
/// C++: template<bool Open = false>
/// C++: inline Points from_zpath64(const ZPath64 &path)
/// C++: {
/// C++:     Points out;
/// C++:     if (!path.empty()) {
/// C++:         out.reserve(path.size() + (Open ? 1 : 0));
/// C++:         for (const ZPoint64 &p : path) out.emplace_back(p.x, p.y);
/// C++:         if (Open) out.emplace_back(out.front());
/// C++:     }
/// C++:     return out;
/// C++: }
pub fn from_zpath64(path: &ZPath64, open: bool) -> Vec<Point> {
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
/// Clipper2ZUtils.hpp:105-110
/// C++: template<bool Open = false>
/// C++: inline void from_zpaths64(const ZPaths64 &paths, VecOfPoints &out)
/// C++: {
/// C++:     out.reserve(out.size() + paths.size());
/// C++:     for (const ZPath64 &path : paths) out.emplace_back(from_zpath64<Open>(path));
/// C++: }
pub fn from_zpaths64_append(paths: &ZPaths64, out: &mut Vec<Vec<Point>>, open: bool) {
    out.reserve(out.len() + paths.len());
    for path in paths {
        out.push(from_zpath64(path, open));
    }
}

/// Convert multiple zpaths back to 2D paths (new vector)
/// Clipper2ZUtils.hpp:111-116
/// C++: template<bool Open = false>
/// C++: inline VecOfPoints from_zpaths64(const ZPaths64 &paths)
/// C++: {
/// C++:     VecOfPoints out;
/// C++:     from_zpaths64<Open>(paths, out);
/// C++:     return out;
/// C++: }
pub fn from_zpaths64(paths: &ZPaths64, open: bool) -> Vec<Vec<Point>> {
    let mut out = Vec::new();
    from_zpaths64_append(paths, &mut out, open);
    out
}

// ============================================================================
// Intersection Visitor
// ============================================================================

/// Intersection pair: (source_z_1, source_z_2)
/// Clipper2ZUtils.hpp:122
/// C++: using Intersection = std::pair<int64_t, int64_t>;
pub type Intersection = (i64, i64);

/// Vector of intersection pairs
/// Clipper2ZUtils.hpp:123
/// C++: using Intersections = std::vector<Intersection>;
pub type Intersections = Vec<Intersection>;

/// Intersection visitor for Clipper2 (zCallback_)
///
/// Tracks intersections between edges from different source contours during
/// Clipper2 boolean operations. When two edges intersect, their source Z-values
/// are paired and stored.
///
/// Clipper2ZUtils.hpp:119-158
/// C++: class Clipper2ZIntersectionVisitor
/// C++: {
/// C++: public:
/// C++:     using Intersection  = std::pair<int64_t, int64_t>;
/// C++:     using Intersections = std::vector<Intersection>;
/// C++:
/// C++:     Clipper2ZIntersectionVisitor(Intersections &intersections) : m_intersections(intersections) {}
/// C++:
/// C++:     void reset() { m_intersections.clear(); }
/// C++:
/// C++:     void operator()(const ZPoint64 &e1bot, const ZPoint64 &e1top, const ZPoint64 &e2bot, const ZPoint64 &e2top, ZPoint64 &pt) { ... }
/// C++:     auto clipper_callback() { ... }
/// C++:     const Intersections &intersections() const { return m_intersections; }
/// C++:
/// C++: private:
/// C++:     Intersections &m_intersections;
/// C++: };
pub struct Clipper2ZIntersectionVisitor {
    // Storage for detected intersections
    // Clipper2ZUtils.hpp:157
    // C++: Intersections &m_intersections;
    intersections: Intersections,
}

impl Clipper2ZIntersectionVisitor {
    // Create a new intersection visitor
    // Clipper2ZUtils.hpp:125
    // C++: Clipper2ZIntersectionVisitor(Intersections &intersections) : m_intersections(intersections) {}
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
    // Clipper2ZUtils.hpp:127
    // C++: void reset() { m_intersections.clear(); }
    pub fn reset(&mut self) {
        self.intersections.clear();
    }

    // Process an intersection between two edges
    //
    // When edges from two different source contours intersect, their Z-values
    // (which encode source contour indices) are paired and stored.
    //
    // Clipper2ZUtils.hpp:129-142
    // C++: void operator()(const ZPoint64 &e1bot, const ZPoint64 &e1top, const ZPoint64 &e2bot, const ZPoint64 &e2top, ZPoint64 &pt)
    // C++: {
    // C++:     std::array<int64_t, 4> srcs{e1bot.z, e1top.z, e2bot.z, e2top.z};
    // C++:     std::sort(srcs.begin(), srcs.end());
    // C++:     auto it = std::unique(srcs.begin(), srcs.end());
    // C++:     int new_size = std::distance(srcs.begin(), it);
    // C++:     assert(new_size == 1 || new_size == 2);
    // C++:     if (new_size == 1) {
    // C++:         pt.z = srcs[0];
    // C++:     }
    // C++:     else if(new_size == 2){
    // C++:         m_intersections.emplace_back(srcs[0], srcs[1]);
    // C++:         pt.z = -int64_t(m_intersections.size());
    // C++:     }
    // C++: }
    pub fn process_intersection(
        &mut self,
        e1bot: &ZPoint64,
        e1top: &ZPoint64,
        e2bot: &ZPoint64,
        e2top: &ZPoint64,
    ) -> ZPoint64 {
        // Collect all source Z values from edge endpoints
        // Clipper2ZUtils.hpp:131
        // C++: std::array<int64_t, 4> srcs{e1bot.z, e1top.z, e2bot.z, e2top.z};
        let mut srcs = [e1bot.2, e1top.2, e2bot.2, e2top.2];

        // Sort and deduplicate to find unique source contours
        // Clipper2ZUtils.hpp:132-134
        // C++: std::sort(srcs.begin(), srcs.end());
        // C++: auto it = std::unique(srcs.begin(), srcs.end());
        // C++: int new_size = std::distance(srcs.begin(), it);
        srcs.sort_unstable();
        let mut unique_count = 1;
        for i in 1..4 {
            if srcs[i] != srcs[unique_count - 1] {
                srcs[unique_count] = srcs[i];
                unique_count += 1;
            }
        }

        // Calculate intersection point (midpoint of edge intersections)
        // Not explicitly in C++ (Clipper2 handles geometry internally)
        let pt_x = (e1bot.0 + e1top.0 + e2bot.0 + e2top.0) / 4;
        let pt_y = (e1bot.1 + e1top.1 + e2bot.1 + e2top.1) / 4;

        // Handle result based on number of unique sources
        // Clipper2ZUtils.hpp:135-141
        // C++: assert(new_size == 1 || new_size == 2);
        // C++: if (new_size == 1) {
        // C++:     pt.z = srcs[0];
        // C++: }
        // C++: else if(new_size == 2){
        // C++:     m_intersections.emplace_back(srcs[0], srcs[1]);
        // C++:     pt.z = -int64_t(m_intersections.size());
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
    // Clipper2ZUtils.hpp:154
    // C++: const Intersections &intersections() const { return m_intersections; }
    pub fn intersections(&self) -> &Intersections {
        &self.intersections
    }

    // Take ownership of collected intersections
    pub fn into_intersections(self) -> Intersections {
        self.intersections
    }
}

impl Default for Clipper2ZIntersectionVisitor {
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
    // Clipper2ZUtils.hpp:15-17
    #[test]
    fn test_zpoint64_lower() {
        let p1 = (0, 0, 0);
        let p2 = (1, 0, 0);
        let p3 = (0, 1, 0);
        let p4 = (0, 0, 1);

        assert!(zpoint64_lower(&p1, &p2)); // x differs
        assert!(zpoint64_lower(&p1, &p3)); // y differs
        assert!(zpoint64_lower(&p1, &p4)); // z differs
        assert!(!zpoint64_lower(&p2, &p1));
    }

    // Test conversion to zpath with closed path
    // Clipper2ZUtils.hpp:20-30
    #[test]
    fn test_to_zpath64_closed() {
        let points = vec![Point::new(0, 0), Point::new(100, 0), Point::new(100, 100)];
        let z = 42;

        let zpath = to_zpath64(&points, z, false);
        assert_eq!(zpath.len(), 3);
        assert_eq!(zpath[0], (0, 0, 42));
        assert_eq!(zpath[1], (100, 0, 42));
        assert_eq!(zpath[2], (100, 100, 42));
    }

    // Test conversion to zpath with open path (duplicates first point)
    // Clipper2ZUtils.hpp:20-30 (Open template parameter)
    #[test]
    fn test_to_zpath64_open() {
        let points = vec![Point::new(0, 0), Point::new(100, 0)];
        let z = 10;

        let zpath = to_zpath64(&points, z, true);
        assert_eq!(zpath.len(), 3);
        assert_eq!(zpath[0], (0, 0, 10));
        assert_eq!(zpath[1], (100, 0, 10));
        assert_eq!(zpath[2], (0, 0, 10)); // first point duplicated
    }

    // Test expolygons_to_zpaths64 with index tracking
    // Clipper2ZUtils.hpp:60-74
    #[test]
    fn test_expolygons_to_zpaths64() {
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
        let zpaths = expolygons_to_zpaths64(&[expoly1, expoly2], &mut base_idx, false);

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

    // Test from_zpath64 conversion back to 2D
    // Clipper2ZUtils.hpp:93-102
    #[test]
    fn test_from_zpath64() {
        let zpath = vec![(0, 0, 10), (100, 0, 10), (100, 100, 10)];

        let points = from_zpath64(&zpath, false);
        assert_eq!(points.len(), 3);
        assert_eq!(points[0], Point::new(0, 0));
        assert_eq!(points[1], Point::new(100, 0));
        assert_eq!(points[2], Point::new(100, 100));
    }

    // Test intersection visitor
    // Clipper2ZUtils.hpp:119-158
    #[test]
    fn test_intersection_visitor() {
        let mut visitor = Clipper2ZIntersectionVisitor::new();

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

    // Test expolygons_to_zpaths64_with_same_z
    // Clipper2ZUtils.hpp:77-89
    #[test]
    fn test_expolygons_to_zpaths64_with_same_z() {
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

        let zpaths = expolygons_to_zpaths64_with_same_z(&[expoly1, expoly2], 42, false);

        // All paths should have Z=42
        assert_eq!(zpaths.len(), 2);
        assert!(zpaths[0].iter().all(|p| p.2 == 42));
        assert!(zpaths[1].iter().all(|p| p.2 == 42));
    }
}
