//! Z-coordinate aware utilities for Clipper2 library integration.
//!
//! 1:1 line-by-line port of `Clipper2ZUtils.hpp` (header-only).
//!
//! C++ Reference: `src/libslic3r/Clipper2ZUtils.hpp`
//!
//! The C++ file lives entirely inside `namespace Slic3r { namespace Clipper2ZUtils {`.
//! All entities are `inline`/template functions; this module mirrors them.
//!
//! Type mapping:
//! - `coord_t`  -> `i64`
//! - `coordf_t` -> `f64`
//! - `Clipper2Lib_Z::Point64` -> `ZPoint64` = `(i64, i64, i64)` (x, y, z)
//! - `Clipper2Lib_Z::Path64`  -> `ZPath64`  = `Vec<ZPoint64>`
//! - `Clipper2Lib_Z::Paths64` -> `ZPaths64` = `Vec<ZPath64>`
//! - `Points`      -> `&[Point]`
//! - `VecOfPoints` -> `Vec<Vec<Point>>`
//!
//! C++ templates are parameterized on `bool Open = false`. Rust has no value
//! template parameters, so `Open` is threaded through as a runtime `open: bool`
//! argument with the same default-`false` semantics at call sites.

// Clipper2ZUtils.hpp:8 -- #include <libslic3r/Point.hpp>
// (`Polygon` from C++ `for (const Polygon &hole : expoly.holes)` is only named
//  explicitly in the test module below.)
use crate::geometry::{ExPolygon, Point};

// ============================================================================
// Type Aliases
// ============================================================================

// Clipper2ZUtils.hpp:11 -- using ZPoint64  = Clipper2Lib_Z::Point64;
/// Clipper2 Z-aware point: `(x, y, z)`.
pub type ZPoint64 = (i64, i64, i64);

// Clipper2ZUtils.hpp:12 -- using ZPoints64 = Clipper2Lib_Z::Path64;
/// Clipper2 Z-aware sequence of points (alias of `ZPath64`).
pub type ZPoints64 = Vec<ZPoint64>;

// Clipper2ZUtils.hpp:13 -- using ZPath64  = Clipper2Lib_Z::Path64;
/// Clipper2 Z-aware path (single contour).
pub type ZPath64 = Vec<ZPoint64>;

// Clipper2ZUtils.hpp:14 -- using ZPaths64 = Clipper2Lib_Z::Paths64;
/// Clipper2 Z-aware paths (multiple contours).
pub type ZPaths64 = Vec<ZPath64>;

// ============================================================================
// Comparison
// ============================================================================

// Clipper2ZUtils.hpp:16 -- inline bool zpoint64_lower(const ZPoint64 &l, const ZPoint64 &r) {
// Clipper2ZUtils.hpp:17 --     return l.x < r.x || (l.x == r.x && (l.y < r.y || (l.y == r.y && l.z < r.z)));
// Clipper2ZUtils.hpp:18 -- }
#[inline]
pub fn zpoint64_lower(l: &ZPoint64, r: &ZPoint64) -> bool {
    l.0 < r.0 || (l.0 == r.0 && (l.1 < r.1 || (l.1 == r.1 && l.2 < r.2)))
}

// ============================================================================
// Conversion: Slic3r / Clipper2 -> Clipper2 Z-types
// ============================================================================

// Clipper2ZUtils.hpp:20 -- // Convert a single path to zpath with a given Z coordinate.
// Clipper2ZUtils.hpp:21 -- // If Open, then duplicate the first point at the end.
// Clipper2ZUtils.hpp:22 -- template<bool Open = false>
// Clipper2ZUtils.hpp:23 -- inline ZPath64 to_zpath64(const Points &path, int64_t z)
// Clipper2ZUtils.hpp:24 -- {
// Clipper2ZUtils.hpp:25 --     ZPath64 out;
// Clipper2ZUtils.hpp:26 --     if (!path.empty()) {
// Clipper2ZUtils.hpp:27 --         out.reserve(path.size() + (Open ? 1 : 0));
// Clipper2ZUtils.hpp:28 --         for (const Point &p : path) out.emplace_back(p.x(), p.y(), z);
// Clipper2ZUtils.hpp:29 --         if (Open) out.emplace_back(out.front());
// Clipper2ZUtils.hpp:30 --     }
// Clipper2ZUtils.hpp:31 --     return out;
// Clipper2ZUtils.hpp:32 -- }
#[inline]
pub fn to_zpath64(path: &[Point], z: i64, open: bool) -> ZPath64 {
    let mut out: ZPath64 = ZPath64::new();
    if !path.is_empty() {
        out.reserve(path.len() + if open { 1 } else { 0 });
        for p in path {
            out.push((p.x(), p.y(), z));
        }
        if open {
            out.push(out[0]);
        }
    }
    out
}

// Clipper2ZUtils.hpp:34 -- template<bool Open = false>
// Clipper2ZUtils.hpp:35 -- inline ZPath64 to_zpath64(const Clipper2Lib_Z::Path64 &path, int64_t z)
// Clipper2ZUtils.hpp:36 -- {
// Clipper2ZUtils.hpp:37 --     ZPath64 out;
// Clipper2ZUtils.hpp:38 --     if (!path.empty()) {
// Clipper2ZUtils.hpp:39 --         out.reserve(path.size() + (Open ? 1 : 0));
// Clipper2ZUtils.hpp:40 --         for (const Clipper2Lib_Z::Point64 &p : path) out.emplace_back(p.x, p.y, z);
// Clipper2ZUtils.hpp:41 --         if (Open) out.emplace_back(out.front());
// Clipper2ZUtils.hpp:42 --     }
// Clipper2ZUtils.hpp:43 --     return out;
// Clipper2ZUtils.hpp:44 -- }
//
// Overload of `to_zpath64` taking a `ZPath64` (Clipper2Lib_Z::Path64). Rust has
// no overloading by argument type without traits, so this mirrors the second
// C++ overload under a distinct name.
#[inline]
pub fn to_zpath64_from_zpath(path: &ZPath64, z: i64, open: bool) -> ZPath64 {
    let mut out: ZPath64 = ZPath64::new();
    if !path.is_empty() {
        out.reserve(path.len() + if open { 1 } else { 0 });
        for p in path {
            out.push((p.0, p.1, z));
        }
        if open {
            out.push(out[0]);
        }
    }
    out
}

// Clipper2ZUtils.hpp:46 -- // Convert multiple paths to zpaths with a given Z coordinate.
// Clipper2ZUtils.hpp:47 -- template<bool Open = false>
// Clipper2ZUtils.hpp:48 -- inline ZPaths64 to_zpaths64(const VecOfPoints &paths, int64_t z)
// Clipper2ZUtils.hpp:49 -- {
// Clipper2ZUtils.hpp:50 --     ZPaths64 out;
// Clipper2ZUtils.hpp:51 --     out.reserve(paths.size());
// Clipper2ZUtils.hpp:52 --     for (const Points &path : paths) out.emplace_back(to_zpath64<Open>(path, z));
// Clipper2ZUtils.hpp:53 --     return out;
// Clipper2ZUtils.hpp:54 -- }
#[inline]
pub fn to_zpaths64(paths: &[Vec<Point>], z: i64, open: bool) -> ZPaths64 {
    let mut out: ZPaths64 = ZPaths64::new();
    out.reserve(paths.len());
    for path in paths {
        out.push(to_zpath64(path, z, open));
    }
    out
}

// Clipper2ZUtils.hpp:56 -- template<bool Open = false>
// Clipper2ZUtils.hpp:57 -- inline ZPaths64 to_zpaths64(const Clipper2Lib_Z::Paths64 &paths, int64_t z)
// Clipper2ZUtils.hpp:58 -- {
// Clipper2ZUtils.hpp:59 --     ZPaths64 out;
// Clipper2ZUtils.hpp:60 --     out.reserve(paths.size());
// Clipper2ZUtils.hpp:61 --     for (const Clipper2Lib_Z::Path64 &path : paths) out.emplace_back(to_zpath64<Open>(path, z));
// Clipper2ZUtils.hpp:62 --     return out;
// Clipper2ZUtils.hpp:63 -- }
//
// Overload of `to_zpaths64` taking `ZPaths64` (Clipper2Lib_Z::Paths64).
#[inline]
pub fn to_zpaths64_from_zpaths(paths: &ZPaths64, z: i64, open: bool) -> ZPaths64 {
    let mut out: ZPaths64 = ZPaths64::new();
    out.reserve(paths.len());
    for path in paths {
        out.push(to_zpath64_from_zpath(path, z, open));
    }
    out
}

// Clipper2ZUtils.hpp:65 -- // Convert multiple expolygons into zpaths with Z specified by index
// Clipper2ZUtils.hpp:66 -- // offset by base_idx.
// Clipper2ZUtils.hpp:67 -- template<bool Open = false>
// Clipper2ZUtils.hpp:68 -- inline ZPaths64 expolygons_to_zpaths64(const ExPolygons &src, int64_t &base_idx)
// Clipper2ZUtils.hpp:69 -- {
// Clipper2ZUtils.hpp:70 --     ZPaths64 out;
// Clipper2ZUtils.hpp:71 --     out.reserve(std::accumulate(src.begin(), src.end(), size_t(0),
// Clipper2ZUtils.hpp:72 --         [](const size_t acc, const ExPolygon &expoly) { return acc + expoly.num_contours(); }));
// Clipper2ZUtils.hpp:73 --     for (const ExPolygon &expoly : src) {
// Clipper2ZUtils.hpp:74 --         out.emplace_back(to_zpath64<Open>(expoly.contour.points, base_idx));
// Clipper2ZUtils.hpp:75 --         for (const Polygon &hole : expoly.holes)
// Clipper2ZUtils.hpp:76 --             out.emplace_back(to_zpath64<Open>(hole.points, base_idx));
// Clipper2ZUtils.hpp:77 --         ++base_idx;
// Clipper2ZUtils.hpp:78 --     }
// Clipper2ZUtils.hpp:79 --     return out;
// Clipper2ZUtils.hpp:80 -- }
#[inline]
pub fn expolygons_to_zpaths64(src: &[ExPolygon], base_idx: &mut i64, open: bool) -> ZPaths64 {
    let mut out: ZPaths64 = ZPaths64::new();
    // Clipper2ZUtils.hpp:71-72 -- std::accumulate(... acc + expoly.num_contours())
    out.reserve(
        src.iter()
            .fold(0usize, |acc, expoly| acc + expoly.num_contours()),
    );
    for expoly in src {
        // Clipper2ZUtils.hpp:74 -- out.emplace_back(to_zpath64<Open>(expoly.contour.points, base_idx));
        out.push(to_zpath64(&expoly.contour.points, *base_idx, open));
        // Clipper2ZUtils.hpp:75-76 -- for (const Polygon &hole : expoly.holes)
        //                                 out.emplace_back(to_zpath64<Open>(hole.points, base_idx));
        for hole in &expoly.holes {
            out.push(to_zpath64(&hole.points, *base_idx, open));
        }
        // Clipper2ZUtils.hpp:77 -- ++base_idx;
        *base_idx += 1;
    }
    out
}

// Clipper2ZUtils.hpp:82 -- // Convert multiple expolygons into zpaths with the same Z.
// Clipper2ZUtils.hpp:83 -- template<bool Open = false>
// Clipper2ZUtils.hpp:84 -- inline ZPaths64 expolygons_to_zpaths64_with_same_z(const ExPolygons &src, int64_t z)
// Clipper2ZUtils.hpp:85 -- {
// Clipper2ZUtils.hpp:86 --     ZPaths64 out;
// Clipper2ZUtils.hpp:87 --     out.reserve(std::accumulate(src.begin(), src.end(), size_t(0),
// Clipper2ZUtils.hpp:88 --         [](const size_t acc, const ExPolygon &expoly) { return acc + expoly.num_contours(); }));
// Clipper2ZUtils.hpp:89 --     for (const ExPolygon &expoly : src) {
// Clipper2ZUtils.hpp:90 --         out.emplace_back(to_zpath64<Open>(expoly.contour.points, z));
// Clipper2ZUtils.hpp:91 --         for (const Polygon &hole : expoly.holes)
// Clipper2ZUtils.hpp:92 --             out.emplace_back(to_zpath64<Open>(hole.points, z));
// Clipper2ZUtils.hpp:93 --     }
// Clipper2ZUtils.hpp:94 --     return out;
// Clipper2ZUtils.hpp:95 -- }
#[inline]
pub fn expolygons_to_zpaths64_with_same_z(src: &[ExPolygon], z: i64, open: bool) -> ZPaths64 {
    let mut out: ZPaths64 = ZPaths64::new();
    // Clipper2ZUtils.hpp:87-88 -- std::accumulate(... acc + expoly.num_contours())
    out.reserve(
        src.iter()
            .fold(0usize, |acc, expoly| acc + expoly.num_contours()),
    );
    for expoly in src {
        // Clipper2ZUtils.hpp:90 -- out.emplace_back(to_zpath64<Open>(expoly.contour.points, z));
        out.push(to_zpath64(&expoly.contour.points, z, open));
        // Clipper2ZUtils.hpp:91-92 -- for (const Polygon &hole : expoly.holes)
        //                                 out.emplace_back(to_zpath64<Open>(hole.points, z));
        for hole in &expoly.holes {
            out.push(to_zpath64(&hole.points, z, open));
        }
    }
    out
}

// ============================================================================
// Conversion: Clipper2 Z-types -> Slic3r
// ============================================================================

// Clipper2ZUtils.hpp:97  -- // Convert a zpath back to 2D Points.
// Clipper2ZUtils.hpp:98  -- // If Open, then duplicate the first point at the end.
// Clipper2ZUtils.hpp:99  -- template<bool Open = false>
// Clipper2ZUtils.hpp:100 -- inline Points from_zpath64(const ZPath64 &path)
// Clipper2ZUtils.hpp:101 -- {
// Clipper2ZUtils.hpp:102 --     Points out;
// Clipper2ZUtils.hpp:103 --     if (!path.empty()) {
// Clipper2ZUtils.hpp:104 --         out.reserve(path.size() + (Open ? 1 : 0));
// Clipper2ZUtils.hpp:105 --         for (const ZPoint64 &p : path) out.emplace_back(p.x, p.y);
// Clipper2ZUtils.hpp:106 --         if (Open) out.emplace_back(out.front());
// Clipper2ZUtils.hpp:107 --     }
// Clipper2ZUtils.hpp:108 --     return out;
// Clipper2ZUtils.hpp:109 -- }
#[inline]
pub fn from_zpath64(path: &ZPath64, open: bool) -> Vec<Point> {
    let mut out: Vec<Point> = Vec::new();
    if !path.is_empty() {
        out.reserve(path.len() + if open { 1 } else { 0 });
        for p in path {
            out.push(Point::new(p.0, p.1));
        }
        if open {
            out.push(out[0]);
        }
    }
    out
}

// Clipper2ZUtils.hpp:111 -- // Convert multiple zpaths back to 2D paths.
// Clipper2ZUtils.hpp:112 -- template<bool Open = false>
// Clipper2ZUtils.hpp:113 -- inline void from_zpaths64(const ZPaths64 &paths, VecOfPoints &out)
// Clipper2ZUtils.hpp:114 -- {
// Clipper2ZUtils.hpp:115 --     out.reserve(out.size() + paths.size());
// Clipper2ZUtils.hpp:116 --     for (const ZPath64 &path : paths) out.emplace_back(from_zpath64<Open>(path));
// Clipper2ZUtils.hpp:117 -- }
#[inline]
pub fn from_zpaths64_append(paths: &ZPaths64, out: &mut Vec<Vec<Point>>, open: bool) {
    out.reserve(out.len() + paths.len());
    for path in paths {
        out.push(from_zpath64(path, open));
    }
}

// Clipper2ZUtils.hpp:118 -- template<bool Open = false>
// Clipper2ZUtils.hpp:119 -- inline VecOfPoints from_zpaths64(const ZPaths64 &paths)
// Clipper2ZUtils.hpp:120 -- {
// Clipper2ZUtils.hpp:121 --     VecOfPoints out;
// Clipper2ZUtils.hpp:122 --     from_zpaths64<Open>(paths, out);
// Clipper2ZUtils.hpp:123 --     return out;
// Clipper2ZUtils.hpp:124 -- }
#[inline]
pub fn from_zpaths64(paths: &ZPaths64, open: bool) -> Vec<Vec<Point>> {
    let mut out: Vec<Vec<Point>> = Vec::new();
    from_zpaths64_append(paths, &mut out, open);
    out
}

// ============================================================================
// Intersection visitor for Clipper2 (zCallback_)
// ============================================================================

// Clipper2ZUtils.hpp:130 -- using Intersection  = std::pair<int64_t, int64_t>;
/// Intersection pair: `(src0, src1)` of two distinct source Z values.
pub type Intersection = (i64, i64);

// Clipper2ZUtils.hpp:131 -- using Intersections = std::vector<Intersection>;
/// Vector of intersection pairs.
pub type Intersections = Vec<Intersection>;

// Clipper2ZUtils.hpp:126 -- // Intersection visitor for Clipper2 (zCallback_).
// Clipper2ZUtils.hpp:127 -- class Clipper2ZIntersectionVisitor
// Clipper2ZUtils.hpp:128 -- {
// Clipper2ZUtils.hpp:129 -- public:
//
// NOTE (divergence): the C++ class stores `Intersections &m_intersections;` (a
// reference to caller-owned storage). The Rust port owns the `Intersections`
// vector directly to avoid threading a borrowed lifetime through the Clipper2
// callback. Observable behavior (contents, ordering, returned `pt.z` values) is
// identical; only the storage ownership differs.
pub struct Clipper2ZIntersectionVisitor {
    // Clipper2ZUtils.hpp:163 -- Intersections &m_intersections;
    m_intersections: Intersections,
}

impl Clipper2ZIntersectionVisitor {
    // Clipper2ZUtils.hpp:133 -- Clipper2ZIntersectionVisitor(Intersections &intersections) : m_intersections(intersections) {}
    #[inline]
    pub fn new() -> Self {
        Self {
            m_intersections: Intersections::new(),
        }
    }

    // Clipper2ZUtils.hpp:135 -- void reset() { m_intersections.clear(); }
    #[inline]
    pub fn reset(&mut self) {
        self.m_intersections.clear();
    }

    // Clipper2ZUtils.hpp:137 -- void operator()(const ZPoint64 &e1bot, const ZPoint64 &e1top, const ZPoint64 &e2bot, const ZPoint64 &e2top, ZPoint64 &pt)
    // Clipper2ZUtils.hpp:138 -- {
    // Clipper2ZUtils.hpp:139 --     std::array<int64_t, 4> srcs{e1bot.z, e1top.z, e2bot.z, e2top.z};
    // Clipper2ZUtils.hpp:140 --     std::sort(srcs.begin(), srcs.end());
    // Clipper2ZUtils.hpp:141 --     auto it = std::unique(srcs.begin(), srcs.end());
    // Clipper2ZUtils.hpp:142 --     int new_size = std::distance(srcs.begin(), it);
    // Clipper2ZUtils.hpp:143 --     assert(new_size == 1 || new_size == 2);
    // Clipper2ZUtils.hpp:144 --     if (new_size == 1) {
    // Clipper2ZUtils.hpp:145 --         pt.z = srcs[0];
    // Clipper2ZUtils.hpp:146 --     }
    // Clipper2ZUtils.hpp:147 --     else if(new_size == 2){
    // Clipper2ZUtils.hpp:148 --         m_intersections.emplace_back(srcs[0], srcs[1]);
    // Clipper2ZUtils.hpp:149 --         pt.z = -int64_t(m_intersections.size());
    // Clipper2ZUtils.hpp:150 --     }
    // Clipper2ZUtils.hpp:151 -- }
    //
    // `pt` is an in/out reference: Clipper2 has already computed `pt.x`/`pt.y`;
    // only `pt.z` is (conditionally) modified here. The `.0`/`.1` fields are
    // left untouched, matching C++.
    #[inline]
    pub fn call(
        &mut self,
        e1bot: &ZPoint64,
        e1top: &ZPoint64,
        e2bot: &ZPoint64,
        e2top: &ZPoint64,
        pt: &mut ZPoint64,
    ) {
        // Clipper2ZUtils.hpp:139 -- std::array<int64_t, 4> srcs{e1bot.z, e1top.z, e2bot.z, e2top.z};
        let mut srcs: [i64; 4] = [e1bot.2, e1top.2, e2bot.2, e2top.2];
        // Clipper2ZUtils.hpp:140 -- std::sort(srcs.begin(), srcs.end());
        srcs.sort_unstable();
        // Clipper2ZUtils.hpp:141-142 -- std::unique + std::distance => count of unique adjacent values
        let mut new_size: usize = 1;
        for i in 1..4 {
            if srcs[i] != srcs[new_size - 1] {
                srcs[new_size] = srcs[i];
                new_size += 1;
            }
        }
        // Clipper2ZUtils.hpp:143 -- assert(new_size == 1 || new_size == 2);
        debug_assert!(new_size == 1 || new_size == 2);
        if new_size == 1 {
            // Clipper2ZUtils.hpp:145 -- pt.z = srcs[0];
            pt.2 = srcs[0];
        } else if new_size == 2 {
            // Clipper2ZUtils.hpp:148 -- m_intersections.emplace_back(srcs[0], srcs[1]);
            self.m_intersections.push((srcs[0], srcs[1]));
            // Clipper2ZUtils.hpp:149 -- pt.z = -int64_t(m_intersections.size());
            pt.2 = -(self.m_intersections.len() as i64);
        }
    }

    // Clipper2ZUtils.hpp:153 -- auto clipper_callback()
    // Clipper2ZUtils.hpp:154 -- {
    // Clipper2ZUtils.hpp:155 --     return [this](const ZPoint64 &e1bot, const ZPoint64 &e1top,
    // Clipper2ZUtils.hpp:156 --                   const ZPoint64 &e2bot, const ZPoint64 &e2top, ZPoint64 &pt) {
    // Clipper2ZUtils.hpp:157 --         return (*this)(e1bot, e1top, e2bot, e2top, pt); };
    // Clipper2ZUtils.hpp:158 -- }
    //
    // The C++ returns a lambda capturing `this`. The Rust equivalent is a
    // closure borrowing `self` mutably; it forwards to `call`.
    #[inline]
    pub fn clipper_callback(
        &mut self,
    ) -> impl FnMut(&ZPoint64, &ZPoint64, &ZPoint64, &ZPoint64, &mut ZPoint64) + '_ {
        move |e1bot, e1top, e2bot, e2top, pt| self.call(e1bot, e1top, e2bot, e2top, pt)
    }

    // Clipper2ZUtils.hpp:160 -- const Intersections &intersections() const { return m_intersections; }
    #[inline]
    pub fn intersections(&self) -> &Intersections {
        &self.m_intersections
    }
}

impl Default for Clipper2ZIntersectionVisitor {
    #[inline]
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
    use crate::geometry::Polygon;

    // Clipper2ZUtils.hpp:16-18
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

    // Clipper2ZUtils.hpp:22-32
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

    // Clipper2ZUtils.hpp:22-32 (Open template parameter)
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

    // Clipper2ZUtils.hpp:34-44 (ZPath64 overload)
    #[test]
    fn test_to_zpath64_from_zpath() {
        let src: ZPath64 = vec![(0, 0, 7), (100, 0, 7), (100, 100, 7)];
        let zpath = to_zpath64_from_zpath(&src, 42, false);
        assert_eq!(zpath.len(), 3);
        assert_eq!(zpath[0], (0, 0, 42));
        assert_eq!(zpath[2], (100, 100, 42));
    }

    // Clipper2ZUtils.hpp:67-80
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

    // Clipper2ZUtils.hpp:100-109
    #[test]
    fn test_from_zpath64() {
        let zpath = vec![(0, 0, 10), (100, 0, 10), (100, 100, 10)];

        let points = from_zpath64(&zpath, false);
        assert_eq!(points.len(), 3);
        assert_eq!(points[0], Point::new(0, 0));
        assert_eq!(points[1], Point::new(100, 0));
        assert_eq!(points[2], Point::new(100, 100));
    }

    // Clipper2ZUtils.hpp:127-164
    #[test]
    fn test_intersection_visitor() {
        let mut visitor = Clipper2ZIntersectionVisitor::new();

        // Self-intersection (same Z on all endpoints).
        let e1bot = (0, 0, 5);
        let e1top = (100, 100, 5);
        let e2bot = (0, 100, 5);
        let e2top = (100, 0, 5);
        // pt comes in with Clipper2-computed x/y; only z should change.
        let mut pt = (50, 50, 999);
        visitor.call(&e1bot, &e1top, &e2bot, &e2top, &mut pt);
        assert_eq!(pt, (50, 50, 5)); // x/y untouched, z = srcs[0]
        assert_eq!(visitor.intersections().len(), 0);

        // True intersection (different Z values).
        visitor.reset();
        let e1bot = (0, 0, 10);
        let e1top = (100, 100, 10);
        let e2bot = (0, 100, 20);
        let e2top = (100, 0, 20);
        let mut pt = (50, 50, 999);
        visitor.call(&e1bot, &e1top, &e2bot, &e2top, &mut pt);
        assert_eq!(pt, (50, 50, -1)); // negative index into intersections
        assert_eq!(visitor.intersections().len(), 1);
        assert_eq!(visitor.intersections()[0], (10, 20));
    }

    // Clipper2ZUtils.hpp:153-158 (callback wrapper)
    #[test]
    fn test_clipper_callback() {
        let mut visitor = Clipper2ZIntersectionVisitor::new();
        {
            let mut cb = visitor.clipper_callback();
            let mut pt = (1, 2, 999);
            cb(&(0, 0, 3), &(1, 1, 3), &(0, 1, 8), &(1, 0, 8), &mut pt);
            assert_eq!(pt, (1, 2, -1));
        }
        assert_eq!(visitor.intersections()[0], (3, 8));
    }

    // Clipper2ZUtils.hpp:84-95
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
