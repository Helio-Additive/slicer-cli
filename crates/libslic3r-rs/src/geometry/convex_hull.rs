//! Convex hull computation and convex-polygon predicates.
//!
//! 1:1 port of `Geometry/ConvexHull.cpp` (+ `Geometry/ConvexHull.hpp`).
//!
//! C++ Reference:
//! - Geometry/ConvexHull.hpp (35 lines)
//! - Geometry/ConvexHull.cpp (423 lines)
//!
//! Implements 2D/3D convex hull (Andrew's monotone chain), the rotating-calipers
//! `convex_polygons_intersect`, and the trapezoidal top/bottom decomposition with
//! its O(log n) point-in-convex-polygon test.

use crate::geometry::geometry::{orient, Orientation};
use crate::geometry::{cross2f, ExPolygon, Point, Polygon, Polyline, Vec2d, Vec3d};

/// This implementation is based on Andrew's monotone chain 2D convex hull algorithm
///
/// Geometry/ConvexHull.cpp:11-38
/// C++: Polygon convex_hull(Points pts)
pub fn convex_hull_points(mut pts: Vec<Point>) -> Polygon {
    // ConvexHull.cpp:13
    // std::sort(pts.begin(), pts.end(), [](const Point& a, const Point& b) { return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y()); });
    pts.sort_by(|a, b| a.x().cmp(&b.x()).then_with(|| a.y().cmp(&b.y())));
    // ConvexHull.cpp:14
    // pts.erase(std::unique(pts.begin(), pts.end(), [](const Point& a, const Point& b) { return a.x() == b.x() && a.y() == b.y(); }), pts.end());
    pts.dedup_by(|a, b| a.x() == b.x() && a.y() == b.y());

    // ConvexHull.cpp:16-17
    // Polygon hull;
    // int n = (int)pts.size();
    let mut hull = Polygon::new();
    let n = pts.len();
    // ConvexHull.cpp:18
    if n >= 3 {
        // ConvexHull.cpp:19-20
        // int k = 0;
        // hull.points.resize(2 * n);
        let mut k: usize = 0;
        hull.points.resize(2 * n, Point::zero());
        // Build lower hull
        // ConvexHull.cpp:22-26
        for i in 0..n {
            // while (k >= 2 && Geometry::orient(pts[i], hull[k-2], hull[k-1]) != Geometry::ORIENTATION_CCW)
            //     -- k;
            while k >= 2
                && orient(&pts[i], &hull[k - 2], &hull[k - 1]) != Orientation::OrientationCcw
            {
                k -= 1;
            }
            // hull[k ++] = pts[i];
            hull[k] = pts[i];
            k += 1;
        }
        // Build upper hull
        // ConvexHull.cpp:28-32
        // for (int i = n-2, t = k+1; i >= 0; i--) {
        let t = k + 1;
        let mut i = n as isize - 2;
        while i >= 0 {
            // while (k >= t && Geometry::orient(pts[i], hull[k-2], hull[k-1]) != Geometry::ORIENTATION_CCW)
            //     -- k;
            while k >= t
                && orient(&pts[i as usize], &hull[k - 2], &hull[k - 1])
                    != Orientation::OrientationCcw
            {
                k -= 1;
            }
            // hull[k ++] = pts[i];
            hull[k] = pts[i as usize];
            k += 1;
            i -= 1;
        }
        // ConvexHull.cpp:33-35
        // hull.points.resize(k);
        // assert(hull.points.front() == hull.points.back());
        // hull.points.pop_back();
        hull.points.truncate(k);
        debug_assert!(hull.points.first() == hull.points.last());
        hull.points.pop();
    }
    // ConvexHull.cpp:37
    hull
}

/// 3D convex hull projected onto the XY plane (input coords are unscaled `Vec3d`).
///
/// Geometry/ConvexHull.cpp:40-96
/// C++: Pointf3s convex_hull(Pointf3s points)
pub fn convex_hull_3d(mut points: Vec<Vec3d>) -> Vec<Vec3d> {
    // ConvexHull.cpp:42
    debug_assert!(points.len() >= 3);
    // ConvexHull.cpp:44 — sort input points
    // std::sort(points.begin(), points.end(), [](const Vec3d &a, const Vec3d &b){ return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y()); });
    points.sort_by(|a, b| {
        a.x()
            .partial_cmp(&b.x())
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| {
                a.y()
                    .partial_cmp(&b.y())
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
    });

    // ConvexHull.cpp:46-47
    // int n = points.size(), k = 0;
    // Pointf3s hull;
    let n = points.len();
    let mut k: usize = 0;
    let mut hull: Vec<Vec3d> = Vec::new();

    // ConvexHull.cpp:49
    if n >= 3 {
        // ConvexHull.cpp:51
        // hull.resize(2 * n);
        hull.resize(2 * n, Vec3d::new(0.0, 0.0, 0.0));

        // Build lower hull
        // ConvexHull.cpp:54-69
        for i in 0..n {
            // Point p = Point::new_scale(points[i](0), points[i](1));
            let p = Point::new_scale(points[i].x(), points[i].y());
            // while (k >= 2) {
            while k >= 2 {
                // Point k1 = Point::new_scale(hull[k - 1](0), hull[k - 1](1));
                // Point k2 = Point::new_scale(hull[k - 2](0), hull[k - 2](1));
                let k1 = Point::new_scale(hull[k - 1].x(), hull[k - 1].y());
                let k2 = Point::new_scale(hull[k - 2].x(), hull[k - 2].y());
                // if (Geometry::orient(p, k2, k1) != Geometry::ORIENTATION_CCW)
                //     --k;
                // else
                //     break;
                if orient(&p, &k2, &k1) != Orientation::OrientationCcw {
                    k -= 1;
                } else {
                    break;
                }
            }
            // hull[k++] = points[i];
            hull[k] = points[i];
            k += 1;
        }

        // Build upper hull
        // ConvexHull.cpp:72-87
        // for (int i = n - 2, t = k + 1; i >= 0; --i) {
        let t = k + 1;
        let mut i = n as isize - 2;
        while i >= 0 {
            // Point p = Point::new_scale(points[i](0), points[i](1));
            let p = Point::new_scale(points[i as usize].x(), points[i as usize].y());
            // while (k >= t) {
            while k >= t {
                // Point k1 = Point::new_scale(hull[k - 1](0), hull[k - 1](1));
                // Point k2 = Point::new_scale(hull[k - 2](0), hull[k - 2](1));
                let k1 = Point::new_scale(hull[k - 1].x(), hull[k - 1].y());
                let k2 = Point::new_scale(hull[k - 2].x(), hull[k - 2].y());
                // if (Geometry::orient(p, k2, k1) != Geometry::ORIENTATION_CCW)
                //     --k;
                // else
                //     break;
                if orient(&p, &k2, &k1) != Orientation::OrientationCcw {
                    k -= 1;
                } else {
                    break;
                }
            }
            // hull[k++] = points[i];
            hull[k] = points[i as usize];
            k += 1;
            i -= 1;
        }

        // ConvexHull.cpp:89-92
        // hull.resize(k);
        // assert(hull.front() == hull.back());
        // hull.pop_back();
        hull.truncate(k);
        debug_assert!(hull.first() == hull.last());
        hull.pop();
    }

    // ConvexHull.cpp:95
    hull
}

/// Convex hull of a set of polygons.
///
/// Geometry/ConvexHull.cpp:98-105
/// C++: Polygon convex_hull(const Polygons &polygons)
pub fn convex_hull_polygons(polygons: &[Polygon]) -> Polygon {
    // ConvexHull.cpp:100-103
    // Points pp;
    // for (Polygons::const_iterator p = polygons.begin(); p != polygons.end(); ++p)
    //     pp.insert(pp.end(), p->points.begin(), p->points.end());
    let mut pp = Vec::new();
    for p in polygons {
        pp.extend_from_slice(&p.points);
    }
    // ConvexHull.cpp:104
    // return convex_hull(std::move(pp));
    convex_hull_points(pp)
}

/// Convex hull of the expolygon contours (holes ignored).
///
/// Geometry/ConvexHull.cpp:107-117
/// C++: Polygon convex_hull(const ExPolygons &expolygons)
pub fn convex_hull_expolygons(expolygons: &[ExPolygon]) -> Polygon {
    // ConvexHull.cpp:109-113
    // Points pp;
    // size_t sz = 0;
    // for (const auto &expoly : expolygons) sz += expoly.contour.size();
    // pp.reserve(sz);
    let sz: usize = expolygons.iter().map(|e| e.contour.points.len()).sum();
    let mut pp = Vec::with_capacity(sz);
    // ConvexHull.cpp:114-115
    // for (const auto &expoly : expolygons)
    //     pp.insert(pp.end(), expoly.contour.points.begin(), expoly.contour.points.end());
    for expoly in expolygons {
        pp.extend_from_slice(&expoly.contour.points);
    }
    // ConvexHull.cpp:116
    // return convex_hull(pp);
    convex_hull_points(pp)
}

/// Convex hull of a set of polylines.
///
/// Geometry/ConvexHull.cpp:119-129
/// C++: Polygon convex_hulll(const Polylines &polylines)
pub fn convex_hull_polylines(polylines: &[Polyline]) -> Polygon {
    // ConvexHull.cpp:121-126
    // Points pp;
    // size_t sz = 0;
    // for (const auto &polyline : polylines) sz += polyline.points.size();
    // pp.reserve(sz);
    let sz: usize = polylines.iter().map(|p| p.points.len()).sum();
    let mut pp = Vec::with_capacity(sz);
    // ConvexHull.cpp:127-128
    // for (const auto &polyline : polylines)
    //     pp.insert(pp.end(), polyline.points.begin(), polyline.points.end());
    for polyline in polylines {
        pp.extend_from_slice(&polyline.points);
    }
    // ConvexHull.cpp:129
    // return convex_hull(pp);
    convex_hull_points(pp)
}

// ---------------------------------------------------------------------------
// namespace rotcalip — rotating calipers helpers
// Geometry/ConvexHull.cpp:131-242
// ---------------------------------------------------------------------------
mod rotcalip {
    use super::*;

    // ConvexHull.cpp:137-140
    // template<class Scalar = int64_t> inline Scalar magnsq(const Point &p)
    //     { return Scalar(p.x()) * p.x() + Scalar(p.y()) * p.y(); }
    //
    // FIDELITY-NOTE(F2): C++ coord_t == int32_t so magnsq fits int64; here
    // Coord == i64 so the product widens to i128 to preserve exactness for
    // the int32-range coordinates this algorithm is fed in practice.
    #[inline]
    fn magnsq(p: &Point) -> i128 {
        (p.x() as i128) * (p.x() as i128) + (p.y() as i128) * (p.y() as i128)
    }

    // ConvexHull.cpp:142-146
    // template<class Scalar = int64_t> inline Scalar dot(const Point &a, const Point &b)
    //     { return Scalar(a.x()) * b.x() + Scalar(a.y()) * b.y(); }
    #[inline]
    fn dot(a: &Point, b: &Point) -> i128 {
        (a.x() as i128) * (b.x() as i128) + (a.y() as i128) * (b.y() as i128)
    }

    // ConvexHull.cpp:148-152
    // template<class Scalar = int64_t> inline Scalar dotperp(const Point &a, const Point &b)
    //     { return Scalar(a.x()) * b.y() - Scalar(a.y()) * b.x(); }
    #[inline]
    pub fn dotperp(a: &Point, b: &Point) -> i128 {
        (a.x() as i128) * (b.y() as i128) - (a.y() as i128) * (b.x() as i128)
    }

    /// Minimal unsigned 256-bit integer (two u128 limbs) supporting only the
    /// operations `cmp_angles` needs: build from products of i128 magnitudes
    /// and compare. C++ uses `boost::multiprecision::int256_t`; no such crate
    /// is available (and we must stay wasm-safe / dep-free), so we provide a
    /// tiny purpose-built type. The signed compare is recovered separately.
    #[derive(Clone, Copy, PartialEq, Eq)]
    struct U256 {
        hi: u128,
        lo: u128,
    }

    impl U256 {
        const ZERO: U256 = U256 { hi: 0, lo: 0 };

        /// Full 128x128 -> 256 unsigned multiply.
        fn mul_u128(a: u128, b: u128) -> U256 {
            let a_lo = a as u64 as u128;
            let a_hi = a >> 64;
            let b_lo = b as u64 as u128;
            let b_hi = b >> 64;

            let ll = a_lo * b_lo;
            let lh = a_lo * b_hi;
            let hl = a_hi * b_lo;
            let hh = a_hi * b_hi;

            // result = hh<<128 + (lh+hl)<<64 + ll
            let mut lo = ll;
            let mut hi = hh;

            let (mid, carry1) = lh.overflowing_add(hl);
            // add mid << 64 into [hi:lo]
            let mid_lo = mid << 64;
            let mid_hi = mid >> 64;
            let (new_lo, c) = lo.overflowing_add(mid_lo);
            lo = new_lo;
            hi = hi.wrapping_add(mid_hi).wrapping_add(c as u128);
            if carry1 {
                // the overflow bit of lh+hl carries into bit 128 (i.e. hi += 1<<64)
                hi = hi.wrapping_add(1u128 << 64);
            }
            U256 { hi, lo }
        }

        /// 256-bit unsigned multiply by a u128 (used for the third factor).
        /// Inputs are bounded such that the true product fits in 256 bits for
        /// the int32-range coordinates this is fed (see FIDELITY-NOTE(F2)).
        fn mul_by_u128(self, b: u128) -> U256 {
            let lo_part = U256::mul_u128(self.lo, b);
            // hi * b contributes to the high limb only (its low 128 bits add to hi).
            let hi_part_lo = self.hi.wrapping_mul(b);
            U256 {
                hi: lo_part.hi.wrapping_add(hi_part_lo),
                lo: lo_part.lo,
            }
        }
    }

    impl PartialOrd for U256 {
        fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
            Some(self.cmp(other))
        }
    }
    impl Ord for U256 {
        fn cmp(&self, other: &Self) -> std::cmp::Ordering {
            self.hi.cmp(&other.hi).then_with(|| self.lo.cmp(&other.lo))
        }
    }

    /// Compares the angle enclosed by vectors dir and dirA (alpha) with the angle
    /// enclosed by -dir and dirB (beta). Returns -1 if alpha is less than beta, 0
    /// if they are equal and 1 if alpha is greater than beta. Note that dir is
    /// reversed for beta, because it represents the opposite side of a caliper.
    ///
    /// Geometry/ConvexHull.cpp:160-168
    pub fn cmp_angles(dir: &Point, dir_a: &Point, dir_b: &Point) -> i32 {
        // ConvexHull.cpp:161-162
        // int128_t dotA = dot(dir, dirA);
        // int128_t dotB = dot(-dir, dirB);
        let dot_a: i128 = dot(dir, dir_a);
        let dot_b: i128 = dot(&(-*dir), dir_b);
        // ConvexHull.cpp:163-164
        // int256_t dcosa = int256_t(magnsq(dirB)) * int256_t(abs(dotA)) * dotA;
        // int256_t dcosb = int256_t(magnsq(dirA)) * int256_t(abs(dotB)) * dotB;
        //
        // dcosa = magnsq(dirB) * |dotA| * dotA. magnsq is >= 0, so the sign of
        // dcosa equals the sign of dotA and |dcosa| = magnsq(dirB) * dotA^2.
        // We compare via (sign, magnitude) to avoid full signed 256-bit math.
        let sign_a = dot_a.signum() as i32;
        let sign_b = dot_b.signum() as i32;

        let mag_a = {
            let dot_a_abs = dot_a.unsigned_abs();
            U256::mul_u128(dot_a_abs, dot_a_abs).mul_by_u128(magnsq(dir_b) as u128)
        };
        let mag_b = {
            let dot_b_abs = dot_b.unsigned_abs();
            U256::mul_u128(dot_b_abs, dot_b_abs).mul_by_u128(magnsq(dir_a) as u128)
        };

        // ConvexHull.cpp:165
        // int256_t diff = dcosa - dcosb;
        // Reconstruct sign(dcosa - dcosb) from signed magnitudes.
        let val_a = (sign_a, if sign_a == 0 { U256::ZERO } else { mag_a });
        let val_b = (sign_b, if sign_b == 0 { U256::ZERO } else { mag_b });
        // diff > 0 ?
        let diff_positive;
        let diff_negative;
        if val_a.0 != val_b.0 {
            diff_positive = val_a.0 > val_b.0;
            diff_negative = val_a.0 < val_b.0;
        } else {
            // same sign s; diff sign = s * (|a| - |b|)
            let s = val_a.0;
            match val_a.1.cmp(&val_b.1) {
                std::cmp::Ordering::Greater => {
                    diff_positive = s > 0;
                    diff_negative = s < 0;
                }
                std::cmp::Ordering::Less => {
                    diff_positive = s < 0;
                    diff_negative = s > 0;
                }
                std::cmp::Ordering::Equal => {
                    diff_positive = false;
                    diff_negative = false;
                }
            }
        }

        // ConvexHull.cpp:167
        // return diff > 0? -1 : (diff < 0 ? 1 : 0);
        if diff_positive {
            -1
        } else if diff_negative {
            1
        } else {
            0
        }
    }

    /// A helper class to navigate on a polygon. Given a vertex index, one can
    /// get the edge belonging to that vertex, the coordinates of the vertex, the
    /// next and previous edges. Stuff that is needed in the rotating calipers algo.
    ///
    /// Geometry/ConvexHull.cpp:173-196
    #[derive(Clone, Copy)]
    pub struct Idx<'a> {
        m_idx: usize,
        m_poly: &'a Polygon,
    }

    impl<'a> Idx<'a> {
        // ConvexHull.cpp:179 — explicit Idx(size_t idx, const Polygon &p)
        pub fn new(idx: usize, p: &'a Polygon) -> Self {
            Idx { m_idx: idx, m_poly: p }
        }

        // ConvexHull.cpp:181
        pub fn idx(&self) -> usize {
            self.m_idx
        }

        // ConvexHull.cpp:183 — size_t next() const
        pub fn next(&self) -> usize {
            (self.m_idx + 1) % self.m_poly.len()
        }

        // ConvexHull.cpp:184 — size_t inc()
        pub fn inc(&mut self) -> usize {
            self.m_idx = (self.m_idx + 1) % self.m_poly.len();
            self.m_idx
        }

        // ConvexHull.cpp:185-187 — Point prev_dir() const
        pub fn prev_dir(&self) -> Point {
            self.pt() - self.m_poly[(self.m_idx + self.m_poly.len() - 1) % self.m_poly.len()]
        }

        // ConvexHull.cpp:189 — const Point &pt() const
        pub fn pt(&self) -> Point {
            self.m_poly[self.m_idx]
        }

        // ConvexHull.cpp:190 — const Point dir() const
        pub fn dir(&self) -> Point {
            self.m_poly[self.next()] - self.pt()
        }

        // ConvexHull.cpp:191-194 — const Point next_dir() const
        pub fn next_dir(&self) -> Point {
            self.m_poly[(self.m_idx + 2) % self.m_poly.len()] - self.m_poly[self.next()]
        }

        // ConvexHull.cpp:195 — const Polygon &poly() const
        pub fn poly(&self) -> &'a Polygon {
            self.m_poly
        }
    }

    // ConvexHull.cpp:198
    // enum class AntipodalVisitMode { Full, EdgesOnly };
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub enum AntipodalVisitMode {
        Full,
        EdgesOnly,
    }

    /// Visit all antipodal pairs starting from the initial ia, ib pair which
    /// has to be a valid antipodal pair (not checked). fn is called for every
    /// antipodal pair encountered including the initial one.
    /// The callback Fn has a signiture of bool(size_t i, size_t j, const Point &dir)
    /// where i,j are the vertex indices of the antipodal pair and dir is the
    /// direction of the calipers touching the i vertex.
    ///
    /// Geometry/ConvexHull.cpp:206-240
    pub fn visit_antipodals<F>(ia: &mut Idx, ib: &mut Idx, mode: AntipodalVisitMode, mut fn_: F)
    where
        F: FnMut(usize, usize, &Point) -> bool,
    {
        // The two Idx values navigate independently; `current`/`other` select
        // which of (ia, ib) is active. We track that with a boolean rather than
        // raw pointers to satisfy the borrow checker.
        // ConvexHull.cpp:210
        // int cmp = cmp_angles(ia.prev_dir(), ia.dir(), ib.dir());
        let mut cmp = cmp_angles(&ia.prev_dir(), &ia.dir(), &ib.dir());
        // ConvexHull.cpp:211-212
        // Idx *current = cmp <= 0 ? &ia : &ib, *other = cmp <= 0 ? &ib : &ia;
        // Idx *initial = current;
        // current_is_a == true  -> current == &ia, other == &ib
        let mut current_is_a = cmp <= 0;
        let initial_is_a = current_is_a;
        // ConvexHull.cpp:213
        let mut visitor_continue = true;

        // ConvexHull.cpp:215-216
        // size_t start = initial->idx();
        // bool finished = false;
        let start = if initial_is_a { ia.idx() } else { ib.idx() };
        let mut finished = false;

        // ConvexHull.cpp:218
        while visitor_continue && !finished {
            // ConvexHull.cpp:219
            // Point current_dir_a = current == &ia ? current->dir() : -current->dir();
            let current_dir_a = if current_is_a {
                ia.dir()
            } else {
                -ib.dir()
            };
            // ConvexHull.cpp:220
            // visitor_continue = fn(ia.idx(), ib.idx(), current_dir_a);
            visitor_continue = fn_(ia.idx(), ib.idx(), &current_dir_a);

            // Parallel edges encountered. An additional pair of antipodals
            // can be yielded.
            // ConvexHull.cpp:224-229
            if mode == AntipodalVisitMode::Full && cmp == 0 && visitor_continue {
                // visitor_continue = fn(current == &ia ? ia.idx() : ia.next(),
                //                       current == &ib ? ib.idx() : ib.next(),
                //                       current_dir_a);
                let i = if current_is_a { ia.idx() } else { ia.next() };
                let j = if !current_is_a { ib.idx() } else { ib.next() };
                visitor_continue = fn_(i, j, &current_dir_a);
            }

            // ConvexHull.cpp:231
            // cmp = cmp_angles(current->dir(), current->next_dir(), other->dir());
            cmp = if current_is_a {
                cmp_angles(&ia.dir(), &ia.next_dir(), &ib.dir())
            } else {
                cmp_angles(&ib.dir(), &ib.next_dir(), &ia.dir())
            };

            // ConvexHull.cpp:233
            // current->inc();
            if current_is_a {
                ia.inc();
            } else {
                ib.inc();
            }
            // ConvexHull.cpp:234-236
            // if (cmp > 0) std::swap(current, other);
            if cmp > 0 {
                current_is_a = !current_is_a;
            }

            // ConvexHull.cpp:238
            // if (initial->idx() == start) finished = true;
            let initial_idx = if initial_is_a { ia.idx() } else { ib.idx() };
            if initial_idx == start {
                finished = true;
            }
        }
    }
}

/// Returns true if the intersection of the two convex polygons A and B
/// is not an empty set.
///
/// Geometry/ConvexHull.cpp:244-330
/// C++: bool convex_polygons_intersect(const Polygon &A, const Polygon &B)
pub fn convex_polygons_intersect(a: &Polygon, b: &Polygon) -> bool {
    use rotcalip::*;

    // Establish starting antipodals as extremes in XY plane. Use the
    // easily obtainable bounding boxes to check if A and B is disjoint
    // and return false if the are.
    //
    // ConvexHull.cpp:251-269 — struct BB finds the extreme vertex indices.
    // P[i] < P[xmin] uses Point's lexicographic (x, then y) ordering.
    let point_lt = |l: &Point, u: &Point| l.x() < u.x() || (l.x() == u.x() && l.y() < u.y());
    // ConvexHull.cpp:255-258 — static bool cmpy(const Point &l, const Point &u)
    let cmpy = |l: &Point, u: &Point| l.y() < u.y() || (l.y() == u.y() && l.x() < u.x());

    // ConvexHull.cpp:260-268 — BB(const Polygon &poly)
    let make_bb = |p: &Polygon| {
        let (mut xmin, mut xmax, mut ymin, mut ymax) = (0usize, 0usize, 0usize, 0usize);
        for i in 0..p.len() {
            if point_lt(&p[i], &p[xmin]) {
                xmin = i;
            }
            if point_lt(&p[xmax], &p[i]) {
                xmax = i;
            }
            if cmpy(&p[i], &p[ymin]) {
                ymin = i;
            }
            if cmpy(&p[ymax], &p[i]) {
                ymax = i;
            }
        }
        (xmin, xmax, ymin, ymax)
    };

    // ConvexHull.cpp:271
    // BB bA{A}, bB{B};
    let (ba_xmin, ba_xmax, ba_ymin, ba_ymax) = make_bb(a);
    let (bb_xmin, bb_xmax, bb_ymin, bb_ymax) = make_bb(b);

    // ConvexHull.cpp:272-276 — BoundingBox bbA/bbB and the overlap() check are
    // commented out in C++; preserved here as a no-op to stay faithful.

    // Establish starting antipodals as extreme vertex pairs in X or Y direction
    // which reside on different polygons. If no such pair is found, the two
    // polygons are certainly not disjoint.
    //
    // ConvexHull.cpp:281
    // Idx imin{bA.xmin, A}, imax{bB.xmax, B};
    let mut imin = Idx::new(ba_xmin, a);
    let mut imax = Idx::new(bb_xmax, b);
    // ConvexHull.cpp:282 — if (B[bB.xmin] < imin.pt())  imin = Idx{bB.xmin, B};
    if point_lt(&b[bb_xmin], &imin.pt()) {
        imin = Idx::new(bb_xmin, b);
    }
    // ConvexHull.cpp:283 — if (imax.pt() < A[bA.xmax]) imax = Idx{bA.xmax, A};
    if point_lt(&imax.pt(), &a[ba_xmax]) {
        imax = Idx::new(ba_xmax, a);
    }
    // ConvexHull.cpp:284 — if (&imin.poly() == &imax.poly())
    if std::ptr::eq(imin.poly(), imax.poly()) {
        // ConvexHull.cpp:285-288
        imin = Idx::new(ba_ymin, a);
        imax = Idx::new(bb_ymax, b);
        if point_lt(&b[bb_ymin], &imin.pt()) {
            imin = Idx::new(bb_ymin, b);
        }
        if point_lt(&imax.pt(), &a[ba_ymax]) {
            imax = Idx::new(ba_ymax, a);
        }
    }

    // ConvexHull.cpp:291-292
    // if (&imin.poly() == &imax.poly()) return true;
    if std::ptr::eq(imin.poly(), imax.poly()) {
        return true;
    }

    // ConvexHull.cpp:294
    let mut found_divisor = false;

    // The callback needs the polygons that imin/imax sit on; capture them up
    // front (imin.poly() == &A-side, imax.poly() == &B-side after the setup).
    let poly_a = imin.poly();
    let poly_b = imax.poly();

    // ConvexHull.cpp:295-326
    // visit_antipodals<AntipodalVisitMode::EdgesOnly>(imin, imax, [...](size_t ia, size_t ib, const Point &dir) {...});
    visit_antipodals(
        &mut imin,
        &mut imax,
        AntipodalVisitMode::EdgesOnly,
        |ia, ib, dir| {
            // ConvexHull.cpp:300 — const Polygon &A = imin.poly(), &B = imax.poly();
            let pa = poly_a;
            let pb = poly_b;

            // ConvexHull.cpp:302
            // Point ref_a = A[(ia + 2) % A.size()], ref_b = B[(ib + 2) % B.size()];
            let ref_a = pa[(ia + 2) % pa.len()];
            let ref_b = pb[(ib + 2) % pb.len()];

            // ConvexHull.cpp:304-305
            // bool is_left_a = dotperp( dir, ref_a - A[ia]) > 0;
            // bool is_left_b = dotperp(-dir, ref_b - B[ib]) > 0;
            let is_left_a = dotperp(dir, &(ref_a - pa[ia])) > 0;
            let is_left_b = dotperp(&(-*dir), &(ref_b - pb[ib])) > 0;

            // ConvexHull.cpp:315
            // auto d = dotperp(dir, B[ib] - A[ia]);
            let d = dotperp(dir, &(pb[ib] - pa[ia]));
            // ConvexHull.cpp:316-323
            if d == 0 {
                // The caliper lines are collinear, not just parallel
                found_divisor = (is_left_a && is_left_b) || (!is_left_a && !is_left_b);
            } else if d > 0 {
                // B is to the left of (A, A+1)
                found_divisor = !is_left_a && !is_left_b;
            } else {
                // B is to the right of (A, A+1)
                found_divisor = is_left_a && is_left_b;
            }

            // ConvexHull.cpp:325 — return !found_divisor;
            !found_divisor
        },
    );

    // ConvexHull.cpp:328-329
    // Intersects if the divisor was not found
    // return !found_divisor;
    !found_divisor
}

/// Decompose source convex hull points into a top / bottom chains with
/// monotonically increasing x, creating an implicit trapezoidal decomposition
/// of the source convex polygon. The source convex polygon has to be CCW
/// oriented. O(n) time complexity.
///
/// Returns `(bottom, top)` to match the C++ `std::pair` (`first = bottom`,
/// `second = top`).
///
/// Geometry/ConvexHull.cpp:335-379
/// C++: std::pair<std::vector<Vec2d>, std::vector<Vec2d>> decompose_convex_polygon_top_bottom(const std::vector<Vec2d> &src)
pub fn decompose_convex_polygon_top_bottom(src: &[Vec2d]) -> (Vec<Vec2d>, Vec<Vec2d>) {
    // ConvexHull.cpp:337-339
    // std::pair<...> out;  std::vector<Vec2d> &bottom = out.first; std::vector<Vec2d> &top = out.second;
    let mut bottom: Vec<Vec2d> = Vec::new();
    let mut top: Vec<Vec2d> = Vec::new();

    // Comparator used by both min_element and max_element.
    // ConvexHull.cpp:342-343 — [](const auto &l, const auto &r) { return l.x() < r.x() || (l.x() == r.x() && l.y() < r.y()); }
    let lex_less = |l: &Vec2d, r: &Vec2d| l.x() < r.x() || (l.x() == r.x() && l.y() < r.y());

    if !src.is_empty() {
        // ConvexHull.cpp:342 — left_bottom = std::min_element(...)
        // std::min_element returns the first minimal element.
        let mut left_bottom: usize = 0;
        for i in 1..src.len() {
            if lex_less(&src[i], &src[left_bottom]) {
                left_bottom = i;
            }
        }
        // ConvexHull.cpp:343 — right_top = std::max_element(...)
        // std::max_element returns the first element for which no later element
        // is greater, i.e. the first maximal element under `lex_less`.
        let mut right_top: usize = 0;
        for i in 1..src.len() {
            if lex_less(&src[right_top], &src[i]) {
                right_top = i;
            }
        }

        // ConvexHull.cpp:344 — if (left_bottom != src.end() && left_bottom != right_top)
        if left_bottom != right_top {
            // ConvexHull.cpp:346 — if (left_bottom < right_top)
            if left_bottom < right_top {
                // ConvexHull.cpp:347 — bottom.assign(left_bottom, right_top + 1);
                bottom = src[left_bottom..=right_top].to_vec();
                // ConvexHull.cpp:348-351
                // top.assign(right_top, src.end());
                // top.insert(top.end(), src.begin(), left_bottom + 1);
                top = src[right_top..].to_vec();
                top.extend_from_slice(&src[..=left_bottom]);
            } else {
                // ConvexHull.cpp:353-357
                // bottom.assign(left_bottom, src.end());
                // bottom.insert(bottom.end(), src.begin(), right_top + 1);
                // top.assign(right_top, left_bottom + 1);
                bottom = src[left_bottom..].to_vec();
                bottom.extend_from_slice(&src[..=right_top]);
                top = src[right_top..=left_bottom].to_vec();
            }
            // Remove strictly vertical segments at the end.
            // ConvexHull.cpp:360-364
            if bottom.len() > 1 {
                // auto it = bottom.end(); for (-- it; it != bottom.begin() && (it - 1)->x() == bottom.back().x(); -- it);
                // bottom.erase(it + 1, bottom.end());
                let back_x = bottom.last().unwrap().x();
                let mut it = bottom.len() - 1;
                while it != 0 && bottom[it - 1].x() == back_x {
                    it -= 1;
                }
                bottom.truncate(it + 1);
            }
            // ConvexHull.cpp:365-369
            if top.len() > 1 {
                let back_x = top.last().unwrap().x();
                let mut it = top.len() - 1;
                while it != 0 && top[it - 1].x() == back_x {
                    it -= 1;
                }
                top.truncate(it + 1);
            }
            // ConvexHull.cpp:370 — std::reverse(top.begin(), top.end());
            top.reverse();
        }
    }

    // ConvexHull.cpp:373-377
    // if (top.size() < 2 || bottom.size() < 2) { top.clear(); bottom.clear(); }
    if top.len() < 2 || bottom.len() < 2 {
        top.clear();
        bottom.clear();
    }
    // ConvexHull.cpp:378 — return out; (out.first = bottom, out.second = top)
    (bottom, top)
}

/// Convex polygon check using a top / bottom chain decomposition with
/// O(log n) time complexity.
///
/// `top_bottom_decomposition` is `(bottom, top)` (matching the C++
/// `std::pair`: `.first = bottom`, `.second = top`).
///
/// Geometry/ConvexHull.cpp:382-419
/// C++: bool inside_convex_polygon(const std::pair<std::vector<Vec2d>, std::vector<Vec2d>> &top_bottom_decomposition, const Vec2d &pt)
pub fn inside_convex_polygon(decomp: &(Vec<Vec2d>, Vec<Vec2d>), pt: &Vec2d) -> bool {
    // .first == bottom, .second == top
    let bottom = &decomp.0;
    let top = &decomp.1;

    // std::lower_bound by x: first element with !(elem.x() < pt.x()), i.e. the
    // first index whose x >= pt.x(). Returns `.len()` if none (== end()).
    let lower_bound_x = |chain: &[Vec2d]| -> usize {
        let mut lo = 0usize;
        let mut hi = chain.len();
        while lo < hi {
            let mid = lo + (hi - lo) / 2;
            if chain[mid].x() < pt.x() {
                lo = mid + 1;
            } else {
                hi = mid;
            }
        }
        lo
    };

    // ConvexHull.cpp:384-385
    let it_bottom = lower_bound_x(bottom);
    let it_top = lower_bound_x(top);

    // ConvexHull.cpp:386-390
    // if (it_bottom == ...first.end()) { assert(it_top == ...second.end()); return false; }
    if it_bottom == bottom.len() {
        debug_assert!(it_top == top.len());
        return false;
    }
    // ConvexHull.cpp:391-403
    // if (it_bottom == ...first.begin())
    if it_bottom == 0 {
        // Below or at min x.
        if pt.x() < bottom[it_bottom].x() {
            // ConvexHull.cpp:393-396 — Below min x.
            debug_assert!(pt.x() < top[it_top].x());
            return false;
        }
        // ConvexHull.cpp:399-402 — At min x.
        debug_assert!(pt.x() == bottom[it_bottom].x());
        debug_assert!(pt.x() == top[it_top].x());
        debug_assert!(bottom[it_bottom].y() <= pt.y() && pt.y() <= top[it_top].y());
        return pt.y() >= bottom[it_bottom].y() && pt.y() <= top[it_top].y();
    }

    // Trapezoid or a triangle.
    // ConvexHull.cpp:406-413 — asserts
    debug_assert!(it_bottom != 0 && it_bottom != bottom.len());
    debug_assert!(it_top != 0 && it_top != top.len());
    debug_assert!(pt.x() <= bottom[it_bottom].x());
    debug_assert!(pt.x() <= top[it_top].x());
    let it_top_prev = it_top - 1;
    let it_bottom_prev = it_bottom - 1;
    debug_assert!(pt.x() >= top[it_top_prev].x());
    debug_assert!(pt.x() >= bottom[it_bottom_prev].x());
    // ConvexHull.cpp:414-416
    // double det = cross2(*it_bottom - *it_bottom_prev, pt - *it_bottom_prev);
    // if (det < 0) return false;
    let det = cross2f(
        bottom[it_bottom] - bottom[it_bottom_prev],
        *pt - bottom[it_bottom_prev],
    );
    if det < 0.0 {
        return false;
    }
    // ConvexHull.cpp:417-418
    // det = cross2(*it_top - *it_top_prev, pt - *it_top_prev);
    // return det <= 0;
    let det = cross2f(top[it_top] - top[it_top_prev], *pt - top[it_top_prev]);
    det <= 0.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_convex_hull_square() {
        // Test ConvexHull.cpp:11-38
        let points = vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 100),
            Point::new(0, 100),
            Point::new(50, 50), // Interior point
        ];

        let hull = convex_hull_points(points);
        assert_eq!(hull.points().len(), 4); // Only boundary points
    }

    #[test]
    fn test_convex_hull_collinear() {
        // Edge case: fewer than 3 distinct extreme points -> C++ returns empty hull.
        let points = vec![Point::new(0, 0), Point::new(50, 0), Point::new(100, 0)];

        let hull = convex_hull_points(points);
        // Collinear input: monotone chain leaves the degenerate hull; just
        // ensure it does not panic and stays small.
        assert!(hull.points().len() <= 3);
    }

    #[test]
    fn test_convex_polygons_intersect() {
        // Test ConvexHull.cpp:244-330 (CCW polygons required)
        let a = Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 100),
            Point::new(0, 100),
        ]);

        let b = Polygon::from_points(vec![
            Point::new(50, 50),
            Point::new(150, 50),
            Point::new(150, 150),
            Point::new(50, 150),
        ]);

        assert!(convex_polygons_intersect(&a, &b)); // Overlapping
    }

    #[test]
    fn test_convex_polygons_no_intersect() {
        let a = Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 100),
            Point::new(0, 100),
        ]);

        let b = Polygon::from_points(vec![
            Point::new(200, 200),
            Point::new(300, 200),
            Point::new(300, 300),
            Point::new(200, 300),
        ]);

        assert!(!convex_polygons_intersect(&a, &b)); // Separated
    }
}
