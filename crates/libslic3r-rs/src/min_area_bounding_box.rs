//! Minimum-area bounding box for polygons.
//!
//! C++ Reference:
//! - MinAreaBoundingBox.hpp
//! - MinAreaBoundingBox.cpp
//!
//! This is a faithful, line-by-line port of `MinAreaBoundingBox.cpp`. The
//! original implementation is a thin wrapper over the libnest2d rotating
//! calipers algorithm (`libnest2d/utils/rotcalipers.hpp`) and the libnest2d
//! libslic3r backend convex hull (`libnest2d/geometry_traits.hpp`). Both of
//! those headers are inlined here (in the `libnest2d` submodule below) because
//! byte-exact G-code parity depends on the exact hull / calipers arithmetic,
//! which differs from the crate's own `geometry::convex_hull`.
//!
//! Type mapping: `coord_t` -> `i64`, `coordf_t` -> `f64`, `long double` -> `f64`.
//! The libnest2d compute type `Unit = int64_t` and the rational `Ratio` type
//! `boost::rational<int128_t>` are reproduced with `i64` and an exact
//! `i128` rational (`Rational` struct below).

use crate::geometry::{ExPolygon, Point, Points, Polygon};

// MinAreaBoundingBox.cpp:20  namespace Slic3r {

// MinAreaBoundingBox.cpp:23  Used as compute type.
//   using Unit = int64_t;
type Unit = i64;

// MinAreaBoundingBox.cpp:25-29
//   using Rational = boost::rational<boost::multiprecision::int128_t>; (Apple)
// Reproduced as an exact 128-bit rational. Only the operations used by the
// rotating-calipers code are implemented: construction from a `Unit`,
// division by a `Unit`, multiplication by a `Unit`, and ordering. boost's
// rational keeps the value in lowest terms with a positive denominator and
// compares exactly; we replicate the exact comparison via cross multiplication
// (the normalization to lowest terms does not affect comparisons, so it is
// omitted, but the denominator sign is normalized to keep cross-multiplied
// comparisons correct).
#[derive(Clone, Copy, Debug)]
struct Rational {
    num: i128,
    den: i128,
}

impl Rational {
    #[inline]
    fn from_int(v: i128) -> Self {
        Rational { num: v, den: 1 }
    }

    // boost::rational<T>(n) / d  -- the rotcalipers code builds rationals as
    //   Ratio(dot) / magnsq(...)
    // which is `Rational::from_int(num) / den`. boost keeps the denominator
    // positive, so we normalize the sign here.
    #[inline]
    fn div_int(self, d: i128) -> Self {
        let mut num = self.num;
        let mut den = self.den * d;
        if den < 0 {
            num = -num;
            den = -den;
        }
        Rational { num, den }
    }

    // m = m * b  -- multiply a rational by an integer factor.
    #[inline]
    fn mul_int(self, b: i128) -> Self {
        Rational {
            num: self.num * b,
            den: self.den,
        }
    }
}

impl PartialEq for Rational {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        // Cross multiplication; denominators are normalized positive.
        self.num as i128 * other.den == other.num as i128 * self.den
    }
}

impl PartialOrd for Rational {
    #[inline]
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        // Denominators are normalized positive (see div_int / from_int), so
        // ordering by cross multiplication is exact.
        let lhs = self.num * other.den;
        let rhs = other.num * self.den;
        lhs.partial_cmp(&rhs)
    }
}

/// The convexity level of the input polygon.
///
/// MinAreaBoundingBox.hpp:24-26
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PolygonLevel {
    // MinAreaBoundingBox.hpp:25  pcConvex
    PcConvex,
    // MinAreaBoundingBox.hpp:25  pcSimple
    PcSimple,
}

// MinAreaBoundingBox.cpp:31-42
//   template<class P>
//   libnest2d::RotatedBox<Point, Unit> minAreaBoundigBox_(
//       const P &p, MinAreaBoundigBox::PolygonLevel lvl)
//
// The original is templated over the input geometry. Here the three concrete
// entry points (Polygon, ExPolygon, Points) all feed their *contour* point
// sequence into the same routine, because the libnest2d convex hull, collinear
// removal and rotating calipers all operate on the contour vertices only.
fn min_area_boundig_box_(contour: &[Point], lvl: PolygonLevel) -> libnest2d::RotatedBox {
    // MinAreaBoundingBox.cpp:35-37
    //   P chull = lvl == pcConvex ? p : libnest2d::sl::convexHull(p);
    let mut chull: Vec<Point> = if lvl == PolygonLevel::PcConvex {
        contour.to_vec()
    } else {
        libnest2d::convex_hull_path(contour)
    };

    // MinAreaBoundingBox.cpp:39
    //   libnest2d::removeCollinearPoints(chull);
    chull = libnest2d::remove_collinear_points(&chull, 0);

    // MinAreaBoundingBox.cpp:41
    //   return libnest2d::minAreaBoundingBox<P, Unit, Rational>(chull);
    libnest2d::min_area_bounding_box(&chull)
}

/// A class that holds a rotated bounding box. If instantiated with a polygon
/// type it will hold the minimum area bounding box for the given polygon.
/// If the input polygon is convex, the complexity is linear to the number of
/// points. Otherwise a convex hull of O(n*log(n)) has to be performed.
///
/// MinAreaBoundingBox.hpp:18-51
#[derive(Debug, Clone)]
pub struct MinAreaBoundigBox {
    // MinAreaBoundingBox.hpp:19  Point m_axis;
    m_axis: Point,
    // MinAreaBoundingBox.hpp:20  long double m_bottom = 0.0l, m_right = 0.0l;
    m_bottom: f64,
    m_right: f64,
}

impl MinAreaBoundigBox {
    // MinAreaBoundingBox.cpp:44-51
    //   MinAreaBoundigBox::MinAreaBoundigBox(const Polygon &p, PolygonLevel pc)
    pub fn from_polygon(p: &Polygon, pc: PolygonLevel) -> Self {
        // MinAreaBoundingBox.cpp:46
        let box_ = min_area_boundig_box_(&p.points, pc);

        // MinAreaBoundingBox.cpp:48-50
        Self {
            m_right: box_.right_extent() as f64,
            m_bottom: box_.bottom_extent() as f64,
            m_axis: box_.axis(),
        }
    }

    // MinAreaBoundingBox.cpp:53-60
    //   MinAreaBoundigBox::MinAreaBoundigBox(const ExPolygon &p, PolygonLevel pc)
    pub fn from_expolygon(p: &ExPolygon, pc: PolygonLevel) -> Self {
        // MinAreaBoundingBox.cpp:55
        let box_ = min_area_boundig_box_(&p.contour.points, pc);

        // MinAreaBoundingBox.cpp:57-59
        Self {
            m_right: box_.right_extent() as f64,
            m_bottom: box_.bottom_extent() as f64,
            m_axis: box_.axis(),
        }
    }

    // MinAreaBoundingBox.cpp:62-69
    //   MinAreaBoundigBox::MinAreaBoundigBox(const Points &pts, PolygonLevel pc)
    pub fn from_points(pts: &Points, pc: PolygonLevel) -> Self {
        // MinAreaBoundingBox.cpp:64
        let box_ = min_area_boundig_box_(pts, pc);

        // MinAreaBoundingBox.cpp:66-68
        Self {
            m_right: box_.right_extent() as f64,
            m_bottom: box_.bottom_extent() as f64,
            m_axis: box_.axis(),
        }
    }

    // MinAreaBoundingBox.cpp:71-77
    //   double MinAreaBoundigBox::angle_to_X() const
    pub fn angle_to_x(&self) -> f64 {
        // MinAreaBoundingBox.cpp:73
        let mut ret = (self.m_axis.y() as f64).atan2(self.m_axis.x() as f64);
        // MinAreaBoundingBox.cpp:74  auto s = std::signbit(ret);
        let s = ret.is_sign_negative();
        // MinAreaBoundingBox.cpp:75  if (s) ret += 2 * PI;
        if s {
            ret += 2.0 * std::f64::consts::PI;
        }
        // MinAreaBoundingBox.cpp:76  return -ret;
        -ret
    }

    // MinAreaBoundingBox.cpp:79-83
    //   long double MinAreaBoundigBox::width() const
    pub fn width(&self) -> f64 {
        // MinAreaBoundingBox.cpp:81-82
        self.m_bottom.abs() / (libnest2d::magnsq_f64(self.m_axis)).sqrt()
    }

    // MinAreaBoundingBox.cpp:85-89
    //   long double MinAreaBoundigBox::height() const
    pub fn height(&self) -> f64 {
        // MinAreaBoundingBox.cpp:87-88
        self.m_right.abs() / (libnest2d::magnsq_f64(self.m_axis)).sqrt()
    }

    // MinAreaBoundingBox.cpp:91-95
    //   long double MinAreaBoundigBox::area() const
    pub fn area(&self) -> f64 {
        // MinAreaBoundingBox.cpp:93  long double asq = magnsq<Point, long double>(m_axis);
        let asq = libnest2d::magnsq_f64(self.m_axis);
        // MinAreaBoundingBox.cpp:94  return m_bottom * m_right / asq;
        self.m_bottom * self.m_right / asq
    }

    // MinAreaBoundingBox.hpp:50
    //   const Point& axis() const { return m_axis; }
    pub fn axis(&self) -> &Point {
        &self.m_axis
    }
}

// MinAreaBoundingBox.cpp:97-100
//   void remove_collinear_points(Polygon &p)
//   { p = libnest2d::removeCollinearPoints<Polygon>(p, Unit(0)); }
pub fn remove_collinear_points_polygon(p: &mut Polygon) {
    p.points = libnest2d::remove_collinear_points(&p.points, 0);
}

// MinAreaBoundingBox.cpp:102-105
//   void remove_collinear_points(ExPolygon &p)
//   { p = libnest2d::removeCollinearPoints<ExPolygon>(p, Unit(0)); }
//
// `removeCollinearPoints<ExPolygon>` returns an ExPolygon whose *contour* has
// the collinear points removed; the holes are not produced by `addVertex`
// (which appends to the contour), so the result has the cleaned contour and no
// holes -- matching `create<ExPolygon>(cleaned_contour)`. We reproduce that
// here: the contour is cleaned and the holes are dropped.
pub fn remove_collinear_points_expolygon(p: &mut ExPolygon) {
    p.contour.points = libnest2d::remove_collinear_points(&p.contour.points, 0);
    p.holes.clear();
}

// MinAreaBoundingBox.cpp:106  } // namespace Slic3r

/// Faithful inlining of the pieces of libnest2d used by
/// `MinAreaBoundingBox.cpp`: `RotatedBox`, `removeCollinearPoints`, the
/// libslic3r-backend `convexHull` (PathTag) and `minAreaBoundingBox`.
///
/// All point arithmetic uses the libnest2d `pointlike` (`pl`) helpers, which
/// operate on `Unit = i64`. `is_clockwise` is `false` for the libslic3r
/// backend (Polygon/Points/ExPolygon are COUNTER_CLOCKWISE), so the convex
/// hull builds the counter-clockwise branch.
mod libnest2d {
    use super::{Rational, Unit};
    use crate::geometry::Point;

    // geometry_traits.hpp:310-318  pointlike::x / y
    #[inline]
    fn get_x(p: &Point) -> Unit {
        p.x()
    }
    #[inline]
    fn get_y(p: &Point) -> Unit {
        p.y()
    }

    // geometry_traits.hpp:348-351  create perpendicular vector
    //   template<class Pt> inline Pt perp(const Pt& p) { return Pt(y(p), -x(p)); }
    #[inline]
    fn perp(p: Point) -> Point {
        Point::new(get_y(&p), -get_x(&p))
    }

    // geometry_traits.hpp:353-357  dotperp
    //   Unit(x(a)) * Unit(y(b)) - Unit(y(a)) * Unit(x(b))
    #[inline]
    fn dotperp(a: Point, b: Point) -> Unit {
        get_x(&a) * get_y(&b) - get_y(&a) * get_x(&b)
    }

    // geometry_traits.hpp:360-364  dot product
    //   Unit(x(a)) * x(b) + Unit(y(a)) * y(b)
    #[inline]
    fn dot(a: Point, b: Point) -> Unit {
        get_x(&a) * get_x(&b) + get_y(&a) * get_y(&b)
    }

    // geometry_traits.hpp:367-371  squared vector magnitude (Unit)
    //   Unit(x(p)) * x(p) + Unit(y(p)) * y(p)
    #[inline]
    fn magnsq(p: Point) -> Unit {
        get_x(&p) * get_x(&p) + get_y(&p) * get_y(&p)
    }

    // magnsq<Point, long double> -- used by RotatedBox::area/width/height and
    // by MinAreaBoundigBox::width/height/area. Computed in f64 (long double).
    #[inline]
    pub fn magnsq_f64(p: Point) -> f64 {
        let x = get_x(&p) as f64;
        let y = get_y(&p) as f64;
        x * x + y * y
    }

    // rotcalipers.hpp:13-47  template<class Pt, class Unit> class RotatedBox
    //   For our use Pt = Point, Unit = i64.
    #[derive(Clone, Copy, Debug)]
    pub struct RotatedBox {
        // rotcalipers.hpp:14  Pt axis_;
        axis_: Point,
        // rotcalipers.hpp:15  Unit bottom_ = Unit(0), right_ = Unit(0);
        bottom_: Unit,
        right_: Unit,
    }

    impl RotatedBox {
        // rotcalipers.hpp:18  RotatedBox() = default;
        #[inline]
        pub fn default() -> Self {
            RotatedBox {
                axis_: Point::new(0, 0),
                bottom_: 0,
                right_: 0,
            }
        }

        // rotcalipers.hpp:19-20  RotatedBox(const Pt& axis, Unit b, Unit r)
        #[inline]
        fn new(axis: Point, b: Unit, r: Unit) -> Self {
            RotatedBox {
                axis_: axis,
                bottom_: b,
                right_: r,
            }
        }

        // rotcalipers.hpp:35  inline Unit bottom_extent() const { return bottom_; }
        #[inline]
        pub fn bottom_extent(&self) -> Unit {
            self.bottom_
        }

        // rotcalipers.hpp:36  inline Unit right_extent() const { return right_; }
        #[inline]
        pub fn right_extent(&self) -> Unit {
            self.right_
        }

        // rotcalipers.hpp:37  inline const Pt& axis() const { return axis_; }
        #[inline]
        pub fn axis(&self) -> Point {
            self.axis_
        }
    }

    // rotcalipers.hpp:49-71  removeCollinearPoints
    //   template <class Poly, class Pt, class Unit>
    //   Poly removeCollinearPoints(const Poly& sh, Unit eps = Unit(0))
    //
    // `sh` is the contour point sequence. Returns the cleaned contour.
    pub fn remove_collinear_points(sh: &[Point], eps: Unit) -> Vec<Point> {
        // rotcalipers.hpp:52  Poly ret; sl::reserve(ret, sl::contourVertexCount(sh));
        let mut ret: Vec<Point> = Vec::with_capacity(sh.len());

        if sh.is_empty() {
            return ret;
        }

        // rotcalipers.hpp:54  Pt eprev = *sl::cbegin(sh) - *std::prev(sl::cend(sh));
        let mut eprev = sh[0] - sh[sh.len() - 1];

        // rotcalipers.hpp:56-57  auto it = cbegin(sh); auto itx = std::next(it);
        let mut it: usize = 0;
        let mut itx: usize = 1;

        // rotcalipers.hpp:58  if(itx != sl::cend(sh)) while (it != sl::cend(sh))
        if itx != sh.len() {
            while it != sh.len() {
                // rotcalipers.hpp:60  Pt enext = *itx - *it;
                let enext = sh[itx] - sh[it];

                // rotcalipers.hpp:62  auto dp = pl::dotperp<Pt, Unit>(eprev, enext);
                let dp = dotperp(eprev, enext);
                // rotcalipers.hpp:63  if(abs(dp) > eps) sl::addVertex(ret, *it);
                if dp.abs() > eps {
                    ret.push(sh[it]);
                }

                // rotcalipers.hpp:65  eprev = enext;
                eprev = enext;
                // rotcalipers.hpp:66  if (++itx == sl::cend(sh)) itx = sl::cbegin(sh);
                itx += 1;
                if itx == sh.len() {
                    itx = 0;
                }
                // rotcalipers.hpp:67  ++it;
                it += 1;
            }
        }

        // rotcalipers.hpp:70  return ret;
        ret
    }

    // rotcalipers.hpp:74-84  rectarea (axis + four support vertices)
    //   Not directly used by minAreaBoundingBox (which uses the a/b overload),
    //   but kept for completeness mirroring the header.
    //
    // rotcalipers.hpp:95-103  rectarea(w, a, b)
    //   R m = R(a) / pl::magnsq<Pt, Unit>(w);  m = m * b;  return m;
    fn rectarea_ab(w: Point, a: Unit, b: Unit) -> Rational {
        // R(a) / magnsq(w)
        let mut m = Rational::from_int(a as i128).div_int(magnsq(w) as i128);
        // m = m * b
        m = m.mul_int(b as i128);
        m
    }

    // rotcalipers.hpp:105-109  rectarea(const RotatedBox&)
    //   rectarea<Pt, Unit, R>(rb.axis(), rb.bottom_extent(), rb.right_extent());
    fn rectarea_box(rb: &RotatedBox) -> Rational {
        rectarea_ab(rb.axis(), rb.bottom_extent(), rb.right_extent())
    }

    // rotcalipers.hpp:113-272  rotcalipers
    //   Only applicable to counter-clockwise oriented convex polygons where only
    //   two points can be collinear with each other.
    //
    // `sh` is the contour point sequence. `visitfn` returns true to continue.
    fn rotcalipers<F: FnMut(&RotatedBox) -> bool>(sh: &[Point], mut visitfn: F) {
        // rotcalipers.hpp:123-125  first/last iterators (here indices)
        if sh.is_empty() {
            return;
        }
        // We operate on a working buffer `pts` and indices [first, last].
        // `first`/`last` are indices into `pts`. The cyclic helpers wrap on
        // [first, last] inclusive.
        let mut pts: Vec<Point> = sh.to_vec();
        let mut first: usize = 0;
        let mut last: usize = pts.len() - 1;

        // rotcalipers.hpp:128  if(last == first) return;
        if last == first {
            return;
        }
        // rotcalipers.hpp:129
        //   if(getX(*first) == getX(*last) && getY(*first) == getY(*last)) --last;
        if get_x(&pts[first]) == get_x(&pts[last]) && get_y(&pts[first]) == get_y(&pts[last]) {
            // last cannot already equal first here (checked above), so a plain
            // decrement matches the C++ `--last`.
            if last == 0 {
                return;
            }
            last -= 1;
        }
        // rotcalipers.hpp:130  if(last - first < 2) return;
        if last < first + 2 {
            return;
        }

        // rotcalipers.hpp:132-147  Orientation check + optional flip.
        {
            // rotcalipers.hpp:134  Point p = *first, q = *std::next(first), r = *last;
            let p = pts[first];
            let q = pts[first + 1];
            let r = pts[last];

            // rotcalipers.hpp:137-138  orientation determinant
            //   d = (Unit(y(q)) - y(p)) * (Unit(x(r)) - x(p))
            //     - (Unit(x(q)) - x(p)) * (Unit(y(r)) - y(p));
            let d = (get_y(&q) - get_y(&p)) * (get_x(&r) - get_x(&p))
                - (get_x(&q) - get_x(&p)) * (get_y(&r) - get_y(&p));

            // rotcalipers.hpp:140-146  if(d > 0) { flip into shcpy }
            if d > 0 {
                // The polygon is clockwise. A flip is needed (for now).
                let mut shcpy: Vec<Point> = Vec::with_capacity(last - first);
                // auto it = last; while(it != first) addVertex(shcpy, *it--);
                let mut iti = last;
                while iti != first {
                    shcpy.push(pts[iti]);
                    iti -= 1;
                }
                // addVertex(shcpy, *first);
                shcpy.push(pts[first]);
                // first = cbegin(shcpy); last = prev(cend(shcpy));
                pts = shcpy;
                first = 0;
                last = pts.len() - 1;
            }
        }

        // rotcalipers.hpp:150-152  Cyclic iterator increment
        //   if(it == last) it = first; else ++it;
        let inc = |it: &mut usize, first: usize, last: usize| {
            if *it == last {
                *it = first;
            } else {
                *it += 1;
            }
        };

        // rotcalipers.hpp:154-157  Cyclic previous iterator
        //   return it == first ? last : std::prev(it);
        let prev = |it: usize, first: usize, last: usize| -> usize {
            if it == first {
                last
            } else {
                it - 1
            }
        };

        // rotcalipers.hpp:159-162  Cyclic next iterator
        //   return it == last ? first : std::next(it);
        let next = |it: usize, first: usize, last: usize| -> usize {
            if it == last {
                first
            } else {
                it + 1
            }
        };

        // rotcalipers.hpp:167-184  Find polygon extremes.
        let mut itw = first;
        let mut min_x = itw;
        let mut max_x = itw;
        let mut min_y = itw;
        let mut max_y = itw;

        loop {
            // rotcalipers.hpp:172  Point v = *it, d = v - *minX;
            let v = pts[itw];

            let d = v - pts[min_x];
            // rotcalipers.hpp:173  if(getX(d)<0 || (getX(d)==0 && getY(d)<0)) minX = it;
            if get_x(&d) < 0 || (get_x(&d) == 0 && get_y(&d) < 0) {
                min_x = itw;
            }

            // rotcalipers.hpp:175-176
            let d = v - pts[max_x];
            if get_x(&d) > 0 || (get_x(&d) == 0 && get_y(&d) > 0) {
                max_x = itw;
            }

            // rotcalipers.hpp:178-179
            let d = v - pts[min_y];
            if get_y(&d) < 0 || (get_y(&d) == 0 && get_x(&d) > 0) {
                min_y = itw;
            }

            // rotcalipers.hpp:181-182
            let d = v - pts[max_y];
            if get_y(&d) > 0 || (get_y(&d) == 0 && get_x(&d) < 0) {
                max_y = itw;
            }

            // rotcalipers.hpp:184  } while(++it != std::next(last));
            itw += 1;
            if itw == last + 1 {
                break;
            }
        }

        // rotcalipers.hpp:189-238  update lambda.
        //   Updates the support vertices; the rectangle with the smallest
        //   rotation is selected, returning the supporting vertices in `rect`.
        let update = |w: Point, rect: &mut [usize; 4], first: usize, last: usize| -> bool {
            // rotcalipers.hpp:192-195
            let b_idx = rect[0];
            let bn = next(b_idx, first, last);
            let r_idx = rect[1];
            let rn = next(r_idx, first, last);
            let t_idx = rect[2];
            let tn = next(t_idx, first, last);
            let l_idx = rect[3];
            let ln = next(l_idx, first, last);

            // rotcalipers.hpp:197  Point b = *Bn - *B, r = ..., t = ..., l = ...;
            let b = pts[bn] - pts[b_idx];
            let r = pts[rn] - pts[r_idx];
            let t = pts[tn] - pts[t_idx];
            let l = pts[ln] - pts[l_idx];
            // rotcalipers.hpp:198  Point pw = perp(w);
            let pw = perp(w);

            // rotcalipers.hpp:201-202  dotted projections
            let dotwpb = dot(w, b);
            let dotwpr = dot(-pw, r);
            let dotwpt = dot(-w, t);
            let dotwpl = dot(pw, l);
            // rotcalipers.hpp:203  Unit dw = magnsq<Pt, Unit>(w);
            let dw = magnsq(w);

            // rotcalipers.hpp:205-209  angles array (Ratio)
            //   angles[i] = (Ratio(dotwp) / magnsq(edge)) * dotwp;
            let angles: [Rational; 4] = [
                Rational::from_int(dotwpb as i128)
                    .div_int(magnsq(b) as i128)
                    .mul_int(dotwpb as i128),
                Rational::from_int(dotwpr as i128)
                    .div_int(magnsq(r) as i128)
                    .mul_int(dotwpr as i128),
                Rational::from_int(dotwpt as i128)
                    .div_int(magnsq(t) as i128)
                    .mul_int(dotwpt as i128),
                Rational::from_int(dotwpl as i128)
                    .div_int(magnsq(l) as i128)
                    .mul_int(dotwpl as i128),
            ];

            // rotcalipers.hpp:211-226
            //   build a vector A of (angle, index) sorted by angle descending,
            //   keeping only edges where rect[i] != rect[j] and angles[i] < dw.
            //   Insertion uses std::lower_bound with comparator ai.first > aj.first.
            let dw_ratio = Rational::from_int(dw as i128);
            let mut a_vec: Vec<(Rational, usize)> = Vec::with_capacity(4);

            // for (size_t i = 3, j = 0; j < 4; i = j++)
            let mut i: usize = 3;
            let mut j: usize = 0;
            while j < 4 {
                // rotcalipers.hpp:215  if(rect[i] != rect[j] && angles[i] < dw)
                if rect[i] != rect[j] && angles[i] < dw_ratio {
                    let iv = (angles[i], i);
                    // std::lower_bound with comparator (ai.first > aj.first):
                    // first position `pos` where !(A[pos].first > iv.first),
                    // i.e. A[pos].first <= iv.first.
                    let mut pos = 0usize;
                    while pos < a_vec.len() && a_vec[pos].0 > iv.0 {
                        pos += 1;
                    }
                    a_vec.insert(pos, iv);
                }
                i = j;
                j += 1;
            }

            // rotcalipers.hpp:229  if(A.empty()) return false;
            if a_vec.is_empty() {
                return false;
            }

            // rotcalipers.hpp:231-233
            //   auto amin = A.front().first;  auto imin = A.front().second;
            //   for(auto& a : A) if(a.first == amin) inc(rect[a.second]);
            let amin = a_vec[0].0;
            let imin = a_vec[0].1;
            for a in &a_vec {
                if a.0 == amin {
                    let mut idx = rect[a.1];
                    inc(&mut idx, first, last);
                    rect[a.1] = idx;
                }
            }

            // rotcalipers.hpp:235  std::rotate(rect.begin(), rect.begin()+imin, rect.end());
            rect.rotate_left(imin);

            // rotcalipers.hpp:237  return true;
            true
        };

        // rotcalipers.hpp:240-241
        //   Point w(1, 0);
        //   std::array<Iterator, 4> rect = {minY, maxX, maxY, minX};
        let mut w = Point::new(1, 0);
        let mut rect: [usize; 4] = [min_y, max_x, max_y, min_x];

        // rotcalipers.hpp:243-248  initial box visit
        {
            // Unit a = dot(w, *rect[1] - *rect[3]);
            let a = dot(w, pts[rect[1]] - pts[rect[3]]);
            // Unit b = dot(-perp(w), *rect[2] - *rect[0]);
            let b = dot(-perp(w), pts[rect[2]] - pts[rect[0]]);
            // if (!visitfn(RotatedBox<Point, Unit>{w, a, b})) return;
            if !visitfn(&RotatedBox::new(w, a, b)) {
                return;
            }
        }

        // rotcalipers.hpp:251-252  edge mask + counter
        //   size_t c = 0, count = last - first + 1;
        //   std::vector<bool> edgemask(count, false);
        let mut c: usize = 0;
        let count: usize = last - first + 1;
        let mut edgemask: Vec<bool> = vec![false; count];

        // rotcalipers.hpp:254  while(c++ < count)
        while {
            let cond = c < count;
            c += 1;
            cond
        } {
            // rotcalipers.hpp:257  if(! update(w, rect)) break;
            if !update(w, &mut rect, first, last) {
                break;
            }

            // rotcalipers.hpp:259  size_t eidx = size_t(rect[0] - first);
            let eidx = rect[0] - first;

            // rotcalipers.hpp:261-262  if(edgemask[eidx]) break; edgemask[eidx]=true;
            if edgemask[eidx] {
                break;
            }
            edgemask[eidx] = true;

            // rotcalipers.hpp:265  w = *rect[0] - *prev(rect[0]);
            w = pts[rect[0]] - pts[prev(rect[0], first, last)];

            // rotcalipers.hpp:267-268
            let a = dot(w, pts[rect[1]] - pts[rect[3]]);
            let b = dot(-perp(w), pts[rect[2]] - pts[rect[0]]);
            // rotcalipers.hpp:269-270  if (!visitfn(RotatedBox<Point, Unit>{w, a, b})) break;
            if !visitfn(&RotatedBox::new(w, a, b)) {
                break;
            }
        }
    }

    // rotcalipers.hpp:276-296  minAreaBoundingBox
    //   template <class S, class Unit, class Ratio>
    //   RotatedBox<TPoint<S>, Unit> minAreaBoundingBox(const S& sh)
    pub fn min_area_bounding_box(sh: &[Point]) -> RotatedBox {
        // rotcalipers.hpp:281  RotatedBox<TPoint<S>, Unit> minbox;
        let mut minbox = RotatedBox::default();
        // rotcalipers.hpp:282  Ratio minarea = std::numeric_limits<Unit>::max();
        //   The initial value is the max of Unit (i64), held as a Ratio.
        let mut minarea = Rational::from_int(i64::MAX as i128);

        // rotcalipers.hpp:283-291  minfn
        let minfn = |rbox: &RotatedBox| -> bool {
            // rotcalipers.hpp:284  Ratio area = rectarea<Ratio>(rbox);
            let area = rectarea_box(rbox);
            // rotcalipers.hpp:285-288  if (area <= minarea) { minarea = area; minbox = rbox; }
            if area <= minarea {
                minarea = area;
                minbox = *rbox;
            }
            // rotcalipers.hpp:290  return true; // continue search
            true
        };

        // rotcalipers.hpp:293  rotcalipers<S, Unit, Ratio>(sh, minfn);
        rotcalipers(sh, minfn);

        // rotcalipers.hpp:295  return minbox;
        minbox
    }

    // geometry_traits.hpp:1048-1113  convexHull(const S& sh, const PathTag&)
    //   For the libslic3r backend, S is a path of Slic3r::Point and
    //   is_clockwise<S>() == false (COUNTER_CLOCKWISE), so the CCW branch runs.
    pub fn convex_hull_path(sh: &[Point]) -> Vec<Point> {
        // geometry_traits.hpp:1055  size_t edges = cend(sh) - cbegin(sh);
        let edges = sh.len();
        // geometry_traits.hpp:1056  if(edges < 3) return {};
        if edges < 3 {
            return Vec::new();
        }

        // geometry_traits.hpp:1058-1060
        let mut closed = false;
        let mut u: Vec<Point> = Vec::with_capacity(1 + edges / 2);
        let mut l: Vec<Point> = Vec::with_capacity(1 + edges / 2);

        // geometry_traits.hpp:1062-1063  copy sh into pts
        let mut pts: Vec<Point> = sh.to_vec();

        // geometry_traits.hpp:1065-1068  drop duplicated closing vertex
        let fpt = pts[0];
        let lpt = pts[pts.len() - 1];
        if get_x(&fpt) == get_x(&lpt) && get_y(&fpt) == get_y(&lpt) {
            closed = true;
            pts.pop();
        }

        // geometry_traits.hpp:1070-1075  sort lexicographically (x, then y)
        pts.sort_by(|v1, v2| {
            let x1 = get_x(v1);
            let x2 = get_x(v2);
            let y1 = get_y(v1);
            let y2 = get_y(v2);
            if x1 == x2 {
                y1.cmp(&y2)
            } else {
                x1.cmp(&x2)
            }
        });

        // geometry_traits.hpp:1077-1080  dir(p, q, r)
        let dir = |p: &Point, q: &Point, r: &Point| -> Unit {
            (get_y(q) - get_y(p)) * (get_x(r) - get_x(p))
                - (get_x(q) - get_x(p)) * (get_y(r) - get_y(p))
        };

        // geometry_traits.hpp:1082-1095  monotone chain
        let mut ik = 0usize;
        while ik != pts.len() {
            // while(U.size()>1 && dir(U[U.size()-2], U.back(), *ik) <= 0) U.pop_back();
            while u.len() > 1 && dir(&u[u.len() - 2], &u[u.len() - 1], &pts[ik]) <= 0 {
                u.pop();
            }
            // while(L.size()>1 && dir(L[L.size()-2], L.back(), *ik) >= 0) L.pop_back();
            while l.len() > 1 && dir(&l[l.len() - 2], &l[l.len() - 1], &pts[ik]) >= 0 {
                l.pop();
            }

            // U.emplace_back(*ik); L.emplace_back(*ik);
            u.push(pts[ik]);
            l.push(pts[ik]);

            // ++ik;
            ik += 1;
        }

        // geometry_traits.hpp:1097  S ret; reserve(ret, U.size() + L.size());
        let mut ret: Vec<Point> = Vec::with_capacity(u.len() + l.len());
        // geometry_traits.hpp:1098-1110  is_clockwise<S>() is false here -> else branch
        //   for(it = L.begin(); it != prev(L.end()); ++it) addVertex(ret, *it);
        for it in 0..l.len().saturating_sub(1) {
            ret.push(l[it]);
        }
        //   for(it = U.rbegin(); it != prev(U.rend()); ++it) addVertex(ret, *it);
        //   U.rbegin() walks U back-to-front; prev(U.rend()) stops before U.front().
        for it in (1..u.len()).rev() {
            ret.push(u[it]);
        }
        //   if(closed) addVertex(ret, *prev(U.rend()));  // == U.front()
        if closed {
            ret.push(u[0]);
        }

        // geometry_traits.hpp:1112  return ret;
        ret
    }
}
