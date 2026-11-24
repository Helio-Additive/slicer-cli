//! Faithful 1:1 port of `Arrange.cpp` (BambuStudio `src/libslic3r/Arrange.cpp`).
//!
//! C++ Reference:
//! - src/libslic3r/Arrange.cpp
//! - src/libslic3r/Arrange.hpp
//!
//! Type mapping: `coord_t` (`int32_t` at the pin, libslic3r.h:40) -> `Coord` (`i64`) with the
//! int32 arithmetic reproduced where it can wrap; `coordf_t`/`double` -> `f64`; `float` -> `f32`.
//! Floating expressions are contracted as clang (`-ffp-contract=on`) contracts them: `a*b + c` and
//! `a*b - c*d` become `fma(a, b, ...)` within one expression, never across statements or calls.
//!
//! The pieces of the header-only libnest2d library that the arrange helpers call are ported in the
//! inline module [`libnest2d`] below: `_Box`, `_Circle`, the libslic3r backend's shape functions
//! (`geometries.hpp`, the boost envelope bounding box, the default `area`, `isInside` for box and
//! circle hosts), the rotating calipers (`utils/rotcalipers.hpp`, with `boost::rational<int128>`
//! as [`libnest2d::Rational`]), the item class `_Item` (`nester.hpp`, the state and transforms the
//! helpers use) and the placer configuration `NfpPConfig` (`placers/nfpplacer.hpp`).
//!
//! NOT ported (and not stubbed): the nester itself and everything that drives it, i.e.
//! `AutoArranger<TBin>` (Arrange.cpp:286-801: `norm`, `dist_for_BOTTOM_LEFT`, `dist_to_bin`,
//! `objfunc`, `preload`, the three `get_objfn` specializations), `_arrange` (Arrange.cpp:916-1099)
//! and the `arrange` entry points (Arrange.cpp:1195-1236). They need libnest2d's `_Nester`, the
//! NFP placer `_NofitPolyPlacer` (placers/nfpplacer.hpp, 1425 lines, with the convex NFP over
//! `boost::rational<int128>` of geometry_traits_nfp.hpp and the nlopt subplex optimizer of
//! optimizers/nlopt), the first-fit selection (selections/firstfit.hpp), boost's R*-tree spatial
//! index (`bgi::rtree<SpatElement, bgi::rstar<16, 4>>`, whose query order feeds the floating sums
//! of `objfunc`) and `boost::geometry::within` for polygon bins. None of these exist in the crate;
//! `slicer_cli-rust` refuses `--arrange`, so nothing here is on the slicing path.

use std::rc::Rc;

use crate::config::{ConfigOption, ConfigOptionResolver, ConfigOptionType};
use crate::geometry::{BoundingBox, ExPolygon, Point, Points, Polygon};
use crate::libslic3r::{EPSILON, SCALING_FACTOR};
use crate::print_config::config_def::{get_bed_shape, DynamicPrintConfig};
use crate::Coord;

use self::libnest2d::{NfpPConfigAlignment, _Box, _Circle, _Item};

// Arrange.hpp:48-105 — the arrangement data types are ported in model_arrange.rs.
// Re-export them so `crate::arrange` mirrors the C++ `Slic3r::arrangement`
// namespace surface (CircleBed, InfiniteBed, UNARRANGED, ArrangePolygon, ...).
pub use crate::model_arrange::{ArrangePolygon, ArrangePolygons, CircleBed, InfiniteBed, UNARRANGED};

// ============================================================================
// Scaling helpers as the C++ spells them (libslic3r.h:81-82, Point.hpp:524-561)
// ============================================================================

/// libslic3r.h:81 `#define scale_(val) ((val) / SCALING_FACTOR)` — a double.
#[inline]
fn scale_(val: f64) -> f64 {
    val / SCALING_FACTOR
}

/// libslic3r.h:82 `#define unscale_(val) ((val) * SCALING_FACTOR)` — a double.
#[inline]
fn unscale_(val: f64) -> f64 {
    val * SCALING_FACTOR
}

/// Point.hpp:535-540 `scaled<coord_t>(const double &v)`: `coord_t(v / double(SCALING_FACTOR))`,
/// a truncation to int32.
#[inline]
fn scaled_f64(v: f64) -> Coord {
    (v / SCALING_FACTOR) as i32 as Coord
}

/// Point.hpp:535-540 `scaled<coord_t>(const float &v)`: `coord_t(v / float(SCALING_FACTOR))`, the
/// division in float, then a truncation to int32.
#[inline]
fn scaled_f32(v: f32) -> Coord {
    (v / SCALING_FACTOR as f32) as i32 as Coord
}

/// libslic3r.h:84 `#define SCALED_EPSILON scale_(EPSILON)`.
#[inline]
fn scaled_epsilon() -> f64 {
    scale_(EPSILON)
}

// ============================================================================
// Arrange.cpp:55-61 — unscaled(ClipperLib::IntPoint)
// ============================================================================

/// Arrange.cpp:55-61
/// `template<class Tout = double, ...> Eigen::Matrix<Tout, 2> unscaled(const ClipperLib::IntPoint &v)`:
/// `{unscaled<Tout>(v.x()), unscaled<Tout>(v.y())}` with `unscaled<double>(v) = double(v) * SCALING_FACTOR`
/// (Point.hpp:560). `ClipperLib::IntPoint` is `Slic3r::Point` in the libslic3r backend.
#[inline]
pub fn unscaled(v: &Point) -> crate::geometry::Vec2d {
    crate::geometry::Vec2d::new(v.x() as f64 * SCALING_FACTOR, v.y() as f64 * SCALING_FACTOR)
}

// ============================================================================
// libnest2d (header-only, src/libnest2d/include/libnest2d) — the parts the helpers use
// ============================================================================

/// The libnest2d pieces Arrange.cpp's helpers call, with `RawShape = Slic3r::ExPolygon`,
/// `Point = Slic3r::Point` (int32 coordinates), `TCompute<Point> = int64_t`
/// (geometry_traits.hpp:76) and `boost::rational<LargeInt>` (Arrange.cpp:25-32, `LargeInt` is
/// `boost::multiprecision::int128_t` on Apple, `__int128` elsewhere) as [`Rational`].
pub mod libnest2d {
    use std::cell::RefCell;
    use std::rc::Rc;

    use crate::clipper_utils::{offset_ex_expolygon_clib, OffsetJoinType};
    use crate::geometry::{ExPolygon, Point};
    use crate::libslic3r::EPSILON;
    use crate::Coord;

    /// common.hpp:99 `const double BP2D_CONSTEXPR Pi = 3.141592653589793238463;`
    pub const PI: f64 = 3.141592653589793238463;
    /// common.hpp:100 `const double BP2D_CONSTEXPR Pi_2 = 2*Pi;`
    pub const PI_2: f64 = 2.0 * PI;
    /// nester.hpp:12 `#define LARGE_COST_TO_REJECT 1e7`
    pub const LARGE_COST_TO_REJECT: f64 = 1e7;
    /// nester.hpp:19 `static const constexpr int BIN_ID_UNSET = -2;`
    pub const BIN_ID_UNSET: i32 = -2;

    /// nester.hpp:15 `#define MIN_SEPARATION scale_(1.0)`: a double, `1.0 / SCALING_FACTOR`.
    #[inline]
    pub fn min_separation() -> f64 {
        1.0 / crate::libslic3r::SCALING_FACTOR
    }

    // ------------------------------------------------------------------------
    // boost::rational<LargeInt>
    // ------------------------------------------------------------------------

    /// `boost::bad_rational`, thrown by `rational::operator/=` on a zero divisor.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct BadRational;

    impl std::fmt::Display for BadRational {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            // boost/rational.hpp: `bad_rational() : std::domain_error("bad rational: zero denominator")`
            f.write_str("bad rational: zero denominator")
        }
    }

    impl std::error::Error for BadRational {}

    /// `boost::rational<int128>`: kept normalized (lowest terms, positive denominator) by every
    /// operation, as boost does.
    #[derive(Debug, Clone, Copy)]
    pub struct Rational {
        num: i128,
        den: i128,
    }

    /// `boost::integer::gcd`: the non-negative greatest common divisor.
    #[inline]
    fn gcd_i128(a: i128, b: i128) -> i128 {
        let (mut a, mut b) = (a.abs(), b.abs());
        while b != 0 {
            let t = a % b;
            a = b;
            b = t;
        }
        a
    }

    impl Rational {
        /// `rational(const T& n)`: `num(n), den(1)`.
        #[inline]
        pub fn from_int(n: i128) -> Self {
            Rational { num: n, den: 1 }
        }

        /// boost/rational.hpp `rational& operator/= (const T& i)`.
        pub fn div_int(mut self, i: i128) -> Result<Self, BadRational> {
            // Avoid repeated construction
            let zero: i128 = 0;

            if i == zero {
                return Err(BadRational);
            }
            if self.num == zero {
                return Ok(self);
            }

            // Avoid overflow and preserve normalization
            let gcd = gcd_i128(self.num, i);
            self.num /= gcd;
            self.den *= i / gcd;

            if self.den < zero {
                self.num = -self.num;
                self.den = -self.den;
            }

            Ok(self)
        }

        /// boost/rational.hpp `rational& operator*= (const T& i)`.
        pub fn mul_int(mut self, i: i128) -> Self {
            // Avoid overflow and preserve normalization
            let gcd = gcd_i128(i, self.den);
            self.num *= i / gcd;
            self.den /= gcd;
            self
        }

        /// boost/rational.hpp `bool operator< (const rational& r) const`: the order by comparing the
        /// simple continued fractions of both values (no products, no overflow).
        pub fn lt(&self, r: &Rational) -> bool {
            // Avoid repeated construction
            let zero: i128 = 0;

            struct Cf {
                n: i128,
                d: i128,
                q: i128,
                r: i128,
            }
            let mut ts = Cf { n: self.num, d: self.den, q: self.num / self.den, r: self.num % self.den };
            let mut rs = Cf { n: r.num, d: r.den, q: r.num / r.den, r: r.num % r.den };
            let mut reverse: u32 = 0;

            // Normalize negative moduli by repeatedly adding the (positive) denominator
            // and decrementing the quotient.
            while ts.r < zero {
                ts.r += ts.d;
                ts.q -= 1;
            }
            while rs.r < zero {
                rs.r += rs.d;
                rs.q -= 1;
            }

            // Loop through and compare each variable's continued-fraction components
            loop {
                // The quotients of the current cycle are the continued-fraction components.
                if ts.q != rs.q {
                    // Since reciprocation changes the relative order of two variables,
                    // the less/greater-than test reverses after each index.
                    return if reverse != 0 { ts.q > rs.q } else { ts.q < rs.q };
                }

                // Prepare the next cycle
                reverse ^= 1;

                if ts.r == zero || rs.r == zero {
                    // At least one variable's c.f. expansion has ended
                    break;
                }

                ts.n = ts.d;
                ts.d = ts.r;
                ts.q = ts.n / ts.d;
                ts.r = ts.n % ts.d;
                rs.n = rs.d;
                rs.d = rs.r;
                rs.q = rs.n / rs.d;
                rs.r = rs.n % rs.d;
            }

            // Compare infinity-valued components for otherwise equal sequences
            if ts.r == rs.r {
                // Both remainders are zero: the sequences and their values are equal.
                false
            } else {
                // Exactly one of the remainders is zero.
                (ts.r != zero) != (reverse != 0)
            }
        }

        /// boost/rational.hpp `bool operator< (const T& i) const`.
        pub fn lt_int(&self, i: i128) -> bool {
            // Avoid repeated construction
            let zero: i128 = 0;

            // Break value into mixed-fraction form, w/ always-nonnegative remainder
            let mut q = self.num / self.den;
            let mut r = self.num % self.den;
            while r < zero {
                r += self.den;
                q -= 1;
            }

            // Compare with just the quotient, since the remainder always bumps the value up.
            q < i
        }

        /// boost/rational.hpp `bool operator== (const rational& r) const`.
        #[inline]
        pub fn eq(&self, r: &Rational) -> bool {
            self.num == r.num && self.den == r.den
        }

        /// boost::operators `x <= y` as `!(y < x)`.
        #[inline]
        pub fn le(&self, r: &Rational) -> bool {
            !r.lt(self)
        }

        /// boost::operators `x > y` as `y < x`.
        #[inline]
        pub fn gt(&self, r: &Rational) -> bool {
            r.lt(self)
        }
    }

    // ------------------------------------------------------------------------
    // geometry_traits.hpp:166-301 — PointPair, _Box, _Circle
    // ------------------------------------------------------------------------

    /// geometry_traits.hpp:171-222 `template<class P> class _Box: PointPair<P>` with `P = Slic3r::Point`.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct _Box {
        p1: Point,
        p2: Point,
        /// geometry_traits.hpp:179 `bool defined = false;`
        pub defined: bool,
    }

    impl Default for _Box {
        /// geometry_traits.hpp:181-182 `_Box(const P& center = {0, 0}): _Box(0, 0, center)`.
        fn default() -> Self {
            _Box::with_size(0, 0, Point::new(0, 0))
        }
    }

    impl _Box {
        /// geometry_traits.hpp:184-185 `_Box(const P& p, const P& pp)`: `defined = true`.
        #[inline]
        pub fn new(p: Point, pp: Point) -> Self {
            _Box { p1: p, p2: pp, defined: true }
        }

        /// geometry_traits.hpp:487-493 `_Box(TCoord<P> width, TCoord<P> height, const P& center)`:
        /// `{center - P{width / 2, height / 2}, center + P{width / 2, height / 2} + P{width % 2, height % 2}}`,
        /// `defined` left at its default (false).
        pub fn with_size(width: i32, height: i32, center: Point) -> Self {
            let (cx, cy) = (center.x() as i32, center.y() as i32);
            _Box {
                p1: Point::new(cx.wrapping_sub(width / 2) as Coord, cy.wrapping_sub(height / 2) as Coord),
                p2: Point::new(
                    cx.wrapping_add(width / 2).wrapping_add(width % 2) as Coord,
                    cy.wrapping_add(height / 2).wrapping_add(height % 2) as Coord,
                ),
                defined: false,
            }
        }

        /// geometry_traits.hpp:190
        #[inline]
        pub fn min_corner(&self) -> Point {
            self.p1
        }

        /// geometry_traits.hpp:191
        #[inline]
        pub fn max_corner(&self) -> Point {
            self.p2
        }

        /// geometry_traits.hpp:193
        #[inline]
        pub fn min_corner_mut(&mut self) -> &mut Point {
            &mut self.p1
        }

        /// geometry_traits.hpp:194
        #[inline]
        pub fn max_corner_mut(&mut self) -> &mut Point {
            &mut self.p2
        }

        /// geometry_traits.hpp:426-430 `width()`: `x(maxCorner()) - x(minCorner())`, an int32.
        #[inline]
        pub fn width(&self) -> i32 {
            (self.p2.x() as i32).wrapping_sub(self.p1.x() as i32)
        }

        /// geometry_traits.hpp:432-436 `height()`: an int32.
        #[inline]
        pub fn height(&self) -> i32 {
            (self.p2.y() as i32).wrapping_sub(self.p1.y() as i32)
        }

        /// geometry_traits.hpp:509-523 `center()`: `getX(minc) / 2 + getX(maxc) / 2` per coordinate,
        /// integer divisions.
        pub fn center(&self) -> Point {
            let (minc, maxc) = (self.p1, self.p2);
            Point::new(
                ((minc.x() as i32) / 2).wrapping_add((maxc.x() as i32) / 2) as Coord,
                ((minc.y() as i32) / 2).wrapping_add((maxc.y() as i32) / 2) as Coord,
            )
        }

        /// geometry_traits.hpp:202-205 `area<Unit = TCompute<P>>()`: `Unit(width()) * height()`, an int64.
        #[inline]
        pub fn area(&self) -> i64 {
            (self.width() as i64).wrapping_mul(self.height() as i64)
        }

        /// geometry_traits.hpp:202-205 `area<double>()`: `double(width()) * height()`.
        #[inline]
        pub fn area_f64(&self) -> f64 {
            self.width() as f64 * self.height() as f64
        }

        /// geometry_traits.hpp:207-219 `_Box intersection(_Box other)`.
        pub fn intersection(&self, other: _Box) -> _Box {
            let mut inter = _Box::default();
            inter.p1.x = self.p1.x().max(other.p1.x());
            inter.p1.y = self.p1.y().max(other.p1.y());
            inter.p2.x = self.p2.x().min(other.p2.x());
            inter.p2.y = self.p2.y().min(other.p2.y());
            inter.defined = true;
            if inter.p2.y() < inter.p1.y() || inter.p2.x() < inter.p1.x() {
                inter.p2 = inter.p1;
                inter.defined = false;
            }
            inter
        }

        /// geometry_traits.hpp:495-507 `static _Box infinite(const P &center)`: corners strictly less
        /// than half of the int32 range away from the center; `defined` stays false.
        pub fn infinite(center: Point) -> _Box {
            let mut ret = _Box::default();

            // It is important for Mx and My to be strictly less than half of the
            // range of type C. width(), height() and area() will not overflow this way.
            let mx = (i32::MIN.wrapping_add(2i32.wrapping_mul(center.x() as i32)) as f64 / 4.01) as i32;
            let my = (i32::MIN.wrapping_add(2i32.wrapping_mul(center.y() as i32)) as f64 / 4.01) as i32;

            ret.p2 = Point::new(
                (center.x() as i32).wrapping_sub(mx) as Coord,
                (center.y() as i32).wrapping_sub(my) as Coord,
            );
            ret.p1 = Point::new(
                (center.x() as i32).wrapping_add(mx) as Coord,
                (center.y() as i32).wrapping_add(my) as Coord,
            );

            ret
        }
    }

    /// geometry_traits.hpp:228-253 `template<class P> class _Circle`.
    #[derive(Debug, Clone, Copy, PartialEq, Default)]
    pub struct _Circle {
        center_: Point,
        radius_: f64,
    }

    impl _Circle {
        /// geometry_traits.hpp:238 `_Circle(const P& center, double r)`
        #[inline]
        pub fn new(center: Point, r: f64) -> Self {
            _Circle { center_: center, radius_: r }
        }
        /// geometry_traits.hpp:240
        #[inline]
        pub fn center(&self) -> Point {
            self.center_
        }
        /// geometry_traits.hpp:241
        #[inline]
        pub fn set_center(&mut self, c: Point) {
            self.center_ = c;
        }
        /// geometry_traits.hpp:243
        #[inline]
        pub fn radius(&self) -> f64 {
            self.radius_
        }
        /// geometry_traits.hpp:244
        #[inline]
        pub fn set_radius(&mut self, r: f64) {
            self.radius_ = r;
        }
        /// geometry_traits.hpp:246-248 `area()`: `Pi_2 * radius_ * radius_`.
        #[inline]
        pub fn area(&self) -> f64 {
            PI_2 * self.radius_ * self.radius_
        }
    }

    // ------------------------------------------------------------------------
    // geometry_traits.hpp:306-380 — pointlike (Unit = int64 unless noted)
    // ------------------------------------------------------------------------

    /// geometry_traits.hpp:331-337 `squaredDistance<P, Unit = int64>(p1, p2)`.
    #[inline]
    pub fn squared_distance(p1: &Point, p2: &Point) -> i64 {
        let (x1, y1, x2, y2) = (p1.x() as i32 as i64, p1.y() as i32 as i64, p2.x() as i32 as i64, p2.y() as i32 as i64);
        let (a, b) = (x2.wrapping_sub(x1), y2.wrapping_sub(y1));
        a.wrapping_mul(a).wrapping_add(b.wrapping_mul(b))
    }

    /// geometry_traits.hpp:339-343 `distance(p1, p2)`: `sqrt(squaredDistance<P, double>(p1, p2))`;
    /// `a * a + b * b` in double contracts to `fma(a, a, b * b)`.
    #[inline]
    pub fn distance(p1: &Point, p2: &Point) -> f64 {
        let (x1, y1, x2, y2) = (p1.x() as i32 as f64, p1.y() as i32 as f64, p2.x() as i32 as f64, p2.y() as i32 as f64);
        let (a, b) = (x2 - x1, y2 - y1);
        a.mul_add(a, b * b).sqrt()
    }

    /// geometry_traits.hpp:346-349 `perp(p)`: `Pt(y(p), -x(p))`.
    #[inline]
    pub fn perp(p: &Point) -> Point {
        Point::new(p.y(), (p.x() as i32).wrapping_neg() as Coord)
    }

    /// geometry_traits.hpp:351-355 `dotperp<Pt, int64>(a, b)`.
    #[inline]
    pub fn dotperp(a: &Point, b: &Point) -> i64 {
        (a.x() as i64).wrapping_mul(b.y() as i64).wrapping_sub((a.y() as i64).wrapping_mul(b.x() as i64))
    }

    /// geometry_traits.hpp:358-362 `dot<Pt, int64>(a, b)`.
    #[inline]
    pub fn dot(a: &Point, b: &Point) -> i64 {
        (a.x() as i64).wrapping_mul(b.x() as i64).wrapping_add((a.y() as i64).wrapping_mul(b.y() as i64))
    }

    /// geometry_traits.hpp:365-369 `magnsq<Pt, int64>(p)`.
    #[inline]
    pub fn magnsq(p: &Point) -> i64 {
        (p.x() as i64).wrapping_mul(p.x() as i64).wrapping_add((p.y() as i64).wrapping_mul(p.y() as i64))
    }

    /// geometry_traits.hpp:365-369 `magnsq<Pt, long double>(p)` (long double is double on the
    /// arm64 reference build): `x * x + y * y` contracts to `fma(x, x, y * y)`.
    #[inline]
    pub fn magnsq_ld(p: &Point) -> f64 {
        let (x, y) = (p.x() as f64, p.y() as f64);
        x.mul_add(x, y * y)
    }

    /// geometries.hpp:30-42 `Point operator-(const Point& p1, const Point& p2)` (int32).
    #[inline]
    fn sub(a: &Point, b: &Point) -> Point {
        Point::new(
            (a.x() as i32).wrapping_sub(b.x() as i32) as Coord,
            (a.y() as i32).wrapping_sub(b.y() as i32) as Coord,
        )
    }

    /// geometries.hpp:24-28 `Point operator+(const Point& p1, const Point& p2)` (int32).
    #[inline]
    fn add(a: &Point, b: &Point) -> Point {
        Point::new(
            (a.x() as i32).wrapping_add(b.x() as i32) as Coord,
            (a.y() as i32).wrapping_add(b.y() as i32) as Coord,
        )
    }

    /// geometries.hpp:30-35 `Point operator -(const Point& p)` (int32).
    #[inline]
    fn neg(p: &Point) -> Point {
        Point::new((p.x() as i32).wrapping_neg() as Coord, (p.y() as i32).wrapping_neg() as Coord)
    }

    // ------------------------------------------------------------------------
    // shapelike, as the libslic3r backend (geometries.hpp, boost_alg.hpp) and the defaults of
    // geometry_traits.hpp give it for Slic3r::ExPolygon
    // ------------------------------------------------------------------------

    /// boost_alg.hpp:399-405 `boundingBox(const PathImpl& sh, const PathTag&)`: `bp2d::Box b;
    /// boost::geometry::envelope(sh, b);` — the min and max of the points, written through the box's
    /// corner accessors, so `defined` keeps the default constructor's `false`. An empty path leaves
    /// boost's inverse box (min at the largest coordinate, max at the lowest).
    pub fn bounding_box_path(points: &[Point]) -> _Box {
        let mut b = _Box::default();
        let (mut minx, mut miny, mut maxx, mut maxy) = (i32::MAX, i32::MAX, i32::MIN, i32::MIN);
        for p in points {
            let (x, y) = (p.x() as i32, p.y() as i32);
            if x < minx {
                minx = x;
            }
            if x > maxx {
                maxx = x;
            }
            if y < miny {
                miny = y;
            }
            if y > maxy {
                maxy = y;
            }
        }
        b.p1 = Point::new(minx as Coord, miny as Coord);
        b.p2 = Point::new(maxx as Coord, maxy as Coord);
        b
    }

    /// geometry_traits.hpp:914-918 `boundingBox(const S& poly, const PolygonTag&)`: the box of the
    /// contour.
    #[inline]
    pub fn bounding_box(sh: &ExPolygon) -> _Box {
        bounding_box_path(&sh.contour.points)
    }

    /// geometry_traits.hpp:926-939 `boundingBox(const Circle& circ, const CircleTag&)`.
    pub fn bounding_box_circle(circ: &_Circle) -> _Box {
        let pmin = Point::new(
            (circ.center().x() as f64 - circ.radius()) as i32 as Coord,
            (circ.center().y() as f64 - circ.radius()) as i32 as Coord,
        );
        let pmax = Point::new(
            (circ.center().x() as f64 + circ.radius()) as i32 as Coord,
            (circ.center().y() as f64 + circ.radius()) as i32 as Coord,
        );
        _Box::new(pmin, pmax)
    }

    /// geometry_traits.hpp:949-965 `boundingBox(const _Box<P>& bb1, const _Box<P>& bb2)`.
    pub fn bounding_box_of_boxes(bb1: &_Box, bb2: &_Box) -> _Box {
        if !bb1.defined {
            return *bb2;
        }
        if !bb2.defined {
            return *bb1;
        }
        let (pminc, pmaxc) = (bb1.min_corner(), bb1.max_corner());
        let (iminc, imaxc) = (bb2.min_corner(), bb2.max_corner());
        let minc = Point::new(pminc.x().min(iminc.x()), pminc.y().min(iminc.y()));
        let maxc = Point::new(pmaxc.x().max(imaxc.x()), pmaxc.y().max(imaxc.y()));
        _Box::new(minc, maxc)
    }

    /// geometry_traits.hpp:987-1003 `area<Cntr, Unit = double>(const Cntr& poly, const PathTag&)`
    /// for `Slic3r::Polygon` (OrientationType COUNTER_CLOCKWISE, so the sign is flipped);
    /// `a += (xj + xi) * (yj - yi)` contracts to `fma(xj + xi, yj - yi, a)`.
    pub fn area_path(poly: &[Point]) -> f64 {
        if poly.len() < 3 {
            return 0.0;
        }

        let mut a: f64 = 0.0;
        let mut j = poly.len() - 1;
        for i in 0..poly.len() {
            let (xj, yj) = (poly[j].x() as f64, poly[j].y() as f64);
            let (xi, yi) = (poly[i].x() as f64, poly[i].y() as f64);
            a = (xj + xi).mul_add(yj - yi, a);
            j = i;
        }
        a /= 2.0;
        // is_clockwise<Slic3r::Polygon>() is false (geometries.hpp:101-104).
        -a
    }

    /// geometry_traits.hpp:1005-1013 `area(const S& poly, const PolygonTag&)`: the contour's area
    /// plus each hole's, accumulated in order.
    pub fn area(sh: &ExPolygon) -> f64 {
        let mut a = area_path(&sh.contour.points);
        for h in &sh.holes {
            a += area_path(&h.points);
        }
        a
    }

    /// MultiPoint.hpp:31 `rotate(double angle)` -> MultiPoint.cpp:27-35 `rotate(cos, sin)`:
    /// `coord_t(round(cos_angle * cur_x - sin_angle * cur_y))` and
    /// `coord_t(round(cos_angle * cur_y + sin_angle * cur_x))`, each contracted to an fma of its
    /// first product.
    fn rotate_points(points: &mut [Point], angle: f64) {
        let (cos_angle, sin_angle) = (angle.cos(), angle.sin());
        for pt in points.iter_mut() {
            let cur_x = pt.x() as f64;
            let cur_y = pt.y() as f64;
            let x = cos_angle.mul_add(cur_x, -(sin_angle * cur_y)).round() as i32;
            let y = cos_angle.mul_add(cur_y, sin_angle * cur_x).round() as i32;
            *pt = Point::new(x as Coord, y as Coord);
        }
    }

    /// geometries.hpp:226-230 `rotate(Slic3r::ExPolygon& sh, const Radians& rads)`:
    /// `sh.rotate(rads)` (ExPolygon.cpp:38-43, contour then holes).
    pub fn rotate(sh: &mut ExPolygon, rads: f64) {
        rotate_points(&mut sh.contour.points, rads);
        for hole in &mut sh.holes {
            rotate_points(&mut hole.points, rads);
        }
    }

    /// geometries.hpp:219-223 `translate(Slic3r::ExPolygon& sh, const Slic3r::Point& offs)`:
    /// `sh.translate(offs)` (ExPolygon.cpp:31-36), int32 additions.
    pub fn translate(sh: &mut ExPolygon, offs: &Point) {
        for p in sh.contour.points.iter_mut() {
            *p = add(p, offs);
        }
        for hole in &mut sh.holes {
            for p in hole.points.iter_mut() {
                *p = add(p, offs);
            }
        }
    }

    /// geometries.hpp:129-135 `offset(Slic3r::ExPolygon& sh, coord_t distance, const PolygonTag&)`:
    /// `Slic3r::offset_ex(sh, distance, jtSquare)` (the float delta, the default miter limit 3) and
    /// the first result, if any, replaces the shape.
    pub fn offset(sh: &mut ExPolygon, distance: Coord) {
        let res = offset_ex_expolygon_clib(sh, (distance as i32 as f32) as f64, OffsetJoinType::Square, 3.0);
        if !res.is_empty() {
            *sh = res[0].clone();
        }
    }

    /// geometry_traits.hpp:1270-1275 `offset(Box& bb, TCoord<Box> d, const BoxTag&)`.
    pub fn offset_box(bb: &mut _Box, d: i32) {
        let md = Point::new(d as Coord, d as Coord);
        bb.p1 = sub(&bb.p1, &md);
        bb.p2 = add(&bb.p2, &md);
    }

    /// geometry_traits.hpp:1277-1280 `offset(C& circ, TCoord<C> d, const CircleTag&)`.
    pub fn offset_circle(circ: &mut _Circle, d: i32) {
        circ.set_radius(circ.radius() + d as f64);
    }

    /// geometry_traits.hpp:1127-1132 `isInside(const TP& point, const TC& circ, PointTag, CircleTag)`:
    /// `squaredDistance(point, circ.center()) < r * r` (the int64 distance compared as a double).
    #[inline]
    pub fn is_inside_point_circle(point: &Point, circ: &_Circle) -> bool {
        let r = circ.radius();
        (squared_distance(point, &circ.center()) as f64) < r * r
    }

    /// geometry_traits.hpp:1135-1146 `isInside(const TP& point, const TB& box, PointTag, BoxTag)`.
    pub fn is_inside_point_box(point: &Point, b: &_Box) -> bool {
        let (px, py) = (point.x(), point.y());
        let (minx, miny) = (b.min_corner().x(), b.min_corner().y());
        let (maxx, maxy) = (b.max_corner().x(), b.max_corner().y());
        px > minx && px < maxx && py > miny && py < maxy
    }

    /// geometry_traits.hpp:1149-1156 `isInside(const S& sh, const TC& circ, PolygonTag, CircleTag)`:
    /// every contour vertex inside the circle.
    pub fn is_inside_shape_circle(sh: &ExPolygon, circ: &_Circle) -> bool {
        sh.contour.points.iter().all(|p| is_inside_point_circle(p, circ))
    }

    /// geometry_traits.hpp:1159-1165 `isInside(const TB& box, const TC& circ, BoxTag, CircleTag)`.
    pub fn is_inside_box_circle(b: &_Box, circ: &_Circle) -> bool {
        is_inside_point_circle(&b.min_corner(), circ) && is_inside_point_circle(&b.max_corner(), circ)
    }

    /// geometry_traits.hpp:1167-1182 `isInside(const TBGuest& ibb, const TBHost& box, BoxTag, BoxTag)`.
    pub fn is_inside_box_box(ibb: &_Box, b: &_Box) -> bool {
        let (imin_x, imax_x) = (ibb.min_corner().x(), ibb.max_corner().x());
        let (imin_y, imax_y) = (ibb.min_corner().y(), ibb.max_corner().y());
        let (min_x, max_x) = (b.min_corner().x(), b.max_corner().x());
        let (min_y, max_y) = (b.min_corner().y(), b.max_corner().y());
        imin_x >= min_x && imax_x <= max_x && imin_y >= min_y && imax_y <= max_y
    }

    /// geometry_traits.hpp:1184-1189 `isInside(const S& poly, const TB& box, PolygonTag, BoxTag)`:
    /// the shape's bounding box inside the box.
    pub fn is_inside_shape_box(sh: &ExPolygon, b: &_Box) -> bool {
        is_inside_box_box(&bounding_box(sh), b)
    }

    // ------------------------------------------------------------------------
    // utils/rotcalipers.hpp
    // ------------------------------------------------------------------------

    /// rotcalipers.hpp:14-46 `template<class Pt, class Unit = TCompute<Pt>> class RotatedBox`.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
    pub struct RotatedBox {
        axis_: Point,
        bottom_: i64,
        right_: i64,
    }

    impl RotatedBox {
        /// rotcalipers.hpp:20-21 `RotatedBox(const Pt& axis, Unit b, Unit r)`
        #[inline]
        pub fn new(axis: Point, b: i64, r: i64) -> Self {
            RotatedBox { axis_: axis, bottom_: b, right_: r }
        }

        /// rotcalipers.hpp:23-26 `long double area() const` (long double is double on arm64).
        #[inline]
        pub fn area(&self) -> f64 {
            let asq = magnsq_ld(&self.axis_);
            self.bottom_ as f64 * self.right_ as f64 / asq
        }

        /// rotcalipers.hpp:28-30 `long double width() const`.
        #[inline]
        pub fn width(&self) -> f64 {
            self.bottom_.wrapping_abs() as f64 / magnsq_ld(&self.axis_).sqrt()
        }

        /// rotcalipers.hpp:32-34 `long double height() const`.
        #[inline]
        pub fn height(&self) -> f64 {
            self.right_.wrapping_abs() as f64 / magnsq_ld(&self.axis_).sqrt()
        }

        /// rotcalipers.hpp:36
        #[inline]
        pub fn bottom_extent(&self) -> i64 {
            self.bottom_
        }

        /// rotcalipers.hpp:37
        #[inline]
        pub fn right_extent(&self) -> i64 {
            self.right_
        }

        /// rotcalipers.hpp:38
        #[inline]
        pub fn axis(&self) -> Point {
            self.axis_
        }

        /// rotcalipers.hpp:40-46 `Radians angleToX() const`.
        pub fn angle_to_x(&self) -> f64 {
            let mut ret = (self.axis_.y() as f64).atan2(self.axis_.x() as f64);
            // horizontal or vertical up-right rectangles are not distinguished, but we need to get the horizontal one
            if ret.abs() < EPSILON && self.bottom_.wrapping_abs() < self.right_.wrapping_abs() {
                ret += PI / 2.0;
            }
            let s = ret.is_sign_negative();
            if s {
                ret += PI_2;
            }
            -ret
        }
    }

    /// rotcalipers.hpp:49-71 `removeCollinearPoints(const Poly& sh, Unit eps = 0)` for a contour.
    pub fn remove_collinear_points(sh: &[Point], eps: i64) -> Vec<Point> {
        let mut ret: Vec<Point> = Vec::with_capacity(sh.len());

        let eprev0 = sub(&sh[0], &sh[sh.len() - 1]);
        let mut eprev = eprev0;

        let mut it = 0usize;
        let mut itx = 1usize;
        if itx != sh.len() {
            while it != sh.len() {
                let enext = sub(&sh[itx], &sh[it]);

                let dp = dotperp(&eprev, &enext);
                if dp.wrapping_abs() > eps {
                    ret.push(sh[it]);
                }

                eprev = enext;
                itx += 1;
                if itx == sh.len() {
                    itx = 0;
                }
                it += 1;
            }
        }

        ret
    }

    /// rotcalipers.hpp:95-102 `rectarea<Pt, Unit, R>(const Pt& w, const Unit& a, const Unit& b)`
    /// with `R = boost::rational<LargeInt>`: `R m = R(a) / magnsq(w); m = m * b;`.
    pub fn rectarea(w: &Point, a: i64, b: i64) -> Result<Rational, BadRational> {
        let m = Rational::from_int(a as i128).div_int(magnsq(w) as i128)?;
        Ok(m.mul_int(b as i128))
    }

    /// rotcalipers.hpp:104-108 `rectarea<R>(const RotatedBox<Pt, Unit> &rb)`.
    #[inline]
    pub fn rectarea_box(rb: &RotatedBox) -> Result<Rational, BadRational> {
        rectarea(&rb.axis(), rb.bottom_extent(), rb.right_extent())
    }

    /// rotcalipers.hpp:112-286 `rotcalipers<RawShape, Unit, Ratio>(const RawShape& sh, VisitFn&& visitfn)`
    /// for `RawShape = ExPolygon` (the contour), `Unit = int64`, `Ratio = boost::rational<LargeInt>`.
    /// A zero divisor in the rational arithmetic (a zero-length edge) is boost's `bad_rational`
    /// exception, returned as `Err`.
    ///
    /// This function is only applicable to counter-clockwise oriented convex polygons where only
    /// two points can be collinear witch each other.
    pub fn rotcalipers<F: FnMut(&RotatedBox) -> bool>(sh: &ExPolygon, mut visitfn: F) -> Result<(), BadRational> {
        let src: &[Point] = &sh.contour.points;
        if src.is_empty() {
            // `std::prev(sl::cend(sh))` of an empty contour is undefined behaviour in the C++.
            return Ok(());
        }

        // Get the first and the last vertex iterator
        let mut first = 0usize;
        let mut last = src.len() - 1;

        // Check conditions and return undefined box if input is not sane.
        if last == first {
            return Ok(());
        }
        if src[first].x() == src[last].x() && src[first].y() == src[last].y() {
            last -= 1;
        }
        if (last as isize) - (first as isize) < 2 {
            return Ok(());
        }

        let mut shcpy: Vec<Point> = Vec::new(); // empty at this point
        let mut use_copy = false;
        {
            let (p, q, r) = (src[first], src[first + 1], src[last]);

            // Determine orientation from first 3 vertex (should be consistent)
            let d: i64 = ((q.y() as i64).wrapping_sub(p.y() as i64))
                .wrapping_mul((r.x() as i64).wrapping_sub(p.x() as i64))
                .wrapping_sub(
                    ((q.x() as i64).wrapping_sub(p.x() as i64)).wrapping_mul((r.y() as i64).wrapping_sub(p.y() as i64)),
                );

            if d > 0 {
                // The polygon is clockwise. A flip is needed (for now)
                shcpy.reserve(last - first);
                let mut it = last;
                while it != first {
                    shcpy.push(src[it]);
                    it -= 1;
                }
                shcpy.push(src[first]);
                use_copy = true;
            }
        }
        let pts: &[Point] = if use_copy {
            first = 0;
            last = shcpy.len() - 1;
            &shcpy
        } else {
            src
        };

        // Cyclic iterator increment
        let inc = |it: &mut usize| {
            if *it == last {
                *it = first;
            } else {
                *it += 1;
            }
        };

        // Cyclic previous iterator
        let prev = |it: usize| -> usize {
            if it == first {
                last
            } else {
                it - 1
            }
        };

        // Cyclic next iterator
        let next = |it: usize| -> usize {
            if it == last {
                first
            } else {
                it + 1
            }
        };

        // Establish initial (axis aligned) rectangle support verices by determining
        // polygon extremes:

        let mut it = first;
        let (mut min_x, mut max_x, mut min_y, mut max_y) = (it, it, it, it);

        loop {
            // Linear walk through the vertices and save the extreme positions
            let v = pts[it];
            let mut d = sub(&v, &pts[min_x]);
            if d.x() < 0 || (d.x() == 0 && d.y() < 0) {
                min_x = it;
            }

            d = sub(&v, &pts[max_x]);
            if d.x() > 0 || (d.x() == 0 && d.y() > 0) {
                max_x = it;
            }

            d = sub(&v, &pts[min_y]);
            if d.y() < 0 || (d.y() == 0 && d.x() > 0) {
                min_y = it;
            }

            d = sub(&v, &pts[max_y]);
            if d.y() > 0 || (d.y() == 0 && d.x() < 0) {
                max_y = it;
            }

            it += 1;
            if it == last + 1 {
                break;
            }
        }

        // Update the vertices defining the bounding rectangle. The rectangle with
        // the smallest rotation is selected and the supporting vertices are
        // returned in the 'rect' argument.
        let update = |w: &Point, rect: &mut [usize; 4]| -> Result<bool, BadRational> {
            let (bb, bn) = (rect[0], next(rect[0]));
            let (rr, rn) = (rect[1], next(rect[1]));
            let (tt, tn) = (rect[2], next(rect[2]));
            let (ll, ln) = (rect[3], next(rect[3]));

            let b = sub(&pts[bn], &pts[bb]);
            let r = sub(&pts[rn], &pts[rr]);
            let t = sub(&pts[tn], &pts[tt]);
            let l = sub(&pts[ln], &pts[ll]);
            let pw = perp(w);

            let dotwpb = dot(w, &b);
            let dotwpr = dot(&neg(&pw), &r);
            let dotwpt = dot(&neg(w), &t);
            let dotwpl = dot(&pw, &l);
            let dw = magnsq(w);

            let angles: [Rational; 4] = [
                Rational::from_int(dotwpb as i128).div_int(magnsq(&b) as i128)?.mul_int(dotwpb as i128),
                Rational::from_int(dotwpr as i128).div_int(magnsq(&r) as i128)?.mul_int(dotwpr as i128),
                Rational::from_int(dotwpt as i128).div_int(magnsq(&t) as i128)?.mul_int(dotwpt as i128),
                Rational::from_int(dotwpl as i128).div_int(magnsq(&l) as i128)?.mul_int(dotwpl as i128),
            ];

            // using AngleIndex = std::pair<Ratio, size_t>;
            let mut a: Vec<(Rational, usize)> = Vec::with_capacity(4);

            let (mut i, mut j) = (3usize, 0usize);
            while j < 4 {
                if rect[i] != rect[j] && angles[i].lt_int(dw as i128) {
                    let iv = (angles[i], i);
                    // std::lower_bound with `ai.first > aj.first`: the first element that is not
                    // greater than iv.
                    let pos = a.partition_point(|ai| ai.0.gt(&iv.0));
                    a.insert(pos, iv);
                }
                i = j;
                j += 1;
            }

            // The polygon is supposed to be a rectangle.
            if a.is_empty() {
                return Ok(false);
            }

            let amin = a[0].0;
            let imin = a[0].1;
            for ai in &a {
                if ai.0.eq(&amin) {
                    inc(&mut rect[ai.1]);
                }
            }

            rect.rotate_left(imin);

            Ok(true)
        };

        let mut w = Point::new(1, 0);
        let mut rect: [usize; 4] = [min_y, max_x, max_y, min_x];

        {
            let a = dot(&w, &sub(&pts[rect[1]], &pts[rect[3]]));
            let b = dot(&neg(&perp(&w)), &sub(&pts[rect[2]], &pts[rect[0]]));
            if !visitfn(&RotatedBox::new(w, a, b)) {
                return Ok(());
            }
        }

        // An edge might be examined twice in which case the algorithm terminates.
        let (mut c, count) = (0usize, last - first + 1);
        let mut edgemask: Vec<bool> = vec![false; count];

        while {
            let go = c < count;
            c += 1;
            go
        } {
            // Update the support vertices, if cannot be updated, break the cycle.
            if !update(&w, &mut rect)? {
                break;
            }

            let eidx = rect[0] - first;

            if edgemask[eidx] {
                break;
            }
            edgemask[eidx] = true;

            // get the unnormalized direction vector
            w = sub(&pts[rect[0]], &pts[prev(rect[0])]);

            let a = dot(&w, &sub(&pts[rect[1]], &pts[rect[3]]));
            let b = dot(&neg(&perp(&w)), &sub(&pts[rect[2]], &pts[rect[0]]));
            if !visitfn(&RotatedBox::new(w, a, b)) {
                break;
            }
        }

        Ok(())
    }

    /// rotcalipers.hpp:288-310 `minAreaBoundingBox<S, Unit, Ratio>(const S& sh)`.
    pub fn min_area_bounding_box(sh: &ExPolygon) -> Result<RotatedBox, BadRational> {
        let mut minbox = RotatedBox::default();
        let mut minarea = Rational::from_int(i64::MAX as i128);
        let mut err: Option<BadRational> = None;
        let minfn = |rbox: &RotatedBox| -> bool {
            match rectarea_box(rbox) {
                Ok(area) => {
                    if area.le(&minarea) {
                        minarea = area;
                        minbox = *rbox;
                    }
                    true // continue search
                }
                Err(e) => {
                    // The exception leaves the visitor and the calipers.
                    err = Some(e);
                    false
                }
            }
        };

        rotcalipers(sh, minfn)?;
        if let Some(e) = err {
            return Err(e);
        }

        Ok(minbox)
    }

    /// rotcalipers.hpp:312-315 `minAreaBoundingBoxRotation(const RawShape& sh)`.
    pub fn min_area_bounding_box_rotation(sh: &ExPolygon) -> Result<f64, BadRational> {
        Ok(min_area_bounding_box(sh)?.angle_to_x())
    }

    /// rotcalipers.hpp:326-370 `fitIntoBoxRotation<S, Unit, Ratio>(const S &shape, const _Box<TPoint<S>> &box, Radians eps = 1e-4)`.
    ///
    /// Note (as in the C++): `prev_rbox` is never updated by the visitor, so `lower_prev` compares
    /// the aspect ratio of a default box (0 / 0) and is always false.
    pub fn fit_into_box_rotation(shape: &ExPolygon, bx: &_Box, eps: f64) -> Result<f64, BadRational> {
        // constexpr auto get_aspect_r = [](const auto &b) -> double { return double(b.width()) / b.height(); };
        let get_aspect_r_box = |b: &_Box| -> f64 { b.width() as f64 / b.height() as f64 };
        let get_aspect_r_rbox = |b: &RotatedBox| -> f64 { b.width() / b.height() };

        let aspect_r = get_aspect_r_box(bx);

        let prev_rbox = RotatedBox::default();
        let (mut a_from, mut a_to) = (0.0_f64, 0.0_f64);
        let visitfn = |rbox: &RotatedBox| -> bool {
            let lower_prev = get_aspect_r_rbox(&prev_rbox) < aspect_r;
            let lower_current = get_aspect_r_rbox(rbox) < aspect_r;

            if lower_prev != lower_current {
                a_from = prev_rbox.angle_to_x();
                a_to = rbox.angle_to_x();
                return false;
            }

            true
        };

        rotcalipers(shape, visitfn)?;

        let rot_shape_bb = |r: f64| -> _Box {
            let mut s = shape.clone();
            rotate(&mut s, r);
            bounding_box(&s)
        };

        let rot_aspect_r = |r: f64| -> f64 { get_aspect_r_box(&rot_shape_bb(r)) };

        // Lets bisect the retrieved interval where the correct aspect ratio is.
        let mut ar_from = rot_aspect_r(a_from);
        let would_fit = |b: &_Box| -> bool { b.width() < bx.width() && b.height() < bx.height() };

        let mut middle = (a_from + a_to) / 2.0;
        let mut box_middle = rot_shape_bb(middle);
        while !would_fit(&box_middle) && (a_to - a_from).abs() > eps {
            let ar_middle = get_aspect_r_box(&box_middle);
            if (ar_from < aspect_r) != (ar_middle < aspect_r) {
                a_to = middle;
            } else {
                a_from = middle;
            }

            ar_from = rot_aspect_r(a_from);
            middle = (a_from + a_to) / 2.0;
            box_middle = rot_shape_bb(middle);
        }

        Ok(middle)
    }

    // ------------------------------------------------------------------------
    // nester.hpp:32-523 — template<class RawShape> class _Item (RawShape = Slic3r::ExPolygon)
    // ------------------------------------------------------------------------

    /// The `mutable` caches of `_Item` (nester.hpp:59-85).
    #[derive(Debug, Clone, Default)]
    struct ItemCache {
        tr_cache_: ExPolygon,
        tr_cache_valid_: bool,
        area_cache_: f64,
        area_cache_valid_: bool,
        inflate_cache_: ExPolygon,
        inflate_cache_valid_: bool,
        expected_infl_trans_cache_: ExPolygon,
        expected_infl_trans_valid_: bool,
        expected_infl_: Coord,
        rmt_: usize,
        lmb_: usize,
        rmt_valid_: bool,
        lmb_valid_: bool,
        bb_cache_bb: _Box,
        bb_cache_valid: bool,
    }

    /// nester.hpp:32-523 `template<class RawShape> class _Item` with `RawShape = Slic3r::ExPolygon`:
    /// the shape, its transformation, the caches and the BBS item attributes.
    ///
    /// Ported here: the constructors, the bin / priority / id accessors, `area`, the vertex
    /// accessors, `isInside` for a box and a circle, the transformation setters and getters,
    /// `transformedShape`, `rawShape`, `resetTransformation`, `boundingBox`, the reference
    /// vertices and the expected-inflation shape. Not ported: `isContourConvex` (`sl::isConvex`),
    /// `isInside` against a point or a polygon and `intersects` / `touches` (boost::geometry
    /// `within`, `intersects`, `touches`), and `toString`.
    #[derive(Debug, Clone)]
    pub struct _Item {
        // The original shape that gets encapsulated.
        sh_: ExPolygon,

        // Transformation data
        translation_: Point,
        rotation_: f64,
        inflation_: Coord,

        // Info about whether the transformations will have to take place
        has_rotation_: bool,
        has_translation_: bool,
        has_inflation_: bool,

        // BBS: true when _trypack degraded this item's bed contour to raw.
        use_raw_shape_: bool,

        cache: RefCell<ItemCache>,

        binid_: i32,
        priority_: i32,
        fixed_: bool,

        /// nester.hpp:91
        pub itemid_: i32,
        /// nester.hpp:92 extrude id to filament type
        pub extrude_id_filament_types: std::collections::BTreeMap<i32, String>,
        /// nester.hpp:93 -1 means unset. otherwise should be one of FilamentTempType ie {0,1,2}
        pub filament_temp_type: i32,
        /// nester.hpp:94
        pub height: f64,
        /// nester.hpp:95
        pub print_temp: f64,
        /// nester.hpp:96
        pub bed_temp: f64,
        /// nester.hpp:97 vitrify temperature
        pub vitrify_temp: f64,
        /// nester.hpp:98
        pub name: String,
        /// nester.hpp:100 BBS: virtual object to mark unprintable region on heatbed
        pub is_virt_object: bool,
        /// nester.hpp:101
        pub is_wipe_tower: bool,
        /// nester.hpp:102
        pub is_extrusion_cali_object: bool,
        /// nester.hpp:103
        pub has_tried_without_extrusion_cali_obj: bool,
        /// nester.hpp:104 `std::vector<std::pair<double, Coord>> allowed_rotations{{0., 0.}}; // <rotation, inflation>`
        pub allowed_rotations: Vec<(f64, Coord)>,
    }

    impl _Item {
        /// nester.hpp:137 `explicit _Item(const RawShape& sh): sh_(sh) {}` (and the move variant).
        pub fn new(sh: ExPolygon) -> Self {
            _Item {
                sh_: sh,
                translation_: Point::new(0, 0),
                rotation_: 0.0,
                inflation_: 0,
                has_rotation_: false,
                has_translation_: false,
                has_inflation_: false,
                use_raw_shape_: false,
                cache: RefCell::new(ItemCache::default()),
                binid_: BIN_ID_UNSET,
                priority_: 0,
                fixed_: false,
                itemid_: 0,
                extrude_id_filament_types: std::collections::BTreeMap::new(),
                filament_temp_type: -1,
                height: 0.0,
                print_temp: 0.0,
                bed_temp: 0.0,
                vitrify_temp: 0.0,
                name: String::new(),
                is_virt_object: false,
                is_wipe_tower: false,
                is_extrusion_cali_object: false,
                has_tried_without_extrusion_cali_obj: false,
                allowed_rotations: vec![(0.0, 0)],
            }
        }

        /// nester.hpp:153-155 `_Item(const TContour<RawShape>& contour, const THolesContainer<RawShape>& holes = {})`:
        /// `sh_(sl::create<RawShape>(contour, holes))` (geometries.hpp:174-182).
        pub fn from_contour(contour: crate::geometry::Polygon, holes: Vec<crate::geometry::Polygon>) -> Self {
            _Item::new(ExPolygon { contour, holes })
        }

        /// nester.hpp:161
        #[inline]
        pub fn is_fixed(&self) -> bool {
            self.fixed_
        }

        /// nester.hpp:162-166
        #[inline]
        pub fn mark_as_fixed_in_bin(&mut self, binid: i32) {
            self.fixed_ = binid >= 0;
            self.binid_ = binid;
        }

        /// nester.hpp:168 `void binId(int idx)`
        #[inline]
        pub fn set_bin_id(&mut self, idx: i32) {
            self.binid_ = idx;
        }

        /// nester.hpp:169 `int binId() const`
        #[inline]
        pub fn bin_id(&self) -> i32 {
            self.binid_
        }

        /// nester.hpp:171 `void priority(int p)`
        #[inline]
        pub fn set_priority(&mut self, p: i32) {
            self.priority_ = p;
        }

        /// nester.hpp:172 `int priority() const`
        #[inline]
        pub fn priority(&self) -> i32 {
            self.priority_
        }

        /// nester.hpp:174 `void itemId(int idx)`
        #[inline]
        pub fn set_item_id(&mut self, idx: i32) {
            self.itemid_ = idx;
        }

        /// nester.hpp:175 `int itemId() const`
        #[inline]
        pub fn item_id(&self) -> i32 {
            self.itemid_
        }

        /// nester.hpp:220-223 `Vertex vertex(unsigned long idx) const`: of the raw contour.
        #[inline]
        pub fn vertex(&self, idx: usize) -> Point {
            self.sh_.contour.points[idx]
        }

        /// nester.hpp:234-238 `void setVertex(unsigned long idx, const Vertex& v)`
        pub fn set_vertex(&mut self, idx: usize, v: Point) {
            self.invalidate_cache();
            self.sh_.contour.points[idx] = v;
        }

        /// nester.hpp:248-257 `double area() const`: the area of the inflated shape, cached.
        pub fn area(&self) -> f64 {
            {
                let c = self.cache.borrow();
                if c.area_cache_valid_ {
                    return c.area_cache_;
                }
            }
            let ret = area(&self.infalted_shape());
            let mut c = self.cache.borrow_mut();
            c.area_cache_ = ret;
            c.area_cache_valid_ = true;
            ret
        }

        /// nester.hpp:283-285
        #[inline]
        pub fn vertex_count(&self) -> usize {
            self.sh_.contour.points.len()
        }

        /// nester.hpp:287-289
        #[inline]
        pub fn hole_count(&self) -> usize {
            self.sh_.holes.len()
        }

        /// nester.hpp:508-511 `bool isInside(const _Box<TPoint<RawShape>>& box) const`:
        /// `sl::isInside(boundingBox(), box)`.
        pub fn is_inside_box(&self, b: &_Box) -> bool {
            is_inside_box_box(&self.bounding_box(), b)
        }

        /// nester.hpp:513-516 `bool isInside(const _Circle<TPoint<RawShape>>& circ) const`.
        pub fn is_inside_circle(&self, circ: &_Circle) -> bool {
            is_inside_shape_circle(&self.transformed_shape(), circ)
        }

        /// nester.hpp:314-317 `void translate(const Vertex& d)`
        pub fn translate(&mut self, d: &Point) {
            let t = add(&self.translation(), d);
            self.set_translation(t);
        }

        /// nester.hpp:319-322 `void rotate(const Radians& rads)`
        pub fn rotate(&mut self, rads: f64) {
            let r = self.rotation() + rads;
            self.set_rotation(r);
        }

        /// nester.hpp:324-329 `void inflation(Coord distance)`
        pub fn set_inflation(&mut self, distance: Coord) {
            self.inflation_ = distance;
            self.has_inflation_ = true;
            self.invalidate_cache();
        }

        /// nester.hpp:331-333 `Coord inflation() const`
        #[inline]
        pub fn inflation(&self) -> Coord {
            self.inflation_
        }

        /// nester.hpp:338 `void useRawShape(bool b)`
        #[inline]
        pub fn set_use_raw_shape(&mut self, b: bool) {
            self.use_raw_shape_ = b;
        }

        /// nester.hpp:339 `bool useRawShape() const`
        #[inline]
        pub fn use_raw_shape(&self) -> bool {
            self.use_raw_shape_
        }

        /// nester.hpp:344-348 `const RawShape& cacheExpectedInflTransShape(Coord expected_infl) const`
        pub fn cache_expected_infl_trans_shape(&self, expected_infl: Coord) -> ExPolygon {
            self.cache.borrow_mut().expected_infl_ = expected_infl;
            self.rebuild_expected_infl_trans_shape()
        }

        /// nester.hpp:354-357 `const RawShape& expectedInflTransShape() const`
        pub fn expected_infl_trans_shape(&self) -> ExPolygon {
            {
                let c = self.cache.borrow();
                if c.expected_infl_trans_valid_ {
                    return c.expected_infl_trans_cache_.clone();
                }
            }
            self.rebuild_expected_infl_trans_shape()
        }

        /// nester.hpp:359-362 `void inflate(Coord distance)`
        pub fn inflate(&mut self, distance: Coord) {
            let d = (self.inflation() as i32).wrapping_add(distance as i32) as Coord;
            self.set_inflation(d);
        }

        /// nester.hpp:364-367 `Radians rotation() const`
        #[inline]
        pub fn rotation(&self) -> f64 {
            self.rotation_
        }

        /// nester.hpp:369-372 `TPoint<RawShape> translation() const`
        #[inline]
        pub fn translation(&self) -> Point {
            self.translation_
        }

        /// nester.hpp:374-382 `void rotation(Radians rot)`
        pub fn set_rotation(&mut self, rot: f64) {
            if self.rotation_ != rot {
                self.rotation_ = rot;
                self.has_rotation_ = true;
                let c = self.cache.get_mut();
                c.tr_cache_valid_ = false;
                c.expected_infl_trans_valid_ = false;
                c.rmt_valid_ = false;
                c.lmb_valid_ = false;
                c.bb_cache_valid = false;
            }
        }

        /// nester.hpp:384-391 `void translation(const TPoint<RawShape>& tr)`
        pub fn set_translation(&mut self, tr: Point) {
            if self.translation_ != tr {
                self.translation_ = tr;
                self.has_translation_ = true;
                let c = self.cache.get_mut();
                c.tr_cache_valid_ = false;
                c.expected_infl_trans_valid_ = false;
                //bb_cache_.valid = false;
            }
        }

        /// nester.hpp:393-404 `const RawShape& transformedShape() const`
        pub fn transformed_shape(&self) -> ExPolygon {
            {
                let c = self.cache.borrow();
                if c.tr_cache_valid_ {
                    return c.tr_cache_.clone();
                }
            }

            let mut cpy = self.infalted_shape();
            if self.has_rotation_ {
                rotate(&mut cpy, self.rotation_);
            }
            if self.has_translation_ {
                translate(&mut cpy, &self.translation_);
            }
            let mut c = self.cache.borrow_mut();
            c.tr_cache_ = cpy.clone();
            c.tr_cache_valid_ = true;
            c.rmt_valid_ = false;
            c.lmb_valid_ = false;

            cpy
        }

        /// nester.hpp:411-414 `const RawShape& rawShape() const`
        #[inline]
        pub fn raw_shape(&self) -> &ExPolygon {
            &self.sh_
        }

        /// nester.hpp:416-420 `void resetTransformation()`
        pub fn reset_transformation(&mut self) {
            self.has_translation_ = false;
            self.has_rotation_ = false;
            self.has_inflation_ = false;
            self.invalidate_cache();
        }

        /// nester.hpp:422-437 `Box boundingBox() const`: the cached box of the inflated (and
        /// rotated) shape, shifted by the translation.
        pub fn bounding_box(&self) -> _Box {
            let valid = self.cache.borrow().bb_cache_valid;
            if !valid {
                let bb = if !self.has_rotation_ {
                    bounding_box(&self.infalted_shape())
                } else {
                    // TODO make sure this works
                    let mut rotsh = self.infalted_shape();
                    rotate(&mut rotsh, self.rotation_);
                    bounding_box(&rotsh)
                };
                let mut c = self.cache.borrow_mut();
                c.bb_cache_bb = bb;
                c.bb_cache_valid = true;
            }

            let c = self.cache.borrow();
            let (bb, tr) = (&c.bb_cache_bb, &self.translation_);
            _Box::new(add(&bb.min_corner(), tr), add(&bb.max_corner(), tr))
        }

        /// nester.hpp:439-441 `Vertex referenceVertex() const`
        #[inline]
        pub fn reference_vertex(&self) -> Point {
            self.rightmost_top_vertex()
        }

        /// nester.hpp:443-450 `Vertex rightmostTopVertex() const`: `std::max_element` with `vsort`
        /// (the first of equal maxima).
        pub fn rightmost_top_vertex(&self) -> Point {
            let need = {
                let c = self.cache.borrow();
                !c.rmt_valid_ || !c.tr_cache_valid_
            };
            let tsh = self.transformed_shape();
            if need {
                // find max x and max y vertex
                let pts = &tsh.contour.points;
                let mut largest = 0usize;
                for i in 1..pts.len() {
                    if Self::vsort(&pts[largest], &pts[i]) {
                        largest = i;
                    }
                }
                let mut c = self.cache.borrow_mut();
                c.rmt_ = largest;
                c.rmt_valid_ = true;
            }
            tsh.contour.points[self.cache.borrow().rmt_]
        }

        /// nester.hpp:452-459 `Vertex leftmostBottomVertex() const`: `std::min_element` with `vsort`
        /// (the first of equal minima).
        pub fn leftmost_bottom_vertex(&self) -> Point {
            let need = {
                let c = self.cache.borrow();
                !c.lmb_valid_ || !c.tr_cache_valid_
            };
            let tsh = self.transformed_shape();
            if need {
                // find min x and min y vertex
                let pts = &tsh.contour.points;
                let mut smallest = 0usize;
                for i in 1..pts.len() {
                    if Self::vsort(&pts[i], &pts[smallest]) {
                        smallest = i;
                    }
                }
                let mut c = self.cache.borrow_mut();
                c.lmb_ = smallest;
                c.lmb_valid_ = true;
            }
            tsh.contour.points[self.cache.borrow().lmb_]
        }

        /// nester.hpp:477-485 `const RawShape& rebuildExpectedInflTransShape() const`
        fn rebuild_expected_infl_trans_shape(&self) -> ExPolygon {
            let mut cpy = self.sh_.clone();
            let expected_infl = self.cache.borrow().expected_infl_;
            if expected_infl > 0 {
                offset(&mut cpy, expected_infl);
            }
            if self.has_rotation_ {
                rotate(&mut cpy, self.rotation_);
            }
            if self.has_translation_ {
                translate(&mut cpy, &self.translation_);
            }
            let mut c = self.cache.borrow_mut();
            c.expected_infl_trans_cache_ = cpy.clone();
            c.expected_infl_trans_valid_ = true;
            cpy
        }

        /// nester.hpp:487-497 `const RawShape& infaltedShape() const`
        fn infalted_shape(&self) -> ExPolygon {
            if self.has_inflation_ {
                {
                    let c = self.cache.borrow();
                    if c.inflate_cache_valid_ {
                        return c.inflate_cache_.clone();
                    }
                }

                let mut inflated = self.sh_.clone();
                offset(&mut inflated, self.inflation_);
                let mut c = self.cache.borrow_mut();
                c.inflate_cache_ = inflated.clone();
                c.inflate_cache_valid_ = true;
                return inflated;
            }
            self.sh_.clone()
        }

        /// nester.hpp:499-508 `void invalidateCache() const`
        fn invalidate_cache(&self) {
            let mut c = self.cache.borrow_mut();
            c.tr_cache_valid_ = false;
            c.expected_infl_trans_valid_ = false;
            c.lmb_valid_ = false;
            c.rmt_valid_ = false;
            c.area_cache_valid_ = false;
            c.inflate_cache_valid_ = false;
            c.bb_cache_valid = false;
        }

        /// nester.hpp:510-515 `static bool vsort(const Vertex& v1, const Vertex& v2)`
        fn vsort(v1: &Point, v2: &Point) -> bool {
            let (x1, x2) = (v1.x(), v2.x());
            let (y1, y2) = (v1.y(), v2.y());
            if y1 == y2 {
                x1 < x2
            } else {
                y1 < y2
            }
        }
    }

    // ------------------------------------------------------------------------
    // placers/nfpplacer.hpp:29-142 — template<class RawShape> struct NfpPConfig
    // ------------------------------------------------------------------------

    /// nfpplacer.hpp:34-43 `enum class Alignment`
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum NfpPConfigAlignment {
        Center,
        BottomLeft,
        BottomRight,
        TopLeft,
        TopRight,
        /// Warning: parts may end up outside the bin with the default object function.
        DontAlign,
        UserDefined,
    }

    /// nfpplacer.hpp:29-142 `template<class RawShape> struct NfpPConfig` with `RawShape = ExPolygon`.
    ///
    /// The callbacks take the item groups (`std::vector<std::reference_wrapper<_Item>>`) as slices of
    /// item references. `m_excluded_items` (an item group the placer fills) is not carried: nothing
    /// ported here reads or writes it.
    #[derive(Clone)]
    pub struct NfpPConfig {
        /// Which angles to try out for better results.
        pub rotations: Vec<f64>,
        /// Where to align the resulting packed pile.
        pub alignment: NfpPConfigAlignment,
        /// Where to start putting objects in the bin.
        pub starting_point: NfpPConfigAlignment,
        /// nfpplacer.hpp:54
        pub best_object_pos: Point,
        /// scaled bed shrink in x and y direction
        pub bed_shrink: Point,
        /// nfpplacer.hpp:76 the fitting function of the placement optimization.
        pub object_function: Option<Rc<dyn Fn(&_Item, &[&_Item]) -> f64>>,
        /// nfpplacer.hpp:83 `float accuracy = 0.65f;`
        pub accuracy: f32,
        /// nfpplacer.hpp:92
        pub explore_holes: bool,
        /// nfpplacer.hpp:97
        pub parallel: bool,
        /// nfpplacer.hpp:99
        pub save_svg: bool,
        /// nfpplacer.hpp:123-126 (merged pile, packed items, remaining items)
        pub before_packing: Option<Rc<dyn Fn(&[ExPolygon], &[&_Item], &[&_Item])>>,
        /// nfpplacer.hpp:128
        pub on_preload: Option<Rc<dyn Fn(&[&_Item], &mut NfpPConfig)>>,
        /// nfpplacer.hpp:131 BBS: sort function for selector
        pub sortfunc: Option<Rc<dyn Fn(&_Item, &_Item) -> bool>>,
        /// nfpplacer.hpp:133
        pub progress_func: Option<Rc<dyn Fn(&str)>>,
        /// nfpplacer.hpp:136 BBS: excluded region for V4 bed
        pub m_excluded_regions: Vec<_Item>,
        /// nfpplacer.hpp:138
        pub m_nonprefered_regions: Vec<_Item>,
    }

    impl Default for NfpPConfig {
        /// nfpplacer.hpp:140-141 `NfpPConfig(): rotations({0.0, Pi/2.0, Pi, 3*Pi/2}),
        /// alignment(Alignment::CENTER), starting_point(Alignment::CENTER) {}`
        fn default() -> Self {
            NfpPConfig {
                rotations: vec![0.0, PI / 2.0, PI, 3.0 * PI / 2.0],
                alignment: NfpPConfigAlignment::Center,
                starting_point: NfpPConfigAlignment::Center,
                best_object_pos: Point::new(0, 0),
                bed_shrink: Point::new(0, 0),
                object_function: None,
                accuracy: 0.65,
                explore_holes: false,
                parallel: true,
                save_svg: false,
                before_packing: None,
                on_preload: None,
                sortfunc: None,
                progress_func: None,
                m_excluded_regions: Vec::new(),
                m_nonprefered_regions: Vec::new(),
            }
        }
    }
}

// ============================================================================
// Arrange.hpp:107-185 — struct ArrangeParams
// ============================================================================

/// Arrange.hpp:107-185 `struct ArrangeParams`.
#[derive(Clone)]
pub struct ArrangeParams {
    /// The minimum distance which is allowed for any
    /// pair of items on the print bed in any direction.
    pub min_obj_distance: Coord,

    /// Optional lower bound for per-item inflation when min_obj_distance == 0.
    /// 0 means disabled (legacy behavior).
    pub min_inflation_floor: Coord,

    /// The accuracy of optimization.
    /// Goes from 0.0 to 1.0 and scales performance as well
    pub accuracy: f32,

    /// Allow parallel execution.
    pub parallel: bool,

    /// Arrange.hpp:128
    pub allow_rotations: bool,

    /// Arrange.hpp:130
    pub do_final_align: bool,

    //BBS: add specific arrange params
    /// Arrange.hpp:133
    pub allow_multi_materials_on_same_plate: bool,
    /// Arrange.hpp:134
    pub avoid_extrusion_cali_region: bool,
    /// Arrange.hpp:135
    pub is_seq_print: bool,
    /// Arrange.hpp:136
    pub align_to_y_axis: bool,
    /// Arrange.hpp:137
    pub save_svg: bool,
    /// Arrange.hpp:138
    pub plate_has_tree_support: bool,
    /// Arrange.hpp:139
    pub brim_max: f64,
    /// Arrange.hpp:140
    pub bed_shrink_x: f32,
    /// Arrange.hpp:141
    pub bed_shrink_y: f32,
    /// Arrange.hpp:142
    pub brim_skirt_distance: f32,
    /// Arrange.hpp:143
    pub clearance_height_to_rod: f32,
    /// Arrange.hpp:144
    pub clearance_height_to_lid: f32,
    /// Arrange.hpp:145
    pub cleareance_radius: f32,
    /// Arrange.hpp:146
    pub nozzle_height: f32,
    /// Arrange.hpp:147 `float printable_height = 256.0;`
    pub printable_height: f32,
    /// Arrange.hpp:148 `Vec2d align_center{ 0.5,0.5 };`
    pub align_center: crate::geometry::Vec2d,

    /// regions cant't be used
    pub excluded_regions: ArrangePolygons,
    /// regions can be used but not prefered
    pub nonprefered_regions: ArrangePolygons,

    /// Progress indicator callback called when an object gets packed.
    /// The unsigned argument is the number of items remaining to pack.
    pub progressind: Option<Rc<dyn Fn(u32, &str)>>,

    /// Arrange.hpp:159
    pub on_packed: Option<Rc<dyn Fn(&ArrangePolygon)>>,

    /// A predicate returning true if abort is needed.
    pub stopcondition: Option<Rc<dyn Fn() -> bool>>,
}

impl Default for ArrangeParams {
    /// Arrange.hpp:164 `ArrangeParams() = default;` with the member initializers.
    fn default() -> Self {
        ArrangeParams {
            min_obj_distance: 0,
            min_inflation_floor: 0,
            accuracy: 1.0,
            parallel: true,
            allow_rotations: false,
            do_final_align: true,
            allow_multi_materials_on_same_plate: true,
            avoid_extrusion_cali_region: true,
            is_seq_print: false,
            align_to_y_axis: false,
            save_svg: false,
            plate_has_tree_support: false,
            brim_max: 0.0,
            bed_shrink_x: 0.0,
            bed_shrink_y: 0.0,
            brim_skirt_distance: 0.0,
            clearance_height_to_rod: 0.0,
            clearance_height_to_lid: 0.0,
            cleareance_radius: 0.0,
            nozzle_height: 0.0,
            printable_height: 256.0,
            align_center: crate::geometry::Vec2d::new(0.5, 0.5),
            excluded_regions: Vec::new(),
            nonprefered_regions: Vec::new(),
            // Arrange.hpp:154-156
            progressind: Some(Rc::new(|st: u32, str: &str| {
                log::debug!("st={}, {}", st, str);
            })),
            on_packed: None,
            stopcondition: None,
        }
    }
}

impl ArrangeParams {
    /// Arrange.hpp:165 `explicit ArrangeParams(coord_t md) : min_obj_distance(md) {}`
    pub fn with_min_obj_distance(md: Coord) -> Self {
        ArrangeParams { min_obj_distance: md, ..ArrangeParams::default() }
    }

    /// Arrange.hpp:166-184 `std::string to_json() const`: `std::to_string` of each value (a bool
    /// as the int it promotes to, a float as `%f`); the string is left open after the last comma,
    /// as the C++ leaves it.
    pub fn to_json(&self) -> String {
        let b = |v: bool| if v { "1" } else { "0" };
        let f = |v: f32| format!("{:.6}", v as f64);
        let mut ret = String::from("{");
        ret += &format!("\"min_obj_distance\":{},", self.min_obj_distance as i32);
        ret += &format!("\"accuracy\":{},", f(self.accuracy));
        ret += &format!("\"parallel\":{},", b(self.parallel));
        ret += &format!("\"allow_rotations\":{},", b(self.allow_rotations));
        ret += &format!("\"do_final_align\":{},", b(self.do_final_align));
        ret += &format!("\"allow_multi_materials_on_same_plate\":{},", b(self.allow_multi_materials_on_same_plate));
        ret += &format!("\"avoid_extrusion_cali_region\":{},", b(self.avoid_extrusion_cali_region));
        ret += &format!("\"is_seq_print\":{},", b(self.is_seq_print));
        ret += &format!("\"bed_shrink_x\":{},", f(self.bed_shrink_x));
        ret += &format!("\"bed_shrink_y\":{},", f(self.bed_shrink_y));
        ret += &format!("\"brim_skirt_distance\":{},", f(self.brim_skirt_distance));
        ret += &format!("\"clearance_height_to_rod\":{},", f(self.clearance_height_to_rod));
        ret += &format!("\"clearance_height_to_lid\":{},", f(self.clearance_height_to_lid));
        ret += &format!("\"cleareance_radius\":{},", f(self.cleareance_radius));
        ret += &format!("\"printable_height\":{},", f(self.printable_height));
        ret
    }
}

// ============================================================================
// PrintConfig.cpp:11558-11658 — the skirt distance the arrange reserves
// ============================================================================

/// PrintConfig.cpp:11558-11658: `has_skirt`, `skirt_expansion_params`,
/// `skirt_nozzle_diameter_for_filament`, `skirt_flow_from_config` and the three
/// `get_real_skirt_dist` overloads, which `update_arrange_params` calls
/// (Arrange.cpp:97). They are PrintConfig.cpp functions; the crate's `print_config.rs` does not
/// carry them, so they are ported here, beside their only caller.
pub mod print_config_skirt {
    //! The skirt helpers of PrintConfig.cpp (11558-11674) as this file has called them; since R947 they call the
    //! paired port in print_config.rs (`config_def`), verified against the pin and bit-equal to the copy that was
    //! here (tests/print_config_skirt_native_parity.rs, `arrange_copy_of_the_skirt_helpers_matches`).
    use crate::config::ConfigOptionResolver;
    use crate::flow::Flow;
    use crate::print_config::config_def as pc;

    /// PrintConfig.cpp:11558-11565 `bool has_skirt(const ConfigBase& cfg)`
    pub fn has_skirt(cfg: &(impl ConfigOptionResolver + ?Sized)) -> bool {
        pc::has_skirt(cfg)
    }

    /// PrintConfig.cpp:11566-11588 `static bool skirt_expansion_params(const ConfigBase &cfg, int &skirt_loops, float &skirt_distance)`
    pub fn skirt_expansion_params(
        cfg: &(impl ConfigOptionResolver + ?Sized),
        skirt_loops: &mut i32,
        skirt_distance: &mut f32,
    ) -> bool {
        pc::skirt_expansion_params(cfg, skirt_loops, skirt_distance)
    }

    /// PrintConfig.cpp:11590-11613 `static float skirt_nozzle_diameter_for_filament(const ConfigBase &cfg, unsigned int filament_id)`
    pub fn skirt_nozzle_diameter_for_filament(cfg: &(impl ConfigOptionResolver + ?Sized), filament_id: u32) -> f32 {
        pc::skirt_nozzle_diameter_for_filament(cfg, filament_id)
    }

    /// PrintConfig.cpp:11615-11635 `static Flow skirt_flow_from_config(const ConfigBase &cfg, unsigned int filament_id)`
    pub fn skirt_flow_from_config(cfg: &(impl ConfigOptionResolver + ?Sized), filament_id: u32) -> Flow {
        pc::skirt_flow_from_config(cfg, filament_id)
    }

    /// PrintConfig.cpp:11637-11647 `float get_real_skirt_dist(const ConfigBase& cfg, const Flow &flow)`
    pub fn get_real_skirt_dist(cfg: &(impl ConfigOptionResolver + ?Sized), flow: &Flow) -> f32 {
        pc::get_real_skirt_dist(cfg, flow)
    }

    /// The overloads of `get_real_skirt_dist` past the first.
    pub mod overloads {
        use super::*;

        /// PrintConfig.cpp:11649-11669 `float get_real_skirt_dist(const ConfigBase& cfg, const std::vector<unsigned int> &filament_ids)`
        pub fn get_real_skirt_dist(cfg: &(impl ConfigOptionResolver + ?Sized), filament_ids: &[u32]) -> f32 {
            pc::get_real_skirt_dist_overloads::get_real_skirt_dist(cfg, filament_ids)
        }

        /// The third overload.
        pub mod no_filaments {
            use super::super::*;

            /// PrintConfig.cpp:11671-11674 `float get_real_skirt_dist(const ConfigBase& cfg)`
            pub fn get_real_skirt_dist(cfg: &(impl ConfigOptionResolver + ?Sized)) -> f32 {
                pc::get_real_skirt_dist_overloads::no_filaments::get_real_skirt_dist(cfg)
            }
        }
    }
}

// ============================================================================
// Arrange.cpp:81-82 — constants
// ============================================================================

// Arrange.cpp:81  const double BIG_ITEM_TRESHOLD = 0.02;
// A coefficient used in separating bigger items and smaller items.
pub const BIG_ITEM_TRESHOLD: f64 = 0.02;

// Arrange.cpp:82  #define VITRIFY_TEMP_DIFF_THRSH 15
// bed temp can be higher than vitrify temp, but not higher than this thresh
pub const VITRIFY_TEMP_DIFF_THRSH: i32 = 15;

/// Print.hpp:45 `#define MAX_OUTER_NOZZLE_RADIUS 4`
const MAX_OUTER_NOZZLE_RADIUS: i32 = 4;

// ============================================================================
// Arrange.cpp:84-196 — arrange parameters from the print config
// ============================================================================

/// Arrange.cpp:84-121 `void update_arrange_params(ArrangeParams& params, const DynamicPrintConfig & print_cfg, const ArrangePolygons& selected)`
pub fn update_arrange_params(params: &mut ArrangeParams, print_cfg: &DynamicPrintConfig, selected: &ArrangePolygons) {
    // Note: skirt_distance is now defined between outermost brim and skirt, not the object and skirt.
    // So we can't do max but do adding instead.
    // Feed the filaments of the arranged items into the estimation so that it follows the same
    // "widest spacing wins" rule as Print::skirt_flow(), which drives both the real skirt and the
    // sequential print collision check.
    let mut used_filaments: Vec<u32> = Vec::new();
    for ap in selected {
        for (first, _) in &ap.extrude_id_filament_types {
            if *first > 0 {
                used_filaments.push(*first as u32);
            }
        }
    }
    // sort_remove_duplicates(used_filaments)
    used_filaments.sort();
    used_filaments.dedup();
    params.brim_skirt_distance = print_config_skirt::overloads::get_real_skirt_dist(print_cfg, &used_filaments);
    params.bed_shrink_x += params.brim_skirt_distance;
    params.bed_shrink_y += params.brim_skirt_distance;
    if params.is_seq_print {
        // set obj distance for auto seq_print
        let nozzle_height = params.nozzle_height;
        let all_objects_are_short = selected.iter().all(|ap| ap.height < nozzle_height as f64);
        if all_objects_are_short {
            params.min_obj_distance = params.min_obj_distance.max(scaled_f64(MAX_OUTER_NOZZLE_RADIUS as f64 + 0.001));
        } else {
            // +0.001mm to avoid clearance check fail due to rounding error
            params.min_obj_distance = params.min_obj_distance.max(scaled_f64(params.cleareance_radius as f64 + 0.001));
        }

        // Add per-object skirt expansion on top of clearance-based distance.
        let skirt_extra: f32 = params.brim_skirt_distance;
        if skirt_extra > 0.0 {
            // Two constraints:
            //   1) print-head clearance when nozzle is at skirt edge: gap >= clearance + skirt_extra
            //   2) prevent skirt line overlap on the bed:             gap >= 2 * skirt_extra
            // Ensuring base >= skirt_extra before adding it satisfies both:
            //   result = max(clearance, skirt_extra) + skirt_extra
            params.min_obj_distance = params.min_obj_distance.max(scaled_f64(skirt_extra as f64 + 0.001));
            params.min_obj_distance =
                (params.min_obj_distance as i32).wrapping_add(scaled_f64(skirt_extra as f64 + 0.001) as i32) as Coord;
        }
    }
}

/// Arrange.cpp:123-145 `void update_selected_items_inflation(ArrangePolygons& selected, const DynamicPrintConfig & print_cfg, ArrangeParams& params)`
pub fn update_selected_items_inflation(selected: &mut ArrangePolygons, print_cfg: &DynamicPrintConfig, params: &mut ArrangeParams) {
    // do not inflate brim_width. Objects are allowed to have overlapped brim.
    let bedpts: Points = get_shrink_bedpts(print_cfg, params);
    let _bedbb: BoundingBox = BoundingBox::from_points(&Polygon::from_points(bedpts).points);
    params.brim_max = 0.0;
    params.plate_has_tree_support = false;
    for ap in selected.iter() {
        params.brim_max = params.brim_max.max(ap.brim_width);
        if ap.has_tree_support {
            params.plate_has_tree_support = true;
        }
    }
    for ap in selected.iter_mut() {
        // 1. if user input a distance, use it
        // 2. if there is an object with tree support, all objects use the max tree branch radius (brim_max=branch diameter)
        // 3. otherwise, use each object's own brim width
        ap.inflation = if params.min_obj_distance != 0 {
            ((params.min_obj_distance as i32) / 2) as Coord
        } else if params.plate_has_tree_support {
            scaled_f64(params.brim_max / 2.0)
        } else {
            scaled_f64(ap.brim_width)
        };
        // STUDIO: callers may ask for a minimum visible gap through params.min_inflation_floor;
        // only in the automatic case (no explicit min_obj_distance). 0 = off.
        if params.min_obj_distance == 0 && params.min_inflation_floor > 0 {
            ap.inflation = ap.inflation.max(params.min_inflation_floor);
        }
    }
    params.brim_skirt_distance = params.brim_skirt_distance.max(params.brim_max as f32);
}

/// Arrange.cpp:147-168 `void update_unselected_items_inflation(ArrangePolygons& unselected, const DynamicPrintConfig & print_cfg, const ArrangeParams& params)`
pub fn update_unselected_items_inflation(
    unselected: &mut ArrangePolygons,
    _print_cfg: &DynamicPrintConfig,
    params: &ArrangeParams,
) {
    let mut exclusion_gap: Coord = scale_(1.0f32 as f64) as i32 as Coord;
    if params.is_seq_print {
        // bed_shrink_x is typically (-params.min_obj_distance / 2+5) for seq_print
        // +1mm gap so the exclusion region is not too close
        exclusion_gap = exclusion_gap.max(
            ((params.min_obj_distance as i32) / 2).wrapping_add(scaled_f32(params.bed_shrink_x + 1.0f32) as i32) as Coord,
        );
        // dont forget to move the excluded region
        for region in unselected.iter_mut() {
            if region.is_virt_object {
                // ExPolygon::translate(double x, double y): Point(coord_t(x), coord_t(y))
                region.poly.translate(Point::new(
                    scaled_f32(params.bed_shrink_x + 1.0f32),
                    scaled_f32(params.bed_shrink_y + 1.0f32),
                ));
            }
        }
    }
    // For occlusion regions, inflation should be larger to prevent generating brim on them.
    // However, extrusion cali regions are exceptional, since we can allow brim overlaps them.
    for ap in unselected.iter_mut() {
        ap.inflation = if !ap.is_virt_object {
            if params.min_obj_distance == 0 {
                scaled_f64(ap.brim_width)
            } else {
                ((params.min_obj_distance as i32) / 2) as Coord
            }
        } else if ap.is_wipe_tower && params.plate_has_tree_support {
            scaled_f64(params.brim_max / 2.0)
        } else if ap.is_extrusion_cali_object {
            0
        } else {
            exclusion_gap
        };
    }
}

/// Arrange.cpp:170-196 `Points get_shrink_bedpts(const DynamicPrintConfig & print_cfg, const ArrangeParams& params)`:
/// it will be accurate after call update_params.
pub fn get_shrink_bedpts(print_cfg: &DynamicPrintConfig, params: &ArrangeParams) -> Points {
    // PrintConfig.hpp:2062 `get_bed_shape(const DynamicPrintConfig &cfg, bool use_share = true)`
    let mut bedpts: Points = get_bed_shape(print_cfg, true);
    // shrink bed by moving to center by dist
    let shrink_fun = |bedpts: &mut Points, dist: f64, direction: usize| {
        // #define SGN(x) ((x) >= 0 ? 1 : -1)
        let sgn = |x: i32| -> i32 {
            if x >= 0 {
                1
            } else {
                -1
            }
        };
        let bb = BoundingBox::from_points(bedpts);
        // BoundingBox.cpp:232-235 `(this->min + this->max) / 2`, int32 vector arithmetic
        let center = [
            ((bb.min.x as i32).wrapping_add(bb.max.x as i32) / 2),
            ((bb.min.y as i32).wrapping_add(bb.max.y as i32) / 2),
        ];
        for pt in bedpts.iter_mut() {
            let c = if direction == 0 { pt.x as i32 } else { pt.y as i32 };
            // pt[direction] += dist * SGN(center[direction] - pt[direction]);
            let v = dist.mul_add(sgn(center[direction].wrapping_sub(c)) as f64, c as f64) as i32 as Coord;
            if direction == 0 {
                pt.x = v;
            } else {
                pt.y = v;
            }
        }
    };

    let mut params_copy = params.clone();
    if params.is_seq_print {
        // for sequential print, we need to inflate the bed because cleareance_radius is so large
        params_copy.bed_shrink_x =
            (params_copy.bed_shrink_x as f64 - unscale_(((params_copy.min_obj_distance as i32) / 2) as f64)) as f32;
        params_copy.bed_shrink_y =
            (params_copy.bed_shrink_y as f64 - unscale_(((params_copy.min_obj_distance as i32) / 2) as f64)) as f32;
    } else {
        // for non-seq print, we need to shrink the bed so the first layer of support won't exceed the bed
        params_copy.bed_shrink_x = params_copy.bed_shrink_x.max(params_copy.brim_skirt_distance / 2.0);
        params_copy.bed_shrink_y = params_copy.bed_shrink_y.max(params_copy.brim_skirt_distance / 2.0);
    }
    shrink_fun(&mut bedpts, scaled_f32(params_copy.bed_shrink_x) as f64, 0);
    shrink_fun(&mut bedpts, scaled_f32(params_copy.bed_shrink_y) as f64, 1);
    bedpts
}

// ============================================================================
// Arrange.cpp:198-283 — placer configuration and the overfit penalties
// ============================================================================

/// Arrange.cpp:198-246 `template<class PConf> void fill_config(PConf& pcfg, const ArrangeParams &params)`:
/// Fill in the placer algorithm configuration with values carefully chosen for Slic3r.
pub fn fill_config(pcfg: &mut libnest2d::NfpPConfig, params: &ArrangeParams) {
    if params.is_seq_print {
        // Start placing the items from the center of the print bed
        pcfg.starting_point = NfpPConfigAlignment::BottomLeft;
    } else {
        // Start placing the items from the center of the print bed
        pcfg.starting_point = NfpPConfigAlignment::TopRight;
    }

    if params.do_final_align {
        // Align the arranged pile into the center of the bin
        pcfg.alignment = NfpPConfigAlignment::Center;
    } else {
        pcfg.alignment = NfpPConfigAlignment::DontAlign;
    }

    // Try 4 angles (45 degree step) and find the one with min cost
    let pi = std::f64::consts::PI;
    if params.allow_rotations {
        pcfg.rotations = vec![0.0, pi / 4.0, pi / 2.0, 3.0 * pi / 4.0];
    } else {
        pcfg.rotations = vec![0.0];
    }

    // Point(double x, double y): coord_t(lrint(x)), coord_t(lrint(y)) (Point.hpp:179)
    pcfg.bed_shrink = Point::new(
        scale_(params.bed_shrink_x as f64).round_ties_even() as i32 as Coord,
        scale_(params.bed_shrink_y as f64).round_ties_even() as i32 as Coord,
    );

    // The accuracy of optimization.
    // Goes from 0.0 to 1.0 and scales performance as well
    pcfg.accuracy = params.accuracy;

    // Allow parallel execution.
    pcfg.parallel = params.parallel;
    pcfg.save_svg = params.save_svg;

    // BBS: excluded regions in BBS bed
    for poly in &params.excluded_regions {
        process_arrangeable(poly, &mut pcfg.m_excluded_regions);
    }
    // BBS: nonprefered regions in BBS bed
    for poly in &params.nonprefered_regions {
        process_arrangeable(poly, &mut pcfg.m_nonprefered_regions);
    }
    for itm in pcfg.m_excluded_regions.iter_mut() {
        itm.mark_as_fixed_in_bin(0);
        itm.inflate(scaled_f64(-2.0 * EPSILON));
    }
}

/// Arrange.cpp:248-258 `static double fixed_overfit(const std::tuple<double, Box>& result, const Box &binbb)`:
/// Apply penalty to object function result. This is used only when alignment after arrange is
/// explicitly disabled (PConfig::Alignment::DONT_ALIGN). Also, this will only work well for Box
/// shaped beds. (The penalty itself is commented out in the C++: the score is returned as is.)
pub fn fixed_overfit(result: &(f64, _Box), _binbb: &_Box) -> f64 {
    let score = result.0;
    //Box pilebb  = std::get<1>(result);
    //Box fullbb  = sl::boundingBox(pilebb, binbb);
    //auto diff = double(fullbb.area()) - binbb.area();
    //if(diff > 0) score += diff;

    score
}

/// Arrange.cpp:260-283 `static double fixed_overfit_topright_sliding(const std::tuple<double, Box> &result, const Box &binbb, const std::vector<Box> &excluded_boxes)`:
/// useful for arranging big circle objects.
pub fn fixed_overfit_topright_sliding(result: &(f64, _Box), binbb: &_Box, excluded_boxes: &[_Box]) -> f64 {
    let mut score = result.0;
    let mut pilebb = result.1;

    let shift_x = (binbb.max_corner().x() as i32).wrapping_sub(pilebb.max_corner().x() as i32);
    let shift_y = (binbb.max_corner().y() as i32).wrapping_sub(pilebb.max_corner().y() as i32);
    let shift_x = 0.max(shift_x); // do not allow left shift
    let shift_y = 0.max(shift_y); // do not allow bottom shift
    {
        let m = pilebb.min_corner_mut();
        *m = Point::new((m.x as i32).wrapping_add(shift_x) as Coord, (m.y as i32).wrapping_add(shift_y) as Coord);
    }
    {
        let m = pilebb.max_corner_mut();
        *m = Point::new((m.x as i32).wrapping_add(shift_x) as Coord, (m.y as i32).wrapping_add(shift_y) as Coord);
    }

    let fullbb = libnest2d::bounding_box_of_boxes(&pilebb, binbb);
    let diff = fullbb.area() as f64 - binbb.area() as f64;
    if diff > 0.0 {
        score += diff;
    }

    // excluded regions and nonprefered regions should not intersect the translated pilebb
    for bb in excluded_boxes {
        let area_ = pilebb.intersection(*bb).area();
        if area_ > 0 {
            score += area_ as f64;
        }
    }

    score
}

// ============================================================================
// Arrange.cpp:883-914 — item filtering and rotations
// ============================================================================

/// The bins `remove_large_items` is written for: `sl::isInside(shape, bin)` for a box
/// (geometry_traits.hpp:1184-1189) and a circle (geometry_traits.hpp:1149-1156). A polygon bin
/// (`boost::geometry::within`, boost_alg.hpp:378-382) is not ported.
pub trait NestBinInside {
    /// `sl::isInside(sh, bin)`
    fn is_inside_bin(&self, sh: &ExPolygon) -> bool;
}

impl NestBinInside for _Box {
    fn is_inside_bin(&self, sh: &ExPolygon) -> bool {
        libnest2d::is_inside_shape_box(sh, self)
    }
}

impl NestBinInside for _Circle {
    fn is_inside_bin(&self, sh: &ExPolygon) -> bool {
        libnest2d::is_inside_shape_circle(sh, self)
    }
}

/// Arrange.cpp:883-894 `template<class Bin> void remove_large_items(std::vector<Item> &items, Bin &&bin)`
pub fn remove_large_items<B: NestBinInside>(items: &mut Vec<_Item>, bin: &B) {
    let mut it = 0usize;
    while it != items.len() {
        //BBS: skip virtual object
        if !items[it].is_virt_object && !bin.is_inside_bin(&items[it].transformed_shape()) {
            items.remove(it);
        } else {
            it += 1;
        }
    }
}

/// Arrange.cpp:896-908 `template<class S> Radians min_area_boundingbox_rotation(const S &sh)`:
/// the calipers' minimum-area box angle, 0 when the rational arithmetic throws (min_area_boundingbox_rotation
/// may throw exception of dividing 0 if the object is already perfectly aligned to X).
pub fn min_area_boundingbox_rotation(sh: &ExPolygon) -> f64 {
    match libnest2d::min_area_bounding_box(sh) {
        Ok(rbox) => rbox.angle_to_x(),
        Err(e) => {
            log::error!("arranging min_area_boundingbox_rotation fails, msg={}", e);
            0.0
        }
    }
}

/// Arrange.cpp:910-914 `template<class S> Radians fit_into_box_rotation(const S &sh, const _Box<TPoint<S>> &box)`:
/// `fitIntoBoxRotation<S, TCompute<S>, boost::rational<LargeInt>>(sh, box)` with the default
/// `eps = 1e-4`. The C++ lets boost's `bad_rational` escape; it is the `Err` here.
pub fn fit_into_box_rotation(sh: &ExPolygon, bx: &_Box) -> Result<f64, libnest2d::BadRational> {
    libnest2d::fit_into_box_rotation(sh, bx, 1e-4)
}

// ============================================================================
// Arrange.cpp:1101-1104 — to_nestbin
// ============================================================================

/// Arrange.cpp:1101-1104 `to_nestbin`: the libnest2d bin of each bed type, one overload per bed.
pub trait ToNestbin {
    /// The libnest2d bin type.
    type Bin;
    /// `to_nestbin(bed)`
    fn to_nestbin(&self) -> Self::Bin;
}

impl ToNestbin for BoundingBox {
    type Bin = _Box;
    /// Arrange.cpp:1101 `inline Box to_nestbin(const BoundingBox &bb) { return Box{{bb.min(X), bb.min(Y)}, {bb.max(X), bb.max(Y)}};}`
    fn to_nestbin(&self) -> _Box {
        _Box::new(Point::new(self.min.x, self.min.y), Point::new(self.max.x, self.max.y))
    }
}

impl ToNestbin for CircleBed {
    type Bin = _Circle;
    /// Arrange.cpp:1102 `inline Circle to_nestbin(const CircleBed &c) { return Circle({c.center()(0), c.center()(1)}, c.radius()); }`
    fn to_nestbin(&self) -> _Circle {
        _Circle::new(Point::new(self.center().x, self.center().y), self.radius())
    }
}

impl ToNestbin for Polygon {
    type Bin = ExPolygon;
    /// Arrange.cpp:1103 `inline ExPolygon to_nestbin(const Polygon &p) { return ExPolygon{p}; }`
    fn to_nestbin(&self) -> ExPolygon {
        ExPolygon::new(self.clone())
    }
}

impl ToNestbin for InfiniteBed {
    type Bin = _Box;
    /// Arrange.cpp:1104 `inline Box to_nestbin(const InfiniteBed &bed) { return Box::infinite({bed.center.x(), bed.center.y()}); }`
    fn to_nestbin(&self) -> _Box {
        _Box::infinite(Point::new(self.center.x, self.center.y))
    }
}

// ============================================================================
// Arrange.cpp:1106-1140 — free helpers on BoundingBox and points
// ============================================================================

/// Arrange.cpp:1106 `inline coord_t width(const BoundingBox& box) { return box.max.x() - box.min.x(); }`
/// (int32 subtraction: `coord_t` is `int32_t`, libslic3r.h:40).
#[inline]
pub fn width(box_: &BoundingBox) -> Coord {
    (box_.max.x() as i32).wrapping_sub(box_.min.x() as i32) as Coord
}

/// Arrange.cpp:1107 `inline coord_t height(const BoundingBox& box) { return box.max.y() - box.min.y(); }`
#[inline]
pub fn height(box_: &BoundingBox) -> Coord {
    (box_.max.y() as i32).wrapping_sub(box_.min.y() as i32) as Coord
}

/// Arrange.cpp:1108 `inline double area(const BoundingBox& box) { return double(width(box)) * height(box); }`
#[inline]
pub fn area(box_: &BoundingBox) -> f64 {
    width(box_) as f64 * height(box_) as f64
}

/// Polygon.cpp:49-61 `double Polygon::area(const Points &points)`, the signed area, with
/// Point.hpp:86-89 `cross2(v1, v2) = v1.x() * v2.y() - v1.y() * v2.x()` contracted to
/// `fma(v1.x, v2.y, -(v1.y * v2.x))`; the sum `a += cross2(...)` is not fused (a call apart).
fn polygon_area_points(points: &Points) -> f64 {
    let mut a = 0.0_f64;
    if points.len() >= 3 {
        let back = points[points.len() - 1];
        let (mut p1x, mut p1y) = (back.x() as f64, back.y() as f64);
        for p in points {
            let (p2x, p2y) = (p.x() as f64, p.y() as f64);
            a += p1x.mul_add(p2y, -(p1y * p2x));
            p1x = p2x;
            p1y = p2y;
        }
    }
    0.5 * a
}

/// Arrange.cpp:1109 `inline double poly_area(const Points &pts) { return std::abs(Polygon::area(pts)); }`
#[inline]
pub fn poly_area(pts: &Points) -> f64 {
    polygon_area_points(pts).abs()
}

/// Arrange.cpp:1110-1115 `inline double distance_to(const Point& p1, const Point& p2)`:
/// int32 differences as doubles, `sqrt(dx*dx + dy*dy)` with the sum contracted to `fma(dx, dx, dy*dy)`.
#[inline]
pub fn distance_to(p1: &Point, p2: &Point) -> f64 {
    // Arrange.cpp:1112
    let dx = (p2.x() as i32).wrapping_sub(p1.x() as i32) as f64;
    // Arrange.cpp:1113
    let dy = (p2.y() as i32).wrapping_sub(p1.y() as i32) as f64;
    // Arrange.cpp:1114
    dx.mul_add(dx, dy * dy).sqrt()
}

/// Arrange.cpp:1117-1140 `static CircleBed to_circle(const Point &center, const Points& points)`
pub fn to_circle(center: &Point, points: &Points) -> CircleBed {
    // Arrange.cpp:1118
    let mut vertex_distances: Vec<f64> = Vec::new();
    // Arrange.cpp:1119
    let mut avg_dist: f64 = 0.0;

    // Arrange.cpp:1121-1126
    for pt in points.iter() {
        let distance = distance_to(center, pt);
        vertex_distances.push(distance);
        avg_dist += distance;
    }

    // Arrange.cpp:1128
    avg_dist /= vertex_distances.len() as f64;

    // Arrange.cpp:1130
    let mut ret = CircleBed::with_center_radius(*center, avg_dist);
    // Arrange.cpp:1131-1137
    for el in vertex_distances.iter().copied() {
        if (el - avg_dist).abs() > 10.0 * scaled_epsilon() {
            // Arrange.cpp:1134  ret = {};
            ret = CircleBed::new();
            break;
        }
    }

    // Arrange.cpp:1139
    ret
}

// ============================================================================
// Arrange.cpp:1142-1193 — items from arrangeables, the bed dispatch
// ============================================================================

/// Arrange.cpp:1142-1174 `static void process_arrangeable(const ArrangePolygon &arrpoly, std::vector<Item> &outp)`:
/// Create Item from Arrangeable.
pub fn process_arrangeable(arrpoly: &ArrangePolygon, outp: &mut Vec<_Item>) {
    let mut p: Polygon = arrpoly.poly.contour.clone();
    let offs = arrpoly.translation;
    let rotation = arrpoly.rotation;

    if p.is_counter_clockwise() {
        p.reverse();
    }

    if p.points.len() < 3 {
        return;
    }

    outp.push(_Item::from_contour(p, Vec::new()));
    let item = outp.last_mut().unwrap();
    item.set_rotation(rotation);
    item.set_translation(Point::new(offs.x, offs.y));
    item.set_bin_id(arrpoly.bed_idx);
    item.set_priority(arrpoly.priority);
    item.set_item_id(arrpoly.itemid);
    item.extrude_id_filament_types = arrpoly.extrude_id_filament_types.clone();
    item.height = arrpoly.height;
    item.name = arrpoly.name.clone();
    //BBS: add virtual object logic
    item.is_virt_object = arrpoly.is_virt_object;
    item.is_wipe_tower = arrpoly.is_wipe_tower;
    item.is_extrusion_cali_object = arrpoly.is_extrusion_cali_object;
    item.bed_temp = arrpoly.first_bed_temp as f64;
    item.print_temp = arrpoly.print_temp as f64;
    item.vitrify_temp = arrpoly.vitrify_temp as f64;
    item.set_inflation((arrpoly.inflation as i32).max(libnest2d::min_separation() as i32) as Coord);
    item.filament_temp_type = arrpoly.filament_temp_type;
}

/// The bed `call_with_bed` hands to its function: the C++ calls a generic lambda with an
/// `InfiniteBed`, a `BoundingBox`, a `CircleBed` or a `Polygon`; here it is one of these variants.
#[derive(Debug, Clone)]
pub enum ArrangeBed {
    /// `fn(InfiniteBed{...})`
    Infinite(InfiniteBed),
    /// `fn(bb)`
    Box(BoundingBox),
    /// `fn(circ)`
    Circle(CircleBed),
    /// `fn(Polygon(bed))`
    Polygon(Polygon),
}

/// Arrange.cpp:1176-1193 `template<class Fn> auto call_with_bed(const Points &bed, Fn &&fn)`:
/// an infinite bed for no or one point, else a box when the points fill their bounding box (to
/// 1e-3), a circle when they are equidistant from its center, a polygon otherwise.
pub fn call_with_bed<R>(bed: &Points, f: impl FnOnce(ArrangeBed) -> R) -> R {
    if bed.is_empty() {
        f(ArrangeBed::Infinite(InfiniteBed::default()))
    } else if bed.len() == 1 {
        f(ArrangeBed::Infinite(InfiniteBed::new(bed[0])))
    } else {
        let bb = BoundingBox::from_points(bed);
        let circ = to_circle(&bb.center(), bed);
        let parea = poly_area(bed);

        if (1.0 - parea / area(&bb)) < 1e-3 {
            f(ArrangeBed::Box(bb))
        } else if !circ.radius().is_nan() {
            f(ArrangeBed::Circle(circ))
        } else {
            f(ArrangeBed::Polygon(Polygon::from_points(bed.clone())))
        }
    }
}

#[allow(unused_imports)]
use crate::config::ConfigOptionResolver as _;

// Keep the imported-but-only-in-signatures names referenced.
#[allow(dead_code)]
fn _config_traits_in_scope(_: &dyn ConfigOption, _: ConfigOptionType) {}

#[cfg(test)]
mod tests {
    use super::libnest2d::*;
    use super::*;
    use crate::geometry::Point;

    #[test]
    fn test_bbox_width_height_area() {
        // Arrange.cpp:1106-1108
        let bb = BoundingBox::from_points_minmax(Point::new(0, 0), Point::new(10, 20));
        assert_eq!(width(&bb), 10);
        assert_eq!(height(&bb), 20);
        assert_eq!(super::area(&bb), 200.0);
    }

    #[test]
    fn test_distance_to() {
        // Arrange.cpp:1110-1115
        let a = Point::new(0, 0);
        let b = Point::new(3, 4);
        assert!((distance_to(&a, &b) - 5.0).abs() < 1e-9);
    }

    #[test]
    fn test_poly_area_square() {
        // Arrange.cpp:1109 — unsigned area of a 10x10 square is 100.
        let pts = vec![Point::new(0, 0), Point::new(10, 0), Point::new(10, 10), Point::new(0, 10)];
        assert!((poly_area(&pts) - 100.0).abs() < 1e-9);
    }

    #[test]
    fn test_to_circle_regular() {
        // Arrange.cpp:1117-1140 — points equidistant from center => a CircleBed
        // whose radius is that distance (no vertex exceeds 10*SCALED_EPSILON).
        let center = Point::new(0, 0);
        let r = 1_000_000;
        let pts = vec![Point::new(r, 0), Point::new(0, r), Point::new(-r, 0), Point::new(0, -r)];
        let c = to_circle(&center, &pts);
        assert!((c.radius() - r as f64).abs() < 1.0);
    }

    #[test]
    fn test_to_circle_non_circular() {
        // A very irregular point set yields a reset (NaN radius) CircleBed.
        let center = Point::new(0, 0);
        let pts = vec![Point::new(1_000_000, 0), Point::new(0, 10), Point::new(-50, 0), Point::new(0, -2_000_000)];
        let c = to_circle(&center, &pts);
        assert!(c.radius().is_nan());
    }

    #[test]
    fn test_rational_order_matches_values() {
        // boost::rational's continued-fraction order agrees with the exact order.
        let a = Rational::from_int(1).div_int(3).unwrap();
        let b = Rational::from_int(2).div_int(6).unwrap();
        let c = Rational::from_int(-7).div_int(2).unwrap();
        assert!(a.eq(&b));
        assert!(!a.lt(&b) && !b.lt(&a));
        assert!(c.lt(&a));
        assert!(c.lt_int(-3));
        assert!(!c.lt_int(-4));
        assert_eq!(Rational::from_int(5).div_int(0).unwrap_err(), BadRational);
    }

    #[test]
    fn test_box_infinite_and_center() {
        // geometry_traits.hpp:495-523
        let b = _Box::infinite(Point::new(0, 0));
        assert!(!b.defined);
        assert!(b.min_corner().x() < 0 && b.max_corner().x() > 0);
        let b2 = _Box::new(Point::new(-3, -5), Point::new(7, 9));
        assert_eq!(b2.center(), Point::new(-1 + 3, -2 + 4));
        assert_eq!(b2.area(), 10 * 14);
    }

    #[test]
    fn test_min_area_bbox_rotated_square() {
        // A square rotated by 30 degrees: the calipers find the 30 degree axis.
        let mut sq = ExPolygon::new(Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(1_000_000, 0),
            Point::new(1_000_000, 1_000_000),
            Point::new(0, 1_000_000),
        ]));
        libnest2d::rotate(&mut sq, 30f64.to_radians());
        let a = min_area_boundingbox_rotation(&sq);
        let deg = a.to_degrees().rem_euclid(90.0);
        assert!((deg - 60.0).abs() < 0.01 || (deg - 30.0).abs() < 0.01, "{deg}");
    }

    #[test]
    fn test_call_with_bed_dispatch() {
        let r = 1_000_000;
        let square = vec![Point::new(0, 0), Point::new(r, 0), Point::new(r, r), Point::new(0, r)];
        assert!(matches!(call_with_bed(&square, |b| b), ArrangeBed::Box(_)));
        assert!(matches!(call_with_bed(&vec![], |b| b), ArrangeBed::Infinite(_)));
        // Arrange.cpp:1117-1139, 1184-1190: the corners of a right isosceles triangle are all at the
        // same distance from the centre of its bounding box, so `to_circle` takes it for a circle.
        let tri = vec![Point::new(0, 0), Point::new(r, 0), Point::new(0, r)];
        assert!(matches!(call_with_bed(&tri, |b| b), ArrangeBed::Circle(_)));
        let scalene = vec![Point::new(0, 0), Point::new(r, 0), Point::new(r / 4, r)];
        assert!(matches!(call_with_bed(&scalene, |b| b), ArrangeBed::Polygon(_)));
    }

    #[test]
    fn test_process_arrangeable_orientation_and_inflation() {
        let mut ap = ArrangePolygon::default();
        ap.poly = ExPolygon::new(Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(1_000_000, 0),
            Point::new(1_000_000, 1_000_000),
            Point::new(0, 1_000_000),
        ]));
        let mut out = Vec::new();
        process_arrangeable(&ap, &mut out);
        assert_eq!(out.len(), 1);
        // the contour is made clockwise
        assert!(!out[0].raw_shape().contour.is_counter_clockwise());
        // MIN_SEPARATION = scale_(1.0) truncated: 99999
        assert_eq!(out[0].inflation(), 99_999);
    }
}
