//! `include/clipper2/clipper.core.h` of Clipper2 as the engine pins it (version 1.5.2,
//! `src/clipper2/Clipper2Lib`): the structures and functions the clipping engine is built
//! on, for `Point64`. Compiled twice, see `mod.rs`; `super::clipper2_build` is the build
//! file that stands for `USINGZ`.
//!
//! The reference is built by Apple clang for arm64, where `-ffp-contract=on` is the default:
//! inside one expression a product that feeds a sum or a difference is fused. Each such
//! place is a `mul_add` here, and nothing else is: a product kept in a variable or returned
//! by a function (`Sqr`) is rounded first, as in the C++.
//!
//! The templates on `PointD` (`ScalePath`, `ScalePaths`, `TransformPath`, `StripNearEqual`,
//! `TranslatePoint`, `ReflectPoint`, `CheckPrecisionRange`, the two-vector `CrossProduct`
//! and `DotProduct`, `ScaleRect`, `RectD`) serve `ClipperD`, clipper.offset.cpp and
//! clipper.rectclip.cpp, and come with those.

use std::cell::Cell;

pub use super::clipper2_build::{Point64, ZCallback64, USINGZ};

// clipper.core.h:51-56 error codes (2^n)
pub const PRECISION_ERROR_I: i32 = 1; // non-fatal
pub const SCALE_ERROR_I: i32 = 2; // non-fatal
pub const NON_PAIR_ERROR_I: i32 = 4; // non-fatal
pub const UNDEFINED_ERROR_I: i32 = 32; // fatal
pub const RANGE_ERROR_I: i32 = 64;

/// clipper.core.h:59
pub const PI: f64 = 3.141592653589793238;

/// clipper.core.h:65 (`CLIPPER2_MAX_DECIMAL_PRECISION` is not defined by the engine)
pub const CLIPPER2_MAX_DEC_PRECISION: i32 = 8;

/// clipper.core.h:68
pub const MAX_COORD: i64 = i64::MAX >> 2;
/// clipper.core.h:69
pub const MIN_COORD: i64 = -MAX_COORD;
/// clipper.core.h:70
pub const INVALID: i64 = i64::MAX;
/// clipper.core.h:74
pub const MAX_DBL: f64 = f64::MAX;

/// clipper.core.h:111
/// C++: enum class FillRule { EvenOdd, NonZero, Positive, Negative };
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FillRule {
    EvenOdd,
    NonZero,
    Positive,
    Negative,
}

/// clipper.core.h:216-224
/// C++: friend bool operator==(const Point& a, const Point& b) { return a.x == b.x && a.y == b.y; }
/// In both builds: with `USINGZ` two points at one place with different z are equal.
impl PartialEq for Point64 {
    #[inline]
    fn eq(&self, b: &Point64) -> bool {
        self.x == b.x && self.y == b.y
    }
}

impl Eq for Point64 {}

/// clipper.core.h:226-229
/// C++: inline Point<T> operator-() const { return Point<T>(-x, -y); }
impl std::ops::Neg for Point64 {
    type Output = Point64;
    fn neg(self) -> Point64 {
        Point64::new(-self.x, -self.y)
    }
}

/// clipper.core.h:231-234
/// C++: inline Point operator+(const Point& b) const { return Point(x + b.x, y + b.y); }
impl std::ops::Add for Point64 {
    type Output = Point64;
    fn add(self, b: Point64) -> Point64 {
        Point64::new(self.x + b.x, self.y + b.y)
    }
}

/// clipper.core.h:236-239
/// C++: inline Point operator-(const Point& b) const { return Point(x - b.x, y - b.y); }
impl std::ops::Sub for Point64 {
    type Output = Point64;
    fn sub(self, b: Point64) -> Point64 {
        Point64::new(self.x - b.x, self.y - b.y)
    }
}

impl Point64 {
    /// clipper.core.h:241
    /// C++: inline void Negate() { x = -x; y = -y; }
    pub fn negate(&mut self) {
        self.x = -self.x;
        self.y = -self.y;
    }
}

/// clipper.core.h:254 `using Path64 = Path<int64_t>;`
pub type Path64 = Vec<Point64>;
/// clipper.core.h:256 `using Paths64 = std::vector< Path64>;`
pub type Paths64 = Vec<Path64>;

/// clipper.core.h:259-261
pub const INVALID_POINT64: Point64 = Point64::new(i64::MAX, i64::MAX);

/// clipper.core.h:266-273
/// C++: static inline Point<T> MidPoint(const Point<T>& p1, const Point<T>& p2)
pub fn mid_point(p1: &Point64, p2: &Point64) -> Point64 {
    // `Point<T> result;` is the point with z = 0.
    Point64::new((p1.x + p2.x) / 2, (p1.y + p2.y) / 2)
}

/// clipper.core.h:283-392 `template <typename T> struct Rect` with `T = int64_t`
/// (`using Rect64 = Rect<int64_t>`, clipper.core.h:280).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Rect64 {
    pub left: i64,
    pub top: i64,
    pub right: i64,
    pub bottom: i64,
}

impl Default for Rect64 {
    /// clipper.core.h:296-301 `Rect(bool is_valid = true)` with the default argument.
    fn default() -> Self {
        Rect64::new_valid(true)
    }
}

impl Rect64 {
    /// clipper.core.h:290-294
    /// C++: Rect(T l, T t, T r, T b)
    pub const fn new(l: i64, t: i64, r: i64, b: i64) -> Self {
        Rect64 { left: l, top: t, right: r, bottom: b }
    }

    /// clipper.core.h:296-307
    /// C++: Rect(bool is_valid = true)
    pub const fn new_valid(is_valid: bool) -> Self {
        if is_valid {
            Rect64 { left: 0, top: 0, right: 0, bottom: 0 }
        } else {
            Rect64 { left: i64::MAX, top: i64::MAX, right: i64::MIN, bottom: i64::MIN }
        }
    }

    /// clipper.core.h:309-316
    /// C++: static Rect<T> InvalidRect()
    pub const fn invalid_rect() -> Self {
        Rect64 { left: i64::MAX, top: i64::MAX, right: i64::MIN, bottom: i64::MIN }
    }

    /// clipper.core.h:318
    /// C++: bool IsValid() const { return left != (std::numeric_limits<T>::max)(); }
    pub fn is_valid(&self) -> bool {
        self.left != i64::MAX
    }

    /// clipper.core.h:320
    /// C++: T Width() const { return right - left; }
    pub fn width(&self) -> i64 {
        self.right - self.left
    }

    /// clipper.core.h:321
    /// C++: T Height() const { return bottom - top; }
    pub fn height(&self) -> i64 {
        self.bottom - self.top
    }

    /// clipper.core.h:322
    /// C++: void Width(T width) { right = left + width; }
    pub fn set_width(&mut self, width: i64) {
        self.right = self.left + width;
    }

    /// clipper.core.h:323
    /// C++: void Height(T height) { bottom = top + height; }
    pub fn set_height(&mut self, height: i64) {
        self.bottom = self.top + height;
    }

    /// clipper.core.h:325-328
    /// C++: Point<T> MidPoint() const
    pub fn mid_point(&self) -> Point64 {
        Point64::new((self.left + self.right) / 2, (self.top + self.bottom) / 2)
    }

    /// clipper.core.h:330-339
    /// C++: Path<T> AsPath() const
    pub fn as_path(&self) -> Path64 {
        let mut result = Path64::with_capacity(4);
        result.push(Point64::new(self.left, self.top));
        result.push(Point64::new(self.right, self.top));
        result.push(Point64::new(self.right, self.bottom));
        result.push(Point64::new(self.left, self.bottom));
        result
    }

    /// clipper.core.h:341-344
    /// C++: bool Contains(const Point<T>& pt) const
    pub fn contains_point(&self, pt: &Point64) -> bool {
        pt.x > self.left && pt.x < self.right && pt.y > self.top && pt.y < self.bottom
    }

    /// clipper.core.h:346-350
    /// C++: bool Contains(const Rect<T>& rec) const
    pub fn contains(&self, rec: &Rect64) -> bool {
        rec.left >= self.left && rec.right <= self.right && rec.top >= self.top && rec.bottom <= self.bottom
    }

    /// clipper.core.h:359
    /// C++: bool IsEmpty() const { return bottom <= top || right <= left; };
    pub fn is_empty(&self) -> bool {
        self.bottom <= self.top || self.right <= self.left
    }

    /// clipper.core.h:361-365
    /// C++: bool Intersects(const Rect<T>& rec) const
    pub fn intersects(&self, rec: &Rect64) -> bool {
        (self.left.max(rec.left) <= self.right.min(rec.right)) && (self.top.max(rec.top) <= self.bottom.min(rec.bottom))
    }

    /// clipper.core.h:372-379
    /// C++: Rect<T>& operator+=(const Rect<T>& other)
    pub fn add_assign(&mut self, other: &Rect64) {
        self.left = self.left.min(other.left);
        self.top = self.top.min(other.top);
        self.right = self.right.max(other.right);
        self.bottom = self.bottom.max(other.bottom);
    }

    /// clipper.core.h:381-386
    /// C++: Rect<T> operator+(const Rect<T>& other) const
    pub fn add(&self, other: &Rect64) -> Rect64 {
        let mut result = *self;
        result.add_assign(other);
        result
    }
}

/// clipper.core.h:417
pub const INVALID_RECT64: Rect64 = Rect64::invalid_rect();

/// clipper.core.h:420-435
/// C++: Rect<T> GetBounds(const Path<T>& path)
pub fn get_bounds(path: &Path64) -> Rect64 {
    let mut xmin = i64::MAX;
    let mut ymin = i64::MAX;
    let mut xmax = i64::MIN;
    let mut ymax = i64::MIN;
    for p in path {
        if p.x < xmin {
            xmin = p.x;
        }
        if p.x > xmax {
            xmax = p.x;
        }
        if p.y < ymin {
            ymin = p.y;
        }
        if p.y > ymax {
            ymax = p.y;
        }
    }
    Rect64::new(xmin, ymin, xmax, ymax)
}

/// clipper.core.h:437-453
/// C++: Rect<T> GetBounds(const Paths<T>& paths)
pub fn get_bounds_paths(paths: &Paths64) -> Rect64 {
    let mut xmin = i64::MAX;
    let mut ymin = i64::MAX;
    let mut xmax = i64::MIN;
    let mut ymax = i64::MIN;
    for path in paths {
        for p in path {
            if p.x < xmin {
                xmin = p.x;
            }
            if p.x > xmax {
                xmax = p.x;
            }
            if p.y < ymin {
                ymin = p.y;
            }
            if p.y > ymax {
                ymax = p.y;
            }
        }
    }
    Rect64::new(xmin, ymin, xmax, ymax)
}

/// clipper.core.h:599-603
/// C++: inline double Sqr(T val)
/// A function: its product is rounded before the caller uses it.
#[inline]
pub fn sqr(val: f64) -> f64 {
    val * val
}

/// clipper.core.h:650-657
/// C++: inline void StripDuplicates( Path<T>& path, bool is_closed_path)
pub fn strip_duplicates(path: &mut Path64, is_closed_path: bool) {
    // std::unique keeps the first of each run of equal points.
    path.dedup_by(|b, a| *a == *b);
    if is_closed_path {
        while path.len() > 1 && path[path.len() - 1] == path[0] {
            path.pop();
        }
    }
}

/// clipper.core.h:659-667
/// C++: inline void StripDuplicates( Paths<T>& paths, bool is_closed_path)
pub fn strip_duplicates_paths(paths: &mut Paths64, is_closed_path: bool) {
    for path in paths.iter_mut() {
        strip_duplicates(path, is_closed_path);
    }
}

/// clipper.core.h:686-689
/// C++: inline int TriSign(int64_t x) // returns 0, 1 or -1
pub fn tri_sign(x: i64) -> i32 {
    (x > 0) as i32 - (x < 0) as i32
}

/// clipper.core.h:691-700
/// C++: struct MultiplyUInt64Result
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct MultiplyUInt64Result {
    pub result: u64,
    pub carry: u64,
}

/// clipper.core.h:702-714
/// C++: inline MultiplyUInt64Result Multiply(uint64_t a, uint64_t b) // #834, #835
/// The 64 x 64 -> 128 bit product in two halves. `ProductsAreEqual` uses it where the
/// compiler has no `__int128`; unsigned arithmetic wraps in the C++, so it does here.
pub fn multiply(a: u64, b: u64) -> MultiplyUInt64Result {
    let lo = |x: u64| x & 0xFFFF_FFFF;
    let hi = |x: u64| x >> 32;

    let x1 = lo(a).wrapping_mul(lo(b));
    let x2 = hi(a).wrapping_mul(lo(b)).wrapping_add(hi(x1));
    let x3 = lo(a).wrapping_mul(hi(b)).wrapping_add(lo(x2));
    let result = lo(x3) << 32 | lo(x1);
    let carry = hi(a).wrapping_mul(hi(b)).wrapping_add(hi(x2)).wrapping_add(hi(x3));

    MultiplyUInt64Result { result, carry }
}

thread_local! {
    /// How often `products_are_equal` had a product that does not fit 64 bits, on this
    /// thread. A diagnostic the parity tests read; it changes nothing.
    static PRODUCTS_128_COUNT: Cell<u64> = const { Cell::new(0) };
}

/// The number of `products_are_equal` calls of this thread in which a product needed more
/// than 64 bits.
pub fn products_128_count() -> u64 {
    PRODUCTS_128_COUNT.with(|c| c.get())
}

/// clipper.core.h:716-739
/// C++: inline bool ProductsAreEqual(int64_t a, int64_t b, int64_t c, int64_t d)
/// returns true if (and only if) a * b == c * d
/// The reference takes the first arm (`__clang__`, 64 bit pointers): `__int128_t`. The
/// other arm (`Multiply` on the absolute values, and the signs) gives the same answer.
pub fn products_are_equal(a: i64, b: i64, c: i64, d: i64) -> bool {
    let ab = (a as i128) * (b as i128);
    let cd = (c as i128) * (d as i128);
    if a.checked_mul(b).is_none() || c.checked_mul(d).is_none() {
        PRODUCTS_128_COUNT.with(|n| n.set(n.get() + 1));
    }
    ab == cd
}

/// clipper.core.h:741-752
/// C++: inline bool IsCollinear(const Point<T>& pt1, const Point<T>& sharedPt, const Point<T>& pt2) // #777
pub fn is_collinear(pt1: &Point64, shared_pt: &Point64, pt2: &Point64) -> bool {
    let a = shared_pt.x - pt1.x;
    let b = pt2.y - shared_pt.y;
    let c = shared_pt.y - pt1.y;
    let d = pt2.x - shared_pt.x;
    // When checking for collinearity with very large coordinate values
    // then ProductsAreEqual is more accurate than using CrossProduct.
    products_are_equal(a, b, c, d)
}

/// clipper.core.h:755-759
/// C++: inline double CrossProduct(const Point<T>& pt1, const Point<T>& pt2, const Point<T>& pt3)
/// `a * b - c * d` in one expression: the left product is fused, `fma(a, b, -(c * d))`.
#[inline]
pub fn cross_product(pt1: &Point64, pt2: &Point64, pt3: &Point64) -> f64 {
    ((pt2.x - pt1.x) as f64).mul_add((pt3.y - pt2.y) as f64, -(((pt2.y - pt1.y) as f64) * ((pt3.x - pt2.x) as f64)))
}

/// clipper.core.h:767-771
/// C++: inline double DotProduct(const Point<T>& pt1, const Point<T>& pt2, const Point<T>& pt3)
/// `a * b + c * d` in one expression: `fma(a, b, c * d)`.
#[inline]
pub fn dot_product(pt1: &Point64, pt2: &Point64, pt3: &Point64) -> f64 {
    ((pt2.x - pt1.x) as f64).mul_add((pt3.x - pt2.x) as f64, ((pt2.y - pt1.y) as f64) * ((pt3.y - pt2.y) as f64))
}

/// clipper.core.h:779-783
/// C++: inline double DistanceSqr(const Point<T> pt1, const Point<T> pt2)
pub fn distance_sqr(pt1: Point64, pt2: Point64) -> f64 {
    sqr((pt1.x - pt2.x) as f64) + sqr((pt1.y - pt2.y) as f64)
}

/// clipper.core.h:785-797
/// C++: inline double PerpendicDistFromLineSqrd(const Point<T>& pt, const Point<T>& line1, const Point<T>& line2)
pub fn perpendic_dist_from_line_sqrd(pt: &Point64, line1: &Point64, line2: &Point64) -> f64 {
    //perpendicular distance of point (x³,y³) = (Ax³ + By³ + C)/Sqrt(A² + B²)
    //see https://en.wikipedia.org/wiki/Perpendicular_distance
    let a = (pt.x - line1.x) as f64;
    let b = (pt.y - line1.y) as f64;
    let c = (line2.x - line1.x) as f64;
    let d = (line2.y - line1.y) as f64;
    if c == 0.0 && d == 0.0 {
        return 0.0;
    }
    // `Sqr(a * d - c * b) / (c * c + d * d)`: fma(a, d, -(c * b)) and fma(c, c, d * d).
    sqr(a.mul_add(d, -(c * b))) / c.mul_add(c, d * d)
}

/// clipper.core.h:799-817
/// C++: inline double Area(const Path<T>& path)
pub fn area(path: &Path64) -> f64 {
    let cnt = path.len();
    if cnt < 3 {
        return 0.0;
    }
    let mut a = 0.0f64;
    let mut it2 = cnt - 1;
    let mut stop = it2;
    if cnt & 1 == 0 {
        stop += 1;
    }
    let mut it1 = 0usize;
    while it1 != stop {
        // `a += double(y sum) * (x difference)`: fma(y sum, x difference, a).
        a = ((path[it2].y + path[it1].y) as f64).mul_add((path[it2].x - path[it1].x) as f64, a);
        it2 = it1 + 1;
        a = ((path[it1].y + path[it2].y) as f64).mul_add((path[it1].x - path[it2].x) as f64, a);
        it1 += 2;
    }
    if cnt & 1 != 0 {
        a = ((path[it2].y + path[it1].y) as f64).mul_add((path[it2].x - path[it1].x) as f64, a);
    }
    a * 0.5
}

/// clipper.core.h:819-829
/// C++: inline double Area(const Paths<T>& paths)
pub fn area_paths(paths: &Paths64) -> f64 {
    let mut a = 0.0;
    for path in paths {
        a += area(path);
    }
    a
}

/// clipper.core.h:831-839
/// C++: inline bool IsPositive(const Path<T>& poly)
pub fn is_positive(poly: &Path64) -> bool {
    // A curve has positive orientation [and area] if a region 'R'
    // is on the left when traveling around the outside of 'R'.
    //https://mathworld.wolfram.com/CurveOrientation.html
    //nb: This statement is premised on using Cartesian coordinates
    area(poly) >= 0.0
}

/// clipper.core.h:897-918
/// C++: inline bool GetSegmentIntersectPt(const Point<T>& ln1a, const Point<T>& ln1b, const Point<T>& ln2a, const Point<T>& ln2b, Point<T>& ip)
/// The `#else` arm: `CLIPPER2_HI_PRECISION` is not defined by the engine. (Later versions
/// of the library call this function `GetLineIntersectPt`.)
/// With `USINGZ`, `ip = ln1a` and `ip = ln1b` copy the z of that point; the last arm
/// leaves the z of `ip` as it was.
pub fn get_segment_intersect_pt(ln1a: &Point64, ln1b: &Point64, ln2a: &Point64, ln2b: &Point64, ip: &mut Point64) -> bool {
    // https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
    let dx1 = (ln1b.x - ln1a.x) as f64;
    let dy1 = (ln1b.y - ln1a.y) as f64;
    let dx2 = (ln2b.x - ln2a.x) as f64;
    let dy2 = (ln2b.y - ln2a.y) as f64;

    // `dy1 * dx2 - dy2 * dx1`: fma(dy1, dx2, -(dy2 * dx1)).
    let det = dy1.mul_add(dx2, -(dy2 * dx1));
    if det == 0.0 {
        return false;
    }
    // `((ln1a.x - ln2a.x) * dy2 - (ln1a.y - ln2a.y) * dx2) / det`: the numerator is
    // fma(x difference, dy2, -(y difference * dx2)).
    let t = ((ln1a.x - ln2a.x) as f64).mul_add(dy2, -(((ln1a.y - ln2a.y) as f64) * dx2)) / det;
    if t <= 0.0 {
        *ip = *ln1a;
    } else if t >= 1.0 {
        *ip = *ln1b;
    } else {
        // `static_cast<T>(ln1a.x + t * dx1)`: fma(t, dx1, ln1a.x), truncated.
        ip.x = t.mul_add(dx1, ln1a.x as f64) as i64;
        ip.y = t.mul_add(dy1, ln1a.y as f64) as i64;
    }
    true
}

/// clipper.core.h:942-947
/// C++: inline int GetSign(const T& val)
#[inline]
pub fn get_sign(val: f64) -> i32 {
    if val == 0.0 {
        return 0;
    }
    if val > 0.0 {
        1
    } else {
        -1
    }
}

/// clipper.core.h:949-968
/// C++: inline bool SegmentsIntersect(const Point64& seg1a, const Point64& seg1b, const Point64& seg2a, const Point64& seg2b, bool inclusive = false)
pub fn segments_intersect(seg1a: &Point64, seg1b: &Point64, seg2a: &Point64, seg2b: &Point64, inclusive: bool) -> bool {
    if inclusive {
        let res1 = cross_product(seg1a, seg2a, seg2b);
        let res2 = cross_product(seg1b, seg2a, seg2b);
        if res1 * res2 > 0.0 {
            return false;
        }
        let res3 = cross_product(seg2a, seg1a, seg1b);
        let res4 = cross_product(seg2b, seg1a, seg1b);
        if res3 * res4 > 0.0 {
            return false;
        }
        res1 != 0.0 || res2 != 0.0 || res3 != 0.0 || res4 != 0.0 // ensures not collinear
    } else {
        (get_sign(cross_product(seg1a, seg2a, seg2b)) * get_sign(cross_product(seg1b, seg2a, seg2b)) < 0)
            && (get_sign(cross_product(seg2a, seg1a, seg1b)) * get_sign(cross_product(seg2b, seg1a, seg1b)) < 0)
    }
}

/// clipper.core.h:970-990
/// C++: inline Point<T> GetClosestPointOnSegment(const Point<T>& offPt, const Point<T>& seg1, const Point<T>& seg2)
/// With `USINGZ` the result is `seg1` with its z where the segment is a point, and a new
/// point with z = 0 otherwise.
pub fn get_closest_point_on_segment(off_pt: &Point64, seg1: &Point64, seg2: &Point64) -> Point64 {
    if seg1.x == seg2.x && seg1.y == seg2.y {
        return *seg1;
    }
    let dx = (seg2.x - seg1.x) as f64;
    let dy = (seg2.y - seg1.y) as f64;
    // The numerator `a * dx + b * dy` is fma(a, dx, b * dy); `Sqr(dx) + Sqr(dy)` adds the
    // results of two calls.
    let mut q = ((off_pt.x - seg1.x) as f64).mul_add(dx, ((off_pt.y - seg1.y) as f64) * dy) / (sqr(dx) + sqr(dy));
    if q < 0.0 {
        q = 0.0;
    } else if q > 1.0 {
        q = 1.0;
    }
    // nearbyint: to the nearest integer, halves to even (the default rounding mode).
    Point64::new(seg1.x + (q * dx).round_ties_even() as i64, seg1.y + (q * dy).round_ties_even() as i64)
}

/// clipper.core.h:992
/// C++: enum class PointInPolygonResult { IsOn, IsInside, IsOutside };
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PointInPolygonResult {
    IsOn,
    IsInside,
    IsOutside,
}

/// clipper.core.h:994-1076
/// C++: inline PointInPolygonResult PointInPolygon(const Point<T>& pt, const Path<T>& polygon)
/// The iterators are indices: `cbegin` is 0, `polygon.cend()` is `polygon.len()`.
pub fn point_in_polygon(pt: &Point64, polygon: &Path64) -> PointInPolygonResult {
    if polygon.len() < 3 {
        return PointInPolygonResult::IsOutside;
    }

    let mut val = 0;
    let cbegin = 0usize;
    let mut first = cbegin;
    let mut curr;
    let mut prev;
    let mut cend = polygon.len();

    while first != cend && polygon[first].y == pt.y {
        first += 1;
    }
    if first == cend {
        // not a proper polygon
        return PointInPolygonResult::IsOutside;
    }

    let mut is_above = polygon[first].y < pt.y;
    let starting_above = is_above;
    curr = first + 1;
    loop {
        if curr == cend {
            if cend == first || first == cbegin {
                break;
            }
            cend = first;
            curr = cbegin;
        }

        if is_above {
            while curr != cend && polygon[curr].y < pt.y {
                curr += 1;
            }
            if curr == cend {
                continue;
            }
        } else {
            while curr != cend && polygon[curr].y > pt.y {
                curr += 1;
            }
            if curr == cend {
                continue;
            }
        }

        if curr == cbegin {
            prev = polygon.len() - 1; //nb: NOT cend (since might equal first)
        } else {
            prev = curr - 1;
        }

        if polygon[curr].y == pt.y {
            if polygon[curr].x == pt.x
                || (polygon[curr].y == polygon[prev].y && ((pt.x < polygon[prev].x) != (pt.x < polygon[curr].x)))
            {
                return PointInPolygonResult::IsOn;
            }
            curr += 1;
            if curr == first {
                break;
            }
            continue;
        }

        if pt.x < polygon[curr].x && pt.x < polygon[prev].x {
            // we're only interested in edges crossing on the left
        } else if pt.x > polygon[prev].x && pt.x > polygon[curr].x {
            val = 1 - val; // toggle val
        } else {
            let d = cross_product(&polygon[prev], &polygon[curr], pt);
            if d == 0.0 {
                return PointInPolygonResult::IsOn;
            }
            if (d < 0.0) == is_above {
                val = 1 - val;
            }
        }
        is_above = !is_above;
        curr += 1;
    }

    if is_above != starting_above {
        cend = polygon.len();
        if curr == cend {
            curr = cbegin;
        }
        if curr == cbegin {
            prev = cend - 1;
        } else {
            prev = curr - 1;
        }
        let d = cross_product(&polygon[prev], &polygon[curr], pt);
        if d == 0.0 {
            return PointInPolygonResult::IsOn;
        }
        if (d < 0.0) == is_above {
            val = 1 - val;
        }
    }

    if val == 0 {
        PointInPolygonResult::IsOutside
    } else {
        PointInPolygonResult::IsInside
    }
}
