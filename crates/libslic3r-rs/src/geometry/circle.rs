//! Circle geometry utilities.
//!
//! Faithful 1:1 port of BambuStudio's `src/libslic3r/Geometry/Circle.hpp` and
//! `src/libslic3r/Geometry/Circle.cpp`.
//!
//! The C++ code is heavily templated over the vector type (`Vec2f`, `Vec2d`).
//! In this crate the working vector type is `Vec2d` (= [`PointF`]), matching the
//! `Circled` / `CircleSqd` instantiations that the rest of libslic3r actually
//! uses, so the monomorphised `Vec2d` flavour is ported directly.

use crate::geometry::{Point, Vec2d};
use crate::libslic3r::{EPSILON, SCALED_EPSILON};
use crate::CoordF;

/// Vector of `Vec2d`, matching BambuStudio's `Vec2ds`.
/// Point.hpp (`using Vec2ds = std::vector<Vec2d>;`)
pub type Vec2ds = Vec<Vec2d>;

// https://en.wikipedia.org/wiki/Circumscribed_circle
// Circumcenter coordinates, Cartesian coordinates
// Circle.hpp:12-31
//
// template<typename Vector>
// Vector circle_center(const Vector &a, const Vector &bsrc, const Vector &csrc, typename Vector::Scalar epsilon)
pub fn circle_center(a: Vec2d, bsrc: Vec2d, csrc: Vec2d, epsilon: CoordF) -> Vec2d {
    // Circle.hpp:16-17
    let b = bsrc - a;
    let c = csrc - a;
    // Circle.hpp:18-19
    let lb = b.x() * b.x() + b.y() * b.y(); // b.squaredNorm()
    let lc = c.x() * c.x() + c.y() * c.y(); // c.squaredNorm()
                                            // Circle.hpp:20
    let d = b.x() * c.y() - b.y() * c.x();
    if d.abs() < epsilon {
        // The three points are collinear. Take the center of the two points
        // furthest away from each other.
        // Circle.hpp:23
        let lbc = {
            let v = csrc - bsrc;
            v.x() * v.x() + v.y() * v.y()
        };
        // Circle.hpp:24-26
        return (if lb > lc && lb > lbc {
            a + bsrc
        } else if lc > lb && lc > lbc {
            a + csrc
        } else {
            bsrc + csrc
        }) * 0.5;
    } else {
        // Circle.hpp:28
        let v = b * lc - c * lb;
        // Circle.hpp:29
        a + Vec2d::new(-v.y(), v.x()) / (2.0 * d)
    }
}

// 2D circle defined by its center and squared radius
// Circle.hpp:34-58
//
// template<typename Vector>
// struct CircleSq
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CircleSq {
    // Circle.hpp:38
    pub center: Vec2d,
    // Circle.hpp:39
    pub radius2: CoordF,
}

impl CircleSq {
    // Circle.hpp:42
    // CircleSq(const Vector &center, const Scalar radius2) : center(center), radius2(radius2) {}
    pub fn new(center: Vec2d, radius2: CoordF) -> Self {
        Self { center, radius2 }
    }

    // Circle.hpp:43
    // CircleSq(const Vector &a, const Vector &b) : center(Scalar(0.5) * (a + b)) { radius2 = (a - center).squaredNorm(); }
    pub fn from_two_points(a: Vec2d, b: Vec2d) -> Self {
        let center = (a + b) * 0.5;
        let d = a - center;
        let radius2 = d.x() * d.x() + d.y() * d.y();
        Self { center, radius2 }
    }

    // Circle.hpp:44-47
    // CircleSq(const Vector &a, const Vector &b, const Vector &c, Scalar epsilon) {
    //     this->center = circle_center(a, b, c, epsilon);
    //     this->radius2 = (a - this->center).squaredNorm();
    // }
    pub fn from_three_points(a: Vec2d, b: Vec2d, c: Vec2d, epsilon: CoordF) -> Self {
        let center = circle_center(a, b, c, epsilon);
        let d = a - center;
        let radius2 = d.x() * d.x() + d.y() * d.y();
        Self { center, radius2 }
    }

    // Circle.hpp:49
    // bool invalid() const { return this->radius2 < 0; }
    pub fn invalid(&self) -> bool {
        self.radius2 < 0.0
    }

    // Circle.hpp:50
    // bool valid() const { return ! this->invalid(); }
    pub fn valid(&self) -> bool {
        !self.invalid()
    }

    // Circle.hpp:51
    // bool contains(const Vector &p) const { return (p - this->center).squaredNorm() < this->radius2; }
    pub fn contains(&self, p: Vec2d) -> bool {
        let d = p - self.center;
        (d.x() * d.x() + d.y() * d.y()) < self.radius2
    }

    // Circle.hpp:52
    // bool contains(const Vector &p, const Scalar epsilon2) const { return (p - this->center).squaredNorm() < this->radius2 + epsilon2; }
    pub fn contains_eps(&self, p: Vec2d, epsilon2: CoordF) -> bool {
        let d = p - self.center;
        (d.x() * d.x() + d.y() * d.y()) < self.radius2 + epsilon2
    }

    // Circle.hpp:54-55
    // CircleSq inflated(Scalar epsilon) const
    //     { assert(this->radius2 >= 0); Scalar r = sqrt(this->radius2) + epsilon; return { this->center, r * r }; }
    pub fn inflated(&self, epsilon: CoordF) -> CircleSq {
        debug_assert!(self.radius2 >= 0.0);
        let r = self.radius2.sqrt() + epsilon;
        CircleSq {
            center: self.center,
            radius2: r * r,
        }
    }

    // Circle.hpp:57
    // static CircleSq make_invalid() { return CircleSq { { 0, 0 }, -1 }; }
    pub fn make_invalid() -> CircleSq {
        CircleSq {
            center: Vec2d::new(0.0, 0.0),
            radius2: -1.0,
        }
    }
}

// 2D circle defined by its center and radius
// Circle.hpp:61-88
//
// template<typename Vector>
// struct Circle
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Circle {
    // Circle.hpp:65
    pub center: Vec2d,
    // Circle.hpp:66
    pub radius: CoordF,
}

impl Circle {
    // Circle.hpp:69
    // Circle(const Vector &center, const Scalar radius) : center(center), radius(radius) {}
    pub fn new(center: Vec2d, radius: CoordF) -> Self {
        Self { center, radius }
    }

    // Circle.hpp:70
    // Circle(const Vector &a, const Vector &b) : center(Scalar(0.5) * (a + b)) { radius = (a - center).norm(); }
    pub fn from_two_points(a: Vec2d, b: Vec2d) -> Self {
        let center = (a + b) * 0.5;
        let radius = (a - center).length();
        Self { center, radius }
    }

    // Circle.hpp:71
    // Circle(const Vector &a, const Vector &b, const Vector &c, const Scalar epsilon) { *this = CircleSq(a, b, c, epsilon); }
    pub fn from_three_points(a: Vec2d, b: Vec2d, c: Vec2d, epsilon: CoordF) -> Self {
        Circle::from(CircleSq::from_three_points(a, b, c, epsilon))
    }

    // Circle.hpp:79
    // bool invalid() const { return this->radius < 0; }
    pub fn invalid(&self) -> bool {
        self.radius < 0.0
    }

    // Circle.hpp:80
    // bool valid() const { return ! this->invalid(); }
    pub fn valid(&self) -> bool {
        !self.invalid()
    }

    // Circle.hpp:81
    // bool contains(const Vector &p) const { return (p - this->center).squaredNorm() <= this->radius * this->radius; }
    pub fn contains(&self, p: Vec2d) -> bool {
        let d = p - self.center;
        (d.x() * d.x() + d.y() * d.y()) <= self.radius * self.radius
    }

    // Circle.hpp:82-83
    // bool contains(const Vector &p, const Scalar epsilon) const
    //     { Scalar re = this->radius + epsilon; return (p - this->center).squaredNorm() < re * re; }
    pub fn contains_eps(&self, p: Vec2d, epsilon: CoordF) -> bool {
        let re = self.radius + epsilon;
        let d = p - self.center;
        (d.x() * d.x() + d.y() * d.y()) < re * re
    }

    // Circle.hpp:85
    // Circle inflated(Scalar epsilon) const { assert(this->radius >= 0); return { this->center, this->radius + epsilon }; }
    pub fn inflated(&self, epsilon: CoordF) -> Circle {
        debug_assert!(self.radius >= 0.0);
        Circle {
            center: self.center,
            radius: self.radius + epsilon,
        }
    }

    // Circle.hpp:87
    // static Circle make_invalid() { return Circle { { 0, 0 }, -1 }; }
    pub fn make_invalid() -> Circle {
        Circle {
            center: Vec2d::new(0.0, 0.0),
            radius: -1.0,
        }
    }
}

// Conversion from CircleSq
// Circle.hpp:74-75
// template<typename Vector2>
// explicit Circle(const CircleSq<Vector2> &c) : center(c.center), radius(c.radius2 <= 0 ? c.radius2 : sqrt(c.radius2)) {}
impl From<CircleSq> for Circle {
    fn from(c: CircleSq) -> Self {
        Circle {
            center: c.center,
            radius: if c.radius2 <= 0.0 {
                c.radius2
            } else {
                c.radius2.sqrt()
            },
        }
    }
}

/// Find the center of the circle corresponding to the vector of Points as an arc.
/// Circle.cpp:11-18
///
/// Point circle_center_taubin_newton(const Points::const_iterator& input_begin, const Points::const_iterator& input_end, size_t cycles)
pub fn circle_center_taubin_newton_points(input: &[Point], cycles: usize) -> Point {
    // Circle.cpp:13-15
    let mut tmp: Vec2ds = Vec::with_capacity(input.len());
    for in_pt in input {
        // unscale(in)
        tmp.push(in_pt.to_f64());
    }
    // Circle.cpp:16
    let center = circle_center_taubin_newton(&tmp, cycles);
    // Circle.cpp:17
    Point::new_scale(center.x(), center.y())
}

/// Adapted from work in "Circular and Linear Regression: Fitting circles and lines by least squares", pg 126
/// Returns a point corresponding to the center of a circle for which all of the points from input_begin to input_end
/// lie on.
/// Circle.cpp:23-96
///
/// Vec2d circle_center_taubin_newton(const Vec2ds::const_iterator& input_begin, const Vec2ds::const_iterator& input_end, size_t cycles)
pub fn circle_center_taubin_newton(input: &[Vec2d], cycles: usize) -> Vec2d {
    // calculate the centroid of the data set
    // Circle.cpp:26
    let sum = input
        .iter()
        .fold(Vec2d::new(0.0, 0.0), |acc, &p| acc + p);
    // Circle.cpp:27-28
    let n = input.len();
    let n_flt = n as f64;
    // Circle.cpp:29
    let centroid = sum / n_flt;

    // Compute the normalized moments of the data set.
    // Circle.cpp:32
    let mut mxx = 0.0;
    let mut myy = 0.0;
    let mut mxy = 0.0;
    let mut mxz = 0.0;
    let mut myz = 0.0;
    let mut mzz = 0.0;
    // Circle.cpp:33-44
    for it in input.iter() {
        // center/normalize the data.
        let xi = it.x() - centroid.x();
        let yi = it.y() - centroid.y();
        let zi = xi * xi + yi * yi;
        mxy += xi * yi;
        mxx += xi * xi;
        myy += yi * yi;
        mxz += xi * zi;
        myz += yi * zi;
        mzz += zi * zi;
    }

    // divide by number of points to get the moments
    // Circle.cpp:47-52
    mxx /= n_flt;
    myy /= n_flt;
    mxy /= n_flt;
    mxz /= n_flt;
    myz /= n_flt;
    mzz /= n_flt;

    // Compute the coefficients of the characteristic polynomial for the circle
    // eq 5.60
    // Circle.cpp:56
    let mz = mxx + myy; // xx + yy = z
                        // Circle.cpp:57
    let cov_xy = mxx * myy - mxy * mxy; // this shows up a couple times so cache it here.
                                        // Circle.cpp:58
    let c3 = 4.0 * mz;
    // Circle.cpp:59
    let c2 = -3.0 * (mz * mz) - mzz;
    // Circle.cpp:60
    let c1 = mz * (mzz - (mz * mz)) + 4.0 * mz * cov_xy - (mxz * mxz) - (myz * myz);
    // Circle.cpp:61
    let c0 = (mxz * mxz) * myy + (myz * myz) * mxx - 2.0 * mxz * myz * mxy - cov_xy * (mzz - (mz * mz));

    // Circle.cpp:63
    let c22 = c2 + c2;
    // Circle.cpp:64
    let c33 = c3 + c3 + c3;

    // solve the characteristic polynomial with Newton's method.
    // Circle.cpp:67-68
    let mut xnew = 0.0;
    let mut ynew = 1e20;

    // Circle.cpp:70
    let mut i = 0;
    while i < cycles {
        // Circle.cpp:71
        let yold = ynew;
        // Circle.cpp:72
        ynew = c0 + xnew * (c1 + xnew * (c2 + xnew * c3));
        // Circle.cpp:73-76
        if ynew.abs() > yold.abs() {
            // BOOST_LOG_TRIVIAL(error) << "Geometry: Fit is going in the wrong direction.\n";
            log::error!("Geometry: Fit is going in the wrong direction.\n");
            return Vec2d::new(f64::NAN, f64::NAN);
        }
        // Circle.cpp:77
        let dy = c1 + xnew * (c22 + xnew * c33);

        // Circle.cpp:79
        let xold = xnew;
        // Circle.cpp:80
        xnew = xold - (ynew / dy);

        // Circle.cpp:82
        if ((xnew - xold) / xnew).abs() < 1e-12 {
            i = cycles; // converged, we're done here
        }

        // Circle.cpp:84-87
        if xnew < 0.0 {
            // reset, we went negative
            xnew = 0.0;
        }

        i += 1;
    }

    // compute the determinant and the circle's parameters now that we've solved.
    // Circle.cpp:91
    let det = xnew * xnew - xnew * mz + cov_xy;

    // Circle.cpp:93
    let mut center = Vec2d::new(
        mxz * (myy - xnew) - myz * mxy,
        myz * (mxx - xnew) - mxz * mxy,
    );
    // Circle.cpp:94
    center = center / (det * 2.0);
    // Circle.cpp:95
    center + centroid
}

/// Circle.cpp:98-109
///
/// Circled circle_taubin_newton(const Vec2ds& input, size_t cycles)
pub fn circle_taubin_newton(input: &[Vec2d], cycles: usize) -> Circle {
    // Circle.cpp:100
    let mut out;
    // Circle.cpp:101-107
    if input.len() < 3 {
        out = Circle::make_invalid();
    } else {
        out = Circle::make_invalid();
        out.center = circle_center_taubin_newton(input, cycles);
        out.radius = input
            .iter()
            .fold(0.0, |acc, &pt| (pt - out.center).length() + acc);
        out.radius /= input.len() as f64;
    }
    out
}

/// Find circle using RANSAC randomized algorithm.
/// Circle.cpp:111-138
///
/// Circled circle_ransac(const Vec2ds &input, size_t iterations, double *min_error)
pub fn circle_ransac(input: &[Vec2d], iterations: usize, min_error: Option<&mut f64>) -> Circle {
    // Circle.cpp:113-114
    if input.len() < 3 {
        return Circle::make_invalid();
    }

    // Circle.cpp:116
    let mut rng = Mt19937::new(); // std::mt19937 rng; (default seed 5489)
                                  // Circle.cpp:117
    let mut samples: Vec<Vec2d> = Vec::new();
    // Circle.cpp:118
    let mut circle_best = Circle::make_invalid();
    // Circle.cpp:119
    let mut err_min = f64::MAX;
    // Circle.cpp:120
    for _iter in 0..iterations {
        // Circle.cpp:121
        samples.clear();
        // Circle.cpp:122: std::sample(input.begin(), input.end(), std::back_inserter(samples), 3, rng);
        sample(input, &mut samples, 3, &mut rng);
        // Circle.cpp:123-126
        let mut c = Circle::new(Vec2d::new(0.0, 0.0), 0.0);
        c.center = circle_center(samples[0], samples[1], samples[2], EPSILON);
        c.radius = input
            .iter()
            .fold(0.0, |acc, &pt| (pt - c.center).length() + acc);
        c.radius /= input.len() as f64;
        // Circle.cpp:127
        let mut err = 0.0_f64;
        // Circle.cpp:128-129
        for pt in input.iter() {
            err = err.max(((*pt - c.center).length() - c.radius).abs());
        }
        // Circle.cpp:130-133
        if err < err_min {
            err_min = err;
            circle_best = c;
        }
    }
    // Circle.cpp:135-136
    if let Some(me) = min_error {
        *me = err_min;
    }
    // Circle.cpp:137
    circle_best
}

// Randomized algorithm by Emo Welzl, working with squared radii for efficiency. The returned circle radius is inflated by epsilon.
// Circle.hpp:108-138
//
// template<typename Vector, typename Points>
// CircleSq<Vector> smallest_enclosing_circle2_welzl(const Points &points, const typename Vector::Scalar epsilon)
//
// The crate instantiation works over `Points` (= Vec<Point>) casting to Vec2d, matching the
// `smallest_enclosing_circle_welzl(const Points &points)` inline at Circle.hpp:148.
pub fn smallest_enclosing_circle2_welzl(points: &[Point], epsilon: CoordF) -> CircleSq {
    // C++ `Point::template cast<Scalar>()` is a plain numeric cast that PRESERVES the
    // scaled-integer magnitude (it is NOT `unscale`). The Welzl algorithm therefore
    // operates in scaled-coordinate doubles and inflates by the *scaled* `epsilon`
    // (SCALED_EPSILON = 10.0). Using `Point::to_f64()` here would unscale (÷ SCALING_FACTOR)
    // and mix unit systems with `epsilon`, diverging from C++; do a raw cast instead.
    // Downstream (build_volume.rs) unscales the resulting circle, matching scaled output.
    let cast = |p: &Point| -> Vec2d { Vec2d::new(p.x as f64, p.y as f64) };

    // Circle.hpp:112
    let mut circle = CircleSq::new(Vec2d::new(0.0, 0.0), 0.0);

    // Circle.hpp:114
    if !points.is_empty() {
        // Circle.hpp:115: points[0].template cast<Scalar>()
        let p0 = cast(&points[0]);
        // Circle.hpp:116-118
        if points.len() == 1 {
            circle.center = p0;
            circle.radius2 = epsilon * epsilon;
        } else {
            // Circle.hpp:120
            circle = CircleSq::from_two_points(p0, cast(&points[1])).inflated(epsilon);
            // Circle.hpp:121-122
            for i in 2..points.len() {
                let p = cast(&points[i]);
                if !circle.contains(p) {
                    // p is the first point on the smallest circle enclosing points[0..i]
                    // Circle.hpp:124
                    circle = CircleSq::from_two_points(p0, p).inflated(epsilon);
                    // Circle.hpp:125-126
                    for j in 1..i {
                        let q = cast(&points[j]);
                        if !circle.contains(q) {
                            // q is the second point on the smallest circle enclosing points[0..i]
                            // Circle.hpp:128
                            circle = CircleSq::from_two_points(p, q).inflated(epsilon);
                            // Circle.hpp:129-131
                            for k in 0..j {
                                let r = cast(&points[k]);
                                if !circle.contains(r) {
                                    circle = CircleSq::from_three_points(p, q, r, epsilon)
                                        .inflated(epsilon);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // Circle.hpp:137
    circle
}

// Randomized algorithm by Emo Welzl. The returned circle radius is inflated by epsilon.
// Circle.hpp:141-145
//
// template<typename Vector, typename Points>
// Circle<Vector> smallest_enclosing_circle_welzl(const Points &points, const typename Vector::Scalar epsilon)
pub fn smallest_enclosing_circle_welzl_eps(points: &[Point], epsilon: CoordF) -> Circle {
    // Circle.hpp:144
    Circle::from(smallest_enclosing_circle2_welzl(points, epsilon))
}

// Randomized algorithm by Emo Welzl. The returned circle radius is inflated by SCALED_EPSILON.
// Circle.hpp:148-151
//
// inline Circled smallest_enclosing_circle_welzl(const Points &points)
pub fn smallest_enclosing_circle_welzl(points: &[Point]) -> Circle {
    // Circle.hpp:150
    smallest_enclosing_circle_welzl_eps(points, SCALED_EPSILON)
}

// Ugly named variant, that accepts the squared line
// Don't call me with a nearly zero length vector!
// sympy:
// factor(solve([a * x + b * y + c, x**2 + y**2 - r**2], [x, y])[0])
// factor(solve([a * x + b * y + c, x**2 + y**2 - r**2], [x, y])[1])
// Circle.hpp:158-172
//
// template<typename T>
// int ray_circle_intersections_r2_lv2_c(T r2, T a, T b, T lv2, T c, std::pair<Vector, Vector> &out)
pub fn ray_circle_intersections_r2_lv2_c(
    r2: CoordF,
    a: CoordF,
    b: CoordF,
    lv2: CoordF,
    c: CoordF,
    out: &mut (Vec2d, Vec2d),
) -> i32 {
    // Circle.hpp:161-162
    let x0 = -a * c;
    let y0 = -b * c;
    // Circle.hpp:163
    let d2 = r2 * lv2 - c * c;
    // Circle.hpp:164-165
    if d2 < 0.0 {
        return 0;
    }
    // Circle.hpp:166
    let d = d2.sqrt();
    // Circle.hpp:167-170
    out.0.x = (x0 + b * d) / lv2;
    out.0.y = (y0 - a * d) / lv2;
    out.1.x = (x0 - b * d) / lv2;
    out.1.y = (y0 + a * d) / lv2;
    // Circle.hpp:171
    if d == 0.0 {
        1
    } else {
        2
    }
}

// Circle.hpp:173-183
//
// template<typename T>
// int ray_circle_intersections(T r, T a, T b, T c, std::pair<Vector, Vector> &out)
pub fn ray_circle_intersections(
    r: CoordF,
    a: CoordF,
    b: CoordF,
    c: CoordF,
    out: &mut (Vec2d, Vec2d),
) -> i32 {
    // Circle.hpp:176
    let lv2 = a * a + b * b;
    // Circle.hpp:177-181
    if lv2 < SCALED_EPSILON * SCALED_EPSILON {
        //FIXME what is the correct epsilon?
        // What if the line touches the circle?
        return 0; // C++ `return false;` (0) — function is declared to return int
    }
    // Circle.hpp:182
    // NOTE: BambuStudio source calls `ray_circle_intersections_r2_lv2_c2` here, which does not
    // exist; the only matching definition is `ray_circle_intersections_r2_lv2_c` above. Ported
    // to the existing definition (the typo'd name is a known upstream compile-time dead path).
    ray_circle_intersections_r2_lv2_c(r * r, a, b, a * a + b * b, c, out)
}

/// Mersenne Twister (`std::mt19937`) faithful to the standard parameters used by
/// libstdc++'s default-constructed engine (seed 5489). Used by [`circle_ransac`].
struct Mt19937 {
    mt: [u32; 624],
    index: usize,
}

impl Mt19937 {
    const N: usize = 624;
    const M: usize = 397;
    const MATRIX_A: u32 = 0x9908_b0df;
    const UPPER_MASK: u32 = 0x8000_0000;
    const LOWER_MASK: u32 = 0x7fff_ffff;

    fn new() -> Self {
        Self::with_seed(5489)
    }

    fn with_seed(seed: u32) -> Self {
        let mut mt = [0u32; Self::N];
        mt[0] = seed;
        for i in 1..Self::N {
            mt[i] = (1_812_433_253u32
                .wrapping_mul(mt[i - 1] ^ (mt[i - 1] >> 30)))
            .wrapping_add(i as u32);
        }
        Self {
            mt,
            index: Self::N,
        }
    }

    fn generate(&mut self) {
        for i in 0..Self::N {
            let y = (self.mt[i] & Self::UPPER_MASK)
                | (self.mt[(i + 1) % Self::N] & Self::LOWER_MASK);
            let mut next = self.mt[(i + Self::M) % Self::N] ^ (y >> 1);
            if y & 1 != 0 {
                next ^= Self::MATRIX_A;
            }
            self.mt[i] = next;
        }
        self.index = 0;
    }

    fn next_u32(&mut self) -> u32 {
        if self.index >= Self::N {
            self.generate();
        }
        let mut y = self.mt[self.index];
        self.index += 1;
        // Tempering
        y ^= y >> 11;
        y ^= (y << 7) & 0x9d2c_5680;
        y ^= (y << 15) & 0xefc6_0000;
        y ^= y >> 18;
        y
    }
}

/// Faithful port of libstdc++'s `std::uniform_int_distribution<size_t>` invocation
/// for a range `[0, range]`, driven by a `std::mt19937` engine. Mirrors the
/// rejection sampling used in `bits/uniform_int_dist.h`.
fn uniform_int(g: &mut Mt19937, range_inclusive: u64) -> u64 {
    // urange = b - a (here a = 0, b = range_inclusive)
    let urange = range_inclusive;
    // mt19937 produces values in [0, 2^32 - 1]
    let urngrange: u64 = u32::MAX as u64; // (max - min) for the engine

    if urngrange > urange {
        // downscaling: __uerange = __urange + 1
        let uerange = urange + 1; // > 1
        let scaling = urngrange / uerange;
        let past = uerange * scaling;
        let mut ret;
        loop {
            ret = g.next_u32() as u64; // (__urng() - __urngmin), __urngmin == 0
            if ret < past {
                break;
            }
        }
        ret / scaling
    } else if urngrange < urange {
        // upscaling is not reachable for mt19937 vs the small ranges used here.
        // Fall back to the libstdc++ general path is unnecessary for circle_ransac.
        // (range never exceeds the engine range in practice.)
        let mut ret;
        loop {
            ret = g.next_u32() as u64;
            if ret <= urange {
                break;
            }
        }
        ret
    } else {
        // urngrange == urange
        g.next_u32() as u64
    }
}

/// Faithful port of libstdc++'s `std::sample` for forward-iterator populations
/// (selection sampling, a.k.a. Algorithm S / Fan-Muller-Rezucha). Selects up to
/// `n` elements from `input` into `out`, in input order, using engine `g`.
fn sample(input: &[Vec2d], out: &mut Vec<Vec2d>, n: usize, g: &mut Mt19937) {
    // _Size __unsampled_sz = std::distance(__first, __last);
    let mut unsampled_sz = input.len();
    // for (__n = std::min(__n, __unsampled_sz); __n != 0; ++__first)
    let mut remaining = n.min(unsampled_sz);
    let mut idx = 0usize;
    while remaining != 0 {
        // if (__d(__g, __param_type{0, --__unsampled_sz}) < __n)
        unsampled_sz -= 1;
        if uniform_int(g, unsampled_sz as u64) < remaining as u64 {
            out.push(input[idx]);
            remaining -= 1;
        }
        idx += 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_circle_new() {
        let c = Circle::new(PointF::new(0.0, 0.0), 5.0);
        assert_eq!(c.center.x, 0.0);
        assert_eq!(c.radius, 5.0);
    }

    #[test]
    fn test_circle_center_circumcircle() {
        // Three points on the unit circle => center (0,0).
        let a = Vec2d::new(1.0, 0.0);
        let b = Vec2d::new(0.0, 1.0);
        let c = Vec2d::new(-1.0, 0.0);
        let center = circle_center(a, b, c, EPSILON);
        assert!(center.x.abs() < 1e-9);
        assert!(center.y.abs() < 1e-9);
    }

    #[test]
    fn test_circle_center_collinear() {
        // Collinear points: returns midpoint of the two furthest-apart points.
        let a = Vec2d::new(0.0, 0.0);
        let b = Vec2d::new(1.0, 0.0);
        let c = Vec2d::new(2.0, 0.0);
        let center = circle_center(a, b, c, 1e-9);
        // Furthest apart are a and c => center (1, 0).
        assert!((center.x - 1.0).abs() < 1e-9);
        assert!(center.y.abs() < 1e-9);
    }

    #[test]
    fn test_circle_taubin_newton() {
        // Points sampled around a unit circle centered at origin.
        let mut pts = Vec2ds::new();
        for k in 0..8 {
            let t = (k as f64) * std::f64::consts::PI / 4.0;
            pts.push(Vec2d::new(t.cos(), t.sin()));
        }
        let c = circle_taubin_newton(&pts, 20);
        assert!(c.valid());
        assert!(c.center.x.abs() < 1e-6);
        assert!(c.center.y.abs() < 1e-6);
        assert!((c.radius - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_circle_taubin_newton_too_few() {
        let pts = vec![Vec2d::new(0.0, 0.0), Vec2d::new(1.0, 0.0)];
        let c = circle_taubin_newton(&pts, 20);
        assert!(c.invalid());
    }

    #[test]
    fn test_circle_sq_from_two_points() {
        let a = Vec2d::new(-1.0, 0.0);
        let b = Vec2d::new(1.0, 0.0);
        let c = CircleSq::from_two_points(a, b);
        assert!((c.center.x - 0.0).abs() < 1e-12);
        assert!((c.radius2 - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_circle_from_circlesq() {
        let csq = CircleSq::new(Vec2d::new(2.0, 3.0), 4.0);
        let c = Circle::from(csq);
        assert_eq!(c.center, Vec2d::new(2.0, 3.0));
        assert!((c.radius - 2.0).abs() < 1e-12);
    }

    #[test]
    fn test_make_invalid() {
        assert!(Circle::make_invalid().invalid());
        assert!(CircleSq::make_invalid().invalid());
    }

    #[test]
    fn test_mt19937_default_sequence() {
        // The 10000th output of a default-seeded std::mt19937 is 4123659995.
        let mut g = Mt19937::new();
        let mut v = 0u32;
        for _ in 0..10000 {
            v = g.next_u32();
        }
        assert_eq!(v, 4_123_659_995);
    }

    #[test]
    fn test_smallest_enclosing_circle_welzl() {
        let points = vec![
            Point::new_scale(0.0, 0.0),
            Point::new_scale(2.0, 0.0),
            Point::new_scale(1.0, 1.0),
        ];
        let circle = smallest_enclosing_circle_welzl(&points);
        assert!(circle.valid());
        // All points must be enclosed (radius inflated by SCALED_EPSILON).
        // The Welzl circle is in scaled-coordinate doubles (C++ `cast<double>()`,
        // not `unscale`), so points must be compared with the same raw cast.
        for p in &points {
            assert!(circle.contains(Vec2d::new(p.x as f64, p.y as f64)));
        }
    }

    #[test]
    fn test_ray_circle_intersections() {
        // Line x = 0 (a=1, b=0, c=0) through a unit circle => (0, 1) and (0, -1).
        let mut out = (Vec2d::new(0.0, 0.0), Vec2d::new(0.0, 0.0));
        let count = ray_circle_intersections(1.0, 1.0, 0.0, 0.0, &mut out);
        assert_eq!(count, 2);
        assert!((out.0.x).abs() < 1e-12);
        assert!((out.0.y.abs() - 1.0).abs() < 1e-12);
        assert!((out.1.x).abs() < 1e-12);
        assert!((out.1.y.abs() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_circle_ransac() {
        let mut pts = Vec2ds::new();
        for k in 0..16 {
            let t = (k as f64) * std::f64::consts::PI / 8.0;
            pts.push(Vec2d::new(2.0 + 3.0 * t.cos(), -1.0 + 3.0 * t.sin()));
        }
        let mut err = 0.0;
        let c = circle_ransac(&pts, 20, Some(&mut err));
        assert!(c.valid());
        assert!((c.center.x - 2.0).abs() < 1e-3);
        assert!((c.center.y + 1.0).abs() < 1e-3);
        assert!((c.radius - 3.0).abs() < 1e-3);
    }
}
