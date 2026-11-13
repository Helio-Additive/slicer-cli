//! Port of ClipperLib, `src/clipper/clipper.cpp` and `clipper.hpp` of the engine (the file
//! `libslic3r/clipper.cpp` compiles it into libslic3r).
//!
//! In progress: this file holds the library's free functions. The clipper itself
//! (`ClipperBase`, `Clipper`, `ClipperOffset`, `PolyTree`) still runs as C++ through
//! `clipper-z-sys`; see `docs/rust-engine/parity/not-ported.md`.
//!
//! The engine builds ClipperLib with `CLIPPERLIB_INT32` (clipper.hpp:83-95), so `cInt` is a
//! 32-bit integer. The float expressions below are written as the C++ build evaluates them:
//! clang for arm64 contracts a product into a fused multiply-add inside one expression.
//! `tests/clipper_native_parity.rs` holds every function here equal to the library.

#![allow(non_camel_case_types)]

/// clipper.hpp:88 `typedef int32_t cInt;`
pub type cInt = i32;

/// clipper.hpp `IntPoint`: an integer point.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
pub struct IntPoint {
    pub x: cInt,
    pub y: cInt,
}

impl IntPoint {
    pub fn new(x: cInt, y: cInt) -> Self {
        Self { x, y }
    }
}

/// clipper.hpp:113 `typedef std::vector<IntPoint> Path;`
pub type Path = Vec<IntPoint>;
/// clipper.hpp:114 `typedef std::vector<Path> Paths;`
pub type Paths = Vec<Path>;

/// clipper.cpp:177-189
/// C++: double Area(const Path &poly)
pub fn area(poly: &Path) -> f64 {
    let size = poly.len() as i32;
    if size < 3 {
        return 0.0;
    }

    let mut a = 0.0f64;
    let mut j = (size - 1) as usize;
    for i in 0..size as usize {
        // a += ((double)poly[j].x() + poly[i].x()) * ((double)poly[j].y() - poly[i].y());
        // One expression: the product is fused into the sum.
        a = (poly[j].x as f64 + poly[i].x as f64).mul_add(poly[j].y as f64 - poly[i].y as f64, a);
        j = i;
    }
    -a * 0.5
}

/// clipper.hpp:192
/// C++: inline bool Orientation(const Path &poly) { return Area(poly) >= 0; }
pub fn orientation(poly: &Path) -> bool {
    area(poly) >= 0.0
}

/// clipper.cpp:220-257
/// C++: int PointInPolygon(const IntPoint &pt, const Path &path)
/// See "The Point in Polygon Problem for Arbitrary Polygons" by Hormann & Agathos.
pub fn point_in_polygon(pt: &IntPoint, path: &Path) -> i32 {
    // returns 0 if false, +1 if true, -1 if pt ON polygon boundary
    let mut result = 0;
    let cnt = path.len();
    if cnt < 3 {
        return 0;
    }
    // (double)(ip.x() - pt.x()) * (ipNext.y() - pt.y()) - (double)(ipNext.x() - pt.x()) * (ip.y() - pt.y()):
    // the differences are taken in cInt and wrap as the C++ ones do; the left product
    // is fused with the right one as the addend.
    let d_of = |ip: &IntPoint, ip_next: &IntPoint| -> f64 {
        let a = ip.x.wrapping_sub(pt.x) as f64;
        let b = ip_next.y.wrapping_sub(pt.y) as f64;
        let c = ip_next.x.wrapping_sub(pt.x) as f64;
        let e = ip.y.wrapping_sub(pt.y) as f64;
        a.mul_add(b, -(c * e))
    };
    let mut ip = path[0];
    for i in 1..=cnt {
        let ip_next = if i == cnt { path[0] } else { path[i] };
        if ip_next.y == pt.y
            && ((ip_next.x == pt.x) || (ip.y == pt.y && ((ip_next.x > pt.x) == (ip.x < pt.x))))
        {
            return -1;
        }
        if (ip.y < pt.y) != (ip_next.y < pt.y) {
            if ip.x >= pt.x {
                if ip_next.x > pt.x {
                    result = 1 - result;
                } else {
                    let d = d_of(&ip, &ip_next);
                    if d == 0.0 {
                        return -1;
                    }
                    if (d > 0.0) == (ip_next.y > ip.y) {
                        result = 1 - result;
                    }
                }
            } else if ip_next.x > pt.x {
                let d = d_of(&ip, &ip_next);
                if d == 0.0 {
                    return -1;
                }
                if (d > 0.0) == (ip_next.y > ip.y) {
                    result = 1 - result;
                }
            }
        }
        ip = ip_next;
    }
    result
}

/// clipper.cpp:3829-3832
/// C++: void ReversePath(Path& p)
pub fn reverse_path(p: &mut Path) {
    p.reverse();
}

/// clipper.cpp:3835-3839
/// C++: void ReversePaths(Paths& p)
pub fn reverse_paths(p: &mut Paths) {
    for path in p.iter_mut() {
        reverse_path(path);
    }
}

/// clipper.cpp:3854-3859
/// C++: inline double DistanceSqrd(const IntPoint& pt1, const IntPoint& pt2)
#[inline]
pub fn distance_sqrd(pt1: &IntPoint, pt2: &IntPoint) -> f64 {
    let dx = pt1.x.wrapping_sub(pt2.x) as f64;
    let dy = pt1.y.wrapping_sub(pt2.y) as f64;
    dx.mul_add(dx, dy * dy)
}

/// clipper.cpp:3862-3876
/// C++: double DistanceFromLineSqrd(const IntPoint& pt, const IntPoint& ln1, const IntPoint& ln2)
pub fn distance_from_line_sqrd(pt: &IntPoint, ln1: &IntPoint, ln2: &IntPoint) -> f64 {
    // The equation of a line in general form (Ax + By + C = 0)
    // given 2 points (x1,y1) & (x2,y2) is ...
    // (y1 - y2)x + (x2 - x1)y + (y2 - y1)x1 - (x2 - x1)y1 = 0
    // A = (y1 - y2); B = (x2 - x1); C = (y2 - y1)x1 - (x2 - x1)y1
    // perpendicular distance of point (x3,y3) = (Ax3 + By3 + C)/Sqrt(A^2 + B^2)
    let a = ln1.y.wrapping_sub(ln2.y) as f64;
    let b = ln2.x.wrapping_sub(ln1.x) as f64;
    // double C = A * ln1.x() + B * ln1.y();
    let c = a.mul_add(ln1.x as f64, b * ln1.y as f64);
    // C = A * pt.x() + B * pt.y() - C;
    let c = a.mul_add(pt.x as f64, b * pt.y as f64) - c;
    // return (C * C) / (A * A + B * B);
    (c * c) / a.mul_add(a, b * b)
}

/// clipper.cpp:3879-3903
/// C++: bool SlopesNearCollinear(const IntPoint& pt1, const IntPoint& pt2, const IntPoint& pt3, double distSqrd)
pub fn slopes_near_collinear(pt1: &IntPoint, pt2: &IntPoint, pt3: &IntPoint, dist_sqrd: f64) -> bool {
    // this function is more accurate when the point that's geometrically
    // between the other 2 points is the one that's tested for distance.
    // ie makes it more likely to pick up 'spikes' ...
    // std::abs of a cInt difference: both wrap as the C++ ones do.
    if pt1.x.wrapping_sub(pt2.x).wrapping_abs() > pt1.y.wrapping_sub(pt2.y).wrapping_abs() {
        if (pt1.x > pt2.x) == (pt1.x < pt3.x) {
            distance_from_line_sqrd(pt1, pt2, pt3) < dist_sqrd
        } else if (pt2.x > pt1.x) == (pt2.x < pt3.x) {
            distance_from_line_sqrd(pt2, pt1, pt3) < dist_sqrd
        } else {
            distance_from_line_sqrd(pt3, pt1, pt2) < dist_sqrd
        }
    } else if (pt1.y > pt2.y) == (pt1.y < pt3.y) {
        distance_from_line_sqrd(pt1, pt2, pt3) < dist_sqrd
    } else if (pt2.y > pt1.y) == (pt2.y < pt3.y) {
        distance_from_line_sqrd(pt2, pt1, pt3) < dist_sqrd
    } else {
        distance_from_line_sqrd(pt3, pt1, pt2) < dist_sqrd
    }
}

/// clipper.cpp:3906-3911
/// C++: bool PointsAreClose(IntPoint pt1, IntPoint pt2, double distSqrd)
pub fn points_are_close(pt1: IntPoint, pt2: IntPoint, dist_sqrd: f64) -> bool {
    let dx = pt1.x.wrapping_sub(pt2.x) as f64;
    let dy = pt1.y.wrapping_sub(pt2.y) as f64;
    dx.mul_add(dx, dy * dy) <= dist_sqrd
}

/// clipper.hpp:266-276 `struct OutPt`. The C++ links the points with pointers; here
/// `next` and `prev` are indices into the vector that owns them.
#[derive(Debug, Clone, Copy, Default)]
pub struct OutPt {
    pub idx: i32,
    pub pt: IntPoint,
    pub next: usize,
    pub prev: usize,
}

/// clipper.cpp:3914-3921
/// C++: OutPt* ExcludeOp(OutPt* op)
pub fn exclude_op(out_pts: &mut [OutPt], op: usize) -> usize {
    let result = out_pts[op].prev;
    out_pts[result].next = out_pts[op].next;
    let next = out_pts[op].next;
    out_pts[next].prev = result;
    out_pts[result].idx = 0;
    result
}

/// clipper.cpp:3925-3981
/// C++: void CleanPolygon(const Path& in_poly, Path& out_poly, double distance)
pub fn clean_polygon(in_poly: &Path, out_poly: &mut Path, distance: f64) {
    // distance = proximity in units/pixels below which vertices
    // will be stripped. Default ~= sqrt(2).

    let mut size = in_poly.len();

    if size == 0 {
        out_poly.clear();
        return;
    }

    let mut out_pts: Vec<OutPt> = vec![OutPt::default(); size];
    for i in 0..size {
        out_pts[i].pt = in_poly[i];
        out_pts[i].next = (i + 1) % size;
        let next = out_pts[i].next;
        out_pts[next].prev = i;
        out_pts[i].idx = 0;
    }

    let dist_sqrd = distance * distance;
    let mut op = 0usize;
    while out_pts[op].idx == 0 && out_pts[op].next != out_pts[op].prev {
        let (prev, next) = (out_pts[op].prev, out_pts[op].next);
        if points_are_close(out_pts[op].pt, out_pts[prev].pt, dist_sqrd) {
            op = exclude_op(&mut out_pts, op);
            size -= 1;
        } else if points_are_close(out_pts[prev].pt, out_pts[next].pt, dist_sqrd) {
            exclude_op(&mut out_pts, next);
            op = exclude_op(&mut out_pts, op);
            size -= 2;
        } else if slopes_near_collinear(&out_pts[prev].pt, &out_pts[op].pt, &out_pts[next].pt, dist_sqrd) {
            op = exclude_op(&mut out_pts, op);
            size -= 1;
        } else {
            out_pts[op].idx = 1;
            op = out_pts[op].next;
        }
    }

    if size < 3 {
        size = 0;
    }
    out_poly.clear();
    out_poly.reserve(size);
    for _ in 0..size {
        out_poly.push(out_pts[op].pt);
        op = out_pts[op].next;
    }
}

/// clipper.hpp:197 `double distance = 1.415`
pub const CLEAN_POLYGON_DEFAULT_DISTANCE: f64 = 1.415;

/// clipper.cpp:3990-3994
/// C++: void CleanPolygons(const Paths& in_polys, Paths& out_polys, double distance)
/// As in the C++, `out_polys` must have the size of `in_polys`.
pub fn clean_polygons(in_polys: &Paths, out_polys: &mut Paths, distance: f64) {
    for i in 0..in_polys.len() {
        clean_polygon(&in_polys[i], &mut out_polys[i], distance);
    }
}
