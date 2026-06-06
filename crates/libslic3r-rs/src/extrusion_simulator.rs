//! Faithful 1:1 port of `ExtrusionSimulator.cpp` / `ExtrusionSimulator.hpp`.
//!
//! C++ Reference:
//! - src/libslic3r/ExtrusionSimulator.hpp
//! - src/libslic3r/ExtrusionSimulator.cpp
//!
//! Line-by-line translation for byte-exact G-code parity. The original C++ uses
//! boost::geometry `point_xy<T>` 2D points and `boost::multi_array<T,2>` grids.
//! The boost point algebra is reproduced here with a small local `V2<T>` vector
//! type, and the 2D arrays are reproduced with row-major `Vec<T>` grids (`A2*`).
//!
//! All of the template helpers in this file are only ever instantiated with
//! `float` (the boost `V2f` = `point_xy<float>`) at their call sites, so they are
//! implemented concretely over `f32` to preserve the exact single-precision
//! arithmetic of the original (critical for parity).

// Optimize the extrusion simulator to the bones.
//#pragma GCC optimize ("O3")
//#undef SLIC3R_DEBUG
//#define NDEBUG

use crate::geometry::{BoundingBox, Point};

// ExtrusionSimulator.cpp:19-21
// #ifndef M_PI
// #define M_PI 3.1415926535897932384626433832795
// #endif
const M_PI: f64 = 3.1415926535897932384626433832795;

// ExtrusionSimulator.hpp:5 (ExtrusionEntity.hpp dependency)
use crate::extrusion_entity::ExtrusionPath;

// libslic3r.h: inline coord_t scale_(coordf_t v)
use crate::scale as scale_;

// libslic3r.h: template<typename T> inline T sqr(T x) { return x * x; }
#[inline]
fn sqr(x: f32) -> f32 {
    x * x
}

// ExtrusionSimulator.hpp:10-17
// enum ExtrusionSimulationType
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ExtrusionSimulationType {
    ExtrusionSimulationSimple,
    ExtrusionSimulationDontSpread,
    ExtrisopmSimulationSpreadNotOverfilled,
    ExtrusionSimulationSpreadFull,
    ExtrusionSimulationSpreadExcess,
}

pub use ExtrusionSimulationType::*;

// ---------------------------------------------------------------------------
// Replacement for the boost::geometry point_xy<T> algebra used in the file.
// ExtrusionSimulator.cpp:27-176
//
// Only the `float` (V2f) and `int` (V2i) instantiations are needed.
// ---------------------------------------------------------------------------

// ExtrusionSimulator.cpp:49-51
// typedef V2<int   >::Type V2i;
// typedef V2<float >::Type V2f;
// typedef V2<double>::Type V2d;
#[derive(Debug, Clone, Copy, PartialEq)]
struct V2f {
    x: f32,
    y: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct V2i {
    x: i32,
    y: i32,
}

// Used for an RGB color.
// ExtrusionSimulator.cpp:54  typedef V3<unsigned char>::Type V3uc;
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct V3uc(u8, u8, u8);

// ExtrusionSimulator.cpp:58-60
// typedef boost::geometry::model::box<V2i> B2i;
// typedef boost::geometry::model::box<V2f> B2f;
#[derive(Debug, Clone, Copy)]
struct B2f {
    min_corner: V2f,
    max_corner: V2f,
}

#[derive(Debug, Clone, Copy)]
struct B2i {
    min_corner: V2i,
    max_corner: V2i,
}

impl V2f {
    #[inline]
    fn new(x: f32, y: f32) -> Self {
        V2f { x, y }
    }
    #[inline]
    fn x(&self) -> f32 {
        self.x
    }
    #[inline]
    fn y(&self) -> f32 {
        self.y
    }
}

impl V2i {
    #[inline]
    fn new(x: i32, y: i32) -> Self {
        V2i { x, y }
    }
    #[inline]
    fn x(&self) -> i32 {
        self.x
    }
    #[inline]
    fn y(&self) -> i32 {
        self.y
    }
    // boost::geometry point_xy mutator: v.x(i)
    #[inline]
    fn set_x(&mut self, x: i32) {
        self.x = x;
    }
    #[inline]
    fn set_y(&mut self, y: i32) {
        self.y = y;
    }
}

impl B2f {
    #[inline]
    fn new(min_corner: V2f, max_corner: V2f) -> Self {
        B2f {
            min_corner,
            max_corner,
        }
    }
    #[inline]
    fn min_corner(&self) -> V2f {
        self.min_corner
    }
    #[inline]
    fn max_corner(&self) -> V2f {
        self.max_corner
    }
    // boost::geometry::expand(box, point)
    #[inline]
    fn expand(&mut self, p: V2f) {
        if p.x < self.min_corner.x {
            self.min_corner.x = p.x;
        }
        if p.y < self.min_corner.y {
            self.min_corner.y = p.y;
        }
        if p.x > self.max_corner.x {
            self.max_corner.x = p.x;
        }
        if p.y > self.max_corner.y {
            self.max_corner.y = p.y;
        }
    }
}

impl B2i {
    #[inline]
    fn new(min_corner: V2i, max_corner: V2i) -> Self {
        B2i {
            min_corner,
            max_corner,
        }
    }
    #[inline]
    fn min_corner(&self) -> V2i {
        self.min_corner
    }
    #[inline]
    fn max_corner(&self) -> V2i {
        self.max_corner
    }
}

// ExtrusionSimulator.cpp:95-103  operator+
#[inline]
fn add(v1: V2f, v2: V2f) -> V2f {
    V2f::new(v1.x + v2.x, v1.y + v2.y)
}

// ExtrusionSimulator.cpp:105-113  operator-
#[inline]
fn sub(v1: V2f, v2: V2f) -> V2f {
    V2f::new(v1.x - v2.x, v1.y - v2.y)
}

// ExtrusionSimulator.cpp:115-122  operator*(v, c)
#[inline]
fn mul(v: V2f, c: f32) -> V2f {
    V2f::new(v.x * c, v.y * c)
}

// ExtrusionSimulator.cpp:133-140  operator/(v, c)
#[inline]
fn div(v: V2f, c: f32) -> V2f {
    V2f::new(v.x / c, v.y / c)
}

// ExtrusionSimulator.cpp:142-148  dot(v1, v2)
#[inline]
fn dot2(v1: V2f, v2: V2f) -> f32 {
    v1.x * v2.x + v1.y * v2.y
}

// ExtrusionSimulator.cpp:150-154  dot(v)
#[inline]
fn dot1(v: V2f) -> f32 {
    dot2(v, v)
}

// ExtrusionSimulator.cpp:156-162  cross(v1, v2)
#[inline]
fn cross(v1: V2f, v2: V2f) -> f32 {
    v1.x() * v2.y() - v2.x() * v1.y()
}

// Euclidian measure
// ExtrusionSimulator.cpp:164-169  l2(v)
#[inline]
fn l2(v: V2f) -> f32 {
    dot1(v).sqrt()
}

// Euclidian measure
// ExtrusionSimulator.cpp:171-176  mag(v)
#[allow(dead_code)]
#[inline]
fn mag(v: V2f) -> f32 {
    l2(v)
}

// ExtrusionSimulator.cpp:178-193
#[inline]
fn dist2_to_line(p0: V2f, p1: V2f, px: V2f) -> f32 {
    let v = sub(p1, p0);
    let mut vx = sub(px, p0);
    let l = dot1(v);
    let mut t = dot2(v, vx);
    if l != 0.0f32 && t > 0.0f32 {
        t /= l;
        vx = sub(px, if t > 1.0f32 { p1 } else { add(p0, mul(v, t)) });
    }
    dot1(vx)
}

// Intersect a circle with a line segment.
// Returns number of intersection points.
// ExtrusionSimulator.cpp:197-234
fn line_circle_intersection(
    p0: V2f,
    p1: V2f,
    center: V2f,
    radius: f32,
    intersection: &mut [V2f; 2],
) -> i32 {
    let v = sub(p1, p0);
    let vc = sub(p0, center);
    let a = dot1(v);
    let b = 2.0f32 * dot2(vc, v);
    let c = dot1(vc) - radius * radius;
    let mut d = b * b - 4.0f32 * a * c;

    if d < 0.0f32 {
        // The circle misses the ray.
        return 0;
    }

    let mut n = 0;
    if d == 0.0f32 {
        // The circle touches the ray at a single tangent point.
        let t = -b / (2.0f32 * a);
        if t >= 0.0f32 && t <= 1.0f32 {
            intersection[n as usize] = add(p0, mul(v, t));
            n += 1;
        }
    } else {
        // The circle intersects the ray in two points.
        d = d.sqrt();
        let mut t = (-b - d) / (2.0f32 * a);
        if t >= 0.0f32 && t <= 1.0f32 {
            intersection[n as usize] = add(p0, mul(v, t));
            n += 1;
        }
        t = (-b + d) / (2.0f32 * a);
        if t >= 0.0f32 && t <= 1.0f32 {
            intersection[n as usize] = add(p0, mul(v, t));
            n += 1;
        }
    }
    n
}

// Which AABB edge a clip pass works against, encoding the per-edge comparison
// direction used in the C++ (left/bottom use `>`, right/top use `<`).
#[derive(Clone, Copy)]
enum ClipEdge {
    // Clip left:   axis = x, limit = aabb.min_corner().x(), "inside" means coord > limit
    Left,
    // Clip bottom: axis = y, limit = aabb.min_corner().y(), "inside" means coord > limit
    Bottom,
    // Clip right:  axis = x, limit = aabb.max_corner().x(), "inside" means coord < limit
    Right,
    // Clip top:    axis = y, limit = aabb.max_corner().y(), "inside" means coord < limit
    Top,
}

// One Sutherland–Hodgman pass against a single AABB edge.
// Reads `nin` points from `in_buf`, writes the clipped points to `out_buf`,
// returns the resulting point count `nout`. Mirrors one of the 4 blocks in
// ExtrusionSimulator.cpp:251-367.
fn clip_pass(in_buf: &[V2f; 8], nin: i32, out_buf: &mut [V2f; 8], aabb: &B2f, edge: ClipEdge) -> i32 {
    let mut nout = 0i32;
    let mut s_idx = (nin - 1) as usize; // const V2T *S = in + nin - 1;
    match edge {
        ClipEdge::Left => {
            let left = aabb.min_corner().x();
            for i in 0..nin as usize {
                let e = in_buf[i]; // const V2T &E = in[i];
                let s = in_buf[s_idx];
                if e.x() == left {
                    out_buf[nout as usize] = e;
                    nout += 1;
                } else if e.x() > left {
                    // E is inside the AABB.
                    if s.x() < left {
                        // S is outside the AABB. Calculate an intersection point.
                        let t = (left - s.x()) / (e.x() - s.x());
                        out_buf[nout as usize] = V2f::new(left, s.y() + t * (e.y() - s.y()));
                        nout += 1;
                    }
                    out_buf[nout as usize] = e;
                    nout += 1;
                } else if s.x() > left {
                    // S is inside the AABB, E is outside the AABB.
                    let t = (left - s.x()) / (e.x() - s.x());
                    out_buf[nout as usize] = V2f::new(left, s.y() + t * (e.y() - s.y()));
                    nout += 1;
                }
                s_idx = i;
            }
        }
        ClipEdge::Bottom => {
            let bottom = aabb.min_corner().y();
            for i in 0..nin as usize {
                let e = in_buf[i];
                let s = in_buf[s_idx];
                if e.y() == bottom {
                    out_buf[nout as usize] = e;
                    nout += 1;
                } else if e.y() > bottom {
                    // E is inside the AABB.
                    if s.y() < bottom {
                        // S is outside the AABB. Calculate an intersection point.
                        let t = (bottom - s.y()) / (e.y() - s.y());
                        out_buf[nout as usize] = V2f::new(s.x() + t * (e.x() - s.x()), bottom);
                        nout += 1;
                    }
                    out_buf[nout as usize] = e;
                    nout += 1;
                } else if s.y() > bottom {
                    // S is inside the AABB, E is outside the AABB.
                    let t = (bottom - s.y()) / (e.y() - s.y());
                    out_buf[nout as usize] = V2f::new(s.x() + t * (e.x() - s.x()), bottom);
                    nout += 1;
                }
                s_idx = i;
            }
        }
        ClipEdge::Right => {
            let right = aabb.max_corner().x();
            for i in 0..nin as usize {
                let e = in_buf[i];
                let s = in_buf[s_idx];
                if e.x() == right {
                    out_buf[nout as usize] = e;
                    nout += 1;
                } else if e.x() < right {
                    // E is inside the AABB.
                    if s.x() > right {
                        // S is outside the AABB. Calculate an intersection point.
                        let t = (right - s.x()) / (e.x() - s.x());
                        out_buf[nout as usize] = V2f::new(right, s.y() + t * (e.y() - s.y()));
                        nout += 1;
                    }
                    out_buf[nout as usize] = e;
                    nout += 1;
                } else if s.x() < right {
                    // S is inside the AABB, E is outside the AABB.
                    let t = (right - s.x()) / (e.x() - s.x());
                    out_buf[nout as usize] = V2f::new(right, s.y() + t * (e.y() - s.y()));
                    nout += 1;
                }
                s_idx = i;
            }
        }
        ClipEdge::Top => {
            let top = aabb.max_corner().y();
            for i in 0..nin as usize {
                let e = in_buf[i];
                let s = in_buf[s_idx];
                if e.y() == top {
                    out_buf[nout as usize] = e;
                    nout += 1;
                } else if e.y() < top {
                    // E is inside the AABB.
                    if s.y() > top {
                        // S is outside the AABB. Calculate an intersection point.
                        let t = (top - s.y()) / (e.y() - s.y());
                        out_buf[nout as usize] = V2f::new(s.x() + t * (e.x() - s.x()), top);
                        nout += 1;
                    }
                    out_buf[nout as usize] = e;
                    nout += 1;
                } else if s.y() < top {
                    // S is inside the AABB, E is outside the AABB.
                    let t = (top - s.y()) / (e.y() - s.y());
                    out_buf[nout as usize] = V2f::new(s.x() + t * (e.x() - s.x()), top);
                    nout += 1;
                }
                s_idx = i;
            }
        }
    }
    debug_assert!(nout <= 8);
    nout
}

// Sutherland–Hodgman clipping of a rectangle against an AABB.
// Expects the first 4 points of rect to be filled at the beginning.
// The clipping may produce up to 8 points.
// Returns the number of resulting points.
// ExtrusionSimulator.cpp:240-371
fn clip_rect_by_aabb(rect: &mut [V2f; 8], aabb: &B2f) -> i32 {
    let mut result: [V2f; 8] = [V2f::new(0.0, 0.0); 8];
    let nin = 4i32;
    // V2T *in = rect; V2T *out = result;
    // The four passes std::swap(in, out) between them; here we ping-pong
    // between `rect` and `result`.
    // Clip left:   in = rect,   out = result
    let nout = clip_pass(rect, nin, &mut result, aabb, ClipEdge::Left);
    // Clip bottom: in = result, out = rect
    let nout = clip_pass(&result, nout, rect, aabb, ClipEdge::Bottom);
    // Clip right:  in = rect,   out = result
    let nout = clip_pass(rect, nout, &mut result, aabb, ClipEdge::Right);
    // Clip top:    in = result, out = rect
    let nout = clip_pass(&result, nout, rect, aabb, ClipEdge::Top);
    // Final results land in `rect`, matching the C++ which leaves the result in
    // the `rect` buffer after the 4 swaps.
    debug_assert!(nout <= 8);
    nout
}

// Calculate area of the circle x AABB intersection.
// The calculation is approximate in a way, that the circular segment
// intersecting the cell is approximated by its chord (a linear segment).
// ExtrusionSimulator.cpp:376-434
fn clip_circle_by_aabb(
    center: V2f,
    radius: f32,
    aabb: &B2f,
    result: &mut [V2f; 8],
    result_arc: &mut [bool; 8],
) -> i32 {
    let rect: [V2f; 4] = [
        aabb.min_corner(),
        V2f::new(aabb.max_corner().x(), aabb.min_corner().y()),
        aabb.max_corner(),
        V2f::new(aabb.min_corner().x(), aabb.max_corner().y()),
    ];

    let mut bits_corners: i32 = 0;
    let r2 = sqr(radius);
    for i in 0..4 {
        bits_corners |= (dot1(sub(rect[i], center)) >= r2) as i32;
        bits_corners <<= 1;
    }
    bits_corners >>= 1;

    if bits_corners == 0 {
        // all inside
        // memcpy(result, rect, sizeof(rect));
        result[..4].copy_from_slice(&rect);
        // memset(result_arc, true, 4);
        for k in 0..4 {
            result_arc[k] = true;
        }
        return 4;
    }

    if bits_corners == 0x0f {
        // all outside
        return 0;
    }

    // Some corners are outside, some are inside. Trim the rectangle.
    let mut n = 0i32;
    for i in 0..4 {
        let inside = (bits_corners & 0x08) == 0;
        bits_corners <<= 1;
        let mut chordal_points: [V2f; 2] = [V2f::new(0.0, 0.0); 2];
        let n_chordal_points =
            line_circle_intersection(rect[i], rect[(i + 1) % 4], center, radius, &mut chordal_points);
        if n_chordal_points == 2 {
            result_arc[n as usize] = true;
            result[n as usize] = chordal_points[0];
            n += 1;
            result_arc[n as usize] = true;
            result[n as usize] = chordal_points[1];
            n += 1;
        } else {
            if inside {
                result_arc[n as usize] = false;
                result[n as usize] = rect[i];
                n += 1;
            }
            if n_chordal_points == 1 {
                result_arc[n as usize] = false;
                result[n as usize] = chordal_points[0];
                n += 1;
            }
        }
    }
    n
}

// ExtrusionSimulator.cpp:435-509: a large commented-out alternate implementation
// of circle_AABB_intersection_area is omitted here (it is dead code in C++).

// ExtrusionSimulator.cpp:511-518
#[inline]
fn poly_area(poly: &[V2f], n: i32) -> f32 {
    let mut area = 0.0f32;
    let mut i = 1i32;
    while i + 1 < n {
        area += cross(
            sub(poly[i as usize], poly[0]),
            sub(poly[(i + 1) as usize], poly[0]),
        );
        i += 1;
    }
    0.5f32 * area
}

// ExtrusionSimulator.cpp:520-527
#[allow(dead_code)]
fn poly_centroid(poly: &[V2f], n: i32) -> V2f {
    let mut centroid = V2f::new(0.0f32, 0.0f32);
    for i in 0..n as usize {
        centroid = add(centroid, poly[i]);
    }
    if n == 0 {
        centroid
    } else {
        div(centroid, n as f32)
    }
}

// ---------------------------------------------------------------------------
// 2D row-major grids, replacing boost::multi_array<T, 2>.
// shape()[0] == rows (nr), shape()[1] == cols (nc); indexing acc[j][i].
// ---------------------------------------------------------------------------

// typedef boost::multi_array<unsigned char, 2> A2uc;  ExtrusionSimulator.cpp:62
// typedef boost::multi_array<float        , 2> A2f;   ExtrusionSimulator.cpp:64
#[derive(Debug, Clone, Default)]
struct A2f {
    rows: usize,
    cols: usize,
    data: Vec<f32>,
}

#[derive(Debug, Clone, Default)]
struct A2uc {
    rows: usize,
    cols: usize,
    data: Vec<u8>,
}

impl A2f {
    fn new() -> Self {
        A2f {
            rows: 0,
            cols: 0,
            data: Vec::new(),
        }
    }
    // boost::extents[nr][nc]
    fn with_extents(nr: usize, nc: usize) -> Self {
        A2f {
            rows: nr,
            cols: nc,
            data: vec![0.0f32; nr * nc],
        }
    }
    // shape()[0] == rows, shape()[1] == cols
    #[inline]
    fn shape0(&self) -> usize {
        self.rows
    }
    #[inline]
    fn shape1(&self) -> usize {
        self.cols
    }
    #[inline]
    fn get(&self, j: usize, i: usize) -> f32 {
        self.data[j * self.cols + i]
    }
    #[inline]
    fn set(&mut self, j: usize, i: usize, v: f32) {
        self.data[j * self.cols + i] = v;
    }
    #[inline]
    fn add_at(&mut self, j: usize, i: usize, v: f32) {
        self.data[j * self.cols + i] += v;
    }
    fn resize(&mut self, nr: usize, nc: usize) {
        self.rows = nr;
        self.cols = nc;
        self.data = vec![0.0f32; nr * nc];
    }
}

impl A2uc {
    fn new() -> Self {
        A2uc {
            rows: 0,
            cols: 0,
            data: Vec::new(),
        }
    }
    #[inline]
    fn shape0(&self) -> usize {
        self.rows
    }
    #[inline]
    fn shape1(&self) -> usize {
        self.cols
    }
    #[inline]
    fn get(&self, j: usize, i: usize) -> u8 {
        self.data[j * self.cols + i]
    }
    #[inline]
    fn set(&mut self, j: usize, i: usize, v: u8) {
        self.data[j * self.cols + i] = v;
    }
    fn resize(&mut self, nr: usize, nc: usize) {
        self.rows = nr;
        self.cols = nc;
        self.data = vec![0u8; nr * nc];
    }
}

// std::clamp(v, lo, hi)
#[inline]
fn clamp_i32(v: i32, lo: i32, hi: i32) -> i32 {
    if v < lo {
        lo
    } else if v > hi {
        hi
    } else {
        v
    }
}

#[inline]
fn clamp_f32(v: f32, lo: f32, hi: f32) -> f32 {
    if v < lo {
        lo
    } else if v > hi {
        hi
    } else {
        v
    }
}

// ExtrusionSimulator.cpp:529-573
fn gcode_paint_layer(polyline: &[V2f], width: f32, thickness: f32, acc: &mut A2f) {
    let nc = acc.shape1() as i32;
    let nr = acc.shape0() as i32;
    //	printf("gcode_paint_layer %d,%d\n", nc, nr);
    for i_line in 1..polyline.len() {
        let p1 = polyline[i_line - 1];
        let p2 = polyline[i_line];
        // printf("p1, p2:  %f,%f %f,%f\n", p1.x(), p1.y(), p2.x(), p2.y());
        let dir = sub(p2, p1);
        let mut vperp = V2f::new(-dir.y(), dir.x());
        vperp = mul(vperp, 0.5f32 * width / l2(vperp));
        // Rectangle of the extrusion.
        let rect: [V2f; 4] = [
            add(p1, vperp),
            sub(p1, vperp),
            sub(p2, vperp),
            add(p2, vperp),
        ];
        // Bounding box of the extrusion.
        let mut bbox_line = B2f::new(rect[0], rect[0]);
        bbox_line.expand(rect[1]);
        bbox_line.expand(rect[2]);
        bbox_line.expand(rect[3]);
        let bbox_linei = B2i::new(
            V2i::new(
                clamp_i32(bbox_line.min_corner().x().floor() as i32, 0, nc - 1),
                clamp_i32(bbox_line.min_corner().y().floor() as i32, 0, nr - 1),
            ),
            V2i::new(
                clamp_i32(bbox_line.max_corner().x().ceil() as i32, 0, nc - 1),
                clamp_i32(bbox_line.max_corner().y().ceil() as i32, 0, nr - 1),
            ),
        );
        // printf("bboxLinei %d,%d %d,%d\n", ...);
        // #ifdef _DEBUG
        //     float area = polyArea(rect, 4);
        //     assert(area > 0.f);
        // #endif /* _DEBUG */
        let mut j = bbox_linei.min_corner().y();
        while j + 1 < bbox_linei.max_corner().y() {
            let mut i = bbox_linei.min_corner().x();
            while i + 1 < bbox_linei.max_corner().x() {
                let mut rect2: [V2f; 8] = [V2f::new(0.0, 0.0); 8];
                // memcpy(rect2, rect, sizeof(rect));
                rect2[..4].copy_from_slice(&rect);
                let n = clip_rect_by_aabb(
                    &mut rect2,
                    &B2f::new(
                        V2f::new(i as f32, j as f32),
                        V2f::new((i + 1) as f32, (j + 1) as f32),
                    ),
                );
                let area = poly_area(&rect2, n);
                debug_assert!(area >= 0.0f32 && area <= 1.000001f32);
                acc.add_at(j as usize, i as usize, area * thickness);
                i += 1;
            }
            j += 1;
        }
    }
}

// ExtrusionSimulator.cpp:575-613
fn gcode_paint_bitmap(polyline: &[V2f], width: f32, bitmap: &mut A2uc, scale: f32) {
    let nc = bitmap.shape1() as i32;
    let nr = bitmap.shape0() as i32;
    let r2 = width * width * 0.25f32;
    //	printf("gcode_paint_layer %d,%d\n", nc, nr);
    for i_line in 1..polyline.len() {
        let p1 = polyline[i_line - 1];
        let p2 = polyline[i_line];
        // printf("p1, p2:  %f,%f %f,%f\n", p1.x(), p1.y(), p2.x(), p2.y());
        let mut dir = sub(p2, p1);
        dir = mul(dir, 0.5f32 * width / l2(dir));
        let vperp = V2f::new(-dir.y(), dir.x());
        // Rectangle of the extrusion.
        let rect: [V2f; 4] = [
            mul(sub(add(p1, vperp), dir), scale),
            mul(sub(sub(p1, vperp), dir), scale),
            mul(add(sub(p2, vperp), dir), scale),
            mul(add(add(p2, vperp), dir), scale),
        ];
        // Bounding box of the extrusion.
        let mut bbox_line = B2f::new(rect[0], rect[0]);
        bbox_line.expand(rect[1]);
        bbox_line.expand(rect[2]);
        bbox_line.expand(rect[3]);
        let bbox_linei = B2i::new(
            V2i::new(
                clamp_i32(bbox_line.min_corner().x().floor() as i32, 0, nc - 1),
                clamp_i32(bbox_line.min_corner().y().floor() as i32, 0, nr - 1),
            ),
            V2i::new(
                clamp_i32(bbox_line.max_corner().x().ceil() as i32, 0, nc - 1),
                clamp_i32(bbox_line.max_corner().y().ceil() as i32, 0, nr - 1),
            ),
        );
        // printf("bboxLinei %d,%d %d,%d\n", ...);
        let mut j = bbox_linei.min_corner().y();
        while j + 1 < bbox_linei.max_corner().y() {
            let mut i = bbox_linei.min_corner().x();
            while i + 1 < bbox_linei.max_corner().x() {
                let d2 = dist2_to_line(
                    p1,
                    p2,
                    div(V2f::new(i as f32 + 0.5f32, j as f32 + 0.5f32), scale),
                );
                if d2 < r2 {
                    bitmap.set(j as usize, i as usize, 1);
                }
                i += 1;
            }
            j += 1;
        }
    }
}

// ExtrusionSimulator.cpp:615-631
#[derive(Debug, Clone, Copy, Default)]
struct Cell {
    // Cell index in the grid.
    idx: V2i,
    // Total volume of the material stored in this cell.
    volume: f32,
    // Area covered inside this cell, <0,1>.
    area: f32,
    // Fraction of the area covered by the print head. <0,1>
    fraction_covered: f32,
    // Height of the covered part in excess to the expected layer height.
    excess_height: f32,
}

impl Default for V2i {
    fn default() -> Self {
        V2i { x: 0, y: 0 }
    }
}

impl Cell {
    // bool operator<(const Cell &c2) const {
    //     return this->excess_height < c2.excess_height;
    // }
    #[inline]
    fn less(&self, c2: &Cell) -> bool {
        self.excess_height < c2.excess_height
    }
}

// ExtrusionSimulator.cpp:633-637
#[derive(Debug, Clone, Copy)]
struct ExtrusionPoint {
    center: V2f,
    radius: f32,
    height: f32,
}

// typedef std::vector<ExtrusionPoint> ExtrusionPoints;  ExtrusionSimulator.cpp:639
type ExtrusionPoints = Vec<ExtrusionPoint>;

// ExtrusionSimulator.cpp:641-847
fn gcode_spread_points(
    acc: &mut A2f,
    mask: &A2f,
    points: &ExtrusionPoints,
    simulation_type: ExtrusionSimulationType,
) {
    let nc = acc.shape1() as i32;
    let nr = acc.shape0() as i32;

    // Maximum radius of the spreading points, to allocate a large enough cell array.
    let mut rmax = 0.0f32;
    for it in points.iter() {
        rmax = rmax.max(it.radius);
    }
    let n_rows_max = (rmax * 2.0f32 + 2.0f32).ceil() as usize;
    let n_cells_max = sqr(n_rows_max as f32) as usize;
    let mut areas_sum: Vec<f32> = vec![0.0f32; n_cells_max];
    let mut cells: Vec<Cell> = vec![Cell::default(); n_cells_max];

    for it in points.iter() {
        let center = it.center;
        let radius = it.radius;
        //const float radius2 = radius * radius;
        let height_target = it.height;
        let bbox = B2f::new(
            sub(center, V2f::new(radius, radius)),
            add(center, V2f::new(radius, radius)),
        );
        let bboxi = B2i::new(
            V2i::new(
                clamp_i32(bbox.min_corner().x().floor() as i32, 0, nc - 1),
                clamp_i32(bbox.min_corner().y().floor() as i32, 0, nr - 1),
            ),
            V2i::new(
                clamp_i32(bbox.max_corner().x().ceil() as i32, 0, nc - 1),
                clamp_i32(bbox.max_corner().y().ceil() as i32, 0, nr - 1),
            ),
        );
        /*
        // Fill in the spans, at which the circle intersects the rows.
        ... (large commented-out block, ExtrusionSimulator.cpp:671-697)
        */
        let mut area_total = 0.0f32;
        let mut volume_total = 0.0f32;
        let mut volume_excess = 0.0f32;
        let mut volume_deficit = 0.0f32;
        let mut n_cells: usize = 0;
        let mut area_circle_total = 0.0f32;
        // #if 0 ... #else  (ExtrusionSimulator.cpp:704-741 disabled branch omitted)
        let mut j = bboxi.min_corner().y();
        while j < bboxi.max_corner().y() {
            let mut i = bboxi.min_corner().x();
            while i < bboxi.max_corner().x() {
                let bb = B2f::new(
                    V2f::new(i as f32, j as f32),
                    V2f::new((i + 1) as f32, (j + 1) as f32),
                );
                let mut poly: [V2f; 8] = [V2f::new(0.0, 0.0); 8];
                let mut poly_arc: [bool; 8] = [false; 8];
                let n = clip_circle_by_aabb(center, radius, &bb, &mut poly, &mut poly_arc);
                let area = poly_area(&poly, n);
                debug_assert!(area >= 0.0f32 && area <= 1.000001f32);
                if area == 0.0f32 {
                    i += 1;
                    continue;
                }
                let mut cell = Cell::default();
                cell.idx.set_x(i);
                cell.idx.set_y(j);
                cell.volume = acc.get(j as usize, i as usize);
                cell.area = mask.get(j as usize, i as usize);
                debug_assert!(cell.area >= 0.0f32 && cell.area <= 1.000001f32);
                area_circle_total += area;
                if cell.area < area {
                    cell.area = area;
                }
                cell.fraction_covered =
                    clamp_f32(if cell.area > 0.0 { area / cell.area } else { 0.0 }, 0.0f32, 1.0f32);
                if cell.fraction_covered == 0.0 {
                    // -- n_cells;  (the slot is simply not committed)
                    i += 1;
                    continue;
                }
                let cell_height = cell.volume / cell.area;
                cell.excess_height = cell_height - height_target;
                if cell.excess_height > 0.0f32 {
                    volume_excess += cell.excess_height * cell.area * cell.fraction_covered;
                } else {
                    volume_deficit -= cell.excess_height * cell.area * cell.fraction_covered;
                }
                volume_total += cell.volume * cell.fraction_covered;
                area_total += cell.area * cell.fraction_covered;
                cells[n_cells] = cell;
                n_cells += 1;
                i += 1;
            }
            j += 1;
        }
        // #endif
        //		float area_circle_total2 = float(M_PI) * sqr(radius);
        //		...
        let _ = area_circle_total;
        let _ = volume_excess;
        let _ = volume_deficit;
        let volume_full = (M_PI as f32) * sqr(radius) * height_target;
        //		if (true) { ... }
        if simulation_type == ExtrusionSimulationSpreadFull || volume_total <= volume_full {
            // The volume under the circle is spreaded fully.
            let height_avg = volume_total / area_total;
            for i in 0..n_cells {
                let cell = cells[i];
                acc.set(
                    cell.idx.y() as usize,
                    cell.idx.x() as usize,
                    (1.0f32 - cell.fraction_covered) * cell.volume
                        + cell.fraction_covered * cell.area * height_avg,
                );
            }
        } else if simulation_type == ExtrusionSimulationSpreadExcess {
            // The volume under the circle does not fit.
            // 1) Fill the underfilled cells and remove them from the list.
            let mut volume_borrowed_total = 0.0f32;
            {
                let mut i = 0usize;
                while i < n_cells {
                    let cell = cells[i];
                    if cell.excess_height <= 0.0 {
                        // Fill in the part of the cell below the circle.
                        let volume_borrowed =
                            -cell.excess_height * cell.area * cell.fraction_covered;
                        debug_assert!(volume_borrowed >= 0.0f32);
                        acc.set(
                            cell.idx.y() as usize,
                            cell.idx.x() as usize,
                            cell.volume + volume_borrowed,
                        );
                        volume_borrowed_total += volume_borrowed;
                        // cell = cells[-- n_cells];
                        n_cells -= 1;
                        cells[i] = cells[n_cells];
                    } else {
                        i += 1;
                    }
                }
            }
            // 2) Sort the remaining cells by their excess height.
            // std::sort(cells.begin(), cells.begin() + n_cells);
            cells[..n_cells].sort_by(|a, b| {
                if a.less(b) {
                    std::cmp::Ordering::Less
                } else if b.less(a) {
                    std::cmp::Ordering::Greater
                } else {
                    std::cmp::Ordering::Equal
                }
            });
            // 3) Prefix sum the areas per excess height.
            // The excess height is discrete with the number of excess cells.
            areas_sum[n_cells - 1] =
                cells[n_cells - 1].area * cells[n_cells - 1].fraction_covered;
            {
                let mut i = n_cells as i64 - 2;
                while i >= 0 {
                    let cell = cells[i as usize];
                    areas_sum[i as usize] =
                        areas_sum[(i + 1) as usize] + cell.area * cell.fraction_covered;
                    i -= 1;
                }
            }
            // 4) Find the excess height, where the volume_excess is over the volume_borrowed_total.
            let mut volume_current = 0.0f32;
            let mut excess_height_prev = 0.0f32;
            let mut i_top: usize = n_cells;
            for i in 0..n_cells {
                let cell = cells[i];
                volume_current += (cell.excess_height - excess_height_prev) * areas_sum[i];
                excess_height_prev = cell.excess_height;
                if volume_current > volume_borrowed_total {
                    i_top = i;
                    break;
                }
            }
            // 5) Remove material from the cells with deficit.
            // First remove all the excess material from the cells, where the deficit is low.
            for i in 0..i_top {
                let cell = cells[i];
                let volume_removed = cell.excess_height * cell.area * cell.fraction_covered;
                acc.set(
                    cell.idx.y() as usize,
                    cell.idx.x() as usize,
                    cell.volume - volume_removed,
                );
                volume_borrowed_total -= volume_removed;
            }
            // Second remove some excess material from the cells, where the deficit is high.
            if i_top < n_cells {
                let height_diff = volume_borrowed_total / areas_sum[i_top];
                for i in i_top..n_cells {
                    let cell = cells[i];
                    acc.set(
                        cell.idx.y() as usize,
                        cell.idx.x() as usize,
                        cell.volume - height_diff * cell.area * cell.fraction_covered,
                    );
                }
            }
        }
    }
}

// ExtrusionSimulator.cpp:849-867
#[inline]
fn create_power_color_gradient_24bit() -> Vec<V3uc> {
    let mut i;
    let mut i_color = 0usize;
    let mut out: Vec<V3uc> = vec![V3uc(0, 0, 0); 6 * 255 + 1];
    i = 0;
    while i < 256 {
        out[i_color] = V3uc(0, 0, i as u8);
        i_color += 1;
        i += 1;
    }
    i = 1;
    while i < 256 {
        out[i_color] = V3uc(0, i as u8, 255);
        i_color += 1;
        i += 1;
    }
    i = 1;
    while i < 256 {
        out[i_color] = V3uc(0, 255, (256 - i) as u8);
        i_color += 1;
        i += 1;
    }
    i = 1;
    while i < 256 {
        out[i_color] = V3uc(i as u8, 255, 0);
        i_color += 1;
        i += 1;
    }
    i = 1;
    while i < 256 {
        out[i_color] = V3uc(255, (256 - i) as u8, 0);
        i_color += 1;
        i += 1;
    }
    i = 1;
    while i < 256 {
        out[i_color] = V3uc(255, 0, i as u8);
        i_color += 1;
        i += 1;
    }
    out
}

// ExtrusionSimulator.cpp:869-878
struct ExtrusionSimulatorImpl {
    image_data: Vec<u8>,
    accumulator: A2f,
    bitmap: A2uc,
    bitmap_oversampled: u32,
    extrusion_points: ExtrusionPoints,
    // RGB gradient to color map the fullness of an accumulator bucket into the output image.
    color_gradient: Vec<V3uc>,
}

// ExtrusionSimulator.hpp:22-55
pub struct ExtrusionSimulator {
    // ExtrusionSimulator.hpp:50-52
    image_size: Point,
    viewport: BoundingBox,
    bbox: BoundingBox,

    // ExtrusionSimulator.hpp:54
    pimpl: Box<ExtrusionSimulatorImpl>,
}

impl ExtrusionSimulator {
    // ExtrusionSimulator.cpp:880-885
    // ExtrusionSimulator::ExtrusionSimulator() : pimpl(new ExtrusionSimulatorImpl)
    pub fn new() -> Self {
        let mut pimpl = Box::new(ExtrusionSimulatorImpl {
            image_data: Vec::new(),
            accumulator: A2f::new(),
            bitmap: A2uc::new(),
            bitmap_oversampled: 0,
            extrusion_points: Vec::new(),
            color_gradient: Vec::new(),
        });
        pimpl.color_gradient = create_power_color_gradient_24bit();
        pimpl.bitmap_oversampled = 4;
        ExtrusionSimulator {
            image_size: Point::new(0, 0),
            viewport: BoundingBox::new(),
            bbox: BoundingBox::new(),
            pimpl,
        }
    }

    // ExtrusionSimulator.cpp:887-891 (~ExtrusionSimulator) handled by Drop of Box.

    // ExtrusionSimulator.cpp:893-917
    pub fn set_image_size(&mut self, image_size: &Point) {
        // printf("ExtrusionSimulator::set_image_size()\n");
        if self.image_size.x() == image_size.x() && self.image_size.y() == image_size.y() {
            return;
        }

        // printf("Setting image size: %d, %d\n", image_size.x, image_size.y);
        self.image_size = *image_size;
        // Allocate the image data in an RGBA format.
        // printf("Allocating image data, size %d\n", image_size.x * image_size.y * 4);
        self.pimpl
            .image_data
            .clear();
        self.pimpl.image_data.resize(
            (image_size.x() * image_size.y() * 4) as usize,
            0,
        );
        // printf("Allocating image data, allocated\n");

        //FIXME fill the image with red vertical lines.
        for r in 0..image_size.y() as usize {
            let mut c = 0usize;
            while c < image_size.x() as usize {
                // Color red
                self.pimpl.image_data[r * image_size.x() as usize * 4 + c * 4] = 255;
                // Opacity full
                self.pimpl.image_data[r * image_size.x() as usize * 4 + c * 4 + 3] = 255;
                c += 2;
            }
        }
        // printf("Allocating image data, set\n");
    }

    // ExtrusionSimulator.cpp:919-929
    pub fn set_viewport(&mut self, viewport: &BoundingBox) {
        // printf("ExtrusionSimulator::set_viewport(...)\n");
        if self.viewport != *viewport {
            self.viewport = *viewport;
            let sz = viewport.size();
            self.pimpl
                .accumulator
                .resize(sz.y() as usize, sz.x() as usize);
            self.pimpl.bitmap.resize(
                (sz.y() * self.pimpl.bitmap_oversampled as i64) as usize,
                (sz.x() * self.pimpl.bitmap_oversampled as i64) as usize,
            );
            // printf("Accumulator size: %d, %d\n", sz.y, sz.x);
        }
    }

    // ExtrusionSimulator.cpp:931-934
    pub fn set_bounding_box(&mut self, bbox: &BoundingBox) {
        self.bbox = *bbox;
    }

    // ExtrusionSimulator.cpp:936-939
    pub fn image_ptr(&self) -> Option<&[u8]> {
        if self.pimpl.image_data.is_empty() {
            None
        } else {
            Some(&self.pimpl.image_data)
        }
    }

    // ExtrusionSimulator.cpp:941-950
    pub fn reset_accumulator(&mut self) {
        // printf("ExtrusionSimulator::reset_accumulator()\n");
        let sz = self.viewport.size();
        // printf("Reset accumulator, Accumulator size: %d, %d\n", sz.y, sz.x);
        // memset(&accumulator[0][0], 0, sizeof(float) * sz.x() * sz.y());
        let acc_count = (sz.x() * sz.y()) as usize;
        for k in 0..acc_count {
            self.pimpl.accumulator.data[k] = 0.0f32;
        }
        // memset(&bitmap[0][0], 0, sz.x()*sz.y()*oversampled*oversampled);
        let bmp_count = (sz.x()
            * sz.y()
            * self.pimpl.bitmap_oversampled as i64
            * self.pimpl.bitmap_oversampled as i64) as usize;
        for k in 0..bmp_count {
            self.pimpl.bitmap.data[k] = 0u8;
        }
        self.pimpl.extrusion_points.clear();
        // printf("Reset accumulator, done.\n");
    }

    // ExtrusionSimulator.cpp:952-983
    pub fn extrude_to_accumulator(
        &mut self,
        path: &ExtrusionPath,
        shift: &Point,
        simulation_type: ExtrusionSimulationType,
    ) {
        // printf("Extruding a path. ...\r\n");
        // Convert the path to V2f points, shift and scale them to the viewport.
        let mut polyline: Vec<V2f> = Vec::with_capacity(path.polyline.points.len());
        let scalex = self.viewport.size().x() as f32 / self.bbox.size().x() as f32;
        let scaley = self.viewport.size().y() as f32 / self.bbox.size().y() as f32;
        let mut w = scale_(path.width) as f32 * scalex;
        //float h = scale_(path.height) * scalex;
        w = scale_(path.mm3_per_mm / path.height) as f32 * scalex;
        // printf("scalex: %f, scaley: %f\n", scalex, scaley);
        // printf("bbox: %d,%d %d,%d\n", ...);
        for it in path.polyline.points.iter() {
            // printf("point %d,%d\n", it->x+shift.x(), it->y+shift.y);
            let center = V2f::new(
                ((*it).x() + shift.x() - self.bbox.min.x()) as f32 * scalex,
                ((*it).y() + shift.y() - self.bbox.min.y()) as f32 * scaley,
            );
            let ept = ExtrusionPoint {
                center,
                radius: w / 2.0f32,
                height: 0.5f32,
            };
            polyline.push(ept.center);
            self.pimpl.extrusion_points.push(ept);
        }
        // Extrude the polyline into an accumulator.
        // printf("width scaled: %f, height scaled: %f\n", w, h);
        gcode_paint_layer(&polyline, w, 0.5f32, &mut self.pimpl.accumulator);

        if simulation_type > ExtrusionSimulationDontSpread {
            let oversampled = self.pimpl.bitmap_oversampled as f32;
            gcode_paint_bitmap(&polyline, w, &mut self.pimpl.bitmap, oversampled);
        }
        // double path.mm3_per_mm;  // mm^3 of plastic per mm of linear head motion
        // float path.width;
        // float path.height;
    }

    // ExtrusionSimulator.cpp:985-1028
    pub fn evaluate_accumulator(&mut self, simulation_type: ExtrusionSimulationType) {
        // printf("ExtrusionSimulator::evaluate_accumulator()\n");
        let sz = self.viewport.size();

        if simulation_type > ExtrusionSimulationDontSpread {
            // Average the cells of a bitmap into a lower resolution floating point mask.
            let mut mask = A2f::with_extents(sz.y() as usize, sz.x() as usize);
            for r in 0..sz.y() as i32 {
                for c in 0..sz.x() as i32 {
                    let mut p = 0.0f32;
                    for j in 0..self.pimpl.bitmap_oversampled {
                        for i in 0..self.pimpl.bitmap_oversampled {
                            if self.pimpl.bitmap.get(
                                (r as u32 * self.pimpl.bitmap_oversampled + j) as usize,
                                (c as u32 * self.pimpl.bitmap_oversampled + i) as usize,
                            ) != 0
                            {
                                p += 1.0f32;
                            }
                        }
                    }
                    p /= (self.pimpl.bitmap_oversampled * self.pimpl.bitmap_oversampled * 2) as f32;
                    mask.set(r as usize, c as usize, p);
                }
            }

            // Spread the excess of the material.
            let points = std::mem::take(&mut self.pimpl.extrusion_points);
            gcode_spread_points(&mut self.pimpl.accumulator, &mask, &points, simulation_type);
            self.pimpl.extrusion_points = points;
        }

        // Color map the accumulator.
        for r in 0..sz.y() as i32 {
            // unsigned char *ptr = &image_data[(image_size.x() * (viewport.min.y() + r) + viewport.min.x()) * 4];
            let mut ptr = ((self.image_size.x() * (self.viewport.min.y() + r as i64)
                + self.viewport.min.x())
                * 4) as usize;
            for c in 0..sz.x() as i32 {
                // #if 1
                let p = self.pimpl.accumulator.get(r as usize, c as usize);
                // #else  float p = mask[r][c];  #endif
                let idx = (p * self.pimpl.color_gradient.len() as f32 + 0.5f32).floor() as i32;
                let clr = self.pimpl.color_gradient
                    [clamp_i32(idx, 0, (self.pimpl.color_gradient.len() - 1) as i32) as usize];
                self.pimpl.image_data[ptr] = clr.0;
                ptr += 1;
                self.pimpl.image_data[ptr] = clr.1;
                ptr += 1;
                self.pimpl.image_data[ptr] = clr.2;
                ptr += 1;
                self.pimpl.image_data[ptr] = if idx == 0 { 0 } else { 255 };
                ptr += 1;
            }
        }
    }
}

impl Default for ExtrusionSimulator {
    fn default() -> Self {
        Self::new()
    }
}
