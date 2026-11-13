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

// ---------------------------------------------------------------------------
// The clipper's own types and helpers (clipper.hpp:72-131, 216-248;
// clipper.cpp:74-85, 117-120, 316-474). They are `static` or `inline` in the
// C++, so the library cannot be asked for them one by one: they are checked
// through the sweep that uses them, once that is ported. Until then they are
// read against the C++, not tested against it.
// ---------------------------------------------------------------------------

/// clipper.hpp:72
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClipType {
    ctIntersection,
    ctUnion,
    ctDifference,
    ctXor,
}

/// clipper.hpp:73
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PolyType {
    #[default]
    ptSubject,
    ptClip,
}

/// clipper.hpp:78
/// By far the most widely used winding rules for polygon filling are EvenOdd & NonZero
/// (GDI, GDI+, XLib, OpenGL, Cairo, AGG, Quartz, SVG, Gr32). Others rules include
/// Positive, Negative and ABS_GTR_EQ_TWO (only in OpenGL).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PolyFillType {
    pftEvenOdd,
    pftNonZero,
    pftPositive,
    pftNegative,
}

/// clipper.hpp:130
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JoinType {
    jtSquare,
    jtRound,
    jtMiter,
}

/// clipper.hpp:131
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EndType {
    etClosedPolygon,
    etClosedLine,
    etOpenButt,
    etOpenSquare,
    etOpenRound,
}

/// clipper.hpp:216 `enum EdgeSide { esLeft = 1, esRight = 2};`. A `TEdge` cleared by
/// `InitEdge` has the side 0, which is neither.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum EdgeSide {
    #[default]
    esNone = 0,
    esLeft = 1,
    esRight = 2,
}

/// clipper.cpp:78 `enum Direction { dRightToLeft, dLeftToRight };`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    dRightToLeft,
    dLeftToRight,
}

/// clipper.cpp:74-76
pub const PI: f64 = 3.141592653589793238;
pub const TWO_PI: f64 = PI * 2.0;
pub const DEF_ARC_TOLERANCE: f64 = 0.25;

/// clipper.cpp:80 — edge not currently 'owning' a solution
pub const UNASSIGNED: i32 = -1;
/// clipper.cpp:81 — edge that would otherwise close a path
pub const SKIP: i32 = -2;

/// clipper.cpp:83 `#define HORIZONTAL (-1.0E+40)`
pub const HORIZONTAL: f64 = -1.0E+40;
/// clipper.cpp:84 `#define TOLERANCE (1.0e-20)`
pub const TOLERANCE: f64 = 1.0e-20;

/// clipper.cpp:85 `#define NEAR_ZERO(val) (((val) > -TOLERANCE) && ((val) < TOLERANCE))`
#[inline]
pub fn near_zero(val: f64) -> bool {
    val > -TOLERANCE && val < TOLERANCE
}

/// clipper.hpp:220-248 `struct TEdge`. The C++ links edges with pointers; here an
/// edge is an index into the vector that owns the edges, and a null pointer is `None`.
#[derive(Debug, Clone, Copy, Default)]
pub struct TEdge {
    /// Bottom point of this edge (with minimum Y).
    pub bot: IntPoint,
    /// Current position.
    pub curr: IntPoint,
    /// Top point of this edge (with maximum Y).
    pub top: IntPoint,
    /// Vector from Bot to Top.
    pub delta: IntPoint,
    /// Slope (dx/dy). For horizontal edges, the slope is set to HORIZONTAL (-1.0E+40).
    pub dx: f64,
    pub poly_typ: PolyType,
    pub side: EdgeSide,
    /// Winding number delta. 1 or -1 depending on winding direction, 0 for open paths and flattened segments.
    pub wind_delta: i32,
    pub wind_cnt: i32,
    /// Winding count of the opposite polytype.
    pub wind_cnt2: i32,
    pub out_idx: i32,
    /// Next edge in the input path.
    pub next: Option<usize>,
    /// Previous edge in the input path.
    pub prev: Option<usize>,
    /// Next edge in the Local Minima List chain.
    pub next_in_lml: Option<usize>,
    pub next_in_ael: Option<usize>,
    pub prev_in_ael: Option<usize>,
    pub next_in_sel: Option<usize>,
    pub prev_in_sel: Option<usize>,
}

/// clipper.cpp:117-120
/// C++: inline cInt Round(double val)
/// `static_cast<cInt>` truncates; a value out of range is undefined in C++ and
/// saturates here.
#[inline]
pub fn round(val: f64) -> cInt {
    (if val < 0.0 { val - 0.5 } else { val + 0.5 }) as cInt
}

/// clipper.cpp:317-319 (the `CLIPPERLIB_INT32` arm)
/// C++: inline bool SlopesEqual(const cInt dx1, const cInt dy1, const cInt dx2, const cInt dy2, bool)
#[inline]
pub fn slopes_equal(dx1: cInt, dy1: cInt, dx2: cInt, dy2: cInt) -> bool {
    (dy1 as i64) * (dx2 as i64) == (dx1 as i64) * (dy2 as i64)
}

/// clipper.cpp:332-333
/// C++: inline bool SlopesEqual(const TEdge &e1, const TEdge &e2, bool UseFullInt64Range)
#[inline]
pub fn slopes_equal_edges(e1: &TEdge, e2: &TEdge) -> bool {
    slopes_equal(e1.delta.x, e1.delta.y, e2.delta.x, e2.delta.y)
}

/// clipper.cpp:334-335
/// C++: inline bool SlopesEqual(const IntPoint &pt1, const IntPoint &pt2, const IntPoint &pt3, bool)
/// The differences are taken in cInt, as in the C++.
#[inline]
pub fn slopes_equal_3(pt1: &IntPoint, pt2: &IntPoint, pt3: &IntPoint) -> bool {
    slopes_equal(
        pt1.x.wrapping_sub(pt2.x),
        pt1.y.wrapping_sub(pt2.y),
        pt2.x.wrapping_sub(pt3.x),
        pt2.y.wrapping_sub(pt3.y),
    )
}

/// clipper.cpp:336-337
/// C++: inline bool SlopesEqual(const IntPoint &pt1, const IntPoint &pt2, const IntPoint &pt3, const IntPoint &pt4, bool)
#[inline]
pub fn slopes_equal_4(pt1: &IntPoint, pt2: &IntPoint, pt3: &IntPoint, pt4: &IntPoint) -> bool {
    slopes_equal(
        pt1.x.wrapping_sub(pt2.x),
        pt1.y.wrapping_sub(pt2.y),
        pt3.x.wrapping_sub(pt4.x),
        pt3.y.wrapping_sub(pt4.y),
    )
}

/// clipper.cpp:341-344
/// C++: inline bool IsHorizontal(TEdge &e)
#[inline]
pub fn is_horizontal(e: &TEdge) -> bool {
    e.delta.y == 0
}

/// clipper.cpp:347-351
/// C++: inline double GetDx(const IntPoint &pt1, const IntPoint &pt2)
#[inline]
pub fn get_dx(pt1: &IntPoint, pt2: &IntPoint) -> f64 {
    if pt1.y == pt2.y {
        HORIZONTAL
    } else {
        pt2.x.wrapping_sub(pt1.x) as f64 / pt2.y.wrapping_sub(pt1.y) as f64
    }
}

/// clipper.cpp:354-359
/// C++: inline cInt TopX(TEdge &edge, const cInt currentY)
#[inline]
pub fn top_x(edge: &TEdge, current_y: cInt) -> cInt {
    if current_y == edge.top.y {
        edge.top.x
    } else {
        edge.bot.x.wrapping_add(round(edge.dx * current_y.wrapping_sub(edge.bot.y) as f64))
    }
}

/// clipper.cpp:362-427
/// C++: void IntersectPoint(TEdge &Edge1, TEdge &Edge2, IntPoint &ip)
pub fn intersect_point(edge1: &TEdge, edge2: &TEdge, ip: &mut IntPoint) {
    if edge1.dx == edge2.dx {
        ip.y = edge1.curr.y;
        ip.x = top_x(edge1, ip.y);
        return;
    } else if edge1.delta.x == 0 {
        ip.x = edge1.bot.x;
        if is_horizontal(edge2) {
            ip.y = edge2.bot.y;
        } else {
            let b2 = edge2.bot.y as f64 - (edge2.bot.x as f64 / edge2.dx);
            ip.y = round(ip.x as f64 / edge2.dx + b2);
        }
    } else if edge2.delta.x == 0 {
        ip.x = edge2.bot.x;
        if is_horizontal(edge1) {
            ip.y = edge1.bot.y;
        } else {
            let b1 = edge1.bot.y as f64 - (edge1.bot.x as f64 / edge1.dx);
            ip.y = round(ip.x as f64 / edge1.dx + b1);
        }
    } else {
        // b1 = double(Edge1.Bot.x()) - double(Edge1.Bot.y()) * Edge1.Dx;
        // `a - b * c` and `a * b + c` are each one expression, contracted by the C++ build.
        let b1 = (-(edge1.bot.y as f64)).mul_add(edge1.dx, edge1.bot.x as f64);
        let b2 = (-(edge2.bot.y as f64)).mul_add(edge2.dx, edge2.bot.x as f64);
        let q = (b2 - b1) / (edge1.dx - edge2.dx);
        ip.y = round(q);
        ip.x = if edge1.dx.abs() < edge2.dx.abs() {
            round(edge1.dx.mul_add(q, b1))
        } else {
            round(edge2.dx.mul_add(q, b2))
        };
    }

    if ip.y < edge1.top.y || ip.y < edge2.top.y {
        if edge1.top.y > edge2.top.y {
            ip.y = edge1.top.y;
        } else {
            ip.y = edge2.top.y;
        }
        if edge1.dx.abs() < edge2.dx.abs() {
            ip.x = top_x(edge1, ip.y);
        } else {
            ip.x = top_x(edge2, ip.y);
        }
    }
    // finally, don't allow 'ip' to be BELOW curr.y() (ie bottom of scanbeam) ...
    if ip.y > edge1.curr.y {
        ip.y = edge1.curr.y;
        // use the more vertical edge to derive X ...
        if edge1.dx.abs() > edge2.dx.abs() {
            ip.x = top_x(edge2, ip.y);
        } else {
            ip.x = top_x(edge1, ip.y);
        }
    }
}

/// clipper.cpp:433-443
/// C++: void ReversePolyPtLinks(OutPt *pp)
/// Reverse a linked loop of points representing a closed polygon. O(n).
pub fn reverse_poly_pt_links(out_pts: &mut [OutPt], pp: Option<usize>) {
    let Some(pp) = pp else { return };
    let mut pp1 = pp;
    loop {
        let pp2 = out_pts[pp1].next;
        out_pts[pp1].next = out_pts[pp1].prev;
        out_pts[pp1].prev = pp2;
        pp1 = pp2;
        if pp1 == pp {
            break;
        }
    }
}

/// clipper.cpp:446-453
/// C++: inline void InitEdge(TEdge* e, TEdge* eNext, TEdge* ePrev, const IntPoint& Pt)
#[inline]
pub fn init_edge(edges: &mut [TEdge], e: usize, e_next: usize, e_prev: usize, pt: &IntPoint) {
    // std::memset(e, 0, sizeof(TEdge));
    edges[e] = TEdge::default();
    edges[e].next = Some(e_next);
    edges[e].prev = Some(e_prev);
    edges[e].curr = *pt;
    edges[e].out_idx = UNASSIGNED;
}

/// clipper.cpp:456-474
/// C++: void InitEdge2(TEdge& e, PolyType Pt)
pub fn init_edge2(edges: &mut [TEdge], e: usize, pt: PolyType) {
    let next_curr = edges[edges[e].next.expect("InitEdge sets Next")].curr;
    let e = &mut edges[e];
    if e.curr.y >= next_curr.y {
        e.bot = e.curr;
        e.top = next_curr;
    } else {
        e.top = e.curr;
        e.bot = next_curr;
    }

    e.delta.x = e.top.x.wrapping_sub(e.bot.x);
    e.delta.y = e.top.y.wrapping_sub(e.bot.y);

    if e.delta.y == 0 {
        e.dx = HORIZONTAL;
    } else {
        e.dx = e.delta.x as f64 / e.delta.y as f64;
    }

    e.poly_typ = pt;
}

/// clipper.cpp:479-487
/// C++: inline TEdge* RemoveEdge(TEdge* e)
/// Called from ClipperBase::AddPathInternal() to remove collinear and duplicate points.
#[inline]
pub fn remove_edge(edges: &mut [TEdge], e: usize) -> usize {
    // removes e from double_linked_list (but without removing from memory)
    let prev = edges[e].prev.expect("an edge in the list has a Prev");
    let next = edges[e].next.expect("an edge in the list has a Next");
    edges[prev].next = Some(next);
    edges[next].prev = Some(prev);
    let result = next;
    edges[e].prev = None; // flag as removed (see ClipperBase.Clear)
    result
}

/// clipper.cpp:490-499
/// C++: inline void ReverseHorizontal(TEdge &e)
#[inline]
pub fn reverse_horizontal(e: &mut TEdge) {
    // swap horizontal edges' Top and Bottom x's so they follow the natural
    // progression of the bounds - ie so their xbots will align with the
    // adjoining lower edge. [Helpful in the ProcessHorizontal() method.]
    std::mem::swap(&mut e.top.x, &mut e.bot.x);
}

/// clipper.cpp:502-521
/// C++: bool GetOverlapSegment(IntPoint pt1a, IntPoint pt1b, IntPoint pt2a, IntPoint pt2b, IntPoint &pt1, IntPoint &pt2)
pub fn get_overlap_segment(
    mut pt1a: IntPoint,
    mut pt1b: IntPoint,
    mut pt2a: IntPoint,
    mut pt2b: IntPoint,
    pt1: &mut IntPoint,
    pt2: &mut IntPoint,
) -> bool {
    // precondition: segments are Collinear.
    if pt1a.x.wrapping_sub(pt1b.x).wrapping_abs() > pt1a.y.wrapping_sub(pt1b.y).wrapping_abs() {
        if pt1a.x > pt1b.x {
            std::mem::swap(&mut pt1a, &mut pt1b);
        }
        if pt2a.x > pt2b.x {
            std::mem::swap(&mut pt2a, &mut pt2b);
        }
        *pt1 = if pt1a.x > pt2a.x { pt1a } else { pt2a };
        *pt2 = if pt1b.x < pt2b.x { pt1b } else { pt2b };
        pt1.x < pt2.x
    } else {
        if pt1a.y < pt1b.y {
            std::mem::swap(&mut pt1a, &mut pt1b);
        }
        if pt2a.y < pt2b.y {
            std::mem::swap(&mut pt2a, &mut pt2b);
        }
        *pt1 = if pt1a.y < pt2a.y { pt1a } else { pt2a };
        *pt2 = if pt1b.y > pt2b.y { pt1b } else { pt2b };
        pt1.y > pt2.y
    }
}

/// clipper.cpp:524-540
/// C++: bool FirstIsBottomPt(const OutPt* btmPt1, const OutPt* btmPt2)
pub fn first_is_bottom_pt(out_pts: &[OutPt], btm_pt1: usize, btm_pt2: usize) -> bool {
    let mut p = out_pts[btm_pt1].prev;
    while out_pts[p].pt == out_pts[btm_pt1].pt && p != btm_pt1 {
        p = out_pts[p].prev;
    }
    let dx1p = get_dx(&out_pts[btm_pt1].pt, &out_pts[p].pt).abs();
    p = out_pts[btm_pt1].next;
    while out_pts[p].pt == out_pts[btm_pt1].pt && p != btm_pt1 {
        p = out_pts[p].next;
    }
    let dx1n = get_dx(&out_pts[btm_pt1].pt, &out_pts[p].pt).abs();

    p = out_pts[btm_pt2].prev;
    while out_pts[p].pt == out_pts[btm_pt2].pt && p != btm_pt2 {
        p = out_pts[p].prev;
    }
    let dx2p = get_dx(&out_pts[btm_pt2].pt, &out_pts[p].pt).abs();
    p = out_pts[btm_pt2].next;
    while out_pts[p].pt == out_pts[btm_pt2].pt && p != btm_pt2 {
        p = out_pts[p].next;
    }
    let dx2n = get_dx(&out_pts[btm_pt2].pt, &out_pts[p].pt).abs();
    (dx1p >= dx2p && dx1p >= dx2n) || (dx1n >= dx2p && dx1n >= dx2n)
}

/// clipper.cpp:544-579
/// C++: OutPt* GetBottomPt(OutPt *pp)
/// Called by GetLowermostRec()
pub fn get_bottom_pt(out_pts: &[OutPt], mut pp: usize) -> usize {
    let mut dups: Option<usize> = None;
    let mut p = out_pts[pp].next;
    while p != pp {
        if out_pts[p].pt.y > out_pts[pp].pt.y {
            pp = p;
            dups = None;
        } else if out_pts[p].pt.y == out_pts[pp].pt.y && out_pts[p].pt.x <= out_pts[pp].pt.x {
            if out_pts[p].pt.x < out_pts[pp].pt.x {
                dups = None;
                pp = p;
            } else if out_pts[p].next != pp && out_pts[p].prev != pp {
                dups = Some(p);
            }
        }
        p = out_pts[p].next;
    }
    if let Some(mut dups) = dups {
        // there appears to be at least 2 vertices at BottomPt so ...
        while dups != p {
            if !first_is_bottom_pt(out_pts, p, dups) {
                pp = dups;
            }
            dups = out_pts[dups].next;
            while out_pts[dups].pt != out_pts[pp].pt {
                dups = out_pts[dups].next;
            }
        }
    }
    pp
}

/// clipper.cpp:582-591
/// C++: bool Pt2IsBetweenPt1AndPt3(const IntPoint &pt1, const IntPoint &pt2, const IntPoint &pt3)
pub fn pt2_is_between_pt1_and_pt3(pt1: &IntPoint, pt2: &IntPoint, pt3: &IntPoint) -> bool {
    if pt1 == pt3 || pt1 == pt2 || pt3 == pt2 {
        false
    } else if pt1.x != pt3.x {
        (pt2.x > pt1.x) == (pt2.x < pt3.x)
    } else {
        (pt2.y > pt1.y) == (pt2.y < pt3.y)
    }
}

/// clipper.cpp:594-599
/// C++: bool HorzSegmentsOverlap(cInt seg1a, cInt seg1b, cInt seg2a, cInt seg2b)
pub fn horz_segments_overlap(mut seg1a: cInt, mut seg1b: cInt, mut seg2a: cInt, mut seg2b: cInt) -> bool {
    if seg1a > seg1b {
        std::mem::swap(&mut seg1a, &mut seg1b);
    }
    if seg2a > seg2b {
        std::mem::swap(&mut seg2a, &mut seg2b);
    }
    seg1a < seg2b && seg2a < seg1b
}

/// clipper.cpp:625-640
/// C++: inline TEdge* FindNextLocMin(TEdge* E)
/// Called from ClipperBase::AddPath() to construct the Local Minima List.
/// Find a local minimum edge on the path starting with E.
#[inline]
pub fn find_next_loc_min(edges: &[TEdge], mut e: usize) -> usize {
    let prev = |i: usize| edges[i].prev.expect("an edge in the list has a Prev");
    let next = |i: usize| edges[i].next.expect("an edge in the list has a Next");
    loop {
        while edges[e].bot != edges[prev(e)].bot || edges[e].curr == edges[e].top {
            e = next(e);
        }
        if !is_horizontal(&edges[e]) && !is_horizontal(&edges[prev(e)]) {
            break;
        }
        while is_horizontal(&edges[prev(e)]) {
            e = prev(e);
        }
        let e2 = e;
        while is_horizontal(&edges[e]) {
            e = next(e);
        }
        if edges[e].top.y == edges[prev(e)].bot.y {
            continue; // ie just an intermediate horz.
        }
        if edges[prev(e2)].bot.x < edges[e].bot.x {
            e = e2;
        }
        break;
    }
    e
}

/// clipper.hpp:258-262 `struct LocalMinimum`: a local minimum of the input, with the two
/// bounds that rise from it.
#[derive(Debug, Clone, Copy, Default)]
pub struct LocalMinimum {
    pub y: cInt,
    pub left_bound: Option<usize>,
    pub right_bound: Option<usize>,
}

/// clipper.hpp `class ClipperBase`, as far as it is ported: the edges of the
/// input paths and the local minima list. The C++ keeps one vector of edges per
/// path (`m_edges`); here they are in one vector and an edge is its index.
#[derive(Debug, Clone, Default)]
pub struct ClipperBase {
    pub m_edges: Vec<TEdge>,
    pub m_minima_list: Vec<LocalMinimum>,
    /// clipper.hpp: `bool m_PreserveCollinear;`
    pub m_preserve_collinear: bool,
    /// clipper.hpp: `bool m_HasOpenPaths;`
    pub m_has_open_paths: bool,
}

impl ClipperBase {
    /// clipper.cpp:760-782
    /// C++: bool ClipperBase::AddPath(const Path &pg, PolyType PolyTyp, bool Closed)
    pub fn add_path(&mut self, pg: &Path, poly_typ: PolyType, closed: bool) -> bool {
        // Remove duplicate end point from a closed input path.
        // Remove duplicate points from the end of the input path.
        let mut high_i = pg.len() as i32 - 1;
        if closed {
            while high_i > 0 && pg[high_i as usize] == pg[0] {
                high_i -= 1;
            }
        }
        while high_i > 0 && pg[high_i as usize] == pg[high_i as usize - 1] {
            high_i -= 1;
        }
        if (closed && high_i < 2) || (!closed && high_i < 1) {
            return false;
        }

        // Allocate a new edge array.
        let base = self.m_edges.len();
        self.m_edges.resize(base + high_i as usize + 1, TEdge::default());
        // Fill in the edge array.
        let result = self.add_path_internal(pg, high_i, poly_typ, closed, base);
        if !result {
            // The C++ drops the edge array it did not remember. Local minima that
            // were pushed before the failure cannot refer to it: every `return
            // false` of AddPathInternal comes before the first push.
            self.m_edges.truncate(base);
        }
        result
    }

    /// clipper.cpp:784-958
    /// C++: bool ClipperBase::AddPathInternal(const Path &pg, int highI, PolyType PolyTyp, bool Closed, TEdge* edges)
    /// `edges` is the index of the path's first edge in `m_edges`.
    pub fn add_path_internal(&mut self, pg: &Path, high_i: i32, poly_typ: PolyType, closed: bool, edges: usize) -> bool {
        fn prev(edges: &[TEdge], i: usize) -> usize {
            edges[i].prev.expect("an edge in the list has a Prev")
        }
        fn next(edges: &[TEdge], i: usize) -> usize {
            edges[i].next.expect("an edge in the list has a Next")
        }
        // `use_lines` is defined (clipper.hpp:48).
        if !closed && poly_typ == PolyType::ptClip {
            panic!("AddPath: Open paths must be subject.");
        }

        debug_assert!(high_i >= 0 && (high_i as usize) < pg.len());
        let high = high_i as usize;

        // 1. Basic (first) edge initialization ...
        self.m_edges[edges + 1].curr = pg[1];
        init_edge(&mut self.m_edges, edges, edges + 1, edges + high, &pg[0]);
        init_edge(&mut self.m_edges, edges + high, edges, edges + high - 1, &pg[high]);
        for i in (1..high).rev() {
            init_edge(&mut self.m_edges, edges + i, edges + i + 1, edges + i - 1, &pg[i]);
        }
        let mut e_start = edges;

        // 2. Remove duplicate vertices, and (when closed) collinear edges ...
        let mut e = e_start;
        let mut e_loop_stop = e_start;
        loop {
            // nb: allows matching start and end points when not Closed ...
            let en = next(&self.m_edges, e);
            if self.m_edges[e].curr == self.m_edges[en].curr && (closed || en != e_start) {
                if e == en {
                    break;
                }
                if e == e_start {
                    e_start = en;
                }
                e = remove_edge(&mut self.m_edges, e);
                e_loop_stop = e;
                continue;
            }
            let ep = prev(&self.m_edges, e);
            if ep == en {
                break; // only two vertices
            } else if closed
                && slopes_equal_3(&self.m_edges[ep].curr, &self.m_edges[e].curr, &self.m_edges[en].curr)
                && (!self.m_preserve_collinear
                    || !pt2_is_between_pt1_and_pt3(&self.m_edges[ep].curr, &self.m_edges[e].curr, &self.m_edges[en].curr))
            {
                // Collinear edges are allowed for open paths but in closed paths
                // the default is to merge adjacent collinear edges into a single edge.
                // However, if the PreserveCollinear property is enabled, only overlapping
                // collinear edges (ie spikes) will be removed from closed paths.
                if e == e_start {
                    e_start = en;
                }
                e = remove_edge(&mut self.m_edges, e);
                e = prev(&self.m_edges, e);
                e_loop_stop = e;
                continue;
            }
            e = en;
            if e == e_loop_stop || (!closed && next(&self.m_edges, e) == e_start) {
                break;
            }
        }

        if (!closed && e == next(&self.m_edges, e))
            || (closed && prev(&self.m_edges, e) == next(&self.m_edges, e))
        {
            return false;
        }

        if !closed {
            self.m_has_open_paths = true;
            let sp = prev(&self.m_edges, e_start);
            self.m_edges[sp].out_idx = SKIP;
        }

        // 3. Do second stage of edge initialization ...
        // IsFlat means all vertices have the same Y coordinate.
        let mut is_flat = true;
        e = e_start;
        loop {
            init_edge2(&mut self.m_edges, e, poly_typ);
            e = next(&self.m_edges, e);
            if is_flat && self.m_edges[e].curr.y != self.m_edges[e_start].curr.y {
                is_flat = false;
            }
            if e == e_start {
                break;
            }
        }

        // 4. Finally, add edge bounds to LocalMinima list ...

        // Totally flat paths must be handled differently when adding them
        // to LocalMinima list to avoid endless loops etc ...
        if is_flat {
            if closed {
                return false;
            }
            let ep = prev(&self.m_edges, e);
            self.m_edges[ep].out_idx = SKIP;
            let loc_min = LocalMinimum { y: self.m_edges[e].bot.y, left_bound: None, right_bound: Some(e) };
            self.m_edges[e].side = EdgeSide::esRight;
            self.m_edges[e].wind_delta = 0;
            loop {
                if self.m_edges[e].bot.x != self.m_edges[prev(&self.m_edges, e)].top.x {
                    reverse_horizontal(&mut self.m_edges[e]);
                }
                let en = next(&self.m_edges, e);
                if self.m_edges[en].out_idx == SKIP {
                    break;
                }
                self.m_edges[e].next_in_lml = Some(en);
                e = en;
            }
            self.m_minima_list.push(loc_min);
            return true;
        }

        let mut left_bound_is_forward: bool;
        let mut e_min: Option<usize> = None;

        // workaround to avoid an endless loop in the while loop below when
        // open paths have matching start and end points ...
        {
            let ep = prev(&self.m_edges, e);
            if self.m_edges[ep].bot == self.m_edges[ep].top {
                e = next(&self.m_edges, e);
            }
        }

        // Find local minima and store them into a Local Minima List.
        // Multiple Local Minima could be created for a single path.
        loop {
            e = find_next_loc_min(&self.m_edges, e);
            if Some(e) == e_min {
                break;
            } else if e_min.is_none() {
                e_min = Some(e);
            }

            // E and E.Prev now share a local minima (left aligned if horizontal).
            // Compare their slopes to find which starts which bound ...
            let ep = prev(&self.m_edges, e);
            let mut loc_min = LocalMinimum { y: self.m_edges[e].bot.y, left_bound: None, right_bound: None };
            let (left, right);
            if self.m_edges[e].dx < self.m_edges[ep].dx {
                left = ep;
                right = e;
                left_bound_is_forward = false; // Q.nextInLML = Q.prev
            } else {
                left = e;
                right = ep;
                left_bound_is_forward = true; // Q.nextInLML = Q.next
            }
            loc_min.left_bound = Some(left);
            loc_min.right_bound = Some(right);
            self.m_edges[left].side = EdgeSide::esLeft;
            self.m_edges[right].side = EdgeSide::esRight;

            if !closed {
                self.m_edges[left].wind_delta = 0;
            } else if next(&self.m_edges, left) == right {
                self.m_edges[left].wind_delta = -1;
            } else {
                self.m_edges[left].wind_delta = 1;
            }
            self.m_edges[right].wind_delta = -self.m_edges[left].wind_delta;

            e = self.process_bound(left, left_bound_is_forward);
            if self.m_edges[e].out_idx == SKIP {
                e = self.process_bound(e, left_bound_is_forward);
            }

            let mut e2 = self.process_bound(right, !left_bound_is_forward);
            if self.m_edges[e2].out_idx == SKIP {
                e2 = self.process_bound(e2, !left_bound_is_forward);
            }

            if self.m_edges[left].out_idx == SKIP {
                loc_min.left_bound = None;
            } else if self.m_edges[right].out_idx == SKIP {
                loc_min.right_bound = None;
            }
            self.m_minima_list.push(loc_min);
            if !left_bound_is_forward {
                e = e2;
            }
        }
        true
    }

    /// clipper.cpp:962-971
    /// C++: void ClipperBase::Clear()
    pub fn clear(&mut self) {
        self.m_minima_list.clear();
        self.m_edges.clear();
        self.m_has_open_paths = false;
    }

    /// clipper.cpp:643-757
    /// C++: TEdge* ClipperBase::ProcessBound(TEdge* E, bool NextIsForward)
    /// Called from ClipperBase::AddPath().
    pub fn process_bound(&mut self, mut e: usize, next_is_forward: bool) -> usize {
        fn prev(edges: &[TEdge], i: usize) -> usize {
            edges[i].prev.expect("an edge in the list has a Prev")
        }
        fn next(edges: &[TEdge], i: usize) -> usize {
            edges[i].next.expect("an edge in the list has a Next")
        }
        let mut result = e;
        let mut horz: usize;

        if self.m_edges[e].out_idx == SKIP {
            // if edges still remain in the current bound beyond the skip edge then
            // create another LocMin and call ProcessBound once more
            if next_is_forward {
                while self.m_edges[e].top.y == self.m_edges[next(&self.m_edges, e)].bot.y {
                    e = next(&self.m_edges, e);
                }
                // don't include top horizontals when parsing a bound a second time,
                // they will be contained in the opposite bound ...
                while e != result && is_horizontal(&self.m_edges[e]) {
                    e = prev(&self.m_edges, e);
                }
            } else {
                while self.m_edges[e].top.y == self.m_edges[prev(&self.m_edges, e)].bot.y {
                    e = prev(&self.m_edges, e);
                }
                while e != result && is_horizontal(&self.m_edges[e]) {
                    e = next(&self.m_edges, e);
                }
            }

            if e == result {
                result = if next_is_forward { next(&self.m_edges, e) } else { prev(&self.m_edges, e) };
            } else {
                // there are more edges in the bound beyond result starting with E
                e = if next_is_forward { next(&self.m_edges, result) } else { prev(&self.m_edges, result) };
                let loc_min = LocalMinimum { y: self.m_edges[e].bot.y, left_bound: None, right_bound: Some(e) };
                self.m_edges[e].wind_delta = 0;
                result = self.process_bound(e, next_is_forward);
                self.m_minima_list.push(loc_min);
            }
            return result;
        }

        let mut e_start: usize;

        if is_horizontal(&self.m_edges[e]) {
            // We need to be careful with open paths because this may not be a
            // true local minima (ie E may be following a skip edge).
            // Also, consecutive horz. edges may start heading left before going right.
            e_start = if next_is_forward { prev(&self.m_edges, e) } else { next(&self.m_edges, e) };
            if is_horizontal(&self.m_edges[e_start]) {
                // ie an adjoining horizontal skip edge
                if self.m_edges[e_start].bot.x != self.m_edges[e].bot.x
                    && self.m_edges[e_start].top.x != self.m_edges[e].bot.x
                {
                    reverse_horizontal(&mut self.m_edges[e]);
                }
            } else if self.m_edges[e_start].bot.x != self.m_edges[e].bot.x {
                reverse_horizontal(&mut self.m_edges[e]);
            }
        }

        e_start = e;
        if next_is_forward {
            loop {
                let rn = next(&self.m_edges, result);
                if !(self.m_edges[result].top.y == self.m_edges[rn].bot.y && self.m_edges[rn].out_idx != SKIP) {
                    break;
                }
                result = rn;
            }
            if is_horizontal(&self.m_edges[result]) && self.m_edges[next(&self.m_edges, result)].out_idx != SKIP {
                // nb: at the top of a bound, horizontals are added to the bound
                // only when the preceding edge attaches to the horizontal's left vertex
                // unless a Skip edge is encountered when that becomes the top divide
                horz = result;
                while is_horizontal(&self.m_edges[prev(&self.m_edges, horz)]) {
                    horz = prev(&self.m_edges, horz);
                }
                if self.m_edges[prev(&self.m_edges, horz)].top.x > self.m_edges[next(&self.m_edges, result)].top.x {
                    result = prev(&self.m_edges, horz);
                }
            }
            while e != result {
                self.m_edges[e].next_in_lml = self.m_edges[e].next;
                if is_horizontal(&self.m_edges[e])
                    && e != e_start
                    && self.m_edges[e].bot.x != self.m_edges[prev(&self.m_edges, e)].top.x
                {
                    reverse_horizontal(&mut self.m_edges[e]);
                }
                e = next(&self.m_edges, e);
            }
            if is_horizontal(&self.m_edges[e])
                && e != e_start
                && self.m_edges[e].bot.x != self.m_edges[prev(&self.m_edges, e)].top.x
            {
                reverse_horizontal(&mut self.m_edges[e]);
            }
            result = next(&self.m_edges, result); // move to the edge just beyond current bound
        } else {
            loop {
                let rp = prev(&self.m_edges, result);
                if !(self.m_edges[result].top.y == self.m_edges[rp].bot.y && self.m_edges[rp].out_idx != SKIP) {
                    break;
                }
                result = rp;
            }
            if is_horizontal(&self.m_edges[result]) && self.m_edges[prev(&self.m_edges, result)].out_idx != SKIP {
                horz = result;
                while is_horizontal(&self.m_edges[next(&self.m_edges, horz)]) {
                    horz = next(&self.m_edges, horz);
                }
                let hn_top_x = self.m_edges[next(&self.m_edges, horz)].top.x;
                let rp_top_x = self.m_edges[prev(&self.m_edges, result)].top.x;
                if hn_top_x == rp_top_x || hn_top_x > rp_top_x {
                    result = next(&self.m_edges, horz);
                }
            }

            while e != result {
                self.m_edges[e].next_in_lml = self.m_edges[e].prev;
                if is_horizontal(&self.m_edges[e])
                    && e != e_start
                    && self.m_edges[e].bot.x != self.m_edges[next(&self.m_edges, e)].top.x
                {
                    reverse_horizontal(&mut self.m_edges[e]);
                }
                e = prev(&self.m_edges, e);
            }
            if is_horizontal(&self.m_edges[e])
                && e != e_start
                && self.m_edges[e].bot.x != self.m_edges[next(&self.m_edges, e)].top.x
            {
                reverse_horizontal(&mut self.m_edges[e]);
            }
            result = prev(&self.m_edges, result); // move to the edge just beyond current bound
        }

        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn edge(bot: (cInt, cInt), top: (cInt, cInt)) -> TEdge {
        let mut edges = vec![TEdge::default(); 2];
        init_edge(&mut edges, 0, 1, 1, &IntPoint::new(bot.0, bot.1));
        init_edge(&mut edges, 1, 0, 0, &IntPoint::new(top.0, top.1));
        init_edge2(&mut edges, 0, PolyType::ptSubject);
        edges[0]
    }

    #[test]
    fn round_is_half_away_from_zero_then_truncated() {
        assert_eq!(round(2.5), 3);
        assert_eq!(round(-2.5), -3);
        assert_eq!(round(2.4999), 2);
        assert_eq!(round(-0.4), 0);
    }

    #[test]
    fn init_edge2_orders_bottom_and_top_and_sets_the_slope() {
        // Y grows downwards in Clipper's terms: the bottom has the larger Y.
        let e = edge((0, 100), (50, 0));
        assert_eq!((e.bot, e.top), (IntPoint::new(0, 100), IntPoint::new(50, 0)));
        assert_eq!(e.delta, IntPoint::new(50, -100));
        assert_eq!(e.dx, -0.5);
        assert_eq!(e.out_idx, UNASSIGNED);
        let flat = edge((0, 7), (9, 7));
        assert!(is_horizontal(&flat));
        assert_eq!(flat.dx, HORIZONTAL);
    }

    #[test]
    fn top_x_and_intersect_point() {
        let a = edge((0, 100), (100, 0));
        let b = edge((100, 100), (0, 0));
        assert_eq!(top_x(&a, 50), 50);
        assert_eq!(top_x(&a, 0), 100);
        let mut ip = IntPoint::default();
        intersect_point(&a, &b, &mut ip);
        assert_eq!(ip, IntPoint::new(50, 50));
        // A vertical edge and a slanted one.
        let v = edge((30, 100), (30, 0));
        intersect_point(&v, &a, &mut ip);
        assert_eq!(ip, IntPoint::new(30, 70));
    }

    #[test]
    fn slopes_equal_uses_64_bit_products() {
        assert!(slopes_equal(2_000_000_000, 1_000_000_000, 2, 1));
        assert!(!slopes_equal(2_000_000_000, 1_000_000_000, 2_000_000_001, 1_000_000_000));
        let (a, b, c) = (IntPoint::new(0, 0), IntPoint::new(5, 5), IntPoint::new(9, 9));
        assert!(slopes_equal_3(&a, &b, &c));
    }

    #[test]
    fn reverse_poly_pt_links_swaps_next_and_prev() {
        let mut pts = vec![OutPt::default(); 3];
        for i in 0..3 {
            pts[i].next = (i + 1) % 3;
            pts[i].prev = (i + 2) % 3;
        }
        reverse_poly_pt_links(&mut pts, Some(0));
        assert_eq!((pts[0].next, pts[0].prev), (2, 1));
        reverse_poly_pt_links(&mut pts, None);
    }
    fn ring(pts: &[(cInt, cInt)]) -> Vec<OutPt> {
        let n = pts.len();
        (0..n)
            .map(|i| OutPt { idx: 0, pt: IntPoint::new(pts[i].0, pts[i].1), next: (i + 1) % n, prev: (i + n - 1) % n })
            .collect()
    }

    #[test]
    fn remove_edge_unlinks_and_flags() {
        let mut edges = vec![TEdge::default(); 3];
        for i in 0..3 {
            init_edge(&mut edges, i, (i + 1) % 3, (i + 2) % 3, &IntPoint::new(i as cInt, 0));
        }
        assert_eq!(remove_edge(&mut edges, 1), 2);
        assert_eq!((edges[0].next, edges[2].prev, edges[1].prev), (Some(2), Some(0), None));
    }

    #[test]
    fn overlap_of_collinear_segments() {
        let p = IntPoint::new;
        let (mut a, mut b) = (IntPoint::default(), IntPoint::default());
        assert!(get_overlap_segment(p(0, 0), p(10, 0), p(4, 0), p(20, 0), &mut a, &mut b));
        assert_eq!((a, b), (p(4, 0), p(10, 0)));
        assert!(!get_overlap_segment(p(0, 0), p(3, 0), p(4, 0), p(20, 0), &mut a, &mut b));
        // Mostly vertical: ordered by descending Y.
        assert!(get_overlap_segment(p(0, 0), p(0, 10), p(0, 6), p(0, 30), &mut a, &mut b));
        assert_eq!((a, b), (p(0, 10), p(0, 6)));
        assert!(horz_segments_overlap(10, 0, 5, 20));
        assert!(!horz_segments_overlap(0, 5, 5, 9));
    }

    #[test]
    fn bottom_point_is_the_largest_y_then_the_smallest_x() {
        let pts = ring(&[(5, 0), (9, 7), (2, 7), (0, 3)]);
        assert_eq!(get_bottom_pt(&pts, 0), 2);
        let p = IntPoint::new;
        assert!(pt2_is_between_pt1_and_pt3(&p(0, 0), &p(3, 3), &p(9, 9)));
        assert!(!pt2_is_between_pt1_and_pt3(&p(0, 0), &p(0, 0), &p(9, 9)));
        assert!(pt2_is_between_pt1_and_pt3(&p(4, 0), &p(4, 5), &p(4, 9)));
    }

    fn base_with(path: &[(cInt, cInt)]) -> (ClipperBase, bool) {
        let pg: Path = path.iter().map(|p| IntPoint::new(p.0, p.1)).collect();
        let mut c = ClipperBase::default();
        let ok = c.add_path(&pg, PolyType::ptSubject, true);
        (c, ok)
    }

    #[test]
    fn add_path_builds_one_local_minimum_for_a_convex_polygon() {
        // A square: Y grows downwards, so the minimum is at the largest Y.
        let (c, ok) = base_with(&[(0, 0), (10, 0), (10, 10), (0, 10)]);
        assert!(ok);
        assert_eq!(c.m_minima_list.len(), 1);
        let lm = c.m_minima_list[0];
        assert_eq!(lm.y, 10);
        let (l, r) = (lm.left_bound.unwrap(), lm.right_bound.unwrap());
        assert_eq!((c.m_edges[l].side, c.m_edges[r].side), (EdgeSide::esLeft, EdgeSide::esRight));
        assert_eq!(c.m_edges[l].wind_delta, -c.m_edges[r].wind_delta);
        assert_ne!(c.m_edges[l].wind_delta, 0);
    }

    #[test]
    fn add_path_drops_duplicates_and_collinear_points_and_rejects_flat_paths() {
        // The closing duplicate, a repeated vertex and a collinear vertex go.
        let (c, ok) = base_with(&[(0, 0), (5, 0), (10, 0), (10, 0), (10, 10), (0, 10), (0, 0)]);
        assert!(ok);
        let live = c.m_edges.iter().filter(|e| e.prev.is_some()).count();
        assert_eq!(live, 4);
        // Two local minima for an M shape (two downward points).
        let (m, ok) = base_with(&[(0, 0), (4, 6), (8, 0), (12, 6), (16, 0), (16, -5), (0, -5)]);
        assert!(ok);
        assert_eq!(m.m_minima_list.len(), 2);
        // Fewer than three distinct points, and a flat closed path, are refused.
        assert!(!base_with(&[(0, 0), (5, 5), (0, 0)]).1);
        let (flat, ok) = base_with(&[(0, 3), (5, 3), (9, 3)]);
        assert!(!ok);
        assert!(flat.m_edges.is_empty() && flat.m_minima_list.is_empty());
    }

}
