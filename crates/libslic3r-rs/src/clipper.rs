//! Port of ClipperLib, `src/clipper/clipper.cpp` and `clipper.hpp` of the engine (the file
//! `libslic3r/clipper.cpp` compiles it into libslic3r).
//!
//! In progress: this file holds the library's free functions, `ClipperBase` and `Clipper`
//! with `Execute` into `Paths` (clipper.cpp up to line 3313 and `DoSimplePolygons`). Nothing in
//! the engine calls the clipper here yet: `ClipperOffset`, `PolyTree` and `BuildResult2` are not
//! ported, and the engine's clipping still runs as C++ through `clipper-z-sys`; see
//! `docs/rust-engine/parity/not-ported.md`.
//!
//! The engine builds ClipperLib with `CLIPPERLIB_INT32` (clipper.hpp:83-95), so `cInt` is a
//! 32-bit integer, with `use_lines` and, here, without `CLIPPERLIB_USE_XYZ`. The float
//! expressions below are written as the C++ build evaluates them: clang for arm64 contracts a
//! product into a fused multiply-add inside one expression.
//! `tests/clipper_native_parity.rs` holds the free functions equal to the library, and
//! `tests/clipper_execute_native_parity.rs` holds `Clipper::execute` equal to it.

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

/// clipper.hpp:213 `struct IntRect { cInt left; cInt top; cInt right; cInt bottom; };`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct IntRect {
    pub left: cInt,
    pub top: cInt,
    pub right: cInt,
    pub bottom: cInt,
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

    /// clipper.cpp:976-1001
    /// C++: void ClipperBase::Reset()
    /// Initialize the Local Minima List:
    /// Sort the LML entries, initialize the left / right bound edges of each Local Minima.
    pub fn reset(&mut self) {
        if self.m_minima_list.is_empty() {
            return; // ie nothing to process
        }
        // std::sort(m_MinimaList.begin(), m_MinimaList.end(), [](lm1, lm2){ return lm1.Y < lm2.Y; });
        //
        // `std::sort` is not stable. Which of two local minima at the same Y comes
        // first is decided by libc++'s algorithm, and the sweep takes them in that
        // order: `libcxx_sort` is that algorithm. (A stable sort agrees with it up
        // to 23 minima only; tests/clipper_execute_native_parity.rs has more.)
        libcxx_sort(&mut self.m_minima_list, |lm1, lm2| lm1.y < lm2.y);

        // reset all edges ...
        for i in 0..self.m_minima_list.len() {
            let lm = self.m_minima_list[i];
            if let Some(e) = lm.left_bound {
                let e = &mut self.m_edges[e];
                e.curr = e.bot;
                e.side = EdgeSide::esLeft;
                e.out_idx = UNASSIGNED;
            }

            if let Some(e) = lm.right_bound {
                let e = &mut self.m_edges[e];
                e.curr = e.bot;
                e.side = EdgeSide::esRight;
                e.out_idx = UNASSIGNED;
            }
        }
    }

    /// clipper.cpp:1005-1043
    /// C++: IntRect ClipperBase::GetBounds()
    /// Get bounds of the edges referenced by the Local Minima List.
    /// Returns (0,0,0,0) for an empty rectangle.
    pub fn get_bounds(&self) -> IntRect {
        let mut result = IntRect::default();
        if self.m_minima_list.is_empty() {
            return result;
        }
        let first = self.m_minima_list[0].left_bound.expect("GetBounds reads LeftBound");
        result.left = self.m_edges[first].bot.x;
        result.top = self.m_edges[first].bot.y;
        result.right = self.m_edges[first].bot.x;
        result.bottom = self.m_edges[first].bot.y;
        for lm in &self.m_minima_list {
            let left_bound = lm.left_bound.expect("GetBounds reads LeftBound");
            result.bottom = result.bottom.max(self.m_edges[left_bound].bot.y);
            let mut e = left_bound;
            loop {
                let bottom_e = e;
                while let Some(next) = self.m_edges[e].next_in_lml {
                    if self.m_edges[e].bot.x < result.left {
                        result.left = self.m_edges[e].bot.x;
                    }
                    if self.m_edges[e].bot.x > result.right {
                        result.right = self.m_edges[e].bot.x;
                    }
                    e = next;
                }
                result.left = result.left.min(self.m_edges[e].bot.x);
                result.right = result.right.max(self.m_edges[e].bot.x);
                result.left = result.left.min(self.m_edges[e].top.x);
                result.right = result.right.max(self.m_edges[e].top.x);
                result.top = result.top.min(self.m_edges[e].top.y);
                if bottom_e == left_bound {
                    e = lm.right_bound.expect("GetBounds reads RightBound");
                } else {
                    break;
                }
            }
        }
        result
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

// ---------------------------------------------------------------------------
// `std::sort` as libc++ implements it (`<__algorithm/sort.h>`, the `__introsort`
// of LLVM 17 and later). ClipperLib sorts the local minima by Y and the
// intersections of a scanbeam by Y with `std::sort`, which is not stable: the
// order of elements with equal keys is the algorithm's, and the sweep takes them
// in that order. Up to 23 elements libc++ sorts by insertion and keeps equal
// elements in place; from 24 on it partitions and does not. The functions below
// follow the header one for one, for a type that is not arithmetic or a
// comparator that is not `std::less` (so: no branchless networks, no bitset
// partition). Iterators are indices into the slice; `__first - 1` is `first - 1`.
// ---------------------------------------------------------------------------

/// sort.h `__sort3` (the arm without `__use_branchless_sort`): stable, 0-2 swaps.
fn libcxx_sort3<T: Copy, F: FnMut(&T, &T) -> bool>(v: &mut [T], x: usize, y: usize, z: usize, c: &mut F) -> bool {
    if !c(&v[y], &v[x]) {
        // if x <= y
        if !c(&v[z], &v[y]) {
            // if y <= z
            return false; // x <= y && y <= z
        }
        v.swap(y, z); // x <= z && y < z
        if c(&v[y], &v[x]) {
            // if x > y
            v.swap(x, y); // x < y && y <= z
        }
        return true; // x <= y && y < z
    }
    if c(&v[z], &v[y]) {
        // x > y, if y > z
        v.swap(x, z); // x < y && y < z
        return true;
    }
    v.swap(x, y); // x > y && y <= z
    if c(&v[z], &v[y]) {
        // if y > z
        v.swap(y, z); // x <= y && y < z
    }
    true
}

/// sort.h `__sort4` (the arm without `__use_branchless_sort`).
fn libcxx_sort4<T: Copy, F: FnMut(&T, &T) -> bool>(v: &mut [T], x1: usize, x2: usize, x3: usize, x4: usize, c: &mut F) {
    libcxx_sort3(v, x1, x2, x3, c);
    if c(&v[x4], &v[x3]) {
        v.swap(x3, x4);
        if c(&v[x3], &v[x2]) {
            v.swap(x2, x3);
            if c(&v[x2], &v[x1]) {
                v.swap(x1, x2);
            }
        }
    }
}

/// sort.h `__sort5` (the arm without `__use_branchless_sort`).
fn libcxx_sort5<T: Copy, F: FnMut(&T, &T) -> bool>(
    v: &mut [T],
    x1: usize,
    x2: usize,
    x3: usize,
    x4: usize,
    x5: usize,
    c: &mut F,
) {
    libcxx_sort4(v, x1, x2, x3, x4, c);
    if c(&v[x5], &v[x4]) {
        v.swap(x4, x5);
        if c(&v[x4], &v[x3]) {
            v.swap(x3, x4);
            if c(&v[x3], &v[x2]) {
                v.swap(x2, x3);
                if c(&v[x2], &v[x1]) {
                    v.swap(x1, x2);
                }
            }
        }
    }
}

/// sort.h `__insertion_sort`.
fn libcxx_insertion_sort<T: Copy, F: FnMut(&T, &T) -> bool>(v: &mut [T], first: usize, last: usize, c: &mut F) {
    if first == last {
        return;
    }
    let mut i = first + 1;
    while i != last {
        let mut j = i - 1;
        if c(&v[i], &v[j]) {
            let t = v[i];
            let mut k = j;
            j = i;
            // do { *j = *k; j = k; } while (j != first && comp(t, *--k));
            loop {
                v[j] = v[k];
                j = k;
                if j == first {
                    break;
                }
                k -= 1;
                if !c(&t, &v[k]) {
                    break;
                }
            }
            v[j] = t;
        }
        i += 1;
    }
}

/// sort.h `__insertion_sort_unguarded`: assumes an element at `first - 1` that is not
/// greater than any element of the range, so the inner loop has no bounds check.
fn libcxx_insertion_sort_unguarded<T: Copy, F: FnMut(&T, &T) -> bool>(v: &mut [T], first: usize, last: usize, c: &mut F) {
    if first == last {
        return;
    }
    let mut i = first + 1;
    while i != last {
        let mut j = i - 1;
        if c(&v[i], &v[j]) {
            let t = v[i];
            let mut k = j;
            j = i;
            // do { *j = *k; j = k; } while (comp(t, *--k));
            loop {
                v[j] = v[k];
                j = k;
                k -= 1;
                if !c(&t, &v[k]) {
                    break;
                }
            }
            v[j] = t;
        }
        i += 1;
    }
}

/// sort.h `__insertion_sort_incomplete`: gives up after 8 insertions and tells whether
/// the range ended up sorted.
fn libcxx_insertion_sort_incomplete<T: Copy, F: FnMut(&T, &T) -> bool>(v: &mut [T], first: usize, last: usize, c: &mut F) -> bool {
    match last - first {
        0 | 1 => return true,
        2 => {
            if c(&v[last - 1], &v[first]) {
                v.swap(first, last - 1);
            }
            return true;
        }
        3 => {
            libcxx_sort3(v, first, first + 1, last - 1, c);
            return true;
        }
        4 => {
            libcxx_sort4(v, first, first + 1, first + 2, last - 1, c);
            return true;
        }
        5 => {
            libcxx_sort5(v, first, first + 1, first + 2, first + 3, last - 1, c);
            return true;
        }
        _ => {}
    }
    let mut j = first + 2;
    libcxx_sort3(v, first, first + 1, j, c);
    const LIMIT: u32 = 8;
    let mut count: u32 = 0;
    let mut i = j + 1;
    while i != last {
        if c(&v[i], &v[j]) {
            let t = v[i];
            let mut k = j;
            j = i;
            loop {
                v[j] = v[k];
                j = k;
                if j == first {
                    break;
                }
                k -= 1;
                if !c(&t, &v[k]) {
                    break;
                }
            }
            v[j] = t;
            count += 1;
            if count == LIMIT {
                i += 1;
                return i == last;
            }
        }
        j = i;
        i += 1;
    }
    true
}

/// sort.h `__partition_with_equals_on_right`: `v[first]` is the pivot; elements
/// equivalent to it stay to its right. Returns the pivot's position and whether the
/// range was already partitioned.
fn libcxx_partition_with_equals_on_right<T: Copy, F: FnMut(&T, &T) -> bool>(
    v: &mut [T],
    mut first: usize,
    mut last: usize,
    c: &mut F,
) -> (usize, bool) {
    let begin = first;
    let pivot = v[first];
    // Find the first element greater or equal to the pivot.
    loop {
        first += 1;
        if !c(&v[first], &pivot) {
            break;
        }
    }
    // Find the last element less than the pivot.
    if begin == first - 1 {
        // while (first < last && !comp(*--last, pivot));
        while first < last && {
            last -= 1;
            !c(&v[last], &pivot)
        } {}
    } else {
        loop {
            last -= 1;
            if c(&v[last], &pivot) {
                break;
            }
        }
    }
    let already_partitioned = first >= last;
    while first < last {
        v.swap(first, last);
        loop {
            first += 1;
            if !c(&v[first], &pivot) {
                break;
            }
        }
        loop {
            last -= 1;
            if c(&v[last], &pivot) {
                break;
            }
        }
    }
    // Move the pivot to its correct position.
    let pivot_pos = first - 1;
    if begin != pivot_pos {
        v[begin] = v[pivot_pos];
    }
    v[pivot_pos] = pivot;
    (pivot_pos, already_partitioned)
}

/// sort.h `__partition_with_equals_on_left`: elements equivalent to the pivot go to
/// its left. Returns the position after the pivot.
fn libcxx_partition_with_equals_on_left<T: Copy, F: FnMut(&T, &T) -> bool>(
    v: &mut [T],
    mut first: usize,
    mut last: usize,
    c: &mut F,
) -> usize {
    let begin = first;
    let pivot = v[first];
    if c(&pivot, &v[last - 1]) {
        loop {
            first += 1;
            if c(&pivot, &v[first]) {
                break;
            }
        }
    } else {
        // while (++first < last && !comp(pivot, *first)) {}
        loop {
            first += 1;
            if !(first < last && !c(&pivot, &v[first])) {
                break;
            }
        }
    }

    if first < last {
        loop {
            last -= 1;
            if !c(&pivot, &v[last]) {
                break;
            }
        }
    }
    while first < last {
        v.swap(first, last);
        loop {
            first += 1;
            if c(&pivot, &v[first]) {
                break;
            }
        }
        loop {
            last -= 1;
            if !c(&pivot, &v[last]) {
                break;
            }
        }
    }
    let pivot_pos = first - 1;
    if begin != pivot_pos {
        v[begin] = v[pivot_pos];
    }
    v[pivot_pos] = pivot;
    first
}

/// sift_down.h `__sift_down`, on the heap `s[..len]`.
fn libcxx_sift_down<T: Copy, F: FnMut(&T, &T) -> bool>(s: &mut [T], c: &mut F, len: usize, mut start: usize) {
    // left-child of start is at 2 * start + 1, right-child at 2 * start + 2
    let mut child = start;
    if len < 2 || (len - 2) / 2 < child {
        return;
    }
    child = 2 * child + 1;
    if child + 1 < len && c(&s[child], &s[child + 1]) {
        // right-child exists and is greater than left-child
        child += 1;
    }
    // check if we are in heap-order
    if c(&s[child], &s[start]) {
        // we are, start is larger than its largest child
        return;
    }
    let top = s[start];
    loop {
        // we are not in heap-order, swap the parent with its largest child
        s[start] = s[child];
        start = child;
        if (len - 2) / 2 < child {
            break;
        }
        // recompute the child based off of the updated parent
        child = 2 * child + 1;
        if child + 1 < len && c(&s[child], &s[child + 1]) {
            child += 1;
        }
        // check if we are in heap-order
        if c(&s[child], &top) {
            break;
        }
    }
    s[start] = top;
}

/// sift_down.h `__floyd_sift_down`, on the heap `s[..len]`; returns the hole.
fn libcxx_floyd_sift_down<T: Copy, F: FnMut(&T, &T) -> bool>(s: &mut [T], c: &mut F, len: usize) -> usize {
    let mut hole = 0usize;
    let mut child = 0usize;
    loop {
        // child_i += child + 1; child = 2 * child + 1: the index of the left child.
        child = 2 * child + 1;
        if child + 1 < len && c(&s[child], &s[child + 1]) {
            // right-child exists and is greater than left-child
            child += 1;
        }
        // swap hole with its largest child
        s[hole] = s[child];
        hole = child;
        // if hole is now a leaf, we're done
        if child > (len - 2) / 2 {
            return hole;
        }
    }
}

/// push_heap.h `__sift_up`, on `s[..last]` with `len == last`.
fn libcxx_sift_up<T: Copy, F: FnMut(&T, &T) -> bool>(s: &mut [T], mut last: usize, c: &mut F, mut len: usize) {
    if len > 1 {
        len = (len - 2) / 2;
        let mut ptr = len;
        last -= 1;
        if c(&s[ptr], &s[last]) {
            let t = s[last];
            loop {
                s[last] = s[ptr];
                last = ptr;
                if len == 0 {
                    break;
                }
                len = (len - 1) / 2;
                ptr = len;
                if !c(&s[ptr], &t) {
                    break;
                }
            }
            s[last] = t;
        }
    }
}

/// pop_heap.h `__pop_heap`, on the heap `s[..last]` with `len == last`.
fn libcxx_pop_heap<T: Copy, F: FnMut(&T, &T) -> bool>(s: &mut [T], mut last: usize, c: &mut F, len: usize) {
    if len > 1 {
        let top = s[0]; // create a hole at first
        let mut hole = libcxx_floyd_sift_down(s, c, len);
        last -= 1;
        if hole == last {
            s[hole] = top;
        } else {
            s[hole] = s[last];
            hole += 1;
            s[last] = top;
            libcxx_sift_up(s, hole, c, hole);
        }
    }
}

/// partial_sort.h `__partial_sort_impl(first, last, last)`, which is what `__introsort`
/// falls back to at depth 0: `__make_heap` (make_heap.h) then `__sort_heap` (sort_heap.h).
fn libcxx_heap_sort<T: Copy, F: FnMut(&T, &T) -> bool>(s: &mut [T], c: &mut F) {
    if s.is_empty() {
        return;
    }
    // __make_heap
    let n = s.len();
    if n > 1 {
        // start from the first parent, there is no need to consider children
        let mut start = (n - 2) / 2;
        loop {
            libcxx_sift_down(s, c, n, start);
            if start == 0 {
                break;
            }
            start -= 1;
        }
    }
    // for (i = middle; i != last; ++i): middle is last, nothing to do.
    // __sort_heap
    let mut last = n;
    let mut n = n;
    while n > 1 {
        libcxx_pop_heap(s, last, c, n);
        last -= 1;
        n -= 1;
    }
}

/// sort.h `__introsort` with `_UseBitSetPartition == false`.
fn libcxx_introsort<T: Copy, F: FnMut(&T, &T) -> bool>(
    v: &mut [T],
    mut first: usize,
    mut last: usize,
    c: &mut F,
    mut depth: isize,
    mut leftmost: bool,
) {
    // Upper bound for using insertion sort for sorting.
    const LIMIT: usize = 24;
    // Lower bound for using Tuckey's ninther technique for median computation.
    const NINTHER_THRESHOLD: usize = 128;
    loop {
        let len = last - first;
        match len {
            0 | 1 => return,
            2 => {
                if c(&v[last - 1], &v[first]) {
                    v.swap(first, last - 1);
                }
                return;
            }
            3 => {
                libcxx_sort3(v, first, first + 1, last - 1, c);
                return;
            }
            4 => {
                libcxx_sort4(v, first, first + 1, first + 2, last - 1, c);
                return;
            }
            5 => {
                libcxx_sort5(v, first, first + 1, first + 2, first + 3, last - 1, c);
                return;
            }
            _ => {}
        }
        // Use insertion sort if the length of the range is below the specified limit.
        if len < LIMIT {
            if leftmost {
                libcxx_insertion_sort(v, first, last, c);
            } else {
                libcxx_insertion_sort_unguarded(v, first, last, c);
            }
            return;
        }
        if depth == 0 {
            // Fallback to heap sort as Introsort suggests.
            libcxx_heap_sort(&mut v[first..last], c);
            return;
        }
        depth -= 1;
        {
            let half_len = len / 2;
            // Use Tuckey's ninther technique or median of 3 for pivot selection
            // depending on the length of the range being sorted.
            if len > NINTHER_THRESHOLD {
                libcxx_sort3(v, first, first + half_len, last - 1, c);
                libcxx_sort3(v, first + 1, first + (half_len - 1), last - 2, c);
                libcxx_sort3(v, first + 2, first + (half_len + 1), last - 3, c);
                libcxx_sort3(v, first + (half_len - 1), first + half_len, first + (half_len + 1), c);
                v.swap(first, first + half_len);
            } else {
                libcxx_sort3(v, first + half_len, first, last - 1, c);
            }
        }
        // The elements to the left of the current range are already sorted. If the
        // range is not the leftmost one and the pivot is the same as the highest
        // element of the range to the left, all the elements in [first, pivot] are
        // equal to the pivot, and the left side needs no sorting.
        if !leftmost && !c(&v[first - 1], &v[first]) {
            first = libcxx_partition_with_equals_on_left(v, first, last, c);
            continue;
        }
        let (mut i, already_partitioned) = libcxx_partition_with_equals_on_right(v, first, last, c);
        // [first, i) < *i and *i <= [i+1, last)
        // If we were given a perfect partition, see if insertion sort is quick...
        if already_partitioned {
            let fs = libcxx_insertion_sort_incomplete(v, first, i, c);
            if libcxx_insertion_sort_incomplete(v, i + 1, last, c) {
                if fs {
                    return;
                }
                last = i;
                continue;
            } else if fs {
                i += 1;
                first = i;
                continue;
            }
        }
        // Sort the left partition recursively and the right partition with tail recursion elimination.
        libcxx_introsort(v, first, i, c, depth, leftmost);
        leftmost = false;
        i += 1;
        first = i;
    }
}

/// `std::sort(first, last, comp)` of libc++: sort.h `__sort_impl` -> `__sort_dispatch`,
/// with the depth limit `2 * __log2i(last - first)`. `comp(a, b)` is "a goes before b".
pub fn libcxx_sort<T: Copy, F: FnMut(&T, &T) -> bool>(v: &mut [T], mut comp: F) {
    let n = v.len();
    // __log2i: 0 for 0, else the index of the highest set bit.
    let log2i = if n == 0 { 0 } else { (usize::BITS - 1 - n.leading_zeros()) as isize };
    let depth_limit = 2 * log2i;
    libcxx_introsort(v, 0, n, &mut comp, depth_limit, true);
}

// ---------------------------------------------------------------------------
// The output polygons and the helpers of the sweep (clipper.cpp:86-104, 192-202,
// 260-313, 1850-1878, 2066-2091, 2187-2200, 2496-2500, 2754-2766, 2846-2885).
// The library does not export them: they are checked through `Clipper::execute`,
// which `tests/clipper_execute_native_parity.rs` holds equal to the library.
// ---------------------------------------------------------------------------

/// The null `OutPt*` of the free list (`m_OutPtsFree`, and the `Next` of its last
/// point). A point in a polygon never has it: its `next` and `prev` are real indices.
const OUT_PT_NULL: usize = usize::MAX;

/// clipper.cpp:88-104 `struct OutRec`: an output polygon. An `OutRec*` is the index
/// of the record in `Clipper::m_poly_outs`, an `OutPt*` an index into
/// `Clipper::m_out_pts`. `PolyNd` is used by `BuildResult2` only, which is not ported.
#[derive(Debug, Clone, Copy, Default)]
pub struct OutRec {
    pub idx: i32,
    pub is_hole: bool,
    pub is_open: bool,
    /// The 'FirstLeft' field points to another OutRec that contains or is the
    /// 'parent' of OutRec. It is 'first left' because the ActiveEdgeList (AEL) is
    /// parsed left from the current edge (owning OutRec) until the owner OutRec
    /// is found.
    pub first_left: Option<usize>,
    /// Linked list of output points.
    pub pts: Option<usize>,
    pub bottom_pt: Option<usize>,
}

/// clipper.hpp:250-256 `struct IntersectNode`.
#[derive(Debug, Clone, Copy)]
pub struct IntersectNode {
    pub edge1: usize,
    pub edge2: usize,
    pub pt: IntPoint,
}

/// clipper.hpp:278-284 `struct Join`. `OutPt2` is null in a ghost join.
#[derive(Debug, Clone, Copy)]
pub struct Join {
    pub out_pt1: Option<usize>,
    pub out_pt2: Option<usize>,
    pub off_pt: IntPoint,
}

/// clipper.hpp:129 `enum InitOptions {ioReverseSolution = 1, ioStrictlySimple = 2, ioPreserveCollinear = 4};`
pub const IO_REVERSE_SOLUTION: i32 = 1;
pub const IO_STRICTLY_SIMPLE: i32 = 2;
pub const IO_PRESERVE_COLLINEAR: i32 = 4;

/// What `throw clipperException(...)` carries (clipper.hpp:547-555), for the throws
/// that `Clipper::ExecuteInternal` catches itself (clipper.cpp:1117, 1136-1139): they
/// make `Execute` return false, so they travel as an `Err` to that catch. The throws
/// nothing in the library catches are panics.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClipperException(pub &'static str);

/// clipper.cpp:192-202
/// C++: double Area(const OutRec &outRec)
pub fn area_out_rec(out_pts: &[OutPt], out_rec: &OutRec) -> f64 {
    let Some(pts) = out_rec.pts else { return 0.0 };
    let mut op = pts;
    let mut a = 0.0f64;
    loop {
        let prev = out_pts[op].prev;
        // a += (double)(op->Prev->Pt.x() + op->Pt.x()) * (double)(op->Prev->Pt.y() - op->Pt.y());
        // The sum and the difference are taken in cInt; one expression: the product
        // is fused into the sum.
        a = (out_pts[prev].pt.x.wrapping_add(out_pts[op].pt.x) as f64)
            .mul_add(out_pts[prev].pt.y.wrapping_sub(out_pts[op].pt.y) as f64, a);
        op = out_pts[op].next;
        if op == pts {
            break;
        }
    }
    a * 0.5
}

/// clipper.cpp:260-296
/// C++: int PointInPolygon (const IntPoint &pt, OutPt *op)
/// Called by Poly2ContainsPoly1().
pub fn point_in_polygon_out_pt(pt: &IntPoint, out_pts: &[OutPt], mut op: usize) -> i32 {
    // returns 0 if false, +1 if true, -1 if pt ON polygon boundary
    let mut result = 0;
    let start_op = op;
    // (double)(op->Pt.x() - pt.x()) * (op->Next->Pt.y() - pt.y()) - (double)(op->Next->Pt.x() - pt.x()) * (op->Pt.y() - pt.y()):
    // the differences are taken in cInt; the left product is fused with the right
    // one as the addend, as in `point_in_polygon`.
    let d_of = |ip: &IntPoint, ip_next: &IntPoint| -> f64 {
        let a = ip.x.wrapping_sub(pt.x) as f64;
        let b = ip_next.y.wrapping_sub(pt.y) as f64;
        let c = ip_next.x.wrapping_sub(pt.x) as f64;
        let e = ip.y.wrapping_sub(pt.y) as f64;
        a.mul_add(b, -(c * e))
    };
    loop {
        let op_pt = out_pts[op].pt;
        let next_pt = out_pts[out_pts[op].next].pt;
        if next_pt.y == pt.y
            && ((next_pt.x == pt.x) || (op_pt.y == pt.y && ((next_pt.x > pt.x) == (op_pt.x < pt.x))))
        {
            return -1;
        }
        if (op_pt.y < pt.y) != (next_pt.y < pt.y) {
            if op_pt.x >= pt.x {
                if next_pt.x > pt.x {
                    result = 1 - result;
                } else {
                    let d = d_of(&op_pt, &next_pt);
                    if d == 0.0 {
                        return -1;
                    }
                    if (d > 0.0) == (next_pt.y > op_pt.y) {
                        result = 1 - result;
                    }
                }
            } else if next_pt.x > pt.x {
                let d = d_of(&op_pt, &next_pt);
                if d == 0.0 {
                    return -1;
                }
                if (d > 0.0) == (next_pt.y > op_pt.y) {
                    result = 1 - result;
                }
            }
        }
        op = out_pts[op].next;
        if start_op == op {
            break;
        }
    }
    result
}

/// clipper.cpp:300-313
/// C++: bool Poly2ContainsPoly1(OutPt *OutPt1, OutPt *OutPt2)
/// This is potentially very expensive! O(n^2)!
pub fn poly2_contains_poly1(out_pts: &[OutPt], out_pt1: usize, out_pt2: usize) -> bool {
    let mut op = out_pt1;
    loop {
        // nb: PointInPolygon returns 0 if false, +1 if true, -1 if pt on polygon
        let res = point_in_polygon_out_pt(&out_pts[op].pt, out_pts, out_pt2);
        if res >= 0 {
            return res > 0;
        }
        op = out_pts[op].next;
        if op == out_pt1 {
            break;
        }
    }
    true
}

/// clipper.cpp:1850-1867
/// C++: OutRec* GetLowermostRec(OutRec *outRec1, OutRec *outRec2)
pub fn get_lowermost_rec(poly_outs: &mut [OutRec], out_pts: &[OutPt], out_rec1: usize, out_rec2: usize) -> usize {
    // work out which polygon fragment has the correct hole state ...
    if poly_outs[out_rec1].bottom_pt.is_none() {
        let pts = poly_outs[out_rec1].pts.expect("GetLowermostRec reads outRec1->Pts");
        poly_outs[out_rec1].bottom_pt = Some(get_bottom_pt(out_pts, pts));
    }
    if poly_outs[out_rec2].bottom_pt.is_none() {
        let pts = poly_outs[out_rec2].pts.expect("GetLowermostRec reads outRec2->Pts");
        poly_outs[out_rec2].bottom_pt = Some(get_bottom_pt(out_pts, pts));
    }
    let out_pt1 = poly_outs[out_rec1].bottom_pt.expect("set above");
    let out_pt2 = poly_outs[out_rec2].bottom_pt.expect("set above");
    let (p1, p2) = (out_pts[out_pt1].pt, out_pts[out_pt2].pt);
    if p1.y > p2.y {
        out_rec1
    } else if p1.y < p2.y {
        out_rec2
    } else if p1.x < p2.x {
        out_rec1
    } else if p1.x > p2.x {
        out_rec2
    } else if out_pts[out_pt1].next == out_pt1 {
        out_rec2
    } else if out_pts[out_pt2].next == out_pt2 {
        out_rec1
    } else if first_is_bottom_pt(out_pts, out_pt1, out_pt2) {
        out_rec1
    } else {
        out_rec2
    }
}

/// clipper.cpp:1870-1878
/// C++: bool Param1RightOfParam2(OutRec* outRec1, OutRec* outRec2)
pub fn param1_right_of_param2(poly_outs: &[OutRec], out_rec1: usize, out_rec2: usize) -> bool {
    let mut out_rec1 = Some(out_rec1);
    loop {
        out_rec1 = poly_outs[out_rec1.expect("the loop ends on a null FirstLeft")].first_left;
        if out_rec1 == Some(out_rec2) {
            return true;
        }
        if out_rec1.is_none() {
            break;
        }
    }
    false
}

/// clipper.cpp:2066-2069
/// C++: inline bool IsMaxima(TEdge *e, const cInt Y)
#[inline]
pub fn is_maxima(edges: &[TEdge], e: Option<usize>, y: cInt) -> bool {
    match e {
        Some(e) => edges[e].top.y == y && edges[e].next_in_lml.is_none(),
        None => false,
    }
}

/// clipper.cpp:2072-2075
/// C++: inline bool IsIntermediate(TEdge *e, const cInt Y)
#[inline]
pub fn is_intermediate(edges: &[TEdge], e: usize, y: cInt) -> bool {
    edges[e].top.y == y && edges[e].next_in_lml.is_some()
}

/// clipper.cpp:2078-2091
/// C++: inline TEdge *GetMaximaPair(TEdge *e)
#[inline]
pub fn get_maxima_pair(edges: &[TEdge], e: usize) -> Option<usize> {
    let mut result: Option<usize> = None;
    let next = edges[e].next.expect("an edge in the list has a Next");
    let prev = edges[e].prev.expect("an edge in the list has a Prev");
    if edges[next].top == edges[e].top && edges[next].next_in_lml.is_none() {
        result = Some(next);
    } else if edges[prev].top == edges[e].top && edges[prev].next_in_lml.is_none() {
        result = Some(prev);
    }

    if let Some(r) = result {
        if edges[r].out_idx == SKIP
            // result is false if both NextInAEL & PrevInAEL are nil & not horizontal ...
            || (edges[r].next_in_ael == edges[r].prev_in_ael && !is_horizontal(&edges[r]))
        {
            return None;
        }
    }
    result
}

/// clipper.cpp:2187-2200
/// C++: inline void GetHorzDirection(TEdge& HorzEdge, Direction& Dir, cInt& Left, cInt& Right)
/// The three results are returned as `(Dir, Left, Right)`.
#[inline]
pub fn get_horz_direction(horz_edge: &TEdge) -> (Direction, cInt, cInt) {
    if horz_edge.bot.x < horz_edge.top.x {
        (Direction::dLeftToRight, horz_edge.bot.x, horz_edge.top.x)
    } else {
        (Direction::dRightToLeft, horz_edge.top.x, horz_edge.bot.x)
    }
}

/// clipper.cpp:2496-2500
/// C++: inline bool EdgesAdjacent(const IntersectNode &inode)
#[inline]
pub fn edges_adjacent(edges: &[TEdge], inode: &IntersectNode) -> bool {
    edges[inode.edge1].next_in_sel == Some(inode.edge2) || edges[inode.edge1].prev_in_sel == Some(inode.edge2)
}

/// clipper.cpp:2754-2766
/// C++: int PointCount(OutPt *Pts)
/// Count the number of points in a closed linked loop starting with Pts.
pub fn point_count(out_pts: &[OutPt], pts: Option<usize>) -> i32 {
    let Some(pts) = pts else { return 0 };
    let mut result = 0;
    let mut p = pts;
    loop {
        result += 1;
        p = out_pts[p].next;
        if p == pts {
            break;
        }
    }
    result
}

/// clipper.cpp:2846-2855
/// C++: inline bool E2InsertsBeforeE1(TEdge &e1, TEdge &e2)
#[inline]
pub fn e2_inserts_before_e1(e1: &TEdge, e2: &TEdge) -> bool {
    if e2.curr.x == e1.curr.x {
        if e2.top.y > e1.top.y {
            e2.top.x < top_x(e1, e2.top.y)
        } else {
            e1.top.x > top_x(e2, e1.top.y)
        }
    } else {
        e2.curr.x < e1.curr.x
    }
}

/// clipper.cpp:2858-2872
/// C++: bool GetOverlap(const cInt a1, const cInt a2, const cInt b1, const cInt b2, cInt& Left, cInt& Right)
pub fn get_overlap(a1: cInt, a2: cInt, b1: cInt, b2: cInt, left: &mut cInt, right: &mut cInt) -> bool {
    if a1 < a2 {
        if b1 < b2 {
            *left = a1.max(b1);
            *right = a2.min(b2);
        } else {
            *left = a1.max(b2);
            *right = a2.min(b1);
        }
    } else if b1 < b2 {
        *left = a2.max(b1);
        *right = a1.min(b2);
    } else {
        *left = a2.max(b2);
        *right = a1.min(b1);
    }
    *left < *right
}

/// clipper.cpp:2876-2885
/// C++: inline void UpdateOutPtIdxs(OutRec& outrec)
/// Make all points of outrec point to outrec.Idx
#[inline]
pub fn update_out_pt_idxs(out_pts: &mut [OutPt], outrec: &OutRec) {
    let pts = outrec.pts.expect("UpdateOutPtIdxs reads outrec.Pts");
    let mut op = pts;
    loop {
        out_pts[op].idx = outrec.idx;
        op = out_pts[op].prev;
        if op == pts {
            break;
        }
    }
}

/// clipper.hpp:388-504 `class Clipper : public ClipperBase`. The base class is the
/// field `base`. Edges are indices into `base.m_edges`, output points indices into
/// `m_out_pts`, output polygons indices into `m_poly_outs`.
#[derive(Debug, Clone)]
pub struct Clipper {
    pub base: ClipperBase,
    /// Output polygons.
    pub m_poly_outs: Vec<OutRec>,
    /// Output points. The C++ allocates them in chunks of `m_OutPtsChunkSize`; here
    /// they are one vector, and a point never moves.
    pub m_out_pts: Vec<OutPt>,
    /// List of free output points, to be used before taking a new point from `m_out_pts`.
    pub m_out_pts_free: usize,
    pub m_joins: Vec<Join>,
    pub m_ghost_joins: Vec<Join>,
    pub m_intersect_list: Vec<IntersectNode>,
    pub m_clip_type: ClipType,
    /// A priority queue (a binary heap) of Y coordinates.
    pub m_scanbeam: std::collections::BinaryHeap<cInt>,
    /// Maxima are collected by ProcessEdgesAtTopOfScanbeam(), consumed by ProcessHorizontal().
    pub m_maxima: Vec<cInt>,
    pub m_active_edges: Option<usize>,
    pub m_sorted_edges: Option<usize>,
    pub m_clip_fill_type: PolyFillType,
    pub m_subj_fill_type: PolyFillType,
    pub m_reverse_output: bool,
    /// Does the result go to a PolyTree or Paths?
    pub m_using_poly_tree: bool,
    pub m_strict_simple: bool,
}

impl Default for Clipper {
    /// clipper.hpp:391 `Clipper(int initOptions = 0);`
    fn default() -> Self {
        Self::new(0)
    }
}

impl Clipper {
    #[inline]
    fn e(&self, e: usize) -> &TEdge {
        &self.base.m_edges[e]
    }

    #[inline]
    fn em(&mut self, e: usize) -> &mut TEdge {
        &mut self.base.m_edges[e]
    }

    /// `op->Pt`
    #[inline]
    fn pt(&self, op: usize) -> IntPoint {
        self.m_out_pts[op].pt
    }

    /// `op->Next`
    #[inline]
    fn nx(&self, op: usize) -> usize {
        self.m_out_pts[op].next
    }

    /// `op->Prev`
    #[inline]
    fn pv(&self, op: usize) -> usize {
        self.m_out_pts[op].prev
    }

    /// `std::swap(e1->Side, e2->Side);`
    #[inline]
    fn swap_side(&mut self, e1: usize, e2: usize) {
        let (s1, s2) = (self.e(e1).side, self.e(e2).side);
        self.em(e1).side = s2;
        self.em(e2).side = s1;
    }

    /// `std::swap(e1->OutIdx, e2->OutIdx);`
    #[inline]
    fn swap_out_idx(&mut self, e1: usize, e2: usize) {
        let (o1, o2) = (self.e(e1).out_idx, self.e(e2).out_idx);
        self.em(e1).out_idx = o2;
        self.em(e2).out_idx = o1;
    }

    /// clipper.cpp:1048-1063
    /// C++: Clipper::Clipper(int initOptions)
    pub fn new(init_options: i32) -> Self {
        let mut base = ClipperBase::default();
        base.m_preserve_collinear = (init_options & IO_PRESERVE_COLLINEAR) != 0;
        base.m_has_open_paths = false;
        Self {
            base,
            m_poly_outs: Vec::new(),
            m_out_pts: Vec::new(),
            m_out_pts_free: OUT_PT_NULL,
            m_joins: Vec::new(),
            m_ghost_joins: Vec::new(),
            m_intersect_list: Vec::new(),
            // The C++ leaves the clip type and the fill types unset until Execute.
            m_clip_type: ClipType::ctIntersection,
            m_scanbeam: std::collections::BinaryHeap::new(),
            m_maxima: Vec::new(),
            m_active_edges: None,
            m_sorted_edges: None,
            m_clip_fill_type: PolyFillType::pftEvenOdd,
            m_subj_fill_type: PolyFillType::pftEvenOdd,
            m_reverse_output: (init_options & IO_REVERSE_SOLUTION) != 0,
            m_using_poly_tree: false,
            m_strict_simple: (init_options & IO_STRICTLY_SIMPLE) != 0,
        }
    }

    /// `ClipperBase::AddPath`, which `Clipper` inherits (clipper.cpp:760-782).
    pub fn add_path(&mut self, pg: &Path, poly_typ: PolyType, closed: bool) -> bool {
        self.base.add_path(pg, poly_typ, closed)
    }

    /// clipper.hpp:393
    /// C++: void Clear() { ClipperBase::Clear(); DisposeAllOutRecs(); }
    pub fn clear(&mut self) {
        self.base.clear();
        self.dispose_all_out_recs();
    }

    /// clipper.hpp:358
    /// C++: bool PreserveCollinear() const {return m_PreserveCollinear;};
    pub fn preserve_collinear(&self) -> bool {
        self.base.m_preserve_collinear
    }

    /// clipper.hpp:359
    /// C++: void PreserveCollinear(bool value) {m_PreserveCollinear = value;};
    pub fn set_preserve_collinear(&mut self, value: bool) {
        self.base.m_preserve_collinear = value;
    }

    /// clipper.hpp:410
    /// C++: bool ReverseSolution() const { return m_ReverseOutput; };
    pub fn reverse_solution(&self) -> bool {
        self.m_reverse_output
    }

    /// clipper.hpp:411
    /// C++: void ReverseSolution(bool value) {m_ReverseOutput = value;};
    pub fn set_reverse_solution(&mut self, value: bool) {
        self.m_reverse_output = value;
    }

    /// clipper.hpp:412
    /// C++: bool StrictlySimple() const {return m_StrictSimple;};
    pub fn strictly_simple(&self) -> bool {
        self.m_strict_simple
    }

    /// clipper.hpp:413
    /// C++: void StrictlySimple(bool value) {m_StrictSimple = value;};
    pub fn set_strictly_simple(&mut self, value: bool) {
        self.m_strict_simple = value;
    }

    /// clipper.hpp:452-453
    /// C++: bool IsEvenOddFillType(const TEdge& edge) const
    #[inline]
    pub fn is_even_odd_fill_type(&self, edge: usize) -> bool {
        if self.e(edge).poly_typ == PolyType::ptSubject {
            self.m_subj_fill_type == PolyFillType::pftEvenOdd
        } else {
            self.m_clip_fill_type == PolyFillType::pftEvenOdd
        }
    }

    /// clipper.hpp:454-455
    /// C++: bool IsEvenOddAltFillType(const TEdge& edge) const
    #[inline]
    pub fn is_even_odd_alt_fill_type(&self, edge: usize) -> bool {
        if self.e(edge).poly_typ == PolyType::ptSubject {
            self.m_clip_fill_type == PolyFillType::pftEvenOdd
        } else {
            self.m_subj_fill_type == PolyFillType::pftEvenOdd
        }
    }

    /// clipper.cpp:1066-1076
    /// C++: void Clipper::Reset()
    pub fn reset(&mut self) {
        self.base.reset();
        self.m_scanbeam = std::collections::BinaryHeap::new();
        self.m_maxima.clear();
        self.m_active_edges = None;
        self.m_sorted_edges = None;
        for lm in self.base.m_minima_list.iter().rev() {
            self.m_scanbeam.push(lm.y);
        }
    }

    /// clipper.cpp:1080-1095
    /// C++: bool Clipper::Execute(ClipType clipType, Paths &solution, PolyFillType subjFillType, PolyFillType clipFillType)
    pub fn execute(
        &mut self,
        clip_type: ClipType,
        solution: &mut Paths,
        subj_fill_type: PolyFillType,
        clip_fill_type: PolyFillType,
    ) -> bool {
        if self.base.m_has_open_paths {
            panic!("Error: PolyTree struct is needed for open path clipping.");
        }
        solution.clear();
        self.m_subj_fill_type = subj_fill_type;
        self.m_clip_fill_type = clip_fill_type;
        self.m_clip_type = clip_type;
        self.m_using_poly_tree = false;
        let succeeded = self.execute_internal();
        if succeeded {
            self.build_result(solution);
        }
        self.dispose_all_out_recs();
        succeeded
    }

    /// clipper.cpp:1113-1180
    /// C++: bool Clipper::ExecuteInternal()
    pub fn execute_internal(&mut self) -> bool {
        let mut succeeded = true;
        // try { ... } catch(...) { succeeded = false; }
        'try_block: {
            self.reset();
            if self.base.m_minima_list.is_empty() {
                return true;
            }
            let mut bot_y = *self.m_scanbeam.peek().expect("one Y per local minimum");
            loop {
                self.m_scanbeam.pop();
                if !(!self.m_scanbeam.is_empty() && bot_y == *self.m_scanbeam.peek().expect("not empty")) {
                    break;
                }
            }
            loop {
                self.insert_local_minima_into_ael(bot_y);
                if self.process_horizontals().is_err() {
                    succeeded = false;
                    break 'try_block;
                }
                self.m_ghost_joins.clear();
                if self.m_scanbeam.is_empty() {
                    break;
                }
                let top_y = *self.m_scanbeam.peek().expect("not empty");
                loop {
                    self.m_scanbeam.pop();
                    if !(!self.m_scanbeam.is_empty() && top_y == *self.m_scanbeam.peek().expect("not empty")) {
                        break;
                    }
                }
                succeeded = self.process_intersections(top_y);
                if !succeeded {
                    break;
                }
                if self.process_edges_at_top_of_scanbeam(top_y).is_err() {
                    succeeded = false;
                    break 'try_block;
                }
                bot_y = top_y;
                if !(!self.m_scanbeam.is_empty() || !self.base.m_minima_list.is_empty()) {
                    break;
                }
            }
        }

        if succeeded {
            // fix orientations ...
            for i in 0..self.m_poly_outs.len() {
                let out_rec = self.m_poly_outs[i];
                if out_rec.pts.is_some()
                    && !out_rec.is_open
                    && (out_rec.is_hole ^ self.m_reverse_output) == (area_out_rec(&self.m_out_pts, &out_rec) > 0.0)
                {
                    reverse_poly_pt_links(&mut self.m_out_pts, out_rec.pts);
                }
            }

            self.join_common_edges();

            // unfortunately FixupOutPolygon() must be done after JoinCommonEdges()
            for i in 0..self.m_poly_outs.len() {
                if self.m_poly_outs[i].pts.is_some() {
                    if self.m_poly_outs[i].is_open {
                        // Removes duplicate points.
                        self.fixup_out_polyline(i);
                    } else {
                        // Removes duplicate points and simplifies consecutive parallel edges by removing the middle vertex.
                        self.fixup_out_polygon(i);
                    }
                }
            }
            // For each polygon, search for exactly duplicate non-successive points.
            // If such a point is found, the loop is split into two pieces.
            // Search for the duplicate points is O(n^2)!
            if self.m_strict_simple {
                self.do_simple_polygons();
            }
        }

        self.m_joins.clear();
        self.m_ghost_joins.clear();
        succeeded
    }

    /// clipper.cpp:1183-1200
    /// C++: OutPt* Clipper::AllocateOutPt()
    /// The free list is the C++ one; a point that is not recycled is a new element of
    /// `m_out_pts` where the C++ takes the next one of its last chunk.
    pub fn allocate_out_pt(&mut self) -> usize {
        if self.m_out_pts_free != OUT_PT_NULL {
            // Recycle some of the already released points.
            let pt = self.m_out_pts_free;
            self.m_out_pts_free = self.m_out_pts[pt].next;
            pt
        } else {
            self.m_out_pts.push(OutPt::default());
            self.m_out_pts.len() - 1
        }
    }

    /// clipper.hpp:481
    /// C++: void DisposeOutPt(OutPt *pt) { pt->Next = m_OutPtsFree; m_OutPtsFree = pt; }
    /// Add the point to a list of free points.
    #[inline]
    pub fn dispose_out_pt(&mut self, pt: usize) {
        self.m_out_pts[pt].next = self.m_out_pts_free;
        self.m_out_pts_free = pt;
    }

    /// clipper.hpp:482
    /// C++: void DisposeOutPts(OutPt*& pp) { if (pp != nullptr) { pp->Prev->Next = m_OutPtsFree; m_OutPtsFree = pp; } }
    #[inline]
    pub fn dispose_out_pts(&mut self, pp: Option<usize>) {
        if let Some(pp) = pp {
            let prev = self.m_out_pts[pp].prev;
            self.m_out_pts[prev].next = self.m_out_pts_free;
            self.m_out_pts_free = pp;
        }
    }

    /// clipper.cpp:1202-1212
    /// C++: void Clipper::DisposeAllOutRecs()
    pub fn dispose_all_out_recs(&mut self) {
        self.m_out_pts.clear();
        self.m_out_pts_free = OUT_PT_NULL;
        self.m_poly_outs.clear();
    }

    /// clipper.cpp:1215-1307
    /// C++: void Clipper::SetWindingCount(TEdge &edge) const
    pub fn set_winding_count(&mut self, edge: usize) {
        let mut e = self.e(edge).prev_in_ael;
        // find the edge of the same polytype that immediately preceeds 'edge' in AEL
        while let Some(ei) = e {
            if !(self.e(ei).poly_typ != self.e(edge).poly_typ || self.e(ei).wind_delta == 0) {
                break;
            }
            e = self.e(ei).prev_in_ael;
        }
        match e {
            None => {
                let wd = self.e(edge).wind_delta;
                self.em(edge).wind_cnt = if wd == 0 { 1 } else { wd };
                self.em(edge).wind_cnt2 = 0;
                e = self.m_active_edges; // ie get ready to calc WindCnt2
            }
            Some(ei) if self.e(edge).wind_delta == 0 && self.m_clip_type != ClipType::ctUnion => {
                self.em(edge).wind_cnt = 1;
                self.em(edge).wind_cnt2 = self.e(ei).wind_cnt2;
                e = self.e(ei).next_in_ael; // ie get ready to calc WindCnt2
            }
            Some(ei) if self.is_even_odd_fill_type(edge) => {
                // EvenOdd filling ...
                if self.e(edge).wind_delta == 0 {
                    // are we inside a subj polygon ...
                    let mut inside = true;
                    let mut e2 = self.e(ei).prev_in_ael;
                    while let Some(e2i) = e2 {
                        if self.e(e2i).poly_typ == self.e(ei).poly_typ && self.e(e2i).wind_delta != 0 {
                            inside = !inside;
                        }
                        e2 = self.e(e2i).prev_in_ael;
                    }
                    self.em(edge).wind_cnt = if inside { 0 } else { 1 };
                } else {
                    self.em(edge).wind_cnt = self.e(edge).wind_delta;
                }
                self.em(edge).wind_cnt2 = self.e(ei).wind_cnt2;
                e = self.e(ei).next_in_ael; // ie get ready to calc WindCnt2
            }
            Some(ei) => {
                // nonZero, Positive or Negative filling ...
                let (e_wind_cnt, e_wind_delta) = (self.e(ei).wind_cnt, self.e(ei).wind_delta);
                let edge_wind_delta = self.e(edge).wind_delta;
                if e_wind_cnt * e_wind_delta < 0 {
                    // prev edge is 'decreasing' WindCount (WC) toward zero
                    // so we're outside the previous polygon ...
                    if e_wind_cnt.abs() > 1 {
                        // outside prev poly but still inside another.
                        // when reversing direction of prev poly use the same WC
                        if e_wind_delta * edge_wind_delta < 0 {
                            self.em(edge).wind_cnt = e_wind_cnt;
                        } else {
                            // otherwise continue to 'decrease' WC ...
                            self.em(edge).wind_cnt = e_wind_cnt + edge_wind_delta;
                        }
                    } else {
                        // now outside all polys of same polytype so set own WC ...
                        self.em(edge).wind_cnt = if edge_wind_delta == 0 { 1 } else { edge_wind_delta };
                    }
                } else {
                    // prev edge is 'increasing' WindCount (WC) away from zero
                    // so we're inside the previous polygon ...
                    if edge_wind_delta == 0 {
                        self.em(edge).wind_cnt = if e_wind_cnt < 0 { e_wind_cnt - 1 } else { e_wind_cnt + 1 };
                    } else if e_wind_delta * edge_wind_delta < 0 {
                        // if wind direction is reversing prev then use same WC
                        self.em(edge).wind_cnt = e_wind_cnt;
                    } else {
                        // otherwise add to WC ...
                        self.em(edge).wind_cnt = e_wind_cnt + edge_wind_delta;
                    }
                }
                self.em(edge).wind_cnt2 = self.e(ei).wind_cnt2;
                e = self.e(ei).next_in_ael; // ie get ready to calc WindCnt2
            }
        }

        // update WindCnt2 ...
        if self.is_even_odd_alt_fill_type(edge) {
            // EvenOdd filling ...
            while e != Some(edge) {
                let ei = e.expect("'edge' is in the AEL after e");
                if self.e(ei).wind_delta != 0 {
                    self.em(edge).wind_cnt2 = if self.e(edge).wind_cnt2 == 0 { 1 } else { 0 };
                }
                e = self.e(ei).next_in_ael;
            }
        } else {
            // nonZero, Positive or Negative filling ...
            while e != Some(edge) {
                let ei = e.expect("'edge' is in the AEL after e");
                self.em(edge).wind_cnt2 += self.e(ei).wind_delta;
                e = self.e(ei).next_in_ael;
            }
        }
    }

    /// clipper.cpp:1310-1407
    /// C++: bool Clipper::IsContributing(const TEdge& edge) const
    pub fn is_contributing(&self, edge: usize) -> bool {
        let edge = self.e(edge);
        let (pft, pft2);
        if edge.poly_typ == PolyType::ptSubject {
            pft = self.m_subj_fill_type;
            pft2 = self.m_clip_fill_type;
        } else {
            pft = self.m_clip_fill_type;
            pft2 = self.m_subj_fill_type;
        }

        match pft {
            PolyFillType::pftEvenOdd => {
                // return false if a subj line has been flagged as inside a subj polygon
                if edge.wind_delta == 0 && edge.wind_cnt != 1 {
                    return false;
                }
            }
            PolyFillType::pftNonZero => {
                if edge.wind_cnt.abs() != 1 {
                    return false;
                }
            }
            PolyFillType::pftPositive => {
                if edge.wind_cnt != 1 {
                    return false;
                }
            }
            // pftNegative
            PolyFillType::pftNegative => {
                if edge.wind_cnt != -1 {
                    return false;
                }
            }
        }

        match self.m_clip_type {
            ClipType::ctIntersection => match pft2 {
                PolyFillType::pftEvenOdd | PolyFillType::pftNonZero => edge.wind_cnt2 != 0,
                PolyFillType::pftPositive => edge.wind_cnt2 > 0,
                PolyFillType::pftNegative => edge.wind_cnt2 < 0,
            },
            ClipType::ctUnion => match pft2 {
                PolyFillType::pftEvenOdd | PolyFillType::pftNonZero => edge.wind_cnt2 == 0,
                PolyFillType::pftPositive => edge.wind_cnt2 <= 0,
                PolyFillType::pftNegative => edge.wind_cnt2 >= 0,
            },
            ClipType::ctDifference => {
                if edge.poly_typ == PolyType::ptSubject {
                    match pft2 {
                        PolyFillType::pftEvenOdd | PolyFillType::pftNonZero => edge.wind_cnt2 == 0,
                        PolyFillType::pftPositive => edge.wind_cnt2 <= 0,
                        PolyFillType::pftNegative => edge.wind_cnt2 >= 0,
                    }
                } else {
                    match pft2 {
                        PolyFillType::pftEvenOdd | PolyFillType::pftNonZero => edge.wind_cnt2 != 0,
                        PolyFillType::pftPositive => edge.wind_cnt2 > 0,
                        PolyFillType::pftNegative => edge.wind_cnt2 < 0,
                    }
                }
            }
            ClipType::ctXor => {
                if edge.wind_delta == 0 {
                    // XOr always contributing unless open
                    match pft2 {
                        PolyFillType::pftEvenOdd | PolyFillType::pftNonZero => edge.wind_cnt2 == 0,
                        PolyFillType::pftPositive => edge.wind_cnt2 <= 0,
                        PolyFillType::pftNegative => edge.wind_cnt2 >= 0,
                    }
                } else {
                    true
                }
            }
        }
    }

    /// clipper.cpp:1411-1449
    /// C++: OutPt* Clipper::AddLocalMinPoly(TEdge *e1, TEdge *e2, const IntPoint &Pt)
    /// Called from Clipper::InsertLocalMinimaIntoAEL() and Clipper::IntersectEdges().
    pub fn add_local_min_poly(&mut self, e1: usize, e2: usize, pt: &IntPoint) -> usize {
        let result;
        let e;
        let prev_e;
        if is_horizontal(self.e(e2)) || self.e(e1).dx > self.e(e2).dx {
            result = self.add_out_pt(e1, pt);
            self.em(e2).out_idx = self.e(e1).out_idx;
            self.em(e1).side = EdgeSide::esLeft;
            self.em(e2).side = EdgeSide::esRight;
            e = e1;
            if self.e(e).prev_in_ael == Some(e2) {
                prev_e = self.e(e2).prev_in_ael;
            } else {
                prev_e = self.e(e).prev_in_ael;
            }
        } else {
            result = self.add_out_pt(e2, pt);
            self.em(e1).out_idx = self.e(e2).out_idx;
            self.em(e1).side = EdgeSide::esRight;
            self.em(e2).side = EdgeSide::esLeft;
            e = e2;
            if self.e(e).prev_in_ael == Some(e1) {
                prev_e = self.e(e1).prev_in_ael;
            } else {
                prev_e = self.e(e).prev_in_ael;
            }
        }

        if let Some(prev_e) = prev_e {
            if self.e(prev_e).out_idx >= 0
                && top_x(self.e(prev_e), pt.y) == top_x(self.e(e), pt.y)
                && slopes_equal_edges(self.e(e), self.e(prev_e))
                && self.e(e).wind_delta != 0
                && self.e(prev_e).wind_delta != 0
            {
                let out_pt = self.add_out_pt(prev_e, pt);
                let off_pt = self.e(e).top;
                self.m_joins.push(Join { out_pt1: Some(result), out_pt2: Some(out_pt), off_pt });
            }
        }
        result
    }

    /// clipper.cpp:1452-1465
    /// C++: void Clipper::AddLocalMaxPoly(TEdge *e1, TEdge *e2, const IntPoint &Pt)
    pub fn add_local_max_poly(&mut self, e1: usize, e2: usize, pt: &IntPoint) {
        self.add_out_pt(e1, pt);
        if self.e(e2).wind_delta == 0 {
            self.add_out_pt(e2, pt);
        }
        if self.e(e1).out_idx == self.e(e2).out_idx {
            self.em(e1).out_idx = UNASSIGNED;
            self.em(e2).out_idx = UNASSIGNED;
        } else if self.e(e1).out_idx < self.e(e2).out_idx {
            self.append_polygon(e1, e2);
        } else {
            self.append_polygon(e2, e1);
        }
    }

    /// clipper.cpp:1468-1485
    /// C++: void Clipper::AddEdgeToSEL(TEdge *edge)
    pub fn add_edge_to_sel(&mut self, edge: usize) {
        // SEL pointers in PEdge are reused to build a list of horizontal edges.
        // However, we don't need to worry about order with horizontal edge processing.
        match self.m_sorted_edges {
            None => {
                self.m_sorted_edges = Some(edge);
                self.em(edge).prev_in_sel = None;
                self.em(edge).next_in_sel = None;
            }
            Some(sorted) => {
                self.em(edge).next_in_sel = Some(sorted);
                self.em(edge).prev_in_sel = None;
                self.em(sorted).prev_in_sel = Some(edge);
                self.m_sorted_edges = Some(edge);
            }
        }
    }

    /// clipper.cpp:1488-1498
    /// C++: void Clipper::CopyAELToSEL()
    pub fn copy_ael_to_sel(&mut self) {
        let mut e = self.m_active_edges;
        self.m_sorted_edges = e;
        while let Some(ei) = e {
            let ed = self.em(ei);
            ed.prev_in_sel = ed.prev_in_ael;
            ed.next_in_sel = ed.next_in_ael;
            e = ed.next_in_ael;
        }
    }

    /// clipper.cpp:1503-1595
    /// C++: void Clipper::InsertLocalMinimaIntoAEL(const cInt botY)
    /// Called from Clipper::ExecuteInternal()
    pub fn insert_local_minima_into_ael(&mut self, bot_y: cInt) {
        while !self.base.m_minima_list.is_empty()
            && self.base.m_minima_list.last().expect("not empty").y == bot_y
        {
            let back = self.base.m_minima_list.pop().expect("not empty");
            let lb = back.left_bound;
            let rb = back.right_bound;

            let mut op1: Option<usize> = None;
            match (lb, rb) {
                (None, _) => {
                    let rb = rb.expect("a local minimum has a bound");
                    // nb: don't insert LB into either AEL or SEL
                    self.insert_edge_into_ael(rb, None);
                    self.set_winding_count(rb);
                    if self.is_contributing(rb) {
                        let bot = self.e(rb).bot;
                        op1 = Some(self.add_out_pt(rb, &bot));
                    }
                }
                (Some(lb), None) => {
                    self.insert_edge_into_ael(lb, None);
                    self.set_winding_count(lb);
                    if self.is_contributing(lb) {
                        let bot = self.e(lb).bot;
                        op1 = Some(self.add_out_pt(lb, &bot));
                    }
                    self.m_scanbeam.push(self.e(lb).top.y);
                }
                (Some(lb), Some(rb)) => {
                    self.insert_edge_into_ael(lb, None);
                    self.insert_edge_into_ael(rb, Some(lb));
                    self.set_winding_count(lb);
                    self.em(rb).wind_cnt = self.e(lb).wind_cnt;
                    self.em(rb).wind_cnt2 = self.e(lb).wind_cnt2;
                    if self.is_contributing(lb) {
                        let bot = self.e(lb).bot;
                        op1 = Some(self.add_local_min_poly(lb, rb, &bot));
                    }
                    self.m_scanbeam.push(self.e(lb).top.y);
                }
            }

            if let Some(rb) = rb {
                if is_horizontal(self.e(rb)) {
                    self.add_edge_to_sel(rb);
                } else {
                    self.m_scanbeam.push(self.e(rb).top.y);
                }
            }

            let (Some(lb), Some(rb)) = (lb, rb) else { continue };

            // if any output polygons share an edge, they'll need joining later ...
            if op1.is_some() && is_horizontal(self.e(rb)) && !self.m_ghost_joins.is_empty() && self.e(rb).wind_delta != 0 {
                for i in 0..self.m_ghost_joins.len() {
                    let jr = self.m_ghost_joins[i];
                    // if the horizontal Rb and a 'ghost' horizontal overlap, then convert
                    // the 'ghost' join to a real join ready for later ...
                    let jr_out_pt1 = jr.out_pt1.expect("a ghost join has OutPt1");
                    if horz_segments_overlap(self.pt(jr_out_pt1).x, jr.off_pt.x, self.e(rb).bot.x, self.e(rb).top.x) {
                        self.m_joins.push(Join { out_pt1: jr.out_pt1, out_pt2: op1, off_pt: jr.off_pt });
                    }
                }
            }

            if self.e(lb).out_idx >= 0 {
                if let Some(lb_prev) = self.e(lb).prev_in_ael {
                    if self.e(lb_prev).curr.x == self.e(lb).bot.x
                        && self.e(lb_prev).out_idx >= 0
                        && slopes_equal_edges(self.e(lb_prev), self.e(lb))
                        && self.e(lb).wind_delta != 0
                        && self.e(lb_prev).wind_delta != 0
                    {
                        let bot = self.e(lb).bot;
                        let op2 = self.add_out_pt(lb_prev, &bot);
                        let off_pt = self.e(lb).top;
                        self.m_joins.push(Join { out_pt1: op1, out_pt2: Some(op2), off_pt });
                    }
                }
            }

            if self.e(lb).next_in_ael != Some(rb) {
                if self.e(rb).out_idx >= 0 {
                    let rb_prev = self.e(rb).prev_in_ael.expect("InsertLocalMinimaIntoAEL reads rb->PrevInAEL");
                    if self.e(rb_prev).out_idx >= 0
                        && slopes_equal_edges(self.e(rb_prev), self.e(rb))
                        && self.e(rb).wind_delta != 0
                        && self.e(rb_prev).wind_delta != 0
                    {
                        let bot = self.e(rb).bot;
                        let op2 = self.add_out_pt(rb_prev, &bot);
                        let off_pt = self.e(rb).top;
                        self.m_joins.push(Join { out_pt1: op1, out_pt2: Some(op2), off_pt });
                    }
                }

                let mut e = self.e(lb).next_in_ael;
                if e.is_some() {
                    while e != Some(rb) {
                        let ei = e.expect("rb is in the AEL after lb");
                        // nb: For calculating winding counts etc, IntersectEdges() assumes
                        // that param1 will be to the Right of param2 ABOVE the intersection ...
                        let curr = self.e(lb).curr;
                        self.intersect_edges(rb, ei, &curr); // order important here
                        e = self.e(ei).next_in_ael;
                    }
                }
            }
        }
    }

    /// clipper.cpp:1598-1608
    /// C++: void Clipper::DeleteFromAEL(TEdge *e)
    pub fn delete_from_ael(&mut self, e: usize) {
        let ael_prev = self.e(e).prev_in_ael;
        let ael_next = self.e(e).next_in_ael;
        if ael_prev.is_none() && ael_next.is_none() && Some(e) != self.m_active_edges {
            return; // already deleted
        }
        if let Some(p) = ael_prev {
            self.em(p).next_in_ael = ael_next;
        } else {
            self.m_active_edges = ael_next;
        }
        if let Some(n) = ael_next {
            self.em(n).prev_in_ael = ael_prev;
        }
        self.em(e).next_in_ael = None;
        self.em(e).prev_in_ael = None;
    }

    /// clipper.cpp:1611-1621
    /// C++: void Clipper::DeleteFromSEL(TEdge *e)
    pub fn delete_from_sel(&mut self, e: usize) {
        let sel_prev = self.e(e).prev_in_sel;
        let sel_next = self.e(e).next_in_sel;
        if sel_prev.is_none() && sel_next.is_none() && Some(e) != self.m_sorted_edges {
            return; // already deleted
        }
        if let Some(p) = sel_prev {
            self.em(p).next_in_sel = sel_next;
        } else {
            self.m_sorted_edges = sel_next;
        }
        if let Some(n) = sel_next {
            self.em(n).prev_in_sel = sel_prev;
        }
        self.em(e).next_in_sel = None;
        self.em(e).prev_in_sel = None;
    }

    /// clipper.cpp:1637-1829
    /// C++: void Clipper::IntersectEdges(TEdge *e1, TEdge *e2, IntPoint &Pt)
    pub fn intersect_edges(&mut self, e1: usize, e2: usize, pt: &IntPoint) {
        let e1_contributing = self.e(e1).out_idx >= 0;
        let e2_contributing = self.e(e2).out_idx >= 0;

        // `use_lines` is defined (clipper.hpp:48).
        // if either edge is on an OPEN path ...
        if self.e(e1).wind_delta == 0 || self.e(e2).wind_delta == 0 {
            // ignore subject-subject open path intersections UNLESS they
            // are both open paths, AND they are both 'contributing maximas' ...
            if self.e(e1).wind_delta == 0 && self.e(e2).wind_delta == 0 {
                return;
            }
            // if intersecting a subj line with a subj poly ...
            else if self.e(e1).poly_typ == self.e(e2).poly_typ
                && self.e(e1).wind_delta != self.e(e2).wind_delta
                && self.m_clip_type == ClipType::ctUnion
            {
                if self.e(e1).wind_delta == 0 {
                    if e2_contributing {
                        self.add_out_pt(e1, pt);
                        if e1_contributing {
                            self.em(e1).out_idx = UNASSIGNED;
                        }
                    }
                } else if e1_contributing {
                    self.add_out_pt(e2, pt);
                    if e2_contributing {
                        self.em(e2).out_idx = UNASSIGNED;
                    }
                }
            } else if self.e(e1).poly_typ != self.e(e2).poly_typ {
                // toggle subj open path OutIdx on/off when Abs(clip.WndCnt) == 1 ...
                if self.e(e1).wind_delta == 0
                    && self.e(e2).wind_cnt.abs() == 1
                    && (self.m_clip_type != ClipType::ctUnion || self.e(e2).wind_cnt2 == 0)
                {
                    self.add_out_pt(e1, pt);
                    if e1_contributing {
                        self.em(e1).out_idx = UNASSIGNED;
                    }
                } else if self.e(e2).wind_delta == 0
                    && self.e(e1).wind_cnt.abs() == 1
                    && (self.m_clip_type != ClipType::ctUnion || self.e(e1).wind_cnt2 == 0)
                {
                    self.add_out_pt(e2, pt);
                    if e2_contributing {
                        self.em(e2).out_idx = UNASSIGNED;
                    }
                }
            }
            return;
        }

        // update winding counts...
        // assumes that e1 will be to the Right of e2 ABOVE the intersection
        if self.e(e1).poly_typ == self.e(e2).poly_typ {
            if self.is_even_odd_fill_type(e1) {
                let old_e1_wind_cnt = self.e(e1).wind_cnt;
                self.em(e1).wind_cnt = self.e(e2).wind_cnt;
                self.em(e2).wind_cnt = old_e1_wind_cnt;
            } else {
                if self.e(e1).wind_cnt + self.e(e2).wind_delta == 0 {
                    self.em(e1).wind_cnt = -self.e(e1).wind_cnt;
                } else {
                    self.em(e1).wind_cnt += self.e(e2).wind_delta;
                }
                if self.e(e2).wind_cnt - self.e(e1).wind_delta == 0 {
                    self.em(e2).wind_cnt = -self.e(e2).wind_cnt;
                } else {
                    self.em(e2).wind_cnt -= self.e(e1).wind_delta;
                }
            }
        } else {
            if !self.is_even_odd_fill_type(e2) {
                self.em(e1).wind_cnt2 += self.e(e2).wind_delta;
            } else {
                self.em(e1).wind_cnt2 = if self.e(e1).wind_cnt2 == 0 { 1 } else { 0 };
            }
            if !self.is_even_odd_fill_type(e1) {
                self.em(e2).wind_cnt2 -= self.e(e1).wind_delta;
            } else {
                self.em(e2).wind_cnt2 = if self.e(e2).wind_cnt2 == 0 { 1 } else { 0 };
            }
        }

        let (e1_fill_type, e2_fill_type, e1_fill_type2, e2_fill_type2);
        if self.e(e1).poly_typ == PolyType::ptSubject {
            e1_fill_type = self.m_subj_fill_type;
            e1_fill_type2 = self.m_clip_fill_type;
        } else {
            e1_fill_type = self.m_clip_fill_type;
            e1_fill_type2 = self.m_subj_fill_type;
        }
        if self.e(e2).poly_typ == PolyType::ptSubject {
            e2_fill_type = self.m_subj_fill_type;
            e2_fill_type2 = self.m_clip_fill_type;
        } else {
            e2_fill_type = self.m_clip_fill_type;
            e2_fill_type2 = self.m_subj_fill_type;
        }

        let e1_wc: cInt = match e1_fill_type {
            PolyFillType::pftPositive => self.e(e1).wind_cnt,
            PolyFillType::pftNegative => -self.e(e1).wind_cnt,
            _ => self.e(e1).wind_cnt.abs(),
        };
        let e2_wc: cInt = match e2_fill_type {
            PolyFillType::pftPositive => self.e(e2).wind_cnt,
            PolyFillType::pftNegative => -self.e(e2).wind_cnt,
            _ => self.e(e2).wind_cnt.abs(),
        };

        if e1_contributing && e2_contributing {
            if (e1_wc != 0 && e1_wc != 1)
                || (e2_wc != 0 && e2_wc != 1)
                || (self.e(e1).poly_typ != self.e(e2).poly_typ && self.m_clip_type != ClipType::ctXor)
            {
                self.add_local_max_poly(e1, e2, pt);
            } else {
                self.add_out_pt(e1, pt);
                self.add_out_pt(e2, pt);
                self.swap_side(e1, e2);
                self.swap_out_idx(e1, e2);
            }
        } else if e1_contributing {
            if e2_wc == 0 || e2_wc == 1 {
                self.add_out_pt(e1, pt);
                self.swap_side(e1, e2);
                self.swap_out_idx(e1, e2);
            }
        } else if e2_contributing {
            if e1_wc == 0 || e1_wc == 1 {
                self.add_out_pt(e2, pt);
                self.swap_side(e1, e2);
                self.swap_out_idx(e1, e2);
            }
        } else if (e1_wc == 0 || e1_wc == 1) && (e2_wc == 0 || e2_wc == 1) {
            // neither edge is currently contributing ...

            let e1_wc2: cInt = match e1_fill_type2 {
                PolyFillType::pftPositive => self.e(e1).wind_cnt2,
                PolyFillType::pftNegative => -self.e(e1).wind_cnt2,
                _ => self.e(e1).wind_cnt2.abs(),
            };
            let e2_wc2: cInt = match e2_fill_type2 {
                PolyFillType::pftPositive => self.e(e2).wind_cnt2,
                PolyFillType::pftNegative => -self.e(e2).wind_cnt2,
                _ => self.e(e2).wind_cnt2.abs(),
            };

            if self.e(e1).poly_typ != self.e(e2).poly_typ {
                self.add_local_min_poly(e1, e2, pt);
            } else if e1_wc == 1 && e2_wc == 1 {
                match self.m_clip_type {
                    ClipType::ctIntersection => {
                        if e1_wc2 > 0 && e2_wc2 > 0 {
                            self.add_local_min_poly(e1, e2, pt);
                        }
                    }
                    ClipType::ctUnion => {
                        if e1_wc2 <= 0 && e2_wc2 <= 0 {
                            self.add_local_min_poly(e1, e2, pt);
                        }
                    }
                    ClipType::ctDifference => {
                        if (self.e(e1).poly_typ == PolyType::ptClip && e1_wc2 > 0 && e2_wc2 > 0)
                            || (self.e(e1).poly_typ == PolyType::ptSubject && e1_wc2 <= 0 && e2_wc2 <= 0)
                        {
                            self.add_local_min_poly(e1, e2, pt);
                        }
                    }
                    ClipType::ctXor => {
                        self.add_local_min_poly(e1, e2, pt);
                    }
                }
            } else {
                self.swap_side(e1, e2);
            }
        }
    }

    /// clipper.cpp:1832-1847
    /// C++: void Clipper::SetHoleState(TEdge *e, OutRec *outrec) const
    pub fn set_hole_state(&mut self, e: usize, outrec: usize) {
        let mut is_hole = false;
        let mut e2 = self.e(e).prev_in_ael;
        while let Some(e2i) = e2 {
            if self.e(e2i).out_idx >= 0 && self.e(e2i).wind_delta != 0 {
                is_hole = !is_hole;
                if self.m_poly_outs[outrec].first_left.is_none() {
                    self.m_poly_outs[outrec].first_left = Some(self.e(e2i).out_idx as usize);
                }
            }
            e2 = self.e(e2i).prev_in_ael;
        }
        if is_hole {
            self.m_poly_outs[outrec].is_hole = true;
        }
    }

    /// clipper.cpp:1881-1887
    /// C++: OutRec* Clipper::GetOutRec(int Idx)
    pub fn get_out_rec(&self, idx: i32) -> usize {
        let mut outrec = idx as usize;
        while outrec != self.m_poly_outs[outrec].idx as usize {
            outrec = self.m_poly_outs[outrec].idx as usize;
        }
        outrec
    }

    /// clipper.cpp:1890-1986
    /// C++: void Clipper::AppendPolygon(TEdge *e1, TEdge *e2) const
    pub fn append_polygon(&mut self, e1: usize, e2: usize) {
        // get the start and ends of both output polygons ...
        let out_rec1 = self.e(e1).out_idx as usize;
        let out_rec2 = self.e(e2).out_idx as usize;

        let hole_state_rec;
        if param1_right_of_param2(&self.m_poly_outs, out_rec1, out_rec2) {
            hole_state_rec = out_rec2;
        } else if param1_right_of_param2(&self.m_poly_outs, out_rec2, out_rec1) {
            hole_state_rec = out_rec1;
        } else {
            hole_state_rec = get_lowermost_rec(&mut self.m_poly_outs, &self.m_out_pts, out_rec1, out_rec2);
        }

        // get the start and ends of both output polygons and
        // join e2 poly onto e1 poly and delete pointers to e2 ...

        let p1_lft = self.m_poly_outs[out_rec1].pts.expect("AppendPolygon reads outRec1->Pts");
        let p1_rt = self.pv(p1_lft);
        let p2_lft = self.m_poly_outs[out_rec2].pts.expect("AppendPolygon reads outRec2->Pts");
        let p2_rt = self.pv(p2_lft);

        let side;
        // join e2 poly onto e1 poly and delete pointers to e2 ...
        if self.e(e1).side == EdgeSide::esLeft {
            if self.e(e2).side == EdgeSide::esLeft {
                // z y x a b c
                reverse_poly_pt_links(&mut self.m_out_pts, Some(p2_lft));
                self.m_out_pts[p2_lft].next = p1_lft;
                self.m_out_pts[p1_lft].prev = p2_lft;
                self.m_out_pts[p1_rt].next = p2_rt;
                self.m_out_pts[p2_rt].prev = p1_rt;
                self.m_poly_outs[out_rec1].pts = Some(p2_rt);
            } else {
                // x y z a b c
                self.m_out_pts[p2_rt].next = p1_lft;
                self.m_out_pts[p1_lft].prev = p2_rt;
                self.m_out_pts[p2_lft].prev = p1_rt;
                self.m_out_pts[p1_rt].next = p2_lft;
                self.m_poly_outs[out_rec1].pts = Some(p2_lft);
            }
            side = EdgeSide::esLeft;
        } else {
            if self.e(e2).side == EdgeSide::esRight {
                // a b c z y x
                reverse_poly_pt_links(&mut self.m_out_pts, Some(p2_lft));
                self.m_out_pts[p1_rt].next = p2_rt;
                self.m_out_pts[p2_rt].prev = p1_rt;
                self.m_out_pts[p2_lft].next = p1_lft;
                self.m_out_pts[p1_lft].prev = p2_lft;
            } else {
                // a b c x y z
                self.m_out_pts[p1_rt].next = p2_lft;
                self.m_out_pts[p2_lft].prev = p1_rt;
                self.m_out_pts[p1_lft].prev = p2_rt;
                self.m_out_pts[p2_rt].next = p1_lft;
            }
            side = EdgeSide::esRight;
        }

        self.m_poly_outs[out_rec1].bottom_pt = None;
        if hole_state_rec == out_rec2 {
            if self.m_poly_outs[out_rec2].first_left != Some(out_rec1) {
                self.m_poly_outs[out_rec1].first_left = self.m_poly_outs[out_rec2].first_left;
            }
            self.m_poly_outs[out_rec1].is_hole = self.m_poly_outs[out_rec2].is_hole;
        }
        self.m_poly_outs[out_rec2].pts = None;
        self.m_poly_outs[out_rec2].bottom_pt = None;
        self.m_poly_outs[out_rec2].first_left = Some(out_rec1);

        let ok_idx = self.e(e1).out_idx;
        let obsolete_idx = self.e(e2).out_idx;

        self.em(e1).out_idx = UNASSIGNED; // nb: safe because we only get here via AddLocalMaxPoly
        self.em(e2).out_idx = UNASSIGNED;

        let mut e = self.m_active_edges;
        while let Some(ei) = e {
            if self.e(ei).out_idx == obsolete_idx {
                self.em(ei).out_idx = ok_idx;
                self.em(ei).side = side;
                break;
            }
            e = self.e(ei).next_in_ael;
        }

        self.m_poly_outs[out_rec2].idx = self.m_poly_outs[out_rec1].idx;
    }

    /// clipper.cpp:1989-2001
    /// C++: OutRec* Clipper::CreateOutRec()
    pub fn create_out_rec(&mut self) -> usize {
        let mut result = OutRec {
            idx: 0,
            is_hole: false,
            is_open: false,
            first_left: None,
            pts: None,
            bottom_pt: None,
        };
        result.idx = self.m_poly_outs.len() as i32;
        self.m_poly_outs.push(result);
        self.m_poly_outs.len() - 1
    }

    /// clipper.cpp:2004-2040
    /// C++: OutPt* Clipper::AddOutPt(TEdge *e, const IntPoint &pt)
    pub fn add_out_pt(&mut self, e: usize, pt: &IntPoint) -> usize {
        if self.e(e).out_idx < 0 {
            let out_rec = self.create_out_rec();
            self.m_poly_outs[out_rec].is_open = self.e(e).wind_delta == 0;
            let new_op = self.allocate_out_pt();
            self.m_poly_outs[out_rec].pts = Some(new_op);
            self.m_out_pts[new_op].idx = self.m_poly_outs[out_rec].idx;
            self.m_out_pts[new_op].pt = *pt;
            self.m_out_pts[new_op].next = new_op;
            self.m_out_pts[new_op].prev = new_op;
            if !self.m_poly_outs[out_rec].is_open {
                self.set_hole_state(e, out_rec);
            }
            self.em(e).out_idx = self.m_poly_outs[out_rec].idx;
            new_op
        } else {
            let out_rec = self.e(e).out_idx as usize;
            // OutRec.Pts is the 'Left-most' point & OutRec.Pts.Prev is the 'Right-most'
            let op = self.m_poly_outs[out_rec].pts.expect("AddOutPt reads outRec->Pts");

            let to_front = self.e(e).side == EdgeSide::esLeft;
            if to_front && *pt == self.pt(op) {
                return op;
            } else if !to_front && *pt == self.pt(self.pv(op)) {
                return self.pv(op);
            }

            let new_op = self.allocate_out_pt();
            self.m_out_pts[new_op].idx = self.m_poly_outs[out_rec].idx;
            self.m_out_pts[new_op].pt = *pt;
            self.m_out_pts[new_op].next = op;
            let op_prev = self.pv(op);
            self.m_out_pts[new_op].prev = op_prev;
            self.m_out_pts[op_prev].next = new_op;
            self.m_out_pts[op].prev = new_op;
            if to_front {
                self.m_poly_outs[out_rec].pts = Some(new_op);
            }
            new_op
        }
    }

    /// clipper.cpp:2043-2050
    /// C++: OutPt* Clipper::GetLastOutPt(TEdge *e)
    pub fn get_last_out_pt(&self, e: usize) -> usize {
        let out_rec = self.e(e).out_idx as usize;
        let pts = self.m_poly_outs[out_rec].pts.expect("GetLastOutPt reads outRec->Pts");
        if self.e(e).side == EdgeSide::esLeft {
            pts
        } else {
            self.pv(pts)
        }
    }

    /// clipper.cpp:2053-2063
    /// C++: void Clipper::ProcessHorizontals()
    pub fn process_horizontals(&mut self) -> Result<(), ClipperException> {
        let mut horz_edge = self.m_sorted_edges;
        while let Some(h) = horz_edge {
            self.delete_from_sel(h);
            self.process_horizontal(h)?;
            horz_edge = self.m_sorted_edges;
        }
        Ok(())
    }

    /// clipper.cpp:2094-2138
    /// C++: void Clipper::SwapPositionsInAEL(TEdge *Edge1, TEdge *Edge2)
    pub fn swap_positions_in_ael(&mut self, edge1: usize, edge2: usize) {
        // check that one or other edge hasn't already been removed from AEL ...
        if self.e(edge1).next_in_ael == self.e(edge1).prev_in_ael
            || self.e(edge2).next_in_ael == self.e(edge2).prev_in_ael
        {
            return;
        }

        if self.e(edge1).next_in_ael == Some(edge2) {
            let next = self.e(edge2).next_in_ael;
            if let Some(n) = next {
                self.em(n).prev_in_ael = Some(edge1);
            }
            let prev = self.e(edge1).prev_in_ael;
            if let Some(p) = prev {
                self.em(p).next_in_ael = Some(edge2);
            }
            self.em(edge2).prev_in_ael = prev;
            self.em(edge2).next_in_ael = Some(edge1);
            self.em(edge1).prev_in_ael = Some(edge2);
            self.em(edge1).next_in_ael = next;
        } else if self.e(edge2).next_in_ael == Some(edge1) {
            let next = self.e(edge1).next_in_ael;
            if let Some(n) = next {
                self.em(n).prev_in_ael = Some(edge2);
            }
            let prev = self.e(edge2).prev_in_ael;
            if let Some(p) = prev {
                self.em(p).next_in_ael = Some(edge1);
            }
            self.em(edge1).prev_in_ael = prev;
            self.em(edge1).next_in_ael = Some(edge2);
            self.em(edge2).prev_in_ael = Some(edge1);
            self.em(edge2).next_in_ael = next;
        } else {
            let next = self.e(edge1).next_in_ael;
            let prev = self.e(edge1).prev_in_ael;
            self.em(edge1).next_in_ael = self.e(edge2).next_in_ael;
            if let Some(n) = self.e(edge1).next_in_ael {
                self.em(n).prev_in_ael = Some(edge1);
            }
            self.em(edge1).prev_in_ael = self.e(edge2).prev_in_ael;
            if let Some(p) = self.e(edge1).prev_in_ael {
                self.em(p).next_in_ael = Some(edge1);
            }
            self.em(edge2).next_in_ael = next;
            if let Some(n) = self.e(edge2).next_in_ael {
                self.em(n).prev_in_ael = Some(edge2);
            }
            self.em(edge2).prev_in_ael = prev;
            if let Some(p) = self.e(edge2).prev_in_ael {
                self.em(p).next_in_ael = Some(edge2);
            }
        }

        if self.e(edge1).prev_in_ael.is_none() {
            self.m_active_edges = Some(edge1);
        } else if self.e(edge2).prev_in_ael.is_none() {
            self.m_active_edges = Some(edge2);
        }
    }

    /// clipper.cpp:2141-2184
    /// C++: void Clipper::SwapPositionsInSEL(TEdge *Edge1, TEdge *Edge2)
    pub fn swap_positions_in_sel(&mut self, edge1: usize, edge2: usize) {
        if self.e(edge1).next_in_sel.is_none() && self.e(edge1).prev_in_sel.is_none() {
            return;
        }
        if self.e(edge2).next_in_sel.is_none() && self.e(edge2).prev_in_sel.is_none() {
            return;
        }

        if self.e(edge1).next_in_sel == Some(edge2) {
            let next = self.e(edge2).next_in_sel;
            if let Some(n) = next {
                self.em(n).prev_in_sel = Some(edge1);
            }
            let prev = self.e(edge1).prev_in_sel;
            if let Some(p) = prev {
                self.em(p).next_in_sel = Some(edge2);
            }
            self.em(edge2).prev_in_sel = prev;
            self.em(edge2).next_in_sel = Some(edge1);
            self.em(edge1).prev_in_sel = Some(edge2);
            self.em(edge1).next_in_sel = next;
        } else if self.e(edge2).next_in_sel == Some(edge1) {
            let next = self.e(edge1).next_in_sel;
            if let Some(n) = next {
                self.em(n).prev_in_sel = Some(edge2);
            }
            let prev = self.e(edge2).prev_in_sel;
            if let Some(p) = prev {
                self.em(p).next_in_sel = Some(edge1);
            }
            self.em(edge1).prev_in_sel = prev;
            self.em(edge1).next_in_sel = Some(edge2);
            self.em(edge2).prev_in_sel = Some(edge1);
            self.em(edge2).next_in_sel = next;
        } else {
            let next = self.e(edge1).next_in_sel;
            let prev = self.e(edge1).prev_in_sel;
            self.em(edge1).next_in_sel = self.e(edge2).next_in_sel;
            if let Some(n) = self.e(edge1).next_in_sel {
                self.em(n).prev_in_sel = Some(edge1);
            }
            self.em(edge1).prev_in_sel = self.e(edge2).prev_in_sel;
            if let Some(p) = self.e(edge1).prev_in_sel {
                self.em(p).next_in_sel = Some(edge1);
            }
            self.em(edge2).next_in_sel = next;
            if let Some(n) = self.e(edge2).next_in_sel {
                self.em(n).prev_in_sel = Some(edge2);
            }
            self.em(edge2).prev_in_sel = prev;
            if let Some(p) = self.e(edge2).prev_in_sel {
                self.em(p).next_in_sel = Some(edge2);
            }
        }

        if self.e(edge1).prev_in_sel.is_none() {
            self.m_sorted_edges = Some(edge1);
        } else if self.e(edge2).prev_in_sel.is_none() {
            self.m_sorted_edges = Some(edge2);
        }
    }

    /// clipper.cpp:2213-2397
    /// C++: void Clipper::ProcessHorizontal(TEdge *horzEdge)
    ///
    /// Notes: Horizontal edges (HEs) at scanline intersections (ie at the Top or
    /// Bottom of a scanbeam) are processed as if layered. The order in which HEs
    /// are processed doesn't matter. HEs intersect with other HE Bot.x()s only [#]
    /// (or they could intersect with Top.x()s only, ie EITHER Bot.x()s OR Top.x()s),
    /// and with other non-horizontal edges [*]. Once these intersections are
    /// processed, intermediate HEs then 'promote' the Edge above (NextInLML) into
    /// the AEL. These 'promoted' edges may in turn intersect [%] with other HEs.
    pub fn process_horizontal(&mut self, mut horz_edge: usize) -> Result<(), ClipperException> {
        let is_open = self.e(horz_edge).out_idx >= 0 && self.m_poly_outs[self.e(horz_edge).out_idx as usize].is_open;

        let (mut dir, mut horz_left, mut horz_right) = get_horz_direction(self.e(horz_edge));

        let mut e_last_horz = horz_edge;
        let mut e_max_pair: Option<usize> = None;
        while let Some(n) = self.e(e_last_horz).next_in_lml {
            if !is_horizontal(self.e(n)) {
                break;
            }
            e_last_horz = n;
        }
        if self.e(e_last_horz).next_in_lml.is_none() {
            e_max_pair = get_maxima_pair(&self.base.m_edges, e_last_horz);
        }

        // std::vector<cInt>::const_iterator maxIt: an index into m_Maxima, end() is the
        // length. std::vector<cInt>::const_reverse_iterator maxRit: the number of
        // elements passed from the back, rend() is the length.
        //
        // FIDELITY-NOTE: the C++ sets only the iterator of the first horizontal's
        // direction and leaves the other one default-constructed. A run of
        // consecutive horizontals that changes direction (open paths only: in a
        // closed path two consecutive horizontals are collinear and AddPath merges
        // them) would then read that iterator, which is undefined behaviour in the
        // C++. Here it starts at the beginning of its direction.
        let n_maxima = self.m_maxima.len();
        let mut max_it: usize = 0;
        let mut max_rit: usize = 0;
        if !self.m_maxima.is_empty() {
            // get the first maxima in range (X) ...
            if dir == Direction::dLeftToRight {
                max_it = 0;
                while max_it != n_maxima && self.m_maxima[max_it] <= self.e(horz_edge).bot.x {
                    max_it += 1;
                }
                if max_it != n_maxima && self.m_maxima[max_it] >= self.e(e_last_horz).top.x {
                    max_it = n_maxima;
                }
            } else {
                max_rit = 0;
                while max_rit != n_maxima && self.m_maxima[n_maxima - 1 - max_rit] > self.e(horz_edge).bot.x {
                    max_rit += 1;
                }
                if max_rit != n_maxima && self.m_maxima[n_maxima - 1 - max_rit] <= self.e(e_last_horz).top.x {
                    max_rit = n_maxima;
                }
            }
        }

        let mut op1: Option<usize> = None;

        loop {
            // loop through consec. horizontal edges

            let is_last_horz = horz_edge == e_last_horz;
            let mut e = if dir == Direction::dLeftToRight {
                self.e(horz_edge).next_in_ael
            } else {
                self.e(horz_edge).prev_in_ael
            };
            while let Some(ei) = e {
                // this code block inserts extra coords into horizontal edges (in output
                // polygons) whereever maxima touch these horizontal edges. This helps
                // 'simplifying' polygons (ie if the Simplify property is set).
                if !self.m_maxima.is_empty() {
                    if dir == Direction::dLeftToRight {
                        while max_it != n_maxima && self.m_maxima[max_it] < self.e(ei).curr.x {
                            if self.e(horz_edge).out_idx >= 0 && !is_open {
                                let p = IntPoint::new(self.m_maxima[max_it], self.e(horz_edge).bot.y);
                                self.add_out_pt(horz_edge, &p);
                            }
                            max_it += 1;
                        }
                    } else {
                        while max_rit != n_maxima && self.m_maxima[n_maxima - 1 - max_rit] > self.e(ei).curr.x {
                            if self.e(horz_edge).out_idx >= 0 && !is_open {
                                let p = IntPoint::new(self.m_maxima[n_maxima - 1 - max_rit], self.e(horz_edge).bot.y);
                                self.add_out_pt(horz_edge, &p);
                            }
                            max_rit += 1;
                        }
                    }
                }

                if (dir == Direction::dLeftToRight && self.e(ei).curr.x > horz_right)
                    || (dir == Direction::dRightToLeft && self.e(ei).curr.x < horz_left)
                {
                    break;
                }

                // Also break if we've got to the end of an intermediate horizontal edge ...
                // nb: Smaller Dx's are to the right of larger Dx's ABOVE the horizontal.
                if self.e(ei).curr.x == self.e(horz_edge).top.x {
                    if let Some(nl) = self.e(horz_edge).next_in_lml {
                        if self.e(ei).dx < self.e(nl).dx {
                            break;
                        }
                    }
                }

                if self.e(horz_edge).out_idx >= 0 && !is_open {
                    // note: may be done multiple times
                    let curr = self.e(ei).curr;
                    op1 = Some(self.add_out_pt(horz_edge, &curr));
                    let mut e_next_horz = self.m_sorted_edges;
                    while let Some(enh) = e_next_horz {
                        if self.e(enh).out_idx >= 0
                            && horz_segments_overlap(
                                self.e(horz_edge).bot.x,
                                self.e(horz_edge).top.x,
                                self.e(enh).bot.x,
                                self.e(enh).top.x,
                            )
                        {
                            let op2 = self.get_last_out_pt(enh);
                            let off_pt = self.e(enh).top;
                            self.m_joins.push(Join { out_pt1: Some(op2), out_pt2: op1, off_pt });
                        }
                        e_next_horz = self.e(enh).next_in_sel;
                    }
                    let off_pt = self.e(horz_edge).bot;
                    self.m_ghost_joins.push(Join { out_pt1: op1, out_pt2: None, off_pt });
                }

                // OK, so far we're still in range of the horizontal Edge  but make sure
                // we're at the last of consec. horizontals when matching with eMaxPair
                if e == e_max_pair && is_last_horz {
                    if self.e(horz_edge).out_idx >= 0 {
                        let top = self.e(horz_edge).top;
                        self.add_local_max_poly(horz_edge, ei, &top);
                    }
                    self.delete_from_ael(horz_edge);
                    self.delete_from_ael(ei);
                    return Ok(());
                }

                if dir == Direction::dLeftToRight {
                    let pt = IntPoint::new(self.e(ei).curr.x, self.e(horz_edge).curr.y);
                    self.intersect_edges(horz_edge, ei, &pt);
                } else {
                    let pt = IntPoint::new(self.e(ei).curr.x, self.e(horz_edge).curr.y);
                    self.intersect_edges(ei, horz_edge, &pt);
                }
                let e_next = if dir == Direction::dLeftToRight {
                    self.e(ei).next_in_ael
                } else {
                    self.e(ei).prev_in_ael
                };
                self.swap_positions_in_ael(horz_edge, ei);
                e = e_next;
            } // end while(e)

            // Break out of loop if HorzEdge.NextInLML is not also horizontal ...
            match self.e(horz_edge).next_in_lml {
                None => break,
                Some(nl) if !is_horizontal(self.e(nl)) => break,
                _ => {}
            }

            self.update_edge_into_ael(&mut horz_edge)?;
            if self.e(horz_edge).out_idx >= 0 {
                let bot = self.e(horz_edge).bot;
                self.add_out_pt(horz_edge, &bot);
            }
            (dir, horz_left, horz_right) = get_horz_direction(self.e(horz_edge));
        } // end for (;;)

        if self.e(horz_edge).out_idx >= 0 && op1.is_none() {
            op1 = Some(self.get_last_out_pt(horz_edge));
            let mut e_next_horz = self.m_sorted_edges;
            while let Some(enh) = e_next_horz {
                if self.e(enh).out_idx >= 0
                    && horz_segments_overlap(
                        self.e(horz_edge).bot.x,
                        self.e(horz_edge).top.x,
                        self.e(enh).bot.x,
                        self.e(enh).top.x,
                    )
                {
                    let op2 = self.get_last_out_pt(enh);
                    let off_pt = self.e(enh).top;
                    self.m_joins.push(Join { out_pt1: Some(op2), out_pt2: op1, off_pt });
                }
                e_next_horz = self.e(enh).next_in_sel;
            }
            let off_pt = self.e(horz_edge).top;
            self.m_ghost_joins.push(Join { out_pt1: op1, out_pt2: None, off_pt });
        }

        if self.e(horz_edge).next_in_lml.is_some() {
            if self.e(horz_edge).out_idx >= 0 {
                let top = self.e(horz_edge).top;
                op1 = Some(self.add_out_pt(horz_edge, &top));
                self.update_edge_into_ael(&mut horz_edge)?;
                if self.e(horz_edge).wind_delta == 0 {
                    return Ok(());
                }
                // nb: HorzEdge is no longer horizontal here
                let e_prev = self.e(horz_edge).prev_in_ael;
                let e_next = self.e(horz_edge).next_in_ael;
                let h = *self.e(horz_edge);
                let prev_joins = match e_prev {
                    Some(p) => {
                        let p = self.e(p);
                        p.curr.x == h.bot.x
                            && p.curr.y == h.bot.y
                            && p.wind_delta != 0
                            && (p.out_idx >= 0 && p.curr.y > p.top.y && slopes_equal_edges(&h, p))
                    }
                    None => false,
                };
                let next_joins = match e_next {
                    Some(n) => {
                        let n = self.e(n);
                        n.curr.x == h.bot.x
                            && n.curr.y == h.bot.y
                            && n.wind_delta != 0
                            && n.out_idx >= 0
                            && n.curr.y > n.top.y
                            && slopes_equal_edges(&h, n)
                    }
                    None => false,
                };
                if prev_joins {
                    let op2 = self.add_out_pt(e_prev.expect("tested above"), &h.bot);
                    self.m_joins.push(Join { out_pt1: op1, out_pt2: Some(op2), off_pt: h.top });
                } else if next_joins {
                    let op2 = self.add_out_pt(e_next.expect("tested above"), &h.bot);
                    self.m_joins.push(Join { out_pt1: op1, out_pt2: Some(op2), off_pt: h.top });
                }
            } else {
                self.update_edge_into_ael(&mut horz_edge)?;
            }
        } else {
            if self.e(horz_edge).out_idx >= 0 {
                let top = self.e(horz_edge).top;
                self.add_out_pt(horz_edge, &top);
            }
            self.delete_from_ael(horz_edge);
        }
        Ok(())
    }

    /// clipper.cpp:2400-2421
    /// C++: void Clipper::UpdateEdgeIntoAEL(TEdge *&e)
    pub fn update_edge_into_ael(&mut self, e: &mut usize) -> Result<(), ClipperException> {
        let Some(next_in_lml) = self.e(*e).next_in_lml else {
            return Err(ClipperException("UpdateEdgeIntoAEL: invalid call"));
        };

        self.em(next_in_lml).out_idx = self.e(*e).out_idx;
        let ael_prev = self.e(*e).prev_in_ael;
        let ael_next = self.e(*e).next_in_ael;
        if let Some(p) = ael_prev {
            self.em(p).next_in_ael = Some(next_in_lml);
        } else {
            self.m_active_edges = Some(next_in_lml);
        }
        if let Some(n) = ael_next {
            self.em(n).prev_in_ael = Some(next_in_lml);
        }
        self.em(next_in_lml).side = self.e(*e).side;
        self.em(next_in_lml).wind_delta = self.e(*e).wind_delta;
        self.em(next_in_lml).wind_cnt = self.e(*e).wind_cnt;
        self.em(next_in_lml).wind_cnt2 = self.e(*e).wind_cnt2;
        *e = next_in_lml;
        let ed = self.em(*e);
        ed.curr = ed.bot;
        ed.prev_in_ael = ael_prev;
        ed.next_in_ael = ael_next;
        if !is_horizontal(self.e(*e)) {
            self.m_scanbeam.push(self.e(*e).top.y);
        }
        Ok(())
    }

    /// clipper.cpp:2424-2449
    /// C++: bool Clipper::ProcessIntersections(const cInt topY)
    pub fn process_intersections(&mut self, top_y: cInt) -> bool {
        if self.m_active_edges.is_none() {
            return true;
        }
        // try { ... } catch(...) { m_SortedEdges = 0; m_IntersectList.clear();
        // throw clipperException("ProcessIntersections error"); }
        // Nothing the block calls throws a clipperException (BuildIntersectList,
        // FixupIntersectionOrder, IntersectEdges, SwapPositionsInAEL and what they
        // call), so the catch arm has nothing to catch.
        self.build_intersect_list(top_y);
        let il_size = self.m_intersect_list.len();
        if il_size == 0 {
            return true;
        }
        if il_size == 1 || self.fixup_intersection_order() {
            for i in 0..self.m_intersect_list.len() {
                let i_node = self.m_intersect_list[i];
                self.intersect_edges(i_node.edge1, i_node.edge2, &i_node.pt);
                self.swap_positions_in_ael(i_node.edge1, i_node.edge2);
            }
            self.m_intersect_list.clear();
        } else {
            return false;
        }
        self.m_sorted_edges = None;
        true
    }

    /// clipper.cpp:2452-2492
    /// C++: void Clipper::BuildIntersectList(const cInt topY)
    pub fn build_intersect_list(&mut self, top_y: cInt) {
        if self.m_active_edges.is_none() {
            return;
        }

        // prepare for sorting ...
        let mut e = self.m_active_edges;
        self.m_sorted_edges = e;
        while let Some(ei) = e {
            let ed = self.em(ei);
            ed.prev_in_sel = ed.prev_in_ael;
            ed.next_in_sel = ed.next_in_ael;
            ed.curr.x = top_x(ed, top_y);
            e = ed.next_in_ael;
        }

        // bubblesort ...
        loop {
            let mut is_modified = false;
            let mut e = self.m_sorted_edges.expect("the SEL holds the AEL");
            while let Some(e_next) = self.e(e).next_in_sel {
                if self.e(e).curr.x > self.e(e_next).curr.x {
                    let mut pt = IntPoint::default();
                    intersect_point(self.e(e), self.e(e_next), &mut pt);
                    self.m_intersect_list.push(IntersectNode { edge1: e, edge2: e_next, pt });
                    self.swap_positions_in_sel(e, e_next);
                    is_modified = true;
                } else {
                    e = e_next;
                }
            }
            if let Some(p) = self.e(e).prev_in_sel {
                self.em(p).next_in_sel = None;
            } else {
                break;
            }
            if !is_modified {
                break;
            }
        }
        self.m_sorted_edges = None; // important
    }

    /// clipper.cpp:2503-2524
    /// C++: bool Clipper::FixupIntersectionOrder()
    pub fn fixup_intersection_order(&mut self) -> bool {
        // pre-condition: intersections are sorted Bottom-most first.
        // Now it's crucial that intersections are made only between adjacent edges,
        // so to ensure this the order of intersections may need adjusting ...
        self.copy_ael_to_sel();
        // std::sort(m_IntersectList.begin(), m_IntersectList.end(), [](node1, node2) { return node2.Pt.y() < node1.Pt.y(); });
        // Not a stable sort: the order of intersections at one Y is libc++'s.
        libcxx_sort(&mut self.m_intersect_list, |node1, node2| node2.pt.y < node1.pt.y);

        let cnt = self.m_intersect_list.len();
        for i in 0..cnt {
            if !edges_adjacent(&self.base.m_edges, &self.m_intersect_list[i]) {
                let mut j = i + 1;
                while j < cnt && !edges_adjacent(&self.base.m_edges, &self.m_intersect_list[j]) {
                    j += 1;
                }
                if j == cnt {
                    return false;
                }
                self.m_intersect_list.swap(i, j);
            }
            let node = self.m_intersect_list[i];
            self.swap_positions_in_sel(node.edge1, node.edge2);
        }
        true
    }

    /// clipper.cpp:2527-2576
    /// C++: void Clipper::DoMaxima(TEdge *e)
    pub fn do_maxima(&mut self, e: usize) -> Result<(), ClipperException> {
        let Some(e_max_pair) = get_maxima_pair(&self.base.m_edges, e) else {
            if self.e(e).out_idx >= 0 {
                let top = self.e(e).top;
                self.add_out_pt(e, &top);
            }
            self.delete_from_ael(e);
            return Ok(());
        };

        let mut e_next = self.e(e).next_in_ael;
        while let Some(en) = e_next {
            if en == e_max_pair {
                break;
            }
            let top = self.e(e).top;
            self.intersect_edges(e, en, &top);
            self.swap_positions_in_ael(e, en);
            e_next = self.e(e).next_in_ael;
        }

        if self.e(e).out_idx == UNASSIGNED && self.e(e_max_pair).out_idx == UNASSIGNED {
            self.delete_from_ael(e);
            self.delete_from_ael(e_max_pair);
        } else if self.e(e).out_idx >= 0 && self.e(e_max_pair).out_idx >= 0 {
            if self.e(e).out_idx >= 0 {
                let top = self.e(e).top;
                self.add_local_max_poly(e, e_max_pair, &top);
            }
            self.delete_from_ael(e);
            self.delete_from_ael(e_max_pair);
        }
        // `use_lines` is defined (clipper.hpp:48).
        else if self.e(e).wind_delta == 0 {
            if self.e(e).out_idx >= 0 {
                let top = self.e(e).top;
                self.add_out_pt(e, &top);
                self.em(e).out_idx = UNASSIGNED;
            }
            self.delete_from_ael(e);

            if self.e(e_max_pair).out_idx >= 0 {
                let top = self.e(e).top;
                self.add_out_pt(e_max_pair, &top);
                self.em(e_max_pair).out_idx = UNASSIGNED;
            }
            self.delete_from_ael(e_max_pair);
        } else {
            return Err(ClipperException("DoMaxima error"));
        }
        Ok(())
    }

    /// clipper.cpp:2579-2681
    /// C++: void Clipper::ProcessEdgesAtTopOfScanbeam(const cInt topY)
    pub fn process_edges_at_top_of_scanbeam(&mut self, top_y: cInt) -> Result<(), ClipperException> {
        let mut e = self.m_active_edges;
        while let Some(mut ei) = e {
            // 1. process maxima, treating them as if they're 'bent' horizontal edges,
            //    but exclude maxima with horizontal edges. nb: e can't be a horizontal.
            let mut is_maxima_edge = is_maxima(&self.base.m_edges, Some(ei), top_y);

            if is_maxima_edge {
                let e_max_pair = get_maxima_pair(&self.base.m_edges, ei);
                is_maxima_edge = match e_max_pair {
                    None => true,
                    Some(mp) => !is_horizontal(self.e(mp)),
                };
            }

            if is_maxima_edge {
                if self.m_strict_simple {
                    self.m_maxima.push(self.e(ei).top.x);
                }
                let e_prev = self.e(ei).prev_in_ael;
                self.do_maxima(ei)?;
                match e_prev {
                    None => e = self.m_active_edges,
                    Some(p) => e = self.e(p).next_in_ael,
                }
            } else {
                // 2. promote horizontal edges, otherwise update Curr.x() and Curr.y() ...
                if is_intermediate(&self.base.m_edges, ei, top_y)
                    && is_horizontal(self.e(self.e(ei).next_in_lml.expect("IsIntermediate")))
                {
                    self.update_edge_into_ael(&mut ei)?;
                    if self.e(ei).out_idx >= 0 {
                        let bot = self.e(ei).bot;
                        self.add_out_pt(ei, &bot);
                    }
                    self.add_edge_to_sel(ei);
                } else {
                    let x = top_x(self.e(ei), top_y);
                    self.em(ei).curr.x = x;
                    self.em(ei).curr.y = top_y;
                }

                // When StrictlySimple and 'e' is being touched by another edge, then
                // make sure both edges have a vertex here ...
                if self.m_strict_simple {
                    if let Some(e_prev) = self.e(ei).prev_in_ael {
                        if self.e(ei).out_idx >= 0
                            && self.e(ei).wind_delta != 0
                            && self.e(e_prev).out_idx >= 0
                            && self.e(e_prev).curr.x == self.e(ei).curr.x
                            && self.e(e_prev).wind_delta != 0
                        {
                            let pt = self.e(ei).curr;
                            let op = self.add_out_pt(e_prev, &pt);
                            let op2 = self.add_out_pt(ei, &pt);
                            self.m_joins.push(Join { out_pt1: Some(op), out_pt2: Some(op2), off_pt: pt }); // StrictlySimple (type-3) join
                        }
                    }
                }

                e = self.e(ei).next_in_ael;
            }
        }

        // 3. Process horizontals at the Top of the scanbeam ...
        // std::sort(m_Maxima.begin(), m_Maxima.end()): plain integers, equal ones are
        // indistinguishable.
        self.m_maxima.sort_unstable();
        self.process_horizontals()?;
        self.m_maxima.clear();

        // 4. Promote intermediate vertices ...
        e = self.m_active_edges;
        while let Some(mut ei) = e {
            if is_intermediate(&self.base.m_edges, ei, top_y) {
                let mut op: Option<usize> = None;
                if self.e(ei).out_idx >= 0 {
                    let top = self.e(ei).top;
                    op = Some(self.add_out_pt(ei, &top));
                }
                self.update_edge_into_ael(&mut ei)?;

                // if output polygons share an edge, they'll need joining later ...
                let e_prev = self.e(ei).prev_in_ael;
                let e_next = self.e(ei).next_in_ael;
                let ed = *self.e(ei);
                let prev_joins = match e_prev {
                    Some(p) => {
                        let p = self.e(p);
                        p.curr.x == ed.bot.x
                            && p.curr.y == ed.bot.y
                            && op.is_some()
                            && p.out_idx >= 0
                            && p.curr.y > p.top.y
                            && slopes_equal_edges(&ed, p)
                            && ed.wind_delta != 0
                            && p.wind_delta != 0
                    }
                    None => false,
                };
                let next_joins = match e_next {
                    Some(n) => {
                        let n = self.e(n);
                        n.curr.x == ed.bot.x
                            && n.curr.y == ed.bot.y
                            && op.is_some()
                            && n.out_idx >= 0
                            && n.curr.y > n.top.y
                            && slopes_equal_edges(&ed, n)
                            && ed.wind_delta != 0
                            && n.wind_delta != 0
                    }
                    None => false,
                };
                if prev_joins {
                    let op2 = self.add_out_pt(e_prev.expect("tested above"), &ed.bot);
                    self.m_joins.push(Join { out_pt1: op, out_pt2: Some(op2), off_pt: ed.top });
                } else if next_joins {
                    let op2 = self.add_out_pt(e_next.expect("tested above"), &ed.bot);
                    self.m_joins.push(Join { out_pt1: op, out_pt2: Some(op2), off_pt: ed.top });
                }
            }
            e = self.e(ei).next_in_ael;
        }
        Ok(())
    }

    /// clipper.cpp:2684-2708
    /// C++: void Clipper::FixupOutPolyline(OutRec &outrec)
    pub fn fixup_out_polyline(&mut self, outrec: usize) {
        let mut pp = self.m_poly_outs[outrec].pts.expect("FixupOutPolyline reads outrec.Pts");
        let mut last_pp = self.pv(pp);
        while pp != last_pp {
            pp = self.nx(pp);
            if self.pt(pp) == self.pt(self.pv(pp)) {
                if pp == last_pp {
                    last_pp = self.pv(pp);
                }
                let tmp_pp = self.pv(pp);
                let pp_next = self.nx(pp);
                self.m_out_pts[tmp_pp].next = pp_next;
                self.m_out_pts[pp_next].prev = tmp_pp;
                self.dispose_out_pt(pp);
                pp = tmp_pp;
            }
        }

        if pp == self.pv(pp) {
            self.dispose_out_pts(Some(pp));
            self.m_poly_outs[outrec].pts = None;
        }
    }

    /// clipper.cpp:2711-2750
    /// C++: void Clipper::FixupOutPolygon(OutRec &outrec)
    pub fn fixup_out_polygon(&mut self, outrec: usize) {
        // FixupOutPolygon() - removes duplicate points and simplifies consecutive
        // parallel edges by removing the middle vertex.
        let mut last_ok: Option<usize> = None;
        self.m_poly_outs[outrec].bottom_pt = None;
        let mut pp = self.m_poly_outs[outrec].pts.expect("FixupOutPolygon reads outrec.Pts");
        let preserve_col = self.base.m_preserve_collinear || self.m_strict_simple;

        loop {
            if self.pv(pp) == pp || self.pv(pp) == self.nx(pp) {
                // Empty loop or a stick. Release the polygon.
                self.dispose_out_pts(Some(pp));
                self.m_poly_outs[outrec].pts = None;
                return;
            }

            // test for duplicate points and collinear edges ...
            let (prev_pt, pp_pt, next_pt) = (self.pt(self.pv(pp)), self.pt(pp), self.pt(self.nx(pp)));
            if pp_pt == next_pt
                || pp_pt == prev_pt
                || (slopes_equal_3(&prev_pt, &pp_pt, &next_pt)
                    && (!preserve_col || !pt2_is_between_pt1_and_pt3(&prev_pt, &pp_pt, &next_pt)))
            {
                last_ok = None;
                let tmp = pp;
                let (pp_prev, pp_next) = (self.pv(pp), self.nx(pp));
                self.m_out_pts[pp_prev].next = pp_next;
                self.m_out_pts[pp_next].prev = pp_prev;
                pp = pp_prev;
                self.dispose_out_pt(tmp);
            } else if Some(pp) == last_ok {
                break;
            } else {
                if last_ok.is_none() {
                    last_ok = Some(pp);
                }
                pp = self.nx(pp);
            }
        }
        self.m_poly_outs[outrec].pts = Some(pp);
    }

    /// clipper.cpp:2769-2788
    /// C++: void Clipper::BuildResult(Paths &polys)
    pub fn build_result(&self, polys: &mut Paths) {
        polys.reserve(self.m_poly_outs.len());
        for out_rec in &self.m_poly_outs {
            debug_assert!(!out_rec.is_open);
            let Some(pts) = out_rec.pts else { continue };
            let mut pg = Path::new();
            let mut p = self.pv(pts);
            let cnt = point_count(&self.m_out_pts, Some(p));
            if cnt < 2 {
                continue;
            }
            pg.reserve(cnt as usize);
            for _ in 0..cnt {
                pg.push(self.pt(p));
                p = self.pv(p);
            }
            polys.push(pg);
        }
    }

    /// clipper.cpp:2888-2914
    /// C++: void Clipper::InsertEdgeIntoAEL(TEdge *edge, TEdge* startEdge)
    pub fn insert_edge_into_ael(&mut self, edge: usize, mut start_edge: Option<usize>) {
        let Some(active) = self.m_active_edges else {
            self.em(edge).prev_in_ael = None;
            self.em(edge).next_in_ael = None;
            self.m_active_edges = Some(edge);
            return;
        };
        if start_edge.is_none() && e2_inserts_before_e1(self.e(active), self.e(edge)) {
            self.em(edge).prev_in_ael = None;
            self.em(edge).next_in_ael = Some(active);
            self.em(active).prev_in_ael = Some(edge);
            self.m_active_edges = Some(edge);
        } else {
            if start_edge.is_none() {
                start_edge = Some(active);
            }
            let mut start_edge = start_edge.expect("set above");
            while let Some(n) = self.e(start_edge).next_in_ael {
                if e2_inserts_before_e1(self.e(n), self.e(edge)) {
                    break;
                }
                start_edge = n;
            }
            let start_next = self.e(start_edge).next_in_ael;
            self.em(edge).next_in_ael = start_next;
            if let Some(n) = start_next {
                self.em(n).prev_in_ael = Some(edge);
            }
            self.em(edge).prev_in_ael = Some(start_edge);
            self.em(start_edge).next_in_ael = Some(edge);
        }
    }

    /// clipper.cpp:2917-2937
    /// C++: OutPt* Clipper::DupOutPt(OutPt* outPt, bool InsertAfter)
    pub fn dup_out_pt(&mut self, out_pt: usize, insert_after: bool) -> usize {
        let result = self.allocate_out_pt();
        self.m_out_pts[result].pt = self.m_out_pts[out_pt].pt;
        self.m_out_pts[result].idx = self.m_out_pts[out_pt].idx;
        if insert_after {
            let out_pt_next = self.nx(out_pt);
            self.m_out_pts[result].next = out_pt_next;
            self.m_out_pts[result].prev = out_pt;
            self.m_out_pts[out_pt_next].prev = result;
            self.m_out_pts[out_pt].next = result;
        } else {
            let out_pt_prev = self.pv(out_pt);
            self.m_out_pts[result].prev = out_pt_prev;
            self.m_out_pts[result].next = out_pt;
            self.m_out_pts[out_pt_prev].next = result;
            self.m_out_pts[out_pt].prev = result;
        }
        result
    }

    /// clipper.cpp:2940-3024
    /// C++: bool Clipper::JoinHorz(OutPt* op1, OutPt* op1b, OutPt* op2, OutPt* op2b, const IntPoint &Pt, bool DiscardLeft)
    pub fn join_horz(
        &mut self,
        mut op1: usize,
        mut op1b: usize,
        mut op2: usize,
        mut op2b: usize,
        pt: &IntPoint,
        discard_left: bool,
    ) -> bool {
        let dir1 = if self.pt(op1).x > self.pt(op1b).x { Direction::dRightToLeft } else { Direction::dLeftToRight };
        let dir2 = if self.pt(op2).x > self.pt(op2b).x { Direction::dRightToLeft } else { Direction::dLeftToRight };
        if dir1 == dir2 {
            return false;
        }

        // When DiscardLeft, we want Op1b to be on the Left of Op1, otherwise we
        // want Op1b to be on the Right. (And likewise with Op2 and Op2b.)
        // So, to facilitate this while inserting Op1b and Op2b ...
        // when DiscardLeft, make sure we're AT or RIGHT of Pt before adding Op1b,
        // otherwise make sure we're AT or LEFT of Pt. (Likewise with Op2b.)
        if dir1 == Direction::dLeftToRight {
            while self.pt(self.nx(op1)).x <= pt.x
                && self.pt(self.nx(op1)).x >= self.pt(op1).x
                && self.pt(self.nx(op1)).y == pt.y
            {
                op1 = self.nx(op1);
            }
            if discard_left && self.pt(op1).x != pt.x {
                op1 = self.nx(op1);
            }
            op1b = self.dup_out_pt(op1, !discard_left);
            if self.pt(op1b) != *pt {
                op1 = op1b;
                self.m_out_pts[op1].pt = *pt;
                op1b = self.dup_out_pt(op1, !discard_left);
            }
        } else {
            while self.pt(self.nx(op1)).x >= pt.x
                && self.pt(self.nx(op1)).x <= self.pt(op1).x
                && self.pt(self.nx(op1)).y == pt.y
            {
                op1 = self.nx(op1);
            }
            if !discard_left && self.pt(op1).x != pt.x {
                op1 = self.nx(op1);
            }
            op1b = self.dup_out_pt(op1, discard_left);
            if self.pt(op1b) != *pt {
                op1 = op1b;
                self.m_out_pts[op1].pt = *pt;
                op1b = self.dup_out_pt(op1, discard_left);
            }
        }

        if dir2 == Direction::dLeftToRight {
            while self.pt(self.nx(op2)).x <= pt.x
                && self.pt(self.nx(op2)).x >= self.pt(op2).x
                && self.pt(self.nx(op2)).y == pt.y
            {
                op2 = self.nx(op2);
            }
            if discard_left && self.pt(op2).x != pt.x {
                op2 = self.nx(op2);
            }
            op2b = self.dup_out_pt(op2, !discard_left);
            if self.pt(op2b) != *pt {
                op2 = op2b;
                self.m_out_pts[op2].pt = *pt;
                op2b = self.dup_out_pt(op2, !discard_left);
            }
        } else {
            while self.pt(self.nx(op2)).x >= pt.x
                && self.pt(self.nx(op2)).x <= self.pt(op2).x
                && self.pt(self.nx(op2)).y == pt.y
            {
                op2 = self.nx(op2);
            }
            if !discard_left && self.pt(op2).x != pt.x {
                op2 = self.nx(op2);
            }
            op2b = self.dup_out_pt(op2, discard_left);
            if self.pt(op2b) != *pt {
                op2 = op2b;
                self.m_out_pts[op2].pt = *pt;
                op2b = self.dup_out_pt(op2, discard_left);
            }
        }

        if (dir1 == Direction::dLeftToRight) == discard_left {
            self.m_out_pts[op1].prev = op2;
            self.m_out_pts[op2].next = op1;
            self.m_out_pts[op1b].next = op2b;
            self.m_out_pts[op2b].prev = op1b;
        } else {
            self.m_out_pts[op1].next = op2;
            self.m_out_pts[op2].prev = op1;
            self.m_out_pts[op1b].prev = op2b;
            self.m_out_pts[op2b].next = op1b;
        }
        true
    }

    /// clipper.cpp:3027-3183
    /// C++: bool Clipper::JoinPoints(Join *j, OutRec* outRec1, OutRec* outRec2)
    /// `j` is the index of the join in `m_joins`.
    pub fn join_points(&mut self, j: usize, out_rec1: usize, out_rec2: usize) -> bool {
        let j_out_pt1 = self.m_joins[j].out_pt1.expect("JoinPoints reads j->OutPt1");
        let j_out_pt2 = self.m_joins[j].out_pt2.expect("JoinPoints reads j->OutPt2");
        let off_pt = self.m_joins[j].off_pt;
        let mut op1 = j_out_pt1;
        let mut op1b;
        let mut op2 = j_out_pt2;
        let mut op2b;

        // There are 3 kinds of joins for output polygons ...
        // 1. Horizontal joins where Join.OutPt1 & Join.OutPt2 are vertices anywhere
        // along (horizontal) collinear edges (& Join.OffPt is on the same horizontal).
        // 2. Non-horizontal joins where Join.OutPt1 & Join.OutPt2 are at the same
        // location at the Bottom of the overlapping segment (& Join.OffPt is above).
        // 3. StrictSimple joins where edges touch but are not collinear and where
        // Join.OutPt1, Join.OutPt2 & Join.OffPt all share the same point.
        let is_horizontal = self.pt(j_out_pt1).y == off_pt.y;

        if is_horizontal && off_pt == self.pt(j_out_pt1) && off_pt == self.pt(j_out_pt2) {
            // Strictly Simple join ...
            if out_rec1 != out_rec2 {
                return false;
            }
            op1b = self.nx(j_out_pt1);
            while op1b != op1 && self.pt(op1b) == off_pt {
                op1b = self.nx(op1b);
            }
            let reverse1 = self.pt(op1b).y > off_pt.y;
            op2b = self.nx(j_out_pt2);
            while op2b != op2 && self.pt(op2b) == off_pt {
                op2b = self.nx(op2b);
            }
            let reverse2 = self.pt(op2b).y > off_pt.y;
            if reverse1 == reverse2 {
                return false;
            }
            if reverse1 {
                op1b = self.dup_out_pt(op1, false);
                op2b = self.dup_out_pt(op2, true);
                self.m_out_pts[op1].prev = op2;
                self.m_out_pts[op2].next = op1;
                self.m_out_pts[op1b].next = op2b;
                self.m_out_pts[op2b].prev = op1b;
                self.m_joins[j].out_pt1 = Some(op1);
                self.m_joins[j].out_pt2 = Some(op1b);
                true
            } else {
                op1b = self.dup_out_pt(op1, true);
                op2b = self.dup_out_pt(op2, false);
                self.m_out_pts[op1].next = op2;
                self.m_out_pts[op2].prev = op1;
                self.m_out_pts[op1b].prev = op2b;
                self.m_out_pts[op2b].next = op1b;
                self.m_joins[j].out_pt1 = Some(op1);
                self.m_joins[j].out_pt2 = Some(op1b);
                true
            }
        } else if is_horizontal {
            // treat horizontal joins differently to non-horizontal joins since with
            // them we're not yet sure where the overlapping is. OutPt1.Pt & OutPt2.Pt
            // may be anywhere along the horizontal edge.
            op1b = op1;
            while self.pt(self.pv(op1)).y == self.pt(op1).y && self.pv(op1) != op1b && self.pv(op1) != op2 {
                op1 = self.pv(op1);
            }
            while self.pt(self.nx(op1b)).y == self.pt(op1b).y && self.nx(op1b) != op1 && self.nx(op1b) != op2 {
                op1b = self.nx(op1b);
            }
            if self.nx(op1b) == op1 || self.nx(op1b) == op2 {
                return false; // a flat 'polygon'
            }

            op2b = op2;
            while self.pt(self.pv(op2)).y == self.pt(op2).y && self.pv(op2) != op2b && self.pv(op2) != op1b {
                op2 = self.pv(op2);
            }
            while self.pt(self.nx(op2b)).y == self.pt(op2b).y && self.nx(op2b) != op2 && self.nx(op2b) != op1 {
                op2b = self.nx(op2b);
            }
            if self.nx(op2b) == op2 || self.nx(op2b) == op1 {
                return false; // a flat 'polygon'
            }

            let (mut left, mut right): (cInt, cInt) = (0, 0);
            // Op1 --> Op1b & Op2 --> Op2b are the extremites of the horizontal edges
            if !get_overlap(self.pt(op1).x, self.pt(op1b).x, self.pt(op2).x, self.pt(op2b).x, &mut left, &mut right) {
                return false;
            }

            // DiscardLeftSide: when overlapping edges are joined, a spike will created
            // which needs to be cleaned up. However, we don't want Op1 or Op2 caught up
            // on the discard Side as either may still be needed for other joins ...
            let pt;
            let discard_left_side;
            if self.pt(op1).x >= left && self.pt(op1).x <= right {
                pt = self.pt(op1);
                discard_left_side = self.pt(op1).x > self.pt(op1b).x;
            } else if self.pt(op2).x >= left && self.pt(op2).x <= right {
                pt = self.pt(op2);
                discard_left_side = self.pt(op2).x > self.pt(op2b).x;
            } else if self.pt(op1b).x >= left && self.pt(op1b).x <= right {
                pt = self.pt(op1b);
                discard_left_side = self.pt(op1b).x > self.pt(op1).x;
            } else {
                pt = self.pt(op2b);
                discard_left_side = self.pt(op2b).x > self.pt(op2).x;
            }
            self.m_joins[j].out_pt1 = Some(op1);
            self.m_joins[j].out_pt2 = Some(op2);
            self.join_horz(op1, op1b, op2, op2b, &pt, discard_left_side)
        } else {
            // nb: For non-horizontal joins ...
            //    1. Jr.OutPt1.Pt.y() == Jr.OutPt2.Pt.y()
            //    2. Jr.OutPt1.Pt > Jr.OffPt.y()

            // make sure the polygons are correctly oriented ...
            op1b = self.nx(op1);
            while self.pt(op1b) == self.pt(op1) && op1b != op1 {
                op1b = self.nx(op1b);
            }
            let reverse1 = self.pt(op1b).y > self.pt(op1).y || !slopes_equal_3(&self.pt(op1), &self.pt(op1b), &off_pt);
            if reverse1 {
                op1b = self.pv(op1);
                while self.pt(op1b) == self.pt(op1) && op1b != op1 {
                    op1b = self.pv(op1b);
                }
                if self.pt(op1b).y > self.pt(op1).y || !slopes_equal_3(&self.pt(op1), &self.pt(op1b), &off_pt) {
                    return false;
                }
            }
            op2b = self.nx(op2);
            while self.pt(op2b) == self.pt(op2) && op2b != op2 {
                op2b = self.nx(op2b);
            }
            let reverse2 = self.pt(op2b).y > self.pt(op2).y || !slopes_equal_3(&self.pt(op2), &self.pt(op2b), &off_pt);
            if reverse2 {
                op2b = self.pv(op2);
                while self.pt(op2b) == self.pt(op2) && op2b != op2 {
                    op2b = self.pv(op2b);
                }
                if self.pt(op2b).y > self.pt(op2).y || !slopes_equal_3(&self.pt(op2), &self.pt(op2b), &off_pt) {
                    return false;
                }
            }

            if op1b == op1 || op2b == op2 || op1b == op2b || (out_rec1 == out_rec2 && reverse1 == reverse2) {
                return false;
            }

            if reverse1 {
                op1b = self.dup_out_pt(op1, false);
                op2b = self.dup_out_pt(op2, true);
                self.m_out_pts[op1].prev = op2;
                self.m_out_pts[op2].next = op1;
                self.m_out_pts[op1b].next = op2b;
                self.m_out_pts[op2b].prev = op1b;
                self.m_joins[j].out_pt1 = Some(op1);
                self.m_joins[j].out_pt2 = Some(op1b);
                true
            } else {
                op1b = self.dup_out_pt(op1, true);
                op2b = self.dup_out_pt(op2, false);
                self.m_out_pts[op1].next = op2;
                self.m_out_pts[op2].prev = op1;
                self.m_out_pts[op1b].prev = op2b;
                self.m_out_pts[op2b].next = op1b;
                self.m_joins[j].out_pt1 = Some(op1);
                self.m_joins[j].out_pt2 = Some(op1b);
                true
            }
        }
    }

    /// clipper.cpp:3187-3200
    /// C++: void Clipper::FixupFirstLefts1(OutRec* OldOutRec, OutRec* NewOutRec) const
    /// This is potentially very expensive! O(n^3)!
    pub fn fixup_first_lefts1(&mut self, old_out_rec: usize, new_out_rec: usize) {
        // tests if NewOutRec contains the polygon before reassigning FirstLeft
        for i in 0..self.m_poly_outs.len() {
            let out_rec = self.m_poly_outs[i];
            let Some(pts) = out_rec.pts else { continue };
            if out_rec.first_left.is_none() {
                continue;
            }
            let mut first_left = out_rec.first_left;
            // Skip empty polygons.
            while let Some(fl) = first_left {
                if self.m_poly_outs[fl].pts.is_some() {
                    break;
                }
                first_left = self.m_poly_outs[fl].first_left;
            }
            if first_left == Some(old_out_rec)
                && poly2_contains_poly1(
                    &self.m_out_pts,
                    pts,
                    self.m_poly_outs[new_out_rec].pts.expect("FixupFirstLefts1 reads NewOutRec->Pts"),
                )
            {
                self.m_poly_outs[i].first_left = Some(new_out_rec);
            }
        }
    }

    /// clipper.cpp:3203-3208
    /// C++: void Clipper::FixupFirstLefts2(OutRec* OldOutRec, OutRec* NewOutRec) const
    pub fn fixup_first_lefts2(&mut self, old_out_rec: usize, new_out_rec: usize) {
        // reassigns FirstLeft WITHOUT testing if NewOutRec contains the polygon
        for out_rec in self.m_poly_outs.iter_mut() {
            if out_rec.first_left == Some(old_out_rec) {
                out_rec.first_left = Some(new_out_rec);
            }
        }
    }

    /// clipper.cpp:3211-3313
    /// C++: void Clipper::JoinCommonEdges()
    pub fn join_common_edges(&mut self) {
        for ji in 0..self.m_joins.len() {
            let join_out_pt1 = self.m_joins[ji].out_pt1.expect("JoinCommonEdges reads join.OutPt1");
            let join_out_pt2 = self.m_joins[ji].out_pt2.expect("JoinCommonEdges reads join.OutPt2");
            let out_rec1 = self.get_out_rec(self.m_out_pts[join_out_pt1].idx);
            let mut out_rec2 = self.get_out_rec(self.m_out_pts[join_out_pt2].idx);

            if self.m_poly_outs[out_rec1].pts.is_none() || self.m_poly_outs[out_rec2].pts.is_none() {
                continue;
            }
            if self.m_poly_outs[out_rec1].is_open || self.m_poly_outs[out_rec2].is_open {
                continue;
            }

            // get the polygon fragment with the correct hole state (FirstLeft)
            // before calling JoinPoints() ...
            let hole_state_rec;
            if out_rec1 == out_rec2 {
                hole_state_rec = out_rec1;
            } else if param1_right_of_param2(&self.m_poly_outs, out_rec1, out_rec2) {
                hole_state_rec = out_rec2;
            } else if param1_right_of_param2(&self.m_poly_outs, out_rec2, out_rec1) {
                hole_state_rec = out_rec1;
            } else {
                hole_state_rec = get_lowermost_rec(&mut self.m_poly_outs, &self.m_out_pts, out_rec1, out_rec2);
            }

            if !self.join_points(ji, out_rec1, out_rec2) {
                continue;
            }

            if out_rec1 == out_rec2 {
                // instead of joining two polygons, we've just created a new one by
                // splitting one polygon into two.
                self.m_poly_outs[out_rec1].pts = self.m_joins[ji].out_pt1;
                self.m_poly_outs[out_rec1].bottom_pt = None;
                out_rec2 = self.create_out_rec();
                self.m_poly_outs[out_rec2].pts = self.m_joins[ji].out_pt2;

                // update all OutRec2.Pts Idx's ...
                let rec2 = self.m_poly_outs[out_rec2];
                update_out_pt_idxs(&mut self.m_out_pts, &rec2);

                // We now need to check every OutRec.FirstLeft pointer. If it points
                // to OutRec1 it may need to point to OutRec2 instead ...
                if self.m_using_poly_tree {
                    for j in 0..self.m_poly_outs.len() - 1 {
                        let o_rec = self.m_poly_outs[j];
                        let mut first_left = o_rec.first_left;
                        while let Some(fl) = first_left {
                            if self.m_poly_outs[fl].pts.is_some() {
                                break;
                            }
                            first_left = self.m_poly_outs[fl].first_left;
                        }
                        if o_rec.pts.is_none()
                            || first_left != Some(out_rec1)
                            || o_rec.is_hole == self.m_poly_outs[out_rec1].is_hole
                        {
                            continue;
                        }
                        if poly2_contains_poly1(
                            &self.m_out_pts,
                            o_rec.pts.expect("tested above"),
                            self.m_joins[ji].out_pt2.expect("JoinPoints sets OutPt2"),
                        ) {
                            self.m_poly_outs[j].first_left = Some(out_rec2);
                        }
                    }
                }

                let pts1 = self.m_poly_outs[out_rec1].pts.expect("JoinPoints sets OutPt1");
                let pts2 = self.m_poly_outs[out_rec2].pts.expect("JoinPoints sets OutPt2");
                if poly2_contains_poly1(&self.m_out_pts, pts2, pts1) {
                    // outRec2 is contained by outRec1 ...
                    self.m_poly_outs[out_rec2].is_hole = !self.m_poly_outs[out_rec1].is_hole;
                    self.m_poly_outs[out_rec2].first_left = Some(out_rec1);

                    // For each m_PolyOuts, replace FirstLeft from outRec2 to outRec1.
                    if self.m_using_poly_tree {
                        self.fixup_first_lefts2(out_rec2, out_rec1);
                    }

                    let rec2 = self.m_poly_outs[out_rec2];
                    if (rec2.is_hole ^ self.m_reverse_output) == (area_out_rec(&self.m_out_pts, &rec2) > 0.0) {
                        reverse_poly_pt_links(&mut self.m_out_pts, rec2.pts);
                    }
                } else if poly2_contains_poly1(&self.m_out_pts, pts1, pts2) {
                    // outRec1 is contained by outRec2 ...
                    self.m_poly_outs[out_rec2].is_hole = self.m_poly_outs[out_rec1].is_hole;
                    self.m_poly_outs[out_rec1].is_hole = !self.m_poly_outs[out_rec2].is_hole;
                    self.m_poly_outs[out_rec2].first_left = self.m_poly_outs[out_rec1].first_left;
                    self.m_poly_outs[out_rec1].first_left = Some(out_rec2);

                    // For each m_PolyOuts, replace FirstLeft from outRec1 to outRec2.
                    if self.m_using_poly_tree {
                        self.fixup_first_lefts2(out_rec1, out_rec2);
                    }

                    let rec1 = self.m_poly_outs[out_rec1];
                    if (rec1.is_hole ^ self.m_reverse_output) == (area_out_rec(&self.m_out_pts, &rec1) > 0.0) {
                        reverse_poly_pt_links(&mut self.m_out_pts, rec1.pts);
                    }
                } else {
                    // the 2 polygons are completely separate ...
                    self.m_poly_outs[out_rec2].is_hole = self.m_poly_outs[out_rec1].is_hole;
                    self.m_poly_outs[out_rec2].first_left = self.m_poly_outs[out_rec1].first_left;

                    // fixup FirstLeft pointers that may need reassigning to OutRec2
                    // For each polygon of m_PolyOuts, replace FirstLeft from outRec1 to outRec2 if the polygon is inside outRec2.
                    if self.m_using_poly_tree {
                        self.fixup_first_lefts1(out_rec1, out_rec2);
                    }
                }
            } else {
                // joined 2 polygons together ...

                self.m_poly_outs[out_rec2].pts = None;
                self.m_poly_outs[out_rec2].bottom_pt = None;
                self.m_poly_outs[out_rec2].idx = self.m_poly_outs[out_rec1].idx;

                self.m_poly_outs[out_rec1].is_hole = self.m_poly_outs[hole_state_rec].is_hole;
                if hole_state_rec == out_rec2 {
                    self.m_poly_outs[out_rec1].first_left = self.m_poly_outs[out_rec2].first_left;
                }
                self.m_poly_outs[out_rec2].first_left = Some(out_rec1);

                // For each m_PolyOuts, replace FirstLeft from outRec2 to outRec1.
                if self.m_using_poly_tree {
                    self.fixup_first_lefts2(out_rec2, out_rec1);
                }
            }
        }
    }

    /// clipper.cpp:3762-3826
    /// C++: void Clipper::DoSimplePolygons()
    /// Called by Clipper::ExecuteInternal()
    /// For each polygon, search for exactly duplicate non-successive points.
    /// If such a point is found, the loop is split into two pieces.
    /// Search for the duplicate points is O(n^2)!
    pub fn do_simple_polygons(&mut self) {
        let mut i = 0usize;
        while i < self.m_poly_outs.len() {
            let outrec = i;
            i += 1;
            let Some(mut op) = self.m_poly_outs[outrec].pts else { continue };
            if self.m_poly_outs[outrec].is_open {
                continue;
            }
            loop {
                // for each Pt in Polygon until duplicate found do ...
                let mut op2 = self.nx(op);
                while Some(op2) != self.m_poly_outs[outrec].pts {
                    if self.pt(op) == self.pt(op2) && self.nx(op2) != op && self.pv(op2) != op {
                        // split the polygon into two ...
                        let op3 = self.pv(op);
                        let op4 = self.pv(op2);
                        self.m_out_pts[op].prev = op4;
                        self.m_out_pts[op4].next = op;
                        self.m_out_pts[op2].prev = op3;
                        self.m_out_pts[op3].next = op2;

                        self.m_poly_outs[outrec].pts = Some(op);
                        let outrec2 = self.create_out_rec();
                        self.m_poly_outs[outrec2].pts = Some(op2);
                        let rec2 = self.m_poly_outs[outrec2];
                        update_out_pt_idxs(&mut self.m_out_pts, &rec2);
                        if poly2_contains_poly1(&self.m_out_pts, op2, op) {
                            // OutRec2 is contained by OutRec1 ...
                            self.m_poly_outs[outrec2].is_hole = !self.m_poly_outs[outrec].is_hole;
                            self.m_poly_outs[outrec2].first_left = Some(outrec);
                            // For each m_PolyOuts, replace FirstLeft from outRec2 to outrec.
                            if self.m_using_poly_tree {
                                self.fixup_first_lefts2(outrec2, outrec);
                            }
                        } else if poly2_contains_poly1(&self.m_out_pts, op, op2) {
                            // OutRec1 is contained by OutRec2 ...
                            self.m_poly_outs[outrec2].is_hole = self.m_poly_outs[outrec].is_hole;
                            self.m_poly_outs[outrec].is_hole = !self.m_poly_outs[outrec2].is_hole;
                            self.m_poly_outs[outrec2].first_left = self.m_poly_outs[outrec].first_left;
                            self.m_poly_outs[outrec].first_left = Some(outrec2);
                            // For each m_PolyOuts, replace FirstLeft from outrec to outrec2.
                            if self.m_using_poly_tree {
                                self.fixup_first_lefts2(outrec, outrec2);
                            }
                        } else {
                            // the 2 polygons are separate ...
                            self.m_poly_outs[outrec2].is_hole = self.m_poly_outs[outrec].is_hole;
                            self.m_poly_outs[outrec2].first_left = self.m_poly_outs[outrec].first_left;
                            // For each polygon of m_PolyOuts, replace FirstLeft from outrec to outrec2 if the polygon is inside outRec2.
                            if self.m_using_poly_tree {
                                self.fixup_first_lefts1(outrec, outrec2);
                            }
                        }
                        op2 = op; // ie get ready for the Next iteration
                    }
                    op2 = self.nx(op2);
                }
                op = self.nx(op);
                if Some(op) == self.m_poly_outs[outrec].pts {
                    break;
                }
            }
        }
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

    #[test]
    fn libcxx_sort_keeps_equal_elements_in_place_below_24_only() {
        // Up to 23 elements libc++ sorts by insertion: stable.
        for n in 0..24usize {
            let mut v: Vec<(i32, usize)> = (0..n).map(|i| (((i * 7) % 3) as i32, i)).collect();
            let mut want = v.clone();
            want.sort_by_key(|e| e.0);
            libcxx_sort(&mut v, |a, b| a.0 < b.0);
            assert_eq!(v, want, "n = {n}");
        }
        // From 24 on it partitions. The order std::sort of libc++ (LLVM 17 and later)
        // gives the ids of 40 elements with the keys (i * 7) % 3:
        let mut v: Vec<(i32, i32)> = (0..40).map(|i| ((i * 7) % 3, i)).collect();
        libcxx_sort(&mut v, |a, b| a.0 < b.0);
        let ids: Vec<i32> = v.iter().map(|e| e.1).collect();
        assert_eq!(
            ids,
            [
                39, 3, 27, 9, 12, 15, 18, 0, 21, 24, 30, 33, 6, 36, 13, 16, 19, 22, 25, 37, 10, 7, 34, 4, 31, 1, 28, 8,
                26, 29, 23, 17, 32, 14, 11, 35, 5, 2, 38, 20
            ]
        );
        // And of 150 (past the ninther threshold), the first and the last ten.
        let mut v: Vec<(i32, i32)> = (0..150).map(|i| ((i * 7) % 3, i)).collect();
        libcxx_sort(&mut v, |a, b| a.0 < b.0);
        let ids: Vec<i32> = v.iter().map(|e| e.1).collect();
        assert_eq!(ids[..10], [51, 144, 147, 3, 141, 138, 6, 135, 132, 9]);
        assert_eq!(ids[140..], [140, 59, 47, 143, 56, 53, 146, 2, 74, 101]);
        assert!(v.windows(2).all(|w| w[0].0 <= w[1].0));
    }

    #[test]
    fn libcxx_sort_and_its_heap_sort_order_any_input() {
        let mut s: u64 = 0x9E37_79B9_7F4A_7C15;
        let mut xs = move || {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            s
        };
        for round in 0..400 {
            let n = (xs() % [7, 30, 200, 1500][round % 4]) as usize;
            let keys = 1 + xs() % [1, 3, 50, 100_000][(round / 4) % 4];
            let v: Vec<(u64, usize)> = (0..n).map(|i| (xs() % keys, i)).collect();
            let mut want = v.clone();
            want.sort();
            for descending in [false, true] {
                let mut a = v.clone();
                libcxx_sort(&mut a, |x, y| if descending { y.0 < x.0 } else { x.0 < y.0 });
                assert!(a.windows(2).all(|w| if descending { w[0].0 >= w[1].0 } else { w[0].0 <= w[1].0 }));
                a.sort();
                assert_eq!(a, want);
            }
            let mut b = v.clone();
            libcxx_heap_sort(&mut b, &mut |x: &(u64, usize), y: &(u64, usize)| x.0 < y.0);
            assert!(b.windows(2).all(|w| w[0].0 <= w[1].0));
            b.sort();
            assert_eq!(b, want);
        }
    }

    fn clip(
        subj: &[&[(cInt, cInt)]],
        clp: &[&[(cInt, cInt)]],
        clip_type: ClipType,
        fill: PolyFillType,
        init_options: i32,
    ) -> (bool, Paths) {
        let p = |v: &[(cInt, cInt)]| -> Path { v.iter().map(|q| IntPoint::new(q.0, q.1)).collect() };
        let mut c = Clipper::new(init_options);
        for s in subj {
            c.add_path(&p(s), PolyType::ptSubject, true);
        }
        for s in clp {
            c.add_path(&p(s), PolyType::ptClip, true);
        }
        let mut solution = vec![vec![IntPoint::new(7, 7)]];
        let ok = c.execute(clip_type, &mut solution, fill, fill);
        (ok, solution)
    }

    const SQUARE_A: &[(cInt, cInt)] = &[(0, 0), (10, 0), (10, 10), (0, 10)];
    const SQUARE_B: &[(cInt, cInt)] = &[(5, 5), (15, 5), (15, 15), (5, 15)];

    #[test]
    fn execute_clips_two_overlapping_squares() {
        let total = |paths: &Paths| -> f64 { paths.iter().map(area).sum() };
        let (ok, inter) = clip(&[SQUARE_A], &[SQUARE_B], ClipType::ctIntersection, PolyFillType::pftNonZero, 0);
        assert!(ok);
        assert_eq!(inter.len(), 1);
        let mut pts = inter[0].clone();
        pts.sort_by_key(|p| (p.x, p.y));
        assert_eq!(pts, [IntPoint::new(5, 5), IntPoint::new(5, 10), IntPoint::new(10, 5), IntPoint::new(10, 10)]);
        assert_eq!(area(&inter[0]), 25.0);

        let (_, union) = clip(&[SQUARE_A], &[SQUARE_B], ClipType::ctUnion, PolyFillType::pftNonZero, 0);
        assert_eq!((union.len(), union[0].len(), total(&union)), (1, 8, 175.0));
        let (_, diff) = clip(&[SQUARE_A], &[SQUARE_B], ClipType::ctDifference, PolyFillType::pftNonZero, 0);
        assert_eq!((diff.len(), diff[0].len(), total(&diff)), (1, 6, 75.0));
        let (_, xor) = clip(&[SQUARE_A], &[SQUARE_B], ClipType::ctXor, PolyFillType::pftNonZero, 0);
        assert_eq!((xor.len(), total(&xor)), (2, 150.0));

        // ReverseSolution turns the outer contour the other way.
        let (_, reversed) = clip(&[SQUARE_A], &[SQUARE_B], ClipType::ctUnion, PolyFillType::pftNonZero, IO_REVERSE_SOLUTION);
        assert_eq!(total(&reversed), -175.0);
    }

    #[test]
    fn execute_gives_a_hole_and_keeps_collinear_points_when_asked() {
        // A square with a square taken out of its middle: an outer contour and a hole.
        let inner: &[(cInt, cInt)] = &[(3, 3), (7, 3), (7, 7), (3, 7)];
        let (ok, ring) = clip(&[SQUARE_A], &[inner], ClipType::ctDifference, PolyFillType::pftEvenOdd, 0);
        assert!(ok);
        assert_eq!(ring.len(), 2);
        let mut areas: Vec<f64> = ring.iter().map(area).collect();
        areas.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert_eq!(areas, [-16.0, 100.0]);

        // The midpoint of an edge goes by default and stays with PreserveCollinear.
        let with_mid: &[(cInt, cInt)] = &[(0, 0), (5, 0), (10, 0), (10, 10), (0, 10)];
        let (_, plain) = clip(&[with_mid], &[], ClipType::ctUnion, PolyFillType::pftNonZero, 0);
        assert_eq!(plain[0].len(), 4);
        let (_, kept) = clip(&[with_mid], &[], ClipType::ctUnion, PolyFillType::pftNonZero, IO_PRESERVE_COLLINEAR);
        assert_eq!(kept[0].len(), 5);
    }

    #[test]
    fn execute_splits_a_touching_polygon_when_strictly_simple() {
        // Two squares that share the corner (10, 10), drawn as one path.
        let bow: &[(cInt, cInt)] = &[(0, 0), (10, 0), (10, 10), (20, 10), (20, 20), (10, 20), (10, 10), (0, 10)];
        let (ok, simple) = clip(&[bow], &[], ClipType::ctUnion, PolyFillType::pftNonZero, IO_STRICTLY_SIMPLE);
        assert!(ok);
        assert_eq!(simple.len(), 2);
        assert!(simple.iter().all(|p| p.len() == 4 && area(p) == 100.0));
    }

    #[test]
    fn execute_with_nothing_to_clip_and_twice() {
        let (ok, none) = clip(&[], &[], ClipType::ctUnion, PolyFillType::pftNonZero, 0);
        assert!(ok && none.is_empty());
        // Execute consumes the local minima list (InsertLocalMinimaIntoAEL pops it):
        // a second Execute finds nothing, as in the C++.
        let mut c = Clipper::default();
        let square: Path = SQUARE_A.iter().map(|q| IntPoint::new(q.0, q.1)).collect();
        assert!(c.add_path(&square, PolyType::ptSubject, true));
        let mut solution = Paths::new();
        assert!(c.execute(ClipType::ctUnion, &mut solution, PolyFillType::pftNonZero, PolyFillType::pftNonZero));
        assert_eq!(solution.len(), 1);
        assert!(c.execute(ClipType::ctUnion, &mut solution, PolyFillType::pftNonZero, PolyFillType::pftNonZero));
        assert!(solution.is_empty());
        assert!(c.m_poly_outs.is_empty() && c.m_out_pts.is_empty());
    }

    #[test]
    #[should_panic(expected = "PolyTree struct is needed for open path clipping")]
    fn execute_into_paths_refuses_open_paths() {
        let mut c = Clipper::default();
        let line: Path = vec![IntPoint::new(0, 0), IntPoint::new(10, 5)];
        assert!(c.add_path(&line, PolyType::ptSubject, false));
        let mut solution = Paths::new();
        c.execute(ClipType::ctUnion, &mut solution, PolyFillType::pftNonZero, PolyFillType::pftNonZero);
    }

    #[test]
    fn out_pt_helpers() {
        // A 4 x 2 rectangle as a ring of output points.
        let pts = ring(&[(0, 0), (4, 0), (4, 2), (0, 2)]);
        let rec = OutRec { pts: Some(0), ..OutRec::default() };
        // Area(OutRec) follows Next, and BuildResult writes a polygon out along Prev:
        // the sign is the opposite of Area(Path) of the same points in Next order.
        assert_eq!(area_out_rec(&pts, &rec), -8.0);
        let path: Path = pts.iter().map(|p| p.pt).collect();
        assert_eq!(area(&path), 8.0);
        assert_eq!(area_out_rec(&pts, &OutRec::default()), 0.0);
        assert_eq!(point_count(&pts, Some(2)), 4);
        assert_eq!(point_count(&pts, None), 0);
        assert_eq!(point_in_polygon_out_pt(&IntPoint::new(1, 1), &pts, 0), 1);
        assert_eq!(point_in_polygon_out_pt(&IntPoint::new(4, 1), &pts, 0), -1);
        assert_eq!(point_in_polygon_out_pt(&IntPoint::new(9, 1), &pts, 0), 0);
        let (mut left, mut right) = (0, 0);
        assert!(get_overlap(0, 10, 20, 4, &mut left, &mut right));
        assert_eq!((left, right), (4, 10));
        assert!(!get_overlap(0, 3, 3, 9, &mut left, &mut right));
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

    #[test]
    fn reset_sorts_the_minima_and_get_bounds_covers_the_path() {
        let mut c = ClipperBase::default();
        let p = |v: &[(cInt, cInt)]| -> Path { v.iter().map(|q| IntPoint::new(q.0, q.1)).collect() };
        assert!(c.add_path(&p(&[(0, 20), (10, 20), (10, 30), (0, 30)]), PolyType::ptSubject, true));
        assert!(c.add_path(&p(&[(-5, 0), (40, 0), (40, 10), (-5, 10)]), PolyType::ptClip, true));
        c.reset();
        let ys: Vec<cInt> = c.m_minima_list.iter().map(|lm| lm.y).collect();
        assert_eq!(ys, vec![10, 30]);
        for lm in &c.m_minima_list {
            let (l, r) = (lm.left_bound.unwrap(), lm.right_bound.unwrap());
            assert_eq!(c.m_edges[l].curr, c.m_edges[l].bot);
            assert_eq!((c.m_edges[l].side, c.m_edges[r].side), (EdgeSide::esLeft, EdgeSide::esRight));
        }
        assert_eq!(c.get_bounds(), IntRect { left: -5, top: 0, right: 40, bottom: 30 });
        assert_eq!(ClipperBase::default().get_bounds(), IntRect::default());
    }

}
