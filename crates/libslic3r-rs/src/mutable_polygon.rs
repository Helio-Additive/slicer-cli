//! MutablePolygon: polygon implemented as a loop of double-linked elements.
//!
//! Faithful 1:1 port of BambuStudio `src/libslic3r/MutablePolygon.{hpp,cpp}`.
//!
//! C++ references are noted inline as `// MutablePolygon.cpp:NNN` / `.hpp:NNN`.
//!
//! Design note (parity-preserving): the C++ class exposes `iterator` /
//! `const_iterator` objects that hold a pointer to the polygon plus an integer
//! index into a backing `std::vector<LinkedPoint>`. Because the algorithms keep
//! several live cursors that mutate the same polygon (it0/it1/it2 and a
//! `range` with begin/end), a Rust port that embedded a `&mut` inside each
//! cursor would not type-check. To preserve the exact control flow we model
//! iterators as `Copy` index cursors (`{ m_idx }`) and route every data access
//! / mutation through `&MutablePolygon` / `&mut MutablePolygon`. The numeric
//! behaviour (integer widths, rounding/truncation, edge cases) matches C++
//! exactly.

use crate::geometry::{cross2, ExPolygons, Point, Polygon, Polygons};
use crate::libslic3r::SCALED_EPSILON;
use crate::{scaled, Coord};

// MutablePolygon.hpp:17 — using IndexType = int32_t;
type IndexType = i32;

// MutablePolygon.cpp / libslic3r.h:275 — template<typename T> constexpr inline T sqr(T x) { return x * x; }
#[inline]
fn sqr_i64(x: i64) -> i64 {
    x * x
}
#[inline]
fn sqr_f64(x: f64) -> f64 {
    x * x
}

// ---------------------------------------------------------------------------
// Vec2i64 / Vec2d helpers
//
// The C++ uses Eigen `Vec2i64` (int64 vector) and `Vec2d` (double vector) with
// `.cast<>()`, `.squaredNorm()` and `.dot()`. The C++ algorithm deliberately
// widens point coordinates to int64 via `.cast<int64_t>()` before computing
// squared norms / dots / cross products (because `coord_t` is `int32_t`, those
// products would otherwise overflow). We mirror that with a local `Vec2i64`:
// the crate's `Point::cross2`/etc. return i128, which would change the integer
// width, so we replicate the Eigen ops on bare (i64,i64) / (f64,f64) tuples to
// match the C++ int64 intermediate arithmetic exactly.
// FIDELITY-NOTE(F2): the crate-wide `Coord = i64` (C++ `coord_t = int32_t`,
// libslic3r.h:40) only differs at the point-storage layer; the int64 intermediate
// math reproduced here is identical to C++.
// ---------------------------------------------------------------------------

#[derive(Clone, Copy)]
struct Vec2i64 {
    x: i64,
    y: i64,
}
impl Vec2i64 {
    #[inline]
    fn squared_norm(self) -> i64 {
        self.x * self.x + self.y * self.y
    }
    #[inline]
    fn dot(self, o: Vec2i64) -> i64 {
        self.x * o.x + self.y * o.y
    }
    #[inline]
    fn cast_f64(self) -> Vec2d {
        Vec2d {
            x: self.x as f64,
            y: self.y as f64,
        }
    }
}
impl std::ops::Sub for Vec2i64 {
    type Output = Vec2i64;
    #[inline]
    fn sub(self, o: Vec2i64) -> Vec2i64 {
        Vec2i64 {
            x: self.x - o.x,
            y: self.y - o.y,
        }
    }
}

#[derive(Clone, Copy)]
struct Vec2d {
    x: f64,
    y: f64,
}
impl Vec2d {
    #[inline]
    fn squared_norm(self) -> f64 {
        self.x * self.x + self.y * self.y
    }
    #[inline]
    fn dot(self, o: Vec2d) -> f64 {
        self.x * o.x + self.y * o.y
    }
    // Eigen `.cast<coord_t>()` truncates toward zero (static_cast double->int).
    // FIDELITY-NOTE(F2): C++ `coord_t` is `int32_t` (libslic3r.h:40), so the cast
    // is double->int32 and the subsequent `Point += delta` store wraps at int32.
    // The crate-wide `Coord = i64` keeps both the delta and the stored coordinate
    // at 64-bit; for in-range polygon coordinates (±2147mm) the result is identical.
    // Narrowing the storage width is the cross-cutting F2 rework, not done per-file.
    #[inline]
    fn cast_coord(self) -> Point {
        Point::new(self.x as Coord, self.y as Coord)
    }
}
impl std::ops::Mul<f64> for Vec2d {
    type Output = Vec2d;
    #[inline]
    fn mul(self, t: f64) -> Vec2d {
        Vec2d {
            x: self.x * t,
            y: self.y * t,
        }
    }
}

// (p_a - p_b).cast<int64_t>() for two Points.
#[inline]
fn diff_i64(a: Point, b: Point) -> Vec2i64 {
    Vec2i64 {
        x: a.x - b.x,
        y: a.y - b.y,
    }
}

// cross2 over Vec2i64 (returns int64 in C++; cross2() in this crate returns i128,
// so we narrow back to the int64 the C++ uses). Inputs are bounded such that the
// product fits in i64 for the cases this algorithm exercises.
#[inline]
fn cross2_i64(a: Vec2i64, b: Vec2i64) -> i64 {
    cross2(Point::new(a.x, a.y), Point::new(b.x, b.y)) as i64
}

// MutablePolygon.hpp:188-195 — struct LinkedPoint
#[derive(Clone, Copy)]
struct LinkedPoint {
    // 8 bytes
    point: Point,
    // 4 bytes
    prev: IndexType,
    // 4 bytes
    next: IndexType,
}

// MutablePolygon.hpp:14 — class MutablePolygon
//
// Polygon implemented as a loop of double linked elements.
// All elements are allocated in a single std::vector<>, thus integer indices are used for
// referencing the previous and next element and inside iterators to survive reallocation
// of the vector.
#[derive(Clone, Default)]
pub struct MutablePolygon {
    m_data: Vec<LinkedPoint>,
    // Number of points in the linked list.
    m_size: IndexType,
    m_head: IndexType,
    // Head of the free list.
    m_head_free: IndexType,
}

// MutablePolygon.hpp:19 — class const_iterator
// MutablePolygon.hpp:41 — class iterator
//
// Both iterators carry only an index here (see module note); data access is
// routed through &MutablePolygon. A single `Iter` type stands in for both,
// since Rust does not have C++'s const/non-const split at the type level.
#[derive(Clone, Copy)]
pub struct Iter {
    m_idx: IndexType,
}

impl Iter {
    // MutablePolygon.hpp:51 — bool valid() const { return m_idx >= 0; }
    #[inline]
    pub fn valid(&self) -> bool {
        self.m_idx >= 0
    }
}

impl MutablePolygon {
    // MutablePolygon.hpp:128 — MutablePolygon() = default;
    pub fn new() -> Self {
        Self {
            m_data: Vec::new(),
            m_size: 0,
            m_head: -1,
            m_head_free: -1,
        }
    }

    // MutablePolygon.hpp:129 — MutablePolygon(const Polygon &rhs, size_t reserve = 0)
    pub fn from_polygon(rhs: &Polygon, reserve: usize) -> Self {
        let mut out = MutablePolygon::new();
        out.assign_inner(&rhs.points, reserve);
        out
    }

    // MutablePolygon.hpp:138 — void assign(IT begin, IT end, size_t reserve = 0)
    pub fn assign_points(&mut self, points: &[Point], reserve: usize) {
        self.m_data.clear();
        self.m_head = -1;
        self.m_head_free = -1;
        self.assign_inner(points, reserve);
    }

    // MutablePolygon.hpp:145 — void assign(const Polygon &rhs, size_t reserve = 0)
    pub fn assign(&mut self, rhs: &Polygon, reserve: usize) {
        self.assign_points(&rhs.points, reserve);
    }

    // MutablePolygon.hpp:149 — void polygon(Polygon &out) const
    pub fn polygon_into(&self, out: &mut Polygon) {
        out.points.clear();
        if self.valid() {
            out.points.reserve(self.size());
            let mut it = self.cbegin();
            out.points.push(*self.at_point(it.m_idx));
            // for (++ it; it != this->cbegin(); ++ it)
            it = self.inc(it);
            while !self.iter_eq(it, self.cbegin()) {
                out.points.push(*self.at_point(it.m_idx));
                it = self.inc(it);
            }
        }
    }

    // MutablePolygon.hpp:160 — Polygon polygon() const
    pub fn polygon(&self) -> Polygon {
        let mut out = Polygon::new();
        self.polygon_into(&mut out);
        out
    }

    // MutablePolygon.hpp:166 — bool empty() const { return m_size == 0; }
    #[inline]
    pub fn empty(&self) -> bool {
        self.m_size == 0
    }
    // MutablePolygon.hpp:167 — size_t size() const { return m_size; }
    #[inline]
    pub fn size(&self) -> usize {
        self.m_size as usize
    }
    // MutablePolygon.hpp:168 — size_t capacity() const { return m_data.capacity(); }
    #[inline]
    pub fn capacity(&self) -> usize {
        self.m_data.capacity()
    }
    // MutablePolygon.hpp:169 — bool valid() const { return m_size >= 3; }
    #[inline]
    pub fn valid(&self) -> bool {
        self.m_size >= 3
    }
    // MutablePolygon.hpp:170 — void clear()
    #[inline]
    pub fn clear(&mut self) {
        self.m_data.clear();
        self.m_size = 0;
        self.m_head = -1;
        self.m_head_free = -1;
    }

    // MutablePolygon.hpp:172 — iterator begin() { return { this, m_head }; }
    #[inline]
    pub fn begin(&self) -> Iter {
        Iter { m_idx: self.m_head }
    }
    // MutablePolygon.hpp:173 — const_iterator cbegin() const { return { this, m_head }; }
    #[inline]
    pub fn cbegin(&self) -> Iter {
        Iter { m_idx: self.m_head }
    }
    // MutablePolygon.hpp:176 — iterator end()
    // End points to the last item before roll over. This is different from the usual end() concept!
    #[inline]
    pub fn end(&self) -> Iter {
        Iter {
            m_idx: if self.empty() {
                -1
            } else {
                self.at(self.m_head).prev
            },
        }
    }
    // MutablePolygon.hpp:177 — const_iterator cend() const
    #[inline]
    pub fn cend(&self) -> Iter {
        self.end()
    }

    // ----- iterator navigation / access (data routed through self) -----

    #[inline]
    fn at(&self, i: IndexType) -> &LinkedPoint {
        &self.m_data[i as usize]
    }
    #[inline]
    fn at_mut(&mut self, i: IndexType) -> &mut LinkedPoint {
        &mut self.m_data[i as usize]
    }
    // MutablePolygon.hpp:30/52 — PointType& operator*()
    #[inline]
    fn at_point(&self, i: IndexType) -> &Point {
        &self.m_data[i as usize].point
    }
    #[inline]
    fn at_point_mut(&mut self, i: IndexType) -> &mut Point {
        &mut self.m_data[i as usize].point
    }

    // MutablePolygon.hpp:47 — iterator& operator++()
    #[inline]
    fn inc(&self, it: Iter) -> Iter {
        Iter {
            m_idx: self.at(it.m_idx).next,
        }
    }
    // MutablePolygon.hpp:45 — iterator& operator--()
    #[inline]
    fn dec(&self, it: Iter) -> Iter {
        Iter {
            m_idx: self.at(it.m_idx).prev,
        }
    }
    // MutablePolygon.hpp:49 — iterator prev() const
    #[inline]
    fn iter_prev(&self, it: Iter) -> Iter {
        Iter {
            m_idx: self.at(it.m_idx).prev,
        }
    }
    // MutablePolygon.hpp:50 — iterator next() const
    #[inline]
    fn iter_next(&self, it: Iter) -> Iter {
        Iter {
            m_idx: self.at(it.m_idx).next,
        }
    }
    // MutablePolygon.hpp:43 — bool operator==(const iterator &rhs) const
    #[inline]
    fn iter_eq(&self, a: Iter, b: Iter) -> bool {
        a.m_idx == b.m_idx
    }

    // MutablePolygon.hpp:56 — iterator& remove() { m_idx = m_data->remove(*this).m_idx; return *this; }
    // Returns the iterator following the removed element.
    #[inline]
    fn iter_remove(&mut self, it: Iter) -> Iter {
        Iter {
            m_idx: self.remove(it.m_idx),
        }
    }
    // MutablePolygon.hpp:57 — iterator insert(const PointType pt) const
    #[inline]
    fn iter_insert(&mut self, it: Iter, pt: Point) -> Iter {
        Iter {
            m_idx: self.insert(it.m_idx, pt),
        }
    }

    // MutablePolygon.hpp:207 — void assign_inner(IT begin, IT end, size_t reserve)
    fn assign_inner(&mut self, points: &[Point], reserve: usize) {
        self.m_size = points.len() as IndexType;
        if self.m_size > 0 {
            self.m_head = 0;
            self.m_data
                .reserve(std::cmp::max(self.m_size as usize, reserve));
            let mut i: IndexType = -1;
            let mut j: IndexType = 1;
            for it in points {
                self.m_data.push(LinkedPoint {
                    point: *it,
                    prev: i,
                    next: j,
                });
                i += 1;
                j += 1;
            }
            let last = (self.m_size - 1) as usize;
            self.m_data[0].prev = self.m_size - 1;
            self.m_data[last].next = 0;
        }
    }

    // MutablePolygon.hpp:221 — IndexType remove(const IndexType i)
    fn remove(&mut self, i: IndexType) -> IndexType {
        debug_assert!(i >= 0);
        debug_assert!(self.m_size > 0);
        debug_assert!(self.m_head != -1);
        let head_free = self.m_head_free;
        let (prev, next);
        {
            let lp = self.at_mut(i);
            prev = lp.prev;
            next = lp.next;
            lp.next = head_free;
        }
        self.m_head_free = i;
        self.m_size -= 1;
        if self.m_size == 0 {
            self.m_head = -1;
        } else if self.m_head == i {
            self.m_head = next;
        }
        debug_assert!(!self.empty() || (prev == i && next == i));
        if self.empty() {
            return -1;
        }
        self.at_mut(prev).next = next;
        self.at_mut(next).prev = prev;
        next
    }

    // MutablePolygon.hpp:242 — IndexType insert(const IndexType i, const Point pt)
    fn insert(&mut self, i: IndexType, pt: Point) -> IndexType {
        debug_assert!(i >= 0);
        let n: IndexType;
        let j = self.at(i).prev;
        if self.m_head_free == -1 {
            // Allocate a new item.
            n = self.m_data.len() as IndexType;
            self.m_data.push(LinkedPoint {
                point: pt,
                prev: j,
                next: i,
            });
        } else {
            n = self.m_head_free;
            self.m_head_free = self.at(n).next;
            *self.at_mut(n) = LinkedPoint {
                point: pt,
                prev: j,
                next: i,
            };
        }
        self.at_mut(j).next = n;
        self.at_mut(i).prev = n;
        self.m_size += 1;
        n
    }
}

// MutablePolygon.hpp:67 — class range
//
// Iterator range for maintaining a range of unprocessed items, see smooth_outward().
// Range from begin to end, inclusive. If the range is invalid, then both
// m_begin and m_end are invalid.
#[derive(Clone, Copy)]
struct Range {
    m_begin: Iter,
    m_end: Iter,
}

impl Range {
    // MutablePolygon.hpp:70 — range(MutablePolygon& poly) : range(poly.begin(), poly.end()) {}
    fn new(poly: &MutablePolygon) -> Self {
        Range {
            m_begin: poly.begin(),
            m_end: poly.end(),
        }
    }

    // MutablePolygon.hpp:74 — MutablePolygon::iterator begin() const
    #[allow(dead_code)]
    fn begin(&self) -> Iter {
        self.m_begin
    }
    // MutablePolygon.hpp:76 — MutablePolygon::iterator end() const
    #[allow(dead_code)]
    fn end(&self) -> Iter {
        self.m_end
    }
    // MutablePolygon.hpp:78 — bool empty() const { return !m_begin.valid(); }
    fn empty(&self) -> bool {
        !self.m_begin.valid()
    }

    // MutablePolygon.hpp:81 — MutablePolygon::iterator process_next()
    // Return begin() and shorten the range by advancing front.
    fn process_next(&mut self, poly: &MutablePolygon) -> Iter {
        debug_assert!(!self.empty());
        let out = self.m_begin;
        self.advance_front(poly);
        out
    }

    // MutablePolygon.hpp:88 — void advance_front()
    fn advance_front(&mut self, poly: &MutablePolygon) {
        debug_assert!(!self.empty());
        if poly.iter_eq(self.m_begin, self.m_end) {
            self.make_empty();
        } else {
            self.m_begin = poly.inc(self.m_begin);
        }
    }

    // MutablePolygon.hpp:96 — void retract_back()
    fn retract_back(&mut self, poly: &MutablePolygon) {
        debug_assert!(!self.empty());
        if poly.iter_eq(self.m_begin, self.m_end) {
            self.make_empty();
        } else {
            self.m_end = poly.dec(self.m_end);
        }
    }

    // MutablePolygon.hpp:104 — MutablePolygon::iterator remove_front(MutablePolygon::iterator it)
    fn remove_front(&mut self, poly: &mut MutablePolygon, it: Iter) -> Iter {
        if !self.empty() && poly.iter_eq(self.m_begin, it) {
            self.advance_front(poly);
        }
        poly.iter_remove(it)
    }

    // MutablePolygon.hpp:110 — MutablePolygon::iterator remove_back(MutablePolygon::iterator it)
    fn remove_back(&mut self, poly: &mut MutablePolygon, it: Iter) -> Iter {
        if !self.empty() && poly.iter_eq(self.m_end, it) {
            self.retract_back(poly);
        }
        poly.iter_remove(it)
    }

    // MutablePolygon.hpp:122 — void make_empty()
    fn make_empty(&mut self) {
        self.m_begin.m_idx = -1;
        self.m_end.m_idx = -1;
    }
}

// MutablePolygon.hpp:288 — inline bool operator==(const MutablePolygon &p1, const MutablePolygon &p2)
impl PartialEq for MutablePolygon {
    fn eq(&self, p2: &MutablePolygon) -> bool {
        let p1 = self;
        if p1.size() != p2.size() {
            return false;
        }
        if p1.empty() {
            return true;
        }
        let begin = p1.cbegin();
        let mut it = begin;
        let mut it2 = p2.cbegin();
        loop {
            if *p1.at_point(it.m_idx) != *p2.at_point(it2.m_idx) {
                return false;
            }
            it = p1.inc(it);
            if p1.iter_eq(it, begin) {
                return true;
            }
            it2 = p2.inc(it2);
        }
    }
}

// MutablePolygon.cpp:8 — void remove_duplicates(MutablePolygon &polygon)
// Remove exact duplicate points. May reduce the polygon down to empty polygon.
pub fn remove_duplicates(polygon: &mut MutablePolygon) {
    // MutablePolygon.cpp:10
    if !polygon.empty() {
        let begin = polygon.begin();
        let mut it = begin;
        // for (++ it; it != begin;)
        it = polygon.inc(it);
        while !polygon.iter_eq(it, begin) {
            let prev = polygon.iter_prev(it);
            // MutablePolygon.cpp:15 — if (*prev == *it)
            if *polygon.at_point(prev.m_idx) == *polygon.at_point(it.m_idx) {
                it = polygon.iter_remove(it);
            } else {
                it = polygon.inc(it);
            }
        }
    }
}

// MutablePolygon.cpp:24 — void remove_duplicates(MutablePolygon &polygon, double eps)
// Remove nearly duplicate points. May reduce the polygon down to empty polygon.
pub fn remove_duplicates_eps(polygon: &mut MutablePolygon, eps: f64) {
    // MutablePolygon.cpp:26
    if !polygon.empty() {
        let eps2 = eps * eps;
        let begin = polygon.begin();
        let mut it = begin;
        // for (++ it; it != begin;)
        it = polygon.inc(it);
        while !polygon.iter_eq(it, begin) {
            let prev = polygon.iter_prev(it);
            // MutablePolygon.cpp:32 — if ((*it - *prev).cast<double>().squaredNorm() < eps2)
            let d = diff_i64(*polygon.at_point(it.m_idx), *polygon.at_point(prev.m_idx));
            if d.cast_f64().squared_norm() < eps2 {
                it = polygon.iter_remove(it);
            } else {
                it = polygon.inc(it);
            }
        }
    }
}

// MutablePolygon.cpp:40 — void remove_duplicates(MutablePolygon& polygon, coord_t scaled_eps, const double max_angle)
pub fn remove_duplicates_angle(polygon: &mut MutablePolygon, scaled_eps: Coord, max_angle: f64) {
    // MutablePolygon.cpp:42
    if polygon.size() >= 3 {
        let cos_max_angle_2 = sqr_f64(max_angle.cos());
        let scaled_eps_sqr = sqr_i64(scaled_eps);
        let begin = polygon.begin();
        let mut it = begin;
        // for (++it; it != begin;)
        it = polygon.inc(it);
        while !polygon.iter_eq(it, begin) {
            let prev = polygon.iter_prev(it);
            let next = polygon.iter_next(it);
            // Vec2i64 v1 = (*it - *prev).cast<int64_t>();
            let v1 = diff_i64(*polygon.at_point(it.m_idx), *polygon.at_point(prev.m_idx));
            let v1_sqr_norm = v1.squared_norm();
            // MutablePolygon.cpp:52
            if v1_sqr_norm < scaled_eps_sqr {
                // if (Vec2i64 v2 = (*next - *prev).cast<int64_t>(); ...)
                let v2 = diff_i64(*polygon.at_point(next.m_idx), *polygon.at_point(prev.m_idx));
                if sqr_f64(v1.dot(v2) as f64)
                    > cos_max_angle_2 * (v1_sqr_norm as f64) * (v2.squared_norm() as f64)
                {
                    it = polygon.iter_remove(it);
                    continue;
                }
            }
            it = next;
        }
    }

    // MutablePolygon.cpp:63
    if polygon.size() < 3 {
        polygon.clear();
    }
}

// MutablePolygon.cpp:74 — static bool clip_narrow_corner(...)
//
// Adapted from Cura ConstPolygonRef::smooth_corner_complex() by Tim Kuipers.
// A concave corner at it1 with position p1 has been removed by the caller between it0 and it2, where |p2 - p0| < shortcut_length.
// Now try to close a concave crack by walking left from it0 and right from it2 as long as the new clipping edge is smaller than shortcut_length
// and the new clipping edge is still inside the polygon (it is a diagonal, it does not intersect polygon boundary).
// Once the traversal stops (always at a clipping edge shorter than shortcut_length), the final trapezoid is clipped with a new clipping edge of shortcut_length.
// Return true if a hole was completely closed (degenerated to an empty polygon) or a single CCW triangle was left, which is not to be simplified any further.
// it0, it2 are updated to the final clipping edge.
#[derive(Clone, Copy, PartialEq)]
enum Status {
    Free,
    Blocked,
    Far,
}

fn clip_narrow_corner(
    p1: Vec2i64,
    it0: &mut Iter,
    it2: &mut Iter,
    unprocessed_range: &mut Range,
    mut dist2_current: i64,
    shortcut_length: i64,
    polygon: &mut MutablePolygon,
) -> bool {
    // MutablePolygon.cpp:82 — MutablePolygon &polygon = it0.polygon();
    debug_assert!(polygon.size() >= 2);

    // MutablePolygon.cpp:85 — const int64_t shortcut_length2 = sqr(shortcut_length);
    let shortcut_length2 = sqr_i64(shortcut_length);

    // MutablePolygon.cpp:92
    let mut forward = Status::Free;
    let mut backward = Status::Free;

    // MutablePolygon.cpp:95 — Vec2i64 p0 = it0->cast<int64_t>();
    let mut p0 = {
        let p = *polygon.at_point(it0.m_idx);
        Vec2i64 { x: p.x, y: p.y }
    };
    // MutablePolygon.cpp:96 — Vec2i64 p2 = it2->cast<int64_t>();
    let mut p2 = {
        let p = *polygon.at_point(it2.m_idx);
        Vec2i64 { x: p.x, y: p.y }
    };
    let mut p02 = Vec2i64 { x: 0, y: 0 };
    let mut p22 = Vec2i64 { x: 0, y: 0 };
    let mut dist2_next: i64 = 0;

    // MutablePolygon.cpp:102 — As long as there is at least a single triangle left in the polygon.
    while polygon.size() >= 3 {
        debug_assert!(dist2_current <= shortcut_length2);
        // MutablePolygon.cpp:104
        if forward == Status::Far && backward == Status::Far {
            p02 = {
                let pp = polygon.iter_prev(*it0);
                let p = *polygon.at_point(pp.m_idx);
                Vec2i64 { x: p.x, y: p.y }
            };
            p22 = {
                let nn = polygon.iter_next(*it2);
                let p = *polygon.at_point(nn.m_idx);
                Vec2i64 { x: p.x, y: p.y }
            };
            let d2 = (p22 - p02).squared_norm();
            // MutablePolygon.cpp:108
            if d2 <= shortcut_length2 {
                // The region was narrow until now and it is still narrow. Trim at both sides.
                let removed_back = unprocessed_range.remove_back(polygon, *it0);
                *it0 = polygon.iter_prev(removed_back);
                *it2 = unprocessed_range.remove_front(polygon, *it2);
                // MutablePolygon.cpp:112
                if polygon.size() <= 2 {
                    // A hole degenerated to an empty polygon.
                    return true;
                }
                forward = Status::Free;
                backward = Status::Free;
                dist2_current = d2;
                p0 = p02;
                p2 = p22;
            } else {
                // The region is widening. Stop traversal and trim the final trapezoid.
                dist2_next = d2;
                break;
            }
        } else if forward != Status::Free && backward != Status::Free {
            // One of the corners is blocked, the other is blocked or too far. Stop traversal.
            break;
        }
        // Try to proceed by flipping a diagonal.
        // Progress by keeping the distance of the clipping edge end points equal to initial p1.
        //FIXME This is an arbitrary condition, maybe a more local condition will be better (take a shorter diagonal?).
        // MutablePolygon.cpp:131
        if forward == Status::Free
            && (backward != Status::Free || (p2 - p1).squared_norm() < (p0 - p1).squared_norm())
        {
            p22 = {
                let nn = polygon.iter_next(*it2);
                let p = *polygon.at_point(nn.m_idx);
                Vec2i64 { x: p.x, y: p.y }
            };
            // MutablePolygon.cpp:133 — if (cross2(p2 - p0, p22 - p0) > 0)
            if cross2_i64(p2 - p0, p22 - p0) > 0 {
                forward = Status::Blocked;
            } else {
                // New clipping edge lenght.
                let d2 = (p22 - p0).squared_norm();
                // MutablePolygon.cpp:138
                if d2 > shortcut_length2 {
                    forward = Status::Far;
                    dist2_next = d2;
                } else {
                    forward = Status::Free;
                    // Make one step in the forward direction.
                    *it2 = unprocessed_range.remove_front(polygon, *it2);
                    p2 = p22;
                    dist2_current = d2;
                }
            }
        } else {
            debug_assert!(backward == Status::Free);
            p02 = {
                let pp = polygon.iter_prev(*it0);
                let p = *polygon.at_point(pp.m_idx);
                Vec2i64 { x: p.x, y: p.y }
            };
            // MutablePolygon.cpp:152 — if (cross2(p02 - p2, p0 - p2) > 0)
            if cross2_i64(p02 - p2, p0 - p2) > 0 {
                backward = Status::Blocked;
            } else {
                // New clipping edge lenght.
                let d2 = (p2 - p02).squared_norm();
                // MutablePolygon.cpp:157
                if d2 > shortcut_length2 {
                    backward = Status::Far;
                    dist2_next = d2;
                } else {
                    backward = Status::Free;
                    // Make one step in the backward direction.
                    let removed_back = unprocessed_range.remove_back(polygon, *it0);
                    *it0 = polygon.iter_prev(removed_back);
                    p0 = p02;
                    dist2_current = d2;
                }
            }
        }
    }

    debug_assert!(dist2_current <= shortcut_length2);
    debug_assert!(polygon.size() >= 2);
    debug_assert!(polygon.size() == 2 || forward == Status::Blocked || forward == Status::Far);
    debug_assert!(polygon.size() == 2 || backward == Status::Blocked || backward == Status::Far);

    // MutablePolygon.cpp:176
    if polygon.size() <= 3 {
        // A hole degenerated to an empty polygon, or a tiny triangle remained.
        // (NDEBUG-only verification block omitted; it has no effect on release behaviour.)
        // MutablePolygon.cpp:197
        if polygon.size() < 3 || (forward == Status::Far && backward == Status::Far) {
            polygon.clear();
        } else {
            // The remaining triangle is CCW oriented, keep it.
            debug_assert!(forward == Status::Blocked || backward == Status::Blocked);
        }
        return true;
    }

    debug_assert!(dist2_current <= shortcut_length2);
    // MutablePolygon.cpp:207
    if (forward == Status::Blocked && backward == Status::Blocked)
        || dist2_current > sqr_i64(shortcut_length - SCALED_EPSILON as i64)
    {
        // The crack is filled, keep the last clipping edge.
    } else if dist2_next < sqr_i64(shortcut_length - SCALED_EPSILON as i64) {
        // To avoid creating tiny edges.
        // MutablePolygon.cpp:211
        if forward == Status::Far {
            let removed_back = unprocessed_range.remove_back(polygon, *it0);
            *it0 = polygon.iter_prev(removed_back);
        }
        if backward == Status::Far {
            *it2 = unprocessed_range.remove_front(polygon, *it2);
        }
        if polygon.size() <= 2 {
            // A hole degenerated to an empty polygon.
            return true;
        }
    } else if forward == Status::Blocked || backward == Status::Blocked {
        // One side is far, the other blocked.
        debug_assert!(forward == Status::Far || backward == Status::Far);
        // MutablePolygon.cpp:221
        if forward == Status::Far {
            // Sort, so we will clip the 1st edge.
            std::mem::swap(&mut p0, &mut p2);
            std::mem::swap(&mut p02, &mut p22);
        }
        // Find point on (p0, p02) at distance shortcut_length from p2.
        // Circle intersects a line at two points, however because |p2 - p0| < shortcut_length,
        // only the second intersection is valid. Because |p2 - p02| > shortcut_length, such
        // intersection should always be found on (p0, p02).
        // MutablePolygon.cpp:234 — const Vec2d v = (p02 - p0).cast<double>();
        let v = (p02 - p0).cast_f64();
        // MutablePolygon.cpp:235 — const Vec2d d = (p0 - p2).cast<double>();
        let d = (p0 - p2).cast_f64();
        let a = v.squared_norm();
        let b = 2. * d.dot(v);
        let mut u = b * b - 4. * a * (d.squared_norm() - shortcut_length2 as f64);
        debug_assert!(u > 0.);
        u = u.sqrt();
        let t = (-b + u) / (2. * a);
        debug_assert!(t > 0. && t < 1.);
        // (backward == Far ? *it2 : *it0) += (v.cast<double>() * t).cast<coord_t>();
        let delta = (v * t).cast_coord();
        let target = if backward == Status::Far { *it2 } else { *it0 };
        *polygon.at_point_mut(target.m_idx) += delta;
    } else {
        // The trapezoid (it0.prev(), it0, it2, it2.next()) is widening. Trim it.
        debug_assert!(forward == Status::Far && backward == Status::Far);
        debug_assert!(dist2_next > shortcut_length2);
        // MutablePolygon.cpp:248
        let dcurrent = (dist2_current as f64).sqrt();
        let t = (shortcut_length as f64 - dcurrent) / ((dist2_next as f64).sqrt() - dcurrent);
        debug_assert!(t > 0. && t < 1.);
        // *it0 += ((p02 - p0).cast<double>() * t).cast<coord_t>();
        let d0 = ((p02 - p0).cast_f64() * t).cast_coord();
        *polygon.at_point_mut(it0.m_idx) += d0;
        // *it2 += ((p22 - p2).cast<double>() * t).cast<coord_t>();
        let d2v = ((p22 - p2).cast_f64() * t).cast_coord();
        *polygon.at_point_mut(it2.m_idx) += d2v;
    }
    false
}

// MutablePolygon.cpp:258 — void smooth_outward(MutablePolygon &polygon, coord_t clip_dist_scaled)
// adapted from Cura ConstPolygonRef::smooth_outward() by Tim Kuipers.
pub fn smooth_outward(polygon: &mut MutablePolygon, clip_dist_scaled: Coord) {
    // MutablePolygon.cpp:260 — remove_duplicates(polygon, scaled<double>(0.01));
    remove_duplicates_eps(polygon, scaled(0.01) as f64);

    // MutablePolygon.cpp:262
    let clip_dist_scaled2 = sqr_i64(clip_dist_scaled);
    let clip_dist_scaled2eps = sqr_i64(clip_dist_scaled + SCALED_EPSILON as i64);
    let foot_dist_min2 = sqr_i64(SCALED_EPSILON as i64);

    // Each source point will be visited exactly once.
    // MutablePolygon.cpp:267
    let mut unprocessed_range = Range::new(polygon);
    while !unprocessed_range.empty() && polygon.size() > 2 {
        let it1_init = unprocessed_range.process_next(polygon);
        let mut it1 = it1_init;
        let mut it0 = polygon.iter_prev(it1);
        let mut it2 = polygon.iter_next(it1);
        let p0 = *polygon.at_point(it0.m_idx);
        let p1 = *polygon.at_point(it1.m_idx);
        let p2 = *polygon.at_point(it2.m_idx);
        // const Vec2i64 v1 = (p0 - p1).cast<int64_t>();
        let v1 = diff_i64(p0, p1);
        // const Vec2i64 v2 = (p2 - p1).cast<int64_t>();
        let v2 = diff_i64(p2, p1);
        // MutablePolygon.cpp:277 — if (cross2(v1, v2) > 0)
        if cross2_i64(v1, v2) > 0 {
            // Concave corner.
            let dot = v1.dot(v2);
            let mut l2v1 = v1.squared_norm() as f64;
            let mut l2v2 = v2.squared_norm() as f64;
            // MutablePolygon.cpp:282
            if dot > 0 || sqr_f64(dot as f64) * 2. < l2v1 * l2v2 {
                // Angle between v1 and v2 bigger than 135 degrees.
                // Simplify the sharp angle.
                // Vec2i64 v02 = (p2 - p0).cast<int64_t>();
                let v02 = diff_i64(p2, p0);
                let l2v02 = v02.squared_norm();
                // it1.remove();
                polygon.iter_remove(it1);
                // MutablePolygon.cpp:288
                if l2v02 < clip_dist_scaled2 {
                    // (p0, p2) is short.
                    // Clip a sharp concave corner by possibly expanding the trimming region left of it0 and right of it2.
                    // Updates it0, it2 and num_to_process.
                    let p1_i64 = Vec2i64 { x: p1.x, y: p1.y };
                    if clip_narrow_corner(
                        p1_i64,
                        &mut it0,
                        &mut it2,
                        &mut unprocessed_range,
                        l2v02,
                        clip_dist_scaled,
                        polygon,
                    ) {
                        // Trimmed down to an empty polygon or to a single CCW triangle.
                        return;
                    }
                } else {
                    // Clip an obtuse corner.
                    // MutablePolygon.cpp:297
                    if l2v02 > clip_dist_scaled2eps {
                        let mut v1d = v1.cast_f64();
                        let mut v2d = v2.cast_f64();
                        // Sort v1d, v2d, shorter first.
                        let swap = l2v1 > l2v2;
                        if swap {
                            std::mem::swap(&mut v1d, &mut v2d);
                            std::mem::swap(&mut l2v1, &mut l2v2);
                        }
                        let lv1 = l2v1.sqrt();
                        let lv2 = l2v2.sqrt();
                        // Bisector between v1 and v2.
                        // Vec2d bisector = v1d / lv1 + v2d / lv2;
                        let bisector = Vec2d {
                            x: v1d.x / lv1 + v2d.x / lv2,
                            y: v1d.y / lv1 + v2d.y / lv2,
                        };
                        let l2bisector = bisector.squared_norm();
                        // Squared distance of the end point of v1 to the bisector.
                        let d2 = l2v1 - sqr_f64(v1d.dot(bisector)) / l2bisector;
                        // MutablePolygon.cpp:313
                        if d2 < foot_dist_min2 as f64 {
                            // Height of the p1, p0, p2 triangle is tiny. Just remove p1.
                        } else if d2 < 0.25 * clip_dist_scaled2 as f64 + SCALED_EPSILON {
                            // The shorter vector is too close to the bisector. Trim the shorter vector fully,
                            // trim the longer vector partially.
                            // Intersection of a circle at p2 of radius = clip_dist_scaled
                            // with a ray (p1, p0), take the intersection after the foot point.
                            // The intersection shall always exist because |p2 - p1| > clip_dist_scaled.
                            // const double b = - 2. * v1d.cast<double>().dot(v2d);
                            let b = -2. * v1d.dot(v2d);
                            let u = b * b - 4. * l2v2 * (l2v1 - clip_dist_scaled2 as f64);
                            debug_assert!(u > 0.);
                            // Take the second intersection along v2.
                            let t = (-b + u.sqrt()) / (2. * l2v2);
                            debug_assert!(t > 0. && t < 1.);
                            // Point pt_new = p1 + (t * v2d).cast<coord_t>();
                            let pt_new = p1 + (v2d * t).cast_coord();
                            // (NDEBUG verification block omitted.)
                            // it2.insert(pt_new);
                            polygon.iter_insert(it2, pt_new);
                        } else {
                            // Cut the corner with a line perpendicular to the bisector.
                            let t = (0.25 * clip_dist_scaled2 as f64 / d2).sqrt();
                            let t2 = t * lv1 / lv2;
                            debug_assert!(t > 0. && t < 1.);
                            debug_assert!(t2 > 0. && t2 < 1.);
                            // Point p0 = p1 + (v1d * t ).cast<coord_t>();
                            let mut p0c = p1 + (v1d * t).cast_coord();
                            // Point p2 = p1 + (v2d * t2).cast<coord_t>();
                            let mut p2c = p1 + (v2d * t2).cast_coord();
                            if swap {
                                std::mem::swap(&mut p0c, &mut p2c);
                            }
                            // it2.insert(p2).insert(p0);
                            let inserted = polygon.iter_insert(it2, p2c);
                            polygon.iter_insert(inserted, p0c);
                        }
                    } else {
                        // Just remove p1.
                        debug_assert!(l2v02 >= clip_dist_scaled2 && l2v02 <= clip_dist_scaled2eps);
                    }
                }
                // it1 = it2;
                it1 = it2;
            } else {
                // ++ it1;
                it1 = polygon.inc(it1);
            }
        } else {
            // ++ it1;
            it1 = polygon.inc(it1);
        }
        let _ = it1; // matches C++ where it1 is updated but loop re-derives from range.
    }

    // MutablePolygon.cpp:357
    if polygon.size() == 3 {
        // Check whether the last triangle is clockwise oriented (it is a hole) and its height is below clip_dist_scaled.
        // If so, fill in the hole.
        let p0 = {
            let b = polygon.begin();
            *polygon.at_point(polygon.iter_prev(b).m_idx)
        };
        let p1 = *polygon.at_point(polygon.begin().m_idx);
        let p2 = {
            let b = polygon.begin();
            *polygon.at_point(polygon.iter_next(b).m_idx)
        };
        let mut v1 = diff_i64(p0, p1);
        let mut v2 = diff_i64(p2, p1);
        // MutablePolygon.cpp:365 — if (cross2(v1, v2) > 0)
        if cross2_i64(v1, v2) > 0 {
            // CW triangle. Measure its height.
            let v3 = diff_i64(p2, p0);
            let mut l12 = v1.squared_norm();
            let mut l22 = v2.squared_norm();
            let l32 = v3.squared_norm();
            // MutablePolygon.cpp:371
            if l22 > l12 && l22 > l32 {
                std::mem::swap(&mut v1, &mut v2);
                std::mem::swap(&mut l12, &mut l22);
            } else if l32 > l12 && l32 > l22 {
                v1 = v3;
                l12 = l32;
            }
            // auto h2 = l22 - sqr(double(v1.dot(v2))) / double(l12);
            let h2 = l22 as f64 - sqr_f64(v1.dot(v2) as f64) / (l12 as f64);
            if h2 < clip_dist_scaled2 as f64 {
                // CW triangle with a low height. Close the hole.
                polygon.clear();
            }
        }
    } else if polygon.size() < 3 {
        // MutablePolygon.cpp:383
        polygon.clear();
    }
}

// MutablePolygon.hpp:312 — inline ExPolygons remove_duplicates(ExPolygons expolygons, coord_t scaled_eps, double max_angle)
pub fn remove_duplicates_expolygons(
    mut expolygons: ExPolygons,
    scaled_eps: Coord,
    max_angle: f64,
) -> ExPolygons {
    let mut mp = MutablePolygon::new();
    for expolygon in expolygons.iter_mut() {
        mp.assign(&expolygon.contour, expolygon.contour.points.len() * 2);
        remove_duplicates_angle(&mut mp, scaled_eps, max_angle);
        mp.polygon_into(&mut expolygon.contour);
        for hole in expolygon.holes.iter_mut() {
            mp.assign(hole, hole.points.len() * 2);
            remove_duplicates_angle(&mut mp, scaled_eps, max_angle);
            mp.polygon_into(hole);
        }
        expolygon.holes.retain(|p| !p.points.is_empty());
    }
    expolygons.retain(|p| !p.contour.points.is_empty());
    expolygons
}

// MutablePolygon.hpp:332 — inline Polygon smooth_outward(Polygon polygon, coord_t clip_dist_scaled)
pub fn smooth_outward_polygon(mut polygon: Polygon, clip_dist_scaled: Coord) -> Polygon {
    let mut mp = MutablePolygon::from_polygon(&polygon, polygon.points.len() * 2);
    smooth_outward(&mut mp, clip_dist_scaled);
    mp.polygon_into(&mut polygon);
    polygon
}

// MutablePolygon.hpp:340 — inline Polygons smooth_outward(Polygons polygons, coord_t clip_dist_scaled)
pub fn smooth_outward_polygons(mut polygons: Polygons, clip_dist_scaled: Coord) -> Polygons {
    let mut mp = MutablePolygon::new();
    for polygon in polygons.iter_mut() {
        mp.assign(polygon, polygon.points.len() * 2);
        smooth_outward(&mut mp, clip_dist_scaled);
        mp.polygon_into(polygon);
    }
    polygons.retain(|p| !p.points.is_empty());
    polygons
}

// MutablePolygon.hpp:352 — inline ExPolygons smooth_outward(ExPolygons expolygons, coord_t clip_dist_scaled)
pub fn smooth_outward_expolygons(
    mut expolygons: ExPolygons,
    clip_dist_scaled: Coord,
) -> ExPolygons {
    let mut mp = MutablePolygon::new();
    for expolygon in expolygons.iter_mut() {
        mp.assign(&expolygon.contour, expolygon.contour.points.len() * 2);
        smooth_outward(&mut mp, clip_dist_scaled);
        mp.polygon_into(&mut expolygon.contour);
        for hole in expolygon.holes.iter_mut() {
            mp.assign(hole, hole.points.len() * 2);
            smooth_outward(&mut mp, clip_dist_scaled);
            mp.polygon_into(hole);
        }
        expolygon.holes.retain(|p| !p.points.is_empty());
    }
    expolygons.retain(|p| !p.contour.points.is_empty());
    expolygons
}
