//! MutablePolygon: polygon implemented as a loop of double-linked elements.
//!
//! Port of BambuStudio `src/libslic3r/MutablePolygon.{hpp,cpp}` at the pin; every function cites its
//! lines as `// MutablePolygon.cpp:NNN` / `.hpp:NNN`.
//!
//! The C++ `iterator` / `const_iterator` hold a pointer to the polygon and an index into its
//! `std::vector<LinkedPoint>`. The algorithms keep several of them alive while they change the same
//! polygon (it0/it1/it2 and a `range`), so here an iterator is the index alone (`Iterator`,
//! `ConstIterator`, both `Copy`) and every method that reads or changes the polygon takes it as
//! `data` (the C++ member `m_data`). Names, order and arithmetic are the C++'s.
//!
//! C++ overloads that share a name are, past the first, a function of the same name in a module
//! `<name>_<what>_overload` (free functions) or a method of the small traits next to the struct
//! (`MutablePolygonConst`, `MutablePolygonRange`, `MutablePolygonIndex`); the earlier names
//! (`remove_duplicates_eps`, `smooth_outward_polygons`, `assign_points`, `polygon_into`, ...) stay as
//! aliases.
//!
//! Gate `MUTABLE_POLYGON_PIN` (faithful, on unless `=0`), three things of `smooth_outward` /
//! `clip_narrow_corner` that the earlier port did differently:
//! - the tolerance of the `remove_duplicates` at the start of `smooth_outward` is
//!   `scaled<double>(0.01)`, the double `0.01 / SCALING_FACTOR` (Point.hpp:536-541), not the
//!   truncated coordinate 999;
//! - the two discriminants `u = b * b - 4. * a * (...)` (MutablePolygon.cpp:238, :322) are one
//!   expression, which clang (arm64, `-ffp-contract=on`) compiles to `fma(b, b, -(4. * a * (...)))`;
//! - `.cast<coord_t>()` of a `Vec2d` is a conversion to `int32_t` (libslic3r.h:40), saturating on
//!   arm64.
//! The pin is a release build: its `assert`s are compiled out, so none of them is in the control flow
//! here either (where one would fire, the pin goes on with the value it has: a negative discriminant
//! gives a NaN `t`, which converts to a zero displacement).
//!
//! Not reproduced: `Point` is two `int32_t` at the pin and two `i64` here, so the sums and differences
//! of points that the pin forms before `.cast<int64_t>()` do not wrap at 32 bits. Coordinates of a
//! polygon on the bed are far inside that range.

use crate::geometry::{ExPolygons, Point, Polygon, Polygons};
use crate::libslic3r::{SCALED_EPSILON, SCALING_FACTOR};
use crate::Coord;

// MutablePolygon.hpp:17 — using IndexType = int32_t;
pub type IndexType = i32;

/// The gate of this file, see the module comment.
#[inline]
fn mutable_polygon_pin() -> bool {
    crate::faithful_gate("MUTABLE_POLYGON_PIN")
}

// libslic3r.h:275 — template<typename T> constexpr inline T sqr(T x) { return x * x; }
#[inline]
fn sqr_i64(x: i64) -> i64 {
    x.wrapping_mul(x)
}
#[inline]
fn sqr_f64(x: f64) -> f64 {
    x * x
}

// ---------------------------------------------------------------------------
// Eigen `Vec2i64` / `Vec2d` (Point.hpp:46-58) as far as this file uses them. The pin widens point
// coordinates to int64 with `.cast<int64_t>()` before it forms squared norms, dot and cross products;
// the products are int64 (they wrap as the machine does, which no polygon on the bed reaches). A
// `Vec2d` product sum `x*x' + y*y'` of Eigen is not fused: the product and the sum are separate
// functors.
// ---------------------------------------------------------------------------

/// Eigen `Vec2i64` (Point.hpp:48).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Vec2i64 {
    pub x: i64,
    pub y: i64,
}
impl Vec2i64 {
    #[inline]
    fn squared_norm(self) -> i64 {
        self.x.wrapping_mul(self.x).wrapping_add(self.y.wrapping_mul(self.y))
    }
    #[inline]
    fn dot(self, o: Vec2i64) -> i64 {
        self.x.wrapping_mul(o.x).wrapping_add(self.y.wrapping_mul(o.y))
    }
    #[inline]
    fn cast_f64(self) -> Vec2d {
        Vec2d { x: self.x as f64, y: self.y as f64 }
    }
}
impl std::ops::Sub for Vec2i64 {
    type Output = Vec2i64;
    #[inline]
    fn sub(self, o: Vec2i64) -> Vec2i64 {
        Vec2i64 { x: self.x.wrapping_sub(o.x), y: self.y.wrapping_sub(o.y) }
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
    // Eigen `.cast<coord_t>()`: a static_cast of each double. `coord_t` is `int32_t` at the pin
    // (libslic3r.h:40), the conversion truncates toward zero and saturates on arm64 (NaN gives 0);
    // with the gate off the earlier conversion to 64 bits.
    #[inline]
    fn cast_coord(self, pin: bool) -> Point {
        if pin {
            Point::new(self.x as i32 as Coord, self.y as i32 as Coord)
        } else {
            Point::new(self.x as Coord, self.y as Coord)
        }
    }
}
impl std::ops::Mul<f64> for Vec2d {
    type Output = Vec2d;
    #[inline]
    fn mul(self, t: f64) -> Vec2d {
        Vec2d { x: self.x * t, y: self.y * t }
    }
}

// `Point::cast<int64_t>()`
#[inline]
fn cast_i64(p: Point) -> Vec2i64 {
    Vec2i64 { x: p.x, y: p.y }
}

// `(a - b).cast<int64_t>()` of two Points.
#[inline]
fn diff_i64(a: Point, b: Point) -> Vec2i64 {
    Vec2i64 { x: a.x.wrapping_sub(b.x), y: a.y.wrapping_sub(b.y) }
}

// Point.hpp:85-89 — template<typename T> inline T cross2(v1, v2) { return v1.x() * v2.y() - v1.y() * v2.x(); }
// over `Vec2i64`: an int64.
#[inline]
fn cross2(v1: Vec2i64, v2: Vec2i64) -> i64 {
    v1.x.wrapping_mul(v2.y).wrapping_sub(v1.y.wrapping_mul(v2.x))
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
#[derive(Clone)]
pub struct MutablePolygon {
    m_data: Vec<LinkedPoint>,
    // Number of points in the linked list.
    m_size: IndexType,
    m_head: IndexType,
    // Head of the free list.
    m_head_free: IndexType,
}

// MutablePolygon.hpp:128 — MutablePolygon() = default; (m_size { 0 }, m_head { -1 }, m_head_free { -1 })
impl Default for MutablePolygon {
    fn default() -> Self {
        MutablePolygon::new()
    }
}

// MutablePolygon.hpp:19-39 — class const_iterator
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct ConstIterator {
    m_idx: IndexType,
}

impl ConstIterator {
    // MutablePolygon.hpp:21 — bool operator==(const const_iterator &rhs) const { ... return m_idx == rhs.m_idx; }
    // MutablePolygon.hpp:22 — operator!= : the derived `PartialEq`.

    // MutablePolygon.hpp:23 — const_iterator& operator--() { m_idx = m_data->at(m_idx).prev; return *this; }
    #[inline]
    pub fn dec(&mut self, data: &MutablePolygon) -> ConstIterator {
        self.m_idx = data.at(self.m_idx).prev;
        *self
    }
    // MutablePolygon.hpp:25 — const_iterator& operator++() { m_idx = m_data->at(m_idx).next; return *this; }
    #[inline]
    pub fn inc(&mut self, data: &MutablePolygon) -> ConstIterator {
        self.m_idx = data.at(self.m_idx).next;
        *self
    }
    // MutablePolygon.hpp:27 — const_iterator prev() const { return { m_data, m_data->at(m_idx).prev }; }
    #[inline]
    pub fn prev(&self, data: &MutablePolygon) -> ConstIterator {
        ConstIterator { m_idx: data.at(self.m_idx).prev }
    }
    // MutablePolygon.hpp:28 — const_iterator next() const { return { m_data, m_data->at(m_idx).next }; }
    #[inline]
    pub fn next(&self, data: &MutablePolygon) -> ConstIterator {
        ConstIterator { m_idx: data.at(self.m_idx).next }
    }
    // MutablePolygon.hpp:29 — bool valid() const { return m_idx >= 0; }
    #[inline]
    pub fn valid(&self) -> bool {
        self.m_idx >= 0
    }
    // MutablePolygon.hpp:30-31 — const PointType& operator*() const { return m_data->at(m_idx).point; }
    #[inline]
    pub fn point<'a>(&self, data: &'a MutablePolygon) -> &'a Point {
        &data.at(self.m_idx).point
    }
    // MutablePolygon.hpp:33 — IndexType size() const { return m_data->size(); }
    #[inline]
    pub fn size(&self, data: &MutablePolygon) -> IndexType {
        data.size() as IndexType
    }
    /// The index into the polygon's storage (`m_idx`), for tests.
    #[inline]
    pub fn idx(&self) -> IndexType {
        self.m_idx
    }
}

// MutablePolygon.hpp:41-64 — class iterator
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Iterator {
    m_idx: IndexType,
}

/// The earlier name of [`Iterator`].
pub type Iter = Iterator;

impl Iterator {
    // MutablePolygon.hpp:43 — bool operator==(const iterator &rhs) const { ... return m_idx == rhs.m_idx; }
    // MutablePolygon.hpp:44 — operator!= : the derived `PartialEq`.

    // MutablePolygon.hpp:45 — iterator& operator--() { m_idx = m_data->at(m_idx).prev; return *this; }
    #[inline]
    pub fn dec(&mut self, data: &MutablePolygon) -> Iterator {
        self.m_idx = data.at(self.m_idx).prev;
        *self
    }
    // MutablePolygon.hpp:47 — iterator& operator++() { m_idx = m_data->at(m_idx).next; return *this; }
    #[inline]
    pub fn inc(&mut self, data: &MutablePolygon) -> Iterator {
        self.m_idx = data.at(self.m_idx).next;
        *self
    }
    // MutablePolygon.hpp:49 — iterator prev() const { return { m_data, m_data->at(m_idx).prev }; }
    #[inline]
    pub fn prev(&self, data: &MutablePolygon) -> Iterator {
        Iterator { m_idx: data.at(self.m_idx).prev }
    }
    // MutablePolygon.hpp:50 — iterator next() const { return { m_data, m_data->at(m_idx).next }; }
    #[inline]
    pub fn next(&self, data: &MutablePolygon) -> Iterator {
        Iterator { m_idx: data.at(self.m_idx).next }
    }
    // MutablePolygon.hpp:51 — bool valid() const { return m_idx >= 0; }
    #[inline]
    pub fn valid(&self) -> bool {
        self.m_idx >= 0
    }
    // MutablePolygon.hpp:52-53 — PointType& operator*() const { return m_data->at(m_idx).point; }
    #[inline]
    pub fn point<'a>(&self, data: &'a MutablePolygon) -> &'a Point {
        &data.at(self.m_idx).point
    }
    // MutablePolygon.hpp:52-53, the same reference for writing (`*it += ...`).
    #[inline]
    pub fn point_mut<'a>(&self, data: &'a mut MutablePolygon) -> &'a mut Point {
        &mut data.at_mut(self.m_idx).point
    }
    // MutablePolygon.hpp:55 — IndexType size() const { return m_data->size(); }
    #[inline]
    pub fn size(&self, data: &MutablePolygon) -> IndexType {
        data.size() as IndexType
    }
    // MutablePolygon.hpp:56 — iterator& remove() { m_idx = m_data->remove(*this).m_idx; return *this; }
    #[inline]
    pub fn remove(&mut self, data: &mut MutablePolygon) -> Iterator {
        self.m_idx = data.remove(*self).m_idx;
        *self
    }
    // MutablePolygon.hpp:57 — iterator insert(const PointType pt) const { return m_data->insert(*this, pt); }
    #[inline]
    pub fn insert(&self, data: &mut MutablePolygon, pt: Point) -> Iterator {
        data.insert(*self, pt)
    }
    /// The index into the polygon's storage (`m_idx`), for tests.
    #[inline]
    pub fn idx(&self) -> IndexType {
        self.m_idx
    }
}

impl MutablePolygon {
    // MutablePolygon.hpp:128 — MutablePolygon() = default;
    pub fn new() -> Self {
        Self { m_data: Vec::new(), m_size: 0, m_head: -1, m_head_free: -1 }
    }

    // MutablePolygon.hpp:129 — MutablePolygon(const Polygon &rhs, size_t reserve = 0)
    pub fn from_polygon(rhs: &Polygon, reserve: usize) -> Self {
        MutablePolygon::from_points(&rhs.points, reserve)
    }

    // MutablePolygon.hpp:130-135 — MutablePolygon(std::initializer_list<Point> rhs, size_t reserve = 0) and
    // template<typename IT> MutablePolygon(IT begin, IT end, size_t reserve = 0)
    pub fn from_points(points: &[Point], reserve: usize) -> Self {
        let mut out = MutablePolygon::new();
        out.assign_inner(points, reserve);
        out
    }

    /// The earlier name of [`MutablePolygonRange::assign`] (MutablePolygon.hpp:137-143).
    pub fn assign_points(&mut self, points: &[Point], reserve: usize) {
        MutablePolygonRange::assign(self, points, reserve);
    }

    // MutablePolygon.hpp:145-147 — void assign(const Polygon &rhs, size_t reserve = 0)
    pub fn assign(&mut self, rhs: &Polygon, reserve: usize) {
        MutablePolygonRange::assign(self, &rhs.points, reserve);
    }

    /// The earlier name of [`MutablePolygonRange::polygon`] (MutablePolygon.hpp:149-158).
    pub fn polygon_into(&self, out: &mut Polygon) {
        MutablePolygonRange::polygon(self, out);
    }

    // MutablePolygon.hpp:160-164 — Polygon polygon() const
    pub fn polygon(&self) -> Polygon {
        let mut out = Polygon::new();
        MutablePolygonRange::polygon(self, &mut out);
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
    // (The number is the allocator's: `Vec` and libc++'s `std::vector` grow differently.)
    #[inline]
    pub fn capacity(&self) -> usize {
        self.m_data.capacity()
    }
    // MutablePolygon.hpp:169 — bool valid() const { return m_size >= 3; }
    #[inline]
    pub fn valid(&self) -> bool {
        self.m_size >= 3
    }
    // MutablePolygon.hpp:170 — void clear() { m_data.clear(); m_size = 0; m_head = IndexType(-1); m_head_free = IndexType(-1); }
    #[inline]
    pub fn clear(&mut self) {
        self.m_data.clear();
        self.m_size = 0;
        self.m_head = -1;
        self.m_head_free = -1;
    }

    // MutablePolygon.hpp:172 — iterator begin() { return { this, m_head }; }
    #[inline]
    pub fn begin(&self) -> Iterator {
        Iterator { m_idx: self.m_head }
    }
    // MutablePolygon.hpp:173 — const_iterator cbegin() const { return { this, m_head }; }
    #[inline]
    pub fn cbegin(&self) -> ConstIterator {
        ConstIterator { m_idx: self.m_head }
    }
    // MutablePolygon.hpp:176 — iterator end() { return { this, this->empty() ? -1 : this->at(m_head).prev }; }
    // End points to the last item before roll over. This is different from the usual end() concept!
    #[inline]
    pub fn end(&self) -> Iterator {
        Iterator { m_idx: if self.empty() { -1 } else { self.at(self.m_head).prev } }
    }
    // MutablePolygon.hpp:177 — const_iterator cend() const { return { this, this->empty() ? -1 : this->at(m_head).prev }; }
    #[inline]
    pub fn cend(&self) -> ConstIterator {
        ConstIterator { m_idx: if self.empty() { -1 } else { self.at(self.m_head).prev } }
    }

    // MutablePolygon.hpp:182 — iterator remove(const iterator it) { return { this, this->remove(it.m_idx) }; }
    // Returns iterator following the removed element. Returned iterator will become invalid if last point is removed.
    // If begin() is removed, then the next element will become the new begin().
    #[inline]
    pub fn remove(&mut self, it: Iterator) -> Iterator {
        Iterator { m_idx: MutablePolygonIndex::remove(self, it.m_idx) }
    }
    // MutablePolygon.hpp:185 — iterator insert(const iterator it, const PointType pt) { return { this, this->insert(it.m_idx, pt) }; }
    // Insert a new point before it. Returns iterator to the newly inserted point.
    // begin() will not change, end() may point to the newly inserted point.
    #[inline]
    pub fn insert(&mut self, it: Iterator, pt: Point) -> Iterator {
        Iterator { m_idx: MutablePolygonIndex::insert(self, it.m_idx, pt) }
    }

    // MutablePolygon.hpp:203 — LinkedPoint& at(IndexType i) { return m_data[i]; }
    #[inline]
    fn at_mut(&mut self, i: IndexType) -> &mut LinkedPoint {
        &mut self.m_data[i as usize]
    }
    // MutablePolygon.hpp:204 — const LinkedPoint& at(IndexType i) const { return m_data[i]; }
    #[inline]
    fn at(&self, i: IndexType) -> &LinkedPoint {
        &self.m_data[i as usize]
    }

    // MutablePolygon.hpp:206-219 — template<typename IT> void assign_inner(IT begin, IT end, size_t reserve)
    fn assign_inner(&mut self, points: &[Point], reserve: usize) {
        self.m_size = points.len() as IndexType;
        if self.m_size > 0 {
            self.m_head = 0;
            self.m_data.reserve(std::cmp::max(self.m_size as usize, reserve));
            let mut i: IndexType = -1;
            let mut j: IndexType = 1;
            for it in points {
                self.m_data.push(LinkedPoint { point: *it, prev: i, next: j });
                i += 1;
                j += 1;
            }
            let last = (self.m_size - 1) as usize;
            self.m_data[0].prev = self.m_size - 1;
            self.m_data[last].next = 0;
        }
    }

    /// The members, for tests: `m_data` as (point, prev, next), `m_size`, `m_head`, `m_head_free`.
    pub fn members(&self) -> (Vec<(Point, IndexType, IndexType)>, IndexType, IndexType, IndexType) {
        (self.m_data.iter().map(|lp| (lp.point, lp.prev, lp.next)).collect(), self.m_size, self.m_head, self.m_head_free)
    }
}

/// The `const` overloads of `begin()` and `end()` (MutablePolygon.hpp:174, :178).
pub trait MutablePolygonConst {
    fn begin(&self) -> ConstIterator;
    fn end(&self) -> ConstIterator;
}

impl MutablePolygonConst for MutablePolygon {
    // MutablePolygon.hpp:174 — const_iterator begin() const { return this->cbegin(); }
    #[inline]
    fn begin(&self) -> ConstIterator {
        self.cbegin()
    }
    // MutablePolygon.hpp:178 — const_iterator end() const { return this->cend(); }
    #[inline]
    fn end(&self) -> ConstIterator {
        self.cend()
    }
}

/// The overloads of `assign` and `polygon` over a range of points and an output polygon
/// (MutablePolygon.hpp:137-143, :149-158).
pub trait MutablePolygonRange {
    fn assign(&mut self, points: &[Point], reserve: usize);
    fn polygon(&self, out: &mut Polygon);
}

impl MutablePolygonRange for MutablePolygon {
    // MutablePolygon.hpp:137-143 — template<typename IT> void assign(IT begin, IT end, size_t reserve = 0)
    fn assign(&mut self, points: &[Point], reserve: usize) {
        self.m_data.clear();
        self.m_head = -1;
        self.m_head_free = -1;
        self.assign_inner(points, reserve);
    }

    // MutablePolygon.hpp:149-158 — void polygon(Polygon &out) const
    fn polygon(&self, out: &mut Polygon) {
        out.points.clear();
        if self.valid() {
            out.points.reserve(self.size());
            let mut it = self.cbegin();
            out.points.push(*it.point(self));
            // for (++ it; it != this->cbegin(); ++ it)
            it.inc(self);
            while it != self.cbegin() {
                out.points.push(*it.point(self));
                it.inc(self);
            }
        }
    }
}

/// The private overloads of `remove` and `insert` over an index (MutablePolygon.hpp:221-260).
trait MutablePolygonIndex {
    fn remove(&mut self, i: IndexType) -> IndexType;
    fn insert(&mut self, i: IndexType, pt: Point) -> IndexType;
}

impl MutablePolygonIndex for MutablePolygon {
    // MutablePolygon.hpp:221-240 — IndexType remove(const IndexType i)
    fn remove(&mut self, i: IndexType) -> IndexType {
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
        if self.empty() {
            return -1;
        }
        self.at_mut(prev).next = next;
        self.at_mut(next).prev = prev;
        next
    }

    // MutablePolygon.hpp:242-260 — IndexType insert(const IndexType i, const Point pt)
    fn insert(&mut self, i: IndexType, pt: Point) -> IndexType {
        let n: IndexType;
        let j = self.at(i).prev;
        if self.m_head_free == -1 {
            // Allocate a new item.
            n = self.m_data.len() as IndexType;
            self.m_data.push(LinkedPoint { point: pt, prev: j, next: i });
        } else {
            n = self.m_head_free;
            self.m_head_free = self.at(n).next;
            *self.at_mut(n) = LinkedPoint { point: pt, prev: j, next: i };
        }
        self.at_mut(j).next = n;
        self.at_mut(i).prev = n;
        self.m_size += 1;
        n
    }
}

// MutablePolygon.hpp:66-126 — class range
//
// Iterator range for maintaining a range of unprocessed items, see smooth_outward().
// Range from begin to end, inclusive. If the range is empty, then both m_begin and m_end are invalid.
#[derive(Clone, Copy, Debug)]
pub struct Range {
    m_begin: Iterator,
    m_end: Iterator,
}

impl Range {
    // MutablePolygon.hpp:70 — range(MutablePolygon& poly) : range(poly.begin(), poly.end()) {}
    pub fn new(poly: &MutablePolygon) -> Self {
        Range::from_iterators(poly.begin(), poly.end())
    }
    // MutablePolygon.hpp:71 — range(MutablePolygon::iterator begin, MutablePolygon::iterator end)
    pub fn from_iterators(begin: Iterator, end: Iterator) -> Self {
        Range { m_begin: begin, m_end: end }
    }

    // MutablePolygon.hpp:74 — MutablePolygon::iterator begin() const { return m_begin; }
    // Start of a range, inclusive. If range is empty, then ! begin().valid().
    pub fn begin(&self) -> Iterator {
        self.m_begin
    }
    // MutablePolygon.hpp:76 — MutablePolygon::iterator end() const { return m_end; }
    // End of a range, inclusive. If range is empty, then ! end().valid().
    pub fn end(&self) -> Iterator {
        self.m_end
    }
    // MutablePolygon.hpp:78 — bool empty() const { return !m_begin.valid(); }
    pub fn empty(&self) -> bool {
        !self.m_begin.valid()
    }

    // MutablePolygon.hpp:81-86 — MutablePolygon::iterator process_next()
    // Return begin() and shorten the range by advancing front.
    pub fn process_next(&mut self, data: &MutablePolygon) -> Iterator {
        let out = self.m_begin;
        self.advance_front(data);
        out
    }

    // MutablePolygon.hpp:88-94 — void advance_front()
    pub fn advance_front(&mut self, data: &MutablePolygon) {
        if self.m_begin == self.m_end {
            self.make_empty();
        } else {
            self.m_begin.inc(data);
        }
    }

    // MutablePolygon.hpp:96-102 — void retract_back()
    pub fn retract_back(&mut self, data: &MutablePolygon) {
        if self.m_begin == self.m_end {
            self.make_empty();
        } else {
            self.m_end.dec(data);
        }
    }

    // MutablePolygon.hpp:104-108 — MutablePolygon::iterator remove_front(MutablePolygon::iterator it)
    pub fn remove_front(&mut self, data: &mut MutablePolygon, mut it: Iterator) -> Iterator {
        if !self.empty() && self.m_begin == it {
            self.advance_front(data);
        }
        it.remove(data)
    }

    // MutablePolygon.hpp:110-114 — MutablePolygon::iterator remove_back(MutablePolygon::iterator it)
    pub fn remove_back(&mut self, data: &mut MutablePolygon, mut it: Iterator) -> Iterator {
        if !self.empty() && self.m_end == it {
            self.retract_back(data);
        }
        it.remove(data)
    }

    // MutablePolygon.hpp:122-125 — void make_empty()
    fn make_empty(&mut self) {
        self.m_begin.m_idx = -1;
        self.m_end.m_idx = -1;
    }
}

// MutablePolygon.hpp:288-304 — inline bool operator==(const MutablePolygon &p1, const MutablePolygon &p2)
// MutablePolygon.hpp:306 — operator!=
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
            if !(*it.point(p1) == *it2.point(p2)) {
                return false;
            }
            if it.inc(p1) == begin {
                return true;
            }
            it2.inc(p2);
        }
    }
}

// MutablePolygon.cpp:7-21 — void remove_duplicates(MutablePolygon &polygon)
// Remove exact duplicate points. May reduce the polygon down to empty polygon.
pub fn remove_duplicates(polygon: &mut MutablePolygon) {
    if !polygon.empty() {
        let begin = polygon.begin();
        let mut it = begin;
        // for (++ it; it != begin;)
        it.inc(polygon);
        while it != begin {
            let prev = it.prev(polygon);
            if *prev.point(polygon) == *it.point(polygon) {
                it = it.remove(polygon);
            } else {
                it.inc(polygon);
            }
        }
    }
}

/// MutablePolygon.cpp:23-38, the overload of `remove_duplicates` with a tolerance.
pub mod remove_duplicates_eps_overload {
    use super::*;
    // MutablePolygon.cpp:23-38 — void remove_duplicates(MutablePolygon &polygon, double eps)
    // Remove nearly duplicate points. May reduce the polygon down to empty polygon.
    pub fn remove_duplicates(polygon: &mut MutablePolygon, eps: f64) {
        if !polygon.empty() {
            let eps2 = eps * eps;
            let begin = polygon.begin();
            let mut it = begin;
            // for (++ it; it != begin;)
            it.inc(polygon);
            while it != begin {
                let prev = it.prev(polygon);
                // if ((*it - *prev).cast<double>().squaredNorm() < eps2)
                let d = diff_i64(*it.point(polygon), *prev.point(polygon)).cast_f64();
                if d.squared_norm() < eps2 {
                    it = it.remove(polygon);
                } else {
                    it.inc(polygon);
                }
            }
        }
    }
}
pub use remove_duplicates_eps_overload::remove_duplicates as remove_duplicates_eps;

/// MutablePolygon.cpp:40-65, the overload of `remove_duplicates` with a length and an angle.
pub mod remove_duplicates_angle_overload {
    use super::*;
    // MutablePolygon.cpp:40-65 — void remove_duplicates(MutablePolygon& polygon, coord_t scaled_eps, const double max_angle)
    pub fn remove_duplicates(polygon: &mut MutablePolygon, scaled_eps: Coord, max_angle: f64) {
        if polygon.size() >= 3 {
            let cos_max_angle_2 = sqr_f64(max_angle.cos());
            let scaled_eps_sqr = sqr_i64(scaled_eps);
            let begin = polygon.begin();
            let mut it = begin;
            // for (++it; it != begin;)
            it.inc(polygon);
            while it != begin {
                let prev = it.prev(polygon);
                let next = it.next(polygon);
                let v1 = diff_i64(*it.point(polygon), *prev.point(polygon));
                let v1_sqr_norm = v1.squared_norm();
                if v1_sqr_norm < scaled_eps_sqr {
                    let v2 = diff_i64(*next.point(polygon), *prev.point(polygon));
                    if sqr_f64(v1.dot(v2) as f64) > cos_max_angle_2 * (v1_sqr_norm as f64) * (v2.squared_norm() as f64) {
                        it = it.remove(polygon);
                        continue;
                    }
                }
                it = next;
            }
        }

        if polygon.size() < 3 {
            polygon.clear();
        }
    }
}
pub use remove_duplicates_angle_overload::remove_duplicates as remove_duplicates_angle;

// MutablePolygon.cpp:87-91 — enum Status of clip_narrow_corner
#[derive(Clone, Copy, PartialEq)]
enum Status {
    Free,
    Blocked,
    Far,
}

// MutablePolygon.cpp:67-255 — static bool clip_narrow_corner(const Vec2i64 p1, MutablePolygon::iterator &it0,
//     MutablePolygon::iterator &it2, MutablePolygon::range &unprocessed_range, int64_t dist2_current,
//     const int64_t shortcut_length)
// (`polygon` is `it0.polygon()` of MutablePolygon.cpp:82.)
//
// Adapted from Cura ConstPolygonRef::smooth_corner_complex() by Tim Kuipers.
// A concave corner at it1 with position p1 has been removed by the caller between it0 and it2, where |p2 - p0| < shortcut_length.
// Now try to close a concave crack by walking left from it0 and right from it2 as long as the new clipping edge is smaller than shortcut_length
// and the new clipping edge is still inside the polygon (it is a diagonal, it does not intersect polygon boundary).
// Once the traversal stops (always at a clipping edge shorter than shortcut_length), the final trapezoid is clipped with a new clipping edge of shortcut_length.
// Return true if a hole was completely closed (degenerated to an empty polygon) or a single CCW triangle was left, which is not to be simplified any further.
// it0, it2 are updated to the final clipping edge.
pub fn clip_narrow_corner(
    p1: Vec2i64,
    it0: &mut Iterator,
    it2: &mut Iterator,
    unprocessed_range: &mut Range,
    mut dist2_current: i64,
    shortcut_length: i64,
    polygon: &mut MutablePolygon,
) -> bool {
    let pin = mutable_polygon_pin();

    let shortcut_length2 = sqr_i64(shortcut_length);

    let mut forward = Status::Free;
    let mut backward = Status::Free;

    let mut p0 = cast_i64(*it0.point(polygon));
    let mut p2 = cast_i64(*it2.point(polygon));
    // Not initialized at the pin; every path that reads them has written them first.
    let mut p02 = Vec2i64 { x: 0, y: 0 };
    let mut p22 = Vec2i64 { x: 0, y: 0 };
    let mut dist2_next: i64 = 0;

    // As long as there is at least a single triangle left in the polygon.
    while polygon.size() >= 3 {
        if forward == Status::Far && backward == Status::Far {
            p02 = cast_i64(*it0.prev(polygon).point(polygon));
            p22 = cast_i64(*it2.next(polygon).point(polygon));
            let d2 = (p22 - p02).squared_norm();
            if d2 <= shortcut_length2 {
                // The region was narrow until now and it is still narrow. Trim at both sides.
                *it0 = unprocessed_range.remove_back(polygon, *it0).prev(polygon);
                *it2 = unprocessed_range.remove_front(polygon, *it2);
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
        if forward == Status::Free && (backward != Status::Free || (p2 - p1).squared_norm() < (p0 - p1).squared_norm()) {
            p22 = cast_i64(*it2.next(polygon).point(polygon));
            if cross2(p2 - p0, p22 - p0) > 0 {
                forward = Status::Blocked;
            } else {
                // New clipping edge lenght.
                let d2 = (p22 - p0).squared_norm();
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
            p02 = cast_i64(*it0.prev(polygon).point(polygon));
            if cross2(p02 - p2, p0 - p2) > 0 {
                backward = Status::Blocked;
            } else {
                // New clipping edge lenght.
                let d2 = (p2 - p02).squared_norm();
                if d2 > shortcut_length2 {
                    backward = Status::Far;
                    dist2_next = d2;
                } else {
                    backward = Status::Free;
                    // Make one step in the backward direction.
                    *it0 = unprocessed_range.remove_back(polygon, *it0).prev(polygon);
                    p0 = p02;
                    dist2_current = d2;
                }
            }
        }
    }

    if polygon.size() <= 3 {
        // A hole degenerated to an empty polygon, or a tiny triangle remained.
        // (MutablePolygon.cpp:178-196 is `#ifndef NDEBUG`.)
        if polygon.size() < 3 || (forward == Status::Far && backward == Status::Far) {
            polygon.clear();
        } else {
            // The remaining triangle is CCW oriented, keep it.
        }
        return true;
    }

    // SCALED_EPSILON is the double `scale_(EPSILON)`; `int64_t(SCALED_EPSILON)` is 10.
    if (forward == Status::Blocked && backward == Status::Blocked) || dist2_current > sqr_i64(shortcut_length - SCALED_EPSILON as i64) {
        // The crack is filled, keep the last clipping edge.
    } else if dist2_next < sqr_i64(shortcut_length - SCALED_EPSILON as i64) {
        // To avoid creating tiny edges.
        if forward == Status::Far {
            *it0 = unprocessed_range.remove_back(polygon, *it0).prev(polygon);
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
        if forward == Status::Far {
            // Sort, so we will clip the 1st edge.
            std::mem::swap(&mut p0, &mut p2);
            std::mem::swap(&mut p02, &mut p22);
        }
        // Find point on (p0, p02) at distance shortcut_length from p2.
        // Circle intersects a line at two points, however because |p2 - p0| < shortcut_length,
        // only the second intersection is valid. Because |p2 - p02| > shortcut_length, such
        // intersection should always be found on (p0, p02).
        let v = (p02 - p0).cast_f64();
        let d = (p0 - p2).cast_f64();
        let a = v.squared_norm();
        let b = 2. * d.dot(v);
        // double u = b * b - 4. * a * (d.squaredNorm() - shortcut_length2); one expression: fma(b, b, -(...))
        let c = 4. * a * (d.squared_norm() - shortcut_length2 as f64);
        let mut u = if pin { b.mul_add(b, -c) } else { b * b - c };
        u = u.sqrt();
        let t = (-b + u) / (2. * a);
        // (backward == Far ? *it2 : *it0) += (v.cast<double>() * t).cast<coord_t>();
        let delta = (v * t).cast_coord(pin);
        let target = if backward == Status::Far { *it2 } else { *it0 };
        *target.point_mut(polygon) += delta;
    } else {
        // The trapezoid (it0.prev(), it0, it2, it2.next()) is widening. Trim it.
        let dcurrent = (dist2_current as f64).sqrt();
        let t = (shortcut_length as f64 - dcurrent) / ((dist2_next as f64).sqrt() - dcurrent);
        let d0 = ((p02 - p0).cast_f64() * t).cast_coord(pin);
        *it0.point_mut(polygon) += d0;
        let d2 = ((p22 - p2).cast_f64() * t).cast_coord(pin);
        *it2.point_mut(polygon) += d2;
    }
    false
}

// MutablePolygon.cpp:257-385 — void smooth_outward(MutablePolygon &polygon, coord_t clip_dist_scaled)
// adapted from Cura ConstPolygonRef::smooth_outward() by Tim Kuipers.
pub fn smooth_outward(polygon: &mut MutablePolygon, clip_dist_scaled: Coord) {
    let pin = mutable_polygon_pin();

    // remove_duplicates(polygon, scaled<double>(0.01)); the double 0.01 / SCALING_FACTOR (Point.hpp:536-541)
    remove_duplicates_eps(polygon, if pin { 0.01 / SCALING_FACTOR } else { crate::scaled(0.01) as f64 });

    let clip_dist_scaled2 = sqr_i64(clip_dist_scaled);
    let clip_dist_scaled2eps = sqr_i64(clip_dist_scaled + SCALED_EPSILON as i64);
    // sqr(SCALED_EPSILON): a double
    let foot_dist_min2 = sqr_f64(SCALED_EPSILON);

    // Each source point will be visited exactly once.
    let mut unprocessed_range = Range::new(polygon);
    while !unprocessed_range.empty() && polygon.size() > 2 {
        let mut it1 = unprocessed_range.process_next(polygon);
        let mut it0 = it1.prev(polygon);
        let mut it2 = it1.next(polygon);
        let p0 = *it0.point(polygon);
        let p1 = *it1.point(polygon);
        let p2 = *it2.point(polygon);
        let v1 = diff_i64(p0, p1);
        let v2 = diff_i64(p2, p1);
        if cross2(v1, v2) > 0 {
            // Concave corner.
            let dot = v1.dot(v2);
            let mut l2v1 = v1.squared_norm() as f64;
            let mut l2v2 = v2.squared_norm() as f64;
            if dot > 0 || sqr_f64(dot as f64) * 2. < l2v1 * l2v2 {
                // Angle between v1 and v2 bigger than 135 degrees.
                // Simplify the sharp angle.
                let v02 = diff_i64(p2, p0);
                let l2v02 = v02.squared_norm();
                it1.remove(polygon);
                if l2v02 < clip_dist_scaled2 {
                    // (p0, p2) is short.
                    // Clip a sharp concave corner by possibly expanding the trimming region left of it0 and right of it2.
                    // Updates it0, it2 and num_to_process.
                    if clip_narrow_corner(cast_i64(p1), &mut it0, &mut it2, &mut unprocessed_range, l2v02, clip_dist_scaled, polygon) {
                        // Trimmed down to an empty polygon or to a single CCW triangle.
                        return;
                    }
                } else {
                    // Clip an obtuse corner.
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
                        let bisector = Vec2d { x: v1d.x / lv1 + v2d.x / lv2, y: v1d.y / lv1 + v2d.y / lv2 };
                        let l2bisector = bisector.squared_norm();
                        // Squared distance of the end point of v1 to the bisector.
                        let d2 = l2v1 - sqr_f64(v1d.dot(bisector)) / l2bisector;
                        if d2 < foot_dist_min2 {
                            // Height of the p1, p0, p2 triangle is tiny. Just remove p1.
                        } else if d2 < 0.25 * clip_dist_scaled2 as f64 + SCALED_EPSILON {
                            // The shorter vector is too close to the bisector. Trim the shorter vector fully,
                            // trim the longer vector partially.
                            // Intersection of a circle at p2 of radius = clip_dist_scaled
                            // with a ray (p1, p0), take the intersection after the foot point.
                            // The intersection shall always exist because |p2 - p1| > clip_dist_scaled.
                            let b = -2. * v1d.dot(v2d);
                            // double u = b * b - 4. * l2v2 * (double(l2v1) - clip_dist_scaled2); one expression: fma(b, b, -(...))
                            let c = 4. * l2v2 * (l2v1 - clip_dist_scaled2 as f64);
                            let u = if pin { b.mul_add(b, -c) } else { b * b - c };
                            // Take the second intersection along v2.
                            let t = (-b + u.sqrt()) / (2. * l2v2);
                            // Point pt_new = p1 + (t * v2d).cast<coord_t>();
                            let pt_new = p1 + (v2d * t).cast_coord(pin);
                            // (MutablePolygon.cpp:328-331 is `#ifndef NDEBUG`.)
                            it2.insert(polygon, pt_new);
                        } else {
                            // Cut the corner with a line perpendicular to the bisector.
                            let t = (0.25 * clip_dist_scaled2 as f64 / d2).sqrt();
                            let t2 = t * lv1 / lv2;
                            let mut p0 = p1 + (v1d * t).cast_coord(pin);
                            let mut p2 = p1 + (v2d * t2).cast_coord(pin);
                            if swap {
                                std::mem::swap(&mut p0, &mut p2);
                            }
                            it2.insert(polygon, p2).insert(polygon, p0);
                        }
                    } else {
                        // Just remove p1.
                    }
                }
                it1 = it2;
            } else {
                it1.inc(polygon);
            }
        } else {
            it1.inc(polygon);
        }
        // `it1` is a local of the loop body at the pin too: its last value is not read.
        let _ = it1;
    }

    if polygon.size() == 3 {
        // Check whether the last triangle is clockwise oriented (it is a hole) and its height is below clip_dist_scaled.
        // If so, fill in the hole.
        let p0 = *polygon.begin().prev(polygon).point(polygon);
        let p1 = *polygon.begin().point(polygon);
        let p2 = *polygon.begin().next(polygon).point(polygon);
        let mut v1 = diff_i64(p0, p1);
        let mut v2 = diff_i64(p2, p1);
        if cross2(v1, v2) > 0 {
            // CW triangle. Measure its height.
            let v3 = diff_i64(p2, p0);
            let mut l12 = v1.squared_norm();
            let mut l22 = v2.squared_norm();
            let l32 = v3.squared_norm();
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
        polygon.clear();
    }
}

/// MutablePolygon.hpp:312-328, the overload of `remove_duplicates` over ExPolygons.
pub mod remove_duplicates_expolygons_overload {
    use super::*;
    // MutablePolygon.hpp:312-328 — inline ExPolygons remove_duplicates(ExPolygons expolygons, coord_t scaled_eps, double max_angle)
    pub fn remove_duplicates(mut expolygons: ExPolygons, scaled_eps: Coord, max_angle: f64) -> ExPolygons {
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
        // `ExPolygon::empty()` is `contour.points.empty()` (ExPolygon.hpp).
        expolygons.retain(|p| !p.contour.points.is_empty());
        expolygons
    }
}
pub use remove_duplicates_expolygons_overload::remove_duplicates as remove_duplicates_expolygons;

/// MutablePolygon.hpp:332-338, the overload of `smooth_outward` over a Polygon.
pub mod smooth_outward_polygon_overload {
    use super::*;
    // MutablePolygon.hpp:332-338 — inline Polygon smooth_outward(Polygon polygon, coord_t clip_dist_scaled)
    pub fn smooth_outward(mut polygon: Polygon, clip_dist_scaled: Coord) -> Polygon {
        let mut mp = MutablePolygon::from_polygon(&polygon, polygon.points.len() * 2);
        super::smooth_outward(&mut mp, clip_dist_scaled);
        mp.polygon_into(&mut polygon);
        polygon
    }
}
pub use smooth_outward_polygon_overload::smooth_outward as smooth_outward_polygon;

/// MutablePolygon.hpp:340-350, the overload of `smooth_outward` over Polygons.
pub mod smooth_outward_polygons_overload {
    use super::*;
    // MutablePolygon.hpp:340-350 — inline Polygons smooth_outward(Polygons polygons, coord_t clip_dist_scaled)
    pub fn smooth_outward(mut polygons: Polygons, clip_dist_scaled: Coord) -> Polygons {
        let mut mp = MutablePolygon::new();
        for polygon in polygons.iter_mut() {
            mp.assign(polygon, polygon.points.len() * 2);
            super::smooth_outward(&mut mp, clip_dist_scaled);
            mp.polygon_into(polygon);
        }
        polygons.retain(|p| !p.points.is_empty());
        polygons
    }
}
pub use smooth_outward_polygons_overload::smooth_outward as smooth_outward_polygons;

/// MutablePolygon.hpp:352-368, the overload of `smooth_outward` over ExPolygons.
pub mod smooth_outward_expolygons_overload {
    use super::*;
    // MutablePolygon.hpp:352-368 — inline ExPolygons smooth_outward(ExPolygons expolygons, coord_t clip_dist_scaled)
    pub fn smooth_outward(mut expolygons: ExPolygons, clip_dist_scaled: Coord) -> ExPolygons {
        let mut mp = MutablePolygon::new();
        for expolygon in expolygons.iter_mut() {
            mp.assign(&expolygon.contour, expolygon.contour.points.len() * 2);
            super::smooth_outward(&mut mp, clip_dist_scaled);
            mp.polygon_into(&mut expolygon.contour);
            for hole in expolygon.holes.iter_mut() {
                mp.assign(hole, hole.points.len() * 2);
                super::smooth_outward(&mut mp, clip_dist_scaled);
                mp.polygon_into(hole);
            }
            expolygon.holes.retain(|p| !p.points.is_empty());
        }
        expolygons.retain(|p| !p.contour.points.is_empty());
        expolygons
    }
}
pub use smooth_outward_expolygons_overload::smooth_outward as smooth_outward_expolygons;
