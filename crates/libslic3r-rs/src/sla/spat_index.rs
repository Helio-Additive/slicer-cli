//! Faithful port of libslic3r/SLA/SpatIndex.{hpp,cpp}
//!
//! C++ Reference:
//! - SLA/SpatIndex.hpp
//! - SLA/SpatIndex.cpp
//!
//! NOTE on the backing store: the C++ implementation wraps a
//! `boost::geometry::index::rtree<_, rstar<16, 4>>` (SpatIndex.cpp:24-30 and
//! SpatIndex.cpp:97-103). Boost is a native header-only dependency that we do
//! not pull in; the Rust port stores the elements in a flat `Vec` and answers
//! the same queries by scanning. The supported query semantics (k-nearest by
//! Euclidean distance, arbitrary predicate filtering, box intersects/within)
//! are identical; only the iteration/result order of tied elements may differ
//! from the rtree's internal order, which the C++ API leaves unspecified
//! anyway.

use crate::bounding_box::BoundingBox;
use crate::geometry::Vec3d;

/// SpatIndex.hpp:16 — `using PointIndexEl = std::pair<Vec3d, unsigned>;`
pub type PointIndexEl = (Vec3d, u32);

/// SpatIndex.hpp:18 — `class PointIndex`
/// SpatIndex.cpp:24-30 — `class PointIndex::Impl` (boost rtree store)
#[derive(Debug, Clone, Default)]
pub struct PointIndex {
    /// SpatIndex.cpp:29 — `BoostIndex m_store;`
    m_store: Vec<PointIndexEl>,
}

impl PointIndex {
    /// SpatIndex.cpp:32 — `PointIndex::PointIndex(): m_impl(new Impl()) {}`
    pub fn new() -> Self {
        Self { m_store: Vec::new() }
    }

    // SpatIndex.cpp:33-48 — destructor / copy / move special members are
    // covered by Rust's Clone/Drop semantics.

    /// SpatIndex.cpp:50-53 — `void PointIndex::insert(const PointIndexEl &el)`
    pub fn insert(&mut self, el: PointIndexEl) {
        // SpatIndex.cpp:52
        self.m_store.push(el);
    }

    /// SpatIndex.hpp:37-40 — `inline void insert(const Vec3d& v, unsigned idx)`
    #[inline]
    pub fn insert_point(&mut self, v: Vec3d, idx: u32) {
        // SpatIndex.hpp:39
        self.insert((v, idx));
    }

    /// SpatIndex.cpp:55-58 — `bool PointIndex::remove(const PointIndexEl &el)`
    /// The boost rtree removes a single element equal to `el` and returns the
    /// number of removed elements; the C++ wrapper returns `removed == 1`.
    pub fn remove(&mut self, el: &PointIndexEl) -> bool {
        // SpatIndex.cpp:57
        match self.m_store.iter().position(|e| e == el) {
            Some(pos) => {
                self.m_store.remove(pos);
                true
            }
            None => false,
        }
    }

    /// SpatIndex.cpp:60-68 —
    /// `std::vector<PointIndexEl> PointIndex::query(std::function<bool(const PointIndexEl &)>) const`
    pub fn query(&self, fn_: &dyn Fn(&PointIndexEl) -> bool) -> Vec<PointIndexEl> {
        // SpatIndex.cpp:65-66 — bgi::satisfies(fn)
        let mut ret: Vec<PointIndexEl> = Vec::new();
        for el in &self.m_store {
            if fn_(el) {
                ret.push(el.clone());
            }
        }
        ret
    }

    /// SpatIndex.cpp:70-76 —
    /// `std::vector<PointIndexEl> PointIndex::nearest(const Vec3d &el, unsigned k) const`
    /// Returns the `k` elements nearest to `el` by Euclidean distance
    /// (boost `bgi::nearest(el, k)` query).
    pub fn nearest(&self, el: &Vec3d, k: u32) -> Vec<PointIndexEl> {
        // SpatIndex.cpp:73-74
        let mut ret: Vec<PointIndexEl> = self.m_store.clone();
        let dist_sq = |p: &Vec3d| -> f64 {
            let dx = p.x - el.x;
            let dy = p.y - el.y;
            let dz = p.z - el.z;
            dx * dx + dy * dy + dz * dz
        };
        ret.sort_by(|a, b| {
            dist_sq(&a.0)
                .partial_cmp(&dist_sq(&b.0))
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        ret.truncate(k as usize);
        ret
    }

    /// SpatIndex.hpp:44-47 — `std::vector<PointIndexEl> query(const Vec3d &v, unsigned k)` (wrapper)
    #[inline]
    pub fn query_nearest(&self, v: &Vec3d, k: u32) -> Vec<PointIndexEl> {
        // SpatIndex.hpp:46
        self.nearest(v, k)
    }

    /// SpatIndex.cpp:78-81 — `size_t PointIndex::size() const`
    pub fn size(&self) -> usize {
        // SpatIndex.cpp:80
        self.m_store.len()
    }

    /// SpatIndex.hpp:51 — `bool empty() const { return size() == 0; }`
    #[inline]
    pub fn empty(&self) -> bool {
        self.size() == 0
    }

    /// SpatIndex.cpp:83-91 —
    /// `void PointIndex::foreach(std::function<void(const PointIndexEl &)> fn)`
    /// The C++ const and non-const overloads both pass each element to `fn`
    /// by const reference; a single method suffices in Rust.
    pub fn foreach(&self, fn_: &mut dyn FnMut(&PointIndexEl)) {
        // SpatIndex.cpp:85 / SpatIndex.cpp:90
        for el in &self.m_store {
            fn_(el);
        }
    }
}

/* **************************************************************************
 * BoxIndex implementation
 * ************************************************************************** */
// SpatIndex.cpp:93-95

/// SpatIndex.hpp:57 — `using BoxIndexEl = std::pair<Slic3r::BoundingBox, unsigned>;`
pub type BoxIndexEl = (BoundingBox, u32);

/// SpatIndex.hpp:83 — `enum QueryType { qtIntersects, qtWithin };`
/// (scoped inside `BoxIndex` in C++)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QueryType {
    /// SpatIndex.hpp:83 — `qtIntersects`
    Intersects,
    /// SpatIndex.hpp:83 — `qtWithin`
    Within,
}

/// `boost::geometry::intersects(a, b)` for two cartesian 2D boxes, as used by
/// the rtree `bgi::intersects` value predicate (SpatIndex.cpp:142).
/// Boost implements it as `!disjoint`, where box/box disjoint is
/// `max1 < min2 || min1 > max2` in any dimension
/// (boost/geometry/algorithms/detail/disjoint/box_box.hpp), so boxes that
/// merely touch on a boundary do intersect.
#[inline]
fn boxes_intersect(a: &BoundingBox, b: &BoundingBox) -> bool {
    a.min.x <= b.max.x
        && b.min.x <= a.max.x
        && a.min.y <= b.max.y
        && b.min.y <= a.max.y
}

/// `boost::geometry::within(a, b)` for two cartesian 2D boxes, as used by the
/// rtree `bgi::within` value predicate (SpatIndex.cpp:145).
/// Boost implements it per dimension as
/// `bing_min <= bed_min && bed_max <= bing_max && bed_min < bed_max`
/// (strategy::within::box_within_coord in
/// boost/geometry/strategies/cartesian/box_in_box.hpp): `a` is contained in
/// `b` and `a` has a non-empty interior in every dimension.
#[inline]
fn box_within(a: &BoundingBox, b: &BoundingBox) -> bool {
    (b.min.x <= a.min.x && a.max.x <= b.max.x && a.min.x < a.max.x)
        && (b.min.y <= a.min.y && a.max.y <= b.max.y && a.min.y < a.max.y)
}

/// SpatIndex.hpp:59 — `class BoxIndex`
/// SpatIndex.cpp:97-103 — `class BoxIndex::Impl` (boost rtree store)
#[derive(Debug, Clone, Default)]
pub struct BoxIndex {
    /// SpatIndex.cpp:102 — `BoostIndex m_store;`
    m_store: Vec<BoxIndexEl>,
}

impl BoxIndex {
    /// SpatIndex.cpp:105 — `BoxIndex::BoxIndex(): m_impl(new Impl()) {}`
    pub fn new() -> Self {
        Self { m_store: Vec::new() }
    }

    // SpatIndex.cpp:106-121 — destructor / copy / move special members are
    // covered by Rust's Clone/Drop semantics.

    /// SpatIndex.cpp:123-126 — `void BoxIndex::insert(const BoxIndexEl &el)`
    pub fn insert(&mut self, el: BoxIndexEl) {
        // SpatIndex.cpp:125
        self.m_store.push(el);
    }

    /// SpatIndex.hpp:76-79 — `void insert(const BoundingBox& bb, unsigned idx)`
    #[inline]
    pub fn insert_box(&mut self, bb: BoundingBox, idx: u32) {
        // SpatIndex.hpp:78
        self.insert((bb, idx));
    }

    /// SpatIndex.cpp:128-131 — `bool BoxIndex::remove(const BoxIndexEl &el)`
    /// The boost rtree removes a single element equal to `el` and returns the
    /// number of removed elements; the C++ wrapper returns `removed == 1`.
    /// The rtree default `equal_to` compares the indexable box with
    /// `geometry::equals` — which only sees the min/max corners exposed by the
    /// Box concept (SLA/BoostAdapter.hpp), NOT `BoundingBox::defined` — and
    /// the pair's second member with `operator==`.
    pub fn remove(&mut self, el: &BoxIndexEl) -> bool {
        // SpatIndex.cpp:130
        match self
            .m_store
            .iter()
            .position(|e| e.0.min == el.0.min && e.0.max == el.0.max && e.1 == el.1)
        {
            Some(pos) => {
                self.m_store.remove(pos);
                true
            }
            None => false,
        }
    }

    /// SpatIndex.cpp:133-149 —
    /// `std::vector<BoxIndexEl> BoxIndex::query(const BoundingBox &qrbb, BoxIndex::QueryType qt)`
    pub fn query(&self, qrbb: &BoundingBox, qt: QueryType) -> Vec<BoxIndexEl> {
        // SpatIndex.cpp:138 — `std::vector<BoxIndexEl> ret; ret.reserve(m_impl->m_store.size());`
        let mut ret: Vec<BoxIndexEl> = Vec::with_capacity(self.m_store.len());

        // SpatIndex.cpp:140
        match qt {
            QueryType::Intersects => {
                // SpatIndex.cpp:142 — bgi::intersects(qrbb)
                for el in &self.m_store {
                    if boxes_intersect(&el.0, qrbb) {
                        ret.push(*el);
                    }
                }
            }
            QueryType::Within => {
                // SpatIndex.cpp:145 — bgi::within(qrbb)
                for el in &self.m_store {
                    if box_within(&el.0, qrbb) {
                        ret.push(*el);
                    }
                }
            }
        }

        // SpatIndex.cpp:148
        ret
    }

    /// SpatIndex.cpp:151-154 — `size_t BoxIndex::size() const`
    pub fn size(&self) -> usize {
        // SpatIndex.cpp:153
        self.m_store.len()
    }

    /// SpatIndex.hpp:89 — `bool empty() const { return size() == 0; }`
    #[inline]
    pub fn empty(&self) -> bool {
        self.size() == 0
    }

    /// SpatIndex.cpp:156-159 —
    /// `void BoxIndex::foreach(std::function<void (const BoxIndexEl &)> fn)`
    pub fn foreach(&self, fn_: &mut dyn FnMut(&BoxIndexEl)) {
        // SpatIndex.cpp:158
        for el in &self.m_store {
            fn_(el);
        }
    }
}
