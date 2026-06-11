//! Faithful 1:1 port of `SLA/IndexedMesh.{hpp,cpp}` from BambuStudio libslic3r.
//!
//! C++ Reference:
//! - src/libslic3r/SLA/IndexedMesh.hpp (154 lines)
//! - src/libslic3r/SLA/IndexedMesh.cpp (457 lines)
//!
//! Fidelity notes (byte-exact G-code parity):
//! - `Vec3d` is Eigen `Matrix<double,3,1>` -> nalgebra `Vector3<f64>`; `Vec3f`/`Vec3i`
//!   come from `crate::triangle_mesh` (the crate-wide mirrors of the Eigen types).
//! - `PointSet = Eigen::MatrixXd` (IndexedMesh.hpp:26) -> `nalgebra::DMatrix<f64>`.
//! - `igl::Hit` stores the ray parameter `t` in *single* precision; the conversions
//!   `ret.m_t = double(hit.t)` (IndexedMesh.cpp:163,197) are reproduced by rounding
//!   the f64 ray parameter through f32 before widening back, and the hit sort at
//!   IndexedMesh.cpp:182-183 compares those f32 values.
//! - C++ `m_tm` is a non-owning `const indexed_triangle_set*` (IndexedMesh.hpp:34).
//!   Rust uses `Arc<indexed_triangle_set>` so that the copy constructor's pointer
//!   copy (IndexedMesh.cpp:95 `m_tm(other.m_tm)`) is reproduced as a refcount bump
//!   (both copies alias the same mesh data).
//! - The crate's `AABBTreeIndirect` port (`crate::aabb_tree_indirect`) takes
//!   `&[Point3F]` (f64) / `&[[usize;3]]` slices instead of the f32
//!   `indexed_triangle_set` arrays the C++ templates accept, so `AABBImpl::init`
//!   caches exact f32->f64-widened copies once (widening is value-exact; query
//!   results are identical to the C++ `cast<double>()` performed per-query).
//! - `#ifdef SLIC3R_HOLE_RAYCASTER` blocks (IndexedMesh.hpp:12 keeps the define
//!   commented out, "hidden ... for possible future use") are NOT compiled in the
//!   C++ build and are therefore not ported: `m_holes`/`load_holes`
//!   (IndexedMesh.hpp:39-43,109-122) and `filter_hits` (IndexedMesh.cpp:210-305).
//! - BLOCKED (not ported, see porter report): `IndexedMesh(const TriangleMesh&)`
//!   (IndexedMesh.cpp:86-90) — the crate's `TriangleMesh` is a documented divergent
//!   struct (f64 vertices, no `its` member; see triangle_mesh.rs "DIVERGENCE"), so
//!   the C++ `&mesh.its` pointer take has no faithful equivalent yet.
//! - C++ default arguments (`calculate_epsilon = false`, `eps = 0.05`,
//!   `throw_on_cancel = [](){}`, `selected_points = {}`) cannot be expressed in
//!   Rust; callers pass them explicitly.

use crate::aabb_tree_indirect::{self, Tree3F};
use crate::geometry::{Point3F, Vec3 as GeoVec3};
use crate::libslic3r::EPSILON;
use crate::normal_utils::indexed_triangle_set;
use crate::sla::ccr;
use crate::triangle_mesh::{
    bounding_box, its_average_edge_length, its_unnormalized_normal, Vec3f, Vec3i,
};
use nalgebra::{DMatrix, Vector3};
use std::cmp::Ordering;
use std::sync::Arc;

/// Eigen `Vec3d` (`Matrix<double,3,1>`).
/// Point.hpp
pub type Vec3d = Vector3<f64>;

/// IndexedMesh.hpp:26
/// C++: `using PointSet = Eigen::MatrixXd;`
pub type PointSet = DMatrix<f64>;

/// libslic3r.h: `template<typename Number> inline bool is_approx(Number value,
/// Number test_value) { return std::fabs(double(value) - double(test_value)) <
/// double(EPSILON); }` — used by the assert at IndexedMesh.cpp:148.
#[inline]
fn is_approx(value: f64, test_value: f64) -> bool {
    (value - test_value).abs() < EPSILON
}

/// Minimal mirror of `igl::Hit` (igl/Hit.h) as consumed by IndexedMesh.cpp: only
/// the primitive `id` and the single-precision ray parameter `t` are read here.
/// `gid`/`u`/`v` are kept for layout fidelity but are not recoverable from the
/// crate's `AABBTreeIndirect` port (which does not expose barycentric coordinates)
/// and are unused by this file, exactly as in the C++.
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
struct IglHit {
    /// igl/Hit.h: `int id;` — primitive id, -1 == no hit
    id: i32,
    /// igl/Hit.h: `int gid;` — geometry id (unused by IndexedMesh)
    gid: i32,
    /// igl/Hit.h: `float u, v;` — barycentric coordinates (unused by IndexedMesh)
    u: f32,
    v: f32,
    /// igl/Hit.h: `float t;` — ray parameter (single precision!)
    t: f32,
}

/// IndexedMesh.cpp:17-69
/// C++: `class IndexedMesh::AABBImpl`
#[derive(Debug, Clone)]
struct AABBImpl {
    /// IndexedMesh.cpp:19 — `AABBTreeIndirect::Tree3f m_tree;`
    /// (the crate's tree stores node boxes in f64; built over the exact
    /// f32->f64-widened vertices below)
    m_tree: Tree3F,
    /// IndexedMesh.cpp:20 — `double m_triangle_ray_epsilon;`
    m_triangle_ray_epsilon: f64,
    /// Rust-only cache (see module notes): exact f64 widenings of
    /// `its.vertices` / `its.indices`, required by the slice-based API of
    /// `crate::aabb_tree_indirect`. Value-identical to the per-query
    /// `cast<double>()` the C++ templates perform.
    vertices_f64: Vec<Point3F>,
    faces: Vec<[usize; 3]>,
}

impl Default for AABBImpl {
    /// IndexedMesh.cpp:81 — `m_aabb(new AABBImpl())` default-constructs the impl;
    /// all members are then set by `init()`.
    fn default() -> Self {
        Self {
            m_tree: Tree3F::new(),
            m_triangle_ray_epsilon: 0.0,
            vertices_f64: Vec::new(),
            faces: Vec::new(),
        }
    }
}

impl AABBImpl {
    /// IndexedMesh.cpp:23-34
    /// C++: `void init(const indexed_triangle_set &its, bool calculate_epsilon)`
    fn init(&mut self, its: &indexed_triangle_set, calculate_epsilon: bool) {
        // IndexedMesh.cpp:25
        self.m_triangle_ray_epsilon = 0.000001;
        // IndexedMesh.cpp:26
        if calculate_epsilon {
            // IndexedMesh.cpp:27 — Calculate epsilon from average triangle edge length.
            // IndexedMesh.cpp:28 — `double l = its_average_edge_length(its);`
            // (C++ widens the float return value to double.)
            let l = its_average_edge_length(its) as f64;
            // IndexedMesh.cpp:29-30
            if l > 0.0 {
                self.m_triangle_ray_epsilon = 0.000001 * l * l;
            }
        }
        // Rust-only: widen the f32 mesh data once for the slice-based tree API
        // (see module notes; the C++ passes `its.vertices, its.indices` directly).
        self.vertices_f64 = its
            .vertices
            .iter()
            .map(|v| Point3F::new(v.x as f64, v.y as f64, v.z as f64))
            .collect();
        self.faces = its
            .indices
            .iter()
            .map(|f| [f[0] as usize, f[1] as usize, f[2] as usize])
            .collect();
        // IndexedMesh.cpp:32-33
        // C++: m_tree = AABBTreeIndirect::build_aabb_tree_over_indexed_triangle_set(
        // C++:     its.vertices, its.indices);
        self.m_tree = aabb_tree_indirect::build_aabb_tree_over_indexed_triangle_set(
            &self.vertices_f64,
            &self.faces,
        );
    }

    /// IndexedMesh.cpp:36-43
    /// C++: `void intersect_ray(const indexed_triangle_set &its, const Vec3d &s,
    ///                          const Vec3d &dir, igl::Hit &hit)` (first-hit overload)
    fn intersect_ray(&self, its: &indexed_triangle_set, s: &Vec3d, dir: &Vec3d, hit: &mut IglHit) {
        debug_assert!(its.indices.len() == self.faces.len());
        // IndexedMesh.cpp:41-42
        // C++: AABBTreeIndirect::intersect_ray_first_hit(its.vertices, its.indices,
        // C++:     m_tree, s, dir, hit, m_triangle_ray_epsilon);
        let origin = Point3F::new(s.x, s.y, s.z);
        let d = Point3F::new(dir.x, dir.y, dir.z);
        if let Some((t, face_idx, _hit_point)) = aabb_tree_indirect::intersect_ray_first_hit_eps(
            &self.vertices_f64,
            &self.faces,
            &self.m_tree,
            &origin,
            &d,
            self.m_triangle_ray_epsilon,
        ) {
            // igl::Hit stores `id` as int and `t` as float (single precision).
            hit.id = face_idx as i32;
            hit.t = t as f32;
        }
        // On a miss the C++ leaves `hit` untouched (the caller pre-seeds t = inf).
    }

    /// IndexedMesh.cpp:45-52
    /// C++: `void intersect_ray(const indexed_triangle_set &its, const Vec3d &s,
    ///                          const Vec3d &dir, std::vector<igl::Hit> &hits)`
    /// (all-hits overload; Rust cannot overload, hence the `_hits` suffix)
    fn intersect_ray_hits(
        &self,
        its: &indexed_triangle_set,
        s: &Vec3d,
        dir: &Vec3d,
        hits: &mut Vec<IglHit>,
    ) {
        debug_assert!(its.indices.len() == self.faces.len());
        // IndexedMesh.cpp:50-51
        // C++: AABBTreeIndirect::intersect_ray_all_hits(its.vertices, its.indices,
        // C++:     m_tree, s, dir, hits, m_triangle_ray_epsilon);
        // (The C++ tree function sorts by t before returning; the crate port does
        // not, but IndexedMesh::query_ray_hits re-sorts at IndexedMesh.cpp:182-183,
        // restoring the order.)
        let origin = Point3F::new(s.x, s.y, s.z);
        let d = Point3F::new(dir.x, dir.y, dir.z);
        let raw = aabb_tree_indirect::intersect_ray_all_hits_eps(
            &self.vertices_f64,
            &self.faces,
            &self.m_tree,
            &origin,
            &d,
            self.m_triangle_ray_epsilon,
        );
        hits.clear();
        hits.extend(raw.into_iter().map(|(t, face_idx, _hit_point)| IglHit {
            id: face_idx as i32,
            gid: -1,
            u: 0.0,
            v: 0.0,
            // igl::Hit stores t in single precision.
            t: t as f32,
        }));
    }

    /// IndexedMesh.cpp:54-68
    /// C++: `double squared_distance(const indexed_triangle_set &its, const Vec3d &point,
    ///                               int &i, Eigen::Matrix<double, 1, 3> &closest)`
    fn squared_distance(
        &self,
        its: &indexed_triangle_set,
        point: &Vec3d,
        i: &mut i32,
        closest: &mut Vec3d,
    ) -> f64 {
        debug_assert!(its.indices.len() == self.faces.len());
        // IndexedMesh.cpp:59 — `size_t idx_unsigned = 0;`
        // IndexedMesh.cpp:60 — `Vec3d closest_vec3d(closest);`
        // IndexedMesh.cpp:61-64
        // C++: double dist = AABBTreeIndirect::squared_distance_to_indexed_triangle_set(
        // C++:     its.vertices, its.indices, m_tree, point, idx_unsigned, closest_vec3d);
        let (dist, idx_unsigned, closest_vec3d) =
            aabb_tree_indirect::squared_distance_to_indexed_triangle_set(
                &self.vertices_f64,
                &self.faces,
                &self.m_tree,
                GeoVec3::new(point.x, point.y, point.z),
            );
        // IndexedMesh.cpp:65
        *i = idx_unsigned as i32;
        // IndexedMesh.cpp:66
        *closest = Vec3d::new(closest_vec3d.x, closest_vec3d.y, closest_vec3d.z);
        // IndexedMesh.cpp:67
        dist
    }
}

/// IndexedMesh.hpp:28-31
/// An index-triangle structure for libIGL functions. Also serves as an
/// alternative (raw) input format for the SLASupportTree.
//  Implemented in libslic3r/SLA/Common.cpp
#[derive(Debug)]
pub struct IndexedMesh {
    // IndexedMesh.hpp:34 — `const indexed_triangle_set* m_tm;`
    // (non-owning pointer in C++; shared ownership here so copies alias the
    // same mesh data exactly as the C++ pointer copy does — see module notes)
    m_tm: Arc<indexed_triangle_set>,
    // IndexedMesh.hpp:35 — `double m_ground_level = 0, m_gnd_offset = 0;`
    m_ground_level: f64,
    m_gnd_offset: f64,
    // IndexedMesh.hpp:37 — `std::unique_ptr<AABBImpl> m_aabb;`
    m_aabb: Box<AABBImpl>,
    // IndexedMesh.hpp:39-43 — `std::vector<DrainHole> m_holes;` is inside
    // `#ifdef SLIC3R_HOLE_RAYCASTER` and is not compiled (hpp:12), hence absent.
}

/// IndexedMesh.hpp:71-107 — Result of a raycast
///
/// C++ nested class `IndexedMesh::hit_result`; the name is preserved verbatim.
/// The C++ `const IndexedMesh *m_mesh` back-pointer becomes a lifetime-bound
/// reference. The Eigen `Vec3d` members are default-constructed (uninitialized)
/// in C++; Rust zero-initializes them.
#[derive(Debug, Clone, Copy)]
#[allow(non_camel_case_types)]
pub struct hit_result<'a> {
    // IndexedMesh.hpp:73-74 — m_t holds a distance from m_source to the intersection.
    m_t: f64,
    // IndexedMesh.hpp:75 — `int m_face_id = -1;`
    m_face_id: i32,
    // IndexedMesh.hpp:76 — `const IndexedMesh *m_mesh = nullptr;`
    m_mesh: Option<&'a IndexedMesh>,
    // IndexedMesh.hpp:77
    m_dir: Vec3d,
    // IndexedMesh.hpp:78
    m_source: Vec3d,
    // IndexedMesh.hpp:79
    m_normal: Vec3d,
    // IndexedMesh.hpp:80 — `friend class IndexedMesh;` (same-module access in Rust)
}

impl<'a> hit_result<'a> {
    /// IndexedMesh.hpp:82-84
    /// C++: `explicit inline hit_result(const IndexedMesh& em): m_mesh(&em) {}`
    /// A valid object of this class can only be obtained from
    /// IndexedMesh::query_ray_hit method. (private in C++ — `pub(crate)` here
    /// would over-expose; module-private suffices for this file's callers)
    fn from_mesh(em: &'a IndexedMesh) -> Self {
        Self {
            m_t: Self::infty(),
            m_face_id: -1,
            m_mesh: Some(em),
            m_dir: Vec3d::zeros(),
            m_source: Vec3d::zeros(),
            m_normal: Vec3d::zeros(),
        }
    }

    /// IndexedMesh.hpp:86-87 — This denotes no hit on the mesh.
    /// C++: `static inline constexpr double infty() { return std::numeric_limits<double>::infinity(); }`
    pub const fn infty() -> f64 {
        f64::INFINITY
    }

    /// IndexedMesh.hpp:89
    /// C++: `explicit inline hit_result(double val = infty()) : m_t(val) {}`
    /// (the C++ default argument `val = infty()` is `hit_result::default()`)
    pub fn new(val: f64) -> Self {
        Self {
            m_t: val,
            m_face_id: -1,
            m_mesh: None,
            m_dir: Vec3d::zeros(),
            m_source: Vec3d::zeros(),
            m_normal: Vec3d::zeros(),
        }
    }

    /// IndexedMesh.hpp:91
    pub fn distance(&self) -> f64 {
        self.m_t
    }

    /// IndexedMesh.hpp:92
    pub fn direction(&self) -> &Vec3d {
        &self.m_dir
    }

    /// IndexedMesh.hpp:93
    pub fn source(&self) -> &Vec3d {
        &self.m_source
    }

    /// IndexedMesh.hpp:94
    /// C++: `inline Vec3d position() const { return m_source + m_dir * m_t; }`
    pub fn position(&self) -> Vec3d {
        self.m_source + self.m_dir * self.m_t
    }

    /// IndexedMesh.hpp:95
    pub fn face(&self) -> i32 {
        self.m_face_id
    }

    /// IndexedMesh.hpp:96
    /// C++: `inline bool is_valid() const { return m_mesh != nullptr; }`
    pub fn is_valid(&self) -> bool {
        self.m_mesh.is_some()
    }

    /// IndexedMesh.hpp:97
    /// C++: `inline bool is_hit() const { return m_face_id >= 0 && !std::isinf(m_t); }`
    pub fn is_hit(&self) -> bool {
        self.m_face_id >= 0 && !self.m_t.is_infinite()
    }

    /// IndexedMesh.hpp:99-102
    pub fn normal(&self) -> &Vec3d {
        // IndexedMesh.hpp:100
        assert!(self.is_valid());
        // IndexedMesh.hpp:101
        &self.m_normal
    }

    /// IndexedMesh.hpp:104-106
    /// C++: `inline bool is_inside() const { return is_hit() && normal().dot(m_dir) > 0; }`
    pub fn is_inside(&self) -> bool {
        self.is_hit() && self.normal().dot(&self.m_dir) > 0.0
    }
}

impl Default for hit_result<'_> {
    /// IndexedMesh.hpp:89 — the C++ default argument `val = infty()`.
    fn default() -> Self {
        Self::new(Self::infty())
    }
}

impl IndexedMesh {
    /// IndexedMesh.cpp:71-78
    /// C++: `template<class M> void IndexedMesh::init(const M &mesh, bool calculate_epsilon)`
    /// Instantiated here for `M = indexed_triangle_set`; the `M = TriangleMesh`
    /// instantiation is blocked together with that constructor (see module notes).
    fn init(&mut self, mesh: &indexed_triangle_set, calculate_epsilon: bool) {
        // IndexedMesh.cpp:73 — `BoundingBoxf3 bb = bounding_box(mesh);`
        let bb = bounding_box(mesh);
        // IndexedMesh.cpp:74 — `m_ground_level += bb.min(Z);`
        // (C++ BoundingBoxf3 default-constructs min to Vec3d::Zero(), so an empty
        // mesh contributes 0 here; the crate's empty BoundingBox3F holds MAX/MIN
        // sentinels, hence the is_defined() guard reproducing the C++ value.)
        self.m_ground_level += if bb.is_defined() { bb.min.z } else { 0.0 };

        // IndexedMesh.cpp:76-77 — Build the AABB accelaration tree
        // C++: m_aabb->init(*m_tm, calculate_epsilon);
        let tm = Arc::clone(&self.m_tm);
        self.m_aabb.init(&tm, calculate_epsilon);
    }

    /// IndexedMesh.cpp:80-84
    /// C++: `IndexedMesh::IndexedMesh(const indexed_triangle_set& tmesh, bool calculate_epsilon)
    ///       : m_aabb(new AABBImpl()), m_tm(&tmesh)`
    /// (IndexedMesh.hpp:51 declares `calculate_epsilon = false` as default —
    /// "calculate epsilon for triangle-ray intersection from an average triangle
    /// edge length. If set to false, a default epsilon is used, which works for
    /// 'reasonable' meshes." Rust callers pass it explicitly. The C++ keeps a
    /// non-owning pointer to the caller's mesh; this port clones it into shared
    /// ownership — see module notes.)
    pub fn new(tmesh: &indexed_triangle_set, calculate_epsilon: bool) -> Self {
        // IndexedMesh.cpp:81
        let mut out = Self {
            m_tm: Arc::new(tmesh.clone()),
            // IndexedMesh.hpp:35 — in-class initializers `= 0`
            m_ground_level: 0.0,
            m_gnd_offset: 0.0,
            m_aabb: Box::new(AABBImpl::default()),
        };
        // IndexedMesh.cpp:83 — `init(tmesh, calculate_epsilon);`
        out.init(tmesh, calculate_epsilon);
        out
    }

    // IndexedMesh.cpp:86-90
    // C++: IndexedMesh::IndexedMesh(const TriangleMesh &mesh, bool calculate_epsilon)
    //     : m_aabb(new AABBImpl()), m_tm(&mesh.its)
    // {
    //     init(mesh, calculate_epsilon);
    // }
    // BLOCKED: the crate's `TriangleMesh` is a documented divergent struct (f64
    // vertices, no `its` member — see triangle_mesh.rs "DIVERGENCE" note), so the
    // C++ `&mesh.its` borrow has no faithful equivalent. No lossy f64->f32 fake is
    // provided; once `TriangleMesh` carries an `indexed_triangle_set`, add
    // `pub fn from_triangle_mesh(mesh: &TriangleMesh, calculate_epsilon: bool)`.

    // IndexedMesh.cpp:92 — `IndexedMesh::~IndexedMesh() {}` (implicit Drop in Rust)

    // IndexedMesh.cpp:106-108:
    // C++: IndexedMesh &IndexedMesh::operator=(IndexedMesh &&other) = default;
    // C++: IndexedMesh::IndexedMesh(IndexedMesh &&other) = default;
    // (native Rust move semantics)

    /// IndexedMesh.hpp:62
    /// C++: `inline double ground_level() const { return m_ground_level + m_gnd_offset; }`
    #[inline]
    pub fn ground_level(&self) -> f64 {
        self.m_ground_level + self.m_gnd_offset
    }

    /// IndexedMesh.hpp:63
    /// C++: `inline void ground_level_offset(double o) { m_gnd_offset = o; }`
    /// (setter overload of `ground_level_offset`; Rust cannot overload, hence `set_`)
    #[inline]
    pub fn set_ground_level_offset(&mut self, o: f64) {
        self.m_gnd_offset = o;
    }

    /// IndexedMesh.hpp:64
    /// C++: `inline double ground_level_offset() const { return m_gnd_offset; }`
    #[inline]
    pub fn ground_level_offset(&self) -> f64 {
        self.m_gnd_offset
    }

    /// IndexedMesh.cpp:112-115
    /// C++: `const std::vector<Vec3f>& IndexedMesh::vertices() const`
    pub fn vertices(&self) -> &[Vec3f] {
        // IndexedMesh.cpp:114
        &self.m_tm.vertices
    }

    /// IndexedMesh.cpp:119-122
    /// C++: `const std::vector<Vec3i>& IndexedMesh::indices() const`
    pub fn indices(&self) -> &[Vec3i] {
        // IndexedMesh.cpp:121
        &self.m_tm.indices
    }

    /// IndexedMesh.cpp:126-129
    /// C++: `const Vec3f& IndexedMesh::vertices(size_t idx) const`
    /// (indexed overload of `vertices`; Rust cannot overload, hence `_at`)
    pub fn vertices_at(&self, idx: usize) -> &Vec3f {
        // IndexedMesh.cpp:128
        &self.m_tm.vertices[idx]
    }

    /// IndexedMesh.cpp:133-136
    /// C++: `const Vec3i& IndexedMesh::indices(size_t idx) const`
    /// (indexed overload of `indices`; Rust cannot overload, hence `_at`)
    pub fn indices_at(&self, idx: usize) -> &Vec3i {
        // IndexedMesh.cpp:135
        &self.m_tm.indices[idx]
    }

    /// IndexedMesh.cpp:139-142
    /// C++: `Vec3d IndexedMesh::normal_by_face_id(int face_id) const`
    pub fn normal_by_face_id(&self, face_id: i32) -> Vec3d {
        // IndexedMesh.cpp:141
        // C++: return its_unnormalized_normal(*m_tm, face_id).cast<double>().normalized();
        its_unnormalized_normal(&self.m_tm, face_id as usize)
            .cast::<f64>()
            .normalize()
    }

    /// IndexedMesh.cpp:145-172
    /// C++: `IndexedMesh::hit_result IndexedMesh::query_ray_hit(const Vec3d &s, const Vec3d &dir) const`
    pub fn query_ray_hit(&self, s: &Vec3d, dir: &Vec3d) -> hit_result<'_> {
        // IndexedMesh.cpp:148 — `assert(is_approx(dir.norm(), 1.));`
        debug_assert!(is_approx(dir.norm(), 1.0));
        // IndexedMesh.cpp:149 — `igl::Hit hit{-1, -1, 0.f, 0.f, 0.f};`
        let mut hit = IglHit {
            id: -1,
            gid: -1,
            u: 0.0,
            v: 0.0,
            t: 0.0,
        };
        // IndexedMesh.cpp:150 — `hit.t = std::numeric_limits<float>::infinity();`
        hit.t = f32::INFINITY;

        // IndexedMesh.cpp:152-159 — `#ifdef SLIC3R_HOLE_RAYCASTER` hole filtering
        // (not compiled; see module notes)

        // IndexedMesh.cpp:161 — `m_aabb->intersect_ray(*m_tm, s, dir, hit);`
        self.m_aabb.intersect_ray(&self.m_tm, s, dir, &mut hit);
        // IndexedMesh.cpp:162 — `hit_result ret(*this);`
        let mut ret = hit_result::from_mesh(self);
        // IndexedMesh.cpp:163 — `ret.m_t = double(hit.t);`
        ret.m_t = hit.t as f64;
        // IndexedMesh.cpp:164
        ret.m_dir = *dir;
        // IndexedMesh.cpp:165
        ret.m_source = *s;
        // IndexedMesh.cpp:166-169
        if !hit.t.is_infinite() && !hit.t.is_nan() {
            ret.m_normal = self.normal_by_face_id(hit.id);
            ret.m_face_id = hit.id;
        }

        // IndexedMesh.cpp:171
        ret
    }

    /// IndexedMesh.cpp:174-207
    /// C++: `std::vector<IndexedMesh::hit_result> IndexedMesh::query_ray_hits(const Vec3d &s, const Vec3d &dir) const`
    pub fn query_ray_hits(&self, s: &Vec3d, dir: &Vec3d) -> Vec<hit_result<'_>> {
        // IndexedMesh.cpp:177
        let mut outs: Vec<hit_result<'_>> = Vec::new();
        // IndexedMesh.cpp:178-179
        let mut hits: Vec<IglHit> = Vec::new();
        self.m_aabb.intersect_ray_hits(&self.m_tm, s, dir, &mut hits);

        // IndexedMesh.cpp:181-183 — The sort is necessary, the hits are not always sorted.
        // C++: std::sort(hits.begin(), hits.end(),
        // C++:           [](const igl::Hit& a, const igl::Hit& b) { return a.t < b.t; });
        hits.sort_unstable_by(|a, b| a.t.partial_cmp(&b.t).unwrap_or(Ordering::Equal));

        // IndexedMesh.cpp:185-191:
        // Remove duplicates. They sometimes appear, for example when the ray is cast
        // along an axis of a cube due to floating-point approximations in igl (?)
        // BBS: STUDIO-2591 A mesh with overlapping faces cannot be painted
        //hits.erase(std::unique(hits.begin(), hits.end(),
        //                       [](const igl::Hit& a, const igl::Hit& b)
        //                       { return a.t == b.t; }),
        //           hits.end());
        // (kept disabled, matching the C++ source)

        // IndexedMesh.cpp:193-194 — Convert the igl::Hit into hit_result
        outs.reserve(hits.len());
        // IndexedMesh.cpp:195-204
        for hit in &hits {
            // IndexedMesh.cpp:196
            outs.push(hit_result::from_mesh(self));
            let back = outs.last_mut().unwrap();
            // IndexedMesh.cpp:197 — `outs.back().m_t = double(hit.t);`
            back.m_t = hit.t as f64;
            // IndexedMesh.cpp:198
            back.m_dir = *dir;
            // IndexedMesh.cpp:199
            back.m_source = *s;
            // IndexedMesh.cpp:200-203
            if !hit.t.is_infinite() && !hit.t.is_nan() {
                back.m_normal = self.normal_by_face_id(hit.id);
                back.m_face_id = hit.id;
            }
        }

        // IndexedMesh.cpp:206
        outs
    }

    // IndexedMesh.cpp:210-305 / IndexedMesh.hpp:109-122:
    // `void load_holes(const std::vector<DrainHole>&)` and
    // `hit_result filter_hits(const std::vector<hit_result>&) const`
    // are inside `#ifdef SLIC3R_HOLE_RAYCASTER`, which IndexedMesh.hpp:9-12 keeps
    // disabled ("an implementation of a hole-aware raycaster that was eventually
    // not used in production"). Not compiled in C++, therefore not ported.

    /// IndexedMesh.cpp:308-315
    /// C++: `double IndexedMesh::squared_distance(const Vec3d &p, int& i, Vec3d& c) const`
    pub fn squared_distance(&self, p: &Vec3d, i: &mut i32, c: &mut Vec3d) -> f64 {
        // IndexedMesh.cpp:309 — `double sqdst = 0;`
        let sqdst;
        // IndexedMesh.cpp:310 — `Eigen::Matrix<double, 1, 3> pp = p;`
        let pp = *p;
        // IndexedMesh.cpp:311 — `Eigen::Matrix<double, 1, 3> cc;` (uninitialized in C++)
        let mut cc = Vec3d::zeros();
        // IndexedMesh.cpp:312
        sqdst = self.m_aabb.squared_distance(&self.m_tm, &pp, i, &mut cc);
        // IndexedMesh.cpp:313
        *c = cc;
        // IndexedMesh.cpp:314
        sqdst
    }

    /// IndexedMesh.hpp:131-136
    /// C++: `inline double squared_distance(const Vec3d &p) const`
    /// (overload without out-parameters; Rust cannot overload, hence `_simple`,
    /// matching the crate precedent in `aabb_mesh.rs`)
    pub fn squared_distance_simple(&self, p: &Vec3d) -> f64 {
        // IndexedMesh.hpp:133 — `int i;`
        let mut i: i32 = 0;
        // IndexedMesh.hpp:134 — `Vec3d c;`
        let mut c = Vec3d::zeros();
        // IndexedMesh.hpp:135
        self.squared_distance(p, &mut i, &mut c)
    }

    /// IndexedMesh.hpp:140
    /// C++: `const indexed_triangle_set * get_triangle_mesh() const { return m_tm; }`
    pub fn get_triangle_mesh(&self) -> &indexed_triangle_set {
        &self.m_tm
    }
}

impl Clone for IndexedMesh {
    /// IndexedMesh.cpp:94-96
    /// C++: `IndexedMesh::IndexedMesh(const IndexedMesh &other):
    ///       m_tm(other.m_tm), m_ground_level(other.m_ground_level),
    ///       m_aabb( new AABBImpl(*other.m_aabb) ) {}`
    /// NOTE: the C++ copy constructor does NOT copy `m_gnd_offset` — it stays at
    /// its in-class default 0. Reproduced exactly.
    fn clone(&self) -> Self {
        Self {
            // pointer copy in C++ — refcount bump here (same aliasing)
            m_tm: Arc::clone(&self.m_tm),
            m_ground_level: self.m_ground_level,
            m_gnd_offset: 0.0,
            m_aabb: self.m_aabb.clone(),
        }
    }

    /// IndexedMesh.cpp:99-104
    /// C++: `IndexedMesh &IndexedMesh::operator=(const IndexedMesh &other)
    ///       { m_tm = other.m_tm; m_ground_level = other.m_ground_level;
    ///         m_aabb.reset(new AABBImpl(*other.m_aabb)); return *this; }`
    /// NOTE: like the copy constructor, `operator=` does not assign `m_gnd_offset`;
    /// the destination keeps its current offset. Reproduced exactly.
    fn clone_from(&mut self, other: &Self) {
        // IndexedMesh.cpp:101
        self.m_tm = Arc::clone(&other.m_tm);
        // IndexedMesh.cpp:102
        self.m_ground_level = other.m_ground_level;
        // IndexedMesh.cpp:103
        self.m_aabb = other.m_aabb.clone();
    }
}

/// IndexedMesh.cpp:318-326
/// C++: `static bool point_on_edge(const Vec3d& p, const Vec3d& e1, const Vec3d& e2,
///                                 double eps = 0.05)`
fn point_on_edge(p: &Vec3d, e1: &Vec3d, e2: &Vec3d, eps: f64) -> bool {
    // IndexedMesh.cpp:321 — `using Line3D = Eigen::ParametrizedLine<double, 3>;`
    // IndexedMesh.cpp:323 — `auto line = Line3D::Through(e1, e2);`
    // Eigen: Through(p0, p1) => origin = p0, direction = (p1 - p0).normalized()
    let origin = *e1;
    let direction = (e2 - e1).normalize();
    // IndexedMesh.cpp:324 — `double d = line.distance(p);`
    // Eigen: distance(p) = sqrt(squaredDistance(p)),
    //        squaredDistance(p) = ((p - origin) - (p - origin).dot(dir) * dir).squaredNorm()
    let diff = p - origin;
    let d = (diff - direction * diff.dot(&direction)).norm();
    // IndexedMesh.cpp:325
    d.abs() < eps
}

/// IndexedMesh.cpp:328-454 / IndexedMesh.hpp:143-149
/// Calculate the normals for the selected points (from 'points' set) on the
/// mesh. This will call squared distance for each point.
/// C++: `PointSet normals(const PointSet& points, const IndexedMesh& mesh,
///                        double eps, std::function<void()> thr, // throw on cancel
///                        const std::vector<unsigned>& pt_indices)`
/// (C++ defaults: `eps = 0.05` // min distance from edges, `thr = [](){}`,
/// `pt_indices = {}` — Rust callers pass them explicitly.)
pub fn normals(
    points: &PointSet,
    mesh: &IndexedMesh,
    eps: f64,
    thr: &(dyn Fn() + Send + Sync), // throw on cancel
    pt_indices: &[u32],
) -> PointSet {
    // IndexedMesh.cpp:334-335
    if points.nrows() == 0 || mesh.vertices().is_empty() || mesh.indices().is_empty() {
        // C++ `return {};` — a default-constructed (0x0) MatrixXd
        return PointSet::zeros(0, 0);
    }

    // IndexedMesh.cpp:337
    let mut range: Vec<u32> = pt_indices.to_vec();
    // IndexedMesh.cpp:338-341
    if range.is_empty() {
        // C++: range.resize(size_t(points.rows()), 0); std::iota(range.begin(), range.end(), 0);
        range = (0..points.nrows() as u32).collect();
    }

    // IndexedMesh.cpp:343 — `PointSet ret(range.size(), 3);` (uninitialized in
    // Eigen; zero-initialized here — every row is written below)
    let mut ret = PointSet::zeros(range.len(), 3);

    // IndexedMesh.cpp:345 — //    for (size_t ridx = 0; ridx < range.size(); ++ridx)
    // IndexedMesh.cpp:346-347
    // C++: ccr::for_each(size_t(0), range.size(),
    //          [&ret, &mesh, &points, thr, eps, &range](size_t ridx) { ... });
    // Rust adaptation: the C++ lambda writes disjoint rows of `ret` from multiple
    // threads, which the borrow checker cannot express on a shared DMatrix; the
    // rows are computed through the same `ccr` facade over an index-tagged row
    // buffer and copied into `ret` afterwards. Same parallelism, same per-row
    // values, same output.
    let mut rows: Vec<(usize, Vec3d)> = (0..range.len()).map(|r| (r, Vec3d::zeros())).collect();
    ccr::for_each_mut(&mut rows, |&mut (ridx, ref mut row)| {
        // IndexedMesh.cpp:348
        thr();
        // IndexedMesh.cpp:349 — `unsigned el = range[ridx];`
        let el = range[ridx];
        // IndexedMesh.cpp:350 — `auto eidx = Eigen::Index(el);`
        let eidx = el as usize;
        // IndexedMesh.cpp:351 — `int faceid = 0;`
        let mut faceid: i32 = 0;
        // IndexedMesh.cpp:352 — `Vec3d p;` (uninitialized in C++)
        let mut p = Vec3d::zeros();

        // IndexedMesh.cpp:354 — `mesh.squared_distance(points.row(eidx), faceid, p);`
        let prow = points.row(eidx);
        mesh.squared_distance(
            &Vec3d::new(prow[0], prow[1], prow[2]),
            &mut faceid,
            &mut p,
        );

        // IndexedMesh.cpp:356 — `auto trindex = mesh.indices(faceid);`
        let trindex = *mesh.indices_at(faceid as usize);

        // IndexedMesh.cpp:358-360
        // C++: const Vec3d &p1 = mesh.vertices(trindex(0)).cast<double>();
        let p1: Vec3d = mesh.vertices_at(trindex[0] as usize).cast::<f64>();
        let p2: Vec3d = mesh.vertices_at(trindex[1] as usize).cast::<f64>();
        let p3: Vec3d = mesh.vertices_at(trindex[2] as usize).cast::<f64>();

        // IndexedMesh.cpp:362-369:
        // We should check if the point lies on an edge of the hosting
        // triangle. If it does then all the other triangles using the
        // same two points have to be searched and the final normal should
        // be some kind of aggregation of the participating triangle
        // normals. We should also consider the cases where the support
        // point lies right on a vertex of its triangle. The procedure is
        // the same, get the neighbor triangles and calculate an average
        // normal.

        // IndexedMesh.cpp:371-373:
        // mark the vertex indices of the edge. ia and ib marks and edge
        // ic will mark a single vertex.
        let mut ia: i32 = -1;
        let mut ib: i32 = -1;
        let mut ic: i32 = -1;

        // IndexedMesh.cpp:375-390
        if (p - p1).norm().abs() < eps {
            ic = trindex[0];
        } else if (p - p2).norm().abs() < eps {
            ic = trindex[1];
        } else if (p - p3).norm().abs() < eps {
            ic = trindex[2];
        } else if point_on_edge(&p, &p1, &p2, eps) {
            ia = trindex[0];
            ib = trindex[1];
        } else if point_on_edge(&p, &p2, &p3, eps) {
            ia = trindex[1];
            ib = trindex[2];
        } else if point_on_edge(&p, &p1, &p3, eps) {
            ia = trindex[0];
            ib = trindex[2];
        }

        // IndexedMesh.cpp:392-393:
        // vector for the neigboring triangles including the detected one.
        let mut neigh: Vec<usize> = Vec::new();
        // IndexedMesh.cpp:394 — The point is right on a vertex of the triangle
        if ic >= 0 {
            // IndexedMesh.cpp:395-400
            for n in 0..mesh.indices().len() {
                thr();
                let ni = *mesh.indices_at(n);
                // C++: if ((ni(X) == ic || ni(Y) == ic || ni(Z) == ic))
                if ni[0] == ic || ni[1] == ic || ni[2] == ic {
                    neigh.push(n);
                }
            }
        } else if ia >= 0 && ib >= 0 {
            // IndexedMesh.cpp:401-409 — the point is on and edge
            // now get all the neigboring triangles
            for n in 0..mesh.indices().len() {
                thr();
                let ni = *mesh.indices_at(n);
                if (ni[0] == ia || ni[1] == ia || ni[2] == ia)
                    && (ni[0] == ib || ni[1] == ib || ni[2] == ib)
                {
                    neigh.push(n);
                }
            }
        }

        // IndexedMesh.cpp:412-416 — Calculate the normals for the neighboring triangles
        let mut neighnorms: Vec<Vec3d> = Vec::with_capacity(neigh.len());
        for tri_id in &neigh {
            // C++: neighnorms.emplace_back(mesh.normal_by_face_id(tri_id));
            // (size_t narrowed to the int parameter)
            neighnorms.push(mesh.normal_by_face_id(*tri_id as i32));
        }

        // IndexedMesh.cpp:418-425:
        // Throw out duplicates. They would cause trouble with summing. We
        // will use std::unique which works on sorted ranges. We will sort
        // by the coefficient-wise sum of the normals. It should force the
        // same elements to be consecutive.
        // C++: std::sort(..., [](const Vec3d &v1, const Vec3d &v2)
        //                     { return v1.sum() < v2.sum(); });
        neighnorms.sort_unstable_by(|v1, v2| {
            v1.sum().partial_cmp(&v2.sum()).unwrap_or(Ordering::Equal)
        });

        // IndexedMesh.cpp:427-437 — `auto lend = std::unique(...)` with the
        // normal-equivalence predicate. std::unique keeps the first element of
        // every run of equivalent elements and returns the new logical end; it
        // does not shrink the vector.
        // C++: auto deq = [](double a, double b) { return std::abs(a - b) < 1e-3; };
        //      return deq(n1(X), n2(X)) && deq(n1(Y), n2(Y)) && deq(n1(Z), n2(Z));
        let deq = |a: f64, b: f64| (a - b).abs() < 1e-3;
        let neq = |n1: &Vec3d, n2: &Vec3d| {
            // Compare normals for equivalence.
            // This is controvers stuff.
            deq(n1[0], n2[0]) && deq(n1[1], n2[1]) && deq(n1[2], n2[2])
        };
        let mut lend = neighnorms.len();
        if lend > 1 {
            let mut last = 0usize;
            for i in 1..neighnorms.len() {
                if !neq(&neighnorms[last], &neighnorms[i]) {
                    last += 1;
                    neighnorms[last] = neighnorms[i];
                }
            }
            lend = last + 1;
        }

        // IndexedMesh.cpp:439 — there were neighbors to count with
        if !neighnorms.is_empty() {
            // IndexedMesh.cpp:440-441:
            // sum up the normals and then normalize the result again.
            // This unification seems to be enough.
            // IndexedMesh.cpp:442 — `Vec3d sumnorm(0, 0, 0);`
            let mut sumnorm = Vec3d::new(0.0, 0.0, 0.0);
            // IndexedMesh.cpp:443 — `sumnorm = std::accumulate(neighnorms.begin(), lend, sumnorm);`
            for nn in &neighnorms[..lend] {
                sumnorm += nn;
            }
            // IndexedMesh.cpp:444 — `sumnorm.normalize();` (in-place)
            sumnorm.normalize_mut();
            // IndexedMesh.cpp:445 — `ret.row(long(ridx)) = sumnorm;`
            *row = sumnorm;
        } else {
            // IndexedMesh.cpp:446 — point lies safely within its triangle
            // IndexedMesh.cpp:447-449
            let u: Vec3d = p2 - p1;
            let v: Vec3d = p3 - p1;
            *row = u.cross(&v).normalize();
        }
    });

    // (assembly of the row buffer back into `ret`; see adaptation note above)
    for (r, v) in rows {
        ret.row_mut(r).copy_from(&v.transpose());
    }

    // IndexedMesh.cpp:453
    ret
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::triangle_mesh::Vec3i;

    /// Unit cube [0,1]^3 as an indexed triangle set (12 triangles).
    fn cube_its() -> indexed_triangle_set {
        let vertices = vec![
            Vec3f::new(0.0, 0.0, 0.0),
            Vec3f::new(1.0, 0.0, 0.0),
            Vec3f::new(1.0, 1.0, 0.0),
            Vec3f::new(0.0, 1.0, 0.0),
            Vec3f::new(0.0, 0.0, 1.0),
            Vec3f::new(1.0, 0.0, 1.0),
            Vec3f::new(1.0, 1.0, 1.0),
            Vec3f::new(0.0, 1.0, 1.0),
        ];
        let indices = vec![
            Vec3i::new(0, 2, 1),
            Vec3i::new(0, 3, 2), // bottom (z=0)
            Vec3i::new(4, 5, 6),
            Vec3i::new(4, 6, 7), // top (z=1)
            Vec3i::new(0, 1, 5),
            Vec3i::new(0, 5, 4), // front (y=0)
            Vec3i::new(2, 3, 7),
            Vec3i::new(2, 7, 6), // back (y=1)
            Vec3i::new(1, 2, 6),
            Vec3i::new(1, 6, 5), // right (x=1)
            Vec3i::new(3, 0, 4),
            Vec3i::new(3, 4, 7), // left (x=0)
        ];
        indexed_triangle_set { vertices, indices }
    }

    #[test]
    fn test_ground_level_and_offset() {
        let its = cube_its();
        let mut m = IndexedMesh::new(&its, false);
        // IndexedMesh.cpp:74 — ground level is bb.min(Z) = 0
        assert_eq!(m.ground_level(), 0.0);
        m.set_ground_level_offset(1.5);
        assert_eq!(m.ground_level_offset(), 1.5);
        assert_eq!(m.ground_level(), 1.5);
        // IndexedMesh.cpp:94-96 — copy does not carry m_gnd_offset
        let c = m.clone();
        assert_eq!(c.ground_level(), 0.0);
    }

    #[test]
    fn test_query_ray_hit_cube() {
        let its = cube_its();
        let m = IndexedMesh::new(&its, false);
        // Ray from above the cube center straight down.
        let s = Vec3d::new(0.5, 0.5, 2.0);
        let dir = Vec3d::new(0.0, 0.0, -1.0);
        let hit = m.query_ray_hit(&s, &dir);
        assert!(hit.is_valid());
        assert!(hit.is_hit());
        // First hit is the top face at z=1 => t = 1.
        assert!((hit.distance() - 1.0).abs() < 1e-6);
        // Outward normal of the top face points +Z; ray points -Z => not inside.
        assert!(!hit.is_inside());
        let pos = hit.position();
        assert!((pos.z - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_query_ray_hits_sorted() {
        let its = cube_its();
        let m = IndexedMesh::new(&its, false);
        let s = Vec3d::new(0.5, 0.5, 2.0);
        let dir = Vec3d::new(0.0, 0.0, -1.0);
        let hits = m.query_ray_hits(&s, &dir);
        // Top and bottom faces.
        assert!(hits.len() >= 2);
        for w in hits.windows(2) {
            assert!(w[0].distance() <= w[1].distance());
        }
        assert!((hits[0].distance() - 1.0).abs() < 1e-6);
        assert!((hits.last().unwrap().distance() - 2.0).abs() < 1e-6);
    }

    #[test]
    fn test_query_ray_miss() {
        let its = cube_its();
        let m = IndexedMesh::new(&its, false);
        let s = Vec3d::new(5.0, 5.0, 2.0);
        let dir = Vec3d::new(0.0, 0.0, -1.0);
        let hit = m.query_ray_hit(&s, &dir);
        assert!(hit.is_valid());
        assert!(!hit.is_hit());
        assert!(hit.distance().is_infinite());
    }

    #[test]
    fn test_squared_distance() {
        let its = cube_its();
        let m = IndexedMesh::new(&its, false);
        let p = Vec3d::new(0.5, 0.5, 2.0);
        let mut i = 0i32;
        let mut c = Vec3d::zeros();
        let d2 = m.squared_distance(&p, &mut i, &mut c);
        assert!((d2 - 1.0).abs() < 1e-9);
        assert!((c.z - 1.0).abs() < 1e-9);
        assert_eq!(m.squared_distance_simple(&p), d2);
    }

    #[test]
    fn test_normals_inside_triangle() {
        let its = cube_its();
        let m = IndexedMesh::new(&its, false);
        // A point in the middle of the top face (away from edges by > eps).
        let mut pts = PointSet::zeros(1, 3);
        pts[(0, 0)] = 0.5;
        pts[(0, 1)] = 0.25;
        pts[(0, 2)] = 1.0;
        let ns = normals(&pts, &m, 0.05, &|| {}, &[]);
        assert_eq!(ns.nrows(), 1);
        assert_eq!(ns.ncols(), 3);
        // Top-face normal is +Z.
        assert!((ns[(0, 0)]).abs() < 1e-6);
        assert!((ns[(0, 1)]).abs() < 1e-6);
        assert!((ns[(0, 2)] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_normals_empty() {
        let its = cube_its();
        let m = IndexedMesh::new(&its, false);
        let pts = PointSet::zeros(0, 3);
        let ns = normals(&pts, &m, 0.05, &|| {}, &[]);
        assert_eq!(ns.nrows(), 0);
    }
}
