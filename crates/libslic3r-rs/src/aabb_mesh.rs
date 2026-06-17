//! Faithful port of `AABBMesh.{hpp,cpp}` from BambuStudio libslic3r.
//!
//! An index-triangle structure coupled with an AABB tree to support ray
//! casting, distance queries, and other higher level geometric operations.
//!
//! C++ Reference:
//! - src/libslic3r/AABBMesh.hpp (142 lines)
//! - src/libslic3r/AABBMesh.cpp (323 lines)
//!
//! Fidelity notes:
//! - C++ `m_tm` is a non-owning `const indexed_triangle_set*` (AABBMesh.hpp:30) whose
//!   vertices are `Vec3f`/`Vec3i` (single precision). The crate's `AABBTreeIndirect`
//!   port (`crate::aabb_tree_indirect`) takes `&[Point3F]` (f64) / `&[[usize;3]]`
//!   slices, and the callers in this crate (`face_detector.rs`,
//!   `sla/reproject_points_on_mesh.rs`) already feed it f64 vertices. To preserve
//!   that existing call-site API this port keeps an owned f64 [`IndexedTriangleSet`]
//!   rather than the crate's f32 `indexed_triangle_set`.
//!   FIDELITY-NOTE(F2): C++ stores mesh data as `Vec3f`/`coord_t=int32` and widens to
//!   `double` per-query (`cast<double>()`); this port stores f64 directly, so any
//!   intermediate value that the C++ would have rounded to f32 first is not
//!   reproduced here. The control flow / formulas below match the C++ exactly.
//! - `igl::Hit` stores the ray parameter `t` in *single* precision; the conversions
//!   `ret.m_t = double(hit.t)` (AABBMesh.cpp:169,202) are reproduced by rounding the
//!   f64 ray parameter through f32 before widening back, and the hit sort/unique at
//!   AABBMesh.cpp:188-196 compares those f32 values.
//! - `#ifdef SLIC3R_HOLE_RAYCASTER` blocks (AABBMesh.hpp:13 keeps the define commented
//!   out, "eventually not used in production version ... hidden ... for possible
//!   future use") are NOT compiled in the C++ build and are therefore not ported:
//!   `m_holes`/`load_holes` (AABBMesh.hpp:38-114) and `filter_hits`
//!   (AABBMesh.cpp:215-310).
//! - BLOCKED (not ported): `AABBMesh(const TriangleMesh&)` (AABBMesh.cpp:84-91) — the
//!   crate's `TriangleMesh` is a documented divergent struct (see triangle_mesh.rs
//!   "DIVERGENCE"); callers convert to [`IndexedTriangleSet`] and call [`AABBMesh::new`].

use crate::aabb_tree_indirect::{self, Tree3F};
use crate::geometry::{Point3F, Vec3};
use crate::normal_utils::indexed_triangle_set as crate_its;
use crate::triangle_mesh::its_face_neighbors;
use crate::CoordF;

/// Indexed triangle set representation
///
/// This is a simple structure holding vertices and triangle indices.
/// admesh/stl.h — `struct indexed_triangle_set`
///
/// NOTE: see module notes — this is the f64 owned variant consumed by this crate's
/// AABB callers, not the crate-wide f32 `indexed_triangle_set`.
#[derive(Debug, Clone)]
pub struct IndexedTriangleSet {
    /// Vertex positions (3D points)
    pub vertices: Vec<Point3F>,

    /// Triangle indices (each triangle references 3 vertices)
    pub indices: Vec<[usize; 3]>,
}

impl IndexedTriangleSet {
    /// Create a new empty indexed triangle set
    pub fn new() -> Self {
        Self {
            vertices: Vec::new(),
            indices: Vec::new(),
        }
    }

    /// Create from vertices and indices
    pub fn from_parts(vertices: Vec<Point3F>, indices: Vec<[usize; 3]>) -> Self {
        Self { vertices, indices }
    }

    /// Convert to the crate's f32 `indexed_triangle_set` so the crate's `its_*`
    /// helpers (which operate on `Vec3f`/`Vec3i`, matching the C++ signatures) can
    /// be reused. FIDELITY-NOTE(F2): the f64->f32 narrowing here mirrors the fact
    /// that the C++ mesh is f32 to begin with.
    fn to_crate_its(&self) -> crate_its {
        use crate::triangle_mesh::{Vec3f, Vec3i};
        crate_its {
            vertices: self
                .vertices
                .iter()
                .map(|v| Vec3f::new(v.x as f32, v.y as f32, v.z as f32))
                .collect(),
            indices: self
                .indices
                .iter()
                .map(|f| Vec3i::new(f[0] as i32, f[1] as i32, f[2] as i32))
                .collect(),
        }
    }
}

impl Default for IndexedTriangleSet {
    fn default() -> Self {
        Self::new()
    }
}

/// Vertex-face index mapping
///
/// Index of face indices incident with a vertex index.
/// TriangleMesh.hpp:168-190 (`VertexFaceIndex`); built in the AABBMesh ctor by the
/// `m_vfidx{tmesh}` member initializer (AABBMesh.cpp:78,87).
#[derive(Debug, Clone, Default)]
pub struct VertexFaceIndex {
    /// TriangleMesh.hpp:188 — `std::vector<size_t> m_vertex_to_face_start;`
    m_vertex_to_face_start: Vec<usize>,
    /// TriangleMesh.hpp:189 — `std::vector<size_t> m_vertex_faces_all;`
    m_vertex_faces_all: Vec<usize>,
}

impl VertexFaceIndex {
    /// Build vertex-face index from indexed triangle set.
    ///
    /// TriangleMesh.cpp:1903-1926 — `void VertexFaceIndex::create(const indexed_triangle_set &its)`
    pub fn from_its(its: &IndexedTriangleSet) -> Self {
        let mut idx = VertexFaceIndex::default();
        // TriangleMesh.cpp:1905
        idx.m_vertex_to_face_start = vec![0usize; its.vertices.len() + 1];
        // TriangleMesh.cpp:1906-1911 — 1) Calculate vertex incidence by scatter.
        for face in &its.indices {
            idx.m_vertex_to_face_start[face[0] + 1] += 1;
            idx.m_vertex_to_face_start[face[1] + 1] += 1;
            idx.m_vertex_to_face_start[face[2] + 1] += 1;
        }
        // TriangleMesh.cpp:1912-1914 — 2) Prefix sum to calculate offsets.
        for i in 2..idx.m_vertex_to_face_start.len() {
            idx.m_vertex_to_face_start[i] += idx.m_vertex_to_face_start[i - 1];
        }
        // TriangleMesh.cpp:1915-1921 — 3) Scatter indices of faces incident to a vertex.
        let total = *idx.m_vertex_to_face_start.last().unwrap_or(&0);
        idx.m_vertex_faces_all = vec![0usize; total];
        for face_idx in 0..its.indices.len() {
            let face = &its.indices[face_idx];
            for i in 0..3 {
                let slot = idx.m_vertex_to_face_start[face[i]];
                idx.m_vertex_faces_all[slot] = face_idx;
                idx.m_vertex_to_face_start[face[i]] += 1;
            }
        }
        // TriangleMesh.cpp:1922-1925 — 4) The previous loop modified
        // m_vertex_to_face_start. Revert the change.
        for i in (1..idx.m_vertex_to_face_start.len()).rev() {
            idx.m_vertex_to_face_start[i] = idx.m_vertex_to_face_start[i - 1];
        }
        if let Some(first) = idx.m_vertex_to_face_start.first_mut() {
            *first = 0;
        }
        idx
    }

    /// Get faces connected to a vertex.
    /// TriangleMesh.hpp:185 — `operator[]`
    pub fn faces_from_vertex(&self, vertex_idx: usize) -> &[usize] {
        if vertex_idx + 1 >= self.m_vertex_to_face_start.len() {
            return &[];
        }
        let begin = self.m_vertex_to_face_start[vertex_idx];
        let end = self.m_vertex_to_face_start[vertex_idx + 1];
        &self.m_vertex_faces_all[begin..end]
    }
}

/// Result of a ray cast operation on the mesh
///
/// C++ nested class `AABBMesh::hit_result`.
/// AABBMesh.hpp:65-100
#[derive(Debug, Clone, Copy)]
pub struct HitResult {
    /// AABBMesh.hpp:67 — m_t holds a distance from m_source to the intersection.
    t: CoordF,

    /// AABBMesh.hpp:68 — `int m_face_id = -1;`
    face_id: i32,

    /// AABBMesh.hpp:70 — `Vec3d m_dir = Vec3d::Zero();`
    dir: Vec3,

    /// AABBMesh.hpp:71 — `Vec3d m_source = Vec3d::Zero();`
    source: Vec3,

    /// AABBMesh.hpp:72 — `Vec3d m_normal = Vec3d::Zero();`
    normal: Vec3,

    /// AABBMesh.hpp:69 — `const AABBMesh *m_mesh = nullptr;` => is_valid() is `m_mesh != nullptr`.
    is_valid_result: bool,
}

impl HitResult {
    /// AABBMesh.hpp:80
    /// C++: `static inline constexpr double infty() { return std::numeric_limits<double>::infinity(); }`
    pub fn infty() -> CoordF {
        CoordF::INFINITY
    }

    /// AABBMesh.hpp:77 — `explicit inline hit_result(const AABBMesh& em): m_mesh(&em) {}`
    /// A valid object of this class can only be obtained from a query method; the
    /// back-pointer is represented as `is_valid_result = true`.
    fn from_mesh() -> Self {
        Self {
            t: Self::infty(),
            face_id: -1,
            dir: Vec3::new(0.0, 0.0, 0.0),
            source: Vec3::new(0.0, 0.0, 0.0),
            normal: Vec3::new(0.0, 0.0, 0.0),
            is_valid_result: true,
        }
    }

    /// AABBMesh.hpp:82 — `explicit inline hit_result(double val = infty()) : m_t(val) {}`
    /// (m_mesh stays nullptr, i.e. is_valid() == false)
    pub fn new() -> Self {
        Self {
            t: Self::infty(),
            face_id: -1,
            dir: Vec3::new(0.0, 0.0, 0.0),
            source: Vec3::new(0.0, 0.0, 0.0),
            normal: Vec3::new(0.0, 0.0, 0.0),
            is_valid_result: false,
        }
    }

    /// AABBMesh.hpp:84 — `inline double distance() const { return m_t; }`
    pub fn distance(&self) -> CoordF {
        self.t
    }

    /// AABBMesh.hpp:85 — `inline const Vec3d& direction() const { return m_dir; }`
    pub fn direction(&self) -> Vec3 {
        self.dir
    }

    /// AABBMesh.hpp:86 — `inline const Vec3d& source() const { return m_source; }`
    pub fn source(&self) -> Vec3 {
        self.source
    }

    /// AABBMesh.hpp:87 — `inline Vec3d position() const { return m_source + m_dir * m_t; }`
    pub fn position(&self) -> Vec3 {
        Vec3::new(
            self.source.x + self.dir.x * self.t,
            self.source.y + self.dir.y * self.t,
            self.source.z + self.dir.z * self.t,
        )
    }

    /// AABBMesh.hpp:88 — `inline int face() const { return m_face_id; }`
    pub fn face(&self) -> i32 {
        self.face_id
    }

    /// AABBMesh.hpp:89 — `inline bool is_valid() const { return m_mesh != nullptr; }`
    pub fn is_valid(&self) -> bool {
        self.is_valid_result
    }

    /// AABBMesh.hpp:90 — `inline bool is_hit() const { return m_face_id >= 0 && !std::isinf(m_t); }`
    pub fn is_hit(&self) -> bool {
        self.face_id >= 0 && !self.t.is_infinite()
    }

    /// AABBMesh.hpp:92-95
    /// C++: `inline const Vec3d& normal() const { assert(is_valid()); return m_normal; }`
    pub fn normal(&self) -> Vec3 {
        assert!(self.is_valid());
        self.normal
    }

    /// AABBMesh.hpp:97-99
    /// C++: `inline bool is_inside() const { return is_hit() && normal().dot(m_dir) > 0; }`
    pub fn is_inside(&self) -> bool {
        self.is_hit() && {
            let dot = self.normal.x * self.dir.x
                + self.normal.y * self.dir.y
                + self.normal.z * self.dir.z;
            dot > 0.0
        }
    }
}

impl Default for HitResult {
    /// AABBMesh.hpp:82 — the C++ default argument `val = infty()`.
    fn default() -> Self {
        Self::new()
    }
}

/// AABB mesh structure for spatial queries
///
/// AABBMesh.hpp:27-137
pub struct AABBMesh {
    /// AABBMesh.hpp:30 — `const indexed_triangle_set* m_tm;` (owned f64 variant here)
    its: IndexedTriangleSet,

    /// AABBMesh.cpp:17 — `AABBTreeIndirect::Tree3f m_tree;`
    aabb_tree: Tree3F,

    /// AABBMesh.hpp:33 — `VertexFaceIndex m_vfidx;` // vertex-face index
    vfidx: VertexFaceIndex,

    /// AABBMesh.hpp:34 — `std::vector<Vec3i> m_fnidx;` // face-neighbor index
    fnidx: Vec<[i32; 3]>,

    /// AABBMesh.cpp:18 — `double m_triangle_ray_epsilon;`
    triangle_ray_epsilon: CoordF,
}

impl AABBMesh {
    /// AABBMesh.cpp:75-82
    /// C++: `AABBMesh::AABBMesh(const indexed_triangle_set &tmesh, bool calculate_epsilon)
    ///       : m_tm(&tmesh), m_aabb(new AABBImpl()), m_vfidx{tmesh},
    ///         m_fnidx{its_face_neighbors(tmesh)} { init(tmesh, calculate_epsilon); }`
    /// (AABBMesh.hpp:48 declares `calculate_epsilon = false` as default — Rust callers
    /// pass it explicitly.)
    pub fn new(its: IndexedTriangleSet, calculate_epsilon: bool) -> Self {
        // AABBMesh.cpp:78 — `m_vfidx{tmesh}`
        let vfidx = VertexFaceIndex::from_its(&its);

        // AABBMesh.cpp:79 — `m_fnidx{its_face_neighbors(tmesh)}`
        let crate_its = its.to_crate_its();
        let fnidx = its_face_neighbors(&crate_its)
            .iter()
            .map(|n| [n[0], n[1], n[2]])
            .collect();

        // AABBMesh.cpp:81 / AABBMesh::AABBImpl::init (AABBMesh.cpp:21-32)
        // AABBMesh.cpp:23 — `m_triangle_ray_epsilon = 0.000001;`
        let mut triangle_ray_epsilon: CoordF = 0.000001;
        // AABBMesh.cpp:24 — `if (calculate_epsilon)`
        if calculate_epsilon {
            // AABBMesh.cpp:25-26 — Calculate epsilon from average triangle edge length.
            // C++: `double l = its_average_edge_length(its);`
            let l = its_average_edge_length(&its);
            // AABBMesh.cpp:27-28 — `if (l > 0) m_triangle_ray_epsilon = 0.000001 * l * l;`
            if l > 0.0 {
                triangle_ray_epsilon = 0.000001 * l * l;
            }
        }

        // AABBMesh.cpp:30-31
        // C++: m_tree = AABBTreeIndirect::build_aabb_tree_over_indexed_triangle_set(
        // C++:     its.vertices, its.indices);
        let aabb_tree = aabb_tree_indirect::build_aabb_tree_over_indexed_triangle_set(
            &its.vertices,
            &its.indices,
        );

        Self {
            its,
            aabb_tree,
            vfidx,
            fnidx,
            triangle_ray_epsilon,
        }
    }

    // AABBMesh.cpp:84-91
    // C++: AABBMesh::AABBMesh(const TriangleMesh &mesh, bool calculate_epsilon)
    //     : m_tm(&mesh.its), ... { init(mesh, calculate_epsilon); }
    // BLOCKED: the crate's `TriangleMesh` is a documented divergent struct (see
    // triangle_mesh.rs "DIVERGENCE"), so the C++ `&mesh.its` borrow has no faithful
    // equivalent; callers convert to `IndexedTriangleSet` and use `new`.

    /// AABBMesh.cpp:118-121
    /// C++: `const std::vector<Vec3f>& AABBMesh::vertices() const { return m_tm->vertices; }`
    pub fn vertices(&self) -> &[Point3F] {
        &self.its.vertices
    }

    /// AABBMesh.cpp:125-128
    /// C++: `const std::vector<Vec3i>& AABBMesh::indices() const { return m_tm->indices; }`
    pub fn indices(&self) -> &[[usize; 3]] {
        &self.its.indices
    }

    /// AABBMesh.cpp:132-135
    /// C++: `const Vec3f& AABBMesh::vertices(size_t idx) const { return m_tm->vertices[idx]; }`
    /// (indexed overload of `vertices`; Rust cannot overload, hence the distinct name)
    pub fn vertex(&self, idx: usize) -> Point3F {
        self.its.vertices[idx]
    }

    /// AABBMesh.cpp:139-142
    /// C++: `const Vec3i& AABBMesh::indices(size_t idx) const { return m_tm->indices[idx]; }`
    /// (indexed overload of `indices`; Rust cannot overload, hence the distinct name)
    pub fn triangle(&self, idx: usize) -> [usize; 3] {
        self.its.indices[idx]
    }

    /// AABBMesh.hpp:133
    /// C++: `const indexed_triangle_set * get_triangle_mesh() const { return m_tm; }`
    pub fn get_triangle_mesh(&self) -> &IndexedTriangleSet {
        &self.its
    }

    /// AABBMesh.hpp:135
    /// C++: `const VertexFaceIndex &vertex_face_index() const { return m_vfidx; }`
    pub fn vertex_face_index(&self) -> &VertexFaceIndex {
        &self.vfidx
    }

    /// AABBMesh.hpp:136
    /// C++: `const std::vector<Vec3i> &face_neighbor_index() const { return m_fnidx; }`
    pub fn face_neighbor_index(&self) -> &[[i32; 3]] {
        &self.fnidx
    }

    /// AABBMesh.cpp:145-148
    /// C++: `Vec3d AABBMesh::normal_by_face_id(int face_id) const`
    /// C++: `return its_unnormalized_normal(*m_tm, face_id).cast<double>().normalized();`
    pub fn normal_by_face_id(&self, face_id: usize) -> Vec3 {
        its_unnormalized_normal(&self.its, face_id).normalized()
    }

    /// AABBMesh.cpp:151-178
    /// C++: `AABBMesh::hit_result AABBMesh::query_ray_hit(const Vec3d &s, const Vec3d &dir) const`
    pub fn query_ray_hit(&self, s: Vec3, dir: Vec3) -> HitResult {
        // AABBMesh.cpp:154 — `assert(is_approx(dir.norm(), 1.));`
        debug_assert!(
            (dir.norm() - 1.0).abs() < 1e-6,
            "Ray direction must be normalized"
        );

        // AABBMesh.cpp:155 — `igl::Hit hit{-1, -1, 0.f, 0.f, 0.f};`
        // AABBMesh.cpp:156 — `hit.t = std::numeric_limits<float>::infinity();`
        // (`hit.id == -1`, `hit.t == +inf` on a miss; only `id`/`t` are read below.)

        // AABBMesh.cpp:158-165 — `#ifdef SLIC3R_HOLE_RAYCASTER` hole filtering
        // (not compiled; see module notes)

        // AABBMesh.cpp:167 — `m_aabb->intersect_ray(*m_tm, s, dir, hit);`
        // AABBMesh.cpp:39-40 — intersect_ray_first_hit(its.vertices, its.indices,
        //                       m_tree, s, dir, hit, m_triangle_ray_epsilon);
        let origin = Point3F::new(s.x, s.y, s.z);
        let d = Point3F::new(dir.x, dir.y, dir.z);
        let hit = aabb_tree_indirect::intersect_ray_first_hit_eps(
            &self.its.vertices,
            &self.its.indices,
            &self.aabb_tree,
            &origin,
            &d,
            self.triangle_ray_epsilon,
        );
        // igl::Hit stores `id` as int and `t` as float (single precision).
        let (hit_id, hit_t): (i32, f32) = match hit {
            Some((t, face_idx, _hit_point)) => (face_idx as i32, t as f32),
            None => (-1, f32::INFINITY),
        };

        // AABBMesh.cpp:168 — `hit_result ret(*this);`
        let mut ret = HitResult::from_mesh();
        // AABBMesh.cpp:169 — `ret.m_t = double(hit.t);`
        ret.t = hit_t as f64;
        // AABBMesh.cpp:170 — `ret.m_dir = dir;`
        ret.dir = dir;
        // AABBMesh.cpp:171 — `ret.m_source = s;`
        ret.source = s;
        // AABBMesh.cpp:172-175
        // C++: if(!std::isinf(hit.t) && !std::isnan(hit.t)) {
        //          ret.m_normal = this->normal_by_face_id(hit.id);
        //          ret.m_face_id = hit.id; }
        if !hit_t.is_infinite() && !hit_t.is_nan() {
            ret.normal = self.normal_by_face_id(hit_id as usize);
            ret.face_id = hit_id;
        }

        // AABBMesh.cpp:177
        ret
    }

    /// AABBMesh.cpp:180-212
    /// C++: `std::vector<AABBMesh::hit_result> AABBMesh::query_ray_hits(const Vec3d &s, const Vec3d &dir) const`
    pub fn query_ray_hits(&self, s: Vec3, dir: Vec3) -> Vec<HitResult> {
        // AABBMesh.cpp:183 — `std::vector<AABBMesh::hit_result> outs;`
        let mut outs: Vec<HitResult> = Vec::new();

        // AABBMesh.cpp:184-185
        // C++: std::vector<igl::Hit> hits;
        // C++: m_aabb->intersect_ray(*m_tm, s, dir, hits);
        // AABBMesh.cpp:48-49 — intersect_ray_all_hits(its.vertices, its.indices,
        //                       m_tree, s, dir, hits, m_triangle_ray_epsilon);
        let origin = Point3F::new(s.x, s.y, s.z);
        let d = Point3F::new(dir.x, dir.y, dir.z);
        let raw = aabb_tree_indirect::intersect_ray_all_hits_eps(
            &self.its.vertices,
            &self.its.indices,
            &self.aabb_tree,
            &origin,
            &d,
            self.triangle_ray_epsilon,
        );
        // igl::Hit stores `id` as int and `t` as float (single precision).
        let mut hits: Vec<(i32, f32)> = raw
            .into_iter()
            .map(|(t, face_idx, _hit_point)| (face_idx as i32, t as f32))
            .collect();

        // AABBMesh.cpp:188-189 — The sort is necessary, the hits are not always sorted.
        // C++: std::sort(hits.begin(), hits.end(),
        // C++:           [](const igl::Hit& a, const igl::Hit& b) { return a.t < b.t; });
        hits.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

        // AABBMesh.cpp:191-196 — Remove duplicates. They sometimes appear, for example
        // when the ray is cast along an axis of a cube due to floating-point
        // approximations in igl (?).
        // C++: hits.erase(std::unique(hits.begin(), hits.end(),
        // C++:                        [](const igl::Hit& a, const igl::Hit& b)
        // C++:                        { return a.t == b.t; }),
        // C++:            hits.end());
        // std::unique collapses consecutive runs comparing equal under the predicate
        // (here a.t == b.t), keeping the first element of each run.
        hits.dedup_by(|a, b| a.1 == b.1);

        // AABBMesh.cpp:199 — `outs.reserve(hits.size());`
        outs.reserve(hits.len());
        // AABBMesh.cpp:200-209
        for (hit_id, hit_t) in &hits {
            // AABBMesh.cpp:201 — `outs.emplace_back(AABBMesh::hit_result(*this));`
            let mut back = HitResult::from_mesh();
            // AABBMesh.cpp:202 — `outs.back().m_t = double(hit.t);`
            back.t = *hit_t as f64;
            // AABBMesh.cpp:203 — `outs.back().m_dir = dir;`
            back.dir = dir;
            // AABBMesh.cpp:204 — `outs.back().m_source = s;`
            back.source = s;
            // AABBMesh.cpp:205-208
            if !hit_t.is_infinite() && !hit_t.is_nan() {
                back.normal = self.normal_by_face_id(*hit_id as usize);
                back.face_id = *hit_id;
            }
            outs.push(back);
        }

        // AABBMesh.cpp:211
        outs
    }

    /// AABBMesh.cpp:313-320
    /// C++: `double AABBMesh::squared_distance(const Vec3d &p, int& i, Vec3d& c) const`
    /// (returns the squared distance, closest face index `i`, and closest point `c`)
    pub fn squared_distance(&self, point: Vec3) -> (CoordF, i32, Vec3) {
        // AABBMesh.cpp:315 — `Eigen::Matrix<double, 1, 3> pp = p;`
        // AABBMesh.cpp:317 / AABBImpl::squared_distance (AABBMesh.cpp:52-66)
        // C++: dist = AABBTreeIndirect::squared_distance_to_indexed_triangle_set(
        // C++:     its.vertices, its.indices, m_tree, point, idx_unsigned, closest_vec3d);
        let (dist_sq, face_idx, closest_point) =
            aabb_tree_indirect::squared_distance_to_indexed_triangle_set(
                &self.its.vertices,
                &self.its.indices,
                &self.aabb_tree,
                point,
            );

        // AABBMesh.cpp:63 — `i = int(idx_unsigned);`
        // AABBMesh.cpp:318 — `c = cc;`  / AABBMesh.cpp:319 — `return sqdst;`
        (dist_sq, face_idx as i32, closest_point)
    }

    /// AABBMesh.hpp:124-129
    /// C++: `inline double squared_distance(const Vec3d &p) const`
    /// (overload without out-parameters; Rust cannot overload, hence `_simple`)
    pub fn squared_distance_simple(&self, point: Vec3) -> CoordF {
        // AABBMesh.hpp:126-128 — `int i; Vec3d c; return squared_distance(p, i, c);`
        self.squared_distance(point).0
    }
}

impl Clone for AABBMesh {
    /// AABBMesh.cpp:95-100
    /// C++: `AABBMesh::AABBMesh(const AABBMesh &other)
    ///       : m_tm(other.m_tm), m_aabb(new AABBImpl(*other.m_aabb)),
    ///         m_vfidx{other.m_vfidx}, m_fnidx{other.m_fnidx} {}`
    fn clone(&self) -> Self {
        Self {
            its: self.its.clone(),
            aabb_tree: self.aabb_tree.clone(),
            vfidx: self.vfidx.clone(),
            fnidx: self.fnidx.clone(),
            triangle_ray_epsilon: self.triangle_ray_epsilon,
        }
    }
}

/// TriangleMesh.cpp:1848-1861
/// C++: `float its_average_edge_length(const indexed_triangle_set &its)`
/// (operates on the owned f64 [`IndexedTriangleSet`]; FIDELITY-NOTE(F2): the C++
/// computes `(v[i]-v[j]).cast<double>().norm()` on f32 vertices then narrows the
/// mean to f32 — here the inputs are already f64 and the result stays f64.)
fn its_average_edge_length(its: &IndexedTriangleSet) -> CoordF {
    // TriangleMesh.cpp:1850-1851
    if its.indices.is_empty() {
        return 0.0;
    }

    // TriangleMesh.cpp:1853
    let mut edge_length: f64 = 0.0;
    // TriangleMesh.cpp:1854-1859
    for triangle in &its.indices {
        let v0 = its.vertices[triangle[0]];
        let v1 = its.vertices[triangle[1]];
        let v2 = its.vertices[triangle[2]];

        // (v[1]-v[0]).norm() + (v[2]-v[0]).norm() + (v[1]-v[2]).norm()
        let d10 = {
            let dx = v1.x - v0.x;
            let dy = v1.y - v0.y;
            let dz = v1.z - v0.z;
            (dx * dx + dy * dy + dz * dz).sqrt()
        };
        let d20 = {
            let dx = v2.x - v0.x;
            let dy = v2.y - v0.y;
            let dz = v2.z - v0.z;
            (dx * dx + dy * dy + dz * dz).sqrt()
        };
        let d12 = {
            let dx = v1.x - v2.x;
            let dy = v1.y - v2.y;
            let dz = v1.z - v2.z;
            (dx * dx + dy * dy + dz * dz).sqrt()
        };
        edge_length += d10 + d20 + d12;
    }
    // TriangleMesh.cpp:1860 — `return edge_length / (3 * its.indices.size());`
    edge_length / (3 * its.indices.len()) as f64
}

/// TriangleMesh.hpp:316-321
/// C++: `inline stl_normal its_unnormalized_normal(const indexed_triangle_set &its, size_t face_id)`
/// C++: `{ its_triangle tri = its_triangle_vertices(its, face_id);
///         return (tri[1] - tri[0]).cross(tri[2] - tri[0]); }`
/// (FIDELITY-NOTE(F2): C++ computes the cross product in f32 then `normal_by_face_id`
/// widens to f64; this owned variant is already f64.)
fn its_unnormalized_normal(its: &IndexedTriangleSet, face_id: usize) -> Vec3 {
    // TriangleMesh.hpp:319 — its_triangle_vertices(its, face_id)
    let triangle = its.indices[face_id];
    let v0 = its.vertices[triangle[0]];
    let v1 = its.vertices[triangle[1]];
    let v2 = its.vertices[triangle[2]];

    // TriangleMesh.hpp:320 — `(tri[1] - tri[0]).cross(tri[2] - tri[0])`
    let e1 = Vec3::new(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
    let e2 = Vec3::new(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
    e1.cross(&e2)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_indexed_triangle_set_creation() {
        let its = IndexedTriangleSet::new();
        assert!(its.vertices.is_empty());
        assert!(its.indices.is_empty());
    }

    #[test]
    fn test_hit_result_creation() {
        let hit = HitResult::new();
        assert_eq!(hit.distance(), HitResult::infty());
        assert!(!hit.is_hit());
        assert!(!hit.is_valid());
    }

    #[test]
    fn test_hit_result_infty() {
        assert!(HitResult::infty().is_infinite());
        assert!(HitResult::infty() > 0.0);
    }

    #[test]
    fn test_vertex_face_index() {
        let vertices = vec![
            Point3F::new(0.0, 0.0, 0.0),
            Point3F::new(1.0, 0.0, 0.0),
            Point3F::new(0.0, 1.0, 0.0),
        ];
        let indices = vec![[0, 1, 2]];
        let its = IndexedTriangleSet::from_parts(vertices, indices);

        let vfidx = VertexFaceIndex::from_its(&its);
        assert_eq!(vfidx.faces_from_vertex(0), &[0]);
        assert_eq!(vfidx.faces_from_vertex(1), &[0]);
        assert_eq!(vfidx.faces_from_vertex(2), &[0]);
    }

    #[test]
    fn test_aabb_mesh_creation() {
        let its = IndexedTriangleSet::new();
        let _mesh = AABBMesh::new(its, false);
    }

    #[test]
    fn test_aabb_mesh_clone() {
        let its = IndexedTriangleSet::new();
        let mesh = AABBMesh::new(its, false);
        let _cloned = mesh.clone();
    }
}
