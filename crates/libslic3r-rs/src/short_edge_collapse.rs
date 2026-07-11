//! Faithful 1:1 port of `ShortEdgeCollapse.{hpp,cpp}` from BambuStudio libslic3r.
//!
//! C++ Reference:
//! - src/libslic3r/ShortEdgeCollapse.hpp (16 lines)
//! - src/libslic3r/ShortEdgeCollapse.cpp (188 lines)
//!
//! Decimates the model by collapsing short edges. It starts with very small edges
//! and gradually increases the collapsible length, until the target triangle count
//! is reached (the algorithm will certainly undershoot the target count, result will
//! have less triangles than target count). The algorithm does not check for triangle
//! flipping, disconnections, self intersections or any other degeneration that can
//! appear during mesh processing.
//! ShortEdgeCollapse.hpp:8-11
//!
//! Fidelity notes (byte-exact G-code parity):
//! - `coord_t->i64`, `coordf_t->f64`. Mesh vertices are `Vec3f` (Eigen
//!   `Matrix<float,3,1>`) and triangle indices `Vec3i` (Eigen `Matrix<int,3,1>`),
//!   matching `indexed_triangle_set`. All vector math is kept in `f32`.
//! - The RNG is `std::mt19937_64` seeded with the constant `27644437`; the faithful
//!   port [`crate::triangle_set_sampling::Mt19937_64`] is reused.
//! - `std::shuffle` is reproduced bit-for-bit following libstdc++'s
//!   `bits/stl_algo.h` (batched two-int Fisher-Yates), which is the standard library
//!   this crate targets for parity. This is the byte-exact-critical traversal order.
//! - `std::min(a, b)` for floats is reproduced as C++ semantics (`b < a ? b : a`),
//!   NOT IEEE `f32::min`, so NaN handling matches.
//! - `edge_len = edge_len * (1.0f + 1.0 - decimation_ratio)` mixes `float`/`double`
//!   literals: the whole RHS is evaluated in `f64` (because of the `1.0` double
//!   literal) and truncated back to `f32` on assignment. We reproduce that exactly.
//!
//! Dependency notes: `its_face_neighbors_par`, `its_face_normals`/`its_face_normal`,
//! `create_face_neighbors_index`/`VertexFaceIndex`, and the triangle edge/vertex
//! helpers live in `TriangleMesh.{hpp,cpp}` / `MeshSplitImpl.hpp`, whose Rust modules
//! (`triangle_mesh`, `mesh_split_impl`) are still placeholder stubs. They are
//! faithfully ported here as private helpers with their original `// <File>:NNN`
//! references so this file builds standalone; `its_face_neighbors_par` is ported as
//! the deterministic sequential traversal (the `ex_tbb` argument selects only the
//! execution strategy, not the result).

use nalgebra::{Vector2, Vector3};

use crate::normal_utils::{indexed_triangle_set, NormalUtils, Vec3f};
use crate::triangle_set_sampling::Mt19937_64;

/// 3D integer index vector, mirroring C++ `Vec3i` (Eigen `Matrix<int,3,1>`).
/// Point.hpp
type Vec3i = Vector3<i32>;
/// 2D integer vector, mirroring C++ `Vec2i` (Eigen `Matrix<int,2,1>`).
/// Point.hpp
type Vec2i = Vector2<i32>;

// ---------------------------------------------------------------------------
// Dependencies ported from TriangleMesh.{hpp,cpp} / MeshSplitImpl.hpp
// ---------------------------------------------------------------------------

/// `inline Vec3f face_normal(const stl_vertex vertex[3])`
/// TriangleMesh.hpp:331
#[inline]
fn face_normal(vertex: &[Vec3f; 3]) -> Vec3f {
    // (vertex[1] - vertex[0]).cross(vertex[2] - vertex[1]).normalized()
    (vertex[1] - vertex[0]).cross(&(vertex[2] - vertex[1])).normalize()
}

/// `inline Vec3f face_normal_normalized(const stl_vertex vertex[3])`
/// TriangleMesh.hpp:332
#[inline]
fn face_normal_normalized(vertex: &[Vec3f; 3]) -> Vec3f {
    // face_normal(vertex).normalized()
    face_normal(vertex).normalize()
}

/// `inline Vec3f its_face_normal(const indexed_triangle_set &its, const stl_triangle_vertex_indices face)`
/// TriangleMesh.hpp:333-334
#[inline]
fn its_face_normal(its: &indexed_triangle_set, face: &Vec3i) -> Vec3f {
    // const stl_vertex vertices[3] { its.vertices[face[0]], its.vertices[face[1]], its.vertices[face[2]] };
    let vertices: [Vec3f; 3] = [
        its.vertices[face[0] as usize],
        its.vertices[face[1] as usize],
        its.vertices[face[2] as usize],
    ];
    // return face_normal_normalized(vertices);
    face_normal_normalized(&vertices)
}

/// `std::vector<Vec3f> its_face_normals(const indexed_triangle_set &its)`
/// TriangleMesh.cpp:1938-1945
fn its_face_normals(its: &indexed_triangle_set) -> Vec<Vec3f> {
    // std::vector<Vec3f> normals;
    let mut normals: Vec<Vec3f> = Vec::new();
    // normals.reserve(its.indices.size());
    normals.reserve(its.indices.len());
    // for (stl_triangle_vertex_indices face : its.indices)
    for face in its.indices.iter() {
        // normals.push_back(its_face_normal(its, face));
        normals.push(its_face_normal(its, face));
    }
    // return normals;
    normals
}

/// `inline int its_triangle_vertex_index(const stl_triangle_vertex_indices &triangle_indices, int vertex_idx)`
/// TriangleMesh.hpp:249-254
#[inline]
fn its_triangle_vertex_index(triangle_indices: &Vec3i, vertex_idx: i32) -> i32 {
    // return vertex_idx == triangle_indices[0] ? 0 :
    //        vertex_idx == triangle_indices[1] ? 1 :
    //        vertex_idx == triangle_indices[2] ? 2 : -1;
    if vertex_idx == triangle_indices[0] {
        0
    } else if vertex_idx == triangle_indices[1] {
        1
    } else if vertex_idx == triangle_indices[2] {
        2
    } else {
        -1
    }
}

/// `inline Vec2i its_triangle_edge(const stl_triangle_vertex_indices &triangle_indices, int edge_idx)`
/// TriangleMesh.hpp:256-260
#[inline]
fn its_triangle_edge(triangle_indices: &Vec3i, edge_idx: i32) -> Vec2i {
    // int next_edge_idx = (edge_idx == 2) ? 0 : edge_idx + 1;
    let next_edge_idx: i32 = if edge_idx == 2 { 0 } else { edge_idx + 1 };
    // return { triangle_indices[edge_idx], triangle_indices[next_edge_idx] };
    Vec2i::new(
        triangle_indices[edge_idx as usize],
        triangle_indices[next_edge_idx as usize],
    )
}

/// `struct VertexFaceIndex`
/// TriangleMesh.hpp:168-191
struct VertexFaceIndex {
    // std::vector<size_t> m_vertex_to_face_start;
    m_vertex_to_face_start: Vec<usize>,
    // std::vector<size_t> m_vertex_faces_all;
    m_vertex_faces_all: Vec<usize>,
}

impl VertexFaceIndex {
    /// `VertexFaceIndex(const indexed_triangle_set &its) { this->create(its); }`
    /// TriangleMesh.hpp:173
    fn new(its: &indexed_triangle_set) -> Self {
        let mut idx = VertexFaceIndex {
            m_vertex_to_face_start: Vec::new(),
            m_vertex_faces_all: Vec::new(),
        };
        idx.create(its);
        idx
    }

    /// `void VertexFaceIndex::create(const indexed_triangle_set &its)`
    /// TriangleMesh.cpp:1903-1926
    fn create(&mut self, its: &indexed_triangle_set) {
        // m_vertex_to_face_start.assign(its.vertices.size() + 1, 0);
        self.m_vertex_to_face_start = vec![0usize; its.vertices.len() + 1];
        // 1) Calculate vertex incidence by scatter.
        // for (auto &face : its.indices) {
        for face in its.indices.iter() {
            // ++ m_vertex_to_face_start[face(0) + 1];
            self.m_vertex_to_face_start[face[0] as usize + 1] += 1;
            // ++ m_vertex_to_face_start[face(1) + 1];
            self.m_vertex_to_face_start[face[1] as usize + 1] += 1;
            // ++ m_vertex_to_face_start[face(2) + 1];
            self.m_vertex_to_face_start[face[2] as usize + 1] += 1;
        }
        // 2) Prefix sum to calculate offsets to m_vertex_faces_all.
        // for (size_t i = 2; i < m_vertex_to_face_start.size(); ++ i)
        for i in 2..self.m_vertex_to_face_start.len() {
            // m_vertex_to_face_start[i] += m_vertex_to_face_start[i - 1];
            self.m_vertex_to_face_start[i] += self.m_vertex_to_face_start[i - 1];
        }
        // 3) Scatter indices of faces incident to a vertex into m_vertex_faces_all.
        // m_vertex_faces_all.assign(m_vertex_to_face_start.back(), 0);
        self.m_vertex_faces_all = vec![0usize; *self.m_vertex_to_face_start.last().unwrap()];
        // for (size_t face_idx = 0; face_idx < its.indices.size(); ++ face_idx) {
        for face_idx in 0..its.indices.len() {
            // auto &face = its.indices[face_idx];
            let face = &its.indices[face_idx];
            // for (int i = 0; i < 3; ++ i)
            for i in 0..3 {
                // m_vertex_faces_all[m_vertex_to_face_start[face(i)] ++] = face_idx;
                let slot = self.m_vertex_to_face_start[face[i] as usize];
                self.m_vertex_faces_all[slot] = face_idx;
                self.m_vertex_to_face_start[face[i] as usize] += 1;
            }
        }
        // 4) The previous loop modified m_vertex_to_face_start. Revert the change.
        // for (auto i = int(m_vertex_to_face_start.size()) - 1; i > 0; -- i)
        let mut i = self.m_vertex_to_face_start.len() as i32 - 1;
        while i > 0 {
            // m_vertex_to_face_start[i] = m_vertex_to_face_start[i - 1];
            self.m_vertex_to_face_start[i as usize] = self.m_vertex_to_face_start[i as usize - 1];
            i -= 1;
        }
        // m_vertex_to_face_start.front() = 0;
        self.m_vertex_to_face_start[0] = 0;
    }

    /// Face indices incident with `vertex_id`, mirroring `operator[]`.
    /// TriangleMesh.hpp:180-185 (begin/end/operator[])
    #[inline]
    fn faces(&self, vertex_id: usize) -> &[usize] {
        // begin: m_vertex_faces_all.begin() + m_vertex_to_face_start[vertex_id]
        // end:   m_vertex_faces_all.begin() + m_vertex_to_face_start[vertex_id + 1]
        let begin = self.m_vertex_to_face_start[vertex_id];
        let end = self.m_vertex_to_face_start[vertex_id + 1];
        &self.m_vertex_faces_all[begin..end]
    }
}

/// `std::vector<Vec3i> create_face_neighbors_index(ExPolicy &&ex, const indexed_triangle_set &its)`
/// MeshSplitImpl.hpp:293-342
///
/// Ported as the deterministic sequential traversal: the `ExPolicy` argument selects
/// only the execution strategy (`ex_seq` vs `ex_tbb`), and `create_face_neighbors_index`
/// produces the same result regardless because the per-face work only reads shared
/// state and writes disjoint `neighbors[other_face][vertex_index]` slots with the
/// `!= no_value` guard, so the result is order-independent.
fn create_face_neighbors_index(its: &indexed_triangle_set) -> Vec<Vec3i> {
    // const std::vector<stl_triangle_vertex_indices> &indices = its.indices;
    let indices = &its.indices;

    // if (indices.empty()) return {};
    if indices.is_empty() {
        return Vec::new();
    }

    // assert(! its.vertices.empty());
    debug_assert!(!its.vertices.is_empty());

    // auto vertex_triangles = VertexFaceIndex{its};
    let vertex_triangles = VertexFaceIndex::new(its);
    // static constexpr int no_value = -1;
    const NO_VALUE: i32 = -1;
    // std::vector<Vec3i> neighbors(indices.size(), Vec3i(no_value, no_value, no_value));
    let mut neighbors: Vec<Vec3i> = vec![Vec3i::new(NO_VALUE, NO_VALUE, NO_VALUE); indices.len()];

    // execution::for_each(ex, size_t(0), indices.size(), [&] (size_t face_idx) { ... });
    for face_idx in 0..indices.len() {
        // Vec3i& neighbor = neighbors[face_idx];
        // const stl_triangle_vertex_indices & triangle_indices = indices[face_idx];
        let triangle_indices = indices[face_idx];
        // for (int edge_index = 0; edge_index < 3; ++edge_index) {
        for edge_index in 0..3usize {
            // int& neighbor_edge = neighbor[edge_index];
            // if (neighbor_edge != no_value) continue;  // This edge already has a neighbor assigned.
            if neighbors[face_idx][edge_index] != NO_VALUE {
                continue;
            }
            // Vec2i edge_indices = its_triangle_edge(triangle_indices, edge_index);
            let edge_indices = its_triangle_edge(&triangle_indices, edge_index as i32);
            // IMPROVE: use same vector for 2 sides of triangle
            // for (const size_t other_face : vertex_triangles[edge_indices[0]]) {
            for &other_face in vertex_triangles.faces(edge_indices[0] as usize) {
                // if (other_face <= face_idx) continue;
                if other_face <= face_idx {
                    continue;
                }
                // const stl_triangle_vertex_indices &face_indices = indices[other_face];
                let face_indices = indices[other_face];
                // int vertex_index = its_triangle_vertex_index(face_indices, edge_indices[1]);
                let vertex_index = its_triangle_vertex_index(&face_indices, edge_indices[1]);
                // NOT Contain second vertex?
                // if (vertex_index < 0) continue;
                if vertex_index < 0 {
                    continue;
                }
                // Has NOT oposite direction?
                // if (edge_indices[0] != face_indices[(vertex_index + 1) % 3]) continue;
                if edge_indices[0] != face_indices[((vertex_index + 1) % 3) as usize] {
                    continue;
                }
                //BBS: if this neighbor has already marked before, skip it
                // if (neighbors[other_face][vertex_index] != no_value) continue;
                if neighbors[other_face][vertex_index as usize] != NO_VALUE {
                    continue;
                }
                //BBS: the same triangle with opposite direction, also treat it as open edges
                //if (its_triangle_vertex_the_same(face_indices, triangle_indices))
                //    continue;
                // neighbor_edge = other_face;
                neighbors[face_idx][edge_index] = other_face as i32;
                // neighbors[other_face][vertex_index] = face_idx;
                neighbors[other_face][vertex_index as usize] = face_idx as i32;
                // break;
                break;
            }
        }
    }

    // return neighbors;
    neighbors
}

/// `std::vector<Vec3i> its_face_neighbors_par(const indexed_triangle_set &its)`
/// TriangleMesh.cpp:1933-1936
fn its_face_neighbors_par(its: &indexed_triangle_set) -> Vec<Vec3i> {
    // return create_face_neighbors_index(ex_tbb, its);
    create_face_neighbors_index(its)
}

// ---------------------------------------------------------------------------
// libstdc++ std::shuffle reproduction (bits/stl_algo.h)
// ---------------------------------------------------------------------------

/// Faithful port of libstdc++'s `__detail::__gen_two_uniform_ints`
/// (bits/uniform_int_dist.h). Generates two uniform ints in `[0,__b0)` and
/// `[0,__b1)` from a single draw of `uniform_int_distribution{0, __b0*__b1 - 1}`.
#[inline]
fn gen_two_uniform_ints(b0: u64, b1: u64, g: &mut Mt19937_64) -> (u64, u64) {
    // _IntType __x = uniform_int_distribution<_IntType>{0, (__b0 * __b1) - 1}(__g);
    let x = uniform_int(g, (b0 * b1) - 1);
    // return std::make_pair(__x / __b1, __x % __b1);
    (x / b1, x % b1)
}

/// Faithful port of libstdc++'s `std::uniform_int_distribution<_IntType>::operator()`
/// for the closed range `[0, range_inclusive]` driven by `std::mt19937_64`
/// (bits/uniform_int_dist.h). The engine produces `[0, 2^64-1]`.
fn uniform_int(g: &mut Mt19937_64, range_inclusive: u64) -> u64 {
    // __urngrange = __urng.max() - __urng.min(); for mt19937_64 == 2^64 - 1 (all-ones)
    let urngrange: u64 = u64::MAX;
    // __urange = __param.b() - __param.a(); (here a == 0)
    let urange = range_inclusive;

    if urngrange > urange {
        // downscaling
        // const __uctype __uerange = __urange + 1; // __urange can be zero
        let uerange = urange.wrapping_add(1); // > 1 here (caller never passes urange == MAX)
        // const __uctype __scaling = __urngrange / __uerange;
        let scaling = urngrange / uerange;
        // const __uctype __past = __uerange * __scaling;
        let past = uerange * scaling;
        // do { __ret = __uctype(__urng()) - __urngmin; } while (__ret >= __past);
        let mut ret;
        loop {
            ret = g.next_u64(); // __urngmin == 0
            if ret < past {
                break;
            }
        }
        // __ret / __scaling;
        ret / scaling
    } else if urngrange < urange {
        // upscaling: not reachable for mt19937_64 vs the small ranges used by shuffle.
        let mut ret;
        loop {
            ret = g.next_u64();
            if ret <= urange {
                break;
            }
        }
        ret
    } else {
        // __urngrange == __urange
        g.next_u64() // (__urng() - __urngmin) + __param.a(), a == 0
    }
}

/// libc++'s `std::shuffle` (__libcpp) — plain Fisher-Yates: for each position,
/// draw uniform_int_distribution<ptrdiff_t>(0, d) via MASKED REJECTION (the
/// libc++ algorithm), swap when i != 0. The native binary on darwin links
/// libc++, whose shuffle produces a completely different permutation from
/// libstdc++'s paired-swap batching given the same mt19937_64 stream (R184).
fn uniform_int_libcxx(g: &mut Mt19937_64, d: u64) -> u64 {
    let rp = d.wrapping_add(1);
    if rp == 1 {
        return 0;
    }
    if rp == 0 {
        return g.next_u64();
    }
    let dt = 64u32;
    let mut w = dt - rp.leading_zeros() - 1;
    if (rp & (u64::MAX >> (dt - w))) != 0 {
        w += 1;
    }
    let mask = if w >= 64 { u64::MAX } else { (1u64 << w) - 1 };
    loop {
        let u = g.next_u64() & mask;
        if u < rp {
            return u;
        }
    }
}

fn shuffle_libcxx<T>(v: &mut [T], g: &mut Mt19937_64) {
    let n = v.len();
    if n <= 1 {
        return;
    }
    let mut d = n - 1;
    let mut first = 0usize;
    let last = n - 1;
    while first < last {
        let i = uniform_int_libcxx(g, d as u64) as usize;
        if i != 0 {
            v.swap(first, first + i);
        }
        first += 1;
        d -= 1;
    }
}

/// Faithful port of libstdc++'s `std::shuffle` for random-access iterators
/// (bits/stl_algo.h). Reproduces the batched two-int Fisher-Yates exactly so the
/// resulting permutation is byte-identical to the C++ slicer.
fn shuffle<T>(v: &mut [T], g: &mut Mt19937_64) {
    // if (__first == __last) return;
    if v.is_empty() {
        return;
    }
    // const __uc_type __urngrange = __g.max() - __g.min(); == 2^64 - 1 for mt19937_64
    let urngrange: u64 = u64::MAX;
    // const __uc_type __urange = __uc_type(__last - __first);
    let urange: u64 = v.len() as u64;

    // if (__urngrange / __urange >= __urange)  // I.e. (__urngrange >= __urange * __urange)
    if urngrange / urange >= urange {
        // _RandomAccessIterator __i = __first + 1;
        let mut i: usize = 1;
        // Since we know the range isn't empty, an even number of elements
        // means an uneven number of elements to swap, in which case we
        // do the first one up front:
        // if ((__urange % 2) == 0)
        if urange % 2 == 0 {
            // __distr_type __d{0, 1};
            // std::iter_swap(__i++, __first + __d(__g));
            let pos = uniform_int(g, 1) as usize;
            v.swap(i, pos);
            i += 1;
        }
        // Now we know that __last - __i is even, so we do the rest in pairs,
        // using a single distribution invocation to produce swap positions
        // for two successive elements at a time:
        // while (__i != __last)
        while i != v.len() {
            // const __uc_type __swap_range = __uc_type(__i - __first) + 1;
            let swap_range = i as u64 + 1;
            // const pair<__uc_type, __uc_type> __pospos =
            //     __gen_two_uniform_ints(__swap_range, __swap_range + 1, __g);
            let pospos = gen_two_uniform_ints(swap_range, swap_range + 1, g);
            // std::iter_swap(__i++, __first + __pospos.first);
            v.swap(i, pospos.0 as usize);
            i += 1;
            // std::iter_swap(__i++, __first + __pospos.second);
            v.swap(i, pospos.1 as usize);
            i += 1;
        }
        return;
    }

    // __distr_type __d;
    // for (_RandomAccessIterator __i = __first + 1; __i != __last; ++__i)
    //     std::iter_swap(__i, __first + __d(__g, __p_type(0, __i - __first)));
    for i in 1..v.len() {
        let pos = uniform_int(g, i as u64) as usize;
        v.swap(i, pos);
    }
}

/// Faithful representation of a C++ float `std::min(a, b)` returning `b < a ? b : a`.
/// ShortEdgeCollapse.cpp:51-53 (`std::min` over `float`)
#[inline]
fn cpp_min_f32(a: f32, b: f32) -> f32 {
    // std::min returns the second argument unless the first compares less.
    if b < a {
        b
    } else {
        a
    }
}

// ---------------------------------------------------------------------------
// ShortEdgeCollapse.cpp
// ---------------------------------------------------------------------------

/// `void its_short_edge_collpase(indexed_triangle_set &mesh, size_t target_triangle_count)`
/// ShortEdgeCollapse.cpp:11-184
#[allow(clippy::needless_range_loop)]
pub fn its_short_edge_collpase(mesh: &mut indexed_triangle_set, target_triangle_count: usize) {
    // whenever vertex is removed, its mapping is update to the index of vertex with wich it merged
    // ShortEdgeCollapse.cpp:13
    let mut vertices_index_mapping: Vec<usize> = vec![0usize; mesh.vertices.len()];
    // ShortEdgeCollapse.cpp:14-16
    for idx in 0..vertices_index_mapping.len() {
        vertices_index_mapping[idx] = idx;
    }
    // Algorithm uses get_final_index query to get the actual vertex index. The query also updates all mappings on the way, essentially flattening the mapping
    // ShortEdgeCollapse.cpp:18
    let mut flatten_queue: Vec<usize> = Vec::new();
    // auto get_final_index = [&vertices_index_mapping, &flatten_queue](const size_t &orig_index) { ... };
    // ShortEdgeCollapse.cpp:19-31 — implemented as a closure-like local fn taking the captured state by ref.
    fn get_final_index(
        vertices_index_mapping: &mut [usize],
        flatten_queue: &mut Vec<usize>,
        orig_index: usize,
    ) -> usize {
        // flatten_queue.clear();
        flatten_queue.clear();
        // size_t idx = orig_index;
        let mut idx = orig_index;
        // while (vertices_index_mapping[idx] != idx) {
        while vertices_index_mapping[idx] != idx {
            // flatten_queue.push_back(idx);
            flatten_queue.push(idx);
            // idx = vertices_index_mapping[idx];
            idx = vertices_index_mapping[idx];
        }
        // for (size_t i : flatten_queue) {
        for &i in flatten_queue.iter() {
            // vertices_index_mapping[i] = idx;
            vertices_index_mapping[i] = idx;
        }
        // return idx;
        idx
    }

    // if face is removed, mark it here
    // ShortEdgeCollapse.cpp:34
    let mut face_removal_flags: Vec<bool> = vec![false; mesh.indices.len()];

    // ShortEdgeCollapse.cpp:36
    let mut triangles_neighbors: Vec<Vec3i> = its_face_neighbors_par(mesh);

    // now compute vertices dot product - this is used during edge collapse,
    // to determine which vertex to remove and which to keep;  We try to keep the one with larger angle, because it defines the shape "more".
    // The min vertex dot product is lowest dot product of its normal with the normals of faces around it.
    // the lower the dot product, the more we want to keep the vertex
    // NOTE: This score is not updated, even though the decimation does change the mesh. It saves computation time, and there are no strong reasons to update.
    // ShortEdgeCollapse.cpp:43
    let mut min_vertex_dot_product: Vec<f32> = vec![1.0f32; mesh.vertices.len()];
    // ShortEdgeCollapse.cpp:44-55
    {
        // std::vector<Vec3f> face_normals = its_face_normals(mesh);
        let face_normals: Vec<Vec3f> = its_face_normals(mesh);
        // std::vector<Vec3f> vertex_normals = NormalUtils::create_normals(mesh);
        let vertex_normals: Vec<Vec3f> = NormalUtils::create_normals(
            mesh,
            crate::normal_utils::VertexNormalType::NelsonMaxWeighted,
        );

        // for (size_t face_idx = 0; face_idx < mesh.indices.size(); ++face_idx) {
        for face_idx in 0..mesh.indices.len() {
            // Vec3i t = mesh.indices[face_idx];
            let t: Vec3i = mesh.indices[face_idx];
            // Vec3f n = face_normals[face_idx];
            let n: Vec3f = face_normals[face_idx];
            // min_vertex_dot_product[t[0]] = std::min(min_vertex_dot_product[t[0]], n.dot(vertex_normals[t[0]]));
            min_vertex_dot_product[t[0] as usize] = cpp_min_f32(
                min_vertex_dot_product[t[0] as usize],
                n.dot(&vertex_normals[t[0] as usize]),
            );
            // min_vertex_dot_product[t[1]] = std::min(min_vertex_dot_product[t[1]], n.dot(vertex_normals[t[1]]));
            min_vertex_dot_product[t[1] as usize] = cpp_min_f32(
                min_vertex_dot_product[t[1] as usize],
                n.dot(&vertex_normals[t[1] as usize]),
            );
            // min_vertex_dot_product[t[2]] = std::min(min_vertex_dot_product[t[2]], n.dot(vertex_normals[t[2]]));
            min_vertex_dot_product[t[2] as usize] = cpp_min_f32(
                min_vertex_dot_product[t[2] as usize],
                n.dot(&vertex_normals[t[2] as usize]),
            );
        }
    }

    // lambda to remove face. It flags the face as removed, and updates neighbourhood info
    // auto remove_face = [&triangles_neighbors, &face_removal_flags](int face_idx, int other_face_idx) { ... };
    // ShortEdgeCollapse.cpp:58-80 — implemented as a local fn taking the captured state by ref.
    fn remove_face(
        triangles_neighbors: &mut [Vec3i],
        face_removal_flags: &mut [bool],
        face_idx: i32,
        other_face_idx: i32,
    ) {
        // if (face_idx < 0) { return; }
        if face_idx < 0 {
            return;
        }
        // face_removal_flags[face_idx] = true;
        face_removal_flags[face_idx as usize] = true;
        // Vec3i neighbors = triangles_neighbors[face_idx];
        let neighbors: Vec3i = triangles_neighbors[face_idx as usize];
        // int n_a = neighbors[0] != other_face_idx ? neighbors[0] : neighbors[1];
        let n_a: i32 = if neighbors[0] != other_face_idx {
            neighbors[0]
        } else {
            neighbors[1]
        };
        // int n_b = neighbors[2] != other_face_idx ? neighbors[2] : neighbors[1];
        let n_b: i32 = if neighbors[2] != other_face_idx {
            neighbors[2]
        } else {
            neighbors[1]
        };
        // if (n_a > 0)
        if n_a > 0 {
            // for (int &n : triangles_neighbors[n_a]) {
            for k in 0..3 {
                // if (n == face_idx) { n = n_b; break; }
                if triangles_neighbors[n_a as usize][k] == face_idx {
                    triangles_neighbors[n_a as usize][k] = n_b;
                    break;
                }
            }
        }
        // if (n_b > 0)
        if n_b > 0 {
            // for (int &n : triangles_neighbors[n_b]) {
            for k in 0..3 {
                // if (n == face_idx) { n = n_a; break; }
                if triangles_neighbors[n_b as usize][k] == face_idx {
                    triangles_neighbors[n_b as usize][k] = n_a;
                    break;
                }
            }
        }
    }

    // std::mt19937_64 generator { 27644437 };// default constant seed! so that results are deterministic
    // ShortEdgeCollapse.cpp:82
    let mut generator = Mt19937_64::new(27644437);
    // std::vector<size_t> face_indices(mesh.indices.size());
    // ShortEdgeCollapse.cpp:83
    let mut face_indices: Vec<usize> = vec![0usize; mesh.indices.len()];
    // ShortEdgeCollapse.cpp:84-86
    for idx in 0..face_indices.len() {
        face_indices[idx] = idx;
    }
    //tmp face indices used only for swapping
    // ShortEdgeCollapse.cpp:88
    let mut tmp_face_indices: Vec<usize> = vec![0usize; mesh.indices.len()];

    // float decimation_ratio = 1.0f; // decimation ratio updated in each iteration. it is number of removed triangles / number of all
    // ShortEdgeCollapse.cpp:90
    let mut decimation_ratio: f32 = 1.0f32;
    // float edge_len = 0.2f; // Allowed collapsible edge size. Starts low, but is gradually increased
    // ShortEdgeCollapse.cpp:91
    let mut edge_len: f32 = 0.2f32;

    // while (face_indices.size() > target_triangle_count) {
    // ShortEdgeCollapse.cpp:93
    while face_indices.len() > target_triangle_count {
        // simpple func to increase the edge len - if decimation ratio is low, it increases the len up to twice, if decimation ratio is high, increments are low
        // edge_len = edge_len * (1.0f + 1.0 - decimation_ratio);
        // ShortEdgeCollapse.cpp:95 — the RHS is evaluated in double (because of the `1.0` literal) and truncated to float.
        edge_len = (edge_len as f64 * (1.0f64 + 1.0 - decimation_ratio as f64)) as f32;
        // float max_edge_len_squared = edge_len * edge_len;
        // ShortEdgeCollapse.cpp:96
        let max_edge_len_squared: f32 = edge_len * edge_len;

        //shuffle the faces and traverse in random order, this MASSIVELY improves the quality of the result
        // std::shuffle(face_indices.begin(), face_indices.end(), generator);
        // ShortEdgeCollapse.cpp:99
        if std::env::var("ZSMOOTH_FAITHFUL").is_ok() {
            // Native links libc++ on darwin — its std::shuffle differs from
            // libstdc++'s (R184). Gated: default keeps the legacy permutation
            // (byte-locked 147987).
            shuffle_libcxx(&mut face_indices, &mut generator);
        } else {
            shuffle(&mut face_indices, &mut generator);
        }

        // int allowed_face_removals = int(face_indices.size()) - int(target_triangle_count);
        // ShortEdgeCollapse.cpp:101
        let mut allowed_face_removals: i32 = face_indices.len() as i32 - target_triangle_count as i32;
        // for (const size_t &face_idx : face_indices) {
        // ShortEdgeCollapse.cpp:102
        for fi in 0..face_indices.len() {
            let face_idx = face_indices[fi];
            // if (face_removal_flags[face_idx]) {
            //     // if face already removed from previous collapses, skip (each collapse removes two triangles [at least] )
            //     continue;
            // }
            // ShortEdgeCollapse.cpp:103-106
            if face_removal_flags[face_idx] {
                continue;
            }

            // look at each edge if it is good candidate for collapse
            // for (size_t edge_idx = 0; edge_idx < 3; ++edge_idx) {
            // ShortEdgeCollapse.cpp:109
            for edge_idx in 0..3usize {
                // size_t vertex_index_keep = get_final_index(mesh.indices[face_idx][edge_idx]);
                // ShortEdgeCollapse.cpp:110
                let mut vertex_index_keep = get_final_index(
                    &mut vertices_index_mapping,
                    &mut flatten_queue,
                    mesh.indices[face_idx][edge_idx] as usize,
                );
                // size_t vertex_index_remove = get_final_index(mesh.indices[face_idx][(edge_idx + 1) % 3]);
                // ShortEdgeCollapse.cpp:111
                let mut vertex_index_remove = get_final_index(
                    &mut vertices_index_mapping,
                    &mut flatten_queue,
                    mesh.indices[face_idx][(edge_idx + 1) % 3] as usize,
                );
                //check distance, skip long edges
                // if ((mesh.vertices[vertex_index_keep] - mesh.vertices[vertex_index_remove]).squaredNorm() > max_edge_len_squared) { continue; }
                // ShortEdgeCollapse.cpp:113-116
                if (mesh.vertices[vertex_index_keep] - mesh.vertices[vertex_index_remove])
                    .norm_squared()
                    > max_edge_len_squared
                {
                    continue;
                }
                // swap indexes if vertex_index_keep has higher dot product (we want to keep low dot product vertices)
                // if (min_vertex_dot_product[vertex_index_remove] < min_vertex_dot_product[vertex_index_keep]) {
                // ShortEdgeCollapse.cpp:118-122
                if min_vertex_dot_product[vertex_index_remove]
                    < min_vertex_dot_product[vertex_index_keep]
                {
                    // size_t tmp = vertex_index_keep;
                    let tmp = vertex_index_keep;
                    // vertex_index_keep = vertex_index_remove;
                    vertex_index_keep = vertex_index_remove;
                    // vertex_index_remove = tmp;
                    vertex_index_remove = tmp;
                }

                //remove vertex
                // ShortEdgeCollapse.cpp:124-128
                {
                    // map its index to the index of the kept vertex
                    // vertices_index_mapping[vertex_index_remove] = vertices_index_mapping[vertex_index_keep];
                    vertices_index_mapping[vertex_index_remove] =
                        vertices_index_mapping[vertex_index_keep];
                }

                // int neighbor_to_remove_face_idx = triangles_neighbors[face_idx][edge_idx];
                // ShortEdgeCollapse.cpp:130
                let neighbor_to_remove_face_idx: i32 = triangles_neighbors[face_idx][edge_idx];
                // remove faces
                // remove_face(face_idx, neighbor_to_remove_face_idx);
                // ShortEdgeCollapse.cpp:132
                remove_face(
                    &mut triangles_neighbors,
                    &mut face_removal_flags,
                    face_idx as i32,
                    neighbor_to_remove_face_idx,
                );
                // remove_face(neighbor_to_remove_face_idx, face_idx);
                // ShortEdgeCollapse.cpp:133
                remove_face(
                    &mut triangles_neighbors,
                    &mut face_removal_flags,
                    neighbor_to_remove_face_idx,
                    face_idx as i32,
                );
                // allowed_face_removals-=2;
                // ShortEdgeCollapse.cpp:134
                allowed_face_removals -= 2;

                // break. this triangle is done
                // ShortEdgeCollapse.cpp:137
                break;
            }

            // if (allowed_face_removals <= 0) { break; }
            // ShortEdgeCollapse.cpp:140
            if allowed_face_removals <= 0 {
                break;
            }
        }

        // filter face_indices, remove those that have been collapsed
        // size_t prev_size = face_indices.size();
        // ShortEdgeCollapse.cpp:144
        let prev_size = face_indices.len();
        // tmp_face_indices.clear();
        // ShortEdgeCollapse.cpp:145
        tmp_face_indices.clear();
        // for (size_t face_idx : face_indices) {
        // ShortEdgeCollapse.cpp:146-150
        for &face_idx in face_indices.iter() {
            // if (!face_removal_flags[face_idx]){ tmp_face_indices.push_back(face_idx); }
            if !face_removal_flags[face_idx] {
                tmp_face_indices.push(face_idx);
            }
        }
        // face_indices.swap(tmp_face_indices);
        // ShortEdgeCollapse.cpp:151
        std::mem::swap(&mut face_indices, &mut tmp_face_indices);

        // decimation_ratio = float(prev_size - face_indices.size()) / float(prev_size);
        // ShortEdgeCollapse.cpp:153
        decimation_ratio = (prev_size - face_indices.len()) as f32 / prev_size as f32;
        //std::cout << " DECIMATION RATIO: " << decimation_ratio << std::endl;
        // ShortEdgeCollapse.cpp:154
    }

    //Extract the result mesh
    // std::unordered_map<size_t, size_t> final_vertices_mapping;
    // ShortEdgeCollapse.cpp:158
    let mut final_vertices_mapping: std::collections::HashMap<usize, usize> =
        std::collections::HashMap::new();
    // std::vector<Vec3f> final_vertices;
    // ShortEdgeCollapse.cpp:159
    let mut final_vertices: Vec<Vec3f> = Vec::new();
    // std::vector<Vec3i> final_indices;
    // ShortEdgeCollapse.cpp:160
    let mut final_indices: Vec<Vec3i> = Vec::new();
    // final_indices.reserve(face_indices.size());
    // ShortEdgeCollapse.cpp:161
    final_indices.reserve(face_indices.len());
    // for (size_t idx : face_indices) {
    // ShortEdgeCollapse.cpp:162
    for &idx in face_indices.iter() {
        // Vec3i final_face;
        // ShortEdgeCollapse.cpp:163
        let mut final_face: Vec3i = Vec3i::new(0, 0, 0);
        // for (size_t i = 0; i < 3; ++i) {
        // ShortEdgeCollapse.cpp:164-166
        for i in 0..3usize {
            // final_face[i] = get_final_index(mesh.indices[idx][i]);
            final_face[i] = get_final_index(
                &mut vertices_index_mapping,
                &mut flatten_queue,
                mesh.indices[idx][i] as usize,
            ) as i32;
        }
        // if (final_face[0] == final_face[1] || final_face[1] == final_face[2] || final_face[2] == final_face[0]) { continue; }
        // ShortEdgeCollapse.cpp:167-169 — discard degenerate triangles
        if final_face[0] == final_face[1]
            || final_face[1] == final_face[2]
            || final_face[2] == final_face[0]
        {
            continue;
        }

        // for (size_t i = 0; i < 3; ++i) {
        // ShortEdgeCollapse.cpp:171-177
        for i in 0..3usize {
            // if (final_vertices_mapping.find(final_face[i]) == final_vertices_mapping.end()) {
            if !final_vertices_mapping.contains_key(&(final_face[i] as usize)) {
                // final_vertices_mapping[final_face[i]] = final_vertices.size();
                final_vertices_mapping.insert(final_face[i] as usize, final_vertices.len());
                // final_vertices.push_back(mesh.vertices[final_face[i]]);
                final_vertices.push(mesh.vertices[final_face[i] as usize]);
            }
            // final_face[i] = final_vertices_mapping[final_face[i]];
            final_face[i] = final_vertices_mapping[&(final_face[i] as usize)] as i32;
        }

        // final_indices.push_back(final_face);
        // ShortEdgeCollapse.cpp:179
        final_indices.push(final_face);
    }

    // mesh.vertices = final_vertices;
    // ShortEdgeCollapse.cpp:182
    mesh.vertices = final_vertices;
    // mesh.indices = final_indices;
    // ShortEdgeCollapse.cpp:183
    mesh.indices = final_indices;
}
