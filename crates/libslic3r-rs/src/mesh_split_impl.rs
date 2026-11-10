//! Faithful 1:1 port of `MeshSplitImpl.hpp` from BambuStudio libslic3r.
//!
//! C++ Reference:
//! - src/libslic3r/MeshSplitImpl.hpp (347 lines, header-only)
//!
//! Splits a mesh into multiple connected meshes, builds the per-face neighbor
//! index, and queries patch counts / splittability.
//!
//! Fidelity notes (byte-exact G-code parity):
//! - `coord_t->i64`, `coordf_t->f64`. Mesh vertices are `Vec3f` (Eigen
//!   `Matrix<float,3,1>`) and triangle indices `Vec3i` (Eigen `Matrix<int,3,1>`),
//!   matching `indexed_triangle_set`. All vector math is kept in `f32`.
//! - The C++ is a set of class/function templates parameterized on `Its` and the
//!   `ExPolicy`. In this codebase the only instantiation is `Its = indexed_triangle_set`
//!   (resolved through `ItsWithNeighborsIndex_<indexed_triangle_set>`) with the index
//!   `std::vector<Vec3i>` produced by `create_face_neighbors_index(ex_tbb, its)`. The
//!   port therefore monomorphizes those templates against `indexed_triangle_set` and
//!   `&[Vec3i]`, preserving the original control flow, ordering and edge cases.
//! - `create_face_neighbors_index` is ported as the deterministic sequential traversal:
//!   the `ExPolicy` argument selects only the execution strategy (`ex_seq` vs `ex_tbb`),
//!   and the result is order-independent because the per-face work only reads shared
//!   state and writes disjoint `neighbors[other_face][vertex_index]` slots under the
//!   `!= no_value` guard. (`its_face_neighbors_par` in `ShortEdgeCollapse` relies on the
//!   same property.)
//! - `std::find` of the first unvisited facet, the `std::sort(facets)` before emitting a
//!   part, and the `vidx_conv` part-id remapping are all reproduced exactly so that the
//!   emitted vertex/face ordering matches the C++ byte-for-byte.

use crate::execution::execution::max_concurrency;
use crate::execution::ExecutionPolicy;
use crate::normal_utils::indexed_triangle_set;

use nalgebra::{Vector2, Vector3};
use std::collections::HashMap;

/// 3D integer index vector, mirroring C++ `Vec3i` (Eigen `Matrix<int,3,1>`).
/// Point.hpp
type Vec3i = Vector3<i32>;
/// 2D integer vector, mirroring C++ `Vec2i` (Eigen `Matrix<int,2,1>`).
/// Point.hpp
type Vec2i = Vector2<i32>;

// ---------------------------------------------------------------------------
// Dependencies ported from TriangleMesh.{hpp,cpp}
//
// These triangle edge/vertex helpers and the `VertexFaceIndex` live in
// TriangleMesh.{hpp,cpp}; they are required by `create_face_neighbors_index`
// (MeshSplitImpl.hpp:293-342). They are reproduced here as private helpers with
// their original `// <File>:NNN` references so this module builds standalone.
// (A second faithful copy lives in `short_edge_collapse`, which predates this
// module; the two are intentionally identical.)
// ---------------------------------------------------------------------------

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

// MeshSplitImpl.hpp:10-11
// template<class ExPolicy>
// std::vector<Vec3i> create_face_neighbors_index(ExPolicy &&ex, const indexed_triangle_set &its);

// MeshSplitImpl.hpp:13 — namespace meshsplit_detail
pub mod meshsplit_detail {
    use super::*;

    // MeshSplitImpl.hpp:15-29
    // The `ItsWithNeighborsIndex_<Its>` traits class extracts the `indexed_triangle_set`
    // and its neighbor index out of the generic `Its` wrapper. In this codebase the only
    // instantiation is `Its = indexed_triangle_set`, whose specialization (lines 22-29)
    // returns the ITS by reference and builds the index via
    // `create_face_neighbors_index(ex_tbb, its)`. These accessors are inlined directly at
    // the call sites of `its_split`/`its_is_splittable`/`its_number_of_patches` below.

    /// Discover connected patches of facets one by one.
    /// MeshSplitImpl.hpp:31-90
    ///
    /// `template<class NeighborIndex> struct NeighborVisitor`. The only `NeighborIndex`
    /// used in the codebase is `std::vector<Vec3i>` (i.e. `&[Vec3i]`), so the port
    /// monomorphizes against that. The constructor that takes ownership of the index
    /// (`NeighborVisitor(its, NeighborIndex &&)`, lines 39-43) is collapsed into the
    /// single borrowing form here, which is all the call sites need.
    pub struct NeighborVisitor<'a> {
        // const indexed_triangle_set  &its;
        #[allow(dead_code)]
        pub its: &'a indexed_triangle_set,
        // const NeighborIndex         &neighbor_index;
        pub neighbor_index: &'a [Vec3i],

        // std::vector<char>            m_visited;
        m_visited: Vec<bool>,

        // std::vector<stack_el>        m_facestack;
        // using stack_el = size_t;
        m_facestack: Vec<usize>,

        // Last face visited.
        // size_t                       m_seed { 0 };
        m_seed: usize,
    }

    impl<'a> NeighborVisitor<'a> {
        /// MeshSplitImpl.hpp:34-38
        /// `NeighborVisitor(const indexed_triangle_set &its, const NeighborIndex &neighbor_index)`
        pub fn new(its: &'a indexed_triangle_set, neighbor_index: &'a [Vec3i]) -> Self {
            let mut v = NeighborVisitor {
                its,
                neighbor_index,
                m_visited: Vec::new(),
                m_facestack: Vec::new(),
                m_seed: 0,
            };
            // m_visited.assign(its.indices.size(), false);
            v.m_visited.resize(its.indices.len(), false);
            // m_facestack.reserve(its.indices.size());
            v.m_facestack.reserve(its.indices.len());
            v
        }

        // MeshSplitImpl.hpp:85
        // void push(const stack_el &s) { m_facestack.emplace_back(s); }
        #[inline]
        fn push(&mut self, s: usize) {
            self.m_facestack.push(s);
        }

        // MeshSplitImpl.hpp:86
        // stack_el pop() { stack_el ret = m_facestack.back(); m_facestack.pop_back(); return ret; }
        #[inline]
        fn pop(&mut self) -> usize {
            self.m_facestack.pop().unwrap()
        }

        /// MeshSplitImpl.hpp:45-72
        /// `template<typename Visitor> void visit(Visitor visitor)`
        ///
        /// `visitor(idx)` returns `bool`: `true` to keep traversing, `false` to stop.
        pub fn visit<Visitor>(&mut self, mut visitor: Visitor)
        where
            Visitor: FnMut(usize) -> bool,
        {
            // MeshSplitImpl.hpp:48-50
            // find the next unvisited facet and push the index
            // auto facet = std::find(m_visited.begin() + m_seed, m_visited.end(), false);
            // m_seed = facet - m_visited.begin();
            let facet = self.m_visited[self.m_seed..]
                .iter()
                .position(|&v| !v)
                .map(|p| self.m_seed + p);
            self.m_seed = match facet {
                Some(pos) => pos,
                None => self.m_visited.len(),
            };

            // MeshSplitImpl.hpp:52
            // if (facet != m_visited.end()) {
            if facet.is_some() {
                // MeshSplitImpl.hpp:53-54
                // Skip this element in the next round.
                // auto idx = m_seed ++;
                let idx = self.m_seed;
                self.m_seed += 1;
                // MeshSplitImpl.hpp:55-56
                // if (! visitor(idx))
                //     return;
                if !visitor(idx) {
                    return;
                }
                // MeshSplitImpl.hpp:57
                // this->push(idx);
                self.push(idx);
                // MeshSplitImpl.hpp:58
                // m_visited[idx] = true;
                self.m_visited[idx] = true;
                // MeshSplitImpl.hpp:59
                // while (! m_facestack.empty()) {
                while !self.m_facestack.is_empty() {
                    // MeshSplitImpl.hpp:60
                    // size_t facet_idx = this->pop();
                    let facet_idx = self.pop();
                    // MeshSplitImpl.hpp:61
                    // for (auto neighbor_idx : neighbor_index[facet_idx]) {
                    for k in 0..3 {
                        let neighbor_idx = self.neighbor_index[facet_idx][k];
                        // MeshSplitImpl.hpp:62
                        // assert(neighbor_idx < int(m_visited.size()));
                        debug_assert!(neighbor_idx < self.m_visited.len() as i32);
                        // MeshSplitImpl.hpp:63
                        // if (neighbor_idx >= 0 && !m_visited[neighbor_idx]) {
                        if neighbor_idx >= 0 && !self.m_visited[neighbor_idx as usize] {
                            // MeshSplitImpl.hpp:64-65
                            // if (! visitor(size_t(neighbor_idx)))
                            //     return;
                            if !visitor(neighbor_idx as usize) {
                                return;
                            }
                            // MeshSplitImpl.hpp:66
                            // m_visited[neighbor_idx] = true;
                            self.m_visited[neighbor_idx as usize] = true;
                            // MeshSplitImpl.hpp:67
                            // this->push(stack_el(neighbor_idx));
                            self.push(neighbor_idx as usize);
                        }
                    }
                }
            }
        }
    }
} // namespace meshsplit_detail

// MeshSplitImpl.hpp:94-109
// `template<class IndexT> struct ItsNeighborsWrapper`
// "Funky wrapper for timinig of its_split() using various neighbor index creating
// methods, see sandboxes/its_neighbor_index/main.cpp". It is a thin alias bundling an
// `indexed_triangle_set` with a precomputed neighbor index. Ported as a borrowing
// wrapper; the owning-constructor overload (line 105) is unused in the codebase.
/// `template<class IndexT> struct ItsNeighborsWrapper`
/// MeshSplitImpl.hpp:95-109
pub struct ItsNeighborsWrapper<'a> {
    // const indexed_triangle_set &its;
    pub its: &'a indexed_triangle_set,
    // const IndexT               &index_ref;
    pub index_ref: &'a [Vec3i],
}

impl<'a> ItsNeighborsWrapper<'a> {
    /// `ItsNeighborsWrapper(const indexed_triangle_set &its, const IndexT &index)`
    /// MeshSplitImpl.hpp:103
    pub fn new(its: &'a indexed_triangle_set, index: &'a [Vec3i]) -> Self {
        // : its{its}, index_ref{index}
        ItsNeighborsWrapper {
            its,
            index_ref: index,
        }
    }

    /// `const auto& get_its() const noexcept { return its; }`
    /// MeshSplitImpl.hpp:107
    #[inline]
    pub fn get_its(&self) -> &indexed_triangle_set {
        self.its
    }

    /// `const auto& get_index() const noexcept { return index_ref; }`
    /// MeshSplitImpl.hpp:108
    #[inline]
    pub fn get_index(&self) -> &[Vec3i] {
        self.index_ref
    }
}

// MeshSplitImpl.hpp:111-124
// `template<class Fn> struct SplitOutputFn`
// "Can be used as the second argument to its_split to apply a functor on each part,
// instead of collecting them into a container." In Rust this role is filled directly by
// passing an `FnMut(indexed_triangle_set)` closure as the output sink to `its_split`
// (see the `OutputIt` callback parameter below), so a dedicated wrapper type is not
// needed; the C++ wrapper only existed to satisfy the OutputIterator interface.

/// Splits a mesh into multiple meshes when possible.
/// MeshSplitImpl.hpp:126-176
///
/// `template<class Its, class OutputIt> void its_split(const Its &m, OutputIt out_it)`.
/// The generic `Its` is monomorphized to the `ItsNeighborsWrapper` accessors (the
/// `ItsWithNeighborsIndex_` traits): `its` is the `indexed_triangle_set` and
/// `neighbor_index` the precomputed `Vec<Vec3i>`. The OutputIterator `out_it` becomes an
/// `FnMut(indexed_triangle_set)` sink that receives each emitted part.
pub fn its_split<OutputIt>(its: &indexed_triangle_set, neighbor_index: &[Vec3i], mut out_it: OutputIt)
where
    OutputIt: FnMut(indexed_triangle_set),
{
    // MeshSplitImpl.hpp:130
    // using namespace meshsplit_detail;
    use meshsplit_detail::NeighborVisitor;

    // MeshSplitImpl.hpp:132
    // const indexed_triangle_set &its = ItsWithNeighborsIndex_<Its>::get_its(m);

    // MeshSplitImpl.hpp:134-137
    // struct VertexConv { size_t part_id = max(); size_t vertex_image; };
    #[derive(Clone, Copy)]
    struct VertexConv {
        // size_t part_id = std::numeric_limits<size_t>::max();
        part_id: usize,
        // size_t vertex_image;
        vertex_image: usize,
    }
    // MeshSplitImpl.hpp:138
    // std::vector<VertexConv> vidx_conv(its.vertices.size());
    let mut vidx_conv: Vec<VertexConv> = vec![
        VertexConv {
            part_id: usize::MAX,
            vertex_image: 0,
        };
        its.vertices.len()
    ];

    // MeshSplitImpl.hpp:140
    // meshsplit_detail::NeighborVisitor visitor(its, ItsWithNeighborsIndex_<Its>::get_index(m));
    let mut visitor = NeighborVisitor::new(its, neighbor_index);

    // MeshSplitImpl.hpp:142
    // std::vector<size_t> facets;
    let mut facets: Vec<usize> = Vec::new();
    // MeshSplitImpl.hpp:143
    // for (size_t part_id = 0;; ++part_id) {
    let mut part_id: usize = 0;
    loop {
        // MeshSplitImpl.hpp:144-146
        // Collect all faces of the next patch.
        // facets.clear();
        facets.clear();
        // visitor.visit([&facets](size_t idx) { facets.emplace_back(idx); return true; });
        visitor.visit(|idx| {
            facets.push(idx);
            true
        });
        // MeshSplitImpl.hpp:147-148
        // if (facets.empty())
        //     break;
        if facets.is_empty() {
            break;
        }
        // MeshSplitImpl.hpp:149
        // std::sort(facets.begin(),facets.end());
        facets.sort_unstable();
        // MeshSplitImpl.hpp:150-153
        // Create a new mesh for the part that was just split off.
        // indexed_triangle_set mesh;
        let mut mesh = indexed_triangle_set::default();
        // mesh.indices.reserve(facets.size());
        mesh.indices.reserve(facets.len());
        // mesh.vertices.reserve(std::min(facets.size() * 3, its.vertices.size()));
        mesh.vertices
            .reserve(std::cmp::min(facets.len() * 3, its.vertices.len()));

        // MeshSplitImpl.hpp:155-171
        // Assign the facets to the new mesh.
        // for (size_t face_id : facets) {
        for &face_id in &facets {
            // const auto &face = its.indices[face_id];
            let face = its.indices[face_id];
            // Vec3i       new_face;
            let mut new_face = Vec3i::new(0, 0, 0);
            // for (size_t v = 0; v < 3; ++v) {
            for v in 0..3usize {
                // auto vi = face(v);
                let vi = face[v];

                // if (vidx_conv[vi].part_id != part_id) {
                if vidx_conv[vi as usize].part_id != part_id {
                    // vidx_conv[vi] = {part_id, mesh.vertices.size()};
                    vidx_conv[vi as usize] = VertexConv {
                        part_id,
                        vertex_image: mesh.vertices.len(),
                    };
                    // mesh.vertices.emplace_back(its.vertices[size_t(vi)]);
                    mesh.vertices.push(its.vertices[vi as usize]);
                }

                // new_face(v) = vidx_conv[vi].vertex_image;
                new_face[v] = vidx_conv[vi as usize].vertex_image as i32;
            }

            // mesh.indices.emplace_back(new_face);
            mesh.indices.push(new_face);
        }

        // MeshSplitImpl.hpp:173-174
        // *out_it = std::move(mesh);
        // ++out_it;
        out_it(mesh);

        part_id += 1;
    }
}

/// MeshSplitImpl.hpp:178-185
/// `template<class Its> std::vector<indexed_triangle_set> its_split(const Its &its)`
pub fn its_split_collect(
    its: &indexed_triangle_set,
    neighbor_index: &[Vec3i],
) -> Vec<indexed_triangle_set> {
    // MeshSplitImpl.hpp:181
    // auto ret = reserve_vector<indexed_triangle_set>(3);
    let mut ret: Vec<indexed_triangle_set> = Vec::with_capacity(3);
    // MeshSplitImpl.hpp:182
    // its_split(its, std::back_inserter(ret));
    its_split(its, neighbor_index, |mesh| ret.push(mesh));

    // MeshSplitImpl.hpp:184
    // return ret;
    ret
}

/// Splits a mesh into multiple meshes when possible.
/// MeshSplitImpl.hpp:187-241
///
/// `template<class Its, class OutputIt, class OutputIt_ship>
///  void its_split_and_keep_relationship(const Its &m, OutputIt out_it, OutputIt_ship out_ship)`.
/// Same as `its_split` but also records, for each emitted part, the
/// `new_face_index -> original_face_index` relationship as an `unordered_map<int,int>`.
pub fn its_split_and_keep_relationship<OutputIt, OutputItShip>(
    its: &indexed_triangle_set,
    neighbor_index: &[Vec3i],
    mut out_it: OutputIt,
    mut out_ship: OutputItShip,
) where
    OutputIt: FnMut(indexed_triangle_set),
    OutputItShip: FnMut(HashMap<i32, i32>),
{
    // MeshSplitImpl.hpp:191
    // using namespace meshsplit_detail;
    use meshsplit_detail::NeighborVisitor;

    // MeshSplitImpl.hpp:193
    // const indexed_triangle_set &its = ItsWithNeighborsIndex_<Its>::get_its(m);

    // MeshSplitImpl.hpp:195-199
    // struct VertexConv { size_t part_id = max(); size_t vertex_image; };
    #[derive(Clone, Copy)]
    struct VertexConv {
        // size_t part_id = std::numeric_limits<size_t>::max();
        part_id: usize,
        // size_t vertex_image;
        vertex_image: usize,
    }
    // MeshSplitImpl.hpp:200
    // std::vector<VertexConv> vidx_conv(its.vertices.size());
    let mut vidx_conv: Vec<VertexConv> = vec![
        VertexConv {
            part_id: usize::MAX,
            vertex_image: 0,
        };
        its.vertices.len()
    ];

    // MeshSplitImpl.hpp:202
    // meshsplit_detail::NeighborVisitor visitor(its, ItsWithNeighborsIndex_<Its>::get_index(m));
    let mut visitor = NeighborVisitor::new(its, neighbor_index);

    // MeshSplitImpl.hpp:204
    // std::vector<size_t> facets;
    let mut facets: Vec<usize> = Vec::new();
    // MeshSplitImpl.hpp:205
    // for (size_t part_id = 0;; ++part_id) {
    let mut part_id: usize = 0;
    loop {
        // MeshSplitImpl.hpp:206-211
        // Collect all faces of the next patch.
        // facets.clear();
        facets.clear();
        // visitor.visit([&facets](size_t idx) { facets.emplace_back(idx); return true; });
        visitor.visit(|idx| {
            facets.push(idx);
            true
        });
        // MeshSplitImpl.hpp:212
        // if (facets.empty()) break;
        if facets.is_empty() {
            break;
        }
        // MeshSplitImpl.hpp:213
        // std::sort(facets.begin(), facets.end());
        facets.sort_unstable();
        // MeshSplitImpl.hpp:214-217
        // Create a new mesh for the part that was just split off.
        // indexed_triangle_set mesh;
        let mut mesh = indexed_triangle_set::default();
        // mesh.indices.reserve(facets.size());
        mesh.indices.reserve(facets.len());
        // mesh.vertices.reserve(std::min(facets.size() * 3, its.vertices.size()));
        mesh.vertices
            .reserve(std::cmp::min(facets.len() * 3, its.vertices.len()));
        // MeshSplitImpl.hpp:218
        // std::unordered_map<int, int> relationship;
        let mut relationship: HashMap<i32, i32> = HashMap::new();
        // MeshSplitImpl.hpp:219-235
        // Assign the facets to the new mesh.
        // for (size_t face_id : facets) {
        for &face_id in &facets {
            // const auto &face = its.indices[face_id];
            let face = its.indices[face_id];
            // Vec3i       new_face;
            let mut new_face = Vec3i::new(0, 0, 0);
            // for (size_t v = 0; v < 3; ++v) {
            for v in 0..3usize {
                // auto vi = face(v);
                let vi = face[v];

                // if (vidx_conv[vi].part_id != part_id) {
                if vidx_conv[vi as usize].part_id != part_id {
                    // vidx_conv[vi] = {part_id, mesh.vertices.size()};
                    vidx_conv[vi as usize] = VertexConv {
                        part_id,
                        vertex_image: mesh.vertices.len(),
                    };
                    // mesh.vertices.emplace_back(its.vertices[size_t(vi)]);
                    mesh.vertices.push(its.vertices[vi as usize]);
                }

                // new_face(v) = vidx_conv[vi].vertex_image;
                new_face[v] = vidx_conv[vi as usize].vertex_image as i32;
            }
            // relationship[mesh.indices.size()] = face_id;
            relationship.insert(mesh.indices.len() as i32, face_id as i32);
            // mesh.indices.emplace_back(new_face);
            mesh.indices.push(new_face);
        }

        // MeshSplitImpl.hpp:237-239
        // *out_it   = std::move(mesh);
        // *out_ship = std::move(relationship);
        // ++out_it;
        out_it(mesh);
        out_ship(relationship);

        part_id += 1;
    }
}

/// `class MeshAndShip`
/// MeshSplitImpl.hpp:242-247
pub struct MeshAndShip {
    // std::vector<indexed_triangle_set> itses;
    pub itses: Vec<indexed_triangle_set>,
    // std::vector<std::unordered_map<int, int>> ships;
    pub ships: Vec<HashMap<i32, i32>>,
}

/// MeshSplitImpl.hpp:249-260
/// `template<class Its> MeshAndShip its_split_and_save_relationship(const Its &its)`
pub fn its_split_and_save_relationship(
    its: &indexed_triangle_set,
    neighbor_index: &[Vec3i],
) -> MeshAndShip {
    // MeshSplitImpl.hpp:252
    // auto ret      = reserve_vector<indexed_triangle_set>(3);
    let mut ret: Vec<indexed_triangle_set> = Vec::with_capacity(3);
    // MeshSplitImpl.hpp:253
    // auto ret_ship = reserve_vector<std::unordered_map<int, int>>(3);
    let mut ret_ship: Vec<HashMap<i32, i32>> = Vec::with_capacity(3);

    // MeshSplitImpl.hpp:255
    // its_split_and_keep_relationship(its, std::back_inserter(ret), std::back_inserter(ret_ship));
    its_split_and_keep_relationship(
        its,
        neighbor_index,
        |mesh| ret.push(mesh),
        |ship| ret_ship.push(ship),
    );
    // MeshSplitImpl.hpp:256-259
    // MeshAndShip mesh_ship;
    // mesh_ship.itses = ret;
    // mesh_ship.ships = ret_ship;
    // return mesh_ship;
    MeshAndShip {
        itses: ret,
        ships: ret_ship,
    }
}

/// MeshSplitImpl.hpp:262-274
/// `template<class Its> bool its_is_splittable(const Its &m)`
pub fn its_is_splittable(its: &indexed_triangle_set, neighbor_index: &[Vec3i]) -> bool {
    // MeshSplitImpl.hpp:265
    // meshsplit_detail::NeighborVisitor visitor(ItsWithNeighborsIndex_<Its>::get_its(m), ItsWithNeighborsIndex_<Its>::get_index(m));
    let mut visitor = meshsplit_detail::NeighborVisitor::new(its, neighbor_index);
    // MeshSplitImpl.hpp:266
    // bool has_some = false;
    let mut has_some = false;
    // MeshSplitImpl.hpp:267
    // bool has_some2 = false;
    let mut has_some2 = false;
    // MeshSplitImpl.hpp:268-269
    // Traverse the 1st patch fully.
    // visitor.visit([&has_some](size_t idx) { has_some = true; return true; });
    visitor.visit(|_idx| {
        has_some = true;
        true
    });
    // MeshSplitImpl.hpp:270-272
    // if (has_some)
    //     // Just check whether there is any face of the 2nd patch.
    //     visitor.visit([&has_some2](size_t idx) { has_some2 = true; return false; });
    if has_some {
        visitor.visit(|_idx| {
            has_some2 = true;
            false
        });
    }
    // MeshSplitImpl.hpp:273
    // return has_some && has_some2;
    has_some && has_some2
}

/// MeshSplitImpl.hpp:276-290
/// `template<class Its> size_t its_number_of_patches(const Its &m)`
pub fn its_number_of_patches(its: &indexed_triangle_set, neighbor_index: &[Vec3i]) -> usize {
    // MeshSplitImpl.hpp:279
    // meshsplit_detail::NeighborVisitor visitor(ItsWithNeighborsIndex_<Its>::get_its(m), ItsWithNeighborsIndex_<Its>::get_index(m));
    let mut visitor = meshsplit_detail::NeighborVisitor::new(its, neighbor_index);
    // MeshSplitImpl.hpp:280
    // size_t num_patches = 0;
    let mut num_patches: usize = 0;
    // MeshSplitImpl.hpp:281
    // for (;;) {
    loop {
        // MeshSplitImpl.hpp:282
        // bool has_some = false;
        let mut has_some = false;
        // MeshSplitImpl.hpp:283-284
        // Traverse the 1st patch fully.
        // visitor.visit([&has_some](size_t idx) { has_some = true; return true; });
        visitor.visit(|_idx| {
            has_some = true;
            true
        });
        // MeshSplitImpl.hpp:285-286
        // if (! has_some)
        //     break;
        if !has_some {
            break;
        }
        // MeshSplitImpl.hpp:287
        // ++ num_patches;
        num_patches += 1;
    }
    // MeshSplitImpl.hpp:289
    // return num_patches;
    num_patches
}

/// `template<class ExPolicy> std::vector<Vec3i> create_face_neighbors_index(ExPolicy &&ex, const indexed_triangle_set &its)`
/// MeshSplitImpl.hpp:292-342
///
/// Ported as the deterministic sequential traversal: the `ExPolicy` argument selects
/// only the execution strategy (`ex_seq` vs `ex_tbb`), and `create_face_neighbors_index`
/// produces the same result regardless because the per-face work only reads shared
/// state and writes disjoint `neighbors[other_face][vertex_index]` slots with the
/// `!= no_value` guard, so the result is order-independent.
pub fn create_face_neighbors_index<EP: ExecutionPolicy>(
    ex: &EP,
    its: &indexed_triangle_set,
) -> Vec<Vec3i> {
    // MeshSplitImpl.hpp:295
    // const std::vector<stl_triangle_vertex_indices> &indices = its.indices;
    let indices = &its.indices;

    // MeshSplitImpl.hpp:297
    // if (indices.empty()) return {};
    if indices.is_empty() {
        return Vec::new();
    }

    // MeshSplitImpl.hpp:299
    // assert(! its.vertices.empty());
    debug_assert!(!its.vertices.is_empty());

    // MeshSplitImpl.hpp:301
    // auto vertex_triangles = VertexFaceIndex{its};
    let vertex_triangles = VertexFaceIndex::new(its);
    // MeshSplitImpl.hpp:302
    // static constexpr int no_value = -1;
    const NO_VALUE: i32 = -1;
    // MeshSplitImpl.hpp:303-304
    // std::vector<Vec3i> neighbors(indices.size(), Vec3i(no_value, no_value, no_value));
    let mut neighbors: Vec<Vec3i> = vec![Vec3i::new(NO_VALUE, NO_VALUE, NO_VALUE); indices.len()];

    // MeshSplitImpl.hpp:306
    // //for (int face_idx = 0; face_idx < indices.size(); face_idx++) {
    // MeshSplitImpl.hpp:307-339
    // execution::for_each(ex, size_t(0), indices.size(), [&] (size_t face_idx) { ... }, execution::max_concurrency(ex));
    //
    // Ported as a sequential loop (see module/function doc): the result is independent
    // of the execution policy. `max_concurrency(ex)` is still evaluated to mirror the
    // call and honor the `ExPolicy` argument.
    let _grain = max_concurrency(ex);
    for face_idx in 0..indices.len() {
        // MeshSplitImpl.hpp:310-311
        // Vec3i& neighbor = neighbors[face_idx];
        // const stl_triangle_vertex_indices & triangle_indices = indices[face_idx];
        let triangle_indices = indices[face_idx];
        // MeshSplitImpl.hpp:312
        // for (int edge_index = 0; edge_index < 3; ++edge_index) {
        for edge_index in 0..3usize {
            // MeshSplitImpl.hpp:313-317
            // check if done
            // int& neighbor_edge = neighbor[edge_index];
            // if (neighbor_edge != no_value)
            //     // This edge already has a neighbor assigned.
            //     continue;
            if neighbors[face_idx][edge_index] != NO_VALUE {
                continue;
            }
            // MeshSplitImpl.hpp:318
            // Vec2i edge_indices = its_triangle_edge(triangle_indices, edge_index);
            let edge_indices = its_triangle_edge(&triangle_indices, edge_index as i32);
            // MeshSplitImpl.hpp:319
            // IMPROVE: use same vector for 2 sides of triangle
            // MeshSplitImpl.hpp:320
            // for (const size_t other_face : vertex_triangles[edge_indices[0]]) {
            for &other_face in vertex_triangles.faces(edge_indices[0] as usize) {
                // MeshSplitImpl.hpp:321
                // if (other_face <= face_idx) continue;
                if other_face <= face_idx {
                    continue;
                }
                // MeshSplitImpl.hpp:322
                // const stl_triangle_vertex_indices &face_indices = indices[other_face];
                let face_indices = indices[other_face];
                // MeshSplitImpl.hpp:323
                // int vertex_index = its_triangle_vertex_index(face_indices, edge_indices[1]);
                let vertex_index = its_triangle_vertex_index(&face_indices, edge_indices[1]);
                // MeshSplitImpl.hpp:324-325
                // NOT Contain second vertex?
                // if (vertex_index < 0) continue;
                if vertex_index < 0 {
                    continue;
                }
                // MeshSplitImpl.hpp:326-327
                // Has NOT oposite direction?
                // if (edge_indices[0] != face_indices[(vertex_index + 1) % 3]) continue;
                if edge_indices[0] != face_indices[((vertex_index + 1) % 3) as usize] {
                    continue;
                }
                // MeshSplitImpl.hpp:328-330
                //BBS: if this neighbor has already marked before, skip it
                // if (neighbors[other_face][vertex_index] != no_value)
                //     continue;
                if neighbors[other_face][vertex_index as usize] != NO_VALUE {
                    continue;
                }
                // MeshSplitImpl.hpp:331-333
                //BBS: the same triangle with opposite direction, also treat it as open edges
                //if (its_triangle_vertex_the_same(face_indices, triangle_indices))
                //    continue;
                // MeshSplitImpl.hpp:334
                // neighbor_edge = other_face;
                neighbors[face_idx][edge_index] = other_face as i32;
                // MeshSplitImpl.hpp:335
                // neighbors[other_face][vertex_index] = face_idx;
                neighbors[other_face][vertex_index as usize] = face_idx as i32;
                // MeshSplitImpl.hpp:336
                // break;
                break;
            }
        }
    }

    // MeshSplitImpl.hpp:341
    // return neighbors;
    neighbors
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::execution::EX_TBB;
    use crate::normal_utils::{StlTriangleVertexIndices, StlVertex};

    // Build a unit triangle (single connected patch).
    fn single_triangle() -> indexed_triangle_set {
        indexed_triangle_set {
            vertices: vec![
                StlVertex::new(0.0, 0.0, 0.0),
                StlVertex::new(1.0, 0.0, 0.0),
                StlVertex::new(0.0, 1.0, 0.0),
            ],
            indices: vec![StlTriangleVertexIndices::new(0, 1, 2)],
        }
    }

    // Build two disjoint triangles (two patches).
    fn two_disjoint_triangles() -> indexed_triangle_set {
        indexed_triangle_set {
            vertices: vec![
                StlVertex::new(0.0, 0.0, 0.0),
                StlVertex::new(1.0, 0.0, 0.0),
                StlVertex::new(0.0, 1.0, 0.0),
                StlVertex::new(10.0, 0.0, 0.0),
                StlVertex::new(11.0, 0.0, 0.0),
                StlVertex::new(10.0, 1.0, 0.0),
            ],
            indices: vec![
                StlTriangleVertexIndices::new(0, 1, 2),
                StlTriangleVertexIndices::new(3, 4, 5),
            ],
        }
    }

    #[test]
    fn empty_index_is_empty() {
        let its = indexed_triangle_set::default();
        let n = create_face_neighbors_index(&EX_TBB, &its);
        assert!(n.is_empty());
    }

    #[test]
    fn single_triangle_has_no_neighbors() {
        let its = single_triangle();
        let n = create_face_neighbors_index(&EX_TBB, &its);
        assert_eq!(n.len(), 1);
        assert_eq!(n[0], Vec3i::new(-1, -1, -1));
        assert_eq!(its_number_of_patches(&its, &n), 1);
        assert!(!its_is_splittable(&its, &n));
    }

    #[test]
    fn two_disjoint_triangles_split() {
        let its = two_disjoint_triangles();
        let n = create_face_neighbors_index(&EX_TBB, &its);
        assert_eq!(its_number_of_patches(&its, &n), 2);
        assert!(its_is_splittable(&its, &n));

        let parts = its_split_collect(&its, &n);
        assert_eq!(parts.len(), 2);
        for p in &parts {
            assert_eq!(p.indices.len(), 1);
            assert_eq!(p.vertices.len(), 3);
        }
    }

    #[test]
    fn split_and_relationship() {
        let its = two_disjoint_triangles();
        let n = create_face_neighbors_index(&EX_TBB, &its);
        let ms = its_split_and_save_relationship(&its, &n);
        assert_eq!(ms.itses.len(), 2);
        assert_eq!(ms.ships.len(), 2);
        // First emitted part is the patch containing face 0 (its.indices is sorted).
        assert_eq!(ms.ships[0].get(&0).copied(), Some(0));
        assert_eq!(ms.ships[1].get(&0).copied(), Some(1));
    }
}
