//! Faithful 1:1 port of `SurfaceMesh.hpp` from BambuStudio libslic3r.
//!
//! C++ Reference:
//! - src/libslic3r/SurfaceMesh.hpp
//!
//! ///|/ Copyright (c) Prusa Research 2022 Lukáš Matěna @lukasmatena
//! ///|/
//! ///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
//!
//! Header-only file: it declares a half-edge style adjacency view over an
//! `indexed_triangle_set`. There is no corresponding `.cpp`; every member is
//! defined inline in the header, so this module is a direct translation of the
//! inline definitions, in source order.
//!
//! Fidelity notes (byte-exact G-code parity):
//! - `Face_index` is C++ `enum Face_index : int`, used as a signed integer index
//!   that may be `-1` for "invalid"/"border". We mirror it as a `#[repr(i32)]`-like
//!   newtype around `i32` so that `int(face_idx) < 0` and `Face_index(-1)` semantics
//!   are preserved exactly.
//! - `m_face_neighbors` is `std::vector<Vec3i>`; we reuse the crate's `Vec3i`
//!   (`nalgebra::Vector3<i32>`), matching the C++ element type and `[]` access.
//! - `point()` returns a reference to `stl_vertex` (= `Vec3f`), so we return `&Vec3f`.
//! - The C++ constructor computes `m_face_neighbors` via `its_face_neighbors_par(its)`
//!   (declared in `TriangleMesh.hpp`, implemented in `TriangleMesh.cpp`). That free
//!   function is ported in this `indexed_triangle_set` domain as
//!   `crate::measure::its_face_neighbors_par`, so `SurfaceMesh::new(its)` computes
//!   `m_face_neighbors` internally, exactly as the C++ constructor does
//!   (`m_face_neighbors(its_face_neighbors_par(its))`).
//! - `boost::container::small_vector<Halfedge_index, 10>` in `degree()` is a small-buffer
//!   optimization of a dynamic array; we use a plain `Vec<Halfedge_index>` which is
//!   semantically identical (same elements, same order, same membership test).

#![allow(non_camel_case_types)]

use crate::triangle_set_sampling::{indexed_triangle_set, Vec3f, Vec3i};

// SurfaceMesh.hpp:19
// enum Face_index : int;
//
// In C++ this is a strongly-typed integer enum used purely as a face index (and
// as the sentinel `Face_index(-1)`). We model it as a newtype over `i32` so that
// `int(face_id)` casts and the `-1` invalid sentinel translate exactly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Face_index(pub i32);

impl Face_index {
    /// C++: `Face_index(int)`
    #[inline]
    pub fn new(v: i32) -> Self {
        Face_index(v)
    }
}

// SurfaceMesh.hpp:21
// Index handles are trivially copyable value types in C++ (only SurfaceMesh's
// copy/assignment are deleted, not the handles); derive Copy to match.
// PartialEq/Eq are impl'd manually below.
#[derive(Debug, Clone, Copy)]
pub struct Halfedge_index {
    // SurfaceMesh.hpp:35
    m_face: Face_index,
    // SurfaceMesh.hpp:36
    m_side: u8,
}

impl Halfedge_index {
    // SurfaceMesh.hpp:25
    // Halfedge_index() : m_face(Face_index(-1)), m_side(0) {}
    #[inline]
    pub fn new() -> Self {
        Halfedge_index {
            m_face: Face_index(-1),
            m_side: 0,
        }
    }

    // SurfaceMesh.hpp:26
    // Face_index face() const { return m_face; }
    #[inline]
    pub fn face(&self) -> Face_index {
        self.m_face
    }

    // SurfaceMesh.hpp:27
    // unsigned char side() const { return m_side; }
    #[inline]
    pub fn side(&self) -> u8 {
        self.m_side
    }

    // SurfaceMesh.hpp:28
    // bool is_invalid() const { return int(m_face) < 0; }
    #[inline]
    pub fn is_invalid(&self) -> bool {
        self.m_face.0 < 0
    }

    // SurfaceMesh.hpp:33
    // Halfedge_index(int face_idx, unsigned char side_idx) : m_face(Face_index(face_idx)), m_side(side_idx) {}
    #[inline]
    fn from_face_side(face_idx: i32, side_idx: u8) -> Self {
        Halfedge_index {
            m_face: Face_index(face_idx),
            m_side: side_idx,
        }
    }
}

// SurfaceMesh.hpp:29-30
// bool operator!=(const Halfedge_index& rhs) const { return ! ((*this) == rhs); }
// bool operator==(const Halfedge_index& rhs) const { return m_face == rhs.m_face && m_side == rhs.m_side; }
impl PartialEq for Halfedge_index {
    #[inline]
    fn eq(&self, rhs: &Halfedge_index) -> bool {
        self.m_face == rhs.m_face && self.m_side == rhs.m_side
    }
}
impl Eq for Halfedge_index {}

impl Default for Halfedge_index {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

// SurfaceMesh.hpp:41
// No PartialEq/Eq: C++ deletes operator== (SurfaceMesh.hpp:47) to force callers
// through SurfaceMesh::is_same_vertex (which compares underlying vertex INDICES,
// not handle identity). Deriving PartialEq would expose the exact by-handle `==`
// the C++ deletion forbids. Verified: no caller compares Vertex_index by value.
#[derive(Debug, Clone, Copy)]
pub struct Vertex_index {
    // SurfaceMesh.hpp:52
    m_face: Face_index,
    // SurfaceMesh.hpp:53
    m_vertex_idx: u8,
}

impl Vertex_index {
    // SurfaceMesh.hpp:45
    // Vertex_index() : m_face(Face_index(-1)), m_vertex_idx(0) {}
    #[inline]
    pub fn new() -> Self {
        Vertex_index {
            m_face: Face_index(-1),
            m_vertex_idx: 0,
        }
    }

    // SurfaceMesh.hpp:46
    // bool is_invalid() const { return int(m_face) < 0; }
    #[inline]
    pub fn is_invalid(&self) -> bool {
        self.m_face.0 < 0
    }

    // SurfaceMesh.hpp:47
    // bool operator==(const Vertex_index& rhs) const = delete; // Use SurfaceMesh::is_same_vertex.
    // (Deliberately not implementing PartialEq: comparison is forbidden in C++.
    //  Use SurfaceMesh::is_same_vertex instead.)

    // SurfaceMesh.hpp:50
    // Vertex_index(int face_idx, unsigned char vertex_idx) : m_face(Face_index(face_idx)), m_vertex_idx(vertex_idx) {}
    #[inline]
    fn from_face_vertex(face_idx: i32, vertex_idx: u8) -> Self {
        Vertex_index {
            m_face: Face_index(face_idx),
            m_vertex_idx: vertex_idx,
        }
    }
}

impl Default for Vertex_index {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

// SurfaceMesh.hpp:58
pub struct SurfaceMesh<'a> {
    // SurfaceMesh.hpp:161
    // const std::vector<Vec3i> m_face_neighbors;
    m_face_neighbors: Vec<Vec3i>,
    // SurfaceMesh.hpp:162
    // const indexed_triangle_set& m_its;
    m_its: &'a indexed_triangle_set,
}

impl<'a> SurfaceMesh<'a> {
    // SurfaceMesh.hpp:60-63
    // explicit SurfaceMesh(const indexed_triangle_set& its)
    // : m_its(its),
    //   m_face_neighbors(its_face_neighbors_par(its))
    // {}
    //
    // `its_face_neighbors_par` (TriangleMesh.cpp:1933-1936) is ported in
    // `crate::measure` in this `indexed_triangle_set` domain (the crate carries a
    // second, structurally distinct `indexed_triangle_set` in `normal_utils`, which
    // is what `crate::triangle_mesh::its_face_neighbors_par` operates on).
    #[inline]
    pub fn new(its: &'a indexed_triangle_set) -> Self {
        SurfaceMesh {
            m_its: its,
            m_face_neighbors: crate::measure::its_face_neighbors_par(its),
        }
    }

    // SurfaceMesh.hpp:64-65
    // SurfaceMesh(const SurfaceMesh&)            = delete;
    // SurfaceMesh& operator=(const SurfaceMesh&) = delete;
    // (Copy/assignment are deleted in C++; we simply do not derive Clone/Copy.)

    // SurfaceMesh.hpp:67
    // Vertex_index source(Halfedge_index h) const { assert(! h.is_invalid()); return Vertex_index(h.m_face, h.m_side); }
    #[inline]
    pub fn source(&self, h: Halfedge_index) -> Vertex_index {
        debug_assert!(!h.is_invalid());
        Vertex_index::from_face_vertex(h.m_face.0, h.m_side)
    }

    // SurfaceMesh.hpp:68
    // Vertex_index target(Halfedge_index h) const { assert(! h.is_invalid()); return Vertex_index(h.m_face, h.m_side == 2 ? 0 : h.m_side + 1); }
    #[inline]
    pub fn target(&self, h: Halfedge_index) -> Vertex_index {
        debug_assert!(!h.is_invalid());
        Vertex_index::from_face_vertex(h.m_face.0, if h.m_side == 2 { 0 } else { h.m_side + 1 })
    }

    // SurfaceMesh.hpp:69
    // Face_index face(Halfedge_index h) const { assert(! h.is_invalid()); return h.m_face; }
    #[inline]
    pub fn face(&self, h: Halfedge_index) -> Face_index {
        debug_assert!(!h.is_invalid());
        h.m_face
    }

    // SurfaceMesh.hpp:71
    // Halfedge_index next(Halfedge_index h) const { assert(! h.is_invalid()); h.m_side = (h.m_side + 1) % 3; return h; }
    #[inline]
    pub fn next(&self, mut h: Halfedge_index) -> Halfedge_index {
        debug_assert!(!h.is_invalid());
        h.m_side = (h.m_side + 1) % 3;
        h
    }

    // SurfaceMesh.hpp:72
    // Halfedge_index prev(Halfedge_index h) const { assert(! h.is_invalid()); h.m_side = (h.m_side == 0 ? 2 : h.m_side - 1); return h; }
    #[inline]
    pub fn prev(&self, mut h: Halfedge_index) -> Halfedge_index {
        debug_assert!(!h.is_invalid());
        h.m_side = if h.m_side == 0 { 2 } else { h.m_side - 1 };
        h
    }

    // SurfaceMesh.hpp:73
    // Halfedge_index halfedge(Vertex_index v) const { return Halfedge_index(v.m_face, (v.m_vertex_idx == 0 ? 2 : v.m_vertex_idx - 1)); }
    #[inline]
    pub fn halfedge(&self, v: Vertex_index) -> Halfedge_index {
        Halfedge_index::from_face_side(
            v.m_face.0,
            if v.m_vertex_idx == 0 {
                2
            } else {
                v.m_vertex_idx - 1
            },
        )
    }

    // SurfaceMesh.hpp:74
    // Halfedge_index halfedge(Face_index f) const { return Halfedge_index(f, 0); }
    #[inline]
    pub fn halfedge_face(&self, f: Face_index) -> Halfedge_index {
        Halfedge_index::from_face_side(f.0, 0)
    }

    // SurfaceMesh.hpp:75-94
    // Halfedge_index opposite(Halfedge_index h) const { ... }
    pub fn opposite(&self, h: Halfedge_index) -> Halfedge_index {
        // SurfaceMesh.hpp:76-77
        if h.is_invalid() {
            return h;
        }

        // SurfaceMesh.hpp:79
        // int face_idx = m_face_neighbors[h.m_face][h.m_side];
        let face_idx = self.m_face_neighbors[h.m_face.0 as usize][h.m_side as usize];
        // SurfaceMesh.hpp:80
        // Halfedge_index h_candidate = halfedge(Face_index(face_idx));
        let mut h_candidate = self.halfedge_face(Face_index(face_idx));

        // SurfaceMesh.hpp:82-83
        if h_candidate.is_invalid() {
            return Halfedge_index::new(); // invalid
        }

        // SurfaceMesh.hpp:85-92
        for _i in 0..3 {
            if self.is_same_vertex(&self.source(h_candidate), &self.target(h)) {
                // Meshes in PrusaSlicer should be fixed enough for the following not to happen.
                debug_assert!(self.is_same_vertex(&self.target(h_candidate), &self.source(h)));
                return h_candidate;
            }
            h_candidate = self.next(h_candidate);
        }
        // SurfaceMesh.hpp:93
        Halfedge_index::new() // invalid
    }

    // SurfaceMesh.hpp:96
    // Halfedge_index next_around_target(Halfedge_index h) const { return opposite(next(h)); }
    #[inline]
    pub fn next_around_target(&self, h: Halfedge_index) -> Halfedge_index {
        let n = self.next(h);
        self.opposite(n)
    }

    // SurfaceMesh.hpp:97
    // Halfedge_index prev_around_target(Halfedge_index h) const { Halfedge_index op = opposite(h); return (op.is_invalid() ? Halfedge_index() : prev(op)); }
    #[inline]
    pub fn prev_around_target(&self, h: Halfedge_index) -> Halfedge_index {
        let op = self.opposite(h);
        if op.is_invalid() {
            Halfedge_index::new()
        } else {
            self.prev(op)
        }
    }

    // SurfaceMesh.hpp:98
    // Halfedge_index next_around_source(Halfedge_index h) const { Halfedge_index op = opposite(h); return (op.is_invalid() ? Halfedge_index() : next(op)); }
    #[inline]
    pub fn next_around_source(&self, h: Halfedge_index) -> Halfedge_index {
        let op = self.opposite(h);
        if op.is_invalid() {
            Halfedge_index::new()
        } else {
            self.next(op)
        }
    }

    // SurfaceMesh.hpp:99
    // Halfedge_index prev_around_source(Halfedge_index h) const { return opposite(prev(h)); }
    #[inline]
    pub fn prev_around_source(&self, h: Halfedge_index) -> Halfedge_index {
        let p = self.prev(h);
        self.opposite(p)
    }

    // SurfaceMesh.hpp:100-118
    // Halfedge_index halfedge(Vertex_index source, Vertex_index target) const { ... }
    pub fn halfedge_src_tgt(&self, source: Vertex_index, target: Vertex_index) -> Halfedge_index {
        // SurfaceMesh.hpp:102
        // Halfedge_index hi(source.m_face, source.m_vertex_idx);
        let mut hi = Halfedge_index::from_face_side(source.m_face.0, source.m_vertex_idx);
        // SurfaceMesh.hpp:103
        debug_assert!(!hi.is_invalid());

        // SurfaceMesh.hpp:105
        // const Vertex_index orig_target = this->target(hi);
        let orig_target = self.target(hi);
        // SurfaceMesh.hpp:106
        // Vertex_index current_target = orig_target;
        let mut current_target = orig_target; // Vertex_index is Copy; this mirrors the C++ value copy

        // SurfaceMesh.hpp:108-115
        while !self.is_same_vertex(&current_target, &target) {
            // SurfaceMesh.hpp:109
            hi = self.next_around_source(hi);
            // SurfaceMesh.hpp:110-111
            if hi.is_invalid() {
                break;
            }
            // SurfaceMesh.hpp:112
            current_target = self.target(hi);
            // SurfaceMesh.hpp:113-114
            if self.is_same_vertex(&current_target, &orig_target) {
                return Halfedge_index::new(); // invalid
            }
        }

        // SurfaceMesh.hpp:117
        hi
    }

    // SurfaceMesh.hpp:120
    // const stl_vertex& point(Vertex_index v) const { return m_its.vertices[m_its.indices[v.m_face][v.m_vertex_idx]]; }
    #[inline]
    pub fn point(&self, v: Vertex_index) -> &Vec3f {
        let idx = self.m_its.indices[v.m_face.0 as usize][v.m_vertex_idx as usize];
        &self.m_its.vertices[idx as usize]
    }

    // SurfaceMesh.hpp:122-139
    // size_t degree(Vertex_index v) const { ... }
    pub fn degree_vertex(&self, v: Vertex_index) -> usize {
        // In case the mesh is broken badly, the loop might end up to be infinite,
        // never getting back to the first halfedge. Remember list of all half-edges
        // and trip if any is encountered for the second time.
        // SurfaceMesh.hpp:127
        let h_first = self.halfedge(v);
        // SurfaceMesh.hpp:128
        // boost::container::small_vector<Halfedge_index, 10> he_visited;
        let mut he_visited: Vec<Halfedge_index> = Vec::new();
        // SurfaceMesh.hpp:129
        let mut h = self.next_around_target(h_first);
        // SurfaceMesh.hpp:130
        let mut degree: usize = 2;
        // SurfaceMesh.hpp:131
        while !h.is_invalid() && h != h_first {
            // SurfaceMesh.hpp:132
            he_visited.push(h);
            // SurfaceMesh.hpp:133
            h = self.next_around_target(h);
            // SurfaceMesh.hpp:134-135
            if !he_visited.iter().any(|&x| x == h) {
                return 0;
            }
            // SurfaceMesh.hpp:136
            degree += 1;
        }
        // SurfaceMesh.hpp:138
        if h.is_invalid() {
            0
        } else {
            degree - 1
        }
    }

    // SurfaceMesh.hpp:141-151
    // size_t degree(Face_index f) const { ... }
    pub fn degree_face(&self, f: Face_index) -> usize {
        // SurfaceMesh.hpp:142
        let mut total: usize = 0;
        // SurfaceMesh.hpp:143-148
        for i in 0u8..3 {
            // SurfaceMesh.hpp:144
            let d = self.degree_vertex(Vertex_index::from_face_vertex(f.0, i));
            // SurfaceMesh.hpp:145-146
            if d == 0 {
                return 0;
            }
            // SurfaceMesh.hpp:147
            total += d;
        }
        // SurfaceMesh.hpp:149
        // assert(total - 6 >= 0); — always true for unsigned size_t; preserved as a no-op assert.
        debug_assert!(total >= 6);
        // SurfaceMesh.hpp:150
        total - 6 // we counted 3 halfedges from f, and one more for each neighbor
    }

    // SurfaceMesh.hpp:153
    // bool is_border(Halfedge_index h) const { return m_face_neighbors[h.m_face][h.m_side] == -1; }
    #[inline]
    pub fn is_border(&self, h: Halfedge_index) -> bool {
        self.m_face_neighbors[h.m_face.0 as usize][h.m_side as usize] == -1
    }

    // SurfaceMesh.hpp:155
    // bool is_same_vertex(const Vertex_index& a, const Vertex_index& b) const { return m_its.indices[a.m_face][a.m_vertex_idx] == m_its.indices[b.m_face][b.m_vertex_idx]; }
    #[inline]
    pub fn is_same_vertex(&self, a: &Vertex_index, b: &Vertex_index) -> bool {
        self.m_its.indices[a.m_face.0 as usize][a.m_vertex_idx as usize]
            == self.m_its.indices[b.m_face.0 as usize][b.m_vertex_idx as usize]
    }

    // SurfaceMesh.hpp:156
    // Vec3i get_face_neighbors(Face_index face_id) const { assert(int(face_id) < int(m_face_neighbors.size())); return m_face_neighbors[face_id]; }
    #[inline]
    pub fn get_face_neighbors(&self, face_id: Face_index) -> Vec3i {
        debug_assert!(face_id.0 < self.m_face_neighbors.len() as i32);
        self.m_face_neighbors[face_id.0 as usize]
    }
}
