//! Port of `TextureToColor/CgalUtils.hpp`: conversion between `TriMesh` and the halfedge mesh the
//! tex2color algorithms work on, and the test whether a `TriMesh` can be one. With it, what the
//! units of the directory use of CGAL itself, written out from the CGAL source (version 6.1.1).
//!
//! C++ Reference:
//! - src/libslic3r/TextureToColor/CgalUtils.hpp
//! - CGAL/Surface_mesh/Surface_mesh.h, CGAL/boost/graph/Euler_operations.h (`add_face`),
//!   CGAL/boost/graph/helpers.h, CGAL/boost/graph/iterator.h
//! - CGAL/Kernel/function_objects.h, CGAL/Cartesian/function_objects.h,
//!   CGAL/constructions/kernel_ftC3.h (the kernel's constructions)
//! - CGAL/AABB_tree.h, CGAL/AABB_traits_3.h, CGAL/AABB_tree/internal/AABB_node.h,
//!   CGAL/AABB_tree/internal/AABB_traversal_traits.h,
//!   CGAL/Filtered_kernel/internal/Static_filters/Do_intersect_3.h (`Tree`)
//! - CGAL/Polygon_mesh_processing/compute_normal.h, border.h, CGAL/centroid.h
//!
//! FIDELITY-NOTE: `CGALMesh` is `CGAL::Surface_mesh<Kernel::Point_3>` in C++ (CgalUtils.hpp:18).
//! The crate has no port of CGAL, so `CGALMesh` here is a stand-in that holds the connectivity
//! arrays of `Surface_mesh` (`vconn_`, `hconn_`, `fconn_`, `vpoint_`) and the members the ported
//! units call, each following the CGAL source named at it: building (`add_vertex`, `add_edge`,
//! `add_face`, which is `CGAL::Euler::add_face`), walking (`halfedge`, `next`, `opposite`, the
//! circulators) and the validity tests. Missing: removal of elements, the free lists and garbage
//! collection (`is_removed` is always false and the iterators visit every index) and property maps
//! other than the vertex point. The indices a mesh built by `trimesh_to_cgal` gives its vertices,
//! halfedges and faces are those of CGAL: `tests/texture_to_color_cgal_utils_native_parity.rs`
//! compares the whole connectivity, the kernel's constructions and the closest points of the
//! tree with CGAL.
//!
//! FIDELITY-NOTE: of CGAL's algorithms over a mesh the file has `PMP::compute_face_normal`,
//! `PMP::extract_boundary_cycles`, `CGAL::centroid` of points, `CGAL::is_closed` and the AABB
//! tree's `closest_point` (`Tree`, with one deviation named there). The others the units call are
//! the trait `PolygonMeshProcessing`, which the crate does not implement.
//!
//! FIDELITY-NOTE: `is_mesh_halfedge_compatible` keeps its sets in `std::unordered_set` and starts
//! the walk around a vertex at `*set.begin()` (CgalUtils.hpp:104). The answer does not depend on
//! which face that is (the faces around the vertex are reached from any of them or from none), so
//! the sets are `HashSet` and the first face is the smallest.
//!
//! Nothing in the slicing path calls this unit.

#![allow(non_camel_case_types)]

use std::collections::{BTreeMap, HashMap, HashSet, VecDeque};

use crate::texture_to_color::tri_mesh::{TriFace, TriMesh};
use crate::triangle_mesh::Vec2f;
use crate::triangle_set_sampling::{Vec3f, Vec3i};

/// `Kernel::Point_3` of `CGAL::Exact_predicates_inexact_constructions_kernel` (CgalUtils.hpp:17):
/// three doubles.
pub type Point_3 = [f64; 3];

/// `Kernel::Vector_3`: three doubles.
pub type Vector_3 = [f64; 3];

/// Surface_mesh.h, `SM_Index`: the index every descriptor holds, `(std::numeric_limits<size_type>::max)()`
/// when it is null. `size_type` is `std::uint32_t`.
const SM_NULL: u32 = u32::MAX;

/// Surface_mesh.h, `class SM_Vertex_index`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct SM_Vertex_index(pub u32);

/// Surface_mesh.h, `class SM_Halfedge_index`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct SM_Halfedge_index(pub u32);

/// Surface_mesh.h, `class SM_Face_index`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct SM_Face_index(pub u32);

/// Surface_mesh.h, `class SM_Edge_index`: it keeps the halfedge it was made from; its index, and
/// what `==` and the hash compare, is that halfedge's index halved.
#[derive(Debug, Clone, Copy)]
pub struct SM_Edge_index {
    halfedge_: SM_Halfedge_index,
}

macro_rules! sm_index {
    ($name:ident) => {
        impl $name {
            /// `SM_Index()`: the null descriptor
            pub fn null() -> Self {
                $name(SM_NULL)
            }
            /// `SM_Index::is_valid()`
            pub fn is_valid(&self) -> bool {
                self.0 != SM_NULL
            }
            /// `operator size_type()`, as a position in an array
            pub fn idx(&self) -> usize {
                self.0 as usize
            }
        }
        impl Default for $name {
            fn default() -> Self {
                Self::null()
            }
        }
    };
}
sm_index!(SM_Vertex_index);
sm_index!(SM_Halfedge_index);
sm_index!(SM_Face_index);

impl SM_Edge_index {
    /// `SM_Edge_index(SM_Halfedge_index he)`
    pub fn from_halfedge(he: SM_Halfedge_index) -> Self {
        SM_Edge_index { halfedge_: he }
    }
    /// `SM_Edge_index(size_type idx)`: `halfedge_(idx * 2)`
    pub fn new(idx: u32) -> Self {
        SM_Edge_index { halfedge_: SM_Halfedge_index(idx * 2) }
    }
    /// `SM_Edge_index::halfedge()`
    pub fn halfedge(&self) -> SM_Halfedge_index {
        self.halfedge_
    }
    /// `operator size_type()`: `halfedge_ / 2`, as a position in an array
    pub fn idx(&self) -> usize {
        (self.halfedge_.0 / 2) as usize
    }
    /// `SM_Edge_index::is_valid()`
    pub fn is_valid(&self) -> bool {
        self.halfedge_.is_valid()
    }
}

impl PartialEq for SM_Edge_index {
    fn eq(&self, other: &Self) -> bool {
        self.halfedge_.0 / 2 == other.halfedge_.0 / 2
    }
}
impl Eq for SM_Edge_index {}
impl std::hash::Hash for SM_Edge_index {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        (self.halfedge_.0 / 2).hash(state)
    }
}

/// Surface_mesh.h, `struct Vertex_connectivity`
#[derive(Debug, Clone, Copy, Default)]
struct Vertex_connectivity {
    halfedge_: SM_Halfedge_index,
}

/// Surface_mesh.h, `struct Halfedge_connectivity`
#[derive(Debug, Clone, Copy, Default)]
struct Halfedge_connectivity {
    face_: SM_Face_index,
    vertex_: SM_Vertex_index,
    next_halfedge_: SM_Halfedge_index,
    prev_halfedge_: SM_Halfedge_index,
}

/// Surface_mesh.h, `struct Face_connectivity`
#[derive(Debug, Clone, Copy, Default)]
struct Face_connectivity {
    halfedge_: SM_Halfedge_index,
}

/// CgalUtils.hpp:18, `CGAL::Surface_mesh<Kernel::Point_3>`. See the FIDELITY-NOTE of the file.
#[derive(Debug, Clone, Default)]
pub struct CGALMesh {
    vconn_: Vec<Vertex_connectivity>,
    hconn_: Vec<Halfedge_connectivity>,
    fconn_: Vec<Face_connectivity>,
    vpoint_: Vec<Point_3>,
}

impl CGALMesh {
    /// `Surface_mesh()`
    pub fn new() -> Self {
        Self::default()
    }

    /// `Surface_mesh::clear()`
    pub fn clear(&mut self) {
        self.vconn_.clear();
        self.hconn_.clear();
        self.fconn_.clear();
        self.vpoint_.clear();
    }

    /// `Surface_mesh::null_face()`
    pub fn null_face() -> SM_Face_index {
        SM_Face_index::null()
    }

    /// `Surface_mesh::null_halfedge()`
    pub fn null_halfedge() -> SM_Halfedge_index {
        SM_Halfedge_index::null()
    }

    /// `Surface_mesh::num_vertices()`
    pub fn num_vertices(&self) -> usize {
        self.vconn_.len()
    }
    /// `Surface_mesh::num_halfedges()`
    pub fn num_halfedges(&self) -> usize {
        self.hconn_.len()
    }
    /// `Surface_mesh::num_edges()`
    pub fn num_edges(&self) -> usize {
        self.hconn_.len() / 2
    }
    /// `Surface_mesh::num_faces()`
    pub fn num_faces(&self) -> usize {
        self.fconn_.len()
    }
    /// `Surface_mesh::number_of_vertices()`: `num_vertices()` less the removed ones, of which
    /// there are none here
    pub fn number_of_vertices(&self) -> usize {
        self.num_vertices()
    }
    /// `Surface_mesh::number_of_halfedges()`
    pub fn number_of_halfedges(&self) -> usize {
        self.num_halfedges()
    }
    /// `Surface_mesh::number_of_edges()`
    pub fn number_of_edges(&self) -> usize {
        self.num_edges()
    }
    /// `Surface_mesh::number_of_faces()`
    pub fn number_of_faces(&self) -> usize {
        self.num_faces()
    }

    /// `Surface_mesh::add_vertex(const Point& p)` (the branch without a free list)
    pub fn add_vertex(&mut self, p: Point_3) -> SM_Vertex_index {
        self.vconn_.push(Vertex_connectivity::default());
        self.vpoint_.push(p);
        SM_Vertex_index((self.num_vertices() - 1) as u32)
    }

    /// `Surface_mesh::add_edge(Vertex_index v0, Vertex_index v1)`: two opposite halfedges, the
    /// returned one with `v1` as target
    pub fn add_edge(&mut self, v0: SM_Vertex_index, v1: SM_Vertex_index) -> SM_Halfedge_index {
        // add_edge(): eprops_.push_back(); hprops_.push_back(); hprops_.push_back();
        self.hconn_.push(Halfedge_connectivity::default());
        self.hconn_.push(Halfedge_connectivity::default());
        let h = SM_Halfedge_index((self.num_halfedges() - 2) as u32);

        self.set_target(h, v1);
        let o = self.opposite(h);
        self.set_target(o, v0);

        h
    }

    /// `Surface_mesh::add_face()` (the branch without a free list)
    fn add_face_empty(&mut self) -> SM_Face_index {
        self.fconn_.push(Face_connectivity::default());
        SM_Face_index((self.num_faces() - 1) as u32)
    }

    /// `Surface_mesh::add_face(Vertex_index v0, Vertex_index v1, Vertex_index v2)`
    pub fn add_face(&mut self, v0: SM_Vertex_index, v1: SM_Vertex_index, v2: SM_Vertex_index) -> SM_Face_index {
        self.add_face_range(&[v0, v1, v2])
    }

    /// `Surface_mesh::add_face(const Range& vertices)`, which is `CGAL::Euler::add_face(vr, g)`
    /// (Euler_operations.h): the face over the vertices in this order, or the null face when it
    /// cannot be added (a vertex twice, fewer than three vertices, a vertex that is closed, an
    /// edge that has its two faces, a patch that cannot be relinked).
    pub fn add_face_range(&mut self, vr: &[SM_Vertex_index]) -> SM_Face_index {
        let mut vertices: Vec<SM_Vertex_index> = vr.to_vec(); // quick and dirty copy
        let n = vertices.len();
        // check that every vertex is unique
        vertices.sort();
        if vertices.windows(2).any(|w| w[0] == w[1]) {
            return Self::null_face();
        }
        vertices.copy_from_slice(vr);
        // don't allow degenerated faces
        if n <= 2 {
            return Self::null_face();
        }

        let mut halfedges = vec![SM_Halfedge_index::null(); n];
        let mut is_new = vec![false; n];

        for i in 0..n {
            let ii = (i + 1) % n;
            if !self.is_isolated(vertices[i]) && !self.is_border_v_bgl(vertices[i]) {
                return Self::null_face();
            }

            // std::pair<halfedge_descriptor, bool> he = halfedge(vertices[i], vertices[ii], g);
            let he = self.halfedge_vv(vertices[i], vertices[ii]);
            halfedges[i] = he; // collect if exists
            is_new[i] = !he.is_valid();

            if !is_new[i] && !self.is_border_h(halfedges[i]) {
                return Self::null_face();
            }
        }

        let mut inner_next;
        let mut inner_prev;
        let mut outer_next;
        let mut outer_prev;
        let mut border_next;
        let mut border_prev;
        let mut patch_start;
        let mut patch_end;
        // cache for set_next and vertex' set_halfedge
        let mut next_cache: Vec<(SM_Halfedge_index, SM_Halfedge_index)> = Vec::with_capacity(3 * n);

        // re-link patches if necessary
        for i in 0..n {
            let ii = (i + 1) % n;
            if !is_new[i] && !is_new[ii] {
                inner_prev = halfedges[i];
                inner_next = halfedges[ii];

                if self.next(inner_prev) != inner_next {
                    // here comes the ugly part... we have to relink a whole patch

                    // search a free gap
                    // free gap will be between border_prev and border_next
                    outer_prev = self.opposite(inner_next);
                    border_prev = outer_prev;
                    loop {
                        border_prev = self.opposite(self.next(border_prev));
                        if !(!self.is_border_h(border_prev) || border_prev == inner_prev) {
                            break;
                        }
                    }
                    border_next = self.next(border_prev);

                    if border_next == inner_next {
                        return Self::null_face();
                    }

                    // other halfedges' indices
                    patch_start = self.next(inner_prev);
                    patch_end = self.prev(inner_next);

                    // relink
                    next_cache.push((border_prev, patch_start));
                    next_cache.push((patch_end, border_next));
                    next_cache.push((inner_prev, inner_next));
                }
            }
        }
        // create missing edges
        for i in 0..n {
            let ii = (i + 1) % n;
            if is_new[i] {
                // edge_descriptor ne = add_edge(vertices[i], vertices[ii], g); halfedges[i] = halfedge(ne, g);
                halfedges[i] = self.add_edge(vertices[i], vertices[ii]);
                // as it may be recycled we have to reset it
                let o = self.opposite(halfedges[i]);
                self.set_face(o, Self::null_face());
            }
        }
        // create the face
        let f = self.add_face_empty();
        self.set_halfedge_f(f, halfedges[n - 1]);

        // setup halfedges
        for i in 0..n {
            let ii = (i + 1) % n;
            let v = vertices[ii];
            inner_prev = halfedges[i];
            inner_next = halfedges[ii];

            let mut id = 0u32;
            if is_new[i] {
                id |= 1;
            }
            if is_new[ii] {
                id |= 2;
            }

            if id != 0 {
                outer_prev = self.opposite(inner_next);
                outer_next = self.opposite(inner_prev);

                // set outer links
                match id {
                    1 => {
                        // prev is new, next is old
                        border_prev = self.prev(inner_next);
                        next_cache.push((border_prev, outer_next));
                        self.set_halfedge_v(v, border_prev);
                    }
                    2 => {
                        // next is new, prev is old
                        border_next = self.next(inner_prev);
                        next_cache.push((outer_prev, border_next));
                        self.set_halfedge_v(v, outer_prev);
                    }
                    _ => {
                        // both are new
                        // try to pick a border halfedge with v as target
                        let mut hv = self.halfedge_v(v);
                        if hv != Self::null_halfedge() && !self.is_border_h(hv) {
                            for h_around_v in self.halfedges_around_target(hv) {
                                if self.is_border_h(h_around_v) {
                                    hv = h_around_v;
                                    break;
                                }
                            }
                            if !self.is_border_h(hv) {
                                hv = Self::null_halfedge();
                            }
                        }

                        if hv == Self::null_halfedge() {
                            self.set_halfedge_v(v, outer_prev);
                            next_cache.push((outer_prev, outer_next));
                        } else {
                            border_prev = hv;
                            border_next = self.next(border_prev);
                            next_cache.push((border_prev, outer_next));
                            next_cache.push((outer_prev, border_next));
                        }
                    }
                }

                // set inner link
                next_cache.push((inner_prev, inner_next));
            }

            // set face index
            self.set_face(halfedges[i], f);
        }

        // process next halfedge cache
        for &(first, second) in next_cache.iter() {
            self.set_next(first, second);
        }

        // adjust vertices' halfedge index
        for i in 0..n {
            self.adjust_incoming_halfedge(vertices[i]);
        }

        f
    }

    /// `Surface_mesh::adjust_incoming_halfedge(Vertex_index v)`: the halfedge of `v` becomes a
    /// border halfedge when one points to it
    pub fn adjust_incoming_halfedge(&mut self, v: SM_Vertex_index) {
        let mut h = self.halfedge_v(v);
        let mut hh = h;

        if h.is_valid() {
            if self.target(h) != v {
                // wrong target, flip
                h = self.opposite(h);
                hh = h;
                self.set_halfedge_v(v, h);
            }

            loop {
                if self.is_border_h(h) {
                    self.set_halfedge_v(v, h);
                    return;
                }
                h = self.next_around_target(h);
                if h == hh {
                    break;
                }
            }
        }
    }

    /// `Surface_mesh::vertices()`
    pub fn vertices(&self) -> impl Iterator<Item = SM_Vertex_index> {
        (0..self.num_vertices() as u32).map(SM_Vertex_index)
    }
    /// `Surface_mesh::halfedges()`
    pub fn halfedges(&self) -> impl Iterator<Item = SM_Halfedge_index> {
        (0..self.num_halfedges() as u32).map(SM_Halfedge_index)
    }
    /// `Surface_mesh::edges()`
    pub fn edges(&self) -> impl Iterator<Item = SM_Edge_index> {
        (0..self.num_edges() as u32).map(SM_Edge_index::new)
    }
    /// `Surface_mesh::faces()`
    pub fn faces(&self) -> impl Iterator<Item = SM_Face_index> {
        (0..self.num_faces() as u32).map(SM_Face_index)
    }

    /// `Surface_mesh::has_valid_index(Vertex_index)`
    pub fn has_valid_index_v(&self, v: SM_Vertex_index) -> bool {
        v.idx() < self.num_vertices()
    }
    /// `Surface_mesh::has_valid_index(Halfedge_index)`
    pub fn has_valid_index_h(&self, h: SM_Halfedge_index) -> bool {
        h.idx() < self.num_halfedges()
    }
    /// `Surface_mesh::has_valid_index(Edge_index)`
    pub fn has_valid_index_e(&self, e: SM_Edge_index) -> bool {
        e.idx() < self.num_edges()
    }
    /// `Surface_mesh::has_valid_index(Face_index)`
    pub fn has_valid_index_f(&self, f: SM_Face_index) -> bool {
        f.idx() < self.num_faces()
    }

    /// `Surface_mesh::is_removed(Vertex_index)`: nothing is removed from this stand-in
    pub fn is_removed_v(&self, _v: SM_Vertex_index) -> bool {
        false
    }
    /// `Surface_mesh::is_removed(Halfedge_index)`
    pub fn is_removed_h(&self, _h: SM_Halfedge_index) -> bool {
        false
    }
    /// `Surface_mesh::is_removed(Edge_index)`
    pub fn is_removed_e(&self, _e: SM_Edge_index) -> bool {
        false
    }
    /// `Surface_mesh::is_removed(Face_index)`
    pub fn is_removed_f(&self, _f: SM_Face_index) -> bool {
        false
    }

    /// `Surface_mesh::is_valid(Vertex_index v)`
    pub fn is_valid_v(&self, v: SM_Vertex_index) -> bool {
        if !self.has_valid_index_v(v) {
            return false;
        }
        let h = self.vconn_[v.idx()].halfedge_;
        if h != Self::null_halfedge() && (!self.has_valid_index_h(h) || self.is_removed_h(h)) {
            return false;
        }
        true
    }

    /// `Surface_mesh::is_valid(Halfedge_index h)`
    pub fn is_valid_h(&self, h: SM_Halfedge_index) -> bool {
        if !self.has_valid_index_h(h) {
            return false;
        }

        let f = self.hconn_[h.idx()].face_;
        let v = self.hconn_[h.idx()].vertex_;
        let hn = self.hconn_[h.idx()].next_halfedge_;
        let hp = self.hconn_[h.idx()].prev_halfedge_;

        let mut valid = true;
        // don't validate the face if this is a border halfedge
        if !self.is_border_h(h) {
            if !self.has_valid_index_f(f) || self.is_removed_f(f) {
                valid = false;
            }
        }
        if !self.has_valid_index_v(v) || self.is_removed_v(v) {
            valid = false;
        }
        if !self.has_valid_index_h(hn) || self.is_removed_h(hn) {
            valid = false;
        }
        if !self.has_valid_index_h(hp) || self.is_removed_h(hp) {
            valid = false;
        }
        valid
    }

    /// `Surface_mesh::is_valid(Edge_index e)`
    pub fn is_valid_e(&self, e: SM_Edge_index) -> bool {
        if !self.has_valid_index_e(e) {
            return false;
        }
        let h = self.halfedge_e(e);
        self.is_valid_h(h) && self.is_valid_h(self.opposite(h))
    }

    /// `Surface_mesh::is_valid(Face_index f)`
    pub fn is_valid_f(&self, f: SM_Face_index) -> bool {
        if !self.has_valid_index_f(f) {
            return false;
        }
        let h = self.fconn_[f.idx()].halfedge_;
        if !self.has_valid_index_h(h) || self.is_removed_h(h) {
            return false;
        }
        true
    }

    /// `Surface_mesh::target(Halfedge_index h)`
    pub fn target(&self, h: SM_Halfedge_index) -> SM_Vertex_index {
        self.hconn_[h.idx()].vertex_
    }
    /// `Surface_mesh::set_target(Halfedge_index h, Vertex_index v)`
    pub fn set_target(&mut self, h: SM_Halfedge_index, v: SM_Vertex_index) {
        self.hconn_[h.idx()].vertex_ = v;
    }
    /// `Surface_mesh::source(Halfedge_index h)`: `target(opposite(h))`
    pub fn source(&self, h: SM_Halfedge_index) -> SM_Vertex_index {
        self.target(self.opposite(h))
    }
    /// `Surface_mesh::face(Halfedge_index h)`
    pub fn face(&self, h: SM_Halfedge_index) -> SM_Face_index {
        self.hconn_[h.idx()].face_
    }
    /// `Surface_mesh::set_face(Halfedge_index h, Face_index f)`
    pub fn set_face(&mut self, h: SM_Halfedge_index, f: SM_Face_index) {
        self.hconn_[h.idx()].face_ = f;
    }
    /// `Surface_mesh::next(Halfedge_index h)`
    pub fn next(&self, h: SM_Halfedge_index) -> SM_Halfedge_index {
        self.hconn_[h.idx()].next_halfedge_
    }
    /// `Surface_mesh::prev(Halfedge_index h)`
    pub fn prev(&self, h: SM_Halfedge_index) -> SM_Halfedge_index {
        self.hconn_[h.idx()].prev_halfedge_
    }
    /// `Surface_mesh::set_next(Halfedge_index h, Halfedge_index nh)`: `nh` follows `h` and `h`
    /// precedes `nh`
    pub fn set_next(&mut self, h: SM_Halfedge_index, nh: SM_Halfedge_index) {
        self.hconn_[h.idx()].next_halfedge_ = nh;
        self.hconn_[nh.idx()].prev_halfedge_ = h;
    }
    /// `Surface_mesh::opposite(Halfedge_index h)`
    pub fn opposite(&self, h: SM_Halfedge_index) -> SM_Halfedge_index {
        SM_Halfedge_index(if (h.0 & 1) != 0 { h.0.wrapping_sub(1) } else { h.0.wrapping_add(1) })
    }
    /// `Surface_mesh::next_around_target(Halfedge_index h)`: `opposite(next(h))`
    pub fn next_around_target(&self, h: SM_Halfedge_index) -> SM_Halfedge_index {
        self.opposite(self.next(h))
    }

    /// `Surface_mesh::halfedge(Vertex_index v)`: an incoming halfedge of `v`, a border one when
    /// `v` is on the border
    pub fn halfedge_v(&self, v: SM_Vertex_index) -> SM_Halfedge_index {
        self.vconn_[v.idx()].halfedge_
    }
    /// `Surface_mesh::set_halfedge(Vertex_index v, Halfedge_index h)`
    pub fn set_halfedge_v(&mut self, v: SM_Vertex_index, h: SM_Halfedge_index) {
        self.vconn_[v.idx()].halfedge_ = h;
    }
    /// `Surface_mesh::halfedge(Face_index f)`
    pub fn halfedge_f(&self, f: SM_Face_index) -> SM_Halfedge_index {
        self.fconn_[f.idx()].halfedge_
    }
    /// `Surface_mesh::set_halfedge(Face_index f, Halfedge_index h)`
    pub fn set_halfedge_f(&mut self, f: SM_Face_index, h: SM_Halfedge_index) {
        self.fconn_[f.idx()].halfedge_ = h;
    }
    /// `Surface_mesh::halfedge(Edge_index e)`
    pub fn halfedge_e(&self, e: SM_Edge_index) -> SM_Halfedge_index {
        e.halfedge()
    }
    /// `Surface_mesh::edge(Halfedge_index h)`
    pub fn edge(&self, h: SM_Halfedge_index) -> SM_Edge_index {
        SM_Edge_index::from_halfedge(h)
    }

    /// `Surface_mesh::halfedge(Vertex_index source, Vertex_index target)`: the halfedge between
    /// the two, or the null halfedge
    pub fn halfedge_vv(&self, source: SM_Vertex_index, target: SM_Vertex_index) -> SM_Halfedge_index {
        let mut h = self.halfedge_v(target);
        let hh = h;

        if h.is_valid() {
            loop {
                if self.source(h) == source {
                    return h;
                }
                h = self.next_around_target(h);
                if h == hh {
                    break;
                }
            }
        }

        SM_Halfedge_index::null()
    }

    /// `Surface_mesh::is_border(Halfedge_index h)`: its face is the null face
    pub fn is_border_h(&self, h: SM_Halfedge_index) -> bool {
        !self.face(h).is_valid()
    }
    /// `Surface_mesh::is_border(Edge_index e)`: one of its two halfedges is a border halfedge
    pub fn is_border_e(&self, e: SM_Edge_index) -> bool {
        self.is_border_h(e.halfedge()) || self.is_border_h(self.opposite(e.halfedge()))
    }
    /// helpers.h, `CGAL::is_border(vertex_descriptor vd, const FaceGraph& g)`: whether a border
    /// halfedge is met going around `vd` from its halfedge (C++ returns that halfedge in an
    /// `optional`, which `add_face` reads as a bool)
    fn is_border_v_bgl(&self, vd: SM_Vertex_index) -> bool {
        for h in self.halfedges_around_target(self.halfedge_v(vd)) {
            if self.is_border_h(h) {
                return true;
            }
        }
        false
    }
    /// `Surface_mesh::is_isolated(Vertex_index v)`
    pub fn is_isolated(&self, v: SM_Vertex_index) -> bool {
        !self.halfedge_v(v).is_valid()
    }

    /// `Surface_mesh::point(Vertex_index v)`
    pub fn point(&self, v: SM_Vertex_index) -> Point_3 {
        self.vpoint_[v.idx()]
    }
    /// `Surface_mesh::point(Vertex_index v)`, the reference to write through
    pub fn point_mut(&mut self, v: SM_Vertex_index) -> &mut Point_3 {
        &mut self.vpoint_[v.idx()]
    }

    /// `Surface_mesh::halfedges_around_target(Halfedge_index h)` (iterator.h,
    /// `Halfedge_around_target_iterator`): `h`, then `opposite(next(.))` until `h` comes again
    pub fn halfedges_around_target(&self, h: SM_Halfedge_index) -> Vec<SM_Halfedge_index> {
        let mut out = Vec::new();
        let mut pos = h;
        loop {
            out.push(pos);
            pos = self.opposite(self.next(pos));
            if pos == h {
                break;
            }
        }
        out
    }

    /// `Surface_mesh::halfedges_around_face(Halfedge_index h)` (iterator.h,
    /// `Halfedge_around_face_iterator`): `h`, then `next(.)` until `h` comes again
    pub fn halfedges_around_face(&self, h: SM_Halfedge_index) -> Vec<SM_Halfedge_index> {
        let mut out = Vec::new();
        let mut pos = h;
        loop {
            out.push(pos);
            pos = self.next(pos);
            if pos == h {
                break;
            }
        }
        out
    }

    /// `Surface_mesh::vertices_around_face(Halfedge_index h)`: the target of each halfedge of
    /// `halfedges_around_face(h)`
    pub fn vertices_around_face(&self, h: SM_Halfedge_index) -> Vec<SM_Vertex_index> {
        self.halfedges_around_face(h).into_iter().map(|hf| self.target(hf)).collect()
    }

    /// `Surface_mesh::faces_around_face(Halfedge_index h)`: the face across each halfedge of
    /// `halfedges_around_face(h)`, the null face across a border edge
    pub fn faces_around_face(&self, h: SM_Halfedge_index) -> Vec<SM_Face_index> {
        self.halfedges_around_face(h).into_iter().map(|hf| self.face(self.opposite(hf))).collect()
    }
}

/// helpers.h, `CGAL::is_closed(const FaceGraph& g)`: no halfedge is a border halfedge
pub fn is_closed(g: &CGALMesh) -> bool {
    for hd in g.halfedges() {
        if g.is_border_h(hd) {
            return false;
        }
    }
    true
}

/// CgalUtils.hpp:20-29
pub fn trimesh_to_cgal(mesh: &TriMesh) -> CGALMesh {
    let mut cm = CGALMesh::new();
    let mut vmap = vec![SM_Vertex_index::null(); mesh.vertices.len()];
    for i in 0..mesh.vertices.len() {
        vmap[i] = cm.add_vertex([mesh.vertices[i].x as f64, mesh.vertices[i].y as f64, mesh.vertices[i].z as f64]);
    }
    for f in mesh.indices.iter() {
        cm.add_face(vmap[f[0] as usize], vmap[f[1] as usize], vmap[f[2] as usize]);
    }
    cm
}

/// CgalUtils.hpp:31-50
pub fn cgal_to_trimesh(cm: &CGALMesh) -> TriMesh {
    let mut mesh = TriMesh::default();
    let mut vmap: BTreeMap<SM_Vertex_index, usize> = BTreeMap::new();
    let mut idx: usize = 0;
    for v in cm.vertices() {
        if !cm.is_valid_v(v) || cm.is_removed_v(v) {
            continue;
        }
        let p = cm.point(v);
        mesh.vertices.push(Vec3f::new(p[0] as f32, p[1] as f32, p[2] as f32));
        vmap.insert(v, idx);
        idx += 1;
    }
    for f in cm.faces() {
        if !cm.is_valid_f(f) || cm.is_removed_f(f) {
            continue;
        }
        let h = cm.halfedge_f(f);
        let v0 = cm.target(h);
        let v1 = cm.target(cm.next(h));
        let v2 = cm.target(cm.next(cm.next(h)));
        // vmap[v]: a vertex that is not in the map is added with 0
        let i0 = *vmap.entry(v0).or_insert(0) as i32;
        let i1 = *vmap.entry(v1).or_insert(0) as i32;
        let i2 = *vmap.entry(v2).or_insert(0) as i32;
        mesh.indices.push(Vec3i::new(i0, i1, i2));
    }
    mesh
}

/// CgalUtils.hpp:52-137: whether the faces can be the faces of a halfedge mesh: no face with a
/// vertex twice or out of range, no directed edge twice, and the faces around every vertex one fan.
pub fn is_mesh_halfedge_compatible(mesh: &TriMesh) -> bool {
    let nv = mesh.vertices.len();
    let mut vtx_to_adj_faces: Vec<HashSet<usize>> = vec![HashSet::new(); nv];
    let mut edge_id: usize = 0;
    let mut edge_to_faces: Vec<HashSet<usize>> = Vec::new();
    let mut vtx_to_prev_vtxs: Vec<HashSet<usize>> = vec![HashSet::new(); nv];
    let mut vtx_to_next_vtxs: Vec<HashSet<usize>> = vec![HashSet::new(); nv];
    let mut vtx_vtx_to_edge: Vec<HashMap<usize, usize>> = vec![HashMap::new(); nv];

    for fid in 0..mesh.indices.len() {
        let face: &TriFace = &mesh.indices[fid];
        if face[0] == face[1] || face[1] == face[2] || face[2] == face[0] {
            return false;
        }
        for i in 0..3usize {
            // static_cast<std::size_t>(face[i]): a negative index is a huge one
            if (face[i] as i64 as usize) >= nv {
                return false;
            }
            vtx_to_adj_faces[face[i] as usize].insert(fid);

            let prev_vtx = face[(i + 2) % 3] as i64 as usize;
            let next_vtx = face[(i + 1) % 3] as i64 as usize;

            if vtx_to_prev_vtxs[face[i] as usize].contains(&prev_vtx) {
                return false;
            }
            vtx_to_prev_vtxs[face[i] as usize].insert(prev_vtx);

            if vtx_to_next_vtxs[face[i] as usize].contains(&next_vtx) {
                return false;
            }
            vtx_to_next_vtxs[face[i] as usize].insert(next_vtx);
        }

        for i in 0..3usize {
            let va = face[i] as usize;
            let vb = face[(i + 1) % 3] as usize;
            if !vtx_vtx_to_edge[va].contains_key(&vb) {
                vtx_vtx_to_edge[va].insert(vb, edge_id);
                vtx_vtx_to_edge[vb].insert(va, edge_id);
                edge_id += 1;
                edge_to_faces.push(HashSet::new());
            }
            let eid = vtx_vtx_to_edge[va][&vb];
            edge_to_faces[eid].insert(fid);
        }
    }

    for vid in 0..nv {
        if vtx_to_adj_faces[vid].is_empty() {
            continue;
        }
        let mut visited_faces: HashSet<usize> = HashSet::new();
        let mut face_queue: VecDeque<usize> = VecDeque::new();
        // *(vtx_to_adj_faces[vid].begin()): see the FIDELITY-NOTE of the file
        let first = *vtx_to_adj_faces[vid].iter().min().expect("not empty");
        face_queue.push_back(first);
        visited_faces.insert(first);
        while let Some(fid) = face_queue.pop_front() {
            let face: &TriFace = &mesh.indices[fid];
            for i in 0..3usize {
                if face[i] as usize != vid {
                    continue;
                }
                let v_next = face[(i + 1) % 3] as usize;
                let v_prev = face[(i + 2) % 3] as usize;
                for nbr in [v_next, v_prev] {
                    let eid = vtx_vtx_to_edge[vid][&nbr];
                    for &adj_fid in edge_to_faces[eid].iter() {
                        if !visited_faces.contains(&adj_fid) && vtx_to_adj_faces[vid].contains(&adj_fid) {
                            visited_faces.insert(adj_fid);
                            face_queue.push_back(adj_fid);
                        }
                    }
                }
                break;
            }
        }

        for fid in vtx_to_adj_faces[vid].iter() {
            if !visited_faces.contains(fid) {
                return false;
            }
        }
    }

    true
}

/// CgalUtils.hpp:139-142, `convert_trimesh_to_cgal(mesh, cgal_mesh)`
pub fn convert_trimesh_to_cgal(mesh: &TriMesh, cgal_mesh: &mut CGALMesh) -> bool {
    *cgal_mesh = trimesh_to_cgal(mesh);
    cgal_mesh.number_of_faces() > 0 || mesh.indices.is_empty()
}

/// CgalUtils.hpp:144-169, `convert_trimesh_to_cgal(mesh, vertex_uvs, cgal_mesh, cgal_vertex_uvs)`
pub fn convert_trimesh_to_cgal_with_uvs(
    mesh: &TriMesh,
    vertex_uvs: &[Vec2f],
    cgal_mesh: &mut CGALMesh,
    cgal_vertex_uvs: &mut Vec<Vec2f>,
) -> bool {
    cgal_mesh.clear();
    let mut vmap = vec![SM_Vertex_index::null(); mesh.vertices.len()];
    cgal_vertex_uvs.clear();

    for i in 0..mesh.vertices.len() {
        vmap[i] = cgal_mesh.add_vertex([mesh.vertices[i].x as f64, mesh.vertices[i].y as f64, mesh.vertices[i].z as f64]);
    }

    cgal_vertex_uvs.resize(cgal_mesh.num_vertices(), Vec2f::new(0.0, 0.0));
    for i in 0..mesh.vertices.len() {
        if i < vertex_uvs.len() {
            cgal_vertex_uvs[vmap[i].idx()] = vertex_uvs[i];
        } else {
            cgal_vertex_uvs[vmap[i].idx()] = Vec2f::new(0.0, 0.0);
        }
    }

    for f in mesh.indices.iter() {
        cgal_mesh.add_face(vmap[f[0] as usize], vmap[f[1] as usize], vmap[f[2] as usize]);
    }

    true
}

// -------------------------------------------------------------------------------------------------
// What ColorUtils.cpp and Repair.hpp use of the CGAL kernel and of CGAL's algorithms over a mesh.
// `Kernel` is `Exact_predicates_inexact_constructions_kernel`: its constructions are the Cartesian
// ones over `double` (written out here from the CGAL source named at each, with the fused
// multiply-adds the reference build makes of them), its predicates are exact.

/// `CGAL::squared_distance` of two points, `Compute_squared_distance_3(pt1, pt2)`
/// (CGAL/Kernel/function_objects.h): `vec = pt2 - pt1; return vec * vec;`, the scalar product of
/// the difference with itself
pub fn squared_distance(pt1: &Point_3, pt2: &Point_3) -> f64 {
    let vec = vector(pt1, pt2);
    scalar_product(&vec, &vec)
}

/// `Construct_vector_3(p, q)`: `q - p`
#[inline]
fn vector(p: &Point_3, q: &Point_3) -> Vector_3 {
    [q[0] - p[0], q[1] - p[1], q[2] - p[2]]
}

/// CGAL/Cartesian/function_objects.h, `Construct_cross_product_vector_3`
#[inline]
fn cross_product(v: &Vector_3, w: &Vector_3) -> Vector_3 {
    [v[1].mul_add(w[2], -(v[2] * w[1])), v[2].mul_add(w[0], -(v[0] * w[2])), v[0].mul_add(w[1], -(v[1] * w[0]))]
}

/// CGAL/Cartesian/function_objects.h, `Compute_scalar_product_3`:
/// `v.x() * w.x() + v.y() * w.y() + v.z() * w.z()`
#[inline]
fn scalar_product(v: &Vector_3, w: &Vector_3) -> f64 {
    v[2].mul_add(w[2], v[0].mul_add(w[0], v[1] * w[1]))
}

/// `a + b` and its rounding error (Knuth)
#[inline]
fn two_sum(a: f64, b: f64) -> (f64, f64) {
    let x = a + b;
    let bv = x - a;
    let av = x - bv;
    (x, (a - av) + (b - bv))
}

/// `a * b` and its rounding error
#[inline]
fn two_product(a: f64, b: f64) -> (f64, f64) {
    let x = a * b;
    (x, a.mul_add(b, -x))
}

/// The terms whose sum is exactly the squared distance of `p` and `q`, each multiplied by `sign`
fn squared_distance_terms(p: &Point_3, q: &Point_3, sign: f64, out: &mut Vec<f64>) {
    for i in 0..3 {
        let (s, t) = two_sum(p[i], -q[i]);
        for (a, b) in [(s, s), (2.0 * s, t), (t, t)] {
            let (x, e) = two_product(a, b);
            out.push(sign * x);
            out.push(sign * e);
        }
    }
}

/// The sign of the exact sum of the terms (Shewchuk's expansions: the terms are added one by one
/// to a sum kept as doubles that do not overlap, of which the last one that is not zero has the
/// sign of the whole)
fn sign_of_sum(terms: &[f64]) -> std::cmp::Ordering {
    let mut e: Vec<f64> = Vec::new();
    for &b in terms.iter() {
        let mut q = b;
        let mut h: Vec<f64> = Vec::with_capacity(e.len() + 1);
        for &ei in e.iter() {
            let (s, r) = two_sum(q, ei);
            q = s;
            if r != 0.0 {
                h.push(r);
            }
        }
        if q != 0.0 {
            h.push(q);
        }
        e = h;
    }
    match e.last() {
        None => std::cmp::Ordering::Equal,
        Some(&v) => v.partial_cmp(&0.0).expect("a number"),
    }
}

/// `Compare_distance_3(p, q, r)` of the kernel (CGAL/predicates/kernel_ftC3.h,
/// `cmp_dist_to_pointC3`): the distance of `p` to `q` against its distance to `r`. An exact
/// predicate: when the two squared distances computed in doubles are too close to tell, their
/// exact difference decides (CGAL does it with intervals and then with exact numbers; the result
/// is the same).
pub fn compare_distance_3(p: &Point_3, q: &Point_3, r: &Point_3) -> std::cmp::Ordering {
    // squared_distanceC3 (CGAL/constructions/kernel_ftC3.h)
    let sq = |a: &Point_3, b: &Point_3| -> f64 {
        let (dx, dy, dz) = (a[0] - b[0], a[1] - b[1], a[2] - b[2]);
        dx * dx + dy * dy + dz * dz
    };
    let dq = sq(p, q);
    let dr = sq(p, r);
    let bound = 16.0 * f64::EPSILON * (dq + dr);
    if dq.is_finite() && dr.is_finite() && dq + dr > 1e-280 && (dq - dr).abs() > bound {
        return dq.partial_cmp(&dr).expect("finite");
    }
    let mut terms: Vec<f64> = Vec::with_capacity(36);
    squared_distance_terms(p, q, 1.0, &mut terms);
    squared_distance_terms(p, r, -1.0, &mut terms);
    sign_of_sum(&terms)
}

/// `Construct_projected_point_3(Line_3(p1, p2), p)` (CGAL/Cartesian/function_objects.h): `p`
/// projected on the line through `p1` with direction `p2 - p1`
fn projection_on_line(p1: &Point_3, p2: &Point_3, p: &Point_3) -> Point_3 {
    // projects p on the line l
    let (lpx, lpy, lpz) = (p1[0], p1[1], p1[2]);
    let (ldx, ldy, ldz) = (p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]);
    let dpx = p[0] - lpx;
    let dpy = p[1] - lpy;
    let dpz = p[2] - lpz;
    let lambda = ldz.mul_add(dpz, ldx.mul_add(dpx, ldy * dpy)) / ldz.mul_add(ldz, ldx.mul_add(ldx, ldy * ldy));
    [lambda.mul_add(ldx, lpx), lambda.mul_add(ldy, lpy), lambda.mul_add(ldz, lpz)]
}

/// `CommonKernelFunctors::Construct_projected_point_3::is_inside_triangle_3_aux`
/// (CGAL/Kernel/function_objects.h)
fn is_inside_triangle_3_aux(w: &Vector_3, p1: &Point_3, p2: &Point_3, q: &Point_3, result: &mut Point_3, outside: &mut bool) -> bool {
    let v = cross_product(&vector(p1, p2), &vector(p1, q));
    if scalar_product(&v, w) < 0.0 {
        if scalar_product(&vector(p1, q), &vector(p1, p2)) >= 0.0 && scalar_product(&vector(p2, q), &vector(p2, p1)) >= 0.0 {
            *result = projection_on_line(p1, p2, q);
            return true;
        }
        *outside = true;
    }

    false
}

/// `Construct_projected_point_3::nearest_point_3`: the nearest of `p1`, `p2`, `p3` from `origin`
fn nearest_point_3(origin: &Point_3, p1: &Point_3, p2: &Point_3, p3: &Point_3) -> Point_3 {
    let dist_origin_p1 = squared_distance(origin, p1);
    let dist_origin_p2 = squared_distance(origin, p2);
    let dist_origin_p3 = squared_distance(origin, p3);

    if dist_origin_p2 >= dist_origin_p1 && dist_origin_p3 >= dist_origin_p1 {
        return *p1;
    }
    if dist_origin_p3 >= dist_origin_p2 {
        return *p2;
    }

    *p3
}

/// `Construct_projected_point_3::is_inside_triangle_3`: whether `p`, in the plane of `t`, is
/// inside `t`; when it is not, `result` is the nearest point of `t`
fn is_inside_triangle_3(p: &Point_3, t: &[Point_3; 3], w: &Vector_3, result: &mut Point_3) -> bool {
    let (t0, t1, t2) = (&t[0], &t[1], &t[2]);

    let mut outside = false;
    if is_inside_triangle_3_aux(w, t0, t1, p, result, &mut outside)
        || is_inside_triangle_3_aux(w, t1, t2, p, result, &mut outside)
        || is_inside_triangle_3_aux(w, t2, t0, p, result, &mut outside)
    {
        return false;
    }

    if outside {
        *result = nearest_point_3(p, t0, t1, t2);
        false
    } else {
        true
    }
}

/// `Construct_projected_point_3(Segment_3(a, b), query)` (CGAL/Kernel/function_objects.h)
fn projection_on_segment(a: &Point_3, b: &Point_3, query: &Point_3) -> Point_3 {
    // is_degenerate(segment)
    if a == b {
        return *a;
    }

    let to_vector = vector(a, b);
    if scalar_product(&to_vector, &vector(a, query)) <= 0.0 {
        return *a;
    }
    if scalar_product(&to_vector, &vector(b, query)) >= 0.0 {
        return *b;
    }
    // If proj is inside segment, returns it
    projection_on_line(a, b, query)
}

/// `Construct_projected_point_3(Triangle_3, Point_3)` of the kernel
/// (`CommonKernelFunctors::Construct_projected_point_3`, CGAL/Kernel/function_objects.h): the
/// point of the triangle nearest to `origin`
pub fn construct_projected_point_3(triangle: &[Point_3; 3], origin: &Point_3) -> Point_3 {
    // supporting_plane(triangle): plane_from_pointsC3 (CGAL/constructions/kernel_ftC3.h)
    let (p, q, r) = (&triangle[0], &triangle[1], &triangle[2]);
    let rpx = p[0] - r[0];
    let rpy = p[1] - r[1];
    let rpz = p[2] - r[2];
    let rqx = q[0] - r[0];
    let rqy = q[1] - r[1];
    let rqz = q[2] - r[2];
    // Cross product rp * rq
    let pa = rpy.mul_add(rqz, -(rqy * rpz));
    let pb = rpz.mul_add(rqx, -(rqz * rpx));
    let pc = rpx.mul_add(rqy, -(rqx * rpy));
    // pd = - pa*rx - pb*ry - pc*rz;
    let pd = (-pc).mul_add(r[2], (-pa).mul_add(r[0], -(pb * r[1])));

    // is_degenerate(plane)
    if pa == 0.0 && pb == 0.0 && pc == 0.0 {
        // If the plane is degenerate, then the triangle is degenerate, and
        // one tries to find to which segment it is equivalent.
        let (a, b, c) = (p, q, r);
        let ab = vector(a, b);
        let ac = vector(a, c);
        let bc = vector(b, c);
        // (std::max)((std::max)(x(ab), y(ab)), z(ab))
        let max3 = |v: &Vector_3| -> f64 {
            let m = if v[0] < v[1] { v[1] } else { v[0] };
            if m < v[2] {
                v[2]
            } else {
                m
            }
        };
        let linf_ab = max3(&ab);
        let linf_ac = max3(&ac);
        let linf_bc = max3(&bc);

        if linf_ab > linf_ac {
            if linf_ab > linf_bc {
                // ab is the maximal segment
                return projection_on_segment(a, b, origin);
            } else {
                // ab > ac, bc >= ab, use bc
                return projection_on_segment(b, c, origin);
            }
        } else {
            // ab <= ac
            if linf_ac > linf_bc {
                // ac is the maximal segment
                return projection_on_segment(a, c, origin);
            } else {
                // ab <= ac, ac <= bc, use bc
                return projection_on_segment(b, c, origin);
            }
        }
    } // degenerate plane

    // Project origin on triangle supporting plane: projection_planeC3
    let num = pc.mul_add(origin[2], pa.mul_add(origin[0], pb * origin[1])) + pd;
    let den = pc.mul_add(pc, pa.mul_add(pa, pb * pb));
    let lambda = num / den;
    let proj: Point_3 = [(-lambda).mul_add(pa, origin[0]), (-lambda).mul_add(pb, origin[1]), (-lambda).mul_add(pc, origin[2])];

    let mut moved_point: Point_3 = [0.0; 3];
    let inside = is_inside_triangle_3(&proj, triangle, &[pa, pb, pc], &mut moved_point);

    // If proj is inside triangle, return it
    if inside {
        return proj;
    }

    // Else return the constructed point
    moved_point
}

/// `PMP::compute_face_normal(f, pmesh)` (CGAL/Polygon_mesh_processing/compute_normal.h): the sum
/// of the normals of the triangles `(source(he), target(he), target(.))` going around the face
/// from `he = halfedge(f)`, each half the cross product of two of its edges, divided by its length
/// when it is not the null vector and its length is not 0
pub fn compute_face_normal(f: SM_Face_index, pmesh: &CGALMesh) -> Vector_3 {
    let mut normal: Vector_3 = [0.0, 0.0, 0.0];

    // internal::sum_normals
    let mut he = pmesh.halfedge_f(f);
    let v = pmesh.source(he);
    let mut the = pmesh.target(he);
    he = pmesh.next(he);
    let mut tnhe = pmesh.target(he);

    let pv = pmesh.point(v);

    while v != tnhe {
        let pvn = pmesh.point(the);
        let pvnn = pmesh.point(tnhe);

        // internal::triangle_normal(pv, pvn, pvnn)
        let n = cross_product(&vector(&pvn, &pvnn), &vector(&pvn, &pv));
        // construct_scaled_vector_3_object()(n, FT(1)/FT(2))
        let n: Vector_3 = [0.5 * n[0], 0.5 * n[1], 0.5 * n[2]];

        normal = [normal[0] + n[0], normal[1] + n[1], normal[2] + n[2]];

        the = tnhe;
        he = pmesh.next(he);
        tnhe = pmesh.target(he);
    }

    if !(normal[0] == 0.0 && normal[1] == 0.0 && normal[2] == 0.0) {
        // internal::normalize
        let sx = normal[0] * normal[0];
        let sy = normal[1] * normal[1];
        let sz = normal[2] * normal[2];
        let norm = (sx + sy + sz).sqrt();
        if norm != 0.0 {
            normal = [normal[0] / norm, normal[1] / norm, normal[2] / norm];
        }
    }

    normal
}

/// `CGAL::centroid(begin, end)` of points (CGAL/centroid.h): the sum of the points, taken as
/// vectors, divided by their number
pub fn centroid(points: &[Point_3]) -> Point_3 {
    let mut v: Vector_3 = [0.0, 0.0, 0.0];
    let mut nb_pts: u32 = 0;
    for p in points.iter() {
        v = [v[0] + p[0], v[1] + p[1], v[2] + p[2]];
        nb_pts += 1;
    }
    let n = nb_pts as f64;
    [v[0] / n, v[1] / n, v[2] / n]
}

// libc++'s `std::nth_element` (`__algorithm/nth_element.h`), which `AABB_traits::Split_primitives`
// orders the faces with: which faces end up in which half of the tree depends on it. Iterators
// are indices into the slice.

/// `__algorithm/sort.h`, `__sort3` (the arm without `__use_branchless_sort`): whether it swapped
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

/// `__algorithm/sort.h`, `__selection_sort` over `[first, last)`
fn libcxx_selection_sort<T: Copy, F: FnMut(&T, &T) -> bool>(v: &mut [T], mut first: usize, last: usize, c: &mut F) {
    let lm1 = last - 1;
    while first != lm1 {
        // std::__min_element(first, last, comp)
        let mut i = first;
        let mut j = first;
        loop {
            j += 1;
            if j == last {
                break;
            }
            if c(&v[j], &v[i]) {
                i = j;
            }
        }
        if i != first {
            v.swap(first, i);
        }
        first += 1;
    }
}

/// `__algorithm/nth_element.h`, `__nth_element(first, nth, last, comp)`
pub fn libcxx_nth_element<T: Copy, F: FnMut(&T, &T) -> bool>(v: &mut [T], mut first: usize, nth: usize, mut last: usize, c: &mut F) {
    const LIMIT: usize = 7;
    loop {
        if nth == last {
            return;
        }
        let len = last - first;
        match len {
            0 | 1 => return,
            2 => {
                last -= 1;
                if c(&v[last], &v[first]) {
                    v.swap(first, last);
                }
                return;
            }
            3 => {
                libcxx_sort3(v, first, first + 1, last - 1, c);
                return;
            }
            _ => {}
        }
        if len <= LIMIT {
            libcxx_selection_sort(v, first, last, c);
            return;
        }
        // __len > __limit >= 3
        let mut m = first + len / 2;
        let lm1 = last - 1;
        let mut n_swaps: u32 = libcxx_sort3(v, first, m, lm1, c) as u32;
        // *__m is median
        // partition [__first, __m) < *__m and *__m <= [__m, __last)
        let mut i = first;
        let mut j = lm1;
        // j points beyond range to be tested, *__lm1 is known to be <= *__m
        // The search going up is known to be guarded but the search coming down isn't.
        // Prime the downward search with a guard.
        let mut restart = false;
        if !c(&v[i], &v[m]) {
            // if *__first == *__m
            // __nth_element_find_guard: manually guard downward moving __j against __i
            let mut found = false;
            loop {
                j -= 1;
                if i == j {
                    break;
                }
                if c(&v[j], &v[m]) {
                    found = true; // found guard for downward moving __j, now use unguarded partition
                    break;
                }
            }
            if found {
                v.swap(i, j);
                n_swaps += 1;
            } else {
                // *__first == *__m, *__m <= all other elements
                // Partition instead into [__first, __i) == *__first and *__first < [__i, __last)
                i += 1; // __first + 1
                j = last;
                j -= 1;
                if !c(&v[first], &v[j]) {
                    // we need a guard if *__first == *(__last-1)
                    loop {
                        if i == j {
                            return; // [__first, __last) all equivalent elements
                        } else if c(&v[first], &v[i]) {
                            v.swap(i, j);
                            n_swaps += 1;
                            i += 1;
                            break;
                        }
                        i += 1;
                    }
                }
                // [__first, __i) == *__first and *__first < [__j, __last) and __j == __last - 1
                if i == j {
                    return;
                }
                loop {
                    while !c(&v[first], &v[i]) {
                        i += 1;
                    }
                    loop {
                        j -= 1;
                        if !c(&v[first], &v[j]) {
                            break;
                        }
                    }
                    if i >= j {
                        break;
                    }
                    v.swap(i, j);
                    n_swaps += 1;
                    i += 1;
                }
                let _ = n_swaps;
                // [__first, __i) == *__first and *__first < [__i, __last)
                // The first part is sorted,
                if nth < i {
                    return;
                }
                // __nth_element the second part
                first = i;
                restart = true;
            }
        }
        if restart {
            continue;
        }
        i += 1;
        // j points beyond range to be tested, *__lm1 is known to be <= *__m
        // if not yet partitioned...
        if i < j {
            // known that *(__i - 1) < *__m
            loop {
                // __m still guards upward moving __i
                while c(&v[i], &v[m]) {
                    i += 1;
                }
                // It is now known that a guard exists for downward moving __j
                loop {
                    j -= 1;
                    if c(&v[j], &v[m]) {
                        break;
                    }
                }
                if i >= j {
                    break;
                }
                v.swap(i, j);
                n_swaps += 1;
                // It is known that __m != __j
                // If __m just moved, follow it
                if m == i {
                    m = j;
                }
                i += 1;
            }
        }
        // [__first, __i) < *__m and *__m <= [__i, __last)
        if i != m && c(&v[m], &v[i]) {
            v.swap(i, m);
            n_swaps += 1;
        }
        // [__first, __i) < *__i and *__i <= [__i+1, __last)
        if nth == i {
            return;
        }
        if n_swaps == 0 {
            // We were given a perfectly partitioned sequence.  Coincidence?
            if nth < i {
                // Check for [__first, __i) already sorted
                j = first;
                m = first;
                loop {
                    j += 1;
                    if j == i {
                        // [__first, __i) sorted
                        return;
                    }
                    if c(&v[j], &v[m]) {
                        // not yet sorted, so sort
                        break;
                    }
                    m = j;
                }
            } else {
                // Check for [__i, __last) already sorted
                j = i;
                m = i;
                loop {
                    j += 1;
                    if j == last {
                        // [__i, __last) sorted
                        return;
                    }
                    if c(&v[j], &v[m]) {
                        // not yet sorted, so sort
                        break;
                    }
                    m = j;
                }
            }
        }
        // __nth_element on range containing __nth
        if nth < i {
            last = i;
        } else {
            i += 1;
            first = i;
        }
    }
}

/// `CGAL::Bbox_3`: xmin, ymin, zmin, xmax, ymax, zmax
type Bbox_3 = [f64; 6];

/// A child of a node of the tree: another node or a face (CGAL/AABB_tree/internal/AABB_node.h
/// keeps both behind one pointer and tells them apart by the number of faces below the node)
#[derive(Debug, Clone, Copy)]
enum AABB_child {
    Node(usize),
    Primitive(SM_Face_index),
}

/// CGAL/AABB_tree/internal/AABB_node.h, `class AABB_node`
#[derive(Debug, Clone, Copy)]
struct AABB_node {
    m_bbox: Bbox_3,
    m_p_left_child: AABB_child,
    m_p_right_child: AABB_child,
}

/// `CGAL::AABB_tree<AABB_traits<Kernel, AABB_face_graph_triangle_primitive<CGALMesh>>>` over the
/// faces of a mesh, as far as `closest_point` goes (ColorUtils.cpp:80-82, 308-309): CGAL/AABB_tree.h,
/// CGAL/AABB_traits_3.h, CGAL/AABB_tree/internal/AABB_node.h and AABB_traversal_traits.h.
///
/// A primitive is a face; its triangle and its reference point are read from the mesh whenever
/// they are needed (`AABB_face_graph_triangle_primitive` does not cache them), so the mesh is an
/// argument of every member that reads it. The boxes of the nodes are those of the mesh at the
/// first query, when CGAL builds the tree, and are not updated when vertices move afterwards:
/// `smooth_region_geom_boundary` moves vertices between queries and gets what this gives.
///
/// FIDELITY-NOTE: the first candidate of a query (`best_hint`) is the nearest of the faces'
/// reference points as they were at `accelerate_distance_queries`. CGAL finds it with a k-d tree
/// (`Orthogonal_k_neighbor_search`), the crate has no port of that: here all the points are
/// compared, and of several at the same computed distance the first in the order of the faces is
/// taken, where CGAL takes the one its k-d tree meets first. The candidate only starts the search
/// (it is replaced by the projection on any face that is not farther), so the point returned is
/// the same unless a box is within rounding of the sphere through the candidate.
#[derive(Debug, Clone)]
pub struct Tree {
    /// `m_primitives`, put in the order of the tree by `build`
    m_primitives: Vec<SM_Face_index>,
    /// `m_nodes`; the root is the first
    m_nodes: Vec<AABB_node>,
    m_need_build: bool,
    /// the points of the search tree
    m_search_points: Vec<Point_3>,
    m_search_tree_constructed: bool,
}

impl Tree {
    /// `Tree(mesh.faces().begin(), mesh.faces().end(), mesh)`: the faces are inserted, nothing is built
    pub fn new(mesh: &CGALMesh) -> Self {
        Tree { m_primitives: mesh.faces().collect(), m_nodes: Vec::new(), m_need_build: true, m_search_points: Vec::new(), m_search_tree_constructed: false }
    }

    /// `AABB_tree::empty()`
    pub fn empty(&self) -> bool {
        self.m_primitives.is_empty()
    }

    /// `One_point_from_face_descriptor_map` (CGAL/boost/graph/property_maps.h): the reference
    /// point of a face
    fn reference_point(mesh: &CGALMesh, f: SM_Face_index) -> Point_3 {
        mesh.point(mesh.target(mesh.halfedge_f(f)))
    }

    /// `Triangle_from_face_descriptor_map` (CGAL/boost/graph/property_maps.h): the triangle of a face
    fn datum(mesh: &CGALMesh, f: SM_Face_index) -> [Point_3; 3] {
        let h = mesh.halfedge_f(f);
        [mesh.point(mesh.target(h)), mesh.point(mesh.target(mesh.next(h))), mesh.point(mesh.source(h))]
    }

    /// `AABB_tree::accelerate_distance_queries()` (`build_kd_tree`): the reference points of the
    /// faces are collected for the hints
    pub fn accelerate_distance_queries(&mut self, mesh: &CGALMesh) -> bool {
        if self.m_primitives.is_empty() {
            return true;
        }
        self.m_search_points = self.m_primitives.iter().map(|&f| Self::reference_point(mesh, f)).collect();
        self.m_search_tree_constructed = true;
        true
    }

    /// `AABB_traits::Compute_bbox` over `[first, beyond)`: the box of the triangles of the faces
    fn compute_bbox(&self, mesh: &CGALMesh, first: usize, beyond: usize) -> Bbox_3 {
        let mut bbox: Bbox_3 = [f64::INFINITY, f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
        for &f in self.m_primitives[first..beyond].iter() {
            for p in Self::datum(mesh, f).iter() {
                for i in 0..3 {
                    // (std::min)(xmin(), b.xmin()), (std::max)(xmax(), b.xmax())
                    if p[i] < bbox[i] {
                        bbox[i] = p[i];
                    }
                    if bbox[i + 3] < p[i] {
                        bbox[i + 3] = p[i];
                    }
                }
            }
        }
        bbox
    }

    /// `AABB_traits::longest_axis`
    fn longest_axis(bbox: &Bbox_3) -> usize {
        let dx = bbox[3] - bbox[0];
        let dy = bbox[4] - bbox[1];
        let dz = bbox[5] - bbox[2];

        if dx >= dy {
            if dx >= dz {
                0
            } else {
                // dz>dx and dx>=dy
                2
            }
        } else {
            // dy>dx
            if dy >= dz {
                1
            } else {
                // dz>dy and dy>dx
                2
            }
        }
    }

    /// `AABB_tree::expand(node, first, beyond, range, compute_bbox, split_primitives)`
    fn expand(&mut self, mesh: &CGALMesh, node: usize, first: usize, beyond: usize, range: usize) {
        let bbox = self.compute_bbox(mesh, first, beyond);
        self.m_nodes[node].m_bbox = bbox;

        // sort primitives along longest axis aabb: AABB_traits::Split_primitives
        let middle = first + (beyond - first) / 2;
        let axis = Self::longest_axis(&bbox);
        let mut less = |p1: &SM_Face_index, p2: &SM_Face_index| -> bool { Self::reference_point(mesh, *p1)[axis] < Self::reference_point(mesh, *p2)[axis] };
        libcxx_nth_element(&mut self.m_primitives, first, middle, beyond, &mut less);

        match range {
            2 => {
                self.m_nodes[node].m_p_left_child = AABB_child::Primitive(self.m_primitives[first]);
                self.m_nodes[node].m_p_right_child = AABB_child::Primitive(self.m_primitives[first + 1]);
            }
            3 => {
                let right = self.new_node();
                self.m_nodes[node].m_p_left_child = AABB_child::Primitive(self.m_primitives[first]);
                self.m_nodes[node].m_p_right_child = AABB_child::Node(right);
                self.expand(mesh, right, first + 1, beyond, 2);
            }
            _ => {
                let new_range = range / 2;
                let left = self.new_node();
                let right = self.new_node();
                self.m_nodes[node].m_p_left_child = AABB_child::Node(left);
                self.m_nodes[node].m_p_right_child = AABB_child::Node(right);
                self.expand(mesh, left, first, first + new_range, new_range);
                self.expand(mesh, right, first + new_range, beyond, range - new_range);
            }
        }
    }

    /// `AABB_tree::new_node()`
    fn new_node(&mut self) -> usize {
        self.m_nodes.push(AABB_node {
            m_bbox: [0.0; 6],
            m_p_left_child: AABB_child::Primitive(SM_Face_index::null()),
            m_p_right_child: AABB_child::Primitive(SM_Face_index::null()),
        });
        self.m_nodes.len() - 1
    }

    /// `AABB_tree::build()`
    pub fn build(&mut self, mesh: &CGALMesh) {
        self.m_nodes.clear();

        if self.m_primitives.len() > 1 {
            // constructs the tree
            let root = self.new_node();
            let n = self.m_primitives.len();
            self.expand(mesh, root, 0, n, n);
        }
        self.m_need_build = false;
    }

    /// `AABB_tree::best_hint(query)`: see the FIDELITY-NOTE of the type
    fn best_hint(&mut self, mesh: &CGALMesh, query: &Point_3) -> Point_3 {
        if !self.m_search_tree_constructed {
            self.accelerate_distance_queries(mesh);
        }
        // Euclidean_distance::interruptible_transformed_distance: `distance += diff*diff` per coordinate
        let dist = |p: &Point_3| -> f64 {
            let mut distance = 0.0f64;
            for i in 0..3 {
                let diff = query[i] - p[i];
                distance = diff.mul_add(diff, distance);
            }
            distance
        };
        let mut best = self.m_search_points[0];
        let mut best_dist = dist(&best);
        for p in self.m_search_points.iter().skip(1) {
            let d = dist(p);
            if d < best_dist {
                best_dist = d;
                best = *p;
            }
        }
        best
    }

    /// `Projection_traits::intersection(query, primitive)` with `AABB_traits::Closest_point`:
    /// the projection of the query on the face replaces the closest point unless it is farther
    fn projection_intersection(mesh: &CGALMesh, query: &Point_3, primitive: SM_Face_index, m_closest_point: &mut Point_3) {
        let closest_point = construct_projected_point_3(&Self::datum(mesh, primitive), query);
        let new_closest_point =
            if compare_distance_3(query, &closest_point, m_closest_point) == std::cmp::Ordering::Greater { *m_closest_point } else { closest_point };
        if new_closest_point != *m_closest_point {
            *m_closest_point = new_closest_point;
        }
    }

    /// `Projection_traits::do_intersect(query, node)`: `AABB_traits::Compare_distance(query,
    /// node.bbox(), m_closest_point) == SMALLER`, which is whether the sphere around the query
    /// through the closest point meets the box
    fn projection_do_intersect(&self, query: &Point_3, node: usize, m_closest_point: &Point_3) -> bool {
        do_intersect_sphere_bbox(query, squared_distance(query, m_closest_point), &self.m_nodes[node].m_bbox, true)
    }

    /// `AABB_node::traversal(query, traits, nb_primitives)` with `Projection_traits`
    fn traversal(&self, mesh: &CGALMesh, node: usize, query: &Point_3, m_closest_point: &mut Point_3, nb_primitives: usize) {
        let left = self.m_nodes[node].m_p_left_child;
        let right = self.m_nodes[node].m_p_right_child;
        let data = |c: AABB_child| -> SM_Face_index {
            match c {
                AABB_child::Primitive(f) => f,
                AABB_child::Node(_) => unreachable!("a face is expected here"),
            }
        };
        let child = |c: AABB_child| -> usize {
            match c {
                AABB_child::Node(n) => n,
                AABB_child::Primitive(_) => unreachable!("a node is expected here"),
            }
        };
        // Recursive traversal
        match nb_primitives {
            2 => {
                Self::projection_intersection(mesh, query, data(left), m_closest_point);
                // traits.go_further() is true
                Self::projection_intersection(mesh, query, data(right), m_closest_point);
            }
            3 => {
                Self::projection_intersection(mesh, query, data(left), m_closest_point);
                if self.projection_do_intersect(query, child(right), m_closest_point) {
                    self.traversal(mesh, child(right), query, m_closest_point, 2);
                }
            }
            _ => {
                if self.projection_do_intersect(query, child(left), m_closest_point) {
                    self.traversal(mesh, child(left), query, m_closest_point, nb_primitives / 2);
                    if self.projection_do_intersect(query, child(right), m_closest_point) {
                        self.traversal(mesh, child(right), query, m_closest_point, nb_primitives - nb_primitives / 2);
                    }
                } else if self.projection_do_intersect(query, child(right), m_closest_point) {
                    self.traversal(mesh, child(right), query, m_closest_point, nb_primitives - nb_primitives / 2);
                }
            }
        }
    }

    /// `AABB_tree::closest_point(query)`: the point of the mesh's faces nearest to `query`. The
    /// tree must not be empty. The tree is built at the first query.
    pub fn closest_point(&mut self, mesh: &CGALMesh, query: &Point_3) -> Point_3 {
        let hint = self.best_hint(mesh, query);
        // closest_point(query, hint): Projection_traits projection_traits(hint, ...); traversal(query, projection_traits)
        let mut m_closest_point = hint;
        match self.m_primitives.len() {
            0 => {}
            1 => Self::projection_intersection(mesh, query, self.m_primitives[0], &mut m_closest_point),
            n => {
                // root_node() builds the tree when it is needed
                if self.m_need_build {
                    self.build(mesh);
                }
                self.traversal(mesh, 0, query, &mut m_closest_point, n);
            }
        }
        m_closest_point
    }
}

/// `Do_intersect_3(Sphere_3(center, ssr), Bbox_3, overestimate)` of the kernel's static filters
/// (CGAL/Filtered_kernel/internal/Static_filters/Do_intersect_3.h): whether the sphere of squared
/// radius `ssr` meets the box. With `overestimate` the answer is true whenever the doubles cannot
/// tell; without it, and for a radius outside the range of the filter, the exact predicate
/// answers (CGAL/Intersections_3/internal/Bbox_3_Sphere_3_do_intersect.h: the squared distance of
/// the center to the box is at most `ssr`).
pub fn do_intersect_sphere_bbox(c: &Point_3, ssr: f64, b: &Bbox_3, overestimate: bool) -> bool {
    let (scx, scy, scz) = (c[0], c[1], c[2]);
    let (bxmin, bymin, bzmin, bxmax, bymax, bzmax) = (b[0], b[1], b[2], b[3], b[4], b[5]);

    if ssr < 1.11261183279326254436e-293 || ssr > 2.80889552322236673473e+306 {
        return do_intersect_sphere_bbox_exact(c, ssr, b);
    }
    let mut distance = 0.0f64;
    let mut max1 = 0.0f64;
    let mut double_tmp_result = 0.0f64;
    let mut eps = 0.0f64;
    // one coordinate: `Some(answer)` when the filter stops there
    let mut step = |sc: f64, bmin: f64, bmax: f64, first: bool| -> Option<bool> {
        let d;
        if sc < bmin {
            d = bmin - sc;
        } else if sc > bmax {
            d = sc - bmax;
        } else {
            return None;
        }
        if first {
            max1 = d;
            distance = d * d;
        } else {
            if max1 < d {
                max1 = d;
            }
            let sq = d * d;
            distance += sq;
        }
        double_tmp_result = distance - ssr;

        if max1 < 3.33558365626356687717e-147 || max1 > 1.67597599124282407923e+153 {
            if overestimate {
                return Some(true);
            } else {
                return Some(do_intersect_sphere_bbox_exact(c, ssr, b));
            }
        }

        let m = max1 * max1;
        eps = 1.99986535548615598560e-15 * (if ssr < m { m } else { ssr });

        if double_tmp_result > eps {
            return Some(false);
        }
        None
    };
    if let Some(r) = step(scx, bxmin, bxmax, true) {
        return r;
    }
    if let Some(r) = step(scy, bymin, bymax, false) {
        return r;
    }
    if let Some(r) = step(scz, bzmin, bzmax, false) {
        return r;
    }

    // double_tmp_result and eps were growing all the time
    // no need to test for > eps as done earlier in at least one case
    if double_tmp_result < -eps {
        true
    } else {
        if overestimate {
            return true;
        }
        do_intersect_sphere_bbox_exact(c, ssr, b)
    }
}

/// CGAL/Intersections_3/internal/Bbox_3_Sphere_3_do_intersect.h, `do_intersect_sphere_box_3`,
/// evaluated exactly: `distance <= sphere.squared_radius()`, `distance` the sum of the squares
/// of what the center is outside the box by in each coordinate
fn do_intersect_sphere_bbox_exact(c: &Point_3, ssr: f64, b: &Bbox_3) -> bool {
    let mut terms: Vec<f64> = Vec::with_capacity(19);
    for i in 0..3 {
        let (s, t) = if c[i] < b[i] {
            two_sum(b[i], -c[i])
        } else if c[i] > b[i + 3] {
            two_sum(c[i], -b[i + 3])
        } else {
            continue;
        };
        for (x, y) in [(s, s), (2.0 * s, t), (t, t)] {
            let (p, e) = two_product(x, y);
            terms.push(p);
            terms.push(e);
        }
    }
    terms.push(-ssr);
    sign_of_sum(&terms) != std::cmp::Ordering::Greater
}

/// `PMP::extract_boundary_cycles(pm, out)` (CGAL/Polygon_mesh_processing/border.h): one border
/// halfedge of every border cycle, in the order of the halfedges
pub fn extract_boundary_cycles(pm: &CGALMesh) -> Vec<SM_Halfedge_index> {
    let mut out: Vec<SM_Halfedge_index> = Vec::new();
    let mut hedge_handled: HashSet<SM_Halfedge_index> = HashSet::new();
    for h in pm.halfedges() {
        if pm.is_border_h(h) && hedge_handled.insert(h) {
            out.push(h);
            for h2 in pm.halfedges_around_face(h) {
                hedge_handled.insert(h2);
            }
        }
    }
    out
}

/// The algorithms of CGAL's Polygon Mesh Processing package that ColorUtils.cpp and Repair.hpp
/// call and the crate has no port of. The functions that call them (`remesh_mesh`,
/// `check_and_repair_self_intersect`, `CloseBoundariesAndRepairManifoldness`, `RepairMesh` and
/// what calls those) take an implementation of this trait and are ports of everything around
/// the calls.
///
/// FIDELITY-NOTE: the crate implements this trait nowhere, so those functions cannot be run by
/// it yet. An implementation has to leave the mesh as CGAL would after `collect_garbage()` in
/// the order CGAL's iterators give: `CGALMesh` has no removed elements.
pub trait PolygonMeshProcessing {
    /// `PMP::does_self_intersect(mesh)`
    fn does_self_intersect(&self, mesh: &CGALMesh) -> bool;
    /// `PMP::experimental::remove_self_intersections(mesh)`
    fn remove_self_intersections(&self, mesh: &mut CGALMesh) -> bool;
    /// `PMP::isotropic_remeshing(mesh.faces(), target_edge_length, mesh,
    /// number_of_iterations(..).protect_constraints(..).edge_is_constrained_map(..)
    /// .vertex_is_constrained_map(..).collapse_constraints(..))`. The maps are indexed by the
    /// edge and by the vertex.
    #[allow(clippy::too_many_arguments)]
    fn isotropic_remeshing(
        &self,
        mesh: &mut CGALMesh,
        target_edge_length: f64,
        number_of_iterations: usize,
        protect_constraints: bool,
        edge_is_constrained_map: &[bool],
        vertex_is_constrained_map: &[bool],
        collapse_constraints: bool,
    );
    /// `PMP::repair_polygon_soup(points, polygons)`
    fn repair_polygon_soup(&self, points: &mut Vec<Point_3>, polygons: &mut Vec<Vec<usize>>);
    /// `PMP::orient_polygon_soup(points, polygons)`
    fn orient_polygon_soup(&self, points: &mut Vec<Point_3>, polygons: &mut Vec<Vec<usize>>) -> bool;
    /// `PMP::polygon_soup_to_polygon_mesh(points, polygons, out)`
    fn polygon_soup_to_polygon_mesh(&self, points: &[Point_3], polygons: &[Vec<usize>], out: &mut CGALMesh);
    /// `PMP::remove_degenerate_faces(mesh)`
    fn remove_degenerate_faces(&self, mesh: &mut CGALMesh) -> bool;
    /// `PMP::stitch_borders(mesh)`
    fn stitch_borders(&self, mesh: &mut CGALMesh) -> usize;
    /// `PMP::duplicate_non_manifold_vertices(mesh)`
    fn duplicate_non_manifold_vertices(&self, mesh: &mut CGALMesh) -> usize;
    /// `PMP::triangulate_hole(mesh, border_halfedge, std::back_inserter(patch_faces))`
    fn triangulate_hole(&self, mesh: &mut CGALMesh, border_halfedge: SM_Halfedge_index, patch_faces: &mut Vec<SM_Face_index>);
}
