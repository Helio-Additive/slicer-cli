//! Port of `libslic3r/CutSurface.cpp` (+ `CutSurface.hpp`): cutting a 2D shape out of the surface
//! of models.
//!
//! C++ Reference:
//! - CutSurface.hpp
//! - CutSurface.cpp
//! - CGAL/Surface_mesh/Surface_mesh.h, CGAL/Surface_mesh/Properties.h,
//!   CGAL/boost/graph/Euler_operations.h, CGAL/boost/graph/iterator.h (module `surface_mesh`)
//!
//! The functions of the namespace `priv` are the free functions of this file, in the order of the
//! C++; a second overload of a name is in a small module named after what it takes
//! (`of_shapes::to_cgal`, `of_mesh::bounding_box`, `of_patch::bounding_box`,
//! `of_patch::is_over_whole_expoly`).
//!
//! `CutMesh` (`CGAL::Surface_mesh<EpicKernel::Point_3>`) is the module `surface_mesh` of this
//! file. Two algorithms of CGAL are taken through the trait `PolygonMeshProcessing`, with the
//! visitor callbacks and the named parameters CutSurface.cpp gives them:
//! `CGAL::Polygon_mesh_processing::corefine` and `CGAL::Polygon_mesh_processing::clip` (and, for
//! `corefine_test`, `CGAL::IO::read_OFF`). The crate has no implementation of the trait;
//! `tests/cut_surface_native_parity.rs` replays what the pin's CGAL did. The two predicates of the
//! kernel the file calls are computed here: `CGAL::orientation` of four points (exact, in
//! `texture_to_color::cgal_utils`) and `AABB_tree::number_of_intersected_primitives` of a ray
//! (the exact triangle and ray test over every face, see `Tree`).
//!
//! `VertexShapeMap` holds `const IntersectingElement *` in C++, pointing into the two property
//! arrays of the shape mesh, which nothing writes after `to_cgal(shapes, projection)`. Here it
//! holds the value the pointer points to (`Option<IntersectingElement>`, `None` for `nullptr`).
//!
//! The debug output of the file (`#ifdef DEBUG_OUTPUT_DIR`: `prepare_dir`, `initialize_store` and
//! the `store` overloads) is not compiled into the engine; of that part the three functions
//! without file output are here (`create_indexed_triangle_set`, `get_triangle_tip`,
//! `create_contour_its`).
//!
//! Nothing in the slicing path calls this unit.

#![allow(non_camel_case_types, non_upper_case_globals, non_snake_case, clippy::too_many_arguments)]

use std::cell::{Cell, RefCell};
use std::cmp::Ordering;
use std::collections::VecDeque;

// CutSurface.cpp:1  #include "CutSurface.hpp"
use crate::aabb_tree_lines::tree2d;
use crate::clipper_utils::ApplySafetyOffset;
use crate::emboss::{IProject3d, IProjection};
use crate::ex_polygons_index::{ExPolygonsIndex, ExPolygonsIndices};
use crate::geometry::geometry::eigen;
use crate::geometry::{BoundingBox, ExPolygon, ExPolygons, Point, Points, Polygon, Polygons, Vec2d, Vec3d};
use crate::normal_utils::{indexed_triangle_set, Vec3f};
use crate::texture_to_color::cgal_utils::{do_intersect_triangle_3_ray_3, orientation_3};
// CutSurface.cpp:883  its_face_normal
use crate::triangle_mesh::its_face_normal_indices;

// CutSurface.hpp:18  using Index = unsigned int;
type Index = u32;
// CutSurface.hpp:19  using Contour = std::vector<Index>;
type Contour = Vec<Index>;
// CutSurface.hpp:20  using Contours = std::vector<Contour>;
type Contours = Vec<Contour>;

/// Represents cutted surface from object
/// Extend index triangle set by outlines
/// CutSurface.hpp:15-23
///
/// `struct SurfaceCut : public indexed_triangle_set` — Rust has no inheritance,
/// so the base `indexed_triangle_set` is embedded as the `its` field. Field
/// access in the C++ source such as `cut.vertices` / `cut.indices` maps to
/// `cut.its.vertices` / `cut.its.indices`.
#[derive(Debug, Clone, Default)]
pub struct SurfaceCut {
    // CutSurface.hpp:15  : public indexed_triangle_set
    pub its: indexed_triangle_set,
    // CutSurface.hpp:22  Contours contours;
    // list of circulated open surface
    pub contours: Contours,
}

impl SurfaceCut {
    pub fn new() -> Self {
        SurfaceCut::default()
    }

    /// `indexed_triangle_set::empty()` (admesh/stl.h:247): `indices.empty() || vertices.empty()`
    pub fn empty(&self) -> bool {
        self.its.indices.is_empty() || self.its.vertices.is_empty()
    }
}
// ===========================================================================
// CGAL::Surface_mesh<EpicKernel::Point_3> (CutSurface.cpp:72-80 `CutMesh`, `VI`, `HI`, `EI`, `FI`, `P3`)
// ===========================================================================

/// What CutSurface.cpp uses of `CGAL::Surface_mesh`, written out from the CGAL headers the engine
/// compiles: CGAL/Surface_mesh/Surface_mesh.h, CGAL/Surface_mesh/Properties.h,
/// CGAL/boost/graph/Euler_operations.h (`add_face`) and CGAL/boost/graph/iterator.h (the ranges
/// around a face and around a vertex). Indices, the numbering of halfedges (`2*e` and `2*e+1`),
/// the properties by name, the removal flags with their free lists and the garbage collection are
/// those of CGAL; the line numbers cited are those of the CGAL 5.4 headers.
///
/// A `Property_map` is a handle on an array the mesh shares with it (CGAL: a pointer to a
/// `Property_array` the container owns), so a visitor can write a property of a mesh while the
/// mesh is being refined, as in C++. `Surface_mesh::join`, the I/O and the per-element validity
/// tests are not here: CutSurface.cpp does not call them.
#[allow(non_camel_case_types, non_snake_case, clippy::len_without_is_empty)]
pub mod surface_mesh {
    use std::any::Any;
    use std::cell::RefCell;
    use std::marker::PhantomData;
    use std::rc::Rc;

    /// Surface_mesh.h:62 `typedef boost::uint32_t size_type;`
    pub type size_type = u32;
    /// `EpicKernel::Point_3`: three doubles
    pub type Point_3 = [f64; 3];
    /// `(std::numeric_limits<size_type>::max)()`: the index of a null descriptor
    pub const SM_NULL: size_type = u32::MAX;

    // Surface_mesh.h:58-88 `class SM_Index<T>` and the three classes derived from it
    macro_rules! sm_index {
        ($name:ident) => {
            #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
            pub struct $name(pub size_type);
            impl $name {
                /// `explicit SM_Index(size_type _idx)`
                pub fn new(idx: size_type) -> Self {
                    $name(idx)
                }
                /// `SM_Index()`: the null descriptor
                pub fn null() -> Self {
                    $name(SM_NULL)
                }
                /// `SM_Index::reset()`
                pub fn reset(&mut self) {
                    self.0 = SM_NULL;
                }
                /// `SM_Index::is_valid()`
                pub fn is_valid(&self) -> bool {
                    self.0 != SM_NULL
                }
                /// `SM_Index::idx()`
                pub fn idx(&self) -> size_type {
                    self.0
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

    /// Surface_mesh.h:232-290 `class SM_Edge_index`: it keeps the halfedge it was made from; its
    /// index, and what `==` and `<` compare, is that halfedge's index halved.
    #[derive(Debug, Clone, Copy)]
    pub struct SM_Edge_index {
        halfedge_: SM_Halfedge_index,
    }

    impl SM_Edge_index {
        /// `SM_Edge_index()`
        pub fn null() -> Self {
            SM_Edge_index { halfedge_: SM_Halfedge_index::null() }
        }
        /// `explicit SM_Edge_index(size_type idx) : halfedge_(idx * 2)`
        pub fn new(idx: size_type) -> Self {
            SM_Edge_index { halfedge_: SM_Halfedge_index(idx.wrapping_mul(2)) }
        }
        /// `explicit SM_Edge_index(SM_Halfedge_index he)`
        pub fn from_halfedge(he: SM_Halfedge_index) -> Self {
            SM_Edge_index { halfedge_: he }
        }
        /// `SM_Edge_index::halfedge()`
        pub fn halfedge(&self) -> SM_Halfedge_index {
            self.halfedge_
        }
        /// `SM_Edge_index::idx()`: `(size_type)halfedge_ / 2`
        pub fn idx(&self) -> size_type {
            self.halfedge_.0 / 2
        }
        /// `SM_Edge_index::reset()`
        pub fn reset(&mut self) {
            self.halfedge_.reset();
        }
        /// `SM_Edge_index::is_valid()`
        pub fn is_valid(&self) -> bool {
            self.halfedge_.is_valid()
        }
    }
    impl Default for SM_Edge_index {
        fn default() -> Self {
            Self::null()
        }
    }
    impl PartialEq for SM_Edge_index {
        fn eq(&self, other: &Self) -> bool {
            self.idx() == other.idx()
        }
    }
    impl Eq for SM_Edge_index {}
    impl PartialOrd for SM_Edge_index {
        fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
            Some(self.cmp(other))
        }
    }
    impl Ord for SM_Edge_index {
        fn cmp(&self, other: &Self) -> std::cmp::Ordering {
            self.idx().cmp(&other.idx())
        }
    }

    /// Surface_mesh.h:474-478 `struct Vertex_connectivity`
    #[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
    pub struct Vertex_connectivity {
        /// an incoming halfedge per vertex (it will be a border halfedge for border vertices)
        pub halfedge_: SM_Halfedge_index,
    }

    /// Surface_mesh.h:483-493 `struct Halfedge_connectivity`
    #[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
    pub struct Halfedge_connectivity {
        /// face incident to halfedge
        pub face_: SM_Face_index,
        /// vertex the halfedge points to
        pub vertex_: SM_Vertex_index,
        /// next halfedge within a face (or along a border)
        pub next_halfedge_: SM_Halfedge_index,
        /// previous halfedge within a face (or along a border)
        pub prev_halfedge_: SM_Halfedge_index,
    }

    /// Surface_mesh.h:498-502 `struct Face_connectivity`
    #[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
    pub struct Face_connectivity {
        /// a halfedge that is part of the face
        pub halfedge_: SM_Halfedge_index,
    }

    // -----------------------------------------------------------------------
    // Properties.h
    // -----------------------------------------------------------------------

    /// Properties.h:37-87 `class Base_property_array`
    trait Base_property_array {
        fn name(&self) -> &str;
        fn reserve(&self, n: usize);
        fn resize(&self, n: usize);
        fn shrink_to_fit(&self);
        fn push_back(&self);
        fn reset(&self, idx: usize);
        fn swap(&self, i0: usize, i1: usize);
        /// `clone()`: a deep copy of the array
        fn clone_array(&self) -> Rc<dyn Base_property_array>;
        fn as_any_rc(self: Rc<Self>) -> Rc<dyn Any>;
        /// the address of the array, what `Property_container::remove` compares
        fn address(&self) -> *const ();
    }

    /// Properties.h:96-213 `class Property_array<T>`: the values `data_` and the default `value_`
    pub struct Property_array<T> {
        name_: String,
        data_: RefCell<Vec<T>>,
        value_: T,
    }

    impl<T: Clone + 'static> Base_property_array for Property_array<T> {
        fn name(&self) -> &str {
            &self.name_
        }
        // Properties.h:112-115
        fn reserve(&self, n: usize) {
            let mut d = self.data_.borrow_mut();
            let len = d.len();
            if n > len {
                d.reserve(n - len);
            }
        }
        // Properties.h:117-120 `data_.resize(n, value_)`
        fn resize(&self, n: usize) {
            self.data_.borrow_mut().resize(n, self.value_.clone());
        }
        // Properties.h:158-161
        fn shrink_to_fit(&self) {
            self.data_.borrow_mut().shrink_to_fit();
        }
        // Properties.h:122-125 `data_.push_back(value_)`
        fn push_back(&self) {
            self.data_.borrow_mut().push(self.value_.clone());
        }
        // Properties.h:127-130 `data_[idx] = value_`
        fn reset(&self, idx: usize) {
            self.data_.borrow_mut()[idx] = self.value_.clone();
        }
        // Properties.h:163-168
        fn swap(&self, i0: usize, i1: usize) {
            self.data_.borrow_mut().swap(i0, i1);
        }
        // Properties.h:170-175
        fn clone_array(&self) -> Rc<dyn Base_property_array> {
            Rc::new(Property_array { name_: self.name_.clone(), data_: RefCell::new(self.data_.borrow().clone()), value_: self.value_.clone() })
        }
        fn as_any_rc(self: Rc<Self>) -> Rc<dyn Any> {
            self
        }
        fn address(&self) -> *const () {
            self as *const Self as *const ()
        }
    }

    /// The index types a property is keyed by (`Property_selector<I>`, Surface_mesh.h:1901-1943)
    pub trait SM_Key: Copy {
        /// `(size_type)i`: the position in the arrays
        fn index(self) -> usize;
        fn container(m: &Surface_mesh) -> &Property_container;
        fn container_mut(m: &mut Surface_mesh) -> &mut Property_container;
    }
    impl SM_Key for SM_Vertex_index {
        fn index(self) -> usize {
            self.0 as usize
        }
        fn container(m: &Surface_mesh) -> &Property_container {
            &m.vprops_
        }
        fn container_mut(m: &mut Surface_mesh) -> &mut Property_container {
            &mut m.vprops_
        }
    }
    impl SM_Key for SM_Halfedge_index {
        fn index(self) -> usize {
            self.0 as usize
        }
        fn container(m: &Surface_mesh) -> &Property_container {
            &m.hprops_
        }
        fn container_mut(m: &mut Surface_mesh) -> &mut Property_container {
            &mut m.hprops_
        }
    }
    impl SM_Key for SM_Edge_index {
        fn index(self) -> usize {
            self.idx() as usize
        }
        fn container(m: &Surface_mesh) -> &Property_container {
            &m.eprops_
        }
        fn container_mut(m: &mut Surface_mesh) -> &mut Property_container {
            &mut m.eprops_
        }
    }
    impl SM_Key for SM_Face_index {
        fn index(self) -> usize {
            self.0 as usize
        }
        fn container(m: &Surface_mesh) -> &Property_container {
            &m.fprops_
        }
        fn container_mut(m: &mut Surface_mesh) -> &mut Property_container {
            &mut m.fprops_
        }
    }

    /// Properties.h:496-600 `class Property_map_base<I, T, ...>` / Surface_mesh.h `Property_map<I, T>`:
    /// a handle on a `Property_array<T>`. A default one refers to no array (`parray_ == nullptr`).
    pub struct Property_map<I, T> {
        parray_: Option<Rc<Property_array<T>>>,
        _key: PhantomData<I>,
    }

    impl<I, T> Clone for Property_map<I, T> {
        fn clone(&self) -> Self {
            Property_map { parray_: self.parray_.clone(), _key: PhantomData }
        }
    }
    impl<I, T> Default for Property_map<I, T> {
        fn default() -> Self {
            Property_map { parray_: None, _key: PhantomData }
        }
    }

    impl<I: SM_Key, T: Clone + 'static> Property_map<I, T> {
        fn array(&self) -> &Rc<Property_array<T>> {
            self.parray_.as_ref().expect("Property_map: no property array (parray_ == nullptr)")
        }
        /// `Property_map_base::reset()`
        pub fn reset(&mut self) {
            self.parray_ = None;
        }
        /// `operator bool`: whether the handle refers to an array
        pub fn is_valid(&self) -> bool {
            self.parray_.is_some()
        }
        /// `operator[](const I& i) const`: the value of `i`
        pub fn get(&self, i: I) -> T {
            self.array().data_.borrow()[i.index()].clone()
        }
        /// `operator[](const I& i) = value`
        pub fn set(&self, i: I, value: T) {
            self.array().data_.borrow_mut()[i.index()] = value;
        }
        /// the number of values (`end() - begin()`)
        pub fn len(&self) -> usize {
            self.array().data_.borrow().len()
        }
        /// `begin()`..`end()`: a copy of the values of every index, removed ones included
        pub fn to_vec(&self) -> Vec<T> {
            self.array().data_.borrow().clone()
        }
        /// the value at a position of the array (`*(begin() + pos)`)
        pub fn get_at(&self, pos: usize) -> T {
            self.array().data_.borrow()[pos].clone()
        }
        /// `*(begin() + pos) = value`
        pub fn set_at(&self, pos: usize, value: T) {
            self.array().data_.borrow_mut()[pos] = value;
        }
    }

    /// Properties.h:231-491 `class Property_container<Ref_class, Key>`
    #[derive(Default)]
    pub struct Property_container {
        parrays_: Vec<Rc<dyn Base_property_array>>,
        size_: usize,
        capacity_: usize,
    }

    impl Clone for Property_container {
        // Properties.h:246-258 `operator=`: performs deep copy of property arrays
        fn clone(&self) -> Self {
            Property_container { parrays_: self.parrays_.iter().map(|a| a.clone_array()).collect(), size_: self.size_, capacity_: self.capacity_ }
        }
    }

    impl Property_container {
        // Properties.h:302-305
        pub fn size(&self) -> usize {
            self.size_
        }
        pub fn capacity(&self) -> usize {
            self.capacity_
        }
        pub fn n_properties(&self) -> usize {
            self.parrays_.len()
        }
        // Properties.h:311-317
        pub fn properties(&self) -> Vec<String> {
            self.parrays_.iter().map(|a| a.name().to_string()).collect()
        }

        // Properties.h:324-336 `get<T>(name, i)`: the array `i` when it has this name and this type
        fn get_at<I: SM_Key, T: Clone + 'static>(&self, name: &str, i: usize) -> (Property_map<I, T>, bool) {
            if self.parrays_[i].name() == name {
                if let Ok(array) = self.parrays_[i].clone().as_any_rc().downcast::<Property_array<T>>() {
                    return (Property_map { parray_: Some(array), _key: PhantomData }, true);
                }
            }
            (Property_map::default(), false)
        }

        // Properties.h:338-360 `add<T>(name, t)`
        pub fn add<I: SM_Key, T: Clone + 'static>(&mut self, name: &str, t: T) -> (Property_map<I, T>, bool) {
            for i in 0..self.parrays_.len() {
                let mut out = self.get_at::<I, T>(name, i);
                if out.1 {
                    out.1 = false;
                    return out;
                }
            }
            // otherwise add the property
            let p = Rc::new(Property_array { name_: name.to_string(), data_: RefCell::new(Vec::new()), value_: t });
            p.reserve(self.capacity_);
            p.resize(self.size_);
            self.parrays_.push(p.clone());
            (Property_map { parray_: Some(p), _key: PhantomData }, true)
        }

        // Properties.h:363-376 `get<T>(name)`
        pub fn get<I: SM_Key, T: Clone + 'static>(&self, name: &str) -> (Property_map<I, T>, bool) {
            for i in 0..self.parrays_.len() {
                let out = self.get_at::<I, T>(name, i);
                if out.1 {
                    return out;
                }
            }
            (Property_map::default(), false)
        }

        // Properties.h:403-420 `remove<T>(h)`
        pub fn remove<I: SM_Key, T: Clone + 'static>(&mut self, h: &mut Property_map<I, T>) -> bool {
            let Some(target) = h.parray_.as_ref().map(|a| Rc::as_ptr(a) as *const ()) else {
                return false;
            };
            for it in 0..self.parrays_.len() {
                if self.parrays_[it].address() == target {
                    self.parrays_.remove(it);
                    h.reset();
                    return true;
                }
            }
            false
        }

        // Properties.h:424-430 `clear()`: delete all properties
        pub fn clear(&mut self) {
            self.parrays_.clear();
            self.size_ = 0;
        }
        // Properties.h:434-439
        pub fn reserve(&mut self, n: usize) {
            for a in &self.parrays_ {
                a.reserve(n);
            }
            self.capacity_ = n.max(self.capacity_);
        }
        // Properties.h:442-447
        pub fn resize(&mut self, n: usize) {
            for a in &self.parrays_ {
                a.resize(n);
            }
            self.size_ = n;
        }
        // Properties.h:450-457: deletes every property past the first `n`
        pub fn resize_property_array(&mut self, n: usize) {
            if self.parrays_.len() <= n {
                return;
            }
            self.parrays_.truncate(n);
        }
        // Properties.h:460-465
        pub fn shrink_to_fit(&mut self) {
            for a in &self.parrays_ {
                a.shrink_to_fit();
            }
            self.capacity_ = self.size_;
        }
        // Properties.h:468-474
        pub fn push_back(&mut self) {
            for a in &self.parrays_ {
                a.push_back();
            }
            self.size_ += 1;
            self.capacity_ = self.size_.max(self.capacity_);
        }
        // Properties.h:477-481
        pub fn reset(&mut self, idx: usize) {
            for a in &self.parrays_ {
                a.reset(idx);
            }
        }
        // Properties.h:484-488
        pub fn swap(&self, i0: usize, i1: usize) {
            for a in &self.parrays_ {
                a.swap(i0, i1);
            }
        }
    }

    /// The descriptors `Surface_mesh::halfedge` takes: a vertex (its incoming halfedge), a face
    /// (a halfedge of it) or an edge (the halfedge it was made from)
    pub trait Has_halfedge: Copy {
        fn halfedge_of(self, m: &Surface_mesh) -> SM_Halfedge_index;
        fn set_halfedge_of(self, m: &Surface_mesh, h: SM_Halfedge_index);
    }
    impl Has_halfedge for SM_Vertex_index {
        // Surface_mesh.h:1670-1673
        fn halfedge_of(self, m: &Surface_mesh) -> SM_Halfedge_index {
            m.vconn_.get(self).halfedge_
        }
        // Surface_mesh.h:1676-1679
        fn set_halfedge_of(self, m: &Surface_mesh, h: SM_Halfedge_index) {
            m.vconn_.set(self, Vertex_connectivity { halfedge_: h });
        }
    }
    impl Has_halfedge for SM_Face_index {
        // Surface_mesh.h:1683-1686
        fn halfedge_of(self, m: &Surface_mesh) -> SM_Halfedge_index {
            m.fconn_.get(self).halfedge_
        }
        // Surface_mesh.h:1689-1692
        fn set_halfedge_of(self, m: &Surface_mesh, h: SM_Halfedge_index) {
            m.fconn_.set(self, Face_connectivity { halfedge_: h });
        }
    }
    impl Has_halfedge for SM_Edge_index {
        // Surface_mesh.h:1763-1766
        fn halfedge_of(self, _m: &Surface_mesh) -> SM_Halfedge_index {
            self.halfedge()
        }
        fn set_halfedge_of(self, _m: &Surface_mesh, _h: SM_Halfedge_index) {
            unreachable!("Surface_mesh has no set_halfedge(Edge_index, Halfedge_index)")
        }
    }

    /// The descriptors `Surface_mesh::is_removed` and `has_valid_index` take
    pub trait Removable: Copy {
        fn is_removed_in(self, m: &Surface_mesh) -> bool;
        fn has_valid_index_in(self, m: &Surface_mesh) -> bool;
    }
    impl Removable for SM_Vertex_index {
        // Surface_mesh.h:1311-1314
        fn is_removed_in(self, m: &Surface_mesh) -> bool {
            m.vremoved_.get(self)
        }
        // Surface_mesh.h:1385-1388
        fn has_valid_index_in(self, m: &Surface_mesh) -> bool {
            self.0 < m.num_vertices()
        }
    }
    impl Removable for SM_Halfedge_index {
        // Surface_mesh.h:1317-1320
        fn is_removed_in(self, m: &Surface_mesh) -> bool {
            m.eremoved_.get(m.edge(self))
        }
        // Surface_mesh.h:1391-1394
        fn has_valid_index_in(self, m: &Surface_mesh) -> bool {
            self.0 < m.num_halfedges()
        }
    }
    impl Removable for SM_Edge_index {
        // Surface_mesh.h:1323-1326
        fn is_removed_in(self, m: &Surface_mesh) -> bool {
            m.eremoved_.get(self)
        }
        // Surface_mesh.h:1396-1399
        fn has_valid_index_in(self, m: &Surface_mesh) -> bool {
            self.idx() < m.num_edges()
        }
    }
    impl Removable for SM_Face_index {
        // Surface_mesh.h:1329-1332
        fn is_removed_in(self, m: &Surface_mesh) -> bool {
            m.fremoved_.get(self)
        }
        // Surface_mesh.h:1401-1404
        fn has_valid_index_in(self, m: &Surface_mesh) -> bool {
            self.0 < m.num_faces()
        }
    }

    /// The state of a mesh as plain arrays, every index included (removed ones too): what a
    /// `Surface_mesh` is set from by `Surface_mesh::set_raw`.
    #[derive(Debug, Clone, Default, PartialEq)]
    pub struct Raw_surface_mesh {
        /// `v:connectivity`: the halfedge of each vertex
        pub vconn: Vec<size_type>,
        /// `h:connectivity`: face, vertex, next and previous halfedge of each halfedge
        pub hconn: Vec<[size_type; 4]>,
        /// `f:connectivity`: the halfedge of each face
        pub fconn: Vec<size_type>,
        /// `v:point`
        pub points: Vec<Point_3>,
        /// `v:removed`, `e:removed`, `f:removed`
        pub vremoved: Vec<bool>,
        pub eremoved: Vec<bool>,
        pub fremoved: Vec<bool>,
        /// `vertices_freelist_`, `edges_freelist_`, `faces_freelist_`
        pub freelists: [size_type; 3],
        /// `garbage_`
        pub garbage: bool,
    }

    /// Surface_mesh.h `class Surface_mesh<P>` with `P = EpicKernel::Point_3`
    pub struct Surface_mesh {
        // Surface_mesh.h:2120-2145
        vprops_: Property_container,
        hprops_: Property_container,
        eprops_: Property_container,
        fprops_: Property_container,

        vconn_: Property_map<SM_Vertex_index, Vertex_connectivity>,
        hconn_: Property_map<SM_Halfedge_index, Halfedge_connectivity>,
        fconn_: Property_map<SM_Face_index, Face_connectivity>,

        vremoved_: Property_map<SM_Vertex_index, bool>,
        eremoved_: Property_map<SM_Edge_index, bool>,
        fremoved_: Property_map<SM_Face_index, bool>,

        vpoint_: Property_map<SM_Vertex_index, Point_3>,

        removed_vertices_: size_type,
        removed_edges_: size_type,
        removed_faces_: size_type,

        vertices_freelist_: size_type,
        edges_freelist_: size_type,
        faces_freelist_: size_type,
        garbage_: bool,
        recycle_: bool,

        anonymous_property_: size_type,
    }

    impl Default for Surface_mesh {
        fn default() -> Self {
            Self::new()
        }
    }

    impl Clone for Surface_mesh {
        // Surface_mesh.h:2215-2250 `operator=`: a deep copy of all properties
        fn clone(&self) -> Self {
            // deep copy of property containers
            let vprops_ = self.vprops_.clone();
            let hprops_ = self.hprops_.clone();
            let eprops_ = self.eprops_.clone();
            let fprops_ = self.fprops_.clone();
            // property handles contain pointers, have to be reassigned
            Surface_mesh {
                vconn_: vprops_.get("v:connectivity").0,
                hconn_: hprops_.get("h:connectivity").0,
                fconn_: fprops_.get("f:connectivity").0,
                vremoved_: vprops_.get("v:removed").0,
                eremoved_: eprops_.get("e:removed").0,
                fremoved_: fprops_.get("f:removed").0,
                vpoint_: vprops_.get("v:point").0,
                vprops_,
                hprops_,
                eprops_,
                fprops_,
                // how many elements are removed?
                removed_vertices_: self.removed_vertices_,
                removed_edges_: self.removed_edges_,
                removed_faces_: self.removed_faces_,
                vertices_freelist_: self.vertices_freelist_,
                edges_freelist_: self.edges_freelist_,
                faces_freelist_: self.faces_freelist_,
                garbage_: self.garbage_,
                recycle_: self.recycle_,
                anonymous_property_: self.anonymous_property_,
            }
        }
    }

    impl Surface_mesh {
        /// Surface_mesh.h:2192-2211 `Surface_mesh()`
        pub fn new() -> Self {
            let mut vprops_ = Property_container::default();
            let mut hprops_ = Property_container::default();
            let mut eprops_ = Property_container::default();
            let mut fprops_ = Property_container::default();
            // allocate standard properties
            let vconn_ = vprops_.add("v:connectivity", Vertex_connectivity::default()).0;
            let hconn_ = hprops_.add("h:connectivity", Halfedge_connectivity::default()).0;
            let fconn_ = fprops_.add("f:connectivity", Face_connectivity::default()).0;
            let vpoint_ = vprops_.add("v:point", [0.0f64; 3]).0;
            let vremoved_ = vprops_.add("v:removed", false).0;
            let eremoved_ = eprops_.add("e:removed", false).0;
            let fremoved_ = fprops_.add("f:removed", false).0;
            Surface_mesh {
                vprops_,
                hprops_,
                eprops_,
                fprops_,
                vconn_,
                hconn_,
                fconn_,
                vremoved_,
                eremoved_,
                fremoved_,
                vpoint_,
                removed_vertices_: 0,
                removed_edges_: 0,
                removed_faces_: 0,
                vertices_freelist_: SM_NULL,
                edges_freelist_: SM_NULL,
                faces_freelist_: SM_NULL,
                garbage_: false,
                recycle_: true,
                anonymous_property_: 0,
            }
        }

        // ---- Adding Vertices, Edges, and Faces ----

        /// Surface_mesh.h:931-945 `add_vertex()`
        fn add_vertex_empty(&mut self) -> SM_Vertex_index {
            let inf = SM_NULL;
            if self.recycle_ && self.vertices_freelist_ != inf {
                let idx = self.vertices_freelist_;
                self.vertices_freelist_ = self.vconn_.get(SM_Vertex_index(self.vertices_freelist_)).halfedge_.0;
                self.removed_vertices_ -= 1;
                self.vremoved_.set(SM_Vertex_index(idx), false);
                self.vprops_.reset(idx as usize);
                SM_Vertex_index(idx)
            } else {
                self.vprops_.push_back();
                SM_Vertex_index(self.num_vertices() - 1)
            }
        }

        /// Surface_mesh.h:950-955 `add_vertex(const Point& p)`
        pub fn add_vertex(&mut self, p: Point_3) -> SM_Vertex_index {
            let v = self.add_vertex_empty();
            self.vpoint_.set(v, p);
            v
        }

        /// Surface_mesh.h:962-982 `add_edge()`
        fn add_edge_empty(&mut self) -> SM_Halfedge_index {
            let inf = SM_NULL;
            if self.recycle_ && self.edges_freelist_ != inf {
                let idx = self.edges_freelist_;
                self.edges_freelist_ = self.hconn_.get(SM_Halfedge_index(self.edges_freelist_)).next_halfedge_.0;
                self.removed_edges_ -= 1;
                self.eremoved_.set(SM_Edge_index::from_halfedge(SM_Halfedge_index(idx)), false);
                self.hprops_.reset(idx as usize);
                self.hprops_.reset(self.opposite(SM_Halfedge_index(idx)).0 as usize);
                self.eprops_.reset(SM_Edge_index::from_halfedge(SM_Halfedge_index(idx)).idx() as usize);
                SM_Halfedge_index(idx)
            } else {
                self.eprops_.push_back();
                self.hprops_.push_back();
                self.hprops_.push_back();
                SM_Halfedge_index(self.num_halfedges() - 2)
            }
        }

        /// Surface_mesh.h:990-999 `add_edge(Vertex_index v0, Vertex_index v1)`: the halfedge with
        /// `v1` as target
        pub fn add_edge(&mut self, v0: SM_Vertex_index, v1: SM_Vertex_index) -> SM_Halfedge_index {
            debug_assert!(v0 != v1);
            let h = self.add_edge_empty();
            self.set_target(h, v1);
            self.set_target(self.opposite(h), v0);
            h
        }

        /// Surface_mesh.h:1002-1016 `add_face()`
        fn add_face_empty(&mut self) -> SM_Face_index {
            let inf = SM_NULL;
            if self.recycle_ && self.faces_freelist_ != inf {
                let idx = self.faces_freelist_;
                self.faces_freelist_ = self.fconn_.get(SM_Face_index(self.faces_freelist_)).halfedge_.0;
                self.removed_faces_ -= 1;
                self.fprops_.reset(idx as usize);
                self.fremoved_.set(SM_Face_index(idx), false);
                SM_Face_index(idx)
            } else {
                self.fprops_.push_back();
                SM_Face_index(self.num_faces() - 1)
            }
        }

        /// Surface_mesh.h:1029-1034 `add_face(Vertex_index v0, Vertex_index v1, Vertex_index v2)`
        pub fn add_face(&mut self, v0: SM_Vertex_index, v1: SM_Vertex_index, v2: SM_Vertex_index) -> SM_Face_index {
            self.add_face_range(&[v0, v1, v2])
        }

        /// Surface_mesh.h:2431-2437 `add_face(const Range& vertices)`, which is
        /// `CGAL::Euler::add_face(vr, g)` (Euler_operations.h:694-869): the face over the
        /// vertices in this order, or the null face when it cannot be added.
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
                // internal::is_isolated(vertices[i], g) is `halfedge(v, g) == null_halfedge()`;
                // is_border(vertices[i], g) is CGAL/boost/graph/helpers.h:61-73
                if !self.is_isolated(vertices[i]) && !self.is_border_vertex(vertices[i], true) {
                    return Self::null_face();
                }

                // std::pair<halfedge_descriptor, bool> he = halfedge(vertices[i], vertices[ii], g);
                let he = self.halfedge_between(vertices[i], vertices[ii]);
                halfedges[i] = he; // collect if exists
                is_new[i] = !he.is_valid();

                if !is_new[i] && !self.is_border(halfedges[i]) {
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
                            if !(!self.is_border(border_prev) || border_prev == inner_prev) {
                                break;
                            }
                        }
                        border_next = self.next(border_prev);
                        debug_assert!(self.is_border(border_prev));
                        debug_assert!(self.is_border(border_next));

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
                    debug_assert!(halfedges[i] != Self::null_halfedge());

                    // as it may be recycled we have to reset it
                    self.set_face(self.opposite(halfedges[i]), Self::null_face());
                    debug_assert!(self.source(halfedges[i]) == vertices[i]);
                }
            }
            // create the face
            let f = self.add_face_empty();
            self.set_halfedge(f, halfedges[n - 1]);

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
                            self.set_halfedge(v, border_prev);
                        }
                        2 => {
                            // next is new, prev is old
                            border_next = self.next(inner_prev);
                            next_cache.push((outer_prev, border_next));
                            self.set_halfedge(v, outer_prev);
                        }
                        _ => {
                            // both are new
                            // try to pick a border halfedge with v as target
                            let mut hv = self.halfedge(v);
                            if hv != Self::null_halfedge() && !self.is_border(hv) {
                                for h_around_v in self.halfedges_around_target(hv) {
                                    if self.is_border(h_around_v) {
                                        hv = h_around_v;
                                        break;
                                    }
                                }
                                if !self.is_border(hv) {
                                    hv = Self::null_halfedge();
                                }
                            }

                            if hv == Self::null_halfedge() {
                                self.set_halfedge(v, outer_prev);
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

        // ---- Low-Level Removal Functions ----

        /// Surface_mesh.h:1063-1068
        pub fn remove_vertex(&mut self, v: SM_Vertex_index) {
            self.vremoved_.set(v, true);
            self.removed_vertices_ += 1;
            self.garbage_ = true;
            self.vconn_.set(v, Vertex_connectivity { halfedge_: SM_Halfedge_index(self.vertices_freelist_) });
            self.vertices_freelist_ = v.0;
        }

        /// Surface_mesh.h:1072-1077
        pub fn remove_edge(&mut self, e: SM_Edge_index) {
            self.eremoved_.set(e, true);
            self.removed_edges_ += 1;
            self.garbage_ = true;
            let h = SM_Halfedge_index(e.idx() << 1);
            let mut c = self.hconn_.get(h);
            c.next_halfedge_ = SM_Halfedge_index(self.edges_freelist_);
            self.hconn_.set(h, c);
            self.edges_freelist_ = e.idx() << 1;
        }

        /// Surface_mesh.h:1082-1087
        pub fn remove_face(&mut self, f: SM_Face_index) {
            self.fremoved_.set(f, true);
            self.removed_faces_ += 1;
            self.garbage_ = true;
            self.fconn_.set(f, Face_connectivity { halfedge_: SM_Halfedge_index(self.faces_freelist_) });
            self.faces_freelist_ = f.0;
        }

        // ---- Memory Management ----

        /// Surface_mesh.h:1100-1103
        pub fn number_of_vertices(&self) -> size_type {
            self.num_vertices() - self.number_of_removed_vertices()
        }
        /// Surface_mesh.h:1106-1109
        pub fn number_of_halfedges(&self) -> size_type {
            self.num_halfedges() - self.number_of_removed_halfedges()
        }
        /// Surface_mesh.h:1112-1115
        pub fn number_of_edges(&self) -> size_type {
            self.num_edges() - self.number_of_removed_edges()
        }
        /// Surface_mesh.h:1118-1121
        pub fn number_of_faces(&self) -> size_type {
            self.num_faces() - self.number_of_removed_faces()
        }
        /// Surface_mesh.h:1124-1129
        pub fn is_empty(&self) -> bool {
            self.num_vertices() == self.number_of_removed_vertices()
                && self.num_halfedges() == self.number_of_removed_halfedges()
                && self.num_faces() == self.number_of_removed_faces()
        }

        /// Surface_mesh.h:2316-2336 `clear_without_removing_property_maps()`
        pub fn clear_without_removing_property_maps(&mut self) {
            self.vprops_.resize(0);
            self.hprops_.resize(0);
            self.eprops_.resize(0);
            self.fprops_.resize(0);

            self.vprops_.shrink_to_fit();
            self.hprops_.shrink_to_fit();
            self.eprops_.shrink_to_fit();
            self.fprops_.shrink_to_fit();

            self.removed_vertices_ = 0;
            self.removed_edges_ = 0;
            self.removed_faces_ = 0;
            self.vertices_freelist_ = SM_NULL;
            self.edges_freelist_ = SM_NULL;
            self.faces_freelist_ = SM_NULL;
            self.garbage_ = false;
            self.recycle_ = true;
            self.anonymous_property_ = 0;
        }

        /// Surface_mesh.h:2306-2314 `clear()`
        pub fn clear(&mut self) {
            self.clear_without_removing_property_maps();
            self.remove_all_property_maps();
        }

        /// Surface_mesh.h:1142-1150
        pub fn reserve(&mut self, nvertices: size_type, nedges: size_type, nfaces: size_type) {
            self.vprops_.reserve(nvertices as usize);
            self.hprops_.reserve(2 * nedges as usize);
            self.eprops_.reserve(nedges as usize);
            self.fprops_.reserve(nfaces as usize);
        }

        /// Surface_mesh.h:1152-1160
        pub fn resize(&mut self, nvertices: size_type, nedges: size_type, nfaces: size_type) {
            self.vprops_.resize(nvertices as usize);
            self.hprops_.resize(2 * nedges as usize);
            self.eprops_.resize(nedges as usize);
            self.fprops_.resize(nfaces as usize);
        }

        // ---- Garbage Collection ----

        /// Surface_mesh.h:1282 the number of used and removed vertices
        pub fn num_vertices(&self) -> size_type {
            self.vprops_.size() as size_type
        }
        /// Surface_mesh.h:1285
        pub fn num_halfedges(&self) -> size_type {
            self.hprops_.size() as size_type
        }
        /// Surface_mesh.h:1288
        pub fn num_edges(&self) -> size_type {
            self.eprops_.size() as size_type
        }
        /// Surface_mesh.h:1291
        pub fn num_faces(&self) -> size_type {
            self.fprops_.size() as size_type
        }
        /// Surface_mesh.h:1296
        pub fn number_of_removed_vertices(&self) -> size_type {
            self.removed_vertices_
        }
        /// Surface_mesh.h:1299
        pub fn number_of_removed_halfedges(&self) -> size_type {
            2 * self.removed_edges_
        }
        /// Surface_mesh.h:1302
        pub fn number_of_removed_edges(&self) -> size_type {
            self.removed_edges_
        }
        /// Surface_mesh.h:1305
        pub fn number_of_removed_faces(&self) -> size_type {
            self.removed_faces_
        }
        /// Surface_mesh.h:1311-1332 `is_removed(v | h | e | f)`
        pub fn is_removed<K: Removable>(&self, k: K) -> bool {
            k.is_removed_in(self)
        }
        /// Surface_mesh.h:1336
        pub fn has_garbage(&self) -> bool {
            self.garbage_
        }
        /// Surface_mesh.h:1385-1404 `has_valid_index(v | h | e | f)`
        pub fn has_valid_index<K: Removable>(&self, k: K) -> bool {
            k.has_valid_index_in(self)
        }
        /// Surface_mesh.h:2700-2707
        pub fn set_recycle_garbage(&mut self, b: bool) {
            self.recycle_ = b;
        }
        /// Surface_mesh.h:2710-2716
        pub fn does_recycle_garbage(&self) -> bool {
            self.recycle_
        }

        /// Surface_mesh.h:2482-2677 `collect_garbage()`: really removes vertices, halfedges, edges
        /// and faces which are marked removed
        pub fn collect_garbage(&mut self) {
            if !self.has_garbage() {
                return;
            }

            let mut nV = self.num_vertices() as i32;
            let mut nE = self.num_edges() as i32;
            let mut nH = self.num_halfedges() as i32;
            let mut nF = self.num_faces() as i32;

            // setup index mapping
            let mut vmap: Property_map<SM_Vertex_index, SM_Vertex_index> = self.add_property_map("v:garbage-collection", SM_Vertex_index::null()).0;
            let mut hmap: Property_map<SM_Halfedge_index, SM_Halfedge_index> = self.add_property_map("h:garbage-collection", SM_Halfedge_index::null()).0;
            let mut fmap: Property_map<SM_Face_index, SM_Face_index> = self.add_property_map("f:garbage-collection", SM_Face_index::null()).0;
            for i in 0..nV {
                vmap.set(SM_Vertex_index(i as u32), SM_Vertex_index(i as u32));
            }
            for i in 0..nH {
                hmap.set(SM_Halfedge_index(i as u32), SM_Halfedge_index(i as u32));
            }
            for i in 0..nF {
                fmap.set(SM_Face_index(i as u32), SM_Face_index(i as u32));
            }

            // really remove vertices
            if nV > 0 {
                let mut i0 = 0i32;
                let mut i1 = nV - 1;
                loop {
                    // find first removed and last un-removed
                    while !self.vremoved_.get(SM_Vertex_index(i0 as u32)) && i0 < i1 {
                        i0 += 1;
                    }
                    while self.vremoved_.get(SM_Vertex_index(i1 as u32)) && i0 < i1 {
                        i1 -= 1;
                    }
                    if i0 >= i1 {
                        break;
                    }
                    // swap
                    self.vprops_.swap(i0 as usize, i1 as usize);
                }
                // remember new size
                nV = if self.vremoved_.get(SM_Vertex_index(i0 as u32)) { i0 } else { i0 + 1 };
            }

            // really remove edges
            if nE > 0 {
                let mut i0 = 0i32;
                let mut i1 = nE - 1;
                loop {
                    // find first removed and last un-removed
                    while !self.eremoved_.get(SM_Edge_index::new(i0 as u32)) && i0 < i1 {
                        i0 += 1;
                    }
                    while self.eremoved_.get(SM_Edge_index::new(i1 as u32)) && i0 < i1 {
                        i1 -= 1;
                    }
                    if i0 >= i1 {
                        break;
                    }
                    // swap
                    self.eprops_.swap(i0 as usize, i1 as usize);
                    self.hprops_.swap(2 * i0 as usize, 2 * i1 as usize);
                    self.hprops_.swap(2 * i0 as usize + 1, 2 * i1 as usize + 1);
                }
                // remember new size
                nE = if self.eremoved_.get(SM_Edge_index::new(i0 as u32)) { i0 } else { i0 + 1 };
                nH = 2 * nE;
            }

            // really remove faces
            if nF > 0 {
                let mut i0 = 0i32;
                let mut i1 = nF - 1;
                loop {
                    // find 1st removed and last un-removed
                    while !self.fremoved_.get(SM_Face_index(i0 as u32)) && i0 < i1 {
                        i0 += 1;
                    }
                    while self.fremoved_.get(SM_Face_index(i1 as u32)) && i0 < i1 {
                        i1 -= 1;
                    }
                    if i0 >= i1 {
                        break;
                    }
                    // swap
                    self.fprops_.swap(i0 as usize, i1 as usize);
                }
                // remember new size
                nF = if self.fremoved_.get(SM_Face_index(i0 as u32)) { i0 } else { i0 + 1 };
            }

            // update vertex connectivity
            for i in 0..nV {
                let v = SM_Vertex_index(i as u32);
                if !self.is_isolated(v) {
                    self.set_halfedge(v, hmap.get(self.halfedge(v)));
                }
            }

            // update halfedge connectivity
            for i in 0..nH {
                let h = SM_Halfedge_index(i as u32);
                self.set_target(h, vmap.get(self.target(h)));
                self.set_next(h, hmap.get(self.next(h)));
                if !self.is_border(h) {
                    self.set_face(h, fmap.get(self.face(h)));
                }
            }

            // update indices of faces
            for i in 0..nF {
                let f = SM_Face_index(i as u32);
                self.set_halfedge(f, hmap.get(self.halfedge(f)));
            }

            // remove index maps
            self.remove_property_map(&mut vmap);
            self.remove_property_map(&mut hmap);
            self.remove_property_map(&mut fmap);

            // finally resize arrays
            self.vprops_.resize(nV as usize);
            self.vprops_.shrink_to_fit();
            self.hprops_.resize(nH as usize);
            self.hprops_.shrink_to_fit();
            self.eprops_.resize(nE as usize);
            self.eprops_.shrink_to_fit();
            self.fprops_.resize(nF as usize);
            self.fprops_.shrink_to_fit();

            self.removed_vertices_ = 0;
            self.removed_edges_ = 0;
            self.removed_faces_ = 0;
            self.vertices_freelist_ = SM_NULL;
            self.edges_freelist_ = SM_NULL;
            self.faces_freelist_ = SM_NULL;
            self.garbage_ = false;
        }

        // ---- Validity Checks ----

        /// Surface_mesh.h:1419-1527 `is_valid(bool verbose)`: an expensive validity check on the
        /// data structure
        pub fn is_valid(&self, verbose: bool) -> bool {
            let mut valid = true;
            let (mut vcount, mut hcount, mut fcount) = (0 as size_type, 0 as size_type, 0 as size_type);
            for it in self.halfedges() {
                hcount += 1;
                valid = valid && self.next(it).is_valid();
                valid = valid && self.opposite(it).is_valid();
                if !valid {
                    if verbose {
                        eprintln!("Integrity of halfedge h{} corrupted.", it.0);
                    }
                    break;
                }

                valid = valid && (self.opposite(it) != it);
                valid = valid && (self.opposite(self.opposite(it)) == it);
                if !valid {
                    if verbose {
                        eprintln!("Integrity of opposite halfedge of h{} corrupted.", it.0);
                    }
                    break;
                }

                valid = valid && (self.next(self.prev(it)) == it);
                if !valid {
                    if verbose {
                        eprintln!("Integrity of previous halfedge of h{} corrupted.", it.0);
                    }
                    break;
                }

                valid = valid && (self.prev(self.next(it)) == it);
                if !valid {
                    if verbose {
                        eprintln!("Integrity of next halfedge of h{} corrupted.", it.0);
                    }
                    break;
                }

                valid = valid && self.target(it).is_valid();
                if !valid {
                    if verbose {
                        eprintln!("Integrity of vertex of halfedge h{} corrupted.", it.0);
                    }
                    break;
                }

                valid = valid && (self.target(it) == self.target(self.opposite(self.next(it))));
                if !valid {
                    if verbose {
                        eprintln!("Halfedge vertex of next opposite is not the same for h{}.", it.0);
                    }
                    break;
                }
            }

            for it in self.vertices() {
                vcount += 1;
                if self.halfedge(it).is_valid() {
                    // not an isolated vertex
                    valid = valid && (self.target(self.halfedge(it)) == it);
                    if !valid {
                        if verbose {
                            eprintln!("Halfedge of v{} is not an incoming halfedge.", it.0);
                        }
                        break;
                    }
                }
            }
            for _it in self.faces() {
                fcount += 1;
            }

            valid = valid && (vcount == self.number_of_vertices());
            if !valid && verbose {
                eprintln!("#vertices: iterated: {} vs number_of_vertices(): {}", vcount, self.number_of_vertices());
            }

            valid = valid && (hcount == self.number_of_halfedges());
            if !valid && verbose {
                eprintln!("#halfedges: iterated: {} vs number_of_halfedges(): {}", hcount, self.number_of_halfedges());
            }

            valid = valid && (fcount == self.number_of_faces());
            if !valid && verbose {
                eprintln!("#faces: iterated: {} vs number_of_faces(): {}", fcount, self.number_of_faces());
            }

            let inf = SM_NULL;
            let mut vfl = self.vertices_freelist_;
            let mut rv: size_type = 0;
            while vfl != inf {
                vfl = self.vconn_.get(SM_Vertex_index(vfl)).halfedge_.0;
                rv += 1;
            }
            valid = valid && (rv == self.removed_vertices_);

            let mut efl = self.edges_freelist_;
            let mut re: size_type = 0;
            while efl != inf {
                efl = self.hconn_.get(SM_Halfedge_index(efl)).next_halfedge_.0;
                re += 1;
            }
            valid = valid && (re == self.removed_edges_);

            let mut ffl = self.faces_freelist_;
            let mut rf: size_type = 0;
            while ffl != inf {
                ffl = self.fconn_.get(SM_Face_index(ffl)).halfedge_.0;
                rf += 1;
            }
            valid = valid && (rf == self.removed_faces_);

            valid
        }

        // ---- Low-Level Connectivity ----

        /// Surface_mesh.h:1608-1611
        pub fn target(&self, h: SM_Halfedge_index) -> SM_Vertex_index {
            self.hconn_.get(h).vertex_
        }
        /// Surface_mesh.h:1614-1617
        pub fn set_target(&self, h: SM_Halfedge_index, v: SM_Vertex_index) {
            let mut c = self.hconn_.get(h);
            c.vertex_ = v;
            self.hconn_.set(h, c);
        }
        /// Surface_mesh.h:1620-1623
        pub fn face(&self, h: SM_Halfedge_index) -> SM_Face_index {
            self.hconn_.get(h).face_
        }
        /// Surface_mesh.h:1626-1629
        pub fn set_face(&self, h: SM_Halfedge_index, f: SM_Face_index) {
            let mut c = self.hconn_.get(h);
            c.face_ = f;
            self.hconn_.set(h, c);
        }
        /// Surface_mesh.h:1632-1635
        pub fn next(&self, h: SM_Halfedge_index) -> SM_Halfedge_index {
            self.hconn_.get(h).next_halfedge_
        }
        /// Surface_mesh.h:1638-1641
        pub fn prev(&self, h: SM_Halfedge_index) -> SM_Halfedge_index {
            self.hconn_.get(h).prev_halfedge_
        }
        /// Surface_mesh.h:1645-1648
        pub fn set_next_only(&self, h: SM_Halfedge_index, nh: SM_Halfedge_index) {
            let mut c = self.hconn_.get(h);
            c.next_halfedge_ = nh;
            self.hconn_.set(h, c);
        }
        /// Surface_mesh.h:1651-1656
        pub fn set_prev_only(&self, h: SM_Halfedge_index, nh: SM_Halfedge_index) {
            if h != Self::null_halfedge() {
                let mut c = self.hconn_.get(h);
                c.prev_halfedge_ = nh;
                self.hconn_.set(h, c);
            }
        }
        /// Surface_mesh.h:1661-1665
        pub fn set_next(&self, h: SM_Halfedge_index, nh: SM_Halfedge_index) {
            self.set_next_only(h, nh);
            self.set_prev_only(nh, h);
        }
        /// Surface_mesh.h:1670-1673, 1683-1686, 1763-1766 `halfedge(v | f | e)`
        pub fn halfedge<K: Has_halfedge>(&self, k: K) -> SM_Halfedge_index {
            k.halfedge_of(self)
        }
        /// Surface_mesh.h:1676-1679, 1689-1692 `set_halfedge(v | f, h)`
        pub fn set_halfedge<K: Has_halfedge>(&self, k: K, h: SM_Halfedge_index) {
            k.set_halfedge_of(self, h)
        }
        /// Surface_mesh.h:1695-1698
        pub fn opposite(&self, h: SM_Halfedge_index) -> SM_Halfedge_index {
            SM_Halfedge_index(if (h.0 & 1) != 0 { h.0.wrapping_sub(1) } else { h.0.wrapping_add(1) })
        }
        /// Surface_mesh.h:1706-1709
        pub fn source(&self, h: SM_Halfedge_index) -> SM_Vertex_index {
            self.target(self.opposite(h))
        }
        /// Surface_mesh.h:1713-1716
        pub fn next_around_target(&self, h: SM_Halfedge_index) -> SM_Halfedge_index {
            self.opposite(self.next(h))
        }
        /// Surface_mesh.h:1720-1723
        pub fn prev_around_target(&self, h: SM_Halfedge_index) -> SM_Halfedge_index {
            self.prev(self.opposite(h))
        }
        /// Surface_mesh.h:2369-2392 `halfedge(Vertex_index source, Vertex_index target)`: the
        /// halfedge between two vertices, null when they are not connected
        pub fn halfedge_between(&self, source: SM_Vertex_index, target: SM_Vertex_index) -> SM_Halfedge_index {
            debug_assert!(self.has_valid_index(source) && self.has_valid_index(target));

            let mut h = self.halfedge(target);
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
        /// Surface_mesh.h:1757-1760
        pub fn edge(&self, h: SM_Halfedge_index) -> SM_Edge_index {
            SM_Edge_index::from_halfedge(h)
        }
        /// Surface_mesh.h:1769-1773 `halfedge(Edge_index e, unsigned int i)`
        pub fn halfedge_of_edge(&self, e: SM_Edge_index, i: u32) -> SM_Halfedge_index {
            debug_assert!(i <= 1);
            SM_Halfedge_index((e.idx() << 1) + i)
        }

        // ---- Borders ----

        /// Surface_mesh.h:1813-1829 `is_border(Vertex_index v, bool check_all_incident_halfedges)`
        pub fn is_border_vertex(&self, v: SM_Vertex_index, check_all_incident_halfedges: bool) -> bool {
            let h = self.halfedge(v);
            if h == Self::null_halfedge() {
                return true;
            }
            if check_all_incident_halfedges {
                let done = h;
                let mut hatc = h;
                loop {
                    if self.is_border(hatc) {
                        return true;
                    }
                    hatc = self.opposite(self.next(hatc));
                    if hatc == done {
                        break;
                    }
                }
                return false;
            }
            self.is_border(h)
        }
        /// Surface_mesh.h:1832-1835 `is_border(Halfedge_index h)`
        pub fn is_border(&self, h: SM_Halfedge_index) -> bool {
            !self.face(h).is_valid()
        }
        /// Surface_mesh.h:1840-1843 `is_border(Edge_index e)`
        pub fn is_border_edge(&self, e: SM_Edge_index) -> bool {
            self.is_border(e.halfedge()) || self.is_border(self.opposite(e.halfedge()))
        }
        /// Surface_mesh.h:1892-1895
        pub fn is_isolated(&self, v: SM_Vertex_index) -> bool {
            !self.halfedge(v).is_valid()
        }

        // ---- Property Handling ----

        /// Surface_mesh.h:1974-1983 `add_property_map<I, T>(name, t)`: the map and whether it was
        /// created (an existing one of this name and type comes back with `false`)
        pub fn add_property_map<I: SM_Key, T: Clone + 'static>(&mut self, name: &str, t: T) -> (Property_map<I, T>, bool) {
            let mut name = name.to_string();
            if name.is_empty() {
                name = format!("anonymous-property-{}", self.anonymous_property_);
                self.anonymous_property_ += 1;
            }
            I::container_mut(self).add::<I, T>(&name, t)
        }
        /// Surface_mesh.h:1989-1993 `property_map<I, T>(name)`
        pub fn property_map<I: SM_Key, T: Clone + 'static>(&self, name: &str) -> (Property_map<I, T>, bool) {
            I::container(self).get::<I, T>(name)
        }
        /// Surface_mesh.h:1998-2002 `remove_property_map(p)`
        pub fn remove_property_map<I: SM_Key, T: Clone + 'static>(&mut self, p: &mut Property_map<I, T>) {
            I::container_mut(self).remove::<I, T>(p);
        }
        /// Surface_mesh.h:2015-2021: the properties the mesh itself keeps stay (three of the
        /// vertices, one of the halfedges, one of the edges, two of the faces)
        pub fn remove_all_property_maps(&mut self) {
            self.vprops_.resize_property_array(3);
            self.fprops_.resize_property_array(2);
            self.eprops_.resize_property_array(1);
            self.hprops_.resize_property_array(1);
        }
        /// Surface_mesh.h:2039-2043 `properties<I>()`: the names of the properties of `I`
        pub fn properties<I: SM_Key>(&self) -> Vec<String> {
            I::container(self).properties()
        }
        /// Surface_mesh.h:2046-2050 `points()`: the property "v:point"
        pub fn points(&self) -> Property_map<SM_Vertex_index, Point_3> {
            self.vpoint_.clone()
        }
        /// Surface_mesh.h:2053-2054 `point(Vertex_index v)`
        pub fn point(&self, v: SM_Vertex_index) -> Point_3 {
            self.vpoint_.get(v)
        }
        /// Surface_mesh.h:2057-2058 `point(Vertex_index v) = p`
        pub fn set_point(&self, v: SM_Vertex_index, p: Point_3) {
            self.vpoint_.set(v, p);
        }

        // ---- Null Elements ----

        /// Surface_mesh.h:2072-2075
        pub fn null_vertex() -> SM_Vertex_index {
            SM_Vertex_index::null()
        }
        /// Surface_mesh.h:2078-2081
        pub fn null_edge() -> SM_Edge_index {
            SM_Edge_index::null()
        }
        /// Surface_mesh.h:2083-2086
        pub fn null_halfedge() -> SM_Halfedge_index {
            SM_Halfedge_index::null()
        }
        /// Surface_mesh.h:2088-2091
        pub fn null_face() -> SM_Face_index {
            SM_Face_index::null()
        }

        /// Surface_mesh.h:2395-2425 `adjust_incoming_halfedge(Vertex_index v)`: make sure that the
        /// incoming halfedge of vertex v is a border halfedge if `v` is a border vertex
        pub fn adjust_incoming_halfedge(&mut self, v: SM_Vertex_index) {
            let mut h = self.halfedge(v);
            let mut hh = h;

            if h.is_valid() {
                if self.target(h) != v {
                    // wrong target, flip
                    h = self.opposite(h);
                    hh = h;
                    self.set_halfedge(v, h);
                }

                loop {
                    if self.is_border(h) {
                        self.set_halfedge(v, h);
                        return;
                    }
                    h = self.next_around_target(h);
                    if h == hh {
                        break;
                    }
                }
            }
        }

        // ---- Ranges (Surface_mesh.h:506-830; Index_iterator skips removed elements when the
        // mesh has garbage) ----

        /// `vertices()`
        pub fn vertices(&self) -> impl Iterator<Item = SM_Vertex_index> + '_ {
            let g = self.has_garbage();
            (0..self.num_vertices()).map(SM_Vertex_index).filter(move |v| !(g && self.is_removed(*v)))
        }
        /// `halfedges()`
        pub fn halfedges(&self) -> impl Iterator<Item = SM_Halfedge_index> + '_ {
            let g = self.has_garbage();
            (0..self.num_halfedges()).map(SM_Halfedge_index).filter(move |h| !(g && self.is_removed(*h)))
        }
        /// `edges()`
        pub fn edges(&self) -> impl Iterator<Item = SM_Edge_index> + '_ {
            let g = self.has_garbage();
            (0..self.num_edges()).map(SM_Edge_index::new).filter(move |e| !(g && self.is_removed(*e)))
        }
        /// `faces()`
        pub fn faces(&self) -> impl Iterator<Item = SM_Face_index> + '_ {
            let g = self.has_garbage();
            (0..self.num_faces()).map(SM_Face_index).filter(move |f| !(g && self.is_removed(*f)))
        }

        /// CGAL/boost/graph/iterator.h:276-360 `halfedges_around_target(h, g)`: `h`, then
        /// `opposite(next(h))` and so on until `h` comes back. A null `h` gives an empty range.
        pub fn halfedges_around_target(&self, h: SM_Halfedge_index) -> Vec<SM_Halfedge_index> {
            let mut out = Vec::new();
            if !h.is_valid() {
                return out;
            }
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
        /// CGAL/boost/graph/iterator.h:366-440 `halfedges_around_face(h, g)`: `h`, then `next(h)`
        /// and so on until `h` comes back
        pub fn halfedges_around_face(&self, h: SM_Halfedge_index) -> Vec<SM_Halfedge_index> {
            let mut out = Vec::new();
            if !h.is_valid() {
                return out;
            }
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
        /// CGAL/boost/graph/iterator.h:1007-1050, 1279 `vertices_around_face(h, g)`: the targets of
        /// the halfedges around the face
        pub fn vertices_around_face(&self, h: SM_Halfedge_index) -> Vec<SM_Vertex_index> {
            self.halfedges_around_face(h).into_iter().map(|x| self.target(x)).collect()
        }
        /// CGAL/boost/graph/iterator.h:844-868, 943 `faces_around_face(h, g)`: the faces of the
        /// opposites of the halfedges around the face; the null face on a border
        pub fn faces_around_face(&self, h: SM_Halfedge_index) -> Vec<SM_Face_index> {
            self.halfedges_around_face(h).into_iter().map(|x| self.face(self.opposite(x))).collect()
        }
        /// CGAL/boost/graph/iterator.h:896-923, 931 `faces_around_target(h, g)`: the faces of the
        /// halfedges around the target of `h`; the null face on a border
        pub fn faces_around_target(&self, h: SM_Halfedge_index) -> Vec<SM_Face_index> {
            self.halfedges_around_target(h).into_iter().map(|x| self.face(x)).collect()
        }

        // ---- The state as plain arrays ----

        /// The state of the mesh as plain arrays (what `set_raw` takes).
        pub fn raw(&self) -> Raw_surface_mesh {
            Raw_surface_mesh {
                vconn: self.vconn_.to_vec().iter().map(|c| c.halfedge_.0).collect(),
                hconn: self.hconn_.to_vec().iter().map(|c| [c.face_.0, c.vertex_.0, c.next_halfedge_.0, c.prev_halfedge_.0]).collect(),
                fconn: self.fconn_.to_vec().iter().map(|c| c.halfedge_.0).collect(),
                points: self.vpoint_.to_vec(),
                vremoved: self.vremoved_.to_vec(),
                eremoved: self.eremoved_.to_vec(),
                fremoved: self.fremoved_.to_vec(),
                freelists: [self.vertices_freelist_, self.edges_freelist_, self.faces_freelist_],
                garbage: self.garbage_,
            }
        }

        /// Sets the connectivity, the points, the removal flags and the free lists of the mesh
        /// from plain arrays. Not a function of CGAL: it is how an implementation of
        /// `PolygonMeshProcessing` that computes outside this file (CGAL behind an FFI, a replay
        /// of recorded results) hands a refined mesh back. Every property array is resized to the
        /// new number of elements as `Property_container::resize` does: the values of the indices
        /// that stay are kept, new indices get the default of the property.
        pub fn set_raw(&mut self, raw: &Raw_surface_mesh) {
            assert!(raw.hconn.len() == 2 * raw.eremoved.len());
            assert!(raw.vconn.len() == raw.points.len() && raw.vconn.len() == raw.vremoved.len());
            assert!(raw.fconn.len() == raw.fremoved.len());
            self.vprops_.resize(raw.vconn.len());
            self.hprops_.resize(raw.hconn.len());
            self.eprops_.resize(raw.eremoved.len());
            self.fprops_.resize(raw.fconn.len());
            self.vprops_.capacity_ = self.vprops_.capacity_.max(self.vprops_.size_);
            self.hprops_.capacity_ = self.hprops_.capacity_.max(self.hprops_.size_);
            self.eprops_.capacity_ = self.eprops_.capacity_.max(self.eprops_.size_);
            self.fprops_.capacity_ = self.fprops_.capacity_.max(self.fprops_.size_);
            for (i, h) in raw.vconn.iter().enumerate() {
                self.vconn_.set_at(i, Vertex_connectivity { halfedge_: SM_Halfedge_index(*h) });
                self.vpoint_.set_at(i, raw.points[i]);
                self.vremoved_.set_at(i, raw.vremoved[i]);
            }
            for (i, c) in raw.hconn.iter().enumerate() {
                self.hconn_.set_at(
                    i,
                    Halfedge_connectivity {
                        face_: SM_Face_index(c[0]),
                        vertex_: SM_Vertex_index(c[1]),
                        next_halfedge_: SM_Halfedge_index(c[2]),
                        prev_halfedge_: SM_Halfedge_index(c[3]),
                    },
                );
            }
            for (i, r) in raw.eremoved.iter().enumerate() {
                self.eremoved_.set_at(i, *r);
            }
            for (i, h) in raw.fconn.iter().enumerate() {
                self.fconn_.set_at(i, Face_connectivity { halfedge_: SM_Halfedge_index(*h) });
                self.fremoved_.set_at(i, raw.fremoved[i]);
            }
            self.removed_vertices_ = raw.vremoved.iter().filter(|r| **r).count() as size_type;
            self.removed_edges_ = raw.eremoved.iter().filter(|r| **r).count() as size_type;
            self.removed_faces_ = raw.fremoved.iter().filter(|r| **r).count() as size_type;
            self.vertices_freelist_ = raw.freelists[0];
            self.edges_freelist_ = raw.freelists[1];
            self.faces_freelist_ = raw.freelists[2];
            self.garbage_ = raw.garbage;
        }
    }
}

// ===========================================================================
// namespace priv (CutSurface.cpp:40-518): types and names
// ===========================================================================

// CutSurface.cpp:42-43
// using Project   = Emboss::IProjection;
// using Project3d = Emboss::IProject3d;

/// CutSurface.cpp:73  `using CutMesh = CGAL::Surface_mesh<EpicKernel::Point_3>;`
pub type CutMesh = surface_mesh::Surface_mesh;
/// CutSurface.cpp:74  `using CutMeshes = std::vector<CutMesh>;`
pub type CutMeshes = Vec<CutMesh>;

/// CutSurface.cpp:76  `using VI = CGAL::SM_Vertex_index;`
pub type VI = surface_mesh::SM_Vertex_index;
/// CutSurface.cpp:77  `using HI = CGAL::SM_Halfedge_index;`
pub type HI = surface_mesh::SM_Halfedge_index;
/// CutSurface.cpp:78  `using EI = CGAL::SM_Edge_index;`
pub type EI = surface_mesh::SM_Edge_index;
/// CutSurface.cpp:79  `using FI = CGAL::SM_Face_index;`
pub type FI = surface_mesh::SM_Face_index;
/// CutSurface.cpp:80  `using P3 = CGAL::Epick::Point_3;`
pub type P3 = surface_mesh::Point_3;

/// CutSurface.cpp:82  `inline Vec3d to_vec3d(const P3 &p)`
pub fn to_vec3d(p: &P3) -> Vec3d {
    Vec3d::new(p[0], p[1], p[2])
}

/// CutSurface.cpp:162-168 `IntersectingElement::Type`: type of intersecting element from extruded
/// shape (3d). The type is read as `attr % 8`, which can be a value with no name (5, 6, 7), so
/// it is the number here and the names are constants.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Type(pub u8);

impl Type {
    pub const edge_1: Type = Type(0);
    pub const face_1: Type = Type(1);
    pub const edge_2: Type = Type(2);
    pub const face_2: Type = Type(3);
    pub const undefined: Type = Type(4);
}

/// IntersectingElement
///
/// Adress polygon inside of ExPolygon
/// Keep information about source of vertex:
///     - from face (one of 2 possible)
///     - from edge (one of 2 possible)
///
/// ```text
///  V1~~~~~V2
///   | f1 /:
///   |   / :
/// e1|  /e2:
///   | /   :
///   |/ f2 :
///  V1'~~~~V2'
/// ```
///
/// | .. edge
/// / .. edge
/// : .. foreign edge - neighbor
/// ~ .. no care edge - idealy should not cross model
/// V1,V1' .. projected 2d point to 3d
/// V2,V2' .. projected 2d point to 3d
///
/// Vertex indexing
/// V1  .. i (vertex_base + 2x index of point in polygon)
/// V1' .. i + 1
/// V2  .. j = i + 2 || 0 (for last i in polygon)
/// V2' .. j + 1
///
/// f1 .. text_face_1 (triangle face made by side of shape contour)
/// f2 .. text_face_2
/// e1 .. text_edge_1 (edge on side of face made by side of shape contour)
/// e2 .. text_edge_2
///
/// CutSurface.cpp:146-181
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IntersectingElement {
    // identify source point in shapes
    pub shape_point_index: u32,
    // store together type, is_first, is_last
    pub attr: u8,
}

impl Default for IntersectingElement {
    // CutSurface.cpp:149, 152
    fn default() -> Self {
        IntersectingElement { shape_point_index: u32::MAX, attr: u8::MAX }
    }
}

impl IntersectingElement {
    // CutSurface.cpp:170-175
    pub fn set_type(&mut self, t: Type) -> &mut Self {
        self.attr = (self.attr as i32 + t.0 as i32 - self.get_type().0 as i32) as u8;
        self
    }
    // CutSurface.cpp:176
    pub fn set_is_first(&mut self) {
        self.attr = self.attr.wrapping_add(8);
    }
    // CutSurface.cpp:177
    pub fn set_is_last(&mut self) {
        self.attr = self.attr.wrapping_add(16);
    }
    // CutSurface.cpp:178
    pub fn get_type(&self) -> Type {
        Type(self.attr % 8)
    }
    // CutSurface.cpp:179
    pub fn is_first(&self) -> bool {
        8 <= self.attr && self.attr < 16
    }
    // CutSurface.cpp:180
    pub fn is_last(&self) -> bool {
        self.attr >= 16
    }
}

// stored in model made by shape
/// CutSurface.cpp:184
pub type EdgeShapeMap = surface_mesh::Property_map<EI, IntersectingElement>;
/// CutSurface.cpp:185
pub type FaceShapeMap = surface_mesh::Property_map<FI, IntersectingElement>;

// stored in surface source - pointer to EdgeShapeMap | FaceShapeMap
/// CutSurface.cpp:188 `CutMesh::Property_map<VI, const IntersectingElement *>` (the value pointed
/// to, see the head of the file)
pub type VertexShapeMap = surface_mesh::Property_map<VI, Option<IntersectingElement>>;

// stored in model made by shape
/// CutSurface.cpp:191
pub const edge_shape_map_name: &str = "e:IntersectingElement";
/// CutSurface.cpp:192
pub const face_shape_map_name: &str = "f:IntersectingElement";

// stored in surface source
/// CutSurface.cpp:195
pub const vert_shape_map_name: &str = "v:IntersectingElement";

/// Flag for faces in CGAL mesh
/// CutSurface.cpp:200-210
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaceType {
    // face inside of the cutted shape
    inside,
    // face outside of the cutted shape
    outside,
    // face without constrained edge (In or Out)
    not_constrained,

    // Helper flag that inside was processed
    inside_processed,
}
/// CutSurface.cpp:211
pub type FaceTypeMap = surface_mesh::Property_map<FI, FaceType>;
/// CutSurface.cpp:212
pub const face_type_map_name: &str = "f:side";

// Conversion one vertex index to another
/// CutSurface.cpp:215
pub type CvtVI2VI = surface_mesh::Property_map<VI, VI>;
// Each Patch track outline vertex conversion to tource model
/// CutSurface.cpp:217
pub const patch_source_name: &str = "v:patch_source";

// For VI that should be reduced, contain VI to use instead of reduced
// Other VI are invalid
/// CutSurface.cpp:221
pub type ReductionMap = CvtVI2VI;
/// CutSurface.cpp:222
pub const vertex_reduction_map_name: &str = "v:reduction";

// A property map containing the constrained-or-not status of each edge
/// CutSurface.cpp:225
pub type EdgeBoolMap = surface_mesh::Property_map<EI, bool>;
/// CutSurface.cpp:226
pub const is_constrained_edge_name: &str = "e:is_constrained";

// Patch made by Cut area of interest from model
// connected faces(triangles) and outlines(halfEdges) for one surface cut
/// CutSurface.cpp:244
pub type CutAOI = (Vec<FI>, Vec<HI>);
// vector of Cutted Area of interest cutted from one CGAL model
/// CutSurface.cpp:246
pub type CutAOIs = Vec<CutAOI>;
// vector of CutAOIs for each model
/// CutSurface.cpp:248
pub type VCutAOIs = Vec<CutAOIs>;

/// CutSurface.cpp:268
pub type Loop = Vec<VI>;
/// CutSurface.cpp:269
pub type Loops = Vec<Loop>;

/// `BoundingBoxf3` (BoundingBox.hpp:104-162, `BoundingBox3Base<Vec3d>`), what CutSurface.cpp uses
/// of it: the constructor from two corners and `intersects`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoundingBoxf3 {
    pub min: Vec3d,
    pub max: Vec3d,
    pub defined: bool,
}

impl Default for BoundingBoxf3 {
    // BoundingBox.hpp:20, 109
    fn default() -> Self {
        BoundingBoxf3 { min: Vec3d::zero(), max: Vec3d::zero(), defined: false }
    }
}

impl BoundingBoxf3 {
    // BoundingBox.hpp:21-22, 110-112
    pub fn new(pmin: Vec3d, pmax: Vec3d) -> Self {
        let mut defined = pmin.x < pmax.x && pmin.y < pmax.y;
        if pmin.z >= pmax.z {
            defined = false;
        }
        BoundingBoxf3 { min: pmin, max: pmax, defined }
    }
    // BoundingBox.hpp:158-160
    pub fn intersects(&self, other: &BoundingBoxf3) -> bool {
        (self.min.x < other.max.x)
            && (self.max.x > other.min.x)
            && (self.min.y < other.max.y)
            && (self.max.y > other.min.y)
            && (self.min.z < other.max.z)
            && (self.max.z > other.min.z)
    }
}

// To track during diff_models,
// what was cutted off, from CutAOI
/// CutSurface.cpp:281-307
#[derive(Clone)]
pub struct SurfacePatch {
    // converted cut to CGAL mesh
    // Mesh is reduced.
    // (do not contain divided triangles on contour - created by side Quad)
    pub mesh: CutMesh,
    // CvtVI2VI cvt = mesh.property_map<VI, VI>(patch_source_name);
    // Conversion VI from this patch to source VI(model) is stored in mesh property

    // Outlines - converted CutAOI.second (half edges)
    // to loops (vertex indicies) by function create_loops
    pub loops: Loops,

    // bounding box of mesh
    pub bb: BoundingBoxf3,

    //// Data needed to find best projection distances
    // index of source model in models
    pub model_id: usize,
    // index of source CutAOI
    pub aoi_id: usize,
    // index of shape from ExPolygons
    pub shape_id: usize,

    // flag that this patch contain whole CutAOI
    pub is_whole_aoi: bool,
}

impl Default for SurfacePatch {
    fn default() -> Self {
        SurfacePatch { mesh: CutMesh::new(), loops: Loops::new(), bb: BoundingBoxf3::default(), model_id: 0, aoi_id: 0, shape_id: 0, is_whole_aoi: true }
    }
}
/// CutSurface.cpp:308
pub type SurfacePatches = Vec<SurfacePatch>;

/// CutSurface.cpp:310-316
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ModelCutId {
    // index of model
    pub model_index: u32,
    // index of cut inside model
    pub cut_index: u32,
}

/// Keep conversion from VCutAOIs to Index and vice versa
/// Model_index .. contour(or hole) poin from ExPolygons
/// Index       .. continous number
/// CutSurface.cpp:323-335
#[derive(Debug, Clone)]
pub struct ModelCut2index {
    m_offsets: Vec<u32>,
    // for check range of index
    m_count: u32,
}

/// To select surface near projection distance
/// CutSurface.cpp:395-408
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ProjectionDistance {
    // index of source model
    pub model_index: u32,

    // index of CutAOI
    pub aoi_index: u32,

    // index of Patch
    pub patch_index: u32,

    // signed distance to projection
    pub distance: f32,
}

impl Default for ProjectionDistance {
    fn default() -> Self {
        ProjectionDistance { model_index: u32::MAX, aoi_index: u32::MAX, patch_index: u32::MAX, distance: f32::MAX }
    }
}
// addresed by ExPolygonsIndices
/// CutSurface.cpp:410
pub type ProjectionDistances = Vec<ProjectionDistance>;

// each point in shapes has its ProjectionDistances
/// CutSurface.cpp:413
pub type VDistances = Vec<ProjectionDistances>;

// ===========================================================================
// CGAL::Polygon_mesh_processing behind a trait
// ===========================================================================

/// `CGAL::Polygon_mesh_processing::Corefinement::Default_visitor<CutMesh>`
/// (CGAL/Polygon_mesh_processing/corefinement.h, concept `PMPCorefinementVisitor`): the callbacks
/// the visitors of CutSurface.cpp define; like the default visitor of CGAL, each does nothing
/// unless a visitor overrides it.
pub trait CorefinementVisitor {
    /// Called when a new intersection point is detected.
    /// The intersection is detected using a face of tm_f and an edge of tm_e.
    fn intersection_point_detected(
        &mut self,
        _i_id: usize,
        _sdim: i32,
        _h_f: HI,
        _h_e: HI,
        _tm_f: &CutMesh,
        _tm_e: &CutMesh,
        _is_target_coplanar: bool,
        _is_source_coplanar: bool,
    ) {
    }
    /// Called when a new vertex is added in tm (either an edge split or a vertex inserted in the
    /// interior of a face).
    fn new_vertex_added(&mut self, _i_id: usize, _v: VI, _tm: &CutMesh) {}
    fn before_subface_creations(&mut self, _f_old: FI, _tm: &CutMesh) {}
    fn after_subface_created(&mut self, _f_new: FI, _tm: &CutMesh) {}
    fn after_subface_creations(&mut self, _tm: &CutMesh) {}
    fn before_subface_created(&mut self, _tm: &CutMesh) {}
    fn before_edge_split(&mut self, _h: HI, _tm: &CutMesh) {}
    fn edge_split(&mut self, _hnew: HI, _tm: &CutMesh) {}
    fn after_edge_split(&mut self) {}
    fn add_retriangulation_edge(&mut self, _h: HI, _tm: &CutMesh) {}
}

/// The named parameters CutSurface.cpp passes (`CGAL::parameters::visitor(v)
/// .edge_is_constrained_map(ecm).throw_on_self_intersection(false)`,
/// `CGAL::parameters::do_not_modify(true)`); `None` is a parameter not given.
#[derive(Default)]
pub struct NamedParameters<'a> {
    pub visitor: Option<&'a mut dyn CorefinementVisitor>,
    pub edge_is_constrained_map: Option<EdgeBoolMap>,
    pub throw_on_self_intersection: Option<bool>,
    pub do_not_modify: Option<bool>,
}

impl<'a> NamedParameters<'a> {
    /// `CGAL::parameters::default_values()`
    pub fn default_values() -> Self {
        NamedParameters::default()
    }
    /// `.visitor(v)`
    pub fn visitor(mut self, v: &'a mut dyn CorefinementVisitor) -> Self {
        self.visitor = Some(v);
        self
    }
    /// `.edge_is_constrained_map(ecm)`
    pub fn edge_is_constrained_map(mut self, ecm: EdgeBoolMap) -> Self {
        self.edge_is_constrained_map = Some(ecm);
        self
    }
    /// `.throw_on_self_intersection(b)`
    pub fn throw_on_self_intersection(mut self, b: bool) -> Self {
        self.throw_on_self_intersection = Some(b);
        self
    }
    /// `.do_not_modify(b)`
    pub fn do_not_modify(mut self, b: bool) -> Self {
        self.do_not_modify = Some(b);
        self
    }
}

/// The algorithms of CGAL CutSurface.cpp calls that are not written out in this file. The meshes
/// handed back keep CGAL's numbering of vertices, halfedges and faces: the functions of this file
/// walk them in index order. An implementation that computes outside this file sets a refined
/// mesh with `CutMesh::set_raw`.
pub trait PolygonMeshProcessing {
    /// `CGAL::Polygon_mesh_processing::corefine(tm1, tm2, np1, np2)`: refines the two meshes
    /// along their intersection (`tm2` stays as it is under `do_not_modify(true)`), reports each
    /// intersection point and each new vertex to the visitor of `np1` and marks the edges on the
    /// intersection in the `edge_is_constrained_map` of `np1`.
    fn corefine(&self, tm1: &mut CutMesh, tm2: &mut CutMesh, np1: NamedParameters<'_>, np2: NamedParameters<'_>);

    /// `CGAL::Polygon_mesh_processing::clip(tm, clipper, np_tm, np_c)`: clips `tm` by the volume
    /// bounded by `clipper`; whether the output surface mesh is manifold (when it is not, `tm`
    /// and `clipper` are only corefined). An `Err` is the exception CGAL throws, with its text.
    fn clip(&self, tm: &mut CutMesh, clipper: &mut CutMesh, np_tm: NamedParameters<'_>, np_c: NamedParameters<'_>) -> Result<bool, String>;

    /// `CGAL::IO::read_OFF(path, mesh)`
    fn read_off(&self, path: &str, mesh: &mut CutMesh) -> bool;
}

/// `EpicKernel::Ray_3` built from two points (CutSurface.cpp:2768): the ray from `source` through
/// `second_point`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ray {
    pub source: P3,
    pub second_point: P3,
}

impl Ray {
    pub fn new(source: P3, second_point: P3) -> Self {
        Ray { source, second_point }
    }
}

/// CutSurface.cpp:2766-2769 `CGAL::AABB_tree<Traits>` over the faces of a `CutMesh`
/// (`AABB_face_graph_triangle_primitive`), what CutSurface.cpp uses of it: `insert`, `build`,
/// `empty` and `number_of_intersected_primitives` of a ray.
///
/// The count is the number of triangles the ray intersects by the exact predicate of the kernel
/// (`Do_intersect_3(Triangle_3, Ray_3)`). CGAL reaches the same triangles through its tree: the
/// boxes of the nodes are tested with exact predicates too and contain their triangles, so the
/// traversal skips none that intersects. The primitives are tested here one by one.
#[derive(Debug, Clone, Default)]
pub struct Tree {
    m_primitives: Vec<[P3; 3]>,
}
/// CutSurface.cpp:2770
pub type Trees = Vec<Tree>;

impl Tree {
    pub fn new() -> Self {
        Tree::default()
    }
    /// `tree.insert(faces(model).first, faces(model).second, model)`: one primitive per face; the
    /// triangle of a face is `target(h)`, `target(next(h))`, `source(h)` of its halfedge `h`
    /// (CGAL/boost/graph/property_maps.h `Triangle_from_face_descriptor_map`).
    pub fn insert(&mut self, faces: impl Iterator<Item = FI>, model: &CutMesh) {
        for f in faces {
            let h = model.halfedge(f);
            self.m_primitives.push([model.point(model.target(h)), model.point(model.target(model.next(h))), model.point(model.source(h))]);
        }
    }
    /// `tree.build()`
    pub fn build(&mut self) {}
    /// `tree.empty()`
    pub fn empty(&self) -> bool {
        self.m_primitives.is_empty()
    }
    /// `tree.number_of_intersected_primitives(ray)`
    pub fn number_of_intersected_primitives(&self, ray: &Ray) -> usize {
        self.m_primitives.iter().filter(|t| do_intersect_triangle_3_ray_3(t, &ray.source, &ray.second_point)).count()
    }
}

// Eigen's `Vec3d` algebra in Eigen's evaluation order (`geometry::eigen`)
fn ev(v: &Vec3d) -> eigen::Vec3d {
    eigen::Vec3d::new(v.x, v.y, v.z)
}
fn gv(v: &eigen::Vec3d) -> Vec3d {
    Vec3d::new(v.x, v.y, v.z)
}
/// `a.dot(b)`
fn dot3(a: &Vec3d, b: &Vec3d) -> f64 {
    eigen::dot3(&ev(a), &ev(b))
}
/// `a.cross(b)`
fn cross3(a: &Vec3d, b: &Vec3d) -> Vec3d {
    gv(&eigen::cross3(&ev(a), &ev(b)))
}
/// `v.normalize()`
fn normalize(v: &mut Vec3d) {
    *v = gv(&eigen::normalized3(&ev(v)));
}

// ===========================================================================
// CutSurface.cpp:525-681  cut_surface, cut2model
// ===========================================================================

/// Cut surface shape from models.
///
/// CutSurface.hpp:37-40 / CutSurface.cpp:525-617
///
/// `pmp` is CGAL's corefinement and clip (see `PolygonMeshProcessing`).
pub fn cut_surface(
    shapes: &ExPolygons,
    models: &[indexed_triangle_set],
    projection: &dyn IProjection,
    projection_ratio: f32,
    pmp: &dyn PolygonMeshProcessing,
) -> SurfaceCut {
    debug_assert!(!models.is_empty());
    debug_assert!(!shapes.is_empty());
    if models.is_empty() || shapes.is_empty() {
        return SurfaceCut::default();
    }

    // for filter out triangles out of bounding box
    let shapes_bb: BoundingBox = crate::geometry::get_extents(shapes);

    // for filttrate opposite triangles and a little more
    let max_angle: f32 = 89.9;
    let mut cgal_models = CutMeshes::new(); // source for patch
    let mut cgal_neg_models = CutMeshes::new(); // model used for differenciate patches
    cgal_models.reserve(models.len());
    for its in models {
        let mut skip_indicies: Vec<bool> = vec![false; its.indices.len()];
        set_skip_for_out_of_aoi(&mut skip_indicies, its, projection, &shapes_bb);

        // create model for differenciate cutted patches
        let flip = true;
        cgal_neg_models.push(to_cgal(its, &skip_indicies, flip));

        // cut out more than only opposit triangles
        set_skip_by_angle(&mut skip_indicies, its, projection, max_angle as f64);
        cgal_models.push(to_cgal(its, &skip_indicies, false));
    }

    let mut cgal_shape: CutMesh = of_shapes::to_cgal(shapes, projection);

    // create tool for convert index to shape Point adress and vice versa
    let s2i = ExPolygonsIndices::new(shapes);
    let mut model_cuts = VCutAOIs::new();
    // cut shape from each cgal model
    for cgal_model in cgal_models.iter_mut() {
        let cutAOIs: CutAOIs = cut_from_model(cgal_model, shapes, &mut cgal_shape, projection_ratio, &s2i, pmp);
        model_cuts.push(cutAOIs);
    }

    let mut patches: SurfacePatches = diff_models(&mut model_cuts, &mut cgal_models, &mut cgal_neg_models, projection, pmp);
    if patches.is_empty() {
        return SurfaceCut::default();
    }

    // fix - convert shape_point_id to expolygon index
    // save 1 param(s2i) from diff_models call
    for patch in patches.iter_mut() {
        patch.shape_id = s2i.cvt_index(patch.shape_id as u32).expolygons_index as usize;
    }

    // calc distance to projection for all outline points of cutAOI(shape)
    // it is used for distiguish the top one
    let shapes_points: u32 = s2i.get_count();
    // for each point collect all projection distances
    let distances: VDistances = calc_distances(&patches, &cgal_models, &cgal_shape, shapes_points as usize, projection_ratio);

    let start: Point = shapes_bb.center(); // only align center

    // Use only outline points
    // for each point select best projection
    let best_projection: ProjectionDistances = choose_best_distance(&distances, shapes, &start, &s2i, &patches);
    let use_patch: Vec<bool> = select_patches(&best_projection, &patches, shapes, &shapes_bb, &s2i, &model_cuts, &cgal_models, projection);
    merge_patches(&mut patches, &use_patch)
}

/// Create model from surface cuts by projection
/// CutSurface.hpp:48-49 / CutSurface.cpp:619-681
pub fn cut2model(cut: &SurfaceCut, projection: &dyn IProject3d) -> indexed_triangle_set {
    debug_assert!(!cut.empty());
    let count_vertices = cut.its.vertices.len() * 2;
    let mut count_indices = cut.its.indices.len() * 2;

    // indices from from zig zag
    for c in &cut.contours {
        debug_assert!(!c.is_empty());
        count_indices += c.len() * 2;
    }

    let mut result = indexed_triangle_set::default();
    result.vertices.reserve(count_vertices);
    result.indices.reserve(count_indices);

    // front
    result.vertices.extend_from_slice(&cut.its.vertices);
    result.indices.extend_from_slice(&cut.its.indices);

    // back
    for v in &cut.its.vertices {
        let vd = Vec3d::new(v.x as f64, v.y as f64, v.z as f64);
        let vd2 = projection.project(&vd);
        result.vertices.push(Vec3f::new(vd2.x as f32, vd2.y as f32, vd2.z as f32));
    }

    let back_offset = cut.its.vertices.len();
    for i in &cut.its.indices {
        // check range of indices in cut
        debug_assert!((i.x as usize) + back_offset < result.vertices.len());
        debug_assert!((i.y as usize) + back_offset < result.vertices.len());
        debug_assert!((i.z as usize) + back_offset < result.vertices.len());
        debug_assert!(i.x >= 0 && (i.x as usize) < cut.its.vertices.len());
        debug_assert!(i.y >= 0 && (i.y as usize) < cut.its.vertices.len());
        debug_assert!(i.z >= 0 && (i.z as usize) < cut.its.vertices.len());
        // Y and Z is swapped CCW triangles for back side
        result.indices.push(Vec3f32i::new(i.x + back_offset as i32, i.z + back_offset as i32, i.y + back_offset as i32));
    }

    // zig zag indices
    for contour in &cut.contours {
        let mut prev_front_index = *contour.last().unwrap() as usize;
        let mut prev_back_index = back_offset + prev_front_index;
        for &front_index_u in contour {
            let front_index = front_index_u as usize;
            debug_assert!(front_index < cut.its.vertices.len());
            let back_index = back_offset + front_index;
            result.indices.push(Vec3f32i::new(front_index as i32, prev_front_index as i32, back_index as i32));
            result.indices.push(Vec3f32i::new(prev_front_index as i32, prev_back_index as i32, back_index as i32));
            prev_front_index = front_index;
            prev_back_index = back_index;
        }
    }

    debug_assert!(count_vertices == result.vertices.len());
    debug_assert!(count_indices == result.indices.len());
    result
}

// Triangle vertex index type used by `result.indices.emplace_back(...)`.
// In C++ this is `stl_triangle_vertex_indices` (`Vec3i`).
use crate::normal_utils::StlTriangleVertexIndices as Vec3f32i;

// ===========================================================================
// set_skip_for_out_of_aoi helping functions
// CutSurface.cpp:683-707  namespace priv
// ===========================================================================

// CutSurface.cpp:686  using PointNormal  = std::pair<Vec3d, Vec3d>;
pub type PointNormal = (Vec3d, Vec3d);
// CutSurface.cpp:687  using PointNormals = std::array<PointNormal, 4>;
pub type PointNormals = [PointNormal; 4];

// CutSurface.cpp:698  using IsOnSides = std::vector<std::array<bool, 4>>;
pub type IsOnSides = Vec<[bool; 4]>;

/// Check
/// CutSurface.cpp:696 / CutSurface.cpp:709-715
pub fn is_out_of(v: &Vec3d, point_normal: &PointNormal) -> bool {
    let p = &point_normal.0;
    let n = &point_normal.1;
    let signed_distance = dot3(&(*v - *p), n);
    signed_distance > 1e-5
}

/// Check if triangle t has all vertices out of any plane
/// CutSurface.cpp:705 / CutSurface.cpp:717-729
pub fn is_all_on_one_side(t: &Vec3f32i, is_on_sides: &IsOnSides) -> bool {
    for side in 0..4 {
        let mut result = true;
        for k in 0..3 {
            let vi = t[k] as usize;
            if !is_on_sides[vi][side] {
                result = false;
                break;
            }
        }
        if result {
            return true;
        }
    }
    false
}

/// Set true for indices out of area of interest
/// CutSurface.cpp:51-54 / CutSurface.cpp:731-819
pub fn set_skip_for_out_of_aoi(skip_indicies: &mut [bool], its: &indexed_triangle_set, projection: &dyn IProjection, shapes_bb: &BoundingBox) {
    debug_assert!(skip_indicies.len() == its.indices.len());
    //   1`*----* 2`
    //    /  2 /|
    // 1 *----* |
    //   |    | * 3`
    //   |    |/
    // 0 *----* 3
    //////////////////
    let zero = (Vec3d::zero(), Vec3d::zero());
    let mut bb: [PointNormal; 4] = [zero; 4];
    let corners: [Point; 4] = [
        shapes_bb.min,
        Point::new(shapes_bb.min.x, shapes_bb.max.y),
        shapes_bb.max,
        Point::new(shapes_bb.max.x, shapes_bb.min.y),
    ];
    let mut index = 0usize;
    for v in corners.iter() {
        bb[index] = projection.create_front_back(v);
        index += 1;
    }

    // define planes to test
    // 0 .. under
    // 1 .. left
    // 2 .. above
    // 3 .. right
    let mut prev_i = 3usize;
    // plane is defined by point and normal
    let mut point_normals: PointNormals = [zero; 4];
    for i in 0..4usize {
        let p1 = bb[i].0;
        let p2 = bb[i].1;
        let p3 = bb[prev_i].0;
        prev_i = i;

        let mut v1 = p2 - p1;
        normalize(&mut v1);
        let mut v2 = p3 - p1;
        normalize(&mut v2);

        let mut normal = cross3(&v2, &v1);
        normalize(&mut normal);

        point_normals[i] = (p1, normal);
    }

    // check that projection is not left handed
    // Fix for reflected projection
    if is_out_of(&point_normals[2].0, &point_normals[0]) {
        // projection is reflected so normals are reflected
        for pn in point_normals.iter_mut() {
            pn.1 = pn.1 * -1.0;
        }
    }

    // same meaning as point normal
    let mut is_on_sides: IsOnSides = vec![[false, false, false, false]; its.vertices.len()];

    // inspect all vertices when it is out of bounding box
    // (tbb::parallel_for in C++; each index is independent)
    for i in 0..its.vertices.len() {
        let vv = &its.vertices[i];
        let v = Vec3d::new(vv.x as f64, vv.y as f64, vv.z as f64);
        // under + above
        for side in [0usize, 2usize] {
            if is_out_of(&v, &point_normals[side]) {
                is_on_sides[i][side] = true;
                // when it is under it can't be above
                break;
            }
        }
        // left + right
        for side in [1usize, 3usize] {
            if is_out_of(&v, &point_normals[side]) {
                is_on_sides[i][side] = true;
                // when it is on left side it can't be on right
                break;
            }
        }
    }

    // inspect all triangles, when it is out of bounding box
    for i in 0..its.indices.len() {
        if is_all_on_one_side(&its.indices[i], &is_on_sides) {
            skip_indicies[i] = true;
        }
    }
}

/// Separate triangles by mask
/// CutSurface.hpp:69 / CutSurface.cpp:821-859
pub fn its_mask(its: &indexed_triangle_set, mask: &[bool]) -> indexed_triangle_set {
    if its.indices.len() != mask.len() {
        debug_assert!(false);
        return indexed_triangle_set::default();
    }

    let mut cvt_vetices: Vec<u32> = vec![u32::MAX; its.vertices.len()];
    let mut vertices_count: usize = 0;
    let mut faces_count: usize = 0;
    for (index, t) in its.indices.iter().enumerate() {
        if !mask[index] {
            continue;
        }
        faces_count += 1;
        for k in 0..3 {
            let vi = t[k] as usize;
            if cvt_vetices[vi] == u32::MAX {
                cvt_vetices[vi] = vertices_count as u32;
                vertices_count += 1;
            }
        }
    }
    if faces_count == 0 {
        return indexed_triangle_set::default();
    }

    let mut result = indexed_triangle_set::default();
    result.indices.reserve(faces_count);
    result.vertices = vec![Vec3f::new(0.0, 0.0, 0.0); vertices_count];
    for i in 0..its.vertices.len() {
        let index = cvt_vetices[i];
        if index == u32::MAX {
            continue;
        }
        result.vertices[index as usize] = its.vertices[i];
    }

    for (fidx, f) in its.indices.iter().enumerate() {
        if mask[fidx] {
            result.indices.push(Vec3f32i::new(cvt_vetices[f[0] as usize] as i32, cvt_vetices[f[1] as usize] as i32, cvt_vetices[f[2] as usize] as i32));
        }
    }

    result
}

/// Separate (A)rea (o)f (I)nterest .. AoI from model
/// NOTE: Only 2d filtration, do not filtrate by Z coordinate
/// CutSurface.hpp:59-61 / CutSurface.cpp:861-870
pub fn its_cut_ao_i(its: &indexed_triangle_set, bb: &BoundingBox, projection: &dyn IProjection) -> indexed_triangle_set {
    let mut skip_indicies: Vec<bool> = vec![false; its.indices.len()];
    set_skip_for_out_of_aoi(&mut skip_indicies, its, projection, bb);
    // invert values in vector of bool
    for b in skip_indicies.iter_mut() {
        *b = !*b;
    }
    its_mask(its, &skip_indicies)
}

/// Set true for indicies outward and almost parallel together.
/// Note: internally calculate normals
/// CutSurface.cpp:65-68 / CutSurface.cpp:872-895
pub fn set_skip_by_angle(skip_indicies: &mut [bool], its: &indexed_triangle_set, projection: &dyn IProject3d, max_angle: f64) {
    debug_assert!(max_angle < 90. && max_angle > 89.);
    debug_assert!(skip_indicies.len() == its.indices.len());
    let threshold = (max_angle / 180. * std::f64::consts::PI).cos() as f32;
    for (index, face) in its.indices.iter().enumerate() {
        if skip_indicies[index] {
            continue;
        }
        let n = its_face_normal_indices(its, face);
        let v = its.vertices[face[0] as usize];
        let vd = Vec3d::new(v.x as f64, v.y as f64, v.z as f64);
        // Improve: For Orthogonal Projection it is same for each vertex
        let projectedd = projection.project(&vd);
        let projected = Vec3f::new(projectedd.x as f32, projectedd.y as f32, projectedd.z as f32);
        let mut project_dir = projected - v;
        project_dir.normalize_mut();
        let cos_alpha = project_dir.dot(&n);
        if cos_alpha > threshold {
            continue;
        }
        skip_indicies[index] = true;
    }
}

/// Convert triangle mesh model to CGAL Surface_mesh
/// Filtrate out opposite triangles
/// CutSurface.cpp:93-95 / CutSurface.cpp:897-968
pub fn to_cgal(its: &indexed_triangle_set, skip_indicies: &[bool], flip: bool) -> CutMesh {
    let vertices = &its.vertices;
    let indices = &its.indices;

    let mut use_vetices: Vec<bool> = vec![false; vertices.len()];

    let mut vertices_count: usize = 0;
    let mut faces_count: usize = 0;
    let mut edges_count: usize = 0;

    for (index, t) in indices.iter().enumerate() {
        if skip_indicies[index] {
            continue;
        }
        faces_count += 1;
        let mut count_used_vertices: usize = 0;
        for k in 0..3 {
            let vi = t[k] as usize;
            if !use_vetices[vi] {
                vertices_count += 1;
                use_vetices[vi] = true;
            } else {
                count_used_vertices += 1;
            }
        }
        match count_used_vertices {
            3 => {} // all edges are already counted
            2 => edges_count += 2,
            1 | 0 => edges_count += 3,
            _ => debug_assert!(false),
        }
    }
    debug_assert!(vertices_count <= vertices.len());
    debug_assert!(edges_count <= (indices.len() * 3));
    debug_assert!(faces_count <= indices.len());

    let mut result = CutMesh::new();
    result.reserve(vertices_count as u32, edges_count as u32, faces_count as u32);

    let mut to_filtrated_vertices_index: Vec<VI> = vec![VI::null(); vertices.len()];
    let mut filtrated_vertices_index: usize = 0;
    for i in 0..vertices.len() {
        if use_vetices[i] {
            to_filtrated_vertices_index[i] = VI::new(filtrated_vertices_index as u32);
            filtrated_vertices_index += 1;
        }
    }

    for (i, v) in vertices.iter().enumerate() {
        if !use_vetices[i] {
            continue;
        }
        result.add_vertex([v.x as f64, v.y as f64, v.z as f64]);
    }

    if !flip {
        for (i, f) in indices.iter().enumerate() {
            if skip_indicies[i] {
                continue;
            }
            result.add_face(to_filtrated_vertices_index[f[0] as usize], to_filtrated_vertices_index[f[1] as usize], to_filtrated_vertices_index[f[2] as usize]);
        }
    } else {
        for (i, f) in indices.iter().enumerate() {
            if skip_indicies[i] {
                continue;
            }
            result.add_face(to_filtrated_vertices_index[f[2] as usize], to_filtrated_vertices_index[f[1] as usize], to_filtrated_vertices_index[f[0] as usize]);
        }
    }

    result
}

/// function to check result of projection. 2d int32_t -> 3d double
/// CutSurface.cpp:108 / CutSurface.cpp:970-988
pub fn exist_duplicit_vertex(mesh: &CutMesh) -> bool {
    let mut points: Vec<Vec3d> = Vec::with_capacity(mesh.vertices().count());
    // copy points
    for vi in mesh.vertices() {
        let p = mesh.point(vi);
        points.push(Vec3d::new(p[0], p[1], p[2]));
    }
    crate::clipper::libcxx_sort(&mut points, |v1, v2| v1.x < v2.x || (v1.x == v2.x && (v1.y < v2.y || (v1.y == v2.y && v1.z < v2.z))));
    // find first duplicit
    points.windows(2).any(|w| w[0] == w[1])
}

/// CutSurface.cpp:990-1061, the overload of `to_cgal` over shapes
pub mod of_shapes {
    use super::*;

    /// Covert 2d shape (e.g. Glyph) to CGAL model
    /// NOTE: internaly create
    /// edge_shape_map .. Property map to store conversion from edge to contour
    /// face_shape_map .. Property map to store conversion from face to contour
    /// CutSurface.cpp:106 / CutSurface.cpp:990-1061
    pub fn to_cgal(shapes: &ExPolygons, projection: &dyn IProjection) -> CutMesh {
        if shapes.is_empty() {
            return CutMesh::new();
        }

        let mut result = CutMesh::new();
        let edge_shape_map: EdgeShapeMap = result.add_property_map(edge_shape_map_name, IntersectingElement::default()).0;
        let face_shape_map: FaceShapeMap = result.add_property_map(face_shape_map_name, IntersectingElement::default()).0;

        let mut indices: Vec<VI> = Vec::new();
        let mut insert_contour = |result: &mut CutMesh, polygon: &Polygon| {
            indices.clear();
            indices.reserve(polygon.points.len() * 2);
            let num_vertices_old: usize = result.number_of_vertices() as usize;
            for polygon_point in &polygon.points {
                let (front, back) = projection.create_front_back(polygon_point);
                let v_front: P3 = [front.x, front.y, front.z];
                let vi1 = result.add_vertex(v_front);
                debug_assert!(vi1.idx() as usize == (indices.len() + num_vertices_old));
                indices.push(vi1);

                let v_back: P3 = [back.x, back.y, back.z];
                let vi2 = result.add_vertex(v_back);
                debug_assert!(vi2.idx() as usize == (indices.len() + num_vertices_old));
                indices.push(vi2);
            }

            let find_edge = |result: &CutMesh, fi: FI, from: VI, to: VI| -> EI {
                let mut hi = result.halfedge(fi);
                while result.target(hi) != to {
                    hi = result.next(hi);
                }
                debug_assert!(result.source(hi) == from);
                debug_assert!(result.target(hi) == to);
                result.edge(hi)
            };

            let mut contour_index: u32 = (num_vertices_old / 2) as u32;
            let mut i: i32 = 0;
            while i < indices.len() as i32 {
                let is_first = i == 0;
                let is_last = (i + 2) as usize >= indices.len();
                let j: i32 = if is_last { 0 } else { i + 2 };
                let (iu, ju) = (i as usize, j as usize);

                let fi1 = result.add_face(indices[iu], indices[ju], indices[iu + 1]);
                let ei1 = find_edge(result, fi1, indices[iu + 1], indices[iu]);
                let ei2 = find_edge(result, fi1, indices[ju], indices[iu + 1]);
                let fi2 = result.add_face(indices[ju], indices[ju + 1], indices[iu + 1]);
                let mut element = IntersectingElement { shape_point_index: contour_index, attr: Type::undefined.0 };
                if is_first {
                    element.set_is_first();
                }
                if is_last {
                    element.set_is_last();
                }
                edge_shape_map.set(ei1, *element.set_type(Type::edge_1));
                face_shape_map.set(fi1, *element.set_type(Type::face_1));
                edge_shape_map.set(ei2, *element.set_type(Type::edge_2));
                face_shape_map.set(fi2, *element.set_type(Type::face_2));
                contour_index += 1;
                i += 2;
            }
        };

        let count_point: usize = crate::geometry::count_points(shapes);
        result.reserve(
            result.number_of_vertices() + 2 * count_point as u32,
            result.number_of_edges() + 4 * count_point as u32,
            result.number_of_faces() + 2 * count_point as u32,
        );

        // Identify polygon
        for shape in shapes {
            insert_contour(&mut result, &shape.contour);
            for hole in &shape.holes {
                insert_contour(&mut result, hole);
            }
        }
        debug_assert!(!exist_duplicit_vertex(&result));
        result
    }
}

impl ModelCut2index {
    /// CutSurface.cpp:1063-1073 `ModelCut2index(const VCutAOIs &cuts)`
    pub fn new(cuts: &VCutAOIs) -> Self {
        // prepare offsets
        let mut m_offsets: Vec<u32> = Vec::with_capacity(cuts.len());
        let mut offset: u32 = 0;
        for model_cuts in cuts {
            m_offsets.push(offset);
            offset = offset.wrapping_add(model_cuts.len() as u32);
        }
        ModelCut2index { m_offsets, m_count: offset }
    }

    /// CutSurface.cpp:1075-1083
    pub fn calc_index(&self, id: &ModelCutId) -> u32 {
        debug_assert!((id.model_index as usize) < self.m_offsets.len());
        let offset = self.m_offsets[id.model_index as usize];
        let res = offset.wrapping_add(id.cut_index);
        debug_assert!(
            (((id.model_index + 1) as usize) < self.m_offsets.len() && res < self.m_offsets[(id.model_index + 1) as usize])
                || (((id.model_index + 1) as usize) == self.m_offsets.len() && res < self.m_count)
        );
        res
    }

    /// CutSurface.cpp:1085-1096
    pub fn calc_id(&self, index: u32) -> ModelCutId {
        debug_assert!(index < self.m_count);
        let mut result = ModelCutId { model_index: 0, cut_index: 0 };
        // find shape index
        for model_index in 1..self.m_offsets.len() {
            if self.m_offsets[model_index] > index {
                break;
            }
            result.model_index = model_index as u32;
        }
        result.cut_index = index.wrapping_sub(self.m_offsets[result.model_index as usize]);
        result
    }

    /// CutSurface.cpp:333
    pub fn get_count(&self) -> u32 {
        self.m_count
    }
    /// CutSurface.cpp:334
    pub fn get_offsets(&self) -> &Vec<u32> {
        &self.m_offsets
    }
}

// ===========================================================================
// cut_from_model help functions (CutSurface.cpp:1098-1246)
// ===========================================================================

/// Track source of intersection
/// Help for anotate inner and outer faces
/// CutSurface.cpp:1105-1173
///
/// `object` and `shape` are references in C++, used to tell which of the two meshes a callback
/// is given; here they are the addresses, compared the same way, and the meshes are read through
/// the references the callbacks get.
pub struct Visitor<'a> {
    pub object: *const CutMesh,
    pub shape: *const CutMesh,

    // Properties of the shape mesh:
    pub edge_shape_map: EdgeShapeMap,
    pub face_shape_map: FaceShapeMap,

    // Properties of the object mesh.
    pub vert_shape_map: VertexShapeMap,

    // check for anomalities
    pub is_valid: &'a Cell<bool>,

    // keep source of intersection for each intersection
    // used to copy data into vert_shape_map
    pub intersections: Vec<Option<IntersectingElement>>,
}

impl<'a> Visitor<'a> {
    /// CutSurface.cpp:1106-1110
    pub fn new(
        object: &CutMesh,
        shape: &CutMesh,
        edge_shape_map: EdgeShapeMap,
        face_shape_map: FaceShapeMap,
        vert_shape_map: VertexShapeMap,
        is_valid: &'a Cell<bool>,
    ) -> Self {
        Visitor { object: object as *const CutMesh, shape: shape as *const CutMesh, edge_shape_map, face_shape_map, vert_shape_map, is_valid, intersections: Vec::new() }
    }
}

impl<'a> CorefinementVisitor for Visitor<'a> {
    /// Called when a new intersection point is detected.
    /// The intersection is detected using a face of tm_f and an edge of tm_e.
    /// Intersecting an edge hh_edge from tm_f with a face h_e of tm_e.
    /// CutSurface.cpp:1248-1298
    fn intersection_point_detected(&mut self, i_id: usize, sdim: i32, h_f: HI, h_e: HI, tm_f: &CutMesh, tm_e: &CutMesh, is_target_coplanar: bool, is_source_coplanar: bool) {
        if i_id >= self.intersections.len() {
            let capacity = crate::utils::next_highest_power_of_2(i_id + 1);
            self.intersections.reserve(capacity.saturating_sub(self.intersections.len()));
            self.intersections.resize(capacity, None);
        }

        let mut intersection_ptr: Option<IntersectingElement> = None;
        if std::ptr::eq(tm_e, self.shape) {
            debug_assert!(std::ptr::eq(tm_f, self.object));
            let (shape, object) = (tm_e, tm_f);
            match sdim {
                1 => {
                    // edge x edge intersection
                    intersection_ptr = Some(self.edge_shape_map.get(shape.edge(h_e)));
                }
                2 => {
                    // edge x face intersection
                    intersection_ptr = Some(self.face_shape_map.get(shape.face(h_e)));
                }
                _ => debug_assert!(false),
            }
            if is_target_coplanar {
                self.vert_shape_map.set(object.source(h_f), intersection_ptr);
            }
            if is_source_coplanar {
                self.vert_shape_map.set(object.target(h_f), intersection_ptr);
            }
        } else {
            debug_assert!(std::ptr::eq(tm_f, self.shape) && std::ptr::eq(tm_e, self.object));
            debug_assert!(!is_target_coplanar);
            debug_assert!(!is_source_coplanar);
            let (shape, object) = (tm_f, tm_e);
            if is_target_coplanar || is_source_coplanar {
                self.is_valid.set(false);
            }
            intersection_ptr = Some(self.edge_shape_map.get(shape.edge(h_f)));
            if sdim == 0 {
                self.vert_shape_map.set(object.target(h_e), intersection_ptr);
            }
        }

        // The C++ dereferences the pointer here; it is null only after the `assert(false)` above.
        if intersection_ptr.expect("intersection_ptr is nullptr").shape_point_index == u32::MAX {
            // there is unexpected intersection
            // Top (or Bottom) shape contour edge (or vertex) intersection
            // Suggest to change projection min/max limits
            self.is_valid.set(false);
        }
        self.intersections[i_id] = intersection_ptr;
    }

    /// Called when a new vertex is added in tm (either an edge split or a vertex inserted in the interior of a face).
    /// Fill vertex_shape_map by intersections
    /// CutSurface.cpp:1300-1309
    fn new_vertex_added(&mut self, i_id: usize, v: VI, tm: &CutMesh) {
        debug_assert!(std::ptr::eq(tm, self.object));
        debug_assert!(i_id < self.intersections.len());
        let intersection_ptr = self.intersections[i_id];
        debug_assert!(intersection_ptr.is_some());
        // intersection was not filled in function intersection_point_detected
        //assert(intersection_ptr->point_index != std::numeric_limits<uint32_t>::max());
        self.vert_shape_map.set(v, intersection_ptr);
    }
}

/// Distiquish face type for half edge
/// CutSurface.cpp:1185-1189 / CutSurface.cpp:1311-1375
pub fn is_face_inside(hi: HI, mesh: &CutMesh, shape_mesh: &CutMesh, vertex_shape_map: &VertexShapeMap, shape2index: &ExPolygonsIndices) -> bool {
    let vi_from = mesh.source(hi);
    let vi_to = mesh.target(hi);
    // This face has a constrained edge.
    let shape_from: IntersectingElement = vertex_shape_map.get(vi_from).expect("*vertex_shape_map[vi_from] is nullptr");
    let shape_to: IntersectingElement = vertex_shape_map.get(vi_to).expect("*vertex_shape_map[vi_to] is nullptr");
    debug_assert!(shape_from.shape_point_index != u32::MAX);
    debug_assert!(shape_from.attr != Type::undefined.0);
    debug_assert!(shape_to.shape_point_index != u32::MAX);
    debug_assert!(shape_to.attr != Type::undefined.0);

    // index into contour
    let i_from: u32 = shape_from.shape_point_index;
    let i_to: u32 = shape_to.shape_point_index;
    let type_from = shape_from.get_type();
    let type_to = shape_to.get_type();
    if i_from == i_to && type_from == type_to {
        // intersecting element must be face
        debug_assert!(type_from == Type::face_1 || type_from == Type::face_2);

        // count of vertices is twice as count of point in the contour
        let i: u32 = i_from.wrapping_mul(2);
        // j is next contour point in vertices
        let mut j: u32 = i.wrapping_add(2);
        if shape_from.is_last() {
            let mut point_id: ExPolygonsIndex = shape2index.cvt_index(i_from);
            point_id.point_index = 0;
            j = shape2index.cvt(&point_id).wrapping_mul(2);
        }

        // opposit point(in triangle face) to edge
        let p: P3 = mesh.point(mesh.target(mesh.next(hi)));

        // abc is source triangle face
        let abcp: Ordering = if type_from == Type::face_1 {
            orientation_3(&shape_mesh.point(VI::new(i)), &shape_mesh.point(VI::new(i.wrapping_add(1))), &shape_mesh.point(VI::new(j)), &p)
        } else {
            // type_from == IntersectingElement::Type::face_2
            orientation_3(&shape_mesh.point(VI::new(j)), &shape_mesh.point(VI::new(i.wrapping_add(1))), &shape_mesh.point(VI::new(j.wrapping_add(1))), &p)
        };
        // CGAL::POSITIVE
        abcp == Ordering::Greater
    } else if i_from < i_to || (i_from == i_to && type_from < type_to) {
        let is_last = shape_to.is_last() && shape_from.is_first();
        // check continuity of indicies
        debug_assert!(i_from == i_to || is_last || (i_from + 1) == i_to);
        !is_last
    } else {
        debug_assert!(i_from > i_to || (i_from == i_to && type_from > type_to));
        let is_last = shape_to.is_first() && shape_from.is_last();
        // check continuity of indicies
        debug_assert!(i_from == i_to || is_last || (i_to + 1) == i_from);
        is_last
    }
}

/// Face with constrained edge are inside/outside by type of intersection
/// Other set to not_constrained(still it could be inside/outside)
/// CutSurface.cpp:1201-1206 / CutSurface.cpp:1377-1398
pub fn set_face_type(
    face_type_map: &FaceTypeMap,
    mesh: &CutMesh,
    vertex_shape_map: &VertexShapeMap,
    ecm: &EdgeBoolMap,
    shape_mesh: &CutMesh,
    shape2index: &ExPolygonsIndices,
) {
    for ei in mesh.edges() {
        if !ecm.get(ei) {
            continue;
        }
        let hi = mesh.halfedge(ei);
        let fi = mesh.face(hi);
        let is_inside = is_face_inside(hi, mesh, shape_mesh, vertex_shape_map, shape2index);
        face_type_map.set(fi, if is_inside { FaceType::inside } else { FaceType::outside });
        let hi_op = mesh.opposite(hi);
        debug_assert!(hi_op.is_valid());
        if !hi_op.is_valid() {
            continue;
        }
        let fi_op = mesh.face(hi_op);
        debug_assert!(fi_op.is_valid());
        if !fi_op.is_valid() {
            continue;
        }
        face_type_map.set(fi_op, if !is_inside { FaceType::inside } else { FaceType::outside });
    }
}

/// Create AOIs(area of interest) on model surface
/// CutSurface.cpp:262-266 / CutSurface.cpp:1400-1443
pub fn cut_from_model(
    cgal_model: &mut CutMesh,
    shapes: &ExPolygons,
    cgal_shape: &mut CutMesh,
    _projection_ratio: f32,
    s2i: &ExPolygonsIndices,
    pmp: &dyn PolygonMeshProcessing,
) -> CutAOIs {
    // pointer to edge or face shape_map
    let vert_shape_map: VertexShapeMap = cgal_model.add_property_map(vert_shape_map_name, None).0;

    // detect anomalities in visitor.
    let is_valid = Cell::new(true);
    // NOTE: map are created when convert shapes to cgal model
    let edge_shape_map: EdgeShapeMap = cgal_shape.property_map(edge_shape_map_name).0;
    let face_shape_map: FaceShapeMap = cgal_shape.property_map(face_shape_map_name).0;
    let mut visitor = Visitor::new(cgal_model, cgal_shape, edge_shape_map, face_shape_map, vert_shape_map.clone(), &is_valid);

    // a property map containing the constrained-or-not status of each edge
    let ecm: EdgeBoolMap = cgal_model.add_property_map(is_constrained_edge_name, false).0;
    {
        let p = NamedParameters::default_values().visitor(&mut visitor).edge_is_constrained_map(ecm.clone()).throw_on_self_intersection(false);
        let q = NamedParameters::default_values().do_not_modify(true);
        pmp.corefine(cgal_model, cgal_shape, p, q);
    }

    if !is_valid.get() {
        return CutAOIs::new();
    }

    let face_type_map: FaceTypeMap = cgal_model.add_property_map(face_type_map_name, FaceType::not_constrained).0;

    // Select inside and outside face in model
    set_face_type(&face_type_map, cgal_model, &vert_shape_map, &ecm, cgal_shape, s2i);

    // flood fill the other faces inside the region.
    flood_fill_inner(cgal_model, &face_type_map);

    // IMPROVE: AOIs area could be created during flood fill
    create_cut_area_of_interests(cgal_model, shapes, &face_type_map)
}

/// Change FaceType from not_constrained to inside
/// For neighbor(or neighbor of neighbor of ...) of inside triangles.
/// Process only not_constrained triangles
/// CutSurface.cpp:1215 / CutSurface.cpp:1445-1508
pub fn flood_fill_inner(mesh: &CutMesh, face_type_map: &FaceTypeMap) {
    let mut process: Vec<FI> = Vec::new();
    // guess count of connected not constrained triangles
    let guess_size: usize = 128;
    process.reserve(guess_size);

    // check if neighbor(one of three in triangle) has type inside
    let has_inside_neighbor = |fi: FI| -> bool {
        let mut hi = mesh.halfedge(fi);
        let hi_end = hi;
        // loop over 3 half edges of face
        loop {
            let hi_opposite = mesh.opposite(hi);
            // open edge doesn't have opposit half edge
            if hi_opposite.is_valid() {
                let fi_opposite = mesh.face(hi_opposite);
                if fi_opposite.is_valid() && face_type_map.get(fi_opposite) == FaceType::inside {
                    return true;
                }
            }
            // exist_next()
            hi = mesh.next(hi);
            if hi == hi_end {
                break;
            }
        }
        false
    };

    for fi in mesh.faces() {
        let type_ = face_type_map.get(fi);
        if type_ != FaceType::not_constrained {
            continue;
        }
        if !has_inside_neighbor(fi) {
            continue;
        }
        debug_assert!(process.is_empty());
        process.push(fi);

        while let Some(process_fi) = process.pop() {
            // Do not fill twice
            if face_type_map.get(process_fi) == FaceType::inside {
                continue;
            }
            face_type_map.set(process_fi, FaceType::inside);

            // check neighbor triangle
            let mut hi = mesh.halfedge(process_fi);
            let hi_end = hi;
            loop {
                let hi_opposite = mesh.opposite(hi);
                // open edge doesn't have opposit half edge
                if hi_opposite.is_valid() {
                    let fi_opposite = mesh.face(hi_opposite);
                    if fi_opposite.is_valid() {
                        let type_opposite = face_type_map.get(fi_opposite);
                        if type_opposite == FaceType::not_constrained {
                            process.push(fi_opposite);
                        }
                    }
                }
                // exist_next()
                hi = mesh.next(hi);
                if hi == hi_end {
                    break;
                }
            }
        }
    }
}

/// Collect connected inside faces
/// Collect outline half edges
/// CutSurface.cpp:1227-1231 / CutSurface.cpp:1510-1558
pub fn collect_surface_data(process: &mut VecDeque<FI>, faces: &mut Vec<FI>, outlines: &mut Vec<HI>, face_type_map: &FaceTypeMap, mesh: &CutMesh) {
    debug_assert!(!process.is_empty());
    debug_assert!(faces.is_empty());
    debug_assert!(outlines.is_empty());
    while let Some(fi) = process.pop_front() {
        let fi_type = face_type_map.get(fi);
        // Do not process twice
        if fi_type == FaceType::inside_processed {
            continue;
        }
        debug_assert!(fi_type == FaceType::inside);
        // flag face as processed
        face_type_map.set(fi, FaceType::inside_processed);
        faces.push(fi);

        // check neighbor triangle
        let mut hi = mesh.halfedge(fi);
        let hi_end = hi;
        loop {
            let hi_opposite = mesh.opposite(hi);
            // open edge doesn't have opposit half edge
            if !hi_opposite.is_valid() {
                outlines.push(hi);
                hi = mesh.next(hi);
            } else {
                let fi_opposite = mesh.face(hi_opposite);
                if !fi_opposite.is_valid() {
                    outlines.push(hi);
                    hi = mesh.next(hi);
                } else {
                    let side = face_type_map.get(fi_opposite);
                    if side == FaceType::inside {
                        process.push_back(fi_opposite);
                    } else if side == FaceType::outside {
                        // store outlines
                        outlines.push(hi);
                    }
                    hi = mesh.next(hi);
                }
            }
            if hi == hi_end {
                break;
            }
        }
    }
}

/// Create map to reduce unnecesary triangles,
/// Triangles are made by divided quad to two triangles
/// on side of cutting shape mesh
/// Note: also use from mesh (have to be created)
/// face_type_map .. Type of shape inside / outside
/// vert_shape_map .. Source of outline vertex
/// CutSurface.cpp:240 / CutSurface.cpp:1560-1651
pub fn create_reduce_map(reduction_map: &ReductionMap, mesh: &CutMesh) {
    let vert_shape_map: VertexShapeMap = mesh.property_map(vert_shape_map_name).0;
    let ecm: EdgeBoolMap = mesh.property_map(is_constrained_edge_name).0;

    // check if vertex was made by edge_2 which is diagonal of quad
    let is_reducible_vertex = |reduction_from: VI| -> bool {
        let Some(ie) = vert_shape_map.get(reduction_from) else {
            return false;
        };
        let type_ = ie.get_type();
        type_ == Type::edge_2
    };

    // Append reduction or change existing one.
    // hi .. HalEdge between outside and inside face.
    // Target vertex will be reduced, source vertex left
    let add_reduction = |hi: HI| {
        let erase = mesh.target(hi);
        let left = mesh.source(hi);
        debug_assert!(is_reducible_vertex(erase));
        debug_assert!(!is_reducible_vertex(left));
        let vi = reduction_map.get(erase);
        // check if it is first add
        if vi.is_valid() {
            return;
        }

        // check that all triangles after reduction has 'erase' and 'left' vertex
        // on same side of opposite line of vertex in triangle
        let v_erase = to_vec3d(&mesh.point(erase));
        let v_left = to_vec3d(&mesh.point(left));
        for fi in mesh.faces_around_target(hi) {
            if !fi.is_valid() {
                continue;
            }
            // get vertices of rest
            let mut vi_a = VI::null();
            let mut vi_b = VI::null();
            for vi in mesh.vertices_around_face(mesh.halfedge(fi)) {
                if !vi.is_valid() {
                    continue;
                }
                if vi == erase {
                    continue;
                }
                if !vi_a.is_valid() {
                    vi_a = vi;
                } else {
                    debug_assert!(!vi_b.is_valid());
                    vi_b = vi;
                }
            }
            debug_assert!(vi_b.is_valid());
            // do not check triangle, which will be removed
            if vi_a == left || vi_b == left {
                continue;
            }

            let v_a = to_vec3d(&mesh.point(vi_a));
            let v_b = to_vec3d(&mesh.point(vi_b));
            // Vectors of triangle edges
            let v_ab = v_b - v_a;
            let v_ae = v_erase - v_a;
            let v_al = v_left - v_a;

            let n1 = cross3(&v_ab, &v_ae);
            let n2 = cross3(&v_ab, &v_al);
            // check that normal has same direction
            if ((n1.x > 0.) != (n2.x > 0.)) || ((n1.y > 0.) != (n2.y > 0.)) || ((n1.z > 0.) != (n2.z > 0.)) {
                return; // this reduction will create CCW triangle
            }
        }

        reduction_map.set(erase, left);
        // I have no better rule than take the first
        // for decide which reduction will be better
        // But it could be use only one of them
    };

    for ei in mesh.edges() {
        if !ecm.get(ei) {
            continue;
        }
        let hi = mesh.halfedge(ei);
        let vi = mesh.target(hi);
        if is_reducible_vertex(vi) {
            add_reduction(hi);
        }

        let hi_op = mesh.opposite(hi);
        let vi_op = mesh.target(hi_op);
        if is_reducible_vertex(vi_op) {
            add_reduction(hi_op);
        }
    }
}

/// Create areas from mesh surface
/// CutSurface.cpp:1242-1244 / CutSurface.cpp:1653-1688
pub fn create_cut_area_of_interests(mesh: &CutMesh, shapes: &ExPolygons, face_type_map: &FaceTypeMap) -> CutAOIs {
    // IMPROVE: Create better heuristic for count.
    let faces_per_cut: usize = mesh.faces().count() / shapes.len();
    let outlines_per_cut: usize = faces_per_cut / 2;
    let cuts_per_model: usize = shapes.len() * 2;

    let mut result = CutAOIs::new();
    result.reserve(cuts_per_model);

    // It is faster to use one queue for all cuts
    let mut process: VecDeque<FI> = VecDeque::new();
    for fi in mesh.faces() {
        if face_type_map.get(fi) != FaceType::inside {
            continue;
        }

        let mut cut: CutAOI = (Vec::new(), Vec::new());
        {
            let (faces, outlines) = (&mut cut.0, &mut cut.1);

            // faces for one surface cut
            faces.reserve(faces_per_cut);
            // outline for one surface cut
            outlines.reserve(outlines_per_cut);

            debug_assert!(process.is_empty());
            // Process queue of faces to separate to surface_cut
            process.push_back(fi);
            collect_surface_data(&mut process, faces, outlines, face_type_map, mesh);
            debug_assert!(!faces.is_empty());
            debug_assert!(!outlines.is_empty());
        }
        result.push(cut);
    }
    result
}

/// Calculate projection distance of point [in mm]
/// CutSurface.cpp:1700-1703 / CutSurface.cpp:1707-1736
pub fn calc_distance(p: &P3, pi: u32, shapes_mesh: &CutMesh, projection_ratio: f32) -> f32 {
    // It is known because shapes_mesh is created inside of private space
    let vi_start = VI::new(pi.wrapping_mul(2));
    let vi_end = VI::new(pi.wrapping_mul(2).wrapping_add(1));

    // Get range for intersection
    let start: P3 = shapes_mesh.point(vi_start);
    let end: P3 = shapes_mesh.point(vi_end);

    // find index in vector with biggest difference
    let mut max_i: usize = 0;
    let mut max_val: f32 = 0.;
    for i in 0..3 {
        let mut val: f32 = (start[i] - end[i]) as f32;
        // abs value
        if val < 0. {
            val *= -1.;
        }
        if max_val < val {
            max_val = val;
            max_i = i;
        }
    }

    let from_start: f32 = (p[max_i] - start[max_i]) as f32;
    let best_distance: f32 = (projection_ratio as f64 * (end[max_i] - start[max_i])) as f32;
    from_start - best_distance
}

/// Calculate distances for SurfacePatches outline points
/// NOTE:
/// each model has to have "vert_shape_map" .. Know source of new vertices
/// CutSurface.cpp:426-430 / CutSurface.cpp:1738-1773
pub fn calc_distances(patches: &SurfacePatches, models: &CutMeshes, shapes_mesh: &CutMesh, count_shapes_points: usize, projection_ratio: f32) -> VDistances {
    let mut result: VDistances = vec![ProjectionDistances::new(); count_shapes_points];
    for (patch_index, patch) in patches.iter().enumerate() {
        // map is created during intersection by corefine visitor
        let vert_shape_map: VertexShapeMap = models[patch.model_id].property_map(vert_shape_map_name).0;
        let patch_index = patch_index as u32;
        // map is created during patch creation / dividing
        let cvt: CvtVI2VI = patch.mesh.property_map(patch_source_name).0;
        // for each point on outline
        for loop_ in &patch.loops {
            for &vi_patch in loop_ {
                let vi_model = cvt.get(vi_patch);
                if !vi_model.is_valid() {
                    continue;
                }
                let Some(ie) = vert_shape_map.get(vi_model) else {
                    continue;
                };
                debug_assert!(ie.shape_point_index != u32::MAX);
                debug_assert!(ie.attr != Type::undefined.0);
                let pi: u32 = ie.shape_point_index;
                debug_assert!(pi as usize <= count_shapes_points);
                let pds = &mut result[pi as usize];
                let model_index = patch.model_id as u32;
                let aoi_index = patch.aoi_id as u32;
                let p: P3 = patch.mesh.point(vi_patch);
                let distance = calc_distance(&p, pi, shapes_mesh, projection_ratio);
                pds.push(ProjectionDistance { model_index, aoi_index, patch_index, distance });
            }
        }
    }
    result
}

// ===========================================================================
// functions for choose_best_distance (CutSurface.cpp:1776-2319)
// ===========================================================================

/// `Linef` (Line.hpp:228-240)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Linef {
    pub a: Vec2d,
    pub b: Vec2d,
}

impl Linef {
    pub fn new(a: Vec2d, b: Vec2d) -> Self {
        Linef { a, b }
    }
}

// AABBTreeLines.hpp over `Linef`. The crate's `aabb_tree_lines` is the instantiation over the
// integer `Line` (centroids and nearest points truncated to `coord_t`); over `Linef` the scalar
// is `double` and nothing is truncated.

/// AABBTreeLines.hpp:166-200 `build_aabb_tree_over_indexed_lines(const std::vector<Linef> &lines)`
fn build_aabb_tree_over_indexed_lines(lines: &[Linef]) -> tree2d::Tree {
    struct InputType {
        m_idx: usize,
        m_bbox: tree2d::BoundingBox,
        m_centroid: [f64; 2],
    }
    impl tree2d::SourceNode for InputType {
        fn idx(&self) -> usize {
            self.m_idx
        }
        fn centroid(&self) -> [f64; 2] {
            self.m_centroid
        }
        fn bbox(&self) -> tree2d::BoundingBox {
            self.m_bbox
        }
    }

    let mut input: Vec<InputType> = Vec::with_capacity(lines.len());
    for (i, line) in lines.iter().enumerate() {
        // n.m_centroid = (line.a + line.b) * 0.5;
        let m_centroid = [(line.a.x + line.b.x) * 0.5, (line.a.y + line.b.y) * 0.5];
        // n.m_bbox = BoundingBox(line.a, line.a); n.m_bbox.extend(line.b);
        let mut m_bbox = tree2d::BoundingBox::from_point([line.a.x, line.a.y]);
        m_bbox.extend_point([line.b.x, line.b.y]);
        input.push(InputType { m_idx: i, m_bbox, m_centroid });
    }

    let mut out = tree2d::Tree::new();
    out.build(input);
    out
}

/// Line.hpp:42-68 `line_alg::distance_to_squared(line, point, nearest_point)` for `Linef`
fn distance_to_squared(line: &Linef, point: &Vec2d, nearest_point: &mut Vec2d) -> f64 {
    let v = Vec2d::new(line.b.x - line.a.x, line.b.y - line.a.y);
    let va = Vec2d::new(point.x - line.a.x, point.y - line.a.y);
    let l2 = v.x * v.x + v.y * v.y; // avoid a sqrt
    if l2 == 0.0 {
        // a == b case
        *nearest_point = line.a;
        return va.x * va.x + va.y * va.y;
    }
    // Consider the line extending the segment, parameterized as a + t (b - a).
    // We find projection of this point onto the line.
    // It falls where t = [(this-a) . (b-a)] / |b-a|^2
    let t = (va.x * v.x + va.y * v.y) / l2;
    if t <= 0.0 {
        // beyond the 'a' end of the segment
        *nearest_point = line.a;
        return va.x * va.x + va.y * va.y;
    } else if t >= 1.0 {
        // beyond the 'b' end of the segment
        *nearest_point = line.b;
        let d = Vec2d::new(point.x - line.b.x, point.y - line.b.y);
        return d.x * d.x + d.y * d.y;
    }

    *nearest_point = Vec2d::new(line.a.x + t * v.x, line.a.y + t * v.y);
    let d = Vec2d::new(t * v.x - va.x, t * v.y - va.y);
    d.x * d.x + d.y * d.y
}

/// AABBTreeIndirect.hpp:551-632 `detail::squared_distance_to_indexed_primitives_recursive` with
/// AABBTreeLines.hpp:17-36 `IndexedLinesDistancer<Linef, Tree<2, double>, Vec2d>`
#[allow(unused_assignments)]
fn squared_distance_to_indexed_primitives_recursive(
    lines: &[Linef],
    tree: &tree2d::Tree,
    origin: &Vec2d,
    node_idx: usize,
    low_sqr_d: f64,
    mut up_sqr_d: f64,
    i: &mut usize,
    c: &mut Vec2d,
) -> f64 {
    if low_sqr_d > up_sqr_d {
        return low_sqr_d;
    }

    // Save the best achieved hit.
    macro_rules! set_min {
        ($sqr_d_candidate:expr, $i_candidate:expr, $c_candidate:expr) => {{
            let sqr_d_candidate: f64 = $sqr_d_candidate;
            if sqr_d_candidate < up_sqr_d {
                *i = $i_candidate;
                *c = $c_candidate;
                up_sqr_d = sqr_d_candidate;
            }
        }};
    }

    let node = tree.node(node_idx);
    assert!(node.is_valid());
    if node.is_leaf() {
        // distancer.closest_point_to_origin(node.idx, sqr_dist)
        let mut nearest_point = Vec2d::new(0.0, 0.0);
        let sqr_dist = distance_to_squared(&lines[node.idx], origin, &mut nearest_point);
        set_min!(sqr_dist, node.idx, nearest_point);
    } else {
        let left_node_idx = node_idx * 2 + 1;
        let right_node_idx = left_node_idx + 1;
        let node_left = tree.node(left_node_idx);
        let node_right = tree.node(right_node_idx);
        assert!(node_left.is_valid());
        assert!(node_right.is_valid());

        let mut looked_left = false;
        let mut looked_right = false;
        let o = [origin.x, origin.y];

        macro_rules! look_left {
            () => {{
                let mut i_left = 0usize;
                let mut c_left = *c;
                let sqr_d_left = squared_distance_to_indexed_primitives_recursive(lines, tree, origin, left_node_idx, low_sqr_d, up_sqr_d, &mut i_left, &mut c_left);
                set_min!(sqr_d_left, i_left, c_left);
                looked_left = true;
            }};
        }
        macro_rules! look_right {
            () => {{
                let mut i_right = 0usize;
                let mut c_right = *c;
                let sqr_d_right = squared_distance_to_indexed_primitives_recursive(lines, tree, origin, right_node_idx, low_sqr_d, up_sqr_d, &mut i_right, &mut c_right);
                set_min!(sqr_d_right, i_right, c_right);
                looked_right = true;
            }};
        }

        // must look left or right if in box
        if node_left.bbox.contains(o) {
            look_left!();
        }
        if node_right.bbox.contains(o) {
            look_right!();
        }
        // if haven't looked left and could be less than current min, then look
        let left_up_sqr_d = node_left.bbox.squared_exterior_distance(o);
        let right_up_sqr_d = node_right.bbox.squared_exterior_distance(o);
        if left_up_sqr_d < right_up_sqr_d {
            if !looked_left && left_up_sqr_d < up_sqr_d {
                look_left!();
            }
            if !looked_right && right_up_sqr_d < up_sqr_d {
                look_right!();
            }
        } else {
            if !looked_right && right_up_sqr_d < up_sqr_d {
                look_right!();
            }
            if !looked_left && left_up_sqr_d < up_sqr_d {
                look_left!();
            }
        }
    }
    up_sqr_d
}

/// AABBTreeLines.hpp:205-219 `squared_distance_to_indexed_lines(lines, tree, point, hit_idx_out,
/// hit_point_out, max_sqr_dist)` over `Linef`: the squared distance to the closest point, or -1
/// if the input is empty or there is no closer point than `max_sqr_dist`
fn squared_distance_to_indexed_lines(lines: &[Linef], tree: &tree2d::Tree, point: &Vec2d, hit_idx_out: &mut usize, hit_point_out: &mut Vec2d, max_sqr_dist: f64) -> f64 {
    if tree.empty() {
        return -1.0;
    }
    squared_distance_to_indexed_primitives_recursive(lines, tree, point, 0, 0.0, max_sqr_dist, hit_idx_out, hit_point_out)
}

// euler square size of vector stored in Point
/// CutSurface.cpp:1782 / CutSurface.cpp:1825-1828
pub fn calc_size_sq(p: &Point) -> f32 {
    // NOTE: p.squaredNorm() can't be use due to overflow max int value
    // (float) p.x() * p.x() + (float) p.y() * p.y(): one expression, the left product is fused
    let (x, y) = (p.x as f32, p.y as f32);
    x.mul_add(x, y * y)
}

// structure to store index and distance together
/// CutSurface.cpp:1785-1791
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClosePoint {
    // index of closest point from another shape
    pub index: u32,
    // squere distance to index
    pub dist_sq: f32,
}

impl Default for ClosePoint {
    fn default() -> Self {
        ClosePoint { index: u32::MAX, dist_sq: f32::MAX }
    }
}

/// CutSurface.cpp:1793-1800
pub struct SearchData {
    // IMPROVE: float lines are enough
    pub lines: Vec<Linef>,
    // convert line index into Shape point index
    pub cvt: Vec<usize>,
    // contain lines from prev point to Point index
    pub tree: tree2d::Tree,
}

/// CutSurface.cpp:1802 / CutSurface.cpp:1830-1864
pub fn create_search_data(shapes: &ExPolygons, mask: &[bool]) -> SearchData {
    // IMPROVE: Use float precission (it is enough)
    let mut sd = SearchData { lines: Vec::new(), cvt: Vec::new(), tree: tree2d::Tree::new() };
    sd.lines.reserve(mask.len());
    sd.cvt.reserve(mask.len());
    let mut index: usize = 0;
    let mut add_lines = |sd: &mut SearchData, poly: &Polygon| {
        let back = poly.points.last().expect("poly.back()");
        let mut prev = Vec2d::new(back.x as f64, back.y as f64);
        let mut use_point = mask[index + poly.points.len() - 1];
        for p in &poly.points {
            if !use_point {
                use_point = mask[index];
                if use_point {
                    prev = Vec2d::new(p.x as f64, p.y as f64);
                }
            } else if !mask[index] {
                use_point = false;
            } else {
                let p_d = Vec2d::new(p.x as f64, p.y as f64);
                sd.lines.push(Linef::new(prev, p_d));
                sd.cvt.push(index);
                prev = p_d;
            }
            index += 1;
        }
    };

    for shape in shapes {
        add_lines(&mut sd, &shape.contour);
        for hole in &shape.holes {
            add_lines(&mut sd, hole);
        }
    }
    sd.tree = build_aabb_tree_over_indexed_lines(&sd.lines);
    sd
}

/// CutSurface.cpp:1803 / CutSurface.cpp:1866-1911
pub fn get_closest_point_index(sd: &SearchData, line_idx: usize, hit_point: &Vec2d, shapes: &ExPolygons, s2i: &ExPolygonsIndices) -> u32 {
    let line = &sd.lines[line_idx];
    let dir = Vec2d::new(line.a.x - line.b.x, line.a.y - line.b.y);
    let dir_abs = Vec2d::new(dir.x.abs(), dir.y.abs());
    // use x coordinate
    let i: usize = if dir_abs.x > dir_abs.y { 0 } else { 1 };
    let coord = |v: &Vec2d| if i == 0 { v.x } else { v.y };

    let use_index = (coord(&line.a) - coord(hit_point)).abs() > (coord(&line.b) - coord(hit_point)).abs();
    let point_index: usize = sd.cvt[line_idx];

    // Lambda used only for check result
    let is_same = |p: &Vec2d, i: usize| -> bool {
        let id = s2i.cvt_index(i as u32);
        let shape = &shapes[id.expolygons_index as usize];
        let poly = if id.polygon_index == 0 { &shape.contour } else { &shape.holes[(id.polygon_index - 1) as usize] };
        let p_ = Point::new(p.x as i64, p.y as i64);
        p_ == poly.points[id.point_index as usize]
    };
    let _ = &is_same;

    if use_index {
        debug_assert!(is_same(&line.b, point_index));
        return point_index as u32;
    }
    let id = s2i.cvt_index(point_index as u32);
    if id.point_index != 0 {
        debug_assert!(is_same(&line.a, point_index - 1));
        return (point_index - 1) as u32;
    }
    let shape = &shapes[id.expolygons_index as usize];
    let count_polygon_points: usize = if id.polygon_index == 0 { shape.contour.points.len() } else { shape.holes[(id.polygon_index - 1) as usize].points.len() };
    let prev_point_index: usize = point_index + (count_polygon_points - 1);
    debug_assert!(is_same(&line.a, prev_point_index));
    // return previous point index
    prev_point_index as u32
}

// use AABB Tree Lines to find closest point
/// CutSurface.cpp:1806 / CutSurface.cpp:1914-1950
pub fn find_closest_point_index(p: &Point, shapes: &ExPolygons, s2i: &ExPolygonsIndices, mask: &[bool]) -> u32 {
    let sd = create_search_data(shapes, mask);
    if sd.tree.nodes().is_empty() {
        // no lines in expolygon, check whether exist point to start
        let mut closest_square_distance: f64 = f64::INFINITY;
        let mut closest_id: u32 = u32::MAX; // uint32_t closest_id = -1;
        for i in 0..mask.len() as u32 {
            if mask[i as usize] {
                let ei: ExPolygonsIndex = s2i.cvt_index(i);
                let s_p: Point = if ei.is_contour() {
                    shapes[ei.expolygons_index as usize].contour.points[ei.point_index as usize]
                } else {
                    shapes[ei.expolygons_index as usize].holes[ei.hole_index() as usize].points[ei.point_index as usize]
                };
                let (dx, dy) = ((p.x - s_p.x) as f64, (p.y - s_p.y) as f64);
                let square_distance: f64 = dx * dx + dy * dy;
                if closest_id as usize >= mask.len() || closest_square_distance > square_distance {
                    closest_id = i;
                    closest_square_distance = square_distance;
                }
            }
        }
        debug_assert!((closest_id as usize) < mask.len());
        return closest_id;
    }
    let mut line_idx: usize = usize::MAX;
    let mut hit_point = Vec2d::new(0.0, 0.0);
    let p_d = Vec2d::new(p.x as f64, p.y as f64);
    let distance_sq = squared_distance_to_indexed_lines(&sd.lines, &sd.tree, &p_d, &mut line_idx, &mut hit_point, f64::INFINITY);
    debug_assert!(distance_sq > 0.);
    let _ = distance_sq;

    // IMPROVE: one could use line ratio to find closest point
    get_closest_point_index(&sd, line_idx, &hit_point, shapes, s2i)
}

/// CutSurface.cpp:1808 / CutSurface.cpp:1952-2021
pub fn find_closest_point_pair(shapes: &ExPolygons, done_shapes: &[bool], s2i: &ExPolygonsIndices, mask: &[bool]) -> (u32, u32) {
    debug_assert!(mask.len() == s2i.get_count() as usize);
    debug_assert!(done_shapes.len() == shapes.len());
    let mut unfinished_mask: Vec<bool> = mask.to_vec(); // copy

    let mut index: usize = 0;
    for shape_index in 0..shapes.len() {
        let count: usize = crate::geometry::count_points_expoly(&shapes[shape_index]);
        if done_shapes[shape_index] {
            for _ in 0..count {
                unfinished_mask[index] = false;
                index += 1;
            }
        } else {
            index += count;
        }
    }
    debug_assert!(index == s2i.get_count() as usize);
    let sd = create_search_data(shapes, &unfinished_mask);

    struct ClosestPair {
        finish_idx: usize,
        unfinished_line_idx: usize,
        hit_point: Vec2d,
        distance_sq: f64,
    }
    let mut cp = ClosestPair { finish_idx: usize::MAX, unfinished_line_idx: usize::MAX, hit_point: Vec2d::new(0.0, 0.0), distance_sq: f64::MAX };

    index = 0;
    for shape_index in 0..shapes.len() {
        let shape = &shapes[shape_index];
        if !done_shapes[shape_index] {
            index += crate::geometry::count_points_expoly(shape);
            continue;
        }

        let mut search_in_polygon = |polygon: &Polygon| {
            for i in 0..polygon.points.len() {
                if mask[index] {
                    let p_d = Vec2d::new(polygon.points[i].x as f64, polygon.points[i].y as f64);
                    let mut line_idx: usize = usize::MAX;
                    let mut hit_point = Vec2d::new(0.0, 0.0);
                    let distance_sq = squared_distance_to_indexed_lines(&sd.lines, &sd.tree, &p_d, &mut line_idx, &mut hit_point, cp.distance_sq);
                    if !(distance_sq < 0. || distance_sq >= cp.distance_sq) {
                        debug_assert!(line_idx < sd.lines.len());
                        cp.distance_sq = distance_sq;
                        cp.unfinished_line_idx = line_idx;
                        cp.hit_point = hit_point;
                        cp.finish_idx = index;
                    }
                }
                index += 1;
            }
        };
        search_in_polygon(&shape.contour);
        for hole in &shape.holes {
            search_in_polygon(hole);
        }
    }
    debug_assert!(index == s2i.get_count() as usize);
    // check that exists result
    if cp.finish_idx == usize::MAX {
        // std::make_pair(size_t max, size_t max) converted to std::pair<uint32_t, uint32_t>
        return (usize::MAX as u32, usize::MAX as u32);
    }

    let unfinished_idx: usize = get_closest_point_index(&sd, cp.unfinished_line_idx, &cp.hit_point, shapes, s2i) as usize;
    (cp.finish_idx as u32, unfinished_idx as u32)
}

// Search for closest projection to wanted distance
/// CutSurface.cpp:1811 / CutSurface.cpp:2023-2038
pub fn get_closest_projection(distance: &ProjectionDistances, wanted_distance: f32) -> Option<&ProjectionDistance> {
    // minimal distance
    let mut min_d: f32 = f32::MAX;
    let mut min_pd: Option<&ProjectionDistance> = None;
    for pd in distance {
        let d: f32 = (pd.distance - wanted_distance).abs();
        // There should be limit for maximal distance
        if min_d > d {
            min_d = d;
            min_pd = Some(pd);
        }
    }
    min_pd
}

// fill result around known index inside one polygon
/// CutSurface.cpp:1814 / CutSurface.cpp:2040-2104
pub fn fill_polygon_distances(pd: &ProjectionDistance, index: u32, id: &ExPolygonsIndex, result: &mut ProjectionDistances, shape: &ExPolygon, distances: &VDistances) {
    let points: &Points = if id.polygon_index == 0 { &shape.contour.points } else { &shape.holes[(id.polygon_index - 1) as usize].points };
    // border of indexes for Polygon
    let first_index: u32 = index.wrapping_sub(id.point_index);
    let last_index: u32 = first_index.wrapping_add(points.len() as u32);

    let mut act_index: u32 = index;
    // `act_pd` points to `pd`, then into `result`; what is read through it is the distance
    let mut act_pd: ProjectionDistance = *pd;

    // Copy starting pd to result
    result[act_index as usize] = *pd;

    fn exist_next(nxt_index: u32, distances: &VDistances, act_index: &mut u32, act_pd: &mut ProjectionDistance, result: &mut ProjectionDistances) -> bool {
        let Some(nxt_pd) = get_closest_projection(&distances[nxt_index as usize], act_pd.distance) else {
            // exist next projection distance ?
            return false;
        };

        // check no rewrite result
        debug_assert!(result[nxt_index as usize].aoi_index == u32::MAX);
        // copy founded projection to result
        result[nxt_index as usize] = *nxt_pd; // copy

        // next
        *act_index = nxt_index;
        *act_pd = result[nxt_index as usize];
        true
    }

    // last index in circle
    let finish_index: u32 = if index == first_index { last_index.wrapping_sub(1) } else { index.wrapping_sub(1) };
    // Positive iteration inside polygon
    loop {
        let mut nxt_index: u32 = act_index.wrapping_add(1);
        // close loop of indexes inside of contour
        if nxt_index == last_index {
            nxt_index = first_index;
        }
        // check that exist next
        if !exist_next(nxt_index, distances, &mut act_index, &mut act_pd, result) {
            break;
        }
        if act_index == finish_index {
            break;
        }
    }

    // when all results for polygon are set no neccessary to iterate negative
    if act_index == finish_index {
        return;
    }

    act_index = index;
    act_pd = *pd;
    // Negative iteration inside polygon
    loop {
        let nxt_index: u32 = if act_index == first_index { last_index.wrapping_sub(1) } else { act_index.wrapping_sub(1) };
        // When iterate negative it must be split to parts
        // and can't iterate in circle
        debug_assert!(nxt_index != index);
        // check that exist next
        if !exist_next(nxt_index, distances, &mut act_index, &mut act_pd, result) {
            break;
        }
    }
}

// search for closest projection for expolygon
// choose correct cut by source point
// IMPROVE: when select distance fill in all distances from Patch
/// CutSurface.cpp:1818 / CutSurface.cpp:2107-2177
pub fn fill_shape_distances(mut start_index: u32, start_pd: &ProjectionDistance, result: &mut ProjectionDistances, s2i: &ExPolygonsIndices, shape: &ExPolygon, distances: &VDistances) {
    let expolygons_index: u32 = s2i.cvt_index(start_index).expolygons_index;
    let first_shape_index: u32 = s2i.cvt(&ExPolygonsIndex { expolygons_index, polygon_index: 0, point_index: 0 });
    loop {
        fill_polygon_distances(start_pd, start_index, &s2i.cvt_index(start_index), result, shape, distances);
        // seaching only inside shape, return index of closed finished point
        let find_close_finished_point = |result: &ProjectionDistances, p: &Point| -> ClosePoint {
            let mut index: u32 = first_shape_index;
            let mut cp = ClosePoint::default();
            let mut check_finished_points = |pts: &Points| {
                for p_ in pts {
                    // finished point with some distances
                    if result[index as usize].aoi_index == u32::MAX {
                        index += 1;
                        continue;
                    }
                    let distance: f32 = calc_size_sq(&Point::new(p_.x - p.x, p_.y - p.y));
                    if cp.dist_sq > distance {
                        cp.dist_sq = distance;
                        cp.index = index;
                    }
                    index += 1;
                }
            };
            check_finished_points(&shape.contour.points);
            for h in &shape.holes {
                check_finished_points(&h.points);
            }
            cp
        };

        // find next closest pair of points
        // (finished + unfinished) in ExPolygon
        start_index = u32::MAX; // unfinished_index
        let mut finished_index: u32 = u32::MAX;
        let mut dist_sq: f32 = f32::MAX;

        // first index in shape
        let mut index: u32 = first_shape_index;
        let mut check_unfinished_points = |pts: &Points| {
            for p in pts {
                // try find unfinished
                if result[index as usize].aoi_index != u32::MAX || distances[index as usize].is_empty() {
                    index += 1;
                    continue;
                }
                let cp: ClosePoint = find_close_finished_point(result, p);
                if dist_sq > cp.dist_sq {
                    dist_sq = cp.dist_sq;
                    start_index = index;
                    finished_index = cp.index;
                }
                index += 1;
            }
        };
        // for each unfinished points
        check_unfinished_points(&shape.contour.points);
        for h in &shape.holes {
            check_unfinished_points(&h.points);
        }
        let _ = finished_index;
        if start_index == u32::MAX {
            break;
        }
    }
}

// find close points between finished and unfinished ExPolygons
/// CutSurface.cpp:1821 / CutSurface.cpp:2179-2215
pub fn find_close_point(p: &Point, result: &ProjectionDistances, finished_shapes: &[bool], s2i: &ExPolygonsIndices, shapes: &ExPolygons) -> ClosePoint {
    // result
    let mut cp = ClosePoint::default();
    // for all finished points
    for shape_index in 0..shapes.len() as u32 {
        if !finished_shapes[shape_index as usize] {
            continue;
        }
        let shape = &shapes[shape_index as usize];
        let mut index: u32 = s2i.cvt(&ExPolygonsIndex { expolygons_index: shape_index, polygon_index: 0, point_index: 0 });
        let mut find_close_point_in_points = |pts: &Points| {
            for p_ in pts {
                // Exist result (is finished) ?
                if result[index as usize].aoi_index == u32::MAX {
                    index += 1;
                    continue;
                }
                let distance_sq: f32 = calc_size_sq(&Point::new(p.x - p_.x, p.y - p_.y));
                if cp.dist_sq > distance_sq {
                    cp.dist_sq = distance_sq;
                    cp.index = index;
                }
                index += 1;
            }
        };
        find_close_point_in_points(&shape.contour.points);
        // shape could be inside of another shape's hole
        for h in &shape.holes {
            find_close_point_in_points(&h.points);
        }
    }
    cp
}

/// Select distances in similar depth between expolygons
/// IMPROVE: when select distance fill in all distances from Patch
/// CutSurface.cpp:441-446 / CutSurface.cpp:2218-2319
pub fn choose_best_distance(distances: &VDistances, shapes: &ExPolygons, start: &Point, s2i: &ExPolygonsIndices, patches: &SurfacePatches) -> ProjectionDistances {
    debug_assert!(distances.len() == crate::geometry::count_points(shapes));

    // vector of patches for shape
    let mut shapes_patches: Vec<Vec<u32>> = vec![Vec::new(); shapes.len()];
    for (i, patch) in patches.iter().enumerate() {
        shapes_patches[patch.shape_id].push(i as u32);
    }

    // collect one closest projection for each outline point
    let mut result: ProjectionDistances = vec![ProjectionDistance::default(); distances.len()];

    // store info about finished shapes
    let mut finished_shapes: Vec<bool> = vec![false; shapes.len()];

    // wanted distance from ideal projection
    // Distances are relative to projection distance
    // so first wanted distance is the closest one (ZERO)
    let mut wanted_distance: f32 = 0.;

    let mut mask_distances: Vec<bool> = vec![true; s2i.get_count() as usize];
    for (i, d) in distances.iter().enumerate() {
        if d.is_empty() {
            mask_distances[i] = false;
        }
    }

    // Select point from shapes(text contour) which is closest to center (all in 2d)
    let mut unfinished_index: u32 = find_closest_point_index(start, shapes, s2i, &mask_distances);
    debug_assert!(unfinished_index < s2i.get_count());
    if unfinished_index >= s2i.get_count() {
        // no point to select
        return result;
    }

    loop {
        let pd: Option<&ProjectionDistance> = get_closest_projection(&distances[unfinished_index as usize], wanted_distance);
        // selection of closest_id should proove that pd has value
        // (functions: get_closest_point_index and find_close_point_in_points)
        debug_assert!(pd.is_some());
        let expolygons_index: u32 = s2i.cvt_index(unfinished_index).expolygons_index;
        let shape = &shapes[expolygons_index as usize];
        let shape_patches = &shapes_patches[expolygons_index as usize];
        if shape_patches.len() == 1 {
            // Speed up, only one patch so copy distance from patch
            let first_shape_index: u32 = s2i.cvt(&ExPolygonsIndex { expolygons_index, polygon_index: 0, point_index: 0 });
            let laset_shape_index: u32 = first_shape_index + crate::geometry::count_points_expoly(shape) as u32;
            for i in first_shape_index..laset_shape_index {
                let pds = &distances[i as usize];
                if pds.is_empty() {
                    continue;
                }
                // check that index belongs to patch
                debug_assert!(pds[0].patch_index == shape_patches[0]);
                result[i as usize] = pds[0];
                if pds.len() == 1 {
                    continue;
                }

                // `pd->distance`: the C++ dereferences the pointer
                let pd_distance = pd.expect("pd is nullptr").distance;
                let mut relative_distance: f32 = (result[i as usize].distance - pd_distance).abs();
                // patch could contain multiple value for one outline point
                // so choose closest to start point
                for pds_index in 1..pds.len() {
                    // check that index still belongs to same patch
                    debug_assert!(pds[pds_index].patch_index == shape_patches[0]);
                    let relative_distance2: f32 = (pds[pds_index].distance - pd_distance).abs();
                    if relative_distance > relative_distance2 {
                        relative_distance = relative_distance2;
                        result[i as usize] = pds[pds_index];
                    }
                }
            }
        } else {
            // multiple patches for expolygon
            // check that exist patch to fill shape
            debug_assert!(!shape_patches.is_empty());
            fill_shape_distances(unfinished_index, pd.expect("pd is nullptr"), &mut result, s2i, shape, distances);
        }

        finished_shapes[expolygons_index as usize] = true;
        // The most close points between finished and unfinished shapes
        let (finished, unfinished) = find_closest_point_pair(shapes, &finished_shapes, s2i, &mask_distances);

        // detection of end (best doesn't have value)
        if finished == u32::MAX {
            break;
        }

        debug_assert!(unfinished != u32::MAX);
        let closest_pd = &result[finished as usize];
        // check that best_cp is finished and has result
        debug_assert!(closest_pd.aoi_index != u32::MAX);
        wanted_distance = closest_pd.distance;
        unfinished_index = unfinished;
    }
    result
}

// ===========================================================================
// functions to help 'diff_model' (CutSurface.cpp:2321-2460)
// ===========================================================================

/// CutSurface.cpp:2323  `const VI default_vi(std::numeric_limits<uint32_t>::max());`
pub const default_vi: VI = surface_mesh::SM_Vertex_index(u32::MAX);

// Keep info about intersection source
/// CutSurface.cpp:2326  `struct Source{ HI  hi; int sdim=0;};`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Source {
    pub hi: HI,
    pub sdim: i32,
}
/// CutSurface.cpp:2327
pub type Sources = Vec<Source>;
/// CutSurface.cpp:2328
pub const vertex_source_map_name: &str = "v:SourceIntersecting";
/// CutSurface.cpp:2329
pub type VertexSourceMap = surface_mesh::Property_map<VI, Source>;

/// Corefine visitor
/// Store intersection source for vertices of constrained edge of tm1
/// Must be used with corefine flag no modification of tm2
/// CutSurface.cpp:2336-2418
///
/// `patch` and `model` are pointers in C++, compared with the addresses of the meshes a callback
/// is given; `sources` is a pointer to a vector the caller owns.
pub struct IntersectionSources<'a> {
    pub patch: *const CutMesh, // patch
    pub model: *const CutMesh, // const model

    pub vmap: VertexSourceMap,

    // keep sources from call intersection_point_detected
    // until call new_vertex_added
    pub sources: &'a RefCell<Sources>,
}

impl<'a> CorefinementVisitor for IntersectionSources<'a> {
    // count intersections
    /// CutSurface.cpp:2348-2390
    fn intersection_point_detected(&mut self, i_id: usize, sdim: i32, h_f: HI, h_e: HI, tm_f: &CutMesh, tm_e: &CutMesh, is_target_coplanar: bool, is_source_coplanar: bool) {
        let mut source: Source;
        if std::ptr::eq(tm_e, self.model) {
            source = Source { hi: h_e, sdim };
            // check other CGAL model that is patch
            debug_assert!(std::ptr::eq(tm_f, self.patch));
            if is_target_coplanar {
                debug_assert!(sdim == 0);
                self.vmap.set(tm_f.source(h_f), source);
            }
            if is_source_coplanar {
                debug_assert!(sdim == 0);
                self.vmap.set(tm_f.target(h_f), source);
            }

            // clear source to be able check that this intersection source is
            // not used any more
            if is_source_coplanar || is_target_coplanar {
                source = Source::default();
            }
        } else {
            source = Source { hi: h_f, sdim };
            debug_assert!(std::ptr::eq(tm_f, self.model) && std::ptr::eq(tm_e, self.patch));
            debug_assert!(!is_target_coplanar);
            debug_assert!(!is_source_coplanar);
            // if (is_target_coplanar) vmap[tm_e.source(h_e)] = source;
            // if (is_source_coplanar) vmap[tm_e.target(h_e)] = source;
            // if (sdim == 0)
            //     vmap[tm_e.target(h_e)] = source;
        }

        // By documentation i_id is consecutive.
        // check id goes in a row, without skips
        debug_assert!(self.sources.borrow().len() == i_id);
        let _ = i_id;
        // add source of intersection
        self.sources.borrow_mut().push(source);
    }

    /// Store VI to intersections by i_id
    /// CutSurface.cpp:2398-2407
    fn new_vertex_added(&mut self, i_id: usize, v: VI, _tm: &CutMesh) {
        // check that it is first insertation into item of vmap
        debug_assert!(!self.vmap.get(v).hi.is_valid());
        // check valid addresing into sources
        debug_assert!(i_id < self.sources.borrow().len());
        // check that source has value
        debug_assert!(self.sources.borrow()[i_id].hi.is_valid());
        // `sources->at(i_id)`: `at` throws out of range, the index panics
        let source = self.sources.borrow()[i_id];
        self.vmap.set(v, source);
    }

    // Not used visitor functions
    /// CutSurface.cpp:2410
    fn before_subface_creations(&mut self, _f_old: FI, _mesh: &CutMesh) {}
    /// CutSurface.cpp:2411
    fn after_subface_created(&mut self, _f_new: FI, _mesh: &CutMesh) {}
    /// CutSurface.cpp:2412
    fn after_subface_creations(&mut self, _mesh: &CutMesh) {}
    /// CutSurface.cpp:2413
    fn before_subface_created(&mut self, _mesh: &CutMesh) {}
    /// CutSurface.cpp:2414
    fn before_edge_split(&mut self, _h: HI, _tm: &CutMesh) {}
    /// CutSurface.cpp:2415
    fn edge_split(&mut self, _hnew: HI, _tm: &CutMesh) {}
    /// CutSurface.cpp:2416
    fn after_edge_split(&mut self) {}
    /// CutSurface.cpp:2417
    fn add_retriangulation_edge(&mut self, _h: HI, _tm: &CutMesh) {}
}

/// Create map1 and map2
/// CutSurface.cpp:2428-2432 / CutSurface.cpp:2462-2555
///
/// map .. Convert tm1.face to type; tm1 .. Corefined mesh; tm2 .. Source of intersection;
/// ecm .. Identify constrainde edge; sources .. Convert tm1.face to type
pub fn create_face_types(map: &FaceTypeMap, tm1: &CutMesh, tm2: &CutMesh, ecm: &EdgeBoolMap, sources: &VertexSourceMap) {
    let get_intersection_source = |s1: &Source, s2: &Source| -> FI {
        // when one of sources is face than return it
        let fi1 = tm2.face(s1.hi);
        if s1.sdim == 2 {
            return fi1;
        }
        let fi2 = tm2.face(s2.hi);
        if s2.sdim == 2 {
            return fi2;
        }
        // both vertices are made by same source triangle
        if fi1 == fi2 {
            return fi1;
        }

        // when one from sources is edge second one decide side of triangle triangle
        let hi1_opposit = tm2.opposite(s1.hi);
        let mut fi1_opposit = FI::null();
        if hi1_opposit.is_valid() {
            fi1_opposit = tm2.face(hi1_opposit);
        }
        if fi2 == fi1_opposit {
            return fi2;
        }

        let hi2_opposit = tm2.opposite(s2.hi);
        let mut fi2_opposit = FI::null();
        if hi2_opposit.is_valid() {
            fi2_opposit = tm2.face(hi2_opposit);
        }
        if fi1 == fi2_opposit {
            return fi1;
        }
        if fi1_opposit.is_valid() && fi1_opposit == fi2_opposit {
            return fi1_opposit;
        }

        // when intersection is vertex need loop over neighbor
        for fi_around_hi1 in tm2.faces_around_target(s1.hi) {
            for fi_around_hi2 in tm2.faces_around_target(s2.hi) {
                if fi_around_hi1 == fi_around_hi2 {
                    return fi_around_hi1;
                }
            }
        }

        // should never rich it
        // Exist case when do not know source triangle for decide side of intersection
        debug_assert!(false);
        FI::null()
    };

    for fi in tm1.faces() {
        map.set(fi, FaceType::not_constrained);
    }
    for ei1 in tm1.edges() {
        if !ecm.get(ei1) {
            continue;
        }

        // get faces from tm1 (f1a + f1b)
        let hi1 = tm1.halfedge(ei1);
        debug_assert!(hi1.is_valid());
        let f1a = tm1.face(hi1);
        debug_assert!(f1a.is_valid());
        let hi_op = tm1.opposite(hi1);
        debug_assert!(hi_op.is_valid());
        let f1b = tm1.face(hi_op);
        debug_assert!(f1b.is_valid());

        // get faces from tm2 (f2a + f2b)
        let vi1_source = tm1.source(hi1);
        debug_assert!(vi1_source.is_valid());
        let vi1_target = tm1.target(hi1);
        debug_assert!(vi1_target.is_valid());

        let s_s: Source = sources.get(vi1_source);
        let s_t: Source = sources.get(vi1_target);
        let fi2 = get_intersection_source(&s_s, &s_t);

        // in release solve situation that face was NOT deduced
        if !fi2.is_valid() {
            continue;
        }

        let hi2 = tm2.halfedge(fi2);
        let mut t: [P3; 3] = [[0.0; 3]; 3];
        let mut ti: usize = 0;
        for vi2 in tm2.vertices_around_face(hi2) {
            t[ti] = tm2.point(vi2);
            ti += 1;
        }

        // triangle tip from face f1a
        let vi1a_tip = tm1.target(tm1.next(hi1));
        debug_assert!(vi1a_tip.is_valid());
        let p: P3 = tm1.point(vi1a_tip);

        // check if f1a is behinde f2a
        // inside mean it will be used
        // outside will be discarded
        // CGAL::orientation(*t[0], *t[1], *t[2], p) == CGAL::POSITIVE
        if orientation_3(&t[0], &t[1], &t[2], &p) == Ordering::Greater {
            map.set(f1a, FaceType::inside);
            map.set(f1b, FaceType::outside);
        } else {
            map.set(f1a, FaceType::outside);
            map.set(f1b, FaceType::inside);
        }
    }
}

/// Implement 'cut' Minus 'clipper', where clipper is reverse input Volume
/// NOTE: clipper will be modified (corefined by cut) !!!
/// CutSurface.cpp:2441 / CutSurface.cpp:2559-2608
///
/// `clipper` is taken by value as in C++ (the caller hands a copy). True on succes, otherwise
/// FALSE.
pub fn clip_cut(cut: &mut SurfacePatch, mut clipper: CutMesh, pmp: &dyn PolygonMeshProcessing) -> bool {
    // CutMesh& tm = cut.mesh;
    // create backup for case that there is no intersection
    let backup_copy: CutMesh = cut.mesh.clone();

    // CutSurface.cpp:2565-2572
    struct ExistIntersectionClipVisitor<'a> {
        exist_intersection: &'a Cell<bool>,
    }
    impl<'a> CorefinementVisitor for ExistIntersectionClipVisitor<'a> {
        fn intersection_point_detected(&mut self, _i_id: usize, _sdim: i32, _h_f: HI, _h_e: HI, _tm_f: &CutMesh, _tm_e: &CutMesh, _is_target_coplanar: bool, _is_source_coplanar: bool) {
            self.exist_intersection.set(true);
        }
    }
    let exist_intersection = Cell::new(false);
    let mut visitor = ExistIntersectionClipVisitor { exist_intersection: &exist_intersection };

    #[allow(unused_assignments)]
    let mut suc = false;
    {
        // namep parameters for model tm and function clip
        let np_tm = NamedParameters::default_values().visitor(&mut visitor).throw_on_self_intersection(false);

        // name parameters for model clipper and function clip
        let np_c = NamedParameters::default_values().throw_on_self_intersection(false);
        // Can't use 'do_not_modify', when Ture than clipper has to be closed !!
        // .do_not_modify(true);
        // .throw_on_self_intersection(false); is set automaticaly by param 'do_not_modify'
        // .clip_volume(false); is set automaticaly by param 'do_not_modify'

        match pmp.clip(&mut cut.mesh, &mut clipper, np_tm, np_c) {
            Ok(s) => suc = s,
            Err(e) => {
                // Degenerated input (e.g. intersecting constraints inside CDT) makes CGAL throw.
                // Keep the patch unclipped instead of crashing the whole job.
                log::warn!("CGAL clip failed: {}", e);
                cut.mesh = backup_copy;
                return false;
            }
        }
    }

    // true if the output surface mesh is manifold.
    // If false is returned tm and clipper are only corefined.
    debug_assert!(suc);
    // decide what TODO when can't clip source object !?!
    if !exist_intersection.get() || !suc {
        // TODO: test if cut is fully in or fully out!!
        cut.mesh = backup_copy;
        return false;
    }
    true
}

/// CutSurface.cpp:2443 / CutSurface.cpp:2610-2624  `bounding_box(const CutAOI &cut, const CutMesh &mesh)`
pub fn bounding_box(cut: &CutAOI, mesh: &CutMesh) -> BoundingBoxf3 {
    // cut.first.front()
    let p_from_cut: P3 = mesh.point(mesh.target(mesh.halfedge(cut.0[0])));
    let mut min: P3 = p_from_cut;
    let mut max: P3 = min;
    for &fi in &cut.0 {
        for vi in mesh.vertices_around_face(mesh.halfedge(fi)) {
            let p: P3 = mesh.point(vi);
            for i in 0..3 {
                if min[i] > p[i] {
                    min[i] = p[i];
                }
                if max[i] < p[i] {
                    max[i] = p[i];
                }
            }
        }
    }
    BoundingBoxf3::new(to_vec3d(&min), to_vec3d(&max))
}

/// CutSurface.cpp:2626-2638, the overload of `bounding_box` over a mesh
pub mod of_mesh {
    use super::*;

    /// CutSurface.cpp:2444 / CutSurface.cpp:2626-2638  `bounding_box(const CutMesh &mesh)`
    pub fn bounding_box(mesh: &CutMesh) -> BoundingBoxf3 {
        // *mesh.points().begin(): the first value of the property "v:point"
        let mut min: P3 = mesh.points().get_at(0);
        let mut max: P3 = min;
        for vi in mesh.vertices() {
            let p: P3 = mesh.point(vi);
            for i in 0..3 {
                if min[i] > p[i] {
                    min[i] = p[i];
                }
                if max[i] < p[i] {
                    max[i] = p[i];
                }
            }
        }
        BoundingBoxf3::new(to_vec3d(&min), to_vec3d(&max))
    }
}

/// CutSurface.cpp:2640-2642 and 3137-3146, the overloads over a patch
pub mod of_patch {
    use super::*;

    /// CutSurface.cpp:2445 / CutSurface.cpp:2640-2642  `bounding_box(const SurfacePatch &ecut)`
    pub fn bounding_box(ecut: &SurfacePatch) -> BoundingBoxf3 {
        of_mesh::bounding_box(&ecut.mesh)
    }

    /// Checking whether patch is uninterrupted cover of whole expolygon it belongs.
    /// CutSurface.cpp:370-373 / CutSurface.cpp:3137-3146
    pub fn is_over_whole_expoly(patch: &SurfacePatch, shapes: &ExPolygons, cutAOIs: &VCutAOIs, meshes: &CutMeshes) -> bool {
        if !patch.is_whole_aoi {
            return false;
        }
        super::is_over_whole_expoly(&cutAOIs[patch.model_id][patch.aoi_id], &shapes[patch.shape_id], &meshes[patch.model_id])
    }
}

/// Create patch
/// CutSurface.cpp:2456-2458 / CutSurface.cpp:2644-2740
///
/// fis .. Define patch faces; mesh .. Source of fis (NOTE: Need temporary add property map for
/// convert vertices); rmap .. Options to reduce vertices from fis (used for skip vertices made by
/// diagonal edge in rectangle of shape side)
pub fn create_surface_patch(fis: &[FI], mesh: &mut CutMesh, rmap: Option<&ReductionMap>) -> SurfacePatch {
    let mut is_counted: surface_mesh::Property_map<VI, bool> = mesh.add_property_map("v:is_counted", false).0;
    let mut count_vertices: u32 = 0;
    match rmap {
        None => {
            for &fi in fis {
                for vi in mesh.vertices_around_face(mesh.halfedge(fi)) {
                    if !is_counted.get(vi) {
                        is_counted.set(vi, true);
                        count_vertices = count_vertices.wrapping_add(1);
                    }
                }
            }
        }
        Some(rmap) => {
            for &fi in fis {
                for vi in mesh.vertices_around_face(mesh.halfedge(fi)) {
                    // Will vertex be reduced?
                    if rmap.get(vi).is_valid() {
                        continue;
                    }
                    if !is_counted.get(vi) {
                        is_counted.set(vi, true);
                        count_vertices = count_vertices.wrapping_add(1);
                    }
                }
            }
        }
    }
    mesh.remove_property_map(&mut is_counted);

    let count_faces: u32 = fis.len() as u32;
    // IMPROVE: Value is greater than neccessary, count edges used twice
    let count_edges: u32 = count_faces.wrapping_mul(3);

    let mut cm = CutMesh::new();
    cm.reserve(count_vertices, count_edges, count_faces);

    // vertex conversion function from mesh VI to result VI
    let mut mesh2result: CvtVI2VI = mesh.add_property_map("v:mesh2result", VI::null()).0;

    match rmap {
        None => {
            for &fi in fis {
                let mut t: [VI; 3] = [VI::null(); 3];
                let mut index: usize = 0;
                for vi in mesh.vertices_around_face(mesh.halfedge(fi)) {
                    let mut vi_cvt = mesh2result.get(vi);
                    if !vi_cvt.is_valid() {
                        vi_cvt = VI::new(cm.vertices().count() as u32);
                        mesh2result.set(vi, vi_cvt);
                        cm.add_vertex(mesh.point(vi));
                    }
                    t[index] = vi_cvt;
                    index += 1;
                }
                cm.add_face(t[0], t[1], t[2]);
            }
        }
        Some(rmap) => {
            for &fi in fis {
                let mut t: [VI; 3] = [VI::null(); 3];
                let mut index: usize = 0;
                let mut exist_reduction = false;
                for mut vi in mesh.vertices_around_face(mesh.halfedge(fi)) {
                    let vi_r = rmap.get(vi);
                    if vi_r.is_valid() {
                        exist_reduction = true;
                        vi = vi_r;
                    }
                    let mut vi_cvt = mesh2result.get(vi);
                    if !vi_cvt.is_valid() {
                        vi_cvt = VI::new(cm.vertices().count() as u32);
                        mesh2result.set(vi, vi_cvt);
                        cm.add_vertex(mesh.point(vi));
                    }
                    t[index] = vi_cvt;
                    index += 1;
                }

                // prevent add reduced triangle
                if exist_reduction && (t[0] == t[1] || t[1] == t[2] || t[2] == t[0]) {
                    continue;
                }

                cm.add_face(t[0], t[1], t[2]);
            }
        }
    }

    debug_assert!(count_vertices as usize == cm.vertices().count());
    debug_assert!((rmap.is_none() && count_faces as usize == cm.faces().count()) || (rmap.is_some() && count_faces as usize >= cm.faces().count()));
    debug_assert!(count_edges as usize >= cm.edges().count());

    // convert VI from this patch to source VI, when exist
    let cvt: CvtVI2VI = cm.add_property_map(patch_source_name, VI::null()).0;
    // vi_s .. VertexIndex into mesh (source)
    // vi_d .. new VertexIndex in cm (destination)
    for vi_s in mesh.vertices() {
        let vi_d = mesh2result.get(vi_s);
        if !vi_d.is_valid() {
            continue;
        }
        cvt.set(vi_d, vi_s);
    }
    mesh.remove_property_map(&mut mesh2result);
    // return {std::move(cm)};
    SurfacePatch { mesh: cm, ..SurfacePatch::default() }
}

// ===========================================================================
// diff_models help functions (CutSurface.cpp:2742-2841)
// ===========================================================================

/// CutSurface.cpp:2745-2753
#[derive(Clone, Default)]
pub struct SurfacePatchEx {
    pub patch: SurfacePatch,

    // flag that part will be deleted
    pub full_inside: bool,
    // flag that Patch could contain more than one part
    pub just_cliped: bool,
}
/// CutSurface.cpp:2754
pub type SurfacePatchesEx = Vec<SurfacePatchEx>;

/// CutSurface.cpp:2757
pub type BBS = Vec<BoundingBoxf3>;

/// Create bounding boxes for AOI
/// CutSurface.cpp:2764 / CutSurface.cpp:2843-2860
pub fn create_bbs(cuts: &VCutAOIs, cut_models: &CutMeshes) -> BBS {
    let mut count: usize = 0;
    for cut in cuts {
        count += cut.len();
    }

    let mut bbs: BBS = Vec::with_capacity(count);
    for model_index in 0..cut_models.len() {
        let cut_model = &cut_models[model_index];
        let cutAOIs = &cuts[model_index];
        for cut_index in 0..cutAOIs.len() {
            let cut = &cutAOIs[cut_index];
            bbs.push(bounding_box(cut, cut_model));
        }
    }
    bbs
}

/// Create AABB trees for check when patch is whole inside of model
/// CutSurface.cpp:2776 / CutSurface.cpp:2863-2873
pub fn create_trees(models: &CutMeshes) -> Trees {
    let mut result: Trees = Vec::with_capacity(models.len());
    for model in models {
        let mut tree = Tree::new();
        tree.insert(model.faces(), model);
        tree.build();
        result.push(tree);
    }
    result
}

/// Check whether bounding box has intersection with model
/// CutSurface.cpp:2786-2789 / CutSurface.cpp:2875-2888
pub fn has_bb_intersection(bb: &BoundingBoxf3, model_index: usize, bbs: &BBS, m2i: &ModelCut2index) -> bool {
    let offsets = m2i.get_offsets();
    // for cut index with model_index2
    let start: usize = offsets[model_index] as usize;
    let next: usize = model_index + 1;
    let end: usize = if next < offsets.len() { offsets[next] as usize } else { m2i.get_count() as usize };
    for bb_index in start..end {
        if bb.intersects(&bbs[bb_index]) {
            return true;
        }
    }
    false
}

/// Only for model without intersection
/// Use ray (in projection direction) from a point from patch
/// and count intersections: pair .. outside | odd .. inside
/// CutSurface.cpp:2801-2803 / CutSurface.cpp:2890-2916
pub fn is_patch_inside_of_model(patch: &SurfacePatch, tree: &Tree, projection: &dyn IProject3d) -> bool {
    // TODO: Solve model with hole in projection direction !!!
    let a: P3 = patch.mesh.point(VI::new(0));
    let a_ = to_vec3d(&a);
    let b_ = projection.project(&a_);
    let b: P3 = [b_.x, b_.y, b_.z];

    let ray_query = Ray::new(a, b);
    let count: usize = tree.number_of_intersected_primitives(&ray_query);
    let is_in = (count % 2) == 1;

    // try opposit direction result should be same, otherwise open model is used
    debug_assert!(((tree.number_of_intersected_primitives(&Ray::new(a, [2. * a[0] - b[0], 2. * a[1] - b[1], 2. * a[2] - b[2]])) % 2) == 1) == is_in);
    is_in
}

/// Return some shape point index which identify shape
/// NOTE: Used to find expolygon index
/// CutSurface.cpp:2812 / CutSurface.cpp:2918-2933
pub fn get_shape_point_index(cut: &CutAOI, model: &CutMesh) -> u32 {
    // map is created during intersection by corefine visitor
    let vert_shape_map: VertexShapeMap = model.property_map(vert_shape_map_name).0;
    // for each half edge of outline
    for &hi in &cut.1 {
        let vi = model.source(hi);
        let Some(ie) = vert_shape_map.get(vi) else {
            continue;
        };
        debug_assert!(ie.shape_point_index != u32::MAX);
        return ie.shape_point_index;
    }
    // can't found any intersecting element in cut
    debug_assert!(false);
    0
}

/// CutSurface.cpp:2814
pub type PatchNumber = surface_mesh::Property_map<FI, usize>;

/// Separate triangles singned with number n
/// CutSurface.cpp:2823-2825 / CutSurface.cpp:2935-2952
///
/// fis .. Face indices owned by separate patch; patch .. Original patch (NOTE: Can't be const.
/// For indexing vetices need temporary add property map); cvt_from .. conversion map
pub fn separate_patch(fis: &[FI], patch: &mut SurfacePatch, cvt_from: &CvtVI2VI) -> SurfacePatch {
    debug_assert!(patch.mesh.is_valid(false));
    let mut patch_new = create_surface_patch(fis, &mut patch.mesh, None);
    patch_new.bb = of_mesh::bounding_box(&patch_new.mesh);
    patch_new.aoi_id = patch.aoi_id;
    patch_new.model_id = patch.model_id;
    patch_new.shape_id = patch.shape_id;
    // fix cvt
    let cvt: CvtVI2VI = patch_new.mesh.property_map(patch_source_name).0;
    // for (VI &vi : cvt): every value of the property array
    for pos in 0..cvt.len() {
        let vi = cvt.get_at(pos);
        if !vi.is_valid() {
            continue;
        }
        cvt.set_at(pos, cvt_from.get(vi));
    }
    patch_new
}

/// Separate connected triangles into it's own patches
/// new patches are added to back of input patches
/// CutSurface.cpp:2833 / CutSurface.cpp:2954-3013
pub fn divide_patch(i: usize, patches: &mut SurfacePatchesEx) {
    {
        let patch_ex = &mut patches[i];
        debug_assert!(patch_ex.just_cliped);
        patch_ex.just_cliped = false;
    }

    let mut new_patches = SurfacePatchesEx::new();
    {
        let patch: &mut SurfacePatch = &mut patches[i].patch;
        debug_assert!(patch.mesh.faces().next().is_some());
        let patch_number_name = "f:patch_number";
        let mut is_processed: surface_mesh::Property_map<FI, bool> = patch.mesh.add_property_map(patch_number_name, false).0;

        let cvt_from: CvtVI2VI = patch.mesh.property_map(patch_source_name).0;

        let mut fis: Vec<FI> = Vec::with_capacity(patch.mesh.faces().count());

        let mut queue: Vec<FI> = Vec::new();
        // IMPROVE: create groups around triangles and than connect groups
        // (`separate_patch` adds and removes properties of the vertices of `cm` inside the loop;
        // the faces it iterates stay as they are)
        let cm_faces: Vec<FI> = patch.mesh.faces().collect();
        for fi_cm in cm_faces {
            if is_processed.get(fi_cm) {
                continue;
            }
            debug_assert!(queue.is_empty());
            queue.push(fi_cm);
            if !fis.is_empty() {
                // Be carefull after push to patches,
                // all ref on patch contain non valid values
                let mut patch_ex_n = SurfacePatchEx::default();
                patch_ex_n.patch = separate_patch(&fis, patch, &cvt_from);
                patch_ex_n.patch.is_whole_aoi = false;
                new_patches.push(patch_ex_n);
                fis.clear();
            }
            // flood fill from triangle fi_cm to surrounding
            loop {
                let fi_q = queue.pop().expect("queue.back()");
                // `continue` of the do-while goes to its condition
                if !is_processed.get(fi_q) {
                    is_processed.set(fi_q, true);
                    fis.push(fi_q);
                    let cm = &patch.mesh;
                    let hi = cm.halfedge(fi_q);
                    for fi in cm.faces_around_face(hi) {
                        // by documentation The face descriptor may be the null face, and it may be several times the same face descriptor.
                        if !fi.is_valid() {
                            continue;
                        }
                        if !is_processed.get(fi) {
                            queue.push(fi);
                        }
                    }
                }
                if queue.is_empty() {
                    break;
                }
            }
        }
        patch.mesh.remove_property_map(&mut is_processed);
        debug_assert!(!fis.is_empty());

        // speed up for only one patch - no dividing (the most common)
        if new_patches.is_empty() {
            patch.bb = of_mesh::bounding_box(&patch.mesh);
            patch.is_whole_aoi = false;
        } else {
            *patch = separate_patch(&fis, patch, &cvt_from);
        }
    }
    if !new_patches.is_empty() {
        // patches.insert(patches.end(), new_patches.begin(), new_patches.end());
        patches.extend(new_patches);
    }
}

/// Fill outline in patches by open edges
/// CutSurface.cpp:2839 / CutSurface.cpp:3015-3038
pub fn collect_open_edges(patches: &mut SurfacePatches) {
    let mut open_half_edges: Vec<HI> = Vec::new();
    for patch in patches.iter_mut() {
        open_half_edges.clear();
        let mesh = &patch.mesh;
        for fi in mesh.faces() {
            let hi1 = mesh.halfedge(fi);
            debug_assert!(hi1.is_valid());
            let hi2 = mesh.next(hi1);
            debug_assert!(hi2.is_valid());
            let hi3 = mesh.next(hi2);
            debug_assert!(hi3.is_valid());
            // Is fi triangle?
            debug_assert!(mesh.next(hi3) == hi1);
            for hi in [hi1, hi2, hi3] {
                let hi_op = mesh.opposite(hi);
                let fi_op = mesh.face(hi_op);
                if !fi_op.is_valid() {
                    open_half_edges.push(hi);
                }
            }
        }
        patch.loops = create_loops(&open_half_edges, mesh);
    }
}

/// Differenciate other models
/// CutSurface.cpp:348-351 / CutSurface.cpp:3040-3135
///
/// cuts .. Patches from meshes; cut_models .. Source points for Cutted AOIs (NOTE: Create
/// Reduction map as mesh property - clean on end); models .. Original models without cut
/// modifications used for differenciation; projection .. Define projection direction
pub fn diff_models(cuts: &mut VCutAOIs, cut_models: &mut CutMeshes, models: &mut CutMeshes, projection: &dyn IProject3d, pmp: &dyn PolygonMeshProcessing) -> SurfacePatches {
    // IMPROVE: when models contain ONE mesh. It is only about convert cuts to patches
    // and reduce unneccessary triangles on contour

    //Convert model_index and cut_index into one index
    let m2i = ModelCut2index::new(cuts);

    // create bounding boxes for cuts
    let bbs: BBS = create_bbs(cuts, cut_models);
    let mut trees: Trees = vec![Tree::new(); models.len()];

    let mut patches = SurfacePatches::new();

    // queue of patches for one AOI (permanent with respect to for loop)
    let mut aoi_patches = SurfacePatchesEx::new();

    //SurfacePatches aoi_patches;
    patches.reserve(m2i.get_count() as usize); // only approximation of count
    let mut index: usize = 0;
    for model_index in 0..models.len() {
        let model_cuts: &CutAOIs = &cuts[model_index];
        let cut_model_: &mut CutMesh = &mut cut_models[model_index];
        let mut vertex_reduction_map: ReductionMap = cut_model_.add_property_map(vertex_reduction_map_name, VI::null()).0;
        create_reduce_map(&vertex_reduction_map, cut_model_);

        for cut_index in 0..model_cuts.len() {
            let cut: &CutAOI = &model_cuts[cut_index];
            let mut patch_ex = SurfacePatchEx::default();
            {
                let patch: &mut SurfacePatch = &mut patch_ex.patch;
                *patch = create_surface_patch(&cut.0, cut_model_, Some(&vertex_reduction_map));
                patch.bb = bbs[index];
                patch.aoi_id = cut_index;
                patch.model_id = model_index;
                patch.shape_id = get_shape_point_index(cut, cut_model_) as usize;
                patch.is_whole_aoi = true;
            }

            aoi_patches.clear();
            aoi_patches.push(patch_ex);
            for model_index2 in 0..models.len() {
                // do not clip source model itself
                if model_index == model_index2 {
                    continue;
                }
                for patch_ex in aoi_patches.iter_mut() {
                    let patch: &mut SurfacePatch = &mut patch_ex.patch;
                    if has_bb_intersection(&patch.bb, model_index2, &bbs, &m2i) && clip_cut(patch, models[model_index2].clone(), pmp) {
                        patch_ex.just_cliped = true;
                    } else {
                        // build tree on demand
                        // NOTE: it is possible not neccessary: e.g. one model
                        let tree: &mut Tree = &mut trees[model_index2];
                        if tree.empty() {
                            let model: &CutMesh = &models[model_index2];
                            tree.insert(model.faces(), model);
                            tree.build();
                        }
                        if is_patch_inside_of_model(patch, tree, projection) {
                            patch_ex.full_inside = true;
                        }
                    }
                }
                // erase full inside
                let mut i = aoi_patches.len();
                while i != 0 {
                    if aoi_patches[i - 1].full_inside {
                        aoi_patches.remove(i - 1);
                    }
                    i -= 1;
                }

                // detection of full AOI inside of model
                if aoi_patches.is_empty() {
                    break;
                }

                // divide cliped into parts
                let end = aoi_patches.len();
                for i in 0..end {
                    if aoi_patches[i].just_cliped {
                        divide_patch(i, &mut aoi_patches);
                    }
                }
            }

            if !aoi_patches.is_empty() {
                patches.reserve(aoi_patches.len());
                for patch in aoi_patches.iter_mut() {
                    patches.push(std::mem::take(&mut patch.patch));
                }
            }
            // ++cut_index, ++index
            index += 1;
        }
        cut_model_.remove_property_map(&mut vertex_reduction_map);
    }

    // Also use outline inside of patches(made by non manifold models)
    // IMPROVE: trace outline from AOIs
    collect_open_edges(&mut patches);
    patches
}

/// Checking whether patch is uninterrupted cover of whole expolygon it belongs.
/// CutSurface.cpp:360-362 / CutSurface.cpp:3148-3230
pub fn is_over_whole_expoly(cutAOI: &CutAOI, _shape: &ExPolygon, mesh: &CutMesh) -> bool {
    // NonInterupted contour is without other point and contain all from shape
    let vert_shape_map: VertexShapeMap = mesh.property_map(vert_shape_map_name).0;
    for &hi in &cutAOI.1 {
        let ie_s = vert_shape_map.get(mesh.source(hi));
        let ie_t = vert_shape_map.get(mesh.target(hi));
        let (Some(mut ie_s), Some(mut ie_t)) = (ie_s, ie_t) else {
            return false;
        };

        debug_assert!(ie_s.attr != Type::undefined.0);
        debug_assert!(ie_t.attr != Type::undefined.0);

        // check if it is neighbor indices
        let mut i_s: u32 = ie_s.shape_point_index;
        let mut i_t: u32 = ie_t.shape_point_index;
        debug_assert!(i_s != u32::MAX);
        debug_assert!(i_t != u32::MAX);
        if i_s == u32::MAX || i_t == u32::MAX {
            return false;
        }

        // made by same index
        if i_s == i_t {
            continue;
        }

        // order from source to target
        if i_s > i_t {
            std::mem::swap(&mut i_s, &mut i_t);
            std::mem::swap(&mut ie_s, &mut ie_t);
        }
        // Must be after fix order !!
        let is_last_polygon_segment = ie_s.is_first() && ie_t.is_last();
        if is_last_polygon_segment {
            std::mem::swap(&mut i_s, &mut i_t);
            std::mem::swap(&mut ie_s, &mut ie_t);
        }

        // Is continous indices
        if !is_last_polygon_segment && (ie_s.is_last() || i_s.wrapping_add(1) != i_t) {
            return false;
        }

        let t_s = ie_s.get_type();
        let t_t = ie_t.get_type();
        if t_s == Type::undefined || t_t == Type::undefined {
            return false;
        }

        // next segment must start with edge intersection
        if t_t != Type::edge_1 {
            return false;
        }

        // After face1 must be edge2 or face2
        if t_s == Type::face_1 {
            return false;
        }
    }

    // When all open edges are on contour than there is NO holes is shape
    let is_open = |hi: HI| -> bool {
        let opposite = mesh.opposite(hi);
        !mesh.face(opposite).is_valid()
    };

    let mut opens: Vec<HI> = Vec::with_capacity(cutAOI.1.len()); // copy
    for &hi in &cutAOI.1 {
        // from lower to bigger
        if is_open(hi) {
            opens.push(hi);
        }
    }
    // std::sort of indices: equal ones are the same value
    opens.sort();

    for &fi in &cutAOI.0 {
        let face_hi = mesh.halfedge(fi);
        for hi in mesh.halfedges_around_face(face_hi) {
            if !is_open(hi) {
                continue;
            }
            // open edge
            let lb = opens.partition_point(|x| *x < hi);
            if lb == opens.len() || opens[lb] != hi {
                return false; // not in contour
            }
        }
    }
    true
}

// ClipperUtils.hpp  `ClipperUtils::PolygonsProvider`: every polygon is one path.
fn polygons_provider(polygons: &[Polygon]) -> crate::clipper::Paths {
    polygons.iter().map(|p| path_of_points(p.points())).collect()
}

// ClipperUtils.hpp  `ClipperUtils::ExPolygonsProvider`: the contour, then the holes, of every
// expolygon.
fn expolygons_provider(expolygons: &[ExPolygon]) -> crate::clipper::Paths {
    let mut paths = crate::clipper::Paths::new();
    for e in expolygons {
        paths.push(path_of_points(e.contour.points()));
        for h in &e.holes {
            paths.push(path_of_points(h.points()));
        }
    }
    paths
}

// The engine builds ClipperLib with `CLIPPERLIB_INT32`: `cInt` is 32 bits wide.
fn path_of_points(points: &[Point]) -> crate::clipper::Path {
    points.iter().map(|p| crate::clipper::IntPoint::new(p.x as i32, p.y as i32)).collect()
}

// ClipperUtils.cpp:567-569  `ExPolygons offset_ex(const ExPolygon &expolygon, const float delta,
// ClipperLib::JoinType joinType = DefaultJoinType, double miterLimit = DefaultMiterLimit)` with
// the defaults `jtMiter` and 3 (ClipperUtils.hpp:30-31).
fn offset_ex_expolygon(expolygon: &ExPolygon, delta: f32) -> ExPolygons {
    let contour = path_of_points(expolygon.contour.points());
    let holes: crate::clipper::Paths = expolygon.holes.iter().map(|h| path_of_points(h.points())).collect();
    crate::clipper_utils::offset_ex_expolygon_paths(&contour, &holes, delta as f64, crate::clipper::JoinType::jtMiter, 3.0)
}

// ClipperUtils.cpp:570-571  `ExPolygons offset_ex(const ExPolygons &expolygons, const float delta,
// ...)` with the same defaults.
fn offset_ex(expolygons: &[ExPolygon], delta: f32) -> ExPolygons {
    let rings: Vec<(crate::clipper::Path, crate::clipper::Paths)> =
        expolygons.iter().map(|e| (path_of_points(e.contour.points()), e.holes.iter().map(|h| path_of_points(h.points())).collect())).collect();
    crate::clipper_utils::offset_ex_expolygons_paths(&rings, delta as f64, crate::clipper::JoinType::jtMiter, 3.0)
}

// ClipperUtils.cpp:813-814  `ExPolygons union_ex(const Polygons &subject, ClipperLib::PolyFillType
// fill_type)`
fn union_ex_polygons(subject: &[Polygon], fill_type: crate::clipper::PolyFillType) -> ExPolygons {
    crate::clipper_utils::_clipper_ex(crate::clipper::ClipType::ctUnion, &polygons_provider(subject), &crate::clipper::Paths::new(), ApplySafetyOffset::No, fill_type)
}

// ClipperUtils.cpp:815-816  `ExPolygons union_ex(const ExPolygons &subject)`
fn union_ex(subject: &[ExPolygon]) -> ExPolygons {
    crate::clipper_utils::_clipper_ex(
        crate::clipper::ClipType::ctUnion,
        &expolygons_provider(subject),
        &crate::clipper::Paths::new(),
        ApplySafetyOffset::No,
        crate::clipper::PolyFillType::pftNonZero,
    )
}

// ClipperUtils.cpp:755-756  `ExPolygons diff_ex(const ExPolygons &subject, const ExPolygons &clip,
// ApplySafetyOffset do_safety_offset)`
fn diff_ex(subject: &[ExPolygon], clip: &[ExPolygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
    crate::clipper_utils::_clipper_ex(
        crate::clipper::ClipType::ctDifference,
        &expolygons_provider(subject),
        &expolygons_provider(clip),
        do_safety_offset,
        crate::clipper::PolyFillType::pftNonZero,
    )
}

// ClipperUtils.cpp:802-803  `ExPolygons intersection_ex(const ExPolygons &subject, const ExPolygons
// &clip, ApplySafetyOffset do_safety_offset = ApplySafetyOffset::No)`
fn intersection_ex(subject: &[ExPolygon], clip: &[ExPolygon]) -> ExPolygons {
    crate::clipper_utils::_clipper_ex(
        crate::clipper::ClipType::ctIntersection,
        &expolygons_provider(subject),
        &expolygons_provider(clip),
        ApplySafetyOffset::No,
        crate::clipper::PolyFillType::pftNonZero,
    )
}

/// Create mask for patches
/// CutSurface.cpp:460-467 / CutSurface.cpp:3232-3402
pub fn select_patches(
    best_distances: &ProjectionDistances,
    patches: &SurfacePatches,
    shapes: &ExPolygons,
    shapes_bb: &BoundingBox,
    s2i: &ExPolygonsIndices,
    cutAOIs: &VCutAOIs,
    meshes: &CutMeshes,
    projection: &dyn IProjection,
) -> Vec<bool> {
    // extension to cover numerical mistake made by back projection patch from 3d to 2d
    // Calculated as one percent of average size(width and height)
    let s: Point = shapes_bb.size();
    // (s.x() + s.y()) / float(2 * 100): the sum of two coord_t, converted to float
    let extend_delta: f32 = (s.x + s.y) as f32 / (2 * 100) as f32;

    // vector of patches for shape
    let mut used_shapes_patches: Vec<Vec<u32>> = vec![Vec::new(); shapes.len()];
    let mut in_distances: Vec<bool> = vec![false; patches.len()];
    for (d_index, d) in best_distances.iter().enumerate() {
        // exist valid projection for shape point?
        if d.patch_index == u32::MAX {
            continue;
        }
        if in_distances[d.patch_index as usize] {
            continue;
        }
        in_distances[d.patch_index as usize] = true;

        let id: ExPolygonsIndex = s2i.cvt_index(d_index as u32);
        used_shapes_patches[id.expolygons_index as usize].push(d.patch_index);
    }

    // vector of patches for shape
    let mut shapes_patches: Vec<Vec<u32>> = vec![Vec::new(); shapes.len()];
    for (patch_index, patch) in patches.iter().enumerate() {
        shapes_patches[patch.shape_id].push(patch_index as u32);
    }

    for shape_index in 0..shapes.len() {
        let shape: &ExPolygon = &shapes[shape_index];
        // std::vector<uint32_t> &used_shape_patches = used_shapes_patches[shape_index];
        if used_shapes_patches[shape_index].is_empty() {
            continue;
        }
        // is used all exist patches?
        // (the C++ compares the size of `used_shapes_patches`, the vector over the shapes)
        if used_shapes_patches.len() == shapes_patches[shape_index].len() {
            continue;
        }
        let used_shape_patches: &mut Vec<u32> = &mut used_shapes_patches[shape_index];
        if used_shape_patches.len() == 1 {
            let patch_index: u32 = used_shape_patches[0];
            let patch: &SurfacePatch = &patches[patch_index as usize];
            if of_patch::is_over_whole_expoly(patch, shapes, cutAOIs, meshes) {
                continue;
            }
        }

        // only shapes containing multiple patches
        // or not full filled are back projected (hard processed)

        // intersection of converted patches to 2d
        let mut fill: ExPolygons = Vec::with_capacity(used_shape_patches.len());

        // Heuristics to predict which patch to be used need average patch depth
        let mut used_patches_depth = Vec2d::new(f64::MAX, f64::MIN_POSITIVE);
        for &patch_index in used_shape_patches.iter() {
            let patch_area: ExPolygon = to_expoly(&patches[patch_index as usize], projection, &mut used_patches_depth);
            let patch_areas: ExPolygons = offset_ex_expolygon(&patch_area, extend_delta);
            fill.extend(patch_areas);
        }
        fill = union_ex(&fill);

        // not cutted area of expolygon
        let mut rest: ExPolygons = diff_ex(std::slice::from_ref(shape), &fill, ApplySafetyOffset::Yes);

        // already filled by multiple patches
        if rest.is_empty() {
            continue;
        }

        // find patches overlaped rest area
        struct PatchShape {
            patch_index: u32,
            #[allow(dead_code)]
            shape: ExPolygon,
            intersection: ExPolygons,
            depth_range_center_distance: f64, // always positive
        }
        let mut patch_shapes: Vec<PatchShape> = Vec::new();

        let used_patches_depth_center: f64 = (used_patches_depth.x + used_patches_depth.y) / 2.;

        // sort used_patches for faster search
        used_shape_patches.sort();
        for &patch_index in &shapes_patches[shape_index] {
            // check is patch already used
            let it = used_shape_patches.partition_point(|x| *x < patch_index);
            if it != used_shape_patches.len() && used_shape_patches[it] == patch_index {
                continue;
            }

            // Heuristics to predict which patch to be used need average patch depth
            let mut patche_depth_range = Vec2d::new(f64::MAX, f64::MIN_POSITIVE);
            let patch_shape: ExPolygon = to_expoly(&patches[patch_index as usize], projection, &mut patche_depth_range);
            let depth_center: f64 = (patche_depth_range.x + patche_depth_range.y) / 2.;
            let depth_range_center_distance: f64 = (used_patches_depth_center - depth_center).abs();

            let patch_intersection: ExPolygons = intersection_ex(std::slice::from_ref(&patch_shape), &rest);
            if patch_intersection.is_empty() {
                continue;
            }

            patch_shapes.push(PatchShape { patch_index, shape: patch_shape, intersection: patch_intersection, depth_range_center_distance });
        }

        // nothing to add
        if patch_shapes.is_empty() {
            continue;
        }
        // only one solution to add
        if patch_shapes.len() == 1 {
            used_shape_patches.push(patch_shapes[0].patch_index);
            continue;
        }

        // Idea: Get depth range of used patches and add patches in order by distance to used depth center
        // std::sort(patch_shapes, a.depth_range_center_distance < b.depth_range_center_distance):
        // the same comparisons and moves over the positions of the elements
        let mut order: Vec<usize> = (0..patch_shapes.len()).collect();
        crate::clipper::libcxx_sort(&mut order, |a, b| patch_shapes[*a].depth_range_center_distance < patch_shapes[*b].depth_range_center_distance);

        for &patch_pos in &order {
            let patch: &PatchShape = &patch_shapes[patch_pos];
            // Check when exist some place to fill
            let patch_intersection: ExPolygons = intersection_ex(&patch.intersection, &rest);
            if patch_intersection.is_empty() {
                continue;
            }

            // Extend for sure
            let intersection: ExPolygons = offset_ex(&patch.intersection, extend_delta);
            rest = diff_ex(&rest, &intersection, ApplySafetyOffset::Yes);

            used_shape_patches.push(patch.patch_index);
            if rest.is_empty() {
                break;
            }
        }

        // QUESTION: How to select which patch to use? How to sort them?
        // Now is used back projection distance from used patches
        //
        // Idealy by outline depth: (need ray cast into patches)
        // how to calc wanted depth - idealy by depth of outline help to overlap
        // how to calc patch depth - depth in place of outline position
        // Which outline to use between
    }

    let mut result: Vec<bool> = vec![false; patches.len()];
    for patches in &used_shapes_patches {
        for &patch_index in patches {
            debug_assert!((patch_index as usize) < result.len());
            // check only onece insertation of patch
            debug_assert!(!result[patch_index as usize]);
            result[patch_index as usize] = true;
        }
    }
    result
}

/// Create closed loops of contour vertices created from open half edges
/// CutSurface.cpp:277 / CutSurface.cpp:3404-3465
pub fn create_loops(outlines: &[HI], mesh: &CutMesh) -> Loops {
    let mut loops = Loops::new();
    let mut unclosed = Loops::new();
    for &hi in outlines {
        let vi_s = mesh.source(hi);
        let vi_t = mesh.target(hi);
        // positions in `unclosed` (the C++ keeps pointers to its elements)
        let mut loop_move: Option<usize> = None;
        let mut loop_connect: Option<usize> = None;
        for (cut_index, cut) in unclosed.iter().enumerate() {
            if *cut.last().expect("cut.back()") != vi_s {
                continue;
            }
            if cut[0] == vi_t {
                // cut closing
                loop_move = Some(cut_index);
            } else {
                loop_connect = Some(cut_index);
            }
            break;
        }
        if let Some(index) = loop_move {
            // index of closed cut
            // move cut to result
            // remove it from unclosed cut
            let closed = unclosed.remove(index);
            loops.push(closed);
        } else if let Some(connect) = loop_connect {
            // try find tail to connect cut
            let mut loop_tail: Option<usize> = None;
            for (cut_index, cut) in unclosed.iter().enumerate() {
                if cut[0] != vi_t {
                    continue;
                }
                loop_tail = Some(cut_index);
                break;
            }
            if let Some(index) = loop_tail {
                // index of tail
                // move to connect vector
                let tail: Loop = std::mem::take(&mut unclosed[index]);
                unclosed[connect].extend(tail);
                // remove tail from unclosed cut
                unclosed.remove(index);
            } else {
                unclosed[connect].push(vi_t);
            }
        } else {
            // not found
            let mut create_cut = true;
            // try to insert to front of cut
            for cut in unclosed.iter_mut() {
                if cut[0] != vi_t {
                    continue;
                }
                cut.insert(0, vi_s);
                create_cut = false;
                break;
            }
            if create_cut {
                unclosed.push(vec![vi_s, vi_t]);
            }
        }
    }
    debug_assert!(unclosed.is_empty());
    loops
}

/// Unptoject points from outline loops of patch
/// CutSurface.cpp:381 / CutSurface.cpp:3467-3510
///
/// depth_range .. Range of unprojected points x .. min, y .. max value
pub fn unproject_loops(patch: &SurfacePatch, projection: &dyn IProjection, depth_range: &mut Vec2d) -> Polygons {
    debug_assert!(!patch.loops.is_empty());
    if patch.loops.is_empty() {
        return Polygons::new();
    }

    // NOTE: this method is working only when patch did not contain outward faces
    let mut polys: Polygons = Vec::with_capacity(patch.loops.len());
    // project conture into 2d space to fillconvert outlines to

    let mut count: usize = 0;
    for l in &patch.loops {
        count += l.len();
    }
    let mut depths: Vec<f32> = Vec::with_capacity(count);

    let mut pts: Points = Points::new();
    for l in &patch.loops {
        pts.clear();
        pts.reserve(l.len());
        for &vi in l {
            let p3: P3 = patch.mesh.point(vi);
            let p = to_vec3d(&p3);
            // `double depth;` is not initialized in C++; every projection writes it
            let mut depth: f64 = 0.0;
            let p2_opt: Option<Vec2d> = projection.unproject(&p, Some(&mut depth));
            if depth_range.x > depth {
                depth_range.x = depth; // min
            }
            if depth_range.y < depth {
                depth_range.y = depth; // max
            }
            // Check when appear that skip is enough for poit which can't be unprojected
            // - it could break contour
            debug_assert!(p2_opt.is_some());
            let Some(p2) = p2_opt else {
                continue;
            };

            // p2_opt->cast<Point::coord_type>(): the conversion of a double to an integer
            pts.push(Point::new(p2.x as i64, p2.y as i64));
            depths.push(depth as f32);
        }
        // minimal is triangle
        debug_assert!(pts.len() >= 3);
        if pts.len() < 3 {
            continue;
        }

        polys.push(Polygon::from_points(pts.clone()));
    }

    debug_assert!(!polys.is_empty());
    polys
}

/// Unproject points from loops and create expolygons
/// CutSurface.cpp:390 / CutSurface.cpp:3512-3533
pub fn to_expoly(patch: &SurfacePatch, projection: &dyn IProjection, depth_range: &mut Vec2d) -> ExPolygon {
    let polys: Polygons = unproject_loops(patch, projection, depth_range);
    // should not be used when no opposit triangle are counted so should not create overlaps
    let fill_type = crate::clipper::PolyFillType::pftEvenOdd;
    let expolys: ExPolygons = union_ex_polygons(&polys, fill_type);
    if expolys.len() == 1 {
        return expolys[0].clone();
    }

    // It should be one expolygon
    debug_assert!(false);

    if expolys.is_empty() {
        return ExPolygon::default();
    }
    // find biggest
    let mut biggest: &ExPolygon = &expolys[0];
    for index in 1..expolys.len() {
        let current: &ExPolygon = &expolys[index];
        if biggest.contour.points().len() < current.contour.points().len() {
            biggest = current;
        }
    }
    biggest.clone()
}

/// Convert patch to indexed_triangle_set
/// CutSurface.cpp:482 / CutSurface.cpp:3535-3587
pub fn patch2cut(patch: &mut SurfacePatch) -> SurfaceCut {
    let mesh: &mut CutMesh = &mut patch.mesh;

    let convert_map_name = "v:convert";
    let mut convert_map: surface_mesh::Property_map<VI, Index> = mesh.add_property_map(convert_map_name, 0).0;

    let indices_size: usize = mesh.faces().count();
    let vertices_size: usize = mesh.vertices().count();

    let mut sc = SurfaceCut::default();
    sc.its.indices.reserve(indices_size);
    sc.its.vertices.reserve(vertices_size);
    for vi in mesh.vertices() {
        // vi order is is not sorted
        // assert(vi.idx() == sc.vertices.size());
        // vi is not continous
        // assert(vi.idx() < vertices_size);
        convert_map.set(vi, sc.its.vertices.len() as Index);
        let p: P3 = mesh.point(vi);
        sc.its.vertices.push(Vec3f::new(p[0] as f32, p[1] as f32, p[2] as f32));
    }

    for fi in mesh.faces() {
        let hi = mesh.halfedge(fi);
        debug_assert!(mesh.next(hi).is_valid());
        debug_assert!(mesh.next(mesh.next(hi)).is_valid());
        // Is fi triangle?
        debug_assert!(mesh.next(mesh.next(mesh.next(hi))) == hi);

        // triangle indicies
        let mut ti = Vec3f32i::new(0, 0, 0);
        let mut i: usize = 0;
        for vi in [mesh.source(hi), mesh.target(hi), mesh.target(mesh.next(hi))] {
            ti[i] = convert_map.get(vi) as i32;
            i += 1;
        }
        sc.its.indices.push(ti);
    }

    sc.contours.reserve(patch.loops.len());
    for loop_ in &patch.loops {
        let mut contour: Contour = Vec::with_capacity(loop_.len());
        for &vi in loop_ {
            contour.push(convert_map.get(vi));
        }
        sc.contours.push(contour);
    }

    // Not neccessary, clean and free memory
    mesh.remove_property_map(&mut convert_map);
    sc
}

/// Merge two surface cuts together
/// Added surface cut will be consumed
/// CutSurface.cpp:475 / CutSurface.cpp:3589-3606
pub fn append(sc: &mut SurfaceCut, mut sc_add: SurfaceCut) {
    if sc.empty() {
        *sc = sc_add;
        return;
    }

    if !sc_add.contours.is_empty() {
        let offset: Index = sc.its.vertices.len() as Index;
        let require: usize = sc.contours.len() + sc_add.contours.len();
        if sc.contours.capacity() < require {
            sc.contours.reserve(require - sc.contours.len());
        }
        for cut in sc_add.contours.iter_mut() {
            for i in cut.iter_mut() {
                *i = i.wrapping_add(offset);
            }
        }
        // Slic3r::append(sc.contours, std::move(sc_add.contours));
        sc.contours.append(&mut sc_add.contours);
    }
    // its_merge(sc, std::move(sc_add));
    crate::triangle_mesh::its_merge(&mut sc.its, &sc_add.its);
}

/// Merge masked patches to one surface cut
/// CutSurface.cpp:491-492 / CutSurface.cpp:3608-3617
pub fn merge_patches(patches: &mut SurfacePatches, mask: &[bool]) -> SurfaceCut {
    let mut result = SurfaceCut::default();
    for (index, patch) in patches.iter_mut().enumerate() {
        if !mask[index] {
            continue;
        }
        append(&mut result, patch2cut(patch));
    }
    result
}

// ===========================================================================
// CutSurface.cpp:3619-4082  #ifdef DEBUG_OUTPUT_DIR (not compiled into the engine): the three
// functions of that part that write no file
// ===========================================================================

/// CutSurface.cpp:3733-3734 / CutSurface.cpp:3737-3769
pub fn create_indexed_triangle_set(faces: &[FI], mesh: &CutMesh) -> indexed_triangle_set {
    let mut vertices: Vec<VI> = Vec::with_capacity(faces.len() * 2);

    let mut its = indexed_triangle_set::default();
    its.indices.reserve(faces.len());
    for &fi in faces {
        let mut hi = mesh.halfedge(fi);
        let hi_end = hi;

        let mut ti: usize = 0;
        let mut t = Vec3f32i::new(0, 0, 0);

        loop {
            let vi = mesh.source(hi);
            // std::find(vertices.begin(), vertices.end(), vi)
            let res = vertices.iter().position(|v| *v == vi).unwrap_or(vertices.len());
            t[ti] = res as i32;
            ti += 1;
            if res == vertices.len() {
                vertices.push(vi);
            }
            hi = mesh.next(hi);
            if hi == hi_end {
                break;
            }
        }

        its.indices.push(t);
    }

    its.vertices.reserve(vertices.len());
    for vi in vertices {
        let p = mesh.point(vi);
        its.vertices.push(Vec3f::new(p[0] as f32, p[1] as f32, p[2] as f32));
    }
    its
}

/// Getter on triangle tip (third vertex of face)
/// CutSurface.cpp:3942-3944 / CutSurface.cpp:3948-3973
pub fn get_triangle_tip(vi1: u32, vi2: u32, its: &indexed_triangle_set) -> u32 {
    debug_assert!((vi1 as usize) < its.vertices.len());
    debug_assert!((vi2 as usize) < its.vertices.len());
    for t in &its.indices {
        let mut tvi: u32 = u32::MAX;
        for k in 0..3 {
            let vi_: u32 = t[k] as u32;
            if vi_ == vi1 {
                continue;
            }
            if vi_ == vi2 {
                continue;
            }
            if tvi == u32::MAX {
                tvi = vi_;
            } else {
                tvi = u32::MAX;
                break;
            }
        }
        if tvi != u32::MAX {
            return tvi;
        }
    }
    // triangle with indices vi1 and vi2 doesnt exist
    debug_assert!(false);
    u32::MAX
}

// `Vec3f` algebra of Eigen (float): `normalize()` (Dot.h: divided by `sqrt(squaredNorm())` when
// the squared norm is positive; the redux of three floats is `(x*x + y*y) + z*z` with the first
// sum fused by clang, `fma(x, x, y*y)`, then `fma(z, z, .)`), `cross` (OrthoMethods.h, each
// difference fused).
fn normalize3f(v: &mut Vec3f) {
    let z = v.z.mul_add(v.z, v.x.mul_add(v.x, v.y * v.y));
    if z > 0.0 {
        let s = z.sqrt();
        *v = Vec3f::new(v.x / s, v.y / s, v.z / s);
    }
}
fn cross3f(a: &Vec3f, b: &Vec3f) -> Vec3f {
    Vec3f::new(a.y.mul_add(b.z, -(a.z * b.y)), a.z.mul_add(b.x, -(a.x * b.z)), a.x.mul_add(b.y, -(a.y * b.x)))
}

/// Create model consist of rectangles for each contour edge
/// CutSurface.cpp:3933 / CutSurface.cpp:3975-4015
pub fn create_contour_its(its: &indexed_triangle_set, contour: &[u32]) -> indexed_triangle_set {
    const line_width: f32 = 0.1;
    let mut result = indexed_triangle_set::default();
    result.vertices.reserve((contour.len() + 1) * 4);
    result.indices.reserve((contour.len() + 1) * 2);
    let mut prev_vi: u32 = *contour.last().expect("contour.back()");
    for &vi in contour {
        let a: Vec3f = its.vertices[vi as usize];
        let b: Vec3f = its.vertices[prev_vi as usize];
        let c: Vec3f = its.vertices[get_triangle_tip(vi, prev_vi, its) as usize];

        let mut v1: Vec3f = b - a; // from a to b
        normalize3f(&mut v1);
        let mut v2: Vec3f = c - a; // from a to c
        normalize3f(&mut v2);
        // triangle normal
        let mut norm: Vec3f = cross3f(&v1, &v2);
        normalize3f(&mut norm);
        // perpendiculat to edge lay on triangle
        let mut perp_to_edge: Vec3f = cross3f(&norm, &v1);
        normalize3f(&mut perp_to_edge);

        let dir: Vec3f = Vec3f::new(-perp_to_edge.x * line_width, -perp_to_edge.y * line_width, -perp_to_edge.z * line_width);

        let ai = result.vertices.len();
        result.vertices.push(a);
        let bi = result.vertices.len();
        result.vertices.push(b);
        let ai2 = result.vertices.len();
        result.vertices.push(a + dir);
        let bi2 = result.vertices.len();
        result.vertices.push(b + dir);

        result.indices.push(Vec3f32i::new(ai as i32, bi as i32, ai2 as i32));
        result.indices.push(Vec3f32i::new(ai2 as i32, bi as i32, bi2 as i32));
        prev_vi = vi;
    }
    result
}

/// CutSurface.hpp:71 / CutSurface.cpp:4084-4091
///
/// `pmp` is CGAL's `read_OFF` and `corefine` (see `PolygonMeshProcessing`).
pub fn corefine_test(model_path: &str, shape_path: &str, pmp: &dyn PolygonMeshProcessing) -> bool {
    let mut model = CutMesh::new();
    let mut shape = CutMesh::new();
    if !pmp.read_off(model_path, &mut model) {
        return false;
    }
    if !pmp.read_off(shape_path, &mut shape) {
        return false;
    }

    pmp.corefine(&mut model, &mut shape, NamedParameters::default_values(), NamedParameters::default_values());
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    // CutSurface.cpp:824-827: a size mismatch is `assert(false); return {};`.
    // `assert` is live only without NDEBUG (the port maps it to
    // `debug_assert!`), so a debug build stops at the assertion exactly as a
    // debug C++ build aborts, and a release build (slicer_cli is built with
    // -DNDEBUG) returns the empty set.
    #[test]
    #[cfg_attr(debug_assertions, should_panic(expected = "assertion failed"))]
    fn its_mask_size_mismatch_returns_empty() {
        let its = indexed_triangle_set::default();
        let mask = vec![true]; // size != its.indices.size() (0)
        let r = its_mask(&its, &mask);
        assert!(r.indices.is_empty());
    }

    // Euler_operations.h:694-869 `Euler::add_face` on a tetrahedron: four faces, six edges, every
    // halfedge with a face, `opposite` of `2e` is `2e + 1`.
    #[test]
    fn surface_mesh_tetrahedron() {
        let mut m = CutMesh::new();
        let v: Vec<VI> = [[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]].iter().map(|p| m.add_vertex(*p)).collect();
        assert!(m.add_face(v[0], v[2], v[1]).is_valid());
        assert!(m.add_face(v[0], v[1], v[3]).is_valid());
        assert!(m.add_face(v[1], v[2], v[3]).is_valid());
        assert!(m.add_face(v[2], v[0], v[3]).is_valid());
        // a face over an edge that already has its two faces cannot be added
        assert!(!m.add_face(v[0], v[1], v[2]).is_valid());
        assert_eq!(m.number_of_vertices(), 4);
        assert_eq!(m.number_of_edges(), 6);
        assert_eq!(m.number_of_faces(), 4);
        assert!(m.is_valid(false));
        for h in m.halfedges() {
            assert!(m.face(h).is_valid());
            assert_eq!(m.opposite(m.opposite(h)), h);
            assert_eq!(m.next(m.next(m.next(h))), h);
        }
    }
}
