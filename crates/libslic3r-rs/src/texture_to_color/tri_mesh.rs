//! Port of `TextureToColor/TriMesh.hpp`: the triangle mesh of the tex2color algorithms, an
//! `indexed_triangle_set` with a face count.
//!
//! C++ Reference:
//! - src/libslic3r/TextureToColor/TriMesh.hpp
//!
//! `struct TriMesh : ::indexed_triangle_set` adds no member. Here it wraps the crate's
//! `indexed_triangle_set` and dereferences to it, so `mesh.vertices` and `mesh.indices` read as
//! in C++.

use std::ops::{Deref, DerefMut};

use crate::triangle_set_sampling::{indexed_triangle_set, Vec3f, Vec3i};

/// TriMesh.hpp:7, `stl_vertex`
pub type TriVertex = Vec3f;
/// TriMesh.hpp:8
pub type TriVertices = Vec<Vec3f>;
/// TriMesh.hpp:9, `stl_triangle_vertex_indices`
pub type TriFace = Vec3i;
/// TriMesh.hpp:10
pub type TriFaces = Vec<Vec3i>;

/// TriMesh.hpp:12-25
#[derive(Debug, Clone, Default)]
pub struct TriMesh(pub indexed_triangle_set);

impl TriMesh {
    /// TriMesh.hpp:20-22, `TriMesh(indices_, vertices_)`
    pub fn new(indices_: TriFaces, vertices_: TriVertices) -> Self {
        TriMesh(indexed_triangle_set { vertices: vertices_, indices: indices_ })
    }

    /// TriMesh.hpp:24
    pub fn facets_count(&self) -> usize {
        self.indices.len()
    }

    /// `indexed_triangle_set::clear()` (admesh/stl.h), inherited
    pub fn clear(&mut self) {
        self.0.indices.clear();
        self.0.vertices.clear();
    }
}

/// TriMesh.hpp:18-19, the converting constructors
impl From<indexed_triangle_set> for TriMesh {
    fn from(d: indexed_triangle_set) -> Self {
        TriMesh(d)
    }
}

impl Deref for TriMesh {
    type Target = indexed_triangle_set;
    fn deref(&self) -> &indexed_triangle_set {
        &self.0
    }
}

impl DerefMut for TriMesh {
    fn deref_mut(&mut self) -> &mut indexed_triangle_set {
        &mut self.0
    }
}
