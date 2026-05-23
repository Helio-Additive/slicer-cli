//! Triangle mesh data structure.
//!
//! This module provides the TriangleMesh type representing a 3D mesh
//! composed of triangles, mirroring BambuStudio's indexed_triangle_set.

use crate::geometry::{BoundingBox3F, Point3F};
use crate::{CoordF, Error, Result};
use serde::{Deserialize, Serialize};
use std::fmt;

/// A single triangle defined by three vertex indices.
#[derive(Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Triangle {
    /// Indices into the vertex array for the three corners.
    pub indices: [u32; 3],
}

impl Triangle {
    // Create a new triangle from vertex indices.
    #[inline]
    pub const fn new(v0: u32, v1: u32, v2: u32) -> Self {
        Self {
            indices: [v0, v1, v2],
        }
    }

    /// Get the vertex index at position i (0, 1, or 2).
    #[inline]
    pub fn vertex(&self, i: usize) -> u32 {
        self.indices[i]
    }

    /// Check if this triangle is degenerate (has duplicate vertices).
    #[inline]
    pub fn is_degenerate(&self) -> bool {
        self.indices[0] == self.indices[1]
            || self.indices[1] == self.indices[2]
            || self.indices[2] == self.indices[0]
    }
}

impl fmt::Debug for Triangle {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Triangle({}, {}, {})",
            self.indices[0], self.indices[1], self.indices[2]
        )
    }
}

impl From<[u32; 3]> for Triangle {
    #[inline]
    fn from(indices: [u32; 3]) -> Self {
        Self { indices }
    }
}

impl From<Triangle> for [u32; 3] {
    #[inline]
    fn from(tri: Triangle) -> Self {
        tri.indices
    }
}

/// A triangle with both vertex indices and resolved vertex positions.
/// Returned by [`TriangleMesh::triangles()`].
pub struct TriangleData {
    /// The vertex indices (into the mesh's vertex array).
    pub indices: [u32; 3],
    /// The resolved vertex positions.
    pub vertices: [Point3F; 3],
}

/// A 3D triangle mesh represented as an indexed triangle set.
///
/// This is the primary mesh representation used throughout the slicer,
/// mirroring BambuStudio's indexed_triangle_set structure.
#[derive(Clone, Default, Serialize, Deserialize)]
pub struct TriangleMesh {
    /// Vertex positions (in mm, floating-point).
    vertices: Vec<Point3F>,
    /// Triangle indices into the vertex array.
    indices: Vec<Triangle>,
    /// Cached bounding box (lazily computed).
    #[serde(skip)]
    bounding_box: Option<BoundingBox3F>,
}

impl TriangleMesh {
    // Create a new empty mesh.
    #[inline]
    pub fn new() -> Self {
        Self {
            vertices: Vec::new(),
            indices: Vec::new(),
            bounding_box: None,
        }
    }

    /// Create a mesh with preallocated capacity.
    pub fn with_capacity(vertex_count: usize, triangle_count: usize) -> Self {
        Self {
            vertices: Vec::with_capacity(vertex_count),
            indices: Vec::with_capacity(triangle_count),
            bounding_box: None,
        }
    }

    /// Create a mesh from vertices and indices.
    pub fn from_parts(vertices: Vec<Point3F>, indices: Vec<Triangle>) -> Self {
        Self {
            vertices,
            indices,
            bounding_box: None,
        }
    }

    /// Get the vertices of the mesh.
    #[inline]
    pub fn vertices(&self) -> &[Point3F] {
        &self.vertices
    }

    /// Get mutable access to the vertices.
    #[inline]
    pub fn vertices_mut(&mut self) -> &mut Vec<Point3F> {
        self.bounding_box = None; // Invalidate cache
        &mut self.vertices
    }

    /// Get the triangle indices.
    #[inline]
    pub fn indices(&self) -> &[Triangle] {
        &self.indices
    }

    /// Get mutable access to the triangle indices.
    #[inline]
    pub fn indices_mut(&mut self) -> &mut Vec<Triangle> {
        &mut self.indices
    }

    /// Get the number of vertices.
    #[inline]
    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    /// Get the number of triangles.
    #[inline]
    pub fn triangle_count(&self) -> usize {
        self.indices.len()
    }

    /// Check if the mesh is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.indices.is_empty()
    }

    /// Add a vertex and return its index.
    pub fn add_vertex(&mut self, v: Point3F) -> u32 {
        let idx = self.vertices.len() as u32;
        self.vertices.push(v);
        self.bounding_box = None;
        idx
    }

    /// Add a triangle.
    pub fn add_triangle(&mut self, tri: Triangle) {
        self.indices.push(tri);
    }

    /// Add a triangle from vertex indices.
    pub fn add_triangle_indices(&mut self, v0: u32, v1: u32, v2: u32) {
        self.indices.push(Triangle::new(v0, v1, v2));
    }

    /// Get a vertex by index.
    #[inline]
    pub fn vertex(&self, idx: u32) -> Point3F {
        self.vertices[idx as usize]
    }

    /// Get the three vertices of a triangle.
    #[inline]
    pub fn triangle_vertices(&self, tri_idx: usize) -> [Point3F; 3] {
        let tri = &self.indices[tri_idx];
        [
            self.vertices[tri.indices[0] as usize],
            self.vertices[tri.indices[1] as usize],
            self.vertices[tri.indices[2] as usize],
        ]
    }

    /// Get the vertex indices of a triangle.
    #[inline]
    pub fn triangle_indices(&self, tri_idx: usize) -> [u32; 3] {
        self.indices[tri_idx].indices
    }

    /// Iterate over triangles, yielding both indices and resolved vertices.
    /// This matches the libslic3r pattern of iterating over triangle data.
    pub fn triangles(&self) -> impl Iterator<Item = TriangleData> + '_ {
        (0..self.triangle_count()).map(move |i| TriangleData {
            indices: self.triangle_indices(i),
            vertices: self.triangle_vertices(i),
        })
    }

    /// Get the bounding box of the mesh.
    pub fn bounding_box(&mut self) -> BoundingBox3F {
        if let Some(bb) = self.bounding_box {
            return bb;
        }

        let mut bb = BoundingBox3F::new();
        for v in &self.vertices {
            bb.merge_point(*v);
        }
        self.bounding_box = Some(bb);
        bb
    }

    /// Get the bounding box without caching (const method).
    pub fn compute_bounding_box(&self) -> BoundingBox3F {
        let mut bb = BoundingBox3F::new();
        for v in &self.vertices {
            bb.merge_point(*v);
        }
        bb
    }

    /// Get the center of the mesh.
    pub fn center(&mut self) -> Point3F {
        self.bounding_box().center()
    }

    /// Get the size of the mesh (bounding box dimensions).
    pub fn size(&mut self) -> Point3F {
        self.bounding_box().size()
    }

    /// Calculate the normal of a triangle.
    pub fn triangle_normal(&self, tri_idx: usize) -> Point3F {
        let [v0, v1, v2] = self.triangle_vertices(tri_idx);
        let e1 = v1 - v0;
        let e2 = v2 - v0;
        e1.cross(&e2).normalize()
    }

    /// Calculate the area of a triangle.
    pub fn triangle_area(&self, tri_idx: usize) -> CoordF {
        let [v0, v1, v2] = self.triangle_vertices(tri_idx);
        let e1 = v1 - v0;
        let e2 = v2 - v0;
        e1.cross(&e2).length() / 2.0
    }

    /// Calculate the total surface area of the mesh.
    pub fn surface_area(&self) -> CoordF {
        let mut total = 0.0;
        for i in 0..self.indices.len() {
            total += self.triangle_area(i);
        }
        total
    }

    /// Calculate the volume of the mesh (assumes watertight mesh).
    /// Uses the signed volume formula based on tetrahedra from origin.
    pub fn volume(&self) -> CoordF {
        let mut total = 0.0;
        for tri in &self.indices {
            let v0 = self.vertices[tri.indices[0] as usize];
            let v1 = self.vertices[tri.indices[1] as usize];
            let v2 = self.vertices[tri.indices[2] as usize];

            // Signed volume of tetrahedron from origin to triangle
            total += v0.dot(&v1.cross(&v2)) / 6.0;
        }
        total.abs()
    }

    /// Translate the mesh by a vector.
    pub fn translate(&mut self, v: Point3F) {
        for vertex in &mut self.vertices {
            *vertex = *vertex + v;
        }
        self.bounding_box = None;
    }

    /// Scale the mesh uniformly about the origin.
    pub fn scale(&mut self, factor: CoordF) {
        for vertex in &mut self.vertices {
            *vertex = *vertex * factor;
        }
        self.bounding_box = None;
    }

    /// Scale the mesh non-uniformly.
    pub fn scale_xyz(&mut self, sx: CoordF, sy: CoordF, sz: CoordF) {
        for vertex in &mut self.vertices {
            vertex.x *= sx;
            vertex.y *= sy;
            vertex.z *= sz;
        }
        self.bounding_box = None;
    }

    /// Center the mesh at the origin.
    pub fn center_at_origin(&mut self) {
        let center = self.center();
        self.translate(-center);
    }

    /// Place the mesh on the Z=0 plane (bottom touching).
    pub fn place_on_bed(&mut self) {
        let bb = self.bounding_box();
        let offset = Point3F::new(0.0, 0.0, -bb.min.z);
        self.translate(offset);
    }

    /// Flip all triangle normals (reverse winding order).
    pub fn flip_normals(&mut self) {
        for tri in &mut self.indices {
            tri.indices.swap(0, 2);
        }
    }

    /// Remove degenerate triangles.
    pub fn remove_degenerate_triangles(&mut self) {
        self.indices.retain(|tri| !tri.is_degenerate());
    }

    /// Check if the mesh has any degenerate triangles.
    pub fn has_degenerate_triangles(&self) -> bool {
        self.indices.iter().any(|tri| tri.is_degenerate())
    }

    /// Merge vertices that are within a tolerance distance.
    pub fn merge_close_vertices(&mut self, tolerance: CoordF) {
        if self.vertices.is_empty() {
            return;
        }

        let tolerance_sq = tolerance * tolerance;
        let mut vertex_map: Vec<u32> = (0..self.vertices.len() as u32).collect();
        let mut new_vertices: Vec<Point3F> = Vec::new();

        for (i, v) in self.vertices.iter().enumerate() {
            let mut found = false;
            for (j, nv) in new_vertices.iter().enumerate() {
                if v.distance_squared(nv) < tolerance_sq {
                    vertex_map[i] = j as u32;
                    found = true;
                    break;
                }
            }
            if !found {
                vertex_map[i] = new_vertices.len() as u32;
                new_vertices.push(*v);
            }
        }

        // Remap triangle indices
        for tri in &mut self.indices {
            for idx in &mut tri.indices {
                *idx = vertex_map[*idx as usize];
            }
        }

        self.vertices = new_vertices;
        self.bounding_box = None;
    }

    /// Remove unused vertices (vertices not referenced by any triangle).
    pub fn remove_unused_vertices(&mut self) {
        let mut used = vec![false; self.vertices.len()];
        for tri in &self.indices {
            for &idx in &tri.indices {
                used[idx as usize] = true;
            }
        }

        // Build mapping from old to new indices
        let mut new_indices: Vec<u32> = vec![0; self.vertices.len()];
        let mut new_vertices: Vec<Point3F> = Vec::new();
        for (i, &is_used) in used.iter().enumerate() {
            if is_used {
                new_indices[i] = new_vertices.len() as u32;
                new_vertices.push(self.vertices[i]);
            }
        }

        // Remap triangle indices
        for tri in &mut self.indices {
            for idx in &mut tri.indices {
                *idx = new_indices[*idx as usize];
            }
        }

        self.vertices = new_vertices;
        self.bounding_box = None;
    }

    /// Clear the mesh.
    pub fn clear(&mut self) {
        self.vertices.clear();
        self.indices.clear();
        self.bounding_box = None;
    }

    /// Reserve capacity for vertices and triangles.
    pub fn reserve(&mut self, vertex_count: usize, triangle_count: usize) {
        self.vertices.reserve(vertex_count);
        self.indices.reserve(triangle_count);
    }

    /// Validate the mesh (check for valid indices).
    pub fn validate(&self) -> Result<()> {
        let vertex_count = self.vertices.len() as u32;
        for (i, tri) in self.indices.iter().enumerate() {
            for &idx in &tri.indices {
                if idx >= vertex_count {
                    return Err(Error::Mesh(format!(
                        "Triangle {} has invalid vertex index {} (only {} vertices)",
                        i, idx, vertex_count
                    )));
                }
            }
        }
        Ok(())
    }

    /// Check if the mesh is likely manifold (simple check based on edge count).
    /// A proper manifold check would require edge connectivity analysis.
    pub fn is_likely_manifold(&self) -> bool {
        // Simple heuristic: check that we have a reasonable vertex-to-triangle ratio
        // For a closed manifold, V - E + F = 2 (Euler characteristic)
        // where E ≈ 1.5 * F for a triangle mesh
        // So V ≈ 0.5 * F + 2, or triangle_count ≈ 2 * vertex_count

        if self.vertices.is_empty() || self.indices.is_empty() {
            return false;
        }

        let ratio = self.indices.len() as f64 / self.vertices.len() as f64;
        // Allow some tolerance around the expected ratio
        ratio > 1.5 && ratio < 2.5
    }

    /// Create a simple cube mesh for testing.
    pub fn cube(size: CoordF) -> Self {
        let half = size / 2.0;
        let vertices = vec![
            // Bottom face
            Point3F::new(-half, -half, -half),
            Point3F::new(half, -half, -half),
            Point3F::new(half, half, -half),
            Point3F::new(-half, half, -half),
            // Top face
            Point3F::new(-half, -half, half),
            Point3F::new(half, -half, half),
            Point3F::new(half, half, half),
            Point3F::new(-half, half, half),
        ];

        let indices = vec![
            // Bottom
            Triangle::new(0, 2, 1),
            Triangle::new(0, 3, 2),
            // Top
            Triangle::new(4, 5, 6),
            Triangle::new(4, 6, 7),
            // Front
            Triangle::new(0, 1, 5),
            Triangle::new(0, 5, 4),
            // Back
            Triangle::new(2, 3, 7),
            Triangle::new(2, 7, 6),
            // Left
            Triangle::new(0, 4, 7),
            Triangle::new(0, 7, 3),
            // Right
            Triangle::new(1, 2, 6),
            Triangle::new(1, 6, 5),
        ];

        Self::from_parts(vertices, indices)
    }
}

impl fmt::Debug for TriangleMesh {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "TriangleMesh({} vertices, {} triangles)",
            self.vertices.len(),
            self.indices.len()
        )
    }
}

impl fmt::Display for TriangleMesh {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "TriangleMesh: {} vertices, {} triangles",
            self.vertices.len(),
            self.indices.len()
        )?;
        if let Some(bb) = self.bounding_box {
            write!(f, ", bounds: {}", bb)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_triangle_new() {
        let tri = Triangle::new(0, 1, 2);
        assert_eq!(tri.indices[0], 0);
        assert_eq!(tri.indices[1], 1);
        assert_eq!(tri.indices[2], 2);
    }

    #[test]
    fn test_triangle_degenerate() {
        let good = Triangle::new(0, 1, 2);
        assert!(!good.is_degenerate());

        let bad1 = Triangle::new(0, 0, 2);
        assert!(bad1.is_degenerate());

        let bad2 = Triangle::new(0, 1, 0);
        assert!(bad2.is_degenerate());
    }

    #[test]
    fn test_mesh_new() {
        let mesh = TriangleMesh::new();
        assert!(mesh.is_empty());
        assert_eq!(mesh.vertex_count(), 0);
        assert_eq!(mesh.triangle_count(), 0);
    }

    #[test]
    fn test_mesh_add_vertex() {
        let mut mesh = TriangleMesh::new();
        let idx = mesh.add_vertex(Point3F::new(1.0, 2.0, 3.0));
        assert_eq!(idx, 0);
        assert_eq!(mesh.vertex_count(), 1);

        let v = mesh.vertex(0);
        assert!((v.x - 1.0).abs() < 1e-10);
        assert!((v.y - 2.0).abs() < 1e-10);
        assert!((v.z - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_mesh_add_triangle() {
        let mut mesh = TriangleMesh::new();
        mesh.add_vertex(Point3F::new(0.0, 0.0, 0.0));
        mesh.add_vertex(Point3F::new(1.0, 0.0, 0.0));
        mesh.add_vertex(Point3F::new(0.0, 1.0, 0.0));
        mesh.add_triangle_indices(0, 1, 2);

        assert_eq!(mesh.triangle_count(), 1);
    }

    #[test]
    fn test_mesh_cube() {
        let mut mesh = TriangleMesh::cube(10.0);
        assert_eq!(mesh.vertex_count(), 8);
        assert_eq!(mesh.triangle_count(), 12);

        let bb = mesh.bounding_box();
        assert!((bb.min.x - (-5.0)).abs() < 1e-10);
        assert!((bb.max.x - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_mesh_bounding_box() {
        let mut mesh = TriangleMesh::new();
        mesh.add_vertex(Point3F::new(0.0, 0.0, 0.0));
        mesh.add_vertex(Point3F::new(10.0, 20.0, 30.0));

        let bb = mesh.bounding_box();
        assert!(bb.is_defined());
        assert!((bb.min.x - 0.0).abs() < 1e-10);
        assert!((bb.max.x - 10.0).abs() < 1e-10);
        assert!((bb.max.z - 30.0).abs() < 1e-10);
    }

    #[test]
    fn test_mesh_translate() {
        let mut mesh = TriangleMesh::cube(10.0);
        mesh.translate(Point3F::new(100.0, 0.0, 0.0));

        let bb = mesh.bounding_box();
        assert!((bb.min.x - 95.0).abs() < 1e-10);
        assert!((bb.max.x - 105.0).abs() < 1e-10);
    }

    #[test]
    fn test_mesh_scale() {
        let mut mesh = TriangleMesh::cube(10.0);
        mesh.scale(2.0);

        let bb = mesh.bounding_box();
        assert!((bb.min.x - (-10.0)).abs() < 1e-10);
        assert!((bb.max.x - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_mesh_center_at_origin() {
        let mut mesh = TriangleMesh::cube(10.0);
        mesh.translate(Point3F::new(100.0, 100.0, 100.0));
        mesh.center_at_origin();

        let center = mesh.center();
        assert!(center.x.abs() < 1e-10);
        assert!(center.y.abs() < 1e-10);
        assert!(center.z.abs() < 1e-10);
    }

    #[test]
    fn test_mesh_triangle_area() {
        let mut mesh = TriangleMesh::new();
        mesh.add_vertex(Point3F::new(0.0, 0.0, 0.0));
        mesh.add_vertex(Point3F::new(10.0, 0.0, 0.0));
        mesh.add_vertex(Point3F::new(0.0, 10.0, 0.0));
        mesh.add_triangle_indices(0, 1, 2);

        let area = mesh.triangle_area(0);
        assert!((area - 50.0).abs() < 1e-10);
    }

    #[test]
    fn test_mesh_validate() {
        let mut mesh = TriangleMesh::new();
        mesh.add_vertex(Point3F::new(0.0, 0.0, 0.0));
        mesh.add_vertex(Point3F::new(1.0, 0.0, 0.0));
        mesh.add_vertex(Point3F::new(0.0, 1.0, 0.0));
        mesh.add_triangle_indices(0, 1, 2);

        assert!(mesh.validate().is_ok());

        // Add invalid triangle
        mesh.add_triangle_indices(0, 1, 100);
        assert!(mesh.validate().is_err());
    }

    #[test]
    fn test_mesh_flip_normals() {
        let mut mesh = TriangleMesh::new();
        mesh.add_vertex(Point3F::new(0.0, 0.0, 0.0));
        mesh.add_vertex(Point3F::new(1.0, 0.0, 0.0));
        mesh.add_vertex(Point3F::new(0.0, 1.0, 0.0));
        mesh.add_triangle_indices(0, 1, 2);

        let normal_before = mesh.triangle_normal(0);
        mesh.flip_normals();
        let normal_after = mesh.triangle_normal(0);

        // Normals should be opposite
        assert!((normal_before.z + normal_after.z).abs() < 1e-10);
    }

    #[test]
    fn test_mesh_remove_degenerate() {
        let mut mesh = TriangleMesh::new();
        mesh.add_vertex(Point3F::new(0.0, 0.0, 0.0));
        mesh.add_vertex(Point3F::new(1.0, 0.0, 0.0));
        mesh.add_vertex(Point3F::new(0.0, 1.0, 0.0));
        mesh.add_triangle_indices(0, 1, 2); // Good triangle
        mesh.add_triangle_indices(0, 0, 2); // Degenerate

        assert_eq!(mesh.triangle_count(), 2);
        assert!(mesh.has_degenerate_triangles());

        mesh.remove_degenerate_triangles();

        assert_eq!(mesh.triangle_count(), 1);
        assert!(!mesh.has_degenerate_triangles());
    }
}
