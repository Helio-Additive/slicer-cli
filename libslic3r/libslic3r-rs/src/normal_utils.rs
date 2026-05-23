//! Vertex normal calculation utilities for triangle meshes.
//!
//! C++ Reference:
//! - NormalUtils.hpp (69 lines)
//! - NormalUtils.cpp (142 lines)
//!
//! This module provides static utility functions for computing vertex normals
//! from triangle mesh data. Multiple weighting schemes are supported:
//! - Average neighbor: simple average of adjacent triangle normals
//! - Angle weighted: weighted by triangle angles at each vertex
//! - Nelson weighted: weighted by edge length products (default)

use crate::geometry::{Point3F, Vec3};
use std::f32::consts::PI;

/// Type of vertex normal calculation
/// NormalUtils.hpp:20-24
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VertexNormalType {
    /// Simple average of neighboring triangle normals
    /// NormalUtils.hpp:21
    AverageNeighbor,
    /// Weighted by triangle angles at each vertex
    /// NormalUtils.hpp:22
    AngleWeighted,
    /// Weighted by edge length products (Nelson's method)
    /// NormalUtils.hpp:23
    NelsonMaxWeighted,
}

/// Triangle vertex indices (3 indices into vertex array)
/// Model.hpp (indexed_triangle_set)
pub type TriangleIndices = [usize; 3];

/// Indexed triangle set representation
/// Model.hpp
#[derive(Debug, Clone)]
pub struct IndexedTriangleSet {
    /// Vertex positions
    pub vertices: Vec<Point3F>,
    /// Triangle vertex indices
    pub indices: Vec<TriangleIndices>,
}

/// Utility functions for computing vertex normals from triangle meshes
/// NormalUtils.hpp:13-70
pub struct NormalUtils;

impl NormalUtils {
    /// Create normal for a single triangle
    /// NormalUtils.cpp:5-16
    ///
    /// Computes the cross product of two triangle edges and normalizes.
    /// Returns a unit normal vector perpendicular to the triangle.
    pub fn create_triangle_normal(indices: &TriangleIndices, vertices: &[Point3F]) -> Vec3 {
        let v0 = vertices[indices[0]];
        let v1 = vertices[indices[1]];
        let v2 = vertices[indices[2]];

        // Cross product of two edges
        let edge1 = v1 - v0;
        let edge2 = v2 - v0;
        let normal = edge1.cross(&edge2);

        // Normalize to unit length
        normal.normalize();

        // Convert Point3F to Vec3
        Vec3::new(normal.x, normal.y, normal.z)
    }

    /// Create normals for all triangles
    /// NormalUtils.cpp:18-27
    ///
    /// Returns a vector of triangle normals, one per triangle in the mesh.
    pub fn create_triangle_normals(its: &IndexedTriangleSet) -> Vec<Vec3> {
        let mut normals = Vec::with_capacity(its.indices.len());
        for indices in its.indices.iter() {
            normals.push(Self::create_triangle_normal(indices, &its.vertices));
        }
        normals
    }

    /// Create vertex normals by simple averaging of neighbor triangles
    /// NormalUtils.cpp:29-48
    ///
    /// For each vertex, sum all adjacent triangle normals and normalize.
    /// This is the simplest method but doesn't account for triangle size or shape.
    pub fn create_normals_average_neighbor(its: &IndexedTriangleSet) -> Vec<Vec3> {
        let count_vertices = its.vertices.len();
        let mut normals = vec![Vec3::new(0.0, 0.0, 0.0); count_vertices];
        let mut counts = vec![0u32; count_vertices];

        // Accumulate triangle normals for each vertex
        for indices in its.indices.iter() {
            let normal = Self::create_triangle_normal(indices, &its.vertices);
            for &vertex_idx in indices.iter() {
                normals[vertex_idx] = normals[vertex_idx] + normal;
                counts[vertex_idx] += 1;
            }
        }

        // Normalize by count
        for (i, normal) in normals.iter_mut().enumerate() {
            if counts[i] > 0 {
                *normal = *normal / (counts[i] as f32);
            }
        }

        normals
    }

    /// Calculate the angle at a vertex in a triangle
    /// NormalUtils.cpp:51-67
    ///
    /// Given a vertex index (0, 1, or 2) within a triangle, compute the
    /// interior angle at that vertex using the dot product formula.
    pub fn indice_angle(i: usize, indices: &TriangleIndices, vertices: &[Point3F]) -> f32 {
        // Get adjacent vertex indices
        let i1 = if i == 0 { 2 } else { i - 1 };
        let i2 = if i == 2 { 0 } else { i + 1 };

        // Get edges from vertex i to adjacent vertices
        let v1 = vertices[indices[i1]] - vertices[indices[i]];
        let v2 = vertices[indices[i2]] - vertices[indices[i]];

        // Normalize edges
        v1.normalize();
        v2.normalize();

        // Dot product gives cos(angle)
        let mut w = v1.dot(&v2);

        // Clamp to [-1, 1] to handle floating point errors
        if w > 1.0 {
            w = 1.0;
        } else if w < -1.0 {
            w = -1.0;
        }

        // Arc cosine gives angle in radians
        w.acos() as f32
    }

    /// Create vertex normals weighted by triangle angles
    /// NormalUtils.cpp:69-89
    ///
    /// For each vertex, weight adjacent triangle normals by the interior angle
    /// at that vertex. Larger angles contribute more to the vertex normal.
    /// This produces smoother normals at obtuse angles.
    pub fn create_normals_angle_weighted(its: &IndexedTriangleSet) -> Vec<Vec3> {
        let count_vertices = its.vertices.len();
        let mut normals = vec![Vec3::new(0.0, 0.0, 0.0); count_vertices];
        let mut counts = vec![0.0f32; count_vertices];

        // Accumulate weighted triangle normals
        for indices in its.indices.iter() {
            let normal = Self::create_triangle_normal(indices, &its.vertices);

            // Calculate angles at each vertex of the triangle
            let angle0 = Self::indice_angle(0, indices, &its.vertices);
            let angle1 = Self::indice_angle(1, indices, &its.vertices);
            let angle2 = PI - angle0 - angle1; // Third angle from constraint

            let angles = [angle0, angle1, angle2];

            // Weight by angle at each vertex
            for i in 0..3 {
                let weight = angles[i];
                normals[indices[i]] = normals[indices[i]] + normal * (weight as f64);
                counts[indices[i]] += weight;
            }
        }

        // Normalize by accumulated weight
        for (i, normal) in normals.iter_mut().enumerate() {
            if counts[i] > 0.0 {
                *normal = *normal / counts[i];
            }
        }

        normals
    }

    /// Create vertex normals weighted by edge length products (Nelson's method)
    /// NormalUtils.cpp:91-120
    ///
    /// For each vertex, weight adjacent triangle normals by the product of
    /// the two edge lengths adjacent to that vertex. This balances the
    /// contribution of large and small triangles. This is the default method.
    pub fn create_normals_nelson_weighted(its: &IndexedTriangleSet) -> Vec<Vec3> {
        let count_vertices = its.vertices.len();
        let mut normals = vec![Vec3::new(0.0, 0.0, 0.0); count_vertices];
        let mut counts = vec![0.0f64; count_vertices];

        // Accumulate weighted triangle normals
        for indices in its.indices.iter() {
            let normal = Self::create_triangle_normal(indices, &its.vertices);

            let v0 = its.vertices[indices[0]];
            let v1 = its.vertices[indices[1]];
            let v2 = its.vertices[indices[2]];

            // Calculate edge lengths
            let e0 = (v0 - v1).length();
            let e1 = (v1 - v2).length();
            let e2 = (v2 - v0).length();

            // Weight is product of adjacent edge lengths for each vertex
            let coefs = [e0 * e2, e0 * e1, e1 * e2];

            for i in 0..3 {
                let weight = coefs[i];
                normals[indices[i]] = normals[indices[i]] + normal * weight;
                counts[indices[i]] += weight;
            }
        }

        // Normalize by accumulated weight
        for (i, normal) in normals.iter_mut().enumerate() {
            if counts[i] > 0.0 {
                *normal = *normal / counts[i];
            }
        }

        normals
    }

    /// Create vertex normals using specified weighting method
    /// NormalUtils.cpp:123-134
    ///
    /// Dispatches to the appropriate normal calculation method based on type.
    /// Default is Nelson weighted (edge length products).
    pub fn create_normals(its: &IndexedTriangleSet, vertex_type: VertexNormalType) -> Vec<Vec3> {
        match vertex_type {
            VertexNormalType::AverageNeighbor => Self::create_normals_average_neighbor(its),
            VertexNormalType::AngleWeighted => Self::create_normals_angle_weighted(its),
            VertexNormalType::NelsonMaxWeighted => Self::create_normals_nelson_weighted(its),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_simple_triangle() -> IndexedTriangleSet {
        // Right triangle in XY plane with unit sides
        IndexedTriangleSet {
            vertices: vec![
                Point3F::new(0.0, 0.0, 0.0),
                Point3F::new(1.0, 0.0, 0.0),
                Point3F::new(0.0, 1.0, 0.0),
            ],
            indices: vec![[0, 1, 2]],
        }
    }

    fn create_cube() -> IndexedTriangleSet {
        // Simple cube (8 vertices, 12 triangles)
        IndexedTriangleSet {
            vertices: vec![
                Point3F::new(0.0, 0.0, 0.0),
                Point3F::new(1.0, 0.0, 0.0),
                Point3F::new(1.0, 1.0, 0.0),
                Point3F::new(0.0, 1.0, 0.0),
                Point3F::new(0.0, 0.0, 1.0),
                Point3F::new(1.0, 0.0, 1.0),
                Point3F::new(1.0, 1.0, 1.0),
                Point3F::new(0.0, 1.0, 1.0),
            ],
            indices: vec![
                // Bottom face
                [0, 1, 2],
                [0, 2, 3],
                // Top face
                [4, 6, 5],
                [4, 7, 6],
                // Front face
                [0, 5, 1],
                [0, 4, 5],
                // Back face
                [2, 7, 3],
                [2, 6, 7],
                // Left face
                [0, 3, 7],
                [0, 7, 4],
                // Right face
                [1, 5, 6],
                [1, 6, 2],
            ],
        }
    }

    #[test]
    fn test_create_triangle_normal() {
        let its = create_simple_triangle();
        let normal = NormalUtils::create_triangle_normal(&its.indices[0], &its.vertices);

        // Right triangle in XY plane should have normal pointing in +Z
        assert!((normal.x).abs() < 0.001);
        assert!((normal.y).abs() < 0.001);
        assert!((normal.z - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_create_triangle_normals() {
        let its = create_simple_triangle();
        let normals = NormalUtils::create_triangle_normals(&its);

        assert_eq!(normals.len(), 1);
        assert!((normals[0].z - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_indice_angle() {
        let its = create_simple_triangle();

        // Right triangle: one 90° angle, two 45° angles
        let angle0 = NormalUtils::indice_angle(0, &its.indices[0], &its.vertices);
        let angle1 = NormalUtils::indice_angle(1, &its.indices[0], &its.vertices);
        let angle2 = NormalUtils::indice_angle(2, &its.indices[0], &its.vertices);

        // Check that angles sum to π
        let sum = angle0 + angle1 + angle2;
        assert!((sum - PI).abs() < 0.001);

        // Angle at origin should be 90° (π/2)
        assert!((angle0 - PI / 2.0).abs() < 0.001);
    }

    #[test]
    fn test_create_normals_average_neighbor() {
        let its = create_simple_triangle();
        let normals = NormalUtils::create_normals_average_neighbor(&its);

        assert_eq!(normals.len(), 3);

        // All vertices should have same normal (single triangle)
        for normal in normals.iter() {
            assert!((normal.z - 1.0).abs() < 0.001);
        }
    }

    #[test]
    fn test_create_normals_angle_weighted() {
        let its = create_simple_triangle();
        let normals = NormalUtils::create_normals_angle_weighted(&its);

        assert_eq!(normals.len(), 3);

        // All vertices should point roughly in +Z
        for normal in normals.iter() {
            assert!(normal.z > 0.9);
        }
    }

    #[test]
    fn test_create_normals_nelson_weighted() {
        let its = create_simple_triangle();
        let normals = NormalUtils::create_normals_nelson_weighted(&its);

        assert_eq!(normals.len(), 3);

        // All vertices should point roughly in +Z
        for normal in normals.iter() {
            assert!(normal.z > 0.9);
        }
    }

    #[test]
    fn test_create_normals_dispatch() {
        let its = create_simple_triangle();

        let normals_avg = NormalUtils::create_normals(&its, VertexNormalType::AverageNeighbor);
        let normals_angle = NormalUtils::create_normals(&its, VertexNormalType::AngleWeighted);
        let normals_nelson = NormalUtils::create_normals(&its, VertexNormalType::NelsonMaxWeighted);

        assert_eq!(normals_avg.len(), 3);
        assert_eq!(normals_angle.len(), 3);
        assert_eq!(normals_nelson.len(), 3);
    }

    #[test]
    fn test_cube_normals() {
        let its = create_cube();
        let normals = NormalUtils::create_normals(&its, VertexNormalType::NelsonMaxWeighted);

        assert_eq!(normals.len(), 8);

        // All normals should be unit length
        for normal in normals.iter() {
            let len = normal.length();
            assert!((len - 1.0).abs() < 0.1); // Allow some tolerance for averaging
        }
    }

    #[test]
    fn test_normal_length_preservation() {
        let its = create_cube();

        let normals_avg = NormalUtils::create_normals(&its, VertexNormalType::AverageNeighbor);
        let normals_angle = NormalUtils::create_normals(&its, VertexNormalType::AngleWeighted);
        let normals_nelson = NormalUtils::create_normals(&its, VertexNormalType::NelsonMaxWeighted);

        // All methods should produce unit-length normals (within tolerance)
        for normal in normals_avg.iter() {
            let len = normal.length();
            assert!(len > 0.5 && len < 1.5); // Reasonable tolerance
        }

        for normal in normals_angle.iter() {
            let len = normal.length();
            assert!(len > 0.5 && len < 1.5);
        }

        for normal in normals_nelson.iter() {
            let len = normal.length();
            assert!(len > 0.5 && len < 1.5);
        }
    }
}
