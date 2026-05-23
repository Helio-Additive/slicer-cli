//! Mesh decimation using quadric error metrics.
//!
//! This module provides mesh simplification via the quadric edge collapse algorithm,
//! mirroring BambuStudio's QuadricEdgeCollapse.cpp.

use crate::geometry::Point3F;
use crate::triangle_mesh::TriangleMesh;

#[derive(Debug, Clone, Copy)]
/// Symmetric matrix (SymMat) for quadric error metric, stores 10 unique elements of a 4x4 symmetric matrix
/// QuadricEdgeCollapse.cpp:16-51
pub struct Quadric {
    /// 4x4 symmetric matrix stored as 10 unique elements
    /// QuadricEdgeCollapse.cpp:19
    pub data: [f64; 10],
}

/// Quadric symmetric matrix implementation
/// QuadricEdgeCollapse.cpp:16-51
impl Quadric {
    // Create a new zero-initialized quadric symmetric matrix
    // QuadricEdgeCollapse.cpp:21
    pub fn new() -> Self {
        // QuadricEdgeCollapse.cpp:21
        Self { data: [0.0; 10] }
    }

    /// Add another quadric to this one, accumulating symmetric matrices
    /// QuadricEdgeCollapse.cpp:46-50
    pub fn add(&mut self, other: &Quadric) {
        // QuadricEdgeCollapse.cpp:48
        for i in 0..10 {
            // QuadricEdgeCollapse.cpp:48
            self.data[i] += other.data[i];
        }
    }

    /// Evaluate the quadric error at a 3D point using quadratic form
    /// QuadricEdgeCollapse.cpp:424-430
    pub fn evaluate(&self, p: &Point3F) -> f64 {
        // QuadricEdgeCollapse.cpp:426
        let x = p.x;
        // QuadricEdgeCollapse.cpp:426
        let y = p.y;
        // QuadricEdgeCollapse.cpp:426
        let z = p.z;
        // QuadricEdgeCollapse.cpp:426
        let a = &self.data;

        // QuadricEdgeCollapse.cpp:427-429
        a[0] * x * x
            + 2.0 * a[1] * x * y
            + 2.0 * a[2] * x * z
            + 2.0 * a[3] * x
            + a[4] * y * y
            + 2.0 * a[5] * y * z
            + 2.0 * a[6] * y
            + a[7] * z * z
            + 2.0 * a[8] * z
            + a[9]
    }
}

/// Default implementation for Quadric, creates zero-initialized symmetric matrix
/// QuadricEdgeCollapse.cpp:21
impl Default for Quadric {
    // Return zero-initialized Quadric
    // QuadricEdgeCollapse.cpp:21
    fn default() -> Self {
        // QuadricEdgeCollapse.cpp:21
        Self::new()
    }
}

/// Mesh decimator using quadric error metrics for edge collapse simplification
/// QuadricEdgeCollapse.hpp:21-26
pub struct QuadricEdgeCollapse {
    /// Target triangle count, 0 means use ratio
    /// QuadricEdgeCollapse.hpp:23
    pub target_triangles: usize,
    /// Target ratio of triangles to keep, 0.5 keeps 50 percent
    /// QuadricEdgeCollapse.hpp:23
    pub target_ratio: f64,
    /// Maximum error threshold for edge collapse
    /// QuadricEdgeCollapse.hpp:24
    pub max_error: f64,
    /// Whether to preserve boundary edges
    /// QuadricEdgeCollapse.cpp:123-128
    pub preserve_boundaries: bool,
    /// Whether to preserve UV boundaries
    /// QuadricEdgeCollapse.cpp:123-128
    pub preserve_uvs: bool,
}

/// QuadricEdgeCollapse implementation for mesh decimation
/// QuadricEdgeCollapse.cpp:160-347
impl QuadricEdgeCollapse {
    // Create a new decimator with default settings
    // QuadricEdgeCollapse.cpp:160-166
    pub fn new() -> Self {
        // QuadricEdgeCollapse.hpp:23-24
        Self {
            target_triangles: 0,
            target_ratio: 0.5,
            max_error: f64::INFINITY,
            preserve_boundaries: true,
            preserve_uvs: false,
        }
    }

    /// Set target triangle count using builder pattern
    /// QuadricEdgeCollapse.hpp:23
    pub fn target_triangles(mut self, count: usize) -> Self {
        // QuadricEdgeCollapse.hpp:23
        self.target_triangles = count;
        // QuadricEdgeCollapse.hpp:23
        self.target_ratio = 0.0;
        // QuadricEdgeCollapse.hpp:23
        self
    }

    /// Set target ratio of triangles to keep using builder pattern
    /// QuadricEdgeCollapse.hpp:23
    pub fn target_ratio(mut self, ratio: f64) -> Self {
        // QuadricEdgeCollapse.hpp:23
        self.target_ratio = ratio.clamp(0.0, 1.0);
        // QuadricEdgeCollapse.hpp:23
        self.target_triangles = 0;
        // QuadricEdgeCollapse.hpp:23
        self
    }

    /// Set maximum error threshold for edge collapse
    /// QuadricEdgeCollapse.hpp:24
    pub fn max_error(mut self, error: f64) -> Self {
        // QuadricEdgeCollapse.cpp:169
        self.max_error = error;
        // QuadricEdgeCollapse.hpp:24
        self
    }

    /// Decimate a mesh using quadric edge collapse algorithm, currently a stub
    /// QuadricEdgeCollapse.cpp:160-347
    pub fn decimate(&self, mesh: &TriangleMesh) -> TriangleMesh {
        // QuadricEdgeCollapse.cpp:168
        // TODO: Implement full edge collapse loop (QuadricEdgeCollapse.cpp:160-347)
        mesh.clone()
    }

    /// Compute quadrics for all vertices by summing per-triangle plane quadrics
    /// QuadricEdgeCollapse.cpp:440-536
    fn compute_quadrics(&self, mesh: &TriangleMesh) -> Vec<Quadric> {
        // QuadricEdgeCollapse.cpp:445
        let vertex_count = mesh.vertex_count();
        // QuadricEdgeCollapse.cpp:445
        let mut quadrics = vec![Quadric::new(); vertex_count];

        // QuadricEdgeCollapse.cpp:449-462
        for tri_idx in 0..mesh.triangle_count() {
            // QuadricEdgeCollapse.cpp:456
            let plane_quadric = self.compute_plane_quadric(mesh, tri_idx);
            // QuadricEdgeCollapse.cpp:456
            let indices = mesh.triangle_indices(tri_idx);
            // QuadricEdgeCollapse.cpp:466-473
            for &idx in &indices {
                // QuadricEdgeCollapse.cpp:472
                quadrics[idx as usize].add(&plane_quadric);
            }
        }

        // QuadricEdgeCollapse.cpp:536
        quadrics
    }

    /// Compute quadric for a plane defined by a triangle using plane equation
    /// QuadricEdgeCollapse.cpp:432-438
    fn compute_plane_quadric(&self, mesh: &TriangleMesh, tri_idx: usize) -> Quadric {
        // QuadricEdgeCollapse.cpp:454
        let normal = mesh.triangle_normal(tri_idx);
        // QuadricEdgeCollapse.cpp:454
        let area = mesh.triangle_area(tri_idx);
        // QuadricEdgeCollapse.cpp:436
        let vertices = mesh.triangle_vertices(tri_idx);
        // QuadricEdgeCollapse.cpp:436
        let p = vertices[0];

        // QuadricEdgeCollapse.cpp:437
        let a = normal.x;
        // QuadricEdgeCollapse.cpp:437
        let b = normal.y;
        // QuadricEdgeCollapse.cpp:437
        let c = normal.z;
        // QuadricEdgeCollapse.cpp:437
        let d = -(a * p.x + b * p.y + c * p.z);

        // QuadricEdgeCollapse.cpp:24-29
        let scale = area;

        // QuadricEdgeCollapse.cpp:24-29
        Quadric {
            data: [
                a * a * scale,
                a * b * scale,
                a * c * scale,
                a * d * scale,
                b * b * scale,
                b * c * scale,
                b * d * scale,
                c * c * scale,
                c * d * scale,
                d * d * scale,
            ],
        }
    }
}

/// Default implementation for QuadricEdgeCollapse with standard settings
/// QuadricEdgeCollapse.cpp:160-166
impl Default for QuadricEdgeCollapse {
    // Return QuadricEdgeCollapse with default parameters
    // QuadricEdgeCollapse.cpp:160-166
    fn default() -> Self {
        // QuadricEdgeCollapse.cpp:160
        Self::new()
    }
}

/// Simplify a mesh using quadric edge collapse with a target ratio
/// QuadricEdgeCollapse.hpp:21-26
pub fn simplify_mesh(mesh: &TriangleMesh, target_ratio: f64) -> TriangleMesh {
    // QuadricEdgeCollapse.cpp:160
    let decimator = QuadricEdgeCollapse::new().target_ratio(target_ratio);
    // QuadricEdgeCollapse.cpp:160
    decimator.decimate(mesh)
}
