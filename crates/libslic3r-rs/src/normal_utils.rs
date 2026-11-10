//! Faithful 1:1 port of `NormalUtils.{hpp,cpp}` from BambuStudio libslic3r.
//!
//! C++ Reference:
//! - src/libslic3r/NormalUtils.hpp (69 lines)
//! - src/libslic3r/NormalUtils.cpp (142 lines)
//!
//! Collection of static functions to create normals.
//!
//! Fidelity notes (byte-exact G-code parity):
//! - C++ stores mesh vertices as `Vec3f` (Eigen `Matrix<float,3,1>`) and triangle
//!   indices as `Vec3crd` (Eigen `Matrix<int,3,1>`); we mirror this with nalgebra
//!   `Vector3<f32>` / `Vector3<i32>` and keep all vector arithmetic in `f32`.
//! - The normals are accumulated and normalized in `f32` exactly as Eigen does. No
//!   division-by-zero guards are added: the C++ divides unconditionally (producing
//!   inf/NaN for isolated vertices) and we reproduce that.
//! - `indice_angle` indexes `vertices` by the *local* corner index `i`/`i1`/`i2`
//!   (NOT `indice[i]`); the `indice` argument is unused by the C++. This is a
//!   faithful reproduction of the upstream behaviour, intentionally preserved.
//! - The `angle_weighted` third angle is computed as `(M_PI - a0 - a1)` in `double`
//!   (because `M_PI` is a `double`) and then stored into a `float` Vec3f component.

use nalgebra::Vector3;

/// 3D single-precision vector, mirroring C++ `Vec3f` (Eigen `Matrix<float,3,1>`).
/// Point.hpp
pub type Vec3f = Vector3<f32>;
/// 3D integer index vector, mirroring C++ `Vec3crd` / `stl_triangle_vertex_indices`.
/// Point.hpp
pub type Vec3crd = Vector3<i32>;
/// Single mesh vertex, mirroring C++ `stl_vertex` (admesh/stl.h => `Vec3f`).
pub type StlVertex = Vec3f;
/// Triangle vertex indices, mirroring C++ `stl_triangle_vertex_indices`.
pub type StlTriangleVertexIndices = Vec3crd;

/// Indexed triangle set, mirroring C++ `indexed_triangle_set` (admesh/stl.h).
///
/// Vertices are stored as `Vec3f` and triangles as `Vec3crd` index triples, accessed
/// via index `[0]`/`[1]`/`[2]` exactly as in the C++ source.
#[derive(Debug, Clone, Default)]
#[allow(non_camel_case_types)]
pub struct indexed_triangle_set {
    /// Vertex positions (single precision), matching C++ `std::vector<stl_vertex>`.
    pub vertices: Vec<StlVertex>,
    /// Triangle vertex indices, matching C++ `std::vector<stl_triangle_vertex_indices>`.
    pub indices: Vec<StlTriangleVertexIndices>,
}

/// `using Normal = Vec3f;`
/// NormalUtils.hpp:16
pub type Normal = Vec3f;
/// `using Normals = std::vector<Normal>;`
/// NormalUtils.hpp:17
pub type Normals = Vec<Normal>;

/// Collection of static function to create normals.
/// NormalUtils.hpp:13-66
///
/// `NormalUtils() = delete;` — only static functions, so this is a unit struct
/// with no constructor.
pub struct NormalUtils;

impl NormalUtils {
    /// Create normal for triangle defined by indices from vertices
    /// NormalUtils.cpp:5-15
    pub fn create_triangle_normal(
        indices: &StlTriangleVertexIndices,
        vertices: &[StlVertex],
    ) -> Vec3f {
        // NormalUtils.cpp:9
        let v0: &StlVertex = &vertices[indices[0] as usize];
        // NormalUtils.cpp:10
        let v1: &StlVertex = &vertices[indices[1] as usize];
        // NormalUtils.cpp:11
        let v2: &StlVertex = &vertices[indices[2] as usize];
        // NormalUtils.cpp:12
        let mut direction: Vec3f = (v1 - v0).cross(&(v2 - v0));
        // NormalUtils.cpp:13
        direction.normalize_mut();
        // NormalUtils.cpp:14
        direction
    }

    /// Create normals for each triangle.
    /// NormalUtils.cpp:17-26
    pub fn create_triangle_normals(its: &indexed_triangle_set) -> Vec<Vec3f> {
        // NormalUtils.cpp:20
        let mut normals: Vec<Vec3f> = Vec::new();
        // NormalUtils.cpp:21
        normals.reserve(its.indices.len());
        // NormalUtils.cpp:22-24
        for index in its.indices.iter() {
            normals.push(Self::create_triangle_normal(index, &its.vertices));
        }
        // NormalUtils.cpp:25
        normals
    }

    /// Create normals for each vertex by averaging neighbor triangles normal.
    /// NormalUtils.cpp:28-47
    pub fn create_normals_average_neighbor(its: &indexed_triangle_set) -> Vec<Vec3f> {
        // NormalUtils.cpp:31
        let count_vertices: usize = its.vertices.len();
        // NormalUtils.cpp:32
        let mut normals: Vec<Vec3f> = vec![Vec3f::new(0.0, 0.0, 0.0); count_vertices];
        // NormalUtils.cpp:33
        let mut count: Vec<u32> = vec![0u32; count_vertices];
        // NormalUtils.cpp:34
        for indice in its.indices.iter() {
            // NormalUtils.cpp:35
            let normal: Vec3f = Self::create_triangle_normal(indice, &its.vertices);
            // NormalUtils.cpp:36-39
            for i in 0..3 {
                normals[indice[i] as usize] += normal;
                count[indice[i] as usize] += 1;
            }
        }
        // normalize to size 1
        // NormalUtils.cpp:41-45
        for index in 0..normals.len() {
            normals[index] /= count[index] as f32;
        }
        // NormalUtils.cpp:46
        normals
    }

    /// calc triangle angle of vertex defined by index to triangle indices
    /// NormalUtils.cpp:49-69
    pub fn indice_angle(i: i32, _indice: &Vec3crd, vertices: &[StlVertex]) -> f32 {
        // NormalUtils.cpp:54
        let i1: i32 = if i == 0 { 2 } else { i - 1 };
        // NormalUtils.cpp:55
        let i2: i32 = if i == 2 { 0 } else { i + 1 };

        // NormalUtils.cpp:57 — NOTE: indexes `vertices` by the local index, not indice[i].
        let mut v1: Vec3f = vertices[i1 as usize] - vertices[i as usize];
        // NormalUtils.cpp:58
        let mut v2: Vec3f = vertices[i2 as usize] - vertices[i as usize];

        // NormalUtils.cpp:60
        v1.normalize_mut();
        // NormalUtils.cpp:61
        v2.normalize_mut();

        // NormalUtils.cpp:63
        let mut w: f32 = v1.dot(&v2);
        // NormalUtils.cpp:64-67
        if w > 1.0 {
            w = 1.0;
        } else if w < -1.0 {
            w = -1.0;
        }
        // NormalUtils.cpp:68
        w.acos()
    }

    /// Create normals for each vertex weighted by triangle angles.
    /// NormalUtils.cpp:71-94
    pub fn create_normals_angle_weighted(its: &indexed_triangle_set) -> Vec<Vec3f> {
        // NormalUtils.cpp:74
        let count_vertices: usize = its.vertices.len();
        // NormalUtils.cpp:75
        let mut normals: Vec<Vec3f> = vec![Vec3f::new(0.0, 0.0, 0.0); count_vertices];
        // NormalUtils.cpp:76
        let mut count: Vec<f32> = vec![0.0f32; count_vertices];
        // NormalUtils.cpp:77
        for indice in its.indices.iter() {
            // NormalUtils.cpp:78
            let normal: Vec3f = Self::create_triangle_normal(indice, &its.vertices);
            // NormalUtils.cpp:79-80
            let mut angles: Vec3f = Vec3f::new(
                Self::indice_angle(0, indice, &its.vertices),
                Self::indice_angle(1, indice, &its.vertices),
                0.0,
            );
            // NormalUtils.cpp:81 — (M_PI - angles[0] - angles[1]) computed in double, stored f32.
            angles[2] = (std::f64::consts::PI - angles[0] as f64 - angles[1] as f64) as f32;
            // NormalUtils.cpp:82-86
            for i in 0..3 {
                let weight: f32 = angles[i];
                normals[indice[i] as usize] += normal * weight;
                count[indice[i] as usize] += weight;
            }
        }
        // normalize to size 1
        // NormalUtils.cpp:88-92
        for index in 0..normals.len() {
            normals[index] /= count[index];
        }
        // NormalUtils.cpp:93
        normals
    }

    /// Create normals for each vertex weighted by edge-length products (Nelson).
    /// NormalUtils.cpp:96-127
    pub fn create_normals_nelson_weighted(its: &indexed_triangle_set) -> Vec<Vec3f> {
        // NormalUtils.cpp:99
        let count_vertices: usize = its.vertices.len();
        // NormalUtils.cpp:100
        let mut normals: Vec<Vec3f> = vec![Vec3f::new(0.0, 0.0, 0.0); count_vertices];
        // NormalUtils.cpp:101
        let mut count: Vec<f32> = vec![0.0f32; count_vertices];
        // NormalUtils.cpp:102
        let vertices: &[StlVertex] = &its.vertices;
        // NormalUtils.cpp:103
        for indice in its.indices.iter() {
            // NormalUtils.cpp:104
            let normal: Vec3f = Self::create_triangle_normal(indice, vertices);

            // NormalUtils.cpp:106
            let v0: &StlVertex = &vertices[indice[0] as usize];
            // NormalUtils.cpp:107
            let v1: &StlVertex = &vertices[indice[1] as usize];
            // NormalUtils.cpp:108
            let v2: &StlVertex = &vertices[indice[2] as usize];

            // NormalUtils.cpp:110
            let e0: f32 = (v0 - v1).norm();
            // NormalUtils.cpp:111
            let e1: f32 = (v1 - v2).norm();
            // NormalUtils.cpp:112
            let e2: f32 = (v2 - v0).norm();

            // NormalUtils.cpp:114
            let coefs: Vec3f = Vec3f::new(e0 * e2, e0 * e1, e1 * e2);
            // NormalUtils.cpp:115-119
            for i in 0..3 {
                let weight: f32 = coefs[i];
                normals[indice[i] as usize] += normal * weight;
                count[indice[i] as usize] += weight;
            }
        }
        // normalize to size 1
        // NormalUtils.cpp:121-125
        for index in 0..normals.len() {
            normals[index] /= count[index];
        }
        // NormalUtils.cpp:126
        normals
    }

    /// calculate normals by averaging normals of neghbor triangles
    /// NormalUtils.cpp:129-142
    pub fn create_normals(its: &indexed_triangle_set, type_: VertexNormalType) -> Vec<Vec3f> {
        // NormalUtils.cpp:133-141
        match type_ {
            // NormalUtils.cpp:134-135
            VertexNormalType::AverageNeighbor => Self::create_normals_average_neighbor(its),
            // NormalUtils.cpp:136-137
            VertexNormalType::AngleWeighted => Self::create_normals_angle_weighted(its),
            // NormalUtils.cpp:138-140
            VertexNormalType::NelsonMaxWeighted => Self::create_normals_nelson_weighted(its),
        }
    }
}

/// Type of vertex normal calculation.
/// NormalUtils.hpp:20-24
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VertexNormalType {
    /// NormalUtils.hpp:21
    AverageNeighbor,
    /// NormalUtils.hpp:22
    AngleWeighted,
    /// NormalUtils.hpp:23
    NelsonMaxWeighted,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_simple_triangle() -> indexed_triangle_set {
        // Right triangle in XY plane with unit sides.
        indexed_triangle_set {
            vertices: vec![
                Vec3f::new(0.0, 0.0, 0.0),
                Vec3f::new(1.0, 0.0, 0.0),
                Vec3f::new(0.0, 1.0, 0.0),
            ],
            indices: vec![Vec3crd::new(0, 1, 2)],
        }
    }

    #[test]
    fn test_create_triangle_normal() {
        let its = create_simple_triangle();
        let normal = NormalUtils::create_triangle_normal(&its.indices[0], &its.vertices);
        // Right triangle in XY plane should have normal pointing in +Z.
        assert!(normal.x.abs() < 0.001);
        assert!(normal.y.abs() < 0.001);
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
    fn test_create_normals_dispatch() {
        let its = create_simple_triangle();
        let normals_avg = NormalUtils::create_normals(&its, VertexNormalType::AverageNeighbor);
        let normals_angle = NormalUtils::create_normals(&its, VertexNormalType::AngleWeighted);
        let normals_nelson = NormalUtils::create_normals(&its, VertexNormalType::NelsonMaxWeighted);
        assert_eq!(normals_avg.len(), 3);
        assert_eq!(normals_angle.len(), 3);
        assert_eq!(normals_nelson.len(), 3);
    }
}
