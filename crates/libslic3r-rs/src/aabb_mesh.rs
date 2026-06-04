//! AABB mesh structure for ray casting and spatial queries
//!
//! An index-triangle structure coupled with an AABB tree to support ray
//! casting, distance queries, and other higher level geometric operations.
//!
//! C++ Reference:
//! - AABBMesh.hpp (155 lines)
//! - AABBMesh.cpp (323 lines)

use crate::aabb_tree_indirect::{self, Node, Tree3F};
use crate::geometry::{BoundingBox3F, Point3F, Vec3};
use crate::CoordF;

/// Indexed triangle set representation
///
/// This is a simple structure holding vertices and triangle indices.
/// Model.hpp
#[derive(Debug, Clone)]
pub struct IndexedTriangleSet {
    /// Vertex positions (3D points)
    /// Model.hpp
    pub vertices: Vec<Point3F>,

    /// Triangle indices (each triangle references 3 vertices)
    /// Model.hpp
    pub indices: Vec<[usize; 3]>,
}

impl IndexedTriangleSet {
    /// Create a new empty indexed triangle set
    pub fn new() -> Self {
        Self {
            vertices: Vec::new(),
            indices: Vec::new(),
        }
    }

    /// Create from vertices and indices
    pub fn from_parts(vertices: Vec<Point3F>, indices: Vec<[usize; 3]>) -> Self {
        Self { vertices, indices }
    }
}

impl Default for IndexedTriangleSet {
    fn default() -> Self {
        Self::new()
    }
}

/// Vertex-face index mapping
///
/// Maps each vertex to the faces (triangles) that use it.
/// AABBMesh.hpp:34
#[derive(Debug, Clone, Default)]
pub struct VertexFaceIndex {
    /// For each vertex, list of face indices that reference it
    /// AABBMesh.hpp
    vertex_to_faces: Vec<Vec<usize>>,
}

impl VertexFaceIndex {
    /// Build vertex-face index from indexed triangle set
    ///
    /// AABBMesh.cpp:87
    pub fn from_its(its: &IndexedTriangleSet) -> Self {
        // AABBMesh.cpp:87-95
        let mut vertex_to_faces = vec![Vec::new(); its.vertices.len()];

        for (face_idx, triangle) in its.indices.iter().enumerate() {
            for &vertex_idx in triangle.iter() {
                vertex_to_faces[vertex_idx].push(face_idx);
            }
        }

        Self { vertex_to_faces }
    }

    /// Get faces connected to a vertex
    pub fn faces_from_vertex(&self, vertex_idx: usize) -> &[usize] {
        self.vertex_to_faces
            .get(vertex_idx)
            .map(|v| v.as_slice())
            .unwrap_or(&[])
    }
}

/// Result of a ray cast operation on the mesh
///
/// AABBMesh.hpp:71-104
#[derive(Debug, Clone)]
pub struct HitResult {
    /// Distance from source to intersection
    /// AABBMesh.hpp:73
    t: CoordF,

    /// Face ID that was hit (-1 if no hit)
    /// AABBMesh.hpp:74
    face_id: i32,

    /// Ray direction
    /// AABBMesh.hpp:76
    dir: Vec3,

    /// Ray source point
    /// AABBMesh.hpp:77
    source: Vec3,

    /// Normal at hit point
    /// AABBMesh.hpp:78
    normal: Vec3,

    /// Whether this result is valid (has mesh reference)
    /// AABBMesh.hpp:75
    is_valid_result: bool,
}

impl HitResult {
    /// Sentinel value for no intersection
    ///
    /// AABBMesh.hpp:87
    pub fn infty() -> CoordF {
        // AABBMesh.hpp:87
        CoordF::INFINITY
    }

    /// Create a new hit result with infinite distance (no hit)
    ///
    /// AABBMesh.hpp:89
    pub fn new() -> Self {
        // AABBMesh.hpp:89
        Self {
            t: Self::infty(),
            face_id: -1,
            dir: Vec3::new(0.0, 0.0, 0.0),
            source: Vec3::new(0.0, 0.0, 0.0),
            normal: Vec3::new(0.0, 0.0, 0.0),
            is_valid_result: false,
        }
    }

    /// Create a hit result with specific values
    ///
    /// AABBMesh.cpp:174
    fn with_values(t: CoordF, face_id: i32, dir: Vec3, source: Vec3, normal: Vec3) -> Self {
        Self {
            t,
            face_id,
            dir,
            source,
            normal,
            is_valid_result: true,
        }
    }

    /// Get distance to hit point
    ///
    /// AABBMesh.hpp:91
    pub fn distance(&self) -> CoordF {
        // AABBMesh.hpp:91
        self.t
    }

    /// Get ray direction
    ///
    /// AABBMesh.hpp:92
    pub fn direction(&self) -> Vec3 {
        // AABBMesh.hpp:92
        self.dir
    }

    /// Get ray source
    ///
    /// AABBMesh.hpp:93
    pub fn source(&self) -> Vec3 {
        // AABBMesh.hpp:93
        self.source
    }

    /// Get hit position
    ///
    /// AABBMesh.hpp:94
    pub fn position(&self) -> Vec3 {
        // AABBMesh.hpp:94
        // C++: return m_source + m_dir * m_t;
        Vec3::new(
            self.source.x + self.dir.x * self.t,
            self.source.y + self.dir.y * self.t,
            self.source.z + self.dir.z * self.t,
        )
    }

    /// Get face ID
    ///
    /// AABBMesh.hpp:95
    pub fn face(&self) -> i32 {
        // AABBMesh.hpp:95
        self.face_id
    }

    /// Check if this is a valid hit result
    ///
    /// AABBMesh.hpp:96
    pub fn is_valid(&self) -> bool {
        // AABBMesh.hpp:96
        self.is_valid_result
    }

    /// Check if ray actually hit the mesh
    ///
    /// AABBMesh.hpp:97
    pub fn is_hit(&self) -> bool {
        // AABBMesh.hpp:97
        // C++: return m_face_id >= 0 && !std::isinf(m_t);
        self.face_id >= 0 && !self.t.is_infinite()
    }

    /// Get normal at hit point
    ///
    /// AABBMesh.hpp:99-102
    pub fn normal(&self) -> Vec3 {
        // AABBMesh.hpp:100
        assert!(self.is_valid());
        // AABBMesh.hpp:101
        self.normal
    }

    /// Check if ray hit from inside
    ///
    /// AABBMesh.hpp:103
    pub fn is_inside(&self) -> bool {
        // AABBMesh.hpp:104
        // C++: return is_hit() && normal().dot(m_dir) > 0;
        self.is_hit() && {
            let dot = self.normal.x * self.dir.x
                + self.normal.y * self.dir.y
                + self.normal.z * self.dir.z;
            dot > 0.0
        }
    }
}

impl Default for HitResult {
    /// Create default hit result (no hit)
    ///
    /// AABBMesh.hpp:89
    fn default() -> Self {
        Self::new()
    }
}

/// AABB mesh structure for spatial queries
///
/// AABBMesh.hpp:26-155
pub struct AABBMesh {
    /// Reference to the indexed triangle set
    /// AABBMesh.hpp:29
    its: IndexedTriangleSet,

    /// AABB tree for accelerated spatial queries
    /// AABBMesh.hpp:31
    /// AABBMesh.cpp:18
    aabb_tree: Tree3F,

    /// Vertex-face index
    /// AABBMesh.hpp:33
    vfidx: VertexFaceIndex,

    /// Face-neighbor index
    /// AABBMesh.hpp:34
    fnidx: Vec<[i32; 3]>,

    /// Triangle-ray intersection epsilon
    /// AABBMesh.cpp:22
    triangle_ray_epsilon: CoordF,
}

impl AABBMesh {
    /// Construct AABB mesh from indexed triangle set
    ///
    /// AABBMesh.cpp:78-85
    /// AABBMesh.hpp:48
    pub fn new(its: IndexedTriangleSet, calculate_epsilon: bool) -> Self {
        // AABBMesh.cpp:79-84
        // Calculate epsilon from average triangle edge length if requested
        let triangle_ray_epsilon = if calculate_epsilon {
            // AABBMesh.cpp:24
            let avg_edge_length = compute_average_edge_length(&its);
            if avg_edge_length > 0.0 {
                0.000001 * avg_edge_length * avg_edge_length
            } else {
                0.000001
            }
        } else {
            0.000001
        };

        // Build vertex-face index
        // AABBMesh.cpp:82
        let vfidx = VertexFaceIndex::from_its(&its);

        // Build face-neighbor index
        // AABBMesh.cpp:83
        let fnidx = compute_face_neighbors(&its);

        // Build AABB tree
        // AABBMesh.cpp:29-31
        // C++: m_tree = AABBTreeIndirect::build_aabb_tree_over_indexed_triangle_set(
        // C++:     its.vertices, its.indices);
        let aabb_tree = aabb_tree_indirect::build_aabb_tree_over_indexed_triangle_set(
            &its.vertices,
            &its.indices,
        );

        Self {
            its,
            aabb_tree,
            vfidx,
            fnidx,
            triangle_ray_epsilon,
        }
    }

    /// Construct from TriangleMesh
    ///
    /// AABBMesh.cpp:87-95
    /// AABBMesh.hpp:49
    /// TODO: Implement when TriangleMesh is ported
    // pub fn from_triangle_mesh(mesh: &TriangleMesh, calculate_epsilon: bool) -> Self {
    //     // Convert TriangleMesh to IndexedTriangleSet
    //     // AABBMesh.cpp:88
    //     let its = mesh.to_indexed_triangle_set();
    //
    //     // AABBMesh.cpp:89-94
    //     Self::new(its, calculate_epsilon)
    // }

    /// Get vertices
    ///
    /// AABBMesh.cpp:131-134
    pub fn vertices(&self) -> &[Point3F] {
        // AABBMesh.cpp:132
        &self.its.vertices
    }

    /// Get indices
    ///
    /// AABBMesh.cpp:138-141
    pub fn indices(&self) -> &[[usize; 3]] {
        // AABBMesh.cpp:139
        &self.its.indices
    }

    /// Get vertex by index
    ///
    /// AABBMesh.cpp:145-148
    pub fn vertex(&self, idx: usize) -> Point3F {
        // AABBMesh.cpp:146
        self.its.vertices[idx]
    }

    /// Get triangle indices by index
    ///
    /// AABBMesh.cpp:152-155
    pub fn triangle(&self, idx: usize) -> [usize; 3] {
        // AABBMesh.cpp:153
        self.its.indices[idx]
    }

    /// Get the indexed triangle set
    ///
    /// AABBMesh.hpp:144
    pub fn get_triangle_mesh(&self) -> &IndexedTriangleSet {
        // AABBMesh.hpp:144
        &self.its
    }

    /// Get vertex-face index
    ///
    /// AABBMesh.hpp:146
    pub fn vertex_face_index(&self) -> &VertexFaceIndex {
        // AABBMesh.hpp:146
        &self.vfidx
    }

    /// Get face-neighbor index
    ///
    /// AABBMesh.hpp:147
    pub fn face_neighbor_index(&self) -> &[[i32; 3]] {
        // AABBMesh.hpp:147
        &self.fnidx
    }

    /// Compute normal for a face
    ///
    /// AABBMesh.cpp:159-162
    pub fn normal_by_face_id(&self, face_id: usize) -> Vec3 {
        // AABBMesh.cpp:160
        // C++: return its_unnormalized_normal(*m_tm, face_id).cast<double>().normalized();
        compute_triangle_normal(&self.its, face_id)
    }

    /// Cast a ray on the mesh, returns the first hit
    ///
    /// AABBMesh.cpp:165-192
    /// AABBMesh.hpp:127
    pub fn query_ray_hit(&self, source: Vec3, dir: Vec3) -> HitResult {
        // AABBMesh.cpp:167
        // C++: assert(is_approx(dir.norm(), 1.));
        // Direction should be normalized
        debug_assert!(
            (dir.norm() - 1.0).abs() < 1e-6,
            "Ray direction must be normalized"
        );

        // AABBMesh.cpp:180
        // C++: m_aabb->intersect_ray(*m_tm, s, dir, hit);
        // Convert Vec3 to Point3F for AABB tree interface
        let source_pt = Point3F {
            x: source.x,
            y: source.y,
            z: source.z,
        };
        let dir_pt = Point3F {
            x: dir.x,
            y: dir.y,
            z: dir.z,
        };

        let hit_opt = aabb_tree_indirect::intersect_ray_first_hit(
            &self.its.vertices,
            &self.its.indices,
            &self.aabb_tree,
            &source_pt,
            &dir_pt,
        );

        // AABBMesh.cpp:181-188
        // C++: AABBMesh::hit_result ret(*this);
        // C++: ret.m_t = double(hit.t);
        // C++: ret.m_source = s;
        // C++: ret.m_dir = dir;
        // C++: ret.m_face_id = hit.id;
        if let Some((t, face_id, _normal)) = hit_opt {
            HitResult {
                t,
                face_id: face_id as i32,
                dir,
                source,
                normal: self.compute_face_normal(face_id),
                is_valid_result: true,
            }
        } else {
            HitResult::new()
        }
    }

    /// Cast a ray on the mesh and return all hits
    ///
    /// AABBMesh.cpp:194-230
    /// AABBMesh.hpp:130
    pub fn query_ray_hits(&self, source: Vec3, dir: Vec3) -> Vec<HitResult> {
        // AABBMesh.cpp:196
        // C++: assert(is_approx(dir.norm(), 1.));
        debug_assert!(
            (dir.norm() - 1.0).abs() < 1e-6,
            "Ray direction must be normalized"
        );

        // AABBMesh.cpp:198-227
        // C++: std::vector<igl::Hit> hits;
        // C++: m_aabb->intersect_ray(*m_tm, s, dir, hits);
        // Convert Vec3 to Point3F for AABB tree interface
        let source_pt = Point3F {
            x: source.x,
            y: source.y,
            z: source.z,
        };
        let dir_pt = Point3F {
            x: dir.x,
            y: dir.y,
            z: dir.z,
        };

        let hits = aabb_tree_indirect::intersect_ray_all_hits(
            &self.its.vertices,
            &self.its.indices,
            &self.aabb_tree,
            &source_pt,
            &dir_pt,
        );

        // Convert hits to HitResult format
        hits.into_iter()
            .map(|(t, face_id, _normal)| HitResult {
                t,
                face_id: face_id as i32,
                dir,
                source,
                normal: self.compute_face_normal(face_id),
                is_valid_result: true,
            })
            .collect()
    }

    /// Compute squared distance to a point, with closest point and face
    ///
    /// AABBMesh.cpp:313-323
    /// AABBMesh.hpp:132-137
    pub fn squared_distance(&self, point: Vec3) -> (CoordF, i32, Vec3) {
        // AABBMesh.cpp:49-58
        // C++: size_t idx_unsigned = 0;
        // C++: Vec3d  closest_vec3d(closest);
        // C++: double dist =
        // C++:     AABBTreeIndirect::squared_distance_to_indexed_triangle_set(
        // C++:         its.vertices, its.indices, m_tree, point, idx_unsigned,
        // C++:         closest_vec3d);
        let (dist_sq, face_idx, closest_point) =
            aabb_tree_indirect::squared_distance_to_indexed_triangle_set(
                &self.its.vertices,
                &self.its.indices,
                &self.aabb_tree,
                point,
            );

        // AABBMesh.cpp:314-320
        (dist_sq, face_idx as i32, closest_point)
    }

    /// Compute squared distance to a point (simple version)
    ///
    /// AABBMesh.hpp:138-142
    pub fn squared_distance_simple(&self, point: Vec3) -> CoordF {
        // AABBMesh.hpp:140-141
        // C++: int   i;
        // C++: Vec3d c;
        // C++: return squared_distance(p, i, c);
        self.squared_distance(point).0
    }

    /// Compute face normal by face ID
    ///
    /// AABBMesh.cpp:156-159
    /// AABBMesh.hpp:144
    fn compute_face_normal(&self, face_id: usize) -> Vec3 {
        // AABBMesh.cpp:157
        // C++: return its_unnormalized_normal(*m_tm, face_id).cast<double>().normalized();
        if face_id >= self.its.indices.len() {
            return Vec3::new(0.0, 0.0, 0.0);
        }

        let tri = self.its.indices[face_id];
        let v0 = self.its.vertices[tri[0]];
        let v1 = self.its.vertices[tri[1]];
        let v2 = self.its.vertices[tri[2]];

        // Compute unnormalized normal via cross product
        let e1 = Vec3::new(
            (v1.x() - v0.x()) as CoordF,
            (v1.y() - v0.y()) as CoordF,
            (v1.z() - v0.z()) as CoordF,
        );
        let e2 = Vec3::new(
            (v2.x() - v0.x()) as CoordF,
            (v2.y() - v0.y()) as CoordF,
            (v2.z() - v0.z()) as CoordF,
        );

        let normal = e1.cross(&e2);
        let length = normal.norm();
        if length > 1e-10 {
            normal / length
        } else {
            Vec3::new(0.0, 0.0, 1.0) // Degenerate triangle, return up vector
        }
    }
}

impl Clone for AABBMesh {
    /// Clone the AABB mesh
    ///
    /// AABBMesh.cpp:98-104
    fn clone(&self) -> Self {
        // AABBMesh.cpp:99-103
        Self {
            its: self.its.clone(),
            aabb_tree: self.aabb_tree.clone(),
            vfidx: self.vfidx.clone(),
            fnidx: self.fnidx.clone(),
            triangle_ray_epsilon: self.triangle_ray_epsilon,
        }
    }
}

/// Compute average edge length of indexed triangle set
///
/// AABBMesh.cpp:24
fn compute_average_edge_length(its: &IndexedTriangleSet) -> CoordF {
    // AABBMesh.cpp:24-27
    if its.indices.is_empty() {
        return 0.0;
    }

    let mut total_length = 0.0;
    let mut edge_count = 0;

    for triangle in &its.indices {
        let v0 = its.vertices[triangle[0]];
        let v1 = its.vertices[triangle[1]];
        let v2 = its.vertices[triangle[2]];

        // Edge 0-1
        let dx = v1.x - v0.x;
        let dy = v1.y - v0.y;
        let dz = v1.z - v0.z;
        total_length += (dx * dx + dy * dy + dz * dz).sqrt();

        // Edge 1-2
        let dx = v2.x - v1.x;
        let dy = v2.y - v1.y;
        let dz = v2.z - v1.z;
        total_length += (dx * dx + dy * dy + dz * dz).sqrt();

        // Edge 2-0
        let dx = v0.x - v2.x;
        let dy = v0.y - v2.y;
        let dz = v0.z - v2.z;
        total_length += (dx * dx + dy * dy + dz * dz).sqrt();

        edge_count += 3;
    }

    if edge_count > 0 {
        total_length / edge_count as CoordF
    } else {
        0.0
    }
}

/// Compute face neighbor index for indexed triangle set
///
/// Returns for each face the indices of neighboring faces (or -1 if no neighbor)
/// AABBMesh.cpp:83
fn compute_face_neighbors(its: &IndexedTriangleSet) -> Vec<[i32; 3]> {
    // TODO: Implement its_face_neighbors when porting utilities
    // For now, return empty neighbors
    vec![[-1, -1, -1]; its.indices.len()]
}

/// Compute unnormalized normal for a triangle
///
/// AABBMesh.cpp:160
fn compute_triangle_normal(its: &IndexedTriangleSet, face_id: usize) -> Vec3 {
    // Get triangle vertices
    let triangle = its.indices[face_id];
    let v0 = its.vertices[triangle[0]];
    let v1 = its.vertices[triangle[1]];
    let v2 = its.vertices[triangle[2]];

    // Compute edges
    let e1 = Vec3::new(
        (v1.x - v0.x) as CoordF,
        (v1.y - v0.y) as CoordF,
        (v1.z - v0.z) as CoordF,
    );
    let e2 = Vec3::new(
        (v2.x - v0.x) as CoordF,
        (v2.y - v0.y) as CoordF,
        (v2.z - v0.z) as CoordF,
    );

    // Cross product
    let nx = e1.y * e2.z - e1.z * e2.y;
    let ny = e1.z * e2.x - e1.x * e2.z;
    let nz = e1.x * e2.y - e1.y * e2.x;

    // Normalize
    let len = (nx * nx + ny * ny + nz * nz).sqrt();
    if len > 0.0 {
        Vec3::new(nx / len, ny / len, nz / len)
    } else {
        Vec3::new(0.0, 0.0, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_indexed_triangle_set_creation() {
        let its = IndexedTriangleSet::new();
        assert!(its.vertices.is_empty());
        assert!(its.indices.is_empty());
    }

    #[test]
    fn test_hit_result_creation() {
        let hit = HitResult::new();
        assert_eq!(hit.distance(), HitResult::infty());
        assert!(!hit.is_hit());
        assert!(!hit.is_valid());
    }

    #[test]
    fn test_hit_result_infty() {
        assert!(HitResult::infty().is_infinite());
        assert!(HitResult::infty() > 0.0);
    }

    #[test]
    fn test_vertex_face_index() {
        let vertices = vec![
            Point3F::new(0.0, 0.0, 0.0),
            Point3F::new(1.0, 0.0, 0.0),
            Point3F::new(0.0, 1.0, 0.0),
        ];
        let indices = vec![[0, 1, 2]];
        let its = IndexedTriangleSet::from_parts(vertices, indices);

        let vfidx = VertexFaceIndex::from_its(&its);
        assert_eq!(vfidx.faces_from_vertex(0), &[0]);
        assert_eq!(vfidx.faces_from_vertex(1), &[0]);
        assert_eq!(vfidx.faces_from_vertex(2), &[0]);
    }

    #[test]
    fn test_aabb_mesh_creation() {
        let its = IndexedTriangleSet::new();
        let _mesh = AABBMesh::new(its, false);
    }

    #[test]
    fn test_aabb_mesh_clone() {
        let its = IndexedTriangleSet::new();
        let mesh = AABBMesh::new(its, false);
        let _cloned = mesh.clone();
    }
}
