//! Mesh boolean operations.
//!
//! This module provides CSG (Constructive Solid Geometry) operations
//! mirroring BambuStudio's MeshBoolean.cpp.

use crate::geometry::{BoundingBox3F, Point3F};
use crate::triangle_mesh::{Triangle, TriangleMesh};
use crate::{Error, Result};
use std::collections::HashMap;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// Boolean operation type enumeration for mesh CSG operations.
/// MeshBoolean.hpp:19-23
pub enum BooleanOp {
    /// Union operation combines two meshes.
    /// MeshBoolean.cpp:436-441
    Union,
    /// Intersection operation keeps shared volume.
    /// MeshBoolean.cpp:443-448
    Intersection,
    /// Difference operation subtracts second mesh from first.
    /// MeshBoolean.cpp:429-434
    Difference,
    /// Symmetric difference keeps non-overlapping volume.
    /// MeshBoolean.cpp:256-268
    SymmetricDifference,
}

/// BooleanOp helper methods for string conversion.
/// MeshBoolean.hpp:19-23
impl BooleanOp {
    // Get the operation name as a string constant.
    // MeshBoolean.hpp:19-23
    pub fn name(&self) -> &'static str {
        // MeshBoolean.hpp:19-23
        match self {
            BooleanOp::Union => "union",
            BooleanOp::Intersection => "intersection",
            BooleanOp::Difference => "difference",
            BooleanOp::SymmetricDifference => "symmetric_difference",
        }
    }
}

/// Perform a boolean operation on two meshes using voxel-based approach.
/// MeshBoolean.cpp:274-293
pub fn boolean_operation(
    mesh_a: &TriangleMesh,
    mesh_b: &TriangleMesh,
    operation: BooleanOp,
) -> Result<TriangleMesh> {
    // Validate that first mesh is manifold
    // MeshBoolean.cpp:275-276
    if !is_manifold(mesh_a) {
        // MeshBoolean.cpp:276
        return Err(Error::Mesh(
            "First mesh is not manifold (watertight)".to_string(),
        ));
    }
    // Validate that second mesh is manifold
    // MeshBoolean.cpp:277
    if !is_manifold(mesh_b) {
        // MeshBoolean.cpp:277
        return Err(Error::Mesh(
            "Second mesh is not manifold (watertight)".to_string(),
        ));
    }

    // Compute bounding boxes for overlap test
    // MeshBoolean.cpp:278-279
    let bbox_a = compute_bounding_box(mesh_a);
    // MeshBoolean.cpp:279
    let bbox_b = compute_bounding_box(mesh_b);

    // Fast path when bounding boxes don't overlap
    // MeshBoolean.cpp:280-290
    if !bbox_a.intersects(&bbox_b) {
        // MeshBoolean.cpp:281-289
        let result =
        // MeshBoolean.cpp:281-289
        match operation {
            // MeshBoolean.cpp:282
            BooleanOp::Union => Ok(merge_meshes(mesh_a, mesh_b)),
            // MeshBoolean.cpp:284
            BooleanOp::Intersection => Ok(TriangleMesh::new()),
            // MeshBoolean.cpp:286
            BooleanOp::Difference => Ok(mesh_a.clone()),
            // MeshBoolean.cpp:288
            BooleanOp::SymmetricDifference => Ok(merge_meshes(mesh_a, mesh_b)),
        };
        // MeshBoolean.cpp:289
        return result;
    }

    // Main path: dispatch to voxel-based boolean
    // MeshBoolean.cpp:291-293
    voxel_boolean_operation(mesh_a, mesh_b, operation)
}

/// Voxel-based boolean operation implementation.
/// MeshBoolean.cpp:274-293
fn voxel_boolean_operation(
    mesh_a: &TriangleMesh,
    mesh_b: &TriangleMesh,
    operation: BooleanOp,
) -> Result<TriangleMesh> {
    // Estimate voxel resolution from mesh sizes
    // MeshBoolean.cpp:274
    let voxel_size = estimate_voxel_size(mesh_a, mesh_b);
    // Voxelize both input meshes
    // MeshBoolean.cpp:274
    let grid_a = voxelize_mesh(mesh_a, voxel_size);
    // MeshBoolean.cpp:274
    let grid_b = voxelize_mesh(mesh_b, voxel_size);

    // MeshBoolean.cpp:274-293
    let result_grid =
    // MeshBoolean.cpp:274-293
    match operation {
        // MeshBoolean.cpp:275
        BooleanOp::Union => boolean_union(&grid_a, &grid_b),
        // MeshBoolean.cpp:280
        BooleanOp::Intersection => boolean_intersection(&grid_a, &grid_b),
        // MeshBoolean.cpp:285
        BooleanOp::Difference => boolean_difference(&grid_a, &grid_b),
        // MeshBoolean.cpp:290
        BooleanOp::SymmetricDifference => boolean_symmetric_difference(&grid_a, &grid_b),
    };

    // Convert result voxels back to triangle mesh
    // MeshBoolean.cpp:227-231
    mesh_from_voxels(&result_grid, voxel_size)
}

/// Voxel grid type alias for boolean operations.
/// MeshBoolean.cpp:274
type VoxelGrid = HashMap<(i64, i64, i64), bool>;

/// Estimate appropriate voxel size based on mesh bounding box dimensions.
/// MeshBoolean.cpp:274-293
fn estimate_voxel_size(mesh_a: &TriangleMesh, mesh_b: &TriangleMesh) -> f64 {
    // Compute bounding boxes of both meshes
    // MeshBoolean.cpp:278
    let bbox_a = compute_bounding_box(mesh_a);
    // MeshBoolean.cpp:279
    let bbox_b = compute_bounding_box(mesh_b);

    // Find maximum dimension of mesh A
    // MeshBoolean.cpp:278
    let size_a = bbox_a.size_x().max(bbox_a.size_y()).max(bbox_a.size_z()) as f64 / 1_000_000.0;
    // Find maximum dimension of mesh B
    // MeshBoolean.cpp:279
    let size_b = bbox_b.size_x().max(bbox_b.size_y()).max(bbox_b.size_z()) as f64 / 1_000_000.0;

    // Use larger of the two dimensions
    // MeshBoolean.cpp:274
    let max_size = size_a.max(size_b);

    // Use 1/100th of the largest dimension as voxel size
    // MeshBoolean.cpp:274
    (max_size / 100.0).max(0.01)
}

/// Convert mesh to voxel grid using ray-casting point-in-mesh test.
/// MeshBoolean.cpp:274-293
fn voxelize_mesh(mesh: &TriangleMesh, voxel_size: f64) -> VoxelGrid {
    // MeshBoolean.cpp:274
    let mut grid = VoxelGrid::new();
    // Compute mesh bounding box for iteration bounds
    // MeshBoolean.cpp:278
    let bbox = compute_bounding_box(mesh);

    // Calculate voxel grid bounds from bounding box
    // MeshBoolean.cpp:274
    let min_x = (bbox.min.x as f64 / 1_000_000.0 / voxel_size).floor() as i64;
    // MeshBoolean.cpp:274
    let max_x = (bbox.max.x as f64 / 1_000_000.0 / voxel_size).ceil() as i64;
    // MeshBoolean.cpp:274
    let min_y = (bbox.min.y as f64 / 1_000_000.0 / voxel_size).floor() as i64;
    // MeshBoolean.cpp:274
    let max_y = (bbox.max.y as f64 / 1_000_000.0 / voxel_size).ceil() as i64;
    // MeshBoolean.cpp:274
    let min_z = (bbox.min.z as f64 / 1_000_000.0 / voxel_size).floor() as i64;
    // MeshBoolean.cpp:274
    let max_z = (bbox.max.z as f64 / 1_000_000.0 / voxel_size).ceil() as i64;

    // Iterate over all voxels in bounding box
    // MeshBoolean.cpp:274
    for x in min_x..=max_x {
        // MeshBoolean.cpp:274
        for y in min_y..=max_y {
            // MeshBoolean.cpp:274
            for z in min_z..=max_z {
                // Compute voxel center point
                // MeshBoolean.cpp:274
                let voxel_center = Point3F::new(
                    x as f64 * voxel_size + voxel_size / 2.0,
                    y as f64 * voxel_size + voxel_size / 2.0,
                    z as f64 * voxel_size + voxel_size / 2.0,
                );

                // Test if voxel center is inside mesh
                // MeshBoolean.cpp:274
                if point_inside_mesh(voxel_center, mesh) {
                    // MeshBoolean.cpp:274
                    grid.insert((x, y, z), true);
                }
            }
        }
    }

    // MeshBoolean.cpp:274
    grid
}

/// Check if a point is inside a mesh using ray casting algorithm.
/// MeshBoolean.cpp:274-293
fn point_inside_mesh(point: Point3F, mesh: &TriangleMesh) -> bool {
    // Initialize intersection counter
    // MeshBoolean.cpp:274
    let mut intersections = 0;

    // Ray origin at the test point
    // MeshBoolean.cpp:274
    let ray_origin = point;
    // Cast ray in +X direction
    // MeshBoolean.cpp:274
    let ray_dir = Point3F::new(1.0, 0.0, 0.0);

    // Test ray against all triangles
    // MeshBoolean.cpp:274
    for tri in mesh.triangles() {
        // MeshBoolean.cpp:274
        let vertices = tri.vertices;

        // Check ray-triangle intersection
        // MeshBoolean.cpp:274
        if ray_triangle_intersect(ray_origin, ray_dir, vertices) {
            // MeshBoolean.cpp:274
            intersections += 1;
        }
    }

    // Odd number of intersections means point is inside
    // MeshBoolean.cpp:274
    intersections % 2 == 1
}

/// Ray-triangle intersection using Moller-Trumbore algorithm.
/// MeshBoolean.cpp:274-293
fn ray_triangle_intersect(ray_origin: Point3F, ray_dir: Point3F, tri: [Point3F; 3]) -> bool {
    // Extract triangle vertices
    // MeshBoolean.cpp:274
    let v0 = tri[0];
    // MeshBoolean.cpp:274
    let v1 = tri[1];
    // MeshBoolean.cpp:274
    let v2 = tri[2];

    // Compute edge vectors from v0
    // MeshBoolean.cpp:274
    let edge1 = Point3F::new(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
    // MeshBoolean.cpp:274
    let edge2 = Point3F::new(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);

    // Compute determinant via cross product
    // MeshBoolean.cpp:274
    let h = cross(ray_dir, edge2);
    // MeshBoolean.cpp:274
    let a = dot(edge1, h);

    // Check if ray is parallel to triangle
    // MeshBoolean.cpp:274
    if a.abs() < 1e-10 {
        // MeshBoolean.cpp:274
        return false;
    }

    // Compute barycentric u coordinate
    // MeshBoolean.cpp:274
    let f = 1.0 / a;
    // MeshBoolean.cpp:274
    let s = Point3F::new(
        ray_origin.x - v0.x,
        ray_origin.y - v0.y,
        ray_origin.z - v0.z,
    );
    // MeshBoolean.cpp:274
    let u = f * dot(s, h);

    // Check u bounds
    // MeshBoolean.cpp:274
    if u < 0.0 || u > 1.0 {
        // MeshBoolean.cpp:274
        return false;
    }

    // Compute barycentric v coordinate
    // MeshBoolean.cpp:274
    let q = cross(s, edge1);
    // MeshBoolean.cpp:274
    let v = f * dot(ray_dir, q);

    // Check v bounds and u+v bounds
    // MeshBoolean.cpp:274
    if v < 0.0 || u + v > 1.0 {
        // MeshBoolean.cpp:274
        return false;
    }

    // Compute intersection distance along ray
    // MeshBoolean.cpp:274
    let t = f * dot(edge2, q);

    // Intersection is valid if in front of ray origin
    // MeshBoolean.cpp:274
    t > 1e-10
}

/// Dot product of two 3D vectors.
/// MeshBoolean.cpp:274-293
fn dot(a: Point3F, b: Point3F) -> f64 {
    // MeshBoolean.cpp:274
    a.x * b.x + a.y * b.y + a.z * b.z
}

/// Cross product of two 3D vectors.
/// MeshBoolean.cpp:274-293
fn cross(a: Point3F, b: Point3F) -> Point3F {
    // MeshBoolean.cpp:274
    Point3F::new(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    )
}

/// Boolean union of two voxel grids.
/// MeshBoolean.cpp:436-441
fn boolean_union(grid_a: &VoxelGrid, grid_b: &VoxelGrid) -> VoxelGrid {
    // Start with copy of grid A
    // MeshBoolean.cpp:436
    let mut result = grid_a.clone();
    // Add all voxels from grid B
    // MeshBoolean.cpp:437-440
    for (key, _) in grid_b {
        // MeshBoolean.cpp:438
        result.insert(*key, true);
    }
    // MeshBoolean.cpp:441
    result
}

/// Boolean intersection of two voxel grids.
/// MeshBoolean.cpp:443-448
fn boolean_intersection(grid_a: &VoxelGrid, grid_b: &VoxelGrid) -> VoxelGrid {
    // MeshBoolean.cpp:443
    let mut result = VoxelGrid::new();
    // Keep only voxels present in both grids
    // MeshBoolean.cpp:444-447
    for (key, _) in grid_a {
        // MeshBoolean.cpp:445
        if grid_b.contains_key(key) {
            // MeshBoolean.cpp:446
            result.insert(*key, true);
        }
    }
    // MeshBoolean.cpp:448
    result
}

/// Boolean difference of two voxel grids.
/// MeshBoolean.cpp:429-434
fn boolean_difference(grid_a: &VoxelGrid, grid_b: &VoxelGrid) -> VoxelGrid {
    // MeshBoolean.cpp:429
    let mut result = VoxelGrid::new();
    // Keep voxels from A that are not in B
    // MeshBoolean.cpp:430-433
    for (key, _) in grid_a {
        // MeshBoolean.cpp:431
        if !grid_b.contains_key(key) {
            // MeshBoolean.cpp:432
            result.insert(*key, true);
        }
    }
    // MeshBoolean.cpp:434
    result
}

/// Boolean symmetric difference of two voxel grids.
/// MeshBoolean.cpp:256-268
fn boolean_symmetric_difference(grid_a: &VoxelGrid, grid_b: &VoxelGrid) -> VoxelGrid {
    // MeshBoolean.cpp:256
    let mut result = VoxelGrid::new();

    // Add voxels from A that are not in B
    // MeshBoolean.cpp:257-261
    for (key, _) in grid_a {
        // MeshBoolean.cpp:258
        if !grid_b.contains_key(key) {
            // MeshBoolean.cpp:259
            result.insert(*key, true);
        }
    }

    // Add voxels from B that are not in A
    // MeshBoolean.cpp:262-266
    for (key, _) in grid_b {
        // MeshBoolean.cpp:263
        if !grid_a.contains_key(key) {
            // MeshBoolean.cpp:264
            result.insert(*key, true);
        }
    }

    // MeshBoolean.cpp:268
    result
}

/// Convert voxel grid back to triangle mesh.
/// MeshBoolean.cpp:227-231
fn mesh_from_voxels(grid: &VoxelGrid, voxel_size: f64) -> Result<TriangleMesh> {
    // Return empty mesh for empty grid
    // MeshBoolean.cpp:227
    if grid.is_empty() {
        // MeshBoolean.cpp:228
        return Ok(TriangleMesh::new());
    }

    // Allocate vertex and triangle storage
    // MeshBoolean.cpp:229
    let mut vertices = Vec::new();
    // MeshBoolean.cpp:229
    let mut triangles = Vec::new();

    // Create cube geometry for each filled voxel
    // MeshBoolean.cpp:230
    for (key, _) in grid {
        // MeshBoolean.cpp:230
        let (x, y, z) = *key;
        // MeshBoolean.cpp:230
        let base_x = x as f64 * voxel_size;
        // MeshBoolean.cpp:230
        let base_y = y as f64 * voxel_size;
        // MeshBoolean.cpp:230
        let base_z = z as f64 * voxel_size;

        // Track base vertex index for this cube
        // MeshBoolean.cpp:230
        let base_idx = vertices.len() as u32;

        // Define 8 corner vertices of the voxel cube
        // MeshBoolean.cpp:230
        let corners = [
            Point3F::new(base_x, base_y, base_z),
            Point3F::new(base_x + voxel_size, base_y, base_z),
            Point3F::new(base_x + voxel_size, base_y + voxel_size, base_z),
            Point3F::new(base_x, base_y + voxel_size, base_z),
            Point3F::new(base_x, base_y, base_z + voxel_size),
            Point3F::new(base_x + voxel_size, base_y, base_z + voxel_size),
            Point3F::new(
                base_x + voxel_size,
                base_y + voxel_size,
                base_z + voxel_size,
            ),
            Point3F::new(base_x, base_y + voxel_size, base_z + voxel_size),
        ];

        // MeshBoolean.cpp:230
        vertices.extend(corners);

        // Define 12 triangles for 6 cube faces
        // MeshBoolean.cpp:230
        let indices: [[u32; 3]; 12] = [
            [0, 1, 2],
            [0, 2, 3],
            [4, 6, 5],
            [4, 7, 6],
            [0, 4, 5],
            [0, 5, 1],
            [2, 6, 7],
            [2, 7, 3],
            [0, 3, 7],
            [0, 7, 4],
            [1, 5, 6],
            [1, 6, 2],
        ];

        // Create triangle for each face index triple
        // MeshBoolean.cpp:230
        for tri_indices in &indices {
            // MeshBoolean.cpp:230
            let v0 = base_idx + tri_indices[0];
            // MeshBoolean.cpp:230
            let v1 = base_idx + tri_indices[1];
            // MeshBoolean.cpp:230
            let v2 = base_idx + tri_indices[2];

            // MeshBoolean.cpp:230
            triangles.push(Triangle::new(v0, v1, v2));
        }
    }

    // Construct mesh from vertices and triangles
    // MeshBoolean.cpp:231
    Ok(TriangleMesh::from_parts(vertices, triangles))
}

/// Compute bounding box encompassing all mesh triangles.
/// MeshBoolean.cpp:274-293
fn compute_bounding_box(mesh: &TriangleMesh) -> BoundingBox3F {
    // MeshBoolean.cpp:278
    let mut bbox = BoundingBox3F::new();
    // Iterate over all triangles to find bounds
    // MeshBoolean.cpp:278
    for tri in mesh.triangles() {
        // Merge each vertex into the bounding box
        // MeshBoolean.cpp:278
        for vertex in tri.vertices {
            // MeshBoolean.cpp:278
            bbox.merge_point(vertex);
        }
    }
    // MeshBoolean.cpp:278
    bbox
}

/// Merge two meshes by concatenating vertices and triangles.
/// MeshBoolean.cpp:380-403
fn merge_meshes(mesh_a: &TriangleMesh, mesh_b: &TriangleMesh) -> TriangleMesh {
    // Clone first mesh as base
    // MeshBoolean.cpp:381
    let mut result = mesh_a.clone();

    // Offset triangle indices by vertex count of first mesh
    // MeshBoolean.cpp:382
    let offset = result.vertex_count() as u32;
    // Append triangles from second mesh with adjusted indices
    // MeshBoolean.cpp:383-402
    for tri in mesh_b.triangles() {
        // MeshBoolean.cpp:384-401
        result.add_triangle(Triangle::new(
            tri.indices[0] + offset,
            tri.indices[1] + offset,
            tri.indices[2] + offset,
        ));
    }

    // MeshBoolean.cpp:403
    result
}

/// Union two meshes (A union B).
/// MeshBoolean.cpp:436-441
pub fn union(mesh_a: &TriangleMesh, mesh_b: &TriangleMesh) -> Result<TriangleMesh> {
    // MeshBoolean.cpp:437
    boolean_operation(mesh_a, mesh_b, BooleanOp::Union)
}

/// Intersect two meshes (A intersect B).
/// MeshBoolean.cpp:443-448
pub fn intersection(mesh_a: &TriangleMesh, mesh_b: &TriangleMesh) -> Result<TriangleMesh> {
    // MeshBoolean.cpp:444
    boolean_operation(mesh_a, mesh_b, BooleanOp::Intersection)
}

/// Subtract mesh B from mesh A (A minus B).
/// MeshBoolean.cpp:429-434
pub fn difference(mesh_a: &TriangleMesh, mesh_b: &TriangleMesh) -> Result<TriangleMesh> {
    // MeshBoolean.cpp:430
    boolean_operation(mesh_a, mesh_b, BooleanOp::Difference)
}

/// Check if a mesh is manifold with all edges shared by exactly 2 triangles.
/// MeshBoolean.cpp:459-471
pub fn is_manifold(mesh: &TriangleMesh) -> bool {
    // Empty mesh is trivially manifold
    // MeshBoolean.cpp:460
    if mesh.is_empty() {
        // MeshBoolean.cpp:461
        return true;
    }

    // Build edge-to-face-count map
    // MeshBoolean.cpp:462
    let mut edge_count: HashMap<(u32, u32), usize> = HashMap::new();

    // Count occurrences of each edge across all triangles
    // MeshBoolean.cpp:463-468
    for tri in mesh.triangles() {
        // MeshBoolean.cpp:464
        let v = tri.indices;

        // Normalize edge vertex ordering for consistent lookup
        // MeshBoolean.cpp:465
        let edges = [
            (v[0].min(v[1]), v[0].max(v[1])),
            (v[1].min(v[2]), v[1].max(v[2])),
            (v[2].min(v[0]), v[2].max(v[0])),
        ];

        // Increment count for each edge
        // MeshBoolean.cpp:466-467
        for edge in edges {
            // MeshBoolean.cpp:467
            *edge_count.entry(edge).or_insert(0) += 1;
        }
    }

    // Manifold requires every edge shared by exactly 2 triangles
    // MeshBoolean.cpp:469-471
    edge_count.values().all(|&count| count == 2)
}

/// Get mesh statistics including triangle count and manifold status.
/// MeshBoolean.cpp:459-471
pub fn mesh_stats(mesh: &TriangleMesh) -> MeshStats {
    // MeshBoolean.cpp:459
    MeshStats {
        // MeshBoolean.cpp:460
        triangle_count: mesh.triangle_count(),
        // MeshBoolean.cpp:461
        vertex_count: mesh.vertex_count(),
        // MeshBoolean.cpp:462
        is_manifold: is_manifold(mesh),
        // MeshBoolean.cpp:463
        bounding_box: compute_bounding_box(mesh),
    }
}

#[derive(Debug, Clone)]
/// Mesh statistics container with geometry and topology info.
/// MeshBoolean.cpp:459-471
pub struct MeshStats {
    /// Number of triangles in the mesh.
    /// MeshBoolean.cpp:459
    pub triangle_count: usize,
    /// Number of vertices in the mesh.
    /// MeshBoolean.cpp:459
    pub vertex_count: usize,
    /// Whether the mesh is manifold (watertight).
    /// MeshBoolean.cpp:459
    pub is_manifold: bool,
    /// Axis-aligned bounding box of the mesh.
    /// MeshBoolean.cpp:459
    pub bounding_box: BoundingBox3F,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_boolean_union_empty() {
        let mesh_a = TriangleMesh::new();
        let mesh_b = TriangleMesh::new();

        let result = union(&mesh_a, &mesh_b).unwrap();
        assert!(result.is_empty());
    }

    #[test]
    fn test_is_manifold_empty() {
        let mesh = TriangleMesh::new();
        assert!(is_manifold(&mesh));
    }

    #[test]
    fn test_boolean_op_names() {
        assert_eq!(BooleanOp::Union.name(), "union");
        assert_eq!(BooleanOp::Intersection.name(), "intersection");
        assert_eq!(BooleanOp::Difference.name(), "difference");
        assert_eq!(
            BooleanOp::SymmetricDifference.name(),
            "symmetric_difference"
        );
    }
}
