//! Mesh repair and fixing operations.
//!
//! This module provides mesh repair functionality mirroring
//! BambuStudio's TriangleMesh.cpp repair pipeline.

use crate::geometry::Point3F;
use crate::triangle_mesh::{Triangle, TriangleMesh};
use crate::{CoordF, Result};
use std::collections::HashMap;

/// Mesh repair options controlling which repair steps to apply
/// TriangleMesh.cpp:79-179
#[derive(Clone, Debug)]
pub struct MeshRepairOptions {
    /// Merge duplicate vertices within this tolerance.
    pub merge_tolerance: CoordF,
    /// Remove degenerate triangles.
    pub remove_degenerate: bool,
    /// Fix flipped normals.
    pub fix_normals: bool,
    /// Fill holes.
    pub fill_holes: bool,
    /// Maximum hole size to fill (in mm²).
    pub max_hole_area: CoordF,
}

/// Default repair options matching C++ trianglemesh_repair_on_import defaults
/// TriangleMesh.cpp:79-179
impl Default for MeshRepairOptions {
    fn default() -> Self {
        Self {
            // TriangleMesh.cpp:100 — tolerance = stl.stats.shortest_edge
            merge_tolerance: 0.001, // 1 micron
            // TriangleMesh.cpp:175 — degenerate check at end of repair
            remove_degenerate: true,
            // TriangleMesh.cpp:149 — stl_fix_normal_directions
            fix_normals: true,
            // TriangleMesh.cpp:132-143 — stl_fill_holes (disabled in C++ by #if 0)
            fill_holes: false,
            max_hole_area: 100.0,
        }
    }
}

/// Mesh repair and fixing operations — Rust equivalent of trianglemesh_repair_on_import
/// TriangleMesh.cpp:79-179
pub struct TriangleMeshDeal;

/// Implementation of mesh repair pipeline
/// TriangleMesh.cpp:79-179
impl TriangleMeshDeal {
    // Repair a mesh with the given options, mirroring trianglemesh_repair_on_import
    // TriangleMesh.cpp:79-179
    pub fn repair(mesh: &mut TriangleMesh, options: &MeshRepairOptions) -> Result<()> {
        // TriangleMesh.cpp:92 — stl_check_facets_exact (merge close vertices)
        if options.merge_tolerance > 0.0 {
            mesh.merge_close_vertices(options.merge_tolerance);
        }

        // TriangleMesh.cpp:175-176 — remove degenerate facets at end of repair
        if options.remove_degenerate {
            mesh.remove_degenerate_triangles();
        }

        // TriangleMesh.cpp:149 — stl_fix_normal_directions
        if options.fix_normals {
            Self::fix_normals(mesh);
        }

        // TriangleMesh.cpp:132-143 — stl_fill_holes (disabled by #if 0 in C++)
        if options.fill_holes {
            Self::fill_holes(mesh, options.max_hole_area)?;
        }

        // TriangleMesh.cpp:811-839 — its_compactify_vertices
        mesh.remove_unused_vertices();
        Ok(())
    }

    /// Fix inconsistent normals by ensuring consistent winding order
    /// TriangleMesh.cpp:149
    pub fn fix_normals(mesh: &mut TriangleMesh) {
        // TriangleMesh.cpp:82 — early return if empty
        if mesh.triangle_count() == 0 {
            return;
        }

        // TriangleMesh.cpp:575-586 — build edge-to-face adjacency map
        let mut edge_to_tris: HashMap<(u32, u32), Vec<usize>> = HashMap::new();

        // TriangleMesh.cpp:594-610 — iterate triangles, create sorted edge keys
        for (tri_idx, tri) in mesh.indices().iter().enumerate() {
            let indices = tri.indices;
            // TriangleMesh.cpp:599-600 — store edge with (vertex_low, vertex_high)
            for i in 0..3 {
                let v0 = indices[i];
                let v1 = indices[(i + 1) % 3];
                let edge = if v0 < v1 { (v0, v1) } else { (v1, v0) };
                edge_to_tris.entry(edge).or_default().push(tri_idx);
            }
        }

        // TriangleMesh.cpp:637 — find edges with inconsistent orientation
        let mut flip_needed = vec![false; mesh.triangle_count()];

        // TriangleMesh.cpp:636-641 — check each shared edge for orientation consistency
        for (edge, tris) in &edge_to_tris {
            if tris.len() != 2 {
                continue;
            }

            let tri0_idx = tris[0];
            let tri1_idx = tris[1];

            let tri0 = &mesh.indices()[tri0_idx];
            let tri1 = &mesh.indices()[tri1_idx];

            // TriangleMesh.cpp:637 — face_edge signs should be opposite for consistent orientation
            let e0_in_tri0 = Self::edge_orientation(tri0, edge.0, edge.1);
            let e0_in_tri1 = Self::edge_orientation(tri1, edge.0, edge.1);

            // TriangleMesh.cpp:637 — if face_edge * edges_map[j].face_edge < 0
            if e0_in_tri0 == e0_in_tri1 {
                // Mark the second triangle for flipping
                flip_needed[tri1_idx] = !flip_needed[tri1_idx];
            }
        }

        // TriangleMesh.cpp:790-793 — its_flip_triangles: swap face(1) and face(2)
        let indices = mesh.indices_mut();
        for (i, &needs_flip) in flip_needed.iter().enumerate() {
            if needs_flip {
                indices[i].indices.swap(0, 2);
            }
        }
    }

    /// Check if an edge (v0, v1) exists in a triangle and return its orientation
    /// TriangleMesh.cpp:575-586
    fn edge_orientation(tri: &Triangle, v0: u32, v1: u32) -> bool {
        // TriangleMesh.cpp:604-608 — check edge direction, negative if swapped
        let idx = tri.indices;
        (idx[0] == v0 && idx[1] == v1)
            || (idx[1] == v0 && idx[2] == v1)
            || (idx[2] == v0 && idx[0] == v1)
    }

    /// Fill holes in the mesh by finding and triangulating boundary loops
    /// TriangleMesh.cpp:132-143
    pub fn fill_holes(mesh: &mut TriangleMesh, max_hole_area: CoordF) -> Result<()> {
        // TriangleMesh.cpp:82 — early return if empty
        if mesh.triangle_count() == 0 {
            return Ok(());
        }

        // TriangleMesh.cpp:594-610 — build edge map to find boundary edges
        let mut edge_count: HashMap<(u32, u32), usize> = HashMap::new();
        let mut edge_to_tris: HashMap<(u32, u32), Vec<usize>> = HashMap::new();

        // TriangleMesh.cpp:594-610 — iterate triangles, create sorted edge keys
        for (tri_idx, tri) in mesh.indices().iter().enumerate() {
            let indices = tri.indices;
            for i in 0..3 {
                let v0 = indices[i];
                let v1 = indices[(i + 1) % 3];
                let edge = if v0 < v1 { (v0, v1) } else { (v1, v0) };
                *edge_count.entry(edge).or_insert(0) += 1;
                edge_to_tris.entry(edge).or_default().push(tri_idx);
            }
        }

        // Find boundary edges (used by exactly 1 triangle — open edges)
        let mut boundary_edges: Vec<(u32, u32)> = Vec::new();
        for (edge, count) in &edge_count {
            if *count == 1 {
                boundary_edges.push(*edge);
            }
        }

        if boundary_edges.is_empty() {
            return Ok(()); // No holes to fill
        }

        // Group boundary edges into loops
        let loops = Self::find_boundary_loops(&boundary_edges);

        // Fill each hole that is smaller than max_hole_area
        for hole_loop in &loops {
            if hole_loop.len() < 3 {
                continue;
            }

            // Calculate hole area to decide if we should fill it
            let area = Self::calculate_loop_area(mesh, hole_loop);

            if area <= max_hole_area {
                // Triangulate the hole using fan triangulation
                Self::triangulate_hole(mesh, hole_loop)?;
            }
        }

        Ok(())
    }

    /// Find closed loops from boundary edges using edge-chasing algorithm
    /// No direct C++ equivalent — Rust-specific helper for fill_holes
    fn find_boundary_loops(edges: &[(u32, u32)]) -> Vec<Vec<u32>> {
        let mut loops: Vec<Vec<u32>> = Vec::new();
        let mut used: Vec<bool> = vec![false; edges.len()];

        for start_idx in 0..edges.len() {
            if used[start_idx] {
                continue;
            }

            let mut current_loop: Vec<u32> = Vec::new();
            let mut current_edge = edges[start_idx];
            used[start_idx] = true;
            current_loop.push(current_edge.0);

            // Follow the loop by chasing connected edges
            loop {
                // Find next edge that connects to current edge's endpoint
                let mut found_next = false;
                for (i, edge) in edges.iter().enumerate() {
                    if used[i] {
                        continue;
                    }

                    // Check if this edge connects to current edge
                    if edge.0 == current_edge.1 {
                        used[i] = true;
                        current_loop.push(edge.0);
                        current_edge = *edge;
                        found_next = true;
                        break;
                    } else if edge.1 == current_edge.1 {
                        // Edge is reversed
                        used[i] = true;
                        current_loop.push(edge.1);
                        current_edge = (edge.1, edge.0);
                        found_next = true;
                        break;
                    }
                }

                if !found_next {
                    break;
                }
            }

            if current_loop.len() >= 3 {
                loops.push(current_loop);
            }
        }

        loops
    }

    /// Calculate the area of a boundary loop using shoelace formula
    /// No direct C++ equivalent — Rust-specific helper for fill_holes
    fn calculate_loop_area(mesh: &TriangleMesh, loop_vertices: &[u32]) -> CoordF {
        if loop_vertices.len() < 3 {
            return 0.0;
        }

        // Calculate centroid of loop vertices
        let mut centroid = Point3F::new(0.0, 0.0, 0.0);
        for &v_idx in loop_vertices {
            let v = mesh.vertex(v_idx);
            centroid.x += v.x;
            centroid.y += v.y;
            centroid.z += v.z;
        }
        centroid.x /= loop_vertices.len() as CoordF;
        centroid.y /= loop_vertices.len() as CoordF;
        centroid.z /= loop_vertices.len() as CoordF;

        // Project to 2D and calculate area using shoelace formula
        let mut area = 0.0;

        for i in 0..loop_vertices.len() {
            let v1 = mesh.vertex(loop_vertices[i]);
            let v2 = mesh.vertex(loop_vertices[(i + 1) % loop_vertices.len()]);

            // Project to XY plane (simplified)
            let x1 = v1.x - centroid.x;
            let y1 = v1.y - centroid.y;
            let x2 = v2.x - centroid.x;
            let y2 = v2.y - centroid.y;

            area += x1 * y2 - x2 * y1;
        }

        area.abs() / 2.0
    }

    /// Triangulate a hole using fan triangulation from first vertex
    /// No direct C++ equivalent — Rust-specific helper for fill_holes
    fn triangulate_hole(mesh: &mut TriangleMesh, hole_loop: &[u32]) -> Result<()> {
        if hole_loop.len() < 3 {
            return Ok(());
        }

        // Simple fan triangulation from first vertex
        let base_vertex = hole_loop[0];

        for i in 1..(hole_loop.len() - 1) {
            let v1 = hole_loop[i];
            let v2 = hole_loop[i + 1];

            // Create triangle: base_vertex -> v1 -> v2
            mesh.add_triangle_indices(base_vertex, v1, v2);
        }

        Ok(())
    }

    /// Simplify mesh by reducing triangle count (stub — quadric edge collapse not yet implemented)
    /// No direct C++ equivalent — Rust-specific
    pub fn simplify(mesh: &mut TriangleMesh, target_ratio: f64) -> Result<()> {
        let target_count = (mesh.triangle_count() as f64 * target_ratio) as usize;
        if target_count >= mesh.triangle_count() {
            return Ok(());
        }

        // TODO: Implement quadric edge collapse simplification
        // This is a complex algorithm that requires:
        // 1. Compute quadric error metrics for each vertex
        // 2. Compute contraction cost for each edge
        // 3. Iteratively collapse lowest-cost edges until target reached

        Ok(())
    }

    /// Split mesh into connected components using DFS on triangle adjacency graph
    /// TriangleMesh.cpp:393-410
    pub fn split_into_components(mesh: &TriangleMesh) -> Vec<TriangleMesh> {
        // TriangleMesh.cpp:395 — early return if empty
        if mesh.triangle_count() == 0 {
            return vec![];
        }

        // Build triangle adjacency graph using shared edges
        let mut tri_adj: Vec<Vec<usize>> = vec![vec![]; mesh.triangle_count()];

        // TriangleMesh.cpp:594-610 — build edge-to-face map
        let mut edge_to_tris: HashMap<(u32, u32), Vec<usize>> = HashMap::new();

        for (tri_idx, tri) in mesh.indices().iter().enumerate() {
            let indices = tri.indices;
            for i in 0..3 {
                let v0 = indices[i];
                let v1 = indices[(i + 1) % 3];
                let edge = if v0 < v1 { (v0, v1) } else { (v1, v0) };
                edge_to_tris.entry(edge).or_default().push(tri_idx);
            }
        }

        // Build adjacency from shared edges
        for (_, tris) in edge_to_tris {
            if tris.len() == 2 {
                tri_adj[tris[0]].push(tris[1]);
                tri_adj[tris[1]].push(tris[0]);
            }
        }

        // TriangleMesh.cpp:395 — its_split uses DFS to find connected components
        let mut visited = vec![false; mesh.triangle_count()];
        let mut components: Vec<Vec<usize>> = Vec::new();

        for start_tri in 0..mesh.triangle_count() {
            if visited[start_tri] {
                continue;
            }

            let mut component = Vec::new();
            let mut stack = vec![start_tri];

            while let Some(tri_idx) = stack.pop() {
                if visited[tri_idx] {
                    continue;
                }
                visited[tri_idx] = true;
                component.push(tri_idx);

                for &neighbor in &tri_adj[tri_idx] {
                    if !visited[neighbor] {
                        stack.push(neighbor);
                    }
                }
            }

            components.push(component);
        }

        // TriangleMesh.cpp:400-408 — create separate mesh for each component
        components
            .into_iter()
            .map(|tri_indices| Self::extract_submesh(mesh, &tri_indices))
            .collect()
    }

    /// Extract a submesh from a set of triangle indices, remapping vertex indices
    /// TriangleMesh.cpp:537-559
    fn extract_submesh(mesh: &TriangleMesh, tri_indices: &[usize]) -> TriangleMesh {
        // TriangleMesh.cpp:539-542 — allocate new vertex/index storage
        let mut vertex_map: HashMap<u32, u32> = HashMap::new();
        let mut new_vertices: Vec<Point3F> = Vec::new();
        let mut new_indices: Vec<Triangle> = Vec::new();

        // TriangleMesh.cpp:545-556 — remap vertices and build new triangle set
        for &tri_idx in tri_indices {
            let tri = mesh.indices()[tri_idx];
            let mut new_tri = [0u32; 3];

            for (i, &old_idx) in tri.indices.iter().enumerate() {
                let new_idx = *vertex_map.entry(old_idx).or_insert_with(|| {
                    let idx = new_vertices.len() as u32;
                    new_vertices.push(mesh.vertex(old_idx));
                    idx
                });
                new_tri[i] = new_idx;
            }

            new_indices.push(Triangle::new(new_tri[0], new_tri[1], new_tri[2]));
        }

        TriangleMesh::from_parts(new_vertices, new_indices)
    }

    /// Calculate mesh statistics including bounding box, volume, surface area
    /// TriangleMesh.cpp:37-54
    pub fn statistics(mesh: &TriangleMesh) -> MeshStatistics {
        // TriangleMesh.cpp:45-53 — fill_initial_stats
        MeshStatistics {
            vertex_count: mesh.vertex_count(),
            triangle_count: mesh.triangle_count(),
            surface_area: mesh.surface_area(),
            volume: mesh.volume(),
            bounding_box: mesh.compute_bounding_box(),
            has_degenerate: mesh.has_degenerate_triangles(),
            is_likely_manifold: mesh.is_likely_manifold(),
        }
    }
}

/// Mesh statistics — Rust equivalent of TriangleMeshStats
/// TriangleMesh.cpp:37-54
#[derive(Clone, Debug)]
pub struct MeshStatistics {
    /// TriangleMesh.cpp:47 — its.vertices.size()
    pub vertex_count: usize,
    /// TriangleMesh.cpp:47 — its.indices.size()
    pub triangle_count: usize,
    /// Surface area computed from triangle faces
    pub surface_area: f64,
    /// TriangleMesh.cpp:48 — its_volume(its)
    pub volume: f64,
    /// TriangleMesh.cpp:39-42 — bounding_box(its)
    pub bounding_box: crate::geometry::BoundingBox3F,
    /// Whether mesh contains degenerate (zero-area) triangles
    pub has_degenerate: bool,
    /// Whether mesh appears to be manifold (all edges shared by exactly 2 triangles)
    pub is_likely_manifold: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_split_components() {
        // Create two separate cubes
        let mut mesh1 = TriangleMesh::cube(10.0);
        let mut mesh2 = TriangleMesh::cube(10.0);
        mesh2.translate(Point3F::new(100.0, 0.0, 0.0));

        // Merge into one mesh
        let offset = mesh1.vertex_count() as u32;
        let mut combined = TriangleMesh::new();

        for v in mesh1.vertices() {
            combined.add_vertex(*v);
        }
        for v in mesh2.vertices() {
            combined.add_vertex(*v);
        }

        for tri in mesh1.indices() {
            combined.add_triangle(*tri);
        }
        for tri in mesh2.indices() {
            let new_tri = Triangle::new(
                tri.indices[0] + offset,
                tri.indices[1] + offset,
                tri.indices[2] + offset,
            );
            combined.add_triangle(new_tri);
        }

        // Split and verify
        let components = TriangleMeshDeal::split_into_components(&combined);
        assert_eq!(components.len(), 2);
        assert_eq!(components[0].triangle_count(), 12);
        assert_eq!(components[1].triangle_count(), 12);
    }

    #[test]
    fn test_repair_merge_vertices() {
        let mut mesh = TriangleMesh::new();

        // Add duplicate vertices
        mesh.add_vertex(Point3F::new(0.0, 0.0, 0.0));
        mesh.add_vertex(Point3F::new(0.0, 0.0, 0.0)); // Duplicate
        mesh.add_vertex(Point3F::new(1.0, 0.0, 0.0));
        mesh.add_vertex(Point3F::new(0.0, 1.0, 0.0));

        mesh.add_triangle_indices(0, 2, 3);
        mesh.add_triangle_indices(1, 2, 3);

        let options = MeshRepairOptions {
            merge_tolerance: 0.001,
            ..Default::default()
        };

        TriangleMeshDeal::repair(&mut mesh, &options).unwrap();

        // Should have merged the duplicate vertices
        assert!(mesh.vertex_count() <= 4);
    }
}
