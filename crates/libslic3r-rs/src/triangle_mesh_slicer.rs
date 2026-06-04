//! Mesh Slicer - Triangle-plane intersection algorithm.
//!
//! This module implements the core slicing algorithm that computes the intersection
//! of a triangle mesh with horizontal planes at specified Z heights.
//!
//! The algorithm is based on BambuStudio/PrusaSlicer's TriangleMeshSlicer:
//! 1. For each triangle, determine which slicing planes intersect it
//! 2. Compute intersection line segments for each plane-triangle intersection
//! 3. Stitch line segments into closed polygons
//! 4. Classify polygons as contours (outer) or holes (inner)
//! 5. Return ExPolygons for each layer

use crate::geometry::{ExPolygon, ExPolygons, Point, Polygon};
use crate::triangle_mesh::TriangleMesh;
use crate::{scale, Coord, CoordF};
use std::collections::HashMap;

/// Represents an intersection point on a slicing plane.
#[derive(Clone, Debug)]
struct IntersectionPoint {
    /// X coordinate (scaled)
    x: Coord,
    /// Y coordinate (scaled)
    y: Coord,
    /// Vertex ID if this point lies on a mesh vertex, -1 otherwise
    point_id: i32,
    /// Edge ID if this point lies on a mesh edge, -1 otherwise
    edge_id: i32,
}

impl IntersectionPoint {
    fn new(x: Coord, y: Coord) -> Self {
        Self {
            x,
            y,
            point_id: -1,
            edge_id: -1,
        }
    }

    fn with_point_id(mut self, id: i32) -> Self {
        self.point_id = id;
        self
    }

    fn with_edge_id(mut self, id: i32) -> Self {
        self.edge_id = id;
        self
    }

    fn to_point(&self) -> Point {
        Point::new(self.x, self.y)
    }
}

/// Type of facet edge intersection with the slicing plane.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum FacetEdgeType {
    /// General case - plane intersects two edges
    General,
    /// Two vertices on plane, third below (upper edge of triangle)
    Top,
    /// Two vertices on plane, third above (lower edge of triangle)
    Bottom,
    /// All three vertices on the plane
    Horizontal,
}

/// Represents an intersection line segment on a slicing plane.
#[derive(Clone, Debug)]
struct IntersectionLine {
    /// Start point
    a: Point,
    /// End point
    b: Point,
    /// Vertex ID for point a (-1 if on edge)
    a_id: i32,
    /// Vertex ID for point b (-1 if on edge)
    b_id: i32,
    /// Edge ID for point a (-1 if on vertex)
    edge_a_id: i32,
    /// Edge ID for point b (-1 if on vertex)
    edge_b_id: i32,
    /// Type of intersection
    edge_type: FacetEdgeType,
    /// Flags for processing
    flags: u32,
}

impl IntersectionLine {
    fn new() -> Self {
        Self {
            a: Point::new(0, 0),
            b: Point::new(0, 0),
            a_id: -1,
            b_id: -1,
            edge_a_id: -1,
            edge_b_id: -1,
            edge_type: FacetEdgeType::General,
            flags: 0,
        }
    }

    const SKIP: u32 = 0x200;

    fn skip(&self) -> bool {
        (self.flags & Self::SKIP) != 0
    }

    fn set_skip(&mut self) {
        self.flags |= Self::SKIP;
    }

    fn reverse(&mut self) {
        std::mem::swap(&mut self.a, &mut self.b);
        std::mem::swap(&mut self.a_id, &mut self.b_id);
        std::mem::swap(&mut self.edge_a_id, &mut self.edge_b_id);
    }
}

/// Result of slicing a single facet.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum FacetSliceType {
    /// No intersection
    NoSlice,
    /// Valid slicing intersection
    Slicing,
    /// Cutting only (for horizontal faces)
    Cutting,
}

/// Slice a single triangle facet against a horizontal plane.
///
/// Returns the intersection line if the plane intersects the triangle.
fn slice_facet(
    slice_z: CoordF,
    vertices: &[[CoordF; 3]; 3],
    indices: &[u32; 3],
    edge_ids: &[i32; 3],
    idx_vertex_lowest: usize,
) -> (FacetSliceType, Option<IntersectionLine>) {
    let mut points: Vec<IntersectionPoint> = Vec::with_capacity(3);
    let mut point_on_layer: Option<usize> = None;

    // Check if all vertices are on the plane (horizontal triangle)
    let horizontal = (vertices[0][2] - slice_z).abs() < 1e-10
        && (vertices[1][2] - slice_z).abs() < 1e-10
        && (vertices[2][2] - slice_z).abs() < 1e-10;

    // Loop through the three edges of the triangle
    for j in 0..3 {
        let k = (idx_vertex_lowest + j) % 3;
        let l = (k + 1) % 3;
        let edge_id = edge_ids[k];
        let a_id = indices[k] as i32;
        let b_id = indices[l] as i32;
        let a = &vertices[k];
        let b = &vertices[l];
        let c = &vertices[(k + 2) % 3];

        let a_z = a[2];
        let b_z = b[2];

        // Is edge aligned with the cutting plane?
        if (a_z - slice_z).abs() < 1e-10 && (b_z - slice_z).abs() < 1e-10 {
            let mut line = IntersectionLine::new();

            if horizontal {
                // All three vertices on the plane
                line.edge_type = FacetEdgeType::Horizontal;
                // Calculate normal to determine orientation
                let v0 = &vertices[0];
                let v1 = &vertices[1];
                let v2 = &vertices[2];
                let normal = (v1[0] - v0[0]) * (v2[1] - v1[1]) - (v1[1] - v0[1]) * (v2[0] - v1[0]);

                let (final_a, final_b, final_a_id, final_b_id) = if normal < 0.0 {
                    // Downward facing, reverse
                    (b, a, b_id, a_id)
                } else {
                    (a, b, a_id, b_id)
                };

                line.a = Point::new(scale(final_a[0]), scale(final_a[1]));
                line.b = Point::new(scale(final_b[0]), scale(final_b[1]));
                line.a_id = final_a_id;
                line.b_id = final_b_id;
                return (FacetSliceType::Cutting, Some(line));
            } else {
                // Two vertices on plane, third above or below
                let third_below = c[2] < slice_z;
                let result = if third_below {
                    FacetSliceType::Slicing
                } else {
                    FacetSliceType::Cutting
                };

                let (final_a, final_b, final_a_id, final_b_id) = if third_below {
                    line.edge_type = FacetEdgeType::Top;
                    (b, a, b_id, a_id)
                } else {
                    line.edge_type = FacetEdgeType::Bottom;
                    (a, b, a_id, b_id)
                };

                line.a = Point::new(scale(final_a[0]), scale(final_a[1]));
                line.b = Point::new(scale(final_b[0]), scale(final_b[1]));
                line.a_id = final_a_id;
                line.b_id = final_b_id;

                if line.a != line.b {
                    return (result, Some(line));
                }
            }
        }

        // Check if only point a is on the plane
        if (a_z - slice_z).abs() < 1e-10 {
            let dominated = point_on_layer
                .map(|idx| points[idx].point_id == a_id)
                .unwrap_or(false);
            if !dominated {
                point_on_layer = Some(points.len());
                points.push(IntersectionPoint::new(scale(a[0]), scale(a[1])).with_point_id(a_id));
            }
        }
        // Check if only point b is on the plane
        else if (b_z - slice_z).abs() < 1e-10 {
            let dominated = point_on_layer
                .map(|idx| points[idx].point_id == b_id)
                .unwrap_or(false);
            if !dominated {
                point_on_layer = Some(points.len());
                points.push(IntersectionPoint::new(scale(b[0]), scale(b[1])).with_point_id(b_id));
            }
        }
        // Check if edge crosses the plane
        else if (a_z < slice_z && b_z > slice_z) || (b_z < slice_z && a_z > slice_z) {
            // Sort edge endpoints consistently
            let (sorted_a, sorted_b, sorted_a_id, sorted_b_id) = if a_id > b_id {
                (b, a, b_id, a_id)
            } else {
                (a, b, a_id, b_id)
            };

            // Calculate intersection point using linear interpolation
            let t = (slice_z - sorted_b[2]) / (sorted_a[2] - sorted_b[2]);

            if t <= 0.0 {
                let dominated = point_on_layer
                    .map(|idx| points[idx].point_id == sorted_a_id)
                    .unwrap_or(false);
                if !dominated {
                    point_on_layer = Some(points.len());
                    points.push(
                        IntersectionPoint::new(scale(sorted_a[0]), scale(sorted_a[1]))
                            .with_point_id(sorted_a_id),
                    );
                }
            } else if t >= 1.0 {
                let dominated = point_on_layer
                    .map(|idx| points[idx].point_id == sorted_b_id)
                    .unwrap_or(false);
                if !dominated {
                    point_on_layer = Some(points.len());
                    points.push(
                        IntersectionPoint::new(scale(sorted_b[0]), scale(sorted_b[1]))
                            .with_point_id(sorted_b_id),
                    );
                }
            } else {
                // General intersection point on the edge
                let x = sorted_b[0] + (sorted_a[0] - sorted_b[0]) * t;
                let y = sorted_b[1] + (sorted_a[1] - sorted_b[1]) * t;
                points.push(IntersectionPoint::new(scale(x), scale(y)).with_edge_id(edge_id));
            }
        }
    }

    // A triangle can intersect a plane at 0, 1, or 2 points
    // 0 or 1 points: no valid intersection line
    // 2 points: valid intersection line
    if points.len() == 2 {
        let mut line = IntersectionLine::new();
        line.edge_type = FacetEdgeType::General;
        line.a = points[1].to_point();
        line.b = points[0].to_point();
        line.a_id = points[1].point_id;
        line.b_id = points[0].point_id;
        line.edge_a_id = points[1].edge_id;
        line.edge_b_id = points[0].edge_id;

        if line.a != line.b {
            return (FacetSliceType::Slicing, Some(line));
        }
    }

    (FacetSliceType::NoSlice, None)
}

/// Build edge ID mapping for consistent edge identification across triangles.
///
/// Each unique edge (pair of vertex indices) gets a unique ID.
fn build_edge_ids(mesh: &TriangleMesh) -> Vec<[i32; 3]> {
    let mut edge_map: HashMap<(u32, u32), i32> = HashMap::new();
    let mut next_edge_id: i32 = 0;
    let mut result = Vec::with_capacity(mesh.triangle_count());

    for tri_idx in 0..mesh.triangle_count() {
        let indices = mesh.triangle_indices(tri_idx);
        let mut edge_ids = [0i32; 3];

        for i in 0..3 {
            let v0 = indices[i];
            let v1 = indices[(i + 1) % 3];
            // Normalize edge direction for consistent lookup
            let key = if v0 < v1 { (v0, v1) } else { (v1, v0) };

            let edge_id = *edge_map.entry(key).or_insert_with(|| {
                let id = next_edge_id;
                next_edge_id += 1;
                id
            });
            edge_ids[i] = edge_id;
        }

        result.push(edge_ids);
    }

    result
}

/// Slice a mesh at multiple Z heights, returning intersection lines for each height.
fn slice_mesh_to_lines(mesh: &TriangleMesh, zs: &[CoordF]) -> Vec<Vec<IntersectionLine>> {
    if mesh.is_empty() || zs.is_empty() {
        return vec![Vec::new(); zs.len()];
    }

    let edge_ids = build_edge_ids(mesh);
    let mut lines: Vec<Vec<IntersectionLine>> = vec![Vec::new(); zs.len()];

    // Process each triangle
    for tri_idx in 0..mesh.triangle_count() {
        let verts = mesh.triangle_vertices(tri_idx);
        let indices = mesh.triangle_indices(tri_idx);
        let tri_edge_ids = &edge_ids[tri_idx];

        // Convert to array format for slice_facet
        let vertices: [[CoordF; 3]; 3] = [
            [verts[0].x, verts[0].y, verts[0].z],
            [verts[1].x, verts[1].y, verts[1].z],
            [verts[2].x, verts[2].y, verts[2].z],
        ];

        // Find Z extents of triangle
        let min_z = vertices[0][2].min(vertices[1][2]).min(vertices[2][2]);
        let max_z = vertices[0][2].max(vertices[1][2]).max(vertices[2][2]);

        // Find the vertex with lowest Z
        let idx_vertex_lowest = if vertices[1][2] == min_z {
            1
        } else if vertices[2][2] == min_z {
            2
        } else {
            0
        };

        // Find which slicing planes intersect this triangle
        let first_layer = zs.partition_point(|&z| z < min_z);
        let last_layer = zs.partition_point(|&z| z <= max_z);

        // Slice against each intersecting plane
        for layer_idx in first_layer..last_layer {
            let slice_z = zs[layer_idx];

            // Skip horizontal triangles (they have no volume contribution)
            if (min_z - max_z).abs() < 1e-10 {
                continue;
            }

            let (slice_type, line) = slice_facet(
                slice_z,
                &vertices,
                &indices,
                tri_edge_ids,
                idx_vertex_lowest,
            );

            if slice_type == FacetSliceType::Slicing {
                if let Some(il) = line {
                    if il.edge_type != FacetEdgeType::Horizontal {
                        lines[layer_idx].push(il);
                    }
                }
            }
        }
    }

    lines
}

/// Chain intersection lines into closed polygons.
///
/// This function takes a collection of line segments and stitches them
/// into closed polygons by matching endpoints.
fn chain_lines_to_polygons(lines: &mut Vec<IntersectionLine>) -> Vec<Polygon> {
    if lines.is_empty() {
        return Vec::new();
    }

    let mut polygons: Vec<Polygon> = Vec::new();

    // Build lookup structures for efficient endpoint matching
    // Key: (point_id, edge_id) where one is -1
    // Value: indices of lines with that endpoint as 'a'
    let mut by_a_point: HashMap<i32, Vec<usize>> = HashMap::new();
    let mut by_a_edge: HashMap<i32, Vec<usize>> = HashMap::new();
    // For coordinate-based fallback matching
    let mut by_a_coord: HashMap<(Coord, Coord), Vec<usize>> = HashMap::new();

    for (idx, line) in lines.iter().enumerate() {
        if line.a_id >= 0 {
            by_a_point.entry(line.a_id).or_default().push(idx);
        }
        if line.edge_a_id >= 0 {
            by_a_edge.entry(line.edge_a_id).or_default().push(idx);
        }
        by_a_coord
            .entry((line.a.x, line.a.y))
            .or_default()
            .push(idx);
    }

    // Process lines to form polygons
    for start_idx in 0..lines.len() {
        if lines[start_idx].skip() {
            continue;
        }

        // Start a new polygon chain
        let mut points: Vec<Point> = Vec::new();
        let mut current_idx = start_idx;
        lines[current_idx].set_skip();

        loop {
            let current_line = &lines[current_idx];
            points.push(current_line.a);

            // Find next line whose 'a' matches current line's 'b'
            let b = &current_line.b;
            let b_id = current_line.b_id;
            let edge_b_id = current_line.edge_b_id;

            let mut next_idx: Option<usize> = None;

            // Try to match by point ID first
            if b_id >= 0 {
                if let Some(candidates) = by_a_point.get(&b_id) {
                    for &idx in candidates {
                        if !lines[idx].skip() {
                            next_idx = Some(idx);
                            break;
                        }
                    }
                }
            }

            // Try to match by edge ID
            if next_idx.is_none() && edge_b_id >= 0 {
                if let Some(candidates) = by_a_edge.get(&edge_b_id) {
                    for &idx in candidates {
                        if !lines[idx].skip() {
                            next_idx = Some(idx);
                            break;
                        }
                    }
                }
            }

            // Fall back to coordinate matching
            if next_idx.is_none() {
                if let Some(candidates) = by_a_coord.get(&(b.x, b.y)) {
                    for &idx in candidates {
                        if !lines[idx].skip() {
                            next_idx = Some(idx);
                            break;
                        }
                    }
                }
            }

            // Looser coordinate matching with tolerance
            if next_idx.is_none() {
                let tolerance: Coord = 10; // ~10 nanometers
                for (idx, line) in lines.iter().enumerate() {
                    if !line.skip()
                        && (line.a.x - b.x).abs() <= tolerance
                        && (line.a.y - b.y).abs() <= tolerance
                    {
                        next_idx = Some(idx);
                        break;
                    }
                }
            }

            match next_idx {
                Some(idx) if idx == start_idx => {
                    // Closed the loop
                    break;
                }
                Some(idx) => {
                    current_idx = idx;
                    lines[current_idx].set_skip();
                }
                None => {
                    // Couldn't close the loop, chain broken
                    // This can happen with non-manifold meshes
                    break;
                }
            }
        }

        // Only add polygons with at least 3 points
        if points.len() >= 3 {
            polygons.push(Polygon::from_points(points));
        }
    }

    polygons
}

/// Convert raw polygons from mesh slicing into ExPolygons.
///
/// Matches the C++ `make_expolygons()` in TriangleMeshSlicer.cpp:1738-1824.
/// Instead of manually classifying polygons by area sign, passes all loops
/// to Clipper's union operation which correctly handles winding/nesting
/// regardless of input polygon orientation.
fn make_expolygons(polygons: Vec<Polygon>) -> ExPolygons {
    if polygons.is_empty() {
        return Vec::new();
    }

    // C++ approach: union_ex(loops, fill_type) handles winding internally.
    // This correctly handles CW/CCW polygons from chain_lines_to_polygons
    // without manual area-based classification.
    let result = crate::clipper_utils::union_polygons_ex(&polygons);

    // C++ also applies a closing offset (offset_out then offset_in) based on
    // closing_radius and extra_offset parameters. For now, use the union
    // result directly — the closing offset can be added when we have the
    // slice_closing_radius config value available here.
    result
}

/// Slice a mesh at a single Z height, returning ExPolygons.
pub fn slice_mesh_at_z(mesh: &TriangleMesh, z: CoordF) -> ExPolygons {
    let mut lines_vec = slice_mesh_to_lines(mesh, &[z]);
    if lines_vec.is_empty() {
        return Vec::new();
    }

    let mut lines = lines_vec.remove(0);
    let polygons = chain_lines_to_polygons(&mut lines);
    make_expolygons(polygons)
}

/// Slice a mesh at multiple Z heights, returning ExPolygons for each height.
pub fn slice_mesh(mesh: &TriangleMesh, zs: &[CoordF]) -> Vec<ExPolygons> {
    let mut all_lines = slice_mesh_to_lines(mesh, zs);

    all_lines
        .iter_mut()
        .map(|lines| {
            let polygons = chain_lines_to_polygons(lines);
            make_expolygons(polygons)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::triangle_mesh::TriangleMesh;

    #[test]
    fn test_slice_cube() {
        // Create a simple cube centered at origin, 10mm on each side
        let mesh = TriangleMesh::cube(10.0);

        // Slice at the middle
        let result = slice_mesh_at_z(&mesh, 0.0);

        // Should produce a single square contour
        assert_eq!(result.len(), 1, "Expected 1 contour for cube slice");

        let expoly = &result[0];
        assert!(expoly.holes.is_empty(), "Cube slice should have no holes");

        // Check approximate area (should be ~100 mm²)
        let area = expoly.area();
        let expected_area = 100.0 * crate::SCALING_FACTOR * crate::SCALING_FACTOR;
        let tolerance = expected_area * 0.01; // 1% tolerance
        assert!(
            (area - expected_area).abs() < tolerance,
            "Area {} not close to expected {}",
            area,
            expected_area
        );
    }

    #[test]
    fn test_slice_cube_multiple_layers() {
        let mesh = TriangleMesh::cube(10.0);

        // Slice at multiple heights
        let zs: Vec<f64> = (-4..=4).map(|i| i as f64).collect();
        let results = slice_mesh(&mesh, &zs);

        assert_eq!(results.len(), zs.len());

        // All slices through the cube should have exactly one contour
        for (i, result) in results.iter().enumerate() {
            assert_eq!(
                result.len(),
                1,
                "Layer {} at z={} should have 1 contour",
                i,
                zs[i]
            );
        }
    }

    #[test]
    fn test_slice_empty_mesh() {
        let mesh = TriangleMesh::new();
        let result = slice_mesh_at_z(&mesh, 0.0);
        assert!(result.is_empty());
    }

    #[test]
    fn test_slice_no_intersection() {
        let mesh = TriangleMesh::cube(10.0);

        // Slice above the cube (cube is at z=-5 to z=5)
        let result = slice_mesh_at_z(&mesh, 10.0);
        assert!(result.is_empty(), "Slice above cube should be empty");

        // Slice below the cube
        let result = slice_mesh_at_z(&mesh, -10.0);
        assert!(result.is_empty(), "Slice below cube should be empty");
    }

    #[test]
    fn test_build_edge_ids() {
        let mesh = TriangleMesh::cube(10.0);
        let edge_ids = build_edge_ids(&mesh);

        // Cube has 12 triangles
        assert_eq!(edge_ids.len(), mesh.triangle_count());

        // Each triangle has 3 edges
        for tri_edges in &edge_ids {
            for &edge_id in tri_edges {
                assert!(edge_id >= 0, "Edge IDs should be non-negative");
            }
        }
    }
}
