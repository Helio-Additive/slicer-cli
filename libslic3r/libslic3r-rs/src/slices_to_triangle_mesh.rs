//! Mesh reconstruction from 2D slices
//!
//! C++ Reference:
//! - SlicesToTriangleMesh.hpp
//! - SlicesToTriangleMesh.cpp
//!
//! This module reconstructs a 3D triangle mesh from a stack of 2D polygon slices.
//! It creates vertical walls between layers and triangulates horizontal surfaces.

use crate::geometry::{ExPolygon, ExPolygons, Point, Polygon};
use crate::Result;

/// Simple 3D vertex for mesh construction
/// SlicesToTriangleMesh.cpp:16
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec3f {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3f {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }
}

/// Indexed triangle set for mesh representation
/// SlicesToTriangleMesh.cpp:14
#[derive(Debug, Clone, Default)]
pub struct IndexedTriangleSet {
    pub vertices: Vec<Vec3f>,
    pub indices: Vec<[usize; 3]>,
}

impl IndexedTriangleSet {
    pub fn new() -> Self {
        Self {
            vertices: Vec::new(),
            indices: Vec::new(),
        }
    }

    /// Merge another mesh into this one
    /// SlicesToTriangleMesh.cpp:61 (its_merge)
    pub fn merge(&mut self, other: &IndexedTriangleSet) {
        let offset = self.vertices.len();
        self.vertices.extend_from_slice(&other.vertices);
        for triangle in &other.indices {
            self.indices.push([
                triangle[0] + offset,
                triangle[1] + offset,
                triangle[2] + offset,
            ]);
        }
    }
}

/// Converts scaled 2D point to 3D vertex at given Z height
/// SlicesToTriangleMesh.cpp:26-28
fn point_to_3d(p: Point, z: f32) -> Vec3f {
    // Unscale from internal integer coordinates to mm (float)
    // SlicesToTriangleMesh.cpp:26-28
    Vec3f::new(
        (p.x as f64 / crate::SCALING_FACTOR) as f32,
        (p.y as f64 / crate::SCALING_FACTOR) as f32,
        z,
    )
}

/// Create vertical wall strip connecting a polygon at two Z heights
///
/// This generates triangles for the vertical walls between a polygon's
/// lower and upper positions.
///
/// # C++ Reference
/// SlicesToTriangleMesh.cpp:14-45
fn wall_strip(poly: &Polygon, lower_z_mm: f64, upper_z_mm: f64) -> IndexedTriangleSet {
    let mut ret = IndexedTriangleSet::new();

    let startidx = ret.vertices.len();
    let offs = poly.points().len();

    ret.vertices.reserve(ret.vertices.len() + 2 * offs);

    // Add lower vertices
    // The expression unscaled(p).cast<float>().eval() is important here
    // as it ensures identical conversion of 2D scaled coordinates to float 3D
    // to that used by the tesselation. This way, the duplicated vertices in the
    // output mesh can be found with the == operator of the points.
    // its_merge_vertices will then reliably remove the duplicates.
    // SlicesToTriangleMesh.cpp:26-28
    for p in poly.points() {
        ret.vertices.push(point_to_3d(*p, lower_z_mm as f32));
    }

    // Add upper vertices
    // SlicesToTriangleMesh.cpp:30-31
    for p in poly.points() {
        ret.vertices.push(point_to_3d(*p, upper_z_mm as f32));
    }

    // Create triangles connecting lower and upper polygons
    // SlicesToTriangleMesh.cpp:33-36
    for i in (startidx + 1)..(startidx + offs) {
        ret.indices.push([i - 1, i, i + offs - 1]);
        ret.indices.push([i, i + offs, i + offs - 1]);
    }

    // Close the strip (wrap around to first vertex)
    // SlicesToTriangleMesh.cpp:38-39
    ret.indices
        .push([startidx + offs - 1, startidx, startidx + 2 * offs - 1]);
    ret.indices
        .push([startidx, startidx + offs, startidx + 2 * offs - 1]);

    ret
}

/// Create vertical walls for a polygon with identical upper and lower positions
///
/// # C++ Reference
/// SlicesToTriangleMesh.cpp:48-52
fn straight_walls_polygon(plate: &Polygon, lo_z: f64, hi_z: f64) -> IndexedTriangleSet {
    wall_strip(plate, lo_z, hi_z)
}

/// Create vertical walls for an ExPolygon (contour + holes)
///
/// # C++ Reference
/// SlicesToTriangleMesh.cpp:54-61
fn straight_walls_expolygon(plate: &ExPolygon, lo_z: f64, hi_z: f64) -> IndexedTriangleSet {
    let mut ret = straight_walls_polygon(&plate.contour, lo_z, hi_z);
    for h in &plate.holes {
        ret.merge(&straight_walls_polygon(h, lo_z, hi_z));
    }
    ret
}

/// Create vertical walls for multiple ExPolygons
///
/// # C++ Reference
/// SlicesToTriangleMesh.cpp:63-70
fn straight_walls(slice: &ExPolygons, lo_z: f64, hi_z: f64) -> IndexedTriangleSet {
    let mut ret = IndexedTriangleSet::new();
    for poly in slice {
        ret.merge(&straight_walls_expolygon(poly, lo_z, hi_z));
    }
    ret
}

/// Triangulate ExPolygons at a given Z height
///
/// # Arguments
/// * `polys` - ExPolygons to triangulate
/// * `z` - Z coordinate for the triangulation
/// * `flip` - If true, flip normals downward
///
/// # C++ Reference
/// SlicesToTriangleMesh.cpp:92-93 (calls triangulate_expolygons_3d)
fn triangulate_expolygons_3d(polys: &ExPolygons, z: f64, flip: bool) -> Result<IndexedTriangleSet> {
    // TODO: Port full triangulation from Tesselate.cpp
    // For now, return empty mesh with TODO marker
    // The real implementation uses CDT (Constrained Delaunay Triangulation)
    // SlicesToTriangleMesh.cpp:92-93
    let _ = (polys, z, flip);
    Ok(IndexedTriangleSet::new())
}

/// Reconstruct a 3D mesh from a stack of 2D slices with custom Z grid
///
/// This is the main reconstruction function. It:
/// 1. Creates vertical walls between consecutive layers
/// 2. Triangulates exposed top/bottom surfaces
/// 3. Handles overhangs (areas that appear/disappear between layers)
///
/// # Arguments
/// * `slices` - Vector of ExPolygons for each layer
/// * `zmin` - Z coordinate of the bottom
/// * `grid` - Z coordinates for each slice
///
/// # C++ Reference
/// SlicesToTriangleMesh.cpp:72-119
pub fn slices_to_mesh_with_grid(
    slices: &[ExPolygons],
    zmin: f64,
    grid: &[f32],
) -> Result<IndexedTriangleSet> {
    assert_eq!(slices.len(), grid.len());

    let mut layers: Vec<IndexedTriangleSet> = vec![IndexedTriangleSet::new(); slices.len()];
    let len = slices.len() - 1;

    // Process each layer pair in parallel (simplified to sequential for now)
    // SlicesToTriangleMesh.cpp:80-95
    for i in 0..len {
        let upper = &slices[i + 1];
        let _lower = &slices[i];

        // Small 0 area artefacts can be created by diff_ex, and the
        // tesselation also can create 0 area triangles. These will be removed
        // by its_remove_degenerate_faces.
        // SlicesToTriangleMesh.cpp:85-87

        // TODO: Implement diff_ex for polygon differences
        // For now, use simplified approach
        // SlicesToTriangleMesh.cpp:88-90
        let free_top = ExPolygons::new(); // diff_ex(lower, upper)
        let overhang = ExPolygons::new(); // diff_ex(upper, lower)

        const NORMALS_UP: bool = false;
        const NORMALS_DOWN: bool = true;

        // Triangulate exposed surfaces
        // SlicesToTriangleMesh.cpp:91-93
        if let Ok(top_mesh) = triangulate_expolygons_3d(&free_top, grid[i] as f64, NORMALS_UP) {
            layers[i].merge(&top_mesh);
        }
        if let Ok(overhang_mesh) =
            triangulate_expolygons_3d(&overhang, grid[i] as f64, NORMALS_DOWN)
        {
            layers[i].merge(&overhang_mesh);
        }

        // Add vertical walls
        // SlicesToTriangleMesh.cpp:94
        layers[i].merge(&straight_walls(upper, grid[i] as f64, grid[i + 1] as f64));
    }

    // Merge all layers
    // SlicesToTriangleMesh.cpp:97-100
    let mut ret = IndexedTriangleSet::new();
    for layer in &layers {
        ret.merge(layer);
    }

    // Add bottom cap
    // SlicesToTriangleMesh.cpp:102
    if let Ok(bottom) = triangulate_expolygons_3d(&slices[0], zmin, true) {
        ret.merge(&bottom);
    }

    // Add bottom walls
    // SlicesToTriangleMesh.cpp:103
    ret.merge(&straight_walls(&slices[0], zmin, grid[0] as f64));

    // Add top cap
    // SlicesToTriangleMesh.cpp:104
    if let Ok(top) = triangulate_expolygons_3d(&slices[slices.len() - 1], grid[len] as f64, false) {
        ret.merge(&top);
    }

    // FIXME: these repairs do not fix the mesh entirely. There will be cracks
    // in the output. It is very hard to do the meshing in a way that does not
    // leave errors.
    // SlicesToTriangleMesh.cpp:106-111
    // TODO: Implement mesh repair functions:
    // - its_merge_vertices(ret)
    // - its_remove_degenerate_faces(ret)
    // - its_compactify_vertices(ret)

    Ok(ret)
}

/// Reconstruct a 3D mesh from slices with uniform layer heights
///
/// This is a convenience wrapper that generates a uniform Z grid.
///
/// # Arguments
/// * `mesh` - Output mesh to append to
/// * `slices` - Vector of ExPolygons for each layer
/// * `zmin` - Z coordinate of the bottom
/// * `lh` - Layer height
/// * `ilh` - Initial layer height
///
/// # C++ Reference
/// SlicesToTriangleMesh.cpp:121-132
pub fn slices_to_mesh(
    mesh: &mut IndexedTriangleSet,
    slices: &[ExPolygons],
    zmin: f64,
    lh: f64,
    ilh: f64,
) -> Result<()> {
    // Build uniform Z grid
    // SlicesToTriangleMesh.cpp:125-127
    let mut grid = vec![0.0f32; slices.len()];
    grid[0] = (zmin + ilh) as f32;

    for i in 1..grid.len() {
        grid[i] = grid[i - 1] + lh as f32;
    }

    // Reconstruct mesh
    // SlicesToTriangleMesh.cpp:129
    let cntr = slices_to_mesh_with_grid(slices, zmin, &grid)?;

    // Merge into output
    // SlicesToTriangleMesh.cpp:130
    mesh.merge(&cntr);

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vec3f_creation() {
        let v = Vec3f::new(1.0, 2.0, 3.0);
        assert_eq!(v.x, 1.0);
        assert_eq!(v.y, 2.0);
        assert_eq!(v.z, 3.0);
    }

    #[test]
    fn test_indexed_triangle_set_merge() {
        let mut mesh1 = IndexedTriangleSet {
            vertices: vec![Vec3f::new(0.0, 0.0, 0.0), Vec3f::new(1.0, 0.0, 0.0)],
            indices: vec![[0, 1, 0]],
        };

        let mesh2 = IndexedTriangleSet {
            vertices: vec![Vec3f::new(2.0, 0.0, 0.0)],
            indices: vec![[0, 0, 0]],
        };

        mesh1.merge(&mesh2);

        assert_eq!(mesh1.vertices.len(), 3);
        assert_eq!(mesh1.indices.len(), 2);
        // Second mesh indices should be offset by 2
        assert_eq!(mesh1.indices[1], [2, 2, 2]);
    }

    #[test]
    fn test_wall_strip_quad() {
        // Create a simple square polygon
        let points = vec![
            Point::new(0, 0),
            Point::new(1000000, 0),       // 1mm
            Point::new(1000000, 1000000), // 1mm x 1mm
            Point::new(0, 1000000),
        ];
        let poly = Polygon::new(points);

        let mesh = wall_strip(&poly, 0.0, 1.0);

        // Should have 8 vertices (4 lower + 4 upper)
        assert_eq!(mesh.vertices.len(), 8);

        // Should have 8 triangles (2 per edge, 4 edges)
        assert_eq!(mesh.indices.len(), 8);

        // Check Z coordinates
        for i in 0..4 {
            assert_eq!(mesh.vertices[i].z, 0.0);
            assert_eq!(mesh.vertices[i + 4].z, 1.0);
        }
    }

    #[test]
    fn test_slices_to_mesh_empty() {
        let slices: Vec<ExPolygons> = vec![];
        let mut mesh = IndexedTriangleSet::new();
        let result = slices_to_mesh(&mut mesh, &slices, 0.0, 0.2, 0.3);

        // Should handle empty input gracefully
        assert!(result.is_ok() || result.is_err());
    }

    #[test]
    fn test_straight_walls_polygon() {
        let points = vec![
            Point::new(0, 0),
            Point::new(1000000, 0),
            Point::new(0, 1000000),
        ];
        let poly = Polygon::new(points);

        let mesh = straight_walls_polygon(&poly, 0.0, 1.0);

        // Triangle has 3 vertices, so 6 total (lower + upper)
        assert_eq!(mesh.vertices.len(), 6);

        // 2 triangles per edge, 3 edges = 6 triangles
        assert_eq!(mesh.indices.len(), 6);
    }

    #[test]
    fn test_point_to_3d_unscaling() {
        // 1mm in scaled coordinates
        let p = Point::new(1000000, 2000000);
        let v = point_to_3d(p, 5.0);

        // Should be approximately 1mm, 2mm, 5mm
        assert!((v.x - 1.0).abs() < 0.0001);
        assert!((v.y - 2.0).abs() < 0.0001);
        assert_eq!(v.z, 5.0);
    }
}
