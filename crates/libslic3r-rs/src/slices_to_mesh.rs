//! Convert slices back to triangle mesh.
//!
//! This module provides functionality to convert layer slices back into
//! a 3D triangle mesh, mirroring BambuStudio's SlicesToTriangleMesh.cpp.

use crate::geometry::{ExPolygon, Point3F};
use crate::layer::Layer;
use crate::triangle_mesh::{Triangle, TriangleMesh};

/// Convert a stack of layers to a triangle mesh
/// SlicesToTriangleMesh.cpp:15-45
/// This creates a 3D mesh representation of the sliced layers,
/// useful for visualization or export.
pub fn slices_to_mesh(layers: &[Layer]) -> TriangleMesh {
    // Initialize empty mesh
    // SlicesToTriangleMesh.cpp:16
    let mut mesh = TriangleMesh::new();

    // Iterate through all layers
    // SlicesToTriangleMesh.cpp:18-42
    for (layer_idx, layer) in layers.iter().enumerate() {
        // Get bottom Z coordinate for current layer
        // SlicesToTriangleMesh.cpp:19
        let z_bottom = layer.bottom_z_mm();
        // Calculate top Z coordinate from next layer or layer height
        // SlicesToTriangleMesh.cpp:20-24
        let z_top = {
            // Check if there is a next layer
            // SlicesToTriangleMesh.cpp:21
            if layer_idx + 1 < layers.len() {
                layers[layer_idx + 1].bottom_z_mm()
            } else {
                z_bottom + layer.height_mm()
            }
        };

        // Process each slice in the layer
        // SlicesToTriangleMesh.cpp:26-40
        for slice in layer.all_slices() {
            // Extrude slice to mesh between Z heights
            // SlicesToTriangleMesh.cpp:27-38
            extrude_expolygon_to_mesh(&mut mesh, &slice, z_bottom, z_top);
        }
    }

    // Return constructed mesh
    // SlicesToTriangleMesh.cpp:43
    mesh
}

/// Extrude an ExPolygon into a mesh between two Z heights
/// SlicesToTriangleMesh.cpp:48-95
fn extrude_expolygon_to_mesh(
    mesh: &mut TriangleMesh,
    expoly: &ExPolygon,
    z_bottom: f64,
    z_top: f64,
) {
    /// Get base vertex index for this expolygon
    /// SlicesToTriangleMesh.cpp:49
    let base_vertex = mesh.vertex_count() as u32;
    /// Get number of vertices in contour
    /// SlicesToTriangleMesh.cpp:50
    let n = expoly.contour.len() as u32;

    /// Add bottom vertices for all contour points
    /// SlicesToTriangleMesh.cpp:52-60
    for pt in expoly.contour.points().iter() {
        /// Add vertex at bottom Z height
        /// SlicesToTriangleMesh.cpp:53-58
        mesh.add_vertex(Point3F::new(
            pt.x as f64 / 1_000_000.0,
            pt.y as f64 / 1_000_000.0,
            z_bottom,
        ));
    }

    /// Add top vertices for all contour points
    /// SlicesToTriangleMesh.cpp:62-70
    for pt in expoly.contour.points().iter() {
        /// Add vertex at top Z height
        /// SlicesToTriangleMesh.cpp:63-68
        mesh.add_vertex(Point3F::new(
            pt.x as f64 / 1_000_000.0,
            pt.y as f64 / 1_000_000.0,
            z_top,
        ));
    }

    /// Create side walls connecting bottom and top vertices
    /// SlicesToTriangleMesh.cpp:72-92
    for i in 0..n {
        /// Calculate next vertex index with wraparound
        /// SlicesToTriangleMesh.cpp:73
        let next = (i + 1) % n;
        /// Calculate vertex indices for quad vertices
        /// SlicesToTriangleMesh.cpp:74-77
        /// Bottom-left vertex index
        /// SlicesToTriangleMesh.cpp:74
        let v0 = base_vertex + i;
        /// Bottom-right vertex index
        /// SlicesToTriangleMesh.cpp:75
        let v1 = base_vertex + next;
        /// Top-left vertex index
        /// SlicesToTriangleMesh.cpp:76
        let v2 = base_vertex + n + i;
        /// Top-right vertex index
        /// SlicesToTriangleMesh.cpp:77
        let v3 = base_vertex + n + next;

        /// Add first triangle of quad (lower-left triangle)
        /// SlicesToTriangleMesh.cpp:79-82
        mesh.add_triangle(Triangle::new(v0, v1, v2));
        /// Add second triangle of quad (upper-right triangle)
        /// SlicesToTriangleMesh.cpp:84-87
        mesh.add_triangle(Triangle::new(v1, v3, v2));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_slices_to_mesh_empty() {
        let layers: Vec<Layer> = vec![];
        let mesh = slices_to_mesh(&layers);
        assert!(mesh.is_empty());
    }
}
