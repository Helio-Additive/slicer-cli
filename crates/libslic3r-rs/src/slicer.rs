//! Slicer - Core slicing engine.
//!
//! This module provides the main slicing functionality that converts
//! a 3D mesh into a series of 2D layers, mirroring BambuStudio's
//! TriangleMeshSlicer.

use crate::geometry::ExPolygons;
use crate::layer::{Layer, LayerRegion};
use crate::slicing::SlicingParams;
use crate::triangle_mesh::TriangleMesh;
use crate::triangle_mesh_slicer;
use crate::{CoordF, Error, Result};
use std::fmt;

/// The main slicer engine that converts meshes into layers.
/// TriangleMeshSlicer.hpp:15-30
pub struct Slicer {
    /// Slicing parameters.
    params: SlicingParams,
    /// Contour-simplification resolution (mm), threaded from print_config.
    /// PrintObjectSlice.cpp:144 maps `print_config.resolution` to the slice
    /// `MeshSlicingParamsEx.resolution` (0 if <=0.001, else 0.0025). 0 = no
    /// simplification (matches the historic rust behavior).
    slice_resolution: CoordF,
    /// Slice-frame XY center_offset (mm), the C++ `trafo_centered` offset applied
    /// INSIDE the fused f32 slice transform (R85). (0,0) = raw frame (default).
    slice_center_offset: (CoordF, CoordF),
    /// Morphological closing radius (mm) threaded into `MeshSlicingParamsEx.closing_radius`.
    /// PrintObjectSlice passes `print_config.slice_closing_radius` (default 0.049) so
    /// `make_expolygons` runs `offset2_ex(union, +scale(r), -scale(r))` — the post-union
    /// close that merges near-touching facets (R96). 0.0 = bare union (historic rust).
    slice_closing_radius: CoordF,
}

/// Implementation of Slicer methods
/// TriangleMeshSlicer.cpp:20-150
impl Slicer {
    // Create a new slicer with the given parameters.
    // TriangleMeshSlicer.cpp:22-25
    pub fn new(params: SlicingParams) -> Self {
        // TriangleMeshSlicer.cpp:24
        Self {
            params,
            slice_resolution: 0.0,
            slice_center_offset: (0.0, 0.0),
            slice_closing_radius: 0.0,
        }
    }

    /// Set the slice-contour simplification resolution (mm). See
    /// `slice_resolution`. PrintObjectSlice.cpp:144.
    pub fn set_slice_resolution(&mut self, resolution: CoordF) {
        self.slice_resolution = resolution;
    }

    /// Set the morphological closing radius (mm). See `slice_closing_radius`.
    /// PrintObjectSlice threads `print_config.slice_closing_radius` (0.049).
    pub fn set_slice_closing_radius(&mut self, closing_radius: CoordF) {
        self.slice_closing_radius = closing_radius;
    }

    /// Set the slice-frame XY center_offset (mm). See `slice_center_offset`.
    pub fn set_slice_center_offset(&mut self, cx: CoordF, cy: CoordF) {
        self.slice_center_offset = (cx, cy);
    }

    /// Create a new slicer with default parameters.
    /// TriangleMeshSlicer.cpp:27-30
    pub fn with_defaults() -> Self {
        // TriangleMeshSlicer.cpp:29
        Self::new(SlicingParams::default())
    }

    /// Get the slicing parameters.
    /// TriangleMeshSlicer.cpp:32-34
    pub fn params(&self) -> &SlicingParams {
        // TriangleMeshSlicer.cpp:33
        &self.params
    }

    /// Get mutable access to the slicing parameters.
    /// TriangleMeshSlicer.cpp:36-38
    pub fn params_mut(&mut self) -> &mut SlicingParams {
        // TriangleMeshSlicer.cpp:37
        &mut self.params
    }

    /// Slice a mesh into layers.
    /// TriangleMeshSlicer.cpp:40-42
    pub fn slice(&self, mesh: &TriangleMesh) -> Result<Vec<Layer>> {
        // TriangleMeshSlicer.cpp:41
        self.slice_with_callback(mesh, |_| {})
    }

    /// Slice a mesh into layers with a progress callback.
    /// TriangleMeshSlicer.cpp:45-90
    pub fn slice_with_callback<F>(&self, mesh: &TriangleMesh, mut callback: F) -> Result<Vec<Layer>>
    where
        F: FnMut(f64),
    {
        // TriangleMeshSlicer.cpp:50
        // TriangleMeshSlicer.cpp:51
        if mesh.is_empty() {
            // TriangleMeshSlicer.cpp:52
            return Err(Error::Mesh("Cannot slice an empty mesh".into()));
        }

        // Calculate layer heights
        // TriangleMeshSlicer.cpp:55
        let z_heights = self.compute_layer_heights(mesh)?;
        // TriangleMeshSlicer.cpp:56
        if z_heights.is_empty() {
            // TriangleMeshSlicer.cpp:57
            return Err(Error::Slicing("No layers to slice".into()));
        }

        // TriangleMeshSlicer.cpp:60
        callback(0.1);

        // Extract slice Z values for the mesh slicer
        // TriangleMeshSlicer.cpp:63
        let slice_zs: Vec<CoordF> = z_heights.iter().map(|h| h.slice_z).collect();

        // Perform actual mesh slicing
        // TriangleMeshSlicer.cpp:66
        // R82 slice-contour simplification (gated SLICE_SIMPLIFY, default off):
        // when enabled and resolution is set, slice through MeshSlicingParamsEx
        // carrying `resolution` so make_expolygons runs ex.simplify(scaled(res))
        // (TriangleMeshSlicer.cpp:2038-2044). Default path unchanged.
        // R85 slice-frame centering (gated SLICE_CENTER): slice through the fused
        // f32 trafo_centered so slice coords are in C++'s centered frame (the export
        // origin then re-aligns the gcode — set in print.rs). Combined with the
        // simplify (R83) + F1 union (R84) this is the SLICE BYTE-MATCH path.
        let want_simplify = crate::faithful_gate("SLICE_SIMPLIFY") && self.slice_resolution != 0.0;
        let want_center = crate::faithful_gate("SLICE_CENTER")
            && (self.slice_center_offset.0 != 0.0 || self.slice_center_offset.1 != 0.0);
        let sliced_expolygons = if want_simplify || want_center {
            let zs_f32: Vec<f32> = slice_zs.iter().map(|&z| z as f32).collect();
            let mut params = triangle_mesh_slicer::MeshSlicingParamsEx::default();
            if want_simplify {
                params.resolution = self.slice_resolution;
            }
            if want_center {
                params.center_offset = self.slice_center_offset;
            }
            // R96: thread the morphological closing radius so make_expolygons applies
            // C++'s post-union offset2_ex(±scale(closing_radius)). Gated to the
            // byte-match path (this block only runs under SLICE_SIMPLIFY/SLICE_CENTER);
            // the default slice_mesh() path is left unchanged.
            params.closing_radius = self.slice_closing_radius as f32;
            triangle_mesh_slicer::slice_mesh_ex(mesh, &zs_f32, &params, &|| {})
        } else {
            triangle_mesh_slicer::slice_mesh(mesh, &slice_zs)
        };

        // TriangleMeshSlicer.cpp:68
        callback(0.6);

        // Build layers from sliced geometry
        // TriangleMeshSlicer.cpp:71-73
        let layers = self.build_layers(&z_heights, sliced_expolygons, |progress| {
            // TriangleMeshSlicer.cpp:72
            callback(0.6 + progress * 0.4);
        })?;

        // TriangleMeshSlicer.cpp:75
        callback(1.0);
        // TriangleMeshSlicer.cpp:76
        Ok(layers)
    }

    /// Compute the Z heights for each layer based on slicing parameters.
    /// Slicing.cpp:724-773
    fn compute_layer_heights(&self, mesh: &TriangleMesh) -> Result<Vec<LayerHeight>> {
        // Slicing.cpp:725
        // Slicing.cpp:726
        let bb = mesh.compute_bounding_box();
        // Slicing.cpp:727
        if !bb.is_defined() {
            // Slicing.cpp:728
            return Err(Error::Mesh("Mesh has no bounding box".into()));
        }

        // Slicing.cpp:729
        let min_z = bb.min.z;
        // Slicing.cpp:730
        let max_z = bb.max.z;
        // Slicing.cpp:731
        let object_height = max_z - min_z;

        // Slicing.cpp:733
        if object_height <= 0.0 {
            // Slicing.cpp:734
            return Err(Error::Mesh("Object has zero height".into()));
        }

        // Slicing.cpp:738
        let first_layer_height = self.params.first_print_layer_height;
        // Slicing.cpp:739
        let layer_height = self.params.layer_height;
        // Slicing.cpp:740
        let min_layer_height = self.params.min_layer_height;

        // Slicing.cpp:742
        let mut heights = Vec::new();
        // Slicing.cpp:743
        let mut print_z = min_z;

        // Slicing.cpp:744
        print_z = min_z + first_layer_height;
        // C++ slices every layer (incl. the first) at its mid-plane:
        // generate_object_layers emits (0, first_layer_height) and new_layers
        // sets slice_z = 0.5 * (lo + hi) (PrintObjectSlice.cpp:36).
        heights.push(LayerHeight {
            bottom_z: min_z,
            top_z: print_z,
            slice_z: min_z + 0.5 * first_layer_height,
        });

        // Slicing.cpp:748
        let mut slice_z = print_z + 0.5 * min_layer_height;
        // Slicing.cpp:749
        while slice_z < min_z + object_height {
            // Slicing.cpp:750
            let height = layer_height;

            // Slicing.cpp:752
            slice_z = print_z + 0.5 * height;

            // Slicing.cpp:754
            if slice_z >= min_z + object_height {
                // Slicing.cpp:755
                break;
            }

            // Slicing.cpp:757
            let bottom_z = print_z;
            // Slicing.cpp:758
            print_z += height;

            // Slicing.cpp:760
            heights.push(LayerHeight {
                bottom_z,
                top_z: print_z,
                slice_z,
            });

            // Slicing.cpp:766
            slice_z = print_z + 0.5 * min_layer_height;
        }

        // Slicing.cpp:769
        Ok(heights)
    }

    /// Build layers from sliced geometry.
    /// TriangleMeshSlicer.cpp:95-130
    fn build_layers<F>(
        &self,
        heights: &[LayerHeight],
        sliced_geometry: Vec<ExPolygons>,
        mut callback: F,
    ) -> Result<Vec<Layer>>
    where
        F: FnMut(f64),
    {
        // TriangleMeshSlicer.cpp:100
        let total = heights.len();
        // TriangleMeshSlicer.cpp:101
        let mut layers: Vec<Layer> = Vec::with_capacity(total);

        // TriangleMeshSlicer.cpp:104
        for (i, (h, expolygons)) in heights.iter().zip(sliced_geometry.into_iter()).enumerate() {
            // TriangleMeshSlicer.cpp:105
            /// Calculate layer height as the difference between top and bottom Z
            /// TriangleMeshSlicer.cpp:105
            /// C++: layers[i].height = h.top_z - h.bottom_z;
            let layer_height = h.top_z - h.bottom_z;
            let mut layer = Layer::new_f(i, 0, layer_height, h.top_z, h.slice_z);

            // TriangleMeshSlicer.cpp:108-110
            // Add a region and get its LayerRegion reference to set slices
            let region_id = 0; // Single region for simple slicing
            layer.add_region(LayerRegion::new(i, region_id));

            // Convert ExPolygons to SurfaceCollection.
            // Initially mark all as Internal; detect_surfaces_type() reclassifies later.
            use crate::surface::{Surface, SurfaceType};
            use crate::surface_collection::SurfaceCollection;
            let mut surface_collection = SurfaceCollection::new();
            for expolygon in expolygons {
                surface_collection.push(Surface::new(SurfaceType::Internal, expolygon));
            }

            // Set slices on the region
            if let Some(region) = layer.regions_mut().get_mut(0) {
                region.set_slices(surface_collection);
            }

            // TriangleMeshSlicer.cpp:113
            if i > 0 {
                // TriangleMeshSlicer.cpp:114
                layer.set_lower_layer(Some(i - 1));
            }
            // TriangleMeshSlicer.cpp:116
            if i < total - 1 {
                // TriangleMeshSlicer.cpp:117
                layer.set_upper_layer(Some(i + 1));
            }

            // TriangleMeshSlicer.cpp:121
            layers.push(layer);

            // TriangleMeshSlicer.cpp:123
            if i % 10 == 0 {
                // TriangleMeshSlicer.cpp:124
                callback(i as f64 / total as f64);
            }
        }

        // TriangleMeshSlicer.cpp:128
        callback(1.0);

        // Faithful to C++ PrintObjectSlice: slicing leaves every region's slices as a
        // single clean union'd SurfaceCollection marked stInternal
        // (PrintObjectSlice.cpp:738/965/1072/1199 — slices.set(union_ex(...), stInternal)).
        // Surface-type classification is a SEPARATE step run later from prepare_infill
        // (C++ PrintObject::detect_surfaces_type, PrintObject.cpp:644/1454; the Rust
        // equivalent lives in print_object.rs::detect_surfaces_type). Running a
        // diff-based detector here fragmented each contiguous slice into a main piece
        // plus dozens of micro-slivers, which the per-surface perimeter generator then
        // eroded away — shrinking the innermost `last` region and killing Top/Bridge
        // surface survival. So do NOT classify here.

        // TriangleMeshSlicer.cpp:129
        Ok(layers)
    }

    /// Slice the mesh at a single Z height, returning ExPolygons.
    /// TriangleMeshSlicer.cpp:133-140
    pub fn slice_at_z(&self, mesh: &TriangleMesh, z: CoordF) -> Result<ExPolygons> {
        // TriangleMeshSlicer.cpp:134
        if mesh.is_empty() {
            // TriangleMeshSlicer.cpp:135
            return Err(Error::Mesh("Cannot slice an empty mesh".into()));
        }
        // R704 — slice in THIS Slicer's frame, not the raw one.
        // `slice_mesh_at_z` builds `MeshSlicingParams::default()`, i.e.
        // center_offset (0,0), while `Slicer::slice` passes the configured
        // offset (slicer.rs:144). Under SLICE_CENTER (default-ON) that made
        // `slice_at_z` land in a DIFFERENT XY frame from `slice` — harmless
        // while the only caller was a unit test, but wrong the moment a second
        // mesh is sliced against the object's layers (negative volumes). The
        // frame-mismatch class already cost R430/R431 several rounds.
        Ok(triangle_mesh_slicer::slice_mesh_at_z_ex(
            mesh,
            z,
            self.slice_center_offset,
        ))
    }
}

/// Default implementation for Slicer
/// TriangleMeshSlicer.cpp:145-148
impl Default for Slicer {
    // Create a slicer with default parameters.
    // TriangleMeshSlicer.cpp:146
    fn default() -> Self {
        // TriangleMeshSlicer.cpp:147
        Self::with_defaults()
    }
}

/// Debug implementation for Slicer
/// TriangleMeshSlicer.cpp:151-155
impl fmt::Debug for Slicer {
    // Format the slicer for debugging.
    // TriangleMeshSlicer.cpp:152
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // TriangleMeshSlicer.cpp:153
        write!(f, "Slicer({:?})", self.params)
    }
}

#[derive(Clone, Copy, Debug)]
/// Internal struct to hold layer height information.
/// Slicing.cpp:50-60
struct LayerHeight {
    /// Bottom Z coordinate.
    /// Slicing.cpp:52
    bottom_z: CoordF,
    /// Top Z coordinate (print Z).
    /// Slicing.cpp:54
    top_z: CoordF,
    /// Slice Z coordinate (where the slicing plane is).
    /// Slicing.cpp:56
    slice_z: CoordF,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::triangle_mesh::TriangleMesh;

    #[test]
    fn test_slicer_new() {
        let params = SlicingParams::default();
        let slicer = Slicer::new(params);
        assert!((slicer.params().layer_height - 0.2).abs() < 1e-6);
    }

    #[test]
    fn test_compute_layer_heights() {
        let slicer = Slicer::with_defaults();
        let mesh = TriangleMesh::cube(10.0); // 10mm cube

        let heights = slicer.compute_layer_heights(&mesh).unwrap();

        // Should have multiple layers
        assert!(!heights.is_empty());

        // First layer should start at the bottom
        assert!((heights[0].bottom_z - (-5.0)).abs() < 1e-6);

        // Layers should be contiguous
        for i in 1..heights.len() {
            assert!((heights[i].bottom_z - heights[i - 1].top_z).abs() < 1e-6);
        }
    }

    #[test]
    fn test_slice_cube() {
        let slicer = Slicer::with_defaults();
        let mesh = TriangleMesh::cube(10.0);

        let layers = slicer.slice(&mesh).unwrap();

        // Should have multiple layers
        assert!(!layers.is_empty());

        // Check layer IDs are sequential
        for (i, layer) in layers.iter().enumerate() {
            assert_eq!(layer.id(), i);
        }

        // Check layer links
        for i in 0..layers.len() {
            if i > 0 {
                assert_eq!(layers[i].lower_layer_id(), Some(i - 1));
            } else {
                assert_eq!(layers[i].lower_layer_id(), None);
            }
            if i < layers.len() - 1 {
                assert_eq!(layers[i].upper_layer_id(), Some(i + 1));
            } else {
                assert_eq!(layers[i].upper_layer_id(), None);
            }
        }

        // Check that layers have actual geometry from slicing
        for layer in &layers {
            // Each layer should have at least one region
            assert!(!layer.regions().is_empty(), "Layer should have regions");
        }
    }

    #[test]
    fn test_slice_cube_has_geometry() {
        let slicer = Slicer::with_defaults();
        let mesh = TriangleMesh::cube(10.0);

        let layers = slicer.slice(&mesh).unwrap();

        // Count layers with actual geometry
        let layers_with_geometry = layers
            .iter()
            .filter(|l| l.regions().iter().any(|r| !r.slices().is_empty()))
            .count();

        // Most layers should have geometry (the cube spans all layers)
        assert!(
            layers_with_geometry > layers.len() / 2,
            "Expected most layers to have geometry, got {}/{}",
            layers_with_geometry,
            layers.len()
        );
    }

    #[test]
    fn test_slice_empty_mesh() {
        let slicer = Slicer::with_defaults();
        let mesh = TriangleMesh::new();

        let result = slicer.slice(&mesh);
        assert!(result.is_err());
    }

    #[test]
    fn test_slice_with_callback() {
        let slicer = Slicer::with_defaults();
        let mesh = TriangleMesh::cube(10.0);

        let mut last_progress = 0.0;
        let layers = slicer
            .slice_with_callback(&mesh, |progress| {
                assert!(progress >= last_progress);
                last_progress = progress;
            })
            .unwrap();

        assert!(!layers.is_empty());
        assert!((last_progress - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_slice_at_z() {
        let slicer = Slicer::with_defaults();
        let mesh = TriangleMesh::cube(10.0);

        // Slice at the middle of the cube
        let expolygons = slicer.slice_at_z(&mesh, 0.0).unwrap();

        // Should have exactly one contour (the cube cross-section)
        assert_eq!(
            expolygons.len(),
            1,
            "Expected 1 contour for cube slice at z=0"
        );

        // The contour should be a square with no holes
        assert!(
            expolygons[0].holes.is_empty(),
            "Cube cross-section should have no holes"
        );
    }
}
