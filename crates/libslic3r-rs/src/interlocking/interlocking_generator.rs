//! Interlocking structure generator.
//!
//! C++ Reference:
//! - Interlocking/InterlockingGenerator.hpp
//! - Interlocking/InterlockingGenerator.cpp
//!
//! Generates interlocking structures between two adjacent mesh regions
//! using different extruders. The structure consists of horizontal beams
//! of the two materials interlaced, with alternating direction in Z.

use std::cell::RefCell;
use std::collections::HashSet;

use super::voxel_utils::{DilationKernel, DilationKernelType, GridPoint3, VoxelUtils};
use crate::geometry::{ExPolygon, Point, Polygon};
use crate::Coord;

/// Hash for GridPoint3 (same as C++ implementation).
///
/// InterlockingGenerator.cpp:11-22
fn grid_point_hash(pp: &GridPoint3) -> u64 {
    let prime: i64 = 31;
    let mut result: i64 = 89;
    result = result * prime + pp[0];
    result = result * prime + pp[1];
    result = result * prime + pp[2];
    result as u64
}

/// Distance between models to be considered adjacent.
///
/// InterlockingGenerator.hpp:152
pub const IGNORED_GAP: Coord = 100;

/// Interlocking generator for creating interlocking structures between mesh regions.
///
/// InterlockingGenerator.hpp:42-170
pub struct InterlockingGenerator {
    /// Index of the first region
    pub region_a_index: usize,
    /// Index of the second region
    pub region_b_index: usize,
    /// Width of the interlocking beams
    pub beam_width: Coord,
    /// Boundary avoidance distance
    pub boundary_avoidance: Coord,
    /// The voxel utility instance
    pub vu: VoxelUtils,
    /// Rotation angle for the interlocking pattern
    pub rotation: f32,
    /// Voxel cell size
    pub cell_size: [Coord; 3],
    /// Number of layers per beam
    pub beam_layer_count: Coord,
    /// Dilation kernel for interface
    pub interface_dilation: DilationKernel,
    /// Dilation kernel for air detection
    pub air_dilation: DilationKernel,
    /// Whether to filter out cells touching air
    pub air_filtering: bool,
}

impl InterlockingGenerator {
    /// Create a new interlocking generator.
    ///
    /// InterlockingGenerator.hpp:68-91
    pub fn new(
        region_a_index: usize,
        region_b_index: usize,
        beam_width: Coord,
        boundary_avoidance: Coord,
        rotation: f32,
        cell_size: [Coord; 3],
        beam_layer_count: Coord,
        interface_dilation: DilationKernel,
        air_dilation: DilationKernel,
        air_filtering: bool,
    ) -> Self {
        Self {
            region_a_index,
            region_b_index,
            beam_width,
            boundary_avoidance,
            vu: VoxelUtils::new(cell_size),
            rotation,
            cell_size,
            beam_layer_count,
            interface_dilation,
            air_dilation,
            air_filtering,
        }
    }

    /// Add boundary cells from a single layer's polygons.
    ///
    /// InterlockingGenerator.cpp:280-301
    pub fn add_layer_boundary_cells(
        &self,
        layers: &[ExPolygon],
        layer_cnt: i32,
        kernel: &DilationKernel,
        cells: &mut HashSet<GridPoint3>,
    ) {
        let cells_ref = RefCell::new(cells);
        let voxel_emplacer = |p: GridPoint3| -> bool {
            if p[2] < 0 {
                return true;
            }
            cells_ref.borrow_mut().insert(p);
            true
        };

        let z = layer_cnt as Coord;
        self.vu
            .walk_dilated_polygons_multi(layers, z, kernel, &voxel_emplacer);

        // Skin processing (empty for single layer)
        // InterlockingGenerator.cpp:295-299
        let skin: Vec<ExPolygon> = Vec::new();
        self.vu
            .walk_dilated_areas_multi(&skin, z, kernel, &voxel_emplacer);
    }

    /// Add boundary cells from multiple layers' polygons.
    ///
    /// InterlockingGenerator.cpp:304-326
    pub fn add_boundary_cells(
        &self,
        layers: &[Vec<ExPolygon>],
        kernel: &DilationKernel,
        cells: &mut HashSet<GridPoint3>,
    ) {
        let cells_ref = RefCell::new(cells);
        let voxel_emplacer = |p: GridPoint3| -> bool {
            if p[2] < 0 {
                return true;
            }
            cells_ref.borrow_mut().insert(p);
            true
        };

        for layer_nr in 0..layers.len() {
            let z = layer_nr as Coord;
            self.vu
                .walk_dilated_polygons_multi(&layers[layer_nr], z, kernel, &voxel_emplacer);

            // Compute skin (XOR with previous layer)
            // InterlockingGenerator.cpp:319-324
            let skin = if layer_nr > 0 {
                // TODO: xor_ex + opening_ex when clipper is available
                Vec::new()
            } else {
                layers[layer_nr].clone()
            };

            self.vu
                .walk_dilated_areas_multi(&skin, z, kernel, &voxel_emplacer);
        }
    }

    /// Generate the microstructure pattern for a single layer (embedding).
    ///
    /// InterlockingGenerator.cpp:347-364
    pub fn generate_layer_microstructure(&self) -> Vec<ExPolygon> {
        let middle = self.cell_size[0] / 2;
        let width = [middle, self.cell_size[0] - middle];

        let mut cell_areas = Vec::with_capacity(2);

        for mesh_idx in 0..2 {
            let offset = if mesh_idx == 1 { middle } else { 0 };
            let area_w = width[mesh_idx];
            let area_h = self.cell_size[1];

            let poly = Polygon::from_points(vec![
                Point::new(offset, 0),
                Point::new(offset + area_w, 0),
                Point::new(offset + area_w, area_h),
                Point::new(offset, area_h),
            ]);

            cell_areas.push(ExPolygon::new(poly));
        }

        cell_areas
    }

    /// Generate the microstructure pattern for interlocking beams.
    ///
    /// Returns [layer_type][mesh_idx] ExPolygons.
    ///
    /// InterlockingGenerator.cpp:366-395
    pub fn generate_microstructure(&self) -> Vec<Vec<Vec<ExPolygon>>> {
        let middle = self.cell_size[0] / 2;
        let width = [middle, self.cell_size[0] - middle];

        // Layer type 0: horizontal beams
        let mut layer0 = Vec::with_capacity(2);
        for mesh_idx in 0..2 {
            let offset = if mesh_idx == 1 { middle } else { 0 };
            let area_w = width[mesh_idx];
            let area_h = self.cell_size[1];

            let poly = Polygon::from_points(vec![
                Point::new(offset, 0),
                Point::new(offset + area_w, 0),
                Point::new(offset + area_w, area_h),
                Point::new(offset, area_h),
            ]);

            layer0.push(vec![ExPolygon::new(poly)]);
        }

        // Layer type 1: rotated 90 degrees (swap x and y)
        // InterlockingGenerator.cpp:387-394
        let mut layer1 = Vec::with_capacity(2);
        for mesh_idx in 0..2 {
            let mut polys = Vec::new();
            for expoly in &layer0[mesh_idx] {
                let rotated_points: Vec<Point> = expoly
                    .contour
                    .points
                    .iter()
                    .map(|p| Point::new(p.y, p.x))
                    .collect();
                polys.push(ExPolygon::new(Polygon::from_points(rotated_points)));
            }
            layer1.push(polys);
        }

        vec![layer0, layer1]
    }

    /// Get shell voxels for a single layer.
    ///
    /// InterlockingGenerator.cpp:243-255
    pub fn get_layer_shell_voxels(
        &self,
        kernel: &DilationKernel,
        layer_polys_a: &[ExPolygon],
        layer_polys_b: &[ExPolygon],
        layer_id: i32,
    ) -> [HashSet<GridPoint3>; 2] {
        let mut voxels = [HashSet::new(), HashSet::new()];

        // Region A
        self.add_layer_boundary_cells(layer_polys_a, layer_id, kernel, &mut voxels[0]);
        // Region B
        self.add_layer_boundary_cells(layer_polys_b, layer_id, kernel, &mut voxels[1]);

        voxels
    }

    /// Get shell voxels across all layers.
    ///
    /// InterlockingGenerator.cpp:257-279
    pub fn get_shell_voxels(
        &self,
        kernel: &DilationKernel,
        layers_a: &[Vec<ExPolygon>],
        layers_b: &[Vec<ExPolygon>],
    ) -> [HashSet<GridPoint3>; 2] {
        let mut voxels = [HashSet::new(), HashSet::new()];

        self.add_boundary_cells(layers_a, kernel, &mut voxels[0]);
        self.add_boundary_cells(layers_b, kernel, &mut voxels[1]);

        voxels
    }

    /// Compute intersection of two mesh voxel sets (cells present in both).
    ///
    /// Returns (has_any_mesh, has_all_meshes) where:
    /// - has_any_mesh: union of both sets
    /// - has_all_meshes: intersection of both sets
    pub fn compute_voxel_intersection(
        voxels_a: &HashSet<GridPoint3>,
        voxels_b: &HashSet<GridPoint3>,
    ) -> (HashSet<GridPoint3>, HashSet<GridPoint3>) {
        let mut has_any = voxels_a.clone();
        for v in voxels_b {
            has_any.insert(*v);
        }

        let has_all: HashSet<GridPoint3> = voxels_a.intersection(voxels_b).cloned().collect();

        (has_any, has_all)
    }

    /// Apply microstructure to a set of cells and produce the resulting polygons
    /// for each region per layer.
    ///
    /// Simplified version of InterlockingGenerator.cpp:430-490
    pub fn apply_microstructure(
        &self,
        cells: &HashSet<GridPoint3>,
        num_layers: usize,
    ) -> [Vec<Vec<ExPolygon>>; 2] {
        let microstructure = self.generate_microstructure();
        let num_interlocking_layers =
            (num_layers + self.beam_layer_count as usize - 1) / self.beam_layer_count as usize;

        let mut structure: [Vec<Vec<ExPolygon>>; 2] = [
            vec![Vec::new(); num_interlocking_layers],
            vec![Vec::new(); num_interlocking_layers],
        ];

        for &grid_loc in cells {
            let bottom_corner = self.vu.to_lower_corner(grid_loc);

            for mesh_idx in 0..2 {
                let mut layer_nr = bottom_corner[2];
                while layer_nr < bottom_corner[2] + self.cell_size[2]
                    && (layer_nr as usize) < num_layers
                {
                    let interlocking_layer =
                        (layer_nr as usize / self.beam_layer_count as usize) % microstructure.len();
                    let struct_layer = layer_nr as usize / self.beam_layer_count as usize;

                    if struct_layer < num_interlocking_layers {
                        let areas = &microstructure[interlocking_layer][mesh_idx];
                        for area in areas {
                            let mut translated = area.clone();
                            translated.translate(Point::new(bottom_corner[0], bottom_corner[1]));
                            structure[mesh_idx][struct_layer].push(translated);
                        }
                    }

                    layer_nr += self.beam_layer_count;
                }
            }
        }

        structure
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_generator() -> InterlockingGenerator {
        let beam_width = 200;
        let cell_width = beam_width * 2;
        InterlockingGenerator::new(
            0,
            1,
            beam_width,
            2,
            0.0,
            [cell_width, cell_width, 2],
            1,
            DilationKernel::new([2, 2, 2], DilationKernelType::Prism),
            DilationKernel::new([2, 2, 2], DilationKernelType::Prism),
            true,
        )
    }

    #[test]
    fn test_generator_creation() {
        let gen = make_generator();
        assert_eq!(gen.region_a_index, 0);
        assert_eq!(gen.region_b_index, 1);
        assert_eq!(gen.beam_width, 200);
    }

    #[test]
    fn test_generate_layer_microstructure() {
        let gen = make_generator();
        let areas = gen.generate_layer_microstructure();
        assert_eq!(areas.len(), 2);
        // Each mesh gets a rectangular area
        for area in &areas {
            assert_eq!(area.contour.points.len(), 4);
        }
    }

    #[test]
    fn test_generate_microstructure() {
        let gen = make_generator();
        let micro = gen.generate_microstructure();
        // Two layer types
        assert_eq!(micro.len(), 2);
        // Each layer type has two meshes
        assert_eq!(micro[0].len(), 2);
        assert_eq!(micro[1].len(), 2);
    }

    #[test]
    fn test_voxel_intersection() {
        let mut a = HashSet::new();
        a.insert([0, 0, 0]);
        a.insert([1, 0, 0]);
        a.insert([0, 1, 0]);

        let mut b = HashSet::new();
        b.insert([1, 0, 0]);
        b.insert([0, 1, 0]);
        b.insert([1, 1, 0]);

        let (any, all) = InterlockingGenerator::compute_voxel_intersection(&a, &b);
        assert_eq!(any.len(), 4); // union
        assert_eq!(all.len(), 2); // intersection: [1,0,0] and [0,1,0]
    }

    #[test]
    fn test_add_boundary_cells_empty() {
        let gen = make_generator();
        let mut cells = HashSet::new();
        let layers: Vec<Vec<ExPolygon>> = vec![vec![]];
        gen.add_boundary_cells(&layers, &gen.interface_dilation, &mut cells);
        // No cells added from empty layer
    }

    #[test]
    fn test_apply_microstructure_empty() {
        let gen = make_generator();
        let cells = HashSet::new();
        let structure = gen.apply_microstructure(&cells, 10);
        // No cells means no structure
        assert!(structure[0].iter().all(|layer| layer.is_empty()));
        assert!(structure[1].iter().all(|layer| layer.is_empty()));
    }

    #[test]
    fn test_apply_microstructure_with_cells() {
        let gen = make_generator();
        let mut cells = HashSet::new();
        cells.insert([0, 0, 0]);
        let structure = gen.apply_microstructure(&cells, 4);
        // Should have some structure generated
        let total: usize = structure[0].iter().map(|l| l.len()).sum::<usize>()
            + structure[1].iter().map(|l| l.len()).sum::<usize>();
        assert!(total > 0);
    }
}
