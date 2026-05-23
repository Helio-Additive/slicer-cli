//! Voxel utilities for interlocking structure generation.
//!
//! C++ Reference:
//! - Interlocking/VoxelUtils.hpp
//! - Interlocking/VoxelUtils.cpp
//!
//! Provides utilities for walking over a 3D voxel grid, intersecting voxels
//! with lines, polygons, and areas. Used by the interlocking generator to
//! determine which voxel cells are occupied by mesh geometry.

use crate::geometry::{ExPolygon, Point, Polygon};
use crate::Coord;

/// A 3D grid point (voxel coordinate).
///
/// VoxelUtils.hpp:15: `using GridPoint3 = Vec3crd;`
pub type GridPoint3 = [Coord; 3];

/// Type for grid coordinates.
///
/// VoxelUtils.hpp:65
pub type GridCoord = Coord;

/// Dilation kernel type.
///
/// VoxelUtils.hpp:44-49
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DilationKernelType {
    /// Cubic kernel: all voxels in a cube around reference
    Cube,
    /// Diamond kernel: uses manhattan distance
    Diamond,
    /// Prism kernel: diamond in XY, extrudes in Z
    Prism,
}

/// Dilation kernel for morphological operations on voxel grids.
///
/// Contains the relative positions of voxels to process around a reference cell.
///
/// VoxelUtils.hpp:20-53
#[derive(Debug, Clone)]
pub struct DilationKernel {
    /// Size of the kernel in voxel cells
    /// VoxelUtils.hpp:50
    pub kernel_size: GridPoint3,

    /// Kernel type
    /// VoxelUtils.hpp:51
    pub kernel_type: DilationKernelType,

    /// All offset positions relative to the reference cell
    /// VoxelUtils.hpp:52
    pub relative_cells: Vec<GridPoint3>,
}

impl DilationKernel {
    /// Create a new dilation kernel.
    ///
    /// VoxelUtils.cpp:12-48
    pub fn new(kernel_size: GridPoint3, kernel_type: DilationKernelType) -> Self {
        let mult = kernel_size[0] * kernel_size[1] * kernel_size[2];
        let half_kernel = [kernel_size[0] / 2, kernel_size[1] / 2, kernel_size[2] / 2];

        let start = [-half_kernel[0], -half_kernel[1], -half_kernel[2]];
        let end = [
            kernel_size[0] - half_kernel[0],
            kernel_size[1] - half_kernel[1],
            kernel_size[2] - half_kernel[2],
        ];

        let mut relative_cells = Vec::with_capacity(mult as usize);

        for x in start[0]..end[0] {
            for y in start[1]..end[1] {
                for z in start[2]..end[2] {
                    if kernel_type != DilationKernelType::Cube {
                        let mut limit = [
                            if x < 0 { start[0] } else { end[0] - 1 },
                            if y < 0 { start[1] } else { end[1] - 1 },
                            if z < 0 { start[2] } else { end[2] - 1 },
                        ];
                        if limit[0] == 0 {
                            limit[0] = 1;
                        }
                        if limit[1] == 0 {
                            limit[1] = 1;
                        }
                        if limit[2] == 0 {
                            limit[2] = 1;
                        }

                        let rel_dists = [
                            mult * x / limit[0],
                            mult * y / limit[1],
                            mult * z / limit[2],
                        ];

                        match kernel_type {
                            DilationKernelType::Diamond => {
                                if rel_dists[0].abs() + rel_dists[1].abs() + rel_dists[2].abs()
                                    > mult
                                {
                                    continue;
                                }
                            }
                            DilationKernelType::Prism => {
                                if rel_dists[0].abs() + rel_dists[1].abs() > mult {
                                    continue;
                                }
                            }
                            _ => {}
                        }
                    }

                    relative_cells.push([x, y, z]);
                }
            }
        }

        Self {
            kernel_size,
            kernel_type,
            relative_cells,
        }
    }
}

/// Utility class for walking over a 3D voxel grid.
///
/// Contains the math for intersecting voxels with lines, polygons, areas, etc.
///
/// VoxelUtils.hpp:62-208
#[derive(Debug, Clone)]
pub struct VoxelUtils {
    /// Size of each voxel cell
    /// VoxelUtils.hpp:67
    pub cell_size: [Coord; 3],
}

impl VoxelUtils {
    /// Create a new VoxelUtils with the given cell size.
    ///
    /// VoxelUtils.hpp:69-72
    pub fn new(cell_size: [Coord; 3]) -> Self {
        Self { cell_size }
    }

    /// Convert a 3D point to a grid point.
    ///
    /// VoxelUtils.hpp:173-176
    pub fn to_grid_point(&self, point: [Coord; 3]) -> GridPoint3 {
        [
            self.to_grid_coord(point[0], 0),
            self.to_grid_coord(point[1], 1),
            self.to_grid_coord(point[2], 2),
        ]
    }

    /// Convert a coordinate to a grid coordinate.
    ///
    /// VoxelUtils.hpp:178-182
    pub fn to_grid_coord(&self, coord: Coord, dim: usize) -> GridCoord {
        coord / self.cell_size[dim] - if coord < 0 { 1 } else { 0 }
    }

    /// Convert a grid point to the lower corner of the voxel cell.
    ///
    /// VoxelUtils.hpp:184-187
    pub fn to_lower_corner(&self, location: GridPoint3) -> [Coord; 3] {
        [
            self.to_lower_coord(location[0], 0),
            self.to_lower_coord(location[1], 1),
            self.to_lower_coord(location[2], 2),
        ]
    }

    /// Convert a grid coordinate to the lower coordinate of the voxel cell.
    ///
    /// VoxelUtils.hpp:189-193
    pub fn to_lower_coord(&self, grid_coord: GridCoord, dim: usize) -> Coord {
        grid_coord * self.cell_size[dim]
    }

    /// Return a rectangular polygon equal to the cross section of a voxel cell.
    ///
    /// VoxelUtils.hpp:198-207
    pub fn to_polygon(&self, p: GridPoint3) -> Polygon {
        let c = self.to_lower_corner(p);
        Polygon::from_points(vec![
            Point::new(c[0], c[1]),
            Point::new(c[0] + self.cell_size[0], c[1]),
            Point::new(c[0] + self.cell_size[0], c[1] + self.cell_size[1]),
            Point::new(c[0], c[1] + self.cell_size[1]),
        ])
    }

    /// Walk a line segment and process each voxel cell it crosses.
    ///
    /// Returns false if the processing function returned false (early termination).
    ///
    /// VoxelUtils.hpp:82 / VoxelUtils.cpp:50-96
    pub fn walk_line<F>(&self, start: [Coord; 3], end: [Coord; 3], process: &F) -> bool
    where
        F: Fn(GridPoint3) -> bool,
    {
        let diff = [end[0] - start[0], end[1] - start[1], end[2] - start[2]];

        let start_cell = self.to_grid_point(start);
        let end_cell = self.to_grid_point(end);

        if start_cell == end_cell {
            return process(start_cell);
        }

        let mut current_cell = start_cell;

        loop {
            if !process(current_cell) {
                return false;
            }

            let mut stepping_dim: i32 = -1;
            let mut percentage_along_line = f64::MAX;

            for dim in 0..3 {
                if diff[dim] == 0 {
                    continue;
                }
                let crossing_boundary = self.to_lower_coord(current_cell[dim], dim)
                    + if diff[dim] > 0 {
                        self.cell_size[dim]
                    } else {
                        0
                    };
                let pct = (crossing_boundary - start[dim]) as f64 / diff[dim] as f64;
                if pct < percentage_along_line {
                    percentage_along_line = pct;
                    stepping_dim = dim as i32;
                }
            }

            if stepping_dim == -1 || percentage_along_line > 1.0 {
                return true;
            }

            let sd = stepping_dim as usize;
            current_cell[sd] += if diff[sd] > 0 { 1 } else { -1 };
        }
    }

    /// Walk the line segments of a polygon and process each crossed voxel.
    ///
    /// VoxelUtils.cpp:99-115
    pub fn walk_polygons<F>(&self, polys: &ExPolygon, z: Coord, process: &F) -> bool
    where
        F: Fn(GridPoint3) -> bool,
    {
        // Convert ExPolygon to flat list of polygons (contour + holes)
        let all_polygons = std::iter::once(&polys.contour).chain(polys.holes.iter());

        for poly in all_polygons {
            if poly.points.is_empty() {
                continue;
            }
            let mut last = *poly.points.last().unwrap();
            for &p in &poly.points {
                if !self.walk_line([last.x, last.y, z], [p.x, p.y, z], process) {
                    return false;
                }
                last = p;
            }
        }
        true
    }

    /// Walk polygons with dilation.
    ///
    /// VoxelUtils.cpp:117-130
    pub fn walk_dilated_polygons<F>(
        &self,
        polys: &ExPolygon,
        z: Coord,
        kernel: &DilationKernel,
        process: &F,
    ) -> bool
    where
        F: Fn(GridPoint3) -> bool,
    {
        let mut translated = polys.clone();
        let mut k = kernel.kernel_size;
        k[0] %= 2;
        k[1] %= 2;
        k[2] %= 2;

        let translation = [
            (1 - k[0]) * self.cell_size[0] / 2,
            (1 - k[1]) * self.cell_size[1] / 2,
            (1 - k[2]) * self.cell_size[2] / 2,
        ];

        if translation[0] != 0 && translation[1] != 0 {
            translated.translate(Point::new(translation[0], translation[1]));
        }

        let dilated = self.dilate(kernel, process);
        self.walk_polygons(&translated, z + translation[2], &dilated)
    }

    /// Walk dilated polygons for a collection of ExPolygons.
    ///
    /// VoxelUtils.hpp:108-117
    pub fn walk_dilated_polygons_multi<F>(
        &self,
        polys: &[ExPolygon],
        z: Coord,
        kernel: &DilationKernel,
        process: &F,
    ) -> bool
    where
        F: Fn(GridPoint3) -> bool,
    {
        for poly in polys {
            if !self.walk_dilated_polygons(poly, z, kernel, process) {
                return false;
            }
        }
        true
    }

    /// Walk all voxels inside an area.
    ///
    /// VoxelUtils.cpp:132-141
    pub fn walk_areas<F>(&self, polys: &ExPolygon, z: Coord, process: &F) -> bool
    where
        F: Fn(GridPoint3) -> bool,
    {
        let mut translated = polys.clone();
        let translation = [
            -self.cell_size[0] / 2,
            -self.cell_size[1] / 2,
            -self.cell_size[2] / 2,
        ];

        if translation[0] != 0 && translation[1] != 0 {
            translated.translate(Point::new(translation[0], translation[1]));
        }

        self.walk_areas_internal(&translated, z, process)
    }

    /// Walk dilated areas.
    ///
    /// VoxelUtils.cpp:190-204
    pub fn walk_dilated_areas<F>(
        &self,
        polys: &ExPolygon,
        z: Coord,
        kernel: &DilationKernel,
        process: &F,
    ) -> bool
    where
        F: Fn(GridPoint3) -> bool,
    {
        let mut translated = polys.clone();
        let mut k = kernel.kernel_size;
        k[0] %= 2;
        k[1] %= 2;
        k[2] %= 2;

        let translation = [
            (1 - k[0]) * self.cell_size[0] / 2 - self.cell_size[0] / 2,
            (1 - k[1]) * self.cell_size[1] / 2 - self.cell_size[1] / 2,
            (1 - k[2]) * self.cell_size[2] / 2 - self.cell_size[2] / 2,
        ];

        if translation[0] != 0 && translation[1] != 0 {
            translated.translate(Point::new(translation[0], translation[1]));
        }

        let dilated = self.dilate(kernel, process);
        self.walk_areas_internal(&translated, z + translation[2], &dilated)
    }

    /// Walk dilated areas for a collection of ExPolygons.
    ///
    /// VoxelUtils.hpp:150-159
    pub fn walk_dilated_areas_multi<F>(
        &self,
        polys: &[ExPolygon],
        z: Coord,
        kernel: &DilationKernel,
        process: &F,
    ) -> bool
    where
        F: Fn(GridPoint3) -> bool,
    {
        for poly in polys {
            if !self.walk_dilated_areas(poly, z, kernel, process) {
                return false;
            }
        }
        true
    }

    /// Create a dilated process function.
    ///
    /// For each cell, process all nearby cells according to the kernel.
    ///
    /// VoxelUtils.cpp:206-218
    pub fn dilate<'a, F>(
        &self,
        kernel: &'a DilationKernel,
        process: &'a F,
    ) -> impl Fn(GridPoint3) -> bool + 'a
    where
        F: Fn(GridPoint3) -> bool,
    {
        let relative_cells = &kernel.relative_cells;
        move |loc: GridPoint3| -> bool {
            for rel in relative_cells {
                let dilated = [loc[0] + rel[0], loc[1] + rel[1], loc[2] + rel[2]];
                if !process(dilated) {
                    return false;
                }
            }
            true
        }
    }

    /// Internal: walk areas using grid sampling.
    ///
    /// VoxelUtils.cpp:176-188
    fn walk_areas_internal<F>(&self, _polys: &ExPolygon, _z: Coord, _process: &F) -> bool
    where
        F: Fn(GridPoint3) -> bool,
    {
        // NOTE: Full implementation requires fill_surface (FillRectilinear) to generate
        // grid sample points inside the polygon. This is a complex dependency.
        // VoxelUtils.cpp:143-174 (spreadDotsArea)
        //
        // For now, return true (no cells processed) since this requires the fill algorithm.
        // The walk_polygons paths (boundary walking) still work correctly.
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dilation_kernel_cube() {
        let kernel = DilationKernel::new([3, 3, 3], DilationKernelType::Cube);
        assert_eq!(kernel.relative_cells.len(), 27); // 3^3
    }

    #[test]
    fn test_dilation_kernel_diamond() {
        let kernel = DilationKernel::new([3, 3, 3], DilationKernelType::Diamond);
        // Diamond kernel has fewer cells than cube
        assert!(kernel.relative_cells.len() < 27);
        assert!(!kernel.relative_cells.is_empty());
    }

    #[test]
    fn test_voxel_utils_grid_point() {
        let vu = VoxelUtils::new([100, 100, 100]);
        let gp = vu.to_grid_point([250, 350, 50]);
        assert_eq!(gp, [2, 3, 0]);
    }

    #[test]
    fn test_voxel_utils_grid_point_negative() {
        let vu = VoxelUtils::new([100, 100, 100]);
        let gp = vu.to_grid_point([-50, -150, 0]);
        assert_eq!(gp, [-1, -2, 0]);
    }

    #[test]
    fn test_voxel_utils_lower_corner() {
        let vu = VoxelUtils::new([100, 100, 100]);
        let corner = vu.to_lower_corner([2, 3, 1]);
        assert_eq!(corner, [200, 300, 100]);
    }

    #[test]
    fn test_voxel_utils_to_polygon() {
        let vu = VoxelUtils::new([100, 100, 100]);
        let poly = vu.to_polygon([0, 0, 0]);
        assert_eq!(poly.points.len(), 4);
        assert_eq!(poly.points[0], Point::new(0, 0));
        assert_eq!(poly.points[2], Point::new(100, 100));
    }

    #[test]
    fn test_walk_line_same_cell() {
        let vu = VoxelUtils::new([100, 100, 100]);
        let mut cells = Vec::new();
        vu.walk_line([10, 10, 10], [50, 50, 50], &|cell| {
            cells.push(cell);
            true
        });
        assert_eq!(cells.len(), 1);
        assert_eq!(cells[0], [0, 0, 0]);
    }

    #[test]
    fn test_walk_line_multiple_cells() {
        let vu = VoxelUtils::new([100, 100, 100]);
        let mut cells = Vec::new();
        vu.walk_line([0, 0, 0], [250, 0, 0], &|cell| {
            cells.push(cell);
            true
        });
        assert!(cells.len() >= 2);
    }

    #[test]
    fn test_walk_line_early_termination() {
        let vu = VoxelUtils::new([100, 100, 100]);
        let mut count = 0;
        let result = vu.walk_line([0, 0, 0], [500, 0, 0], &|_cell| {
            count += 1;
            count < 2 // stop after 2nd cell
        });
        assert!(!result);
        assert_eq!(count, 2);
    }

    #[test]
    fn test_dilate() {
        let vu = VoxelUtils::new([100, 100, 100]);
        let kernel = DilationKernel::new([3, 3, 3], DilationKernelType::Cube);
        let mut cells = Vec::new();
        let dilated = vu.dilate(&kernel, &|cell: GridPoint3| {
            cells.push(cell);
            true
        });
        // Apply dilated to a single cell
        dilated([5, 5, 5]);
        assert_eq!(cells.len(), 27); // 3^3 cells around [5,5,5]
    }
}
