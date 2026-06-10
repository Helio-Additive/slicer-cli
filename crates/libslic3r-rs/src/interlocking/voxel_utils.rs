//! Faithful 1:1 port of BambuStudio `src/libslic3r/Interlocking/VoxelUtils.{hpp,cpp}`.
//!
//! // Copyright (c) 2022 Ultimaker B.V.
//! // CuraEngine is released under the terms of the AGPLv3 or higher.
//!
//! Utilities for walking over a 3D voxel grid: intersecting voxels with
//! lines, polygons and areas. Used by the interlocking generator.
//!
//! C++ Reference:
//! - Interlocking/VoxelUtils.hpp
//! - Interlocking/VoxelUtils.cpp

use crate::fill::fill_rectilinear::{fill_surface_by_lines, FillRectilinearParams};
use crate::geometry::{deg2rad, ExPolygon, Point, Polygon};
use crate::libslic3r::unscale;
use crate::Coord;

/// VoxelUtils.hpp:15 `using GridPoint3 = Vec3crd;`
pub type GridPoint3 = [Coord; 3];

/// VoxelUtils.hpp:65 `using grid_coord_t = coord_t;`
pub type GridCoord = Coord;

// VoxelUtils.hpp:44-49
// A cubic kernel checks all voxels in a cube around a reference voxel.
//  _____
// |\ ___\
// | |    |
//  \|____|
//
// A diamond kernel uses a manhattan distance to create a diamond shape around a reference voxel.
//  /|\
// /_|_\
// \ | /
//  \|/
//
// A prism kernel is diamond in XY, but extrudes straight in Z around a reference voxel.
//   / \
//  /   \
// |\   /|
// | \ / |
// |  |  |
//  \ | /
//   \|/
//
// (C++ nested enum `DilationKernel::Type`; flattened to `DilationKernelType`
// because Rust has no nested type declarations inside structs.)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DilationKernelType {
    /// VoxelUtils.hpp:46
    Cube,
    /// VoxelUtils.hpp:47
    Diamond,
    /// VoxelUtils.hpp:48
    Prism,
}

/// VoxelUtils.hpp:17-20
/// Class for holding the relative positiongs wrt a reference cell on which to perform a dilation.
#[derive(Debug, Clone)]
pub struct DilationKernel {
    /// VoxelUtils.hpp:50 `GridPoint3 kernel_size_;` //!< Size of the kernel in number of voxel cells
    pub kernel_size: GridPoint3,
    /// VoxelUtils.hpp:51 `Type type_;`
    pub kernel_type: DilationKernelType,
    /// VoxelUtils.hpp:52 `std::vector<GridPoint3> relative_cells_;`
    /// All offset positions relative to some reference cell which is to be dilated
    pub relative_cells: Vec<GridPoint3>,
}

impl DilationKernel {
    /// VoxelUtils.cpp:12-48
    /// `DilationKernel::DilationKernel(GridPoint3 kernel_size, DilationKernel::Type type)`
    pub fn new(kernel_size: GridPoint3, kernel_type: DilationKernelType) -> Self {
        // VoxelUtils.cpp:16
        // multiplier for division to avoid rounding and to avoid use of floating point numbers
        let mult: Coord = kernel_size[0] * kernel_size[1] * kernel_size[2];
        // VoxelUtils.cpp:17
        let mut relative_cells: Vec<GridPoint3> = Vec::with_capacity(mult as usize);
        // VoxelUtils.cpp:18
        let half_kernel = [kernel_size[0] / 2, kernel_size[1] / 2, kernel_size[2] / 2];

        // VoxelUtils.cpp:20
        let start = [-half_kernel[0], -half_kernel[1], -half_kernel[2]];
        // VoxelUtils.cpp:21
        let end = [
            kernel_size[0] - half_kernel[0],
            kernel_size[1] - half_kernel[1],
            kernel_size[2] - half_kernel[2],
        ];
        // VoxelUtils.cpp:22-26
        for x in start[0]..end[0] {
            for y in start[1]..end[1] {
                for z in start[2]..end[2] {
                    // VoxelUtils.cpp:28
                    let current: GridPoint3 = [x, y, z];
                    // VoxelUtils.cpp:29
                    if kernel_type != DilationKernelType::Cube {
                        // VoxelUtils.cpp:31
                        let mut limit: GridPoint3 = [
                            if x < 0 { start[0] } else { end[0] - 1 },
                            if y < 0 { start[1] } else { end[1] - 1 },
                            if z < 0 { start[2] } else { end[2] - 1 },
                        ];
                        // VoxelUtils.cpp:32-33
                        if limit[0] == 0 {
                            limit[0] = 1;
                        }
                        // VoxelUtils.cpp:34-35
                        if limit[1] == 0 {
                            limit[1] = 1;
                        }
                        // VoxelUtils.cpp:36-37
                        if limit[2] == 0 {
                            limit[2] = 1;
                        }
                        // VoxelUtils.cpp:38
                        // const GridPoint3 rel_dists = (mult * current).array() / limit.array();
                        let rel_dists: GridPoint3 = [
                            mult * current[0] / limit[0],
                            mult * current[1] / limit[1],
                            mult * current[2] / limit[2],
                        ];
                        // VoxelUtils.cpp:39-42
                        if (kernel_type == DilationKernelType::Diamond
                            && rel_dists[0] + rel_dists[1] + rel_dists[2] > mult)
                            || (kernel_type == DilationKernelType::Prism
                                && rel_dists[0] + rel_dists[1] > mult)
                        {
                            continue; // don't consider this cell
                        }
                    }
                    // VoxelUtils.cpp:44
                    relative_cells.push([x, y, z]);
                }
            }
        }

        // VoxelUtils.cpp:13-14 (member initializer list)
        Self {
            kernel_size,
            kernel_type,
            relative_cells,
        }
    }
}

/// VoxelUtils.hpp:57-62
/// Utility class for walking over a 3D voxel grid.
///
/// Contains the math for intersecting voxels with lines, polgons, areas, etc.
#[derive(Debug, Clone)]
pub struct VoxelUtils {
    /// VoxelUtils.hpp:67 `Vec3crd cell_size_;`
    pub cell_size: [Coord; 3],
}

impl VoxelUtils {
    /// VoxelUtils.hpp:69-72
    /// `VoxelUtils(Vec3crd cell_size) : cell_size_(cell_size) {}`
    pub fn new(cell_size: [Coord; 3]) -> Self {
        Self { cell_size }
    }

    /// VoxelUtils.cpp:50-96
    /// `bool VoxelUtils::walkLine(Vec3crd start, Vec3crd end, const std::function<bool(GridPoint3)>& process_cell_func) const`
    ///
    /// Process voxels which a line segment crosses. (VoxelUtils.hpp:74-82)
    /// Returns whether executing was stopped short as indicated by the
    /// `process_cell_func`.
    pub fn walk_line<F>(&self, start: [Coord; 3], end: [Coord; 3], process_cell_func: &F) -> bool
    where
        F: Fn(GridPoint3) -> bool,
    {
        // VoxelUtils.cpp:52
        let diff = [end[0] - start[0], end[1] - start[1], end[2] - start[2]];

        // VoxelUtils.cpp:54-55
        let start_cell = self.to_grid_point(start);
        let end_cell = self.to_grid_point(end);
        // VoxelUtils.cpp:56-59
        if start_cell == end_cell {
            return process_cell_func(start_cell);
        }

        // VoxelUtils.cpp:61
        let mut current_cell = start_cell;
        // VoxelUtils.cpp:62
        loop {
            // VoxelUtils.cpp:64
            let continue_ = process_cell_func(current_cell);

            // VoxelUtils.cpp:66-69
            if !continue_ {
                return false;
            }

            // VoxelUtils.cpp:71
            let mut stepping_dim: i32 = -1; // dimension in which the line next exits the current cell
            // VoxelUtils.cpp:72
            let mut percentage_along_line = f64::MAX;
            // VoxelUtils.cpp:73
            for dim in 0..3 {
                // VoxelUtils.cpp:75-78
                if diff[dim] == 0 {
                    continue;
                }
                // VoxelUtils.cpp:79
                // coord_t crossing_boundary = toLowerCoord(current_cell[dim], dim) + (diff[dim] > 0) * cell_size_[dim];
                let crossing_boundary = self.to_lower_coord(current_cell[dim], dim)
                    + if diff[dim] > 0 { self.cell_size[dim] } else { 0 };
                // VoxelUtils.cpp:80
                let percentage_along_line_here =
                    (crossing_boundary - start[dim]) as f64 / diff[dim] as f64;
                // VoxelUtils.cpp:81-85
                if percentage_along_line_here < percentage_along_line {
                    percentage_along_line = percentage_along_line_here;
                    stepping_dim = dim as i32;
                }
            }
            // VoxelUtils.cpp:87
            debug_assert!(stepping_dim != -1);
            // VoxelUtils.cpp:88-92
            if percentage_along_line > 1.0 {
                // next cell is beyond the end
                return true;
            }
            // VoxelUtils.cpp:93
            let stepping_dim = stepping_dim as usize;
            current_cell[stepping_dim] += if diff[stepping_dim] > 0 { 1 } else { -1 };
        }
        // VoxelUtils.cpp:95 `return true;` (unreachable in C++ as well)
    }

    /// VoxelUtils.cpp:99-115
    /// `bool VoxelUtils::walkPolygons(const ExPolygon& polys, coord_t z, const std::function<bool(GridPoint3)>& process_cell_func) const`
    ///
    /// Process voxels which the line segments of a polygon crosses.
    /// \warning Voxels may be processed multiple times! (VoxelUtils.hpp:84-94)
    pub fn walk_polygons<F>(&self, polys: &ExPolygon, z: Coord, process_cell_func: &F) -> bool
    where
        F: Fn(GridPoint3) -> bool,
    {
        // VoxelUtils.cpp:101
        // for (const Polygon& poly : to_polygons(polys))
        // (`to_polygons(const ExPolygon&)` yields contour followed by holes.)
        for poly in std::iter::once(&polys.contour).chain(polys.holes.iter()) {
            // VoxelUtils.cpp:103 `Point last = poly.back();`
            // (C++ `back()` on an empty polygon is UB; here it panics.)
            let mut last = *poly
                .points
                .last()
                .expect("VoxelUtils::walkPolygons: empty polygon");
            // VoxelUtils.cpp:104
            for &p in &poly.points {
                // VoxelUtils.cpp:106
                let continue_ =
                    self.walk_line([last.x, last.y, z], [p.x, p.y, z], process_cell_func);
                // VoxelUtils.cpp:107-110
                if !continue_ {
                    return false;
                }
                // VoxelUtils.cpp:111
                last = p;
            }
        }
        // VoxelUtils.cpp:114
        true
    }

    /// VoxelUtils.cpp:117-130
    /// `bool VoxelUtils::walkDilatedPolygons(const ExPolygon& polys, coord_t z, const DilationKernel& kernel, const std::function<bool(GridPoint3)>& process_cell_func) const`
    ///
    /// Process voxels near the line segments of a polygon. For each voxel the
    /// polygon crosses we process each of the offset voxels according to the
    /// kernel. (VoxelUtils.hpp:96-107)
    pub fn walk_dilated_polygons<F>(
        &self,
        polys: &ExPolygon,
        z: Coord,
        kernel: &DilationKernel,
        process_cell_func: &F,
    ) -> bool
    where
        F: Fn(GridPoint3) -> bool,
    {
        // VoxelUtils.cpp:119
        let mut translated = polys.clone();
        // VoxelUtils.cpp:120-123
        let mut k = kernel.kernel_size;
        k[0] %= 2;
        k[1] %= 2;
        k[2] %= 2;
        // VoxelUtils.cpp:124
        // const Vec3crd translation = (Vec3crd(1, 1, 1) - k).array() * cell_size_.array() / 2;
        let translation = [
            (1 - k[0]) * self.cell_size[0] / 2,
            (1 - k[1]) * self.cell_size[1] / 2,
            (1 - k[2]) * self.cell_size[2] / 2,
        ];
        // VoxelUtils.cpp:125-128
        if translation[0] != 0 && translation[1] != 0 {
            translated.translate(Point::new(translation[0], translation[1]));
        }
        // VoxelUtils.cpp:129
        let dilated = self.dilate(kernel, process_cell_func);
        self.walk_polygons(&translated, z + translation[2], &dilated)
    }

    /// VoxelUtils.hpp:108-117
    /// `bool walkDilatedPolygons(const ExPolygons& polys, ...)` (ExPolygons overload;
    /// Rust has no overloading, hence the `_multi` suffix.)
    pub fn walk_dilated_polygons_multi<F>(
        &self,
        polys: &[ExPolygon],
        z: Coord,
        kernel: &DilationKernel,
        process_cell_func: &F,
    ) -> bool
    where
        F: Fn(GridPoint3) -> bool,
    {
        // VoxelUtils.hpp:110-115
        for poly in polys {
            if !self.walk_dilated_polygons(poly, z, kernel, process_cell_func) {
                return false;
            }
        }

        // VoxelUtils.hpp:116
        true
    }

    /// VoxelUtils.cpp:132-141
    /// `bool VoxelUtils::walkAreas(const ExPolygon& polys, coord_t z, const std::function<bool(GridPoint3)>& process_cell_func) const`
    ///
    /// Process all voxels inside the area of a polygons object.
    /// \warning The voxels along the area are not processed. Thin areas might
    /// not process any voxels at all. (VoxelUtils.hpp:126-136)
    pub fn walk_areas<F>(&self, polys: &ExPolygon, z: Coord, process_cell_func: &F) -> bool
    where
        F: Fn(GridPoint3) -> bool,
    {
        // VoxelUtils.cpp:134
        let mut translated = polys.clone();
        // VoxelUtils.cpp:135
        // offset half a cell so that the dots of spreadDotsArea are centered on the middle of the cell isntead of the lower corners.
        let translation = [
            -self.cell_size[0] / 2,
            -self.cell_size[1] / 2,
            -self.cell_size[2] / 2,
        ];
        // VoxelUtils.cpp:136-139
        if translation[0] != 0 && translation[1] != 0 {
            translated.translate(Point::new(translation[0], translation[1]));
        }
        // VoxelUtils.cpp:140
        self._walk_areas(&translated, z, process_cell_func)
    }

    /// VoxelUtils.cpp:176-188
    /// `bool VoxelUtils::_walkAreas(const ExPolygon& polys, coord_t z, const std::function<bool(GridPoint3)>& process_cell_func) const`
    ///
    /// \warning the `polys` is assumed to be translated by half the cell_size
    /// in xy already (VoxelUtils.hpp:120-123)
    fn _walk_areas<F>(&self, polys: &ExPolygon, z: Coord, process_cell_func: &F) -> bool
    where
        F: Fn(GridPoint3) -> bool,
    {
        // VoxelUtils.cpp:178
        let skin_points = spread_dots_area(polys, Point::new(self.cell_size[0], self.cell_size[1]));
        // VoxelUtils.cpp:179
        for p in skin_points {
            // VoxelUtils.cpp:181
            let continue_ = process_cell_func(self.to_grid_point([
                p.x + self.cell_size[0] / 2,
                p.y + self.cell_size[1] / 2,
                z,
            ]));
            // VoxelUtils.cpp:182-185
            if !continue_ {
                return false;
            }
        }
        // VoxelUtils.cpp:187
        true
    }

    /// VoxelUtils.cpp:190-204
    /// `bool VoxelUtils::walkDilatedAreas(const ExPolygon& polys, coord_t z, const DilationKernel& kernel, const std::function<bool(GridPoint3)>& process_cell_func) const`
    ///
    /// Process all voxels inside the area of a polygons object. For each voxel
    /// inside the polygon we process each of the offset voxels according to
    /// the kernel. (VoxelUtils.hpp:138-149)
    pub fn walk_dilated_areas<F>(
        &self,
        polys: &ExPolygon,
        z: Coord,
        kernel: &DilationKernel,
        process_cell_func: &F,
    ) -> bool
    where
        F: Fn(GridPoint3) -> bool,
    {
        // VoxelUtils.cpp:192
        let mut translated = polys.clone();
        // VoxelUtils.cpp:193-196
        let mut k = kernel.kernel_size;
        k[0] %= 2;
        k[1] %= 2;
        k[2] %= 2;
        // VoxelUtils.cpp:197-198
        // const Vec3crd translation = (Vec3crd(1, 1, 1) - k).array() * cell_size_.array() / 2 // offset half a cell when using an even kernel
        //                            - cell_size_.array() / 2; // offset half a cell so that the dots of spreadDotsArea are centered on the middle of the cell isntead of the lower corners.
        let translation = [
            (1 - k[0]) * self.cell_size[0] / 2 - self.cell_size[0] / 2,
            (1 - k[1]) * self.cell_size[1] / 2 - self.cell_size[1] / 2,
            (1 - k[2]) * self.cell_size[2] / 2 - self.cell_size[2] / 2,
        ];
        // VoxelUtils.cpp:199-202
        if translation[0] != 0 && translation[1] != 0 {
            translated.translate(Point::new(translation[0], translation[1]));
        }
        // VoxelUtils.cpp:203
        let dilated = self.dilate(kernel, process_cell_func);
        self._walk_areas(&translated, z + translation[2], &dilated)
    }

    /// VoxelUtils.hpp:150-159
    /// `bool walkDilatedAreas(const ExPolygons& polys, ...)` (ExPolygons overload;
    /// Rust has no overloading, hence the `_multi` suffix.)
    pub fn walk_dilated_areas_multi<F>(
        &self,
        polys: &[ExPolygon],
        z: Coord,
        kernel: &DilationKernel,
        process_cell_func: &F,
    ) -> bool
    where
        F: Fn(GridPoint3) -> bool,
    {
        // VoxelUtils.hpp:152-157
        for poly in polys {
            if !self.walk_dilated_areas(poly, z, kernel, process_cell_func) {
                return false;
            }
        }

        // VoxelUtils.hpp:158
        true
    }

    /// VoxelUtils.cpp:206-218
    /// `std::function<bool(GridPoint3)> VoxelUtils::dilate(const DilationKernel& kernel, const std::function<bool(GridPoint3)>& process_cell_func) const`
    ///
    /// Dilate with a kernel. Extends the `process_cell_func`, so that for each
    /// cell we process nearby cells as well. (VoxelUtils.hpp:161-171)
    pub fn dilate<'a, F>(
        &self,
        kernel: &'a DilationKernel,
        process_cell_func: &'a F,
    ) -> impl Fn(GridPoint3) -> bool + 'a
    where
        F: Fn(GridPoint3) -> bool,
    {
        // VoxelUtils.cpp:208
        move |loc: GridPoint3| -> bool {
            // VoxelUtils.cpp:210
            for rel in &kernel.relative_cells {
                // VoxelUtils.cpp:212
                let continue_ = process_cell_func([loc[0] + rel[0], loc[1] + rel[1], loc[2] + rel[2]]);
                // VoxelUtils.cpp:213-214
                if !continue_ {
                    return false;
                }
            }
            // VoxelUtils.cpp:216
            true
        }
    }

    /// VoxelUtils.hpp:173-176
    /// `GridPoint3 toGridPoint(const Vec3crd& point) const`
    pub fn to_grid_point(&self, point: [Coord; 3]) -> GridPoint3 {
        // VoxelUtils.hpp:175
        [
            self.to_grid_coord(point[0], 0),
            self.to_grid_coord(point[1], 1),
            self.to_grid_coord(point[2], 2),
        ]
    }

    /// VoxelUtils.hpp:178-182
    /// `grid_coord_t toGridCoord(const coord_t& coord, const size_t dim) const`
    pub fn to_grid_coord(&self, coord: Coord, dim: usize) -> GridCoord {
        // VoxelUtils.hpp:180
        debug_assert!(dim < 3);
        // VoxelUtils.hpp:181 `return coord / cell_size_[dim] - (coord < 0);`
        coord / self.cell_size[dim] - if coord < 0 { 1 } else { 0 }
    }

    /// VoxelUtils.hpp:184-187
    /// `Vec3crd toLowerCorner(const GridPoint3& location) const`
    pub fn to_lower_corner(&self, location: GridPoint3) -> [Coord; 3] {
        // VoxelUtils.hpp:186
        [
            self.to_lower_coord(location[0], 0),
            self.to_lower_coord(location[1], 1),
            self.to_lower_coord(location[2], 2),
        ]
    }

    /// VoxelUtils.hpp:189-193
    /// `coord_t toLowerCoord(const grid_coord_t& grid_coord, const size_t dim) const`
    pub fn to_lower_coord(&self, grid_coord: GridCoord, dim: usize) -> Coord {
        // VoxelUtils.hpp:191
        debug_assert!(dim < 3);
        // VoxelUtils.hpp:192
        grid_coord * self.cell_size[dim]
    }

    /// VoxelUtils.hpp:195-207
    /// `Polygon toPolygon(const GridPoint3 p) const`
    ///
    /// Returns a rectangular polygon equal to the cross section of a voxel
    /// cell at coordinate `p`.
    pub fn to_polygon(&self, p: GridPoint3) -> Polygon {
        // VoxelUtils.hpp:200-206
        let c = self.to_lower_corner(p);
        Polygon::from_points(vec![
            Point::new(c[0], c[1]),
            Point::new(c[0] + self.cell_size[0], c[1]),
            Point::new(c[0] + self.cell_size[0], c[1] + self.cell_size[1]),
            Point::new(c[0], c[1] + self.cell_size[1]),
        ])
    }
}

/// VoxelUtils.cpp:143-174
/// `static Points spreadDotsArea(const ExPolygon& polygons, Point grid_size)`
fn spread_dots_area(polygons: &ExPolygon, grid_size: Point) -> Vec<Point> {
    // VoxelUtils.cpp:145-148
    //     std::unique_ptr<Fill> filler(Fill::new_from_type(ipAlignedRectilinear));
    //     filler->angle        = Geometry::deg2rad(90.f);
    //     filler->spacing      = unscaled(grid_size.x());
    //     filler->bounding_box = get_extents(polygons);
    //
    // The crate has no virtual `Fill` factory. `FillAlignedRectilinear` is a
    // `FillRectilinear` whose `_layer_angle()` returns 0 (FillRectilinear.hpp);
    // with the default `Fill::layer_id == size_t(-1)` and the default
    // `Surface::bridge_angle < 0`, `Fill::_infill_direction()`
    // (FillBase.cpp:199-241) yields exactly `filler->angle + float(M_PI/2.)`
    // and the bounding-box reference point, which is only consulted in the
    // non-full-infill branch of `fill_surface_by_lines`
    // (FillRectilinear.cpp:2861-2877) and is therefore unused here because
    // `params.density == 1` selects the full-infill branch.
    //
    // Replicate the C++ f32 angle arithmetic exactly:
    // Geometry.hpp:295 `deg2rad(90.f)` = float(PI) * 90.f / 180.f,
    // FillBase.cpp:239 `out_angle += float(M_PI/2.)`.
    let filler_angle: f32 = std::f32::consts::PI * 90.0_f32 / 180.0_f32;
    debug_assert_eq!(filler_angle as f64, deg2rad(90.0) as f32 as f64);
    let rotate_angle = (filler_angle + std::f64::consts::FRAC_PI_2 as f32) as f64;
    let spacing = unscale(grid_size.x);

    // VoxelUtils.cpp:150-152
    //     FillParams params;
    //     params.density = 1.f;
    //     params.anchor_length_max = 0;
    // `anchor_length_max = 0` makes `FillParams::dont_connect()` true
    // (FillBase.hpp:55 `anchor_length_max < 0.05f`), and `density == 1` makes
    // `FillParams::full_infill()` true.
    let params = FillRectilinearParams {
        density: 1.0,
        monotonic: false,
        dont_connect: true,
        link_max_length: 0,
        full_infill: true,
        dont_adjust: false,
        consistent_pattern: false,
    };

    // VoxelUtils.cpp:154-155
    //     Surface surface(stInternal, polygons);
    //     auto    polylines = filler->fill_surface(&surface, params);
    // `FillRectilinear::fill_surface` (FillRectilinear.cpp:3074-3088) forwards
    // a full-infill request to `fill_surface_by_lines(surface, params, 0.f,
    // 0.f, polylines_out)`; the Surface wrapper only carries the expolygon
    // here (stInternal is not solid, bridge_angle/thickness_layers defaults).
    let polylines = fill_surface_by_lines(polygons, spacing, rotate_angle, 0.0, &params);

    // VoxelUtils.cpp:157
    let mut result: Vec<Point> = Vec::new();
    // VoxelUtils.cpp:158
    for line in &polylines {
        // VoxelUtils.cpp:159
        debug_assert!(line.points.len() == 2);
        // VoxelUtils.cpp:160-161
        let mut a = line.points[0];
        let mut b = line.points[1];
        // VoxelUtils.cpp:162
        debug_assert!(a.x == b.x);
        // VoxelUtils.cpp:163-165
        if a.y > b.y {
            std::mem::swap(&mut a, &mut b);
        }
        // VoxelUtils.cpp:166
        let mut y = a.y - (a.y % grid_size.y) - grid_size.y;
        while y < b.y {
            // VoxelUtils.cpp:167-168
            if y < a.y {
                y += grid_size.y;
                continue;
            }
            // VoxelUtils.cpp:169
            result.push(Point::new(a.x, y));
            y += grid_size.y;
        }
    }

    // VoxelUtils.cpp:173
    result
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
        let cells_ref = std::cell::RefCell::new(&mut cells);
        vu.walk_line([10, 10, 10], [50, 50, 50], &|cell| {
            cells_ref.borrow_mut().push(cell);
            true
        });
        assert_eq!(cells.len(), 1);
        assert_eq!(cells[0], [0, 0, 0]);
    }

    #[test]
    fn test_walk_line_multiple_cells() {
        let vu = VoxelUtils::new([100, 100, 100]);
        let cells = std::cell::RefCell::new(Vec::new());
        vu.walk_line([0, 0, 0], [250, 0, 0], &|cell| {
            cells.borrow_mut().push(cell);
            true
        });
        assert!(cells.borrow().len() >= 2);
    }

    #[test]
    fn test_walk_line_early_termination() {
        let vu = VoxelUtils::new([100, 100, 100]);
        let count = std::cell::Cell::new(0);
        let result = vu.walk_line([0, 0, 0], [500, 0, 0], &|_cell| {
            count.set(count.get() + 1);
            count.get() < 2 // stop after 2nd cell
        });
        assert!(!result);
        assert_eq!(count.get(), 2);
    }

    #[test]
    fn test_dilate() {
        let vu = VoxelUtils::new([100, 100, 100]);
        let kernel = DilationKernel::new([3, 3, 3], DilationKernelType::Cube);
        let cells = std::cell::RefCell::new(Vec::new());
        let process = |cell: GridPoint3| {
            cells.borrow_mut().push(cell);
            true
        };
        let dilated = vu.dilate(&kernel, &process);
        // Apply dilated to a single cell
        dilated([5, 5, 5]);
        assert_eq!(cells.borrow().len(), 27); // 3^3 cells around [5,5,5]
    }

    #[test]
    fn test_walk_areas_processes_interior_cells() {
        // A 10x10-cell square area must yield interior dot cells via
        // spreadDotsArea (VoxelUtils.cpp:143-188).
        let cell = 100_000; // 0.1 mm cells
        let vu = VoxelUtils::new([cell, cell, 1]);
        let square = ExPolygon::new(Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(10 * cell, 0),
            Point::new(10 * cell, 10 * cell),
            Point::new(0, 10 * cell),
        ]));
        let cells = std::cell::RefCell::new(std::collections::HashSet::new());
        let ok = vu.walk_areas(&square, 0, &|p: GridPoint3| {
            cells.borrow_mut().insert(p);
            true
        });
        assert!(ok);
        assert!(
            !cells.borrow().is_empty(),
            "walk_areas must process interior voxels"
        );
        for p in cells.borrow().iter() {
            assert_eq!(p[2], 0);
        }
    }
}
