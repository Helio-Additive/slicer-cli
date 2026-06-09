//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

//! 2D field that maintains locations which need to be supported for Lightning
//! Infill.
//!
//! C++ Reference:
//! - Fill/Lightning/DistanceField.hpp
//! - Fill/Lightning/DistanceField.cpp
//!
//! Faithful 1:1 line-by-line port of `Slic3r::FillLightning::DistanceField`.

use crate::clipper2_utils::{offset2_ex_2, union_ex_2};
use crate::fill::fill_rectilinear::sample_grid_pattern;
use crate::geometry::{get_extents_polygons, BoundingBox, Line, Point, Polygon};
use crate::Coord;

// DistanceField.cpp:18
// The cell-size should be small compared to the radius, but not so small as to be inefficient.
const RADIUS_PER_CELL_SIZE: Coord = 6;

// DistanceField.hpp:368-372 — Slic3r::PointHash
// struct PointHash {
//     size_t operator()(const Vec2crd &pt) const {
//         return coord_t((89 * 31 + int64_t(pt.x())) * 31 + pt.y());
//     }
// };
#[inline]
fn point_hash(pt: &Point) -> i64 {
    (89i64 * 31)
        .wrapping_add(pt.x())
        .wrapping_mul(31)
        .wrapping_add(pt.y())
}

/// Represents a small discrete area of infill that needs to be supported.
///
/// DistanceField.hpp:86-92
#[derive(Debug, Clone, Copy)]
pub struct UnsupportedCell {
    // DistanceField.hpp:89 — The position of the center of this cell.
    pub loc: Point,
    // DistanceField.hpp:91 — How far this cell is removed from the
    // ``current_outline`` polygon, the edge of the infill area.
    pub dist_to_boundary: Coord,
}

/// Links the unsupported points to a grid point, so that we can quickly look
/// up the cell belonging to a certain position in the grid.
///
/// DistanceField.hpp:110-184 — class UnsupportedPointsGrid
#[derive(Debug, Clone, Default)]
pub struct UnsupportedPointsGrid {
    // DistanceField.hpp:168
    m_size: usize,

    // DistanceField.hpp:170
    m_grid_range: BoundingBox,
    // DistanceField.hpp:171
    m_grid_size: Point,

    // DistanceField.hpp:173
    m_data: Vec<usize>,
    // DistanceField.hpp:174
    m_data_erased: Vec<bool>,
}

// DistanceField.hpp:140 / :144 — std::numeric_limits<size_t>::max()
const SIZE_T_MAX: usize = usize::MAX;

impl UnsupportedPointsGrid {
    // DistanceField.hpp:113 — UnsupportedPointsGrid() = default;
    pub fn new() -> Self {
        Self::default()
    }

    // DistanceField.hpp:114-136
    pub fn initialize<F>(&mut self, unsupported_points: &[UnsupportedCell], map_cell_to_grid: F)
    where
        F: Fn(&Point) -> Point,
    {
        // DistanceField.hpp:116-117
        if unsupported_points.is_empty() {
            return;
        }

        // DistanceField.hpp:119-121
        let mut unsupported_points_bbox = BoundingBox::new();
        for cell in unsupported_points {
            unsupported_points_bbox.merge_point(cell.loc);
        }

        // DistanceField.hpp:123
        self.m_size = unsupported_points.len();
        // DistanceField.hpp:124
        self.m_grid_range = BoundingBox::from_points_minmax(
            map_cell_to_grid(&unsupported_points_bbox.min),
            map_cell_to_grid(&unsupported_points_bbox.max),
        );
        // DistanceField.hpp:125 — m_grid_size = m_grid_range.size() + Point::Ones();
        self.m_grid_size = self.m_grid_range.size() + Point::new(1, 1);

        // DistanceField.hpp:127
        self.m_data = vec![SIZE_T_MAX; (self.m_grid_size.y() * self.m_grid_size.x()) as usize];
        // DistanceField.hpp:128
        self.m_data_erased = vec![true; (self.m_grid_size.y() * self.m_grid_size.x()) as usize];

        // DistanceField.hpp:130-135
        for (cell_idx, cell) in unsupported_points.iter().enumerate() {
            let flat_idx = self.map_to_flat_array(&map_cell_to_grid(&cell.loc));
            debug_assert!(self.m_data[flat_idx] == SIZE_T_MAX);
            self.m_data[flat_idx] = cell_idx;
            self.m_data_erased[flat_idx] = false;
        }
    }

    // DistanceField.hpp:138
    pub fn size(&self) -> usize {
        self.m_size
    }

    // DistanceField.hpp:140-151
    pub fn find_cell_idx(&self, grid_addr: &Point) -> usize {
        // DistanceField.hpp:142-143
        if !self.m_grid_range.contains_point(grid_addr) {
            return SIZE_T_MAX;
        }

        // DistanceField.hpp:145-148
        let flat_idx = self.map_to_flat_array(grid_addr);
        if !self.m_data_erased[flat_idx] {
            debug_assert!(self.m_data[flat_idx] != SIZE_T_MAX);
            return self.m_data[flat_idx];
        }

        // DistanceField.hpp:150
        SIZE_T_MAX
    }

    // DistanceField.hpp:153-165
    pub fn mark_erased(&mut self, grid_addr: &Point) {
        // DistanceField.hpp:155-157
        debug_assert!(self.m_grid_range.contains_point(grid_addr));
        if !self.m_grid_range.contains_point(grid_addr) {
            return;
        }

        // DistanceField.hpp:159
        let flat_idx = self.map_to_flat_array(grid_addr);
        // DistanceField.hpp:160-161
        debug_assert!(!self.m_data_erased[flat_idx] && self.m_data[flat_idx] != SIZE_T_MAX);
        debug_assert!(self.m_size != 0);

        // DistanceField.hpp:163-164
        self.m_data_erased[flat_idx] = true;
        self.m_size -= 1;
    }

    // DistanceField.hpp:176-183
    #[inline]
    fn map_to_flat_array(&self, loc: &Point) -> usize {
        // DistanceField.hpp:178
        let offset_loc = *loc - self.m_grid_range.min;
        // DistanceField.hpp:179
        let flat_idx = self.m_grid_size.x() * offset_loc.y() + offset_loc.x();
        // DistanceField.hpp:180
        debug_assert!(offset_loc.x() >= 0 && offset_loc.y() >= 0);
        // DistanceField.hpp:181
        debug_assert!((flat_idx as usize) < (self.m_grid_size.y() * self.m_grid_size.x()) as usize);
        flat_idx as usize
    }
}

/// 2D field that maintains locations which need to be supported for Lightning
/// Infill.
///
/// This field contains a set of "cells", spaced out in a grid. Each cell
/// maintains how far it is removed from the edge, which is used to determine
/// how it gets supported by Lightning Infill.
///
/// DistanceField.hpp:24-205 — class DistanceField
#[derive(Debug, Clone, Default)]
pub struct DistanceField {
    // DistanceField.hpp:74 — Spacing between grid points to consider supporting.
    m_cell_size: Coord,

    // DistanceField.hpp:80 — The radius of the area of the layer above supported
    // by a point on a branch of a tree.
    m_supporting_radius: Coord,
    // DistanceField.hpp:81
    m_supporting_radius2: i64,

    // DistanceField.hpp:97 — Cells which still need to be supported at some point.
    m_unsupported_points: Vec<UnsupportedCell>,
    // DistanceField.hpp:98
    m_unsupported_points_erased: Vec<bool>,

    // DistanceField.hpp:103 — BoundingBox of all points in m_unsupported_points.
    // Used for mapping of sign integer numbers to positive integer numbers.
    m_unsupported_points_bbox: BoundingBox,

    // DistanceField.hpp:186
    m_unsupported_points_grid: UnsupportedPointsGrid,
}

impl DistanceField {
    /// Construct a new field to calculate Lightning Infill with.
    /// \param radius The radius of influence that an infill line is expected to
    /// support in the layer above.
    /// \param current_outline The total infill area on this layer.
    /// \param current_overhang The overhang that needs to be supported on this
    /// layer.
    ///
    /// DistanceField.cpp:39-96
    pub fn new(
        radius: Coord,
        _current_outline: &[Polygon],
        current_outlines_bbox: &BoundingBox,
        current_overhang: &[Polygon],
    ) -> Self {
        // DistanceField.cpp:40-42 — member initializer list
        let mut field = DistanceField {
            m_cell_size: radius / RADIUS_PER_CELL_SIZE,
            m_supporting_radius: radius,
            m_unsupported_points_bbox: *current_outlines_bbox,
            m_supporting_radius2: 0,
            m_unsupported_points: Vec::new(),
            m_unsupported_points_erased: Vec::new(),
            m_unsupported_points_grid: UnsupportedPointsGrid::new(),
        };

        // DistanceField.cpp:44
        field.m_supporting_radius2 = (radius as i64) * (radius as i64);
        // DistanceField.cpp:45-46 — Sample source polygons with a regular grid sampling pattern.
        let overhang_bbox = get_extents_polygons(current_overhang);
        // DistanceField.cpp:47-48 — remove dangling lines which causes
        // sample_grid_pattern crash (fails the OUTER_LOW assertions)
        let expolys = offset2_ex_2(
            &union_ex_2(current_overhang),
            (-field.m_cell_size / 2) as f64,
            (field.m_cell_size / 2) as f64,
        );
        // DistanceField.cpp:49
        for expoly in &expolys {
            // DistanceField.cpp:50
            let sampled_points = sample_grid_pattern(expoly, field.m_cell_size, &overhang_bbox);
            // DistanceField.cpp:51
            let unsupported_points_prev_size = field.m_unsupported_points.len();
            // DistanceField.cpp:52
            field.m_unsupported_points.resize(
                unsupported_points_prev_size + sampled_points.len(),
                UnsupportedCell {
                    loc: Point::zero(),
                    dist_to_boundary: 0,
                },
            );

            // DistanceField.cpp:54-72 — tbb::parallel_for, faithfully evaluated
            // sequentially (each iteration writes a disjoint output index).
            for sp_idx in 0..sampled_points.len() {
                // DistanceField.cpp:56
                let sp = sampled_points[sp_idx];
                // DistanceField.cpp:57-58 — Find a squared distance to the source
                // expolygon boundary.
                let mut d2 = f64::MAX;
                // DistanceField.cpp:59
                for icontour in 0..=expoly.holes.len() {
                    // DistanceField.cpp:60
                    let contour: &Polygon = if icontour == 0 {
                        &expoly.contour
                    } else {
                        &expoly.holes[icontour - 1]
                    };
                    // DistanceField.cpp:61
                    if contour.points.len() > 2 {
                        // DistanceField.cpp:62
                        let mut prev = *contour.points.last().unwrap();
                        // DistanceField.cpp:63
                        for p2 in &contour.points {
                            // DistanceField.cpp:64
                            d2 = d2.min(Line::distance_to_squared(sp, prev, *p2));
                            // DistanceField.cpp:65
                            prev = *p2;
                        }
                    }
                }
                // DistanceField.cpp:69
                field.m_unsupported_points[unsupported_points_prev_size + sp_idx] = UnsupportedCell {
                    loc: sp,
                    dist_to_boundary: d2.sqrt() as Coord,
                };
                // DistanceField.cpp:70
                debug_assert!(field.m_unsupported_points_bbox.contains_point(&sp));
            }
        } // end of parallel_for

        // DistanceField.cpp:74-79
        field
            .m_unsupported_points
            .sort_by(|a: &UnsupportedCell, b: &UnsupportedCell| {
                // DistanceField.cpp:75
                const PRIME_FOR_HASH: i64 = 191;
                // DistanceField.cpp:76-78
                // PointHash returns size_t (unsigned); `% prime_for_hash` converts
                // the coord_t prime to size_t, so the modulo is unsigned.
                let less = if (b.dist_to_boundary - a.dist_to_boundary).abs() > radius {
                    a.dist_to_boundary < b.dist_to_boundary
                } else {
                    ((point_hash(&a.loc) as u64) % (PRIME_FOR_HASH as u64))
                        < ((point_hash(&b.loc) as u64) % (PRIME_FOR_HASH as u64))
                };
                if less {
                    std::cmp::Ordering::Less
                } else {
                    std::cmp::Ordering::Greater
                }
            });

        // DistanceField.cpp:81
        field
            .m_unsupported_points_erased
            .resize(field.m_unsupported_points.len(), false);
        // DistanceField.cpp:82
        field.m_unsupported_points_erased.fill(false);

        // DistanceField.cpp:84
        let bbox_min = field.m_unsupported_points_bbox.min;
        let cell_size = field.m_cell_size;
        field.m_unsupported_points_grid.initialize(
            &field.m_unsupported_points,
            // self.to_grid_point(p)
            |p: &Point| (*p - bbox_min) / cell_size,
        );

        // DistanceField.cpp:86-88 — Because the distance between two points is at
        // least one axis equal to m_cell_size, every cell in
        // m_unsupported_points_grid contains exactly one point.
        debug_assert!(field.m_unsupported_points.len() == field.m_unsupported_points_grid.size());

        field
    }

    /// Gets the next unsupported location to be supported by a new branch.
    /// \return ``true`` if successful, or ``false`` if there are no more points
    /// to consider.
    ///
    /// DistanceField.hpp:43-54
    pub fn try_get_next_point(
        &self,
        out_unsupported_location: &mut Point,
        out_unsupported_cell_idx: &mut usize,
        start_idx: usize,
    ) -> bool {
        // DistanceField.hpp:45
        for point_idx in start_idx..self.m_unsupported_points.len() {
            // DistanceField.hpp:46
            if !self.m_unsupported_points_erased[point_idx] {
                // DistanceField.hpp:47
                *out_unsupported_cell_idx = point_idx;
                // DistanceField.hpp:48
                *out_unsupported_location = self.m_unsupported_points[point_idx].loc;
                // DistanceField.hpp:49
                return true;
            }
        }

        // DistanceField.hpp:53
        false
    }

    /// Update the distance field with a newly added branch.
    ///
    /// The branch is a line extending from \p to_node to \p added_leaf . This
    /// function updates the grid cells so that the distance field knows how far
    /// off it is from being supported by the current pattern.
    /// \param to_node The node endpoint of the newly added branch.
    /// \param added_leaf The location of the leaf of the newly added branch,
    /// drawing a straight line to the node.
    ///
    /// DistanceField.cpp:98-153
    pub fn update(&mut self, to_node: &Point, added_leaf: &Point) {
        // DistanceField.cpp:100 — Vec2d v = (added_leaf - to_node).cast<double>();
        let v = ((added_leaf.x() - to_node.x()) as f64, (added_leaf.y() - to_node.y()) as f64);
        // DistanceField.cpp:101 — auto l2 = v.squaredNorm();
        let l2 = v.0 * v.0 + v.1 * v.1;
        // DistanceField.cpp:102 — Vec2d extent = Vec2d(-v.y(), v.x()) * m_supporting_radius / sqrt(l2);
        let extent = {
            let s = self.m_supporting_radius as f64 / l2.sqrt();
            (-v.1 * s, v.0 * s)
        };

        // DistanceField.cpp:104
        let mut grid: BoundingBox;
        {
            // DistanceField.cpp:106 — Point diagonal(m_supporting_radius, m_supporting_radius);
            let diagonal = Point::new(self.m_supporting_radius, self.m_supporting_radius);
            // DistanceField.cpp:107 — Point iextent(extent.cast<coord_t>());
            let iextent = Point::new(extent.0 as Coord, extent.1 as Coord);
            // DistanceField.cpp:108 — grid = BoundingBox(added_leaf - diagonal, added_leaf + diagonal);
            grid = BoundingBox::from_points_minmax(*added_leaf - diagonal, *added_leaf + diagonal);
            // DistanceField.cpp:109
            grid.merge_point(*to_node - iextent);
            // DistanceField.cpp:110
            grid.merge_point(*to_node + iextent);
            // DistanceField.cpp:111
            grid.merge_point(*added_leaf - iextent);
            // DistanceField.cpp:112
            grid.merge_point(*added_leaf + iextent);

            // DistanceField.cpp:114-118 — Clip grid by m_unsupported_points_bbox.
            // Mainly to ensure that grid.min is a non-negative value.
            grid.min.x = grid.min.x.max(self.m_unsupported_points_bbox.min.x);
            grid.min.y = grid.min.y.max(self.m_unsupported_points_bbox.min.y);
            grid.max.x = grid.max.x.min(self.m_unsupported_points_bbox.max.x);
            grid.max.y = grid.max.y.min(self.m_unsupported_points_bbox.max.y);

            // DistanceField.cpp:120
            grid.min = self.to_grid_point(&grid.min);
            // DistanceField.cpp:121
            grid.max = self.to_grid_point(&grid.max);
        }

        // DistanceField.cpp:124-125
        let mut grid_addr = Point::zero();
        let mut grid_loc;
        // DistanceField.cpp:126
        grid_addr.y = grid.min.y();
        while grid_addr.y() <= grid.max.y() {
            // DistanceField.cpp:127
            grid_addr.x = grid.min.x();
            while grid_addr.x() <= grid.max.x() {
                // DistanceField.cpp:128
                grid_loc = self.from_grid_point(&grid_addr);
                // DistanceField.cpp:129-130 — Test inside a circle at the new leaf.
                if {
                    let dx = (grid_loc.x() - added_leaf.x()) as i64;
                    let dy = (grid_loc.y() - added_leaf.y()) as i64;
                    dx * dx + dy * dy
                } > self.m_supporting_radius2
                {
                    // DistanceField.cpp:131-132 — Not inside a circle at the end of
                    // the new leaf. Test inside a rotated rectangle.
                    // DistanceField.cpp:133 — Vec2d vx = (grid_loc - to_node).cast<double>();
                    let vx = (
                        (grid_loc.x() - to_node.x()) as f64,
                        (grid_loc.y() - to_node.y()) as f64,
                    );
                    // DistanceField.cpp:134 — double d = v.dot(vx);
                    let mut d = v.0 * vx.0 + v.1 * vx.1;
                    // DistanceField.cpp:135
                    if d >= 0.0 && d <= l2 {
                        // DistanceField.cpp:136 — d = extent.dot(vx);
                        d = extent.0 * vx.0 + extent.1 * vx.1;
                        // DistanceField.cpp:137-139
                        if d < -1.0 || d > 1.0 {
                            // Not inside a rotated rectangle.
                            grid_addr.x += 1;
                            continue;
                        }
                    }
                }
                // DistanceField.cpp:142-143 — Inside a circle at the end of the new
                // leaf, or inside a rotated rectangle. Remove unsupported leafs at
                // this grid location.
                // DistanceField.cpp:144
                let cell_idx = self.m_unsupported_points_grid.find_cell_idx(&grid_addr);
                if cell_idx != SIZE_T_MAX {
                    // DistanceField.cpp:145
                    let cell = self.m_unsupported_points[cell_idx];
                    // DistanceField.cpp:146
                    let dx = (cell.loc.x() - added_leaf.x()) as i64;
                    let dy = (cell.loc.y() - added_leaf.y()) as i64;
                    if dx * dx + dy * dy <= self.m_supporting_radius2 {
                        // DistanceField.cpp:147
                        self.m_unsupported_points_erased[cell_idx] = true;
                        // DistanceField.cpp:148
                        self.m_unsupported_points_grid.mark_erased(&grid_addr);
                    }
                }
                grid_addr.x += 1;
            }
            grid_addr.y += 1;
        }
    }

    /// Maps the point to the grid coordinates.
    ///
    /// DistanceField.hpp:191-193
    #[inline]
    fn to_grid_point(&self, point: &Point) -> Point {
        (*point - self.m_unsupported_points_bbox.min) / self.m_cell_size
    }

    /// Maps the point to the grid coordinates.
    ///
    /// DistanceField.hpp:198-200
    #[inline]
    fn from_grid_point(&self, point: &Point) -> Point {
        *point * self.m_cell_size + self.m_unsupported_points_bbox.min
    }
}
