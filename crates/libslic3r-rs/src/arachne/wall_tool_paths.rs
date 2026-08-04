// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.
//
//! WallToolPaths - generates variable-width wall toolpaths using the Arachne algorithm.
//!
//! C++ Reference:
//! - Arachne/WallToolPaths.cpp
//! - Arachne/WallToolPaths.hpp
//!
//! 1:1 line-by-line port. `coord_t` -> `i64` (`Coord`), `coordf_t` -> `f64` (`CoordF`).
//!
//! BLOCKED symbols (see module notes at the bottom for details):
//! - `WallToolPaths::generate` central call `wall_maker.generateToolpaths(toolpaths)` —
//!   `SkeletalTrapezoidation` is not yet a working port (it has no `generate_toolpaths`).
//! - `WallToolPaths::stitch_tool_paths` PolylineStitcher call — the
//!   `PolylineStitcher<VariableWidthLines, ExtrusionLine, ExtrusionJunction>` instantiation
//!   is not yet available in the Rust port.

use crate::arachne::beading_strategy::beading_strategy::{
    FIRST_WALL_CONTOUR_MARKED_WIDTH, WALL_CONTOUR_MARKED_WIDTH,
};
use crate::arachne::beading_strategy::beading_strategy_factory::BeadingStrategyFactory;
use crate::arachne::utils::extrusion_junction::ExtrusionJunction;
use crate::arachne::utils::extrusion_line::{ExtrusionLine, VariableWidthLines};
use crate::arachne::utils::linear_alg2d;
use crate::arachne::utils::polyline_stitcher::PolylineStitcher;
use crate::arachne::utils::sparse_line_grid::{LineLocatorTrait, SparseLineGrid};
use crate::arachne::utils::sparse_point_grid::{LocatorTrait, SparsePointGrid};
use crate::clipper_utils;
use crate::flow::Flow;
use crate::geometry::{
    area_polygons as geom_area_polygons, deg2rad, shorter_then, ExPolygon, Line, Point, Polygon,
    Polygons,
};
use crate::{scaled, unscale, unscaled, Coord, CoordF};

// ===========================================================================
// Constants — WallToolPaths.hpp:18-21
// ===========================================================================

// WallToolPaths.hpp:18  constexpr bool fill_outline_gaps = true;
pub const FILL_OUTLINE_GAPS: bool = true;

// WallToolPaths.hpp:19  constexpr coord_t meshfix_maximum_resolution = scaled<coord_t>(0.5);
pub const MESHFIX_MAXIMUM_RESOLUTION: Coord = scaled_c(0.5);

// WallToolPaths.hpp:20  constexpr coord_t meshfix_maximum_deviation = scaled<coord_t>(0.025);
pub const MESHFIX_MAXIMUM_DEVIATION: Coord = scaled_c(0.025);

// WallToolPaths.hpp:21  constexpr coord_t meshfix_maximum_extrusion_area_deviation = scaled<coord_t>(2.);
pub const MESHFIX_MAXIMUM_EXTRUSION_AREA_DEVIATION: Coord = scaled_c(2.0);

/// `const`-context `scaled<coord_t>(mm)` — multiply by `SCALING_FACTOR` (100_000) and round.
/// Used for the `constexpr` constants above. `crate::scaled` is not `const fn`.
const fn scaled_c(mm: f64) -> Coord {
    // crate::SCALING_FACTOR == 100_000.0; round to nearest.
    (mm * crate::SCALING_FACTOR + 0.5) as Coord
}

/// `Slic3r::sqr(x)` — square helper.
#[inline]
fn sqr_i64(x: i64) -> i64 {
    x * x
}

#[inline]
fn sqr_f64(x: f64) -> f64 {
    x * x
}

/// WallToolPaths.hpp:23-32  class WallToolPathsParams
#[derive(Debug, Clone, Copy)]
pub struct WallToolPathsParams {
    // WallToolPaths.hpp:26
    pub min_bead_width: f32,
    // WallToolPaths.hpp:27
    pub min_feature_size: f32,
    // WallToolPaths.hpp:28
    pub wall_transition_length: f32,
    // WallToolPaths.hpp:29
    pub wall_transition_angle: f32,
    // WallToolPaths.hpp:30
    pub wall_transition_filter_deviation: f32,
    // WallToolPaths.hpp:31
    pub wall_distribution_count: i32,
}

impl Default for WallToolPathsParams {
    fn default() -> Self {
        Self {
            min_bead_width: 0.34,
            min_feature_size: 0.1,
            wall_transition_length: 0.4,
            // R549: this field holds DEGREES -- `deg2rad` is applied to it at
            // WallToolPaths.cpp:456 (mirrored below). The old default stored
            // 0.174533 (10 degrees already in radians), so the conversion ran
            // twice and yielded 0.0030462 rad, making `cap = sin(angle/2)` 57.3x
            // too small in `updateIsCentral`. C++ has no default: every C++ site
            // assigns degrees (PerimeterGenerator.cpp:1551 passes
            // `object_config->wall_transition_angle.value`, FillConcentric.cpp:89
            // assigns literal 10).
            wall_transition_angle: if crate::faithful_gate("ARACHNE_WTP_ANGLE_DEG") {
                10.0
            } else {
                0.174533
            },
            wall_transition_filter_deviation: 0.025,
            wall_distribution_count: 1,
        }
    }
}

/// WallToolPaths.hpp:34-148  class WallToolPaths
pub struct WallToolPaths {
    // WallToolPaths.hpp:129  const Polygons& outline;
    outline: Polygons,
    // WallToolPaths.hpp:130  coord_t bead_width_0;
    bead_width_0: Coord,
    // WallToolPaths.hpp:131  coord_t bead_width_x;
    bead_width_x: Coord,
    // WallToolPaths.hpp:132  size_t inset_count;
    inset_count: usize,
    // WallToolPaths.hpp:133  coord_t wall_0_inset;
    wall_0_inset: Coord,
    // WallToolPaths.hpp:134  coordf_t layer_height;
    layer_height: CoordF,
    // WallToolPaths.hpp:135  bool print_thin_walls;
    print_thin_walls: bool,
    // WallToolPaths.hpp:136  coord_t min_feature_size;
    min_feature_size: Coord,
    // WallToolPaths.hpp:137  coord_t min_bead_width;
    min_bead_width: Coord,
    // WallToolPaths.hpp:138  double small_area_length;
    small_area_length: f64,
    // WallToolPaths.hpp:139  coord_t wall_transition_filter_deviation;
    wall_transition_filter_deviation: Coord,
    // WallToolPaths.hpp:140  bool toolpaths_generated;
    toolpaths_generated: bool,
    // WallToolPaths.hpp:141  std::vector<VariableWidthLines> toolpaths;
    toolpaths: Vec<VariableWidthLines>,
    // WallToolPaths.hpp:142  Polygons inner_contour;
    inner_contour: Polygons,
    // WallToolPaths.hpp:143  Polygons first_wall_contour;
    first_wall_contour: Polygons,
    // WallToolPaths.hpp:144  const WallToolPathsParams m_params;
    m_params: WallToolPathsParams,
    // WallToolPaths.hpp:146  bool enable_hole_compensation{ false };
    enable_hole_compensation: bool,
    // WallToolPaths.hpp:147  std::vector<int> hole_indices;
    hole_indices: Vec<i32>,
}

// ===========================================================================
// Free functions — WallToolPaths.cpp:50-439
// ===========================================================================

/// WallToolPaths.cpp:50  void simplify(Polygon &thiss, const int64_t smallest_line_segment_squared,
///                                      const int64_t allowed_error_distance_squared)
pub fn simplify_polygon(
    thiss: &mut Polygon,
    smallest_line_segment_squared: i64,
    allowed_error_distance_squared: i64,
) {
    // WallToolPaths.cpp:52-55
    if thiss.len() < 3 {
        thiss.points.clear();
        return;
    }
    // WallToolPaths.cpp:56-57
    if thiss.len() == 3 {
        return;
    }

    // WallToolPaths.cpp:59
    let mut new_path = Polygon::new();
    // WallToolPaths.cpp:60
    let mut previous = *thiss.points.last().unwrap();
    // WallToolPaths.cpp:61
    let mut previous_previous = thiss.points[thiss.points.len() - 2];
    // WallToolPaths.cpp:62
    let mut current = thiss.points[0];

    /* When removing a vertex, we check the height of the triangle of the area
     being removed from the original polygon by the simplification. However,
     when consecutively removing multiple vertices the height of the previously
     removed vertices w.r.t. the shortcut path changes.
     In order to not recompute the new height value of previously removed
     vertices we compute the height of a representative triangle, which covers
     the same amount of area as the area being cut off. We use the Shoelace
     formula to accumulate the area under the removed segments. This works by
     computing the area in a 'fan' where each of the blades of the fan go from
     the origin to one of the segments. While removing vertices the area in
     this fan accumulates. By subtracting the area of the blade connected to
     the short-cutting segment we obtain the total area of the cutoff region.
     From this area we compute the height of the representative triangle using
     the standard formula for a triangle area: A = .5*b*h
     */
    // WallToolPaths.cpp:79  Twice the Shoelace formula for area of polygon per line segment.
    let mut accumulated_area_removed: i64 = (previous.x as i64) * (current.y as i64)
        - (previous.y as i64) * (current.x as i64);

    // WallToolPaths.cpp:81
    for point_idx in 0..thiss.points.len() {
        // WallToolPaths.cpp:82
        current = thiss.points[point_idx % thiss.points.len()];

        // Check if the accumulated area doesn't exceed the maximum.
        // WallToolPaths.cpp:85-92
        let next: Point;
        if point_idx + 1 < thiss.points.len() {
            next = thiss.points[point_idx + 1];
        } else if point_idx + 1 == thiss.points.len() && new_path.len() > 1 {
            // don't spill over if the [next] vertex will then be equal to [previous]
            next = new_path[0]; // Spill over to new polygon for checking removed area.
        } else {
            next = thiss.points[(point_idx + 1) % thiss.points.len()];
        }
        // WallToolPaths.cpp:93  Twice the Shoelace formula for area of polygon per line segment.
        let removed_area_next: i64 =
            (current.x as i64) * (next.y as i64) - (current.y as i64) * (next.x as i64);
        // WallToolPaths.cpp:94  area between the origin and the short-cutting segment
        let negative_area_closing: i64 =
            (next.x as i64) * (previous.y as i64) - (next.y as i64) * (previous.x as i64);
        accumulated_area_removed += removed_area_next;

        // WallToolPaths.cpp:97
        let length2: i64 = (current - previous).length_squared() as i64;
        if length2 < scaled(25.0) {
            // We're allowed to always delete segments of less than 5 micron.
            // WallToolPaths.cpp:99-100
            continue;
        }

        // WallToolPaths.cpp:103  close the shortcut area polygon
        let area_removed_so_far: i64 = accumulated_area_removed + negative_area_closing;
        // WallToolPaths.cpp:104
        let base_length_2: i64 = (next - previous).length_squared() as i64;

        // WallToolPaths.cpp:106-107  Two line segments form a line back and forth with no area.
        if base_length_2 == 0 {
            continue; // Remove the vertex.
        }
        // We want to check if the height of the triangle formed by previous, current and next vertices
        // is less than allowed_error_distance_squared.
        //1/2 L = A           [actual area is half of the computed shoelace value]
        //A = 1/2 * b * h     [triangle area formula]
        //L = b * h           [apply above two and take out the 1/2]
        //h = L / b           [divide by b]
        //h^2 = (L / b)^2     [square it]
        //h^2 = L^2 / b^2     [factor the divisor]
        // WallToolPaths.cpp:115
        let height_2: i64 = (area_removed_so_far as f64 * area_removed_so_far as f64
            / base_length_2 as f64) as i64;
        // WallToolPaths.cpp:116-118
        // scaled<double>(0.005) == 0.005 * SCALING_FACTOR (100_000) == 500.0
        if height_2 <= sqr_i64(scaled(0.005)) // Almost exactly colinear (barring rounding errors).
            && Line::distance_to_infinite(current, previous, next) <= 0.005 * crate::SCALING_FACTOR
        {
            // make sure that height_2 is not small because of cancellation of positive and negative areas
            continue;
        }

        // WallToolPaths.cpp:120-122
        if length2 < smallest_line_segment_squared && height_2 <= allowed_error_distance_squared
        // removing the vertex doesn't introduce too much error.
        {
            // WallToolPaths.cpp:123
            let next_length2: i64 = (current - next).length_squared() as i64;
            if next_length2 > 4 * smallest_line_segment_squared {
                // Special case; The next line is long. If we were to remove this, it could happen that we get quite noticeable artifacts.
                // We should instead move this point to a location where both edges are kept and then remove the previous point that we wanted to keep.
                // By taking the intersection of these two lines, we get a point that preserves the direction (so it makes the corner a bit more pointy).
                // We just need to be sure that the intersection point does not introduce an artifact itself.
                // WallToolPaths.cpp:130
                let intersection =
                    Line::new(previous_previous, previous).intersection_infinite(&Line::new(current, next));
                // WallToolPaths.cpp:131-138
                match intersection {
                    Some(intersection_point)
                        if Line::distance_to_infinite_squared(intersection_point, previous, current)
                            <= allowed_error_distance_squared as f64
                            && (intersection_point - previous).length_squared() as i64
                                <= smallest_line_segment_squared
                            && (intersection_point - next).length_squared() as i64
                                <= smallest_line_segment_squared =>
                    {
                        // New point seems like a valid one.
                        // WallToolPaths.cpp:140-141
                        current = intersection_point;
                        // If there was a previous point added, remove it.
                        // WallToolPaths.cpp:143-146
                        if !new_path.is_empty() {
                            new_path.points.pop();
                            previous = previous_previous;
                        }
                    }
                    _ => {
                        // We can't find a better spot for it, but the size of the line is more than 5 micron.
                        // So the only thing we can do here is leave it in...
                        // WallToolPaths.cpp:135-138
                    }
                }
            } else {
                // WallToolPaths.cpp:148-149
                continue; // Remove the vertex.
            }
        }
        // Don't remove the vertex.
        // WallToolPaths.cpp:152-156
        accumulated_area_removed = removed_area_next; // so that in the next iteration it's the area between the origin, [previous] and [current]
        previous_previous = previous;
        previous = current; // Note that "previous" is only updated if we don't remove the vertex.
        new_path.points.push(current);
    }

    // WallToolPaths.cpp:159
    *thiss = new_path;
}

/// WallToolPaths.cpp:191  void simplify(Polygons &thiss, smallest_line_segment, allowed_error_distance)
pub fn simplify_polygons(thiss: &mut Polygons, smallest_line_segment: i64, allowed_error_distance: i64) {
    // WallToolPaths.cpp:193
    let allowed_error_distance_squared: i64 =
        (allowed_error_distance as i64) * (allowed_error_distance as i64);
    // WallToolPaths.cpp:194
    let smallest_line_segment_squared: i64 =
        (smallest_line_segment as i64) * (smallest_line_segment as i64);
    // WallToolPaths.cpp:195
    let mut p = 0usize;
    while p < thiss.len() {
        // WallToolPaths.cpp:197
        simplify_polygon(
            &mut thiss[p],
            smallest_line_segment_squared,
            allowed_error_distance_squared,
        );
        // WallToolPaths.cpp:198-202
        if thiss[p].len() < 3 {
            thiss.remove(p);
            // p-- then loop p++ -> stay at same index. Emulate with `continue` w/o increment.
            continue;
        }
        p += 1;
    }
}

/// `LocToLineGrid` element. C++ stores a `PolygonsPointIndex` (which itself is a `(polygons*, poly_idx, point_idx)`).
/// Because the Rust `PolygonsPointIndex` carries a borrow of the source `Polygons`, and
/// `fixSelfIntersections` mutates the source while querying the grid, we store the indices and
/// resolve the segment against the live polygons via the locator. This is numerically identical to
/// the C++ behaviour: the grid cells are computed once (before any mutation) from the original
/// vertex positions, and the segment endpoints used by `getNearby` are read from the live array.
/// WallToolPaths.cpp:206
#[derive(Debug, Clone, Copy)]
pub struct PolyPointLoc {
    pub poly_idx: usize,
    pub point_idx: usize,
    // Cached vertex position at insertion time, used by the line-grid locator.
    pub p: Point,
    // Cached next vertex position at insertion time, used by the line-grid locator.
    pub p_next: Point,
}

/// Locator returning the line segment for a `PolyPointLoc`, mirroring
/// `PolygonsPointIndexSegmentLocator` (WallToolPaths.cpp:206 type alias).
#[derive(Debug, Clone, Copy, Default)]
pub struct PolyPointLocSegmentLocator;

impl LineLocatorTrait<PolyPointLoc> for PolyPointLocSegmentLocator {
    fn locate(&self, elem: &PolyPointLoc) -> (Point, Point) {
        (elem.p, elem.p_next)
    }
}

pub type LocToLineGrid = SparseLineGrid<PolyPointLoc, PolyPointLocSegmentLocator>;

/// WallToolPaths.cpp:207  std::unique_ptr<LocToLineGrid> createLocToLineGrid(const Polygons &polygons, int square_size)
pub fn create_loc_to_line_grid(polygons: &Polygons, square_size: i64) -> LocToLineGrid {
    // WallToolPaths.cpp:209-211
    let mut n_points: usize = 0;
    for poly in polygons.iter() {
        n_points += poly.len();
    }

    // WallToolPaths.cpp:213
    let mut ret: LocToLineGrid = SparseLineGrid::new(square_size, n_points, 1.0);

    // WallToolPaths.cpp:215-217
    for poly_idx in 0..polygons.len() {
        for point_idx in 0..polygons[poly_idx].len() {
            let poly = &polygons[poly_idx];
            let next_point_idx = (point_idx + 1) % poly.len();
            ret.insert(PolyPointLoc {
                poly_idx,
                point_idx,
                p: poly[point_idx],
                p_next: poly[next_point_idx],
            });
        }
    }
    ret
}

/* Note: Also tries to solve for near-self intersections, when epsilon >= 1
 */
/// WallToolPaths.cpp:223  void fixSelfIntersections(const coord_t epsilon, Polygons &thiss)
pub fn fix_self_intersections(epsilon: Coord, thiss: &mut Polygons) {
    // WallToolPaths.cpp:225-228
    if epsilon < 1 {
        // ClipperLib::SimplifyPolygons(ClipperUtils::PolygonsProvider(thiss));
        *thiss = clipper_simplify_polygons(thiss);
        return;
    }

    // WallToolPaths.cpp:230
    let half_epsilon: i64 = (epsilon + 1) / 2;

    // Points too close to line segments should be moved a little away from those line segments,
    // but less than epsilon, so at least half-epsilon distance between points can still be guaranteed.
    // WallToolPaths.cpp:234
    const GRID_SIZE: Coord = scaled_c(2.0);
    // WallToolPaths.cpp:235
    let query_grid = create_loc_to_line_grid(thiss, GRID_SIZE);

    // WallToolPaths.cpp:237
    let move_dist: i64 = std::cmp::max(2i64, half_epsilon - 2);
    // WallToolPaths.cpp:238
    let half_epsilon_sqrd: i64 = half_epsilon * half_epsilon;

    // WallToolPaths.cpp:240
    let n = thiss.len();
    // WallToolPaths.cpp:241
    for poly_idx in 0..n {
        // WallToolPaths.cpp:242
        let pathlen = thiss[poly_idx].len();
        // WallToolPaths.cpp:243
        for point_idx in 0..pathlen {
            // WallToolPaths.cpp:244  Point &pt = thiss[poly_idx][point_idx];
            let pt = thiss[poly_idx][point_idx];
            // WallToolPaths.cpp:245  for (const auto &line : query_grid->getNearby(pt, epsilon))
            let nearby = query_grid.get_nearby(pt, epsilon);
            for line in nearby.iter() {
                // WallToolPaths.cpp:246
                let line_next_idx = (line.point_idx + 1) % thiss[line.poly_idx].len();
                // WallToolPaths.cpp:247-248
                if poly_idx == line.poly_idx && (point_idx == line.point_idx || point_idx == line_next_idx) {
                    continue;
                }

                // WallToolPaths.cpp:250  const Line segment(thiss[line.poly_idx][line.point_idx], thiss[line.poly_idx][line_next_idx]);
                let seg_a = thiss[line.poly_idx][line.point_idx];
                let seg_b = thiss[line.poly_idx][line_next_idx];
                // WallToolPaths.cpp:251-252  segment.distance_to_squared(pt, &segment_closest_point);
                let segment_closest_point = pt.project_onto_segment(seg_a, seg_b);

                // WallToolPaths.cpp:254
                if half_epsilon_sqrd as i128 >= (pt - segment_closest_point).length_squared() {
                    // WallToolPaths.cpp:255  const Point &other = thiss[poly_idx][(point_idx + 1) % pathlen];
                    let other = thiss[poly_idx][(point_idx + 1) % pathlen];
                    // WallToolPaths.cpp:256
                    let vec = if linear_alg2d::point_is_left_of_line(other, seg_a, seg_b) > 0 {
                        seg_b - seg_a
                    } else {
                        seg_a - seg_b
                    };
                    // WallToolPaths.cpp:257-258  asserts on overflow (no-op in release)
                    debug_assert!(sqr_f64(vec.x as f64) < i64::MAX as f64);
                    debug_assert!(sqr_f64(vec.y as f64) < i64::MAX as f64);
                    // WallToolPaths.cpp:259  const int64_t len = vec.norm();
                    let len: i64 = vec.length() as i64;
                    // WallToolPaths.cpp:260-261
                    let mpt = &mut thiss[poly_idx][point_idx];
                    mpt.x += (-(vec.y as i64) * move_dist) / len;
                    mpt.y += ((vec.x as i64) * move_dist) / len;
                }
            }
        }
    }

    // WallToolPaths.cpp:267  ClipperLib::SimplifyPolygons(ClipperUtils::PolygonsProvider(thiss));
    *thiss = clipper_simplify_polygons(thiss);
}

/*
 * Removes overlapping consecutive line segments which don't delimit a positive area.
 */
/// WallToolPaths.cpp:273  void removeDegenerateVerts(Polygons &thiss)
pub fn remove_degenerate_verts(thiss: &mut Polygons) {
    // isDegenerate lambda — WallToolPaths.cpp:279-283
    let is_degenerate = |last: Point, now: Point, next: Point| -> bool {
        let last_line = now - last;
        let next_line = next - now;
        last_line.dot(&next_line) == -1 * (last_line.length() as i128 * next_line.length() as i128) as i128
    };

    // WallToolPaths.cpp:275
    let mut poly_idx = 0usize;
    while poly_idx < thiss.len() {
        // WallToolPaths.cpp:277
        let mut result = Polygon::new();
        // WallToolPaths.cpp:284
        let mut is_changed = false;
        // WallToolPaths.cpp:285
        let poly_size = thiss[poly_idx].len();
        for idx in 0..poly_size {
            // WallToolPaths.cpp:286
            let last = if result.len() == 0 {
                *thiss[poly_idx].points.last().unwrap()
            } else {
                *result.points.last().unwrap()
            };
            // WallToolPaths.cpp:287-288
            if idx + 1 == poly_size && result.len() == 0 {
                break;
            }

            // WallToolPaths.cpp:290
            let next = if idx + 1 == poly_size {
                result[0]
            } else {
                thiss[poly_idx][idx + 1]
            };
            // WallToolPaths.cpp:291
            if is_degenerate(last, thiss[poly_idx][idx], next) {
                // lines are in the opposite direction
                // don't add vert to the result
                // WallToolPaths.cpp:293
                is_changed = true;
                // WallToolPaths.cpp:294-295
                while result.len() > 1
                    && is_degenerate(result[result.len() - 2], *result.points.last().unwrap(), next)
                {
                    result.points.pop();
                }
            } else {
                // WallToolPaths.cpp:297
                result.points.push(thiss[poly_idx][idx]);
            }
        }

        // WallToolPaths.cpp:301-308
        if is_changed {
            if result.len() > 2 {
                thiss[poly_idx] = result;
            } else {
                thiss.remove(poly_idx);
                continue; // effectively the next iteration has the same poly_idx
            }
        }
        poly_idx += 1;
    }
}

/// WallToolPaths.cpp:312  void removeSmallAreas(Polygons &thiss, const double min_area_size, const bool remove_holes)
pub fn remove_small_areas(thiss: &mut Polygons, min_area_size: f64, remove_holes: bool) {
    // to_path lambda + ClipperLib::Area == signed polygon area. Polygon::area() returns the same.
    // WallToolPaths.cpp:314-319

    // WallToolPaths.cpp:321  auto new_end = thiss.end();
    let mut new_end = thiss.len();
    if remove_holes {
        // WallToolPaths.cpp:322-331
        let mut it = 0usize;
        while it < new_end {
            // All polygons smaller than target are removed by replacing them with a polygon from the back of the vector.
            if thiss[it].area().abs() < min_area_size {
                new_end -= 1;
                thiss[it] = thiss[new_end].clone();
                continue; // Don't increment the iterator such that the polygon just swapped in is checked next.
            }
            it += 1;
        }
    } else {
        // For each polygon, computes the signed area, move small outlines at the end of the vector and keep pointer on small holes
        // WallToolPaths.cpp:334
        let mut small_holes: Vec<Polygon> = Vec::new();
        // WallToolPaths.cpp:335
        let mut it = 0usize;
        while it < new_end {
            let area = thiss[it].area();
            // WallToolPaths.cpp:336
            if area.abs() < min_area_size {
                // WallToolPaths.cpp:337
                if area >= 0.0 {
                    // WallToolPaths.cpp:338-344
                    new_end -= 1;
                    if it < new_end {
                        thiss.swap(new_end, it);
                        continue;
                    } else {
                        // Don't self-swap the last Path
                        break;
                    }
                } else {
                    // WallToolPaths.cpp:346
                    small_holes.push(thiss[it].clone());
                }
            }
            it += 1;
        }

        // Removes small holes that have their first point inside one of the removed outlines
        // Iterating in reverse ensures that unprocessed small holes won't be moved
        // WallToolPaths.cpp:354
        let removed_outlines_start = new_end;
        // WallToolPaths.cpp:355  for (auto hole_it = small_holes.rbegin(); hole_it < small_holes.rend(); hole_it++)
        for hole_it in (0..small_holes.len()).rev() {
            // WallToolPaths.cpp:356  for (auto outline_it = removed_outlines_start; outline_it < thiss.end(); outline_it++)
            for outline_it in removed_outlines_start..thiss.len() {
                // WallToolPaths.cpp:357  if (Polygon(*outline_it).contains(*hole_it->begin()))
                if thiss[outline_it].contains(&small_holes[hole_it].points[0]) {
                    // WallToolPaths.cpp:358-360
                    new_end -= 1;
                    small_holes[hole_it] = thiss[new_end].clone();
                    break;
                }
            }
        }
    }
    // WallToolPaths.cpp:363  thiss.resize(new_end-thiss.begin());
    thiss.truncate(new_end);
}

/// WallToolPaths.cpp:366  void removeColinearEdges(Polygon &poly, const double max_deviation_angle)
pub fn remove_colinear_edges_polygon(poly: &mut Polygon, max_deviation_angle: f64) {
    // TODO: Can be made more efficient (for example, use pointer-types for process-/skip-indices, so we can swap them without copy).
    // WallToolPaths.cpp:369
    let mut num_removed_in_iteration: usize;
    // WallToolPaths.cpp:370  do {
    loop {
        // WallToolPaths.cpp:371
        num_removed_in_iteration = 0;
        // WallToolPaths.cpp:372
        let mut process_indices: Vec<bool> = vec![true; poly.points.len()];

        // WallToolPaths.cpp:374
        let mut go = true;
        // WallToolPaths.cpp:375
        while go {
            // WallToolPaths.cpp:376
            go = false;

            // WallToolPaths.cpp:378-379
            let pathlen = poly.len();
            // WallToolPaths.cpp:380-381
            if pathlen <= 3 {
                return;
            }

            // WallToolPaths.cpp:383
            let mut skip_indices: Vec<bool> = vec![false; poly.points.len()];

            // WallToolPaths.cpp:385
            let mut new_path = Polygon::new();
            // WallToolPaths.cpp:386
            let mut point_idx = 0usize;
            while point_idx < pathlen {
                // Don't iterate directly over process-indices, but do it this way, because there are points _in_ process-indices that should nonetheless be skipped:
                // WallToolPaths.cpp:389-392
                if !process_indices[point_idx] {
                    new_path.points.push(poly[point_idx]);
                    point_idx += 1;
                    continue;
                }

                // Should skip the last point for this iteration if the old first was removed (which can be seen from the fact that the new first was skipped):
                // WallToolPaths.cpp:395-400
                if point_idx == (pathlen - 1) && skip_indices[0] {
                    skip_indices[new_path.len()] = true;
                    go = true;
                    new_path.points.push(poly[point_idx]);
                    break;
                }

                // WallToolPaths.cpp:402-404
                let prev = poly[(point_idx + pathlen - 1) % pathlen];
                let pt = poly[point_idx];
                let next = poly[(point_idx + 1) % pathlen];

                // WallToolPaths.cpp:406  [0 : 2 * pi]
                let mut angle = linear_alg2d::get_angle_left(prev, pt, next) as f64;
                // WallToolPaths.cpp:407  map [pi : 2 * pi] to [0 : pi]
                if angle >= std::f64::consts::PI {
                    angle -= std::f64::consts::PI;
                }

                // Check if the angle is within limits for the point to 'make sense', given the maximum deviation.
                // If the angle indicates near-parallel segments ignore the point 'pt'
                // WallToolPaths.cpp:411-419
                if angle > max_deviation_angle && angle < std::f64::consts::PI - max_deviation_angle {
                    new_path.points.push(pt);
                } else if point_idx != (pathlen - 1) {
                    // Skip the next point, since the current one was removed:
                    skip_indices[new_path.len()] = true;
                    go = true;
                    new_path.points.push(next);
                    point_idx += 1;
                }
                point_idx += 1;
            }
            // WallToolPaths.cpp:421
            let old_len = pathlen;
            *poly = new_path;
            // WallToolPaths.cpp:422
            num_removed_in_iteration += old_len - poly.points.len();

            // WallToolPaths.cpp:424-425
            process_indices.clear();
            process_indices.extend_from_slice(&skip_indices);
        }

        // WallToolPaths.cpp:427  } while (num_removed_in_iteration > 0);
        if num_removed_in_iteration == 0 {
            break;
        }
    }
}

/// WallToolPaths.cpp:430  void removeColinearEdges(Polygons &thiss, const double max_deviation_angle = 0.0005)
pub fn remove_colinear_edges(thiss: &mut Polygons, max_deviation_angle: f64) {
    // WallToolPaths.cpp:432
    let mut p: i64 = 0;
    while p < thiss.len() as i64 {
        // WallToolPaths.cpp:433
        remove_colinear_edges_polygon(&mut thiss[p as usize], max_deviation_angle);
        // WallToolPaths.cpp:434-437
        if thiss[p as usize].len() < 3 {
            thiss.remove(p as usize);
            p -= 1;
        }
        p += 1;
    }
}

/// WallToolPaths.cpp:650  template<typename T> bool shorterThan(const T &shape, const coord_t check_length)
/// Instantiated for `ExtrusionLine` in `removeSmallLines`.
fn shorter_than_extrusion_line(shape: &ExtrusionLine, check_length: Coord) -> bool {
    // WallToolPaths.cpp:652
    if shape.junctions.is_empty() {
        return true;
    }
    let mut p0 = &shape.junctions[shape.junctions.len() - 1].p;
    // WallToolPaths.cpp:653
    let mut length: i64 = 0;
    // WallToolPaths.cpp:654
    for p1 in shape.junctions.iter() {
        // WallToolPaths.cpp:655
        length += (*p0 - p1.p).length() as i64;
        // WallToolPaths.cpp:656-657
        if length >= check_length {
            return false;
        }
        p0 = &p1.p;
    }
    // WallToolPaths.cpp:660
    true
}

// ===========================================================================
// ClipperLib::SimplifyPolygons port. Used by fix_self_intersections.
// ===========================================================================

/// `ClipperLib::SimplifyPolygons(ClipperUtils::PolygonsProvider(thiss))` — performs a union of the
/// polygons with itself under the default (non-zero/even-odd-agnostic) fill rule, returning the
/// simplified `Polygons`. The crate's clipper backend exposes this via `union_polygons_ex` →
/// flatten to `Polygons`.
/// FIDELITY-NOTE(F1): geo-clipper approximation vs C++ ClipperLib — ClipperLib uses EvenOdd fill
/// internally for SimplifyPolygons, but the crate's `union_polygons_ex` uses the geo backend
/// (non-zero, fixed scale 1000) rather than ClipperLib at coord_t integer precision.
fn clipper_simplify_polygons(thiss: &Polygons) -> Polygons {
    let ex = clipper_utils::union_polygons_ex(thiss);
    expolygons_to_polygons(&ex)
}

/// Flatten `ExPolygons` into `Polygons` (contour followed by holes), matching
/// `to_polygons(const ExPolygons&)` in ClipperUtils.
fn expolygons_to_polygons(ex: &[ExPolygon]) -> Polygons {
    let mut out: Polygons = Vec::new();
    for e in ex.iter() {
        out.push(e.contour.clone());
        for h in e.holes.iter() {
            out.push(h.clone());
        }
    }
    out
}

/// `union_(const Polygons&)` — NonZero fill. ClipperUtils.cpp.
fn union_polygons(subject: &Polygons) -> Polygons {
    expolygons_to_polygons(&clipper_utils::union_polygons_ex(subject))
}

/// `area(const Polygons&)` — Polygon.hpp:132.
fn area_polygons(polys: &Polygons) -> f64 {
    geom_area_polygons(polys)
}

/// `offset(const Polygons&, const float)` — ClipperUtils. Default jtMiter, mitre-limit 3.
/// FIDELITY-NOTE(F1): geo-clipper approximation vs C++ ClipperLib — offset is computed via the
/// geo backend (fixed scale 1000) rather than ClipperLib at coord_t integer precision.
fn offset_polygons(polygons: &Polygons, delta: Coord) -> Polygons {
    let ex = clipper_utils::union_polygons_ex(polygons);
    let off = clipper_utils::offset_expolygons(
        &ex,
        delta as CoordF / crate::SCALING_FACTOR,
        clipper_utils::OffsetJoinType::Miter,
    );
    expolygons_to_polygons(&off)
}

// ===========================================================================
// LineLoc grid types for getRegionOrder — WallToolPaths.cpp:847-866
// ===========================================================================

/// WallToolPaths.cpp:847-851  struct LineLoc { ExtrusionJunction j; const ExtrusionLine *line; };
/// The `line` pointer is represented as an index into the `input` slice to stay borrow-safe.
#[derive(Debug, Clone, Copy)]
pub struct LineLoc {
    pub j: ExtrusionJunction,
    pub line_idx: usize,
}

/// WallToolPaths.cpp:852-855  struct Locator { Point operator()(const LineLoc &elem) { return elem.j.p; } };
#[derive(Debug, Clone, Copy, Default)]
pub struct LineLocLocator;

impl LocatorTrait<LineLoc> for LineLocLocator {
    fn locate(&self, elem: &LineLoc) -> Point {
        elem.j.p
    }
}

impl WallToolPaths {
    /// WallToolPaths.cpp:26-42  WallToolPaths::WallToolPaths(...)
    pub fn new(
        outline: Polygons,
        bead_width_0: Coord,
        bead_width_x: Coord,
        inset_count: usize,
        wall_0_inset: Coord,
        layer_height: CoordF,
        params: WallToolPathsParams,
    ) -> Self {
        Self {
            // WallToolPaths.cpp:28
            outline,
            // WallToolPaths.cpp:29
            bead_width_0,
            // WallToolPaths.cpp:30
            bead_width_x,
            // WallToolPaths.cpp:31
            inset_count,
            // WallToolPaths.cpp:32
            wall_0_inset,
            // WallToolPaths.cpp:33
            layer_height,
            // WallToolPaths.cpp:34
            print_thin_walls: FILL_OUTLINE_GAPS,
            // WallToolPaths.cpp:35  min_feature_size(scaled<coord_t>(params.min_feature_size))
            min_feature_size: scaled(params.min_feature_size as f64),
            // WallToolPaths.cpp:36  min_bead_width(scaled<coord_t>(params.min_bead_width))
            min_bead_width: scaled(params.min_bead_width as f64),
            // WallToolPaths.cpp:37  small_area_length(static_cast<double>(bead_width_0) / 2.)
            small_area_length: bead_width_0 as f64 / 2.0,
            // WallToolPaths.cpp:38  wall_transition_filter_deviation(scaled<coord_t>(params.wall_transition_filter_deviation))
            wall_transition_filter_deviation: scaled(params.wall_transition_filter_deviation as f64),
            // WallToolPaths.cpp:39
            toolpaths_generated: false,
            toolpaths: Vec::new(),
            inner_contour: Vec::new(),
            first_wall_contour: Vec::new(),
            // WallToolPaths.cpp:40
            m_params: params,
            // WallToolPaths.hpp:146
            enable_hole_compensation: false,
            // WallToolPaths.hpp:147
            hole_indices: Vec::new(),
        }
    }

    /// WallToolPaths.cpp:44-48  void WallToolPaths::EnableHoleCompensation(bool enable_, const std::vector<int>& hole_indices_)
    pub fn enable_hole_compensation(&mut self, enable: bool, hole_indices: Vec<i32>) {
        // WallToolPaths.cpp:46
        self.enable_hole_compensation = enable;
        // WallToolPaths.cpp:47
        self.hole_indices = hole_indices;
    }

    /// WallToolPaths.cpp:441-550  const std::vector<VariableWidthLines> &WallToolPaths::generate()
    pub fn generate(&mut self) -> &Vec<VariableWidthLines> {
        // WallToolPaths.cpp:443-444
        if self.inset_count < 1 {
            return &self.toolpaths;
        }

        // WallToolPaths.cpp:446
        let original_outline_size = self.outline.len();
        // WallToolPaths.cpp:447
        let mut outline_size_change = false;
        // Lambda for checking size changes — WallToolPaths.cpp:449-451
        // (inlined below where used)

        // WallToolPaths.cpp:453
        let smallest_segment: Coord = MESHFIX_MAXIMUM_RESOLUTION;
        // WallToolPaths.cpp:454
        let allowed_distance: Coord = MESHFIX_MAXIMUM_DEVIATION;
        // WallToolPaths.cpp:455
        let epsilon_offset: Coord = (allowed_distance / 2) - 1;
        // WallToolPaths.cpp:456
        let transitioning_angle: f64 = deg2rad(self.m_params.wall_transition_angle as f64);
        // R550: the six resolved params, deduped, mirroring [CPP-WTPPARAMS].
        if std::env::var_os("WTPPARAMS").is_some() {
            use std::collections::BTreeSet;
            use std::sync::Mutex;
            static SEEN: Mutex<Option<BTreeSet<String>>> = Mutex::new(None);
            let line = format!(
                "min_bead_width={:.6} min_feature_size={:.6} wall_transition_length={:.6} \
                 wall_transition_angle={:.6}(deg) -> {:.9}(rad) \
                 wall_transition_filter_deviation={:.6} wall_distribution_count={}",
                self.m_params.min_bead_width,
                self.m_params.min_feature_size,
                self.m_params.wall_transition_length,
                self.m_params.wall_transition_angle,
                transitioning_angle,
                self.m_params.wall_transition_filter_deviation,
                self.m_params.wall_distribution_count,
            );
            if let Ok(mut g) = SEEN.lock() {
                if g.get_or_insert_with(BTreeSet::new).insert(line.clone()) {
                    eprintln!("[WTPPARAMS] {line}");
                }
            }
        }
        // WallToolPaths.cpp:457  (consumed by the blocked SkeletalTrapezoidation wall_maker)
        const _DISCRETIZATION_STEP_SIZE: Coord = scaled_c(0.8);

        // Simplify outline for boost::voronoi consumption. Absolutely no self intersections or near-self intersections allowed:
        // WallToolPaths.cpp:461
        let mut prepared_outline = offset_polygons(
            &offset_polygons(
                &offset_polygons(&self.outline, -epsilon_offset),
                epsilon_offset * 2,
            ),
            -epsilon_offset,
        );
        // WallToolPaths.cpp:462  update_outline_size_change(prepared_outline);
        outline_size_change |= original_outline_size != prepared_outline.len();

        // WallToolPaths.cpp:470-477  process_with_size_check(...) — operation then size check.
        simplify_polygons(&mut prepared_outline, smallest_segment, allowed_distance);
        outline_size_change |= original_outline_size != prepared_outline.len();

        fix_self_intersections(epsilon_offset, &mut prepared_outline);
        outline_size_change |= original_outline_size != prepared_outline.len();

        remove_degenerate_verts(&mut prepared_outline);
        outline_size_change |= original_outline_size != prepared_outline.len();

        remove_colinear_edges(&mut prepared_outline, 0.005);
        outline_size_change |= original_outline_size != prepared_outline.len();

        // Removing collinear edges may introduce self intersections, so we need to fix them again
        fix_self_intersections(epsilon_offset, &mut prepared_outline);
        outline_size_change |= original_outline_size != prepared_outline.len();

        remove_degenerate_verts(&mut prepared_outline);
        outline_size_change |= original_outline_size != prepared_outline.len();

        remove_small_areas(
            &mut prepared_outline,
            self.small_area_length * self.small_area_length,
            false,
        );
        outline_size_change |= original_outline_size != prepared_outline.len();

        // The functions above could produce intersecting polygons that could cause a crash inside Arachne.
        // Applying Clipper union should be enough to get rid of this issue.
        // WallToolPaths.cpp:483
        prepared_outline = union_polygons(&prepared_outline);
        // WallToolPaths.cpp:484
        outline_size_change |= original_outline_size != prepared_outline.len();

        // WallToolPaths.cpp:486-489
        if area_polygons(&prepared_outline) <= 0.0 {
            debug_assert!(self.toolpaths.is_empty());
            return &self.toolpaths;
        }

        // WallToolPaths.cpp:491  (consumed by the blocked SkeletalTrapezoidation wall_maker)
        let _apply_hole_compensation = self.enable_hole_compensation && !outline_size_change;

        // WallToolPaths.cpp:493
        let external_perimeter_extrusion_width = Flow::rounded_rectangle_extrusion_width_from_spacing(
            unscale(self.bead_width_0),
            self.layer_height,
        );
        // WallToolPaths.cpp:494
        let _perimeter_extrusion_width = Flow::rounded_rectangle_extrusion_width_from_spacing(
            unscale(self.bead_width_x),
            self.layer_height,
        );

        // WallToolPaths.cpp:496
        let wall_transition_length: Coord = scaled(self.m_params.wall_transition_length as f64);

        // WallToolPaths.cpp:498
        let wall_split_middle_threshold: f64 = (2.0 * unscaled(self.min_bead_width)
            / external_perimeter_extrusion_width
            - 1.0)
            .clamp(0.01, 0.99);
        // WallToolPaths.cpp:499
        let wall_add_middle_threshold: f64 =
            (unscaled(self.min_bead_width) / _perimeter_extrusion_width).clamp(0.01, 0.99);

        // WallToolPaths.cpp:501
        let wall_distribution_count = self.m_params.wall_distribution_count;
        // WallToolPaths.cpp:502
        let max_bead_count: Coord = if self.inset_count < (Coord::MAX / 2) as usize {
            (2 * self.inset_count) as Coord
        } else {
            Coord::MAX
        };
        // WallToolPaths.cpp:503-517
        let beading_strat = BeadingStrategyFactory::make_strategy(
            self.bead_width_0,
            self.bead_width_x,
            wall_transition_length,
            transitioning_angle,
            self.print_thin_walls,
            self.min_bead_width,
            self.min_feature_size,
            wall_split_middle_threshold,
            wall_add_middle_threshold,
            max_bead_count,
            self.wall_0_inset,
            wall_distribution_count,
            // minimum_variable_line_ratio (extra arg present in this crate's factory; C++ default).
            0.5,
        );
        // WallToolPaths.cpp:518
        let transition_filter_dist: Coord = scaled(100.0);
        // WallToolPaths.cpp:519
        let allowed_filter_deviation: Coord = self.wall_transition_filter_deviation;

        // WallToolPaths.cpp:520-532
        //   SkeletalTrapezoidation wall_maker(prepared_outline, *beading_strat, ...);
        //   wall_maker.generateToolpaths(toolpaths);
        // The Rust SkeletalTrapezoidation::new does not call construct_from_polygons
        // in its ctor (see skeletal_trapezoidation.rs), so the graph is built
        // explicitly after construction, mirroring the C++ ctor body
        // (constructFromPolygons(polys)).
        {
            use crate::arachne::skeletal_trapezoidation::SkeletalTrapezoidation;
            let mut wall_maker = SkeletalTrapezoidation::new(
                // WallToolPaths.cpp:524 *beading_strat
                &*beading_strat,
                // WallToolPaths.cpp:525 beading_strat->getTransitioningAngle()
                beading_strat.get_transitioning_angle(),
                // WallToolPaths.cpp:526 discretization_step_size
                _DISCRETIZATION_STEP_SIZE,
                // WallToolPaths.cpp:527 transition_filter_dist
                transition_filter_dist,
                // WallToolPaths.cpp:528 allowed_filter_deviation
                allowed_filter_deviation,
                // WallToolPaths.cpp:529 wall_transition_length (beading_propagation_transition_dist)
                wall_transition_length,
                // WallToolPaths.cpp:530 apply_hole_compensation
                _apply_hole_compensation,
                // WallToolPaths.cpp:531 hole_indices
                self.hole_indices.clone(),
            );
            // C++ ctor body: constructFromPolygons(polys) — here `polys` is the
            // prepared_outline argument the C++ ctor receives at WallToolPaths.cpp:522.
            wall_maker.construct_from_polygons(&prepared_outline);
            // WallToolPaths.cpp:533 wall_maker.generateToolpaths(toolpaths);
            // generateToolpaths defaults filter_outermost_central_edges = true
            // (SkeletalTrapezoidation.hpp).
            wall_maker.generate_toolpaths(&mut self.toolpaths, true);
        }

        // R544 probe (STAGEPROBE=1): bracket EVERY post-processing stage (R543's
        // method). The width variation is intact at junction creation (28,419
        // distinct) and 98% flat per loop by the time the perimeter generator sees
        // it, so exactly one of these five stages flattens it.
        stageprobe("0 after generate_toolpaths", &self.toolpaths);

        // WallToolPaths.cpp:534
        Self::stitch_tool_paths(&mut self.toolpaths, self.bead_width_x);
        stageprobe("1 after stitch_tool_paths", &self.toolpaths);

        // WallToolPaths.cpp:536
        Self::remove_small_lines(&mut self.toolpaths);
        stageprobe("2 after remove_small_lines", &self.toolpaths);

        // WallToolPaths.cpp:538
        self.separate_out_inner_contour();
        stageprobe("3 after separate_out_inner_contour", &self.toolpaths);

        // WallToolPaths.cpp:540
        Self::simplify_tool_paths(&mut self.toolpaths);
        stageprobe("4 after simplify_tool_paths", &self.toolpaths);

        // WallToolPaths.cpp:542
        Self::remove_empty_tool_paths(&mut self.toolpaths);
        stageprobe("5 after remove_empty_tool_paths", &self.toolpaths);
        // WallToolPaths.cpp:543-547  assert sorted by inset_idx (debug-only)
        debug_assert!(self
            .toolpaths
            .windows(2)
            .all(|w| w[0][0].inset_idx <= w[1][0].inset_idx));
        // WallToolPaths.cpp:548
        self.toolpaths_generated = true;
        // WallToolPaths.cpp:549
        &self.toolpaths
    }

    /// WallToolPaths.cpp:552-648  void WallToolPaths::stitchToolPaths(std::vector<VariableWidthLines> &toolpaths, const coord_t bead_width_x)
    pub fn stitch_tool_paths(toolpaths: &mut Vec<VariableWidthLines>, bead_width_x: Coord) {
        // WallToolPaths.cpp:554  In 0-width contours, junctions can cause up to 1-line-width gaps.
        let stitch_distance: Coord = bead_width_x - 1;

        // WallToolPaths.cpp:556
        for wall_idx in 0..toolpaths.len() {
            // WallToolPaths.cpp:559-560
            let mut stitched_polylines: VariableWidthLines = Vec::new();
            let mut closed_polygons: VariableWidthLines = Vec::new();
            // WallToolPaths.cpp:561
            // PolylineStitcher<VariableWidthLines, ExtrusionLine, ExtrusionJunction>::stitch(
            //     wall_lines, stitched_polylines, closed_polygons, stitch_distance)
            // (snap_distance takes the C++ default scaled(0.01), PolylineStitcher.hpp:53.)
            let wall_lines = std::mem::take(&mut toolpaths[wall_idx]);
            PolylineStitcher::stitch_extrusion(
                &wall_lines,
                &mut stitched_polylines,
                &mut closed_polygons,
                stitch_distance,
                scaled(0.01),
            );

            // WallToolPaths.cpp:622  wall_lines = stitched_polylines;
            toolpaths[wall_idx] = stitched_polylines;

            // WallToolPaths.cpp:624
            for mut wall_polygon in closed_polygons.into_iter() {
                // WallToolPaths.cpp:626-629
                if wall_polygon.junctions.is_empty() {
                    continue;
                }

                // PolylineStitcher, in some cases, produced closed extrusion (polygons),
                // but the endpoints differ by a small distance. So we reconnect them.
                // WallToolPaths.cpp:634-637
                if wall_polygon.junctions.first().unwrap().p != wall_polygon.junctions.last().unwrap().p
                    && (wall_polygon.junctions.last().unwrap().p
                        - wall_polygon.junctions.first().unwrap().p)
                        .length()
                        < stitch_distance as f64
                {
                    let front = *wall_polygon.junctions.first().unwrap();
                    wall_polygon.junctions.push(front);
                }
                // WallToolPaths.cpp:638
                wall_polygon.is_closed = true;
                // WallToolPaths.cpp:639  add stitched polygons to result
                toolpaths[wall_idx].push(wall_polygon);
            }
        }
    }

    /// WallToolPaths.cpp:663-678  void WallToolPaths::removeSmallLines(std::vector<VariableWidthLines> &toolpaths)
    pub fn remove_small_lines(toolpaths: &mut Vec<VariableWidthLines>) {
        // WallToolPaths.cpp:665
        for inset in toolpaths.iter_mut() {
            // WallToolPaths.cpp:666
            let mut line_idx = 0usize;
            while line_idx < inset.len() {
                // WallToolPaths.cpp:667-670
                let mut min_width: Coord = Coord::MAX;
                for j in inset[line_idx].junctions.iter() {
                    min_width = std::cmp::min(min_width, j.w);
                }
                // WallToolPaths.cpp:671
                let line = &inset[line_idx];
                if line.is_odd && !line.is_closed && shorter_than_extrusion_line(line, min_width / 2) {
                    // remove line — WallToolPaths.cpp:672-674
                    let last = inset.len() - 1;
                    inset[line_idx] = inset[last].clone();
                    inset.pop();
                    // line_idx-- then loop ++ -> reconsider current position.
                    continue;
                }
                line_idx += 1;
            }
        }
    }

    /// WallToolPaths.cpp:680-692  void WallToolPaths::simplifyToolPaths(std::vector<VariableWidthLines> &toolpaths)
    pub fn simplify_tool_paths(toolpaths: &mut Vec<VariableWidthLines>) {
        // WallToolPaths.cpp:682
        for toolpaths_idx in 0..toolpaths.len() {
            // WallToolPaths.cpp:684
            let maximum_resolution: i64 = MESHFIX_MAXIMUM_RESOLUTION;
            // WallToolPaths.cpp:685
            let maximum_deviation: i64 = MESHFIX_MAXIMUM_DEVIATION;
            // WallToolPaths.cpp:686  unit: μm²
            let maximum_extrusion_area_deviation: i64 = MESHFIX_MAXIMUM_EXTRUSION_AREA_DEVIATION;
            // WallToolPaths.cpp:687
            for line in toolpaths[toolpaths_idx].iter_mut() {
                // WallToolPaths.cpp:689
                line.simplify(
                    maximum_resolution * maximum_resolution,
                    maximum_deviation * maximum_deviation,
                    maximum_extrusion_area_deviation,
                );
            }
        }
    }

    /// WallToolPaths.cpp:694-699  const std::vector<VariableWidthLines> &WallToolPaths::getToolPaths()
    pub fn get_tool_paths(&mut self) -> &Vec<VariableWidthLines> {
        // WallToolPaths.cpp:696-697
        if !self.toolpaths_generated {
            return self.generate();
        }
        // WallToolPaths.cpp:698
        &self.toolpaths
    }

    /// WallToolPaths.cpp:701-770  void WallToolPaths::separateOutInnerContour()
    pub fn separate_out_inner_contour(&mut self) {
        // enum PathType { ActualPath, WallContour, FirstWallContour } — WallToolPaths.cpp:703-707

        // We'll remove all 0-width paths from the original toolpaths and store them separately as polygons.
        // WallToolPaths.cpp:710-711
        let mut actual_toolpaths: Vec<VariableWidthLines> = Vec::with_capacity(self.toolpaths.len());
        // WallToolPaths.cpp:712-713  wall_contour_paths (reserved, unused for output)
        // WallToolPaths.cpp:714  first_wall_contour_paths (unused for output)

        // WallToolPaths.cpp:715-716
        self.inner_contour.clear();
        self.first_wall_contour.clear();

        // WallToolPaths.cpp:717
        for inset in self.toolpaths.iter() {
            // WallToolPaths.cpp:718-719
            if inset.is_empty() {
                continue;
            }
            // WallToolPaths.cpp:720
            // `type` is determined from the first junction of the (last) line — matches C++ which
            // overwrites `type` for every line but `break`s after the first junction of each.
            #[derive(PartialEq)]
            enum PathType {
                ActualPath,
                WallContour,
                FirstWallContour,
            }
            let mut path_type = PathType::ActualPath;
            // WallToolPaths.cpp:721-731
            for line in inset.iter() {
                for j in line.junctions.iter() {
                    if j.w == WALL_CONTOUR_MARKED_WIDTH {
                        path_type = PathType::WallContour;
                    } else if j.w == FIRST_WALL_CONTOUR_MARKED_WIDTH {
                        path_type = PathType::FirstWallContour;
                    } else {
                        path_type = PathType::ActualPath;
                    }
                    break;
                }
            }

            // WallToolPaths.cpp:733
            if path_type == PathType::WallContour {
                // WallToolPaths.cpp:739-744
                for line in inset.iter() {
                    if line.is_odd {
                        continue; // odd lines don't contribute to the contour
                    } else if line.is_closed {
                        // sometimes an very small even polygonal wall is not stitched into a polygon
                        self.inner_contour.push(line.to_polygon());
                    }
                }
            } else if path_type == PathType::FirstWallContour {
                // WallToolPaths.cpp:746-753
                for line in inset.iter() {
                    if line.is_odd {
                        continue;
                    } else if line.is_closed {
                        self.first_wall_contour.push(line.to_polygon());
                    }
                }
            } else {
                // WallToolPaths.cpp:755
                actual_toolpaths.push(inset.clone());
            }
        }
        // WallToolPaths.cpp:758-761
        if !actual_toolpaths.is_empty() {
            self.toolpaths = actual_toolpaths; // Filtered out the 0-width paths.
        } else {
            self.toolpaths.clear();
        }

        // The output walls from the skeletal trapezoidation have no known winding order...
        // The even-odd rule would be incorrect if the polygon self-intersects, but that should never be generated by the skeletal trapezoidation.
        // WallToolPaths.cpp:768  inner_contour = union_(inner_contour, ClipperLib::PolyFillType::pftEvenOdd);
        // FIDELITY-NOTE(F1): geo-clipper approximation vs C++ ClipperLib — the crate clipper
        // backend (union_polygons_ex) only offers a non-zero union; the even-odd fill rule used
        // here by C++ for self-overlapping wall contours is not available at coord_t precision.
        self.inner_contour = union_polygons(&self.inner_contour);
        // WallToolPaths.cpp:769
        self.first_wall_contour = union_polygons(&self.first_wall_contour);
    }

    /// WallToolPaths.cpp:772-783  const Polygons& WallToolPaths::getInnerContour()
    pub fn get_inner_contour(&mut self) -> &Polygons {
        // WallToolPaths.cpp:774-777
        if !self.toolpaths_generated && self.inset_count > 0 {
            self.generate();
        } else if self.inset_count == 0 {
            // WallToolPaths.cpp:778-781
            return &self.outline;
        }
        // WallToolPaths.cpp:782
        &self.inner_contour
    }

    /// WallToolPaths.cpp:785-796  const Polygons& WallToolPaths::getFirstWallContour()
    pub fn get_first_wall_contour(&mut self) -> &Polygons {
        // WallToolPaths.cpp:787-790
        if !self.toolpaths_generated && self.inset_count > 0 {
            self.generate();
        } else if self.inset_count == 0 {
            // WallToolPaths.cpp:784,793  static `Polygons EmptyPolygons;` returned by const-ref.
            // Faithfully return a separate empty `Polygons` rather than mutating the member.
            static EMPTY_POLYGONS: std::sync::OnceLock<Polygons> = std::sync::OnceLock::new();
            return EMPTY_POLYGONS.get_or_init(Vec::new);
        }
        // WallToolPaths.cpp:795
        &self.first_wall_contour
    }

    /// WallToolPaths.cpp:799-806  bool WallToolPaths::removeEmptyToolPaths(std::vector<VariableWidthLines> &toolpaths)
    pub fn remove_empty_tool_paths(toolpaths: &mut Vec<VariableWidthLines>) -> bool {
        // WallToolPaths.cpp:801-804
        toolpaths.retain(|lines| !lines.is_empty());
        // WallToolPaths.cpp:805
        toolpaths.is_empty()
    }

    /// WallToolPaths.cpp:816-901  WallToolPaths::getRegionOrder(...)
    ///
    /// Returns the set of ordered pairs (a, b) of line indices into `input`, where `a` must be
    /// printed before `b`. C++ returns pointer pairs; here we return index pairs into `input`
    /// (a pointer-pair would not be hashable/portable across the FFI boundary).
    pub fn get_region_order(
        input: &[&ExtrusionLine],
        outer_to_inner: bool,
    ) -> std::collections::HashSet<(usize, usize)> {
        // WallToolPaths.cpp:818
        let mut order_requirements: std::collections::HashSet<(usize, usize)> =
            std::collections::HashSet::new();

        // WallToolPaths.cpp:840-843  compute max_line_w
        let mut max_line_w: Coord = 0;
        for line in input.iter() {
            for junction in line.junctions.iter() {
                max_line_w = std::cmp::max(max_line_w, junction.w);
            }
        }
        // WallToolPaths.cpp:844-845
        if max_line_w == 0 {
            return order_requirements;
        }

        // How much farther two verts may be apart due to corners.
        // WallToolPaths.cpp:863
        const DIAGONAL_EXTENSION: f32 = 1.9;
        // WallToolPaths.cpp:864
        let searching_radius: Coord = (max_line_w as f32 * DIAGONAL_EXTENSION) as Coord;
        // WallToolPaths.cpp:865-866
        let mut grid: SparsePointGrid<LineLoc, LineLocLocator> =
            SparsePointGrid::new(searching_radius, 0, 1.0);

        // WallToolPaths.cpp:868-869
        for (line_idx, line) in input.iter().enumerate() {
            for junction in line.junctions.iter() {
                grid.insert(LineLoc {
                    j: *junction,
                    line_idx,
                });
            }
        }
        // WallToolPaths.cpp:870  for (const std::pair<const SquareGrid::GridPoint, LineLoc> &pair : grid)
        // Iterating `input`'s junctions yields exactly the same set of `lineloc_here` as iterating
        // the grid (every junction was inserted exactly once).
        for (here_idx, here_line) in input.iter().enumerate() {
            for junction in here_line.junctions.iter() {
                // WallToolPaths.cpp:871-873
                let here = here_idx;
                let loc_here = junction.p;
                let w_here = junction.w;
                // WallToolPaths.cpp:874
                let nearby_verts = grid.get_nearby(loc_here, searching_radius);
                // WallToolPaths.cpp:875
                for lineloc_nearby in nearby_verts.iter() {
                    // WallToolPaths.cpp:876
                    let nearby = lineloc_nearby.line_idx;
                    // WallToolPaths.cpp:877-878
                    if nearby == here {
                        continue;
                    }
                    let here_inset = input[here].inset_idx;
                    let nearby_inset = input[nearby].inset_idx;
                    // WallToolPaths.cpp:879-880
                    if nearby_inset == here_inset {
                        continue;
                    }
                    // WallToolPaths.cpp:881-882  not directly adjacent
                    if nearby_inset > here_inset + 1 {
                        continue;
                    }
                    // WallToolPaths.cpp:883-884  not directly adjacent
                    if here_inset > nearby_inset + 1 {
                        continue;
                    }
                    // WallToolPaths.cpp:885-886  points are too far away from each other
                    let dvec = loc_here - lineloc_nearby.j.p;
                    let thresh: Coord = (((w_here + lineloc_nearby.j.w) / 2) as f32
                        * DIAGONAL_EXTENSION) as Coord;
                    if !shorter_then(&dvec, thresh) {
                        continue;
                    }
                    let here_is_odd = input[here].is_odd;
                    let nearby_is_odd = input[nearby].is_odd;
                    // WallToolPaths.cpp:887-897
                    if here_is_odd || nearby_is_odd {
                        // WallToolPaths.cpp:888-889
                        if here_is_odd && !nearby_is_odd && nearby_inset < here_inset {
                            order_requirements.insert((nearby, here));
                        }
                        // WallToolPaths.cpp:890-891
                        if nearby_is_odd && !here_is_odd && here_inset < nearby_inset {
                            order_requirements.insert((here, nearby));
                        }
                    } else if (nearby_inset < here_inset) == outer_to_inner {
                        // WallToolPaths.cpp:892-893
                        order_requirements.insert((nearby, here));
                    } else {
                        // WallToolPaths.cpp:894-896
                        debug_assert!((nearby_inset > here_inset) == outer_to_inner);
                        order_requirements.insert((here, nearby));
                    }
                }
            }
        }
        // WallToolPaths.cpp:900
        order_requirements
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_wall_tool_paths_params_default() {
        let params = WallToolPathsParams::default();
        assert!(params.min_bead_width > 0.0);
        assert!(params.min_feature_size > 0.0);
        assert!(params.wall_transition_angle > 0.0);
    }

    #[test]
    fn test_meshfix_constants() {
        // scaled<coord_t>(0.5) with SCALING_FACTOR = 100_000 -> 50_000
        assert_eq!(MESHFIX_MAXIMUM_RESOLUTION, 50_000);
        assert_eq!(MESHFIX_MAXIMUM_DEVIATION, 2_500);
        assert_eq!(MESHFIX_MAXIMUM_EXTRUSION_AREA_DEVIATION, 200_000);
    }

    #[test]
    fn test_wall_tool_paths_creation() {
        let outline = Polygons::new();
        let params = WallToolPathsParams::default();
        let wall_paths = WallToolPaths::new(outline, 400, 400, 3, 0, 0.2, params);

        assert_eq!(wall_paths.bead_width_0, 400);
        assert_eq!(wall_paths.bead_width_x, 400);
        assert_eq!(wall_paths.inset_count, 3);
        assert!(!wall_paths.toolpaths_generated);
    }

    #[test]
    fn test_enable_hole_compensation() {
        let outline = Polygons::new();
        let params = WallToolPathsParams::default();
        let mut wall_paths = WallToolPaths::new(outline, 400, 400, 3, 0, 0.2, params);

        assert!(!wall_paths.enable_hole_compensation);

        wall_paths.enable_hole_compensation(true, vec![0, 1, 2]);
        assert!(wall_paths.enable_hole_compensation);
        assert_eq!(wall_paths.hole_indices, vec![0, 1, 2]);
    }

    #[test]
    fn test_remove_empty_tool_paths() {
        let mut toolpaths = vec![vec![], vec![ExtrusionLine::new(0, false)], vec![]];

        // returns true iff the result is empty
        let all_empty = WallToolPaths::remove_empty_tool_paths(&mut toolpaths);
        assert!(!all_empty);
        assert_eq!(toolpaths.len(), 1);
    }

    #[test]
    fn test_remove_empty_tool_paths_all_empty() {
        let mut toolpaths: Vec<VariableWidthLines> = vec![vec![], vec![], vec![]];

        let all_empty = WallToolPaths::remove_empty_tool_paths(&mut toolpaths);
        assert!(all_empty);
        assert_eq!(toolpaths.len(), 0);
    }
}

/// R544 probe (STAGEPROBE=1): per-stage width-variation accounting inside
/// `WallToolPaths::generate`'s post-processing chain.
///
/// Reports, per stage: number of ExtrusionLines, total junctions, and the share
/// of lines whose junction widths are all equal ("flat"). The stage where the
/// flat share jumps is the one that discards the variation.
#[allow(dead_code)]
pub(crate) fn stageprobe(stage: &str, toolpaths: &[VariableWidthLines]) {
    if std::env::var_os("STAGEPROBE").is_none() {
        return;
    }
    use std::collections::HashMap;
    use std::sync::Mutex;
    static ACC: Mutex<Option<HashMap<String, (usize, usize, usize, usize)>>> = Mutex::new(None);
    let mut lines = 0usize;
    let mut juncs = 0usize;
    let mut flat = 0usize;
    let mut distinct_total = 0usize;
    for vwl in toolpaths {
        for line in vwl.iter() {
            if line.junctions.is_empty() {
                continue;
            }
            lines += 1;
            juncs += line.junctions.len();
            let mut ws: Vec<i64> = line.junctions.iter().map(|j| j.w).collect();
            ws.sort_unstable();
            ws.dedup();
            distinct_total += ws.len();
            if ws.len() <= 1 {
                flat += 1;
            }
        }
    }
    if let Ok(mut g) = ACC.lock() {
        let m = g.get_or_insert_with(HashMap::new);
        let e = m.entry(stage.to_string()).or_insert((0, 0, 0, 0));
        e.0 += lines;
        e.1 += juncs;
        e.2 += flat;
        e.3 += distinct_total;
        // Print a full table every time stage 5 has accumulated a round number of
        // lines, so the stages are always compared on the same population.
        if stage.starts_with('5') && e.0 > 0 && e.0 % 20_000 < lines.max(1) {
            let mut keys: Vec<&String> = m.keys().collect();
            keys.sort();
            eprintln!("[STAGEPROBE] ---- cumulative ----");
            for k in keys {
                let (l, j, f, d) = m[k];
                eprintln!(
                    "  {k:38} lines={l:8} juncs={j:9} flat={:5.1}% distinct_w/line={:.2}",
                    100.0 * f as f64 / l.max(1) as f64,
                    d as f64 / l.max(1) as f64,
                );
            }
        }
    }
}
