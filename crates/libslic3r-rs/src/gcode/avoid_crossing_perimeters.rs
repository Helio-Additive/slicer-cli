//! Faithful 1:1 port of `GCode/AvoidCrossingPerimeters.cpp` (+ `.hpp`) from
//! BambuStudio/libslic3r.
//!
//! This module routes travel moves around perimeter walls to avoid crossing
//! them. The port preserves the C++ function order, names (snake_case),
//! control flow, constants, and integer-vs-float arithmetic.
//!
//! # Parity notes
//!
//! - `coord_t` -> `i64` (`Point::x`/`y` are `i64`), `coordf_t` -> `f64`.
//! - The C++ `.cast<double>()` on integer `Point`/`Vec2crd` is a *raw* cast
//!   of the scaled integer value to `f64`. It is NOT `Point::to_f64()`, which
//!   *unscales*. To preserve byte-exact arithmetic this port casts the raw
//!   `i64` coordinates with `as f64` directly (see `cast_d` helper).
//! - The C++ `EdgeGrid::Grid::line(idx)` accessor maps to the Rust
//!   `EdgeGrid::segment(idx)` which returns the same `Line`.
//!
//! # Context of a layer
//!
//! The C++ reaches the print object and the print of a `const Layer &` through its parent pointers,
//! and tests `dynamic_cast<const SupportLayer *>(&layer)`. Here they come with the layer as a
//! [`LayerOwner`]: the `crate::print::faithful::Print`, the index of the object in its `objects()`,
//! and the support layer when the layer is one. `GCode` is `crate::g_code::gcode_class::GCode`.
//!
//! The perimeter-spacing helpers (`get_default_perimeter_spacing`,
//! `get_perimeter_spacing`, `get_perimeter_spacing_external`,
//! `get_external_perimeter_width`) read the config hierarchy wired on the layer
//! (`layer.object().print().config()`, `LayerRegion::region()`/`flow`).

// AvoidCrossingPerimeters.cpp:1-14 — includes
use crate::clipper::{ClipType, JoinType, Path, Paths, PolyFillType, PolyTree};
use crate::clipper_utils::{
    _clipper_ex, clipper_do, clipper_do_polytree, expolygons_offset_pt, offset_paths, poly_tree_to_ex_polygons, raw_offset,
    variable_offset_inner_ex_clib, ApplySafetyOffset,
};
use crate::edge_grid::EdgeGrid;
use crate::g_code::gcode_class::GCode;
use crate::gcode::wipe_tower::{cfg_floats, get_at};
use crate::geometry::ExPolygon;
use crate::print::faithful::{Print, SupportLayer};
use crate::flow::FlowRole;
use crate::geometry::{perp, BoundingBox, BoundingBoxF, Line, Point, PointF, Polygon, Polyline};
use crate::layer::Layer;
use crate::utils::{next_idx_modulo, prev_idx_modulo};
use std::collections::HashSet;

// AvoidCrossingPerimeters.cpp — `SCALED_EPSILON` (libslic3r.h, ported as f64 = 10.0).
use crate::libslic3r::SCALED_EPSILON;

// Vec2d in libslic3r maps to the crate's PointF (f64 2D vector).
type Vec2d = PointF;

/// Raw `.cast<double>()` of a scaled integer `Point` (Eigen cast, no unscale).
#[inline]
fn cast_d(p: Point) -> Vec2d {
    Vec2d {
        x: p.x as f64,
        y: p.y as f64,
    }
}

// AvoidCrossingPerimeters.cpp:18 — struct TravelPoint
#[derive(Clone, Copy)]
struct TravelPoint {
    point: Point,
    // Index of the polygon containing this point. A negative value indicates that the point is not on any border.
    border_idx: i32,
    // simplify_travel() doesn't remove this point.
    do_not_remove: bool, // = false
}

// AvoidCrossingPerimeters.cpp:27 — struct Intersection
#[derive(Clone, Copy)]
struct Intersection {
    // Index of the polygon containing this point of intersection.
    border_idx: usize,
    // Index of the line on the polygon containing this point of intersection.
    line_idx: usize,
    // Point of intersection.
    point: Point,
    // Distance from the first point in the corresponding boundary
    distance: f32,
    // simplify_travel() doesn't remove this point.
    do_not_remove: bool, // = false
}

// AvoidCrossingPerimeters.cpp:41 — struct ClosestLine
#[derive(Clone, Copy)]
struct ClosestLine {
    // Index of the polygon containing this line.
    border_idx: usize,
    // Index of this line on the polygon containing it.
    line_idx: usize,
    // Closest point on the line.
    point: Point,
}

// AvoidCrossingPerimeters.cpp:51-89 — struct AllIntersectionsVisitor
// Finding all intersections of a set of contours with a line segment.
//
// In C++ this is a stateful functor invoked via grid.visit_cells_intersecting_line.
// The Rust EdgeGrid invokes a `FnMut(usize, usize)` closure, so the visitor state
// is held in local variables captured by the closure in `apply_*` below.
struct AllIntersectionsVisitor<'a> {
    grid: &'a EdgeGrid,
    intersections: Vec<Intersection>,
    travel_line: Line,
    intersection_set: HashSet<(usize, usize)>,
}

impl<'a> AllIntersectionsVisitor<'a> {
    // AvoidCrossingPerimeters.cpp:54 — AllIntersectionsVisitor(grid, intersections)
    #[allow(dead_code)]
    fn new(grid: &'a EdgeGrid) -> Self {
        AllIntersectionsVisitor {
            grid,
            intersections: Vec::new(),
            travel_line: Line::new(Point::new(0, 0), Point::new(0, 0)),
            intersection_set: HashSet::new(),
        }
    }

    // AvoidCrossingPerimeters.cpp:59 — AllIntersectionsVisitor(grid, intersections, travel_line)
    fn with_line(grid: &'a EdgeGrid, travel_line: Line) -> Self {
        AllIntersectionsVisitor {
            grid,
            intersections: Vec::new(),
            travel_line,
            intersection_set: HashSet::new(),
        }
    }

    // AvoidCrossingPerimeters.cpp:65 — void reset()
    #[allow(dead_code)]
    fn reset(&mut self) {
        self.intersection_set.clear();
    }

    // AvoidCrossingPerimeters.cpp:69 — bool operator()(coord_t iy, coord_t ix)
    fn visit(&mut self, iy: usize, ix: usize) -> bool {
        // Called with a row and column of the grid cell, which is intersected by a line.
        let cell_data_range = self.grid.cell_data_range_at(iy, ix);
        for &it_contour_and_segment in cell_data_range {
            // AvoidCrossingPerimeters.cpp:75-79
            if let Some(intersection_point) =
                self.travel_line.intersection(&self.grid.segment(it_contour_and_segment))
            {
                if !self.intersection_set.contains(&it_contour_and_segment) {
                    self.intersections.push(Intersection {
                        border_idx: it_contour_and_segment.0,
                        line_idx: it_contour_and_segment.1,
                        point: intersection_point,
                        distance: 0.0,
                        do_not_remove: false,
                    });
                    self.intersection_set.insert(it_contour_and_segment);
                }
            }
        }
        // Continue traversing the grid along the edge.
        true
    }

    // Run the visitor along the stored travel_line and return the collected intersections.
    fn run(mut self) -> Vec<Intersection> {
        let a = self.travel_line.a;
        let b = self.travel_line.b;
        // The C++ AllIntersectionsVisitor is invoked with the same start/end as the travel line.
        let grid = self.grid;
        grid.visit_cells_intersecting_line(a, b, |iy, ix| self.visit(iy, ix));
        self.intersections
    }
}

// AvoidCrossingPerimeters.cpp:91-119 — struct FirstIntersectionVisitor
// Visitor to check for any collision of a line segment with any contour stored inside the edge_grid.
//
// Returns whether `pt_current`..`pt_next` intersects any stored segment.
fn first_intersection_visitor_intersect(
    grid: &EdgeGrid,
    pt_current: &Point,
    pt_next: &Point,
) -> bool {
    // AvoidCrossingPerimeters.cpp:96-113 — operator()
    let mut intersect = false;
    grid.visit_cells_intersecting_line(*pt_current, *pt_next, |iy, ix| {
        let cell_data_range = grid.cell_data_range_at(iy, ix);
        for &it_contour_and_segment in cell_data_range {
            // End points of the line segment and their vector.
            let segment = grid.segment(it_contour_and_segment);
            if crate::geometry::segments_intersect(segment.a, segment.b, *pt_current, *pt_next) {
                intersect = true;
                // AvoidCrossingPerimeters.cpp — return false to stop traversal.
                return false;
            }
        }
        // Continue traversing the grid along the edge.
        true
    });
    intersect
}

// AvoidCrossingPerimeters.cpp:121-157 — struct MinDistanceVisitor
// Visitor to create a list of closet lines to a defined point.
struct MinDistanceVisitor<'a> {
    grid: &'a EdgeGrid,
    center: Point,
    closest_lines: Vec<ClosestLine>,
    closest_lines_set: HashSet<(usize, usize)>,
    max_distance_squared: f64,
}

impl<'a> MinDistanceVisitor<'a> {
    // AvoidCrossingPerimeters.cpp:124 — MinDistanceVisitor(grid, center, max_distance_squared)
    fn new(grid: &'a EdgeGrid, center: Point, max_distance_squared: f64) -> Self {
        MinDistanceVisitor {
            grid,
            center,
            closest_lines: Vec::new(),
            closest_lines_set: HashSet::new(),
            max_distance_squared,
        }
    }

    // AvoidCrossingPerimeters.cpp:128 — void init()
    #[allow(dead_code)]
    fn init(&mut self) {
        self.closest_lines.clear();
        self.closest_lines_set.clear();
    }

    // AvoidCrossingPerimeters.cpp:134 — bool operator()(coord_t iy, coord_t ix)
    fn visit(&mut self, iy: usize, ix: usize) -> bool {
        // Called with a row and column of the grid cell, which is inside a bounding box.
        let cell_data_range = self.grid.cell_data_range_at(iy, ix);
        for &it_contour_and_segment in cell_data_range {
            // End points of the line segment and their vector.
            let segment = self.grid.segment(it_contour_and_segment);
            // AvoidCrossingPerimeters.cpp:142-146
            if !self.closest_lines_set.contains(&it_contour_and_segment) {
                let mut closest_point = Point::new(0, 0);
                let dist_sq = line_alg_distance_to_squared(
                    &Line::new(segment.a, segment.b),
                    &self.center,
                    &mut closest_point,
                );
                if dist_sq <= self.max_distance_squared {
                    self.closest_lines.push(ClosestLine {
                        border_idx: it_contour_and_segment.0,
                        line_idx: it_contour_and_segment.1,
                        point: closest_point,
                    });
                    self.closest_lines_set.insert(it_contour_and_segment);
                }
            }
        }
        // Continue traversing the grid along the edge.
        true
    }
}

// `line_alg::distance_to_squared(line, point, &closest_point)` — distance squared
// from `point` to the segment `line`, writing the closest point. Mirrors the
// libslic3r `line_alg::distance_to_squared` semantics used by MinDistanceVisitor.
// Line.hpp:43-69
fn line_alg_distance_to_squared(line: &Line, point: &Point, closest_point: &mut Point) -> f64 {
    // Line.hpp:45-47
    let v = cast_d(Point::new(line.b.x - line.a.x, line.b.y - line.a.y));
    let va = cast_d(Point::new(point.x - line.a.x, point.y - line.a.y));
    let l2 = v.x * v.x + v.y * v.y; // avoid a sqrt
    let va_sq = va.x * va.x + va.y * va.y;
    // Line.hpp:48-52 — a == b case
    if l2 == 0.0 {
        *closest_point = line.a;
        return va_sq;
    }
    // Line.hpp:53-56 — projection parameter t = (va . v) / |v|^2
    let t = (va.x * v.x + va.y * v.y) / l2;
    if t <= 0.0 {
        // Line.hpp:57-60 — beyond the 'a' end of the segment
        *closest_point = line.a;
        va_sq
    } else if t >= 1.0 {
        // Line.hpp:61-64 — beyond the 'b' end of the segment
        *closest_point = line.b;
        let d = cast_d(Point::new(point.x - line.b.x, point.y - line.b.y));
        d.x * d.x + d.y * d.y
    } else {
        // Line.hpp:67-68 — projection falls within the segment
        let foot_x = line.a.x as f64 + t * v.x;
        let foot_y = line.a.y as f64 + t * v.y;
        *closest_point = Point::new(foot_x as i64, foot_y as i64);
        let rx = t * v.x - va.x;
        let ry = t * v.y - va.y;
        rx * rx + ry * ry
    }
}

// AvoidCrossingPerimeters.cpp:159-170 — get_closest_lines_in_radius
// Returns sorted list of closest lines to a passed point within a passed radius
fn get_closest_lines_in_radius(grid: &EdgeGrid, center: &Point, search_radius: f32) -> Vec<ClosestLine> {
    // `Point radius_vector(search_radius, search_radius)`: two floats pick `Point(double, double)`, which rounds.
    let radius_vector = Point::new((search_radius as f64).round_ties_even() as i64, (search_radius as f64).round_ties_even() as i64);
    let mut visitor = MinDistanceVisitor::new(grid, *center, (search_radius * search_radius) as f64);
    grid.visit_cells_intersecting_box(
        BoundingBox::from_points_minmax(
            Point::new(center.x - radius_vector.x, center.y - radius_vector.y),
            Point::new(center.x + radius_vector.x, center.y + radius_vector.y),
        ),
        |iy, ix| visitor.visit(iy, ix),
    );
    let mut closest_lines = visitor.closest_lines;
    // std::sort
    crate::clipper::libcxx_sort(&mut closest_lines, |l, r| {
        let dl = {
            let d = cast_d(Point::new(center.x - l.point.x, center.y - l.point.y));
            d.x * d.x + d.y * d.y
        };
        let dr = {
            let d = cast_d(Point::new(center.x - r.point.x, center.y - r.point.y));
            d.x * d.x + d.y * d.y
        };
        dl < dr
    });
    closest_lines
}

// AvoidCrossingPerimeters.cpp:172-295 — extend_for_closest_lines
// When the offset is too big, then original travel doesn't have to cross created boundaries.
// For these cases, this function adds another intersection with lines around the start and the end point of the original travel.
fn extend_for_closest_lines(
    intersections: &[Intersection],
    boundary: &Boundary,
    start: &Point,
    end: &Point,
    search_radius: f32,
) -> Vec<Intersection> {
    let start_lines = get_closest_lines_in_radius(&boundary.grid, start, search_radius);
    let end_lines = get_closest_lines_in_radius(&boundary.grid, end, search_radius);

    // AvoidCrossingPerimeters.cpp:184-187
    // Compute distance to the closest point in the ClosestLine from begin of contour.
    let compute_distance = |closest_line: &ClosestLine| -> f32 {
        // C++ uses `.cast<float>().norm()` — cast the integer Point difference to
        // f32 then compute sqrt in float precision.
        let dist_from_line_begin = {
            let dx = (closest_line.point.x
                - boundary.boundaries[closest_line.border_idx].points[closest_line.line_idx].x)
                as f32;
            let dy = (closest_line.point.y
                - boundary.boundaries[closest_line.border_idx].points[closest_line.line_idx].y)
                as f32;
            (dx * dx + dy * dy).sqrt()
        };
        boundary.boundaries_params[closest_line.border_idx][closest_line.line_idx] + dist_from_line_begin
    };

    // AvoidCrossingPerimeters.cpp:190-203
    // It tries to find closest lines for both start point and end point of the travel which has the same border_idx
    let endpoints_close_to_same_boundary = || -> (usize, usize) {
        let mut boundaries_from_start: HashSet<usize> = HashSet::new();
        for cl_start in &start_lines {
            boundaries_from_start.insert(cl_start.border_idx);
        }
        for (cl_end_idx, cl_end) in end_lines.iter().enumerate() {
            if boundaries_from_start.contains(&cl_end.border_idx) {
                for (cl_start_idx, cl_start) in start_lines.iter().enumerate() {
                    if cl_start.border_idx == cl_end.border_idx {
                        return (cl_start_idx, cl_end_idx);
                    }
                }
            }
        }
        (usize::MAX, usize::MAX)
    };

    // AvoidCrossingPerimeters.cpp:205-218
    // If the existing two lines within the search radius start and end point belong to the same boundary,
    // discard all intersection points because the whole detour could be on one boundary.
    if !start_lines.is_empty() && !end_lines.is_empty() {
        let cl_indices = endpoints_close_to_same_boundary();
        if cl_indices.0 != usize::MAX {
            debug_assert!(cl_indices.1 != usize::MAX);
            let cl_start = &start_lines[cl_indices.0];
            let cl_end = &end_lines[cl_indices.1];
            let mut new_intersections: Vec<Intersection> = Vec::new();
            new_intersections.push(Intersection {
                border_idx: cl_start.border_idx,
                line_idx: cl_start.line_idx,
                point: cl_start.point,
                distance: compute_distance(cl_start),
                do_not_remove: true,
            });
            new_intersections.push(Intersection {
                border_idx: cl_end.border_idx,
                line_idx: cl_end.line_idx,
                point: cl_end.point,
                distance: compute_distance(cl_end),
                do_not_remove: true,
            });
            return new_intersections;
        }
    }

    // AvoidCrossingPerimeters.cpp:220-230
    // Returns ClosestLine which is closer to the point "close_to" then point inside passed Intersection.
    let get_closer = |closest_lines: &[ClosestLine], intersection: &Intersection, close_to: &Point| -> usize {
        for (idx, cl) in closest_lines.iter().enumerate() {
            // Note: C++ uses `.cast<float>().squaredNorm()` (float accumulation).
            let old_dist = {
                let d = Point::new(close_to.x - intersection.point.x, close_to.y - intersection.point.y);
                (d.x as f32) * (d.x as f32) + (d.y as f32) * (d.y as f32)
            };
            let cl_dist = {
                let d = Point::new(close_to.x - cl.point.x, close_to.y - cl.point.y);
                (d.x as f32) * (d.x as f32) + (d.y as f32) * (d.y as f32)
            };
            if cl.border_idx == intersection.border_idx
                && old_dist as f64 <= (search_radius * search_radius) as f64
                && cl_dist < old_dist
            {
                return idx;
            }
        }
        usize::MAX
    };

    // AvoidCrossingPerimeters.cpp:232-258
    // Try to find ClosestLine with same boundary_idx as any existing Intersection
    let find_closest_line_with_same_boundary_idx =
        |closest_lines: &[ClosestLine], intersections: &[Intersection], reverse: bool| -> usize {
            let mut boundaries_indices: HashSet<usize> = HashSet::new();
            for closest_line in closest_lines {
                boundaries_indices.insert(closest_line.border_idx);
            }

            // This function must be called only in the case that exists closest_line with boundary_idx equals to intersection.border_idx
            let find_closest_line_index = |intersection: &Intersection| -> usize {
                for (idx, closest_line) in closest_lines.iter().enumerate() {
                    if closest_line.border_idx == intersection.border_idx {
                        return idx;
                    }
                }
                // This is an invalid state.
                debug_assert!(false);
                usize::MAX
            };

            if reverse {
                for intersection in intersections.iter().rev() {
                    if boundaries_indices.contains(&intersection.border_idx) {
                        return find_closest_line_index(intersection);
                    }
                }
            } else {
                for intersection in intersections.iter() {
                    if boundaries_indices.contains(&intersection.border_idx) {
                        return find_closest_line_index(intersection);
                    }
                }
            }
            usize::MAX
        };

    // AvoidCrossingPerimeters.cpp:260-276
    let mut new_intersections: Vec<Intersection> = intersections.to_vec();
    if !new_intersections.is_empty() && !start_lines.is_empty() {
        let cl_start_idx = get_closer(&start_lines, &new_intersections[0], start);
        if cl_start_idx != usize::MAX {
            // If there is any ClosestLine around the start point closer to the Intersection, then replace this Intersection with ClosestLine.
            let cl_start = &start_lines[cl_start_idx];
            new_intersections[0] = Intersection {
                border_idx: cl_start.border_idx,
                line_idx: cl_start.line_idx,
                point: cl_start.point,
                distance: compute_distance(cl_start),
                do_not_remove: true,
            };
        } else {
            // Check if there is any ClosestLine with the same boundary_idx as any Intersection. If this ClosestLine exists, then add it to the
            // vector of intersections. This allows in some cases when it is more than one around ClosestLine start point chose that one which
            // minimizes the number of contours (also length of the detour) in result detour. If there doesn't exist any ClosestLine like this, then
            // use the first one, which is the closest one to the start point.
            let start_closest_lines_idx =
                find_closest_line_with_same_boundary_idx(&start_lines, &new_intersections, true);
            let cl_start = if start_closest_lines_idx != usize::MAX {
                &start_lines[start_closest_lines_idx]
            } else {
                &start_lines[0]
            };
            new_intersections.insert(
                0,
                Intersection {
                    border_idx: cl_start.border_idx,
                    line_idx: cl_start.line_idx,
                    point: cl_start.point,
                    distance: compute_distance(cl_start),
                    do_not_remove: true,
                },
            );
        }
    }

    // AvoidCrossingPerimeters.cpp:278-293
    if !new_intersections.is_empty() && !end_lines.is_empty() {
        let cl_end_idx = get_closer(&end_lines, new_intersections.last().unwrap(), end);
        if cl_end_idx != usize::MAX {
            // If there is any ClosestLine around the end point closer to the Intersection, then replace this Intersection with ClosestLine.
            let cl_end = &end_lines[cl_end_idx];
            let last = new_intersections.len() - 1;
            new_intersections[last] = Intersection {
                border_idx: cl_end.border_idx,
                line_idx: cl_end.line_idx,
                point: cl_end.point,
                distance: compute_distance(cl_end),
                do_not_remove: true,
            };
        } else {
            // Check if there is any ClosestLine with the same boundary_idx as any Intersection. If this ClosestLine exists, then add it to the
            // vector of intersections. This allows in some cases when it is more than one around ClosestLine end point chose that one which
            // minimizes the number of contours (also length of the detour) in result detour. If there doesn't exist any ClosestLine like this, then
            // use the first one, which is the closest one to the end point.
            let end_closest_lines_idx =
                find_closest_line_with_same_boundary_idx(&end_lines, &new_intersections, false);
            let cl_end = if end_closest_lines_idx != usize::MAX {
                &end_lines[end_closest_lines_idx]
            } else {
                &end_lines[0]
            };
            new_intersections.push(Intersection {
                border_idx: cl_end.border_idx,
                line_idx: cl_end.line_idx,
                point: cl_end.point,
                distance: compute_distance(cl_end),
                do_not_remove: true,
            });
        }
    }
    new_intersections
}

// AvoidCrossingPerimeters.cpp:297-314 — find_first_different_vertex<forward>
// point_idx is the index from which is different vertex is searched.
fn find_first_different_vertex(polygon: &Polygon, point_idx: usize, point: &Point, forward: bool) -> Point {
    debug_assert!(point_idx < polygon.points.len());
    // Solve case when vertex on passed index point_idx is different that pass point. This helps the following code keep simple.
    if *point != polygon.points[point_idx] {
        return polygon.points[point_idx];
    }

    let mut line_idx = (point_idx as i32 + 1) % polygon.points.len() as i32;
    debug_assert!(line_idx != point_idx as i32);
    if forward {
        while *point == polygon.points[line_idx as usize] && line_idx != point_idx as i32 {
            line_idx = if line_idx + 1 < polygon.points.len() as i32 {
                line_idx + 1
            } else {
                0
            };
        }
    } else {
        while *point == polygon.points[line_idx as usize] && line_idx != point_idx as i32 {
            line_idx = if line_idx - 1 >= 0 {
                line_idx - 1
            } else {
                polygon.points.len() as i32 - 1
            };
        }
    }
    debug_assert!(*point != polygon.points[line_idx as usize]);
    polygon.points[line_idx as usize]
}

// AvoidCrossingPerimeters.cpp:316-321 — three_points_inward_normal
fn three_points_inward_normal(left: &Point, middle: &Point, right: &Point) -> Vec2d {
    debug_assert!(left != middle);
    debug_assert!(middle != right);
    let n1 = vec2d_normalized(cast_d(perp(Point::new(middle.x - left.x, middle.y - left.y))));
    let n2 = vec2d_normalized(cast_d(perp(Point::new(right.x - middle.x, right.y - middle.y))));
    vec2d_normalized(Vec2d {
        x: n1.x + n2.x,
        y: n1.y + n2.y,
    })
}

// Eigen `.normalized()` — unit vector (returns zero vector for zero input only via 0/0 -> NaN in Eigen;
// libslic3r relies on non-degenerate inputs here, matching the asserts in the callers).
#[inline]
fn vec2d_normalized(v: Vec2d) -> Vec2d {
    let n = (v.x * v.x + v.y * v.y).sqrt();
    Vec2d {
        x: v.x / n,
        y: v.y / n,
    }
}

// AvoidCrossingPerimeters.cpp:323-332 — get_polygon_vertex_inward_normal
// Compute normal of the polygon's vertex in an inward direction
fn get_polygon_vertex_inward_normal(polygon: &Polygon, point_idx: usize) -> Vec2d {
    let left_idx = prev_idx_modulo(point_idx, polygon.points.len());
    let right_idx = next_idx_modulo(point_idx, polygon.points.len());
    let middle = polygon.points[point_idx];
    let left = find_first_different_vertex(polygon, left_idx, &middle, false);
    let right = find_first_different_vertex(polygon, right_idx, &middle, true);
    three_points_inward_normal(&left, &middle, &right)
}

// AvoidCrossingPerimeters.cpp:334-338 — get_polygon_vertex_offset
// Compute offset of point_idx of the polygon in a direction of inward normal
fn get_polygon_vertex_offset(polygon: &Polygon, point_idx: usize, offset: i64) -> Point {
    let normal = get_polygon_vertex_inward_normal(polygon, point_idx);
    Point::new(
        polygon.points[point_idx].x + (normal.x * offset as f64) as i64,
        polygon.points[point_idx].y + (normal.y * offset as f64) as i64,
    )
}

// AvoidCrossingPerimeters.cpp:340-346 — get_middle_point_offset
// Compute offset (in the direction of inward normal) of the point(passed on "middle") based on the nearest points laying on the polygon (left_idx and right_idx).
fn get_middle_point_offset(
    polygon: &Polygon,
    left_idx: usize,
    right_idx: usize,
    middle: &Point,
    offset: i64,
) -> Point {
    let left = find_first_different_vertex(polygon, left_idx, middle, false);
    let right = find_first_different_vertex(polygon, right_idx, middle, true);
    let normal = three_points_inward_normal(&left, middle, &right);
    Point::new(
        middle.x + (normal.x * offset as f64) as i64,
        middle.y + (normal.y * offset as f64) as i64,
    )
}

// AvoidCrossingPerimeters.cpp:348-355 — to_polyline
fn to_polyline(travel: &[TravelPoint]) -> Polyline {
    let mut result = Polyline::new();
    result.points.reserve(travel.len());
    for t_point in travel {
        result.points.push(t_point.point);
    }
    result
}

/// The two `export_travel_to_svg` overloads of AvoidCrossingPerimeters.cpp:360-385
/// (AVOID_CROSSING_PERIMETERS_DEBUG_OUTPUT), by the type of the result travel.
#[allow(dead_code)]
trait ExportTravelToSvg {
    fn export_travel_to_svg(&self, boundary: &[Polygon], original_travel: &Line, intersections: &[Intersection], path: &str);
}

// AvoidCrossingPerimeters.cpp:360-376
// static void export_travel_to_svg(const Polygons &boundary, const Line &original_travel, const Polyline &result_travel, const std::vector<Intersection> &intersections, const std::string &path)
impl ExportTravelToSvg for Polyline {
    fn export_travel_to_svg(&self, boundary: &[Polygon], original_travel: &Line, intersections: &[Intersection], path: &str) {
        let result_travel = self;
        let bbox = get_extents(boundary);
        let mut svg = crate::svg::SVG::new_bbox_default(path, &bbox);
        svg.draw_outline_polygons(&boundary.to_vec(), "green", 0.0);
        svg.draw_line(original_travel, "blue", 0.0);
        svg.draw_polyline(result_travel, "red", 0.0);
        svg.draw_point(&original_travel.a, "black", 0);
        svg.draw_point(&original_travel.b, "grey", 0);

        for intersection in intersections {
            svg.draw_point(&intersection.point, "lightseagreen", 0);
        }
    }
}

// AvoidCrossingPerimeters.cpp:378-385
// static void export_travel_to_svg(const Polygons &boundary, const Line &original_travel, const std::vector<TravelPoint> &result_travel, const std::vector<Intersection> &intersections, const std::string &path)
impl ExportTravelToSvg for [TravelPoint] {
    fn export_travel_to_svg(&self, boundary: &[Polygon], original_travel: &Line, intersections: &[Intersection], path: &str) {
        to_polyline(self).export_travel_to_svg(boundary, original_travel, intersections, path);
    }
}

// AvoidCrossingPerimeters.cpp:388-389 — enum class Direction
// Returns a direction of the shortest path along the polygon boundary
#[derive(Clone, Copy, PartialEq, Eq)]
enum Direction {
    Forward,
    Backward,
}

// AvoidCrossingPerimeters.cpp:390-421 — get_shortest_direction
// Returns a direction of the shortest path along the polygon boundary
fn get_shortest_direction(
    boundary: &Boundary,
    intersection_first: &Intersection,
    intersection_second: &Intersection,
    contour_length: f32,
) -> Direction {
    debug_assert!(intersection_first.border_idx == intersection_second.border_idx);
    let poly = &boundary.boundaries[intersection_first.border_idx];
    let mut dist_first = intersection_first.distance;
    let mut dist_second = intersection_second.distance;

    debug_assert!(dist_first >= 0.0 && dist_first <= contour_length);
    debug_assert!(dist_second >= 0.0 && dist_second <= contour_length);

    let mut reversed = false;
    if dist_first > dist_second {
        std::mem::swap(&mut dist_first, &mut dist_second);
        reversed = true;
    }
    let mut total_length_forward = dist_second - dist_first;
    let mut total_length_backward = dist_first + contour_length - dist_second;
    if reversed {
        std::mem::swap(&mut total_length_forward, &mut total_length_backward);
    }

    // C++ uses `.cast<float>().norm()` for all four terms — f32 accumulation.
    let poly_size = poly.points.len();
    total_length_forward -= {
        let dx = (intersection_first.point.x - poly.points[intersection_first.line_idx].x) as f32;
        let dy = (intersection_first.point.y - poly.points[intersection_first.line_idx].y) as f32;
        (dx * dx + dy * dy).sqrt()
    };
    total_length_backward -= {
        let idx = (intersection_first.line_idx + 1) % poly_size;
        let dx = (poly.points[idx].x - intersection_first.point.x) as f32;
        let dy = (poly.points[idx].y - intersection_first.point.y) as f32;
        (dx * dx + dy * dy).sqrt()
    };

    total_length_forward -= {
        let idx = (intersection_second.line_idx + 1) % poly_size;
        let dx = (poly.points[idx].x - intersection_second.point.x) as f32;
        let dy = (poly.points[idx].y - intersection_second.point.y) as f32;
        (dx * dx + dy * dy).sqrt()
    };
    total_length_backward -= {
        let dx = (intersection_second.point.x - poly.points[intersection_second.line_idx].x) as f32;
        let dy = (intersection_second.point.y - poly.points[intersection_second.line_idx].y) as f32;
        (dx * dx + dy * dy).sqrt()
    };

    if total_length_forward < total_length_backward {
        return Direction::Forward;
    }
    Direction::Backward
}

// AvoidCrossingPerimeters.cpp:423-431 — ConvertBBoxToPolyline
#[allow(non_snake_case)]
pub fn ConvertBBoxToPolyline(bbox: &crate::bounding_box::BoundingBoxf) -> Polyline {
    let left_bottom = Point::new(bbox.min.x as i64, bbox.min.y as i64);
    let left_up = Point::new(bbox.min.x as i64, bbox.max.y as i64);
    let right_up = Point::new(bbox.max.x as i64, bbox.max.y as i64);
    let right_bottom = Point::new(bbox.max.x as i64, bbox.min.y as i64);

    Polyline::from_points(vec![left_bottom, right_bottom, right_up, left_up, left_bottom])
}

// AvoidCrossingPerimeters.cpp:433-477 — simplify_travel
// Straighten the travel path as long as it does not collide with the contours stored in edge_grid.
fn simplify_travel(boundary: &Boundary, travel: &[TravelPoint]) -> Vec<TravelPoint> {
    let mut simplified_path: Vec<TravelPoint> = Vec::with_capacity(travel.len());
    simplified_path.push(TravelPoint {
        point: travel[0].point,
        border_idx: travel[0].border_idx,
        do_not_remove: travel[0].do_not_remove,
    });
    // Try to skip some points in the path.
    //FIXME maybe use a binary search to trim the line?
    //FIXME how about searching tangent point at long segments?
    let mut point_idx = 1;
    while point_idx < travel.len() {
        let current_point = travel[point_idx - 1].point;
        let mut next_point = travel[point_idx].point;
        let mut next_border_idx = travel[point_idx].border_idx;
        let mut next_do_not_remove = travel[point_idx].do_not_remove;

        if !travel[point_idx].do_not_remove {
            let mut point_idx_2 = point_idx + 1;
            while point_idx_2 < travel.len() {
                if travel[point_idx_2].do_not_remove {
                    break;
                }
                if travel[point_idx_2].point == current_point {
                    next_point = travel[point_idx_2].point;
                    next_border_idx = travel[point_idx_2].border_idx;
                    next_do_not_remove = travel[point_idx_2].do_not_remove;
                    point_idx = point_idx_2;
                    point_idx_2 += 1;
                    continue;
                }

                // Check if deleting point causes crossing a boundary
                if !first_intersection_visitor_intersect(&boundary.grid, &current_point, &travel[point_idx_2].point) {
                    next_point = travel[point_idx_2].point;
                    next_border_idx = travel[point_idx_2].border_idx;
                    next_do_not_remove = travel[point_idx_2].do_not_remove;
                    point_idx = point_idx_2;
                }
                point_idx_2 += 1;
            }
        }

        simplified_path.push(TravelPoint {
            point: next_point,
            border_idx: next_border_idx,
            do_not_remove: next_do_not_remove,
        });
        point_idx += 1;
    }

    simplified_path
}

// AvoidCrossingPerimeters.cpp:479-489 — get_default_perimeter_spacing
// called by get_perimeter_spacing() / get_perimeter_spacing_external()
//
// C++ takes `const PrintObject &print_object`; the callers all reach it
// through `*layer.object()`, which in Rust is the config-only upward view
// `ObjectRef` — sufficient because the body only reads
// `print_object.print()->config().nozzle_diameter`.
fn get_default_perimeter_spacing(print_object: &crate::print_object::ObjectRef<'_>) -> f32 {
    // AvoidCrossingPerimeters.cpp:482-483
    // C++: std::vector<unsigned int> printing_extruders = print_object.object_extruders();
    //      assert(!printing_extruders.empty());
    // AvoidCrossingPerimeters.cpp:484-487
    // C++: for (unsigned int extruder_id : printing_extruders)
    //          avg_extruder += float(scale_(print_object.print()->config().nozzle_diameter.get_at(extruder_id)));
    //      avg_extruder /= printing_extruders.size();
    // This crate models `PrintConfig::nozzle_diameter` as a single-extruder
    // scalar (see print_region.rs module docs), so `get_at(extruder_id)`
    // collapses onto a direct read and the average over the asserted-non-empty
    // printing-extruder set is exactly `float(scale_(nozzle_diameter))`,
    // independent of which extruder ids `object_extruders()` would return.
    // C++ `scale_(val)` is the raw macro `((val) / SCALING_FACTOR)` (libslic3r.h:81).
    let avg_extruder =
        (print_object.print().config().nozzle_diameter / crate::libslic3r::SCALING_FACTOR) as f32;
    // AvoidCrossingPerimeters.cpp:488
    avg_extruder
}

// AvoidCrossingPerimeters.cpp:491-507 — get_perimeter_spacing
// called by get_boundary() / avoid_perimeters_inner()
fn get_perimeter_spacing(layer: &Layer) -> f32 {
    let mut regions_count: usize = 0;
    let mut perimeter_spacing: f32 = 0.0;
    // AvoidCrossingPerimeters.cpp:496-500
    // C++: for (const LayerRegion *layer_region : layer.regions())
    //          if (layer_region != nullptr && !layer_region->slices.empty())
    // (Rust LayerRegions are owned values, so the nullptr check is vacuous.)
    for layer_region in layer.regions() {
        if !layer_region.slices.is_empty() {
            // C++: perimeter_spacing += layer_region->flow(frPerimeter).scaled_spacing();
            // The one-arg C++ overload reads `m_layer->height`; the Rust
            // `flow(role, layer_height)` threads it explicitly (LayerRegion.cpp:21-23).
            perimeter_spacing += layer_region
                .flow(FlowRole::Perimeter, layer.height)
                .expect("LayerRegion::flow(frPerimeter)")
                .scaled_spacing() as f32;
            regions_count += 1;
        }
    }

    // AvoidCrossingPerimeters.cpp:502
    debug_assert!(perimeter_spacing >= 0.0);
    // AvoidCrossingPerimeters.cpp:503-506
    if regions_count != 0 {
        perimeter_spacing /= regions_count as f32;
    } else {
        perimeter_spacing = get_default_perimeter_spacing(&layer.object());
    }
    perimeter_spacing
}

/// `layer.object()->print()->objects()` as `get_perimeter_spacing_external` reads it: for every
/// object, the layer `object->get_layer_at_printz(print_z, epsilon)` when there is one. Both prints
/// of the crate provide it.
pub trait PrintObjectsLayers {
    fn layers_at_printz(&self, print_z: f64, epsilon: f64) -> Vec<&Layer>;
}

impl PrintObjectsLayers for crate::print::Print {
    fn layers_at_printz(&self, print_z: f64, epsilon: f64) -> Vec<&Layer> {
        let mut out = Vec::new();
        for object in self.objects() {
            if let Some(l_idx) = object.get_layer_at_printz(print_z, epsilon) {
                out.push(&object.layers()[l_idx]);
            }
        }
        out
    }
}

impl PrintObjectsLayers for Print {
    fn layers_at_printz(&self, print_z: f64, epsilon: f64) -> Vec<&Layer> {
        let mut out = Vec::new();
        for object in self.objects() {
            if let Some(l) = object.get_layer_at_printz(print_z, epsilon) {
                out.push(l);
            }
        }
        out
    }
}

// AvoidCrossingPerimeters.cpp:510-529 — get_perimeter_spacing_external
// called by get_boundary_external()
//
// C++ reaches the Print through the parent pointer chain
// `layer.object()->print()->objects()`; here the owning print is an argument.
fn get_perimeter_spacing_external<P: PrintObjectsLayers + ?Sized>(layer: &Layer, print: &P) -> f32 {
    let mut regions_count: usize = 0;
    let mut perimeter_spacing: f32 = 0.0;
    // AvoidCrossingPerimeters.cpp:515-521
    // C++: for (const PrintObject *object : layer.object()->print()->objects())
    //          if (const Layer *l = object->get_layer_at_printz(layer.print_z, EPSILON); l)
    for l in print.layers_at_printz(layer.print_z, crate::libslic3r::EPSILON) {
        // C++: for (const LayerRegion *layer_region : l->regions())
        //          if (layer_region != nullptr && !layer_region->slices.empty())
        for layer_region in l.regions() {
            if !layer_region.slices.is_empty() {
                // C++: perimeter_spacing += layer_region->flow(frPerimeter).scaled_spacing();
                // (one-arg overload reads l->height — that layer's own height)
                perimeter_spacing += layer_region
                    .flow(FlowRole::Perimeter, l.height)
                    .expect("LayerRegion::flow(frPerimeter)")
                    .scaled_spacing() as f32;
                regions_count += 1;
            }
        }
    }

    // AvoidCrossingPerimeters.cpp:523
    debug_assert!(perimeter_spacing >= 0.0);
    // AvoidCrossingPerimeters.cpp:524-527
    if regions_count != 0 {
        perimeter_spacing /= regions_count as f32;
    } else {
        perimeter_spacing = get_default_perimeter_spacing(&layer.object());
    }
    perimeter_spacing
}

// AvoidCrossingPerimeters.cpp:531-547 — get_external_perimeter_width
fn get_external_perimeter_width(layer: &Layer) -> f32 {
    let mut regions_count: usize = 0;
    let mut perimeter_width: f32 = 0.0;
    // AvoidCrossingPerimeters.cpp:536-540
    // C++: for (const LayerRegion *layer_region : layer.regions())
    //          if (layer_region != nullptr && !layer_region->slices.empty())
    for layer_region in layer.regions() {
        if !layer_region.slices.is_empty() {
            // C++: perimeter_width += float(layer_region->flow(frExternalPerimeter).scaled_width());
            perimeter_width += layer_region
                .flow(FlowRole::ExternalPerimeter, layer.height)
                .expect("LayerRegion::flow(frExternalPerimeter)")
                .scaled_width() as f32;
            regions_count += 1;
        }
    }

    // AvoidCrossingPerimeters.cpp:542
    debug_assert!(perimeter_width >= 0.0);
    // AvoidCrossingPerimeters.cpp:543-546
    if regions_count != 0 {
        perimeter_width /= regions_count as f32;
    } else {
        perimeter_width = get_default_perimeter_spacing(&layer.object());
    }
    perimeter_width
}

// AvoidCrossingPerimeters.cpp:549-688 — avoid_perimeters_inner
fn avoid_perimeters_inner(
    boundary: &Boundary,
    start_point: &Point,
    end_point: &Point,
    layer: &Layer,
    result_out: &mut Vec<TravelPoint>,
) -> usize {
    let boundaries = &boundary.boundaries;
    let edge_grid = &boundary.grid;
    let mut start = *start_point;
    let mut end = *end_point;
    // Find all intersections between boundaries and the line segment, sort them along the line segment.
    let mut intersections: Vec<Intersection>;
    {
        let visitor = AllIntersectionsVisitor::with_line(edge_grid, Line::new(start, end));
        intersections = visitor.run();
        let mut dir = cast_d(Point::new(end.x - start.x, end.y - start.y));
        // if do not intersect due to the boundaries inner-offset, try to find the closest point to do intersect again!
        if intersections.is_empty() {
            // try to find the closest point on boundaries to start/end with distance less than extend_distance, which is noted as new start_point/end_point
            // C++: auto search_radius = 1.5 * get_perimeter_spacing(layer); (double)
            let search_radius = 1.5 * get_perimeter_spacing(layer) as f64;
            let closest_line_to_start = get_closest_lines_in_radius(&boundary.grid, &start, search_radius as f32);
            let closest_line_to_end = get_closest_lines_in_radius(&boundary.grid, &end, search_radius as f32);
            if !(closest_line_to_start.is_empty() && closest_line_to_end.is_empty()) {
                let new_start_point0 = if closest_line_to_start.is_empty() {
                    start
                } else {
                    closest_line_to_start[0].point
                };
                let new_end_point0 = if closest_line_to_end.is_empty() {
                    end
                } else {
                    closest_line_to_end[0].point
                };
                dir = cast_d(Point::new(
                    new_end_point0.x - new_start_point0.x,
                    new_end_point0.y - new_start_point0.y,
                ));
                let unit_direction = vec2d_normalized(dir);
                // out-offset new_start_point/new_end_point epsilon along the Line(new_start_point, new_end_point) for right intersection!
                let eps = SCALED_EPSILON as i64 as f64;
                let new_start_point = Point::new(
                    new_start_point0.x - (unit_direction.x * eps) as i64,
                    new_start_point0.y - (unit_direction.y * eps) as i64,
                );
                let new_end_point = Point::new(
                    new_end_point0.x + (unit_direction.x * eps) as i64,
                    new_end_point0.y + (unit_direction.y * eps) as i64,
                );
                let visitor =
                    AllIntersectionsVisitor::with_line(edge_grid, Line::new(new_start_point, new_end_point));
                intersections = visitor.run();
                if !intersections.is_empty() {
                    start = new_start_point;
                    end = new_end_point;
                }
            }
        }

        for intersection in intersections.iter_mut() {
            // C++ uses `.cast<float>().norm()` — f32 accumulation.
            let dist_from_line_begin = {
                let dx = (intersection.point.x
                    - boundary.boundaries[intersection.border_idx].points[intersection.line_idx].x)
                    as f32;
                let dy = (intersection.point.y
                    - boundary.boundaries[intersection.border_idx].points[intersection.line_idx].y)
                    as f32;
                (dx * dx + dy * dy).sqrt()
            };
            intersection.distance =
                boundary.boundaries_params[intersection.border_idx][intersection.line_idx] + dist_from_line_begin;
        }
        // std::sort with `(r.point - l.point).cast<double>().dot(dir) > 0.`
        crate::clipper::libcxx_sort(&mut intersections, |l, r| {
            let v = cast_d(Point::new(r.point.x - l.point.x, r.point.y - l.point.y));
            v.x * dir.x + v.y * dir.y > 0.0
        });

        // Search radius should always be at least equals to the value of offset used for computing boundaries.
        // C++: const float search_radius = 2.0f * get_perimeter_spacing(layer);
        let search_radius = 2.0 * get_perimeter_spacing(layer);
        // When the offset is too big, then original travel doesn't have to cross created boundaries.
        // These cases are fixed by calling extend_for_closest_lines.
        intersections = extend_for_closest_lines(&intersections, boundary, &start, &end, search_radius);
    }

    let mut result: Vec<TravelPoint> = Vec::new();
    result.push(TravelPoint {
        point: start,
        border_idx: -1,
        do_not_remove: false,
    });

    // AvoidCrossingPerimeters.cpp:610-660
    let mut it_first = 0usize;
    while it_first < intersections.len() {
        // The entry point to the boundary polygon
        let intersection_first = intersections[it_first].clone();
        // Search for the farthest intersection different from it_first but with the same border_idx
        // (C++: std::find_if over the reverse range [rbegin, make_reverse_iterator(it_first) - 1)).
        let mut it_second: Option<usize> = None;
        let mut j = intersections.len();
        while j > it_first + 1 {
            j -= 1;
            if intersection_first.border_idx == intersections[j].border_idx {
                it_second = Some(j);
                break;
            }
        }

        // Append the first intersection into the path
        let left_idx = intersection_first.line_idx;
        let right_idx = if intersection_first.line_idx + 1 == boundaries[intersection_first.border_idx].points.len() {
            0
        } else {
            intersection_first.line_idx + 1
        };
        // Offset of the polygon's point using get_middle_point_offset is used to simplify the calculation of intersection between the
        // boundary and the travel. The appended point is translated in the direction of inward normal. This translation ensures that the
        // appended point will be inside the polygon and not on the polygon border.
        result.push(TravelPoint {
            point: get_middle_point_offset(
                &boundaries[intersection_first.border_idx],
                left_idx,
                right_idx,
                &intersection_first.point,
                SCALED_EPSILON as i64,
            ),
            border_idx: intersection_first.border_idx as i32,
            do_not_remove: intersection_first.do_not_remove,
        });

        // Check if intersection line also exit the boundary polygon
        if let Some(it_second_idx) = it_second {
            // The exit point from the boundary polygon
            let intersection_second = intersections[it_second_idx].clone();
            let shortest_direction = get_shortest_direction(
                boundary,
                &intersection_first,
                &intersection_second,
                *boundary.boundaries_params[intersection_first.border_idx].last().unwrap(),
            );
            // Append the path around the border into the path
            if shortest_direction == Direction::Forward {
                let mut line_idx = intersection_first.line_idx as i32;
                while line_idx != intersection_second.line_idx as i32 {
                    let bsize = boundaries[intersection_first.border_idx].points.len() as i32;
                    let vtx_idx = if line_idx + 1 == bsize { 0 } else { line_idx + 1 } as usize;
                    result.push(TravelPoint {
                        point: get_polygon_vertex_offset(
                            &boundaries[intersection_first.border_idx],
                            vtx_idx,
                            SCALED_EPSILON as i64,
                        ),
                        border_idx: intersection_first.border_idx as i32,
                        do_not_remove: false,
                    });
                    line_idx = if line_idx + 1 < bsize { line_idx + 1 } else { 0 };
                }
            } else {
                let mut line_idx = intersection_first.line_idx as i32;
                while line_idx != intersection_second.line_idx as i32 {
                    result.push(TravelPoint {
                        point: get_polygon_vertex_offset(
                            &boundaries[intersection_second.border_idx],
                            (line_idx + 0) as usize,
                            SCALED_EPSILON as i64,
                        ),
                        border_idx: intersection_first.border_idx as i32,
                        do_not_remove: false,
                    });
                    let bsize = boundaries[intersection_first.border_idx].points.len() as i32;
                    line_idx = if line_idx - 1 >= 0 { line_idx - 1 } else { bsize - 1 };
                }
            }

            // Append the farthest intersection into the path
            let left_idx = intersection_second.line_idx;
            let right_idx = if intersection_second.line_idx >= (boundaries[intersection_second.border_idx].points.len() - 1) {
                0
            } else {
                intersection_second.line_idx + 1
            };
            result.push(TravelPoint {
                point: get_middle_point_offset(
                    &boundaries[intersection_second.border_idx],
                    left_idx,
                    right_idx,
                    &intersection_second.point,
                    SCALED_EPSILON as i64,
                ),
                border_idx: intersection_second.border_idx as i32,
                do_not_remove: intersection_second.do_not_remove,
            });
            // Skip intersections in between
            it_first = it_second_idx;
        }
        it_first += 1;
    }

    result.push(TravelPoint {
        point: end,
        border_idx: -1,
        do_not_remove: false,
    });

    let _result_polyline = to_polyline(&result);

    if !intersections.is_empty() {
        result = simplify_travel(boundary, &result);
    }

    let _simplified_result_polyline = to_polyline(&result);

    // append(result_out, std::move(result));
    let n = intersections.len();
    result_out.append(&mut result);
    n
}

// AvoidCrossingPerimeters.cpp:690-711 — avoid_perimeters
// Called by AvoidCrossingPerimeters::travel_to()
#[allow(dead_code)] // sole C++ caller AvoidCrossingPerimeters::travel_to() is BLOCKED (see module docs)
fn avoid_perimeters(
    boundary: &Boundary,
    start: &Point,
    end: &Point,
    layer: &Layer,
    result_out: &mut Polyline,
) -> usize {
    // Travel line is completely or partially inside the bounding box.
    let mut path: Vec<TravelPoint> = Vec::new();
    let num_intersections = avoid_perimeters_inner(boundary, start, end, layer, &mut path);
    *result_out = to_polyline(&path);

    num_intersections
}

/// The two `any_expolygon_contains` overloads of AvoidCrossingPerimeters.cpp:716-766, by the type of
/// the travel.
trait AnyExpolygonContains {
    fn any_expolygon_contains(
        &self,
        ex_polygons: &[crate::geometry::ExPolygon],
        ex_polygons_bboxes: &[BoundingBox],
        grid_lslice: &EdgeGrid,
    ) -> bool;
}

// AvoidCrossingPerimeters.cpp:716-737 — any_expolygon_contains (Line)
// Check if anyone of ExPolygons contains whole travel.
// called by need_wipe() and AvoidCrossingPerimeters::travel_to()
impl AnyExpolygonContains for Line {
    fn any_expolygon_contains(
        &self,
        ex_polygons: &[crate::geometry::ExPolygon],
        ex_polygons_bboxes: &[BoundingBox],
        grid_lslice: &EdgeGrid,
    ) -> bool {
        let travel = self;
        debug_assert!(ex_polygons.len() == ex_polygons_bboxes.len());
        if !grid_lslice.bbox().contains_point(&travel.a) || !grid_lslice.bbox().contains_point(&travel.b) {
            return false;
        }

        let intersect = first_intersection_visitor_intersect(grid_lslice, &travel.a, &travel.b);
        if !intersect {
            for (idx, ex_polygon) in ex_polygons.iter().enumerate() {
                let bbox = &ex_polygons_bboxes[idx];
                if bbox.contains_point(&travel.a) && bbox.contains_point(&travel.b) && ex_polygon.contains(&travel.a, true) {
                    return true;
                }
            }
        }
        false
    }
}

// AvoidCrossingPerimeters.cpp:741-766 — any_expolygon_contains (Polyline)
// Check if anyone of ExPolygons contains whole travel.
// called by need_wipe()
impl AnyExpolygonContains for Polyline {
    fn any_expolygon_contains(
        &self,
        ex_polygons: &[crate::geometry::ExPolygon],
        ex_polygons_bboxes: &[BoundingBox],
        grid_lslice: &EdgeGrid,
    ) -> bool {
        let travel = self;
        debug_assert!(ex_polygons.len() == ex_polygons_bboxes.len());
        if travel.points.iter().any(|point| !grid_lslice.bbox().contains_point(point)) {
            return false;
        }

        let mut any_intersection = false;
        for line_idx in 1..travel.points.len() {
            let pt_current = travel.points[line_idx - 1];
            let pt_next = travel.points[line_idx];
            any_intersection = first_intersection_visitor_intersect(grid_lslice, &pt_current, &pt_next);
            if any_intersection {
                break;
            }
        }

        if !any_intersection {
            for (idx, ex_polygon) in ex_polygons.iter().enumerate() {
                let bbox = &ex_polygons_bboxes[idx];
                if travel.points.iter().all(|point| bbox.contains_point(point)) && ex_polygon.contains(&travel.points[0], true) {
                    return true;
                }
            }
        }
        false
    }
}

// AvoidCrossingPerimeters.cpp:768-799 — need_wipe
fn need_wipe(
    gcodegen: &GCode<'_>,
    lslices_offset: &[crate::geometry::ExPolygon],
    lslices_offset_bboxes: &[BoundingBox],
    grid_lslices_offset: &EdgeGrid,
    original_travel: &Line,
    result_travel: &Polyline,
    intersection_count: usize,
) -> bool {
    let filament_id = gcodegen.writer().filament().expect("need_wipe: no filament").id() as usize;
    let z_lift_enabled = get_at(&cfg_floats(gcodegen.config(), "z_hop"), filament_id) > 0.0;
    let mut wipe_needed = false;

    // If the original unmodified path doesn't have any intersection with boundary, then it is entirely inside the object otherwise is entirely
    // outside the object.
    if intersection_count > 0 {
        // The original layer is intersected with defined boundaries. Then it is necessary to make a detailed test.
        // If the z-lift is enabled, then a wipe is needed when the original travel leads above the holes.
        if z_lift_enabled {
            if original_travel.any_expolygon_contains(lslices_offset, lslices_offset_bboxes, grid_lslices_offset) {
                // Check if original_travel and result_travel are not same.
                // If both are the same, then it is possible to skip testing of result_travel
                wipe_needed = !(result_travel.points.len() > 2
                    && result_travel.points[0] == original_travel.a
                    && *result_travel.points.last().unwrap() == original_travel.b)
                    && !result_travel.any_expolygon_contains(lslices_offset, lslices_offset_bboxes, grid_lslices_offset);
            } else {
                wipe_needed = true;
            }
        } else {
            wipe_needed = !result_travel.any_expolygon_contains(lslices_offset, lslices_offset_bboxes, grid_lslices_offset);
        }
    }

    wipe_needed
}

// AvoidCrossingPerimeters.cpp:801-833 — resample_polygon
// Adds points around all vertices so that the offset affects only small sections around these vertices.
fn resample_polygon(polygon: &mut Polygon, dist_from_vertex: f64, max_allowed_distance: f64) {
    let mut resampled_poly: Vec<Point> = Vec::with_capacity(3 * polygon.points.len());
    let n = polygon.points.len();
    for pt_idx in 0..n {
        resampled_poly.push(polygon.points[pt_idx]);

        let p1 = polygon.points[pt_idx];
        let p2 = polygon.points[next_idx_modulo(pt_idx, n)];
        let line_vec = cast_d(Point::new(p2.x - p1.x, p2.y - p1.y));
        let line_length = (line_vec.x * line_vec.x + line_vec.y * line_vec.y).sqrt();
        let line_vec_norm = vec2d_normalized(line_vec);
        let vertex_offset_vec = Point::new(
            (line_vec_norm.x * dist_from_vertex) as i64,
            (line_vec_norm.y * dist_from_vertex) as i64,
        );
        if line_length > 2.0 * dist_from_vertex && vertex_offset_vec != Point::new(0, 0) {
            resampled_poly.push(Point::new(p1.x + vertex_offset_vec.x, p1.y + vertex_offset_vec.y));

            let new_vertex_vec = cast_d(Point::new(
                p2.x - p1.x - 2 * vertex_offset_vec.x,
                p2.y - p1.y - 2 * vertex_offset_vec.y,
            ));
            let new_vertex_vec_length = (new_vertex_vec.x * new_vertex_vec.x + new_vertex_vec.y * new_vertex_vec.y).sqrt();
            if new_vertex_vec_length > max_allowed_distance {
                let prev_point = cast_d(*resampled_poly.last().unwrap());
                let parts_count = (new_vertex_vec_length / max_allowed_distance).ceil() as usize;
                for part_idx in 1..parts_count {
                    let part_param = part_idx as f64 / parts_count as f64;
                    let new_point = Vec2d {
                        x: prev_point.x + new_vertex_vec.x * part_param,
                        y: prev_point.y + new_vertex_vec.y * part_param,
                    };
                    resampled_poly.push(Point::new(new_point.x as i64, new_point.y as i64));
                }
            }

            resampled_poly.push(Point::new(p2.x - vertex_offset_vec.x, p2.y - vertex_offset_vec.y));
        }
    }
    polygon.points = resampled_poly;
}

// AvoidCrossingPerimeters.cpp:835-840 — resample_expolygon
fn resample_expolygon(ex_polygon: &mut crate::geometry::ExPolygon, dist_from_vertex: f64, max_allowed_distance: f64) {
    resample_polygon(&mut ex_polygon.contour, dist_from_vertex, max_allowed_distance);
    for polygon in ex_polygon.holes.iter_mut() {
        resample_polygon(polygon, dist_from_vertex, max_allowed_distance);
    }
}

// AvoidCrossingPerimeters.cpp:842-846 — resample_expolygons
fn resample_expolygons(ex_polygons: &mut [crate::geometry::ExPolygon], dist_from_vertex: f64, max_allowed_distance: f64) {
    for ex_poly in ex_polygons.iter_mut() {
        resample_expolygon(ex_poly, dist_from_vertex, max_allowed_distance);
    }
}

// AvoidCrossingPerimeters.cpp:848-854 — precompute_polygon_distances
fn precompute_polygon_distances(polygon: &Polygon, polygon_distances_out: &mut Vec<f32>) {
    polygon_distances_out.clear();
    polygon_distances_out.resize(polygon.points.len() + 1, 0.0);
    let n = polygon.points.len();
    for point_idx in 1..n {
        let d = cast_d(Point::new(
            polygon.points[point_idx].x - polygon.points[point_idx - 1].x,
            polygon.points[point_idx].y - polygon.points[point_idx - 1].y,
        ));
        polygon_distances_out[point_idx] =
            polygon_distances_out[point_idx - 1] + (d.x * d.x + d.y * d.y).sqrt() as f32;
    }
    let d = cast_d(Point::new(
        polygon.points[n - 1].x - polygon.points[0].x,
        polygon.points[n - 1].y - polygon.points[0].y,
    ));
    let last = polygon_distances_out.len() - 1;
    polygon_distances_out[last] = polygon_distances_out[n - 1] + (d.x * d.x + d.y * d.y).sqrt() as f32;
}

// AvoidCrossingPerimeters.cpp:856-862 — precompute_expolygon_distances
fn precompute_expolygon_distances(
    ex_polygon: &crate::geometry::ExPolygon,
    expolygon_distances_out: &mut Vec<Vec<f32>>,
) {
    expolygon_distances_out.clear();
    expolygon_distances_out.resize(ex_polygon.holes.len() + 1, Vec::new());
    precompute_polygon_distances(&ex_polygon.contour, &mut expolygon_distances_out[0]);
    for hole_idx in 0..ex_polygon.holes.len() {
        precompute_polygon_distances(&ex_polygon.holes[hole_idx], &mut expolygon_distances_out[hole_idx + 1]);
    }
}

/// libslic3r.h:84 `#define SCALED_EPSILON scale_(EPSILON)` as the double the C++ computes with:
/// 1e-4 / 0.00001.
const SCALED_EPSILON_D: f64 = crate::libslic3r::EPSILON / crate::libslic3r::SCALING_FACTOR;

/// `ClipperUtils::SinglePathProvider(polygon.points)`
fn polygon_path(polygon: &Polygon) -> Path {
    polygon.points.iter().map(|p| crate::clipper::IntPoint::new(p.x as i32, p.y as i32)).collect()
}

/// `ClipperUtils::PolygonsProvider(polygons)`
fn polygons_paths(polygons: &[Polygon]) -> Paths {
    polygons.iter().map(polygon_path).collect()
}

/// `ClipperUtils::ExPolygonsProvider(expolygons)`: every contour, then its holes.
fn expolygons_paths(expolygons: &[ExPolygon]) -> Paths {
    let mut out = Paths::new();
    for expoly in expolygons {
        out.push(polygon_path(&expoly.contour));
        for hole in &expoly.holes {
            out.push(polygon_path(hole));
        }
    }
    out
}

/// ClipperUtils.cpp `to_polygons(ClipperLib::Paths &&paths)`
fn paths_polygons(paths: &Paths) -> Vec<Polygon> {
    paths.iter().map(|p| Polygon::from_points(p.iter().map(|q| Point::new(q.x as i64, q.y as i64)).collect())).collect()
}

/// ClipperUtils.cpp:815-816 `ExPolygons union_ex(const ExPolygons &subject)`
fn union_ex(subject: &[ExPolygon]) -> Vec<ExPolygon> {
    poly_tree_to_ex_polygons(&clipper_do_polytree(ClipType::ctUnion, &expolygons_paths(subject), &Paths::new(), PolyFillType::pftNonZero))
}

/// ClipperUtils.cpp:813-814 `ExPolygons union_ex(const Polygons &subject, ClipperLib::PolyFillType fill_type = pftNonZero)`
fn union_ex_polygons(subject: &[Polygon]) -> Vec<ExPolygon> {
    _clipper_ex(ClipType::ctUnion, &polygons_paths(subject), &Paths::new(), ApplySafetyOffset::No, PolyFillType::pftNonZero)
}

/// ClipperUtils.cpp:570-571 `ExPolygons offset_ex(const ExPolygons &expolygons, const float delta)`, with
/// the default join type and miter limit.
fn offset_ex(expolygons: &[ExPolygon], delta: f32) -> Vec<ExPolygon> {
    let expolys: Vec<(Path, Paths)> = expolygons.iter().map(|e| (polygon_path(&e.contour), polygons_paths(&e.holes))).collect();
    poly_tree_to_ex_polygons(&expolygons_offset_pt(&expolys, delta as f64, JoinType::jtMiter, 3.0))
}

/// ClipperUtils.cpp:415-416 `ExPolygons offset_ex(const Polygons &polygons, const float delta)`
fn offset_ex_polygons(polygons: &[Polygon], delta: f32) -> Vec<ExPolygon> {
    poly_tree_to_ex_polygons(&offset_paths::<PolyTree>(&polygons_paths(polygons), delta as f64, JoinType::jtMiter, 3.0))
}

/// ClipperUtils.cpp:413-414 `Polygons offset(const Polygons &polygons, const float delta)`
fn offset_polygons(polygons: &[Polygon], delta: f32) -> Vec<Polygon> {
    paths_polygons(&offset_paths::<Paths>(&polygons_paths(polygons), delta as f64, JoinType::jtMiter, 3.0))
}

/// ClipperUtils.cpp:410-411 `Polygons offset(const Polygon &polygon, const float delta)`
fn offset_polygon(polygon: &Polygon, delta: f32) -> Vec<Polygon> {
    paths_polygons(&raw_offset(&vec![polygon_path(polygon)], delta as f64, JoinType::jtMiter, 3.0, crate::clipper::EndType::etClosedPolygon))
}

/// ClipperUtils.cpp:755-756 `ExPolygons diff_ex(const ExPolygons &subject, const ExPolygons &clip)`
fn diff_ex(subject: &[ExPolygon], clip: &[ExPolygon]) -> Vec<ExPolygon> {
    _clipper_ex(ClipType::ctDifference, &expolygons_paths(subject), &expolygons_paths(clip), ApplySafetyOffset::No, PolyFillType::pftNonZero)
}

/// ExPolygon.hpp `to_polygons(const ExPolygons &src)`: every contour, then its holes.
fn to_polygons(src: &[ExPolygon]) -> Vec<Polygon> {
    let mut polygons: Vec<Polygon> = Vec::new();
    for ex_poly in src {
        polygons.push(ex_poly.contour.clone());
        polygons.extend(ex_poly.holes.iter().cloned());
    }
    polygons
}

/// The visitor of `contour_distance` (AvoidCrossingPerimeters.cpp:879-996).
struct ContourDistanceVisitor<'a> {
    grid: &'a EdgeGrid,
    idx_contour: usize,
    boundary_parameters: &'a [f32],
    dist_same_contour_accept: f64,
    dist_same_contour_reject: f64,

    idx_point: usize,
    point: Point,
    // Direction inside the contour from idx_point, not normalized.
    dir_inside: Vec2d,
    found: bool,
    distance: f64,
}

impl<'a> ContourDistanceVisitor<'a> {
    // AvoidCrossingPerimeters.cpp:883-890 — void init(const Points &contour, const Point &apoint)
    fn init(&mut self, contour: &[Point], idx_point: usize) {
        self.idx_point = idx_point;
        self.point = contour[idx_point];
        self.found = false;
        self.dir_inside = Self::dir_inside_at_point(contour, self.idx_point);
        self.distance = f64::MAX;
    }

    // AvoidCrossingPerimeters.cpp:892-949 — bool operator()(coord_t iy, coord_t ix)
    fn visit(&mut self, iy: usize, ix: usize) -> bool {
        // Called with a row and colum of the grid cell, which is intersected by a line.
        let grid = self.grid;
        let cell_data_range = grid.cell_data_range_at(iy, ix);
        for &it_contour_and_segment in cell_data_range {
            // End points of the line segment and their vector.
            let segment = grid.segment(it_contour_and_segment);
            let v = cast_d(Point::new(segment.b.x - segment.a.x, segment.b.y - segment.a.y));
            let va = cast_d(Point::new(self.point.x - segment.a.x, self.point.y - segment.a.y));
            let l2: f64 = v.x * v.x + v.y * v.y; // avoid a sqrt
            let t: f64 = if l2 == 0.0 { 0.0 } else { ((va.x * v.x + va.y * v.y) / l2).clamp(0.0, 1.0) };
            // Closest point from this->point to the segment.
            let foot = Vec2d { x: segment.a.x as f64 + t * v.x, y: segment.a.y as f64 + t * v.y };
            let bisector = Vec2d { x: foot.x - self.point.x as f64, y: foot.y - self.point.y as f64 };
            let dist: f64 = (bisector.x * bisector.x + bisector.y * bisector.y).sqrt();

            if (!self.found || dist < self.distance) && (self.dir_inside.x * bisector.x + self.dir_inside.y * bisector.y) > 0.0 {
                let mut accept = true;
                if it_contour_and_segment.0 == self.idx_contour {
                    // Complex case: The closest segment originates from the same contour as the starting point.
                    // Reject the closest point if its distance along the contour is reasonable compared to the current contour bisector
                    // (this->pt, foot).
                    let contour = &grid.contours()[it_contour_and_segment.0];
                    let mut param_lo: f64 = self.boundary_parameters[self.idx_point] as f64;
                    let mut param_hi: f64 = t * l2.sqrt();
                    let param_end: f64 = *self.boundary_parameters.last().unwrap() as f64;
                    let ipt: usize = it_contour_and_segment.1;
                    if ipt + 1 < contour.points().len() {
                        param_hi += self.boundary_parameters[ipt] as f64;
                    }
                    if param_lo > param_hi {
                        std::mem::swap(&mut param_lo, &mut param_hi);
                    }
                    debug_assert!(param_lo > -SCALED_EPSILON_D && param_lo <= param_end + SCALED_EPSILON_D);
                    debug_assert!(param_hi > -SCALED_EPSILON_D && param_hi <= param_end + SCALED_EPSILON_D);
                    let dist_along_contour: f64 = (param_hi - param_lo).min(param_lo + param_end - param_hi);
                    if dist_along_contour < self.dist_same_contour_accept {
                        accept = false;
                    } else if dist < self.dist_same_contour_reject + SCALED_EPSILON_D {
                        // this->point is close to foot. This point will only be accepted if the path along the contour is significantly
                        // longer than the bisector. That is, the path shall not bulge away from the bisector too much.
                        // Bulge is estimated by 0.6 of the circle circumference drawn around the bisector.
                        // Test whether the contour is convex or concave.
                        let inside = if t == 0.0 {
                            Self::inside_corner(contour, ipt, &self.point)
                        } else if t == 1.0 {
                            Self::inside_corner(contour, contour.segment_idx_next(ipt), &self.point)
                        } else {
                            Self::left_of_segment(contour, ipt, &self.point)
                        };
                        accept = inside && dist_along_contour > 0.6 * std::f64::consts::PI * dist;
                    }
                }
                if accept && (!self.found || dist < self.distance) {
                    // Simple case: Just measure the shortest distance.
                    self.distance = dist;
                    self.found = true;
                }
            }
        }
        // Continue traversing the grid.
        true
    }

    /// `Slic3r::cross2(v1, v2)`: `v1.x() * v2.y() - v1.y() * v2.x()`, contracted.
    #[inline]
    fn cross2(v1: Vec2d, v2: Vec2d) -> f64 {
        v1.x.mul_add(v2.y, -(v1.y * v2.x))
    }

    // AvoidCrossingPerimeters.cpp:966-973 — static Vec2d dir_inside_at_point(const Points &contour, size_t i)
    fn dir_inside_at_point(contour: &[Point], i: usize) -> Vec2d {
        let iprev = prev_idx_modulo(i, contour.len());
        let inext = next_idx_modulo(i, contour.len());
        let v1 = cast_d(Point::new(contour[i].x - contour[iprev].x, contour[i].y - contour[iprev].y));
        let v2 = cast_d(Point::new(contour[inext].x - contour[i].x, contour[inext].y - contour[i].y));
        Vec2d { x: -v1.y - v2.y, y: v1.x + v2.x }
    }

    // AvoidCrossingPerimeters.cpp:975-987 — static bool inside_corner(const EdgeGrid::Contour &contour, size_t i, const Point &pt_oposite)
    fn inside_corner(contour: &crate::edge_grid::Contour, i: usize, pt_oposite: &Point) -> bool {
        let pt = cast_d(*pt_oposite);
        let pt_prev = *contour.segment_prev(i);
        let pt_this = *contour.segment_start(i);
        let pt_next = *contour.segment_end(i);
        let v1 = cast_d(Point::new(pt_this.x - pt_prev.x, pt_this.y - pt_prev.y));
        let v2 = cast_d(Point::new(pt_next.x - pt_this.x, pt_next.y - pt_this.y));
        let left_of_v1 = Self::cross2(v1, Vec2d { x: pt.x - pt_prev.x as f64, y: pt.y - pt_prev.y as f64 }) > 0.0;
        let left_of_v2 = Self::cross2(v2, Vec2d { x: pt.x - pt_this.x as f64, y: pt.y - pt_this.y as f64 }) > 0.0;
        if Self::cross2(v1, v2) > 0.0 {
            left_of_v1 && left_of_v2 // convex corner
        } else {
            left_of_v1 || left_of_v2 // concave corner
        }
    }

    // AvoidCrossingPerimeters.cpp:989-996 — static bool left_of_segment(const EdgeGrid::Contour &contour, size_t i, const Point &pt_oposite)
    fn left_of_segment(contour: &crate::edge_grid::Contour, i: usize, pt_oposite: &Point) -> bool {
        let pt = cast_d(*pt_oposite);
        let pt_this = *contour.segment_start(i);
        let pt_next = *contour.segment_end(i);
        let v = cast_d(Point::new(pt_next.x - pt_this.x, pt_next.y - pt_this.y));
        Self::cross2(v, Vec2d { x: pt.x - pt_this.x as f64, y: pt.y - pt_this.y as f64 }) > 0.0
    }
}

// AvoidCrossingPerimeters.cpp:865-1009 — contour_distance
// It is highly based on the function contour_distance2 from the ElephantFootCompensation.cpp
fn contour_distance(
    grid: &EdgeGrid,
    poly_distances: &[f32],
    contour_idx: usize,
    polygon: &Polygon,
    compensation: f64,
    search_radius: f64,
) -> Vec<f32> {
    debug_assert!(!polygon.points.is_empty());
    debug_assert!(polygon.points.len() >= 2);

    let mut out: Vec<f32> = Vec::new();

    if polygon.points.len() > 2 {
        let mut visitor = ContourDistanceVisitor {
            grid,
            idx_contour: contour_idx,
            boundary_parameters: poly_distances,
            dist_same_contour_accept: 0.5 * compensation * std::f64::consts::PI,
            dist_same_contour_reject: search_radius,
            idx_point: 0,
            point: Point::new(0, 0),
            dir_inside: Vec2d { x: 0.0, y: 0.0 },
            found: false,
            distance: 0.0,
        };

        out.reserve(polygon.points.len());
        // `Point radius_vector(search_radius, search_radius)`: the constructor of two doubles rounds.
        let radius_vector = Point::new(search_radius.round_ties_even() as i64, search_radius.round_ties_even() as i64);
        for idx_point in 0..polygon.points.len() {
            let pt = polygon.points[idx_point];
            visitor.init(&polygon.points, idx_point);
            grid.visit_cells_intersecting_box(
                BoundingBox::from_points_minmax(
                    Point::new(pt.x - radius_vector.x, pt.y - radius_vector.y),
                    Point::new(pt.x + radius_vector.x, pt.y + radius_vector.y),
                ),
                |iy, ix| visitor.visit(iy, ix),
            );
            out.push(if visitor.found { visitor.distance.min(search_radius) } else { search_radius } as f32);
        }
    }

    out
}

/// AvoidCrossingPerimeters.cpp:1040-1088 and 1205-1246: the loop over `min_contour_width_values` that
/// `inner_offset` and `compute_object_boundary_local` both run on one expolygon.
fn offset_ex_poly_by_contour_widths(ex_poly: &mut ExPolygon, bbox: &BoundingBox, offset_dis: f64, min_contour_width_values: &[f64]) {
    for (min_contour_width_idx, &min_contour_width) in min_contour_width_values.iter().enumerate() {
        let search_radius: f64 = 2.0 * (offset_dis + min_contour_width);

        let mut grid = EdgeGrid::new();
        grid.set_bbox(bbox.clone());
        grid.create_from_expolygon(ex_poly, (0.7 * search_radius) as i64);

        let mut ex_poly_distances: Vec<Vec<f32>> = Vec::new();
        precompute_expolygon_distances(ex_poly, &mut ex_poly_distances);

        let mut offsets: Vec<Vec<f32>> = Vec::new();
        offsets.reserve(ex_poly.holes.len() + 1);
        for idx_contour in 0..=ex_poly.holes.len() {
            let poly = if idx_contour == 0 { &ex_poly.contour } else { &ex_poly.holes[idx_contour - 1] };
            let mut distances: Vec<f32> = contour_distance(&grid, &ex_poly_distances[idx_contour], idx_contour, poly, offset_dis, search_radius);
            for distance in distances.iter_mut() {
                if (*distance as f64) < min_contour_width {
                    *distance = 0.0f32;
                } else if (*distance as f64) > 2.0f64.mul_add(offset_dis, min_contour_width) {
                    *distance = -(offset_dis as f32);
                } else {
                    *distance = -(*distance - min_contour_width as f32) / 2.0f32;
                }
            }
            offsets.push(distances);
        }

        let mut offset_ex_poly: Vec<ExPolygon> = variable_offset_inner_ex_clib(ex_poly, &offsets, 2.0);
        // If variable_offset_inner_ex produces empty result, then original ex_polygon is used
        if offset_ex_poly.len() == 1 && offset_ex_poly[0].holes.len() == ex_poly.holes.len() {
            *ex_poly = offset_ex_poly.swap_remove(0);
            break;
        } else if (min_contour_width_idx + 1) < min_contour_width_values.len() {
            continue; // Try the next round with bigger min_contour_width.
        } else if offset_ex_poly.len() == 1 {
            *ex_poly = offset_ex_poly.swap_remove(0);
            break;
        } else if offset_ex_poly.len() > 1 {
            // fix_after_inner_offset called inside variable_offset_inner_ex sometimes produces
            // tiny artefacts polygons, so these artefacts are removed.
            let mut max_area: f64 = offset_ex_poly[0].area();
            let mut max_area_idx: usize = 0;
            for poly_idx in 1..offset_ex_poly.len() {
                let area: f64 = offset_ex_poly[poly_idx].area();
                if max_area < area {
                    max_area = area;
                    max_area_idx = poly_idx;
                }
            }
            *ex_poly = offset_ex_poly.swap_remove(max_area_idx);
            break;
        }
    }
}

/// AvoidCrossingPerimeters.cpp:1020-1029 and 1186-1195: removes the holes that an offset of 1 mm closes.
fn remove_too_small_holes(ex_poly: &mut ExPolygon) {
    let mut iter = 0usize;
    while iter < ex_poly.holes.len() {
        let out_offset_holes = offset_polygon(&ex_poly.holes[iter], ((1.0f32 as f64) / crate::libslic3r::SCALING_FACTOR) as f32);
        if out_offset_holes.is_empty() {
            ex_poly.holes.remove(iter);
        } else {
            iter += 1;
        }
    }
}

// AvoidCrossingPerimeters.cpp:1014-1091 — inner_offset
// Polygon offset which ensures that if a polygon breaks up into several separate parts, the original polygon will be used in these places.
// ExPolygons are handled one by one so returned ExPolygons could intersect.
fn inner_offset(ex_polygons: &[ExPolygon], offset_dis: f64) -> Vec<ExPolygon> {
    // try different offset_dis distances
    let min_contour_width_values: Vec<f64> = vec![offset_dis / 2.0, offset_dis, 2.0f64.mul_add(offset_dis, SCALED_EPSILON_D)];
    let mut ex_poly_result: Vec<ExPolygon> = ex_polygons.to_vec();
    // remove too small holes from the ex_poly
    for ex_poly in ex_poly_result.iter_mut() {
        remove_too_small_holes(ex_poly);
    }
    resample_expolygons(&mut ex_poly_result, offset_dis / 2.0, 0.5 / crate::libslic3r::SCALING_FACTOR);

    for ex_poly in ex_poly_result.iter_mut() {
        let mut bbox = crate::ex_polygon::get_extents_expoly(ex_poly);
        bbox.expand(SCALED_EPSILON_D as i64);

        // Filter out expolygons smaller than 0.1mm^2
        let bbox_size = cast_d(Point::new(bbox.max.x - bbox.min.x, bbox.max.y - bbox.min.y));
        let scaled_01 = (0.1f32 as f64) / crate::libslic3r::SCALING_FACTOR;
        if bbox_size.x * bbox_size.y < scaled_01 * scaled_01 {
            continue;
        }

        offset_ex_poly_by_contour_widths(ex_poly, &bbox, offset_dis, &min_contour_width_values);
    }
    ex_poly_result
}

// AvoidCrossingPerimeters.cpp:1093-1109 — get_support_polygons
fn get_support_polygons(support_layer: Option<&SupportLayer>, offset: f32) -> Vec<ExPolygon> {
    if let Some(support_layer) = support_layer {
        // If the support layer is available, then use its support islands.
        if !support_layer.support_islands.is_empty() {
            offset_ex(&support_layer.support_islands, offset)
        } else {
            let mut support_infill_polygons: Vec<ExPolygon> = Vec::new();
            let mut covered: Vec<Polygon> = Vec::new();
            support_layer.support_fills.polygons_covered_by_spacing(&mut covered, offset);
            let infill_polygons: Vec<ExPolygon> = union_ex_polygons(&covered);
            // get all the contours of infill_polygons
            for ex_poly in &infill_polygons {
                support_infill_polygons.push(ExPolygon::new(ex_poly.contour.clone()));
            }
            support_infill_polygons
        }
    } else {
        // If the support layer is not available, then return empty ExPolygons.
        Vec::new()
    }
}

/// What the C++ reaches from a `const Layer &` through its parent pointers and its dynamic type.
#[derive(Clone, Copy)]
pub struct LayerOwner<'a> {
    /// `layer.object()->print()`
    pub print: &'a Print,
    /// `layer.object()`: its index in `print.objects()`
    pub object: usize,
    /// `dynamic_cast<const SupportLayer *>(&layer)`
    pub support_layer: Option<&'a SupportLayer>,
}

// AvoidCrossingPerimeters.cpp:1111-1158 — get_boundary
// called by AvoidCrossingPerimeters::travel_to()
fn get_boundary(layer: &Layer, owner: &LayerOwner<'_>, perimeter_spacing: f32, include_supports_in_boundary: bool) -> Vec<ExPolygon> {
    // const float perimeter_spacing = get_perimeter_spacing(layer);
    let perimeter_offset: f32 = perimeter_spacing / 2.0f32;

    let object = &owner.print.objects()[owner.object];
    let support_layer = owner.support_layer;
    let object_supports_ptr = object.get_support_layer_at_printz(layer.print_z, crate::libslic3r::EPSILON);

    let mut boundary: Vec<ExPolygon> = layer.lslices.clone();
    if include_supports_in_boundary {
        let support_infill_polygons = get_support_polygons(object_supports_ptr, perimeter_spacing);
        boundary.extend(support_infill_polygons);
    }

    if support_layer.is_some() {
        let layer_below = object.get_first_layer_bellow_printz(layer.print_z, crate::libslic3r::EPSILON);
        if let Some(layer_below) = layer_below {
            boundary.extend(layer_below.lslices.iter().cloned());
        }
        // After calling inner_offset it is necessary to call union_ex because of the possibility of intersection ExPolygons
        boundary = union_ex(&boundary);
    } else {
        boundary = union_ex(&boundary);
    }

    boundary = union_ex(&inner_offset(&boundary, 1.5 * perimeter_spacing as f64));

    // Collect all top layers that will not be crossed.
    let mut polygons_count: usize = 0;
    for layer_region in layer.regions() {
        for surface in &layer_region.fill_surfaces.surfaces {
            if surface.is_top() {
                polygons_count += 1;
            }
        }
    }

    if polygons_count > 0 {
        let mut top_layer_polygons: Vec<ExPolygon> = Vec::new();
        top_layer_polygons.reserve(polygons_count);
        for layer_region in layer.regions() {
            for surface in &layer_region.fill_surfaces.surfaces {
                if surface.is_top() {
                    top_layer_polygons.push(surface.expolygon.clone());
                }
            }
        }

        top_layer_polygons = union_ex(&top_layer_polygons);
        // return diff_ex(boundary, top_layer_polygons);
        return diff_ex(&boundary, &offset_ex(&top_layer_polygons, (-1.2 * perimeter_offset as f64) as f32));
    }

    boundary
}

// AvoidCrossingPerimeters.cpp:1160-1254 — compute_object_boundary_local
// Local-coordinate (pre-instance-shift) external boundary geometry for a single object on a single
// layer: each island run through the resample + variable-width inner-offset pipeline. This is the
// expensive part of get_boundary_external()'s per-object work and depends only on
// (object, layer, is_support_layer, include_supports_in_boundary) - never on any PrintInstance's
// world-space shift - so it is cached per (object, layer). See ObjectBoundaryCache.
fn compute_object_boundary_local(
    object: &crate::print::faithful::PrintObject,
    layer: &Layer,
    support_layer: Option<&SupportLayer>,
    include_supports_in_boundary: bool,
    perimeter_spacing: f32,
    offset_dis: f32,
    min_contour_width_values: &[f64],
) -> Vec<ExPolygon> {
    let mut ex_polys_per_obj: Vec<ExPolygon> = Vec::new();
    let mut offset_ex_polys_per_obj: Vec<ExPolygon> = Vec::new();

    if let Some(l) = object.get_layer_at_printz(layer.print_z, crate::libslic3r::EPSILON) {
        for island in &l.lslices {
            ex_polys_per_obj.push(island.clone());
        }
    }
    if support_layer.is_some() {
        let layer_below = object.get_first_layer_bellow_printz(layer.print_z, crate::libslic3r::EPSILON);
        if let Some(layer_below) = layer_below {
            for island in &layer_below.lslices {
                ex_polys_per_obj.push(island.clone());
            }
        }
        if include_supports_in_boundary {
            ex_polys_per_obj.extend(get_support_polygons(support_layer, perimeter_spacing));
        }
    }

    for ex_poly in ex_polys_per_obj.iter_mut() {
        remove_too_small_holes(ex_poly);
    }

    // compute offset_dis distances for each hole per_obj
    resample_expolygons(&mut ex_polys_per_obj, offset_dis as f64 / 2.0, 0.5 / crate::libslic3r::SCALING_FACTOR);

    for ex_poly in ex_polys_per_obj.iter_mut() {
        let mut bbox = crate::ex_polygon::get_extents_expoly(ex_poly);
        bbox.expand(SCALED_EPSILON_D as i64);
        let bbox_size = cast_d(Point::new(bbox.max.x - bbox.min.x, bbox.max.y - bbox.min.y));
        let scaled_01 = (0.1f32 as f64) / crate::libslic3r::SCALING_FACTOR;
        if bbox_size.x * bbox_size.y < scaled_01 * scaled_01 {
            continue;
        }
        // ex_polys_num += 1;
        offset_ex_poly_by_contour_widths(ex_poly, &bbox, offset_dis as f64, min_contour_width_values);
    }

    for ex_poly in &ex_polys_per_obj {
        if ex_poly.contour.points.is_empty() {
            continue;
        }
        // append(holes_per_obj, ex_poly.holes);
        offset_ex_polys_per_obj.push(ex_poly.clone());
    }
    offset_ex_polys_per_obj
}

// AvoidCrossingPerimeters.hpp:61-64 — struct ObjectBoundaryCache
// Per-object, per-print-height cache of get_boundary_external()'s expensive local-coordinate
// geometry (each object's resampled islands run through the variable-width inner-offset pipeline).
// Invalidation key is the print height (print_z).
pub struct ObjectBoundaryCache {
    /// `std::unordered_map<const PrintObject *, ExPolygons> per_object`, by the index of the object.
    pub per_object: std::collections::HashMap<usize, Vec<ExPolygon>>,
    pub built_for_print_z: f64,
}

impl Default for ObjectBoundaryCache {
    fn default() -> Self {
        ObjectBoundaryCache { per_object: std::collections::HashMap::new(), built_for_print_z: -1.0 }
    }
}

// AvoidCrossingPerimeters.cpp:1259-1274 — get_object_boundary_local_cached
// Returns compute_object_boundary_local()'s result for (object, print height), computing and caching
// it on first use for this print height and reusing the cached value on every subsequent call at the
// same height. The cache is invalidated (cleared) whenever the print height (layer.print_z) changes.
#[allow(clippy::too_many_arguments)]
fn get_object_boundary_local_cached<'c>(
    cache: &'c mut ObjectBoundaryCache,
    print: &Print,
    object: usize,
    layer: &Layer,
    support_layer: Option<&SupportLayer>,
    include_supports_in_boundary: bool,
    perimeter_spacing: f32,
    offset_dis: f32,
    min_contour_width_values: &[f64],
) -> &'c Vec<ExPolygon> {
    if cache.built_for_print_z != layer.print_z {
        cache.per_object.clear();
        cache.built_for_print_z = layer.print_z;
    }
    cache.per_object.entry(object).or_insert_with(|| {
        compute_object_boundary_local(
            &print.objects()[object],
            layer,
            support_layer,
            include_supports_in_boundary,
            perimeter_spacing,
            offset_dis,
            min_contour_width_values,
        )
    })
}

// AvoidCrossingPerimeters.cpp:1276-1303 — get_boundary_external
fn get_boundary_external(
    layer: &Layer,
    owner: &LayerOwner<'_>,
    include_supports_in_boundary: bool,
    boundary_cache: &mut ObjectBoundaryCache,
) -> Vec<Polygon> {
    let perimeter_spacing: f32 = get_perimeter_spacing_external(layer, owner.print);
    // const float flow_width        = get_external_perimeter_width(layer);
    let offset_dis: f32 = (1.5 * perimeter_spacing as f64) as f32;
    // double      min_contour_width = {offset_dis / 2.};
    let min_contour_width_values: Vec<f64> =
        vec![offset_dis as f64 / 2.0, offset_dis as f64, 2.0f64.mul_add(offset_dis as f64, SCALED_EPSILON_D)];
    let mut offset_ex_polygons: Vec<ExPolygon> = Vec::new();

    // Collect all holes for all printed objects and their instances, which will be printed at the same time as passed "layer".
    let support_layer = owner.support_layer;
    for object in 0..owner.print.objects().len() {
        // The per-object local boundary geometry depends only on (object, layer); reuse it across the
        // repeated rebuilds within this layer instead of recomputing the expensive offset pipeline.
        let offset_ex_polys_per_obj = get_object_boundary_local_cached(
            boundary_cache,
            owner.print,
            object,
            layer,
            support_layer,
            include_supports_in_boundary,
            perimeter_spacing,
            offset_dis,
            &min_contour_width_values,
        );
        for instance in owner.print.objects()[object].instances() {
            let mut index = offset_ex_polygons.len();
            offset_ex_polygons.extend(offset_ex_polys_per_obj.iter().cloned());
            while index < offset_ex_polygons.len() {
                offset_ex_polygons[index].translate(instance.shift);
                index += 1;
            }
        }
    }
    to_polygons(&offset_ex_polygons)
}

// AvoidCrossingPerimeters.cpp:1276-1281 — init_boundary_distances
fn init_boundary_distances(boundary: &mut Boundary) {
    boundary.boundaries_params.clear();
    boundary.boundaries_params.resize(boundary.boundaries.len(), Vec::new());
    for poly_idx in 0..boundary.boundaries.len() {
        let mut params = Vec::new();
        precompute_polygon_distances(&boundary.boundaries[poly_idx], &mut params);
        boundary.boundaries_params[poly_idx] = params;
    }
}

// AvoidCrossingPerimeters.cpp:1283-1295 — init_boundary
pub fn init_boundary(boundary: &mut Boundary, boundary_polygons: Vec<Polygon>) {
    boundary.clear();
    boundary.boundaries = boundary_polygons;

    let mut bbox = get_extents(&boundary.boundaries);
    bbox.expand(SCALED_EPSILON as i64); // BoundingBox::offset(SCALED_EPSILON)
    boundary.bbox = BoundingBoxF::from_points_minmax(
        Vec2d { x: bbox.min.x as f64, y: bbox.min.y as f64 },
        Vec2d { x: bbox.max.x as f64, y: bbox.max.y as f64 },
    );
    boundary.grid.set_bbox(bbox);
    // FIXME 1mm grid?
    boundary.grid.create_from_polygons(&boundary.boundaries, crate::scaled(1.0));
    init_boundary_distances(boundary);
}

// AvoidCrossingPerimeters.cpp:1297-1312 — init_boundary (with merge points)
pub fn init_boundary_with_merge_points(
    boundary: &mut Boundary,
    boundary_polygons: Vec<Polygon>,
    merge_points: &[Point],
) {
    boundary.clear();
    boundary.boundaries = boundary_polygons;

    let mut bbox = get_extents(&boundary.boundaries);
    for merge_point in merge_points {
        bbox.merge_point(*merge_point);
    }
    bbox.expand(bbox_radius(&bbox) as i64); // BoundingBox::offset(bbox.radius())
    boundary.bbox = BoundingBoxF::from_points_minmax(
        Vec2d { x: bbox.min.x as f64, y: bbox.min.y as f64 },
        Vec2d { x: bbox.max.x as f64, y: bbox.max.y as f64 },
    );
    boundary.grid.set_bbox(bbox);
    // FIXME 1mm grid?
    boundary.grid.create_from_polygons(&boundary.boundaries, crate::scaled(1.0));
    init_boundary_distances(boundary);
}

// AvoidCrossingPerimeters.cpp:1353-1362 (DEBUG_BOUNDARIES)
// void SaveInitTravelToSVG(const AvoidCrossingPerimeters::Boundary &boundary, const ExPolygons &lslices, const Line &travel, const std::string &file_path)
#[allow(non_snake_case, dead_code)]
pub fn SaveInitTravelToSVG(boundary: &Boundary, lslices: &[ExPolygon], travel: &Line, file_path: &str) {
    let mut bbox = get_extents(&boundary.boundaries);
    bbox.expand(bbox_radius(&bbox) as i64);
    let mut svg = crate::svg::SVG::new_bbox_default(file_path, &bbox);

    svg.draw_outline_expolygons(lslices, "green", "blue", 0.0);
    svg.draw_outline_polygons(&boundary.boundaries, "black", 0.0);
    svg.draw_line(travel, "blue", 0.0);
}

// BoundingBox::radius() — half the diagonal length. (BoundingBox.hpp)
fn bbox_radius(bbox: &BoundingBox) -> f64 {
    let w = (bbox.max.x - bbox.min.x) as f64;
    let h = (bbox.max.y - bbox.min.y) as f64;
    0.5 * (w * w + h * h).sqrt()
}

// get_extents(Polygons) — bounding box of all polygon points (Geometry.hpp/BoundingBox.hpp).
fn get_extents(polygons: &[Polygon]) -> BoundingBox {
    let mut bbox = BoundingBox::new();
    for poly in polygons {
        for p in &poly.points {
            bbox.merge_point(*p);
        }
    }
    bbox
}

// AvoidCrossingPerimeters.cpp:37-52 (.hpp) — struct Boundary
/// Collection of boundaries used for detection of crossing perimeters for travels.
pub struct Boundary {
    // Collection of boundaries used for detection of crossing perimeters for travels
    pub boundaries: Vec<Polygon>,
    // Bounding box of boundaries
    pub bbox: BoundingBoxF,
    // Precomputed distances of all points in boundaries
    pub boundaries_params: Vec<Vec<f32>>,
    // Used for detection of intersection between line and any polygon from boundaries
    pub grid: EdgeGrid,
}

impl Boundary {
    pub fn new() -> Self {
        Boundary {
            boundaries: Vec::new(),
            bbox: BoundingBoxF::new(),
            boundaries_params: Vec::new(),
            grid: EdgeGrid::new(),
        }
    }

    // AvoidCrossingPerimeters.hpp:47 — void clear()
    pub fn clear(&mut self) {
        self.boundaries.clear();
        self.boundaries_params.clear();
    }
}

impl Default for Boundary {
    fn default() -> Self {
        Self::new()
    }
}

// AvoidCrossingPerimeters.hpp:15-71 — class AvoidCrossingPerimeters
//
// The state, the once-modifiers, `travel_to` and `init_layer`; the C++ statics they call
// (`avoid_perimeters`, `avoid_perimeters_inner`, `simplify_travel`, the perimeter-spacing helpers,
// the boundary builders) are the free functions above.
pub struct AvoidCrossingPerimeters {
    m_use_external_mp: bool,
    // just for the next travel move
    m_use_external_mp_once: bool,
    // this flag disables reduce_crossing_wall just for the next travel move
    // we enable it by default for the first travel move in print
    m_disabled_once: bool,

    // Lslices offseted by half an external perimeter width. Used for detection if line or polyline is inside of any polygon.
    m_lslices_offset: Vec<crate::geometry::ExPolygon>,
    m_lslices_offset_bboxes: Vec<BoundingBox>,
    // Used for detection of line or polyline is inside of any polygon.
    m_grid_lslice: EdgeGrid,
    // Store all needed data for travels inside object
    m_internal: Boundary,
    // Store all needed data for travels outside object
    m_external: Boundary,
    // Per-object local-coordinate boundary geometry cache for the current layer (see ObjectBoundaryCache).
    m_object_boundary_cache: ObjectBoundaryCache,
}

impl AvoidCrossingPerimeters {
    pub fn new() -> Self {
        AvoidCrossingPerimeters {
            m_use_external_mp: false,
            m_use_external_mp_once: false,
            m_disabled_once: true,
            m_lslices_offset: Vec::new(),
            m_lslices_offset_bboxes: Vec::new(),
            m_grid_lslice: EdgeGrid::new(),
            m_internal: Boundary::new(),
            m_external: Boundary::new(),
            m_object_boundary_cache: ObjectBoundaryCache::default(),
        }
    }

    // AvoidCrossingPerimeters.hpp:19 — void use_external_mp(bool use = true)
    pub fn use_external_mp(&mut self, use_: bool) {
        self.m_use_external_mp = use_;
    }

    // AvoidCrossingPerimeters.hpp:20 — bool used_external_mp()
    pub fn used_external_mp(&self) -> bool {
        self.m_use_external_mp
    }

    // AvoidCrossingPerimeters.hpp:21 — void use_external_mp_once()
    pub fn use_external_mp_once(&mut self) {
        self.m_use_external_mp_once = true;
    }

    // AvoidCrossingPerimeters.hpp:22 — bool used_external_mp_once()
    pub fn used_external_mp_once(&self) -> bool {
        self.m_use_external_mp_once
    }

    // AvoidCrossingPerimeters.hpp:23 — void disable_once()
    pub fn disable_once(&mut self) {
        self.m_disabled_once = true;
    }

    // AvoidCrossingPerimeters.hpp:24 — bool disabled_once() const
    pub fn disabled_once(&self) -> bool {
        self.m_disabled_once
    }

    // AvoidCrossingPerimeters.hpp:25 — void reset_once_modifiers()
    pub fn reset_once_modifiers(&mut self) {
        self.m_use_external_mp_once = false;
        self.m_disabled_once = false;
    }
}

/// `AvoidCrossingPerimeters::travel_to(const GCode&, const Point&)` (AvoidCrossingPerimeters.hpp:29-33),
/// the overload without `could_be_wipe_disabled`: `TravelToPoint::travel_to(&mut acp, gcodegen, owner, point)`.
pub trait TravelToPoint {
    fn travel_to(&mut self, gcodegen: &GCode<'_>, owner: &LayerOwner<'_>, point: &Point) -> Polyline;
}

impl TravelToPoint for AvoidCrossingPerimeters {
    // AvoidCrossingPerimeters.hpp:29-33
    // Polyline travel_to(const GCode& gcodegen, const Point& point)
    fn travel_to(&mut self, gcodegen: &GCode<'_>, owner: &LayerOwner<'_>, point: &Point) -> Polyline {
        let mut could_be_wipe_disabled = false;
        AvoidCrossingPerimeters::travel_to(self, gcodegen, owner, point, &mut could_be_wipe_disabled)
    }
}

impl AvoidCrossingPerimeters {
    // AvoidCrossingPerimeters.cpp:1366-1448
    // Polyline AvoidCrossingPerimeters::travel_to(const GCode &gcodegen, const Point &point, bool *could_be_wipe_disabled)
    // Plan travel, which avoids perimeter crossings by following the boundaries of the layer.
    //
    // `owner` is what the C++ reaches from `gcodegen.layer()` (see `LayerOwner`).
    pub fn travel_to(&mut self, gcodegen: &GCode<'_>, owner: &LayerOwner<'_>, point: &Point, could_be_wipe_disabled: &mut bool) -> Polyline {
        use crate::config::ConfigOptionResolver;
        // If use_external, then perform the path planning in the world coordinate system (correcting for the gcodegen offset).
        // Otherwise perform the path planning in the coordinate system of the active object.
        let use_external = self.m_use_external_mp || self.m_use_external_mp_once;
        // Point::new_scale(x, y): Point(coord_t(scale_(x)), coord_t(scale_(y)))
        let scaled_origin = if use_external {
            Point::new(
                (gcodegen.origin().x / crate::libslic3r::SCALING_FACTOR) as i64,
                (gcodegen.origin().y / crate::libslic3r::SCALING_FACTOR) as i64,
            )
        } else {
            Point::new(0, 0)
        };
        let start = Point::new(gcodegen.last_pos().x + scaled_origin.x, gcodegen.last_pos().y + scaled_origin.y);
        let end = Point::new(point.x + scaled_origin.x, point.y + scaled_origin.y);
        let travel = Line::new(start, end);
        let mut result_pl = Polyline::new();
        let mut travel_intersection_count: usize = 0;
        let startf = cast_d(start);
        let endf = cast_d(end);

        let layer: &Layer = gcodegen.layer().expect("AvoidCrossingPerimeters::travel_to: no layer");
        let is_support_layer = owner.support_layer.is_some();
        let include_supports_in_boundary =
            gcodegen.config().option("avoid_crossing_wall_includes_support").and_then(|o| o.get_bool().ok()).unwrap_or(false);
        if !use_external
            && (is_support_layer
                || (!self.m_lslices_offset.is_empty()
                    && !travel.any_expolygon_contains(&self.m_lslices_offset, &self.m_lslices_offset_bboxes, &self.m_grid_lslice)))
        {
            // Initialize m_internal only when it is necessary.
            if self.m_internal.boundaries.is_empty() {
                init_boundary_with_merge_points(
                    &mut self.m_internal,
                    to_polygons(&get_boundary(layer, owner, get_perimeter_spacing(layer), include_supports_in_boundary)),
                    &[start, end],
                );
            } else if !(self.m_internal.bbox.contains_point(&startf) && self.m_internal.bbox.contains_point(&endf)) {
                // check if start and end are in bbox, if not, merge start and end points to bbox
                self.m_internal.clear();
                init_boundary_with_merge_points(
                    &mut self.m_internal,
                    to_polygons(&get_boundary(layer, owner, get_perimeter_spacing(layer), include_supports_in_boundary)),
                    &[start, end],
                );
            }

            if !self.m_internal.boundaries.is_empty() {
                travel_intersection_count = avoid_perimeters(&self.m_internal, &start, &end, layer, &mut result_pl);
                *result_pl.points.first_mut().unwrap() = start;
                *result_pl.points.last_mut().unwrap() = end;
            }
        } else if use_external {
            // Initialize m_external only when exist any external travel for the current layer.
            if self.m_external.boundaries.is_empty() {
                let boundary = get_boundary_external(layer, owner, include_supports_in_boundary, &mut self.m_object_boundary_cache);
                init_boundary_with_merge_points(&mut self.m_external, boundary, &[start, end]);
            } else if !(self.m_external.bbox.contains_point(&startf) && self.m_external.bbox.contains_point(&endf)) {
                // check if start and end are in bbox
                self.m_external.clear();
                let boundary = get_boundary_external(layer, owner, include_supports_in_boundary, &mut self.m_object_boundary_cache);
                init_boundary_with_merge_points(&mut self.m_external, boundary, &[start, end]);
            }

            // Trim the travel line by the bounding box.
            if !self.m_external.boundaries.is_empty() {
                travel_intersection_count = avoid_perimeters(&self.m_external, &start, &end, layer, &mut result_pl);
                *result_pl.points.first_mut().unwrap() = start;
                *result_pl.points.last_mut().unwrap() = end;
            }
        }

        if result_pl.points.is_empty() {
            // Travel line is completely outside the bounding box.
            result_pl = Polyline::from_points(vec![start, end]);
            travel_intersection_count = 0;
        }

        let opt_max_detour = gcodegen
            .config()
            .option("max_travel_detour_distance")
            .and_then(|o| o.as_any().downcast_ref::<crate::config::ConfigOptionFloatOrPercent>())
            .map(|o| (o.value, o.percent))
            .unwrap_or((0.0, false));
        let mut max_detour_length_exceeded = false;
        if opt_max_detour.0 > 0.0 {
            let direct_length: f64 = travel.length();
            let detour: f64 = result_pl.length() - direct_length;
            let max_detour_length: f64 =
                if opt_max_detour.1 { direct_length * 0.01 * opt_max_detour.0 } else { opt_max_detour.0 / crate::libslic3r::SCALING_FACTOR };
            if detour > max_detour_length {
                result_pl = Polyline::from_points(vec![start, end]);
                max_detour_length_exceeded = true;
            }
        }

        if use_external {
            result_pl.translate(Point::new(-scaled_origin.x, -scaled_origin.y));
            *could_be_wipe_disabled = false;
        } else if max_detour_length_exceeded {
            *could_be_wipe_disabled = false;
        } else {
            *could_be_wipe_disabled = !need_wipe(
                gcodegen,
                &self.m_lslices_offset,
                &self.m_lslices_offset_bboxes,
                &self.m_grid_lslice,
                &travel,
                &result_pl,
                travel_intersection_count,
            );
        }

        result_pl
    }

    // ************************************* AvoidCrossingPerimeters::init_layer() *****************************************

    // AvoidCrossingPerimeters.cpp:1452-1475
    // void AvoidCrossingPerimeters::init_layer(const Layer &layer)
    pub fn init_layer(&mut self, layer: &Layer) {
        self.m_internal.clear();
        self.m_external.clear();
        // Note: m_object_boundary_cache is intentionally NOT cleared here. init_layer() is called once per
        // object at each print height, so clearing here would defeat cross-object reuse within a height.
        // The cache self-invalidates by print_z inside get_object_boundary_local_cached().

        self.m_lslices_offset.clear();
        self.m_lslices_offset_bboxes.clear();
        for coeff in [0.6f32, 0.5f32, 0.45f32] {
            self.m_lslices_offset = offset_ex(&layer.lslices, -get_external_perimeter_width(layer) * coeff);
            if !self.m_lslices_offset.is_empty() {
                break;
            }
        }
        self.m_lslices_offset_bboxes.reserve(self.m_lslices_offset.len());
        for ex_polygon in &self.m_lslices_offset {
            self.m_lslices_offset_bboxes.push(crate::ex_polygon::get_extents_expoly(ex_polygon));
        }

        let mut bbox_slice = crate::ex_polygon::get_extents(&layer.lslices);
        bbox_slice.expand(SCALED_EPSILON_D as i64);

        self.m_grid_lslice.set_bbox(bbox_slice);
        //FIXME 1mm grid?
        self.m_grid_lslice.create_from_polygons(&to_polygons(&self.m_lslices_offset), crate::scaled(1.0));
    }
}

impl Default for AvoidCrossingPerimeters {
    fn default() -> Self {
        Self::new()
    }
}

/// AvoidCrossingPerimeters.cpp:1477-1866: the block under `#if 0`, the planner as it was before the
/// boundaries were built on demand. The C++ does not compile it; it is kept here function by function.
/// Its `travel_to` (cpp:1686-1723) is not here: it calls a `need_wipe` of five arguments and reads
/// `max_travel_detour_distance` as a float, neither of which exists at the pin.
#[allow(dead_code)]
mod legacy {
    use super::*;

    // AvoidCrossingPerimeters.cpp:1478-1484
    // static double travel_length(const std::vector<TravelPoint> &travel)
    fn travel_length(travel: &[TravelPoint]) -> f64 {
        let mut total_length: f64 = 0.0;
        for idx in 1..travel.len() {
            let d = cast_d(Point::new(travel[idx].point.x - travel[idx - 1].point.x, travel[idx].point.y - travel[idx - 1].point.y));
            total_length += (d.x * d.x + d.y * d.y).sqrt();
        }

        total_length
    }

    // AvoidCrossingPerimeters.cpp:1487-1577
    // static size_t avoid_perimeters_inner(const AvoidCrossingPerimeters::Boundary &boundary, const Point &start, const Point &end, std::vector<TravelPoint> &result_out)
    // Called by avoid_perimeters() and by simplify_travel_heuristics().
    fn avoid_perimeters_inner(boundary: &Boundary, start: &Point, end: &Point, result_out: &mut Vec<TravelPoint>) -> usize {
        let boundaries = &boundary.boundaries;
        let edge_grid = &boundary.grid;
        // Find all intersections between boundaries and the line segment, sort them along the line segment.
        let mut intersections: Vec<Intersection>;
        {
            let visitor = AllIntersectionsVisitor::with_line(edge_grid, Line::new(*start, *end));
            intersections = visitor.run();
            let dir = cast_d(Point::new(end.x - start.x, end.y - start.y));
            for intersection in intersections.iter_mut() {
                intersection.distance = boundary.boundaries_params[intersection.border_idx][intersection.line_idx];
            }
            crate::clipper::libcxx_sort(&mut intersections, |l, r| {
                let v = cast_d(Point::new(r.point.x - l.point.x, r.point.y - l.point.y));
                v.x * dir.x + v.y * dir.y > 0.0
            });
        }

        let mut result: Vec<TravelPoint> = Vec::new();
        result.push(TravelPoint { point: *start, border_idx: -1, do_not_remove: false });
        let mut it_first = 0usize;
        while it_first < intersections.len() {
            // The entry point to the boundary polygon
            let intersection_first = intersections[it_first];
            // Skip the it_first from the search for the farthest exit point from the boundary polygon
            // Search for the farthest intersection different from it_first but with the same border_idx
            let mut it_second: Option<usize> = None;
            let mut j = intersections.len();
            while j > it_first + 1 {
                j -= 1;
                if intersection_first.border_idx == intersections[j].border_idx {
                    it_second = Some(j);
                    break;
                }
            }

            // Append the first intersection into the path
            let mut left_idx: usize = intersection_first.line_idx;
            let mut right_idx: usize =
                if intersection_first.line_idx + 1 == boundaries[intersection_first.border_idx].points.len() { 0 } else { intersection_first.line_idx + 1 };
            // Offset of the polygon's point using get_middle_point_offset is used to simplify the calculation of intersection between the
            // boundary and the travel. The appended point is translated in the direction of inward normal. This translation ensures that the
            // appended point will be inside the polygon and not on the polygon border.
            result.push(TravelPoint {
                point: get_middle_point_offset(
                    &boundaries[intersection_first.border_idx],
                    left_idx,
                    right_idx,
                    &intersection_first.point,
                    SCALED_EPSILON_D as i64,
                ),
                border_idx: intersection_first.border_idx as i32,
                do_not_remove: false,
            });

            // Check if intersection line also exit the boundary polygon
            if let Some(it_second) = it_second {
                // The exit point from the boundary polygon
                let intersection_second = intersections[it_second];
                let shortest_direction = get_shortest_direction(
                    boundary,
                    &intersection_first,
                    &intersection_second,
                    *boundary.boundaries_params[intersection_first.border_idx].last().unwrap(),
                );
                // Append the path around the border into the path
                let size = boundaries[intersection_first.border_idx].points.len() as i32;
                if shortest_direction == Direction::Forward {
                    let mut line_idx = intersection_first.line_idx as i32;
                    while line_idx != intersection_second.line_idx as i32 {
                        result.push(TravelPoint {
                            point: get_polygon_vertex_offset(
                                &boundaries[intersection_first.border_idx],
                                if line_idx + 1 == size { 0 } else { (line_idx + 1) as usize },
                                SCALED_EPSILON_D as i64,
                            ),
                            border_idx: intersection_first.border_idx as i32,
                            do_not_remove: false,
                        });
                        line_idx = if line_idx + 1 < size { line_idx + 1 } else { 0 };
                    }
                } else {
                    let mut line_idx = intersection_first.line_idx as i32;
                    while line_idx != intersection_second.line_idx as i32 {
                        result.push(TravelPoint {
                            point: get_polygon_vertex_offset(&boundaries[intersection_second.border_idx], (line_idx + 0) as usize, SCALED_EPSILON_D as i64),
                            border_idx: intersection_first.border_idx as i32,
                            do_not_remove: false,
                        });
                        line_idx = if line_idx - 1 >= 0 { line_idx - 1 } else { size - 1 };
                    }
                }

                // Append the farthest intersection into the path
                left_idx = intersection_second.line_idx;
                right_idx = if intersection_second.line_idx >= (boundaries[intersection_second.border_idx].points.len() - 1) {
                    0
                } else {
                    intersection_second.line_idx + 1
                };
                result.push(TravelPoint {
                    point: get_middle_point_offset(
                        &boundaries[intersection_second.border_idx],
                        left_idx,
                        right_idx,
                        &intersection_second.point,
                        SCALED_EPSILON_D as i64,
                    ),
                    border_idx: intersection_second.border_idx as i32,
                    do_not_remove: false,
                });
                // Skip intersections in between
                it_first = it_second;
            }
            it_first += 1;
        }

        result.push(TravelPoint { point: *end, border_idx: -1, do_not_remove: false });

        if !intersections.is_empty() {
            result = simplify_travel(boundary, &result);
        }

        result_out.append(&mut result);
        intersections.len()
    }

    // AvoidCrossingPerimeters.cpp:1579-1656
    // static std::vector<TravelPoint> simplify_travel_heuristics(const AvoidCrossingPerimeters::Boundary &boundary, const std::vector<TravelPoint> &travel)
    fn simplify_travel_heuristics(boundary: &Boundary, travel: &[TravelPoint]) -> Vec<TravelPoint> {
        let norm = |a: &Point, b: &Point| -> f64 {
            let d = cast_d(Point::new(a.x - b.x, a.y - b.y));
            (d.x * d.x + d.y * d.y).sqrt()
        };
        let mut simplified_path: Vec<TravelPoint> = Vec::new();
        let mut visitor = AllIntersectionsVisitor::new(&boundary.grid);
        simplified_path.reserve(travel.len());
        simplified_path.push(travel[0]);
        let mut point_idx: usize = 1;
        while point_idx < travel.len() {
            // Skip all indexes on the same polygon
            while point_idx < travel.len() && travel[point_idx - 1].border_idx == travel[point_idx].border_idx {
                simplified_path.push(travel[point_idx]);
                point_idx += 1;
            }

            if point_idx < travel.len() {
                let current = travel[point_idx - 1];
                let next = travel[point_idx];
                let mut new_next = next;
                let mut new_point_idx: usize = point_idx;
                let mut path_length: f64 = norm(&next.point, &current.point);
                let mut new_path_shorter_by: f64 = 0.0;
                let mut border_idx_change_count: usize = 0;
                let mut shortcut: Vec<TravelPoint> = Vec::new();
                for point_idx_2 in point_idx + 1..travel.len() {
                    let possible_new_next = travel[point_idx_2];
                    if travel[point_idx_2 - 1].border_idx != travel[point_idx_2].border_idx {
                        border_idx_change_count += 1;
                    }

                    if border_idx_change_count >= 2 {
                        break;
                    }

                    path_length += norm(&possible_new_next.point, &travel[point_idx_2 - 1].point);
                    let shortcut_length: f64 = norm(&possible_new_next.point, &current.point);
                    if (path_length - shortcut_length) <= 10.0 / crate::libslic3r::SCALING_FACTOR {
                        continue;
                    }

                    visitor.intersections.clear();
                    visitor.reset();
                    visitor.travel_line.a = current.point;
                    visitor.travel_line.b = possible_new_next.point;
                    let (a, b) = (visitor.travel_line.a, visitor.travel_line.b);
                    boundary.grid.visit_cells_intersecting_line(a, b, |iy, ix| visitor.visit(iy, ix));
                    if !visitor.intersections.is_empty() {
                        let dir = cast_d(Point::new(b.x - a.x, b.y - a.y));
                        crate::clipper::libcxx_sort(&mut visitor.intersections, |l, r| {
                            let v = cast_d(Point::new(r.point.x - l.point.x, r.point.y - l.point.y));
                            v.x * dir.x + v.y * dir.y > 0.0
                        });
                        let mut last_border_idx_count: usize = 0;
                        for intersection in &visitor.intersections {
                            if intersection.border_idx as i32 == possible_new_next.border_idx {
                                last_border_idx_count += 1;
                            }
                        }

                        if last_border_idx_count > 0 {
                            continue;
                        }

                        let mut possible_shortcut: Vec<TravelPoint> = Vec::new();
                        avoid_perimeters_inner(boundary, &current.point, &possible_new_next.point, &mut possible_shortcut);
                        let shortcut_travel: f64 = travel_length(&possible_shortcut);
                        if path_length > shortcut_travel && path_length - shortcut_travel > new_path_shorter_by {
                            new_path_shorter_by = path_length - shortcut_travel;
                            shortcut = possible_shortcut;
                            new_next = possible_new_next;
                            new_point_idx = point_idx_2;
                        }
                    }
                }

                if !shortcut.is_empty() {
                    debug_assert!(shortcut.len() >= 2);
                    simplified_path.extend_from_slice(&shortcut[1..shortcut.len() - 1]);
                    point_idx = new_point_idx;
                }

                simplified_path.push(new_next);
            }
            point_idx += 1;
        }

        simplified_path
    }

    // AvoidCrossingPerimeters.cpp:1658-1684
    // static size_t avoid_perimeters(const AvoidCrossingPerimeters::Boundary &boundary, const Point &start, const Point &end, Polyline &result_out)
    // Called by AvoidCrossingPerimeters::travel_to()
    fn avoid_perimeters(boundary: &Boundary, start: &Point, end: &Point, result_out: &mut Polyline) -> usize {
        // Travel line is completely or partially inside the bounding box.
        let mut path: Vec<TravelPoint> = Vec::new();
        let num_intersections = avoid_perimeters_inner(boundary, start, end, &mut path);
        if num_intersections != 0 {
            path = simplify_travel_heuristics(boundary, &path);
            path.reverse();
            path = simplify_travel_heuristics(boundary, &path);
            path.reverse();
        }

        *result_out = to_polyline(&path);

        num_intersections
    }

    // AvoidCrossingPerimeters.cpp:1727-1738
    // static std::pair<Polygons, Polygons> split_expolygon(const ExPolygons &ex_polygons)
    // called by AvoidCrossingPerimeters::init_layer()->get_boundary()/get_boundary_external()
    fn split_expolygon(ex_polygons: &[ExPolygon]) -> (Vec<Polygon>, Vec<Polygon>) {
        let mut contours: Vec<Polygon> = Vec::new();
        let mut holes: Vec<Polygon> = Vec::new();
        contours.reserve(ex_polygons.len());
        holes.reserve(ex_polygons.iter().fold(0usize, |sum, ex_poly| sum + ex_poly.holes.len()));
        for ex_poly in ex_polygons {
            contours.push(ex_poly.contour.clone());
            holes.extend(ex_poly.holes.iter().cloned());
        }
        (contours, holes)
    }

    /// ClipperUtils.cpp:677-678 `Polygons diff(const Polygons &subject, const Polygons &clip)`
    fn diff(subject: &[Polygon], clip: &[Polygon]) -> Vec<Polygon> {
        paths_polygons(&clipper_do::<Paths>(ClipType::ctDifference, &polygons_paths(subject), &polygons_paths(clip), PolyFillType::pftNonZero))
    }

    /// ClipperUtils.cpp:741-742 `ExPolygons diff_ex(const Polygons &subject, const Polygons &clip)`
    fn diff_ex_polygons(subject: &[Polygon], clip: &[Polygon]) -> Vec<ExPolygon> {
        _clipper_ex(ClipType::ctDifference, &polygons_paths(subject), &polygons_paths(clip), ApplySafetyOffset::No, PolyFillType::pftNonZero)
    }

    /// ClipperUtils.cpp:802-803 `ExPolygons intersection_ex(const ExPolygons &subject, const ExPolygons &clip)`
    fn intersection_ex(subject: &[ExPolygon], clip: &[ExPolygon]) -> Vec<ExPolygon> {
        _clipper_ex(ClipType::ctIntersection, &expolygons_paths(subject), &expolygons_paths(clip), ApplySafetyOffset::No, PolyFillType::pftNonZero)
    }

    // AvoidCrossingPerimeters.cpp:1741-1799
    // static ExPolygons get_boundary(const Layer &layer)
    // called by AvoidCrossingPerimeters::init_layer()
    fn get_boundary(layer: &Layer) -> Vec<ExPolygon> {
        let perimeter_spacing: f32 = get_perimeter_spacing(layer);
        let perimeter_offset: f32 = perimeter_spacing / 2.0f32;
        let mut polygons_count: usize = 0;
        for layer_region in layer.regions() {
            polygons_count += layer_region.slices.surfaces.len();
        }

        let mut boundary: Vec<ExPolygon> = Vec::new();
        boundary.reserve(polygons_count);
        for layer_region in layer.regions() {
            for surface in &layer_region.slices.surfaces {
                boundary.push(surface.expolygon.clone());
            }
        }

        boundary = union_ex(&boundary);
        let mut perimeter_boundary: Vec<ExPolygon> = offset_ex(&boundary, -perimeter_offset);
        let mut result_boundary: Vec<ExPolygon>;
        if perimeter_boundary.len() != boundary.len() {
            //FIXME ???
            // If any part of the polygon is missing after shrinking, then for misisng parts are is used the boundary of the slice.
            let missing_perimeter_boundary: Vec<ExPolygon> = offset_ex(
                &diff_ex(&boundary, &offset_ex(&perimeter_boundary, perimeter_offset + (SCALED_EPSILON_D as f32) / 2.0f32)),
                perimeter_offset + SCALED_EPSILON_D as f32,
            );
            perimeter_boundary = offset_ex(&perimeter_boundary, perimeter_offset);
            perimeter_boundary.extend(missing_perimeter_boundary);
            // By calling intersection_ex some artifacts arose by previous operations are removed.
            result_boundary = intersection_ex(&offset_ex(&perimeter_boundary, -perimeter_offset), &boundary);
        } else {
            result_boundary = perimeter_boundary;
        }

        let (contours, holes) = split_expolygon(&boundary);
        // Add an outer boundary to avoid crossing perimeters from supports
        let outer_boundary: Vec<ExPolygon> = union_ex_polygons(&diff(
            &offset_polygon(&crate::geometry::convex_hull_polygons(&contours), 2.0f32 * perimeter_spacing),
            &offset_polygons(&contours, perimeter_spacing + perimeter_offset),
        ));
        result_boundary.extend(outer_boundary);
        let holes_boundary: Vec<ExPolygon> = offset_ex_polygons(&holes, -perimeter_spacing);
        result_boundary.extend(holes_boundary);
        result_boundary = union_ex(&result_boundary);

        // Collect all top layers that will not be crossed.
        polygons_count = 0;
        for layer_region in layer.regions() {
            for surface in &layer_region.fill_surfaces.surfaces {
                if surface.is_top() {
                    polygons_count += 1;
                }
            }
        }

        if polygons_count > 0 {
            let mut top_layer_polygons: Vec<ExPolygon> = Vec::new();
            top_layer_polygons.reserve(polygons_count);
            for layer_region in layer.regions() {
                for surface in &layer_region.fill_surfaces.surfaces {
                    if surface.is_top() {
                        top_layer_polygons.push(surface.expolygon.clone());
                    }
                }
            }

            top_layer_polygons = union_ex(&top_layer_polygons);
            return diff_ex(&result_boundary, &offset_ex(&top_layer_polygons, -perimeter_offset));
        }

        result_boundary
    }

    // AvoidCrossingPerimeters.cpp:1802-1833
    // static ExPolygons get_boundary_external(const Layer &layer)
    // called by AvoidCrossingPerimeters::init_layer()
    fn get_boundary_external(layer: &Layer, owner: &LayerOwner<'_>) -> Vec<ExPolygon> {
        let perimeter_spacing: f32 = get_perimeter_spacing_external(layer, owner.print);
        let perimeter_offset: f32 = perimeter_spacing / 2.0f32;
        let mut boundary: Vec<ExPolygon> = Vec::new();
        // Collect all polygons for all printed objects and their instances, which will be printed at the same time as passed "layer".
        for object in owner.print.objects() {
            let mut polygons_per_obj: Vec<ExPolygon> = Vec::new();
            //FIXME with different layering, layers on other objects will not be found at this object's print_z.
            // Search an overlap of layers?
            if let Some(l) = object.get_layer_at_printz(layer.print_z, crate::libslic3r::EPSILON) {
                for layer_region in l.regions() {
                    for surface in &layer_region.slices.surfaces {
                        polygons_per_obj.push(surface.expolygon.clone());
                    }
                }
            }

            for instance in object.instances() {
                let mut boundary_idx = boundary.len();
                boundary.extend(polygons_per_obj.iter().cloned());
                while boundary_idx < boundary.len() {
                    boundary[boundary_idx].translate(instance.shift);
                    boundary_idx += 1;
                }
            }
        }
        boundary = union_ex(&boundary);
        let (contours, holes) = split_expolygon(&boundary);
        // Polygons in which is possible traveling without crossing perimeters of another object.
        // A convex hull allows removing unnecessary detour caused by following the boundary of the object.
        let mut result_boundary: Vec<ExPolygon> = diff_ex_polygons(
            &offset_polygon(&crate::geometry::convex_hull_polygons(&contours), 2.0f32 * perimeter_spacing),
            &offset_polygons(&contours, perimeter_spacing + perimeter_offset),
        );
        // All holes are extended for forcing travel around the outer perimeter of a hole when a hole is crossed.
        result_boundary.extend(diff_ex_polygons(&offset_polygons(&holes, perimeter_spacing), &offset_polygons(&holes, perimeter_offset)));
        union_ex(&result_boundary)
    }

    /// `AvoidCrossingPerimeters::init_layer` of the `#if 0` block.
    pub(super) trait InitLayerLegacy {
        fn init_layer(&mut self, layer: &Layer, owner: &LayerOwner<'_>);
    }

    impl InitLayerLegacy for AvoidCrossingPerimeters {
        // AvoidCrossingPerimeters.cpp:1835-1865
        // void AvoidCrossingPerimeters::init_layer(const Layer &layer)
        fn init_layer(&mut self, layer: &Layer, owner: &LayerOwner<'_>) {
            self.m_internal.boundaries.clear();
            self.m_external.boundaries.clear();

            self.m_internal.boundaries = to_polygons(&get_boundary(layer));
            self.m_external.boundaries = to_polygons(&get_boundary_external(layer, owner));

            let mut bbox = get_extents(&self.m_internal.boundaries);
            bbox.expand(SCALED_EPSILON_D as i64);
            let mut bbox_external = get_extents(&self.m_external.boundaries);
            bbox_external.expand(SCALED_EPSILON_D as i64);
            let mut bbox_slice = crate::ex_polygon::get_extents(&layer.lslices);
            bbox_slice.expand(SCALED_EPSILON_D as i64);

            self.m_internal.bbox = BoundingBoxF::from_points_minmax(
                Vec2d { x: bbox.min.x as f64, y: bbox.min.y as f64 },
                Vec2d { x: bbox.max.x as f64, y: bbox.max.y as f64 },
            );
            self.m_external.bbox = BoundingBoxF::from_points_minmax(
                Vec2d { x: bbox_external.min.x as f64, y: bbox_external.min.y as f64 },
                Vec2d { x: bbox_external.max.x as f64, y: bbox_external.max.y as f64 },
            );

            self.m_internal.grid.set_bbox(bbox);
            //FIX1ME 1mm grid?
            self.m_internal.grid.create_from_polygons(&self.m_internal.boundaries, crate::scaled(1.0));
            self.m_external.grid.set_bbox(bbox_external);
            //FIX1ME 1mm grid?
            self.m_external.grid.create_from_polygons(&self.m_external.boundaries, crate::scaled(1.0));
            self.m_grid_lslice.set_bbox(bbox_slice);
            //FIX1ME 1mm grid?
            self.m_grid_lslice.create_from_polygons(&to_polygons(&layer.lslices), crate::scaled(1.0));

            init_boundary_distances(&mut self.m_internal);
            init_boundary_distances(&mut self.m_external);
        }
    }
}

/// The functions of this file that are private in the C++ (`static`), for
/// `tests/avoid_crossing_perimeters_native_parity.rs`, which compares them with the pin.
#[doc(hidden)]
pub mod native_test_api {
    use super::*;

    /// `inner_offset(ex_polygons, offset_dis)`
    pub fn inner_offset(ex_polygons: &[ExPolygon], offset_dis: f64) -> Vec<ExPolygon> {
        super::inner_offset(ex_polygons, offset_dis)
    }

    /// `init_boundary(&boundary, std::move(polygons))`, then `boundary.boundaries_params`.
    pub fn boundary_params(polygons: &[Polygon]) -> Vec<Vec<f32>> {
        let mut boundary = Boundary::new();
        init_boundary(&mut boundary, polygons.to_vec());
        boundary.boundaries_params
    }

    /// `get_closest_lines_in_radius(boundary.grid, center, search_radius)`: border index, line index, point.
    pub fn get_closest_lines_in_radius(polygons: &[Polygon], center: &Point, search_radius: f32) -> Vec<(usize, usize, Point)> {
        let mut boundary = Boundary::new();
        init_boundary(&mut boundary, polygons.to_vec());
        super::get_closest_lines_in_radius(&boundary.grid, center, search_radius).iter().map(|c| (c.border_idx, c.line_idx, c.point)).collect()
    }

    /// The intersections of the travel with the boundary as `avoid_perimeters_inner` collects them
    /// (AvoidCrossingPerimeters.cpp:560-562, 585-590: visited, their distance set, sorted along the
    /// travel), then `extend_for_closest_lines(intersections, boundary, start, end, search_radius)`:
    /// border index, line index, point, distance, do_not_remove.
    pub fn extend_for_closest_lines(polygons: &[Polygon], start: &Point, end: &Point, search_radius: f32) -> Vec<(usize, usize, Point, f32, bool)> {
        let mut boundary = Boundary::new();
        init_boundary(&mut boundary, polygons.to_vec());
        let mut intersections = AllIntersectionsVisitor::with_line(&boundary.grid, Line::new(*start, *end)).run();
        let dir = cast_d(Point::new(end.x - start.x, end.y - start.y));
        for intersection in intersections.iter_mut() {
            let p = boundary.boundaries[intersection.border_idx].points[intersection.line_idx];
            let dx = (intersection.point.x - p.x) as f32;
            let dy = (intersection.point.y - p.y) as f32;
            let dist_from_line_begin = (dx * dx + dy * dy).sqrt();
            intersection.distance = boundary.boundaries_params[intersection.border_idx][intersection.line_idx] + dist_from_line_begin;
        }
        crate::clipper::libcxx_sort(&mut intersections, |l, r| {
            let v = cast_d(Point::new(r.point.x - l.point.x, r.point.y - l.point.y));
            v.x * dir.x + v.y * dir.y > 0.0
        });
        super::extend_for_closest_lines(&intersections, &boundary, start, end, search_radius)
            .iter()
            .map(|i| (i.border_idx, i.line_idx, i.point, i.distance, i.do_not_remove))
            .collect()
    }

    /// `simplify_travel(boundary, travel)`; a travel point is its point, border index and do_not_remove.
    pub fn simplify_travel(polygons: &[Polygon], travel: &[(Point, i32, bool)]) -> Vec<(Point, i32, bool)> {
        let mut boundary = Boundary::new();
        init_boundary(&mut boundary, polygons.to_vec());
        let travel: Vec<TravelPoint> = travel.iter().map(|t| TravelPoint { point: t.0, border_idx: t.1, do_not_remove: t.2 }).collect();
        super::simplify_travel(&boundary, &travel).iter().map(|t| (t.point, t.border_idx, t.do_not_remove)).collect()
    }

    /// The grid of `AvoidCrossingPerimeters::init_layer` over `ex_polygons` (their extents grown by
    /// SCALED_EPSILON, a resolution of `coord_t(scale_(1.))`), their bounding boxes, then
    /// `any_expolygon_contains` of a line (two points) or of a polyline.
    pub fn any_expolygon_contains(ex_polygons: &[ExPolygon], travel: &[Point]) -> bool {
        let bboxes: Vec<BoundingBox> = ex_polygons.iter().map(crate::ex_polygon::get_extents_expoly).collect();
        let mut bbox_slice = crate::ex_polygon::get_extents(ex_polygons);
        bbox_slice.expand(SCALED_EPSILON_D as i64);
        let mut grid = EdgeGrid::new();
        grid.set_bbox(bbox_slice);
        grid.create_from_polygons(&to_polygons(ex_polygons), crate::scaled(1.0));
        if travel.len() == 2 {
            Line::new(travel[0], travel[1]).any_expolygon_contains(ex_polygons, &bboxes, &grid)
        } else {
            Polyline::from_points(travel.to_vec()).any_expolygon_contains(ex_polygons, &bboxes, &grid)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_square() -> Polygon {
        Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(1_000_000, 0),
            Point::new(1_000_000, 1_000_000),
            Point::new(0, 1_000_000),
        ])
    }

    #[test]
    fn test_precompute_polygon_distances() {
        let square = make_square();
        let mut distances: Vec<f32> = Vec::new();
        precompute_polygon_distances(&square, &mut distances);
        // 4 vertices -> 5 entries (closing distance at the end)
        assert_eq!(distances.len(), 5);
        // Square perimeter = 4 * 1mm = 4_000_000 scaled units
        assert!((distances[4] - 4_000_000.0).abs() < 1.0);
    }

    #[test]
    fn test_to_polyline() {
        let travel = vec![
            TravelPoint { point: Point::new(0, 0), border_idx: -1, do_not_remove: false },
            TravelPoint { point: Point::new(100, 100), border_idx: -1, do_not_remove: false },
        ];
        let pl = to_polyline(&travel);
        assert_eq!(pl.points.len(), 2);
    }

    #[test]
    fn test_init_boundary() {
        let mut boundary = Boundary::new();
        init_boundary(&mut boundary, vec![make_square()]);
        assert_eq!(boundary.boundaries.len(), 1);
        assert_eq!(boundary.boundaries_params.len(), 1);
        assert_eq!(boundary.boundaries_params[0].len(), 5);
    }

    #[test]
    fn test_once_modifiers() {
        let mut acp = AvoidCrossingPerimeters::new();
        assert!(acp.disabled_once()); // disabled for first move
        acp.reset_once_modifiers();
        assert!(!acp.disabled_once());
        acp.use_external_mp_once();
        assert!(acp.used_external_mp_once());
    }

    #[test]
    fn test_find_first_different_vertex() {
        let square = make_square();
        let pt = Point::new(0, 0);
        // forward from index 0 (== pt) should find next distinct vertex
        let v = find_first_different_vertex(&square, 0, &pt, true);
        assert_ne!(v, pt);
    }
}
