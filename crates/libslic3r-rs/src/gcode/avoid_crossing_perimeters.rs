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
//! # Blocked symbols (NOT ported — see module-level `// BLOCKED:` comments)
//!
//! The following depend on libslic3r infrastructure that is not yet faithfully
//! ported into this crate and are therefore omitted rather than faked:
//!
//! - `inner_offset`, `get_support_polygons`, `get_boundary`,
//!   `get_boundary_external`: require `variable_offset_inner_ex`
//!   (ClipperUtils.cpp:1390) which is NOT yet ported (see
//!   `elephant_foot_compensation.rs:811`), plus `SupportLayer`,
//!   `PrintObject::instances()` traversal.
//! - `need_wipe`: requires the `GCode` generator class
//!   (`gcodegen.config()`, `gcodegen.writer().filament()`), which is not
//!   ported (the Rust `gcode::generator::GCode` is a text container, not the
//!   path-planning generator).
//! - `AvoidCrossingPerimeters::travel_to` and
//!   `AvoidCrossingPerimeters::init_layer`: depend on the `GCode` generator
//!   and on `get_boundary`/`get_boundary_external` above.
//!
//! The perimeter-spacing helpers (`get_default_perimeter_spacing`,
//! `get_perimeter_spacing`, `get_perimeter_spacing_external`,
//! `get_external_perimeter_width`) became portable once the config hierarchy
//! was wired (`layer.object().print().config()`,
//! `LayerRegion::region()`/`flow`) and are ported below.

// AvoidCrossingPerimeters.cpp:1-14 — includes
use crate::edge_grid::EdgeGrid;
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
struct TravelPoint {
    point: Point,
    // Index of the polygon containing this point. A negative value indicates that the point is not on any border.
    border_idx: i32,
    // simplify_travel() doesn't remove this point.
    do_not_remove: bool, // = false
}

// AvoidCrossingPerimeters.cpp:27 — struct Intersection
#[derive(Clone)]
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
#[derive(Clone)]
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
    let radius_vector = Point::new(search_radius as i64, search_radius as i64);
    let mut visitor = MinDistanceVisitor::new(grid, *center, (search_radius * search_radius) as f64);
    grid.visit_cells_intersecting_box(
        BoundingBox::from_points_minmax(
            Point::new(center.x - radius_vector.x, center.y - radius_vector.y),
            Point::new(center.x + radius_vector.x, center.y + radius_vector.y),
        ),
        |iy, ix| visitor.visit(iy, ix),
    );
    let mut closest_lines = visitor.closest_lines;
    closest_lines.sort_by(|l, r| {
        let dl = {
            let d = cast_d(Point::new(center.x - l.point.x, center.y - l.point.y));
            d.x * d.x + d.y * d.y
        };
        let dr = {
            let d = cast_d(Point::new(center.x - r.point.x, center.y - r.point.y));
            d.x * d.x + d.y * d.y
        };
        dl.partial_cmp(&dr).unwrap_or(std::cmp::Ordering::Equal)
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

// AvoidCrossingPerimeters.cpp:510-529 — get_perimeter_spacing_external
// called by get_boundary_external()
//
// PARTIAL SIGNATURE: C++ reaches the Print through the parent pointer chain
// `layer.object()->print()->objects()`. The Rust upward views (`ObjectRef`/
// `PrintRef`) are config-only snapshots, so the owning `Print` is threaded in
// by the caller — same convention as `LayerRegion::flow`'s threaded
// `layer_height`. The body is otherwise line-by-line faithful.
#[allow(dead_code)] // sole C++ caller get_boundary_external() is BLOCKED (see module docs)
fn get_perimeter_spacing_external(layer: &Layer, print: &crate::print::Print) -> f32 {
    let mut regions_count: usize = 0;
    let mut perimeter_spacing: f32 = 0.0;
    // AvoidCrossingPerimeters.cpp:515-521
    // C++: for (const PrintObject *object : layer.object()->print()->objects())
    //          if (const Layer *l = object->get_layer_at_printz(layer.print_z, EPSILON); l)
    for object in print.objects() {
        if let Some(l_idx) = object.get_layer_at_printz(layer.print_z, crate::libslic3r::EPSILON) {
            let l = &object.layers()[l_idx];
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
#[allow(dead_code)] // sole C++ caller AvoidCrossingPerimeters::init_layer() is BLOCKED (see module docs)
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
        intersections.sort_by(|l, r| {
            // C++ predicate: less(l,r) == (r.point - l.point).cast<double>().dot(dir) > 0.
            // Derived total order: dot>0 => Less, dot<0 => Greater, dot==0 => Equal.
            let v = cast_d(Point::new(r.point.x - l.point.x, r.point.y - l.point.y));
            let dot = v.x * dir.x + v.y * dir.y;
            if dot > 0.0 {
                std::cmp::Ordering::Less
            } else if dot < 0.0 {
                std::cmp::Ordering::Greater
            } else {
                std::cmp::Ordering::Equal
            }
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

// AvoidCrossingPerimeters.cpp:713-737 — any_expolygon_contains (Line)
// Check if anyone of ExPolygons contains whole travel.
// called by need_wipe() and AvoidCrossingPerimeters::travel_to()
fn any_expolygon_contains_line(
    ex_polygons: &[crate::geometry::ExPolygon],
    ex_polygons_bboxes: &[BoundingBox],
    grid_lslice: &EdgeGrid,
    travel: &Line,
) -> bool {
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

// AvoidCrossingPerimeters.cpp:739-766 — any_expolygon_contains (Polyline)
// Check if anyone of ExPolygons contains whole travel.
// called by need_wipe()
fn any_expolygon_contains_polyline(
    ex_polygons: &[crate::geometry::ExPolygon],
    ex_polygons_bboxes: &[BoundingBox],
    grid_lslice: &EdgeGrid,
    travel: &Polyline,
) -> bool {
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
            if travel.points.iter().all(|point| bbox.contains_point(point))
                && ex_polygon.contains(&travel.points[0], true)
            {
                return true;
            }
        }
    }
    false
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
// PARTIAL: the public `travel_to` and `init_layer` methods are BLOCKED (they
// require the `GCode` generator class — `gcodegen.writer()`, `gcodegen.config()`,
// `gcodegen.origin()` — plus `get_boundary`/`get_boundary_external`, which are
// blocked on `variable_offset_inner_ex`; see module docs), so only the state
// and the trivial once-modifiers accessors are ported here. The heavy lifting
// (`avoid_perimeters`, `avoid_perimeters_inner`, `simplify_travel`, the
// perimeter-spacing helpers, the boundary builders, etc.) is ported above as
// free functions matching the C++ statics.
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

impl Default for AvoidCrossingPerimeters {
    fn default() -> Self {
        Self::new()
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
