//! Port of `GCode/BedExcludeChecker.cpp` and `GCode/BedExcludeChecker.hpp`.
//!
//! Runtime-combined exclude areas used only for toolpath collision checks.
//! Callers are responsible for collecting the business-specific areas to inspect
//! (for example: bed exclude area + optional clumping detection area).

use crate::clipper_utils::diff_pl_clib;
use crate::edge_grid::EdgeGrid as Grid; // C++ `EdgeGrid::Grid`
use crate::gcode::g_code_processor::{EMoveType, GCodeProcessorResult, MoveVertex};
use crate::extrusion_entity::ExtrusionRole;
use crate::geometry::polygon::{get_extents_polygons, polygon_is_convex_poly};
use crate::geometry::{
    liang_barsky_line_clipping_interval, segments_intersect, BoundingBox, Point, Polygon, Polyline,
};
use crate::Coord;

// BedExcludeChecker.cpp:16
const K_GRID_CELL_SIZE_MM: f64 = 2.0;

// BedExcludeChecker.cpp:18 `scaled_point`
fn scaled_point(point: &[f32; 3]) -> Point {
    Point::new_scale(point[0] as f64, point[1] as f64)
}

// BedExcludeChecker.cpp:23 `is_valid_extrusion_move`
fn is_valid_extrusion_move(mv: &MoveVertex) -> bool {
    mv.move_type == EMoveType::Extrude
        && mv.extrusion_role != ExtrusionRole::Custom
        && mv.width > 0.0f32
        && mv.height > 0.0f32
}

// BedExcludeChecker.cpp:29 `visit_extrusion_segments`
fn visit_extrusion_segments<F>(gcode_result: &GCodeProcessorResult, mut visitor: F) -> bool
where
    F: FnMut(&Point, &Point) -> bool,
{
    for i in 1..gcode_result.moves.len() {
        let prev = &gcode_result.moves[i - 1];
        let curr = &gcode_result.moves[i];
        if !is_valid_extrusion_move(curr) {
            continue;
        }

        let mut segment_start = scaled_point(&prev.position);
        if curr.is_arc_move_with_interpolation_points() {
            for interpolation_point in &curr.interpolation_points {
                let segment_end = scaled_point(interpolation_point);
                if segment_start != segment_end && !visitor(&segment_start, &segment_end) {
                    return false;
                }
                segment_start = segment_end;
            }
        }

        let segment_end = scaled_point(&curr.position);
        if segment_start != segment_end && !visitor(&segment_start, &segment_end) {
            return false;
        }
    }
    true
}

// BedExcludeChecker.cpp:55 `clip_segment_to_bbox`
fn clip_segment_to_bbox(
    start: &Point,
    end: &Point,
    bbox: &BoundingBox,
    clipped_start: &mut Point,
    clipped_end: &mut Point,
) -> bool {
    *clipped_start = *start;
    *clipped_end = *end;
    if bbox.contains_point(clipped_start) && bbox.contains_point(clipped_end) {
        return true;
    }

    let line_start = (start.x as f64, start.y as f64);
    let line_vector = ((end.x - start.x) as f64, (end.y - start.y) as f64);
    let bb_min = (bbox.min.x as f64, bbox.min.y as f64);
    let bb_max = (bbox.max.x as f64, bbox.max.y as f64);
    let interval = match liang_barsky_line_clipping_interval(line_start, line_vector, bb_min, bb_max) {
        Some(interval) => interval,
        None => return false,
    };

    // `.cast<coord_t>()` truncates.
    *clipped_start = Point::new(
        (line_start.0 + interval.0 * line_vector.0) as Coord,
        (line_start.1 + interval.0 * line_vector.1) as Coord,
    );
    *clipped_end = Point::new(
        (line_start.0 + interval.1 * line_vector.0) as Coord,
        (line_start.1 + interval.1 * line_vector.1) as Coord,
    );
    true
}

// BedExcludeChecker.cpp:75 `segment_intersects_exclude_grid`
fn segment_intersects_exclude_grid(
    start: &Point,
    end: &Point,
    exclude_bbox: &BoundingBox,
    grid: &Grid,
    exclude_polygons: &[&Polygon],
) -> bool {
    let mut clipped_start = Point::new(0, 0);
    let mut clipped_end = Point::new(0, 0);
    if !clip_segment_to_bbox(start, end, exclude_bbox, &mut clipped_start, &mut clipped_end) {
        return false;
    }

    let mut tested_polygon: Vec<u8> = vec![0; exclude_polygons.len()];
    let mut intersects = false;
    let visit_candidate_cell = |row: usize, col: usize| -> bool {
        let cell_data_range = grid.cell_data_range_at(row, col);
        for it in cell_data_range {
            let polygon_id = it.0;
            if tested_polygon[polygon_id] == 0 {
                tested_polygon[polygon_id] = 1;
                let polygon = exclude_polygons[polygon_id];
                if polygon.contains(start) || polygon.contains(end) {
                    intersects = true;
                    return false;
                }
            }

            let edge = grid.segment(*it);
            if segments_intersect(*start, *end, edge.a, edge.b) {
                intersects = true;
                return false;
            }
        }
        true
    };

    // `need_consider_eps` is true.
    grid.visit_cells_intersecting_line_eps(clipped_start, clipped_end, visit_candidate_cell);
    intersects
}

/// BedExcludeChecker.cpp:119
/// C++: bool toolpath_intersects_bed_exclude_area_2d(const GCodeProcessorResult& gcode_result,
///          const std::vector<Polygon>& combined_exclude_area_for_toolpath_check)
pub fn toolpath_intersects_bed_exclude_area_2d(
    gcode_result: &GCodeProcessorResult,
    combined_exclude_area_for_toolpath_check: &[Polygon],
) -> bool {
    let mut exclude_polygons: Vec<&Polygon> =
        Vec::with_capacity(combined_exclude_area_for_toolpath_check.len());
    for polygon in combined_exclude_area_for_toolpath_check {
        if !polygon.points.is_empty() {
            exclude_polygons.push(polygon);
        }
    }

    if exclude_polygons.is_empty() {
        return false;
    }

    // The exact check should be bounded by the runtime exclude areas themselves.
    // wrapping_exclude_area may sit on / beyond the printable_area edge, so clipping
    // to printable_area may incorrectly discard the very segments we need to test.
    let exclude_bbox = get_extents_polygons(combined_exclude_area_for_toolpath_check);
    if !exclude_bbox.is_defined() {
        return false;
    }

    // The grid's contours are the non-empty
    // polygons, in order, so a contour index is an index into `exclude_polygons`.
    let mut grid = Grid::new();
    let grid_polygons: Vec<Polygon> = exclude_polygons.iter().map(|p| (*p).clone()).collect();
    grid.create_from_polygons(&grid_polygons, crate::scale(K_GRID_CELL_SIZE_MM));

    let all_segments_clear = visit_extrusion_segments(gcode_result, |start, end| {
        !segment_intersects_exclude_grid(start, end, &exclude_bbox, &grid, &exclude_polygons)
    });
    !all_segments_clear
}

/// BedExcludeChecker.cpp:180
/// C++: bool toolpath_exceeds_boundary_2d(const GCodeProcessorResult& gcode_result, const Polygon& boundary)
/// Returns true when an extrusion segment leaves the supplied boundary.
pub fn toolpath_exceeds_boundary_2d(gcode_result: &GCodeProcessorResult, boundary: &Polygon) -> bool {
    if boundary.points.len() < 3 {
        return false;
    }

    let boundary_is_convex = polygon_is_convex_poly(boundary);
    let all_segments_inside = visit_extrusion_segments(gcode_result, |start, end| {
        if !boundary.contains(start) || !boundary.contains(end) {
            return false;
        }

        // Endpoints are sufficient for convex boundaries. Clip only for the
        // uncommon concave case where a segment may leave and re-enter.
        boundary_is_convex
            || diff_pl_clib(
                &[Polyline::from_points(vec![*start, *end])],
                std::slice::from_ref(boundary),
            )
            .is_empty()
    });
    !all_segments_inside
}

#[cfg(test)]
mod tests {
    use super::*;

    fn extrude(x: f32, y: f32) -> MoveVertex {
        let mut m = MoveVertex::default();
        m.move_type = EMoveType::Extrude;
        m.extrusion_role = ExtrusionRole::ExternalPerimeter;
        m.width = 0.4;
        m.height = 0.2;
        m.position = [x, y, 0.2];
        m
    }

    fn result(points: &[(f32, f32)]) -> GCodeProcessorResult {
        let mut r = GCodeProcessorResult::default();
        r.moves = points.iter().map(|p| extrude(p.0, p.1)).collect();
        r
    }

    fn square(x0: f64, y0: f64, x1: f64, y1: f64) -> Polygon {
        Polygon::from_points(vec![
            Point::new_scale(x0, y0),
            Point::new_scale(x1, y0),
            Point::new_scale(x1, y1),
            Point::new_scale(x0, y1),
        ])
    }

    #[test]
    fn a_path_through_the_exclude_area_is_found() {
        let area = vec![square(10.0, 10.0, 20.0, 20.0)];
        assert!(toolpath_intersects_bed_exclude_area_2d(&result(&[(0.0, 15.0), (30.0, 15.0)]), &area));
        assert!(toolpath_intersects_bed_exclude_area_2d(&result(&[(12.0, 12.0), (13.0, 13.0)]), &area));
        assert!(!toolpath_intersects_bed_exclude_area_2d(&result(&[(0.0, 5.0), (30.0, 5.0)]), &area));
        assert!(!toolpath_intersects_bed_exclude_area_2d(&result(&[(0.0, 15.0), (30.0, 15.0)]), &[]));
    }

    #[test]
    fn a_path_leaving_the_boundary_is_found() {
        let boundary = square(0.0, 0.0, 100.0, 100.0);
        assert!(!toolpath_exceeds_boundary_2d(&result(&[(10.0, 10.0), (90.0, 90.0)]), &boundary));
        assert!(toolpath_exceeds_boundary_2d(&result(&[(10.0, 10.0), (110.0, 90.0)]), &boundary));
        // A non-extrusion move is not looked at.
        let mut r = result(&[(10.0, 10.0), (110.0, 90.0)]);
        r.moves[1].move_type = EMoveType::Travel;
        assert!(!toolpath_exceeds_boundary_2d(&r, &boundary));
    }
}
