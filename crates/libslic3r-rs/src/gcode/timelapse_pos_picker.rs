//! Faithful 1:1 port of `GCode/TimelapsePosPicker.{hpp,cpp}` from BambuStudio.
//!
//! The timelapse position picker selects a safe location for the toolhead to
//! move to while a timelapse photo is taken, avoiding collisions with already
//! printed objects, the camera occlusion zone and the rod clearance area.
//!
//! Porting status (see the module-level note and `PORT_LEDGER.json`): the pure
//! geometry routine `pick_pos_internal` is ported faithfully and is fully
//! self-contained. The `TimelapsePosPicker` class methods are BLOCKED on
//! libslic3r types/config that are not yet ported in this crate
//! (`PrintInstance`, `PrintObject::instances/has_raft/slicing_parameters/config`,
//! `Print::get_fake_wipe_tower/wipe_tower_data`, and the
//! `printable_area`/`bed_exclude_area`/`extruder_printable_area`/
//! `extruder_printable_height`/`initial_layer_print_height` config options, plus
//! the vector form of `nozzle_diameter` and the `TimelapseType` enum). They are
//! intentionally NOT faked here.

// TimelapsePosPicker.cpp:1   #include "ClipperUtils.hpp"
// TimelapsePosPicker.cpp:2   #include "TimelapsePosPicker.hpp"
// TimelapsePosPicker.cpp:3   #include "Layer.hpp"
use crate::clipper_utils::intersection_pl;
use crate::geometry::{ExPolygons, Point, Polyline};
use crate::scale;
use crate::utils::next_idx_modulo;

// TimelapsePosPicker.hpp:12  const Point DefaultTimelapsePos = Point(0, 0);
pub const DEFAULT_TIMELAPSE_POS: Point = Point { x: 0, y: 0 };
// TimelapsePosPicker.hpp:13  const Point DefaultCameraPos = Point(0, 0);
pub const DEFAULT_CAMERA_POS: Point = Point { x: 0, y: 0 };

// TimelapsePosPicker.cpp:5   constexpr int FILTER_THRESHOLD = 5;
#[allow(dead_code)]
pub(crate) const FILTER_THRESHOLD: i32 = 5;
// TimelapsePosPicker.cpp:6   constexpr int MAX_CANDIDATE_SIZE = 5;
const MAX_CANDIDATE_SIZE: usize = 5;

/// TimelapsePosPicker.cpp:364-441
///
/// Selects the nearest position within the given safe areas relative to the
/// current position.
///
/// This function determines the closest point in the safe areas to the provided
/// current position. If the current position is already inside a safe area, it
/// returns the current position. If no safe areas are defined, returns the
/// default timelapse position.
///
/// * `curr_pos` - The reference point representing the current position.
/// * `safe_areas` - A collection of extended polygons defining the safe areas.
/// * returns the nearest point within the safe areas or the default timelapse
///   position if no safe areas exist.
pub fn pick_pos_internal(
    curr_pos: &Point,
    safe_areas: &ExPolygons,
    path_collision_area: &ExPolygons,
    detect_path_collision: bool,
) -> Point {
    // TimelapsePosPicker.cpp:366-373
    // struct CandidatePoint { double dist; Point point; bool operator< { return dist < other.dist; } }

    // TimelapsePosPicker.cpp:375-376
    // if any safe area contains curr_pos, return curr_pos
    if safe_areas.iter().any(|p| p.contains_point(curr_pos)) {
        return *curr_pos;
    }

    // TimelapsePosPicker.cpp:378-379
    if safe_areas.is_empty() {
        return DEFAULT_TIMELAPSE_POS;
    }

    // TimelapsePosPicker.cpp:381  std::priority_queue<CandidatePoint> max_heap;
    // C++ uses a max-heap (default std::priority_queue ordering by operator<,
    // which compares `dist`). We model it with a Vec and explicit pop-of-max
    // (see `peek_max`/`pop_max`).
    let mut max_heap: Vec<CandidatePoint> = Vec::new();

    // TimelapsePosPicker.cpp:383
    // constexpr double candidate_point_segment = scale_(5), weight_of_camera = 1./3.;
    let candidate_point_segment: f64 = scale(5.0) as f64;
    let weight_of_camera: f64 = 1.0 / 3.0;

    // TimelapsePosPicker.cpp:384-388
    // move distance + Camera occlusion penalty function
    let penalty_func = |curr_post: &Point, camera_pos: &Point, candidatet: &Point| -> f64 {
        // (curr_post - candidatet).cwiseAbs().sum()
        let move_l1 = ((curr_post.x - candidatet.x).abs() + (curr_post.y - candidatet.y).abs()) as f64;
        // (CameraPos - candidatet).cwiseAbs().sum()
        let cam_l1 = ((camera_pos.x - candidatet.x).abs() + (camera_pos.y - candidatet.y).abs()) as f64;
        move_l1 - weight_of_camera * cam_l1
    };

    // TimelapsePosPicker.cpp:390-421
    for expoly in safe_areas {
        // Polygons polys = to_polygons(expoly);
        let polys = expoly.to_polygons();
        for poly in &polys {
            for idx in 0..poly.points.len() {
                // TimelapsePosPicker.cpp:394-395
                let mut best_penalty: f64 = f64::MAX;
                let mut best_candidate: Point = DEFAULT_TIMELAPSE_POS; // the best candidate from current line
                //std::vector<Point> candidate_source;

                // TimelapsePosPicker.cpp:397
                let next = poly.points[next_idx_modulo(idx, poly.points.len())];
                let seg_l1 =
                    ((poly.points[idx].x - next.x).abs() + (poly.points[idx].y - next.y).abs()) as f64;
                if seg_l1 < candidate_point_segment {
                    // TimelapsePosPicker.cpp:398-399
                    // only check the start point if the line is short
                    best_candidate = poly.points[idx];
                    best_penalty = penalty_func(curr_pos, &DEFAULT_CAMERA_POS, &best_candidate);
                } else {
                    // TimelapsePosPicker.cpp:401  Point direct_of_line = next - cur;
                    let mut direct_of_line = next - poly.points[idx];
                    // TimelapsePosPicker.cpp:402  double length_L1 = direct_of_line.cwiseAbs().sum();
                    let length_l1: f64 = (direct_of_line.x.abs() + direct_of_line.y.abs()) as f64;
                    // TimelapsePosPicker.cpp:403
                    // for long line use 5mm segmentation to check
                    let num_steps: i32 = (length_l1 / candidate_point_segment) as i32;
                    // TimelapsePosPicker.cpp:404-406
                    // divide by length_L1 instead of steps, prevent lose accuracy for the step length
                    direct_of_line.x =
                        (direct_of_line.x as f64 * candidate_point_segment / length_l1) as i64;
                    direct_of_line.y =
                        (direct_of_line.y as f64 * candidate_point_segment / length_l1) as i64;
                    // TimelapsePosPicker.cpp:407-415
                    for line_seg_i in 0..=num_steps {
                        let candidate = poly.points[idx] + direct_of_line * (line_seg_i as i64);
                        let dist = penalty_func(curr_pos, &DEFAULT_CAMERA_POS, &candidate);
                        if dist < best_penalty {
                            best_penalty = dist;
                            best_candidate = candidate;
                        } //only push the best point into heap for the whole line
                    }
                }
                // TimelapsePosPicker.cpp:417  max_heap.push({best_penalty, best_candidate});
                max_heap.push(CandidatePoint {
                    dist: best_penalty,
                    point: best_candidate,
                });
                // TimelapsePosPicker.cpp:418
                // if (max_heap.size() > MAX_CANDIDATE_SIZE) max_heap.pop();
                if max_heap.len() > MAX_CANDIDATE_SIZE {
                    pop_max(&mut max_heap);
                }
            }
        }
    }

    // TimelapsePosPicker.cpp:423-428
    let mut top_candidates: Vec<Point> = Vec::new();
    while !max_heap.is_empty() {
        // top_candidates.push_back(max_heap.top().point); max_heap.pop();
        let top = peek_max(&max_heap);
        top_candidates.push(top.point);
        pop_max(&mut max_heap);
    }
    top_candidates.reverse();

    // TimelapsePosPicker.cpp:430-438
    for p in &top_candidates {
        if !detect_path_collision {
            return *p;
        }

        // Polyline path(curr_pos, p);
        let path = Polyline::from_points(vec![*curr_pos, *p]);

        // if (intersection_pl(path, path_collision_area).empty()) return p;
        if intersection_pl(std::slice::from_ref(&path), path_collision_area).is_empty() {
            return *p;
        }
    }

    // TimelapsePosPicker.cpp:440
    DEFAULT_TIMELAPSE_POS
}

/// TimelapsePosPicker.cpp:366-373  struct CandidatePoint { double dist; Point point; }
///
/// `operator<` compares `dist`, so `std::priority_queue<CandidatePoint>` is a
/// max-heap on `dist`.
#[derive(Clone, Copy)]
struct CandidatePoint {
    dist: f64,
    point: Point,
}

/// Helper modelling `std::priority_queue::top()` for the `CandidatePoint`
/// max-heap used in `pick_pos_internal` (the element with the greatest `dist`).
fn peek_max(heap: &[CandidatePoint]) -> CandidatePoint {
    let mut max_i = 0usize;
    for (i, c) in heap.iter().enumerate() {
        if c.dist > heap[max_i].dist {
            max_i = i;
        }
    }
    heap[max_i]
}

/// Helper modelling `std::priority_queue::pop()`: remove the element with the
/// greatest `dist`.
fn pop_max(heap: &mut Vec<CandidatePoint>) {
    if heap.is_empty() {
        return;
    }
    let mut max_i = 0usize;
    for (i, c) in heap.iter().enumerate() {
        if c.dist > heap[max_i].dist {
            max_i = i;
        }
    }
    heap.swap_remove(max_i);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{ExPolygon, Polygon};

    /// A point already inside a safe area returns itself (cpp:375-376).
    #[test]
    fn test_pick_pos_internal_inside_returns_curr() {
        let square = Polygon {
            points: vec![
                Point::new(scale(0.0), scale(0.0)),
                Point::new(scale(100.0), scale(0.0)),
                Point::new(scale(100.0), scale(100.0)),
                Point::new(scale(0.0), scale(100.0)),
            ],
        };
        let safe = vec![ExPolygon::new(square)];
        let curr = Point::new(scale(50.0), scale(50.0));
        let res = pick_pos_internal(&curr, &safe, &Vec::new(), false);
        assert_eq!(res, curr);
    }

    /// Empty safe areas yield the default timelapse position (cpp:378-379).
    #[test]
    fn test_pick_pos_internal_empty_returns_default() {
        let curr = Point::new(scale(10.0), scale(10.0));
        let res = pick_pos_internal(&curr, &Vec::new(), &Vec::new(), false);
        assert_eq!(res, DEFAULT_TIMELAPSE_POS);
    }

    /// With a safe area not containing curr and no path-collision detection, a
    /// boundary point of the safe area is returned (cpp:390-432).
    #[test]
    fn test_pick_pos_internal_outside_returns_boundary() {
        let square = Polygon {
            points: vec![
                Point::new(scale(10.0), scale(10.0)),
                Point::new(scale(20.0), scale(10.0)),
                Point::new(scale(20.0), scale(20.0)),
                Point::new(scale(10.0), scale(20.0)),
            ],
        };
        let safe = vec![ExPolygon::new(square)];
        let curr = Point::new(scale(0.0), scale(0.0));
        let res = pick_pos_internal(&curr, &safe, &Vec::new(), false);
        // Must be a real candidate, not the fallback default.
        assert_ne!(res, DEFAULT_TIMELAPSE_POS);
    }
}
