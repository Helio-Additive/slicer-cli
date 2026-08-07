//! R623 — unit tests for the leaf helpers of C++'s LIVE wall-gap chain.
//!
//! The chain is
//!     generate_support_wall_new (WipeTower.cpp:5030)
//!       -> contrust_gap_for_skip_points (:595)
//!         -> remove_points_from_polygon (:510)
//!       -> WipeTowerWriter::generate_path (:1249)
//!
//! R622 established that the OTHER chain — `generate_support_wall` (:5081) and
//! `remove_points_from_segment` (:298) — is dead: both of its call sites (:3661,
//! :4977) are commented out. These helpers belong to the live side.
//!
//! They are ported and tested ahead of `remove_points_from_polygon` itself,
//! because that function is meaningless until they are correct and, since
//! nothing calls them yet, gcode parity cannot check them. These tests are the
//! only check that the port is faithful; they are written against the C++
//! sources at WipeTower.cpp:230, :337 and :399.
//!
//! (This is an integration test rather than a `#[cfg(test)]` module because
//! `cargo test --lib` does not compile in this crate — a pre-existing condition,
//! which is why all eight guard suites are integration tests too.)

use slicer::gcode::wipe_tower::{
    insert_points, move_point_along_polygon, ray_intersetion_line, PointWithFlag,
};
use slicer::gcode::wipe_tower::Vec2f;

/// A 10mm square, counter-clockwise from the origin.
fn square() -> Vec<Vec2f> {
    vec![
        Vec2f::new(0.0, 0.0),
        Vec2f::new(10.0, 0.0),
        Vec2f::new(10.0, 10.0),
        Vec2f::new(0.0, 10.0),
    ]
}

#[test]
fn ray_hits_segment_ahead() {
    // Ray from (-1, 5) pointing +x meets the segment x=10 at (10, 5).
    let hit = ray_intersetion_line(
        Vec2f::new(-1.0, 5.0),
        Vec2f::new(1.0, 0.0),
        Vec2f::new(10.0, 0.0),
        Vec2f::new(10.0, 10.0),
    )
    .expect("ray should hit");
    assert!((hit.x - 10.0).abs() < 1e-4, "x = {}", hit.x);
    assert!((hit.y - 5.0).abs() < 1e-4, "y = {}", hit.y);
}

#[test]
fn ray_misses_segment_behind_it() {
    // Same segment, ray pointing -x: t1 < 0, so C++ returns no hit.
    assert!(ray_intersetion_line(
        Vec2f::new(-1.0, 5.0),
        Vec2f::new(-1.0, 0.0),
        Vec2f::new(10.0, 0.0),
        Vec2f::new(10.0, 10.0),
    )
    .is_none());
}

#[test]
fn ray_misses_when_parallel() {
    // Parallel ray and segment -> |denom| < EPSILON -> no hit.
    assert!(ray_intersetion_line(
        Vec2f::new(0.0, 5.0),
        Vec2f::new(1.0, 0.0),
        Vec2f::new(0.0, 0.0),
        Vec2f::new(10.0, 0.0),
    )
    .is_none());
}

#[test]
fn ray_misses_past_segment_end() {
    // Ray at y = 20 is beyond the segment's span -> t2 > 1 -> no hit.
    assert!(ray_intersetion_line(
        Vec2f::new(-1.0, 20.0),
        Vec2f::new(1.0, 0.0),
        Vec2f::new(10.0, 0.0),
        Vec2f::new(10.0, 10.0),
    )
    .is_none());
}

#[test]
fn move_forward_within_the_starting_edge() {
    // From (2,0) on edge 0, 3mm forward stays on edge 0 at (5,0).
    let r = move_point_along_polygon(&square(), Vec2f::new(2.0, 0.0), 0, 3.0, true, 7);
    assert_eq!(r.idx, 0);
    assert_eq!(r.pair_idx, 7);
    assert!((r.pos.x - 5.0).abs() < 1e-4, "pos = ({}, {})", r.pos.x, r.pos.y);
    assert!((r.pos.y - 0.0).abs() < 1e-4, "pos = ({}, {})", r.pos.x, r.pos.y);
    // dis_from_idx is measured from points[start_idx] = (0,0).
    assert!((r.dis_from_idx - 5.0).abs() < 1e-4, "d = {}", r.dis_from_idx);
}

#[test]
fn move_forward_across_a_corner() {
    // From (8,0), 5mm forward runs 2mm to the corner then 3mm up edge 1.
    let r = move_point_along_polygon(&square(), Vec2f::new(8.0, 0.0), 0, 5.0, true, 1);
    assert_eq!(r.idx, 1, "should land on the second edge");
    assert!((r.pos.x - 10.0).abs() < 1e-4, "pos = ({}, {})", r.pos.x, r.pos.y);
    assert!((r.pos.y - 3.0).abs() < 1e-4, "pos = ({}, {})", r.pos.x, r.pos.y);
    assert!((r.dis_from_idx - 3.0).abs() < 1e-4, "d = {}", r.dis_from_idx);
}

#[test]
fn move_backward_within_the_starting_edge() {
    // From (5,0) on edge 0, 3mm backward is (2,0); C++ measures dis_from_idx
    // from points[start_idx] = (0,0), so 2.0 — NOT mirrored from the forward case.
    let r = move_point_along_polygon(&square(), Vec2f::new(5.0, 0.0), 0, 3.0, false, 4);
    assert_eq!(r.idx, 0);
    assert_eq!(r.pair_idx, 4);
    assert!((r.pos.x - 2.0).abs() < 1e-4, "pos = ({}, {})", r.pos.x, r.pos.y);
    assert!((r.dis_from_idx - 2.0).abs() < 1e-4, "d = {}", r.dis_from_idx);
}

#[test]
fn move_backward_across_a_corner() {
    // From (2,0), 5mm backward: 2mm to the origin, then 3mm on up the closing
    // edge. C++ (:381-383) computes the position from the edge's FAR end,
    //     pos = points[i+1] - ratio * (points[i+1] - points[i])
    //         = (0,0) - 0.3 * ((0,0) - (0,10)) = (0, 3)
    // so the answer is 3mm ABOVE the origin, not 3mm below the top corner.
    // Asserting (0,7) here is the natural mistake and this test exists to catch
    // it: walking backwards past the origin on a CCW square goes UP the left
    // edge, and the distance is measured from where the walk arrived.
    let r = move_point_along_polygon(&square(), Vec2f::new(2.0, 0.0), 0, 5.0, false, 2);
    assert_eq!(r.idx, 3, "should land on the closing edge");
    assert!((r.pos.x - 0.0).abs() < 1e-4, "pos = ({}, {})", r.pos.x, r.pos.y);
    assert!((r.pos.y - 3.0).abs() < 1e-4, "pos = ({}, {})", r.pos.x, r.pos.y);
    // dis_from_idx = segmentLength - remainingDistance = 10 - 3 = 7.
    assert!((r.dis_from_idx - 7.0).abs() < 1e-4, "d = {}", r.dis_from_idx);
}

#[test]
fn insert_points_tags_an_existing_vertex() {
    let mut pl: Vec<PointWithFlag> = square()
        .into_iter()
        .map(|p| PointWithFlag { pos: p, pair_idx: -1, is_forward: false })
        .collect();
    let n = pl.len();
    // Landing exactly on points[1] tags it rather than splicing.
    insert_points(&mut pl, 1, Vec2f::new(10.0, 0.0), 3, true);
    assert_eq!(pl.len(), n, "no point should be inserted");
    assert_eq!(pl[1].pair_idx, 3);
    assert!(pl[1].is_forward);
}

#[test]
fn insert_points_tags_the_following_vertex() {
    let mut pl: Vec<PointWithFlag> = square()
        .into_iter()
        .map(|p| PointWithFlag { pos: p, pair_idx: -1, is_forward: false })
        .collect();
    let n = pl.len();
    // Landing on points[2] while idx = 1 tags the NEXT vertex.
    insert_points(&mut pl, 1, Vec2f::new(10.0, 10.0), 5, false);
    assert_eq!(pl.len(), n);
    assert_eq!(pl[2].pair_idx, 5);
}

#[test]
fn insert_points_splices_a_new_vertex() {
    let mut pl: Vec<PointWithFlag> = square()
        .into_iter()
        .map(|p| PointWithFlag { pos: p, pair_idx: -1, is_forward: false })
        .collect();
    let n = pl.len();
    // Mid-edge -> a new vertex goes in at idx + 1.
    insert_points(&mut pl, 0, Vec2f::new(5.0, 0.0), 9, true);
    assert_eq!(pl.len(), n + 1);
    assert_eq!(pl[1].pair_idx, 9);
    assert!((pl[1].pos.x - 5.0).abs() < 1e-4);
    // The original vertices keep their order around the splice.
    assert!((pl[2].pos.x - 10.0).abs() < 1e-4);
}
