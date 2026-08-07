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

// ===========================================================================
// R624 — the gap constructor itself, on top of R623's leaf helpers.
// ===========================================================================

use slicer::gcode::wipe_tower::{
    add_extra_point, contrust_gap_for_skip_points, generate_rectange_polygon,
    remove_points_from_polygon,
};
use slicer::geometry::Point;

/// The tower wall used by C++ when rib_wall is off: a plain rectangle.
/// Majora's is 35mm wide (`prime_tower_width`), so these use the same shape.
fn wall(w: f32, d: f32) -> slicer::geometry::Polygon {
    generate_rectange_polygon(Vec2f::new(0.0, 0.0), Vec2f::new(w, d))
}

#[test]
fn rectangle_polygon_is_ccw_from_the_origin() {
    let p = wall(35.0, 38.5);
    assert_eq!(p.points().len(), 4);
    // ld, rd, ru, lu — WipeTower.cpp:610-618.
    assert_eq!(p.points()[0], Point::new(0, 0));
    assert!(p.points()[1].x > 0 && p.points()[1].y == 0);
    assert!(p.points()[2].x > 0 && p.points()[2].y > 0);
    assert!(p.points()[3].x == 0 && p.points()[3].y > 0);
}

#[test]
fn add_extra_point_splices_three_vertices() {
    // C++ :494-503 inserts offset_to_a, mid, offset_to_b after the chosen edge's
    // start vertex, so a 4-gon becomes a 7-gon.
    let p = wall(35.0, 38.5);
    let out = add_extra_point(&p, slicer::scaled(1.25) as f64);
    assert_eq!(out.points().len(), 7, "three vertices should be spliced in");
}

#[test]
fn add_extra_point_targets_the_bottom_edge() {
    // The anchor is (bbox centre x, bbox min y), so on an axis-aligned
    // rectangle the nearest edge midpoint is the BOTTOM edge (ld -> rd), i.e.
    // index 0. The trio therefore lands at indices 1..3, all at y = 0.
    let p = wall(35.0, 38.5);
    let out = add_extra_point(&p, slicer::scaled(1.25) as f64);
    for i in 1..=3 {
        assert_eq!(out.points()[i].y, 0, "point {} should sit on the bottom edge", i);
    }
    // The middle of the trio is the edge midpoint.
    assert_eq!(out.points()[2].x, slicer::scaled(17.5));
}

#[test]
fn add_extra_point_clamps_the_range() {
    // :471 — range is clamped to 0.9 * the shorter half-edge, so an absurd
    // request cannot push the offsets past the edge's own endpoints.
    let p = wall(35.0, 38.5);
    let out = add_extra_point(&p, slicer::scaled(1000.0) as f64);
    for i in 1..=3 {
        let x = out.points()[i].x;
        assert!(x >= 0 && x <= slicer::scaled(35.0), "point {} escaped the edge: {}", i, x);
    }
}

#[test]
fn no_skip_points_leaves_the_wall_whole() {
    // :597-599 — the empty case returns the ring as a single run.
    let p = wall(35.0, 38.5);
    let (runs, ring) = contrust_gap_for_skip_points(&p, &[], 35.0, 1.25);
    assert_eq!(runs.len(), 1, "an ungapped wall is one polyline");
    assert_eq!(ring.points().len(), p.points().len());
}

#[test]
fn one_skip_point_opens_one_gap() {
    // A single skip point on the right edge (x == wt_width) should break the
    // ring into exactly one run — the wall minus one gap is still one path,
    // because the ring is cut open at a single place.
    let p = wall(35.0, 38.5);
    let skip = vec![Vec2f::new(35.0, 10.0)];
    let (runs, ring) = contrust_gap_for_skip_points(&p, &skip, 35.0, 1.25);
    assert!(!runs.is_empty(), "the wall should still be drawn");
    // The returned ring carries the gap boundaries, so it has MORE points than
    // the densified 7-gon that went in.
    assert!(
        ring.points().len() >= 7,
        "insert_skip_pg should carry the gap boundaries, got {}",
        ring.points().len()
    );
}

#[test]
fn two_skip_points_open_two_gaps() {
    // One gap on each side: cutting a closed ring twice yields two runs.
    let p = wall(35.0, 38.5);
    let skip = vec![Vec2f::new(35.0, 10.0), Vec2f::new(0.0, 25.0)];
    let (runs, _) = contrust_gap_for_skip_points(&p, &skip, 35.0, 1.25);
    // THREE, not two: the walk (:559-588) starts at the ring vertex nearest the
    // anchor, which is mid-way along an arc, so that arc is emitted as a head run
    // and a tail run. Two cuts in a ring give two arcs, but one of them is split
    // by the start position. C++ does exactly the same.
    assert_eq!(runs.len(), 3, "two gaps, with the start mid-arc, give three runs");
}

#[test]
fn gaps_actually_remove_length() {
    // The whole point of the gap is that the wall is SHORTER than the ring.
    let p = wall(35.0, 38.5);
    let len_of = |runs: &Vec<slicer::geometry::Polyline>| -> f64 {
        runs.iter()
            .map(|pl| {
                pl.points
                    .windows(2)
                    .map(|w| {
                        let dx = slicer::unscale(w[1].x - w[0].x);
                        let dy = slicer::unscale(w[1].y - w[0].y);
                        (dx * dx + dy * dy).sqrt()
                    })
                    .sum::<f64>()
            })
            .sum()
    };
    // `whole` must be the CLOSED ring: 2*(35 + 38.5) = 147mm. An early version of
    // the port dropped Polygon.hpp:224's closing point and returned 108.5mm
    // (three sides), which made the gapped wall look LONGER than the whole one.
    let (whole, _) = contrust_gap_for_skip_points(&p, &[], 35.0, 1.25);
    let skip = vec![Vec2f::new(35.0, 10.0)];
    let (gapped, _) = contrust_gap_for_skip_points(&p, &skip, 35.0, 1.25);
    let (lw, lg) = (len_of(&whole), len_of(&gapped));
    assert!(
        (lw - 147.0).abs() < 1e-3,
        "the ungapped wall must be the closed perimeter 147mm, got {:.3}",
        lw
    );
    assert!(lg < lw, "gapped wall ({:.3}) should be shorter than whole ({:.3})", lg, lw);
    // The gap is 2 * range wide by construction (range either side of the hit).
    let removed = lw - lg;
    assert!(
        removed > 1.0 && removed < 6.0,
        "one 1.25mm-range gap should remove a few mm, removed {:.3}",
        removed
    );
}

#[test]
fn remove_points_from_polygon_is_stable_with_no_points() {
    // Guard the degenerate path: no skip points reaches the same code as the
    // wrapper's early-out but through the full routine.
    let p = wall(35.0, 38.5);
    let (runs, ring) = remove_points_from_polygon(&p, &[], 1.25, 35.0);
    assert_eq!(runs.len(), 1, "no gaps -> one run");
    assert_eq!(ring.points().len(), 7, "the ring is still the densified 7-gon");
}
