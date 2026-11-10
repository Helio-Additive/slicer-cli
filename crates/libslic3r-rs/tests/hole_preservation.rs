//! R699 — does an ExPolygon HOLE survive our boolean primitives?
//!
//! R698 measured a hole deficit against C++ that is already 2.16x at bracket A
//! (identical surface counts, 45 holes vs 97) and 39.6x after mm-segmentation
//! (44 vs 1,740), with our area 0.14% LARGER — the signature of holes being
//! filled in rather than kept.
//!
//! Stage-boundary censuses cannot say which primitive loses them. This does:
//! one square with one square hole, pushed through each primitive on its own.

use slicer::clipper_utils::{difference, intersection, union_ex, union_polygons_ex};
use slicer::geometry::{ExPolygon, Point, Polygon};

/// Axis-aligned rectangle, CCW.
fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Polygon {
    Polygon::from_points(vec![
        Point::new(x0, y0),
        Point::new(x1, y0),
        Point::new(x1, y1),
        Point::new(x0, y1),
    ])
}

/// 10mm square (scaled 1e5) with a centred 2mm square hole.
fn square_with_hole() -> ExPolygon {
    let outer = rect(0, 0, 1_000_000, 1_000_000);
    // Hole wound CW (reverse of the contour) as an interior ring.
    let mut hole = rect(400_000, 400_000, 600_000, 600_000);
    hole.points.reverse();
    ExPolygon::with_holes(outer, vec![hole])
}

fn holes_of(v: &[ExPolygon]) -> usize {
    v.iter().map(|e| e.holes.len()).sum()
}

fn report(name: &str, out: &[ExPolygon]) -> usize {
    let h = holes_of(out);
    eprintln!(
        "[R699] {name:28} expolygons={} holes={} area_mm2={:.4}",
        out.len(),
        h,
        out.iter().map(|e| e.area()).sum::<f64>()
            / (slicer::SCALING_FACTOR * slicer::SCALING_FACTOR),
    );
    h
}

#[test]
fn hole_survives_union_polygons_ex() {
    // union_polygons_ex takes the flat rings (contour + holes) the way every
    // *_clib reconstruction feeds it, and must re-nest the interior ring.
    let ex = square_with_hole();
    let mut rings: Vec<Polygon> = vec![ex.contour.clone()];
    rings.extend(ex.holes.iter().cloned());
    let out = union_polygons_ex(&rings);
    assert_eq!(report("union_polygons_ex(rings)", &out), 1, "hole lost");
}

#[test]
fn hole_survives_union_ex() {
    let out = union_ex(&[square_with_hole()]);
    assert_eq!(report("union_ex([ex])", &out), 1, "hole lost");
}

#[test]
fn hole_survives_intersection_with_cover() {
    // Intersect with a square that covers everything: the result must be the
    // input, hole included.
    let cover = ExPolygon::new(rect(-100_000, -100_000, 1_100_000, 1_100_000));
    let out = intersection(&[square_with_hole()], &[cover]);
    assert_eq!(report("intersection(ex, cover)", &out), 1, "hole lost");
}

#[test]
fn hole_survives_difference_with_disjoint() {
    // Subtract a square that touches nothing: the result must be the input.
    let far = ExPolygon::new(rect(5_000_000, 5_000_000, 6_000_000, 6_000_000));
    let out = difference(&[square_with_hole()], &[far]);
    assert_eq!(report("difference(ex, disjoint)", &out), 1, "hole lost");
}

#[test]
fn difference_creates_a_hole() {
    // The mm-segmentation shape: subtract an interior island from a solid
    // square. C++ produces one ExPolygon with one hole; this is the operation
    // that should have created ~1,700 holes on Majora and created ~0.
    let solid = ExPolygon::new(rect(0, 0, 1_000_000, 1_000_000));
    let island = ExPolygon::new(rect(400_000, 400_000, 600_000, 600_000));
    let out = difference(&[solid], &[island]);
    assert_eq!(report("difference(solid, island)", &out), 1, "hole not created");
}
