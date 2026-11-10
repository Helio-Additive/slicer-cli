//! R633 — pin `intersection_pl`'s contract for `is_through_overhang`.
//!
//! `GCode.cpp:7019` decides whether a travel crosses an overhang with
//! `intersection_pl(travel, overhang).empty()`. That is the ONLY geometric test
//! in the predicate, so its edge cases are the predicate's edge cases. The one
//! that matters most is a travel lying ENTIRELY INSIDE the overhang: Clipper
//! clips open paths against a closed region, and a fully-contained path must
//! come back as itself, not as nothing. A travel that starts and ends inside an
//! overhang is the common case near a hull flare — if that returned empty, the
//! predicate would only fire on travels that happen to straddle the boundary.

use slicer::clipper_utils::intersection_pl;
use slicer::geometry::{ExPolygon, Point, Polygon, Polyline};

/// Axis-aligned square as an ExPolygon, corners in mm, CCW.
fn square(x0: f64, y0: f64, x1: f64, y1: f64) -> ExPolygon {
    let p = |x: f64, y: f64| Point::new(slicer::scale(x), slicer::scale(y));
    ExPolygon {
        contour: Polygon {
            points: vec![p(x0, y0), p(x1, y0), p(x1, y1), p(x0, y1)],
        },
        holes: vec![],
    }
}

fn seg(x0: f64, y0: f64, x1: f64, y1: f64) -> Polyline {
    let mut pl = Polyline::new();
    pl.points.push(Point::new(slicer::scale(x0), slicer::scale(y0)));
    pl.points.push(Point::new(slicer::scale(x1), slicer::scale(y1)));
    pl
}

#[test]
fn travel_fully_inside_overhang_intersects() {
    // The case the predicate depends on most: both endpoints inside.
    let oh = square(0.0, 0.0, 10.0, 10.0);
    let travel = seg(2.0, 2.0, 8.0, 8.0);
    let got = intersection_pl(&[travel], std::slice::from_ref(&oh));
    assert!(
        !got.is_empty(),
        "a travel entirely inside the overhang must intersect it — C++ \
         GCode.cpp:7019 promotes exactly this case to a spiral lift"
    );
}

#[test]
fn travel_crossing_the_boundary_intersects() {
    let oh = square(0.0, 0.0, 10.0, 10.0);
    let travel = seg(-5.0, 5.0, 5.0, 5.0);
    let got = intersection_pl(&[travel], std::slice::from_ref(&oh));
    assert!(!got.is_empty(), "a travel crossing into the overhang must intersect");
}

#[test]
fn travel_fully_outside_does_not_intersect() {
    let oh = square(0.0, 0.0, 10.0, 10.0);
    let travel = seg(20.0, 20.0, 30.0, 30.0);
    let got = intersection_pl(&[travel], std::slice::from_ref(&oh));
    assert!(got.is_empty(), "a travel clear of the overhang must not intersect");
}

#[test]
fn travel_inside_a_hole_does_not_intersect() {
    // Holes are why `loverhangs` is ExPolygons and not Polygons (R631).
    let mut oh = square(0.0, 0.0, 10.0, 10.0);
    let mut hole = square(3.0, 3.0, 7.0, 7.0).contour;
    hole.points.reverse(); // holes wind opposite the contour
    oh.holes.push(hole);
    let travel = seg(4.0, 4.0, 6.0, 6.0);
    let got = intersection_pl(&[travel], std::slice::from_ref(&oh));
    assert!(
        got.is_empty(),
        "a travel inside a hole is not over the overhang region"
    );
}

#[test]
fn short_travel_inside_overhang_intersects() {
    // Majora's clipped travels are often well under 1mm (measured R633: the
    // sampled clips ran 0.06-0.8mm). A sub-millimetre path must still clip.
    let oh = square(-10.0, -10.0, 10.0, 10.0);
    let travel = seg(-7.59, 2.30, -7.40, 3.04);
    let got = intersection_pl(&[travel], std::slice::from_ref(&oh));
    assert!(
        !got.is_empty(),
        "a sub-millimetre travel inside the overhang must still intersect"
    );
}
