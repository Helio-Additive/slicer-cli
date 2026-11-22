//! Native parity of the BridgeDetector.cpp / BridgeDetector.hpp port (`slicer::bridge_detector`) against
//! vectors made by the C++ at the pin (tests/data/bridge_detector/gen.cpp).
//!
//! Groups: "len" (Line::length and Line::direction as the detector takes them), "bd" (a BridgeDetector over
//! bridges and lower slices: the constructor, bridge_direction_candidates, detect_angle with and without an
//! override, coverage plain and precise, unsupported_edges, and get_trapezoids2 / get_trapezoids3_half on the
//! first bridge), "dbd" (detect_bridging_direction of floating edges) and "dbda" (detect_bridging_direction
//! of an area to cover and its anchors). The lines of the vectors that start with "in" are the inputs,
//! which this test reads back; every other line it has to reproduce bit for bit.

mod support_shared;

use slicer::bridge_detector::{
    detect_bridging_direction, detect_bridging_direction_areas, get_trapezoids2, get_trapezoids2_angle, get_trapezoids3_half, BridgeDetector,
};
use slicer::geometry::{ExPolygons, Line, Lines, Point, Polygon, Polygons, Polyline};
use support_shared::*;

/// gen.cpp `PLS`
fn pls_s(ps: &[Polyline]) -> String {
    let mut o = format!("{}", ps.len());
    for p in ps {
        o += &format!(" {}", p.points.len());
        for q in &p.points {
            o += &format!(" {}", pt(q));
        }
    }
    o
}

/// gen.cpp `run_len`: the two functions of Line the detector's results depend on, through a detector-free
/// path: a line is its own floating edge, and its direction is a bridge direction candidate.
fn run_len(r: &mut Run) {
    while let Some(line) = r.input("in len") {
        let mut t = Tok::new(&line);
        let a = t.pt();
        let b = t.pt();
        let l = Line::new(a, b);
        r.out(format!("len {} {}", hd(slicer::bridge_detector::line_length(&l)), hd(slicer::bridge_detector::line_direction(&l))));
    }
}

/// gen.cpp `run_bd`
fn run_bd(r: &mut Run) -> usize {
    let mut k = 0;
    while let Some(line) = r.input(&format!("in bd {k}")) {
        let s = format!("bd {k}");
        k += 1;
        let mut t = Tok::new(&line);
        let spacing = t.i();
        let single = t.b();
        let a = t.d();
        let ov = t.d();
        let bridge: ExPolygons = t.expolys();
        let lower: ExPolygons = t.expolys();

        let mut bd = if single { BridgeDetector::new(bridge[0].clone(), &lower, spacing) } else { BridgeDetector::new_multi(&bridge, &lower, spacing) };
        r.out(format!("{s} init {} {} {} {}", hd(bd.resolution), hd(bd.angle), pls_s(bd.edges()), expolys_s(bd.anchor_regions())));
        {
            let cand = bd.bridge_direction_candidates();
            let mut o = format!("{s} cand {}", cand.len());
            for c in cand {
                o += &format!(" {}", hd(c));
            }
            r.out(o);
        }
        r.out(format!("{s} none {} {}", polys_s(&bd.coverage(-1., false)), pls_s(&bd.unsupported_edges(-1.))));
        let found = bd.detect_angle(0.);
        r.out(format!("{s} detect {} {}", hb(found), hd(bd.angle)));
        r.out(format!("{s} cov0 {}", polys_s(&bd.coverage(-1., false))));
        r.out(format!("{s} cov1 {}", polys_s(&bd.coverage(-1., true))));
        r.out(format!("{s} covA {}", polys_s(&bd.coverage(a, false))));
        r.out(format!("{s} covB {}", polys_s(&bd.coverage(a, true))));
        r.out(format!("{s} un {}", pls_s(&bd.unsupported_edges(-1.))));
        {
            let mut un = vec![Polyline::from_points(vec![Point::new(1, 2), Point::new(3, 4)])];
            bd.unsupported_edges_into(a, &mut un);
            r.out(format!("{s} unA {}", pls_s(&un)));
        }
        {
            let mut bd2 = BridgeDetector::new_multi(&bridge, &lower, spacing);
            let found2 = bd2.detect_angle(ov);
            r.out(format!("{s} ov {} {} {}", hb(found2), hd(bd2.angle), polys_s(&bd2.coverage(-1., false))));
        }
        {
            let (mut t2, mut t3): (Polygons, Polygons) = (Vec::new(), Vec::new());
            let mut t2a: Polygons = vec![Polygon::from_points(vec![Point::new(0, 0), Point::new(100000, 0), Point::new(100000, 50000), Point::new(0, 50000)])];
            get_trapezoids2(&bridge[0], &mut t2);
            get_trapezoids2_angle(&bridge[0], &mut t2a, a);
            get_trapezoids3_half(&bridge[0], &mut t3, spacing as f32);
            r.out(format!("{s} tz2 {}", polys_s(&t2)));
            r.out(format!("{s} tz2a {}", polys_s(&t2a)));
            r.out(format!("{s} tz3 {}", polys_s(&t3)));
        }
    }
    k
}

/// gen.cpp `run_dbd`
fn run_dbd(r: &mut Run) {
    while let Some(line) = r.input("in dbd") {
        let mut t = Tok::new(&line);
        let n = t.i();
        let edges: Lines = (0..n)
            .map(|_| {
                let a = t.pt();
                let b = t.pt();
                Line::new(a, b)
            })
            .collect();
        let area: Polygons = t.polys();
        let (dir, cost) = detect_bridging_direction(&edges, &area);
        r.out(format!("dbd {} {} {}", hd(dir.x), hd(dir.y), hd(cost)));
    }
}

/// gen.cpp `run_dbda`
fn run_dbda(r: &mut Run) {
    while let Some(line) = r.input("in dbda") {
        let mut t = Tok::new(&line);
        let to_cover: Polygons = t.polys();
        let anchors: Polygons = t.polys();
        let (dir, cost) = detect_bridging_direction_areas(&to_cover, &anchors);
        r.out(format!("dbda {} {} {}", hd(dir.x), hd(dir.y), hd(cost)));
    }
}

#[test]
fn bridge_detector_native_parity() {
    let mut r = Run::load("bridge_detector");
    run_len(&mut r);
    let scenarios = run_bd(&mut r);
    run_dbd(&mut r);
    run_dbda(&mut r);
    eprintln!("bridge_detector: {scenarios} detector scenarios");
    let n = r.expected.iter().filter(|l| !l.starts_with("in ")).count();
    for tag in ["len ", "dbd ", "dbda "] {
        let (total, ok) = r.tally(tag);
        eprintln!("bridge_detector: {tag}: {ok} of {total}");
    }
    for stage in ["init", "cand", "none", "detect", "cov0", "cov1", "covA", "covB", "un", "unA", "ov", "tz2", "tz2a", "tz3"] {
        let (mut total, mut ok) = (0, 0);
        for k in 0..scenarios {
            let (t, o) = r.tally(&format!("bd {k} {stage} "));
            total += t;
            ok += o;
        }
        eprintln!("bridge_detector: stage {stage}: {ok} of {total}");
    }
    let bad = r.finish();
    eprintln!("bridge_detector: {} result lines, {} differ", n, bad);
    assert_eq!(bad, 0);
}
