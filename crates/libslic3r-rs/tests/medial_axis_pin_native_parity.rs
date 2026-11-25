//! The medial axis against the pin's own code (not the old FFI shim): `Geometry::MedialAxis`
//! (Geometry/MedialAxis.cpp:445-688; `build(ThickPolylines*)` 449-519, `build(Polylines*)`
//! 521-528, `process_edge_neighbors` 545-590, `validate_edge` 592-688) and the three
//! `ExPolygon::medial_axis` overloads (ExPolygon.cpp:263-371, 373-380, ExPolygon.hpp:75-76), as
//! libslic3r_core.a compiles them, on 1,196 ExPolygons from `tests/data/medial_axis_pin/gen.cpp`
//! (rotated strips, rings, narrow wedges, offset random walks, and the gap areas of random stars
//! cut as PerimeterGenerator.cpp:1346-1358 cuts them, with and without the douglas_peucker).
//! Points, the two widths per segment (bit for bit) and the endpoint flags must be equal.
//! The widths need MEDIAL_AXIS_EIGEN_PIN (on by default): with it at 0 the port spells the
//! Eigen vector steps as the old shim's fused scalars.

use slicer::geometry::{ExPolygon, MedialAxis, MedialAxisThickPolyline, Point, Polygon, Polyline};
use std::io::Read;

fn hexf(s: &str) -> f64 {
    let (neg, s) = match s.strip_prefix('-') {
        Some(r) => (true, r),
        None => (false, s),
    };
    let v = if s == "0x0p+0" {
        0.0
    } else {
        let s = s.strip_prefix("0x").expect("hex float");
        let (mant, exp) = s.split_once('p').expect("exponent");
        let exp: i32 = exp.parse().expect("exp");
        let (int_part, frac_part) = mant.split_once('.').unwrap_or((mant, ""));
        let digits = format!("{int_part}{frac_part}");
        let m = u64::from_str_radix(&digits, 16).expect("mantissa");
        (m as f64) * 2f64.powi(exp - 4 * frac_part.len() as i32)
    };
    if neg {
        -v
    } else {
        v
    }
}

#[derive(Debug, PartialEq)]
struct Thick {
    points: Vec<(i64, i64)>,
    width: Vec<u64>,
    endpoints: (bool, bool),
}

fn points_of(line: &str, tag: &str) -> Vec<(i64, i64)> {
    let mut it = line.split_whitespace();
    assert_eq!(it.next(), Some(tag), "{line}");
    let n: usize = it.next().unwrap().parse().unwrap();
    let v: Vec<i64> = it.map(|t| t.parse().unwrap()).collect();
    assert_eq!(v.len(), 2 * n);
    v.chunks(2).map(|c| (c[0], c[1])).collect()
}

fn read_thick<'a>(lines: &mut impl Iterator<Item = &'a str>, tag: &str) -> Vec<Thick> {
    let head = lines.next().unwrap();
    let mut it = head.split_whitespace();
    assert_eq!(it.next(), Some(tag), "{head}");
    let n: usize = it.next().unwrap().parse().unwrap();
    (0..n)
        .map(|_| {
            let points = points_of(lines.next().unwrap(), "P");
            let w = lines.next().unwrap();
            let mut wi = w.split_whitespace();
            assert_eq!(wi.next(), Some("W"));
            let m: usize = wi.next().unwrap().parse().unwrap();
            let width: Vec<u64> = wi.map(|t| hexf(t).to_bits()).collect();
            assert_eq!(width.len(), m);
            let e: Vec<&str> = lines.next().unwrap().split_whitespace().collect();
            assert_eq!(e[0], "E");
            Thick { points, width, endpoints: (e[1] == "1", e[2] == "1") }
        })
        .collect()
}

/// `SAME` (the points of the thick result) or the polylines' points.
fn read_plain<'a>(lines: &mut impl Iterator<Item = &'a str>, tag: &str, thick: &[Thick]) -> Vec<Vec<(i64, i64)>> {
    let head = lines.next().unwrap();
    let mut it = head.split_whitespace();
    assert_eq!(it.next(), Some(tag), "{head}");
    let n = it.next().unwrap();
    if n == "SAME" {
        return thick.iter().map(|t| t.points.clone()).collect();
    }
    let n: usize = n.parse().unwrap();
    (0..n).map(|_| points_of(lines.next().unwrap(), "Q")).collect()
}

fn thick_of(pp: &[MedialAxisThickPolyline]) -> Vec<Thick> {
    pp.iter()
        .map(|p| Thick {
            points: p.points.iter().map(|q| (q.x, q.y)).collect(),
            width: p.width.iter().map(|w| w.to_bits()).collect(),
            endpoints: p.endpoints,
        })
        .collect()
}

fn plain_of(pls: &[Polyline]) -> Vec<Vec<(i64, i64)>> {
    pls.iter().map(|p| p.points.iter().map(|q| (q.x, q.y)).collect()).collect()
}

fn polygon(pts: &[(i64, i64)]) -> Polygon {
    Polygon::from_points(pts.iter().map(|&(x, y)| Point::new(x, y)).collect())
}

#[test]
fn medial_axis_matches_the_pin() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/medial_axis_pin/vectors.txt.gz");
    let mut text = String::new();
    flate2::read::GzDecoder::new(std::fs::File::open(path).unwrap()).read_to_string(&mut text).unwrap();
    let mut lines = text.lines().peekable();
    // group -> (cases, mismatches)
    let mut tally = std::collections::BTreeMap::<&'static str, (usize, usize)>::new();
    let mut first_bad: Vec<String> = Vec::new();
    let mut cases = 0usize;
    while let Some(head) = lines.next() {
        let h: Vec<&str> = head.split_whitespace().collect();
        assert_eq!(h[0], "CASE", "{head}");
        let (min_w, max_w) = (hexf(h[3]), hexf(h[4]));
        let contour = polygon(&points_of(lines.next().unwrap(), "POLY"));
        let mut holes = Vec::new();
        while lines.peek().is_some_and(|l| l.starts_with("HOLE")) {
            holes.push(polygon(&points_of(lines.next().unwrap(), "HOLE")));
        }
        let ex = ExPolygon { contour, holes };
        let build = read_thick(&mut lines, "BUILD");
        let buildp = read_plain(&mut lines, "BUILDP", &build);
        let ma = read_thick(&mut lines, "MA");
        let map = read_plain(&mut lines, "MAP", &ma);
        let mar = read_plain(&mut lines, "MAR", &ma);
        assert_eq!(lines.next(), Some("END"));
        cases += 1;

        let mut check = |group: &'static str, ok: bool| {
            let e = tally.entry(group).or_insert((0, 0));
            e.0 += 1;
            if !ok {
                e.1 += 1;
                if first_bad.len() < 8 {
                    first_bad.push(format!("{group} case {}", h[1]));
                }
            }
        };

        // MedialAxis::build(ThickPolylines*), MedialAxis.cpp:449-519.
        let mut pp = Vec::new();
        MedialAxis::new(min_w, max_w, &ex).build(&mut pp);
        let got = thick_of(&pp);
        check("MedialAxis::build points", got.len() == build.len() && got.iter().zip(&build).all(|(a, b)| a.points == b.points));
        check("MedialAxis::build widths+endpoints", got == build);
        // MedialAxis::build(Polylines*), MedialAxis.cpp:521-528.
        let mut pls = Vec::new();
        MedialAxis::new(min_w, max_w, &ex).build_polylines(&mut pls);
        check("MedialAxis::build(Polylines)", plain_of(&pls) == buildp);

        // ExPolygon::medial_axis(min, max, ThickPolylines*), ExPolygon.cpp:263-371.
        let mut pp = Vec::new();
        ex.medial_axis_thick_polylines(min_w, max_w, &mut pp);
        let got = thick_of(&pp);
        check("ExPolygon::medial_axis points", got.len() == ma.len() && got.iter().zip(&ma).all(|(a, b)| a.points == b.points));
        check("ExPolygon::medial_axis widths+endpoints", got == ma);
        // ExPolygon::medial_axis(min, max, Polylines*), ExPolygon.cpp:373-380.
        let mut pls = Vec::new();
        ex.medial_axis_into_polylines(min_w, max_w, &mut pls);
        check("ExPolygon::medial_axis(Polylines*)", plain_of(&pls) == map);
        // Polylines ExPolygon::medial_axis(min, max), ExPolygon.hpp:75-76.
        check("ExPolygon::medial_axis() -> Polylines", plain_of(&ex.medial_axis_returning_polylines(min_w, max_w)) == mar);
    }
    for (group, (n, bad)) in &tally {
        eprintln!("{group}: {n} cases, {bad} mismatches");
    }
    eprintln!("first mismatches: {first_bad:?}");
    assert_eq!(cases, 1196);
    assert!(tally.values().all(|&(_, bad)| bad == 0), "{tally:?}");
}
