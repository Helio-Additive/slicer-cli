//! `slicer::geometry::Polygon` (the port of libslic3r/Polygon.hpp and Polygon.cpp) against the C++ at the pin.
//!
//! `tests/data/polygon/vectors.txt.gz` is the output of `gen.cpp` beside it; the header of gen.cpp describes
//! the line. Every case carries its inputs and the pin's results; the same calls run here and every result
//! has to be the pin's (integers equal, doubles bit-equal). `centroid` is compared on the default arm of its
//! gate `POLYGON_CENTROID_PIN`. The families named in `REPORTED` are counted and printed, not asserted: they
//! are not equal to the pin yet.

use slicer::geometry::{Line, Point, Polygon, Polygons};
use std::collections::BTreeMap;
use std::io::Read;

/// Families that are not equal to the pin yet (counted and printed, not asserted).
const REPORTED: &[&str] = &[];

fn read_vectors() -> String {
    let path = std::env::var("POLYGON_VECTORS")
        .unwrap_or_else(|_| format!("{}/tests/data/polygon/vectors.txt.gz", env!("CARGO_MANIFEST_DIR")));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    if path.ends_with(".gz") {
        flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    } else {
        std::io::BufReader::new(file).read_to_string(&mut text).expect("read");
    }
    text
}

struct Tok<'a> {
    t: Vec<&'a str>,
    k: usize,
}
impl<'a> Tok<'a> {
    fn s(&mut self) -> &'a str {
        self.k += 1;
        self.t[self.k - 1]
    }
    fn i(&mut self) -> i64 {
        self.s().parse().expect("integer")
    }
    fn d(&mut self) -> f64 {
        f64::from_bits(u64::from_str_radix(self.s(), 16).expect("double"))
    }
    fn pt(&mut self) -> Point {
        let x = self.i();
        let y = self.i();
        Point::new(x, y)
    }
    fn pts(&mut self) -> Vec<Point> {
        let n = self.i() as usize;
        (0..n).map(|_| self.pt()).collect()
    }
}

#[test]
fn polygon_matches_the_pin() {
    let text = read_vectors();
    let mut calls: BTreeMap<&'static str, (usize, usize)> = BTreeMap::new();
    let mut first: Vec<String> = Vec::new();
    let mut cases = 0usize;
    for (ln, line) in text.lines().enumerate() {
        let mut t = Tok { t: line.split(' ').filter(|x| !x.is_empty()).collect(), k: 0 };
        assert_eq!(t.s(), "c");
        cases += 1;
        let pts = t.pts();
        let cut = Line::new(t.pt(), t.pt());
        let p = t.pt();
        let threshold = t.d();
        let tolerance = t.d();
        let spacing = t.d();
        assert_eq!(t.s(), "->");
        let poly = Polygon::from_points(pts.clone());

        let mut check = |name: &'static str, ok: bool, what: String| {
            let e = calls.entry(name).or_insert((0, 0));
            e.0 += 1;
            if !ok {
                e.1 += 1;
                if first.len() < 24 {
                    first.push(format!("line {}: {name}: {what}", ln + 1));
                }
            }
        };
        macro_rules! pt_eq {
            ($name:expr, $got:expr) => {{
                let want = t.pt();
                let got: Point = $got;
                check($name, got == want, format!("got {got:?} want {want:?}"));
            }};
        }
        macro_rules! pts_eq {
            ($name:expr, $got:expr) => {{
                let want = t.pts();
                let got: Vec<Point> = $got;
                check($name, got == want, format!("got {got:?} want {want:?}"));
            }};
        }
        macro_rules! int_eq {
            ($name:expr, $got:expr) => {{
                let want = t.i();
                let got: i64 = $got;
                check($name, got == want, format!("got {got} want {want}"));
            }};
        }
        macro_rules! dbl_eq {
            ($name:expr, $got:expr) => {{
                let want = t.d();
                let got: f64 = $got;
                check($name, got.to_bits() == want.to_bits(), format!("got {got:e} want {want:e}"));
            }};
        }

        dbl_eq!("length", poly.length());
        dbl_eq!("area", poly.area());
        int_eq!("is_counter_clockwise", poly.is_counter_clockwise() as i64);
        int_eq!("is_clockwise", poly.is_clockwise() as i64);
        pt_eq!("centroid", poly.centroid());
        {
            let n = t.i() as usize;
            let want: Vec<Line> = (0..n).map(|_| Line::new(t.pt(), t.pt())).collect();
            let got = poly.lines();
            check("lines", got == want, format!("got {} lines want {}", got.len(), want.len()));
        }
        {
            let mut x = Point::new(0, 0);
            let ok = poly.first_intersection(&cut, &mut x);
            int_eq!("first_intersection", ok as i64);
            pt_eq!("first_intersection", x);
        }
        {
            let mut xs: Vec<Point> = Vec::new();
            let ok = poly.intersections(&cut, &mut xs);
            int_eq!("intersections", ok as i64);
            pts_eq!("intersections", xs);
        }
        pts_eq!("convex_points", poly.convex_points(threshold));
        pts_eq!("concave_points", poly.concave_points(threshold));
        pt_eq!("point_projection", poly.point_projection(&p));
        {
            let mut tris: Polygons = Vec::new();
            poly.triangulate_convex(&mut tris);
            let n = t.i() as usize;
            let want: Vec<Vec<Point>> = (0..n).map(|_| t.pts()).collect();
            let got: Vec<Vec<Point>> = tris.iter().map(|q| q.points().to_vec()).collect();
            check("triangulate_convex", got == want, format!("got {} polygons want {}", got.len(), want.len()));
        }
        {
            let mut dp = poly.clone();
            dp.douglas_peucker(tolerance);
            pts_eq!("douglas_peucker", dp.points().to_vec());
        }
        {
            let mut ccw = poly.clone();
            int_eq!("make_counter_clockwise", ccw.make_counter_clockwise() as i64);
            pts_eq!("make_counter_clockwise", ccw.points().to_vec());
            let mut cw = poly.clone();
            int_eq!("make_clockwise", cw.make_clockwise() as i64);
            pts_eq!("make_clockwise", cw.points().to_vec());
        }
        pts_eq!("equally_spaced_points", poly.equally_spaced_points(spacing));
        assert_eq!(t.k, t.t.len(), "line {}: tokens left", ln + 1);
    }
    let mut total = 0usize;
    let mut bad = 0usize;
    for (name, (n, m)) in &calls {
        let reported = REPORTED.contains(name);
        println!("{name}: {n} calls, {m} mismatches{}", if reported { " (reported, not asserted)" } else { "" });
        if !reported {
            total += n;
            bad += m;
        }
    }
    println!("polygon: {cases} cases, {total} asserted calls, {bad} mismatches");
    for f in &first {
        println!("{f}");
    }
    assert_eq!(cases, 3000);
    assert_eq!(bad, 0, "{bad} of {total} results differ from the pin");
}
