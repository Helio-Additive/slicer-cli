//! `ExPolygon` (the port of libslic3r/ExPolygon.hpp and ExPolygon.cpp: `slicer::ex_polygon` and
//! `slicer::geometry`) against the C++ at the pin.
//!
//! `tests/data/ex_polygon/vectors.txt.gz` is the output of `gen.cpp` beside it; the header of gen.cpp describes
//! the line. Every case carries its inputs and the pin's results; the same calls run here and every result
//! has to be the pin's (integers equal, doubles bit-equal). `simplify_p`, `simplify` and
//! `expolygons_simplify` are compared through the functions that take the tolerance in the pin's unit
//! (`simplify_p_scaled_tolerance`, `simplify_scaled_tolerance`, `expolygons_simplify_scaled_tolerance`).
//! The families named in `REPORTED` are counted and printed, not asserted: they are not equal to the pin yet.

use slicer::geometry::{BoundingBox, ExPolygon, Line, Point, Polygon, Polyline};
use std::collections::BTreeMap;
use std::io::Read;

/// Families that are not equal to the pin yet (counted and printed, not asserted).
/// R931: empty. `scale(x, y)` truncates as MultiPoint.cpp:12-19, `projection_onto(expolygons)` takes the
/// closing edge of each ring (Polygon::lines, ExPolygon.cpp:206-223), and `geometry::ExPolygon::scale`,
/// `Polygon::scale`, `Polyline::scale` truncate under the gate `EXPOLYGON_SCALE_PIN` (on by default; the
/// test asserts that arm, MultiPoint.cpp:6-10 with Point.hpp:199).
const REPORTED: &[&str] = &[];

fn read_vectors() -> String {
    let path = std::env::var("EX_POLYGON_VECTORS")
        .unwrap_or_else(|_| format!("{}/tests/data/ex_polygon/vectors.txt.gz", env!("CARGO_MANIFEST_DIR")));
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
    fn polys(&mut self) -> Vec<Vec<Point>> {
        let n = self.i() as usize;
        (0..n).map(|_| self.pts()).collect()
    }
    /// An expolygon as its rings: the contour, then the holes.
    fn ex(&mut self) -> Vec<Vec<Point>> {
        let contour = self.pts();
        let mut rings = vec![contour];
        rings.extend(self.polys());
        rings
    }
    fn exs(&mut self) -> Vec<Vec<Vec<Point>>> {
        let n = self.i() as usize;
        (0..n).map(|_| self.ex()).collect()
    }
    fn lines(&mut self) -> Vec<Line> {
        let n = self.i() as usize;
        (0..n).map(|_| Line::new(self.pt(), self.pt())).collect()
    }
    fn bb(&mut self) -> (Point, Point, bool) {
        (self.pt(), self.pt(), self.i() != 0)
    }
}

fn make(rings: &[Vec<Point>]) -> ExPolygon {
    ExPolygon::with_holes(
        Polygon::from_points(rings[0].clone()),
        rings[1..].iter().map(|r| Polygon::from_points(r.clone())).collect(),
    )
}
fn rings(ex: &ExPolygon) -> Vec<Vec<Point>> {
    let mut out = vec![ex.contour.points().to_vec()];
    out.extend(ex.holes.iter().map(|h| h.points().to_vec()));
    out
}
fn rings_of(exs: &[ExPolygon]) -> Vec<Vec<Vec<Point>>> {
    exs.iter().map(rings).collect()
}
fn bb_of(bb: &BoundingBox) -> (Point, Point, bool) {
    (bb.min, bb.max, bb.is_defined())
}

#[test]
fn ex_polygon_matches_the_pin() {
    let text = read_vectors();
    let mut calls: BTreeMap<&'static str, (usize, usize)> = BTreeMap::new();
    let mut first: Vec<String> = Vec::new();
    let mut cases = 0usize;
    for (ln, line) in text.lines().enumerate() {
        let mut t = Tok { t: line.split(' ').filter(|x| !x.is_empty()).collect(), k: 0 };
        assert_eq!(t.s(), "c");
        cases += 1;
        let ex = make(&t.ex());
        let other = make(&t.ex());
        let p = t.pt();
        let cut = Line::new(t.pt(), t.pt());
        let polyline = Polyline::from_points(t.pts());
        let tolerance = t.d();
        let eps = t.d();
        let factor = t.d();
        let v = t.pt();
        assert_eq!(t.s(), "->");

        let mut check = |name: &'static str, ok: bool, what: String| {
            let e = calls.entry(name).or_insert((0, 0));
            e.0 += 1;
            if !ok {
                e.1 += 1;
                if first.len() < 24 && !REPORTED.contains(&name) {
                    first.push(format!("line {}: {name}: {what}", ln + 1));
                }
            }
        };
        macro_rules! eq {
            ($name:expr, $got:expr, $want:expr) => {{
                let want = $want;
                let got = $got;
                let what = if got == want { String::new() } else { format!("got {got:?} want {want:?}") };
                check($name, got == want, what.chars().take(400).collect());
            }};
        }
        macro_rules! dbl_eq {
            ($name:expr, $got:expr) => {{
                let want = t.d();
                let got: f64 = $got;
                check($name, got.to_bits() == want.to_bits(), format!("got {got:e} want {want:e}"));
            }};
        }

        dbl_eq!("area", ex.area());
        eq!("is_valid", ex.is_valid() as i64, t.i());
        eq!("contains(point, true)", ex.contains(&p, true) as i64, t.i());
        eq!("contains(point, false)", ex.contains(&p, false) as i64, t.i());
        eq!("on_boundary", ex.on_boundary(&p, eps) as i64, t.i());
        eq!("point_projection", ex.point_projection(&p), t.pt());
        eq!("contains(line)", ex.contains_line(&cut) as i64, t.i());
        eq!("contains(polyline)", ex.contains_polyline(&polyline) as i64, t.i());
        {
            let pls = vec![polyline.clone(), Polyline::from_points(vec![cut.a, cut.b])];
            eq!("contains(polylines)", ex.contains_polylines(&pls) as i64, t.i());
        }
        eq!("overlaps", ex.overlaps(&other) as i64, t.i());
        {
            let got: Vec<Vec<Point>> = ex.simplify_p_scaled_tolerance(tolerance).iter().map(|q| q.points().to_vec()).collect();
            eq!("simplify_p", got, t.polys());
        }
        eq!("simplify", rings_of(&ex.simplify_scaled_tolerance(tolerance)), t.exs());
        {
            let mut dp = ex.clone();
            dp.douglas_peucker(tolerance);
            eq!("douglas_peucker", rings(&dp), t.ex());
        }
        eq!("lines", ex.lines(), t.lines());
        {
            let mut tr = ex.clone();
            tr.translate(v);
            eq!("translate", rings(&tr), t.ex());
        }
        {
            let mut sc = ex.clone();
            sc.scale_factor(factor);
            eq!("scale", rings(&sc), t.ex());
            // `ExPolygon::scale` of slicer::geometry, gate `EXPOLYGON_SCALE_PIN` on (ExPolygon.cpp:17-22).
            let mut sr = ex.clone();
            sr.scale(factor);
            check("scale (geometry::ExPolygon::scale)", rings(&sr) == rings(&sc), String::new());
            // `MultiPoint::scale(double)` through Polygon and Polyline (MultiPoint.cpp:6-10).
            let mut pc = ex.contour.clone();
            pc.scale(factor);
            check("scale (geometry::Polygon::scale)", pc.points().to_vec() == rings(&sc)[0], String::new());
            let mut pl = slicer::geometry::Polyline::from_points(ex.contour.points().to_vec());
            pl.scale(factor);
            check("scale (geometry::Polyline::scale)", pl.points().to_vec() == rings(&sc)[0], String::new());
        }
        {
            let mut rc = ex.clone();
            let removed = rc.remove_colinear_points();
            eq!("remove_colinear_points", removed as i64, t.i());
            eq!("remove_colinear_points", rings(&rc), t.ex());
        }
        let exs = vec![ex.clone(), other.clone()];
        let sxe = vec![other.clone(), ex.clone()];
        eq!("count_points(expolygon)", slicer::geometry::count_points_expoly(&ex) as i64, t.i());
        eq!("count_points(expolygons)", slicer::geometry::count_points(&exs) as i64, t.i());
        eq!("number_polygons", slicer::geometry::number_polygons(&exs) as i64, t.i());
        eq!("to_lines(expolygons)", slicer::geometry::to_lines(&exs), t.lines());
        eq!("to_points(expolygons)", slicer::geometry::to_points(&exs), t.pts());
        {
            let got: Vec<Vec<Point>> = slicer::geometry::to_polygons(&exs).iter().map(|q| q.points().to_vec()).collect();
            eq!("to_polygons(expolygons)", got, t.polys());
        }
        eq!("get_extents(expolygon)", bb_of(&slicer::ex_polygon::get_extents_expoly(&ex)), t.bb());
        eq!("get_extents(expolygons)", bb_of(&slicer::ex_polygon::get_extents(&exs)), t.bb());
        dbl_eq!("area(expolygons)", slicer::geometry::area_expolygons(&exs));
        eq!("expolygons_simplify", rings_of(&slicer::geometry::expolygons_simplify_scaled_tolerance(&exs, tolerance)), t.exs());
        eq!("expolygons_contain(true)", slicer::geometry::expolygons_contain_border_result(&sxe, &p, true) as i64, t.i());
        eq!("expolygons_contain(false)", slicer::geometry::expolygons_contain_border_result(&sxe, &p, false) as i64, t.i());
        eq!("overlaps(expolygons, expolygon)", slicer::ex_polygon::overlaps_expolys_expoly(&sxe, &ex) as i64, t.i());
        eq!("has_duplicate_points(expolygon)", slicer::ex_polygon::has_duplicate_points_expoly(&ex) as i64, t.i());
        dbl_eq!("get_expolygons_area", slicer::ex_polygon::get_expolygons_area(&exs));
        let range: i64 = [60, 3000, 400000, 2000000][ln % 4];
        {
            let mut sy = ex.clone();
            sy.symmetric_y(v.y);
            eq!("symmetric_y", rings(&sy), t.ex());
        }
        eq!("projection_onto(expolygons)", slicer::ex_polygon::projection_onto(&exs, &p), t.pt());
        eq!("has_duplicate_points(expolygons)", slicer::ex_polygon::has_duplicate_points(&exs) as i64, t.i());
        {
            let mut sc = ex.clone();
            sc.scale_xy(factor, 2.25 - factor);
            eq!("scale(x, y)", rings(&sc), t.ex());
        }
        {
            let collision = vec![other.clone()];
            eq!("split_expoly_with_holes", rings_of(&ex.split_expoly_with_holes(range / 20, &collision)), t.exs());
        }
        {
            let n = t.i() as usize;
            let want: Vec<(Point, Point, bool)> = (0..n).map(|_| t.bb()).collect();
            let got: Vec<(Point, Point, bool)> = slicer::ex_polygon::get_extents_vector(&exs).iter().map(bb_of).collect();
            eq!("get_extents_vector", got, want);
        }
        {
            let mut r = exs.clone();
            let modified = slicer::ex_polygon::remove_small_and_small_holes(&mut r, 0.02 * range as f64 * range as f64);
            eq!("remove_small_and_small_holes", modified as i64, t.i());
            eq!("remove_small_and_small_holes", rings_of(&r), t.exs());
        }
        {
            let mut k = sxe.clone();
            slicer::geometry::keep_largest_contour_only(&mut k);
            eq!("keep_largest_contour_only", rings_of(&k), t.exs());
        }
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
    println!("ex_polygon: {cases} cases, {total} asserted calls, {bad} mismatches");
    for f in &first {
        println!("{f}");
    }
    assert_eq!(cases, 2000);
    assert_eq!(bad, 0, "{bad} of {total} results differ from the pin");
}
