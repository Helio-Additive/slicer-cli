//! The inline free functions of ExPolygon.hpp:93-449 (`slicer::ex_polygon`: `count_points`, `number_polygons`,
//! `to_lines`, `to_points`, `translate`, `polygons_append`, `expolygons_append`, `expolygons_rotate`,
//! `expolygons_contain`) against the C++ at the pin.
//!
//! `tests/data/ex_polygon_rest/vectors.txt.gz` is the output of `gen.cpp` beside it; the header of gen.cpp
//! describes the lines. Integers have to be equal.

use slicer::ex_polygon::{
    count_points, expolygons_append, expolygons_simplify, expolygons_contain, expolygons_rotate, number_polygons, polygons_append,
    to_expolygons, to_lines, to_points, to_polygon_ptrs, to_polygons, to_polylines, translate,
};
use slicer::geometry::{ExPolygon, Line, Point, Polygon};
use std::collections::BTreeMap;
use std::io::Read;

fn read_vectors() -> String {
    let path = std::env::var("EX_POLYGON_REST_VECTORS")
        .unwrap_or_else(|_| format!("{}/tests/data/ex_polygon_rest/vectors.txt.gz", env!("CARGO_MANIFEST_DIR")));
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
    fn exs(&mut self) -> Vec<ExPolygon> {
        let n = self.i() as usize;
        (0..n)
            .map(|_| {
                let nh = self.i() as usize;
                let contour = Polygon::from_points(self.pts());
                let holes = (0..nh).map(|_| Polygon::from_points(self.pts())).collect();
                ExPolygon::with_holes(contour, holes)
            })
            .collect()
    }
    fn lines(&mut self) -> Vec<Line> {
        let n = self.i() as usize;
        (0..n).map(|_| Line::new(self.pt(), self.pt())).collect()
    }
}

fn ex_eq(a: &[ExPolygon], b: &[ExPolygon]) -> bool {
    a.len() == b.len()
        && a.iter().zip(b).all(|(x, y)| {
            x.contour.points == y.contour.points
                && x.holes.len() == y.holes.len()
                && x.holes.iter().zip(&y.holes).all(|(h, g)| h.points == g.points)
        })
}
fn lines_eq(a: &[Line], b: &[Line]) -> bool {
    a.len() == b.len() && a.iter().zip(b).all(|(x, y)| x.a == y.a && x.b == y.b)
}

#[test]
fn ex_polygon_free_functions_match_the_pin() {
    let text = read_vectors();
    let mut calls: BTreeMap<&'static str, usize> = BTreeMap::new();
    let mut bad: BTreeMap<&'static str, usize> = BTreeMap::new();
    let mut first_bad: Vec<String> = Vec::new();
    let mut check = |tag: &'static str, ok: bool, case: i64| {
        *calls.entry(tag).or_default() += 1;
        if !ok {
            *bad.entry(tag).or_default() += 1;
            if first_bad.len() < 12 {
                first_bad.push(format!("case {case} {tag}"));
            }
        }
    };
    let mut ex: Vec<ExPolygon> = Vec::new();
    let mut case = -1i64;
    let mut cases = 0usize;
    for line in text.lines() {
        let mut t = Tok { t: line.split_ascii_whitespace().collect(), k: 0 };
        match t.s() {
            "C" => {
                case = t.i();
                ex = t.exs();
                cases += 1;
            }
            "N" => {
                check("count_points(ExPolygons)", count_points(&ex) == t.i() as usize, case);
                for e in &ex {
                    check("count_points(ExPolygon)", count_points(e) == t.i() as usize, case);
                }
                check("number_polygons", number_polygons(&ex) == t.i() as usize, case);
            }
            "L" => check("to_lines(ExPolygons)", lines_eq(&to_lines(&ex), &t.lines()), case),
            "L1" => check("to_lines(ExPolygon)", lines_eq(&to_lines(&ex[0]), &t.lines()), case),
            "P" => check("to_points(ExPolygons)", to_points(&ex) == t.pts(), case),
            "P1" => check("to_points(ExPolygon)", to_points(&ex[0]) == t.pts(), case),
            "T" => {
                let d = t.pt();
                let mut e2 = ex.clone();
                translate(&mut e2, &d);
                check("translate(ExPolygons)", ex_eq(&e2, &t.exs()), case);
            }
            "R" => {
                let a = t.d();
                let mut e2 = ex.clone();
                expolygons_rotate(&mut e2, a);
                check("expolygons_rotate", ex_eq(&e2, &t.exs()), case);
            }
            "S" => {
                let tol = t.d();
                check("expolygons_simplify", ex_eq(&expolygons_simplify(&ex, tol), &t.exs()), case);
            }
            "Q" => {
                let pt = t.pt();
                let border = t.i() != 0;
                let want = t.i() != 0;
                check("expolygons_contain", expolygons_contain(&ex, &pt, border) == want, case);
            }
            "A" => {
                let n = t.i() as usize;
                let want: Vec<Vec<Point>> = (0..n).map(|_| t.pts()).collect();
                let mut dst = vec![ex[0].contour.clone()];
                polygons_append(&mut dst, &ex);
                polygons_append(&mut dst, &ex[0]);
                polygons_append(&mut dst, ex.clone());
                polygons_append(&mut dst, ex.last().unwrap().clone());
                let got: Vec<Vec<Point>> = dst.iter().map(|p| p.points.clone()).collect();
                check("polygons_append", got == want, case);
            }
            "E" => {
                let want = t.exs();
                let mut dst = vec![ex.last().unwrap().clone()];
                expolygons_append(&mut dst, &ex);
                expolygons_append(&mut dst, ex.clone());
                let mut empty: Vec<ExPolygon> = Vec::new();
                expolygons_append(&mut empty, ex.clone());
                expolygons_append(&mut dst, &empty);
                check("expolygons_append", ex_eq(&dst, &want), case);
            }
            "PL" => {
                let n = t.i() as usize;
                let want: Vec<Vec<Point>> = (0..n).map(|_| t.pts()).collect();
                let mut got = to_polylines(&ex);
                got.extend(to_polylines(&ex[0]));
                got.extend(to_polylines(ex.clone()));
                got.extend(to_polylines(ex.last().unwrap().clone()));
                let got: Vec<Vec<Point>> = got.iter().map(|p| p.points().to_vec()).collect();
                check("to_polylines", got == want, case);
            }
            "PG" => {
                let n = t.i() as usize;
                let want: Vec<Vec<Point>> = (0..n).map(|_| t.pts()).collect();
                let mut got = to_polygons(&ex);
                got.extend(to_polygons(&ex[0]));
                got.extend(to_polygons(ex.clone()));
                got.extend(to_polygons(ex.last().unwrap().clone()));
                let got: Vec<Vec<Point>> = got.iter().map(|p| p.points.clone()).collect();
                check("to_polygons", got == want, case);
                // ExPolygon.hpp:306-326: the same rings by pointer
                let ptrs: Vec<Vec<Point>> = to_polygon_ptrs(ex.as_slice()).iter().map(|p| p.points.clone()).collect();
                let n0 = number_polygons(&ex);
                check("to_polygon_ptrs", ptrs[..] == want[..n0], case);
            }
            "TE" => {
                let want = t.exs();
                let pp = to_polygons(&ex);
                let mut a = to_expolygons(&pp);
                a.extend(to_expolygons(pp));
                check("to_expolygons", ex_eq(&a, &want), case);
            }
            other => panic!("unknown tag {other}"),
        }
    }
    let total: usize = calls.values().sum();
    let nbad: usize = bad.values().sum();
    eprintln!("ex_polygon_rest: {cases} cases, {total} calls, {nbad} mismatches");
    for (k, v) in &calls {
        eprintln!("  {k}: {v} calls, {} mismatches", bad.get(k).copied().unwrap_or(0));
    }
    for s in &first_bad {
        eprintln!("  {s}");
    }
    assert!(cases > 0);
    assert_eq!(nbad, 0, "{nbad} mismatches");
}
