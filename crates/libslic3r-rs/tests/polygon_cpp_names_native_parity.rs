//! Polygon.hpp / Polygon.cpp overloads under their C++ names against the pin
//! (tests/data/polygon_cpp_names, gen.cpp): contains(Polygon) / contains(Polygons) (both border
//! results), area(Polygons), has_duplicate_points(const Polygon&), to_lines(Polygons),
//! to_polylines(Polygons&&), to_polygons(const paths&) / (paths&&), polygons_append (copy / move),
//! remove_same_neighbor(Polygon&) / (Polygons&).

use slicer::geometry::{
    HasDuplicatePoints, PathsToPolygons, Point, Polygon, PolygonContains, Polygons, PolygonsAppend,
    PolygonsArea, PolygonsToLines, PolygonsToPolylines, RemoveSameNeighbor,
};
use std::collections::BTreeMap;
use std::io::Read;

fn vectors() -> String {
    let path = format!("{}/tests/data/polygon_cpp_names/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let raw = std::fs::read(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut s = String::new();
    flate2::read::GzDecoder::new(&raw[..]).read_to_string(&mut s).unwrap();
    s
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
        self.s().parse().unwrap()
    }
    fn p(&mut self) -> Point {
        let x = self.i();
        let y = self.i();
        Point::new(x, y)
    }
    fn pts(&mut self) -> Vec<Point> {
        let n = self.i() as usize;
        (0..n).map(|_| self.p()).collect()
    }
    fn polys(&mut self, tag: &str) -> Vec<Vec<Point>> {
        assert_eq!(self.s(), tag);
        let n = self.i() as usize;
        (0..n).map(|_| self.pts()).collect()
    }
}

fn polygon(points: Vec<Point>) -> Polygon {
    let mut p = Polygon::new();
    p.points = points;
    p
}
fn pts_of(ps: &[Polygon]) -> Vec<Vec<Point>> {
    ps.iter().map(|p| p.points.clone()).collect()
}

#[test]
fn polygon_cpp_names_match_the_pin() {
    let text = vectors();
    let mut counts = BTreeMap::<&'static str, (usize, usize)>::new();
    let mut check = |name: &'static str, ok: bool| {
        let e = counts.entry(name).or_insert((0, 0));
        e.0 += 1;
        if !ok {
            e.1 += 1;
        }
    };
    for (li, line) in text.lines().enumerate() {
        let mut t = Tok { t: line.split_whitespace().collect(), k: 0 };
        assert_eq!(t.s(), "PG");
        let ps: Polygons = t.polys("in").into_iter().map(polygon).collect();
        assert_eq!(t.s(), "q");
        let q = t.p();
        assert_eq!(t.s(), "|");
        let (c1, c2, c3, c4) = (t.i() == 1, t.i() == 1, t.i() == 1, t.i() == 1);
        check("contains(const Polygon&, Point, bool)", PolygonContains::contains(&ps[0], &q, false) == c1 && PolygonContains::contains(&ps[0], &q, true) == c2);
        check("contains(const Polygons&, Point, bool)", PolygonContains::contains(&ps[..], &q, false) == c3 && PolygonContains::contains(&ps[..], &q, true) == c4);
        let a = f64::from_bits(u64::from_str_radix(t.s(), 16).unwrap());
        check("area(const Polygons&)", PolygonsArea::area(&ps[..]).to_bits() == a.to_bits());
        check("has_duplicate_points(const Polygon&)", HasDuplicatePoints::has_duplicate_points(&ps[0]) == (t.i() == 1));
        assert_eq!(t.s(), "L");
        let n = t.i() as usize;
        let lines: Vec<(Point, Point)> = (0..n).map(|_| (t.p(), t.p())).collect();
        let got: Vec<(Point, Point)> = ps[..].to_lines().iter().map(|l| (l.a, l.b)).collect();
        check("to_lines(const Polygons&)", got == lines);
        assert_eq!(t.s(), "PL");
        let n = t.i() as usize;
        let pls: Vec<Vec<Point>> = (0..n).map(|_| t.pts()).collect();
        let got: Vec<Vec<Point>> = ps.clone().to_polylines().into_iter().map(|pl| pl.points).collect();
        check("to_polylines(Polygons&&)", got == pls);
        let mut paths: Vec<Vec<Point>> = pts_of(&ps);
        if li % 3 == 0 {
            paths.push(Vec::new());
        }
        let tp1 = t.polys("TP1");
        check("to_polygons(const std::vector<Points>&)", pts_of(&(&paths[..]).to_polygons()) == tp1);
        let tp2 = t.polys("TP2");
        check("to_polygons(std::vector<Points>&&)", pts_of(&paths.clone().to_polygons()) == tp2);
        let mut dst1: Polygons = Vec::new();
        if li % 2 == 1 {
            dst1.push(ps.last().unwrap().clone());
        }
        let mut dst2 = dst1.clone();
        dst1.polygons_append(&ps[..]);
        check("polygons_append(Polygons&, const Polygons&)", pts_of(&dst1) == t.polys("A1"));
        dst2.polygons_append(ps.clone());
        check("polygons_append(Polygons&, Polygons&&)", pts_of(&dst2) == t.polys("A2"));
        assert_eq!(t.s(), "R1");
        let r1 = t.i() == 1;
        let one_pts = t.pts();
        let mut one = ps[0].clone();
        let g1 = RemoveSameNeighbor::remove_same_neighbor(&mut one);
        check("remove_same_neighbor(Polygon&)", g1 == r1 && one.points == one_pts);
        assert_eq!(t.s(), "R2");
        let r2 = t.i() == 1;
        let rs = t.polys("RS");
        let mut all = ps.clone();
        let g2 = RemoveSameNeighbor::remove_same_neighbor(&mut all);
        check("remove_same_neighbor(Polygons&)", g2 == r2 && pts_of(&all) == rs);
    }
    let mut bad = 0;
    for (name, (n, m)) in &counts {
        eprintln!("{name}: {n} cases, {m} mismatches");
        bad += m;
    }
    assert_eq!(counts.len(), 12);
    assert_eq!(bad, 0);
}
