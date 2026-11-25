//! The members of class MultiPoint (MultiPoint.hpp:17-115, MultiPoint.cpp:6-177) under their C++
//! names, on Polygons and Polylines, against the pin (tests/data/multi_point_cpp_names, gen.cpp):
//! scale x2, translate x2, rotate x3, reverse, size, empty, is_valid, closest_point_index,
//! closest_point, distance_to, clear, append x4, intersection, first_intersection, intersections.

use slicer::geometry::{Line, Point, Polygon, Polyline};
use slicer::multi_point::{
    MultiPoint, MultiPointAppend, MultiPointRotate, MultiPointScale, MultiPointTranslate,
};
use std::collections::BTreeMap;
use std::io::Read;

fn vectors() -> String {
    let path = format!("{}/tests/data/multi_point_cpp_names/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
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
    fn d(&mut self) -> f64 {
        f64::from_bits(u64::from_str_radix(self.s(), 16).unwrap())
    }
    fn p(&mut self) -> Point {
        let x = self.i();
        let y = self.i();
        Point::new(x, y)
    }
    fn pts(&mut self, tag: &str) -> Vec<Point> {
        assert_eq!(self.s(), tag);
        let n = self.i() as usize;
        (0..n).map(|_| self.p()).collect()
    }
}

fn run<T: MultiPoint>(mp: &mut T, t: &mut Tok, check: &mut dyn FnMut(&'static str, bool)) {
    let q = t.p();
    assert_eq!(t.s(), "ln");
    let ln = Line::new(t.p(), t.p());
    assert_eq!(t.s(), "f");
    let (f, fx, fy, tx, ty, a, ca, sa) = (t.d(), t.d(), t.d(), t.d(), t.d(), t.d(), t.d(), t.d());
    assert_eq!(t.s(), "c");
    let c = t.p();
    assert_eq!(t.s(), "|");
    check("MultiPoint::size", mp.size() as i64 == t.i());
    check("MultiPoint::empty", mp.empty() == (t.i() == 1));
    check("MultiPoint::is_valid", mp.is_valid() == (t.i() == 1));
    check("MultiPoint::closest_point_index", mp.closest_point_index(&q) as i64 == t.i());
    let cl = t.p();
    check("MultiPoint::closest_point", mp.closest_point(&q).map_or(true, |p| *p == cl));
    let dist = t.d();
    let got = if mp.empty() { 0.0 } else { mp.distance_to(&q) };
    check("MultiPoint::distance_to", got.to_bits() == dist.to_bits());
    let (b1, ip, b2, fp, b3) = (t.i() == 1, t.p(), t.i() == 1, t.p(), t.i() == 1);
    let n = t.i() as usize;
    let ips: Vec<Point> = (0..n).map(|_| t.p()).collect();
    let mut gip = Point::new(0, 0);
    let ok = mp.intersection(&ln, &mut gip) == b1 && gip == ip;
    check("MultiPoint::intersection", ok);
    let mut gfp = Point::new(0, 0);
    let ok = mp.first_intersection(&ln, &mut gfp) == b2 && gfp == fp;
    check("MultiPoint::first_intersection", ok);
    let mut gips = Vec::new();
    let ok = mp.intersections(&ln, &mut gips) == b3 && gips == ips;
    check("MultiPoint::intersections", ok);
    let orig = mp.points().clone();
    let mut step = |name: &'static str, tag: &str, mp: &mut T, t: &mut Tok, op: &dyn Fn(&mut T)| {
        op(mp);
        let want = t.pts(tag);
        check(name, *mp.points() == want);
        *mp.points_mut() = orig.clone();
    };
    step("MultiPoint::scale(double)", "s1", mp, t, &|m| MultiPointScale::<f64>::scale(m, f));
    step("MultiPoint::scale(double, double)", "s2", mp, t, &|m| MultiPointScale::<(f64, f64)>::scale(m, (fx, fy)));
    step("MultiPoint::translate(double, double)", "t1", mp, t, &|m| MultiPointTranslate::<(f64, f64)>::translate(m, (tx, ty)));
    step("MultiPoint::translate(const Point&)", "t2", mp, t, &|m| MultiPointTranslate::<&Point>::translate(m, &c));
    step("MultiPoint::rotate(double)", "r1", mp, t, &|m| MultiPointRotate::<f64>::rotate(m, a));
    step("MultiPoint::rotate(double, double)", "r2", mp, t, &|m| MultiPointRotate::<(f64, f64)>::rotate(m, (ca, sa)));
    step("MultiPoint::rotate(double, const Point&)", "r3", mp, t, &|m| MultiPointRotate::<(f64, &Point)>::rotate(m, (a, &c)));
    step("MultiPoint::reverse", "rv", mp, t, &|m| MultiPoint::reverse(m));
    let src = vec![c, q];
    step("MultiPoint::append(const Point&)", "a1", mp, t, &|m| MultiPointAppend::<&Point>::append(m, &q));
    step("MultiPoint::append(const Points&)", "a2", mp, t, &|m| MultiPointAppend::<&[Point]>::append(m, &src[..]));
    step("MultiPoint::append(begin, end)", "a3", mp, t, &|m| MultiPointAppend::append(m, src[1..].iter()));
    step("MultiPoint::append(Points&&)", "a4", mp, t, &|m| MultiPointAppend::<Vec<Point>>::append(m, src.clone()));
    step("MultiPoint::clear", "cl", mp, t, &|m| MultiPoint::clear(m));
}

#[test]
fn multi_point_cpp_names_match_the_pin() {
    let text = vectors();
    let mut counts = BTreeMap::<&'static str, (usize, usize)>::new();
    let mut check = |name: &'static str, ok: bool| {
        let e = counts.entry(name).or_insert((0, 0));
        e.0 += 1;
        if !ok {
            e.1 += 1;
        }
    };
    for line in text.lines() {
        let mut t = Tok { t: line.split_whitespace().collect(), k: 0 };
        assert_eq!(t.s(), "MP");
        let kind = t.i();
        let pts = t.pts("in");
        assert_eq!(t.s(), "q");
        if kind == 0 {
            let mut pg = Polygon::new();
            pg.points = pts;
            run(&mut pg, &mut t, &mut check);
        } else {
            let mut pl = Polyline::new();
            pl.points = pts;
            run(&mut pl, &mut t, &mut check);
        }
    }
    let mut bad = 0;
    for (name, (n, m)) in &counts {
        eprintln!("{name}: {n} cases, {m} mismatches");
        bad += m;
    }
    assert_eq!(counts.len(), 22);
    assert_eq!(bad, 0);
}
