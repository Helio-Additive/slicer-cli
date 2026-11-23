//! `slicer::multi_point` (the port of libslic3r/MultiPoint.hpp / MultiPoint.cpp) against the C++ at the pin.
//!
//! `tests/data/multi_point/vectors.txt.gz` is the output of `gen.cpp` beside it (the pin's MultiPoint through
//! Polyline, whose lines() are the open chain); the header of gen.cpp describes the line. Every result has to be
//! the pin's (integers equal, doubles bit-equal). MultiPoint::visivalingam is not compared: it is undefined
//! behaviour at the pin (gen.cpp says why). `area.txt.gz` (`gen area`) holds area(const Points&) on 3..99
//! points, where clang's vectorization of the loop decides which products are fused; `mp3.txt.gz` (`gen mp3`)
//! holds MultiPoint3 through Polyline3. R933.

use slicer::geometry::{BoundingBox, Point};
use slicer::multi_point as mp;
use std::collections::BTreeMap;
use std::io::Read;

fn read_vectors() -> String {
    let path = std::env::var("MULTI_POINT_VECTORS")
        .unwrap_or_else(|_| format!("{}/tests/data/multi_point/vectors.txt.gz", env!("CARGO_MANIFEST_DIR")));
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
    fn bx(&mut self) -> (bool, Point, Point) {
        let d = self.i() != 0;
        let a = self.pt();
        let b = self.pt();
        (d, a, b)
    }
}

fn box_of(b: &BoundingBox) -> (bool, Point, Point) {
    (b.is_defined(), b.min, b.max)
}

#[test]
fn multi_point_matches_the_pin() {
    let text = read_vectors();
    let mut calls: BTreeMap<&'static str, (usize, usize)> = BTreeMap::new();
    let mut first: Vec<String> = Vec::new();
    let mut cases = 0usize;
    for (ln, line) in text.lines().enumerate() {
        let mut t = Tok { t: line.split(' ').filter(|x| !x.is_empty()).collect(), k: 0 };
        assert_eq!(t.s(), "m");
        cases += 1;
        let pts = t.pts();
        let v = t.pt();
        let c = t.d();
        let s = t.d();
        let angle = t.d();
        let center = t.pt();
        let q = t.pt();
        let eps = t.d();
        let tol = t.d();
        let x_axis = t.i();
        assert_eq!(t.s(), "->");

        let mut check = |name: &'static str, ok: bool, what: String| {
            let e = calls.entry(name).or_insert((0, 0));
            e.0 += 1;
            if !ok {
                e.1 += 1;
                if first.len() < 16 && !name.ends_with("reported)") {
                    first.push(format!("line {}: {name}: {what}", ln + 1));
                }
            }
        };
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
        macro_rules! box_eq {
            ($name:expr, $got:expr) => {{
                let want = t.bx();
                let got = box_of(&$got);
                // An undefined box: only `defined` is compared (the pin leaves min/max as constructed).
                let ok = got.0 == want.0 && (!want.0 || (got.1 == want.1 && got.2 == want.2));
                check($name, ok, format!("got {got:?} want {want:?}"));
            }};
        }

        pts_eq!("translate", {
            let mut p = pts.clone();
            mp::translate(&mut p, v);
            p
        });
        pts_eq!("rotate(cos, sin)", {
            let mut p = pts.clone();
            mp::rotate(&mut p, c, s);
            p
        });
        pts_eq!("rotate(angle, center)", {
            let mut p = pts.clone();
            mp::rotate_around(&mut p, angle, center);
            p
        });
        dbl_eq!("length()", mp::length(&pts));
        dbl_eq!("length(Points)", mp::length_points(&pts));
        int_eq!("find_point", mp::find_point(&pts, &q) as i64);
        int_eq!("find_point(eps)", mp::find_point_eps(&pts, &q, eps) as i64);
        int_eq!("has_boundary_point", mp::has_boundary_point(&pts, &q) as i64);
        box_eq!("bounding_box", mp::bounding_box(&pts));
        int_eq!("has_duplicate_points", mp::has_duplicate_points(&pts) as i64);
        {
            let mut p = pts.clone();
            let r = mp::remove_duplicate_points(&mut p);
            int_eq!("remove_duplicate_points", r as i64);
            pts_eq!("remove_duplicate_points", p);
        }
        {
            let mut p = pts.clone();
            let r = mp::remove_colinear_points(&mut p);
            int_eq!("remove_colinear_points", r as i64);
            pts_eq!("remove_colinear_points", p);
        }
        if t.i() != 0 {
            pts_eq!("concave_hull_2d", mp::concave_hull_2d(&pts, tol));
        }
        pts_eq!("symmetric_y", {
            let mut p = pts.clone();
            mp::symmetric_y(&mut p, x_axis);
            p
        });
        dbl_eq!("area(Points)", mp::area(&pts));
        box_eq!("get_extents", mp::get_extents(&pts));
        assert_eq!(t.k, t.t.len(), "line {}: tokens left", ln + 1);
    }
    let mut total = 0usize;
    let mut bad = 0usize;
    for (name, (n, m)) in &calls {
        println!("{name}: {n} calls, {m} mismatches");
        if !name.ends_with("reported)") {
            total += n;
            bad += m;
        }
    }
    println!("multi_point: {cases} cases, {total} asserted calls, {bad} mismatches");
    for f in &first {
        println!("{f}");
    }
    assert_eq!(cases, 5000);
    assert_eq!(bad, 0, "{bad} of {total} results differ from the pin");
}

/// MultiPoint.hpp:144-149 `area(const Points&)` on n = 3..99 points with coordinates up to 1e8 (`gen area`):
/// clang's ordered vectorization of the loop (16 iterations per step) against the fused scalar remainder.
#[test]
fn multi_point_area_matches_the_pin() {
    let path = std::env::var("MULTI_POINT_AREA")
        .unwrap_or_else(|_| format!("{}/tests/data/multi_point/area.txt.gz", env!("CARGO_MANIFEST_DIR")));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    if path.ends_with(".gz") {
        flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    } else {
        std::io::BufReader::new(file).read_to_string(&mut text).expect("read");
    }
    let (mut cases, mut bad) = (0usize, 0usize);
    let mut first: Vec<String> = Vec::new();
    for (ln, line) in text.lines().enumerate() {
        let mut t = Tok { t: line.split(' ').filter(|x| !x.is_empty()).collect(), k: 0 };
        assert_eq!(t.s(), "a");
        cases += 1;
        let pts = t.pts();
        assert_eq!(t.s(), "->");
        let want = t.d();
        let got = mp::area(&pts);
        if got.to_bits() != want.to_bits() {
            bad += 1;
            if first.len() < 8 {
                first.push(format!("line {} (n = {}): got {got:e} want {want:e}", ln + 1, pts.len()));
            }
        }
        assert_eq!(t.k, t.t.len(), "line {}: tokens left", ln + 1);
    }
    println!("area(Points): {cases} cases, {bad} mismatches");
    for f in &first {
        println!("{f}");
    }
    assert_eq!(cases, 97 * 8);
    assert_eq!(bad, 0, "{bad} of {cases} areas differ from the pin");
}

/// MultiPoint.hpp:111-125 / MultiPoint.cpp:388-433 `MultiPoint3` through the pin's Polyline3 (`gen mp3`): translate
/// by doubles and by a Point, length of the open chain, is_valid, bounding_box (BoundingBox3 of the points),
/// remove_duplicate_points.
#[test]
fn multi_point3_matches_the_pin() {
    use slicer::geometry::Point3;
    let path = std::env::var("MULTI_POINT3_VECTORS")
        .unwrap_or_else(|_| format!("{}/tests/data/multi_point/mp3.txt.gz", env!("CARGO_MANIFEST_DIR")));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    if path.ends_with(".gz") {
        flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    } else {
        std::io::BufReader::new(file).read_to_string(&mut text).expect("read");
    }
    fn p3s(t: &mut Tok) -> Vec<Point3> {
        let n = t.i() as usize;
        (0..n)
            .map(|_| {
                let x = t.i();
                let y = t.i();
                let z = t.i();
                Point3::new(x, y, z)
            })
            .collect()
    }
    let mut calls: BTreeMap<&'static str, (usize, usize)> = BTreeMap::new();
    let mut first: Vec<String> = Vec::new();
    let mut cases = 0usize;
    for (ln, line) in text.lines().enumerate() {
        let mut t = Tok { t: line.split(' ').filter(|x| !x.is_empty()).collect(), k: 0 };
        assert_eq!(t.s(), "t");
        cases += 1;
        let mp3 = mp::MultiPoint3 { points: p3s(&mut t) };
        let x = t.d();
        let y = t.d();
        let v = t.pt();
        assert_eq!(t.s(), "->");
        let mut check = |name: &'static str, ok: bool, what: String| {
            let e = calls.entry(name).or_insert((0, 0));
            e.0 += 1;
            if !ok {
                e.1 += 1;
                if first.len() < 16 {
                    first.push(format!("line {}: {name}: {what}", ln + 1));
                }
            }
        };
        {
            let mut m = mp3.clone();
            m.translate(x, y);
            let want = p3s(&mut t);
            check("translate(x, y)", m.points == want, format!("got {:?} want {want:?}", m.points));
        }
        {
            let mut m = mp3.clone();
            m.translate_point(&v);
            let want = p3s(&mut t);
            check("translate(Point)", m.points == want, format!("got {:?} want {want:?}", m.points));
        }
        {
            let want = t.d();
            let got = mp3.length();
            check("length", got.to_bits() == want.to_bits(), format!("got {got:e} want {want:e}"));
        }
        {
            let want = t.i() != 0;
            check("is_valid", mp3.is_valid() == want, String::new());
        }
        {
            let wd = t.i() != 0;
            let wmin = p3s_one(&mut t);
            let wmax = p3s_one(&mut t);
            let b = mp3.bounding_box();
            let ok = b.is_defined() == wd && b.min == wmin && b.max == wmax;
            check("bounding_box", ok, format!("got {:?} {:?} {} want {wmin:?} {wmax:?} {wd}", b.min, b.max, b.is_defined()));
        }
        {
            let mut m = mp3.clone();
            let r = m.remove_duplicate_points();
            let wr = t.i() != 0;
            let want = p3s(&mut t);
            check("remove_duplicate_points", r == wr && m.points == want, format!("got {r} {:?} want {wr} {want:?}", m.points));
        }
        assert_eq!(t.k, t.t.len(), "line {}: tokens left", ln + 1);
    }
    fn p3s_one(t: &mut Tok) -> Point3 {
        let x = t.i();
        let y = t.i();
        let z = t.i();
        Point3::new(x, y, z)
    }
    let mut total = 0usize;
    let mut bad = 0usize;
    for (name, (n, m)) in &calls {
        println!("{name}: {n} calls, {m} mismatches");
        total += n;
        bad += m;
    }
    println!("multi_point3: {cases} cases, {total} asserted calls, {bad} mismatches");
    for f in &first {
        println!("{f}");
    }
    assert_eq!(cases, 2000);
    assert_eq!(bad, 0, "{bad} of {total} results differ from the pin");
}
