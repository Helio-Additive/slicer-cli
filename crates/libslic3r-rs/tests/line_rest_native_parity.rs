//! `slicer::geometry::line_alg` (the port of `namespace line_alg`, Line.hpp:23-135, on a 2D integer
//! `Line`) and the classes `Line3`, `Linef3` and `Line3float` (Line.hpp:212-290, Line.cpp:128-133) against
//! the C++ at the pin.
//!
//! `tests/data/line_rest/vectors.txt.gz` is the output of `gen.cpp` beside it; the header of gen.cpp
//! describes the line. Integers have to be equal, doubles bit-equal.

use slicer::geometry::{line_alg, Line, Line3, Line3float, Linef3, Point, Point3, Point3F};
use std::collections::BTreeMap;
use std::io::Read;

fn read_vectors() -> String {
    let path = std::env::var("LINE_REST_VECTORS")
        .unwrap_or_else(|_| format!("{}/tests/data/line_rest/vectors.txt.gz", env!("CARGO_MANIFEST_DIR")));
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
    fn f(&mut self) -> f32 {
        f32::from_bits(u32::from_str_radix(self.s(), 16).expect("float"))
    }
    fn p3(&mut self) -> Point3F {
        Point3F::new(self.d(), self.d(), self.d())
    }
    fn p3f(&mut self) -> [f32; 3] {
        [self.f(), self.f(), self.f()]
    }
    fn p3c(&mut self) -> Point3 {
        Point3::new(self.i(), self.i(), self.i())
    }
    fn pt(&mut self) -> Point {
        let x = self.i();
        let y = self.i();
        Point::new(x, y)
    }
}

#[test]
fn line_alg_matches_the_pin() {
    let text = read_vectors();
    let mut calls: BTreeMap<&'static str, (usize, usize)> = BTreeMap::new();
    let mut first: Vec<String> = Vec::new();
    let mut cases = 0usize;
    let mut inside = 0usize;
    for (ln, line_text) in text.lines().enumerate() {
        let mut t = Tok { t: line_text.split(' ').filter(|x| !x.is_empty()).collect(), k: 0 };
        assert_eq!(t.s(), "c");
        cases += 1;
        let a = t.pt();
        let b = t.pt();
        let p = t.pt();
        assert_eq!(t.s(), "->");
        let line = Line::new(a, b);
        let mut check = |name: &'static str, ok: bool, what: String| {
            let e = calls.entry(name).or_insert((0, 0));
            e.0 += 1;
            if !ok {
                e.1 += 1;
                if first.len() < 12 {
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
        macro_rules! dbl_eq {
            ($name:expr, $got:expr) => {{
                let want = t.d();
                let got: f64 = $got;
                check($name, got.to_bits() == want.to_bits(), format!("got {got:e} want {want:e}"));
            }};
        }
        pt_eq!("get_a", line_alg::get_a(&line));
        pt_eq!("get_b", line_alg::get_b(&line));
        {
            let mut nearest = Point::new(0, 0);
            dbl_eq!("distance_to_squared(line, point, nearest)", line_alg::distance_to_squared(&line, &p, &mut nearest));
            if nearest != a && nearest != b {
                inside += 1;
            }
            pt_eq!("distance_to_squared nearest_point", nearest);
            dbl_eq!("distance_to(line, point)", line_alg::distance_to(&line, &p));
        }
        {
            let mut closest = Point::new(0, 0);
            dbl_eq!(
                "distance_to_infinite_squared(line, point, closest)",
                line_alg::distance_to_infinite_squared(&line, &p, &mut closest)
            );
            pt_eq!("distance_to_infinite_squared closest_point", closest);
            dbl_eq!("distance_to_infinite(line, point)", line_alg::distance_to_infinite(&line, &p));
        }
        assert_eq!(t.s(), "|");
        let l3 = Line3::new(t.p3c(), t.p3c());
        let lf = Linef3::new(t.p3(), t.p3());
        let z = t.d();
        let factor = t.d();
        let pt = t.p3();
        let l3f = Line3float::new(t.p3f(), t.p3f());
        assert_eq!(t.s(), "->");
        macro_rules! p3_eq {
            ($name:expr, $got:expr) => {{
                let want = t.p3();
                let got: Point3F = $got;
                let ok = got.x.to_bits() == want.x.to_bits() && got.y.to_bits() == want.y.to_bits() && got.z.to_bits() == want.z.to_bits();
                check($name, ok, format!("got {got:?} want {want:?}"));
            }};
        }
        macro_rules! p3f_eq {
            ($name:expr, $got:expr) => {{
                let want = t.p3f();
                let got: [f32; 3] = $got;
                let ok = (0..3).all(|k| got[k].to_bits() == want[k].to_bits());
                check($name, ok, format!("got {got:?} want {want:?}"));
            }};
        }
        macro_rules! flt_eq {
            ($name:expr, $got:expr) => {{
                let want = t.f();
                let got: f32 = $got;
                check($name, got.to_bits() == want.to_bits(), format!("got {got:e} want {want:e}"));
            }};
        }
        dbl_eq!("Line3::length", l3.length());
        {
            let want = t.p3c();
            let got = l3.vector();
            check("Line3::vector", got == want, format!("got {got:?} want {want:?}"));
        }
        p3_eq!("Linef3::intersect_plane", lf.intersect_plane(z));
        {
            let mut c = lf;
            c.scale(factor);
            p3_eq!("Linef3::scale", c.a);
            p3_eq!("Linef3::scale", c.b);
        }
        p3_eq!("Linef3::vector", lf.vector());
        p3_eq!("Linef3::unit_vector", lf.unit_vector());
        dbl_eq!("Linef3::length", lf.length());
        {
            let mut c = lf;
            c.reverse();
            p3_eq!("Linef3::reverse", c.a);
            p3_eq!("Linef3::reverse", c.b);
        }
        {
            let mut ip = Point3F::new(0.0, 0.0, 0.0);
            let mut pl = 0f32;
            Linef3::get_point_projection_to_line(&pt, &lf.a, &lf.vector(), &mut ip, &mut pl);
            p3_eq!("Linef3::get_point_projection_to_line", ip);
            flt_eq!("Linef3::get_point_projection_to_line proj_length", pl);
        }
        flt_eq!("Linef3::get_distance_of_point_to_line", lf.get_distance_of_point_to_line(&pt));
        p3f_eq!("Line3float::vector", l3f.vector());
        p3f_eq!("Line3float::unit_vector", l3f.unit_vector());
        dbl_eq!("Line3float::length", l3f.length());
        assert_eq!(t.s(), "|");
        let mut m = [0f64; 16];
        for v in m.iter_mut() {
            *v = t.d();
        }
        assert_eq!(t.s(), "->");
        {
            let tl = slicer::geometry::line_transform(&lf, &slicer::geometry::Transform3D { matrix: m });
            p3_eq!("transform(Linef3, Transform3d)", tl.a);
            p3_eq!("transform(Linef3, Transform3d)", tl.b);
        }
        assert_eq!(t.k, t.t.len(), "line {}: tokens left", ln + 1);
    }
    let mut total = 0usize;
    let mut bad = 0usize;
    for (name, (n, m)) in &calls {
        println!("{name}: {n} calls, {m} mismatches");
        total += n;
        bad += m;
    }
    println!("line_rest: {cases} cases, {total} calls, {bad} mismatches; nearest point inside the segment {inside}");
    for f in &first {
        println!("{f}");
    }
    assert_eq!(cases, 3000);
    assert_eq!(bad, 0, "{bad} of {total} results differ from the pin");
}
