//! `slicer::geometry::Line` (the port of libslic3r/Line.hpp and Line.cpp) against the C++ at the pin.
//!
//! `tests/data/line/vectors.txt.gz` is the output of `gen.cpp` beside it; the header of gen.cpp describes
//! the line. Every case carries its inputs and the pin's results; the same calls run here and every result
//! has to be the pin's (integers equal, doubles bit-equal; two NaNs count as equal).

use slicer::geometry::{get_extents_lines, BoundingBox, Line, Point};
use std::collections::BTreeMap;
use std::io::Read;

fn read_vectors() -> String {
    let path = std::env::var("LINE_VECTORS")
        .unwrap_or_else(|_| format!("{}/tests/data/line/vectors.txt.gz", env!("CARGO_MANIFEST_DIR")));
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
}

#[test]
fn line_matches_the_pin() {
    let text = read_vectors();
    let mut calls: BTreeMap<&'static str, (usize, usize)> = BTreeMap::new();
    let mut first: Vec<String> = Vec::new();
    let mut cases = 0usize;
    for (ln, line) in text.lines().enumerate() {
        let mut t = Tok { t: line.split(' ').filter(|x| !x.is_empty()).collect(), k: 0 };
        assert_eq!(t.s(), "c");
        cases += 1;
        let l = Line::new(t.pt(), t.pt());
        let m = Line::new(t.pt(), t.pt());
        let p = t.pt();
        let angle = t.d();
        let offset = t.d();
        let bbox = BoundingBox::from_points_minmax(t.pt(), t.pt());
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
        macro_rules! pt_eq {
            ($name:expr, $got:expr) => {{
                let want = t.pt();
                let got: Point = $got;
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
                check(
                    $name,
                    got.to_bits() == want.to_bits() || (got.is_nan() && want.is_nan()),
                    format!("got {got:e} want {want:e}"),
                );
            }};
        }

        dbl_eq!("length", l.length());
        pt_eq!("midpoint", l.midpoint());
        dbl_eq!("atan2_", l.atan2_());
        dbl_eq!("orientation", l.orientation());
        dbl_eq!("direction", l.direction_angle());
        pt_eq!("vector", l.vector());
        pt_eq!("normal", l.normal());
        dbl_eq!("distance_to_squared(point)", Line::distance_to_squared(p, l.a, l.b));
        dbl_eq!("distance_to(point)", l.distance_to(&p));
        dbl_eq!("distance_to_squared(point, a, b)", Line::distance_to_squared(p, l.a, l.b));
        dbl_eq!("distance_to(point, a, b)", Line::distance_to_squared(p, l.a, l.b).sqrt());
        dbl_eq!("distance_to_infinite_squared(point, a, b)", Line::distance_to_infinite_squared(p, l.a, l.b));
        dbl_eq!("distance_to_infinite(point, a, b)", Line::distance_to_infinite(p, l.a, l.b));
        dbl_eq!("perp_distance_to", l.perp_distance_to(&p));
        int_eq!("parallel_to(line)", l.parallel_to(&m) as i64);
        int_eq!("perpendicular_to(line)", l.perpendicular_to(&m) as i64);
        int_eq!("parallel_to(angle)", l.parallel_to_angle(angle) as i64);
        int_eq!("perpendicular_to(angle)", l.perpendicular_to_angle(angle) as i64);
        {
            let got = l.intersection(&m);
            int_eq!("intersection", got.is_some() as i64);
            pt_eq!("intersection", got.unwrap_or(Point::new(0, 0)));
        }
        {
            let got = l.intersection_infinite(&m);
            int_eq!("intersection_infinite", got.is_some() as i64);
            pt_eq!("intersection_infinite", got.unwrap_or(Point::new(0, 0)));
        }
        {
            let got = l.overlap(&m);
            int_eq!("overlap", got.is_some() as i64);
            dbl_eq!("overlap", got.unwrap_or(0.0));
        }
        {
            let mut c = l;
            let ok = c.clip_with_bbox(&bbox);
            int_eq!("clip_with_bbox", ok as i64);
            pt_eq!("clip_with_bbox", c.a);
            pt_eq!("clip_with_bbox", c.b);
        }
        {
            let mut e = l;
            e.extend(offset);
            pt_eq!("extend", e.a);
            pt_eq!("extend", e.b);
        }
        {
            let bb = get_extents_lines(&[l, m]);
            int_eq!("get_extents(Lines)", bb.is_defined() as i64);
            pt_eq!("get_extents(Lines)", bb.min);
            pt_eq!("get_extents(Lines)", bb.max);
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
    println!("line: {cases} cases, {total} calls, {bad} mismatches");
    for f in &first {
        println!("{f}");
    }
    assert_eq!(cases, 4000);
    assert_eq!(bad, 0, "{bad} of {total} results differ from the pin");
}
