//! `slicer::geometry::Point` (the port of libslic3r/Point.hpp and Point.cpp) against the C++ at the pin.
//!
//! `tests/data/point/vectors.txt.gz` is the output of `gen.cpp` beside it; the header of gen.cpp describes
//! the line. Every case carries its inputs and the pin's results; the same calls run here and every result
//! has to be the pin's (integers equal, doubles bit-equal). `Point::new_scale` is compared on the arm of its
//! opt-in gate `POINT_NEW_SCALE_PIN`, which the test switches on; `is_in_lines` on the default arm of
//! `POINT_IS_IN_LINES_PIN`. The free `operator*(const Point&, double)` of
//! Point.hpp:255-258 is in the vectors and is not compared: nothing in the Rust file ports it under a name.

use slicer::geometry::{collect_duplicates, has_duplicate_points, Line, Point, Polygon, Polyline};
use std::collections::BTreeMap;
use std::io::Read;

fn read_vectors() -> String {
    let path = std::env::var("POINT_VECTORS")
        .unwrap_or_else(|_| format!("{}/tests/data/point/vectors.txt.gz", env!("CARGO_MANIFEST_DIR")));
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
fn point_matches_the_pin() {
    // `Point::new_scale` is compared on its pin arm, which is an opt-in gate (off by default). The gates are
    // read once per process, on first use, and this is the only test of this binary.
    #[allow(unused_unsafe)]
    unsafe {
        std::env::set_var("POINT_NEW_SCALE_PIN", "1");
    }
    let text = read_vectors();
    let mut calls: BTreeMap<&'static str, (usize, usize)> = BTreeMap::new();
    let mut first: Vec<String> = Vec::new();
    let mut cases = 0usize;
    for (ln, line) in text.lines().enumerate() {
        let mut t = Tok { t: line.split(' ').filter(|x| !x.is_empty()).collect(), k: 0 };
        assert_eq!(t.s(), "c");
        cases += 1;
        let p = t.pt();
        let a = t.pt();
        let b = t.pt();
        let angle = t.d();
        let cos_a = t.d();
        let sin_a = t.d();
        let sx = t.d();
        let sy = t.d();
        let factor = t.d();
        let n = t.i() as usize;
        let pts: Vec<Point> = (0..n).map(|_| t.pt()).collect();
        assert_eq!(t.s(), "->");

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

        pt_eq!("rotated(angle, center)", p.rotated_center_overload(angle, &a));
        pt_eq!("rotated(cos, sin)", p.rotated_cos_sin_overload(cos_a, sin_a));
        pt_eq!("rotated(angle)", p.rotated(angle));
        pt_eq!("rotate_90_degree_ccw", p.rotate_90_degree_ccw());
        int_eq!("nearest_point_index", p.nearest_point_index(&pts) as i64);
        {
            let mut found = Point::new(0, 0);
            let ok = p.nearest_point(&pts, &mut found);
            int_eq!("nearest_point", ok as i64);
            pt_eq!("nearest_point", found);
        }
        dbl_eq!("ccw(p1, p2)", p.ccw(&a, &b));
        dbl_eq!("ccw(line)", p.ccw_line(&Line::new(a, b)));
        dbl_eq!("ccw_angle", p.ccw_angle(&a, &b));
        pt_eq!("projection_onto(Line)", p.projection_onto(&Line::new(a, b)));
        {
            let polyline = Polyline::from_points(pts.clone());
            pt_eq!(
                "projection_onto(MultiPoint) polyline",
                p.projection_onto_multi_point_overload(polyline.first_point(), &polyline.lines())
            );
            let got_old = p.projection_onto_multipoint(&pts);
            let want_old = Point::new(t.t[t.k - 2].parse().unwrap(), t.t[t.k - 1].parse().unwrap());
            check("projection_onto_multipoint (polyline)", got_old == want_old, format!("got {got_old:?} want {want_old:?}"));
            let polygon = Polygon::from_points(pts.clone());
            pt_eq!(
                "projection_onto(MultiPoint) polygon",
                p.projection_onto_multi_point_overload(polygon.first_point(), &polygon.lines())
            );
        }
        int_eq!("is_in_lines", p.is_in_lines(&pts) as i64);
        pt_eq!("new_scale", Point::new_scale(sx, sy));
        int_eq!("both_comp >", p.both_comp(&a, ">") as i64);
        int_eq!("both_comp <", p.both_comp(&a, "<") as i64);
        int_eq!("any_comp >", p.any_comp(&a, ">") as i64);
        int_eq!("any_comp <", p.any_comp(&a, "<") as i64);
        int_eq!("any_comp(val) >", p.any_comp_val(a.x(), ">") as i64);
        int_eq!("any_comp(val) <", p.any_comp_val(a.x(), "<") as i64);
        int_eq!("operator<", (p < a) as i64);
        pt_eq!("operator*(double) member", p * factor);
        let _free_operator_mul = t.pt(); // Point.hpp:255-258, not compared (see the header)
        int_eq!("has_duplicate_points", has_duplicate_points(pts.clone()) as i64);
        {
            let dups = collect_duplicates(pts.clone());
            let want_n = t.i() as usize;
            let want: Vec<Point> = (0..want_n).map(|_| t.pt()).collect();
            check("collect_duplicates", dups == want, format!("got {dups:?} want {want:?}"));
        }
        // int128::orient and int128::cross (Point.cpp:289-299), exported as `slicer::geometry::point_int128`.
        int_eq!("int128::orient", slicer::geometry::point_int128::orient(&p, &a, &b) as i64);
        int_eq!("int128::cross", slicer::geometry::point_int128::cross(&a, &b) as i64);
        assert_eq!(t.k, t.t.len(), "line {}: tokens left", ln + 1);
    }
    let mut total = 0usize;
    let mut bad = 0usize;
    for (name, (n, m)) in &calls {
        println!("{name}: {n} calls, {m} mismatches");
        total += n;
        bad += m;
    }
    println!("point: {cases} cases, {total} calls, {bad} mismatches");
    for f in &first {
        println!("{f}");
    }
    assert_eq!(cases, 4000);
    assert_eq!(bad, 0, "{bad} of {total} results differ from the pin");
}
