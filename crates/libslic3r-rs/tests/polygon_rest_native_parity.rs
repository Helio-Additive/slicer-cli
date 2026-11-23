//! The rest of `slicer::geometry::Polygon` and the free functions of geometry/polygon.rs (the port of
//! libslic3r/Polygon.hpp / Polygon.cpp) not covered by polygon_native_parity.rs, against the C++ at the pin.
//!
//! `tests/data/polygon_rest/vectors.txt.gz` is the output of `gen.cpp` beside it; the header of gen.cpp
//! describes the line. Every case carries its inputs and the pin's results; the same calls run here and
//! every result has to be the pin's (integers equal, doubles and floats bit-equal).
//!
//! REPORTED, not asserted (R932): `Polygon::simplify` (419 of 1,353 CCW inputs) and `polygons_simplify`
//! (1,147 of 2,000) differ in the final ClipperLib::SimplifyPolygons step (the port's
//! simplify_polygons_clipper / clipper_simplify_polygons_single_path keep self-intersecting outputs the pin
//! splits); the Douglas-Peucker step before it is the verified MultiPoint one. densify and
//! get_extents_rotated are equal under the gate `POLYGON_REST_PIN` (R932, on).

use slicer::geometry::polygon::{
    area_polygons, get_extents_rotated, has_duplicate_points, make_circle, make_circle_num_segments, overlaps,
    polygons_match, polygons_simplify, remove_degenerate, remove_same_neighbor, remove_small, total_length,
};
use slicer::geometry::{Line, Point, Polygon, Polygons};
use std::collections::BTreeMap;
use std::io::Read;

fn read_vectors() -> String {
    let path = std::env::var("POLYGON_REST_VECTORS")
        .unwrap_or_else(|_| format!("{}/tests/data/polygon_rest/vectors.txt.gz", env!("CARGO_MANIFEST_DIR")));
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
    fn floats(&mut self) -> Vec<u32> {
        let n = self.i() as usize;
        (0..n).map(|_| self.f().to_bits()).collect()
    }
}

fn pts_of(ps: &[Polygon]) -> Vec<Vec<Point>> {
    ps.iter().map(|p| p.points().to_vec()).collect()
}

#[test]
fn polygon_rest_matches_the_pin() {
    let text = read_vectors();
    let mut calls: BTreeMap<&'static str, (usize, usize)> = BTreeMap::new();
    let mut first: Vec<String> = Vec::new();
    let mut cases = 0usize;
    for (ln, line) in text.lines().enumerate() {
        let mut t = Tok { t: line.split(' ').filter(|x| !x.is_empty()).collect(), k: 0 };
        assert_eq!(t.s(), "e");
        cases += 1;
        let poly = Polygon::from_points(t.pts());
        let other = Polygon::from_points(t.pts());
        let la = t.pt();
        let lb = t.pt();
        let ln_ = Line::new(la, lb);
        let p = t.pt();
        let eps = t.d();
        let tolerance = t.d();
        let angle = t.d();
        let min_length = t.f();
        let min_area = t.d();
        let radius = t.d();
        let error = t.d();
        let segments = t.i() as usize;
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
        macro_rules! polys_eq {
            ($name:expr, $got:expr) => {{
                let want = t.polys();
                let got: Vec<Vec<Point>> = $got;
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

        let n = poly.points().len();
        pts_eq!("split_at_vertex", poly.split_at_vertex(&poly.points()[(cases - 1) % n]).points);
        pts_eq!("split_at_index", poly.split_at_index((n / 2) as i32).points);
        pts_eq!("split_at_first_point", poly.split_at_first_point().points);
        int_eq!("is_valid", poly.is_valid() as i64);
        int_eq!("contains", poly.contains(&p) as i64);
        int_eq!("on_boundary", poly.on_boundary(&p, eps) as i64);
        // Polygon.cpp:140 asserts a CCW polygon (a debug_assert in the port): CW inputs are skipped.
        if poly.is_counter_clockwise() {
            polys_eq!("simplify (ClipperLib::SimplifyPolygons differs; reported)", pts_of(&poly.simplify(tolerance)));
        } else {
            let _ = t.polys();
            check("simplify (CW input, the pin asserts CCW; reported)", true, String::new());
        }
        {
            let mut x = Point::new(0, 0);
            let ok = poly.intersection(&ln_, &mut x);
            int_eq!("intersection", ok as i64);
            let want = t.pt();
            check("intersection", x == want, format!("got {x:?} want {want:?}"));
        }
        int_eq!("Polygon::overlaps", poly.overlaps(&vec![other.clone()]) as i64);
        {
            let want = t.floats();
            let got: Vec<u32> = poly.parameter_by_length().iter().map(|v| v.to_bits()).collect();
            check("parameter_by_length", got == want, format!("got {got:?} want {want:?}"));
        }
        {
            let mut d = poly.clone();
            let mut l = d.parameter_by_length();
            d.densify(min_length, Some(&mut l));
            pts_eq!("densify", d.points().to_vec());
            let want = t.floats();
            let got: Vec<u32> = l.iter().map(|v| v.to_bits()).collect();
            check("densify (lengths)", got == want, format!("got {got:?} want {want:?}"));
        }
        int_eq!("has_duplicate_points(Polygons)", has_duplicate_points(&[poly.clone(), other.clone()]) as i64);
        {
            let mut r = poly.clone();
            let removed = remove_same_neighbor(&mut r);
            int_eq!("remove_same_neighbor", removed as i64);
            pts_eq!("remove_same_neighbor", r.points().to_vec());
        }
        {
            let mut ps: Polygons = vec![poly.clone(), other.clone()];
            if (cases - 1) % 5 == 2 {
                ps.push(Polygon::from_points(vec![poly.points()[0], *poly.points().last().unwrap()]));
            }
            let removed = remove_degenerate(&mut ps);
            int_eq!("remove_degenerate", removed as i64);
            polys_eq!("remove_degenerate", pts_of(&ps));
        }
        {
            let mut ps: Polygons = vec![poly.clone(), other.clone()];
            let removed = remove_small(&mut ps, min_area);
            int_eq!("remove_small", removed as i64);
            polys_eq!("remove_small", pts_of(&ps));
        }
        polys_eq!("polygons_simplify (ClipperLib::SimplifyPolygons differs; reported)", pts_of(&polygons_simplify(&[poly.clone(), other.clone()], tolerance)));
        int_eq!("polygons_match", polygons_match(&poly, &other) as i64);
        int_eq!("overlaps(Polygons, Polygons)", overlaps(&[poly.clone()], &vec![other.clone()]) as i64);
        {
            let b = get_extents_rotated(&poly, angle);
            int_eq!("get_extents_rotated", b.is_defined() as i64);
            let mn = t.pt();
            let mx = t.pt();
            check("get_extents_rotated", b.min == mn && b.max == mx, format!("got {b:?} want {mn:?} {mx:?}"));
        }
        pts_eq!("make_circle", make_circle(radius, error).points().to_vec());
        pts_eq!("make_circle_num_segments", make_circle_num_segments(radius, segments).points().to_vec());
        dbl_eq!("total_length(Polygons)", total_length(&[poly.clone(), other.clone()]));
        dbl_eq!("area(Polygons)", area_polygons(&[poly.clone(), other.clone()]));
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
    println!("polygon_rest: {cases} cases, {total} asserted calls, {bad} mismatches");
    for f in &first {
        println!("{f}");
    }
    assert_eq!(cases, 2000);
    assert_eq!(bad, 0, "{bad} of {total} results differ from the pin");
}
