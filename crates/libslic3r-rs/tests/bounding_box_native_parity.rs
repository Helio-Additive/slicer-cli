//! `slicer::geometry::BoundingBox` (the port of the 2D integer box of libslic3r/BoundingBox.hpp and
//! BoundingBox.cpp) against the C++ at the pin.
//!
//! `tests/data/bounding_box/vectors.txt.gz` is the output of `gen.cpp` beside it; the header of gen.cpp
//! describes the line. Every case carries its inputs and the pin's results; the same calls run here and
//! every result has to be the pin's (integers equal, doubles bit-equal).
//!
//! `scale` / `scaled` are compared on the arm of the opt-in gate `BOUNDING_BOX_SCALE_PIN`, which the test
//! switches on. `size`, `contains(point)` and `contains(box)` read `min`/`max` without asking for a defined
//! box, as the pin (BoundingBox.cpp:176-180, BoundingBox.hpp:51-57), under the gate `BOUNDING_BOX_DEFINED_PIN`
//! (R931, on by default): asserted on defined and undefined boxes, the undefined ones counted under their own
//! names "(box not defined)".

use slicer::geometry::{BoundingBox, Point};
use std::collections::BTreeMap;
use std::io::Read;

fn read_vectors() -> String {
    let path = std::env::var("BOUNDING_BOX_VECTORS")
        .unwrap_or_else(|_| format!("{}/tests/data/bounding_box/vectors.txt.gz", env!("CARGO_MANIFEST_DIR")));
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

// `BoundingBox(const Points &points)`: `construct<false>` on an undefined box (BoundingBox.hpp:25-35, 88-102),
// which `BoundingBox::from_points` is under the gate `BOUNDING_BOX_FROM_POINTS_PIN` (R931, on by default);
// `construct` itself is compared as well.
fn from_points(points: &[Point]) -> BoundingBox {
    let mut bb = BoundingBox::new();
    BoundingBox::construct(&mut bb, points, false);
    let via = BoundingBox::from_points(points);
    assert!(
        via.min == bb.min && via.max == bb.max && via.is_defined() == bb.is_defined(),
        "BoundingBox::from_points differs from construct<false> on {points:?}"
    );
    bb
}

#[test]
fn bounding_box_matches_the_pin() {
    // The gate is read once per process, on first use, and this is the only test of this binary.
    #[allow(unused_unsafe)]
    unsafe {
        std::env::set_var("BOUNDING_BOX_SCALE_PIN", "1");
    }
    let text = read_vectors();
    let mut calls: BTreeMap<&'static str, (usize, usize)> = BTreeMap::new();
    let mut first: Vec<String> = Vec::new();
    let mut cases = 0usize;
    for (ln, line) in text.lines().enumerate() {
        let mut t = Tok { t: line.split(' ').filter(|x| !x.is_empty()).collect(), k: 0 };
        assert_eq!(t.s(), "c");
        cases += 1;
        let n = t.i() as usize;
        let pts: Vec<Point> = (0..n).map(|_| t.pt()).collect();
        let m = t.i() as usize;
        let pts2: Vec<Point> = (0..m).map(|_| t.pt()).collect();
        let q = t.pt();
        let angle = t.d();
        let center = t.pt();
        let factor = t.d();
        let delta = t.d();
        let cell = t.i();
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
        macro_rules! box_eq {
            ($name:expr, $got:expr) => {{
                let b: BoundingBox = $got;
                int_eq!($name, b.is_defined() as i64);
                pt_eq!($name, b.min);
                pt_eq!($name, b.max);
            }};
        }

        let bb = from_points(&pts);
        let bb2 = from_points(&pts2);
        box_eq!("BoundingBox(points)", bb);
        box_eq!("BoundingBox(points)", bb2);
        if bb.is_defined() {
            pt_eq!("size", bb.size());
        } else {
            pt_eq!("size (box not defined)", bb.size());
        }
        dbl_eq!("radius", bb.radius());
        dbl_eq!("area", bb.area() as f64);
        pt_eq!("center", bb.center());
        if bb.is_defined() {
            int_eq!("contains(point)", bb.contains_point(&q) as i64);
        } else {
            int_eq!("contains(point) (box not defined)", bb.contains_point(&q) as i64);
        }
        if bb.is_defined() && bb2.is_defined() {
            int_eq!("contains(box)", bb.contains(&bb2) as i64);
        } else {
            int_eq!("contains(box) (box not defined)", bb.contains(&bb2) as i64);
        }
        int_eq!("overlap", bb.overlap(&bb2) as i64);
        {
            let mut b = bb;
            b.merge_point(q);
            box_eq!("merge(point)", b);
        }
        {
            let mut b = bb;
            b.merge(&bb2);
            box_eq!("merge(box)", b);
        }
        {
            let mut b = bb;
            b.offset(delta);
            pt_eq!("offset", b.min);
            pt_eq!("offset", b.max);
        }
        {
            let b = bb.inflated(delta);
            pt_eq!("inflated", b.min);
            pt_eq!("inflated", b.max);
        }
        {
            let mut b = bb;
            b.scale(factor);
            pt_eq!("scale", b.min);
            pt_eq!("scale", b.max);
        }
        {
            let b = bb.scaled(factor);
            pt_eq!("scaled", b.min);
            pt_eq!("scaled", b.max);
        }
        box_eq!("rotated(angle)", bb.rotated(angle));
        box_eq!("rotated(angle, center)", bb.rotated_center_overload(angle, &center));
        {
            let poly = bb.polygon();
            let got: Vec<Point> = poly.points().to_vec();
            let want: Vec<Point> = (0..4).map(|_| t.pt()).collect();
            check("polygon", got == want, format!("got {got:?} want {want:?}"));
        }
        {
            let mut b = bb;
            b.align_to_grid(cell);
            pt_eq!("align_to_grid", b.min);
            pt_eq!("align_to_grid", b.max);
        }
        {
            let got: Vec<Point> = bb.corners().to_vec();
            let want: Vec<Point> = (0..4).map(|_| t.pt()).collect();
            check("operator[]", got == want, format!("got {got:?} want {want:?}"));
        }
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
    println!("bounding_box: {cases} cases, {total} asserted calls, {bad} mismatches");
    for f in &first {
        println!("{f}");
    }
    assert_eq!(cases, 4000);
    assert_eq!(bad, 0, "{bad} of {total} results differ from the pin");
}
