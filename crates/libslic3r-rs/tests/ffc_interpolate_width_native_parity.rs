//! Native parity of `fill_floating_concentric::interpolate_width` with the pin's
//! `interpolate_width` (override layer `libslic3r/bambustudio/libslic3r/Fill/FillFloatingConcentric.cpp:220-258`).
//! Vectors: `tests/data/ffc_interpolate_width/gen.cpp` (LCG inputs, 500 cases); the result must be
//! bit-equal (FFC_INTERP_WIDTH_FMA: the pin's `(1 - t) * width_prev + t * width_next` is contracted
//! to one fma).
use slicer::fill::fill_floating_concentric::interpolate_width;
use slicer::geometry::{Point, ThickPolyline};

fn hex(s: &str) -> f64 {
    f64::from_bits(u64::from_str_radix(s, 16).expect("hex double"))
}

#[test]
fn interpolate_width_matches_the_pin() {
    let text = std::fs::read_to_string(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/data/ffc_interpolate_width/vectors.txt"
    ))
    .expect("vectors");
    let (mut cases, mut differ) = (0usize, 0usize);
    for line in text.lines() {
        let mut t = line.split_whitespace();
        assert_eq!(t.next(), Some("C"));
        let mut int = || t.next().unwrap().parse::<i64>().unwrap();
        let (n, m, sir, dw, idx) = (int() as usize, int() as usize, int() as i32, int() as i32, int() as usize);
        let mut points = Vec::with_capacity(n);
        for _ in 0..n {
            let (x, y) = (int(), int());
            points.push(Point::new(x, y));
        }
        let rest: Vec<&str> = t.collect();
        let mut k = 0usize;
        let mut widths = Vec::with_capacity(2 * n - 2);
        for _ in 0..2 * n - 2 {
            widths.push(hex(rest[k]));
            k += 1;
        }
        let mut path = Vec::with_capacity(m);
        for _ in 0..m {
            let x: i64 = rest[k].parse().unwrap();
            let y: i64 = rest[k + 1].parse().unwrap();
            let z: i64 = rest[k + 2].parse().unwrap();
            path.push((x, y, z));
            k += 3;
        }
        let want = hex(rest[k]);
        // The C++ width layout (2 per segment) in `widths`, as fill_floating_concentric.rs reads it.
        let mut line = ThickPolyline::new();
        line.points = points;
        line.widths = widths;
        let got = interpolate_width(&path, &line, sir, dw, idx);
        cases += 1;
        if got.to_bits() != want.to_bits() {
            differ += 1;
            if differ <= 5 {
                eprintln!("case {cases}: port {got:e} pin {want:e}");
            }
        }
    }
    eprintln!("interpolate_width: {cases} cases, {differ} differ");
    assert_eq!(cases, 500);
    assert_eq!(differ, 0);
}
