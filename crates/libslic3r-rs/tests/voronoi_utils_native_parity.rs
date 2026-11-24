//! `Geometry::VoronoiUtils::discretize_parabola` (VoronoiUtils.cpp:103-197) against the pin's own
//! function: vectors from `tests/data/voronoi_utils/gen.cpp` (the parabolic edge of Majora's layer 110
//! whose marking moved with the float transition angle in R945, and 1,500 random parabolic edges with
//! start and end on the parabola, step sizes 2,000 to 80,000 and float transitioning angles).
//! Every discretized point must be equal.

use slicer::geometry::voronoi_utils::discretize_parabola;
use slicer::geometry::Point;
use std::io::Read;

fn parse_hex_f64(s: &str) -> f64 {
    let s = s.trim();
    let (neg, s) = match s.strip_prefix('-') {
        Some(r) => (true, r),
        None => (false, s),
    };
    let s = s.strip_prefix("0x").expect("hex float");
    let (mant, exp) = s.split_once('p').expect("exponent");
    let (int, frac) = mant.split_once('.').unwrap_or((mant, ""));
    let mut v = u64::from_str_radix(int, 16).expect("hex digits") as f64;
    let mut scale = 1.0 / 16.0;
    for c in frac.chars() {
        v += c.to_digit(16).expect("hex digit") as f64 * scale;
        scale /= 16.0;
    }
    let e: i32 = exp.parse().expect("exponent digits");
    let v = v * 2f64.powi(e);
    if neg {
        -v
    } else {
        v
    }
}

fn point(x: &str, y: &str) -> Point {
    Point::new(x.parse().expect("x"), y.parse().expect("y"))
}

#[test]
fn discretize_parabola_native_parity() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/voronoi_utils/vectors.txt.gz");
    let file = std::fs::File::open(path).expect("vectors");
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    let (mut cases, mut differ) = (0usize, 0usize);
    for line in text.lines() {
        let t: Vec<&str> = line.split_whitespace().collect();
        assert_eq!(t[0], "dp");
        let p = point(t[1], t[2]);
        let a = point(t[3], t[4]);
        let b = point(t[5], t[6]);
        let start = point(t[7], t[8]);
        let end = point(t[9], t[10]);
        let step: i64 = t[11].parse().expect("step");
        let angle = parse_hex_f64(t[12]) as f32;
        assert_eq!(t[13], "=>");
        let n: usize = t[14].parse().expect("count");
        let expected: Vec<Point> = t[15..15 + n]
            .iter()
            .map(|s| {
                let (x, y) = s.split_once(',').expect("x,y");
                point(x, y)
            })
            .collect();
        let got = discretize_parabola(p, a, b, start, end, step, angle);
        cases += 1;
        if got != expected {
            if differ < 5 {
                eprintln!("differs: {line}\n  port: {got:?}");
            }
            differ += 1;
        }
    }
    eprintln!("discretize_parabola: {cases} cases, {differ} differ");
    assert!(cases > 1000);
    assert_eq!(differ, 0);
}
