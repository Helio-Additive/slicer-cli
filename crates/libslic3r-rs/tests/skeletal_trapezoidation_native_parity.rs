//! `SkeletalTrapezoidation::interpolate` (both overloads, SkeletalTrapezoidation.cpp:1709-1771) against
//! the pin.
//!
//! `tests/data/skeletal_trapezoidation/vectors.txt.gz` is the output of `gen.cpp` beside it, which
//! links the engine's libslic3r and calls the pin's own member functions on generated beadings (the
//! header comment there describes the lines). Half of the right-hand beadings share widths and
//! locations with the left one, as the beadings of neighbouring nodes do. The `i3` lines carry a flag
//! telling whether a fused and an unfused evaluation of the two sums truncate differently; none of
//! the generated cases (nor any call on the Majora and Benchy fixtures, probe ST_INTERP_DUMP) is such a
//! case, so these vectors do not tell the contraction apart (gate ARACHNE_INTERPOLATE_FMA).

use slicer::arachne::beading_strategy::beading_strategy::Beading;
use slicer::arachne::beading_strategy::BeadingStrategyFactory;
use slicer::arachne::skeletal_trapezoidation::{Interpolate, SkeletalTrapezoidation};
use std::io::Read;

fn vectors() -> String {
    let path = std::env::var("SKELETAL_TRAPEZOIDATION_PARITY_VECTORS").unwrap_or_else(|_| {
        format!("{}/tests/data/skeletal_trapezoidation/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"))
    });
    let file = std::fs::File::open(&path).expect("vectors");
    let mut text = String::new();
    if path.ends_with(".gz") {
        flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    } else {
        std::io::BufReader::new(file).read_to_string(&mut text).expect("text");
    }
    text
}

fn parse_beading(t: &mut std::slice::Iter<'_, &str>) -> Beading {
    let mut num = || t.next().expect("token").parse::<i64>().expect("integer");
    let total_thickness = num();
    let left_over = num();
    let n = num() as usize;
    let bead_widths: Vec<i64> = (0..n).map(|_| num()).collect();
    let toolpath_locations: Vec<i64> = (0..n).map(|_| num()).collect();
    Beading { total_thickness, bead_widths, toolpath_locations, left_over }
}

fn same(a: &Beading, b: &Beading) -> bool {
    a.total_thickness == b.total_thickness
        && a.left_over == b.left_over
        && a.bead_widths == b.bead_widths
        && a.toolpath_locations == b.toolpath_locations
}

fn parse_hex_f64(s: &str) -> f64 {
    // printf("%a") of a double in [0, 1]: 0x0p+0, 0x1p+0, 0x1.<hex>p<exp>.
    let s = s.strip_prefix("0x").expect("hex float");
    let (mant, exp) = s.split_once('p').expect("exponent");
    let (int, frac) = mant.split_once('.').unwrap_or((mant, ""));
    let mut m = u64::from_str_radix(int, 16).expect("mantissa") as f64;
    let mut scale = 1.0 / 16.0;
    for c in frac.chars() {
        m += c.to_digit(16).expect("hex digit") as f64 * scale;
        scale /= 16.0;
    }
    m * 2f64.powi(exp.parse::<i32>().expect("exp"))
}

#[test]
fn interpolate_native_parity() {
    let strategy = BeadingStrategyFactory::make_strategy(
        42_000, 42_000, 40_000, std::f64::consts::PI / 18.0, true, 35_700, 10_500, 0.7, 0.85, 4, 0, 1, 0.5,
    );
    let st = SkeletalTrapezoidation::new(&*strategy, strategy.get_transitioning_angle(), 80_000, 10_000_000, 2_500, 40_000, false, Vec::new());
    let text = vectors();
    let (mut i3, mut i4, mut sensitive, mut differ) = (0usize, 0usize, 0usize, 0usize);
    for line in text.lines() {
        let tokens: Vec<&str> = line.split_whitespace().collect();
        let mut t = tokens.iter();
        let kind = *t.next().expect("kind");
        let ratio = parse_hex_f64(t.next().expect("ratio"));
        let (expected, got) = match kind {
            "i3" => {
                sensitive += (*t.next().expect("flag") == "1") as usize;
                assert_eq!(*t.next().unwrap(), "L");
                let left = parse_beading(&mut t);
                assert_eq!(*t.next().unwrap(), "R");
                let right = parse_beading(&mut t);
                assert_eq!(*t.next().unwrap(), "=>");
                i3 += 1;
                (parse_beading(&mut t), st.interpolate((&left, ratio, &right)))
            }
            "i4" => {
                let sr = t.next().expect("switching radius").parse::<i64>().expect("integer");
                assert_eq!(*t.next().unwrap(), "L");
                let left = parse_beading(&mut t);
                assert_eq!(*t.next().unwrap(), "R");
                let right = parse_beading(&mut t);
                assert_eq!(*t.next().unwrap(), "=>");
                i4 += 1;
                (parse_beading(&mut t), st.interpolate((&left, ratio, &right, sr)))
            }
            other => panic!("unknown line kind {other}"),
        };
        if !same(&expected, &got) {
            if differ < 5 {
                eprintln!("differs: {line}\n  port: {:?} {:?} {:?}", got.total_thickness, got.bead_widths, got.toolpath_locations);
            }
            differ += 1;
        }
    }
    eprintln!("interpolate: {i3} three-argument cases ({sensitive} fma-sensitive), {i4} four-argument cases, {differ} differ");
    assert!(i3 > 1000 && i4 > 1000, "too few cases: {i3} / {i4}");
    assert_eq!(differ, 0);
}
