//! `ray_circle_intersections_r2_lv2_c` and `ray_circle_intersections` against Geometry/Circle.hpp
//! at the pin.
//!
//! `tests/data/geometry_circle/vectors.txt.gz` is the output of `gen.cpp` beside it, which
//! instantiates the header templates for double as slicer_cli's libslic3r does (the header comment
//! there says how, and describes the lines): half the cases built as FillBase.cpp:708 calls the
//! function, half random, some refused by the epsilon of ray_circle_intersections. Every double is
//! compared bit for bit, with the count.

use slicer::geometry::{ray_circle_intersections, ray_circle_intersections_r2_lv2_c, Vec2d};
use std::io::Read;

fn read_vectors() -> String {
    let path = format!("{}/tests/data/geometry_circle/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    text
}

fn d(hex: &str) -> f64 {
    f64::from_bits(u64::from_str_radix(hex, 16).expect("double bits"))
}

fn fields(n: i32, out: &(Vec2d, Vec2d)) -> String {
    format!(
        "{} {:016x} {:016x} {:016x} {:016x}",
        n,
        out.0.x.to_bits(),
        out.0.y.to_bits(),
        out.1.x.to_bits(),
        out.1.y.to_bits()
    )
}

#[test]
fn ray_circle_intersections_match_the_pin() {
    let text = read_vectors();
    let (mut r2_cases, mut rc_cases, mut hits, mut bad) = (0, 0, 0, 0);
    for line in text.lines() {
        let (lhs, want) = line.split_once(" -> ").expect("arrow");
        let a: Vec<&str> = lhs.split_whitespace().collect();
        let mut out = (Vec2d::new(0.0, 0.0), Vec2d::new(0.0, 0.0));
        let n = match a[0] {
            "R2" => {
                r2_cases += 1;
                ray_circle_intersections_r2_lv2_c(d(a[1]), d(a[2]), d(a[3]), d(a[4]), d(a[5]), &mut out)
            }
            "RC" => {
                rc_cases += 1;
                ray_circle_intersections(d(a[1]), d(a[2]), d(a[3]), d(a[4]), &mut out)
            }
            tag => panic!("unknown tag {tag}"),
        };
        hits += usize::from(n > 0);
        let got = fields(n, &out);
        if got != want {
            bad += 1;
            if bad <= 10 {
                eprintln!("{lhs}\n   want {want}\n   got  {got}");
            }
        }
    }
    assert_eq!((r2_cases, rc_cases), (1500, 700));
    assert!(hits > 1000, "{hits}");
    assert_eq!(bad, 0);
}
