//! `is_inside_corner`, `point_is_left_of_line` and `get_angle_left` against
//! Arachne/utils/linearAlg2D.hpp at the pin.
//!
//! `tests/data/linear_alg2d/vectors.txt.gz` is the output of `gen.cpp` beside it, which includes
//! the header and calls the three inline functions on generated points (the header comment there
//! says how, and describes the lines): coordinates from a few units to 100 mm, with the corner
//! cases (straight, pointy, a degenerate leg, axis-aligned legs, the query on a leg or at the
//! corner). The results are compared exactly, the angle bit for bit.

use slicer::arachne::utils::linear_alg2d::{get_angle_left, is_inside_corner, point_is_left_of_line};
use slicer::geometry::Point;
use std::io::Read;

fn read_vectors() -> String {
    let path = format!("{}/tests/data/linear_alg2d/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    text
}

fn points(f: &[&str]) -> Vec<Point> {
    f.chunks(2)
        .map(|c| Point::new(c[0].parse().expect("x"), c[1].parse().expect("y")))
        .collect()
}

#[test]
fn linear_alg2d_matches_the_pin() {
    let text = read_vectors();
    let (mut counts, mut bad) = ([0usize; 3], 0);
    for line in text.lines() {
        let (lhs, want) = line.split_once(" -> ").expect("arrow");
        let f: Vec<&str> = lhs.split_whitespace().collect();
        let p = points(&f[1..]);
        let got = match f[0] {
            "IC" => {
                counts[0] += 1;
                (is_inside_corner(p[0], p[1], p[2], p[3]) as i32).to_string()
            }
            "PL" => {
                counts[1] += 1;
                point_is_left_of_line(p[0], p[1], p[2]).to_string()
            }
            "AL" => {
                counts[2] += 1;
                format!("{:08x}", get_angle_left(p[0], p[1], p[2]).to_bits())
            }
            tag => panic!("unknown tag {tag}"),
        };
        if got != want {
            bad += 1;
            if bad <= 10 {
                eprintln!("{lhs}: want {want} got {got}");
            }
        }
    }
    assert_eq!(counts, [3000, 3000, 3000]);
    assert_eq!(bad, 0);
}
