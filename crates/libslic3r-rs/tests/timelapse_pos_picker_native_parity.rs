//! `pick_pos_internal` against GCode/TimelapsePosPicker.cpp at the pin.
//!
//! `tests/data/timelapse_pos_picker/vectors.txt.gz` is the output of `gen.cpp` beside it, which
//! links the engine's libslic3r and calls the function on generated safe areas, collision areas,
//! positions and farthest points (the header comment there describes the lines). A third of the
//! cases lie on a 1 mm grid and a third on an 8 mm grid, so that candidates of equal penalty meet in
//! the heap of five.

use slicer::gcode::timelapse_pos_picker::pick_pos_internal;
use slicer::geometry::{ExPolygon, ExPolygons, Point, Polygon};
use std::io::Read;

fn read_vectors() -> String {
    let path = format!("{}/tests/data/timelapse_pos_picker/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    text
}

#[test]
fn pick_pos_internal_matches_the_pin() {
    let text = read_vectors();
    let (mut cases, mut bad, mut picked) = (0, 0, 0);
    for line in text.lines() {
        let (lhs, want) = line.split_once(" -> ").expect("arrow");
        let mut it = lhs.split_whitespace();
        assert_eq!(it.next(), Some("CASE"));
        let mut num = || -> i64 { it.next().expect("number").parse().expect("integer") };
        let curr = Point::new(num(), num());
        let detect = num() != 0;
        let has_farthest = num() != 0;
        let farthest_xy = Point::new(num(), num());
        let farthest = if has_farthest { Some(farthest_xy) } else { None };
        let mut areas = |num: &mut dyn FnMut() -> i64| -> ExPolygons {
            let n = num() as usize;
            let mut out = ExPolygons::new();
            for _ in 0..n {
                let mut ring = |num: &mut dyn FnMut() -> i64| -> Polygon {
                    let k = num() as usize;
                    Polygon::from_points((0..k).map(|_| Point::new(num(), num())).collect())
                };
                let mut e = ExPolygon::new(ring(num));
                let holes = num() as usize;
                for _ in 0..holes {
                    e.holes.push(ring(num));
                }
                out.push(e);
            }
            out
        };
        let safe = areas(&mut num);
        let coll = areas(&mut num);
        let got = pick_pos_internal(&curr, &safe, &coll, detect, &farthest);
        let got = format!("{} {}", got.x, got.y);
        cases += 1;
        if got != "0 0" {
            picked += 1;
        }
        if got != want {
            bad += 1;
            if bad <= 5 {
                eprintln!("case {cases}: got {got}, want {want}\n  {lhs}");
            }
        }
    }
    assert_eq!(bad, 0, "{bad} of {cases} cases differ");
    assert_eq!(cases, 2000);
    assert!(picked > 1300, "{picked}");
}
