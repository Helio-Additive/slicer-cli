//! SeamPlacer's `PerimeterDistancer` (SeamPlacer.cpp:886-921) against the pin.
//!
//! `tests/data/seam_perimeter_distancer/gen.cpp` carries the pin's class body (over `Linef`,
//! `AABBTreeIndirect::Tree<2, double>`) and the pin's AABBTreeLines; it writes random
//! outlines (islands with holes) and 200 f32 query points per outline with the signed
//! distance as raw float bits. The port's distancer must give the same bits
//! (SEAM_DISTANCER_LINEF, on by default: a query over the unscaled lines in double).
use slicer::gcode::seam_placer::{PerimeterDistancer, Vec2f};
use slicer::geometry::{ExPolygon, Point, Polygon};

#[test]
fn perimeter_distancer_matches_the_pin() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/seam_perimeter_distancer/vectors.txt");
    let text = std::fs::read_to_string(path).expect("vectors.txt");
    let mut outline: Vec<ExPolygon> = Vec::new();
    let mut distancer: Option<PerimeterDistancer> = None;
    let (mut cases, mut bad, mut scenarios) = (0usize, 0usize, 0usize);
    for line in text.lines() {
        let t: Vec<&str> = line.split_whitespace().collect();
        match t.first().copied() {
            Some("S") => {
                outline.clear();
                distancer = None;
                scenarios += 1;
            }
            Some("E") => {
                let e: usize = t[1].parse().unwrap();
                let h: i64 = t[2].parse().unwrap();
                let n: usize = t[3].parse().unwrap();
                let pts: Vec<Point> = (0..n)
                    .map(|k| Point::new(t[4 + 2 * k].parse().unwrap(), t[5 + 2 * k].parse().unwrap()))
                    .collect();
                let pg = Polygon::from_points(pts);
                if h < 0 {
                    assert_eq!(e, outline.len());
                    outline.push(ExPolygon::new(pg));
                } else {
                    outline[e].holes.push(pg);
                }
                distancer = None;
            }
            Some("Q") => {
                if distancer.is_none() {
                    distancer = Some(PerimeterDistancer::new(&outline));
                }
                let x = f32::from_bits(u32::from_str_radix(t[1], 16).unwrap());
                let y = f32::from_bits(u32::from_str_radix(t[2], 16).unwrap());
                let want = u32::from_str_radix(t[3], 16).unwrap();
                let got = distancer.as_ref().unwrap().distance_from_perimeter(&Vec2f::new(x, y)).to_bits();
                cases += 1;
                if got != want {
                    if bad < 5 {
                        eprintln!("mismatch at ({x}, {y}): pin {} port {}", f32::from_bits(want), f32::from_bits(got));
                    }
                    bad += 1;
                }
            }
            _ => {}
        }
    }
    assert!(scenarios >= 24 && cases >= 4800, "vectors too short: {scenarios} outlines, {cases} queries");
    assert_eq!(bad, 0, "{bad} of {cases} distances differ from the pin");
}
