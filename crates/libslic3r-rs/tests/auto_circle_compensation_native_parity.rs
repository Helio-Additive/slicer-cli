//! `LayerRegion::auto_circle_compensation` (LayerRegion.cpp:17-18, 69-125) against the pin.
//!
//! `tests/data/auto_circle_compensation/vectors.txt.gz` is the output of `gen.cpp` beside it, which
//! links the engine's libslic3r and calls the pin's member on generated surfaces (approximate circles
//! with and without holes, rectangles, stars, rings of too few points) with random
//! AutoContourHolesCompensationParams for two filaments, a random wall filament and a random manual
//! offset (the header comment there describes the lines). Every output surface has to be equal:
//! contour and holes point by point, `counter_circle_compensation` and `holes_circle_compensation`.

use slicer::geometry::{ExPolygon, Point, Polygon};
use slicer::layer::LayerRegion;
use slicer::layer_region::AutoContourHolesCompensationParams;
use slicer::surface::{Surface, SurfaceType};
use slicer::surface_collection::SurfaceCollection;
use std::io::Read;

fn read_vectors() -> String {
    let path = format!("{}/tests/data/auto_circle_compensation/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    text
}

fn parse_polygon<'a>(it: &mut impl Iterator<Item = &'a str>) -> Polygon {
    let n: usize = it.next().unwrap().parse().unwrap();
    let mut pts = Vec::with_capacity(n);
    for _ in 0..n {
        let x: i64 = it.next().unwrap().parse().unwrap();
        let y: i64 = it.next().unwrap().parse().unwrap();
        pts.push(Point::new(x, y));
    }
    Polygon::from_points(pts)
}

/// One "s ..." line: the expolygon, and with `flags` the two compensation members.
fn parse_surface(line: &str, flags: bool) -> (ExPolygon, bool, Vec<i32>) {
    let mut it = line.split(' ');
    assert_eq!(it.next(), Some("s"));
    let contour = parse_polygon(&mut it);
    let nholes: usize = it.next().unwrap().parse().unwrap();
    let holes = (0..nholes).map(|_| parse_polygon(&mut it)).collect();
    let (mut counter, mut hole_idx) = (false, Vec::new());
    if flags {
        counter = it.next().unwrap() == "1";
        let k: usize = it.next().unwrap().parse().unwrap();
        hole_idx = (0..k).map(|_| it.next().unwrap().parse().unwrap()).collect();
    }
    (ExPolygon::with_holes(contour, holes), counter, hole_idx)
}

fn hex_f64(s: &str) -> f64 {
    f64::from_bits(u64::from_str_radix(s, 16).unwrap())
}

#[test]
fn auto_circle_compensation_matches_the_pin() {
    let text = read_vectors();
    let mut lines = text.lines().peekable();
    let (mut cases, mut surfaces, mut bad, mut compensated) = (0usize, 0usize, 0usize, 0usize);
    while let Some(head) = lines.next() {
        // case <id> <wall_filament> <manual_offset float>
        let h: Vec<&str> = head.split(' ').collect();
        assert_eq!(h[0], "case");
        let wall_filament: i64 = h[2].parse().unwrap();
        let manual_offset = f32::from_bits(u32::from_str_radix(h[3], 16).unwrap());
        // params: 12 members x 2 filaments
        let p: Vec<f64> = lines.next().unwrap().split(' ').skip(1).map(hex_f64).collect();
        assert_eq!(p.len(), 24);
        let v = |k: usize| vec![p[2 * k], p[2 * k + 1]];
        let params = AutoContourHolesCompensationParams {
            counter_speed_coef: v(0),
            counter_diameter_coef: v(1),
            counter_compensate_coef: v(2),
            hole_speed_coef: v(3),
            hole_diameter_coef: v(4),
            hole_compensate_coef: v(5),
            counter_limit_min_value: v(6),
            counter_limit_max_value: v(7),
            hole_limit_min_value: v(8),
            hole_limit_max_value: v(9),
            circle_compensation_speed: v(10),
            diameter_limit: v(11),
        };
        let n: usize = lines.next().unwrap().strip_prefix("in ").unwrap().parse().unwrap();
        let mut slices = SurfaceCollection::default();
        for _ in 0..n {
            let (ex, _, _) = parse_surface(lines.next().unwrap(), false);
            slices.surfaces.push(Surface::new(SurfaceType::Internal, ex));
        }
        assert_eq!(lines.next(), Some("out"));
        LayerRegion::auto_circle_compensation_for_wall_filament(wall_filament, &mut slices, &params, manual_offset);
        for s in &slices.surfaces {
            let (ex, counter, hole_idx) = parse_surface(lines.next().unwrap(), true);
            surfaces += 1;
            if counter || !hole_idx.is_empty() {
                compensated += 1;
            }
            let equal = s.expolygon.contour.points == ex.contour.points
                && s.expolygon.holes.len() == ex.holes.len()
                && s.expolygon.holes.iter().zip(&ex.holes).all(|(a, b)| a.points == b.points)
                && s.counter_circle_compensation == counter
                && s.holes_circle_compensation == hole_idx;
            if !equal {
                if bad < 5 {
                    eprintln!(
                        "case {} differs: counter {} vs {}, holes {:?} vs {:?}, contour {} vs {} points",
                        h[1],
                        s.counter_circle_compensation,
                        counter,
                        s.holes_circle_compensation,
                        hole_idx,
                        s.expolygon.contour.points.len(),
                        ex.contour.points.len()
                    );
                }
                bad += 1;
            }
        }
        cases += 1;
    }
    eprintln!("auto_circle_compensation: {cases} cases, {surfaces} surfaces ({compensated} with a flag set), {bad} differ");
    assert!(cases >= 300 && compensated > 0);
    assert_eq!(bad, 0);
}
