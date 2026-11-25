//! `Point::projection_onto(const Line&)` (Point.cpp:157-180), `Point::projection_onto(const MultiPoint&)`
//! (Point.cpp:141-155, on a Polygon and on a Polyline of the same points) and
//! `get_extents<false/true>(const Points&)` / `get_extents<false/true>(const VecOfPoints&)` (Point.cpp:251-272)
//! against the pin. Vectors from `tests/data/point_projection/gen.cpp` (4,000 cases, gzip -9n).

use slicer::geometry::{get_extents_points as get_extents, get_extents_vec_of_points, PointProjectionOntoMultiPoint};
use slicer::geometry::{Line, Point, Polygon, Polyline};
use std::io::Read;

#[test]
fn point_projection_and_extents_match_the_pin() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/point_projection/vectors.txt.gz");
    let mut text = String::new();
    flate2::read::GzDecoder::new(std::fs::File::open(path).unwrap()).read_to_string(&mut text).unwrap();
    let lines: Vec<Vec<i64>> = text
        .lines()
        .map(|l| {
            let mut it = l.split_whitespace();
            let tag = it.next().unwrap();
            let code = match tag {
                "PL" => 0,
                "Q" => 1,
                "PM" => 2,
                "BB" => 3,
                "W" => 4,
                "WE" => 5,
                _ => panic!("bad line {l}"),
            };
            std::iter::once(code).chain(it.map(|s| s.parse::<i64>().unwrap())).collect()
        })
        .collect();
    let pts = |v: &[i64]| -> Vec<Point> { (0..v[1] as usize).map(|k| Point::new(v[2 + 2 * k], v[3 + 2 * k])).collect() };
    let bbline = |b: &slicer::geometry::BoundingBox| -> Vec<i64> {
        vec![3, b.is_defined() as i64, b.min.x, b.min.y, b.max.x, b.max.y]
    };
    let mut counts = [0usize; 5];
    let mut bad = [0usize; 5];
    let mut i = 0;
    let mut cases = 0;
    while i < lines.len() {
        // PL
        let v = &lines[i];
        assert_eq!(v[0], 0);
        let p = Point::new(v[1], v[2]);
        let l = Line::new(Point::new(v[3], v[4]), Point::new(v[5], v[6]));
        let r = p.projection_onto(&l);
        counts[0] += 1;
        if (r.x, r.y) != (v[7], v[8]) {
            bad[0] += 1;
        }
        // Q, PM
        let q = pts(&lines[i + 1]);
        let pm = &lines[i + 2];
        let p = Point::new(pm[1], pm[2]);
        let pg = Polygon::from_points(q.clone());
        let py = Polyline::from_points(q.clone());
        let a = PointProjectionOntoMultiPoint::<Polygon>::projection_onto(&p, &pg);
        let b = PointProjectionOntoMultiPoint::<Polyline>::projection_onto(&p, &py);
        counts[1] += 1;
        if (a.x, a.y, b.x, b.y) != (pm[3], pm[4], pm[5], pm[6]) {
            bad[1] += 1;
        }
        // BB x2 for Points
        counts[2] += 1;
        if bbline(&get_extents(&q, false)) != lines[i + 3] || bbline(&get_extents(&q, true)) != lines[i + 4] {
            bad[2] += 1;
        }
        // W lines, WE, BB x2 for VecOfPoints
        let mut j = i + 5;
        let mut vp: Vec<Vec<Point>> = Vec::new();
        while lines[j][0] == 4 {
            vp.push(pts(&lines[j]));
            j += 1;
        }
        assert_eq!(lines[j][0], 5);
        assert_eq!(lines[j][1] as usize, vp.len());
        counts[3] += 1;
        if bbline(&get_extents_vec_of_points(&vp, false)) != lines[j + 1] {
            bad[3] += 1;
        }
        counts[4] += 1;
        if bbline(&get_extents_vec_of_points(&vp, true)) != lines[j + 2] {
            bad[4] += 1;
        }
        i = j + 3;
        cases += 1;
    }
    eprintln!(
        "{cases} cases: projection_onto(Line) {}/{}, projection_onto(MultiPoint) {}/{}, get_extents(Points) {}/{}, get_extents<false>(VecOfPoints) {}/{}, get_extents<true>(VecOfPoints) {}/{} mismatches",
        bad[0], counts[0], bad[1], counts[1], bad[2], counts[2], bad[3], counts[3], bad[4], counts[4]
    );
    assert_eq!(cases, 4000);
    assert_eq!(bad, [0; 5]);
}
