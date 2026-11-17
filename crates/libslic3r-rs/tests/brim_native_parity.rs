//! `slicer::brim` (the port of libslic3r/Brim.cpp) against the C++ at the pin: the three overloads
//! of `compSecondMoment`, the second moment of area the automatic brim width is computed from.
//!
//! `tests/data/brim/vectors.txt` is the output of `gen.cpp` beside it, which links the pin's
//! Brim.cpp and runs the three overloads over 42 polygons (counter-clockwise, clockwise, of two
//! points), 30 expolygons with up to three holes and 20 groups of them; the header of gen.cpp
//! describes the lines. The same calls run here and have to give the same doubles, bit for bit,
//! with the faithful arm of the gate `BRIM_SECOND_MOMENT_PIN` (the default).
//!
//! The rest of Brim.cpp works on a sliced Print and is not driven here.

use slicer::brim::{
    comp_second_moment_of_expolygon, comp_second_moment_of_expolygons, comp_second_moment_of_polygon, ExPolyProp,
};
use slicer::geometry::{ExPolygon, Point, PointF, Polygon};
use std::collections::BTreeMap;

fn hd(v: f64) -> String {
    format!("{:016x}", v.to_bits())
}

fn poly(tok: &[&str], k: &mut usize) -> Polygon {
    let n: usize = tok[*k].parse().unwrap();
    *k += 1;
    let mut p = Polygon::default();
    for _ in 0..n {
        p.points.push(Point::new(tok[*k].parse().unwrap(), tok[*k + 1].parse().unwrap()));
        *k += 2;
    }
    p
}

fn expoly(tok: &[&str], k: &mut usize) -> ExPolygon {
    let holes: usize = tok[*k].parse().unwrap();
    *k += 1;
    let mut e = ExPolygon::default();
    e.contour = poly(tok, k);
    for _ in 0..holes {
        e.holes.push(poly(tok, k));
    }
    e
}

#[test]
fn comp_second_moment_matches_the_pin() {
    let path = format!("{}/tests/data/brim/vectors.txt", env!("CARGO_MANIFEST_DIR"));
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut by_op: BTreeMap<String, (usize, usize)> = BTreeMap::new();
    for line in text.lines() {
        let (lhs, want) = line.split_once(" -> ").expect("arrow");
        let a: Vec<&str> = lhs.split(' ').collect();
        let mut k = 1usize;
        let got = match a[0] {
            "poly" => {
                let p = poly(&a, &mut k);
                let mut sm = PointF::new(1., 2.);
                let ok = comp_second_moment_of_polygon::comp_second_moment(&p, &mut sm);
                format!("{} {} {}", ok as i32, hd(sm.x()), hd(sm.y()))
            }
            "expoly" => {
                let e = expoly(&a, &mut k);
                let mut prop = ExPolyProp::default();
                let ok = comp_second_moment_of_expolygon::comp_second_moment(&e, &mut prop);
                format!(
                    "{} {} {} {} {} {}",
                    ok as i32,
                    hd(prop.aera),
                    hd(prop.centroid.x()),
                    hd(prop.centroid.y()),
                    hd(prop.second_moment_of_area_respect_to_centroid.x()),
                    hd(prop.second_moment_of_area_respect_to_centroid.y())
                )
            }
            "expolys" => {
                let n: usize = a[k].parse().unwrap();
                k += 1;
                let group: Vec<ExPolygon> = (0..n).map(|_| expoly(&a, &mut k)).collect();
                let (mut ix, mut iy) = (-1.0, -2.0);
                let ok = comp_second_moment_of_expolygons::comp_second_moment(&group, &mut ix, &mut iy);
                format!("{} {} {}", ok as i32, hd(ix), hd(iy))
            }
            other => panic!("unknown line {other}"),
        };
        assert_eq!(k, a.len(), "{}", a[0]);
        let e = by_op.entry(a[0].to_string()).or_insert((0, 0));
        e.0 += 1;
        if got != want {
            e.1 += 1;
            let head: String = lhs.chars().take(70).collect();
            eprintln!("{head}: rust {got} c++ {want}");
        }
    }
    println!("lines by overload (compared, differing): {by_op:?}");
    assert!(by_op.values().all(|v| v.1 == 0), "lines differ from the C++: {by_op:?}");
}
