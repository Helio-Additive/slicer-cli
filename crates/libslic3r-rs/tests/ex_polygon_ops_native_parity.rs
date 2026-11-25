//! Native parity for ExPolygon (R947): ExPolygon::overlaps (ExPolygon.cpp:158-173),
//! ExPolygon::contains(Line) / contains(Polyline) (76-90), get_extents_rotated(ExPolygon) (506-509),
//! has_duplicate_points(ExPolygon) (531-), to_unscaled_linesf (ExPolygon.hpp:180-198) on random
//! star-shaped ExPolygons with holes (a quarter snapped to a grid, every fifth with a repeated point).
//! Vectors: tests/data/ex_polygon/gen_ops.cpp (ops_vectors.txt.gz).

// R948: the functions are called under their C++ names (traits for the overloads); the older names
// (overlaps_expolys, contains_line, has_duplicate_points_expoly, remove_sticks_expoly, ...) delegate to them.
use slicer::ex_polygon::{
    expolygons_match, get_extents_rotated, get_extents_rotated_expoly, is_narrow_expolygon, projection_onto, to_unscaled_linesf,
    ExPolygonArea, ExPolygonContains, ExPolygonHasDuplicatePoints, ExPolygonsOverlaps,
};
use slicer::geometry::{ExPolygon, Line, Point, Polygon, Polyline};

fn hexf(s: &str) -> f64 {
    let (neg, s) = match s.strip_prefix('-') {
        Some(r) => (true, r),
        None => (false, s),
    };
    let s = s.strip_prefix("0x").expect("hex float");
    let (mant, exp) = s.split_once('p').expect("exponent");
    let exp: i32 = exp.parse().expect("exp");
    let (int_part, frac_part) = mant.split_once('.').unwrap_or((mant, ""));
    let digits = format!("{int_part}{frac_part}");
    let m = u64::from_str_radix(&digits, 16).expect("mantissa");
    let v = (m as f64) * 2f64.powi(exp - 4 * frac_part.len() as i32);
    if neg {
        -v
    } else {
        v
    }
}

fn vectors() -> String {
    match std::env::var("EX_POLYGON_OPS_VECTORS") {
        Ok(path) => std::fs::read_to_string(&path).expect("vectors"),
        Err(_) => {
            use std::io::Read;
            let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ex_polygon/ops_vectors.txt.gz");
            let mut s = String::new();
            flate2::read::GzDecoder::new(std::fs::File::open(path).expect("vectors.txt.gz"))
                .read_to_string(&mut s)
                .expect("gzip");
            s
        }
    }
}

fn read_points(line: &str, tag: &str) -> Vec<Point> {
    let mut w = line.split_whitespace();
    assert_eq!(w.next(), Some(tag));
    let n: usize = w.next().unwrap().parse().unwrap();
    (0..n)
        .map(|_| Point { x: w.next().unwrap().parse().unwrap(), y: w.next().unwrap().parse().unwrap() })
        .collect()
}

fn read_ex<'a>(it: &mut impl Iterator<Item = &'a str>) -> ExPolygon {
    let nh: usize = it.next().unwrap().split_whitespace().nth(1).unwrap().parse().unwrap();
    let mut ex = ExPolygon::new(Polygon::from_points(read_points(it.next().unwrap(), "c")));
    for _ in 0..nh {
        ex.holes.push(Polygon::from_points(read_points(it.next().unwrap(), "h")));
    }
    ex
}

fn same_exs(a: &[ExPolygon], b: &[ExPolygon]) -> bool {
    a.len() == b.len()
        && a.iter().zip(b).all(|(x, y)| {
            x.contour.points == y.contour.points && x.holes.len() == y.holes.len() && x.holes.iter().zip(&y.holes).all(|(p, q)| p.points == q.points)
        })
}

fn tail<'a>(it: &mut impl Iterator<Item = &'a str>, tag: &str) -> Vec<&'a str> {
    let w: Vec<&str> = it.next().unwrap().split_whitespace().collect();
    assert_eq!(w[0], tag);
    w[1..].to_vec()
}

#[test]
fn ex_polygon_ops_native_parity() {
    let text = vectors();
    let mut it = text.lines();
    let mut cases = 0usize;
    let (mut ov_ne, mut cl_ne, mut cp_ne, mut bb_ne, mut dup_ne, mut lf_ne) = (0usize, 0usize, 0usize, 0usize, 0usize, 0usize);
    let (mut ov_true, mut cl_true, mut cp_true, mut dup_true) = (0usize, 0usize, 0usize, 0usize);
    let mut x2_ne = [0usize; 11];
    let mut narrow_true = 0usize;
    // area(ExPolygons), remove_same_neighbor, keep_largest_contour_only, remove_sticks
    let mut misc_ne = [0usize; 4];
    while let Some(line) = it.next() {
        assert!(line.starts_with("case "));
        let a = read_ex(&mut it);
        let b = read_ex(&mut it);
        let pl = Polyline::from_points(read_points(it.next().unwrap(), "pl"));
        let l: Vec<i64> = tail(&mut it, "ln").iter().map(|v| v.parse().unwrap()).collect();
        let ln = Line::new(Point { x: l[0], y: l[1] }, Point { x: l[2], y: l[3] });
        let ang = hexf(tail(&mut it, "ang")[0]);
        let ov: Vec<bool> = tail(&mut it, "ov").iter().map(|v| *v == "1").collect();
        let cl = tail(&mut it, "cl")[0] == "1";
        let cp = tail(&mut it, "cp")[0] == "1";
        let bb: Vec<i64> = tail(&mut it, "bb").iter().map(|v| v.parse().unwrap()).collect();
        let dup = tail(&mut it, "dup")[0] == "1";
        let nlf: usize = tail(&mut it, "lf")[0].parse().unwrap();
        let lf: Vec<Vec<f64>> = (0..nlf).map(|_| it.next().unwrap().split_whitespace().map(hexf).collect()).collect();
        let p: Vec<i64> = tail(&mut it, "pt").iter().map(|v| v.parse().unwrap()).collect();
        let pt = Point { x: p[0], y: p[1] };
        let mw = hexf(tail(&mut it, "mw")[0]);
        let rot: usize = tail(&mut it, "rot")[0].parse().unwrap();
        let x2: Vec<i64> = tail(&mut it, "x2").iter().map(|v| v.parse().unwrap()).collect();
        {
            // A with its contour rotated by `rot` (std::rotate in the generator)
            let mut ar = a.clone();
            ar.contour.points.rotate_left(rot);
            let ab = vec![a.clone(), b.clone()];
            let bbx = get_extents_rotated(&ab, ang);
            let proj = projection_onto(&ab, &pt);
            let narrow = is_narrow_expolygon(&a, mw, 100000.0 * 100000.0, 0.1);
            let got = [
                <[ExPolygon] as ExPolygonsOverlaps<[ExPolygon]>>::overlaps(&[a.clone()], &[b.clone()]) as i64,
                <[ExPolygon] as ExPolygonsOverlaps<ExPolygon>>::overlaps(&[b.clone()], &a) as i64,
                narrow as i64,
                proj.x,
                proj.y,
                expolygons_match(&a, &ar) as i64,
                expolygons_match(&a, &b) as i64,
                bbx.min.x,
                bbx.min.y,
                bbx.max.x,
                bbx.max.y,
            ];
            for (i, (g, w)) in got.iter().zip(&x2).enumerate() {
                if g != w {
                    x2_ne[i] += 1;
                }
            }
            narrow_true += x2[2] as usize;
        }
        {
            let ar = hexf(tail(&mut it, "ar")[0]);
            // area(const ExPolygons&) (ExPolygon.hpp:488) by its C++ name and by the older one
            let abv = [a.clone(), b.clone()];
            if <[ExPolygon] as ExPolygonArea>::area(&abv).to_bits() != ar.to_bits()
                || slicer::geometry::area_expolygons(&abv).to_bits() != ar.to_bits()
            {
                misc_ne[0] += 1;
            }
            let t = tail(&mut it, "rsn");
            let (rb, k): (bool, usize) = (t[0] == "1", t[1].parse().unwrap());
            let want: Vec<ExPolygon> = (0..k).map(|_| read_ex(&mut it)).collect();
            let mut v = vec![a.clone(), b.clone()];
            if v[0].contour.points.len() > 1 {
                let p1 = v[0].contour.points[1];
                v[0].contour.points.insert(1, p1);
            }
            let r = slicer::ex_polygon::remove_same_neighbor(&mut v);
            if r != rb || !same_exs(&v, &want) {
                misc_ne[1] += 1;
            }
            let k: usize = tail(&mut it, "klc")[0].parse().unwrap();
            let want: Vec<ExPolygon> = (0..k).map(|_| read_ex(&mut it)).collect();
            let mut v = vec![a.clone(), b.clone()];
            slicer::ex_polygon::keep_largest_contour_only(&mut v);
            if !same_exs(&v, &want) {
                misc_ne[2] += 1;
            }
            let rb = tail(&mut it, "rs")[0] == "1";
            let want = vec![read_ex(&mut it)];
            let mut e = a.clone();
            let r = slicer::ex_polygon::remove_sticks(&mut e);
            if r != rb || !same_exs(&[e], &want) {
                misc_ne[3] += 1;
            }
        }
        cases += 1;
        ov_true += ov[0] as usize;
        cl_true += cl as usize;
        cp_true += cp as usize;
        dup_true += dup as usize;

        if a.overlaps(&b) != ov[0] || b.overlaps(&a) != ov[1] {
            ov_ne += 1;
        }
        if <ExPolygon as ExPolygonContains<Line>>::contains(&a, &ln) != cl {
            cl_ne += 1;
        }
        if <ExPolygon as ExPolygonContains<Polyline>>::contains(&a, &pl) != cp {
            cp_ne += 1;
        }
        let rb = get_extents_rotated_expoly(&a, ang);
        if (rb.min.x, rb.min.y, rb.max.x, rb.max.y) != (bb[0], bb[1], bb[2], bb[3]) {
            bb_ne += 1;
        }
        if <ExPolygon as ExPolygonHasDuplicatePoints>::has_duplicate_points(&a) != dup {
            dup_ne += 1;
        }
        let rl = to_unscaled_linesf(&[a.clone(), b.clone()]);
        let eq = rl.len() == lf.len()
            && rl.iter().zip(&lf).all(|(r, w)| {
                r.a.x.to_bits() == w[0].to_bits() && r.a.y.to_bits() == w[1].to_bits() && r.b.x.to_bits() == w[2].to_bits() && r.b.y.to_bits() == w[3].to_bits()
            });
        if !eq {
            lf_ne += 1;
        }
    }
    eprintln!(
        "ex_polygon_ops_native_parity: {cases} cases (pin true: overlaps {ov_true}, contains(Line) {cl_true}, contains(Polyline) {cp_true}, dup {dup_true}); differ: overlaps {ov_ne}, contains(Line) {cl_ne}, contains(Polyline) {cp_ne}, get_extents_rotated {bb_ne}, has_duplicate_points {dup_ne}, to_unscaled_linesf {lf_ne}; narrow (pin true {narrow_true}) / ovs / match / projection_onto / extents(ExPolygons) differ per field {:?}; area / remove_same_neighbor / keep_largest_contour_only / remove_sticks differ {:?}",
        x2_ne, misc_ne
    );
    assert_eq!(misc_ne, [0usize; 4]);
    assert!(cases > 0);
    assert_eq!((ov_ne, cl_ne, cp_ne, bb_ne, dup_ne, lf_ne), (0, 0, 0, 0, 0, 0));
    // is_narrow_expolygon (field 2) follows the pin with IS_NARROW_EXPOLYGON_PIN (on by default).
    if slicer::faithful_gate("IS_NARROW_EXPOLYGON_PIN") {
        assert_eq!(x2_ne, [0usize; 11]);
    } else {
        let mut rest = x2_ne;
        rest[2] = 0;
        assert_eq!(rest, [0usize; 11]);
    }
}
