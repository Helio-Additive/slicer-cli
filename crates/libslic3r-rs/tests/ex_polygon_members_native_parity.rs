//! Native parity (R951) for the ExPolygon members and free-function overloads under their C++ names:
//! ExPolygon::scale x2, translate x2, rotate x2, clear, empty, is_valid, num_contours, simplify_p x2,
//! simplify x2 (ExPolygon.hpp:38-86, ExPolygon.cpp:16-67, 225-261), to_polylines x4, to_polygons x4,
//! to_expolygons x2, polygons_append x4, expolygons_append x2, get_extents x2, get_extents_rotated x2,
//! to_linesf (ExPolygon.hpp:150-178, 214-435, ExPolygon.cpp:490-520), on random star-shaped ExPolygons
//! with holes (a quarter snapped to a grid, some with a CCW hole or a CW contour).
//! Vectors: tests/data/ex_polygon/gen_members.cpp (members_vectors.txt.gz). Every line of the pin's
//! output is rebuilt from the Rust results in the same format and compared as text.

use slicer::ex_polygon::{
    count_points, expolygons_append, polygons_append, to_expolygons, to_linesf, to_polygons, to_polylines, ExPolygonMembers,
    ExPolygonRotate, ExPolygonScale, ExPolygonSimplify, ExPolygonSimplifyP, ExPolygonTranslate, GetExtents, GetExtentsRotated,
};
use slicer::geometry::{BoundingBox, ExPolygon, Point, Polygon, Polyline};
use std::fmt::Write;

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
    match std::env::var("EX_POLYGON_MEMBERS_VECTORS") {
        Ok(path) => std::fs::read_to_string(&path).expect("vectors"),
        Err(_) => {
            use std::io::Read;
            let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ex_polygon/members_vectors.txt.gz");
            let mut s = String::new();
            flate2::read::GzDecoder::new(std::fs::File::open(path).expect("members_vectors.txt.gz"))
                .read_to_string(&mut s)
                .expect("gzip");
            s
        }
    }
}

fn read_ex(line: &str, tag: &str) -> ExPolygon {
    let mut w = line.split_whitespace();
    assert_eq!(w.next(), Some(tag));
    let nh: usize = w.next().unwrap().parse().unwrap();
    let ring = |w: &mut std::str::SplitWhitespace| -> Vec<Point> {
        let n: usize = w.next().unwrap().parse().unwrap();
        (0..n).map(|_| Point::new(w.next().unwrap().parse().unwrap(), w.next().unwrap().parse().unwrap())).collect()
    };
    let mut ex = ExPolygon::new(Polygon::from_points(ring(&mut w)));
    for _ in 0..nh {
        ex.holes.push(Polygon::from_points(ring(&mut w)));
    }
    ex
}

fn pts(s: &mut String, p: &[Point]) {
    write!(s, " {}", p.len()).unwrap();
    for q in p {
        write!(s, " {} {}", q.x, q.y).unwrap();
    }
}
fn ex1(s: &mut String, e: &ExPolygon) {
    write!(s, " {}", e.holes.len()).unwrap();
    pts(s, &e.contour.points);
    for h in &e.holes {
        pts(s, &h.points);
    }
}
fn exo(tag: &str, e: &ExPolygon) -> String {
    let mut s = tag.to_string();
    ex1(&mut s, e);
    s
}
fn exs(tag: &str, v: &[ExPolygon]) -> String {
    let mut s = format!("{tag} {}", v.len());
    for e in v {
        ex1(&mut s, e);
    }
    s
}
fn pgs(tag: &str, v: &[Polygon]) -> String {
    let mut s = format!("{tag} {}", v.len());
    for p in v {
        pts(&mut s, &p.points);
    }
    s
}
fn pls(tag: &str, v: &[Polyline]) -> String {
    let mut s = format!("{tag} {}", v.len());
    for p in v {
        pts(&mut s, &p.points);
    }
    s
}
fn bbo(tag: &str, b: &BoundingBox) -> String {
    format!("{tag} {} {} {} {} {}", b.is_defined() as i32, b.min.x, b.min.y, b.max.x, b.max.y)
}

#[test]
fn ex_polygon_members_native_parity() {
    let text = vectors();
    let mut it = text.lines().peekable();
    let mut cases = 0usize;
    let mut lines_cmp = 0usize;
    let mut mismatches: std::collections::BTreeMap<String, usize> = Default::default();
    let mut first: Vec<String> = Vec::new();
    while let Some(l) = it.next() {
        assert!(l.starts_with("case "), "{l}");
        let a = read_ex(it.next().unwrap(), "a");
        let b = read_ex(it.next().unwrap(), "b");
        let p: Vec<&str> = it.next().unwrap().split_whitespace().collect();
        assert_eq!(p[0], "prm");
        let (fac, fx, fy, tx, ty, ang) = (hexf(p[1]), hexf(p[2]), hexf(p[3]), hexf(p[4]), hexf(p[5]), hexf(p[6]));
        let ctr = Point::new(p[7].parse().unwrap(), p[8].parse().unwrap());
        let tol = hexf(p[9]);
        let ab = vec![a.clone(), b.clone()];
        let mut mine: Vec<String> = Vec::new();
        {
            let mut e = a.clone();
            ExPolygonScale::<f64>::scale(&mut e, fac);
            mine.push(exo("sc1", &e));
            let mut e = a.clone();
            ExPolygonScale::<(f64, f64)>::scale(&mut e, (fx, fy));
            mine.push(exo("sc2", &e));
            let mut e = a.clone();
            ExPolygonTranslate::<&Point>::translate(&mut e, &ctr);
            mine.push(exo("tr1", &e));
            let mut e = a.clone();
            ExPolygonTranslate::<(f64, f64)>::translate(&mut e, (tx, ty));
            mine.push(exo("tr2", &e));
            let mut e = a.clone();
            ExPolygonRotate::<f64>::rotate(&mut e, ang);
            mine.push(exo("ro1", &e));
            let mut e = a.clone();
            ExPolygonRotate::<(f64, &Point)>::rotate(&mut e, (ang, &ctr));
            mine.push(exo("ro2", &e));
            let mut e = a.clone();
            let e0 = ExPolygonMembers::empty(&e);
            let nc = ExPolygonMembers::num_contours(&e);
            let v = ExPolygonMembers::is_valid(&e);
            ExPolygonMembers::clear(&mut e);
            mine.push(format!("mem {} {} {} {} {}", e0 as i32, nc, v as i32, ExPolygonMembers::empty(&e) as i32, e.holes.len()));
        }
        mine.push(pgs("sp1", &ExPolygonSimplifyP::<f64>::simplify_p(&a, tol)));
        {
            let mut pp = to_polygons(&b);
            ExPolygonSimplifyP::<(f64, &mut Vec<Polygon>)>::simplify_p(&a, (tol, &mut pp));
            mine.push(pgs("sp2", &pp));
        }
        mine.push(exs("si1", &ExPolygonSimplify::<f64>::simplify(&a, tol)));
        {
            let mut v = vec![b.clone()];
            ExPolygonSimplify::<(f64, &mut Vec<ExPolygon>)>::simplify(&a, (tol, &mut v));
            mine.push(exs("si2", &v));
            let mut v: Vec<ExPolygon> = Vec::new();
            ExPolygonSimplify::<(f64, &mut Vec<ExPolygon>)>::simplify(&a, (tol, &mut v));
            mine.push(exs("si3", &v));
        }
        mine.push(pls("pl1", &to_polylines(&a)));
        mine.push(pls("pl2", &to_polylines(ab.as_slice())));
        mine.push(pls("pl3", &to_polylines(a.clone())));
        mine.push(pls("pl4", &to_polylines(ab.clone())));
        mine.push(pgs("pg1", &to_polygons(&a)));
        mine.push(pgs("pg2", &to_polygons(ab.as_slice())));
        mine.push(pgs("pg3", &to_polygons(a.clone())));
        mine.push(pgs("pg4", &to_polygons(ab.clone())));
        {
            let src = to_polygons(ab.as_slice());
            mine.push(exs("te1", &to_expolygons(src.as_slice())));
            mine.push(exs("te2", &to_expolygons(src.clone())));
        }
        {
            let mut d = to_polygons(&b);
            polygons_append(&mut d, &a);
            mine.push(pgs("pa1", &d));
            let mut d = to_polygons(&b);
            polygons_append(&mut d, ab.as_slice());
            mine.push(pgs("pa2", &d));
            let mut d = to_polygons(&b);
            polygons_append(&mut d, a.clone());
            mine.push(pgs("pa3", &d));
            let mut d = to_polygons(&b);
            polygons_append(&mut d, ab.clone());
            mine.push(pgs("pa4", &d));
        }
        {
            let mut d = vec![b.clone()];
            expolygons_append(&mut d, ab.as_slice());
            mine.push(exs("ea1", &d));
            let mut d: Vec<ExPolygon> = Vec::new();
            expolygons_append(&mut d, ab.clone());
            mine.push(exs("ea2", &d));
            let mut d = vec![b.clone()];
            expolygons_append(&mut d, ab.clone());
            mine.push(exs("ea3", &d));
        }
        mine.push(bbo("ge1", &GetExtents::get_extents(&a)));
        mine.push(bbo("ge2", &GetExtents::get_extents(ab.as_slice())));
        mine.push(bbo("gr1", &GetExtentsRotated::get_extents_rotated(&a, ang)));
        mine.push(bbo("gr2", &GetExtentsRotated::get_extents_rotated(ab.as_slice(), ang)));
        {
            let l0 = to_linesf(&ab, 0);
            let l1 = to_linesf(&ab, count_points(ab.as_slice()) as u32);
            let mut s = format!("lf {} {}", l0.len(), l1.len());
            for l in &l0 {
                write!(s, " {:.1} {:.1} {:.1} {:.1}", l.a.x, l.a.y, l.b.x, l.b.y).unwrap();
            }
            let same = l0.len() == l1.len() && l0.iter().zip(&l1).all(|(x, y)| x.a == y.a && x.b == y.b);
            write!(s, " {}", same as i32).unwrap();
            mine.push(s);
        }
        for m in &mine {
            let pin = it.next().unwrap();
            lines_cmp += 1;
            if pin != m {
                let tag = pin.split_whitespace().next().unwrap_or("?").to_string();
                *mismatches.entry(tag.clone()).or_default() += 1;
                if first.len() < 6 {
                    first.push(format!("case {cases} {tag}:\n  pin  {}\n  rust {}", &pin[..pin.len().min(300)], &m[..m.len().min(300)]));
                }
            }
        }
        cases += 1;
    }
    eprintln!("ex_polygon_members: {cases} cases, {lines_cmp} result lines, mismatches by tag {mismatches:?}");
    for f in &first {
        eprintln!("{f}");
    }
    assert!(cases >= 1000, "too few cases: {cases}");
    assert!(mismatches.is_empty(), "mismatches: {mismatches:?}");
}
