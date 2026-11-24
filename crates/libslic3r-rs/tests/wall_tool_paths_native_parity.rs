//! The outline preparation of `WallToolPaths::generate` (WallToolPaths.cpp:458-480) against the pin's
//! own free functions `simplify`, `fixSelfIntersections`, `removeDegenerateVerts`, `removeColinearEdges`
//! and `removeSmallAreas` (WallToolPaths.cpp:50-439): vectors from `tests/data/wall_tool_paths/gen.cpp`.
//! "chain" and "real" cases run the seven steps in generate()'s order, each step fed the pin's output of
//! the step before; the "real" case is a Majora layer-1 outline whose fixSelfIntersections output
//! differed until R946 (ARACHNE_FIXSELF_PIN). "direct" cases run each function on raw generated
//! outlines with its own arguments. Every output polygon must be equal, point for point.

use slicer::arachne::wall_tool_paths::{
    fix_self_intersections, remove_colinear_edges, remove_degenerate_verts, remove_small_areas,
    simplify_polygons,
};
use slicer::geometry::{Point, Polygon, Polygons};
use std::io::Read;

fn parse_hex_f64(s: &str) -> f64 {
    let s = s.trim();
    let (neg, s) = match s.strip_prefix('-') {
        Some(r) => (true, r),
        None => (false, s),
    };
    let s = s.strip_prefix("0x").expect("hex float");
    let (mant, exp) = s.split_once('p').expect("exponent");
    let (int, frac) = mant.split_once('.').unwrap_or((mant, ""));
    let mut v = u64::from_str_radix(int, 16).expect("hex digits") as f64;
    let mut scale = 1.0 / 16.0;
    for c in frac.chars() {
        v += c.to_digit(16).expect("hex digit") as f64 * scale;
        scale /= 16.0;
    }
    let e: i32 = exp.parse().expect("exponent digits");
    let v = v * 2f64.powi(e);
    if neg {
        -v
    } else {
        v
    }
}

/// Reads "step <name> <n>" and the n "poly" lines after it.
fn read_step<'a>(lines: &mut impl Iterator<Item = &'a str>, name: &str) -> Polygons {
    let head: Vec<&str> = lines.next().expect("step line").split_whitespace().collect();
    assert_eq!(head[0], "step");
    assert_eq!(head[1], name, "step order");
    let n: usize = head[2].parse().expect("polygon count");
    let mut out = Polygons::new();
    for _ in 0..n {
        let t: Vec<&str> = lines.next().expect("poly line").split_whitespace().collect();
        assert_eq!(t[0], "poly");
        let np: usize = t[1].parse().expect("point count");
        let mut p = Polygon::new();
        for s in &t[2..2 + np] {
            let (x, y) = s.split_once(',').expect("x,y");
            p.points.push(Point::new(x.parse().expect("x"), y.parse().expect("y")));
        }
        out.push(p);
    }
    out
}

fn same(a: &Polygons, b: &Polygons) -> bool {
    a.len() == b.len() && a.iter().zip(b.iter()).all(|(p, q)| p.points == q.points)
}

#[derive(Default)]
struct Tally {
    cases: std::collections::BTreeMap<String, (usize, usize)>,
}

impl Tally {
    fn add(&mut self, step: &str, ok: bool) {
        let e = self.cases.entry(step.to_string()).or_default();
        e.0 += 1;
        if !ok {
            e.1 += 1;
        }
    }
}

#[test]
fn wall_tool_paths_outline_preparation_native_parity() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/wall_tool_paths/vectors.txt.gz");
    let file = std::fs::File::open(path).expect("vectors");
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    let mut lines = text.lines().peekable();
    let mut tally = Tally::default();
    let mut first_diff: Option<String> = None;
    let mut note = |tally: &mut Tally, case: &str, step: &str, ok: bool| {
        tally.add(step, ok);
        if !ok && first_diff.is_none() {
            first_diff = Some(format!("{case} {step}"));
        }
    };
    while let Some(line) = lines.next() {
        let t: Vec<&str> = line.split_whitespace().collect();
        assert_eq!(t[0], "case");
        let case = format!("{} {}", t[1], t[2]);
        match t[1] {
            "real" | "chain" => {
                let p: Vec<&str> = lines.next().expect("param").split_whitespace().collect();
                assert_eq!(p[0], "param");
                let smallest_segment: i64 = p[1].parse().unwrap();
                let allowed_distance: i64 = p[2].parse().unwrap();
                let epsilon_offset: i64 = p[3].parse().unwrap();
                let small_area_length = parse_hex_f64(p[4]);
                let input = read_step(&mut lines, "in");
                // WallToolPaths.cpp:470
                let mut v = input.clone();
                simplify_polygons(&mut v, smallest_segment, allowed_distance);
                let want = read_step(&mut lines, "simplify");
                note(&mut tally, &case, "chain simplify", same(&v, &want));
                // WallToolPaths.cpp:471
                let mut v = want;
                fix_self_intersections(epsilon_offset, &mut v);
                let want = read_step(&mut lines, "fixself");
                note(&mut tally, &case, "chain fixself 1", same(&v, &want));
                // WallToolPaths.cpp:472
                let mut v = want;
                remove_degenerate_verts(&mut v);
                let want = read_step(&mut lines, "degenerate");
                note(&mut tally, &case, "chain degenerate 1", same(&v, &want));
                // WallToolPaths.cpp:473
                let mut v = want;
                remove_colinear_edges(&mut v, 0.005);
                let want = read_step(&mut lines, "colinear");
                note(&mut tally, &case, "chain colinear", same(&v, &want));
                // WallToolPaths.cpp:475
                let mut v = want;
                fix_self_intersections(epsilon_offset, &mut v);
                let want = read_step(&mut lines, "fixself");
                note(&mut tally, &case, "chain fixself 2", same(&v, &want));
                // WallToolPaths.cpp:476
                let mut v = want;
                remove_degenerate_verts(&mut v);
                let want = read_step(&mut lines, "degenerate");
                note(&mut tally, &case, "chain degenerate 2", same(&v, &want));
                // WallToolPaths.cpp:477
                let mut v = want;
                remove_small_areas(&mut v, small_area_length * small_area_length, false);
                let want = read_step(&mut lines, "smallareas");
                note(&mut tally, &case, "chain smallareas", same(&v, &want));
            }
            "direct" => {
                let input = read_step(&mut lines, "in");
                // simplify
                let a: Vec<&str> = lines.next().unwrap().split_whitespace().collect();
                assert_eq!(&a[..2], &["arg", "simplify"]);
                let mut v = input.clone();
                simplify_polygons(&mut v, a[2].parse().unwrap(), a[3].parse().unwrap());
                let want = read_step(&mut lines, "simplify");
                note(&mut tally, &case, "direct simplify", same(&v, &want));
                // fixSelfIntersections
                let a: Vec<&str> = lines.next().unwrap().split_whitespace().collect();
                assert_eq!(&a[..2], &["arg", "fixself"]);
                let mut v = input.clone();
                fix_self_intersections(a[2].parse().unwrap(), &mut v);
                let want = read_step(&mut lines, "fixself");
                note(&mut tally, &case, "direct fixself", same(&v, &want));
                // removeDegenerateVerts
                let mut v = input.clone();
                remove_degenerate_verts(&mut v);
                let want = read_step(&mut lines, "degenerate");
                note(&mut tally, &case, "direct degenerate", same(&v, &want));
                // removeColinearEdges
                let a: Vec<&str> = lines.next().unwrap().split_whitespace().collect();
                assert_eq!(&a[..2], &["arg", "colinear"]);
                let mut v = input.clone();
                remove_colinear_edges(&mut v, parse_hex_f64(a[2]));
                let want = read_step(&mut lines, "colinear");
                note(&mut tally, &case, "direct colinear", same(&v, &want));
                // removeSmallAreas
                let a: Vec<&str> = lines.next().unwrap().split_whitespace().collect();
                assert_eq!(&a[..2], &["arg", "smallareas"]);
                let mut v = input.clone();
                remove_small_areas(&mut v, parse_hex_f64(a[2]), a[3] == "1");
                let want = read_step(&mut lines, "smallareas");
                note(&mut tally, &case, "direct smallareas", same(&v, &want));
            }
            other => panic!("unknown case kind {other}"),
        }
    }
    let mut total_diff = 0usize;
    for (step, (n, d)) in &tally.cases {
        println!("{step}: {n} cases, {d} differ");
        total_diff += d;
    }
    assert!(!tally.cases.is_empty());
    assert_eq!(total_diff, 0, "first differing case: {:?}", first_diff);
}
