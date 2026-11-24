//! Native parity for Tesselate.cpp over glu-libtess (R947): the pin's
//! triangulate_expolygons_3d, triangulate_expolygon_3d and
//! triangulate_expolygon_2d on random ExPolygons with holes (star polygons,
//! grid-snapped ones with collinear and duplicate points, rectangles with
//! overlapping holes that intersect: the combine callback), compared bit for
//! bit with `slicer::tesselate` (gate GLU_LIBTESS_PIN, on by default).
//! Vectors: tests/data/tesselate/gen.cpp.

use slicer::geometry::{ExPolygon, ExPolygons, Point, Polygon};
use slicer::tesselate::{
    triangulate_expolygon_2d, triangulate_expolygon_2f, triangulate_expolygon_3d, triangulate_expolygons_2d, triangulate_expolygons_2f,
    triangulate_expolygons_3d,
};

/// Parses C's `%a` output.
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
    match std::env::var("TESSELATE_VECTORS") {
        Ok(path) => std::fs::read_to_string(&path).expect("vectors"),
        Err(_) => {
            use std::io::Read;
            let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/tesselate/vectors.txt.gz");
            let mut s = String::new();
            flate2::read::GzDecoder::new(std::fs::File::open(path).expect("vectors.txt.gz"))
                .read_to_string(&mut s)
                .expect("gzip");
            s
        }
    }
}

fn read_poly(line: &str, tag: &str) -> Polygon {
    let mut w = line.split_whitespace();
    assert_eq!(w.next(), Some(tag));
    let n: usize = w.next().unwrap().parse().unwrap();
    let mut pts = Vec::with_capacity(n);
    for _ in 0..n {
        let x: i64 = w.next().unwrap().parse().unwrap();
        let y: i64 = w.next().unwrap().parse().unwrap();
        pts.push(Point { x, y });
    }
    Polygon::from_points(pts)
}

fn read_block<'a>(it: &mut impl Iterator<Item = &'a str>, tag: &str, dim: usize) -> Vec<Vec<f64>> {
    let head = it.next().unwrap();
    let mut w = head.split_whitespace();
    assert_eq!(w.next(), Some(tag));
    let n: usize = w.next().unwrap().parse().unwrap();
    (0..n)
        .map(|_| {
            let v: Vec<f64> = it.next().unwrap().split_whitespace().map(hexf).collect();
            assert_eq!(v.len(), dim);
            v
        })
        .collect()
}

#[test]
fn tesselate_native_parity() {
    let text = vectors();
    let mut it = text.lines();
    let (mut cases, mut tris) = (0usize, 0usize);
    let (mut bad3, mut bad1, mut bad2) = (0usize, 0usize, 0usize);
    let mut first_bad: Option<String> = None;
    let (mut combine_cases, mut combine_vertices) = (0usize, 0usize);
    let mut bad_rest = 0usize;
    while let Some(line) = it.next() {
        let w: Vec<&str> = line.split_whitespace().collect();
        assert_eq!(w[0], "case");
        let k: usize = w[1].parse().unwrap();
        let z = hexf(w[3]);
        let flip = w[4] == "1";
        let nex: usize = w[5].parse().unwrap();
        let mut exs: ExPolygons = Vec::new();
        for _ in 0..nex {
            let hl = it.next().unwrap();
            let nh: usize = hl.split_whitespace().nth(1).unwrap().parse().unwrap();
            let contour = read_poly(it.next().unwrap(), "c");
            let mut ex = ExPolygon::new(contour);
            for _ in 0..nh {
                ex.holes.push(read_poly(it.next().unwrap(), "h"));
            }
            exs.push(ex);
        }
        let r3 = read_block(&mut it, "r3", 3);
        let r1 = read_block(&mut it, "r1", 3);
        let r2 = read_block(&mut it, "r2", 2);
        let r4 = read_block(&mut it, "r4", 2);
        let r5 = read_block(&mut it, "r5", 2);
        let r6 = read_block(&mut it, "r6", 2);
        cases += 1;
        tris += r3.len() / 3;
        // Output vertices that are no input point: intersections made by the combine callback.
        {
            let mut input = std::collections::HashSet::new();
            for ex in &exs {
                for p in std::iter::once(&ex.contour).chain(ex.holes.iter()) {
                    for pt in &p.points {
                        input.insert(((pt.x as f64 * 0.00001).to_bits(), (pt.y as f64 * 0.00001).to_bits()));
                    }
                }
            }
            let n = r3.iter().filter(|v| !input.contains(&(v[0].to_bits(), v[1].to_bits()))).count();
            if n > 0 {
                combine_cases += 1;
                combine_vertices += n;
            }
        }

        let o3 = triangulate_expolygons_3d(&exs, z, flip).unwrap();
        let eq3 = o3.len() == r3.len()
            && o3.iter().zip(&r3).all(|(a, b)| a.x.to_bits() == b[0].to_bits() && a.y.to_bits() == b[1].to_bits() && a.z.to_bits() == b[2].to_bits());
        let o1 = triangulate_expolygon_3d(&exs[0], z, flip).unwrap();
        let eq1 = o1.len() == r1.len()
            && o1.iter().zip(&r1).all(|(a, b)| a.x.to_bits() == b[0].to_bits() && a.y.to_bits() == b[1].to_bits() && a.z.to_bits() == b[2].to_bits());
        let o2 = triangulate_expolygon_2d(&exs[0], flip).unwrap();
        let eq2 = o2.len() == r2.len() && o2.iter().zip(&r2).all(|(a, b)| a.x.to_bits() == b[0].to_bits() && a.y.to_bits() == b[1].to_bits());
        let o4 = triangulate_expolygons_2d(&exs, flip).unwrap();
        let eq4 = o4.len() == r4.len() && o4.iter().zip(&r4).all(|(a, b)| a.x.to_bits() == b[0].to_bits() && a.y.to_bits() == b[1].to_bits());
        let o5 = triangulate_expolygon_2f(&exs[0], flip).unwrap();
        let eq5 = o5.len() == r5.len() && o5.iter().zip(&r5).all(|(a, b)| (a.x as f64).to_bits() == b[0].to_bits() && (a.y as f64).to_bits() == b[1].to_bits());
        let o6 = triangulate_expolygons_2f(&exs, flip).unwrap();
        let eq6 = o6.len() == r6.len() && o6.iter().zip(&r6).all(|(a, b)| (a.x as f64).to_bits() == b[0].to_bits() && (a.y as f64).to_bits() == b[1].to_bits());
        if !(eq4 && eq5 && eq6) {
            bad_rest += 1;
        }
        if !eq3 {
            bad3 += 1;
            if first_bad.is_none() {
                first_bad = Some(format!("case {k} mode {} r3: rust {} pin {}", w[2], o3.len(), r3.len()));
            }
        }
        if !eq1 {
            bad1 += 1;
        }
        if !eq2 {
            bad2 += 1;
        }
    }
    eprintln!(
        "tesselate_native_parity: {cases} cases, {tris} pin triangles (expolygons_3d); {combine_cases} cases with {combine_vertices} intersection vertices; differ: expolygons_3d {bad3}, expolygon_3d {bad1}, expolygon_2d {bad2}, expolygons_2d / expolygon_2f / expolygons_2f {bad_rest}; first {:?}",
        first_bad
    );
    assert!(cases > 0);
    // With GLU_LIBTESS_PIN=0 tesselate.rs runs the earlier earcut arm: reported, not asserted.
    if slicer::faithful_gate("GLU_LIBTESS_PIN") {
        assert_eq!((bad3, bad1, bad2, bad_rest), (0, 0, 0, 0), "first {:?}", first_bad);
    }
}
