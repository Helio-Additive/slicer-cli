//! R934: `model::its_convex_hull` (the stand-in for qhull, TriangleMesh.cpp:1737) drops the faces that left the
//! hull after every insertion. This compares it with the earlier body, which kept them, verbatim below, on LCG
//! point sets (random, on a grid with coplanar points, on a sphere): the vertices and faces must be the same.
use slicer::model::its_convex_hull;
use slicer::triangle_mesh::{Vec3f, Vec3i};
use std::collections::HashMap;

fn reference(pts: &[Vec3f]) -> (Vec<Vec3f>, Vec<Vec3i>) {
    let mut out_v: Vec<Vec3f> = Vec::new();
    let mut out_i: Vec<Vec3i> = Vec::new();
    let mut points: Vec<[f64; 3]> = pts.iter().map(|p| [p.x as f64, p.y as f64, p.z as f64]).collect();
    points.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    points.dedup();
    let n = points.len();
    if n < 4 {
        return (out_v, out_i);
    }
    let sub = |a: &[f64; 3], b: &[f64; 3]| [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    let cross = |u: &[f64; 3], v: &[f64; 3]| [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
    let dot = |u: &[f64; 3], v: &[f64; 3]| u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
    let orient = |pts: &Vec<[f64; 3]>, a: usize, b: usize, c: usize, p: usize| {
        let n = cross(&sub(&pts[b], &pts[a]), &sub(&pts[c], &pts[a]));
        dot(&n, &sub(&pts[p], &pts[a]))
    };
    let mut i0 = 0;
    let mut i1 = 0;
    for i in 0..n {
        if points[i][0] < points[i0][0] {
            i0 = i;
        }
        if points[i][0] > points[i1][0] {
            i1 = i;
        }
    }
    if i0 == i1 {
        i1 = if i0 == 0 { 1 } else { 0 };
    }
    let d01 = sub(&points[i1], &points[i0]);
    let mut i2 = usize::MAX;
    let mut best = 0.0;
    for i in 0..n {
        let c = cross(&d01, &sub(&points[i], &points[i0]));
        let d = dot(&c, &c);
        if d > best {
            best = d;
            i2 = i;
        }
    }
    if i2 == usize::MAX {
        return (out_v, out_i);
    }
    let mut i3 = usize::MAX;
    let mut best = 0.0;
    for i in 0..n {
        let d = orient(&points, i0, i1, i2, i).abs();
        if d > best {
            best = d;
            i3 = i;
        }
    }
    if i3 == usize::MAX {
        return (out_v, out_i);
    }
    let mut faces: Vec<[usize; 3]> = Vec::new();
    if orient(&points, i0, i1, i2, i3) > 0.0 {
        faces.extend_from_slice(&[[i0, i2, i1], [i0, i1, i3], [i1, i2, i3], [i2, i0, i3]]);
    } else {
        faces.extend_from_slice(&[[i0, i1, i2], [i0, i3, i1], [i1, i3, i2], [i2, i3, i0]]);
    }
    let mut alive = vec![true; faces.len()];
    for p in 0..n {
        if p == i0 || p == i1 || p == i2 || p == i3 {
            continue;
        }
        let visible: Vec<usize> =
            (0..faces.len()).filter(|&f| alive[f] && orient(&points, faces[f][0], faces[f][1], faces[f][2], p) > 0.0).collect();
        if visible.is_empty() {
            continue;
        }
        let mut edge_count: HashMap<(usize, usize), usize> = HashMap::new();
        for &f in &visible {
            let t = faces[f];
            for k in 0..3 {
                *edge_count.entry((t[k], t[(k + 1) % 3])).or_insert(0) += 1;
            }
        }
        let mut horizon: Vec<(usize, usize)> = Vec::new();
        for &f in &visible {
            let t = faces[f];
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                if !edge_count.contains_key(&(b, a)) {
                    horizon.push((a, b));
                }
            }
        }
        for &f in &visible {
            alive[f] = false;
        }
        for (a, b) in horizon {
            faces.push([a, b, p]);
            alive.push(true);
        }
    }
    let mut map = vec![-1i32; n];
    for (f, t) in faces.iter().enumerate() {
        if !alive[f] {
            continue;
        }
        let mut idx = Vec3i::zeros();
        for k in 0..3 {
            if map[t[k]] == -1 {
                map[t[k]] = out_v.len() as i32;
                let p = points[t[k]];
                out_v.push(Vec3f::new(p[0] as f32, p[1] as f32, p[2] as f32));
            }
            idx[k] = map[t[k]];
        }
        out_i.push(idx);
    }
    (out_v, out_i)
}

struct Lcg(u64);
impl Lcg {
    fn next(&mut self) -> f32 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((self.0 >> 33) as f64 / (1u64 << 31) as f64) as f32
    }
}

#[test]
fn compacted_hull_equals_the_earlier_body() {
    let mut rng = Lcg(934);
    let mut cases = 0;
    for case in 0..60 {
        let n = 4 + (case * 37) % 400;
        let pts: Vec<Vec3f> = (0..n)
            .map(|_| match case % 3 {
                // random in a box
                0 => Vec3f::new(rng.next() * 40.0 - 20.0, rng.next() * 30.0, rng.next() * 10.0),
                // on a coarse grid: many coplanar and collinear points
                1 => Vec3f::new((rng.next() * 6.0).floor(), (rng.next() * 5.0).floor(), (rng.next() * 4.0).floor()),
                // on a sphere
                _ => {
                    let (u, v) = (rng.next() * 6.283185, rng.next() * 3.141592);
                    Vec3f::new(10.0 * v.sin() * u.cos(), 10.0 * v.sin() * u.sin(), 10.0 * v.cos())
                }
            })
            .collect();
        let (rv, ri) = reference(&pts);
        let got = its_convex_hull(&pts);
        assert_eq!(got.vertices.len(), rv.len(), "case {case}: vertex count");
        assert_eq!(got.indices.len(), ri.len(), "case {case}: face count");
        for (a, b) in got.vertices.iter().zip(rv.iter()) {
            assert_eq!((a.x.to_bits(), a.y.to_bits(), a.z.to_bits()), (b.x.to_bits(), b.y.to_bits(), b.z.to_bits()), "case {case}");
        }
        for (a, b) in got.indices.iter().zip(ri.iter()) {
            assert_eq!((a[0], a[1], a[2]), (b[0], b[1], b[2]), "case {case}");
        }
        cases += 1;
    }
    println!("its_convex_hull compaction: {cases} cases equal");
}
