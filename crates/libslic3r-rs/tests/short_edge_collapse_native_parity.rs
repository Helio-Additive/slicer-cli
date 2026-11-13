//! `its_short_edge_collpase` (ShortEdgeCollapse.cpp:11-185): the Rust port against the C++,
//! compiled with Eigen in `eigen-transform-sys`: the vertices bit for bit and the faces.

use slicer::normal_utils::{indexed_triangle_set, Vec3f};
use slicer::short_edge_collapse::its_short_edge_collpase;

type Vec3i = nalgebra::Vector3<i32>;

fn xorshift(s: &mut u64) -> u64 {
    *s ^= *s << 13;
    *s ^= *s >> 7;
    *s ^= *s << 17;
    *s
}

/// A closed, bumpy sphere of `rings` x `segs` quads split in two.
fn mesh(s: &mut u64, rings: usize, segs: usize, radius: f32) -> (Vec<f32>, Vec<i32>) {
    let mut v: Vec<f32> = Vec::new();
    let mut bump = |s: &mut u64| 1.0 + ((xorshift(s) % 2001) as f32 / 1000.0 - 1.0) * 0.03;
    v.extend([0.0, 0.0, radius]);
    for r in 1..rings {
        let th = std::f32::consts::PI * r as f32 / rings as f32;
        for g in 0..segs {
            let ph = 2.0 * std::f32::consts::PI * g as f32 / segs as f32;
            let k = radius * bump(s);
            v.extend([k * th.sin() * ph.cos(), k * th.sin() * ph.sin(), k * th.cos()]);
        }
    }
    v.extend([0.0, 0.0, -radius]);
    let last = ((rings - 1) * segs + 1) as i32;
    let at = |r: usize, g: usize| (1 + (r - 1) * segs + (g % segs)) as i32;
    let mut f: Vec<i32> = Vec::new();
    for g in 0..segs {
        f.extend([0, at(1, g), at(1, g + 1)]);
        f.extend([last, at(rings - 1, g + 1), at(rings - 1, g)]);
    }
    for r in 1..rings - 1 {
        for g in 0..segs {
            f.extend([at(r, g), at(r + 1, g), at(r + 1, g + 1)]);
            f.extend([at(r, g), at(r + 1, g + 1), at(r, g + 1)]);
        }
    }
    (v, f)
}

fn compare(verts: &[f32], idx: &[i32], target: usize, what: &str) {
    let (wv, wi) = eigen_transform_sys::short_edge_collapse(verts, idx, target);
    let mut its = indexed_triangle_set::default();
    its.vertices = verts.chunks_exact(3).map(|c| Vec3f::new(c[0], c[1], c[2])).collect();
    its.indices = idx.chunks_exact(3).map(|c| Vec3i::new(c[0], c[1], c[2])).collect();
    its_short_edge_collpase(&mut its, target);
    let gv: Vec<u32> = its.vertices.iter().flat_map(|p| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]).collect();
    let gi: Vec<i32> = its.indices.iter().flat_map(|t| [t.x, t.y, t.z]).collect();
    assert_eq!(gi.len(), wi.len(), "face count, {what}");
    assert_eq!(gi, wi, "faces, {what}");
    assert_eq!(gv, wv.iter().map(|x| x.to_bits()).collect::<Vec<_>>(), "vertices, {what}");
}

/// A binary STL as an indexed mesh: vertices with the same three floats are one vertex.
fn load_stl(path: &std::path::Path) -> (Vec<f32>, Vec<i32>) {
    let data = std::fs::read(path).expect("fixture STL");
    let n = u32::from_le_bytes(data[80..84].try_into().unwrap()) as usize;
    let mut verts: Vec<f32> = Vec::new();
    let mut idx: Vec<i32> = Vec::new();
    let mut seen: std::collections::HashMap<[u32; 3], i32> = std::collections::HashMap::new();
    for t in 0..n {
        let base = 84 + t * 50 + 12;
        let mut tri = [0i32; 3];
        for (c, slot) in tri.iter_mut().enumerate() {
            let o = base + c * 12;
            let f = |k: usize| f32::from_le_bytes(data[o + 4 * k..o + 4 * k + 4].try_into().unwrap());
            let p = [f(0), f(1), f(2)];
            let key = [p[0].to_bits(), p[1].to_bits(), p[2].to_bits()];
            *slot = *seen.entry(key).or_insert_with(|| {
                verts.extend(p);
                (verts.len() / 3 - 1) as i32
            });
        }
        if tri[0] != tri[1] && tri[1] != tri[2] && tri[0] != tri[2] {
            idx.extend(tri);
        }
    }
    (verts, idx)
}

#[test]
fn against_the_cpp() {
    let mut s: u64 = 0x9E37_79B9_7F4A_7C15;
    for round in 0..46 {
        // The last rounds are large: tens of thousands of faces, as a real model has.
        let big = round >= 40;
        let rings = if big { 60 + 30 * (round - 40) } else { 8 + (xorshift(&mut s) % 20) as usize };
        let segs = if big { 90 + 40 * (round - 40) } else { 10 + (xorshift(&mut s) % 30) as usize };
        let (verts, idx) = mesh(&mut s, rings, segs, [5.0f32, 40.0, 120.0][round % 3]);
        let target = idx.len() / 3 / (2 + (xorshift(&mut s) % 4) as usize);
        compare(&verts, &idx, target, &format!("sphere {round} ({rings}x{segs}, target {target})"));
    }
    // A real model: the Benchy of the parity fixtures, to the seam placer's two targets.
    let stl = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/rust-engine/fixtures/3DBenchy.stl");
    let (verts, idx) = load_stl(&stl);
    assert!(idx.len() / 3 > 100_000, "the Benchy has {} faces", idx.len() / 3);
    for target in [16_000usize, 2_000] {
        compare(&verts, &idx, target, &format!("Benchy to {target}"));
    }
    // Dirty meshes: a sphere with some faces repeated, some turned over, some
    // degenerate, and extra faces hung on existing edges (three faces on an edge).
    for round in 0..60 {
        let rings = 8 + (xorshift(&mut s) % 16) as usize;
        let segs = 10 + (xorshift(&mut s) % 20) as usize;
        let (verts, mut idx) = mesh(&mut s, rings, segs, 30.0);
        let nv = (verts.len() / 3) as u64;
        let nf = idx.len() / 3;
        for _ in 0..(3 + round % 7) {
            let f = (xorshift(&mut s) % nf as u64) as usize;
            let (a, b, c) = (idx[3 * f], idx[3 * f + 1], idx[3 * f + 2]);
            match xorshift(&mut s) % 4 {
                0 => idx.extend([a, b, c]),
                1 => idx.extend([a, c, b]),
                2 => idx.extend([a, b, b]),
                _ => idx.extend([a, b, (xorshift(&mut s) % nv) as i32]),
            }
        }
        let target = idx.len() / 3 / 3;
        compare(&verts, &idx, target, &format!("dirty sphere {round} ({rings}x{segs})"));
    }
}
