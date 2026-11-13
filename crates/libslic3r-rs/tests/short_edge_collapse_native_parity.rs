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

#[test]
fn bumpy_spheres() {
    // The port's own arm is opt-in; the gates are read once, on first use, so this
    // has to come before any call into the engine. This file has one test.
    std::env::set_var("SEC_RUST", "1");
    let mut s: u64 = 0x9E37_79B9_7F4A_7C15;
    for round in 0..40 {
        let rings = 8 + (xorshift(&mut s) % 20) as usize;
        let segs = 10 + (xorshift(&mut s) % 30) as usize;
        let (verts, idx) = mesh(&mut s, rings, segs, [5.0f32, 40.0, 120.0][round % 3]);
        let target = idx.len() / 3 / (2 + (xorshift(&mut s) % 4) as usize);
        let (wv, wi) = eigen_transform_sys::short_edge_collapse(&verts, &idx, target);
        assert!(wi.len() < idx.len(), "the C++ collapsed nothing, round {round}");

        let mut its = indexed_triangle_set::default();
        its.vertices = verts.chunks_exact(3).map(|c| Vec3f::new(c[0], c[1], c[2])).collect();
        its.indices = idx.chunks_exact(3).map(|c| Vec3i::new(c[0], c[1], c[2])).collect();
        its_short_edge_collpase(&mut its, target);

        let gv: Vec<u32> = its.vertices.iter().flat_map(|p| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]).collect();
        let gi: Vec<i32> = its.indices.iter().flat_map(|t| [t.x, t.y, t.z]).collect();
        assert_eq!(gi.len(), wi.len(), "face count, round {round} ({rings}x{segs}, target {target})");
        assert_eq!(gi, wi, "faces, round {round}");
        assert_eq!(gv, wv.iter().map(|x| x.to_bits()).collect::<Vec<_>>(), "vertices, round {round}");
    }
}
