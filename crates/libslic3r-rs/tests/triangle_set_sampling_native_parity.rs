//! `sample_its_uniform_parallel` (TriangleSetSampling.cpp:9-70): the Rust port against the C++,
//! compiled with Eigen in `eigen-transform-sys`, bit for bit: the total area, the triangle of every
//! sample, its position and its normal.

use slicer::triangle_set_sampling::{indexed_triangle_set, sample_its_uniform_parallel, Vec3f};

fn xorshift(s: &mut u64) -> u64 {
    *s ^= *s << 13;
    *s ^= *s >> 7;
    *s ^= *s << 17;
    *s
}

#[test]
fn random_meshes() {
    let mut s: u64 = 0x9E37_79B9_7F4A_7C15;
    for round in 0..400 {
        let nv = 8 + (xorshift(&mut s) % 40) as usize;
        let span = [1.0f64, 100.0, 300.0][round % 3];
        let verts: Vec<f32> = (0..nv * 3)
            .map(|_| ((xorshift(&mut s) % 2_000_001) as f64 / 1_000_000.0 * span - span) as f32)
            .collect();
        let nt = 10 + (xorshift(&mut s) % 60) as usize;
        let idx: Vec<i32> = (0..nt)
            .flat_map(|_| {
                let a = (xorshift(&mut s) % nv as u64) as i32;
                let b = (a + 1 + (xorshift(&mut s) % 5) as i32) % nv as i32;
                let c = (b + 1 + (xorshift(&mut s) % 5) as i32) % nv as i32;
                [a, b, c]
            })
            .collect();
        let n = 300usize;
        let (wpos, wnrm, wtri, warea) = eigen_transform_sys::sample_uniform(&verts, &idx, n);

        let mut its = indexed_triangle_set::default();
        its.vertices = verts.chunks_exact(3).map(|c| Vec3f::new(c[0], c[1], c[2])).collect();
        its.indices = idx
            .chunks_exact(3)
            .map(|c| nalgebra::Vector3::new(c[0], c[1], c[2]))
            .collect();
        let got = sample_its_uniform_parallel(n, &its);

        assert_eq!(got.total_area.to_bits(), warea.to_bits(), "total area, round {round}");
        for i in 0..n {
            assert_eq!(got.triangle_indices[i] as i64, wtri[i], "triangle of sample {i}, round {round}");
            let p = got.positions[i];
            let nr = got.normals[i];
            for (d, (g, w)) in [(p.x, wpos[3 * i]), (p.y, wpos[3 * i + 1]), (p.z, wpos[3 * i + 2])].iter().enumerate() {
                assert_eq!(g.to_bits(), w.to_bits(), "position {i}[{d}], round {round}");
            }
            for (d, (g, w)) in [(nr.x, wnrm[3 * i]), (nr.y, wnrm[3 * i + 1]), (nr.z, wnrm[3 * i + 2])].iter().enumerate() {
                assert_eq!(g.to_bits(), w.to_bits(), "normal {i}[{d}], round {round}");
            }
        }
    }
}
