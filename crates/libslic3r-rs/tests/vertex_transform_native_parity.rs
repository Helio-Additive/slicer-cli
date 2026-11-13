//! The float transforms of mesh vertices (`make_trafo_for_slicing` and `tf * v`,
//! TriangleMeshSlicer.cpp:1945-1951 and :2007-2008; the f32 product of MultiMaterialSegmentation.cpp:2303):
//! the Rust port against the C++, compiled with Eigen in `eigen-transform-sys`, bit for bit.

use slicer::triangle_mesh_slicer as tms;

fn xorshift(s: &mut u64) -> u64 {
    *s ^= *s << 13;
    *s ^= *s >> 7;
    *s ^= *s << 17;
    *s
}

fn unit(s: &mut u64) -> f64 {
    (xorshift(s) % 2_000_001) as f64 / 1_000_000.0 - 1.0
}

fn trafo(s: &mut u64, identity_linear: bool) -> [f64; 16] {
    let mut t = [0f64; 16];
    for r in 0..3 {
        for c in 0..3 {
            t[r * 4 + c] = if identity_linear { (r == c) as u8 as f64 } else { unit(s) * 1.3 };
        }
        t[r * 4 + 3] = unit(s) * 180.0;
    }
    t[15] = 1.0;
    t
}

fn verts(s: &mut u64) -> Vec<f32> {
    (0..60).map(|_| (unit(s) * 150.0) as f32).collect()
}

fn bits(v: &[f32]) -> Vec<u32> {
    v.iter().map(|x| x.to_bits()).collect()
}

#[test]
fn unified() {
    let mut s: u64 = 0x9E37_79B9_7F4A_7C15;
    for i in 0..20_000 {
        let t = trafo(&mut s, i % 3 == 0);
        let voff = (unit(&mut s) * 120.0, unit(&mut s) * 120.0, unit(&mut s) * 5.0);
        let v = verts(&mut s);
        assert_eq!(
            bits(&tms::transform_verts_unified(&t, 0.00001, voff, &v)),
            bits(&eigen_transform_sys::transform_verts_unified(&t, 0.00001, voff, &v)),
            "trafo {t:?} voff {voff:?}"
        );
    }
}

#[test]
fn for_slicing() {
    let mut s: u64 = 0xD1B5_4A32_D192_ED03;
    for _ in 0..20_000 {
        let (cx, cy) = (unit(&mut s) * 200.0, unit(&mut s) * 200.0);
        let v = verts(&mut s);
        assert_eq!(
            bits(&tms::transform_verts_for_slicing(0.00001, cx, cy, &v)),
            bits(&eigen_transform_sys::transform_verts_for_slicing(0.00001, cx, cy, &v)),
            "centre {cx} {cy}"
        );
    }
}

#[test]
fn affine_f32_pair() {
    let mut s: u64 = 0xA076_1D64_78BD_642F;
    for i in 0..20_000 {
        let a = trafo(&mut s, i % 4 == 0);
        let b = trafo(&mut s, i % 5 == 0);
        let v = verts(&mut s);
        assert_eq!(
            bits(&tms::transform_verts_affine_f32_pair(&a, &b, &v)),
            bits(&eigen_transform_sys::transform_verts_affine_f32_pair(&a, &b, &v)),
            "a {a:?} b {b:?}"
        );
    }
}
