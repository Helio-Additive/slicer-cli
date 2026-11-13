//! `raycast_visibility` (SeamPlacer.cpp:135-214): the Rust port against the C++, compiled with
//! Eigen and the engine's AABB tree in `eigen-transform-sys`, on the Benchy of the parity
//! fixtures decimated as the seam placer decimates it.

use slicer::gcode::seam_placer::{raycast_visibility, SQR_RAYS_PER_SAMPLE_POINT};
use slicer::triangle_set_sampling::{indexed_triangle_set, sample_its_uniform_parallel, Vec3f};

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
fn benchy() {
    let stl = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/rust-engine/fixtures/3DBenchy.stl");
    let (verts, idx) = load_stl(&stl);
    // Decimate with the C++, so the mesh is the one the C++ seam placer has.
    let (verts, idx) = eigen_transform_sys::short_edge_collapse(&verts, &idx, 16_000);

    let mut its = indexed_triangle_set::default();
    its.vertices = verts.chunks_exact(3).map(|c| Vec3f::new(c[0], c[1], c[2])).collect();
    its.indices = idx.chunks_exact(3).map(|c| nalgebra::Vector3::new(c[0], c[1], c[2])).collect();
    let samples = sample_its_uniform_parallel(30_000, &its);

    let pos: Vec<f32> = samples.positions.iter().flat_map(|p| [p.x, p.y, p.z]).collect();
    let nrm: Vec<f32> = samples.normals.iter().flat_map(|p| [p.x, p.y, p.z]).collect();
    let want = eigen_transform_sys::raycast_visibility_native(&verts, &idx, &pos, &nrm, SQR_RAYS_PER_SAMPLE_POINT);

    let verts_f64: Vec<slicer::geometry::Point3F> =
        its.vertices.iter().map(|v| slicer::geometry::Point3F::new(v.x as f64, v.y as f64, v.z as f64)).collect();
    let faces: Vec<[usize; 3]> = its.indices.iter().map(|i| [i.x as usize, i.y as usize, i.z as usize]).collect();
    let tree = slicer::aabb_tree_indirect::build_aabb_tree_over_indexed_triangle_set(&verts_f64, &faces);
    let got = raycast_visibility(&tree, &its, &verts_f64, &faces, &samples, its.indices.len());

    assert_eq!(got.len(), want.len());
    let bad: Vec<(usize, f32, f32)> = got
        .iter()
        .zip(&want)
        .enumerate()
        .filter(|(_, (g, w))| g.to_bits() != w.to_bits())
        .map(|(i, (g, w))| (i, *g, *w))
        .collect();
    assert!(bad.is_empty(), "{} of {} samples differ, the first: {:?}", bad.len(), got.len(), &bad[..bad.len().min(5)]);
}
