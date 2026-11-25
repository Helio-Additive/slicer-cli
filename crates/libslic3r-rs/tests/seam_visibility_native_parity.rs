//! SeamPlacer's `GlobalModelInfo::calculate_point_visibility` (SeamPlacer.cpp:301-326) against the pin.
//!
//! `tests/data/seam_visibility/gen.cpp` carries the pin's function body and `CoordinateFunctor`
//! (SeamPlacer.cpp:264-271) over the pin's `KDTreeIndirect<3, float, ...>` built as
//! `compute_global_occlusion` builds it (SeamPlacer.cpp:611-612), on three LCG sample sets (500 to
//! 20,000 samples with random normals and visibilities) and 7,500 queries, half of them at the
//! search radius from a sample. It writes the samples and per query the visibility and the number
//! of samples `find_nearby_points` returns (raw float bits). The port must give the same bits:
//! SEAM_VISIBILITY_EIGEN_DOT (Eigen's Vec3f dot, the third product fused) and SEAM_NEARBY_DIST_FMA
//! (the visitor's `dist += d * d` fused, KDTreeIndirect.hpp:300-303), both on by default.
use std::io::Read;

use slicer::gcode::seam_placer::{seam_find_nearby_points, GlobalModelInfo, Vec3f};

fn f(s: &str) -> f32 {
    f32::from_bits(u32::from_str_radix(s, 16).unwrap())
}

#[test]
fn point_visibility_matches_the_pin() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/seam_visibility/vectors.txt.gz");
    let file = std::fs::File::open(path).expect("vectors.txt.gz");
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");

    let mut gmi = GlobalModelInfo::default();
    let (mut cases, mut bad_vis, mut bad_count, mut scenarios) = (0usize, 0usize, 0usize, 0usize);
    let mut queries: Vec<(Vec3f, u32, usize)> = Vec::new();
    let mut run = |gmi: &GlobalModelInfo, queries: &mut Vec<(Vec3f, u32, usize)>, cases: &mut usize, bad_vis: &mut usize, bad_count: &mut usize| {
        if queries.is_empty() {
            return;
        }
        let tree = gmi.build_mesh_samples_tree();
        let positions = &gmi.mesh_samples.positions;
        for (pos, vis_bits, count) in queries.drain(..) {
            *cases += 1;
            let vis = gmi.calculate_point_visibility(&tree, &pos);
            let n = seam_find_nearby_points(&tree, |i: usize, d: usize| positions[i][d], &pos, gmi.mesh_samples_radius, |_| true).len();
            if vis.to_bits() != vis_bits {
                if *bad_vis < 10 {
                    eprintln!("visibility at {:?}: port {:08x} pin {:08x}", pos, vis.to_bits(), vis_bits);
                }
                *bad_vis += 1;
            }
            if n != count {
                if *bad_count < 10 {
                    eprintln!("nearby count at {:?}: port {} pin {}", pos, n, count);
                }
                *bad_count += 1;
            }
        }
    };
    for line in text.lines() {
        let t: Vec<&str> = line.split_whitespace().collect();
        match t.first().copied() {
            Some("SCENARIO") => {
                run(&gmi, &mut queries, &mut cases, &mut bad_vis, &mut bad_count);
                gmi = GlobalModelInfo::default();
                scenarios += 1;
            }
            Some("R") => gmi.mesh_samples_radius = f(t[1]),
            Some("S") => {
                let i: usize = t[1].parse().unwrap();
                assert_eq!(i, gmi.mesh_samples.positions.len());
                gmi.mesh_samples.positions.push(Vec3f::new(f(t[2]), f(t[3]), f(t[4])));
                gmi.mesh_samples.normals.push(Vec3f::new(f(t[5]), f(t[6]), f(t[7])));
                gmi.mesh_samples_visibility.push(f(t[8]));
            }
            Some("Q") => {
                let pos = Vec3f::new(f(t[1]), f(t[2]), f(t[3]));
                let vis_bits = u32::from_str_radix(t[4], 16).unwrap();
                queries.push((pos, vis_bits, t[5].parse().unwrap()));
            }
            _ => {}
        }
    }
    run(&gmi, &mut queries, &mut cases, &mut bad_vis, &mut bad_count);
    eprintln!("{} scenarios, {} queries: {} visibilities and {} nearby counts differ", scenarios, cases, bad_vis, bad_count);
    assert_eq!(scenarios, 3);
    assert_eq!(cases, 7500);
    assert_eq!(bad_count, 0, "find_nearby_points counts differ from the pin");
    assert_eq!(bad_vis, 0, "calculate_point_visibility differs from the pin");
}
