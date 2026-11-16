//! The mesh makers of `slicer::triangle_mesh` (the port of TriangleMesh.cpp's its_make_*) against
//! the C++ at the pin.
//!
//! `tests/data/triangle_mesh_makers/vectors.txt.gz` is the output of `gen.cpp` beside it, which
//! links the pin's TriangleMesh.cpp and prints, for a set of arguments of every maker, every vertex
//! (floats bit for bit) and every face; the header of gen.cpp describes the lines. The same calls
//! run here and have to print the same lines. Run with the faithful arm of `ITS_MAKE_FLOAT_STEP`
//! (the default).

use slicer::normal_utils::indexed_triangle_set;
use slicer::triangle_mesh::{
    its_make_cone, its_make_cube, its_make_cylinder, its_make_frustum_dowel, its_make_prism, its_make_pyramid,
    its_make_snap, its_make_sphere, its_make_thread, its_make_xoy_center_rect,
};
use std::io::Read;

fn read_vectors() -> String {
    let path = format!("{}/tests/data/triangle_mesh_makers/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    text
}

fn d(hex: &str) -> f64 {
    f64::from_bits(u64::from_str_radix(hex, 16).expect("double bits"))
}

fn f(hex: &str) -> f32 {
    f32::from_bits(u32::from_str_radix(hex, 16).expect("float bits"))
}

fn mesh_text(its: &indexed_triangle_set) -> String {
    let mut s = format!("{} {}", its.vertices.len(), its.indices.len());
    for v in &its.vertices {
        for k in 0..3 {
            s += &format!(" {:08x}", v[k].to_bits());
        }
    }
    for t in &its.indices {
        s += &format!(" {} {} {}", t[0], t[1], t[2]);
    }
    s
}

#[test]
fn mesh_makers_match_the_pin() {
    let text = read_vectors();
    let mut by_maker: std::collections::BTreeMap<String, (usize, usize)> = std::collections::BTreeMap::new();
    for line in text.lines() {
        let (lhs, want) = line.split_once(" -> ").expect("arrow");
        let a: Vec<&str> = lhs.split(' ').collect();
        let its = match a[0] {
            "rect" => its_make_xoy_center_rect(f(a[1]), f(a[2]), f(a[3])),
            "cube" => its_make_cube(d(a[1]), d(a[2]), d(a[3])),
            "prism" => its_make_prism(f(a[1]), f(a[2]), f(a[3])),
            "cylinder" => its_make_cylinder(d(a[1]), d(a[2]), d(a[3])),
            "thread" => its_make_thread(d(a[1]), d(a[2]), d(a[3]), d(a[4])),
            "cone" => its_make_cone(d(a[1]), d(a[2]), d(a[3])),
            "dowel" => its_make_frustum_dowel(d(a[1]), d(a[2]), a[3].parse().unwrap()),
            "pyramid" => its_make_pyramid(f(a[1]), f(a[2])),
            "sphere" => its_make_sphere(d(a[1]), d(a[2])),
            "snap" => its_make_snap(d(a[1]), d(a[2]), f(a[3]), f(a[4])),
            m => panic!("unknown maker {m}"),
        };
        let got = mesh_text(&its);
        let e = by_maker.entry(a[0].to_string()).or_insert((0, 0));
        e.0 += 1;
        if got != want {
            e.1 += 1;
            let (gw, ww): (Vec<&str>, Vec<&str>) = (got.split(' ').collect(), want.split(' ').collect());
            let first = (0..gw.len().min(ww.len())).find(|&k| gw[k] != ww[k]);
            eprintln!(
                "{lhs}: rust {} tokens, c++ {}; first difference at token {:?}: rust {:?} c++ {:?}",
                gw.len(),
                ww.len(),
                first,
                first.map(|k| gw[k]),
                first.map(|k| ww[k])
            );
        }
    }
    println!("meshes by maker (compared, differing): {by_maker:?}");
    assert!(by_maker.values().all(|&(_, bad)| bad == 0), "meshes differ from the C++: {by_maker:?}");
}
