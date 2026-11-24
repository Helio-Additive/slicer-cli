//! R941: slice_mesh_slabs (TriangleMeshSlicer.cpp:2177-2280) with make_slab_loops (1589-1710)
//! and chain_lines_by_triangle_connectivity (1063-1161) against the pin, on meshes of the pin's
//! makers with slicing planes through their vertices (loops chain through shared vertices).
//! Vectors: tests/data/triangle_mesh_slicer/gen.cpp (`gen <out.txt>`, gzip -9n into vectors.txt.gz),
//! or TMS_VECTORS=<plain file>.
//! The comparison is exact: same polygons, same order, same start point, same points.
use slicer::normal_utils::{indexed_triangle_set, StlTriangleVertexIndices, StlVertex};
use slicer::triangle_mesh_slicer::slice_mesh_slabs;

fn hex(s: &str) -> f64 {
    // %a of a double: [-]0x1.<hex>p<exp> or 0x0p+0
    let neg = s.starts_with('-');
    let s = s.trim_start_matches('-').trim_start_matches("0x");
    let (mant, exp) = s.split_once('p').unwrap();
    let (int_part, frac) = mant.split_once('.').unwrap_or((mant, ""));
    let mut v = u64::from_str_radix(int_part, 16).unwrap() as f64;
    let mut scale = 1.0 / 16.0;
    for c in frac.chars() {
        v += c.to_digit(16).unwrap() as f64 * scale;
        scale /= 16.0;
    }
    let e: i32 = exp.parse().unwrap();
    let r = v * 2f64.powi(e);
    if neg { -r } else { r }
}

fn read_layers<'a>(it: &mut impl Iterator<Item = &'a str>, tag: &str, nz: usize) -> Vec<Vec<Vec<(i64, i64)>>> {
    let mut out = Vec::with_capacity(nz);
    for i in 0..nz {
        let h: Vec<&str> = it.next().unwrap().split_whitespace().collect();
        assert_eq!(h[0], tag);
        assert_eq!(h[1].parse::<usize>().unwrap(), i);
        let np: usize = h[2].parse().unwrap();
        let mut layer = Vec::with_capacity(np);
        for _ in 0..np {
            let t: Vec<&str> = it.next().unwrap().split_whitespace().collect();
            assert_eq!(t[0], "p");
            let n: usize = t[1].parse().unwrap();
            layer.push((0..n).map(|k| (t[2 + 2 * k].parse().unwrap(), t[3 + 2 * k].parse().unwrap())).collect());
        }
        out.push(layer);
    }
    out
}

#[test]
fn slice_mesh_slabs_native_parity() {
    let text = match std::env::var("TMS_VECTORS") {
        Ok(path) => std::fs::read_to_string(&path).expect("vectors"),
        Err(_) => {
            use std::io::Read;
            let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/triangle_mesh_slicer/vectors.txt.gz");
            let mut s = String::new();
            flate2::read::GzDecoder::new(std::fs::File::open(path).expect("vectors.txt.gz"))
                .read_to_string(&mut s)
                .expect("gzip");
            s
        }
    };
    let mut it = text.lines();
    let (mut cases, mut layers_eq, mut layers_ne, mut loops) = (0usize, 0usize, 0usize, 0usize);
    let mut bad: Vec<String> = Vec::new();
    while let Some(head) = it.next() {
        let h: Vec<&str> = head.split_whitespace().collect();
        assert_eq!(h[0], "case");
        let (nv, nt, nz): (usize, usize, usize) = (h[2].parse().unwrap(), h[3].parse().unwrap(), h[4].parse().unwrap());
        let mut its = indexed_triangle_set::default();
        for _ in 0..nv {
            let t: Vec<&str> = it.next().unwrap().split_whitespace().collect();
            its.vertices.push(StlVertex::new(hex(t[1]) as f32, hex(t[2]) as f32, hex(t[3]) as f32));
        }
        for _ in 0..nt {
            let t: Vec<&str> = it.next().unwrap().split_whitespace().collect();
            its.indices.push(StlTriangleVertexIndices::new(t[1].parse().unwrap(), t[2].parse().unwrap(), t[3].parse().unwrap()));
        }
        let zs: Vec<f32> = (0..nz).map(|_| hex(it.next().unwrap().split_whitespace().nth(1).unwrap()) as f32).collect();
        let want_top = read_layers(&mut it, "top", nz);
        let want_bot = read_layers(&mut it, "bot", nz);
        // transform_mesh_vertices_for_slicing, identity (TriangleMeshSlicer.cpp:1964-1970):
        // v.x() *= float(1. / SCALING_FACTOR), v.y() likewise, z unscaled.
        let s = (1.0 / slicer::libslic3r::SCALING_FACTOR) as f32;
        let tv: Vec<StlVertex> = its.vertices.iter().map(|v| StlVertex::new(v.x * s, v.y * s, v.z)).collect();
        let (top, bot) = slice_mesh_slabs(&its, &zs, &tv, false, true, true);
        for (tag, got, want) in [("top", &top, &want_top), ("bot", &bot, &want_bot)] {
            assert_eq!(got.len(), want.len(), "case {} {} layer count", h[1], tag);
            for (i, (g, w)) in got.iter().zip(want.iter()).enumerate() {
                let gp: Vec<Vec<(i64, i64)>> =
                    g.iter().map(|p| p.points.iter().map(|q| (q.x as i64, q.y as i64)).collect()).collect();
                loops += w.len();
                if &gp == w {
                    layers_eq += 1;
                } else {
                    layers_ne += 1;
                    if bad.len() < 8 {
                        let mut gs: Vec<(i64, i64)> = gp.iter().flatten().copied().collect();
                        let mut ws: Vec<(i64, i64)> = w.iter().flatten().copied().collect();
                        gs.sort();
                        ws.sort();
                        let lens = |v: &Vec<Vec<(i64, i64)>>| v.iter().map(|p| p.len()).collect::<Vec<_>>();
                        bad.push(format!(
                            "case {} {} layer {}: rust {} loops {:?}, pin {} loops {:?}, same point multiset {}; rust first {:?}, pin first {:?}",
                            h[1], tag, i, gp.len(), lens(&gp), w.len(), lens(w), gs == ws,
                            gp.first().map(|p| p.iter().take(3).collect::<Vec<_>>()),
                            w.first().map(|p| p.iter().take(3).collect::<Vec<_>>())
                        ));
                    }
                }
            }
        }
        cases += 1;
    }
    eprintln!("slice_mesh_slabs: {} cases, {} layers equal, {} differ, {} pin loops", cases, layers_eq, layers_ne, loops);
    for b in &bad {
        eprintln!("  {}", b);
    }
    assert!(cases > 0);
    assert_eq!(layers_ne, 0, "slab loops differ from the pin");
}
