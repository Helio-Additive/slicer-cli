//! admesh's whole import repair against the pin: `trianglemesh_repair_on_import` (TriangleMesh.cpp:102-197:
//! stl_check_facets_exact, stl_check_facets_nearby x2, stl_remove_unconnected_facets,
//! stl_fix_normal_directions, stl_fix_normal_values, stl_calculate_volume, stl_verify_neighbors, the second
//! exact check) and `TriangleMesh::from_stl` (TriangleMesh.cpp:204-235: stl_generate_shared_vertices and the
//! its_volume flip), on the binary STLs of `tests/data/stl_repair` (written by `mk_inputs.py`), vectors from
//! `gen.cpp` (gzip -9n). Required bit-equal: the stats after stl_open (stlinit.cpp:157-389) and after the repair, every
//! repaired facet (vertices and normal as float bits) and its neighbors, its_volume and the indexed set, and
//! the same indexed set from `stl::read_stl_file` (STL_ADMESH_REPAIR_PIN on).

use slicer::stl_repair::{StlFacet, StlFile, StlStats};

#[derive(Default)]
struct Case {
    name: String,
    s0: Vec<String>,
    s1: Vec<String>,
    r: Vec<String>,
    n: Vec<String>,
    vol: String,
    v: Vec<[u32; 3]>,
    f: Vec<[i32; 3]>,
    m: bool,
}

fn vectors() -> Vec<Case> {
    use std::io::Read;
    let path = format!("{}/tests/data/stl_repair/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    let mut out: Vec<Case> = Vec::new();
    for line in text.lines() {
        let t: Vec<&str> = line.split_whitespace().collect();
        let rest = || t[1..].iter().map(|s| s.to_string()).collect::<Vec<String>>();
        match t[0] {
            "FILE" => out.push(Case { name: t[1].to_string(), ..Default::default() }),
            "S0" => out.last_mut().unwrap().s0 = rest(),
            "S1" => out.last_mut().unwrap().s1 = rest(),
            "R" => out.last_mut().unwrap().r.push(t[1..].join(" ")),
            "N" => out.last_mut().unwrap().n.push(t[1..].join(" ")),
            "VOL" => out.last_mut().unwrap().vol = t[1].to_string(),
            "V" => out.last_mut().unwrap().v.push([1, 2, 3].map(|i| u32::from_str_radix(t[i], 16).unwrap())),
            "F" => out.last_mut().unwrap().f.push([1, 2, 3].map(|i| t[i].parse::<i32>().unwrap())),
            "M" => out.last_mut().unwrap().m = t[1] == "1",
            _ => panic!("bad line {line}"),
        }
    }
    out
}

fn stats_line(s: &StlStats) -> Vec<String> {
    let b = |f: f32| format!("{:08x}", f.to_bits());
    let mut v = vec![s.number_of_facets.to_string()];
    for k in 0..3 {
        v.push(b(s.min[k]));
    }
    for k in 0..3 {
        v.push(b(s.max[k]));
    }
    for k in 0..3 {
        v.push(b(s.size[k]));
    }
    v.push(b(s.bounding_diameter));
    v.push(b(s.shortest_edge));
    v.push(b(s.volume));
    for i in [
        s.connected_edges,
        s.connected_facets_1_edge,
        s.connected_facets_2_edge,
        s.connected_facets_3_edge,
        s.facets_w_1_bad_edge,
        s.facets_w_2_bad_edge,
        s.facets_w_3_bad_edge,
        s.edges_fixed,
        s.degenerate_facets,
        s.facets_removed,
        s.facets_added,
        s.facets_reversed,
        s.backwards_edges,
        s.normals_fixed,
        s.number_of_parts,
    ] {
        v.push(i.to_string());
    }
    v
}

// stlinit.cpp:53-152 (binary branch: facet count from the file size) and 249-330.
fn read_binary(path: &str) -> Vec<Option<StlFacet>> {
    let data = std::fs::read(path).unwrap();
    let n = (data.len() - 84) / 50;
    let f = |o: usize| f32::from_le_bytes([data[o], data[o + 1], data[o + 2], data[o + 3]]);
    let v = |o: usize| [f(o), f(o + 4), f(o + 8)];
    (0..n)
        .map(|i| {
            let o = 84 + i * 50;
            let facet = StlFacet::new(v(o), v(o + 12), v(o + 24), v(o + 36));
            if facet.vertex.iter().any(|p| p.iter().any(|c| c.is_nan())) {
                None
            } else {
                Some(facet)
            }
        })
        .collect()
}

#[test]
fn stl_repair_matches_admesh() {
    let dir = format!("{}/tests/data/stl_repair", env!("CARGO_MANIFEST_DIR"));
    let cases = vectors();
    assert_eq!(cases.len(), 10);
    let mut mismatches = 0;
    let mut checked = 0;
    for c in &cases {
        assert!(c.m, "{}: the generator's step-by-step repair differs from ReadSTLFile", c.name);
        let path = format!("{dir}/{}", c.name);
        let mut stl = StlFile::from_read_facets(read_binary(&path));
        let mut bad = |what: &str, ok: bool| {
            checked += 1;
            if !ok {
                eprintln!("{}: {what} differs", c.name);
                mismatches += 1;
            }
        };
        let s0 = stats_line(&stl.stats);
        bad("stats after stl_open", s0 == c.s0);
        slicer::stl_repair::trianglemesh_repair_on_import(&mut stl);
        let s1 = stats_line(&stl.stats);
        if s1 != c.s1 {
            eprintln!("  port {:?}\n  pin  {:?}", s1, c.s1);
        }
        bad("stats after the repair", s1 == c.s1);
        let r: Vec<String> = stl.facet_start[..stl.stats.number_of_facets as usize]
            .iter()
            .map(|f| {
                let mut w = Vec::new();
                for k in 0..3 {
                    for j in 0..3 {
                        w.push(format!("{:08x}", f.vertex[k][j].to_bits()));
                    }
                }
                for j in 0..3 {
                    w.push(format!("{:08x}", f.normal[j].to_bits()));
                }
                w.join(" ")
            })
            .collect();
        bad("repaired facets", r == c.r);
        let n: Vec<String> = stl.neighbors_start[..stl.stats.number_of_facets as usize]
            .iter()
            .map(|n| {
                format!(
                    "{} {} {} {} {} {}",
                    n.neighbor[0], n.neighbor[1], n.neighbor[2], n.which_vertex_not[0], n.which_vertex_not[1],
                    n.which_vertex_not[2]
                )
            })
            .collect();
        bad("neighbors", n == c.n);
        let (sv, si) = slicer::stl_repair::stl_generate_shared_vertices(&stl);
        let its = slicer::normal_utils::indexed_triangle_set {
            vertices: sv.iter().map(|v| slicer::normal_utils::Vec3f::new(v[0], v[1], v[2])).collect(),
            indices: si.iter().map(|t| slicer::normal_utils::Vec3crd::new(t[0], t[1], t[2])).collect(),
        };
        let vol = slicer::triangle_mesh::its_volume(&its);
        bad("its_volume", format!("{:08x}", vol.to_bits()) == c.vol);
        let (verts, idx) = slicer::stl_repair::from_stl(&mut stl, false);
        let vb: Vec<[u32; 3]> =
            verts.iter().map(|p| [(p.x as f32).to_bits(), (p.y as f32).to_bits(), (p.z as f32).to_bits()]).collect();
        let fb: Vec<[i32; 3]> = idx.iter().map(|t| [t[0] as i32, t[1] as i32, t[2] as i32]).collect();
        bad("shared vertices", vb == c.v);
        bad("triangles", fb == c.f);
        // The reader route (stl.rs, STL_ADMESH_REPAIR_PIN on).
        let mesh = slicer::stl::read_stl_file(std::path::Path::new(&path)).unwrap();
        let mv: Vec<[u32; 3]> = mesh
            .vertices()
            .iter()
            .map(|p| [(p.x as f32).to_bits(), (p.y as f32).to_bits(), (p.z as f32).to_bits()])
            .collect();
        let mf: Vec<[i32; 3]> = mesh
            .indices()
            .iter()
            .map(|t| [t.indices[0] as i32, t.indices[1] as i32, t.indices[2] as i32])
            .collect();
        bad("read_stl_file vertices", mv == c.v);
        bad("read_stl_file triangles", mf == c.f);
    }
    eprintln!("{} files, {} comparisons, {} mismatches", cases.len(), checked, mismatches);
    assert_eq!(mismatches, 0);
}
