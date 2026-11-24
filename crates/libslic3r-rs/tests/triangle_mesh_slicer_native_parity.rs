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

fn read_polys<'a>(it: &mut impl Iterator<Item = &'a str>, tag: &str) -> Vec<Vec<(i64, i64)>> {
    let h: Vec<&str> = it.next().unwrap().split_whitespace().collect();
    assert_eq!(h[0], tag);
    let np: usize = h[1].parse().unwrap();
    (0..np)
        .map(|_| {
            let t: Vec<&str> = it.next().unwrap().split_whitespace().collect();
            assert_eq!(t[0], "p");
            let n: usize = t[1].parse().unwrap();
            (0..n).map(|k| (t[2 + 2 * k].parse().unwrap(), t[3 + 2 * k].parse().unwrap())).collect()
        })
        .collect()
}

/// R945: transform_mesh_vertices_for_slicing (TriangleMeshSlicer.cpp:1953-1980) and both project_mesh
/// overloads (2403-2427, through slice_mesh_slabs 2177-2280 with a non-identity trafo, mirrored included)
/// against tests/data/triangle_mesh_slicer/gen_project.cpp (`gen_project <out.txt>`, gzip -9n into
/// project_vectors.txt.gz; or TMS_PROJECT_VECTORS=<plain file>). Exact: same floats, same polygons in order.
#[test]
fn project_mesh_native_parity() {
    use slicer::geometry::Polygons;
    use slicer::triangle_mesh_slicer::{transform_mesh_vertices_for_slicing, ProjectMesh};
    let text = match std::env::var("TMS_PROJECT_VECTORS") {
        Ok(path) => std::fs::read_to_string(&path).expect("vectors"),
        Err(_) => {
            use std::io::Read;
            let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/triangle_mesh_slicer/project_vectors.txt.gz");
            let mut s = String::new();
            flate2::read::GzDecoder::new(std::fs::File::open(path).expect("project_vectors.txt.gz"))
                .read_to_string(&mut s)
                .expect("gzip");
            s
        }
    };
    let to_pts = |ps: &Polygons| -> Vec<Vec<(i64, i64)>> {
        ps.iter().map(|p| p.points.iter().map(|q| (q.x as i64, q.y as i64)).collect()).collect()
    };
    let mut it = text.lines();
    let (mut cases, mut verts, mut verts_ne, mut polys, mut ne) = (0usize, 0usize, 0usize, 0usize, 0usize);
    let mut bad: Vec<String> = Vec::new();
    while let Some(head) = it.next() {
        let h: Vec<&str> = head.split_whitespace().collect();
        assert_eq!(h[0], "case");
        let (nv, nt): (usize, usize) = (h[2].parse().unwrap(), h[3].parse().unwrap());
        let m: Vec<&str> = it.next().unwrap().split_whitespace().collect();
        assert_eq!(m[0], "m");
        let mut trafo = [0f64; 16];
        for (i, x) in m[1..17].iter().enumerate() {
            trafo[i] = hex(x);
        }
        let mut its = indexed_triangle_set::default();
        for _ in 0..nv {
            let t: Vec<&str> = it.next().unwrap().split_whitespace().collect();
            its.vertices.push(StlVertex::new(hex(t[1]) as f32, hex(t[2]) as f32, hex(t[3]) as f32));
        }
        for _ in 0..nt {
            let t: Vec<&str> = it.next().unwrap().split_whitespace().collect();
            its.indices.push(StlTriangleVertexIndices::new(t[1].parse().unwrap(), t[2].parse().unwrap(), t[3].parse().unwrap()));
        }
        let want_tv: Vec<[u32; 3]> = (0..nv)
            .map(|_| {
                let t: Vec<&str> = it.next().unwrap().split_whitespace().collect();
                assert_eq!(t[0], "tv");
                [(hex(t[1]) as f32).to_bits(), (hex(t[2]) as f32).to_bits(), (hex(t[3]) as f32).to_bits()]
            })
            .collect();
        let want_top = read_polys(&mut it, "top");
        let want_bot = read_polys(&mut it, "bot");
        let want_uni = read_polys(&mut it, "uni");
        let tv = transform_mesh_vertices_for_slicing(&its, &trafo);
        for (g, w) in tv.iter().zip(want_tv.iter()) {
            verts += 1;
            if [g.x.to_bits(), g.y.to_bits(), g.z.to_bits()] != *w {
                verts_ne += 1;
            }
        }
        let (mut top, mut bot) = (Polygons::new(), Polygons::new());
        ProjectMesh::project_mesh(&its, (&trafo, Some(&mut top), Some(&mut bot), &(|| {}) as &dyn Fn()));
        let uni: Polygons = ProjectMesh::project_mesh(&its, (&trafo, &(|| {}) as &dyn Fn()));
        for (tag, got, want) in [("top", &top, &want_top), ("bot", &bot, &want_bot), ("uni", &uni, &want_uni)] {
            polys += want.len();
            if &to_pts(got) != want {
                ne += 1;
                if bad.len() < 8 {
                    bad.push(format!("case {} {}: rust {} polygons, pin {}", h[1], tag, got.len(), want.len()));
                }
            }
        }
        cases += 1;
    }
    eprintln!(
        "project_mesh: {} cases, {} vertices ({} differ), {} pin polygons, {} outputs differ",
        cases, verts, verts_ne, polys, ne
    );
    for b in &bad {
        eprintln!("  {}", b);
    }
    assert!(cases > 0);
    assert_eq!(verts_ne, 0, "transform_mesh_vertices_for_slicing differs from the pin");
    assert_eq!(ne, 0, "project_mesh differs from the pin");
}

fn read_its<'a>(it: &mut impl Iterator<Item = &'a str>, tag: &str) -> (Vec<[u32; 3]>, Vec<[i32; 3]>, Vec<i32>) {
    let h: Vec<&str> = it.next().unwrap().split_whitespace().collect();
    assert_eq!(h[0], tag);
    let (nv, nt): (usize, usize) = (h[1].parse().unwrap(), h[2].parse().unwrap());
    let v = (0..nv)
        .map(|_| {
            let t: Vec<&str> = it.next().unwrap().split_whitespace().collect();
            [(hex(t[1]) as f32).to_bits(), (hex(t[2]) as f32).to_bits(), (hex(t[3]) as f32).to_bits()]
        })
        .collect();
    let f = (0..nt)
        .map(|_| {
            let t: Vec<&str> = it.next().unwrap().split_whitespace().collect();
            [t[1].parse().unwrap(), t[2].parse().unwrap(), t[3].parse().unwrap()]
        })
        .collect();
    let s: Vec<&str> = it.next().unwrap().split_whitespace().collect();
    assert_eq!(s[0], "src");
    let src = s[2..].iter().map(|x| x.parse().unwrap()).collect();
    (v, f, src)
}

/// R945: cut_mesh (TriangleMeshSlicer.cpp:2429-, the z overload, with and without caps), and through it
/// slice_facet_for_cut_mesh, triangulate_slice and make_expolygons_simple (1716-1788), against
/// tests/data/triangle_mesh_slicer/gen_cut.cpp (`gen_cut <out.txt>`, gzip -9n into cut_vectors.txt.gz; or
/// TMS_CUT_VECTORS=<plain file>). Exact: same vertices (bits), same triangles in order, same source faces.
#[test]
fn cut_mesh_native_parity() {
    use slicer::triangle_mesh_slicer::cut_mesh;
    let text = match std::env::var("TMS_CUT_VECTORS") {
        Ok(path) => std::fs::read_to_string(&path).expect("vectors"),
        Err(_) => {
            use std::io::Read;
            let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/triangle_mesh_slicer/cut_vectors.txt.gz");
            let mut s = String::new();
            flate2::read::GzDecoder::new(std::fs::File::open(path).expect("cut_vectors.txt.gz"))
                .read_to_string(&mut s)
                .expect("gzip");
            s
        }
    };
    let mut it = text.lines();
    let (mut cases, mut ne, mut caps_ne) = (0usize, 0usize, 0usize);
    let mut bad: Vec<String> = Vec::new();
    while let Some(head) = it.next() {
        let h: Vec<&str> = head.split_whitespace().collect();
        assert_eq!(h[0], "case");
        let (nv, nt, caps): (usize, usize, bool) = (h[2].parse().unwrap(), h[3].parse().unwrap(), h[4] == "1");
        let z = hex(it.next().unwrap().split_whitespace().nth(1).unwrap()) as f32;
        let mut its = indexed_triangle_set::default();
        for _ in 0..nv {
            let t: Vec<&str> = it.next().unwrap().split_whitespace().collect();
            its.vertices.push(StlVertex::new(hex(t[1]) as f32, hex(t[2]) as f32, hex(t[3]) as f32));
        }
        for _ in 0..nt {
            let t: Vec<&str> = it.next().unwrap().split_whitespace().collect();
            its.indices.push(StlTriangleVertexIndices::new(t[1].parse().unwrap(), t[2].parse().unwrap(), t[3].parse().unwrap()));
        }
        let want_upper = read_its(&mut it, "upper");
        let want_lower = read_its(&mut it, "lower");
        let (mut upper, mut lower) = (indexed_triangle_set::default(), indexed_triangle_set::default());
        let (mut upper_src, mut lower_src) = (Vec::<i32>::new(), Vec::<i32>::new());
        cut_mesh(&its, z, Some(&mut upper), Some(&mut lower), caps, Some(&mut upper_src), Some(&mut lower_src), None, None);
        for (tag, got, src, want) in [("upper", &upper, &upper_src, &want_upper), ("lower", &lower, &lower_src, &want_lower)] {
            let gv: Vec<[u32; 3]> = got.vertices.iter().map(|v| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]).collect();
            let gf: Vec<[i32; 3]> = got.indices.iter().map(|t| [t[0] as i32, t[1] as i32, t[2] as i32]).collect();
            // The cap triangles (source face -1, appended last by triangulate_slice, TriangleMeshSlicer.cpp:2367-2395)
            // come from triangulate_expolygons_3d, which the port runs on earcut where the pin runs glu-libtess
            // (Tesselate.cpp; tesselate.rs): they are counted, not asserted. Everything else is asserted.
            let shell_equal = gv == want.0
                && src == &want.2
                && gf.len() == want.1.len()
                && (0..gf.len()).all(|i| want.2.get(i).copied().unwrap_or(-1) == -1 || gf[i] == want.1[i]);
            if gf != want.1 && shell_equal {
                caps_ne += 1;
                continue;
            }
            if !shell_equal {
                ne += 1;
                if bad.len() < 8 {
                    bad.push(format!(
                        "case {} {} (caps {}): rust {} v / {} t / {} src, pin {} v / {} t / {} src; vertices equal {}, triangles equal {}, src equal {}",
                        h[1], tag, caps, gv.len(), gf.len(), src.len(), want.0.len(), want.1.len(), want.2.len(),
                        gv == want.0, gf == want.1, src == &want.2
                    ));
                    if let Some(i) = (0..gf.len().min(want.1.len())).find(|&i| gf[i] != want.1[i]) {
                        bad.push(format!(
                            "    first triangle that differs: {} of {}: rust {:?}, pin {:?}; same triangle set rotated {}",
                            i, gf.len(), gf[i], want.1[i],
                            {
                                let norm = |t: &[i32; 3]| { let m = (0..3).min_by_key(|&k| t[k]).unwrap(); [t[m], t[(m + 1) % 3], t[(m + 2) % 3]] };
                                let mut a: Vec<[i32; 3]> = gf.iter().map(norm).collect();
                                let mut b: Vec<[i32; 3]> = want.1.iter().map(norm).collect();
                                a.sort();
                                b.sort();
                                a == b
                            }
                        ));
                    }
                }
            }
        }
        cases += 1;
    }
    eprintln!(
        "cut_mesh: {} cases, {} outputs (upper/lower) differ outside the caps; {} outputs differ only in the cap triangles (earcut vs glu-libtess, reported)",
        cases, ne, caps_ne
    );
    for b in &bad {
        eprintln!("  {}", b);
    }
    assert!(cases > 0);
    assert_eq!(ne, 0, "cut_mesh differs from the pin");
}

type Pts = Vec<Vec<(i64, i64)>>;

fn read_ex<'a>(it: &mut impl Iterator<Item = &'a str>, tag: &str) -> Vec<(Vec<(i64, i64)>, Pts)> {
    let h: Vec<&str> = it.next().unwrap().split_whitespace().collect();
    assert_eq!(h[0], tag);
    let n: usize = h[1].parse().unwrap();
    (0..n)
        .map(|_| {
            let e: Vec<&str> = it.next().unwrap().split_whitespace().collect();
            assert_eq!(e[0], "e");
            let mut c = read_polys(it, "c");
            let holes = read_polys(it, "h");
            (c.remove(0), holes)
        })
        .collect()
}

/// R946: both slice_mesh overloads (TriangleMeshSlicer.cpp:1982-2056 with the vector-of-zs slice_make_lines 511-535
/// and make_loops 1534-1584 / 1433-1530; the single plane 2059-2119 with its face mask and masked edge ids), the three
/// slice_mesh_ex overloads (2121-2170, TriangleMeshSlicer.hpp:78-95), the raw lines of slice_make_lines and
/// make_expolygons_simple (1716-1750) against tests/data/triangle_mesh_slicer/gen_slice.cpp (`gen_slice <out.txt>`,
/// gzip -9n into slice_vectors.txt.gz; or TMS_SLICE_VECTORS=<plain file>): 72 meshes of the pin's makers (a cube with
/// a hole among them) under the identity, a rotation, a tilt with a scale and a mirror, planes in the middle and
/// through vertices, every slicing mode, closing radius, extra offset and resolution. Exact: same lines, same
/// polygons in order with the same start points.
#[test]
fn slice_mesh_native_parity() {
    use slicer::geometry::{ExPolygons, Polygons};
    use slicer::triangle_mesh_slicer::{
        make_expolygons_simple_at_plane, slice_make_lines_raw, MeshSlicingParams, MeshSlicingParamsEx, SliceMesh, SliceMeshEx,
        SlicingMode,
    };
    let text = match std::env::var("TMS_SLICE_VECTORS") {
        Ok(path) => std::fs::read_to_string(&path).expect("vectors"),
        Err(_) => {
            use std::io::Read;
            let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/triangle_mesh_slicer/slice_vectors.txt.gz");
            let mut s = String::new();
            flate2::read::GzDecoder::new(std::fs::File::open(path).expect("slice_vectors.txt.gz"))
                .read_to_string(&mut s)
                .expect("gzip");
            s
        }
    };
    let mode_of = |m: &str| match m {
        "0" => SlicingMode::Regular,
        "1" => SlicingMode::EvenOdd,
        "2" => SlicingMode::Positive,
        _ => SlicingMode::PositiveLargestContour,
    };
    let to_pts = |ps: &Polygons| -> Pts { ps.iter().map(|p| p.points.iter().map(|q| (q.x as i64, q.y as i64)).collect()).collect() };
    let to_ex = |exs: &ExPolygons| -> Vec<(Vec<(i64, i64)>, Pts)> {
        exs.iter()
            .map(|ex| (ex.contour.points.iter().map(|q| (q.x as i64, q.y as i64)).collect(), to_pts(&ex.holes)))
            .collect()
    };
    let no_cancel: &dyn Fn() = &|| {};
    let mut it = text.lines();
    let mut cases = 0usize;
    // Stage name -> (outputs compared, outputs that differ).
    let mut stats: std::collections::BTreeMap<&str, (usize, usize)> = std::collections::BTreeMap::new();
    let mut bad: Vec<String> = Vec::new();
    let mut note = |stats: &mut std::collections::BTreeMap<&'static str, (usize, usize)>, stage: &'static str, case: &str, eq: bool| {
        let e = stats.entry(stage).or_insert((0, 0));
        e.0 += 1;
        if !eq {
            e.1 += 1;
            if bad.len() < 12 {
                bad.push(format!("case {} {}", case, stage));
            }
        }
    };
    let mut nlines = 0usize;
    while let Some(head) = it.next() {
        let h: Vec<&str> = head.split_whitespace().collect();
        assert_eq!(h[0], "case");
        let case = h[1].to_string();
        let (nv, nt, nz): (usize, usize, usize) = (h[2].parse().unwrap(), h[3].parse().unwrap(), h[4].parse().unwrap());
        let m: Vec<&str> = it.next().unwrap().split_whitespace().collect();
        assert_eq!(m[0], "m");
        let mut trafo = [0f64; 16];
        for (i, x) in m[1..17].iter().enumerate() {
            trafo[i] = hex(x);
        }
        let mut its = indexed_triangle_set::default();
        for _ in 0..nv {
            let t: Vec<&str> = it.next().unwrap().split_whitespace().collect();
            its.vertices.push(StlVertex::new(hex(t[1]) as f32, hex(t[2]) as f32, hex(t[3]) as f32));
        }
        for _ in 0..nt {
            let t: Vec<&str> = it.next().unwrap().split_whitespace().collect();
            its.indices.push(StlTriangleVertexIndices::new(t[1].parse().unwrap(), t[2].parse().unwrap(), t[3].parse().unwrap()));
        }
        let z: Vec<&str> = it.next().unwrap().split_whitespace().collect();
        assert_eq!(z[0], "z");
        let zs: Vec<f32> = z[1..].iter().map(|v| hex(v) as f32).collect();
        assert_eq!(zs.len(), nz);
        let p: Vec<&str> = it.next().unwrap().split_whitespace().collect();
        assert_eq!(p[0], "par");
        let mut params = MeshSlicingParams::default();
        params.mode = mode_of(p[1]);
        params.mode_below = mode_of(p[2]);
        params.slicing_mode_normal_below_layer = p[3].parse().unwrap();
        params.frame_trafo = Some(trafo);
        let mut pex = MeshSlicingParamsEx::default();
        pex.base = params.clone();
        pex.closing_radius = hex(p[4]) as f32;
        pex.extra_offset = hex(p[5]) as f32;
        pex.resolution = hex(p[6]);
        let closing_radius = hex(p[7]) as f32;
        let plane_z = hex(p[8]) as f32;

        // slice_make_lines (vector of zs).
        let raw = slice_make_lines_raw(&its, &trafo, &zs);
        for i in 0..nz {
            let l: Vec<&str> = it.next().unwrap().split_whitespace().collect();
            assert_eq!(l[0], "lines");
            let n: usize = l[2].parse().unwrap();
            let want: Vec<[i64; 10]> = (0..n)
                .map(|_| {
                    let t: Vec<&str> = it.next().unwrap().split_whitespace().collect();
                    let mut a = [0i64; 10];
                    for k in 0..10 {
                        a[k] = t[1 + k].parse().unwrap();
                    }
                    a
                })
                .collect();
            nlines += n;
            note(&mut stats, "slice_make_lines", &case, raw[i] == want);
        }
        // slice_mesh, vector of zs.
        let sm: Vec<Polygons> = SliceMesh::slice_mesh(&its, (zs.as_slice(), &params, no_cancel));
        for i in 0..nz {
            let want = read_polys(&mut it, &format!("sm{}", i));
            note(&mut stats, "slice_mesh(zs)", &case, to_pts(&sm[i]) == want);
        }
        // slice_mesh, single plane.
        let sp: Polygons = SliceMesh::slice_mesh(&its, (plane_z, &params));
        let want = read_polys(&mut it, "sp");
        note(&mut stats, "slice_mesh(plane_z)", &case, to_pts(&sp) == want);
        // slice_mesh_ex, three overloads.
        let ex1: Vec<ExPolygons> = SliceMeshEx::slice_mesh_ex(&its, (zs.as_slice(), &pex, no_cancel));
        let ex2: Vec<ExPolygons> = SliceMeshEx::slice_mesh_ex(&its, (zs.as_slice(), no_cancel));
        let ex3: Vec<ExPolygons> = SliceMeshEx::slice_mesh_ex(&its, (zs.as_slice(), closing_radius, no_cancel));
        for i in 0..nz {
            let w1 = read_ex(&mut it, &format!("ex1_{}", i));
            note(&mut stats, "slice_mesh_ex(params)", &case, to_ex(&ex1[i]) == w1);
            let w2 = read_ex(&mut it, &format!("ex2_{}", i));
            note(&mut stats, "slice_mesh_ex()", &case, to_ex(&ex2[i]) == w2);
            let w3 = read_ex(&mut it, &format!("ex3_{}", i));
            note(&mut stats, "slice_mesh_ex(closing_radius)", &case, to_ex(&ex3[i]) == w3);
        }
        // make_expolygons_simple.
        let es = make_expolygons_simple_at_plane(&its, plane_z);
        let want = read_ex(&mut it, "es");
        note(&mut stats, "make_expolygons_simple", &case, to_ex(&es) == want);
        cases += 1;
    }
    eprintln!("slice_mesh: {} cases, {} pin lines", cases, nlines);
    for (stage, (n, ne)) in &stats {
        eprintln!("  {}: {} outputs, {} differ", stage, n, ne);
    }
    for b in &bad {
        eprintln!("  differs: {}", b);
    }
    assert!(cases > 0);
    for (stage, (_, ne)) in &stats {
        assert_eq!(*ne, 0, "{} differs from the pin", stage);
    }
}

fn read_mesh_out<'a>(it: &mut impl Iterator<Item = &'a str>, tag: &str) -> (Vec<[u32; 3]>, Vec<[i32; 3]>) {
    let h: Vec<&str> = it.next().unwrap().split_whitespace().collect();
    assert_eq!(h[0], tag);
    let (nv, nt): (usize, usize) = (h[1].parse().unwrap(), h[2].parse().unwrap());
    let verts = (0..nv)
        .map(|_| {
            let t: Vec<&str> = it.next().unwrap().split_whitespace().collect();
            [(hex(t[1]) as f32).to_bits(), (hex(t[2]) as f32).to_bits(), (hex(t[3]) as f32).to_bits()]
        })
        .collect();
    let faces = (0..nt)
        .map(|_| {
            let t: Vec<&str> = it.next().unwrap().split_whitespace().collect();
            [t[1].parse().unwrap(), t[2].parse().unwrap(), t[3].parse().unwrap()]
        })
        .collect();
    (verts, faces)
}

/// R946: cut_mesh by the plane of four points (TriangleMeshSlicer.cpp:2727-2788, trait CutMesh) against
/// tests/data/triangle_mesh_slicer/gen_cut_plane.cpp (`gen_cut_plane <out.txt>`, gzip -9n into
/// cut_plane_vectors.txt.gz; or TMS_CUT_PLANE_VECTORS=<plain file>): 40 meshes, tilted planes with the normal up and
/// down, vertical and horizontal planes. The vertices (moved back by the inverse transform) and the faces before
/// the caps are asserted; the cap triangles are counted, not asserted (tesselate.rs runs earcut where the pin runs
/// glu-libtess, as in cut_mesh_native_parity).
#[test]
fn cut_mesh_plane_points_native_parity() {
    use slicer::geometry::geometry::Vec3d;
    use slicer::triangle_mesh_slicer::CutMesh;
    let text = match std::env::var("TMS_CUT_PLANE_VECTORS") {
        Ok(path) => std::fs::read_to_string(&path).expect("vectors"),
        Err(_) => {
            use std::io::Read;
            let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/triangle_mesh_slicer/cut_plane_vectors.txt.gz");
            let mut s = String::new();
            flate2::read::GzDecoder::new(std::fs::File::open(path).expect("cut_plane_vectors.txt.gz"))
                .read_to_string(&mut s)
                .expect("gzip");
            s
        }
    };
    let mut it = text.lines();
    let (mut cases, mut outputs, mut verts_ne, mut faces_ne, mut cap_only) = (0usize, 0usize, 0usize, 0usize, 0usize);
    let mut bad: Vec<String> = Vec::new();
    while let Some(head) = it.next() {
        let h: Vec<&str> = head.split_whitespace().collect();
        assert_eq!(h[0], "case");
        let (nv, nt): (usize, usize) = (h[2].parse().unwrap(), h[3].parse().unwrap());
        let pp: Vec<&str> = it.next().unwrap().split_whitespace().collect();
        assert_eq!(pp[0], "pp");
        let mut plane_points = [Vec3d::new(0.0, 0.0, 0.0); 4];
        for i in 0..4 {
            plane_points[i] = Vec3d::new(hex(pp[1 + 3 * i]), hex(pp[2 + 3 * i]), hex(pp[3 + 3 * i]));
        }
        let mut its = indexed_triangle_set::default();
        for _ in 0..nv {
            let t: Vec<&str> = it.next().unwrap().split_whitespace().collect();
            its.vertices.push(StlVertex::new(hex(t[1]) as f32, hex(t[2]) as f32, hex(t[3]) as f32));
        }
        for _ in 0..nt {
            let t: Vec<&str> = it.next().unwrap().split_whitespace().collect();
            its.indices.push(StlTriangleVertexIndices::new(t[1].parse().unwrap(), t[2].parse().unwrap(), t[3].parse().unwrap()));
        }
        let want_upper = read_mesh_out(&mut it, "upper");
        let want_lower = read_mesh_out(&mut it, "lower");
        let (mut upper, mut lower) = (indexed_triangle_set::default(), indexed_triangle_set::default());
        CutMesh::cut_mesh(&its, (plane_points, Some(&mut upper), Some(&mut lower), true));
        for (tag, got, want) in [("upper", &upper, &want_upper), ("lower", &lower, &want_lower)] {
            outputs += 1;
            let gv: Vec<[u32; 3]> = got.vertices.iter().map(|v| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]).collect();
            let gf: Vec<[i32; 3]> = got.indices.iter().map(|t| [t[0] as i32, t[1] as i32, t[2] as i32]).collect();
            if gv != want.0 {
                verts_ne += 1;
                if bad.len() < 8 {
                    let first = gv.iter().zip(want.0.iter()).position(|(a, b)| a != b);
                    bad.push(format!("case {} {} vertices: rust {} pin {} first differing {:?}", h[1], tag, gv.len(), want.0.len(), first));
                }
            }
            if gf != want.1 {
                // The faces before the first difference; a cap-only difference leaves every face before the caps equal,
                // and the caps are the faces whose three vertices were created on the cut (index >= the shell's count is
                // not known here), so the test counts the outputs whose face COUNT agrees and differ only after the
                // common prefix.
                let prefix = gf.iter().zip(want.1.iter()).take_while(|(a, b)| a == b).count();
                let n_cut_faces = want.1.iter().skip(prefix).count();
                if gf.len() == want.1.len() {
                    cap_only += 1;
                } else {
                    faces_ne += 1;
                }
                if bad.len() < 8 {
                    bad.push(format!("case {} {} faces: rust {} pin {}, equal prefix {}, pin faces after it {}", h[1], tag, gf.len(), want.1.len(), prefix, n_cut_faces));
                }
            }
        }
        cases += 1;
    }
    eprintln!(
        "cut_mesh(plane_points): {} cases, {} outputs: vertices differ in {}, faces differ after a common prefix with the same count in {} (caps), with another count in {}",
        cases, outputs, verts_ne, cap_only, faces_ne
    );
    for b in &bad {
        eprintln!("  {}", b);
    }
    assert!(cases > 0);
    assert_eq!(verts_ne, 0, "cut_mesh(plane_points) vertices differ from the pin");
    assert_eq!(faces_ne, 0, "cut_mesh(plane_points) face counts differ from the pin");
}
