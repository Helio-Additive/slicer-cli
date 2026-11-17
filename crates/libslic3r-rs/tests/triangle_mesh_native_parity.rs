//! `slicer::triangle_mesh` (the port of libslic3r/TriangleMesh.hpp and TriangleMesh.cpp) and the
//! `TriangleMesh` class as the crate carries it (`slicer::model::ModelMesh`) against the C++ at
//! the pin.
//!
//! `tests/data/triangle_mesh/vectors.txt.gz` is the output of `gen.cpp` beside it, which links
//! the pin's TriangleMesh.cpp and runs, over nine synthetic meshes (closed, open, in two parts, a
//! soup with degenerate faces, with a reversed face, with a non-manifold edge, inside out), the
//! statistics of the constructor, the `its_*` helpers, the STL and OBJ writers, the
//! transformations and the queries of the class, and `its_make_groove_plane`; the header of
//! gen.cpp describes the lines. The same calls run here and have to print the same lines: floats
//! and doubles bit for bit, indices, scaled points, files byte for byte (compared by length and
//! hash).
//!
//! The mesh makers have their own test, `triangle_mesh_makers_native_parity.rs`.

use nalgebra::{Matrix3, Matrix4};
use slicer::bounding_box::BoundingBoxf3;
use slicer::geometry::geometry::Axis;
use slicer::geometry::Point;
use slicer::model::ModelMesh;
use slicer::normal_utils::indexed_triangle_set;
use slicer::triangle_mesh::*;
use std::collections::{BTreeMap, HashMap};
use std::io::Read;

type Vec3d = nalgebra::Vector3<f64>;

fn read_vectors() -> String {
    let path = format!("{}/tests/data/triangle_mesh/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
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
fn hd(v: f64) -> String {
    format!("{:016x}", v.to_bits())
}
fn hf(v: f32) -> String {
    format!("{:08x}", v.to_bits())
}
fn v3f(v: &Vec3f) -> String {
    format!("{} {} {}", hf(v[0]), hf(v[1]), hf(v[2]))
}
fn v3i(v: &Vec3i) -> String {
    format!("{} {} {}", v[0], v[1], v[2])
}
fn b(v: bool) -> &'static str {
    if v {
        "1"
    } else {
        "0"
    }
}

fn its_text(its: &indexed_triangle_set) -> String {
    let mut s = format!("{} {}", its.vertices.len(), its.indices.len());
    for v in &its.vertices {
        s += " ";
        s += &v3f(v);
    }
    for t in &its.indices {
        s += " ";
        s += &v3i(t);
    }
    s
}

fn parse_its(tok: &[&str]) -> indexed_triangle_set {
    let nv: usize = tok[0].parse().unwrap();
    let nf: usize = tok[1].parse().unwrap();
    let mut its = indexed_triangle_set::default();
    let mut k = 2;
    for _ in 0..nv {
        its.vertices.push(Vec3f::new(f(tok[k]), f(tok[k + 1]), f(tok[k + 2])));
        k += 3;
    }
    for _ in 0..nf {
        its.indices.push(Vec3i::new(tok[k].parse().unwrap(), tok[k + 1].parse().unwrap(), tok[k + 2].parse().unwrap()));
        k += 3;
    }
    assert_eq!(k, tok.len());
    its
}

fn st(s: &TriangleMeshStats) -> String {
    format!(
        "{} {} {} {} {} {} {} {} {} {} {} {} {} {} {}",
        s.number_of_facets,
        v3f(&s.max),
        v3f(&s.min),
        v3f(&s.size),
        hf(s.volume),
        s.number_of_parts,
        s.open_edges,
        s.non_manifold_edges,
        s.non_manifold_vertices,
        b(s.has_reversed_faces),
        s.repaired_errors.edges_fixed,
        s.repaired_errors.degenerate_facets,
        s.repaired_errors.facets_removed,
        s.repaired_errors.facets_reversed,
        s.repaired_errors.backwards_edges
    )
}

fn mesh(m: &ModelMesh) -> String {
    format!("{} | {}", st(m.stats()), its_text(&m.its))
}

fn v3is(v: &[Vec3i]) -> String {
    let mut s = v.len().to_string();
    for t in v {
        s += " ";
        s += &v3i(t);
    }
    s
}

fn pts(p: &[Point]) -> String {
    let mut s = p.len().to_string();
    for q in p {
        s += &format!(" {} {}", q.x, q.y);
    }
    s
}

fn bb(x: &BoundingBoxf3) -> String {
    format!("{} {} {} {} {} {} {}", b(x.defined), hd(x.min.x), hd(x.min.y), hd(x.min.z), hd(x.max.x), hd(x.max.y), hd(x.max.z))
}

fn fileh(path: &str) -> String {
    let data = std::fs::read(path).unwrap_or_default();
    let mut h: u64 = 1469598103934665603;
    for c in &data {
        h ^= *c as u64;
        h = h.wrapping_mul(1099511628211);
    }
    format!("{} {:016x}", data.len(), h)
}

fn mat3f(a: &[&str]) -> Matrix3<f32> {
    Matrix3::from_fn(|i, j| f(a[i * 3 + j]))
}
fn mat34f(a: &[&str]) -> Matrix4<f32> {
    Matrix4::from_fn(|i, j| if i < 3 { f(a[i * 4 + j]) } else if j == 3 { 1.0 } else { 0.0 })
}
fn mat3d(a: &[&str]) -> Matrix3<f64> {
    Matrix3::from_fn(|i, j| d(a[i * 3 + j]))
}
fn mat34d(a: &[&str]) -> Matrix4<f64> {
    Matrix4::from_fn(|i, j| if i < 3 { d(a[i * 4 + j]) } else if j == 3 { 1.0 } else { 0.0 })
}
fn axis(a: &str) -> Axis {
    match a {
        "0" => Axis::X,
        "1" => Axis::Y,
        _ => Axis::Z,
    }
}

#[test]
fn triangle_mesh_matches_the_pin() {
    let text = read_vectors();
    let tmp = std::env::temp_dir().join(format!("triangle_mesh_native_parity_{}", std::process::id()));
    std::fs::create_dir_all(&tmp).expect("scratch directory");
    let fa = tmp.join("a.stl").to_string_lossy().into_owned();
    let fb = tmp.join("b.stl").to_string_lossy().into_owned();
    let fo = tmp.join("o.obj").to_string_lossy().into_owned();

    let mut meshes: HashMap<String, indexed_triangle_set> = HashMap::new();
    // The mesh a chain of class operations works on (gen.cpp: one `TriangleMesh m` per block).
    let mut m = ModelMesh::new();
    // The set the three chained clean-up calls work on.
    let mut chain = indexed_triangle_set::default();
    let mut by_op: BTreeMap<String, (usize, usize)> = BTreeMap::new();
    let mut shown = 0usize;

    for line in text.lines() {
        if let Some(rest) = line.strip_prefix("M ") {
            let tok: Vec<&str> = rest.split(' ').collect();
            meshes.insert(tok[0].to_string(), parse_its(&tok[1..]));
            continue;
        }
        let (lhs, want) = line.split_once(" -> ").expect("arrow");
        let a: Vec<&str> = lhs.split(' ').collect();
        let op = a[0];
        if op == "groove" {
            let mut groove = Groove::default();
            groove.depth = f(a[1]);
            groove.width = f(a[2]);
            groove.flaps_angle = f(a[3]);
            groove.angle = f(a[4]);
            let mut verts = Vec::new();
            let its = its_make_groove_plane(&groove, f(a[5]), &mut verts);
            let mut got = format!("{} | {}", its_text(&its), verts.len());
            for v in &verts {
                got += &format!(" {} {} {}", hd(v.x), hd(v.y), hd(v.z));
            }
            check(&mut by_op, &mut shown, op, lhs, &got, want);
            continue;
        }
        let src = meshes.get(a[1]).unwrap_or_else(|| panic!("mesh {}", a[1])).clone();
        let nf = src.indices.len();
        let nb = its_face_neighbors(&src);
        let got: String = match op {
            "stats" => st(ModelMesh::from_its(src.clone()).stats()),
            "stats_flags" => {
                let mm = ModelMesh::from_its(src.clone());
                let s = mm.stats();
                format!("{} {} {} {}", b(s.manifold()), b(s.has_open_edges()), b(s.has_any_issue()), b(s.repaired()))
            }
            "volume" => hf(its_volume(&src)),
            "avg_edge" => hf(its_average_edge_length(&src)),
            "neighbors" => v3is(&nb),
            "neighbors_par" => v3is(&its_face_neighbors_par(&src)),
            "edge_ids" => v3is(&its_face_edge_ids(&src)),
            "edge_ids_cb" => v3is(&its_face_edge_ids_cb(&src, || {})),
            "edge_ids_mask" => {
                let mask: Vec<bool> = (0..nf).map(|i| i % 3 != 0).collect();
                v3is(&its_face_edge_ids_mask(&src, &mask))
            }
            "edge_ids_nb" => {
                let mut num = -7;
                let ids = its_face_edge_ids_neighbors(&src, &nb, a[2] == "1", Some(&mut num));
                format!("{} {} {}", num, v3is(&ids), v3is(&nb))
            }
            "merge_vertices" => {
                chain = src.clone();
                let n = its_merge_vertices(&mut chain, true);
                format!("{} {}", n, its_text(&chain))
            }
            "merge_remove_degenerate" => {
                let n = its_remove_degenerate_faces(&mut chain, true);
                format!("{} {}", n, its_text(&chain))
            }
            "merge_remove_compactify" => {
                let n = its_compactify_vertices(&mut chain, true);
                format!("{} {}", n, its_text(&chain))
            }
            "remove_degenerate" => {
                let mut its = src.clone();
                let n = its_remove_degenerate_faces(&mut its, true);
                format!("{} {}", n, its_text(&its))
            }
            "compactify" => {
                let mut its = src.clone();
                let n = its_compactify_vertices(&mut its, true);
                format!("{} {}", n, its_text(&its))
            }
            "flip" => {
                let mut its = src.clone();
                its_flip_triangles(&mut its);
                its_text(&its)
            }
            "reverse" => {
                let mut its = src.clone();
                its_reverse_all_facets(&mut its);
                its_text(&its)
            }
            "split" => {
                let parts = its_split(&src);
                let mut s = parts.len().to_string();
                for p in &parts {
                    s += " | ";
                    s += &its_text(p);
                }
                s
            }
            "patches" => format!("{} {}", its_number_of_patches(&src), its_number_of_patches_neighbors(&src, &nb)),
            "splittable" => format!("{} {}", b(its_is_splittable(&src)), b(its_is_splittable_neighbors(&src, &nb))),
            "open_edges" => its_num_open_edges(&src).to_string(),
            "normals" => {
                let n = its_face_normals(&src);
                let mut s = n.len().to_string();
                for v in &n {
                    s += " ";
                    s += &v3f(v);
                }
                s
            }
            "face_normal" => {
                let mut s = String::new();
                let mut i = 0;
                while i < nf {
                    let t = its_triangle_vertices(&src, i);
                    s += &format!(
                        "{} {} {} {} {} ",
                        v3f(&its_face_normal(&src, i as i32)),
                        v3f(&its_face_normal_indices(&src, &src.indices[i])),
                        v3f(&its_unnormalized_normal(&src, i)),
                        v3f(&face_normal(&t)),
                        v3f(&face_normal_normalized(&t))
                    );
                    i += 3;
                }
                s + "end"
            }
            "bbox" => {
                let x = bounding_box(&src);
                format!(
                    "{} {} {} {} {} {} {}",
                    b(x.is_defined()),
                    hd(x.min.x),
                    hd(x.min.y),
                    hd(x.min.z),
                    hd(x.max.x),
                    hd(x.max.y),
                    hd(x.max.z)
                )
            }
            "vfi" => {
                let mut vfi = VertexFaceIndex::default();
                vfi.create(&src);
                let again = VertexFaceIndex::new(&src);
                let mut s = String::new();
                for v in 0..src.vertices.len() {
                    assert_eq!(vfi.faces(v), again.faces(v));
                    assert_eq!(vfi.end(v) - vfi.begin(v), vfi.count(v));
                    s += &vfi.count(v).to_string();
                    for face in vfi.faces(v) {
                        s += &format!(" {face}");
                    }
                    s += " ; ";
                }
                vfi.clear();
                s + "end"
            }
            "triangle_helpers" => {
                let mut s = String::new();
                let mut i = 0;
                while i + 1 < nf {
                    let ta = src.indices[i];
                    let tb = src.indices[i + 1];
                    for k in 0..3 {
                        let e = its_triangle_edge(&ta, k as i32);
                        s += &format!(
                            "{} {} {} {} {} {} ",
                            its_triangle_vertex_index(&ta, tb[k]),
                            e[0],
                            e[1],
                            its_triangle_edge_index(&ta, &e),
                            its_triangle_edge_index(&tb, &e),
                            its_triangle_edge_index(&ta, &Vec2i::new(e[1], e[0]))
                        );
                    }
                    let perms = [
                        Vec3i::new(ta[0], ta[1], ta[2]),
                        Vec3i::new(ta[0], ta[2], ta[1]),
                        Vec3i::new(ta[1], ta[0], ta[2]),
                        Vec3i::new(ta[1], ta[2], ta[0]),
                        Vec3i::new(ta[2], ta[0], ta[1]),
                        Vec3i::new(ta[2], ta[1], ta[0]),
                    ];
                    for p in &perms {
                        s += b(its_triangle_vertex_the_same(&ta, p));
                        s += " ";
                    }
                    s += b(its_triangle_vertex_the_same(&ta, &tb));
                    s += " ";
                    i += 5;
                }
                s + "end"
            }
            "its_merge" => {
                let mut x = src.clone();
                its_merge(&mut x, &its_make_cube(1., 2., 3.));
                its_text(&x)
            }
            "its_merge_tris" => {
                let tris: Vec<Vec3f> = a[2..].chunks(3).map(|c| Vec3f::new(f(c[0]), f(c[1]), f(c[2]))).collect();
                let mut x = src.clone();
                its_merge_triangles(&mut x, &tris);
                its_text(&x)
            }
            "its_merge_pointf3s" => {
                let tris: Vec<slicer::geometry::Point3F> =
                    a[2..].chunks(3).map(|c| slicer::geometry::Point3F::new(d(c[0]), d(c[1]), d(c[2]))).collect();
                let mut x = src.clone();
                its_merge_pointf3s(&mut x, &tris);
                its_text(&x)
            }
            "pts_m3" => {
                let mut p = Vec::new();
                its_collect_mesh_projection_points_above(&src, &mat3f(&a[3..]), f(a[2]), &mut p);
                pts(&p)
            }
            "pts_t3" => {
                let mut p = Vec::new();
                its_collect_mesh_projection_points_above_transform::its_collect_mesh_projection_points_above(
                    &src,
                    &mat34f(&a[3..]),
                    f(a[2]),
                    &mut p,
                );
                pts(&p)
            }
            "hull_m3" => pts(&its_convex_hull_2d_above(&src, &mat3f(&a[3..]), f(a[2])).points),
            "hull_t3" => {
                pts(&its_convex_hull_2d_above_transform::its_convex_hull_2d_above(&src, &mat34f(&a[3..]), f(a[2])).points)
            }
            "stl_ascii" => {
                let ok = its_write_stl_ascii_its::its_write_stl_ascii(&fa, "label", &src);
                format!("{} {}", b(ok), fileh(&fa))
            }
            "stl_binary" => {
                let ok = its_write_stl_binary_its::its_write_stl_binary(&fb, "label", &src);
                format!("{} {}", b(ok), fileh(&fb))
            }
            "c_write_ascii" => {
                let ok = ModelMesh::from_its(src.clone()).write_ascii(&fa);
                format!("{} {}", b(ok), fileh(&fa))
            }
            "c_write_binary" => {
                let ok = ModelMesh::from_its(src.clone()).write_binary(&fb);
                format!("{} {}", b(ok), fileh(&fb))
            }
            "c_write_obj" => {
                ModelMesh::from_its(src.clone()).write_obj_file(&fo);
                fileh(&fo)
            }
            "store_triangle" => {
                let ok = its_store_triangle(&src, &fo, a[2].parse().unwrap());
                format!("{} {}", b(ok), fileh(&fo))
            }
            "store_triangles" => {
                let tris: Vec<usize> = a[2..].iter().map(|x| x.parse().unwrap()).collect();
                let ok = its_store_triangles(&src, &fo, &tris);
                format!("{} {}", b(ok), fileh(&fo))
            }
            "c_scale" => {
                m = ModelMesh::from_its(src.clone());
                TriangleMeshScaleUniform::scale(&mut m, f(a[2]));
                mesh(&m)
            }
            "c_scale3" => {
                m.scale(&Vec3f::new(f(a[2]), f(a[3]), f(a[4])));
                mesh(&m)
            }
            "c_translate3" => {
                m.translate(&Vec3f::new(f(a[2]), f(a[3]), f(a[4])));
                mesh(&m)
            }
            "c_translate" => {
                TriangleMeshTranslateXyz::translate(&mut m, f(a[3]), f(a[4]), f(a[2]));
                mesh(&m)
            }
            "c_translate0" => {
                TriangleMeshTranslateXyz::translate(&mut m, 0.0, 0.0, 0.0);
                mesh(&m)
            }
            "c_align" => {
                m.align_to_origin();
                mesh(&m)
            }
            "c_volume" => hf(m.volume()),
            "c_rotate_axis" => {
                m = ModelMesh::from_its(src.clone());
                TriangleMeshRotateAxis::rotate(&mut m, f(a[2]), axis(a[3]));
                mesh(&m)
            }
            "c_rotate_xyz" => {
                match a[3] {
                    "0" => m.rotate_x(f(a[2])),
                    "1" => m.rotate_y(f(a[2])),
                    _ => m.rotate_z(f(a[2])),
                }
                mesh(&m)
            }
            "c_rotate_axis0" => {
                TriangleMeshRotateAxis::rotate(&mut m, 0.0, axis(a[2]));
                mesh(&m)
            }
            "c_mirror" => {
                m.mirror(axis(a[2]));
                mesh(&m)
            }
            "c_mirror_xyz" => {
                match a[2] {
                    "0" => m.mirror_y(),
                    "1" => m.mirror_z(),
                    _ => m.mirror_x(),
                }
                mesh(&m)
            }
            "c_rotate_vec" => {
                m = ModelMesh::from_its(src.clone());
                TriangleMeshRotateVec3d::rotate(&mut m, f(a[2]), &Vec3d::new(d(a[3]), d(a[4]), d(a[5])));
                mesh(&m)
            }
            "c_rotate_center" => {
                m.rotate(d(a[2]), &Point::new(a[3].parse().unwrap(), a[4].parse().unwrap()));
                mesh(&m)
            }
            "c_flip" => {
                m.flip_triangles();
                mesh(&m)
            }
            "c_transform" => {
                let mut x = ModelMesh::from_its(src.clone());
                x.transform(&mat34d(&a[3..]), a[2] == "1");
                mesh(&x)
            }
            "c_transform3" => {
                let mut x = ModelMesh::from_its(src.clone());
                x.transform_matrix3(&mat3d(&a[3..]), a[2] == "1");
                mesh(&x)
            }
            "c_tbbox" => bb(&ModelMesh::from_its(src.clone()).transformed_bounding_box(&mat34d(&a[2..]))),
            "c_tbbox_z" => bb(&TriangleMeshTransformedBoundingBoxAbove::transformed_bounding_box(
                &ModelMesh::from_its(src.clone()),
                &mat34d(&a[3..]),
                d(a[2]),
            )),
            "c_bbox" => {
                let x = ModelMesh::from_its(src.clone());
                let (s, c) = (x.size(), x.center());
                format!(
                    "{} {} {} {} {} {} {} {}",
                    bb(&x.bounding_box()),
                    bb(&x.bounding_box()),
                    hd(s.x),
                    hd(s.y),
                    hd(s.z),
                    hd(c.x),
                    hd(c.y),
                    hd(c.z)
                )
            }
            "c_misc" => {
                let mut x = ModelMesh::from_its(src.clone());
                format!("{} {} {} {} {}", x.facets_count(), b(x.empty()), b(x.is_splittable()), x.memsize(), x.release_optional())
            }
            "c_hull" => pts(&ModelMesh::from_its(src.clone()).convex_hull().points),
            "c_split" => {
                let parts = ModelMesh::from_its(src.clone()).split(f(a[2]));
                let mut s = parts.len().to_string();
                for p in &parts {
                    s += " | ";
                    s += &mesh(p);
                }
                s
            }
            "c_split_rel" => {
                let mut rel: Vec<HashMap<i32, i32>> = Vec::new();
                let parts = ModelMesh::from_its(src.clone()).split_and_save_relationship(&mut rel, f(a[2]));
                let mut s = parts.len().to_string();
                for p in &parts {
                    s += " | ";
                    s += &mesh(p);
                }
                s += &format!(" # {}", rel.len());
                for mp in &rel {
                    let mut kv: Vec<(i32, i32)> = mp.iter().map(|(k, v)| (*k, *v)).collect();
                    kv.sort();
                    s += &format!(" | {}", kv.len());
                    for (k, v) in kv {
                        s += &format!(" {k} {v}");
                    }
                }
                s
            }
            "c_merge" => {
                let mut other = ModelMesh::from_its(its_make_cube(1.5, 0.5, 2.5));
                other.translate(&Vec3f::new(3.0, -2.0, 0.25));
                let mut merged = ModelMesh::from_its(src.clone());
                merged.merge(&other);
                mesh(&merged)
            }
            "c_son" => {
                let mut son = ModelMesh::new();
                ModelMesh::from_its(src.clone()).extract_son_mesh(&mut son, a[2].parse().unwrap(), a[3].parse().unwrap());
                its_text(&son.its)
            }
            "c_clear" => {
                let mut x = ModelMesh::from_its(src.clone());
                x.clear();
                mesh(&x)
            }
            other => panic!("unknown line {other}"),
        };
        check(&mut by_op, &mut shown, op, lhs, &got, want);
    }
    let _ = std::fs::remove_dir_all(&tmp);
    let total: usize = by_op.values().map(|v| v.0).sum();
    println!("lines compared: {total}; by operation (compared, differing): {by_op:?}");
    let bad: Vec<(&String, &(usize, usize))> = by_op.iter().filter(|(_, v)| v.1 != 0).collect();
    assert!(bad.is_empty(), "lines differ from the C++: {bad:?}");
}

fn check(by_op: &mut BTreeMap<String, (usize, usize)>, shown: &mut usize, op: &str, lhs: &str, got: &str, want: &str) {
    let e = by_op.entry(op.to_string()).or_insert((0, 0));
    e.0 += 1;
    if got != want {
        e.1 += 1;
        if *shown < 60 {
            *shown += 1;
            let (gw, ww): (Vec<&str>, Vec<&str>) = (got.split(' ').collect(), want.split(' ').collect());
            let first = (0..gw.len().min(ww.len())).find(|&k| gw[k] != ww[k]);
            let head: String = lhs.chars().take(60).collect();
            eprintln!(
                "{head}: rust {} tokens, c++ {}; first difference at token {:?}: rust {:?} c++ {:?}",
                gw.len(),
                ww.len(),
                first,
                first.map(|k| gw[k]),
                first.map(|k| ww[k])
            );
        }
    }
}
