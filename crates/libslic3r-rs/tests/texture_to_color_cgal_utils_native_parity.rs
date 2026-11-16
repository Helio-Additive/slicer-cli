//! `texture_to_color/cgal_utils.rs` against Bambu Studio's `TextureToColor/CgalUtils.hpp` at the
//! pin and against CGAL itself.
//!
//! `tests/data/texture_to_color_cgal_utils/vectors_*.txt` are written by `gen.cpp` of that
//! directory (how to build it is in its header). Each line is one call with its arguments and its
//! result; the test makes the same call and requires the same result, floating point numbers bit
//! for bit. `CGALMesh` is compared with `CGAL::Surface_mesh` index by index: the halfedge of every
//! vertex and face, and target, face, next and prev of every halfedge.

use slicer::texture_to_color::cgal_utils::*;
use slicer::texture_to_color::tri_mesh::TriMesh;
use slicer::triangle_mesh::Vec2f;
use slicer::triangle_set_sampling::{Vec3f, Vec3i};

struct Tok<'a>(std::str::SplitWhitespace<'a>);

impl<'a> Tok<'a> {
    fn s(&mut self) -> &'a str {
        self.0.next().expect("token")
    }
    fn u(&mut self) -> usize {
        self.s().parse().expect("integer")
    }
    fn i(&mut self) -> i64 {
        self.s().parse().expect("integer")
    }
    fn d(&mut self) -> f64 {
        f64::from_bits(u64::from_str_radix(self.s(), 16).expect("double"))
    }
    fn f(&mut self) -> f32 {
        f32::from_bits(u32::from_str_radix(self.s(), 16).expect("float"))
    }
    fn d3(&mut self) -> [f64; 3] {
        [self.d(), self.d(), self.d()]
    }
    fn arrow(&mut self) {
        assert_eq!(self.s(), "->");
    }
    fn uvs(&mut self) -> Vec<Vec2f> {
        let n = self.u();
        (0..n).map(|_| Vec2f::new(self.f(), self.f())).collect()
    }
    fn mesh(&mut self) -> TriMesh {
        let nv = self.u();
        let vertices: Vec<Vec3f> = (0..nv).map(|_| Vec3f::new(self.f(), self.f(), self.f())).collect();
        let nf = self.u();
        let indices: Vec<Vec3i> = (0..nf)
            .map(|_| Vec3i::new(self.s().parse().unwrap(), self.s().parse().unwrap(), self.s().parse().unwrap()))
            .collect();
        TriMesh::new(indices, vertices)
    }
    fn end(&mut self) {
        assert!(self.0.next().is_none(), "tokens left on the line");
    }
}

fn bits3(a: &[f64; 3]) -> [u64; 3] {
    [a[0].to_bits(), a[1].to_bits(), a[2].to_bits()]
}

fn mesh_bits(m: &TriMesh) -> (Vec<[u32; 3]>, Vec<[i32; 3]>) {
    (
        m.vertices.iter().map(|v| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]).collect(),
        m.indices.iter().map(|f| [f[0], f[1], f[2]]).collect(),
    )
}

fn idx(i: u32) -> i64 {
    if i == u32::MAX {
        -1
    } else {
        i as i64
    }
}

/// The connectivity in the order `gen.cpp` prints it.
fn connectivity(cm: &CGALMesh) -> (Vec<(i64, [u64; 3])>, Vec<[i64; 4]>, Vec<i64>) {
    (
        cm.vertices().map(|v| (idx(cm.halfedge_v(v).0), bits3(&cm.point(v)))).collect(),
        cm.halfedges().map(|h| [idx(cm.target(h).0), idx(cm.face(h).0), idx(cm.next(h).0), idx(cm.prev(h).0)]).collect(),
        cm.faces().map(|f| idx(cm.halfedge_f(f).0)).collect(),
    )
}

fn read_connectivity(t: &mut Tok) -> (Vec<(i64, [u64; 3])>, Vec<[i64; 4]>, Vec<i64>) {
    let nv = t.u();
    let v = (0..nv).map(|_| (t.i(), bits3(&t.d3()))).collect();
    let nh = t.u();
    let h = (0..nh).map(|_| [t.i(), t.i(), t.i(), t.i()]).collect();
    let nf = t.u();
    let f = (0..nf).map(|_| t.i()).collect();
    (v, h, f)
}

/// Runs every line of a vectors file; the number of lines per tag is returned.
fn run(text: &str) -> std::collections::BTreeMap<String, usize> {
    let mut counts = std::collections::BTreeMap::new();
    let mut mesh = TriMesh::default();
    let mut cm = CGALMesh::new();
    let mut tree: Option<Tree> = None;
    for (lineno, line) in text.lines().enumerate() {
        let mut t = Tok(line.split_whitespace());
        let tag = t.s();
        *counts.entry(tag.to_string()).or_insert(0) += 1;
        let at = format!("line {} ({})", lineno + 1, tag);
        match tag {
            "mesh" => {
                mesh = t.mesh();
                cm = CGALMesh::new();
                tree = None;
            }
            "trimesh_to_cgal" => {
                t.arrow();
                cm = trimesh_to_cgal(&mesh);
                assert!(connectivity(&cm) == read_connectivity(&mut t), "{at}");
            }
            "counts" => {
                t.arrow();
                let got = (cm.number_of_vertices(), cm.number_of_halfedges(), cm.number_of_edges(), cm.number_of_faces(), is_closed(&cm) as usize);
                assert_eq!(got, (t.u(), t.u(), t.u(), t.u(), t.u()), "{at}");
            }
            "cgal_to_trimesh" => {
                t.arrow();
                assert!(mesh_bits(&cgal_to_trimesh(&cm)) == mesh_bits(&t.mesh()), "{at}");
            }
            "convert_trimesh_to_cgal" => {
                t.arrow();
                let mut cm2 = CGALMesh::new();
                cm2.add_vertex([1.0, 2.0, 3.0]);
                let ok = convert_trimesh_to_cgal(&mesh, &mut cm2);
                assert_eq!(ok as usize, t.u(), "{at}");
                assert_eq!((connectivity(&cm2) == connectivity(&cm)) as usize, t.u(), "{at}");
            }
            "convert_trimesh_to_cgal_with_uvs" => {
                let uvs = t.uvs();
                t.arrow();
                let mut cm2 = CGALMesh::new();
                cm2.add_vertex([1.0, 2.0, 3.0]);
                let mut cuvs = vec![Vec2f::new(9.0, 9.0)];
                let ok = convert_trimesh_to_cgal_with_uvs(&mesh, &uvs, &mut cm2, &mut cuvs);
                assert_eq!(ok as usize, t.u(), "{at}");
                assert_eq!((connectivity(&cm2) == connectivity(&cm)) as usize, t.u(), "{at}");
                let want = t.uvs();
                assert!(cuvs.iter().map(|u| [u.x.to_bits(), u.y.to_bits()]).eq(want.iter().map(|u| [u.x.to_bits(), u.y.to_bits()])), "{at}");
            }
            "is_mesh_halfedge_compatible" => {
                t.arrow();
                assert_eq!(is_mesh_halfedge_compatible(&mesh) as usize, t.u(), "{at}");
            }
            "halfedges_around_target" => {
                t.arrow();
                let mut around: Vec<usize> = Vec::new();
                for v in cm.vertices() {
                    if cm.is_isolated(v) {
                        around.push(0);
                        continue;
                    }
                    let hs = cm.halfedges_around_target(cm.halfedge_v(v));
                    around.push(hs.len());
                    around.extend(hs.iter().map(|h| h.idx()));
                }
                let n = t.u();
                let want: Vec<usize> = (0..n).map(|_| t.u()).collect();
                assert!(around == want, "{at}");
            }
            // checked by tests/texture_to_color_repair_native_parity.rs
            "ComputeBoundaryEdgeStats" => continue,
            "construct_projected_point_3" => {
                let tri = [t.d3(), t.d3(), t.d3()];
                let q = t.d3();
                t.arrow();
                assert_eq!(bits3(&construct_projected_point_3(&tri, &q)), bits3(&t.d3()), "{at}");
            }
            "compare_distance_3" => {
                let (p, q, r) = (t.d3(), t.d3(), t.d3());
                t.arrow();
                let got = match compare_distance_3(&p, &q, &r) {
                    std::cmp::Ordering::Less => -1,
                    std::cmp::Ordering::Equal => 0,
                    std::cmp::Ordering::Greater => 1,
                };
                assert_eq!(got, t.i(), "{at}");
            }
            "squared_distance" => {
                let (p, q) = (t.d3(), t.d3());
                t.arrow();
                assert_eq!(squared_distance(&p, &q).to_bits(), t.d().to_bits(), "{at}");
            }
            "centroid" => {
                let n = t.u();
                let pts: Vec<[f64; 3]> = (0..n).map(|_| t.d3()).collect();
                t.arrow();
                assert_eq!(bits3(&centroid(&pts)), bits3(&t.d3()), "{at}");
            }
            "compute_face_normal" => {
                t.arrow();
                cm = trimesh_to_cgal(&mesh);
                assert_eq!(cm.number_of_faces(), t.u(), "{at}");
                for f in cm.faces() {
                    assert_eq!(bits3(&compute_face_normal(f, &cm)), bits3(&t.d3()), "{at}: face {}", f.idx());
                }
            }
            "closest_point" => {
                let q = t.d3();
                t.arrow();
                if tree.is_none() {
                    let mut tr = Tree::new(&cm);
                    tr.accelerate_distance_queries(&cm);
                    tree = Some(tr);
                }
                assert_eq!(bits3(&tree.as_mut().unwrap().closest_point(&cm, &q)), bits3(&t.d3()), "{at}");
            }
            other => panic!("unknown tag {other}"),
        }
        t.end();
    }
    counts
}

fn expect(counts: &std::collections::BTreeMap<String, usize>, want: &[(&str, usize)]) {
    let got: Vec<(&str, usize)> = counts.iter().map(|(k, v)| (k.as_str(), *v)).collect();
    let mut want = want.to_vec();
    want.sort();
    assert_eq!(got, want);
}

#[test]
fn surface_mesh_matches_cgal() {
    let counts = run(include_str!("data/texture_to_color_cgal_utils/vectors_mesh.txt"));
    expect(
        &counts,
        &[
            ("mesh", 112),
            ("trimesh_to_cgal", 32),
            ("counts", 32),
            ("cgal_to_trimesh", 32),
            ("convert_trimesh_to_cgal", 32),
            ("convert_trimesh_to_cgal_with_uvs", 32),
            ("is_mesh_halfedge_compatible", 112),
            ("halfedges_around_target", 32),
            ("ComputeBoundaryEdgeStats", 32),
        ],
    );
}

#[test]
fn kernel_constructions_match_cgal() {
    let counts = run(include_str!("data/texture_to_color_cgal_utils/vectors_kernel.txt"));
    expect(&counts, &[("construct_projected_point_3", 600), ("compare_distance_3", 400), ("squared_distance", 400), ("centroid", 100)]);
}

#[test]
fn closest_point_matches_cgal() {
    let counts = run(include_str!("data/texture_to_color_cgal_utils/vectors_tree.txt"));
    expect(&counts, &[("mesh", 28), ("compute_face_normal", 28), ("closest_point", 672)]);
}
