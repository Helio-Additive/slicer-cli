//! `texture_to_color/color_utils.rs` against Bambu Studio's `TextureToColor/ColorUtils.cpp` at the
//! pin.
//!
//! `tests/data/texture_to_color_color_utils/vectors_*.txt` are written by `gen.cpp` of that
//! directory, which includes the C++ unit unchanged and runs it over CGAL (how to build it is in
//! its header). Each line is one call with its arguments and its result; the test makes the same
//! call and requires the same result, floating point numbers bit for bit.

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;

use slicer::texture_to_color::cgal_utils::{self, CGALMesh};
use slicer::texture_to_color::color_utils::*;
use slicer::texture_to_color::tri_mesh::TriMesh;
use slicer::triangle_mesh::Vec2f;
use slicer::triangle_set_sampling::{Vec3d, Vec3f, Vec3i};

struct Tok<'a>(std::str::SplitWhitespace<'a>);

impl<'a> Tok<'a> {
    fn s(&mut self) -> &'a str {
        self.0.next().expect("token")
    }
    fn u(&mut self) -> usize {
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
    fn u3(&mut self) -> [usize; 3] {
        [self.u(), self.u(), self.u()]
    }
    fn arrow(&mut self) {
        assert_eq!(self.s(), "->");
    }
    fn colors(&mut self) -> Vec<[usize; 3]> {
        let n = self.u();
        (0..n).map(|_| self.u3()).collect()
    }
    fn dcolors(&mut self) -> Vec<[f64; 3]> {
        let n = self.u();
        (0..n).map(|_| self.d3()).collect()
    }
    fn labels(&mut self) -> Vec<usize> {
        let n = self.u();
        (0..n).map(|_| self.u()).collect()
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

fn dcolor_bits(v: &[[f64; 3]]) -> Vec<[u64; 3]> {
    v.iter().map(bits3).collect()
}

fn mesh_bits(m: &TriMesh) -> (Vec<[u32; 3]>, Vec<[i32; 3]>) {
    (
        m.vertices.iter().map(|v| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]).collect(),
        m.indices.iter().map(|f| [f[0], f[1], f[2]]).collect(),
    )
}

/// The first difference of two meshes, in words; `None` when they are equal bit for bit.
fn mesh_diff(a: &TriMesh, b: &TriMesh) -> Option<String> {
    let (av, af) = mesh_bits(a);
    let (bv, bf) = mesh_bits(b);
    if av.len() != bv.len() || af.len() != bf.len() {
        return Some(format!("{} vertices and {} faces, expected {} and {}", av.len(), af.len(), bv.len(), bf.len()));
    }
    let nv = av.iter().zip(bv.iter()).filter(|(x, y)| x != y).count();
    if let Some(i) = (0..av.len()).find(|&i| av[i] != bv[i]) {
        return Some(format!("{} of {} vertices differ, the first is {}: {:?}, expected {:?}", nv, av.len(), i, a.vertices[i], b.vertices[i]));
    }
    (0..af.len()).find(|&i| af[i] != bf[i]).map(|i| format!("face {}: {:?}, expected {:?}", i, af[i], bf[i]))
}

fn uv_bits(v: &[Vec2f]) -> Vec<[u32; 2]> {
    v.iter().map(|u| [u.x.to_bits(), u.y.to_bits()]).collect()
}

fn to_cgal(m: &TriMesh) -> CGALMesh {
    let mut cm = CGALMesh::new();
    cgal_utils::convert_trimesh_to_cgal(m, &mut cm);
    cm
}

/// Runs every line of a vectors file; the number of lines per tag is returned.
fn run(text: &str) -> std::collections::BTreeMap<String, usize> {
    let mut counts = std::collections::BTreeMap::new();
    let mut mesh = TriMesh::default();
    for (lineno, line) in text.lines().enumerate() {
        let mut t = Tok(line.split_whitespace());
        let tag = t.s();
        *counts.entry(tag.to_string()).or_insert(0) += 1;
        let at = format!("line {} ({})", lineno + 1, tag);
        match tag {
            "mesh" => {
                mesh = t.mesh();
            }
            "rgb2hsv" => {
                let c = t.u3();
                t.arrow();
                assert_eq!(bits3(&convert_rgb_to_hsv(&c)), bits3(&t.d3()), "{at}");
            }
            "hsv2rgb" => {
                let h = t.d3();
                t.arrow();
                assert_eq!(convert_hsv_to_rgb(&h), t.u3(), "{at}");
            }
            "rgb2xyz" => {
                let c = t.u3();
                t.arrow();
                assert_eq!(bits3(&convert_rgb_to_xyz(&c)), bits3(&t.d3()), "{at}");
            }
            "srgb2xyz" => {
                let c = t.d3();
                t.arrow();
                assert_eq!(bits3(&convert_srgb01_to_xyz(c[0], c[1], c[2])), bits3(&t.d3()), "{at}");
            }
            "srgb2lab" => {
                let c = t.d3();
                t.arrow();
                assert_eq!(bits3(&convert_srgb01_to_lab(&c)), bits3(&t.d3()), "{at}");
            }
            "xyz2lab" => {
                let c = t.d3();
                t.arrow();
                assert_eq!(bits3(&convert_xyz_to_lab(&c)), bits3(&t.d3()), "{at}");
            }
            "rgb2lab" => {
                let c = t.u3();
                t.arrow();
                assert_eq!(bits3(&convert_rgb_to_lab(&c)), bits3(&t.d3()), "{at}");
            }
            "lab2rgb" => {
                let l = t.d3();
                t.arrow();
                assert_eq!(convert_lab_to_rgb(&l), t.u3(), "{at}");
            }
            "ciede2000" => {
                let a = t.d3();
                let b = t.d3();
                t.arrow();
                assert_eq!(ciede2000(&a, &b).to_bits(), t.d().to_bits(), "{at}");
            }
            "de_rgb" => {
                let a = t.u3();
                let b = t.u3();
                t.arrow();
                assert_eq!(calc_rgb_color_difference_by_ciede2000(&a, &b).to_bits(), t.d().to_bits(), "{at}");
            }
            "sq_rgb" => {
                let a = t.u3();
                let b = t.u3();
                t.arrow();
                assert_eq!(calc_rgb_color_difference_by_squared_rgb(&a, &b).to_bits(), t.d().to_bits(), "{at}");
            }
            "de_srgb01" => {
                let a = t.d3();
                let b = t.d3();
                t.arrow();
                assert_eq!(calc_rgb_color_difference_by_ciede2000_srgb01(&a, &b).to_bits(), t.d().to_bits(), "{at}");
            }
            "sq_double" => {
                let a = t.d3();
                let b = t.d3();
                t.arrow();
                assert_eq!(calc_rgb_color_difference_by_squared_rgb_double(&a, &b).to_bits(), t.d().to_bits(), "{at}");
            }
            "normalize" => {
                let mut v = t.d3();
                t.arrow();
                normalize(&mut v);
                assert_eq!(bits3(&v), bits3(&t.d3()), "{at}");
            }
            "angle" => {
                let a = t.d3();
                let b = t.d3();
                t.arrow();
                assert_eq!(get_angle_between_vectors(&a, &b).to_bits(), t.d().to_bits(), "{at}");
            }
            "fps" | "kmeans" => {
                let lab = t.u() != 0;
                let k = t.u();
                let max_iter = if tag == "kmeans" { t.u() } else { 0 };
                let working = t.dcolors();
                t.arrow();
                let f: WorkingDistFunc = if lab { ciede2000 } else { calc_rgb_color_difference_by_squared_rgb_double };
                let got = if tag == "fps" {
                    farthest_point_sampling_init(&working, k, f)
                } else {
                    kmeans_core(&working, k, max_iter, f, &None)
                };
                assert_eq!(dcolor_bits(&got), dcolor_bits(&t.dcolors()), "{at}");
            }
            "cluster_k_means" => {
                let mut p = ClusterParameters::default();
                p.color_difference_method = if t.u() == 0 { ColorDifferenceMethod::RGB } else { ColorDifferenceMethod::Lab };
                p.cluster_k = t.u();
                p.max_iter = t.u();
                let colors = t.colors();
                t.arrow();
                assert_eq!(cluster_k_means(&colors, &p), t.colors(), "{at}");
            }
            "cluster_to_specified_colors" => {
                let colors = t.colors();
                let specified = t.colors();
                t.arrow();
                assert_eq!(cluster_to_specified_colors(&colors, &specified), t.colors(), "{at}");
            }
            "cluster_adaptive" => {
                let mut p = ClusterParameters::default();
                p.color_difference_method = if t.u() == 0 { ColorDifferenceMethod::RGB } else { ColorDifferenceMethod::Lab };
                p.max_color_distance = t.d();
                p.max_cluster_k = t.u();
                p.max_iter = t.u();
                let cancel_after = t.u();
                let calls = Arc::new(AtomicUsize::new(0));
                if cancel_after > 0 {
                    let calls = calls.clone();
                    p.cancel_callback = Some(Arc::new(move || calls.fetch_add(1, Ordering::SeqCst) + 1 >= cancel_after));
                }
                let colors = t.colors();
                t.arrow();
                assert_eq!(cluster_adaptive(&colors, &p), t.colors(), "{at}");
                assert_eq!(calls.load(Ordering::SeqCst), t.u(), "{at}: calls of the cancel callback");
            }
            "calc_nearest_color_id" => {
                let colors = t.colors();
                let c = t.u3();
                t.arrow();
                let mut id = 12345usize;
                let ok = calc_nearest_color_id(&colors, &c, &mut id);
                assert_eq!((ok as usize, id), (t.u(), t.u()), "{at}");
            }
            "average_edge_length_impl" => {
                t.arrow();
                assert_eq!(average_edge_length_impl(&to_cgal(&mesh)).to_bits(), t.d().to_bits(), "{at}");
            }
            "calc_face_normals" => {
                t.arrow();
                let mut normals: Vec<Vec3d> = Vec::new();
                calc_face_normals(&to_cgal(&mesh), &mut normals);
                let n = t.u();
                assert_eq!(normals.len(), n, "{at}");
                for (i, got) in normals.iter().enumerate() {
                    assert_eq!(bits3(&[got.x, got.y, got.z]), bits3(&t.d3()), "{at}: face {i}");
                }
            }
            "is_closed" => {
                t.arrow();
                assert_eq!(is_closed(&mesh) as usize, t.u(), "{at}");
            }
            "get_connected_face_groups" => {
                t.arrow();
                let groups = get_connected_face_groups(&to_cgal(&mesh));
                let got: Vec<Vec<usize>> = groups.iter().map(|g| g.iter().map(|f| f.idx()).collect()).collect();
                let n = t.u();
                let want: Vec<Vec<usize>> = (0..n).map(|_| t.labels()).collect();
                assert_eq!(got, want, "{at}");
            }
            "get_components" => {
                let uvs = t.uvs();
                t.arrow();
                let mut meshes = vec![mesh.clone()];
                let mut mesh_uvs: Vec<Vec<Vec2f>> = Vec::new();
                let ok = get_components(&mesh, &uvs, &mut meshes, &mut mesh_uvs);
                assert_eq!(ok as usize, t.u(), "{at}");
                let n = t.u();
                assert_eq!(meshes.len(), n, "{at}");
                assert_eq!(mesh_uvs.len(), n, "{at}");
                for c in 0..n {
                    assert_eq!(mesh_diff(&meshes[c], &t.mesh()), None, "{at}: component {c}");
                    assert!(uv_bits(&mesh_uvs[c]) == uv_bits(&t.uvs()), "{at}: uvs of component {c}");
                }
            }
            "smooth_region_topo_boundary" => {
                let mut labels = t.labels();
                let max_iters = t.u();
                t.arrow();
                let mut cm = to_cgal(&mesh);
                let ok = smooth_region_topo_boundary(&mut cm, &mut labels, max_iters);
                assert_eq!(ok as usize, t.u(), "{at}");
                assert_eq!(labels, t.labels(), "{at}");
            }
            "mesh_cluster" => {
                let centers = t.colors();
                let mut rgbs = t.colors();
                let mut ids = t.labels();
                t.arrow();
                let ok = mesh_cluster(&mesh, &centers, &mut rgbs, &mut ids);
                assert_eq!(ok as usize, t.u(), "{at}");
                assert!(rgbs == t.colors(), "{at}: colors");
                assert!(ids == t.labels(), "{at}: ids");
            }
            "save_polylines" => {
                let n = t.u();
                let polylines: Vec<Vec<[f64; 3]>> = (0..n).map(|_| t.dcolors()).collect();
                t.arrow();
                let path = std::env::temp_dir().join(format!("slicer_save_polylines_{}_{}.obj", std::process::id(), lineno));
                assert!(save_polylines(path.to_str().unwrap(), &polylines));
                let got = std::fs::read(&path).expect("the file");
                let _ = std::fs::remove_file(&path);
                let len = t.u();
                let want: Vec<u8> = (0..len).map(|_| t.u() as u8).collect();
                assert_eq!(String::from_utf8_lossy(&got), String::from_utf8_lossy(&want), "{at}");
            }
            "smooth_region_labels" => {
                let mut labels = t.labels();
                let sp = SmoothParameters { smooth_weight: t.d() };
                t.arrow();
                let mut m = mesh.clone();
                let ok = smooth_region_with_labels(&mut m, &mut labels, &sp);
                assert_eq!(ok as usize, t.u(), "{at}");
                assert_eq!(mesh_diff(&m, &t.mesh()), None, "{at}");
                assert!(labels == t.labels(), "{at}: labels");
            }
            "smooth_region_colors" => {
                let mut colors = t.colors();
                let sp = SmoothParameters { smooth_weight: t.d() };
                t.arrow();
                let mut m = mesh.clone();
                let ok = smooth_region(&mut m, &mut colors, &sp);
                assert_eq!(ok as usize, t.u(), "{at}");
                assert_eq!(mesh_diff(&m, &t.mesh()), None, "{at}");
                assert!(colors == t.colors(), "{at}: colors");
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
fn color_spaces_match_the_pin() {
    let counts = run(include_str!("data/texture_to_color_color_utils/vectors_color.txt"));
    expect(
        &counts,
        &[
            ("rgb2hsv", 150),
            ("hsv2rgb", 200),
            ("rgb2xyz", 150),
            ("srgb2xyz", 200),
            ("srgb2lab", 200),
            ("xyz2lab", 200),
            ("rgb2lab", 150),
            ("lab2rgb", 450),
        ],
    );
}

#[test]
fn color_differences_match_the_pin() {
    let counts = run(include_str!("data/texture_to_color_color_utils/vectors_difference.txt"));
    expect(
        &counts,
        &[("ciede2000", 600), ("de_rgb", 300), ("sq_rgb", 300), ("de_srgb01", 200), ("sq_double", 200), ("normalize", 200), ("angle", 200)],
    );
}

#[test]
fn clustering_of_colors_matches_the_pin() {
    let counts = run(include_str!("data/texture_to_color_color_utils/vectors_cluster.txt"));
    expect(
        &counts,
        &[
            ("fps", 60),
            ("kmeans", 60),
            ("cluster_k_means", 80),
            ("cluster_to_specified_colors", 40),
            ("cluster_adaptive", 36),
            ("calc_nearest_color_id", 60),
        ],
    );
}

#[test]
fn mesh_functions_match_the_pin() {
    let counts = run(include_str!("data/texture_to_color_color_utils/vectors_mesh.txt"));
    expect(
        &counts,
        &[
            ("mesh", 71),
            ("average_edge_length_impl", 30),
            ("calc_face_normals", 30),
            ("is_closed", 30),
            ("get_connected_face_groups", 30),
            ("get_components", 30),
            ("smooth_region_topo_boundary", 41),
            ("save_polylines", 6),
        ],
    );
}

#[test]
fn mesh_cluster_matches_the_pin() {
    let counts = run(include_str!("data/texture_to_color_color_utils/vectors_mesh_cluster.txt"));
    expect(&counts, &[("mesh", 90), ("mesh_cluster", 90)]);
}

#[test]
fn smooth_region_matches_the_pin() {
    let counts = run(include_str!("data/texture_to_color_color_utils/vectors_smooth.txt"));
    expect(&counts, &[("mesh", 48), ("smooth_region_labels", 24), ("smooth_region_colors", 24)]);
}
