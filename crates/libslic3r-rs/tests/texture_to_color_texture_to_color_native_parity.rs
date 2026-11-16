//! `texture_to_color/texture_to_color.rs` against Bambu Studio's
//! `TextureToColor/TextureToColor.cpp` at the pin.
//!
//! `tests/data/texture_to_color_texture_to_color/vectors_*.txt` are written by `gen.cpp` of that
//! directory, which includes the C++ unit unchanged and calls it with OpenCV's `cv::Mat` (how to
//! build it is in its header). Each line is one call with its arguments and its result; the test
//! makes the same call and requires the same result: the meshes bit for bit, the colors, the
//! progress reports and how often the cancel callback was asked.
//!
//! The meshes of the vectors are halfedge-compatible, so `RepairMesh` (CGAL, reached through
//! `PolygonMeshProcessing`) is not called; the implementation below fails the test if it is.

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};

use slicer::texture_to_color::callbacks::{AlgoCancelCallback, AlgoProgress, AlgoProgressCallback};
use slicer::texture_to_color::cgal_utils::{CGALMesh, PolygonMeshProcessing, Point_3, SM_Face_index, SM_Halfedge_index};
use slicer::texture_to_color::texture_to_color::*;
use slicer::texture_to_color::tri_mesh::TriMesh;
use slicer::triangle_mesh::Vec2f;
use slicer::triangle_set_sampling::{Vec3f, Vec3i};

type RGB = [usize; 3];

struct Tok<'a>(std::str::SplitWhitespace<'a>);

impl<'a> Tok<'a> {
    fn s(&mut self) -> &'a str {
        self.0.next().expect("token")
    }
    fn u(&mut self) -> usize {
        self.s().parse().expect("integer")
    }
    fn i(&mut self) -> i32 {
        self.s().parse().expect("integer")
    }
    fn d(&mut self) -> f64 {
        f64::from_bits(u64::from_str_radix(self.s(), 16).expect("double"))
    }
    fn f(&mut self) -> f32 {
        f32::from_bits(u32::from_str_radix(self.s(), 16).expect("float"))
    }
    fn u3(&mut self) -> [usize; 3] {
        [self.u(), self.u(), self.u()]
    }
    fn arrow(&mut self) {
        assert_eq!(self.s(), "->");
    }
    fn colors(&mut self) -> Vec<RGB> {
        let n = self.u();
        (0..n).map(|_| self.u3()).collect()
    }
    fn labels(&mut self) -> Vec<usize> {
        let n = self.u();
        (0..n).map(|_| self.u()).collect()
    }
    fn uv(&mut self) -> Vec2f {
        Vec2f::new(self.f(), self.f())
    }
    fn face_uvs(&mut self) -> Vec<FaceUVArray> {
        let n = self.u();
        (0..n).map(|_| [self.uv(), self.uv(), self.uv()]).collect()
    }
    fn mesh(&mut self) -> TriMesh {
        let nv = self.u();
        let vertices: Vec<Vec3f> = (0..nv).map(|_| Vec3f::new(self.f(), self.f(), self.f())).collect();
        let nf = self.u();
        let indices: Vec<Vec3i> = (0..nf).map(|_| Vec3i::new(self.i(), self.i(), self.i())).collect();
        TriMesh::new(indices, vertices)
    }
    fn texture(&mut self) -> Mat {
        let rows = self.i();
        let cols = self.i();
        let channels = self.i();
        let step = self.u();
        let hex = self.s();
        let data: Vec<u8> =
            if hex == "-" { Vec::new() } else { (0..hex.len() / 2).map(|k| u8::from_str_radix(&hex[2 * k..2 * k + 2], 16).unwrap()).collect() };
        assert_eq!(data.len(), rows as usize * step);
        Mat { rows, cols, channels, step, data }
    }
    fn settings(&mut self) -> TextureToColorSettings {
        let mut s = TextureToColorSettings::default();
        s.target_colors_num = self.u();
        s.smooth_weight = self.d();
        s.oversampling_iters = self.u();
        s.oversampling_min_face_count = self.u();
        s.oversampling_max_face_count = self.u();
        s.max_color_distance = self.d();
        s.max_cluster_k = self.u();
        s.mesh_repair_decision = if self.u() != 0 { MeshRepairDecision::RepairAndImport } else { MeshRepairDecision::ImportWithoutRepair };
        s
    }
    fn events(&mut self) -> Vec<(i32, String)> {
        let n = self.u();
        (0..n)
            .map(|_| {
                let e = self.s();
                let (p, m) = e.split_once('~').expect("percent~message");
                (p.parse().unwrap(), m.to_string())
            })
            .collect()
    }
    fn end(&mut self) {
        assert!(self.0.next().is_none(), "tokens left on the line");
    }
}

fn mesh_bits(m: &TriMesh) -> (Vec<[u32; 3]>, Vec<[i32; 3]>) {
    (m.vertices.iter().map(|v| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]).collect(), m.indices.iter().map(|f| [f[0], f[1], f[2]]).collect())
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

fn uv_bits(v: &[FaceUVArray]) -> Vec<[u32; 6]> {
    v.iter().map(|f| [f[0].x.to_bits(), f[0].y.to_bits(), f[1].x.to_bits(), f[1].y.to_bits(), f[2].x.to_bits(), f[2].y.to_bits()]).collect()
}

/// `RepairMesh` is not reached by the vectors (their meshes are halfedge-compatible).
struct NoRepair;

impl PolygonMeshProcessing for NoRepair {
    fn does_self_intersect(&self, _: &CGALMesh) -> bool {
        panic!("RepairMesh reached")
    }
    fn remove_self_intersections(&self, _: &mut CGALMesh) -> bool {
        panic!("RepairMesh reached")
    }
    fn isotropic_remeshing(&self, _: &mut CGALMesh, _: f64, _: usize, _: bool, _: &[bool], _: &[bool], _: bool) {
        panic!("RepairMesh reached")
    }
    fn repair_polygon_soup(&self, _: &mut Vec<Point_3>, _: &mut Vec<Vec<usize>>) {
        panic!("RepairMesh reached")
    }
    fn orient_polygon_soup(&self, _: &mut Vec<Point_3>, _: &mut Vec<Vec<usize>>) -> bool {
        panic!("RepairMesh reached")
    }
    fn polygon_soup_to_polygon_mesh(&self, _: &[Point_3], _: &[Vec<usize>], _: &mut CGALMesh) {
        panic!("RepairMesh reached")
    }
    fn remove_degenerate_faces(&self, _: &mut CGALMesh) -> bool {
        panic!("RepairMesh reached")
    }
    fn stitch_borders(&self, _: &mut CGALMesh) -> usize {
        panic!("RepairMesh reached")
    }
    fn duplicate_non_manifold_vertices(&self, _: &mut CGALMesh) -> usize {
        panic!("RepairMesh reached")
    }
    fn triangulate_hole(&self, _: &mut CGALMesh, _: SM_Halfedge_index, _: &mut Vec<SM_Face_index>) {
        panic!("RepairMesh reached")
    }
}

/// The progress reports of one call, as the generator prints them (blanks as `_`).
struct Events(Arc<Mutex<Vec<(i32, String)>>>);

impl Events {
    fn new() -> Self {
        Events(Arc::new(Mutex::new(Vec::new())))
    }
    fn callback(&self) -> AlgoProgressCallback {
        let seen = self.0.clone();
        Some(Arc::new(move |p: AlgoProgress| seen.lock().unwrap().push((p.percent, p.message.replace(' ', "_")))))
    }
    fn get(&self) -> Vec<(i32, String)> {
        self.0.lock().unwrap().clone()
    }
}

/// The cancel callback of the generator: none when `after` is 0, else true from its `after`-th call.
struct Cancel(Arc<AtomicUsize>, usize);

impl Cancel {
    fn new(after: usize) -> Self {
        Cancel(Arc::new(AtomicUsize::new(0)), after)
    }
    fn callback(&self) -> AlgoCancelCallback {
        if self.1 == 0 {
            return None;
        }
        let (calls, after) = (self.0.clone(), self.1);
        Some(Arc::new(move || calls.fetch_add(1, Ordering::SeqCst) + 1 >= after))
    }
    fn calls(&self) -> usize {
        self.0.load(Ordering::SeqCst)
    }
}

fn check_cache(t: &mut Tok, cache: &Arc<Mutex<MeshRepairCache>>, at: &str) {
    let c = cache.lock().unwrap();
    let attempt = match c.win10_attempt {
        MeshRepairAttempt::NotAttempted => 0,
        MeshRepairAttempt::Succeeded => 1,
        MeshRepairAttempt::Failed => 2,
    };
    assert_eq!(attempt, t.u(), "{at}: cache win10_attempt");
    assert_eq!(c.has_prepared_mesh as usize, t.u(), "{at}: cache has_prepared_mesh");
    assert_eq!(mesh_diff(&c.mesh, &t.mesh()), None, "{at}: cache mesh");
    assert!(c.face_colors == t.colors(), "{at}: cache face_colors");
    assert_eq!(
        [c.key_input_face_count, c.key_oversampling_iters, c.key_oversampling_min_face_count, c.key_oversampling_max_face_count],
        [t.u(), t.u(), t.u(), t.u()],
        "{at}: cache keys"
    );
}

/// Runs every line of a vectors file; the number of lines per tag is returned.
fn run(text: &str) -> std::collections::BTreeMap<String, usize> {
    let mut counts = std::collections::BTreeMap::new();
    let mut mesh = TriMesh::default();
    let mut texture = Mat::default();
    let mut cache: Option<Arc<Mutex<MeshRepairCache>>> = None;
    for (lineno, line) in text.lines().enumerate() {
        let mut t = Tok(line.split_whitespace());
        let tag = t.s();
        *counts.entry(tag.to_string()).or_insert(0) += 1;
        let at = format!("line {} ({})", lineno + 1, tag);
        match tag {
            "mesh" => {
                mesh = t.mesh();
            }
            "texture" => {
                texture = t.texture();
            }
            "get_pixel_color" => {
                let (u, v) = (t.f(), t.f());
                t.arrow();
                assert_eq!(get_pixel_color(u, v, &texture), t.u3(), "{at}");
            }
            "sample_face_color" => {
                let uvs: FaceUVArray = [t.uv(), t.uv(), t.uv()];
                t.arrow();
                assert_eq!(sample_face_color(&uvs, &texture), t.u3(), "{at}");
            }
            "count_cluster_label_usage" => {
                let labels = t.labels();
                let k = t.u();
                t.arrow();
                assert_eq!(count_cluster_label_usage(&labels, k), t.labels(), "{at}");
            }
            "discard_unused_cluster_centers" => {
                let mut centers = t.colors();
                let mut labels = t.labels();
                t.arrow();
                let ok = discard_unused_cluster_centers(&mut centers, &mut labels, "test");
                assert_eq!(ok as usize, t.u(), "{at}");
                assert_eq!(centers, t.colors(), "{at}: centers");
                assert_eq!(labels, t.labels(), "{at}: labels");
            }
            "ensure_all_cluster_centers_used" => {
                let source = t.colors();
                let centers = t.colors();
                let mut labels = t.labels();
                t.arrow();
                let ok = ensure_all_cluster_centers_used(&source, &centers, &mut labels, "test");
                assert_eq!(ok as usize, t.u(), "{at}");
                assert_eq!(labels, t.labels(), "{at}: labels");
            }
            "matches_input" => {
                let mut c = MeshRepairCache::default();
                c.has_prepared_mesh = t.u() != 0;
                c.key_input_face_count = 10;
                c.key_oversampling_iters = 2;
                c.key_oversampling_min_face_count = 100;
                c.key_oversampling_max_face_count = 1000;
                let (a, b, d, e) = (t.u(), t.u(), t.u(), t.u());
                t.arrow();
                assert_eq!(c.matches_input(a, b, d, e) as usize, t.u(), "{at}");
            }
            "SaveToOFF" => {
                let colors = t.colors();
                t.arrow();
                let dir = std::env::temp_dir().join(format!("slicer_save_to_off_{}_{}", std::process::id(), lineno));
                let path = dir.join("sub").join("gen.off");
                SaveToOFF(path.to_str().unwrap(), &mesh, &colors);
                let got = std::fs::read(&path).expect("the file");
                let _ = std::fs::remove_dir_all(&dir);
                let len = t.u();
                let want: Vec<u8> = (0..len).map(|_| t.u() as u8).collect();
                assert_eq!(String::from_utf8_lossy(&got), String::from_utf8_lossy(&want), "{at}");
            }
            "linear_subdivision" => {
                let mut uvs = t.face_uvs();
                let with_cb = t.u() != 0;
                t.arrow();
                let seen = std::cell::RefCell::new(Vec::<usize>::new());
                let cb = |p: i32| seen.borrow_mut().push(p as usize);
                let mut m = mesh.clone();
                let ok = linear_subdivision(&mut m, &mut uvs, if with_cb { Some(&cb) } else { None });
                assert_eq!(ok as usize, t.u(), "{at}");
                assert_eq!(mesh_diff(&m, &t.mesh()), None, "{at}");
                assert!(uv_bits(&uvs) == uv_bits(&t.face_uvs()), "{at}: uvs");
                assert_eq!(*seen.borrow(), t.labels(), "{at}: progress");
            }
            "quantize_colors" => {
                let mut s = TextureToColorSettings::default();
                s.target_colors_num = t.u();
                s.max_color_distance = t.d();
                s.max_cluster_k = t.u();
                let cancel = Cancel::new(t.u());
                let colors = t.colors();
                t.arrow();
                let mut centers: Vec<RGB> = vec![[1, 2, 3]];
                let mut ids: Vec<usize> = vec![9];
                let ok = quantize_colors(&colors, &s, &cancel.callback(), &mut centers, &mut ids);
                assert_eq!(ok as usize, t.u(), "{at}");
                assert_eq!(centers, t.colors(), "{at}: centers");
                assert_eq!(ids, t.labels(), "{at}: ids");
                assert_eq!(cancel.calls(), t.u(), "{at}: calls of the cancel callback");
            }
            "quantize_vertex_colors" => {
                let mut s = TextureToColorSettings::default();
                s.target_colors_num = t.u();
                let n = t.u();
                let vc: Vec<VertexColor> = (0..n).map(|_| [t.f(), t.f(), t.f(), t.f()]).collect();
                t.arrow();
                let mut centers: Vec<RGB> = Vec::new();
                let mut ids: Vec<usize> = Vec::new();
                let ok = quantize_vertex_colors(&vc, &s, &None, &mut centers, &mut ids);
                assert_eq!(ok as usize, t.u(), "{at}");
                assert_eq!(centers, t.colors(), "{at}: centers");
                assert_eq!(ids, t.labels(), "{at}: ids");
            }
            "adaptive_split_by_vertex_clusters" => {
                let ids = t.labels();
                let centers = t.colors();
                t.arrow();
                let mut m = mesh.clone();
                let mut out: Vec<RGB> = vec![[4, 5, 6]];
                let ok = adaptive_split_by_vertex_clusters(&mut m, &ids, &centers, &mut out);
                assert_eq!(ok as usize, t.u(), "{at}");
                assert_eq!(mesh_diff(&m, &t.mesh()), None, "{at}");
                assert_eq!(out, t.colors(), "{at}: colors");
            }
            "repair_cluster_smooth" => {
                let mut colors = t.colors();
                let mut s = t.settings();
                let with_cache = t.u() != 0;
                let round = t.u();
                let cancel = Cancel::new(t.u());
                let key = t.u();
                t.arrow();
                if with_cache {
                    if round == 0 {
                        cache = Some(Arc::new(Mutex::new(MeshRepairCache::default())));
                    }
                    s.mesh_repair_cache = cache.clone();
                }
                let ev = Events::new();
                let mut m = mesh.clone();
                let mut out: Vec<RGB> = vec![[7, 7, 7]];
                let ok = repair_cluster_smooth(&mut m, &mut colors, &mut out, &s, &ev.callback(), &cancel.callback(), "test", key, &NoRepair);
                assert_eq!(ok as usize, t.u(), "{at}");
                assert_eq!(mesh_diff(&m, &t.mesh()), None, "{at}: mesh");
                assert!(colors == t.colors(), "{at}: face colors");
                assert!(out == t.colors(), "{at}: clustered face colors");
                assert_eq!(ev.get(), t.events(), "{at}: progress");
                assert_eq!(cancel.calls(), t.u(), "{at}: calls of the cancel callback");
                if with_cache {
                    check_cache(&mut t, cache.as_ref().unwrap(), &at);
                }
            }
            "repair_cluster_smooth_with_callback" => {
                let mut colors = t.colors();
                let mut s = t.settings();
                let with_cache = t.u() != 0;
                let round = t.u();
                let cancel = Cancel::new(t.u());
                let key = t.u();
                let mode = t.u();
                let cancel_queries = t.u();
                let repaired = t.mesh();
                t.arrow();
                // The callback of the generator, replayed: two messages, `cancel_queries` questions
                // to the cancel callback, then the repaired mesh of the line or a failure.
                let input = mesh.clone();
                s.mesh_repair_callback = Some(Arc::new(
                    move |its: &slicer::triangle_set_sampling::indexed_triangle_set,
                          out: &mut slicer::triangle_set_sampling::indexed_triangle_set,
                          progress: &dyn Fn(Option<&str>, u32),
                          cancel: &dyn Fn() -> bool,
                          error: &mut String|
                          -> bool {
                        assert_eq!(mesh_diff(&TriMesh::from(its.clone()), &input), None, "the mesh given to the repair callback");
                        progress(Some("Repairing on Windows"), 10);
                        progress(None, 50);
                        for _ in 0..cancel_queries {
                            if cancel() {
                                *error = "cancelled".to_string();
                                return false;
                            }
                        }
                        if mode == 2 {
                            *error = "no repair".to_string();
                            return false;
                        }
                        *out = repaired.0.clone();
                        true
                    },
                ));
                if with_cache {
                    if round == 0 {
                        cache = Some(Arc::new(Mutex::new(MeshRepairCache::default())));
                    }
                    s.mesh_repair_cache = cache.clone();
                }
                let ev = Events::new();
                let mut m = mesh.clone();
                let mut out: Vec<RGB> = vec![[7, 7, 7]];
                let ok = repair_cluster_smooth(&mut m, &mut colors, &mut out, &s, &ev.callback(), &cancel.callback(), "test", key, &NoRepair);
                assert_eq!(ok as usize, t.u(), "{at}");
                assert_eq!(mesh_diff(&m, &t.mesh()), None, "{at}: mesh");
                assert!(colors == t.colors(), "{at}: face colors");
                assert!(out == t.colors(), "{at}: clustered face colors");
                assert_eq!(ev.get(), t.events(), "{at}: progress");
                assert_eq!(cancel.calls(), t.u(), "{at}: calls of the cancel callback");
                if with_cache {
                    check_cache(&mut t, cache.as_ref().unwrap(), &at);
                }
            }
            "TextureToColor" => {
                let n = t.u();
                let uvs: Vec<Vec<Vec2f>> = (0..n)
                    .map(|_| {
                        let k = t.u();
                        (0..k).map(|_| t.uv()).collect()
                    })
                    .collect();
                let mut s = t.settings();
                let with_cache = t.u() != 0;
                let round = t.u();
                let cancel = Cancel::new(t.u());
                t.arrow();
                if with_cache {
                    if round == 0 {
                        cache = Some(Arc::new(Mutex::new(MeshRepairCache::default())));
                    }
                    s.mesh_repair_cache = cache.clone();
                }
                let ev = Events::new();
                let mut out_mesh = mesh.clone();
                let mut out: Vec<RGB> = vec![[7, 7, 7]];
                let ok = TextureToColor(&mesh, &uvs, &texture, &mut out_mesh, &mut out, &s, &ev.callback(), &cancel.callback(), &NoRepair);
                assert_eq!(ok as usize, t.u(), "{at}");
                assert_eq!(mesh_diff(&out_mesh, &t.mesh()), None, "{at}: mesh");
                assert!(out == t.colors(), "{at}: face colors");
                assert_eq!(ev.get(), t.events(), "{at}: progress");
                assert_eq!(cancel.calls(), t.u(), "{at}: calls of the cancel callback");
                if with_cache {
                    check_cache(&mut t, cache.as_ref().unwrap(), &at);
                }
            }
            "ClusterAndSmooth" => {
                let colors = t.colors();
                let s = t.settings();
                let cancel = Cancel::new(t.u());
                let n = t.u();
                let vc: Vec<[f32; 4]> = (0..n).map(|_| [t.f(), t.f(), t.f(), t.f()]).collect();
                t.arrow();
                let ev = Events::new();
                let mut out_mesh = TriMesh::default();
                let mut out: Vec<RGB> = vec![[7, 7, 7]];
                let ok = ClusterAndSmooth(&mesh, &colors, &mut out_mesh, &mut out, &s, &ev.callback(), &cancel.callback(), &vc, &NoRepair);
                assert_eq!(ok as usize, t.u(), "{at}");
                assert_eq!(mesh_diff(&out_mesh, &t.mesh()), None, "{at}: mesh");
                assert!(out == t.colors(), "{at}: face colors");
                assert_eq!(ev.get(), t.events(), "{at}: progress");
                assert_eq!(cancel.calls(), t.u(), "{at}: calls of the cancel callback");
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
fn sampling_and_cluster_bookkeeping_match_the_pin() {
    let counts = run(include_str!("data/texture_to_color_texture_to_color/vectors_sampling.txt"));
    expect(
        &counts,
        &[
            ("texture", 24),
            ("get_pixel_color", 960),
            ("sample_face_color", 480),
            ("count_cluster_label_usage", 60),
            ("discard_unused_cluster_centers", 60),
            ("ensure_all_cluster_centers_used", 60),
            ("matches_input", 8),
            ("mesh", 6),
            ("SaveToOFF", 6),
        ],
    );
}

#[test]
fn subdivision_quantization_and_split_match_the_pin() {
    let counts = run(include_str!("data/texture_to_color_texture_to_color/vectors_split.txt"));
    expect(
        &counts,
        &[
            ("mesh", 54),
            ("linear_subdivision", 14),
            ("quantize_colors", 40),
            ("quantize_vertex_colors", 20),
            ("adaptive_split_by_vertex_clusters", 40),
        ],
    );
}

#[test]
fn pipeline_matches_the_pin() {
    let counts = run(include_str!("data/texture_to_color_texture_to_color/vectors_pipeline.txt"));
    expect(
        &counts,
        &[("mesh", 96), ("texture", 26), ("repair_cluster_smooth", 40), ("TextureToColor", 32), ("ClusterAndSmooth", 24)],
    );
}

#[test]
fn pipeline_with_a_repair_callback_matches_the_pin() {
    let counts = run(include_str!("data/texture_to_color_texture_to_color/vectors_repair.txt"));
    expect(&counts, &[("mesh", 60), ("repair_cluster_smooth_with_callback", 60)]);
}
