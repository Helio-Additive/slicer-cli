//! `texture_painting.rs` against Bambu Studio's `TexturePainting.cpp` at the pin.
//!
//! `tests/data/texture_painting/vectors_*.txt` are written by `gen.cpp` of that directory, which
//! includes the C++ unit unchanged and links the rest of the engine (how to build it is in its
//! header). Each line is one call with its arguments and its result; the test makes the same call
//! and requires the same result: meshes and UVs bit for bit, colors, pixels, progress reports,
//! the questions to the cancel callback, and for `apply_painted_mesh_to_volume` the mesh, shifts,
//! offset and paintings of the volume.
//!
//! The meshes of the vectors are halfedge-compatible, so `RepairMesh` (CGAL, reached through
//! `PolygonMeshProcessing`) is not called; the implementation below fails the test if it is.

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};

use slicer::model::FacetsAnnotation;
use slicer::texture_painting::*;
use slicer::texture_to_color::cgal_utils::{CGALMesh, PolygonMeshProcessing, Point_3, SM_Face_index, SM_Halfedge_index};
use slicer::texture_to_color::texture_to_color::{Mat, MeshRepairAttempt, MeshRepairCache};
use slicer::texture_to_color::tri_mesh::TriMesh;
use slicer::triangle_mesh::Vec2f;
use slicer::triangle_set_sampling::{indexed_triangle_set, Vec3d, Vec3f, Vec3i};

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
    fn u3(&mut self) -> RGB {
        [self.u(), self.u(), self.u()]
    }
    fn i3(&mut self) -> [i32; 3] {
        [self.i(), self.i(), self.i()]
    }
    fn f2(&mut self) -> [f32; 2] {
        [self.f(), self.f()]
    }
    fn f3(&mut self) -> [f32; 3] {
        [self.f(), self.f(), self.f()]
    }
    fn f4(&mut self) -> [f32; 4] {
        [self.f(), self.f(), self.f(), self.f()]
    }
    fn d3(&mut self) -> Vec3d {
        Vec3d::new(self.d(), self.d(), self.d())
    }
    fn arrow(&mut self) {
        assert_eq!(self.s(), "->");
    }
    fn bytes(&mut self) -> Vec<u8> {
        let hex = self.s();
        if hex == "-" {
            return Vec::new();
        }
        (0..hex.len() / 2).map(|k| u8::from_str_radix(&hex[2 * k..2 * k + 2], 16).unwrap()).collect()
    }
    fn colors(&mut self) -> Vec<RGB> {
        let n = self.u();
        (0..n).map(|_| self.u3()).collect()
    }
    fn texture(&mut self) -> Mat {
        let rows = self.i();
        let cols = self.i();
        let channels = self.i();
        let step = self.u();
        let data = self.bytes();
        assert_eq!(data.len(), rows as usize * step);
        Mat { rows, cols, channels, step, data }
    }
    fn mesh(&mut self) -> TriMesh {
        let nv = self.u();
        let vertices: Vec<Vec3f> = (0..nv).map(|_| Vec3f::new(self.f(), self.f(), self.f())).collect();
        let nf = self.u();
        let indices: Vec<Vec3i> = (0..nf).map(|_| Vec3i::new(self.i(), self.i(), self.i())).collect();
        TriMesh::new(indices, vertices)
    }
    fn uvs(&mut self) -> Vec<Vec<[u32; 2]>> {
        let n = self.u();
        (0..n)
            .map(|_| {
                let k = self.u();
                (0..k).map(|_| [self.s(), self.s()].map(|h| u32::from_str_radix(h, 16).unwrap())).collect()
            })
            .collect()
    }
    fn textured(&mut self) -> TexturedMesh {
        let mut m = TexturedMesh::default();
        let n = self.u();
        m.vertices = (0..n).map(|_| self.f3()).collect();
        let n = self.u();
        m.indices = (0..n).map(|_| self.i3()).collect();
        let n = self.u();
        m.uvs = (0..n).map(|_| self.f2()).collect();
        let n = self.u();
        m.textures = (0..n)
            .map(|_| {
                let (width, height, channels) = (self.i(), self.i(), self.i());
                let len = self.u();
                let data = self.bytes();
                assert_eq!(data.len(), len);
                TextureImage { width, height, channels, data }
            })
            .collect();
        let n = self.u();
        m.material_ids = (0..n).map(|_| self.i()).collect();
        let n = self.u();
        m.material_texture_map = (0..n).map(|_| self.i()).collect();
        let n = self.u();
        m.material_colors = (0..n).map(|_| self.f4()).collect();
        let n = self.u();
        m.uv_coords = (0..n).map(|_| self.f2()).collect();
        let n = self.u();
        m.uv_indices = (0..n).map(|_| self.i3()).collect();
        m.precomputed_face_colors = self.colors();
        let n = self.u();
        m.precomputed_vertex_colors = (0..n).map(|_| self.f4()).collect();
        m.is_standard_3mf = self.u() != 0;
        m
    }
    fn painted(&mut self) -> PaintedMesh {
        let mut p = PaintedMesh::default();
        let n = self.u();
        p.vertices = (0..n).map(|_| self.f3()).collect();
        let n = self.u();
        p.indices = (0..n).map(|_| self.i3()).collect();
        p.face_colors = self.colors();
        p.cluster_colors = self.colors();
        let n = self.u();
        for _ in 0..n {
            let c = self.u3();
            let v = self.i();
            p.original_color_to_cluster.insert(c, v);
        }
        p
    }
    fn cache(&mut self) -> MeshRepairCache {
        let mut c = MeshRepairCache::default();
        c.win10_attempt = match self.u() {
            0 => MeshRepairAttempt::NotAttempted,
            1 => MeshRepairAttempt::Succeeded,
            _ => MeshRepairAttempt::Failed,
        };
        c.has_prepared_mesh = self.u() != 0;
        c.mesh = self.mesh();
        c.face_colors = self.colors();
        c.key_input_face_count = self.u();
        c.key_oversampling_iters = self.u();
        c.key_oversampling_min_face_count = self.u();
        c.key_oversampling_max_face_count = self.u();
        c
    }
    fn settings(&mut self) -> TexturePaintingSettings {
        let mut s = TexturePaintingSettings::default();
        s.target_colors_num = self.u();
        s.smooth_weight = self.d();
        s.oversampling_iters = self.u();
        s.mesh_repair_decision = if self.u() != 0 { MeshRepairDecision::RepairAndImport } else { MeshRepairDecision::ImportWithoutRepair };
        s
    }
    fn events(&mut self) -> Vec<(i32, String)> {
        let n = self.u();
        (0..n)
            .map(|_| {
                let (p, m) = self.s().split_once('~').expect("percent~message");
                (p.parse().unwrap(), m.to_string())
            })
            .collect()
    }
    fn facets(&mut self) -> FacetsAnnotation {
        let n = self.u();
        let pairs: Vec<(i32, i32)> = (0..n).map(|_| (self.i(), self.i())).collect();
        let nbits = self.u();
        let bits = self.s();
        let bits: Vec<bool> = if bits == "-" { Vec::new() } else { bits.chars().map(|c| c == '1').collect() };
        assert_eq!(bits.len(), nbits);
        FacetsAnnotation { data: (pairs, bits) }
    }
    fn volume(&mut self) -> ModelVolume {
        let nv = self.u();
        let vertices: Vec<Vec3f> = (0..nv).map(|_| Vec3f::new(self.f(), self.f(), self.f())).collect();
        let nf = self.u();
        let indices: Vec<Vec3i> = (0..nf).map(|_| Vec3i::new(self.i(), self.i(), self.i())).collect();
        let mut mesh = VolumeMesh::new(indexed_triangle_set { vertices, indices });
        mesh.init_shift = self.d3();
        let offset = self.d3();
        let mesh_offset = self.d3();
        let is_converted_from_inches = self.u() != 0;
        let is_converted_from_meters = self.u() != 0;
        ModelVolume {
            source: ModelVolumeSource { mesh_offset, is_converted_from_inches, is_converted_from_meters },
            mesh,
            convex_hull: None,
            convex_hull_2d: vec![slicer::Point::new(1, 2)],
            offset,
            supported_facets: self.facets(),
            seam_facets: self.facets(),
            mmu_segmentation_facets: self.facets(),
            fuzzy_skin_facets: self.facets(),
            object: Some(ModelObject { bounding_box_valid: true, raw_bounding_box_valid: true, raw_mesh_bounding_box_valid: true }),
        }
    }
    fn end(&mut self) {
        assert!(self.0.next().is_none(), "tokens left on the line");
    }
}

fn f32_bits3(v: &[[f32; 3]]) -> Vec<[u32; 3]> {
    v.iter().map(|p| [p[0].to_bits(), p[1].to_bits(), p[2].to_bits()]).collect()
}

/// The first difference of two painted meshes, in words; `None` when they are equal bit for bit.
fn painted_diff(a: &PaintedMesh, b: &PaintedMesh) -> Option<String> {
    if f32_bits3(&a.vertices) != f32_bits3(&b.vertices) {
        return Some(format!("vertices: {} against {}", a.vertices.len(), b.vertices.len()));
    }
    if a.indices != b.indices {
        return Some("indices".into());
    }
    if a.face_colors != b.face_colors {
        return Some("face colors".into());
    }
    if a.cluster_colors != b.cluster_colors {
        return Some(format!("cluster colors {:?} against {:?}", a.cluster_colors, b.cluster_colors));
    }
    if a.original_color_to_cluster != b.original_color_to_cluster {
        return Some(format!("original_color_to_cluster {:?} against {:?}", a.original_color_to_cluster, b.original_color_to_cluster));
    }
    None
}

fn mesh_diff(a: &TriMesh, b: &TriMesh) -> Option<String> {
    let av: Vec<[u32; 3]> = a.vertices.iter().map(|v| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]).collect();
    let bv: Vec<[u32; 3]> = b.vertices.iter().map(|v| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]).collect();
    if av != bv {
        return Some(format!("vertices: {} against {}", av.len(), bv.len()));
    }
    if a.indices != b.indices {
        return Some("indices".into());
    }
    None
}

fn cache_diff(a: &MeshRepairCache, b: &MeshRepairCache) -> Option<String> {
    if a.win10_attempt != b.win10_attempt || a.has_prepared_mesh != b.has_prepared_mesh {
        return Some("attempt / prepared".into());
    }
    if let Some(d) = mesh_diff(&a.mesh, &b.mesh) {
        return Some(format!("mesh: {d}"));
    }
    if a.face_colors != b.face_colors {
        return Some("face colors".into());
    }
    let ka = [a.key_input_face_count, a.key_oversampling_iters, a.key_oversampling_min_face_count, a.key_oversampling_max_face_count];
    let kb = [b.key_input_face_count, b.key_oversampling_iters, b.key_oversampling_min_face_count, b.key_oversampling_max_face_count];
    if ka != kb {
        return Some(format!("keys {ka:?} against {kb:?}"));
    }
    None
}

/// The matrices compared as OpenCV prints them: an empty matrix has one channel there.
fn mat_key(m: &Mat) -> (i32, i32, i32, usize, Vec<u8>) {
    let channels = if m.empty() && m.channels == 0 { 1 } else { m.channels };
    (m.rows, m.cols, channels, m.step, m.data.clone())
}

fn uv_bits(v: &[Vec<Vec2f>]) -> Vec<Vec<[u32; 2]>> {
    v.iter().map(|f| f.iter().map(|u| [u.x.to_bits(), u.y.to_bits()]).collect()).collect()
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
    fn callback(&self) -> PaintProgressCallback {
        let seen = self.0.clone();
        Some(Arc::new(move |p: i32, m: &str| seen.lock().unwrap().push((p, m.replace(' ', "_")))))
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
    fn callback(&self) -> PaintCancelCallback {
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

fn check_volume(got: &ModelVolume, want: &ModelVolume, at: &str) {
    let gv: Vec<[u32; 3]> = got.mesh.its.vertices.iter().map(|v| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]).collect();
    let wv: Vec<[u32; 3]> = want.mesh.its.vertices.iter().map(|v| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]).collect();
    assert!(gv == wv, "{at}: volume vertices");
    assert_eq!(got.mesh.its.indices, want.mesh.its.indices, "{at}: volume faces");
    let bits = |v: &Vec3d| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()];
    assert_eq!(bits(&got.mesh.init_shift), bits(&want.mesh.init_shift), "{at}: init shift");
    assert_eq!(bits(&got.offset), bits(&want.offset), "{at}: offset");
    assert_eq!(bits(&got.source.mesh_offset), bits(&want.source.mesh_offset), "{at}: source mesh offset");
    assert_eq!(got.source, want.source, "{at}: source");
    assert_eq!(got.supported_facets, want.supported_facets, "{at}: supported facets");
    assert_eq!(got.seam_facets, want.seam_facets, "{at}: seam facets");
    assert_eq!(got.mmu_segmentation_facets, want.mmu_segmentation_facets, "{at}: mmu segmentation facets");
    assert_eq!(got.fuzzy_skin_facets, want.fuzzy_skin_facets, "{at}: fuzzy skin facets");
}

/// Runs every line of a vectors file; the number of lines per tag is returned.
fn run(text: &str) -> BTreeMap<String, usize> {
    let mut counts = BTreeMap::new();
    let mut textured = TexturedMesh::default();
    let mut texture = Mat::default();
    let mut volume: Option<ModelVolume> = None;
    let mut cache: MeshRepairCachePtr = None;
    for (lineno, line) in text.lines().enumerate() {
        let mut t = Tok(line.split_whitespace());
        let tag = t.s();
        *counts.entry(tag.to_string()).or_insert(0) += 1;
        let at = format!("line {} ({})", lineno + 1, tag);
        match tag {
            "textured" => textured = t.textured(),
            "texture" => texture = t.texture(),
            "volume" => volume = Some(t.volume()),
            "decode_texture_image" => {
                let (width, height, channels) = (t.i(), t.i(), t.i());
                let len = t.u();
                let data = t.bytes();
                assert_eq!(data.len(), len);
                let img = TextureImage { width, height, channels, data };
                t.arrow();
                assert_eq!(mat_key(&decode_texture_image(&img)), mat_key(&t.texture()), "{at}");
                let mut px: Vec<u8> = vec![1, 2];
                let (mut w, mut h) = (-5, -6);
                let ok = decode_texture_to_pixels(&img, &mut px, &mut w, &mut h);
                assert_eq!((ok as usize, w, h), (t.u(), t.i(), t.i()), "{at}: decode_texture_to_pixels");
                let len = t.u();
                let want = t.bytes();
                assert_eq!(want.len(), len);
                assert_eq!(px, want, "{at}: pixels");
            }
            "build_tex2color_mesh" => {
                t.arrow();
                let mut mesh = TriMesh::default();
                let mut uvs: Vec<Vec<Vec2f>> = Vec::new();
                build_tex2color_mesh(&textured, &mut mesh, &mut uvs);
                assert_eq!(mesh_diff(&mesh, &t.mesh()), None, "{at}");
                assert_eq!(uv_bits(&uvs), t.uvs(), "{at}: uvs");
            }
            "build_multi_texture_atlas" => {
                t.arrow();
                let mut atlas = Mat::default();
                let mut uvs: Vec<Vec<Vec2f>> = Vec::new();
                let ok = build_multi_texture_atlas(&textured, &mut atlas, &mut uvs);
                assert_eq!(ok as usize, t.u(), "{at}");
                assert_eq!(mat_key(&atlas), mat_key(&t.texture()), "{at}: atlas");
                assert_eq!(uv_bits(&uvs), t.uvs(), "{at}: uvs");
            }
            "sample_original_face_colors" => {
                t.arrow();
                let mut out: Vec<RGB> = vec![[1, 2, 3]];
                let ok = sample_original_face_colors(&textured, &mut out);
                assert_eq!(ok as usize, t.u(), "{at}");
                assert_eq!(out, t.colors(), "{at}: colors");
            }
            "sample_face_from_uvs" => {
                let (a, b, c) = (t.f2(), t.f2(), t.f2());
                t.arrow();
                assert_eq!(sample_face_from_uvs(&texture, &a, &b, &c), t.u3(), "{at}");
            }
            "sample_face_from_texture" => {
                let n = t.u();
                let uvs: Vec<[f32; 2]> = (0..n).map(|_| t.f2()).collect();
                let f = t.i3();
                t.arrow();
                assert_eq!(sample_face_from_texture(&texture, &uvs, &f), t.u3(), "{at}");
            }
            "extract_painted_mesh" => {
                let mesh = t.mesh();
                let colors = t.colors();
                t.arrow();
                let mut painted = PaintedMesh::default();
                painted.original_color_to_cluster.insert([1, 2, 3], 4);
                painted.vertices.push([7.0, 7.0, 7.0]);
                extract_painted_mesh(&mesh, &colors, &mut painted);
                assert_eq!(painted_diff(&painted, &t.painted()), None, "{at}");
            }
            "compute_delta_e" => {
                let (a, b, f) = (t.u3(), t.u3(), t.f4());
                t.arrow();
                assert_eq!(compute_delta_e(&a, &f).to_bits(), t.d().to_bits(), "{at}: against RGBA");
                assert_eq!(compute_delta_e_rgb(&a, &b).to_bits(), t.d().to_bits(), "{at}: against RGB");
            }
            "match_clusters_to_filaments" => {
                let clusters = t.colors();
                let n = t.u();
                let fils: Vec<[f32; 4]> = (0..n).map(|_| t.f4()).collect();
                t.arrow();
                let got = match_clusters_to_filaments(&clusters, &fils, &vec!["x".to_string(); n]);
                let n = t.u();
                assert_eq!(got.len(), n, "{at}");
                for m in got.iter() {
                    assert_eq!((m.cluster_index, m.filament_index), (t.i(), t.i()), "{at}");
                    assert_eq!(m.delta_e.to_bits(), t.d().to_bits(), "{at}: delta_e");
                    assert_eq!(m.cluster_color, t.u3(), "{at}: cluster color");
                    assert_eq!(m.filament_color.map(f32::to_bits), t.f4().map(f32::to_bits), "{at}: filament color");
                }
            }
            "cache" => {
                let c: MeshRepairCachePtr = if t.u() != 0 { Some(Arc::new(Mutex::new(t.cache()))) } else { None };
                let mut s = TexturePaintingSettings::default();
                s.oversampling_iters = t.u();
                let faces = t.u();
                t.arrow();
                assert_eq!(mesh_repair_cache_win10_failed(&c) as usize, t.u(), "{at}: mesh_repair_cache_win10_failed");
                let mut out = PaintedMesh::default();
                out.vertices.push([5.0, 5.0, 5.0]);
                let copied = copy_prepared_mesh_from_cache(&c, &mut out);
                assert_eq!(copied as usize, t.u(), "{at}: copy_prepared_mesh_from_cache");
                assert_eq!(painted_diff(&out, &t.painted()), None, "{at}: copy_prepared_mesh_from_cache");
                assert_eq!(mesh_repair_cache_can_skip_input(&c, faces, &s) as usize, t.u(), "{at}: mesh_repair_cache_can_skip_input");
                assert_eq!(c.is_some() as usize, t.u(), "{at}");
                if let Some(c) = &c {
                    assert_eq!(cache_diff(&c.lock().unwrap(), &t.cache()), None, "{at}: cache after");
                }
            }
            "texture_to_painting" => {
                let mut s = t.settings();
                let with_cache = t.u() != 0;
                let round = t.u();
                let cancel = Cancel::new(t.u());
                let no_progress = t.u() != 0;
                t.arrow();
                if with_cache {
                    if round == 0 {
                        cache = make_mesh_repair_cache();
                    }
                    s.mesh_repair_cache = cache.clone();
                }
                let ev = Events::new();
                let mut painted = PaintedMesh::default();
                painted.original_color_to_cluster.insert([9, 9, 9], 1);
                let progress = if no_progress { None } else { ev.callback() };
                let ok = texture_to_painting(&textured, &mut painted, &s, &progress, &cancel.callback(), &NoRepair);
                assert_eq!(ok as usize, t.u(), "{at}");
                assert_eq!(painted_diff(&painted, &t.painted()), None, "{at}");
                assert_eq!(ev.get(), t.events(), "{at}: progress");
                assert_eq!(cancel.calls(), t.u(), "{at}: calls of the cancel callback");
                if with_cache {
                    assert_eq!(cache_diff(&cache.as_ref().unwrap().lock().unwrap(), &t.cache()), None, "{at}: cache");
                }
            }
            "cluster_from_repair_cache" => {
                let mut s = t.settings();
                t.arrow();
                s.mesh_repair_cache = cache.clone();
                let ev = Events::new();
                let mut painted = PaintedMesh::default();
                let ok = cluster_from_repair_cache(&mut painted, &s, &ev.callback(), &None, &NoRepair);
                assert_eq!(ok as usize, t.u(), "{at}");
                assert_eq!(painted_diff(&painted, &t.painted()), None, "{at}");
                assert_eq!(ev.get(), t.events(), "{at}: progress");
                assert_eq!(cache_diff(&cache.as_ref().unwrap().lock().unwrap(), &t.cache()), None, "{at}: cache");
            }
            "face_colors_to_painting" => {
                let s = t.settings();
                let cancel = Cancel::new(t.u());
                t.arrow();
                let ev = Events::new();
                let mut painted = PaintedMesh::default();
                let ok = face_colors_to_painting(&textured, &mut painted, &s, &ev.callback(), &cancel.callback(), &NoRepair);
                assert_eq!(ok as usize, t.u(), "{at}");
                assert_eq!(painted_diff(&painted, &t.painted()), None, "{at}");
                assert_eq!(ev.get(), t.events(), "{at}: progress");
                assert_eq!(cancel.calls(), t.u(), "{at}: calls of the cancel callback");
            }
            "standard_3mf_colors_to_painting" => {
                let mut s = TexturePaintingSettings::default();
                s.target_colors_num = t.u();
                let cancel = Cancel::new(t.u());
                let no_progress = t.u() != 0;
                t.arrow();
                let ev = Events::new();
                let mut painted = PaintedMesh::default();
                painted.original_color_to_cluster.insert([9, 9, 9], 1);
                let progress = if no_progress { None } else { ev.callback() };
                let ok = standard_3mf_colors_to_painting(&textured, &mut painted, &s, &progress, &cancel.callback());
                assert_eq!(ok as usize, t.u(), "{at}");
                assert_eq!(painted_diff(&painted, &t.painted()), None, "{at}");
                assert_eq!(ev.get(), t.events(), "{at}: progress");
                assert_eq!(cancel.calls(), t.u(), "{at}: calls of the cancel callback");
            }
            "apply_painted_mesh_to_volume" => {
                let painted = t.painted();
                let n = t.u();
                let matches: Vec<FilamentMatch> = (0..n)
                    .map(|_| FilamentMatch { cluster_index: t.i(), filament_index: t.i(), ..FilamentMatch::default() })
                    .collect();
                t.arrow();
                let mut vol = volume.take().expect("a volume line before");
                let ok = apply_painted_mesh_to_volume(&painted, &matches, &mut vol, &|_its: &indexed_triangle_set| indexed_triangle_set::default());
                assert_eq!(ok as usize, t.u(), "{at}");
                let want = t.volume();
                check_volume(&vol, &want, &at);
                if ok {
                    // Read off TexturePainting.cpp:749-771: the hull is (re)computed, the 2D hull
                    // and the object's bounding boxes are invalidated.
                    assert!(vol.convex_hull.is_some(), "{at}: convex hull");
                    assert!(vol.convex_hull_2d.is_empty(), "{at}: 2D convex hull");
                    let o = vol.object.as_ref().unwrap();
                    assert!(!o.bounding_box_valid && !o.raw_bounding_box_valid && !o.raw_mesh_bounding_box_valid, "{at}: object bounding box");
                } else {
                    assert!(vol.convex_hull.is_none() && !vol.convex_hull_2d.is_empty(), "{at}: untouched");
                }
            }
            other => panic!("unknown tag {other}"),
        }
        t.end();
    }
    counts
}

fn expect(counts: &BTreeMap<String, usize>, want: &[(&str, usize)]) {
    let got: Vec<(&str, usize)> = counts.iter().map(|(k, v)| (k.as_str(), *v)).collect();
    let mut want = want.to_vec();
    want.sort();
    assert_eq!(got, want);
}

#[test]
fn decoding_meshes_sampling_matching_and_cache_match_the_pin() {
    let counts = run(include_str!("data/texture_painting/vectors_basic.txt"));
    expect(
        &counts,
        &[
            ("decode_texture_image", 30),
            ("textured", 40),
            ("build_tex2color_mesh", 40),
            ("build_multi_texture_atlas", 40),
            ("sample_original_face_colors", 40),
            ("texture", 12),
            ("sample_face_from_uvs", 240),
            ("sample_face_from_texture", 120),
            ("extract_painted_mesh", 10),
            ("compute_delta_e", 40),
            ("match_clusters_to_filaments", 20),
            ("cache", 24),
        ],
    );
}

#[test]
fn painting_pipelines_match_the_pin() {
    let counts = run(include_str!("data/texture_painting/vectors_pipeline.txt"));
    expect(
        &counts,
        &[
            ("textured", 84),
            ("texture_to_painting", 37),
            ("cluster_from_repair_cache", 7),
            ("face_colors_to_painting", 24),
            ("standard_3mf_colors_to_painting", 30),
        ],
    );
}

#[test]
fn apply_painted_mesh_to_volume_matches_the_pin() {
    let counts = run(include_str!("data/texture_painting/vectors_volume.txt"));
    expect(&counts, &[("volume", 24), ("apply_painted_mesh_to_volume", 24)]);
}
