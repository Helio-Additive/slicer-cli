//! `paint_reproject.rs` against Bambu Studio's `PaintReproject.cpp` at the pin, for the functions
//! the port has so far.
//!
//! `tests/data/paint_reproject/vectors_*.txt` are written by `gen.cpp` of that directory, which
//! includes the C++ unit unchanged and links the rest of the engine (how to build it is in its
//! header). Each line is one call with its arguments and its result; the test makes the same call
//! and requires the same result, floating point numbers bit for bit.

use std::collections::BTreeMap;
use std::fmt::Write as _;

use slicer::geometry::Point3F;
use slicer::model::FacetsAnnotation;
use slicer::paint_reproject::*;
use slicer::texture_painting::EXTRUDER_MAX;
use slicer::triangle_mesh::{indexed_triangle_set, Triangle, TriangleMesh, Vec3f, Vec3i};
use slicer::aabb_tree_indirect::{build_aabb_tree_over_indexed_triangle_set_f, intersecting_f, squared_distance_to_indexed_triangle_set_f, traverse_f, BoundingBox3f, NodeF};
use slicer::triangle_selector::{EnforcerBlockerType, RegionCoverage, Transform3f, TriangleSelector};

/// The vectors are kept gzipped (`gzip -9n` of the generator's output).
fn read_vectors(rel: &str) -> String {
    use std::io::Read as _;
    let path = format!("{}/tests/data/{}.txt.gz", env!("CARGO_MANIFEST_DIR"), rel);
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    text
}

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
    fn v2(&mut self) -> Vec2d {
        Vec2d::new(self.d(), self.d())
    }
    fn v3(&mut self) -> Vec3f {
        Vec3f::new(self.f(), self.f(), self.f())
    }
    fn tri(&mut self) -> [Vec3f; 3] {
        [self.v3(), self.v3(), self.v3()]
    }
    fn poly(&mut self) -> Vec<Vec2d> {
        let n = self.u();
        (0..n).map(|_| self.v2()).collect()
    }
    fn trafo(&mut self) -> Transform3d {
        let mut t = Transform3d::zeros();
        for r in 0..4 {
            for c in 0..4 {
                t[(r, c)] = self.d();
            }
        }
        t
    }
    fn mesh(&mut self) -> indexed_triangle_set {
        let nv = self.u();
        let vertices: Vec<Vec3f> = (0..nv).map(|_| self.v3()).collect();
        let nf = self.u();
        let indices: Vec<Vec3i> = (0..nf).map(|_| Vec3i::new(self.i(), self.i(), self.i())).collect();
        indexed_triangle_set { vertices, indices }
    }
    fn data(&mut self) -> AnnotationData {
        let n = self.u();
        let pairs: Vec<(i32, i32)> = (0..n).map(|_| (self.i(), self.i())).collect();
        let nbits = self.u();
        let bits = self.s();
        let bits: Vec<bool> = if bits == "-" { Vec::new() } else { bits.chars().map(|c| c == '1').collect() };
        assert_eq!(bits.len(), nbits);
        (pairs, bits)
    }
    fn arrow(&mut self) {
        assert_eq!(self.s(), "->");
    }
    fn end(&mut self) {
        assert!(self.0.next().is_none(), "tokens left on the line");
    }
}

fn bits2(v: &Vec2d) -> [u64; 2] {
    [v.x.to_bits(), v.y.to_bits()]
}

fn bits3(v: &Vec3f) -> [u32; 3] {
    [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]
}

fn poly_bits(p: &[Vec2d]) -> Vec<[u64; 2]> {
    p.iter().map(bits2).collect()
}

fn tri_bits(t: &[Vec3f]) -> Vec<[u32; 3]> {
    t.iter().map(bits3).collect()
}

/// `TriangleMesh(its)` of the crate, as `TriangleSelector::new` takes it
fn selector_mesh(its: &indexed_triangle_set) -> TriangleMesh {
    TriangleMesh::from_parts(
        its.vertices.iter().map(|v| Point3F::new(v.x as f64, v.y as f64, v.z as f64)).collect(),
        its.indices.iter().map(|f| Triangle::new(f[0] as u32, f[1] as u32, f[2] as u32)).collect(),
    )
}

/// Runs every line of a vectors file; the number of lines per tag is returned.
fn run(text: &str) -> BTreeMap<String, usize> {
    let mut counts = BTreeMap::new();
    let mut mesh = indexed_triangle_set::default();
    let mut src_mesh = indexed_triangle_set::default();
    let mut annotation = FacetsAnnotation::default();
    for (lineno, line) in text.lines().enumerate() {
        let mut t = Tok(line.split_whitespace());
        let tag = t.s();
        *counts.entry(tag.to_string()).or_insert(0) += 1;
        let at = format!("line {} ({})", lineno + 1, tag);
        match tag {
            "mesh" => mesh = t.mesh(),
            "annotation" => {
                annotation = FacetsAnnotation { data: t.data() };
                src_mesh = mesh.clone();
            }
            "cross_2d" => {
                let (a, b) = (t.v2(), t.v2());
                t.arrow();
                assert_eq!(cross_2d(&a, &b).to_bits(), t.d().to_bits(), "{at}");
            }
            "project_to_2d" => {
                let p = t.v3();
                let axis = t.i();
                t.arrow();
                assert_eq!(bits2(&project_to_2d(&p, axis)), bits2(&t.v2()), "{at}");
            }
            "polygon_signed_area" => {
                let p = t.poly();
                t.arrow();
                assert_eq!(polygon_signed_area(&p).to_bits(), t.d().to_bits(), "{at}");
            }
            "clip_polygon_by_triangle" => {
                let p = t.poly();
                let clip = [t.v2(), t.v2(), t.v2()];
                t.arrow();
                assert_eq!(poly_bits(&clip_polygon_by_triangle(p, clip)), poly_bits(&t.poly()), "{at}");
            }
            "triangle_overlap_area" => {
                let (l, r) = (t.tri(), t.tri());
                t.arrow();
                let axis = dominant_projection_axis(&l);
                assert_eq!(axis, t.i(), "{at}: dominant_projection_axis");
                assert_eq!(triangle_overlap_area(&l, &r, axis).to_bits(), t.d().to_bits(), "{at}");
                assert_eq!(triangle_overlap_area(&l, &r, (axis + 1) % 3).to_bits(), t.d().to_bits(), "{at}: other axis");
            }
            "dominant_projection_axis" => {
                let tr = t.tri();
                t.arrow();
                assert_eq!(dominant_projection_axis(&tr), t.i(), "{at}");
            }
            "transform_triangle" => {
                let m = t.trafo();
                let tr = t.tri();
                t.arrow();
                assert_eq!(tri_bits(&transform_triangle(&tr, &m)), tri_bits(&t.tri()), "{at}");
            }
            "count_valid_nodes" => {
                t.arrow();
                let mut sel = TriangleSelector::new(selector_mesh(&src_mesh), 0.6);
                sel.deserialize(&annotation.data, true, EXTRUDER_MAX, EnforcerBlockerType::NONE, EnforcerBlockerType::NONE);
                assert_eq!(count_valid_nodes(&sel), t.u(), "{at}");
            }
            "annotation_bit_ranges" => {
                let a = FacetsAnnotation { data: t.data() };
                let rc = t.i();
                t.arrow();
                let n = t.u();
                let want: Vec<(i32, i32)> = (0..n).map(|_| (t.i(), t.i())).collect();
                assert_eq!(annotation_bit_ranges(&a, rc), want, "{at}");
            }
            "extract_source_roots" => {
                let n = t.u();
                let mask: Vec<i8> = (0..n).map(|_| t.u() as i8).collect();
                let no_ranges = t.u() != 0;
                t.arrow();
                let ranges = if no_ranges { Vec::new() } else { annotation_bit_ranges(&annotation, src_mesh.indices.len() as i32) };
                assert_eq!(extract_source_roots(&annotation, &mask, &ranges), t.data(), "{at}");
            }
            "splice_congruent_paint" => {
                let n = t.u();
                let mut map: Vec<i32> = Vec::new();
                let mut origin: Vec<CutFaceOrigin> = Vec::new();
                for _ in 0..n {
                    map.push(t.i());
                    origin.push(match t.u() {
                        0 => CutFaceOrigin::Invented,
                        1 => CutFaceOrigin::Congruent,
                        _ => CutFaceOrigin::Fragment,
                    });
                }
                let no_ranges = t.u() != 0;
                t.arrow();
                let ranges = if no_ranges { Vec::new() } else { annotation_bit_ranges(&annotation, src_mesh.indices.len() as i32) };
                assert_eq!(splice_congruent_paint(&annotation, &map, &origin, &ranges), t.data(), "{at}");
            }
            "append_shifted_annotation" => {
                let mut dst = FacetsAnnotation { data: t.data() };
                let offset = t.i();
                t.arrow();
                append_shifted_annotation(&mut dst, &annotation, offset);
                assert_eq!(dst.data, t.data(), "{at}");
            }
            "classify_cut_faces" => {
                let dst_to_src = t.trafo();
                let n = t.u();
                let map: Vec<i32> = (0..n).map(|_| t.i()).collect();
                let tol = t.f();
                t.arrow();
                let dst = mesh.clone();
                let dst_in_source = DestinationInSourceFrame::new(&dst, &dst_to_src);
                let c = classify_cut_faces(&src_mesh, &dst, &map, &dst_in_source, tol);
                let n = t.u();
                let origin: Vec<CutFaceOrigin> = (0..n)
                    .map(|_| match t.u() {
                        0 => CutFaceOrigin::Invented,
                        1 => CutFaceOrigin::Congruent,
                        _ => CutFaceOrigin::Fragment,
                    })
                    .collect();
                assert_eq!(c.origin, origin, "{at}: origin");
                let n = t.u();
                assert_eq!(c.fragment_faces, (0..n).map(|_| t.u() as i8).collect::<Vec<_>>(), "{at}: fragment faces");
                let n = t.u();
                assert_eq!(c.fragment_source_roots, (0..n).map(|_| t.u() as i8).collect::<Vec<_>>(), "{at}: fragment source roots");
                assert_eq!((c.congruent_count, c.fragment_count, c.invented_count), (t.u(), t.u(), t.u()), "{at}: counts");
                let n = t.u();
                let want: Vec<Vec3f> = (0..n).map(|_| t.v3()).collect();
                assert_eq!(tri_bits(dst_in_source.vertices()), tri_bits(&want), "{at}: vertices in the source frame");
                let tr = t.tri();
                assert_eq!(tri_bits(&dst_in_source.to_source(&tr)), tri_bits(&t.tri()), "{at}: to_source");
            }
            "face_samples" => {
                let f = t.i();
                t.arrow();
                let got = face_samples(&src_mesh, f);
                let want: Vec<Vec3f> = (0..4).map(|_| t.v3()).collect();
                assert_eq!(tri_bits(&got), tri_bits(&want), "{at}");
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
fn planar_helpers_match_the_pin() {
    let counts = run(include_str!("data/paint_reproject/vectors_planar.txt"));
    expect(
        &counts,
        &[
            ("cross_2d", 100),
            ("project_to_2d", 30),
            ("polygon_signed_area", 60),
            ("clip_polygon_by_triangle", 300),
            ("triangle_overlap_area", 200),
            ("dominant_projection_axis", 60),
            ("transform_triangle", 60),
        ],
    );
}

#[test]
fn annotation_splicing_and_cut_classification_match_the_pin() {
    let counts = run(include_str!("data/paint_reproject/vectors_annotation.txt"));
    expect(
        &counts,
        &[
            ("mesh", 60),
            ("annotation", 30),
            ("count_valid_nodes", 30),
            ("annotation_bit_ranges", 120),
            ("extract_source_roots", 30),
            ("splice_congruent_paint", 30),
            ("append_shifted_annotation", 30),
            ("classify_cut_faces", 30),
            ("face_samples", 180),
        ],
    );
}

// ---------------------------------------------------------------------------------------------
// The float AABB tree, the samplers and the reprojection drivers: each answer is compared as the
// generator prints it.
// ---------------------------------------------------------------------------------------------

#[derive(Default)]
struct Out(String);

impl Out {
    fn d(&mut self, v: f64) {
        write!(self.0, " {:016x}", v.to_bits()).unwrap();
    }
    fn f(&mut self, v: f32) {
        write!(self.0, " {:08x}", v.to_bits()).unwrap();
    }
    fn i(&mut self, v: i64) {
        write!(self.0, " {}", v).unwrap();
    }
    fn v3(&mut self, v: &Vec3f) {
        self.f(v.x);
        self.f(v.y);
        self.f(v.z);
    }
    fn bx(&mut self, b: &BoundingBox3f) {
        self.v3(&b.min);
        self.v3(&b.max);
    }
    fn data(&mut self, d: &AnnotationData) {
        self.i(d.0.len() as i64);
        for p in &d.0 {
            self.i(p.0 as i64);
            self.i(p.1 as i64);
        }
        self.i(d.1.len() as i64);
        self.0.push(' ');
        if d.1.is_empty() {
            self.0.push('-');
        }
        for &b in &d.1 {
            self.0.push(if b { '1' } else { '0' });
        }
    }
    fn trafof(&mut self, t: &Transform3f) {
        for r in 0..4 {
            for c in 0..4 {
                self.f(t[(r, c)]);
            }
        }
    }
}

fn check(at: &str, got: &Out, want: &str) {
    let g: Vec<&str> = got.0.split_whitespace().collect();
    let w: Vec<&str> = want.split_whitespace().collect();
    if g != w {
        let k = g.iter().zip(w.iter()).position(|(a, b)| a != b).unwrap_or(g.len().min(w.len()));
        panic!(
            "{at}: answers differ at token {k} (got {} tokens, want {}): got {:?}, want {:?}",
            g.len(),
            w.len(),
            &g[k.saturating_sub(3)..(k + 3).min(g.len())],
            &w[k.saturating_sub(3)..(k + 3).min(w.len())]
        );
    }
}

/// gen.cpp `Calls`: what the progress and cancel callbacks saw
#[derive(Default)]
struct Calls {
    progress: i64,
    percent_sum: i64,
    last: i64,
    cancel: i32,
    cancel_at: i32,
}

impl Calls {
    fn new(cancel_at: i32) -> Self {
        Calls { last: -1, cancel_at, ..Default::default() }
    }
    fn put(&self, o: &mut Out) {
        o.i(self.progress);
        o.i(self.percent_sum);
        o.i(self.last);
        o.i(self.cancel as i64);
    }
}

struct ReprojectState {
    mesh: indexed_triangle_set,
    src: &'static indexed_triangle_set,
    mmu: FacetsAnnotation,
    supported: FacetsAnnotation,
    seam: FacetsAnnotation,
    projector: Option<PaintReprojector<'static>>,
    region: Option<PaintRegionSource>,
    map: Vec<i32>,
    dst_to_src: Transform3d,
}

fn run_reproject_line(st: &mut ReprojectState, tag: &str, t: &mut Tok, want: &str, at: &str) {
    let mut o = Out::default();
    match tag {
        "mesh" => {
            st.mesh = t.mesh();
            return;
        }
        "annotation" => {
            st.mmu = FacetsAnnotation { data: t.data() };
            st.src = Box::leak(Box::new(st.mesh.clone()));
            st.projector = None;
            st.region = None;
            return;
        }
        "annotation2" => {
            st.supported = FacetsAnnotation { data: t.data() };
            st.seam = FacetsAnnotation { data: t.data() };
            return;
        }
        "aabb_build" => {
            let tree = build_aabb_tree_over_indexed_triangle_set_f(&st.mesh.vertices, &st.mesh.indices, 0.0);
            o.i(tree.nodes().len() as i64);
            for n in tree.nodes() {
                o.i(if n.idx == usize::MAX { -1 } else if n.idx == usize::MAX - 1 { -2 } else { n.idx as i64 });
                if n.is_valid() {
                    o.bx(&n.bbox);
                }
            }
        }
        "aabb_distance" => {
            let p = t.v3();
            let tree = build_aabb_tree_over_indexed_triangle_set_f(&st.mesh.vertices, &st.mesh.indices, 0.0);
            let mut hit = usize::MAX;
            let mut hp = Vec3f::zeros();
            let d = squared_distance_to_indexed_triangle_set_f(&st.mesh.vertices, &st.mesh.indices, &tree, &p, &mut hit, &mut hp);
            o.f(d);
            o.i(if hit == usize::MAX { -1 } else { hit as i64 });
            o.v3(&hp);
        }
        "aabb_traverse" => {
            let q = BoundingBox3f::new(t.v3(), t.v3());
            let tree = build_aabb_tree_over_indexed_triangle_set_f(&st.mesh.vertices, &st.mesh.indices, 0.0);
            let mut out: Vec<usize> = Vec::new();
            traverse_f(&tree, intersecting_f(q), |n: &NodeF| {
                out.push(n.idx);
                true
            });
            o.i(out.len() as i64);
            for x in out {
                o.i(x as i64);
            }
        }
        "measure_point_sample" => {
            let maxd = t.d();
            let tri = t.tri();
            let src = st.src;
            let mmu = &st.mmu;
            let pt = st.projector.get_or_insert_with(|| PaintReprojector::new(src, mmu, true, maxd));
            let m = pt.measure_point_sample(&tri, &tri);
            o.i(m.dominant_state.0 as i64);
            o.d(m.surface_area);
            o.d(m.error_area);
            o.i(pt.fallback_count() as i64);
        }
        "measure_overlap" => {
            let f = t.i();
            let tri = t.tri();
            let pt = st.projector.as_mut().expect("a projector");
            let roots = pt.resolve_source_roots(f, &tri);
            let m = pt.measure_overlap(&roots, &tri, &tri);
            o.i(roots.len() as i64);
            for r in &roots {
                o.i(*r as i64);
            }
            o.i(m.dominant_state.0 as i64);
            o.d(m.surface_area);
            o.d(m.error_area);
            o.i(pt.rebound_count() as i64);
            o.i(pt.fallback_count() as i64);
        }
        "region_source" => {
            let band = t.d();
            let rs = PaintRegionSource::new(st.src, &st.mmu.data, band, None);
            o.i(rs.empty() as i64);
            o.i(rs.leaf_count() as i64);
            o.f(rs.finest_leaf_edge());
            o.i(rs.states().len() as i64);
            for s in rs.states() {
                o.i(s.state.0 as i64);
                o.d(s.area);
                o.bx(&s.bbox);
            }
            st.region = Some(rs);
        }
        "coverage" => {
            let _band = t.d();
            let state = t.i();
            let tri = t.tri();
            let c = st.region.as_ref().unwrap().coverage(EnforcerBlockerType(state as i8), &tri);
            o.i(match c {
                RegionCoverage::None => 0,
                RegionCoverage::Partial => 1,
                RegionCoverage::Full => 2,
            });
        }
        "reproject_point" => {
            let dst_to_src = t.trafo();
            let maxd = t.d();
            let mut calls = Calls::new(t.i());
            let mut out = FacetsAnnotation::default();
            let ok = {
                let calls = std::cell::RefCell::new(&mut calls);
                let mut progress = |p: i32, _m: Option<&str>| {
                    let mut c = calls.borrow_mut();
                    c.progress += 1;
                    c.percent_sum += p as i64;
                    c.last = p as i64;
                };
                let mut cancel = || {
                    let mut c = calls.borrow_mut();
                    c.cancel += 1;
                    c.cancel >= c.cancel_at
                };
                let cancel_at = calls.borrow().cancel_at;
                let cancel_cb: Option<&mut PaintReprojectCancelCallback> = if cancel_at > 0 { Some(&mut cancel) } else { None };
                reproject_one_annotation(&st.mmu, &mut out, st.src, &st.mesh, &dst_to_src, None, ReprojectMode::PointNearest, "mmu", Some(&mut progress), cancel_cb, maxd, None, 1e-6)
            };
            o.i(ok as i64);
            o.data(&out.data);
            calls.put(&mut o);
        }
        "reproject_overlap" => {
            st.dst_to_src = t.trafo();
            let n = t.u();
            st.map = (0..n).map(|_| t.i()).collect();
            let mut out = FacetsAnnotation::default();
            let ok = reproject_one_annotation(&st.mmu, &mut out, st.src, &st.mesh, &st.dst_to_src, Some(&st.map), ReprojectMode::OverlapCoplanar, "mmu", None, None, -1.0, None, REPROJECT_ABSOLUTE_ERROR_EPSILON);
            o.i(ok as i64);
            o.data(&out.data);
        }
        "reproject_paint" => {
            let mut calls = Calls::new(t.i());
            let src = ModelVolume { mesh: st.src.clone(), supported_facets: st.supported.clone(), seam_facets: st.seam.clone(), mmu_segmentation_facets: st.mmu.clone(), fuzzy_skin_facets: FacetsAnnotation::default() };
            let mut dst = ModelVolume { mesh: st.mesh.clone(), ..Default::default() };
            let ok = {
                let calls = std::cell::RefCell::new(&mut calls);
                let mut progress = |p: i32, _m: Option<&str>| {
                    let mut c = calls.borrow_mut();
                    c.progress += 1;
                    c.percent_sum += p as i64;
                    c.last = p as i64;
                };
                let mut cancel = || {
                    let mut c = calls.borrow_mut();
                    c.cancel += 1;
                    c.cancel >= c.cancel_at
                };
                let cancel_at = calls.borrow().cancel_at;
                let cancel_cb: Option<&mut PaintReprojectCancelCallback> = if cancel_at > 0 { Some(&mut cancel) } else { None };
                reproject_paint(&src, &mut dst, &st.map, &st.dst_to_src, Some(&mut progress), cancel_cb)
            };
            o.i(ok as i64);
            o.data(&dst.supported_facets.data);
            o.data(&dst.seam_facets.data);
            o.data(&dst.mmu_segmentation_facets.data);
            o.data(&dst.fuzzy_skin_facets.data);
            calls.put(&mut o);
        }
        "reproject_paint_geometric" => {
            let dst_to_src = t.trafo();
            let wkind = t.i();
            let world = if wkind != 0 { Some(t.trafo()) } else { None };
            let mut calls = Calls::new(t.i());
            let (mut a, mut b, mut c, mut d) = (FacetsAnnotation::default(), FacetsAnnotation::default(), FacetsAnnotation::default(), FacetsAnnotation::default());
            let fuzzy = FacetsAnnotation::default();
            let ok = {
                let calls = std::cell::RefCell::new(&mut calls);
                let mut progress = |p: i32, _m: Option<&str>| {
                    let mut c = calls.borrow_mut();
                    c.progress += 1;
                    c.percent_sum += p as i64;
                    c.last = p as i64;
                };
                let mut cancel = || {
                    let mut c = calls.borrow_mut();
                    c.cancel += 1;
                    c.cancel >= c.cancel_at
                };
                let cancel_at = calls.borrow().cancel_at;
                let cancel_cb: Option<&mut PaintReprojectCancelCallback> = if cancel_at > 0 { Some(&mut cancel) } else { None };
                reproject_paint_geometric(st.src, &st.supported, &st.seam, &st.mmu, &fuzzy, &st.mesh, &dst_to_src, &mut a, &mut b, &mut c, &mut d, Some(&mut progress), cancel_cb, world.as_ref())
            };
            o.i(ok as i64);
            o.data(&a.data);
            o.data(&b.data);
            o.data(&c.data);
            o.data(&d.data);
            calls.put(&mut o);
        }
        "region_edge_limit" => {
            let wkind = t.i();
            let world = if wkind != 0 { Some(t.trafo()) } else { None };
            let el = RegionEdgeLimit::new(&st.mesh, world.as_ref());
            o.f(el.value());
            o.i(el.trafo().is_some() as i64);
            if let Some(tr) = el.trafo() {
                o.trafof(tr);
            }
        }
        other => panic!("{at}: unknown tag {other}"),
    }
    t.end();
    check(at, &o, want);
}

fn run_reproject(text: &str) -> BTreeMap<String, usize> {
    let mut counts = BTreeMap::new();
    let mut st = ReprojectState {
        mesh: indexed_triangle_set::default(),
        src: Box::leak(Box::new(indexed_triangle_set::default())),
        mmu: FacetsAnnotation::default(),
        supported: FacetsAnnotation::default(),
        seam: FacetsAnnotation::default(),
        projector: None,
        region: None,
        map: Vec::new(),
        dst_to_src: Transform3d::identity(),
    };
    for (lineno, line) in text.lines().enumerate() {
        let (args, want) = match line.find(" ->") {
            Some(k) => (&line[..k], &line[k + 3..]),
            None => (line, ""),
        };
        let mut t = Tok(args.split_whitespace());
        let tag = t.s();
        *counts.entry(tag.to_string()).or_insert(0) += 1;
        let at = format!("line {} ({})", lineno + 1, tag);
        run_reproject_line(&mut st, tag, &mut t, want, &at);
    }
    counts
}

#[test]
fn float_aabb_tree_matches_the_pin() {
    let counts = run_reproject(&read_vectors("paint_reproject/vectors_aabb"));
    expect(&counts, &[("mesh", 40), ("aabb_build", 40), ("aabb_distance", 1600), ("aabb_traverse", 400)]);
}

#[test]
fn samplers_and_reprojection_match_the_pin() {
    let counts = run_reproject(&read_vectors("paint_reproject/vectors_reproject"));
    eprintln!("{counts:?}");
    assert_eq!(counts["reproject_paint_geometric"], 24);
    assert_eq!(counts["reproject_paint"], 24);
    assert_eq!(counts["reproject_point"], 12);
}

struct Operand {
    mesh: indexed_triangle_set,
    supported: FacetsAnnotation,
    seam: FacetsAnnotation,
    mmu: FacetsAnnotation,
    fuzzy: FacetsAnnotation,
    has_seam: bool,
    base_filament: i32,
    trafo: Transform3d,
}

fn run_volumes(text: &str) -> BTreeMap<String, usize> {
    let mut counts = BTreeMap::new();
    let mut mesh = indexed_triangle_set::default();
    let mut operands: Vec<Operand> = Vec::new();
    let mut case_ops: Vec<Operand> = Vec::new();
    let mut dst = indexed_triangle_set::default();
    for (lineno, line) in text.lines().enumerate() {
        let (args, want) = match line.find(" ->") {
            Some(k) => (&line[..k], &line[k + 3..]),
            None => (line, ""),
        };
        let mut t = Tok(args.split_whitespace());
        let tag = t.s();
        *counts.entry(tag.to_string()).or_insert(0) += 1;
        let at = format!("line {} ({})", lineno + 1, tag);
        let mut o = Out::default();
        match tag {
            "mesh" => {
                mesh = t.mesh();
                t.end();
                continue;
            }
            "operand" => {
                let (supported, seam, mmu, fuzzy) = (t.data(), t.data(), t.data(), t.data());
                let has_seam = t.i() != 0;
                let base_filament = t.i();
                let trafo = t.trafo();
                t.end();
                operands.push(Operand {
                    mesh: mesh.clone(),
                    supported: FacetsAnnotation { data: supported },
                    seam: FacetsAnnotation { data: seam },
                    mmu: FacetsAnnotation { data: mmu },
                    fuzzy: FacetsAnnotation { data: fuzzy },
                    has_seam,
                    base_filament,
                    trafo,
                });
                continue;
            }
            "reproject_paint_from_volumes" => {
                let concatenated = t.i() != 0;
                t.end();
                dst = mesh.clone();
                case_ops = std::mem::take(&mut operands);
                let sources: Vec<PaintSourceVolume> = case_ops
                    .iter()
                    .map(|op| PaintSourceVolume {
                        mesh: Some(&op.mesh),
                        supported: Some(&op.supported),
                        seam: if op.has_seam { Some(&op.seam) } else { None },
                        mmu: Some(&op.mmu),
                        fuzzy: Some(&op.fuzzy),
                        source_to_destination: op.trafo,
                        base_filament: op.base_filament,
                    })
                    .collect();
                let (mut a, mut b, mut c, mut d) = (FacetsAnnotation::default(), FacetsAnnotation::default(), FacetsAnnotation::default(), FacetsAnnotation::default());
                let ok = reproject_paint_from_volumes(&dst, &sources, &mut a, &mut b, &mut c, &mut d, None, None, None, concatenated);
                o.i(ok as i64);
                o.data(&a.data);
                o.data(&b.data);
                o.data(&c.data);
                o.data(&d.data);
            }
            "measure_operand" => {
                let f = t.i();
                t.end();
                let first = &case_ops[0];
                let mut m = first.mesh.clone();
                its_transform_fixed(&mut m, &first.trafo, true);
                let tree = build_aabb_tree_over_indexed_triangle_set_f(&m.vertices, &m.indices, 0.0);
                let bbox = mesh_bounding_box(&m);
                let e = bbox.1 - bbox.0;
                let extent = ((e.x * e.x + e.y * e.y) + e.z * e.z).sqrt();
                let op = BooleanOperand { mesh: m, ann: PaintKeepPrepared::default(), base_filament: 0, tree, bbox, extent };
                let samples = face_samples(&dst, f);
                let hit = measure_operand(&op, 0, &samples);
                o.d(operand_band(&op));
                o.d(operand_reach(&op));
                o.i(hit.op as i64);
                o.f(hit.distance);
                o.i(hit.covers as i64);
            }
            other => panic!("{at}: unknown tag {other}"),
        }
        check(&at, &o, want);
    }
    counts
}

#[test]
fn boolean_and_merge_reprojection_match_the_pin() {
    let counts = run_volumes(&read_vectors("paint_reproject/vectors_volumes"));
    eprintln!("{counts:?}");
    assert_eq!(counts["reproject_paint_from_volumes"], 30);
    assert_eq!(counts["measure_operand"], 240);
}
