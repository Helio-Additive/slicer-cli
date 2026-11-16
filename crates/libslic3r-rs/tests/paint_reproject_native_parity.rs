//! `paint_reproject.rs` against Bambu Studio's `PaintReproject.cpp` at the pin, for the functions
//! the port has so far.
//!
//! `tests/data/paint_reproject/vectors_*.txt` are written by `gen.cpp` of that directory, which
//! includes the C++ unit unchanged and links the rest of the engine (how to build it is in its
//! header). Each line is one call with its arguments and its result; the test makes the same call
//! and requires the same result, floating point numbers bit for bit.

use std::collections::BTreeMap;

use slicer::geometry::Point3F;
use slicer::model::FacetsAnnotation;
use slicer::paint_reproject::*;
use slicer::texture_painting::EXTRUDER_MAX;
use slicer::triangle_mesh::{indexed_triangle_set, Triangle, TriangleMesh, Vec3f, Vec3i};
use slicer::triangle_selector::{EnforcerBlockerType, TriangleSelector};

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
