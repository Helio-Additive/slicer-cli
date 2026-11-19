//! `cut_surface.rs` against CutSurface.cpp at the pin.
//!
//! `tests/data/cut_surface/vectors.txt.gz` is the output of `gen.cpp` beside it, which compiles
//! CutSurface.cpp into itself, links the engine's libslic3r and runs `Slic3r::cut_surface` over
//! thirteen cases stage by stage (the header comment there says how, and describes the records).
//! The generator requires the staged run to be equal to the pin's `cut_surface` for every case.
//!
//! The crate has no corefinement: `CGAL::Polygon_mesh_processing::corefine` and `clip` are taken
//! through the trait `PolygonMeshProcessing`. `Replay` below implements it from the vectors: it
//! makes the calls to the visitor CGAL made, in CGAL's order, and sets the mesh and the
//! constrained edges as CGAL left them. Everything before and after is computed by the port and
//! has to be equal to the pin's, record by record: the skip flags, the three kinds of `to_cgal`
//! mesh with every halfedge, the two property maps of the shape, the source of every vertex after
//! the visitor ran, the face types after each of the three passes, the areas of interest, the
//! reduction maps, the patches (mesh, source vertices, loops, bounding box), the distances, the
//! chosen distances, the mask of the patches, the surface cut and `cut2model` of it. Each case is
//! run a second time through `cut_surface` itself.
//!
//! Integers are decimal, doubles and floats the hex of their bits.

use slicer::cut_surface::surface_mesh::Raw_surface_mesh;
use slicer::cut_surface::*;
use slicer::emboss::{OrthoProject, OrthoProject3d};
use slicer::ex_polygons_index::ExPolygonsIndices;
use slicer::geometry::{ExPolygon, ExPolygons, Point, Polygon, Polygons};
use slicer::normal_utils::{indexed_triangle_set, StlTriangleVertexIndices, Vec3f};
use std::cell::{Cell, RefCell};
use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::fmt::Write as _;
use std::io::Read;

type Vec3d = slicer::geometry::Vec3d;
type Transform3d = nalgebra::Matrix4<f64>;

fn read_vectors() -> String {
    let path = format!("{}/tests/data/cut_surface/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    text
}

// ---------------------------------------------------------------------------------------------
// Reading
// ---------------------------------------------------------------------------------------------

struct Tok<'a>(std::str::SplitWhitespace<'a>);
impl<'a> Tok<'a> {
    fn new(s: &'a str) -> Self {
        Tok(s.split_whitespace())
    }
    fn s(&mut self) -> &'a str {
        self.0.next().expect("token")
    }
    fn i(&mut self) -> i64 {
        self.s().parse().expect("integer")
    }
    fn u(&mut self) -> usize {
        self.s().parse().expect("unsigned")
    }
    fn u32(&mut self) -> u32 {
        self.s().parse().expect("u32")
    }
    fn b(&mut self) -> bool {
        self.u() != 0
    }
    fn d(&mut self) -> f64 {
        f64::from_bits(u64::from_str_radix(self.s(), 16).expect("double bits"))
    }
    fn f(&mut self) -> f32 {
        f32::from_bits(u32::from_str_radix(self.s(), 16).expect("float bits"))
    }
    fn polygon(&mut self) -> Polygon {
        let n = self.u();
        Polygon::from_points((0..n).map(|_| Point::new(self.i(), self.i())).collect())
    }
    fn polygons(&mut self) -> Polygons {
        let n = self.u();
        (0..n).map(|_| self.polygon()).collect()
    }
    fn expolygons(&mut self) -> ExPolygons {
        let n = self.u();
        (0..n)
            .map(|_| {
                let contour = self.polygon();
                let holes = self.polygons();
                let mut e = ExPolygon::new(contour);
                e.holes = holes;
                e
            })
            .collect()
    }
    fn its(&mut self) -> indexed_triangle_set {
        let mut its = indexed_triangle_set::default();
        let nv = self.u();
        for _ in 0..nv {
            its.vertices.push(Vec3f::new(self.f(), self.f(), self.f()));
        }
        let ni = self.u();
        for _ in 0..ni {
            its.indices.push(StlTriangleVertexIndices::new(self.i() as i32, self.i() as i32, self.i() as i32));
        }
        its
    }
    fn bits(&mut self) -> Vec<bool> {
        let n = self.u();
        let s = self.s();
        if n == 0 {
            assert_eq!(s, "-");
            return Vec::new();
        }
        assert_eq!(s.len(), n);
        s.bytes().map(|c| c == b'1').collect()
    }
    fn mesh(&mut self) -> Raw_surface_mesh {
        let (nv, ne, nf) = (self.u(), self.u(), self.u());
        let mut raw = Raw_surface_mesh::default();
        for _ in 0..nv {
            raw.points.push([self.d(), self.d(), self.d()]);
            raw.vconn.push(self.u32());
            raw.vremoved.push(self.b());
        }
        for _ in 0..2 * ne {
            raw.hconn.push([self.u32(), self.u32(), self.u32(), self.u32()]);
        }
        for _ in 0..ne {
            raw.eremoved.push(self.b());
        }
        for _ in 0..nf {
            raw.fconn.push(self.u32());
            raw.fremoved.push(self.b());
        }
        raw.freelists = [self.u32(), self.u32(), self.u32()];
        raw.garbage = self.b();
        raw
    }
    fn u32s(&mut self) -> Vec<u32> {
        let n = self.u();
        (0..n).map(|_| self.u32()).collect()
    }
}

// ---------------------------------------------------------------------------------------------
// Writing, the way the generator does
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
    fn u<T: std::fmt::Display>(&mut self, v: T) {
        write!(self.0, " {}", v).unwrap();
    }
    fn bits(&mut self, b: &[bool]) {
        write!(self.0, " {} ", b.len()).unwrap();
        if b.is_empty() {
            self.0.push('-');
        }
        for x in b {
            self.0.push(if *x { '1' } else { '0' });
        }
    }
    fn its(&mut self, its: &indexed_triangle_set) {
        self.u(its.vertices.len());
        for v in &its.vertices {
            self.f(v.x);
            self.f(v.y);
            self.f(v.z);
        }
        self.u(its.indices.len());
        for t in &its.indices {
            self.u(t[0]);
            self.u(t[1]);
            self.u(t[2]);
        }
    }
    fn mesh(&mut self, m: &CutMesh) {
        let raw = m.raw();
        self.u(raw.vconn.len());
        self.u(raw.eremoved.len());
        self.u(raw.fconn.len());
        for i in 0..raw.vconn.len() {
            for c in raw.points[i] {
                self.d(c);
            }
            self.u(raw.vconn[i]);
            self.u(raw.vremoved[i] as u8);
        }
        for h in &raw.hconn {
            for c in h {
                self.u(*c);
            }
        }
        for r in &raw.eremoved {
            self.u(*r as u8);
        }
        for i in 0..raw.fconn.len() {
            self.u(raw.fconn[i]);
            self.u(raw.fremoved[i] as u8);
        }
        for l in raw.freelists {
            self.u(l);
        }
        self.u(raw.garbage as u8);
    }
    fn ies(&mut self, v: &[IntersectingElement]) {
        self.u(v.len());
        for ie in v {
            self.u(ie.shape_point_index);
            self.u(ie.attr);
        }
    }
    fn vsm(&mut self, m: &CutMesh) {
        let map: VertexShapeMap = m.property_map(vert_shape_map_name).0;
        let v = map.to_vec();
        self.u(v.len());
        for ie in v {
            match ie {
                None => self.0.push_str(" -"),
                Some(ie) => write!(self.0, " {}:{}", ie.shape_point_index, ie.attr).unwrap(),
            }
        }
    }
    fn face_types(&mut self, map: &FaceTypeMap) {
        let v = map.to_vec();
        write!(self.0, " {} ", v.len()).unwrap();
        if v.is_empty() {
            self.0.push('-');
        }
        for t in v {
            self.0.push((b'0' + t as u8) as char);
        }
    }
    fn vi_map(&mut self, map: &CvtVI2VI) {
        let v = map.to_vec();
        self.u(v.len());
        for x in v {
            self.u(x.0);
        }
    }
    fn aois(&mut self, aois: &CutAOIs) {
        self.u(aois.len());
        for aoi in aois {
            self.u(aoi.0.len());
            for f in &aoi.0 {
                self.u(f.0);
            }
            self.u(aoi.1.len());
            for h in &aoi.1 {
                self.u(h.0);
            }
        }
    }
    fn patch(&mut self, patch: &SurfacePatch) {
        self.u(patch.model_id);
        self.u(patch.aoi_id);
        self.u(patch.shape_id);
        self.u(patch.is_whole_aoi as u8);
        for c in [patch.bb.min.x, patch.bb.min.y, patch.bb.min.z, patch.bb.max.x, patch.bb.max.y, patch.bb.max.z] {
            self.d(c);
        }
        self.u(patch.bb.defined as u8);
        self.u(patch.loops.len());
        for l in &patch.loops {
            self.u(l.len());
            for v in l {
                self.u(v.0);
            }
        }
        self.mesh(&patch.mesh);
        self.vi_map(&patch.mesh.property_map(patch_source_name).0);
    }
    fn distance(&mut self, d: &ProjectionDistance) {
        self.u(d.model_index);
        self.u(d.aoi_index);
        self.u(d.patch_index);
        self.f(d.distance);
    }
    fn cut(&mut self, cut: &SurfaceCut) {
        self.its(&cut.its);
        self.u(cut.contours.len());
        for c in &cut.contours {
            self.u(c.len());
            for i in c {
                self.u(*i);
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// What CGAL did, replayed
// ---------------------------------------------------------------------------------------------

#[derive(Clone)]
enum Event {
    /// `intersection_point_detected(i_id, sdim, h_f, h_e, tm_f, tm_e, is_target_coplanar,
    /// is_source_coplanar)`; `e_is_shape` says that `tm_e` is the shape (and `tm_f` the object)
    I { i_id: usize, sdim: i32, h_f: u32, h_e: u32, e_is_shape: bool, is_target_coplanar: bool, is_source_coplanar: bool },
    /// `new_vertex_added(i_id, v, object)`
    N { i_id: usize, v: u32 },
}

#[derive(Clone)]
struct CorefineRecord {
    events: Vec<Event>,
    post: Raw_surface_mesh,
    ecm: Vec<bool>,
}

#[derive(Clone)]
struct ClipRecord {
    exception: bool,
    suc: bool,
    exist_intersection: bool,
    post: Raw_surface_mesh,
    patch_source: Vec<u32>,
}

struct Replay {
    corefines: RefCell<VecDeque<CorefineRecord>>,
    clips: RefCell<VecDeque<ClipRecord>>,
}

impl PolygonMeshProcessing for Replay {
    fn corefine(&self, tm1: &mut CutMesh, tm2: &mut CutMesh, np1: NamedParameters<'_>, np2: NamedParameters<'_>) {
        let rec = self.corefines.borrow_mut().pop_front().expect("a corefine the pin did not make");
        // what CutSurface.cpp passes
        assert_eq!(np1.throw_on_self_intersection, Some(false));
        assert_eq!(np2.do_not_modify, Some(true));
        let visitor = np1.visitor.expect("visitor");
        // CGAL reports every intersection point before it changes the object (the generator
        // stops otherwise), then refines the object and reports the new vertices.
        let mut refined = false;
        for e in &rec.events {
            match *e {
                Event::I { i_id, sdim, h_f, h_e, e_is_shape, is_target_coplanar, is_source_coplanar } => {
                    assert!(!refined);
                    let (tm_f, tm_e): (&CutMesh, &CutMesh) = if e_is_shape { (&*tm1, &*tm2) } else { (&*tm2, &*tm1) };
                    visitor.intersection_point_detected(i_id, sdim, HI::new(h_f), HI::new(h_e), tm_f, tm_e, is_target_coplanar, is_source_coplanar);
                }
                Event::N { i_id, v } => {
                    if !refined {
                        tm1.set_raw(&rec.post);
                        refined = true;
                    }
                    visitor.new_vertex_added(i_id, VI::new(v), &*tm1);
                }
            }
        }
        if !refined {
            tm1.set_raw(&rec.post);
        }
        let ecm = np1.edge_is_constrained_map.expect("edge_is_constrained_map");
        assert_eq!(ecm.len(), rec.ecm.len());
        for (i, b) in rec.ecm.iter().enumerate() {
            ecm.set_at(i, *b);
        }
    }

    fn clip(&self, tm: &mut CutMesh, clipper: &mut CutMesh, np_tm: NamedParameters<'_>, np_c: NamedParameters<'_>) -> Result<bool, String> {
        let rec = self.clips.borrow_mut().pop_front().expect("a clip the pin did not make");
        assert_eq!(np_tm.throw_on_self_intersection, Some(false));
        assert_eq!(np_c.throw_on_self_intersection, Some(false));
        if rec.exception {
            return Err("replayed exception".to_string());
        }
        if rec.exist_intersection {
            // the visitor of clip_cut only notes that CGAL reported an intersection point
            let visitor = np_tm.visitor.expect("visitor");
            visitor.intersection_point_detected(0, 0, HI::null(), HI::null(), &*tm, &*clipper, false, false);
        }
        tm.set_raw(&rec.post);
        // CGAL keeps the values of the properties of the vertices that stay; new ones get the default
        let patch_source: CvtVI2VI = tm.property_map(patch_source_name).0;
        let have: Vec<u32> = patch_source.to_vec().iter().map(|v| v.0).collect();
        assert_eq!(have, rec.patch_source, "v:patch_source after clip");
        Ok(rec.suc)
    }

    fn read_off(&self, _path: &str, _mesh: &mut CutMesh) -> bool {
        panic!("read_OFF is not part of the vectors")
    }
}

// ---------------------------------------------------------------------------------------------
// A case
// ---------------------------------------------------------------------------------------------

/// Tags whose first value is the index of a model or of a patch: it is part of the key.
const INDEXED: [&str; 16] =
    ["SKIP1", "NEG", "SKIP2", "MODEL", "ITS_CUT_AOI", "COREFINE", "POST", "ECM", "VSM", "VALID", "FT1", "FT2", "FT3", "AOIS", "REDUCE", "PATCH"];
/// Records that are inputs of the case or of the replay
const INPUTS: [&str; 5] = ["IN_SHAPES", "IN_MODELS", "IN_PROJ", "COREFINE", "REAL_EQUAL"];

#[derive(Default)]
struct Case<'a> {
    name: &'a str,
    records: BTreeMap<String, &'a str>,
    its: Vec<&'a str>,
    clips: Vec<&'a str>,
}

/// The cases, and the `D_` records after them (tag and values).
fn parse_cases(text: &str) -> (Vec<Case<'_>>, Vec<(&str, &str)>) {
    let mut cases: Vec<Case> = Vec::new();
    let mut direct: Vec<(&str, &str)> = Vec::new();
    for line in text.lines() {
        let (tag, rest) = line.split_once(' ').unwrap_or((line, ""));
        if tag == "CASE" {
            cases.push(Case { name: rest, ..Default::default() });
            continue;
        }
        if tag.starts_with("D_") {
            direct.push((tag, rest));
            continue;
        }
        let case = cases.last_mut().expect("a record before the first CASE");
        match tag {
            "IN_ITS" => case.its.push(rest),
            "CLIP" => case.clips.push(rest),
            _ => {
                let (key, rest) = if INDEXED.contains(&tag) {
                    let (index, rest) = rest.split_once(' ').unwrap_or((rest, ""));
                    (format!("{tag} {index}"), rest)
                } else {
                    (tag.to_string(), rest)
                };
                assert!(case.records.insert(key.clone(), rest).is_none(), "{}: {key} twice", case.name);
            }
        }
    }
    (cases, direct)
}

/// Compares what the port computed with the records of a case and counts.
struct Checker<'a, 'b> {
    case: &'b Case<'a>,
    seen: BTreeSet<String>,
    compared: &'b mut BTreeMap<String, (usize, usize)>,
    failures: &'b mut Vec<String>,
}

impl<'a, 'b> Checker<'a, 'b> {
    fn record(&mut self, key: &str) -> &'a str {
        self.seen.insert(key.to_string());
        self.case.records.get(key).copied().unwrap_or_else(|| panic!("{}: no record {key}", self.case.name))
    }
    fn has(&self, key: &str) -> bool {
        self.case.records.contains_key(key)
    }
    fn check(&mut self, key: &str, out: Out) {
        let want = self.record(key);
        let tag = key.split(' ').next().unwrap().to_string();
        let entry = self.compared.entry(tag).or_insert((0, 0));
        entry.0 += 1;
        // the generator writes a space before every value, the key has taken the first
        if out.0.strip_prefix(' ').unwrap_or(&out.0) != want {
            entry.1 += 1;
            let have = out.0.strip_prefix(' ').unwrap_or(&out.0);
            let at = have.bytes().zip(want.bytes()).position(|(a, b)| a != b).unwrap_or(have.len().min(want.len()));
            let from = at.saturating_sub(40);
            self.failures.push(format!(
                "{} {key}: differs at byte {at} (lengths {} and {})\n  port: ...{}\n  pin:  ...{}",
                self.case.name,
                have.len(),
                want.len(),
                &have[from..(at + 60).min(have.len())],
                &want[from..(at + 60).min(want.len())]
            ));
        }
    }
}

fn events_of(line: &str) -> Vec<Event> {
    let mut t = Tok::new(line);
    let n = t.u();
    let mut events = Vec::with_capacity(n);
    for _ in 0..n {
        match t.s() {
            "I" => events.push(Event::I {
                i_id: t.u(),
                sdim: t.i() as i32,
                h_f: t.u32(),
                h_e: t.u32(),
                e_is_shape: t.b(),
                is_target_coplanar: t.b(),
                is_source_coplanar: t.b(),
            }),
            "N" => events.push(Event::N { i_id: t.u(), v: t.u32() }),
            other => panic!("event {other}"),
        }
    }
    events
}

fn replay_of(case: &Case) -> Replay {
    let mut corefines = VecDeque::new();
    let mut m = 0;
    while let Some(line) = case.records.get(&format!("COREFINE {m}")) {
        let events = events_of(line);
        let post = Tok::new(case.records[&format!("POST {m}")]).mesh();
        let ecm = Tok::new(case.records[&format!("ECM {m}")]).bits();
        corefines.push_back(CorefineRecord { events, post, ecm });
        m += 1;
    }
    let mut clips = VecDeque::new();
    for line in &case.clips {
        let mut t = Tok::new(line);
        let (exception, suc, exist_intersection) = (t.b(), t.b(), t.b());
        let (post, patch_source) = if exception { (Raw_surface_mesh::default(), Vec::new()) } else { (t.mesh(), t.u32s()) };
        clips.push_back(ClipRecord { exception, suc, exist_intersection, post, patch_source });
    }
    Replay { corefines: RefCell::new(corefines), clips: RefCell::new(clips) }
}

fn run_case(case: &Case, compared: &mut BTreeMap<String, (usize, usize)>, failures: &mut Vec<String>) {
    let mut c = Checker { case, seen: BTreeSet::new(), compared, failures };

    // the inputs
    let shapes: ExPolygons = Tok::new(c.record("IN_SHAPES")).expolygons();
    let n_models = Tok::new(c.record("IN_MODELS")).u();
    assert_eq!(n_models, case.its.len());
    let models: Vec<indexed_triangle_set> = case.its.iter().map(|l| Tok::new(l).its()).collect();
    let (tr, dir, projection_ratio) = {
        let mut t = Tok::new(c.record("IN_PROJ"));
        let mut m = Transform3d::zeros();
        for r in 0..4 {
            for col in 0..4 {
                m[(r, col)] = t.d();
            }
        }
        (m, Vec3d::new(t.d(), t.d(), t.d()), t.f())
    };
    let projection = OrthoProject::new(tr, dir);
    let replay = replay_of(case);

    // ---- Slic3r::cut_surface (CutSurface.cpp:525-617), stage by stage ----
    let shapes_bb = slicer::geometry::get_extents(&shapes);
    {
        let mut o = Out::default();
        o.u(shapes_bb.min.x);
        o.u(shapes_bb.min.y);
        o.u(shapes_bb.max.x);
        o.u(shapes_bb.max.y);
        c.check("BB", o);
    }

    let max_angle: f32 = 89.9;
    let mut cgal_models = CutMeshes::new();
    let mut cgal_neg_models = CutMeshes::new();
    for (m, its) in models.iter().enumerate() {
        let mut skip_indicies = vec![false; its.indices.len()];
        set_skip_for_out_of_aoi(&mut skip_indicies, its, &projection, &shapes_bb);
        let mut o = Out::default();
        o.bits(&skip_indicies);
        c.check(&format!("SKIP1 {m}"), o);

        cgal_neg_models.push(to_cgal(its, &skip_indicies, true));
        let mut o = Out::default();
        o.mesh(cgal_neg_models.last().unwrap());
        c.check(&format!("NEG {m}"), o);

        set_skip_by_angle(&mut skip_indicies, its, &projection, max_angle as f64);
        let mut o = Out::default();
        o.bits(&skip_indicies);
        c.check(&format!("SKIP2 {m}"), o);

        cgal_models.push(to_cgal(its, &skip_indicies, false));
        let mut o = Out::default();
        o.mesh(cgal_models.last().unwrap());
        c.check(&format!("MODEL {m}"), o);

        let mut o = Out::default();
        o.its(&its_cut_ao_i(its, &shapes_bb, &projection));
        c.check(&format!("ITS_CUT_AOI {m}"), o);
    }

    let mut cgal_shape: CutMesh = of_shapes::to_cgal(&shapes, &projection);
    {
        let mut o = Out::default();
        o.mesh(&cgal_shape);
        c.check("SHAPE", o);
        let esm: EdgeShapeMap = cgal_shape.property_map(edge_shape_map_name).0;
        let fsm: FaceShapeMap = cgal_shape.property_map(face_shape_map_name).0;
        let mut o = Out::default();
        o.ies(&esm.to_vec());
        c.check("ESM", o);
        let mut o = Out::default();
        o.ies(&fsm.to_vec());
        c.check("FSM", o);
    }

    let s2i = ExPolygonsIndices::new(&shapes);
    let mut model_cuts = VCutAOIs::new();
    for m in 0..cgal_models.len() {
        // priv::cut_from_model (CutSurface.cpp:1400-1443), stage by stage
        let cgal_model = &mut cgal_models[m];
        let vert_shape_map: VertexShapeMap = cgal_model.add_property_map(vert_shape_map_name, None).0;
        let is_valid = Cell::new(true);
        let edge_shape_map: EdgeShapeMap = cgal_shape.property_map(edge_shape_map_name).0;
        let face_shape_map: FaceShapeMap = cgal_shape.property_map(face_shape_map_name).0;
        let mut visitor = Visitor::new(cgal_model, &cgal_shape, edge_shape_map, face_shape_map, vert_shape_map.clone(), &is_valid);
        let ecm: EdgeBoolMap = cgal_model.add_property_map(is_constrained_edge_name, false).0;
        {
            let p = NamedParameters::default_values().visitor(&mut visitor).edge_is_constrained_map(ecm.clone()).throw_on_self_intersection(false);
            let q = NamedParameters::default_values().do_not_modify(true);
            replay.corefine(cgal_model, &mut cgal_shape, p, q);
        }
        // the replay set these two; they come back out as they went in
        let mut o = Out::default();
        o.mesh(cgal_model);
        c.check(&format!("POST {m}"), o);
        let mut o = Out::default();
        o.bits(&ecm.to_vec());
        c.check(&format!("ECM {m}"), o);
        // what the visitor of the port wrote
        let mut o = Out::default();
        o.vsm(cgal_model);
        c.check(&format!("VSM {m}"), o);
        let mut o = Out::default();
        o.u(is_valid.get() as u8);
        c.check(&format!("VALID {m}"), o);

        let mut aois = CutAOIs::new();
        if is_valid.get() {
            let face_type_map: FaceTypeMap = cgal_model.add_property_map(face_type_map_name, FaceType::not_constrained).0;
            set_face_type(&face_type_map, cgal_model, &vert_shape_map, &ecm, &cgal_shape, &s2i);
            let mut o = Out::default();
            o.face_types(&face_type_map);
            c.check(&format!("FT1 {m}"), o);

            flood_fill_inner(cgal_model, &face_type_map);
            let mut o = Out::default();
            o.face_types(&face_type_map);
            c.check(&format!("FT2 {m}"), o);

            aois = create_cut_area_of_interests(cgal_model, &shapes, &face_type_map);
            let mut o = Out::default();
            o.face_types(&face_type_map);
            c.check(&format!("FT3 {m}"), o);
        }
        let mut o = Out::default();
        o.aois(&aois);
        c.check(&format!("AOIS {m}"), o);
        model_cuts.push(aois);
    }

    // priv::create_reduce_map, which priv::diff_models runs for every model on a property it removes
    for m in 0..cgal_models.len() {
        let mut reduction_map: ReductionMap = cgal_models[m].add_property_map(vertex_reduction_map_name, VI::null()).0;
        create_reduce_map(&reduction_map, &cgal_models[m]);
        let mut o = Out::default();
        o.vi_map(&reduction_map);
        c.check(&format!("REDUCE {m}"), o);
        cgal_models[m].remove_property_map(&mut reduction_map);
    }

    let mut patches: SurfacePatches = diff_models(&mut model_cuts, &mut cgal_models, &mut cgal_neg_models, &projection, &replay);
    assert!(replay.clips.borrow().is_empty(), "{}: the pin clipped more often", case.name);
    {
        let mut o = Out::default();
        o.u(patches.len());
        c.check("PATCHES", o);
    }
    for (i, patch) in patches.iter().enumerate() {
        let key = format!("PATCH {i}");
        if !c.has(&key) {
            break; // the number of patches is wrong, PATCHES says so
        }
        let mut o = Out::default();
        o.patch(patch);
        c.check(&key, o);
    }

    let mut result = SurfaceCut::default();
    if !patches.is_empty() {
        for patch in patches.iter_mut() {
            patch.shape_id = s2i.cvt_index(patch.shape_id as u32).expolygons_index as usize;
        }
        let mut o = Out::default();
        o.u(patches.len());
        for patch in &patches {
            o.u(patch.shape_id);
        }
        c.check("SHAPE_IDS", o);

        let shapes_points: u32 = s2i.get_count();
        let distances: VDistances = calc_distances(&patches, &cgal_models, &cgal_shape, shapes_points as usize, projection_ratio);
        let mut o = Out::default();
        o.u(distances.len());
        for ds in &distances {
            o.u(ds.len());
            for d in ds {
                o.distance(d);
            }
        }
        c.check("DIST", o);

        let start: Point = shapes_bb.center();
        let best_projection: ProjectionDistances = choose_best_distance(&distances, &shapes, &start, &s2i, &patches);
        let mut o = Out::default();
        o.u(best_projection.len());
        for d in &best_projection {
            o.distance(d);
        }
        c.check("BEST", o);

        let use_patch: Vec<bool> = select_patches(&best_projection, &patches, &shapes, &shapes_bb, &s2i, &model_cuts, &cgal_models, &projection);
        let mut o = Out::default();
        o.bits(&use_patch);
        c.check("USE", o);

        result = merge_patches(&mut patches, &use_patch);
    }
    let mut o = Out::default();
    o.cut(&result);
    c.check("CUT", o);

    if !result.empty() {
        let back = OrthoProject3d::new(dir);
        let mut o = Out::default();
        o.its(&cut2model(&result, &back));
        c.check("CUT2MODEL", o);
    }

    // ---- the same case through cut_surface itself ----
    let replay = replay_of(case);
    let whole = cut_surface(&shapes, &models, &projection, projection_ratio, &replay);
    assert!(replay.corefines.borrow().is_empty() && replay.clips.borrow().is_empty(), "{}: cut_surface made fewer calls to CGAL than the pin", case.name);
    let want = case.records["CUT"];
    let mut o = Out::default();
    o.cut(&whole);
    let entry = c.compared.entry("cut_surface".to_string()).or_insert((0, 0));
    entry.0 += 1;
    if o.0.trim_start() != want {
        entry.1 += 1;
        c.failures.push(format!("{}: cut_surface differs from the pin's result", case.name));
    }

    // every record of the case was compared
    for key in case.records.keys() {
        let tag = key.split(' ').next().unwrap();
        assert!(c.seen.contains(key) || INPUTS.contains(&tag), "{}: record {key} was not compared", case.name);
    }
}

#[test]
fn cut_surface_stages_equal_to_the_pin() {
    let text = read_vectors();
    let (cases, _) = parse_cases(&text);
    assert_eq!(cases.len(), 13);
    let mut compared: BTreeMap<String, (usize, usize)> = BTreeMap::new();
    let mut failures: Vec<String> = Vec::new();
    for case in &cases {
        assert_eq!(case.records.get("REAL_EQUAL").copied(), Some("1"), "{}", case.name);
        run_case(case, &mut compared, &mut failures);
    }
    let mut total = 0;
    for (tag, (n, bad)) in &compared {
        println!("{tag:12} {n:3} compared, {bad} differ");
        total += n;
    }
    println!("{} cases, {} records compared, {} differ", cases.len(), total, failures.len());
    for f in failures.iter().take(12) {
        println!("{f}");
    }
    assert!(failures.is_empty(), "{} records differ from the pin", failures.len());
}

/// The functions of CutSurface.cpp that `cut_surface` never calls, against the pin's (the `D_`
/// records): the count of the triangles a ray intersects (`create_trees` and the tree, which the
/// port computes without CGAL's tree), `ModelCut2index::calc_index` and `calc_id`, `calc_size_sq`,
/// `find_close_point`, and `IntersectionSources` as the visitor of a replayed corefinement with
/// `create_face_types` over what it wrote and `bounding_box` of a patch.
#[test]
fn functions_cut_surface_does_not_call_equal_to_the_pin() {
    let text = read_vectors();
    let (_, direct) = parse_cases(&text);
    let mut compared: BTreeMap<&str, (usize, usize)> = BTreeMap::new();
    let mut failures: Vec<String> = Vec::new();
    let mut note = |tag: &'static str, n: usize, bad: usize, what: String| {
        let e = compared.entry(tag).or_insert((0, 0));
        e.0 += n;
        e.1 += bad;
        if bad != 0 {
            failures.push(what);
        }
    };

    let mut i = 0;
    while i < direct.len() {
        let (tag, rest) = direct[i];
        i += 1;
        match tag {
            "D_TREE" => {
                let (input, want) = rest.split_once(" -> ").expect("->");
                let mut t = Tok::new(input);
                let flip = t.b();
                let its = t.its();
                let n = t.u();
                let rays: Vec<[f64; 6]> = (0..n).map(|_| [t.d(), t.d(), t.d(), t.d(), t.d(), t.d()]).collect();
                let want: Vec<usize> = want.split_whitespace().map(|x| x.parse().unwrap()).collect();
                assert_eq!(want.len(), n);
                let models: CutMeshes = vec![to_cgal(&its, &vec![false; its.indices.len()], flip)];
                let trees: Trees = create_trees(&models);
                let mut bad = 0;
                let mut first = String::new();
                for (k, r) in rays.iter().enumerate() {
                    let have = trees[0].number_of_intersected_primitives(&Ray::new([r[0], r[1], r[2]], [r[3], r[4], r[5]]));
                    if have != want[k] {
                        if bad == 0 {
                            first = format!("ray {k}: port {have}, pin {}", want[k]);
                        }
                        bad += 1;
                    }
                }
                note("D_TREE", n, bad, format!("D_TREE ({} triangles): {bad} of {n} rays differ; {first}", its.indices.len()));
            }
            "D_M2I" => {
                let (input, want) = rest.split_once(" -> ").expect("->");
                let mut t = Tok::new(input);
                let n = t.u();
                let sizes: Vec<usize> = (0..n).map(|_| t.u()).collect();
                let cuts: VCutAOIs = sizes.iter().map(|k| vec![(Vec::new(), Vec::new()); *k]).collect();
                let m2i = ModelCut2index::new(&cuts);
                let mut o = Out::default();
                o.u(m2i.get_count());
                for off in m2i.get_offsets() {
                    o.u(*off);
                }
                for (m, k) in sizes.iter().enumerate() {
                    for c in 0..*k {
                        o.u(m2i.calc_index(&ModelCutId { model_index: m as u32, cut_index: c as u32 }));
                    }
                }
                for index in 0..m2i.get_count() {
                    let id = m2i.calc_id(index);
                    o.u(id.model_index);
                    o.u(id.cut_index);
                }
                note("D_M2I", 1, (o.0.trim_start() != want) as usize, format!("D_M2I {input}: port{} pin {want}", o.0));
            }
            "D_SIZE" => {
                let (input, want) = rest.split_once(" -> ").expect("->");
                let mut t = Tok::new(input);
                let p = Point::new(t.i(), t.i());
                let mut o = Out::default();
                o.f(calc_size_sq(&p));
                note("D_SIZE", 1, (o.0.trim_start() != want) as usize, format!("D_SIZE {input}: port{} pin {want}", o.0));
            }
            "D_CLOSE" => {
                let (input, want) = rest.split_once(" -> ").expect("->");
                let mut t = Tok::new(input);
                let shapes = t.expolygons();
                let n = t.u();
                let result: ProjectionDistances = (0..n).map(|_| ProjectionDistance { aoi_index: t.u32(), ..Default::default() }).collect();
                let finished = t.bits();
                let p = Point::new(t.i(), t.i());
                let s2i = ExPolygonsIndices::new(&shapes);
                let cp = find_close_point(&p, &result, &finished, &s2i, &shapes);
                let mut o = Out::default();
                o.u(cp.index);
                o.f(cp.dist_sq);
                note("D_CLOSE", 1, (o.0.trim_start() != want) as usize, format!("D_CLOSE: port{} pin {want}", o.0));
            }
            "D_FT" => {
                let mut t = Tok::new(rest);
                let its1 = t.its();
                let its2 = t.its();
                let mut next = |tag: &str| -> &str {
                    assert_eq!(direct[i].0, tag);
                    i += 1;
                    direct[i - 1].1
                };
                let events = events_of(next("D_FT_COREFINE"));
                let post = Tok::new(next("D_FT_POST")).mesh();
                let ecm_bits = Tok::new(next("D_FT_ECM")).bits();
                let replay = Replay { corefines: RefCell::new(VecDeque::from(vec![CorefineRecord { events, post, ecm: ecm_bits }])), clips: RefCell::new(VecDeque::new()) };

                let mut tm1 = to_cgal(&its1, &vec![false; its1.indices.len()], false);
                let mut tm2 = to_cgal(&its2, &vec![false; its2.indices.len()], false);
                let vmap: VertexSourceMap = tm1.add_property_map(vertex_source_map_name, Source::default()).0;
                let sources = RefCell::new(Sources::new());
                let mut visitor = IntersectionSources { patch: &tm1 as *const CutMesh, model: &tm2 as *const CutMesh, vmap: vmap.clone(), sources: &sources };
                let ecm: EdgeBoolMap = tm1.add_property_map(is_constrained_edge_name, false).0;
                {
                    let p = NamedParameters::default_values().visitor(&mut visitor).edge_is_constrained_map(ecm.clone()).throw_on_self_intersection(false);
                    let q = NamedParameters::default_values().do_not_modify(true);
                    replay.corefine(&mut tm1, &mut tm2, p, q);
                }

                let mut o = Out::default();
                let v = vmap.to_vec();
                o.u(v.len());
                for s in &v {
                    o.u(s.hi.0);
                    o.u(s.sdim);
                }
                let want = next("D_FT_VMAP");
                note("D_FT_VMAP", 1, (o.0.trim_start() != want) as usize, "D_FT_VMAP differs".to_string());

                let mut o = Out::default();
                o.u(sources.borrow().len());
                for s in sources.borrow().iter() {
                    o.u(s.hi.0);
                    o.u(s.sdim);
                }
                let want = next("D_FT_SOURCES");
                note("D_FT_SOURCES", 1, (o.0.trim_start() != want) as usize, "D_FT_SOURCES differs".to_string());

                let map: FaceTypeMap = tm1.add_property_map(face_type_map_name, FaceType::not_constrained).0;
                create_face_types(&map, &tm1, &tm2, &ecm, &vmap);
                let mut o = Out::default();
                o.face_types(&map);
                let want = next("D_FT_TYPES");
                note("D_FT_TYPES", 1, (o.0.trim_start() != want) as usize, format!("D_FT_TYPES: port{} pin {want}", o.0));

                let patch = SurfacePatch { mesh: tm1.clone(), ..Default::default() };
                let bb = of_patch::bounding_box(&patch);
                let mut o = Out::default();
                for c in [bb.min.x, bb.min.y, bb.min.z, bb.max.x, bb.max.y, bb.max.z] {
                    o.d(c);
                }
                o.u(bb.defined as u8);
                let want = next("D_FT_BB");
                note("D_FT_BB", 1, (o.0.trim_start() != want) as usize, format!("D_FT_BB: port{} pin {want}", o.0));
            }
            other => panic!("record {other}"),
        }
    }

    let mut total = 0;
    for (tag, (n, bad)) in &compared {
        println!("{tag:12} {n:4} compared, {bad} differ");
        total += n;
    }
    println!("{} compared, {} records differ", total, failures.len());
    for f in failures.iter().take(12) {
        println!("{f}");
    }
    assert!(total > 2000);
    assert!(failures.is_empty(), "{} records differ from the pin", failures.len());
}
