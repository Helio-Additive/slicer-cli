//! `triangle_selector.rs` against Bambu Studio's `TriangleSelector.cpp` at the pin.
//!
//! `tests/data/triangle_selector/vectors_*.txt` are written by `gen.cpp` of that directory, which
//! includes the C++ unit unchanged and links the rest of the engine (how to build it is in its
//! header). Each line is one call with its arguments, `->`, and what the C++ answered; the test
//! makes the same call and requires the same answer, token for token, floating point numbers bit
//! for bit. The callbacks handed to `set_facets_with_subdivision` and `paint_region` are the
//! generator's, repeated here operation by operation.
//!
//! The lines are read with the faithful gates at their defaults (all on).

use std::collections::BTreeMap;
use std::fmt::Write as _;

use slicer::geometry::Point3F;
use slicer::triangle_mesh::{indexed_triangle_set, Triangle as MeshTriangle, TriangleMesh, Vec3f, Vec3i};
use slicer::triangle_selector::*;

/// The vectors are kept gzipped (`gzip -9n` of the generator's output).
fn read_vectors(rel: &str) -> String {
    use std::io::Read as _;
    let path = format!("{}/tests/data/{}.txt.gz", env!("CARGO_MANIFEST_DIR"), rel);
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    text
}

type Data = SerializedData;

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
    fn v3(&mut self) -> Vec3f {
        Vec3f::new(self.f(), self.f(), self.f())
    }
    fn ints(&mut self) -> Vec<i32> {
        let n = self.u();
        (0..n).map(|_| self.i()).collect()
    }
    fn bits(&mut self) -> Vec<bool> {
        let n = self.u();
        let s = self.s();
        let bits: Vec<bool> = if s == "-" { Vec::new() } else { s.chars().map(|c| c == '1').collect() };
        assert_eq!(bits.len(), n);
        bits
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
    fn trafof(&mut self) -> Transform3f {
        let mut t = Transform3f::zeros();
        for r in 0..4 {
            for c in 0..4 {
                t[(r, c)] = self.f();
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
    fn data(&mut self) -> Data {
        let n = self.u();
        let pairs: Vec<(i32, i32)> = (0..n).map(|_| (self.i(), self.i())).collect();
        (pairs, self.bits())
    }
    fn clp(&mut self) -> ClippingPlane {
        ClippingPlane { normal: self.v3(), offset: self.f() }
    }
    fn end(&mut self) {
        assert!(self.0.next().is_none(), "tokens left among the arguments");
    }
}

/// The answer in the generator's format.
#[derive(Default)]
struct Out(String);

impl Out {
    fn d(&mut self, v: f64) {
        write!(self.0, " {:016x}", v.to_bits()).unwrap();
    }
    fn f(&mut self, v: f32) {
        write!(self.0, " {:08x}", v.to_bits()).unwrap();
    }
    fn u(&mut self, v: usize) {
        write!(self.0, " {}", v).unwrap();
    }
    fn i(&mut self, v: i64) {
        write!(self.0, " {}", v).unwrap();
    }
    fn b(&mut self, v: bool) {
        self.i(v as i64);
    }
    fn v3(&mut self, v: &Vec3f) {
        self.f(v.x);
        self.f(v.y);
        self.f(v.z);
    }
    fn mesh(&mut self, m: &indexed_triangle_set) {
        self.u(m.vertices.len());
        for v in &m.vertices {
            self.v3(v);
        }
        self.u(m.indices.len());
        for f in &m.indices {
            write!(self.0, " {} {} {}", f[0], f[1], f[2]).unwrap();
        }
    }
    fn data(&mut self, d: &Data) {
        self.u(d.0.len());
        for p in &d.0 {
            self.i(p.0 as i64);
            self.i(p.1 as i64);
        }
        self.u(d.1.len());
        self.0.push(' ');
        if d.1.is_empty() {
            self.0.push('-');
        }
        for &b in &d.1 {
            self.0.push(if b { '1' } else { '0' });
        }
    }
    /// gen.cpp `pselector`
    fn selector(&mut self, sel: &TriangleSelector) {
        self.data(&sel.serialize());
        let valid = sel.get_triangles().iter().filter(|t| t.valid()).count();
        self.u(sel.get_triangles().len());
        self.u(valid);
        self.u(sel.get_vertices().len());
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

/// `TriangleMesh(its)` of the crate, as `TriangleSelector::new` takes it
fn selector_mesh(its: &indexed_triangle_set) -> TriangleMesh {
    TriangleMesh::from_parts(
        its.vertices.iter().map(|v| Point3F::new(v.x as f64, v.y as f64, v.z as f64)).collect(),
        its.indices.iter().map(|f| MeshTriangle::new(f[0] as u32, f[1] as u32, f[2] as u32)).collect(),
    )
}

// ---------------------------------------------------------------------------------------------
// The generator's callbacks (gen.cpp `tri_area`, `Bands`, `Region`), one operation per statement.
// ---------------------------------------------------------------------------------------------

fn tri_area(t: &[Vec3f; 3]) -> f64 {
    let ax = t[1].x as f64 - t[0].x as f64;
    let ay = t[1].y as f64 - t[0].y as f64;
    let az = t[1].z as f64 - t[0].z as f64;
    let bx = t[2].x as f64 - t[0].x as f64;
    let by = t[2].y as f64 - t[0].y as f64;
    let bz = t[2].z as f64 - t[0].z as f64;
    let cx = ay * bz - az * by;
    let cy = az * bx - ax * bz;
    let cz = ax * by - ay * bx;
    let mut s = cx * cx;
    s += cy * cy;
    s += cz * cz;
    0.5 * s.sqrt()
}

#[derive(Clone, Copy)]
struct Bands {
    band: f64,
    shift: f64,
    nstates: i32,
    state_offset: i32,
    quirk: i32,
}

impl Bands {
    fn read(t: &mut Tok) -> Bands {
        Bands { band: t.d(), shift: t.d(), nstates: t.i(), state_offset: t.i(), quirk: t.i() }
    }
    fn state_at(&self, x: f64) -> i32 {
        let q = (x + self.shift) / self.band;
        let s = (q.floor() as i32) % self.nstates;
        if s < 0 {
            0
        } else {
            s + if s == 0 { 0 } else { self.state_offset }
        }
    }
    fn eval(&self, root: i32, t: &[Vec3f; 3]) -> FacetSubdivisionMeasurement {
        let mut m = FacetSubdivisionMeasurement::default();
        let area = tri_area(t);
        let (s0, s1, s2) = (self.state_at(t[0].x as f64), self.state_at(t[1].x as f64), self.state_at(t[2].x as f64));
        let mut cx = t[0].x as f64;
        cx += t[1].x as f64;
        cx += t[2].x as f64;
        cx /= 3.0;
        let sc = self.state_at(cx);
        m.dominant_state = EnforcerBlockerType(sc as i8);
        m.surface_area = area;
        m.error_area = if s0 == s1 && s1 == s2 && s2 == sc { 0.0 } else { area };
        if root % 7 == 3 {
            m.error_area *= 0.5;
        }
        if self.quirk == 1 && root % 5 == 1 {
            let e = m.error_area * 2.0;
            m.error_area = e + 0.25;
        }
        if self.quirk == 2 && root % 5 == 2 {
            m.surface_area = -area;
        }
        if self.quirk == 3 && root % 5 == 3 {
            m.error_area = -1.0;
        }
        m
    }
}

fn std_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}
fn std_max(a: f64, b: f64) -> f64 {
    if a < b {
        b
    } else {
        a
    }
}

#[derive(Clone, Copy)]
struct Region {
    kind: i32,
    cx: f64,
    cy: f64,
    r2: f64,
    k: f64,
}

impl Region {
    fn read(t: &mut Tok) -> Region {
        Region { kind: t.i(), cx: t.d(), cy: t.d(), r2: t.d(), k: t.d() }
    }
    fn inside(&self, v: &Vec3f) -> bool {
        if self.kind == 1 {
            return (v.x as f64) < self.k;
        }
        let dx = v.x as f64 - self.cx;
        let dy = v.y as f64 - self.cy;
        let d = dx * dx;
        let e = dy * dy;
        d + e <= self.r2
    }
    fn coverage(&self, t: &[Vec3f; 3]) -> RegionCoverage {
        let n = self.inside(&t[0]) as i32 + self.inside(&t[1]) as i32 + self.inside(&t[2]) as i32;
        if n == 3 {
            return RegionCoverage::Full;
        }
        if n > 0 {
            return RegionCoverage::Partial;
        }
        let (x0, x1, x2) = (t[0].x as f64, t[1].x as f64, t[2].x as f64);
        let (y0, y1, y2) = (t[0].y as f64, t[1].y as f64, t[2].y as f64);
        let lo_x = std_min(std_min(x0, x1), x2);
        let hi_x = std_max(std_max(x0, x1), x2);
        let lo_y = std_min(std_min(y0, y1), y2);
        let hi_y = std_max(std_max(y0, y1), y2);
        if self.kind == 1 {
            return if lo_x < self.k { RegionCoverage::Partial } else { RegionCoverage::None };
        }
        let r = self.r2.sqrt();
        let (a, b, c, d) = (self.cx - r, self.cx + r, self.cy - r, self.cy + r);
        if hi_x < a || lo_x > b || hi_y < c || lo_y > d {
            return RegionCoverage::None;
        }
        RegionCoverage::Partial
    }
}

// ---------------------------------------------------------------------------------------------

struct State {
    its: indexed_triangle_set,
    sel: Option<TriangleSelector>,
}

fn new_selector(its: &indexed_triangle_set) -> TriangleSelector {
    TriangleSelector::new(selector_mesh(its), 0.6)
}

/// One line. `args` are the tokens before `->`, `want` the text after it.
fn run_line(st: &mut State, tag: &str, args: &mut Tok, want: &str, at: &str) {
    let mut o = Out::default();
    match tag {
        "mesh" => {
            st.its = args.mesh();
            st.sel = Some(new_selector(&st.its));
            return;
        }
        "set_facets_with_subdivision" => {
            let b = Bands::read(args);
            let faces = args.ints();
            let budget = args.u();
            let rel = args.d();
            let abs = args.d();
            let depth = args.i();
            let cancel_at = args.i();
            args.end();
            let sel = st.sel.as_mut().unwrap();
            let mut calls = 0i64;
            let mut cancels = 0i32;
            let mut eval = |root: i32, t: &[Vec3f; 3]| {
                calls += 1;
                b.eval(root, t)
            };
            let mut cancel = || {
                cancels += 1;
                cancels >= cancel_at
            };
            let cancel_cb: Option<&mut FacetSubdivisionCancelCallback> = if cancel_at > 0 { Some(&mut cancel) } else { None };
            let r = sel.set_facets_with_subdivision(&faces, &mut eval, budget, rel, abs, depth, cancel_cb);
            o.d(r.surface_area);
            o.d(r.error_area);
            o.u(r.nodes_created);
            o.b(r.node_budget_exhausted);
            o.b(r.depth_limit_reached);
            o.b(r.canceled);
            o.i(calls);
            o.i(cancels as i64);
            o.selector(sel);
        }
        "paint_region" => {
            let g = Region::read(args);
            let start = args.ints();
            let state = args.i();
            let edge = args.f();
            let budget = args.u();
            let depth = args.i();
            let cancel_at = args.i();
            let mask: Option<Vec<i8>> = if args.i() != 0 { Some(args.ints().into_iter().map(|m| m as i8).collect()) } else { None };
            let trafo: Option<Transform3f> = if args.i() != 0 { Some(args.trafof()) } else { None };
            let before = args.f();
            args.end();
            let sel = st.sel.as_mut().unwrap();
            sel.set_edge_limit(before);
            let mut calls = 0i64;
            let mut cancels = 0i32;
            let mut cov = |t: &[Vec3f; 3]| {
                calls += 1;
                g.coverage(t)
            };
            let mut cancel = || {
                cancels += 1;
                cancels >= cancel_at
            };
            let cancel_cb: Option<&mut FacetSubdivisionCancelCallback> = if cancel_at > 0 { Some(&mut cancel) } else { None };
            let r = sel.paint_region(&start, Some(&mut cov), EnforcerBlockerType(state as i8), edge, budget, depth, cancel_cb, mask.as_deref(), trafo.as_ref());
            let strict = sel.get_facets_strict(EnforcerBlockerType(state as i8));
            o.u(r.nodes_created);
            o.u(r.facets_touched);
            o.b(r.node_budget_exhausted);
            o.b(r.canceled);
            o.i(calls);
            o.i(cancels as i64);
            o.f(sel.edge_limit_sqr());
            o.selector(sel);
            o.u(strict.vertices.len());
            o.u(strict.indices.len());
        }
        "paint_region_empty" => {
            args.end();
            let sel = st.sel.as_mut().unwrap();
            let r = sel.paint_region(&[0], None, EnforcerBlockerType(1), 0.2, 100, 5, None, None, None);
            o.u(r.nodes_created);
            o.u(r.facets_touched);
            o.b(r.node_budget_exhausted);
            o.b(r.canceled);
            o.selector(sel);
        }
        "state" => {
            args.end();
            o.selector(st.sel.as_ref().unwrap());
        }
        "facets" => {
            let s = EnforcerBlockerType(args.i() as i8);
            args.end();
            let sel = st.sel.as_ref().unwrap();
            let (a, b) = (sel.get_facets(s), sel.get_facets_strict(s));
            o.i(sel.num_facets(s) as i64);
            o.b(sel.has_facets(s));
            o.b(TriangleSelector::has_facets_data(&sel.serialize(), s));
            o.u(a.vertices.len());
            o.u(a.indices.len());
            o.u(b.vertices.len());
            o.u(b.indices.len());
        }
        "facets_full" => {
            let s = EnforcerBlockerType(args.i() as i8);
            args.end();
            o.mesh(&st.sel.as_ref().unwrap().get_facets_strict(s));
        }
        "facets_per_type" => {
            args.end();
            let mut per_type = Vec::new();
            st.sel.as_ref().unwrap().get_facets_per_type(&mut per_type);
            o.u(per_type.len());
            let mut first = 0usize;
            for (s, its) in per_type.iter().enumerate() {
                if !its.indices.is_empty() {
                    o.u(s);
                    o.u(its.vertices.len());
                    o.u(its.indices.len());
                    if first == 0 && s > 0 {
                        first = s;
                    }
                }
            }
            o.u(first);
            if first > 0 {
                o.mesh(&per_type[first]);
            }
        }
        "edges" => {
            let t = args.i();
            args.end();
            let sel = st.sel.as_ref().unwrap();
            for v in sel.facet_vertices(t).iter() {
                o.v3(v);
            }
            let tr = &sel.get_triangles()[t as usize];
            for e in 0..3usize {
                let (vi, vj) = (tr.verts_idxs[e], tr.verts_idxs[(e + 1) % 3]);
                let first = sel.neighbor_child(t, vi, vj, Partition::First);
                let second = sel.neighbor_child(t, vi, vj, Partition::Second);
                let mid = sel.triangle_midpoint(t, vi, vj);
                let subs = sel.triangle_subtriangles(t, vi, vj);
                // the `const Triangle &` overloads answer the same
                assert_eq!(tr.neighbor_child(sel, vi, vj, Partition::First), first, "{at}");
                assert_eq!(tr.neighbor_child(sel, vi, vj, Partition::Second), second, "{at}");
                assert_eq!(tr.triangle_midpoint(sel, vi, vj), mid, "{at}");
                assert_eq!(tr.triangle_subtriangles(vi, vj), subs, "{at}");
                for x in [vi, vj, first, second, mid, subs.0, subs.1] {
                    o.i(x as i64);
                }
            }
        }
        "neighbor_edges" => {
            let f = args.i();
            args.end();
            let sel = st.sel.as_ref().unwrap();
            let tr = &sel.get_triangles()[f as usize];
            for e in 0..3usize {
                let n = sel.get_neighbors()[f as usize][e];
                let (vi, vj) = (tr.verts_idxs[(e + 1) % 3], tr.verts_idxs[e]);
                let subs = sel.triangle_subtriangles(n, vi, vj);
                for x in [n, vi, vj, sel.neighbor_child(n, vi, vj, Partition::First), sel.neighbor_child(n, vi, vj, Partition::Second), sel.triangle_midpoint(n, vi, vj), subs.0, subs.1] {
                    o.i(x as i64);
                }
            }
        }
        "select_unsplit_triangle" => {
            let f = args.i();
            let hit = args.v3();
            args.end();
            let sel = st.sel.as_ref().unwrap();
            o.i(sel.select_unsplit_triangle(&hit, f) as i64);
            o.i(sel.select_unsplit_triangle_n(&hit, f, &sel.get_neighbors()[f as usize]) as i64);
        }
        "fill_unpainted" | "replace_state" | "shift_states_above" => {
            let (a, c) = (args.i(), args.i());
            args.end();
            let data = st.sel.as_ref().unwrap().serialize();
            let mut s2 = new_selector(&st.its);
            // gen.cpp: s2.deserialize(data, true) with the default arguments
            s2.deserialize(&data, true, EnforcerBlockerType::EXTRUDER_MAX, EnforcerBlockerType::NONE, EnforcerBlockerType::NONE);
            match tag {
                "fill_unpainted" => s2.fill_unpainted(EnforcerBlockerType(a as i8)),
                "replace_state" => s2.replace_state(EnforcerBlockerType(a as i8), EnforcerBlockerType(c as i8)),
                _ => s2.shift_states_above(EnforcerBlockerType(a as i8), c),
            }
            o.selector(&s2);
        }
        "read_paint_nibble" => {
            let bits = args.bits();
            let mut ibit = args.i();
            args.end();
            let mut nibble = -7;
            let ok = read_paint_nibble(&bits, &mut ibit, &mut nibble);
            o.b(ok);
            o.i(ibit as i64);
            o.i(nibble as i64);
        }
        "clamp_paint_state" => {
            let (s, m) = (args.i(), args.i());
            args.end();
            o.i(clamp_paint_state(s, EnforcerBlockerType(m as i8)).0 as i64);
        }
        "deserialize" => {
            let d = args.data();
            let (max_ebt, to_delete, replace) = (args.i(), args.i(), args.i());
            args.end();
            let mut sel = new_selector(&st.its);
            sel.deserialize(&d, true, EnforcerBlockerType(max_ebt as i8), EnforcerBlockerType(to_delete as i8), EnforcerBlockerType(replace as i8));
            o.selector(&sel);
            for s in [0, 1, 2, 3, 17, 18, 33, 96, 100, 120] {
                o.b(TriangleSelector::has_facets_data(&d, EnforcerBlockerType(s)));
            }
            st.sel = Some(sel);
        }
        "deserialize_more" => {
            let d = args.data();
            args.end();
            let sel = st.sel.as_mut().unwrap();
            sel.deserialize(&d, false, EnforcerBlockerType::EXTRUDER_MAX, EnforcerBlockerType::NONE, EnforcerBlockerType::NONE);
            o.selector(sel);
        }
        "select_patch" => {
            let kind = args.i();
            let f = args.i();
            let (c1, c2, camera) = (args.v3(), args.v3(), args.v3());
            let (radius, z_world, height) = (args.f(), args.f(), args.f());
            let clp = args.clp();
            let state = args.i();
            let splitting = args.i() != 0;
            let highlight = args.f();
            let trafo = args.trafo();
            args.end();
            let mut trafo_no_translate = trafo;
            for r in 0..3 {
                trafo_no_translate[(r, 3)] = 0.0;
            }
            let cursor = match kind {
                0 => SinglePointCursor::cursor_factory(&c1, &camera, radius, CursorType::Sphere, &trafo, &clp),
                1 => SinglePointCursor::cursor_factory(&c1, &camera, radius, CursorType::Circle, &trafo, &clp),
                2 => DoublePointCursor::cursor_factory(&c1, &c2, &camera, radius, CursorType::Sphere, &trafo, &clp),
                3 => DoublePointCursor::cursor_factory(&c1, &c2, &camera, radius, CursorType::Circle, &trafo, &clp),
                _ => SinglePointCursor::cursor_factory_height_range(z_world, &camera, height, &trafo, &clp),
            };
            let sel = st.sel.as_mut().unwrap();
            sel.select_patch(f, cursor, EnforcerBlockerType(state as i8), &trafo_no_translate, splitting, highlight);
            o.f(sel.edge_limit_sqr());
            o.f(sel.old_cursor_radius_sqr());
            o.selector(sel);
        }
        "bucket_fill" | "seed_fill" => {
            let f = args.i();
            let hit = args.v3();
            let clp = args.clp();
            let angle = args.f();
            let sel = st.sel.as_mut().unwrap();
            let state;
            if tag == "bucket_fill" {
                let propagate = args.i() != 0;
                let force = args.i() != 0;
                state = args.i();
                args.end();
                sel.bucket_fill_select_triangles(&hit, f, &clp, angle, propagate, force);
            } else {
                let highlight = args.f();
                let force = args.i() != 0;
                state = args.i();
                let trafo_no_translate = args.trafo();
                args.end();
                sel.seed_fill_select_triangles(&hit, f, &trafo_no_translate, &clp, angle, highlight, force);
            }
            let contour = sel.get_seed_fill_contour();
            o.u(contour.len());
            for e in &contour {
                o.i(e.x as i64);
                o.i(e.y as i64);
            }
            let mut fill_state = -1;
            let fill = sel.get_seed_fill_mesh(&mut fill_state);
            o.u(fill.vertices.len());
            o.u(fill.indices.len());
            for t in &fill.indices {
                write!(o.0, " {} {} {}", t[0], t[1], t[2]).unwrap();
            }
            o.i(fill_state as i64);
            sel.seed_fill_apply_on_triangles(EnforcerBlockerType(state as i8));
            o.selector(sel);
        }
        "bucket_fill_skipped" => {
            args.end();
        }
        other => panic!("{at}: unknown tag {other}"),
    }
    check(at, &o, want);
}

/// Runs every line of a vectors file; the number of lines per tag is returned.
fn run(text: &str) -> BTreeMap<String, usize> {
    let mut counts = BTreeMap::new();
    let mut st = State { its: indexed_triangle_set::default(), sel: None };
    for (lineno, line) in text.lines().enumerate() {
        let (args, want) = match line.find(" ->") {
            Some(k) => (&line[..k], &line[k + 3..]),
            None => (line, ""),
        };
        let mut t = Tok(args.split_whitespace());
        let tag = t.s();
        *counts.entry(tag.to_string()).or_insert(0) += 1;
        let at = format!("line {} ({})", lineno + 1, tag);
        run_line(&mut st, tag, &mut t, want, &at);
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
fn subdivision_and_tree_queries_match_the_pin() {
    let counts = run(&read_vectors("triangle_selector/vectors_subdivide"));
    eprintln!("{counts:?}");
    assert!(counts["set_facets_with_subdivision"] >= 40);
    assert!(counts["edges"] >= 500);
    assert!(counts["fill_unpainted"] == 40 && counts["replace_state"] == 40 && counts["shift_states_above"] == 40);
}

#[test]
fn region_painting_matches_the_pin() {
    let counts = run(&read_vectors("triangle_selector/vectors_region"));
    eprintln!("{counts:?}");
    assert!(counts["paint_region"] >= 80);
    assert_eq!(counts["paint_region_empty"], 50);
}

#[test]
fn paint_stream_decoding_matches_the_pin() {
    let counts = run(&read_vectors("triangle_selector/vectors_stream"));
    expect(&counts, &[("read_paint_nibble", 200), ("clamp_paint_state", 100), ("mesh", 150), ("deserialize", 150), ("deserialize_more", counts.get("deserialize_more").copied().unwrap_or(0))]);
}

#[test]
fn brush_strokes_and_fills_match_the_pin() {
    let counts = run(&read_vectors("triangle_selector/vectors_cursor"));
    eprintln!("{counts:?}");
    assert!(counts["select_patch"] >= 80);
}
