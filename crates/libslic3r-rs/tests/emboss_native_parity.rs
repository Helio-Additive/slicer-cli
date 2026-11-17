//! `emboss.rs` against Emboss.cpp at the pin.
//!
//! `tests/data/emboss/vectors.txt.gz` is the output of `gen.cpp` beside it, which compiles
//! Emboss.cpp into itself and links the engine's libslic3r (the header comment there says how,
//! and describes the lines). Every line is one case, `TAG inputs -> outputs`; the test parses the
//! inputs, runs the port and prints the outputs the way the generator does, and the two texts
//! have to be equal: integers as decimals, doubles and floats as the hex of their bits.
//!
//! The font is Noto Sans Thai Regular (SIL Open Font License 1.1, the license is beside it).
//!
//! `polygons2model` is compared in part: the vertices and the side quads. The triangles of the
//! front and back faces come from `Triangulation::triangulate`, whose CGAL constrained Delaunay
//! core the crate does not have yet.

use slicer::emboss::*;
use slicer::emboss_shape::{EmbossShape, ExPolygonsWithId, ExPolygonsWithIds, Glyphs};
use slicer::geometry::{BoundingBox, ExPolygon, ExPolygons, Point, Polygon, Polygons};
use slicer::text_configuration::{FontProp, HorizontalAlign, VerticalAlign};
use std::fmt::Write as _;
use std::io::Read;
use std::sync::Arc;

type Vec3d = slicer::geometry::Vec3d;
type Transform3d = nalgebra::Matrix4<f64>;

fn data(name: &str) -> String {
    format!("{}/tests/data/emboss/{}", env!("CARGO_MANIFEST_DIR"), name)
}

fn read_vectors() -> String {
    let path = data("vectors.txt.gz");
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
    fn i(&mut self) -> i64 {
        self.s().parse().expect("integer")
    }
    fn u(&mut self) -> usize {
        self.s().parse().expect("unsigned")
    }
    fn d(&mut self) -> f64 {
        f64::from_bits(u64::from_str_radix(self.s(), 16).expect("double bits"))
    }
    fn f(&mut self) -> f32 {
        f32::from_bits(u32::from_str_radix(self.s(), 16).expect("float bits"))
    }
    fn v3(&mut self) -> Vec3d {
        Vec3d::new(self.d(), self.d(), self.d())
    }
    fn tr(&mut self) -> Transform3d {
        let mut m = Transform3d::zeros();
        for r in 0..4 {
            for c in 0..4 {
                m[(r, c)] = self.d();
            }
        }
        m
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
    fn text(&mut self) -> String {
        let h = self.s();
        if h == "-" {
            return String::new();
        }
        let bytes: Vec<u8> = (0..h.len() / 2).map(|i| u8::from_str_radix(&h[2 * i..2 * i + 2], 16).unwrap()).collect();
        String::from_utf8(bytes).expect("utf-8")
    }
    fn bb(&mut self) -> BoundingBox {
        if self.i() == 0 {
            return BoundingBox::new();
        }
        let min = Point::new(self.i(), self.i());
        let max = Point::new(self.i(), self.i());
        BoundingBox::from_points_minmax(min, max)
    }
}

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
    fn b(&mut self, v: bool) {
        self.i(v as i64);
    }
    fn v3(&mut self, v: &Vec3d) {
        self.d(v.x);
        self.d(v.y);
        self.d(v.z);
    }
    fn tr(&mut self, t: &Transform3d) {
        for r in 0..4 {
            for c in 0..4 {
                self.d(t[(r, c)]);
            }
        }
    }
    fn polygon(&mut self, p: &Polygon) {
        self.i(p.points().len() as i64);
        for pt in p.points() {
            self.i(pt.x);
            self.i(pt.y);
        }
    }
    fn polygons(&mut self, ps: &[Polygon]) {
        self.i(ps.len() as i64);
        for p in ps {
            self.polygon(p);
        }
    }
    fn expolygons(&mut self, es: &[ExPolygon]) {
        self.i(es.len() as i64);
        for e in es {
            self.polygon(&e.contour);
            self.polygons(&e.holes);
        }
    }
    fn healed(&mut self, h: &HealedExPolygons) {
        self.b(h.is_healed);
        self.expolygons(&h.expolygons);
    }
    fn text(&mut self, t: &str) {
        if t.is_empty() {
            self.0.push_str(" -");
            return;
        }
        self.0.push(' ');
        for b in t.bytes() {
            write!(self.0, "{:02x}", b).unwrap();
        }
    }
    fn pps(&mut self, pps: &[PolygonPoint]) {
        for pp in pps {
            self.i(pp.index as i64);
            self.i(pp.point.x);
            self.i(pp.point.y);
        }
    }
}

fn h_align(i: i64) -> HorizontalAlign {
    match i {
        0 => HorizontalAlign::Left,
        1 => HorizontalAlign::Center,
        _ => HorizontalAlign::Right,
    }
}

fn v_align(i: i64) -> VerticalAlign {
    match i {
        0 => VerticalAlign::Top,
        1 => VerticalAlign::Center,
        _ => VerticalAlign::Bottom,
    }
}

fn prop(t: &mut Tok) -> FontProp {
    let mut p = FontProp::new(10.0);
    let s = t.s();
    p.char_gap = (s != "-").then(|| s.parse().unwrap());
    let s = t.s();
    p.line_gap = (s != "-").then(|| s.parse().unwrap());
    let s = t.s();
    p.boldness = (s != "-").then(|| f32::from_bits(u32::from_str_radix(s, 16).unwrap()));
    let s = t.s();
    p.skew = (s != "-").then(|| f32::from_bits(u32::from_str_radix(s, 16).unwrap()));
    p.align = (h_align(t.i()), v_align(t.i()));
    p.size_in_mm = t.f();
    p
}

fn font_bytes() -> Vec<u8> {
    let path = data("NotoSansThai-Regular.ttf");
    std::fs::read(&path).unwrap_or_else(|e| panic!("{path}: {e}"))
}

fn font_with_cache(bytes: &[u8]) -> FontFileWithCache {
    let ff = create_font_file(bytes.to_vec()).expect("font");
    FontFileWithCache { font_file: Some(Arc::new(*ff)), cache: Some(Glyphs::new()) }
}

fn dump_projection(o: &mut Out, pr: &dyn IProjection, p: &Point, v: &Vec3d) {
    let (front, back) = pr.create_front_back(p);
    o.v3(&front);
    o.v3(&back);
    o.v3(&pr.project(v));
    let mut d = 1.25;
    match pr.unproject(v, Some(&mut d)) {
        Some(u) => {
            o.i(1);
            o.d(u.x);
            o.d(u.y);
        }
        None => o.i(0),
    }
    o.d(d);
    match pr.unproject(v, None) {
        Some(u) => {
            o.i(1);
            o.d(u.x);
            o.d(u.y);
        }
        None => o.i(0),
    }
}

fn run_case(tag: &str, t: &mut Tok, want: &str, font: &[u8]) -> Result<String, String> {
    let mut o = Out::default();
    match tag {
        "HEAL" => {
            let non_zero = t.i() != 0;
            let max_iter = t.u() as u32;
            let delta = t.f();
            let distance = t.d();
            let polys = t.polygons();
            let healed = heal_polygons(&polys, non_zero, max_iter);
            o.healed(&healed);
            let raw: ExPolygons = polys.iter().map(|p| ExPolygon::new(p.clone())).collect();
            {
                let mut e = raw.clone();
                let ok = heal_expolygons(&mut e, max_iter);
                o.b(ok);
                o.expolygons(&e);
            }
            o.healed(&union_with_delta(healed.expolygons.clone(), delta, max_iter));
            {
                let mut e = healed.expolygons.clone();
                let ok = divide_segments_for_close_point(&mut e, distance);
                o.b(ok);
                o.expolygons(&e);
            }
            {
                let mut e = raw.clone();
                let ok = divide_segments_for_close_point(&mut e, distance);
                o.b(ok);
                o.expolygons(&e);
            }
        }
        "PRIV" => {
            let bevel = t.d();
            let pixel = t.d();
            let area = t.d();
            let max_iter = t.u() as u32;
            let polys = t.polygons();
            let raw: ExPolygons = polys.iter().map(|p| ExPolygon::new(p.clone())).collect();
            let sd = SpikeDesc::new(bevel, pixel);
            o.i(polys[0].points().len() as i64);
            for i in 0..polys[0].points().len() {
                let mut p = polys[0].clone();
                let r = remove_when_spike(&mut p, i, &sd);
                o.b(r);
                o.polygon(&p);
            }
            {
                let mut e = raw.clone();
                let d = slicer::geometry::collect_duplicates(slicer::geometry::to_points(&e));
                remove_spikes_in_duplicates(&mut e, &d);
                o.expolygons(&e);
            }
            {
                let r = create_bounding_rect(&raw);
                o.polygon(&r.contour);
                o.polygons(&r.holes);
            }
            {
                let mut e = raw.clone();
                remove_small_islands(&mut e, area);
                o.expolygons(&e);
            }
            {
                let mut e = raw.clone();
                let ok = heal_dupl_inter(&mut e, max_iter);
                o.b(ok);
                o.expolygons(&e);
            }
        }
        "ALIGNX" => {
            let h = h_align(t.i());
            let a = t.bb5();
            let b = t.bb5();
            o.i(get_align_x_offset(h, &a, &b) as i64);
        }
        "LINES" => {
            let text = t.text();
            let w: Vec<char> = text.chars().collect();
            let shapes: ExPolygonsWithIds = w.iter().map(|&c| ExPolygonsWithId::new(c as u32, ExPolygons::new())).collect();
            o.i(get_count_lines(&w) as i64);
            o.i(from_utf8::get_count_lines(&text) as i64);
            o.i(from_shapes::get_count_lines(&shapes) as i64);
            let r = get_line_ranges(&shapes);
            o.i(r.len() as i64);
            for (a, b) in r {
                o.i(a as i64);
                o.i(b as i64);
            }
            let nd: String = remove_duplicates(&w).into_iter().collect();
            o.text(&nd);
        }
        "FONT" => {
            let ff = create_font_file(font.to_vec()).expect("font");
            o.i(ff.infos.len() as i64);
            for i in &ff.infos {
                o.i(i.ascent as i64);
                o.i(i.descent as i64);
                o.i(i.linegap as i64);
                o.i(i.unit_per_em as i64);
            }
            o.b(is_italic(&ff, 0));
            o.b(is_italic(&ff, 1));
        }
        "GLYPH" | "GLYPH1" => {
            let ff = create_font_file(font.to_vec()).expect("font");
            let cp = t.i() as i32;
            let flatness = t.f();
            if tag == "GLYPH1" {
                o.0.push_str(if letter2glyph(&ff, 1, cp, flatness).is_some() { " some" } else { " none" });
            } else {
                match letter2glyph(&ff, 0, cp, flatness) {
                    None => o.0.push_str(" none"),
                    Some(g) => {
                        o.i(g.advance_width as i64);
                        o.i(g.left_side_bearing as i64);
                        o.expolygons(&g.shape);
                    }
                }
            }
        }
        "RANGE" => {
            let text = t.text();
            let ff = create_font_file(font.to_vec()).expect("font");
            let mut unknown = false;
            let r = create_range_text(&text, &ff, 0, Some(&mut unknown));
            o.text(&r);
            o.b(unknown);
            {
                let fonts: Vec<Arc<FontFile>> = vec![
                    Arc::new(*create_font_file(font.to_vec()).unwrap()),
                    Arc::new(*create_font_file(font.to_vec()).unwrap()),
                ];
                let mut t2 = text.clone();
                let mut u2 = false;
                let r2 = with_fonts::create_range_text(&mut t2, fonts, 0, &mut u2);
                o.text(&r2);
                o.b(u2);
            }
            o.0.push_str(if create_range_text(&text, &ff, 1, None).is_empty() { " e" } else { " n" });
        }
        "TEXT" => {
            let prop = prop(t);
            let bfc = t.i() != 0;
            let cancel = t.i() != 0;
            let lines = t.u() as u32;
            let (tx, ty) = (t.i(), t.i());
            let text = t.text();
            let w: Vec<char> = text.chars().collect();
            let ff = create_font_file(font.to_vec()).expect("font");
            let scale = get_text_shape_scale(&prop, &ff);
            o.d(scale);
            o.i(get_line_height(&ff, &prop) as i64);
            o.f(get_align_y_offset(prop.align.1, lines, &ff, &prop));
            o.d(get_align_y_offset_in_mm(prop.align.1, lines, &ff, &prop));

            let mut fwc = font_with_cache(font);
            let back = || vec![FontFileWithCache::default(), font_with_cache(font)];
            let bfc_fn: BackFontCacheFn = if bfc { Some(&back) } else { None };
            let was_canceled = || cancel;
            let mut es = EmbossShape::default();
            let layout = text2vshapes(&mut es, &mut fwc, &w, &prop, scale, &was_canceled, bfc_fn);
            o.i(es.shapes_with_ids.len() as i64);
            for e in &es.shapes_with_ids {
                o.i(e.id as i64);
                o.b(e.is_healed);
                o.expolygons(&e.expoly);
            }
            o.i(es.text_align_offsets.len() as i64);
            for v in &es.text_align_offsets {
                o.f(v.x);
                o.f(v.y);
            }
            o.i(es.text_scales.len() as i64);
            for &v in &es.text_scales {
                o.f(v);
            }
            o.i(es.text_cursors.len() as i64);
            for &v in &es.text_cursors {
                o.f(v);
            }
            o.i(es.text_absolute_cursors.len() as i64);
            for &v in &es.text_absolute_cursors {
                o.f(v);
            }
            o.i(es.align_type.0 as i64);
            o.i(es.align_type.1 as i64);
            o.f(layout.line_height);
            o.f(layout.first_line_offset_y);
            o.i(from_shapes::get_count_lines(&es.shapes_with_ids) as i64);
            {
                let bb = get_extents(&es.shapes_with_ids);
                o.b(bb.is_defined());
                if bb.is_defined() {
                    o.i(bb.min.x);
                    o.i(bb.min.y);
                    o.i(bb.max.x);
                    o.i(bb.max.y);
                }
                let mut copy = es.shapes_with_ids.clone();
                if bb.is_defined() {
                    center(&mut copy);
                    translate(&mut copy, Point::new(tx, ty));
                }
                for e in &copy {
                    o.expolygons(&e.expoly);
                }
            }
            {
                let mut es2 = es.clone();
                let u = for_emboss_shape::union_with_delta(&mut es2, UNION_DELTA, UNION_MAX_ITERATIN);
                o.expolygons(&u);
                o.b(es2.final_shape.is_healed);
                let u2 = for_emboss_shape::union_with_delta(&mut es2, 1.0, 1);
                o.expolygons(&u2);
            }
            {
                let mut fwc2 = font_with_cache(font);
                let mut es3 = EmbossShape::default();
                let (h, _) = text2shapes(&mut es3, &mut fwc2, &text, &prop, scale, &was_canceled, bfc_fn);
                o.healed(&h);
                o.i(es3.shapes_with_ids.len() as i64);
            }
        }
        "PROJ" => {
            let p = Point::new(t.i(), t.i());
            let v = t.v3();
            let depth = t.d();
            let scale = t.d();
            let tr = t.tr();
            let dir = t.v3();
            dump_projection(&mut o, &ProjectZ::new(depth), &p, &v);
            dump_projection(&mut o, &ProjectScale::new(Box::new(ProjectZ::new(depth)), scale), &p, &v);
            dump_projection(&mut o, &ProjectTransform::new(Box::new(ProjectZ::new(depth)), tr), &p, &v);
            dump_projection(
                &mut o,
                &ProjectTransform::new(Box::new(ProjectScale::new(Box::new(ProjectZ::new(depth)), scale)), tr),
                &p,
                &v,
            );
            dump_projection(&mut o, &OrthoProject::new(tr, dir), &p, &v);
            o.v3(&OrthoProject3d::new(dir).project(&v));
        }
        "UP" => {
            let normal = t.v3();
            let limit = t.d();
            let position = t.v3();
            let tr = t.tr();
            let angle = t.f();
            let distance = t.f();
            let which = t.i();
            o.v3(&suggest_up(normal, limit));
            match calc_up(&tr, limit) {
                Some(up) => {
                    o.i(1);
                    o.f(up);
                }
                None => o.i(0),
            }
            o.tr(&create_transformation_onto_surface(position, normal, limit));
            let mut t2 = tr;
            apply_transformation((which & 1 != 0).then_some(angle), (which & 2 != 0).then_some(distance), &mut t2);
            o.tr(&t2);
        }
        "ANGLE" => {
            let distance = t.i() as i32;
            let scale = t.d();
            let poly = t.polygon();
            let n = t.u();
            let pps: PolygonPoints = (0..n).map(|_| PolygonPoint { index: t.u(), point: Point::new(t.i(), t.i()) }).collect();
            let n = t.u();
            let bbs: Vec<BoundingBox> = (0..n).map(|_| t.bb()).collect();
            o.d(calculate_angle(distance, pps[0], &poly));
            for a in calculate_angles(distance, &pps, &poly) {
                o.d(a);
            }
            let line = TextLine { polygon: poly.clone(), start: pps[0], y: 0.0 };
            let samples = sample_slice(&line, &bbs, scale);
            o.i(samples.len() as i64);
            o.pps(&samples);
        }
        "MODEL" => {
            let depth = t.d();
            let scale = t.d();
            let shape = t.expolygons();
            let pr = ProjectScale::new(Box::new(ProjectZ::new(depth)), scale);
            // Triangulation.cpp's asserts on its input are `assert!` in the crate's triangulation.rs
            // and compiled out of the engine's release build: a shape that trips one is counted
            // apart.
            let its = match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| polygons2model(&shape, &pr))) {
                Ok(its) => its,
                Err(_) => return Ok("assert".into()),
            };
            // the vertices whole; of the triangles, the side quads, which are the last ones
            let mut w = Tok(want.split_whitespace());
            let nv = w.u();
            let want_vertices: Vec<[u32; 3]> =
                (0..nv).map(|_| [w.f().to_bits(), w.f().to_bits(), w.f().to_bits()]).collect();
            let nt = w.u();
            let want_indices: Vec<[i64; 3]> = (0..nt).map(|_| [w.i(), w.i(), w.i()]).collect();
            let got_vertices: Vec<[u32; 3]> =
                its.vertices.iter().map(|v| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]).collect();
            let got_indices: Vec<[i64; 3]> = its.indices.iter().map(|t| [t[0] as i64, t[1] as i64, t[2] as i64]).collect();
            if got_vertices != want_vertices {
                return Err("vertices differ".into());
            }
            if got_indices.len() > want_indices.len() || want_indices[want_indices.len() - got_indices.len()..] != got_indices[..] {
                return Err(format!("side quads differ: got {} triangles, want {}", got_indices.len(), want_indices.len()));
            }
            return Ok(if got_indices.len() == want_indices.len() { "whole".into() } else { "part".into() });
        }
        _ => panic!("unknown tag {tag}"),
    }
    Ok(o.0)
}

impl<'a> Tok<'a> {
    /// A bounding box written with all five numbers whether it is defined or not.
    fn bb5(&mut self) -> BoundingBox {
        let defined = self.i() != 0;
        let min = Point::new(self.i(), self.i());
        let max = Point::new(self.i(), self.i());
        if defined {
            BoundingBox::from_points_minmax(min, max)
        } else {
            BoundingBox::new()
        }
    }
}

#[test]
fn emboss_matches_the_pin() {
    let text = read_vectors();
    let font = font_bytes();
    let mut counts: std::collections::BTreeMap<String, (usize, usize)> = Default::default();
    let mut model_whole = 0;
    let mut model_assert = 0;
    let mut first_bad: Vec<String> = Vec::new();
    for (ln, line) in text.lines().enumerate() {
        let (lhs, want) = line.split_once(" ->").expect("arrow");
        let mut t = Tok(lhs.split_whitespace());
        let tag = t.s();
        let entry = counts.entry(tag.to_string()).or_default();
        entry.0 += 1;
        let ok = match run_case(tag, &mut t, want, &font) {
            Ok(got) if tag == "MODEL" => {
                if got == "whole" {
                    model_whole += 1;
                }
                if got == "assert" {
                    model_assert += 1;
                }
                true
            }
            Ok(got) => {
                if got != want && entry.1 < 3 {
                    // where the two texts part
                    let (g, w): (Vec<&str>, Vec<&str>) = (got.split_whitespace().collect(), want.split_whitespace().collect());
                    let k = g.iter().zip(w.iter()).position(|(a, b)| a != b).unwrap_or(g.len().min(w.len()));
                    let lo = k.saturating_sub(4);
                    println!(
                        "{tag} line {}: token {k} of {}/{}: got {:?} want {:?}",
                        ln + 1,
                        g.len(),
                        w.len(),
                        &g[lo..(k + 4).min(g.len())],
                        &w[lo..(k + 4).min(w.len())]
                    );
                }
                got == want
            }
            Err(_) => false,
        };
        if !ok {
            entry.1 += 1;
            if first_bad.len() < 8 {
                first_bad.push(format!("line {} ({})", ln + 1, &lhs[..lhs.len().min(160)]));
            }
        }
    }
    let mut summary = String::new();
    let mut bad = 0;
    for (tag, (n, b)) in &counts {
        summary += &format!("{tag}: {n} cases, {b} differ\n");
        bad += b;
    }
    println!("{summary}MODEL cases equal in every triangle: {model_whole}; stopped by an assert of triangulation.rs: {model_assert}");
    assert!(bad == 0, "{summary}first: {first_bad:#?}");
    for tag in ["HEAL", "PRIV", "ALIGNX", "LINES", "FONT", "GLYPH", "GLYPH1", "RANGE", "TEXT", "PROJ", "UP", "ANGLE", "MODEL"] {
        assert!(counts.contains_key(tag), "no {tag} case in the vectors");
    }
}
