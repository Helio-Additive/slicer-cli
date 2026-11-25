//! Clipper2's clipper.h and clipper.core.h free functions under their C++ names (crate
//! `slicer::clipper2`) against the pin's Clipper2 (src/clipper2, libClipper2.a and its headers),
//! on the random paths of `tests/data/clipper2_cpp_names/gen.cpp` (13 groups of 1,000 calls,
//! 200 calls per group of the double overloads; gzip -9n). clipper.core.h: GetBounds x3, StripNearEqual x2, StripDuplicates x2,
//! CrossProduct x2, DotProduct x2, Area x2, GetSegmentIntersectPt, Rect64 Width / Height /
//! Contains x2. clipper.h: BooleanOp, Intersect, Union x2, Difference, Xor, InflatePaths,
//! TranslatePath(s), RectClip x2, RectClipLines x2, TrimCollinear, Ellipse,
//! RamerDouglasPeucker x2, SimplifyPath, Distance, Length, NearCollinear, MakePath, MakePathD,
//! Ellipse(Rect64), and the PathD / PathsD / RectD / PointD overloads of all of them. Integers and doubles
//! (bit for bit) must be equal.

use slicer::clipper2::*;
use std::io::Read;

struct Tok<'a>(std::str::SplitWhitespace<'a>);

impl<'a> Tok<'a> {
    fn s(&mut self) -> &'a str {
        self.0.next().expect("token")
    }
    fn i(&mut self) -> i64 {
        self.s().parse().expect("integer")
    }
    fn d(&mut self) -> f64 {
        f64::from_bits(u64::from_str_radix(self.s(), 16).expect("double"))
    }
    fn b(&mut self) -> bool {
        self.i() != 0
    }
    fn path(&mut self) -> Path64 {
        let n = self.i() as usize;
        (0..n).map(|_| Point64::new(self.i(), self.i())).collect()
    }
    fn paths(&mut self) -> Paths64 {
        let n = self.i() as usize;
        (0..n).map(|_| self.path()).collect()
    }
    fn rect(&mut self) -> Rect64 {
        Rect64::new(self.i(), self.i(), self.i(), self.i())
    }
    fn rectd(&mut self) -> [u64; 4] {
        [self.d().to_bits(), self.d().to_bits(), self.d().to_bits(), self.d().to_bits()]
    }
    fn pd(&mut self) -> PointD {
        PointD::new(self.d(), self.d())
    }
    fn pathd(&mut self) -> PathD {
        let n = self.i() as usize;
        (0..n).map(|_| self.pd()).collect()
    }
    fn pathsd(&mut self) -> PathsD {
        let n = self.i() as usize;
        (0..n).map(|_| self.pathd()).collect()
    }
    fn rect_d(&mut self) -> RectD {
        RectD { left: self.d(), top: self.d(), right: self.d(), bottom: self.d() }
    }
    fn arrow(&mut self) {
        assert_eq!(self.s(), "->");
    }
}

fn flat(p: &Path64) -> Vec<(i64, i64)> {
    p.iter().map(|q| (q.x, q.y)).collect()
}

fn flats(ps: &Paths64) -> Vec<Vec<(i64, i64)>> {
    ps.iter().map(flat).collect()
}

fn flatd(p: &PathD) -> Vec<(u64, u64)> {
    p.iter().map(|q| (q.x.to_bits(), q.y.to_bits())).collect()
}

fn flatsd(ps: &PathsD) -> Vec<Vec<(u64, u64)>> {
    ps.iter().map(flatd).collect()
}

fn rectd_of(r: &RectD) -> [u64; 4] {
    [r.left.to_bits(), r.top.to_bits(), r.right.to_bits(), r.bottom.to_bits()]
}

fn rect_of(r: &Rect64) -> [i64; 4] {
    [r.left, r.top, r.right, r.bottom]
}

fn fill_rule(i: i64) -> FillRule {
    [FillRule::EvenOdd, FillRule::NonZero, FillRule::Positive, FillRule::Negative][i as usize]
}

fn clip_type(i: i64) -> ClipType {
    [ClipType::NoClip, ClipType::Intersection, ClipType::Union, ClipType::Difference, ClipType::Xor][i as usize]
}

fn join_type(i: i64) -> JoinType {
    [JoinType::Square, JoinType::Bevel, JoinType::Round, JoinType::Miter][i as usize]
}

fn end_type(i: i64) -> EndType {
    [EndType::Polygon, EndType::Joined, EndType::Butt, EndType::Square, EndType::Round][i as usize]
}

#[test]
fn clipper2_free_functions_match_the_pin() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/clipper2_cpp_names/vectors.txt.gz");
    let mut text = String::new();
    flate2::read::GzDecoder::new(std::fs::File::open(path).unwrap()).read_to_string(&mut text).unwrap();
    let mut counts = std::collections::BTreeMap::<&'static str, (usize, usize)>::new();
    let mut first_bad: Vec<String> = Vec::new();
    let mut tally = |name: &'static str, ok: bool, line_no: usize| {
        let e = counts.entry(name).or_insert((0, 0));
        e.0 += 1;
        if !ok {
            e.1 += 1;
            if first_bad.len() < 10 {
                first_bad.push(format!("{name} line {}", line_no + 1));
            }
        }
    };
    let mut last_ps: Paths64 = Vec::new();
    for (ln, line) in text.lines().enumerate() {
        let mut t = Tok(line.split_whitespace());
        match t.s() {
            "bounds" => {
                let (p, ps) = (t.path(), t.paths());
                t.arrow();
                // The generator's strip / area lines that follow run the paths overloads on these.
                last_ps = ps.clone();
                tally("GetBounds(Path64)", rect_of(&get_bounds(&p)) == rect_of(&t.rect()), ln);
                tally("GetBounds(Paths64)", rect_of(&get_bounds_paths(&ps)) == rect_of(&t.rect()), ln);
                let r = get_bounds_d_from_64(&p);
                tally(
                    "GetBounds<double,int64_t>(Path64)",
                    [r.left.to_bits(), r.top.to_bits(), r.right.to_bits(), r.bottom.to_bits()] == t.rectd(),
                    ln,
                );
            }
            "strip_near_equal" => {
                let p = t.path();
                let md = t.d();
                let closed = t.b();
                t.arrow();
                tally("StripNearEqual(Path64)", flat(&strip_near_equal(&p, md, closed)) == flat(&t.path()), ln);
                tally(
                    "StripNearEqual(Paths64)",
                    flats(&strip_near_equal_paths(&last_ps, md, closed)) == flats(&t.paths()),
                    ln,
                );
            }
            "strip_duplicates" => {
                let p = t.path();
                let closed = t.b();
                t.arrow();
                let mut q = p.clone();
                strip_duplicates(&mut q, closed);
                tally("StripDuplicates(Path64)", flat(&q) == flat(&t.path()), ln);
                let mut qs = last_ps.clone();
                strip_duplicates_paths(&mut qs, closed);
                tally("StripDuplicates(Paths64)", flats(&qs) == flats(&t.paths()), ln);
            }
            "area" => {
                let p = t.path();
                t.arrow();
                tally("Area(Path64)", area(&p).to_bits() == t.d().to_bits(), ln);
                tally("Area(Paths64)", area_paths(&last_ps).to_bits() == t.d().to_bits(), ln);
            }
            "length" => {
                let p = t.path();
                let closed = t.b();
                t.arrow();
                tally("Length(Path64)", length(&p, closed).to_bits() == t.d().to_bits(), ln);
            }
            "points" => {
                let a = Point64::new(t.i(), t.i());
                let b = Point64::new(t.i(), t.i());
                let c = Point64::new(t.i(), t.i());
                let d = Point64::new(t.i(), t.i());
                let va = PointD::new(t.d(), t.d());
                let vb = PointD::new(t.d(), t.d());
                let ang = t.d();
                t.arrow();
                tally("CrossProduct(pt1, pt2, pt3)", cross_product(&a, &b, &c).to_bits() == t.d().to_bits(), ln);
                tally("CrossProduct(vec1, vec2)", cross_product_vec(&va, &vb).to_bits() == t.d().to_bits(), ln);
                tally("DotProduct(pt1, pt2, pt3)", dot_product(&a, &b, &c).to_bits() == t.d().to_bits(), ln);
                tally("DotProduct(vec1, vec2)", dot_product_vec(&va, &vb).to_bits() == t.d().to_bits(), ln);
                let mut ip = Point64::new(0, 0);
                let hit = get_segment_intersect_pt(&a, &b, &c, &d, &mut ip);
                let (wh, wx, wy) = (t.b(), t.i(), t.i());
                tally("GetSegmentIntersectPt", hit == wh && (ip.x, ip.y) == (wx, wy), ln);
                tally("Distance", distance(a, b).to_bits() == t.d().to_bits(), ln);
                tally("NearCollinear", near_collinear(&a, &b, &c, ang) == t.b(), ln);
            }
            "rect" => {
                let (r, r2) = (t.rect(), t.rect());
                let c = Point64::new(t.i(), t.i());
                t.arrow();
                tally("Rect64::Width", r.width() == t.i(), ln);
                tally("Rect64::Height", r.height() == t.i(), ln);
                tally("Rect64::Contains(Point64)", r.contains_point(&c) == t.b(), ln);
                tally("Rect64::Contains(Rect64)", r.contains(&r2) == t.b(), ln);
            }
            "boolean" => {
                let (ct, fr) = (clip_type(t.i()), fill_rule(t.i()));
                let (s, c) = (t.paths(), t.paths());
                t.arrow();
                tally("BooleanOp", flats(&boolean_op(ct, fr, &s, &c)) == flats(&t.paths()), ln);
                tally("Intersect", flats(&intersect(&s, &c, fr)) == flats(&t.paths()), ln);
                tally("Union(subjects, clips)", flats(&union(&s, &c, fr)) == flats(&t.paths()), ln);
                tally("Union(subjects)", flats(&union_subjects(&s, fr)) == flats(&t.paths()), ln);
                tally("Difference", flats(&difference(&s, &c, fr)) == flats(&t.paths()), ln);
                tally("Xor", flats(&xor(&s, &c, fr)) == flats(&t.paths()), ln);
            }
            "inflate" => {
                let s = t.paths();
                let delta = t.d();
                let (jt, et) = (join_type(t.i()), end_type(t.i()));
                let (ml, at) = (t.d(), t.d());
                t.arrow();
                tally("InflatePaths", flats(&inflate_paths(&s, delta, jt, et, ml, at)) == flats(&t.paths()), ln);
            }
            "translate" => {
                let s = t.paths();
                let (dx, dy) = (t.i(), t.i());
                t.arrow();
                tally("TranslatePath", flat(&translate_path(&s[0], dx, dy)) == flat(&t.path()), ln);
                tally("TranslatePaths", flats(&translate_paths(&s, dx, dy)) == flats(&t.paths()), ln);
            }
            "rectclip" => {
                let r = t.rect();
                let s = t.paths();
                t.arrow();
                tally("RectClip(rect, paths)", flats(&rect_clip(&r, &s)) == flats(&t.paths()), ln);
                tally("RectClip(rect, path)", flats(&rect_clip_path(&r, &s[0])) == flats(&t.paths()), ln);
                tally("RectClipLines(rect, lines)", flats(&rect_clip_lines(&r, &s)) == flats(&t.paths()), ln);
                tally("RectClipLines(rect, line)", flats(&rect_clip_lines_path(&r, &s[0])) == flats(&t.paths()), ln);
            }
            "simplify" => {
                let s = t.paths();
                let open = t.b();
                let eps = t.d();
                t.arrow();
                tally("TrimCollinear", flat(&trim_collinear(&s[0], open)) == flat(&t.path()), ln);
                tally("RamerDouglasPeucker(path)", flat(&ramer_douglas_peucker(&s[0], eps)) == flat(&t.path()), ln);
                tally("RamerDouglasPeucker(paths)", flats(&ramer_douglas_peucker_paths(&s, eps)) == flats(&t.paths()), ln);
                tally("SimplifyPath", flat(&simplify_path(&s[0], eps, !open)) == flat(&t.path()), ln);
            }
            "ellipse" => {
                let ctr = Point64::new(t.i(), t.i());
                let (rx, ry) = (t.d(), t.d());
                let steps = t.i() as usize;
                t.arrow();
                tally("Ellipse(center)", flat(&ellipse(&ctr, rx, ry, steps)) == flat(&t.path()), ln);
            }
            "boolean_d" => {
                let (ct, fr) = (clip_type(t.i()), fill_rule(t.i()));
                let prec = t.i() as i32;
                let (s, c) = (t.pathsd(), t.pathsd());
                t.arrow();
                tally("BooleanOp(PathsD)", flatsd(&boolean_op_d(ct, fr, &s, &c, prec).unwrap()) == flatsd(&t.pathsd()), ln);
                tally("Intersect(PathsD)", flatsd(&intersect_d(&s, &c, fr, prec).unwrap()) == flatsd(&t.pathsd()), ln);
                tally("Union(PathsD, PathsD)", flatsd(&union_d(&s, &c, fr, prec).unwrap()) == flatsd(&t.pathsd()), ln);
                tally("Union(PathsD)", flatsd(&union_subjects_d(&s, fr, prec).unwrap()) == flatsd(&t.pathsd()), ln);
                tally("Difference(PathsD)", flatsd(&difference_d(&s, &c, fr, prec).unwrap()) == flatsd(&t.pathsd()), ln);
                tally("Xor(PathsD)", flatsd(&xor_d(&s, &c, fr, prec).unwrap()) == flatsd(&t.pathsd()), ln);
            }
            "inflate_d" => {
                let s = t.pathsd();
                let delta = t.d();
                let (jt, et) = (join_type(t.i()), end_type(t.i()));
                let ml = t.d();
                let prec = t.i() as i32;
                let at = t.d();
                t.arrow();
                tally("InflatePaths(PathsD)", flatsd(&inflate_paths_d(&s, delta, jt, et, ml, prec, at).unwrap()) == flatsd(&t.pathsd()), ln);
            }
            "translate_d" => {
                let s = t.pathsd();
                let (dx, dy) = (t.d(), t.d());
                t.arrow();
                tally("TranslatePath(PathD)", flatd(&translate_path_d(&s[0], dx, dy)) == flatd(&t.pathd()), ln);
                tally("TranslatePaths(PathsD)", flatsd(&translate_paths_d(&s, dx, dy)) == flatsd(&t.pathsd()), ln);
            }
            "rectclip_d" => {
                let r = t.rect_d();
                let prec = t.i() as i32;
                let s = t.pathsd();
                t.arrow();
                tally("RectClip(RectD, PathsD)", flatsd(&rect_clip_d(&r, &s, prec).unwrap()) == flatsd(&t.pathsd()), ln);
                tally("RectClip(RectD, PathD)", flatsd(&rect_clip_path_d(&r, &s[0], prec).unwrap()) == flatsd(&t.pathsd()), ln);
                tally("RectClipLines(RectD, PathsD)", flatsd(&rect_clip_lines_d(&r, &s, prec).unwrap()) == flatsd(&t.pathsd()), ln);
                tally("RectClipLines(RectD, PathD)", flatsd(&rect_clip_lines_path_d(&r, &s[0], prec).unwrap()) == flatsd(&t.pathsd()), ln);
            }
            "simplify_d" => {
                let s = t.pathsd();
                let prec = t.i() as i32;
                let open = t.b();
                let eps = t.d();
                t.arrow();
                tally("TrimCollinear(PathD)", flatd(&trim_collinear_d(&s[0], prec, open).unwrap()) == flatd(&t.pathd()), ln);
                tally("RamerDouglasPeucker(PathD)", flatd(&ramer_douglas_peucker_d(&s[0], eps)) == flatd(&t.pathd()), ln);
                tally("RamerDouglasPeucker(PathsD)", flatsd(&ramer_douglas_peucker_paths_d(&s, eps)) == flatsd(&t.pathsd()), ln);
                tally("SimplifyPath(PathD)", flatd(&simplify_path_d(&s[0], eps, !open)) == flatd(&t.pathd()), ln);
                tally("Length(PathD)", length_d(&s[0], !open).to_bits() == t.d().to_bits(), ln);
            }
            "ellipse_d" => {
                let ctr = t.pd();
                let (rx, ry) = (t.d(), t.d());
                let steps = t.i() as usize;
                let er = t.rect();
                t.arrow();
                tally("Ellipse(PointD)", flatd(&ellipse_d(&ctr, rx, ry, steps)) == flatd(&t.pathd()), ln);
                tally("Ellipse(Rect64)", flat(&ellipse_rect(&er, steps)) == flat(&t.path()), ln);
            }
            "make_path" => {
                let n = t.i() as usize;
                let li: Vec<i64> = (0..n).map(|_| t.i()).collect();
                let n = t.i() as usize;
                let ld: Vec<f64> = (0..n).map(|_| t.d()).collect();
                t.arrow();
                tally("MakePath(std::vector)", flat(&make_path(&li).unwrap()) == flat(&t.path()), ln);
                tally("MakePathD(std::vector)", flatd(&make_path_d(&ld).unwrap()) == flatd(&t.pathd()), ln);
            }
            "core_d" => {
                let s = t.pathsd();
                let md = t.d();
                let closed = t.b();
                t.arrow();
                let p = s[0].clone();
                tally("GetBounds(PathD)", rectd_of(&get_bounds_d(&p)) == rectd_of(&t.rect_d()), ln);
                tally("GetBounds(PathsD)", rectd_of(&get_bounds_paths_d(&s)) == rectd_of(&t.rect_d()), ln);
                tally("StripNearEqual(PathD)", flatd(&strip_near_equal_d(&p, md, closed)) == flatd(&t.pathd()), ln);
                tally("StripNearEqual(PathsD)", flatsd(&strip_near_equal_paths_d(&s, md, closed)) == flatsd(&t.pathsd()), ln);
                let mut q = p.clone();
                strip_duplicates_d(&mut q, closed);
                tally("StripDuplicates(PathD)", flatd(&q) == flatd(&t.pathd()), ln);
                let mut qs = s.clone();
                strip_duplicates_paths_d(&mut qs, closed);
                tally("StripDuplicates(PathsD)", flatsd(&qs) == flatsd(&t.pathsd()), ln);
                tally("Area(PathD)", area_d(&p).to_bits() == t.d().to_bits(), ln);
                tally("Area(PathsD)", area_paths_d(&s).to_bits() == t.d().to_bits(), ln);
            }
            "points_d" => {
                let (a, b, c, d) = (t.pd(), t.pd(), t.pd(), t.pd());
                let v1 = Point64::new(t.i(), t.i());
                let v2 = Point64::new(t.i(), t.i());
                t.arrow();
                tally("CrossProduct(PointD x3)", cross_product_d(&a, &b, &c).to_bits() == t.d().to_bits(), ln);
                tally("DotProduct(PointD x3)", dot_product_d(&a, &b, &c).to_bits() == t.d().to_bits(), ln);
                tally("CrossProduct(Point64 vec1, vec2)", cross_product_vec64(&v1, &v2).to_bits() == t.d().to_bits(), ln);
                tally("DotProduct(Point64 vec1, vec2)", dot_product_vec64(&v1, &v2).to_bits() == t.d().to_bits(), ln);
                let mut ip = PointD::new(0.0, 0.0);
                let hit = get_segment_intersect_pt_d(&a, &b, &c, &d, &mut ip);
                let wh = t.b();
                let wp = t.pd();
                tally("GetSegmentIntersectPt(PointD)", hit == wh && (ip.x.to_bits(), ip.y.to_bits()) == (wp.x.to_bits(), wp.y.to_bits()), ln);
                tally("Distance(PointD)", distance_d(a, b).to_bits() == t.d().to_bits(), ln);
            }
            "rect_d" => {
                let r = t.rect_d();
                let c = t.pd();
                t.arrow();
                tally("RectD::Width", r.width().to_bits() == t.d().to_bits(), ln);
                tally("RectD::Height", r.height().to_bits() == t.d().to_bits(), ln);
                tally("RectD::Contains(PointD)", r.contains_point(&c) == t.b(), ln);
            }
            other => panic!("unknown tag {other}"),
        }
    }
    for (name, (n, bad)) in &counts {
        eprintln!("{name}: {n} cases, {bad} mismatches");
    }
    eprintln!("first mismatches: {first_bad:?}");
    assert_eq!(counts.len(), 39 + 39);
    assert!(counts.values().all(|&(_, bad)| bad == 0), "{counts:?}");
}
