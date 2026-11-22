//! `clipper_utils::cpp`, ClipperUtils.hpp / ClipperUtils.cpp under their C++ names, against vectors of the
//! oracle (tests/data/clipper_utils/gen.cpp): every overload is called once per case on the same inputs and
//! has to return the same paths in the same order, each with the same points from the same start vertex.
//!
//! The format of the vectors is in the header of gen.cpp. This file is written by a script from the table of
//! overloads, as gen.cpp is: `call()` below is that table.

use std::io::Read;

use slicer::clipper::{ClipType, EndType, JoinType, PolyFillType, PolyTree, POLY_TREE_ROOT};
use slicer::clipper_utils::cpp::{self, *};
use slicer::clipper_utils::ApplySafetyOffset;
use slicer::geometry::{BoundingBox, ExPolygon, Line, Point, Polygon, Polyline};
use slicer::surface::{Surface, SurfaceType};

struct Tok<'a> {
    it: std::str::Split<'a, char>,
    line_no: usize,
}

impl<'a> Tok<'a> {
    fn s(&mut self) -> &'a str {
        let line_no = self.line_no;
        self.it.next().unwrap_or_else(|| panic!("line {line_no}: a token is missing"))
    }
    fn i(&mut self) -> i64 {
        self.s().parse().expect("integer")
    }
    fn u(&mut self) -> usize {
        self.s().parse().expect("unsigned")
    }
    fn pt(&mut self) -> Point {
        let x = self.i();
        let y = self.i();
        Point { x, y }
    }
    fn pts(&mut self) -> Vec<Point> {
        let m = self.u();
        (0..m).map(|_| self.pt()).collect()
    }
    fn pgs(&mut self) -> Vec<Polygon> {
        let n = self.u();
        (0..n).map(|_| Polygon { points: self.pts() }).collect()
    }
    fn pls(&mut self) -> Vec<Polyline> {
        let n = self.u();
        (0..n)
            .map(|_| {
                let mut pl = Polyline::new();
                pl.points = self.pts();
                pl
            })
            .collect()
    }
    fn exs(&mut self) -> Vec<ExPolygon> {
        let n = self.u();
        (0..n)
            .map(|_| {
                let contour = Polygon { points: self.pts() };
                let holes = self.pgs();
                ExPolygon { contour, holes }
            })
            .collect()
    }
    fn lns(&mut self) -> Vec<Line> {
        let n = self.u();
        (0..n)
            .map(|_| {
                let a = self.pt();
                let b = self.pt();
                Line { a, b }
            })
            .collect()
    }
    fn shapes(&mut self) -> Shapes {
        let pgs = self.pgs();
        let exs = self.exs();
        let pls = self.pls();
        let lns = self.lns();
        let sf: Vec<Surface> = exs.iter().map(|e| Surface::new(SurfaceType::Internal, e.clone())).collect();
        Shapes { pg: pgs[0].clone(), ex: exs[0].clone(), pl: pls[0].clone(), ln: lns[0].clone(), pgs, exs, sf, pls, lns }
    }
}

struct Shapes {
    pg: Polygon,
    pgs: Vec<Polygon>,
    ex: ExPolygon,
    exs: Vec<ExPolygon>,
    sf: Vec<Surface>,
    pl: Polyline,
    pls: Vec<Polyline>,
    ln: Line,
    lns: Vec<Line>,
}

struct Params {
    jt: JoinType,
    ml: f64,
    et: EndType,
    delta: f32,
    delta_b: f32,
    od: f32,
    od2: f32,
    lw: f32,
    bbox: BoundingBox,
}

fn s_pts(out: &mut Vec<i64>, pts: &[Point]) {
    out.push(pts.len() as i64);
    for p in pts {
        out.push(p.x);
        out.push(p.y);
    }
}
fn s_pgs(v: &[Polygon]) -> Vec<i64> {
    let mut out = vec![v.len() as i64];
    for p in v {
        s_pts(&mut out, &p.points);
    }
    out
}
fn s_pls(v: &[Polyline]) -> Vec<i64> {
    let mut out = vec![v.len() as i64];
    for p in v {
        s_pts(&mut out, &p.points);
    }
    out
}
fn s_exs(v: &[ExPolygon]) -> Vec<i64> {
    let mut out = vec![v.len() as i64];
    for e in v {
        s_pts(&mut out, &e.contour.points);
        out.push(e.holes.len() as i64);
        for h in &e.holes {
            s_pts(&mut out, &h.points);
        }
    }
    out
}
fn s_lns(v: &[Line]) -> Vec<i64> {
    let mut out = vec![v.len() as i64];
    for l in v {
        out.extend([l.a.x, l.a.y, l.b.x, l.b.y]);
    }
    out
}
fn s_pg(p: &Polygon) -> Vec<i64> {
    let mut out = Vec::new();
    s_pts(&mut out, &p.points);
    out
}
fn s_points(p: &[Point]) -> Vec<i64> {
    let mut out = Vec::new();
    s_pts(&mut out, p);
    out
}
fn s_pt(t: &PolyTree) -> Vec<i64> {
    fn node(t: &PolyTree, n: usize, out: &mut Vec<i64>) {
        let nd = t.node(n);
        out.push(nd.contour.len() as i64);
        for q in &nd.contour {
            out.push(q.x as i64);
            out.push(q.y as i64);
        }
        out.push(nd.childs.len() as i64);
        for &c in &nd.childs {
            node(t, c, out);
        }
    }
    let mut out = Vec::new();
    node(t, POLY_TREE_ROOT, &mut out);
    out
}

// One call of gen.cpp: the id of the line, the result as the generator prints it. `None`: an id this test
// does not know.
#[allow(clippy::too_many_lines)]
fn call(id: &str, a: &Shapes, b: &Shapes, a_sp: &[&Surface], b_sp: &[&Surface], p: &Params) -> Option<Vec<i64>> {
    let (jt, ml, et, delta, delta_b, od, od2, lw) = (p.jt, p.ml, p.et, p.delta, p.delta_b, p.od, p.od2, p.lw);
    let bbox = p.bbox.clone();
    let _ = (b_sp, &bbox, et, lw, od2, delta_b);
    Some(match id {
        "diff:Pg:Pg:0" => s_pgs(&Diff::diff(&a.pg, &b.pg, ApplySafetyOffset::No)),
        "diff:Pg:Pg:1" => s_pgs(&Diff::diff(&a.pg, &b.pg, ApplySafetyOffset::Yes)),
        "diff:Pgs:Pgs:0" => s_pgs(&Diff::diff(a.pgs.as_slice(), b.pgs.as_slice(), ApplySafetyOffset::No)),
        "diff:Pgs:Pgs:1" => s_pgs(&Diff::diff(a.pgs.as_slice(), b.pgs.as_slice(), ApplySafetyOffset::Yes)),
        "diff:Pgs:Exs:0" => s_pgs(&Diff::diff(a.pgs.as_slice(), b.exs.as_slice(), ApplySafetyOffset::No)),
        "diff:Pgs:Exs:1" => s_pgs(&Diff::diff(a.pgs.as_slice(), b.exs.as_slice(), ApplySafetyOffset::Yes)),
        "diff:Exs:Pgs:0" => s_pgs(&Diff::diff(a.exs.as_slice(), b.pgs.as_slice(), ApplySafetyOffset::No)),
        "diff:Exs:Pgs:1" => s_pgs(&Diff::diff(a.exs.as_slice(), b.pgs.as_slice(), ApplySafetyOffset::Yes)),
        "diff:Exs:Exs:0" => s_pgs(&Diff::diff(a.exs.as_slice(), b.exs.as_slice(), ApplySafetyOffset::No)),
        "diff:Exs:Exs:1" => s_pgs(&Diff::diff(a.exs.as_slice(), b.exs.as_slice(), ApplySafetyOffset::Yes)),
        "diff:Sf:Pgs:0" => s_pgs(&Diff::diff(a.sf.as_slice(), b.pgs.as_slice(), ApplySafetyOffset::No)),
        "diff:Sf:Pgs:1" => s_pgs(&Diff::diff(a.sf.as_slice(), b.pgs.as_slice(), ApplySafetyOffset::Yes)),
        "intersection:Pg:Pg:0" => s_pgs(&Intersection::intersection(&a.pg, &b.pg, ApplySafetyOffset::No)),
        "intersection:Pg:Pg:1" => s_pgs(&Intersection::intersection(&a.pg, &b.pg, ApplySafetyOffset::Yes)),
        "intersection:Pgs:Ex:0" => s_pgs(&Intersection::intersection(a.pgs.as_slice(), &b.ex, ApplySafetyOffset::No)),
        "intersection:Pgs:Ex:1" => s_pgs(&Intersection::intersection(a.pgs.as_slice(), &b.ex, ApplySafetyOffset::Yes)),
        "intersection:Pgs:Pgs:0" => s_pgs(&Intersection::intersection(a.pgs.as_slice(), b.pgs.as_slice(), ApplySafetyOffset::No)),
        "intersection:Pgs:Pgs:1" => s_pgs(&Intersection::intersection(a.pgs.as_slice(), b.pgs.as_slice(), ApplySafetyOffset::Yes)),
        "intersection:Ex:Ex:0" => s_pgs(&Intersection::intersection(&a.ex, &b.ex, ApplySafetyOffset::No)),
        "intersection:Ex:Ex:1" => s_pgs(&Intersection::intersection(&a.ex, &b.ex, ApplySafetyOffset::Yes)),
        "intersection:Exs:Pgs:0" => s_pgs(&Intersection::intersection(a.exs.as_slice(), b.pgs.as_slice(), ApplySafetyOffset::No)),
        "intersection:Exs:Pgs:1" => s_pgs(&Intersection::intersection(a.exs.as_slice(), b.pgs.as_slice(), ApplySafetyOffset::Yes)),
        "intersection:Exs:Exs:0" => s_pgs(&Intersection::intersection(a.exs.as_slice(), b.exs.as_slice(), ApplySafetyOffset::No)),
        "intersection:Exs:Exs:1" => s_pgs(&Intersection::intersection(a.exs.as_slice(), b.exs.as_slice(), ApplySafetyOffset::Yes)),
        "intersection:Sf:Pgs:0" => s_pgs(&Intersection::intersection(a.sf.as_slice(), b.pgs.as_slice(), ApplySafetyOffset::No)),
        "intersection:Sf:Pgs:1" => s_pgs(&Intersection::intersection(a.sf.as_slice(), b.pgs.as_slice(), ApplySafetyOffset::Yes)),
        "intersection:Sf:Exs:0" => s_pgs(&Intersection::intersection(a.sf.as_slice(), b.exs.as_slice(), ApplySafetyOffset::No)),
        "intersection:Sf:Exs:1" => s_pgs(&Intersection::intersection(a.sf.as_slice(), b.exs.as_slice(), ApplySafetyOffset::Yes)),
        "intersection:Pgs:Pg:0" => s_pgs(&Intersection::intersection(a.pgs.as_slice(), &b.pg, ApplySafetyOffset::No)),
        "intersection:Pgs:Pg:1" => s_pgs(&Intersection::intersection(a.pgs.as_slice(), &b.pg, ApplySafetyOffset::Yes)),
        "diff_ex:Pgs:Pgs:0" => s_exs(&DiffEx::diff_ex(a.pgs.as_slice(), b.pgs.as_slice(), ApplySafetyOffset::No)),
        "diff_ex:Pgs:Pgs:1" => s_exs(&DiffEx::diff_ex(a.pgs.as_slice(), b.pgs.as_slice(), ApplySafetyOffset::Yes)),
        "diff_ex:Pgs:Sf:0" => s_exs(&DiffEx::diff_ex(a.pgs.as_slice(), b.sf.as_slice(), ApplySafetyOffset::No)),
        "diff_ex:Pgs:Sf:1" => s_exs(&DiffEx::diff_ex(a.pgs.as_slice(), b.sf.as_slice(), ApplySafetyOffset::Yes)),
        "diff_ex:Pg:Exs:0" => s_exs(&DiffEx::diff_ex(&a.pg, b.exs.as_slice(), ApplySafetyOffset::No)),
        "diff_ex:Pg:Exs:1" => s_exs(&DiffEx::diff_ex(&a.pg, b.exs.as_slice(), ApplySafetyOffset::Yes)),
        "diff_ex:Pgs:Exs:0" => s_exs(&DiffEx::diff_ex(a.pgs.as_slice(), b.exs.as_slice(), ApplySafetyOffset::No)),
        "diff_ex:Pgs:Exs:1" => s_exs(&DiffEx::diff_ex(a.pgs.as_slice(), b.exs.as_slice(), ApplySafetyOffset::Yes)),
        "diff_ex:Ex:Pg:0" => s_exs(&DiffEx::diff_ex(&a.ex, &b.pg, ApplySafetyOffset::No)),
        "diff_ex:Ex:Pg:1" => s_exs(&DiffEx::diff_ex(&a.ex, &b.pg, ApplySafetyOffset::Yes)),
        "diff_ex:Ex:Pgs:0" => s_exs(&DiffEx::diff_ex(&a.ex, b.pgs.as_slice(), ApplySafetyOffset::No)),
        "diff_ex:Ex:Pgs:1" => s_exs(&DiffEx::diff_ex(&a.ex, b.pgs.as_slice(), ApplySafetyOffset::Yes)),
        "diff_ex:Exs:Pgs:0" => s_exs(&DiffEx::diff_ex(a.exs.as_slice(), b.pgs.as_slice(), ApplySafetyOffset::No)),
        "diff_ex:Exs:Pgs:1" => s_exs(&DiffEx::diff_ex(a.exs.as_slice(), b.pgs.as_slice(), ApplySafetyOffset::Yes)),
        "diff_ex:Exs:Exs:0" => s_exs(&DiffEx::diff_ex(a.exs.as_slice(), b.exs.as_slice(), ApplySafetyOffset::No)),
        "diff_ex:Exs:Exs:1" => s_exs(&DiffEx::diff_ex(a.exs.as_slice(), b.exs.as_slice(), ApplySafetyOffset::Yes)),
        "diff_ex:Sf:Pgs:0" => s_exs(&DiffEx::diff_ex(a.sf.as_slice(), b.pgs.as_slice(), ApplySafetyOffset::No)),
        "diff_ex:Sf:Pgs:1" => s_exs(&DiffEx::diff_ex(a.sf.as_slice(), b.pgs.as_slice(), ApplySafetyOffset::Yes)),
        "diff_ex:Sf:Exs:0" => s_exs(&DiffEx::diff_ex(a.sf.as_slice(), b.exs.as_slice(), ApplySafetyOffset::No)),
        "diff_ex:Sf:Exs:1" => s_exs(&DiffEx::diff_ex(a.sf.as_slice(), b.exs.as_slice(), ApplySafetyOffset::Yes)),
        "diff_ex:Exs:Sf:0" => s_exs(&DiffEx::diff_ex(a.exs.as_slice(), b.sf.as_slice(), ApplySafetyOffset::No)),
        "diff_ex:Exs:Sf:1" => s_exs(&DiffEx::diff_ex(a.exs.as_slice(), b.sf.as_slice(), ApplySafetyOffset::Yes)),
        "diff_ex:Sf:Sf:0" => s_exs(&DiffEx::diff_ex(a.sf.as_slice(), b.sf.as_slice(), ApplySafetyOffset::No)),
        "diff_ex:Sf:Sf:1" => s_exs(&DiffEx::diff_ex(a.sf.as_slice(), b.sf.as_slice(), ApplySafetyOffset::Yes)),
        "diff_ex:Sp:Pgs:0" => s_exs(&DiffEx::diff_ex(a_sp, b.pgs.as_slice(), ApplySafetyOffset::No)),
        "diff_ex:Sp:Pgs:1" => s_exs(&DiffEx::diff_ex(a_sp, b.pgs.as_slice(), ApplySafetyOffset::Yes)),
        "diff_ex:Sp:Exs:0" => s_exs(&DiffEx::diff_ex(a_sp, b.exs.as_slice(), ApplySafetyOffset::No)),
        "diff_ex:Sp:Exs:1" => s_exs(&DiffEx::diff_ex(a_sp, b.exs.as_slice(), ApplySafetyOffset::Yes)),
        "diff_ex:Pg:Pgs:0" => s_exs(&DiffEx::diff_ex(&a.pg, b.pgs.as_slice(), ApplySafetyOffset::No)),
        "diff_ex:Pg:Pgs:1" => s_exs(&DiffEx::diff_ex(&a.pg, b.pgs.as_slice(), ApplySafetyOffset::Yes)),
        "diff_ex:Pg:Pg:0" => s_exs(&DiffEx::diff_ex(&a.pg, &b.pg, ApplySafetyOffset::No)),
        "diff_ex:Pg:Pg:1" => s_exs(&DiffEx::diff_ex(&a.pg, &b.pg, ApplySafetyOffset::Yes)),
        "diff_ex:Ex:Ex:0" => s_exs(&DiffEx::diff_ex(&a.ex, &b.ex, ApplySafetyOffset::No)),
        "diff_ex:Ex:Ex:1" => s_exs(&DiffEx::diff_ex(&a.ex, &b.ex, ApplySafetyOffset::Yes)),
        "diff_ex:Ex:Exs:0" => s_exs(&DiffEx::diff_ex(&a.ex, b.exs.as_slice(), ApplySafetyOffset::No)),
        "diff_ex:Ex:Exs:1" => s_exs(&DiffEx::diff_ex(&a.ex, b.exs.as_slice(), ApplySafetyOffset::Yes)),
        "diff_ex:Exs:Ex:0" => s_exs(&DiffEx::diff_ex(a.exs.as_slice(), &b.ex, ApplySafetyOffset::No)),
        "diff_ex:Exs:Ex:1" => s_exs(&DiffEx::diff_ex(a.exs.as_slice(), &b.ex, ApplySafetyOffset::Yes)),
        "intersection_ex:Pgs:Pgs:0" => s_exs(&IntersectionEx::intersection_ex(a.pgs.as_slice(), b.pgs.as_slice(), ApplySafetyOffset::No)),
        "intersection_ex:Pgs:Pgs:1" => s_exs(&IntersectionEx::intersection_ex(a.pgs.as_slice(), b.pgs.as_slice(), ApplySafetyOffset::Yes)),
        "intersection_ex:Ex:Pgs:0" => s_exs(&IntersectionEx::intersection_ex(&a.ex, b.pgs.as_slice(), ApplySafetyOffset::No)),
        "intersection_ex:Ex:Pgs:1" => s_exs(&IntersectionEx::intersection_ex(&a.ex, b.pgs.as_slice(), ApplySafetyOffset::Yes)),
        "intersection_ex:Ex:Ex:0" => s_exs(&IntersectionEx::intersection_ex(&a.ex, &b.ex, ApplySafetyOffset::No)),
        "intersection_ex:Ex:Ex:1" => s_exs(&IntersectionEx::intersection_ex(&a.ex, &b.ex, ApplySafetyOffset::Yes)),
        "intersection_ex:Pgs:Exs:0" => s_exs(&IntersectionEx::intersection_ex(a.pgs.as_slice(), b.exs.as_slice(), ApplySafetyOffset::No)),
        "intersection_ex:Pgs:Exs:1" => s_exs(&IntersectionEx::intersection_ex(a.pgs.as_slice(), b.exs.as_slice(), ApplySafetyOffset::Yes)),
        "intersection_ex:Exs:Pgs:0" => s_exs(&IntersectionEx::intersection_ex(a.exs.as_slice(), b.pgs.as_slice(), ApplySafetyOffset::No)),
        "intersection_ex:Exs:Pgs:1" => s_exs(&IntersectionEx::intersection_ex(a.exs.as_slice(), b.pgs.as_slice(), ApplySafetyOffset::Yes)),
        "intersection_ex:Exs:Ex:0" => s_exs(&IntersectionEx::intersection_ex(a.exs.as_slice(), &b.ex, ApplySafetyOffset::No)),
        "intersection_ex:Exs:Ex:1" => s_exs(&IntersectionEx::intersection_ex(a.exs.as_slice(), &b.ex, ApplySafetyOffset::Yes)),
        "intersection_ex:Ex:Exs:0" => s_exs(&IntersectionEx::intersection_ex(&a.ex, b.exs.as_slice(), ApplySafetyOffset::No)),
        "intersection_ex:Ex:Exs:1" => s_exs(&IntersectionEx::intersection_ex(&a.ex, b.exs.as_slice(), ApplySafetyOffset::Yes)),
        "intersection_ex:Exs:Exs:0" => s_exs(&IntersectionEx::intersection_ex(a.exs.as_slice(), b.exs.as_slice(), ApplySafetyOffset::No)),
        "intersection_ex:Exs:Exs:1" => s_exs(&IntersectionEx::intersection_ex(a.exs.as_slice(), b.exs.as_slice(), ApplySafetyOffset::Yes)),
        "intersection_ex:Sf:Pgs:0" => s_exs(&IntersectionEx::intersection_ex(a.sf.as_slice(), b.pgs.as_slice(), ApplySafetyOffset::No)),
        "intersection_ex:Sf:Pgs:1" => s_exs(&IntersectionEx::intersection_ex(a.sf.as_slice(), b.pgs.as_slice(), ApplySafetyOffset::Yes)),
        "intersection_ex:Sf:Exs:0" => s_exs(&IntersectionEx::intersection_ex(a.sf.as_slice(), b.exs.as_slice(), ApplySafetyOffset::No)),
        "intersection_ex:Sf:Exs:1" => s_exs(&IntersectionEx::intersection_ex(a.sf.as_slice(), b.exs.as_slice(), ApplySafetyOffset::Yes)),
        "intersection_ex:Sf:Sf:0" => s_exs(&IntersectionEx::intersection_ex(a.sf.as_slice(), b.sf.as_slice(), ApplySafetyOffset::No)),
        "intersection_ex:Sf:Sf:1" => s_exs(&IntersectionEx::intersection_ex(a.sf.as_slice(), b.sf.as_slice(), ApplySafetyOffset::Yes)),
        "intersection_ex:Sp:Exs:0" => s_exs(&IntersectionEx::intersection_ex(a_sp, b.exs.as_slice(), ApplySafetyOffset::No)),
        "intersection_ex:Sp:Exs:1" => s_exs(&IntersectionEx::intersection_ex(a_sp, b.exs.as_slice(), ApplySafetyOffset::Yes)),
        "xor_ex:Exs:Ex:0" => s_exs(&XorEx::xor_ex(a.exs.as_slice(), &b.ex, ApplySafetyOffset::No)),
        "xor_ex:Exs:Ex:1" => s_exs(&XorEx::xor_ex(a.exs.as_slice(), &b.ex, ApplySafetyOffset::Yes)),
        "xor_ex:Exs:Exs:0" => s_exs(&XorEx::xor_ex(a.exs.as_slice(), b.exs.as_slice(), ApplySafetyOffset::No)),
        "xor_ex:Exs:Exs:1" => s_exs(&XorEx::xor_ex(a.exs.as_slice(), b.exs.as_slice(), ApplySafetyOffset::Yes)),
        "diff_pl:Pl:Pgs" => s_pls(&DiffPl::diff_pl(&a.pl, b.pgs.as_slice())),
        "diff_pl:Pls:Pgs" => s_pls(&DiffPl::diff_pl(a.pls.as_slice(), b.pgs.as_slice())),
        "diff_pl:Pl:Ex" => s_pls(&DiffPl::diff_pl(&a.pl, &b.ex)),
        "diff_pl:Pl:Exs" => s_pls(&DiffPl::diff_pl(&a.pl, b.exs.as_slice())),
        "diff_pl:Pls:Ex" => s_pls(&DiffPl::diff_pl(a.pls.as_slice(), &b.ex)),
        "diff_pl:Pls:Exs" => s_pls(&DiffPl::diff_pl(a.pls.as_slice(), b.exs.as_slice())),
        "diff_pl:Pgs:Pgs" => s_pls(&DiffPl::diff_pl(a.pgs.as_slice(), b.pgs.as_slice())),
        "intersection_pl:Pls:Pg" => s_pls(&IntersectionPl::intersection_pl(a.pls.as_slice(), &b.pg)),
        "intersection_pl:Pl:Ex" => s_pls(&IntersectionPl::intersection_pl(&a.pl, &b.ex)),
        "intersection_pl:Pls:Ex" => s_pls(&IntersectionPl::intersection_pl(a.pls.as_slice(), &b.ex)),
        "intersection_pl:Pl:Pgs" => s_pls(&IntersectionPl::intersection_pl(&a.pl, b.pgs.as_slice())),
        "intersection_pl:Pl:Exs" => s_pls(&IntersectionPl::intersection_pl(&a.pl, b.exs.as_slice())),
        "intersection_pl:Pls:Pgs" => s_pls(&IntersectionPl::intersection_pl(a.pls.as_slice(), b.pgs.as_slice())),
        "intersection_pl:Pls:Exs" => s_pls(&IntersectionPl::intersection_pl(a.pls.as_slice(), b.exs.as_slice())),
        "intersection_pl:Pgs:Pgs" => s_pls(&IntersectionPl::intersection_pl(a.pgs.as_slice(), b.pgs.as_slice())),
        "offset:Offset:Pg" => s_pgs(&Offset::offset(&a.pg, od, jt, ml)),
        "offset:Offset:Pgs" => s_pgs(&Offset::offset(a.pgs.as_slice(), od, jt, ml)),
        "offset:Offset:Ex" => s_pgs(&Offset::offset(&a.ex, od, jt, ml)),
        "offset:Offset:Exs" => s_pgs(&Offset::offset(a.exs.as_slice(), od, jt, ml)),
        "offset:Offset:Sf" => s_pgs(&Offset::offset(a.sf.as_slice(), od, jt, ml)),
        "offset:Offset:Sp" => s_pgs(&Offset::offset(a_sp, od, jt, ml)),
        "offset:OffsetPolyline:Pl" => s_pgs(&OffsetPolyline::offset(&a.pl, delta, jt, ml, et)),
        "offset:OffsetPolyline:Pls" => s_pgs(&OffsetPolyline::offset(a.pls.as_slice(), delta, jt, ml, et)),
        "offset_ex:OffsetEx:Pgs" => s_exs(&OffsetEx::offset_ex(a.pgs.as_slice(), od, jt, ml)),
        "offset_ex:OffsetEx:Ex" => s_exs(&OffsetEx::offset_ex(&a.ex, od, jt, ml)),
        "offset_ex:OffsetEx:Exs" => s_exs(&OffsetEx::offset_ex(a.exs.as_slice(), od, jt, ml)),
        "offset_ex:OffsetEx:Sf" => s_exs(&OffsetEx::offset_ex(a.sf.as_slice(), od, jt, ml)),
        "offset_ex:OffsetEx:Sp" => s_exs(&OffsetEx::offset_ex(a_sp, od, jt, ml)),
        "offset_ex:OffsetEx:Pg" => s_exs(&OffsetEx::offset_ex(&a.pg, od, jt, ml)),
        "contour_to_polygons:ContourToPolygons:Pg" => s_pgs(&ContourToPolygons::contour_to_polygons(&a.pg, lw, jt, ml)),
        "contour_to_polygons:ContourToPolygons:Pgs" => s_pgs(&ContourToPolygons::contour_to_polygons(a.pgs.as_slice(), lw, jt, ml)),
        "expand:Expand:Pg" => s_pgs(&Expand::expand(&a.pg, delta, jt, ml)),
        "expand:Expand:Pgs" => s_pgs(&Expand::expand(a.pgs.as_slice(), delta, jt, ml)),
        "shrink_ex:ShrinkEx:Pgs" => s_exs(&ShrinkEx::shrink_ex(a.pgs.as_slice(), delta, jt, ml)),
        "shrink_ex:ShrinkEx:Exs" => s_exs(&ShrinkEx::shrink_ex(a.exs.as_slice(), delta, jt, ml)),
        "offset2_ex:Offset2Ex:Exs" => s_exs(&Offset2Ex::offset2_ex(a.exs.as_slice(), od, od2, jt, ml)),
        "offset2_ex:Offset2Ex:Sf" => s_exs(&Offset2Ex::offset2_ex(a.sf.as_slice(), od, od2, jt, ml)),
        "closing:Pgs:2" => s_pgs(&Closing::closing(a.pgs.as_slice(), (delta, delta_b), jt, ml)),
        "closing:Pgs:1" => s_pgs(&Closing::closing(a.pgs.as_slice(), delta, jt, ml)),
        "closing_ex:Pgs:2" => s_exs(&ClosingEx::closing_ex(a.pgs.as_slice(), (delta, delta_b), jt, ml)),
        "closing_ex:Sf:2" => s_exs(&ClosingEx::closing_ex(a.sf.as_slice(), (delta, delta_b), jt, ml)),
        "closing_ex:Pgs:1" => s_exs(&ClosingEx::closing_ex(a.pgs.as_slice(), delta, jt, ml)),
        "closing_ex:Exs:1" => s_exs(&ClosingEx::closing_ex(a.exs.as_slice(), delta, jt, ml)),
        "closing_ex:Sf:1" => s_exs(&ClosingEx::closing_ex(a.sf.as_slice(), delta, jt, ml)),
        "opening:Pgs:2" => s_pgs(&Opening::opening(a.pgs.as_slice(), (delta, delta_b), jt, ml)),
        "opening:Exs:2" => s_pgs(&Opening::opening(a.exs.as_slice(), (delta, delta_b), jt, ml)),
        "opening:Sf:2" => s_pgs(&Opening::opening(a.sf.as_slice(), (delta, delta_b), jt, ml)),
        "opening:Pgs:1" => s_pgs(&Opening::opening(a.pgs.as_slice(), delta, jt, ml)),
        "opening:Exs:1" => s_pgs(&Opening::opening(a.exs.as_slice(), delta, jt, ml)),
        "opening:Sf:1" => s_pgs(&Opening::opening(a.sf.as_slice(), delta, jt, ml)),
        "opening_ex:OpeningEx:Exs" => s_exs(&OpeningEx::opening_ex(a.exs.as_slice(), delta, jt, ml)),
        "opening_ex:OpeningEx:Sf" => s_exs(&OpeningEx::opening_ex(a.sf.as_slice(), delta, jt, ml)),
        "union_safety_offset:Pgs" => s_pgs(&UnionSafetyOffset::union_safety_offset(a.pgs.as_slice())),
        "union_safety_offset_ex:Pgs" => s_exs(&UnionSafetyOffsetEx::union_safety_offset_ex(a.pgs.as_slice())),
        "union_safety_offset:Exs" => s_pgs(&UnionSafetyOffset::union_safety_offset(a.exs.as_slice())),
        "union_safety_offset_ex:Exs" => s_exs(&UnionSafetyOffsetEx::union_safety_offset_ex(a.exs.as_slice())),
        "expand_ex" => s_exs(&cpp::expand_ex(a.pgs.as_slice(), delta, jt, ml)),
        "shrink" => s_pgs(&cpp::shrink(a.pgs.as_slice(), delta, jt, ml)),
        "offset2" => s_pgs(&cpp::offset2(a.exs.as_slice(), od, od2, jt, ml)),
        "diff_clipped:Pgs:Pgs:0" => s_pgs(&DiffClipped::diff_clipped(a.pgs.as_slice(), b.pgs.as_slice(), ApplySafetyOffset::No)),
        "diff_clipped:Exs:Pgs:0" => s_exs(&DiffClipped::diff_clipped(a.exs.as_slice(), b.pgs.as_slice(), ApplySafetyOffset::No)),
        "diff_clipped:Exs:Exs:0" => s_exs(&DiffClipped::diff_clipped(a.exs.as_slice(), b.exs.as_slice(), ApplySafetyOffset::No)),
        "intersection_clipped:0" => s_pgs(&cpp::intersection_clipped(a.pgs.as_slice(), b.pgs.as_slice(), ApplySafetyOffset::No)),
        "diff_clipped:Pgs:Pgs:1" => s_pgs(&DiffClipped::diff_clipped(a.pgs.as_slice(), b.pgs.as_slice(), ApplySafetyOffset::Yes)),
        "diff_clipped:Exs:Pgs:1" => s_exs(&DiffClipped::diff_clipped(a.exs.as_slice(), b.pgs.as_slice(), ApplySafetyOffset::Yes)),
        "diff_clipped:Exs:Exs:1" => s_exs(&DiffClipped::diff_clipped(a.exs.as_slice(), b.exs.as_slice(), ApplySafetyOffset::Yes)),
        "intersection_clipped:1" => s_pgs(&cpp::intersection_clipped(a.pgs.as_slice(), b.pgs.as_slice(), ApplySafetyOffset::Yes)),
        "union_:Pgs" => s_pgs(&Union_::union_(a.pgs.as_slice(), ())),
        "union_:Exs" => s_pgs(&Union_::union_(a.exs.as_slice(), ())),
        "union_:Pgs:Pgs" => s_pgs(&Union_::union_(a.pgs.as_slice(), b.pgs.as_slice())),
        "union_:Pgs:Ex" => s_pgs(&Union_::union_(a.pgs.as_slice(), &b.ex)),
        "union_ex:Exs" => s_exs(&UnionEx::union_ex(a.exs.as_slice(), ())),
        "union_ex:Exs:Pgs" => s_exs(&UnionEx::union_ex(a.exs.as_slice(), b.pgs.as_slice())),
        "union_ex:Sf" => s_exs(&UnionEx::union_ex(a.sf.as_slice(), ())),
        "union_ex:Exs:Exs:0" => s_exs(&UnionEx::union_ex(a.exs.as_slice(), (b.exs.as_slice(), false))),
        "union_ex:Exs:Exs:1" => s_exs(&UnionEx::union_ex(a.exs.as_slice(), (b.exs.as_slice(), true))),
        "_clipper_ln:i" => s_lns(&cpp::_clipper_ln(ClipType::ctIntersection, a.lns.as_slice(), b.pgs.as_slice())),
        "_clipper_ln:d" => s_lns(&cpp::_clipper_ln(ClipType::ctDifference, a.lns.as_slice(), b.pgs.as_slice())),
        "diff_ln" => s_lns(&cpp::diff_ln(a.lns.as_slice(), b.pgs.as_slice())),
        "intersection_ln:Lns" => s_lns(&IntersectionLn::intersection_ln(a.lns.as_slice(), b.pgs.as_slice())),
        "intersection_ln:Ln" => s_lns(&IntersectionLn::intersection_ln(&a.ln, b.pgs.as_slice())),
        "union_pt:Pgs" => s_pt(&UnionPt::union_pt(a.pgs.as_slice())),
        "union_pt:Exs" => s_pt(&UnionPt::union_pt(a.exs.as_slice())),
        "union_pt_chained_outside_in" => s_pgs(&cpp::union_pt_chained_outside_in(a.pgs.as_slice())),
        "simplify_polygons:0" => s_pgs(&cpp::simplify_polygons(a.pgs.as_slice(), false)),
        "simplify_polygons:1" => s_pgs(&cpp::simplify_polygons(a.pgs.as_slice(), true)),
        "simplify_polygons_ex:0" => s_exs(&cpp::simplify_polygons_ex(a.pgs.as_slice(), false)),
        "simplify_polygons_ex:1" => s_exs(&cpp::simplify_polygons_ex(a.pgs.as_slice(), true)),
        "top_level_islands" => s_pgs(&cpp::top_level_islands(a.pgs.as_slice())),
        "ccp:Points" => s_points(&ClipClipperPolygon::clip_clipper_polygon_with_subject_bbox(a.pg.points.as_slice(), &bbox)),
        "ccp:Polygon:0" => s_pg(&cpp::clip_clipper_polygon_with_subject_bbox(&a.pg, &bbox, false)),
        "ccp:Polygon:1" => s_pg(&cpp::clip_clipper_polygon_with_subject_bbox(&a.pg, &bbox, true)),
        "ccps:Pgs" => s_pgs(&ClipClipperPolygons::clip_clipper_polygons_with_subject_bbox(a.pgs.as_slice(), &bbox, false)),
        "union_:Pgs:pftEvenOdd" => s_pgs(&Union_::union_(a.pgs.as_slice(), PolyFillType::pftEvenOdd)),
        "union_ex:Pgs:pftEvenOdd" => s_exs(&UnionEx::union_ex(a.pgs.as_slice(), PolyFillType::pftEvenOdd)),
        "union_:Pgs:pftNonZero" => s_pgs(&Union_::union_(a.pgs.as_slice(), PolyFillType::pftNonZero)),
        "union_ex:Pgs:pftNonZero" => s_exs(&UnionEx::union_ex(a.pgs.as_slice(), PolyFillType::pftNonZero)),
        "union_:Pgs:pftPositive" => s_pgs(&Union_::union_(a.pgs.as_slice(), PolyFillType::pftPositive)),
        "union_ex:Pgs:pftPositive" => s_exs(&UnionEx::union_ex(a.pgs.as_slice(), PolyFillType::pftPositive)),
        "union_:Pgs:pftNegative" => s_pgs(&Union_::union_(a.pgs.as_slice(), PolyFillType::pftNegative)),
        "union_ex:Pgs:pftNegative" => s_exs(&UnionEx::union_ex(a.pgs.as_slice(), PolyFillType::pftNegative)),
        "ccps:Ex:0" => s_pgs(&ClipClipperPolygons::clip_clipper_polygons_with_subject_bbox(&a.ex, &bbox, false)),
        "ccps:Exs:0" => s_pgs(&ClipClipperPolygons::clip_clipper_polygons_with_subject_bbox(a.exs.as_slice(), &bbox, false)),
        "ccps:Ex:1" => s_pgs(&ClipClipperPolygons::clip_clipper_polygons_with_subject_bbox(&a.ex, &bbox, true)),
        "ccps:Exs:1" => s_pgs(&ClipClipperPolygons::clip_clipper_polygons_with_subject_bbox(a.exs.as_slice(), &bbox, true)),
        _ => return None,
    })
}

#[test]
#[ignore = "no vectors yet: tests/data/clipper_utils/gen.cpp does not link, it calls two diff_ex overloads that ClipperUtils.hpp declares and ClipperUtils.cpp never defines"]
fn clipper_utils_cpp_native_parity() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/clipper_utils/vectors.txt.gz");
    let path = std::env::var("CLIPPER_UTILS_CPP_VECTORS").unwrap_or_else(|_| path.to_string());
    let mut text = String::new();
    if path.ends_with(".gz") {
        let file = std::fs::File::open(&path).expect("vectors.txt.gz");
        flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    } else {
        text = std::fs::read_to_string(&path).expect("vectors.txt");
    }
    let mut case: Option<(Shapes, Shapes, Params)> = None;
    let mut by_fn: std::collections::BTreeMap<String, (usize, usize)> = std::collections::BTreeMap::new();
    let (mut cases, mut calls, mut bad, mut shown) = (0usize, 0usize, 0usize, 0usize);
    for (line_idx, line) in text.lines().enumerate() {
        let line_no = line_idx + 1;
        let mut t = Tok { it: line.split(' '), line_no };
        let id = t.s();
        if id == "in" {
            let jt = match t.i() {
                0 => JoinType::jtSquare,
                1 => JoinType::jtRound,
                _ => JoinType::jtMiter,
            };
            let ml = t.i() as f64 / 100.;
            let et = match t.i() {
                2 => EndType::etOpenButt,
                3 => EndType::etOpenSquare,
                4 => EndType::etOpenRound,
                e => panic!("end type {e}"),
            };
            let delta = t.i() as f32 / 2.;
            let delta_b = t.i() as f32 / 2.;
            let sgn = t.i() as f32;
            let a = t.shapes();
            let b = t.shapes();
            let bbox = BoundingBox::from_points_minmax(t.pt(), t.pt());
            case = Some((a, b, Params { jt, ml, et, delta, delta_b, od: sgn * delta, od2: -sgn * delta_b, lw: delta + 2., bbox }));
            cases += 1;
            continue;
        }
        let (a, b, p) = case.as_ref().expect("a case before its calls");
        let a_sp: Vec<&Surface> = a.sf.iter().collect();
        let b_sp: Vec<&Surface> = b.sf.iter().collect();
        let expected: Vec<i64> = t.it.map(|v| v.parse().expect("integer")).collect();
        let got = call(id, a, b, &a_sp, &b_sp, p).unwrap_or_else(|| panic!("line {line_no}: unknown call {id}"));
        calls += 1;
        let name = id.split(':').next().unwrap().to_string();
        let e = by_fn.entry(name).or_default();
        e.0 += 1;
        if got != expected {
            e.1 += 1;
            bad += 1;
            if shown < 10 {
                shown += 1;
                let cut = |v: &[i64]| {
                    let s = format!("{v:?}");
                    if s.len() > 500 {
                        format!("{}...", &s[..500])
                    } else {
                        s
                    }
                };
                eprintln!("MISMATCH line {line_no} (case {cases}) {id}\n  rust {}\n  pin  {}", cut(&got), cut(&expected));
            }
        }
    }
    for (name, (n, m)) in &by_fn {
        eprintln!("{name}: {n} calls, {m} mismatches");
    }
    eprintln!("{cases} cases, {calls} calls, {bad} mismatches");
    assert_eq!(bad, 0, "results that differ from the pin");
    assert_eq!((cases, calls), (48, 48 * 201), "cases and calls of the vectors file");
}
