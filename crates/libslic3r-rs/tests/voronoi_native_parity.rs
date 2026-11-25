//! Geometry::VoronoiDiagram (Geometry/Voronoi.hpp, Voronoi.cpp; the port in `geometry/voronoi.rs`)
//! against the pin, compiled with the engine's flags, on the inputs of `tests/data/voronoi/gen.cpp`
//! (gzip -9n): 400 segment sets (star polygons, slivers, grids of squares, and polygons with crossing
//! segments, which the issue detection flags and the repair by rotation tries and fails on), 100 point
//! sets and 100 point-and-segment sets. Per diagram the issue type, state, `is_valid`, `num_vertices`,
//! `num_edges`, `num_cells`, every vertex (x, y bit for bit, colour), every edge (vertex0, twin, next,
//! cell, primary, linear) and every cell (source index and category) must be equal (a NaN vertex
//! coordinate, which only the crossing-segment inputs give, as a NaN of any sign or payload); then
//! `encode_input_segment_endpoint`, `decode_input_segment_endpoint` and `segment_traits<Segment>::get`.

use slicer::geometry::voronoi_diagram::boost_polygon::{
    decode_input_segment_endpoint, encode_input_segment_endpoint, ConstructVoronoiPoints,
    ConstructVoronoiPointsSegments, SegmentTraits, VoronoiDiagram,
};
use slicer::geometry::{Line, Point};
use std::io::Read;

struct Tok<'a>(std::str::SplitWhitespace<'a>);

impl<'a> Tok<'a> {
    fn s(&mut self) -> &'a str {
        self.0.next().expect("token")
    }
    fn i(&mut self) -> i64 {
        self.s().parse().expect("integer")
    }
    fn u(&mut self) -> usize {
        self.s().parse().expect("count")
    }
    fn d(&mut self) -> f64 {
        f64::from_bits(u64::from_str_radix(self.s(), 16).expect("double"))
    }
    fn points(&mut self) -> Vec<Point> {
        let n = self.u();
        (0..n).map(|_| Point::new(self.i(), self.i())).collect()
    }
    fn lines(&mut self) -> Vec<Line> {
        let n = self.u();
        (0..n).map(|_| Line::new(Point::new(self.i(), self.i()), Point::new(self.i(), self.i()))).collect()
    }
}

fn opt(v: Option<usize>) -> i64 {
    v.map_or(-1, |x| x as i64)
}

/// One `d` line of the generator against the port's diagram; returns the first difference.
fn check(vd: &VoronoiDiagram, line: &str) -> Option<String> {
    let mut t = Tok(line.split_whitespace());
    assert_eq!(t.s(), "d");
    let (issue, state, valid) = (t.i(), t.i(), t.i());
    let (nv, ne, nc) = (t.u(), t.u(), t.u());
    if vd.get_issue_type() as i64 != issue || vd.get_state() as i64 != state || i64::from(vd.is_valid()) != valid {
        return Some(format!(
            "issue/state/valid: port {:?}/{:?}/{} pin {issue}/{state}/{valid}",
            vd.get_issue_type(),
            vd.get_state(),
            vd.is_valid()
        ));
    }
    if (vd.num_vertices(), vd.num_edges(), vd.num_cells()) != (nv, ne, nc) {
        return Some(format!(
            "counts: port {}/{}/{} pin {nv}/{ne}/{nc}",
            vd.num_vertices(),
            vd.num_edges(),
            vd.num_cells()
        ));
    }
    let d = vd.diagram();
    assert_eq!(t.s(), "V");
    for (i, v) in d.vertices().iter().enumerate() {
        let (x, y, c) = (t.d(), t.d(), t.u());
        // Bit for bit; a NaN (vertices of the crossing-segment inputs boost does not expect) only as a NaN:
        // its sign and payload come from the arithmetic that produced it.
        let same = |a: f64, b: f64| a.to_bits() == b.to_bits() || (a.is_nan() && b.is_nan());
        if !same(v.x(), x) || !same(v.y(), y) || v.color() != c {
            return Some(format!("vertex {i}: port ({:e}, {:e}) c{} pin ({x:e}, {y:e}) c{c}", v.x(), v.y(), v.color()));
        }
    }
    assert_eq!(t.s(), "E");
    for (i, e) in d.edges().iter().enumerate() {
        let want = [t.i(), t.i(), t.i(), t.i(), t.i(), t.i()];
        let got = [
            opt(e.vertex0()),
            opt(e.twin()),
            opt(e.next()),
            opt(e.cell()),
            i64::from(e.is_primary()),
            i64::from(e.is_linear()),
        ];
        if got != want {
            return Some(format!("edge {i}: port {got:?} pin {want:?}"));
        }
    }
    assert_eq!(t.s(), "C");
    for (i, c) in d.cells().iter().enumerate() {
        let (idx, cat) = (t.u(), t.u());
        if (c.source_index(), c.source_category()) != (idx, cat) {
            return Some(format!("cell {i}: port {}/{} pin {idx}/{cat}", c.source_index(), c.source_category()));
        }
    }
    None
}

#[test]
fn voronoi_diagram_matches_the_pin() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/voronoi/vectors.txt.gz");
    let mut text = String::new();
    flate2::read::GzDecoder::new(std::fs::File::open(path).unwrap()).read_to_string(&mut text).unwrap();
    let mut lines = text.lines();
    let (mut n_seg, mut n_pts, mut n_ps, mut n_enc) = (0usize, 0usize, 0usize, 0usize);
    let (mut repaired_tries, mut bad) = (0usize, Vec::<String>::new());
    let mut segs: Vec<Line> = Vec::new();
    while let Some(line) = lines.next() {
        let mut t = Tok(line.split_whitespace());
        match t.s() {
            "s" => {
                segs = t.lines();
                let mut vd = VoronoiDiagram::new();
                vd.construct_voronoi(&segs, true);
                n_seg += 1;
                if vd.get_state() as i64 != 0 {
                    repaired_tries += 1;
                }
                if let Some(e) = check(&vd, lines.next().unwrap()) {
                    bad.push(format!("segments case {n_seg}: {e}"));
                }
            }
            "p" => {
                let pts = t.points();
                let mut vd = VoronoiDiagram::new();
                ConstructVoronoiPoints::construct_voronoi(&mut vd, &pts);
                n_pts += 1;
                if let Some(e) = check(&vd, lines.next().unwrap()) {
                    bad.push(format!("points case {n_pts}: {e}"));
                }
            }
            "ps" => {
                let pts = t.points();
                let ls = t.lines();
                let mut vd = VoronoiDiagram::new();
                ConstructVoronoiPointsSegments::construct_voronoi(&mut vd, &pts, &ls);
                n_ps += 1;
                if let Some(e) = check(&vd, lines.next().unwrap()) {
                    bad.push(format!("points+segments case {n_ps}: {e}"));
                }
            }
            "e" => {
                let (i, high, color) = (t.u(), t.i() == 1, t.u());
                let (dx, dy, gx, gy) = (t.i(), t.i(), t.i(), t.i());
                n_enc += 1;
                let c = encode_input_segment_endpoint(i, high);
                let dec = decode_input_segment_endpoint(color, &segs);
                let g = SegmentTraits::get(&segs[i], high);
                if c != color || (dec.x, dec.y) != (dx, dy) || (g.x, g.y) != (gx, gy) {
                    bad.push(format!("endpoint coding {n_enc}: {line}"));
                }
            }
            other => panic!("unknown record {other}"),
        }
    }
    eprintln!(
        "voronoi: {n_seg} segment diagrams ({repaired_tries} with issues and a repair try), {n_pts} point diagrams, \
         {n_ps} point+segment diagrams, {n_enc} endpoint codings; {} mismatches; first: {:?}",
        bad.len(),
        bad.first()
    );
    assert_eq!((n_seg, n_pts, n_ps, n_enc), (400, 100, 100, 1600));
    assert!(repaired_tries > 0);
    assert!(bad.is_empty(), "{} mismatches, first: {:?}", bad.len(), bad.first());
}
