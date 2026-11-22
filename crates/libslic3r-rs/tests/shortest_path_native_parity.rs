//! ShortestPath.cpp of libslic3r against vectors of the oracle (tests/data/shortest_path/gen.cpp): the order
//! and the reversal flags the chaining functions return, on inputs made to hit equal distances: shared end
//! points, duplicates, rasters at one spacing, closed segments, single items and empty inputs.
//!
//! The format of a line is in the header of gen.cpp.

use std::io::Read;

use slicer::extrusion_entity::{
    ExtrusionEntityCollection, ExtrusionEntityType, ExtrusionLoop, ExtrusionLoopRole, ExtrusionMultiPath, ExtrusionPath, ExtrusionRole,
};
use slicer::geometry::{ExPolygon, Line, Point, Polygon, Polyline};
use slicer::shortest_path::{
    chain_and_reorder_extrusion_entities, chain_and_reorder_extrusion_paths, chain_expolygons, chain_extrusion_entities, chain_extrusion_paths, chain_lines,
    chain_clipper_polynodes, chain_path_items, chain_points, chain_polylines, chain_print_object_instances, chain_segments_greedy, chain_segments_greedy2, chain_segments_greedy_constrained_reversals,
    chain_segments_greedy_constrained_reversals2, do_crossover, improve_ordering_by_two_exchanges_with_segment_flipping, minimum_crossover_cost,
    reorder_by_shortest_traverse, reorder_by_two_exchanges_with_segment_flipping, reorder_extrusion_entities, ChainPrintObjectInstances, ConnectionCost,
    FlipEdge,
};

fn hex_f64(s: &str) -> f64 {
    let (neg, s) = match s.strip_prefix('-') {
        Some(r) => (true, r),
        None => (false, s),
    };
    let s = s.strip_prefix("0x").expect("hex float");
    let (mant, exp) = s.split_once('p').expect("hex float exponent");
    let (ip, fp) = mant.split_once('.').unwrap_or((mant, ""));
    let mut m = u64::from_str_radix(ip, 16).unwrap();
    let mut e: i32 = exp.parse().unwrap();
    for c in fp.chars() {
        m = m * 16 + c.to_digit(16).unwrap() as u64;
        e -= 4;
    }
    let v = (m as f64) * 2f64.powi(e);
    if neg {
        -v
    } else {
        v
    }
}

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
    fn b(&mut self) -> bool {
        self.i() != 0
    }
    fn f(&mut self) -> f64 {
        hex_f64(self.s())
    }
    fn pt(&mut self) -> Point {
        let x = self.i();
        let y = self.i();
        Point { x, y }
    }
    fn v2(&mut self) -> [f64; 2] {
        let x = self.f();
        let y = self.f();
        [x, y]
    }
    fn polyline(&mut self) -> Polyline {
        let m = self.u();
        let mut pl = Polyline::new();
        for _ in 0..m {
            pl.points.push(self.pt());
        }
        pl
    }
    fn polylines(&mut self, n: usize) -> Vec<Polyline> {
        (0..n).map(|_| self.polyline()).collect()
    }
    fn chain(&mut self) -> Vec<(usize, bool)> {
        let n = self.u();
        (0..n)
            .map(|_| {
                let idx = self.u();
                let rev = self.b();
                (idx, rev)
            })
            .collect()
    }
    fn path(&mut self) -> ExtrusionPath {
        let mut p = ExtrusionPath::new(ExtrusionRole::InternalInfill);
        p.mm3_per_mm = 1.;
        p.polyline = self.polyline();
        p
    }
    fn entity(&mut self) -> ExtrusionEntityType {
        match self.i() {
            0 => {
                let cr = self.b();
                let mut p = self.path();
                p.can_reverse = cr;
                ExtrusionEntityType::Path(p)
            }
            1 => {
                let np = self.u();
                let paths = (0..np).map(|_| self.path()).collect();
                ExtrusionEntityType::Loop(ExtrusionLoop::new(paths, ExtrusionLoopRole::default()))
            }
            2 => {
                let cr = self.b();
                let np = self.u();
                let mut m = ExtrusionMultiPath::new();
                m.paths = (0..np).map(|_| self.path()).collect();
                m.can_reverse = cr;
                ExtrusionEntityType::MultiPath(m)
            }
            3 => {
                let no_sort = self.b();
                let is_reverse = self.b();
                let k = self.u();
                let mut c = ExtrusionEntityCollection::new();
                c.no_sort = no_sort;
                // gen.cpp prints can_reverse() of a collection that may be sorted and 0 for a no_sort one, whose
                // is_reverse cannot be read and does not matter (ExtrusionEntityCollection.hpp:63-69).
                c.is_reverse = is_reverse;
                for _ in 0..k {
                    c.entities.push(self.entity());
                }
                ExtrusionEntityType::Collection(Box::new(c))
            }
            k => panic!("entity kind {k}"),
        }
    }
    fn entities(&mut self, n: usize) -> Vec<ExtrusionEntityType> {
        (0..n).map(|_| self.entity()).collect()
    }
    fn edges(&mut self, n: usize) -> Vec<FlipEdge> {
        (0..n)
            .map(|i| {
                let p1 = self.v2();
                let p2 = self.v2();
                FlipEdge::new(p1, p2, i)
            })
            .collect()
    }
    fn edges_out(&mut self, n: usize) -> Vec<(usize, [u64; 4])> {
        (0..n)
            .map(|_| {
                let src = self.u();
                let p1 = self.v2();
                let p2 = self.v2();
                (src, [p1[0].to_bits(), p1[1].to_bits(), p2[0].to_bits(), p2[1].to_bits()])
            })
            .collect()
    }
    fn done(&mut self) {
        assert!(self.it.next().is_none(), "tokens left on the line");
    }
}

fn edges_bits(edges: &[FlipEdge]) -> Vec<(usize, [u64; 4])> {
    edges.iter().map(|e| (e.source_index, [e.p1[0].to_bits(), e.p1[1].to_bits(), e.p2[0].to_bits(), e.p2[1].to_bits()])).collect()
}

fn points_of(pls: &[Polyline]) -> Vec<Vec<Point>> {
    pls.iter().map(|p| p.points.clone()).collect()
}

// The line gen.cpp prints for an entity (put_entity).
fn put_entity(out: &mut Vec<i64>, e: &ExtrusionEntityType) {
    fn put_polyline(out: &mut Vec<i64>, pl: &Polyline) {
        out.push(pl.points.len() as i64);
        for p in &pl.points {
            out.push(p.x);
            out.push(p.y);
        }
    }
    match e {
        ExtrusionEntityType::Collection(c) => {
            out.extend([3, c.no_sort as i64, if c.no_sort { 0 } else { c.can_reverse() as i64 }, c.entities.len() as i64]);
            for e in &c.entities {
                put_entity(out, e);
            }
        }
        ExtrusionEntityType::Loop(l) => {
            out.extend([1, l.paths.len() as i64]);
            for p in &l.paths {
                put_polyline(out, &p.polyline);
            }
        }
        ExtrusionEntityType::MultiPath(m) => {
            out.extend([2, m.can_reverse() as i64, m.paths.len() as i64]);
            for p in &m.paths {
                put_polyline(out, &p.polyline);
            }
        }
        ExtrusionEntityType::Path(p) => {
            out.extend([0, p.can_reverse() as i64]);
            put_polyline(out, &p.polyline);
        }
    }
}

fn put_entities(ents: &[ExtrusionEntityType]) -> Vec<i64> {
    let mut out = Vec::new();
    for e in ents {
        put_entity(&mut out, e);
    }
    out
}

#[derive(Default)]
struct Tally {
    kinds: std::collections::BTreeMap<String, (usize, usize)>,
    shown: usize,
}

impl Tally {
    fn check<T: PartialEq + std::fmt::Debug>(&mut self, kind: &str, line_no: usize, what: &str, got: &T, expected: &T) {
        let e = self.kinds.entry(kind.to_string()).or_default();
        e.0 += 1;
        if got != expected {
            e.1 += 1;
            if self.shown < 12 {
                self.shown += 1;
                let (g, x) = (format!("{got:?}"), format!("{expected:?}"));
                let cut = |s: &str| if s.len() > 600 { format!("{}...", &s[..600]) } else { s.to_string() };
                eprintln!("MISMATCH line {line_no} {kind} {what}\n  rust {}\n  pin  {}", cut(&g), cut(&x));
            }
        }
    }
}

#[test]
fn shortest_path_native_parity() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/shortest_path/vectors.txt.gz");
    let path = std::env::var("SHORTEST_PATH_VECTORS").unwrap_or_else(|_| path.to_string());
    let mut text = String::new();
    if path.ends_with(".gz") {
        let file = std::fs::File::open(&path).expect("vectors.txt.gz");
        flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    } else {
        text = std::fs::read_to_string(&path).expect("vectors.txt");
    }
    let mut tally = Tally::default();
    let mut lines = 0usize;
    for (line_idx, line) in text.lines().enumerate() {
        let line_no = line_idx + 1;
        lines += 1;
        let mut t = Tok { it: line.split(' '), line_no };
        let kind = t.s();
        match kind {
            "traits" => {
                // the pin's coord_t is an int
                assert_eq!(line, "traits 4");
            }
            "cp" => {
                let n = t.u();
                let has = t.b();
                let start = t.pt();
                let pts: Vec<Point> = (0..n).map(|_| t.pt()).collect();
                let k = t.u();
                let expected: Vec<usize> = (0..k).map(|_| t.u()).collect();
                t.done();
                let got = chain_points(&pts, if has { Some(&start) } else { None });
                tally.check(kind, line_no, "order", &got, &expected);
            }
            "g1" | "g2" => {
                let rcf = t.b();
                let n = t.u();
                let has = t.b();
                let start = t.pt();
                let mut segs: Vec<(Point, Point)> = Vec::new();
                let mut cr: Vec<bool> = Vec::new();
                for _ in 0..n {
                    let a = t.pt();
                    let b = t.pt();
                    segs.push((a, b));
                    cr.push(t.b());
                }
                let expected = t.chain();
                t.done();
                let segment_end_point = |idx: usize, first_point: bool| -> Point { if first_point { segs[idx].0 } else { segs[idx].1 } };
                let could_reverse = |idx: usize| -> bool { cr[idx] };
                let sn = if has { Some(&start) } else { None };
                let got = match (kind, rcf) {
                    ("g1", false) => chain_segments_greedy(segment_end_point, n, sn),
                    ("g1", true) => chain_segments_greedy_constrained_reversals(segment_end_point, could_reverse, n, sn),
                    ("g2", false) => chain_segments_greedy2(segment_end_point, n, sn),
                    _ => chain_segments_greedy_constrained_reversals2(segment_end_point, could_reverse, n, sn),
                };
                let name = format!("{kind}{}", if rcf { "c" } else { "" });
                tally.check(&name, line_no, "chain", &got, &expected);
            }
            "pl" => {
                let n = t.u();
                let has = t.b();
                let start = t.pt();
                let pls = t.polylines(n);
                let k = t.u();
                let expected = t.polylines(k);
                t.done();
                let got = chain_polylines(pls, if has { Some(&start) } else { None });
                tally.check(kind, line_no, "polylines", &points_of(&got), &points_of(&expected));
            }
            "rst" => {
                let n = t.u();
                let mut pls = t.polylines(n);
                let expected = t.polylines(n);
                t.done();
                reorder_by_shortest_traverse(&mut pls);
                tally.check(kind, line_no, "polylines", &points_of(&pls), &points_of(&expected));
            }
            "imp" => {
                let n = t.u();
                let mut pls = t.polylines(n);
                let expected = t.polylines(n);
                t.done();
                improve_ordering_by_two_exchanges_with_segment_flipping(&mut pls, false);
                tally.check(kind, line_no, "polylines", &points_of(&pls), &points_of(&expected));
            }
            "two" => {
                let n = t.u();
                let mut edges = t.edges(n);
                let expected = t.edges_out(n);
                t.done();
                reorder_by_two_exchanges_with_segment_flipping(&mut edges);
                tally.check(kind, line_no, "edges", &edges_bits(&edges), &expected);
            }
            "mcc" => {
                let n = t.u();
                let edges = t.edges(n);
                let a = t.u();
                let b = t.u();
                let c: Vec<f64> = (0..7).map(|_| t.f()).collect();
                let expected = (t.f().to_bits(), t.u());
                t.done();
                let (c1, c2, c3) = (
                    ConnectionCost { cost: c[0], cost_flipped: c[1] },
                    ConnectionCost { cost: c[2], cost_flipped: c[3] },
                    ConnectionCost { cost: c[4], cost_flipped: c[5] },
                );
                let got = minimum_crossover_cost(&edges, (0, a), &c1, (a, b), &c2, (b, n), &c3, c[6]);
                tally.check(kind, line_no, "cost and flip", &(got.0.to_bits(), got.1), &expected);
            }
            "dc" => {
                let n = t.u();
                let edges = t.edges(n);
                let a = t.u();
                let b = t.u();
                let i = t.u();
                let expected = t.edges_out(n);
                t.done();
                let mut edges_out = edges.clone();
                do_crossover(&edges, &mut edges_out, (0, a), (a, b), (b, n), i);
                tally.check(kind, line_no, "edges", &edges_bits(&edges_out), &expected);
            }
            "ex" => {
                let n = t.u();
                let mut expolys: Vec<ExPolygon> = Vec::new();
                for _ in 0..n {
                    let contour = Polygon { points: t.polyline().points };
                    let h = t.u();
                    let holes = (0..h).map(|_| Polygon { points: t.polyline().points }).collect();
                    expolys.push(ExPolygon { contour, holes });
                }
                let expected: Vec<usize> = (0..n).map(|_| t.u()).collect();
                t.done();
                let got = chain_expolygons(&expolys);
                tally.check(kind, line_no, "order", &got, &expected);
            }
            "cl" => {
                let eps = t.f();
                let n = t.u();
                let lines_in: Vec<Line> = (0..n)
                    .map(|_| {
                        let a = t.pt();
                        let b = t.pt();
                        Line { a, b }
                    })
                    .collect();
                let k = t.u();
                let expected = t.polylines(k);
                t.done();
                let got = chain_lines(&lines_in, eps);
                tally.check(kind, line_no, "polylines", &points_of(&got), &points_of(&expected));
            }
            "ep" => {
                let n = t.u();
                let has = t.b();
                let start = t.pt();
                let mut paths: Vec<ExtrusionPath> = (0..n).map(|_| t.path()).collect();
                let expected_chain = t.chain();
                let k = t.u();
                let expected = t.polylines(k);
                t.done();
                let sn = if has { Some(&start) } else { None };
                let got_chain = chain_extrusion_paths(&paths, sn);
                tally.check(kind, line_no, "chain", &got_chain, &expected_chain);
                chain_and_reorder_extrusion_paths(&mut paths, sn);
                let got: Vec<Polyline> = paths.iter().map(|p| p.polyline.clone()).collect();
                tally.check("ep_reorder", line_no, "paths", &points_of(&got), &points_of(&expected));
            }
            "ee" => {
                let n = t.u();
                let has = t.b();
                let start = t.pt();
                let mut ents = t.entities(n);
                let all_full = t.b();
                let expected_chain = t.chain();
                let sn = if has { Some(&start) } else { None };
                if all_full {
                    let got_chain = chain_extrusion_entities(&ents, sn);
                    tally.check(kind, line_no, "chain", &got_chain, &expected_chain);
                }
                if t.b() {
                    let chain2 = t.chain();
                    let expected = put_entities(&t.entities(n));
                    let mut copy = ents.clone();
                    reorder_extrusion_entities(&mut copy, &chain2);
                    tally.check("ee_reorder", line_no, "entities", &put_entities(&copy), &expected);
                }
                let k = t.u();
                let expected = put_entities(&t.entities(k));
                t.done();
                chain_and_reorder_extrusion_entities(&mut ents, sn);
                tally.check("ee_chain_and_reorder", line_no, "entities", &put_entities(&ents), &expected);
            }
            "pn" => {
                let n = t.u();
                let pts: Vec<Point> = (0..n).map(|_| t.pt()).collect();
                let k = t.u();
                let expected: Vec<usize> = (0..k).map(|_| t.u()).collect();
                t.done();
                // The nodes are named by their place in the input, as gen.cpp prints them.
                let nodes: Vec<usize> = (0..n).collect();
                tally.check(kind, line_no, "nodes", &chain_clipper_polynodes(&pts, &nodes), &expected);
                tally.check("pn_items", line_no, "items", &chain_path_items(&pts, &nodes), &expected);
            }
            "inst" => {
                use slicer::print::faithful::{Print, PrintInstance, PrintObject};
                let no = t.u();
                let mut print = Print::new();
                for i in 0..no {
                    let k = t.u();
                    let instances: Vec<PrintInstance> = (0..k).map(|j| PrintInstance { print_object: i, model_instance: j, shift: t.pt(), id: j }).collect();
                    print.m_objects.push(PrintObject::with_model_object(i, slicer::geometry::geometry::Transform3d::identity(), instances));
                }
                let has = t.b();
                let start = t.pt();
                let mut read = |t: &mut Tok| -> Vec<(usize, usize)> {
                    let m = t.u();
                    (0..m)
                        .map(|_| {
                            let o = t.u();
                            let i = t.u();
                            (o, i)
                        })
                        .collect()
                };
                let expected = read(&mut t);
                let expected_print = read(&mut t);
                t.done();
                // An instance is named by its object and its place among the object's instances.
                let name = |v: Vec<&PrintInstance>| -> Vec<(usize, usize)> { v.iter().map(|pi| (pi.print_object, pi.model_instance)).collect() };
                let objects: Vec<&PrintObject> = print.objects().iter().collect();
                let got = name(chain_print_object_instances(&objects, if has { Some(&start) } else { None }));
                tally.check(kind, line_no, "instances", &got, &expected);
                let got_print = name(ChainPrintObjectInstances::chain_print_object_instances(&print));
                tally.check("inst_print", line_no, "instances", &got_print, &expected_print);
            }
            _ => panic!("unknown line: {line}"),
        }
    }
    let mut bad = 0usize;
    for (kind, (cases, mismatches)) in &tally.kinds {
        eprintln!("{kind}: {cases} cases, {mismatches} mismatches");
        bad += mismatches;
    }
    eprintln!("{lines} lines, {bad} mismatches");
    assert_eq!(bad, 0, "results that differ from the pin");
    assert_eq!(lines, 4543, "lines of the vectors file");
}
