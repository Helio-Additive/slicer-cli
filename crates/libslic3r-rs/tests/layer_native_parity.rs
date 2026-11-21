//! Native parity of `ContinuitiousDistancer` and `Layer::calculate_perimeter_continuity`
//! (`slicer::layer`) against vectors made by the C++ at the pin (tests/data/layer/gen.cpp).
//!
//! Every line of the vectors is "<what> <inputs> => <results>": this test reads the inputs back, calls
//! the port and has to print the same results. `LAYER_PARITY_VECTORS` names a file of the generator to
//! read instead of the committed one.

use slicer::geometry::{BoundingBox, Point};
use slicer::layer::{ContinuitiousDistancer, Layer, LoopNode, NodeContour};
use std::io::Read;

/// The tokens of the inputs of a line.
struct Tok<'a> {
    t: std::str::SplitWhitespace<'a>,
}
impl<'a> Tok<'a> {
    fn new(s: &'a str) -> Self {
        Tok { t: s.split_whitespace() }
    }
    fn s(&mut self) -> &'a str {
        self.t.next().expect("token")
    }
    fn i(&mut self) -> i64 {
        self.s().parse().expect("integer")
    }
    fn f(&mut self) -> f32 {
        f32::from_bits(u32::from_str_radix(self.s(), 16).expect("float"))
    }
    fn b(&mut self) -> bool {
        self.s() == "1"
    }
    fn point(&mut self) -> Point {
        let x = self.i();
        let y = self.i();
        Point::new(x, y)
    }
    /// gen.cpp `NODE(n)`
    fn node(&mut self) -> LoopNode {
        let node_id = self.i() as usize;
        let min = self.point();
        let max = self.point();
        let is_loop = self.b();
        let np = self.i();
        let pts: Vec<Point> = (0..np).map(|_| self.point()).collect();
        let nw = self.i();
        let widths: Vec<i64> = (0..nw).map(|_| self.i()).collect();
        let mut node = LoopNode::new(node_id, node_id, BoundingBox::from_points_minmax(min, max));
        node.node_contour = NodeContour { pts, widths, is_loop };
        node
    }
}

fn read_vectors() -> Vec<String> {
    if let Ok(path) = std::env::var("LAYER_PARITY_VECTORS") {
        let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
        return text.lines().map(|l| l.to_string()).collect();
    }
    let path = format!("{}/tests/data/layer/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).unwrap_or_else(|e| panic!("{path}: {e}"));
    text.lines().map(|l| l.to_string()).collect()
}

#[test]
fn layer_native_parity() {
    let expected = read_vectors();
    let mut mismatches = 0usize;
    let mut by_kind: std::collections::BTreeMap<String, (usize, usize)> = std::collections::BTreeMap::new();
    for (k, line) in expected.iter().enumerate() {
        let (head, want) = line.split_once(" => ").unwrap_or_else(|| panic!("no result in {line}"));
        let (what, inputs) = head.split_once(' ').unwrap_or((head, ""));
        let mut t = Tok::new(inputs);
        let result: String = match what {
            "dist" => {
                let np = t.i();
                let pts: Vec<Point> = (0..np).map(|_| t.point()).collect();
                let distancer = ContinuitiousDistancer::new(&pts);
                let nq = t.i();
                let mut o: Vec<String> = Vec::new();
                for _ in 0..nq {
                    let x = t.f();
                    let y = t.f();
                    o.push(format!("{:08x}", distancer.distance_from_perimeter([x, y]).to_bits()));
                }
                o.join(" ")
            }
            "continuity" => {
                let n = t.i();
                let mut layer = Layer::new(1, 0, 0.2, 0.4, 0.3);
                for _ in 0..n {
                    let node = t.node();
                    layer.loop_nodes.push(node);
                }
                let m = t.i();
                let mut prev_nodes: Vec<LoopNode> = (0..m).map(|_| t.node()).collect();
                layer.calculate_perimeter_continuity(&mut prev_nodes);
                let mut o: Vec<String> = Vec::new();
                for node in &layer.loop_nodes {
                    o.push(node.lower_node_ids.len().to_string());
                    o.extend(node.lower_node_ids.iter().map(|id| id.to_string()));
                }
                for node in &prev_nodes {
                    o.push(node.upper_node_ids.len().to_string());
                    o.extend(node.upper_node_ids.iter().map(|id| id.to_string()));
                }
                o.join(" ")
            }
            other => panic!("unknown line {other}"),
        };
        let entry = by_kind.entry(what.to_string()).or_insert((0, 0));
        entry.0 += 1;
        if result != want {
            entry.1 += 1;
            mismatches += 1;
            if mismatches <= 10 {
                let cut = |s: &str| if s.len() > 500 { format!("{}...", &s[..500]) } else { s.to_string() };
                eprintln!("line {} {}:\n  C++:  {}\n  Rust: {}", k + 1, cut(head), cut(want), cut(&result));
            }
        }
    }
    for (kind, (n, bad)) in &by_kind {
        eprintln!("{kind}: {n} lines, {bad} differ");
    }
    eprintln!("layer parity: {} lines, {} mismatches", expected.len(), mismatches);
    assert_eq!(mismatches, 0, "{mismatches} of {} lines differ from the pin", expected.len());
}
