//! Shared by the native parity tests of the support units (support_parameters, support_common,
//! support_material): the formatting of the vectors, their reader, and `Print::apply` on a fresh Print as
//! tests/print_native_parity.rs and tests/tree_support_native_parity.rs assemble it.
#![allow(dead_code)]

use slicer::config::{ConfigBase, ConfigOptionInts, ConfigOptionResolver, ConfigSubstitutionContext, ForwardCompatibilitySubstitutionRule};
use slicer::geometry::{ExPolygon, ExPolygons, Point, Polygon};
use slicer::model::*;
use slicer::normal_utils::indexed_triangle_set;
use slicer::print::faithful::{Print, PrintObject};
use slicer::print_apply::*;
use slicer::print_config::config_def::DynamicPrintConfig;
use slicer::triangle_mesh::{its_make_cube, Vec3f};

pub fn hd(v: f64) -> String {
    format!("{:016x}", v.to_bits())
}
pub fn hf(v: f32) -> String {
    format!("{:08x}", v.to_bits())
}
pub fn hb(v: bool) -> &'static str {
    if v {
        "1"
    } else {
        "0"
    }
}
pub fn pt(p: &Point) -> String {
    format!("{} {}", p.x, p.y)
}
pub fn poly_s(p: &Polygon) -> String {
    let mut o = format!("{}", p.points.len());
    for q in &p.points {
        o += &format!(" {}", pt(q));
    }
    o
}
pub fn polys_s(ps: &[Polygon]) -> String {
    let mut o = format!("{}", ps.len());
    for p in ps {
        o += &format!(" {}", poly_s(p));
    }
    o
}
pub fn expoly_s(e: &ExPolygon) -> String {
    let mut o = format!("{} {}", poly_s(&e.contour), e.holes.len());
    for h in &e.holes {
        o += &format!(" {}", poly_s(h));
    }
    o
}
pub fn expolys_s(es: &[ExPolygon]) -> String {
    let mut o = format!("{}", es.len());
    for e in es {
        o += &format!(" {}", expoly_s(e));
    }
    o
}

/// The tokens of an input line.
pub struct Tok<'a> {
    pub t: std::str::SplitWhitespace<'a>,
}
impl<'a> Tok<'a> {
    pub fn new(s: &'a str) -> Self {
        Tok { t: s.split_whitespace() }
    }
    pub fn s(&mut self) -> &'a str {
        self.t.next().expect("token")
    }
    pub fn i(&mut self) -> i64 {
        self.s().parse().expect("integer")
    }
    pub fn d(&mut self) -> f64 {
        f64::from_bits(u64::from_str_radix(self.s(), 16).expect("double"))
    }
    pub fn b(&mut self) -> bool {
        self.s() == "1"
    }
    pub fn poly(&mut self) -> Polygon {
        let n = self.i();
        let mut p = Polygon::new();
        for _ in 0..n {
            let x = self.i();
            let y = self.i();
            p.points.push(Point::new(x, y));
        }
        p
    }
    pub fn expoly(&mut self) -> ExPolygon {
        let contour = self.poly();
        let nh = self.i();
        let mut holes = Vec::new();
        for _ in 0..nh {
            holes.push(self.poly());
        }
        ExPolygon::with_holes(contour, holes)
    }
    pub fn expolys(&mut self) -> ExPolygons {
        let n = self.i();
        (0..n).map(|_| self.expoly()).collect()
    }
}

/// The vectors, read in order: an input line is handed out and echoed, a result line is produced.
pub struct Run {
    pub expected: Vec<String>,
    pub pos: usize,
    pub lines: Vec<String>,
    pub surplus: usize,
}
impl Run {
    /// The next input line when it is the input `prefix`; it is echoed. The results produced since the
    /// input before it are aligned with the vectors: missing ones are padded, surplus ones counted.
    pub fn input(&mut self, prefix: &str) -> Option<String> {
        let j = (self.pos..self.expected.len()).find(|&k| self.expected[k].starts_with("in "))?;
        let line = self.expected[j].clone();
        if !(line.starts_with(prefix) && line[prefix.len()..].starts_with(' ')) {
            return None;
        }
        while self.lines.len() < j {
            self.lines.push("<missing>".into());
        }
        if self.lines.len() > j {
            self.surplus += self.lines.len() - j;
            self.lines.truncate(j);
        }
        self.lines.push(line.clone());
        self.pos = j + 1;
        Some(line[prefix.len() + 1..].to_string())
    }
    pub fn out(&mut self, s: String) {
        self.lines.push(s);
    }
}

pub fn box_its(sx: f64, sy: f64, sz: f64, ox: f32, oy: f32, oz: f32) -> indexed_triangle_set {
    let mut its = its_make_cube(sx, sy, sz);
    for v in &mut its.vertices {
        *v += Vec3f::new(ox, oy, oz);
    }
    its
}

pub fn deser(config: &mut DynamicPrintConfig, key: &str, value: &str) {
    let mut ctxt = ConfigSubstitutionContext::new(ForwardCompatibilitySubstitutionRule::Disable);
    config.set_deserialize(key, value, &mut ctxt, false).unwrap_or_else(|e| panic!("{key}={value}: {e:?}"));
}

/// `Print::apply` on a fresh Print, as tests/print_native_parity.rs assembles it: the builder a live caller of the
/// organic tree support uses (src/support/tree_support_3d.rs `assemble_faithful_print`, PrintApply.cpp:1290-2331).
pub fn assemble(model: &Model, config: &DynamicPrintConfig) -> Print {
    slicer::support::tree_support_3d::assemble_faithful_print(model, config)
}

impl<'a> Tok<'a> {
    pub fn f(&mut self) -> f32 {
        f32::from_bits(u32::from_str_radix(self.s(), 16).expect("float"))
    }
    pub fn polys(&mut self) -> Vec<Polygon> {
        let n = self.i();
        (0..n).map(|_| self.poly()).collect()
    }
    pub fn pt(&mut self) -> Point {
        let x = self.i();
        let y = self.i();
        Point::new(x, y)
    }
}

impl Run {
    /// The vectors of `data/<unit>/vectors.txt` (or `vectors.txt.gz`).
    pub fn load(unit: &str) -> Run {
        use std::io::Read;
        let dir = format!("{}/tests/data/{}", env!("CARGO_MANIFEST_DIR"), unit);
        let mut text = String::new();
        if let Ok(f) = std::fs::File::open(format!("{dir}/vectors.txt.gz")) {
            flate2::read::GzDecoder::new(f).read_to_string(&mut text).expect("vectors.txt.gz");
        } else {
            text = std::fs::read_to_string(format!("{dir}/vectors.txt")).expect("vectors.txt");
        }
        Run { expected: text.lines().map(|l| l.to_string()).collect(), pos: 0, lines: Vec::new(), surplus: 0 }
    }
    /// Every line produced against the vectors: the number of lines that differ, the first few printed.
    pub fn finish(&mut self) -> usize {
        while self.lines.len() < self.expected.len() {
            self.lines.push("<missing>".into());
        }
        let mut bad = self.surplus;
        if self.lines.len() > self.expected.len() {
            bad += self.lines.len() - self.expected.len();
        }
        let mut shown = 0;
        for (i, e) in self.expected.iter().enumerate() {
            if &self.lines[i] != e {
                bad += 1;
                if shown < 12 {
                    shown += 1;
                    let cut = |s: &str| if s.len() > 700 { format!("{}...", &s[..700]) } else { s.to_string() };
                    eprintln!("line {}:\n  expected {}\n  got      {}", i + 1, cut(e), cut(&self.lines[i]));
                }
            }
        }
        bad
    }
    /// The number of result lines (not inputs) of the vectors whose first tokens are `prefix`, and how many
    /// of them the run reproduced.
    pub fn tally(&self, prefix: &str) -> (usize, usize) {
        let mut n = 0;
        let mut ok = 0;
        for (i, e) in self.expected.iter().enumerate() {
            if e.starts_with(prefix) {
                n += 1;
                if self.lines.get(i) == Some(e) {
                    ok += 1;
                }
            }
        }
        (n, ok)
    }
}
