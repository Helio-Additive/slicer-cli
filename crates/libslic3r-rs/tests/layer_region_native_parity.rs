//! Native parity of the member functions of LayerRegion (LayerRegion.cpp; `slicer::layer::LayerRegion`,
//! the methods the slicing path calls) against vectors made by the C++ at the pin
//! (tests/data/layer_region/gen.cpp) on the LayerRegion of a Layer of a real Print.
//!
//! A line "lcfg<TAB>tag<TAB>key=value..." holds the options the functions read as the pin's Print holds
//! them after `Print::apply`; the test builds the configs of this crate from them through their
//! `set_deserialize`, as the slicing path does. Every other line is "<what> <inputs> => <results>": the
//! test reads the inputs back, calls the port and has to print the same results. A long result is "#"
//! and the FNV-1a hash of its text unless the generator ran with "full".
//!
//! `LAYER_REGION_PARITY_VECTORS` names a file of the generator to read instead of the committed one,
//! `LAYER_REGION_FILTER` a kind of line to run alone, `LAYER_REGION_SHOW` how many differing lines to
//! print (with a file of full results).

use slicer::flow::{Flow, FlowRole};
use slicer::geometry::{ExPolygon, ExPolygons, Point, Polygon};
use slicer::layer::LayerRegion;
use slicer::surface::{Surface, SurfaceType};
use std::io::Read;

fn hd(v: f64) -> String {
    format!("{:016x}", v.to_bits())
}
fn hf(v: f64) -> String {
    format!("{:08x}", (v as f32).to_bits())
}
fn hb(v: bool) -> &'static str {
    if v {
        "1"
    } else {
        "0"
    }
}
fn poly_s(p: &Polygon) -> String {
    let mut o = p.points.len().to_string();
    for q in &p.points {
        o += &format!(" {} {}", q.x, q.y);
    }
    o
}
fn expoly_s(e: &ExPolygon) -> String {
    let mut o = format!("{} {}", poly_s(&e.contour), e.holes.len());
    for h in &e.holes {
        o += &format!(" {}", poly_s(h));
    }
    o
}
/// gen.cpp `SURFACES(ss)`: "type thickness thickness_layers bridge_angle expolygon" per surface. A surface
/// without a bridge angle has the -1 of the C++.
fn surfaces_s(ss: &[Surface]) -> String {
    let mut o = ss.len().to_string();
    for s in ss {
        o += &format!(" {} {} {} {} {}", s.surface_type as usize, hd(s.thickness), s.thickness_layers, hd(s.bridge_angle.unwrap_or(-1.)), expoly_s(&s.expolygon));
    }
    o
}
/// gen.cpp `FLOW(f)`
fn flow_s(f: &Flow) -> String {
    format!("{} {} {} {} {} {}", hf(f.width()), hf(f.height()), hf(f.spacing()), hf(f.nozzle_diameter()), hb(f.is_bridge()), hd(f.mm3_per_mm().unwrap_or(-1.)))
}

fn fnv(s: &str) -> String {
    let mut h: u64 = 1469598103934665603;
    for c in s.bytes() {
        h ^= c as u64;
        h = h.wrapping_mul(1099511628211);
    }
    format!("{h:016x}")
}

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
    fn d(&mut self) -> f64 {
        f64::from_bits(u64::from_str_radix(self.s(), 16).expect("double"))
    }
    fn f(&mut self) -> f32 {
        f32::from_bits(u32::from_str_radix(self.s(), 16).expect("float"))
    }
    fn poly(&mut self) -> Polygon {
        let n = self.i();
        let mut p = Polygon::new();
        for _ in 0..n {
            let x = self.i();
            let y = self.i();
            p.points.push(Point::new(x, y));
        }
        p
    }
    fn polys(&mut self) -> Vec<Polygon> {
        let n = self.i();
        (0..n).map(|_| self.poly()).collect()
    }
    fn expoly(&mut self) -> ExPolygon {
        let contour = self.poly();
        let nh = self.i();
        let mut holes = Vec::new();
        for _ in 0..nh {
            holes.push(self.poly());
        }
        ExPolygon::with_holes(contour, holes)
    }
    fn expolys(&mut self) -> ExPolygons {
        let n = self.i();
        (0..n).map(|_| self.expoly()).collect()
    }
    fn surfaces(&mut self) -> Vec<Surface> {
        let n = self.i();
        let mut ss = Vec::new();
        for _ in 0..n {
            let surface_type = SurfaceType::from_u8(self.i() as u8);
            let thickness = self.d();
            let thickness_layers = self.i() as usize;
            let bridge_angle = self.d();
            let mut s = Surface::new(surface_type, self.expoly());
            s.thickness = thickness;
            s.thickness_layers = thickness_layers;
            s.bridge_angle = if bridge_angle == -1. { None } else { Some(bridge_angle) };
            ss.push(s);
        }
        ss
    }
}

fn read_vectors() -> Vec<String> {
    if let Ok(path) = std::env::var("LAYER_REGION_PARITY_VECTORS") {
        let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
        return text.lines().map(|l| l.to_string()).collect();
    }
    let path = format!("{}/tests/data/layer_region/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).unwrap_or_else(|e| panic!("{path}: {e}"));
    text.lines().map(|l| l.to_string()).collect()
}

/// The configs of this crate made of the options of an "lcfg" line.
struct Cfg {
    tag: String,
    kv: std::collections::BTreeMap<String, String>,
    print_config: std::sync::Arc<slicer::print_config::PrintConfig>,
    object_config: std::sync::Arc<slicer::print_config::PrintObjectConfig>,
    region: std::sync::Arc<slicer::print_region::PrintRegion>,
}

fn cfg_of(rest: &str) -> Cfg {
    let fields: Vec<&str> = rest.split('\t').collect();
    let mut print_config = slicer::print_config::PrintConfig::default();
    let mut object_config = slicer::print_config::PrintObjectConfig::default();
    let mut region_config = slicer::region_config::PrintRegionConfig::default();
    let mut kv = std::collections::BTreeMap::new();
    for f in &fields[1..] {
        let (key, value) = f.split_once('=').expect("key=value");
        kv.insert(key.to_string(), value.to_string());
        // The project's values of the options with a value per extruder are lists; the slicing path hands
        // the first one over (app_slice.rs `load_bambustudio_settings`).
        let first = match key {
            "nozzle_diameter" => value.split(',').next().unwrap_or(value),
            _ => value,
        };
        print_config.set_deserialize(key, first);
        object_config.set_deserialize(key, first);
        region_config.set_deserialize(key, first);
    }
    Cfg {
        tag: fields[0].to_string(),
        kv,
        print_config: std::sync::Arc::new(print_config),
        object_config: std::sync::Arc::new(object_config),
        region: std::sync::Arc::new(slicer::print_region::PrintRegion::new(region_config)),
    }
}

/// The first token two texts differ in, with the tokens around it.
fn first_difference(a: &str, b: &str) -> String {
    let ta: Vec<&str> = a.split(' ').collect();
    let tb: Vec<&str> = b.split(' ').collect();
    let k = ta.iter().zip(tb.iter()).position(|(x, y)| x != y).unwrap_or(ta.len().min(tb.len()));
    let from = k.saturating_sub(10);
    let cut = |t: &[&str]| t[from.min(t.len())..(k + 12).min(t.len())].join(" ");
    format!("token {k} of {} / {}\n      C++:  {}\n      Rust: {}", ta.len(), tb.len(), cut(&ta), cut(&tb))
}

const KINDS: [&str; 6] = ["flow", "clip", "prep", "trim", "efc", "pes"];
/// The kinds whose lines have to be equal to the pin: the test fails on a difference in one of them and
/// reports the others.
/// R923: `flow` and `prep` are equal with the gates LAYER_REGION_FLOW_PIN and LAYER_REGION_PREPARE_PIN on
/// (their default); `pes` (process_external_surfaces) is reported, and held to the bound below.
const ASSERTED: [&str; 5] = ["flow", "clip", "prep", "trim", "efc"];
/// R924: with the gate LAYER_REGION_EXTERNAL_PIN on (its default: the solid and sparse surfaces carry the
/// layer thickness, LayerRegion.cpp:891-898, and expansion_min is not floored, LayerRegion.cpp:804-805,824)
/// `pes` differs from the pin on 3 of 280 lines (no_walls 2, large_min_area 1: a few points of one expanded
/// contour are 1 to 30 units off, the token counts are equal). Not equal yet: the test holds the count.
const PES_KNOWN_DIFFERING: usize = 3;

#[test]
fn layer_region_native_parity() {
    let expected = read_vectors();
    let filter = std::env::var("LAYER_REGION_FILTER").ok();
    let show: usize = std::env::var("LAYER_REGION_SHOW").ok().and_then(|v| v.parse().ok()).unwrap_or(0);
    // Per kind: lines and those that differ; per tag and kind the differing ones.
    let mut lines = [0usize; 6];
    let mut bad = [0usize; 6];
    let mut by_tag: std::collections::BTreeMap<(String, usize), usize> = std::collections::BTreeMap::new();
    let mut cfg: Option<Cfg> = None;
    let mut shown = 0usize;
    for (line_no, line) in expected.iter().enumerate() {
        if let Some(rest) = line.strip_prefix("lcfg\t") {
            cfg = Some(cfg_of(rest));
            continue;
        }
        let c = cfg.as_ref().expect("lcfg");
        let (what, rest) = line.split_once(' ').expect("kind");
        let kind = KINDS.iter().position(|k| *k == what).unwrap_or_else(|| panic!("unknown line {}", line_no + 1));
        if filter.as_ref().is_some_and(|f| f != what) {
            continue;
        }
        let (inputs, theirs) = rest.split_once(" => ").expect("results");
        let mut t = Tok::new(inputs);
        let layer_id = t.i() as usize;
        let height = t.d();
        let mut layerm = LayerRegion::new(layer_id, 0);
        layerm.set_config_hierarchy(c.region.clone(), c.object_config.clone(), c.print_config.clone());
        let mine: String = match what {
            "flow" => {
                const ROLES: [FlowRole; 5] = [FlowRole::ExternalPerimeter, FlowRole::Perimeter, FlowRole::Infill, FlowRole::SolidInfill, FlowRole::TopSolidInfill];
                let mut o = String::new();
                for role in ROLES {
                    o += &format!("{} ", flow_s(&layerm.flow(role, height).expect("flow")));
                }
                for role in ROLES {
                    for thick in [false, true] {
                        o += &format!("{} ", flow_s(&layerm.bridging_flow(role, thick, height).expect("bridging_flow")));
                    }
                }
                o += &format!("{} ", flow_s(&layerm.flow(FlowRole::Perimeter, 0.28).expect("flow")));
                // LayerRegion.cpp:957-961 `infill_area_threshold()`: the square of the scaled spacing of the
                // solid infill flow of the layer.
                o += &hd(layerm.infill_area_threshold_with_flow(&layerm.flow(FlowRole::SolidInfill, height).expect("flow")));
                o
            }
            "clip" => {
                layerm.slices.surfaces = t.surfaces();
                layerm.fill_expolygons = t.expolys();
                layerm.slices_to_fill_surfaces_clipped();
                surfaces_s(&layerm.fill_surfaces.surfaces)
            }
            "prep" => {
                layerm.fill_surfaces.surfaces = t.surfaces();
                layerm.prepare_fill_surfaces();
                surfaces_s(&layerm.fill_surfaces.surfaces)
            }
            "trim" => {
                layerm.slices.surfaces = t.surfaces();
                let trimming = t.polys();
                layerm.trim_surfaces(&trimming);
                surfaces_s(&layerm.slices.surfaces)
            }
            "efc" => {
                let step = t.f();
                layerm.slices.surfaces = t.surfaces();
                let trimming = t.polys();
                layerm.elephant_foot_compensation_step((step, &trimming));
                surfaces_s(&layerm.slices.surfaces)
            }
            "pes" => {
                layerm.fill_surfaces.surfaces = t.surfaces();
                layerm.process_external_surfaces(height).expect("process_external_surfaces");
                surfaces_s(&layerm.fill_surfaces.surfaces)
            }
            _ => unreachable!(),
        };
        let mine = mine.trim_end().to_string();
        let theirs = theirs.trim_end();
        lines[kind] += 1;
        let equal = match theirs.strip_prefix('#') {
            Some(hash) => fnv(&mine) == hash,
            None => mine == theirs,
        };
        if !equal {
            bad[kind] += 1;
            *by_tag.entry((c.tag.clone(), kind)).or_insert(0) += 1;
            if shown < show && !theirs.starts_with('#') {
                shown += 1;
                eprintln!("line {} {} {what} (layer {layer_id}): {}", line_no + 1, c.tag, first_difference(theirs, &mine));
            }
        }
        let _ = &c.kv;
    }
    for k in 0..6 {
        if lines[k] > 0 {
            let tags: Vec<String> = by_tag.iter().filter(|((_, kind), _)| *kind == k).map(|((tag, _), n)| format!("{tag} {n}")).collect();
            eprintln!("{}: {} lines, {} differ{}{}", KINDS[k], lines[k], bad[k], if tags.is_empty() { "" } else { ": " }, tags.join(", "));
        }
    }
    eprintln!("layer_region parity: {} lines, {} differ", lines.iter().sum::<usize>(), bad.iter().sum::<usize>());
    let failing: Vec<String> = (0..6).filter(|k| ASSERTED.contains(&KINDS[*k]) && bad[*k] > 0).map(|k| format!("{} {}", KINDS[k], bad[k])).collect();
    assert!(failing.is_empty(), "LayerRegion differs from the pin: {}", failing.join(", "));
    assert!(bad[5] <= PES_KNOWN_DIFFERING, "process_external_surfaces: {} lines differ from the pin, {PES_KNOWN_DIFFERING} are known", bad[5]);
}
