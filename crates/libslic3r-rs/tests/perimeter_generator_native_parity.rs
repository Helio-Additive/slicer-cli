//! Native parity of the functions of PerimeterGenerator.cpp that can be called on their own
//! (`slicer::perimeter_generator`) against vectors made by the C++ at the pin
//! (tests/data/perimeter_generator/gen.cpp).
//!
//! Every line of the vectors but the configuration lines is "<what> <inputs> => <results>": this test
//! reads the inputs back, calls the port and has to print the same results. A configuration line is
//! "cfg<TAB>multi_nozzle_diameter_mode<TAB>key=value..." (the keys set over the defaults; the lines
//! after it are computed with it) or "rcfg<TAB>index<TAB>key=value..." (the config of a PrintRegion).
//! `PERIMETER_GENERATOR_PARITY_VECTORS` names a file of the generator to read instead of the committed
//! one, `PERIMETER_GENERATOR_PARITY_DUMP` a file the lines of this test are written to.

use slicer::config::{ConfigBase, ConfigOptionEnumsGeneric, ConfigOptionResolver, ConfigSubstitutionContext, ForwardCompatibilitySubstitutionRule};
use slicer::arachne::utils::extrusion_junction::ExtrusionJunction;
use slicer::arachne::utils::extrusion_line::ExtrusionLine;
use slicer::clipper_z_utils::ZPaths;
use slicer::extrusion_entity::{ExtrusionEntityCollection, ExtrusionEntityType, ExtrusionPath, ExtrusionRole};
use slicer::layer::LoopNode;
use slicer::flow::Flow;
use slicer::geometry::{BoundingBox, ExPolygon, ExPolygons, Point, Polygon};
use slicer::layer::LayerRegion;
use slicer::perimeter_generator::*;
use slicer::print::faithful::{static_config_defaults, Print, PrintRegion, PrintRegionPtr};
use slicer::print_config::config_def::DynamicPrintConfig;
use slicer::surface::{Surface, SurfaceType};
use std::cell::RefCell;
use std::io::Read;
use std::rc::Rc;

fn hd(v: f64) -> String {
    format!("{:016x}", v.to_bits())
}
fn hb(v: bool) -> &'static str {
    if v {
        "1"
    } else {
        "0"
    }
}
fn pt(p: &Point) -> String {
    format!("{} {}", p.x, p.y)
}
fn poly_s(p: &Polygon) -> String {
    let mut o = p.points.len().to_string();
    for q in &p.points {
        o += &format!(" {}", pt(q));
    }
    o
}
fn polys_s(ps: &[Polygon]) -> String {
    let mut o = ps.len().to_string();
    for p in ps {
        o += &format!(" {}", poly_s(p));
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
fn expolys_s(es: &[ExPolygon]) -> String {
    let mut o = es.len().to_string();
    for e in es {
        o += &format!(" {}", expoly_s(e));
    }
    o
}
/// gen.cpp `BBOX(b)`: the corners of a box that is defined, "0" alone of one that is not.
fn bbox_s(b: &BoundingBox) -> String {
    if b.is_defined() {
        format!("1 {} {}", pt(&b.min), pt(&b.max))
    } else {
        "0".to_string()
    }
}
/// ExtrusionEntity.hpp:45-70 `enum ExtrusionRole`
fn role_from_int(v: i64) -> ExtrusionRole {
    match v {
        0 => ExtrusionRole::None,
        1 => ExtrusionRole::Perimeter,
        2 => ExtrusionRole::ExternalPerimeter,
        3 => ExtrusionRole::OverhangPerimeter,
        4 => ExtrusionRole::InternalInfill,
        14 => ExtrusionRole::SupportMaterial,
        _ => panic!("role {v}"),
    }
}
fn role_to_int(r: ExtrusionRole) -> i64 {
    match r {
        ExtrusionRole::None => 0,
        ExtrusionRole::Perimeter => 1,
        ExtrusionRole::ExternalPerimeter => 2,
        ExtrusionRole::OverhangPerimeter => 3,
        ExtrusionRole::InternalInfill => 4,
        ExtrusionRole::SupportMaterial => 14,
        other => panic!("role {other:?}"),
    }
}
fn paths_s(paths: &[ExtrusionPath]) -> String {
    let mut o = paths.len().to_string();
    for p in paths {
        o += &format!(" {} {} {}", role_to_int(p.role), hd(p.overhang_degree), p.polyline.points.len());
        for q in &p.polyline.points {
            o += &format!(" {}", pt(q));
        }
    }
    o
}

/// gen.cpp `XPATHS(paths)`: the paths with everything thick_polyline_to_multi_path sets.
fn xpaths_s(paths: &[ExtrusionPath]) -> String {
    let mut o = paths.len().to_string();
    for p in paths {
        o += &format!(
            " {} {} {:08x} {:08x} {} {}",
            role_to_int(p.role),
            hd(p.mm3_per_mm),
            (p.width as f32).to_bits(),
            (p.height as f32).to_bits(),
            hd(p.overhang_degree),
            p.polyline.points.len()
        );
        for q in &p.polyline.points {
            o += &format!(" {}", pt(q));
        }
    }
    o
}

/// gen.cpp `COLL(coll)`
fn coll_s(coll: &ExtrusionEntityCollection) -> String {
    let paths_of = |paths: &[ExtrusionPath]| {
        let mut o = xpaths_s(paths);
        for p in paths {
            o += &format!(" {}", p.customize_flag as i32);
        }
        o
    };
    let mut o = coll.entities.len().to_string();
    for e in &coll.entities {
        match e {
            ExtrusionEntityType::Loop(l) => o += &format!(" L {} {} {}", l.loop_role.0, l.customize_flag as i32, paths_of(&l.paths)),
            ExtrusionEntityType::MultiPath(m) => o += &format!(" M {}", paths_of(&m.paths)),
            _ => o += " ?",
        }
    }
    format!("{o} {} {}", coll.loop_node_range.0, coll.loop_node_range.1)
}
/// gen.cpp `NODES(nodes)`
fn nodes_s(nodes: &[LoopNode]) -> String {
    let mut o = nodes.len().to_string();
    for n in nodes {
        o += &format!(" {} {} {} {} {}", n.node_id, n.loop_id, bbox_s(&n.bbox), hb(n.node_contour.is_loop), n.node_contour.pts.len());
        for i in 0..n.node_contour.pts.len() {
            o += &format!(" {} {}", pt(&n.node_contour.pts[i]), n.node_contour.widths[i]);
        }
    }
    o
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
    fn b(&mut self) -> bool {
        self.s() == "1"
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
    /// gen.cpp `FlowCase`: "1 dmr nozzle" or "0 width height nozzle".
    fn flow(&mut self) -> Flow {
        if self.b() {
            let dmr = self.f();
            let nozzle = self.f();
            Flow::bridging_flow(dmr as f64, nozzle as f64)
        } else {
            let width = self.f();
            let height = self.f();
            let nozzle = self.f();
            Flow::new(width as f64, height as f64, nozzle as f64).expect("flow")
        }
    }
    fn zpaths(&mut self) -> ZPaths {
        let n = self.i();
        let mut paths: ZPaths = Vec::new();
        for _ in 0..n {
            let np = self.i();
            let mut path = Vec::new();
            for _ in 0..np {
                let x = self.i();
                let y = self.i();
                let z = self.i();
                path.push((x, y, z));
            }
            paths.push(path);
        }
        paths
    }
    fn polys(&mut self) -> Vec<Polygon> {
        let n = self.i();
        (0..n).map(|_| self.poly()).collect()
    }
    fn paths(&mut self) -> Vec<ExtrusionPath> {
        let n = self.i();
        let mut paths = Vec::new();
        for _ in 0..n {
            let mut p = ExtrusionPath::new(role_from_int(self.i()));
            p.overhang_degree = self.d();
            let np = self.i();
            for _ in 0..np {
                let x = self.i();
                let y = self.i();
                p.polyline.points.push(Point::new(x, y));
            }
            paths.push(p);
        }
        paths
    }
}

/// gen.cpp `set_keys(full, kvs)`: the keys of a case over the defaults.
fn full_config(kvs: &[&str]) -> DynamicPrintConfig {
    let mut full = DynamicPrintConfig::full_print_config();
    for kv in kvs {
        let (key, value) = kv.split_once('=').expect("key=value");
        if key == "extruder_type" {
            // The generator sets this one by value (PrintConfig.hpp:376-380 `enum ExtruderType`).
            let opt = full.option_mut(key, true).expect("extruder_type").expect("extruder_type");
            let opt = opt.as_any_mut().downcast_mut::<ConfigOptionEnumsGeneric>().expect("ConfigOptionEnumsGeneric");
            opt.values = value.split(',').map(|v| if v == "Bowden" { 1 } else { 0 }).collect();
            continue;
        }
        let mut ctxt = ConfigSubstitutionContext::new(ForwardCompatibilitySubstitutionRule::Disable);
        full.set_deserialize(key, value, &mut ctxt, false).unwrap_or_else(|e| panic!("{key}={value}: {e:?}"));
    }
    full
}

/// `T config; config.apply(full, true);` of the static class `class_name`.
fn static_config(class_name: &str, full: &DynamicPrintConfig) -> DynamicPrintConfig {
    let mut config = static_config_defaults(class_name);
    config.apply(full, true).unwrap_or_else(|e| panic!("{class_name}: {e:?}"));
    config
}

/// What a "cfg" line sets up: gen.cpp `run_configs`.
struct Cfg {
    multi_nozzle_diameter_mode: bool,
    print: Print,
    region_config: DynamicPrintConfig,
    object_config: DynamicPrintConfig,
}

fn region_index(regions: &[PrintRegionPtr], r: &PrintRegionPtr) -> i64 {
    regions.iter().position(|x| Rc::ptr_eq(x, r)).map(|i| i as i64).unwrap_or(-1)
}

/// gen.cpp: a LayerRegion of the region with the expolygons as the surfaces of its slices.
fn layer_region_of(es: &ExPolygons) -> LayerRegion {
    let mut layer_region = LayerRegion::new(0, 0);
    for e in es {
        layer_region.slices.surfaces.push(Surface::new(SurfaceType::Internal, e.clone()));
    }
    layer_region
}

fn perimeter_regions_s(regions: &[PrintRegionPtr], prs: &[PerimeterRegion]) -> String {
    let mut o = String::new();
    for pr in prs {
        o += &format!(" {} {} {}", region_index(regions, &pr.region), expolys_s(&pr.expolygons), bbox_s(&pr.bbox));
    }
    o
}

fn read_vectors() -> Vec<String> {
    if let Ok(path) = std::env::var("PERIMETER_GENERATOR_PARITY_VECTORS") {
        let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
        return text.lines().map(|l| l.to_string()).collect();
    }
    let path = format!("{}/tests/data/perimeter_generator/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).unwrap_or_else(|e| panic!("{path}: {e}"));
    text.lines().map(|l| l.to_string()).collect()
}

#[test]
fn perimeter_generator_native_parity() {
    let expected = read_vectors();
    let mut lines: Vec<String> = Vec::with_capacity(expected.len());
    let mut cfg: Option<Cfg> = None;
    let mut region_configs: Vec<DynamicPrintConfig> = Vec::new();
    let mut regions: Vec<PrintRegionPtr> = Vec::new();

    for line in &expected {
        if let Some(rest) = line.strip_prefix("cfg\t") {
            let fields: Vec<&str> = rest.split('\t').collect();
            let full = full_config(&fields[1..]);
            let mut print = Print::new();
            print.m_config = static_config("PrintConfig", &full);
            cfg = Some(Cfg {
                multi_nozzle_diameter_mode: fields[0] == "1",
                print,
                region_config: static_config("PrintRegionConfig", &full),
                object_config: static_config("PrintObjectConfig", &full),
            });
            lines.push(line.clone());
            continue;
        }
        if let Some(rest) = line.strip_prefix("rcfg\t") {
            let fields: Vec<&str> = rest.split('\t').collect();
            let index: usize = fields[0].parse().expect("index");
            assert_eq!(index, region_configs.len());
            let config = static_config("PrintRegionConfig", &full_config(&fields[1..]));
            regions.push(Rc::new(RefCell::new(PrintRegion::new(config.clone()))));
            region_configs.push(config);
            lines.push(line.clone());
            continue;
        }
        let (head, _) = line.split_once(" =>").unwrap_or_else(|| panic!("no result in {line}"));
        let (what, inputs) = head.split_once(' ').unwrap_or((head, ""));
        let mut t = Tok::new(inputs);
        let result: String = match what {
            "lowpass" => {
                let mut paths = t.paths();
                lowpass_filter_by_paths_overhang_degree(&mut paths);
                paths_s(&paths)
            }
            "smooth" => {
                let mut paths = t.paths();
                smooth_overhang_level(&mut paths);
                paths_s(&paths)
            }
            "flags" => {
                let e = t.expoly();
                let contour_flag = t.b();
                let nf = t.i();
                let holes_flag: Vec<i32> = (0..nf).map(|_| t.i() as i32).collect();
                let npre = t.i();
                let mut flags_out: Vec<i32> = (0..npre).map(|_| t.i() as i32).collect();
                let polygons = to_polygons_with_flag(&e, contour_flag, &holes_flag, &mut flags_out);
                let mut o = format!("{} {}", polys_s(&polygons), flags_out.len());
                for f in &flags_out {
                    o += &format!(" {f}");
                }
                o
            }
            "loop" => {
                let depth = t.i() as usize;
                let is_contour = t.b();
                let nc = t.i();
                let square = |side: i64| Polygon::from_points(vec![Point::new(0, 0), Point::new(side, 0), Point::new(side, side), Point::new(0, side)]);
                let mut l = PerimeterGeneratorLoop::new(square(1000), depth, is_contour, false, 0.);
                for _ in 0..nc {
                    let child_is_contour = t.b();
                    l.children.push(PerimeterGeneratorLoop::new(square(500), depth + 1, child_is_contour, false, 0.));
                }
                format!("{} {}", hb(l.is_external()), hb(l.is_internal_contour()))
            }
            "overhang_speed" => {
                let c = cfg.as_ref().expect("cfg");
                hb(is_enable_overhang_speed(&c.print, &c.region_config)).to_string()
            }
            "fuzzy_slowdown" => {
                let c = cfg.as_ref().expect("cfg");
                let perimeter_regions_empty = t.b();
                hb(fuzzy_skin_allows_overhang_slowdown(&c.region_config, perimeter_regions_empty)).to_string()
            }
            "wall_loops" => {
                let c = cfg.as_ref().expect("cfg");
                PerimeterGenerator::effective_wall_loops(&c.print.m_config, &c.region_config, c.multi_nozzle_diameter_mode).to_string()
            }
            "dist_boundary" => {
                let c = cfg.as_ref().expect("cfg");
                let width = t.d();
                let b = PerimeterGenerator::dist_boundary(&c.print.m_config, &c.region_config, width);
                format!("{} {}", hd(b.0), hd(b.1))
            }
            "lower_series" => {
                let c = cfg.as_ref().expect("cfg");
                let has_lower = t.b();
                let width = t.f();
                let lower = t.expolys();
                let series =
                    PerimeterGenerator::generate_lower_polygons_series(&c.print.m_config, &c.region_config, if has_lower { Some(&lower) } else { None }, width);
                let mut o = series.len().to_string();
                for ps in &series {
                    o += &format!(" {}", polys_s(ps));
                }
                o
            }
            "compat" => {
                let i = t.i() as usize;
                let j = t.i() as usize;
                hb(PerimeterRegion::has_compatible_perimeter_regions(&region_configs[i], &region_configs[j])).to_string()
            }
            "region_new" => {
                let r = t.i() as usize;
                let es = t.expolys();
                let pr = PerimeterRegion::new(regions[r].clone(), &layer_region_of(&es));
                perimeter_regions_s(&regions, std::slice::from_ref(&pr)).trim_start().to_string()
            }
            "merge" => {
                let n = t.i();
                let mut prs: PerimeterRegions = Vec::new();
                for _ in 0..n {
                    let r = t.i() as usize;
                    let es = t.expolys();
                    prs.push(PerimeterRegion::new(regions[r].clone(), &layer_region_of(&es)));
                }
                PerimeterRegion::merge_compatible_perimeter_regions(&mut prs);
                format!("{}{}", prs.len(), perimeter_regions_s(&regions, &prs))
            }
            "bridge_wall" => {
                let flow = t.flow();
                let role = role_from_int(t.i());
                let path_overhang = t.zpaths();
                let mut paths: Vec<ExtrusionPath> = Vec::new();
                detect_brigde_wall_arachne(&mut paths, &path_overhang, role, &flow);
                xpaths_s(&paths)
            }
            "traverse" => {
                let c = cfg.as_ref().expect("cfg");
                let layer_id = t.i() as i32;
                let starts_at_object_base = t.b();
                let slice_z = t.d();
                let perimeter_flow = t.flow();
                let ext_perimeter_flow = t.flow();
                let overhang_flow = t.flow();
                let nodes_before = t.i() as usize;
                let lower_slices_polygons = t.polys();
                let n = t.i();
                let mut lines: Vec<ExtrusionLine> = Vec::new();
                let mut contours: Vec<bool> = Vec::new();
                for _ in 0..n {
                    let inset_idx = t.i() as usize;
                    let is_odd = t.b();
                    let is_closed = t.b();
                    contours.push(t.b());
                    let nj = t.i();
                    let mut line = ExtrusionLine::with_closed(inset_idx, is_odd, is_closed);
                    for _ in 0..nj {
                        let x = t.i();
                        let y = t.i();
                        let w = t.i();
                        let flag = t.b();
                        line.junctions.push(ExtrusionJunction::with_hole_compensation(Point::new(x, y), w, inset_idx, flag));
                    }
                    lines.push(line);
                }
                // gen.cpp: the nodes of the islands before this one.
                let mut loop_nodes: Vec<LoopNode> = (0..nodes_before).map(|i| LoopNode::new(i, i, BoundingBox::new())).collect();
                let members = PerimeterGeneratorArachneMembers {
                    print: &c.print,
                    config: &c.region_config,
                    object_config: &c.object_config,
                    layer_id,
                    starts_at_object_base,
                    slice_z,
                    perimeter_flow,
                    ext_perimeter_flow,
                    overhang_flow,
                    preset_nozzle_diameter: 0.4,
                    multi_nozzle_diameter_mode: c.multi_nozzle_diameter_mode,
                    perimeter_regions_empty: true,
                    lower_slices_polygons,
                };
                let mut pg_extrusions: Vec<PerimeterGeneratorArachneExtrusion> = lines
                    .iter_mut()
                    .zip(contours.iter())
                    .map(|(extrusion, is_contour)| PerimeterGeneratorArachneExtrusion { extrusion, is_contour: *is_contour, fuzzify: false })
                    .collect();
                // Fuzzy skin is off in every case of the generator (its random source cannot be
                // repeated) and there are no perimeter regions: FuzzySkin.cpp:241-247 hands the
                // extrusion back as it is.
                let coll = traverse_extrusions(&members, &mut loop_nodes, &mut pg_extrusions, &|extrusion, _, _, _, _| extrusion.clone());
                format!("{} {}", coll_s(&coll), nodes_s(&loop_nodes))
            }
            other => panic!("unknown line {other}"),
        };
        lines.push(format!("{head} => {result}"));
    }

    if let Ok(path) = std::env::var("PERIMETER_GENERATOR_PARITY_DUMP") {
        let _ = std::fs::write(path, lines.join("\n"));
    }
    let mut mismatches = 0usize;
    let mut by_kind: std::collections::BTreeMap<String, (usize, usize)> = std::collections::BTreeMap::new();
    for (k, (e, l)) in expected.iter().zip(lines.iter()).enumerate() {
        let kind = e.split(|c| c == ' ' || c == '\t').next().unwrap_or("").to_string();
        let entry = by_kind.entry(kind).or_insert((0, 0));
        entry.0 += 1;
        if e != l {
            entry.1 += 1;
            mismatches += 1;
            if mismatches <= 12 {
                let cut = |s: &str| {
                    let r = s.split_once(" => ").map(|x| x.1).unwrap_or(s);
                    if r.len() > 600 {
                        format!("{}...", &r[..600])
                    } else {
                        r.to_string()
                    }
                };
                let head = e.split_once(" => ").map(|x| x.0).unwrap_or(e);
                let head = if head.len() > 300 { format!("{}...", &head[..300]) } else { head.to_string() };
                eprintln!("line {} {}:\n  C++:  {}\n  Rust: {}", k + 1, head, cut(e), cut(l));
            }
        }
    }
    for (kind, (n, bad)) in &by_kind {
        eprintln!("{kind}: {n} lines, {bad} differ");
    }
    eprintln!("perimeter_generator parity: {} lines, {} mismatches", expected.len(), mismatches);
    assert_eq!(lines.len(), expected.len());
    assert_eq!(mismatches, 0, "{mismatches} of {} lines differ from the pin", expected.len());
}
