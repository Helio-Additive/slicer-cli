//! `group_fills` of Fill.cpp at the pin against the two versions of this crate, on vectors made by the
//! C++ (tests/data/fill/gen.cpp: a real Print, layers with fill surfaces set by hand, 20 configurations
//! x 12 layers).
//!
//! For the line-by-line port (`slicer::fill::fill::group_fills`, fill/fill.rs) the test REQUIRES that
//! every line comes out as the pin's list of SurfaceFill, in order (240 of 240 since R925, when
//! Fill.cpp:227-231 `starts_at_object_base` and 303-329, the flows of a thick bridge and of a merged
//! span, were ported). For the live grouping (`slicer::fill::group_fills`, fill/mod.rs) it REPORTS:
//! how many lines are equal and where the first difference is, with a floor (201 of 240 with overlap
//! and angle masked, which the live parameters hold as a ratio and in degrees): THE LIVE VERSION IS
//! NOT EQUAL TO THE PIN.
//! The pattern of a fill is left out of the comparison (the two versions hold it in enums of their own
//! order); it is reported by name. The vectors do not reach stSubTop surfaces, the conformal infill
//! parameters, `initial_layer_flow_ratio` or an object on a raft (object base above 0).
//! `FILL_PARITY_VECTORS` names a file of the generator to read instead of the committed one,
//! `FILL_PARITY_SHOW` the number of differing lines to print per version.

#[allow(unused_imports)]
use slicer::config::ConfigBase;
use slicer::geometry::{ExPolygon, ExPolygons, Point, Polygon};
use slicer::layer::{Layer, LayerRegion};
use slicer::surface::{Surface, SurfaceType};
use std::io::Read;

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

struct Tok<'a> {
    it: std::str::Split<'a, char>,
}
impl<'a> Tok<'a> {
    fn new(s: &'a str) -> Self {
        Tok { it: s.split(' ') }
    }
    fn s(&mut self) -> &'a str {
        self.it.next().expect("token")
    }
    fn i(&mut self) -> i64 {
        self.s().parse().expect("integer")
    }
    fn d(&mut self) -> f64 {
        f64::from_bits(u64::from_str_radix(self.s(), 16).expect("double"))
    }
    fn b(&mut self) -> bool {
        self.s() == "1"
    }
    fn poly(&mut self) -> Polygon {
        let n = self.i();
        let mut points = Vec::new();
        for _ in 0..n {
            let x = self.i();
            let y = self.i();
            points.push(Point::new(x, y));
        }
        Polygon { points }
    }
    fn expoly(&mut self) -> ExPolygon {
        let contour = self.poly();
        let n = self.i();
        let holes = (0..n).map(|_| self.poly()).collect();
        ExPolygon { contour, holes }
    }
    fn expolys(&mut self) -> ExPolygons {
        let n = self.i();
        (0..n).map(|_| self.expoly()).collect()
    }
}

fn read_vectors() -> Vec<String> {
    if let Ok(path) = std::env::var("FILL_PARITY_VECTORS") {
        let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
        return text.lines().map(|l| l.to_string()).collect();
    }
    let path = format!("{}/tests/data/fill/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).unwrap_or_else(|e| panic!("{path}: {e}"));
    text.lines().map(|l| l.to_string()).collect()
}

// Surface.hpp:9-30
fn surface_type(v: i64) -> SurfaceType {
    match v {
        0 => SurfaceType::Top,
        1 => SurfaceType::Bottom,
        2 => SurfaceType::BottomBridge,
        3 => SurfaceType::Internal,
        4 => SurfaceType::FloatingVerticalShell,
        5 => SurfaceType::InternalSolid,
        6 => SurfaceType::SubTop,
        7 => SurfaceType::InternalBridge,
        8 => SurfaceType::InternalVoid,
        _ => panic!("surface type {v}"),
    }
}

// The names of InfillPattern in the order of PrintConfig.hpp:89-95.
const PATTERNS: [&str; 29] = [
    "Concentric", "Rectilinear", "Grid", "Line", "Cubic", "Triangles", "Stars", "Gyroid", "Honeycomb", "AdaptiveCubic", "Monotonic", "MonotonicLine", "AlignedRectilinear",
    "3DHoneycomb", "HilbertCurve", "ArchimedeanChords", "OctagramSpiral", "SupportCubic", "SupportBase", "ConcentricInternal", "Lightning", "CrossHatch", "ZigZag", "CrossZag",
    "FloatingConcentric", "LockedZag", "2DLattice", "IroningArchimedeanSpiral", "GlobalMonotonicLine",
];

// One SurfaceFill as the generator prints it, the pattern replaced by its name behind a '@'.
macro_rules! fill_s {
    ($f:expr) => {{
        let f = $f;
        let p = &f.params;
        let mut o = format!(
            "{} {} {} {} @{:?} {} {} {:08x} {} {:08x} {:08x} {} {:08x} {:08x} {:08x} {:08x} {:08x} {:08x} {} {} {}",
            if f.region_id == usize::MAX { -1 } else { f.region_id as i64 },
            f.surface.surface_type as i64,
            hd(f.surface.thickness),
            p.extruder,
            p.pattern,
            hd(p.spacing),
            hd(p.overlap),
            p.angle.to_bits(),
            hb(p.bridge),
            p.bridge_angle.to_bits(),
            p.density.to_bits(),
            p.multiline,
            p.anchor_length.to_bits(),
            p.anchor_length_max.to_bits(),
            (p.flow.width() as f32).to_bits(),
            (p.flow.height() as f32).to_bits(),
            (p.flow.nozzle_diameter() as f32).to_bits(),
            (p.flow.spacing() as f32).to_bits(),
            hb(p.flow.is_bridge()),
            p.extrusion_role as i64,
            f.region_id_group.len()
        );
        for r in &f.region_id_group {
            o += &format!(" {r}");
        }
        o += &format!(" {} {}", expolys_s(&f.expolygons), expolys_s(&f.no_overlap_expolygons));
        o
    }};
}

const FIELDS: [&str; 21] = [
    "region_id", "surface_type", "thickness", "extruder", "pattern", "spacing", "overlap", "angle", "bridge", "bridge_angle", "density", "multiline", "anchor_length",
    "anchor_length_max", "flow.width", "flow.height", "flow.nozzle", "flow.spacing", "flow.bridge", "extrusion_role", "region_id_group",
];

#[derive(Default)]
struct Stat {
    lines: usize,
    failed: usize,
    same_count: usize,
    equal_in_order: usize,
    equal_as_set: usize,
    pattern_names: std::collections::BTreeMap<String, usize>,
    first_field: std::collections::BTreeMap<String, usize>,
    shown: usize,
}

/// Compares the fills of one version with the pin's of one line.
fn compare(stat: &mut Stat, what: &str, line_no: usize, tag: &str, theirs: &[String], mine: Option<Vec<String>>, show: usize) {
    stat.lines += 1;
    let Some(mine) = mine else {
        stat.failed += 1;
        return;
    };
    // The pattern apart: "@name" of the pin against the Debug name of this crate, counted by pair.
    let strip = |fills: &[String]| -> Vec<String> { fills.iter().map(|f| f.split(' ').map(|t| if t.starts_with('@') { "@" } else { t }).collect::<Vec<_>>().join(" ")).collect() };
    for (a, b) in theirs.iter().zip(mine.iter()) {
        let pa = a.split(' ').nth(4).unwrap_or("?");
        let pb = b.split(' ').nth(4).unwrap_or("?");
        *stat.pattern_names.entry(format!("pin {pa} / rust {pb}")).or_insert(0) += 1;
    }
    let (a, b) = (strip(theirs), strip(&mine));
    if a.len() == b.len() {
        stat.same_count += 1;
    }
    if a == b {
        stat.equal_in_order += 1;
        stat.equal_as_set += 1;
        return;
    }
    let (mut sa, mut sb) = (a.clone(), b.clone());
    sa.sort();
    sb.sort();
    if sa == sb {
        stat.equal_as_set += 1;
    }
    // The first fill that differs and the first field of it.
    let k = a.iter().zip(b.iter()).position(|(x, y)| x != y).unwrap_or(a.len().min(b.len()));
    let field = match (a.get(k), b.get(k)) {
        (Some(x), Some(y)) => {
            let (tx, ty): (Vec<&str>, Vec<&str>) = (x.split(' ').collect(), y.split(' ').collect());
            let at = tx.iter().zip(ty.iter()).position(|(p, q)| p != q).unwrap_or(tx.len().min(ty.len()));
            if at < FIELDS.len() { FIELDS[at].to_string() } else { "expolygons".to_string() }
        }
        _ => "number of fills".to_string(),
    };
    if stat.shown < show {
        stat.shown += 1;
        eprintln!("{what}: line {} ({tag}): {} fills of the pin, {} here; first difference in fill {k}: {field}\n    pin:  {}\n    rust: {}", line_no + 1, a.len(), b.len(), theirs.get(k).map(|s| &s[..s.len().min(300)]).unwrap_or("-"), mine.get(k).map(|s| &s[..s.len().min(300)]).unwrap_or("-"));
    }
    *stat.first_field.entry(field).or_insert(0) += 1;
}

#[test]
fn fill_group_fills_native_parity_report() {
    let expected = read_vectors();
    let show: usize = std::env::var("FILL_PARITY_SHOW").ok().and_then(|v| v.parse().ok()).unwrap_or(2);
    let mut cfg: Option<(String, std::sync::Arc<slicer::print_config::PrintConfig>, std::sync::Arc<slicer::print_config::PrintObjectConfig>, std::sync::Arc<slicer::print_region::PrintRegion>)> = None;
    let (mut live, mut live_masked, mut port, mut port_areas) = (Stat::default(), Stat::default(), Stat::default(), Stat::default());
    let mut cases = 0usize;
    for (line_no, line) in expected.iter().enumerate() {
        if let Some(rest) = line.strip_prefix("gcfg\t") {
            let fields: Vec<&str> = rest.split('\t').collect();
            let mut print_config = slicer::print_config::PrintConfig::default();
            let mut object_config = slicer::print_config::PrintObjectConfig::default();
            let mut region_config = slicer::region_config::PrintRegionConfig::default();
            for f in &fields[1..] {
                let (key, value) = f.split_once('=').expect("key=value");
                // The first value of the options with a value per extruder, as the slicing path hands over.
                let first = if key == "nozzle_diameter" { value.split(',').next().unwrap_or(value) } else { value };
                print_config.set_deserialize(key, first);
                // PrintObjectConfig::set_deserialize knows a part of the keys only (not `thick_bridges`);
                // the rest goes through apply_key_value, as the slicing path's config loading does.
                if !object_config.set_deserialize(key, first) {
                    object_config.apply_key_value(key, first);
                }
                region_config.set_deserialize(key, first);
            }
            cfg = Some((fields[0].to_string(), std::sync::Arc::new(print_config), std::sync::Arc::new(object_config), std::sync::Arc::new(slicer::print_region::PrintRegion::new(region_config))));
            continue;
        }
        // The "mf" lines belong to `fill_make_fills_native_parity_report` below.
        if line.starts_with("mf ") {
            continue;
        }
        let Some(rest) = line.strip_prefix("gf ") else { panic!("unknown line {}", line_no + 1) };
        let (tag, print_config, object_config, region) = cfg.as_ref().expect("gcfg");
        let (inputs, results) = rest.split_once(" => ").expect("results");
        cases += 1;
        let mut t = Tok::new(inputs);
        let layer_id = t.i() as usize;
        let height = t.d();
        let print_z = t.d();
        let slice_z = t.d();
        let n = t.i();
        let mut layer = Layer::new(layer_id, 0, height, print_z, slice_z);
        let mut layerm = LayerRegion::new(layer_id, 0);
        layerm.set_config_hierarchy(region.clone(), object_config.clone(), print_config.clone());
        for _ in 0..n {
            let ty = surface_type(t.i());
            let thickness = t.d();
            let bridge_angle = t.d();
            let mut s = Surface::new(ty, t.expoly());
            s.thickness = thickness;
            s.bridge_angle = if bridge_angle >= 0. { Some(bridge_angle) } else { None };
            layerm.fill_surfaces.surfaces.push(s);
        }
        layer.add_region(layerm);
        layer.set_object_configs(object_config.clone(), print_config.clone());
        let has_lower = t.b();
        let lower_internal = t.expolys();
        let mut lower = Layer::new(layer_id.saturating_sub(1), 0, height, print_z - height, slice_z - height);
        let mut lower_m = LayerRegion::new(layer_id.saturating_sub(1), 0);
        lower_m.set_config_hierarchy(region.clone(), object_config.clone(), print_config.clone());
        for e in &lower_internal {
            lower_m.fill_surfaces.surfaces.push(Surface::new(SurfaceType::Internal, e.clone()));
        }
        lower.add_region(lower_m);
        lower.set_object_configs(object_config.clone(), print_config.clone());

        // The pin's list: the count, then one piece per fill; its pattern token becomes "@name".
        let mut pieces = results.split(" F ");
        let count: usize = pieces.next().expect("count").trim().parse().expect("count");
        let theirs: Vec<String> = pieces
            .map(|p| {
                let mut toks: Vec<String> = p.split(' ').map(|s| s.to_string()).collect();
                let v: usize = toks[4].parse().expect("pattern");
                toks[4] = format!("@{}", PATTERNS.get(v).copied().unwrap_or("?"));
                toks.join(" ")
            })
            .collect();
        assert_eq!(theirs.len(), count, "line {}", line_no + 1);

        // The live grouping (fill/mod.rs): the areas of sparse infill of the layer below are handed over.
        let mine_live = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let mut lock_param = slicer::fill::LockRegionParam::default();
            let areas: &[ExPolygon] = if has_lower { &lower_internal } else { &[] };
            slicer::fill::group_fills(&layer, areas, &mut lock_param).ok().map(|fills| fills.iter().map(|f| fill_s!(f)).collect::<Vec<String>>())
        }))
        .unwrap_or(None);
        // The live params hold the overlap as the ratio of the option and the angle in degrees where the
        // pin's hold 0 and radians: a second row with both tokens masked shows what else differs.
        let mask = |fills: &[String]| -> Vec<String> {
            fills.iter().map(|f| f.split(' ').enumerate().map(|(k, t)| if k == 6 || k == 7 { "~" } else { t }).collect::<Vec<_>>().join(" ")).collect()
        };
        compare(&mut live_masked, "live fill/mod.rs, overlap and angle aside", line_no, tag, &mask(&theirs), mine_live.as_ref().map(|m| mask(m)), show);
        compare(&mut live, "live fill/mod.rs", line_no, tag, &theirs, mine_live, 0);
        // The line-by-line port (fill/fill.rs).
        let mine_port = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let mut lock_param = slicer::fill::LockRegionParam::default();
            slicer::fill::fill::group_fills(&layer, if has_lower { Some(&lower) } else { None }, &mut lock_param).ok().map(|fills| fills.iter().map(|f| fill_s!(f)).collect::<Vec<String>>())
        }))
        .unwrap_or(None);
        compare(&mut port, "port fill/fill.rs", line_no, tag, &theirs, mine_port, show);
        // The same port handed the areas of sparse infill of the layer below, as Layer::make_fills holds them.
        let mine_port_areas = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let mut lock_param = slicer::fill::LockRegionParam::default();
            let areas: &[ExPolygon] = if has_lower { &lower_internal } else { &[] };
            slicer::fill::fill::group_fills_with_lower_internal_areas(&layer, areas, &mut lock_param, 0.).ok().map(|fills| fills.iter().map(|f| fill_s!(f)).collect::<Vec<String>>())
        }))
        .unwrap_or(None);
        compare(&mut port_areas, "port fill/fill.rs, lower areas handed over", line_no, tag, &theirs, mine_port_areas, show);
    }
    for (what, s) in [("live fill/mod.rs group_fills", &live), ("live fill/mod.rs group_fills, overlap and angle aside", &live_masked), ("port fill/fill.rs group_fills", &port), ("port fill/fill.rs group_fills_with_lower_internal_areas", &port_areas)] {
        eprintln!(
            "{what}: {} lines; error or panic {}; same number of fills {}; equal to the pin in order {} (pattern aside); equal as a set {}",
            s.lines, s.failed, s.same_count, s.equal_in_order, s.equal_as_set
        );
        eprintln!("    first differing field: {:?}", s.first_field);
        eprintln!("    patterns (pin / rust, by position): {:?}", s.pattern_names);
    }
    // A report: what is required is that the vectors were read and both versions ran on every line.
    assert_eq!(cases, 252, "the vectors hold 21 configurations x 12 layers");
    assert_eq!(live.lines, cases);
    assert_eq!(port.lines, cases);
    // Floors, not equality: the numbers of lines equal to the pin in order when this report was written
    // (R924). A change that lowers one moved a grouping away from the pin; raise them as they grow, and
    // turn them into `== cases` once a version is equal.
    assert_eq!(live.failed + port.failed, 0, "a group_fills returned an error or panicked");
    // R925: the line-by-line port is equal to the pin on every line (Fill.cpp:227-231 and 303-329 ported).
    assert_eq!(port.equal_in_order, cases, "port fill/fill.rs group_fills: {} of {} lines equal to the pin", port.equal_in_order, cases);
    assert_eq!(port_areas.equal_in_order, cases, "port fill/fill.rs group_fills_with_lower_internal_areas: {} of {} lines equal to the pin", port_areas.equal_in_order, cases);
    assert!(live_masked.equal_in_order >= 201, "live fill/mod.rs group_fills: {} lines equal to the pin, 201 before", live_masked.equal_in_order);
}


// ---------------------------------------------------------------------------------------------
// Layer::make_fills (Fill.cpp:678-867): the `fills` of every region after the pin's make_fills on
// the layer of each "gf" line ("mf" lines of the generator), against the LIVE
// `Layer::make_fills_at_object_base` of this crate on the same inputs. A REPORT with floors: the
// live assembly is NOT equal to the pin on every line; the counts below say how far it is.

use slicer::extrusion_entity::{ExtrusionEntityType, ExtrusionPath};

fn path_s(p: &ExtrusionPath) -> String {
    let mut o = format!(
        "P {} {:08x} {:08x} {} {}",
        slicer::extrusion_entity::role_to_string(p.role).replace(' ', "_"),
        (p.width as f32).to_bits(),
        (p.height as f32).to_bits(),
        hd(p.mm3_per_mm),
        p.polyline.points.len()
    );
    for q in &p.polyline.points {
        o += &format!(" {}", pt(q));
    }
    o
}
// gen.cpp `ENT`.
fn ent_s(e: &ExtrusionEntityType) -> String {
    match e {
        ExtrusionEntityType::Collection(c) => {
            let mut o = format!("C {} {}", hb(c.no_sort), c.entities.len());
            for x in &c.entities {
                o += &format!(" {}", ent_s(x));
            }
            o
        }
        ExtrusionEntityType::Loop(l) => {
            let mut o = format!("L {}", l.paths.len());
            for x in &l.paths {
                o += &format!(" {}", path_s(x));
            }
            o
        }
        ExtrusionEntityType::MultiPath(m) => {
            let mut o = format!("M {}", m.paths.len());
            for x in &m.paths {
                o += &format!(" {}", path_s(x));
            }
            o
        }
        ExtrusionEntityType::Path(p) => path_s(p),
    }
}
// One entity of the pin's line, as its tokens joined; `k` moves behind it.
fn take_entity(t: &[&str], k: &mut usize) -> String {
    let start = *k;
    fn walk(t: &[&str], k: &mut usize) {
        let kind = t[*k];
        *k += 1;
        match kind {
            "C" => {
                let n: usize = t[*k + 1].parse().expect("count");
                *k += 2;
                for _ in 0..n {
                    walk(t, k);
                }
            }
            "L" | "M" => {
                let n: usize = t[*k].parse().expect("count");
                *k += 1;
                for _ in 0..n {
                    walk(t, k);
                }
            }
            "P" => {
                let n: usize = t[*k + 4].parse().expect("points");
                *k += 5 + 2 * n;
            }
            other => panic!("entity kind {other}"),
        }
    }
    walk(t, k);
    t[start..*k].join(" ")
}
// The paths of an entity without their points: kind, role, width, height, mm3_per_mm (what the
// conversion of polylines into extrusions decides), the number of points aside.
fn headers(e: &str) -> Vec<String> {
    let t: Vec<&str> = e.split(' ').collect();
    let mut o = Vec::new();
    let mut k = 0;
    while k < t.len() {
        match t[k] {
            "C" => {
                o.push(format!("C {} {}", t[k + 1], t[k + 2]));
                k += 3;
            }
            "L" | "M" => {
                o.push(format!("{} {}", t[k], t[k + 1]));
                k += 2;
            }
            "P" => {
                o.push(t[k..k + 5].join(" "));
                let n: usize = t[k + 5].parse().expect("points");
                k += 6 + 2 * n;
            }
            other => panic!("entity token {other}"),
        }
    }
    o
}
// The role of the first path of an entity.
fn first_role(e: &str) -> String {
    let t: Vec<&str> = e.split(' ').collect();
    t.iter().position(|x| *x == "P").and_then(|k| t.get(k + 1)).unwrap_or(&"-").to_string()
}

#[test]
fn fill_make_fills_native_parity_report() {
    let expected = read_vectors();
    let show: usize = std::env::var("FILL_PARITY_SHOW").ok().and_then(|v| v.parse().ok()).unwrap_or(2);
    let mut cfg: Option<(String, std::sync::Arc<slicer::print_config::PrintConfig>, std::sync::Arc<slicer::print_config::PrintObjectConfig>, std::sync::Arc<slicer::print_region::PrintRegion>)> = None;
    let mut last_gf: Option<&str> = None;
    // Per configuration, in the generator's order: lines, lines equal, entities of the pin, entities equal.
    let mut per_cfg: Vec<(String, usize, usize, usize, usize)> = Vec::new();
    let (mut lines, mut failed, mut same_count, mut equal_lines, mut entities, mut equal_entities, mut equal_headers) = (0usize, 0usize, 0usize, 0usize, 0usize, 0usize, 0usize);
    // By the role of an entity's first path: entities of the pin, equal here, equal points aside.
    let mut by_role: std::collections::BTreeMap<String, (usize, usize, usize)> = std::collections::BTreeMap::new();
    let mut shown = 0usize;
    for (line_no, line) in expected.iter().enumerate() {
        if let Some(rest) = line.strip_prefix("gcfg\t") {
            let fields: Vec<&str> = rest.split('\t').collect();
            let mut print_config = slicer::print_config::PrintConfig::default();
            let mut object_config = slicer::print_config::PrintObjectConfig::default();
            let mut region_config = slicer::region_config::PrintRegionConfig::default();
            for f in &fields[1..] {
                let (key, value) = f.split_once('=').expect("key=value");
                let first = if key == "nozzle_diameter" { value.split(',').next().unwrap_or(value) } else { value };
                print_config.set_deserialize(key, first);
                if !object_config.set_deserialize(key, first) {
                    object_config.apply_key_value(key, first);
                }
                region_config.set_deserialize(key, first);
            }
            cfg = Some((fields[0].to_string(), std::sync::Arc::new(print_config), std::sync::Arc::new(object_config), std::sync::Arc::new(slicer::print_region::PrintRegion::new(region_config))));
            per_cfg.push((fields[0].to_string(), 0, 0, 0, 0));
            continue;
        }
        if let Some(rest) = line.strip_prefix("gf ") {
            last_gf = Some(rest);
            continue;
        }
        let Some(rest) = line.strip_prefix("mf ") else { panic!("unknown line {}", line_no + 1) };
        let (tag, print_config, object_config, region) = cfg.as_ref().expect("gcfg");
        let (inputs, _) = last_gf.expect("gf before mf").split_once(" => ").expect("results");
        lines += 1;
        // The layer of the "gf" line, as the test above builds it.
        let mut t = Tok::new(inputs);
        let layer_id = t.i() as usize;
        let height = t.d();
        let print_z = t.d();
        let slice_z = t.d();
        let n = t.i();
        let mut layer = Layer::new(layer_id, 0, height, print_z, slice_z);
        let mut layerm = LayerRegion::new(layer_id, 0);
        layerm.set_config_hierarchy(region.clone(), object_config.clone(), print_config.clone());
        for _ in 0..n {
            let ty = surface_type(t.i());
            let thickness = t.d();
            let bridge_angle = t.d();
            let mut s = Surface::new(ty, t.expoly());
            s.thickness = thickness;
            s.bridge_angle = if bridge_angle >= 0. { Some(bridge_angle) } else { None };
            layerm.fill_surfaces.surfaces.push(s);
        }
        layer.add_region(layerm);
        layer.set_object_configs(object_config.clone(), print_config.clone());
        let has_lower = t.b();
        let lower_internal = t.expolys();

        // The pin's line: the object's box, then per region the entities of `fills`.
        let toks: Vec<&str> = rest.split(' ').collect();
        let mut k = 0usize;
        let bbox = if toks[k] == "1" {
            let v: Vec<i64> = toks[k + 1..k + 5].iter().map(|s| s.parse().expect("box")).collect();
            k += 5;
            Some(slicer::geometry::BoundingBox::from_points_minmax(Point::new(v[0], v[1]), Point::new(v[2], v[3])))
        } else {
            k += 1;
            None
        };
        layer.set_object_bounding_box(bbox);
        let regions: usize = toks[k].parse().expect("regions");
        k += 1;
        let mut theirs: Vec<Vec<String>> = Vec::new();
        for _ in 0..regions {
            assert_eq!(toks[k], "R", "line {}", line_no + 1);
            let count: usize = toks[k + 1].parse().expect("entities");
            k += 2;
            theirs.push((0..count).map(|_| take_entity(&toks, &mut k)).collect());
        }
        assert_eq!(k, toks.len(), "line {}: tokens left", line_no + 1);

        // The live assembly: the areas of sparse infill of the layer below handed over as
        // print_object.rs does (Fill.cpp:457-462), the object's base at 0.
        let mine: Option<Vec<Vec<String>>> = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let areas: &[ExPolygon] = if has_lower { &lower_internal } else { &[] };
            layer.make_fills_at_object_base(areas, &[], None, 0.).ok().map(|_| layer.regions().iter().map(|r| r.fills.entities.iter().map(ent_s).collect()).collect())
        }))
        .unwrap_or(None);
        let row = per_cfg.last_mut().expect("gcfg");
        row.1 += 1;
        let pin_entities: usize = theirs.iter().map(|r| r.len()).sum();
        entities += pin_entities;
        row.3 += pin_entities;
        let Some(mine) = mine else {
            failed += 1;
            for e in theirs.iter().flatten() {
                by_role.entry(first_role(e)).or_insert((0, 0, 0)).0 += 1;
            }
            continue;
        };
        if mine.iter().map(|r| r.len()).collect::<Vec<_>>() == theirs.iter().map(|r| r.len()).collect::<Vec<_>>() {
            same_count += 1;
        }
        if mine == theirs {
            equal_lines += 1;
            row.2 += 1;
        }
        let empty: Vec<String> = Vec::new();
        for (r, region_theirs) in theirs.iter().enumerate() {
            let region_mine = mine.get(r).unwrap_or(&empty);
            for (i, e) in region_theirs.iter().enumerate() {
                let slot = by_role.entry(first_role(e)).or_insert((0, 0, 0));
                slot.0 += 1;
                let m = region_mine.get(i);
                if m == Some(e) {
                    slot.1 += 1;
                    equal_entities += 1;
                    row.4 += 1;
                }
                if m.map(|m| headers(m)) == Some(headers(e)) {
                    slot.2 += 1;
                    equal_headers += 1;
                }
                if m != Some(e) && shown < show {
                    shown += 1;
                    eprintln!("make_fills: line {} ({tag}), region {r}, entity {i} of {} (here {}):\n    pin:  {}\n    rust: {}", line_no + 1, region_theirs.len(), region_mine.len(), &e[..e.len().min(400)], m.map(|s| &s[..s.len().min(400)]).unwrap_or("-"));
                }
            }
            // The entities the live assembly lays beyond the pin's (R929: top_monotonicline).
            for (i, m) in region_mine.iter().enumerate().skip(region_theirs.len()) {
                if shown < show {
                    shown += 1;
                    eprintln!("make_fills: line {} ({tag}), region {r}, entity {i} here, none of the pin ({} entities):\n    rust: {}", line_no + 1, region_theirs.len(), &m[..m.len().min(400)]);
                }
            }
        }
    }
    eprintln!("live Layer::make_fills: {lines} lines; error or panic {failed}; same number of entities per region {same_count}; lines equal to the pin {equal_lines}");
    eprintln!("    entities of the pin {entities}; equal here by position {equal_entities}; equal in kind, role, width, height and mm3_per_mm of every path (points aside) {equal_headers}");
    eprintln!("    by the role of the first path (entities of the pin, equal, equal points aside): {by_role:?}");
    for (tag, n, eq, ents, eq_ents) in &per_cfg {
        eprintln!("    {tag}: {eq} of {n} lines equal; {eq_ents} of {ents} entities");
    }
    assert_eq!(lines, 252, "the vectors hold 21 configurations x 12 layers");
    // Floors, not equality (R927, when this report was written): raise them as the live assembly
    // gets closer to the pin, and turn them into equality once it is there.
    assert!(failed <= FLOOR_FAILED, "live make_fills: {failed} lines with an error or a panic, {FLOOR_FAILED} before");
    assert!(equal_lines >= FLOOR_LINES, "live make_fills: {equal_lines} lines equal to the pin, {FLOOR_LINES} before");
    assert!(equal_entities >= FLOOR_ENTITIES, "live make_fills: {equal_entities} entities equal to the pin, {FLOOR_ENTITIES} before");
    assert!(equal_headers >= FLOOR_HEADERS, "live make_fills: {equal_headers} entities equal to the pin points aside, {FLOOR_HEADERS} before");
    // The configurations whose 12 lines are all equal to the pin (R928, with FILL_MULTILINES_PIN:
    // FillCubic where the live assembly laid a grid, and FILL_MULTILINES_ONE_LINE: one line alone
    // gets no anchors; R929, with FILL_CONCENTRIC_PIN: FillConcentric through
    // Fill::fill_surface_extrusion for a configured ipConcentric, and FILL_MONOLINE_NO_OVERLAP:
    // no monotonic line where the filler has no no_overlap_expolygons) are REQUIRED to stay equal.
    for (tag, n, eq, _, _) in &per_cfg {
        if EQUAL_CONFIGURATIONS.contains(&tag.as_str()) {
            assert_eq!(eq, n, "live make_fills, configuration {tag}: {eq} of {n} lines equal to the pin, all of them before");
        }
    }
}
const EQUAL_CONFIGURATIONS: [&str; 21] =
    ["base", "grid", "gyroid_25", "line_40", "honeycomb", "dense_100", "no_infill", "top_concentric", "top_monotonicline", "bottom_concentric", "solid_concentric", "solid_rectilinear", "direction_30", "no_narrow", "thick_bridges", "bridge_flow_09", "layer_02", "widths", "overlap_30", "bridge_angle_45", "sub_top"];
// The counts of the report (first run R927: 103 / 512 / 604; R928 with FILL_PATH_HEIGHT_PIN and
// FILL_MULTILINES_PIN: 217 / 700 / 715; with FILL_MULTILINES_ONE_LINE: 223 / 706 / 715;
// R929 with FILL_CONCENTRIC_PIN: 235 / 746 / 755; with FILL_MONOLINE_NO_OVERLAP: 241 / 746 / 755;
// R931 with FILL_LINE_PIN: 245 / 750 / 755; with FILL_GYROID_SPACING_PIN: 247 / 752 / 755; with
// FILL_HONEYCOMB_PIN: 252 / 758 / 758, every line of the pin): lines with an error or a panic at most; equal lines, equal
// entities, entities equal points aside at least.
const FLOOR_FAILED: usize = 0;
const FLOOR_LINES: usize = 252;
const FLOOR_ENTITIES: usize = 758;
const FLOOR_HEADERS: usize = 758;

// Layer::make_ironing (Fill.cpp:983-1250): what the pin's make_ironing appends to the empty `fills` of
// each region of a layer whose slices, fill surfaces, lslices and upper layer are set by hand ("il"
// lines of tests/data/fill/vectors_ironing.txt.gz, 11 configurations x 8 layers: ironing_type top /
// topmost / solid, dense sparse infill, spacing and flow, inset, direction, the concentric pattern,
// no ironing, no top or bottom shells), against `Layer::make_ironing` of fill/fill.rs on the same inputs.
// `FILL_IRONING_VECTORS` names a file of the generator to read instead of the committed one.
fn read_ironing_vectors() -> Vec<String> {
    if let Ok(path) = std::env::var("FILL_IRONING_VECTORS") {
        let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
        return text.lines().map(|l| l.to_string()).collect();
    }
    let path = format!("{}/tests/data/fill/vectors_ironing.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).unwrap_or_else(|e| panic!("{path}: {e}"));
    text.lines().map(|l| l.to_string()).collect()
}

#[test]
fn fill_make_ironing_native_parity() {
    let expected = read_ironing_vectors();
    let show: usize = std::env::var("FILL_PARITY_SHOW").ok().and_then(|v| v.parse().ok()).unwrap_or(2);
    let mut cfg: Option<(String, std::sync::Arc<slicer::print_config::PrintConfig>, std::sync::Arc<slicer::print_config::PrintObjectConfig>, std::sync::Arc<slicer::print_region::PrintRegion>)> = None;
    let (mut lines, mut equal_lines, mut entities, mut equal_entities, mut nonempty) = (0usize, 0usize, 0usize, 0usize, 0usize);
    let mut per_cfg: Vec<(String, usize, usize)> = Vec::new();
    let mut shown = 0usize;
    for (line_no, line) in expected.iter().enumerate() {
        if let Some(rest) = line.strip_prefix("ic\t") {
            let fields: Vec<&str> = rest.split('\t').collect();
            let mut print_config = slicer::print_config::PrintConfig::default();
            let mut object_config = slicer::print_config::PrintObjectConfig::default();
            let mut region_config = slicer::region_config::PrintRegionConfig::default();
            for f in &fields[1..] {
                let (key, value) = f.split_once('=').expect("key=value");
                let first = if key == "nozzle_diameter" { value.split(',').next().unwrap_or(value) } else { value };
                print_config.set_deserialize(key, first);
                if !object_config.set_deserialize(key, first) {
                    object_config.apply_key_value(key, first);
                }
                region_config.set_deserialize(key, first);
            }
            cfg = Some((fields[0].to_string(), std::sync::Arc::new(print_config), std::sync::Arc::new(object_config), std::sync::Arc::new(slicer::print_region::PrintRegion::new(region_config))));
            per_cfg.push((fields[0].to_string(), 0, 0));
            continue;
        }
        let Some(rest) = line.strip_prefix("il ") else { panic!("unknown line {}", line_no + 1) };
        let (tag, print_config, object_config, region) = cfg.as_ref().expect("ic");
        let (inputs, result) = rest.split_once(" => ").expect("results");
        lines += 1;
        let mut t = Tok::new(inputs);
        let layer_id = t.i() as usize;
        let height = t.d();
        let print_z = t.d();
        let slice_z = t.d();
        let has_upper = t.b();
        let mut layer = Layer::new(layer_id, 0, height, print_z, slice_z);
        let mut layerm = LayerRegion::new(layer_id, 0);
        layerm.set_config_hierarchy(region.clone(), object_config.clone(), print_config.clone());
        let n = t.i();
        for _ in 0..n {
            let ty = surface_type(t.i());
            layerm.slices.surfaces.push(Surface::new(ty, t.expoly()));
        }
        let m = t.i();
        for _ in 0..m {
            let ty = surface_type(t.i());
            layerm.fill_surfaces.surfaces.push(Surface::new(ty, t.expoly()));
        }
        layer.lslices = t.expolys();
        layer.add_region(layerm);
        layer.set_object_configs(object_config.clone(), print_config.clone());
        if has_upper {
            layer.set_upper_layer(Some(layer_id + 1));
        }
        let toks: Vec<&str> = result.split(' ').collect();
        let mut k = 0usize;
        let bbox = if toks[k] == "1" {
            let v: Vec<i64> = toks[k + 1..k + 5].iter().map(|s| s.parse().expect("box")).collect();
            k += 5;
            Some(slicer::geometry::BoundingBox::from_points_minmax(Point::new(v[0], v[1]), Point::new(v[2], v[3])))
        } else {
            k += 1;
            None
        };
        layer.set_object_bounding_box(bbox);
        let regions: usize = toks[k].parse().expect("regions");
        k += 1;
        let mut theirs: Vec<Vec<String>> = Vec::new();
        for _ in 0..regions {
            assert_eq!(toks[k], "R", "line {}", line_no + 1);
            let count: usize = toks[k + 1].parse().expect("entities");
            k += 2;
            theirs.push((0..count).map(|_| take_entity(&toks, &mut k)).collect());
        }
        assert_eq!(k, toks.len(), "line {}: tokens left", line_no + 1);
        layer.make_ironing().expect("make_ironing");
        let mine: Vec<Vec<String>> = layer.regions().iter().map(|r| r.fills.entities.iter().map(ent_s).collect()).collect();
        let pin_entities: usize = theirs.iter().map(|r| r.len()).sum();
        entities += pin_entities;
        if pin_entities > 0 {
            nonempty += 1;
        }
        let row = per_cfg.last_mut().expect("ic");
        row.1 += 1;
        for (r, region_theirs) in theirs.iter().enumerate() {
            let region_mine = mine.get(r).cloned().unwrap_or_default();
            for (i, e) in region_theirs.iter().enumerate() {
                if region_mine.get(i) == Some(e) {
                    equal_entities += 1;
                } else if shown < show {
                    shown += 1;
                    let m = region_mine.get(i);
                    eprintln!("make_ironing: line {} ({tag}), region {r}, entity {i} of {} (here {}):\n    pin:  {}\n    rust: {}", line_no + 1, region_theirs.len(), region_mine.len(), &e[..e.len().min(600)], m.map(|s| &s[..s.len().min(600)]).unwrap_or("-"));
                }
            }
        }
        if mine == theirs {
            equal_lines += 1;
            row.2 += 1;
        } else if shown < show && mine.iter().map(|r| r.len()).sum::<usize>() != pin_entities {
            shown += 1;
            eprintln!("make_ironing: line {} ({tag}): pin {} entities, here {}", line_no + 1, pin_entities, mine.iter().map(|r| r.len()).sum::<usize>());
        }
    }
    eprintln!("Layer::make_ironing (fill/fill.rs): {lines} lines ({nonempty} with ironing), {equal_lines} equal to the pin; entities {equal_entities} of {entities} equal");
    for (tag, n, eq) in &per_cfg {
        eprintln!("    {tag}: {eq} of {n} lines equal");
    }
    assert_eq!(lines, 88, "lines of the vectors");
    assert_eq!(equal_lines, lines, "make_ironing: {equal_lines} of {lines} lines equal to the pin");
}

// Layer::generate_sparse_infill_polylines_for_anchoring (Fill.cpp:869-980, no octrees and no lightning
// generator, as Fill.cpp:760 calls it): the polylines the pin lays for the stInternal fills of a layer
// whose fill surfaces are set by hand ("ga" lines of tests/data/fill/vectors_anchoring.txt, 7
// configurations x 8 layers: cubic, triangles, tri-hexagon (stars), cubic at 30% and 30 degrees,
// triangles at 85%, grid, zig-zag), against `Layer::generate_sparse_infill_polylines_for_anchoring_at_object_base`
// of layer.rs on the same inputs (the object's base at 0, its bounding box as the pin's).
fn read_anchoring_vectors() -> Vec<String> {
    let path = std::env::var("FILL_ANCHORING_VECTORS")
        .unwrap_or_else(|_| format!("{}/tests/data/fill/vectors_anchoring.txt", env!("CARGO_MANIFEST_DIR")));
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    text.lines().map(|l| l.to_string()).collect()
}

#[test]
fn fill_anchoring_polylines_native_parity() {
    let expected = read_anchoring_vectors();
    let show: usize = std::env::var("FILL_PARITY_SHOW").ok().and_then(|v| v.parse().ok()).unwrap_or(2);
    let mut cfg: Option<(String, std::sync::Arc<slicer::print_config::PrintConfig>, std::sync::Arc<slicer::print_config::PrintObjectConfig>, std::sync::Arc<slicer::print_region::PrintRegion>)> = None;
    // Per configuration: lines, lines equal.
    let mut per_cfg: Vec<(String, usize, usize)> = Vec::new();
    let (mut lines, mut equal, mut shown) = (0usize, 0usize, 0usize);
    for (line_no, line) in expected.iter().enumerate() {
        if let Some(rest) = line.strip_prefix("acfg\t") {
            let fields: Vec<&str> = rest.split('\t').collect();
            let mut print_config = slicer::print_config::PrintConfig::default();
            let mut object_config = slicer::print_config::PrintObjectConfig::default();
            let mut region_config = slicer::region_config::PrintRegionConfig::default();
            for f in &fields[1..] {
                let (key, value) = f.split_once('=').expect("key=value");
                let first = if key == "nozzle_diameter" { value.split(',').next().unwrap_or(value) } else { value };
                print_config.set_deserialize(key, first);
                if !object_config.set_deserialize(key, first) {
                    object_config.apply_key_value(key, first);
                }
                region_config.set_deserialize(key, first);
            }
            cfg = Some((fields[0].to_string(), std::sync::Arc::new(print_config), std::sync::Arc::new(object_config), std::sync::Arc::new(slicer::print_region::PrintRegion::new(region_config))));
            per_cfg.push((fields[0].to_string(), 0, 0));
            continue;
        }
        let Some(rest) = line.strip_prefix("ga ") else { panic!("unknown line {}", line_no + 1) };
        let (tag, print_config, object_config, region) = cfg.as_ref().expect("acfg");
        let (inputs, result) = rest.split_once(" => ").expect("results");
        lines += 1;
        let mut t = Tok::new(inputs);
        let layer_id = t.i() as usize;
        let height = t.d();
        let print_z = t.d();
        let slice_z = t.d();
        let n = t.i();
        let mut layer = Layer::new(layer_id, 0, height, print_z, slice_z);
        let mut layerm = LayerRegion::new(layer_id, 0);
        layerm.set_config_hierarchy(region.clone(), object_config.clone(), print_config.clone());
        for _ in 0..n {
            let ty = surface_type(t.i());
            let thickness = t.d();
            let bridge_angle = t.d();
            let mut s = Surface::new(ty, t.expoly());
            s.thickness = thickness;
            s.bridge_angle = if bridge_angle >= 0. { Some(bridge_angle) } else { None };
            layerm.fill_surfaces.surfaces.push(s);
        }
        layer.add_region(layerm);
        layer.set_object_configs(object_config.clone(), print_config.clone());
        // The pin's line: the object's box, then the polylines.
        let toks: Vec<&str> = result.split(' ').collect();
        let mut k = 0usize;
        let bbox = if toks[k] == "1" {
            let v: Vec<i64> = toks[k + 1..k + 5].iter().map(|s| s.parse().expect("box")).collect();
            k += 5;
            Some(slicer::geometry::BoundingBox::from_points_minmax(Point::new(v[0], v[1]), Point::new(v[2], v[3])))
        } else {
            k += 1;
            None
        };
        layer.set_object_bounding_box(bbox);
        let count: usize = toks[k].parse().expect("polylines");
        k += 1;
        let mut theirs: Vec<String> = Vec::with_capacity(count);
        for _ in 0..count {
            let np: usize = toks[k].parse().expect("points");
            theirs.push(toks[k..k + 1 + 2 * np].join(" "));
            k += 1 + 2 * np;
        }
        assert_eq!(k, toks.len(), "line {}: tokens left", line_no + 1);
        let mine: Option<Vec<String>> = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            layer.generate_sparse_infill_polylines_for_anchoring_at_object_base(0.).ok().map(|pls| {
                pls.iter()
                    .map(|pl| {
                        let mut s = pl.points().len().to_string();
                        for p in pl.points() {
                            s.push(' ');
                            s.push_str(&pt(p));
                        }
                        s
                    })
                    .collect()
            })
        }))
        .unwrap_or(None);
        let row = per_cfg.last_mut().expect("acfg");
        row.1 += 1;
        if mine.as_ref() == Some(&theirs) {
            equal += 1;
            row.2 += 1;
        } else if shown < show {
            shown += 1;
            let m = mine.as_ref();
            let first = (0..theirs.len().max(m.map_or(0, |m| m.len()))).find(|&i| m.and_then(|m| m.get(i)) != theirs.get(i)).unwrap_or(0);
            eprintln!(
                "anchoring: line {} ({tag}): pin {} polylines, here {:?}; first differing polyline {first}:\n    pin:  {}\n    rust: {}",
                line_no + 1,
                theirs.len(),
                m.map(|m| m.len()),
                theirs.get(first).map(|s| &s[..s.len().min(300)]).unwrap_or("-"),
                m.and_then(|m| m.get(first)).map(|s| &s[..s.len().min(300)]).unwrap_or("-")
            );
        }
    }
    eprintln!("live Layer::generate_sparse_infill_polylines_for_anchoring: {equal} of {lines} lines equal to the pin");
    for (tag, n, eq) in &per_cfg {
        eprintln!("    {tag}: {eq} of {n} lines equal");
    }
    assert_eq!(lines, 56, "the vectors hold 7 configurations x 8 layers");
    assert!(equal >= ANCHORING_FLOOR, "anchoring: {equal} lines equal to the pin, {ANCHORING_FLOOR} before");
}
// Lines equal to the pin (R935, when this test was written: 56 of 56, with FILL_ANCHOR_MULTILINES_PIN and
// FILL_ANCHOR_ONE_LINE_PIN; with both =0 the cubic, triangles and stars lines differ).
const ANCHORING_FLOOR: usize = 56;
