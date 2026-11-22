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
    assert_eq!(cases, 240, "the vectors hold 20 configurations x 12 layers");
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
