//! `ExtrusionPath`, `ExtrusionLoop` and `ExtrusionMultiPath` (ExtrusionEntity.cpp/.hpp) against the C++
//! at the pin: lengths, `min_mm3_per_mm`, `polygon`, `make_clockwise` / `make_counter_clockwise`,
//! `has_overhang_point` / `has_overhang_paths`, `split_at_vertex`, `split_at` (with a given epsilon and with the
//! default `scaled<double>(0.001)`), `clip_end`, `polygons_covered_by_width` / `polygons_covered_by_spacing`
//! and the multi-path's `length`, `min_mm3_per_mm` and `as_polyline`.
//!
//! `tests/data/extrusion_entity/vectors.txt.gz` is the output of `gen.cpp` beside it; the header of gen.cpp
//! describes the lines. Integers have to be equal, doubles and floats bit-equal.

use slicer::extrusion_entity::{
    ExtrusionEntityCollection, ExtrusionEntityType, ExtrusionLoop, ExtrusionLoopRole, ExtrusionMultiPath, ExtrusionPath,
    ExtrusionRole,
};
use slicer::geometry::{Point, Polygon};
use std::collections::BTreeMap;
use std::io::Read;

const ROLES: [ExtrusionRole; 22] = [
    ExtrusionRole::None,
    ExtrusionRole::Perimeter,
    ExtrusionRole::ExternalPerimeter,
    ExtrusionRole::OverhangPerimeter,
    ExtrusionRole::InternalInfill,
    ExtrusionRole::SolidInfill,
    ExtrusionRole::FloatingVerticalShell,
    ExtrusionRole::TopSolidInfill,
    ExtrusionRole::BottomSurface,
    ExtrusionRole::Ironing,
    ExtrusionRole::BridgeInfill,
    ExtrusionRole::GapFill,
    ExtrusionRole::Skirt,
    ExtrusionRole::Brim,
    ExtrusionRole::SupportMaterial,
    ExtrusionRole::SupportMaterialInterface,
    ExtrusionRole::SupportTransition,
    ExtrusionRole::SupportIroning,
    ExtrusionRole::WipeTower,
    ExtrusionRole::Custom,
    ExtrusionRole::Flush,
    ExtrusionRole::Mixed,
];

fn read_vectors() -> String {
    let path = std::env::var("EXTRUSION_ENTITY_VECTORS")
        .unwrap_or_else(|_| format!("{}/tests/data/extrusion_entity/vectors.txt.gz", env!("CARGO_MANIFEST_DIR")));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    if path.ends_with(".gz") {
        flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    } else {
        std::io::BufReader::new(file).read_to_string(&mut text).expect("read");
    }
    text
}

struct Tok<'a> {
    t: Vec<&'a str>,
    k: usize,
}
impl<'a> Tok<'a> {
    fn new(line: &'a str) -> Self {
        Tok { t: line.split_ascii_whitespace().collect(), k: 0 }
    }
    fn s(&mut self) -> &'a str {
        self.k += 1;
        self.t[self.k - 1]
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
    fn pt(&mut self) -> Point {
        let x = self.i();
        let y = self.i();
        Point::new(x, y)
    }
    fn pts(&mut self) -> Vec<Point> {
        let n = self.i() as usize;
        (0..n).map(|_| self.pt()).collect()
    }
    /// `<count>` then per path `<role> <mm3> <points>`.
    fn paths(&mut self) -> Vec<(u8, u64, Vec<Point>)> {
        let n = self.i() as usize;
        (0..n).map(|_| (self.i() as u8, self.d().to_bits(), self.pts())).collect()
    }
    fn polys(&mut self) -> Vec<Vec<Point>> {
        let n = self.i() as usize;
        (0..n).map(|_| self.pts()).collect()
    }
}

/// A path inside a collection entity: `<role> <mm3 double> <width float> <height float> <points>`.
fn read_pathw(t: &mut Tok) -> ExtrusionPath {
    let role = ROLES[t.i() as usize];
    let mm3 = t.d();
    let width = t.f();
    let height = t.f();
    // ExtrusionEntity.hpp:229 (gen.cpp rpath: overhang 0, curve 0)
    let mut path = ExtrusionPath::with_overhang(0.0, 0, role, mm3, width as f64, height as f64);
    *path.polyline.points_mut() = t.pts();
    path
}
fn read_pathsw(t: &mut Tok) -> Vec<ExtrusionPath> {
    let n = t.i() as usize;
    (0..n).map(|_| read_pathw(t)).collect()
}
/// `P <path> | M <paths> | L <paths> | K <no_sort> <count> <entity>*` (gen.cpp pent()).
fn read_entity(t: &mut Tok) -> ExtrusionEntityType {
    match t.s() {
        "P" => ExtrusionEntityType::Path(read_pathw(t)),
        // ExtrusionEntity.hpp:436 `ExtrusionMultiPath(const ExtrusionPaths &paths)`: m_can_reverse stays true
        "M" => ExtrusionEntityType::MultiPath(ExtrusionMultiPath { paths: read_pathsw(t), can_reverse: true }),
        "L" => ExtrusionEntityType::Loop(ExtrusionLoop::new(read_pathsw(t), ExtrusionLoopRole::new())),
        "K" => ExtrusionEntityType::Collection(Box::new(read_collection(t))),
        other => panic!("unknown entity {other}"),
    }
}
fn read_collection(t: &mut Tok) -> ExtrusionEntityCollection {
    let mut c = ExtrusionEntityCollection::new();
    c.no_sort = t.i() != 0;
    let n = t.i() as usize;
    for _ in 0..n {
        c.entities.push(read_entity(t));
    }
    c
}
fn write_pathw(out: &mut Vec<String>, p: &ExtrusionPath) {
    out.push((p.role as u8).to_string());
    out.push(format!("{:016x}", p.mm3_per_mm.to_bits()));
    out.push(format!("{:08x}", (p.width as f32).to_bits()));
    out.push(format!("{:08x}", (p.height as f32).to_bits()));
    out.push(p.polyline.points().len().to_string());
    for q in p.polyline.points() {
        out.push(q.x.to_string());
        out.push(q.y.to_string());
    }
}
fn write_pathsw(out: &mut Vec<String>, paths: &[ExtrusionPath]) {
    out.push(paths.len().to_string());
    for p in paths {
        write_pathw(out, p);
    }
}
fn write_entity(out: &mut Vec<String>, e: &ExtrusionEntityType) {
    match e {
        ExtrusionEntityType::Path(p) => {
            out.push("P".into());
            write_pathw(out, p);
        }
        ExtrusionEntityType::MultiPath(m) => {
            out.push("M".into());
            write_pathsw(out, &m.paths);
        }
        ExtrusionEntityType::Loop(l) => {
            out.push("L".into());
            write_pathsw(out, &l.paths);
        }
        ExtrusionEntityType::Collection(c) => write_collection(out, c),
    }
}
fn write_collection(out: &mut Vec<String>, c: &ExtrusionEntityCollection) {
    out.push("K".into());
    out.push((c.no_sort as u8).to_string());
    out.push(c.entities.len().to_string());
    for e in &c.entities {
        write_entity(out, e);
    }
}
fn collection_tokens(c: &ExtrusionEntityCollection) -> Vec<String> {
    let mut v = Vec::new();
    write_collection(&mut v, c);
    v
}
fn rest(t: &Tok) -> Vec<String> {
    t.t[t.k..].iter().map(|s| s.to_string()).collect()
}

fn paths_of(paths: &[ExtrusionPath]) -> Vec<(u8, u64, Vec<Point>)> {
    paths.iter().map(|p| (p.role as u8, p.mm3_per_mm.to_bits(), p.polyline.points().to_vec())).collect()
}
fn polys_of(polys: &[Polygon]) -> Vec<Vec<Point>> {
    polys.iter().map(|p| p.points().to_vec()).collect()
}

#[test]
fn extrusion_entity_matches_the_pin() {
    let text = read_vectors();
    let mut lines = text.lines().peekable();
    let mut calls: BTreeMap<&'static str, usize> = BTreeMap::new();
    let mut bad: BTreeMap<&'static str, usize> = BTreeMap::new();
    let mut first_bad: Vec<String> = Vec::new();
    let mut cases = 0usize;
    let mut check = |tag: &'static str, ok: bool, case: i64, what: String| {
        *calls.entry(tag).or_default() += 1;
        if !ok {
            *bad.entry(tag).or_default() += 1;
            if first_bad.len() < 12 {
                first_bad.push(format!("case {case} {tag}: {what}"));
            }
        }
    };
    while let Some(line) = lines.next() {
        let mut t = Tok::new(line);
        if line.starts_with("R ") {
            // ExtrusionEntity.hpp:92-135, ExtrusionEntity.cpp:613-689
            t.s();
            let role = ROLES[t.i() as usize];
            let want: Vec<bool> = (0..6).map(|_| t.i() != 0).collect();
            let got = [
                slicer::extrusion_entity::is_perimeter(role),
                slicer::extrusion_entity::is_infill(role),
                slicer::extrusion_entity::is_top_surface(role),
                slicer::extrusion_entity::is_solid_infill(role),
                slicer::extrusion_entity::is_bridge(role),
                slicer::extrusion_entity::is_support(role),
            ];
            check("role helpers", got[..] == want[..], -1, format!("{role:?}"));
            let back = ROLES[t.i() as usize];
            let s = line.splitn(10, ' ').nth(9).unwrap();
            check("role_to_string", slicer::extrusion_entity::role_to_string(role) == s, -1, format!("{role:?} {s}"));
            check("string_to_role", slicer::extrusion_entity::string_to_role(s) == back, -1, format!("{s}"));
            continue;
        }
        if line.starts_with("RS ") {
            t.s();
            let want = ROLES[t.i() as usize];
            let s = line.splitn(3, ' ').nth(2).unwrap();
            check("string_to_role", slicer::extrusion_entity::string_to_role(s) == want, -1, format!("{s}"));
            continue;
        }
        assert_eq!(t.s(), "C", "{line}");
        let case = t.i();
        let npaths = t.i() as usize;
        cases += 1;
        let mut paths = Vec::new();
        for _ in 0..npaths {
            let mut t = Tok::new(lines.next().unwrap());
            assert_eq!(t.s(), "p");
            let role = ROLES[t.i() as usize];
            let mm3 = t.d();
            let width = t.f();
            let height = t.f();
            let od = t.d();
            // ExtrusionEntity.hpp:229 `ExtrusionPath(double overhang_degree, int curve_degree, role, mm3_per_mm, float width, float height)`
            // (the curve degree is not printed: nothing compared here reads it)
            let mut path = ExtrusionPath::with_overhang(od, 0, role, mm3, width as f64, height as f64);
            *path.polyline.points_mut() = t.pts();
            paths.push(path);
        }
        let lp = ExtrusionLoop::new(paths.clone(), ExtrusionLoopRole::new());
        let mut coll: Option<ExtrusionEntityCollection> = None;
        // Result lines up to the next case.
        while let Some(&next) = lines.peek() {
            if next.starts_with("C ") {
                break;
            }
            let line = lines.next().unwrap();
            let mut t = Tok::new(line);
            match t.s() {
                "LEN" => {
                    let want = t.d();
                    let got = lp.length();
                    check("loop.length", got.to_bits() == want.to_bits(), case, format!("{got:e} vs {want:e}"));
                }
                "PL" => {
                    for p in &paths {
                        let want = t.d();
                        let got = p.length();
                        check("path.length", got.to_bits() == want.to_bits(), case, format!("{got:e} vs {want:e}"));
                    }
                }
                "MIN" => {
                    let want = t.d();
                    let got = lp.min_mm3_per_mm();
                    check("loop.min_mm3_per_mm", got.to_bits() == want.to_bits(), case, format!("{got:e} vs {want:e}"));
                }
                "CCW" => {
                    let want = t.i() != 0;
                    check("loop.is_counter_clockwise", lp.is_counter_clockwise() == want, case, String::new());
                }
                "POLY" => {
                    let want = t.pts();
                    check("loop.polygon", lp.polygon().points() == &want[..], case, String::new());
                }
                tag @ ("MCW" | "MCCW") => {
                    let want_ret = t.i() != 0;
                    let want = t.paths();
                    let mut l = lp.clone();
                    let ret = if tag == "MCW" { l.make_clockwise() } else { l.make_counter_clockwise() };
                    let name = if tag == "MCW" { "loop.make_clockwise" } else { "loop.make_counter_clockwise" };
                    check(name, ret == want_ret && paths_of(&l.paths) == want, case, String::new());
                }
                "HOP" => {
                    let pt = t.pt();
                    let want = t.i() != 0;
                    check("loop.has_overhang_point", lp.has_overhang_point(&pt) == want, case, format!("{pt:?}"));
                }
                "HOPP" => {
                    let want = t.i() != 0;
                    check("loop.has_overhang_paths", lp.has_overhang_paths() == want, case, String::new());
                }
                "SAV" => {
                    let pt = t.pt();
                    let eps = t.d();
                    let want_ret = t.i() != 0;
                    let want = t.paths();
                    let mut l = lp.clone();
                    let ret = l.split_at_vertex(&pt, eps);
                    check("loop.split_at_vertex", ret == want_ret && paths_of(&l.paths) == want, case, format!("{pt:?} eps {eps}"));
                }
                tag @ ("SA" | "SAD") => {
                    let pt = t.pt();
                    let prefer = t.i() != 0;
                    // ExtrusionEntity.hpp:522 the default epsilon is scaled<double>(0.001) = 0.001 / SCALING_FACTOR
                    let eps = if tag == "SA" { t.d() } else { 0.001 / 0.00001 };
                    let want = t.paths();
                    let mut l = lp.clone();
                    l.split_at(&pt, prefer, eps);
                    let name = if tag == "SA" { "loop.split_at" } else { "loop.split_at(default eps)" };
                    check(name, paths_of(&l.paths) == want, case, format!("{pt:?} prefer {prefer} eps {eps}"));
                }
                "CE" => {
                    let dist = t.d();
                    let want = t.paths();
                    let mut out = Vec::new();
                    lp.clip_end(dist, &mut out);
                    check("loop.clip_end", paths_of(&out) == want, case, format!("dist {dist}"));
                }
                tag @ ("CW" | "CS") => {
                    let eps = t.f();
                    let want = t.polys();
                    let mut out = Vec::new();
                    if tag == "CW" {
                        lp.polygons_covered_by_width(&mut out, eps);
                    } else {
                        lp.polygons_covered_by_spacing(&mut out, eps);
                    }
                    let name = if tag == "CW" { "loop.polygons_covered_by_width" } else { "loop.polygons_covered_by_spacing" };
                    check(name, polys_of(&out) == want, case, format!("eps {eps}: {} vs {} polygons", out.len(), want.len()));
                }
                "MPL" => {
                    let want = t.d();
                    let mp = ExtrusionMultiPath { paths: paths.clone(), can_reverse: true };
                    let got = mp.length();
                    check("multipath.length", got.to_bits() == want.to_bits(), case, format!("{got:e} vs {want:e}"));
                }
                "MPMIN" => {
                    let want = t.d();
                    let mp = ExtrusionMultiPath { paths: paths.clone(), can_reverse: true };
                    let got = mp.min_mm3_per_mm();
                    check("multipath.min_mm3_per_mm", got.to_bits() == want.to_bits(), case, format!("{got:e} vs {want:e}"));
                }
                "MPPL" => {
                    let want = t.pts();
                    let mp = ExtrusionMultiPath { paths: paths.clone(), can_reverse: true };
                    check("multipath.as_polyline", mp.as_polyline().points() == &want[..], case, String::new());
                }
                "KIN" => {
                    let mut t2 = Tok { t: t.t.clone(), k: t.k };
                    t2.s();
                    coll = Some(read_collection(&mut t2));
                    // the reader and writer round-trip the C++ line
                    check("collection.read", collection_tokens(coll.as_ref().unwrap()) == rest(&t), case, String::new());
                }
                "KAPP" => {
                    // ExtrusionEntity.hpp:732-763, the same inputs as gen.cpp
                    let mut app = ExtrusionEntityCollection::new();
                    let poly = lp.polygon();
                    let loops = vec![poly.clone(), Polygon::from_points(vec![poly.points()[0]]), poly.clone()];
                    slicer::extrusion_entity::extrusion_entities_append_loops(
                        &mut app.entities, loops, ExtrusionRole::Perimeter, 0.0123, 0.45, 0.2,
                    );
                    let pls = vec![
                        lp.paths[0].polyline.clone(),
                        slicer::geometry::Polyline::from_points(vec![poly.points()[0]]),
                        lp.as_polyline(),
                    ];
                    slicer::extrusion_entity::extrusion_entities_append_loops_and_paths(
                        &mut app.entities, pls, ExtrusionRole::SolidInfill, 0.0456, 0.42, 0.16,
                    );
                    check("extrusion_entities_append_loops(_and_paths)", collection_tokens(&app) == rest(&t), case, String::new());
                }
                "KIC" => {
                    let want = t.i() as usize;
                    check("collection.items_count", coll.as_ref().unwrap().items_count() == want, case, String::new());
                }
                tag @ ("KF0" | "KF1") => {
                    let f = coll.as_ref().unwrap().flatten(tag == "KF1");
                    let name = if tag == "KF1" { "collection.flatten(true)" } else { "collection.flatten(false)" };
                    check(name, collection_tokens(&f) == rest(&t), case, String::new());
                }
                "KMIN" => {
                    let want = t.d();
                    let got = coll.as_ref().unwrap().min_mm3_per_mm();
                    check("collection.min_mm3_per_mm", got.to_bits() == want.to_bits(), case, format!("{got:e} vs {want:e}"));
                }
                "KTV" => {
                    let want = t.d();
                    let got = coll.as_ref().unwrap().total_volume();
                    check("collection.total_volume", got.to_bits() == want.to_bits(), case, format!("{got:e} vs {want:e}"));
                }
                "KCH" => {
                    let start = t.pt();
                    let role = ROLES[t.i() as usize];
                    let ch = coll.as_ref().unwrap().chained_path_from_self(&start, role);
                    check("collection.chained_path_from", collection_tokens(&ch) == rest(&t), case, format!("{start:?} {role:?}"));
                }
                "KREV" => {
                    let mut r = coll.as_ref().unwrap().clone();
                    r.reverse();
                    check("collection.reverse", collection_tokens(&r) == rest(&t), case, String::new());
                }
                "KCW" => {
                    let eps = t.f();
                    let want = t.polys();
                    let mut out = Vec::new();
                    coll.as_ref().unwrap().polygons_covered_by_width(&mut out, eps);
                    check("collection.polygons_covered_by_width", polys_of(&out) == want, case, format!("{} vs {} polygons", out.len(), want.len()));
                }
                other => panic!("unknown tag {other} in {line}"),
            }
        }
    }
    let total: usize = calls.values().sum();
    let nbad: usize = bad.values().sum();
    eprintln!("extrusion_entity: {cases} cases, {total} calls, {nbad} mismatches");
    for (k, v) in &calls {
        eprintln!("  {k}: {v} calls, {} mismatches", bad.get(k).copied().unwrap_or(0));
    }
    for s in &first_bad {
        eprintln!("  {s}");
    }
    assert!(cases > 0);
    assert_eq!(nbad, 0, "{nbad} mismatches");
}
