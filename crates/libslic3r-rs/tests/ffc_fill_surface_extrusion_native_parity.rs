//! FillFloatingConcentric::fill_surface_extrusion against the pin (override
//! libslic3r/bambustudio/libslic3r/Fill/FillFloatingConcentric.cpp:880-1000).
//!
//! The vectors (`tests/data/ffc_fill_surface_extrusion/vectors.txt.gz`) are 54 fill calls of the
//! H2D Majora job, recorded by the port's FFCDUMP probe (surface, lower-layer unsupported areas,
//! lower sparse polygons, flow, spacing, loop clipping) and run through the pin's own code by
//! `tests/data/ffc_fill_surface_extrusion/gen.cpp` on one thread. Every path the port makes
//! (points, width, mm3_per_mm, height, floating-vertical-shell flag) must be bit-equal.

use std::io::Read;

use slicer::extrusion_entity::{CustomizeFlag, ExtrusionEntityType, ExtrusionPath, ExtrusionRole};
use slicer::fill::fill_floating_concentric::FillFloatingConcentric;
use slicer::fill::FillParams;
use slicer::flow::Flow;
use slicer::geometry::{ExPolygon, Point, Polygon};
use slicer::print_config::{PrintConfig, PrintObjectConfig};
use slicer::surface::{Surface, SurfaceType};

fn f64_from_hex(s: &str) -> f64 {
    f64::from_bits(u64::from_str_radix(s, 16).expect("hex"))
}

fn read_pts(it: &mut std::slice::Iter<'_, &str>) -> Vec<Point> {
    let n: usize = it.next().unwrap().parse().unwrap();
    (0..n)
        .map(|_| {
            let x: i64 = it.next().unwrap().parse().unwrap();
            let y: i64 = it.next().unwrap().parse().unwrap();
            Point::new(x, y)
        })
        .collect()
}

fn read_expolys<'a>(lines: &mut impl Iterator<Item = &'a str>) -> Vec<ExPolygon> {
    let head: Vec<&str> = lines.next().unwrap().split_whitespace().collect();
    let n: usize = head[1].parse().unwrap();
    (0..n)
        .map(|_| {
            let toks: Vec<&str> = lines.next().unwrap().split_whitespace().collect();
            let mut it = toks[1..].iter();
            let mut e = ExPolygon::new(Polygon::from_points(read_pts(&mut it)));
            let nh: usize = it.next().unwrap().parse().unwrap();
            for _ in 0..nh {
                e.holes.push(Polygon::from_points(read_pts(&mut it)));
            }
            e
        })
        .collect()
}

fn path_record(p: &ExtrusionPath) -> String {
    let mut s = format!("Q {}", p.polyline.points.len());
    for q in p.polyline.points.iter() {
        s.push_str(&format!(" {} {}", q.x(), q.y()));
    }
    s.push_str(&format!(
        " {:016x} {:016x} {:016x} {}",
        p.width.to_bits(),
        p.mm3_per_mm.to_bits(),
        p.height.to_bits(),
        (p.get_customize_flag() == CustomizeFlag::FloatingVerticalShell) as u8
    ));
    s
}

fn flatten(e: &ExtrusionEntityType, out: &mut Vec<String>) {
    match e {
        ExtrusionEntityType::Path(p) => out.push(path_record(p)),
        ExtrusionEntityType::Loop(l) => l.paths.iter().for_each(|p| out.push(path_record(p))),
        ExtrusionEntityType::MultiPath(m) => m.paths.iter().for_each(|p| out.push(path_record(p))),
        ExtrusionEntityType::Collection(c) => c.entities.iter().for_each(|x| flatten(x, out)),
    }
}

#[test]
fn ffc_fill_surface_extrusion_matches_pin() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ffc_fill_surface_extrusion/vectors.txt.gz");
    let file = std::fs::File::open(path).expect("vectors");
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    let mut lines = text.lines();
    let (mut cases, mut paths_checked, mut bad) = (0usize, 0usize, 0usize);
    while let Some(head) = lines.next() {
        if !head.starts_with("CALL ") {
            continue;
        }
        let t: Vec<&str> = head.split_whitespace().collect();
        let width = f64_from_hex(t[1]);
        let height = f64_from_hex(t[2]);
        let nozzle = f64_from_hex(t[3]);
        let layer_height = f64_from_hex(t[6]);
        let spacing = f64_from_hex(t[7]);
        let loop_clipping: i64 = t[8].parse().unwrap();
        let detect = t[9] == "1";
        let nozzle_cfg = f64_from_hex(t[10]);
        let expolys = read_expolys(&mut lines);
        let unsup = read_expolys(&mut lines);
        let sh: Vec<&str> = lines.next().unwrap().split_whitespace().collect();
        let ns: usize = sh[1].parse().unwrap();
        let sparse: Vec<Polygon> = (0..ns)
            .map(|_| {
                let toks: Vec<&str> = lines.next().unwrap().split_whitespace().collect();
                Polygon::from_points(read_pts(&mut toks[1..].iter()))
            })
            .collect();
        let eh: Vec<&str> = lines.next().unwrap().split_whitespace().collect();
        assert_eq!(eh[0], "EXP");
        let ne: usize = eh[1].parse().unwrap();
        let expected: Vec<String> = (0..ne).map(|_| lines.next().unwrap().to_string()).collect();
        assert_eq!(lines.next(), Some("END"));

        let mut pc = PrintConfig::default();
        pc.nozzle_diameter = nozzle_cfg;
        let mut poc = PrintObjectConfig::default();
        poc.detect_floating_vertical_shell = detect;
        let mut f = FillFloatingConcentric {
            spacing,
            loop_clipping,
            no_overlap_expolygons: expolys.clone(),
            lower_layer_unsupport_areas: unsup,
            lower_sparse_polys: sparse,
            print_config: Some(&pc),
            print_object_config: Some(&poc),
        };
        let mut params = FillParams::default();
        params.flow = Flow::new(width, height, nozzle).expect("flow");
        params.layer_height = layer_height;
        params.extrusion_role = ExtrusionRole::FloatingVerticalShell;
        let surface = Surface::new(
            SurfaceType::InternalSolid,
            expolys.first().cloned().unwrap_or_else(|| ExPolygon::new(Polygon::from_points(vec![]))),
        );
        let mut out = Vec::new();
        f.fill_surface_extrusion(&surface, &params, &mut out);
        let mut got = Vec::new();
        out.iter().for_each(|e| flatten(e, &mut got));
        paths_checked += expected.len();
        if got != expected {
            bad += 1;
            if bad <= 3 {
                let k = got.iter().zip(expected.iter()).position(|(a, b)| a != b);
                eprintln!("case {}: {} paths vs pin {}, first differing path {:?}", cases, got.len(), expected.len(), k);
            }
        }
        cases += 1;
    }
    eprintln!("ffc_fill_surface_extrusion: {} cases, {} pin paths, {} cases differ", cases, paths_checked, bad);
    assert_eq!(cases, 54);
    assert_eq!(bad, 0);
}
