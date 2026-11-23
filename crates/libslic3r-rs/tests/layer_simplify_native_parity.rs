//! R935: `Layer::simplify_support_entity_collection` / `simplify_support_path` / `simplify_support_multi_path`
//! / `simplify_support_loop` (Layer.cpp:622-685) against the pin. Vectors from
//! tests/data/perimeter_generator/gen.cpp `run_layer_simplify` (mode 'gen simplify <file>'), gzip -9n in
//! tests/data/perimeter_generator/layer_simplify_vectors.txt.gz: 64 collections of a path, a multi path of
//! two paths, a loop of two paths and a nested collection of one path, with the print config's spiral_mode,
//! enable_arc_fitting and resolution (Layer.cpp:640-643). Compared: the points of every path and its
//! fitting result (start and end index, move type, arc length bits). Also `Layer::simplify_wall_extrusion_path` /
//! `simplify_infill_extrusion_path` (Layer.hpp:266-267 -> LayerRegion.cpp:1037-1110) on the same paths with
//! wall and infill roles (the 'lsrg' line after each 'lsim' line).

use std::io::Read;
use std::sync::Arc;

use slicer::extrusion_entity::{
    ExtrusionEntityCollection, ExtrusionEntityType, ExtrusionLoop, ExtrusionLoopRole, ExtrusionMultiPath, ExtrusionPath,
    ExtrusionRole,
};
use slicer::geometry::{Point, Polyline};

struct Tok<'a> {
    it: std::str::SplitWhitespace<'a>,
}
impl<'a> Tok<'a> {
    fn i(&mut self) -> i64 {
        self.it.next().expect("token").parse().expect("int")
    }
    fn d(&mut self) -> f64 {
        f64::from_bits(u64::from_str_radix(self.it.next().expect("token"), 16).expect("hex double"))
    }
    fn b(&mut self) -> bool {
        self.i() != 0
    }
    fn polyline(&mut self) -> Polyline {
        let n = self.i();
        let mut pl = Polyline::new();
        for _ in 0..n {
            let x = self.i();
            let y = self.i();
            pl.points.push(Point::new(x, y));
        }
        pl
    }
}

fn path_s(pl: &Polyline) -> String {
    let mut s = pl.points.len().to_string();
    for p in &pl.points {
        s += &format!(" {} {}", p.x, p.y);
    }
    s += &format!(" {}", pl.fitting_result.len());
    for f in &pl.fitting_result {
        s += &format!(
            " {} {} {} {:016x}",
            f.start_point_index,
            f.end_point_index,
            f.path_type.clone() as i32,
            f.arc_data.length.to_bits()
        );
    }
    s
}

fn read_path(r: &mut Tok<'_>) -> String {
    let n = r.i();
    let mut th = n.to_string();
    for _ in 0..2 * n {
        th += &format!(" {}", r.i());
    }
    let nf = r.i();
    th += &format!(" {nf}");
    for _ in 0..nf {
        let s = r.i();
        let e = r.i();
        let ty = r.i();
        let len = r.d();
        th += &format!(" {s} {e} {ty} {:016x}", len.to_bits());
    }
    th
}

fn make_path(pl: Polyline) -> ExtrusionPath {
    let mut p = ExtrusionPath::new(ExtrusionRole::SupportMaterial);
    p.mm3_per_mm = 0.05;
    p.width = 0.4;
    p.height = 0.2;
    p.polyline = pl;
    p
}

#[test]
fn layer_simplify_support_native_parity() {
    let path = format!("{}/tests/data/perimeter_generator/layer_simplify_vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).unwrap_or_else(|e| panic!("{path}: {e}"));
    let (mut cases, mut paths, mut bad, mut bad_arc, mut shown) = (0usize, 0usize, 0usize, 0usize, 0usize);
    let lines: Vec<&str> = text.lines().collect();
    let (mut region_paths, mut region_bad) = (0usize, 0usize);
    for (line_no, line) in lines.iter().enumerate() {
        if line.starts_with("lsrg ") {
            continue;
        }
        let rest = line.strip_prefix("lsim ").unwrap_or_else(|| panic!("unknown line {}", line_no + 1));
        let (inputs, results) = rest.split_once(" => ").expect("results");
        let mut t = Tok { it: inputs.split_whitespace() };
        let spiral = t.b();
        let arc = t.b();
        let resolution = t.d();
        let pls: Vec<Polyline> = (0..6).map(|_| t.polyline()).collect();
        let mut pc = slicer::print_config::PrintConfig::default();
        pc.spiral_vase = spiral;
        pc.arc_fitting_enabled = arc;
        pc.resolution = resolution;
        let pc = Arc::new(pc);
        let mut layer = slicer::layer::Layer::new(1, 0, 0.2, 0.4, 0.3);
        layer.set_object_configs(Arc::new(slicer::print_config::PrintObjectConfig::default()), pc.clone());
        let build = |roles: Option<&[ExtrusionRole]>| -> ExtrusionEntityCollection {
            let role = |i: usize| roles.map(|r| r[i]).unwrap_or(ExtrusionRole::SupportMaterial);
            let path = |i: usize| {
                let mut p = make_path(pls[i].clone());
                p.set_extrusion_role(role(i));
                p
            };
            let mut coll = ExtrusionEntityCollection::new();
            coll.entities.push(ExtrusionEntityType::Path(path(0)));
            let mut mp = ExtrusionMultiPath::new();
            mp.paths.push(path(1));
            mp.paths.push(path(2));
            coll.entities.push(ExtrusionEntityType::MultiPath(mp));
            coll.entities.push(ExtrusionEntityType::Loop(ExtrusionLoop::new(vec![path(3), path(4)], ExtrusionLoopRole::DEFAULT)));
            let mut nested = ExtrusionEntityCollection::new();
            nested.entities.push(ExtrusionEntityType::Path(path(5)));
            coll.entities.push(ExtrusionEntityType::Collection(Box::new(nested)));
            coll
        };
        let paths_of = |coll: &ExtrusionEntityCollection| -> Vec<String> {
            let mut out: Vec<String> = Vec::new();
            for e in &coll.entities {
                match e {
                    ExtrusionEntityType::Path(p) => out.push(path_s(&p.polyline)),
                    ExtrusionEntityType::MultiPath(m) => m.paths.iter().for_each(|p| out.push(path_s(&p.polyline))),
                    ExtrusionEntityType::Loop(l) => l.paths.iter().for_each(|p| out.push(path_s(&p.polyline))),
                    ExtrusionEntityType::Collection(c) => match &c.entities[0] {
                        ExtrusionEntityType::Path(p) => out.push(path_s(&p.polyline)),
                        _ => panic!("nested path"),
                    },
                }
            }
            out
        };
        // Layer.hpp:266-267 simplify_wall_extrusion_path / simplify_infill_extrusion_path ->
        // LayerRegion.cpp:1037-1110, on the same paths with the roles of the following 'lsrg' line.
        {
            let rline = lines[line_no + 1].strip_prefix("lsrg ").unwrap_or_else(|| panic!("line {}: lsrg", line_no + 2));
            let (rin, rres) = rline.split_once(" => ").expect("results");
            let roles: Vec<ExtrusionRole> = rin
                .split_whitespace()
                .map(|v| match v {
                    "1" => ExtrusionRole::Perimeter,
                    "2" => ExtrusionRole::ExternalPerimeter,
                    "4" => ExtrusionRole::InternalInfill,
                    "5" => ExtrusionRole::SolidInfill,
                    other => panic!("role {other}"),
                })
                .collect();
            let mut layerm = slicer::layer::LayerRegion::new(1, 0);
            layerm.set_config_hierarchy(
                Arc::new(slicer::print_region::PrintRegion::new(slicer::region_config::PrintRegionConfig::default())),
                Arc::new(slicer::print_config::PrintObjectConfig::default()),
                pc.clone(),
            );
            layerm.perimeters = build(Some(&roles));
            layerm.fills = build(Some(&roles));
            let mut rl = slicer::layer::Layer::new(1, 0, 0.2, 0.4, 0.3);
            rl.set_object_configs(Arc::new(slicer::print_config::PrintObjectConfig::default()), pc.clone());
            rl.add_region(layerm);
            rl.simplify_wall_extrusion_path();
            rl.simplify_infill_extrusion_path();
            let mut mine = paths_of(&rl.regions()[0].perimeters);
            mine.extend(paths_of(&rl.regions()[0].fills));
            let mut r = Tok { it: rres.split_whitespace() };
            for m in &mine {
                let th = read_path(&mut r);
                region_paths += 1;
                if *m != th {
                    region_bad += 1;
                }
            }
        }

        let mut coll = ExtrusionEntityCollection::new();
        coll.entities.push(ExtrusionEntityType::Path(make_path(pls[0].clone())));
        let mut mp = ExtrusionMultiPath::new();
        mp.paths.push(make_path(pls[1].clone()));
        mp.paths.push(make_path(pls[2].clone()));
        coll.entities.push(ExtrusionEntityType::MultiPath(mp));
        coll.entities.push(ExtrusionEntityType::Loop(ExtrusionLoop::new(
            vec![make_path(pls[3].clone()), make_path(pls[4].clone())],
            ExtrusionLoopRole::DEFAULT,
        )));
        let mut nested = ExtrusionEntityCollection::new();
        nested.entities.push(ExtrusionEntityType::Path(make_path(pls[5].clone())));
        coll.entities.push(ExtrusionEntityType::Collection(Box::new(nested)));

        // Layer.cpp:622-636
        layer.simplify_support_entity_collection(&mut coll);

        let mut mine: Vec<String> = Vec::new();
        for e in &coll.entities {
            match e {
                ExtrusionEntityType::Path(p) => mine.push(path_s(&p.polyline)),
                ExtrusionEntityType::MultiPath(m) => m.paths.iter().for_each(|p| mine.push(path_s(&p.polyline))),
                ExtrusionEntityType::Loop(l) => l.paths.iter().for_each(|p| mine.push(path_s(&p.polyline))),
                ExtrusionEntityType::Collection(c) => match &c.entities[0] {
                    ExtrusionEntityType::Path(p) => mine.push(path_s(&p.polyline)),
                    _ => panic!("nested path"),
                },
            }
        }
        let mut r = Tok { it: results.split_whitespace() };
        cases += 1;
        for (k, m) in mine.iter().enumerate() {
            let th = read_path(&mut r);
            paths += 1;
            if *m != th {
                bad += 1;
                if arc && !spiral {
                    bad_arc += 1;
                }
                if shown < 4 {
                    shown += 1;
                    let at = m.bytes().zip(th.bytes()).position(|(a, b)| a != b).unwrap_or(m.len().min(th.len()));
                    eprintln!(
                        "line {} path {k} (spiral {spiral} arc {arc} resolution {resolution}): pin ..{}.. mine ..{}..",
                        line_no + 1,
                        &th[at.saturating_sub(40)..(at + 80).min(th.len())],
                        &m[at.saturating_sub(40)..(at + 80).min(m.len())]
                    );
                }
            }
        }
    }
    eprintln!("layer simplify_support parity: {cases} collections, {paths} paths; differing {bad} ({bad_arc} with arc fitting)");
    eprintln!("layer simplify_wall/infill_extrusion_path parity: {region_paths} paths; differing {region_bad}");
    assert_eq!(cases, 64);
    assert_eq!(bad, 0, "Layer::simplify_support_* differs from the pin");
    assert_eq!(region_paths, 64 * 12);
    assert_eq!(region_bad, 0, "Layer::simplify_wall_extrusion_path / simplify_infill_extrusion_path differ from the pin");
}
