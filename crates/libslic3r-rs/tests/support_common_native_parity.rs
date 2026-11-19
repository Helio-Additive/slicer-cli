//! Native parity of the Support/SupportCommon.cpp port (`slicer::support::support_common`) against
//! vectors made by the C++ at the pin (tests/data/support_common/gen.cpp).
//!
//! The generator runs the helpers on polygons (safe_union, safe_offset_inc, draw_perimeters,
//! keep_polylines_near_bbox) and, for nine scenarios, builds a Print of one box with `Print::apply`, a
//! SupportParameters of its object and a storage of support generator layers, and runs the steps
//! `PrintObjectSupportMaterial::generate()` runs once the layers exist: compact_layers,
//! generate_interface_layers, generate_raft_base, generate_support_layers, generate_support_toolpaths. The
//! first scenario also runs the path helpers one by one (tree_supports_generate_paths,
//! fill_expolygons_with_sheath_generate_paths, LoopInterfaceProcessor::generate,
//! SupportGeneratorLayerExtruded::merge, modulate_extrusion_by_overlapping_layers,
//! modulate_extrusion_paths_by_overlapping_layers, idx_higher_or_equal, idx_lower_or_equal,
//! fill_expolygon_generate_paths and the two overloads of fill_expolygons_generate_paths). Eight more
//! scenarios ("m") build a Print in the mode of several nozzle diameters (in two of them of one diameter,
//! where no plan is made) and run merged_band_footprint and merge_support_layers_multi_diameter on object
//! layers and support generator layers written by hand; the plan `make_support_merge_plan` gave in the
//! C++ is an input. The lines of the vectors
//! that start with "in" are the inputs, which this test reads back; every other line it has to reproduce.
//!
//! The fillers are the generator's `RecFill` on both sides: it writes down what it is asked (the
//! "fill_surface" lines) and answers with lines clipped with the surface, so that what is compared is
//! everything around the fillers and not the fillers of the engine.

mod support_shared;

use slicer::config::{ConfigOptionInts, ConfigOptionStrings};
use slicer::geometry::{BoundingBox, ExPolygon, ExPolygons, Point, Polygon, Polygons, Polyline, Polylines};
use slicer::model::*;
use slicer::print_config::config_def::DynamicPrintConfig;
use slicer::support::support_common::*;
use slicer::support::support_layer::{SupporLayerType, SupportGeneratorLayer, SupportGeneratorLayerStorage, SupportGeneratorLayersPtr};
use slicer::support::support_parameters::SupportParameters;
use support_shared::*;

use slicer::extrusion_entity::{ExtrusionEntityCollection, ExtrusionEntityType, ExtrusionMultiPath, ExtrusionPath, ExtrusionRole};
use slicer::fill::fill_base::Fill;
use slicer::fill::FillParams;
use slicer::flow::Flow;
use slicer::print_config::InfillPattern;
use slicer::surface::Surface;
use ordered_float::OrderedFloat;
use slicer::adaptive_layer_merge::{AdaptiveLayerBand, AdaptiveLayerHeightRange, AdaptiveMergePatch};
use slicer::layer::{Layer, LayerRegion};
use slicer::support_merge_plan::{Candidate, Slab, SupportMergePlan};
use slicer::surface::SurfaceType;
use std::cell::RefCell;
use std::collections::BTreeMap;
use std::rc::Rc;

type Log = Rc<RefCell<Vec<String>>>;

/// gen.cpp `RecFill`
struct RecFill {
    base: Fill,
    pattern: InfillPattern,
    log: Log,
}
impl SupportFill for RecFill {
    fn fill(&self) -> &Fill {
        &self.base
    }
    fn fill_mut(&mut self) -> &mut Fill {
        &mut self.base
    }
    fn fill_surface(&mut self, surface: &Surface, params: &FillParams) -> Result<Polylines, InfillFailedException> {
        let f = &self.base;
        self.log.borrow_mut().push(format!(
            "fill_surface {} {} {} {} {} {} {} {} {} {} {} {}",
            self.pattern as i32,
            hf(f.angle),
            hd(f.spacing),
            f.link_max_length,
            pt(&f.bounding_box.min),
            pt(&f.bounding_box.max),
            f.layer_id as i64,
            hd(f.z),
            hd(f.overlap),
            hf(params.density as f32),
            hb(params.dont_adjust),
            expoly_s(&surface.expolygon)
        ));
        let pts = &surface.expolygon.contour.points;
        if pts.is_empty() {
            return Ok(Vec::new());
        }
        let (mut lo, mut hi) = (pts[0], pts[0]);
        for q in pts {
            lo = Point::new(lo.x.min(q.x), lo.y.min(q.y));
            hi = Point::new(hi.x.max(q.x), hi.y.max(q.y));
        }
        let d: f64 = if params.density as f32 > 0. { params.density as f32 as f64 } else { 1. };
        let mut step: i64 = ((f.spacing / slicer::libslic3r::SCALING_FACTOR) / d) as i32 as i64;
        if step < 20000 {
            step = 20000;
        }
        let slant: i64 = if self.pattern == InfillPattern::Rectilinear { 0 } else { 7770 };
        let mut lines: Polylines = Vec::new();
        let mut y = lo.y + step / 2;
        while y < hi.y {
            lines.push(Polyline::from_points(vec![Point::new(lo.x - 1000, y), Point::new(hi.x + 1000, y + slant)]));
            y += step;
        }
        if self.pattern == InfillPattern::Concentric && lines.len() % 2 == 1 {
            return Err(InfillFailedException);
        }
        Ok(cu::intersection_pl_ex(&lines, std::slice::from_ref(&surface.expolygon)))
    }
}
struct RecFactory {
    log: Log,
}
impl SupportFillFactory for RecFactory {
    fn new_from_type(&self, pattern: InfillPattern) -> Box<dyn SupportFill> {
        Box::new(RecFill { base: Fill::new(), pattern, log: self.log.clone() })
    }
}
fn flush(r: &mut Run, log: &Log) {
    for l in log.borrow_mut().drain(..) {
        r.out(l);
    }
}

fn flow_from(t: &mut Tok) -> Flow {
    let (w, h, n) = (t.f(), t.f(), t.f());
    Flow::new(w as f64, h as f64, n as f64).expect("flow")
}
/// gen.cpp `PATH(p)`
fn path_s(p: &ExtrusionPath) -> String {
    format!(
        "P {} {} {} {} {} {} {}",
        p.role as i32,
        hd(p.mm3_per_mm),
        hf(p.width as f32),
        hf(p.height as f32),
        hb(p.can_reverse),
        hd(p.overhang_degree),
        pts_s(&p.polyline.points)
    )
}
/// gen.cpp `ENT(e)`
fn ent_s(e: &ExtrusionEntityType) -> String {
    match e {
        ExtrusionEntityType::Collection(c) => {
            let mut o = format!("C {} {}", hb(c.no_sort), c.entities.len());
            for ee in &c.entities {
                o += &format!(" {}", ent_s(ee));
            }
            o
        }
        ExtrusionEntityType::MultiPath(m) => {
            let mut o = format!("M {} {}", hb(m.can_reverse), m.paths.len());
            for q in &m.paths {
                o += &format!(" {}", path_s(q));
            }
            o
        }
        ExtrusionEntityType::Path(q) => path_s(q),
        _ => "X".into(),
    }
}
/// gen.cpp `ENTS(v)`
fn ents_s(v: &[ExtrusionEntityType]) -> String {
    let mut o = format!("{}", v.len());
    for e in v {
        o += &format!(" {}", ent_s(e));
    }
    o
}
fn role_from(v: i64) -> ExtrusionRole {
    match v {
        14 => ExtrusionRole::SupportMaterial,
        15 => ExtrusionRole::SupportMaterialInterface,
        16 => ExtrusionRole::SupportTransition,
        17 => ExtrusionRole::SupportIroning,
        _ => panic!("role {v}"),
    }
}
fn tok_path(t: &mut Tok) -> ExtrusionPath {
    let role = role_from(t.i());
    let mm3_per_mm = t.d();
    let (w, h) = (t.f(), t.f());
    let mut q = ExtrusionPath::with_params(role, mm3_per_mm, w as f64, h as f64, false);
    q.can_reverse = t.b();
    q.overhang_degree = t.d();
    let n = t.i();
    q.polyline = Polyline::from_points((0..n).map(|_| t.pt()).collect());
    q
}
fn tok_ent(t: &mut Tok) -> ExtrusionEntityType {
    match t.s() {
        "P" => ExtrusionEntityType::Path(tok_path(t)),
        "M" => {
            let mut m = ExtrusionMultiPath::new();
            m.can_reverse = t.b();
            let n = t.i();
            for _ in 0..n {
                assert_eq!(t.s(), "P");
                m.paths.push(tok_path(t));
            }
            ExtrusionEntityType::MultiPath(m)
        }
        "C" => {
            let mut c = ExtrusionEntityCollection::new();
            c.no_sort = t.b();
            let n = t.i();
            for _ in 0..n {
                c.entities.push(tok_ent(t));
            }
            ExtrusionEntityType::Collection(Box::new(c))
        }
        other => panic!("entity {other}"),
    }
}
fn tok_ents(t: &mut Tok) -> Vec<ExtrusionEntityType> {
    let n = t.i();
    (0..n).map(|_| tok_ent(t)).collect()
}
fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Polygon {
    let sc = |v: f64| (v / slicer::libslic3r::SCALING_FACTOR).round_ties_even() as i64;
    Polygon::from_points(vec![Point::new(sc(x0), sc(y0)), Point::new(sc(x1), sc(y0)), Point::new(sc(x1), sc(y1)), Point::new(sc(x0), sc(y1))])
}

/// gen.cpp `run_path_helpers(p0)`
fn run_path_helpers(r: &mut Run, p0: &SupportParameters) {
    let log: Log = Rc::new(RefCell::new(Vec::new()));
    let factory = RecFactory { log: log.clone() };
    let mut shapes: Vec<Polygons> = Vec::new();
    while let Some(line) = r.input("in shape") {
        shapes.push(Tok::new(&line).polys());
    }
    let mut flows: Vec<Flow> = Vec::new();
    while let Some(line) = r.input("in flow") {
        flows.push(flow_from(&mut Tok::new(&line)));
    }

    // tree_supports_generate_paths
    let areas = [p0.tree_branch_diameter_double_wall_area_scaled, 0., 0.1, 3e12];
    for i in 0..shapes.len() {
        for f in 0..flows.len() {
            for a in 0..4 {
                if (i + f + a) % 2 == 1 {
                    continue;
                }
                let mut p = p0.clone();
                p.tree_branch_diameter_double_wall_area_scaled = areas[a];
                p.multi_nozzle_diameter_mode = (i + a) % 3 == 0;
                let mut dst = Vec::new();
                tree_supports_generate_paths(&mut dst, &shapes[i], &flows[f], &p).expect("tree_supports_generate_paths");
                r.out(format!("tsgp {i} {f} {} {} {}", hd(areas[a]), hb(p.multi_nozzle_diameter_mode), ents_s(&dst)));
            }
        }
    }

    // fill_expolygons_with_sheath_generate_paths
    let fs: [(f32, bool, bool, InfillPattern, ExtrusionRole); 6] = [
        (0., true, false, InfillPattern::Rectilinear, ExtrusionRole::SupportMaterial),
        (0.25, true, false, InfillPattern::Rectilinear, ExtrusionRole::SupportMaterial),
        (0.4, true, true, InfillPattern::SupportBase, ExtrusionRole::SupportMaterial),
        (0.5, false, false, InfillPattern::Concentric, ExtrusionRole::SupportMaterialInterface),
        (1., false, true, InfillPattern::Rectilinear, ExtrusionRole::SupportTransition),
        (0., false, false, InfillPattern::Honeycomb, ExtrusionRole::SupportMaterial),
    ];
    for i in 0..shapes.len() {
        for f in 0..flows.len() {
            for (k, fk) in fs.iter().enumerate() {
                if (i + f + k) % 2 == 0 {
                    continue;
                }
                let mut filler = factory.new_from_type(fk.3);
                filler.fill_mut().angle = 0.25f32 * (k + 1) as f32;
                filler.fill_mut().spacing = flows[f].spacing();
                filler.fill_mut().link_max_length = 100 * k as i64;
                filler.fill_mut().bounding_box = BoundingBox::from_points_minmax(Point::new(-5, -7), Point::new(11, 13));
                let mut dst = Vec::new();
                fill_expolygons_with_sheath_generate_paths(&mut dst, &shapes[i], filler.as_mut(), fk.0, fk.4, &flows[f], p0, fk.1, fk.2)
                    .expect("fill_expolygons_with_sheath_generate_paths");
                flush(r, &log);
                r.out(format!("fesg {i} {f} {k} {}", ents_s(&dst)));
            }
        }
    }

    // LoopInterfaceProcessor::generate
    while let Some(line) = r.input("in lip") {
        let mut t = Tok::new(&line);
        let circle_radius = t.d();
        let n_contact_loops = t.i() as i32;
        let flow = flows[t.i() as usize];
        let rest: Vec<&str> = t.t.collect();
        let mut storage = SupportGeneratorLayerStorage::new();
        read_layer(&mut storage, &rest.join(" "));
        let mut proc = LoopInterfaceProcessor::new(1.5 * flow.scaled_width() as f64);
        assert_eq!(proc.circle_radius.to_bits(), circle_radius.to_bits());
        proc.n_contact_loops = n_contact_loops;
        let mut ex = SupportGeneratorLayerExtruded::default();
        ex.layer = Some(0);
        proc.generate(&mut ex, &flow, &mut storage).expect("LoopInterfaceProcessor::generate");
        r.out(format!("lip {} {} {} {}", poly_s(&proc.circle), ents_s(&ex.extrusions), polys_s(ex.polygons_to_extrude(&storage)), layer_s(&storage[0])));
    }

    // SupportGeneratorLayerExtruded: could_merge, merge, polygons_append
    for c in 0..6 {
        let mut storage = SupportGeneratorLayerStorage::new();
        {
            let a = storage.allocate(SupporLayerType::SltBase);
            a.print_z = 1.;
            a.height = 0.2;
            a.polygons = vec![rect(0., 0., 10., 6.)];
        }
        {
            let b = storage.allocate(SupporLayerType::SltTopContact);
            b.print_z = 1.;
            b.height = if c >= 4 { 0.25 } else { 0.2 };
            b.polygons = vec![rect(8., 2., 16., 9.)];
        }
        let mut ea = SupportGeneratorLayerExtruded::default();
        let mut eb = SupportGeneratorLayerExtruded::default();
        ea.layer = Some(0);
        eb.layer = Some(1);
        if c & 1 != 0 {
            ea.set_polygons_to_extrude(vec![rect(1., 1., 9., 5.)]);
        }
        if c & 2 != 0 {
            eb.set_polygons_to_extrude(vec![rect(9., 3., 15., 8.)]);
        }
        let cm = ea.could_merge(&eb, &storage);
        if cm {
            ea.merge(&mut eb, &mut storage);
        }
        let mut app: Polygons = Vec::new();
        ea.polygons_append(&mut app, &storage);
        r.out(format!(
            "extruded_merge {c} {} {} {} {} {} {}",
            hb(cm),
            polys_s(ea.polygons_to_extrude(&storage)),
            polys_s(&storage[0].polygons),
            polys_s(&storage[1].polygons),
            hb(eb.empty(&storage)),
            polys_s(&app)
        ));
    }

    // modulate_extrusion_by_overlapping_layers
    while let Some(line) = r.input("in mod") {
        let parts: Vec<&str> = line.split(" | ").collect();
        let mut storage = SupportGeneratorLayerStorage::new();
        read_layer(&mut storage, parts[0]);
        let n: usize = parts[1].trim().parse().expect("count");
        let mut overlapping: SupportGeneratorLayersPtr = Vec::new();
        for k in 0..n {
            read_layer(&mut storage, parts[2 + k]);
            overlapping.push(1 + k);
        }
        let line = r.input("in mod_ents").expect("mod_ents");
        let mut ents = tok_ents(&mut Tok::new(&line));
        modulate_extrusion_by_overlapping_layers(&mut ents, 0, &overlapping, &storage).expect("modulate_extrusion_by_overlapping_layers");
        r.out(format!("mod {}", ents_s(&ents)));
        // modulate_extrusion_paths_by_overlapping_layers by itself, on paths only.
        let line = r.input("in modp_ents").expect("modp_ents");
        let mut paths = tok_ents(&mut Tok::new(&line));
        modulate_extrusion_paths_by_overlapping_layers(&mut paths, 0, &overlapping, &storage).expect("modulate_extrusion_paths_by_overlapping_layers");
        r.out(format!("modp {}", ents_s(&paths)));
    }

    // idx_higher_or_equal, idx_lower_or_equal: the binary search and the linear one from several starts.
    {
        let zs = [0.2, 0.4, 0.4, 0.75, 1.0, 1.3, 1.3, 2.0, 2.4];
        for n in [0usize, 1, 2, 3, 9] {
            let v = &zs[..n];
            for thr in [-1., 0.2, 0.3, 0.4, 0.75, 1.3, 2.4, 5.] {
                let ge = |z: &f64| *z >= thr;
                let le = |z: &f64| *z <= thr;
                let mut line = format!("idx {n} {} {}", hd(thr), idx_higher_or_equal(v, -1, ge));
                for start in [0isize, 1, 4] {
                    if start <= n as isize {
                        line += &format!(" {}", idx_higher_or_equal(v, start, ge));
                    }
                }
                line += &format!(" {}", idx_lower_or_equal(v, -2, le));
                for start in [-1isize, 0, 2, 8] {
                    if start < n as isize {
                        line += &format!(" {}", idx_lower_or_equal(v, start, le));
                    }
                }
                r.out(line);
            }
        }
    }

    // fill_expolygon_generate_paths and the two overloads of fill_expolygons_generate_paths.
    while let Some(line) = r.input("in fegp") {
        let mut t = Tok::new(&line);
        let i = t.i() as usize;
        let density = t.f();
        let flow = flows[i % 3];
        let pattern = match i % 3 {
            1 => InfillPattern::Concentric,
            2 => InfillPattern::Honeycomb,
            _ => InfillPattern::Rectilinear,
        };
        let mut filler = factory.new_from_type(pattern);
        filler.fill_mut().angle = 0.3f32 * i as f32;
        filler.fill_mut().spacing = flow.spacing();
        filler.fill_mut().link_max_length = 50 * i as i64;
        filler.fill_mut().bounding_box = BoundingBox::from_points_minmax(Point::new(-3, -4), Point::new(21, 17));
        let ex = cu::union_ex(&shapes[i]);
        let mut fp = FillParams::new();
        fp.density = density as f64;
        fp.dont_adjust = i % 2 == 0;
        let role = if i % 2 == 1 { ExtrusionRole::SupportMaterialInterface } else { ExtrusionRole::SupportMaterial };
        let mut dst = Vec::new();
        fill_expolygon_generate_paths(&mut dst, ex[0].clone(), filler.as_mut(), &fp, density, role, &flow).expect("fill_expolygon_generate_paths");
        flush(r, &log);
        r.out(format!("fegp {i} {}", ents_s(&dst)));
        let mut dst = Vec::new();
        fill_expolygons_generate_paths_params(&mut dst, ex.clone(), filler.as_mut(), &fp, density, role, &flow).expect("fill_expolygons_generate_paths");
        flush(r, &log);
        r.out(format!("fesgp {i} {}", ents_s(&dst)));
        let mut dst = Vec::new();
        fill_expolygons_generate_paths(&mut dst, ex.clone(), filler.as_mut(), density, role, &flow).expect("fill_expolygons_generate_paths");
        flush(r, &log);
        r.out(format!("fesgd {i} {}", ents_s(&dst)));
    }
}

fn pts_s(pts: &[Point]) -> String {
    let mut o = format!("{}", pts.len());
    for q in pts {
        o += &format!(" {}", pt(q));
    }
    o
}
fn pls_s(ps: &[Polyline]) -> String {
    let mut o = format!("{}", ps.len());
    for p in ps {
        o += &format!(" {}", pts_s(&p.points));
    }
    o
}
fn tok_pls(t: &mut Tok) -> Polylines {
    let n = t.i();
    (0..n)
        .map(|_| {
            let m = t.i();
            Polyline::from_points((0..m).map(|_| t.pt()).collect())
        })
        .collect()
}

fn layer_type_from(v: i64) -> SupporLayerType {
    match v {
        0 => SupporLayerType::SltUnknown,
        1 => SupporLayerType::SltRaftBase,
        2 => SupporLayerType::SltRaftInterface,
        3 => SupporLayerType::SltBottomContact,
        4 => SupporLayerType::SltBottomInterface,
        5 => SupporLayerType::SltBase,
        6 => SupporLayerType::SltTopInterface,
        7 => SupporLayerType::SltTopContact,
        8 => SupporLayerType::SltIntermediate,
        _ => panic!("layer type {v}"),
    }
}
fn opt_s(p: &Option<Polygons>) -> String {
    match p {
        Some(p) => format!("1 {}", polys_s(p)),
        None => "0".into(),
    }
}
/// gen.cpp `LAYER(l)`
fn layer_s(l: &SupportGeneratorLayer) -> String {
    format!(
        "{} {} {} {} {} {} {} {} {} {} {} {}",
        l.layer_type as i32,
        hd(l.print_z),
        hd(l.bottom_z),
        hd(l.height),
        l.idx_object_layer_above as i64,
        l.idx_object_layer_below as i64,
        hb(l.bridging),
        hb(l.up),
        polys_s(&l.polygons),
        opt_s(&l.contact_polygons),
        opt_s(&l.overhang_polygons),
        opt_s(&l.enforcer_polygons)
    )
}
fn tok_opt(t: &mut Tok) -> Option<Polygons> {
    if t.b() {
        Some(t.polys())
    } else {
        None
    }
}
fn read_layer(storage: &mut SupportGeneratorLayerStorage, line: &str) {
    let mut t = Tok::new(line);
    let l = storage.allocate(layer_type_from(t.i()));
    l.print_z = t.d();
    l.bottom_z = t.d();
    l.height = t.d();
    l.idx_object_layer_above = t.i() as usize;
    l.idx_object_layer_below = t.i() as usize;
    l.bridging = t.b();
    l.up = t.b();
    l.polygons = t.polys();
    l.contact_polygons = tok_opt(&mut t);
    l.overhang_polygons = tok_opt(&mut t);
    l.enforcer_polygons = tok_opt(&mut t);
}
fn tok_list(t: &mut Tok) -> SupportGeneratorLayersPtr {
    let n = t.i();
    (0..n)
        .map(|_| {
            let v = t.i();
            if v < 0 {
                NULL_LAYER
            } else {
                v as usize
            }
        })
        .collect()
}
fn list_s(v: &SupportGeneratorLayersPtr) -> String {
    let mut o = format!("{}", v.len());
    for &l in v {
        o += &format!(" {}", if l == NULL_LAYER { -1 } else { l as i64 });
    }
    o
}
fn dump_storage(r: &mut Run, tag: &str, storage: &SupportGeneratorLayerStorage) {
    r.out(format!("{tag} storage {}", storage.len()));
    for i in 0..storage.len() {
        r.out(format!("{tag} L {}", layer_s(&storage[i])));
    }
}

fn run_helpers(r: &mut Run) {
    use slicer::support::support_common::cu;
    let mut sets: Vec<Polygons> = Vec::new();
    while let Some(line) = r.input("in set") {
        sets.push(Tok::new(&line).polys());
    }
    for i in 0..sets.len() {
        for j in 0..sets.len() {
            if j != i && j != (i + 1) % sets.len() && j != 6 {
                continue;
            }
            r.out(format!("safe_union {i} {j} {}", polys_s(&safe_union(&sets[i], &sets[j]))));
            let (a, b) = (cu::union_ex(&sets[i]), cu::union_ex(&sets[j]));
            r.out(format!("safe_union_ex {i} {j} {}", expolys_s(&safe_union_ex(&a, &b))));
            let ra: ExPolygons = sets[i].iter().map(|p| ExPolygon::new(p.clone())).collect();
            let rb: ExPolygons = sets[j].iter().map(|p| ExPolygon::new(p.clone())).collect();
            r.out(format!("safe_union_raw {i} {j} {}", expolys_s(&safe_union_ex(&ra, &rb))));
        }
    }
    let mut collisions: Vec<Polygons> = Vec::new();
    while let Some(line) = r.input("in collision") {
        collisions.push(Tok::new(&line).polys());
    }
    let args: [(i64, i64, i64, usize); 11] = [
        (0, 20000, 0, 0),
        (0, 20000, 30000, 1),
        (30000, 20000, 0, 1),
        (30000, 20000, 50000, 1),
        (250000, 20000, 10000, 1),
        (250000, 60000, 0, 3),
        (70000, 40000, 20000, 12),
        (90000, 100000, 100000, 5),
        (45000, -1, 0, 1),
        (123457, 31000, 7000, 2),
        (400000, 21000, 15000, 30),
    ];
    for i in 0..sets.len() {
        for c in 0..collisions.len() {
            for (a, g) in args.iter().enumerate() {
                if (i + c + a) % 3 != 0 && a > 1 {
                    continue;
                }
                let id = format!("{i} {c} {} {} {} {}", g.0, g.1, g.2, g.3);
                r.out(format!("safe_offset_inc {id} {}", polys_s(&safe_offset_inc(&sets[i], g.0, &collisions[c], g.1, g.2, g.3))));
                r.out(format!(
                    "safe_offset_inc_ex {id} {}",
                    expolys_s(&safe_offset_inc_ex(&cu::union_ex(&sets[i]), g.0, &collisions[c], g.1, g.2, g.3))
                ));
            }
        }
    }

    // draw_perimeters
    let mut i = 0;
    while let Some(line) = r.input("in expoly") {
        let expoly = Tok::new(&line).expoly();
        for clip in [0., 6300., 123456.7, 2500000., 1e9] {
            r.out(format!("draw_perimeters {i} {} {}", hd(clip), pls_s(&draw_perimeters(&expoly, clip))));
        }
        i += 1;
    }

    // keep_polylines_near_bbox
    let line = r.input("in lines").expect("lines");
    let lines = tok_pls(&mut Tok::new(&line));
    let boxes = [
        (-200000, -200000, 400000, 500000),
        (10000000, 10000000, 11000000, 11000000),
        (-5000000, -5000000, 5000000, 5000000),
        (1000000, -100000, 1050000, 0),
        (2900000, 1900000, 3100000, 2100000),
    ];
    for b in boxes {
        let bb = BoundingBox::from_points_minmax(Point::new(b.0, b.1), Point::new(b.2, b.3));
        let mut l = lines.clone();
        keep_polylines_near_bbox(&mut l, &bb);
        r.out(format!("keep_polylines_near_bbox {} {} {}", pt(&bb.min), pt(&bb.max), pls_s(&l)));
    }
}

fn run_scenario(r: &mut Run, scenario: usize) {
    let s = format!("s{scenario}");
    let line = r.input(&format!("in {s} model")).expect("model");
    let mut t = Tok::new(&line);
    let (sx, sy, sz, ix, iy) = (t.i() as f64, t.i() as f64, t.i() as f64, t.i() as f64, t.i() as f64);
    let mut model = Model::new();
    {
        let o = model.add_object_new();
        o.name = "box".into();
        o.add_volume_mesh(ModelMesh::from_its(box_its(sx, sy, sz, -(sx as f32) / 2.0, -(sy as f32) / 2.0, 0.0)), ModelVolumeType::ModelPart, true);
        o.add_instance().set_offset(&slicer::geometry::geometry::Vec3d::new(ix, iy, 0.0));
    }
    let mut config = DynamicPrintConfig::full_print_config();
    config.set_num_filaments(3).unwrap();
    config.set_key_value(
        "filament_colour",
        Box::new(ConfigOptionStrings::from_values(vec!["#FF0000".into(), "#00FF00".into(), "#0000FF".into()])),
    );
    config.set_key_value("filament_map", Box::new(ConfigOptionInts::from_values(vec![1, 1, 1])));
    while let Some(line) = r.input(&format!("in {s} cfg")) {
        let (key, value) = line.split_once(' ').expect("key value");
        deser(&mut config, key, value);
    }
    let mut print = assemble(&model, &config);
    let mut obj = print.m_objects.remove(0);
    let sp = obj.slicing_parameters().clone();
    r.out(format!(
        "{s} slicing {} {} {} {} {} {} {} {} {} {} {} {} {}",
        hd(sp.layer_height),
        hd(sp.first_print_layer_height),
        sp.base_raft_layers,
        sp.interface_raft_layers,
        hd(sp.base_raft_layer_height),
        hd(sp.interface_raft_layer_height),
        hd(sp.contact_raft_layer_height),
        hd(sp.raft_base_top_z),
        hd(sp.raft_interface_top_z),
        hd(sp.raft_contact_top_z),
        hd(sp.object_print_z_min),
        hb(sp.soluble_interface),
        sp.raft_layers()
    ));

    let mut layer_id = 0usize;
    while let Some(line) = r.input(&format!("in {s} layer")) {
        let mut t = Tok::new(&line);
        let (print_z, height, slice_z) = (t.d(), t.d(), t.d());
        let mut layer = slicer::layer::Layer::new(layer_id, obj.m_model_object, height, print_z, slice_z);
        layer.lslices = t.expolys();
        obj.m_layers.push(layer);
        layer_id += 1;
    }

    let p = SupportParameters::new(&obj, &print).expect("SupportParameters");
    if scenario == 0 {
        run_path_helpers(r, &p);
    }

    let mut storage = SupportGeneratorLayerStorage::new();
    let line = r.input(&format!("in {s} storage")).expect("storage");
    let n = Tok::new(&line).i();
    for _ in 0..n {
        let line = r.input(&format!("in {s} L")).expect("L");
        read_layer(&mut storage, &line);
    }
    let line = r.input(&format!("in {s} lists")).expect("lists");
    let mut t = Tok::new(&line);
    let bottom_contacts = tok_list(&mut t);
    let top_contacts = tok_list(&mut t);
    let intermediate_layers = tok_list(&mut t);
    let mut top_interface_layers = tok_list(&mut t);
    let mut top_base_interface_layers = tok_list(&mut t);

    // compact_layers on a copy of the lists.
    {
        let mut v = top_interface_layers.clone();
        v.extend_from_slice(&top_contacts);
        v.push(NULL_LAYER);
        compact_layers(&mut v, &storage);
        r.out(format!("{s} compact_layers {}", list_s(&v)));
    }

    // With no intermediate layers the interfaces of the organic supports pass through (on copies).
    {
        let none: SupportGeneratorLayersPtr = Vec::new();
        let mut ti = top_interface_layers.clone();
        let mut tb = top_base_interface_layers.clone();
        if scenario == 7 {
            ti.remove(1);
        }
        let res = generate_interface_layers(obj.config(), &p, &bottom_contacts, &top_contacts, &mut ti, &mut tb, &none, &mut storage);
        r.out(format!("{s} interface_layers_no_base {} {} {} {}", list_s(&res.0), list_s(&res.1), list_s(&ti), list_s(&tb)));
    }
    if scenario == 7 {
        top_interface_layers.remove(1);
    }

    let (interface_layers, base_interface_layers) = generate_interface_layers(
        obj.config(),
        &p,
        &bottom_contacts,
        &top_contacts,
        &mut top_interface_layers,
        &mut top_base_interface_layers,
        &intermediate_layers,
        &mut storage,
    );
    dump_storage(r, &format!("{s} interface"), &storage);
    r.out(format!(
        "{s} interface lists {} {} {} {}",
        list_s(&interface_layers),
        list_s(&base_interface_layers),
        list_s(&top_interface_layers),
        list_s(&top_base_interface_layers)
    ));

    let raft_layers = generate_raft_base(&obj, &p, &sp, &top_contacts, &interface_layers, &base_interface_layers, &intermediate_layers, &mut storage);
    dump_storage(r, &format!("{s} raft"), &storage);
    r.out(format!("{s} raft lists {}", list_s(&raft_layers)));

    let layers_sorted = generate_support_layers(
        &mut obj,
        &raft_layers,
        &bottom_contacts,
        &top_contacts,
        &intermediate_layers,
        &interface_layers,
        &base_interface_layers,
        &mut storage,
    );
    dump_storage(r, &format!("{s} support_layers"), &storage);
    {
        let mut sorted = true;
        for i in 1..layers_sorted.len() {
            if storage[layers_sorted[i]] < storage[layers_sorted[i - 1]] {
                sorted = false;
            }
        }
        let mut idx = layers_sorted.clone();
        idx.sort();
        let mut line = format!("{s} layers_sorted {} {}", hb(sorted), idx.len());
        for i in idx {
            line += &format!(" {i}");
        }
        r.out(line);
        let mut line = format!("{s} object_support_layers {}", obj.support_layers().len());
        for sl in obj.support_layers() {
            line += &format!(" {} {} {} {} {}", sl.layer.id(), sl.interface_id(), hd(sl.layer.print_z), hd(sl.layer.height), hd(sl.layer.slice_z));
        }
        r.out(line);
    }

    // The toolpaths.
    {
        let line = r.input(&format!("in {s} toolpaths")).expect("toolpaths");
        let mut t = Tok::new(&line);
        let n = t.i();
        let cooldown_areas: Vec<ExPolygons> = (0..n).map(|_| t.expolys()).collect();
        let n = t.i();
        let mut lightning: Vec<Polylines> = (0..n).map(|_| tok_pls(&mut t)).collect();
        let n = t.i() as usize;
        assert_eq!(n, obj.m_support_layers.len());
        for sl in obj.m_support_layers.iter_mut() {
            sl.planned_nozzle_diameter = t.d();
            sl.planned_interface_nozzle_diameter = t.d();
            sl.interface_slab_height = t.d();
        }
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let factory = RecFactory { log: log.clone() };
        let config = obj.config().clone();
        generate_support_toolpaths(
            &mut obj.m_support_layers,
            &config,
            &p,
            &sp,
            &raft_layers,
            &bottom_contacts,
            &top_contacts,
            &intermediate_layers,
            &interface_layers,
            &base_interface_layers,
            &cooldown_areas,
            if scenario == 11 { Some(&mut lightning) } else { None },
            None,
            &mut storage,
            &factory,
        )
        .expect("generate_support_toolpaths");
        flush(r, &log);
        dump_storage(r, &format!("{s} toolpaths"), &storage);
        for sl in obj.support_layers() {
            r.out(format!("{s} toolpaths fills {} {} | {}", sl.layer.id(), ents_s(&sl.support_fills.entities), expolys_s(&sl.support_islands)));
        }
        let mut line = format!("{s} toolpaths lightning {}", lightning.len());
        for l in &lightning {
            line += &format!(" {}", pls_s(l));
        }
        r.out(line);
    }
}

// ---------------------------------------------------------------------------------------------------------
// merge_support_layers_multi_diameter, merged_band_footprint
// ---------------------------------------------------------------------------------------------------------

/// gen.cpp `CAND(c)`
fn cand_s(c: &Candidate) -> String {
    format!("{} {} {} {} {}", c.filament_id, hd(c.nozzle_diameter), hd(c.window.min_height), hd(c.window.max_height), c.band_size)
}
fn tok_cand(t: &mut Tok) -> Candidate {
    Candidate {
        filament_id: t.i() as i32,
        nozzle_diameter: t.d(),
        window: AdaptiveLayerHeightRange { min_height: t.d(), max_height: t.d() },
        band_size: t.i() as usize,
    }
}
/// gen.cpp `PLAN(plan)`
fn tok_plan(t: &mut Tok) -> SupportMergePlan {
    let mut plan = SupportMergePlan::default();
    plan.enabled = t.b();
    plan.interface_fold_enabled = t.b();
    assert_eq!(t.s(), "PZ");
    let n = t.i();
    plan.layer_print_z = (0..n).map(|_| t.d()).collect();
    assert_eq!(t.s(), "LH");
    let n = t.i();
    plan.layer_heights = (0..n).map(|_| t.d()).collect();
    assert_eq!(t.s(), "CT");
    let n = t.i();
    let table: Vec<Candidate> = (0..n).map(|_| tok_cand(t)).collect();
    for tag in ["CPL", "ICPL"] {
        assert_eq!(t.s(), tag);
        let layers = t.i();
        let list: Vec<Vec<Candidate>> = (0..layers)
            .map(|_| {
                let n = t.i();
                (0..n).map(|_| table[t.i() as usize]).collect()
            })
            .collect();
        if tag == "CPL" {
            plan.candidates_per_layer = list;
        } else {
            plan.interface_candidates_per_layer = list;
        }
    }
    assert_eq!(t.s(), "FB");
    for _ in 0..t.i() {
        let filament_id = t.i() as i32;
        let n = t.i();
        let ladder: Vec<AdaptiveLayerBand> = (0..n).map(|_| AdaptiveLayerBand { bottom_idx: t.i() as usize, top_idx: t.i() as usize, merged_height: t.d() }).collect();
        plan.fixed_bands_per_filament.insert(filament_id, ladder);
    }
    assert_eq!(t.s(), "SL");
    let n = t.i();
    plan.slabs = (0..n)
        .map(|_| Slab {
            bottom_idx: t.i() as usize,
            top_idx: t.i() as usize,
            top_z: t.d(),
            height: t.d(),
            filament_id: t.i() as i32,
            nozzle_diameter: t.d(),
            bottom_anchor: t.b(),
        })
        .collect();
    assert_eq!(t.s(), "SI");
    let n = t.i();
    plan.slab_index_per_layer = (0..n).map(|_| t.i() as i32).collect();
    plan
}
/// gen.cpp `CMAP(m)`
fn cmap_s(m: &BTreeMap<OrderedFloat<f64>, Candidate>) -> String {
    let mut o = format!("{}", m.len());
    for (k, v) in m {
        o += &format!(" {} {}", hd(k.0), cand_s(v));
    }
    o
}
/// gen.cpp `DMAP(m)`
fn dmap_s(m: &BTreeMap<OrderedFloat<f64>, f64>) -> String {
    let mut o = format!("{}", m.len());
    for (k, v) in m {
        o += &format!(" {} {}", hd(k.0), hd(*v));
    }
    o
}
/// gen.cpp `AREAS(v)`
fn areas_s(v: &[ExPolygons]) -> String {
    let mut o = format!("{}", v.len());
    for e in v {
        o += &format!(" {}", expolys_s(e));
    }
    o
}
fn tok_areas(t: &mut Tok) -> Vec<ExPolygons> {
    let n = t.i();
    (0..n).map(|_| t.expolys()).collect()
}

/// gen.cpp `run_merge(m, sc)`. The line "params": whether the Print uses several nozzle diameters, the gaps and
/// the heights of its slicing parameters, as `Print::apply` leaves them in the mode of several diameters.
fn run_merge(r: &mut Run, m: usize) {
    let s = format!("m{m}");
    let line = r.input(&format!("in {s} model")).expect("model");
    let mut t = Tok::new(&line);
    let (sx, sy, sz, ix, iy) = (t.i() as f64, t.i() as f64, t.i() as f64, t.i() as f64, t.i() as f64);
    let mut model = Model::new();
    {
        let o = model.add_object_new();
        o.name = "box".into();
        o.add_volume_mesh(ModelMesh::from_its(box_its(sx, sy, sz, -(sx as f32) / 2.0, -(sy as f32) / 2.0, 0.0)), ModelVolumeType::ModelPart, true);
        o.add_instance().set_offset(&slicer::geometry::geometry::Vec3d::new(ix, iy, 0.0));
    }
    let mut config = DynamicPrintConfig::full_print_config();
    config.set_num_filaments(3).unwrap();
    config.set_key_value(
        "filament_colour",
        Box::new(ConfigOptionStrings::from_values(vec!["#FF0000".into(), "#00FF00".into(), "#0000FF".into()])),
    );
    config.set_key_value("filament_map", Box::new(ConfigOptionInts::from_values(vec![1, 1, 1])));
    while let Some(line) = r.input(&format!("in {s} cfg")) {
        let (key, value) = line.split_once(' ').expect("key value");
        deser(&mut config, key, value);
    }
    let mut print = assemble(&model, &config);
    let mut sp = print.m_objects[0].slicing_parameters().clone();

    let mut layer_id = 0usize;
    while let Some(line) = r.input(&format!("in {s} layer")) {
        let mut t = Tok::new(&line);
        let (print_z, height, slice_z) = (t.d(), t.d(), t.d());
        let mut layer = Layer::new(layer_id, print.m_objects[0].m_model_object, height, print_z, slice_z);
        layer.lslices = t.expolys();
        let mut lr = LayerRegion::new(layer_id, 0);
        for e in &layer.lslices {
            lr.slices.surfaces.push(Surface::new(SurfaceType::Internal, e.clone()));
        }
        let n = t.i();
        for _ in 0..n {
            let mut patch = AdaptiveMergePatch::default();
            patch.bottom_idx = t.i() as usize;
            patch.top_idx = t.i() as usize;
            patch.set_area(t.expolys());
            lr.adaptive_merge_patches.push(patch);
        }
        layer.add_region(lr);
        print.m_objects[0].m_layers.push(layer);
        layer_id += 1;
    }
    let n = layer_id;
    let k = (n * 3) / 4;
    let obj = &print.m_objects[0];

    let line = r.input(&format!("in {s} plan")).expect("plan");
    let plan = tok_plan(&mut Tok::new(&line));
    let line = r.input(&format!("in {s} params")).expect("params");
    let mut t = Tok::new(&line);
    let mut p = SupportParameters::new(obj, &print).expect("SupportParameters");
    r.out(format!(
        "{s} params {} {} {} {} {} {}",
        hb(print.uses_multiple_nozzle_diameters()),
        hd(p.gap_xy),
        hd(sp.gap_support_object),
        hd(sp.raft_contact_top_z),
        hd(sp.object_print_z_min),
        hd(sp.layer_height)
    ));
    // What the merge reads of the two, as the C++ had it.
    p.gap_xy = t.d();
    sp.gap_support_object = t.d();
    sp.raft_contact_top_z = t.d();

    for i in 0..n {
        if i % 3 == 0 || (i + 7 >= k && i < k) {
            r.out(format!("{s} band {i} {}", polys_s(&merged_band_footprint(&obj.layers()[i], i))));
        }
    }

    let mut storage = SupportGeneratorLayerStorage::new();
    let line = r.input(&format!("in {s} storage")).expect("storage");
    let count = Tok::new(&line).i();
    for _ in 0..count {
        let line = r.input(&format!("in {s} L")).expect("L");
        read_layer(&mut storage, &line);
    }
    let line = r.input(&format!("in {s} lists")).expect("lists");
    let mut t = Tok::new(&line);
    let raft_layers = tok_list(&mut t);
    let mut bottom_contacts = tok_list(&mut t);
    let mut top_contacts = tok_list(&mut t);
    let mut intermediate_layers = tok_list(&mut t);
    let mut interface_layers = tok_list(&mut t);
    let mut base_interface_layers = tok_list(&mut t);

    let line = r.input(&format!("in {s} areas")).expect("areas");
    let mut t = Tok::new(&line);
    let by_z = t.b();
    let nz = t.i();
    let area_z: Vec<f64> = (0..nz).map(|_| t.d()).collect();
    let mut cooldown = tok_areas(&mut t);
    let mut lightning = tok_areas(&mut t);
    let top_gap = t.d();

    let mut planned_filaments: BTreeMap<OrderedFloat<f64>, Candidate> = BTreeMap::new();
    let mut planned_interface_filaments: BTreeMap<OrderedFloat<f64>, Candidate> = BTreeMap::new();
    let mut planned_base_heights: BTreeMap<OrderedFloat<f64>, f64> = BTreeMap::new();
    let mut planned_interface_heights: BTreeMap<OrderedFloat<f64>, f64> = BTreeMap::new();
    merge_support_layers_multi_diameter(
        obj,
        &plan,
        &p,
        &sp,
        &raft_layers,
        &mut bottom_contacts,
        &mut top_contacts,
        &mut intermediate_layers,
        &mut interface_layers,
        &mut base_interface_layers,
        &mut planned_filaments,
        &mut planned_interface_filaments,
        &mut planned_base_heights,
        &mut planned_interface_heights,
        if by_z { Some(&area_z) } else { None },
        Some(&mut cooldown),
        if m == 4 { None } else { Some(&mut lightning) },
        top_gap,
        &mut storage,
    );
    dump_storage(r, &format!("{s} merge"), &storage);
    r.out(format!(
        "{s} merge lists {} {} {} {} {} {}",
        list_s(&raft_layers),
        list_s(&bottom_contacts),
        list_s(&top_contacts),
        list_s(&intermediate_layers),
        list_s(&interface_layers),
        list_s(&base_interface_layers)
    ));
    r.out(format!(
        "{s} merge planned {} | {} | {} | {}",
        cmap_s(&planned_filaments),
        cmap_s(&planned_interface_filaments),
        dmap_s(&planned_base_heights),
        dmap_s(&planned_interface_heights)
    ));
    r.out(format!("{s} merge areas {} | {}", areas_s(&cooldown), areas_s(&lightning)));
}

#[test]
fn support_common_native_parity() {
    let mut r = Run::load("support_common");
    run_helpers(&mut r);
    let mut scenario = 0;
    while r.expected[r.pos..].iter().any(|l| l.starts_with(&format!("in s{scenario} model"))) {
        run_scenario(&mut r, scenario);
        scenario += 1;
    }
    assert_eq!(scenario, 14, "scenarios");
    let mut merges = 0;
    while r.expected[r.pos..].iter().any(|l| l.starts_with(&format!("in m{merges} model"))) {
        run_merge(&mut r, merges);
        merges += 1;
    }
    assert_eq!(merges, 8, "merge scenarios");
    let n = r.expected.iter().filter(|l| !l.starts_with("in ")).count();
    for tag in [
        "safe_union ",
        "safe_union_ex ",
        "safe_union_raw ",
        "safe_offset_inc ",
        "safe_offset_inc_ex ",
        "draw_perimeters ",
        "keep_polylines_near_bbox ",
        "tsgp ",
        "fesg ",
        "lip ",
        "extruded_merge ",
        "mod ",
        "modp ",
        "idx ",
        "fegp ",
        "fesgp ",
        "fesgd ",
        "fill_surface ",
    ] {
        let (total, ok) = r.tally(tag);
        eprintln!("support_common: {tag}: {ok} of {total}");
    }
    for stage in ["slicing", "compact_layers", "interface_layers_no_base", "interface", "raft", "support_layers", "layers_sorted", "object_support_layers", "toolpaths"] {
        let (mut total, mut ok) = (0, 0);
        for sc in 0..scenario {
            let (t, o) = r.tally(&format!("s{sc} {stage} "));
            total += t;
            ok += o;
        }
        eprintln!("support_common: {stage}: {ok} of {total}");
    }
    for stage in ["params", "band", "merge L", "merge lists", "merge planned", "merge areas"] {
        let (mut total, mut ok) = (0, 0);
        for sc in 0..merges {
            let (t, o) = r.tally(&format!("m{sc} {stage} "));
            total += t;
            ok += o;
        }
        eprintln!("support_common: {stage}: {ok} of {total}");
    }
    let bad = r.finish();
    eprintln!("support_common: {} result lines over {} scenarios and {} merge scenarios, {} differ", n, scenario, merges, bad);
    assert_eq!(bad, 0);
}
