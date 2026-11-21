//! Native parity of the Support/TreeSupport3D.cpp/.hpp port (`slicer::support::tree_support_3d`) against
//! vectors made by the C++ at the pin (tests/data/tree_support_3d/gen.cpp).
//!
//! The generator runs the helpers that take no volumes on polylines, polygons and points of a linear
//! congruential generator (`polyline_sample_next_point_at_distance`, `ensure_maximum_distance_polyline`,
//! `move_inside`, `move_inside_if_outside`, `getEffectiveDTT`, the `support_element_*radius` functions,
//! `SupportElementState::propagate_down`), and for eighteen scenarios builds a Print of one box with
//! `Print::apply`, fills in the layers of its object and their overhangs by hand and runs, on one thread:
//! `group_meshes`, the constructor of `TreeModelVolumes`, `precalculate`, `get_avoidance_status`,
//! `convert_lines_to_internal`, `evaluate_point_for_next_layer_function`, `split_lines`,
//! `generate_support_infill_lines`, `generate_initial_areas` (with `generate_raft_contact`,
//! `sample_overhang_area`, `RichInterfacePlacer` and `finalize_raft_contact`), `create_layer_pathing`
//! (with `increase_areas_one_layer`, `increase_single_area` and `merge_influence_areas`),
//! `create_nodes_from_area` (with `set_points_on_areas`, `set_to_model_contact` and
//! `remove_deleted_elements`), `generate_branch_areas`, `smooth_branch_areas` and `draw_areas` (with
//! `drop_non_gracious_areas` and `finalize_interface_and_support_areas`).
//!
//! The fillers are the generator's `RecFill`, which this test has as well: everything around the fillers
//! is compared, not the fillers of the engine. The lines of the vectors that start with "in" are the
//! inputs, which this test reads back; every other line it has to reproduce, bit for bit. The long lines
//! are compared by their FNV-1a hash; `TREE_SUPPORT_3D_PARITY_VECTORS` names a file of the generator with
//! every line in full (`gen <file> full`), for finding the first token that differs.

mod support_shared;

use slicer::build_volume::BuildVolume;
use slicer::config::{ConfigOptionInts, ConfigOptionStrings};
use slicer::fill::fill_base::Fill;
use slicer::fill::FillParams;
use slicer::geometry::{Point, Polygon, Polygons, Polyline, Polylines};
use slicer::model::*;
use slicer::print_config::config_def::DynamicPrintConfig;
use slicer::print_config::InfillPattern;
use slicer::slicing::SlicingParams;
use slicer::support::support_common::cu;
use slicer::support::support_common::{InfillFailedException, SupportFill, SupportFillFactory};
use slicer::support::support_layer::{SupportGeneratorLayerStorage, SupportGeneratorLayersPtr};
use slicer::support::support_parameters::SupportParameters;
use slicer::support::tree_model_volumes::{AvoidanceType, TreeModelVolumes};
use slicer::support::tree_support_3d::*;
use slicer::support::tree_support_common::{
    InterfacePlacer, LayerIndex, LineStatus, TreeSupportMeshGroupSettings, TreeSupportSettings, SOLUBLE, SUPPORT_GENERATOR_LAYER_PTR_NULL,
};
use slicer::surface::Surface;
use std::cell::RefCell;
use std::rc::Rc;
use support_shared::*;

/// gen.cpp `fnv(s)`
fn fnv(s: &str) -> String {
    let mut h: u64 = 1469598103934665603;
    for c in s.bytes() {
        h ^= c as u64;
        h = h.wrapping_mul(1099511628211);
    }
    format!("{h:016x}")
}

struct Ctx {
    r: Run,
    full: bool,
    /// The scenarios whose draw_areas() stopped at a debug assertion.
    draw_stopped: usize,
    /// The calls of smooth_outward() by this test that stopped at a debug assertion.
    smooth_outward_stopped: usize,
}
impl Ctx {
    /// gen.cpp `outb(head, body)`
    fn outb(&mut self, head: String, body: String) {
        if self.full {
            self.r.out(format!("{head} {body}"));
        } else {
            self.r.out(format!("{head} #{}", fnv(&body)));
        }
    }
}

/// gen.cpp `PTS(pts)`
fn pts_s(pts: &[Point]) -> String {
    let mut o = format!("{}", pts.len());
    for q in pts {
        o += &format!(" {}", pt(q));
    }
    o
}
/// gen.cpp `PLS(ps)`
fn pls_s(ps: &[Polyline]) -> String {
    let mut o = format!("{}", ps.len());
    for p in ps {
        o += &format!(" {}", pts_s(&p.points));
    }
    o
}
/// gen.cpp `LINES(lines)`
fn lines_s(lines: &[LineInformation]) -> String {
    let mut o = format!("{}", lines.len());
    for l in lines {
        o += &format!(" {}", l.len());
        for p in l {
            o += &format!(" {} {}", pt(&p.0), p.1 as i32);
        }
    }
    o
}

fn tok_pts(t: &mut Tok) -> Vec<Point> {
    let n = t.i();
    (0..n).map(|_| t.pt()).collect()
}
fn tok_pls(t: &mut Tok) -> Polylines {
    let n = t.i();
    (0..n).map(|_| Polyline::from_points(tok_pts(t))).collect()
}

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
            "{} {} {} {} {} {} {}",
            self.pattern as i32,
            hf(f.angle),
            hd(f.spacing),
            f.layer_id as i64,
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
        let mut step: i64 = ((f.spacing / slicer::libslic3r::SCALING_FACTOR) / d) as i64;
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

/// gen.cpp `flush_fills(tag)`
fn flush_fills(c: &mut Ctx, tag: &str, log: &Log) {
    let fills: Vec<String> = std::mem::take(&mut *log.borrow_mut());
    if c.full {
        for l in &fills {
            c.r.out(format!("{tag} fill {l}"));
        }
    }
    let mut body = String::new();
    for l in &fills {
        body += l;
        body += "\n";
    }
    c.r.out(format!("{tag} fills {} #{}", fills.len(), fnv(&body)));
}

fn avoidance_type_from(v: i64) -> AvoidanceType {
    match v {
        0 => AvoidanceType::Slow,
        1 => AvoidanceType::FastSafe,
        2 => AvoidanceType::Fast,
        _ => panic!("AvoidanceType {v}"),
    }
}

/// gen.cpp `STATE(s)`
fn state_s(s: &SupportElementState) -> String {
    format!(
        "{}{}{}{}{}{}{}{}{}{} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {}{}{}{} {}",
        hb(s.to_buildplate),
        hb(s.to_model_gracious),
        hb(s.use_min_xy_dist),
        hb(s.supports_roof),
        hb(s.can_use_safe_radius),
        hb(s.skip_ovalisation),
        hb(s.lost),
        hb(s.verylost),
        hb(s.deleted),
        hb(s.marked),
        s.type_,
        hd(s.radius),
        hf(s.print_z),
        s.xy_min_distance,
        s.target_height,
        pt(&s.target_position),
        pt(&s.next_position),
        s.layer_idx,
        s.effective_radius_height,
        s.distance_to_top,
        pt(&s.result_on_layer),
        s.increased_to_model_radius,
        hd(s.elephant_foot_increases),
        s.dont_move_until,
        s.last_area_increase.type_ as i32,
        s.last_area_increase.increase_speed,
        hb(s.last_area_increase.increase_radius),
        hb(s.last_area_increase.no_error),
        hb(s.last_area_increase.use_min_distance),
        hb(s.last_area_increase.move_),
        s.missing_roof_layers
    )
}

/// The state of an "in state" line.
fn tok_state(t: &mut Tok) -> SupportElementState {
    let mut s = SupportElementState::default();
    let bits: Vec<bool> = t.s().bytes().map(|b| b == b'1').collect();
    s.to_buildplate = bits[0];
    s.to_model_gracious = bits[1];
    s.use_min_xy_dist = bits[2];
    s.supports_roof = bits[3];
    s.can_use_safe_radius = bits[4];
    s.skip_ovalisation = bits[5];
    s.lost = bits[6];
    s.verylost = bits[7];
    s.deleted = bits[8];
    s.marked = bits[9];
    s.type_ = t.i() as i32;
    s.radius = t.d();
    s.print_z = t.f();
    s.xy_min_distance = t.i();
    s.target_height = t.i() as LayerIndex;
    s.target_position = t.pt();
    s.next_position = t.pt();
    s.layer_idx = t.i() as LayerIndex;
    s.effective_radius_height = t.i() as u32;
    s.distance_to_top = t.i() as u32;
    s.result_on_layer = t.pt();
    s.increased_to_model_radius = t.i();
    s.elephant_foot_increases = t.d();
    s.dont_move_until = t.i() as u32;
    s.last_area_increase.type_ = avoidance_type_from(t.i());
    s.last_area_increase.increase_speed = t.i();
    let bits: Vec<bool> = t.s().bytes().map(|b| b == b'1').collect();
    s.last_area_increase.increase_radius = bits[0];
    s.last_area_increase.no_error = bits[1];
    s.last_area_increase.use_min_distance = bits[2];
    s.last_area_increase.move_ = bits[3];
    s.missing_roof_layers = t.i() as u32;
    s
}

/// gen.cpp `ELEMENT(e)`
fn element_s(e: &SupportElement) -> String {
    let mut o = format!("{} p {}", state_s(&e.state), e.parents.len());
    for p in &e.parents {
        o += &format!(" {p}");
    }
    o + " a " + &polys_s(&e.influence_area)
}

/// gen.cpp `dump_move_bounds(tag, move_bounds)`
fn dump_move_bounds(c: &mut Ctx, tag: &str, move_bounds: &[SupportElements]) {
    for (layer_idx, layer) in move_bounds.iter().enumerate() {
        if layer.is_empty() {
            continue;
        }
        if c.full {
            c.r.out(format!("{tag} layer {layer_idx} {}", layer.len()));
            for e in layer {
                c.r.out(format!("{tag} element {layer_idx} {}", element_s(e)));
            }
        } else {
            let mut all = String::new();
            for e in layer {
                all += &element_s(e);
                all += "\n";
            }
            c.r.out(format!("{tag} layer {layer_idx} {} #{}", layer.len(), fnv(&all)));
        }
    }
}

/// gen.cpp `dump_draw_areas(tag, move_bounds, linear_data, linear_data_layers)`
fn dump_draw_areas(c: &mut Ctx, tag: &str, move_bounds: &[SupportElements], linear_data: &[DrawArea], linear_data_layers: &[usize]) {
    for layer_idx in 0..linear_data_layers.len().saturating_sub(1) {
        let (begin, end) = (linear_data_layers[layer_idx], linear_data_layers[layer_idx + 1]);
        if begin == end {
            continue;
        }
        let mut all = String::new();
        for j in begin..end {
            let a = &linear_data[j];
            let child: i64 = a.child_element.map_or(-1, |ch| ch.1 as i64);
            let line = format!("{child} {} {}", hb(move_bounds[a.element.0][a.element.1].state.marked), polys_s(&a.polygons));
            if c.full {
                c.r.out(format!("{tag} area {layer_idx} {} {line}", j - begin));
            }
            all += &line;
            all += "\n";
        }
        c.r.out(format!("{tag} layer {layer_idx} {} #{}", end - begin, fnv(&all)));
    }
}

/// gen.cpp `dump_generator_layers(tag, layers)`
fn dump_generator_layers(c: &mut Ctx, tag: &str, storage: &SupportGeneratorLayerStorage, layers: &SupportGeneratorLayersPtr) {
    c.r.out(format!("{tag} {}", layers.len()));
    for (i, &l) in layers.iter().enumerate() {
        if l != SUPPORT_GENERATOR_LAYER_PTR_NULL {
            let l = &storage[l];
            c.outb(format!("{tag} layer {i} {} {} {} {}", l.layer_type as i32, hd(l.print_z), hd(l.bottom_z), hd(l.height)), polys_s(&l.polygons));
        }
    }
}

fn run_scenario(c: &mut Ctx, scenario: usize) {
    let s = format!("s{scenario}");
    let line = c.r.input(&format!("in {s} model")).expect("model");
    let mut t = Tok::new(&line);
    let (sx, sy, sz, ix, iy) = (t.i() as f64, t.i() as f64, t.i() as f64, t.i() as f64, t.i() as f64);
    let mut model = Model::new();
    {
        let o = model.add_object_new();
        o.name = "tee".into();
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
    while let Some(line) = c.r.input(&format!("in {s} cfg")) {
        let (key, value) = line.split_once(' ').expect("key value");
        deser(&mut config, key, value);
    }
    let mut print = assemble(&model, &config);

    let mut layer_overhangs: Vec<Polygons> = Vec::new();
    let mut layer_id = 0usize;
    while let Some(line) = c.r.input(&format!("in {s} layer")) {
        let mut t = Tok::new(&line);
        let (print_z, height, slice_z) = (t.d(), t.d(), t.d());
        let obj = &mut print.m_objects[0];
        let mut layer = slicer::layer::Layer::new(layer_id, obj.m_model_object, height, print_z, slice_z);
        layer.lslices = t.expolys();
        layer.lslices_extrudable = layer.lslices.clone();
        layer_overhangs.push(t.polys());
        if layer_id > 0 {
            layer.lower_layer_id = Some(layer_id - 1);
            obj.m_layers[layer_id - 1].upper_layer_id = Some(layer_id);
        }
        obj.m_layers.push(layer);
        layer_id += 1;
    }

    // group_meshes()
    SOLUBLE.store(false, std::sync::atomic::Ordering::Relaxed);
    let grouped_meshes = group_meshes(&print, &[0]).expect("group_meshes");
    let cfg: &TreeSupportSettings = &grouped_meshes[0].0;
    c.r.out(format!(
        "{s} group_meshes {} {} {} {} {} {} {} {} {} {}",
        grouped_meshes.len(),
        grouped_meshes[0].1.len(),
        grouped_meshes[0].1[0],
        hb(SOLUBLE.load(std::sync::atomic::Ordering::Relaxed)),
        cfg.min_radius,
        cfg.branch_radius,
        cfg.xy_distance,
        cfg.xy_min_distance,
        cfg.z_distance_top_layers,
        cfg.raft_layers.len()
    ));

    // The build volume and the volumes.
    let line = c.r.input(&format!("in {s} bed")).expect("bed");
    let mut t = Tok::new(&line);
    let printable_height = t.d();
    let bed = t.poly();
    let bedptsf: Vec<slicer::geometry::PointF> = bed
        .points
        .iter()
        .map(|p| slicer::geometry::PointF::new(p.x as f64 * slicer::libslic3r::SCALING_FACTOR, p.y as f64 * slicer::libslic3r::SCALING_FACTOR))
        .collect();
    let build_volume = BuildVolume::new_from_config(bedptsf, printable_height, Vec::new(), Vec::new());
    let obj = &print.m_objects[0];
    let mut volumes = TreeModelVolumes::from_print_object(obj, &print, &build_volume, cfg.maximum_move_distance, cfg.maximum_move_distance_slow, 0, &[])
        .expect("TreeModelVolumes");

    // The overhangs, by the index of the support layer.
    let num_raft_layers = cfg.raft_layers.len();
    let num_layers = obj.layer_count() + num_raft_layers;
    let mut overhangs: Vec<Polygons> = vec![Vec::new(); num_layers];
    for (i, o) in layer_overhangs.iter().enumerate() {
        overhangs[i + num_raft_layers] = o.clone();
    }

    // precalculate()
    let mut num_support_layers = precalculate(&print, &overhangs, cfg, &[0], &mut volumes, &|| {}) as usize;
    c.r.out(format!("{s} precalculate {num_support_layers} {}", hb(volumes.members().m_precalculated)));
    let has_support = num_support_layers > 0;
    let has_raft = !cfg.raft_layers.is_empty();
    num_support_layers = std::cmp::max(num_support_layers, cfg.raft_layers.len());
    if num_support_layers == 0 {
        return;
    }

    let mut support_params = SupportParameters::new(obj, &print).expect("SupportParameters");
    support_params.with_sheath = true;
    let log: Log = Rc::new(RefCell::new(Vec::new()));
    let factory = RecFactory { log: log.clone() };

    // The helpers that query the volumes.
    {
        let top = num_support_layers as LayerIndex - 1;
        let line = c.r.input(&format!("in {s} points")).expect("points");
        let pts = tok_pts(&mut Tok::new(&line));
        let mut line = format!("{s} get_avoidance_status");
        for p in &pts {
            for r in [cfg.min_radius, cfg.branch_radius] {
                for l in [1, top / 2, top] {
                    for xy in [-1, 0, cfg.xy_min_distance + 5000] {
                        line += &format!(" {}", get_avoidance_status(p, r, l, &volumes, cfg, xy) as i32);
                    }
                }
            }
        }
        c.r.out(line);
        let line = c.r.input(&format!("in {s} polylines")).expect("polylines");
        let polylines = tok_pls(&mut Tok::new(&line));
        for l in [1, top / 2, top] {
            for xy in [-1, cfg.xy_min_distance + 5000] {
                let lines = convert_lines_to_internal(&volumes, cfg, &polylines, l, xy);
                c.outb(format!("{s} convert_lines_to_internal {l} {xy}"), lines_s(&lines));
                let mut ev = String::new();
                for li in &lines {
                    for p in li {
                        ev += hb(evaluate_point_for_next_layer_function(&volumes, cfg, l as usize, p, xy));
                    }
                }
                c.outb(format!("{s} evaluate_point_for_next_layer_function {l} {xy}"), ev);
                let split = split_lines(&lines, |p| evaluate_point_for_next_layer_function(&volumes, cfg, l as usize, p, xy));
                c.outb(format!("{s} split_lines {l} {xy}"), format!("{} | {}", lines_s(&split.0), lines_s(&split.1)));
            }
        }
        // generate_support_infill_lines()
        log.borrow_mut().clear();
        let mut areas: Vec<Polygons> = Vec::new();
        while let Some(line) = c.r.input(&format!("in {s} area")) {
            areas.push(Tok::new(&line).polys());
        }
        for (a, area) in areas.iter().enumerate() {
            for roof in [false, true] {
                for l in [4, 5] {
                    for dist in [100000i64, 254321] {
                        c.outb(
                            format!("{s} generate_support_infill_lines {a} {} {l} {dist}", hb(roof)),
                            pls_s(&generate_support_infill_lines(area, &support_params, roof, l, dist, &factory)),
                        );
                    }
                }
            }
        }
        flush_fills(c, &format!("{s} generate_support_infill_lines"), &log);
    }

    // generate_initial_areas(), as generate_support_areas() calls it.
    let sp = obj.slicing_parameters().clone();
    let mut layer_storage = SupportGeneratorLayerStorage::new();
    let mut top_contacts: SupportGeneratorLayersPtr = Vec::new();
    let mut interface_layers: SupportGeneratorLayersPtr = Vec::new();
    let mut base_interface_layers: SupportGeneratorLayersPtr = Vec::new();
    if support_params.has_top_contacts || has_raft {
        top_contacts = vec![SUPPORT_GENERATOR_LAYER_PTR_NULL; num_support_layers];
    }
    if support_params.has_interfaces() || has_raft {
        interface_layers = vec![SUPPORT_GENERATOR_LAYER_PTR_NULL; num_support_layers];
    }
    if support_params.has_base_interfaces() || has_raft {
        base_interface_layers = vec![SUPPORT_GENERATOR_LAYER_PTR_NULL; num_support_layers];
    }
    let mut interface_placer = InterfacePlacer::new(&sp, &support_params, cfg);
    c.r.out(format!(
        "{s} support_params {} {} {} {} {} {} {} {}",
        hb(support_params.has_top_contacts),
        hb(support_params.has_interfaces()),
        hb(support_params.has_base_interfaces()),
        hb(support_params.soluble_interface),
        support_params.num_top_interface_layers,
        hb(has_support),
        hb(has_raft),
        num_support_layers
    ));
    if !has_support {
        return;
    }
    let mut move_bounds: Vec<SupportElements> = vec![Vec::new(); num_support_layers];
    log.borrow_mut().clear();
    generate_initial_areas(
        obj,
        &print,
        &volumes,
        cfg,
        &overhangs,
        &mut move_bounds,
        &mut interface_placer,
        InterfacePlacerOutputs {
            layer_storage: &mut layer_storage,
            top_contacts: &mut top_contacts,
            top_interfaces: &mut interface_layers,
            top_base_interfaces: &mut base_interface_layers,
        },
        &factory,
        &|| {},
    )
    .expect("generate_initial_areas");
    flush_fills(c, &format!("{s} initial"), &log);
    dump_move_bounds(c, &format!("{s} initial"), &move_bounds);
    dump_generator_layers(c, &format!("{s} initial top_contacts"), &layer_storage, &top_contacts);
    dump_generator_layers(c, &format!("{s} initial interface_layers"), &layer_storage, &interface_layers);
    dump_generator_layers(c, &format!("{s} initial base_interface_layers"), &layer_storage, &base_interface_layers);

    // create_layer_pathing(), with the number of threads the pin saw (the size of its first buckets).
    let threads: usize = c.r.input(&format!("in {s} threads")).expect("threads").trim().parse().expect("threads");
    MERGE_INFLUENCE_AREAS_NUM_THREADS.store(threads, std::sync::atomic::Ordering::Relaxed);
    create_layer_pathing(&volumes, cfg, &mut move_bounds, &|| {});
    dump_move_bounds(c, &format!("{s} pathing"), &move_bounds);

    // create_nodes_from_area()
    create_nodes_from_area(&volumes, cfg, &mut move_bounds, &|| {});
    dump_move_bounds(c, &format!("{s} nodes"), &move_bounds);

    // generate_branch_areas() and smooth_branch_areas() on a copy of the elements, then draw_areas() itself.
    {
        let mut mb: Vec<SupportElements> = move_bounds.clone();
        let (mut linear_data, linear_data_layers) = draw_areas_linear_data(&mb);
        generate_branch_areas(&volumes, cfg, &mb, &mut linear_data, &|| {});
        dump_draw_areas(c, &format!("{s} branch"), &mb, &linear_data, &linear_data_layers);
        smooth_branch_areas(cfg, &mut mb, &mut linear_data, &linear_data_layers, &|| {});
        dump_draw_areas(c, &format!("{s} smooth"), &mb, &linear_data, &linear_data_layers);
        // The union of the areas of every layer (required) and smooth_outward() of it (reported, see the end
        // of the test).
        for layer_idx in 0..linear_data_layers.len().saturating_sub(1) {
            let mut all: Polygons = Vec::new();
            for a in &linear_data[linear_data_layers[layer_idx]..linear_data_layers[layer_idx + 1]] {
                all.extend(a.polygons.iter().cloned());
            }
            if all.is_empty() {
                continue;
            }
            let u = cu::union_(&all);
            c.outb(format!("{s} layer_union {layer_idx}"), polys_s(&u));
            let width = cfg.support_line_width;
            match std::panic::catch_unwind(|| slicer::mutable_polygon::smooth_outward_polygons(u, width)) {
                Ok(smoothed) => c.outb(format!("{s} smooth_outward {layer_idx}"), polys_s(&smoothed)),
                Err(_) => {
                    c.smooth_outward_stopped += 1;
                    c.r.out(format!("{s} smooth_outward {layer_idx} stopped at a debug assertion"));
                }
            }
        }
    }
    let mut bottom_contacts: SupportGeneratorLayersPtr = Vec::new();
    let mut intermediate_layers: SupportGeneratorLayersPtr = vec![SUPPORT_GENERATOR_LAYER_PTR_NULL; num_support_layers];
    if support_params.has_bottom_contacts {
        bottom_contacts = vec![SUPPORT_GENERATOR_LAYER_PTR_NULL; num_support_layers];
    }
    // draw_areas() ends in finalize_interface_and_support_areas(), which smooths the areas of a layer with
    // smooth_outward() of mutable_polygon.rs. That port is not equal to the pin yet (see the end of the test):
    // a debug assertion of it (MutablePolygon.cpp:242 is an `assert`, which the pin's release build does not
    // have) stops some scenarios here. The stop is counted and the scenario goes on with what it has.
    let stopped = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        draw_areas(
            obj,
            &volumes,
            cfg,
            &overhangs,
            &mut move_bounds,
            &mut bottom_contacts,
            &mut top_contacts,
            &mut intermediate_layers,
            &mut layer_storage,
            &|| {},
        );
    }))
    .is_err();
    if stopped {
        c.draw_stopped += 1;
    }
    dump_generator_layers(c, &format!("{s} draw bottom_contacts"), &layer_storage, &bottom_contacts);
    dump_generator_layers(c, &format!("{s} draw top_contacts"), &layer_storage, &top_contacts);
    dump_generator_layers(c, &format!("{s} draw intermediate_layers"), &layer_storage, &intermediate_layers);
}

/// gen.cpp `run_helpers()`
fn run_helpers(c: &mut Ctx) {
    // polyline_sample_next_point_at_distance(), ensure_maximum_distance_polyline()
    let mut i = 0usize;
    while let Some(line) = c.r.input("in polyline") {
        let pl = Polyline::from_points(tok_pts(&mut Tok::new(&line)));
        let mut line = format!("polyline_sample_next_point_at_distance {i}");
        let mut start_idx = 0usize;
        while start_idx + 1 < pl.points.len() {
            for dist in [30000., 150000.5, 799999.25, 2500000.] {
                let start_pt = if start_idx % 4 == 0 {
                    pl.points[start_idx]
                } else {
                    // `Point((a + b) / 2)`: a division of integers.
                    Point::new((pl.points[start_idx].x + pl.points[start_idx + 1].x) / 2, (pl.points[start_idx].y + pl.points[start_idx + 1].y) / 2)
                };
                match polyline_sample_next_point_at_distance(&pl.points, &start_pt, start_idx, dist) {
                    Some(r) => line += &format!(" {} {}", pt(&r.0), r.1),
                    None => line += " -",
                }
            }
            start_idx += 2;
        }
        c.r.out(line);
        for distance in [50000., 200000., 612345.5] {
            for min_points in [0usize, 1, 2, 3, 5] {
                c.r.out(format!(
                    "ensure_maximum_distance_polyline {i} {} {min_points} {}",
                    hd(distance),
                    pls_s(&ensure_maximum_distance_polyline(&[pl.clone(), Polyline::new(), pl.clone()], distance, min_points))
                ));
            }
        }
        i += 1;
    }
    // move_inside(), move_inside_if_outside()
    let mut i = 0usize;
    while let Some(line) = c.r.input("in polygons") {
        let polygons: Vec<Polygon> = Tok::new(&line).polys();
        let mut line = format!("move_inside {i}");
        while let Some(l) = c.r.input("in from") {
            let from0 = Tok::new(&l).pt();
            for distance in [0, 500, -500, 30000, -30000] {
                for max_dist2 in [i64::MAX, 200000i64 * 200000] {
                    let mut from = from0;
                    let r = move_inside(&polygons, &mut from, distance, max_dist2);
                    line += &format!(" {} {}", r, pt(&from));
                }
            }
            line += &format!(" {}", pt(&move_inside_if_outside(&polygons, from0, 0, i64::MAX)));
        }
        c.r.out(line);
        i += 1;
    }
    // getEffectiveDTT(), support_element_radius(), support_element_collision_radius(),
    // support_element_model_collision_radius(), SupportElementState::propagate_down()
    {
        let mut m = TreeSupportMeshGroupSettings::default();
        m.support_tree_branch_diameter_angle = 0.14;
        m.support_tree_tip_diameter = 60000;
        m.support_xy_distance_overhang = 15000;
        let mut sp = SlicingParams::default();
        sp.layer_height = 0.2;
        let cfg = TreeSupportSettings::new(&m, &sp);
        let line = c.r.input("in settings").expect("settings");
        assert_eq!(
            line,
            format!(
                "{} {} {} {} {} {} {}",
                cfg.min_radius,
                cfg.branch_radius,
                cfg.tip_layers,
                cfg.increase_radius_until_layer,
                hd(cfg.branch_radius_increase_per_layer),
                hd(cfg.bp_radius_increase_per_layer),
                cfg.xy_min_distance
            ),
            "the settings of the helpers"
        );
        let mut i = 0usize;
        while let Some(line) = c.r.input("in state") {
            let s = tok_state(&mut Tok::new(&line));
            assert_eq!(state_s(&s), line, "a state read back");
            let e = SupportElement::new(s.clone(), Vec::new());
            let d = SupportElementState::propagate_down(&s);
            c.r.out(format!(
                "support_element {i} {} {} {} {} {} {} {} {}",
                get_effective_dtt(&cfg, &s),
                support_element_radius(&cfg, &s),
                support_element_collision_radius(&cfg, &s),
                support_element_model_collision_radius(&cfg, &s),
                support_element_radius(&cfg, &e),
                support_element_collision_radius(&cfg, &e),
                hb(s.result_on_layer_is_set()),
                state_s(&d)
            ));
            i += 1;
        }
    }
    c.r.out(format!("tiny_area_threshold {}", hd(tiny_area_threshold())));
    // circles_intersect()
    {
        use slicer::geometry::geometry::Vec3d;
        let mut line = String::from("circles_intersect ");
        while let Some(l) = c.r.input("in circles") {
            let mut t = Tok::new(&l);
            let mut v3 = |t: &mut Tok| Vec3d::new(t.d(), t.d(), t.d());
            let p1 = v3(&mut t);
            let n1 = v3(&mut t);
            let r1 = t.d();
            let p2 = v3(&mut t);
            let n2 = v3(&mut t);
            let r2 = t.d();
            line += if circles_intersect(&p1, &n1, r1, &p2, &n2, r2) { "1" } else { "0" };
        }
        c.r.out(line);
    }
    // discretize_circle(), triangulate_fan(), triangulate_strip()
    {
        use slicer::normal_utils::{indexed_triangle_set, Vec3f};
        let f3 = |v: &Vec3f| format!("{} {} {}", hf(v.x), hf(v.y), hf(v.z));
        let mut i = 0usize;
        while let Some(l) = c.r.input("in mesh") {
            let mut t = Tok::new(&l);
            let mut v3 = |t: &mut Tok| Vec3f::new(t.f(), t.f(), t.f());
            let c1 = v3(&mut t);
            let n1 = v3(&mut t);
            let r1 = t.f();
            let c2 = v3(&mut t);
            let n2 = v3(&mut t);
            let r2 = t.f();
            let eps = t.f();
            let mut its = indexed_triangle_set::default();
            its.vertices.push(c1);
            let a = discretize_circle(&c1, &n1, r1, eps, &mut its.vertices);
            let b = discretize_circle(&c2, &n2, r2, eps, &mut its.vertices);
            triangulate_fan(&mut its, false, 0, a.0, a.1);
            triangulate_strip(&mut its, a.0, a.1, b.0, b.1);
            its.vertices.push(c2);
            let apex = its.vertices.len() as i32 - 1;
            triangulate_fan(&mut its, true, apex, b.0, b.1);
            let mut line = format!("mesh {i} {} {} {} {} {}", a.0, a.1, b.0, b.1, its.vertices.len());
            for v in &its.vertices {
                line += &format!(" {}", f3(v));
            }
            line += &format!(" {}", its.indices.len());
            for f in &its.indices {
                line += &format!(" {} {} {}", f[0], f[1], f[2]);
            }
            c.r.out(line);
            i += 1;
        }
        // discretize_polygon()
        let mut i = 0usize;
        while let Some(l) = c.r.input("in dpoly") {
            let mut t = Tok::new(&l);
            let z = t.f();
            let polys: Vec<Polygon> = t.polys();
            let center = Vec3f::new(0., 0., z);
            let mut pts: Vec<Vec3f> = vec![center];
            let r = discretize_polygon(&center, &polys, &mut pts);
            let mut line = format!("discretize_polygon {i} {} {}", r.0, r.1);
            // The first point is the centre the generator put there, of which the function reads the height only.
            for v in &pts[1..] {
                line += &format!(" {}", f3(v));
            }
            c.r.out(line);
            i += 1;
        }
    }
}

/// gen.cpp `NUM_SCENARIOS`
const NUM_SCENARIOS: usize = 18;

#[test]
fn tree_support_3d_native_parity() {
    let run = match std::env::var("TREE_SUPPORT_3D_PARITY_VECTORS") {
        // A diagnostic: a file of the generator with every line in full.
        Ok(path) => {
            let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
            Run { expected: text.lines().map(|l| l.to_string()).collect(), pos: 0, lines: Vec::new(), surplus: 0 }
        }
        Err(_) => Run::load("tree_support_3d"),
    };
    let mut c = Ctx { r: run, full: false, draw_stopped: 0, smooth_outward_stopped: 0 };
    c.full = c.r.input("in mode").expect("mode") == "full";
    // A diagnostic: one scenario only, of a file the generator wrote for that scenario.
    let only: Option<usize> = std::env::var("TREE_SUPPORT_3D_PARITY_ONLY").ok().and_then(|v| v.parse().ok());
    match only {
        Some(scenario) => run_scenario(&mut c, scenario),
        None => {
            run_helpers(&mut c);
            for scenario in 0..NUM_SCENARIOS {
                run_scenario(&mut c, scenario);
            }
        }
    }
    // A diagnostic: the lines this test produced, written where TREE_SUPPORT_3D_PARITY_DUMP names a file.
    if let Ok(path) = std::env::var("TREE_SUPPORT_3D_PARITY_DUMP") {
        let _ = std::fs::write(path, c.r.lines.join("\n"));
    }
    let mut by_kind: std::collections::BTreeMap<String, (usize, usize)> = std::collections::BTreeMap::new();
    for (k, e) in c.r.expected.iter().enumerate() {
        if e.starts_with("in ") {
            continue;
        }
        let toks: Vec<&str> = e.split(' ').collect();
        let is_scenario = toks[0].len() > 1 && toks[0].starts_with('s') && toks[0][1..].bytes().all(|b| b.is_ascii_digit());
        let kind = if is_scenario {
            let stage = toks.get(1).copied().unwrap_or("");
            if matches!(stage, "initial" | "pathing" | "nodes" | "branch" | "smooth" | "draw") {
                format!("{stage} {}", toks.get(2).copied().unwrap_or(""))
            } else {
                stage.to_string()
            }
        } else {
            toks[0].to_string()
        };
        let entry = by_kind.entry(kind).or_insert((0, 0));
        entry.0 += 1;
        if c.r.lines.get(k) != Some(e) {
            entry.1 += 1;
        }
    }
    for (kind, (n, bad)) in &by_kind {
        eprintln!("{kind}: {n} lines, {bad} mismatches");
    }
    let bad = c.r.finish();
    eprintln!("{} lines, {} mismatches", c.r.expected.len(), bad);
    // The stage "draw" (the layers draw_areas() hands on) and the lines "smooth_outward" are reported, not
    // required. The branch areas are equal to the pin's before and after the smoothing of the branches (the
    // stages "branch" and "smooth"), and so is their union by layer ("layer_union").
    // finalize_interface_and_support_areas() then runs that union through smooth_outward() of
    // mutable_polygon.rs, which does not give the pin's polygons yet: the lines "smooth_outward" are that call
    // by itself, on the same unions. Every other line is required, and so are these from the moment they come
    // out equal.
    let (mut draw_lines, mut draw_bad, mut smooth_lines, mut smooth_bad, mut required_bad) = (0usize, 0usize, 0usize, 0usize, 0usize);
    for (kind, (n, bad)) in &by_kind {
        if kind.starts_with("draw ") {
            draw_lines += n;
            draw_bad += bad;
        } else if kind == "smooth_outward" {
            smooth_lines += n;
            smooth_bad += bad;
        } else {
            required_bad += bad;
        }
    }
    eprintln!(
        "smooth_outward of mutable_polygon.rs (reported, not required): {smooth_bad} of {smooth_lines} lines differ, {} calls stopped at a debug assertion",
        c.smooth_outward_stopped
    );
    eprintln!(
        "draw_areas (reported, not required): {draw_bad} of {draw_lines} lines differ, {} scenarios stopped at a debug assertion of smooth_outward",
        c.draw_stopped
    );
    assert_eq!(required_bad, 0, "lines of the vectors that differ, the stage \"draw\" and the lines \"smooth_outward\" aside");
    if draw_bad == 0 && c.draw_stopped == 0 && smooth_bad == 0 && c.smooth_outward_stopped == 0 {
        assert_eq!(bad, 0, "lines of the vectors that differ");
    }
}
