//! Native parity of the Support/TreeSupport.cpp port (`slicer::support::tree_support`) against vectors
//! made by the C++ at the pin (tests/data/tree_support/gen.cpp).
//!
//! The generator runs the helpers of TreeSupport.cpp on polygons, and for four scenarios builds a
//! Print of one object with `Print::apply`, fills its object layers by hand (a stem with a wide top and
//! the overhangs of the top) and runs the tree support on it: the constructor, the collision and
//! avoidance areas of `TreeSupportData`, the radii of a branch, the top gap, `create_tree_support_layers`,
//! `generate_contact_points`, `plan_layer_heights`, `drop_nodes`, `smooth_nodes`, `insert_dropped_node`
//! and `tree_support_node_disk`. The lines of the vectors that start with "in" are the inputs (the
//! configuration keys, the layers, the arguments), which this test reads back; every other line it has
//! to reproduce.
//!
//! The scenarios from 4 on reach the branches of `generate_contact_points` and `drop_nodes` the first four
//! leave out (branches that land on the object, build plate only, the strong style, big flat and thin
//! plate overhangs of the hybrid style, expanded supports, sharp tails, islands of two nodes, more than a
//! hundred layers, a raft without a top gap, enforcer points). Their long lines are in the vectors as "#"
//! and the FNV-1a hash of the text; `TREE_SUPPORT_PARITY_VECTORS` names a file the generator wrote with
//! every line in full (`gen <file> full`), for finding the first token that differs.
//!
//! The scenarios from 100004 on are a Print with two nozzle diameters, where `make_support_merge_plan` plans
//! slabs for the coarse nozzle: the plan, `fold_nodes_multi_diameter` and `smooth_line_width_multi_diameter`
//! (the nodes and the support layers after each) are compared, and the stages after them run on the folded
//! tree. Every scenario ends with `TreeSupport::generate()`, the whole chain, by a tree support of its own:
//! the nodes, the areas of the support layers and their toolpaths. The scenarios from 200000 on are random
//! ones with two nozzle diameters (the filaments of the supports and of the object, regions with perimeters
//! and fills in the layers, layers of half the height).
//!
//! A scenario with the lightning pattern in the bases of its supports makes the generator of the lightning
//! infill (Fill/Lightning/Generator.cpp:100-143, `Generator::new_for_support`): its trees and the polylines of
//! every layer are compared ("lightning_trees"). `rand()`, which picks the branch a polyline goes on with, is
//! restarted with `srand(1)` on both sides.

use slicer::config::{
    ConfigBase, ConfigOptionInts, ConfigOptionResolver, ConfigOptionStrings, ConfigSubstitutionContext, ForwardCompatibilitySubstitutionRule,
};
use slicer::extrusion_entity::{ExtrusionEntityType, ExtrusionPath};
use slicer::fill::fill_base::Fill;
use slicer::fill::FillParams;
use slicer::geometry::{BoundingBox, ExPolygon, ExPolygons, Point, Polygon, Polygons, Polyline, Polylines};
use slicer::print_config::InfillPattern;
use slicer::support::support_common::cu;
use slicer::support::support_common::{InfillFailedException, SupportFill, SupportFillFactory};
use slicer::surface::Surface;
use std::cell::RefCell;
use std::rc::Rc;
use slicer::model::*;
use slicer::normal_utils::indexed_triangle_set;
use slicer::print::faithful::{Print, PrintObject};
use slicer::print_apply::*;
use slicer::print_config::config_def::DynamicPrintConfig;
use slicer::support::support_parameters::SupportParameters;
use slicer::support::tree_support::*;
use slicer::triangle_mesh::{its_make_cube, Vec3f};
use std::collections::HashMap;
use std::io::Read;

fn hd(v: f64) -> String {
    format!("{:016x}", v.to_bits())
}
fn hf(v: f32) -> String {
    format!("{:08x}", v.to_bits())
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
    let mut o = format!("{}", p.points.len());
    for q in &p.points {
        o += &format!(" {}", pt(q));
    }
    o
}
fn polys_s(ps: &[Polygon]) -> String {
    let mut o = format!("{}", ps.len());
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
    let mut o = format!("{}", es.len());
    for e in es {
        o += &format!(" {}", expoly_s(e));
    }
    o
}

fn pts_s(pts: &[Point]) -> String {
    let mut o = format!("{}", pts.len());
    for q in pts {
        o += &format!(" {}", pt(q));
    }
    o
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
        ExtrusionEntityType::Loop(l) => {
            let mut o = format!("L {}", l.paths.len());
            for q in &l.paths {
                o += &format!(" {}", path_s(q));
            }
            o
        }
        ExtrusionEntityType::Path(q) => path_s(q),
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
            "{} {} {} {} {} {} {} {} {} {} {} {} {}",
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
            hb(params.dont_sort),
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

/// The tokens of an input line.
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
    fn b(&mut self) -> bool {
        self.s() == "1"
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
}

/// The vectors, read in order: an input line is handed out and echoed, a result line is produced.
struct Run {
    expected: Vec<String>,
    pos: usize,
    lines: Vec<String>,
    surplus: usize,
    /// The vectors carry every line in full ("in mode full").
    full: bool,
}

/// gen.cpp `fnv(s)`
fn fnv(s: &str) -> String {
    let mut h: u64 = 1469598103934665603;
    for c in s.bytes() {
        h ^= c as u64;
        h = h.wrapping_mul(1099511628211);
    }
    format!("{h:016x}")
}
impl Run {
    /// The next input line when it is the input `prefix`; it is echoed. The results produced since the
    /// input before it are aligned with the vectors: missing ones are padded, surplus ones counted.
    fn input(&mut self, prefix: &str) -> Option<String> {
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
    fn out(&mut self, s: String) {
        self.lines.push(s);
    }
    /// gen.cpp `outb(head, body, hashed)`
    fn outb(&mut self, head: String, body: String, hashed: bool) {
        if hashed && !self.full {
            self.lines.push(format!("{head} #{}", fnv(&body)));
        } else {
            self.lines.push(format!("{head} {body}"));
        }
    }
}

fn box_its(sx: f64, sy: f64, sz: f64, ox: f32, oy: f32, oz: f32) -> indexed_triangle_set {
    let mut its = its_make_cube(sx, sy, sz);
    for v in &mut its.vertices {
        *v += Vec3f::new(ox, oy, oz);
    }
    its
}

fn deser(config: &mut DynamicPrintConfig, key: &str, value: &str) {
    let mut ctxt = ConfigSubstitutionContext::new(ForwardCompatibilitySubstitutionRule::Disable);
    config.set_deserialize(key, value, &mut ctxt, false).unwrap_or_else(|e| panic!("{key}={value}: {e:?}"));
}

/// PrintApply.cpp:1683-1701 `apply_multi_diameter_object_tuning` of `Print::apply`: the gaps between the supports
/// and the object of a Print with several nozzle diameters, by the nozzle of the support filament.
fn apply_multi_diameter_object_tuning(print: &Print, config: &mut DynamicPrintConfig, model_object: &ModelObject, default_support_nozzle: f64) {
    use slicer::print_config::config_def::{multi_nozzle_diameter_mode_enabled, multi_nozzle_support_z_distance};
    if !multi_nozzle_diameter_mode_enabled(&print.base.base.m_full_print_config) {
        return;
    }
    let support_filament: u32 =
        if config.opt_int("support_interface_filament") > 0 { config.opt_int("support_interface_filament") as u32 } else { config.opt_int("support_filament") as u32 };
    let nozzle_diameter: f64 = if support_filament == 0 && default_support_nozzle > slicer::libslic3r::EPSILON {
        default_support_nozzle
    } else {
        slicer::print::faithful::nozzle_diameter_for_filament(&print.m_config, support_filament) as f64
    };
    let combo = multi_nozzle_support_combo(&print.base.base.m_full_print_config, config, model_object);
    let top = if combo.zero_top { 0. } else { multi_nozzle_support_z_distance(nozzle_diameter, *config.opt_float("support_top_z_distance")) };
    let bottom = if combo.zero_bottom { 0. } else { multi_nozzle_support_z_distance(nozzle_diameter, *config.opt_float("support_bottom_z_distance")) };
    config.set_key_value("support_top_z_distance", Box::new(slicer::config::ConfigOptionFloat::with_value(top)));
    config.set_key_value("support_bottom_z_distance", Box::new(slicer::config::ConfigOptionFloat::with_value(bottom)));
}

/// `Print::apply` on a fresh Print, as tests/print_native_parity.rs assembles it.
fn assemble(model: &Model, config: &DynamicPrintConfig) -> Print {
    let mut print = Print::new();
    let mut new_full_config = config.clone();
    // PrintApply.cpp:1305-1307
    for key in ["print_settings_id", "filament_settings_id", "printer_settings_id"] {
        let _ = new_full_config.option_mut(key, true);
    }
    // PrintApply.cpp:1323-1325: no objects on the fresh Print, no used filaments.
    new_full_config.normalize_fdm_1().unwrap();
    let mut changed_keys_ori_values = slicer::config::DynamicConfig::new();
    let _ = new_full_config.normalize_fdm_2(0, 0, Some(&mut changed_keys_ori_values)).unwrap();
    // PrintApply.cpp:1438-1456: one extruder.
    let mut print_variant_index: Vec<i32> = vec![0];
    print.base.base.m_ori_full_print_config = new_full_config.clone();
    let filament_maps: Vec<i32> = new_full_config
        .option("filament_map")
        .and_then(|o| o.as_any().downcast_ref::<ConfigOptionInts>())
        .map(|o| o.values.clone())
        .unwrap_or_else(|| vec![1]);
    // PrintApply.cpp:1495-1506
    let mut filament_overrides = DynamicPrintConfig::new();
    let print_diff = print_config_diffs(&print.m_config, &new_full_config, &mut filament_overrides, 0, &filament_maps);
    let full_config_diff = full_print_config_diffs(&print.base.base.m_full_print_config, &new_full_config, 0);
    let object_diff = print.m_default_object_config.diff(&new_full_config).unwrap();
    let region_diff = print.m_default_region_config.diff(&new_full_config).unwrap();
    // PrintApply.cpp:1644-1678
    if !full_config_diff.is_empty() {
        let _ = print.m_config.apply_only(&new_full_config, &print_diff, true);
        let keys = filament_overrides.keys();
        let _ = print.m_config.apply_only(&filament_overrides, &keys, true);
        let _ = print.m_default_object_config.apply_only(&new_full_config, &object_diff, true);
        let _ = print.m_default_region_config.apply_only(&new_full_config, &region_diff, true);
        print.base.base.m_full_print_config = new_full_config.clone();
        print.update_filament_self_index_cache();
    }
    let num_extruders = print.m_config.option("filament_diameter").and_then(|o| o.as_vector_base()).map(|v| v.size()).unwrap_or(0);
    print.m_model = model.clone();
    // PrintApply.cpp:1958-1990: the PrintObjects of every ModelObject, in the order of the model.
    let mut statuses: Vec<(usize, Vec<PrintObjectTrafoAndInstances>)> = Vec::new();
    for (mo_idx, model_object) in print.m_model.objects.iter().enumerate() {
        let print_instances = print_objects_from_model_object(model_object);
        statuses.push((mo_idx, print_instances));
    }
    for (mo_idx, print_instances) in &statuses {
        let model_object = &print.m_model.objects[*mo_idx];
        let mut config = object_config_from_model_object(&print.m_default_object_config, model_object, num_extruders, &mut print_variant_index);
        // PrintApply.cpp:1976-1978 `apply_multi_diameter_object_tuning(config, *model_object, m_single_diameter_layering_nozzle)`
        apply_multi_diameter_object_tuning(&print, &mut config, model_object, print.m_single_diameter_layering_nozzle);
        for pi in print_instances {
            // PrintApply.cpp:1983 `new PrintObject(this, model_object, trafo, std::move(instances))`
            let mut po = PrintObject::new(&mut print, *mo_idx, pi.trafo, pi.instances.clone());
            po.m_config = config.clone();
            print.m_objects.push(po);
        }
    }
    let n_objects = print.m_objects.len();
    for i in 0..n_objects {
        for inst in print.m_objects[i].m_instances.iter_mut() {
            inst.print_object = i;
        }
    }
    // PrintApply.cpp:2116-2272: the regions, shared by the PrintObjects of one ModelObject.
    let mut i = 0;
    while i < n_objects {
        let mo_idx = print.m_objects[i].m_model_object;
        let mut end = i + 1;
        while end < n_objects && print.m_objects[end].m_model_object == mo_idx {
            end += 1;
        }
        let model_object = &print.m_model.objects[mo_idx];
        let trafo = statuses.iter().find(|s| s.0 == mo_idx).unwrap().1[0].trafo;
        let xy = *print.m_objects[i].m_config.opt_float("xy_contour_compensation") as f32;
        let regions = generate_print_object_regions(
            None,
            model_object,
            Some(&print.m_model),
            &LayerRanges::new(&model_object.layer_config_ranges),
            &print.m_default_region_config,
            &trafo,
            num_extruders,
            if model_object.is_mm_painted() { 0.0 } else { xy },
            &[],
            &mut print_variant_index,
            false,
            &[],
        );
        for k in i..end {
            regions.borrow_mut().ref_cnt_inc();
            print.m_objects[k].m_shared_regions = Some(regions.clone());
        }
        i = end;
    }
    print.update_print_regions_from_objects();
    // PrintApply.cpp:2053-2090: the config checked again with the objects in place.
    let new_used_filaments = print.extruders(true).len() as i32;
    let new_changed_keys =
        new_full_config.normalize_fdm_2(print.m_objects.len() as i32, new_used_filaments, Some(&mut changed_keys_ori_values)).unwrap();
    if !new_changed_keys.is_empty() {
        print.invalidate_state_by_config_options(&new_changed_keys);
        print.invalidate_step(slicer::print::faithful::PrintStep::psGCodeExport);
        let _ = print.m_config.apply_only(&new_full_config, &new_changed_keys, true);
        let _ = print.m_default_object_config.apply_only(&new_full_config, &new_changed_keys, true);
        let _ = print.m_default_region_config.apply_only(&new_full_config, &new_changed_keys, true);
        print.base.base.m_full_print_config = new_full_config.clone();
        print.update_filament_self_index_cache();
    }
    // PrintApply.cpp:2294-2306: the single-diameter layering nozzle (0 off multi-diameter mode).
    let single_nozzle = print.resolve_single_diameter_layering_nozzle();
    if (single_nozzle - print.m_single_diameter_layering_nozzle).abs() > slicer::libslic3r::EPSILON {
        print.m_single_diameter_layering_nozzle = single_nozzle;
    }
    // PrintApply.cpp:2308-2324: the object configs retuned with the nozzle just resolved.
    for i in 0..print.m_objects.len() {
        let mut tuned_config = print.m_objects[i].m_config.clone();
        let model_object = &print.m_model.objects[print.m_objects[i].m_model_object];
        apply_multi_diameter_object_tuning(&print, &mut tuned_config, model_object, print.m_single_diameter_layering_nozzle);
        let diff = print.m_objects[i].m_config.diff(&tuned_config).unwrap();
        if !diff.is_empty() {
            let _ = print.m_objects[i].m_config.apply_only(&tuned_config, &diff, true);
        }
    }
    // PrintApply.cpp:2326-2331
    for i in 0..print.m_objects.len() {
        PrintObject::update_slicing_parameters(&mut print, i);
        print.m_support_used |= print.m_objects[i].m_config.opt_bool("enable_support");
    }
    print
}


fn node_ref(nodes: &[SupportNode], n: Option<NodeId>) -> String {
    match n {
        None => "-".into(),
        Some(n) => format!("{} {}", pt(&nodes[n].position), hd(nodes[n].print_z)),
    }
}

/// gen.cpp `NODE(n)`
fn node_s(nodes: &[SupportNode], id: NodeId) -> String {
    let n = &nodes[id];
    // TreeSupport.hpp:39-43 `enum TreeNodeType { eCircle, eSquare, ePolygon }`
    let type_ = match n.type_ {
        TreeNodeType::ECircle => 0,
        TreeNodeType::ESquare => 1,
        TreeNodeType::EPolygon => 2,
    };
    let mut o = format!(
        "{} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} p {} c {} {} {}",
        pt(&n.position),
        hd(n.print_z),
        hd(n.height),
        hd(n.radius),
        hd(n.dist_mm_to_top),
        n.distance_to_top,
        n.support_roof_layers_below,
        n.obj_layer_nr,
        hb(n.to_buildplate),
        hb(n.valid),
        type_,
        hb(n.is_corner),
        hb(n.is_sharp_tail),
        hb(n.fading),
        hb(n.need_extra_wall),
        hb(n.is_processed),
        hf(n.carried_mass),
        hd(n.max_move_dist),
        pt(&n.movement),
        pt(&n.orig_pos),
        hd(n.target_radius),
        hd(n.overhang_degree),
        pt(&n.skin_direction),
        node_ref(nodes, n.parent),
        node_ref(nodes, n.child),
        n.parents.len(),
        n.merged_neighbours.len(),
    );
    if n.type_ == TreeNodeType::EPolygon {
        o += &format!(" o {} {}", expoly_s(&n.overhang), hd(n.origin_area));
    }
    o
}

/// gen.cpp `NODE2(n)`
fn node2_s(nodes: &[SupportNode], id: NodeId) -> String {
    let n = &nodes[id];
    format!("{} s {} {} {} {}", node_s(nodes, id), hd(n.slab_height), n.slab_bottom_obj_layer, n.slab_index, hd(n.line_width))
}

/// gen.cpp `dump_nodes(tag, contact_nodes, hashed)`
fn dump_nodes(r: &mut Run, tag: &str, nodes: &[SupportNode], contact_nodes: &[Vec<NodeId>], hashed: bool) {
    dump_nodes_of(r, tag, nodes, contact_nodes, hashed, false)
}

/// gen.cpp `dump_nodes(tag, contact_nodes, hashed, folded)`
fn dump_nodes_of(r: &mut Run, tag: &str, nodes: &[SupportNode], contact_nodes: &[Vec<NodeId>], hashed: bool, folded: bool) {
    for (layer_nr, layer) in contact_nodes.iter().enumerate() {
        if layer.is_empty() {
            continue;
        }
        let mut lines: Vec<String> = layer.iter().map(|&n| if folded { node2_s(nodes, n) } else { node_s(nodes, n) }).collect();
        lines.sort();
        if hashed && !r.full {
            let mut all = String::new();
            for l in &lines {
                all += l;
                all.push('\n');
            }
            r.out(format!("{tag} layer {} {} #{}", layer_nr, lines.len(), fnv(&all)));
            continue;
        }
        r.out(format!("{tag} layer {} {}", layer_nr, lines.len()));
        for l in lines {
            r.out(format!("{tag} node {l}"));
        }
    }
}

/// gen.cpp `BBOX(b)`
fn bbox_s(b: &BoundingBox) -> String {
    format!("{} {} {}", hb(b.is_defined()), pt(&b.min), pt(&b.max))
}

/// gen.cpp `AREA_GROUPS(l)`
fn area_groups_s(l: &slicer::print::faithful::SupportLayer) -> String {
    let mut o = format!("{}", l.area_groups.len());
    for g in &l.area_groups {
        o += &format!(
            " {} {} {} {} {} {} {} {}",
            g.type_,
            g.area,
            g.interface_id,
            hd(g.dist_to_top),
            hb(g.need_infill),
            hb(g.need_extra_wall),
            hb(g.need_cooling),
            hd(g.line_width)
        );
    }
    o
}

/// gen.cpp `dump_support_layers(tag, obj)`
// The trees of the lightning infill of the supports, a line per layer: every root depth first, a node as its
// location, whether it is a root and the number of its children.
fn dump_lightning_node(n: &slicer::fill::lightning::tree_node::NodeSPtr, body: &mut String) {
    let n = n.borrow();
    *body += &format!(" {} {} {} {}", n.m_p.x, n.m_p.y, hb(n.m_is_root), n.m_children.len());
    for c in &n.m_children {
        dump_lightning_node(c, body);
    }
}
fn dump_lightning(r: &mut Run, head: &str, generator: &slicer::fill::lightning::generator::Generator) {
    r.out(format!(
        "{head} {} {} {} {} {}",
        generator.m_supporting_radius,
        generator.m_wall_supporting_radius,
        generator.m_prune_length,
        generator.m_straightening_max_distance,
        generator.m_lightning_layers.len()
    ));
    for (i, layer) in generator.m_lightning_layers.iter().enumerate() {
        let mut body = format!("{}", layer.tree_roots.len());
        for root in &layer.tree_roots {
            dump_lightning_node(root, &mut body);
        }
        r.outb(format!("{head} {i}"), body, true);
    }
    // Layer::convertToLines() of every layer, clipped by a rectangle inside of the extents of its trees: the
    // polylines before generate_toolpaths() chains them. rand() restarts for it.
    slicer::fill::lightning::tree_node::c_srand(1);
    for (i, layer) in generator.m_lightning_layers.iter().enumerate() {
        if !layer.tree_roots.is_empty() {
            let bb = slicer::fill::lightning::tree_node::get_extents_roots(&layer.tree_roots);
            let (dx, dy) = ((bb.max.x - bb.min.x) / 8, (bb.max.y - bb.min.y) / 8);
            let limit = Polygon::from_points(vec![
                Point::new(bb.min.x + dx, bb.min.y + dy),
                Point::new(bb.max.x - dx, bb.min.y + dy),
                Point::new(bb.max.x - dx, bb.max.y - dy),
                Point::new(bb.min.x + dx, bb.max.y - dy),
            ]);
            let lines = layer.convert_to_lines(std::slice::from_ref(&limit), 0);
            let mut body = format!("{}", lines.len());
            for pl in &lines {
                body += &format!(" {}", pts_s(&pl.points));
            }
            r.outb(format!("{head} lines {i}"), body, true);
            // and chained, as generate_toolpaths() chains them (ShortestPath.cpp chain_polylines())
            let chained = slicer::shortest_path::chain_polylines(lines.clone(), None);
            let mut body = format!("{}", chained.len());
            for pl in &chained {
                body += &format!(" {}", pts_s(&pl.points));
            }
            r.outb(format!("{head} chained {i}"), body, true);
        }
    }
}

fn dump_support_layers(r: &mut Run, tag: &str, obj: &PrintObject) {
    let layers = obj.support_layers();
    r.out(format!("{tag} {}", layers.len()));
    for (i, sl) in layers.iter().enumerate() {
        let mut body = format!(
            "base {} roof {} roof1 {} floor {} gap {} islands {} bboxes {}",
            expolys_s(&sl.base_areas),
            expolys_s(&sl.roof_areas),
            expolys_s(&sl.roof_1st_layer),
            expolys_s(&sl.floor_areas),
            expolys_s(&sl.roof_gap_areas),
            expolys_s(&sl.support_islands),
            sl.layer.lslices_bboxes.len()
        );
        for b in &sl.layer.lslices_bboxes {
            body += &format!(" {}", bbox_s(b));
        }
        body += &format!(" groups {}", area_groups_s(sl));
        let id_of = |idx: Option<usize>| idx.map(|i| layers[i].layer.id() as i64).unwrap_or(-1);
        r.outb(
            format!(
                "{tag} layer {} {} {} {} {} {} {} {} {} {}",
                i,
                sl.layer.id(),
                sl.interface_id(),
                hd(sl.layer.print_z),
                hd(sl.layer.height),
                hd(sl.layer.slice_z),
                id_of(sl.layer.lower_layer_id),
                id_of(sl.layer.upper_layer_id),
                hd(sl.base_slab_height),
                hd(sl.interface_slab_height)
            ),
            body,
            true,
        );
    }
}

/// gen.cpp `PLAN(plan)`
fn plan_s(plan: &slicer::support_merge_plan::SupportMergePlan) -> String {
    let mut o = format!("{}", plan.slabs.len());
    for s in &plan.slabs {
        o += &format!(" {} {} {} {} {} {} {}", s.bottom_idx, s.top_idx, hd(s.top_z), hd(s.height), s.filament_id, hd(s.nozzle_diameter), hb(s.bottom_anchor));
    }
    o += &format!(" SI {}", plan.slab_index_per_layer.len());
    for i in &plan.slab_index_per_layer {
        o += &format!(" {i}");
    }
    for list in [&plan.candidates_per_layer, &plan.interface_candidates_per_layer] {
        o += &format!(" CL {}", list.len());
        for layer in list {
            o += &format!(" {}", layer.len());
            for c in layer {
                o += &format!(" {} {} {} {} {}", c.filament_id, hd(c.nozzle_diameter), hd(c.window.min_height), hd(c.window.max_height), c.band_size);
            }
        }
    }
    o += &format!(" FB {}", plan.fixed_bands_per_filament.len());
    for (filament_id, bands) in &plan.fixed_bands_per_filament {
        o += &format!(" {} {}", filament_id, bands.len());
        for b in bands {
            o += &format!(" {} {} {}", b.bottom_idx, b.top_idx, hd(b.merged_height));
        }
    }
    o += &format!(" PZ {}", plan.layer_print_z.len());
    for v in &plan.layer_print_z {
        o += &format!(" {}", hd(*v));
    }
    o += &format!(" LH {}", plan.layer_heights.len());
    for v in &plan.layer_heights {
        o += &format!(" {}", hd(*v));
    }
    o
}

/// gen.cpp `dump_fold(tag, ts, obj)`
fn dump_fold(r: &mut Run, tag: &str, ts: &TreeSupport) {
    let mut o = format!("{}", ts.m_fold_carriers.len());
    for carriers in &ts.m_fold_carriers {
        o += &format!(" {}", carriers.len());
        for c in carriers {
            o += &format!(" {} {}", c.layer, hd(c.bottom_z));
        }
    }
    o += &format!(" SL {}", ts.m_fold_slab_of_layer.len());
    for i in &ts.m_fold_slab_of_layer {
        o += &format!(" {i}");
    }
    o += &format!(" BO {}", ts.m_base_only_fold_layers.len());
    for i in &ts.m_base_only_fold_layers {
        o += &format!(" {i}");
    }
    r.out(format!("{tag} state {o}"));
    let layers = ts.m_object.support_layers();
    let mut o = format!("{}", layers.len());
    for sl in layers {
        o += &format!(
            " {} {} {} {} {} {}",
            hd(sl.base_slab_height),
            hd(sl.interface_slab_height),
            sl.planned_filament_id,
            hd(sl.planned_nozzle_diameter),
            sl.planned_interface_filament_id,
            hd(sl.planned_interface_nozzle_diameter)
        );
    }
    r.out(format!("{tag} layers {o}"));
}

/// `generate_tree_support_3D()` of the generate() pass: the generator leaves the organic style out.
struct NoOrganic;
impl TreeSupport3DGenerator for NoOrganic {
    fn generate_tree_support_3d(&self, _tree_support: &mut TreeSupport<'_>) -> slicer::Result<()> {
        panic!("the organic style is not run by the generator");
    }
}

/// gen.cpp `dump_detected(tag, ts, obj)`
fn dump_detected(r: &mut Run, tag: &str, ts: &TreeSupport) {
    let obj: &PrintObject = ts.m_object;
    r.out(format!(
        "{tag} {} {} {} {} {} {} {}",
        hb(ts.has_sharp_tails),
        hb(ts.has_cantilever),
        hd(ts.max_cantilever_dist),
        ts.m_highest_overhang_layer,
        ts.m_vertical_enforcer_points.len(),
        obj.support_layers().len(),
        obj.layers().len()
    ));
    for (i, l) in obj.layers().iter().enumerate() {
        let mut body = format!("ext {} ov {}", expolys_s(&l.lslices_extrudable), l.loverhangs_with_type.len());
        for ov in &l.loverhangs_with_type {
            body += &format!(" {} {}", ov.1, expoly_s(&ov.0));
        }
        body += &format!(" lo {} st {} h {}", expolys_s(&l.loverhangs), expolys_s(&ts.layer_sharp_tails[i]), ts.layer_sharp_tails_height[i].len());
        for h in &ts.layer_sharp_tails_height[i] {
            body += &format!(" {}", hf(*h));
        }
        body += &format!(" ca {}", expolys_s(&ts.layer_cantilevers[i]));
        r.outb(format!("{tag} layer {i}"), body, true);
    }
}

/// gen.cpp `run_helpers()`
fn run_helpers(r: &mut Run) {
    let mut i = 0;
    while let Some(line) = r.input("in poly") {
        let p = Tok::new(&line).poly();
        r.out(format!("centroid {} {}", i, pt(&polygon_centroid(&p))));
        for max_move in [0.05, 0.3, 2.0] {
            let mut movements: HashMap<Point, Point> = HashMap::new();
            smooth_filter(&p, 2, &mut movements, max_move);
            let mut line = format!("smooth_filter {} {}", i, hd(max_move));
            for q in &p.points {
                line += &format!(" {}", pt(&movements[q]));
            }
            r.out(line);
        }
        for max_edge in [2.0, 0.7, 11.3] {
            let mut q = p.clone();
            densify_polygon(&mut q, max_edge);
            r.out(format!("densify_polygon {} {} {}", i, hd(max_edge), poly_s(&q)));
        }
        i += 1;
    }
    let mut i = 0;
    while let Some(line) = r.input("in expoly") {
        let e = Tok::new(&line).expoly();
        let edge_cache = EdgeCache::new(&e);
        let mut line = format!("edge_cache {} {} {}", i, edge_cache.hole_count(), hd(edge_cache.circumference()));
        for h in 0..edge_cache.hole_count() {
            line += &format!(" {}", hd(edge_cache.circumference_hole(h)));
        }
        for h in 0..edge_cache.hole_count() + 1 {
            let step = (3.0 / slicer::libslic3r::SCALING_FACTOR) / (if h == 0 { edge_cache.circumference() } else { edge_cache.circumference_hole(h - 1) });
            let mut distance = 0.0;
            while distance < 1.0 {
                line += &format!(" {}", pt(&if h == 0 { edge_cache.coords(distance) } else { edge_cache.coords_hole(h - 1, distance) }));
                distance += step;
            }
        }
        r.out(line);
        i += 1;
    }
    let mut regions: Vec<ExPolygon> = Vec::new();
    let mut c = 0;
    while let Some(line) = r.input("in carrier") {
        let carrier = Tok::new(&line).expolys();
        let bboxes: Vec<BoundingBox> = carrier.iter().map(|e| e.bounding_box()).collect();
        let mut k = 0;
        loop {
            if c == 0 {
                match r.input("in region") {
                    Some(line) => regions.push(Tok::new(&line).expoly()),
                    None => break,
                }
            } else if k >= regions.len() {
                break;
            }
            r.out(format!(
                "carried_area {} {} {} {}",
                c,
                k,
                hd(carried_area(&carrier, &bboxes, &regions[k])),
                hd(carried_area_ratio(&carrier, &bboxes, &regions[k]))
            ));
            k += 1;
        }
        c += 1;
    }
}

/// gen.cpp `run_scenario(scenario)`
fn run_scenario(r: &mut Run, scenario: usize) {
    let s = format!("s{scenario}");
    // gen.cpp `FIRST_HASHED`: the long lines of this scenario are hashed.
    let h = scenario >= 4;
    let line = r.input(&format!("in {s} model")).expect("model");
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
    while let Some(line) = r.input(&format!("in {s} cfg")) {
        let (key, value) = line.split_once(' ').expect("key value");
        deser(&mut config, key, value);
    }
    let mut print = assemble(&model, &config);
    let mut obj = print.m_objects.remove(0);

    // gen.cpp `regions_mode(scenario)`: the layers carry a region of the object with their slices, perimeters
    // and fills (`MultiRandom::regions`).
    let regions: usize = r.input(&format!("in {s} regions")).map(|line| line.trim().parse().expect("regions")).unwrap_or(0);
    let mut layer_id = 0usize;
    while let Some(line) = r.input(&format!("in {s} layer")) {
        let mut t = Tok::new(&line);
        let (print_z, height, slice_z) = (t.d(), t.d(), t.d());
        let mut layer = slicer::layer::Layer::new(layer_id, obj.m_model_object, height, print_z, slice_z);
        layer.lslices = t.expolys();
        layer.lslices_extrudable = layer.lslices.clone();
        if regions > 0 {
            use slicer::extrusion_entity::{ExtrusionEntityCollection, ExtrusionRole};
            let i = layer_id;
            layer.lslices_bboxes = layer.lslices.iter().map(|e| e.contour.bounding_box()).collect();
            let mut lr = slicer::layer::LayerRegion::new(layer_id, 0);
            for e in &layer.lslices {
                lr.slices.surfaces.push(Surface::new(slicer::surface::SurfaceType::Internal, e.clone()));
            }
            let path = |role: ExtrusionRole| {
                let mut p = ExtrusionPath::with_params(role, 0.04, 0.42f32 as f64, height as f32 as f64, false);
                p.polyline.points = vec![Point::new(0, 0), Point::new(100000, 0)];
                p
            };
            if regions == 2 || regions == 4 || (regions == 1 && i % 2 == 0) || (regions == 3 && (i / 3) % 2 == 0) {
                lr.perimeters.entities.push(ExtrusionEntityType::Path(path(ExtrusionRole::ExternalPerimeter)));
            }
            if regions == 4 && i % 3 != 2 {
                let mut fill = ExtrusionEntityCollection::new();
                fill.entities.push(ExtrusionEntityType::Path(path(if i % 3 == 0 { ExtrusionRole::InternalInfill } else { ExtrusionRole::SolidInfill })));
                lr.fills.entities.push(ExtrusionEntityType::Collection(Box::new(fill)));
            }
            layer.add_region(lr);
        }
        let n_overhangs = t.i();
        for _ in 0..n_overhangs {
            let type_ = t.i() as i32;
            let overhang = t.expoly();
            add_overhang(&mut layer, &overhang, type_);
        }
        if layer_id > 0 {
            layer.lower_layer_id = Some(layer_id - 1);
            obj.m_layers[layer_id - 1].upper_layer_id = Some(layer_id);
        }
        obj.m_layers.push(layer);
        layer_id += 1;
    }
    let n_layers = obj.m_layers.len();

    let line = r.input(&format!("in {s} slicing")).expect("slicing");
    let mut t = Tok::new(&line);
    let mut sp = obj.m_slicing_params.clone();
    sp.layer_height = t.d();
    sp.min_layer_height = t.d();
    sp.max_layer_height = t.d();
    sp.max_suport_layer_height = t.d();
    sp.base_raft_layers = t.i() as usize;
    sp.interface_raft_layers = t.i() as usize;
    sp.base_raft_layer_height = t.d();
    sp.interface_raft_layer_height = t.d();
    sp.first_print_layer_height = t.d();
    sp.object_print_z_min = t.d();
    sp.soluble_interface = t.b();
    sp.gap_object_support = t.d();
    let line = r.input(&format!("in {s} bed")).expect("bed");
    let bed = Tok::new(&line).poly();
    let line = r.input(&format!("in {s} params")).expect("params");
    let mut t = Tok::new(&line);
    // `SupportParameters m_support_params(object)`; the members the generator wrote down are the pin's.
    obj.m_slicing_params = sp.clone();
    let params = SupportParameters::new(&obj, &print).expect("SupportParameters");
    assert_eq!(
        (
            params.support_style,
            params.support_base_pattern,
            params.num_top_interface_layers,
            params.support_extrusion_width.to_bits(),
            params.independent_layer_height,
            params.soluble_interface,
            params.thresh_big_overhang.to_bits()
        ),
        (t.i() as i32, t.i() as i32, t.i() as usize, t.d().to_bits(), t.b(), t.b(), t.d().to_bits()),
        "{s} params"
    );

    let mut ts = TreeSupport::new(&mut obj, &print, &sp, params, &bed);
    ts.lightning_support_generator = Some(Box::new(FillLightningSupportGenerator));

    // The constructor.
    r.out(format!(
        "{s} ctor {} {} {} {} {} {} {} {} {} {} {} {}",
        ts.m_raft_layers,
        ts.support_type,
        hd(ts.diameter_angle_scale_factor),
        hb(ts.is_slim),
        hb(ts.is_strong),
        hd(ts.tip_radius),
        hd(ts.base_radius),
        hb(ts.with_infill),
        hf(ts.top_z_distance),
        hf(ts.m_top_z_max),
        ts.m_top_z_outlines.len(),
        expoly_s(&ts.m_machine_border)
    ));

    // TreeSupportData
    {
        let d = &ts.m_ts_data;
        r.out(format!("{s} data {} {} {} {}", hd(d.m_xy_distance), hd(d.m_radius_sample_resolution), hd(d.branch_scale_factor), d.m_layer_outlines.len()));
        for l in [0, 9, 10, 20, n_layers - 12, n_layers - 1] {
            r.outb(
                format!("{s} outline {} {}", l, hd(d.m_max_move_distances[l])),
                format!(
                    "{} {} {} {}",
                    expolys_s(&d.m_layer_outlines[l]),
                    expolys_s(&d.m_layer_outlines_below[l]),
                    polys_s(&d.get_contours(l)),
                    polys_s(&d.get_contours_with_holes(l))
                ),
                h,
            );
        }
        for radius in [0., 0.1, 0.2, 0.25, 0.4, 0.45000000000000001, 1.3, 2.0000001, 5.55] {
            r.out(format!("{s} ceil_radius {} {}", hd(radius), hd(d.ceil_radius(radius))));
        }
        for radius in [0., 0.4, 1.3] {
            for l in [0, 3, 11, n_layers - 13, n_layers - 12, n_layers - 2] {
                r.outb(format!("{s} collision {} {}", hd(radius), l), expolys_s(&d.get_collision(radius, l)), h);
            }
        }
        for radius in [0.4, 2.] {
            for l in [0, 5, n_layers - 13, n_layers - 12] {
                r.outb(
                    format!("{s} avoidance {} {}", hd(radius), l),
                    format!("{} {}", expolys_s(&d.get_avoidance(radius, l, 0)), expolys_s(&ts.get_avoidance(radius, l))),
                    h,
                );
            }
        }
        r.outb(
            format!("{s} collision_polys"),
            format!("{} {}", polys_s(&ts.get_collision_polys(0.4, 3)), expolys_s(&ts.get_collision(0.4, 3))),
            h,
        );
    }

    // The radii of a branch.
    for mm in [0., 0.1, 0.39, 0.8, 1.5, 2.5, 7.3, 30., 200.] {
        r.out(format!(
            "{s} calc_radius {} {} {} {}",
            hd(mm),
            hd(ts.calc_radius(mm)),
            hd(ts.calc_branch_radius(1.7, mm, 0.11, true)),
            hd(ts.calc_branch_radius(ts.base_radius, mm, ts.diameter_angle_scale_factor, false))
        ));
    }
    for n in [0usize, 1, 4, 9, 10, 21, 80, 400] {
        r.out(format!(
            "{s} calc_branch_radius_layers {} {} {}",
            n,
            hd(ts.calc_branch_radius_layers(ts.base_radius, n, 10, ts.diameter_angle_scale_factor)),
            hd(ts.calc_branch_radius_layers(1.3, n, 4, 0.2))
        ));
    }

    // The top gap.
    let rect = |x0: f64, y0: f64, x1: f64, y1: f64| -> Polygon {
        let sc = |v: f64| (v / slicer::libslic3r::SCALING_FACTOR).round_ties_even() as i64;
        Polygon::from_points(vec![Point::new(sc(x0), sc(y0)), Point::new(sc(x1), sc(y0)), Point::new(sc(x1), sc(y1)), Point::new(sc(x0), sc(y1))])
    };
    for l in [0, 7, n_layers - 12] {
        r.out(format!("{s} top_z {} {} {}", l, hf(ts.top_z_at(l)), hf(ts.top_z_for_overhang(l, &ExPolygon::new(rect(-20., -5., -5.3, 5.))))));
    }
    for l in [n_layers - 14, n_layers - 13, 3] {
        for clamp in [false, true] {
            let mut collision: ExPolygons = (*ts.get_collision(0., l)).clone();
            ts.union_outlines_within_top_z(&mut collision, l, clamp);
            r.outb(format!("{s} union_outlines_within_top_z {} {}", l, hb(clamp)), expolys_s(&collision), h);
        }
    }
    r.out(format!("{s} support_flare_step_width {} {}", hd(ts.support_flare_step_width(3.2, 0.2)), hd(ts.support_flare_step_width(0.2, 0.2))));

    // The pipeline of TreeSupport::generate() after the overhang detection.
    ts.create_tree_support_layers();
    {
        let mut line = format!("{s} raft_layers {} {}", ts.m_raft_layers, ts.m_object.support_layers().len());
        for sl in ts.m_object.support_layers() {
            line += &format!(" {} {} {} {}", sl.layer.id(), hd(sl.layer.print_z), hd(sl.layer.height), hd(sl.layer.slice_z));
        }
        r.out(line);
    }
    ts.m_ts_data.is_slim = ts.is_slim;
    // The enforcer points detect_overhangs() collects.
    if h {
        let line = r.input(&format!("in {s} enforcer_points")).expect("enforcer_points");
        let mut t = Tok::new(&line);
        let n = t.i();
        for _ in 0..n {
            let pt = [t.f(), t.f(), t.f()];
            let normal = [t.f(), t.f(), t.f()];
            ts.m_vertical_enforcer_points.push((pt, normal));
        }
    }
    ts.generate_contact_points();
    r.out(format!(
        "{s} contact_points {} {} {}{}",
        ts.contact_nodes.len(),
        ts.avg_node_per_layer,
        ts.m_ts_data.contact_nodes.len(),
        if h { format!(" {}", hf(ts.nodes_angle)) } else { String::new() }
    ));
    dump_nodes(r, &format!("{s} contact"), &ts.m_ts_data.contact_nodes, &ts.contact_nodes, h);

    ts.m_ts_data.layer_heights = ts.plan_layer_heights();
    {
        let mut line = format!("{s} layer_heights {}", ts.m_ts_data.layer_heights.len());
        for lh in &ts.m_ts_data.layer_heights {
            line += &format!(" {} {} {}", hd(lh.print_z), hd(lh.height), lh.obj_layer_nr);
        }
        r.out(line);
        let layers = ts.m_object.support_layers();
        let mut line = format!("{s} support_layers {}", layers.len());
        for sl in layers {
            let id_of = |idx: Option<usize>| idx.map(|i| layers[i].layer.id() as i64).unwrap_or(-1);
            line += &format!(
                " {} {} {} {} {} {}",
                sl.layer.id(),
                hd(sl.layer.print_z),
                hd(sl.layer.height),
                hd(sl.layer.slice_z),
                id_of(sl.layer.lower_layer_id),
                id_of(sl.layer.upper_layer_id)
            );
        }
        r.out(line);
    }
    dump_nodes(r, &format!("{s} planned"), &ts.m_ts_data.contact_nodes, &ts.contact_nodes, h);

    // The plan of the slabs, as TreeSupport::generate() makes it ahead of drop_nodes().
    ts.m_support_merge_plan = ts.make_support_merge_plan_of_object();
    r.outb(
        format!(
            "{s} plan {} {} {}",
            hb(ts.m_print.uses_multiple_nozzle_diameters()),
            hb(ts.m_support_merge_plan.enabled),
            hb(ts.m_support_merge_plan.interface_fold_enabled)
        ),
        plan_s(&ts.m_support_merge_plan),
        h,
    );
    let folds = ts.m_support_merge_plan.enabled;

    ts.drop_nodes();
    r.out(format!("{s} dropped {} {}", ts.m_ts_data.contact_nodes.len(), hd(SupportNode::diameter_angle_scale_factor())));
    dump_nodes(r, &format!("{s} drop"), &ts.m_ts_data.contact_nodes, &ts.contact_nodes, h);

    ts.smooth_nodes();
    dump_nodes(r, &format!("{s} smooth"), &ts.m_ts_data.contact_nodes, &ts.contact_nodes, h);

    // insert_dropped_node: a node already in the layer, and one that is not.
    for layer_nr in 0..ts.contact_nodes.len() - 1 {
        if !ts.contact_nodes[layer_nr].is_empty() && !ts.contact_nodes[layer_nr + 1].is_empty() {
            let mut nodes_layer = ts.contact_nodes[layer_nr].clone();
            let front = nodes_layer[0];
            ts.insert_dropped_node(&mut nodes_layer, front);
            let before = nodes_layer.len();
            let other = ts.contact_nodes[layer_nr + 1][0];
            ts.insert_dropped_node(&mut nodes_layer, other);
            r.out(format!(
                "{s} insert_dropped_node {} {} {} {}",
                layer_nr,
                before,
                nodes_layer.len(),
                node_ref(&ts.m_ts_data.contact_nodes, nodes_layer.last().copied())
            ));
            break;
        }
    }

    // tree_support_node_disk over the nodes of a few layers.
    {
        let line = r.input(&format!("in {s} disk")).expect("disk");
        let mut t = Tok::new(&line);
        let (branch_radius, branch_radius_scaled) = (t.d(), t.d());
        let unit_circle = t.poly();
        let mut done = 0;
        let mut layer_nr = 1;
        while layer_nr < ts.contact_nodes.len() && done < 4 {
            if !ts.contact_nodes[layer_nr].is_empty() {
                let nodes = &ts.m_ts_data.contact_nodes;
                let mut lines: Vec<String> = ts.contact_nodes[layer_nr]
                    .iter()
                    .map(|&n| {
                        format!(
                            "{} d {} {}",
                            node_s(nodes, n),
                            expoly_s(&tree_support_node_disk(&nodes[n], &unit_circle, branch_radius, branch_radius_scaled, false, 0.0)),
                            expoly_s(&tree_support_node_disk(&nodes[n], &unit_circle, branch_radius, branch_radius_scaled, true, 0.9))
                        )
                    })
                    .collect();
                lines.sort();
                if h && !r.full {
                    let mut all = String::new();
                    for l in &lines {
                        all += l;
                        all.push('\n');
                    }
                    r.out(format!("{s} disk {layer_nr} {} #{}", lines.len(), fnv(&all)));
                } else {
                    for l in lines {
                        r.out(format!("{s} disk {layer_nr} {l}"));
                    }
                }
                done += 1;
            }
            layer_nr += 7;
        }
    }

    // The support areas: the steps of TreeSupport::generate() after smooth_nodes().
    ts.fold_nodes_multi_diameter();
    if folds {
        dump_fold(r, &format!("{s} fold"), &ts);
        dump_nodes_of(r, &format!("{s} folded"), &ts.m_ts_data.contact_nodes, &ts.contact_nodes, h, true);
    }
    ts.smooth_line_width_multi_diameter().expect("smooth_line_width_multi_diameter");
    if folds {
        dump_nodes_of(r, &format!("{s} widened"), &ts.m_ts_data.contact_nodes, &ts.contact_nodes, h, true);
    }
    ts.draw_circles();
    dump_support_layers(r, &format!("{s} draw_circles"), ts.m_object);
    {
        let mut zs: Vec<(f64, usize)> = ts.print_z_to_lightninglayer.iter().map(|(k, v)| (f64::from_bits(*k), *v)).collect();
        zs.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mut line = format!("{s} lightning {}", zs.len());
        for z in &zs {
            line += &format!(" {} {}", hd(z.0), z.1);
        }
        r.out(line);
        // The generator of the lightning infill draw_circles() made (Fill/Lightning/Generator.cpp:100-143, the
        // constructor for the supports): its width, the radius of the density, the overhangs, then its trees.
        if let Some(line) = r.input(&format!("in {s} lightning_width")) {
            let generator = ts.generator.as_ref().expect("the lightning generator");
            let m_infill_extrusion_width = Tok::new(&line).f();
            assert_eq!(generator.m_infill_extrusion_width.to_bits(), m_infill_extrusion_width.to_bits(), "{s} m_infill_extrusion_width");
            let mut body = format!("{} {}", generator.m_supporting_radius, generator.m_overhang_per_layer.len());
            for polys in &generator.m_overhang_per_layer {
                body += &format!(" {}", polys_s(polys));
            }
            r.outb(format!("{s} lightning_generator"), body, true);
            dump_lightning(r, &format!("{s} lightning_trees"), generator);
        }
    }

    // The toolpaths, with the generator's fillers. A scenario with the lightning pattern in the bases writes
    // "toolpaths_lightning" lines, which are counted and not compared: the trees of the lightning generator and
    // the polylines of Layer::convertToLines() are equal ("lightning_trees"), but generate_toolpaths() chains
    // the polylines with ShortestPath.cpp's chain_polylines(), and the port of that orders polylines that share
    // an end point differently (the "lightning_trees chained" lines, not compared either).
    {
        let tag = if ts.m_support_params.base_fill_pattern == InfillPattern::Lightning { format!("{s} toolpaths_lightning") } else { format!("{s} toolpaths") };
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let factory = RecFactory { log: log.clone() };
        // Node::convertToPolylines() takes the branch a polyline goes on with from rand(): the stream starts here
        slicer::fill::lightning::tree_node::c_srand(1);
        ts.generate_toolpaths(&factory).expect("generate_toolpaths");
        let mut fills: Vec<String> = log.borrow_mut().drain(..).collect();
        fills.sort();
        let mut body = String::new();
        for l in &fills {
            body += l;
            body.push('\n');
        }
        if r.full {
            for l in &fills {
                r.out(format!("{tag} fill {l}"));
            }
        }
        r.out(format!("{tag} fills {} #{}", fills.len(), fnv(&body)));
        let layers = ts.m_object.support_layers();
        for (i, sl) in layers.iter().enumerate() {
            r.outb(format!("{tag} layer {} {} {}", i, hb(sl.support_fills.no_sort), sl.support_fills.entities.len()), ents_s(&sl.support_fills.entities), true);
        }
    }
    // prune_floating_supports(), which the pin does not call: the areas of the support layers after it.
    ts.prune_floating_supports();
    dump_support_layers(r, &format!("{s} prune"), ts.m_object);
    drop(ts);

    // detect_overhangs() on the layers of the scenario, by a second tree support of the object.
    {
        let params = SupportParameters::new(&obj, &print).expect("SupportParameters");
        let mut ts2 = TreeSupport::new(&mut obj, &print, &sp, params, &bed);
        ts2.detect_overhangs(false).expect("detect_overhangs");
        dump_detected(r, &format!("{s} detect"), &ts2);
        // get_trim_support_regions(), which the pin does not call, for support layers at a few heights.
        let layers = ts2.m_object.layers();
        let zs = [0.6, 3.3, layers[layers.len() / 2].print_z, layers[layers.len() - 1].print_z - 0.9];
        for (k, z) in zs.iter().enumerate() {
            let h = if k == 1 { 0.3 } else { 0.2 };
            let sl = slicer::print::faithful::SupportLayer::new(k, 0, ts2.m_object.m_model_object, h, *z, *z - h / 2.);
            r.outb(
                format!("{s} trim_regions {} {}", hd(*z), hd(h)),
                format!(
                    "{} {}",
                    polys_s(&ts2.get_trim_support_regions(ts2.m_object, &sl, 0.2, 0.1, 0.35)),
                    polys_s(&ts2.get_trim_support_regions(ts2.m_object, &sl, 0., 0.45, 0.8))
                ),
                true,
            );
        }
    }

    // TreeSupport::generate(), the whole chain, by a third tree support of the object.
    {
        let params = SupportParameters::new(&obj, &print).expect("SupportParameters");
        let mut ts3 = TreeSupport::new(&mut obj, &print, &sp, params, &bed);
        ts3.lightning_support_generator = Some(Box::new(FillLightningSupportGenerator));
        // TreeSupport.hpp `smsTreeOrganic`
        if ts3.m_support_params.support_style == slicer::print_config::enums::SupportMaterialStyle::SmsTreeOrganic as i32 {
            r.out(format!("{s} generate organic"));
            return;
        }
        let log: Log = Rc::new(RefCell::new(Vec::new()));
        let factory = RecFactory { log: log.clone() };
        slicer::fill::lightning::tree_node::c_srand(1);
        ts3.generate(&factory, &NoOrganic).expect("generate");
        r.out(format!(
            "{s} generate state {} {} {} {} {} {} {} {} {}",
            hb(ts3.has_sharp_tails),
            hb(ts3.has_cantilever),
            hd(ts3.max_cantilever_dist),
            ts3.m_highest_overhang_layer,
            ts3.m_raft_layers,
            ts3.contact_nodes.len(),
            ts3.m_ts_data.contact_nodes.len(),
            ts3.m_ts_data.layer_heights.len(),
            hb(ts3.m_support_merge_plan.enabled)
        ));
        dump_fold(r, &format!("{s} generate fold"), &ts3);
        dump_nodes_of(r, &format!("{s} generate nodes"), &ts3.m_ts_data.contact_nodes, &ts3.contact_nodes, true, true);
        dump_support_layers(r, &format!("{s} generate areas"), ts3.m_object);
        if let Some(generator) = ts3.generator.as_ref() {
            dump_lightning(r, &format!("{s} generate lightning_trees"), generator);
        }
        let tag =
            if ts3.m_support_params.base_fill_pattern == InfillPattern::Lightning { format!("{s} generate_toolpaths_lightning") } else { format!("{s} generate_toolpaths") };
        let mut fills: Vec<String> = log.borrow_mut().drain(..).collect();
        fills.sort();
        let mut body = String::new();
        for l in &fills {
            body += l;
            body.push('\n');
        }
        if r.full {
            for l in &fills {
                r.out(format!("{tag} fill {l}"));
            }
        }
        r.out(format!("{tag} fills {} #{}", fills.len(), fnv(&body)));
        let layers = ts3.m_object.support_layers();
        for (i, sl) in layers.iter().enumerate() {
            r.outb(format!("{tag} layer {} {} {}", i, hb(sl.support_fills.no_sort), sl.support_fills.entities.len()), ents_s(&sl.support_fills.entities), true);
        }
    }
}

fn read_vectors() -> Vec<String> {
    // A diagnostic: a file of the generator with every line in full.
    if let Ok(path) = std::env::var("TREE_SUPPORT_PARITY_VECTORS") {
        let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
        return text.lines().map(|l| l.to_string()).collect();
    }
    let path = format!("{}/tests/data/tree_support/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).unwrap_or_else(|e| panic!("{path}: {e}"));
    text.lines().map(|l| l.to_string()).collect()
}

#[test]
fn tree_support_native_parity() {
    let mut r = Run { expected: read_vectors(), pos: 0, lines: Vec::new(), surplus: 0, full: false };
    r.full = r.input("in mode").expect("mode") == "full";
    // A diagnostic: one scenario only, of a file the generator wrote for that scenario.
    let only: Option<usize> = std::env::var("TREE_SUPPORT_PARITY_ONLY").ok().and_then(|v| v.parse().ok());
    if only.is_none() {
        run_helpers(&mut r);
    }
    match only {
        Some(scenario) => run_scenario(&mut r, scenario),
        // gen.cpp `NUM_SCENARIOS`
        None => {
            for scenario in 0..45 {
                run_scenario(&mut r, scenario);
            }
            // gen.cpp `FIRST_EXTRA`, `NUM_EXTRA`
            for scenario in 100000..100011 {
                run_scenario(&mut r, scenario);
            }
            // gen.cpp `FIRST_MULTI_RANDOM`, `NUM_MULTI_RANDOM`
            for scenario in 200000..200027 {
                run_scenario(&mut r, scenario);
            }
        }
    }
    // A diagnostic: the lines this test produced, written where TREE_SUPPORT_PARITY_DUMP names a file.
    if let Ok(path) = std::env::var("TREE_SUPPORT_PARITY_DUMP") {
        let _ = std::fs::write(path, r.lines.join("\n"));
    }
    let mut mismatches = r.surplus;
    let mut not_compared = 0usize;
    let mut by_kind: std::collections::BTreeMap<String, (usize, usize)> = std::collections::BTreeMap::new();
    for k in 0..r.expected.len().max(r.lines.len()) {
        let e = r.expected.get(k).map(|s| s.as_str()).unwrap_or("<missing>");
        let l = r.lines.get(k).map(|s| s.as_str()).unwrap_or("<missing>");
        let kind: String = e.split(' ').take(if e.starts_with('s') { 2 } else { 1 }).collect::<Vec<_>>().join(" ");
        let entry = by_kind.entry(kind).or_insert((0, 0));
        entry.0 += 1;
        if e != l {
            entry.1 += 1;
            // Counted, not compared: see the toolpaths of run_scenario().
            if matches!(e.split(' ').nth(1), Some("toolpaths_lightning") | Some("generate_toolpaths_lightning")) || e.contains(" lightning_trees chained ") {
                not_compared += 1;
                continue;
            }
            mismatches += 1;
            if mismatches <= 12 {
                let cut = |s: &str| if s.len() > 700 { format!("{}...", &s[..700]) } else { s.to_string() };
                eprintln!("line {}:\n  C++:  {}\n  Rust: {}", k + 1, cut(e), cut(l));
            }
        }
    }
    for (kind, (n, bad)) in &by_kind {
        eprintln!("{kind}: {n} lines, {bad} mismatches");
    }
    eprintln!("{} lines, {} mismatches", r.expected.len(), mismatches);
    eprintln!("{not_compared} lines of the lightning toolpaths differ and are not compared");
    // Every line has to be equal; the surplus lines (a stage that wrote more lines than the pin) count.
    assert_eq!(mismatches, 0);
}
