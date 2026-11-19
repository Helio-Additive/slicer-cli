//! Native parity of the Support/TreeModelVolumes.cpp/.hpp port (`slicer::support::tree_model_volumes`) and
//! of the Support/TreeSupportCommon.hpp port (`slicer::support::tree_support_common`) against vectors made
//! by the C++ at the pin (tests/data/tree_model_volumes/gen.cpp).
//!
//! The generator builds, for sixteen scenarios, a Print of one box with `Print::apply`, fills in the
//! layers of its object by hand and runs, on one thread: `TreeSupportMeshGroupSettings(print_object)`,
//! `TreeSupportSettings(mesh settings, slicing parameters)` with getRadius, recommendedMinRadius,
//! getActualZ and the `layer_*` helpers, the `InterfacePlacer`, the constructor of `TreeModelVolumes`,
//! ceilRadius and getRadiusNextCeil, the getters of a volume that was not precalculated (each computes
//! what it needs), `clear_all_but_object_collision`, `clear`, and `precalculate()` with every cache it
//! leaves. The lines of the vectors that start with "in" are the inputs, which this test reads back;
//! every other line it has to reproduce, bit for bit. The long lines are compared by their FNV-1a hash;
//! `TREE_MODEL_VOLUMES_PARITY_VECTORS` names a file of the generator with every line in full.

mod support_shared;

use slicer::build_volume::BuildVolume;
use slicer::config::{ConfigOptionInts, ConfigOptionStrings};
use slicer::geometry::{Point, Polygon};
use slicer::model::*;
use slicer::print_config::config_def::DynamicPrintConfig;
use slicer::support::support_layer::{SupportGeneratorLayerStorage, SupportGeneratorLayersPtr};
use slicer::support::support_parameters::SupportParameters;
use slicer::support::tree_model_volumes::{AvoidanceType, TreeModelVolumes, TreeModelVolumesCache};
use slicer::support::tree_support_common::*;
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

fn coords_s(v: &[i64]) -> String {
    let mut o = format!("{}", v.len());
    for c in v {
        o += &format!(" {c}");
    }
    o
}
fn doubles_s(v: &[f64]) -> String {
    let mut o = format!("{}", v.len());
    for c in v {
        o += &format!(" {}", hd(*c));
    }
    o
}

/// gen.cpp `rect(x0, y0, x1, y1)`: `Point(double, double)` of `scale_()`.
fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Polygon {
    let s = |v: f64| (v / slicer::libslic3r::SCALING_FACTOR).round_ties_even() as i64;
    Polygon::from_points(vec![Point::new(s(x0), s(y0)), Point::new(s(x1), s(y0)), Point::new(s(x1), s(y1)), Point::new(s(x0), s(y1))])
}

const CACHES: [(&str, TreeModelVolumesCache); 11] = [
    ("collision", TreeModelVolumesCache::Collision),
    ("collision_holefree", TreeModelVolumesCache::CollisionHolefree),
    ("avoidance", TreeModelVolumesCache::Avoidance),
    ("avoidance_slow", TreeModelVolumesCache::AvoidanceSlow),
    ("avoidance_to_model", TreeModelVolumesCache::AvoidanceToModel),
    ("avoidance_to_model_slow", TreeModelVolumesCache::AvoidanceToModelSlow),
    ("placeable_areas", TreeModelVolumesCache::PlaceableAreas),
    ("avoidance_holefree", TreeModelVolumesCache::AvoidanceHolefree),
    ("avoidance_holefree_to_model", TreeModelVolumesCache::AvoidanceHolefreeToModel),
    ("wall_restrictions", TreeModelVolumesCache::WallRestrictions),
    ("wall_restrictions_min", TreeModelVolumesCache::WallRestrictionsMin),
];

/// gen.cpp `dump_caches(tag, v)`
fn dump_caches(c: &mut Ctx, tag: &str, v: &TreeModelVolumes) {
    for (name, cache) in CACHES {
        let sorted = v.cache_sorted(cache);
        c.r.out(format!("{tag} cache {name} {}", sorted.len()));
        let mut i = 0;
        while i < sorted.len() {
            let mut j = i + 1;
            while j < sorted.len() && sorted[i].0 .1 == sorted[j].0 .1 {
                j += 1;
            }
            let mut head = format!("{tag} cache {name} layer {} {}", sorted[i].0 .1, j - i);
            for e in &sorted[i..j] {
                head += &format!(" {}", e.0 .0);
            }
            if c.full {
                c.r.out(head);
                for e in &sorted[i..j] {
                    c.r.out(format!("{tag} cache {name} area {} {} {}", sorted[i].0 .1, e.0 .0, polys_s(&e.1)));
                }
            } else {
                let mut body = String::new();
                for e in &sorted[i..j] {
                    body += &polys_s(&e.1);
                    body += "\n";
                }
                c.r.out(format!("{head} #{}", fnv(&body)));
            }
            i = j;
        }
    }
}

/// gen.cpp `MEMBERS(v)`
fn members_s(v: &TreeModelVolumes) -> String {
    let m = v.members();
    format!(
        "{} {} {} {} {} {} {} {} {} {} {} {}",
        m.m_max_move,
        m.m_max_move_slow,
        m.m_min_resolution,
        hb(m.m_precalculated),
        m.m_current_outline_idx,
        m.m_current_min_xy_dist,
        m.m_current_min_xy_dist_delta,
        hb(m.m_support_rests_on_model),
        m.m_increase_until_radius,
        m.m_radius_0,
        doubles_s(&m.m_raft_layers),
        coords_s(&m.m_ignorable_radii)
    )
}

fn run_scenario(c: &mut Ctx, scenario: usize) {
    let s = format!("s{scenario}");
    let line = c.r.input(&format!("in {s} model")).expect("model");
    let mut t = Tok::new(&line);
    let (sx, sy, sz, ix, iy) = (t.i() as f64, t.i() as f64, t.i() as f64, t.i() as f64, t.i() as f64);
    let has_blocker = t.b();
    let (bsx, bsy, bsz, bx, by, bz) = (t.d(), t.d(), t.d(), t.d(), t.d(), t.d());
    let mut model = Model::new();
    {
        let o = model.add_object_new();
        o.name = "tee".into();
        o.add_volume_mesh(ModelMesh::from_its(box_its(sx, sy, sz, -(sx as f32) / 2.0, -(sy as f32) / 2.0, 0.0)), ModelVolumeType::ModelPart, true);
        if has_blocker {
            let b = o.add_volume_mesh(ModelMesh::from_its(box_its(bsx, bsy, bsz, bx as f32, by as f32, bz as f32)), ModelVolumeType::ModelPart, true);
            b.set_type(ModelVolumeType::SupportBlocker);
        }
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

    let mut layer_id = 0usize;
    while let Some(line) = c.r.input(&format!("in {s} layer")) {
        let mut t = Tok::new(&line);
        let (print_z, height, slice_z) = (t.d(), t.d(), t.d());
        let obj = &mut print.m_objects[0];
        let mut layer = slicer::layer::Layer::new(layer_id, obj.m_model_object, height, print_z, slice_z);
        layer.lslices = t.expolys();
        layer.lslices_extrudable = layer.lslices.clone();
        if layer_id > 0 {
            layer.lower_layer_id = Some(layer_id - 1);
            obj.m_layers[layer_id - 1].upper_layer_id = Some(layer_id);
        }
        obj.m_layers.push(layer);
        layer_id += 1;
    }
    let n_object_layers = layer_id;

    let line = c.r.input(&format!("in {s} soluble")).expect("soluble");
    SOLUBLE.store(Tok::new(&line).b(), std::sync::atomic::Ordering::Relaxed);

    let obj = &print.m_objects[0];
    let sp = obj.slicing_parameters().clone();

    // TreeSupportMeshGroupSettings
    let m = TreeSupportMeshGroupSettings::new(obj, &print).expect("TreeSupportMeshGroupSettings");
    c.r.out(format!(
        "{s} mesh_group_settings {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {}",
        m.layer_height,
        m.resolution,
        m.min_feature_size,
        hd(m.support_angle),
        m.support_line_width,
        m.support_roof_line_width,
        hb(m.support_bottom_enable),
        m.support_bottom_height,
        hb(m.support_material_buildplate_only),
        m.support_xy_distance,
        m.support_xy_distance_1st_layer,
        m.support_xy_distance_overhang,
        coords_s(&m.support_xy_distance_overhang_per_region),
        m.support_top_distance,
        m.support_bottom_distance,
        m.support_interface_skip_height,
        hb(m.support_roof_enable),
        m.support_roof_layers,
        hb(m.support_floor_enable),
        m.support_floor_layers,
        hd(m.minimum_roof_area),
        doubles_s(&m.support_roof_angles),
        m.support_roof_pattern.to_i32(),
        m.support_line_spacing,
        m.support_bottom_offset,
        m.support_wall_count,
        m.support_roof_line_distance,
        m.minimum_support_area,
        m.minimum_bottom_area,
        m.support_offset,
        hd(m.support_tree_angle),
        hd(m.support_tree_branch_diameter_angle),
        m.support_tree_branch_distance,
        m.support_tree_branch_diameter,
        hd(m.support_tree_angle_slow),
        m.support_tree_max_diameter_increase_by_merges_when_support_to_model,
        m.support_tree_min_height_to_model,
        m.support_tree_bp_diameter,
        hd(m.support_tree_top_rate),
        m.support_tree_tip_diameter
    ));

    // TreeSupportSettings
    let cfg = TreeSupportSettings::new(&m, &sp);
    c.r.out(format!(
        "{s} settings {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {}",
        cfg.support_line_width,
        cfg.layer_height,
        cfg.branch_radius,
        cfg.min_radius,
        cfg.maximum_move_distance,
        cfg.maximum_move_distance_slow,
        cfg.support_bottom_layers,
        cfg.tip_layers,
        hd(cfg.branch_radius_increase_per_layer),
        cfg.max_to_model_radius_increase,
        cfg.min_dtt_to_model,
        cfg.increase_radius_until_radius,
        cfg.increase_radius_until_layer,
        hb(cfg.support_rests_on_model),
        cfg.xy_distance,
        cfg.bp_radius,
        cfg.layer_start_bp_radius,
        hd(cfg.bp_radius_increase_per_layer),
        cfg.xy_min_distance,
        coords_s(&cfg.xy_min_distance_per_region),
        cfg.z_distance_top_layers,
        cfg.z_distance_bottom_layers,
        doubles_s(&cfg.support_roof_angles),
        cfg.roof_pattern.to_i32(),
        cfg.support_roof_line_width,
        cfg.support_line_spacing,
        cfg.support_bottom_offset,
        cfg.support_wall_count,
        cfg.resolution,
        cfg.support_roof_line_distance,
        cfg.interface_preference as i32,
        cfg.min_feature_size,
        doubles_s(&cfg.raft_layers),
        hb(cfg == cfg)
    ));
    {
        let mut line = format!("{s} get_radius");
        for dtt in [0usize, 1, 2, 5, cfg.tip_layers, cfg.tip_layers + 1, 40, 333] {
            for e in [0., 1., 2.5, 40.] {
                line += &format!(" {}", cfg.get_radius(dtt, e));
            }
        }
        c.r.out(line);
        let mut line = format!("{s} recommended_min_radius");
        for l in [0, 1, 2, 7, cfg.layer_start_bp_radius - 1, cfg.layer_start_bp_radius, cfg.layer_start_bp_radius + 1, 400] {
            line += &format!(" {}", cfg.recommended_min_radius(l));
        }
        c.r.out(line);
        let num_layers = (n_object_layers + cfg.raft_layers.len()) as i32;
        let mut line = format!("{s} layer_z");
        for l in [0usize, 1, 2, 3, 4, 11, (num_layers - 1) as usize, (num_layers + 3) as usize] {
            line += &format!(" {} {}", hd(layer_z(&sp, &cfg, l)), hd(layer_mid_z(&sp, &cfg, l as LayerIndex)));
        }
        c.r.out(line);
        let mut line = format!("{s} layer_idx");
        for z in [-1., 0., 0.05, 0.1, 0.2, 0.3, 0.30000000000000004, 0.4, 0.56, 1.1, 1.2, 3.7, 3.8, 3.9000000000000004, 7.123, 30.] {
            line += &format!(
                " {} {} {} {} {} {}",
                layer_idx_ceil(&sp, &cfg, z),
                layer_idx_floor(&sp, &cfg, z),
                layer_idx_mid_ceil(&sp, &cfg, z, num_layers),
                layer_idx_mid_floor(&sp, &cfg, z, num_layers),
                layer_idx_mid_ceil(&sp, &cfg, z, 0),
                layer_idx_mid_floor(&sp, &cfg, z, 0)
            );
        }
        c.r.out(line);
        let mut c2 = cfg.clone();
        c2.set_actual_z(vec![100, 250, 400]);
        c.r.out(format!("{s} get_actual_z {} {} {} {}", c2.get_actual_z(0), c2.get_actual_z(2), c2.get_actual_z(3), c2.get_actual_z(7)));
    }

    // InterfacePlacer
    {
        let support_params = SupportParameters::new(obj, &print).expect("SupportParameters");
        let num_layers = n_object_layers + cfg.raft_layers.len();
        let mut layer_storage = SupportGeneratorLayerStorage::new();
        let mut top_contacts: SupportGeneratorLayersPtr = vec![SUPPORT_GENERATOR_LAYER_PTR_NULL; num_layers];
        let mut top_interfaces: SupportGeneratorLayersPtr = vec![SUPPORT_GENERATOR_LAYER_PTR_NULL; num_layers];
        let mut top_base_interfaces: SupportGeneratorLayersPtr = vec![SUPPORT_GENERATOR_LAYER_PTR_NULL; num_layers];
        if support_params.has_top_contacts {
            let mut placer = InterfacePlacer::new(&sp, &support_params, &cfg);
            let ntop = support_params.num_top_interface_layers;
            placer.add_roof(vec![rect(0., 0., 3., 3.)], 12, 0, &mut layer_storage, &mut top_contacts, &mut top_interfaces, &mut top_base_interfaces);
            placer.add_roof(vec![rect(1., 1., 4., 4.)], 12, 0, &mut layer_storage, &mut top_contacts, &mut top_interfaces, &mut top_base_interfaces);
            placer.add_roof(vec![rect(5., 0., 8., 3.)], 11, std::cmp::min(1, ntop), &mut layer_storage, &mut top_contacts, &mut top_interfaces, &mut top_base_interfaces);
            placer.add_roof(vec![rect(5., 5., 8., 8.)], 10, ntop, &mut layer_storage, &mut top_contacts, &mut top_interfaces, &mut top_base_interfaces);
            let mut roofs: Vec<Vec<Polygon>> = Vec::new();
            for i in 0..=ntop {
                roofs.push(if i == 1 { Vec::new() } else { vec![rect(-9., i as f64, -6., i as f64 + 2.)] });
            }
            placer.add_roofs(roofs, 9, &mut layer_storage, &mut top_contacts, &mut top_interfaces, &mut top_base_interfaces);
            placer.add_roof_build_plate(vec![rect(-3., -3., -1., -1.)], 1, &mut layer_storage, &mut top_contacts, &mut top_interfaces, &mut top_base_interfaces);
            placer.add_roof_build_plate(vec![rect(-3., 3., -1., 5.)], 7, &mut layer_storage, &mut top_contacts, &mut top_interfaces, &mut top_base_interfaces);
        }
        c.r.out(format!(
            "{s} interface_placer {} {} {}",
            hb(support_params.has_top_contacts),
            support_params.num_top_interface_layers,
            support_params.num_top_interface_layers_only()
        ));
        for (which, layers) in [&top_contacts, &top_interfaces, &top_base_interfaces].into_iter().enumerate() {
            for (i, &l) in layers.iter().enumerate() {
                if l != SUPPORT_GENERATOR_LAYER_PTR_NULL {
                    let l = &layer_storage[l];
                    c.r.out(format!(
                        "{s} interface_placer layer {which} {i} {} {} {} {} {}",
                        l.layer_type as i32,
                        hd(l.print_z),
                        hd(l.bottom_z),
                        hd(l.height),
                        polys_s(&l.polygons)
                    ));
                }
            }
        }
    }

    // The build volume.
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

    // The constructor.
    {
        let v = TreeModelVolumes::from_print_object(obj, &print, &build_volume, cfg.maximum_move_distance, cfg.maximum_move_distance_slow, 0, &[])
            .expect("TreeModelVolumes");
        let mm = v.members();
        c.r.out(format!(
            "{s} ctor {} {} {} {} {} {}",
            members_s(&v),
            poly_s(&v.m_bed_area),
            polys_s(&mm.m_machine_border),
            mm.m_layer_outlines.len(),
            mm.m_layer_outlines[0].len(),
            mm.m_anti_overhang.len()
        ));
        for (l, o) in mm.m_layer_outlines[0].iter().enumerate() {
            c.outb(format!("{s} ctor outline {l}"), polys_s(o));
        }
        for (l, o) in mm.m_anti_overhang.iter().enumerate() {
            c.outb(format!("{s} ctor anti_overhang {l}"), polys_s(o));
        }

        // ceilRadius, getRadiusNextCeil
        let mut line = format!("{s} ceil_radius");
        for r in
            [0i64, 1, mm.m_radius_0 - 1, mm.m_radius_0, mm.m_radius_0 + 1, 49999, 50000, 99999, 100000, 149999, 150000, 150001, 224999, 225000, 225001, 400000, 1234567]
        {
            line += &format!(" {} {} {}", v.ceil_radius(r), v.ceil_radius_min_xy(r, true), v.ceil_radius_min_xy(r, false));
            if r > 0 {
                line += &format!(" {} {}", v.get_radius_next_ceil(r, true), v.get_radius_next_ceil(r, false));
            } else {
                line += " - -";
            }
        }
        c.r.out(line);

        // The getters on a volume that was not precalculated.
        let line = c.r.expected.iter().find(|l| l.starts_with(&format!("in {s} precalculate "))).expect("precalculate").clone();
        let max_layer = Tok::new(&line[format!("in {s} precalculate ").len()..]).i() as LayerIndex;
        let nl = mm.m_layer_outlines[0].len() as LayerIndex;
        let top = std::cmp::min(nl - 1, max_layer);
        let nop = || {};
        c.outb(format!("{s} lazy placeable0 {}", top / 2), polys_s(&v.get_placeable_areas(0, top / 2, &nop)));
        c.outb(format!("{s} lazy placeable0 again {}", top / 2), polys_s(&v.get_placeable_areas(0, top / 2, &nop)));
        for r in [0, cfg.min_radius, cfg.branch_radius] {
            for l in [0, 3, top] {
                for b in [true, false] {
                    c.outb(format!("{s} lazy collision {r} {l} {}", hb(b)), polys_s(&v.get_collision(r, l, b)));
                }
            }
        }
        for r in [cfg.min_radius, cfg.branch_radius + 30000] {
            for l in [0, 2, top - 1] {
                for ty in [AvoidanceType::Slow, AvoidanceType::FastSafe, AvoidanceType::Fast] {
                    for to_model in [false, true] {
                        c.outb(
                            format!("{s} lazy avoidance {r} {l} {} {}", ty as i32, hb(to_model)),
                            polys_s(&v.get_avoidance(r, l, ty, to_model, to_model)),
                        );
                    }
                }
            }
        }
        for r in [0, cfg.min_radius, cfg.branch_radius] {
            for l in [1, 4, top] {
                c.outb(format!("{s} lazy placeable {r} {l}"), polys_s(&v.get_placeable_areas(r, l, &nop)));
            }
        }
        for r in [cfg.min_radius, cfg.branch_radius] {
            for l in [1, 5, top] {
                for b in [true, false] {
                    c.outb(format!("{s} lazy wall_restriction {r} {l} {}", hb(b)), polys_s(&v.get_wall_restriction(r, l, b)));
                }
            }
        }
        for r in [0, 10, cfg.min_radius, cfg.branch_radius, 100000000] {
            for l in [0, 3, top, nl + 5] {
                let a = v.get_collision_lower_bound_area(l, r);
                c.outb(
                    format!("{s} lazy lower_bound {r} {l} {} {}", hb(a.is_some()), a.as_ref().map(|a| a.0).unwrap_or(-1)),
                    a.as_ref().map(|a| polys_s(&a.1)).unwrap_or_else(|| "-".to_string()),
                );
            }
        }
        dump_caches(c, &format!("{s} lazy"), &v);
        let mut v = v;
        v.clear_all_but_object_collision();
        dump_caches(c, &format!("{s} cleared"), &v);
        v.clear();
        c.r.out(format!("{s} cleared_all {}", v.cache_sorted(TreeModelVolumesCache::Collision).len()));
    }

    // precalculate()
    {
        let mut v = TreeModelVolumes::from_print_object(obj, &print, &build_volume, cfg.maximum_move_distance, cfg.maximum_move_distance_slow, 0, &[])
            .expect("TreeModelVolumes");
        let line = c.r.input(&format!("in {s} precalculate")).expect("precalculate");
        let max_layer = Tok::new(&line).i() as LayerIndex;
        v.precalculate(obj, max_layer, &|| {});
        c.r.out(format!("{s} precalculated {}", members_s(&v)));
        let mut line = format!("{s} ceil_radius");
        let mut r = 1i64;
        while r < 1300000 {
            line += &format!(" {} {}", v.ceil_radius_min_xy(r, true), v.ceil_radius_min_xy(r, false));
            r += 9973;
        }
        c.r.out(line);
        dump_caches(c, &format!("{s} pre"), &v);
    }
}

#[test]
fn tree_model_volumes_native_parity() {
    let run = match std::env::var("TREE_MODEL_VOLUMES_PARITY_VECTORS") {
        // A diagnostic: a file of the generator with every line in full.
        Ok(path) => {
            let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
            Run { expected: text.lines().map(|l| l.to_string()).collect(), pos: 0, lines: Vec::new(), surplus: 0 }
        }
        Err(_) => Run::load("tree_model_volumes"),
    };
    let mut c = Ctx { r: run, full: false };
    c.full = c.r.input("in mode").expect("mode") == "full";
    // A diagnostic: one scenario only, of a file the generator wrote for that scenario.
    let only: Option<usize> = std::env::var("TREE_MODEL_VOLUMES_PARITY_ONLY").ok().and_then(|v| v.parse().ok());
    match only {
        Some(scenario) => run_scenario(&mut c, scenario),
        // gen.cpp `NUM_SCENARIOS`
        None => {
            for scenario in 0..16 {
                run_scenario(&mut c, scenario);
            }
        }
    }
    // A diagnostic: the lines this test produced, written where TREE_MODEL_VOLUMES_PARITY_DUMP names a file.
    if let Ok(path) = std::env::var("TREE_MODEL_VOLUMES_PARITY_DUMP") {
        let _ = std::fs::write(path, c.r.lines.join("\n"));
    }
    let mut by_kind: std::collections::BTreeMap<String, (usize, usize)> = std::collections::BTreeMap::new();
    for (k, e) in c.r.expected.iter().enumerate() {
        if e.starts_with("in ") {
            continue;
        }
        let toks: Vec<&str> = e.split(' ').collect();
        let kind = if toks.len() > 3 && (toks[1] == "lazy" || toks[1] == "pre" || toks[1] == "cleared") {
            format!("{} {} {}", toks[1], toks[2], toks[3])
        } else {
            toks.get(1).copied().unwrap_or("").to_string()
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
    assert_eq!(bad, 0, "lines of the vectors that differ");
}
