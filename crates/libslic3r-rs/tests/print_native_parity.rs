//! Native parity of the Print.cpp / PrintApply.cpp port (`slicer::print::faithful`,
//! `slicer::print_apply`) against vectors made by the C++ at the pin (tests/data/print/gen.cpp).
//!
//! The generator builds small Models with a full print config, runs `Print::apply` and prints the
//! PrintObjects (instances, configs, regions, the region lists of their layer ranges), the query
//! functions of Print.cpp and the static helpers of PrintApply.cpp. This test builds the same
//! models and assembles the Print as `Print::apply` does for a fresh Print, with the ported
//! functions: the config diffs and their application (PrintApply.cpp:1586-1678), the PrintObjects
//! from `print_objects_from_model_object` with `object_config_from_model_object`
//! (PrintApply.cpp:1962-1990), the regions from `generate_print_object_regions`
//! (PrintApply.cpp:2251-2263) and `Print::m_print_regions` (PrintApply.cpp:2275-2292). Every line
//! has to be equal.

use slicer::config::{
    ConfigBase, ConfigOptionFloats, ConfigOptionInt, ConfigOptionInts, ConfigOptionResolver, ConfigOptionStrings, ConfigSubstitutionContext,
    ForwardCompatibilitySubstitutionRule,
};
use slicer::geometry::geometry::{Transform3d, Vec3d};
use slicer::model::*;
use slicer::normal_utils::indexed_triangle_set;
use slicer::object_id::ObjectID;
use slicer::print::faithful::{AlignedBox3f, Print, PrintObject};
use slicer::print_apply::*;
use slicer::print_config::config_def::DynamicPrintConfig;
use slicer::triangle_mesh::{its_make_cube, Vec3f};
use std::rc::Rc;

struct Out {
    lines: Vec<String>,
    base: usize,
}

impl Out {
    fn out(&mut self, s: String) {
        self.lines.push(s);
    }
    fn id(&self, id: ObjectID) -> String {
        if id.id == 0 {
            "-1".into()
        } else {
            format!("{}", id.id as i64 - self.base as i64)
        }
    }
}

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
fn hm(t: &Transform3d) -> String {
    let mut o = Vec::new();
    for r in 0..3 {
        for c in 0..4 {
            o.push(hd(t[(r, c)]));
        }
    }
    o.join(" ")
}
fn vi<T: std::fmt::Display>(v: &[T]) -> String {
    let mut o = format!("{}", v.len());
    for x in v {
        o += &format!(" {}", x);
    }
    o
}
fn bx(b: &AlignedBox3f) -> String {
    format!("{} {} {} {} {} {}", hf(b.min[0]), hf(b.min[1]), hf(b.min[2]), hf(b.max[0]), hf(b.max[1]), hf(b.max[2]))
}

fn box_its(sx: f64, sy: f64, sz: f64, ox: f32, oy: f32, oz: f32) -> indexed_triangle_set {
    let mut its = its_make_cube(sx, sy, sz);
    for v in &mut its.vertices {
        *v += Vec3f::new(ox, oy, oz);
    }
    its
}

fn mesh(its: indexed_triangle_set) -> ModelMesh {
    ModelMesh::from_its(its)
}

fn deser(config: &mut DynamicPrintConfig, key: &str, value: &str) {
    let mut ctxt = ConfigSubstitutionContext::new(ForwardCompatibilitySubstitutionRule::Disable);
    config.set_deserialize(key, value, &mut ctxt, false).unwrap_or_else(|e| panic!("{key}={value}: {e:?}"));
}

/// gen.cpp `base_config(scenario)`
fn base_config(scenario: i32) -> DynamicPrintConfig {
    let mut config = DynamicPrintConfig::full_print_config();
    config.set_num_filaments(3).unwrap();
    config.set_key_value(
        "filament_colour",
        Box::new(ConfigOptionStrings::from_values(vec!["#FF0000".into(), "#00FF00".into(), "#0000FF".into()])),
    );
    config.set_key_value("filament_map", Box::new(ConfigOptionInts::from_values(vec![1, 1, 1])));
    if scenario == 1 {
        deser(&mut config, "skirt_loops", "2");
        deser(&mut config, "skirt_height", "1");
        deser(&mut config, "brim_type", "outer_only");
        deser(&mut config, "brim_width", "5");
    }
    if scenario == 2 {
        deser(&mut config, "draft_shield", "enabled");
        deser(&mut config, "skirt_loops", "1");
        deser(&mut config, "ooze_prevention", "1");
        deser(&mut config, "enable_prime_tower", "1");
        deser(&mut config, "wall_filament", "2");
        deser(&mut config, "sparse_infill_density", "15%");
    }
    if scenario == 3 {
        deser(&mut config, "enable_support", "1");
        deser(&mut config, "support_filament", "3");
        deser(&mut config, "support_interface_filament", "0");
        deser(&mut config, "brim_type", "no_brim");
        deser(&mut config, "timelapse_type", "1");
        deser(&mut config, "enable_prime_tower", "1");
        deser(&mut config, "print_sequence", "by object");
    }
    if scenario == 4 {
        deser(&mut config, "enable_prime_tower", "1");
        deser(&mut config, "wipe_tower_no_sparse_layers", "1");
        deser(&mut config, "wall_filament", "2");
        deser(&mut config, "initial_layer_line_width", "0.5");
        deser(&mut config, "skirt_loops", "2");
        deser(&mut config, "brim_width", "3");
    }
    config
}

fn range_key(lo: f64, hi: f64) -> (ordered_float::OrderedFloat<f64>, ordered_float::OrderedFloat<f64>) {
    (ordered_float::OrderedFloat(lo), ordered_float::OrderedFloat(hi))
}

/// gen.cpp `build_model(scenario)`
fn build_model(scenario: i32, o: &mut Out) -> Model {
    let mut model = Model::new();
    o.base = model.id().id;
    if scenario == 0 || scenario == 1 {
        let ob = model.add_object_new();
        ob.name = "cube".into();
        ob.add_volume_mesh(mesh(box_its(20.0, 20.0, 20.0, -10.0, -10.0, 0.0)), ModelVolumeType::ModelPart, true);
        ob.add_instance().set_offset(&Vec3d::new(100.0, 100.0, 0.0));
        ob.add_instance().set_offset(&Vec3d::new(140.5, 90.25, 0.0));
    }
    if scenario == 1 || scenario == 2 {
        let ob = model.add_object_new();
        ob.name = "two".into();
        ob.config.set("extruder", 2);
        ob.add_volume_mesh(mesh(box_its(10.0, 12.0, 8.0, -5.0, -6.0, 0.0)), ModelVolumeType::ModelPart, true).config.set("wall_filament", 3);
        {
            let v1 = ob.add_volume_mesh(mesh(box_its(4.0, 4.0, 4.0, 0.0, 0.0, 2.0)), ModelVolumeType::ParameterModifier, true);
            v1.config.set("sparse_infill_filament", 1);
            v1.set_offset(&Vec3d::new(1.0, -1.0, 0.5));
        }
        ob.add_volume_mesh(mesh(box_its(3.0, 3.0, 3.0, 20.0, 20.0, 0.0)), ModelVolumeType::ParameterModifier, true).config.set("wall_loops", 5);
        ob.add_volume_mesh(mesh(box_its(2.0, 2.0, 10.0, -1.0, -1.0, 0.0)), ModelVolumeType::NegativeVolume, true);
        ob.add_instance().set_offset(&Vec3d::new(60.0, 60.0, 0.0));
        {
            let i1 = ob.add_instance();
            i1.set_offset(&Vec3d::new(80.0, 40.0, 0.0));
            i1.set_rotation(&Vec3d::new(0.0, 0.0, 0.5));
        }
        ob.add_instance().set_offset(&Vec3d::new(30.0, 120.0, 0.0));
    }
    if scenario == 2 || scenario == 3 {
        let ob = model.add_object_new();
        ob.name = "ranges".into();
        ob.add_volume_mesh(mesh(box_its(15.0, 15.0, 30.0, -7.5, -7.5, 0.0)), ModelVolumeType::ModelPart, true);
        let mut ctxt = ConfigSubstitutionContext::new(ForwardCompatibilitySubstitutionRule::Disable);
        {
            let r = ob.layer_config_ranges.entry(range_key(2.0, 6.0)).or_default();
            r.set("extruder", 3);
            r.set_deserialize("layer_height", "0.2", &mut ctxt, false).unwrap();
        }
        {
            let r = ob.layer_config_ranges.entry(range_key(10.0, 14.0)).or_default();
            r.set("wall_filament", 2);
            r.set_deserialize("layer_height", "0.2", &mut ctxt, false).unwrap();
        }
        ob.add_instance().set_offset(&Vec3d::new(150.0, 50.0, 0.0));
    }
    if scenario == 4 {
        {
            let ob = model.add_object_new();
            ob.name = "tall".into();
            ob.add_volume_mesh(mesh(box_its(10.0, 10.0, 40.0, -5.0, -5.0, 0.0)), ModelVolumeType::ModelPart, true);
            ob.add_instance().set_offset(&Vec3d::new(32.0, 232.0, 0.0));
        }
        let ob = model.add_object_new();
        ob.name = "short".into();
        ob.config.set("extruder", 2);
        ob.add_volume_mesh(mesh(box_its(12.0, 8.0, 2.0, -6.0, -4.0, 0.0)), ModelVolumeType::ModelPart, true);
        ob.add_instance().set_offset(&Vec3d::new(150.0, 150.0, 0.0));
        let i2 = ob.add_instance();
        i2.set_offset(&Vec3d::new(20.0, 200.0, 0.0));
        i2.set_rotation(&Vec3d::new(0.0, 0.0, 0.25));
    }
    if scenario == 3 {
        let ob = model.add_object_new();
        ob.name = "support".into();
        ob.config.set("support_interface_filament", 2);
        ob.add_volume_mesh(mesh(box_its(8.0, 8.0, 8.0, -4.0, -4.0, 0.0)), ModelVolumeType::ModelPart, true);
        ob.add_instance().set_offset(&Vec3d::new(50.0, 150.0, 0.0));
    }
    model
}

/// The Print `Print::apply(model, config)` makes on a fresh Print (see the module comment).
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
        let config = object_config_from_model_object(&print.m_default_object_config, model_object, num_extruders, &mut print_variant_index);
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
    // PrintApply.cpp:2326-2331
    for i in 0..print.m_objects.len() {
        PrintObject::update_slicing_parameters(&mut print, i);
        print.m_support_used |= print.m_objects[i].m_config.opt_bool("enable_support");
    }
    print
}

fn dump_helpers(o: &mut Out, scenario: i32, model: &Model, config: &DynamicPrintConfig) {
    let sc = format!("s{}", scenario);
    for (im, mo) in model.objects.iter().enumerate() {
        let p = format!("{} m{}", sc, im);
        let trafos = print_objects_from_model_object(mo);
        o.out(format!("{} trafos {}", p, trafos.len()));
        for t in &trafos {
            let mut inst = String::new();
            for pi in &t.instances {
                inst += &format!(" {} {} {}", pi.model_instance, pi.shift.x, pi.shift.y);
            }
            o.out(format!("{} trafo {} {}{}", p, hm(&t.trafo), t.instances.len(), inst));
        }
        let lr = LayerRanges::new(&mo.layer_config_ranges);
        let mut rs = String::new();
        for r in lr.begin() {
            rs += &format!(" {} {} {}", hd(r.layer_height_range.0), hd(r.layer_height_range.1), hb(r.config.is_some()));
        }
        o.out(format!("{} layer_ranges {}{}", p, lr.size(), rs));
        o.out(format!(
            "{} lr_config {} {} {}",
            p,
            hb(lr.config((2.0, 6.0)).is_some()),
            hb(lr.config((0.0, 2.0)).is_some()),
            hb(lr.config((6.0, 10.0)).is_some())
        ));
        for (iv, v) in mo.volumes.iter().enumerate() {
            o.out(format!("{} v{} solid_or_modifier {}", p, iv, hb(model_volume_solid_or_modifier(v))));
        }
        o.out(format!(
            "{} lh_equal {} {}",
            p,
            hb(layer_height_ranges_equal(&mo.layer_config_ranges, &mo.layer_config_ranges, true)),
            hb(layer_height_ranges_equal(&mo.layer_config_ranges, &TLayerConfigRanges::new(), false))
        ));
    }
    let mut changed = config.clone();
    changed.set_key_value("skirt_loops", Box::new(ConfigOptionInt::with_value(7)));
    changed.set_key_value("nozzle_temperature", Box::new(ConfigOptionInts::from_values(vec![222, 223, 224])));
    changed.set_key_value("wipe_tower_x", Box::new(ConfigOptionFloats::from_values(vec![10.0, 20.0])));
    // PrintConfig pc; pc.apply(config, true);
    let mut pc = slicer::print::faithful::static_config_defaults("PrintConfig");
    let keys = pc.keys();
    let _ = pc.apply_only(config, &keys, true);
    let mut overrides = DynamicPrintConfig::new();
    let d1 = print_config_diffs(&pc, &changed, &mut overrides, 0, &[1, 1, 1]);
    let mut s1 = String::new();
    for k in &d1 {
        s1 += &format!(" {}", k);
    }
    o.out(format!("{} print_config_diffs {}{}", sc, d1.len(), s1));
    let d2 = full_print_config_diffs(config, &changed, 0);
    let mut s2 = String::new();
    for k in &d2 {
        s2 += &format!(" {}", k);
    }
    o.out(format!("{} full_print_config_diffs {}{}", sc, d2.len(), s2));
    o.out(format!(
        "{} same_ams {} {} {}",
        sc,
        hb(same_ams_grouping(&[1, 2, 3], &[1, 4, 3], 2)),
        hb(same_ams_grouping(&[1, 2, 3], &[2, 2, 3], 2)),
        hb(same_ams_grouping(&[1, 2], &[1, 2, 3], 2))
    ));
}

fn dump_print(o: &mut Out, scenario: i32, print: &Print) {
    let sc = format!("s{}", scenario);
    o.out(format!("{} objects {} regions {}", sc, print.objects().len(), print.num_print_regions()));
    for (io, po) in print.objects().iter().enumerate() {
        let mo = po.model_object(print);
        let p = format!("{} o{}", sc, io);
        o.out(format!("{} model {} trafo {}", p, po.m_model_object, hm(po.trafo())));
        let mut inst = String::new();
        for pi in po.instances() {
            inst += &format!(" {} {} {}", pi.model_instance, pi.shift.x, pi.shift.y);
        }
        o.out(format!("{} instances {}{}", p, po.instances().len(), inst));
        let oc = po.config();
        let enum_of = |key: &str| oc.option(key).and_then(|x| x.get_int().ok()).unwrap_or(0);
        o.out(format!(
            "{} config {} {} {} {} {} {} {} {}",
            p,
            oc.opt_int("support_filament"),
            oc.opt_int("support_interface_filament"),
            hb(oc.opt_bool("enable_support")),
            oc.opt_int("raft_layers"),
            enum_of("brim_type"),
            hd(*oc.opt_float("brim_width")),
            hb(po.has_brim()),
            hb(po.has_support_material())
        ));
        let r = po.shared_regions().unwrap();
        let r = r.borrow();
        o.out(format!("{} all_regions {} layer_ranges {} cached {}", p, r.all_regions.len(), r.layer_ranges.len(), r.cached_volume_ids.len()));
        for (ir, reg) in r.all_regions.iter().enumerate() {
            let reg = reg.borrow();
            let rc = reg.config();
            o.out(format!(
                "{} region {} {} {} {} {} {} {} {} {}",
                p,
                ir,
                rc.opt_int("wall_filament"),
                rc.opt_int("sparse_infill_filament"),
                rc.opt_int("solid_infill_filament"),
                rc.opt_int("wall_loops"),
                hd(*rc.opt_float("sparse_infill_density")),
                reg.print_object_region_id(),
                reg.print_region_id(),
                hb(reg.gradient_volume_id().valid())
            ));
            o.out(format!("{} region_hash {} {}", p, ir, reg.config_hash() as i64));
        }
        for (il, lr) in r.layer_ranges.iter().enumerate() {
            let l = format!("{} lr{}", p, il);
            o.out(format!("{} range {} {} {}", l, hd(lr.layer_height_range.0), hd(lr.layer_height_range.1), hb(lr.config.is_some())));
            for ve in &lr.volumes {
                o.out(format!("{} extents {} {}", l, o.id(ve.volume_id), bx(&ve.bbox)));
            }
            for vr in &lr.volume_regions {
                let rid = vr.region.as_ref().map(|x| x.borrow().print_object_region_id()).unwrap_or(-1);
                o.out(format!("{} vregion {} {} {}", l, vr.model_volume.unwrap(), vr.parent, rid));
            }
            for pr in &lr.painted_regions {
                o.out(format!("{} pregion {} {} {}", l, pr.extruder_id, pr.parent, pr.region.as_ref().unwrap().borrow().print_object_region_id()));
            }
        }
        let _ = mo;
        o.out(format!("{} object_extruders {}", p, vi(&print.print_object_object_extruders(po))));
    }
    o.out(format!("{} extruders {} custom {}", sc, vi(&print.extruders(false)), vi(&print.extruders(true))));
    o.out(format!("{} object_extruders {}", sc, vi(&print.object_extruders())));
    o.out(format!("{} support_material_extruders {}", sc, vi(&print.support_material_extruders())));
    o.out(format!(
        "{} flags {} {} {} {} {} {} {} {} {} {} {} {}",
        sc,
        hb(print.has_skirt()),
        hb(print.has_infinite_skirt()),
        hb(print.has_brim()),
        hb(print.has_auto_brim()),
        hb(print.has_support_material()),
        hb(print.is_sequential_print()),
        hb(print.has_wipe_tower()),
        hb(print.enable_timelapse_print()),
        hb(print.uses_multiple_nozzle_diameters()),
        hb(print.support_material_on_wipe_tower()),
        hb(print.is_dynamic_group_reorder()),
        hb(print.empty())
    ));
    o.out(format!(
        "{} num_object_instances {} max_allowed_layer_height {} skirt_first_layer_height {}",
        sc,
        print.num_object_instances(),
        hd(print.max_allowed_layer_height()),
        hd(print.skirt_first_layer_height())
    ));
    let usage: Vec<i32> = print.get_filament_usage_type().iter().map(|t| *t as i32).collect();
    o.out(format!("{} usage {}", sc, vi(&usage)));
    o.out(format!(
        "{} filament_maps {} volume_maps {} mode {}",
        sc,
        vi(&print.get_filament_maps()),
        vi(&print.get_filament_volume_maps()),
        print.get_filament_map_mode() as i32
    ));
    let modes: Vec<i32> = print.get_available_filament_map_modes().iter().map(|m| *m as i32).collect();
    o.out(format!("{} map_modes {}", sc, vi(&modes)));
    o.out(format!("{} extruder_id {} {} {}", sc, print.get_extruder_id(0), print.get_extruder_id(2), print.get_extruder_id(7)));
    let pu = print.get_physical_unprintable_filaments(&[0, 1, 2]);
    o.out(format!("{} physical_unprintable {}", sc, pu.len()));
    let rids: Vec<i32> = (0..print.num_print_regions()).map(|i| print.get_print_region(i).borrow().print_region_id()).collect();
    o.out(format!("{} print_region_ids {}", sc, vi(&rids)));
}


/// gen.cpp `dump_verify`
fn dump_verify(o: &mut Out, scenario: i32, print: &mut Print) {
    let sc = format!("s{}", scenario);
    let num_extruders = print.m_config.option("filament_diameter").and_then(|x| x.as_vector_base()).map(|v| v.size()).unwrap_or(0);
    for io in 0..print.m_objects.len() {
        let regions = print.m_objects[io].m_shared_regions.clone().unwrap();
        if io > 0 && Rc::ptr_eq(print.m_objects[io - 1].m_shared_regions.as_ref().unwrap(), &regions) {
            continue;
        }
        let mo_idx = print.m_objects[io].m_model_object;
        let default_region_config = print.m_default_region_config.clone();
        let mut vidx = vec![0];
        let mut calls = 0usize;
        let ok = {
            let mut cb = |_: &DynamicPrintConfig, _: &DynamicPrintConfig, k: &[String]| calls += k.len();
            verify_update_print_object_regions(
                &print.m_model.objects[mo_idx],
                Some(&print.m_model),
                &default_region_config,
                num_extruders,
                &[],
                &mut regions.borrow_mut(),
                &mut cb,
                &mut vidx,
            )
        };
        let refs: String = regions.borrow().all_regions.iter().map(|r| format!(" {}", print_region_ref_cnt(&r.borrow()))).collect();
        o.out(format!("{} verify o{} {} {}{}", sc, io, hb(ok), calls, refs));
        if let Some(v) = print.m_model.objects[mo_idx].volumes.iter_mut().find(|v| v.is_modifier()) {
            v.config.set("wall_loops", 6);
        }
        calls = 0;
        let ok = {
            let mut cb = |_: &DynamicPrintConfig, _: &DynamicPrintConfig, k: &[String]| calls += k.len();
            verify_update_print_object_regions(
                &print.m_model.objects[mo_idx],
                Some(&print.m_model),
                &default_region_config,
                num_extruders,
                &[],
                &mut regions.borrow_mut(),
                &mut cb,
                &mut vidx,
            )
        };
        let wl: String = regions
            .borrow()
            .all_regions
            .iter()
            .map(|r| format!(" {} {}", r.borrow().config().opt_int("wall_loops"), print_region_ref_cnt(&r.borrow())))
            .collect();
        o.out(format!("{} verify2 o{} {} {}{}", sc, io, hb(ok), calls, wl));
    }
}


fn flow_s(f: &slicer::flow::Flow) -> String {
    format!("{} {} {} {}", hf(f.width() as f32), hf(f.height() as f32), hf(f.spacing() as f32), hf(f.nozzle_diameter() as f32))
}
fn poly_s(p: &slicer::geometry::Polygon) -> String {
    let mut o = format!("{}", p.points.len());
    for pt in &p.points {
        o += &format!(" {} {}", pt.x, pt.y);
    }
    o
}
fn polys_s(ps: &[slicer::geometry::Polygon]) -> String {
    let mut o = format!("{}", ps.len());
    for p in ps {
        o += &format!(" {}", poly_s(p));
    }
    o
}
fn bb_s(b: &slicer::geometry::BoundingBox) -> String {
    format!("{} {} {} {}", b.min.x, b.min.y, b.max.x, b.max.y)
}

/// gen.cpp `dump_more`: the slicing parameters, the printed filaments, the flows, the wipe tower estimate
/// and the compacted wipe tower clearance.
fn dump_more(o: &mut Out, scenario: i32, print: &mut Print) {
    use slicer::print::faithful::*;
    let sc = format!("s{}", scenario);
    for io in 0..print.m_objects.len() {
        let sp = print.m_objects[io].compute_slicing_parameters(print, 0.0);
        o.out(format!(
            "{} o{} sp {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {}",
            sc,
            io,
            hb(sp.valid),
            sp.base_raft_layers,
            sp.interface_raft_layers,
            hd(sp.base_raft_layer_height),
            hd(sp.interface_raft_layer_height),
            hd(sp.contact_raft_layer_height),
            hd(sp.layer_height),
            sp.layer_grid_subdivision,
            sp.layer_grid_multiplier,
            hb(sp.single_diameter_layering),
            hd(sp.min_layer_height),
            hd(sp.max_layer_height),
            hd(sp.max_suport_layer_height),
            hd(sp.first_print_layer_height),
            hd(sp.first_object_layer_height),
            hb(sp.first_object_layer_bridging),
            hb(sp.soluble_interface),
            hd(sp.gap_raft_object),
            hd(sp.gap_object_support),
            hd(sp.gap_support_object),
            hd(sp.raft_base_top_z),
            hd(sp.raft_interface_top_z),
            hd(sp.raft_contact_top_z),
            hd(sp.object_print_z_min),
            hd(sp.object_print_z_max)
        ));
        PrintObject::update_slicing_parameters(print, io);
        let sp = print.m_objects[io].slicing_parameters();
        o.out(format!("{} o{} sp_valid {} {}", sc, io, hb(sp.valid), hd(sp.object_print_z_max)));
    }
    o.out(format!("{} printed_filaments {} single_nozzle {}", sc, vi(&print.printed_filaments()), hd(print.resolve_single_diameter_layering_nozzle())));
    o.out(format!("{} brim_flow {}", sc, flow_s(&print.brim_flow().unwrap())));
    o.out(format!("{} skirt_flow {}", sc, flow_s(&print.skirt_flow().unwrap())));
    for f in 1..=3u32 {
        let last = print.m_objects.len() - 1;
        o.out(format!(
            "{} skirt_flow_f{} {} {}",
            sc,
            f,
            flow_s(&print.skirt_flow_for_filament(f).unwrap()),
            flow_s(&print.skirt_flow_for_object(&print.m_objects[last], f).unwrap())
        ));
    }
    {
        let o0 = &print.m_objects[0];
        o.out(format!(
            "{} md_flows {} {} {}",
            sc,
            flow_s(&print.multi_diameter_brim_flow(o0, 2).unwrap()),
            flow_s(&print.multi_diameter_skirt_flow_on_brim_layer(o0, 2).unwrap()),
            flow_s(&print.multi_diameter_support_brim_flow(o0).unwrap())
        ));
    }
    let n = print.extruders(false).len();
    let wtd = print.wipe_tower_data(n);
    o.out(format!("{} wtd {} {}", sc, hf(wtd.depth), hf(wtd.brim_width)));
    o.out(format!("{} height_limit_lines {}", sc, hb(should_show_height_limit_lines(print))));
    let mut polys: Vec<slicer::geometry::Polygon> = Vec::new();
    let mut hp: Vec<(slicer::geometry::Polygon, f32)> = Vec::new();
    let e = Print::compacted_wipe_tower_clearance_valid(print, Some(&mut polys), Some(&mut hp));
    let mut hps = format!("{}", hp.len());
    for h in &hp {
        hps += &format!(" {} {}", poly_s(&h.0), hf(h.1));
    }
    let eid = o.id(e.object.unwrap_or(ObjectID { id: 0 }));
    o.out(format!("{} compacted_valid {} {} {} hp {}", sc, hb(!e.string.is_empty()), eid, polys_s(&polys), hps));
    o.out(format!("{} compacted_valid_msg {}", sc, e.string));

    let cfg = print.config();
    o.out(format!("{} padding {} {}", sc, hd(compacted_tower_footprint_padding(cfg, 2.0)), hd(compacted_tower_footprint_padding(cfg, -1.0))));
    let pt = |x: i64, y: i64| slicer::geometry::Point::new(x, y);
    let fp = slicer::geometry::Polygon::from_points(vec![pt(1000000, 2000000), pt(4500000, 2000000), pt(4500000, 6000000), pt(1000000, 6000000)]);
    let zone = compacted_wipe_tower_zone(cfg, &fp);
    o.out(format!(
        "{} zone {} body {} nozzle {} rod {} {}",
        sc,
        poly_s(&zone.hull),
        polys_s(&zone.grown_body),
        polys_s(&zone.grown_nozzle),
        bb_s(&zone.bbox_rod),
        hd(zone.body_radius)
    ));
    for dx in [0i64, 3000000, 8000000] {
        for rise in [1.0f64, 30.0] {
            let h = slicer::geometry::Polygon::from_points(vec![
                pt(5000000 + dx, 2500000),
                pt(6000000 + dx, 2500000),
                pt(6000000 + dx, 4000000),
                pt(5000000 + dx, 4000000),
            ]);
            let c = compacted_wipe_tower_clearance(cfg, &zone, &h, rise);
            o.out(format!(
                "{} clearance {} {} {} {} {} {} {} offender {}",
                sc,
                dx,
                hd(rise),
                hd(c.allowed_rise),
                hd(c.far_clearance),
                hb(c.near_body),
                hd(c.body_clearance),
                hb(compacted_tower_body_tier(&c)),
                poly_s(&compacted_wipe_tower_offender_outline(&h, c.body_clearance))
            ));
        }
    }
    o.out(format!("{} rings {} {}", sc, polys_s(&compacted_wipe_tower_rings(&zone, false)), compacted_wipe_tower_rings(&zone, true).len()));
}

/// gen.cpp `dump_wipe_tower`
fn dump_wipe_tower(o: &mut Out, print: &mut Print) {
    use slicer::geometry::{Point, Polyline};
    use slicer::print::faithful::FakeWipeTower;
    let mut fwt = FakeWipeTower::default();
    fwt.pos = [12.25, 30.5];
    let pl = |pts: &[(i64, i64)]| Polyline::from_points(pts.iter().map(|&(x, y)| Point::new(x, y)).collect());
    fwt.outer_wall.insert(ordered_float::OrderedFloat(0.2f32), vec![pl(&[(0, 0), (100000, 0), (100000, 50000)])]);
    fwt.outer_wall.insert(ordered_float::OrderedFloat(0.4f32), vec![pl(&[(10, 20), (30, 40)]), pl(&[(-5, 7), (9, -11), (13, 17)])]);
    fwt.outer_wall.insert(ordered_float::OrderedFloat(0.68f32), vec![pl(&[(1, 2), (3, 4)])]);
    let els = fwt.getTrueExtrusionLayersFromWipeTower();
    o.out(format!("wt layers {} {}", els.layers.len(), 3));
    for el in &els.layers {
        let mut l = format!("wt layer {} {}", hf(el.bottom_z), el.paths.len());
        for p in &el.paths {
            l += &format!(" | {} {} {} {}", hf(p.height as f32), hf(p.width as f32), hd(p.mm3_per_mm), p.role as i32);
            for pt in &p.polyline.points {
                l += &format!(" {} {}", pt.x, pt.y);
            }
        }
        o.out(l);
    }
    let wtd = &mut print.m_wipe_tower_data;
    wtd.rib_offset = [1.5, -0.75];
    fn mesh_line(o: &mut Out, name: &str, m: &slicer::model::ModelMesh) {
        let mut l = format!("{} {} {}", name, m.its.vertices.len(), m.its.indices.len());
        for v in m.its.vertices.iter().take(12) {
            l += &format!(" {} {} {}", hf(v.x), hf(v.y), hf(v.z));
        }
        for f in m.its.indices.iter().take(4) {
            l += &format!(" {} {} {}", f[0], f[1], f[2]);
        }
        o.out(l);
    }
    wtd.construct_mesh(35.0, 20.0, 30.0, 3.0, false, 8.0, 4.0, false);
    let d = wtd.wipe_tower_mesh_data.as_ref().unwrap();
    o.out(format!("wt mesh_bottom {}", poly_s(&d.bottom)));
    mesh_line(o, "wt mesh_tower", &d.real_wipe_tower_mesh);
    mesh_line(o, "wt mesh_brim", &d.real_brim_mesh);
    // gen.cpp leaves the rib tower branch out (see there).
    wtd.construct_mesh(0.0, 30.0, 25.0, 2.0, true, 8.0, 4.0, true);
    o.out(format!("wt empty {} {}", hb(wtd.wipe_tower_mesh_data.is_some()), wtd.wipe_tower_mesh_data.as_ref().unwrap().bottom.points.len()));
}

fn read_vectors() -> String {
    let path = format!("{}/tests/data/print/vectors.txt", env!("CARGO_MANIFEST_DIR"));
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"))
}

#[test]
fn print_native_parity() {
    let mut o = Out { lines: Vec::new(), base: 0 };
    for scenario in 0..5 {
        let model = build_model(scenario, &mut o);
        let config = base_config(scenario);
        dump_helpers(&mut o, scenario, &model, &config);
        let mut print = assemble(&model, &config);
        dump_print(&mut o, scenario, &print);
        dump_more(&mut o, scenario, &mut print);
        dump_verify(&mut o, scenario, &mut print);
        if scenario == 4 {
            dump_wipe_tower(&mut o, &mut print);
        }
    }
    let expected = read_vectors();
    let expected: Vec<&str> = expected.lines().collect();
    let mut mismatches = 0usize;
    for k in 0..expected.len().max(o.lines.len()) {
        let e = expected.get(k).copied().unwrap_or("<missing>");
        let r = o.lines.get(k).map(|s| s.as_str()).unwrap_or("<missing>");
        if e != r {
            mismatches += 1;
            if mismatches <= 40 {
                eprintln!("line {}:\n  C++:  {}\n  Rust: {}", k + 1, e, r);
            }
        }
    }
    eprintln!("{} lines, {} mismatches", expected.len(), mismatches);
    assert_eq!(mismatches, 0);
}

