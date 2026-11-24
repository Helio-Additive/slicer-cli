//! Native parity of the clearance checks of Print.cpp (`slicer::print::faithful`):
//! `Print::sequential_print_clearance_valid` (Print.cpp:777-1122) and `layered_print_cleareance_valid`
//! (Print.cpp:1123-1241), against vectors made by the C++ at the pin (tests/data/print_clearance/gen.cpp).
//!
//! The generator builds small Models (cubes with several instances, a rotated instance, tall and short
//! objects) with a full print config of three filaments and the keys of each scenario (sequential and
//! by-layer prints, bed exclusion areas, a clumping detection area, a prime tower), runs `Print::apply`
//! and both checks. This test builds the same models, assembles the Print as
//! tests/print_native_parity.rs does (its `assemble`, copied) and runs the ported checks. Every line has
//! to be equal: the messages, their object, the collision and height polygons, the warnings and the
//! ModelInstances' arrange_order the checks set.
#![allow(dead_code)]

use slicer::config::{ConfigBase, ConfigOptionInts, ConfigOptionResolver, ConfigOptionStrings, ConfigSubstitutionContext, ForwardCompatibilitySubstitutionRule};
use slicer::geometry::geometry::{Transform3d, Vec3d};
use slicer::model::*;
use slicer::normal_utils::indexed_triangle_set;
use slicer::object_id::ObjectID;
use slicer::print::faithful::{AlignedBox3f, Print, PrintObject};
use slicer::print_apply::*;
use slicer::print_config::config_def::DynamicPrintConfig;
use slicer::triangle_mesh::{its_make_cube, Vec3f};

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

fn msg(s: &str) -> String {
    s.replace('\n', "\\n")
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
    if scenario == 0 {
        deser(&mut config, "print_sequence", "by object");
        deser(&mut config, "skirt_loops", "2");
        deser(&mut config, "bed_exclude_area", "0x0,40x0,40x40,0x40");
    }
    if scenario == 1 {
        deser(&mut config, "print_sequence", "by object");
    }
    if scenario == 2 {
        deser(&mut config, "enable_prime_tower", "1");
        deser(&mut config, "wall_filament", "2");
        deser(&mut config, "wipe_tower_x", "90");
        deser(&mut config, "wipe_tower_y", "90");
        deser(&mut config, "enable_wrapping_detection", "1");
        deser(&mut config, "wrapping_exclude_area", "300x300,310x300,310x310,300x310");
    }
    if scenario == 3 || scenario == 7 {
        deser(&mut config, "enable_prime_tower", "1");
        deser(&mut config, "wall_filament", "2");
        deser(&mut config, "bed_exclude_area", "0x0,50x0,50x50,0x50");
        deser(&mut config, "wipe_tower_x", "20");
        deser(&mut config, "wipe_tower_y", "20");
        deser(&mut config, "enable_wrapping_detection", "1");
        deser(&mut config, "wrapping_exclude_area", "300x300,310x300,310x310,300x310");
        if scenario == 3 {
            deser(&mut config, "wipe_tower_rotation_angle", "30");
        }
    }
    if scenario == 4 {
        deser(&mut config, "enable_wrapping_detection", "1");
        deser(&mut config, "wrapping_exclude_area", "140x140,160x140,160x160,140x160");
    }
    if scenario == 5 {
        deser(&mut config, "enable_prime_tower", "1");
        deser(&mut config, "wall_filament", "2");
        deser(&mut config, "enable_wrapping_detection", "1");
        deser(&mut config, "wrapping_exclude_area", "190x190,250x190,250x250,190x250");
        deser(&mut config, "wipe_tower_x", "200");
        deser(&mut config, "wipe_tower_y", "200");
    }
    if scenario == 6 {
        deser(&mut config, "bed_exclude_area", "0x0,40x0,40x40,0x40,200x0,250x0,250x30,200x30");
        deser(&mut config, "print_sequence", "by object");
        deser(&mut config, "skirt_loops", "1");
        deser(&mut config, "skirt_distance", "6");
    }
    config
}

fn add_box<'a>(model: &'a mut Model, name: &str, b: (f64, f64, f64, f32, f32, f32)) -> &'a mut ModelObject {
    let ob = model.add_object_new();
    ob.name = name.into();
    ob.add_volume_mesh(mesh(box_its(b.0, b.1, b.2, b.3, b.4, b.5)), ModelVolumeType::ModelPart, true);
    ob
}

/// gen.cpp `build_model(scenario)`
fn build_model(scenario: i32, o: &mut Out) -> Model {
    let mut model = Model::new();
    o.base = model.id().id;
    if scenario == 0 || scenario == 6 {
        {
            let ob = add_box(&mut model, "a", (20.0, 20.0, 20.0, -10.0, -10.0, 0.0));
            ob.add_instance().set_offset(&Vec3d::new(100.0, 100.0, 0.0));
            ob.add_instance().set_offset(&Vec3d::new(125.0, 100.0, 0.0));
            let i2 = ob.add_instance();
            i2.set_offset(&Vec3d::new(180.0, 60.0, 0.0));
            i2.set_rotation(&Vec3d::new(0.0, 0.0, 0.5));
        }
        add_box(&mut model, "b", (10.0, 10.0, 60.0, -5.0, -5.0, 0.0)).add_instance().set_offset(&Vec3d::new(200.0, 200.0, 0.0));
        add_box(&mut model, "c", (8.0, 8.0, 5.0, -4.0, -4.0, 0.0)).add_instance().set_offset(&Vec3d::new(30.0, 30.0, 0.0));
        add_box(&mut model, "d", (10.0, 10.0, 50.0, -5.0, -5.0, 0.0)).add_instance().set_offset(&Vec3d::new(60.0, 205.0, 0.0));
    }
    if scenario == 1 {
        add_box(&mut model, "a", (20.0, 20.0, 10.0, -10.0, -10.0, 0.0)).add_instance().set_offset(&Vec3d::new(50.0, 50.0, 0.0));
        add_box(&mut model, "b", (20.0, 20.0, 10.0, -10.0, -10.0, 0.0)).add_instance().set_offset(&Vec3d::new(200.0, 200.0, 0.0));
    }
    if (2..=5).contains(&scenario) || scenario == 7 {
        {
            let ob = add_box(&mut model, "a", (20.0, 20.0, 20.0, -10.0, -10.0, 0.0));
            ob.add_volume_mesh(mesh(box_its(6.0, 6.0, 6.0, 12.0, -3.0, 0.0)), ModelVolumeType::ModelPart, true);
            ob.add_instance().set_offset(&Vec3d::new(100.0, 100.0, 0.0));
            let i1 = ob.add_instance();
            i1.set_offset(&Vec3d::new(60.0, 170.0, 0.0));
            i1.set_rotation(&Vec3d::new(0.0, 0.0, 0.3));
        }
        add_box(&mut model, "b", (10.0, 12.0, 8.0, -5.0, -6.0, 0.0)).add_instance().set_offset(&Vec3d::new(150.0, 150.0, 0.0));
    }
    model
}

/// gen.cpp `orders(model, base)`
fn orders(model: &Model, base: i64) -> String {
    let mut o = String::new();
    for mo in &model.objects {
        for mi in &mo.instances {
            o += &format!(" {}", mi.arrange_order as i64 - base);
        }
    }
    o
}

fn read_vectors() -> String {
    let path = format!("{}/tests/data/print_clearance/vectors.txt", env!("CARGO_MANIFEST_DIR"));
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"))
}

/// The two tests allocate ObjectIDs from one process-wide counter and write them relative to their models: run
/// one at a time.
static SERIAL: std::sync::Mutex<()> = std::sync::Mutex::new(());

#[test]
fn clearance_checks_match_the_pin() {
    let _serial = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
    use slicer::print::faithful::layered_print_cleareance_valid;
    let mut o = Out { lines: Vec::new(), base: 0 };
    for scenario in 0..8 {
        let model = build_model(scenario, &mut o);
        let config = base_config(scenario);
        let mut print = assemble(&model, &config);
        let sc = format!("c{}", scenario);
        let n = print.extruders(false).len();
        let depth = print.wipe_tower_data(n).depth;
        o.out(format!(
            "{} objects {} sequential {} short {} wipe_tower {} extruders {} depth {}",
            sc,
            print.m_objects.len(),
            hb(print.is_sequential_print()),
            hb(print.is_all_objects_are_short()),
            hb(print.has_wipe_tower()),
            n,
            hf(depth)
        ));
        let mut polys: Vec<slicer::geometry::Polygon> = Vec::new();
        let mut hp: Vec<(slicer::geometry::Polygon, f32)> = Vec::new();
        let e = Print::sequential_print_clearance_valid(&mut print, Some(&mut polys), Some(&mut hp));
        let mut hps = format!("{}", hp.len());
        for h in &hp {
            hps += &format!(" {} {}", poly_s(&h.0), hf(h.1));
        }
        let eid = o.id(e.object.unwrap_or(ObjectID { id: 0 }));
        o.out(format!("{} seq {} {} polys {} hp {}", sc, hb(!e.string.is_empty()), eid, polys_s(&polys), hps));
        o.out(format!("{} seq_msg {}", sc, msg(&e.string)));
        o.out(format!("{} seq_order{}", sc, orders(&print.m_model, 0)));
        let e2 = Print::sequential_print_clearance_valid(&mut print, None, None);
        o.out(format!("{} seq_null {}", sc, msg(&e2.string)));
        let mut warnings = Vec::new();
        let l = layered_print_cleareance_valid(&mut print, Some(&mut warnings));
        let mut ws = format!("{}", warnings.len());
        for w in &warnings {
            ws += &format!(" {} {}", hb(w.is_warning), msg(&w.string));
        }
        let lid = o.id(l.object.unwrap_or(ObjectID { id: 0 }));
        o.out(format!("{} lay {} {} {} warnings {}", sc, hb(!l.string.is_empty()), lid, hb(l.is_warning), ws));
        o.out(format!("{} lay_msg {}", sc, msg(&l.string)));
        o.out(format!("{} lay_order{}", sc, orders(&print.m_model, o.base as i64)));
        let l2 = layered_print_cleareance_valid(&mut print, None);
        o.out(format!("{} lay_null {}", sc, msg(&l2.string)));
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

/// gen.cpp `validate_line(tag, print)`: Print::validate with all three out-parameters.
fn validate_line(o: &mut Out, tag: &str, print: &mut Print) {
    let mut warnings = Vec::new();
    let mut polys: Vec<slicer::geometry::Polygon> = Vec::new();
    let mut hp: Vec<(slicer::geometry::Polygon, f32)> = Vec::new();
    let e = print.validate(Some(&mut warnings), Some(&mut polys), Some(&mut hp));
    let dash = |s: &str| if s.is_empty() { "-".to_string() } else { msg(s) };
    let mut ws = format!("{}", warnings.len());
    for w in &warnings {
        ws += &format!(" {} {} {} {} {}", hb(w.is_warning), hb(w.object.is_some()), dash(&w.opt_key), dash(&w.hypetext), msg(&w.string));
    }
    let mut ps = format!("{}", e.params.len());
    for p in &e.params {
        ps += &format!(" {}", p);
    }
    o.out(format!(
        "{} validate {} {} {} {} params {} polys {} hp {}",
        tag,
        e.r#type as i32,
        hb(e.object.is_some()),
        dash(&e.opt_key),
        hb(e.is_warning),
        ps,
        polys.len(),
        hp.len()
    ));
    o.out(format!("{} validate_msg {}", tag, msg(&e.string)));
    o.out(format!("{} validate_warnings {}", tag, ws));
}

/// R947: `Print::validate` (Print.cpp:1413-1844) on the eight scenarios and on variants of scenario 1 that reach
/// its other checks (by-object timelapse and clumping detection, layer heights, spiral vase, the bed type's
/// temperature, the precise outer wall warning, line widths, the prime tower's relative E and ooze prevention,
/// the precise Z warning): `gen validate` (tests/data/print_clearance/vectors_validate.txt).
#[test]
fn validate_matches_the_pin() {
    let _serial = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
    let mut o = Out { lines: Vec::new(), base: 0 };
    for scenario in 0..8 {
        let model = build_model(scenario, &mut o);
        let config = base_config(scenario);
        let mut print = assemble(&model, &config);
        validate_line(&mut o, &format!("c{}", scenario), &mut print);
    }
    let variants: Vec<Vec<(&str, &str)>> = vec![
        vec![("timelapse_type", "1")],
        vec![("enable_wrapping_detection", "1")],
        vec![("layer_height", "0.6")],
        vec![("print_sequence", "by layer"), ("spiral_mode", "1")],
        vec![("curr_bed_type", "Cool Plate"), ("cool_plate_temp", "0")],
        vec![("precise_outer_wall", "1"), ("wall_sequence", "outer wall/inner wall")],
        vec![("sparse_infill_line_width", "2")],
        vec![("enable_support", "1"), ("support_line_width", "0.1")],
        vec![("initial_layer_print_height", "0.5")],
        vec![("print_sequence", "by layer"), ("enable_prime_tower", "1"), ("wall_filament", "2"), ("use_relative_e_distances", "0")],
        vec![("print_sequence", "by layer"), ("enable_prime_tower", "1"), ("wall_filament", "2"), ("ooze_prevention", "1")],
        vec![("print_sequence", "by layer"), ("enable_prime_tower", "1"), ("wall_filament", "2"), ("precise_z_height", "1")],
    ];
    for (v, keys) in variants.iter().enumerate() {
        let model = build_model(1, &mut o);
        let mut config = base_config(1);
        for (k, val) in keys {
            deser(&mut config, k, val);
        }
        let mut print = assemble(&model, &config);
        validate_line(&mut o, &format!("v{}", v), &mut print);
    }
    let path = format!("{}/tests/data/print_clearance/vectors_validate.txt", env!("CARGO_MANIFEST_DIR"));
    let expected = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
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
