//! Native parity of the Support/SupportParameters.hpp port (`slicer::support::support_parameters`)
//! against vectors made by the C++ at the pin (tests/data/support_parameters/gen.cpp).
//!
//! The generator builds, for nine configurations, a Print of one box with `Print::apply`, gives the
//! object six layers (their heights) and constructs `SupportParameters(object)`; it prints every member,
//! the interface angles, the flows for a nozzle and the free functions of the header. The lines of the
//! vectors that start with "in" are the inputs, which this test reads back; every other line it has to
//! reproduce, bit for bit.

mod support_shared;

use slicer::config::{ConfigOptionInts, ConfigOptionStrings};
use slicer::flow::Flow;
use slicer::model::*;
use slicer::print_config::config_def::DynamicPrintConfig;
use slicer::print_config::enums::{SupportMaterialInterfacePattern, SupportMaterialStyle, SupportType};
use slicer::support::support_parameters::*;
use support_shared::*;

fn flow_s(f: &Flow) -> String {
    format!("{} {} {} {} {}", hf(f.width() as f32), hf(f.height() as f32), hf(f.spacing() as f32), hf(f.nozzle_diameter() as f32), hb(f.is_bridge()))
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

    let mut layer_id = 0usize;
    while let Some(line) = r.input(&format!("in {s} layer")) {
        let mut t = Tok::new(&line);
        let (print_z, height, slice_z) = (t.d(), t.d(), t.d());
        obj.m_layers.push(slicer::layer::Layer::new(layer_id, obj.m_model_object, height, print_z, slice_z));
        layer_id += 1;
    }
    let line = r.input(&format!("in {s} variable")).expect("variable");
    obj.has_variable_layer_heights = Tok::new(&line).b();

    let sp = obj.slicing_parameters().clone();
    r.out(format!(
        "{s} slicing {} {} {} {} {}",
        hd(sp.layer_height),
        hd(sp.first_print_layer_height),
        sp.base_raft_layers,
        sp.interface_raft_layers,
        hb(sp.soluble_interface)
    ));

    let p = SupportParameters::new(&obj, &print).expect("SupportParameters");
    r.out(format!(
        "{s} flags {} {} {} {} {} {} {} {} {} {} {} {} {}",
        hb(p.soluble_interface),
        hb(p.soluble_interface_non_soluble_base),
        hb(p.has_top_contacts),
        hb(p.has_bottom_contacts),
        p.num_top_interface_layers,
        p.num_bottom_interface_layers,
        p.num_top_base_interface_layers,
        p.num_bottom_base_interface_layers,
        hb(p.has_contacts()),
        hb(p.has_interfaces()),
        hb(p.has_base_interfaces()),
        p.num_top_interface_layers_only(),
        // size_t arithmetic of the C++ (it wraps where the base layers outnumber the interface layers).
        p.num_bottom_interface_layers_only() as i64
    ));
    r.out(format!(
        "{s} flows {} {} {} {} {}",
        flow_s(&p.first_layer_flow),
        flow_s(&p.support_material_flow),
        flow_s(&p.support_material_interface_flow),
        flow_s(&p.support_material_bottom_interface_flow),
        flow_s(&p.raft_interface_flow)
    ));
    r.out(format!(
        "{s} widths {} {} {} {} {} {} {} {} {}",
        hd(p.support_base_width),
        hd(p.support_first_layer_width),
        hf(p.preset_nozzle_diameter),
        hb(p.multi_nozzle_diameter_mode),
        hd(p.support_extrusion_width),
        hb(p.can_merge_support_regions),
        hd(p.support_layer_height_min),
        hd(p.gap_xy),
        hd(p.gap_xy_first_layer)
    ));
    r.out(format!(
        "{s} angles {} {} {} {} {} {} {} {} {} {}",
        hf(p.base_angle),
        hf(p.interface_angle),
        hd(p.interface_spacing),
        hd(p.support_expansion),
        hd(p.interface_density),
        hd(p.raft_interface_density),
        hd(p.support_spacing),
        hd(p.support_density),
        p.support_style,
        p.support_base_pattern
    ));
    r.out(format!(
        "{s} patterns {} {} {} {} {} {} {} {} {} {} {}",
        p.base_fill_pattern as i32,
        p.interface_fill_pattern as i32,
        p.raft_interface_fill_pattern as i32,
        p.contact_fill_pattern as i32,
        hb(p.with_sheath),
        hd(p.tree_branch_diameter_double_wall_area_scaled),
        hf(p.raft_angle_1st_layer),
        hf(p.raft_angle_base),
        hf(p.raft_angle_interface),
        hb(p.independent_layer_height),
        hd(p.thresh_big_overhang)
    ));
    r.out(format!(
        "{s} ironing {} {} {} {} {} {} {}",
        hb(p.enable_support_ironing),
        p.ironing_pattern as i32,
        hd(p.ironing_line_spacing),
        hd(p.ironing_flow_percent),
        hd(p.ironing_speed),
        hd(p.ironing_angle),
        hd(p.ironing_inset)
    ));
    for interface_id in [0usize, 1, 2, 7] {
        let mut line = format!("{s} interface_angle {} {}", interface_id, hf(p.raft_interface_angle(interface_id)));
        for pattern in 0..5 {
            line += &format!(" {}", hf(p.support_interface_angle(interface_id, SupportMaterialInterfacePattern::from_i32(pattern))));
        }
        r.out(line);
    }
    for nozzle in [0.2, 0.4, 0.6, 0.8] {
        for lh in [0.08f32, 0.2f32, 0.28f32] {
            r.out(format!(
                "{s} flow_for_nozzle {} {} {} {} {}",
                hd(nozzle),
                hf(lh),
                flow_s(&p.support_material_flow_for_nozzle(nozzle, lh).expect("flow")),
                flow_s(&p.support_material_interface_flow_for_nozzle(nozzle, lh).expect("flow")),
                flow_s(&p.support_material_first_layer_flow_for_nozzle(nozzle, lh).expect("flow"))
            ));
        }
    }
    r.out(format!("{s} xy_distance {}", hd(resolved_support_object_xy_distance(print.config(), obj.config()))));
    let full = print.full_print_config();
    r.out(format!(
        "{s} flag {} {} {} {}",
        hb(config_flag_or_false(full, "enable_support")),
        hb(config_flag_or_false(full, "thick_bridges")),
        hb(config_flag_or_false(full, "no_such_key")),
        hb(config_flag_or_false(full, "layer_height"))
    ));
}

#[test]
fn support_parameters_native_parity() {
    let mut r = Run::load("support_parameters");
    for style in 0..=6 {
        for type_ in 0..=3 {
            let mut line = format!("resolve_support_style {style} {type_}");
            for bits in 0..32 {
                line += &format!(
                    " {}",
                    resolve_support_style(
                        SupportMaterialStyle::from_i32(style),
                        SupportType::from_i32(type_),
                        bits & 1 != 0,
                        bits & 2 != 0,
                        bits & 4 != 0,
                        bits & 8 != 0,
                        bits & 16 != 0
                    ) as i32
                );
            }
            r.out(line);
        }
    }
    let mut scenario = 0;
    while r.expected[r.pos..].iter().any(|l| l.starts_with(&format!("in s{scenario} model"))) {
        run_scenario(&mut r, scenario);
        scenario += 1;
    }
    assert_eq!(scenario, 9, "scenarios");
    let n = r.expected.iter().filter(|l| !l.starts_with("in ")).count();
    let bad = r.finish();
    eprintln!("support_parameters: {} result lines over {} scenarios, {} differ", n, scenario, bad);
    assert_eq!(bad, 0);
}
