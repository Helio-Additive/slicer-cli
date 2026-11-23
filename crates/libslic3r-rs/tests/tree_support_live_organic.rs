//! The live caller of the organic tree support (gate `TREE_SUPPORT_ORGANIC_PIN`, R932): a T-shaped object (a pillar
//! with a slab overhanging it on both sides) with `support_type tree(auto)` and `support_style tree_organic`, made
//! into a faithful Print by the builder the gated path uses (`assemble_faithful_print`, PrintApply.cpp), sliced by
//! the faithful `PrintObject::slice()` (PrintObjectSlice.cpp), and its layers handed to the function the gated path
//! calls (`generate_live_tree_support`: TreeSupport::generate -> generate_tree_support_3D with the engine's fillers).
//! The support layers it hands back must exist below the slab and carry toolpaths.

mod support_shared;

use slicer::model::*;
use slicer::print_config::config_def::DynamicPrintConfig;
use slicer::print_object_slice::NoSliceVolumesCores;
use slicer::support::tree_support_3d::{assemble_faithful_print, generate_live_tree_support};
use support_shared::*;

/// The gate is read from a snapshot of the environment taken at the first gate read of the process
/// (`slicer::opt_in_gate`), so every test of this binary sets it before anything else runs.
fn gate_on() {
    std::env::set_var("TREE_SUPPORT_ORGANIC_PIN", "1");
}

#[test]
fn organic_tree_support_through_the_live_bridge() {
    gate_on();
    let mut model = Model::new();
    {
        let o = model.add_object_new();
        o.name = "tee".into();
        // The pillar, 6 x 6 x 10 mm, and the slab on it, 24 x 6 x 2 mm, overhanging 9 mm on both sides.
        o.add_volume_mesh(ModelMesh::from_its(box_its(6.0, 6.0, 10.0, -3.0, -3.0, 0.0)), ModelVolumeType::ModelPart, true);
        o.add_volume_mesh(ModelMesh::from_its(box_its(24.0, 6.0, 2.0, -12.0, -3.0, 10.0)), ModelVolumeType::ModelPart, true);
        o.add_instance().set_offset(&slicer::geometry::geometry::Vec3d::new(100.0, 100.0, 0.0));
    }
    let mut config = DynamicPrintConfig::full_print_config();
    for (key, value) in [
        ("enable_support", "1"),
        ("support_type", "tree(auto)"),
        ("support_style", "tree_organic"),
        ("layer_height", "0.2"),
        ("initial_layer_print_height", "0.2"),
        ("printable_area", "0x0,256x0,256x256,0x256"),
    ] {
        deser(&mut config, key, value);
    }
    let mut print = assemble_faithful_print(&model, &config);
    assert_eq!(print.m_objects.len(), 1);
    let mut obj = print.m_objects.remove(0);
    obj.slice(&print, &mut NoSliceVolumesCores).expect("PrintObject::slice");
    print.m_objects.insert(0, obj);
    let layers = print.m_objects[0].m_layers.clone();
    assert!(layers.len() > 50, "the object has {} layers", layers.len());
    let model_object_id = print.m_objects[0].m_model_object;

    let support = generate_live_tree_support(&mut print, model_object_id, &layers)
        .expect("the print holds an object of the ModelObject")
        .expect("TreeSupport::generate");
    let with_fills = support.iter().filter(|l| l.support_fills.as_ref().map_or(false, |f| !f.entities.is_empty())).count();
    eprintln!("support layers {}, with toolpaths {}", support.len(), with_fills);
    assert!(!support.is_empty(), "no support layers");
    assert!(with_fills > 0, "no support layer has toolpaths");
    // The supports stand below the slab (z 10 mm).
    assert!(support.iter().any(|l| l.print_z < 10.0 && !l.lslices.is_empty()), "no support islands below the slab");
}

/// A binary STL of the T-shape: the pillar, 6 x 6 x 10 mm, and the slab on it, 24 x 6 x 2 mm (two closed boxes).
fn tee_stl() -> Vec<u8> {
    fn boxed(x0: f32, y0: f32, z0: f32, x1: f32, y1: f32, z1: f32) -> Vec<[[f32; 3]; 3]> {
        let v = [[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0], [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]];
        let f = [[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5], [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]];
        f.iter().map(|t| [v[t[0]], v[t[1]], v[t[2]]]).collect()
    }
    let mut tris = boxed(-3.0, -3.0, 0.0, 3.0, 3.0, 10.0);
    tris.extend(boxed(-12.0, -3.0, 10.0, 12.0, 3.0, 12.0));
    let mut out = vec![b' '; 80];
    out.extend_from_slice(&(tris.len() as u32).to_le_bytes());
    for t in &tris {
        out.extend_from_slice(&[0u8; 12]);
        for p in t {
            for c in p {
                out.extend_from_slice(&c.to_le_bytes());
            }
        }
        out.extend_from_slice(&[0u8; 2]);
    }
    out
}

/// The live slicer with the gate on (R933): app_slice.rs builds the Model of the placed STL
/// (`live_model_from_desktop_placement`) and hands it to the live Print, whose support phase (print.rs) then runs the
/// pin's tree support step (`PrintObject::generate_support_material_tree_pin`) on the faithful Print made from it.
/// The live object must come out with support layers that carry toolpaths, below the slab, in the live layers'
/// frame (centred on the object: the slab spans x in [-12, 12] mm, y in [-3, 3] mm there).
#[test]
fn organic_tree_support_through_the_live_slicer() {
    live_slicer_tee("default", &[]);
}

/// As above with `support_base_pattern honeycomb`. Found in R933: the organic path does not ask the engine factory
/// for FillHoneycomb on this model (`ENGINE_SUPPORT_FILL_HONEYCOMB_CALLS` stays 0; the support layers equal the
/// default's), so this runs the setting through the live slicer only; `EngineSupportFillHoneycomb` is not exercised.
#[test]
fn organic_tree_support_through_the_live_slicer_honeycomb() {
    live_slicer_tee("honeycomb", &[("support_base_pattern", "honeycomb")]);
}

fn live_slicer_tee(tag: &str, extra: &[(&str, &str)]) {
    gate_on();
    let manifest = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let tmp = std::path::PathBuf::from(env!("CARGO_TARGET_TMPDIR"));
    let stl = tmp.join(format!("tree_support_live_tee_{tag}.stl"));
    std::fs::write(&stl, tee_stl()).unwrap();
    let mut settings: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(manifest.join("tests/data/cube_arachne_settings.json")).unwrap()).unwrap();
    for (key, value) in [("enable_support", "1"), ("support_type", "tree(auto)"), ("support_style", "tree_organic")].iter().chain(extra.iter()) {
        settings[*key] = serde_json::Value::String(value.to_string());
    }
    let settings_path = tmp.join(format!("tree_support_live_tee_{tag}.json"));
    std::fs::write(&settings_path, serde_json::to_string(&settings).unwrap()).unwrap();

    let print = slicer::app_slice::slice_stl_to_print(&stl, &settings_path).expect("slice");
    let obj = &print.objects()[0];
    let with_fills = obj.support_layers.iter().filter(|l| l.support_fills.as_ref().map_or(false, |f| !f.entities.is_empty())).count();
    eprintln!(
        "{tag}: FillHoneycomb fills so far in this process {}",
        slicer::support::tree_support_3d::ENGINE_SUPPORT_FILL_HONEYCOMB_CALLS.load(std::sync::atomic::Ordering::Relaxed)
    );
    let entities: usize = obj.support_layers.iter().map(|l| l.support_fills.as_ref().map_or(0, |f| f.entities.len())).sum();
    eprintln!("{tag}: object layers {}, support layers {}, with toolpaths {}, top-level entities {}", obj.layers().len(), obj.support_layers.len(), with_fills, entities);
    assert!(with_fills > 0, "no support layer has toolpaths");
    // The organic branches thicken towards the bed, so the islands reach past the slab's shadow by a few mm; a
    // frame other than the live layers' (the bed's, the faithful object's without m_center_offset) would put them
    // a hundred millimetres or more away.
    let mut below = 0;
    let (mut max_x, mut max_y) = (0.0f64, 0.0f64);
    for l in &obj.support_layers {
        for ex in &l.lslices {
            for p in &ex.contour.points {
                let (x, y) = (p.x() as f64 * 1e-5, p.y() as f64 * 1e-5);
                max_x = max_x.max(x.abs());
                max_y = max_y.max(y.abs());
                assert!(x.abs() < 25.0 && y.abs() < 15.0, "support island point ({x}, {y}) mm at z {} far from the object", l.print_z);
            }
        }
        if l.print_z < 10.0 && !l.lslices.is_empty() {
            below += 1;
        }
    }
    eprintln!("{tag}: support islands within |x| <= {max_x:.3} mm, |y| <= {max_y:.3} mm of the object's centre");
    assert!(below > 0, "no support islands below the slab");
}
