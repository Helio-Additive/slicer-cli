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

#[test]
fn organic_tree_support_through_the_live_bridge() {
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
