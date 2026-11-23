//! The live ironing step against `Layer::make_ironing` alone.
//!
//! PrintObject.cpp:1251-1265: `PrintObject::ironing` runs `Layer::make_ironing` (Fill.cpp:983-1250) on every
//! layer. A 10 mm cube with `ironing_type = top` goes through `Print::process` (the live path); the ironing
//! entities the step appended to the regions' fills are then compared with what `Layer::make_ironing` gives
//! when it runs alone on a copy of each layer with those entities taken out. With FILL_IRONING_LIVE_PIN on
//! (the default) they must be equal; with FILL_IRONING_LIVE_PIN=0 (the earlier generator, gcode/ironing.rs)
//! the test reports how many layers differ.
use slicer::extrusion_entity::{ExtrusionEntityType, ExtrusionRole};
use slicer::print::{Print, PrintObject};
use slicer::print_config::{IroningType, PrintObjectConfig};

fn role_of(e: &ExtrusionEntityType) -> ExtrusionRole {
    match e {
        ExtrusionEntityType::Path(p) => p.role,
        ExtrusionEntityType::MultiPath(m) => m.role(),
        ExtrusionEntityType::Loop(l) => l.paths.first().map(|p| p.role).unwrap_or(ExtrusionRole::None),
        ExtrusionEntityType::Collection(c) => c.entities.first().map(role_of).unwrap_or(ExtrusionRole::Mixed),
    }
}

/// Every region's ironing entities of one layer, as text (paths, points, widths, heights, mm3_per_mm).
fn ironing_of(layer: &slicer::layer::Layer) -> Vec<String> {
    let mut out = Vec::new();
    for (r, layerm) in layer.regions().iter().enumerate() {
        for e in &layerm.fills.entities {
            if role_of(e) == ExtrusionRole::Ironing {
                out.push(format!("region {r}: {e:?}"));
            }
        }
    }
    out
}

fn sliced_ironed_cube() -> Print {
    let mut region = slicer::region_config::PrintRegionConfig::default();
    assert!(region.set_deserialize("ironing_type", "top"));
    let mut print = Print::new();
    print.set_default_region_config(region);
    let mut object_config = PrintObjectConfig::default();
    // The earlier generator (FILL_IRONING_LIVE_PIN=0) reads the object's copy of the option.
    object_config.ironing_type = IroningType::TopSurfaces;
    let obj = PrintObject::with_config(slicer::TriangleMesh::cube(10.0), object_config);
    print.add_object(obj);
    print.process(None, false).expect("the cube slices");
    print
}

#[test]
fn live_ironing_equals_layer_make_ironing() {
    let print = sliced_ironed_cube();
    let layers = print.objects()[0].layers();
    assert!(layers.len() > 10, "the cube has {} layers", layers.len());
    let mut ironed_layers = 0usize;
    let mut differ = 0usize;
    let mut first_differ = None;
    for (idx, layer) in layers.iter().enumerate() {
        let live = ironing_of(layer);
        // The same layer without the ironing entities, then Layer::make_ironing alone.
        let mut alone = layer.clone();
        for layerm in alone.regions_mut() {
            layerm.fills.entities.retain(|e| role_of(e) != ExtrusionRole::Ironing);
        }
        alone.make_ironing().expect("make_ironing");
        let pin = ironing_of(&alone);
        if !pin.is_empty() {
            ironed_layers += 1;
        }
        if live != pin {
            differ += 1;
            if first_differ.is_none() {
                first_differ = Some((idx, live.len(), pin.len()));
            }
        }
    }
    eprintln!(
        "live ironing: {} layers, {} ironed by Layer::make_ironing, {} differ from it (first: {:?})",
        layers.len(),
        ironed_layers,
        differ,
        first_differ
    );
    // ironing_type = top irons the top surface of the cube only (Fill.cpp:1060-1067, 1157-1160).
    assert_eq!(ironed_layers, 1, "Layer::make_ironing irons the cube's top layer");
    if slicer::faithful_gate("FILL_IRONING_LIVE_PIN") {
        assert_eq!(differ, 0, "the live step equals Layer::make_ironing on every layer");
    }
}
