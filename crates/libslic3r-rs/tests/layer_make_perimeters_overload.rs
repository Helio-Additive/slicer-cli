//! R934: the arms of the pin's `LayerRegion::make_perimeters` overload (LayerRegion.cpp:131-206) and of
//! `generate_region_perimeters` (Layer.cpp:284-361) on one square region with the default configs. These
//! check the plumbing (which outputs are kept, discarded or appended, what reaches the PerimeterConfig);
//! they are not vectors of the pin.

use std::cell::RefCell;
use std::sync::Arc;

use slicer::adaptive_layer_merge::AdaptiveMergePatch;
use slicer::geometry::{ExPolygon, Point, Polygon};
use slicer::layer::{generate_region_perimeters, has_first_band_source, Layer, LayerRegion, MakePerimetersArgs};
use slicer::perimeter_generator::PerimeterConfig;
use slicer::surface::{Surface, SurfaceType};
use slicer::surface_collection::SurfaceCollection;

fn square(x0: i64, size: i64) -> ExPolygon {
    ExPolygon::new(Polygon::from_points(vec![
        Point::new(x0, x0),
        Point::new(x0 + size, x0),
        Point::new(x0 + size, x0 + size),
        Point::new(x0, x0 + size),
    ]))
}

fn region(layer_id: usize, slices: &SurfaceCollection) -> LayerRegion {
    let mut lm = LayerRegion::new(layer_id, 0);
    lm.set_config_hierarchy(
        Arc::new(slicer::print_region::PrintRegion::new(slicer::region_config::PrintRegionConfig::default())),
        Arc::new(slicer::print_config::PrintObjectConfig::default()),
        Arc::new(slicer::print_config::PrintConfig::default()),
    );
    lm.slices = slices.clone();
    lm
}

fn slices_of(ex: ExPolygon) -> SurfaceCollection {
    let mut s = SurfaceCollection::new();
    s.surfaces.push(Surface::new(SurfaceType::Internal, ex));
    s
}

#[test]
fn make_perimeters_overload_arms() {
    let slices = slices_of(square(0, 2_000_000));
    let h = 0.2;
    let z = 1.0;

    // plain() is make_perimeters_with.
    let mut a = region(4, &slices);
    a.make_perimeters_with(&slices, h, 4, z, &|_| {}).unwrap();
    let mut b = region(4, &slices);
    b.make_perimeters_overload(&slices, h, 4, z, &MakePerimetersArgs::plain(), &|_| {}).unwrap();
    assert!(!a.perimeters.entities.is_empty());
    assert_eq!(a.perimeters.entities.len(), b.perimeters.entities.len());
    assert_eq!(a.fill_surfaces.surfaces.len(), b.fill_surfaces.surfaces.len());
    assert_eq!(format!("{:?}", a.perimeters), format!("{:?}", b.perimeters));
    let walls = a.perimeters.entities.len();
    let fills = a.fill_surfaces.surfaces.len();

    // fill_only: the walls, thin fills and loop nodes go to discarded collections (LayerRegion.cpp:133-176);
    // the fill surfaces are appended.
    let mut c = region(4, &slices);
    c.make_perimeters_with(&slices, h, 4, z, &|_| {}).unwrap();
    let before = format!("{:?}", c.perimeters);
    let mut args = MakePerimetersArgs::plain();
    args.fill_only = true;
    c.make_perimeters_overload(&slices, h, 4, z, &args, &|_| {}).unwrap();
    assert_eq!(format!("{:?}", c.perimeters), before);
    assert_eq!(c.fill_surfaces.surfaces.len(), 2 * fills);

    // clear_outputs = false: the new walls after the kept ones.
    let mut d = region(4, &slices);
    d.make_perimeters_with(&slices, h, 4, z, &|_| {}).unwrap();
    let mut args = MakePerimetersArgs::plain();
    args.clear_outputs = false;
    d.make_perimeters_overload(&slices, h, 4, z, &args, &|_| {}).unwrap();
    assert_eq!(d.perimeters.entities.len(), 2 * walls);
    for (k, (range, _)) in d.perimeter_islands.iter().enumerate() {
        assert!(range.end <= d.perimeters.entities.len(), "island {k}");
    }

    // perimeter_layer_height, starts_at_object_base and the overrides reach the generator (:144-199).
    let seen: RefCell<Option<(f64, bool, bool, bool, Option<f64>)>> = RefCell::new(None);
    let mut e = region(4, &slices);
    e.lower_slices = Some(vec![square(0, 3_000_000)]);
    let mut args = MakePerimetersArgs::plain();
    args.perimeter_layer_height = 0.3;
    args.starts_at_object_base = true;
    args.upper_slices_override = Some(vec![square(100, 1_000_000)]);
    e.make_perimeters_overload(&slices, h, 4, z, &args, &|pc: &mut PerimeterConfig| {
        *seen.borrow_mut() = Some((
            pc.layer_height,
            pc.starts_at_object_base,
            pc.lower_slices.is_none(),
            pc.upper_slices.as_ref().is_some_and(|u| u.len() == 1 && u[0].contour.points[0] == Point::new(100, 100)),
            pc.overhang_flow.as_ref().map(|f| f.height()),
        ));
    })
    .unwrap();
    let (lh, base, no_lower, upper_override, overhang_h) = seen.borrow().clone().unwrap();
    assert_eq!(lh, 0.3);
    assert!(base);
    assert!(no_lower, "a band anchored at the object base takes no lower slices");
    assert!(upper_override);
    // overhang_flow = wall_flow(frPerimeter).with_flow_ratio(bridge_flow) at the merged height (:196-198)
    assert_eq!(overhang_h, Some(0.3f32 as f64)); // Flow holds float (Flow.hpp)
}

#[test]
fn generate_region_perimeters_arms() {
    let slices = slices_of(square(0, 2_000_000));
    let h = 0.2;
    // Object layers 0..3, one region each, with the square.
    let mut layers: Vec<Layer> = Vec::new();
    for i in 0..3 {
        let mut layer = Layer::new(i, 0, h, h * (i as f64 + 1.), h * (i as f64 + 0.5));
        layer.lslices = vec![square(0, 2_000_000)];
        layer.add_region(region(i, &slices));
        layers.push(layer);
    }
    let layers_ref = &layers;
    let get_layer = |i: usize| -> &Layer { &layers_ref[i] };

    // No patches: make_perimeters_with (Layer.cpp:287-290).
    let mut a = region(1, &slices);
    generate_region_perimeters(&mut a, &slices, 0, 3, &get_layer, h, 1, 0.4, &|_| {}).unwrap();
    let mut b = region(1, &slices);
    b.make_perimeters_with(&slices, h, 1, 0.4, &|_| {}).unwrap();
    assert_eq!(format!("{:?}", a.perimeters), format!("{:?}", b.perimeters));
    assert!(!has_first_band_source(&a));

    // Swallowed (no group top): fill_only, nothing extruded (Layer.cpp:292-302).
    let mut c = region(1, &slices);
    let mut swallowed = AdaptiveMergePatch::default();
    swallowed.bottom_idx = 0;
    swallowed.top_idx = 2;
    swallowed.set_area(vec![square(0, 2_000_000)]);
    c.adaptive_merge_patches.push(swallowed);
    generate_region_perimeters(&mut c, &slices, 0, 3, &get_layer, h, 1, 0.4, &|_| {}).unwrap();
    assert!(c.perimeters.entities.is_empty());
    assert!(!c.fill_surfaces.surfaces.is_empty());

    // Group top of a band from layer 0 to 2: walls at the merged height, from layer 0's slices, at the
    // object base (Layer.cpp:325-356); has_first_band_source (:233-238).
    let mut d = region(2, &SurfaceCollection::new());
    let mut top = AdaptiveMergePatch::default();
    top.bottom_idx = 0;
    top.top_idx = 2;
    top.group_top = true;
    top.merged_height = 0.3;
    top.set_area(vec![square(0, 2_000_000)]);
    d.adaptive_merge_patches.push(top);
    assert!(has_first_band_source(&d));
    let seen: RefCell<Vec<(f64, bool)>> = RefCell::new(Vec::new());
    generate_region_perimeters(&mut d, &SurfaceCollection::new(), 0, 3, &get_layer, h, 2, 0.6, &|pc: &mut PerimeterConfig| {
        seen.borrow_mut().push((pc.layer_height, pc.starts_at_object_base));
    })
    .unwrap();
    assert_eq!(seen.borrow().as_slice(), &[(0.3, true)]);
    assert!(!d.perimeters.entities.is_empty());

    // A group top whose area misses the source: nothing generated, outputs cleared (:357-360).
    let mut e = region(2, &slices);
    e.make_perimeters_with(&slices, h, 2, 0.6, &|_| {}).unwrap();
    let mut off = AdaptiveMergePatch::default();
    off.bottom_idx = 1;
    off.top_idx = 2;
    off.group_top = true;
    off.merged_height = 0.3;
    off.set_area(vec![square(50_000_000, 1_000_000)]);
    e.adaptive_merge_patches.push(off);
    generate_region_perimeters(&mut e, &slices, 0, 3, &get_layer, h, 2, 0.6, &|_| {}).unwrap();
    assert!(e.perimeters.entities.is_empty());
    assert!(e.thin_fills.entities.is_empty());
}

/// R935: the live `Layer::make_perimeters` (Layer.cpp:201-400) with the object's layers runs
/// `generate_region_perimeters` (:366) for its regions (gate LAYER_GENERATE_REGION_PERIMETERS) and runs a
/// region with no slices of its own when it is a first band source (:233-239, gate LAYER_FIRST_BAND_SOURCE).
/// Compared with the lambda called directly on a copy of the region; without the object's layers the
/// region's walls are the R934 `make_perimeters` call. Plumbing, not vectors of the pin.
#[test]
fn layer_make_perimeters_in_object_arms() {
    if std::env::var("LAYER_GENERATE_REGION_PERIMETERS").as_deref() == Ok("0")
        || std::env::var("LAYER_FIRST_BAND_SOURCE").as_deref() == Ok("0")
    {
        return;
    }
    let slices = slices_of(square(0, 2_000_000));
    let h = 0.2;
    let mut layers: Vec<Layer> = Vec::new();
    for i in 0..3 {
        let mut layer = Layer::new(i, 0, h, h * (i as f64 + 1.), h * (i as f64 + 0.5));
        layer.lslices = vec![square(0, 2_000_000)];
        // Layer 2 is the group top of a band from layer 0 and has no slices of its own; layer 1 is swallowed.
        let own = if i == 2 { SurfaceCollection::new() } else { slices.clone() };
        let mut lm = region(i, &own);
        let mut patch = AdaptiveMergePatch::default();
        patch.bottom_idx = 0;
        patch.top_idx = 2;
        patch.group_top = i == 2;
        patch.merged_height = 0.3;
        patch.set_area(vec![square(0, 2_000_000)]);
        if i >= 1 {
            lm.adaptive_merge_patches.push(patch);
        }
        layer.add_region(lm);
        layers.push(layer);
    }
    let snapshot = layers.clone();
    let get_layer = |i: usize| -> &Layer { &snapshot[i] };

    // Group top with no slices: Layer::make_perimeters runs it (first band source) through the lambda.
    let mut direct = snapshot[2].regions()[0].clone();
    direct.fill_surfaces.surfaces.clear();
    direct.fill_no_overlap_expolygons.clear();
    generate_region_perimeters(&mut direct, &SurfaceCollection::new(), 0, 3, &get_layer, h, 2, 0.6, &|_| {}).unwrap();
    let mut top = layers[2].clone();
    top.make_perimeters_in_object(Some(&snapshot)).unwrap();
    assert!(!top.regions()[0].perimeters.entities.is_empty());
    assert_eq!(format!("{:?}", top.regions()[0].perimeters), format!("{:?}", direct.perimeters));
    assert_eq!(format!("{:?}", top.regions()[0].fill_surfaces), format!("{:?}", direct.fill_surfaces));

    // Without the object's layers: the region runs, but the R934 call on its own (empty) slices.
    let mut alone = layers[2].clone();
    alone.make_perimeters_in_object(None).unwrap();
    assert!(alone.regions()[0].perimeters.entities.is_empty());

    // Swallowed layer: fill_only, nothing extruded, fill surfaces kept (Layer.cpp:292-302).
    let mut swallowed = layers[1].clone();
    swallowed.make_perimeters_in_object(Some(&snapshot)).unwrap();
    assert!(swallowed.regions()[0].perimeters.entities.is_empty());
    assert!(!swallowed.regions()[0].fill_surfaces.surfaces.is_empty());
    // The same layer through make_perimeters(None) extrudes its walls (no object, R934 call).
    let mut plain = layers[1].clone();
    plain.make_perimeters(None).unwrap();
    assert!(!plain.regions()[0].perimeters.entities.is_empty());

    // Layer 0 has no patches: the lambda is make_perimeters_with, equal to make_perimeters(None).
    let mut l0 = layers[0].clone();
    l0.make_perimeters_in_object(Some(&snapshot)).unwrap();
    let mut l0_plain = layers[0].clone();
    l0_plain.make_perimeters(None).unwrap();
    assert_eq!(format!("{:?}", l0.regions()[0].perimeters), format!("{:?}", l0_plain.regions()[0].perimeters));
}
