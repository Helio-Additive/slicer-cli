//! Integration test for the Tier-1 MMU-segmentation orchestrator
//! `multi_material_segmentation_by_painting_tier1`
//! (MultiMaterialSegmentation.cpp:2095-2409).
//!
//! Synthetic case: a 40 mm square is the (merged) region slice for 3 layers, and a single
//! painted sub-mesh for extruder slot 1 is a vertical quad (2 triangles) that coincides with
//! the square's bottom side and spans z 0..1 mm, so every layer's slice plane (z = 0.2/0.4/0.6)
//! crosses it and paints that one side. With three unpainted sides the layer is NOT a single
//! colour, so the full graph/colorize/extract path runs.
//!
//! NOTE on the output shape: the orchestrator returns `merge_segmented_layers`' result,
//! `[layer][num_extruders]`, indexed by 0-based extruder (slot `j` == painted filament slot
//! `j + 1`); the unpainted/default colour is dropped exactly as C++ does. So for
//! `num_extruders == 2` each layer has 2 slots, and the painted slot 1 lands at index 0. This
//! matches the C++ consumer `PrintObjectSlice.cpp:877-879` (`segmentation[l][0..num_extruders]`).

use slicer::geometry::{ExPolygon, Point, Polygon};
use slicer::multi_material_segmentation::multi_material_segmentation_by_painting_tier1;
use slicer::normal_utils::{indexed_triangle_set, Vec3crd, Vec3f};

#[test]
fn by_painting_one_side_segments_a_subregion() {
    // 40 mm square centred at the origin, CCW, scaled coords (100000 units / mm).
    let h = 2_000_000; // 20 mm scaled
    let p0 = Point::new(-h, -h);
    let p1 = Point::new(h, -h);
    let p2 = Point::new(h, h);
    let p3 = Point::new(-h, h);
    let square = ExPolygon::new(Polygon::from_points(vec![p0, p1, p2, p3]));

    // Same square as the merged region slices for 3 layers.
    let layer_slices = vec![vec![square.clone()], vec![square.clone()], vec![square.clone()]];
    let layer_slice_zs = vec![0.2_f64, 0.4, 0.6];

    // Painted sub-mesh for extruder slot 1: a vertical quad on the bottom side (y = -20 mm),
    // x from -20..20 mm, z 0..1 mm — 2 triangles.
    let s = 20.0_f32;
    let quad = indexed_triangle_set {
        vertices: vec![
            Vec3f::new(-s, -s, 0.0),
            Vec3f::new(s, -s, 0.0),
            Vec3f::new(s, -s, 1.0),
            Vec3f::new(-s, -s, 1.0),
        ],
        indices: vec![Vec3crd::new(0, 1, 2), Vec3crd::new(0, 2, 3)],
    };
    let painted = vec![(1u8, quad)];

    let num_extruders = 2;
    let result = multi_material_segmentation_by_painting_tier1(
        &layer_slices,
        &layer_slice_zs,
        &painted,
        num_extruders,
        0.0, // segmented_max_width — no cut
        0.0, // segmented_interlocking_depth — no cut
    );

    // 3 layers, num_extruders (0-based extruder) slots each.
    assert_eq!(result.len(), 3, "expected 3 layers, got {}", result.len());
    for (l, layer) in result.iter().enumerate() {
        assert_eq!(
            layer.len(),
            num_extruders,
            "layer {} should have num_extruders ({}) colour slots, got {}",
            l,
            num_extruders,
            layer.len()
        );
    }

    // Painted filament slot 1 -> merged index 0.
    let full_area = square.area();
    let painted_area: Vec<f64> = result
        .iter()
        .map(|layer| layer[0].iter().map(|e| e.area()).sum::<f64>())
        .collect();

    // At least one layer segmented a non-empty region for the painted colour.
    assert!(
        painted_area.iter().any(|&a| a > 0.0),
        "expected at least one layer with a non-empty painted (slot 1) region; areas = {:?}",
        painted_area
    );

    // The painted colour segmented a sub-region, not the whole square.
    for (l, &a) in painted_area.iter().enumerate() {
        assert!(
            a < full_area,
            "layer {}: painted area {} must be < full square area {}",
            l,
            a,
            full_area
        );
    }
}
