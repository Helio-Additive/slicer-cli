//! R934 TOOL_ORDERING_PERIMETER_ISLANDS: the view of `layerm->perimeters` ToolOrdering takes from a
//! LayerRegion. The pin's entries are island collections (PerimeterGenerator.cpp:1340-1342,
//! 1837-1838), cast with `dynamic_cast<const ExtrusionEntityCollection&>` by collect_extruders
//! (ToolOrdering.cpp:943-951) and mark_wiping_extrusions (:3985-3987); the collection's role
//! (ExtrusionEntityCollection::role: erMixed over walls and gap fill) and total volume are the island's.
//! This crate keeps the walls flat with `LayerRegion::perimeter_islands` beside them.

use slicer::extrusion_entity::{
    ExtrusionEntityCollection, ExtrusionEntityType, ExtrusionLoop, ExtrusionLoopRole, ExtrusionPath, ExtrusionRole,
};
use slicer::gcode::tool_ordering::ToolOrderingLayerRegion;
use slicer::geometry::{Point, Polyline};
use slicer::layer::LayerRegion;

fn path(role: ExtrusionRole, pts: &[(i64, i64)], mm3_per_mm: f64) -> ExtrusionPath {
    let mut p = ExtrusionPath::new(role);
    p.polyline = Polyline::from_points(pts.iter().map(|&(x, y)| Point::new(x, y)).collect());
    p.mm3_per_mm = mm3_per_mm;
    p.width = 0.45;
    p.height = 0.2;
    p
}

fn square_loop(role: ExtrusionRole, x0: i64, size: i64, mm3_per_mm: f64) -> ExtrusionEntityType {
    let pts = [(x0, 0), (x0 + size, 0), (x0 + size, size), (x0, size), (x0, 0)];
    ExtrusionEntityType::Loop(ExtrusionLoop::new(vec![path(role, &pts, mm3_per_mm)], ExtrusionLoopRole::DEFAULT))
}

fn entity_volume(e: &ExtrusionEntityType) -> f64 {
    match e {
        ExtrusionEntityType::Path(p) => p.total_volume(),
        ExtrusionEntityType::MultiPath(m) => m.total_volume(),
        ExtrusionEntityType::Loop(l) => l.total_volume(),
        ExtrusionEntityType::Collection(c) => c.total_volume(),
    }
}

#[test]
fn tool_ordering_perimeter_islands_view() {
    let gate_on = std::env::var("TOOL_ORDERING_PERIMETER_ISLANDS").map_or(true, |v| v != "0");
    let mut layerm = LayerRegion::new(3, 0);
    // Island 0: an external and an inner wall and a gap fill; island 1: one external wall; then a wall
    // that belongs to no recorded island (it stays at the top level, as perimeter_island_collections keeps it).
    let flat = vec![
        square_loop(ExtrusionRole::ExternalPerimeter, 0, 2_000_000, 0.05),
        square_loop(ExtrusionRole::Perimeter, 450_000, 1_100_000, 0.05),
        ExtrusionEntityType::Path(path(ExtrusionRole::GapFill, &[(900_000, 900_000), (1_100_000, 1_100_000)], 0.02)),
        square_loop(ExtrusionRole::ExternalPerimeter, 5_000_000, 1_000_000, 0.05),
        square_loop(ExtrusionRole::Perimeter, 9_000_000, 1_000_000, 0.05),
    ];
    layerm.perimeters.entities = flat.clone();
    let mut shell0 = ExtrusionEntityCollection::new();
    shell0.no_sort = true;
    shell0.loop_node_range = (2, 4);
    let shell1 = ExtrusionEntityCollection::new();
    layerm.perimeter_islands.push((0..3, shell0));
    layerm.perimeter_islands.push((3..4, shell1));

    let view = ToolOrderingLayerRegion::perimeters_from_layer_region(&layerm);
    if gate_on {
        assert_eq!(view.entities.len(), 3);
        let c0 = match &view.entities[0] {
            ExtrusionEntityType::Collection(c) => c,
            _ => panic!("island 0 is not a collection"),
        };
        let c1 = match &view.entities[1] {
            ExtrusionEntityType::Collection(c) => c,
            _ => panic!("island 1 is not a collection"),
        };
        assert_eq!(c0.entities.len(), 3);
        assert_eq!(c1.entities.len(), 1);
        assert!(c0.no_sort);
        assert_eq!(c0.loop_node_range, (2, 4));
        // ExtrusionEntityCollection::role(): erMixed over walls and gap fill, the common role otherwise.
        assert_eq!(c0.role(), ExtrusionRole::Mixed);
        assert_eq!(c1.role(), ExtrusionRole::ExternalPerimeter);
        assert!(matches!(view.entities[2], ExtrusionEntityType::Loop(_)));
        let v0: f64 = flat[0..3].iter().map(entity_volume).sum();
        assert_eq!(c0.total_volume(), v0);
        let wiping = ToolOrderingLayerRegion::wiping_perimeters(&view, 7u64 << 32, 1);
        assert_eq!(wiping.len(), 3);
        assert_eq!(wiping[0].role, ExtrusionRole::Mixed);
        assert_eq!(wiping[0].total_volume, v0);
        assert_eq!(wiping[1].id, (7u64 << 32) | 1);
        assert_eq!(wiping[2].extruder, 1);
    } else {
        // The earlier arm: the flat list as stored.
        assert_eq!(view.entities.len(), 5);
        assert!(view.entities.iter().all(|e| !matches!(e, ExtrusionEntityType::Collection(_))));
    }
    // Nothing recorded: the view is `perimeters` as stored (either arm).
    layerm.perimeter_islands.clear();
    let plain = ToolOrderingLayerRegion::perimeters_from_layer_region(&layerm);
    assert_eq!(plain.entities.len(), 5);
}
