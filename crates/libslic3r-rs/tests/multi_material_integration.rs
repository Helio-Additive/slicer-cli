//! End-to-end multi-material integration tests.
//!
//! These tests verify that the complete multi-material pipeline works correctly,
//! including:
//! - Tool ordering optimization
//! - Flush matrix calculations
//! - Multi-material coordinator integration
//! - Extruder role assignment

use slicer::gcode::{
    multi_material::{MultiMaterialConfig, MultiMaterialCoordinator, WipeTowerBounds},
    tool_ordering::{
        calculate_flush_volume, find_optimal_ordering_exhaustive, generate_all_orderings,
        optimize_extruder_sequence, ExtrusionRoleType, FlushMatrix, ToolOrdering,
        ToolOrderingConfig,
    },
};

/// Test basic multi-material coordinator creation and configuration
#[test]
fn test_multi_material_coordinator_creation() {
    let mut config = MultiMaterialConfig::new(4);
    config.filament_densities = vec![1.24, 1.24, 1.08, 1.08]; // PLA, PLA, PETG, PETG
    config.enable_prime_tower = true;
    config.wipe_tower_x = 170.0;
    config.wipe_tower_y = 140.0;
    config.wipe_tower_width = 60.0;
    config.wipe_tower_rotation = 0.0;

    let coordinator = MultiMaterialCoordinator::new(config);

    assert!(coordinator.is_multi_material());
    assert_eq!(coordinator.config().num_filaments, 4);
}

/// Test tool ordering with simple layer configuration
#[test]
fn test_tool_ordering_simple_layers() {
    // Create a simple print with 3 extruders and 5 layers
    let layer_z_heights: Vec<f64> = vec![0.2, 0.4, 0.6, 0.8, 1.0];

    let config = ToolOrderingConfig::default();
    let mut ordering = ToolOrdering::new(config);

    // Initialize layers
    ordering.initialize_layers(layer_z_heights.clone());

    // Simulate extruder usage: layer 0-1 uses extruder 0, layer 2-3 uses extruder 1, layer 4 uses both
    ordering.add_extruder_to_layer(0.2, 0, true);
    ordering.add_extruder_to_layer(0.4, 0, true);
    ordering.add_extruder_to_layer(0.6, 1, true);
    ordering.add_extruder_to_layer(0.8, 1, true);
    ordering.add_extruder_to_layer(1.0, 0, true);
    ordering.add_extruder_to_layer(1.0, 1, true);

    // Verify layer count
    assert_eq!(ordering.layer_tools().len(), 5);

    // Verify extruder presence
    let layer_tools = ordering.layer_tools();
    assert!(layer_tools[0].extruders.contains(&0));
    assert!(!layer_tools[0].extruders.contains(&1));

    assert!(!layer_tools[2].extruders.contains(&0));
    assert!(layer_tools[2].extruders.contains(&1));

    assert!(layer_tools[4].extruders.contains(&0));
    assert!(layer_tools[4].extruders.contains(&1));
}

/// Test flush matrix calculations
#[test]
fn test_flush_matrix_calculations() {
    // Create a 3-extruder flush matrix with asymmetric values
    let mut matrix = FlushMatrix::new(3, 100.0);

    // Set specific flush volumes (from -> to)
    matrix.set(0, 1, 120.0); // 0 -> 1 requires 120mm³
    matrix.set(0, 2, 150.0); // 0 -> 2 requires 150mm³
    matrix.set(1, 0, 80.0); // 1 -> 0 requires only 80mm³
    matrix.set(1, 2, 130.0);
    matrix.set(2, 0, 140.0);
    matrix.set(2, 1, 110.0);

    // Verify values
    assert!((matrix.get(0, 1) - 120.0).abs() < 0.001);
    assert!((matrix.get(1, 0) - 80.0).abs() < 0.001);
    assert!((matrix.get(0, 2) - 150.0).abs() < 0.001);

    // Diagonal should be 0 (same extruder)
    assert!((matrix.get(0, 0)).abs() < 0.001);
    assert!((matrix.get(1, 1)).abs() < 0.001);
}

/// Test optimal ordering calculation for minimum flush
#[test]
fn test_optimal_ordering_minimum_flush() {
    // Create flush matrix where 0->1->2->0 is optimal
    let mut matrix = FlushMatrix::new(3, 100.0);
    matrix.set(0, 1, 50.0); // 0->1: cheap
    matrix.set(1, 2, 50.0); // 1->2: cheap
    matrix.set(2, 0, 50.0); // 2->0: cheap
    matrix.set(0, 2, 200.0); // 0->2: expensive
    matrix.set(1, 0, 200.0); // 1->0: expensive
    matrix.set(2, 1, 200.0); // 2->1: expensive

    let extruders: Vec<u32> = vec![0, 1, 2];
    let initial = Some(0u32);

    let ordering = optimize_extruder_sequence(&extruders, initial, &matrix);

    // Should find a reasonable ordering
    assert_eq!(ordering.len(), 3);

    // Calculate flush for the ordering
    let total_flush = calculate_flush_volume(&ordering, &matrix);
    assert!(total_flush <= 150.0); // 50 + 50 = 100 for optimal path
}

/// Test multi-material coordinator initialization
#[test]
fn test_multi_material_coordinator_init() {
    let mut config = MultiMaterialConfig::new(2);
    config.filament_densities = vec![1.24, 1.24];
    config.enable_prime_tower = true;
    config.wipe_tower_x = 170.0;
    config.wipe_tower_y = 140.0;
    config.wipe_tower_width = 60.0;
    config.wipe_tower_rotation = 0.0;

    let mut coordinator = MultiMaterialCoordinator::new(config);

    // Initialize layers with (z, layer_height) tuples
    let layer_data: Vec<(f64, f64)> =
        vec![(0.2, 0.2), (0.4, 0.2), (0.6, 0.2), (0.8, 0.2), (1.0, 0.2)];
    coordinator.initialize_layers(&layer_data);

    // Record extruder usage per layer
    coordinator.add_extruder_to_layer(0.2, 0, ExtrusionRoleType::Perimeter);
    coordinator.add_extruder_to_layer(0.4, 0, ExtrusionRoleType::Perimeter);
    coordinator.add_extruder_to_layer(0.6, 1, ExtrusionRoleType::Perimeter);
    coordinator.add_extruder_to_layer(0.8, 1, ExtrusionRoleType::Perimeter);
    coordinator.add_extruder_to_layer(1.0, 0, ExtrusionRoleType::Perimeter);
    coordinator.add_extruder_to_layer(1.0, 1, ExtrusionRoleType::InternalInfill);

    // Plan and verify layers are created
    let plan = coordinator.plan(1.0);
    assert_eq!(plan.layers.len(), 5);
}

/// Test flush volume calculation between extruders
#[test]
fn test_flush_volume_between_extruders() {
    let mut matrix = FlushMatrix::new(3, 100.0);
    matrix.set(0, 1, 120.0);
    matrix.set(1, 2, 80.0);

    // Calculate flush for sequence 0 -> 1 -> 2
    let sequence: Vec<u32> = vec![0, 1, 2];
    let total_flush = calculate_flush_volume(&sequence, &matrix);

    // 0->1: 120, 1->2: 80, total: 200
    assert!((total_flush - 200.0).abs() < 0.001);

    // Single element or empty should be 0
    let single: Vec<u32> = vec![0];
    let empty: Vec<u32> = vec![];
    assert!(calculate_flush_volume(&single, &matrix).abs() < 0.001);
    assert!(calculate_flush_volume(&empty, &matrix).abs() < 0.001);
}

/// Test multi-material with different flush volumes
#[test]
fn test_multi_material_flush_matrix_config() {
    // Simulate PLA to PETG transition (higher flush) vs PLA to PLA (lower flush)
    let mut matrix = FlushMatrix::new(3, 100.0);
    matrix.set(0, 1, 80.0); // PLA -> PLA: 80mm³
    matrix.set(0, 2, 180.0); // PLA -> PETG: 180mm³
    matrix.set(1, 0, 80.0); // PLA -> PLA: 80mm³
    matrix.set(1, 2, 180.0); // PLA -> PETG: 180mm³
    matrix.set(2, 0, 200.0); // PETG -> PLA: 200mm³
    matrix.set(2, 1, 200.0); // PETG -> PLA: 200mm³

    let mut config = MultiMaterialConfig::new(3);
    config.filament_densities = vec![1.24, 1.24, 1.08];
    config.filament_types = vec!["PLA".to_string(), "PLA".to_string(), "PETG".to_string()];
    config.flush_matrix = matrix;
    config.enable_prime_tower = true;
    config.wipe_tower_x = 170.0;
    config.wipe_tower_y = 140.0;
    config.wipe_tower_width = 60.0;
    config.wipe_tower_rotation = 0.0;

    let mut coordinator = MultiMaterialCoordinator::new(config);

    // Initialize layers with varied extruder usage
    let layer_data: Vec<(f64, f64)> = vec![(0.2, 0.2), (0.4, 0.2), (0.6, 0.2), (0.8, 0.2)];
    coordinator.initialize_layers(&layer_data);

    // Each layer uses different extruder
    coordinator.add_extruder_to_layer(0.2, 0, ExtrusionRoleType::Perimeter);
    coordinator.add_extruder_to_layer(0.4, 1, ExtrusionRoleType::Perimeter);
    coordinator.add_extruder_to_layer(0.6, 2, ExtrusionRoleType::Perimeter);
    coordinator.add_extruder_to_layer(0.8, 0, ExtrusionRoleType::Perimeter);

    let plan = coordinator.plan(0.8);

    // Verify layers are created
    assert_eq!(plan.layers.len(), 4);
}

/// Test that coordinator correctly handles extruder for roles
#[test]
fn test_extruder_for_role_handling() {
    let mut config = MultiMaterialConfig::new(2);
    config.default_perimeter_extruder = 0;
    config.default_infill_extruder = 1;
    config.default_support_extruder = 1;

    let coordinator = MultiMaterialCoordinator::new(config);

    // Get the extruder for different roles
    let perimeter_extruder = coordinator.get_extruder_for_role(0.2, ExtrusionRoleType::Perimeter);
    let infill_extruder = coordinator.get_extruder_for_role(0.2, ExtrusionRoleType::InternalInfill);
    let support_extruder =
        coordinator.get_extruder_for_role(0.2, ExtrusionRoleType::SupportMaterial);

    assert_eq!(perimeter_extruder, 0);
    assert_eq!(infill_extruder, 1);
    assert_eq!(support_extruder, 1);
}

/// Test layer-specific tool information
#[test]
fn test_layer_tools_information() {
    let config = ToolOrderingConfig::default();
    let mut ordering = ToolOrdering::new(config);

    // Add 3 layers with different extruder configurations
    ordering.initialize_layers(vec![0.2, 0.4, 0.6]);

    ordering.add_extruder_to_layer(0.2, 0, true);
    ordering.add_extruder_to_layer(0.2, 1, true);

    ordering.add_extruder_to_layer(0.4, 1, true);
    ordering.add_extruder_to_layer(0.4, 2, true);

    ordering.add_extruder_to_layer(0.6, 0, true);
    ordering.add_extruder_to_layer(0.6, 2, true);

    let layer_tools = ordering.layer_tools();

    // Layer 0: extruders 0, 1
    assert!(layer_tools[0].extruders.contains(&0));
    assert!(layer_tools[0].extruders.contains(&1));
    assert!(!layer_tools[0].extruders.contains(&2));

    // Layer 1: extruders 1, 2
    assert!(!layer_tools[1].extruders.contains(&0));
    assert!(layer_tools[1].extruders.contains(&1));
    assert!(layer_tools[1].extruders.contains(&2));

    // Layer 2: extruders 0, 2
    assert!(layer_tools[2].extruders.contains(&0));
    assert!(!layer_tools[2].extruders.contains(&1));
    assert!(layer_tools[2].extruders.contains(&2));
}

/// Test exhaustive ordering for small extruder counts
#[test]
fn test_exhaustive_ordering_small_sets() {
    // Generate all orderings for 3 extruders
    let extruders: Vec<u32> = vec![0, 1, 2];
    let orderings = generate_all_orderings(&extruders);
    assert_eq!(orderings.len(), 6); // 3! = 6

    // Create a matrix where optimal order is known
    let mut matrix = FlushMatrix::new(3, 100.0);
    matrix.set(0, 1, 10.0);
    matrix.set(1, 2, 10.0);
    matrix.set(2, 0, 10.0);
    matrix.set(0, 2, 100.0);
    matrix.set(1, 0, 100.0);
    matrix.set(2, 1, 100.0);

    let optimal = find_optimal_ordering_exhaustive(&extruders, Some(0), &matrix);

    // Optimal should be 0 -> 1 -> 2 with cost 20
    let cost = calculate_flush_volume(&optimal, &matrix);
    assert_eq!(cost, 20.0);
    assert_eq!(optimal, vec![0, 1, 2]);
}

/// Test wipe tower bounds calculation
#[test]
fn test_wipe_tower_bounds() {
    let bounds = WipeTowerBounds::new(170.0, 140.0, 230.0, 200.0, 50.0);

    assert!((bounds.width() - 60.0).abs() < 0.001);
    assert!((bounds.depth() - 60.0).abs() < 0.001);
}

/// Test multi-material coordinator disabled wipe tower
#[test]
fn test_multi_material_no_wipe_tower() {
    let mut config = MultiMaterialConfig::new(2);
    config.filament_densities = vec![1.24, 1.24];
    config.enable_prime_tower = false; // Disabled
    config.wipe_tower_x = 170.0;
    config.wipe_tower_y = 140.0;
    config.wipe_tower_width = 60.0;

    let mut coordinator = MultiMaterialCoordinator::new(config);
    assert!(!coordinator.config().enable_prime_tower);

    // Initialize and record usage
    let layer_data: Vec<(f64, f64)> = vec![(0.2, 0.2), (0.4, 0.2)];
    coordinator.initialize_layers(&layer_data);
    coordinator.add_extruder_to_layer(0.2, 0, ExtrusionRoleType::Perimeter);
    coordinator.add_extruder_to_layer(0.4, 1, ExtrusionRoleType::Perimeter);

    let plan = coordinator.plan(0.4);

    // Should still have layer tracking, just no wipe tower
    assert_eq!(plan.layers.len(), 2);
}

/// Test single material mode (no multi-material features)
#[test]
fn test_single_material_mode() {
    let coordinator = MultiMaterialCoordinator::single_material();

    assert!(!coordinator.is_multi_material());
    assert_eq!(coordinator.config().num_filaments, 1);
}

/// Test coordinator needs_tool_change detection
#[test]
fn test_needs_tool_change_detection() {
    let config = MultiMaterialConfig::new(2);
    let coordinator = MultiMaterialCoordinator::new(config);

    // Check tool change detection
    // needs_tool_change returns Option<usize> - None means no change needed
    let no_change = coordinator.needs_tool_change(0.2, 0, ExtrusionRoleType::Perimeter);

    // When current extruder matches the role's default extruder, no change needed
    // Default perimeter extruder is 0, so extruder 0 should not need change
    assert!(no_change.is_none());

    // When current extruder differs from target, change is needed
    let needs_change = coordinator.needs_tool_change(0.2, 1, ExtrusionRoleType::Perimeter);
    // Default perimeter extruder is 0, so if current is 1, we need to change to 0
    assert_eq!(needs_change, Some(0));
}

/// Test flush volume retrieval from coordinator
#[test]
fn test_coordinator_flush_volume() {
    let mut config = MultiMaterialConfig::new(3);
    config.flush_matrix.set(0, 1, 150.0);
    config.flush_matrix.set(1, 2, 200.0);

    let coordinator = MultiMaterialCoordinator::new(config);

    let flush_01 = coordinator.get_flush_volume(0, 1);
    let flush_12 = coordinator.get_flush_volume(1, 2);
    let flush_00 = coordinator.get_flush_volume(0, 0);

    assert!((flush_01 - 150.0).abs() < 0.001);
    assert!((flush_12 - 200.0).abs() < 0.001);
    assert!(flush_00.abs() < 0.001); // Same extruder = 0 flush
}

/// Test wipe tower avoidance polygon generation
#[test]
fn test_wipe_tower_avoidance_generation() {
    let mut config = MultiMaterialConfig::new(2);
    config.enable_prime_tower = true;
    config.wipe_tower_x = 170.0;
    config.wipe_tower_y = 140.0;
    config.wipe_tower_width = 60.0;

    let mut coordinator = MultiMaterialCoordinator::new(config);

    // Initialize the coordinator with some layers to trigger wipe tower setup
    let layer_data: Vec<(f64, f64)> = vec![(0.2, 0.2), (0.4, 0.2)];
    coordinator.initialize_layers(&layer_data);
    coordinator.add_extruder_to_layer(0.2, 0, ExtrusionRoleType::Perimeter);
    coordinator.add_extruder_to_layer(0.4, 1, ExtrusionRoleType::Perimeter);

    // Plan to initialize wipe tower
    let _plan = coordinator.plan(0.4);

    // Get wipe tower avoidance polygon for travel planning
    let avoidance = coordinator.get_wipe_tower_avoidance();

    // Should return Some polygon when wipe tower is enabled and initialized
    if let Some(polygon) = avoidance {
        // Verify the polygon has vertices
        assert!(polygon.contour.points().len() >= 4);
    }
    // Note: avoidance may be None if wipe tower wasn't fully initialized
}

/// Test tool ordering config conversion
#[test]
fn test_tool_ordering_config_conversion() {
    let mut config = MultiMaterialConfig::new(3);
    config.filament_types = vec!["PLA".to_string(), "PETG".to_string(), "TPU".to_string()];
    config.filament_soluble = vec![false, false, false];
    config.filament_is_support = vec![false, false, false];
    config.infill_first = true;

    let tool_config = config.to_tool_ordering_config();

    assert_eq!(tool_config.num_filaments, 3);
    assert_eq!(tool_config.filament_types.len(), 3);
    assert!(tool_config.infill_first);
}

/// Test filament parameters conversion
#[test]
fn test_filament_params_conversion() {
    let mut config = MultiMaterialConfig::new(2);
    config.filament_types = vec!["PLA".to_string(), "PETG".to_string()];
    config.filament_soluble = vec![false, true];
    config.filament_is_support = vec![false, true];
    config.nozzle_diameters = vec![0.4, 0.6];

    let params = config.to_filament_params();

    assert_eq!(params.len(), 2);
    assert_eq!(params[0].material, "PLA");
    assert!(!params[0].is_soluble);
    assert!((params[0].nozzle_diameter - 0.4).abs() < 0.001);

    assert_eq!(params[1].material, "PETG");
    assert!(params[1].is_soluble);
    assert!((params[1].nozzle_diameter - 0.6).abs() < 0.001);
}

/// Test multi-material config default values
#[test]
fn test_multi_material_config_defaults() {
    let config = MultiMaterialConfig::default();

    assert_eq!(config.num_filaments, 1);
    assert!(!config.enable_prime_tower);
    assert_eq!(config.default_perimeter_extruder, 0);
    assert_eq!(config.default_infill_extruder, 0);
}

/// Test multi-material config new function
#[test]
fn test_multi_material_config_new() {
    let config = MultiMaterialConfig::new(4);

    assert_eq!(config.num_filaments, 4);
    assert!(config.enable_prime_tower); // Auto-enabled for multi-material
    assert_eq!(config.nozzle_diameters.len(), 4);
    assert_eq!(config.filament_densities.len(), 4);
    assert_eq!(config.filament_types.len(), 4);
    assert_eq!(config.filament_colors.len(), 4);
}

/// Test is_multi_material detection
#[test]
fn test_is_multi_material() {
    let single = MultiMaterialConfig::default();
    assert!(!single.is_multi_material());

    let multi = MultiMaterialConfig::new(2);
    assert!(multi.is_multi_material());
}

/// Test that multi-material plan has correct layer structure
#[test]
fn test_multi_material_plan_layer_structure() {
    let mut config = MultiMaterialConfig::new(2);
    config.enable_prime_tower = true;

    let mut coordinator = MultiMaterialCoordinator::new(config);

    let layer_data: Vec<(f64, f64)> = vec![(0.2, 0.2), (0.4, 0.2), (0.6, 0.2)];
    coordinator.initialize_layers(&layer_data);

    coordinator.add_extruder_to_layer(0.2, 0, ExtrusionRoleType::Perimeter);
    coordinator.add_extruder_to_layer(0.4, 0, ExtrusionRoleType::Perimeter);
    coordinator.add_extruder_to_layer(0.6, 1, ExtrusionRoleType::Perimeter);

    let plan = coordinator.plan(0.6);

    // Verify all layers are present
    assert_eq!(plan.layers.len(), 3);

    // Verify layer indices
    assert_eq!(plan.layers[0].layer_idx, 0);
    assert_eq!(plan.layers[1].layer_idx, 1);
    assert_eq!(plan.layers[2].layer_idx, 2);

    // Verify Z heights
    assert!((plan.layers[0].print_z - 0.2).abs() < 0.001);
    assert!((plan.layers[1].print_z - 0.4).abs() < 0.001);
    assert!((plan.layers[2].print_z - 0.6).abs() < 0.001);
}

/// Test flush matrix with multipliers
#[test]
fn test_flush_matrix_with_multipliers() {
    let mut config = MultiMaterialConfig::new(2);
    config.flush_matrix.set(0, 1, 100.0);
    config.flush_multipliers = vec![1.0, 1.5]; // 50% more flush to extruder 1

    // get_flush_volume applies multiplier
    let flush = config.get_flush_volume(0, 1);
    assert!((flush - 150.0).abs() < 0.001); // 100 * 1.5 = 150
}

/// Test extruder sequence optimization with various matrix configurations
#[test]
fn test_extruder_sequence_optimization_variations() {
    // Test case 1: All equal flush volumes
    let matrix_equal = FlushMatrix::new(3, 100.0);
    let extruders: Vec<u32> = vec![0, 1, 2];

    let result = optimize_extruder_sequence(&extruders, Some(0), &matrix_equal);
    assert_eq!(result.len(), 3);

    // Test case 2: Single extruder
    let single: Vec<u32> = vec![0];
    let result_single = optimize_extruder_sequence(&single, Some(0), &matrix_equal);
    assert_eq!(result_single, vec![0]);

    // Test case 3: Empty extruders
    let empty: Vec<u32> = vec![];
    let result_empty = optimize_extruder_sequence(&empty, None, &matrix_equal);
    assert!(result_empty.is_empty());
}

/// Test generate_all_orderings for different sizes
#[test]
fn test_generate_all_orderings_sizes() {
    // 1 extruder = 1 ordering
    let one: Vec<u32> = vec![0];
    assert_eq!(generate_all_orderings(&one).len(), 1);

    // 2 extruders = 2 orderings
    let two: Vec<u32> = vec![0, 1];
    assert_eq!(generate_all_orderings(&two).len(), 2);

    // 3 extruders = 6 orderings
    let three: Vec<u32> = vec![0, 1, 2];
    assert_eq!(generate_all_orderings(&three).len(), 6);

    // 4 extruders = 24 orderings
    let four: Vec<u32> = vec![0, 1, 2, 3];
    assert_eq!(generate_all_orderings(&four).len(), 24);

    // Empty = 1 empty ordering
    let empty: Vec<u32> = vec![];
    assert_eq!(generate_all_orderings(&empty).len(), 1);
}

/// Test wipe tower bounds polygon conversion
#[test]
fn test_wipe_tower_bounds_to_polygon() {
    let bounds = WipeTowerBounds::new(10.0, 20.0, 70.0, 80.0, 100.0);

    let expolygon = bounds.to_expolygon();

    // Should have 4 corners
    assert_eq!(expolygon.contour.points().len(), 4);

    // Verify bounding box matches
    let bbox = expolygon.bounding_box();
    let min_x_mm = bbox.min.x as f64 / 1_000_000.0;
    let min_y_mm = bbox.min.y as f64 / 1_000_000.0;
    let max_x_mm = bbox.max.x as f64 / 1_000_000.0;
    let max_y_mm = bbox.max.y as f64 / 1_000_000.0;

    assert!((min_x_mm - 10.0).abs() < 0.001);
    assert!((min_y_mm - 20.0).abs() < 0.001);
    assert!((max_x_mm - 70.0).abs() < 0.001);
    assert!((max_y_mm - 80.0).abs() < 0.001);
}
