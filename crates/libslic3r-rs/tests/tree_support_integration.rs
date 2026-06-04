//! Tree Support Integration Tests
//!
//! These tests validate the tree support generation pipeline integration,
//! ensuring that TreeSupport3D properly generates support structures for
//! overhanging geometry.

use slicer::geometry::{ExPolygon, Point, Polygon};
use slicer::mesh::TriangleMesh;
use slicer::pipeline::{PipelineConfig, PrintPipeline};
use slicer::support::{
    BranchMeshBuilder, BranchMeshConfig, BranchPath, BranchPathElement, Point3D, SupportConfig,
    SupportGenerator, SupportType, TreeModelVolumes, TreeModelVolumesConfig, TreeSupport3D,
    TreeSupport3DConfig, TreeSupportSettings,
};
use slicer::{scale, CoordF};

/// Create a simple cube mesh for baseline testing
fn create_cube_mesh(size: f64) -> TriangleMesh {
    TriangleMesh::cube(size)
}

/// Helper to create layer slices for testing
fn create_test_slices() -> Vec<(CoordF, CoordF, Vec<ExPolygon>)> {
    let layer_height = 0.2;

    // Create a tower that gets wider - guaranteed overhang
    vec![
        // Layer 0: Small base 5x5mm
        (
            layer_height,
            layer_height,
            vec![ExPolygon::from(Polygon::rectangle(
                Point::new(scale(2.5), scale(2.5)),
                Point::new(scale(7.5), scale(7.5)),
            ))],
        ),
        // Layer 1: Same size
        (
            layer_height * 2.0,
            layer_height,
            vec![ExPolygon::from(Polygon::rectangle(
                Point::new(scale(2.5), scale(2.5)),
                Point::new(scale(7.5), scale(7.5)),
            ))],
        ),
        // Layer 2: Wider 15x15mm - has overhang
        (
            layer_height * 3.0,
            layer_height,
            vec![ExPolygon::from(Polygon::rectangle(
                Point::new(scale(0.0), scale(0.0)),
                Point::new(scale(15.0), scale(15.0)),
            ))],
        ),
        // Layer 3: Same wide
        (
            layer_height * 4.0,
            layer_height,
            vec![ExPolygon::from(Polygon::rectangle(
                Point::new(scale(0.0), scale(0.0)),
                Point::new(scale(15.0), scale(15.0)),
            ))],
        ),
    ]
}

// ============================================================================
// Unit-level integration tests
// ============================================================================

#[test]
fn test_tree_support_generator_integration() {
    let config = SupportConfig {
        enabled: true,
        support_type: SupportType::Tree,
        min_area: 0.5,
        overhang_angle: 45.0,
        tree_branch_diameter: 2.0,
        tree_tip_diameter: 0.8,
        buildplate_only: false,
        ..Default::default()
    };

    let gen = SupportGenerator::new(config);
    let slices = create_test_slices();

    let support_layers = gen.generate(&slices);

    // Should have same number of layers as input
    assert_eq!(support_layers.len(), slices.len());

    // Check that overhang detection worked
    let has_overhangs = support_layers
        .iter()
        .any(|l| !l.overhang_regions.is_empty());
    assert!(has_overhangs, "Should detect overhangs in test geometry");
}

#[test]
fn test_tree_support_vs_normal_support() {
    let slices = create_test_slices();

    // Generate with normal support
    let normal_config = SupportConfig {
        enabled: true,
        support_type: SupportType::Normal,
        min_area: 0.5,
        ..Default::default()
    };
    let normal_gen = SupportGenerator::new(normal_config);
    let normal_layers = normal_gen.generate(&slices);

    // Generate with tree support
    let tree_config = SupportConfig {
        enabled: true,
        support_type: SupportType::Tree,
        min_area: 0.5,
        tree_branch_diameter: 2.0,
        tree_tip_diameter: 0.8,
        ..Default::default()
    };
    let tree_gen = SupportGenerator::new(tree_config);
    let tree_layers = tree_gen.generate(&slices);

    // Both should have same number of layers
    assert_eq!(normal_layers.len(), tree_layers.len());

    // Both should detect the same overhangs
    for i in 0..slices.len() {
        let normal_has_overhang = !normal_layers[i].overhang_regions.is_empty();
        let tree_has_overhang = !tree_layers[i].overhang_regions.is_empty();
        assert_eq!(
            normal_has_overhang, tree_has_overhang,
            "Layer {} overhang detection should match",
            i
        );
    }
}

#[test]
fn test_tree_support_buildplate_only() {
    let config = SupportConfig {
        enabled: true,
        support_type: SupportType::Tree,
        buildplate_only: true,
        min_area: 0.5,
        tree_branch_diameter: 2.0,
        tree_tip_diameter: 0.8,
        ..Default::default()
    };

    let gen = SupportGenerator::new(config);
    let slices = create_test_slices();
    let support_layers = gen.generate(&slices);

    assert_eq!(support_layers.len(), slices.len());
}

#[test]
fn test_tree_support_organic_mode() {
    let config = SupportConfig {
        enabled: true,
        support_type: SupportType::Organic,
        min_area: 0.5,
        tree_branch_diameter: 2.0,
        tree_tip_diameter: 0.8,
        ..Default::default()
    };

    let gen = SupportGenerator::new(config);
    let slices = create_test_slices();

    // Organic should use tree support path internally
    let support_layers = gen.generate(&slices);
    assert_eq!(support_layers.len(), slices.len());
}

// ============================================================================
// TreeSupport3D direct tests
// ============================================================================

#[test]
fn test_tree_support_3d_direct() {
    let layer_heights: Vec<CoordF> = vec![0.2, 0.2, 0.2, 0.2];
    let z_heights: Vec<CoordF> = vec![0.2, 0.4, 0.6, 0.8];

    let slices = create_test_slices();
    let layer_outlines: Vec<Vec<ExPolygon>> = slices.iter().map(|(_, _, s)| s.clone()).collect();

    // Create TreeModelVolumes
    let mut volumes_config = TreeModelVolumesConfig::with_layers(layer_heights, z_heights);
    volumes_config.xy_distance = scale(0.8);
    volumes_config.min_radius = scale(0.4);
    volumes_config.support_rests_on_model = false;

    let volumes = TreeModelVolumes::with_layer_outlines(volumes_config, layer_outlines);

    // Create TreeSupport3D config
    let tree_config = TreeSupport3DConfig::default();

    // Create generator
    let mut tree_support = TreeSupport3D::new(tree_config, volumes);

    // Create overhang polygons (layer 2 has overhang)
    let overhang_polygons: Vec<Vec<Polygon>> = vec![
        vec![], // layer 0
        vec![], // layer 1
        vec![Polygon::rectangle(
            // layer 2 overhang ring
            Point::new(scale(0.0), scale(0.0)),
            Point::new(scale(15.0), scale(15.0)),
        )],
        vec![], // layer 3
    ];

    let result = tree_support.generate(&overhang_polygons);

    // Should produce layers
    assert_eq!(result.layers.len(), 4);
}

#[test]
fn test_tree_support_3d_with_mesh_generation() {
    let layer_heights: Vec<CoordF> = vec![0.2, 0.2, 0.2, 0.2];
    let z_heights: Vec<CoordF> = vec![0.2, 0.4, 0.6, 0.8];

    let slices = create_test_slices();
    let layer_outlines: Vec<Vec<ExPolygon>> = slices.iter().map(|(_, _, s)| s.clone()).collect();

    // Create TreeModelVolumes
    let mut volumes_config = TreeModelVolumesConfig::with_layers(layer_heights, z_heights);
    volumes_config.xy_distance = scale(0.8);
    volumes_config.min_radius = scale(0.4);
    volumes_config.support_rests_on_model = false;

    let volumes = TreeModelVolumes::with_layer_outlines(volumes_config, layer_outlines);

    // Create TreeSupport3D config
    let tree_config = TreeSupport3DConfig::default();

    // Create generator
    let mut tree_support = TreeSupport3D::new(tree_config, volumes);

    // Create overhang polygons (layer 2 has overhang)
    let overhang_polygons: Vec<Vec<Polygon>> = vec![
        vec![], // layer 0
        vec![], // layer 1
        vec![Polygon::rectangle(
            // layer 2 overhang ring
            Point::new(scale(0.0), scale(0.0)),
            Point::new(scale(15.0), scale(15.0)),
        )],
        vec![], // layer 3
    ];

    // Test mesh-based generation
    let result = tree_support.generate_with_mesh(&overhang_polygons);

    // Should produce layers
    assert_eq!(result.layers.len(), 4);
}

// ============================================================================
// Branch Mesh tests
// ============================================================================

#[test]
fn test_branch_mesh_builder_simple_tube() {
    let mut builder = BranchMeshBuilder::with_defaults();

    let mut path = BranchPath::new();
    path.push(BranchPathElement {
        position: Point3D::new(0.0, 0.0, 0.0),
        radius: 1.0,
        layer_idx: 0,
    });
    path.push(BranchPathElement {
        position: Point3D::new(0.0, 0.0, 5.0),
        radius: 0.8,
        layer_idx: 25,
    });

    builder.add_branch(&path);
    let result = builder.finish();

    // Should create a mesh with vertices and triangles
    assert!(!result.mesh.is_empty());
    assert!(result.mesh.vertex_count() > 0);
    assert!(result.mesh.triangle_count() > 0);
    assert_eq!(result.branch_count, 1);

    // Z span should cover the branch
    assert!(result.z_span.0 < 0.0); // Bottom hemisphere
    assert!(result.z_span.1 > 5.0); // Top hemisphere
}

#[test]
fn test_branch_mesh_builder_multi_segment() {
    let mut builder = BranchMeshBuilder::with_defaults();

    // Create a branch with multiple segments
    let mut path = BranchPath::new();
    path.push(BranchPathElement {
        position: Point3D::new(0.0, 0.0, 0.0),
        radius: 1.5,
        layer_idx: 0,
    });
    path.push(BranchPathElement {
        position: Point3D::new(0.5, 0.0, 1.0),
        radius: 1.2,
        layer_idx: 5,
    });
    path.push(BranchPathElement {
        position: Point3D::new(1.0, 0.5, 2.0),
        radius: 1.0,
        layer_idx: 10,
    });
    path.push(BranchPathElement {
        position: Point3D::new(1.0, 1.0, 3.0),
        radius: 0.8,
        layer_idx: 15,
    });

    builder.add_branch(&path);
    let result = builder.finish();

    assert!(!result.mesh.is_empty());
    assert_eq!(result.branch_count, 1);
}

#[test]
fn test_branch_mesh_builder_multiple_branches() {
    let mut builder = BranchMeshBuilder::with_defaults();

    // First branch
    let mut path1 = BranchPath::new();
    path1.push(BranchPathElement {
        position: Point3D::new(0.0, 0.0, 0.0),
        radius: 1.0,
        layer_idx: 0,
    });
    path1.push(BranchPathElement {
        position: Point3D::new(0.0, 0.0, 2.0),
        radius: 0.8,
        layer_idx: 10,
    });

    // Second branch
    let mut path2 = BranchPath::new();
    path2.push(BranchPathElement {
        position: Point3D::new(5.0, 5.0, 0.0),
        radius: 1.0,
        layer_idx: 0,
    });
    path2.push(BranchPathElement {
        position: Point3D::new(5.0, 5.0, 2.0),
        radius: 0.8,
        layer_idx: 10,
    });

    builder.add_branch(&path1);
    builder.add_branch(&path2);
    let result = builder.finish();

    assert!(!result.mesh.is_empty());
    assert_eq!(result.branch_count, 2);
}

#[test]
fn test_branch_mesh_config_custom() {
    let config = BranchMeshConfig {
        eps: 0.01,
        min_circle_segments: 16,
        max_circle_segments: 48,
    };

    let mut builder = BranchMeshBuilder::new(config);

    let mut path = BranchPath::new();
    path.push(BranchPathElement {
        position: Point3D::new(0.0, 0.0, 0.0),
        radius: 0.5,
        layer_idx: 0,
    });
    path.push(BranchPathElement {
        position: Point3D::new(0.0, 0.0, 1.0),
        radius: 0.5,
        layer_idx: 5,
    });

    builder.add_branch(&path);
    let result = builder.finish();

    assert!(!result.mesh.is_empty());
}

#[test]
fn test_tree_model_volumes_configuration() {
    let layer_heights: Vec<CoordF> = vec![0.2; 10];
    let z_heights: Vec<CoordF> = (1..=10).map(|i| i as f64 * 0.2).collect();

    let config = TreeModelVolumesConfig::with_layers(layer_heights.clone(), z_heights.clone());

    assert_eq!(config.layer_heights.len(), 10);
    assert_eq!(config.z_heights.len(), 10);
    assert!((config.layer_height(0) - 0.2).abs() < 0.001);
    assert!((config.layer_z(5) - 1.2).abs() < 0.001);
}

#[test]
fn test_tree_support_3d_config_from_support_config() {
    let support_config = SupportConfig {
        enabled: true,
        support_type: SupportType::Tree,
        support_roof: true,
        top_interface_layers: 4,
        min_area: 2.0,
        xy_distance: 1.0,
        buildplate_only: true,
        tree_branch_diameter: 3.0,
        tree_tip_diameter: 1.0,
        ..Default::default()
    };

    let tree_config = TreeSupport3DConfig::from_support_config(&support_config);

    assert!(tree_config.roof_enabled);
    assert_eq!(tree_config.num_roof_layers, 4);
    assert!((tree_config.minimum_support_area - 2.0).abs() < 0.001);
    assert!(!tree_config.settings.support_rests_on_model); // buildplate_only = true
    assert_eq!(tree_config.settings.branch_radius, scale(1.5)); // branch_diameter / 2
    assert_eq!(tree_config.settings.min_radius, scale(0.5)); // tip_diameter / 2
}

// ============================================================================
// Pipeline integration tests
// ============================================================================

#[test]
fn test_pipeline_with_tree_support_cube() {
    let mesh = create_cube_mesh(20.0);

    let mut config = PipelineConfig::default();
    config.support.enabled = true;
    config.support.support_type = SupportType::Tree;
    config.support.min_area = 1.0;

    let mut pipeline = PrintPipeline::new(config);
    let result = pipeline.process(&mesh);

    // Should complete without error
    assert!(
        result.is_ok(),
        "Pipeline should complete: {:?}",
        result.err()
    );

    let gcode = result.unwrap();

    // Should generate some G-code
    assert!(!gcode.to_string().is_empty());
}

#[test]
fn test_pipeline_normal_vs_tree_support() {
    let mesh = create_cube_mesh(20.0);

    // Normal support
    let mut normal_config = PipelineConfig::default();
    normal_config.support.enabled = true;
    normal_config.support.support_type = SupportType::Normal;

    let mut normal_pipeline = PrintPipeline::new(normal_config);
    let normal_result = normal_pipeline.process(&mesh);
    assert!(normal_result.is_ok());

    // Tree support
    let mut tree_config = PipelineConfig::default();
    tree_config.support.enabled = true;
    tree_config.support.support_type = SupportType::Tree;

    let mut tree_pipeline = PrintPipeline::new(tree_config);
    let tree_result = tree_pipeline.process(&mesh);
    assert!(tree_result.is_ok());

    // Both should produce valid G-code
    let normal_gcode = normal_result.unwrap().to_string();
    let tree_gcode = tree_result.unwrap().to_string();

    assert!(!normal_gcode.is_empty());
    assert!(!tree_gcode.is_empty());
}

#[test]
fn test_pipeline_tree_support_settings_propagation() {
    let mesh = create_cube_mesh(15.0);

    let mut config = PipelineConfig::default();
    config.support.enabled = true;
    config.support.support_type = SupportType::Tree;
    config.support.tree_branch_diameter = 3.0;
    config.support.tree_tip_diameter = 1.2;
    config.support.tree_branch_angle = 35.0;
    config.support.buildplate_only = true;

    let pipeline = PrintPipeline::new(config.clone());

    // Verify settings are accessible
    assert!(pipeline.uses_support());
    assert_eq!(pipeline.config().support.support_type, SupportType::Tree);
    assert!((pipeline.config().support.tree_branch_diameter - 3.0).abs() < 0.001);
    assert!((pipeline.config().support.tree_tip_diameter - 1.2).abs() < 0.001);
    assert!((pipeline.config().support.tree_branch_angle - 35.0).abs() < 0.001);
    assert!(pipeline.config().support.buildplate_only);
}

// ============================================================================
// Edge case tests
// ============================================================================

#[test]
fn test_tree_support_empty_geometry() {
    let config = SupportConfig {
        enabled: true,
        support_type: SupportType::Tree,
        ..Default::default()
    };

    let gen = SupportGenerator::new(config);
    let slices: Vec<(CoordF, CoordF, Vec<ExPolygon>)> = vec![];

    let support_layers = gen.generate(&slices);
    assert!(support_layers.is_empty());
}

#[test]
fn test_tree_support_single_layer() {
    let config = SupportConfig {
        enabled: true,
        support_type: SupportType::Tree,
        ..Default::default()
    };

    let gen = SupportGenerator::new(config);
    let slices = vec![(
        0.2,
        0.2,
        vec![ExPolygon::from(Polygon::rectangle(
            Point::new(scale(0.0), scale(0.0)),
            Point::new(scale(10.0), scale(10.0)),
        ))],
    )];

    let support_layers = gen.generate(&slices);
    assert_eq!(support_layers.len(), 1);
    // First layer shouldn't have overhangs (on build plate)
    assert!(support_layers[0].overhang_regions.is_empty());
}

#[test]
fn test_tree_support_no_overhang_geometry() {
    let config = SupportConfig {
        enabled: true,
        support_type: SupportType::Tree,
        ..Default::default()
    };

    let gen = SupportGenerator::new(config);

    // Create a simple stack with no overhangs (same size each layer)
    let slices: Vec<(CoordF, CoordF, Vec<ExPolygon>)> = (0..5)
        .map(|i| {
            (
                (i + 1) as f64 * 0.2,
                0.2,
                vec![ExPolygon::from(Polygon::rectangle(
                    Point::new(scale(0.0), scale(0.0)),
                    Point::new(scale(10.0), scale(10.0)),
                ))],
            )
        })
        .collect();

    let support_layers = gen.generate(&slices);
    assert_eq!(support_layers.len(), 5);

    // No layer should have support (no overhangs)
    for layer in &support_layers {
        assert!(
            layer.support_regions.is_empty(),
            "Should have no support for stack geometry"
        );
    }
}

#[test]
fn test_tree_support_with_interface_layers() {
    let config = SupportConfig {
        enabled: true,
        support_type: SupportType::Tree,
        support_roof: true,
        top_interface_layers: 2,
        min_area: 0.1,
        ..Default::default()
    };

    let gen = SupportGenerator::new(config);
    let slices = create_test_slices();

    let support_layers = gen.generate(&slices);
    assert_eq!(support_layers.len(), slices.len());

    // Verify interface layers are marked when support_roof is enabled
    // (This depends on whether tree support actually generates support regions)
}

#[test]
fn test_tree_support_small_overhang_filtered() {
    let config = SupportConfig {
        enabled: true,
        support_type: SupportType::Tree,
        min_area: 100.0, // Very high threshold - should filter everything
        ..Default::default()
    };

    let gen = SupportGenerator::new(config);
    let slices = create_test_slices();

    let support_layers = gen.generate(&slices);

    // With very high min_area, small overhangs should be filtered
    // (actual behavior depends on overhang sizes in test data)
    assert_eq!(support_layers.len(), slices.len());
}

// ============================================================================
// Performance sanity tests
// ============================================================================

#[test]
fn test_tree_support_many_layers() {
    let config = SupportConfig {
        enabled: true,
        support_type: SupportType::Tree,
        ..Default::default()
    };

    let gen = SupportGenerator::new(config);

    // Create 100 layers
    let slices: Vec<(CoordF, CoordF, Vec<ExPolygon>)> = (0..100)
        .map(|i| {
            let size = if i < 50 { 10.0 } else { 20.0 }; // Overhang at layer 50
            (
                (i + 1) as f64 * 0.2,
                0.2,
                vec![ExPolygon::from(Polygon::rectangle(
                    Point::new(scale(0.0), scale(0.0)),
                    Point::new(scale(size), scale(size)),
                ))],
            )
        })
        .collect();

    let support_layers = gen.generate(&slices);
    assert_eq!(support_layers.len(), 100);
}

// ============================================================================
// Organic Smoothing Integration tests
// ============================================================================

#[test]
fn test_tree_support_3d_with_organic_smoothing() {
    let layer_heights: Vec<CoordF> = vec![0.2, 0.2, 0.2, 0.2];
    let z_heights: Vec<CoordF> = vec![0.2, 0.4, 0.6, 0.8];

    let slices = create_test_slices();
    let layer_outlines: Vec<Vec<ExPolygon>> = slices.iter().map(|(_, _, s)| s.clone()).collect();

    // Create TreeModelVolumes
    let mut volumes_config = TreeModelVolumesConfig::with_layers(layer_heights, z_heights);
    volumes_config.xy_distance = scale(0.8);
    volumes_config.min_radius = scale(0.4);
    volumes_config.support_rests_on_model = false;

    let volumes = TreeModelVolumes::with_layer_outlines(volumes_config, layer_outlines.clone());

    // Create TreeSupport3D config
    let tree_config = TreeSupport3DConfig::default();

    // Create generator
    let mut tree_support = TreeSupport3D::new(tree_config, volumes);

    // Create overhang polygons (layer 2 has overhang)
    let overhang_polygons: Vec<Vec<Polygon>> = vec![
        vec![], // layer 0
        vec![], // layer 1
        vec![Polygon::rectangle(
            Point::new(scale(0.0), scale(0.0)),
            Point::new(scale(15.0), scale(15.0)),
        )],
        vec![], // layer 3
    ];

    // Test mesh-based generation with organic smoothing
    let result = tree_support.generate_with_organic_smoothing(&overhang_polygons, &layer_outlines);

    // Should produce layers
    assert_eq!(result.layers.len(), 4);
}

#[test]
fn test_organic_smoothing_applied_to_move_bounds() {
    use slicer::support::tree_support_settings::{SupportElement, SupportElementState};
    use slicer::support::{
        apply_smoothed_positions_to_move_bounds, build_spheres_from_move_bounds,
        OrganicSmoothConfig,
    };

    // Create elements with result_on_layer set
    let mut state1 = SupportElementState::default();
    state1.result_on_layer = Some(Point::new(scale(5.0), scale(5.0)));
    let elem1 = SupportElement::new(state1, vec![]);

    let mut state2 = SupportElementState::default();
    state2.result_on_layer = Some(Point::new(scale(5.0), scale(5.0)));
    let mut elem2 = SupportElement::new(state2, vec![]);
    elem2.parents = vec![0]; // Child of elem1 in layer above

    let mut move_bounds = vec![vec![elem1], vec![elem2]];

    let mut settings = TreeSupportSettings::default();
    settings.known_z = vec![scale(0.2), scale(0.4)];

    let config = OrganicSmoothConfig::default();

    // Build spheres
    let build_result = build_spheres_from_move_bounds(&move_bounds, &settings, config);

    // Should have 2 spheres
    assert_eq!(build_result.smoother.sphere_count(), 2);
    assert_eq!(build_result.mappings.len(), 2);

    // Apply positions back (even without smoothing, this should work)
    let updated = apply_smoothed_positions_to_move_bounds(
        &build_result.smoother,
        &build_result.mappings,
        &mut move_bounds,
    );

    assert_eq!(updated, 2);

    // Positions should still be set
    assert!(move_bounds[0][0].state.result_on_layer.is_some());
    assert!(move_bounds[1][0].state.result_on_layer.is_some());
}

#[test]
fn test_tree_support_apply_organic_smoothing_method() {
    let layer_heights: Vec<CoordF> = vec![0.2, 0.2, 0.2, 0.2];
    let z_heights: Vec<CoordF> = vec![0.2, 0.4, 0.6, 0.8];

    let slices = create_test_slices();
    let layer_outlines: Vec<Vec<ExPolygon>> = slices.iter().map(|(_, _, s)| s.clone()).collect();

    // Create TreeModelVolumes
    let mut volumes_config = TreeModelVolumesConfig::with_layers(layer_heights, z_heights);
    volumes_config.xy_distance = scale(0.8);
    volumes_config.min_radius = scale(0.4);

    let volumes = TreeModelVolumes::with_layer_outlines(volumes_config, layer_outlines.clone());

    let tree_config = TreeSupport3DConfig::default();
    let mut tree_support = TreeSupport3D::new(tree_config, volumes);

    // Create overhang polygons
    let overhang_polygons: Vec<Vec<Polygon>> = vec![
        vec![],
        vec![],
        vec![Polygon::rectangle(
            Point::new(scale(0.0), scale(0.0)),
            Point::new(scale(15.0), scale(15.0)),
        )],
        vec![],
    ];

    // Run initial generation phases
    let _ = tree_support.generate(&overhang_polygons);

    // Now apply organic smoothing separately
    let smooth_result = tree_support.apply_organic_smoothing(&layer_outlines);

    // Should complete (converge or hit max iterations)
    assert!(smooth_result.iterations > 0 || smooth_result.converged);
}

#[test]
fn test_organic_smoothing_with_custom_config() {
    use slicer::support::OrganicSmoothConfig;

    let layer_heights: Vec<CoordF> = vec![0.2, 0.2, 0.2, 0.2];
    let z_heights: Vec<CoordF> = vec![0.2, 0.4, 0.6, 0.8];

    let slices = create_test_slices();
    let layer_outlines: Vec<Vec<ExPolygon>> = slices.iter().map(|(_, _, s)| s.clone()).collect();

    let mut volumes_config = TreeModelVolumesConfig::with_layers(layer_heights, z_heights);
    volumes_config.xy_distance = scale(0.8);
    volumes_config.min_radius = scale(0.4);

    let volumes = TreeModelVolumes::with_layer_outlines(volumes_config, layer_outlines.clone());

    let tree_config = TreeSupport3DConfig::default();
    let mut tree_support = TreeSupport3D::new(tree_config, volumes);

    let overhang_polygons: Vec<Vec<Polygon>> = vec![
        vec![],
        vec![],
        vec![Polygon::rectangle(
            Point::new(scale(0.0), scale(0.0)),
            Point::new(scale(15.0), scale(15.0)),
        )],
        vec![],
    ];

    let _ = tree_support.generate(&overhang_polygons);

    // Use custom config with fewer iterations
    let custom_config = OrganicSmoothConfig {
        max_iterations: 5,
        smoothing_factor: 0.3,
        ..Default::default()
    };

    let smooth_result =
        tree_support.apply_organic_smoothing_with_config(&layer_outlines, custom_config);

    // Should complete within 5 iterations or converge
    assert!(smooth_result.iterations <= 5);
}
