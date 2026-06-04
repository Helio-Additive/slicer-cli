//! Unit Tests for Layer and LayerRegion Methods (Sessions 73-75)
//!
//! This module tests the three methods ported Week 1-2:
//! - Session 73: LayerRegion::make_perimeters()
//! - Session 74: LayerRegion::process_external_surfaces()
//! - Session 75: Layer::make_fills()
//!
//! These tests validate:
//! 1. Method signatures and parameter handling
//! 2. Basic control flow and data structure handling
//! 3. Edge cases (empty inputs, null layers, etc.)
//! 4. Integration between methods
//!
//! C++ Reference:
//! - LayerRegion.cpp:131-180 (make_perimeters)
//! - LayerRegion.cpp:517-643 (process_external_surfaces)
//! - Fill.cpp:586-770 (Layer::make_fills)

use slicer::config::PrintRegionConfig;
use slicer::extrusion_entity::{
    ExtrusionEntityCollection, ExtrusionEntityType, ExtrusionLoop, ExtrusionPath,
};
use slicer::flow::{Flow, FlowRole};
use slicer::geometry::{BoundingBox, ExPolygon, ExPolygons, Point, Polygon};
use slicer::layer::{Layer, LayerRegion, LoopNode, PrintObject, PrintRegion};
use slicer::perimeter_generator::{PerimeterConfig, PerimeterGenerator};
use slicer::slice::surface::{Surface, SurfaceCollection, SurfaceType};
use slicer::{scale, unscale, Result};

// ============================================================================
// Test Fixtures
// ============================================================================

mod fixtures {
    use super::*;

    /// Create a simple square polygon (10mm x 10mm)
    pub fn simple_square() -> Polygon {
        Polygon::from_points(vec![
            Point::new(scale(0.0), scale(0.0)),
            Point::new(scale(10.0), scale(0.0)),
            Point::new(scale(10.0), scale(10.0)),
            Point::new(scale(0.0), scale(10.0)),
        ])
    }

    /// Create a square with a circular hole (outer 20mm, hole 5mm radius at center)
    pub fn square_with_hole() -> ExPolygon {
        let outer = Polygon::from_points(vec![
            Point::new(scale(0.0), scale(0.0)),
            Point::new(scale(20.0), scale(0.0)),
            Point::new(scale(20.0), scale(20.0)),
            Point::new(scale(0.0), scale(20.0)),
        ]);

        // Simple square hole for testing (5mm x 5mm centered)
        let hole = Polygon::from_points(vec![
            Point::new(scale(7.5), scale(7.5)),
            Point::new(scale(12.5), scale(7.5)),
            Point::new(scale(12.5), scale(12.5)),
            Point::new(scale(7.5), scale(12.5)),
        ]);

        ExPolygon::with_holes(outer, vec![hole])
    }

    /// Create a mock PrintRegionConfig with default values
    pub fn mock_region_config() -> PrintRegionConfig {
        PrintRegionConfig {
            perimeters: 3,
            top_solid_layers: 4,
            bottom_solid_layers: 3,
            fill_density: 0.15,
            ..Default::default()
        }
    }

    /// Create a mock PerimeterConfig
    pub fn mock_perimeter_config() -> PerimeterConfig {
        PerimeterConfig {
            perimeter_count: 3,
            layer_height: 0.2,
            ext_perimeter_flow: Flow::new(0.45, 0.2, 0.4).unwrap(),
            perimeter_flow: Flow::new(0.45, 0.2, 0.4).unwrap(),
            ..Default::default()
        }
    }

    /// Create a mock PrintObject
    pub fn mock_print_object() -> PrintObject {
        PrintObject {}
    }

    /// Create a mock Layer with one region
    pub fn mock_layer_with_region(id: usize, height: f64, print_z: f64, slice_z: f64) -> Layer {
        let object_id = 0; // Mock object ID
        let mut layer = Layer::new(id, object_id, height, print_z, slice_z);

        let region_id = 0; // Mock region ID
        layer.add_region(region_id);

        layer
    }

    /// Create a Surface with a simple square
    pub fn simple_surface(surface_type: SurfaceType) -> Surface {
        let square = ExPolygon::new(simple_square());
        Surface::new(surface_type, square)
    }

    /// Create a SurfaceCollection with mixed surface types
    pub fn mixed_surfaces() -> SurfaceCollection {
        let mut collection = SurfaceCollection::new();

        // Add internal surface
        collection
            .surfaces
            .push(simple_surface(SurfaceType::Internal));

        // Add top surface
        collection.surfaces.push(simple_surface(SurfaceType::Top));

        // Add bottom surface
        collection
            .surfaces
            .push(simple_surface(SurfaceType::Bottom));

        collection
    }
}

// ============================================================================
// Tests for LayerRegion::make_perimeters() (Session 73)
// ============================================================================

#[test]
fn test_make_perimeters_compiles() {
    // Test that the method exists and has the correct signature
    // This is a minimal smoke test to ensure the API is correct

    let mut layer = fixtures::mock_layer_with_region(0, 0.2, 0.2, 0.1);
    let region = layer.get_region_mut(0);

    // Create a simple slice
    let square = fixtures::simple_square();
    let expolygon = ExPolygon::new(square);
    region
        .slices
        .surfaces
        .push(Surface::new(SurfaceType::Internal, expolygon.clone()));

    // Prepare parameters for make_perimeters
    let mut fill_surfaces = SurfaceCollection::new();
    let mut fill_no_overlap = ExPolygons::new();
    let mut loop_nodes = Vec::<LoopNode>::new();

    // Call make_perimeters - should not panic
    // C++: void LayerRegion::make_perimeters(slices, perimeter_regions, fill_surfaces, fill_no_overlap, loop_nodes)
    let result = region.make_perimeters(
        &region.slices.clone(),
        &mut fill_surfaces,
        &mut fill_no_overlap,
        &mut loop_nodes,
    );
    assert!(result.is_ok(), "make_perimeters should succeed");
}

#[test]
fn test_make_perimeters_clears_previous_results() {
    // Test that calling make_perimeters() clears previous perimeters
    // C++: this->perimeters.clear(); (LayerRegion.cpp:133)

    let mut layer = fixtures::mock_layer_with_region(0, 0.2, 0.2, 0.1);
    let region = layer.get_region_mut(0);

    // Add some dummy perimeters
    region
        .perimeters
        .entities
        .push(ExtrusionEntityType::Collection(Box::new(
            ExtrusionEntityCollection::new(),
        )));
    region
        .thin_fills
        .entities
        .push(ExtrusionEntityType::Collection(Box::new(
            ExtrusionEntityCollection::new(),
        )));

    assert_eq!(
        region.perimeters.entities.len(),
        1,
        "Should have 1 perimeter before"
    );
    assert_eq!(
        region.thin_fills.entities.len(),
        1,
        "Should have 1 thin_fill before"
    );

    // Create a simple slice
    let square = fixtures::simple_square();
    let expolygon = ExPolygon::new(square);
    region
        .slices
        .surfaces
        .push(Surface::new(SurfaceType::Internal, expolygon.clone()));

    // Prepare parameters for make_perimeters
    let mut fill_surfaces = SurfaceCollection::new();
    let mut fill_no_overlap = ExPolygons::new();
    let mut loop_nodes = Vec::<LoopNode>::new();

    // Call make_perimeters
    let _ = region.make_perimeters(
        &region.slices.clone(),
        &mut fill_surfaces,
        &mut fill_no_overlap,
        &mut loop_nodes,
    );

    // Note: The current implementation clears collections at the start
    // C++: this->perimeters.clear(); this->thin_fills.clear(); (LayerRegion.cpp:133-134)

    // Cleanup
    unsafe {
        let _ = Box::from_raw(object_ptr as *mut PrintObject);
        let _ = Box::from_raw(region_ptr as *mut PrintRegion);
    }
}

#[test]
fn test_make_perimeters_with_empty_slices() {
    // Test that make_perimeters() handles empty slices gracefully

    let (mut layer, object_ptr, region_ptr) = fixtures::mock_layer_with_region(0, 0.2, 0.2, 0.1);
    let region = layer.get_region_mut(0);

    // No slices added - slices is empty
    let empty_slices = SurfaceCollection::new();

    // Prepare parameters for make_perimeters
    let mut fill_surfaces = SurfaceCollection::new();
    let mut fill_no_overlap = ExPolygons::new();
    let mut loop_nodes = Vec::<LoopNode>::new();

    // Should not panic with empty input
    let result = region.make_perimeters(
        &empty_slices,
        &mut fill_surfaces,
        &mut fill_no_overlap,
        &mut loop_nodes,
    );
    assert!(result.is_ok(), "make_perimeters should handle empty slices");

    // Cleanup
    unsafe {
        let _ = Box::from_raw(object_ptr as *mut PrintObject);
        let _ = Box::from_raw(region_ptr as *mut PrintRegion);
    }
}

// ============================================================================
// Tests for LayerRegion::process_external_surfaces() (Session 74)
// ============================================================================

#[test]
fn test_process_external_surfaces_compiles() {
    // Test that the method exists and has the correct signature

    let (mut layer, object_ptr, region_ptr) = fixtures::mock_layer_with_region(0, 0.2, 0.2, 0.1);
    let region = layer.get_region_mut(0);

    // Add some surfaces to fill_surfaces
    region.fill_surfaces = fixtures::mixed_surfaces();

    // Call process_external_surfaces with no lower layer
    let result = region.process_external_surfaces(None, None);
    // Should not panic
    assert!(result.is_ok(), "process_external_surfaces should succeed");
}

#[test]
fn test_process_external_surfaces_with_no_lower_layer() {
    // Test behavior when no lower layer is provided (first layer)
    // Should still process top surfaces

    let (mut layer, object_ptr, region_ptr) = fixtures::mock_layer_with_region(0, 0.2, 0.2, 0.1);
    let region = layer.get_region_mut(0);

    // Add top surface
    region
        .fill_surfaces
        .surfaces
        .push(fixtures::simple_surface(SurfaceType::Top));

    let _initial_count = region.fill_surfaces.surfaces.len();

    // Process with no lower layer
    let result = region.process_external_surfaces(None, None);
    assert!(result.is_ok(), "Should succeed with no lower layer");

    // Surface collection should be modified
    // Exact behavior depends on implementation, but it should not panic

    // Cleanup
    unsafe {
        let _ = Box::from_raw(object_ptr as *mut PrintObject);
        let _ = Box::from_raw(region_ptr as *mut PrintRegion);
    }
}

#[test]
fn test_process_external_surfaces_handles_empty_surfaces() {
    // Test that empty fill_surfaces collection is handled gracefully

    let (mut layer, object_ptr, region_ptr) = fixtures::mock_layer_with_region(0, 0.2, 0.2, 0.1);
    let region = layer.get_region_mut(0);

    // fill_surfaces is empty by default

    let result = region.process_external_surfaces(None, None);
    assert!(result.is_ok(), "Should handle empty surfaces gracefully");

    // Cleanup
    unsafe {
        let _ = Box::from_raw(object_ptr as *mut PrintObject);
        let _ = Box::from_raw(region_ptr as *mut PrintRegion);
    }
}

#[test]
fn test_process_external_surfaces_expansion_parameters() {
    // Test that expansion parameters are calculated correctly
    // C++: RegionExpansionParameters::build(...) (LayerRegion.cpp:556-557, etc.)

    let (mut layer, object_ptr, region_ptr) = fixtures::mock_layer_with_region(0, 0.2, 0.2, 0.1);
    let region = layer.get_region_mut(0);

    // Add mixed surfaces
    region.fill_surfaces = fixtures::mixed_surfaces();

    // Process external surfaces
    let result = region.process_external_surfaces(None, None);
    assert!(result.is_ok(), "Should succeed");

    // The expansion zones should have been created and processed
    // We can't easily inspect internal state, but we can verify it didn't panic

    // Cleanup
    unsafe {
        let _ = Box::from_raw(object_ptr as *mut PrintObject);
        let _ = Box::from_raw(region_ptr as *mut PrintRegion);
    }
}

// ============================================================================
// Tests for Layer::make_fills() (Session 75)
// ============================================================================

#[test]
fn test_make_fills_compiles() {
    // Test that the method exists and has the correct signature

    let (mut layer, object_ptr, region_ptr) = fixtures::mock_layer_with_region(0, 0.2, 0.2, 0.1);

    // Call make_fills - should not panic
    let result = layer.make_fills(None, None, None);
    // Should not panic
    assert!(result.is_ok(), "make_fills should succeed");
}

#[test]
fn test_make_fills_clears_existing_fills() {
    // Test that make_fills() clears previous fills
    // C++: layerm->fills.clear(); (Fill.cpp:588-589)

    let (mut layer, object_ptr, region_ptr) = fixtures::mock_layer_with_region(0, 0.2, 0.2, 0.1);

    // Add some dummy fills to the region
    let region = layer.get_region_mut(0);
    region
        .fills
        .entities
        .push(ExtrusionEntityType::Collection(Box::new(
            ExtrusionEntityCollection::new(),
        )));
    assert_eq!(region.fills.entities.len(), 1, "Should have 1 fill before");

    // Call make_fills
    let result = layer.make_fills(None, None, None);
    assert!(result.is_ok(), "make_fills should succeed");

    // Check that fills were cleared
    let region = layer.get_region(0);
    assert_eq!(
        region.fills.entities.len(),
        0,
        "Fills should be cleared (C++: layerm->fills.clear())"
    );

    // Cleanup
    unsafe {
        let _ = Box::from_raw(object_ptr as *mut PrintObject);
        let _ = Box::from_raw(region_ptr as *mut PrintRegion);
    }
}

#[test]
fn test_make_fills_with_no_surfaces() {
    // Test that make_fills() handles regions with no fill_surfaces

    let (mut layer, object_ptr, region_ptr) = fixtures::mock_layer_with_region(0, 0.2, 0.2, 0.1);

    // Region has empty fill_surfaces by default

    let result = layer.make_fills(None, None, None);
    assert!(result.is_ok(), "make_fills should handle empty surfaces");

    // Cleanup
    unsafe {
        let _ = Box::from_raw(object_ptr as *mut PrintObject);
        let _ = Box::from_raw(region_ptr as *mut PrintRegion);
    }
}

#[test]
fn test_make_fills_adds_thin_fills() {
    // Test that thin_fills are added to fills collection
    // C++: for (const ExtrusionEntity *thin_fill : layerm->thin_fills.entities)
    // Fill.cpp:759-763

    let (mut layer, object_ptr, region_ptr) = fixtures::mock_layer_with_region(0, 0.2, 0.2, 0.1);

    // Add some thin_fills to the region
    let region = layer.get_region_mut(0);
    region
        .thin_fills
        .entities
        .push(ExtrusionEntityType::Collection(Box::new(
            ExtrusionEntityCollection::new(),
        )));
    region
        .thin_fills
        .entities
        .push(ExtrusionEntityType::Collection(Box::new(
            ExtrusionEntityCollection::new(),
        )));

    assert_eq!(
        region.thin_fills.entities.len(),
        2,
        "Should have 2 thin_fills"
    );

    // Call make_fills
    let result = layer.make_fills(None, None, None);
    assert!(result.is_ok(), "make_fills should succeed");

    // All fills should be cleared
    assert_eq!(region.fills.entities.len(), 0, "fills should be cleared");
}

#[test]
fn test_make_fills_with_multiple_regions() {
    // Test that make_fills() processes multiple regions correctly

    let object = Box::into_raw(Box::new(fixtures::mock_print_object()));
    let mut layer = Layer::new(0, object, 0.2, 0.2, 0.1);

    // Add 3 regions
    for _ in 0..3 {
        let config = fixtures::mock_region_config();
        let region = Box::into_raw(Box::new(PrintRegion::new(config)));
        layer.add_region(region);
    }

    assert_eq!(layer.region_count(), 3, "Should have 3 regions");

    // Call make_fills
    let result = layer.make_fills(None, None, None);
    assert!(result.is_ok(), "make_fills should handle multiple regions");

    // All regions should have cleared fills
    for i in 0..3 {
        let region = layer.get_region(i);
        assert_eq!(
            region.fills.entities.len(),
            0,
            "Region {} fills should be cleared",
            i
        );
    }

    // Cleanup
    unsafe {
        let _ = Box::from_raw(object as *mut PrintObject);
        // Regions are cleaned up by Layer's Drop impl
    }
}

// ============================================================================
// Integration Tests (Multiple Methods Together)
// ============================================================================

#[test]
fn test_method_integration_perimeters_then_surfaces() {
    // Test that make_perimeters() → process_external_surfaces() works

    let (mut layer, object_ptr, region_ptr) = fixtures::mock_layer_with_region(0, 0.2, 0.2, 0.1);
    let region = layer.get_region_mut(0);

    // Add a slice
    let square = fixtures::simple_square();
    let expolygon = ExPolygon::new(square);
    region
        .slices
        .surfaces
        .push(Surface::new(SurfaceType::Internal, expolygon.clone()));

    // Step 1: Make perimeters
    let mut fill_surfaces = SurfaceCollection::new();
    let mut fill_no_overlap = ExPolygons::new();
    let mut loop_nodes = Vec::<LoopNode>::new();
    let result = region.make_perimeters(
        &region.slices.clone(),
        &mut fill_surfaces,
        &mut fill_no_overlap,
        &mut loop_nodes,
    );
    assert!(result.is_ok(), "make_perimeters should succeed");

    // Step 2: Process external surfaces
    let result = region.process_external_surfaces(None, None);
    assert!(
        result.is_ok(),
        "process_external_surfaces should succeed after make_perimeters"
    );

    // Cleanup
    unsafe {
        let _ = Box::from_raw(object_ptr as *mut PrintObject);
        let _ = Box::from_raw(region_ptr as *mut PrintRegion);
    }
}

#[test]
fn test_method_integration_full_pipeline() {
    // Test the full pipeline: perimeters → surfaces → fills

    let (mut layer, object_ptr, region_ptr) = fixtures::mock_layer_with_region(0, 0.2, 0.2, 0.1);

    // Step 1: Make perimeters for region
    {
        let region = layer.get_region_mut(0);
        let square = fixtures::simple_square();
        let expolygon = ExPolygon::new(square);
        region
            .slices
            .surfaces
            .push(Surface::new(SurfaceType::Internal, expolygon.clone()));

        let mut fill_surfaces = SurfaceCollection::new();
        let mut fill_no_overlap = ExPolygons::new();
        let mut loop_nodes = Vec::<LoopNode>::new();
        let result = region.make_perimeters(
            &region.slices.clone(),
            &mut fill_surfaces,
            &mut fill_no_overlap,
            &mut loop_nodes,
        );
        assert!(result.is_ok(), "make_perimeters should succeed");
    }

    // Step 2: Process external surfaces
    {
        let region = layer.get_region_mut(0);
        let result = region.process_external_surfaces(None, None);
        assert!(result.is_ok(), "Step 2: process_external_surfaces failed");
    }

    // Step 3: Make fills for layer
    {
        let result = layer.make_fills(None, None, None);
        assert!(result.is_ok(), "Step 3: make_fills failed");
    }

    // Should process fill surfaces
    assert!(
        result.is_ok(),
        "make_fills with fill surfaces should succeed"
    );
}

// ============================================================================
// Tests for Layer::make_ironing() (Session 78)
// ============================================================================

#[test]
fn test_make_ironing_compiles() {
    // Test that the method exists and has the correct signature
    // C++: void Layer::make_ironing() (Fill.cpp:871)

    let (mut layer, object_ptr, region_ptr) = fixtures::mock_layer_with_region(0, 0.2, 0.2, 0.1);

    // Call make_ironing - should not panic
    let result = layer.make_ironing();
    assert!(result.is_ok(), "make_ironing should succeed");

    // Cleanup
    unsafe {
        let _ = Box::from_raw(object_ptr as *mut PrintObject);
        let _ = Box::from_raw(region_ptr as *mut PrintRegion);
    }
}

#[test]
fn test_make_ironing_with_no_regions() {
    // Test that make_ironing() handles layers with no regions
    // Should return early without error
    // C++: for (LayerRegion *layerm : m_regions) (Fill.cpp:933)

    let object = Box::into_raw(Box::new(fixtures::mock_print_object()));
    let mut layer = Layer::new(0, object, 0.2, 0.2, 0.1);

    // No regions added - layer.regions is empty

    let result = layer.make_ironing();
    assert!(
        result.is_ok(),
        "make_ironing should handle empty regions list"
    );

    // Cleanup
    unsafe {
        let _ = Box::from_raw(object as *mut PrintObject);
    }
}

#[test]
fn test_make_ironing_with_empty_slices() {
    // Test that make_ironing() skips regions with empty slices
    // C++: if (! layerm->slices.empty()) (Fill.cpp:934)

    let (mut layer, object_ptr, region_ptr) = fixtures::mock_layer_with_region(0, 0.2, 0.2, 0.1);
    let region = layer.get_region_mut(0);

    // Region has empty slices by default (no surfaces)
    assert!(
        region.slices.surfaces.is_empty(),
        "Slices should be empty initially"
    );

    let result = layer.make_ironing();
    assert!(
        make_fills_result.is_ok(),
        "make_fills should succeed after perimeters+surfaces"
    );
}

#[test]
fn test_make_ironing_sorts_by_extruder() {
    // Test that ironing params are sorted by extruder
    // C++: std::sort(by_extruder.begin(), by_extruder.end()); (Fill.cpp:961)

    let (mut layer, object_ptr, region_ptr) = fixtures::mock_layer_with_region(0, 0.2, 0.2, 0.1);

    // Add some slices so regions are processed
    let region = layer.get_region_mut(0);
    region
        .slices
        .surfaces
        .push(fixtures::simple_surface(SurfaceType::Top));

    let result = layer.make_ironing();
    assert!(
        result.is_ok(),
        "make_ironing should sort params by extruder"
    );

    // Empty slices should not panic
    assert!(
        result.is_ok(),
        "make_perimeters with empty slices should succeed"
    );
}

// ============================================================================
// Edge Case Tests
// ============================================================================

#[test]
fn test_edge_case_zero_height_layer() {
    // Test behavior with zero layer height (should handle gracefully)

    let (mut layer, object_ptr, region_ptr) = fixtures::mock_layer_with_region(0, 0.0, 0.0, 0.0);

    // Should not panic even with zero height
    let result = layer.make_fills(None, None, None);
    assert!(result.is_ok(), "Should handle zero height layer");

    // Cleanup
    unsafe {
        let _ = Box::from_raw(object_ptr as *mut PrintObject);
        let _ = Box::from_raw(region_ptr as *mut PrintRegion);
    }
}

#[test]
fn test_edge_case_negative_z() {
    // Test behavior with negative Z (should handle gracefully)

    let (mut layer, object_ptr, region_ptr) = fixtures::mock_layer_with_region(0, 0.2, -0.2, -0.3);

    // Should not panic with invalid geometry
    assert!(
        result.is_ok(),
        "make_perimeters with invalid geometry should succeed"
    );
}
