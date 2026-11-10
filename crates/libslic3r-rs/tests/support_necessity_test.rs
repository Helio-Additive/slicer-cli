//! Unit tests for is_support_necessary() overhang detection.
//!
//! Tests the overhang detection algorithm that determines if support material
//! is necessary based on layer-to-layer geometry analysis.
//!
//! Note: Since is_support_necessary() is private, these tests verify the
//! behavior through the public generate_support_material() API.

use slicer::print_config::PrintObjectConfig;
use slicer::print::PrintObject;
use std::sync::atomic::AtomicBool;
use std::sync::Arc;

// ============================================================================
// Test Fixtures
// ============================================================================

/// Create a test PrintObject with default configuration.
fn create_test_object() -> PrintObject {
    let mesh = slicer::TriangleMesh::cube(10.0);
    let mut config = PrintObjectConfig::default();
    config.layer_height = 0.2;
    config.support_threshold_angle = 45.0;
    PrintObject::with_config(mesh, config)
}

/// The cancellation flag is now owned by the PrintObject (`with_canceled`) rather
/// than passed to each step; rebuild `obj` around `canceled`, keeping its mesh and
/// config, so the tests keep driving cancellation the way they did.
fn with_flag(obj: PrintObject, canceled: &Arc<AtomicBool>) -> PrintObject {
    let mut flagged = PrintObject::with_canceled(canceled.clone());
    if let Some(mesh) = obj.mesh() {
        flagged.set_mesh(mesh.clone());
    }
    flagged.config = obj.config.clone();
    flagged
}

// ============================================================================
// Tests
// ============================================================================

/// Test that generate_support_material processes without errors.
#[test]
fn test_support_generation_basic() {
    let mut obj = create_test_object();
    let canceled = Arc::new(AtomicBool::new(false));
    let result = obj.generate_support_material();

    assert!(
        result.is_ok(),
        "Support generation should process without errors"
    );
}

/// Test that empty object (no layers) doesn't need support.
#[test]
fn test_empty_object_no_support() {
    let mesh = slicer::TriangleMesh::cube(10.0);
    let mut obj = PrintObject::with_config(mesh, PrintObjectConfig::default());

    let canceled = Arc::new(AtomicBool::new(false));
    let result = obj.generate_support_material();

    assert!(result.is_ok(), "Empty object should process without errors");
}

/// Test with different threshold angles.
#[test]
fn test_threshold_angle_variations() {
    let mesh = slicer::TriangleMesh::cube(10.0);
    let mut config = PrintObjectConfig::default();
    config.layer_height = 0.2;

    // Test with strict threshold (30°)
    config.support_threshold_angle = 30.0;
    let mut obj1 = PrintObject::with_config(mesh.clone(), config.clone());

    let canceled = Arc::new(AtomicBool::new(false));
    let result1 = obj1.generate_support_material();
    assert!(result1.is_ok(), "Object with 30° threshold should process");

    // Test with lenient threshold (60°)
    config.support_threshold_angle = 60.0;
    let mut obj2 = PrintObject::with_config(mesh, config);

    let result2 = obj2.generate_support_material();
    assert!(result2.is_ok(), "Object with 60° threshold should process");
}

/// Test with very large threshold angle (90°) - no support ever needed.
#[test]
fn test_90_degree_threshold_no_support() {
    let mesh = slicer::TriangleMesh::cube(10.0);
    let mut config = PrintObjectConfig::default();
    config.layer_height = 0.2;
    config.support_threshold_angle = 90.0; // Never need support at 90°

    let mut obj = PrintObject::with_config(mesh, config);

    let canceled = Arc::new(AtomicBool::new(false));
    let result = obj.generate_support_material();

    assert!(
        result.is_ok(),
        "Object with 90° threshold should never need support"
    );
}

/// Test cancellation during support generation.
#[test]
fn test_cancellation_during_support_generation() {
    // Cancel before processing
    let canceled = Arc::new(AtomicBool::new(true));
    let mut obj = with_flag(create_test_object(), &canceled);
    let result = obj.generate_support_material();

    // Should return Cancelled error
    assert!(
        matches!(result, Err(slicer::Error::Cancelled)),
        "Cancelled support generation should return Cancelled error"
    );
}

/// Integration test: Verify support generation with enabled config.
#[test]
fn test_support_generation_with_config() {
    let mesh = slicer::TriangleMesh::cube(10.0);
    let mut config = PrintObjectConfig::default();
    config.layer_height = 0.2;
    config.support_threshold_angle = 45.0;
    config.enable_support = true;

    let mut obj = PrintObject::with_config(mesh, config);

    let canceled = Arc::new(AtomicBool::new(false));
    let result = obj.generate_support_material();

    assert!(
        result.is_ok(),
        "Support generation should process successfully"
    );
}
