//! Integration tests for Print::process() orchestration.
//!
//! Tests processing state tracking, cancellation, and full pipeline execution.

use slicer::config::PrintObjectConfig;
use slicer::print::{Print, PrintObject, PrintObjectState};
use std::sync::atomic::AtomicBool;
use std::sync::Arc;

/// Test that Print::process() runs end-to-end without errors.
#[test]
fn test_print_process_basic() {
    // Create a simple print with one object
    let mut print = Print::new();

    // For now, just test that the process method exists and can be called
    // with an empty print (should succeed immediately)
    let result = print.process();
    assert!(result.is_ok(), "Empty print should process successfully");
}

/// Test processing state tracking on PrintObject.
#[test]
fn test_print_object_state_tracking() {
    // Create a PrintObject
    let mesh = create_test_cube_mesh();
    let mut obj = PrintObject::new(mesh);

    // Initial state should be empty
    assert!(!obj.is_step_done(PrintObjectState::PERIMETERS_DONE));
    assert!(!obj.is_step_done(PrintObjectState::INFILL_DONE));
    assert!(!obj.is_step_done(PrintObjectState::IRONING_DONE));

    // Simulate processing steps
    let canceled = Arc::new(AtomicBool::new(false));

    // After make_perimeters, PERIMETERS_DONE should be set
    let result = obj.make_perimeters(&canceled);
    assert!(result.is_ok());
    assert!(obj.is_step_done(PrintObjectState::PERIMETERS_DONE));
    assert!(!obj.is_step_done(PrintObjectState::INFILL_DONE));

    // Clear state
    obj.clear_state();
    assert!(!obj.is_step_done(PrintObjectState::PERIMETERS_DONE));
}

/// Test cancellation during processing.
#[test]
fn test_print_cancellation() {
    let mut print = Print::new();

    // Cancel before processing
    print.cancel();

    // Process should return Cancelled error
    let result = print.process();
    assert!(result.is_err(), "Cancelled print should fail");
    if let Err(e) = result {
        assert!(
            matches!(e, slicer::Error::Cancelled),
            "Expected Cancelled error, got: {:?}",
            e
        );
    }
}

/// Test that cancellation flag is checked during PrintObject methods.
#[test]
fn test_printobject_cancellation() {
    let mesh = create_test_cube_mesh();
    let mut obj = PrintObject::new(mesh);

    // Create cancellation flag and set it
    let canceled = Arc::new(AtomicBool::new(true));

    // Each method should return Cancelled error
    assert!(matches!(
        obj.make_perimeters(&canceled),
        Err(slicer::Error::Cancelled)
    ));
    assert!(matches!(
        obj.prepare_infill(&canceled),
        Err(slicer::Error::Cancelled)
    ));
    assert!(matches!(
        obj.infill(&canceled),
        Err(slicer::Error::Cancelled)
    ));
    assert!(matches!(
        obj.ironing(&canceled),
        Err(slicer::Error::Cancelled)
    ));
    assert!(matches!(
        obj.generate_support_material(&canceled),
        Err(slicer::Error::Cancelled)
    ));
    assert!(matches!(
        obj.detect_overhangs_for_lift(&canceled),
        Err(slicer::Error::Cancelled)
    ));
}

/// Test status callback functionality.
#[test]
fn test_status_callback() {
    let mut print = Print::new();

    // Track status updates
    let status_updates = Arc::new(std::sync::Mutex::new(Vec::new()));
    let status_updates_clone = Arc::clone(&status_updates);

    // Set callback
    print.set_status_callback(move |percent, message| {
        status_updates_clone
            .lock()
            .unwrap()
            .push((percent, message.to_string()));
    });

    // Process (should trigger status callbacks)
    let _ = print.process();

    // Verify we got status updates
    let updates = status_updates.lock().unwrap();
    assert!(!updates.is_empty(), "Should have received status updates");

    // Verify we got the initial and final status
    assert!(
        updates.iter().any(|(p, _)| *p == 0),
        "Should have 0% status"
    );
    assert!(
        updates.iter().any(|(p, _)| *p == 100),
        "Should have 100% status"
    );

    // Verify status messages contain expected phase names
    let messages: Vec<String> = updates.iter().map(|(_, m)| m.clone()).collect();
    let all_messages = messages.join(" | ");
    assert!(
        all_messages.contains("perimeter"),
        "Should mention perimeters in status"
    );
    assert!(
        all_messages.contains("infill"),
        "Should mention infill in status"
    );
}

/// Test Print::is_canceled() and throw_if_canceled().
#[test]
fn test_print_cancellation_checking() {
    let print = Print::new();

    // Initially not canceled
    assert!(!print.is_canceled());
    assert!(print.throw_if_canceled().is_ok());

    // After cancellation
    print.cancel();
    assert!(print.is_canceled());
    assert!(matches!(
        print.throw_if_canceled(),
        Err(slicer::Error::Cancelled)
    ));
}

/// Test that state tracking persists across multiple operations.
#[test]
fn test_state_persistence() {
    let mesh = create_test_cube_mesh();
    let mut obj = PrintObject::new(mesh);
    let canceled = Arc::new(AtomicBool::new(false));

    // Run perimeters
    let _ = obj.make_perimeters(&canceled);
    assert!(obj.is_step_done(PrintObjectState::PERIMETERS_DONE));

    // Run infill (should preserve perimeters state)
    let _ = obj.infill(&canceled);
    assert!(obj.is_step_done(PrintObjectState::PERIMETERS_DONE));
    assert!(obj.is_step_done(PrintObjectState::PREPARE_INFILL_DONE));
    assert!(obj.is_step_done(PrintObjectState::INFILL_DONE));

    // Run ironing (should preserve all previous states)
    let _ = obj.ironing(&canceled);
    assert!(obj.is_step_done(PrintObjectState::PERIMETERS_DONE));
    assert!(obj.is_step_done(PrintObjectState::INFILL_DONE));
    assert!(obj.is_step_done(PrintObjectState::IRONING_DONE));
}

// ============================================================================
// Test Fixtures
// ============================================================================

/// Create a simple cube mesh for testing.
///
/// Creates a 10mm × 10mm × 10mm cube centered at origin.
fn create_test_cube_mesh() -> slicer::mesh::TriangleMesh {
    // Use the built-in cube() constructor
    slicer::mesh::TriangleMesh::cube(10.0)
}

/// Load a test STL file if available, otherwise use cube mesh.
#[allow(dead_code)]
fn _load_test_mesh() -> slicer::mesh::TriangleMesh {
    // Fallback to generated cube
    create_test_cube_mesh()
}

/// Create a sliced PrintObject ready for processing.
///
/// This is a more complete fixture that includes slicing.
#[allow(dead_code)]
fn _create_sliced_print_object() -> PrintObject {
    let mesh = create_test_cube_mesh();
    let config = PrintObjectConfig::default();

    // Create PrintObject without slicing for now
    // TODO: Add slicing once slice_mesh is exported
    let obj = PrintObject::with_config(mesh, config);

    obj
}
