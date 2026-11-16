//! Integration tests for Print::process() orchestration.
//!
//! Tests processing state tracking, cancellation, and full pipeline execution.

// API notes (tests updated to the current engine API):
// - `PrintObjectState::*_DONE` bitflags became the `PrintObjectStep` enum.
// - `PrintObject` owns its cancellation flag (`with_canceled`) instead of taking it per call.
// - `Print::process` takes `(slice_time, use_cache)`.
use slicer::print::{Print, PrintObject, PrintObjectStep};
use slicer::print_config::PrintObjectConfig;
use std::sync::atomic::AtomicBool;
use std::sync::Arc;

/// Test that Print::process() runs end-to-end without errors.
#[test]
fn test_print_process_basic() {
    // Create a simple print with one object
    let mut print = Print::new();

    // For now, just test that the process method exists and can be called
    // with an empty print (should succeed immediately)
    let result = print.process(None, false);
    assert!(result.is_ok(), "Empty print should process successfully");
}

/// Test processing state tracking on PrintObject.
#[test]
fn test_print_object_state_tracking() {
    // Create a PrintObject inside a Print, as C++ does (see print_with_cube).
    let mut print = print_with_cube();
    let obj = &mut print.objects_mut()[0];

    // Initial state should be empty
    assert!(!obj.is_step_done(PrintObjectStep::Perimeters));
    assert!(!obj.is_step_done(PrintObjectStep::Infill));
    assert!(!obj.is_step_done(PrintObjectStep::Ironing));

    // Simulate processing steps

    // After make_perimeters, PERIMETERS_DONE should be set
    let result = obj.make_perimeters();
    assert!(result.is_ok());
    assert!(obj.is_step_done(PrintObjectStep::Perimeters));
    assert!(!obj.is_step_done(PrintObjectStep::Infill));

    // Clear state
    obj.invalidate_all_steps();
    assert!(!obj.is_step_done(PrintObjectStep::Perimeters));
}

/// Test cancellation during processing.
#[test]
fn test_print_cancellation() {
    // An empty print returns before any step, canceled or not: Print::process
    // has `if (m_objects.empty()) return;` ahead of all the work
    // (Print.cpp:2219-2220), so nothing reaches a throw_if_canceled.
    let mut empty = Print::new();
    empty.cancel();
    assert!(empty.process(None, false).is_ok(), "an empty print returns at once");

    // With an object, the first step's set_started throws CanceledException
    // (PrintBase.hpp:154-157), which the port returns as Error::Cancelled.
    let mut print = print_with_cube();
    print.cancel();
    let result = print.process(None, false);
    assert!(
        matches!(result, Err(slicer::Error::Cancelled)),
        "Expected Cancelled error, got: {:?}",
        result
    );
}

/// Test that cancellation flag is checked during PrintObject methods.
#[test]
fn test_printobject_cancellation() {
    let mesh = create_test_cube_mesh();
    let canceled = Arc::new(AtomicBool::new(true));
    let mut obj = PrintObject::with_canceled(canceled.clone());
    obj.set_mesh(mesh);

    // Create cancellation flag and set it

    // Each method should return Cancelled error
    assert!(matches!(
        obj.make_perimeters(),
        Err(slicer::Error::Cancelled)
    ));
    assert!(matches!(
        obj.prepare_infill(),
        Err(slicer::Error::Cancelled)
    ));
    assert!(matches!(
        obj.infill(),
        Err(slicer::Error::Cancelled)
    ));
    assert!(matches!(
        obj.ironing(),
        Err(slicer::Error::Cancelled)
    ));
    assert!(matches!(
        obj.generate_support_material(),
        Err(slicer::Error::Cancelled)
    ));
    // `detect_overhangs_for_lift` no longer returns a `Result` (it is infallible in
    // the current engine), so there is no `Err(Cancelled)` to match; it must still
    // be callable on a cancelled object without panicking.
    obj.detect_overhangs_for_lift();
}

/// Test status callback functionality.
///
/// The statuses come from the steps: PrintObjectSlice.cpp:1098 "Slicing mesh"
/// at 5, PrintObject.cpp:482 "Generating walls" at 15, PrintObject.cpp:909
/// "Generating infill regions" at 25, and so on. An empty print returns at
/// Print.cpp:2219-2220 before any of them, so its callback is never called.
/// (The per-object statuses are not reported by the port's PrintObject yet.)
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

    // Process an empty print
    assert!(print.process(None, false).is_ok());

    // Print.cpp:2219-2220: no object, no step, no status.
    let updates = status_updates.lock().unwrap();
    assert!(updates.is_empty(), "an empty print reports no status, got {:?}", *updates);
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
    let mut print = print_with_cube();
    let obj = &mut print.objects_mut()[0];

    // Run perimeters
    let _ = obj.make_perimeters();
    assert!(obj.is_step_done(PrintObjectStep::Perimeters));

    // Run infill (should preserve perimeters state)
    let _ = obj.infill();
    assert!(obj.is_step_done(PrintObjectStep::Perimeters));
    assert!(obj.is_step_done(PrintObjectStep::PrepareInfill));
    assert!(obj.is_step_done(PrintObjectStep::Infill));

    // Run ironing (should preserve all previous states)
    let _ = obj.ironing();
    assert!(obj.is_step_done(PrintObjectStep::Perimeters));
    assert!(obj.is_step_done(PrintObjectStep::Infill));
    assert!(obj.is_step_done(PrintObjectStep::Ironing));
}

// ============================================================================
// Test Fixtures
// ============================================================================

/// Create a simple cube mesh for testing.
///
/// Creates a 10mm × 10mm × 10mm cube centered at origin.
fn create_test_cube_mesh() -> slicer::TriangleMesh {
    // Use the built-in cube() constructor
    slicer::TriangleMesh::cube(10.0)
}

/// A Print holding one cube object.
///
/// C++ never has a PrintObject on its own: Print::apply creates it together
/// with its PrintObjectRegions (PrintApply.cpp), and the layers' regions point
/// into them. `Print::add_object` is the port's equivalent (it shares the
/// regions, the print config and the cancel flag); a bare `PrintObject` has no
/// regions, and `LayerRegion::region()` stops on the missing hierarchy.
fn print_with_cube() -> Print {
    let mut print = Print::new();
    let mut obj = PrintObject::with_canceled(Arc::new(AtomicBool::new(false)));
    obj.set_mesh(create_test_cube_mesh());
    print.add_object(obj);
    print
}

/// Load a test STL file if available, otherwise use cube mesh.
#[allow(dead_code)]
fn _load_test_mesh() -> slicer::TriangleMesh {
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
