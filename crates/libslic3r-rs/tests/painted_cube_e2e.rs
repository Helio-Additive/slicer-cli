//! End-to-end multicolour smoke: slice the painted-cube 3MF fixture through
//! the full Tier-1 chain (paint decode → painted regions → MMU segmentation →
//! apply → slice → gcode) and assert the multi-material plumbing held.
//!
//! The fixture (tests/data/painted_cube.3mf) is a 20mm cube whose +X face is
//! painted extruder 1 (`paint_color="4"`) and -X face extruder 2 (`"8"`), with
//! an embedded 2-filament BambuStudio project_settings.config (classic walls).
//!
//! Toolchange emission (campaign layers 5-6) is not wired yet, so the gcode is
//! single-tool — this test guards the segmentation pipeline, the `; filament: N`
//! header, and that fill/walls survive the multi-region split.

use std::path::PathBuf;

#[test]
fn painted_cube_slices_end_to_end() {
    let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let fixture = manifest.join("tests/data/painted_cube.3mf");
    let out = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("painted_cube.gcode");

    slicer::app_slice::slice_3mf_to_gcode(&fixture, None, &out)
        .expect("painted cube should slice");

    let gcode = std::fs::read_to_string(&out).expect("read gcode");
    let count = |tag: &str| gcode.matches(tag).count();

    // 2-filament config must surface in the header (num_filaments chain).
    assert!(
        gcode.contains("; filament: 2\n"),
        "expected '; filament: 2' header"
    );
    // The multi-region split must not break walls/fill.
    assert!(count("; FEATURE: Outer wall") > 0, "outer walls missing");
    assert!(
        count("; FEATURE: Sparse infill") > 0 || count("; FEATURE: Internal solid infill") > 0,
        "fill missing after painted split"
    );
    assert!(gcode.len() > 100_000, "suspiciously small gcode");

    // Toolchange emission (campaign layers 5-6, Tier-1 bare T-commands): the
    // two painted faces force per-layer tool alternation. The start-gcode
    // template carries a couple of its own T lines, so require clearly more
    // than that — one change per painted layer band at minimum.
    let toolchanges = gcode
        .lines()
        .filter(|l| l.starts_with("T1") && l.contains("tool change"))
        .count();
    assert!(
        toolchanges >= 10,
        "expected per-layer T1 toolchanges from the painted split, got {toolchanges}"
    );

    // CONFIG_BLOCK must echo FULL per-filament arrays on multi-material
    // prints — BambuStudio's gcode preview indexes filament_colour /
    // nozzle_temperature by tool id and CRASHES on a 1-element echo next to
    // T1 in the body (observed: loading the Majora rust gcode killed
    // BambuStudio). coStrings join with ';', numeric vectors with ','.
    assert!(
        gcode.contains("; filament_colour = #00AE42;#E28C00"),
        "filament_colour must echo both filaments ;-joined"
    );
    assert!(
        gcode.contains("; filament_diameter = 1.75,1.75"),
        "filament_diameter must echo both filaments ,-joined"
    );
}
