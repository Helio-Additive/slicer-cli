//! Regression test for the Arachne infill-area chain.
//!
//! With `wall_generator=arachne`, the per-layer infill region comes from
//! `WallToolPaths::get_inner_contour()`: LimitedBeadingStrategy inserts a
//! 0-width "wall contour" marker bead, SkeletalTrapezoidation emits it as open
//! polyline segments, and `PolylineStitcher::stitch` (WallToolPaths.cpp:561)
//! chains + closes them so `separateOutInnerContour` can collect the closed
//! polygon into `inner_contour`. Before the stitcher was ported (it was a
//! stub that only partitioned by a pre-existing `is_closed` flag), the marked
//! loop stayed open, `inner_contour` came back empty on every layer, and ALL
//! fill (sparse, internal-solid, top, bottom, bridge) vanished — arachne
//! slices produced hollow walls-only G-code while classic was fine.
//!
//! This test slices a small cube with the arachne wall generator and asserts
//! the fill features exist. It exercises stitch → separate_out_inner_contour →
//! infill_area → fill_surfaces end-to-end.

use std::path::PathBuf;

#[test]
fn arachne_cube_has_infill() {
    let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let repo = manifest.join("../..");
    let stl = repo.join("fixtures/smoke/stl/Cube_25.6.stl");
    let settings = manifest.join("tests/data/cube_arachne_settings.json");
    let out = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("arachne_cube.gcode");

    slicer::app_slice::slice_to_gcode(&stl, &settings, &out)
        .expect("arachne cube slice should succeed");

    let gcode = std::fs::read_to_string(&out).expect("read gcode");
    let count = |tag: &str| gcode.matches(tag).count();

    let sparse = count("; FEATURE: Sparse infill");
    let solid = count("; FEATURE: Internal solid infill");
    let top = count("; FEATURE: Top surface");
    let bottom = count("; FEATURE: Bottom surface");
    let outer = count("; FEATURE: Outer wall");

    assert!(outer > 0, "outer walls missing entirely (slice broken)");
    assert!(
        sparse > 0,
        "no sparse infill — arachne inner-contour chain regressed (stitcher/marker)"
    );
    assert!(solid > 0, "no internal solid infill with arachne walls");
    assert!(top > 0 && bottom > 0, "top/bottom shells missing with arachne walls");
}
