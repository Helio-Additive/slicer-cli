//! Tests for `transform_gcode` — the wipe-tower local→absolute coordinate
//! rewrite (port of BambuStudio GCode.cpp:298 `transform_gcode`).
//!
//! Lives as an integration target because the crate's in-lib `#[cfg(test)]`
//! target does not currently compile (see memory / PARITY_STATUS).

use slicer::gcode::wipe_tower::Vec2f;
use slicer::gcode::wipe_tower_integration::transform_gcode;

fn xy(line: &str) -> (f32, f32) {
    let mut x = f32::NAN;
    let mut y = f32::NAN;
    for tok in line.split_whitespace() {
        if let Some(v) = tok.strip_prefix('X') {
            x = v.parse().unwrap();
        } else if let Some(v) = tok.strip_prefix('Y') {
            y = v.parse().unwrap();
        }
    }
    (x, y)
}

#[test]
fn pure_translation_shifts_xy_keeps_e() {
    let g = "G1 X0.500 Y0.750 E1.0867\n";
    let out = transform_gcode(g, Vec2f::new(0.0, 0.0), Vec2f::new(100.0, 200.0), 0.0);
    let (x, y) = xy(&out);
    assert!((x - 100.5).abs() < 1e-3, "x={x} out={out}");
    assert!((y - 200.75).abs() < 1e-3, "y={y} out={out}");
    assert!(out.contains("E1.0867"), "E preserved: {out}");
}

#[test]
fn non_g1_lines_pass_through() {
    let g = "; Tool change from T0 to T1\nT1\nG92 E0\n";
    let out = transform_gcode(g, Vec2f::new(0.0, 0.0), Vec2f::new(50.0, 60.0), 0.0);
    assert!(out.contains("; Tool change from T0 to T1"));
    assert!(out.contains("\nT1\n"));
    assert!(out.contains("G92 E0"));
}

#[test]
fn rotation_90_degrees() {
    // (1,0) rotated +90° -> (0,1); no translation.
    let g = "G1 X1.000 Y0.000 E0.5\n";
    let out = transform_gcode(
        g,
        Vec2f::new(0.0, 0.0),
        Vec2f::new(0.0, 0.0),
        std::f32::consts::FRAC_PI_2,
    );
    let (x, y) = xy(&out);
    assert!(x.abs() < 1e-3, "x≈0, got {x}");
    assert!((y - 1.0).abs() < 1e-3, "y≈1, got {y}");
}

#[test]
fn omitted_axis_carries_previous_position() {
    // Second move omits Y — it must keep the prior Y (0.75), transformed.
    let g = "G1 X0.500 Y0.750 E1.0\nG1 X2.500 E2.0\n";
    let out = transform_gcode(g, Vec2f::new(0.0, 0.0), Vec2f::new(10.0, 20.0), 0.0);
    let second = out.lines().nth(1).unwrap();
    let (x, _) = xy(second);
    assert!((x - 12.5).abs() < 1e-3, "second X transformed: {second}");
}

#[test]
fn preserves_trailing_newline_structure() {
    let g = "G1 X0 Y0 E1\nT1\n";
    let out = transform_gcode(g, Vec2f::new(0.0, 0.0), Vec2f::new(0.0, 0.0), 0.0);
    assert!(out.ends_with('\n'));
    assert!(!out.ends_with("\n\n"));
}
