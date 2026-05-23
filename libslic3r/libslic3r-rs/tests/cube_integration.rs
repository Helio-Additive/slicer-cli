//! Cube Integration Tests
//!
//! These tests validate the Rust slicer against BambuStudio reference output
//! for a simple 2.792mm cube (Cube.stl). The cube is small enough that:
//! - All interior layers are solid (no sparse infill)
//! - The topmost layer uses only 1 perimeter (top_one_wall_type = "all top")
//! - Layer heights snap to the 0.2mm grid (14 layers, last at Z=2.8)
//!
//! Reference G-code: data/reference_gcodes/Cube_PLA_5m56s.gcode
//! Reference settings (from G-code header):
//!   layer_height = 0.2, initial_layer_print_height = 0.2
//!   wall_loops = 2, detect_thin_wall = 0
//!   top_shell_layers = 5, bottom_shell_layers = 3
//!   sparse_infill_density = 15%
//!   elefant_foot_compensation = 0.15
//!   top_one_wall_type = all top

use std::collections::HashMap;
use std::fs;
use std::path::PathBuf;

use slicer::config::TopOneWallType;
use slicer::mesh::load_stl;
use slicer::pipeline::{PipelineConfig, PrintPipeline};
use slicer::profiles::SliceConfig;
use slicer::slice::{Slicer, SlicingParams};
use slicer::TriangleMesh;

/// Path to test STL files.
fn test_stls_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("data/test_stls")
}

/// Path to reference G-code files.
fn reference_gcodes_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("data/reference_gcodes")
}

/// Load the Cube STL file.
fn load_cube_stl() -> TriangleMesh {
    let stl_path = test_stls_dir().join("Cube.stl");
    load_stl(&stl_path).expect("Failed to load Cube.stl")
}

/// Load the reference Cube G-code.
fn load_cube_reference_gcode() -> String {
    let gcode_path = reference_gcodes_dir().join("Cube_PLA_5m56s.gcode");
    fs::read_to_string(&gcode_path).expect("Failed to load Cube_PLA_5m56s.gcode")
}

/// Build a PipelineConfig matching the BambuStudio benchy_h2d_pla profile
/// (which is also the profile used for the Cube reference G-code).
fn cube_pipeline_config() -> PipelineConfig {
    let slice_config = SliceConfig::benchy_reference();
    let data_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("data");
    let registry = slicer::profiles::ProfileRegistry::load_from_directory(&data_dir)
        .expect("Failed to load profile registry");
    slice_config
        .build_pipeline_config(&registry)
        .expect("Failed to build pipeline config")
}

/// Count occurrences of each `; FEATURE: <name>` in G-code text.
fn count_features(gcode: &str) -> HashMap<String, usize> {
    let mut counts = HashMap::new();
    for line in gcode.lines() {
        let trimmed = line.trim();
        if let Some(feature) = trimmed.strip_prefix("; FEATURE: ") {
            *counts.entry(feature.to_string()).or_insert(0) += 1;
        }
    }
    counts
}

/// Count `; CHANGE_LAYER` markers.
fn count_change_layers(gcode: &str) -> usize {
    gcode
        .lines()
        .filter(|l| l.trim() == "; CHANGE_LAYER")
        .count()
}

/// Extract unique sorted Z values from `; Layer N, Z = X.XXX` comment lines.
///
/// We parse layer comments rather than `G1 Z` moves because the writer only
/// emits explicit `G1 Z` when the Z actually changes (e.g. retract lifts),
/// so not every layer produces a standalone `G1 Z<layer_z>` line.
fn extract_layer_z_heights(gcode: &str) -> Vec<f64> {
    let mut zs = Vec::new();
    for line in gcode.lines() {
        let trimmed = line.trim();
        // Match "; Layer N, Z = X.XXX"
        if let Some(rest) = trimmed.strip_prefix("; Layer ") {
            if let Some(z_part) = rest.split("Z = ").nth(1) {
                let z_str = z_part.split_whitespace().next().unwrap_or(z_part);
                if let Ok(z) = z_str.parse::<f64>() {
                    let z_rounded = (z * 1000.0).round() / 1000.0;
                    zs.push(z_rounded);
                }
            }
        }
    }
    zs
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[test]
fn test_cube_stl_loads() {
    let mesh = load_cube_stl();
    assert!(!mesh.is_empty(), "Cube mesh should not be empty");
    assert_eq!(mesh.triangle_count(), 12, "A cube has 12 triangles");
}

#[test]
fn test_cube_slicing_produces_14_layers() {
    let mesh = load_cube_stl();
    let slicer = Slicer::new(SlicingParams::with_layer_heights(0.2, 0.2));
    let layers = slicer.slice(&mesh).expect("Slicing should succeed");

    assert_eq!(
        layers.len(),
        14,
        "Cube ~2.792mm with 0.2mm layers should produce 14 layers (rounded up to 2.8mm)"
    );

    // First layer starts at 0.0 (or very close)
    assert!(
        layers[0].bottom_z_mm().abs() < 0.01,
        "First layer bottom_z should be ~0"
    );

    // Last layer top_z should be 2.8 (14 × 0.2), NOT 2.792
    let last_top_z = layers[13].top_z_mm();
    assert!(
        (last_top_z - 2.8).abs() < 0.01,
        "Last layer top_z should be 2.8 (grid-snapped), got {:.4}",
        last_top_z
    );
}

#[test]
fn test_cube_layer_heights_on_grid() {
    let mesh = load_cube_stl();
    let slicer = Slicer::new(SlicingParams::with_layer_heights(0.2, 0.2));
    let layers = slicer.slice(&mesh).expect("Slicing should succeed");

    for (i, layer) in layers.iter().enumerate() {
        let expected_top_z = (i + 1) as f64 * 0.2;
        let actual_top_z = layer.top_z_mm();
        assert!(
            (actual_top_z - expected_top_z).abs() < 0.01,
            "Layer {} top_z should be {:.1}, got {:.4}",
            i,
            expected_top_z,
            actual_top_z
        );
    }
}

#[test]
fn test_cube_layers_contiguous() {
    let mesh = load_cube_stl();
    let slicer = Slicer::new(SlicingParams::with_layer_heights(0.2, 0.2));
    let layers = slicer.slice(&mesh).expect("Slicing should succeed");

    for i in 1..layers.len() {
        let prev_top = layers[i - 1].top_z_mm();
        let curr_bottom = layers[i].bottom_z_mm();
        assert!(
            (prev_top - curr_bottom).abs() < 0.001,
            "Layer {} bottom ({:.4}) should equal layer {} top ({:.4})",
            i,
            curr_bottom,
            i - 1,
            prev_top
        );
    }
}

#[test]
fn test_cube_pipeline_feature_counts_match_reference() {
    let mesh = load_cube_stl();
    let config = cube_pipeline_config();
    let mut pipeline = PrintPipeline::new(config);

    let gcode = pipeline.process(&mesh).expect("Pipeline should succeed");
    let gcode_str = gcode.content();

    let features = count_features(gcode_str);

    // Reference feature counts (from Cube_PLA_5m56s.gcode, excluding ; FEATURE: Custom):
    //   Outer wall: 14
    //   Inner wall: 13
    //   Internal solid infill: 12
    //   Top surface: 1
    //   Bottom surface: 1
    assert_eq!(
        features.get("Outer wall").copied().unwrap_or(0),
        14,
        "Expected 14 Outer wall features"
    );
    assert_eq!(
        features.get("Inner wall").copied().unwrap_or(0),
        13,
        "Expected 13 Inner wall features (topmost layer has only outer wall)"
    );
    assert_eq!(
        features.get("Internal solid infill").copied().unwrap_or(0),
        12,
        "Expected 12 Internal solid infill features"
    );
    assert_eq!(
        features.get("Top surface").copied().unwrap_or(0),
        1,
        "Expected 1 Top surface feature on the last layer"
    );
    assert_eq!(
        features.get("Bottom surface").copied().unwrap_or(0),
        1,
        "Expected 1 Bottom surface feature on the first layer"
    );
}

#[test]
fn test_cube_pipeline_change_layer_count() {
    let mesh = load_cube_stl();
    let config = cube_pipeline_config();
    let mut pipeline = PrintPipeline::new(config);

    let gcode = pipeline.process(&mesh).expect("Pipeline should succeed");
    let gcode_str = gcode.content();

    let change_layers = count_change_layers(gcode_str);
    assert_eq!(
        change_layers, 14,
        "Expected 14 CHANGE_LAYER markers (one per layer)"
    );
}

#[test]
fn test_cube_pipeline_z_heights_match_reference() {
    let mesh = load_cube_stl();
    let config = cube_pipeline_config();
    let mut pipeline = PrintPipeline::new(config);

    let gcode = pipeline.process(&mesh).expect("Pipeline should succeed");
    let gcode_str = gcode.content();

    let our_zs = extract_layer_z_heights(gcode_str);

    // Reference Z heights: 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8
    let expected_zs: Vec<f64> = (1..=14)
        .map(|i| (i as f64 * 0.2 * 1000.0).round() / 1000.0)
        .collect();

    assert_eq!(
        our_zs.len(),
        14,
        "Expected 14 layer Z values, got {}. Values: {:?}",
        our_zs.len(),
        our_zs
    );

    for (i, (&actual, &expected)) in our_zs.iter().zip(expected_zs.iter()).enumerate() {
        assert!(
            (actual - expected).abs() < 0.05,
            "Layer {} Z mismatch: expected {:.1}, got {:.3}",
            i,
            expected,
            actual
        );
    }

    // The maximum Z should be 2.8 (not 2.792)
    let max_z = our_zs.last().copied().unwrap_or(0.0);
    assert!(
        (max_z - 2.8).abs() < 0.05,
        "Maximum layer Z should be 2.8, got {:.3}",
        max_z
    );
}

#[test]
fn test_cube_top_one_wall_config() {
    let config = cube_pipeline_config();

    // The benchy_h2d_pla config should have top_one_wall_type = AllTop
    assert_eq!(
        config.object.top_one_wall_type,
        TopOneWallType::AllTop,
        "benchy_h2d_pla config should set top_one_wall_type to AllTop"
    );
}

#[test]
fn test_cube_topmost_layer_has_no_inner_wall() {
    let mesh = load_cube_stl();
    let config = cube_pipeline_config();
    let mut pipeline = PrintPipeline::new(config);

    let gcode = pipeline.process(&mesh).expect("Pipeline should succeed");
    let gcode_str = gcode.content();

    // Find the last CHANGE_LAYER and check features after it
    let lines: Vec<&str> = gcode_str.lines().collect();
    let last_change_layer_idx = lines
        .iter()
        .rposition(|l| l.trim() == "; CHANGE_LAYER")
        .expect("Should have at least one CHANGE_LAYER");

    let last_layer_features: Vec<&str> = lines[last_change_layer_idx..]
        .iter()
        .filter_map(|l| l.trim().strip_prefix("; FEATURE: "))
        .collect();

    assert!(
        !last_layer_features.is_empty(),
        "Last layer should have at least one feature"
    );
    assert!(
        last_layer_features.contains(&"Outer wall"),
        "Last layer should have Outer wall. Features: {:?}",
        last_layer_features
    );
    assert!(
        !last_layer_features.contains(&"Inner wall"),
        "Last layer should NOT have Inner wall (top_one_wall_type = all top). Features: {:?}",
        last_layer_features
    );
    assert!(
        last_layer_features.contains(&"Top surface"),
        "Last layer should have Top surface. Features: {:?}",
        last_layer_features
    );
}

#[test]
fn test_cube_first_layer_has_bottom_surface() {
    let mesh = load_cube_stl();
    let config = cube_pipeline_config();
    let mut pipeline = PrintPipeline::new(config);

    let gcode = pipeline.process(&mesh).expect("Pipeline should succeed");
    let gcode_str = gcode.content();

    // Find the first CHANGE_LAYER and collect features until the second CHANGE_LAYER
    let lines: Vec<&str> = gcode_str.lines().collect();
    let mut change_layer_positions: Vec<usize> = lines
        .iter()
        .enumerate()
        .filter_map(|(i, l)| {
            if l.trim() == "; CHANGE_LAYER" {
                Some(i)
            } else {
                None
            }
        })
        .collect();

    assert!(
        change_layer_positions.len() >= 2,
        "Should have at least 2 CHANGE_LAYER markers"
    );

    let first_layer_end = change_layer_positions[1];
    let first_layer_features: Vec<&str> = lines[..first_layer_end]
        .iter()
        .filter_map(|l| l.trim().strip_prefix("; FEATURE: "))
        .collect();

    // Reference: Layer 0 has Inner wall, Outer wall, Bottom surface
    assert!(
        first_layer_features.contains(&"Inner wall"),
        "First layer should have Inner wall. Features: {:?}",
        first_layer_features
    );
    assert!(
        first_layer_features.contains(&"Outer wall"),
        "First layer should have Outer wall. Features: {:?}",
        first_layer_features
    );
    assert!(
        first_layer_features.contains(&"Bottom surface"),
        "First layer should have Bottom surface. Features: {:?}",
        first_layer_features
    );
}

#[test]
fn test_cube_middle_layers_have_internal_solid_infill() {
    let mesh = load_cube_stl();
    let config = cube_pipeline_config();
    let mut pipeline = PrintPipeline::new(config);

    let gcode = pipeline.process(&mesh).expect("Pipeline should succeed");
    let gcode_str = gcode.content();

    // Parse per-layer features
    let lines: Vec<&str> = gcode_str.lines().collect();
    let change_layer_positions: Vec<usize> = lines
        .iter()
        .enumerate()
        .filter_map(|(i, l)| {
            if l.trim() == "; CHANGE_LAYER" {
                Some(i)
            } else {
                None
            }
        })
        .collect();

    assert_eq!(change_layer_positions.len(), 14, "Should have 14 layers");

    // Middle layers (1..13 exclusive of last) should have Internal solid infill
    // (since 15% sparse infill on such a tiny cube gets promoted to solid)
    for layer_idx in 1..13 {
        let start = change_layer_positions[layer_idx];
        let end = if layer_idx + 1 < change_layer_positions.len() {
            change_layer_positions[layer_idx + 1]
        } else {
            lines.len()
        };

        let layer_features: Vec<&str> = lines[start..end]
            .iter()
            .filter_map(|l| l.trim().strip_prefix("; FEATURE: "))
            .collect();

        assert!(
            layer_features.contains(&"Internal solid infill"),
            "Layer {} should have Internal solid infill. Features: {:?}",
            layer_idx,
            layer_features
        );
        assert!(
            layer_features.contains(&"Inner wall"),
            "Layer {} should have Inner wall. Features: {:?}",
            layer_idx,
            layer_features
        );
        assert!(
            layer_features.contains(&"Outer wall"),
            "Layer {} should have Outer wall. Features: {:?}",
            layer_idx,
            layer_features
        );
        // Middle layers should NOT have sparse infill (promoted to solid)
        assert!(
            !layer_features.contains(&"Sparse infill"),
            "Layer {} should NOT have Sparse infill (narrow region promoted to solid). Features: {:?}",
            layer_idx,
            layer_features
        );
    }
}

#[test]
fn test_cube_reference_feature_match() {
    // Verify our output feature distribution matches the reference G-code exactly
    let reference_gcode = load_cube_reference_gcode();
    let ref_features = count_features(&reference_gcode);

    let mesh = load_cube_stl();
    let config = cube_pipeline_config();
    let mut pipeline = PrintPipeline::new(config);

    let gcode = pipeline.process(&mesh).expect("Pipeline should succeed");
    let our_features = count_features(gcode.content());

    // Compare feature counts (excluding "Custom" which is machine G-code)
    let features_to_check = [
        "Outer wall",
        "Inner wall",
        "Internal solid infill",
        "Top surface",
        "Bottom surface",
    ];

    for feature in &features_to_check {
        let ref_count = ref_features.get(*feature).copied().unwrap_or(0);
        let our_count = our_features.get(*feature).copied().unwrap_or(0);
        assert_eq!(
            our_count, ref_count,
            "Feature '{}' count mismatch: ours={}, reference={}",
            feature, our_count, ref_count
        );
    }
}

#[test]
fn test_top_one_wall_type_parsing() {
    assert_eq!(
        TopOneWallType::from_str_bambu("all top"),
        TopOneWallType::AllTop
    );
    assert_eq!(
        TopOneWallType::from_str_bambu("alltop"),
        TopOneWallType::AllTop
    );
    assert_eq!(
        TopOneWallType::from_str_bambu("all_top"),
        TopOneWallType::AllTop
    );
    assert_eq!(
        TopOneWallType::from_str_bambu("topmost"),
        TopOneWallType::TopMost
    );
    assert_eq!(
        TopOneWallType::from_str_bambu("top most"),
        TopOneWallType::TopMost
    );
    assert_eq!(TopOneWallType::from_str_bambu("none"), TopOneWallType::None);
    assert_eq!(TopOneWallType::from_str_bambu(""), TopOneWallType::None);

    assert!(TopOneWallType::AllTop.is_enabled());
    assert!(TopOneWallType::TopMost.is_enabled());
    assert!(!TopOneWallType::None.is_enabled());

    assert!(TopOneWallType::AllTop.is_all_top());
    assert!(!TopOneWallType::TopMost.is_all_top());
    assert!(!TopOneWallType::None.is_all_top());
}
