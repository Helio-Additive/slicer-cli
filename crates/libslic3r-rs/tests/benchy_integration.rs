//! 3DBenchy Integration Tests
//!
//! The tests cover:
//! 1. STL loading and mesh validation
//! 2. Layer slicing accuracy
//!
//! The model is the documented Benchy fixture,
//! `tests/rust-engine/fixtures/3DBenchy.stl` (docs/rust-engine/README.md,
//! "Fixtures"). It is not committed: download it as the README says. Every test
//! that needs it prints `SKIPPED: ...` and returns when it is missing.
//!
//! These tests used to read a BambuStudio reference G-code,
//! `data/reference_gcodes/3DBenchy.gcode`, which is not kept in the repository.
//! The tests that only inspected that file (its header, layer list, move and
//! line counts) are gone with it; comparing the port's output with the C++
//! engine is the parity workflow (`slicer-parity compare`, same README). The
//! layer heights the slicer is checked against are now the pin's own:
//! generate_object_layers (Slicing.cpp) gives 240 layers, [0, 0.2] up to
//! [47.8, 48], for this 48 mm model at 0.2 mm (a generator calling it).

use std::collections::HashMap;
use std::fs;
use std::path::PathBuf;

use slicer::debug::compare::{ComparisonConfig, GCodeComparator, ParsedGCode};
use slicer::stl::read_stl_file as load_stl;
// DISABLED import: `slicer::pipeline` (PrintPipeline/PipelineConfig) was deleted from the
// engine; the tests that drove it are gated with #[cfg(any())] below.
#[cfg(any())]
use slicer::pipeline::{PipelineConfig, PrintPipeline};
use slicer::{Slicer, SlicingParams};
use slicer::TriangleMesh;

/// The documented Benchy fixture (docs/rust-engine/README.md, "Fixtures").
fn benchy_stl_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../tests/rust-engine/fixtures/3DBenchy.stl")
}

/// Load the Benchy fixture, or say the test is skipped when it is missing.
fn load_benchy_stl() -> Option<TriangleMesh> {
    let stl_path = benchy_stl_path();
    if !stl_path.exists() {
        eprintln!(
            "SKIPPED: the documented fixture {} is missing; download it as \
             docs/rust-engine/README.md (\"Fixtures\") says",
            stl_path.display()
        );
        return None;
    }
    Some(load_stl(&stl_path).expect("Failed to load 3DBenchy.stl"))
}

/// The pin's layer count for the Benchy at 0.2 mm layers with a 0.2 mm first
/// layer: generate_object_layers (Slicing.cpp) gives 240 layers for the 48 mm
/// model, the last [47.8, 48].
const PIN_LAYER_COUNT: usize = 240;

/// The reference G-code these helpers read is not kept (see the module
/// comment); only the compiled-out pipeline tests below use them.
#[cfg(any())]
fn load_benchy_reference_gcode() -> String {
    let gcode_path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("data/reference_gcodes/3DBenchy.gcode");
    fs::read_to_string(&gcode_path).expect("Failed to load 3DBenchy.gcode")
}

/// Parse G-code header settings from BambuStudio format
#[allow(dead_code)]
fn parse_gcode_settings(gcode: &str) -> HashMap<String, String> {
    let mut settings = HashMap::new();

    for line in gcode.lines() {
        let line = line.trim();
        if line.starts_with("; ") && line.contains(" = ") {
            if let Some(pos) = line.find(" = ") {
                let key = line[2..pos].trim().to_string();
                let value = line[pos + 3..].trim().to_string();
                settings.insert(key, value);
            }
        }
    }

    settings
}

/// Extract layer information from reference G-code
#[allow(dead_code)]
#[derive(Debug, Clone)]
struct LayerInfo {
    layer_num: usize,
    z_height: f64,
    line_start: usize,
    line_end: usize,
}

/// Parse layer changes from G-code
#[allow(dead_code)]
fn parse_layer_changes(gcode: &str) -> Vec<LayerInfo> {
    let mut layers = Vec::new();
    let mut current_layer: Option<LayerInfo> = None;
    let mut current_z: f64 = 0.0;
    let mut layer_num: usize = 0;

    for (line_num, line) in gcode.lines().enumerate() {
        let line = line.trim();

        if line == "; CHANGE_LAYER" {
            // End previous layer if exists
            if let Some(mut layer) = current_layer.take() {
                layer.line_end = line_num;
                layers.push(layer);
            }

            layer_num += 1;
            current_layer = Some(LayerInfo {
                layer_num,
                z_height: current_z,
                line_start: line_num,
                line_end: 0,
            });
        } else if line.starts_with("; Z_HEIGHT: ") {
            if let Ok(z) = line[12..].trim().parse::<f64>() {
                current_z = z;
                if let Some(ref mut layer) = current_layer {
                    layer.z_height = z;
                }
            }
        }
    }

    // Close last layer
    if let Some(mut layer) = current_layer {
        layer.line_end = gcode.lines().count();
        layers.push(layer);
    }

    layers
}

/// Extract G-code commands (G0, G1, G2, G3) from a string
#[derive(Debug, Clone, PartialEq)]
struct GCodeMove {
    command: String,
    x: Option<f64>,
    y: Option<f64>,
    z: Option<f64>,
    e: Option<f64>,
    f: Option<f64>,
    i: Option<f64>,
    j: Option<f64>,
}

impl GCodeMove {
    fn parse(line: &str) -> Option<Self> {
        let line = line.trim();

        // Skip comments and empty lines
        if line.is_empty() || line.starts_with(';') {
            return None;
        }

        // Check if it's a move command
        if !line.starts_with("G0")
            && !line.starts_with("G1")
            && !line.starts_with("G2")
            && !line.starts_with("G3")
        {
            return None;
        }

        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.is_empty() {
            return None;
        }

        let command = parts[0].to_string();
        let mut mov = GCodeMove {
            command,
            x: None,
            y: None,
            z: None,
            e: None,
            f: None,
            i: None,
            j: None,
        };

        for part in &parts[1..] {
            if part.starts_with(';') {
                break; // End of command, rest is comment
            }

            let (code, value) = part.split_at(1);
            if let Ok(v) = value.parse::<f64>() {
                match code {
                    "X" => mov.x = Some(v),
                    "Y" => mov.y = Some(v),
                    "Z" => mov.z = Some(v),
                    "E" => mov.e = Some(v),
                    "F" => mov.f = Some(v),
                    "I" => mov.i = Some(v),
                    "J" => mov.j = Some(v),
                    _ => {}
                }
            }
        }

        Some(mov)
    }
}

/// Statistics about G-code comparison
#[derive(Debug, Default)]
struct GCodeComparisonStats {
    total_moves: usize,
    matching_moves: usize,
    position_differences: Vec<f64>,
    extrusion_differences: Vec<f64>,
}

// =============================================================================
// TESTS
// =============================================================================

#[test]
fn test_benchy_stl_exists() {
    // The fixture is optional; when it is present it must be the documented
    // model (a binary STL of 225706 facets, 48 mm tall).
    let stl_path = benchy_stl_path();
    if !stl_path.exists() {
        eprintln!(
            "SKIPPED: the documented fixture {} is missing; download it as \
             docs/rust-engine/README.md (\"Fixtures\") says",
            stl_path.display()
        );
        return;
    }
    let len = fs::metadata(&stl_path).expect("stat 3DBenchy.stl").len();
    assert_eq!(len, 84 + 50 * 225706, "not the documented 3DBenchy.stl: {:?}", stl_path);
}

#[test]
fn test_benchy_stl_loads_successfully() {
    let Some(mesh) = load_benchy_stl() else { return };

    // 3DBenchy should have a reasonable number of triangles
    assert!(
        mesh.triangle_count() > 1000,
        "3DBenchy should have many triangles, got {}",
        mesh.triangle_count()
    );

    // Should have vertices
    assert!(
        mesh.vertex_count() > 1000,
        "3DBenchy should have many vertices, got {}",
        mesh.vertex_count()
    );
}

#[test]
fn test_benchy_mesh_bounding_box() {
    let Some(mesh) = load_benchy_stl() else { return };
    let bb = mesh.compute_bounding_box();

    assert!(bb.is_defined(), "Bounding box should be defined");

    // The fixture spans x [-29.176, 30.825], y [-15.502, 15.502], z [0, 48].
    let size = bb.size();
    assert!((size.x - 60.001).abs() < 1e-3, "X size {}", size.x);
    assert!((size.y - 31.004).abs() < 1e-3, "Y size {}", size.y);
    assert!((size.z - 48.0).abs() < 1e-6, "Z size {}", size.z);
}

#[test]
fn test_benchy_mesh_is_valid() {
    let Some(mesh) = load_benchy_stl() else { return };

    // All triangles should have valid vertices (checked via triangle_vertices)
    for i in 0..mesh.triangle_count() {
        // This will panic if indices are invalid
        let vertices = mesh.triangle_vertices(i);
        // Vertices should be finite numbers
        for v in &vertices {
            assert!(v.x.is_finite(), "Invalid vertex X coordinate");
            assert!(v.y.is_finite(), "Invalid vertex Y coordinate");
            assert!(v.z.is_finite(), "Invalid vertex Z coordinate");
        }
    }
}

#[test]
fn test_slicing_produces_correct_layer_count() {
    let Some(mesh) = load_benchy_stl() else { return };

    let params = SlicingParams {
        first_print_layer_height: 0.2,
        layer_height: 0.2,
        ..Default::default()
    };

    let slicer = Slicer::new(params);
    let layers = slicer.slice(&mesh).expect("Slicing should succeed");

    assert_eq!(layers.len(), PIN_LAYER_COUNT, "the pin slices this model into 240 layers");
}

#[test]
fn test_slicing_layer_heights_match_reference() {
    let Some(mesh) = load_benchy_stl() else { return };

    let params = SlicingParams {
        first_print_layer_height: 0.2,
        layer_height: 0.2,
        ..Default::default()
    };

    let slicer = Slicer::new(params);
    let layers = slicer.slice(&mesh).expect("Slicing should succeed");

    // The pin's layers are [0.2 i, 0.2 (i + 1)] for i in 0..240.
    assert_eq!(layers.len(), PIN_LAYER_COUNT);
    let mut max_z_diff = 0.0f64;
    for (i, layer) in layers.iter().enumerate() {
        let pin_z = 0.2 * (i + 1) as f64;
        max_z_diff = max_z_diff.max((layer.print_z - pin_z).abs());
    }
    assert!(max_z_diff < 1e-6, "Maximum Z height difference {} from the pin's layers", max_z_diff);
}

#[test]
fn test_slicing_produces_actual_geometry() {
    let Some(mesh) = load_benchy_stl() else { return };

    let params = SlicingParams {
        first_print_layer_height: 0.2,
        layer_height: 0.2,
        ..Default::default()
    };

    let slicer = Slicer::new(params);
    let layers = slicer.slice(&mesh).expect("Slicing should succeed");

    // Count layers that have actual geometry (ExPolygons with contours)
    let layers_with_geometry: Vec<_> = layers
        .iter()
        .enumerate()
        .filter(|(_, l)| l.regions().iter().any(|r| !r.slices.is_empty()))
        .collect();

    println!(
        "Layers with geometry: {}/{}",
        layers_with_geometry.len(),
        layers.len()
    );

    // All layers should have geometry since 3DBenchy is a solid object
    assert!(
        layers_with_geometry.len() > layers.len() * 9 / 10,
        "Expected >90% of layers to have geometry, got {}/{}",
        layers_with_geometry.len(),
        layers.len()
    );

    // Check that some layers have meaningful contours
    let mut total_contours = 0;
    let mut total_holes = 0;
    for layer in &layers {
        for region in layer.regions() {
            for expoly in region.slices.to_expolygons() {
                total_contours += 1;
                total_holes += expoly.holes.len();
            }
        }
    }

    println!(
        "Total contours: {}, Total holes: {}",
        total_contours, total_holes
    );

    // 3DBenchy should produce many contours across all layers
    assert!(
        total_contours > 200,
        "Expected many contours, got {}",
        total_contours
    );

    // 3DBenchy has internal cavities that should produce holes
    assert!(
        total_holes > 0,
        "Expected some holes (3DBenchy has internal cavities)"
    );
}

#[test]
fn test_gcode_move_parsing() {
    // Test the G-code move parser with various formats
    let line1 = "G1 X100.5 Y200.3 E0.5 F3000";
    let mov1 = GCodeMove::parse(line1).expect("Should parse G1 move");
    assert_eq!(mov1.command, "G1");
    assert_eq!(mov1.x, Some(100.5));
    assert_eq!(mov1.y, Some(200.3));
    assert_eq!(mov1.e, Some(0.5));
    assert_eq!(mov1.f, Some(3000.0));

    let line2 = "G2 X10 Y20 I5 J-5 E0.1";
    let mov2 = GCodeMove::parse(line2).expect("Should parse G2 arc");
    assert_eq!(mov2.command, "G2");
    assert_eq!(mov2.i, Some(5.0));
    assert_eq!(mov2.j, Some(-5.0));

    let comment = "; CHANGE_LAYER";
    assert!(GCodeMove::parse(comment).is_none(), "Should skip comments");

    let empty = "";
    assert!(GCodeMove::parse(empty).is_none(), "Should skip empty lines");
}

// =============================================================================
// PIPELINE INTEGRATION TESTS
// =============================================================================

// DISABLED: needs slicer::pipeline::{PrintPipeline, PipelineConfig} (and slicer::profiles), which were deleted from the engine (replaced by Print::process / app_slice)
#[cfg(any())]
#[test]
fn test_pipeline_generates_gcode_for_benchy() {
    let mesh = load_benchy_stl();

    // Configure pipeline similar to reference settings
    let config = PipelineConfig::new()
        .layer_height(0.2)
        .first_layer_height(0.2)
        .perimeters(3)
        .infill_density(0.15)
        .nozzle_diameter(0.4);

    let mut pipeline = PrintPipeline::new(config);
    let result = pipeline.process(&mesh);

    assert!(
        result.is_ok(),
        "Pipeline should process benchy: {:?}",
        result
    );

    let gcode = result.unwrap();
    assert!(!gcode.is_empty(), "G-code should not be empty");

    // Basic sanity checks
    let content = gcode.content();
    assert!(content.contains("G1"), "G-code should have extrusion moves");
    assert!(
        content.contains("Layer"),
        "G-code should have layer comments"
    );
    assert!(content.contains("G28"), "G-code should have homing");
    assert!(
        content.contains("M104") || content.contains("M109"),
        "G-code should set temperature"
    );
}

// DISABLED: needs slicer::pipeline::{PrintPipeline, PipelineConfig} (and slicer::profiles), which were deleted from the engine (replaced by Print::process / app_slice)
#[cfg(any())]
#[test]
fn test_pipeline_layer_count_reasonable() {
    let mesh = load_benchy_stl();
    let reference_gcode = load_benchy_reference_gcode();

    // Get reference layer count
    let ref_layers = parse_layer_changes(&reference_gcode);
    let ref_layer_count = ref_layers.len();

    // Generate with same layer height
    let config = PipelineConfig::new()
        .layer_height(0.2)
        .first_layer_height(0.2)
        .perimeters(3)
        .infill_density(0.15);

    let mut pipeline = PrintPipeline::new(config);
    let gcode = pipeline.process(&mesh).expect("Pipeline should succeed");

    // Count layers in generated G-code
    let generated_layer_count = gcode
        .content()
        .lines()
        .filter(|line| line.contains("; Layer "))
        .count();

    // Should have a similar number of layers (within 10%)
    let tolerance = (ref_layer_count as f64 * 0.1).ceil() as usize;
    let diff = (generated_layer_count as i64 - ref_layer_count as i64).unsigned_abs() as usize;

    assert!(
        diff <= tolerance,
        "Layer count should be within 10%: generated={}, reference={}, diff={}",
        generated_layer_count,
        ref_layer_count,
        diff
    );
}

// DISABLED: needs slicer::pipeline::{PrintPipeline, PipelineConfig} (and slicer::profiles), which were deleted from the engine (replaced by Print::process / app_slice)
#[cfg(any())]
#[test]
fn test_pipeline_extrusion_reasonable() {
    let mesh = load_benchy_stl();

    let config = PipelineConfig::new()
        .layer_height(0.2)
        .first_layer_height(0.2)
        .perimeters(3)
        .infill_density(0.15);

    let mut pipeline = PrintPipeline::new(config);
    let gcode = pipeline.process(&mesh).expect("Pipeline should succeed");

    // Count extrusion moves and calculate total E
    let mut total_e = 0.0f64;
    let mut last_e = 0.0f64;

    for line in gcode.content().lines() {
        if line.starts_with("G1") && line.contains(" E") {
            if let Some(e_value) = extract_e_value(line) {
                // Absolute extrusion mode
                if e_value > last_e {
                    total_e += e_value - last_e;
                }
                last_e = e_value;
            }
        } else if line.starts_with("G92") && line.contains("E0") {
            last_e = 0.0;
        }
    }

    // Reference shows ~3869mm filament
    // We should be in a reasonable range (say 1000-10000mm)
    assert!(
        total_e > 1000.0,
        "Total extrusion should be > 1000mm, got {}",
        total_e
    );
    assert!(
        total_e < 20000.0,
        "Total extrusion should be < 20000mm, got {}",
        total_e
    );
}

/// Extract E value from a G-code line
fn extract_e_value(line: &str) -> Option<f64> {
    for part in line.split_whitespace() {
        if part.starts_with('E') {
            return part[1..].parse().ok();
        }
    }
    None
}

// DISABLED: needs slicer::pipeline::{PrintPipeline, PipelineConfig} (and slicer::profiles), which were deleted from the engine (replaced by Print::process / app_slice)
#[cfg(any())]
#[test]
fn test_pipeline_gcode_stats() {
    let mesh = load_benchy_stl();

    let config = PipelineConfig::new()
        .layer_height(0.2)
        .perimeters(3)
        .infill_density(0.15);

    let mut pipeline = PrintPipeline::new(config);
    let gcode = pipeline.process(&mesh).expect("Pipeline should succeed");

    let stats = &gcode.stats;
    assert!(
        stats.layer_count > 100,
        "Should have >100 layers for benchy"
    );
    assert!(
        stats.extrusion_distance_mm > 0.0,
        "Should have extrusion distance"
    );
}

// FUTURE TESTS - Comparison against reference
// =============================================================================

// DISABLED: needs slicer::pipeline::{PrintPipeline, PipelineConfig} (and slicer::profiles), which were deleted from the engine (replaced by Print::process / app_slice)
#[cfg(any())]
#[test]
#[ignore = "Requires closer parity with BambuStudio output"]
fn test_generated_gcode_layer_count_matches() {
    // This test will compare the number of layers in generated G-code
    // vs the reference G-code exactly
    let mesh = load_benchy_stl();
    let reference_gcode = load_benchy_reference_gcode();
    let ref_layers = parse_layer_changes(&reference_gcode);

    let config = PipelineConfig::new()
        .layer_height(0.2)
        .first_layer_height(0.2)
        .perimeters(3)
        .infill_density(0.15);

    let mut pipeline = PrintPipeline::new(config);
    let gcode = pipeline.process(&mesh).expect("Pipeline should succeed");

    let generated_layer_count = gcode
        .content()
        .lines()
        .filter(|line| line.contains("; Layer "))
        .count();

    assert_eq!(
        generated_layer_count,
        ref_layers.len(),
        "Layer count should match exactly"
    );
}

// The placeholders for total extrusion, print time and byte parity against the
// reference G-code (todo!() bodies, ignored) are gone with that G-code: the
// comparison with the C++ engine is `slicer-parity compare` and the scripts in
// scripts/parity/ (docs/rust-engine/README.md, "Parity workflow").

// DISABLED: needs slicer::pipeline::{PrintPipeline, PipelineConfig} (and slicer::profiles), which were deleted from the engine (replaced by Print::process / app_slice)
#[cfg(any())]
#[test]
fn test_toolpath_semantic_equivalence() {
    // This test compares toolpaths semantically using the GCodeComparator
    let mesh = load_benchy_stl();
    let reference_gcode = load_benchy_reference_gcode();

    // Generate G-code using our pipeline
    let config = PipelineConfig::default();
    let mut pipeline = PrintPipeline::new(config);
    let gcode = pipeline.process(&mesh).expect("Pipeline should succeed");
    let generated_gcode = gcode.content();

    // Parse both G-codes
    let ref_parsed = ParsedGCode::from_string(&reference_gcode);
    let gen_parsed = ParsedGCode::from_string(&generated_gcode);

    // Use relaxed comparison settings since we're not at full parity yet
    let comparison_config = ComparisonConfig::relaxed();
    let comparator = GCodeComparator::new(comparison_config);
    let comparison = comparator.compare(&ref_parsed, &gen_parsed);

    println!("\n{}", comparison.summary());

    // Use stats from our generated gcode for layer count (more reliable than parser detection)
    let gen_layer_count = gcode.stats.layer_count;
    let ref_layer_count = 240; // Known from reference gcode (;LAYER_COUNT:240)

    let layer_count_diff_percent = ((gen_layer_count as i32 - ref_layer_count as i32).abs() as f64
        / ref_layer_count as f64)
        * 100.0;
    println!(
        "Layer count (from stats): {} vs {} ({:.1}% diff)",
        ref_layer_count, gen_layer_count, layer_count_diff_percent
    );
    // Allow up to 5% difference in layer count
    assert!(
        layer_count_diff_percent < 5.0,
        "Layer count differs by more than 5%: {} vs {}",
        ref_layer_count,
        gen_layer_count
    );

    // Check total moves are in reasonable range
    println!(
        "Total moves: {} (ref) vs {} (gen)",
        comparison.ref_total_moves, comparison.gen_total_moves
    );

    // Check generated gcode has substantial content
    assert!(
        comparison.gen_total_moves > 100000,
        "Generated gcode should have substantial moves"
    );

    // Report overall match percentage (informational, not a hard requirement yet)
    // We don't assert on match percentage since our algorithms differ from reference
    println!(
        "Overall semantic match: {:.1}% (informational only)",
        comparison.match_percentage()
    );
}

// =============================================================================
// BENCHMARK/METRICS TESTS
// =============================================================================

#[test]
fn test_slicing_performance() {
    use std::time::Instant;

    let Some(mesh) = load_benchy_stl() else { return };
    let params = SlicingParams {
        first_print_layer_height: 0.2,
        layer_height: 0.2,
        ..Default::default()
    };

    let slicer = Slicer::new(params);

    let start = Instant::now();
    let _layers = slicer.slice(&mesh).expect("Slicing should succeed");
    let elapsed = start.elapsed();

    println!("Slicing time: {:?}", elapsed);

    // Should complete in reasonable time (adjust as implementation matures)
    assert!(
        elapsed.as_secs() < 30,
        "Slicing took too long: {:?}",
        elapsed
    );
}

// =============================================================================
// HELPER FUNCTIONS FOR FUTURE COMPARISON TESTS
// =============================================================================

/// Calculate the total extrusion from G-code
#[allow(dead_code)]
fn calculate_total_extrusion(gcode: &str) -> f64 {
    let mut total_positive = 0.0;

    for line in gcode.lines() {
        if let Some(mov) = GCodeMove::parse(line) {
            if let Some(e) = mov.e {
                if e > 0.0 {
                    total_positive += e;
                }
            }
        }
    }

    total_positive
}

/// Extract feature sections from G-code (e.g., "Outer wall", "Inner wall", "Infill")
#[allow(dead_code)]
fn extract_features(gcode: &str) -> HashMap<String, usize> {
    let mut features = HashMap::new();
    let mut current_feature = String::from("Unknown");

    for line in gcode.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with("; FEATURE: ") {
            current_feature = trimmed[11..].to_string();
        } else if GCodeMove::parse(trimmed).is_some() {
            *features.entry(current_feature.clone()).or_insert(0) += 1;
        }
    }

    features
}
