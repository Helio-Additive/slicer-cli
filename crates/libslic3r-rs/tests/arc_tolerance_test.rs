//! Test to determine what units geo-clipper expects for arc tolerance.
//!
//! We need to figure out if JoinType::Round(x) expects:
//! - x in millimeters (unscaled)
//! - x in scaled internal units
//!
//! This test creates a simple square and offsets it with Round joins,
//! then counts the number of points generated to infer the arc precision.

use slicer::clipper_utils::{offset_polygon, OffsetJoinType};
use slicer::geometry::{Point, Polygon};
use slicer::{scale, unscale};

#[test]
fn test_arc_tolerance_units() {
    // Create a 10mm x 10mm square
    let mut square = Polygon::new();
    square.push(Point::new_scale(0.0, 0.0));
    square.push(Point::new_scale(10.0, 0.0));
    square.push(Point::new_scale(10.0, 10.0));
    square.push(Point::new_scale(0.0, 10.0));

    println!("\n=== Arc Tolerance Units Test ===");
    println!("Square: 10mm x 10mm");
    println!("Original points: {}", square.len());

    // Debug: Check actual coordinate values
    println!("\nDEBUG - Actual coordinates:");
    println!("  SCALING_FACTOR = {}", slicer::SCALING_FACTOR);
    println!("  scale(1.0) = {}", scale(1.0));
    println!("  scale(10.0) = {}", scale(10.0));
    for (i, pt) in square.points().iter().enumerate() {
        println!(
            "  Point {}: ({}, {}) scaled = ({:.2}, {:.2}) mm",
            i,
            pt.x,
            pt.y,
            unscale(pt.x),
            unscale(pt.y)
        );
    }

    // Test with current default (Round with 0.25 tolerance)
    // Offset by 1mm outward with Round joins
    let offset_result = offset_polygon(&square, 1.0, OffsetJoinType::Round);

    println!("\nTesting with current Round defaults:");
    println!("  Offset: 1mm outward");

    if offset_result.is_empty() {
        println!("  Result: FAILED - No output");
    } else {
        for (i, result_poly) in offset_result.iter().enumerate() {
            let point_count = result_poly.contour.len();

            // Calculate expected arc behavior
            // A 1mm offset creates 4 quarter-circles at corners, each with radius 1mm
            // Arc length per corner = π/2 * r = π/2 * 1mm ≈ 1.571mm
            // Total arc length = 4 * 1.571 = 6.283mm

            println!("\n  Polygon {}: {} points", i, point_count);

            if point_count > 4 {
                let arc_segments = (point_count.saturating_sub(4)) / 4;
                println!("  Approx points per corner: {}", arc_segments);

                // Estimate what tolerance this implies
                // For a quarter circle with radius r and n segments:
                // tolerance ≈ r * (1 - cos(π/(2*n)))
                if arc_segments > 0 {
                    let angle_per_segment = std::f64::consts::PI / (2.0 * arc_segments as f64);
                    let implied_tolerance = 1.0 * (1.0 - angle_per_segment.cos());
                    println!("  Implied tolerance: {:.6} mm", implied_tolerance);
                }
            }
        }
    }

    println!("\n=== Analysis ===");
    println!("BambuStudio uses:");
    println!("  - ArcTolerance = 3.0 (scaled units)");
    println!("  - In their scale: 3.0 * 0.00001 = 0.00003 mm");
    println!("  - Expected: Very fine arcs (hundreds of segments per corner)");
    println!("\nOur current Rust:");
    println!("  - JoinType::Round(0.25)");
    println!("  - If this is mm: Should give ~5-10 segments per corner (coarse)");
    println!("  - If this is scaled: Should give ~0 segments (way too coarse)");
    println!("\nCompare the point counts above to determine units.");
}

#[test]
fn test_compare_join_types() {
    // Create a 10mm x 10mm square
    let mut square = Polygon::new();
    square.push(Point::new_scale(0.0, 0.0));
    square.push(Point::new_scale(10.0, 0.0));
    square.push(Point::new_scale(10.0, 10.0));
    square.push(Point::new_scale(0.0, 10.0));

    println!("\n=== Join Type Comparison ===");
    println!("Original square: {} points", square.len());

    // Test Square join (should add exactly 4 points at corners)
    let square_result = offset_polygon(&square, 1.0, OffsetJoinType::Square);
    if !square_result.is_empty() {
        println!("Square join: {} points", square_result[0].contour.len());
    }

    // Test Miter join
    let miter_result = offset_polygon(&square, 1.0, OffsetJoinType::Miter);
    if !miter_result.is_empty() {
        println!("Miter join: {} points", miter_result[0].contour.len());
    }

    // Test Round join (current default)
    let round_result = offset_polygon(&square, 1.0, OffsetJoinType::Round);
    if !round_result.is_empty() {
        println!("Round join: {} points", round_result[0].contour.len());
    }

    println!("\nExpected for BambuStudio behavior:");
    println!("  - Square: 8 points (4 corners + 4 straight edges)");
    println!("  - Miter: 8 points (sharp corners)");
    println!("  - Round: Many more points (smooth arcs at corners)");
}

#[test]
fn test_scaling_factor_verification() {
    println!("\n=== Scaling Factor Verification ===");
    println!("Current Rust SCALING_FACTOR: {}", slicer::SCALING_FACTOR);
    println!("1mm scales to: {}", scale(1.0));
    println!("100,000 units unscales to: {:.6} mm", unscale(100_000));
    println!("1,000,000 units unscales to: {:.6} mm", unscale(1_000_000));

    println!("\nBambuStudio uses:");
    println!("  SCALING_FACTOR = 0.00001");
    println!("  1mm = 1 / 0.00001 = 100,000 units");
    println!("  1 unit = 0.00001mm = 10 nanometers");

    println!("\nCurrent Rust uses:");
    println!("  SCALING_FACTOR = 1,000,000");
    println!("  1mm = 1,000,000 units");
    println!("  1 unit = 0.000001mm = 1 nanometer");

    println!("\n✅ FIXED: Scale factor now matches BambuStudio!");

    // Verify the math
    assert_eq!(
        scale(1.0),
        100_000,
        "Current scale factor matches BambuStudio"
    );

    println!("\nscale(1.0) = {} ✓ (matches BambuStudio)", scale(1.0));
}

#[test]
fn test_fine_vs_coarse_arc_tolerance() {
    // Create a small square to test with
    let mut square = Polygon::new();
    square.push(Point::new_scale(0.0, 0.0));
    square.push(Point::new_scale(5.0, 0.0));
    square.push(Point::new_scale(5.0, 5.0));
    square.push(Point::new_scale(0.0, 5.0));

    println!("\n=== Fine vs Coarse Arc Tolerance ===");
    println!("Testing 5mm square with 0.5mm offset");

    // Offset with Round joins (uses default tolerance)
    let result = offset_polygon(&square, 0.5, OffsetJoinType::Round);

    if !result.is_empty() {
        let point_count = result[0].contour.len();
        println!("Result: {} points", point_count);

        // With 0.5mm radius corners:
        // - Very fine tolerance (0.00003mm): ~200-400 points per corner
        // - Coarse tolerance (0.25mm): ~2-4 points per corner

        if point_count < 20 {
            println!("→ This is COARSE arc approximation");
            println!("→ Tolerance is likely in millimeters (0.25mm)");
        } else if point_count > 100 {
            println!("→ This is FINE arc approximation");
            println!("→ Tolerance is likely very small (0.00003mm or similar)");
        } else {
            println!("→ This is MEDIUM arc approximation");
            println!("→ Need more analysis to determine units");
        }
    }

    println!("\nConclusion:");
    println!("If point count is low (<20): geo-clipper uses mm, need to change to 0.00003");
    println!("If point count is high (>100): geo-clipper uses scaled units, need to change to 3.0");
}
