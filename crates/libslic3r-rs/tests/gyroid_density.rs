//! Gyroid infill DENSITY regression test (R457).
//!
//! The gyroid filler is what Majora's Mask actually uses for sparse infill
//! (`sparse_infill_pattern = "gyroid"`), and R456 found two real bugs in the call
//! path by chasing a line-length shortfall. Length-per-area is the property that
//! matters and the one that silently regresses, so pin it here on a single large
//! square where there is no partition, no fragmentation and no classification to
//! confound it.
//!
//! Reference chain (FillGyroid.cpp):
//!   :164  density_adjusted = density * DensityAdjust        (DensityAdjust = 2.44)
//!   :166  distance         = scale_(this->spacing) / density_adjusted
//!   :135  waves are emitted every M_PI in normalised units, i.e. every
//!         PI * scaleFactor in scaled units, where scaleFactor == distance.
//!
//! So for a fill of area A the straight-line-equivalent length is A / (PI*distance),
//! and the real length is that times the wave's arc-length factor (> 1, because each
//! wave oscillates rather than running straight).

use slicer::clipper_utils::intersection_pl;
use slicer::fill::fill_gyroid::{generate_gyroid_infill, GyroidConfig};
use slicer::geometry::{ExPolygon, Point, Polygon};

/// Majora's resolved sparse-infill flow: width 0.45 at layer height 0.3 gives
/// spacing = 0.45 - 0.3*(1 - PI/4) = 0.38562...
const SPACING: f64 = 0.3856;
const DENSITY: f64 = 0.10;
const DENSITY_ADJUST: f64 = 2.44;

fn square(side_mm: f64) -> ExPolygon {
    let s = slicer::scale(side_mm);
    ExPolygon::new(Polygon::from_points(vec![
        Point::new(0, 0),
        Point::new(s, 0),
        Point::new(s, s),
        Point::new(0, s),
    ]))
}

/// Total clipped gyroid length (mm) over one square, mirroring what
/// `Layer::make_fills` does for `InfillPattern::Gyroid`.
fn gyroid_length_mm(side_mm: f64, z: f64) -> f64 {
    let ex = square(side_mm);
    let cfg = GyroidConfig { z, spacing: SPACING, density: DENSITY, angle: 0.0 };
    let mut bb = slicer::geometry::BoundingBox::empty();
    for p in &ex.contour.points {
        bb.merge_point(*p);
    }
    let raw = generate_gyroid_infill(&cfg, bb.min, bb.max);
    let clipped = intersection_pl(&raw, std::slice::from_ref(&ex));
    let minlength = slicer::scale(0.8 * SPACING) as f64;
    clipped
        .iter()
        .filter(|pl| pl.length() >= minlength)
        .map(|pl| pl.length())
        .sum::<f64>()
        / slicer::SCALING_FACTOR
}

#[test]
fn gyroid_length_per_area_matches_the_cpp_wave_spacing() {
    let side = 100.0_f64;
    let area = side * side;

    // FillGyroid.cpp:164-166.
    let distance_mm = SPACING / (DENSITY * DENSITY_ADJUST);
    // FillGyroid.cpp:135 — one wave every M_PI in normalised units.
    let wave_pitch_mm = std::f64::consts::PI * distance_mm;
    // A fill of `area` covered by parallel waves of pitch `wave_pitch` has this
    // much length if each wave ran perfectly straight.
    let straight_equivalent = area / wave_pitch_mm;

    // Average over several z so the test does not sit on one gyroid phase (the
    // `vertical` branch at FillGyroid.cpp:119 flips with sin(z) vs cos(z)).
    let zs = [0.3_f64, 1.5, 3.0, 7.5, 12.0, 20.1];
    let lengths: Vec<f64> = zs.iter().map(|&z| gyroid_length_mm(side, z)).collect();
    let mean = lengths.iter().sum::<f64>() / lengths.len() as f64;
    let arc_factor = mean / straight_equivalent;

    println!(
        "gyroid: distance {:.4}mm  wave pitch {:.4}mm  straight-equiv {:.0}mm\n\
         per-z lengths {:?}\n\
         mean {:.0}mm over {:.0}mm2 = {:.4} mm/mm2  (arc factor {:.3})",
        distance_mm, wave_pitch_mm, straight_equivalent, lengths, mean, area, mean / area, arc_factor
    );

    // Every wave must span the square, so the length can never be BELOW the
    // straight-line equivalent (minus the minlength filter's small bites at the
    // corners); and a gyroid wave's arc length is bounded well under 2x straight.
    assert!(
        arc_factor > 0.9,
        "gyroid produces less line than parallel straight waves at the same pitch \
         (arc factor {arc_factor:.3}) — waves are missing or clipped away"
    );
    assert!(
        arc_factor < 2.0,
        "gyroid arc factor {arc_factor:.3} is implausibly high — wave pitch is wrong"
    );
}

/// Length must scale with area, not with the number of pieces: two 50mm squares
/// should give ~the same total as the same area in one piece. This is the
/// fragmentation control for the R457 measurement.
#[test]
fn gyroid_length_is_insensitive_to_fragmentation() {
    let big = gyroid_length_mm(100.0, 3.0);
    // Four 50mm squares = the same 10,000 mm2.
    let small = 4.0 * gyroid_length_mm(50.0, 3.0);
    let ratio = small / big;
    println!("gyroid fragmentation: 1x100mm {big:.0}mm vs 4x50mm {small:.0}mm  ratio {ratio:.3}");
    // Splitting into 4 pieces adds boundary, so a few percent loss to the
    // minlength filter is expected; anything approaching 2x is a real defect.
    assert!(
        (0.85..1.15).contains(&ratio),
        "fragmenting the same area changed gyroid length by {ratio:.3}x"
    );
}
