//! Gyroid infill pattern generation.
//!
//! Ported from BambuStudio's `Fill/FillGyroid.cpp`.
//!
//! The gyroid is a triply periodic minimal surface whose implicit equation is:
//!   sin(x)*cos(y) + sin(y)*cos(z) + sin(z)*cos(x) = 0
//!
//! This module generates polylines that approximate cross-sections of the gyroid
//! surface at a given Z height, suitable for use as infill paths.

use crate::geometry::{Point, Polyline};
use crate::{scale, Coord};
use std::f64::consts::{FRAC_PI_2, PI};

/// Small epsilon for floating-point comparisons, matching C++ EPSILON.
const EPSILON: f64 = 1e-4;

/// Maximum tolerance for curve approximation (in mm).
/// Corresponds to `FillGyroid::PatternTolerance` in the C++ source.
const PATTERN_TOLERANCE: f64 = 0.2;

/// Density adjustment factor.
/// Corresponds to `FillGyroid::DensityAdjust` in the C++ source.
const DENSITY_ADJUST: f64 = 2.44;

/// Gyroid wave equation.
///
/// Evaluates the implicit gyroid surface cross-section at position `x` for a
/// given Z-layer (encoded as `z_sin`, `z_cos`). The `vertical` flag selects
/// which orientation of the wave to generate, and `flip` shifts the phase to
/// produce alternating even/odd polylines.
///
/// Corresponds to the static `f()` function in `FillGyroid.cpp`.
#[inline]
fn f(x: f64, z_sin: f64, z_cos: f64, vertical: bool, flip: bool) -> f64 {
    if vertical {
        let phase_offset = if z_cos < 0.0 { PI } else { 0.0 } + PI;
        let a = (x + phase_offset).sin();
        let b = -z_cos;
        let res = z_sin * (x + phase_offset + if flip { PI } else { 0.0 }).cos();
        let r = (a * a + b * b).sqrt();
        (a / r).asin() + (res / r).asin() + PI
    } else {
        let phase_offset = if z_sin < 0.0 { PI } else { 0.0 };
        let a = (x + phase_offset).cos();
        let b = -z_sin;
        let res = z_cos * (x + phase_offset + if flip { 0.0 } else { PI }).sin();
        let r = (a * a + b * b).sqrt();
        (a / r).asin() + (res / r).asin() + 0.5 * PI
    }
}

/// Build one period of the gyroid wave with adaptive refinement.
///
/// Starts with coarse samples at π/2 intervals, then iteratively subdivides
/// segments whose midpoint deviates from the straight line by more than
/// `tolerance` (measured via cross-product area).
///
/// Corresponds to `make_one_period()` in `FillGyroid.cpp`.
fn make_one_period(
    width: f64,
    z_cos: f64,
    z_sin: f64,
    vertical: bool,
    flip: bool,
    tolerance: f64,
) -> Vec<[f64; 2]> {
    let dx = FRAC_PI_2;
    let limit = (2.0 * PI).min(width);

    let mut points: Vec<[f64; 2]> = Vec::with_capacity((limit / tolerance / 3.0).ceil() as usize);

    // Initial coarse sampling at π/2 intervals
    let mut x = 0.0;
    while x < limit - EPSILON {
        points.push([x, f(x, z_sin, z_cos, vertical, flip)]);
        x += dx;
    }
    points.push([limit, f(limit, z_sin, z_cos, vertical, flip)]);

    // Adaptive refinement: subdivide until all midpoints are within tolerance
    loop {
        let size = points.len();
        for i in 1..size {
            let lp = points[i - 1];
            let rp = points[i];
            let mx = lp[0] + (rp[0] - lp[0]) / 2.0;
            let my = f(mx, z_sin, z_cos, vertical, flip);
            let ip = [mx, my];
            // Cross product of (ip - lp) and (ip - rp) measures area of triangle;
            // if it exceeds tolerance^2 the curve deviates too much from the chord.
            let cross = (ip[0] - lp[0]) * (ip[1] - rp[1]) - (ip[1] - lp[1]) * (ip[0] - rp[0]);
            if cross.abs() > tolerance * tolerance {
                points.push(ip);
            }
        }

        if points.len() == size {
            break;
        }

        // Re-sort by x so the next iteration sees them in order
        points.sort_by(|a, b| a[0].partial_cmp(&b[0]).unwrap());
    }

    points
}

/// Extend one period to fill `width`, apply offset and clamping, and produce a
/// `Polyline` in scaled integer coordinates.
///
/// Corresponds to `make_wave()` in `FillGyroid.cpp`.
fn make_wave(
    one_period: &[[f64; 2]],
    width: f64,
    height: f64,
    offset: f64,
    scale_factor: f64,
    z_cos: f64,
    z_sin: f64,
    vertical: bool,
    flip: bool,
) -> Polyline {
    let mut points: Vec<[f64; 2]> = one_period.to_vec();
    let period = points.last().unwrap()[0];

    if (width - period).abs() > EPSILON {
        // Tile the single period to cover the full width
        let n = points.len();
        points.pop(); // remove last point (will be start of next tile)
        let base_len = points.len();

        loop {
            let idx = points.len() - n + 1; // +1 because we popped one
            if idx >= base_len {
                break;
            }
            let new_x = points[points.len() - base_len][0] + period;
            let new_y = points[points.len() - base_len][1];
            points.push([new_x, new_y]);
            if new_x >= width - EPSILON {
                break;
            }
        }

        // If we haven't covered the width, keep tiling
        while points.last().unwrap()[0] < width - EPSILON {
            let idx = points.len() - base_len;
            let new_x = points[idx][0] + period;
            let new_y = points[idx][1];
            points.push([new_x, new_y]);
        }

        // Final endpoint exactly at width
        points.push([width, f(width, z_sin, z_cos, vertical, flip)]);
    }

    // Build the polyline: apply y-offset, clamp, optionally swap axes, scale
    let mut polyline = Polyline::new();
    polyline.points.reserve(points.len());
    for pt in &mut points {
        pt[1] += offset;
        pt[1] = pt[1].clamp(0.0, height);

        let (px, py) = if vertical {
            (pt[1], pt[0])
        } else {
            (pt[0], pt[1])
        };

        polyline.points.push(Point::new(
            (px * scale_factor).round() as Coord,
            (py * scale_factor).round() as Coord,
        ));
    }

    polyline
}

/// Generate gyroid infill waves for the given parameters.
///
/// This is the main entry point, corresponding to `make_gyroid_waves()` in
/// `FillGyroid.cpp`. It produces a set of polylines that tile the bounding box.
///
/// # Parameters
///
/// * `grid_z` - Current layer Z height in scaled coordinates.
/// * `density_adjusted` - Fill density after adjustment (density * DENSITY_ADJUST).
/// * `line_spacing` - Nominal line spacing in mm.
/// * `width` - Bounding box width in multiples of the wave period.
/// * `height` - Bounding box height in multiples of the wave period.
///
/// # Returns
///
/// A vector of `Polyline`s representing the gyroid infill paths.
pub fn make_gyroid_waves(
    grid_z: f64,
    density_adjusted: f64,
    line_spacing: f64,
    width: f64,
    height: f64,
) -> Vec<Polyline> {
    let scale_factor = scale(line_spacing) as f64 / density_adjusted;

    // Tolerance in scaled units, clamped to PATTERN_TOLERANCE.
    // The C++ computes: min(line_spacing/2, PatternTolerance) / unscale<double>(scaleFactor)
    // where unscale<double>(v) = v / SCALING_FACTOR = v * 0.00001
    let tolerance =
        (line_spacing / 2.0).min(PATTERN_TOLERANCE) / (scale_factor / crate::SCALING_FACTOR);

    let z = grid_z / scale_factor;
    let z_sin = z.sin();
    let z_cos = z.cos();

    let vertical = z_sin.abs() <= z_cos.abs();
    let mut lower_bound = 0.0_f64;
    let mut upper_bound = height;
    let mut flip = true;
    let mut w = width;
    let mut h = height;

    if vertical {
        flip = false;
        lower_bound = -PI;
        upper_bound = w - FRAC_PI_2;
        std::mem::swap(&mut w, &mut h);
    }

    let one_period_odd = make_one_period(w, z_cos, z_sin, vertical, flip, tolerance);
    let flip_even = !flip;
    let one_period_even = make_one_period(w, z_cos, z_sin, vertical, flip_even, tolerance);

    let mut result: Vec<Polyline> = Vec::new();

    let mut y0 = lower_bound;
    while y0 < upper_bound + EPSILON {
        // Odd polyline
        result.push(make_wave(
            &one_period_odd,
            w,
            h,
            y0,
            scale_factor,
            z_cos,
            z_sin,
            vertical,
            flip,
        ));
        y0 += PI;
        // Even polyline
        if y0 < upper_bound + EPSILON {
            result.push(make_wave(
                &one_period_even,
                w,
                h,
                y0,
                scale_factor,
                z_cos,
                z_sin,
                vertical,
                flip_even,
            ));
        }
        y0 += PI;
    }

    result
}

/// Configuration for gyroid infill generation.
#[derive(Debug, Clone)]
pub struct GyroidConfig {
    /// Layer Z height in mm.
    pub z: f64,
    /// Fill angle in radians.
    pub angle: f64,
    /// Line spacing in mm.
    pub spacing: f64,
    /// Fill density (0.0 to 1.0).
    pub density: f64,
}

impl Default for GyroidConfig {
    fn default() -> Self {
        Self {
            z: 0.0,
            angle: 0.0,
            spacing: 0.4,
            density: 0.2,
        }
    }
}

/// Generate gyroid infill polylines for a bounding box.
///
/// This is the high-level API that mirrors `FillGyroid::_fill_surface_single()`.
/// It computes density-adjusted spacing, aligns the bounding box, generates the
/// wave pattern, and returns the raw polylines (before clipping to the fill
/// region, which is handled by the caller).
///
/// # Parameters
///
/// * `config` - Gyroid fill configuration.
/// * `bb_min` - Minimum corner of the bounding box (scaled coordinates).
/// * `bb_max` - Maximum corner of the bounding box (scaled coordinates).
///
/// # Returns
///
/// A vector of `Polyline`s, translated to the bounding-box origin.
pub fn generate_gyroid_infill(
    config: &GyroidConfig,
    bb_min: Point,
    bb_max: Point,
) -> Vec<Polyline> {
    let density_adjusted = (config.density * DENSITY_ADJUST).max(f64::MIN_POSITIVE);
    let distance = scale(config.spacing) as f64 / density_adjusted;

    // Align bounding box to grid
    let align = (2.0 * PI * distance) as Coord;
    let aligned_min_x = if align != 0 {
        (bb_min.x / align) * align - align
    } else {
        bb_min.x
    };
    let aligned_min_y = if align != 0 {
        (bb_min.y / align) * align - align
    } else {
        bb_min.y
    };
    let aligned_min = Point::new(aligned_min_x.min(bb_min.x), aligned_min_y.min(bb_min.y));

    let bb_size_x = (bb_max.x - aligned_min.x) as f64;
    let bb_size_y = (bb_max.y - aligned_min.y) as f64;

    let width = (bb_size_x / distance).ceil() + 1.0;
    let height = (bb_size_y / distance).ceil() + 1.0;

    let mut polylines = make_gyroid_waves(
        scale(config.z) as f64,
        density_adjusted,
        config.spacing,
        width,
        height,
    );

    // Translate polylines to the bounding box origin
    for pl in &mut polylines {
        for pt in &mut pl.points {
            pt.x += aligned_min.x;
            pt.y += aligned_min.y;
        }
    }

    polylines
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_f_function_symmetry() {
        // The gyroid f() should produce finite values
        let z_sin = (1.0_f64).sin();
        let z_cos = (1.0_f64).cos();
        let val = f(0.0, z_sin, z_cos, false, false);
        assert!(val.is_finite(), "f() returned non-finite: {}", val);
    }

    #[test]
    fn test_make_one_period_produces_points() {
        let z_sin = (0.5_f64).sin();
        let z_cos = (0.5_f64).cos();
        let pts = make_one_period(2.0 * PI, z_cos, z_sin, false, false, 0.1);
        assert!(
            pts.len() >= 5,
            "Expected at least 5 points, got {}",
            pts.len()
        );
        // Points should be sorted by x
        for i in 1..pts.len() {
            assert!(pts[i][0] >= pts[i - 1][0], "Points not sorted by x");
        }
    }

    #[test]
    fn test_make_gyroid_waves_produces_polylines() {
        let polylines = make_gyroid_waves(
            scale(0.3) as f64, // grid_z
            0.2 * DENSITY_ADJUST,
            0.4,  // line_spacing
            10.0, // width
            10.0, // height
        );
        assert!(!polylines.is_empty(), "Expected non-empty polylines");
        for pl in &polylines {
            assert!(
                pl.points.len() >= 2,
                "Each polyline should have at least 2 points"
            );
        }
    }

    #[test]
    fn test_generate_gyroid_infill() {
        let config = GyroidConfig {
            z: 0.3,
            spacing: 0.4,
            density: 0.2,
            angle: 0.0,
        };
        let bb_min = Point::new(0, 0);
        let bb_max = Point::new(scale(20.0), scale(20.0));
        let polylines = generate_gyroid_infill(&config, bb_min, bb_max);
        assert!(!polylines.is_empty(), "Expected non-empty gyroid infill");
    }
}
