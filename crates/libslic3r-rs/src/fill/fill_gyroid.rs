//! Gyroid mathematical-surface infill pattern.
//!
//! C++ Reference:
//! - Fill/FillGyroid.hpp
//! - Fill/FillGyroid.cpp
//!
//! Faithful 1:1 port of `Slic3r::FillGyroid` (FillGyroid.cpp). Generates a
//! gyroid infill by sampling the implicit gyroid surface cross-section for the
//! current layer Z and tiling the resulting waves across the fill region.

// FillGyroid.cpp:1-8
//   #include "../ClipperUtils.hpp"
//   #include "../ShortestPath.hpp"
//   #include "../Surface.hpp"
//   #include <cmath>
//   #include <algorithm>
//   #include <iostream>
//   #include "FillGyroid.hpp"
use super::{connect_infill_expolygon, multiline_fill, FillParams};
use crate::clipper_utils::intersection_pl;
use crate::geometry::{align_to_grid_point, BoundingBox, ExPolygon, Point, Polyline};
use crate::shortest_path::chain_polylines;
use crate::{scale, Coord, CoordF, SCALING_FACTOR};
use std::f64::consts::{FRAC_PI_2, PI};

// FillGyroid.cpp:10 — namespace Slic3r

/// EPSILON, matching libslic3r's `EPSILON` (libslic3r.h:65 — `1e-4`).
const EPSILON: f64 = 1e-4;

// FillGyroid.hpp:22 — `static constexpr float CorrectionAngle = -45.;`
// Correction applied to regular infill angle to maximize printing
// speed in default configuration (degrees).
const CORRECTION_ANGLE: f32 = -45.0;

// FillGyroid.hpp:25 — `static constexpr double DensityAdjust = 2.44;`
// Density adjustment to have a good %of weight.
const DENSITY_ADJUST: f64 = 2.44;

// FillGyroid.hpp:28 — `static constexpr double PatternTolerance = 0.2;`
// Gyroid upper resolution tolerance (mm^-2).
const PATTERN_TOLERANCE: f64 = 0.2;

/// `sqr(x)` — Slic3r helper (libslic3r.h), `x * x`.
#[inline]
fn sqr(x: f64) -> f64 {
    x * x
}

/// `cross2(a, b)` — 2D cross product `a.x*b.y - a.y*b.x` (Point.hpp).
#[inline]
fn cross2(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

// FillGyroid.cpp:12-30
//   static inline double f(double x, double z_sin, double z_cos, bool vertical, bool flip)
#[inline]
fn f(x: f64, z_sin: f64, z_cos: f64, vertical: bool, flip: bool) -> f64 {
    if vertical {
        // FillGyroid.cpp:15 — double phase_offset = (z_cos < 0 ? M_PI : 0) + M_PI;
        let phase_offset = (if z_cos < 0.0 { PI } else { 0.0 }) + PI;
        // FillGyroid.cpp:16 — double a   = sin(x + phase_offset);
        let a = (x + phase_offset).sin();
        // FillGyroid.cpp:17 — double b   = - z_cos;
        let b = -z_cos;
        // FillGyroid.cpp:18 — double res = z_sin * cos(x + phase_offset + (flip ? M_PI : 0.));
        let res = z_sin * (x + phase_offset + if flip { PI } else { 0.0 }).cos();
        // FillGyroid.cpp:19 — double r   = sqrt(sqr(a) + sqr(b));
        let r = (sqr(a) + sqr(b)).sqrt();
        // FillGyroid.cpp:20 — return asin(a/r) + asin(res/r) + M_PI;
        (a / r).asin() + (res / r).asin() + PI
    } else {
        // FillGyroid.cpp:23 — double phase_offset = z_sin < 0 ? M_PI : 0.;
        let phase_offset = if z_sin < 0.0 { PI } else { 0.0 };
        // FillGyroid.cpp:24 — double a   = cos(x + phase_offset);
        let a = (x + phase_offset).cos();
        // FillGyroid.cpp:25 — double b   = - z_sin;
        let b = -z_sin;
        // FillGyroid.cpp:26 — double res = z_cos * sin(x + phase_offset + (flip ? 0 : M_PI));
        let res = z_cos * (x + phase_offset + if flip { 0.0 } else { PI }).sin();
        // FillGyroid.cpp:27 — double r   = sqrt(sqr(a) + sqr(b));
        let r = (sqr(a) + sqr(b)).sqrt();
        // FillGyroid.cpp:28 — return (asin(a/r) + asin(res/r) + 0.5 * M_PI);
        (a / r).asin() + (res / r).asin() + 0.5 * PI
    }
}

// FillGyroid.cpp:32-63
//   static inline Polyline make_wave(
//       const std::vector<Vec2d>& one_period, double width, double height, double offset, double scaleFactor,
//       double z_cos, double z_sin, bool vertical, bool flip)
#[allow(clippy::too_many_arguments)]
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
    // FillGyroid.cpp:36 — std::vector<Vec2d> points = one_period;
    let mut points: Vec<[f64; 2]> = one_period.to_vec();
    // FillGyroid.cpp:37 — double period = points.back()(0);
    let period = points.last().unwrap()[0];
    // FillGyroid.cpp:38 — if (width != period) // do not extend if already truncated
    if width != period {
        // FillGyroid.cpp:40 — points.reserve(one_period.size() * size_t(floor(width / period)));
        points.reserve(one_period.len() * (width / period).floor() as usize);
        // FillGyroid.cpp:41 — points.pop_back();
        points.pop();

        // FillGyroid.cpp:43 — size_t n = points.size();
        let n = points.len();
        // FillGyroid.cpp:44-46
        //   do {
        //       points.emplace_back(points[points.size()-n].x() + period, points[points.size()-n].y());
        //   } while (points.back()(0) < width - EPSILON);
        loop {
            let src = points[points.len() - n];
            points.push([src[0] + period, src[1]]);
            if !(points.last().unwrap()[0] < width - EPSILON) {
                break;
            }
        }

        // FillGyroid.cpp:48 — points.emplace_back(Vec2d(width, f(width, z_sin, z_cos, vertical, flip)));
        points.push([width, f(width, z_sin, z_cos, vertical, flip)]);
    }

    // FillGyroid.cpp:51-52 — and construct the final polyline to return:
    //   Polyline polyline;
    let mut polyline = Polyline::new();
    // FillGyroid.cpp:53 — polyline.points.reserve(points.size());
    polyline.points.reserve(points.len());
    // FillGyroid.cpp:54 — for (auto& point : points) {
    for point in &mut points {
        // FillGyroid.cpp:55 — point(1) += offset;
        point[1] += offset;
        // FillGyroid.cpp:56 — point(1) = std::clamp(double(point.y()), 0., height);
        point[1] = point[1].clamp(0.0, height);
        // FillGyroid.cpp:57-58 — if (vertical) std::swap(point(0), point(1));
        if vertical {
            point.swap(0, 1);
        }
        // FillGyroid.cpp:59 — polyline.points.emplace_back((point * scaleFactor).cast<coord_t>());
        // Eigen's `cast<coord_t>()` truncates toward zero.
        // FIDELITY-NOTE(F2): crate Coord = i64 but C++ coord_t = int32_t
        // (libslic3r.h:40); for large scaled coordinates the int32 truncation
        // would wrap whereas i64 does not. Coord width is crate-wide.
        polyline.points.push(Point::new(
            (point[0] * scale_factor) as Coord,
            (point[1] * scale_factor) as Coord,
        ));
    }

    // FillGyroid.cpp:62 — return polyline;
    polyline
}

// FillGyroid.cpp:65-103
//   static std::vector<Vec2d> make_one_period(double width, double scaleFactor, double z_cos, double z_sin, bool vertical, bool flip, double tolerance)
#[allow(clippy::too_many_arguments)]
fn make_one_period(
    width: f64,
    _scale_factor: f64,
    z_cos: f64,
    z_sin: f64,
    vertical: bool,
    flip: bool,
    tolerance: f64,
) -> Vec<[f64; 2]> {
    // FillGyroid.cpp:67 — std::vector<Vec2d> points;
    let mut points: Vec<[f64; 2]> = Vec::new();
    // FillGyroid.cpp:68 — double dx = M_PI_2; // exact coordinates on main inflexion lobes
    let dx = FRAC_PI_2;
    // FillGyroid.cpp:69 — double limit = std::min(2*M_PI, width);
    let limit = (2.0 * PI).min(width);
    // FillGyroid.cpp:70 — points.reserve(coord_t(ceil(limit / tolerance / 3)));
    points.reserve((limit / tolerance / 3.0).ceil() as usize);

    // FillGyroid.cpp:72-74
    //   for (double x = 0.; x < limit - EPSILON; x += dx) {
    //       points.emplace_back(Vec2d(x, f(x, z_sin, z_cos, vertical, flip)));
    //   }
    let mut x = 0.0;
    while x < limit - EPSILON {
        points.push([x, f(x, z_sin, z_cos, vertical, flip)]);
        x += dx;
    }
    // FillGyroid.cpp:75 — points.emplace_back(Vec2d(limit, f(limit, z_sin, z_cos, vertical, flip)));
    points.push([limit, f(limit, z_sin, z_cos, vertical, flip)]);

    // FillGyroid.cpp:77 — piecewise increase in resolution up to requested tolerance
    // FillGyroid.cpp:78 — for(;;)
    loop {
        // FillGyroid.cpp:80 — size_t size = points.size();
        let size = points.len();
        // FillGyroid.cpp:81 — for (unsigned int i = 1;i < size; ++i) {
        for i in 1..size {
            // FillGyroid.cpp:82 — auto& lp = points[i-1]; // left point
            let lp = points[i - 1];
            // FillGyroid.cpp:83 — auto& rp = points[i];   // right point
            let rp = points[i];
            // FillGyroid.cpp:84 — double x = lp(0) + (rp(0) - lp(0)) / 2;
            let x = lp[0] + (rp[0] - lp[0]) / 2.0;
            // FillGyroid.cpp:85 — double y = f(x, z_sin, z_cos, vertical, flip);
            let y = f(x, z_sin, z_cos, vertical, flip);
            // FillGyroid.cpp:86 — Vec2d ip = {x, y};
            let ip = [x, y];
            // FillGyroid.cpp:87 — if (std::abs(cross2(Vec2d(ip - lp), Vec2d(ip - rp))) > sqr(tolerance)) {
            let ip_lp = [ip[0] - lp[0], ip[1] - lp[1]];
            let ip_rp = [ip[0] - rp[0], ip[1] - rp[1]];
            if cross2(ip_lp, ip_rp).abs() > sqr(tolerance) {
                // FillGyroid.cpp:88 — points.emplace_back(std::move(ip));
                points.push(ip);
            }
        }

        // FillGyroid.cpp:92-93 — if (size == points.size()) break;
        if size == points.len() {
            break;
        } else {
            // FillGyroid.cpp:96-99 — insert new points in order
            //   std::sort(points.begin(), points.end(),
            //             [](const Vec2d &lhs, const Vec2d &rhs) { return lhs(0) < rhs(0); });
            points.sort_by(|lhs, rhs| lhs[0].partial_cmp(&rhs[0]).unwrap());
        }
    }

    // FillGyroid.cpp:102 — return points;
    points
}

// FillGyroid.cpp:105-146
//   static Polylines make_gyroid_waves(double gridZ, double density_adjusted, double line_spacing, double width, double height)
fn make_gyroid_waves(
    grid_z: f64,
    density_adjusted: f64,
    line_spacing: f64,
    width: f64,
    height: f64,
) -> Vec<Polyline> {
    // FillGyroid.cpp:107 — const double scaleFactor = scale_(line_spacing) / density_adjusted;
    // scale_(v) == v * SCALING_FACTOR (kept in floating point — not yet truncated).
    let scale_factor = (line_spacing * SCALING_FACTOR) / density_adjusted;

    // FillGyroid.cpp:109-111
    //   tolerance in scaled units. clamp the maximum tolerance as there's
    //   no processing-speed benefit to do so beyond a certain point
    //   const double tolerance = std::min(line_spacing / 2, FillGyroid::PatternTolerance) / unscale<double>(scaleFactor);
    // unscale<double>(v) == double(v) * SCALING_FACTOR (libslic3r.h:112), i.e.
    // v / 100000 with this crate's SCALING_FACTOR == 100000; no truncation.
    let tolerance =
        (line_spacing / 2.0).min(PATTERN_TOLERANCE) / (scale_factor / SCALING_FACTOR);

    // FillGyroid.cpp:113-114
    //   //scale factor for 5% : 8 712 388
    //   // 1z = 10^-6 mm ?
    // FillGyroid.cpp:115 — const double z     = gridZ / scaleFactor;
    let z = grid_z / scale_factor;
    // FillGyroid.cpp:116 — const double z_sin = sin(z);
    let z_sin = z.sin();
    // FillGyroid.cpp:117 — const double z_cos = cos(z);
    let z_cos = z.cos();

    // FillGyroid.cpp:119 — bool vertical = (std::abs(z_sin) <= std::abs(z_cos));
    let vertical = z_sin.abs() <= z_cos.abs();
    // FillGyroid.cpp:120 — double lower_bound = 0.;
    let mut lower_bound = 0.0;
    // FillGyroid.cpp:121 — double upper_bound = height;
    let mut upper_bound = height;
    // FillGyroid.cpp:122 — bool flip = true;
    let mut flip = true;
    // mutable copies of the swap-able parameters
    let mut width = width;
    let mut height = height;
    // FillGyroid.cpp:123 — if (vertical) {
    if vertical {
        // FillGyroid.cpp:124 — flip = false;
        flip = false;
        // FillGyroid.cpp:125 — lower_bound = -M_PI;
        lower_bound = -PI;
        // FillGyroid.cpp:126 — upper_bound = width - M_PI_2;
        upper_bound = width - FRAC_PI_2;
        // FillGyroid.cpp:127 — std::swap(width,height);
        std::mem::swap(&mut width, &mut height);
    }

    // FillGyroid.cpp:130 — std::vector<Vec2d> one_period_odd = make_one_period(width, scaleFactor, z_cos, z_sin, vertical, flip, tolerance);
    // creates one period of the waves, so it doesn't have to be recalculated all the time
    let one_period_odd = make_one_period(width, scale_factor, z_cos, z_sin, vertical, flip, tolerance);
    // FillGyroid.cpp:131 — flip = !flip; // even polylines are a bit shifted
    flip = !flip;
    // FillGyroid.cpp:132 — std::vector<Vec2d> one_period_even = make_one_period(width, scaleFactor, z_cos, z_sin, vertical, flip, tolerance);
    let one_period_even = make_one_period(width, scale_factor, z_cos, z_sin, vertical, flip, tolerance);
    // FillGyroid.cpp:133 — Polylines result;
    let mut result: Vec<Polyline> = Vec::new();

    // FillGyroid.cpp:135 — for (double y0 = lower_bound; y0 < upper_bound + EPSILON; y0 += M_PI) {
    let mut y0 = lower_bound;
    while y0 < upper_bound + EPSILON {
        // FillGyroid.cpp:136-137 — creates odd polylines
        //   result.emplace_back(make_wave(one_period_odd, width, height, y0, scaleFactor, z_cos, z_sin, vertical, flip));
        result.push(make_wave(
            &one_period_odd,
            width,
            height,
            y0,
            scale_factor,
            z_cos,
            z_sin,
            vertical,
            flip,
        ));
        // FillGyroid.cpp:138-139 — creates even polylines
        //   y0 += M_PI;
        y0 += PI;
        // FillGyroid.cpp:140 — if (y0 < upper_bound + EPSILON) {
        if y0 < upper_bound + EPSILON {
            // FillGyroid.cpp:141 — result.emplace_back(make_wave(one_period_even, width, height, y0, scaleFactor, z_cos, z_sin, vertical, flip));
            result.push(make_wave(
                &one_period_even,
                width,
                height,
                y0,
                scale_factor,
                z_cos,
                z_sin,
                vertical,
                flip,
            ));
        }
        // FillGyroid.cpp:135 — y0 += M_PI (loop increment)
        y0 += PI;
    }

    // FillGyroid.cpp:145 — return result;
    result
}

// FillGyroid.cpp:148-149
//   // FIXME: needed to fix build on Mac on buildserver
//   constexpr double FillGyroid::PatternTolerance;

/// FillGyroid pattern generator.
///
/// FillGyroid.hpp:10 — `class FillGyroid : public Fill`.
///
/// The base `Slic3r::Fill` members this filler reads (`angle`, `z`, `spacing`)
/// are held here directly, mirroring the inherited C++ fields.
#[derive(Debug, Clone, Default)]
pub struct FillGyroid {
    /// Base `Fill::angle` in radians (FillBase.hpp).
    pub angle: f32,
    /// Base `Fill::z` in unscaled coordinates (FillBase.hpp).
    pub z: CoordF,
    /// Base `Fill::spacing` in unscaled coordinates (FillBase.hpp).
    pub spacing: CoordF,
}

impl FillGyroid {
    pub fn new(angle: f32, z: CoordF, spacing: CoordF) -> Self {
        Self { angle, z, spacing }
    }

    /// FillGyroid.hpp:17 — `bool use_bridge_flow() const override { return false; }`.
    pub fn use_bridge_flow(&self) -> bool {
        false
    }

    /// FillGyroid.hpp:18 — `bool is_self_crossing() override { return false; }`.
    pub fn is_self_crossing(&self) -> bool {
        false
    }

    // FillGyroid.cpp:151-210
    //   void FillGyroid::_fill_surface_single(
    //       const FillParams                &params,
    //       unsigned int                     thickness_layers,
    //       const std::pair<float, Point>   &direction,
    //       ExPolygon                        expolygon,
    //       Polylines                       &polylines_out)
    pub fn fill_surface_single(
        &mut self,
        params: &FillParams,
        _thickness_layers: u32,
        _direction: &(f32, Point),
        mut expolygon: ExPolygon,
        polylines_out: &mut Vec<Polyline>,
    ) {
        // FillGyroid.cpp:158 — auto infill_angle = float(this->angle + (CorrectionAngle * 2*M_PI) / 360.);
        let infill_angle =
            (self.angle as f64 + (CORRECTION_ANGLE as f64 * 2.0 * PI) / 360.0) as f32;
        // FillGyroid.cpp:159-160 — if(std::abs(infill_angle) >= EPSILON) expolygon.rotate(-infill_angle);
        if (infill_angle as f64).abs() >= EPSILON {
            expolygon.rotate(-infill_angle as CoordF);
        }

        // FillGyroid.cpp:162 — BoundingBox bb = expolygon.contour.bounding_box();
        let mut bb: BoundingBox = expolygon.contour.bounding_box();
        // FillGyroid.cpp:163-164 — Density adjusted to have a good %of weight.
        //   double density_adjusted = std::max(0., params.density * DensityAdjust / params.multiline);
        let density_adjusted =
            0.0_f64.max(params.density as f64 * DENSITY_ADJUST / params.multiline as f64);
        // FillGyroid.cpp:165-166 — Distance between the gyroid waves in scaled coordinates.
        //   coord_t distance = coord_t(scale_(this->spacing) / density_adjusted);
        // scale_(v) == v * SCALING_FACTOR; coord_t(...) truncates toward zero.
        let distance: Coord = ((self.spacing * SCALING_FACTOR) / density_adjusted) as Coord;

        // FillGyroid.cpp:168-169 — align bounding box to a multiple of our grid module
        //   bb.merge(align_to_grid(bb.min, Point(2*M_PI*distance, 2*M_PI*distance)));
        let grid = Point::new(
            (2.0 * PI * distance as f64) as Coord,
            (2.0 * PI * distance as f64) as Coord,
        );
        let aligned = align_to_grid_point(bb.min, grid);
        bb.merge_point(aligned);

        // FillGyroid.cpp:171-177 — generate pattern
        //   Polylines polylines = make_gyroid_waves(
        //       scale_(this->z),
        //       density_adjusted,
        //       this->spacing,
        //       ceil(bb.size()(0) / distance) + 1.,
        //       ceil(bb.size()(1) / distance) + 1.);
        let bb_size = bb.size();
        let mut polylines = make_gyroid_waves(
            (self.z * SCALING_FACTOR) as f64,
            density_adjusted,
            self.spacing,
            (bb_size.x as f64 / distance as f64).ceil() + 1.0,
            (bb_size.y as f64 / distance as f64).ceil() + 1.0,
        );

        // FillGyroid.cpp:179-181 — shift the polyline to the grid origin
        //   for (Polyline &pl : polylines)
        //       pl.translate(bb.min);
        for pl in &mut polylines {
            pl.translate(bb.min);
        }
        // FillGyroid.cpp:182-183 — Apply multiline offset if needed
        //   multiline_fill(polylines, params, spacing);
        multiline_fill(&mut polylines, params, self.spacing as f32);

        // FillGyroid.cpp:185 — polylines = intersection_pl(polylines, expolygon);
        // FIDELITY-NOTE(F1): geo-clipper approximation vs C++ ClipperLib
        // (clipper_utils::intersection_pl uses the `geo` crate at fixed scale
        // 1000, not ClipperLib at coord_t integer precision).
        let mut polylines: Vec<Polyline> =
            intersection_pl(&polylines, std::slice::from_ref(&expolygon));

        // FillGyroid.cpp:187 — if (! polylines.empty()) {
        if !polylines.is_empty() {
            // FillGyroid.cpp:188-190
            //   Remove very small bits, but be careful to not remove infill lines connecting thin walls!
            //   The infill perimeter lines should be separated by around a single infill line width.
            //   const double minlength = scale_(0.8 * this->spacing);
            let minlength = (0.8 * self.spacing) * SCALING_FACTOR;
            // FillGyroid.cpp:191-193
            //   polylines.erase(
            //       std::remove_if(polylines.begin(), polylines.end(), [minlength](const Polyline &pl) { return pl.length() < minlength; }),
            //       polylines.end());
            polylines.retain(|pl| !(pl.length() < minlength));
        }

        // FillGyroid.cpp:196 — if (! polylines.empty()) {
        if !polylines.is_empty() {
            // FillGyroid.cpp:197-198 — connect lines
            //   size_t polylines_out_first_idx = polylines_out.size();
            let polylines_out_first_idx = polylines_out.len();
            // FillGyroid.cpp:199-202
            //   if (params.dont_connect())
            //       append(polylines_out, chain_polylines(polylines));
            //   else
            //       this->connect_infill(std::move(polylines), expolygon, polylines_out, this->spacing, params);
            if params.dont_connect() {
                append(polylines_out, chain_polylines(polylines, None));
            } else {
                connect_infill_expolygon(
                    polylines,
                    &expolygon,
                    self.spacing,
                    params,
                    polylines_out,
                );
            }

            // FillGyroid.cpp:204-208 — new paths must be rotated back
            //   if (std::abs(infill_angle) >= EPSILON) {
            //       for (auto it = polylines_out.begin() + polylines_out_first_idx; it != polylines_out.end(); ++ it)
            //           it->rotate(infill_angle);
            //   }
            if (infill_angle as f64).abs() >= EPSILON {
                for pl in polylines_out.iter_mut().skip(polylines_out_first_idx) {
                    pl.rotate(infill_angle as CoordF);
                }
            }
        }
    }
}

// FillGyroid.cpp:212 — } // namespace Slic3r

/// `append(dst, src)` — Slic3r helper that moves all elements of `src` onto the
/// end of `dst`. Used at FillGyroid.cpp:200.
#[inline]
fn append(dst: &mut Vec<Polyline>, mut src: Vec<Polyline>) {
    if dst.is_empty() {
        *dst = src;
    } else {
        dst.append(&mut src);
    }
}

// ---------------------------------------------------------------------------
// Compatibility wrappers
//
// These are NOT part of FillGyroid.cpp; they expose the gyroid pattern through
// the simplified `(config, bb_min, bb_max)` API that the rest of this crate
// (`layer.rs`, `fill/mod.rs`) currently consumes. They reuse the faithful
// `make_gyroid_waves` port above so behaviour stays in sync with the C++.
// ---------------------------------------------------------------------------

/// Configuration for gyroid infill generation (crate-local convenience API).
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

/// Generate gyroid infill polylines for a bounding box (crate-local convenience
/// API). Mirrors the geometry-generating portion of
/// `FillGyroid::_fill_surface_single()` (grid alignment + wave generation +
/// translation to the grid origin), but skips the rotation/clipping/connection
/// steps which the callers handle themselves.
pub fn generate_gyroid_infill(
    config: &GyroidConfig,
    bb_min: Point,
    bb_max: Point,
) -> Vec<Polyline> {
    // FillGyroid.cpp:164 — density adjusted (no multiline here)
    let density_adjusted = (config.density * DENSITY_ADJUST).max(f64::MIN_POSITIVE);
    // FillGyroid.cpp:166 — coord_t distance = coord_t(scale_(spacing) / density_adjusted);
    let distance: Coord = ((config.spacing * SCALING_FACTOR) / density_adjusted) as Coord;

    // FillGyroid.cpp:162/168-169 — build a bounding box and align it to the grid module.
    let mut bb = BoundingBox::from_points_minmax(bb_min, bb_max);
    if distance != 0 {
        let grid = Point::new(
            (2.0 * PI * distance as f64) as Coord,
            (2.0 * PI * distance as f64) as Coord,
        );
        let aligned = align_to_grid_point(bb.min, grid);
        bb.merge_point(aligned);
    }

    let bb_size = bb.size();
    let (width, height) = if distance != 0 {
        (
            (bb_size.x as f64 / distance as f64).ceil() + 1.0,
            (bb_size.y as f64 / distance as f64).ceil() + 1.0,
        )
    } else {
        (1.0, 1.0)
    };

    // FillGyroid.cpp:172-177 — generate pattern.
    let mut polylines = make_gyroid_waves(
        scale(config.z) as f64,
        density_adjusted,
        config.spacing,
        width,
        height,
    );

    // FillGyroid.cpp:180-181 — shift the polyline to the grid origin.
    for pl in &mut polylines {
        pl.translate(bb.min);
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
        let pts = make_one_period(2.0 * PI, 1.0, z_cos, z_sin, false, false, 0.1);
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
