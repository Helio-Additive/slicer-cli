//! CrossHatch infill pattern.
//!
//! C++ Reference:
//! - Fill/FillCrossHatch.hpp
//! - Fill/FillCrossHatch.cpp
//!
//! Faithful 1:1 port of `Slic3r::FillCrossHatch` (FillCrossHatch.cpp).

// FillCrossHatch.cpp:1-6
//   #include "../ClipperUtils.hpp"
//   #include "../ShortestPath.hpp"
//   #include "../Surface.hpp"
//   #include <cmath>
//   #include "FillCrossHatch.hpp"
use super::{connect_infill_expolygon, multiline_fill, FillParams};
use crate::clipper_utils::intersection_pl;
use crate::geometry::{align_to_grid_point, BoundingBox, ExPolygon, Point, PointF, Polyline};
use crate::shortest_path::chain_polylines;
use crate::{Coord, CoordF, SCALING_FACTOR};

// FillCrossHatch.cpp:8 — namespace Slic3r

/// EPSILON, matching libslic3r's `EPSILON` (libslic3r.h:65 — `1e-4`).
const EPSILON: f64 = 1e-4;

/// `scale_(val)` macro — libslic3r.h:81 `#define scale_(val) ((val) / SCALING_FACTOR)`.
/// Returns an unrounded `double`; callers truncate toward zero when assigning to
/// `coord_t`.
#[inline]
fn scale_(val: f64) -> f64 {
    val / SCALING_FACTOR
}

/// `Point(double x, double y)` — Point.hpp:179
/// `Point(coord_t(lrint(x)), coord_t(lrint(y)))`. `lrint` rounds to the nearest
/// integer using the default round-half-to-even mode, matched here by
/// `f64::round_ties_even`.
#[inline]
fn point_round(x: f64, y: f64) -> Point {
    Point::new(x.round_ties_even() as Coord, y.round_ties_even() as Coord)
}

/// `Point(const Vec2d &rhs)` — Point.hpp:181
/// `Point(coord_t(lrint(rhs.x())), coord_t(lrint(rhs.y())))`.
#[inline]
fn point_from_vec2d(v: PointF) -> Point {
    point_round(v.x, v.y)
}

// CrossHatch Infill: Enhances 3D Printing Speed & Reduces Noise
// CrossHatch, as its name hints, alternates line direction by 90 degrees every few layers to improve adhesion.
// It introduces transform layers between direction shifts for better line cohesion, which fixes the weakness of line infill.
// The transform technique is inspired by David Eccles, improved 3D honeycomb but we made a more flexible implementation.
// This method notably increases printing speed, meeting the demands of modern high-speed 3D printers, and reduces noise for most layers.
// By Bambu Lab

// graph credits: David Eccles (gringer).
// But we made a different definition for points.
/*    o---o
 *   /     \
 *  /       \
 *           \       /
 *            \     /
 *             o---o
 *   p1   p2  p3   p4
 */

// FillCrossHatch.cpp:28-38
//   static Pointfs generate_one_cycle(double progress, coordf_t period)
fn generate_one_cycle(progress: f64, period: CoordF) -> Vec<PointF> {
    // FillCrossHatch.cpp:30
    let mut out: Vec<PointF> = Vec::new();
    // FillCrossHatch.cpp:31 — double offset = progress * 1. / 8. * period;
    let offset = progress * 1. / 8. * period;
    // FillCrossHatch.cpp:32 — out.reserve(4);
    out.reserve(4);
    // FillCrossHatch.cpp:33 — out.push_back(Vec2d(0.25 * period - offset, offset));
    out.push(PointF::new(0.25 * period - offset, offset));
    // FillCrossHatch.cpp:34 — out.push_back(Vec2d(0.25 * period + offset, offset));
    out.push(PointF::new(0.25 * period + offset, offset));
    // FillCrossHatch.cpp:35 — out.push_back(Vec2d(0.75 * period - offset, -offset));
    out.push(PointF::new(0.75 * period - offset, -offset));
    // FillCrossHatch.cpp:36 — out.push_back(Vec2d(0.75 * period + offset, -offset));
    out.push(PointF::new(0.75 * period + offset, -offset));
    // FillCrossHatch.cpp:37 — return out;
    out
}

// FillCrossHatch.cpp:40-108
//   static Polylines generate_transform_pattern(double inprogress, int direction, coordf_t ingrid_size, coordf_t inwidth, coordf_t inheight)
fn generate_transform_pattern(
    inprogress: f64,
    direction: i32,
    ingrid_size: CoordF,
    inwidth: CoordF,
    inheight: CoordF,
) -> Vec<Polyline> {
    // FillCrossHatch.cpp:42 — coordf_t width = inwidth;
    let mut width = inwidth;
    // FillCrossHatch.cpp:43 — coordf_t height = inheight;
    let mut height = inheight;
    // FillCrossHatch.cpp:44 — coordf_t grid_size = ingrid_size * 2; // we due with odd and even saparately.
    let grid_size = ingrid_size * 2.0; // we due with odd and even saparately.
    // FillCrossHatch.cpp:45 — double progress = inprogress;
    let progress = inprogress;
    // FillCrossHatch.cpp:46 — Polylines out_polylines;
    let mut out_polylines: Vec<Polyline> = Vec::new();

    // FillCrossHatch.cpp:48-49 — generate template patterns;
    //   Pointfs one_cycle_points = generate_one_cycle(progress, grid_size);
    let one_cycle_points = generate_one_cycle(progress, grid_size);

    // FillCrossHatch.cpp:51-53
    //   Polyline one_cycle;
    //   one_cycle.points.reserve(one_cycle_points.size());
    //   for (size_t i = 0; i < one_cycle_points.size(); i++) one_cycle.points.push_back(Point(one_cycle_points[i]));
    let mut one_cycle = Polyline::new();
    one_cycle.points.reserve(one_cycle_points.len());
    for i in 0..one_cycle_points.len() {
        one_cycle.points.push(point_from_vec2d(one_cycle_points[i]));
    }

    // FillCrossHatch.cpp:55-59 — swap if vertical
    //   if (direction < 0) {
    //       width  = height;
    //       height = inwidth;
    //   }
    if direction < 0 {
        width = height;
        height = inwidth;
    }

    // FillCrossHatch.cpp:61-65 — replicate polylines;
    //   Polylines odd_polylines;
    //   Polyline  odd_poly;
    //   int       num_of_cycle = width / grid_size + 2;
    //   odd_poly.points.reserve(num_of_cycle * one_cycle.size());
    let mut odd_polylines: Vec<Polyline> = Vec::new();
    let mut odd_poly = Polyline::new();
    let num_of_cycle: i32 = (width / grid_size + 2.0) as i32;
    odd_poly
        .points
        .reserve((num_of_cycle as usize) * one_cycle.size());

    // FillCrossHatch.cpp:67-74 — replicate to odd line
    //   Point translate = Point(0, 0);
    //   for (size_t i = 0; i < num_of_cycle; i++) {
    //       Polyline odd_points;
    //       odd_points = Polyline(one_cycle);
    //       odd_points.translate(Point(i * grid_size, 0.0));
    //       odd_poly.points.insert(odd_poly.points.end(), odd_points.begin(), odd_points.end());
    //   }
    let _translate = Point::new(0, 0);
    for i in 0..(num_of_cycle as usize) {
        let mut odd_points: Polyline;
        odd_points = one_cycle.clone();
        odd_points.translate(point_round(i as f64 * grid_size, 0.0));
        odd_poly
            .points
            .extend_from_slice(&odd_points.points);
    }

    // FillCrossHatch.cpp:76-83 — fill the height
    //   int num_of_lines = height / grid_size + 2;
    //   odd_polylines.reserve(num_of_lines * odd_poly.size());
    //   for (size_t i = 0; i < num_of_lines; i++) {
    //       Polyline poly = odd_poly;
    //       poly.translate(Point(0.0, grid_size * i));
    //       odd_polylines.push_back(poly);
    //   }
    let num_of_lines: i32 = (height / grid_size + 2.0) as i32;
    odd_polylines.reserve((num_of_lines as usize) * odd_poly.size());
    for i in 0..(num_of_lines as usize) {
        let mut poly = odd_poly.clone();
        poly.translate(point_round(0.0, grid_size * i as f64));
        odd_polylines.push(poly);
    }
    // FillCrossHatch.cpp:84-85 — save to output
    //   out_polylines.insert(out_polylines.end(), odd_polylines.begin(), odd_polylines.end());
    out_polylines.extend(odd_polylines.iter().cloned());

    // FillCrossHatch.cpp:87-94 — replicate to even lines
    //   Polylines even_polylines;
    //   even_polylines.reserve(odd_polylines.size());
    //   for (size_t i = 0; i < odd_polylines.size(); i++) {
    //       Polyline even = odd_poly;
    //       even.translate(Point(-0.5 * grid_size, (i + 0.5) * grid_size));
    //       even_polylines.push_back(even);
    //   }
    let mut even_polylines: Vec<Polyline> = Vec::new();
    even_polylines.reserve(odd_polylines.len());
    for i in 0..odd_polylines.len() {
        let mut even = odd_poly.clone();
        even.translate(point_round(-0.5 * grid_size, (i as f64 + 0.5) * grid_size));
        even_polylines.push(even);
    }

    // FillCrossHatch.cpp:96-97 — save for output
    //   out_polylines.insert(out_polylines.end(), even_polylines.begin(), even_polylines.end());
    out_polylines.extend(even_polylines);

    // FillCrossHatch.cpp:99-105 — change to vertical if need
    //   if (direction < 0) {
    //       // swap xy, see if we need better performance method
    //       for (Polyline &poly : out_polylines) {
    //           for (Point &p : poly) { std::swap(p.x(), p.y()); }
    //       }
    //   }
    if direction < 0 {
        // swap xy, see if we need better performance method
        for poly in &mut out_polylines {
            for p in poly.points_mut() {
                std::mem::swap(&mut p.x, &mut p.y);
            }
        }
    }

    // FillCrossHatch.cpp:107 — return out_polylines;
    out_polylines
}

// FillCrossHatch.cpp:110-142
//   static Polylines generate_repeat_pattern(int direction, coordf_t grid_size, coordf_t inwidth, coordf_t inheight)
fn generate_repeat_pattern(
    direction: i32,
    grid_size: CoordF,
    inwidth: CoordF,
    inheight: CoordF,
) -> Vec<Polyline> {
    // FillCrossHatch.cpp:112 — coordf_t width  = inwidth;
    let mut width = inwidth;
    // FillCrossHatch.cpp:113 — coordf_t height = inheight;
    let mut height = inheight;
    // FillCrossHatch.cpp:114 — Polylines out_polylines;
    let mut out_polylines: Vec<Polyline> = Vec::new();

    // FillCrossHatch.cpp:116-120 — swap if vertical
    //   if (direction < 0) {
    //       width  = height;
    //       height = inwidth;
    //   }
    if direction < 0 {
        width = height;
        height = inwidth;
    }

    // FillCrossHatch.cpp:122-123
    //   int num_of_lines = height / grid_size + 1;
    //   out_polylines.reserve(num_of_lines);
    let num_of_lines: i32 = (height / grid_size + 1.0) as i32;
    out_polylines.reserve(num_of_lines as usize);

    // FillCrossHatch.cpp:125-131
    //   for (int i = 0; i < num_of_lines; i++) {
    //       Polyline poly;
    //       poly.points.reserve(2);
    //       poly.append(Point(coordf_t(0), coordf_t(grid_size * i)));
    //       poly.append(Point(width, coordf_t(grid_size * i)));
    //       out_polylines.push_back(poly);
    //   }
    for i in 0..num_of_lines {
        let mut poly = Polyline::new();
        poly.points.reserve(2);
        poly.points.push(point_round(0.0, grid_size * i as f64));
        poly.points.push(point_round(width, grid_size * i as f64));
        out_polylines.push(poly);
    }

    // FillCrossHatch.cpp:133-139 — change to vertical if needed
    //   if (direction < 0) {
    //       // swap xy
    //       for (Polyline &poly : out_polylines) {
    //           for (Point &p : poly) { std::swap(p.x(), p.y()); }
    //       }
    //   }
    if direction < 0 {
        // swap xy
        for poly in &mut out_polylines {
            for p in poly.points_mut() {
                std::mem::swap(&mut p.x, &mut p.y);
            }
        }
    }

    // FillCrossHatch.cpp:141 — return out_polylines;
    out_polylines
}

// FillCrossHatch.cpp:144-176
// it makes the real patterns that overlap the bounding box
// repeat_ratio define the ratio between the height of repeat pattern and grid
//   static Polylines generate_infill_layers(coordf_t z_height, double repeat_ratio, coordf_t grid_size, coordf_t width, coordf_t height)
fn generate_infill_layers(
    mut z_height: CoordF,
    repeat_ratio: f64,
    grid_size: CoordF,
    width: CoordF,
    height: CoordF,
) -> Vec<Polyline> {
    // FillCrossHatch.cpp:148 — Polylines result;
    let result: Vec<Polyline>;
    // FillCrossHatch.cpp:149 — coordf_t trans_layer_size  = grid_size * 0.4; // upper.
    let trans_layer_size = grid_size * 0.4; // upper.
    // FillCrossHatch.cpp:150 — coordf_t repeat_layer_size = grid_size * repeat_ratio; // lower.
    let repeat_layer_size = grid_size * repeat_ratio; // lower.
    // FillCrossHatch.cpp:151 — z_height += repeat_layer_size / 2 ; // offset to improve first few layer strength and reduce the risk of warpping.
    z_height += repeat_layer_size / 2.0; // offset to improve first few layer strength and reduce the risk of warpping.
    // FillCrossHatch.cpp:152 — coordf_t period = trans_layer_size + repeat_layer_size;
    let period = trans_layer_size + repeat_layer_size;
    // FillCrossHatch.cpp:153 — coordf_t remains = z_height - std::floor(z_height / period) * period;
    let remains = z_height - (z_height / period).floor() * period;
    // FillCrossHatch.cpp:154 — coordf_t trans_z = remains - repeat_layer_size; // put repeat layer first.
    let trans_z = remains - repeat_layer_size; // put repeat layer first.
    // FillCrossHatch.cpp:155 — coordf_t repeat_z = remains;
    let _repeat_z = remains;

    // FillCrossHatch.cpp:157 — int phase = fmod(z_height, period * 2) - (period - 1); // add epsilon
    let phase: i32 = ((z_height % (period * 2.0)) - (period - 1.0)) as i32; // add epsilon
    // FillCrossHatch.cpp:158 — int direction = phase <= 0 ? -1 : 1;
    let direction: i32 = if phase <= 0 { -1 } else { 1 };

    // FillCrossHatch.cpp:160-163 — this is a repeat layer
    //   if (trans_z < 0) {
    //       result = generate_repeat_pattern(direction, grid_size, width, height);
    //   }
    if trans_z < 0.0 {
        result = generate_repeat_pattern(direction, grid_size, width, height);
    }
    // FillCrossHatch.cpp:164-173 — this is a transform layer
    //   else {
    //       double progress = fmod(trans_z, trans_layer_size) / trans_layer_size;
    //       // split the progress to forward and backward, with a opposite direction.
    //       if (progress < 0.5)
    //           result = generate_transform_pattern((progress + 0.1) * 2, direction, grid_size, width, height); // increase overlapping.
    //       else
    //           result = generate_transform_pattern((1.1 - progress) * 2, -1 * direction, grid_size, width, height);
    //   }
    else {
        let progress = (trans_z % trans_layer_size) / trans_layer_size;

        // split the progress to forward and backward, with a opposite direction.
        if progress < 0.5 {
            result = generate_transform_pattern((progress + 0.1) * 2.0, direction, grid_size, width, height);
        // increase overlapping.
        } else {
            result = generate_transform_pattern((1.1 - progress) * 2.0, -1 * direction, grid_size, width, height);
        }
    }

    // FillCrossHatch.cpp:175 — return result;
    result
}

/// `Slic3r::FillCrossHatch` — FillCrossHatch.hpp:12 `class FillCrossHatch : public Fill`.
///
/// The base `Slic3r::Fill` members this filler reads (`angle`, `z`, `spacing`)
/// are held here directly, mirroring the inherited C++ fields.
#[derive(Debug, Clone, Default)]
pub struct FillCrossHatch {
    /// Base `Fill::angle` in radians (FillBase.hpp).
    pub angle: f32,
    /// Base `Fill::z` in unscaled coordinates (FillBase.hpp).
    pub z: CoordF,
    /// Base `Fill::spacing` in unscaled coordinates (FillBase.hpp).
    pub spacing: CoordF,
}

impl FillCrossHatch {
    pub fn new(angle: f32, z: CoordF, spacing: CoordF) -> Self {
        Self { angle, z, spacing }
    }

    /// FillCrossHatch.hpp:17 — `bool is_self_crossing() override { return false; }`.
    pub fn is_self_crossing(&self) -> bool {
        false
    }

    // FillCrossHatch.cpp:178-235
    //   void FillCrossHatch::_fill_surface_single(
    //       const FillParams &params, unsigned int thickness_layers, const std::pair<float, Point> &direction, ExPolygon expolygon, Polylines &polylines_out)
    pub fn fill_surface_single(
        &mut self,
        params: &FillParams,
        _thickness_layers: u32,
        _direction: &(f32, Point),
        mut expolygon: ExPolygon,
        polylines_out: &mut Vec<Polyline>,
    ) {
        // FillCrossHatch.cpp:181-183 — rotate angle
        //   auto infill_angle = float(this->angle);
        //   if (std::abs(infill_angle) >= EPSILON) expolygon.rotate(-infill_angle);
        let infill_angle = self.angle;
        if (infill_angle as f64).abs() >= EPSILON {
            expolygon.rotate(-infill_angle as CoordF);
        }

        // FillCrossHatch.cpp:185-186 — get the rotated bounding box
        //   BoundingBox bb = expolygon.contour.bounding_box();
        let mut bb: BoundingBox = expolygon.contour.bounding_box();

        // FillCrossHatch.cpp:188-190 — linespace modifier
        //   double density_adjusted = params.density / params.multiline;
        //   coord_t line_spacing = coord_t(scale_(this->spacing) / density_adjusted);
        let density_adjusted = params.density as f64 / params.multiline as f64;
        let mut line_spacing: Coord = (scale_(self.spacing) / density_adjusted) as Coord;

        // FillCrossHatch.cpp:192-193 — reduce density
        //   if (params.density < 0.999) line_spacing *= 1.08;
        if params.density < 0.999 {
            line_spacing = (line_spacing as f64 * 1.08) as Coord;
        }

        // FillCrossHatch.cpp:195 — bb.merge(align_to_grid(bb.min, Point(line_spacing * 4, line_spacing * 4)));
        bb.merge_point(align_to_grid_point(
            bb.min,
            Point::new(line_spacing * 4, line_spacing * 4),
        ));

        // FillCrossHatch.cpp:197-201 — generate pattern
        //   //Orca: optimize the cross-hatch infill pattern to improve strength when low infill density is used.
        //   double repeat_ratio = 1.0;
        //   if (params.density < 0.3)
        //       repeat_ratio = std::clamp(1.0 - std::exp(-5 * params.density), 0.2, 1.0);
        //Orca: optimize the cross-hatch infill pattern to improve strength when low infill density is used.
        let mut repeat_ratio: f64 = 1.0;
        if params.density < 0.3 {
            repeat_ratio = (1.0 - (-5.0 * params.density as f64).exp()).clamp(0.2, 1.0);
        }

        // FillCrossHatch.cpp:203 — Polylines polylines = generate_infill_layers(scale_(this->z), repeat_ratio, line_spacing, bb.size()(0), bb.size()(1));
        let bb_size = bb.size();
        let mut polylines = generate_infill_layers(
            scale_(self.z),
            repeat_ratio,
            line_spacing as CoordF,
            bb_size.x as CoordF,
            bb_size.y as CoordF,
        );

        // FillCrossHatch.cpp:205-206 — shift the pattern to the actual space
        //   for (Polyline &pl : polylines) { pl.translate(bb.min); }
        for pl in &mut polylines {
            pl.translate(bb.min);
        }

        // FillCrossHatch.cpp:208-209 — Apply multiline offset if needed
        //   multiline_fill(polylines, params, spacing);
        multiline_fill(&mut polylines, params, self.spacing as f32);

        // FillCrossHatch.cpp:211 — polylines = intersection_pl(polylines, to_polygons(expolygon));
        // FIDELITY-NOTE(F1): geo-clipper approximation vs C++ ClipperLib — `intersection_pl`
        // clips the infill against the ExPolygon (contour minus holes), matching
        // `to_polygons(expolygon)` semantically, but the underlying boolean op does not run at
        // ClipperLib coord_t integer precision.
        let mut polylines: Vec<Polyline> =
            intersection_pl(&polylines, std::slice::from_ref(&expolygon));

        // FillCrossHatch.cpp:213-220 — remove small remains from gyroid infill
        //   if (!polylines.empty()) {
        //       // Remove very small bits, but be careful to not remove infill lines connecting thin walls!
        //       // The infill perimeter lines should be separated by around a single infill line width.
        //       const double minlength = scale_(0.8 * this->spacing);
        //       polylines.erase(std::remove_if(polylines.begin(), polylines.end(), [minlength](const Polyline &pl)
        //           { return pl.length() < minlength; }), polylines.end());
        //   }
        if !polylines.is_empty() {
            // Remove very small bits, but be careful to not remove infill lines connecting thin walls!
            // The infill perimeter lines should be separated by around a single infill line width.
            let minlength = scale_(0.8 * self.spacing);
            polylines.retain(|pl| !(pl.length() < minlength));
        }

        // FillCrossHatch.cpp:222-234
        //   if (!polylines.empty()) {
        //       int infill_start_idx = polylines_out.size(); // only rotate what belongs to us.
        //       // connect lines
        //       if (params.dont_connect() || polylines.size() <= 1)
        //           append(polylines_out, chain_polylines(std::move(polylines)));
        //       else
        //           this->connect_infill(std::move(polylines), expolygon, polylines_out, this->spacing, params);
        //       // rotate back
        //       if (std::abs(infill_angle) >= EPSILON) {
        //           for (auto it = polylines_out.begin() + infill_start_idx; it != polylines_out.end(); ++it) it->rotate(infill_angle);
        //       }
        //   }
        if !polylines.is_empty() {
            let infill_start_idx = polylines_out.len(); // only rotate what belongs to us.
            // connect lines
            if params.dont_connect() || polylines.len() <= 1 {
                append(polylines_out, chain_polylines(polylines, None));
            } else {
                connect_infill_expolygon(polylines, &expolygon, self.spacing, params, polylines_out);
            }

            // rotate back
            if (infill_angle as f64).abs() >= EPSILON {
                for pl in polylines_out.iter_mut().skip(infill_start_idx) {
                    pl.rotate(infill_angle as CoordF);
                }
            }
        }
    }
}

// FillCrossHatch.cpp:237 — } // namespace Slic3r

/// `append(dst, src)` — Slic3r helper that moves all elements of `src` onto the
/// end of `dst`. Used at FillCrossHatch.cpp:226.
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
// These are NOT part of FillCrossHatch.cpp; they expose the cross-hatch pattern
// through the simplified `(config / density)` API that the rest of this crate
// (`fill/mod.rs`) currently consumes. They reuse the faithful
// `FillCrossHatch::fill_surface_single` port above so behaviour stays in sync
// with the C++.
// ---------------------------------------------------------------------------

/// Configuration for cross-hatch infill generation (crate-local convenience API).
#[derive(Debug, Clone)]
pub struct CrossHatchConfig {
    /// Layer Z height in mm.
    pub z: f64,
    /// Fill angle in radians.
    pub angle: f64,
    /// Line spacing in mm.
    pub spacing: f64,
    /// Fill density (0.0 to 1.0).
    pub density: f64,
}

impl Default for CrossHatchConfig {
    fn default() -> Self {
        Self {
            z: 0.0,
            angle: 0.0,
            spacing: 0.4,
            density: 0.2,
        }
    }
}

/// Result of cross-hatch infill generation (crate-local convenience API).
#[derive(Debug, Clone, Default)]
pub struct CrossHatchResult {
    /// Generated polylines representing the infill pattern.
    pub polylines: Vec<Polyline>,
}

impl CrossHatchResult {
    /// Check if any infill was generated.
    pub fn has_infill(&self) -> bool {
        !self.polylines.is_empty()
    }

    /// Get the total number of polylines.
    pub fn polyline_count(&self) -> usize {
        self.polylines.len()
    }
}

/// Convenience helper: generate cross-hatch infill for a set of fill regions
/// (crate-local API). Drives the faithful `FillCrossHatch::fill_surface_single`
/// once per region.
pub fn generate_cross_hatch_with_angle(
    fill_area: &[ExPolygon],
    z_height_mm: CoordF,
    spacing: CoordF,
    density: CoordF,
    angle: CoordF,
) -> CrossHatchResult {
    let mut result = CrossHatchResult::default();
    if fill_area.is_empty() || density <= 0.0 {
        return result;
    }

    let mut filler = FillCrossHatch::new(angle as f32, z_height_mm, spacing);
    let mut params = FillParams::new();
    params.density = density;
    // chain the lines without connecting through the perimeter in this helper.
    params.anchor_length_max = 0.0;
    let direction = (angle as f32, Point::new(0, 0));

    for expoly in fill_area {
        filler.fill_surface_single(
            &params,
            1,
            &direction,
            expoly.clone(),
            &mut result.polylines,
        );
    }

    result
}
