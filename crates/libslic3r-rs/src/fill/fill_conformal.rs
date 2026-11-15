//! Conformal infill: sparse infill that follows each layer's outline.
//!
//! C++ Reference:
//! - Fill/FillConformal.hpp
//! - Fill/FillConformal.cpp
//!
//! Line-by-line port of `Slic3r::FillConformal`. The entry is `fill_surface`,
//! called by `FillRectilinear::fill_surface` (FillRectilinear.cpp:3077-3081)
//! when `FillParams::conformal` is set: an equal-angle radial zigzag around a
//! locked pole (`FillRadialZigZag::fill_region`) and rectilinear lines in the
//! circle around the pole. The older polar / medial-axis / chart paths are kept
//! in the C++ file but not called; they are ported here in the same state.
//!
//! Floating point: the C++ program is built with `-ffp-contract=on`, so a
//! scalar `a * b + c` of ONE source expression is a fused multiply-add; those
//! places are written with `mul_add` here, and nowhere else.

// FillConformal.cpp:1-18
//   #include "FillConformal.hpp"
//   #include "FillConformalChart.hpp"
//   #include "FillRadialZigZag.hpp"
//   #include "FillRectilinear.hpp"
//   #include "../ClipperUtils.hpp"
//   #include "../ExPolygon.hpp"
//   #include "../BoundingBox.hpp"
//   #include "../Geometry.hpp"
//   #include "../Line.hpp"
//   #include "../Point.hpp"
//   #include "../Polygon.hpp"
//   #include "../Surface.hpp"
#![allow(dead_code)]

use std::f64::consts::PI;

use super::fill_base::Fill;
use super::fill_conformal_chart::{extract_generator, LayerGenerator};
use super::fill_radial_zig_zag;
use super::fill_rectilinear::{fill_surface_by_lines, FillRectilinearParams};
use super::FillParams;
use crate::clipper_utils::{
    diff_pl_clib, intersection_clib, intersection_pl_clib, offset_expolygons_clib_scaled, OffsetJoinType,
};
use crate::geometry::{BoundingBox, ExPolygon, Line, Point, Polygon, Polyline};
use crate::print_config::{ConformalStagger, InfillPattern};
use crate::surface::Surface;
use crate::Coord;

// FillConformal.cpp:20-21 — namespace Slic3r { namespace FillConformal {

/// libslic3r.h:52 — `static constexpr double SCALING_FACTOR = 0.00001;`
const SCALING_FACTOR: f64 = 0.00001;

/// libslic3r.h:60 — `#define scale_(val) ((val) / SCALING_FACTOR)`: a double.
#[inline]
fn scale_(val: f64) -> f64 {
    val / SCALING_FACTOR
}

/// Point.hpp:540 — `scaled<coord_t>(v)`: `coord_t(v / SCALING_FACTOR)`, truncating.
#[inline]
fn scaled_coord(v: f64) -> Coord {
    (v / SCALING_FACTOR) as Coord
}

/// libslic3r.h — `unscale<double>(v)`: `double(v) * double(SCALING_FACTOR)`.
#[inline]
fn unscale_d(v: f64) -> f64 {
    v * SCALING_FACTOR
}

/// `(a - b).cast<double>().norm()` — Eigen, not fused.
#[inline]
fn diff_norm(a: Point, b: Point) -> f64 {
    let dx = (a.x - b.x) as f64;
    let dy = (a.y - b.y) as f64;
    (dx * dx + dy * dy).sqrt()
}

/// `get_extents(const ExPolygon&)` (the contour's box).
#[inline]
fn get_extents(ex: &ExPolygon) -> BoundingBox {
    BoundingBox::from_points(&ex.contour.points)
}

/// ClipperUtils.cpp:567-569 — `offset_ex(const ExPolygon&, const float delta)`.
fn offset_ex(expolygon: &ExPolygon, delta: f32) -> Vec<ExPolygon> {
    offset_expolygons_clib_scaled(std::slice::from_ref(expolygon), delta as f64, OffsetJoinType::Miter)
}

/// ClipperUtils.cpp:925-926 — `intersection_pl(const Polyline&, const ExPolygon&)`.
fn intersection_pl(subject: &Polyline, clip: &ExPolygon) -> Vec<Polyline> {
    intersection_pl_clib(std::slice::from_ref(subject), std::slice::from_ref(clip))
}

/// ExPolygon.cpp:76-90 — `ExPolygon::contains(const Line &line)`, through
/// `ExPolygon::contains(const Polyline&)`: the boxes overlap and
/// `diff_pl(polyline, *this)` (ClipperUtils.cpp:913-914) is empty.
fn expolygon_contains_line(ex: &ExPolygon, a: Point, b: Point) -> bool {
    let bbox1 = get_extents(ex);
    // ExPolygon.cpp:85 — `bbox2.inflated(1);` discards its result.
    let (min2x, max2x) = (a.x.min(b.x), a.x.max(b.x));
    let (min2y, max2y) = (a.y.min(b.y), a.y.max(b.y));
    if bbox1.max.x < min2x || bbox1.min.x > max2x || bbox1.max.y < min2y || bbox1.min.y > max2y {
        return false;
    }
    let polyline = Polyline::from_points(vec![a, b]);
    let mut clip: Vec<Polygon> = Vec::with_capacity(1 + ex.holes.len());
    clip.push(ex.contour.clone());
    clip.extend(ex.holes.iter().cloned());
    diff_pl_clib(std::slice::from_ref(&polyline), &clip).is_empty()
}

/// FillConformal.cpp:23-27 — `struct Centerline`
#[derive(Debug, Clone, Default)]
struct Centerline {
    polyline: Polyline,
    closed: bool,
}

/// FillConformal.cpp:29-35 — `static Centerline to_centerline(const LayerGenerator &lg)`
fn to_centerline(lg: &LayerGenerator) -> Centerline {
    let mut g = Centerline::default();
    g.polyline = lg.polyline.clone();
    g.closed = lg.closed;
    g
}

/// FillConformal.cpp:37-40 — `static size_t layer_index(const Fill &fill)`
fn layer_index(fill: &Fill) -> usize {
    if fill.layer_id == usize::MAX {
        0
    } else {
        fill.layer_id
    }
}

/// FillConformal.cpp:42-48 — `static BoundingBox object_or_region_bb(const Fill &fill, const ExPolygon &region)`
fn object_or_region_bb(fill: &Fill, region: &ExPolygon) -> BoundingBox {
    let mut bb = fill.bounding_box.clone();
    if bb.max.x - bb.min.x <= 0 || bb.max.y - bb.min.y <= 0 {
        bb = get_extents(region);
    }
    bb
}

/// FillConformal.cpp:50-53 — `static double line_spacing_mm(const Fill &fill, const FillParams &params)`
fn line_spacing_mm(fill: &Fill, params: &FillParams) -> f64 {
    // `double(params.density)`: the C++ member is a float.
    fill.spacing / (0.05_f64).max((params.density as f32) as f64)
}

// Polar diameters: N from the object AABB long side so disk/cylinder tests keep a stable family.
/// FillConformal.cpp:56-62 — `static int stable_polar_count(const Fill &fill, const FillParams &params, const ExPolygon &region)`
fn stable_polar_count(fill: &Fill, params: &FillParams, region: &ExPolygon) -> i32 {
    let bb = object_or_region_bb(fill, region);
    let char_len = unscale_d((bb.max.x - bb.min.x).max(bb.max.y - bb.min.y) as f64);
    let delta = line_spacing_mm(fill, params);
    std::cmp::max(2, (char_len / delta.max(1e-6)).round() as i32)
}

// Space samples by target line spacing along a generator span.
/// FillConformal.cpp:65-69 — `static int chart_line_count(double L_scaled, double delta_mm)`
fn chart_line_count(l_scaled: f64, delta_mm: f64) -> i32 {
    let lmm = unscale_d(l_scaled);
    std::cmp::max(2, (lmm / delta_mm.max(1e-6)).round() as i32)
}

/// FillConformal.cpp:71-78 — `static double theta0_for_layer(const Fill &fill, const FillParams &params, int N)`
fn theta0_for_layer(fill: &Fill, params: &FillParams, n: i32) -> f64 {
    let mut theta0 = 0.0_f64;
    let odd = (layer_index(fill) % 2) == 1;
    if odd && params.conformal_stagger == ConformalStagger::HalfStep {
        theta0 += PI / (2.0 * n as f64);
    }
    theta0
}

/// FillConformal.cpp:80-83 — `static Vec2d unit_from_angle(double theta)`
fn unit_from_angle(theta: f64) -> (f64, f64) {
    (theta.cos(), theta.sin())
}

/// FillConformal.cpp:85-95 — `static void append_closed_polygon(const Polygon &poly, Polylines &out)`
fn append_closed_polygon(poly: &Polygon, out: &mut Vec<Polyline>) {
    if poly.points.len() < 3 {
        return;
    }
    let mut pl = Polyline::from_points(poly.points.clone());
    if pl.points[0] != pl.points[pl.points.len() - 1] {
        let front = pl.points[0];
        pl.points.push(front);
    }
    if pl.length() > scale_(0.2) {
        out.push(pl);
    }
}

/// FillConformal.cpp:97-115 — `static bool fill_offset_loops(const ExPolygon &region, const Fill &fill, const FillParams &params, Polylines &out)`
fn fill_offset_loops(region: &ExPolygon, fill: &Fill, params: &FillParams, out: &mut Vec<Polyline>) -> bool {
    let step = scale_(line_spacing_mm(fill, params));
    if step <= 0.0 {
        return false;
    }
    let bb = get_extents(region);
    let max_in = 0.5 * bb.size().x.min(bb.size().y) as f64;
    let mut any = false;
    let mut o = step;
    // o < max_in - 0.5 * step: fused.
    while o < (-0.5_f64).mul_add(step, max_in) {
        let inner = offset_ex(region, (-o) as f32);
        for ex in &inner {
            append_closed_polygon(&ex.contour, out);
            for hole in &ex.holes {
                append_closed_polygon(hole, out);
            }
            any = true;
        }
        o += step;
    }
    any && !out.is_empty()
}

/// FillConformal.cpp:117-133 — `static void clip_ray_to_region(const Point &origin, const Vec2d &dir, const ExPolygon &region, Polylines &scans)`
fn clip_ray_to_region(origin: &Point, dir: (f64, f64), region: &ExPolygon, scans: &mut Vec<Polyline>) {
    let bb = get_extents(region);
    let ray_len = bb.size().x.max(bb.size().y) as f64 + scaled_coord(2.0) as f64;
    let mut n = dir;
    let nn = (n.0 * n.0 + n.1 * n.1).sqrt();
    if nn < 1e-9 {
        return;
    }
    n = (n.0 / nn, n.1 / nn);
    // coord_t(n.x() * ray_len): truncating.
    let off = Point::new((n.0 * ray_len) as Coord, (n.1 * ray_len) as Coord);
    let ray = Polyline::from_points(vec![
        Point::new(origin.x - off.x, origin.y - off.y),
        Point::new(origin.x + off.x, origin.y + off.y),
    ]);
    let hits = intersection_pl(&ray, region);
    for hit in hits {
        if hit.length() > scale_(0.2) {
            scans.push(hit);
        }
    }
}

/// FillConformal.cpp:135-141 — `static double polyline_dist_to_point(const Polyline &pl, const Point &p)`
fn polyline_dist_to_point(pl: &Polyline, p: &Point) -> f64 {
    let mut best = f64::MAX;
    for i in 1..pl.points.len() {
        // Line::distance_to(p, a, b) is sqrt(distance_to_squared(p, a, b)).
        best = best.min(Line::distance_to_squared(*p, pl.points[i - 1], pl.points[i]).sqrt());
    }
    best
}

// Keep the fragment that belongs to this sample (local wall thickness / local width),
// not the far-side hit across a hole.
/// FillConformal.cpp:145-161 — `static void clip_local_chord(const Point &origin, const Vec2d &dir, const ExPolygon &region, Polylines &scans)`
fn clip_local_chord(origin: &Point, dir: (f64, f64), region: &ExPolygon, scans: &mut Vec<Polyline>) {
    let mut hits: Vec<Polyline> = Vec::new();
    clip_ray_to_region(origin, dir, region, &mut hits);
    if hits.is_empty() {
        return;
    }
    let mut best = 0usize;
    let mut best_d = polyline_dist_to_point(&hits[0], origin);
    for i in 1..hits.len() {
        let d = polyline_dist_to_point(&hits[i], origin);
        if d < best_d {
            best_d = d;
            best = i;
        }
    }
    scans.push(hits.swap_remove(best));
}

/// FillConformal.cpp:163-172 — `static bool fill_polar_rays(const ExPolygon &region, const Fill &fill, const FillParams &params, Polylines &scans)`
fn fill_polar_rays(region: &ExPolygon, fill: &Fill, params: &FillParams, scans: &mut Vec<Polyline>) -> bool {
    let c = fill_radial_zig_zag::polygon_centroid(&region.contour);
    let n = stable_polar_count(fill, params, region);
    let theta0 = theta0_for_layer(fill, params, n);
    let dtheta = PI / n as f64;
    for k in 0..n {
        // theta0 + k * dtheta: fused.
        clip_ray_to_region(&c, unit_from_angle((k as f64).mul_add(dtheta, theta0)), region, scans);
    }
    scans.len() >= 2
}

/// FillConformal.cpp:174-177 — `static double generator_length(const Centerline &g)`
fn generator_length(g: &Centerline) -> f64 {
    g.polyline.length()
}

/// FillConformal.cpp:179-217 — `static bool sample_generator(const Centerline &g, double s, Point &p, Vec2d &tangent)`
fn sample_generator(g: &Centerline, mut s: f64, p: &mut Point, tangent: &mut (f64, f64)) -> bool {
    let pts = &g.polyline.points;
    if pts.len() < 2 {
        return false;
    }
    let closed = g.closed && pts.len() >= 3 && pts[0] == pts[pts.len() - 1];
    let nseg = pts.len() - 1;
    let mut l = 0.0_f64;
    for i in 0..nseg {
        l += diff_norm(pts[i + 1], pts[i]);
    }
    if l < 1.0 {
        return false;
    }
    if closed {
        s %= l; // std::fmod
        if s < 0.0 {
            s += l;
        }
    } else {
        s = (0.0_f64).max(s.min(l));
    }
    let mut acc = 0.0_f64;
    for i in 0..nseg {
        let d = ((pts[i + 1].x - pts[i].x) as f64, (pts[i + 1].y - pts[i].y) as f64);
        let len = (d.0 * d.0 + d.1 * d.1).sqrt();
        if len < 1.0 {
            continue;
        }
        if acc + len >= s - 1e-3 {
            let t = (0.0_f64).max((1.0_f64).min((s - acc) / len));
            // std::lround: half away from zero.
            *p = Point::new(pts[i].x + (d.0 * t).round() as Coord, pts[i].y + (d.1 * t).round() as Coord);
            *tangent = (d.0 / len, d.1 / len);
            return true;
        }
        acc += len;
    }
    *p = pts[nseg];
    let d = ((pts[nseg].x - pts[nseg - 1].x) as f64, (pts[nseg].y - pts[nseg - 1].y) as f64);
    let ln = (d.0 * d.0 + d.1 * d.1).sqrt();
    *tangent = if ln > 1.0 { (d.0 / ln, d.1 / ln) } else { (1.0, 0.0) };
    true
}

/// FillConformal.cpp:227-231 — the lambda `dist_to_ray` of `s0_world_plus_x`.
fn dist_to_ray(p: &Point, c: &Point) -> f64 {
    let v = ((p.x - c.x) as f64, (p.y - c.y) as f64);
    let t = (0.0_f64).max(v.0);
    let w = (v.0 - t, v.1 - 0.0);
    (w.0 * w.0 + w.1 * w.1).sqrt()
}

// s=0: nearest generator point to the ray from centroid along world +X.
/// FillConformal.cpp:220-250 — `static double s0_world_plus_x(const Centerline &g, const Point &c)`
fn s0_world_plus_x(g: &Centerline, c: &Point) -> f64 {
    let pts = &g.polyline.points;
    if pts.len() < 2 {
        return 0.0;
    }
    let closed = g.closed && pts.len() >= 3 && pts[0] == pts[pts.len() - 1];
    let nseg = pts.len() - 1;
    let mut best_d = f64::MAX;
    let mut best_s = 0.0_f64;
    let mut acc = 0.0_f64;
    for i in 0..nseg {
        let a = pts[i];
        let d = dist_to_ray(&a, c);
        if d < best_d {
            best_d = d;
            best_s = acc;
        }
        acc += diff_norm(pts[i + 1], a);
    }
    if !closed {
        let d = dist_to_ray(&pts[nseg], c);
        if d < best_d {
            best_s = acc;
        }
    }
    best_s
}

/// FillConformal.cpp:252-258 — `static void orient_open_plus_x(Centerline &g)`
fn orient_open_plus_x(g: &mut Centerline) {
    if g.closed || g.polyline.points.len() < 2 {
        return;
    }
    if g.polyline.points[g.polyline.points.len() - 1].x > g.polyline.points[0].x {
        g.polyline.reverse();
    }
}

/// FillConformal.cpp:260-264 — `struct Chord`
#[derive(Debug, Clone, Default)]
struct Chord {
    s: f64,
    pl: Polyline,
}

/// FillConformal.cpp:266-284 — `static bool polylines_interior_cross(const Polyline &a, const Polyline &b, double end_eps)`
fn polylines_interior_cross(a: &Polyline, b: &Polyline, end_eps: f64) -> bool {
    use crate::geometry::geometry::{segment_segment_intersection, Vec2d};
    let norm = |v: &Vec2d| (v.x * v.x + v.y * v.y).sqrt();
    for ia in 1..a.points.len() {
        let p1 = Vec2d::new(a.points[ia - 1].x as f64, a.points[ia - 1].y as f64);
        let v1 = Vec2d::new(a.points[ia].x as f64, a.points[ia].y as f64) - p1;
        for ib in 1..b.points.len() {
            let p2 = Vec2d::new(b.points[ib - 1].x as f64, b.points[ib - 1].y as f64);
            let v2 = Vec2d::new(b.points[ib].x as f64, b.points[ib].y as f64) - p2;
            let mut hit = Vec2d::new(0.0, 0.0);
            if !segment_segment_intersection(&p1, &v1, &p2, &v2, &mut hit) {
                continue;
            }
            let da = norm(&(hit - p1)).min(norm(&(hit - (p1 + v1))));
            let db = norm(&(hit - p2)).min(norm(&(hit - (p2 + v2))));
            if da > end_eps && db > end_eps {
                return true;
            }
        }
    }
    false
}

/// FillConformal.cpp:286-306 — `static void drop_crossing_chords(std::vector<Chord> &chords)`
fn drop_crossing_chords(chords: &mut Vec<Chord>) {
    // std::sort (libc++, not stable) by decreasing length. The elements are moved
    // through an index permutation: the sort makes the same comparisons and the
    // same moves on the indices as it would on the chords.
    let lengths: Vec<f64> = chords.iter().map(|c| c.pl.length()).collect();
    let mut order: Vec<usize> = (0..chords.len()).collect();
    crate::clipper::libcxx_sort(&mut order, |a: &usize, b: &usize| lengths[*a] > lengths[*b]);
    let mut taken: Vec<Option<Chord>> = std::mem::take(chords).into_iter().map(Some).collect();
    let sorted: Vec<Chord> = order.iter().map(|&i| taken[i].take().expect("a permutation")).collect();

    let end_eps = scale_(0.4);
    let mut kept: Vec<Chord> = Vec::with_capacity(sorted.len());
    for c in sorted {
        let mut hit = false;
        for k in &kept {
            if polylines_interior_cross(&c.pl, &k.pl, end_eps) {
                hit = true;
                break;
            }
        }
        if !hit {
            kept.push(c);
        }
    }
    *chords = kept;
}

/// FillConformal.cpp:308-353 — `static bool emit_chords(Centerline g, const ExPolygon &region, const Fill &fill, const FillParams &params, bool world_anchor, Polylines &scans)`
fn emit_chords(g: Centerline, region: &ExPolygon, fill: &Fill, params: &FillParams, world_anchor: bool, scans: &mut Vec<Polyline>) -> bool {
    let l = generator_length(&g);
    if l < scale_(0.5) {
        return false;
    }

    let delta = line_spacing_mm(fill, params);
    let half_closed = g.closed && region.holes.is_empty();
    let span = if half_closed { 0.5 * l } else { l };
    let n = chart_line_count(span, delta);
    let centroid = fill_radial_zig_zag::polygon_centroid(&region.contour);
    let mut s0 = if world_anchor && g.closed { s0_world_plus_x(&g, &centroid) } else { 0.0 };
    let odd = (layer_index(fill) % 2) == 1;
    let half_step = odd && params.conformal_stagger == ConformalStagger::HalfStep;
    if g.closed && half_step {
        s0 += span / (2.0 * n as f64);
    }

    let mut chords: Vec<Chord> = Vec::with_capacity(n as usize);
    for k in 0..n {
        let mut s: f64;
        if g.closed {
            // s0 + double(k) * span / double(N): the sum takes a quotient, nothing to fuse.
            s = s0 + k as f64 * span / n as f64;
        } else {
            s = (k as f64 + 0.5) * l / n as f64;
            if half_step {
                s += l / (2.0 * n as f64);
            }
        }
        let mut p = Point::new(0, 0);
        let mut tangent = (0.0_f64, 0.0_f64);
        if !sample_generator(&g, s, &mut p, &mut tangent) {
            continue;
        }
        let nrm = (-tangent.1, tangent.0);
        let mut hits: Vec<Polyline> = Vec::new();
        clip_local_chord(&p, nrm, region, &mut hits);
        for hit in hits {
            chords.push(Chord { s, pl: hit });
        }
    }

    drop_crossing_chords(&mut chords);
    // std::sort (libc++, not stable) by s: chords of one sample share s, so their
    // order after the sort is the sort's. Same index permutation as above.
    let keys: Vec<f64> = chords.iter().map(|c| c.s).collect();
    let mut order: Vec<usize> = (0..chords.len()).collect();
    crate::clipper::libcxx_sort(&mut order, |a: &usize, b: &usize| keys[*a] < keys[*b]);
    let mut taken: Vec<Option<Chord>> = chords.into_iter().map(Some).collect();
    for &i in &order {
        scans.push(taken[i].take().expect("a permutation").pl);
    }
    scans.len() >= 2
}

/// FillConformal.cpp:355-363 — `static bool generator_near_region(const LayerGenerator &g, const ExPolygon &region)`
fn generator_near_region(g: &LayerGenerator, region: &ExPolygon) -> bool {
    let mut bb = get_extents(region);
    // bb.offset(scaled<coord_t>(3.))
    let d = scaled_coord(3.0);
    bb.min = Point::new(bb.min.x - d, bb.min.y - d);
    bb.max = Point::new(bb.max.x + d, bb.max.y + d);
    for p in &g.polyline.points {
        if p.x >= bb.min.x && p.x <= bb.max.x && p.y >= bb.min.y && p.y <= bb.max.y {
            return true;
        }
    }
    false
}

/// FillConformal.cpp:365-373 — `static bool fill_chart_rays(const ExPolygon &region, const Fill &fill, const FillParams &params, Polylines &scans)`
fn fill_chart_rays(region: &ExPolygon, fill: &Fill, params: &FillParams, scans: &mut Vec<Polyline>) -> bool {
    let mut lg = LayerGenerator::default();
    if !extract_generator(region, fill.spacing, &mut lg) {
        return false;
    }
    let mut g = to_centerline(&lg);
    orient_open_plus_x(&mut g);
    emit_chords(g, region, fill, params, true, scans)
}

/// FillConformal.cpp:375-389 — `static bool connection_inside(const Point &a, const Point &b, const ExPolygon &region, const ExPolygons &fat, double max_len)`
fn connection_inside(a: &Point, b: &Point, region: &ExPolygon, fat: &[ExPolygon], max_len: f64) -> bool {
    let len = diff_norm(*b, *a);
    if len < scale_(0.05) {
        return true;
    }
    if len > max_len {
        return false;
    }
    if expolygon_contains_line(region, *a, *b) {
        return true;
    }
    for ex in fat {
        if expolygon_contains_line(ex, *a, *b) {
            return true;
        }
    }
    false
}

/// FillConformal.cpp:403-418 — the lambda `join_clean` of `connect_chart_polylines`.
#[allow(clippy::too_many_arguments)]
fn join_clean(
    a: &Point,
    b: &Point,
    cur: &Polyline,
    nxt: &Polyline,
    region: &ExPolygon,
    fat: &[ExPolygon],
    max_len: f64,
    end_eps: f64,
    out: &[Polyline],
) -> bool {
    if !connection_inside(a, b, region, fat, max_len) {
        return false;
    }
    if diff_norm(*b, *a) < scale_(0.08) {
        return true;
    }
    let hop = Polyline::from_points(vec![*a, *b]);
    !polylines_interior_cross(&hop, cur, end_eps) && !polylines_interior_cross(&hop, nxt, end_eps) && {
        let mut clean = true;
        for o in out {
            if polylines_interior_cross(&hop, o, end_eps) {
                clean = false;
                break;
            }
        }
        clean
    }
}

/// FillConformal.cpp:419-439 — the lambda `try_append` of `connect_chart_polylines`.
#[allow(clippy::too_many_arguments)]
fn try_append(
    current: &mut Polyline,
    nxt: &mut Polyline,
    region: &ExPolygon,
    fat: &[ExPolygon],
    max_len: f64,
    end_eps: f64,
    out: &[Polyline],
) -> bool {
    if current.points.len() < 2 || nxt.points.len() < 2 {
        return false;
    }
    let tip = current.points[current.points.len() - 1];
    let nxt_front = nxt.points[0];
    let nxt_back = nxt.points[nxt.points.len() - 1];
    let ok_f = join_clean(&tip, &nxt_front, current, nxt, region, fat, max_len, end_eps, out);
    let ok_b = join_clean(&tip, &nxt_back, current, nxt, region, fat, max_len, end_eps, out);
    if !ok_f && !ok_b {
        return false;
    }
    #[allow(unused_assignments)]
    let mut rev = false;
    if ok_f && ok_b {
        let df = diff_norm(nxt_front, tip);
        let db = diff_norm(nxt_back, tip);
        rev = db < df;
    } else {
        rev = ok_b;
    }
    if rev {
        nxt.reverse();
    }
    current.append(nxt);
    true
}

// Join consecutive s-ordered chords if the straight link stays inside the island.
// Unlike Fill::connect_infill this does not assume parallel scanlines, so holes
// get a zigzag around the opening instead of hundreds of loose segments.
/// FillConformal.cpp:394-458 — `static void connect_chart_polylines(Polylines &&scans, const ExPolygon &region, double max_len, Polylines &out)`
fn connect_chart_polylines(mut scans: Vec<Polyline>, region: &ExPolygon, max_len: f64, out: &mut Vec<Polyline>) {
    if scans.len() <= 1 {
        out.append(&mut scans);
        return;
    }

    let fat = offset_ex(region, scale_(0.4) as f32);
    let end_eps = scale_(0.45);

    let mut current = std::mem::take(&mut scans[0]);
    for i in 1..scans.len() {
        if !try_append(&mut current, &mut scans[i], region, &fat, max_len, end_eps, out) {
            if current.points.len() >= 2 {
                out.push(std::mem::take(&mut current));
            }
            current = std::mem::take(&mut scans[i]);
        }
    }
    if current.points.len() >= 2 {
        out.push(current);
    }

    // Close a single zigzag into a loop when the wrap-around hop is short.
    if out.len() == 1 {
        let pl = &mut out[0];
        if pl.points.len() >= 2 {
            let (back, front) = (pl.points[pl.points.len() - 1], pl.points[0]);
            if connection_inside(&back, &front, region, &fat, max_len) {
                pl.append_point(front);
            }
        }
    }
}

/// FillConformal.cpp:460-473 — `static bool is_blob_island(const ExPolygon &region)`
fn is_blob_island(region: &ExPolygon) -> bool {
    if !region.holes.is_empty() {
        return false;
    }
    let bb = get_extents(region);
    let w = std::cmp::max(1 as Coord, bb.size().x) as f64;
    let h = std::cmp::max(1 as Coord, bb.size().y) as f64;
    let aspect = w.max(h) / w.min(h);
    if aspect >= 2.5 {
        return false;
    }
    // L-shapes and other concave islands share a square AABB with a disk;
    // require the island to fill most of its box before using polar diameters.
    region.area().abs() > 0.65 * w * h
}

/// FillConformal.cpp:475-483 — `static double hub_line_width_mm(const Fill &fill, const FillParams &params)`
fn hub_line_width_mm(fill: &Fill, params: &FillParams) -> f64 {
    // `Flow::width()` returns a float in the C++.
    let w = (params.flow.width() as f32) as f64;
    if w > 1e-6 {
        return w;
    }
    if fill.spacing > 1e-6 {
        return fill.spacing;
    }
    0.4
}

/// FillConformal.cpp:485-525 — `static Polylines fill_hub_rectilinear(const Fill &src, const Surface *surface, const FillParams &params, const ExPolygon &hub)`
fn fill_hub_rectilinear(src: &Fill, surface: &Surface, params: &FillParams, hub: &ExPolygon) -> Vec<Polyline> {
    // FillConformal.cpp:488-497 — `FillRectilinear rect;` with the data of `src`.
    let mut rect = Fill::new();
    rect.layer_id = src.layer_id;
    rect.lock_region_id = src.lock_region_id;
    rect.z = src.z;
    rect.spacing = src.spacing;
    rect.overlap = src.overlap;
    rect.angle = src.angle;
    rect.link_max_length = src.link_max_length;
    rect.loop_clipping = src.loop_clipping;
    rect.bounding_box = src.bounding_box.clone();
    // FillConformal.cpp:498-505
    let mut s = surface.clone();
    s.expolygon = hub.clone();
    let mut p = params.clone();
    p.conformal = false;
    p.pattern = InfillPattern::Rectilinear;
    // Do not walk the hub circle: that traces the A−B / A∩B boundary.
    p.anchor_length = 0.0;
    p.anchor_length_max = 0.0;
    // FillConformal.cpp:506 — `Polylines raw = rect.fill_surface(&s, p);`
    //
    // `FillRectilinear::fill_surface` (FillRectilinear.cpp:3075-3095) with
    // `conformal` off and `multiline == 1` (or full infill) is
    // `fill_surface_by_lines(surface, params, 0.f, 0.f, polylines_out)`; the
    // direction is `Fill::_infill_direction` (FillBase.cpp:199-241).
    // FIDELITY-NOTE: with `multiline > 1` and density below 1 the C++ takes
    // `fill_surface_by_multilines` (FillRectilinear.cpp:3087-3093); the crate's
    // rectilinear entry has no such arm, so the single-line raster is used.
    let (angle, _shift) = super::infill_direction(
        rect.angle,
        rect.layer_id,
        s.thickness_layers as u32,
        s.bridge_angle.unwrap_or(-1.0),
        &rect.bounding_box,
    );
    let density = (p.density as f32) as f64;
    let rparams = FillRectilinearParams {
        density,
        monotonic: false,
        dont_connect: p.dont_connect(),
        link_max_length: rect.link_max_length,
        full_infill: (p.density as f32) > 0.9999_f32,
        dont_adjust: p.dont_adjust,
        consistent_pattern: false,
    };
    let raw = fill_surface_by_lines(&s.expolygon, rect.spacing, angle as f64, rect.overlap, &rparams);
    // dont_connect still chain_polylines, which strings endpoints around circle B.
    // FillConformal.cpp:508-524
    let mut out: Vec<Polyline> = Vec::with_capacity(raw.len());
    for pl in &raw {
        if pl.points.len() < 2 {
            continue;
        }
        if pl.points.len() == 2 {
            out.push(pl.clone());
            continue;
        }
        for i in 1..pl.points.len() {
            let seg = Polyline::from_points(vec![pl.points[i - 1], pl.points[i]]);
            if seg.length() > scale_(0.05) {
                out.push(seg);
            }
        }
    }
    out
}

// Fill via equal-angle radial ZigZag around a locked pole.
// Older polar / medial-axis / chart paths are kept in this file but not called.
// Returns false if the island cannot be filled conformally; caller falls back to world-axis scan.
/// FillConformal.cpp:527-568 — `bool fill_surface(const Fill &fill, const Surface *surface, const FillParams &params, Polylines &out)`
pub fn fill_surface(fill: &Fill, surface: Option<&Surface>, params: &FillParams, out: &mut Vec<Polyline>) -> bool {
    // FillConformal.cpp:529-530 — `params.density < 0.0001f`
    let surface = match surface {
        Some(s) if !((params.density as f32) < 0.0001_f32) => s,
        _ => return false,
    };

    // FillConformal.cpp:532-535 — fill.overlap - 0.5 * fill.spacing: fused.
    let inset = scale_((-0.5_f64).mul_add(fill.spacing, fill.overlap)) as f32;
    let mut regions = offset_ex(&surface.expolygon, inset);
    if regions.is_empty() {
        regions.push(surface.expolygon.clone());
    }

    // FillConformal.cpp:537-561
    let mut any_ok = false;
    for region in &regions {
        let mut radial: Vec<Polyline> = Vec::new();
        if fill_radial_zig_zag::fill_region(region, fill, params, &mut radial) && !radial.is_empty() {
            out.append(&mut radial);
            any_ok = true;
        }
        let mut pole = Point::new(0, 0);
        let mut disk = ExPolygon::empty();
        if fill_radial_zig_zag::make_hub_disk(region, fill, params, &mut pole, &mut disk) {
            // Inset from circle B so rectilinear hops stay inside A∩B.
            let gap = 1.0 * hub_line_width_mm(fill, params);
            let slim = offset_ex(&disk, -(scale_(gap) as f32));
            if !slim.is_empty() {
                // intersection_ex(const ExPolygon&, const ExPolygons&), ClipperUtils.cpp:800-801.
                let hub = intersection_clib(std::slice::from_ref(region), &slim);
                for h in &hub {
                    let mut rl = fill_hub_rectilinear(fill, surface, params, h);
                    if !rl.is_empty() {
                        out.append(&mut rl);
                        any_ok = true;
                    }
                }
            }
        }
    }

    // FillConformal.cpp:563-567
    if !any_ok || out.is_empty() {
        out.clear();
        return false;
    }
    true
}

// FillConformal.cpp:570-571 — } // namespace FillConformal } // namespace Slic3r

#[cfg(test)]
mod tests {
    use super::*;
    use crate::surface::SurfaceType;

    fn mm(v: f64) -> Coord {
        (v * 100000.0).round() as Coord
    }

    fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Polygon {
        Polygon::from_points(vec![
            Point::new(mm(x0), mm(y0)),
            Point::new(mm(x1), mm(y0)),
            Point::new(mm(x1), mm(y1)),
            Point::new(mm(x0), mm(y1)),
        ])
    }

    fn square() -> ExPolygon {
        ExPolygon::new(rect(-10.0, -10.0, 10.0, 10.0))
    }

    fn square_with_hole() -> ExPolygon {
        let mut hole = rect(-4.0, -4.0, 4.0, 4.0);
        hole.points.reverse();
        ExPolygon::with_holes(rect(-10.0, -10.0, 10.0, 10.0), vec![hole])
    }

    fn l_shape() -> ExPolygon {
        ExPolygon::new(Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(mm(20.0), 0),
            Point::new(mm(20.0), mm(10.0)),
            Point::new(mm(10.0), mm(10.0)),
            Point::new(mm(10.0), mm(20.0)),
            Point::new(0, mm(20.0)),
        ]))
    }

    fn pl(points: &[(f64, f64)]) -> Polyline {
        Polyline::from_points(points.iter().map(|&(x, y)| Point::new(mm(x), mm(y))).collect())
    }

    fn test_fill(layer_id: usize) -> Fill {
        let mut f = Fill::new();
        f.layer_id = layer_id;
        f.z = 0.2 * (layer_id + 1) as f64;
        f.spacing = 0.45;
        f.angle = 0.0;
        f
    }

    fn test_params(density: f64) -> FillParams {
        let mut p = FillParams::new();
        p.density = density;
        p.conformal = true;
        p
    }

    fn assert_inside(paths: &[Polyline], region: &ExPolygon, tol_mm: f64) {
        let fat = offset_ex(region, scale_(tol_mm) as f32);
        let inside = |p: &Point| fat.iter().any(|ex| ex.contains(p, true));
        for pl in paths {
            assert!(pl.points.len() >= 2);
            for i in 0..pl.points.len() {
                assert!(inside(&pl.points[i]), "vertex {:?} outside", pl.points[i]);
                if i > 0 {
                    let m = Point::new((pl.points[i - 1].x + pl.points[i].x) / 2, (pl.points[i - 1].y + pl.points[i].y) / 2);
                    assert!(inside(&m), "midpoint {m:?} outside");
                }
            }
        }
    }

    fn globals() -> std::sync::MutexGuard<'static, ()> {
        let g = fill_radial_zig_zag::TEST_GLOBALS.lock().unwrap_or_else(|e| e.into_inner());
        fill_radial_zig_zag::reset_n_lock();
        g
    }

    #[test]
    fn counts_and_phases() {
        let fill = test_fill(1);
        let mut params = test_params(0.2);
        assert_eq!(layer_index(&fill), 1);
        assert_eq!(layer_index(&Fill::new()), 0); // size_t(-1)
        assert_eq!(line_spacing_mm(&fill, &params), 0.45 / (0.2_f32 as f64));
        params.density = 0.01;
        assert_eq!(line_spacing_mm(&fill, &params), 0.45 / 0.05);
        params.density = 0.2;
        // 20 mm box over 2.25 mm lines.
        assert_eq!(stable_polar_count(&fill, &params, &square()), 9);
        assert_eq!(chart_line_count(scale_(45.0), 2.25), 20);
        assert_eq!(chart_line_count(scale_(1.0), 2.25), 2);
        assert_eq!(theta0_for_layer(&fill, &params, 9), 0.0);
        params.conformal_stagger = ConformalStagger::HalfStep;
        assert_eq!(theta0_for_layer(&fill, &params, 9), PI / 18.0);
        assert_eq!(theta0_for_layer(&test_fill(2), &params, 9), 0.0);
        assert_eq!(unit_from_angle(0.0), (1.0, 0.0));
        // The object's box when the fill has one.
        let mut with_bb = test_fill(0);
        with_bb.bounding_box = BoundingBox::from_points(&[Point::new(0, 0), Point::new(mm(45.0), mm(10.0))]);
        assert_eq!(stable_polar_count(&with_bb, &params, &square()), 20);
        assert_eq!(hub_line_width_mm(&fill, &params), (params.flow.width() as f32) as f64);
    }

    #[test]
    fn offset_loops_step_inwards() {
        let fill = test_fill(0);
        let params = test_params(0.2);
        let mut out = Vec::new();
        assert!(fill_offset_loops(&square(), &fill, &params, &mut out));
        // Offsets 2.25, 4.5, 6.75 mm: below 10 - 1.125.
        assert_eq!(out.len(), 3);
        for (i, pl) in out.iter().enumerate() {
            assert_eq!(pl.points[0], pl.points[pl.points.len() - 1]);
            let side = 20.0 - 2.0 * 2.25 * (i + 1) as f64;
            assert!((pl.length() - 4.0 * scale_(side)).abs() < scale_(0.01), "loop {i}");
        }
        assert_inside(&out, &square(), 0.01);
        let mut closed = Vec::new();
        append_closed_polygon(&rect(0.0, 0.0, 1.0, 1.0), &mut closed);
        assert_eq!(closed.len(), 1);
        assert_eq!(closed[0].points.len(), 5);
        append_closed_polygon(&rect(0.0, 0.0, 0.01, 0.01), &mut closed); // shorter than 0.2 mm
        assert_eq!(closed.len(), 1);
    }

    #[test]
    fn polar_rays_are_diameters_through_the_centroid() {
        let fill = test_fill(0);
        let params = test_params(0.2);
        let mut scans = Vec::new();
        assert!(fill_polar_rays(&square(), &fill, &params, &mut scans));
        assert_eq!(scans.len(), 9);
        for (k, s) in scans.iter().enumerate() {
            assert_eq!(s.points.len(), 2);
            let (a, b) = (s.points[0], s.points[1]);
            // Through the centre, at k * pi / 9.
            let th = ((b.y - a.y) as f64).atan2((b.x - a.x) as f64);
            let want = k as f64 * PI / 9.0;
            let d = (th - want).rem_euclid(PI);
            assert!(d < 1e-3 || PI - d < 1e-3, "ray {k}: {th}");
            assert!(polyline_dist_to_point(s, &Point::new(0, 0)) < 3.0);
        }
        assert_inside(&scans, &square(), 0.01);
        // A ray across a hole: the local chord is the piece nearest the sample.
        let mut local = Vec::new();
        clip_local_chord(&Point::new(mm(7.0), 0), (1.0, 0.0), &square_with_hole(), &mut local);
        assert_eq!(local.len(), 1);
        let xs = (local[0].points[0].x.min(local[0].points[1].x), local[0].points[0].x.max(local[0].points[1].x));
        assert_eq!(xs, (mm(4.0), mm(10.0)));
    }

    #[test]
    fn generator_sampling() {
        let g = Centerline { polyline: pl(&[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)]), closed: false };
        assert_eq!(generator_length(&g), mm(20.0) as f64);
        let mut p = Point::new(0, 0);
        let mut t = (0.0, 0.0);
        assert!(sample_generator(&g, mm(4.0) as f64, &mut p, &mut t));
        assert_eq!((p, t), (Point::new(mm(4.0), 0), (1.0, 0.0)));
        assert!(sample_generator(&g, mm(15.0) as f64, &mut p, &mut t));
        assert_eq!((p, t), (Point::new(mm(10.0), mm(5.0)), (0.0, 1.0)));
        assert!(sample_generator(&g, 1e12, &mut p, &mut t));
        assert_eq!(p, Point::new(mm(10.0), mm(10.0)));

        // A closed square loop about the origin: s = 0 moves to where +X leaves it.
        let ring = Centerline { polyline: pl(&[(-5.0, -5.0), (5.0, -5.0), (5.0, 5.0), (-5.0, 5.0), (-5.0, -5.0)]), closed: true };
        // Vertices only: (5,-5) and (5,5) are both 5 mm from the ray; the first wins.
        assert_eq!(s0_world_plus_x(&ring, &Point::new(0, 0)), mm(10.0) as f64);
        let mut open = Centerline { polyline: pl(&[(0.0, 0.0), (10.0, 0.0)]), closed: false };
        orient_open_plus_x(&mut open);
        assert_eq!(open.polyline.points[0], Point::new(mm(10.0), 0));
        let lg = LayerGenerator { z: 0.0, closed: false, polyline: pl(&[(30.0, 0.0), (40.0, 0.0)]), length: 0.0 };
        assert!(!generator_near_region(&lg, &square()));
        let lg = LayerGenerator { z: 0.0, closed: false, polyline: pl(&[(12.0, 0.0), (40.0, 0.0)]), length: 0.0 };
        assert!(generator_near_region(&lg, &square()));
        let c = to_centerline(&lg);
        assert_eq!(c.polyline.points, lg.polyline.points);
    }

    #[test]
    fn crossing_chords_are_dropped_longest_first() {
        // `Geometry::segment_segment_intersection` (Geometry.hpp:86-115) as written
        // accepts s in [0, 1] on the second segment and t in [-1, 0] on the first
        // (`t_numer` is negated when the denominator is positive, and not when it
        // was negative), and returns p1 + |t| * v1. The port keeps that: the
        // "crossing" below is the one the C++ arithmetic reports.
        let a = pl(&[(0.0, 0.0), (10.0, 0.0)]);
        let b = pl(&[(-5.0, -5.5), (-5.0, 5.5)]);
        let c = pl(&[(0.0, 3.0), (4.0, 3.0)]);
        assert!(polylines_interior_cross(&a, &b, scale_(0.4)));
        assert!(!polylines_interior_cross(&a, &c, scale_(0.4))); // parallel
        assert!(!polylines_interior_cross(&c, &b, scale_(0.4)));
        // A geometric crossing in front of p1 is not reported.
        assert!(!polylines_interior_cross(&a, &pl(&[(5.0, -5.0), (5.0, 5.0)]), scale_(0.4)));
        let mut chords = vec![
            Chord { s: 1.0, pl: c.clone() },
            Chord { s: 2.0, pl: b.clone() },
            Chord { s: 3.0, pl: a.clone() },
        ];
        drop_crossing_chords(&mut chords);
        // Longest first: the 11 mm chord is kept, the 10 mm one that "crosses" it is
        // dropped, the 4 mm one is kept.
        let ss: Vec<f64> = chords.iter().map(|c| c.s).collect();
        assert_eq!(ss, vec![2.0, 1.0]);
    }

    #[test]
    fn chart_rays_cross_a_strip_at_the_line_spacing() {
        let strip = ExPolygon::new(rect(0.0, 0.0, 40.0, 4.0));
        let fill = test_fill(0);
        let params = test_params(0.2);
        let mut scans = Vec::new();
        assert!(fill_chart_rays(&strip, &fill, &params, &mut scans));
        assert!(scans.len() >= 8, "{} chords", scans.len());
        assert_inside(&scans, &strip, 0.01);
        // The chords inside the straight middle of the strip are vertical.
        let mut xs: Vec<Coord> = Vec::new();
        for s in &scans {
            let (a, b) = (s.points[0], s.points[s.points.len() - 1]);
            if a.x > mm(6.0) && a.x < mm(34.0) {
                assert!((a.x - b.x).abs() < mm(0.05), "chord {a:?} {b:?}");
                assert!(((a.y - b.y).abs() - mm(4.0)).abs() < mm(0.01));
                xs.push(a.x);
            }
        }
        assert!(xs.len() >= 4);
        // Even pitch, close to the 2.25 mm line spacing.
        xs.sort();
        let pitch = (xs[1] - xs[0]).abs();
        assert!((pitch - mm(2.25)).abs() < mm(0.4), "pitch {pitch}");
        for w in xs.windows(2) {
            assert!(((w[1] - w[0]) - pitch).abs() < mm(0.02));
        }

        let mut out = Vec::new();
        connect_chart_polylines(scans.clone(), &strip, scale_(8.0), &mut out);
        assert!(!out.is_empty() && out.len() < scans.len());
        assert_inside(&out, &strip, 0.45);
        let mut one = Vec::new();
        connect_chart_polylines(vec![scans[0].clone()], &strip, scale_(8.0), &mut one);
        assert_eq!(one.len(), 1);
        assert!(connection_inside(&Point::new(mm(1.0), mm(1.0)), &Point::new(mm(3.0), mm(3.0)), &strip, &[], scale_(8.0)));
        assert!(!connection_inside(&Point::new(mm(1.0), mm(1.0)), &Point::new(mm(30.0), mm(3.0)), &strip, &[], scale_(8.0)));
        assert!(!connection_inside(&Point::new(mm(1.0), mm(1.0)), &Point::new(mm(1.0), mm(6.0)), &strip, &[], scale_(8.0)));
    }

    #[test]
    fn blob_islands() {
        assert!(is_blob_island(&square()));
        assert!(!is_blob_island(&square_with_hole()));
        assert!(!is_blob_island(&ExPolygon::new(rect(0.0, 0.0, 40.0, 4.0))));
        assert!(is_blob_island(&l_shape())); // 300 of 400 mm2: above 0.65
        let thin_l = ExPolygon::new(Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(mm(20.0), 0),
            Point::new(mm(20.0), mm(4.0)),
            Point::new(mm(4.0), mm(4.0)),
            Point::new(mm(4.0), mm(20.0)),
            Point::new(0, mm(20.0)),
        ]));
        assert!(!is_blob_island(&thin_l));
    }

    #[test]
    fn fill_surface_fills_the_island_and_its_hub() {
        let _g = globals();
        for (name, expoly) in [
            ("square", square()),
            ("square with hole", square_with_hole()),
            ("L", l_shape()),
            ("strip", ExPolygon::new(rect(0.0, 0.0, 40.0, 3.0))),
        ] {
            for density in [0.15, 0.4] {
                for layer in [0usize, 1] {
                    let fill = test_fill(layer);
                    let params = test_params(density);
                    let surface = Surface::new(SurfaceType::Internal, expoly.clone());
                    let mut out = Vec::new();
                    let ok = fill_surface(&fill, Some(&surface), &params, &mut out);
                    if !ok {
                        assert!(out.is_empty(), "{name}: cleared on failure");
                        continue;
                    }
                    assert!(!out.is_empty(), "{name}");
                    // Paths are laid on the island inset by half a spacing; rim hops may sit on its contour.
                    assert_inside(&out, &expoly, 0.25);
                    let mut again = Vec::new();
                    assert!(fill_surface(&fill, Some(&surface), &params, &mut again));
                    assert_eq!(again.len(), out.len(), "{name}");
                    for (a, b) in again.iter().zip(out.iter()) {
                        assert_eq!(a.points, b.points, "{name}");
                    }
                }
            }
        }
    }

    #[test]
    fn fill_surface_of_a_square_has_radial_scans_and_hub_lines() {
        let _g = globals();
        let fill = test_fill(0);
        let params = test_params(0.2);
        let surface = Surface::new(SurfaceType::Internal, square());
        let mut out = Vec::new();
        assert!(fill_surface(&fill, Some(&surface), &params, &mut out));
        // The region is the square inset by 0.225 mm; n = even(lround(39.1 / 2.25)) = 18,
        // the hub radius 18 * 1.2 * 0.4 / (2 pi) = 1.375 mm.
        let r_hub = scale_(18.0 * 1.2 * 0.4 / (2.0 * PI));
        let o = Point::new(0, 0);
        let radius = |p: &Point| diff_norm(*p, o);
        // Hub lines: two-point segments inside circle B shrunk by one line width.
        let hub: Vec<&Polyline> = out.iter().filter(|pl| pl.points.iter().all(|p| radius(p) < r_hub)).collect();
        assert!(!hub.is_empty(), "no hub lines");
        for pl in &hub {
            assert_eq!(pl.points.len(), 2);
            for p in &pl.points {
                assert!(radius(p) <= r_hub - scale_(0.4) + scale_(0.02), "{p:?}");
            }
        }
        // The radial part stays outside circle B grown by one line width.
        let radial: Vec<&Polyline> = out.iter().filter(|pl| !pl.points.iter().all(|p| radius(p) < r_hub)).collect();
        assert!(!radial.is_empty(), "no radial paths");
        for pl in &radial {
            for p in &pl.points {
                assert!(radius(p) >= r_hub + scale_(0.4) - scale_(0.05), "{p:?}");
            }
        }
        // 18 scans reach the inset contour.
        let on_rim = radial
            .iter()
            .flat_map(|pl| pl.points.iter())
            .filter(|p| (p.x.abs().max(p.y.abs()) - mm(9.775)).abs() < mm(0.01))
            .count();
        assert!(on_rim >= 18, "{on_rim} rim points");
    }

    #[test]
    fn fill_surface_refuses_no_surface_and_no_density() {
        let _g = globals();
        let fill = test_fill(0);
        let surface = Surface::new(SurfaceType::Internal, square());
        let mut out = vec![Polyline::new()];
        assert!(!fill_surface(&fill, None, &test_params(0.2), &mut out));
        assert!(!fill_surface(&fill, Some(&surface), &test_params(0.00005), &mut out));
        // Two holes: the radial fill declines, and so does the hub (no pole).
        let mut two = square_with_hole();
        let mut h2 = rect(6.0, 6.0, 8.0, 8.0);
        h2.points.reverse();
        two.holes[0] = {
            let mut h = rect(-8.0, -8.0, -6.0, -6.0);
            h.points.reverse();
            h
        };
        two.holes.push(h2);
        let surface = Surface::new(SurfaceType::Internal, two);
        let mut params = test_params(0.2);
        params.conformal_hub_radius = -1.0;
        let mut out = vec![Polyline::new()];
        assert!(!fill_surface(&fill, Some(&surface), &params, &mut out));
        assert!(out.is_empty());
    }
}
