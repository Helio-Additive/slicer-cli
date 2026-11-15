//! Equal-angle radial zigzag around a locked pole (the conformal infill).
//!
//! C++ Reference:
//! - Fill/FillRadialZigZag.hpp
//! - Fill/FillRadialZigZag.cpp
//!
//! Line-by-line port of `Slic3r::FillRadialZigZag`: rays cast from a pole that is
//! pinned per print-object region before the layers are filled
//! (`pin_scan_counts`), clipped to the island, paired into scans and joined by
//! hops along the rims (`fill_region`), plus the circle around the pole that is
//! left to rectilinear infill (`make_hub_disk`).
//!
//! Floating point: the C++ program is built with `-ffp-contract=on`, so a
//! scalar `a * b + c` of ONE source expression is a fused multiply-add; those
//! places are written with `mul_add` here, and nowhere else. Eigen vector
//! expressions (`dot`, `norm`, `squaredNorm`) are not fused.

// FillRadialZigZag.cpp:1-16
//   #include "FillRadialZigZag.hpp"
//   #include "../BoundingBox.hpp"
//   #include "../ClipperUtils.hpp"
//   #include "../ExPolygon.hpp"
//   #include "../Line.hpp"
//   #include "../Point.hpp"
//   #include "../Polygon.hpp"
use std::collections::BTreeMap;
use std::f64::consts::PI;
use std::sync::Mutex;

use super::fill_base::Fill;
use super::FillParams;
use crate::clipper_utils::{
    clipper_do_polytree, diff_pl_clib, difference_clib, intersection_clib, intersection_pl_clib,
    offset_expolygons_clib_scaled, poly_tree_to_ex_polygons, OffsetJoinType,
};
use crate::geometry::{make_circle_num_segments, BoundingBox, ExPolygon, Point, Polygon, Polyline};
use crate::print_config::{ConformalPole, ConformalStagger, InfillPattern};
use crate::Coord;

/// `Vec3d` of the C++ (Eigen) — nalgebra here, as elsewhere in the crate.
pub type Vec3d = nalgebra::Vector3<f64>;

// FillRadialZigZag.cpp:18-19 — namespace Slic3r { namespace FillRadialZigZag {

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

/// Point.hpp:185 — `Point::new_scale(x, y)`: `Point(coord_t(scale_(x)), coord_t(scale_(y)))`.
#[inline]
fn point_new_scale(x: f64, y: f64) -> Point {
    Point::new(scale_(x) as Coord, scale_(y) as Coord)
}

/// `get_extents(const ExPolygon&)` (ExPolygon.cpp: the contour's box).
#[inline]
fn get_extents(ex: &ExPolygon) -> BoundingBox {
    BoundingBox::from_points(&ex.contour.points)
}

/// ClipperUtils.cpp:567-569 — `offset_ex(const ExPolygon&, const float delta)`.
fn offset_ex(expolygon: &ExPolygon, delta: f32) -> Vec<ExPolygon> {
    offset_expolygons_clib_scaled(std::slice::from_ref(expolygon), delta as f64, OffsetJoinType::Miter)
}

/// ClipperUtils.cpp:792-793 — `intersection_ex(const ExPolygon&, const ExPolygon&)`.
fn intersection_ex(subject: &ExPolygon, clip: &ExPolygon) -> Vec<ExPolygon> {
    intersection_clib(std::slice::from_ref(subject), std::slice::from_ref(clip))
}

/// ClipperUtils.hpp:480-486 — `diff_ex(const ExPolygon&, const ExPolygons&)`.
fn diff_ex(subject: &ExPolygon, clip: &[ExPolygon]) -> Vec<ExPolygon> {
    difference_clib(std::slice::from_ref(subject), clip)
}

/// ClipperUtils.cpp:815-816 — `union_ex(const ExPolygons &subject)`:
/// `PolyTreeToExPolygons(clipper_do_polytree(ctUnion, ExPolygonsProvider(subject), EmptyPathsProvider(), pftNonZero))`.
fn union_ex(subject: &[ExPolygon]) -> Vec<ExPolygon> {
    let mut paths = crate::clipper::Paths::new();
    for ex in subject {
        let mut push = |poly: &Polygon| {
            paths.push(
                poly.points
                    .iter()
                    .map(|p| crate::clipper::IntPoint::new(p.x as i32, p.y as i32))
                    .collect(),
            );
        };
        push(&ex.contour);
        for h in &ex.holes {
            push(h);
        }
    }
    poly_tree_to_ex_polygons(&clipper_do_polytree(
        crate::clipper::ClipType::ctUnion,
        &paths,
        &crate::clipper::Paths::new(),
        crate::clipper::PolyFillType::pftNonZero,
    ))
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
    // BoundingBox.hpp `overlap`.
    if bbox1.max.x < min2x || bbox1.min.x > max2x || bbox1.max.y < min2y || bbox1.min.y > max2y {
        return false;
    }
    let polyline = Polyline::from_points(vec![a, b]);
    let mut clip: Vec<Polygon> = Vec::with_capacity(1 + ex.holes.len());
    clip.push(ex.contour.clone());
    clip.extend(ex.holes.iter().cloned());
    diff_pl_clib(std::slice::from_ref(&polyline), &clip).is_empty()
}

/// Polygon.cpp:171-186 — `Point Polygon::centroid() const`, with the constructor the
/// pin's `Point(Vec2d(...))` resolves to: `explicit Point(const Vec2d&)`
/// (Point.hpp:181), which rounds with `lrint`.
// FIDELITY-NOTE: `crate::geometry::Polygon::centroid` ends with a truncating cast
// where the pin rounds, which moves the pole of a collar by one unit on about half
// of the layers. The body is repeated here with the rounding of the pin so that
// the conformal fill does not depend on it; the crate's method is left as it is
// (other callers, other outputs).
pub(crate) fn polygon_centroid(poly: &Polygon) -> Point {
    let mut area_sum = 0.0_f64;
    let mut c = (0.0_f64, 0.0_f64);
    if poly.points.len() >= 3 {
        let back = poly.points[poly.points.len() - 1];
        let mut p1 = (back.x as f64, back.y as f64);
        for p in &poly.points {
            let p2 = (p.x as f64, p.y as f64);
            // cross2(p1, p2), Point.hpp:86-90, as `crate::geometry::cross2f` has it.
            let a = p1.0 * p2.1 - p1.1 * p2.0;
            area_sum += a;
            c = (c.0 + (p1.0 + p2.0) * a, c.1 + (p1.1 + p2.1) * a);
            p1 = p2;
        }
    }
    let d = 3.0 * area_sum;
    Point::new((c.0 / d).round_ties_even() as Coord, (c.1 / d).round_ties_even() as Coord)
}

/// FillRadialZigZag.hpp:16-26 — `struct RadialRay`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RadialRay {
    pub theta: f64,
    pub inner: Point,
    pub outer: Point,
    pub mid: Point,
    pub inner_radius: f64,
    pub outer_radius: f64,
    pub valid: bool,
    /// one-interval graze too thin to be a scan; keep outer as the hit point
    pub lip: bool,
}

impl Default for RadialRay {
    fn default() -> Self {
        Self {
            theta: 0.0,
            inner: Point::new(0, 0),
            outer: Point::new(0, 0),
            mid: Point::new(0, 0),
            inner_radius: 0.0,
            outer_radius: 0.0,
            valid: false,
            lip: false,
        }
    }
}

/// FillRadialZigZag.hpp:28-40 — `struct RadialInfillResult`
#[derive(Debug, Clone)]
pub struct RadialInfillResult {
    pub success: bool,
    pub pole: Point,
    pub valid_ratio: f64,
    pub num_final_rays: i32,
    pub n_scan: i32,
    pub n_lock: i32,
    pub lock_rows: usize,
    pub rays: Vec<RadialRay>,
    pub extrusion_paths: Vec<Polyline>,
    pub fallback_reason: String,
}

impl Default for RadialInfillResult {
    fn default() -> Self {
        Self {
            success: false,
            pole: Point::new(0, 0),
            valid_ratio: 0.0,
            num_final_rays: 0,
            n_scan: 0,
            n_lock: 0,
            lock_rows: 0,
            rays: Vec::new(),
            extrusion_paths: Vec::new(),
            fallback_reason: String::new(),
        }
    }
}

/// FillRadialZigZag.hpp:42-67 — `struct LayerDiag`
#[derive(Debug, Clone, Default)]
pub struct LayerDiag {
    pub layer_id: usize,
    pub success: bool,
    pub pole_x: f64,
    pub pole_y: f64,
    pub lock_x: f64,
    pub lock_y: f64,
    pub layer_x: f64,
    pub layer_y: f64,
    pub has_lock: bool,
    pub axis_applied: bool,
    pub pole_inside: bool,
    pub drifted: bool,
    pub pole_mode: i32,
    pub inner_mean: f64,
    pub outer_mean: f64,
    pub island_mm: f64,
    pub valid_ratio: f64,
    pub num_final_rays: i32,
    pub n_scan: i32,
    pub n_lock: i32,
    pub n_holes: i32,
    pub lock_rows: usize,
    pub fallback_reason: String,
    pub rays: Vec<RadialRay>,
}

/// FillRadialZigZag.hpp:69-77 — `struct PoleVis`
#[derive(Debug, Clone, Default)]
pub struct PoleVis {
    pub region_id: usize,
    pub axis_ok: bool,
    pub bezier_ok: bool,
    pub layer_poles: Vec<Vec3d>,
    pub axis_polyline: Vec<Vec3d>,
    pub bezier_polyline: Vec<Vec3d>,
}

// FillRadialZigZag.cpp:21-27
const K_TEST_RAYS: i32 = 360;
const K_DENSE_RAYS: i32 = 720;
// Among rays that actually hit material, how many have exactly one interval.
// Misses (open side / empty sector) are ignored — they are not a failure.
const K_VALID_THRESHOLD: f64 = 0.90;
const K_MIN_HIT_RAYS: i32 = 8;
const K_MIN_SEG_MM: f64 = 0.2;

// FillRadialZigZag.cpp:29 — namespace {

/// FillRadialZigZag.cpp:31-33 — `g_dbg_mutex`, `g_dbg_on`, `g_dbg_log`.
struct DbgState {
    on: bool,
    log: Vec<LayerDiag>,
}
static G_DBG: Mutex<DbgState> = Mutex::new(DbgState { on: false, log: Vec::new() });

/// FillRadialZigZag.cpp:35-42 — `struct LockedIslandN`
#[derive(Debug, Clone, Copy)]
struct LockedIslandN {
    key: Point,
    size: f64,
    n: i32,
    theta0: f64,
    pole: Point,
}

/// FillRadialZigZag.cpp:43-50 — `struct AxisLine`
#[derive(Debug, Clone, Copy, Default)]
struct AxisLine {
    ok: bool,
    x0: f64,
    ax: f64,
    y0: f64,
    ay: f64,
}

/// FillRadialZigZag.cpp:52-59 — `struct QuadBezier`
#[derive(Debug, Clone, Copy, Default)]
struct QuadBezier {
    ok: bool,
    z0: f64,
    z1: f64,
    p0x: f64,
    p1x: f64,
    p2x: f64,
    p0y: f64,
    p1y: f64,
    p2y: f64,
}

/// FillRadialZigZag.cpp:63-69 — `struct LipMem`
#[derive(Debug, Clone, Copy)]
struct LipMem {
    pole: Point,
    left: Point,
    right: Point,
    ok: bool,
}

impl Default for LipMem {
    fn default() -> Self {
        Self { pole: Point::new(0, 0), left: Point::new(0, 0), right: Point::new(0, 0), ok: false }
    }
}

/// FillRadialZigZag.cpp:71-83 — `struct RegionLock`
#[derive(Debug, Clone, Default)]
struct RegionLock {
    n_by_layer: Vec<Vec<LockedIslandN>>,
    n_global: i32,
    axis: AxisLine,
    bezier: QuadBezier,
    zs: Vec<f64>,
    poles: Vec<Point>,
    lips_by_layer: BTreeMap<usize, Vec<LipMem>>,
    // Modifier island before later volumes clip it. Rays are cast here.
    // Pairing and hops follow the final fill region (already inside walls).
    cast_by_layer: Vec<Vec<ExPolygon>>,
}

/// FillRadialZigZag.cpp:61,85 — `std::mutex g_n_mutex;` and
/// `std::map<size_t, RegionLock> g_locks;`: the map lives inside its mutex.
static G_LOCKS: Mutex<BTreeMap<usize, RegionLock>> = Mutex::new(BTreeMap::new());

fn lock_g_locks() -> std::sync::MutexGuard<'static, BTreeMap<usize, RegionLock>> {
    G_LOCKS.lock().unwrap_or_else(|e| e.into_inner())
}

fn lock_g_dbg() -> std::sync::MutexGuard<'static, DbgState> {
    G_DBG.lock().unwrap_or_else(|e| e.into_inner())
}

/// FillRadialZigZag.cpp:87-96 — `struct LockedScan`
#[derive(Debug, Clone, Copy)]
struct LockedScan {
    n: i32,
    theta0: f64,
    pole: Point,
    ok: bool,
    has_pole: bool,
    table_rows: usize,
    row_islands: usize,
}

impl Default for LockedScan {
    fn default() -> Self {
        Self { n: 0, theta0: 0.0, pole: Point::new(0, 0), ok: false, has_pole: false, table_rows: 0, row_islands: 0 }
    }
}

// FillRadialZigZag.cpp:98-100
const K_N_MATCH_MIN_MM: f64 = 12.0;
const K_POLE_HOLD_MM: f64 = 1.0;
const K_POLE_SLEW_MM: f64 = 0.5;

/// FillRadialZigZag.cpp:102-106 — `static double island_size(const ExPolygon &ex)`
fn island_size(ex: &ExPolygon) -> f64 {
    let bb = get_extents(ex);
    bb.size().x.max(bb.size().y) as f64
}

/// FillRadialZigZag.cpp:108-111 — `static double n_match_limit(double size_a, double size_b)`
fn n_match_limit(size_a: f64, size_b: f64) -> f64 {
    scale_(K_N_MATCH_MIN_MM).max(0.45 * size_a.max(size_b))
}

/// FillRadialZigZag.cpp:113-121 — `static double ang_diff(double a, double b)`
fn ang_diff(a: f64, b: f64) -> f64 {
    let mut d = a - b;
    while d > PI {
        d -= 2.0 * PI;
    }
    while d < -PI {
        d += 2.0 * PI;
    }
    d.abs()
}

/// FillRadialZigZag.cpp:123-129 — `static int even_scan_count(int n)`
fn even_scan_count(mut n: i32) -> i32 {
    n = std::cmp::max(4, n);
    if n % 2 != 0 {
        n += 1;
    }
    n
}

/// FillRadialZigZag.cpp:131-137 — `static double estimate_mid_len(const ExPolygon &region)`
fn estimate_mid_len(region: &ExPolygon) -> f64 {
    let outer = region.contour.length();
    if region.holes.is_empty() {
        return 0.5 * outer;
    }
    0.5 * (outer + region.holes[0].length())
}

/// FillRadialZigZag.cpp:139-142 — `static size_t layer_index(const Fill &fill)`
fn layer_index(fill: &Fill) -> usize {
    if fill.layer_id == usize::MAX {
        0
    } else {
        fill.layer_id
    }
}

/// FillRadialZigZag.cpp:144-147 — `static double line_spacing_mm(const Fill &fill, const FillParams &params)`
fn line_spacing_mm(fill: &Fill, params: &FillParams) -> f64 {
    // `double(params.density)`: the C++ member is a float.
    fill.spacing / (0.05_f64).max((params.density as f32) as f64)
}

/// FillRadialZigZag.cpp:149-154 — `static Point lerp_pt(const Point &a, const Point &b, double t)`
fn lerp_pt(a: &Point, b: &Point, t: f64) -> Point {
    // double(a.x()) + t * (double(b.x()) - double(a.x())): fused.
    let x = t.mul_add(b.x as f64 - a.x as f64, a.x as f64);
    let y = t.mul_add(b.y as f64 - a.y as f64, a.y as f64);
    // std::lround: half away from zero.
    Point::new(x.round() as Coord, y.round() as Coord)
}

/// FillRadialZigZag.cpp:156-159 — `static Point avg_pt(const Point &a, const Point &b)`
fn avg_pt(a: &Point, b: &Point) -> Point {
    lerp_pt(a, b, 0.5)
}

/// FillRadialZigZag.cpp:161-164 — `static double dist_pts(const Point &a, const Point &b)`
fn dist_pts(a: &Point, b: &Point) -> f64 {
    // (b - a).cast<double>().norm(): Eigen, not fused.
    let dx = (b.x - a.x) as f64;
    let dy = (b.y - a.y) as f64;
    (dx * dx + dy * dy).sqrt()
}

/// FillRadialZigZag.cpp:166-173 — `static Point raw_pole(const ExPolygon &ex)`
fn raw_pole(ex: &ExPolygon) -> Point {
    // Hole centroid is the radial center of a collar. Bbox center jumps when a
    // thin arm appears or the AABB resizes, even if the hole barely moved.
    if !ex.holes.is_empty() && ex.holes[0].points.len() >= 3 {
        return polygon_centroid(&ex.holes[0]);
    }
    get_extents(ex).center()
}

/// FillRadialZigZag.cpp:175-184 — `static Point hold_or_slew_pole(const Point &prev, const Point &raw)`
fn hold_or_slew_pole(prev: &Point, raw: &Point) -> Point {
    let d = dist_pts(prev, raw);
    if d <= scale_(K_POLE_HOLD_MM) {
        return *prev;
    }
    let step = scale_(K_POLE_SLEW_MM);
    if d <= step {
        return *raw;
    }
    lerp_pt(prev, raw, step / d)
}

/// FillRadialZigZag.cpp:186-193 — `static bool is_supported_topology(const ExPolygon &region)`
fn is_supported_topology(region: &ExPolygon) -> bool {
    if region.contour.points.len() < 3 {
        return false;
    }
    if region.holes.len() > 1 {
        return false;
    }
    region.holes.is_empty() || region.holes[0].points.len() >= 3
}

/// FillRadialZigZag.cpp:195-199 — `static double size_mismatch(double a, double b)`
fn size_mismatch(a: f64, b: f64) -> f64 {
    let lo = (1.0_f64).max(a.min(b));
    a.max(b) / lo
}

/// FillRadialZigZag.cpp:201-212 — `static const RegionLock *find_lock(size_t region_id)`
fn find_lock(g_locks: &BTreeMap<usize, RegionLock>, region_id: usize) -> Option<&RegionLock> {
    if let Some(it) = g_locks.get(&region_id) {
        return Some(it);
    }
    if region_id != 0 {
        if let Some(it) = g_locks.get(&0) {
            return Some(it);
        }
    }
    None
}

/// FillRadialZigZag.cpp:214-240 — `static LipMem recall_lips(size_t region_id, size_t layer_id, const Point &pole)`
fn recall_lips(region_id: usize, layer_id: usize, pole: &Point) -> LipMem {
    let out = LipMem::default();
    if layer_id == 0 {
        return out;
    }
    let g_locks = lock_g_locks();
    let rl = match find_lock(&g_locks, region_id) {
        Some(rl) => rl,
        None => return out,
    };
    let it = match rl.lips_by_layer.get(&(layer_id - 1)) {
        Some(it) => it,
        None => return out,
    };
    let mut best: Option<&LipMem> = None;
    let mut best_d = f64::MAX;
    for m in it {
        if !m.ok {
            continue;
        }
        let d = dist_pts(&m.pole, pole);
        if d < best_d {
            best_d = d;
            best = Some(m);
        }
    }
    if let Some(b) = best {
        if best_d < scale_(8.0) {
            return *b;
        }
    }
    out
}

/// FillRadialZigZag.cpp:242-253 — `static void store_lips(size_t region_id, size_t layer_id, LipMem mem)`
fn store_lips(region_id: usize, layer_id: usize, mem: LipMem) {
    if !mem.ok {
        return;
    }
    let mut g_locks = lock_g_locks();
    let key = if g_locks.contains_key(&region_id) {
        region_id
    } else if region_id != 0 && g_locks.contains_key(&0) {
        0
    } else {
        return;
    };
    if let Some(rl) = g_locks.get_mut(&key) {
        rl.lips_by_layer.entry(layer_id).or_default().push(mem);
    }
}

/// FillRadialZigZag.cpp:255-279 — `static const ExPolygon *lookup_cast_island(size_t region_id, size_t layer_id, const ExPolygon &clip)`
///
/// The C++ returns a pointer into the lock table, read after the mutex is
/// released; a copy is returned here.
fn lookup_cast_island(region_id: usize, layer_id: usize, clip: &ExPolygon) -> Option<ExPolygon> {
    let g_locks = lock_g_locks();
    let rl = find_lock(&g_locks, region_id)?;
    if layer_id >= rl.cast_by_layer.len() {
        return None;
    }
    let row = &rl.cast_by_layer[layer_id];
    if row.is_empty() {
        return None;
    }
    if row.len() == 1 {
        return Some(row[0].clone());
    }
    let mut best: Option<&ExPolygon> = None;
    let mut best_a = -1.0_f64;
    for ex in row {
        let hit = intersection_ex(ex, clip);
        let mut a = 0.0_f64;
        for h in &hit {
            a += h.area().abs();
        }
        if a > best_a {
            best_a = a;
            best = Some(ex);
        }
    }
    if best_a > 1.0 {
        best.cloned()
    } else {
        Some(row[0].clone())
    }
}

/// FillRadialZigZag.cpp:281-326 — `static LockedScan lookup_locked(size_t layer_id, const Point &key, double query_size, size_t region_id)`
fn lookup_locked(layer_id: usize, key: &Point, query_size: f64, region_id: usize) -> LockedScan {
    let g_locks = lock_g_locks();
    let mut out = LockedScan::default();
    let rl = match find_lock(&g_locks, region_id) {
        Some(rl) => rl,
        None => return out,
    };
    out.table_rows = rl.n_by_layer.len();
    if layer_id >= rl.n_by_layer.len() {
        return out;
    }
    let row = &rl.n_by_layer[layer_id];
    out.row_islands = row.len();
    if row.is_empty() {
        return out;
    }
    let mut pick: Option<&LockedIslandN> = None;
    if row.len() == 1 {
        pick = Some(&row[0]);
    } else {
        let mut best_score = f64::MAX;
        for is in row {
            if size_mismatch(query_size, is.size) > 3.0 {
                continue;
            }
            let d = dist_pts(key, &is.key);
            let lim = n_match_limit(query_size, is.size);
            if d > lim {
                continue;
            }
            // size_mismatch(...) * scale_(100.) + d: fused.
            let score = size_mismatch(query_size, is.size).mul_add(scale_(100.0), d);
            if score < best_score {
                best_score = score;
                pick = Some(is);
            }
        }
        if pick.is_none() {
            let mut p = &row[0];
            for is in row {
                if is.size > p.size {
                    p = is;
                }
            }
            pick = Some(p);
        }
    }
    let pick = pick.expect("row is not empty");
    out.n = pick.n;
    out.theta0 = pick.theta0;
    out.pole = pick.pole;
    out.ok = true;
    out.has_pole = true;
    out
}

/// FillRadialZigZag.cpp:328-359 — `static AxisLine fit_axis_line(const std::vector<double> &zs, const std::vector<Point> &poles)`
fn fit_axis_line(zs: &[f64], poles: &[Point]) -> AxisLine {
    let mut line = AxisLine::default();
    if zs.len() < 2 || zs.len() != poles.len() {
        return line;
    }
    let n = zs.len() as f64;
    let (mut sum_z, mut sum_z2, mut sum_x, mut sum_y, mut sum_zx, mut sum_zy) = (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64);
    for i in 0..zs.len() {
        let z = zs[i];
        let x = unscale_d(poles[i].x as f64);
        let y = unscale_d(poles[i].y as f64);
        sum_z += z;
        sum_z2 = z.mul_add(z, sum_z2); // sum_z2 += z * z
        sum_x += x;
        sum_y += y;
        sum_zx = z.mul_add(x, sum_zx); // sum_zx += z * x
        sum_zy = z.mul_add(y, sum_zy); // sum_zy += z * y
    }
    // n * sum_z2 - sum_z * sum_z: the left product is fused, the right rounded first.
    let denom = n.mul_add(sum_z2, -(sum_z * sum_z));
    if denom.abs() < 1e-12 {
        line.x0 = sum_x / n;
        line.y0 = sum_y / n;
        line.ok = true;
        return line;
    }
    line.ax = n.mul_add(sum_zx, -(sum_z * sum_x)) / denom;
    line.ay = n.mul_add(sum_zy, -(sum_z * sum_y)) / denom;
    // (sum_x - line.ax * sum_z) / n: fused.
    line.x0 = (-line.ax).mul_add(sum_z, sum_x) / n;
    line.y0 = (-line.ay).mul_add(sum_z, sum_y) / n;
    line.ok = true;
    line
}

/// FillRadialZigZag.cpp:361-370 — `static bool lookup_axis(double z_mm, Point &out, size_t region_id)`
fn lookup_axis(z_mm: f64, out: &mut Point, region_id: usize) -> bool {
    let g_locks = lock_g_locks();
    let rl = match find_lock(&g_locks, region_id) {
        Some(rl) if rl.axis.ok => rl,
        _ => return false,
    };
    let axis = &rl.axis;
    // axis.x0 + axis.ax * z_mm: fused.
    *out = point_new_scale(axis.ax.mul_add(z_mm, axis.x0), axis.ay.mul_add(z_mm, axis.y0));
    true
}

/// FillRadialZigZag.cpp:372-404 — `static bool solve3(const double A[3][3], const double b[3], double x[3])`
fn solve3(a: &[[f64; 3]; 3], b: &[f64; 3], x: &mut [f64; 3]) -> bool {
    let mut m: [[f64; 4]; 3] = [
        [a[0][0], a[0][1], a[0][2], b[0]],
        [a[1][0], a[1][1], a[1][2], b[1]],
        [a[2][0], a[2][1], a[2][2], b[2]],
    ];
    for col in 0..3 {
        let mut piv = col;
        for r in col + 1..3 {
            if m[r][col].abs() > m[piv][col].abs() {
                piv = r;
            }
        }
        if m[piv][col].abs() < 1e-12 {
            return false;
        }
        if piv != col {
            for c in col..4 {
                let t = m[col][c];
                m[col][c] = m[piv][c];
                m[piv][c] = t;
            }
        }
        let d = m[col][col];
        for c in col..4 {
            m[col][c] /= d;
        }
        for r in 0..3 {
            if r == col {
                continue;
            }
            let f = m[r][col];
            for c in col..4 {
                // M[r][c] -= f * M[col][c]: fused.
                m[r][c] = (-f).mul_add(m[col][c], m[r][c]);
            }
        }
    }
    x[0] = m[0][3];
    x[1] = m[1][3];
    x[2] = m[2][3];
    true
}

/// FillRadialZigZag.cpp:406-421 — `static QuadBezier bezier_from_line(const AxisLine &line, double z0, double z1)`
fn bezier_from_line(line: &AxisLine, z0: f64, z1: f64) -> QuadBezier {
    let mut q = QuadBezier::default();
    if !line.ok {
        return q;
    }
    q.z0 = z0;
    q.z1 = z1;
    // line.x0 + line.ax * z0: fused.
    q.p0x = line.ax.mul_add(z0, line.x0);
    q.p0y = line.ay.mul_add(z0, line.y0);
    q.p2x = line.ax.mul_add(z1, line.x0);
    q.p2y = line.ay.mul_add(z1, line.y0);
    q.p1x = 0.5 * (q.p0x + q.p2x);
    q.p1y = 0.5 * (q.p0y + q.p2y);
    q.ok = true;
    q
}

// Spatial quadratic Bezier through bounding-box centers, parameterized by z.
// B(t) = (1-t)^2 P0 + 2(1-t)t P1 + t^2 P2, t = (z-zmin)/(zmax-zmin).
/// FillRadialZigZag.cpp:425-471 — `static QuadBezier fit_bbox_bezier(const std::vector<double> &zs, const std::vector<Point> &bbs)`
fn fit_bbox_bezier(zs: &[f64], bbs: &[Point]) -> QuadBezier {
    let mut q = QuadBezier::default();
    if zs.len() < 2 || zs.len() != bbs.len() {
        return q;
    }
    let mut zmin = zs[0];
    let mut zmax = zs[0];
    for &z in zs {
        zmin = zmin.min(z);
        zmax = zmax.max(z);
    }
    if zmax - zmin < 1e-9 {
        return bezier_from_line(&fit_axis_line(zs, bbs), zmin, zmax);
    }
    if zs.len() < 3 {
        return bezier_from_line(&fit_axis_line(zs, bbs), zmin, zmax);
    }

    let mut a = [[0.0_f64; 3]; 3];
    let mut bx = [0.0_f64; 3];
    let mut by = [0.0_f64; 3];
    for i in 0..zs.len() {
        let t = (zs[i] - zmin) / (zmax - zmin);
        let u = 1.0 - t;
        let w = [u * u, 2.0 * u * t, t * t];
        let x = unscale_d(bbs[i].x as f64);
        let y = unscale_d(bbs[i].y as f64);
        for r in 0..3 {
            bx[r] = w[r].mul_add(x, bx[r]); // bx[r] += w[r] * x
            by[r] = w[r].mul_add(y, by[r]); // by[r] += w[r] * y
            for c in 0..3 {
                a[r][c] = w[r].mul_add(w[c], a[r][c]); // A[r][c] += w[r] * w[c]
            }
        }
    }
    let mut px = [0.0_f64; 3];
    let mut py = [0.0_f64; 3];
    if !solve3(&a, &bx, &mut px) || !solve3(&a, &by, &mut py) {
        return bezier_from_line(&fit_axis_line(zs, bbs), zmin, zmax);
    }
    q.ok = true;
    q.z0 = zmin;
    q.z1 = zmax;
    q.p0x = px[0];
    q.p1x = px[1];
    q.p2x = px[2];
    q.p0y = py[0];
    q.p1y = py[1];
    q.p2y = py[2];
    q
}

/// `u * u * p0 + 2. * u * t * p1 + t * t * p2` as clang contracts it
/// (FillRadialZigZag.cpp:485-486 and 1983-1984): the left product of each sum is
/// fused, the right one rounded first.
#[inline]
fn bezier_eval(u: f64, t: f64, p0: f64, p1: f64, p2: f64) -> f64 {
    (t * t).mul_add(p2, (u * u).mul_add(p0, 2.0 * u * t * p1))
}

/// FillRadialZigZag.cpp:473-489 — `static bool lookup_bezier(double z_mm, Point &out, size_t region_id)`
fn lookup_bezier(z_mm: f64, out: &mut Point, region_id: usize) -> bool {
    let g_locks = lock_g_locks();
    let rl = match find_lock(&g_locks, region_id) {
        Some(rl) if rl.bezier.ok => rl,
        _ => return false,
    };
    let bz = &rl.bezier;
    let mut t = 0.0_f64;
    if bz.z1 - bz.z0 > 1e-12 {
        t = (z_mm - bz.z0) / (bz.z1 - bz.z0);
    }
    t = (0.0_f64).max((1.0_f64).min(t));
    let u = 1.0 - t;
    let x = bezier_eval(u, t, bz.p0x, bz.p1x, bz.p2x);
    let y = bezier_eval(u, t, bz.p0y, bz.p1y, bz.p2y);
    *out = point_new_scale(x, y);
    true
}

/// FillRadialZigZag.cpp:491-496 — `static double ray_length_scaled(const ExPolygon &region, const Point &pole)`
fn ray_length_scaled(region: &ExPolygon, pole: &Point) -> f64 {
    let bb = get_extents(region);
    let span = bb.size().x.max(bb.size().y) as f64;
    span + dist_pts(pole, &bb.center()) + scaled_coord(4.0) as f64
}

/// FillRadialZigZag.cpp:498-502 — `static Point along_dir(const Point &pole, const Vec2d &dir, double dist)`
fn along_dir(pole: &Point, dir: (f64, f64), dist: f64) -> Point {
    // double(pole.x()) + dir.x() * dist: fused.
    Point::new(
        dir.0.mul_add(dist, pole.x as f64).round() as Coord,
        dir.1.mul_add(dist, pole.y as f64).round() as Coord,
    )
}

// One-way ray t>=0 clipped to Omega. Valid iff exactly one thick material interval.
// A unique graze too thin/short to be a scan is kept as lip (the far hit point)
// so the zigzag can connect to that one point instead of inventing a spoke.
// Pole in a hole: interval is [inner wall, outer wall] (two boundary hits).
// Pole inside solid: interval is the half-chord [near pole, exit] (one boundary hit).
/// FillRadialZigZag.cpp:509-558 — `static bool cast_ray(const ExPolygon &region, const Point &pole, double theta, double hub_scaled, RadialRay &ray)`
fn cast_ray(region: &ExPolygon, pole: &Point, theta: f64, hub_scaled: f64, ray: &mut RadialRay) -> bool {
    *ray = RadialRay::default();
    ray.theta = theta;
    let dir = (theta.cos(), theta.sin());
    let len = ray_length_scaled(region, pole);
    let tip = along_dir(pole, dir, len);
    if tip == *pole {
        return false;
    }

    let probe = Polyline::from_points(vec![*pole, tip]);
    let mut hits = intersection_pl(&probe, region);
    hits.retain(|pl| !(pl.points.len() < 2));
    let min_len = scale_(K_MIN_SEG_MM);
    let mut long_hits = hits.clone();
    long_hits.retain(|pl| !(pl.length() < min_len));
    if long_hits.len() != 1 && hits.len() != 1 {
        return false;
    }
    let mut hit = if long_hits.len() == 1 { long_hits[0].clone() } else { hits[0].clone() };
    if dist_pts(&hit.points[0], pole) > dist_pts(&hit.points[hit.points.len() - 1], pole) {
        hit.reverse();
    }

    ray.inner = hit.points[0];
    ray.outer = hit.points[hit.points.len() - 1];
    let pole_inside = region.contains(pole, true);
    if pole_inside && hub_scaled > 1.0 {
        if dist_pts(&ray.outer, pole) <= hub_scaled + min_len {
            return false;
        }
        let hub = along_dir(pole, dir, hub_scaled);
        // dist_pts(hub, pole) + min_len * 0.25: fused.
        if region.contains(&hub, true) && min_len.mul_add(0.25, dist_pts(&hub, pole)) < dist_pts(&ray.outer, pole) {
            ray.inner = hub;
        }
    }
    ray.mid = avg_pt(&ray.inner, &ray.outer);
    ray.inner_radius = dist_pts(&ray.inner, pole);
    ray.outer_radius = dist_pts(&ray.outer, pole);
    // ray.inner_radius + min_len * 0.25: fused.
    ray.valid = long_hits.len() == 1 && ray.outer_radius > min_len.mul_add(0.25, ray.inner_radius);
    if !ray.valid {
        ray.lip = true;
        ray.inner = ray.outer;
        ray.mid = ray.outer;
        ray.inner_radius = ray.outer_radius;
    }
    ray.valid
}

// Fill region is inset from the slice. A graze on the cut can miss Omega but
// still nick the outset. Keep that as a single lip point on Omega's contour —
// never promote it to a scan.
/// FillRadialZigZag.cpp:563-581 — `static void maybe_outset_lip(const ExPolygon &region, const ExPolygons &fat, const Point &pole, double hub_scaled, RadialRay &ray)`
fn maybe_outset_lip(region: &ExPolygon, fat: &[ExPolygon], pole: &Point, hub_scaled: f64, ray: &mut RadialRay) {
    if ray.valid || ray.lip || fat.is_empty() {
        return;
    }
    let mut fr = RadialRay::default();
    if !cast_ray(&fat[0], pole, ray.theta, hub_scaled, &mut fr) && !fr.lip {
        return;
    }
    let on_fill = region.point_projection(&fr.outer);
    if dist_pts(&on_fill, &fr.outer) > scale_(2.0) {
        return;
    }
    ray.lip = true;
    ray.valid = false;
    ray.outer = on_fill;
    ray.inner = on_fill;
    ray.mid = on_fill;
    ray.outer_radius = dist_pts(&on_fill, pole);
    ray.inner_radius = ray.outer_radius;
}

/// FillRadialZigZag.cpp:583 — `enum class RayClass { Miss, One, Multi };`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RayClass {
    Miss,
    One,
    Multi,
}

/// FillRadialZigZag.cpp:585-605 — `static RayClass classify_ray(const ExPolygon &region, const Point &pole, double theta)`
fn classify_ray(region: &ExPolygon, pole: &Point, theta: f64) -> RayClass {
    let dir = (theta.cos(), theta.sin());
    let len = ray_length_scaled(region, pole);
    let tip = along_dir(pole, dir, len);
    if tip == *pole {
        return RayClass::Miss;
    }

    let probe = Polyline::from_points(vec![*pole, tip]);
    let mut hits = intersection_pl(&probe, region);
    let min_len = scale_(K_MIN_SEG_MM);
    hits.retain(|pl| !(pl.points.len() < 2 || pl.length() < min_len));
    if hits.is_empty() {
        return RayClass::Miss;
    }
    if hits.len() == 1 {
        return RayClass::One;
    }
    RayClass::Multi
}

// R = (# one-interval hits) / (# rays that hit any material).
// A miss means "no material in this direction" (open side of a U), not a bad pole.
/// FillRadialZigZag.cpp:609-623 — `static double evaluate_pole(const ExPolygon &region, const Point &pole, int n_test)`
fn evaluate_pole(region: &ExPolygon, pole: &Point, n_test: i32) -> f64 {
    let (mut n_one, mut n_multi) = (0i32, 0i32);
    for i in 0..n_test {
        match classify_ray(region, pole, 2.0 * PI * i as f64 / n_test as f64) {
            RayClass::One => n_one += 1,
            RayClass::Multi => n_multi += 1,
            RayClass::Miss => {}
        }
    }
    let n_hit = n_one + n_multi;
    if n_one < K_MIN_HIT_RAYS || n_hit <= 0 {
        return 0.0;
    }
    n_one as f64 / n_hit as f64
}

/// FillRadialZigZag.cpp:630-635 — the lambda `push_unique` of `pole_candidates`.
fn push_unique(out: &mut Vec<Point>, p: &Point) {
    for q in out.iter() {
        if dist_pts(p, q) < scale_(0.05) {
            return;
        }
    }
    out.push(*p);
}

/// FillRadialZigZag.cpp:625-644 — `static std::vector<Point> pole_candidates(const ExPolygon &region)`
fn pole_candidates(region: &ExPolygon) -> Vec<Point> {
    let mut out: Vec<Point> = Vec::new();
    let bb_c = get_extents(region).center();
    let outer_c = polygon_centroid(&region.contour);
    push_unique(&mut out, &bb_c);
    push_unique(&mut out, &outer_c);
    if !region.holes.is_empty() {
        let inner_c = polygon_centroid(&region.holes[0]);
        push_unique(&mut out, &inner_c);
        push_unique(&mut out, &avg_pt(&inner_c, &outer_c));
    }
    out
}

/// FillRadialZigZag.cpp:646-679 — `static bool pick_pole(const ExPolygon &region, bool has_lock, const Point &pole_lock, Point &out_pole, double &out_r)`
fn pick_pole(region: &ExPolygon, has_lock: bool, pole_lock: &Point, out_pole: &mut Point, out_r: &mut f64) -> bool {
    if has_lock {
        *out_r = evaluate_pole(region, pole_lock, K_TEST_RAYS);
        if *out_r >= K_VALID_THRESHOLD {
            *out_pole = *pole_lock;
            return true;
        }
        let raw = raw_pole(region);
        for t in [0.25, 0.5, 0.75, 1.0] {
            let p = lerp_pt(pole_lock, &raw, t);
            let r = evaluate_pole(region, &p, K_TEST_RAYS);
            if r >= K_VALID_THRESHOLD {
                *out_pole = p;
                *out_r = r;
                return true;
            }
        }
    }
    let candidates = pole_candidates(region);
    *out_pole = if candidates.is_empty() { get_extents(region).center() } else { candidates[0] };
    *out_r = evaluate_pole(region, out_pole, K_TEST_RAYS);
    if *out_r >= K_VALID_THRESHOLD {
        return true;
    }
    for i in 1..candidates.len() {
        let r = evaluate_pole(region, &candidates[i], K_TEST_RAYS);
        if r > *out_r {
            *out_r = r;
            *out_pole = candidates[i];
        }
    }
    *out_r >= K_VALID_THRESHOLD
}

/// FillRadialZigZag.cpp:681-691 — `static std::vector<RadialRay> dense_rays(const ExPolygon &region, const Point &pole, double hub_scaled, int n)`
fn dense_rays(region: &ExPolygon, pole: &Point, hub_scaled: f64, n: i32) -> Vec<RadialRay> {
    let mut rays: Vec<RadialRay> = Vec::with_capacity(n as usize);
    for i in 0..n {
        let mut ray = RadialRay::default();
        if cast_ray(region, pole, 2.0 * PI * i as f64 / n as f64, hub_scaled, &mut ray) {
            rays.push(ray);
        }
    }
    rays
}

/// FillRadialZigZag.cpp:693-698 — `struct MidSample`
#[derive(Debug, Clone, Copy)]
struct MidSample {
    s: f64,
    theta: f64,
    #[allow(dead_code)]
    mid: Point,
}

/// FillRadialZigZag.cpp:700-726 — `static bool build_mid_samples(const std::vector<RadialRay> &rays, std::vector<MidSample> &samples, double &length, bool &closed)`
fn build_mid_samples(rays: &[RadialRay], samples: &mut Vec<MidSample>, length: &mut f64, closed: &mut bool) -> bool {
    samples.clear();
    *length = 0.0;
    *closed = false;
    if rays.len() < 8 {
        return false;
    }

    samples.reserve(rays.len());
    for i in 0..rays.len() {
        samples.push(MidSample { s: 0.0, theta: rays[i].theta, mid: rays[i].mid });
    }

    let mut max_internal = 0.0_f64;
    let mut i = 0usize;
    while i + 1 < rays.len() {
        let d = dist_pts(&rays[i].mid, &rays[i + 1].mid);
        samples[i + 1].s = samples[i].s + d;
        max_internal = max_internal.max(d);
        i += 1;
    }
    let wrap_d = dist_pts(&rays[rays.len() - 1].mid, &rays[0].mid);
    // Full ring: wrap is comparable to other steps. Open U/C: wrap jumps the empty side.
    *closed = wrap_d <= scale_(K_MIN_SEG_MM).max(2.5 * max_internal);
    *length = samples[samples.len() - 1].s;
    if *closed {
        *length += wrap_d;
    }
    *length > scale_(K_MIN_SEG_MM)
}

/// FillRadialZigZag.cpp:728-735 — `static double wrap_theta(double th)`
fn wrap_theta(mut th: f64) -> f64 {
    while th < 0.0 {
        th += 2.0 * PI;
    }
    while th >= 2.0 * PI {
        th -= 2.0 * PI;
    }
    th
}

/// FillRadialZigZag.cpp:737-743 — `static double ccw_from(double from, double to)`
fn ccw_from(from: f64, to: f64) -> f64 {
    let mut d = wrap_theta(to) - wrap_theta(from);
    if d < 0.0 {
        d += 2.0 * PI;
    }
    d
}

/// FillRadialZigZag.cpp:745-777 — `static double sample_theta_at_s(const std::vector<MidSample> &samples, double length, double s, bool closed)`
fn sample_theta_at_s(samples: &[MidSample], length: f64, mut s: f64, closed: bool) -> f64 {
    if samples.is_empty() || length <= 1.0 {
        return 0.0;
    }
    if closed {
        s %= length; // std::fmod
        if s < 0.0 {
            s += length;
        }
    } else {
        s = (0.0_f64).max(s.min(length));
    }

    let nseg = if closed {
        samples.len()
    } else if !samples.is_empty() {
        samples.len() - 1
    } else {
        0
    };
    for i in 0..nseg {
        let a = &samples[i];
        let b = &samples[(i + 1) % samples.len()];
        let s0 = a.s;
        let s1 = if closed && i + 1 == samples.len() { length } else { b.s };
        if s > s1 + 1e-9 {
            continue;
        }
        let span = (1.0_f64).max(s1 - s0);
        let t = (0.0_f64).max((1.0_f64).min((s - s0) / span));
        let th0 = a.theta;
        let th1 = b.theta;
        let mut dth = th1 - th0;
        if dth > PI {
            dth -= 2.0 * PI;
        }
        if dth < -PI {
            dth += 2.0 * PI;
        }
        // th0 + t * dth: fused.
        return wrap_theta(t.mul_add(dth, th0));
    }
    wrap_theta(samples[samples.len() - 1].theta)
}

/// FillRadialZigZag.cpp:779-803 — `static double s_for_theta(const std::vector<MidSample> &samples, double length, bool closed, double target)`
fn s_for_theta(samples: &[MidSample], length: f64, closed: bool, target: f64) -> f64 {
    if samples.is_empty() {
        return 0.0;
    }
    let n = samples.len();
    let nseg = if closed {
        n
    } else if n > 0 {
        n - 1
    } else {
        0
    };
    let mut best_s = samples[0].s;
    let mut best_d = ang_diff(samples[0].theta, target);
    for i in 0..nseg {
        let a = &samples[i];
        let b = &samples[(i + 1) % n];
        let s0 = a.s;
        let s1 = if closed && i + 1 == n { length } else { b.s };
        let d_ab = ccw_from(a.theta, b.theta);
        let d_at = ccw_from(a.theta, target);
        if d_ab > 1e-12 && d_at <= d_ab + 1e-12 {
            // s0 + (d_at / d_ab) * (s1 - s0): fused.
            return (d_at / d_ab).mul_add(s1 - s0, s0);
        }
        let d = ang_diff(a.theta, target);
        if d < best_d {
            best_d = d;
            best_s = a.s;
        }
    }
    best_s
}

/// FillRadialZigZag.cpp:805-831 — `static bool locate_on_polygon(const Polygon &poly, const Point &p, size_t &seg, double &t)`
fn locate_on_polygon(poly: &Polygon, p: &Point, seg: &mut usize, t: &mut f64) -> bool {
    let pts = &poly.points;
    if pts.len() < 2 {
        return false;
    }
    let mut best = f64::MAX;
    *seg = 0;
    *t = 0.0;
    let n = pts.len();
    for i in 0..n {
        let a = pts[i];
        let b = pts[(i + 1) % n];
        let ab = ((b.x - a.x) as f64, (b.y - a.y) as f64);
        // Eigen squaredNorm / dot: not fused.
        let len2 = ab.0 * ab.0 + ab.1 * ab.1;
        let mut u = 0.0_f64;
        if len2 > 1.0 {
            let pa = ((p.x - a.x) as f64, (p.y - a.y) as f64);
            u = (0.0_f64).max((1.0_f64).min((pa.0 * ab.0 + pa.1 * ab.1) / len2));
        }
        let q = lerp_pt(&a, &b, u);
        let d = dist_pts(p, &q);
        if d < best {
            best = d;
            *seg = i;
            *t = u;
        }
    }
    best < scale_(3.0)
}

/// FillRadialZigZag.cpp:833-842 — `static double dist_to_polygon(const Polygon &poly, const Point &p)`
fn dist_to_polygon(poly: &Polygon, p: &Point) -> f64 {
    let mut s = 0usize;
    let mut t = 0.0_f64;
    if !locate_on_polygon(poly, p, &mut s, &mut t) {
        return f64::MAX;
    }
    let pts = &poly.points;
    let q = lerp_pt(&pts[s], &pts[(s + 1) % pts.len()], t);
    dist_pts(p, &q)
}

/// FillRadialZigZag.cpp:847-849 and 869-871 — the lambda `at` of `walk_forward` / `walk_backward`.
#[inline]
fn walk_at(pts: &[Point], s: usize, u: f64) -> Point {
    let n = pts.len();
    lerp_pt(&pts[s], &pts[(s + 1) % n], u)
}

/// FillRadialZigZag.cpp:844-864 — `static Polyline walk_forward(const Points &pts, size_t sa, double ta, size_t sb, double tb)`
fn walk_forward(pts: &[Point], sa: usize, ta: f64, sb: usize, tb: f64) -> Polyline {
    let n = pts.len();
    let mut pl = Polyline::new();
    pl.append_point(walk_at(pts, sa, ta));
    if sa == sb && tb + 1e-9 >= ta {
        pl.append_point(walk_at(pts, sb, tb));
        return pl;
    }
    let mut i = (sa + 1) % n;
    pl.append_point(pts[i]);
    while i != sb {
        i = (i + 1) % n;
        pl.append_point(pts[i]);
    }
    pl.append_point(walk_at(pts, sb, tb));
    pl
}

/// FillRadialZigZag.cpp:866-886 — `static Polyline walk_backward(const Points &pts, size_t sa, double ta, size_t sb, double tb)`
fn walk_backward(pts: &[Point], sa: usize, ta: f64, sb: usize, tb: f64) -> Polyline {
    let n = pts.len();
    let mut pl = Polyline::new();
    pl.append_point(walk_at(pts, sa, ta));
    if sa == sb && ta + 1e-9 >= tb {
        pl.append_point(walk_at(pts, sb, tb));
        return pl;
    }
    pl.append_point(pts[sa]);
    let mut i = sa;
    while i != (sb + 1) % n {
        i = (i + n - 1) % n;
        pl.append_point(pts[i]);
    }
    pl.append_point(walk_at(pts, sb, tb));
    pl
}

/// FillRadialZigZag.cpp:888-894 — `static bool locate_or_project(const Polygon &poly, Point p, size_t &seg, double &t)`
fn locate_or_project(poly: &Polygon, mut p: Point, seg: &mut usize, t: &mut f64) -> bool {
    if locate_on_polygon(poly, &p, seg, t) {
        return true;
    }
    p = poly.point_projection(&p);
    locate_on_polygon(poly, &p, seg, t)
}

/// FillRadialZigZag.cpp:896-909 — `static Polyline shorter_boundary_arc(const Polygon &poly, const Point &a, const Point &b)`
fn shorter_boundary_arc(poly: &Polygon, a: &Point, b: &Point) -> Polyline {
    let (mut sa, mut sb) = (0usize, 0usize);
    let (mut ta, mut tb) = (0.0_f64, 0.0_f64);
    if !locate_or_project(poly, *a, &mut sa, &mut ta) || !locate_or_project(poly, *b, &mut sb, &mut tb) {
        return Polyline::new();
    }
    let fwd = walk_forward(&poly.points, sa, ta, sb, tb);
    let bak = walk_backward(&poly.points, sa, ta, sb, tb);
    if fwd.points.len() < 2 {
        return bak;
    }
    if bak.points.len() < 2 {
        return fwd;
    }
    if fwd.length() <= bak.length() {
        fwd
    } else {
        bak
    }
}

// Length-weighted |R - r0|. Vertex-mean radius is biased by tessellation: a
// concave outer dent can have many low-R verts, so the short same-rim walk
// looks "inner" and the long way around the C wins — then arc_lim rejects it
// and the adjacent 铆线 disappears (layers 1–154 on an open vamp).
/// FillRadialZigZag.cpp:915-929 — `static double radius_mad_len(const Polyline &pl, const Point &pole, double r0)`
fn radius_mad_len(pl: &Polyline, pole: &Point, r0: f64) -> f64 {
    if pl.points.len() < 2 {
        return f64::MAX;
    }
    let (mut acc, mut l) = (0.0_f64, 0.0_f64);
    for i in 1..pl.points.len() {
        let len = dist_pts(&pl.points[i - 1], &pl.points[i]);
        if len < 1.0 {
            continue;
        }
        let r = 0.5 * (dist_pts(&pl.points[i - 1], pole) + dist_pts(&pl.points[i], pole));
        acc = (r - r0).abs().mul_add(len, acc); // acc += std::abs(r - r0) * len
        l += len;
    }
    if l > 1.0 {
        acc / l
    } else {
        f64::MAX
    }
}

/// FillRadialZigZag.cpp:931-958 — `static bool polyline_has_radial_cut(const Polyline &pl, const Point &pole)`
fn polyline_has_radial_cut(pl: &Polyline, pole: &Point) -> bool {
    if pl.points.len() < 2 {
        return false;
    }
    let mut rmin = dist_pts(&pl.points[0], pole);
    let mut rmax = rmin;
    let mut radial_seg = false;
    for i in 0..pl.points.len() {
        let r = dist_pts(&pl.points[i], pole);
        rmin = rmin.min(r);
        rmax = rmax.max(r);
        if i == 0 {
            continue;
        }
        let len = dist_pts(&pl.points[i - 1], &pl.points[i]);
        if len < scale_(1.2) {
            continue;
        }
        let v = ((pl.points[i].x - pl.points[i - 1].x) as f64, (pl.points[i].y - pl.points[i - 1].y) as f64);
        let rad = ((pl.points[i - 1].x - pole.x) as f64, (pl.points[i - 1].y - pole.y) as f64);
        let rn = (rad.0 * rad.0 + rad.1 * rad.1).sqrt();
        if rn < 1.0 {
            continue;
        }
        if ((v.0 * rad.0 + v.1 * rad.1) / (len * rn)).abs() > 0.88 {
            radial_seg = true;
        }
    }
    // A real cut walks inner↔outer (several mm of radius). A dented O-ring hop
    // can have one locally-radial edge without leaving its rim.
    radial_seg && (rmax - rmin) > scale_(2.0)
}

// On a C (one contour, no hole) the inner and outer rims share the same polygon.
// Stay on the rim the endpoints already sit on (radius of a,b). Never walk the
// radial cut (inner↔outer) — that is the opening 回环.
/// FillRadialZigZag.cpp:963-990 — `static Polyline same_side_boundary_arc(const Polygon &poly, const Point &a, const Point &b, const Point &pole, bool /*want_outer*/)`
fn same_side_boundary_arc(poly: &Polygon, a: &Point, b: &Point, pole: &Point, _want_outer: bool) -> Polyline {
    let (mut sa, mut sb) = (0usize, 0usize);
    let (mut ta, mut tb) = (0.0_f64, 0.0_f64);
    if !locate_or_project(poly, *a, &mut sa, &mut ta) || !locate_or_project(poly, *b, &mut sb, &mut tb) {
        return Polyline::new();
    }
    let fwd = walk_forward(&poly.points, sa, ta, sb, tb);
    let bak = walk_backward(&poly.points, sa, ta, sb, tb);
    let fwd_cut = polyline_has_radial_cut(&fwd, pole);
    let bak_cut = polyline_has_radial_cut(&bak, pole);
    if fwd_cut && !bak_cut {
        return if bak.points.len() >= 2 { bak } else { Polyline::new() };
    }
    if bak_cut && !fwd_cut {
        return if fwd.points.len() >= 2 { fwd } else { Polyline::new() };
    }
    if fwd_cut && bak_cut {
        return Polyline::new();
    }
    if fwd.points.len() < 2 {
        return bak;
    }
    if bak.points.len() < 2 {
        return fwd;
    }
    let r0 = 0.5 * (dist_pts(a, pole) + dist_pts(b, pole));
    let df = radius_mad_len(&fwd, pole, r0);
    let db = radius_mad_len(&bak, pole, r0);
    if (df - db).abs() < scale_(0.35) {
        return if fwd.length() <= bak.length() { fwd } else { bak };
    }
    if df <= db {
        fwd
    } else {
        bak
    }
}

/// FillRadialZigZag.cpp:992-1008 — `static bool step_is_radial_cut(const Point &a, const Point &b, const Point &pole)`
fn step_is_radial_cut(a: &Point, b: &Point, pole: &Point) -> bool {
    let len = dist_pts(a, b);
    if len < scale_(0.4) {
        return false;
    }
    let dr = (dist_pts(b, pole) - dist_pts(a, pole)).abs();
    // Stay on the rim: a slightly diagonal contour edge is not the opening cut.
    if dr < scale_(0.8) {
        return false;
    }
    let v = ((b.x - a.x) as f64, (b.y - a.y) as f64);
    let r = ((a.x - pole.x) as f64, (a.y - pole.y) as f64);
    let rn = (r.0 * r.0 + r.1 * r.1).sqrt();
    if rn < 1.0 {
        return false;
    }
    let al = ((v.0 * r.0 + v.1 * r.1) / (len * rn)).abs();
    (al > 0.85 && len >= scale_(1.2)) || (dr / len > 0.65 && dr > scale_(1.2))
}

// Walk the rim from `from` until the opening cut. Keep the corner vertex;
// do not step onto the radial cut (inner↔outer).
/// FillRadialZigZag.cpp:1012-1061 — `static Polyline rim_extend(const Polygon &poly, const Point &from, bool forward, const Point &pole, double r0, double band, double max_len)`
fn rim_extend(poly: &Polygon, from: &Point, forward: bool, pole: &Point, r0: f64, band: f64, max_len: f64) -> Polyline {
    let mut s = 0usize;
    let mut t = 0.0_f64;
    let mut on = *from;
    if !locate_on_polygon(poly, from, &mut s, &mut t) {
        on = poly.point_projection(from);
        if !locate_on_polygon(poly, &on, &mut s, &mut t) {
            return Polyline::new();
        }
    }
    let pts = &poly.points;
    let n = pts.len();
    if n < 3 {
        return Polyline::new();
    }
    let mut pl = Polyline::new();
    pl.append_point(*from);
    let mut len = 0.0_f64;
    let mut prev = *from;
    if dist_pts(from, &on) >= scale_(0.05) {
        pl.append_point(on);
        prev = on;
    }
    let mut i = if forward { (s + 1) % n } else { s };
    let mut guard = n as i32 + 2;
    loop {
        // while (guard-- > 0)
        let cont = guard > 0;
        guard -= 1;
        if !cont {
            break;
        }
        let q = pts[i];
        let step = dist_pts(&prev, &q);
        if step < 1.0 {
            i = if forward { (i + 1) % n } else { (i + n - 1) % n };
            continue;
        }
        if step_is_radial_cut(&prev, &q, pole) {
            break;
        }
        if (dist_pts(&q, pole) - r0).abs() > band {
            break;
        }
        if len + step > max_len {
            let u = (max_len - len) / step;
            let w = lerp_pt(&prev, &q, u);
            if !step_is_radial_cut(&prev, &w, pole) && (dist_pts(&w, pole) - r0).abs() <= band {
                pl.append_point(w);
            }
            break;
        }
        pl.append_point(q);
        len += step;
        prev = q;
        i = if forward { (i + 1) % n } else { (i + n - 1) % n };
    }
    if pl.points.len() >= 2 {
        pl
    } else {
        Polyline::new()
    }
}

/// FillRadialZigZag.cpp:1063-1066 — `static double point_theta(const Point &p, const Point &pole)`
fn point_theta(p: &Point, pole: &Point) -> f64 {
    ((p.y - pole.y) as f64).atan2((p.x - pole.x) as f64)
}

/// FillRadialZigZag.cpp:1068-1081 — `static bool connector_inside(const Point &a, const Point &b, const ExPolygon &region, const ExPolygons &fat, double max_len)`
fn connector_inside(a: &Point, b: &Point, region: &ExPolygon, fat: &[ExPolygon], max_len: f64) -> bool {
    let len = dist_pts(a, b);
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

/// FillRadialZigZag.cpp:1083-1088 — `static bool on_hub_radius(const Point &p, const Point &pole, double hub_cut_scaled, double tol_mm = 4.5)`
fn on_hub_radius(p: &Point, pole: &Point, hub_cut_scaled: f64, tol_mm: f64) -> bool {
    if hub_cut_scaled <= 1.0 {
        return false;
    }
    (dist_pts(p, pole) - hub_cut_scaled).abs() < scale_(tol_mm)
}

/// The default argument of `on_hub_radius` (FillRadialZigZag.cpp:1083): `tol_mm = 4.5`.
const ON_HUB_RADIUS_TOL_MM: f64 = 4.5;

// Multi-vertex walk that stays on circle B. A 2-point 铆线 is not this.
/// FillRadialZigZag.cpp:1091-1112 — `static bool polyline_follows_hub(const Polyline &pl, const Point &pole, double hub_cut_scaled)`
fn polyline_follows_hub(pl: &Polyline, pole: &Point, hub_cut_scaled: f64) -> bool {
    if hub_cut_scaled <= 1.0 || pl.points.len() < 3 {
        return false;
    }
    let (mut on, mut tot) = (0.0_f64, 0.0_f64);
    let mut n_hub = 0;
    for p in &pl.points {
        if on_hub_radius(p, pole, hub_cut_scaled, 3.5) {
            n_hub += 1;
        }
    }
    for i in 1..pl.points.len() {
        let len = dist_pts(&pl.points[i - 1], &pl.points[i]);
        if len < 1.0 {
            continue;
        }
        tot += len;
        let r = 0.5 * (dist_pts(&pl.points[i - 1], pole) + dist_pts(&pl.points[i], pole));
        if (r - hub_cut_scaled).abs() < scale_(3.5) {
            on += len;
        }
    }
    if n_hub >= 3 {
        return true;
    }
    on > scale_(6.0) || (tot > scale_(4.0) && on > 0.55 * tot)
}

/// FillRadialZigZag.cpp:1123-1131 — the lambda `try_straight` of `make_connector`.
fn try_straight(a: &Point, b: &Point, chord: f64, region: &ExPolygon, fat: &[ExPolygon], max_len: f64, out: &mut Polyline) -> bool {
    // Adjacent rims at ~45 mm / 90 rays are ~3 mm apart; 4.5×spacing can be
    // shorter than that when infill is dense, so always allow this chord.
    let lim = max_len.max((chord * 1.1).min(scale_(8.0)));
    if !connector_inside(a, b, region, fat, lim) {
        return false;
    }
    *out = Polyline::from_points(vec![*a, *b]);
    true
}

/// FillRadialZigZag.cpp:1144-1155 — the lambda `finish_arc` of `make_connector`.
fn finish_arc(mut arc: Polyline, arc_lim: f64, a: &Point, b: &Point, pole: &Point, hub_cut_scaled: f64, out: &mut Polyline) -> bool {
    if arc.points.len() < 2 || arc.length() > arc_lim {
        return false;
    }
    if arc.points[0] != *a {
        arc.append_before(*a);
    }
    if arc.points[arc.points.len() - 1] != *b {
        arc.append_point(*b);
    }
    if polyline_follows_hub(&arc, pole, hub_cut_scaled) {
        return false;
    }
    *out = arc;
    out.points.len() >= 2
}

/// FillRadialZigZag.cpp:1114-1185 — `static bool make_connector(const Point &a, const Point &b, bool toward_outer, const ExPolygon &region, const ExPolygons &fat, double max_len, const Point &pole, Polyline &out, double hub_cut_scaled = 0.)`
#[allow(clippy::too_many_arguments)]
fn make_connector(
    a: &Point,
    b: &Point,
    toward_outer: bool,
    region: &ExPolygon,
    fat: &[ExPolygon],
    max_len: f64,
    pole: &Point,
    out: &mut Polyline,
    hub_cut_scaled: f64,
) -> bool {
    *out = Polyline::new();
    let chord = dist_pts(a, b);
    if chord < scale_(0.05) {
        return true;
    }

    // Inner 铆线 at circle B: short a–b chord. Do not walk the punched hole
    // or the C-shaped circular bite (that extra stroke = 大圆边).
    if on_hub_radius(a, pole, hub_cut_scaled, ON_HUB_RADIUS_TOL_MM) || on_hub_radius(b, pole, hub_cut_scaled, ON_HUB_RADIUS_TOL_MM) {
        if try_straight(a, b, chord, region, fat, max_len, out) {
            return true;
        }
        return false;
    }

    if try_straight(a, b, chord, region, fat, max_len, out) {
        return true;
    }

    // Chord left the island (C opening, real inner hole). Walk the real contour,
    // not an inset copy — stitching onto a 0.08 mm inset made a dogleg at each end.
    if !region.holes.is_empty() {
        if hub_cut_scaled > 1.0 && !toward_outer {
            return false;
        }
        let poly = if toward_outer { &region.contour } else { &region.holes[0] };
        let arc_lim = (max_len * 3.0).max(0.35 * poly.length());
        if finish_arc(shorter_boundary_arc(poly, a, b), arc_lim, a, b, pole, hub_cut_scaled, out) {
            return true;
        }
        return false;
    }

    // C / U: both endpoints sit on the one contour (inner and outer rims share
    // it). Do not walk a split inset fragment: that snaps hops onto the other rim.
    let cpoly = &region.contour;
    let both_on_rim = dist_to_polygon(cpoly, a) < scale_(1.2) && dist_to_polygon(cpoly, b) < scale_(1.2);
    if both_on_rim {
        let arc_lim = (max_len * 3.0).max(12.0 * chord).max(0.42 * cpoly.length());
        if finish_arc(same_side_boundary_arc(cpoly, a, b, pole, toward_outer), arc_lim, a, b, pole, hub_cut_scaled, out)
            && !polyline_has_radial_cut(out, pole)
        {
            return true;
        }
        let sh = shorter_boundary_arc(cpoly, a, b);
        if !polyline_has_radial_cut(&sh, pole)
            && finish_arc(sh, arc_lim, a, b, pole, hub_cut_scaled, out)
            && !polyline_has_radial_cut(out, pole)
        {
            return true;
        }
    }
    false
}

/// FillRadialZigZag.cpp:1187-1209 — `static bool path_inside_region(const Polylines &paths, const ExPolygon &region)`
#[allow(dead_code)]
fn path_inside_region(paths: &[Polyline], region: &ExPolygon) -> bool {
    let fat = offset_ex(region, scale_(0.45) as f32);
    for pl in paths {
        for i in 1..pl.points.len() {
            let (la, lb) = (pl.points[i - 1], pl.points[i]);
            if dist_pts(&la, &lb) < scale_(0.05) {
                continue;
            }
            let mut ok = expolygon_contains_line(region, la, lb);
            if !ok {
                for ex in &fat {
                    if expolygon_contains_line(ex, la, lb) {
                        ok = true;
                        break;
                    }
                }
            }
            if !ok {
                return false;
            }
        }
    }
    !paths.is_empty()
}

/// FillRadialZigZag.cpp:1211-1220 — `static int wrap_idx(int i, int n, bool closed)`
fn wrap_idx(i: i32, n: i32, closed: bool) -> i32 {
    if n <= 0 {
        return -1;
    }
    if closed {
        return (i % n + n) % n;
    }
    if i < 0 || i >= n {
        return -1;
    }
    i
}

/// FillRadialZigZag.cpp:1222-1254 — `static bool clip_scan_chord(const Point &from, const Point &to, const ExPolygon &region, Point &a, Point &b)`
fn clip_scan_chord(from: &Point, to: &Point, region: &ExPolygon, a: &mut Point, b: &mut Point) -> bool {
    if from == to {
        return false;
    }
    let probe = Polyline::from_points(vec![*from, *to]);
    let mut hits = intersection_pl(&probe, region);
    let min_len = scale_(K_MIN_SEG_MM);
    hits.retain(|pl| !(pl.points.len() < 2 || pl.length() < min_len));
    if hits.is_empty() {
        return false;
    }
    // Prefer a 2-point radial clip. A long hole-walk (64-gon around circle B)
    // is longer than the real scan and used to become the infill path.
    let mut best = &hits[0];
    for pl in &hits {
        let simple = pl.points.len() <= 3;
        let best_simple = best.points.len() <= 3;
        if simple != best_simple {
            if simple {
                best = pl;
            }
            continue;
        }
        if pl.length() > best.length() {
            best = pl;
        }
    }
    *a = best.points[0];
    *b = best.points[best.points.len() - 1];
    if dist_pts(a, from) > dist_pts(b, from) {
        std::mem::swap(a, b);
    }
    dist_pts(a, b) >= min_len
}

/// FillRadialZigZag.cpp:1256-1261 — `static bool in_circular(int i, int a, int b, int n)`
fn in_circular(i: i32, a: i32, b: i32, _n: i32) -> bool {
    if a <= b {
        return i >= a && i <= b;
    }
    i >= a || i <= b
}

/// FillRadialZigZag.cpp:1263-1287 — `static bool longest_invalid_run(const std::vector<char> &valid, int &gap_a, int &gap_b)`
fn longest_invalid_run(valid: &[u8], gap_a: &mut i32, gap_b: &mut i32) -> bool {
    let n = valid.len() as i32;
    *gap_a = -1;
    *gap_b = -1;
    if n < 3 {
        return false;
    }
    let mut best = 0;
    for i in 0..n {
        if valid[i as usize] != 0 {
            continue;
        }
        if valid[((i + n - 1) % n) as usize] != 0 {
            let (mut len, mut j) = (0, i);
            while len < n && valid[j as usize] == 0 {
                j = (j + 1) % n;
                len += 1;
            }
            if len > best {
                best = len;
                *gap_a = i;
                *gap_b = (i + len - 1 + n) % n;
            }
        }
    }
    best >= 1
}

/// FillRadialZigZag.cpp:1289-1298 — `static Polyline trim_to_len(Polyline pl, double max_len)`
fn trim_to_len(mut pl: Polyline, max_len: f64) -> Polyline {
    if pl.points.len() < 2 {
        return Polyline::new();
    }
    let l = pl.length();
    if l <= max_len + 1.0 {
        return pl;
    }
    pl.clip_end(l - max_len);
    if pl.points.len() >= 2 {
        pl
    } else {
        Polyline::new()
    }
}

/// FillRadialZigZag.cpp:1300-1310 — `static const Polygon *poly_for_point(const ExPolygon &region, const Point &p)`
fn poly_for_point<'a>(region: &'a ExPolygon, p: &Point) -> Option<&'a Polygon> {
    let mut s = 0usize;
    let mut t = 0.0_f64;
    if locate_on_polygon(&region.contour, p, &mut s, &mut t) {
        return Some(&region.contour);
    }
    for h in &region.holes {
        if locate_on_polygon(h, p, &mut s, &mut t) {
            return Some(h);
        }
    }
    None
}

// Walk the island contour from `from` toward `toward`. Used to reach a single
// lip hit, or to nudge the last endpoint a short way when this layer missed.
/// FillRadialZigZag.cpp:1314-1336 — `static Polyline cap_along_edge(const Point &from, const Point &toward, const ExPolygon &region, double max_len, bool reach_target)`
fn cap_along_edge(from: &Point, toward: &Point, region: &ExPolygon, max_len: f64, reach_target: bool) -> Polyline {
    let mut poly = poly_for_point(region, from);
    let mut dest = *toward;
    if poly.is_none() {
        poly = poly_for_point(region, toward);
    }
    let poly = match poly {
        Some(p) => p,
        None => return Polyline::new(),
    };
    let mut ds = 0usize;
    let mut dt = 0.0_f64;
    if !locate_on_polygon(poly, toward, &mut ds, &mut dt) {
        dest = poly.point_projection(toward);
    }
    let mut arc = shorter_boundary_arc(poly, from, &dest);
    if arc.points.len() < 2 {
        return Polyline::new();
    }
    if dist_pts(&arc.points[0], from) > scale_(0.05) {
        arc.append_before(*from);
    }
    let mut cap = trim_to_len(arc, max_len);
    if !reach_target && cap.points.len() >= 2 && dist_pts(&cap.points[cap.points.len() - 1], &dest) < scale_(0.4) {
        cap.clip_end(scale_(0.4));
    }
    if cap.points.len() >= 2 {
        cap
    } else {
        Polyline::new()
    }
}

/// FillRadialZigZag.cpp:1338-1348 — `static void prepend_cap(Polyline &dst, Polyline cap)`
fn prepend_cap(dst: &mut Polyline, mut cap: Polyline) {
    if cap.points.len() < 2 || dst.points.len() < 2 {
        return;
    }
    if dist_pts(&cap.points[cap.points.len() - 1], &dst.points[0]) > dist_pts(&cap.points[0], &dst.points[0]) {
        cap.reverse();
    }
    if cap.points.len() >= 2 && dist_pts(&cap.points[cap.points.len() - 1], &dst.points[0]) < scale_(0.15) {
        cap.points.pop();
    }
    cap.points.extend_from_slice(&dst.points);
    std::mem::swap(&mut dst.points, &mut cap.points);
}

/// FillRadialZigZag.cpp:1350-1361 — `static void append_cap(Polyline &dst, Polyline cap)`
fn append_cap(dst: &mut Polyline, mut cap: Polyline) {
    if cap.points.len() < 2 || dst.points.len() < 2 {
        return;
    }
    if dist_pts(&cap.points[0], &dst.points[dst.points.len() - 1]) > dist_pts(&cap.points[cap.points.len() - 1], &dst.points[dst.points.len() - 1]) {
        cap.reverse();
    }
    let mut i0 = 0usize;
    if dist_pts(&cap.points[0], &dst.points[dst.points.len() - 1]) < scale_(0.15) {
        i0 = 1;
    }
    for i in i0..cap.points.len() {
        dst.append_point(cap.points[i]);
    }
}

/// FillRadialZigZag.cpp:1363-1379 — `static void stitch_to_point(Polyline &dst, bool at_start, const Point &target, const ExPolygon &region, double max_len, bool reach_target)`
#[allow(dead_code)]
fn stitch_to_point(dst: &mut Polyline, at_start: bool, target: &Point, region: &ExPolygon, max_len: f64, reach_target: bool) {
    if dst.points.len() < 2 {
        return;
    }
    let from = if at_start { dst.points[0] } else { dst.points[dst.points.len() - 1] };
    if dist_pts(&from, target) < scale_(0.08) {
        return;
    }
    let mut cap = cap_along_edge(&from, target, region, max_len, reach_target);
    if cap.points.len() < 2 && dist_pts(&from, target) <= max_len {
        cap = Polyline::from_points(vec![from, *target]);
    }
    if at_start {
        prepend_cap(dst, cap);
    } else {
        append_cap(dst, cap);
    }
}

/// FillRadialZigZag.cpp:1381-1397 — `static Point nearest_poly_end(const Polylines &paths, const Point &ref)`
fn nearest_poly_end(paths: &[Polyline], ref_: &Point) -> Point {
    let mut best = *ref_;
    let mut best_d = f64::MAX;
    for pl in paths {
        if pl.points.len() < 2 {
            continue;
        }
        for p in [pl.points[0], pl.points[pl.points.len() - 1]] {
            let d = dist_pts(&p, ref_);
            if d < best_d {
                best_d = d;
                best = p;
            }
        }
    }
    best
}

/// FillRadialZigZag.cpp:1405-1409 — `struct Scan` of `connect_rays`.
#[derive(Debug, Clone, Copy)]
struct Scan {
    inner_pt: Point,
    outer_pt: Point,
    valid: bool,
}

impl Default for Scan {
    fn default() -> Self {
        Self { inner_pt: Point::new(0, 0), outer_pt: Point::new(0, 0), valid: false }
    }
}

/// FillRadialZigZag.cpp:1598-1602 — the lambda `toward_gap` of `cap_same_side`.
fn toward_gap(c: &Polyline, pole: &Point, gap_th: f64, d0: f64) -> bool {
    if c.points.len() < 2 {
        return false;
    }
    ang_diff(point_theta(&c.points[c.points.len() - 1], pole), gap_th) + 1e-6 < d0
}

/// FillRadialZigZag.cpp:1569-1623 — the lambda `cap_same_side` of `connect_rays`.
#[allow(clippy::too_many_arguments)]
fn cap_same_side(
    pl: &mut Polyline,
    at_start: bool,
    s: &Scan,
    region: &ExPolygon,
    pole: &Point,
    hub_cut_scaled: f64,
    spacing_mm: f64,
    gap_th: f64,
    near_mm: f64,
) {
    if !s.valid {
        return;
    }
    let p = if at_start { pl.points[0] } else { pl.points[pl.points.len() - 1] };
    #[allow(unused_assignments)]
    let mut want_outer = false;
    if dist_pts(&p, &s.outer_pt) < near_mm {
        want_outer = true;
    } else if dist_pts(&p, &s.inner_pt) < near_mm {
        want_outer = false;
    } else {
        return;
    }
    let mut poly = &region.contour;
    if !region.holes.is_empty() && !want_outer {
        if hub_cut_scaled > 1.0 {
            return;
        }
        poly = &region.holes[0];
    }
    let r0 = dist_pts(&p, pole);
    // Circle B bite on a C contour: do not rim-walk it (one-shot 大圆边).
    if on_hub_radius(&p, pole, hub_cut_scaled, ON_HUB_RADIUS_TOL_MM)
        || (hub_cut_scaled > 1.0 && (r0 - hub_cut_scaled).abs() < scale_(4.5))
    {
        return;
    }
    let r_in = dist_pts(&s.inner_pt, pole);
    let r_out = dist_pts(&s.outer_pt, pole);
    let band = scale_(0.5).max(0.18 * (r_out - r_in).abs());
    let cap_max = scale_(12.0 * spacing_mm).max(0.18 * poly.length());
    let fwd = rim_extend(poly, &p, true, pole, r0, band, cap_max);
    let bak = rim_extend(poly, &p, false, pole, r0, band, cap_max);
    let d0 = ang_diff(point_theta(&p, pole), gap_th);
    let mut cap = Polyline::new();
    let fu = toward_gap(&fwd, pole, gap_th, d0);
    let bu = toward_gap(&bak, pole, gap_th, d0);
    if fu && !bu {
        cap = fwd;
    } else if bu && !fu {
        cap = bak;
    } else if fu && bu {
        let df = ang_diff(point_theta(&fwd.points[fwd.points.len() - 1], pole), gap_th);
        let db = ang_diff(point_theta(&bak.points[bak.points.len() - 1], pole), gap_th);
        cap = if df <= db { fwd } else { bak };
    }
    if cap.points.len() < 2 {
        return;
    }
    if polyline_follows_hub(&cap, pole, hub_cut_scaled) {
        return;
    }
    if at_start {
        prepend_cap(pl, cap);
    } else {
        append_cap(pl, cap);
    }
}

/// FillRadialZigZag.cpp:1399-1643 — `static Polylines connect_rays(const std::vector<RadialRay> &rays, const ExPolygon &region, double spacing_mm, bool connect, int skew, bool closed, bool reverse_walk, const LipMem *prev_lips, LipMem &this_lips, bool stitch_ends, const Point &pole, double hub_cut_scaled = 0.)`
#[allow(clippy::too_many_arguments)]
fn connect_rays(
    rays: &[RadialRay],
    region: &ExPolygon,
    spacing_mm: f64,
    connect: bool,
    skew: i32,
    closed: bool,
    reverse_walk: bool,
    _prev_lips: Option<&LipMem>,
    this_lips: &mut LipMem,
    stitch_ends: bool,
    pole: &Point,
    hub_cut_scaled: f64,
) -> Vec<Polyline> {
    // FillRadialZigZag.cpp:1404-1426
    let n = rays.len() as i32;
    let mut scans: Vec<Scan> = Vec::with_capacity(n as usize);
    for i in 0..n {
        let mut s = Scan::default();
        if !rays[i as usize].valid {
            scans.push(s);
            continue;
        }
        let j = wrap_idx(i + skew, n, closed);
        if j < 0 || !rays[j as usize].valid {
            scans.push(s);
            continue;
        }
        if clip_scan_chord(&rays[i as usize].inner, &rays[j as usize].outer, region, &mut s.inner_pt, &mut s.outer_pt) {
            s.valid = true;
        }
        scans.push(s);
    }

    // FillRadialZigZag.cpp:1428-1440
    let mut slot_ok: Vec<u8> = vec![0; n as usize];
    for i in 0..n {
        slot_ok[i as usize] = if scans[i as usize].valid { 1 } else { 0 };
    }
    let (mut gap_a, mut gap_b) = (-1i32, -1i32);
    let mut gap_len = 0;
    let found_gap = longest_invalid_run(&slot_ok, &mut gap_a, &mut gap_b);
    if found_gap && gap_a >= 0 && gap_b >= 0 {
        gap_len = (gap_b - gap_a + n) % n + 1;
    }
    // A hole means this fill is an O-ring, not a C. One missed clip is not an
    // opening; treating it as one skips the last→first 铆线 (收尾未链接).
    let open_gap = region.holes.is_empty() && found_gap && gap_len >= 3;
    let lip_l = if open_gap { (gap_a + n - 1) % n } else { -1 };
    let lip_r = if open_gap { (gap_b + 1) % n } else { -1 };

    // FillRadialZigZag.cpp:1442-1453
    if !connect || scans.len() <= 1 {
        let mut out: Vec<Polyline> = Vec::with_capacity(scans.len());
        for s in &scans {
            if !s.valid {
                continue;
            }
            out.push(Polyline::from_points(vec![s.inner_pt, s.outer_pt]));
        }
        return out;
    }

    // FillRadialZigZag.cpp:1455-1456
    let fat = offset_ex(region, scale_(0.4) as f32);
    let max_len = scale_(8.0 * spacing_mm);

    // FillRadialZigZag.cpp:1458-1466
    let mut out: Vec<Polyline> = Vec::new();
    let mut current = Polyline::new();
    let mut ended_outer = false;
    let mut prev_i: i32 = -1;
    // Open C/U: start at the lip just after the opening so the +X seam
    // (k=n-1 → k=0) is hopped. Do not start at world +X — that splits the
    // body into two paths and drops the one 铆线 that should close the C.
    let i0 = if open_gap && lip_r >= 0 && scans[lip_r as usize].valid { lip_r } else { 0 };
    let start_at_lip = open_gap && lip_r >= 0 && i0 == lip_r;
    // FillRadialZigZag.cpp:1467-1508
    for step in 0..n {
        let i = (i0 + step) % n;
        if !scans[i as usize].valid {
            continue;
        }
        let cross_gap = open_gap && prev_i >= 0 && in_circular((prev_i + 1) % n, gap_a, gap_b, n);
        if cross_gap && start_at_lip {
            break;
        }
        let going_out = if reverse_walk { (i % 2) == 1 } else { (i % 2) == 0 };
        let a = if going_out { scans[i as usize].inner_pt } else { scans[i as usize].outer_pt };
        let b = if going_out { scans[i as usize].outer_pt } else { scans[i as usize].inner_pt };
        if current.points.is_empty() {
            current.append_point(a);
            current.append_point(b);
        } else if cross_gap {
            if current.points.len() >= 2 {
                out.push(std::mem::replace(&mut current, Polyline::new()));
            }
            current = Polyline::new();
            current.append_point(a);
            current.append_point(b);
        } else {
            let mut hop = Polyline::new();
            let back = current.points[current.points.len() - 1];
            let hop_outer = 0.5 * (dist_pts(&back, pole) + dist_pts(&a, pole))
                >= 0.5 * (dist_pts(&scans[i as usize].inner_pt, pole) + dist_pts(&scans[i as usize].outer_pt, pole));
            if !make_connector(&back, &a, hop_outer, region, &fat, max_len, pole, &mut hop, hub_cut_scaled)
                || polyline_has_radial_cut(&hop, pole)
            {
                if current.points.len() >= 2 {
                    out.push(std::mem::replace(&mut current, Polyline::new()));
                }
                current = Polyline::new();
                current.append_point(a);
                current.append_point(b);
            } else {
                for pi in 0..hop.points.len() {
                    current.append_point(hop.points[pi]);
                }
                current.append_point(b);
            }
        }
        ended_outer = going_out;
        prev_i = i;
    }
    // FillRadialZigZag.cpp:1509-1510
    if current.points.len() >= 2 {
        out.push(current);
    }

    // Close the ring seam (last scan → first). Previously this ran only when
    // every hop already succeeded (`out.empty()`). One failed mid-hop skipped
    // the last→first 铆线, so an O-ring looked unfinished at the join.
    // FillRadialZigZag.cpp:1515-1552
    if closed && !open_gap && !out.is_empty() && out[0].points.len() >= 2 && out[out.len() - 1].points.len() >= 2 {
        let a = out[out.len() - 1].points[out[out.len() - 1].points.len() - 1];
        let b = out[0].points[0];
        let loop_one = out.len() == 1;
        if dist_pts(&a, &b) < scale_(0.08) {
            if loop_one && out[0].points[0] != out[0].points[out[0].points.len() - 1] {
                let front = out[0].points[0];
                out[0].append_point(front);
            }
        } else {
            let mut toward_outer = true;
            if !region.holes.is_empty() {
                let da = dist_to_polygon(&region.holes[0], &a);
                let db = dist_to_polygon(&region.holes[0], &b);
                let oa = dist_to_polygon(&region.contour, &a);
                let ob = dist_to_polygon(&region.contour, &b);
                if da + db + 1.0 < oa + ob {
                    toward_outer = false;
                }
            } else {
                toward_outer = ended_outer;
            }
            let mut hop = Polyline::new();
            if make_connector(&a, &b, toward_outer, region, &fat, max_len, pole, &mut hop, hub_cut_scaled)
                && !polyline_has_radial_cut(&hop, pole)
            {
                if loop_one {
                    let pl = &mut out[0];
                    for k in 0..hop.points.len() {
                        pl.append_point(hop.points[k]);
                    }
                    let front = pl.points[0];
                    pl.append_point(front);
                } else {
                    let first_points = out[0].points.clone();
                    let last_idx = out.len() - 1;
                    let last = &mut out[last_idx];
                    for k in 0..hop.points.len() {
                        last.append_point(hop.points[k]);
                    }
                    for p in &first_points {
                        last.append_point(*p);
                    }
                    out.remove(0);
                }
            }
        }
    }

    // Open C/U: extend each path end along the SAME rim toward the opening,
    // and stop at the cut. Do not walk inner↔outer, and do not join the two lips.
    // FillRadialZigZag.cpp:1556-1641
    if stitch_ends && open_gap && lip_l >= 0 && lip_r >= 0 {
        let (mut sx, mut sy) = (0.0_f64, 0.0_f64);
        let mut gi = gap_a;
        for _k in 0..n {
            let th = rays[gi as usize].theta;
            sx += th.cos();
            sy += th.sin();
            if gi == gap_b {
                break;
            }
            gi = (gi + 1) % n;
        }
        let gap_th = sy.atan2(sx);
        let near_mm = scale_(1.5);
        // `L` and `R` (FillRadialZigZag.cpp:1624-1625) are never null here.
        let l = scans[lip_l as usize];
        let r = scans[lip_r as usize];
        for pl in out.iter_mut() {
            cap_same_side(pl, true, &l, region, pole, hub_cut_scaled, spacing_mm, gap_th, near_mm);
            cap_same_side(pl, false, &l, region, pole, hub_cut_scaled, spacing_mm, gap_th, near_mm);
            cap_same_side(pl, true, &r, region, pole, hub_cut_scaled, spacing_mm, gap_th, near_mm);
            cap_same_side(pl, false, &r, region, pole, hub_cut_scaled, spacing_mm, gap_th, near_mm);
        }
        this_lips.ok = l.valid || r.valid;
        if l.valid {
            this_lips.left = nearest_poly_end(&out, &l.outer_pt);
        }
        if r.valid {
            this_lips.right = nearest_poly_end(&out, &r.outer_pt);
        }
    }
    out
}

/// FillRadialZigZag.cpp:1645-1782 — `static RadialInfillResult generate(const ExPolygon &region, double spacing_mm, bool connect, double s0_mm, double hub_mm, int n_lock, double theta_lock, int skew, bool reverse_walk, bool has_pole_lock, const Point &pole_lock, bool equal_angle, double horiz_move, size_t region_id, size_t layer_id, const ExPolygon *clip_region, double hub_cut_mm = 0.)`
#[allow(clippy::too_many_arguments)]
fn generate(
    region: &ExPolygon,
    spacing_mm: f64,
    connect: bool,
    s0_mm: f64,
    hub_mm: f64,
    n_lock: i32,
    theta_lock: f64,
    skew: i32,
    reverse_walk: bool,
    has_pole_lock: bool,
    pole_lock: &Point,
    equal_angle: bool,
    horiz_move: f64,
    region_id: usize,
    layer_id: usize,
    clip_region: Option<&ExPolygon>,
    hub_cut_mm: f64,
) -> RadialInfillResult {
    // FillRadialZigZag.cpp:1651-1659
    let mut result = RadialInfillResult::default();
    if !is_supported_topology(region) {
        result.fallback_reason = "Unsupported topology".to_string();
        return result;
    }
    if spacing_mm < 1e-6 {
        result.fallback_reason = "Invalid spacing".to_string();
        return result;
    }

    // FillRadialZigZag.cpp:1661-1675
    let mut best_pole = Point::new(0, 0);
    let mut best_r = 0.0_f64;
    if hub_cut_mm > 0.05 {
        // Keep the hub center even when it sits in the punched hole; pick_pole
        // would otherwise drift into A−B and rays would cross circle B.
        best_pole = *pole_lock;
        best_r = 1.0;
    } else if !pick_pole(region, has_pole_lock, pole_lock, &mut best_pole, &mut best_r) {
        result.pole = best_pole;
        result.valid_ratio = best_r;
        result.fallback_reason = "No valid radial pole".to_string();
        return result;
    }
    result.pole = best_pole;
    result.valid_ratio = best_r;

    // FillRadialZigZag.cpp:1677-1679
    let hub_scaled = if region.contains(&best_pole, true) { scale_((0.4_f64).max(hub_mm)) } else { 0.0 };
    let h = scale_(spacing_mm);
    let fat_lip = offset_ex(region, scale_(1.0) as f32);

    // FillRadialZigZag.cpp:1681-1684
    #[allow(unused_assignments)]
    let mut n_scan = 0i32;
    let mut mid_closed = true;
    let mut final_rays: Vec<RadialRay> = Vec::new();
    let mut n_valid = 0i32;

    if equal_angle {
        // FillRadialZigZag.cpp:1687-1690
        n_scan = if n_lock >= 3 {
            n_lock
        } else {
            even_scan_count((estimate_mid_len(region) / h.max(1.0)).round() as i32)
        };
        n_scan = even_scan_count(n_scan);
        let dtheta = 2.0 * PI / std::cmp::max(1, n_scan) as f64;
        // CrossZag: δθ = horiz_move / r_mid. Use the pinned fan (n_scan · spacing),
        // not this slice's mid-curve. A C-shaped layer has no hole, so
        // estimate_mid_len returns ~½ of an annulus and used to jump δθ by ~2×.
        // FillRadialZigZag.cpp:1694-1699
        let mut dtheta0 = 0.0_f64;
        if horiz_move.abs() > 1.0 {
            let mid_lock = std::cmp::max(1, n_scan) as f64 * h;
            dtheta0 = horiz_move * 2.0 * PI / mid_lock;
        }
        let theta0 = theta_lock + (if s0_mm > 1e-9 { 0.5 * dtheta } else { 0.0 }) + dtheta0;
        // FillRadialZigZag.cpp:1700-1713
        final_rays.reserve(n_scan as usize);
        for k in 0..n_scan {
            // theta0 + double(k) * dtheta: fused.
            let theta = (k as f64).mul_add(dtheta, theta0);
            let mut ray = RadialRay::default();
            if !cast_ray(region, &best_pole, theta, hub_scaled, &mut ray) && !ray.lip {
                ray = RadialRay::default();
                ray.theta = theta;
                ray.valid = false;
                maybe_outset_lip(region, &fat_lip, &best_pole, hub_scaled, &mut ray);
            }
            if ray.valid {
                n_valid += 1;
            }
            final_rays.push(ray);
        }
    } else {
        // FillRadialZigZag.cpp:1715-1721
        let dense = dense_rays(region, &best_pole, hub_scaled, K_DENSE_RAYS);
        let mut samples: Vec<MidSample> = Vec::new();
        let mut mid_len = 0.0_f64;
        if !build_mid_samples(&dense, &mut samples, &mut mid_len, &mut mid_closed) {
            result.fallback_reason = "Mid-curve failed".to_string();
            return result;
        }

        // FillRadialZigZag.cpp:1723-1732
        n_scan = (mid_len / h.max(1.0)).round() as i32;
        if n_lock >= 3 {
            n_scan = n_lock;
        }
        n_scan = if mid_closed { even_scan_count(n_scan) } else { std::cmp::max(3, n_scan) };
        let step = mid_len / std::cmp::max(1, if mid_closed { n_scan } else { std::cmp::max(1, n_scan - 1) }) as f64;
        // World +X: k=0 is the scan that actually sits on the lock angle. Do not
        // rotate afterwards — nearest/CCW pick flickers by one slot when a sample
        // crosses the lock.
        let s_align = if mid_closed { s_for_theta(&samples, mid_len, true, theta_lock) } else { 0.0 };
        let s0 = s_align + scale_(s0_mm) + horiz_move;

        // FillRadialZigZag.cpp:1734-1747
        final_rays.reserve(n_scan as usize);
        for k in 0..n_scan {
            // s0 + double(k) * step: fused.
            let theta = sample_theta_at_s(&samples, mid_len, (k as f64).mul_add(step, s0), mid_closed);
            let mut ray = RadialRay::default();
            if !cast_ray(region, &best_pole, theta, hub_scaled, &mut ray) && !ray.lip {
                ray = RadialRay::default();
                ray.theta = theta;
                ray.valid = false;
                maybe_outset_lip(region, &fat_lip, &best_pole, hub_scaled, &mut ray);
            }
            if ray.valid {
                n_valid += 1;
            }
            final_rays.push(ray);
        }
    }

    // FillRadialZigZag.cpp:1750-1755
    result.n_scan = n_scan;
    result.n_lock = n_lock;
    if n_valid < 3 {
        result.fallback_reason = "Too few final rays".to_string();
        return result;
    }

    // FillRadialZigZag.cpp:1757-1770
    let prev_lips = recall_lips(region_id, layer_id, &best_pole);
    let mut this_lips = LipMem::default();
    this_lips.pole = best_pole;
    // Rays were cast on `region` (modifier island). Pairing and 铆线 follow
    // the final fill (`clip_region`), which already sits inside walls — do not
    // guess a wall inset or fatten the clip back into other parts.
    let link = match clip_region {
        Some(c) => c,
        None => region,
    };
    let paths = connect_rays(
        &final_rays,
        link,
        spacing_mm,
        connect,
        skew,
        mid_closed,
        reverse_walk,
        if prev_lips.ok { Some(&prev_lips) } else { None },
        &mut this_lips,
        true,
        &best_pole,
        scale_(hub_cut_mm),
    );
    if paths.is_empty() {
        result.fallback_reason = "Path generation failed".to_string();
        return result;
    }
    // Do not retry with stitch_ends off: C/U lip caps sit on the fill contour
    // and Clipper contains() often rejects the boundary, which used to strip
    // the 孤点 extensions.
    // FillRadialZigZag.cpp:1774-1781
    this_lips.pole = best_pole;
    store_lips(region_id, layer_id, this_lips);

    result.success = true;
    result.num_final_rays = n_valid;
    result.rays = final_rays;
    result.extrusion_paths = paths;
    result
}

/// FillRadialZigZag.cpp:1784-1828 — `static void record_diag(const Fill &fill, const RadialInfillResult &r, bool has_lock, const Point &pole_lock, const Point &layer_pole, bool axis_applied, int pole_mode, const ExPolygon &region)`
#[allow(clippy::too_many_arguments)]
fn record_diag(
    fill: &Fill,
    r: &RadialInfillResult,
    has_lock: bool,
    pole_lock: &Point,
    layer_pole: &Point,
    axis_applied: bool,
    pole_mode: i32,
    region: &ExPolygon,
) {
    // The C++ reads `g_dbg_on` without the mutex; it is read under it here.
    if !lock_g_dbg().on {
        return;
    }
    let mut d = LayerDiag::default();
    d.layer_id = layer_index(fill);
    d.success = r.success;
    d.pole_x = unscale_d(r.pole.x as f64);
    d.pole_y = unscale_d(r.pole.y as f64);
    d.has_lock = has_lock;
    if has_lock {
        d.lock_x = unscale_d(pole_lock.x as f64);
        d.lock_y = unscale_d(pole_lock.y as f64);
    }
    d.layer_x = unscale_d(layer_pole.x as f64);
    d.layer_y = unscale_d(layer_pole.y as f64);
    d.axis_applied = axis_applied;
    d.pole_inside = region.contains(&r.pole, true);
    d.drifted = has_lock && dist_pts(&r.pole, pole_lock) > scale_(0.4);
    d.pole_mode = pole_mode;
    d.n_holes = region.holes.len() as i32;
    d.island_mm = unscale_d(island_size(region));
    let mut n_in = 0;
    for ray in &r.rays {
        if !ray.valid {
            continue;
        }
        d.inner_mean += unscale_d(ray.inner_radius);
        d.outer_mean += unscale_d(ray.outer_radius);
        n_in += 1;
    }
    if n_in > 0 {
        d.inner_mean /= n_in as f64;
        d.outer_mean /= n_in as f64;
    }
    d.valid_ratio = r.valid_ratio;
    d.num_final_rays = r.num_final_rays;
    d.n_scan = r.n_scan;
    d.n_lock = r.n_lock;
    d.lock_rows = r.lock_rows;
    d.fallback_reason = r.fallback_reason.clone();
    d.rays = r.rays.clone();
    lock_g_dbg().log.push(d);
}

// FillRadialZigZag.cpp:1830 — } // namespace

/// FillRadialZigZag.cpp:1832-1836 — `void debug_enable(bool on)`
pub fn debug_enable(on: bool) {
    lock_g_dbg().on = on;
}

/// FillRadialZigZag.cpp:1838-1842 — `void debug_clear()`
pub fn debug_clear() {
    lock_g_dbg().log.clear();
}

/// FillRadialZigZag.cpp:1844-1848 — `void reset_n_lock()`
pub fn reset_n_lock() {
    lock_g_locks().clear();
}

/// FillRadialZigZag.cpp:1854-1859 — `struct Cand` of `pin_scan_counts`.
#[derive(Debug, Clone, Copy)]
struct Cand {
    key: Point,
    size: f64,
    #[allow(dead_code)]
    n_raw: i32,
    pole_raw: Point,
}

// Pin ray count, pole, and zigzag start phase before parallel make_fills.
// n is the even scan count of the largest island in this region; every layer
// of the same region uses that n. n_override > 0 forces that even count
// (at least 4) instead of the density-based estimate.
// Invalid rays keep their slot (no reindex).
// Zig-Zag walk starts at world +X (k=0). Pole holds for ≤1 mm jitter, else
// slews ≤0.5 mm/layer. If print_z is provided (one value per layer), also fit
// a 3D axis (hole/bbox poles) and a quadratic Bezier (bbox centers) for
// ConformalPole::Axis / Bezier. region_id keeps modifier locks separate from
// the parent remainder (default 0 for unit tests).
/// FillRadialZigZag.cpp:1850-1946 — `void pin_scan_counts(const std::vector<std::vector<ExPolygon>> &islands_by_layer, double spacing_mm, const std::vector<double> &print_z, size_t region_id, int n_override)`
pub fn pin_scan_counts(islands_by_layer: &[Vec<ExPolygon>], spacing_mm: f64, print_z: &[f64], region_id: usize, n_override: i32) {
    // FillRadialZigZag.cpp:1853-1864
    let h = scale_(spacing_mm.max(1e-6));
    let mut cands_by_layer: Vec<Vec<Cand>> = vec![Vec::new(); islands_by_layer.len()];
    let mut n_global = 4;
    let force_n = n_override > 0;
    if force_n {
        n_global = even_scan_count(n_override);
    }
    // FillRadialZigZag.cpp:1865-1879
    for i in 0..islands_by_layer.len() {
        let merged = union_ex(&islands_by_layer[i]);
        for ex in &merged {
            if ex.contour.points.len() < 3 {
                continue;
            }
            let l = estimate_mid_len(ex);
            let n_raw = even_scan_count((l / h.max(1.0)).round() as i32);
            if !force_n {
                n_global = std::cmp::max(n_global, n_raw);
            }
            cands_by_layer[i].push(Cand { key: get_extents(ex).center(), size: island_size(ex), n_raw, pole_raw: raw_pole(ex) });
        }
        // std::sort (libc++, not stable): the order of islands of equal size
        // depends on it, so the same sort is used.
        crate::clipper::libcxx_sort(&mut cands_by_layer[i], |a: &Cand, b: &Cand| a.size > b.size);
    }

    // FillRadialZigZag.cpp:1881-1929
    let mut table: Vec<Vec<LockedIslandN>> = vec![Vec::new(); islands_by_layer.len()];
    let mut prev: Vec<LockedIslandN> = Vec::new();
    let mut fit_z: Vec<f64> = Vec::new();
    let mut fit_p: Vec<Point> = Vec::new();
    let mut fit_bb: Vec<Point> = Vec::new();
    for i in 0..islands_by_layer.len() {
        let cands = &cands_by_layer[i];
        let mut used: Vec<u8> = vec![0; prev.len()];
        let mut cur: Vec<LockedIslandN> = Vec::with_capacity(cands.len());
        for ci in 0..cands.len() {
            let c = &cands[ci];
            let mut best: i32 = -1;
            let mut best_score = f64::MAX;
            // Primary island continues even if the AABB jumped or a satellite appeared.
            if ci == 0 && !prev.is_empty() {
                best = 0;
            } else {
                for j in 0..prev.len() {
                    if used[j] != 0 {
                        continue;
                    }
                    if size_mismatch(c.size, prev[j].size) > 3.0 {
                        continue;
                    }
                    let d = dist_pts(&c.key, &prev[j].key);
                    let lim = n_match_limit(c.size, prev[j].size);
                    if d > lim {
                        continue;
                    }
                    // size_mismatch(...) * scale_(100.) + d: fused.
                    let score = size_mismatch(c.size, prev[j].size).mul_add(scale_(100.0), d);
                    if score < best_score {
                        best_score = score;
                        best = j as i32;
                    }
                }
            }
            let mut pole = c.pole_raw;
            if best >= 0 {
                used[best as usize] = 1;
                pole = hold_or_slew_pole(&prev[best as usize].pole, &c.pole_raw);
            }
            cur.push(LockedIslandN { key: c.key, size: c.size, n: n_global, theta0: 0.0, pole });
        }
        if !cands.is_empty() && i < print_z.len() {
            fit_z.push(print_z[i]);
            fit_p.push(cands[0].pole_raw);
            fit_bb.push(cands[0].key);
        }
        table[i] = cur.clone();
        prev = cur;
    }
    // FillRadialZigZag.cpp:1930-1945
    let mut axis = AxisLine::default();
    let mut bezier = QuadBezier::default();
    if print_z.len() == islands_by_layer.len() {
        axis = fit_axis_line(&fit_z, &fit_p);
        bezier = fit_bbox_bezier(&fit_z, &fit_bb);
    }
    let rl = RegionLock {
        n_by_layer: table,
        n_global,
        axis,
        bezier,
        zs: fit_z,
        poles: fit_p,
        lips_by_layer: BTreeMap::new(),
        cast_by_layer: islands_by_layer.to_vec(),
    };
    lock_g_locks().insert(region_id, rl);
}

// Copy fitted poles/axis/Bezier out of the per-region lock table (call before reset_n_lock).
/// FillRadialZigZag.cpp:1948-1992 — `std::vector<PoleVis> snapshot_pole_vis()`
pub fn snapshot_pole_vis() -> Vec<PoleVis> {
    let g_locks = lock_g_locks();
    let mut out: Vec<PoleVis> = Vec::with_capacity(g_locks.len());
    // std::map iterates in key order, as BTreeMap does.
    for (k, rl) in g_locks.iter() {
        let mut vis = PoleVis::default();
        vis.region_id = *k;
        vis.axis_ok = rl.axis.ok;
        vis.bezier_ok = rl.bezier.ok;
        vis.layer_poles.reserve(rl.zs.len());
        let mut i = 0usize;
        while i < rl.zs.len() && i < rl.poles.len() {
            vis.layer_poles.push(Vec3d::new(unscale_d(rl.poles[i].x as f64), unscale_d(rl.poles[i].y as f64), rl.zs[i]));
            i += 1;
        }
        if rl.axis.ok {
            let mut z0 = if rl.zs.is_empty() { if rl.bezier.ok { rl.bezier.z0 } else { 0.0 } } else { rl.zs[0] };
            let mut z1 = if rl.zs.is_empty() { if rl.bezier.ok { rl.bezier.z1 } else { 1.0 } } else { rl.zs[rl.zs.len() - 1] };
            if z1 - z0 < 1e-6 {
                z0 -= 2.0;
                z1 += 2.0;
            }
            let n = std::cmp::max(2, rl.zs.len() as i32);
            vis.axis_polyline.reserve(n as usize);
            for i in 0..n {
                let t = if n == 1 { 0.0 } else { i as f64 / (n - 1) as f64 };
                // z0 + t * (z1 - z0), axis.x0 + axis.ax * z: fused.
                let z = t.mul_add(z1 - z0, z0);
                vis.axis_polyline.push(Vec3d::new(rl.axis.ax.mul_add(z, rl.axis.x0), rl.axis.ay.mul_add(z, rl.axis.y0), z));
            }
        }
        if rl.bezier.ok {
            let ns = 48;
            vis.bezier_polyline.reserve(ns as usize + 1);
            for i in 0..=ns {
                let t = i as f64 / ns as f64;
                let u = 1.0 - t;
                let x = bezier_eval(u, t, rl.bezier.p0x, rl.bezier.p1x, rl.bezier.p2x);
                let y = bezier_eval(u, t, rl.bezier.p0y, rl.bezier.p1y, rl.bezier.p2y);
                // rl.bezier.z0 + t * (rl.bezier.z1 - rl.bezier.z0): fused.
                let z = t.mul_add(rl.bezier.z1 - rl.bezier.z0, rl.bezier.z0);
                vis.bezier_polyline.push(Vec3d::new(x, y, z));
            }
        }
        out.push(vis);
    }
    out
}

/// FillRadialZigZag.cpp:1994-1998 — `std::vector<LayerDiag> debug_snapshot()`
pub fn debug_snapshot() -> Vec<LayerDiag> {
    lock_g_dbg().log.clone()
}

/// FillRadialZigZag.cpp:2000-2007 — `static bool link_flip_layer(size_t layer_id, int keep, int flip)`
fn link_flip_layer(layer_id: usize, mut keep: i32, flip: i32) -> bool {
    if flip <= 0 {
        return false;
    }
    keep = std::cmp::max(0, keep);
    let period = keep + flip;
    (layer_id % period as usize) as i32 >= keep
}

// FillRadialZigZag.cpp:2009-2011
const K_HUB_K: f64 = 1.2;
// Full line width on each side of the hub so extrusion beads do not sit on the A−B / A∩B cut.
const K_HUB_CLEARANCE: f64 = 1.0;

/// FillRadialZigZag.cpp:2013-2023 — `struct RadialBind`
///
/// `gen` and `hop` are pointers in the C++ (into the lock table or to the
/// region); they are owned copies here. `hop` is set exactly when the C++
/// pointer is not null.
#[derive(Debug, Clone)]
struct RadialBind {
    gen: Option<ExPolygon>,
    hop: Option<ExPolygon>,
    locked: LockedScan,
    n_lock: i32,
    pole: Point,
    layer_pole: Point,
    has_pole: bool,
    axis_applied: bool,
}

/// FillRadialZigZag.cpp:2025-2064 — `static RadialBind bind_radial(const ExPolygon &region, const Fill &fill, const FillParams &params)`
fn bind_radial(region: &ExPolygon, fill: &Fill, params: &FillParams) -> RadialBind {
    let mut b = RadialBind {
        gen: None,
        hop: None,
        locked: LockedScan::default(),
        n_lock: 0,
        pole: Point::new(0, 0),
        layer_pole: Point::new(0, 0),
        has_pole: false,
        axis_applied: false,
    };
    let region_id = fill.lock_region_id;
    let cast_island = lookup_cast_island(region_id, layer_index(fill), region);
    if let Some(ci) = cast_island {
        b.gen = Some(ci);
        b.hop = Some(region.clone());
    } else {
        b.gen = Some(region.clone());
    }
    let gen = b.gen.as_ref().expect("gen is set");
    b.locked = lookup_locked(layer_index(fill), &get_extents(gen).center(), island_size(gen), region_id);
    b.n_lock = if b.locked.ok { b.locked.n } else { 0 };
    {
        let g_locks = lock_g_locks();
        if b.n_lock < 3 {
            if let Some(rl) = find_lock(&g_locks, region_id) {
                b.n_lock = rl.n_global;
            }
        }
    }
    b.layer_pole = b.locked.pole;
    b.pole = b.locked.pole;
    b.has_pole = b.locked.has_pole;
    if params.conformal_pole == ConformalPole::Axis {
        let mut ap = Point::new(0, 0);
        if lookup_axis(fill.z, &mut ap, region_id) {
            b.pole = ap;
            b.has_pole = true;
            b.axis_applied = true;
        }
    } else if params.conformal_pole == ConformalPole::Bezier {
        let mut ap = Point::new(0, 0);
        if lookup_bezier(fill.z, &mut ap, region_id) {
            b.pole = ap;
            b.has_pole = true;
        }
    }
    b
}

/// FillRadialZigZag.cpp:2066-2074 — `static int hub_n_scan(const ExPolygon &region, const Fill &fill, const FillParams &params, int n_lock)`
fn hub_n_scan(region: &ExPolygon, fill: &Fill, params: &FillParams, n_lock: i32) -> i32 {
    if params.conformal_ray_count > 0 {
        return even_scan_count(params.conformal_ray_count);
    }
    if n_lock >= 3 {
        return even_scan_count(n_lock);
    }
    let h = scale_(line_spacing_mm(fill, params));
    even_scan_count((estimate_mid_len(region) / h.max(1.0)).round() as i32)
}

/// FillRadialZigZag.cpp:2076-2084 — `static double hub_line_width_mm(const Fill &fill, const FillParams &params)`
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

/// FillRadialZigZag.cpp:2086-2103 — `static ExPolygon max_overlap_piece(const ExPolygons &parts, const ExPolygon &key)`
fn max_overlap_piece(parts: &[ExPolygon], key: &ExPolygon) -> ExPolygon {
    if parts.len() == 1 {
        return parts[0].clone();
    }
    let mut best = &parts[0];
    let mut best_a = -1.0_f64;
    for p in parts {
        let hit = intersection_ex(p, key);
        let mut a = 0.0_f64;
        for h in &hit {
            a += h.area().abs();
        }
        if a > best_a {
            best_a = a;
            best = p;
        }
    }
    best.clone()
}

// Circle B around the layer pole. conformal_hub_radius: 0 = auto n*1.2*w/(2π),
// >0 = mm override, <0 = off. Returns false if hub is disabled or radius is tiny.
/// FillRadialZigZag.cpp:2105-2133 — `bool make_hub_disk(const ExPolygon &region, const Fill &fill, const FillParams &params, Point &pole, ExPolygon &disk)`
pub fn make_hub_disk(region: &ExPolygon, fill: &Fill, params: &FillParams, pole: &mut Point, disk: &mut ExPolygon) -> bool {
    if params.conformal_hub_radius < 0.0 {
        return false;
    }
    let b = bind_radial(region, fill, params);
    let gen = match b.gen.as_ref() {
        Some(g) => g,
        None => return false,
    };
    let mut pole_use = b.pole;
    if !b.has_pole {
        let mut br = 0.0_f64;
        if !pick_pole(gen, false, &Point::new(0, 0), &mut pole_use, &mut br) {
            return false;
        }
    }
    let n = hub_n_scan(gen, fill, params, b.n_lock);
    let w = hub_line_width_mm(fill, params);
    let r = if params.conformal_hub_radius > 0.0 {
        params.conformal_hub_radius as f64
    } else {
        n as f64 * K_HUB_K * w / (2.0 * PI)
    };
    if r < 0.05 {
        return false;
    }
    let mut c = make_circle_num_segments(scale_(r), 64);
    c.translate(pole_use);
    c.make_counter_clockwise();
    *disk = ExPolygon::new(c);
    *pole = pole_use;
    true
}

// Fill a single already-inset infill island. Returns false to let the caller fall back.
/// FillRadialZigZag.cpp:2135-2197 — `bool fill_region(const ExPolygon &region, const Fill &fill, const FillParams &params, Polylines &out)`
pub fn fill_region(region: &ExPolygon, fill: &Fill, params: &FillParams, out: &mut Vec<Polyline>) -> bool {
    // FillRadialZigZag.cpp:2137-2150
    let spacing_mm = line_spacing_mm(fill, params);
    let odd = (layer_index(fill) % 2) == 1;
    let alternate = params.conformal_stagger == ConformalStagger::Alternate || params.conformal_stagger == ConformalStagger::HalfStep;
    let reverse_walk = link_flip_layer(layer_index(fill), params.conformal_link_keep_layers, params.conformal_link_flip_layers);
    let s0_mm = 0.0_f64;
    let connect = !params.dont_connect();
    let skew = if alternate { if odd { -1 } else { 1 } } else { 0 };
    let b = bind_radial(region, fill, params);
    let theta_lock = 0.0_f64;
    let equal_angle = true;
    let n_lock = if params.conformal_ray_count > 0 { even_scan_count(params.conformal_ray_count) } else { b.n_lock };

    // FillRadialZigZag.cpp:2152-2156
    let gen = b.gen.as_ref().expect("bind_radial sets gen");
    let mut gen_parts: Vec<ExPolygon> = vec![gen.clone()];
    let mut hop_parts: Vec<ExPolygon> = Vec::new();
    let has_hop = b.hop.is_some();
    if let Some(hop) = b.hop.as_ref() {
        hop_parts.push(hop.clone());
    }

    // FillRadialZigZag.cpp:2158-2173
    let mut hub_pole = Point::new(0, 0);
    let mut disk = ExPolygon::empty();
    let mut hub_cut_mm = 0.0_f64;
    if make_hub_disk(gen, fill, params, &mut hub_pole, &mut disk) {
        let gap = K_HUB_CLEARANCE * hub_line_width_mm(fill, params);
        let r_mm = 0.5 * unscale_d(get_extents(&disk).size().x as f64);
        hub_cut_mm = r_mm + gap;
        let mut cut = offset_ex(&disk, scale_(gap) as f32);
        if cut.is_empty() {
            cut.push(disk.clone());
        }
        gen_parts = diff_ex(gen, &cut);
        if gen_parts.is_empty() {
            return false;
        }
        if let Some(hop) = b.hop.as_ref() {
            hop_parts = diff_ex(hop, &cut);
        }
    }

    // FillRadialZigZag.cpp:2175-2196
    let mut any = false;
    for gp in &gen_parts {
        let mut hp: Option<&ExPolygon> = None;
        let hp_store: ExPolygon;
        if has_hop && !hop_parts.is_empty() {
            hp_store = max_overlap_piece(&hop_parts, gp);
            hp = Some(&hp_store);
        }
        let pin_pole = hub_cut_mm > 0.05;
        let mut r = generate(
            gp,
            spacing_mm,
            connect,
            s0_mm,
            fill.spacing,
            n_lock,
            theta_lock,
            skew,
            reverse_walk,
            pin_pole || b.has_pole,
            if pin_pole { &hub_pole } else { &b.pole },
            equal_angle,
            if params.pattern == InfillPattern::CrossZag { params.horiz_move as f64 } else { 0.0 },
            fill.lock_region_id,
            layer_index(fill),
            hp,
            hub_cut_mm,
        );
        r.n_lock = n_lock;
        r.lock_rows = b.locked.table_rows;
        record_diag(fill, &r, b.has_pole, &b.pole, &b.layer_pole, b.axis_applied, params.conformal_pole as i32, gp);
        if r.success && !r.extrusion_paths.is_empty() {
            out.append(&mut r.extrusion_paths);
            any = true;
        }
    }
    any
}

// FillRadialZigZag.cpp:2199-2200 — } // namespace FillRadialZigZag } // namespace Slic3r

/// The lock table and the debug log are process globals, as in the C++; the
/// tests that touch them (here and in `fill_conformal`) take this lock.
#[cfg(test)]
pub(crate) static TEST_GLOBALS: Mutex<()> = Mutex::new(());

#[cfg(test)]
mod tests {
    use super::*;

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

    fn circle(r: f64, n: usize, cx: f64, cy: f64) -> Polygon {
        let mut c = make_circle_num_segments(scale_(r), n);
        c.translate(Point::new(mm(cx), mm(cy)));
        c
    }

    fn ring(r_out: f64, r_in: f64, cx: f64, cy: f64) -> ExPolygon {
        let mut hole = circle(r_in, 64, cx, cy);
        hole.points.reverse();
        ExPolygon::with_holes(circle(r_out, 96, cx, cy), vec![hole])
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

    fn test_fill(layer_id: usize, region_id: usize) -> Fill {
        let mut f = Fill::new();
        f.layer_id = layer_id;
        f.lock_region_id = region_id;
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

    /// Every vertex and every segment midpoint of the paths lies in the region grown by `tol_mm`.
    fn assert_inside(paths: &[Polyline], region: &ExPolygon, tol_mm: f64) {
        let fat = offset_ex(region, scale_(tol_mm) as f32);
        assert!(!fat.is_empty());
        let inside = |p: &Point| fat.iter().any(|ex| ex.contains(p, true));
        for pl in paths {
            assert!(pl.points.len() >= 2);
            for i in 0..pl.points.len() {
                assert!(inside(&pl.points[i]), "vertex {:?} outside", pl.points[i]);
                if i > 0 {
                    let m = avg_pt(&pl.points[i - 1], &pl.points[i]);
                    assert!(inside(&m), "midpoint {:?} outside", m);
                }
            }
        }
    }

    fn globals() -> std::sync::MutexGuard<'static, ()> {
        let g = TEST_GLOBALS.lock().unwrap_or_else(|e| e.into_inner());
        reset_n_lock();
        debug_enable(false);
        debug_clear();
        g
    }

    #[test]
    fn polygon_centroid_rounds_like_the_pin() {
        // (1.5, 1.5): lrint gives 2 (ties to even), a truncating cast gives 1.
        assert_eq!(polygon_centroid(&Polygon::from_points(vec![Point::new(0, 0), Point::new(3, 0), Point::new(3, 3), Point::new(0, 3)])), Point::new(2, 2));
        // (0.5, 0.5): ties to even.
        assert_eq!(polygon_centroid(&Polygon::from_points(vec![Point::new(0, 0), Point::new(1, 0), Point::new(1, 1), Point::new(0, 1)])), Point::new(0, 0));
        assert_eq!(polygon_centroid(&rect(-10.0, -4.0, 30.0, 8.0)), Point::new(mm(10.0), mm(2.0)));
    }

    #[test]
    fn small_integer_helpers() {
        assert_eq!(even_scan_count(0), 4);
        assert_eq!(even_scan_count(4), 4);
        assert_eq!(even_scan_count(5), 6);
        assert_eq!(even_scan_count(17), 18);
        assert_eq!(wrap_idx(-1, 8, true), 7);
        assert_eq!(wrap_idx(8, 8, true), 0);
        assert_eq!(wrap_idx(-1, 8, false), -1);
        assert_eq!(wrap_idx(8, 8, false), -1);
        assert_eq!(wrap_idx(3, 8, false), 3);
        assert_eq!(wrap_idx(3, 0, true), -1);
        assert!(in_circular(3, 2, 5, 8));
        assert!(!in_circular(6, 2, 5, 8));
        assert!(in_circular(7, 6, 1, 8));
        assert!(in_circular(0, 6, 1, 8));
        assert!(!in_circular(3, 6, 1, 8));
        // keep 2, flip 1: layers 0,1 forward, 2 reversed, 3,4 forward, 5 reversed.
        let flips: Vec<bool> = (0..6).map(|l| link_flip_layer(l, 2, 1)).collect();
        assert_eq!(flips, vec![false, false, true, false, false, true]);
        assert!(!link_flip_layer(7, 1, 0));
        assert!(link_flip_layer(0, 0, 1));
    }

    #[test]
    fn longest_invalid_run_finds_the_gap() {
        let (mut a, mut b) = (0, 0);
        assert!(longest_invalid_run(&[1, 1, 0, 0, 0, 1, 0, 1], &mut a, &mut b));
        assert_eq!((a, b), (2, 4));
        // The run wraps the end.
        assert!(longest_invalid_run(&[0, 1, 1, 1, 0, 0], &mut a, &mut b));
        assert_eq!((a, b), (4, 0));
        assert!(!longest_invalid_run(&[1, 1, 1, 1], &mut a, &mut b));
        assert_eq!((a, b), (-1, -1));
        assert!(!longest_invalid_run(&[0, 1], &mut a, &mut b));
    }

    #[test]
    fn angle_helpers() {
        assert_eq!(ang_diff(0.25, 0.75), 0.5);
        assert!((ang_diff(0.1, 2.0 * PI - 0.1) - 0.2).abs() < 1e-12);
        assert_eq!(wrap_theta(0.5), 0.5);
        assert!((wrap_theta(-0.5) - (2.0 * PI - 0.5)).abs() < 1e-15);
        assert!((wrap_theta(2.0 * PI + 0.5) - 0.5).abs() < 1e-15);
        assert!((ccw_from(0.5, 0.25) - (2.0 * PI - 0.25)).abs() < 1e-15);
        assert_eq!(ccw_from(0.25, 0.5), 0.25);
        assert_eq!(point_theta(&Point::new(0, 500), &Point::new(0, 0)), PI / 2.0);
    }

    #[test]
    fn point_helpers() {
        let a = Point::new(0, 0);
        let b = Point::new(10, -10);
        assert_eq!(lerp_pt(&a, &b, 0.25), Point::new(3, -3)); // lround(2.5) = 3, lround(-2.5) = -3
        assert_eq!(avg_pt(&a, &b), Point::new(5, -5));
        assert_eq!(dist_pts(&Point::new(0, 0), &Point::new(300, 400)), 500.0);
        assert_eq!(along_dir(&Point::new(100, 100), (1.0, 0.0), 50.4), Point::new(150, 100));
        assert_eq!(size_mismatch(2.0, 8.0), 4.0);
        assert_eq!(size_mismatch(0.0, 0.5), 0.5); // max(a, b) / max(1, min(a, b))
        // max(scale_(12), 0.45 * max)
        assert_eq!(n_match_limit(1.0, 2.0), scale_(12.0));
        assert_eq!(n_match_limit(1e8, 2.0), 0.45 * 1e8);
        // Hold within 1 mm, slew 0.5 mm beyond.
        let prev = Point::new(0, 0);
        assert_eq!(hold_or_slew_pole(&prev, &Point::new(mm(0.9), 0)), prev);
        let slewed = hold_or_slew_pole(&prev, &Point::new(mm(4.0), 0));
        assert!((slewed.x - mm(0.5)).abs() <= 1 && slewed.y == 0, "{slewed:?}");
    }

    #[test]
    fn solve3_and_fits() {
        let a = [[2.0, 0.0, 0.0], [0.0, 4.0, 0.0], [1.0, 0.0, 8.0]];
        let mut x = [0.0; 3];
        assert!(solve3(&a, &[2.0, 8.0, 17.0], &mut x));
        assert_eq!(x, [1.0, 2.0, 2.0]);
        assert!(!solve3(&[[1.0, 2.0, 3.0], [2.0, 4.0, 6.0], [0.0, 0.0, 1.0]], &[1.0, 2.0, 3.0], &mut x));

        // Poles on the line x = 1 + 0.5 z, y = -2 + 0.25 z.
        let zs: Vec<f64> = (0..8).map(|i| i as f64).collect();
        let poles: Vec<Point> = zs.iter().map(|z| Point::new(mm(1.0 + 0.5 * z), mm(-2.0 + 0.25 * z))).collect();
        let line = fit_axis_line(&zs, &poles);
        assert!(line.ok);
        assert!((line.x0 - 1.0).abs() < 1e-9 && (line.ax - 0.5).abs() < 1e-9);
        assert!((line.y0 + 2.0).abs() < 1e-9 && (line.ay - 0.25).abs() < 1e-9);
        assert!(!fit_axis_line(&zs[..1], &poles[..1]).ok);
        // All z equal: the mean.
        let flat = fit_axis_line(&[1.0, 1.0], &[Point::new(mm(1.0), 0), Point::new(mm(3.0), 0)]);
        assert!(flat.ok && (flat.x0 - 2.0).abs() < 1e-12 && flat.ax == 0.0);

        // Centres on a parabola in z: the quadratic Bezier goes through them.
        let bbs: Vec<Point> = zs.iter().map(|z| Point::new(mm(0.1 * z * z), mm(z * 1.0))).collect();
        let q = fit_bbox_bezier(&zs, &bbs);
        assert!(q.ok);
        assert_eq!((q.z0, q.z1), (0.0, 7.0));
        for (i, z) in zs.iter().enumerate() {
            let t = (z - q.z0) / (q.z1 - q.z0);
            let x = bezier_eval(1.0 - t, t, q.p0x, q.p1x, q.p2x);
            let y = bezier_eval(1.0 - t, t, q.p0y, q.p1y, q.p2y);
            assert!((x - unscale_d(bbs[i].x as f64)).abs() < 1e-6, "x {x}");
            assert!((y - unscale_d(bbs[i].y as f64)).abs() < 1e-6, "y {y}");
        }
        // Two layers: the Bezier of the fitted line.
        let q2 = fit_bbox_bezier(&zs[..2], &poles[..2]);
        assert!(q2.ok);
        assert!((q2.p1x - 0.5 * (q2.p0x + q2.p2x)).abs() < 1e-12);
    }

    #[test]
    fn polygon_walks() {
        let sq = rect(0.0, 0.0, 10.0, 10.0);
        let (mut s, mut t) = (0usize, 0.0_f64);
        assert!(locate_on_polygon(&sq, &Point::new(mm(4.0), 0), &mut s, &mut t));
        assert_eq!(s, 0);
        assert!((t - 0.4).abs() < 1e-12);
        assert!(!locate_on_polygon(&sq, &Point::new(mm(5.0), mm(5.0)), &mut s, &mut t)); // 5 mm away
        assert_eq!(dist_to_polygon(&sq, &Point::new(mm(4.0), mm(1.0))), mm(1.0) as f64);
        assert_eq!(dist_to_polygon(&sq, &Point::new(mm(5.0), mm(5.0))), f64::MAX);

        // From the middle of the bottom edge to the middle of the right edge.
        let fwd = walk_forward(&sq.points, 0, 0.5, 1, 0.5);
        assert_eq!(fwd.points, vec![Point::new(mm(5.0), 0), Point::new(mm(10.0), 0), Point::new(mm(10.0), mm(5.0))]);
        let bak = walk_backward(&sq.points, 0, 0.5, 1, 0.5);
        assert_eq!(
            bak.points,
            vec![
                Point::new(mm(5.0), 0),
                Point::new(0, 0),
                Point::new(0, mm(10.0)),
                Point::new(mm(10.0), mm(10.0)),
                Point::new(mm(10.0), mm(5.0)),
            ]
        );
        let arc = shorter_boundary_arc(&sq, &Point::new(mm(5.0), 0), &Point::new(mm(10.0), mm(5.0)));
        assert_eq!(arc.points, fwd.points);
        assert_eq!(arc.length(), mm(10.0) as f64);
        // Same segment, forward.
        let same = walk_forward(&sq.points, 0, 0.2, 0, 0.6);
        assert_eq!(same.points, vec![Point::new(mm(2.0), 0), Point::new(mm(6.0), 0)]);

        let trimmed = trim_to_len(fwd.clone(), mm(7.0) as f64);
        assert!((trimmed.length() - mm(7.0) as f64).abs() < 2.0);
        assert_eq!(trim_to_len(fwd.clone(), mm(20.0) as f64).points, fwd.points);

        // A corner walk has no radial cut about the centre; a spoke does.
        let centre = Point::new(mm(5.0), mm(5.0));
        assert!(!polyline_has_radial_cut(&fwd, &centre));
        let spoke = Polyline::from_points(vec![Point::new(mm(6.0), mm(5.0)), Point::new(mm(10.0), mm(5.0))]);
        assert!(polyline_has_radial_cut(&spoke, &centre));
        assert!(step_is_radial_cut(&spoke.points[0], &spoke.points[1], &centre));
        assert!(!step_is_radial_cut(&Point::new(mm(10.0), mm(4.0)), &Point::new(mm(10.0), mm(6.0)), &centre));
        assert_eq!(radius_mad_len(&spoke, &centre, mm(3.0) as f64), 0.0);
    }

    #[test]
    fn caps_are_joined_to_the_path_ends() {
        let mut dst = Polyline::from_points(vec![Point::new(0, 0), Point::new(mm(5.0), 0)]);
        let cap = Polyline::from_points(vec![Point::new(mm(5.0), 0), Point::new(mm(5.0), mm(2.0))]);
        append_cap(&mut dst, cap);
        assert_eq!(dst.points, vec![Point::new(0, 0), Point::new(mm(5.0), 0), Point::new(mm(5.0), mm(2.0))]);
        // Given the wrong way round: reversed, and its joint vertex dropped.
        let cap = Polyline::from_points(vec![Point::new(0, 0), Point::new(0, mm(-2.0))]);
        prepend_cap(&mut dst, cap);
        assert_eq!(dst.points[0], Point::new(0, mm(-2.0)));
        assert_eq!(dst.points[1], Point::new(0, 0));
        assert_eq!(dst.points.len(), 4);
        assert_eq!(
            nearest_poly_end(std::slice::from_ref(&dst), &Point::new(mm(5.0), mm(3.0))),
            Point::new(mm(5.0), mm(2.0))
        );
    }

    #[test]
    fn cast_ray_in_a_ring_and_in_a_solid() {
        let region = ring(10.0, 5.0, 0.0, 0.0);
        let pole = Point::new(0, 0);
        let mut ray = RadialRay::default();
        // 45 degrees: between vertices of neither polygon in particular.
        assert!(cast_ray(&region, &pole, 0.3, 0.0, &mut ray));
        assert!(ray.valid && !ray.lip);
        // The polygons are inscribed: the hit radii are at most the circle radii.
        assert!(ray.inner_radius <= scale_(5.0) + 2.0 && ray.inner_radius > scale_(4.98));
        assert!(ray.outer_radius <= scale_(10.0) + 2.0 && ray.outer_radius > scale_(9.98));
        assert_eq!(ray.mid, avg_pt(&ray.inner, &ray.outer));
        assert_eq!(classify_ray(&region, &pole, 0.3), RayClass::One);
        assert_eq!(evaluate_pole(&region, &pole, K_TEST_RAYS), 1.0);

        // Pole in solid: the scan starts at the hub radius.
        let solid = ExPolygon::new(rect(-10.0, -10.0, 10.0, 10.0));
        assert!(cast_ray(&solid, &pole, 0.0, scale_(2.0), &mut ray));
        assert_eq!(ray.inner, Point::new(scaled_like(2.0), 0));
        assert_eq!(ray.outer, Point::new(mm(10.0), 0));
        // A pole outside sees two intervals through the ring's hole.
        let far = Point::new(mm(-30.0), 0);
        assert_eq!(classify_ray(&region, &far, 0.0), RayClass::Multi);
        assert_eq!(classify_ray(&region, &far, PI), RayClass::Miss);
        assert!(!cast_ray(&region, &far, 0.0, 0.0, &mut ray));
    }

    /// `along_dir(pole, dir, scale_(v))` for a pole at the origin and dir = +X.
    fn scaled_like(v: f64) -> Coord {
        scale_(v).round() as Coord
    }

    #[test]
    fn pick_pole_takes_the_hole_centre_of_a_collar() {
        let region = ring(10.0, 5.0, 3.0, -2.0);
        let mut pole = Point::new(0, 0);
        let mut r = 0.0;
        assert!(pick_pole(&region, false, &Point::new(0, 0), &mut pole, &mut r));
        assert!(r >= K_VALID_THRESHOLD);
        assert!(dist_pts(&pole, &Point::new(mm(3.0), mm(-2.0))) < scale_(0.05), "{pole:?}");
        let rp = raw_pole(&region);
        assert!(dist_pts(&rp, &Point::new(mm(3.0), mm(-2.0))) < scale_(0.01));
        assert!(is_supported_topology(&region));
        assert!((estimate_mid_len(&region) - 0.5 * (region.contour.length() + region.holes[0].length())).abs() < 1e-6);
        let mut two = region.clone();
        two.holes.push(two.holes[0].clone());
        assert!(!is_supported_topology(&two));
        assert_eq!(generate(&two, 2.0, true, 0.0, 0.45, 0, 0.0, 0, false, false, &pole, true, 0.0, 99, 0, None, 0.0).fallback_reason, "Unsupported topology");
        assert_eq!(generate(&region, 0.0, true, 0.0, 0.45, 0, 0.0, 0, false, false, &pole, true, 0.0, 99, 0, None, 0.0).fallback_reason, "Invalid spacing");
    }

    #[test]
    fn mid_samples_of_a_ring_close() {
        let region = ring(10.0, 5.0, 0.0, 0.0);
        let dense = dense_rays(&region, &Point::new(0, 0), 0.0, K_DENSE_RAYS);
        assert_eq!(dense.len(), K_DENSE_RAYS as usize);
        let mut samples = Vec::new();
        let (mut len, mut closed) = (0.0, false);
        assert!(build_mid_samples(&dense, &mut samples, &mut len, &mut closed));
        assert!(closed);
        // The mid circle has radius 7.5 mm.
        assert!((len - 2.0 * PI * scale_(7.5)).abs() < scale_(0.1), "len {len}");
        let th = sample_theta_at_s(&samples, len, 0.25 * len, true);
        assert!((th - PI / 2.0).abs() < 0.01, "theta {th}");
        let s = s_for_theta(&samples, len, true, PI);
        assert!((s - 0.5 * len).abs() < 0.005 * len, "s {s}");
        assert!(!build_mid_samples(&dense[..4], &mut samples, &mut len, &mut closed));
    }

    #[test]
    fn fill_region_of_a_ring_is_one_closed_zigzag() {
        let _g = globals();
        let region = ring(10.0, 5.0, 0.0, 0.0);
        let fill = test_fill(0, 0);
        let mut params = test_params(0.2);
        params.conformal_hub_radius = -1.0; // no hub: the pole is in the hole anyway
        params.conformal_ray_count = 24;
        debug_enable(true);
        let mut out = Vec::new();
        assert!(fill_region(&region, &fill, &params, &mut out));
        let diag = debug_snapshot();
        debug_enable(false);
        debug_clear();
        assert_eq!(diag.len(), 1);
        let d = &diag[0];
        assert!(d.success && !d.pole_inside && d.n_holes == 1);
        assert_eq!((d.n_scan, d.n_lock, d.num_final_rays), (24, 24, 24));
        // Equal angles, the first on world +X.
        for (k, ray) in d.rays.iter().enumerate() {
            assert!(ray.valid);
            assert_eq!(ray.theta, (k as f64).mul_add(2.0 * PI / 24.0, 0.0));
        }
        assert!((d.inner_mean - 5.0).abs() < 0.02 && (d.outer_mean - 10.0).abs() < 0.02);
        // One path through all 24 scans, closed.
        assert_eq!(out.len(), 1);
        let pl = &out[0];
        assert_eq!(pl.points[0], pl.points[pl.points.len() - 1]);
        assert!(pl.points.len() >= 48);
        assert_inside(&out, &region, 0.45);
        // The radial part alone is 24 scans of 5 mm; the hops follow the rims.
        assert!(pl.length() > 24.0 * scale_(4.9));
        // Deterministic.
        let mut again = Vec::new();
        assert!(fill_region(&region, &fill, &params, &mut again));
        assert_eq!(again.len(), 1);
        assert_eq!(again[0].points, out[0].points);
    }

    #[test]
    fn fill_region_density_sets_the_ray_count() {
        let _g = globals();
        let region = ring(10.0, 5.0, 0.0, 0.0);
        let fill = test_fill(0, 0);
        debug_enable(true);
        for (density, _) in [(0.2, 0), (0.4, 0)] {
            let mut params = test_params(density);
            params.conformal_hub_radius = -1.0;
            let mut out = Vec::new();
            assert!(fill_region(&region, &fill, &params, &mut out));
            assert_inside(&out, &region, 0.45);
        }
        let diag = debug_snapshot();
        debug_enable(false);
        debug_clear();
        assert_eq!(diag.len(), 2);
        // n = even(lround(mid_len / (spacing / density))), mid_len = half of both rims.
        for (d, density) in diag.iter().zip([0.2_f32, 0.4_f32]) {
            let h = scale_(0.45 / density as f64);
            let want = even_scan_count((estimate_mid_len(&region) / h).round() as i32);
            assert_eq!(d.n_scan, want);
            assert_eq!(d.num_final_rays, want);
        }
        assert!(diag[1].n_scan > diag[0].n_scan);
    }

    #[test]
    fn fill_region_unconnected_gives_one_segment_per_ray() {
        let _g = globals();
        let region = ring(10.0, 5.0, 0.0, 0.0);
        let fill = test_fill(0, 0);
        let mut params = test_params(0.2);
        params.conformal_hub_radius = -1.0;
        params.conformal_ray_count = 16;
        params.anchor_length_max = 0.0; // dont_connect()
        let mut out = Vec::new();
        assert!(fill_region(&region, &fill, &params, &mut out));
        assert_eq!(out.len(), 16);
        for (k, pl) in out.iter().enumerate() {
            assert_eq!(pl.points.len(), 2);
            let th = point_theta(&pl.points[1], &Point::new(0, 0));
            assert!(ang_diff(th, k as f64 * 2.0 * PI / 16.0) < 1e-3, "ray {k}: {th}");
            assert!((pl.length() - scale_(5.0)).abs() < scale_(0.05));
        }
        assert_inside(&out, &region, 0.1);
    }

    #[test]
    fn fill_region_alternate_stagger_skews_the_scans() {
        let _g = globals();
        let region = ring(10.0, 5.0, 0.0, 0.0);
        let mut params = test_params(0.2);
        params.conformal_hub_radius = -1.0;
        params.conformal_ray_count = 16;
        params.anchor_length_max = 0.0;
        params.conformal_stagger = ConformalStagger::Alternate;
        let mut even = Vec::new();
        assert!(fill_region(&region, &test_fill(0, 0), &params, &mut even));
        let mut odd = Vec::new();
        assert!(fill_region(&region, &test_fill(1, 0), &params, &mut odd));
        assert_eq!((even.len(), odd.len()), (16, 16));
        // Inner end on ray k, outer end on ray k + 1 (even layers) or k - 1 (odd layers).
        let step = 2.0 * PI / 16.0;
        let o = Point::new(0, 0);
        assert!(ang_diff(point_theta(&even[0].points[0], &o), 0.0) < 1e-3);
        assert!(ang_diff(point_theta(&even[0].points[1], &o), step) < 1e-3);
        assert!(ang_diff(point_theta(&odd[0].points[0], &o), 0.0) < 1e-3);
        assert!(ang_diff(point_theta(&odd[0].points[1], &o), -step) < 1e-3);
        assert_inside(&even, &region, 0.1);
        assert_inside(&odd, &region, 0.1);
    }

    #[test]
    fn fill_region_of_a_square_cuts_the_hub() {
        let _g = globals();
        let region = ExPolygon::new(rect(-10.0, -10.0, 10.0, 10.0));
        let fill = test_fill(0, 0);
        let params = test_params(0.2);
        let mut pole = Point::new(1, 1);
        let mut disk = ExPolygon::empty();
        assert!(make_hub_disk(&region, &fill, &params, &mut pole, &mut disk));
        assert_eq!(pole, Point::new(0, 0));
        assert_eq!(disk.contour.points.len(), 64);
        // r = n * 1.2 * w / (2 pi) with n = even(lround(40 mm / 2.25 mm)) = 18, w = 0.4.
        let r = 18.0 * 1.2 * 0.4 / (2.0 * PI);
        assert!((unscale_d(get_extents(&disk).size().x as f64) - 2.0 * r).abs() < 1e-3);

        let mut out = Vec::new();
        assert!(fill_region(&region, &fill, &params, &mut out));
        assert!(!out.is_empty());
        assert_inside(&out, &region, 0.45);
        // Nothing inside circle B grown by one line width.
        let keep_out = scale_(r + 0.4) - scale_(0.05);
        for pl in &out {
            for p in &pl.points {
                assert!(dist_pts(p, &pole) >= keep_out, "{p:?} in the hub");
            }
        }
        let mut again = Vec::new();
        assert!(fill_region(&region, &fill, &params, &mut again));
        assert_eq!(again.len(), out.len());
        for (a, b) in again.iter().zip(out.iter()) {
            assert_eq!(a.points, b.points);
        }

        // Hub off: rays start 0.45 mm from the pole (hub_mm = spacing).
        let mut off = params.clone();
        off.conformal_hub_radius = -1.0;
        assert!(!make_hub_disk(&region, &fill, &off, &mut pole, &mut disk));
        let mut out2 = Vec::new();
        assert!(fill_region(&region, &fill, &off, &mut out2));
        assert_inside(&out2, &region, 0.45);
        // A forced radius.
        let mut forced = params.clone();
        forced.conformal_hub_radius = 3.0;
        assert!(make_hub_disk(&region, &fill, &forced, &mut pole, &mut disk));
        assert!((unscale_d(get_extents(&disk).size().x as f64) - 6.0).abs() < 1e-3);
    }

    #[test]
    fn fill_region_of_an_l_shape_and_a_thin_strip() {
        let _g = globals();
        for (name, region) in [("L", l_shape()), ("strip", ExPolygon::new(rect(0.0, 0.0, 40.0, 3.0)))] {
            for density in [0.15, 0.4] {
                let fill = test_fill(2, 0);
                let params = test_params(density);
                let mut out = Vec::new();
                let ok = fill_region(&region, &fill, &params, &mut out);
                if !ok {
                    assert!(out.is_empty(), "{name}: paths without success");
                    continue;
                }
                assert!(!out.is_empty(), "{name}");
                assert_inside(&out, &region, 0.45);
                let mut again = Vec::new();
                assert!(fill_region(&region, &fill, &params, &mut again));
                assert_eq!(again.len(), out.len(), "{name}");
                for (a, b) in again.iter().zip(out.iter()) {
                    assert_eq!(a.points, b.points, "{name}");
                }
            }
        }
    }

    #[test]
    fn pin_scan_counts_locks_n_and_the_pole() {
        let _g = globals();
        // Ten layers of a collar that drifts 0.2 mm per layer in x.
        let islands: Vec<Vec<ExPolygon>> = (0..10).map(|i| vec![ring(10.0, 5.0, 0.2 * i as f64, 0.0)]).collect();
        let zs: Vec<f64> = (0..10).map(|i| 0.2 * (i + 1) as f64).collect();
        pin_scan_counts(&islands, 2.25, &zs, 0, 0);

        let h = scale_(2.25);
        let n_want = islands
            .iter()
            .map(|row| even_scan_count((estimate_mid_len(&row[0]) / h).round() as i32))
            .max()
            .unwrap();
        for i in 0..10 {
            let key = get_extents(&islands[i][0]).center();
            let locked = lookup_locked(i, &key, island_size(&islands[i][0]), 0);
            assert!(locked.ok && locked.has_pole);
            assert_eq!(locked.n, n_want);
            assert_eq!((locked.table_rows, locked.row_islands), (10, 1));
            // Within the 1 mm hold of the first pole for the first layers, then slewing.
            assert!(dist_pts(&locked.pole, &raw_pole(&islands[i][0])) <= scale_(1.0) + 2.0, "layer {i}");
        }
        assert_eq!(lookup_locked(0, &Point::new(0, 0), 1.0, 0).pole, lookup_locked(4, &Point::new(0, 0), 1.0, 0).pole);
        assert!(!lookup_locked(10, &Point::new(0, 0), 1.0, 0).ok);
        // Another region falls back to region 0 (find_lock).
        assert!(lookup_locked(3, &Point::new(0, 0), 1.0, 7).ok);

        // The axis through the hole centres: x = z - 0.2 (0.2 mm per 0.2 mm layer), y = 0.
        let mut p = Point::new(0, 0);
        assert!(lookup_axis(1.0, &mut p, 0));
        assert!((p.x - mm(0.8)).abs() < 200 && p.y.abs() < 200, "{p:?}");
        assert!(lookup_bezier(1.0, &mut p, 0));
        assert!((p.x - mm(0.8)).abs() < 2000 && p.y.abs() < 2000, "{p:?}");

        let vis = snapshot_pole_vis();
        assert_eq!(vis.len(), 1);
        assert!(vis[0].axis_ok && vis[0].bezier_ok);
        assert_eq!(vis[0].layer_poles.len(), 10);
        assert_eq!(vis[0].axis_polyline.len(), 10);
        assert_eq!(vis[0].bezier_polyline.len(), 49);
        assert_eq!(vis[0].axis_polyline[0].z, 0.2);

        // A forced count.
        pin_scan_counts(&islands, 2.25, &zs, 3, 7);
        assert_eq!(lookup_locked(0, &Point::new(0, 0), 1.0, 3).n, 8);
        assert_eq!(snapshot_pole_vis().len(), 2);

        // The cast island of the table is where the rays are cast from.
        let clip = ring(9.0, 6.0, 0.0, 0.0);
        let cast = lookup_cast_island(0, 0, &clip).expect("cast island");
        assert_eq!(cast.contour.points, islands[0][0].contour.points);
        assert!(lookup_cast_island(0, 10, &clip).is_none());

        // Lips are remembered per layer and recalled from the layer below.
        store_lips(0, 4, LipMem { pole: Point::new(0, 0), left: Point::new(1, 1), right: Point::new(2, 2), ok: true });
        assert!(recall_lips(0, 5, &Point::new(mm(1.0), 0)).ok);
        assert!(!recall_lips(0, 5, &Point::new(mm(20.0), 0)).ok);
        assert!(!recall_lips(0, 4, &Point::new(0, 0)).ok);

        reset_n_lock();
        assert!(!lookup_locked(0, &Point::new(0, 0), 1.0, 0).ok);
        assert!(snapshot_pole_vis().is_empty());
    }

    #[test]
    fn pinned_layers_share_the_ray_count_and_the_axis_pole() {
        let _g = globals();
        let islands: Vec<Vec<ExPolygon>> = (0..6).map(|i| vec![ring(10.0 + i as f64, 5.0, 0.0, 0.0)]).collect();
        let zs: Vec<f64> = (0..6).map(|i| 0.2 * (i + 1) as f64).collect();
        pin_scan_counts(&islands, 2.25, &zs, 0, 0);
        debug_enable(true);
        let mut params = test_params(0.2);
        params.conformal_hub_radius = -1.0;
        params.conformal_pole = ConformalPole::Axis;
        for i in 0..6 {
            let mut out = Vec::new();
            assert!(fill_region(&islands[i][0], &test_fill(i, 0), &params, &mut out));
            assert_inside(&out, &islands[i][0], 0.45);
        }
        let diag = debug_snapshot();
        debug_enable(false);
        debug_clear();
        assert_eq!(diag.len(), 6);
        // n of the largest island on every layer.
        let n = diag[5].n_scan;
        for d in &diag {
            assert!(d.success && d.has_lock && d.axis_applied);
            assert_eq!(d.n_scan, n);
            assert_eq!(d.lock_rows, 6);
            assert!(d.pole_x.abs() < 0.01 && d.pole_y.abs() < 0.01);
        }
    }

    #[test]
    fn connect_rays_leaves_an_opening_open() {
        let _g = globals();
        // A C: the ring with a 4 mm slot cut through its +X side.
        let c = diff_ex(&ring(10.0, 5.0, 0.0, 0.0), &[ExPolygon::new(rect(0.0, -2.0, 12.0, 2.0))]);
        assert_eq!(c.len(), 1);
        let region = &c[0];
        assert!(region.holes.is_empty());
        let fill = test_fill(0, 0);
        let mut params = test_params(0.2);
        params.conformal_hub_radius = -1.0;
        params.conformal_ray_count = 36;
        let mut out = Vec::new();
        let ok = fill_region(region, &fill, &params, &mut out);
        assert!(ok, "the C is filled from the centre of its box");
        assert_inside(&out, region, 0.45);
        // No path crosses the slot: no point strictly inside it.
        for pl in &out {
            for p in &pl.points {
                assert!(!(p.x > mm(5.5) && p.y.abs() < mm(1.5)), "{p:?} in the opening");
            }
        }
    }
}
