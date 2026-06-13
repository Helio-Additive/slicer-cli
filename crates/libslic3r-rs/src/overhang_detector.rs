//! Faithful 1:1 port of `src/libslic3r/OverhangDetector.{hpp,cpp}` (BambuStudio).
//!
//! C++ Reference:
//! - OverhangDetector.hpp
//! - OverhangDetector.cpp
//!
//! coord_t -> i64, coordf_t -> f64. See per-line `// OverhangDetector.cpp:NNN`
//! and `// OverhangDetector.hpp:NNN` references.
//!
//! BLOCKED symbols (see module-level NOTE): `clip_extrusion` and the Arachne
//! `detect_overhang_degree(Flow, role, lower_polys, clip_paths, extrusion_path,
//! nozzle_diameter)` overload both depend on the legacy `ClipperLib_Z::Clipper`
//! engine (custom per-edge `ZFillFunction` + `PolyTree` + `Execute` +
//! `PolyTreeToPaths`). The crate's only clipping backend is Clipper2 via
//! `clipper2c-sys`, which is compiled with `CLIPPER2_USINGZ=OFF` (verified in
//! the vendored `clipper2c/CMakeLists.txt`) and the safe `clipper2` 0.5 wrapper
//! exposes no `usingz`/`zCallback` feature at all — i.e. there is genuinely no
//! Z-aware boolean clip with a user fill callback available, and reproducing one
//! byte-exactly would mean porting a whole Vatti/Clipper engine. Same blocker as
//! documented in `line_segmentation.rs` and `fill/fill_floating_concentric.rs`.
//!
//! NOTE (2026-06-13): the Arachne `extrusion_paths_append(std::list, ZPaths,
//! role, Flow, overhang)` helper *is* now ported as
//! `arachne::utils::extrusion_line::extrusion_paths_append_list` (along with
//! `to_thick_polyline_z` / `thick_polyline_to_multi_path`). The Arachne overload
//! is therefore blocked SOLELY on `clip_extrusion` (its first statement consumes
//! the Z-clip result); everything downstream of that is portable. We do not
//! emit a body that calls a faked `clip_extrusion`. These two symbols stay NOT
//! PORTED and are documented below.

use crate::aabb_tree_lines::{
    build_aabb_tree_over_indexed_lines, squared_distance_to_indexed_lines, tree2d, LinesDistancer,
};
// OverhangDetector.hpp:9-11 — using ZPoint = ClipperLib_Z::IntPoint; ZPath; ZPaths;
use crate::clipper_z_utils::{ZPath, ZPaths};
use crate::extrusion_entity::{ExtrusionPath, ExtrusionRole};

// C++: using ExtrusionPaths = std::vector<ExtrusionPath>; (ExtrusionEntity.hpp)
pub type ExtrusionPaths = Vec<ExtrusionPath>;
use crate::geometry::{Line, Point, PointF, Polygon, Polyline, Polylines};

// ---------------------------------------------------------------------------
// File-scope constants (OverhangDetector.hpp:13-19).
//
// NOTE on `scale_`: the C++ macro is `#define scale_(val) ((val) / SCALING_FACTOR)`
// with `SCALING_FACTOR = 0.00001`, i.e. `scale_(val)` evaluates to a *double*
// equal to `val * 100000.0` (no rounding/truncation; the result is a `coordf_t`).
// We therefore replicate it with a float multiply, NOT crate::scale (which
// rounds & returns i64). `cut_length` keeps its C++ `double` type.
// ---------------------------------------------------------------------------

/// OverhangDetector.cpp `scale_` macro, double-valued (val / SCALING_FACTOR).
#[inline]
fn scale_(val: f64) -> f64 {
    // libslic3r.h:58,81 — SCALING_FACTOR = 0.00001; scale_(val) = (val)/SCALING_FACTOR
    val / 0.00001
}

// OverhangDetector.hpp:13
pub const OVERHANG_SAMPLING_NUMBER: i32 = 6;
// OverhangDetector.hpp:14
pub const MIN_DEGREE_GAP_CLASSIC: f64 = 0.1;
// OverhangDetector.hpp:15
pub const MIN_DEGREE_GAP_ARACHNE: f64 = 0.25;
// OverhangDetector.hpp:16
pub const MAX_OVERHANG_DEGREE: i32 = OVERHANG_SAMPLING_NUMBER - 1;
// OverhangDetector.hpp:17
// static const std::vector<double> non_uniform_degree_map = { 0, 10, 25, 50, 75, 100};
pub const NON_UNIFORM_DEGREE_MAP: [f64; 6] = [0.0, 10.0, 25.0, 50.0, 75.0, 100.0];
// OverhangDetector.hpp:18
pub const INSERT_POINT_COUNT: i32 = 3;

// OverhangDetector.hpp:19 — static const double cut_length = scale_(0.6);
#[inline]
fn cut_length() -> f64 {
    scale_(0.6)
}

// EPSILON from libslic3r.
use crate::libslic3r::EPSILON;

/// Faithful `Point operator*(const Point&, const double&)` (Point.hpp:255-258).
///
/// C++: `inline Point operator*(const Point& l, const double& r) { return {
/// coord_t(l.x() * r), coord_t(l.y() * r) }; }` — `coord_t(double)` is a *cast*
/// (truncation toward zero), NOT rounding. The crate's `Mul<CoordF> for Point`
/// operator uses `.round()` instead, which diverges from C++ here. Every `Point
/// * double` in this file (`dir * t`, `front() + dir * (...)`, `pa + (pb-pa)*t`)
/// must truncate to match byte-exact G-code, so we use this helper instead of
/// the crate operator. (Divergence corrected — see module NOTE.)
#[inline]
fn point_mul_f64(l: Point, r: f64) -> Point {
    // Point.hpp:257 — { coord_t(l.x() * r), coord_t(l.y() * r) }
    Point::new((l.x() as f64 * r) as i64, (l.y() as f64 * r) as i64)
}

// ---------------------------------------------------------------------------
// OverhangDetector.hpp:23-30 — class OverhangDistancer
// ---------------------------------------------------------------------------

/// OverhangDetector.hpp:23
///
/// C++ stores `std::vector<Linef>` + `AABBTreeIndirect::Tree<2, double>`. The
/// Rust `build_aabb_tree_over_indexed_lines` / `squared_distance_to_indexed_lines`
/// pair operate on integer `Line` whose coordinates already equal the scaled
/// polygon coordinates and are internally cast to `f64`, so the numeric result
/// is identical to building over `Linef`.
pub struct OverhangDistancer {
    // OverhangDetector.hpp:25
    lines: Vec<Line>,
    // OverhangDetector.hpp:26
    tree: tree2d::Tree,
}

impl OverhangDistancer {
    // OverhangDetector.cpp:148-154
    pub fn new(layer_polygons: &[Polygon]) -> Self {
        // OverhangDetector.cpp:149 — ctor body
        let mut lines: Vec<Line> = Vec::new();
        // OverhangDetector.cpp:150 — for (const Polygon& island : layer_polygons)
        for island in layer_polygons {
            // OverhangDetector.cpp:151 — for (const auto& line : island.lines())
            //   lines.emplace_back(line.a.cast<double>(), line.b.cast<double>());
            for line in island.edges() {
                lines.push(Line::new(line.a, line.b));
            }
        }
        // OverhangDetector.cpp:153 — tree = build_aabb_tree_over_indexed_lines(lines);
        let tree = build_aabb_tree_over_indexed_lines(&lines);
        Self { lines, tree }
    }

    // OverhangDetector.cpp:156-166
    pub fn distance_from_perimeter(&self, point: PointF) -> f32 {
        // OverhangDetector.cpp:158 — Vec2d p = point.cast<double>();
        let p = point;
        // OverhangDetector.cpp:159 — size_t hit_idx_out{};
        let mut hit_idx_out: usize = 0;
        // OverhangDetector.cpp:160 — Vec2d hit_point_out = Vec2d::Zero();
        let mut hit_point_out = PointF::zero();
        // OverhangDetector.cpp:161 — auto distance = squared_distance_to_indexed_lines(...)
        let mut distance = squared_distance_to_indexed_lines(
            &self.lines,
            &self.tree,
            p,
            &mut hit_idx_out,
            &mut hit_point_out,
            f64::INFINITY,
        );
        // OverhangDetector.cpp:162 — if (distance < 0) return std::numeric_limits<float>::max();
        if distance < 0.0 {
            return f32::MAX;
        }
        // OverhangDetector.cpp:164 — distance = sqrt(distance);
        distance = distance.sqrt();
        // OverhangDetector.cpp:165 — return distance;
        distance as f32
    }
}

// ---------------------------------------------------------------------------
// OverhangDetector.hpp:32-40 / .cpp:319-336 — class SignedOverhangDistancer
// ---------------------------------------------------------------------------

/// OverhangDetector.hpp:32
///
/// NOTE (divergence): C++ uses `AABBTreeLines::LinesDistancer<Linef>` (double
/// coordinates) so `distance_from_perimeter(const Vec2d&)` can query a
/// fractional-coordinate point. The Rust `LinesDistancer` is specialized to the
/// integer `Line`/`Point` type, so the query point is rounded to integer coords
/// (sub-unit precision is lost). This is only reachable from the *blocked*
/// Arachne `detect_overhang_degree` overload, so it does not affect any
/// currently-portable code path.
pub struct SignedOverhangDistancer {
    // OverhangDetector.hpp:34
    distancer: LinesDistancer,
}

impl SignedOverhangDistancer {
    // OverhangDetector.cpp:319-326
    pub fn new(layer_polygons: &[Polygon]) -> Self {
        // OverhangDetector.cpp:321 — std::vector<Linef> lines;
        let mut lines: Vec<Line> = Vec::new();
        // OverhangDetector.cpp:322 — for (const Polygon &island : layer_polygons)
        for island in layer_polygons {
            // OverhangDetector.cpp:323 — for (const auto &line : island.lines())
            //   lines.emplace_back(line.a.cast<double>(), line.b.cast<double>());
            for line in island.edges() {
                lines.push(Line::new(line.a, line.b));
            }
        }
        // OverhangDetector.cpp:325 — distancer = AABBTreeLines::LinesDistancer<Linef>(lines);
        let distancer = LinesDistancer::new(lines);
        Self { distancer }
    }

    // OverhangDetector.cpp:328-331
    pub fn distance_from_perimeter(&self, point: PointF) -> f64 {
        // OverhangDetector.cpp:330 — return distancer.distance_from_lines<true>(point);
        // (divergence: integer-point query, see struct NOTE)
        let p = Point::new(point.x.round() as i64, point.y.round() as i64);
        self.distancer.distance_from_lines::<true>(p)
    }

    // OverhangDetector.cpp:333-336
    pub fn distance_from_perimeter_extra(&self, point: PointF) -> (f32, usize, PointF) {
        // OverhangDetector.cpp:335 — return distancer.distance_from_lines_extra<true>(point);
        let p = Point::new(point.x.round() as i64, point.y.round() as i64);
        let (dist, idx, np) = self.distancer.distance_from_lines_extra::<true>(p);
        (dist as f32, idx, np)
    }
}

// ---------------------------------------------------------------------------
// OverhangDetector.hpp:42-48 — struct SplitPoly
// ---------------------------------------------------------------------------

/// OverhangDetector.hpp:42
#[derive(Debug, Clone)]
pub struct SplitPoly {
    // OverhangDetector.hpp:46
    pub polyline: Polyline,
    // OverhangDetector.hpp:47
    pub degree: f64,
}

impl SplitPoly {
    // OverhangDetector.hpp:44 — SplitPoly(Polyline polyline) : polyline(polyline) {}
    pub fn new(polyline: Polyline) -> Self {
        Self {
            polyline,
            degree: 0.0,
        }
    }

    // OverhangDetector.hpp:45 — SplitPoly(Polyline polyline, double degree)
    pub fn with_degree(polyline: Polyline, degree: f64) -> Self {
        Self { polyline, degree }
    }
}

// ---------------------------------------------------------------------------
// OverhangDetector.hpp:50-100 — struct SplitLines
// ---------------------------------------------------------------------------

/// OverhangDetector.hpp:50
#[derive(Debug, Clone, Default)]
pub struct SplitLines {
    // OverhangDetector.hpp:97
    pub start: Vec<SplitPoly>,
    // OverhangDetector.hpp:98
    pub end: Vec<SplitPoly>,
    // OverhangDetector.hpp:99
    pub middle: Vec<SplitPoly>,
}

impl SplitLines {
    // OverhangDetector.hpp:53-95 — SplitLines(Polyline polyline, bool upsampling)
    pub fn new(polyline: Polyline, upsampling: bool) -> Self {
        let mut start: Vec<SplitPoly> = Vec::new();
        let mut end: Vec<SplitPoly> = Vec::new();
        let mut middle: Vec<SplitPoly> = Vec::new();

        // OverhangDetector.hpp:55 — double length = polyline.length();
        let length = polyline.length();
        // OverhangDetector.hpp:56-59 — if (length < 2 * cut_length) { middle.push_back(polyline); return; }
        if length < 2.0 * cut_length() {
            middle.push(SplitPoly::new(polyline));
            return Self {
                start,
                end,
                middle,
            };
        }

        // OverhangDetector.hpp:62 — int sampling_number = upsampling ? insert_point_count : 2;
        let sampling_number = if upsampling { INSERT_POINT_COUNT } else { 2 };
        // OverhangDetector.hpp:63 — int cut_count = std::min(int(length / cut_length), sampling_number);
        let mut cut_count = std::cmp::min((length / cut_length()) as i32, sampling_number);
        // OverhangDetector.hpp:64 — double final_cut_length = std::min(polyline.length()/cut_count, cut_length);
        let final_cut_length = (polyline.length() / cut_count as f64).min(cut_length());
        // OverhangDetector.hpp:65 — Point dir = polyline.back() - polyline.front();
        let dir = polyline.last_point() - polyline.first_point();
        // OverhangDetector.hpp:67 — cut_count = cut_count / 2;
        cut_count /= 2;

        // OverhangDetector.hpp:68-84 — lambda cut_polyline(base_length, first_point, last_point, &out)
        let cut_polyline =
            |base_length: f32, first_point: Point, last_point: Point, out: &mut Vec<SplitPoly>| {
                // OverhangDetector.hpp:69 — Point start = first_point;
                let mut seg_start = first_point;
                // OverhangDetector.hpp:70 — Point end;
                let mut seg_end: Point;
                // OverhangDetector.hpp:71 — for (size_t cnt = 0; cnt < cut_count - 1; cnt++)
                let mut cnt: i64 = 0;
                while cnt < (cut_count as i64) - 1 {
                    // OverhangDetector.hpp:72 — Polyline line;
                    let mut line = Polyline::new();
                    // OverhangDetector.hpp:74 — line.append(start);
                    line.push(seg_start);
                    // OverhangDetector.hpp:76 — double t = ((cnt+1)*cut_length + base_length) / length;
                    let t = ((cnt + 1) as f64 * cut_length() + base_length as f64) / length;
                    // OverhangDetector.hpp:77 — end = first_point + dir * t;
                    // (Point*double truncates — point_mul_f64, see NOTE.)
                    seg_end = first_point + point_mul_f64(dir, t);
                    // OverhangDetector.hpp:78 — line.append(end);
                    line.push(seg_end);
                    // OverhangDetector.hpp:80 — out.emplace_back(SplitPoly(line));
                    out.push(SplitPoly::new(line));
                    // OverhangDetector.hpp:81 — start = end;
                    seg_start = seg_end;
                    cnt += 1;
                }
                // OverhangDetector.hpp:83 — out.emplace_back(SplitPoly(Polyline(start, last_point)));
                out.push(SplitPoly::new(Polyline::from_points(vec![
                    seg_start, last_point,
                ])));
            };

        // OverhangDetector.hpp:86 — double trim_length = final_cut_length * cut_count;
        let trim_length = final_cut_length * cut_count as f64;
        // OverhangDetector.hpp:88 — double middle_length = length - trim_length;
        let middle_length = length - trim_length;
        // OverhangDetector.hpp:89 — Point start_pt = polyline.front() + dir * (trim_length / length);
        // (Point*double truncates — point_mul_f64, see NOTE.)
        let start_pt = polyline.first_point() + point_mul_f64(dir, trim_length / length);
        // OverhangDetector.hpp:90 — Point end_pt = polyline.front() + dir * ((length - trim_length)/length);
        let end_pt = polyline.first_point() + point_mul_f64(dir, (length - trim_length) / length);
        // OverhangDetector.hpp:91 — middle.emplace_back(SplitPoly(Polyline(start_pt, end_pt)));
        middle.push(SplitPoly::new(Polyline::from_points(vec![start_pt, end_pt])));

        // OverhangDetector.hpp:93 — cut_polyline(0, polyline.front(), start_pt, start);
        cut_polyline(0.0, polyline.first_point(), start_pt, &mut start);
        // OverhangDetector.hpp:94 — cut_polyline(middle_length, end_pt, polyline.back(), end);
        cut_polyline(
            middle_length as f32,
            end_pt,
            polyline.last_point(),
            &mut end,
        );

        Self {
            start,
            end,
            middle,
        }
    }
}

// OverhangDetector.hpp:102 — using DegreePolylines = std::vector<SplitLines>;
pub type DegreePolylines = Vec<SplitLines>;

// ---------------------------------------------------------------------------
// OverhangDetector.cpp:8-16 — ZPath_to_polylines
// ---------------------------------------------------------------------------

// OverhangDetector.cpp:8-16
pub fn z_path_to_polylines(paths: &ZPaths) -> Polylines {
    // OverhangDetector.cpp:10 — Polylines lines;
    let mut lines: Polylines = Vec::new();
    // OverhangDetector.cpp:11 — for (auto& path : paths)
    for path in paths {
        // OverhangDetector.cpp:12 — lines.emplace_back();
        lines.push(Polyline::new());
        // OverhangDetector.cpp:13 — for (auto& p : path) lines.back().points.push_back(Point{ p.x(), p.y() });
        let back = lines.last_mut().unwrap();
        for p in path {
            back.points.push(Point::new(p.0, p.1));
        }
    }
    // OverhangDetector.cpp:15 — return lines;
    lines
}

// ---------------------------------------------------------------------------
// OverhangDetector.cpp:18-108 — clip_extrusion  [BLOCKED]
//
// NOTE (BLOCKED): requires the legacy `ClipperLib_Z::Clipper` with a custom
// `ZFillFunction`, `PolyTree`, `Execute(clipType, polytree, pftNonZero,
// pftNonZero)`, and `PolyTreeToPaths`. The crate's clipper backend is Clipper2
// (f64 / Centi scaling) and exposes no Z-aware clipper with a user fill callback
// nor a `PolyTree` traversal. Faithfully (byte-exactly) reproducing this needs a
// port of the bundled `clipper/clipper_z` engine, which is not yet available.
// NOT PORTED — see module header.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// OverhangDetector.cpp:110-140 — add_sampling_points (single ZPath)
// ---------------------------------------------------------------------------

// OverhangDetector.cpp:110-140
pub fn add_sampling_points(path: &ZPath, min_sampling_interval: f64) -> ZPath {
    // OverhangDetector.cpp:112 — ZPath sampled_path;
    let mut sampled_path: ZPath = Vec::new();
    // OverhangDetector.cpp:113-114 — if (path.empty()) return sampled_path;
    if path.is_empty() {
        return sampled_path;
    }
    // OverhangDetector.cpp:115 — sampled_path.reserve(1.5 * path.size());
    sampled_path.reserve((1.5 * path.len() as f64) as usize);
    // OverhangDetector.cpp:116 — for (size_t idx = 0; idx < path.size(); ++idx)
    for idx in 0..path.len() {
        // OverhangDetector.cpp:117 — ZPoint curr_zp = path[idx];
        let curr_zp = path[idx];
        // OverhangDetector.cpp:118 — Point curr_p = { curr_zp.x(), curr_zp.y() };
        let curr_p = Point::new(curr_zp.0, curr_zp.1);
        // OverhangDetector.cpp:119 — sampled_path.emplace_back(curr_zp);
        sampled_path.push(curr_zp);
        // OverhangDetector.cpp:120 — if (idx + 1 < path.size())
        if idx + 1 < path.len() {
            // OverhangDetector.cpp:121 — ZPoint next_zp = path[idx + 1];
            let next_zp = path[idx + 1];
            // OverhangDetector.cpp:122 — Point next_p = { next_zp.x(), next_zp.y() };
            let next_p = Point::new(next_zp.0, next_zp.1);

            // OverhangDetector.cpp:124 — double dist = (next_p - curr_p).cast<double>().norm();
            let dist = (next_p - curr_p).to_f64().length();
            // OverhangDetector.cpp:125 — if (dist > min_sampling_interval)
            if dist > min_sampling_interval {
                // OverhangDetector.cpp:126 — size_t num_samples = floor(dist / min_sampling_interval);
                let num_samples = (dist / min_sampling_interval).floor() as usize;
                // OverhangDetector.cpp:127 — for (size_t j = 1; j <= num_samples; ++j)
                for j in 1..=num_samples {
                    // OverhangDetector.cpp:128 — double t = j * min_sampling_interval / dist;
                    let t = j as f64 * min_sampling_interval / dist;
                    // OverhangDetector.cpp:129 — ZPoint new_point;
                    // OverhangDetector.cpp:130 — new_point.x() = curr_p.x() + t*(next_p.x()-curr_p.x());
                    let nx = (curr_p.x() as f64 + t * (next_p.x() - curr_p.x()) as f64) as i64;
                    // OverhangDetector.cpp:131 — new_point.y() = curr_p.y() + t*(next_p.y()-curr_p.y());
                    let ny = (curr_p.y() as f64 + t * (next_p.y() - curr_p.y()) as f64) as i64;
                    // OverhangDetector.cpp:132 — new_point.z() = curr_zp.z() + t*(next_zp.z()-curr_zp.z());
                    let nz = (curr_zp.2 as f64 + t * (next_zp.2 - curr_zp.2) as f64) as i64;
                    // OverhangDetector.cpp:133 — sampled_path.push_back(new_point);
                    sampled_path.push((nx, ny, nz));
                }
            }
        }
    }
    // OverhangDetector.cpp:138 — sampled_path.shrink_to_fit();
    sampled_path.shrink_to_fit();
    // OverhangDetector.cpp:139 — return sampled_path;
    sampled_path
}

// OverhangDetector.cpp:141-146 — add_sampling_points (ZPaths overload)
pub fn add_sampling_points_paths(paths: &ZPaths, min_sampling_interval: f64) -> ZPaths {
    // OverhangDetector.cpp:142-143 — ZPaths res; res.resize(paths.size());
    let mut res: ZPaths = vec![Vec::new(); paths.len()];
    // OverhangDetector.cpp:144 — for (...) res[i] = add_sampling_points(paths[i], min_sampling_interval);
    for i in 0..res.len() {
        res[i] = add_sampling_points(&paths[i], min_sampling_interval);
    }
    // OverhangDetector.cpp:145 — return res;
    res
}

// ---------------------------------------------------------------------------
// OverhangDetector.cpp:168-317 — detect_overhang_degree (Arachne/Flow overload)
//
// NOTE (BLOCKED): the body's FIRST statement is
// `ZPaths paths_in_range = clip_extrusion(extrusion_path, clip_paths,
// ClipperLib_Z::ctIntersection);` and the entire function operates on that
// clipped, Z-width-carrying result. `clip_extrusion` is BLOCKED (above) on the
// missing Z-aware Clipper engine, so this overload cannot be implemented without
// fabricating that result — which the porting rules forbid.
//
// Everything *downstream* of the clip IS now portable: the per-Flow
// `extrusion_paths_append(std::list<ExtrusionPath>&, ZPaths, role, Flow,
// overhang)` is ported as
// `arachne::utils::extrusion_line::extrusion_paths_append_list`, and
// `SignedOverhangDistancer` is implemented above (with the documented
// integer-query divergence). The sole remaining blocker is `clip_extrusion`.
// NOT PORTED — see module header.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// OverhangDetector.cpp:338-342 — get_base_degree (free fn, degree_trace arg)
// ---------------------------------------------------------------------------

// OverhangDetector.cpp:338-342
pub fn get_base_degree(d: f64, degree_trace: f64) -> f64 {
    // OverhangDetector.cpp:340 — double degee_base = int(d / degree_trace) * degree_trace;
    let degee_base = (d / degree_trace) as i32 as f64 * degree_trace;
    // OverhangDetector.cpp:341 — return degee_base >= max_overhang_degree ? max_overhang_degree : degee_base;
    if degee_base >= MAX_OVERHANG_DEGREE as f64 {
        MAX_OVERHANG_DEGREE as f64
    } else {
        degee_base
    }
}

// ---------------------------------------------------------------------------
// OverhangDetector.cpp:344-361 — get_mapped_degree
// ---------------------------------------------------------------------------

// OverhangDetector.cpp:344-361
pub fn get_mapped_degree(overhang_dist: f64, lower_bound: f64, upper_bound: f64) -> f64 {
    // BBS : calculate overhang degree -- overhang length / width
    // OverhangDetector.cpp:347 — double this_degree = (overhang_dist - lower_bound)/(upper_bound - lower_bound)*100;
    let this_degree = (overhang_dist - lower_bound) / (upper_bound - lower_bound) * 100.0;
    // BBS: covert to terraced overhang
    // OverhangDetector.cpp:349 — double terraced_overhang = 0;
    let mut terraced_overhang = 0.0;
    // OverhangDetector.cpp:350 — if (this_degree >= 100)
    if this_degree >= 100.0 {
        // OverhangDetector.cpp:351 — terraced_overhang = max_overhang_degree;
        terraced_overhang = MAX_OVERHANG_DEGREE as f64;
    }
    // OverhangDetector.cpp:352 — else if (this_degree > EPSILON * 100)
    else if this_degree > EPSILON * 100.0 {
        // OverhangDetector.cpp:353 — int upper_bound_idx = std::upper_bound(non_uniform_degree_map.begin(), end(), this_degree) - begin();
        let upper_bound_idx = upper_bound_index(&NON_UNIFORM_DEGREE_MAP, this_degree) as i32;
        // OverhangDetector.cpp:354 — int lower_bound_idx = upper_bound_idx - 1;
        let lower_bound_idx = upper_bound_idx - 1;

        // OverhangDetector.cpp:356 — double t = (this_degree - map[lower_bound_idx]) / (map[upper_bound_idx] - map[lower_bound_idx]);
        let t = (this_degree - NON_UNIFORM_DEGREE_MAP[lower_bound_idx as usize])
            / (NON_UNIFORM_DEGREE_MAP[upper_bound_idx as usize]
                - NON_UNIFORM_DEGREE_MAP[lower_bound_idx as usize]);
        // OverhangDetector.cpp:357 — terraced_overhang = (1.0 - t)*lower_bound_idx + t*upper_bound_idx;
        terraced_overhang = (1.0 - t) * lower_bound_idx as f64 + t * upper_bound_idx as f64;
    }

    // OverhangDetector.cpp:360 — return terraced_overhang;
    terraced_overhang
}

/// `std::upper_bound` over a sorted slice: index of first element strictly
/// greater than `value` (returns slice length when none).
#[inline]
fn upper_bound_index(arr: &[f64], value: f64) -> usize {
    // Mirrors std::upper_bound semantics used by get_mapped_degree /
    // detect_overhang_degree's mapping block.
    let mut lo = 0usize;
    let mut hi = arr.len();
    while lo < hi {
        let mid = lo + (hi - lo) / 2;
        if value < arr[mid] {
            hi = mid;
        } else {
            lo = mid + 1;
        }
    }
    lo
}

// ---------------------------------------------------------------------------
// OverhangDetector.cpp:363-385 — merged_with_degree
// ---------------------------------------------------------------------------

// OverhangDetector.cpp:363-385
pub fn merged_with_degree(in_: &mut Vec<SplitPoly>) {
    // OverhangDetector.cpp:364 — std::vector<SplitPoly> out;
    let mut out: Vec<SplitPoly> = Vec::new();

    // standardization
    // OverhangDetector.cpp:367 — Polyline merged_lines;
    let mut merged_lines = Polyline::new();
    // OverhangDetector.cpp:368 — double degree_base = -1;
    let mut degree_base: f64 = -1.0;
    // OverhangDetector.cpp:369 — for (size_t idx = 0; idx < in.size(); idx++)
    for idx in 0..in_.len() {
        // OverhangDetector.cpp:370 — double degree = get_base_degree(in[idx].degree, min_degree_gap_classic);
        let degree = get_base_degree(in_[idx].degree, MIN_DEGREE_GAP_CLASSIC);

        // OverhangDetector.cpp:372 — if (!merged_lines.empty() && degree_base != degree)
        if !merged_lines.is_empty() && degree_base != degree {
            // OverhangDetector.cpp:373 — out.emplace_back(SplitPoly(merged_lines, degree_base));
            out.push(SplitPoly::with_degree(merged_lines.clone(), degree_base));
            // OverhangDetector.cpp:374 — merged_lines.clear();
            merged_lines.clear();
        }
        // OverhangDetector.cpp:376 — degree_base = degree;
        degree_base = degree;
        // OverhangDetector.cpp:377 — merged_lines.append(in[idx].polyline);
        merged_lines.append(&in_[idx].polyline);
    }

    // OverhangDetector.cpp:380 — if (!merged_lines.empty())
    if !merged_lines.is_empty() {
        // OverhangDetector.cpp:381 — out.emplace_back(SplitPoly(merged_lines, degree_base));
        out.push(SplitPoly::with_degree(merged_lines, degree_base));
    }

    // OverhangDetector.cpp:384 — in = std::move(out);
    *in_ = out;
}

// ---------------------------------------------------------------------------
// OverhangDetector.cpp:387-423 — smoothing_degrees
// ---------------------------------------------------------------------------

// OverhangDetector.cpp:387-423
pub fn smoothing_degrees(lines: &mut SplitLines) {
    // too short
    // OverhangDetector.cpp:390-391 — if (lines.start.empty() || lines.middle.empty()) return;
    if lines.start.is_empty() || lines.middle.is_empty() {
        return;
    }

    // OverhangDetector.cpp:393 — double d1 = lines.start.back().degree;
    let d1 = lines.start.last().unwrap().degree;
    // OverhangDetector.cpp:394 — double d2 = lines.end.front().degree;
    let d2 = lines.end.first().unwrap().degree;

    // OverhangDetector.cpp:396 — if (lines.middle.front().polyline.length() < 2*cut_length || std::abs(d2-d1) < min_degree_gap_classic)
    if lines.middle[0].polyline.length() < 2.0 * cut_length()
        || (d2 - d1).abs() < MIN_DEGREE_GAP_CLASSIC
    {
        // OverhangDetector.cpp:397 — lines.middle.front().degree = (d2 + d1)/2;
        lines.middle[0].degree = (d2 + d1) / 2.0;
        // OverhangDetector.cpp:398 — return;
        return;
    }

    // OverhangDetector.cpp:401 — std::vector<SplitPoly> out;
    let mut out: Vec<SplitPoly> = Vec::new();
    //BBS: smoothing polyline by degree
    // compare cut length and degree
    // OverhangDetector.cpp:404 — double length = lines.middle.front().polyline.length();
    let length = lines.middle[0].polyline.length();
    // OverhangDetector.cpp:405 — int length_cut = length / cut_length;
    let length_cut = (length / cut_length()) as i32;
    // OverhangDetector.cpp:406 — int degree_cut = std::abs(d2 - d1) / min_degree_gap_classic / 0.6;
    let degree_cut = ((d2 - d1).abs() / MIN_DEGREE_GAP_CLASSIC / 0.6) as i32;
    // OverhangDetector.cpp:407 — int count = std::min(length_cut, degree_cut);
    let count = std::cmp::min(length_cut, degree_cut);
    // OverhangDetector.cpp:408 — double cut_gap = length / count;
    let cut_gap = length / count as f64;
    // OverhangDetector.cpp:409 — double degree_gap = (d2 - d1) / count;
    let degree_gap = (d2 - d1) / count as f64;
    //cut
    // OverhangDetector.cpp:411 — Point dir = lines.middle.front().polyline.back() - lines.middle.front().polyline.front();
    let dir = lines.middle[0].polyline.last_point() - lines.middle[0].polyline.first_point();
    // OverhangDetector.cpp:412 — Point start = lines.middle.front().polyline.front();
    let mut start = lines.middle[0].polyline.first_point();
    // OverhangDetector.cpp:413 — Point end;
    let mut end: Point;
    // OverhangDetector.cpp:414 — for (size_t idx = 0; idx < count - 1; idx++)
    let mut idx: i64 = 0;
    while idx < (count as i64) - 1 {
        // OverhangDetector.cpp:415 — double t = (idx + 1) * cut_gap / length;
        let t = (idx + 1) as f64 * cut_gap / length;
        // OverhangDetector.cpp:416 — end = lines.middle.front().polyline.front() + dir * t;
        // (Point*double truncates — point_mul_f64, see NOTE.)
        end = lines.middle[0].polyline.first_point() + point_mul_f64(dir, t);
        // OverhangDetector.cpp:417 — double degree = d1 + (idx + 1) * degree_gap;
        let degree = d1 + (idx + 1) as f64 * degree_gap;
        // OverhangDetector.cpp:418 — out.push_back(SplitPoly(Polyline(start, end), degree));
        out.push(SplitPoly::with_degree(
            Polyline::from_points(vec![start, end]),
            degree,
        ));
        // OverhangDetector.cpp:419 — start = end;
        start = end;
        idx += 1;
    }
    // OverhangDetector.cpp:421 — out.push_back(SplitPoly(Polyline(start, lines.middle.front().polyline.back()), d1 + count * degree_gap));
    out.push(SplitPoly::with_degree(
        Polyline::from_points(vec![start, lines.middle[0].polyline.last_point()]),
        d1 + count as f64 * degree_gap,
    ));
    // OverhangDetector.cpp:422 — lines.middle = out;
    lines.middle = out;
}

// ---------------------------------------------------------------------------
// OverhangDetector.cpp:425-450 — check_degree
// ---------------------------------------------------------------------------

// OverhangDetector.cpp:425-450
pub fn check_degree(
    input: &mut DegreePolylines,
    prev_layer_distancer: &OverhangDistancer,
    lower_bound: f64,
    upper_bound: f64,
    out: &mut Vec<SplitPoly>,
) {
    // OverhangDetector.cpp:427-433 — lambda chek_overhang
    let chek_overhang = |lines: &mut Vec<SplitPoly>| {
        // OverhangDetector.cpp:428 — for (size_t i = 0; i < lines.size(); ++i)
        for i in 0..lines.len() {
            // OverhangDetector.cpp:429 — Point mid = (lines[i].polyline.front() + lines[i].polyline.back()) / 2;
            let mid = (lines[i].polyline.first_point() + lines[i].polyline.last_point()) / 2;
            // OverhangDetector.cpp:430 — double overhang_dist = prev_layer_distancer->distance_from_perimeter(mid.cast<float>());
            // C++ `mid.cast<float>()` builds a Vec2f (single precision); the
            // distancer then casts it back to double (`Vec2d p = point.cast<double>()`).
            // Faithfully round-trip through f32 so the query point matches C++
            // (for large scaled coords this differs from a bare `as f64`).
            let overhang_dist = prev_layer_distancer
                .distance_from_perimeter(PointF::new(mid.x as f32 as f64, mid.y as f32 as f64))
                as f64;
            // OverhangDetector.cpp:431 — lines[i].degree = get_mapped_degree(overhang_dist, lower_bound, upper_bound);
            lines[i].degree = get_mapped_degree(overhang_dist, lower_bound, upper_bound);
        }
    };

    // OverhangDetector.cpp:435 — for (size_t idx = 0; idx < input.size(); ++idx)
    for idx in 0..input.len() {
        // check each part's degree
        // OverhangDetector.cpp:437 — if (input[idx].start.empty())
        if input[idx].start.is_empty() {
            // OverhangDetector.cpp:438 — chek_overhang(input[idx].middle);
            chek_overhang(&mut input[idx].middle);
        } else {
            // OverhangDetector.cpp:440 — chek_overhang(input[idx].start);
            chek_overhang(&mut input[idx].start);
            // OverhangDetector.cpp:441 — chek_overhang(input[idx].end);
            chek_overhang(&mut input[idx].end);
        }

        // smoothing
        // OverhangDetector.cpp:445 — smoothing_degrees(input[idx]);
        smoothing_degrees(&mut input[idx]);
        // OverhangDetector.cpp:446 — out.insert(out.end(), input[idx].start.begin(), input[idx].start.end());
        out.extend(input[idx].start.iter().cloned());
        // OverhangDetector.cpp:447 — out.insert(out.end(), input[idx].middle.begin(), input[idx].middle.end());
        out.extend(input[idx].middle.iter().cloned());
        // OverhangDetector.cpp:448 — out.insert(out.end(), input[idx].end.begin(), input[idx].end.end());
        out.extend(input[idx].end.iter().cloned());
    }
}

// ---------------------------------------------------------------------------
// OverhangDetector.cpp:452-464 — prepare_split_polylines
// ---------------------------------------------------------------------------

// OverhangDetector.cpp:452-464
pub fn prepare_split_polylines(polyline: Polyline) -> DegreePolylines {
    // OverhangDetector.cpp:454 — DegreePolylines out;
    let mut out: DegreePolylines = Vec::new();
    // OverhangDetector.cpp:455 — if (polyline.size() == 2)
    if polyline.len() == 2 {
        // OverhangDetector.cpp:456 — out.emplace_back(SplitLines(polyline, true));
        out.push(SplitLines::new(polyline, true));
    } else {
        // OverhangDetector.cpp:458 — for (size_t idx = 0; idx < polyline.size() - 1; idx++)
        for idx in 0..polyline.len() - 1 {
            // OverhangDetector.cpp:459 — out.emplace_back(SplitLines(Polyline(polyline[idx], polyline[idx + 1]), false));
            out.push(SplitLines::new(
                Polyline::from_points(vec![polyline.points[idx], polyline.points[idx + 1]]),
                false,
            ));
        }
    }

    // OverhangDetector.cpp:463 — return out;
    out
}

// ---------------------------------------------------------------------------
// ExtrusionEntity.hpp:637 — extrusion_paths_append (classic Polyline&&,
// overhang_degree, curva_degree, role, mm3_per_mm, width, height).
//
// This is the only `extrusion_paths_append` overload used by the *classic*
// detect_overhang_degree (cpp:496-503), and it has no Arachne dependency, so it
// is ported inline here.
//
// NOTE (divergence): C++ `ExtrusionPath::overhang_degree` is a `double`
// (ExtrusionEntity.hpp:216) but the crate's `ExtrusionPath.overhang_degree`
// field is `i32` (extrusion_entity.rs). Storing the continuous overhang degree
// therefore truncates the fractional part. This is a pre-existing divergence in
// extrusion_entity.rs (out of scope for this file); we store the value via
// `as i32` to match the existing field type.
// ---------------------------------------------------------------------------

// ExtrusionEntity.hpp:637-647
fn extrusion_paths_append_classic(
    dst: &mut ExtrusionPaths,
    mut polyline: Polyline,
    overhang_degree: f64,
    curva_degree: i32,
    role: ExtrusionRole,
    mm3_per_mm: f64,
    width: f32,
    height: f32,
) {
    // ExtrusionEntity.hpp:639 — dst.reserve(dst.size() + 1);
    dst.reserve(dst.len() + 1);
    // ExtrusionEntity.hpp:640 — if (polyline.is_valid())
    if polyline.is_valid() {
        // ExtrusionEntity.hpp:641 — dst.push_back(ExtrusionPath(overhang_degree, curva_degree, role, mm3_per_mm, width, height));
        // ExtrusionEntity.hpp:229 — ExtrusionPath(double overhang_degree, int curve_degree, role, double mm3_per_mm, float width, float height)
        let mut ep = ExtrusionPath::new(role);
        ep.overhang_degree = overhang_degree as i32; // (divergence: field is i32, see NOTE)
        ep.curve_degree = curva_degree;
        ep.mm3_per_mm = mm3_per_mm;
        ep.width = width as f64;
        ep.height = height as f64;
        // ExtrusionEntity.hpp:642 — dst.back().polyline = std::move(polyline);
        ep.polyline = std::mem::take(&mut polyline);
        dst.push(ep);
    }
    // ExtrusionEntity.hpp:644 — polyline.clear();
    polyline.clear();
}

// ---------------------------------------------------------------------------
// OverhangDetector.cpp:467-506 — detect_overhang_degree (classic overload)
// ---------------------------------------------------------------------------

// OverhangDetector.cpp:467-506
#[allow(clippy::too_many_arguments)]
pub fn detect_overhang_degree(
    lower_polygons: Vec<Polygon>,
    role: ExtrusionRole,
    extrusion_mm3_per_mm: f64,
    extrusion_width: f64,
    layer_height: f64,
    middle_overhang_polyines: Polylines,
    lower_bound: f64,
    upper_bound: f64,
    paths: &mut ExtrusionPaths,
) {
    // BBS: collect lower_polygons points
    //Polylines;
    // OverhangDetector.cpp:479 — Points lower_polygon_points;  (declared, unused)
    let _lower_polygon_points: Vec<Point> = Vec::new();
    // OverhangDetector.cpp:480 — std::vector<size_t> polygons_bound; (declared, unused)
    let _polygons_bound: Vec<usize> = Vec::new();

    // OverhangDetector.cpp:482-483 — prev_layer_distancer = std::make_unique<OverhangDistancer>(lower_polygons);
    let prev_layer_distancer = OverhangDistancer::new(&lower_polygons);
    //BBS: get overhang degree and split path
    // OverhangDetector.cpp:485 — for (size_t polyline_idx = 0; polyline_idx < middle_overhang_polyines.size(); ++polyline_idx)
    for polyline_idx in 0..middle_overhang_polyines.len() {
        //filter too short polyline
        // OverhangDetector.cpp:487 — std::vector<SplitPoly> out;
        let mut out: Vec<SplitPoly> = Vec::new();

        // OverhangDetector.cpp:489 — Polyline middle_poly = middle_overhang_polyines[polyline_idx];
        let middle_poly = middle_overhang_polyines[polyline_idx].clone();
        // OverhangDetector.cpp:490 — DegreePolylines splited_lines = prepare_split_polylines(middle_poly);
        let mut splited_lines = prepare_split_polylines(middle_poly);
        // OverhangDetector.cpp:491 — check_degree(splited_lines, prev_layer_distancer, lower_bound, upper_bound, out);
        check_degree(
            &mut splited_lines,
            &prev_layer_distancer,
            lower_bound,
            upper_bound,
            &mut out,
        );

        // OverhangDetector.cpp:493 — merged_with_degree(out);
        merged_with_degree(&mut out);
        // merge path by degree
        // OverhangDetector.cpp:495 — for (SplitPoly &polylines_collection : out)
        for polylines_collection in out.drain(..) {
            // OverhangDetector.cpp:496-503 — extrusion_paths_append(paths, std::move(polyline), degree, int(0), role, mm3_per_mm, width, height);
            extrusion_paths_append_classic(
                paths,
                polylines_collection.polyline,
                polylines_collection.degree,
                0,
                role,
                extrusion_mm3_per_mm,
                extrusion_width as f32,
                layer_height as f32,
            );
        }
        // OverhangDetector.cpp:504 — out.clear();
        // (out already drained above)
    }
}
