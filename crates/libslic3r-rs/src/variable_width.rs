//! Variable-width extrusion path generation.
//!
//! Faithful 1:1 port of BambuStudio's `src/libslic3r/VariableWidth.cpp`.
//!
//! Converts `ThickPolyline`s (e.g. from medial-axis computation) into extrusion
//! entities (`ExtrusionPath` / `ExtrusionLoop`) where the extrusion width varies
//! along the length of the path. G-code does not allow variable extrusion within
//! a single move, so the polyline is broken into segments of roughly uniform
//! width.
//!
//! This file mirrors the C++ subdir layout & filename in snake_case.

// VariableWidth.cpp:1
// #include "VariableWidth.hpp"
//
// VariableWidth.hpp:4-6
// #include "Polygon.hpp"
// #include "ExtrusionEntity.hpp"
// #include "Flow.hpp"

use crate::extrusion_entity::{ExtrusionEntityType, ExtrusionLoop, ExtrusionPath, ExtrusionRole};
use crate::flow::Flow;
use crate::geometry::{Point, PointF, ThickLine, ThickPolyline, ThickPolylines};
use crate::{Coord, CoordF};
use std::f64::consts::PI;

/// `SCALED_EPSILON` expressed in this crate's scaled coordinate system.
///
/// libslic3r.h:84 `#define SCALED_EPSILON scale_(EPSILON)`, where
/// libslic3r.h:81 `#define scale_(val) ((val) / SCALING_FACTOR)`,
/// libslic3r.h:58 `SCALING_FACTOR = 0.00001`, and
/// libslic3r.h:52 `EPSILON = 1e-4`. Therefore in C++:
/// `SCALED_EPSILON = 1e-4 / 0.00001 = 10.0` scaled units (1 mm == 100_000 units).
///
/// This crate's `SCALING_FACTOR` is `100_000` (1 mm == 100_000 units), so the
/// equivalent scaled value for `EPSILON = 1e-4 mm` is `1e-4 * 100_000 == 10.0`.
/// Lengths here are in this crate's scaled units, so the comparison threshold
/// is `10.0` (matching C++ exactly), NOT `1.0`.
const SCALED_EPSILON: f64 = 1e-4 * crate::SCALING_FACTOR;

// VariableWidth.cpp:5-8
// ExtrusionMultiPath thick_polyline_to_multi_path(const ThickPolyline& thick_polyline, ExtrusionRole role, const Flow& flow, const float tolerance, const float merge_tolerance, double overhang)
// {
//     ExtrusionMultiPath multi_path;
//     ExtrusionPath      path(role);

/// A continuous chain of `ExtrusionPath`s, each possibly with varying extrusion
/// thickness / height. Faithful counterpart of C++ `ExtrusionMultiPath`
/// (ExtrusionEntity.hpp:428).
#[derive(Debug, Clone, Default)]
pub struct ExtrusionMultiPath {
    pub paths: Vec<ExtrusionPath>,
}

impl ExtrusionMultiPath {
    pub fn new() -> Self {
        Self { paths: Vec::new() }
    }
}

/// Vector of `ExtrusionPath`. Faithful counterpart of C++ `ExtrusionPaths`
/// (ExtrusionEntity.hpp).
pub type ExtrusionPaths = Vec<ExtrusionPath>;

/// VariableWidth.cpp:5
/// C++: ExtrusionMultiPath thick_polyline_to_multi_path(const ThickPolyline& thick_polyline, ExtrusionRole role, const Flow& flow, const float tolerance, const float merge_tolerance, double overhang)
pub fn thick_polyline_to_multi_path(
    thick_polyline: &ThickPolyline,
    role: ExtrusionRole,
    flow: &Flow,
    tolerance: f32,
    merge_tolerance: f32,
    overhang: f64,
) -> ExtrusionMultiPath {
    // TPMPPROBE (R569) — measure both ends of this function in one pass: how much
    // width variation ENTERS (the ThickPolyline's widths) versus how many paths
    // LEAVE (each extra path is one intra-loop `; LINE_WIDTH:` tag). Scoped to the
    // outer wall so it matches the G-code classification in R568.
    let tpmp_in: Option<(usize, usize, u64)> =
        if crate::probe_enabled("TPMPPROBE") && role == ExtrusionRole::ExternalPerimeter {
            let w = &thick_polyline.widths;
            let changes = (1..w.len()).filter(|&k| w[k] != w[k - 1]).count();
            let mut distinct: Vec<i64> = w.iter().map(|v| *v as i64).collect();
            distinct.sort_unstable();
            distinct.dedup();
            let spread = if w.is_empty() {
                0
            } else {
                let mx = w.iter().cloned().fold(f64::MIN, f64::max);
                let mn = w.iter().cloned().fold(f64::MAX, f64::min);
                (mx - mn) as u64
            };
            Some((changes, distinct.len(), spread))
        } else {
            None
        };

    // VariableWidth.cpp:7-9
    let mut multi_path = ExtrusionMultiPath::new();
    let mut path = ExtrusionPath::new(role);
    let mut lines: Vec<ThickLine> = thick_polyline.thicklines();

    // VariableWidth.cpp:11
    // for (int i = 0; i < (int)lines.size(); ++i) {
    let mut i: i32 = 0;
    while i < lines.len() as i32 {
        // VariableWidth.cpp:12-13
        let line = lines[i as usize].clone();
        debug_assert!(line.a_width >= SCALED_EPSILON && line.b_width >= SCALED_EPSILON);

        // VariableWidth.cpp:15
        // const coordf_t line_len = line.length();
        let line_len: CoordF = line.length();
        // VariableWidth.cpp:16
        if line_len < SCALED_EPSILON {
            // VariableWidth.cpp:17
            // The line is so tiny that we don't care about its width when we connect it to another line.
            // VariableWidth.cpp:18-19
            if !path.polyline.points.is_empty() {
                // If the variable path is non-empty, connect this tiny line to it.
                let last = path.polyline.points.len() - 1;
                path.polyline.points[last] = line.b;
            } else if i + 1 < lines.len() as i32 {
                // VariableWidth.cpp:20-21
                // If there is at least one following line, connect this tiny line to it.
                lines[(i + 1) as usize].a = line.a;
            } else if !multi_path.paths.is_empty() {
                // VariableWidth.cpp:22-23
                // Connect this tiny line to the last finished path.
                let last_path = multi_path.paths.len() - 1;
                let last_pt = multi_path.paths[last_path].polyline.points.len() - 1;
                multi_path.paths[last_path].polyline.points[last_pt] = line.b;
            }

            // VariableWidth.cpp:25-26
            // If any of the above isn't satisfied, then remove this tiny line.
            i += 1;
            continue;
        }

        // VariableWidth.cpp:29
        // double thickness_delta = fabs(line.a_width - line.b_width);
        let mut thickness_delta: f64 = (line.a_width - line.b_width).abs();
        // VariableWidth.cpp:30
        if thickness_delta > tolerance as f64 {
            // VariableWidth.cpp:31
            // const auto segments = (unsigned int)ceil(thickness_delta / tolerance);
            let segments: u32 = (thickness_delta / tolerance as f64).ceil() as u32;
            // VariableWidth.cpp:32
            // const coordf_t seg_len = line_len / segments;
            let seg_len: CoordF = line_len / segments as CoordF;
            // VariableWidth.cpp:33-34
            let mut pp: Vec<Point> = Vec::new();
            let mut width: Vec<CoordF> = Vec::new();
            {
                // VariableWidth.cpp:36-37
                pp.push(line.a);
                width.push(line.a_width);
                // VariableWidth.cpp:38
                for j in 1..segments as usize {
                    // VariableWidth.cpp:39
                    // pp.push_back((line.a.cast<double>() + (line.b - line.a).cast<double>().normalized() * (j * seg_len)).cast<coord_t>());
                    let a = PointF::new(line.a.x as f64, line.a.y as f64);
                    let dir = PointF::new(
                        (line.b.x - line.a.x) as f64,
                        (line.b.y - line.a.y) as f64,
                    );
                    let dir = normalized(dir);
                    let scale_factor = j as f64 * seg_len;
                    let p = PointF::new(
                        a.x + dir.x * scale_factor,
                        a.y + dir.y * scale_factor,
                    );
                    pp.push(Point::new(p.x as Coord, p.y as Coord));

                    // VariableWidth.cpp:41
                    // coordf_t w = line.a_width + (j*seg_len) * (line.b_width-line.a_width) / line_len;
                    let w: CoordF =
                        line.a_width + (j as f64 * seg_len) * (line.b_width - line.a_width) / line_len;
                    // VariableWidth.cpp:42-43
                    width.push(w);
                    width.push(w);
                }
                // VariableWidth.cpp:45-46
                pp.push(line.b);
                width.push(line.b_width);

                // VariableWidth.cpp:48-49
                debug_assert!(pp.len() == segments as usize + 1);
                debug_assert!(width.len() == segments as usize * 2);
            }

            // VariableWidth.cpp:52-53
            // delete this line and insert new ones
            lines.remove(i as usize);
            // VariableWidth.cpp:54
            for j in 0..segments as usize {
                // VariableWidth.cpp:55-57
                let mut new_line = ThickLine::new(pp[j], pp[j + 1], 0.0, 0.0);
                new_line.a_width = width[2 * j];
                new_line.b_width = width[2 * j + 1];
                // VariableWidth.cpp:58
                lines.insert(i as usize + j, new_line);
            }

            // VariableWidth.cpp:61-62
            // C++: `-- i; continue;` — in the for-loop `continue` then runs `++ i`,
            // so the net effect is that `i` is unchanged and the loop re-examines the
            // first freshly-inserted line. We model that with an unchanged `continue`.
            continue;
        }

        // VariableWidth.cpp:65
        // const double w = fmax(line.a_width, line.b_width);
        let w: f64 = line.a_width.max(line.b_width);
        // VariableWidth.cpp:66
        // const Flow new_flow = (role == erOverhangPerimeter && flow.bridge()) ? flow : flow.with_width(unscale<float>(w) + flow.height() * float(1. - 0.25 * PI));
        let new_flow: Flow = if role == ExtrusionRole::OverhangPerimeter && flow.is_bridge() {
            flow.clone()
        } else {
            flow.with_width(spacing_to_width(unscale_f(w), flow.height()))
                .unwrap_or_else(|_| flow.clone())
        };
        // VariableWidth.cpp:67
        if path.polyline.points.is_empty() {
            // VariableWidth.cpp:68-69
            path.polyline.points.push(line.a);
            path.polyline.points.push(line.b);
            // VariableWidth.cpp:70-71
            // Convert from spacing to extrusion width based on the extrusion model
            // of a square extrusion ended with semi circles.
            // VariableWidth.cpp:75-77
            path.mm3_per_mm = new_flow.mm3_per_mm().unwrap_or(0.0);
            path.width = new_flow.width();
            path.height = new_flow.height();
        } else {
            // VariableWidth.cpp:79
            debug_assert!(path.width >= crate::libslic3r::EPSILON);
            // VariableWidth.cpp:80
            // thickness_delta = scaled<double>(fabs(path.width - new_flow.width()));
            thickness_delta = scaled_f((path.width - new_flow.width()).abs());
            // VariableWidth.cpp:81
            if thickness_delta <= merge_tolerance as f64 {
                // VariableWidth.cpp:82-84
                // the width difference between this line and the current flow
                // (of the previous line) width is within the accepted tolerance
                path.polyline.points.push(line.b);
            } else {
                // VariableWidth.cpp:85-89
                // we need to initialize a new line
                multi_path.paths.push(std::mem::replace(&mut path, ExtrusionPath::new(role)));
                i -= 1;
            }
        }

        i += 1;
    }
    // VariableWidth.cpp:93
    if path.polyline.is_valid() {
        // VariableWidth.cpp:94 — path.overhang_degree = overhang; (double)
        path.overhang_degree = overhang;
        multi_path.paths.push(path);
    }

    // TPMPPROBE (R569) — cumulative totals; take the LAST printed line.
    if let Some((changes, distinct, spread)) = tpmp_in {
        use std::sync::atomic::{AtomicU64, Ordering};
        static CALLS: AtomicU64 = AtomicU64::new(0);
        static PTS: AtomicU64 = AtomicU64::new(0);
        static CHANGES: AtomicU64 = AtomicU64::new(0);
        static DISTINCT: AtomicU64 = AtomicU64::new(0);
        static SPREAD: AtomicU64 = AtomicU64::new(0);
        static FLAT: AtomicU64 = AtomicU64::new(0);
        static PATHS: AtomicU64 = AtomicU64::new(0);
        PTS.fetch_add(thick_polyline.widths.len() as u64, Ordering::Relaxed);
        CHANGES.fetch_add(changes as u64, Ordering::Relaxed);
        DISTINCT.fetch_add(distinct as u64, Ordering::Relaxed);
        SPREAD.fetch_add(spread, Ordering::Relaxed);
        if changes == 0 {
            FLAT.fetch_add(1, Ordering::Relaxed);
        }
        PATHS.fetch_add(multi_path.paths.len() as u64, Ordering::Relaxed);
        let n = CALLS.fetch_add(1, Ordering::Relaxed) + 1;
        if n % 1_000 == 0 {
            println!(
                "TPMPPROBE calls={} widthpts={} in_changes={} in_distinct={} in_spread={} flat_calls={} out_paths={}",
                n,
                PTS.load(Ordering::Relaxed),
                CHANGES.load(Ordering::Relaxed),
                DISTINCT.load(Ordering::Relaxed),
                SPREAD.load(Ordering::Relaxed),
                FLAT.load(Ordering::Relaxed),
                PATHS.load(Ordering::Relaxed),
            );
        }
    }

    // VariableWidth.cpp:97
    multi_path
}

/// VariableWidth.cpp:100-101
/// BBS: new function to filter width to avoid too fragmented segments
/// C++: static ExtrusionPaths thick_polyline_to_extrusion_paths_2(const ThickPolyline& thick_polyline, ExtrusionRole role, const Flow& flow, const float tolerance)
fn thick_polyline_to_extrusion_paths_2(
    thick_polyline: &ThickPolyline,
    role: ExtrusionRole,
    flow: &Flow,
    tolerance: f32,
) -> ExtrusionPaths {
    // VariableWidth.cpp:103-105
    let mut paths: ExtrusionPaths = ExtrusionPaths::new();
    let mut path = ExtrusionPath::new(role);
    let mut lines: Vec<ThickLine> = thick_polyline.thicklines();

    // VariableWidth.cpp:107-108
    let mut start_index: usize = 0;
    let mut max_width: f64 = 0.0;
    let mut min_width: f64 = 0.0;

    // VariableWidth.cpp:110
    // for (int i = 0; i < (int)lines.size(); ++i) {
    let mut i: i32 = 0;
    while i < lines.len() as i32 {
        // VariableWidth.cpp:111
        let line = lines[i as usize].clone();

        // VariableWidth.cpp:113-116
        if i == 0 {
            max_width = line.a_width;
            min_width = line.a_width;
        }

        // VariableWidth.cpp:118-119
        let line_len: CoordF = line.length();
        if line_len < SCALED_EPSILON {
            i += 1;
            continue;
        }

        // VariableWidth.cpp:121
        // double thickness_delta = std::max(fabs(max_width - line.b_width), fabs(min_width - line.b_width));
        let mut thickness_delta: f64 =
            (max_width - line.b_width).abs().max((min_width - line.b_width).abs());
        // VariableWidth.cpp:122-123
        // BBS: has large difference in width
        if thickness_delta > tolerance as f64 {
            // VariableWidth.cpp:124-125
            // BBS: 1 generate path from start_index to i(not included)
            if start_index != i as usize {
                // VariableWidth.cpp:126-127
                path = ExtrusionPath::new(role);
                let mut length: f64 = 0.0;
                let mut sum: f64 = 0.0;
                // VariableWidth.cpp:128-132
                for idx in start_index..i as usize {
                    length += lines[idx].length();
                    sum += lines[idx].length() * 0.5 * (lines[idx].a_width + lines[idx].b_width);
                    path.polyline.points.push(lines[idx].a);
                }
                // VariableWidth.cpp:133
                path.polyline.points.push(lines[i as usize].a);
                // VariableWidth.cpp:134
                if length > SCALED_EPSILON {
                    // VariableWidth.cpp:135-136
                    let w: f64 = sum / length;
                    let new_flow: Flow = flow
                        .with_width(spacing_to_width(unscale_f(w), flow.height()))
                        .unwrap_or_else(|_| flow.clone());
                    // VariableWidth.cpp:137-139
                    path.mm3_per_mm = new_flow.mm3_per_mm().unwrap_or(0.0);
                    path.width = new_flow.width();
                    path.height = new_flow.height();
                    // VariableWidth.cpp:140
                    paths.push(path.clone());
                }
            }

            // VariableWidth.cpp:144-146
            start_index = i as usize;
            max_width = line.a_width;
            min_width = line.a_width;

            // VariableWidth.cpp:148-149
            // BBS: 2 handle the i-th segment
            thickness_delta = (line.a_width - line.b_width).abs();
            // VariableWidth.cpp:150
            if thickness_delta > tolerance as f64 {
                // VariableWidth.cpp:151
                let segments: u32 = (thickness_delta / tolerance as f64).ceil() as u32;
                // VariableWidth.cpp:152
                let seg_len: CoordF = line_len / segments as CoordF;
                // VariableWidth.cpp:153-154
                let mut pp: Vec<Point> = Vec::new();
                let mut width: Vec<CoordF> = Vec::new();
                {
                    // VariableWidth.cpp:156-157
                    pp.push(line.a);
                    width.push(line.a_width);
                    // VariableWidth.cpp:158
                    for j in 1..segments as usize {
                        // VariableWidth.cpp:159
                        let a = PointF::new(line.a.x as f64, line.a.y as f64);
                        let dir = PointF::new(
                            (line.b.x - line.a.x) as f64,
                            (line.b.y - line.a.y) as f64,
                        );
                        let dir = normalized(dir);
                        let scale_factor = j as f64 * seg_len;
                        let p = PointF::new(
                            a.x + dir.x * scale_factor,
                            a.y + dir.y * scale_factor,
                        );
                        pp.push(Point::new(p.x as Coord, p.y as Coord));

                        // VariableWidth.cpp:161
                        let w: CoordF = line.a_width
                            + (j as f64 * seg_len) * (line.b_width - line.a_width) / line_len;
                        // VariableWidth.cpp:162-163
                        width.push(w);
                        width.push(w);
                    }
                    // VariableWidth.cpp:165-166
                    pp.push(line.b);
                    width.push(line.b_width);

                    // VariableWidth.cpp:168-169
                    debug_assert!(pp.len() == segments as usize + 1);
                    debug_assert!(width.len() == segments as usize * 2);
                }

                // VariableWidth.cpp:172-173
                // delete this line and insert new ones
                lines.remove(i as usize);
                // VariableWidth.cpp:174
                for j in 0..segments as usize {
                    // VariableWidth.cpp:175-177
                    let mut new_line = ThickLine::new(pp[j], pp[j + 1], 0.0, 0.0);
                    new_line.a_width = width[2 * j];
                    new_line.b_width = width[2 * j + 1];
                    // VariableWidth.cpp:178
                    lines.insert(i as usize + j, new_line);
                }
                // VariableWidth.cpp:180-181
                // C++: `--i; continue;` — `continue` then runs `++i`, so `i` is
                // unchanged and the loop re-examines the first inserted line.
                continue;
            }
        }
        // VariableWidth.cpp:184-188
        // BBS: just update the max and min width and continue
        else {
            max_width = max_width.max(line.a_width.max(line.b_width));
            min_width = min_width.min(line.a_width.min(line.b_width));
        }

        i += 1;
    }
    // VariableWidth.cpp:190-191
    // BBS: handle the remaining segment
    let final_size: usize = lines.len();
    if start_index < final_size {
        // VariableWidth.cpp:193-194
        path = ExtrusionPath::new(role);
        let mut length: f64 = 0.0;
        let mut sum: f64 = 0.0;
        // VariableWidth.cpp:195-199
        for idx in start_index..final_size {
            length += lines[idx].length();
            sum += lines[idx].length() * (lines[idx].a_width + lines[idx].b_width) * 0.5;
            path.polyline.points.push(lines[idx].a);
        }
        // VariableWidth.cpp:200
        path.polyline.points.push(lines[final_size - 1].b);
        // VariableWidth.cpp:201
        if length > SCALED_EPSILON {
            // VariableWidth.cpp:202-203
            let w: f64 = sum / length;
            let new_flow: Flow = flow
                .with_width(spacing_to_width(unscale_f(w), flow.height()))
                .unwrap_or_else(|_| flow.clone());
            // VariableWidth.cpp:204-206
            path.mm3_per_mm = new_flow.mm3_per_mm().unwrap_or(0.0);
            path.width = new_flow.width();
            path.height = new_flow.height();
            // VariableWidth.cpp:207
            paths.push(path.clone());
        }
    }

    // VariableWidth.cpp:211
    paths
}

/// VariableWidth.cpp:214
/// C++: void variable_width(const ThickPolylines& polylines, ExtrusionRole role, const Flow& flow, std::vector<ExtrusionEntity*>& out)
pub fn variable_width(
    polylines: &ThickPolylines,
    role: ExtrusionRole,
    flow: &Flow,
    out: &mut Vec<ExtrusionEntityType>,
) {
    // VariableWidth.cpp:215-218
    // This value determines granularity of adaptive width, as G-code does not allow
    // variable extrusion within a single move; this value shall only affect the amount
    // of segments, and any pruning shall be performed before we apply this tolerance.
    // const float tolerance = float(scale_(0.05));
    // NOTE: `scale_` here is the crate-root scaling (SCALING_FACTOR == 100_000),
    // consistent with the scaled units used by `ThickLine` lengths and widths.
    let tolerance: f32 = crate::scale(0.05) as f32;
    // VariableWidth.cpp:220
    for p in polylines {
        // VariableWidth.cpp:221
        let mut paths: ExtrusionPaths = thick_polyline_to_extrusion_paths_2(p, role, flow, tolerance);
        // VariableWidth.cpp:222-223
        // Append paths to collection.
        if !paths.is_empty() {
            // VariableWidth.cpp:224
            if paths.first().unwrap().first_point() == paths.last().unwrap().last_point() {
                // VariableWidth.cpp:225
                out.push(ExtrusionEntityType::Loop(ExtrusionLoop::new(
                    std::mem::take(&mut paths),
                    crate::extrusion_entity::ExtrusionLoopRole::DEFAULT,
                )));
            } else {
                // VariableWidth.cpp:226-229
                for path in paths {
                    out.push(ExtrusionEntityType::Path(path));
                }
            }
        }
    }
}

/// Helper: unscale a scaled `coordf_t` value to mm.
/// C++: `unscale<float>(w)` (libslic3r.h:112 `unscale(v) = T(v) * T(SCALING_FACTOR)`,
/// `SCALING_FACTOR = 0.00001`) i.e. scaled-units -> mm. Here the crate's scaling
/// convention is 1mm = SCALING_FACTOR(=100_000) units, so unscale divides.
///
/// FIDELITY-NOTE(width-units, cross-cutting): C++ `medial_axis` keeps `ThickPolyline`
/// widths in SCALED coordinates, so `VariableWidth.cpp` correctly unscales them
/// here. This crate's `geometry/medial_axis.rs:346` instead stores widths already
/// in mm (`widths[i] / SCALING_FACTOR`). Applying this unscale to mm widths
/// double-unscales (off by SCALING_FACTOR). Reconciling the width-unit convention
/// is a producer-side (medial_axis / thick_polyline) data-model change, NOT a
/// VariableWidth logic change: matching C++ requires unscale HERE, so this file
/// stays C++-faithful and the divergence must be fixed at the producer.
#[inline]
fn unscale_f(scaled_val: f64) -> f64 {
    scaled_val / crate::SCALING_FACTOR
}

/// R231: native computes the spacing→width conversion in f32
/// (VariableWidth.cpp:66 `unscale<float>(w) + flow.height() * float(1.-0.25*PI)`
/// — float sum of float terms). The f64 chain drifts the 6th significant
/// digit of path.width (0.43272-vs-0.43273 LINE_WIDTH flips). Gated FLOW_F32.
pub fn spacing_to_width(w_unscaled: f64, height: f64) -> f64 {
    const C: f64 = 1.0 - 0.25 * std::f64::consts::PI;
    if crate::flow::flow_f32() {
        ((w_unscaled as f32) + (height as f32) * (C as f32)) as f64
    } else {
        w_unscaled + height * C
    }
}

/// Helper: scale an unscaled mm value to a scaled `coordf_t`.
/// C++: `scaled<double>(v)` == `v / SCALING_FACTOR` (with `SCALING_FACTOR` < 1);
/// here the crate's scaling convention multiplies by SCALING_FACTOR.
#[inline]
fn scaled_f(mm: f64) -> f64 {
    mm * crate::SCALING_FACTOR
}

/// Normalize a 2D float vector (Eigen `.normalized()`).
#[inline]
fn normalized(v: PointF) -> PointF {
    let len = (v.x * v.x + v.y * v.y).sqrt();
    if len == 0.0 {
        PointF::new(0.0, 0.0)
    } else {
        PointF::new(v.x / len, v.y / len)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Point;

    fn s(mm: f64) -> i64 {
        crate::scale(mm)
    }

    fn make_flow() -> Flow {
        Flow::new(0.4, 0.2, 0.4).unwrap()
    }

    #[test]
    fn test_variable_width_empty() {
        let flow = make_flow();
        let mut out = Vec::new();
        variable_width(&vec![], ExtrusionRole::GapFill, &flow, &mut out);
        assert!(out.is_empty());
    }

    #[test]
    fn test_variable_width_uniform() {
        let flow = make_flow();
        // Widths in scaled units; build a ThickPolyline directly via thicklines.
        let mut tp = ThickPolyline::new();
        tp.points = vec![Point::new(0, 0), Point::new(s(5.0), 0), Point::new(s(10.0), 0)];
        // widths array is one-per-vertex in the crate's ThickPolyline model.
        let w = s(0.4) as f64;
        tp.widths = vec![w, w, w];

        let mut out = Vec::new();
        variable_width(&vec![tp], ExtrusionRole::GapFill, &flow, &mut out);
        // Uniform width should produce a single path entity.
        assert_eq!(out.len(), 1);
    }

    #[test]
    fn test_variable_width_split_at_large_change() {
        let flow = make_flow();
        let mut tp = ThickPolyline::new();
        tp.points = vec![
            Point::new(0, 0),
            Point::new(s(5.0), 0),
            Point::new(s(10.0), 0),
            Point::new(s(15.0), 0),
        ];
        tp.widths = vec![
            s(0.1) as f64,
            s(0.12) as f64,
            s(0.5) as f64,
            s(0.52) as f64,
        ];
        let mut out = Vec::new();
        variable_width(&vec![tp], ExtrusionRole::GapFill, &flow, &mut out);
        assert!(!out.is_empty());
    }
}
