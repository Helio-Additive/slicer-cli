//! Faithful 1:1 port of `src/libslic3r/Fill/FillFloatingConcentric.cpp` (+ `.hpp`).
//!
//! Line refs are given as `// FillFloatingConcentric.cpp:NNN`.
//!
//! Type mapping: `coord_t` -> `i64` (`Coord`), `coordf_t` -> `f64` (`CoordF`).
//!
//! # Width layout
//!
//! C++ `ThickPolyline::width` stores TWO width values per segment, i.e.
//! `width.size() == 2 * points.size() - 2`. `FloatingThickPolyline` inherits that
//! layout. This module therefore stores the floating thick polyline's `width`
//! exactly as the C++ does (a flat `Vec<f64>` of length `2*N-2`), independent of
//! the crate's per-vertex `geometry::ThickPolyline::widths` convention.
//!
//! # Blocked symbols (NOT ported here — require not-yet-available backends)
//!
//! The following C++ symbols depend on the legacy `ClipperLib_Z::Clipper`
//! (custom `ZFillFunction` + `PolyTree` + `Execute` + `PolyTreeToPaths`) and/or
//! `EdgeGrid::Grid::has_intersecting_edges`, neither of which exists in this
//! crate (the clipper backend is Clipper2 f64; see `overhang_detector.rs`,
//! which documents the same limitation). They are intentionally left out and
//! must be ported once a Z-aware clipper with a user fill callback lands:
//!
//! - `detect_floating_line`           (needs `ClipperLib_Z::Clipper` + ZFillFunction)
//! - `FillFloatingConcentric::resplit_order_loops`     (needs `detect_floating_line` + `EdgeGrid::has_intersecting_edges`)
//! - `FillFloatingConcentric::_fill_surface_single`    (needs the above + Fill base threading)
//! - `FillFloatingConcentric::fill_surface_arachne_floating` / `fill_surface_extrusion`
//!                                     (need the Fill base class: `no_overlap_expolygons`,
//!                                      `lower_layer_unsupport_areas`, `lower_sparse_polys`,
//!                                      `_infill_direction`, `ExtrusionEntityCollection` output)
//!
//! All other functions are ported faithfully below.

use crate::arachne::utils::extrusion_line::ExtrusionLine;
use crate::arachne::wall_tool_paths::WallToolPaths;
use crate::clipper_z_utils::ZPath;
use crate::extrusion_entity::{CustomizeFlag, ExtrusionPath, ExtrusionRole};
use crate::flow::Flow;
use crate::geometry::{
    get_extents, BoundingBox, ExPolygons, Point, Polygon, Polygons, ThickPolyline,
};
use crate::libslic3r::SCALED_EPSILON;
use crate::utils::prev_idx_modulo;
use crate::{Coord, CoordF};
use std::collections::{HashMap, HashSet};
use std::f64::consts::PI;

// =============================================================================
// FillFloatingConcentric.hpp data structures
// =============================================================================

/// FillFloatingConcentric.hpp:9-18
/// `struct FloatingThickline : public ThickLine`
#[derive(Clone, Debug)]
pub struct FloatingThickline {
    // ThickLine base.
    pub a: Point,
    pub b: Point,
    pub a_width: CoordF,
    pub b_width: CoordF,
    // FillFloatingConcentric.hpp:16-17
    pub is_a_floating: bool,
    pub is_b_floating: bool,
}

impl FloatingThickline {
    /// FillFloatingConcentric.hpp:11-15
    /// `FloatingThickline(const Point& a, const Point& b, double wa, double wb, bool a_floating, bool b_floating) :ThickLine(a, b, wa, wb)`
    pub fn new(
        a: Point,
        b: Point,
        wa: CoordF,
        wb: CoordF,
        a_floating: bool,
        b_floating: bool,
    ) -> Self {
        Self {
            a,
            b,
            a_width: wa,
            b_width: wb,
            is_a_floating: a_floating,
            is_b_floating: b_floating,
        }
    }

    /// `ThickLine::length()` — inherited from `Line`.
    pub fn length(&self) -> CoordF {
        self.a.distance(&self.b)
    }
}

/// FillFloatingConcentric.hpp:19
/// `using FloatingThicklines = std::vector<FloatingThickline>;`
pub type FloatingThicklines = Vec<FloatingThickline>;

/// FillFloatingConcentric.hpp:21-25
/// `struct FloatingPolyline : public Polyline`
#[derive(Clone, Debug, Default)]
pub struct FloatingPolyline {
    // Polyline base.
    pub points: Vec<Point>,
    // FillFloatingConcentric.hpp:23
    pub is_floating: Vec<bool>,
}

impl FloatingPolyline {
    /// `Polyline::is_closed()` — `this->points.front() == this->points.back()`.
    pub fn is_closed(&self) -> bool {
        !self.points.is_empty() && self.points.first() == self.points.last()
    }

    /// FillFloatingConcentric.cpp:18-32
    /// `FloatingPolyline FloatingPolyline::rebase_at(size_t idx)`
    pub fn rebase_at(&self, idx: usize) -> FloatingPolyline {
        // FillFloatingConcentric.cpp:20-21
        if !self.is_closed() {
            return FloatingPolyline::default();
        }

        // FillFloatingConcentric.cpp:23-24
        // `FloatingPolyline ret = *this;`
        // `static_cast<Polyline&>(ret) = Polyline::rebase_at(idx);`
        let mut ret = self.clone();
        ret.points = polyline_rebase_at(&self.points, idx);
        // FillFloatingConcentric.cpp:25
        let n = self.points.len();
        // FillFloatingConcentric.cpp:26
        ret.is_floating.resize(n, false);
        // FillFloatingConcentric.cpp:27-29
        for j in 0..(n - 1) {
            ret.is_floating[j] = self.is_floating[(idx + j) % (n - 1)];
        }
        // FillFloatingConcentric.cpp:30
        let front = ret.is_floating[0];
        ret.is_floating.push(front);
        // FillFloatingConcentric.cpp:31
        ret
    }
}

/// FillFloatingConcentric.hpp:26
/// `using FloatingPolylines = std::vector<FloatingPolyline>;`
pub type FloatingPolylines = Vec<FloatingPolyline>;

/// FillFloatingConcentric.hpp:28-33
/// `struct FloatingThickPolyline :public ThickPolyline`
#[derive(Clone, Debug, Default)]
pub struct FloatingThickPolyline {
    // ThickPolyline base.
    pub points: Vec<Point>,
    /// Two entries per segment: `width.len() == 2 * points.len() - 2`.
    pub width: Vec<CoordF>,
    pub endpoints: (bool, bool),
    // FillFloatingConcentric.hpp:30
    pub is_floating: Vec<bool>,
}

impl FloatingThickPolyline {
    /// `MultiPoint::empty()`.
    pub fn empty(&self) -> bool {
        self.points.is_empty()
    }

    /// `Polyline::is_closed()` — `this->points.front() == this->points.back()`.
    pub fn is_closed(&self) -> bool {
        !self.points.is_empty() && self.points.first() == self.points.last()
    }

    /// `MultiPoint::last_point()`.
    pub fn last_point(&self) -> Point {
        *self.points.last().unwrap()
    }

    /// `Polyline::is_valid()` — Polyline.hpp: `this->points.size() >= 2`.
    pub fn is_valid(&self) -> bool {
        self.points.len() >= 2
    }

    /// `Polyline::clip_end(double distance)` — Polyline.cpp:52-72.
    ///
    /// Inherited from `Polyline`; operates only on `points` (the per-segment
    /// `width` vector is intentionally left untouched, matching the base-class
    /// behaviour — see geometry::ThickPolyline::clip_end). `is_floating` is
    /// likewise left untouched (the C++ FloatingThickPolyline does not override
    /// clip_end).
    pub fn clip_end(&mut self, mut distance: f64) {
        while distance > 0.0 {
            // Polyline.cpp:57
            let last_point = (
                self.points.last().unwrap().x as f64,
                self.points.last().unwrap().y as f64,
            );
            // Polyline.cpp:58
            self.points.pop();
            // Polyline.cpp:60-63
            if self.points.is_empty() {
                return;
            }
            // Polyline.cpp:64-65
            let vx = self.points.last().unwrap().x as f64 - last_point.0;
            let vy = self.points.last().unwrap().y as f64 - last_point.1;
            let lsqr = vx * vx + vy * vy;
            // Polyline.cpp:66-69
            if lsqr > distance * distance {
                let s = distance / lsqr.sqrt();
                let nx = last_point.0 + vx * s;
                let ny = last_point.1 + vy * s;
                self.points.push(Point::new(nx as Coord, ny as Coord));
                break;
            }
            // Polyline.cpp:71
            distance -= lsqr.sqrt();
        }
    }

    /// `ThickPolyline::get_width_at(size_t point_idx)` — Polyline.cpp:666-671.
    pub fn get_width_at(&self, point_idx: usize) -> CoordF {
        // Polyline.cpp:668-669
        if point_idx < 2 {
            return self.width[point_idx];
        }
        // Polyline.cpp:670
        self.width[2 * point_idx - 1]
    }

    /// FillFloatingConcentric.cpp:34-47
    /// `FloatingThickPolyline FloatingThickPolyline::rebase_at(size_t idx)`
    pub fn rebase_at(&self, idx: usize) -> FloatingThickPolyline {
        // FillFloatingConcentric.cpp:36-37
        if !self.is_closed() {
            return FloatingThickPolyline::default();
        }
        // FillFloatingConcentric.cpp:38-39
        // `FloatingThickPolyline ret = *this;`
        // `static_cast<ThickPolyline&>(ret) = ThickPolyline::rebase_at(idx);`
        let mut ret = self.clone();
        let base = thick_polyline_rebase_at(&self.points, &self.width, idx);
        ret.points = base.0;
        ret.width = base.1;
        // FillFloatingConcentric.cpp:40
        let n = self.points.len();
        // FillFloatingConcentric.cpp:41
        ret.is_floating.resize(n, false);
        // FillFloatingConcentric.cpp:42-44
        for j in 0..(n - 1) {
            ret.is_floating[j] = self.is_floating[(idx + j) % (n - 1)];
        }
        // FillFloatingConcentric.cpp:45
        let front = ret.is_floating[0];
        ret.is_floating.push(front);
        // FillFloatingConcentric.cpp:46
        ret
    }

    /// FillFloatingConcentric.cpp:49-58
    /// `FloatingThicklines FloatingThickPolyline::floating_thicklines() const`
    pub fn floating_thicklines(&self) -> FloatingThicklines {
        // FillFloatingConcentric.cpp:51
        let mut lines = FloatingThicklines::new();
        // FillFloatingConcentric.cpp:52
        if self.points.len() >= 2 {
            // FillFloatingConcentric.cpp:53
            lines.reserve(self.points.len() - 1);
            // FillFloatingConcentric.cpp:54-55
            for i in 0..(self.points.len() - 1) {
                lines.push(FloatingThickline::new(
                    self.points[i],
                    self.points[i + 1],
                    self.width[2 * i],
                    self.width[2 * i + 1],
                    self.is_floating[i],
                    self.is_floating[i + 1],
                ));
            }
        }
        // FillFloatingConcentric.cpp:57
        lines
    }
}

/// FillFloatingConcentric.hpp:34
/// `using FloatingThickPolylines = std::vector<FloatingThickPolyline>;`
pub type FloatingThickPolylines = Vec<FloatingThickPolyline>;

// -----------------------------------------------------------------------------
// Polyline / ThickPolyline base-class helpers used by the floating rebase_at.
// Faithful translations of Polyline.cpp:621-664 (the C++ base classes that the
// FloatingPolyline / FloatingThickPolyline rebase_at methods delegate to via
// `static_cast<Base&>(ret) = Base::rebase_at(idx)`).
// -----------------------------------------------------------------------------

/// Polyline.cpp:621-632 `Polyline Polyline::rebase_at(size_t idx)`.
/// (Caller has already verified `is_closed()`.)
fn polyline_rebase_at(points: &[Point], idx: usize) -> Vec<Point> {
    // Polyline.cpp:625
    let mut ret = points.to_vec();
    // Polyline.cpp:626
    let n = points.len();
    // Polyline.cpp:627-629
    for j in 0..(n - 1) {
        ret[j] = points[(idx + j) % (n - 1)];
    }
    // Polyline.cpp:630
    ret[n - 1] = ret[0];
    // Polyline.cpp:631
    ret
}

/// Polyline.cpp:634-664 `ThickPolyline ThickPolyline::rebase_at(size_t idx)`.
/// Returns `(points, width)`. (Caller has already verified `is_closed()`.)
fn thick_polyline_rebase_at(
    points: &[Point],
    width: &[CoordF],
    idx: usize,
) -> (Vec<Point>, Vec<CoordF>) {
    // Polyline.cpp:640
    let ret_points = polyline_rebase_at(points, idx);
    // Polyline.cpp:641
    let n = points.len();
    // Polyline.cpp:642
    let mut ret_width = vec![0.0_f64; 2 * n - 2];

    // Polyline.cpp:644-648
    let get_in_width = |i: usize| -> CoordF {
        if i == 0 {
            return width[0];
        }
        if i == n - 1 {
            return *width.last().unwrap();
        }
        width[2 * i - 1]
    };
    // Polyline.cpp:649-653
    let get_out_width = |i: usize| -> CoordF {
        if i == 0 {
            return width[0];
        }
        if i == n - 1 {
            return *width.last().unwrap();
        }
        width[2 * i]
    };

    // Polyline.cpp:655
    ret_width[0] = get_out_width(idx % (n - 1));
    // Polyline.cpp:656-660
    for j in 1..(n - 1) {
        let i = (idx + j) % (n - 1);
        ret_width[2 * j - 1] = get_in_width(i);
        ret_width[2 * j] = get_out_width(i);
    }

    // Polyline.cpp:662
    ret_width[2 * n - 3] = ret_width[0];
    // Polyline.cpp:663
    (ret_points, ret_width)
}

// =============================================================================
// Free functions
// =============================================================================

/// FillFloatingConcentric.cpp:61-203
/// `static ExtrusionPaths floating_thick_polyline_to_extrusion_paths(const FloatingThickPolyline& floating_polyline, ExtrusionRole role, const Flow& flow, const float tolerance)`
//BBS: new function to filter width to avoid too fragmented segments
pub fn floating_thick_polyline_to_extrusion_paths(
    floating_polyline: &FloatingThickPolyline,
    role: ExtrusionRole,
    flow: &Flow,
    tolerance: f32,
) -> Vec<ExtrusionPath> {
    // FillFloatingConcentric.cpp:64
    let mut paths: Vec<ExtrusionPath> = Vec::new();
    // FillFloatingConcentric.cpp:65
    let mut path = ExtrusionPath::new(role);
    // FillFloatingConcentric.cpp:66
    let mut lines: FloatingThicklines = floating_polyline.floating_thicklines();

    // FillFloatingConcentric.cpp:68
    let mut start_index: usize = 0;
    // FillFloatingConcentric.cpp:69
    let mut max_width: f64 = 0.0;
    let mut min_width: f64 = 0.0;

    // FillFloatingConcentric.cpp:71-76
    // `auto set_flow_for_path = [&flow](ExtrusionPath& path, double width) { ... };`
    let set_flow_for_path = |path: &mut ExtrusionPath, width: f64| {
        // Flow new_flow = flow.with_width(unscale<float>(width) + flow.height() * float(1. - 0.25 * PI));
        // C++ `unscale<float>(width)` == `float(width) * float(SCALING_FACTOR)` with
        // NO integer truncation of `width` (which here is a fractional averaged
        // width). The whole sum is computed in `float` (f32) in C++, so mirror that.
        let unscaled_w = (width as f32) / (crate::SCALING_FACTOR as f32);
        // C++ `float(1. - 0.25 * PI)`: the inner expression is evaluated in double,
        // then cast to float.
        let arg = unscaled_w + (flow.height() as f32) * ((1.0 - 0.25 * PI) as f32);
        let new_flow: Flow = flow
            .with_width(arg as f64)
            .expect("Flow::with_width");
        path.mm3_per_mm = new_flow.mm3_per_mm_unchecked();
        path.width = new_flow.width();
        path.height = new_flow.height();
    };

    // FillFloatingConcentric.cpp:78-82
    // `auto append_path_and_reset = [...](double& length, double& sum, ExtrusionPath& path){ ... };`
    let append_path_and_reset =
        |length: &mut f64, sum: &mut f64, path: &mut ExtrusionPath, paths: &mut Vec<ExtrusionPath>| {
            *length = 0.0;
            *sum = 0.0;
            paths.push(std::mem::replace(path, ExtrusionPath::new(role)));
        };

    // FillFloatingConcentric.cpp:84
    let mut i: i32 = 0;
    while i < lines.len() as i32 {
        // FillFloatingConcentric.cpp:85
        let line = lines[i as usize].clone();

        // FillFloatingConcentric.cpp:87-89
        if i == 0 {
            max_width = line.a_width;
            min_width = line.a_width;
        }

        // FillFloatingConcentric.cpp:91
        let line_len: CoordF = line.length();
        // FillFloatingConcentric.cpp:92
        if line_len < SCALED_EPSILON {
            i += 1;
            continue;
        }

        // FillFloatingConcentric.cpp:94
        let mut thickness_delta: f64 =
            (max_width - line.b_width).abs().max((min_width - line.b_width).abs());
        //BBS: has large difference in width
        // FillFloatingConcentric.cpp:96
        if thickness_delta > tolerance as f64 {
            //BBS: 1 generate path from start_index to i(not included)
            // FillFloatingConcentric.cpp:98
            if start_index != i as usize {
                // FillFloatingConcentric.cpp:99
                path = ExtrusionPath::new(role);
                // FillFloatingConcentric.cpp:100
                let mut length: f64 = 0.0;
                let mut sum: f64 = 0.0;
                // FillFloatingConcentric.cpp:101
                let mut is_floating = false;
                // FillFloatingConcentric.cpp:102
                for idx in start_index..(i as usize) {
                    // FillFloatingConcentric.cpp:103
                    let curr_floating = lines[idx].is_a_floating && lines[idx].is_b_floating;
                    // FillFloatingConcentric.cpp:104
                    if curr_floating != is_floating && length != 0.0 {
                        // FillFloatingConcentric.cpp:105
                        path.polyline.append_point(lines[idx].a);
                        // FillFloatingConcentric.cpp:106-107
                        if is_floating {
                            path.set_customize_flag(CustomizeFlag::FloatingVerticalShell);
                        }
                        // FillFloatingConcentric.cpp:108
                        set_flow_for_path(&mut path, sum / length);
                        // FillFloatingConcentric.cpp:109
                        append_path_and_reset(&mut length, &mut sum, &mut path, &mut paths);
                    }
                    // FillFloatingConcentric.cpp:111
                    is_floating = curr_floating;

                    // FillFloatingConcentric.cpp:113
                    let line_length = lines[idx].length();
                    // FillFloatingConcentric.cpp:114
                    length += line_length;
                    // FillFloatingConcentric.cpp:115
                    sum += line_length * (lines[idx].a_width + lines[idx].b_width) * 0.5;
                    // FillFloatingConcentric.cpp:116
                    path.polyline.append_point(lines[idx].a);
                }
                // FillFloatingConcentric.cpp:118
                path.polyline.append_point(lines[i as usize].a);
                // FillFloatingConcentric.cpp:119
                if length > SCALED_EPSILON {
                    // FillFloatingConcentric.cpp:120-121
                    if lines[i as usize].is_a_floating && lines[i as usize].is_b_floating {
                        path.set_customize_flag(CustomizeFlag::FloatingVerticalShell);
                    }
                    // FillFloatingConcentric.cpp:122
                    set_flow_for_path(&mut path, sum / length);
                    // FillFloatingConcentric.cpp:123
                    paths.push(std::mem::replace(&mut path, ExtrusionPath::new(role)));
                }
            }

            // FillFloatingConcentric.cpp:127
            start_index = i as usize;
            // FillFloatingConcentric.cpp:128
            max_width = line.a_width;
            // FillFloatingConcentric.cpp:129
            min_width = line.a_width;

            //BBS: 2 handle the i-th segment
            // FillFloatingConcentric.cpp:132
            thickness_delta = (line.a_width - line.b_width).abs();
            // FillFloatingConcentric.cpp:133
            if thickness_delta > tolerance as f64 {
                // FillFloatingConcentric.cpp:134
                let segments: u32 = (thickness_delta / tolerance as f64).ceil() as u32;
                // FillFloatingConcentric.cpp:135
                let seg_len: CoordF = line_len / segments as f64;
                // FillFloatingConcentric.cpp:136
                let mut pp: Vec<Point> = Vec::new();
                // FillFloatingConcentric.cpp:137
                let mut width: Vec<CoordF> = Vec::new();
                {
                    // FillFloatingConcentric.cpp:139
                    pp.push(line.a);
                    // FillFloatingConcentric.cpp:140
                    width.push(line.a_width);
                    // FillFloatingConcentric.cpp:141
                    for j in 1..(segments as usize) {
                        // FillFloatingConcentric.cpp:142
                        // pp.push_back((line.a.cast<double>() + (line.b - line.a).cast<double>().normalized() * (j * seg_len)).cast<coord_t>());
                        let ax = line.a.x as f64;
                        let ay = line.a.y as f64;
                        let dx = (line.b.x - line.a.x) as f64;
                        let dy = (line.b.y - line.a.y) as f64;
                        let dnorm = (dx * dx + dy * dy).sqrt();
                        let nx = dx / dnorm;
                        let ny = dy / dnorm;
                        let dist = j as f64 * seg_len;
                        pp.push(Point::new(
                            (ax + nx * dist) as Coord,
                            (ay + ny * dist) as Coord,
                        ));

                        // FillFloatingConcentric.cpp:144
                        let w: CoordF = line.a_width
                            + (j as f64 * seg_len) * (line.b_width - line.a_width) / line_len;
                        // FillFloatingConcentric.cpp:145-146
                        width.push(w);
                        width.push(w);
                    }
                    // FillFloatingConcentric.cpp:148
                    pp.push(line.b);
                    // FillFloatingConcentric.cpp:149
                    width.push(line.b_width);

                    // FillFloatingConcentric.cpp:151-152
                    debug_assert!(pp.len() == segments as usize + 1);
                    debug_assert!(width.len() == segments as usize * 2);
                }

                // delete this line and insert new ones
                // FillFloatingConcentric.cpp:156
                lines.remove(i as usize);
                // FillFloatingConcentric.cpp:157-160
                for j in 0..(segments as usize) {
                    let new_line = FloatingThickline::new(
                        pp[j],
                        pp[j + 1],
                        width[2 * j],
                        width[2 * j + 1],
                        line.is_a_floating,
                        line.is_b_floating,
                    );
                    lines.insert(i as usize + j, new_line);
                }
                // FillFloatingConcentric.cpp:161  `--i;`
                i -= 1;
                // FillFloatingConcentric.cpp:162  `continue;`
                // C++ `continue` jumps to the for-loop's `++i`; replicate that
                // increment here (net: `i` unchanged), then `continue` skips the
                // unconditional `i += 1` at the bottom of this loop body.
                i += 1;
                continue;
            }
        }
        //BBS: just update the max and min width and continue
        // FillFloatingConcentric.cpp:166-169
        else {
            max_width = max_width.max(line.a_width.max(line.b_width));
            min_width = min_width.min(line.a_width.min(line.b_width));
        }

        i += 1;
    }

    //BBS: handle the remaining segment
    // FillFloatingConcentric.cpp:172
    let final_size = lines.len();
    // FillFloatingConcentric.cpp:173
    if start_index < final_size {
        // FillFloatingConcentric.cpp:174
        path = ExtrusionPath::new(role);
        // FillFloatingConcentric.cpp:175
        let mut length: f64 = 0.0;
        let mut sum: f64 = 0.0;
        // FillFloatingConcentric.cpp:176
        let mut is_floating = false;
        // FillFloatingConcentric.cpp:177
        for idx in start_index..final_size {
            // FillFloatingConcentric.cpp:178
            let curr_floating = lines[idx].is_a_floating && lines[idx].is_b_floating;
            // FillFloatingConcentric.cpp:179
            if curr_floating != is_floating && length != 0.0 {
                // FillFloatingConcentric.cpp:180
                path.polyline.append_point(lines[idx].a);
                // FillFloatingConcentric.cpp:181-182
                if is_floating {
                    path.set_customize_flag(CustomizeFlag::FloatingVerticalShell);
                }
                // FillFloatingConcentric.cpp:183
                set_flow_for_path(&mut path, sum / length);
                // FillFloatingConcentric.cpp:184
                append_path_and_reset(&mut length, &mut sum, &mut path, &mut paths);
            }
            // FillFloatingConcentric.cpp:186
            is_floating = curr_floating;
            // FillFloatingConcentric.cpp:187
            let line_length = lines[idx].length();
            // FillFloatingConcentric.cpp:188
            length += line_length;
            // FillFloatingConcentric.cpp:189
            sum += line_length * (lines[idx].a_width + lines[idx].b_width) * 0.5;
            // FillFloatingConcentric.cpp:190
            path.polyline.append_point(lines[idx].a);
        }
        // FillFloatingConcentric.cpp:193
        path.polyline.append_point(lines[final_size - 1].b);
        // FillFloatingConcentric.cpp:194
        if length > SCALED_EPSILON {
            // FillFloatingConcentric.cpp:195-196
            if lines[final_size - 1].is_a_floating && lines[final_size - 1].is_b_floating {
                path.set_customize_flag(CustomizeFlag::FloatingVerticalShell);
            }
            // FillFloatingConcentric.cpp:197
            set_flow_for_path(&mut path, sum / length);
            // FillFloatingConcentric.cpp:198
            paths.push(std::mem::replace(&mut path, ExtrusionPath::new(role)));
        }
    }

    // FillFloatingConcentric.cpp:202
    if crate::probe_enabled("FVSPROBE") {
        let all = floating_polyline.floating_thicklines();
        let nfl = all.iter().filter(|l| l.is_a_floating && l.is_b_floating).count();
        let fp = floating_polyline
            .points
            .first()
            .copied()
            .unwrap_or_else(|| crate::geometry::Point::new(0, 0));
        eprintln!(
            "FVSPROBE n={} fl={} out={} p0={},{}",
            all.len(),
            nfl,
            paths.len(),
            fp.x,
            fp.y
        );
    }
    paths
}

/// FillFloatingConcentric.cpp:205-243
/// `double interpolate_width(const ZPath& path, const ThickPolyline& line, const int subject_idx_range, const int default_width, size_t idx)`
pub fn interpolate_width(
    path: &ZPath,
    line: &ThickPolyline,
    subject_idx_range: i32,
    default_width: i32,
    idx: usize,
) -> f64 {
    // FillFloatingConcentric.cpp:211
    let mut prev_idx: i32 = idx as i32;
    // FillFloatingConcentric.cpp:212-213
    while prev_idx >= 0
        && (path[prev_idx as usize].2 < 0 || path[prev_idx as usize].2 >= subject_idx_range as i64)
    {
        prev_idx -= 1;
    }

    // FillFloatingConcentric.cpp:215
    let mut next_idx: i32 = idx as i32;
    // FillFloatingConcentric.cpp:216-217
    while (next_idx as usize) < path.len()
        && (path[next_idx as usize].2 < 0 || path[next_idx as usize].2 >= subject_idx_range as i64)
    {
        next_idx += 1;
    }

    // FillFloatingConcentric.cpp:219-220
    let width_prev: f64;
    let width_next: f64;
    // FillFloatingConcentric.cpp:221-227
    if prev_idx < 0 {
        width_prev = default_width as f64;
    } else {
        let prev_z_idx = path[prev_idx as usize].2 as usize;
        width_prev = thick_polyline_get_width_at(line, prev_z_idx);
    }

    // FillFloatingConcentric.cpp:229-235
    if next_idx as usize >= path.len() {
        width_next = default_width as f64;
    } else {
        let next_z_idx = path[next_idx as usize].2 as usize;
        width_next = thick_polyline_get_width_at(line, next_z_idx);
    }
    // FillFloatingConcentric.cpp:236
    let prev = Point::new(path[prev_idx as usize].0, path[prev_idx as usize].1);
    // FillFloatingConcentric.cpp:237
    let next = Point::new(path[next_idx as usize].0, path[next_idx as usize].1);
    // FillFloatingConcentric.cpp:238
    let curr = Point::new(path[idx].0, path[idx].1);
    // FillFloatingConcentric.cpp:239
    let d_total = (((next.x - prev.x) as f64).powi(2) + ((next.y - prev.y) as f64).powi(2)).sqrt();
    // FillFloatingConcentric.cpp:240
    let d_curr = (((curr.x - prev.x) as f64).powi(2) + ((curr.y - prev.y) as f64).powi(2)).sqrt();
    // FillFloatingConcentric.cpp:241
    let t = if d_total > 0.0 { d_curr / d_total } else { 0.0 };
    // FillFloatingConcentric.cpp:242
    (1.0 - t) * width_prev + t * width_next
}

/// `ThickPolyline::get_width_at(size_t point_idx)` — Polyline.cpp:666-671.
/// The crate's `geometry::ThickPolyline` stores `widths` in the C++ 2-per-segment
/// layout (see `fill_concentric.rs`), so this mirrors the C++ getter exactly.
fn thick_polyline_get_width_at(line: &ThickPolyline, point_idx: usize) -> CoordF {
    // Polyline.cpp:668-669
    if point_idx < 2 {
        return line.widths[point_idx];
    }
    // Polyline.cpp:670
    line.widths[2 * point_idx - 1]
}

/// FillFloatingConcentric.cpp:245-387
/// `FloatingThickPolyline merge_lines(ZPaths lines, const std::vector<bool>& mark_flags, const ThickPolyline& line, const int subject_idx_range ,const int default_width)`
pub fn merge_lines(
    lines: Vec<ZPath>,
    mark_flags: &[bool],
    line: &ThickPolyline,
    subject_idx_range: i32,
    default_width: i32,
) -> FloatingThickPolyline {
    // FillFloatingConcentric.cpp:247-248
    // using PathFlag = std::vector<bool>;
    // using PathFlags = std::vector<PathFlag>;
    let mut lines = lines;

    // FillFloatingConcentric.cpp:250
    let mut used: Vec<bool> = vec![false; lines.len()];
    // FillFloatingConcentric.cpp:251
    let mut merged_paths: Vec<ZPath> = Vec::new();
    // FillFloatingConcentric.cpp:252
    let mut merged_marks: Vec<Vec<bool>> = Vec::new();

    // FillFloatingConcentric.cpp:254-257
    // `auto update_path_flag = [](PathFlag& mark_flags, const ZPath& path, bool mark) {...};`
    let update_path_flag = |mark_flags: &mut Vec<bool>, path: &ZPath, mark: bool| {
        for _p in path.iter() {
            mark_flags.push(mark);
        }
    };

    // FillFloatingConcentric.cpp:259-260
    let mut start_z_map: HashMap<i64, HashSet<usize>> = HashMap::new();
    let mut end_z_map: HashMap<i64, HashSet<usize>> = HashMap::new();

    // FillFloatingConcentric.cpp:262-269
    for idx in 0..lines.len() {
        // FillFloatingConcentric.cpp:263-266
        if lines[idx].is_empty() {
            used[idx] = true;
            continue;
        }
        // FillFloatingConcentric.cpp:267
        start_z_map.entry(lines[idx].first().unwrap().2).or_default().insert(idx);
        // FillFloatingConcentric.cpp:268
        end_z_map.entry(lines[idx].last().unwrap().2).or_default().insert(idx);
    }

    // FillFloatingConcentric.cpp:271-282
    // `auto remove_from_map = [&start_z_map, &end_z_map, &lines](size_t idx) {...};`
    let remove_from_map =
        |start_z_map: &mut HashMap<i64, HashSet<usize>>,
         end_z_map: &mut HashMap<i64, HashSet<usize>>,
         lines: &[ZPath],
         idx: usize| {
            // FillFloatingConcentric.cpp:272-273
            if lines[idx].is_empty() {
                return;
            }
            // FillFloatingConcentric.cpp:274-275
            let start_z = lines[idx].first().unwrap().2;
            let end_z = lines[idx].last().unwrap().2;
            // FillFloatingConcentric.cpp:276-278
            if let Some(s) = start_z_map.get_mut(&start_z) {
                s.remove(&idx);
                if s.is_empty() {
                    start_z_map.remove(&start_z);
                }
            }
            // FillFloatingConcentric.cpp:279-281
            if let Some(e) = end_z_map.get_mut(&end_z) {
                e.remove(&idx);
                if e.is_empty() {
                    end_z_map.remove(&end_z);
                }
            }
        };

    // FillFloatingConcentric.cpp:284-360
    for idx in 0..lines.len() {
        // FillFloatingConcentric.cpp:285-286
        if used[idx] {
            continue;
        }
        // FillFloatingConcentric.cpp:287
        let mut curr_path: ZPath = lines[idx].clone();
        // FillFloatingConcentric.cpp:288
        let mut curr_mark: Vec<bool> = Vec::new();
        // FillFloatingConcentric.cpp:289
        update_path_flag(&mut curr_mark, &curr_path, mark_flags[idx]);
        // FillFloatingConcentric.cpp:290
        used[idx] = true;
        // FillFloatingConcentric.cpp:291
        remove_from_map(&mut start_z_map, &mut end_z_map, &lines, idx);

        // FillFloatingConcentric.cpp:293
        let mut merged;
        // FillFloatingConcentric.cpp:294
        loop {
            // FillFloatingConcentric.cpp:295
            merged = false;
            // FillFloatingConcentric.cpp:296
            let curr_end = curr_path.last().unwrap().2;
            // FillFloatingConcentric.cpp:297
            let curr_start = curr_path.first().unwrap().2;

            // search after
            // FillFloatingConcentric.cpp:300-318
            {
                // FillFloatingConcentric.cpp:301-308
                if let Some(j) = start_z_map.get(&curr_end).and_then(|s| s.iter().min().copied()) /* R99 determinism: min index (was .next() — HashSet<usize> RandomState order); C++ picks *unordered_set.begin() (arbitrary) */ {
                    remove_from_map(&mut start_z_map, &mut end_z_map, &lines, j);
                    curr_path.extend(lines[j].iter().copied());
                    update_path_flag(&mut curr_mark, &lines[j], mark_flags[j]);
                    used[j] = true;
                    merged = true;
                }
                // FillFloatingConcentric.cpp:309-317
                else if let Some(j) = end_z_map.get(&curr_end).and_then(|s| s.iter().min().copied()) /* R99 determinism: min index (was .next() — HashSet<usize> RandomState order); C++ picks *unordered_set.begin() (arbitrary) */
                {
                    remove_from_map(&mut start_z_map, &mut end_z_map, &lines, j);
                    lines[j].reverse();
                    curr_path.extend(lines[j].iter().copied());
                    update_path_flag(&mut curr_mark, &lines[j], mark_flags[j]);
                    used[j] = true;
                    merged = true;
                }
            }

            // FillFloatingConcentric.cpp:320-321
            if merged {
                continue;
            }

            //search before
            // FillFloatingConcentric.cpp:324-354
            {
                // FillFloatingConcentric.cpp:325-338
                if let Some(j) = end_z_map.get(&curr_start).and_then(|s| s.iter().min().copied()) /* R99 determinism: min index (was .next() — HashSet<usize> RandomState order); C++ picks *unordered_set.begin() (arbitrary) */ {
                    remove_from_map(&mut start_z_map, &mut end_z_map, &lines, j);
                    let mut new_path: ZPath = lines[j].clone();
                    let mut new_mark: Vec<bool> = Vec::new();
                    update_path_flag(&mut new_mark, &new_path, mark_flags[j]);

                    new_path.extend(curr_path.iter().copied());
                    new_mark.extend(curr_mark.iter().copied());
                    curr_path = new_path;
                    curr_mark = new_mark;
                    used[j] = true;
                    merged = true;
                }
                // FillFloatingConcentric.cpp:339-353
                else if let Some(j) =
                    start_z_map.get(&curr_start).and_then(|s| s.iter().min().copied()) /* R99 determinism: min index (was .next() — HashSet<usize> RandomState order); C++ picks *unordered_set.begin() (arbitrary) */
                {
                    remove_from_map(&mut start_z_map, &mut end_z_map, &lines, j);
                    let mut new_path: ZPath = lines[j].clone();
                    new_path.reverse();
                    let mut new_mark: Vec<bool> = Vec::new();
                    update_path_flag(&mut new_mark, &new_path, mark_flags[j]);

                    new_path.extend(curr_path.iter().copied());
                    new_mark.extend(curr_mark.iter().copied());
                    curr_path = new_path;
                    curr_mark = new_mark;
                    used[j] = true;
                    merged = true;
                }
            }

            // FillFloatingConcentric.cpp:356
            if !merged {
                break;
            }
        }

        // FillFloatingConcentric.cpp:358
        merged_paths.push(curr_path);
        // FillFloatingConcentric.cpp:359
        merged_marks.push(curr_mark);
    }

    // FillFloatingConcentric.cpp:362
    debug_assert!(merged_marks.len() == 1);

    // FillFloatingConcentric.cpp:364
    let mut res = FloatingThickPolyline::default();

    // FillFloatingConcentric.cpp:366
    let valid_path = &merged_paths[0];
    // FillFloatingConcentric.cpp:367
    let valid_mark = &merged_marks[0];

    // FillFloatingConcentric.cpp:369-382
    for idx in 0..valid_path.len() {
        // FillFloatingConcentric.cpp:370
        let zvalue = valid_path[idx].2 as i32;
        // FillFloatingConcentric.cpp:371
        res.points.push(Point::new(valid_path[idx].0, valid_path[idx].1));
        // FillFloatingConcentric.cpp:372
        res.is_floating.push(valid_mark[idx]);
        // FillFloatingConcentric.cpp:373
        if 0 <= zvalue && zvalue < subject_idx_range {
            // FillFloatingConcentric.cpp:374
            res.width
                .push(thick_polyline_get_width_at(line, prev_idx_modulo(zvalue as usize, line.points.len())));
            // FillFloatingConcentric.cpp:375
            res.width.push(thick_polyline_get_width_at(line, zvalue as usize));
        } else {
            // FillFloatingConcentric.cpp:378
            let width =
                interpolate_width(valid_path, line, subject_idx_range, default_width, idx);
            // FillFloatingConcentric.cpp:379-380
            res.width.push(width);
            res.width.push(width);
        }
    }
    // FillFloatingConcentric.cpp:383
    // res.width = std::vector<coordf_t>(res.width.begin() + 1, res.width.end()-1);
    res.width = res.width[1..res.width.len() - 1].to_vec();
    // FillFloatingConcentric.cpp:384
    debug_assert!(res.width.len() == 2 * res.points.len() - 2);

    // FillFloatingConcentric.cpp:386
    res
}

/// FillFloatingConcentric.cpp:389-489
/// `FloatingThickPolyline detect_floating_line(const ThickPolyline& line, const ExPolygons& floating_areas, const double default_width, bool force_no_detect)`
///
/// The Z-clipper half (the ClipperLib_Z::Clipper with the custom ZFillFunction +
/// ctIntersection/ctDifference) is provided by `clipper_z::detect_floating`
/// (crates/clipper-z-sys); `merge_lines` (above) is the back half.
pub fn detect_floating_line(
    line: &ThickPolyline,
    floating_areas: &ExPolygons,
    default_width: f64,
    force_no_detect: bool,
) -> FloatingThickPolyline {
    // FillFloatingConcentric.cpp:391-402 — early out: no floating overlap.
    {
        // Polyline polyline = line; (the C++ slices the ThickPolyline to a Polyline)
        let bbox_line = BoundingBox::from_points(&line.points);
        let bbox_area = get_extents(floating_areas);
        // FillFloatingConcentric.cpp:395
        // if (force_no_detect || !bbox_area.overlap(bbox_line) || intersection_pl(polyline, floating_areas).empty())
        let polyline = crate::geometry::Polyline::from_points(line.points.clone());
        if force_no_detect
            || !bbox_area.intersects(&bbox_line)
            || crate::clipper_utils::intersection_pl(
                std::slice::from_ref(&polyline),
                floating_areas,
            )
            .is_empty()
        {
            // FillFloatingConcentric.cpp:396-401
            let mut res = FloatingThickPolyline::default();
            res.width = line.widths.clone();
            res.points = line.points.clone();
            res.is_floating.resize(res.points.len(), false);
            return res;
        }
    }

    // FillFloatingConcentric.cpp:406-411 — the hash is baked into the shim's
    // ZFillFunction; nothing to do here.

    // FillFloatingConcentric.cpp:413-417 — subject ZPath: (x, y, vertex_index).
    let mut idx: i64 = 0;
    let mut subject_path: ZPath = Vec::with_capacity(line.points.len());
    for p in &line.points {
        subject_path.push((p.x, p.y, idx));
        idx += 1;
    }
    // FillFloatingConcentric.cpp:419 — subject_idx_range = idx;
    let subject_idx_range = idx as i32;

    // FillFloatingConcentric.cpp:420-427 — clip ZPaths: floating-area polygons,
    // z = a per-vertex index continuing past subject_idx_range.
    let floating_polygons = crate::geometry::to_polygons(floating_areas);
    let mut clip_paths: Vec<ZPath> = Vec::with_capacity(floating_polygons.len());
    for poly in &floating_polygons {
        let mut zp: ZPath = Vec::with_capacity(poly.points.len());
        for p in &poly.points {
            zp.push((p.x, p.y, idx));
            idx += 1;
        }
        clip_paths.push(zp);
    }

    // FillFloatingConcentric.cpp:457-475 — run both passes (ctIntersection +
    // ctDifference) via the Z-clipper; to_merge = diff_out ++ intersect_out with
    // floating_flags true for the intersect tail.
    let (to_merge, num_diff_paths) =
        crate::clipper_z::detect_floating(&subject_path, &clip_paths, subject_idx_range);
    let mut floating_flags: Vec<bool> = vec![false; to_merge.len()];
    for f in floating_flags.iter_mut().skip(num_diff_paths) {
        *f = true;
    }

    // FillFloatingConcentric.cpp:489 — merge_lines(to_merge, floating_flags, line, subject_idx_range, default_width)
    merge_lines(
        to_merge,
        &floating_flags,
        line,
        subject_idx_range,
        default_width as i32,
    )
}

/// FillFloatingConcentric.cpp:495-502
/// `int start_none_floating_idx(int idx, const std::vector<int>& none_floating_count)`
pub fn start_none_floating_idx(idx: i32, none_floating_count: &[i32]) -> i32 {
    // FillFloatingConcentric.cpp:497
    let backtrace_idx = idx - none_floating_count[idx as usize] + 1;
    // FillFloatingConcentric.cpp:498-501
    if backtrace_idx >= 0 {
        backtrace_idx
    } else {
        none_floating_count.len() as i32 + backtrace_idx
    }
}

/// FillFloatingConcentric.cpp:504-560
/// `template<typename PointContainer> void get_none_floating_prefix(const PointContainer& container, const ExPolygons& floating_areas, const Polygons& sparse_polys, std::vector<double>& none_floating_length, std::vector<int>& none_floating_count)`
///
/// `PointContainer::points` is passed directly as `points`.
pub fn get_none_floating_prefix(
    points: &[Point],
    floating_areas: &ExPolygons,
    sparse_polys: &Polygons,
    none_floating_length: &mut Vec<f64>,
    none_floating_count: &mut Vec<i32>,
) {
    // FillFloatingConcentric.cpp:507-508
    *none_floating_length = vec![0.0; points.len()];
    *none_floating_count = vec![0; points.len()];

    // FillFloatingConcentric.cpp:510-512
    let mut floating_bboxs: Vec<BoundingBox> = Vec::new();
    for fa in floating_areas.iter() {
        floating_bboxs.push(get_extents(std::slice::from_ref(fa)));
    }
    // FillFloatingConcentric.cpp:513-515
    let mut sparse_bboxs: Vec<BoundingBox> = Vec::new();
    for sp in sparse_polys.iter() {
        sparse_bboxs.push(polygon_get_extents(sp));
    }

    // FillFloatingConcentric.cpp:517-532
    // `auto point_in_floating_area = [...](const Point& p)->bool {...};`
    let point_in_floating_area = |p: &Point| -> bool {
        // FillFloatingConcentric.cpp:518-523
        for idx in 0..sparse_polys.len() {
            if !sparse_bboxs[idx].contains_point(p) {
                continue;
            }
            if sparse_polys[idx].contains(p) {
                return false;
            }
        }
        // FillFloatingConcentric.cpp:524-529
        for idx in 0..floating_areas.len() {
            if !floating_bboxs[idx].contains_point(p) {
                continue;
            }
            if floating_areas[idx].contains_point(p) {
                return true;
            }
        }
        // FillFloatingConcentric.cpp:531
        false
    };

    // FillFloatingConcentric.cpp:534-550
    for idx in 0..points.len() {
        // FillFloatingConcentric.cpp:535
        let p = points[idx];
        // FillFloatingConcentric.cpp:536
        if !point_in_floating_area(&p) {
            // FillFloatingConcentric.cpp:537-540
            if idx == 0 {
                none_floating_count[idx] = 1;
            } else {
                none_floating_count[idx] = none_floating_count[idx - 1] + 1;
            }
            // FillFloatingConcentric.cpp:541-544
            if none_floating_count[idx] > 1 {
                let prev = points[prev_idx_modulo(idx, points.len())];
                none_floating_length[idx] = none_floating_length[idx - 1]
                    + (((prev.x - p.x) as f64).powi(2) + ((prev.y - p.y) as f64).powi(2)).sqrt();
            } else {
                none_floating_length[idx] = 0.0;
            }
        } else {
            // FillFloatingConcentric.cpp:547-548
            none_floating_length[idx] = 0.0;
            none_floating_count[idx] = 0;
        }
    }

    // FillFloatingConcentric.cpp:552-559
    if *none_floating_count.last().unwrap() > 0 {
        for idx in 0..points.len() {
            // FillFloatingConcentric.cpp:554-555
            if none_floating_count[idx] == 0 {
                break;
            }
            // FillFloatingConcentric.cpp:556
            none_floating_count[idx] = none_floating_count[prev_idx_modulo(idx, points.len())] + 1;
            // FillFloatingConcentric.cpp:557
            let prev = points[prev_idx_modulo(idx, points.len())];
            let curr = points[idx];
            none_floating_length[idx] = none_floating_length[prev_idx_modulo(idx, points.len())]
                + (((prev.x - curr.x) as f64).powi(2) + ((prev.y - curr.y) as f64).powi(2)).sqrt();
        }
    }
}

/// FillFloatingConcentric.cpp:562-577
/// `template<typename PointContainer> int get_best_loop_start(const PointContainer& container, const ExPolygons& floating_areas, const Polygons& sparse_polys)`
pub fn get_best_loop_start(
    points: &[Point],
    floating_areas: &ExPolygons,
    sparse_polys: &Polygons,
) -> i32 {
    // FillFloatingConcentric.cpp:564-565
    let mut none_floating_length: Vec<f64> = Vec::new();
    let mut none_floating_count: Vec<i32> = Vec::new();

    // FillFloatingConcentric.cpp:567
    let floating_bbox = get_extents(floating_areas);
    // FillFloatingConcentric.cpp:568
    let poly_bbox = BoundingBox::from_points(points);

    // FillFloatingConcentric.cpp:570-571
    if !poly_bbox.intersects(&floating_bbox) {
        return 0;
    }

    // FillFloatingConcentric.cpp:573
    let clipped_sparse_polys =
        clip_clipper_polygons_with_subject_bbox(sparse_polys, &poly_bbox);
    // FillFloatingConcentric.cpp:574
    get_none_floating_prefix(
        points,
        floating_areas,
        &clipped_sparse_polys,
        &mut none_floating_length,
        &mut none_floating_count,
    );
    // FillFloatingConcentric.cpp:575
    // `int best_idx = std::distance(begin, std::max_element(begin, end));`
    let best_idx = max_element_index(&none_floating_length) as i32;
    // FillFloatingConcentric.cpp:576
    start_none_floating_idx(best_idx, &none_floating_count)
}

/// FillFloatingConcentric.cpp:579-601
/// `template<typename PointContainer> std::vector<int> get_loop_start_candidates(const PointContainer& container, const ExPolygons& floating_areas, const Polygons& sparse_polys)`
pub fn get_loop_start_candidates(
    points: &[Point],
    floating_areas: &ExPolygons,
    sparse_polys: &Polygons,
) -> Vec<i32> {
    // FillFloatingConcentric.cpp:582-583
    let mut none_floating_length: Vec<f64> = Vec::new();
    let mut none_floating_count: Vec<i32> = Vec::new();

    // FillFloatingConcentric.cpp:585
    let floating_bbox = get_extents(floating_areas);
    // FillFloatingConcentric.cpp:586
    let poly_bbox = BoundingBox::from_points(points);
    // FillFloatingConcentric.cpp:587
    let mut candidate_list: Vec<i32> = Vec::new();

    // FillFloatingConcentric.cpp:589-593
    if !poly_bbox.intersects(&floating_bbox) {
        candidate_list.resize(points.len(), 0);
        for (i, c) in candidate_list.iter_mut().enumerate() {
            *c = i as i32;
        }
        return candidate_list;
    }
    // FillFloatingConcentric.cpp:594
    let clipped_sparse_polys =
        clip_clipper_polygons_with_subject_bbox(sparse_polys, &poly_bbox);
    // FillFloatingConcentric.cpp:595
    get_none_floating_prefix(
        points,
        floating_areas,
        &clipped_sparse_polys,
        &mut none_floating_length,
        &mut none_floating_count,
    );
    // FillFloatingConcentric.cpp:596-599
    for idx in 0..none_floating_length.len() {
        if none_floating_length[idx] > 0.0 {
            candidate_list.push(start_none_floating_idx(idx as i32, &none_floating_count));
        }
    }
    // FillFloatingConcentric.cpp:600
    candidate_list
}

/// FillFloatingConcentric.cpp:604-679
/// `void smooth_floating_line(FloatingThickPolyline& line,coord_t max_gap_threshold, coord_t min_floating_threshold)`
pub fn smooth_floating_line(
    line: &mut FloatingThickPolyline,
    max_gap_threshold: Coord,
    min_floating_threshold: Coord,
) {
    // FillFloatingConcentric.cpp:606-607
    if line.empty() {
        return;
    }
    // FillFloatingConcentric.cpp:608-612
    // struct LineParts { int start; int end; bool is_floating; };
    #[derive(Clone, Copy)]
    struct LineParts {
        start: i32,
        end: i32,
        is_floating: bool,
    }

    // FillFloatingConcentric.cpp:614-627
    // `auto build_line_parts = [&](const FloatingThickPolyline& line)->std::vector<LineParts> {...};`
    let build_line_parts = |line: &FloatingThickPolyline| -> Vec<LineParts> {
        // FillFloatingConcentric.cpp:615
        let mut line_parts: Vec<LineParts> = Vec::new();
        // FillFloatingConcentric.cpp:616
        let mut current_val = line.is_floating[0];
        // FillFloatingConcentric.cpp:617
        let mut start: i32 = 0;
        // FillFloatingConcentric.cpp:618-624
        for idx in 1..line.is_floating.len() {
            if line.is_floating[idx] != current_val {
                line_parts.push(LineParts {
                    start,
                    end: (idx - 1) as i32,
                    is_floating: current_val,
                });
                current_val = line.is_floating[idx];
                start = idx as i32;
            }
        }
        // FillFloatingConcentric.cpp:625
        line_parts.push(LineParts {
            start,
            end: (line.is_floating.len() - 1) as i32,
            is_floating: current_val,
        });
        // FillFloatingConcentric.cpp:626
        line_parts
    };

    // FillFloatingConcentric.cpp:629-636
    let mut distance_prefix: Vec<f64> = vec![0.0; line.points.len()];
    for idx in 0..line.points.len() {
        if idx == 0 {
            distance_prefix[idx] = 0.0;
        } else {
            let dx = (line.points[idx].x - line.points[idx - 1].x) as f64;
            let dy = (line.points[idx].y - line.points[idx - 1].y) as f64;
            distance_prefix[idx] = distance_prefix[idx - 1] + (dx * dx + dy * dy).sqrt();
        }
    }
    // FillFloatingConcentric.cpp:637-661
    {
        // remove too small gaps
        // FillFloatingConcentric.cpp:639
        let line_parts = build_line_parts(line);
        // FillFloatingConcentric.cpp:640
        let mut gaps_to_merge: Vec<(i32, i32)> = Vec::new();

        // FillFloatingConcentric.cpp:642-654
        for i in 1..line_parts.len().saturating_sub(1) {
            // i + 1 < line_parts.size()
            if i + 1 >= line_parts.len() {
                break;
            }
            // FillFloatingConcentric.cpp:643
            let curr = line_parts[i];
            // FillFloatingConcentric.cpp:644
            if !curr.is_floating {
                // FillFloatingConcentric.cpp:645-646
                let prev = line_parts[i - 1];
                let next = line_parts[i + 1];
                // FillFloatingConcentric.cpp:647
                if prev.is_floating && next.is_floating {
                    // FillFloatingConcentric.cpp:648
                    let total_length =
                        distance_prefix[next.start as usize] - distance_prefix[prev.end as usize];
                    // FillFloatingConcentric.cpp:649-651
                    if total_length < max_gap_threshold as f64 {
                        gaps_to_merge.push((curr.start, curr.end));
                    }
                }
            }
        }

        // FillFloatingConcentric.cpp:656-660
        for gap in &gaps_to_merge {
            for i in gap.0..=gap.1 {
                line.is_floating[i as usize] = true;
            }
        }
    }

    // FillFloatingConcentric.cpp:663-678
    {
        // FillFloatingConcentric.cpp:664
        let line_parts = build_line_parts(line);
        // FillFloatingConcentric.cpp:665
        let mut segments_to_remove: Vec<(i32, i32)> = Vec::new();

        // FillFloatingConcentric.cpp:667-671
        for part in &line_parts {
            if part.is_floating
                && distance_prefix[part.end as usize] - distance_prefix[part.start as usize]
                    < min_floating_threshold as f64
            {
                segments_to_remove.push((part.start, part.end));
            }
        }

        // FillFloatingConcentric.cpp:673-677
        for seg in &segments_to_remove {
            for i in seg.0..=seg.1 {
                line.is_floating[i as usize] = false;
            }
        }
    }
}

// FillFloatingConcentric.cpp:681-730
// `FloatingThickPolylines FillFloatingConcentric::resplit_order_loops(...)`
//
// BLOCKED: depends on `detect_floating_line` (blocked, Z-clipper) and
// `EdgeGrid::Grid::has_intersecting_edges` (not ported in this crate's EdgeGrid),
// plus `print_object_config->detect_floating_vertical_shell` threaded through
// the Fill base. Port once those land.

/// FillFloatingConcentric.cpp:806-877
/// `static std::vector<const Arachne::ExtrusionLine*> toplogic_sort_extruisons(const std::vector<Arachne::ExtrusionLine*>& all_extrusions)`
///
/// Returns indices into `all_extrusions` (rather than borrowed pointers) so the
/// caller can reuse the slice without aliasing issues; the visiting order is
/// identical to the C++ pointer order.
pub fn toplogic_sort_extruisons(all_extrusions: &[&ExtrusionLine]) -> Vec<usize> {
    // FillFloatingConcentric.cpp:808
    let mut ordered_extrusions: Vec<usize> = Vec::new();
    // Find topological order with constraints from extrusions_constrains.
    // FillFloatingConcentric.cpp:810
    let mut blocked: Vec<usize> = vec![0; all_extrusions.len()];
    // FillFloatingConcentric.cpp:811
    let mut blocking: Vec<Vec<usize>> = vec![Vec::new(); all_extrusions.len()];
    // FillFloatingConcentric.cpp:812-814
    // map_extrusion_to_idx: in our port, get_region_order already returns index
    // pairs into `all_extrusions`, so this map is the identity.

    // FillFloatingConcentric.cpp:816
    let extrusions_constrains = WallToolPaths::get_region_order(all_extrusions, true);
    // FillFloatingConcentric.cpp:817-821
    for (before, after) in extrusions_constrains.iter() {
        // FillFloatingConcentric.cpp:818-820
        blocked[*after] += 1;
        blocking[*before].push(*after);
    }

    // FillFloatingConcentric.cpp:823
    let mut processed: Vec<bool> = vec![false; all_extrusions.len()];
    // FillFloatingConcentric.cpp:824
    let mut current_position: Point = if all_extrusions.is_empty() {
        Point::new(0, 0)
    } else {
        all_extrusions[0].junctions[0].p
    };
    // FillFloatingConcentric.cpp:825
    while ordered_extrusions.len() < all_extrusions.len() {
        // FillFloatingConcentric.cpp:826
        let mut best_candidate: usize = 0;
        // FillFloatingConcentric.cpp:827
        let mut best_distance_sqr: f64 = f64::MAX;
        // FillFloatingConcentric.cpp:828
        let mut is_best_closed = false;

        // FillFloatingConcentric.cpp:830
        let mut available_candidates: Vec<usize> = Vec::new();
        // FillFloatingConcentric.cpp:831-835
        for candidate in 0..all_extrusions.len() {
            if processed[candidate] || blocked[candidate] != 0 {
                continue; // Not a valid candidate.
            }
            available_candidates.push(candidate);
        }

        // FillFloatingConcentric.cpp:837-839
        available_candidates.sort_by(|a_idx, b_idx| {
            all_extrusions[*a_idx]
                .is_closed
                .cmp(&all_extrusions[*b_idx].is_closed)
        });

        // FillFloatingConcentric.cpp:841-861
        for &candidate_path_idx in available_candidates.iter() {
            // FillFloatingConcentric.cpp:842
            let path = all_extrusions[candidate_path_idx];

            // FillFloatingConcentric.cpp:844-850
            if path.junctions.is_empty() {
                // No vertices in the path. Can't find the start position then or really plan it in. Put that at the end.
                if best_distance_sqr == f64::MAX {
                    best_candidate = candidate_path_idx;
                    is_best_closed = path.is_closed;
                }
                continue;
            }

            // FillFloatingConcentric.cpp:852
            let candidate_position = path.junctions[0].p;
            // FillFloatingConcentric.cpp:853
            // `double distance_sqr = (current_position - candidate_position).cast<double>().norm();`
            let dx = (current_position.x - candidate_position.x) as f64;
            let dy = (current_position.y - candidate_position.y) as f64;
            let distance_sqr = (dx * dx + dy * dy).sqrt();
            // FillFloatingConcentric.cpp:854
            if distance_sqr < best_distance_sqr {
                // Closer than the best candidate so far.
                // FillFloatingConcentric.cpp:855
                if path.is_closed
                    || (!path.is_closed && best_distance_sqr != f64::MAX)
                    || (!path.is_closed && !is_best_closed)
                {
                    best_candidate = candidate_path_idx;
                    best_distance_sqr = distance_sqr;
                    is_best_closed = path.is_closed;
                }
            }
        }

        // FillFloatingConcentric.cpp:863
        let best_path = all_extrusions[best_candidate];
        // FillFloatingConcentric.cpp:864
        ordered_extrusions.push(best_candidate);
        // FillFloatingConcentric.cpp:865
        processed[best_candidate] = true;
        // FillFloatingConcentric.cpp:866-867
        for unlocked_idx in blocking[best_candidate].clone() {
            blocked[unlocked_idx] -= 1;
        }

        // FillFloatingConcentric.cpp:869-874
        if !best_path.junctions.is_empty() {
            // If all paths were empty, the best path is still empty. We don't upate the current position then.
            if best_path.is_closed {
                current_position = best_path.junctions[0].p; // We end where we started.
            } else {
                current_position = best_path.junctions.last().unwrap().p; // Pick the other end from where we started.
            }
        }
    }
    // FillFloatingConcentric.cpp:876
    ordered_extrusions
}

// =============================================================================
// Local helpers (not in FillFloatingConcentric.cpp; thin wrappers over crate
// primitives to express idioms used by the ported functions above).
// =============================================================================

/// `ClipperUtils::clip_clipper_polygons_with_subject_bbox(sparse_polys, poly_bbox)`
/// — ClipperUtils.cpp:144-151.
///
/// Faithful: clips each polygon's point sequence against `bbox` (the Sutherland-
/// style per-vertex side test in `clip_clipper_polygon_with_subject_bbox`), then
/// drops empties. NOT a whole-polygon bbox filter: the clipped contours feed
/// `point_in_floating_area`'s `Polygon::contains(p)` test, whose result depends
/// on the clipped geometry, so the point reduction must match C++.
fn clip_clipper_polygons_with_subject_bbox(polys: &Polygons, bbox: &BoundingBox) -> Polygons {
    crate::clipper_utils::clip_clipper_polygons_with_subject_bbox_polygons(polys, bbox)
}

/// Polygon.cpp `BoundingBox get_extents(const Polygon &poly)`.
/// (The crate's `polygon` module is private and its `get_extents` is not
/// glob-re-exported because it collides with the ExPolygon variant; this is the
/// same computation: `BoundingBox(poly.points)`.)
fn polygon_get_extents(poly: &Polygon) -> BoundingBox {
    BoundingBox::from_points(&poly.points)
}

/// `std::distance(begin, std::max_element(begin, end))`.
/// `std::max_element` returns the first element comparing greatest; on an empty
/// range it returns `begin` (index 0). We replicate the "first maximum" tie-break.
fn max_element_index(v: &[f64]) -> usize {
    if v.is_empty() {
        return 0;
    }
    let mut best = 0usize;
    for i in 1..v.len() {
        if v[i] > v[best] {
            best = i;
        }
    }
    best
}

// =============================================================================
// FillFloatingConcentric — the Arachne floating-concentric filler.
//
// FillFloatingConcentric.hpp:36-70 `class FillFloatingConcentric : public FillConcentric`.
// The Rust fill module has no shared `Fill` base struct, so the base members
// this filler reads (`no_overlap_expolygons`, `spacing`, `loop_clipping`, the
// `print_config`/`print_object_config` pointers) are carried directly, mirroring
// `FillConcentricInternal`.
// =============================================================================

use crate::arachne::utils::extrusion_line::{to_thick_polyline, VariableWidthLines};
use crate::arachne::wall_tool_paths::WallToolPathsParams;
use crate::extrusion_entity::{
    ExtrusionEntityCollection, ExtrusionEntityType, ExtrusionLoop, ExtrusionLoopRole,
};
use crate::fill::FillParams;
use crate::geometry::ExPolygon;
use crate::print_config::{PrintConfig, PrintObjectConfig};
use crate::surface::Surface;

/// FillFloatingConcentric.hpp:36-70
pub struct FillFloatingConcentric<'a> {
    // Fill base (FillBase.hpp): `coordf_t spacing;` — unscaled mm.
    pub spacing: f64,
    // Fill base (FillBase.hpp): `coord_t loop_clipping;` — scaled.
    pub loop_clipping: Coord,
    // Fill base (FillBase.hpp): `ExPolygons no_overlap_expolygons;`
    pub no_overlap_expolygons: ExPolygons,
    // FillFloatingConcentric.hpp:40
    pub lower_layer_unsupport_areas: ExPolygons,
    // FillFloatingConcentric.hpp:41
    pub lower_sparse_polys: Polygons,
    // FillConcentricInternal.hpp:19 (inherited via FillConcentric chain)
    pub print_config: Option<&'a PrintConfig>,
    // FillConcentricInternal.hpp:20
    pub print_object_config: Option<&'a PrintObjectConfig>,
}

impl<'a> FillFloatingConcentric<'a> {
    /// `bool no_sort() const { return true; }`
    pub fn no_sort(&self) -> bool {
        true
    }

    /// FillFloatingConcentric.cpp:682-730
    /// `FloatingThickPolylines FillFloatingConcentric::resplit_order_loops(Point curr_point, std::vector<const Arachne::ExtrusionLine*> all_extrusions, const ExPolygons& floating_areas, const Polygons& sparse_polys, const coord_t default_width)`
    ///
    /// `all_extrusions` is borrowed (the C++ takes a vector of pointers); the
    /// visiting order is the C++ order (already produced by toplogic_sort).
    fn resplit_order_loops(
        &self,
        mut curr_point: Point,
        all_extrusions: &[&ExtrusionLine],
        floating_areas: &ExPolygons,
        sparse_polys: &Polygons,
        default_width: Coord,
    ) -> FloatingThickPolylines {
        // FillFloatingConcentric.cpp:684
        let mut result: FloatingThickPolylines = Vec::new();
        let detect_floating_vs = self
            .print_object_config
            .map(|c| c.detect_floating_vertical_shell)
            .unwrap_or(false);

        // FillFloatingConcentric.cpp:686
        for idx in 0..all_extrusions.len() {
            // FillFloatingConcentric.cpp:687-688
            if all_extrusions[idx].is_empty() {
                continue;
            }
            // FillFloatingConcentric.cpp:689
            let thick_polyline = to_thick_polyline(all_extrusions[idx]);
            // FillFloatingConcentric.cpp:690
            let mut is_self_intersect = false;
            // FillFloatingConcentric.cpp:691-700
            if detect_floating_vs {
                // FillFloatingConcentric.cpp:693
                let polyline =
                    crate::geometry::Polyline::from_points(thick_polyline.points.clone());
                let bbox_line = BoundingBox::from_points(&polyline.points);

                // FillFloatingConcentric.cpp:696-698
                // EdgeGrid::Grid grid; grid.set_bbox(bbox_line);
                // grid.create({polyline.points}, scaled(10.), !all_extrusions[idx]->is_closed);
                let mut grid = crate::edge_grid::EdgeGrid::new();
                grid.set_bbox(bbox_line);
                // Native: grid.create({polyline.points}, scaled(10.),
                // !is_closed) — the closed flag keeps the raw closing edge.
                grid.create_from_polylines_flag(
                    std::slice::from_ref(&polyline),
                    crate::scale(10.0),
                    !all_extrusions[idx].is_closed,
                );
                // FillFloatingConcentric.cpp:699-700
                if grid.has_intersecting_edges() {
                    is_self_intersect = true;
                }
            }
            // FillFloatingConcentric.cpp:702
            // detect_floating_line(thick_polyline, floating_areas, default_width,
            //   !detect_floating_vertical_shell || is_self_intersect)
            // R801 — NATIVE UB: FFC.cpp:707-710 builds the self-intersect
            // EdgeGrid from `{ polyline.points }`, a TEMPORARY vector<Points>;
            // native Contour stores raw pointers into it, and
            // has_intersecting_edges reads FREED memory (FVSDUMP4: garbage
            // coords (0,0)-(0,0), always slots 1/3). The result: native's
            // is_self_intersect fires on ~76% of loops as allocator noise and
            // floating detection is EFFECTIVELY OFF (347/138,450 floating
            // thicklines on Majora). Rust cannot reproduce use-after-free, so
            // FVS_FORCE_NO_DETECT (faithful ON) mirrors native's dominant
            // observable behaviour: skip detection. The exact-machinery arm
            // (FVS_SELFX grid) stays for the day native fixes the UB.
            let force_no_detect = !detect_floating_vs
                || is_self_intersect
                || crate::faithful_gate("FVS_FORCE_NO_DETECT");
            if crate::probe_enabled("FVS_DEBUG") {
                use std::sync::atomic::Ordering::Relaxed;
                if all_extrusions[idx].is_closed
                    && thick_polyline.points.first() == thick_polyline.points.last()
                {
                    crate::layer::FVS_CLOSED_DUP.fetch_add(1, Relaxed);
                } else if all_extrusions[idx].is_closed {
                    crate::layer::FVS_CLOSED_NODUP.fetch_add(1, Relaxed);
                }
                crate::layer::FVS_LINES.fetch_add(1, Relaxed);
                if !detect_floating_vs { crate::layer::FVS_FLAG_OFF.fetch_add(1, Relaxed); }
                if is_self_intersect { crate::layer::FVS_SELF_INT.fetch_add(1, Relaxed); }
                if floating_areas.is_empty() { crate::layer::FVS_NO_AREAS.fetch_add(1, Relaxed); }
            }
            let thick_line_with_floating = detect_floating_line(
                &thick_polyline,
                floating_areas,
                default_width as f64,
                force_no_detect,
            );
            // FillFloatingConcentric.cpp:703
            let mut thick_line_with_floating = thick_line_with_floating;
            smooth_floating_line(
                &mut thick_line_with_floating,
                crate::scale(2.0),
                crate::scale(2.0),
            );
            // FillFloatingConcentric.cpp:704
            let mut split_idx: i32 = 0;
            // FillFloatingConcentric.cpp:705-721
            if !floating_areas.is_empty()
                && all_extrusions[idx].is_closed
                && thick_line_with_floating.points.first()
                    == thick_line_with_floating.points.last()
            {
                // FillFloatingConcentric.cpp:706-718
                if idx == 0 {
                    split_idx = get_best_loop_start(
                        &thick_line_with_floating.points,
                        floating_areas,
                        sparse_polys,
                    );
                } else {
                    let candidates = get_loop_start_candidates(
                        &thick_line_with_floating.points,
                        floating_areas,
                        sparse_polys,
                    );
                    let mut min_dist = f64::MAX;
                    for candidate in candidates {
                        let p = thick_line_with_floating.points[candidate as usize];
                        let dx = (curr_point.x - p.x) as f64;
                        let dy = (curr_point.y - p.y) as f64;
                        let dist = (dx * dx + dy * dy).sqrt();
                        if min_dist > dist {
                            min_dist = dist;
                            split_idx = candidate;
                        }
                    }
                }
                // FillFloatingConcentric.cpp:719-721
                result.push(thick_line_with_floating.rebase_at(split_idx as usize));
            } else {
                // FillFloatingConcentric.cpp:723-726
                result.push(thick_line_with_floating);
            }
            // FillFloatingConcentric.cpp:728
            curr_point = result.last().unwrap().last_point();
        }
        // FillFloatingConcentric.cpp:730
        result
    }

    /// FillFloatingConcentric.cpp:878-933
    /// `void FillFloatingConcentric::_fill_surface_single(const FillParams& params, unsigned int thickness_layers, const std::pair<float, Point>& direction, ExPolygon expolygon, FloatingThickPolylines& thick_polylines_out)`
    fn fill_surface_single(
        &self,
        params: &FillParams,
        expolygon: &ExPolygon,
        thick_polylines_out: &mut FloatingThickPolylines,
    ) {
        let print_config = self.print_config.unwrap();
        // FillFloatingConcentric.cpp:884
        let bbox_size: Point = expolygon.contour.bounding_box().size();
        // FillFloatingConcentric.cpp:885
        let min_spacing: Coord = params.flow.scaled_spacing();
        // FillFloatingConcentric.cpp:887
        let loops_count: Coord = std::cmp::max(bbox_size.x(), bbox_size.y()) / min_spacing + 1;
        // FillFloatingConcentric.cpp:888
        let polygons: Polygons = crate::geometry::to_polygons_expoly(expolygon);

        // FillFloatingConcentric.cpp:890-898
        let min_nozzle_diameter: f64 = print_config.nozzle_diameter;
        let mut input_params = WallToolPathsParams::default();
        input_params.min_bead_width = (0.85 * min_nozzle_diameter) as f32;
        input_params.min_feature_size = (0.25 * min_nozzle_diameter) as f32;
        input_params.wall_transition_length = 0.4;
        input_params.wall_transition_angle = 10.0;
        input_params.wall_transition_filter_deviation = (0.25 * min_nozzle_diameter) as f32;
        input_params.wall_distribution_count = 1;

        // FillFloatingConcentric.cpp:900
        let mut wall_tool_paths = WallToolPaths::new(
            polygons,
            min_spacing,
            min_spacing,
            loops_count as usize,
            0,
            params.layer_height,
            input_params,
        );

        // FillFloatingConcentric.cpp:902
        let loops: Vec<VariableWidthLines> = wall_tool_paths.get_tool_paths().clone();
        // FillFloatingConcentric.cpp:903-913
        let mut all_extrusions: Vec<&ExtrusionLine> = Vec::new();
        for loop_ in loops.iter() {
            if loop_.is_empty() {
                continue;
            }
            for wall in loop_.iter() {
                all_extrusions.push(wall);
            }
        }
        // FillFloatingConcentric.cpp:912 — ordered_extrusions = toplogic_sort_extruisons(all_extrusions);
        let order = toplogic_sort_extruisons(&all_extrusions);
        let ordered: Vec<&ExtrusionLine> = order.iter().map(|&i| all_extrusions[i]).collect();

        // FillFloatingConcentric.cpp:916-918
        let firts_poly_idx = thick_polylines_out.len();
        let thick_polylines = self.resplit_order_loops(
            Point::new(0, 0),
            &ordered,
            &self.lower_layer_unsupport_areas,
            &self.lower_sparse_polys,
            min_spacing,
        );
        // append(thick_polylines_out, thick_polylines);
        thick_polylines_out.extend(thick_polylines);

        // FillFloatingConcentric.cpp:922-931 — clip + keep valid only.
        let mut j = firts_poly_idx;
        for i in firts_poly_idx..thick_polylines_out.len() {
            thick_polylines_out[i].clip_end(self.loop_clipping as f64);
            if thick_polylines_out[i].is_valid() {
                if j < i {
                    thick_polylines_out[j] = std::mem::take(&mut thick_polylines_out[i]);
                }
                j += 1;
            }
        }
        if j < thick_polylines_out.len() {
            thick_polylines_out.truncate(j);
        }
    }

    /// FillFloatingConcentric.cpp:936-944
    /// `FloatingThickPolylines FillFloatingConcentric::fill_surface_arachne_floating(const Surface* surface, const FillParams& params)`
    fn fill_surface_arachne_floating(&self, params: &FillParams) -> FloatingThickPolylines {
        // FillFloatingConcentric.cpp:939
        let mut out: FloatingThickPolylines = Vec::new();
        // FillFloatingConcentric.cpp:941-942 — for each expoly in no_overlap_expolygons.
        // (C++ moves each expoly into _fill_surface_single; `fill_surface_single`
        // only reads the expoly, so iterate by reference.)
        for expoly in &self.no_overlap_expolygons {
            self.fill_surface_single(params, expoly, &mut out);
        }
        out
    }

    /// FillFloatingConcentric.cpp:946-1000
    /// `void FillFloatingConcentric::fill_surface_extrusion(const Surface* surface, const FillParams& params, ExtrusionEntitiesPtr& out)`
    pub fn fill_surface_extrusion(
        &mut self,
        _surface: &Surface,
        params: &FillParams,
        out: &mut Vec<ExtrusionEntityType>,
    ) {
        // FillFloatingConcentric.cpp:948
        let floating_lines = self.fill_surface_arachne_floating(params);
        // FillFloatingConcentric.cpp:950-951
        if floating_lines.is_empty() {
            return;
        }
        // FillFloatingConcentric.cpp:952
        let new_flow: Flow = params
            .flow
            .with_spacing(self.spacing as f32 as f64)
            .expect("with_spacing");

        // FillFloatingConcentric.cpp:956-959
        let mut ecc = ExtrusionEntityCollection::new();
        ecc.no_sort = true;

        // FillFloatingConcentric.cpp:962 — tolerance = float(scale_(0.05));
        let tolerance = crate::scale(0.05) as f32;
        // FillFloatingConcentric.cpp:963-980
        for line in &floating_lines {
            // FillFloatingConcentric.cpp:964
            let paths = floating_thick_polyline_to_extrusion_paths(
                line,
                params.extrusion_role,
                &new_flow,
                tolerance,
            );
            // FillFloatingConcentric.cpp:966-979
            if !paths.is_empty() {
                // FillFloatingConcentric.cpp:968-969
                if paths.first().unwrap().first_point() == paths.last().unwrap().last_point() {
                    // FillFloatingConcentric.cpp:969 — new ExtrusionLoop(std::move(paths))
                    // (default ExtrusionLoopRole == elrDefault).
                    let loop_ = ExtrusionLoop::new(paths, ExtrusionLoopRole::default());
                    ecc.entities.push(ExtrusionEntityType::Loop(loop_));
                } else {
                    // FillFloatingConcentric.cpp:971-977
                    for path in paths {
                        ecc.entities.push(ExtrusionEntityType::Path(path));
                    }
                }
            }
        }

        // FillFloatingConcentric.cpp:954-957 — out.push_back(ecc) (always, even if
        // ecc stays empty because every paths set was empty — matches C++ which
        // push_backs the freshly-new'd ecc before the loop).
        out.push(ExtrusionEntityType::Collection(Box::new(ecc)));
    }
}
