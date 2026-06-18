// FillBase.cpp — faithful 1:1 port of the infill <-> perimeter connection machinery.
//
// This module ports the self-contained algorithmic core of BambuStudio's
// `src/libslic3r/Fill/FillBase.cpp`: the `ContourIntersectionPoint` graph, the
// `take_*` perimeter-walking helpers, `line_rounded_thick_segment_collision`,
// `mark_boundary_segments_*`, `create_boundary_infill_graph`, the
// `Fill::connect_infill` family, the support-infill connector
// `Fill::connect_base_support`, `multiline_fill`, `_adjust_solid_spacing` and
// `_infill_direction`.
//
// C++ uses raw pointers into a `std::vector<ContourIntersectionPoint>` and
// pointer arithmetic (`&cp - data()`) to recover an element's index. Rust models
// the same graph with `usize` indices into the same backing `Vec`: a null
// pointer becomes `usize::MAX` (`NULL_CP`), `prev_on_contour`/`next_on_contour`
// become indices, and `&cp - data()` becomes the element index. The control flow,
// constants, rounding and edge cases are preserved verbatim.
//
// NOTE on casts: C++ `Point::cast<double>()` is a *raw* numeric cast of the
// scaled integer coordinates to double (it does NOT unscale). Therefore this
// port uses `pt.x() as f64` rather than `Point::to_f64()` (which unscales).
// Likewise `scale_(v)` in a `double` context is `v * SCALING_FACTOR` without
// rounding; `scale_` producing a `coord_t` rounds via `crate::scale`.

use crate::edge_grid::EdgeGrid;
use crate::geometry::{
    liang_barsky_line_clipping_interval, lerp, perp, ray_circle_intersections_r2_lv2_c,
    BoundingBox, BoundingBoxF as BoundingBoxf, ExPolygon, Point, Polygon, Polyline, Vec2d,
};
use crate::utils::{next_idx_modulo, next_value_modulo, prev_idx_modulo, prev_value_modulo};
use crate::{Coord, SCALING_FACTOR};

use super::FillParams;

// libslic3r.h:84 — SCALED_EPSILON = scale_(EPSILON) = 1e-4 * 1e5 = 10.0
const SCALED_EPSILON: f64 = crate::libslic3r::SCALED_EPSILON;
// libslic3r.h:52 — EPSILON = 1e-4
const EPSILON: f64 = 1e-4;

// `scale_(val)` in a floating-point context: val / SCALING_FACTOR (the C++ macro)
// where SCALING_FACTOR == 1e-5, i.e. multiply by the crate's SCALING_FACTOR (1e5).
#[inline]
fn scale_f(v: f64) -> f64 {
    v * SCALING_FACTOR
}

// Sentinel for a null `ContourIntersectionPoint*`.
const NULL_CP: usize = usize::MAX;

// FillBase.cpp:1184 — static constexpr auto boundary_idx_unconnected = std::numeric_limits<size_t>::max();
const BOUNDARY_IDX_UNCONNECTED: usize = usize::MAX;

// Squared point-to-segment distance, float coordinates.
// Mirrors `line_alg::distance_to_squared(Linef{a, b}, p)` (Line.hpp:42-69).
#[inline]
fn linef_distance_to_squared(a: Vec2d, b: Vec2d, point: Vec2d) -> f64 {
    // Line.hpp:45-47
    let v = Vec2d::new(b.x - a.x, b.y - a.y);
    let va = Vec2d::new(point.x - a.x, point.y - a.y);
    let l2 = v.x * v.x + v.y * v.y; // squaredNorm
    if l2 == 0.0 {
        // Line.hpp:48-52 — a == b case.
        return va.x * va.x + va.y * va.y;
    }
    // Line.hpp:56
    let t = (va.x * v.x + va.y * v.y) / l2;
    if t <= 0.0 {
        // Line.hpp:57-60 — beyond the 'a' end.
        va.x * va.x + va.y * va.y
    } else if t >= 1.0 {
        // Line.hpp:61-64 — beyond the 'b' end.
        let d = Vec2d::new(point.x - b.x, point.y - b.y);
        d.x * d.x + d.y * d.y
    } else {
        // Line.hpp:67-68
        let proj = Vec2d::new(t * v.x - va.x, t * v.y - va.y);
        proj.x * proj.x + proj.y * proj.y
    }
}

#[inline]
fn linef_distance_to(a: Vec2d, b: Vec2d, point: Vec2d) -> f64 {
    linef_distance_to_squared(a, b, point).sqrt()
}

// A single T joint of an infill line to a closed contour or one of its holes.
// FillBase.cpp:244-292
#[derive(Clone)]
pub struct ContourIntersectionPoint {
    // Contour and point on a contour where an infill line is connected to.
    // FillBase.cpp:246-248
    pub contour_idx: usize,
    pub point_idx: usize,
    // Eucleidean parameter of point_idx along its contour.
    // FillBase.cpp:249
    pub param: f64,
    // Other intersection points along the same contour. If there is only a single T-joint on a contour
    // with an intersection line, then the prev_on_contour and next_on_contour remain nulls.
    // FillBase.cpp:251-253
    pub prev_on_contour: usize,
    pub next_on_contour: usize,
    // Length of the contour not yet allocated to some extrusion path going back (clockwise), or masked out by some overlapping infill line.
    // FillBase.cpp:255
    pub contour_not_taken_length_prev: f64,
    // Length of the contour not yet allocated to some extrusion path going forward (counter-clockwise), or masked out by some overlapping infill line.
    // FillBase.cpp:257
    pub contour_not_taken_length_next: f64,
    // End point is consumed if an infill line connected to this T-joint was already connected left or right along the contour,
    // or if the infill line was processed, but it was not possible to connect it left or right along the contour.
    // FillBase.cpp:260
    pub consumed: bool,
    // Whether the contour was trimmed by an overlapping infill line, or whether part of this contour was connected to some infill line.
    // FillBase.cpp:262-263
    pub prev_trimmed: bool,
    pub next_trimmed: bool,
}

impl ContourIntersectionPoint {
    // FillBase.cpp:1358 — ContourIntersectionPoint{ contour_idx, point_idx } with the
    // remaining members default-initialized per the field initializers in the struct.
    fn new(contour_idx: usize, point_idx: usize) -> Self {
        Self {
            contour_idx,
            point_idx,
            param: 0.0,
            prev_on_contour: NULL_CP,
            next_on_contour: NULL_CP,
            // FillBase.cpp:255 / 257 — std::numeric_limits<double>::max()
            contour_not_taken_length_prev: f64::MAX,
            contour_not_taken_length_next: f64::MAX,
            consumed: false,
            prev_trimmed: false,
            next_trimmed: false,
        }
    }

    // FillBase.cpp:265
    fn consume_prev(&mut self) {
        self.contour_not_taken_length_prev = 0.0;
        self.prev_trimmed = true;
        self.consumed = true;
    }
    // FillBase.cpp:266
    fn consume_next(&mut self) {
        self.contour_not_taken_length_next = 0.0;
        self.next_trimmed = true;
        self.consumed = true;
    }

    // FillBase.cpp:268-273
    fn trim_prev(&mut self, new_len: f64) {
        if new_len < self.contour_not_taken_length_prev {
            self.contour_not_taken_length_prev = new_len;
            self.prev_trimmed = true;
        }
    }
    // FillBase.cpp:274-279
    fn trim_next(&mut self, new_len: f64) {
        if new_len < self.contour_not_taken_length_next {
            self.contour_not_taken_length_next = new_len;
            self.next_trimmed = true;
        }
    }

    // The end point of an infill line connected to this T-joint was not processed yet and a piece of the contour could be extruded going backwards.
    // FillBase.cpp:282
    fn could_take_prev(&self) -> bool {
        !self.consumed && self.contour_not_taken_length_prev > SCALED_EPSILON
    }
    // The end point of an infill line connected to this T-joint was not processed yet and a piece of the contour could be extruded going forward.
    // FillBase.cpp:284
    fn could_take_next(&self) -> bool {
        !self.consumed && self.contour_not_taken_length_next > SCALED_EPSILON
    }
}

// `could_connect_prev`/`could_connect_next` reference the neighbouring node and so are
// implemented as free functions on the backing vector (FillBase.cpp:287-291).
#[inline]
fn could_connect_prev(cps: &[ContourIntersectionPoint], idx: usize) -> bool {
    let cp = &cps[idx];
    !cp.consumed
        && cp.prev_on_contour != idx
        && !cps[cp.prev_on_contour].consumed
        && !cp.prev_trimmed
        && !cps[cp.prev_on_contour].next_trimmed
}
#[inline]
fn could_connect_next(cps: &[ContourIntersectionPoint], idx: usize) -> bool {
    let cp = &cps[idx];
    !cp.consumed
        && cp.next_on_contour != idx
        && !cps[cp.next_on_contour].consumed
        && !cp.next_trimmed
        && !cps[cp.next_on_contour].prev_trimmed
}

// Distance from param1 to param2 when going counter-clockwise.
// FillBase.cpp:295-303
#[inline]
fn closed_contour_distance_ccw(param1: f64, param2: f64, contour_length: f64) -> f64 {
    let mut d = param2 - param1;
    if d < 0.0 {
        d += contour_length;
    }
    d
}

// Distance from param1 to param2 when going clockwise.
// FillBase.cpp:305-309
#[inline]
fn closed_contour_distance_cw(param1: f64, param2: f64, contour_length: f64) -> f64 {
    closed_contour_distance_ccw(param2, param1, contour_length)
}

// Length along the contour from cp1 to cp2 going counter-clockwise.
// FillBase.cpp:311-319
fn path_length_along_contour_ccw(
    cps: &[ContourIntersectionPoint],
    cp1: usize,
    cp2: usize,
    contour_length: f64,
) -> f64 {
    closed_contour_distance_ccw(cps[cp1].param, cps[cp2].param, contour_length)
}

// Lengths along the contour from cp1 to cp2 going CCW and going CW.
// FillBase.cpp:321-338
#[allow(dead_code)]
fn path_lengths_along_contour(
    cps: &[ContourIntersectionPoint],
    cp1: usize,
    cp2: usize,
    contour_length: f64,
) -> (f64, f64) {
    // Zero'th param is the length of the contour.
    let mut param_lo = cps[cp1].param;
    let mut param_hi = cps[cp2].param;
    let mut reversed = false;
    if param_lo > param_hi {
        std::mem::swap(&mut param_lo, &mut param_hi);
        reversed = true;
    }
    let mut out = (param_hi - param_lo, param_lo + contour_length - param_hi);
    if reversed {
        std::mem::swap(&mut out.0, &mut out.1);
    }
    out
}

// Add contour points from interval (idx_start, idx_end> to polyline.
// FillBase.cpp:341-352
fn take_cw_full(pl: &mut Polyline, contour: &[Point], idx_start: usize, idx_end: usize) {
    debug_assert!(!pl.points.is_empty() && *pl.points.last().unwrap() == contour[idx_start]);
    let mut i = if idx_start == 0 {
        contour.len() - 1
    } else {
        idx_start - 1
    };
    while i != idx_end {
        pl.points.push(contour[i]);
        if i == 0 {
            i = contour.len();
        }
        i -= 1;
    }
    pl.points.push(contour[i]);
}

// Add contour points from interval (idx_start, idx_end> to polyline, limited by the Eucleidean length taken.
// FillBase.cpp:355-392
fn take_cw_limited(
    pl: &mut Polyline,
    contour: &[Point],
    params: &[f64],
    idx_start: usize,
    idx_end: usize,
    length_to_take: f64,
) -> f64 {
    // Length of the contour.
    let length = *params.last().unwrap();
    // Parameter (length from contour.front()) for the first point.
    let p0 = params[idx_start];
    // Current (2nd) point of the contour.
    let mut i = if idx_start == 0 {
        contour.len() - 1
    } else {
        idx_start - 1
    };
    // Previous point of the contour.
    let mut iprev = idx_start;
    // Length of the contour curve taken for iprev.
    let mut lprev = 0.0;

    loop {
        let l = closed_contour_distance_cw(p0, params[i], length);
        if l >= length_to_take {
            // Trim the last segment.
            let t = (length_to_take - lprev) / (l - lprev);
            pl.points.push(lerp(contour[iprev], contour[i], t));
            return length_to_take;
        }
        // Continue with the other segments.
        pl.points.push(contour[i]);
        if i == idx_end {
            return l;
        }
        iprev = i;
        lprev = l;
        if i == 0 {
            i = contour.len();
        }
        i -= 1;
    }
}

// Add contour points from interval (idx_start, idx_end> to polyline.
// FillBase.cpp:395-407
fn take_ccw_full(pl: &mut Polyline, contour: &[Point], idx_start: usize, idx_end: usize) {
    debug_assert!(!pl.points.is_empty() && *pl.points.last().unwrap() == contour[idx_start]);
    let mut i = idx_start;
    i += 1;
    if i == contour.len() {
        i = 0;
    }
    while i != idx_end {
        pl.points.push(contour[i]);
        i += 1;
        if i == contour.len() {
            i = 0;
        }
    }
    pl.points.push(contour[i]);
}

// Add contour points from interval (idx_start, idx_end> to polyline, limited by the Eucleidean length taken.
// Returns length of the contour taken.
// FillBase.cpp:411-448
fn take_ccw_limited(
    pl: &mut Polyline,
    contour: &[Point],
    params: &[f64],
    idx_start: usize,
    idx_end: usize,
    length_to_take: f64,
) -> f64 {
    // Length of the contour.
    let length = *params.last().unwrap();
    // Parameter (length from contour.front()) for the first point.
    let p0 = params[idx_start];
    // Current (2nd) point of the contour.
    let mut i = idx_start;
    i += 1;
    if i == contour.len() {
        i = 0;
    }
    // Previous point of the contour.
    let mut iprev = idx_start;
    // Length of the contour curve taken at iprev.
    let mut lprev = 0.0;
    loop {
        let l = closed_contour_distance_ccw(p0, params[i], length);
        if l >= length_to_take {
            // Trim the last segment.
            let t = (length_to_take - lprev) / (l - lprev);
            pl.points.push(lerp(contour[iprev], contour[i], t));
            return length_to_take;
        }
        // Continue with the other segments.
        pl.points.push(contour[i]);
        if i == idx_end {
            return l;
        }
        iprev = i;
        lprev = l;
        i += 1;
        if i == contour.len() {
            i = 0;
        }
    }
}

// Connect end of pl1 to the start of pl2 using the perimeter contour.
// If clockwise, then a clockwise segment from idx_start to idx_end is taken, otherwise a counter-clockwise segment is being taken.
// FillBase.cpp:452-474
fn take_points(
    pl1: &mut Polyline,
    pl2: &Polyline,
    contour: &[Point],
    idx_start: usize,
    idx_end: usize,
    clockwise: bool,
) {
    {
        // Reserve memory at pl1 for the connecting contour and pl2.
        let mut new_points = idx_end as i64 - idx_start as i64 - 1;
        if new_points < 0 {
            new_points += contour.len() as i64;
        }
        pl1.points
            .reserve(new_points as usize + pl2.points.len());
    }

    if clockwise {
        take_cw_full(pl1, contour, idx_start, idx_end);
    } else {
        take_ccw_full(pl1, contour, idx_start, idx_end);
    }

    // pl1.points.insert(pl1.points.end(), pl2.points.begin() + 1, pl2.points.end());
    pl1.points.extend_from_slice(&pl2.points[1..]);
}

// FillBase.cpp:476-496 — take(pl1, pl2, contour, cp_start, cp_end, clockwise).
// `cps`, `infill`, `idx1`, `idx2` give the polylines and the graph, since `take`
// also mutates the ContourIntersectionPoints in between cp_start and cp_end.
fn take_cp(
    cps: &mut [ContourIntersectionPoint],
    pl1: &mut Polyline,
    pl2: &Polyline,
    contour: &[Point],
    cp_start_in: usize,
    cp_end_in: usize,
    clockwise: bool,
) {
    take_points(
        pl1,
        pl2,
        contour,
        cps[cp_start_in].point_idx,
        cps[cp_end_in].point_idx,
        clockwise,
    );

    // Mark the contour segments in between cp_start and cp_end as consumed.
    let (cp_start, cp_end) = if clockwise {
        (cp_end_in, cp_start_in)
    } else {
        (cp_start_in, cp_end_in)
    };
    if cps[cp_start].next_on_contour != cp_end {
        let mut cp = cps[cp_start].next_on_contour;
        while cps[cp].next_on_contour != cp_end {
            cps[cp].consume_prev();
            cps[cp].consume_next();
            cp = cps[cp].next_on_contour;
        }
    }
    cps[cp_start].consume_next();
    cps[cp_end].consume_prev();
}

// FillBase.cpp:498-595 — take_limited(pl1, contour, params, cp_start, cp_end, clockwise, take_max_length, line_half_width)
fn take_limited(
    cps: &mut [ContourIntersectionPoint],
    pl1: &mut Polyline,
    contour: &[Point],
    params: &[f64],
    cp_start: usize,
    cp_end: usize,
    clockwise: bool,
    take_max_length: f64,
    line_half_width: f64,
) {
    // FillBase.cpp:513-514
    if !(if clockwise {
        cps[cp_start].could_take_prev()
    } else {
        cps[cp_start].could_take_next()
    }) {
        return;
    }

    // FillBase.cpp:516-522
    debug_assert!(
        pl1.points.first() == Some(&contour[cps[cp_start].point_idx])
            || pl1.points.last() == Some(&contour[cps[cp_start].point_idx])
    );
    let add_at_start = pl1.points.first() == Some(&contour[cps[cp_start].point_idx]);
    let mut pl_tmp: Vec<Point> = Vec::new();
    if add_at_start {
        pl_tmp = std::mem::take(&mut pl1.points);
        pl1.points.clear();
    }

    {
        // Reserve memory at pl1 for the perimeter segment.
        // FillBase.cpp:524-531
        let mut new_points =
            cps[cp_end].point_idx as i64 - cps[cp_start].point_idx as i64 - 1;
        if new_points < 0 {
            new_points += contour.len() as i64;
        }
        pl1.points
            .reserve(pl_tmp.len() + new_points as usize);
    }

    // FillBase.cpp:533-534
    let length = *params.last().unwrap();
    let mut length_to_go = take_max_length;
    cps[cp_start].consumed = true;
    if cp_start == cp_end {
        // FillBase.cpp:536-544
        length_to_go = (0.0_f64).max(length_to_go.min(length - line_half_width));
        length_to_go = length_to_go.min(if clockwise {
            cps[cp_start].contour_not_taken_length_prev
        } else {
            cps[cp_start].contour_not_taken_length_next
        });
        cps[cp_start].consume_prev();
        cps[cp_start].consume_next();
        if length_to_go > SCALED_EPSILON {
            let p_idx = cps[cp_start].point_idx;
            if clockwise {
                take_cw_limited(pl1, contour, params, p_idx, p_idx, length_to_go);
            } else {
                take_ccw_limited(pl1, contour, params, p_idx, p_idx, length_to_go);
            }
        }
    } else if clockwise {
        // FillBase.cpp:545-567 — Going clockwise from cp_start to cp_end.
        let mut cp = cp_start;
        while cp != cp_end {
            // Length of the segment from cp to cp->prev_on_contour.
            let prev = cps[cp].prev_on_contour;
            let l = closed_contour_distance_cw(cps[cp].param, cps[prev].param, length);
            length_to_go = length_to_go.min(cps[cp].contour_not_taken_length_prev);
            // Don't overlap with an already extruded infill line.
            length_to_go = (0.0_f64).max(length_to_go.min(l - line_half_width));
            cps[cp].consume_prev();
            if l >= length_to_go {
                if length_to_go > SCALED_EPSILON {
                    cps[prev].trim_next(l - length_to_go);
                    take_cw_limited(
                        pl1,
                        contour,
                        params,
                        cps[cp].point_idx,
                        cps[prev].point_idx,
                        length_to_go,
                    );
                }
                break;
            } else {
                cps[prev].trim_next(0.0);
                take_cw_full(pl1, contour, cps[cp].point_idx, cps[prev].point_idx);
                length_to_go -= l;
            }
            cp = prev;
        }
    } else {
        // FillBase.cpp:568-589
        let mut cp = cp_start;
        while cp != cp_end {
            let next = cps[cp].next_on_contour;
            let l = closed_contour_distance_ccw(cps[cp].param, cps[next].param, length);
            length_to_go = length_to_go.min(cps[cp].contour_not_taken_length_next);
            // Don't overlap with an already extruded infill line.
            length_to_go = (0.0_f64).max(length_to_go.min(l - line_half_width));
            cps[cp].consume_next();
            if l >= length_to_go {
                if length_to_go > SCALED_EPSILON {
                    cps[next].trim_prev(l - length_to_go);
                    take_ccw_limited(
                        pl1,
                        contour,
                        params,
                        cps[cp].point_idx,
                        cps[next].point_idx,
                        length_to_go,
                    );
                }
                break;
            } else {
                cps[next].trim_prev(0.0);
                take_ccw_full(pl1, contour, cps[cp].point_idx, cps[next].point_idx);
                length_to_go -= l;
            }
            cp = next;
        }
    }

    // FillBase.cpp:591-594
    if add_at_start {
        pl1.reverse();
        pl1.points.extend_from_slice(&pl_tmp);
    }
}

// Return an index of start of a segment and a point of the clipping point at distance from the end of polyline.
// FillBase.cpp:598-606
#[derive(Clone, Copy)]
struct SegmentPoint {
    // Segment index, defining a line <idx_segment, idx_segment + 1).
    idx_segment: usize,
    // Parameter of point in <0, 1) along the line <idx_segment, idx_segment + 1)
    t: f64,
    point: Vec2d,
}

impl SegmentPoint {
    fn invalid() -> Self {
        Self {
            idx_segment: usize::MAX,
            t: 0.0,
            point: Vec2d::zero(),
        }
    }
    fn valid(&self) -> bool {
        self.idx_segment != usize::MAX
    }
}

// FillBase.cpp:608-631
fn clip_start_segment_and_point(polyline: &[Point], mut distance: f64) -> SegmentPoint {
    // Initialized to "invalid".
    let mut out = SegmentPoint::invalid();
    if polyline.len() >= 2 {
        let mut pt_prev = Vec2d::new(polyline[0].x() as f64, polyline[0].y() as f64);
        for i in 1..polyline.len() {
            let pt = Vec2d::new(polyline[i].x() as f64, polyline[i].y() as f64);
            let v = Vec2d::new(pt.x - pt_prev.x, pt.y - pt_prev.y);
            let l = (v.x * v.x + v.y * v.y).sqrt();
            if l > distance {
                out.idx_segment = i - 1;
                out.t = distance / l;
                out.point = Vec2d::new(pt_prev.x + out.t * v.x, pt_prev.y + out.t * v.y);
                break;
            }
            distance -= l;
            pt_prev = pt;
        }
    }
    out
}

// FillBase.cpp:633-658
fn clip_end_segment_and_point(polyline: &[Point], mut distance: f64) -> SegmentPoint {
    // Initialized to "invalid".
    let mut out = SegmentPoint::invalid();
    if polyline.len() >= 2 {
        let mut pt_next = Vec2d::new(
            polyline[polyline.len() - 1].x() as f64,
            polyline[polyline.len() - 1].y() as f64,
        );
        let mut i = polyline.len() as isize - 2;
        while i >= 0 {
            let pt = Vec2d::new(polyline[i as usize].x() as f64, polyline[i as usize].y() as f64);
            let v = Vec2d::new(pt.x - pt_next.x, pt.y - pt_next.y);
            let l = (v.x * v.x + v.y * v.y).sqrt();
            if l > distance {
                out.idx_segment = i as usize;
                out.t = distance / l;
                out.point = Vec2d::new(pt_next.x + out.t * v.x, pt_next.y + out.t * v.y);
                // Store the parameter referenced to the starting point of a segment.
                out.t = 1.0 - out.t;
                break;
            }
            distance -= l;
            pt_next = pt;
            i -= 1;
        }
    }
    out
}

// Calculate intersection of a line with a thick segment.
// Returns Eucledian parameters of the line / thick segment overlap.
// FillBase.cpp:662-763
fn line_rounded_thick_segment_collision(
    line_a: Vec2d,
    line_b: Vec2d,
    segment_a: Vec2d,
    segment_b: Vec2d,
    offset: f64,
    out_interval: &mut (f64, f64),
) -> bool {
    let line_v0 = Vec2d::new(line_b.x - line_a.x, line_b.y - line_a.y);
    let mut lv = line_v0.x * line_v0.x + line_v0.y * line_v0.y; // squaredNorm

    let segment_v = Vec2d::new(segment_b.x - segment_a.x, segment_b.y - segment_a.y);
    let segment_l = (segment_v.x * segment_v.x + segment_v.y * segment_v.y).sqrt();
    let offset2 = offset * offset;

    let intersects;
    if lv < SCALED_EPSILON * SCALED_EPSILON {
        // FillBase.cpp:676-687 — Very short line vector. Just test whether the center point is inside the offset line.
        let lpt = Vec2d::new(0.5 * (line_a.x + line_b.x), 0.5 * (line_a.y + line_b.y));
        if segment_l > SCALED_EPSILON {
            intersects = linef_distance_to_squared(segment_a, segment_b, lpt) < offset2;
        } else {
            let mid = Vec2d::new(
                0.5 * (segment_a.x + segment_b.x) - lpt.x,
                0.5 * (segment_a.y + segment_b.y) - lpt.y,
            );
            intersects = (mid.x * mid.x + mid.y * mid.y) < offset2;
        }
        if intersects {
            out_interval.0 = 0.0;
            out_interval.1 = lv.sqrt();
        }
    } else {
        // FillBase.cpp:688-740 — Output interval.
        let mut tmin = f64::MAX;
        let mut tmax = -f64::MAX;

        // Intersections with the inflated segment.
        // FillBase.cpp:716-732
        if segment_l > SCALED_EPSILON {
            ray_circle_intersection_interval_extend(
                segment_a, offset2, line_a, line_v0, &mut tmin, &mut tmax,
            );
            ray_circle_intersection_interval_extend(
                segment_b, offset2, line_a, line_v0, &mut tmin, &mut tmax,
            );
            // Clip the line segment transformed into a coordinate space of the segment,
            // where the segment spans (0, 0) to (segment_l, 0).
            let dir_x = Vec2d::new(segment_v.x / segment_l, segment_v.y / segment_l);
            let dir_y = Vec2d::new(-dir_x.y, dir_x.x);
            let line_p0 = Vec2d::new(line_a.x - segment_a.x, line_a.y - segment_a.y);
            if let Some((t0, t1)) = liang_barsky_line_clipping_interval(
                (
                    line_p0.x * dir_x.x + line_p0.y * dir_x.y,
                    line_p0.x * dir_y.x + line_p0.y * dir_y.y,
                ),
                (
                    line_v0.x * dir_x.x + line_v0.y * dir_x.y,
                    line_v0.x * dir_y.x + line_v0.y * dir_y.y,
                ),
                (0.0, -offset),
                (segment_l, offset),
            ) {
                tmin = tmin.min(t0);
                tmax = tmax.max(t1);
            }
        } else {
            let mid = Vec2d::new(
                0.5 * (segment_a.x + segment_b.x),
                0.5 * (segment_a.y + segment_b.y),
            );
            ray_circle_intersection_interval_extend(
                mid, offset, line_a, line_v0, &mut tmin, &mut tmax,
            );
        }

        intersects = tmin <= tmax;
        if intersects {
            lv = lv.sqrt();
            out_interval.0 = tmin * lv;
            out_interval.1 = tmax * lv;
        }
    }

    intersects
}

// FillBase.cpp:699-713 — Intersections with the inflated segment end points (lambda).
#[inline]
#[allow(clippy::too_many_arguments)]
fn ray_circle_intersection_interval_extend(
    segment_pt: Vec2d,
    offset2: f64,
    line_pt: Vec2d,
    line_vec: Vec2d,
    tmin: &mut f64,
    tmax: &mut f64,
) {
    let mut pts = (Vec2d::zero(), Vec2d::zero());
    let p0 = Vec2d::new(line_pt.x - segment_pt.x, line_pt.y - segment_pt.y);
    let lv2 = line_vec.x * line_vec.x + line_vec.y * line_vec.y; // squaredNorm
    if ray_circle_intersections_r2_lv2_c(
        offset2,
        line_vec.y,
        -line_vec.x,
        lv2,
        -line_vec.y * p0.x + line_vec.x * p0.y,
        &mut pts,
    ) != 0
    {
        let mut t_lo = ((pts.0.x - p0.x) * line_vec.x + (pts.0.y - p0.y) * line_vec.y) / lv2;
        let mut t_hi = ((pts.1.x - p0.x) * line_vec.x + (pts.1.y - p0.y) * line_vec.y) / lv2;
        if t_lo > t_hi {
            std::mem::swap(&mut t_lo, &mut t_hi);
        }
        t_lo = t_lo.max(0.0);
        t_hi = t_hi.min(1.0);
        if t_lo <= t_hi {
            *tmin = tmin.min(t_lo);
            *tmax = tmax.max(t_hi);
        }
    }
}

// Mark the segments of split boundary as consumed if they are very close to some of the infill line.
// FillBase.cpp:917-1162
#[allow(clippy::too_many_arguments)]
fn mark_boundary_segments_touching_infill(
    boundary: &[Vec<Point>],
    boundary_parameters: &[Vec<f64>],
    // boundary_intersections[contour] = list of indices into `cps`.
    boundary_intersections: &[Vec<usize>],
    cps: &mut [ContourIntersectionPoint],
    boundary_bbox: &BoundingBox,
    infill: &[Polyline],
    clip_distance: f64,
    distance_colliding: f64,
) {
    // FillBase.cpp:948-952
    let mut grid = EdgeGrid::new();
    // Make sure that the the grid is big enough for queries against the thick segment.
    grid.set_bbox(boundary_bbox.expanded((distance_colliding * 1.43) as Coord));
    // Inflate the bounding box by a thick line width.
    {
        let polylines: Vec<Polyline> = boundary
            .iter()
            .map(|pts| Polyline::from_points(pts.clone()))
            .collect();
        grid.create_from_polylines(
            &polylines,
            (clip_distance.max(distance_colliding) + scale_f(10.0)) as Coord,
        );
    }

    let radius = distance_colliding;

    // FillBase.cpp:1081-1156 — process each infill polyline.
    for polyline in infill {
        // Clip the infill polyline by the Eucledian distance along the polyline.
        let start_point = clip_start_segment_and_point(&polyline.points, clip_distance);
        let end_point = clip_end_segment_and_point(&polyline.points, clip_distance);
        if start_point.valid()
            && end_point.valid()
            && (start_point.idx_segment < end_point.idx_segment
                || (start_point.idx_segment == end_point.idx_segment
                    && start_point.t < end_point.t))
        {
            // FillBase.cpp:1094 — The clipped polyline is non-empty.
            for point_idx in start_point.idx_segment..=end_point.idx_segment {
                // FillBase.cpp:1116-1117
                let pt1 = if point_idx == start_point.idx_segment {
                    start_point.point
                } else {
                    Vec2d::new(
                        polyline.points[point_idx].x() as f64,
                        polyline.points[point_idx].y() as f64,
                    )
                };
                let pt2 = if point_idx == end_point.idx_segment {
                    end_point.point
                } else {
                    Vec2d::new(
                        polyline.points[point_idx + 1].x() as f64,
                        polyline.points[point_idx + 1].y() as f64,
                    )
                };
                // visitor.init(pt1, pt2);
                let mut infill_bbox = BoundingBoxf::new();
                infill_bbox.merge_point(pt1);
                infill_bbox.merge_point(pt2);
                infill_bbox.expand(radius + SCALED_EPSILON);

                // FillBase.cpp:1131-1143 — Simulate tracing of a thick line.
                let vn = {
                    let v = Vec2d::new(pt2.x - pt1.x, pt2.y - pt1.y);
                    let l = (v.x * v.x + v.y * v.y).sqrt();
                    if l > 0.0 {
                        Vec2d::new(v.x / l * distance_colliding, v.y / l * distance_colliding)
                    } else {
                        Vec2d::zero()
                    }
                };
                let vperp = perp(Point::new(vn.x as Coord, vn.y as Coord));
                let vperp = Vec2d::new(vperp.x as f64, vperp.y as f64);

                let visit = |a: Vec2d, b: Vec2d, cps: &mut [ContourIntersectionPoint]| {
                    grid.visit_cells_intersecting_line(
                        Point::new(a.x as Coord, a.y as Coord),
                        Point::new(b.x as Coord, b.y as Coord),
                        |iy, ix| {
                            mark_visitor_cell(
                                &grid,
                                iy,
                                ix,
                                boundary_intersections,
                                boundary_parameters,
                                cps,
                                pt1,
                                pt2,
                                radius,
                                &infill_bbox,
                            );
                            // FillBase.cpp — Continue traversing the grid along the edge.
                            true
                        },
                    );
                };

                let a = Vec2d::new(pt1.x - vn.x - vperp.x, pt1.y - vn.y - vperp.y);
                let b = Vec2d::new(pt2.x + vn.x - vperp.x, pt2.y + vn.y - vperp.y);
                visit(a, b, cps);
                let a = Vec2d::new(pt1.x - vn.x + vperp.x, pt1.y - vn.y + vperp.y);
                let b = Vec2d::new(pt2.x + vn.x + vperp.x, pt2.y + vn.y + vperp.y);
                visit(a, b, cps);
            }
        }
    }
}

// FillBase.cpp:971-1058 — the EdgeGrid Visitor operator() body.
#[allow(clippy::too_many_arguments)]
fn mark_visitor_cell(
    grid: &EdgeGrid,
    iy: usize,
    ix: usize,
    boundary_intersections: &[Vec<usize>],
    boundary_parameters: &[Vec<f64>],
    cps: &mut [ContourIntersectionPoint],
    infill_pt1: Vec2d,
    infill_pt2: Vec2d,
    radius: f64,
    infill_bbox: &BoundingBoxf,
) {
    // Called with a row and colum of the grid cell, which is intersected by a line.
    let cell_data_range = grid.cell_data_range_at(iy, ix).to_vec();
    for it_contour_and_segment in cell_data_range {
        // End points of the line segment and their vector.
        let segment = grid.segment(it_contour_and_segment);
        let intersections = &boundary_intersections[it_contour_and_segment.0];
        if intersections.is_empty() {
            // There is no infil line touching this contour, thus effort will be saved to calculate overlap with other infill lines.
            continue;
        }
        let seg_pt1 = Vec2d::new(segment.a.x() as f64, segment.a.y() as f64);
        let seg_pt2 = Vec2d::new(segment.b.x() as f64, segment.b.y() as f64);
        let mut interval = (0.0_f64, 0.0_f64);
        let mut bbox_seg = BoundingBoxf::new();
        bbox_seg.merge_point(seg_pt1);
        bbox_seg.merge_point(seg_pt2);
        if infill_bbox.intersects(&bbox_seg)
            && line_rounded_thick_segment_collision(
                seg_pt1,
                seg_pt2,
                infill_pt1,
                infill_pt2,
                radius,
                &mut interval,
            )
        {
            // The boundary segment intersects with the infill segment thickened by radius.
            // 1) Find the Euclidian parameters of seg_pt1 and seg_pt2 on its boundary contour.
            let contour_parameters = &boundary_parameters[it_contour_and_segment.0];
            let contour_length = *contour_parameters.last().unwrap();
            let param_seg_pt1 = contour_parameters[it_contour_and_segment.1];
            let param_seg_pt2 = contour_parameters[it_contour_and_segment.1 + 1];
            // FillBase.cpp:1005-1006
            let param_overlap1 = param_seg_pt2.min(param_seg_pt1 + interval.0);
            let param_overlap2 = param_seg_pt2.min(param_seg_pt1 + interval.1);
            // 2) Find the ContourIntersectionPoints before param_overlap1 and after param_overlap2.
            // Find the span of ContourIntersectionPoints, that is trimmed by the interval (param_overlap1, param_overlap2).
            let ip_low;
            let ip_high;
            if intersections.len() == 1 {
                // Only a single infill line touches this contour.
                ip_low = intersections[0];
                ip_high = intersections[0];
            } else {
                // FillBase.cpp:1015-1023 — lower_bound_by_predicate.
                let it_low = lower_bound_by_predicate(intersections, |l| {
                    cps[*l].param < param_overlap1
                });
                let it_high = lower_bound_by_predicate(intersections, |l| {
                    cps[*l].param < param_overlap2
                });
                let mut il = if it_low == intersections.len() {
                    intersections[0]
                } else {
                    intersections[it_low]
                };
                let ih = if it_high == intersections.len() {
                    intersections[0]
                } else {
                    intersections[it_high]
                };
                if cps[il].param != param_overlap1 {
                    il = cps[il].prev_on_contour;
                }
                ip_low = il;
                ip_high = ih;
            }
            // Mark all ContourIntersectionPoints between ip_low and ip_high as consumed.
            // FillBase.cpp:1027-1031
            if cps[ip_low].next_on_contour != ip_high {
                let mut ip = cps[ip_low].next_on_contour;
                while ip != ip_high {
                    cps[ip].consume_prev();
                    cps[ip].consume_next();
                    ip = cps[ip].next_on_contour;
                }
            }
            // Subtract the interval from the first and last segments.
            // FillBase.cpp:1033-1038
            let mut trim_l =
                closed_contour_distance_ccw(cps[ip_low].param, param_overlap1, contour_length);
            cps[ip_low].trim_next(trim_l);
            trim_l = closed_contour_distance_ccw(param_overlap2, cps[ip_high].param, contour_length);
            cps[ip_high].trim_prev(trim_l);
        }
    }
}

// Utils.hpp `lower_bound_by_predicate` — first index whose predicate is false.
fn lower_bound_by_predicate<T, F: Fn(&T) -> bool>(slice: &[T], pred: F) -> usize {
    let mut lo = 0usize;
    let mut len = slice.len();
    while len > 0 {
        let half = len / 2;
        let mid = lo + half;
        if pred(&slice[mid]) {
            lo = mid + 1;
            len -= half + 1;
        } else {
            len = half;
        }
    }
    lo
}

// FillBase.cpp:1186-1267 — the BoundaryInfillGraph (graph of infill lines vs. boundary).
pub struct BoundaryInfillGraph {
    pub boundary: Vec<Vec<Point>>,
    pub boundary_params: Vec<Vec<f64>>,
    pub map_infill_end_point_to_boundary: Vec<ContourIntersectionPoint>,
}

// FillBase.cpp:1219-1225 — Direction enum.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Direction {
    Left,
    Right,
    Up,
    Down,
    Taken,
}

impl BoundaryInfillGraph {
    // FillBase.cpp:1192-1196
    fn point(&self, cp: &ContourIntersectionPoint) -> Point {
        self.boundary[cp.contour_idx][cp.point_idx]
    }
    fn point_idx(&self, idx: usize) -> Point {
        let cp = &self.map_infill_end_point_to_boundary[idx];
        self.boundary[cp.contour_idx][cp.point_idx]
    }

    // FillBase.cpp:1227-1231
    fn dir(p1: Point, p2: Point) -> Direction {
        if p1.x() == p2.x() {
            if p1.y() < p2.y() {
                Direction::Up
            } else {
                Direction::Down
            }
        } else if p1.x() < p2.x() {
            Direction::Right
        } else {
            Direction::Left
        }
    }

    // FillBase.cpp:1233-1238
    fn dir_prev(&self, idx: usize) -> Direction {
        let cp = &self.map_infill_end_point_to_boundary[idx];
        if cp.could_take_prev() {
            Self::dir(self.point(cp), self.point_idx(cp.prev_on_contour))
        } else {
            Direction::Taken
        }
    }

    // FillBase.cpp:1240-1245
    fn dir_next(&self, idx: usize) -> Direction {
        let cp = &self.map_infill_end_point_to_boundary[idx];
        if cp.could_take_next() {
            Self::dir(self.point(cp), self.point_idx(cp.next_on_contour))
        } else {
            Direction::Taken
        }
    }

    // FillBase.cpp:1247-1249
    fn first(&self, idx: usize) -> bool {
        (idx & 1) == 0
    }

    // FillBase.cpp:1251-1257 — other(cp) is the other end of the same infill line.
    fn other(idx: usize) -> usize {
        idx ^ 1
    }

    // FillBase.cpp:1259-1261
    fn prev_vertical(&self, idx: usize) -> bool {
        let cp = &self.map_infill_end_point_to_boundary[idx];
        self.point(cp).x() == self.point_idx(cp.prev_on_contour).x()
    }

    // FillBase.cpp:1263-1265
    fn next_vertical(&self, idx: usize) -> bool {
        let cp = &self.map_infill_end_point_to_boundary[idx];
        self.point(cp).x() == self.point_idx(cp.next_on_contour).x()
    }
}

// After mark_boundary_segments_touching_infill() marks boundary segments overlapping trimmed infill lines,
// there are possibly some very short boundary segments unmarked, but overlapping the untrimmed infill lines fully
// Mark those short boundary segments.
// FillBase.cpp:1273-1351
fn mark_boundary_segments_overlapping_infill(
    graph: &mut BoundaryInfillGraph,
    infill: &[Polyline],
    spacing: f64,
) {
    let n = graph.map_infill_end_point_to_boundary.len();
    for cp_idx in 0..n {
        let contour_idx = graph.map_infill_end_point_to_boundary[cp_idx].contour_idx;
        let contour = &graph.boundary[contour_idx];
        let contour_params = &graph.boundary_params[contour_idx];
        let infill_polyline = &infill[cp_idx / 2];
        let radius = 0.5 * (spacing + SCALED_EPSILON);
        let infill_a = Vec2d::new(
            infill_polyline.points[0].x() as f64,
            infill_polyline.points[0].y() as f64,
        );
        let infill_b = Vec2d::new(
            infill_polyline.points.last().unwrap().x() as f64,
            infill_polyline.points.last().unwrap().y() as f64,
        );

        // FillBase.cpp:1287-1317
        if graph.map_infill_end_point_to_boundary[cp_idx].could_take_next() {
            let cp_point_idx = graph.map_infill_end_point_to_boundary[cp_idx].point_idx;
            let next_point_idx = graph.map_infill_end_point_to_boundary
                [graph.map_infill_end_point_to_boundary[cp_idx].next_on_contour]
                .point_idx;
            let not_taken_next =
                graph.map_infill_end_point_to_boundary[cp_idx].contour_not_taken_length_next;
            let mut inside = true;
            let mut i = cp_point_idx;
            while i != next_point_idx {
                let j = next_idx_modulo(i, contour.len());
                let seg_pt2 = Vec2d::new(contour[j].x() as f64, contour[j].y() as f64);
                if linef_distance_to_squared(infill_a, infill_b, seg_pt2) < radius * radius {
                    // The segment is completely inside.
                } else {
                    let mut interval = (0.0_f64, 0.0_f64);
                    line_rounded_thick_segment_collision(
                        Vec2d::new(contour[i].x() as f64, contour[i].y() as f64),
                        seg_pt2,
                        infill_a,
                        infill_b,
                        radius,
                        &mut interval,
                    );
                    let len_out = closed_contour_distance_ccw(
                        contour_params[cp_point_idx],
                        contour_params[i],
                        *contour_params.last().unwrap(),
                    ) + interval.1;
                    if len_out < not_taken_next {
                        // Leaving the infill line region before exiting cp.contour_not_taken_length_next,
                        // thus at least some of the contour is outside and we will extrude this segment.
                        inside = false;
                        break;
                    }
                }
                if closed_contour_distance_ccw(
                    contour_params[cp_point_idx],
                    contour_params[j],
                    *contour_params.last().unwrap(),
                ) >= not_taken_next
                {
                    break;
                }
                i = j;
            }
            if inside {
                let next_on_contour =
                    graph.map_infill_end_point_to_boundary[cp_idx].next_on_contour;
                if !graph.map_infill_end_point_to_boundary[cp_idx].next_trimmed {
                    // The arc from cp to cp.next_on_contour was not trimmed yet, however it is completely overlapping the infill line.
                    graph.map_infill_end_point_to_boundary[next_on_contour].trim_prev(0.0);
                }
                graph.map_infill_end_point_to_boundary[cp_idx].trim_next(0.0);
            }
        } else {
            graph.map_infill_end_point_to_boundary[cp_idx].trim_next(0.0);
        }

        // FillBase.cpp:1318-1349
        if graph.map_infill_end_point_to_boundary[cp_idx].could_take_prev() {
            let cp_point_idx = graph.map_infill_end_point_to_boundary[cp_idx].point_idx;
            let prev_point_idx = graph.map_infill_end_point_to_boundary
                [graph.map_infill_end_point_to_boundary[cp_idx].prev_on_contour]
                .point_idx;
            let not_taken_prev =
                graph.map_infill_end_point_to_boundary[cp_idx].contour_not_taken_length_prev;
            let mut inside = true;
            let mut i = cp_point_idx;
            while i != prev_point_idx {
                let j = prev_idx_modulo(i, contour.len());
                let seg_pt2 = Vec2d::new(contour[j].x() as f64, contour[j].y() as f64);
                // Distance of the second segment line from the infill line.
                if linef_distance_to_squared(infill_a, infill_b, seg_pt2) < radius * radius {
                    // The segment is completely inside.
                } else {
                    let mut interval = (0.0_f64, 0.0_f64);
                    line_rounded_thick_segment_collision(
                        Vec2d::new(contour[i].x() as f64, contour[i].y() as f64),
                        seg_pt2,
                        infill_a,
                        infill_b,
                        radius,
                        &mut interval,
                    );
                    let len_out = closed_contour_distance_cw(
                        contour_params[cp_point_idx],
                        contour_params[i],
                        *contour_params.last().unwrap(),
                    ) + interval.1;
                    if len_out < not_taken_prev {
                        inside = false;
                        break;
                    }
                }
                if closed_contour_distance_cw(
                    contour_params[cp_point_idx],
                    contour_params[j],
                    *contour_params.last().unwrap(),
                ) >= not_taken_prev
                {
                    break;
                }
                i = j;
            }
            if inside {
                let prev_on_contour =
                    graph.map_infill_end_point_to_boundary[cp_idx].prev_on_contour;
                if !graph.map_infill_end_point_to_boundary[cp_idx].prev_trimmed {
                    // The arc from cp to cp.prev_on_contour was not trimmed yet, however it is completely overlapping the infill line.
                    graph.map_infill_end_point_to_boundary[prev_on_contour].trim_next(0.0);
                }
                graph.map_infill_end_point_to_boundary[cp_idx].trim_prev(0.0);
            }
        } else {
            graph.map_infill_end_point_to_boundary[cp_idx].trim_prev(0.0);
        }
    }
}

// FillBase.cpp:1353-1487
pub fn create_boundary_infill_graph(
    infill_ordered: &[Polyline],
    boundary_src: &[&Polygon],
    bbox: &BoundingBox,
    spacing: f64,
) -> BoundaryInfillGraph {
    let mut out = BoundaryInfillGraph {
        boundary: vec![Vec::new(); boundary_src.len()],
        boundary_params: vec![Vec::new(); boundary_src.len()],
        map_infill_end_point_to_boundary: (0..infill_ordered.len() * 2)
            .map(|_| {
                ContourIntersectionPoint::new(BOUNDARY_IDX_UNCONNECTED, BOUNDARY_IDX_UNCONNECTED)
            })
            .collect(),
    };

    // boundary_intersection_points[contour] = list of indices into map_infill_end_point_to_boundary.
    let mut boundary_intersection_points: Vec<Vec<usize>> = vec![Vec::new(); out.boundary.len()];

    {
        // Project the infill_ordered end points onto boundary_src.
        // FillBase.cpp:1361-1382
        // intersection_points: (ClosestPointResult, end_point_idx)
        let mut intersection_points: Vec<(crate::edge_grid::ClosestPointResult, usize)> =
            Vec::with_capacity(infill_ordered.len() * 2);
        {
            let mut grid = EdgeGrid::new();
            grid.set_bbox(bbox.expanded(SCALED_EPSILON as Coord));
            let polygons: Vec<Polygon> = boundary_src.iter().map(|p| (*p).clone()).collect();
            grid.create_from_polygons(&polygons, scale_f(10.0) as Coord);
            for (pl_idx, pl) in infill_ordered.iter().enumerate() {
                for (which, pt) in
                    [pl.points[0], *pl.points.last().unwrap()].into_iter().enumerate()
                {
                    let cp = grid.closest_point(&pt, SCALED_EPSILON as Coord);
                    if cp.is_valid() {
                        // The infill end point shall lie on the contour.
                        intersection_points.push((cp, pl_idx * 2 + which));
                    }
                }
            }
            // FillBase.cpp:1376-1381
            intersection_points.sort_by(|cp1, cp2| {
                let a = (cp1.0.contour_idx, cp1.0.start_point_idx, cp1.0.t);
                let b = (cp2.0.contour_idx, cp2.0.start_point_idx, cp2.0.t);
                a.partial_cmp(&b).unwrap_or(std::cmp::Ordering::Equal)
            });
        }

        // FillBase.cpp:1383-1463
        let mut it = 0usize;
        let it_end = intersection_points.len();
        for idx_contour in 0..boundary_src.len() {
            let contour_src = boundary_src[idx_contour];
            // Build the destination contour (contour_dst) and the per-contour intersection list.
            let mut contour_dst: Vec<Point> = Vec::new();
            let mut contour_intersection_points: Vec<usize> = Vec::new();
            let mut pfirst: usize = NULL_CP;
            let mut pprev: usize = NULL_CP;

            for idx_point in 0..contour_src.points.len() {
                let ipt = contour_src.points[idx_point];
                if contour_dst.is_empty() || *contour_dst.last().unwrap() != ipt {
                    contour_dst.push(ipt);
                }
                while it != it_end
                    && intersection_points[it].0.contour_idx == idx_contour
                    && intersection_points[it].0.start_point_idx == idx_point
                {
                    // Add these points to the destination contour.
                    let end_pt_idx = intersection_points[it].1;
                    let infill_line = &infill_ordered[end_pt_idx / 2];
                    let pt = if end_pt_idx & 1 != 0 {
                        *infill_line.points.last().unwrap()
                    } else {
                        infill_line.points[0]
                    };
                    let mut idx_tjoint_pt = 0usize;
                    if idx_point + 1 < contour_src.points.len() || pt != contour_dst[0] {
                        if pt != *contour_dst.last().unwrap() {
                            contour_dst.push(pt);
                        }
                        idx_tjoint_pt = contour_dst.len() - 1;
                    }
                    out.map_infill_end_point_to_boundary[end_pt_idx] =
                        ContourIntersectionPoint::new(idx_contour, idx_tjoint_pt);
                    let pthis = end_pt_idx;
                    if pprev != NULL_CP {
                        out.map_infill_end_point_to_boundary[pprev].next_on_contour = pthis;
                        out.map_infill_end_point_to_boundary[pthis].prev_on_contour = pprev;
                    } else {
                        pfirst = pthis;
                    }
                    contour_intersection_points.push(pthis);
                    pprev = pthis;
                    it += 1;
                }
                // FillBase.cpp:1435-1438 — inside the idx_point loop (matches C++ exactly).
                if pfirst != NULL_CP {
                    out.map_infill_end_point_to_boundary[pprev].next_on_contour = pfirst;
                    out.map_infill_end_point_to_boundary[pfirst].prev_on_contour = pprev;
                }
            }

            // Parametrize the new boundary with the intersection points inserted.
            // FillBase.cpp:1441-1448
            let mut contour_params: Vec<f64> = vec![0.0; contour_dst.len() + 1];
            for i in 1..contour_dst.len() {
                let d = Vec2d::new(
                    contour_dst[i].x() as f64 - contour_dst[i - 1].x() as f64,
                    contour_dst[i].y() as f64 - contour_dst[i - 1].y() as f64,
                );
                contour_params[i] = contour_params[i - 1] + (d.x * d.x + d.y * d.y).sqrt();
            }
            let last = contour_params.len() - 1;
            let d = Vec2d::new(
                contour_dst.last().unwrap().x() as f64 - contour_dst[0].x() as f64,
                contour_dst.last().unwrap().y() as f64 - contour_dst[0].y() as f64,
            );
            contour_params[last] = contour_params[last - 1] + (d.x * d.x + d.y * d.y).sqrt();

            // Map parameters from contour_params to boundary_intersection_points.
            // FillBase.cpp:1450-1451
            for &ip in &contour_intersection_points {
                out.map_infill_end_point_to_boundary[ip].param =
                    contour_params[out.map_infill_end_point_to_boundary[ip].point_idx];
            }
            // and measure distance to the previous and next intersection point.
            // FillBase.cpp:1453-1462
            let contour_length = *contour_params.last().unwrap();
            for &ip in &contour_intersection_points {
                let next_on_contour = out.map_infill_end_point_to_boundary[ip].next_on_contour;
                if next_on_contour == ip {
                    out.map_infill_end_point_to_boundary[ip].contour_not_taken_length_prev =
                        contour_length;
                    out.map_infill_end_point_to_boundary[ip].contour_not_taken_length_next =
                        contour_length;
                } else {
                    let prev_on_contour =
                        out.map_infill_end_point_to_boundary[ip].prev_on_contour;
                    let ip_param = out.map_infill_end_point_to_boundary[ip].param;
                    let prev_param = out.map_infill_end_point_to_boundary[prev_on_contour].param;
                    let next_param = out.map_infill_end_point_to_boundary[next_on_contour].param;
                    out.map_infill_end_point_to_boundary[ip].contour_not_taken_length_prev =
                        closed_contour_distance_ccw(prev_param, ip_param, contour_length);
                    out.map_infill_end_point_to_boundary[ip].contour_not_taken_length_next =
                        closed_contour_distance_ccw(ip_param, next_param, contour_length);
                }
            }

            out.boundary[idx_contour] = contour_dst;
            out.boundary_params[idx_contour] = contour_params;
            boundary_intersection_points[idx_contour] = contour_intersection_points;
        }

        // Mark the points and segments of split out.boundary as consumed if they are very close to some of the infill line.
        // FillBase.cpp:1474-1483
        {
            // @supermerill used 2. * scale_(spacing)
            let clip_distance = 1.7 * scale_f(spacing);
            // Allow a bit of overlap.
            let distance_colliding = 0.8 * scale_f(spacing);
            mark_boundary_segments_touching_infill(
                &out.boundary,
                &out.boundary_params,
                &boundary_intersection_points,
                &mut out.map_infill_end_point_to_boundary,
                bbox,
                infill_ordered,
                clip_distance,
                distance_colliding,
            );
        }
    }

    out
}

// FillBase.cpp:1164-1173 — connect_infill(ExPolygon overload).
pub fn connect_infill_expolygon(
    infill_ordered: Vec<Polyline>,
    boundary_src: &ExPolygon,
    polylines_out: &mut Vec<Polyline>,
    spacing: f64,
    params: &FillParams,
) {
    let mut polygons_src: Vec<&Polygon> = Vec::with_capacity(boundary_src.holes.len() + 1);
    polygons_src.push(&boundary_src.contour);
    for polygon in &boundary_src.holes {
        polygons_src.push(polygon);
    }
    // get_extents(boundary_src.contour) — bounding box of the outer contour points.
    let bbox = BoundingBox::from_points(&boundary_src.contour.points);
    connect_infill(infill_ordered, &polygons_src, &bbox, polylines_out, spacing, params);
}

// FillBase.cpp:1175-1182 — connect_infill(Polygons overload).
pub fn connect_infill_polygons(
    infill_ordered: Vec<Polyline>,
    boundary_src: &[Polygon],
    bbox: &BoundingBox,
    polylines_out: &mut Vec<Polyline>,
    spacing: f64,
    params: &FillParams,
) {
    let polygons_src: Vec<&Polygon> = boundary_src.iter().collect();
    connect_infill(infill_ordered, &polygons_src, bbox, polylines_out, spacing, params);
}

// FillBase.cpp:1501-1733 — Fill::connect_infill (the std::vector<const Polygon*> overload).
pub fn connect_infill(
    mut infill_ordered: Vec<Polyline>,
    boundary_src: &[&Polygon],
    bbox: &BoundingBox,
    polylines_out: &mut Vec<Polyline>,
    spacing: f64,
    params: &FillParams,
) {
    // FillBase.cpp:1507-1508
    let anchor_length = scale_f(params.anchor_length);
    let anchor_length_max = scale_f(params.anchor_length_max);

    // FillBase.cpp:1515
    let mut graph = create_boundary_infill_graph(&infill_ordered, boundary_src, bbox, spacing);

    // FillBase.cpp:1517-1518
    let mut merged_with: Vec<usize> = (0..infill_ordered.len()).collect();

    // FillBase.cpp:1534
    let line_half_width = 0.5 * scale_f(spacing);

    // FillBase.cpp:1611-1622
    struct Arc {
        intersection: usize,
        arc_length: f64,
    }
    let mut arches: Vec<Arc> = Vec::new();
    if !params.dont_sort {
        arches.reserve(graph.map_infill_end_point_to_boundary.len());
        for cp in 0..graph.map_infill_end_point_to_boundary.len() {
            let c = &graph.map_infill_end_point_to_boundary[cp];
            if c.contour_idx != BOUNDARY_IDX_UNCONNECTED
                && c.next_on_contour != cp
                && could_connect_next(&graph.map_infill_end_point_to_boundary, cp)
            {
                let contour_idx = c.contour_idx;
                let next = c.next_on_contour;
                let len = path_length_along_contour_ccw(
                    &graph.map_infill_end_point_to_boundary,
                    cp,
                    next,
                    *graph.boundary_params[contour_idx].last().unwrap(),
                );
                arches.push(Arc {
                    intersection: cp,
                    arc_length: len,
                });
            }
        }
        arches.sort_by(|l, r| l.arc_length.partial_cmp(&r.arc_length).unwrap_or(std::cmp::Ordering::Equal));
    }

    // FillBase.cpp:1625-1661
    for arc in &arches {
        if !graph.map_infill_end_point_to_boundary[arc.intersection].consumed
            && !graph.map_infill_end_point_to_boundary
                [graph.map_infill_end_point_to_boundary[arc.intersection].next_on_contour]
                .consumed
        {
            let cp1 = arc.intersection;
            let cp2 = graph.map_infill_end_point_to_boundary[arc.intersection].next_on_contour;
            let polyline_idx1 = get_and_update_merged_with(&mut merged_with, cp1 / 2);
            let polyline_idx2 = get_and_update_merged_with(&mut merged_with, cp2 / 2);
            let contour_idx = graph.map_infill_end_point_to_boundary[cp1].contour_idx;
            if polyline_idx1 != polyline_idx2 {
                if arc.arc_length < anchor_length_max {
                    // Not closing a loop, connecting the lines.
                    let cp1_point = graph.boundary[contour_idx]
                        [graph.map_infill_end_point_to_boundary[cp1].point_idx];
                    let cp2_point = graph.boundary[contour_idx]
                        [graph.map_infill_end_point_to_boundary[cp2].point_idx];
                    if Some(&cp1_point) == infill_ordered[polyline_idx1].points.first() {
                        infill_ordered[polyline_idx1].reverse();
                    }
                    if Some(&cp2_point) == infill_ordered[polyline_idx2].points.last() {
                        infill_ordered[polyline_idx2].reverse();
                    }
                    // take(polyline1, polyline2, contour, cp1, cp2, false)
                    {
                        let polyline2_clone = infill_ordered[polyline_idx2].clone();
                        let contour = graph.boundary[contour_idx].clone();
                        take_cp(
                            &mut graph.map_infill_end_point_to_boundary,
                            &mut infill_ordered[polyline_idx1],
                            &polyline2_clone,
                            &contour,
                            cp1,
                            cp2,
                            false,
                        );
                    }
                    // Mark the second polygon as merged with the first one.
                    if polyline_idx2 < polyline_idx1 {
                        infill_ordered[polyline_idx2] =
                            std::mem::take(&mut infill_ordered[polyline_idx1]);
                        infill_ordered[polyline_idx1].points.clear();
                        merged_with[polyline_idx1] = merged_with[polyline_idx2];
                    } else {
                        infill_ordered[polyline_idx2].points.clear();
                        merged_with[polyline_idx2] = merged_with[polyline_idx1];
                    }
                } else if anchor_length > SCALED_EPSILON {
                    // Move along the perimeter, but don't take the whole arc.
                    let contour = graph.boundary[contour_idx].clone();
                    let contour_params = graph.boundary_params[contour_idx].clone();
                    take_limited(
                        &mut graph.map_infill_end_point_to_boundary,
                        &mut infill_ordered[polyline_idx1],
                        &contour,
                        &contour_params,
                        cp1,
                        cp2,
                        false,
                        anchor_length,
                        line_half_width,
                    );
                    take_limited(
                        &mut graph.map_infill_end_point_to_boundary,
                        &mut infill_ordered[polyline_idx2],
                        &contour,
                        &contour_params,
                        cp2,
                        cp1,
                        true,
                        anchor_length,
                        line_half_width,
                    );
                }
            }
        }
    }

    // Connect the remaining open infill lines to the perimeter lines if possible.
    // FillBase.cpp:1664-1727
    for contour_point in 0..graph.map_infill_end_point_to_boundary.len() {
        if !graph.map_infill_end_point_to_boundary[contour_point].consumed
            && graph.map_infill_end_point_to_boundary[contour_point].contour_idx
                != BOUNDARY_IDX_UNCONNECTED
        {
            let contour_idx = graph.map_infill_end_point_to_boundary[contour_point].contour_idx;
            let contour_back = *graph.boundary_params[contour_idx].last().unwrap();

            let lprev = if could_connect_prev(&graph.map_infill_end_point_to_boundary, contour_point)
            {
                let prev = graph.map_infill_end_point_to_boundary[contour_point].prev_on_contour;
                path_length_along_contour_ccw(
                    &graph.map_infill_end_point_to_boundary,
                    prev,
                    contour_point,
                    contour_back,
                )
            } else {
                f64::MAX
            };
            let lnext = if could_connect_next(&graph.map_infill_end_point_to_boundary, contour_point)
            {
                let next = graph.map_infill_end_point_to_boundary[contour_point].next_on_contour;
                path_length_along_contour_ccw(
                    &graph.map_infill_end_point_to_boundary,
                    contour_point,
                    next,
                    contour_back,
                )
            } else {
                f64::MAX
            };
            let polyline_idx = get_and_update_merged_with(&mut merged_with, contour_point / 2);

            let mut connected = false;
            for l in [lprev.min(lnext), lprev.max(lnext)] {
                if l == f64::MAX || l > anchor_length_max {
                    break;
                }
                // Take the complete contour.
                let reversed = l == lprev;
                let cp2 = if reversed {
                    graph.map_infill_end_point_to_boundary[contour_point].prev_on_contour
                } else {
                    graph.map_infill_end_point_to_boundary[contour_point].next_on_contour
                };
                // Identify which end of the polyline touches the boundary.
                let polyline_idx2 = get_and_update_merged_with(&mut merged_with, cp2 / 2);
                if polyline_idx == polyline_idx2 {
                    // Try the other side.
                    continue;
                }
                // Not closing a loop.
                let cp_point = graph.boundary[contour_idx]
                    [graph.map_infill_end_point_to_boundary[contour_point].point_idx];
                if Some(&cp_point) == infill_ordered[polyline_idx].points.first() {
                    infill_ordered[polyline_idx].reverse();
                }
                let cp2_point =
                    graph.boundary[contour_idx][graph.map_infill_end_point_to_boundary[cp2].point_idx];
                if Some(&cp2_point) == infill_ordered[polyline_idx2].points.last() {
                    infill_ordered[polyline_idx2].reverse();
                }
                {
                    let polyline2_clone = infill_ordered[polyline_idx2].clone();
                    let contour = graph.boundary[contour_idx].clone();
                    take_cp(
                        &mut graph.map_infill_end_point_to_boundary,
                        &mut infill_ordered[polyline_idx],
                        &polyline2_clone,
                        &contour,
                        contour_point,
                        cp2,
                        reversed,
                    );
                }
                if polyline_idx < polyline_idx2 {
                    // Mark the second polyline as merged with the first one.
                    merged_with[polyline_idx2] = polyline_idx;
                    infill_ordered[polyline_idx2].points.clear();
                } else {
                    // Mark the first polyline as merged with the second one.
                    merged_with[polyline_idx] = polyline_idx2;
                    infill_ordered[polyline_idx2] =
                        std::mem::take(&mut infill_ordered[polyline_idx]);
                    infill_ordered[polyline_idx].points.clear();
                }
                connected = true;
                break;
            }
            if !connected && anchor_length > SCALED_EPSILON {
                // Let's take the longer now, as this improves the chance of another hook to be placed on the other side of this contour point.
                let cp = &graph.map_infill_end_point_to_boundary[contour_point];
                let l = cp.contour_not_taken_length_prev.max(cp.contour_not_taken_length_next);
                if l > SCALED_EPSILON {
                    let take_prev = cp.contour_not_taken_length_prev > cp.contour_not_taken_length_next;
                    let target = if take_prev {
                        graph.map_infill_end_point_to_boundary[contour_point].prev_on_contour
                    } else {
                        graph.map_infill_end_point_to_boundary[contour_point].next_on_contour
                    };
                    let contour = graph.boundary[contour_idx].clone();
                    let contour_params = graph.boundary_params[contour_idx].clone();
                    take_limited(
                        &mut graph.map_infill_end_point_to_boundary,
                        &mut infill_ordered[polyline_idx],
                        &contour,
                        &contour_params,
                        contour_point,
                        target,
                        take_prev,
                        anchor_length,
                        line_half_width,
                    );
                }
            }
        }
    }

    // FillBase.cpp:1729-1732
    polylines_out.reserve(infill_ordered.iter().filter(|pl| !pl.points.is_empty()).count());
    for pl in infill_ordered.into_iter() {
        if !pl.points.is_empty() {
            polylines_out.push(pl);
        }
    }
}

// FillBase.cpp:1520-1532 — get_and_update_merged_with.
fn get_and_update_merged_with(merged_with: &mut [usize], polyline_idx: usize) -> usize {
    let mut last = polyline_idx;
    loop {
        let lower = merged_with[last];
        if lower == last {
            merged_with[polyline_idx] = last;
            return last;
        }
        last = lower;
    }
}

// The extended bounding box of the whole object that covers any rotation of every layer.
// FillBase.cpp:1490-1499
pub fn extended_object_bounding_box(bounding_box: &BoundingBox) -> BoundingBox {
    let mut out = bounding_box.clone();
    out.merge_point(Point::new(out.min.y(), out.min.x()));
    out.merge_point(Point::new(out.max.y(), out.max.x()));
    // The bounding box is scaled by sqrt(2.) to ensure that the bounding box
    // covers any possible rotations.
    out.scaled(2.0_f64.sqrt())
}

// Calculate a new spacing to fill width with possibly integer number of lines,
// the first and last line being centered at the interval ends.
// FillBase.cpp:179-195
pub fn adjust_solid_spacing(width: Coord, distance: Coord) -> Coord {
    debug_assert!(width >= 0);
    debug_assert!(distance > 0);
    // floor(width / distance)
    let number_of_intervals = ((width as f64 - EPSILON) / distance as f64) as Coord;
    let mut distance_new = if number_of_intervals == 0 {
        distance
    } else {
        ((width as f64 - EPSILON) / number_of_intervals as f64) as Coord
    };
    let factor = distance_new as f64 / distance as f64;
    // How much could the extrusion width be increased? By 20%.
    let factor_max = 1.2;
    if factor > factor_max {
        distance_new = (distance as f64 * factor_max + 0.5).floor() as Coord;
    }
    distance_new
}

// Extend the infill lines along the perimeters, this is mainly useful for grid aligned support, where a perimeter line may be nearly
// aligned with the infill lines.
// FillBase.cpp:1737-1853
fn base_support_extend_infill_lines(
    infill: &mut [Polyline],
    graph: &mut BoundaryInfillGraph,
    spacing: f64,
    params: &FillParams,
) {
    let line_spacing = scale_f(spacing) / params.density as f64;
    // Maximum deviation perpendicular to the infill line to allow merging as a continuation of the same infill line.
    let dist_max_x = (line_spacing * 0.33) as Coord;
    // Minimum length of the arc away from the infill end point to allow merging as a continuation of the same infill line.
    let dist_min_y = (line_spacing * 0.5) as Coord;

    let n = graph.map_infill_end_point_to_boundary.len();
    for cp_idx in 0..n {
        let contour_idx = graph.map_infill_end_point_to_boundary[cp_idx].contour_idx;
        let pt = {
            let cp = &graph.map_infill_end_point_to_boundary[cp_idx];
            graph.boundary[contour_idx][cp.point_idx]
        };
        let first = graph.first(cp_idx);
        let mut extend_next_idx: i64 = -1;
        let mut extend_prev_idx: i64 = -1;
        let mut dist_y_prev: Coord = 0;
        let mut dist_y_next: Coord = 0;
        let mut arc_len_prev: f64 = 0.0;
        let mut arc_len_next: f64 = 0.0;

        // FillBase.cpp:1764-1788
        if !graph.next_vertical(cp_idx) {
            let contour = &graph.boundary[contour_idx];
            let contour_param = &graph.boundary_params[contour_idx];
            let cp_point_idx = graph.map_infill_end_point_to_boundary[cp_idx].point_idx;
            let next_point_idx = graph.map_infill_end_point_to_boundary
                [graph.map_infill_end_point_to_boundary[cp_idx].next_on_contour]
                .point_idx;
            let mut i = cp_point_idx;
            let mut j = next_idx_modulo(i, contour.len());
            while j != next_point_idx {
                let p2 = contour[j];
                if (p2.x() - pt.x()).abs() > dist_max_x {
                    break;
                }
                i = j;
                j = next_idx_modulo(j, contour.len());
            }
            if i != cp_point_idx {
                let p2 = contour[i];
                let mut dist_y = p2.y() - pt.y();
                if first {
                    dist_y = -dist_y;
                }
                if dist_y > dist_min_y {
                    arc_len_next = closed_contour_distance_ccw(
                        contour_param[cp_point_idx],
                        contour_param[i],
                        *contour_param.last().unwrap(),
                    );
                    if arc_len_next
                        < graph.map_infill_end_point_to_boundary[cp_idx]
                            .contour_not_taken_length_next
                    {
                        extend_next_idx = i as i64;
                        dist_y_next = dist_y;
                    }
                }
            }
        }

        // FillBase.cpp:1790-1814
        if !graph.prev_vertical(cp_idx) {
            let contour = &graph.boundary[contour_idx];
            let contour_param = &graph.boundary_params[contour_idx];
            let cp_point_idx = graph.map_infill_end_point_to_boundary[cp_idx].point_idx;
            let prev_point_idx = graph.map_infill_end_point_to_boundary
                [graph.map_infill_end_point_to_boundary[cp_idx].prev_on_contour]
                .point_idx;
            let mut i = cp_point_idx;
            let mut j = prev_idx_modulo(i, contour.len());
            while j != prev_point_idx {
                let p2 = contour[j];
                if (p2.x() - pt.x()).abs() > dist_max_x {
                    break;
                }
                i = j;
                j = prev_idx_modulo(j, contour.len());
            }
            if i != cp_point_idx {
                let p2 = contour[i];
                let mut dist_y = p2.y() - pt.y();
                if first {
                    dist_y = -dist_y;
                }
                if dist_y > dist_min_y {
                    arc_len_prev = closed_contour_distance_ccw(
                        contour_param[i],
                        contour_param[cp_point_idx],
                        *contour_param.last().unwrap(),
                    );
                    if arc_len_prev
                        < graph.map_infill_end_point_to_boundary[cp_idx]
                            .contour_not_taken_length_prev
                    {
                        extend_prev_idx = i as i64;
                        dist_y_prev = dist_y;
                    }
                }
            }
        }

        // FillBase.cpp:1816-1818 — Which side to move the point?
        if extend_prev_idx >= 0 && extend_next_idx >= 0 {
            if dist_y_prev < dist_y_next {
                extend_prev_idx = -1;
            } else {
                extend_next_idx = -1;
            }
        }

        // FillBase.cpp:1822-1851
        let infill_line = &mut infill[cp_idx / 2];
        if extend_prev_idx >= 0 {
            let contour = graph.boundary[contour_idx].clone();
            let contour_param = &graph.boundary_params[contour_idx];
            let cp_point_idx = graph.map_infill_end_point_to_boundary[cp_idx].point_idx;
            if first {
                infill_line.reverse();
            }
            take_cw_full(infill_line, &contour, cp_point_idx, extend_prev_idx as usize);
            if first {
                infill_line.reverse();
            }
            graph.map_infill_end_point_to_boundary[cp_idx].point_idx = extend_prev_idx as usize;
            let new_point_idx = extend_prev_idx as usize;
            let prev_on_contour =
                graph.map_infill_end_point_to_boundary[cp_idx].prev_on_contour;
            if graph.map_infill_end_point_to_boundary[cp_idx].prev_trimmed {
                graph.map_infill_end_point_to_boundary[cp_idx].contour_not_taken_length_prev -=
                    arc_len_prev;
            } else {
                let v = closed_contour_distance_ccw(
                    contour_param[graph.map_infill_end_point_to_boundary[prev_on_contour].point_idx],
                    contour_param[new_point_idx],
                    *contour_param.last().unwrap(),
                );
                graph.map_infill_end_point_to_boundary[cp_idx].contour_not_taken_length_prev = v;
                graph.map_infill_end_point_to_boundary[prev_on_contour]
                    .contour_not_taken_length_next = v;
            }
            graph.map_infill_end_point_to_boundary[cp_idx].trim_next(0.0);
            let next_on_contour =
                graph.map_infill_end_point_to_boundary[cp_idx].next_on_contour;
            graph.map_infill_end_point_to_boundary[next_on_contour].prev_trimmed = true;
        } else if extend_next_idx >= 0 {
            let contour = graph.boundary[contour_idx].clone();
            let contour_param = &graph.boundary_params[contour_idx];
            let cp_point_idx = graph.map_infill_end_point_to_boundary[cp_idx].point_idx;
            if first {
                infill_line.reverse();
            }
            take_ccw_full(infill_line, &contour, cp_point_idx, extend_next_idx as usize);
            if first {
                infill_line.reverse();
            }
            graph.map_infill_end_point_to_boundary[cp_idx].point_idx = extend_next_idx as usize;
            let new_point_idx = extend_next_idx as usize;
            graph.map_infill_end_point_to_boundary[cp_idx].trim_prev(0.0);
            let prev_on_contour =
                graph.map_infill_end_point_to_boundary[cp_idx].prev_on_contour;
            graph.map_infill_end_point_to_boundary[prev_on_contour].next_trimmed = true;
            let next_on_contour =
                graph.map_infill_end_point_to_boundary[cp_idx].next_on_contour;
            if graph.map_infill_end_point_to_boundary[cp_idx].next_trimmed {
                graph.map_infill_end_point_to_boundary[cp_idx].contour_not_taken_length_next -=
                    arc_len_next;
            } else {
                let v = closed_contour_distance_ccw(
                    contour_param[new_point_idx],
                    contour_param[graph.map_infill_end_point_to_boundary[next_on_contour].point_idx],
                    *contour_param.last().unwrap(),
                );
                graph.map_infill_end_point_to_boundary[cp_idx].contour_not_taken_length_next = v;
                graph.map_infill_end_point_to_boundary[next_on_contour]
                    .contour_not_taken_length_prev = v;
            }
        }
    }
}

// Side of the band for emit_loops_in_band.
// FillBase.cpp:1915-1920
#[derive(Clone, Copy, PartialEq, Eq)]
enum Side {
    Left,
    Right,
    Mid,
    Unknown,
}

// FillBase.cpp:1922-1925
#[derive(Clone, Copy, PartialEq, Eq)]
enum InOutBand {
    Entering,
    Leaving,
}

// FillBase.cpp:1927-1996 — the `State` machine of emit_loops_in_band.
struct EmitState<'a> {
    m_left: Coord,
    m_right: Coord,
    m_min_length: f64,
    m_polylines_out: &'a mut Vec<Polyline>,
    m_polyline: Polyline,
    m_polyline_end: usize,
    side1: Side,
    side2: Side,
}

impl<'a> EmitState<'a> {
    fn new(
        left: Coord,
        right: Coord,
        min_length: f64,
        polylines_out: &'a mut Vec<Polyline>,
    ) -> Self {
        Self {
            m_left: left,
            m_right: right,
            m_min_length: min_length,
            m_polylines_out: polylines_out,
            m_polyline: Polyline::new(),
            m_polyline_end: 0,
            side1: Side::Unknown,
            side2: Side::Unknown,
        }
    }

    // FillBase.cpp:1932-1935
    fn add_inner_point(&mut self, p: Point) {
        self.m_polyline.points.push(p);
    }

    // FillBase.cpp:1937-1941
    fn add_outer_point(&mut self, p: Point) {
        if self.m_polyline_end > 0 {
            self.m_polyline.points.push(p);
        }
    }

    // FillBase.cpp:1943-1969
    fn add_interpolated_point(&mut self, p1: Point, p2: Point, side: Side, inout: InOutBand) {
        let x = if side == Side::Left {
            self.m_left
        } else {
            self.m_right
        };
        let y = p1.y()
            + ((x - p1.x()) as f64 * (p2.y() - p1.y()) as f64 / (p2.x() - p1.x()) as f64) as Coord;

        if inout == InOutBand::Leaving {
            self.m_polyline_end = self.m_polyline.size();
            self.m_polyline.points.push(Point::new(x, y));
        } else {
            // Entering
            if self.m_polyline_end > 0 {
                if (self.side1 == Side::Left)
                    == ((y - self.m_polyline.points[self.m_polyline_end].y()) < 0)
                {
                    // Emit the vertical segment. Remove the point, where the source contour was split the last time at m_left / m_right.
                    self.m_polyline.points.remove(self.m_polyline_end);
                } else {
                    // Don't emit the vertical segment, split the contour.
                    self.finalize();
                    self.m_polyline.points.push(Point::new(x, y));
                }
                self.m_polyline_end = 0;
            } else {
                self.m_polyline.points.push(Point::new(x, y));
            }
        }
    }

    // FillBase.cpp:1971-1981
    fn finalize(&mut self) {
        self.m_polyline.points.truncate(self.m_polyline_end);
        if !self.m_polyline.points.is_empty() {
            if !self.m_polylines_out.is_empty() && {
                let back = self.m_polylines_out.last().unwrap();
                let d = Point::new(
                    back.points.last().unwrap().x() - self.m_polyline.points[0].x(),
                    back.points.last().unwrap().y() - self.m_polyline.points[0].y(),
                );
                (d.x() as i128 * d.x() as i128 + d.y() as i128 * d.y() as i128)
                    < SCALED_EPSILON as i128
            } {
                let extra: Vec<Point> = self.m_polyline.points[1..].to_vec();
                self.m_polylines_out.last_mut().unwrap().points.extend(extra);
            } else if self.m_polyline.length() > self.m_min_length {
                self.m_polylines_out
                    .push(std::mem::replace(&mut self.m_polyline, Polyline::new()));
            }
            self.m_polyline.clear();
        }
    }
}

// Called by Fill::connect_base_support() as part of the sparse support infill generator.
// Emit contour loops tracing the contour from tbegin to tend inside a band of (left, right).
// FillBase.cpp:1858-2042
#[allow(clippy::too_many_arguments)]
fn emit_loops_in_band(
    left: Coord,
    right: Coord,
    contour: &[Point],
    contour_params: &[f64],
    tbegin: f64,
    tend: f64,
    min_length: f64,
    polylines_out: &mut Vec<Polyline>,
) {
    // Find iterators of the range of segments, where the first and last segment contains tbegin and tend.
    // FillBase.cpp:1883-1895
    let mut ibegin;
    let mut iend;
    {
        let mut it_begin = lower_bound_f64(contour_params, tbegin);
        let it_end = lower_bound_f64(contour_params, tend);
        if contour_params[it_begin] != tbegin {
            it_begin -= 1;
        }
        ibegin = it_begin;
        iend = it_end;
    }

    if ibegin == contour.len() {
        ibegin = 0;
    }
    if iend == contour.len() {
        iend = 0;
    }

    // Trim the start and end segment to calculate start and end points.
    // FillBase.cpp:1903-1912
    let pbegin;
    let pend;
    {
        let t1 = contour_params[ibegin];
        let t2 = *next_value_modulo(ibegin, contour_params);
        pbegin = lerp(
            contour[ibegin],
            *next_value_modulo(ibegin, contour),
            (tbegin - t1) / (t2 - t1),
        );
        let t1 = contour_params[iend];
        let t2 = *prev_value_modulo(iend, contour_params);
        pend = lerp(
            contour[iend],
            *prev_value_modulo(iend, contour),
            (tend - t1) / (t2 - t1),
        );
    }

    let mut state = EmitState::new(left, right, min_length, polylines_out);

    // FillBase.cpp:2000-2007
    let side = |p: Point| -> Side {
        let x = p.x();
        if x < left {
            Side::Left
        } else if x > right {
            Side::Right
        } else {
            Side::Mid
        }
    };
    let mut p1 = pbegin;
    state.side1 = side(p1);
    if state.side1 == Side::Mid {
        state.add_inner_point(p1);
    }

    // FillBase.cpp:2009-2040
    let mut i = ibegin;
    while i != iend {
        let mut inext = i + 1;
        if inext == contour.len() {
            inext = 0;
        }
        let p2 = if inext == iend { pend } else { contour[inext] };
        state.side2 = side(p2);
        if state.side1 == Side::Mid {
            if state.side2 == Side::Mid {
                // Inside the band.
                state.add_inner_point(p2);
            } else {
                // From intisde the band to the outside of the band.
                state.add_interpolated_point(p1, p2, state.side2, InOutBand::Leaving);
                state.add_outer_point(p2);
            }
        } else if state.side2 == Side::Mid {
            // From outside the band into the band.
            state.add_interpolated_point(p1, p2, state.side1, InOutBand::Entering);
            state.add_inner_point(p2);
        } else if state.side1 != state.side2 {
            // Both points outside the band.
            state.add_interpolated_point(p1, p2, state.side1, InOutBand::Entering);
            state.add_interpolated_point(p1, p2, state.side2, InOutBand::Leaving);
        } else {
            // Complete segment is outside.
            state.add_outer_point(p2);
        }
        state.side1 = state.side2;
        p1 = p2;
        i = inext;
    }
    state.finalize();
}

// std::lower_bound on a sorted f64 slice — first index with value >= key.
fn lower_bound_f64(slice: &[f64], key: f64) -> usize {
    let mut lo = 0usize;
    let mut len = slice.len();
    while len > 0 {
        let half = len / 2;
        let mid = lo + half;
        if slice[mid] < key {
            lo = mid + 1;
            len -= half + 1;
        } else {
            len = half;
        }
    }
    lo
}

// To classify perimeter segments connecting infill lines, whether they are required for structural stability of the supports.
// FillBase.cpp:2076-2085
#[derive(Clone, Copy, Default)]
struct SupportArcCost {
    // Connecting one end of an infill line to the other end of the same infill line.
    self_loop: bool,
    // Some of the arc touches some infill line.
    #[allow(dead_code)]
    open: bool,
    // How needed is this arch for support structural stability.
    cost: f64,
}

// FillBase.cpp:2087-2103
fn evaluate_support_arch_cost(pl: &Polyline) -> f64 {
    let front = pl.points[0];
    let back = *pl.points.last().unwrap();

    let mut ymin = front.y();
    let mut ymax = back.y();
    if ymin > ymax {
        std::mem::swap(&mut ymin, &mut ymax);
    }

    let mut dmax = 0.0_f64;
    // Maximum distance in Y axis out of the (ymin, ymax) band and from the (front, back) line.
    let line_a = Vec2d::new(front.x() as f64, front.y() as f64);
    let line_b = Vec2d::new(back.x() as f64, back.y() as f64);
    for pt in &pl.points {
        let pf = Vec2d::new(pt.x() as f64, pt.y() as f64);
        dmax = dmax
            .max(linef_distance_to(line_a, line_b, pf))
            .max(((pt.y() - ymax) as f64).max((ymin - pt.y()) as f64));
    }
    dmax
}

// Costs for prev / next arch of each infill line end point.
// FillBase.cpp:2106-2147
fn evaluate_support_arches(graph: &BoundaryInfillGraph) -> Vec<SupportArcCost> {
    let mut arches: Vec<SupportArcCost> =
        vec![SupportArcCost::default(); graph.map_infill_end_point_to_boundary.len() * 2];

    let mut pl = Polyline::new();
    for cp_idx in 0..graph.map_infill_end_point_to_boundary.len() {
        let infill_line_idx = cp_idx;
        let first = (infill_line_idx & 1) == 0;
        let other_end = if first {
            infill_line_idx + 1
        } else {
            infill_line_idx - 1
        };
        let cp = &graph.map_infill_end_point_to_boundary[cp_idx];
        let contour_idx = cp.contour_idx;
        let cp_point = self_point(graph, cp_idx);
        let cp_point_idx = cp.point_idx;
        let next_on_contour = cp.next_on_contour;
        let prev_on_contour = cp.prev_on_contour;
        let next_point_idx = graph.map_infill_end_point_to_boundary[next_on_contour].point_idx;
        let prev_point_idx = graph.map_infill_end_point_to_boundary[prev_on_contour].point_idx;
        let next_trimmed = cp.next_trimmed;
        let prev_trimmed = cp.prev_trimmed;
        let not_taken_next = cp.contour_not_taken_length_next;
        let not_taken_prev = cp.contour_not_taken_length_prev;

        arches[infill_line_idx * 2].self_loop = prev_on_contour == other_end;
        arches[infill_line_idx * 2].open = prev_trimmed;
        arches[infill_line_idx * 2 + 1].self_loop = next_on_contour == other_end;
        arches[infill_line_idx * 2 + 1].open = next_trimmed;

        if not_taken_next > SCALED_EPSILON {
            pl.clear();
            pl.points.push(cp_point);
            if next_trimmed {
                take_ccw_limited(
                    &mut pl,
                    &graph.boundary[contour_idx],
                    &graph.boundary_params[contour_idx],
                    cp_point_idx,
                    next_point_idx,
                    not_taken_next,
                );
            } else {
                take_ccw_full(
                    &mut pl,
                    &graph.boundary[contour_idx],
                    cp_point_idx,
                    next_point_idx,
                );
            }
            arches[infill_line_idx * 2 + 1].cost = evaluate_support_arch_cost(&pl);
        }

        if not_taken_prev > SCALED_EPSILON {
            pl.clear();
            pl.points.push(cp_point);
            if prev_trimmed {
                take_cw_limited(
                    &mut pl,
                    &graph.boundary[contour_idx],
                    &graph.boundary_params[contour_idx],
                    cp_point_idx,
                    prev_point_idx,
                    not_taken_prev,
                );
            } else {
                take_cw_full(
                    &mut pl,
                    &graph.boundary[contour_idx],
                    cp_point_idx,
                    prev_point_idx,
                );
            }
            arches[infill_line_idx * 2].cost = evaluate_support_arch_cost(&pl);
        }
    }

    arches
}

#[inline]
fn self_point(graph: &BoundaryInfillGraph, idx: usize) -> Point {
    let cp = &graph.map_infill_end_point_to_boundary[idx];
    graph.boundary[cp.contour_idx][cp.point_idx]
}

// FillBase.cpp:2605-2612 — connect_base_support(Polygons overload).
pub fn connect_base_support_polygons(
    infill_ordered: Vec<Polyline>,
    boundary_src: &[Polygon],
    bbox: &BoundingBox,
    polylines_out: &mut Vec<Polyline>,
    spacing: f64,
    params: &FillParams,
) {
    let polygons_src: Vec<&Polygon> = boundary_src.iter().collect();
    connect_base_support(
        infill_ordered,
        &polygons_src,
        bbox,
        polylines_out,
        spacing,
        params,
    );
}

// Both the poly_with_offset and polylines_out are rotated, so the infill lines are strictly vertical.
// FillBase.cpp:2150-2603
pub fn connect_base_support(
    mut infill_ordered: Vec<Polyline>,
    boundary_src: &[&Polygon],
    bbox: &BoundingBox,
    polylines_out: &mut Vec<Polyline>,
    spacing: f64,
    params: &FillParams,
) {
    let mut graph = create_boundary_infill_graph(&infill_ordered, boundary_src, bbox, spacing);

    // FillBase.cpp:2165-2168
    let line_half_width = 0.5 * scale_f(spacing);
    let line_spacing = scale_f(spacing) / params.density as f64;
    let min_arch_length = 1.3 * line_spacing;
    let trim_length = line_half_width * 0.3;

    // FillBase.cpp:2173
    mark_boundary_segments_overlapping_infill(&mut graph, &infill_ordered, scale_f(spacing));

    // Detect loops with zero infill end points connected. Extrude these loops as perimeters.
    // FillBase.cpp:2181-2194
    {
        let mut num_boundary_contour_infill_points = vec![0usize; graph.boundary.len()];
        for cp in &graph.map_infill_end_point_to_boundary {
            num_boundary_contour_infill_points[cp.contour_idx] += 1;
        }
        for i in 0..num_boundary_contour_infill_points.len() {
            if num_boundary_contour_infill_points[i] == 0
                && *graph.boundary_params[i].last().unwrap() > trim_length + 0.5 * line_spacing
            {
                // Emit a perimeter.
                let mut pl = Polyline::from_points(graph.boundary[i].clone());
                let front = pl.points[0];
                pl.points.push(front);
                pl.clip_end(trim_length);
                if pl.size() > 1 {
                    polylines_out.push(pl);
                }
            }
        }
    }

    // FillBase.cpp:2198-2224 — Before processing the boundary arches, emit trimmed arches departing the infill line.
    for cp_idx in 0..graph.map_infill_end_point_to_boundary.len() {
        let cp = &graph.map_infill_end_point_to_boundary[cp_idx];
        let next_on_contour = cp.next_on_contour;
        if next_on_contour != NULL_CP
            && cp.next_trimmed
            && graph.map_infill_end_point_to_boundary[next_on_contour].prev_trimmed
        {
            let first = graph.first(cp_idx);
            let left0 = self_point(&graph, cp_idx).x();
            let mut left = left0;
            let mut right = left0;
            if first {
                left += line_half_width as Coord;
                right += (line_spacing - line_half_width) as Coord;
            } else {
                left -= (line_spacing - line_half_width) as Coord;
                right -= line_half_width as Coord;
            }
            let contour_idx = graph.map_infill_end_point_to_boundary[cp_idx].contour_idx;
            let contour_length = *graph.boundary_params[contour_idx].last().unwrap();
            let cp = &graph.map_infill_end_point_to_boundary[cp_idx];
            let mut param_start = cp.param + cp.contour_not_taken_length_next;
            let mut param_end = graph.map_infill_end_point_to_boundary[next_on_contour].param
                - graph.map_infill_end_point_to_boundary[next_on_contour]
                    .contour_not_taken_length_prev;
            if param_start >= contour_length {
                param_start -= contour_length;
            }
            if param_end < 0.0 {
                param_end += contour_length;
            }
            let contour = graph.boundary[contour_idx].clone();
            let contour_params = graph.boundary_params[contour_idx].clone();
            emit_loops_in_band(
                left,
                right,
                &contour,
                &contour_params,
                param_start,
                param_end,
                0.5 * line_spacing,
                polylines_out,
            );
        }
    }

    // FillBase.cpp:2229
    base_support_extend_infill_lines(&mut infill_ordered, &mut graph, spacing, params);

    // FillBase.cpp:2235-2236
    let mut merged_with: Vec<usize> = (0..infill_ordered.len()).collect();

    // FillBase.cpp:2252-2254 — vertical(dir).
    let vertical = |dir: Direction| dir == Direction::Up || dir == Direction::Down;

    // FillBase.cpp:2338-2360 — Consume all vertical arches.
    for cp_idx in 0..graph.map_infill_end_point_to_boundary.len() {
        if graph.map_infill_end_point_to_boundary[cp_idx].consumed {
            continue;
        }
        let cp_other = BoundaryInfillGraph::other(cp_idx);
        let dir_prev = graph.dir_prev(cp_idx);
        let dir_next = graph.dir_next(cp_idx);
        let prev_on_contour = graph.map_infill_end_point_to_boundary[cp_idx].prev_on_contour;
        let next_on_contour = graph.map_infill_end_point_to_boundary[cp_idx].next_on_contour;
        let can_take_prev = vertical(dir_prev)
            && !graph.map_infill_end_point_to_boundary[prev_on_contour].consumed
            && prev_on_contour != cp_other;
        let can_take_next = vertical(dir_next)
            && !graph.map_infill_end_point_to_boundary[next_on_contour].consumed
            && next_on_contour != cp_other;
        if can_take_prev && (!can_take_next || take_vertical_prev(&graph, cp_idx)) {
            let cp = &graph.map_infill_end_point_to_boundary[cp_idx];
            if !cp.prev_trimmed || cp.contour_not_taken_length_prev > min_arch_length {
                // take previous
                take_next(
                    &mut graph,
                    &mut infill_ordered,
                    &mut merged_with,
                    line_half_width,
                    trim_length,
                    prev_on_contour,
                    false,
                );
            }
        } else if can_take_next {
            let cp = &graph.map_infill_end_point_to_boundary[cp_idx];
            if !cp.next_trimmed || cp.contour_not_taken_length_next > min_arch_length {
                // take next
                take_next(
                    &mut graph,
                    &mut infill_ordered,
                    &mut merged_with,
                    line_half_width,
                    trim_length,
                    cp_idx,
                    true,
                );
            }
        }
    }

    // FillBase.cpp:2366-2369
    let arches = evaluate_support_arches(&graph);
    let cost_low = line_spacing * 1.3;
    let cost_high = line_spacing * 2.0;
    let cost_veryhigh = line_spacing * 3.0;

    // FillBase.cpp:2371-2408 — Connect along the high-cost arches.
    {
        let mut selected: Vec<usize> = Vec::with_capacity(graph.map_infill_end_point_to_boundary.len());
        for cp_idx in 0..graph.map_infill_end_point_to_boundary.len() {
            if graph.map_infill_end_point_to_boundary[cp_idx].consumed {
                continue;
            }
            let cost_prev_idx = cp_idx * 2;
            let cost_next_idx = cp_idx * 2 + 1;
            let mut cost_min = arches[cost_prev_idx].cost;
            let mut cost_max = arches[cost_next_idx].cost;
            if cost_min > cost_max {
                std::mem::swap(&mut cost_min, &mut cost_max);
            }
            if cost_max < cost_low || cost_min > cost_high {
                continue;
            }
            let cost_diff_relative = (cost_max - cost_min) / cost_max;
            if cost_diff_relative < 0.25 {
                continue;
            }
            if arches[cost_prev_idx].cost > cost_low {
                selected.push(cost_prev_idx);
            }
            if arches[cost_next_idx].cost > cost_low {
                selected.push(cost_next_idx);
            }
        }
        // Take the longest arch first.
        selected.sort_by(|&l, &r| {
            arches[r].cost.partial_cmp(&arches[l].cost).unwrap_or(std::cmp::Ordering::Equal)
        });
        for arc in selected {
            let cp_idx = arc / 2;
            if !graph.map_infill_end_point_to_boundary[cp_idx].consumed {
                let prev = (arc & 1) == 0;
                if prev {
                    let prev_on_contour =
                        graph.map_infill_end_point_to_boundary[cp_idx].prev_on_contour;
                    take_next(
                        &mut graph,
                        &mut infill_ordered,
                        &mut merged_with,
                        line_half_width,
                        trim_length,
                        prev_on_contour,
                        false,
                    );
                } else {
                    take_next(
                        &mut graph,
                        &mut infill_ordered,
                        &mut merged_with,
                        line_half_width,
                        trim_length,
                        cp_idx,
                        true,
                    );
                }
            }
        }
    }

    // Traverse the unconnected lines in a zig-zag fashion, left to right only.
    // FillBase.cpp:2486-2499
    for cp_idx in 0..graph.map_infill_end_point_to_boundary.len() {
        if graph.map_infill_end_point_to_boundary[cp_idx].consumed {
            continue;
        }
        let first = (cp_idx & 1) == 0;
        if first {
            let next_on_contour = graph.map_infill_end_point_to_boundary[cp_idx].next_on_contour;
            if get_and_update_merged_with(&mut merged_with, cp_idx / 2)
                != get_and_update_merged_with(&mut merged_with, next_on_contour / 2)
            {
                take_next(
                    &mut graph,
                    &mut infill_ordered,
                    &mut merged_with,
                    line_half_width,
                    trim_length,
                    cp_idx,
                    true,
                );
            }
        } else {
            let prev_on_contour = graph.map_infill_end_point_to_boundary[cp_idx].prev_on_contour;
            if get_and_update_merged_with(&mut merged_with, cp_idx / 2)
                != get_and_update_merged_with(&mut merged_with, prev_on_contour / 2)
            {
                take_next(
                    &mut graph,
                    &mut infill_ordered,
                    &mut merged_with,
                    line_half_width,
                    trim_length,
                    prev_on_contour,
                    false,
                );
            }
        }
    }

    // Add the left caps.
    // FillBase.cpp:2506-2521
    for cp_idx in 0..graph.map_infill_end_point_to_boundary.len() {
        let first = (cp_idx & 1) == 0;
        let other_end = if first { cp_idx + 1 } else { cp_idx - 1 };
        let loop_next = graph.map_infill_end_point_to_boundary[cp_idx].next_on_contour == other_end;
        let loop_prev =
            graph.map_infill_end_point_to_boundary[other_end].next_on_contour == cp_idx;
        if loop_prev && graph.map_infill_end_point_to_boundary[cp_idx].could_take_prev() {
            let prev_on_contour = graph.map_infill_end_point_to_boundary[cp_idx].prev_on_contour;
            take_next(
                &mut graph,
                &mut infill_ordered,
                &mut merged_with,
                line_half_width,
                trim_length,
                prev_on_contour,
                false,
            );
        }
        if loop_next && graph.map_infill_end_point_to_boundary[cp_idx].could_take_next() {
            take_next(
                &mut graph,
                &mut infill_ordered,
                &mut merged_with,
                line_half_width,
                trim_length,
                cp_idx,
                true,
            );
        }
    }

    // Connect with T joints using long arches.
    // FillBase.cpp:2528-2548
    {
        let mut candidates: Vec<usize> = Vec::new();
        for cp_idx in 0..graph.map_infill_end_point_to_boundary.len() {
            if graph.map_infill_end_point_to_boundary[cp_idx].could_take_prev() {
                candidates.push(cp_idx * 2);
            }
            if graph.map_infill_end_point_to_boundary[cp_idx].could_take_next() {
                candidates.push(cp_idx * 2 + 1);
            }
        }
        candidates.sort_by(|&c1, &c2| {
            arches[c2].cost.partial_cmp(&arches[c1].cost).unwrap_or(std::cmp::Ordering::Equal)
        });
        for candidate in candidates {
            let cp_idx = candidate / 2;
            let prev = (candidate & 1) == 0;
            if prev {
                let prev_on_contour =
                    graph.map_infill_end_point_to_boundary[cp_idx].prev_on_contour;
                if graph.map_infill_end_point_to_boundary[cp_idx].could_take_prev()
                    && (get_and_update_merged_with(&mut merged_with, cp_idx / 2)
                        != get_and_update_merged_with(&mut merged_with, prev_on_contour / 2)
                        || arches[candidate].cost > cost_high)
                {
                    take_next(
                        &mut graph,
                        &mut infill_ordered,
                        &mut merged_with,
                        line_half_width,
                        trim_length,
                        prev_on_contour,
                        false,
                    );
                }
            } else {
                let next_on_contour =
                    graph.map_infill_end_point_to_boundary[cp_idx].next_on_contour;
                if graph.map_infill_end_point_to_boundary[cp_idx].could_take_next()
                    && (get_and_update_merged_with(&mut merged_with, cp_idx / 2)
                        != get_and_update_merged_with(&mut merged_with, next_on_contour / 2)
                        || arches[candidate].cost > cost_high)
                {
                    take_next(
                        &mut graph,
                        &mut infill_ordered,
                        &mut merged_with,
                        line_half_width,
                        trim_length,
                        cp_idx,
                        true,
                    );
                }
            }
        }
    }

    // Add very long arches and reasonably long caps even if both of its end points were already consumed.
    // FillBase.cpp:2555-2593
    let cap_cost = 0.5 * line_spacing;
    for cp_idx in 0..graph.map_infill_end_point_to_boundary.len() {
        let cost_prev = arches[cp_idx * 2];
        let cost_next = arches[cp_idx * 2 + 1];
        if graph.map_infill_end_point_to_boundary[cp_idx].contour_not_taken_length_prev
            > SCALED_EPSILON
            && (if cost_prev.self_loop {
                cost_prev.cost > cap_cost
            } else {
                cost_prev.cost > cost_veryhigh
            })
        {
            let contour_idx = graph.map_infill_end_point_to_boundary[cp_idx].contour_idx;
            let cp_point = self_point(&graph, cp_idx);
            let mut pl = Polyline::from_points(vec![cp_point]);
            if !graph.map_infill_end_point_to_boundary[cp_idx].prev_trimmed {
                let not_taken =
                    graph.map_infill_end_point_to_boundary[cp_idx].contour_not_taken_length_prev;
                graph.map_infill_end_point_to_boundary[cp_idx].trim_prev(not_taken - line_half_width);
                let prev_on_contour =
                    graph.map_infill_end_point_to_boundary[cp_idx].prev_on_contour;
                graph.map_infill_end_point_to_boundary[prev_on_contour].trim_next(0.0);
            }
            if graph.map_infill_end_point_to_boundary[cp_idx].contour_not_taken_length_prev
                > SCALED_EPSILON
            {
                let cp_point_idx = graph.map_infill_end_point_to_boundary[cp_idx].point_idx;
                let prev_point_idx = graph.map_infill_end_point_to_boundary
                    [graph.map_infill_end_point_to_boundary[cp_idx].prev_on_contour]
                    .point_idx;
                let not_taken =
                    graph.map_infill_end_point_to_boundary[cp_idx].contour_not_taken_length_prev;
                take_cw_limited(
                    &mut pl,
                    &graph.boundary[contour_idx],
                    &graph.boundary_params[contour_idx],
                    cp_point_idx,
                    prev_point_idx,
                    not_taken,
                );
                graph.map_infill_end_point_to_boundary[cp_idx].trim_prev(0.0);
                pl.clip_start(line_half_width);
                polylines_out.push(pl);
            }
        }
        if graph.map_infill_end_point_to_boundary[cp_idx].contour_not_taken_length_next
            > SCALED_EPSILON
            && (if cost_next.self_loop {
                cost_next.cost > cap_cost
            } else {
                cost_next.cost > cost_veryhigh
            })
        {
            let contour_idx = graph.map_infill_end_point_to_boundary[cp_idx].contour_idx;
            let cp_point = self_point(&graph, cp_idx);
            let mut pl = Polyline::from_points(vec![cp_point]);
            if !graph.map_infill_end_point_to_boundary[cp_idx].next_trimmed {
                let not_taken =
                    graph.map_infill_end_point_to_boundary[cp_idx].contour_not_taken_length_next;
                graph.map_infill_end_point_to_boundary[cp_idx].trim_next(not_taken - line_half_width);
                let next_on_contour =
                    graph.map_infill_end_point_to_boundary[cp_idx].next_on_contour;
                graph.map_infill_end_point_to_boundary[next_on_contour].trim_prev(0.0);
            }
            if graph.map_infill_end_point_to_boundary[cp_idx].contour_not_taken_length_next
                > SCALED_EPSILON
            {
                let cp_point_idx = graph.map_infill_end_point_to_boundary[cp_idx].point_idx;
                let next_point_idx = graph.map_infill_end_point_to_boundary
                    [graph.map_infill_end_point_to_boundary[cp_idx].next_on_contour]
                    .point_idx;
                let not_taken =
                    graph.map_infill_end_point_to_boundary[cp_idx].contour_not_taken_length_next;
                take_ccw_limited(
                    &mut pl,
                    &graph.boundary[contour_idx],
                    &graph.boundary_params[contour_idx],
                    cp_point_idx,
                    next_point_idx,
                    not_taken,
                );
                graph.map_infill_end_point_to_boundary[cp_idx].trim_next(0.0);
                pl.clip_start(line_half_width);
                polylines_out.push(pl);
            }
        }
    }

    // FillBase.cpp:2599-2602
    polylines_out.reserve(infill_ordered.iter().filter(|pl| !pl.points.is_empty()).count());
    for pl in infill_ordered.into_iter() {
        if !pl.points.is_empty() {
            polylines_out.push(pl);
        }
    }
}

// FillBase.cpp:2256-2262 — When both left / right arch connected to cp is vertical, which one to take?
fn take_vertical_prev(graph: &BoundaryInfillGraph, cp_idx: usize) -> bool {
    let cp = &graph.map_infill_end_point_to_boundary[cp_idx];
    if cp.prev_trimmed == cp.next_trimmed {
        // Both are either trimmed or not trimmed. Take the longer contour.
        cp.contour_not_taken_length_prev > cp.contour_not_taken_length_next
    } else {
        // One is trimmed, the other is not trimmed. Take the not trimmed.
        !cp.prev_trimmed && cp.next_trimmed
    }
}

// FillBase.cpp:2273-2334 — take_next lambda.
#[allow(clippy::too_many_arguments)]
fn take_next(
    graph: &mut BoundaryInfillGraph,
    infill_ordered: &mut [Polyline],
    merged_with: &mut [usize],
    line_half_width: f64,
    trim_length: f64,
    cp_idx: usize,
    take_first: bool,
) {
    // Indices of the polylines to be connected by a perimeter segment.
    let cp1 = cp_idx;
    let cp2 = graph.map_infill_end_point_to_boundary[cp_idx].next_on_contour;
    if if take_first {
        graph.map_infill_end_point_to_boundary[cp1].consumed
    } else {
        graph.map_infill_end_point_to_boundary[cp2].consumed
    } {
        return;
    }
    let polyline_idx1 = get_and_update_merged_with(merged_with, cp1 / 2);
    let polyline_idx2 = get_and_update_merged_with(merged_with, cp2 / 2);
    let contour_idx = graph.map_infill_end_point_to_boundary[cp1].contour_idx;
    let contour_params_back = *graph.boundary_params[contour_idx].last().unwrap();

    let mut trimmed = if take_first {
        graph.map_infill_end_point_to_boundary[cp1].next_trimmed
    } else {
        graph.map_infill_end_point_to_boundary[cp2].prev_trimmed
    };
    if !trimmed {
        // Trim the end if closing a loop or making a T-joint.
        trimmed = cp1 == cp2
            || polyline_idx1 == polyline_idx2
            || (if take_first {
                graph.map_infill_end_point_to_boundary[cp2].consumed
            } else {
                graph.map_infill_end_point_to_boundary[cp1].consumed
            });
        if !trimmed {
            let cp1_first = (cp1 & 1) == 0;
            let cp1_other = if cp1_first { cp1 + 1 } else { cp1 - 1 };
            // Self loop, connecting the end points of the same infill line.
            trimmed = cp2 == cp1_other;
        }
        if trimmed {
            // Single end point on a contour, or a self loop, or closing a chain of infill lines.
            let len = if cp1 == cp2 {
                contour_params_back
            } else {
                path_length_along_contour_ccw(
                    &graph.map_infill_end_point_to_boundary,
                    cp1,
                    cp2,
                    contour_params_back,
                )
            };
            if take_first {
                graph.map_infill_end_point_to_boundary[cp1]
                    .trim_next((0.0_f64).max(len - trim_length - SCALED_EPSILON));
                graph.map_infill_end_point_to_boundary[cp2].trim_prev(0.0);
            } else {
                graph.map_infill_end_point_to_boundary[cp1].trim_next(0.0);
                graph.map_infill_end_point_to_boundary[cp2]
                    .trim_prev((0.0_f64).max(len - trim_length - SCALED_EPSILON));
            }
        }
    }
    if trimmed {
        let contour = graph.boundary[contour_idx].clone();
        let contour_params = graph.boundary_params[contour_idx].clone();
        if take_first {
            take_limited(
                &mut graph.map_infill_end_point_to_boundary,
                &mut infill_ordered[polyline_idx1],
                &contour,
                &contour_params,
                cp1,
                cp2,
                false,
                1e10,
                line_half_width,
            );
        } else {
            take_limited(
                &mut graph.map_infill_end_point_to_boundary,
                &mut infill_ordered[polyline_idx2],
                &contour,
                &contour_params,
                cp2,
                cp1,
                true,
                1e10,
                line_half_width,
            );
        }
    } else if !graph.map_infill_end_point_to_boundary[cp1].consumed
        && !graph.map_infill_end_point_to_boundary[cp2].consumed
    {
        let cp1_point =
            graph.boundary[contour_idx][graph.map_infill_end_point_to_boundary[cp1].point_idx];
        let cp2_point =
            graph.boundary[contour_idx][graph.map_infill_end_point_to_boundary[cp2].point_idx];
        if Some(&cp1_point) == infill_ordered[polyline_idx1].points.first() {
            infill_ordered[polyline_idx1].reverse();
        }
        if Some(&cp2_point) == infill_ordered[polyline_idx2].points.last() {
            infill_ordered[polyline_idx2].reverse();
        }
        let polyline2_clone = infill_ordered[polyline_idx2].clone();
        let contour = graph.boundary[contour_idx].clone();
        take_cp(
            &mut graph.map_infill_end_point_to_boundary,
            &mut infill_ordered[polyline_idx1],
            &polyline2_clone,
            &contour,
            cp1,
            cp2,
            false,
        );
        // Mark the second polygon as merged with the first one.
        if polyline_idx2 < polyline_idx1 {
            infill_ordered[polyline_idx2] = std::mem::take(&mut infill_ordered[polyline_idx1]);
            infill_ordered[polyline_idx1].points.clear();
            merged_with[polyline_idx1] = merged_with[polyline_idx2];
        } else {
            infill_ordered[polyline_idx2].points.clear();
            merged_with[polyline_idx2] = merged_with[polyline_idx1];
        }
    }
}

// Fill MultiLine
// FillBase.cpp:2615-2675
pub fn multiline_fill(polylines: &mut Vec<Polyline>, params: &FillParams, spacing: f32) {
    if params.multiline > 1 {
        let n_lines = params.multiline;
        let n_polylines = polylines.len() as i32;
        let mut all_polylines: Vec<Polyline> =
            Vec::with_capacity((n_lines * n_polylines) as usize);

        let center = (n_lines - 1) as f32 / 2.0f32;

        // current polyline as the center line, offset to both sides
        for line in 0..n_lines {
            let offset = (line as f32 - center) * spacing;

            for pl in polylines.iter() {
                let n = pl.points.len();
                if n < 2 {
                    all_polylines.push(pl.clone());
                    continue;
                }

                let mut new_points: Vec<Point> = Vec::with_capacity(n);
                // Offset each point along the normal direction
                for i in 0..n {
                    let mut tangent: (f32, f32);
                    if i == 0 {
                        tangent = (
                            (pl.points[1].x() - pl.points[0].x()) as f32,
                            (pl.points[1].y() - pl.points[0].y()) as f32,
                        );
                    } else if i == n - 1 {
                        tangent = (
                            (pl.points[n - 1].x() - pl.points[n - 2].x()) as f32,
                            (pl.points[n - 1].y() - pl.points[n - 2].y()) as f32,
                        );
                    } else {
                        tangent = (
                            (pl.points[i + 1].x() - pl.points[i - 1].x()) as f32,
                            (pl.points[i + 1].y() - pl.points[i - 1].y()) as f32,
                        );
                    }
                    let mut len = tangent.0.hypot(tangent.1);
                    if len == 0.0 {
                        len = 1.0f32;
                    }
                    tangent = (tangent.0 / len, tangent.1 / len);
                    let normal = (-tangent.1, tangent.0);

                    let mut p = pl.points[i];
                    p.x += crate::scale((normal.0 * offset) as f64);
                    p.y += crate::scale((normal.1 * offset) as f64);
                    new_points.push(p);
                }

                all_polylines.push(Polyline::from_points(new_points));
            }
        }
        *polylines = all_polylines;
    }
}
