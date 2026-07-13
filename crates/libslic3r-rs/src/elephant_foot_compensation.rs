//! Elephant foot compensation for the first layer.
//!
//! 1:1 line-by-line port of `ElephantFootCompensation.cpp` / `.hpp` from
//! BambuStudio/libslic3r. `coord_t` -> `i64`, `coordf_t` -> `f64`. The Eigen
//! `Vec2d`/`Vec2f` vectors are mapped onto raw `f64`/`f32` component arithmetic so
//! the integer-vs-float and truncation behaviour matches the C++ exactly.
//!
//! The body of `elephant_foot_compensation(ExPolygon, min_contour_width,
//! compensation)` calls `variable_offset_inner_ex` (ClipperUtils.cpp:1390), which
//! depends on the mittered-offset pipeline (`mittered_offset_path_scaled`,
//! `fix_after_inner_offset`, `fix_after_outer_offset`). That pipeline is now
//! ported in `clipper2_utils.rs` (backed by the Clipper2 FFI, matching the
//! crate-wide convention), so the full compensation path is wired up.

use crate::clipper2_utils::variable_offset_inner_ex;
use crate::edge_grid::EdgeGrid;
use crate::flow::Flow;
use crate::geometry::{
    cross2f, expolygons_simplify, get_extents_expoly, segments_intersect, BoundingBox, ExPolygon,
    ExPolygons, Point, PointF, Points, Polygon,
};
use crate::libslic3r::{EPSILON, SCALED_EPSILON};
use crate::scale;
use crate::utils::{next_idx_modulo, prev_idx_modulo};

// ElephantFootCompensation.cpp:13-14 — <cmath>, <cassert>
const M_PI: f64 = std::f64::consts::PI;

// ElephantFootCompensation.cpp:16 — // #define CONTOUR_DISTANCE_DEBUG_SVG

// ElephantFootCompensation.cpp:20-28
struct ResampledPoint {
    idx_src: usize,
    // Is this point interpolated or initial?
    interpolated: bool,
    // Euclidean distance along the curve from the 0th point.
    curve_parameter: f64,
}

impl ResampledPoint {
    // ElephantFootCompensation.cpp:21
    fn new(idx_src: usize, interpolated: bool, curve_parameter: f64) -> Self {
        Self {
            idx_src,
            interpolated,
            curve_parameter,
        }
    }
}

// Cast a scaled integer Point to a double vector (Eigen `.cast<double>()`).
#[inline]
fn pt_to_d(p: Point) -> PointF {
    PointF::new(p.x as f64, p.y as f64)
}

// Cast a double vector to a scaled integer Point (Eigen `.cast<coord_t>()`),
// truncating toward zero just like C++ `static_cast<coord_t>`.
#[inline]
fn d_to_pt(v: PointF) -> Point {
    Point::new(v.x as i64, v.y as i64)
}

#[inline]
fn norm(v: PointF) -> f64 {
    (v.x * v.x + v.y * v.y).sqrt()
}

#[inline]
fn normalized(v: PointF) -> PointF {
    // Eigen `.normalized()`: divide by Euclidean norm (NaN for the zero vector,
    // matching C++; the call sites only normalize differences of distinct points).
    let n = norm(v);
    PointF::new(v.x / n, v.y / n)
}

// Distance calculated using SDF (Shape Diameter Function).
// The distance is calculated by casting a fan of rays and measuring the intersection distance.
// Thus the calculation is relatively slow. For the Elephant foot compensation purpose, this distance metric does not avoid
// pinching off small pieces of a contour, thus this function has been superseded by contour_distance2().
// ElephantFootCompensation.cpp:30-229
#[allow(clippy::needless_range_loop, dead_code)]
fn contour_distance(
    grid: &EdgeGrid,
    idx_contour: usize,
    contour: &Points,
    resampled_point_parameters: &[ResampledPoint],
    search_radius: f64,
) -> Vec<f32> {
    // ElephantFootCompensation.cpp:36-37
    debug_assert!(!contour.is_empty());
    debug_assert!(contour.len() >= 2);

    // ElephantFootCompensation.cpp:39
    let mut out: Vec<f32> = Vec::new();

    // ElephantFootCompensation.cpp:41
    if contour.len() > 2 {
        // ElephantFootCompensation.cpp:53-151 — struct Visitor + instantiation
        struct Visitor<'a> {
            grid: &'a EdgeGrid,
            idx_contour: usize,
            resampled_point_parameters: &'a [ResampledPoint],
            dist_same_contour_reject: f64,

            idx_point_start: usize,
            pt_start: Point,
            pt_end: Point,
            pt: PointF,
            dir: PointF,
            // Minium parameter along the vector (pt_end - pt_start).
            t_min: f64,
        }

        impl<'a> Visitor<'a> {
            // ElephantFootCompensation.cpp:57-79
            fn init(&mut self, aidx_point_start: usize, apt_start: Point, mut dir: PointF, radius: f64) {
                self.idx_point_start = aidx_point_start;
                self.pt = pt_to_d(apt_start) + dir * SCALED_EPSILON;
                dir = dir * radius;
                self.pt_start = d_to_pt(self.pt);
                // Trim the vector by the grid's bounding box.
                let bbox = self.grid.bbox();
                let mut t = 1.0_f64;
                for axis in 0..2usize {
                    let dir_axis = if axis == 0 { dir.x } else { dir.y };
                    let pt_axis = if axis == 0 { self.pt.x } else { self.pt.y };
                    let bbox_max_axis = if axis == 0 { bbox.max.x } else { bbox.max.y } as f64;
                    let bbox_min_axis = if axis == 0 { bbox.min.x } else { bbox.min.y } as f64;
                    let dx = dir_axis.abs();
                    if dx >= EPSILON {
                        let tedge = if dir_axis > 0.0 {
                            bbox_max_axis - SCALED_EPSILON - pt_axis
                        } else {
                            pt_axis - bbox_min_axis - SCALED_EPSILON
                        };
                        if tedge < dx {
                            t = t.min(tedge / dx);
                        }
                    }
                }
                self.dir = dir;
                if t < 1.0 {
                    dir = dir * t;
                }
                self.pt_end = d_to_pt(self.pt + dir);
                self.t_min = 1.0;
                debug_assert!(
                    self.grid.bbox().contains_point(&self.pt_start)
                        && self.grid.bbox().contains_point(&self.pt_end)
                );
            }

            // ElephantFootCompensation.cpp:81-137
            fn visit(&mut self, iy: usize, ix: usize) -> bool {
                // Called with a row and colum of the grid cell, which is intersected by a line.
                let cell_data_range = self.grid.cell_data_range_at(iy, ix);
                let mut valid = true;
                for it_contour_and_segment in cell_data_range {
                    // End points of the line segment and their vector.
                    let segment = self.grid.segment(*it_contour_and_segment);
                    if segments_intersect(segment.a, segment.b, self.pt_start, self.pt_end) {
                        // The two segments intersect. Calculate the intersection.
                        let pt2 = pt_to_d(segment.a);
                        let dir2 = pt_to_d(segment.b) - pt2;
                        let vptpt2 = self.pt - pt2;
                        let denom = self.dir.x * dir2.y - dir2.x * self.dir.y;

                        if denom.abs() >= EPSILON {
                            let t = cross2f(dir2, vptpt2) / denom;
                            debug_assert!(t > -EPSILON && t < 1.0 + EPSILON);
                            let mut this_valid = true;
                            if it_contour_and_segment.0 == self.idx_contour {
                                // The intersected segment originates from the same contour as the starting point.
                                // Reject the intersection if it is close to the starting point.
                                // Find the start and end points of this segment
                                let mut param_lo =
                                    self.resampled_point_parameters[self.idx_point_start].curve_parameter;
                                let param_hi;
                                let param_end =
                                    self.resampled_point_parameters.last().unwrap().curve_parameter;
                                {
                                    let contour = &self.grid.contours()[it_contour_and_segment.0];
                                    let mut ipt = it_contour_and_segment.1;
                                    let it = lower_bound_resampled(self.resampled_point_parameters, ipt);
                                    debug_assert!(
                                        it < self.resampled_point_parameters.len()
                                            && self.resampled_point_parameters[it].idx_src == ipt
                                            && !self.resampled_point_parameters[it].interpolated
                                    );
                                    let t2 = cross2f(self.dir, vptpt2) / denom;
                                    debug_assert!(t2 > -EPSILON && t2 < 1.0 + EPSILON);
                                    // contour.begin() + (++ipt) == contour.end()
                                    ipt += 1;
                                    if ipt == contour.points().len() {
                                        param_hi = t2 * norm(dir2);
                                    } else {
                                        param_hi = self.resampled_point_parameters[it].curve_parameter
                                            + t2 * norm(dir2);
                                    }
                                }
                                let mut param_hi = param_hi;
                                if param_lo > param_hi {
                                    std::mem::swap(&mut param_lo, &mut param_hi);
                                }
                                debug_assert!(param_lo >= 0.0 && param_lo <= param_end);
                                debug_assert!(param_hi >= 0.0 && param_hi <= param_end);
                                this_valid = param_hi > param_lo + self.dist_same_contour_reject
                                    && param_hi - param_end < param_lo - self.dist_same_contour_reject;
                            }
                            if t < self.t_min {
                                self.t_min = t;
                                valid = this_valid;
                            }
                        }
                    }
                    if !valid {
                        self.t_min = 1.0;
                    }
                }
                // Continue traversing the grid along the edge.
                true
            }
        }

        // ElephantFootCompensation.cpp:54-55, 151
        let mut visitor = Visitor {
            grid,
            idx_contour,
            resampled_point_parameters,
            dist_same_contour_reject: search_radius,
            idx_point_start: 0,
            pt_start: Point::new(0, 0),
            pt_end: Point::new(0, 0),
            pt: PointF::new(0.0, 0.0),
            dir: PointF::new(0.0, 0.0),
            t_min: 1.0,
        };

        // ElephantFootCompensation.cpp:153-158
        let mut pt_this = contour[contour.len() - 1];
        let mut idx_pt_this = contour.len() - 1;
        let pt_prev = contour[contour.len() - 2];
        // perpenduclar vector
        let perp = |v: PointF| -> PointF { PointF::new(v.y, -v.x) };
        let mut vprev = normalized(pt_to_d(pt_this) - pt_to_d(pt_prev));
        // ElephantFootCompensation.cpp:159
        out.reserve(contour.len() + 1);
        // ElephantFootCompensation.cpp:160-222
        for idx_pt_next in 0..contour.len() {
            let pt_next = contour[idx_pt_next];
            let vnext = normalized(pt_to_d(pt_next) - pt_to_d(pt_this));
            let dir = -normalized(perp(vprev) + perp(vnext));
            let dir_perp = perp(dir);
            let cross = cross2f(vprev, vnext);
            let dot = vprev.x * vnext.x + vprev.y * vnext.y;
            let a = if cross < 0.0 || dot > 0.5 {
                M_PI / 3.0
            } else {
                0.48 * (1.0_f64.min(-dot)).acos()
            };
            // Throw rays, collect distances.
            let mut distances: Vec<f64> = Vec::new();
            let num_rays: i32 = 15;

            // ElephantFootCompensation.cpp:178-193
            let mut i = -num_rays + 1;
            while i < num_rays {
                let angle = a * (i as f64) / (num_rays as f64);
                let c = angle.cos();
                let s = angle.sin();
                let v = dir * c + dir_perp * s;
                visitor.init(idx_pt_this, pt_this, v, search_radius);
                grid.visit_cells_intersecting_line(visitor.pt_start, visitor.pt_end, |row, col| {
                    visitor.visit(row, col)
                });
                distances.push(visitor.t_min);
                i += 1;
            }
            // ElephantFootCompensation.cpp:197
            distances.sort_by(|x, y| x.partial_cmp(y).unwrap());
            // ElephantFootCompensation.cpp:213-215 — #else branch (#if 0 averaging disabled)
            out.push((distances[0] * search_radius) as f32);
            // ElephantFootCompensation.cpp:219-221
            pt_this = pt_next;
            idx_pt_this = idx_pt_next;
            vprev = vnext;
        }
        let _ = (idx_pt_this, pt_prev);
        // Rotate the vector by one item.
        // ElephantFootCompensation.cpp:224-225
        let front = out[0];
        out.push(front);
        out.remove(0);
    }

    // ElephantFootCompensation.cpp:228
    out
}

// std::lower_bound over resampled_point_parameters with comparator
//   l.idx_src < r.idx_src || (l.idx_src == r.idx_src && int(l.interpolated) > int(r.interpolated))
// searching for key (ipt, interpolated=false). Returns the first index whose element is
// not "less than" the key. ElephantFootCompensation.cpp:109-112 / 289-292
fn lower_bound_resampled(params: &[ResampledPoint], ipt: usize) -> usize {
    // key: idx_src = ipt, interpolated = false (int = 0)
    // less(l, key) is true while we keep advancing.
    let less = |l: &ResampledPoint| -> bool {
        l.idx_src < ipt || (l.idx_src == ipt && (l.interpolated as i32) > 0)
    };
    let mut lo = 0usize;
    let mut len = params.len();
    while len > 0 {
        let half = len / 2;
        let mid = lo + half;
        if less(&params[mid]) {
            lo = mid + 1;
            len -= half + 1;
        } else {
            len = half;
        }
    }
    lo
}

// Contour distance by measuring the closest point of an ExPolygon stored inside the EdgeGrid, while filtering out points of the same contour
// at concave regions, or convex regions with low curvature (curvature is estimated as a ratio between contour length and chordal distance crossing the contour ends).
// ElephantFootCompensation.cpp:231-416
fn contour_distance2(
    grid: &EdgeGrid,
    idx_contour: usize,
    contour: &Points,
    resampled_point_parameters: &[ResampledPoint],
    compensation: f64,
    search_radius: f64,
) -> Vec<f32> {
    // ElephantFootCompensation.cpp:235-236
    debug_assert!(!contour.is_empty());
    debug_assert!(contour.len() >= 2);

    // ElephantFootCompensation.cpp:238
    let mut out: Vec<f32> = Vec::new();

    // ElephantFootCompensation.cpp:240
    if contour.len() > 2 {
        // ElephantFootCompensation.cpp:252-383 — struct Visitor + instantiation
        struct Visitor<'a> {
            grid: &'a EdgeGrid,
            idx_contour: usize,
            resampled_point_parameters: &'a [ResampledPoint],
            dist_same_contour_accept: f64,
            dist_same_contour_reject: f64,

            idx_point: usize,
            point: Point,
            // Direction inside the contour from idx_point, not normalized.
            dir_inside: PointF,
            found: bool,
            distance: f64,
        }

        impl<'a> Visitor<'a> {
            // ElephantFootCompensation.cpp:256-262
            fn init(&mut self, contour: &Points, idx_point: usize) {
                self.idx_point = idx_point;
                self.point = contour[idx_point];
                self.found = false;
                self.dir_inside = Self::dir_inside_at_point(contour, self.idx_point);
                self.distance = f64::MAX;
            }

            // ElephantFootCompensation.cpp:264-328
            fn visit(&mut self, iy: usize, ix: usize) -> bool {
                // Called with a row and colum of the grid cell, which is intersected by a line.
                let cell_data_range = self.grid.cell_data_range_at(iy, ix);
                for it_contour_and_segment in cell_data_range {
                    // End points of the line segment and their vector.
                    let segment = self.grid.segment(*it_contour_and_segment);
                    let v = pt_to_d(segment.b) - pt_to_d(segment.a);
                    let va = pt_to_d(self.point) - pt_to_d(segment.a);
                    let l2 = v.x * v.x + v.y * v.y; // avoid a sqrt
                    let t = if l2 == 0.0 {
                        0.0
                    } else {
                        ((va.x * v.x + va.y * v.y) / l2).clamp(0.0, 1.0)
                    };
                    // Closest point from this->point to the segment.
                    let foot = pt_to_d(segment.a) + v * t;
                    let bisector = foot - pt_to_d(self.point);
                    let dist = norm(bisector);
                    if (!self.found || dist < self.distance)
                        && (self.dir_inside.x * bisector.x + self.dir_inside.y * bisector.y) > 0.0
                    {
                        let mut accept = true;
                        if it_contour_and_segment.0 == self.idx_contour {
                            // Complex case: The closest segment originates from the same contour as the starting point.
                            // Reject the closest point if its distance along the contour is reasonable compared to the current contour bisector (this->pt, foot).
                            let mut param_lo =
                                self.resampled_point_parameters[self.idx_point].curve_parameter;
                            let param_hi;
                            let param_end =
                                self.resampled_point_parameters.last().unwrap().curve_parameter;
                            let contour_eg = &self.grid.contours()[it_contour_and_segment.0];
                            let ipt = it_contour_and_segment.1;
                            {
                                let it =
                                    lower_bound_resampled(self.resampled_point_parameters, ipt);
                                debug_assert!(
                                    it < self.resampled_point_parameters.len()
                                        && self.resampled_point_parameters[it].idx_src == ipt
                                        && !self.resampled_point_parameters[it].interpolated
                                );
                                let mut ph = t * l2.sqrt();
                                // contour.begin() + ipt + 1 < contour.end()
                                if ipt + 1 < contour_eg.points().len() {
                                    ph += self.resampled_point_parameters[it].curve_parameter;
                                }
                                param_hi = ph;
                            }
                            let mut param_hi = param_hi;
                            if param_lo > param_hi {
                                std::mem::swap(&mut param_lo, &mut param_hi);
                            }
                            debug_assert!(
                                param_lo > -SCALED_EPSILON && param_lo <= param_end + SCALED_EPSILON
                            );
                            debug_assert!(
                                param_hi > -SCALED_EPSILON && param_hi <= param_end + SCALED_EPSILON
                            );
                            let dist_along_contour =
                                (param_hi - param_lo).min(param_lo + param_end - param_hi);
                            if dist_along_contour < self.dist_same_contour_accept {
                                accept = false;
                            } else if dist < self.dist_same_contour_reject + SCALED_EPSILON {
                                // this->point is close to foot. This point will only be accepted if the path along the contour is significantly
                                // longer than the bisector. That is, the path shall not bulge away from the bisector too much.
                                // Bulge is estimated by 0.6 of the circle circumference drawn around the bisector.
                                // Test whether the contour is convex or concave.
                                let inside = if t == 0.0 {
                                    Self::inside_corner(contour_eg, ipt, self.point)
                                } else if t == 1.0 {
                                    Self::inside_corner(
                                        contour_eg,
                                        contour_eg.segment_idx_next(ipt),
                                        self.point,
                                    )
                                } else {
                                    Self::left_of_segment(contour_eg, ipt, self.point)
                                };
                                accept = inside && dist_along_contour > 0.6 * M_PI * dist;
                            }
                        }
                        if accept && (!self.found || dist < self.distance) {
                            // Simple case: Just measure the shortest distance.
                            self.distance = dist;
                            self.found = true;
                        }
                    }
                }
                // Continue traversing the grid.
                true
            }

            // ElephantFootCompensation.cpp:348-354
            fn dir_inside_at_point(contour: &Points, i: usize) -> PointF {
                let iprev = prev_idx_modulo(i, contour.len());
                let inext = next_idx_modulo(i, contour.len());
                let v1 = pt_to_d(contour[i]) - pt_to_d(contour[iprev]);
                let v2 = pt_to_d(contour[inext]) - pt_to_d(contour[i]);
                PointF::new(-v1.y - v2.y, v1.x + v2.x)
            }

            // ElephantFootCompensation.cpp:361-373
            fn inside_corner(
                contour: &crate::edge_grid::Contour,
                i: usize,
                pt_oposite: Point,
            ) -> bool {
                let pt = pt_to_d(pt_oposite);
                let pt_prev = *contour.segment_prev(i);
                let pt_this = *contour.segment_start(i);
                let pt_next = *contour.segment_end(i);
                let v1 = pt_to_d(pt_this) - pt_to_d(pt_prev);
                let v2 = pt_to_d(pt_next) - pt_to_d(pt_this);
                let left_of_v1 = cross2f(v1, pt - pt_to_d(pt_prev)) > 0.0;
                let left_of_v2 = cross2f(v2, pt - pt_to_d(pt_this)) > 0.0;
                if cross2f(v1, v2) > 0.0 {
                    left_of_v1 && left_of_v2 // convex corner
                } else {
                    left_of_v1 || left_of_v2 // concave corner
                }
            }

            // ElephantFootCompensation.cpp:375-382
            fn left_of_segment(
                contour: &crate::edge_grid::Contour,
                i: usize,
                pt_oposite: Point,
            ) -> bool {
                let pt = pt_to_d(pt_oposite);
                let pt_this = *contour.segment_start(i);
                let pt_next = *contour.segment_end(i);
                let v = pt_to_d(pt_next) - pt_to_d(pt_this);
                cross2f(v, pt - pt_to_d(pt_this)) > 0.0
            }
        }

        // ElephantFootCompensation.cpp:253-254, 383
        let mut visitor = Visitor {
            grid,
            idx_contour,
            resampled_point_parameters,
            dist_same_contour_accept: 0.5 * compensation * M_PI,
            dist_same_contour_reject: search_radius,
            idx_point: 0,
            point: Point::new(0, 0),
            dir_inside: PointF::new(0.0, 0.0),
            found: false,
            distance: f64::MAX,
        };

        // ElephantFootCompensation.cpp:385-403
        out.reserve(contour.len());
        let radius_vector = Point::new(search_radius as i64, search_radius as i64);
        for idx_pt in 0..contour.len() {
            let pt = contour[idx_pt];
            visitor.init(contour, idx_pt);
            grid.visit_cells_intersecting_box(
                BoundingBox::from_points_minmax(
                    Point::new(pt.x - radius_vector.x, pt.y - radius_vector.y),
                    Point::new(pt.x + radius_vector.x, pt.y + radius_vector.y),
                ),
                |row, col| visitor.visit(row, col),
            );
            out.push(if visitor.found {
                visitor.distance.min(search_radius) as f32
            } else {
                search_radius as f32
            });
        }
    }

    // ElephantFootCompensation.cpp:415
    out
}

// ElephantFootCompensation.cpp:418-446
fn resample_polygon(
    contour: &Points,
    dist: f64,
    resampled_point_parameters: &mut Vec<ResampledPoint>,
) -> Points {
    let mut out: Points = Vec::new();
    out.reserve(contour.len());
    resampled_point_parameters.reserve(contour.len());
    // ElephantFootCompensation.cpp:423
    if contour.len() > 2 {
        let mut pt_prev = pt_to_d(contour[contour.len() - 1]);
        for idx_this in 0..contour.len() {
            let pt = contour[idx_this];
            let pt_this = pt_to_d(pt);
            let v = pt_this - pt_prev;
            let l = norm(v);
            let n = (l / dist).ceil() as usize;
            let l_step = l / (n as f64);
            // ElephantFootCompensation.cpp:432-437
            for i in 1..n {
                let interpolation_parameter = (i as f64) / (n as f64);
                let new_pt = pt_prev + v * interpolation_parameter;
                out.push(d_to_pt(new_pt));
                resampled_point_parameters.push(ResampledPoint::new(idx_this, true, l_step));
            }
            // ElephantFootCompensation.cpp:438-439
            out.push(pt);
            resampled_point_parameters.push(ResampledPoint::new(idx_this, false, l_step));
            pt_prev = pt_this;
        }
        // ElephantFootCompensation.cpp:442-443
        for i in 1..resampled_point_parameters.len() {
            resampled_point_parameters[i].curve_parameter +=
                resampled_point_parameters[i - 1].curve_parameter;
        }
    }
    out
}

// ElephantFootCompensation.cpp:448-463 — #if 0 smooth_compensation (disabled). Omitted.

// Scalar linear interpolation matching the C++ `lerp(float, float, float)` template
// used in smooth_compensation_banded (Slic3r::lerp). ElephantFootCompensation.cpp:490-491,515-516
#[inline]
fn lerp_f32(a: f32, b: f32, t: f32) -> f32 {
    (1.0 - t) * a + t * b
}

// ElephantFootCompensation.cpp:465-532
fn smooth_compensation_banded(
    contour: &Points,
    band: f32,
    compensation: &mut Vec<f32>,
    strength: f32,
    num_iterations: usize,
) {
    debug_assert_eq!(contour.len(), compensation.len());
    debug_assert!(contour.len() > 2);
    let mut out: Vec<f32> = compensation.clone();
    let dist_min2 = band * band;
    const USE_MIN: bool = false;
    // ElephantFootCompensation.cpp:472-531
    for _iter in 0..num_iterations {
        for i in 0..compensation.len() as i32 {
            let i = i as usize;
            // const Vec2f pthis = contour[i].cast<float>();
            let pthis_x = contour[i].x as f32;
            let pthis_y = contour[i].y as f32;

            // --- previous direction ---
            let mut j = prev_idx_modulo(i, contour.len());
            let mut pprev_x = contour[j].x as f32;
            let mut pprev_y = contour[j].y as f32;
            let mut prev = compensation[j];
            let mut l2 = {
                let dx = pthis_x - pprev_x;
                let dy = pthis_y - pprev_y;
                dx * dx + dy * dy
            };
            if l2 < dist_min2 {
                let mut l = l2.sqrt();
                let mut jprev = j;
                j = prev_idx_modulo(j, contour.len());
                while j != i {
                    let pp_x = contour[j].x as f32;
                    let pp_y = contour[j].y as f32;
                    let lthis = {
                        let dx = pp_x - pprev_x;
                        let dy = pp_y - pprev_y;
                        (dx * dx + dy * dy).sqrt()
                    };
                    let lnext = l + lthis;
                    if lnext > band {
                        // Interpolate the compensation value.
                        let interp = lerp_f32(compensation[jprev], compensation[j], (band - l) / lthis);
                        prev = if USE_MIN { prev.min(interp) } else { interp };
                        break;
                    }
                    prev = if USE_MIN {
                        prev.min(compensation[j])
                    } else {
                        compensation[j]
                    };
                    pprev_x = pp_x;
                    pprev_y = pp_y;
                    l = lnext;
                    jprev = j;
                    j = prev_idx_modulo(j, contour.len());
                }
            }

            // --- next direction ---
            j = next_idx_modulo(i, contour.len());
            pprev_x = contour[j].x as f32;
            pprev_y = contour[j].y as f32;
            let mut next = compensation[j];
            l2 = {
                let dx = pprev_x - pthis_x;
                let dy = pprev_y - pthis_y;
                dx * dx + dy * dy
            };
            if l2 < dist_min2 {
                let mut l = l2.sqrt();
                let mut jprev = j;
                j = next_idx_modulo(j, contour.len());
                while j != i {
                    let pp_x = contour[j].x as f32;
                    let pp_y = contour[j].y as f32;
                    let lthis = {
                        let dx = pp_x - pprev_x;
                        let dy = pp_y - pprev_y;
                        (dx * dx + dy * dy).sqrt()
                    };
                    let lnext = l + lthis;
                    if lnext > band {
                        // Interpolate the compensation value.
                        let interp = lerp_f32(compensation[jprev], compensation[j], (band - l) / lthis);
                        next = if USE_MIN { next.min(interp) } else { interp };
                        break;
                    }
                    next = if USE_MIN {
                        next.min(compensation[j])
                    } else {
                        compensation[j]
                    };
                    pprev_x = pp_x;
                    pprev_y = pp_y;
                    l = lnext;
                    jprev = j;
                    j = next_idx_modulo(j, contour.len());
                }
            }

            // ElephantFootCompensation.cpp:526-528
            // Native (clang, -ffp-contract=fast on ARM) fuses the outer add into
            // an FMA: laplacian = fma(comp[i], 1-s, (0.5*s)*(prev+next)). Rust does
            // not auto-contract, so force it with mul_add to match bit-for-bit.
            let laplacian =
                compensation[i].mul_add(1.0 - strength, (0.5 * strength) * (prev + next));
            // Compensations are negative. Only apply the laplacian if it leads to lower compensation.
            out[i] = laplacian.max(compensation[i]);
        }
        std::mem::swap(&mut out, compensation);
    }
}

// ElephantFootCompensation.cpp:534-542 — #ifndef NDEBUG
#[cfg(debug_assertions)]
fn validate_expoly_orientation(expoly: &ExPolygon) -> bool {
    let mut valid = expoly.contour.is_counter_clockwise();
    for h in &expoly.holes {
        valid &= h.is_clockwise();
    }
    valid
}

// ElephantFootCompensation.cpp:544-618
// hpp:12 — ExPolygon elephant_foot_compensation(const ExPolygon &input, double min_contour_width, const double compensation)
pub fn elephant_foot_compensation_with_width(
    input_expoly: &ExPolygon,
    min_contour_width: f64,
    compensation: f64,
) -> ExPolygon {
    #[cfg(debug_assertions)]
    debug_assert!(validate_expoly_orientation(input_expoly));

    // ElephantFootCompensation.cpp:548-552
    // Native scale_() DIVIDES by SCALING_FACTOR=1e-5 with NO rounding — the
    // scaled values stay fractional doubles (0.15 → 15000.000000000002); the
    // crate scale() rounds to integer, shifting every threshold by ulps.
    let scaled_compensation = compensation / 0.00001_f64;
    let min_contour_width = min_contour_width / 0.00001_f64;
    let min_contour_width_compensated = min_contour_width + 2.0 * scaled_compensation;
    // Make the search radius a bit larger for the averaging in contour_distance over a fan of rays to work.
    let search_radius = min_contour_width_compensated + min_contour_width * 0.5;

    // ElephantFootCompensation.cpp:554-555
    let bbox = get_extents_expoly_contour(&input_expoly.contour);
    let bbox_size = bbox.size();
    let out: ExPolygon;
    // ElephantFootCompensation.cpp:557-563
    if (bbox_size.x as f64) < min_contour_width_compensated + SCALED_EPSILON
        || (bbox_size.y as f64) < min_contour_width_compensated + SCALED_EPSILON
        || input_expoly.area() < min_contour_width_compensated * min_contour_width_compensated * 5.0
    {
        // The contour is tiny. Don't correct it.
        out = input_expoly.clone();
    } else {
        // ElephantFootCompensation.cpp:566-571
        let mut grid = EdgeGrid::new();
        #[cfg(debug_assertions)]
        debug_assert!(validate_expoly_orientation(input_expoly));
        let mut bbox = get_extents_expoly_contour(&input_expoly.contour);
        // C++ bbox.offset(SCALED_EPSILON): PointClass(delta, delta) truncates the
        // f64 delta toward zero for the integer Point, so expand by 10.
        bbox.expand(SCALED_EPSILON as i64);
        grid.set_bbox(bbox);
        grid.create_from_expolygon(input_expoly, (0.7 * search_radius) as i64);
        // ElephantFootCompensation.cpp:572-573
        let mut deltas: Vec<Vec<f32>> = Vec::new();
        deltas.reserve(input_expoly.holes.len() + 1);
        // ElephantFootCompensation.cpp:574-575
        let mut resampled: ExPolygon = input_expoly.clone();
        let resample_interval = 0.5_f64 / 0.00001_f64;
        // ElephantFootCompensation.cpp:576-596
        for idx_contour in 0..=input_expoly.holes.len() {
            let poly: &mut Polygon = if idx_contour == 0 {
                &mut resampled.contour
            } else {
                &mut resampled.holes[idx_contour - 1]
            };
            let mut resampled_point_parameters: Vec<ResampledPoint> = Vec::new();
            poly.points = resample_polygon(
                &poly.points,
                resample_interval,
                &mut resampled_point_parameters,
            );
            debug_assert!(poly.is_counter_clockwise() == (idx_contour == 0));
            let mut dists = contour_distance2(
                &grid,
                idx_contour,
                &poly.points,
                &resampled_point_parameters,
                scaled_compensation,
                search_radius,
            );
            // ElephantFootCompensation.cpp:582-592
            for d in dists.iter_mut() {
                // Convert contour width to available compensation distance.
                // Native compares float d against the DOUBLE thresholds
                // (d promotes to double); only the else-branch arithmetic is f32
                // (d - float(min_contour_width)) / 2.f.
                if (*d as f64) < min_contour_width {
                    *d = 0.0;
                } else if (*d as f64) > min_contour_width_compensated {
                    *d = -(scaled_compensation as f32);
                } else {
                    *d = -(*d - min_contour_width as f32) / 2.0;
                }
                debug_assert!(*d >= -(scaled_compensation as f32) && *d <= 0.0);
            }
            // ElephantFootCompensation.cpp:594
            smooth_compensation_banded(
                &poly.points,
                (0.8 * resample_interval) as f32,
                &mut dists,
                0.3,
                3,
            );
            deltas.push(dists);
        }

        // ElephantFootCompensation.cpp:598
        // R320: native runs this through ClipperLib(1) — the Clipper2 route
        // produced different intersection vertices (1512-vs-1528, R318).
        let mut out_vec =
            crate::clipper_utils::variable_offset_inner_ex_clib(&resampled, &deltas, 2.0);
        // ElephantFootCompensation.cpp:599-613
        if out_vec.len() == 1 {
            out = std::mem::take(&mut out_vec[0]);
        } else {
            // Something went wrong, don't compensate.
            out = input_expoly.clone();
            // ElephantFootCompensation.cpp:612 — assert(out_vec.size() == 1);
            debug_assert_eq!(out_vec.len(), 1);
        }
    }

    // ElephantFootCompensation.cpp:616-617
    #[cfg(debug_assertions)]
    debug_assert!(validate_expoly_orientation(&out));
    out
}

// ElephantFootCompensation.cpp:620-625
// hpp:14 — ExPolygon elephant_foot_compensation(const ExPolygon &input, const Flow &external_perimeter_flow, const double compensation)
pub fn elephant_foot_compensation_with_flow(
    input: &ExPolygon,
    external_perimeter_flow: &Flow,
    compensation: f64,
) -> ExPolygon {
    // The contour shall be wide enough to apply the external perimeter plus compensation on both sides.
    let min_contour_width = external_perimeter_flow.width() + external_perimeter_flow.spacing();
    elephant_foot_compensation_with_width(input, min_contour_width, compensation)
}

// ElephantFootCompensation.cpp:627-635
// hpp:15 — ExPolygons elephant_foot_compensation(const ExPolygons &input, const Flow &external_perimeter_flow, const double compensation)
pub fn elephant_foot_compensation_expolygons_with_flow(
    input: &ExPolygons,
    external_perimeter_flow: &Flow,
    compensation: f64,
) -> ExPolygons {
    let mut out: ExPolygons = Vec::new();
    // C++: expolygons_simplify(input, SCALED_EPSILON). C++ `simplify` feeds the
    // tolerance straight into Douglas-Peucker in scaled integer units (no rescale),
    // so the effective tolerance is SCALED_EPSILON (= 10 scaled units). The Rust
    // `expolygons_simplify` instead rescales its argument via `scale()` (mm in), so
    // pass the unscaled-mm equivalent EPSILON (scale(EPSILON) == SCALED_EPSILON == 10)
    // to reproduce the identical Douglas-Peucker tolerance.
    let simplified_exps = expolygons_simplify(input, EPSILON);
    out.reserve(simplified_exps.len());
    for expoly in &simplified_exps {
        out.push(elephant_foot_compensation_with_flow(
            expoly,
            external_perimeter_flow,
            compensation,
        ));
    }
    out
}

// ElephantFootCompensation.cpp:637-645
// hpp:13 — ExPolygons elephant_foot_compensation(const ExPolygons &input, double min_contour_width, const double compensation)
pub fn elephant_foot_compensation_expolygons_with_width(
    input: &ExPolygons,
    min_contour_width: f64,
    compensation: f64,
) -> ExPolygons {
    let mut out: ExPolygons = Vec::new();
    // C++: expolygons_simplify(input, SCALED_EPSILON). See note above: Rust's
    // `expolygons_simplify` rescales its tolerance (mm in), so pass EPSILON so that
    // scale(EPSILON) == SCALED_EPSILON == 10, matching the C++ Douglas-Peucker tolerance.
    let simplified_exps = expolygons_simplify(input, EPSILON);
    out.reserve(simplified_exps.len());
    for expoly in &simplified_exps {
        out.push(elephant_foot_compensation_with_width(
            expoly,
            min_contour_width,
            compensation,
        ));
    }
    out
}

// get_extents(Polygon) — the BoundingBox of a single contour's points.
// BambuStudio computes this via get_extents(const Polygon&) which is just the
// bounding box of the polygon points (same as get_extents over a 1-element ExPolygon
// without holes). ElephantFootCompensation.cpp:554, 568
#[inline]
fn get_extents_expoly_contour(contour: &Polygon) -> BoundingBox {
    let ex = ExPolygon::new(contour.clone());
    get_extents_expoly(&ex)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Point;

    fn square(side: i64) -> ExPolygon {
        // CCW square
        ExPolygon::new(Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(side, 0),
            Point::new(side, side),
            Point::new(0, side),
        ]))
    }

    #[test]
    fn test_resample_polygon_passthrough_short() {
        // contour.size() <= 2 returns empty
        let mut params = Vec::new();
        let pts = vec![Point::new(0, 0), Point::new(10, 0)];
        let out = resample_polygon(&pts, scale(0.5) as f64, &mut params);
        assert!(out.is_empty());
        assert!(params.is_empty());
    }

    #[test]
    fn test_tiny_contour_unchanged() {
        // A 0.1mm square is below min_contour_width_compensated; returned unchanged.
        let input = square(scale(0.1));
        let out = elephant_foot_compensation_with_width(&input, 0.4, 0.2);
        assert_eq!(out.contour.points().len(), input.contour.points().len());
    }

    #[test]
    fn test_lower_bound_resampled() {
        let params = vec![
            ResampledPoint::new(0, false, 0.0),
            ResampledPoint::new(1, true, 0.0),
            ResampledPoint::new(1, false, 0.0),
            ResampledPoint::new(2, false, 0.0),
        ];
        // key (1,false): first element not-less-than. Element 1 is (1,true) which is
        // "less" (interpolated>0), element 2 is (1,false) -> found at index 2.
        assert_eq!(lower_bound_resampled(&params, 1), 2);
        assert_eq!(lower_bound_resampled(&params, 0), 0);
        assert_eq!(lower_bound_resampled(&params, 2), 3);
    }
}
