// Polygon offsetting using Voronoi diagram prodiced by boost::polygon.
//
// Faithful 1:1 port of BambuStudio's src/libslic3r/Geometry/VoronoiOffset.cpp
// (+ header VoronoiOffset.hpp).
//
// coord_t -> i64 (Coord), coordf_t -> f64 (CoordF).
//
// The boost::polygon Voronoi diagram is backed by the pure-Rust `boostvoronoi`
// crate (wasm-safe). `VD = Geometry::VoronoiDiagram` in C++ is `bv::Diagram` here.
//
// The header-declared category types (VertexCategory / EdgeCategory / CellCategory)
// and their color accessors, as well as `annotate_inside_outside`, are implemented
// (against the bv::Diagram API) in `voronoi_annotation.rs`; they are re-exported here
// so this module mirrors the public surface of VoronoiOffset.hpp.
// `reset_inside_outside_annotations` (VoronoiOffset.cpp:640-648) is ported directly in
// this file below. The remainder of VoronoiOffset.cpp is ported below.

use boostvoronoi::prelude as bv;

use crate::geometry::{cross2f, lerpf, Line, Point, Polygon, Vec2d};
use crate::libslic3r::{EPSILON, SCALED_EPSILON};
use crate::Coord;

// VoronoiOffset.hpp: using VD = Slic3r::Geometry::VoronoiDiagram;
pub use crate::geometry::voronoi_annotation::{
    annotate_inside_outside, cell_category, edge_category, set_cell_category, set_edge_category,
    set_vertex_category, vertex_category, CellCategory, EdgeCategory, VertexCategory,
};

// ---------------------------------------------------------------------------
// VoronoiOffset.hpp inline helpers (contour_point / vertex_point / edge_offset_*)
// ---------------------------------------------------------------------------

// VoronoiOffset.hpp:16-17
// inline const Point& contour_point(const VD::cell_type &cell, const Line &line)
//     { return ((cell.source_category() == SOURCE_CATEGORY_SEGMENT_START_POINT) ? line.a : line.b); }
#[inline]
fn contour_point_line(cell: &bv::Cell, line: &Line) -> Point {
    if cell.source_category() == bv::SourceCategory::SegmentStart {
        line.a
    } else {
        line.b
    }
}

// VoronoiOffset.hpp:21-22
// inline const Point& contour_point(const VD::cell_type &cell, const Lines &lines)
//     { return contour_point(cell, lines[cell.source_index()]); }
#[inline]
fn contour_point_lines(cell: &bv::Cell, lines: &[Line]) -> Point {
    contour_point_line(cell, &lines[cell.source_index().usize()])
}

// VoronoiOffset.hpp:26-27
// inline Vec2d vertex_point(const VD::vertex_type &v) { return Vec2d(v.x(), v.y()); }
#[inline]
fn vertex_point(v: &bv::Vertex) -> Vec2d {
    Vec2d::new(v.x(), v.y())
}

// VoronoiOffset.hpp:110-111
// static inline bool edge_offset_no_intersection(const Vec2d &intersection_point)
//     { return std::isnan(intersection_point.x()); }
#[inline]
pub fn edge_offset_no_intersection(intersection_point: &Vec2d) -> bool {
    intersection_point.x().is_nan()
}

// VoronoiOffset.hpp:112-113
// static inline bool edge_offset_has_intersection(const Vec2d &intersection_point)
//     { return ! edge_offset_no_intersection(intersection_point); }
#[inline]
pub fn edge_offset_has_intersection(intersection_point: &Vec2d) -> bool {
    !edge_offset_no_intersection(intersection_point)
}

// ---------------------------------------------------------------------------
// Local primitives mirroring the Eigen / libslic3r helpers used by the C++.
// ---------------------------------------------------------------------------

// `pt.cast<double>()` in C++ is a *raw* double cast of the scaled integer coords
// (NOT an unscale). Point::to_f64() unscales, so we must not use it here.
#[inline]
fn cast_double(p: Point) -> Vec2d {
    Vec2d::new(p.x as f64, p.y as f64)
}

// Eigen Vec2d::squaredNorm()
#[inline]
fn squared_norm(v: Vec2d) -> f64 {
    v.x * v.x + v.y * v.y
}

// Eigen Vec2d::norm()
#[inline]
fn norm(v: Vec2d) -> f64 {
    squared_norm(v).sqrt()
}

// Eigen Vec2d::dot()
#[inline]
fn dot(a: Vec2d, b: Vec2d) -> f64 {
    a.x * b.x + a.y * b.y
}

// Eigen Vec2d::normalized()
#[inline]
fn normalized(v: Vec2d) -> Vec2d {
    let n = norm(v);
    Vec2d::new(v.x / n, v.y / n)
}

// Slic3r::sqr (libslic3r.h)
#[inline]
fn sqr(x: f64) -> f64 {
    x * x
}

// Geometry::foot_pt<Vec2d>(line_pt, line_dir, pt)  (Geometry.hpp:169-175)
#[inline]
fn foot_pt_dir(line_pt: Vec2d, line_dir: Vec2d, pt: Vec2d) -> Vec2d {
    let v = pt - line_pt;
    let l2 = squared_norm(line_dir);
    let t = if l2 == 0. { 0. } else { dot(v, line_dir) / l2 };
    line_pt + line_dir * t
}

// Geometry::foot_pt(const Line &iline, const Point &ipt)  (Geometry.hpp:177-180)
#[inline]
fn foot_pt_line(iline: &Line, ipt: Point) -> Vec2d {
    foot_pt_dir(
        cast_double(iline.a),
        cast_double(iline.b - iline.a),
        cast_double(ipt),
    )
}

// Geometry::ray_point_distance<Vec2d>(ray_pt, ray_dir, pt)  (Geometry.hpp:187-190)
#[inline]
fn ray_point_distance(ray_pt: Vec2d, ray_dir: Vec2d, pt: Vec2d) -> f64 {
    norm(foot_pt_dir(ray_pt, ray_dir, pt) - pt)
}

// ---------------------------------------------------------------------------
// namespace detail
// ---------------------------------------------------------------------------
mod detail {
    use super::*;

    // VoronoiOffset.cpp:21-59
    // Intersect a circle with a ray, return the two parameters.
    // Currently used for unbounded Voronoi edges only.
    pub fn first_circle_segment_intersection_parameter(
        center: Vec2d,
        r: f64,
        pt: Vec2d,
        v: Vec2d,
    ) -> f64 {
        // VoronoiOffset.cpp:26
        let d: Vec2d = pt - center;
        // VoronoiOffset.cpp:27-33 (NDEBUG block)
        #[cfg(debug_assertions)]
        {
            // Start point should be inside, end point should be outside the circle.
            let d0 = norm(pt - center);
            let d1 = norm((pt + v) - center);
            debug_assert!(d0 < r + SCALED_EPSILON);
            debug_assert!(d1 > r - SCALED_EPSILON);
        }
        // VoronoiOffset.cpp:34-36
        let a = squared_norm(v);
        let b = 2. * dot(d, v);
        let c = squared_norm(d) - r * r;
        // VoronoiOffset.cpp:37 std::pair<int, std::array<double, 2>> out;
        // VoronoiOffset.cpp:38
        let mut u = b * b - 4. * a * c;
        // VoronoiOffset.cpp:39
        debug_assert!(u > -EPSILON);
        // VoronoiOffset.cpp:40
        let t;
        if u <= 0. {
            // VoronoiOffset.cpp:42-45
            // Degenerate to a single closest point.
            t = -b / (2. * a);
            debug_assert!(t >= -EPSILON && t <= 1. + EPSILON);
            t.clamp(0., 1.)
        } else {
            // VoronoiOffset.cpp:47-58
            u = u.sqrt();
            // out.first = 2;
            let t0 = (-b - u) / (2. * a);
            let t1 = (-b + u) / (2. * a);
            // One of the intersections shall be found inside the segment.
            debug_assert!(
                (t0 >= -EPSILON && t0 <= 1. + EPSILON) || (t1 >= -EPSILON && t1 <= 1. + EPSILON)
            );
            if t1 < 0. {
                return 0.;
            }
            if t0 > 1. {
                return 1.;
            }
            if t0 > 0. {
                t0
            } else {
                t1
            }
        }
    }

    // VoronoiOffset.cpp:61-65
    pub struct Intersections {
        pub count: i32,
        pub pts: [Vec2d; 2],
    }

    // VoronoiOffset.cpp:67-116
    // Return maximum two points, that are at distance "d" from both points
    pub fn point_point_equal_distance_points(pt1: Point, pt2: Point, d: f64) -> Intersections {
        // The result is then shifted to pt2.
        // VoronoiOffset.cpp:77-79
        let mut cx = (pt1.x() - pt2.x()) as f64;
        let mut cy = (pt1.y() - pt2.y()) as f64;
        let cl = cx * cx + cy * cy;
        // VoronoiOffset.cpp:80
        let discr = 4. * d * d - cl;
        // VoronoiOffset.cpp:81-84
        if discr < 0. {
            // No intersection point found, the two circles are too far away.
            return Intersections {
                count: 0,
                pts: [Vec2d::zero(), Vec2d::zero()],
            };
        }
        // VoronoiOffset.cpp:85-88
        // Avoid division by zero if a gets too small.
        let xy_swapped = cx.abs() < cy.abs();
        if xy_swapped {
            std::mem::swap(&mut cx, &mut cy);
        }
        // VoronoiOffset.cpp:89-97
        let u;
        let cnt;
        if discr == 0. {
            cnt = 1;
            u = 0.;
        } else {
            cnt = 2;
            u = 0.5 * cx * (cl * discr).sqrt() / cl;
        }
        // VoronoiOffset.cpp:98-101
        let v = 0.5 * cy - u;
        let w = 2. * cy;
        let e = 0.5 / cx;
        let f = 0.5 * cy + u;
        // VoronoiOffset.cpp:102-103
        let mut out = Intersections {
            count: cnt,
            pts: [
                Vec2d::new(-e * (v * w - cl), v),
                Vec2d::new(-e * (w * f - cl), f),
            ],
        };
        // VoronoiOffset.cpp:104-107
        if xy_swapped {
            std::mem::swap(&mut out.pts[0].x, &mut out.pts[0].y);
            std::mem::swap(&mut out.pts[1].x, &mut out.pts[1].y);
        }
        // VoronoiOffset.cpp:108-109
        out.pts[0] = out.pts[0] + cast_double(pt2);
        out.pts[1] = out.pts[1] + cast_double(pt2);

        // VoronoiOffset.cpp:111-114 (asserts)
        debug_assert!((norm(out.pts[0] - cast_double(pt1)) - d).abs() < SCALED_EPSILON);
        debug_assert!((norm(out.pts[1] - cast_double(pt1)) - d).abs() < SCALED_EPSILON);
        debug_assert!((norm(out.pts[0] - cast_double(pt2)) - d).abs() < SCALED_EPSILON);
        debug_assert!((norm(out.pts[1] - cast_double(pt2)) - d).abs() < SCALED_EPSILON);
        out
    }

    // VoronoiOffset.cpp:118-200
    // Return maximum two points, that are at distance "d" from both the line and point.
    pub fn line_point_equal_distance_points(line: &Line, ipt: Point, d: f64) -> Intersections {
        debug_assert!(line.a != ipt && line.b != ipt);
        // Calculating two points of distance "d" to a ray and a point.
        // Point.
        // VoronoiOffset.cpp:124-128
        let pt: Vec2d = cast_double(ipt);
        let mut lv: Vec2d = cast_double(line.b - line.a);
        let l2 = squared_norm(lv);
        let lpv: Vec2d = cast_double(line.a - ipt);
        let mut c = cross2f(lpv, lv);
        // VoronoiOffset.cpp:129-132
        if c < 0. {
            lv = -lv;
            c = -c;
        }

        // Line equation (ax + by + c - d * sqrt(l2)).
        // VoronoiOffset.cpp:135-136
        let mut a = -lv.y();
        let mut b = lv.x();
        // Line point shifted by -ipt is on the line.
        // VoronoiOffset.cpp:138
        debug_assert!((lpv.x() * a + lpv.y() * b + c).abs() < SCALED_EPSILON);
        // Line vector (a, b) points towards ipt.
        // VoronoiOffset.cpp:140
        debug_assert!(a * lpv.x() + b * lpv.y() < -SCALED_EPSILON);

        // VoronoiOffset.cpp:142-152 (NDEBUG block)
        #[cfg(debug_assertions)]
        {
            // Foot point of ipt on line.
            let ft = foot_pt_line(line, ipt);
            // Center point between ipt and line, its distance to both line and ipt is equal.
            let centerpt = (ft + pt) * 0.5 - pt;
            let dcenter = 0.5 * norm(ft - pt);
            // Verify that the center point
            debug_assert!(
                (centerpt.x() * a + centerpt.y() * b + c - dcenter * l2.sqrt()).abs()
                    < SCALED_EPSILON * l2.sqrt()
            );
        }

        // The result is then shifted to ipt.
        // VoronoiOffset.cpp:162-166
        let dscaled = d * l2.sqrt();
        let s = c * (2. * dscaled - c);
        if s < 0. {
            // Distance of pt from line is bigger than 2 * d.
            return Intersections {
                count: 0,
                pts: [Vec2d::zero(), Vec2d::zero()],
            };
        }
        // VoronoiOffset.cpp:167-181
        let u;
        let cnt;
        // Avoid division by zero if a gets too small.
        let xy_swapped = a.abs() < b.abs();
        if xy_swapped {
            std::mem::swap(&mut a, &mut b);
        }
        if s == 0. {
            // Distance of pt from line is 2 * d.
            cnt = 1;
            u = 0.;
        } else {
            // Distance of pt from line is smaller than 2 * d.
            cnt = 2;
            u = a * s.sqrt() / l2;
        }
        // VoronoiOffset.cpp:182-187
        let e = dscaled - c;
        let f = b * e / l2;
        let g = f - u;
        let h = f + u;
        let mut out = Intersections {
            count: cnt,
            pts: [
                Vec2d::new((-b * g + e) / a, g),
                Vec2d::new((-b * h + e) / a, h),
            ],
        };
        // VoronoiOffset.cpp:188-191
        if xy_swapped {
            std::mem::swap(&mut out.pts[0].x, &mut out.pts[0].y);
            std::mem::swap(&mut out.pts[1].x, &mut out.pts[1].y);
        }
        // VoronoiOffset.cpp:192-193
        out.pts[0] = out.pts[0] + pt;
        out.pts[1] = out.pts[1] + pt;

        // VoronoiOffset.cpp:195-198 (asserts)
        debug_assert!(
            (ray_point_distance(cast_double(line.a), cast_double(line.b - line.a), out.pts[0]) - d)
                .abs()
                < SCALED_EPSILON
        );
        debug_assert!(
            (ray_point_distance(cast_double(line.a), cast_double(line.b - line.a), out.pts[1]) - d)
                .abs()
                < SCALED_EPSILON
        );
        debug_assert!((norm(out.pts[0] - cast_double(ipt)) - d).abs() < SCALED_EPSILON);
        debug_assert!((norm(out.pts[1] - cast_double(ipt)) - d).abs() < SCALED_EPSILON);
        out
    }

    // VoronoiOffset.cpp:202-226
    // Double vertex equal to a coord_t point after conversion to double.
    // Boost uses ULP comparison (voronoi_diagram_traits::vertex_equality_predicate ULPS = 128).
    // We mirror the ULP-equality already used by voronoi_annotation.rs.
    // Used only by `on_site` (which the C++ uses inside NDEBUG verify code).
    #[allow(dead_code)]
    pub fn vertex_equal_to_point_xy(vx: f64, vy: f64, ipt: Point) -> bool {
        const ULPS: i64 = 128;
        ulp_eq(vx, ipt.x() as f64, ULPS) && ulp_eq(vy, ipt.y() as f64, ULPS)
    }

    // boost::polygon::detail::ulp_comparison<double>: |a-b| within `ulps` representable steps.
    #[allow(dead_code)]
    fn ulp_eq(a: f64, b: f64, ulps: i64) -> bool {
        if a == b {
            return true;
        }
        if a.is_nan() || b.is_nan() {
            return false;
        }
        if a.is_infinite() || b.is_infinite() {
            return a == b;
        }
        let a_bits = a.to_bits() as i64;
        let b_bits = b.to_bits() as i64;
        if (a_bits ^ b_bits) < 0 {
            return a.abs() < f64::EPSILON && b.abs() < f64::EPSILON;
        }
        (a_bits - b_bits).abs() <= ulps
    }

    // VoronoiOffset.cpp:228-234
    // dist_to_site(const Lines &lines, const VD::cell_type &cell, const Vec2d &point)
    // Used by the C++ only inside NDEBUG verification / asserts.
    #[allow(dead_code)]
    pub fn dist_to_site(diagram: &bv::Diagram, lines: &[Line], cell: &bv::Cell, point: Vec2d) -> f64 {
        let line = &lines[cell.source_index().usize()];
        let _ = diagram;
        if cell.contains_point() {
            norm(
                cast_double(
                    if cell.source_category() == bv::SourceCategory::SegmentStart {
                        line.a
                    } else {
                        line.b
                    },
                ) - point,
            )
        } else {
            norm(foot_pt_dir(cast_double(line.a), cast_double(line.b - line.a), point) - point)
        }
    }

    // VoronoiOffset.cpp:236-246
    // on_site(const Lines &lines, const VD::cell_type &cell, const Vec2d &pt)
    // Used by the C++ only inside NDEBUG verification / asserts.
    #[allow(dead_code)]
    pub fn on_site(lines: &[Line], cell: &bv::Cell, pt: Vec2d) -> bool {
        let line = &lines[cell.source_index().usize()];
        let on_contour = |ipt: Point| vertex_equal_to_point_xy(pt.x(), pt.y(), ipt);
        if cell.contains_point() {
            on_contour(contour_point_line(cell, line))
        } else {
            debug_assert!(!(on_contour(line.a) && on_contour(line.b)));
            on_contour(line.a) || on_contour(line.b)
        }
    }

    // VoronoiOffset.cpp:257-295
    // point_point_dr_dl_thresholds
    pub fn point_point_dr_dl_thresholds(
        pt1_site: Point,
        pt2_site: Point,
        voronoi_point1: Vec2d,
        voronoi_point2: Vec2d,
        threshold_tan_alpha_half: f64,
    ) -> (Vec2d, Vec2d) {
        // VoronoiOffset.cpp:275-277
        let dir_y: Vec2d = cast_double(pt2_site - pt1_site);
        let mut dir_x: Vec2d = normalized(Vec2d::new(-dir_y.y(), dir_y.x()));
        let cntr: Vec2d = (cast_double(pt1_site) + cast_double(pt2_site)) * 0.5;
        // VoronoiOffset.cpp:278-279
        let mut t1 = dot(voronoi_point1 - cntr, dir_x);
        let mut t2 = dot(voronoi_point2 - cntr, dir_x);
        // VoronoiOffset.cpp:280-284
        if t1 > t2 {
            t1 = -t1;
            t2 = -t2;
            dir_x = -dir_x;
        }
        // VoronoiOffset.cpp:285
        let x = 0.5 * norm(dir_y) * threshold_tan_alpha_half;
        // VoronoiOffset.cpp:286
        let nan = f64::NAN;
        // VoronoiOffset.cpp:287
        let mut out = (Vec2d::new(nan, nan), Vec2d::new(nan, nan));
        // VoronoiOffset.cpp:288-293
        if t2 > -x && t1 < x {
            // Intervals overlap.
            dir_x = dir_x * x;
            out.0 = if t1 < -x { cntr - dir_x } else { voronoi_point1 };
            out.1 = if t2 > x { cntr + dir_x } else { voronoi_point2 };
        }
        out
    }

    // VoronoiOffset.cpp:306-360
    // point_segment_dr_dl_thresholds
    pub fn point_segment_dr_dl_thresholds(
        pt_site: Point,
        line_site: &Line,
        voronoi_point1: Vec2d,
        voronoi_point2: Vec2d,
        threshold_tan_alpha_half: f64,
    ) -> (Vec2d, Vec2d) {
        // Foot point of the point site on the line site.
        // VoronoiOffset.cpp:324
        let ft = foot_pt_line(line_site, pt_site);
        // Minimum distance of the bisector (parabolic arc) from the two sites, squared.
        // VoronoiOffset.cpp:326-327
        let dir_pt_ft: Vec2d = cast_double(pt_site) - ft;
        let b = 0.5 * norm(dir_pt_ft);
        // VoronoiOffset.cpp:328
        let nan = f64::NAN;
        // VoronoiOffset.cpp:329
        let mut out = (Vec2d::new(nan, nan), Vec2d::new(nan, nan));
        {
            // +x, -x are the two parameters along the line_site, where threshold_tan_alpha_half is met.
            // VoronoiOffset.cpp:332
            let x = 2. * b * threshold_tan_alpha_half;
            // Project voronoi_point1/2 to line_site.
            // VoronoiOffset.cpp:334
            let mut dir_x: Vec2d = normalized(cast_double(line_site.b - line_site.a));
            // VoronoiOffset.cpp:335-336
            let mut t1 = dot(voronoi_point1 - ft, dir_x);
            let mut t2 = dot(voronoi_point2 - ft, dir_x);
            // VoronoiOffset.cpp:337-341
            if t1 > t2 {
                t1 = -t1;
                t2 = -t2;
                dir_x = -dir_x;
            }
            // VoronoiOffset.cpp:342-357
            if t2 > -x && t1 < x {
                // Intervals overlap.
                let t1_valid = t1 < -x;
                let t2_valid = t2 > x;
                // Direction of the Y axis of the parabola.
                let mut dir_y = Vec2d::new(-dir_x.y(), dir_x.x());
                // Orient the Y axis towards the point site.
                if dot(dir_y, dir_pt_ft) < 0. {
                    dir_y = -dir_y;
                }
                // Equation of the parabola: y = b + a * x^2
                let a = 0.25 / b;
                dir_x = dir_x * x;
                dir_y = dir_y * (b + a * x * x);
                out.0 = if t1_valid {
                    ft - dir_x + dir_y
                } else {
                    voronoi_point1
                };
                out.1 = if t2_valid {
                    ft + dir_x + dir_y
                } else {
                    voronoi_point2
                };
            }
        }
        out
    }

    // VoronoiOffset.cpp:362-391
    // point_point_skeleton_thresholds
    #[allow(dead_code)]
    pub fn point_point_skeleton_thresholds(
        pt1_site: Point,
        pt2_site: Point,
        voronoi_point1: Vec2d,
        voronoi_point2: Vec2d,
        tan_alpha_half: f64,
    ) -> (Vec2d, Vec2d) {
        // VoronoiOffset.cpp:371-373
        let dir_y: Vec2d = cast_double(pt2_site - pt1_site);
        let mut dir_x: Vec2d = normalized(Vec2d::new(-dir_y.y(), dir_y.x()));
        let cntr: Vec2d = (cast_double(pt1_site) + cast_double(pt2_site)) * 0.5;
        // VoronoiOffset.cpp:374-375
        let mut t1 = dot(voronoi_point1 - cntr, dir_x);
        let mut t2 = dot(voronoi_point2 - cntr, dir_x);
        // VoronoiOffset.cpp:376-380
        if t1 > t2 {
            t1 = -t1;
            t2 = -t2;
            dir_x = -dir_x;
        }
        // VoronoiOffset.cpp:381
        let x = 0.5 * norm(dir_y) * tan_alpha_half;
        // VoronoiOffset.cpp:382
        let nan = f64::NAN;
        // VoronoiOffset.cpp:383
        let mut out = (Vec2d::new(nan, nan), Vec2d::new(nan, nan));
        // VoronoiOffset.cpp:384-389
        if t2 > -x && t1 < x {
            // Intervals overlap.
            dir_x = dir_x * x;
            out.0 = if t1 < -x { cntr - dir_x } else { voronoi_point1 };
            out.1 = if t2 > x { cntr + dir_x } else { voronoi_point2 };
        }
        out
    }

    // VoronoiOffset.cpp:393-439
    // point_segment_skeleton_thresholds
    #[allow(dead_code)]
    pub fn point_segment_skeleton_thresholds(
        pt_site: Point,
        line_site: &Line,
        voronoi_point1: Vec2d,
        voronoi_point2: Vec2d,
        threshold_cos_alpha: f64,
    ) -> (Vec2d, Vec2d) {
        // Foot point of the point site on the line site.
        // VoronoiOffset.cpp:402
        let ft = foot_pt_line(line_site, pt_site);
        // Minimum distance of the bisector (parabolic arc) from the two sites, squared.
        // VoronoiOffset.cpp:404
        let dir_pt_ft: Vec2d = cast_double(pt_site) - ft;
        // Distance of Voronoi point site from the Voronoi line site.
        // VoronoiOffset.cpp:406
        let l = norm(dir_pt_ft);
        // VoronoiOffset.cpp:407
        let nan = f64::NAN;
        // VoronoiOffset.cpp:408
        let mut out = (Vec2d::new(nan, nan), Vec2d::new(nan, nan));
        // +x, -x are the two parameters along the line_site, where threshold is met.
        // VoronoiOffset.cpp:410-412
        let r = l / (1. + threshold_cos_alpha);
        let x2 = r * r - sqr(l - r);
        let x = x2.sqrt();
        // Project voronoi_point1/2 to line_site.
        // VoronoiOffset.cpp:414
        let mut dir_x: Vec2d = normalized(cast_double(line_site.b - line_site.a));
        // VoronoiOffset.cpp:415-416
        let mut t1 = dot(voronoi_point1 - ft, dir_x);
        let mut t2 = dot(voronoi_point2 - ft, dir_x);
        // VoronoiOffset.cpp:417-421
        if t1 > t2 {
            t1 = -t1;
            t2 = -t2;
            dir_x = -dir_x;
        }
        // VoronoiOffset.cpp:422-437
        if t2 > -x && t1 < x {
            // Intervals overlap.
            let t1_valid = t1 < -x;
            let t2_valid = t2 > x;
            // Direction of the Y axis of the parabola.
            let mut dir_y = Vec2d::new(-dir_x.y(), dir_x.x());
            // Orient the Y axis towards the point site.
            if dot(dir_y, dir_pt_ft) < 0. {
                dir_y = -dir_y;
            }
            // Equation of the parabola: y = b + a * x^2
            let a = 0.5 / l;
            dir_x = dir_x * x;
            dir_y = dir_y * (0.5 * l + a * x2);
            out.0 = if t1_valid {
                ft - dir_x + dir_y
            } else {
                voronoi_point1
            };
            out.1 = if t2_valid {
                ft + dir_x + dir_y
            } else {
                voronoi_point2
            };
        }
        out
    }
} // namespace detail

// ---------------------------------------------------------------------------
// reset_inside_outside_annotations  (VoronoiOffset.cpp:640-648)
// ---------------------------------------------------------------------------
pub fn reset_inside_outside_annotations(diagram: &mut bv::Diagram) {
    // VoronoiOffset.cpp:642-643
    // for (const VD::vertex_type &v : vd.vertices())
    //     set_vertex_category(v, VertexCategory::Unknown);
    let vertex_ids: Vec<bv::VertexIndex> =
        diagram.vertices().iter().map(|v| v.get_id()).collect();
    for vertex_id in vertex_ids {
        set_vertex_category(diagram, vertex_id, VertexCategory::Unknown);
    }
    // VoronoiOffset.cpp:644-645
    // for (const VD::edge_type &e : vd.edges())
    //     set_edge_category(e, EdgeCategory::Unknown);
    for edge_idx in 0..diagram.num_edges() {
        let edge_id = diagram.edge_index_unchecked(edge_idx);
        set_edge_category(diagram, edge_id, EdgeCategory::Unknown);
    }
    // VoronoiOffset.cpp:646-647
    // for (const VD::cell_type &c : vd.cells())
    //     set_cell_category(c, CellCategory::Unknown);
    let cell_ids: Vec<bv::CellIndex> = diagram.cells().iter().map(|c| c.id()).collect();
    for cell_id in cell_ids {
        set_cell_category(diagram, cell_id, CellCategory::Unknown);
    }
}

// ---------------------------------------------------------------------------
// signed_vertex_distances  (VoronoiOffset.cpp:969-1010)
// ---------------------------------------------------------------------------
pub fn signed_vertex_distances(diagram: &bv::Diagram, lines: &[Line]) -> Vec<f64> {
    // vd shall be annotated.
    // assert(debug::verify_inside_outside_annotations(vd));

    // VoronoiOffset.cpp:974
    let mut out: Vec<f64> = vec![0.; diagram.vertices().len()];
    // VoronoiOffset.cpp:976
    for vertex_idx in 0..diagram.vertices().len() {
        let vertex = &diagram.vertices()[vertex_idx];
        let vertex_id = vertex.get_id();
        // VoronoiOffset.cpp:977
        let vc = vertex_category(diagram, vertex_id);
        // VoronoiOffset.cpp:978
        let dist;
        // VoronoiOffset.cpp:979-1003
        if vc == VertexCategory::OnContour {
            dist = 0.;
        } else {
            // VoronoiOffset.cpp:982-991
            let first_edge = vertex.get_incident_edge().ok();
            let mut edge = first_edge;
            let mut point_cell: Option<bv::CellIndex> = None;
            // do { ... } while (edge != first_edge);
            if let Some(fe) = first_edge {
                let mut cur = fe;
                loop {
                    if let Ok(cell_id) = diagram.edge_get_cell(cur) {
                        if let Ok(cell) = diagram.cell(cell_id) {
                            if cell.contains_point() {
                                point_cell = Some(cell_id);
                                break;
                            }
                        }
                    }
                    // edge = edge->rot_next();
                    match diagram.edge_rot_next(cur) {
                        Some(n) => cur = n,
                        None => break,
                    }
                    if cur == fe {
                        break;
                    }
                }
                edge = Some(cur);
            }
            // VoronoiOffset.cpp:992-1000
            if point_cell.is_none() {
                // Project vertex onto a contour segment.
                let line = &lines[diagram
                    .edge_get_cell(edge.unwrap())
                    .unwrap()
                    .pipe_source_index(diagram)];
                let d = ray_point_distance(
                    cast_double(line.a),
                    cast_double(line.b - line.a),
                    vertex_point(vertex),
                );
                // VoronoiOffset.cpp:1001-1002
                dist = if vc == VertexCategory::Inside { -d } else { d };
            } else {
                // Distance to a contour point.
                let cell = diagram.cell(point_cell.unwrap()).unwrap();
                let cp = contour_point_lines(cell, lines);
                let d = norm(cast_double(cp) - vertex_point(vertex));
                // VoronoiOffset.cpp:1001-1002
                dist = if vc == VertexCategory::Inside { -d } else { d };
            }
        }
        // VoronoiOffset.cpp:1004
        out[vertex_idx] = dist;
    }

    // assert(debug::verify_signed_distances(vd, lines, out));
    out
}

// Tiny helper to keep the C++ `lines[edge->cell()->source_index()]` chain readable.
trait CellIndexSourcePipe {
    fn pipe_source_index(self, diagram: &bv::Diagram) -> usize;
}
impl CellIndexSourcePipe for bv::CellIndex {
    fn pipe_source_index(self, diagram: &bv::Diagram) -> usize {
        diagram.cell(self).unwrap().source_index().usize()
    }
}

// ---------------------------------------------------------------------------
// edge_offset_contour_intersections  (VoronoiOffset.cpp:1012-1327)
// ---------------------------------------------------------------------------
pub fn edge_offset_contour_intersections(
    diagram: &bv::Diagram,
    lines: &[Line],
    vertex_distances: &[f64],
    mut offset_distance: f64,
) -> Vec<Vec2d> {
    // vd shall be annotated.
    // assert(debug::verify_inside_outside_annotations(vd));

    // VoronoiOffset.cpp:1021-1024
    let outside = offset_distance > 0.;
    if !outside {
        offset_distance = -offset_distance;
    }
    debug_assert!(offset_distance > 0.);

    // VoronoiOffset.cpp:1028
    let nan = f64::NAN;
    // By default none edge has an intersection with the offset curve.
    // VoronoiOffset.cpp:1030
    let num_edges = diagram.num_edges();
    let mut out: Vec<Vec2d> = vec![Vec2d::new(nan, 0.); num_edges];

    // VoronoiOffset.cpp:1032
    for edge_idx in 0..num_edges {
        let edge_id = diagram.edge_index_unchecked(edge_idx);
        // VoronoiOffset.cpp:1034-1036
        if edge_offset_has_intersection(&out[edge_idx]) || out[edge_idx].y() != 0. {
            // This edge was already classified.
            continue;
        }

        // VoronoiOffset.cpp:1038-1039
        let v0 = diagram.edge_get_vertex0(edge_id).ok().flatten();
        let v1 = diagram.edge_get_vertex1(edge_id).ok().flatten();
        // VoronoiOffset.cpp:1040-1043
        if v0.is_none() {
            // assert(vertex_category(v1) == OnContour || Outside);
            continue;
        }
        let v0 = v0.unwrap();

        // VoronoiOffset.cpp:1045-1046
        let mut d0 = vertex_distances[v0.usize()];
        let mut d1 = if let Some(v1) = v1 {
            vertex_distances[v1.usize()]
        } else {
            f64::MAX
        };
        // VoronoiOffset.cpp:1047
        debug_assert!(d0 * d1 >= 0.);
        // VoronoiOffset.cpp:1048-1051
        if !outside {
            d0 = -d0;
            d1 = -d1;
        }

        // VoronoiOffset.cpp:1066-1070
        let dmin;
        let dmax;
        if d0 < d1 {
            dmin = d0;
            dmax = d1;
        } else {
            dmax = d0;
            dmin = d1;
        }
        // Offset distance may be lower than dmin, but never higher than dmax.
        // VoronoiOffset.cpp:1076-1077
        if offset_distance >= dmax {
            continue;
        }

        // Edge candidate, intersection points were not calculated yet.
        // VoronoiOffset.cpp:1081-1085
        let cell_id = diagram.edge_get_cell(edge_id).unwrap();
        let twin_id = diagram.edge_get_twin(edge_id).unwrap();
        let cell2_id = diagram.edge_get_cell(twin_id).unwrap();
        let cell = *diagram.cell(cell_id).unwrap();
        let cell2 = *diagram.cell(cell2_id).unwrap();
        let line0 = lines[cell.source_index().usize()];
        let line1 = lines[cell2.source_index().usize()];
        let edge_idx2 = twin_id.usize();

        let v0_pt = vertex_point(diagram.vertex(v0).unwrap());

        // VoronoiOffset.cpp:1086-1127
        if v1.is_none() {
            // assert(edge.is_infinite()); assert(edge.is_linear());
            // Unconstrained edges have always montonous distance.
            debug_assert!(d0 != d1);
            // VoronoiOffset.cpp:1091
            if offset_distance > dmin {
                // There is certainly an intersection with the offset curve.
                // VoronoiOffset.cpp:1093-1102
                if cell.contains_point() && cell2.contains_point() {
                    // assert(! edge.is_secondary());
                    let pt0 = contour_point_line(&cell, &line0);
                    let pt1 = contour_point_line(&cell2, &line1);
                    // pt is inside the circle (pt0, offset_distance), (pt + dir) is certainly outside.
                    let dir = Vec2d::new(
                        (pt0.y() - pt1.y()) as f64,
                        (pt1.x() - pt0.x()) as f64,
                    ) * (2. * offset_distance);
                    let pt = Vec2d::new(v0_pt.x(), v0_pt.y());
                    let t = detail::first_circle_segment_intersection_parameter(
                        Vec2d::new(pt0.x() as f64, pt0.y() as f64),
                        offset_distance,
                        pt,
                        dir,
                    );
                    debug_assert!(t > 0.);
                    out[edge_idx] = pt + dir * t;
                } else {
                    // VoronoiOffset.cpp:1103-1123
                    // Infinite edges could not be created by two segment sites.
                    debug_assert!(cell.contains_point() != cell2.contains_point());
                    // Linear edge goes through the endpoint of a segment.
                    // assert(edge.is_secondary());
                    let ipt = if cell.contains_segment() {
                        contour_point_line(&cell2, &line1)
                    } else {
                        contour_point_line(&cell, &line0)
                    };
                    // Infinite edge starts at an input contour, there is always an intersection.
                    let line = if cell.contains_segment() { &line0 } else { &line1 };
                    debug_assert!(line.a == ipt || line.b == ipt);
                    out[edge_idx] = cast_double(ipt)
                        + normalized(Vec2d::new(
                            (line.b.y() - line.a.y()) as f64,
                            (line.a.x() - line.b.x()) as f64,
                        )) * offset_distance;
                }
            } else if offset_distance == dmin {
                // VoronoiOffset.cpp:1124-1125
                out[edge_idx] = v0_pt;
            }
            // The other edge of an unconstrained edge starting with null vertex
            // shall never be intersected. Mark it as visited.
            // VoronoiOffset.cpp:1127
            out[edge_idx2].y = 1.;
        } else {
            let v1 = v1.unwrap();
            let v1_pt = vertex_point(diagram.vertex(v1).unwrap());
            // assert(edge.is_finite());
            // VoronoiOffset.cpp:1130
            let mut done = false;
            // Bisector of two line segments, distance along the bisector is linear.
            // VoronoiOffset.cpp:1132
            let bisector = cell.contains_segment() && cell2.contains_segment();
            let edge_is_secondary = diagram.edges()[edge_idx].is_secondary();
            // VoronoiOffset.cpp:1135-1159
            if bisector || edge_is_secondary {
                // assert(edge.is_linear());
                // VoronoiOffset.cpp:1146
                if !bisector || (dmin != dmax && offset_distance >= dmin) {
                    // VoronoiOffset.cpp:1147-1148
                    let mut t = (offset_distance - dmin) / (dmax - dmin);
                    t = t.clamp(0., 1.);
                    // VoronoiOffset.cpp:1149-1157
                    if d1 < d0 {
                        out[edge_idx2] = lerpf(v1_pt, v0_pt, t);
                        // mark visited
                        out[edge_idx].y = 1.;
                    } else {
                        out[edge_idx] = lerpf(v0_pt, v1_pt, t);
                        // mark visited
                        out[edge_idx2].y = 1.;
                    }
                    done = true;
                }
            } else {
                // Point - Segment or Point - Point edge, distance along this Voronoi edge
                // may not be monotonous (see C++ comment block VoronoiOffset.cpp:1161-1179).
                debug_assert!(cell.contains_point() || cell2.contains_point());
                // VoronoiOffset.cpp:1181-1188
                let mut num_intersections: usize = 0;
                let point_vs_segment = cell.contains_point() != cell2.contains_point();
                let pt0 = if cell.contains_point() {
                    contour_point_line(&cell, &line0)
                } else {
                    contour_point_line(&cell2, &line1)
                };
                // Project p0 to line segment <v0, v1>.
                let p0 = Vec2d::new(v0_pt.x(), v0_pt.y());
                let p1 = Vec2d::new(v1_pt.x(), v1_pt.y());
                let px = Vec2d::new(pt0.x() as f64, pt0.y() as f64);
                // VoronoiOffset.cpp:1188-1197
                let mut pt1_opt: Option<Point> = None;
                let dir: Vec2d;
                if point_vs_segment {
                    let line = if cell.contains_segment() { &line0 } else { &line1 };
                    dir = cast_double(line.b - line.a);
                } else {
                    let p = contour_point_line(&cell2, &line1);
                    pt1_opt = Some(p);
                    // Perpendicular to the (pt1 - pt0) direction.
                    dir = Vec2d::new((pt0.y() - p.y()) as f64, (p.x() - pt0.x()) as f64);
                }
                // VoronoiOffset.cpp:1198-1199
                let s0 = dot(p0 - px, dir);
                let s1 = dot(p1 - px, dir);
                // VoronoiOffset.cpp:1200-1240
                let mut dmin = dmin;
                if offset_distance >= dmin {
                    // This Voronoi edge is intersected by the offset curve just once.
                    num_intersections = 1;
                } else {
                    // VoronoiOffset.cpp:1207-1239
                    let mut dmin_new = 0.0_f64;
                    let mut found = false;
                    if point_vs_segment {
                        if s0 * s1 <= 0. {
                            // VoronoiOffset.cpp:1215-1217
                            let line = if cell.contains_segment() { &line0 } else { &line1 };
                            dmin_new =
                                0.5 * norm(foot_pt_dir(cast_double(line.a), dir, px) - px);
                            found = true;
                        }
                    } else {
                        // Point-Point Voronoi sites.
                        if s0 * s1 <= 0. {
                            // VoronoiOffset.cpp:1223-1224
                            dmin_new = 0.5 * norm(cast_double(pt1_opt.unwrap()) - px);
                            found = true;
                        }
                    }
                    if found {
                        debug_assert!(dmin_new < dmax + SCALED_EPSILON);
                        debug_assert!(dmin_new < dmin + SCALED_EPSILON);
                        // VoronoiOffset.cpp:1230-1238
                        if dmin_new <= offset_distance {
                            dmin = dmin_new;
                            num_intersections = (offset_distance > dmin) as usize + 1;
                        }
                    }
                }
                // VoronoiOffset.cpp:1241-1317
                if num_intersections > 0 {
                    let mut intersections = if point_vs_segment {
                        debug_assert!(cell.contains_point() || cell2.contains_point());
                        detail::line_point_equal_distance_points(
                            if cell.contains_segment() { &line0 } else { &line1 },
                            pt0,
                            offset_distance,
                        )
                    } else {
                        detail::point_point_equal_distance_points(
                            pt0,
                            pt1_opt.unwrap(),
                            offset_distance,
                        )
                    };
                    // Adjust the result to the number of intersection points expected.
                    // VoronoiOffset.cpp:1252-1304
                    if num_intersections == 2 {
                        match intersections.count {
                            0 => {
                                // No intersection found even though one or two were expected.
                            }
                            1 => {
                                // Tangential point found.
                            }
                            _ => {
                                // Two intersection points found. Sort them.
                                debug_assert!(intersections.count == 2);
                                let q0 = dot(intersections.pts[0] - px, dir);
                                let q1 = dot(intersections.pts[1] - px, dir);
                                debug_assert!(q0 * q1 <= 0.);
                                debug_assert!(s0 * s1 <= 0.);
                                // Sort the intersection points by dir.
                                if (q0 < q1) != (s0 < s1) {
                                    intersections.pts.swap(0, 1);
                                }
                            }
                        }
                    } else {
                        debug_assert!(num_intersections == 1);
                        match intersections.count {
                            0 => {
                                // No intersection found. This should not happen.
                                // Create one artificial intersection point by repeating the
                                // dmin point, which is supposed to be close to the minimum.
                                intersections.pts[0] = if dmin == d0 { p0 } else { p1 };
                                intersections.count = 1;
                            }
                            1 => {
                                // One intersection found. This is a tangential point. Use it.
                            }
                            _ => {
                                // Two intersections found.
                                debug_assert!(intersections.count == 2);
                                let q0 = dot(intersections.pts[0] - px, dir);
                                let q1 = dot(intersections.pts[1] - px, dir);
                                debug_assert!(q0 * q1 <= 0.);
                                let s = if dmax == d0 { s0 } else { s1 };
                                let take_2nd = if s > 0. { q1 > q0 } else { q1 < q0 };
                                if take_2nd {
                                    intersections.pts[0] = intersections.pts[1];
                                }
                                intersections.count -= 1;
                            }
                        }
                    }
                    // VoronoiOffset.cpp:1305-1316
                    debug_assert!(intersections.count > 0);
                    if intersections.count == 2 {
                        out[edge_idx] = intersections.pts[1];
                        out[edge_idx2] = intersections.pts[0];
                        done = true;
                    } else if intersections.count == 1 {
                        let (ei, ei2) = if d1 < d0 {
                            (edge_idx2, edge_idx)
                        } else {
                            (edge_idx, edge_idx2)
                        };
                        out[ei] = intersections.pts[0];
                        out[ei2].y = 1.;
                        done = true;
                    }
                }
            }
            // VoronoiOffset.cpp:1319-1320
            if !done {
                out[edge_idx].y = 1.;
                out[edge_idx2].y = 1.;
            }
        }
    }

    // assert(debug::verify_offset_intersection_points(vd, lines, offset_distance, out));
    out
}

// ---------------------------------------------------------------------------
// offset (signed_vertex_distances overload)  (VoronoiOffset.cpp:1329-1511)
// ---------------------------------------------------------------------------
pub fn offset_with_distances(
    diagram: &bv::Diagram,
    lines: &[Line],
    signed_vertex_distances: &[f64],
    offset_distance: f64,
    discretization_error: f64,
) -> Vec<Polygon> {
    // VoronoiOffset.cpp:1396
    let mut edge_points =
        edge_offset_contour_intersections(diagram, lines, signed_vertex_distances, offset_distance);
    let num_edges = diagram.num_edges();

    // VoronoiOffset.cpp:1409-1415
    // next_offset_edge: returns the twin of the first subsequent edge (in the `next` ring
    // around the current edge) whose twin's edge has an offset intersection.
    let next_offset_edge =
        |edge_points: &[Vec2d], start_edge: bv::EdgeIndex| -> Option<bv::EdgeIndex> {
            let mut edge = diagram.edge_get_next(start_edge).ok()?;
            while edge != start_edge {
                let twin = diagram.edge_get_twin(edge).ok()?;
                if edge_offset_has_intersection(&edge_points[twin.usize()]) {
                    return Some(twin);
                }
                edge = diagram.edge_get_next(edge).ok()?;
            }
            // assert(false);
            None
        };

    // VoronoiOffset.cpp:1417-1419
    let inside_offset = offset_distance < 0.;
    let offset_distance = if inside_offset {
        -offset_distance
    } else {
        offset_distance
    };

    // Track the offset curves.
    // VoronoiOffset.cpp:1422-1425
    let mut out: Vec<Polygon> = Vec::new();
    let angle_step = 2. * ((offset_distance - discretization_error) / offset_distance).acos();
    let cos_threshold = angle_step.cos();
    let nan = f64::NAN;
    // VoronoiOffset.cpp:1426
    for seed_edge_idx in 0..num_edges {
        // VoronoiOffset.cpp:1427
        let last_pt0 = edge_points[seed_edge_idx];
        // VoronoiOffset.cpp:1428
        if edge_offset_has_intersection(&last_pt0) {
            let start_edge = diagram.edge_index_unchecked(seed_edge_idx);
            let mut edge = start_edge;
            let mut poly = Polygon::new();
            let mut last_pt = last_pt0;
            // VoronoiOffset.cpp:1432-1501  do { ... } while (edge != start_edge);
            loop {
                // find the next edge
                // VoronoiOffset.cpp:1434
                let next_edge = next_offset_edge(&edge_points, edge);
                // assert(next_edge);
                let next_edge = match next_edge {
                    Some(n) => n,
                    None => break,
                };
                // Interpolate a circular segment or insert a linear segment.
                // VoronoiOffset.cpp:1445
                let cell_id = diagram.edge_get_cell(edge).unwrap();
                let cell = *diagram.cell(cell_id).unwrap();
                // Mark the edge / offset curve intersection point as consumed.
                // VoronoiOffset.cpp:1447-1449
                let p1 = last_pt;
                let p2 = edge_points[next_edge.usize()];
                edge_points[next_edge.usize()].x = nan;
                // VoronoiOffset.cpp:1464-1493
                if cell.contains_point() {
                    // Discretize an arc from p1 to p2 with radius = offset_distance.
                    // VoronoiOffset.cpp:1468-1469
                    let line0 = lines[cell.source_index().usize()];
                    let center = cast_double(
                        if cell.source_category() == bv::SourceCategory::SegmentStart {
                            line0.a
                        } else {
                            line0.b
                        },
                    );
                    // VoronoiOffset.cpp:1470-1474
                    let v1 = p1 - center;
                    let v2 = p2 - center;
                    let ccw = cross2f(v1, v2) > 0.;
                    let mut cos_a = dot(v1, v2);
                    let nrm = norm(v1) * norm(v2);
                    debug_assert!(nrm > 0.);
                    // VoronoiOffset.cpp:1476-1492
                    if cos_a < cos_threshold * nrm {
                        // Angle is bigger than the threshold, the arc will be discretized.
                        cos_a /= nrm;
                        debug_assert!(cos_a > -1. - EPSILON && cos_a < 1. + EPSILON);
                        let angle = cos_a.max(-1.).min(1.).acos();
                        let n_steps = (angle / angle_step).ceil() as usize;
                        let mut astep = angle / n_steps as f64;
                        if !ccw {
                            astep *= -1.;
                        }
                        let mut a = astep;
                        let mut i = 1;
                        while i < n_steps {
                            let c = a.cos();
                            let s = a.sin();
                            let p = center
                                + Vec2d::new(
                                    c * v1.x() - s * v1.y(),
                                    s * v1.x() + c * v1.y(),
                                );
                            poly.points.push(Point::new(p.x() as Coord, p.y() as Coord));
                            i += 1;
                            a += astep;
                        }
                    }
                }
                // VoronoiOffset.cpp:1494-1498
                {
                    let pt_last = Point::new(p2.x() as Coord, p2.y() as Coord);
                    if poly.points.is_empty() || *poly.points.last().unwrap() != pt_last {
                        poly.points.push(pt_last);
                    }
                }
                // VoronoiOffset.cpp:1499-1500
                edge = next_edge;
                last_pt = p2;
                // VoronoiOffset.cpp:1501
                if edge == start_edge {
                    break;
                }
            }

            // VoronoiOffset.cpp:1503-1506
            while !poly.points.is_empty() && poly.points.first() == poly.points.last() {
                poly.points.pop();
            }
            if poly.points.len() >= 3 {
                out.push(poly);
            }
        }
    }

    out
}

// ---------------------------------------------------------------------------
// offset (top-level overload)  (VoronoiOffset.cpp:1513-1522)
// ---------------------------------------------------------------------------
pub fn offset(
    diagram: &mut bv::Diagram,
    lines: &[Line],
    offset_distance: f64,
    discretization_error: f64,
) -> Vec<Polygon> {
    // VoronoiOffset.cpp:1519
    annotate_inside_outside(diagram, lines);
    // VoronoiOffset.cpp:1520
    let dist = signed_vertex_distances(diagram, lines);
    // VoronoiOffset.cpp:1521
    offset_with_distances(diagram, lines, &dist, offset_distance, discretization_error)
}

// ---------------------------------------------------------------------------
// skeleton_edges_rough  (VoronoiOffset.cpp:1535-1635)
// ---------------------------------------------------------------------------
pub fn skeleton_edges_rough(diagram: &bv::Diagram, lines: &[Line], threshold_alpha: f64) -> Vec<Vec2d> {
    // vd shall be annotated.
    // assert(debug::verify_inside_outside_annotations(vd));

    // VoronoiOffset.cpp:1545
    let nan = f64::NAN;
    // By default no edge is annotated as being part of the skeleton.
    // VoronoiOffset.cpp:1547
    let num_edges = diagram.num_edges();
    let mut out: Vec<Vec2d> = vec![Vec2d::new(nan, nan); num_edges];
    // Threshold at a sharp corner, derived from a dot product of the sharp corner edges.
    // VoronoiOffset.cpp:1549
    let threshold_cos_alpha = threshold_alpha.cos();
    // For sharp corners, dr/dl = sin(alpha/2). Substituting the dr/dl threshold with tan(alpha/2).
    // VoronoiOffset.cpp:1552
    let threshold_tan_alpha_half = (0.5 * threshold_alpha).tan();

    // VoronoiOffset.cpp:1554
    for edge_idx in 0..num_edges {
        let edge_id = diagram.edge_index_unchecked(edge_idx);
        let edge = &diagram.edges()[edge_idx];
        let twin_id = diagram.edge_get_twin(edge_id).unwrap();
        // VoronoiOffset.cpp:1556-1563
        if
        // Ignore secondary and unbounded edges, they shall never be part of the skeleton.
        edge.is_secondary()
            || diagram.edge_is_infinite(edge_id).unwrap_or(true)
            // Skip the twin edge of an edge, that has already been processed.
            || edge_idx > twin_id.usize()
            // Ignore outer edges.
            || (edge_category(diagram, edge_id) != EdgeCategory::PointsInside
                && edge_category(diagram, twin_id) != EdgeCategory::PointsInside)
        {
            continue;
        }
        // VoronoiOffset.cpp:1564-1570
        let v0 = diagram.edge_get_vertex0(edge_id).ok().flatten().unwrap();
        let v1 = diagram.edge_get_vertex1(edge_id).ok().flatten().unwrap();
        let cell_id = diagram.edge_get_cell(edge_id).unwrap();
        let cell2_id = diagram.edge_get_cell(twin_id).unwrap();
        let cell = *diagram.cell(cell_id).unwrap();
        let cell2 = *diagram.cell(cell2_id).unwrap();
        let line0 = lines[cell.source_index().usize()];
        let line1 = lines[cell2.source_index().usize()];
        let edge_idx2 = twin_id.usize();
        let v0_pt = vertex_point(diagram.vertex(v0).unwrap());
        let v1_pt = vertex_point(diagram.vertex(v1).unwrap());
        // VoronoiOffset.cpp:1571-1604
        if cell.contains_segment() && cell2.contains_segment() {
            // Bisector of two line segments, distance along the bisector is linear,
            // dr/dl is constant. Using sin^2(a) = (1-cos(2a))/2.
            // VoronoiOffset.cpp:1575-1577
            let lv0: Vec2d = cast_double(line0.b - line0.a);
            let lv1: Vec2d = cast_double(line1.b - line1.a);
            let d = dot(lv0, lv1);
            // VoronoiOffset.cpp:1578-1585
            if d < 0. {
                let cos_alpha = -d / (norm(lv0) * norm(lv1));
                if cos_alpha > threshold_cos_alpha {
                    // The whole bisector is a skeleton segment.
                    out[edge_idx] = v0_pt;
                    out[edge_idx2] = v1_pt;
                }
            }
        } else {
            // An infinite Voronoi Edge-Point (parabola) or Point-Point (line) bisector,
            // clipped to a finite Voronoi segment.
            // VoronoiOffset.cpp:1590-1603
            debug_assert!(cell.contains_point() || cell2.contains_point());
            if cell.contains_point() != cell2.contains_point() {
                // Point - Segment
                let pt0 = if cell.contains_point() {
                    contour_point_line(&cell, &line0)
                } else {
                    contour_point_line(&cell2, &line1)
                };
                let line = if cell.contains_segment() { &line0 } else { &line1 };
                let (a, b) = detail::point_segment_dr_dl_thresholds(
                    pt0,
                    line,
                    v0_pt,
                    v1_pt,
                    threshold_tan_alpha_half,
                );
                out[edge_idx] = a;
                out[edge_idx2] = b;
            } else {
                // Point - Point
                let pt0 = contour_point_line(&cell, &line0);
                let pt1 = contour_point_line(&cell2, &line1);
                let (a, b) = detail::point_point_dr_dl_thresholds(
                    pt0,
                    pt1,
                    v0_pt,
                    v1_pt,
                    threshold_tan_alpha_half,
                );
                out[edge_idx] = a;
                out[edge_idx2] = b;
            }
        }
    }

    out
}
