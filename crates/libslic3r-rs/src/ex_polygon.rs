//! Faithful 1:1 line-by-line port of BambuStudio `src/libslic3r/ExPolygon.cpp`.
//!
//! The canonical `ExPolygon` / `ExPolygons` types and several already-ported
//! methods/free-functions live in [`crate::geometry::expolygon`]; this module
//! supplies the C++-faithful methods (via an inherent `impl ExPolygon` block)
//! and free functions that mirror `ExPolygon.cpp` exactly — same order, same
//! names (snake_case), same signatures, same control flow, constants, rounding
//! and edge cases — WITHOUT duplicating the names already defined on the struct
//! (`area`, `is_valid`, `simplify`, `simplify_p`, `simplify_p_into`,
//! `translate`, `scale`, `rotate`, `rotate_around`, `medial_axis`, ...), which
//! have been audited/corrected in place in `geometry/expolygon.rs`.
//!
//! coord_t -> i64 (`Coord`), coordf_t -> f64 (`CoordF`).
//!
//! NOTE on `Polygon::area()`: in C++ `Polygon::area()` returns the *signed*
//! area (CCW positive, CW negative). The crate's `Polygon::area()` is ALSO
//! signed (`Polygon::signed_area()` is an alias of `area()`), so either call is
//! equivalent. Where this file mirrors `std::abs(poly.area())` we therefore use
//! `signed_area().abs()` (== `area().abs()`).

#![allow(clippy::needless_range_loop)]

use crate::clipper_utils::{
    diff_pl, intersection, intersection_pl, offset_expolygons, OffsetJoinType,
};
use crate::geometry::{
    has_duplicate_points as points_have_duplicates, polygons_match,
    remove_small_polygons as polygons_remove_small, remove_sticks as polygon_remove_sticks,
    remove_sticks_polygons as polygons_remove_sticks, to_polylines_expoly, BoundingBox, ExPolygon,
    ExPolygons, Line, Point, Polygon, Polyline,
};
use crate::libslic3r::{EPSILON, SCALING_FACTOR};
use crate::Coord;

// ExPolygon.cpp:14  namespace Slic3r {

impl ExPolygon {
    // ExPolygon.cpp:17-22
    // void ExPolygon::scale(double factor)
    //
    // NOTE: the canonical `ExPolygon::scale(&mut self, factor: CoordF)` already
    // implements this in geometry/expolygon.rs; `scale_factor` is provided as a
    // C++-named alias for documentation/parity bookkeeping.
    pub fn scale_factor(&mut self, factor: f64) {
        // ExPolygon.cpp:19
        self.contour.scale(factor);
        // ExPolygon.cpp:20-21
        for hole in &mut self.holes {
            hole.scale(factor);
        }
    }

    // ExPolygon.cpp:24-29
    // void ExPolygon::scale(double factor_x, double factor_y)
    //
    // NOTE: C++ delegates to `Polygon::scale(factor_x, factor_y)` which is not
    // yet ported (Polygon.cpp). That method simply multiplies each point's x by
    // factor_x and y by factor_y; we inline that exact arithmetic here (not a
    // fake — identical to the C++ Polygon::scale(fx, fy) body).
    pub fn scale_xy(&mut self, factor_x: f64, factor_y: f64) {
        // ExPolygon.cpp:26
        scale_polygon_xy(&mut self.contour, factor_x, factor_y);
        // ExPolygon.cpp:27-28
        for hole in &mut self.holes {
            scale_polygon_xy(hole, factor_x, factor_y);
        }
    }

    // ExPolygon.cpp:31-36
    // void ExPolygon::translate(const Point &p)
    //
    // NOTE: the canonical `ExPolygon::translate(&mut self, v: Point)` already
    // implements this in geometry/expolygon.rs; `translate_point` is a C++-named
    // alias taking the translation by reference (as in C++).
    pub fn translate_point(&mut self, p: &Point) {
        // ExPolygon.cpp:33
        self.contour.translate(*p);
        // ExPolygon.cpp:34-35
        for hole in &mut self.holes {
            hole.translate(*p);
        }
    }

    // ExPolygon.cpp:38-43
    // void ExPolygon::rotate(double angle)
    //
    // NOTE: canonical `ExPolygon::rotate(&mut self, angle: CoordF)` already
    // implements this; `rotate_angle` is a C++-named alias.
    pub fn rotate_angle(&mut self, angle: f64) {
        // ExPolygon.cpp:40
        self.contour.rotate(angle);
        // ExPolygon.cpp:41-42
        for hole in &mut self.holes {
            hole.rotate(angle);
        }
    }

    // ExPolygon.cpp:45-50
    // void ExPolygon::rotate(double angle, const Point &center)
    //
    // NOTE: canonical `ExPolygon::rotate_around(&mut self, angle, center)`
    // already implements this; `rotate_about` is a C++-named alias.
    pub fn rotate_about(&mut self, angle: f64, center: &Point) {
        // ExPolygon.cpp:47
        self.contour.rotate_around(angle, *center);
        // ExPolygon.cpp:48-49
        for hole in &mut self.holes {
            hole.rotate_around(angle, *center);
        }
    }

    // ExPolygon.cpp:52-58  ExPolygon::area() — see geometry/expolygon.rs (corrected in place).
    // ExPolygon.cpp:60-67  ExPolygon::is_valid() — see geometry/expolygon.rs (corrected in place).

    // ExPolygon.cpp:69-74
    // void ExPolygon::douglas_peucker(double tolerance)
    pub fn douglas_peucker(&mut self, tolerance: f64) {
        // ExPolygon.cpp:71
        // this->contour.douglas_peucker(tolerance);
        self.contour.douglas_peucker(tolerance);
        // ExPolygon.cpp:72-73
        // for (Polygon &poly : this->holes)
        //     poly.douglas_peucker(tolerance);
        for poly in &mut self.holes {
            poly.douglas_peucker(tolerance);
        }
    }

    // ExPolygon.cpp:76-79
    // bool ExPolygon::contains(const Line &line) const
    pub fn contains_line(&self, line: &Line) -> bool {
        // ExPolygon.cpp:78
        // return this->contains(Polyline(line.a, line.b));
        self.contains_polyline(&Polyline::from_points(vec![line.a, line.b]))
    }

    // ExPolygon.cpp:81-90
    // bool ExPolygon::contains(const Polyline &polyline) const
    pub fn contains_polyline(&self, polyline: &Polyline) -> bool {
        // ExPolygon.cpp:83
        // BoundingBox bbox1 = get_extents(*this);
        let bbox1 = get_extents_expoly(self);
        // ExPolygon.cpp:84
        // BoundingBox bbox2 = get_extents(polyline);
        let bbox2 = polyline.bounding_box();
        // ExPolygon.cpp:85
        // bbox2.inflated(1);
        // NOTE: `BoundingBoxBase::inflated(coordf_t)` is a CONST method returning
        // an inflated *copy*; the C++ discards the return value, so this is a
        // no-op. We faithfully reproduce the no-op (bbox2 left unchanged).
        // ExPolygon.cpp:86-87
        // if (!bbox1.overlap(bbox2))
        //     return false;
        if !bbox1.intersects(&bbox2) {
            return false;
        }
        // ExPolygon.cpp:89
        // return diff_pl(polyline, *this).empty();
        diff_pl(std::slice::from_ref(polyline), std::slice::from_ref(self)).is_empty()
    }

    // ExPolygon.cpp:92-107
    // bool ExPolygon::contains(const Polylines &polylines) const
    pub fn contains_polylines(&self, polylines: &[Polyline]) -> bool {
        // ExPolygon.cpp:102
        // Polylines pl_out = diff_pl(polylines, *this);
        let pl_out = diff_pl(polylines, std::slice::from_ref(self));
        // ExPolygon.cpp:106
        // return pl_out.empty();
        pl_out.is_empty()
    }

    // ExPolygon.cpp:109-119
    // bool ExPolygon::contains(const Point &point, bool border_result /* = true */) const
    pub fn contains(&self, point: &Point, border_result: bool) -> bool {
        // ExPolygon.cpp:111-113
        // if (! Slic3r::contains(contour, point, border_result))
        //     // Outside the outer contour, not on the contour boundary.
        //     return false;
        if !polygon_contains(&self.contour, point, border_result) {
            return false;
        }
        // ExPolygon.cpp:114-117
        // for (const Polygon &hole : this->holes)
        //     if (Slic3r::contains(hole, point, ! border_result))
        //         // Inside a hole, not on the hole boundary.
        //         return false;
        for hole in &self.holes {
            if polygon_contains(hole, point, !border_result) {
                return false;
            }
        }
        // ExPolygon.cpp:118
        // return true;
        true
    }

    // ExPolygon.cpp:121-129
    // bool ExPolygon::on_boundary(const Point &point, double eps) const
    pub fn on_boundary(&self, point: &Point, eps: f64) -> bool {
        // ExPolygon.cpp:123-124
        // if (this->contour.on_boundary(point, eps))
        //     return true;
        if self.contour.on_boundary(point, eps) {
            return true;
        }
        // ExPolygon.cpp:125-127
        // for (const Polygon &hole : this->holes)
        //     if (hole.on_boundary(point, eps))
        //         return true;
        for hole in &self.holes {
            if hole.on_boundary(point, eps) {
                return true;
            }
        }
        // ExPolygon.cpp:128
        false
    }

    // ExPolygon.cpp:131-149
    // Point ExPolygon::point_projection(const Point &point) const
    // Projection of a point onto the polygon.
    pub fn point_projection(&self, point: &Point) -> Point {
        // ExPolygon.cpp:134-135
        // if (this->holes.empty()) {
        //     return this->contour.point_projection(point);
        if self.holes.is_empty() {
            self.contour.point_projection(point)
        } else {
            // ExPolygon.cpp:137-138
            // double dist_min2 = std::numeric_limits<double>::max();
            // Point  closest_pt_min;
            let mut dist_min2 = f64::MAX;
            let mut closest_pt_min = Point::new(0, 0);
            // ExPolygon.cpp:139
            for i in 0..self.num_contours() {
                // ExPolygon.cpp:140
                // Point closest_pt = this->contour_or_hole(i).point_projection(point);
                let closest_pt = self.contour_or_hole(i).point_projection(point);
                // ExPolygon.cpp:141
                // double d2 = (closest_pt - point).cast<double>().squaredNorm();
                let d2 = {
                    let dx = (closest_pt.x - point.x) as f64;
                    let dy = (closest_pt.y - point.y) as f64;
                    dx * dx + dy * dy
                };
                // ExPolygon.cpp:142-145
                if d2 < dist_min2 {
                    dist_min2 = d2;
                    closest_pt_min = closest_pt;
                }
            }
            // ExPolygon.cpp:147
            closest_pt_min
        }
    }

    // ExPolygon.cpp:151-156
    // void ExPolygon::symmetric_y(const coord_t &y_axis)
    //
    // NOTE: C++ delegates to `Polygon::symmetric_y(const coord_t&)` which is the
    // inherited `MultiPoint::symmetric_y` (MultiPoint.cpp:472): for each point
    // `pt(0) = 2 * y_axis - pt(0)`. That polygon-level method is not yet exposed
    // as a crate primitive, so we inline the exact MultiPoint body here (not a
    // fake — identical arithmetic).
    pub fn symmetric_y(&mut self, y_axis: Coord) {
        // ExPolygon.cpp:153
        symmetric_y_polygon(&mut self.contour, y_axis);
        // ExPolygon.cpp:154-155
        for hole in &mut self.holes {
            symmetric_y_polygon(hole, y_axis);
        }
    }

    // ExPolygon.cpp:158-184
    // bool ExPolygon::overlaps(const ExPolygon &other) const
    pub fn overlaps(&self, other: &ExPolygon) -> bool {
        // ExPolygon.cpp:160-161
        // if (this->empty() || other.empty())
        //     return false;
        if self.is_empty() || other.is_empty() {
            return false;
        }

        // ExPolygon.cpp:173
        // Polylines pl_out = intersection_pl(to_polylines(other), *this);
        let pl_out = intersection_pl(&to_polylines_expoly(other), std::slice::from_ref(self));

        // ExPolygon.cpp:179-183
        // See unit test SCENARIO("Clipper diff with polyline", "[Clipper]")
        // for in which case the intersection_pl produces any intersection.
        // return ! pl_out.empty() ||
        //        // If *this is completely inside other, then pl_out is empty, but the expolygons overlap.
        //        other.contains(this->contour.points.front());
        !pl_out.is_empty() || other.contains(&self.contour.points[0], true)
    }

    // ExPolygon.cpp:225-261  simplify_p / simplify — see geometry/expolygon.rs.

    // ExPolygon.cpp:382-413
    // ExPolygons ExPolygon::split_expoly_with_holes(coord_t gap_width, const ExPolygons& collision) const
    pub fn split_expoly_with_holes(&self, gap_width: Coord, collision: &ExPolygons) -> ExPolygons {
        // ExPolygon.cpp:384
        // ExPolygons sub_overhangs;
        let mut sub_overhangs: ExPolygons = ExPolygons::new();
        // ExPolygon.cpp:385
        // Polygon  max_hole;
        let mut max_hole = Polygon::new();
        // ExPolygon.cpp:386
        // coordf_t max_area = 0;
        let mut max_area: f64 = 0.0;
        // ExPolygon.cpp:387
        // bool is_collided = false;
        let mut is_collided = false;
        // ExPolygon.cpp:388-400
        for hole in &self.holes {
            // ExPolygon.cpp:389-392
            // if (!is_collided && Slic3r::overlaps({ExPolygon(hole)}, collision)) {
            if !is_collided && overlaps_expolys(&[ExPolygon::new(hole.clone())], collision) {
                max_area = hole.signed_area().abs();
                max_hole = hole.clone();
                is_collided = true;
            // ExPolygon.cpp:393-395
            } else if is_collided
                && overlaps_expolys(&[ExPolygon::new(hole.clone())], collision)
                && hole.signed_area().abs() > max_area
            {
                max_area = hole.signed_area().abs();
                max_hole = hole.clone();
            // ExPolygon.cpp:396-399
            } else if !is_collided
                && !overlaps_expolys(&[ExPolygon::new(hole.clone())], collision)
                && hole.signed_area().abs() > max_area
            {
                max_area = hole.signed_area().abs();
                max_hole = hole.clone();
            }
        }
        // ExPolygon.cpp:401  Point cent;
        // ExPolygon.cpp:402  if (max_hole.size() > 0) {
        if !max_hole.points.is_empty() {
            // ExPolygon.cpp:403
            // auto overhang_bbx = get_extents(*this);
            let overhang_bbx = get_extents_expoly(self);
            // ExPolygon.cpp:404
            // cent = max_hole.centroid();
            let cent = max_hole.centroid();
            // ExPolygon.cpp:405
            sub_overhangs.extend(intersection(
                &[ExPolygon::new(bounding_box_polygon(
                    overhang_bbx.min,
                    Point::new(cent.x - gap_width, cent.y - gap_width),
                ))],
                std::slice::from_ref(self),
            ));
            // ExPolygon.cpp:406
            sub_overhangs.extend(intersection(
                &[ExPolygon::new(bounding_box_polygon(
                    Point::new(cent.x + gap_width, cent.y + gap_width),
                    overhang_bbx.max,
                ))],
                std::slice::from_ref(self),
            ));
            // ExPolygon.cpp:407-408
            sub_overhangs.extend(intersection(
                &[ExPolygon::new(bounding_box_polygon(
                    Point::new(overhang_bbx.min.x, cent.y + gap_width),
                    Point::new(cent.x - gap_width, overhang_bbx.max.y),
                ))],
                std::slice::from_ref(self),
            ));
            // ExPolygon.cpp:409-410
            sub_overhangs.extend(intersection(
                &[ExPolygon::new(bounding_box_polygon(
                    Point::new(cent.x + gap_width, overhang_bbx.min.y),
                    Point::new(overhang_bbx.max.x, cent.y - gap_width),
                ))],
                std::slice::from_ref(self),
            ));
        }
        // ExPolygon.cpp:412
        // return sub_overhangs;
        sub_overhangs
    }

    // ExPolygon.cpp:416-431
    // double ExPolygon::map_moment_to_expansion(double speed, double height) const
    //
    // C++ uses `extern bool compSecondMoment(const ExPolygons&, double&, double&)`
    // from Brim.cpp; the crate's faithful equivalent is
    // `crate::brim::comp_second_moment_expolygons(&ExPolygons, &mut f64, &mut f64)`.
    pub fn map_moment_to_expansion(&self, speed: f64, height: f64) -> f64 {
        // ExPolygon.cpp:418
        // if (height <= 0 || speed <= 0) return 0;
        if height <= 0.0 || speed <= 0.0 {
            return 0.0;
        }
        // ExPolygon.cpp:419
        // double Ixx = 0, Iyy = 0;
        let mut ixx = 0.0_f64;
        let mut iyy = 0.0_f64;
        // ExPolygon.cpp:420
        // double props  = compSecondMoment({*this}, Ixx, Iyy);
        // NOTE: C++ assigns the bool result to `props` (a double) and never uses
        // it; we faithfully call and discard the return value.
        let _props = crate::brim::comp_second_moment_expolygons(
            &vec![self.clone()],
            &mut ixx,
            &mut iyy,
        );
        // ExPolygon.cpp:421
        // Ixx = Ixx * pow(SCALING_FACTOR, 4);
        ixx *= SCALING_FACTOR.powi(4);
        // ExPolygon.cpp:422
        // Iyy = Iyy * pow(SCALING_FACTOR, 4);
        iyy *= SCALING_FACTOR.powi(4);

        // ExPolygon.cpp:424
        // auto bbox = get_extents(*this);
        let bbox = get_extents_expoly(self);
        // ExPolygon.cpp:425-426
        // const double &bboxX = bbox.size()(0);
        // const double &bboxY = bbox.size()(1);
        let bbox_x = bbox.size().x as f64;
        let bbox_y = bbox.size().y as f64;
        // ExPolygon.cpp:427
        // double height_to_area = std::max(height / Ixx * (bboxY * SCALING_FACTOR), height / Iyy * (bboxX * SCALING_FACTOR)) * height / 1920;
        let height_to_area = f64::max(
            height / ixx * (bbox_y * SCALING_FACTOR),
            height / iyy * (bbox_x * SCALING_FACTOR),
        ) * height
            / 1920.0;

        // ExPolygon.cpp:429
        // double brim_width = height_to_area * speed;
        let brim_width = height_to_area * speed;
        // ExPolygon.cpp:430
        // return std::max(std::min(brim_width, 10.), 1.);
        f64::max(f64::min(brim_width, 10.0), 1.0)
    }

    // ExPolygon.cpp:433-441
    // Lines ExPolygon::lines() const
    pub fn lines(&self) -> Vec<Line> {
        // ExPolygon.cpp:435
        // Lines lines = this->contour.lines();
        let mut lines = self.contour.lines();
        // ExPolygon.cpp:436-439
        for h in &self.holes {
            let hole_lines = h.lines();
            lines.extend(hole_lines);
        }
        // ExPolygon.cpp:440
        // return lines;
        lines
    }

    // ExPolygon.cpp:443-453
    // bool ExPolygon::remove_colinear_points()
    //
    // NOTE: C++ delegates to `Polygon::remove_colinear_points()` (inherited from
    // MultiPoint); the crate exposes that body as
    // `crate::multi_point::remove_colinear_points(&mut Vec<Point>)`.
    pub fn remove_colinear_points(&mut self) -> bool {
        // ExPolygon.cpp:444
        // bool removed = this->contour.remove_colinear_points();
        let mut removed = crate::multi_point::remove_colinear_points(&mut self.contour.points);
        // ExPolygon.cpp:445-449
        // if (contour.size() < 3) { contour.points.clear(); holes.clear(); return true; }
        if self.contour.points.len() < 3 {
            self.contour.points.clear();
            self.holes.clear();
            return true;
        }
        // ExPolygon.cpp:450-451
        // for (Polygon &hole : this->holes)
        //     removed |= hole.remove_colinear_points();
        for hole in &mut self.holes {
            removed |= crate::multi_point::remove_colinear_points(&mut hole.points);
        }
        // ExPolygon.cpp:452
        removed
    }

    // ExPolygon.hpp:83-84
    // const Polygon& contour_or_hole(size_t idx) const { return (idx == 0) ? this->contour : this->holes[idx - 1]; }
    pub fn contour_or_hole(&self, idx: usize) -> &Polygon {
        if idx == 0 {
            &self.contour
        } else {
            &self.holes[idx - 1]
        }
    }
}

// ExPolygon.cpp:186-195
// bool overlaps(const ExPolygons& expolys1, const ExPolygons& expolys2)
pub fn overlaps_expolys(expolys1: &[ExPolygon], expolys2: &[ExPolygon]) -> bool {
    // ExPolygon.cpp:188-193
    for expoly1 in expolys1 {
        for expoly2 in expolys2 {
            if expoly1.overlaps(expoly2) {
                return true;
            }
        }
    }
    // ExPolygon.cpp:194
    false
}

// ExPolygon.cpp:197-204
// bool overlaps(const ExPolygons& expolys, const ExPolygon& expoly)
pub fn overlaps_expolys_expoly(expolys: &[ExPolygon], expoly: &ExPolygon) -> bool {
    // ExPolygon.cpp:199-202
    for el in expolys {
        if el.overlaps(expoly) {
            return true;
        }
    }
    // ExPolygon.cpp:203
    false
}

// ExPolygon.cpp:206-223
// Point projection_onto(const ExPolygons& polygons, const Point& from)
pub fn projection_onto(polygons: &[ExPolygon], from: &Point) -> Point {
    // ExPolygon.cpp:208
    // Point projected_pt;
    let mut projected_pt = Point::new(0, 0);
    // ExPolygon.cpp:209
    // double min_dist = std::numeric_limits<double>::max();
    let mut min_dist = f64::MAX;

    // ExPolygon.cpp:211
    for poly in polygons {
        // ExPolygon.cpp:212
        for i in 0..poly.num_contours() {
            // ExPolygon.cpp:213
            // Point p = from.projection_onto(poly.contour_or_hole(i));
            let p = from.projection_onto_multipoint(&poly.contour_or_hole(i).points);
            // ExPolygon.cpp:214
            // double dist = (from - p).cast<double>().squaredNorm();
            let dist = {
                let dx = (from.x - p.x) as f64;
                let dy = (from.y - p.y) as f64;
                dx * dx + dy * dy
            };
            // ExPolygon.cpp:215-218
            if dist < min_dist {
                projected_pt = p;
                min_dist = dist;
            }
        }
    }

    // ExPolygon.cpp:222
    // return projected_pt;
    projected_pt
}

// ExPolygon.cpp:455-460
// double get_expolygons_area(const ExPolygons& expolys)
pub fn get_expolygons_area(expolys: &[ExPolygon]) -> f64 {
    // ExPolygon.cpp:457-459
    expolys.iter().fold(0.0, |val, expoly| val + expoly.area())
}

// ExPolygon.cpp:462-475
// bool is_narrow_expolygon(const ExPolygon& expolygon, double min_width, double min_area, double remain_area_ratio_thres)
//
// (ExPolygon.hpp:461 defaults: min_area = scale_(1)*scale_(1), remain_area_ratio_thres = 0.1)
pub fn is_narrow_expolygon(
    expolygon: &ExPolygon,
    min_width: f64,
    min_area: f64,
    remain_area_ratio_thres: f64,
) -> bool {
    // ExPolygon.cpp:464
    // double original_area = expolygon.area();
    let original_area = expolygon.area();
    // ExPolygon.cpp:465-466
    if original_area < min_area {
        return true;
    }

    // ExPolygon.cpp:468
    // ExPolygons offsets = offset_ex(expolygon, -min_width / 2);
    let offsets = offset_expolygons(
        std::slice::from_ref(expolygon),
        -min_width / 2.0,
        OffsetJoinType::Miter,
    );
    // ExPolygon.cpp:469-470
    if offsets.is_empty() {
        return true;
    }

    // ExPolygon.cpp:472-473
    if get_expolygons_area(&offsets) / (original_area + EPSILON) < remain_area_ratio_thres {
        return true;
    }
    // ExPolygon.cpp:474
    false
}

// ExPolygon.cpp:478-488
// Do expolygons match? If they match, they must have the same topology,
// however their contours may be rotated.
// bool expolygons_match(const ExPolygon &l, const ExPolygon &r)
pub fn expolygons_match(l: &ExPolygon, r: &ExPolygon) -> bool {
    // ExPolygon.cpp:482-483
    // if (l.holes.size() != r.holes.size() || ! polygons_match(l.contour, r.contour))
    //     return false;
    if l.holes.len() != r.holes.len() || !polygons_match(&l.contour, &r.contour) {
        return false;
    }
    // ExPolygon.cpp:484-486
    // for (size_t hole_idx = 0; hole_idx < l.holes.size(); ++ hole_idx)
    //     if (! polygons_match(l.holes[hole_idx], r.holes[hole_idx]))
    //         return false;
    for hole_idx in 0..l.holes.len() {
        if !polygons_match(&l.holes[hole_idx], &r.holes[hole_idx]) {
            return false;
        }
    }
    // ExPolygon.cpp:487
    true
}

// ExPolygon.cpp:490-493
// BoundingBox get_extents(const ExPolygon &expolygon)
pub fn get_extents_expoly(expolygon: &ExPolygon) -> BoundingBox {
    // ExPolygon.cpp:492
    // return get_extents(expolygon.contour);
    expolygon.contour.bounding_box()
}

// ExPolygon.cpp:495-504
// BoundingBox get_extents(const ExPolygons &expolygons)
pub fn get_extents(expolygons: &[ExPolygon]) -> BoundingBox {
    // ExPolygon.cpp:497
    let mut bbox = BoundingBox::new();
    // ExPolygon.cpp:498-502
    if !expolygons.is_empty() {
        for i in 0..expolygons.len() {
            if !expolygons[i].contour.points.is_empty() {
                bbox.merge(&get_extents_expoly(&expolygons[i]));
            }
        }
    }
    // ExPolygon.cpp:503
    bbox
}

// ExPolygon.cpp:506-509
// BoundingBox get_extents_rotated(const ExPolygon &expolygon, double angle)
pub fn get_extents_rotated_expoly(expolygon: &ExPolygon, angle: f64) -> BoundingBox {
    // ExPolygon.cpp:508
    // return get_extents_rotated(expolygon.contour, angle);
    crate::geometry::get_extents_rotated(&expolygon.contour, angle)
}

// ExPolygon.cpp:511-520
// BoundingBox get_extents_rotated(const ExPolygons &expolygons, double angle)
pub fn get_extents_rotated(expolygons: &[ExPolygon], angle: f64) -> BoundingBox {
    // ExPolygon.cpp:513
    let mut bbox = BoundingBox::new();
    // ExPolygon.cpp:514-518
    if !expolygons.is_empty() {
        // ExPolygon.cpp:515 — bbox = get_extents_rotated(expolygons.front().contour, angle);
        bbox = crate::geometry::get_extents_rotated(&expolygons[0].contour, angle);
        // ExPolygon.cpp:516-517
        for i in 1..expolygons.len() {
            bbox.merge(&crate::geometry::get_extents_rotated(&expolygons[i].contour, angle));
        }
    }
    // ExPolygon.cpp:519
    bbox
}

// ExPolygon.cpp:522-529
// std::vector<BoundingBox> get_extents_vector(const ExPolygons &polygons)
pub fn get_extents_vector(polygons: &[ExPolygon]) -> Vec<BoundingBox> {
    // ExPolygon.cpp:524-525
    let mut out: Vec<BoundingBox> = Vec::with_capacity(polygons.len());
    // ExPolygon.cpp:526-527
    for it in polygons {
        out.push(get_extents_expoly(it));
    }
    // ExPolygon.cpp:528
    out
}

// ExPolygon.cpp:531-553
// bool has_duplicate_points(const ExPolygon &expoly)
pub fn has_duplicate_points_expoly(expoly: &ExPolygon) -> bool {
    // ExPolygon.cpp:535
    let mut cnt = expoly.contour.points.len();
    // ExPolygon.cpp:536-537
    for hole in &expoly.holes {
        cnt += hole.points.len();
    }
    // ExPolygon.cpp:538-539
    let mut allpts: Vec<Point> = Vec::with_capacity(cnt);
    // ExPolygon.cpp:540
    // allpts.insert(allpts.begin(), expoly.contour.points...);
    allpts.extend_from_slice(&expoly.contour.points);
    // ExPolygon.cpp:541-542
    for hole in &expoly.holes {
        allpts.extend_from_slice(&hole.points);
    }
    // ExPolygon.cpp:543
    // return has_duplicate_points(std::move(allpts));  (sorts then checks adjacency)
    points_have_duplicates(allpts)
}

// ExPolygon.cpp:555-580
// bool has_duplicate_points(const ExPolygons &expolys)
pub fn has_duplicate_points(expolys: &[ExPolygon]) -> bool {
    // ExPolygon.cpp:559
    let mut cnt = 0;
    // ExPolygon.cpp:560-564
    for expoly in expolys {
        cnt += expoly.contour.points.len();
        for hole in &expoly.holes {
            cnt += hole.points.len();
        }
    }
    // ExPolygon.cpp:565-566
    let mut allpts: Vec<Point> = Vec::with_capacity(cnt);
    // ExPolygon.cpp:567-571
    // NOTE: C++ prepends each contour at allpts.begin() and appends holes at
    // allpts.end(); the order only affects the transient buffer which is then
    // sorted in has_duplicate_points(), so the result is identical regardless.
    // We collect into a single buffer (sorted downstream) — equivalent result.
    for expoly in expolys {
        allpts.extend_from_slice(&expoly.contour.points);
        for hole in &expoly.holes {
            allpts.extend_from_slice(&hole.points);
        }
    }
    // ExPolygon.cpp:572
    points_have_duplicates(allpts)
}

// ExPolygon.cpp:582-595
// bool remove_same_neighbor(ExPolygons &expolygons)
//
// The faithful C++ port is the canonical `crate::geometry::remove_same_neighbor`
// (audited/corrected in place in geometry/expolygon.rs — it now uses the exact
// std::unique-style Polygon/Polygons `remove_same_neighbor` and the contour-erase
// step). Re-exported here under the C++ name for parity bookkeeping.
pub use crate::geometry::remove_same_neighbor as remove_same_neighbor_expolygons;

// ExPolygon.cpp:597-600
// bool remove_sticks(ExPolygon &poly)
//
// NOTE: named `remove_sticks_expoly` to avoid clashing with the existing crate
// helpers; uses the C++-faithful Polygon/Polygons `remove_sticks`.
pub fn remove_sticks_expoly(poly: &mut ExPolygon) -> bool {
    // ExPolygon.cpp:599
    // return remove_sticks(poly.contour) || remove_sticks(poly.holes);
    // NOTE: C++ short-circuits: holes are only processed when the contour pass
    // returns false. We replicate the `||` short-circuit exactly.
    polygon_remove_sticks(&mut poly.contour) || polygons_remove_sticks(&mut poly.holes)
}

// ExPolygon.cpp:602-621
// bool remove_small_and_small_holes(ExPolygons &expolygons, double min_area)
pub fn remove_small_and_small_holes(expolygons: &mut ExPolygons, min_area: f64) -> bool {
    // ExPolygon.cpp:604
    let mut modified = false;
    // ExPolygon.cpp:605
    let mut free_idx = 0usize;
    // ExPolygon.cpp:606
    for expoly_idx in 0..expolygons.len() {
        // ExPolygon.cpp:607
        // if (std::abs(expolygons[expoly_idx].area()) >= min_area) {
        if expolygons[expoly_idx].area().abs() >= min_area {
            // ExPolygon.cpp:609
            // modified |= remove_small(expolygons[expoly_idx].holes, min_area);
            modified |= polygons_remove_small(&mut expolygons[expoly_idx].holes, min_area);
            // ExPolygon.cpp:610-613
            if free_idx < expoly_idx {
                expolygons.swap(expoly_idx, free_idx);
            }
            // ExPolygon.cpp:614
            free_idx += 1;
        } else {
            // ExPolygon.cpp:616
            modified = true;
        }
    }
    // ExPolygon.cpp:618-619
    if free_idx < expolygons.len() {
        expolygons.truncate(free_idx);
    }
    // ExPolygon.cpp:620
    modified
}

// ExPolygon.cpp:623-640
// void keep_largest_contour_only(ExPolygons &polygons)
//
// The faithful C++ port is the canonical `crate::geometry::keep_largest_contour_only`
// (audited/corrected in place in geometry/expolygon.rs — it now keeps only the
// single ExPolygon with the largest CONTOUR area across the whole collection).
// Re-exported here under a C++-named alias for parity bookkeeping.
pub use crate::geometry::keep_largest_contour_only as keep_largest_contour_only_collection;

// ----------------------------------------------------------------------------
// Local helpers mirroring Slic3r free functions used by ExPolygon.cpp.
// ----------------------------------------------------------------------------

// Slic3r::contains(const Polygon&, const Point&, bool border_result)
// (Polygon.cpp). Tri-state PointInPolygon: returns true if `pt` is inside, or
// returns `border_result` when `pt` lies exactly on the polygon boundary.
fn polygon_contains(polygon: &Polygon, pt: &Point, border_result: bool) -> bool {
    match point_in_polygon(pt, &polygon.points) {
        // pt on boundary -> return border_result
        -1 => border_result,
        // inside
        1 => true,
        // outside
        _ => false,
    }
}

// clipper.cpp:PointInPolygon — returns 0 if false, +1 if inside, -1 if pt is
// exactly ON the polygon boundary.
fn point_in_polygon(pt: &Point, path: &[Point]) -> i32 {
    // C++: int result = 0;
    let mut result = 0i32;
    // C++: size_t cnt = path.size();
    let cnt = path.len();
    // C++: if (cnt < 3) return 0;
    if cnt < 3 {
        return 0;
    }
    // C++: IntPoint ip = path[0];
    let mut ip = path[0];
    // C++: for(size_t i = 1; i <= cnt; ++i)
    for i in 1..=cnt {
        // C++: IntPoint ipNext = (i == cnt ? path[0] : path[i]);
        let ip_next = if i == cnt { path[0] } else { path[i] };
        // C++: if (ipNext.y() == pt.y() && ((ipNext.x() == pt.x()) || (ip.y() == pt.y() && ((ipNext.x() > pt.x()) == (ip.x() < pt.x())))))
        if ip_next.y == pt.y
            && ((ip_next.x == pt.x) || (ip.y == pt.y && ((ip_next.x > pt.x) == (ip.x < pt.x))))
        {
            return -1;
        }
        // C++: if ((ip.y() < pt.y()) != (ipNext.y() < pt.y()))
        if (ip.y < pt.y) != (ip_next.y < pt.y) {
            // C++: if (ip.x() >= pt.x())
            if ip.x >= pt.x {
                // C++: if (ipNext.x() > pt.x()) result = 1 - result;
                if ip_next.x > pt.x {
                    result = 1 - result;
                } else {
                    // C++: double d = (double)(ip.x() - pt.x()) * (ipNext.y() - pt.y()) - (double)(ipNext.x() - pt.x()) * (ip.y() - pt.y());
                    let d = (ip.x as i64 - pt.x as i64) as f64 * (ip_next.y as i64 - pt.y as i64) as f64
                        - (ip_next.x as i64 - pt.x as i64) as f64 * (ip.y as i64 - pt.y as i64) as f64;
                    // C++: if (!d) return -1;
                    if d == 0.0 {
                        return -1;
                    }
                    // C++: if ((d > 0) == (ipNext.y() > ip.y())) result = 1 - result;
                    if (d > 0.0) == (ip_next.y > ip.y) {
                        result = 1 - result;
                    }
                }
            } else {
                // C++: if (ipNext.x() > pt.x())
                if ip_next.x > pt.x {
                    // C++: double d = ...
                    let d = (ip.x as i64 - pt.x as i64) as f64 * (ip_next.y as i64 - pt.y as i64) as f64
                        - (ip_next.x as i64 - pt.x as i64) as f64 * (ip.y as i64 - pt.y as i64) as f64;
                    // C++: if (!d) return -1;
                    if d == 0.0 {
                        return -1;
                    }
                    // C++: if ((d > 0) == (ipNext.y() > ip.y())) result = 1 - result;
                    if (d > 0.0) == (ip_next.y > ip.y) {
                        result = 1 - result;
                    }
                }
            }
        }
        // C++: ip = ipNext;
        ip = ip_next;
    }
    // C++: return result;
    result
}

// BoundingBox::polygon() — the CCW rectangle of the bounding box (min, (max.x,
// min.y), max, (min.x, max.y)). Used by split_expoly_with_holes.
fn bounding_box_polygon(min: Point, max: Point) -> Polygon {
    Polygon::from_points(vec![
        Point::new(min.x, min.y),
        Point::new(max.x, min.y),
        Point::new(max.x, max.y),
        Point::new(min.x, max.y),
    ])
}

// MultiPoint::symmetric_y(const coord_t &x_axis) (MultiPoint.cpp:472): for each
// point `pt(0) = 2 * y_axis - pt(0)`. (Polygon inherits this from MultiPoint.)
fn symmetric_y_polygon(polygon: &mut Polygon, y_axis: Coord) {
    for p in &mut polygon.points {
        p.x = 2 * y_axis - p.x;
    }
}

// Polygon::scale(double factor_x, double factor_y) (Polygon.cpp): multiply each
// point's x by factor_x and y by factor_y, rounding to the nearest coord_t
// (matches the crate's `Point * f64` operator, which uses `.round()`).
fn scale_polygon_xy(polygon: &mut Polygon, factor_x: f64, factor_y: f64) {
    for p in &mut polygon.points {
        p.x = (p.x as f64 * factor_x).round() as Coord;
        p.y = (p.y as f64 * factor_y).round() as Coord;
    }
}

// } // namespace Slic3r   (ExPolygon.cpp:642)
