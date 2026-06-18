//! Polyline type for open paths.
//!
//! 1:1 port of BambuStudio `src/libslic3r/Polyline.cpp` / `Polyline.hpp`.
//! coord_t -> i64 (Coord), coordf_t -> f64 (CoordF).
//!
//! NOTE: In addition to the faithful C++ `Polyline.cpp` translation below, this
//! module retains a handful of Rust-only convenience helpers (closest_point,
//! direction_at, translate/scale/rotate, etc.) that are consumed elsewhere in
//! the crate. These are clearly marked as Rust-only and are not part of the C++
//! `Polyline.cpp` surface.

use super::{BoundingBox, Line, Point, Polygon};
use crate::arc_fitter::{ArcFitter, EMovePathType, PathFittingData};
use crate::circle::ArcSegment;
use crate::{Coord, CoordF};
use serde::{Deserialize, Serialize};
use std::fmt;
use std::ops::{Deref, DerefMut, Index, IndexMut};

/// An open polyline defined by a sequence of points.
///
/// Polyline.hpp:19 `class Polyline : public MultiPoint`
/// MultiPoint.hpp: `Points points;`
#[derive(Clone, Default, Serialize, Deserialize)]
pub struct Polyline {
    /// Points defining the polyline (public to match C++ MultiPoint).
    /// MultiPoint.hpp: `Points points;`
    pub points: Vec<Point>,
    /// BBS: store arc fitting result.
    /// Polyline.hpp:130 `std::vector<PathFittingData> fitting_result;`
    /// `PathFittingData` is not (de)serializable; it is reconstructed by arc
    /// fitting, so it is skipped for serde and defaults to empty.
    #[serde(skip)]
    pub fitting_result: Vec<PathFittingData>,
}

// Polyline.hpp:144 `inline bool operator==(const Polyline &lhs, const Polyline &rhs) { return lhs.points == rhs.points; }`
// C++ equality compares ONLY points (not fitting_result).
impl PartialEq for Polyline {
    fn eq(&self, other: &Self) -> bool {
        self.points == other.points
    }
}

impl Polyline {
    /// Polyline.hpp:21 `Polyline() {};`
    #[inline]
    pub fn new() -> Self {
        Self {
            points: Vec::new(),
            fitting_result: Vec::new(),
        }
    }

    /// Polyline.hpp:33 `explicit Polyline(const Points &points) : MultiPoint(points) { fitting_result.clear(); }`
    #[inline]
    pub fn from_points(points: Vec<Point>) -> Self {
        Self {
            points,
            fitting_result: Vec::new(),
        }
    }

    /// Create a polyline from a polygon (closes the polygon by repeating the first point).
    /// Rust-only convenience (not in Polyline.cpp).
    #[inline]
    pub fn from_polygon(polygon: &Polygon) -> Self {
        let mut points = polygon.points().to_vec();
        if !points.is_empty() && points.first() != points.last() {
            points.push(points[0]);
        }
        Self::from_points(points)
    }

    /// Polyline.hpp:49 `static Polyline new_scale(const std::vector<Vec2d> &points)`
    pub fn new_scale(points: &[super::Vec2d]) -> Polyline {
        // Polyline.hpp:50-51
        let mut pl = Polyline::new();
        pl.points.reserve(points.len());
        // Polyline.hpp:52-53
        for pt in points {
            pl.points.push(Point::new_scale(pt.x, pt.y));
        }
        // Polyline.hpp:54-55 BBS: new_scale doesn't support arc, so clean
        pl.fitting_result.clear();
        // Polyline.hpp:56
        pl
    }

    /// Create a polyline with the given capacity.
    /// Rust-only convenience.
    #[inline]
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            points: Vec::with_capacity(capacity),
            fitting_result: Vec::new(),
        }
    }

    /// Rust-only accessor.
    #[inline]
    pub fn points(&self) -> &[Point] {
        &self.points
    }

    /// Rust-only accessor.
    #[inline]
    pub fn points_mut(&mut self) -> &mut Vec<Point> {
        &mut self.points
    }

    /// Rust-only: consume the polyline and return its points.
    #[inline]
    pub fn into_points(self) -> Vec<Point> {
        self.points
    }

    /// MultiPoint::size().
    #[inline]
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// MultiPoint::size().
    #[inline]
    pub fn size(&self) -> usize {
        self.points.len()
    }

    /// MultiPoint::empty().
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// MultiPoint::empty().
    #[inline]
    pub fn empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Rust-only convenience.
    #[inline]
    pub fn push(&mut self, point: Point) {
        self.points.push(point);
    }

    /// Rust-only convenience.
    #[inline]
    pub fn pop(&mut self) -> Option<Point> {
        self.points.pop()
    }

    /// Polyline.hpp:114 `void clear() { MultiPoint::clear(); this->fitting_result.clear(); }`
    #[inline]
    pub fn clear(&mut self) {
        self.points.clear();
        self.fitting_result.clear();
    }

    /// Rust-only convenience.
    #[inline]
    pub fn reserve(&mut self, additional: usize) {
        self.points.reserve(additional);
    }

    /// Rust-only convenience.
    #[inline]
    pub fn first(&self) -> Option<&Point> {
        self.points.first()
    }

    /// Rust-only convenience.
    #[inline]
    pub fn last(&self) -> Option<&Point> {
        self.points.last()
    }

    /// MultiPoint::first_point().
    #[inline]
    pub fn first_point(&self) -> Point {
        self.points[0]
    }

    /// Polyline.hpp:110 `const Point& last_point() const override { return this->points.back(); }`
    #[inline]
    pub fn last_point(&self) -> Point {
        self.points[self.points.len() - 1]
    }

    /// MultiPoint::is_valid() — `points.size() >= 2`.
    #[inline]
    pub fn is_valid(&self) -> bool {
        self.points.len() >= 2
    }

    /// Polyline.hpp:127 `bool is_closed() const { return this->points.front() == this->points.back(); }`
    #[inline]
    pub fn is_closed(&self) -> bool {
        // Guard against empty so callers don't panic; C++ assumes non-empty.
        !self.points.is_empty() && self.points.first() == self.points.last()
    }

    /// MultiPoint::length().
    pub fn length(&self) -> CoordF {
        if self.points.len() < 2 {
            return 0.0;
        }
        let mut total = 0.0;
        for i in 0..(self.points.len() - 1) {
            total += self.points[i].distance(&self.points[i + 1]);
        }
        total
    }

    /// MultiPoint::find_point(const Point&) -> int (index, or -1).
    /// MultiPoint.cpp.
    #[inline]
    pub fn find_point(&self, point: &Point) -> i32 {
        crate::multi_point::find_point(&self.points, point)
    }

    /// Polyline.cpp:12-20 `const Point& Polyline::leftmost_point() const`
    pub fn leftmost_point(&self) -> Point {
        // Polyline.cpp:14
        let mut p = &self.points[0];
        // Polyline.cpp:15-18
        for it in self.points.iter().skip(1) {
            if it.x < p.x {
                p = it;
            }
        }
        // Polyline.cpp:19
        *p
    }

    /// Polyline.cpp:22-32 `Lines Polyline::lines() const`
    pub fn lines(&self) -> Vec<Line> {
        // Polyline.cpp:24
        let mut lines: Vec<Line> = Vec::new();
        // Polyline.cpp:25
        if self.points.len() >= 2 {
            // Polyline.cpp:26
            lines.reserve(self.points.len() - 1);
            // Polyline.cpp:27-29
            for i in 0..(self.points.len() - 1) {
                lines.push(Line::new(self.points[i], self.points[i + 1]));
            }
        }
        // Polyline.cpp:31
        lines
    }

    /// Polyline.cpp:34-49 `void Polyline::reverse()`
    pub fn reverse(&mut self) {
        // Polyline.cpp:37 BBS: reverse points -> MultiPoint::reverse()
        self.points.reverse();
        // Polyline.cpp:38-39 BBS: reverse the fitting_result
        if !self.fitting_result.is_empty() {
            let size = self.points.len();
            // Polyline.cpp:40-46
            for i in 0..self.fitting_result.len() {
                let fr = &mut self.fitting_result[i];
                // Polyline.cpp:41 std::swap(start_point_index, end_point_index)
                std::mem::swap(&mut fr.start_point_index, &mut fr.end_point_index);
                // Polyline.cpp:42 start_point_index = MultiPoint::size() - 1 - start_point_index
                fr.start_point_index = size - 1 - fr.start_point_index;
                // Polyline.cpp:43 end_point_index = MultiPoint::size() - 1 - end_point_index
                fr.end_point_index = size - 1 - fr.end_point_index;
                // Polyline.cpp:44-45
                if fr.is_arc_move() {
                    fr.reverse_arc_path();
                }
            }
            // Polyline.cpp:47
            self.fitting_result.reverse();
        }
    }

    /// Rust-only: return a reversed copy.
    pub fn reversed(&self) -> Self {
        let mut result = self.clone();
        result.reverse();
        result
    }

    /// Polyline.cpp:51-92 `void Polyline::clip_end(double distance)`
    /// removes the given distance from the end of the polyline
    pub fn clip_end(&mut self, mut distance: f64) {
        // Polyline.cpp:54
        let mut last_point_inserted = false;
        // Polyline.cpp:55
        let mut remove_after_index = self.points.len();
        // Polyline.cpp:56
        while distance > 0.0 {
            // Polyline.cpp:57 — cast<double>() is a raw integer-to-double cast.
            let last_point = (self.last_point().x as f64, self.last_point().y as f64);
            // Polyline.cpp:58
            self.points.pop();
            // Polyline.cpp:59
            remove_after_index -= 1;
            // Polyline.cpp:60-63
            if self.points.is_empty() {
                self.fitting_result.clear();
                return;
            }
            // Polyline.cpp:64 — v = last_point() - last_point
            let vx = self.last_point().x as f64 - last_point.0;
            let vy = self.last_point().y as f64 - last_point.1;
            // Polyline.cpp:65 — lsqr = v.squaredNorm()
            let lsqr = vx * vx + vy * vy;
            // Polyline.cpp:66
            if lsqr > distance * distance {
                // Polyline.cpp:67 — (last_point + v * (distance / sqrt(lsqr))).cast<coord_t>()
                // cast<coord_t> truncates toward zero.
                let s = distance / lsqr.sqrt();
                let nx = last_point.0 + vx * s;
                let ny = last_point.1 + vy * s;
                self.points.push(Point::new(nx as Coord, ny as Coord));
                // Polyline.cpp:68
                last_point_inserted = true;
                // Polyline.cpp:69
                break;
            }
            // Polyline.cpp:71
            distance -= lsqr.sqrt();
        }
        // C++ `last_point_inserted` is assigned but unused below; silence warning.
        let _ = last_point_inserted;

        // Polyline.cpp:74-76 BBS: don't need to clip fitting result if it's empty
        if self.fitting_result.is_empty() {
            return;
        }
        // Polyline.cpp:77-78
        while !self.fitting_result.is_empty()
            && self.fitting_result.last().unwrap().start_point_index >= remove_after_index
        {
            self.fitting_result.pop();
        }
        // Polyline.cpp:79
        if !self.fitting_result.is_empty() {
            // Polyline.cpp:80-82 BBS: last remaining segment is arc move, then clip the arc at last point
            let path_type = self.fitting_result.last().unwrap().path_type;
            if path_type == EMovePathType::ArcMoveCcw || path_type == EMovePathType::ArcMoveCw {
                let last_point = self.last_point();
                // Polyline.cpp:83
                if self
                    .fitting_result
                    .last_mut()
                    .unwrap()
                    .arc_data
                    .clip_end(last_point)
                {
                    // Polyline.cpp:84-85 BBS: succeed to clip arc, then update the last point
                    let end_point = self.fitting_result.last().unwrap().arc_data.end_point;
                    *self.points.last_mut().unwrap() = end_point;
                } else {
                    // Polyline.cpp:86-88 BBS: Failed to clip arc, then back to linear move
                    self.fitting_result.last_mut().unwrap().path_type = EMovePathType::LinearMove;
                }
            }
            // Polyline.cpp:90
            self.fitting_result.last_mut().unwrap().end_point_index = self.points.len() - 1;
        }
    }

    /// Polyline.cpp:94-101 `void Polyline::clip_start(double distance)`
    /// removes the given distance from the start of the polyline
    pub fn clip_start(&mut self, distance: f64) {
        // Polyline.cpp:97
        self.reverse();
        // Polyline.cpp:98
        self.clip_end(distance);
        // Polyline.cpp:99-100
        if self.points.len() >= 2 {
            self.reverse();
        }
    }

    /// Polyline.cpp:103-109 `void Polyline::extend_end(double distance)`
    pub fn extend_end(&mut self, distance: f64) {
        // Polyline.cpp:106 BBS: append a new last point by extending the last segment.
        // v = (points.back() - *(points.end() - 2)).cast<double>().normalized()
        let n = self.points.len();
        let back = self.points[n - 1];
        let prev = self.points[n - 2];
        let dx = (back.x - prev.x) as f64;
        let dy = (back.y - prev.y) as f64;
        let norm = (dx * dx + dy * dy).sqrt();
        let vx = dx / norm;
        let vy = dy / norm;
        // Polyline.cpp:107 — new_last_point = points.back() + (v * distance).cast<coord_t>()
        // cast<coord_t> truncates toward zero.
        let new_last_point = Point::new(
            back.x + (vx * distance) as Coord,
            back.y + (vy * distance) as Coord,
        );
        // Polyline.cpp:108
        self.append_point(new_last_point);
    }

    /// Polyline.cpp:111-116 `void Polyline::extend_start(double distance)`
    pub fn extend_start(&mut self, distance: f64) {
        // Polyline.cpp:113
        self.reverse();
        // Polyline.cpp:114
        self.extend_end(distance);
        // Polyline.cpp:115
        self.reverse();
    }

    /// Polyline.cpp:118-144 `Points Polyline::equally_spaced_points(double distance) const`
    /// this method returns a collection of points picked on the polygon contour
    /// so that they are evenly spaced according to the input distance
    pub fn equally_spaced_points(&self, distance: f64) -> Vec<Point> {
        // Polyline.cpp:121
        let mut points: Vec<Point> = Vec::new();
        // Polyline.cpp:123
        points.push(self.first_point());
        // Polyline.cpp:124
        let mut len = 0.0_f64;

        // Polyline.cpp:126
        let mut i = 1;
        while i < self.points.len() {
            // Polyline.cpp:127 — p1 = (it-1).cast<double>()
            let p1x = self.points[i - 1].x as f64;
            let p1y = self.points[i - 1].y as f64;
            // Polyline.cpp:128 — v = it.cast<double>() - p1
            let vx = self.points[i].x as f64 - p1x;
            let vy = self.points[i].y as f64 - p1y;
            // Polyline.cpp:129
            let segment_length = (vx * vx + vy * vy).sqrt();
            // Polyline.cpp:130
            len += segment_length;
            // Polyline.cpp:131-132
            if len < distance {
                i += 1;
                continue;
            }
            // Polyline.cpp:133-137
            if len == distance {
                points.push(self.points[i]);
                len = 0.0;
                i += 1;
                continue;
            }
            // Polyline.cpp:138 — how much we take of this segment
            let take = segment_length - (len - distance);
            // Polyline.cpp:139 — (p1 + v * (take / v.norm())).cast<coord_t>() ; cast truncates.
            let s = take / (vx * vx + vy * vy).sqrt();
            points.push(Point::new(
                (p1x + vx * s) as Coord,
                (p1y + vy * s) as Coord,
            ));
            // Polyline.cpp:140 — --it
            i -= 1;
            // Polyline.cpp:141
            len = -take;
            // Note: i was decremented; the outer loop's ++it (i += 1) brings us
            // back to the same segment, matching C++ for-loop semantics.
            i += 1;
        }
        // Polyline.cpp:143
        points
    }

    /// Polyline.cpp:146-150 `void Polyline::simplify(double tolerance)`
    pub fn simplify(&mut self, tolerance: f64) {
        // Polyline.cpp:148 — points = MultiPoint::_douglas_peucker(points, tolerance)
        self.points = crate::multi_point::douglas_peucker(&self.points, tolerance);
        // Polyline.cpp:149
        self.fitting_result.clear();
    }

    /// Polyline.cpp:152-156 `void Polyline::simplify_by_fitting_arc(double tolerance)`
    pub fn simplify_by_fitting_arc(&mut self, tolerance: f64) {
        // Polyline.cpp:155 BBS: do arc fit first, then use DP simplify on straight part.
        // C++ returns void; ignore the Rust Result.
        let _ = ArcFitter::do_arc_fitting_and_simplify(
            &mut self.points,
            &mut self.fitting_result,
            tolerance,
        );
    }

    /// Polyline.cpp:158-197 `Polylines Polyline::equally_spaced_lines(double distance) const`
    pub fn equally_spaced_lines(&self, distance: f64) -> Polylines {
        // Polyline.cpp:160
        let mut lines: Polylines = Vec::new();
        // Polyline.cpp:161
        let mut line = Polyline::new();
        // Polyline.cpp:162
        line.append_point(self.first_point());
        // Polyline.cpp:163
        let mut len = 0.0_f64;

        // Polyline.cpp:165
        let mut i = 1;
        while i < self.points.len() {
            // Polyline.cpp:166 — p1 = line.points.back().cast<double>()
            let back = *line.points.last().unwrap();
            let p1x = back.x as f64;
            let p1y = back.y as f64;
            // Polyline.cpp:167 — v = it.cast<double>() - p1
            let vx = self.points[i].x as f64 - p1x;
            let vy = self.points[i].y as f64 - p1y;
            // Polyline.cpp:168
            let segment_length = (vx * vx + vy * vy).sqrt();
            // Polyline.cpp:169
            len += segment_length;
            // Polyline.cpp:170-171
            if len < distance {
                i += 1;
                continue;
            }
            // Polyline.cpp:172-180
            if len == distance {
                line.append_point(self.points[i]);
                lines.push(line.clone());

                line.clear();
                line.append_point(self.points[i]);
                len = 0.0;
                i += 1;
                continue;
            }
            // Polyline.cpp:181 — how much we take of this segment
            let take = distance;
            // Polyline.cpp:182 — (p1 + v * (take / v.norm())).cast<coord_t>() ; cast truncates.
            let s = take / (vx * vx + vy * vy).sqrt();
            line.append_point(Point::new(
                (p1x + vx * s) as Coord,
                (p1y + vy * s) as Coord,
            ));
            // Polyline.cpp:183
            lines.push(line.clone());

            // Polyline.cpp:185-186
            line.clear();
            line.append_point(lines.last().unwrap().last_point());
            // Polyline.cpp:187 — --it
            i -= 1;
            // Polyline.cpp:188
            len = -take;
            i += 1;
        }
        // Polyline.cpp:190-195 add the last reminder
        if line.size() == 1 {
            line.append_point(self.last_point());
            if line.first_point() != line.last_point() {
                lines.push(line.clone());
            }
        }
        // Polyline.cpp:196
        lines
    }

    /// Polyline.cpp:223-265 `void Polyline::split_at(Point &point, Polyline* p1, Polyline* p2) const`
    pub fn split_at_point(&self, point: &mut Point, p1: &mut Polyline, p2: &mut Polyline) {
        // Polyline.cpp:225
        if self.points.is_empty() {
            return;
        }

        // Polyline.cpp:227-228 0 judge whether the point is on the polyline
        let mut index = self.find_point(point);
        // Polyline.cpp:229-234
        if index != -1 {
            // BBS: the spilit point is on the polyline, then easy
            self.split_at_index(index as usize, p1, p2);
            *point = if p1.is_valid() {
                p1.last_point()
            } else {
                p2.first_point()
            };
            return;
        }

        // Polyline.cpp:236-237 1 find the line to split at
        let mut line_idx: usize = 0;
        // Polyline.cpp:238
        let mut p = self.first_point();
        // Polyline.cpp:239 — min = (p - point).cast<double>().norm()
        let mut min = {
            let dx = (p.x - point.x) as f64;
            let dy = (p.y - point.y) as f64;
            (dx * dx + dy * dy).sqrt()
        };
        // Polyline.cpp:240
        let lines = self.lines();
        // Polyline.cpp:241-248
        for (li, line) in lines.iter().enumerate() {
            // Polyline.cpp:242 — p_tmp = point.projection_onto(*line)
            let p_tmp = point.project_onto_segment(line.a, line.b);
            // Polyline.cpp:243 — (p_tmp - point).cast<double>().norm() < min
            let dx = (p_tmp.x - point.x) as f64;
            let dy = (p_tmp.y - point.y) as f64;
            let d = (dx * dx + dy * dy).sqrt();
            if d < min {
                // Polyline.cpp:244-246
                p = p_tmp;
                let mdx = (p.x - point.x) as f64;
                let mdy = (p.y - point.y) as f64;
                min = (mdx * mdx + mdy * mdy).sqrt();
                line_idx = li;
            }
        }

        // Polyline.cpp:250-252 2 judge whether the cloest point is one vertex of polyline.
        index = self.find_point(&p);
        // Polyline.cpp:253-264
        if index != -1 {
            // Polyline.cpp:255-257
            self.split_at_index(index as usize, p1, p2);
            p1.append_point(*point);
            p2.append_before(*point);
        } else {
            // Polyline.cpp:259-263
            let mut temp = Polyline::new();
            self.split_at_index(line_idx, p1, &mut temp);
            p1.append_point(*point);
            self.split_at_index(line_idx + 1, &mut temp, p2);
            p2.append_before(*point);
        }
    }

    /// Polyline.cpp:268-298 `bool Polyline::split_at_index(const size_t index, Polyline* p1, Polyline* p2) const`
    pub fn split_at_index(&self, index: usize, p1: &mut Polyline, p2: &mut Polyline) -> bool {
        // Polyline.cpp:270-271
        if index > self.size() - 1 {
            return false;
        }

        // Polyline.cpp:273-276
        if index == 0 {
            p1.clear();
            p1.append_point(self.first_point());
            *p2 = self.clone();
        // Polyline.cpp:277-280
        } else if index == self.size() - 1 {
            p2.clear();
            p2.append_point(self.last_point());
            *p1 = self.clone();
        } else {
            // Polyline.cpp:282-288 BBS: spilit first part
            p1.clear();
            p1.points.reserve(index + 1);
            p1.points
                .extend_from_slice(&self.points[0..index + 1]);
            // Polyline.cpp:286
            let mut new_endpoint = Point::new(0, 0);
            // Polyline.cpp:287-288
            if self.split_fitting_result_before_index(index, &mut new_endpoint, &mut p1.fitting_result)
            {
                *p1.points.last_mut().unwrap() = new_endpoint;
            }

            // Polyline.cpp:290-295
            p2.clear();
            p2.points.reserve(self.size() - index);
            p2.points
                .extend_from_slice(&self.points[index..self.size()]);
            // Polyline.cpp:293
            let mut new_startpoint = Point::new(0, 0);
            // Polyline.cpp:294-295
            if self.split_fitting_result_after_index(index, &mut new_startpoint, &mut p2.fitting_result)
            {
                *p2.points.first_mut().unwrap() = new_startpoint;
            }
        }
        // Polyline.cpp:297
        true
    }

    /// Polyline.cpp:300-344 `bool Polyline::split_at_length(const double length, Polyline *p1, Polyline *p2) const`
    pub fn split_at_length(&self, length: f64, p1: &mut Polyline, p2: &mut Polyline) -> bool {
        // Polyline.cpp:302
        if self.points.is_empty() {
            return false;
        }
        // Polyline.cpp:303
        if length < 0.0 || length > self.length() {
            return false;
        }

        // Polyline.cpp:305-308
        if length < crate::libslic3r::SCALED_EPSILON {
            p1.clear();
            p1.append_point(self.first_point());
            *p2 = self.clone();
        // Polyline.cpp:309-312 — is_approx(length, this->length(), SCALED_EPSILON)
        // libslic3r.h:288 is_approx(a, b, precision) = fabs(a - b) < precision
        } else if (length - self.length()).abs() < crate::libslic3r::SCALED_EPSILON {
            p2.clear();
            p2.append_point(self.last_point());
            *p1 = self.clone();
        } else {
            // Polyline.cpp:314-315 1 find the line to split at
            let mut line_idx: usize = 0;
            // Polyline.cpp:316
            let mut acc_length = 0.0_f64;
            // Polyline.cpp:317
            let mut p = self.first_point();
            // Polyline.cpp:318-328
            for l in self.lines() {
                // Polyline.cpp:319
                p = l.b;
                // Polyline.cpp:321
                let current_length = l.length();
                // Polyline.cpp:322-325
                if acc_length + current_length >= length {
                    // Polyline.cpp:323 — lerp(l.a, l.b, (length - acc_length) / current_length)
                    // Point.hpp:298-302 lerp: ((1-t)*a + t*b).cast<coord_t>() — truncates
                    // toward zero. NOTE: the crate-wide `super::lerp` ROUNDS instead of
                    // truncating, so inline the faithful C++ formula here (matches
                    // line_segmentation.rs which keeps a local truncating lerp too).
                    let t = (length - acc_length) / current_length;
                    p = Point::new(
                        ((1.0 - t) * l.a.x as CoordF + t * l.b.x as CoordF) as Coord,
                        ((1.0 - t) * l.a.y as CoordF + t * l.b.y as CoordF) as Coord,
                    );
                    break;
                }
                // Polyline.cpp:326
                acc_length += current_length;
                // Polyline.cpp:327
                line_idx += 1;
            }

            // Polyline.cpp:330-332 2 judge whether the cloest point is one vertex of polyline.
            let index = self.find_point(&p);
            // Polyline.cpp:333-341
            if index != -1 {
                self.split_at_index(index as usize, p1, p2);
            } else {
                let mut temp = Polyline::new();
                self.split_at_index(line_idx, p1, &mut temp);
                p1.append_point(p);
                self.split_at_index(line_idx + 1, &mut temp, p2);
                p2.append_before(p);
            }
        }
        // Polyline.cpp:343
        true
    }

    /// Polyline.cpp:346-356 `bool Polyline::is_straight() const`
    pub fn is_straight(&self) -> bool {
        // Check that each segment's direction is equal to the line connecting
        // first point and last point. (Checking each line against the previous
        // one would cause the error to accumulate.)
        // Polyline.cpp:351 — dir = Line(first_point(), last_point()).direction()
        let dir = line_direction(self.first_point(), self.last_point());
        // Polyline.cpp:352-354
        for line in self.lines() {
            if !line_parallel_to_angle(&line, dir) {
                return false;
            }
        }
        // Polyline.cpp:355
        true
    }

    /// Polyline.cpp:358-379 `void Polyline::append(const Polyline &src)`
    pub fn append(&mut self, src: &Polyline) {
        // Polyline.cpp:360
        if !src.is_valid() {
            return;
        }

        // Polyline.cpp:362-364
        if self.points.is_empty() {
            self.points = src.points.clone();
            self.fitting_result = src.fitting_result.clone();
        } else {
            // Polyline.cpp:366-367 BBS: append the first point to create connection first.
            self.append_point(src.points[0]);
            // Polyline.cpp:368-373 BBS: append a polyline which has fitting data to a polyline
            // without fitting data. Then create a fake fitting data first.
            if self.fitting_result.is_empty() && !src.fitting_result.is_empty() {
                self.fitting_result.push(PathFittingData::new(
                    0,
                    self.points.len() - 1,
                    EMovePathType::LinearMove,
                    ArcSegment::new(),
                ));
            }
            // Polyline.cpp:374-375 BBS: then append the remain points
            self.multipoint_append_iter(&src.points[1..]);
            // Polyline.cpp:376-377 BBS: finally append the fitting data
            self.append_fitting_result_after_append_polyline(src);
        }
    }

    /// Polyline.cpp:381-404 `void Polyline::append(Polyline &&src)`
    /// Move variant; in Rust we take by value and clear the source is not needed
    /// since ownership is transferred, but we mirror the logic.
    pub fn append_move(&mut self, mut src: Polyline) {
        // Polyline.cpp:383
        if !src.is_valid() {
            return;
        }

        // Polyline.cpp:385-387
        if self.points.is_empty() {
            self.points = std::mem::take(&mut src.points);
            self.fitting_result = std::mem::take(&mut src.fitting_result);
        } else {
            // Polyline.cpp:389-390
            self.append_point(src.points[0]);
            // Polyline.cpp:393-396
            if self.fitting_result.is_empty() && !src.fitting_result.is_empty() {
                self.fitting_result.push(PathFittingData::new(
                    0,
                    self.points.len() - 1,
                    EMovePathType::LinearMove,
                    ArcSegment::new(),
                ));
            }
            // Polyline.cpp:398
            self.multipoint_append_iter(&src.points[1..]);
            // Polyline.cpp:400
            self.append_fitting_result_after_append_polyline(&src);
            // Polyline.cpp:401-402
            src.points.clear();
            src.fitting_result.clear();
        }
    }

    /// Polyline.hpp:59-65 `void append(const Point &point)`
    pub fn append_point(&mut self, point: Point) {
        // Polyline.hpp:61-62 BBS: don't need to append same point
        if !self.empty() && self.last_point() == point {
            return;
        }
        // Polyline.hpp:63 MultiPoint::append(point)
        self.points.push(point);
        // Polyline.hpp:64
        self.append_fitting_result_after_append_points();
    }

    /// Polyline.hpp:67-80 `void append_before(const Point& point)`
    pub fn append_before(&mut self, point: Point) {
        // Polyline.hpp:69-70 BBS: don't need to append same point
        if !self.empty() && self.first_point() == point {
            return;
        }
        // Polyline.hpp:71-74
        if self.size() == 1 {
            self.fitting_result.clear();
            self.points.push(point);
            self.points.reverse();
        } else {
            // Polyline.hpp:76-78
            self.reverse();
            self.append_point(point);
            self.reverse();
        }
    }

    /// Polyline.hpp:82-88 `void append(const Points &src)`
    pub fn append_points(&mut self, src: &[Point]) {
        // Polyline.hpp:84-87 BBS: don't need to append same point
        if !self.empty() && !src.is_empty() && self.last_point() == src[0] {
            self.append_iter(&src[1..]);
        } else {
            self.append_iter(src);
        }
    }

    /// Polyline.hpp:89-96 `void append(const Points::const_iterator &begin, const Points::const_iterator &end)`
    pub fn append_iter(&mut self, src: &[Point]) {
        // Polyline.hpp:91-94 BBS: don't need to append same point
        if !self.empty() && !src.is_empty() && self.last_point() == src[0] {
            self.points.extend_from_slice(&src[1..]);
        } else {
            self.points.extend_from_slice(src);
        }
        // Polyline.hpp:95
        self.append_fitting_result_after_append_points();
    }

    /// MultiPoint::append(begin, end) — raw append used internally (no dedup).
    /// Polyline.cpp uses `MultiPoint::append(...)` directly (no fitting update).
    fn multipoint_append_iter(&mut self, src: &[Point]) {
        self.points.extend_from_slice(src);
    }

    /// Polyline.cpp:406-417 `void Polyline::append_fitting_result_after_append_points()`
    fn append_fitting_result_after_append_points(&mut self) {
        // Polyline.cpp:407
        if !self.fitting_result.is_empty() {
            // Polyline.cpp:408-409
            if self.fitting_result.last().unwrap().is_linear_move() {
                self.fitting_result.last_mut().unwrap().end_point_index = self.points.len() - 1;
            } else {
                // Polyline.cpp:411-414
                let new_start = self.fitting_result.last().unwrap().end_point_index;
                let new_end = self.points.len() - 1;
                if new_start != new_end {
                    self.fitting_result.push(PathFittingData::new(
                        new_start,
                        new_end,
                        EMovePathType::LinearMove,
                        ArcSegment::new(),
                    ));
                }
            }
        }
    }

    /// Polyline.cpp:419-439 `void Polyline::append_fitting_result_after_append_polyline(const Polyline& src)`
    fn append_fitting_result_after_append_polyline(&mut self, src: &Polyline) {
        // Polyline.cpp:421
        if !self.fitting_result.is_empty() {
            // Polyline.cpp:422-423 BBS: offset and save the fitting_result from src polyline
            if !src.fitting_result.is_empty() {
                // Polyline.cpp:424-426
                let old_size = self.fitting_result.len();
                let index_offset = self.fitting_result.last().unwrap().end_point_index;
                self.fitting_result
                    .extend_from_slice(&src.fitting_result);
                // Polyline.cpp:427-430
                for i in old_size..self.fitting_result.len() {
                    self.fitting_result[i].start_point_index += index_offset;
                    self.fitting_result[i].end_point_index += index_offset;
                }
            } else {
                // Polyline.cpp:431-436 BBS: the append polyline has no fitting data,
                // then append as linear move directly
                let new_start = self.fitting_result.last().unwrap().end_point_index;
                let new_end = self.size() - 1;
                if new_start != new_end {
                    self.fitting_result.push(PathFittingData::new(
                        new_start,
                        new_end,
                        EMovePathType::LinearMove,
                        ArcSegment::new(),
                    ));
                }
            }
        }
    }

    /// Polyline.cpp:441-446 `void Polyline::reset_to_linear_move()`
    pub fn reset_to_linear_move(&mut self) {
        // Polyline.cpp:443
        self.fitting_result.clear();
        // Polyline.cpp:444
        self.fitting_result.push(PathFittingData::new(
            0,
            self.points.len() - 1,
            EMovePathType::LinearMove,
            ArcSegment::new(),
        ));
        // Polyline.cpp:445
        self.fitting_result.shrink_to_fit();
    }

    /// Polyline.cpp:448-480 `bool Polyline::split_fitting_result_before_index(...)`
    fn split_fitting_result_before_index(
        &self,
        index: usize,
        new_endpoint: &mut Point,
        data: &mut Vec<PathFittingData>,
    ) -> bool {
        // Polyline.cpp:450
        data.clear();
        // Polyline.cpp:451
        *new_endpoint = self.points[index];
        // Polyline.cpp:452
        if !self.fitting_result.is_empty() {
            // Polyline.cpp:453-454 BBS: max size
            data.reserve(self.fitting_result.len());
            // Polyline.cpp:455-462 BBS: save fitting result before index
            for i in 0..self.fitting_result.len() {
                if self.fitting_result[i].start_point_index < index {
                    data.push(self.fitting_result[i].clone());
                } else {
                    break;
                }
            }

            // Polyline.cpp:464
            if !data.is_empty() {
                // Polyline.cpp:465-466 BBS: need to clip the arc and generate new end point
                if data.last().unwrap().is_arc_move() && data.last().unwrap().end_point_index > index
                {
                    // Polyline.cpp:467-472
                    if !data.last_mut().unwrap().arc_data.clip_end(self.points[index]) {
                        // BBS: failed to clip arc, then return to be linear move
                        data.last_mut().unwrap().path_type = EMovePathType::LinearMove;
                    } else {
                        // BBS: succeed to clip arc, then update and return the new end point
                        *new_endpoint = data.last().unwrap().arc_data.end_point;
                    }
                }
                // Polyline.cpp:474
                data.last_mut().unwrap().end_point_index = index;
            }
            // Polyline.cpp:476
            data.shrink_to_fit();
            // Polyline.cpp:477
            return true;
        }
        // Polyline.cpp:479
        false
    }

    /// Polyline.cpp:481-515 `bool Polyline::split_fitting_result_after_index(...)`
    fn split_fitting_result_after_index(
        &self,
        index: usize,
        new_startpoint: &mut Point,
        data: &mut Vec<PathFittingData>,
    ) -> bool {
        // Polyline.cpp:483
        data.clear();
        // Polyline.cpp:484
        *new_startpoint = self.points[index];
        // Polyline.cpp:485
        if !self.fitting_result.is_empty() {
            // Polyline.cpp:486
            data.reserve(self.fitting_result.len());
            // Polyline.cpp:487-490
            for i in 0..self.fitting_result.len() {
                if self.fitting_result[i].end_point_index > index {
                    data.push(self.fitting_result[i].clone());
                }
            }
            // Polyline.cpp:491
            if !data.is_empty() {
                // Polyline.cpp:492-509
                for i in 0..data.len() {
                    if i != 0 {
                        // Polyline.cpp:493-495
                        data[i].start_point_index -= index;
                        data[i].end_point_index -= index;
                    } else {
                        // Polyline.cpp:496-497
                        data[i].end_point_index -= index;
                        // Polyline.cpp:498-499 BBS: need to clip the arc and generate new start point
                        if data[0].is_arc_move() && data[0].start_point_index < index {
                            // Polyline.cpp:500-505
                            if !data[0].arc_data.clip_start(self.points[index]) {
                                // BBS: failed to clip arc, then return to be linear move
                                data[0].path_type = EMovePathType::LinearMove;
                            } else {
                                // BBS: succeed to clip arc, then update and return the new start point
                                *new_startpoint = data[0].arc_data.start_point;
                            }
                        }
                        // Polyline.cpp:507
                        data[i].start_point_index = 0;
                    }
                }
            }
            // Polyline.cpp:511
            data.shrink_to_fit();
            // Polyline.cpp:512
            return true;
        }
        // Polyline.cpp:514
        false
    }

    /// Polyline.cpp:621-632 `Polyline Polyline::rebase_at(size_t idx)`
    pub fn rebase_at(&self, idx: usize) -> Polyline {
        // Polyline.cpp:623-624
        if !self.is_closed() {
            return Polyline::new();
        }
        // Polyline.cpp:625
        let mut ret = self.clone();
        // Polyline.cpp:626
        let n = self.points.len();
        // Polyline.cpp:627-629
        for j in 0..(n - 1) {
            ret.points[j] = self.points[(idx + j) % (n - 1)];
        }
        // Polyline.cpp:630
        let first = ret.points[0];
        ret.points[n - 1] = first;
        // Polyline.cpp:631
        ret
    }

    // -------------------------------------------------------------------------
    // Rust-only convenience helpers retained for crate-internal callers below.
    // These are NOT part of the C++ Polyline.cpp surface.
    // -------------------------------------------------------------------------

    /// Rust-only: line segment at the given index.
    #[inline]
    pub fn edge(&self, index: usize) -> Line {
        Line::new(self.points[index], self.points[index + 1])
    }

    /// Rust-only: all edges (alias for `lines`).
    pub fn edges(&self) -> Vec<Line> {
        self.lines()
    }

    /// Rust-only: number of edges.
    #[inline]
    pub fn edge_count(&self) -> usize {
        if self.points.len() < 2 {
            0
        } else {
            self.points.len() - 1
        }
    }

    /// Rust-only: bounding box.
    pub fn bounding_box(&self) -> BoundingBox {
        BoundingBox::from_points(&self.points)
    }

    /// Rust-only: closest point on the polyline to `p`.
    pub fn closest_point(&self, p: &Point) -> Point {
        if self.points.is_empty() {
            return Point::zero();
        }
        if self.points.len() == 1 {
            return self.points[0];
        }
        let mut closest = self.points[0];
        let mut min_dist = i128::MAX;
        for edge in self.edges() {
            let proj = edge.project_point(p);
            let dist = p.distance_squared(&proj);
            if dist < min_dist {
                min_dist = dist;
                closest = proj;
            }
        }
        closest
    }

    /// Rust-only: distance from a point to the polyline.
    pub fn distance_to_point(&self, p: &Point) -> CoordF {
        let closest = self.closest_point(p);
        p.distance(&closest)
    }

    /// Rust-only: translate by a vector.
    pub fn translate(&mut self, v: Point) {
        for p in &mut self.points {
            *p = *p + v;
        }
    }

    /// Rust-only: translated copy.
    pub fn translated(&self, v: Point) -> Self {
        let mut result = self.clone();
        result.translate(v);
        result
    }

    /// Rust-only: scale about origin.
    pub fn scale(&mut self, factor: CoordF) {
        for p in &mut self.points {
            *p = *p * factor;
        }
    }

    /// Rust-only: scaled copy.
    pub fn scaled(&self, factor: CoordF) -> Self {
        let mut result = self.clone();
        result.scale(factor);
        result
    }

    /// Rust-only: rotate about origin.
    pub fn rotate(&mut self, angle: CoordF) {
        for p in &mut self.points {
            *p = p.rotate(angle);
        }
    }

    /// Rust-only: rotated copy.
    pub fn rotated(&self, angle: CoordF) -> Self {
        let mut result = self.clone();
        result.rotate(angle);
        result
    }

    /// Rotate every point about `center`.
    ///
    /// MultiPoint.cpp:37-46 — `void MultiPoint::rotate(double angle, const Point &center)`.
    pub fn rotate_around(&mut self, angle: CoordF, center: Point) {
        for p in &mut self.points {
            *p = p.rotate_around(angle, center);
        }
    }

    /// Rust-only: index-based two-way split, using C++ `split_at_index` semantics.
    /// Returns (p1, p2). Retained for crate callers (e.g. curve_analyzer).
    pub fn split_at(&self, index: usize) -> (Self, Self) {
        let mut p1 = Polyline::new();
        let mut p2 = Polyline::new();
        self.split_at_index(index, &mut p1, &mut p2);
        (p1, p2)
    }

    /// Rust-only: concatenate two polylines (faithful append semantics).
    pub fn concat(&self, other: &Polyline) -> Self {
        let mut result = self.clone();
        result.append(other);
        result
    }

    /// Rust-only: convert to a polygon.
    pub fn to_polygon(&self) -> Polygon {
        Polygon::from_points(self.points.clone())
    }

    /// Rust-only: simplified copy (DP).
    pub fn simplified(&self, tolerance: f64) -> Self {
        let mut result = self.clone();
        result.simplify(tolerance);
        result
    }

    /// Rust-only: direction unit vector at a vertex.
    pub fn direction_at(&self, index: usize) -> Option<Point> {
        if self.points.len() < 2 || index >= self.points.len() {
            return None;
        }
        if index == 0 {
            let edge = self.edge(0);
            let dir = edge.direction();
            let len = dir.length();
            if len > 0.0 {
                Some(dir * (1.0 / len))
            } else {
                None
            }
        } else if index == self.points.len() - 1 {
            let edge = self.edge(index - 1);
            let dir = edge.direction();
            let len = dir.length();
            if len > 0.0 {
                Some(dir * (1.0 / len))
            } else {
                None
            }
        } else {
            let in_edge = self.edge(index - 1);
            let out_edge = self.edge(index);
            let in_dir = in_edge.direction();
            let out_dir = out_edge.direction();
            let in_len = in_dir.length();
            let out_len = out_dir.length();
            if in_len > 0.0 && out_len > 0.0 {
                let avg = Point::new(
                    ((in_dir.x as CoordF / in_len + out_dir.x as CoordF / out_len) / 2.0).round()
                        as Coord,
                    ((in_dir.y as CoordF / in_len + out_dir.y as CoordF / out_len) / 2.0).round()
                        as Coord,
                );
                let avg_len = avg.length();
                if avg_len > 0.0 {
                    Some(avg * (1.0 / avg_len))
                } else {
                    None
                }
            } else {
                None
            }
        }
    }

    /// Rust-only: Douglas-Peucker wrapper (MultiPoint::_douglas_peucker).
    pub fn douglas_peucker(polyline: &Polyline, tolerance: CoordF) -> Polyline {
        use crate::geometry::simplify::douglas_peucker_polyline;
        douglas_peucker_polyline(polyline, tolerance)
    }
}

// Line.cpp:60-66 `double Line::direction() const` (inlined for is_straight).
// Computes the line direction angle, normalized like C++.
fn line_direction(a: Point, b: Point) -> CoordF {
    // Line.cpp:62 atan2_() = atan2(b.y - a.y, b.x - a.x)
    let atan2 = ((b.y - a.y) as CoordF).atan2((b.x - a.x) as CoordF);
    // Line.cpp:63-65
    let pi = std::f64::consts::PI;
    if (atan2 - pi).abs() < crate::libslic3r::EPSILON {
        0.0
    } else if atan2 < 0.0 {
        atan2 + pi
    } else {
        atan2
    }
}

// Line.cpp:68-71 `bool Line::parallel_to(double angle) const`
fn line_parallel_to_angle(line: &Line, angle: CoordF) -> bool {
    super::directions_parallel(line_direction(line.a, line.b), angle, 0.0)
}

// =============================================================================
// Free functions from Polyline.cpp / Polyline.hpp
// =============================================================================

/// Polyline.cpp:517-520 `BoundingBox get_extents(const Polyline &polyline)`
pub fn get_extents(polyline: &Polyline) -> BoundingBox {
    // Polyline.cpp:519
    polyline.bounding_box()
}

/// Polyline.cpp:522-531 `BoundingBox get_extents(const Polylines &polylines)`
pub fn get_extents_polylines(polylines: &Polylines) -> BoundingBox {
    // Polyline.cpp:524
    let mut bb = BoundingBox::new();
    // Polyline.cpp:525-529
    if !polylines.is_empty() {
        bb = polylines[0].bounding_box();
        for i in 1..polylines.len() {
            // C++: bb.merge(polylines[i].points) — merges each point.
            for p in &polylines[i].points {
                bb.merge_point(*p);
            }
        }
    }
    // Polyline.cpp:530
    bb
}

/// Polyline.cpp:533-545 `bool remove_same_neighbor(Polyline &polyline)`
/// Return True when erase some otherwise False.
pub fn remove_same_neighbor(polyline: &mut Polyline) -> bool {
    // Polyline.cpp:536
    let points = &mut polyline.points;
    // Polyline.cpp:537
    if points.is_empty() {
        return false;
    }
    // Polyline.cpp:538 std::unique
    let before = points.len();
    points.dedup();
    // Polyline.cpp:540-541 no duplicits
    if points.len() == before {
        return false;
    }
    // Polyline.cpp:543-544 (erase already done by dedup)
    true
}

/// Polyline.cpp:547-555 `bool remove_same_neighbor(Polylines &polylines)`
pub fn remove_same_neighbor_polylines(polylines: &mut Polylines) -> bool {
    // Polyline.cpp:549
    if polylines.is_empty() {
        return false;
    }
    // Polyline.cpp:550-551
    let mut exist = false;
    for polyline in polylines.iter_mut() {
        exist |= remove_same_neighbor(polyline);
    }
    // Polyline.cpp:553 remove empty polylines (points.size() <= 1)
    polylines.retain(|p| p.points.len() > 1);
    // Polyline.cpp:554
    exist
}

/// Polyline.cpp:557-569 `const Point& leftmost_point(const Polylines &polylines)`
pub fn leftmost_point(polylines: &Polylines) -> crate::Result<Point> {
    // Polyline.cpp:559-560
    if polylines.is_empty() {
        return Err(crate::Error::InvalidInput(
            "leftmost_point() called on empty PolylineCollection".to_string(),
        ));
    }
    // Polyline.cpp:561-562
    let mut p = polylines[0].leftmost_point();
    // Polyline.cpp:563-567
    for it in polylines.iter().skip(1) {
        let p2 = it.leftmost_point();
        if p2.x < p.x {
            p = p2;
        }
    }
    // Polyline.cpp:568
    Ok(p)
}

/// Polyline.cpp:571-586 `bool remove_degenerate(Polylines &polylines)`
pub fn remove_degenerate(polylines: &mut Polylines) -> bool {
    // Polyline.cpp:573
    let mut modified = false;
    // Polyline.cpp:574
    let mut j = 0usize;
    // Polyline.cpp:575-582
    for i in 0..polylines.len() {
        if polylines[i].points.len() >= 2 {
            if j < i {
                polylines.swap(i, j);
            }
            j += 1;
        } else {
            modified = true;
        }
    }
    // Polyline.cpp:583-584
    if j < polylines.len() {
        polylines.truncate(j);
    }
    // Polyline.cpp:585
    modified
}

/// Polyline.cpp:588-608 `std::pair<int, Point> foot_pt(const Points &polyline, const Point &pt)`
/// Returns index of a segment of a polyline and foot point of pt on polyline.
pub fn foot_pt(polyline: &[Point], pt: &Point) -> (i32, Point) {
    // Polyline.cpp:590
    if polyline.len() < 2 {
        return (-1, Point::new(0, 0));
    }

    // Polyline.cpp:592
    let mut d2_min = f64::MAX;
    // Polyline.cpp:593
    let mut foot_pt_min = Point::new(0, 0);
    // Polyline.cpp:594
    let mut prev = polyline[0];
    // Polyline.cpp:595-596
    let mut it_proj = 0usize;
    // Polyline.cpp:597-606
    for it in 1..polyline.len() {
        // Polyline.cpp:598 — foot_pt = pt.projection_onto(Line(prev, *it))
        let foot_pt = pt.project_onto_segment(prev, polyline[it]);
        // Polyline.cpp:599 — d2 = (foot_pt - pt).cast<double>().squaredNorm()
        let dx = (foot_pt.x - pt.x) as f64;
        let dy = (foot_pt.y - pt.y) as f64;
        let d2 = dx * dx + dy * dy;
        // Polyline.cpp:600-604
        if d2 < d2_min {
            d2_min = d2;
            foot_pt_min = foot_pt;
            it_proj = it;
        }
        // Polyline.cpp:605
        prev = polyline[it];
    }
    // Polyline.cpp:607 — make_pair(int(it_proj - begin) - 1, foot_pt_min)
    (it_proj as i32 - 1, foot_pt_min)
}

/// Polyline.hpp:163-168 `inline double total_length(const Polylines &polylines)`
pub fn total_length(polylines: &Polylines) -> f64 {
    // Polyline.hpp:164
    let mut total = 0.0;
    // Polyline.hpp:165-166
    for pl in polylines {
        total += pl.length();
    }
    // Polyline.hpp:167
    total
}

/// Polyline.hpp:170-179 `inline Lines to_lines(const Polyline &poly)`
pub fn to_lines(poly: &Polyline) -> Vec<Line> {
    // Polyline.hpp:172
    let mut lines: Vec<Line> = Vec::new();
    // Polyline.hpp:173
    if poly.points.len() >= 2 {
        lines.reserve(poly.points.len() - 1);
        // Polyline.hpp:175-176
        for it in 0..(poly.points.len() - 1) {
            lines.push(Line::new(poly.points[it], poly.points[it + 1]));
        }
    }
    // Polyline.hpp:178
    lines
}

/// Polyline.hpp:181-195 `inline Lines to_lines(const Polylines &polys)`
pub fn to_lines_polylines(polys: &Polylines) -> Vec<Line> {
    // Polyline.hpp:183-186
    let mut n_lines = 0usize;
    for poly in polys {
        if poly.points.len() > 1 {
            n_lines += poly.points.len() - 1;
        }
    }
    // Polyline.hpp:187-188
    let mut lines: Vec<Line> = Vec::new();
    lines.reserve(n_lines);
    // Polyline.hpp:189-193
    for poly in polys {
        for it in 0..poly.points.len().saturating_sub(1) {
            lines.push(Line::new(poly.points[it], poly.points[it + 1]));
        }
    }
    // Polyline.hpp:194
    lines
}

/// Polyline.hpp:197-204 `inline Polylines to_polylines(const std::vector<Points> &paths)`
pub fn to_polylines(paths: &[Vec<Point>]) -> Polylines {
    // Polyline.hpp:199-200
    let mut out: Polylines = Vec::new();
    out.reserve(paths.len());
    // Polyline.hpp:201-202
    for path in paths {
        out.push(Polyline::from_points(path.clone()));
    }
    // Polyline.hpp:203
    out
}

/// Polyline.hpp:215-218 `inline void polylines_append(Polylines &dst, const Polylines &src)`
pub fn polylines_append(dst: &mut Polylines, src: &Polylines) {
    // Polyline.hpp:217
    dst.extend_from_slice(src);
}

/// Polyline.hpp:220-228 `inline void polylines_append(Polylines &dst, Polylines &&src)`
pub fn polylines_append_move(dst: &mut Polylines, mut src: Polylines) {
    // Polyline.hpp:222-227
    if dst.is_empty() {
        *dst = std::mem::take(&mut src);
    } else {
        dst.append(&mut src);
        src.clear();
    }
}

impl fmt::Debug for Polyline {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Polyline({} points)", self.points.len())
    }
}

impl fmt::Display for Polyline {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Polyline[")?;
        for (i, p) in self.points.iter().enumerate() {
            if i > 0 {
                write!(f, " -> ")?;
            }
            write!(f, "{}", p)?;
        }
        write!(f, "]")
    }
}

impl Deref for Polyline {
    type Target = [Point];

    fn deref(&self) -> &Self::Target {
        &self.points
    }
}

impl DerefMut for Polyline {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.points
    }
}

impl Index<usize> for Polyline {
    type Output = Point;

    fn index(&self, index: usize) -> &Self::Output {
        &self.points[index]
    }
}

impl IndexMut<usize> for Polyline {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.points[index]
    }
}

impl FromIterator<Point> for Polyline {
    fn from_iter<I: IntoIterator<Item = Point>>(iter: I) -> Self {
        Self::from_points(iter.into_iter().collect())
    }
}

impl IntoIterator for Polyline {
    type Item = Point;
    type IntoIter = std::vec::IntoIter<Point>;

    fn into_iter(self) -> Self::IntoIter {
        self.points.into_iter()
    }
}

impl<'a> IntoIterator for &'a Polyline {
    type Item = &'a Point;
    type IntoIter = std::slice::Iter<'a, Point>;

    fn into_iter(self) -> Self::IntoIter {
        self.points.iter()
    }
}

impl<'a> IntoIterator for &'a mut Polyline {
    type Item = &'a mut Point;
    type IntoIter = std::slice::IterMut<'a, Point>;

    fn into_iter(self) -> Self::IntoIter {
        self.points.iter_mut()
    }
}

impl From<Vec<Point>> for Polyline {
    fn from(points: Vec<Point>) -> Self {
        Self::from_points(points)
    }
}

impl From<Polyline> for Vec<Point> {
    fn from(polyline: Polyline) -> Self {
        polyline.into_points()
    }
}

impl From<Polygon> for Polyline {
    fn from(polygon: Polygon) -> Self {
        Self::from_points(polygon.into_points())
    }
}

/// Polyline.hpp:16 `typedef std::vector<Polyline> Polylines;`
pub type Polylines = Vec<Polyline>;

#[cfg(test)]
mod tests {
    use super::*;

    fn make_polyline() -> Polyline {
        Polyline::from_points(vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 100),
            Point::new(0, 100),
        ])
    }

    #[test]
    fn test_polyline_new() {
        let pl = Polyline::new();
        assert!(pl.is_empty());
        assert_eq!(pl.len(), 0);
    }

    #[test]
    fn test_polyline_from_points() {
        let pl = make_polyline();
        assert_eq!(pl.len(), 4);
        assert!(!pl.is_empty());
    }

    #[test]
    fn test_polyline_lines() {
        let pl = make_polyline();
        let lines = pl.lines();
        assert_eq!(lines.len(), 3); // 4 points = 3 lines (open path)
        assert_eq!(lines[0].a, Point::new(0, 0));
        assert_eq!(lines[0].b, Point::new(100, 0));
    }

    #[test]
    fn test_polyline_length() {
        let pl = make_polyline();
        let len = pl.length();
        assert!((len - 300.0).abs() < 1.0); // 100 + 100 + 100
    }

    #[test]
    fn test_polyline_is_closed() {
        let pl = make_polyline();
        assert!(!pl.is_closed());

        let closed = Polyline::from_points(vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 100),
            Point::new(0, 0),
        ]);
        assert!(closed.is_closed());
    }

    #[test]
    fn test_polyline_first_last() {
        let pl = make_polyline();
        assert_eq!(pl.first_point(), Point::new(0, 0));
        assert_eq!(pl.last_point(), Point::new(0, 100));
    }

    #[test]
    fn test_polyline_reverse() {
        let mut pl = make_polyline();
        pl.reverse();
        assert_eq!(pl.first_point(), Point::new(0, 100));
        assert_eq!(pl.last_point(), Point::new(0, 0));
    }

    #[test]
    fn test_polyline_leftmost_point() {
        let pl = Polyline::from_points(vec![
            Point::new(50, 0),
            Point::new(10, 100),
            Point::new(80, 50),
        ]);
        assert_eq!(pl.leftmost_point(), Point::new(10, 100));
    }

    #[test]
    fn test_polyline_split_at_index() {
        let pl = make_polyline();
        let (first, second) = pl.split_at(2);
        assert_eq!(first.len(), 3);
        assert_eq!(second.len(), 2);
        assert_eq!(first.last_point(), second.first_point());
    }

    #[test]
    fn test_polyline_append_faithful() {
        let mut pl1 = Polyline::from_points(vec![Point::new(0, 0), Point::new(100, 0)]);
        let pl2 = Polyline::from_points(vec![Point::new(100, 0), Point::new(100, 100)]);
        pl1.append(&pl2);
        // The shared point (100,0) is deduplicated by append_point.
        assert_eq!(pl1.len(), 3);
    }

    #[test]
    fn test_polyline_is_valid() {
        let pl = make_polyline();
        assert!(pl.is_valid());

        let single = Polyline::from_points(vec![Point::new(0, 0)]);
        assert!(!single.is_valid());
    }

    #[test]
    fn test_polyline_clip_end() {
        let mut pl = Polyline::from_points(vec![Point::new(0, 0), Point::new(100, 0)]);
        pl.clip_end(30.0);
        assert_eq!(pl.last_point(), Point::new(70, 0));
    }

    #[test]
    fn test_polyline_is_straight() {
        let straight = Polyline::from_points(vec![
            Point::new(0, 0),
            Point::new(50, 0),
            Point::new(100, 0),
        ]);
        assert!(straight.is_straight());
        let bent = make_polyline();
        assert!(!bent.is_straight());
    }

    #[test]
    fn test_remove_degenerate() {
        let mut polys = vec![
            Polyline::from_points(vec![Point::new(0, 0)]),
            Polyline::from_points(vec![Point::new(0, 0), Point::new(10, 0)]),
        ];
        let modified = remove_degenerate(&mut polys);
        assert!(modified);
        assert_eq!(polys.len(), 1);
    }

    #[test]
    fn test_foot_pt() {
        let pts = vec![Point::new(0, 0), Point::new(100, 0)];
        let (idx, foot) = foot_pt(&pts, &Point::new(50, 20));
        assert_eq!(idx, 0);
        assert_eq!(foot, Point::new(50, 0));
    }
}
