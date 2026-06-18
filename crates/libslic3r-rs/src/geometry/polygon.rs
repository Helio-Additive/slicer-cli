//! Polygon type for closed contours.
//!
//! Faithful 1:1 port of BambuStudio `src/libslic3r/Polygon.cpp` (+ `Polygon.hpp`).
//! coord_t -> i64 (Coord), coordf_t -> f64 (CoordF).
//!
//! Notes on faithfulness:
//! - C++ `pt.cast<double>()` is a RAW integer->double cast (scaled units), NOT an
//!   unscale. So `(a - b).cast<double>().norm()` == `a.distance(&b)` (which uses
//!   raw integer coords), and `.squaredNorm()` == `a.distance_squared(&b) as f64`.
//!   We must NEVER use `Point::to_f64()` (which unscales) for `cast<double>()`.
//! - `MultiPoint::_douglas_peucker` == `crate::multi_point::douglas_peucker`.
//! - `cross2(Vec2d, Vec2d)` (float) == `crate::geometry::cross2f`;
//!   `cross2(<int64>, <int64>)` == `crate::geometry::cross2`.

use super::{cross2, cross2f, BoundingBox, Line, Point, PointF, Polyline};
use crate::{Coord, CoordF};
use serde::{Deserialize, Serialize};
use std::fmt;
use std::ops::{Deref, DerefMut, Index, IndexMut};

/// A closed polygon defined by a sequence of points.
///
/// Polygon.hpp:22 — `class Polygon : public MultiPoint`
/// C++: `class MultiPoint { public: Points points; }`
#[derive(Clone, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Polygon {
    /// Points defining the polygon (public to match C++ MultiPoint).
    /// MultiPoint.hpp:9 — `Points points;`
    pub points: Vec<Point>,
}

impl Polygon {
    // ------------------------------------------------------------------
    // Constructors (Polygon.hpp:25-38) + crate ergonomic helpers.
    // ------------------------------------------------------------------

    /// Polygon.hpp:25 — `Polygon() = default;`
    #[inline]
    pub fn new() -> Self {
        Self { points: Vec::new() }
    }

    /// Polygon.hpp:26 — `explicit Polygon(const Points &points) : MultiPoint(points) {}`
    #[inline]
    pub fn from_points(points: Vec<Point>) -> Self {
        Self { points }
    }

    /// Polygon.hpp:30 — `static Polygon new_scale(const std::vector<Vec2d> &points)`
    pub fn new_scale(points: &[PointF]) -> Self {
        // Polygon.hpp:31-35
        let mut pgn = Polygon::new();
        pgn.points.reserve(points.len());
        for pt in points {
            pgn.points.push(Point::new_scale(pt.x, pt.y));
        }
        pgn
    }

    /// Create a polygon with the given capacity. (crate helper)
    #[inline]
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            points: Vec::with_capacity(capacity),
        }
    }

    /// (crate helper) Get the points of this polygon.
    #[inline]
    pub fn points(&self) -> &[Point] {
        &self.points
    }

    /// (crate helper) Get a mutable reference to the points.
    #[inline]
    pub fn points_mut(&mut self) -> &mut Vec<Point> {
        &mut self.points
    }

    /// (crate helper) Consume the polygon and return its points.
    #[inline]
    pub fn into_points(self) -> Vec<Point> {
        self.points
    }

    /// (crate helper) Number of points.
    #[inline]
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// (crate helper) Whether the polygon has no points.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// (crate helper) First point.
    #[inline]
    pub fn first_point(&self) -> Point {
        self.points[0]
    }

    /// Polygon.hpp:44 — `const Point& last_point() const { return this->points.front(); }`
    #[inline]
    pub fn last_point(&self) -> Point {
        self.points[0]
    }

    /// (crate helper) Push a point.
    #[inline]
    pub fn push(&mut self, point: Point) {
        self.points.push(point);
    }

    /// (crate helper) Clear all points.
    #[inline]
    pub fn clear(&mut self) {
        self.points.clear();
    }

    /// (crate helper) Reserve capacity.
    #[inline]
    pub fn reserve(&mut self, additional: usize) {
        self.points.reserve(additional);
    }

    // ------------------------------------------------------------------
    // MultiPoint operations used as Polygon methods.
    // (MultiPoint has no Rust base class; reverse lives here.)
    // ------------------------------------------------------------------

    /// MultiPoint.hpp `reverse()` — reverse the order of points in place.
    #[inline]
    pub fn reverse(&mut self) {
        self.points.reverse();
    }

    /// (crate helper) Return a reversed copy.
    pub fn reversed(&self) -> Self {
        let mut result = self.clone();
        result.reverse();
        result
    }

    /// MultiPoint `rotate(angle)` — rotate every point about the origin. (crate helper)
    pub fn rotate(&mut self, angle: CoordF) {
        for p in &mut self.points {
            *p = p.rotate(angle);
        }
    }

    /// (crate helper) Return a rotated copy.
    pub fn rotated(&self, angle: CoordF) -> Self {
        let mut result = self.clone();
        result.rotate(angle);
        result
    }

    /// MultiPoint `rotate(cos, sin)` — used by `polygons_rotate`.
    pub fn rotate_by_cos_sin(&mut self, cos_angle: CoordF, sin_angle: CoordF) {
        for p in &mut self.points {
            *p = p.rotate_by_cos_sin(cos_angle, sin_angle);
        }
    }

    /// MultiPoint `translate(v)` — translate every point. (crate helper)
    pub fn translate(&mut self, v: Point) {
        for p in &mut self.points {
            *p = *p + v;
        }
    }

    /// (crate helper) Return a translated copy.
    pub fn translated(&self, v: Point) -> Self {
        let mut result = self.clone();
        result.translate(v);
        result
    }

    /// (crate helper) Rotate about a center point.
    pub fn rotate_around(&mut self, angle: CoordF, center: Point) {
        for p in &mut self.points {
            *p = p.rotate_around(angle, center);
        }
    }

    /// (crate helper) Return a copy rotated about a center.
    pub fn rotated_around(&self, angle: CoordF, center: Point) -> Self {
        let mut result = self.clone();
        result.rotate_around(angle, center);
        result
    }

    // ------------------------------------------------------------------
    // Polygon.cpp — methods, in source order.
    // ------------------------------------------------------------------

    /// Polygon.cpp:11 — `double Polygon::length() const`
    pub fn length(&self) -> CoordF {
        // Polygon.cpp:13
        let mut l = 0.0_f64;
        // Polygon.cpp:14
        if self.points.len() > 1 {
            // Polygon.cpp:15 — (back - front).cast<double>().norm()
            l = self.points[self.points.len() - 1].distance(&self.points[0]);
            // Polygon.cpp:16-17
            for i in 1..self.points.len() {
                l += self.points[i].distance(&self.points[i - 1]);
            }
        }
        // Polygon.cpp:19
        l
    }

    /// Polygon.cpp:22 — `Lines Polygon::lines() const`
    pub fn lines(&self) -> Vec<Line> {
        // Polygon.cpp:24 — return to_lines(*this);
        to_lines(self)
    }

    /// Polygon.cpp:27 — `Polyline Polygon::split_at_vertex(const Point &point) const`
    pub fn split_at_vertex(&self, point: &Point) -> Polyline {
        // Polygon.cpp:29-32 — find index of point
        for (i, pt) in self.points.iter().enumerate() {
            if *pt == *point {
                return self.split_at_index(i as i32);
            }
        }
        // Polygon.cpp:33 — throw Slic3r::InvalidArgument("Point not found");
        panic!("Point not found");
    }

    /// Split a closed polygon into an open polyline, with the split point duplicated at both ends.
    /// Polygon.cpp:38 — `Polyline Polygon::split_at_index(int index) const`
    pub fn split_at_index(&self, index: i32) -> Polyline {
        // Polygon.cpp:40-41
        let mut polyline = Polyline::new();
        polyline.points.reserve(self.points.len() + 1);
        let index = index as usize;
        // Polygon.cpp:42-43 — from points.begin()+index to end()
        for it in &self.points[index..] {
            polyline.points.push(*it);
        }
        // Polygon.cpp:44-45 — from points.begin() to begin()+index+1
        for it in &self.points[..=index] {
            polyline.points.push(*it);
        }
        // Polygon.cpp:46
        polyline
    }

    /// Split a closed polygon into an open polyline, with the split point duplicated at both ends.
    /// Polygon.hpp:52 — `Polyline split_at_first_point() const { return this->split_at_index(0); }`
    #[inline]
    pub fn split_at_first_point(&self) -> Polyline {
        self.split_at_index(0)
    }

    /// Polygon.hpp:53 — `Points equally_spaced_points(double distance) const`
    #[inline]
    pub fn equally_spaced_points(&self, distance: CoordF) -> Vec<Point> {
        self.split_at_first_point().equally_spaced_points(distance)
    }

    /// Polygon.cpp:49 — `static double Polygon::area(const Points &points)`
    pub fn area_of(points: &[Point]) -> CoordF {
        // Polygon.cpp:51
        let mut a = 0.0_f64;
        // Polygon.cpp:52
        if points.len() >= 3 {
            // Polygon.cpp:53 — Vec2d p1 = points.back().cast<double>();
            let mut p1 = PointF::new(
                points[points.len() - 1].x as CoordF,
                points[points.len() - 1].y as CoordF,
            );
            // Polygon.cpp:54-58
            for p in points {
                let p2 = PointF::new(p.x as CoordF, p.y as CoordF);
                a += cross2f(p1, p2);
                p1 = p2;
            }
        }
        // Polygon.cpp:60
        0.5 * a
    }

    /// Polygon.cpp:63 — `double Polygon::area() const`
    #[inline]
    pub fn area(&self) -> CoordF {
        // Polygon.cpp:65 — return Polygon::area(points);
        Polygon::area_of(&self.points)
    }

    /// (crate helper) Signed area; alias for `area()` (C++ `area()` is already signed).
    #[inline]
    pub fn signed_area(&self) -> CoordF {
        self.area()
    }

    /// Polygon.cpp:68 — `bool Polygon::is_counter_clockwise() const`
    #[inline]
    pub fn is_counter_clockwise(&self) -> bool {
        // Polygon.cpp:70 — return ClipperLib::Orientation(this->points);
        // ClipperLib::Orientation(path) == (Area(path) >= 0).
        clipper_orientation(&self.points)
    }

    /// Polygon.cpp:73 — `bool Polygon::is_clockwise() const`
    #[inline]
    pub fn is_clockwise(&self) -> bool {
        // Polygon.cpp:75 — return !this->is_counter_clockwise();
        !self.is_counter_clockwise()
    }

    /// Polygon.cpp:78 — `bool Polygon::make_counter_clockwise()`
    pub fn make_counter_clockwise(&mut self) -> bool {
        // Polygon.cpp:80-83
        if !self.is_counter_clockwise() {
            self.reverse();
            return true;
        }
        // Polygon.cpp:84
        false
    }

    /// Polygon.cpp:87 — `bool Polygon::make_clockwise()`
    pub fn make_clockwise(&mut self) -> bool {
        // Polygon.cpp:89-92
        if self.is_counter_clockwise() {
            self.reverse();
            return true;
        }
        // Polygon.cpp:93
        false
    }

    /// Polygon.cpp:96 — `void Polygon::douglas_peucker(double tolerance)`
    pub fn douglas_peucker(&mut self, tolerance: CoordF) {
        // Polygon.cpp:98 — this->points.push_back(this->points.front());
        self.points.push(self.points[0]);
        // Polygon.cpp:99 — Points p = MultiPoint::_douglas_peucker(this->points, tolerance);
        let mut p = crate::multi_point::douglas_peucker(&self.points, tolerance);
        // Polygon.cpp:100 — p.pop_back();
        p.pop();
        // Polygon.cpp:101 — this->points = std::move(p);
        self.points = p;
    }

    /// Polygon.cpp:104 — `bool Polygon::is_approx_circle(...)`
    /// Returns Some((center, diameter)) when the polygon approximates a circle.
    pub fn is_approx_circle(
        &self,
        max_deviation: CoordF,
        max_variance: CoordF,
    ) -> Option<(Point, CoordF)> {
        // Polygon.cpp:106-108
        if self.points.len() < 8 {
            return None;
        }

        // Polygon.cpp:110 — center = centroid();
        let center = self.centroid();
        // Polygon.cpp:111-115
        let mut distances: Vec<CoordF> = Vec::new();
        for point in &self.points {
            // Polygon.cpp:113 — sqrt(pow(point.x - center.x, 2) + pow(point.y - center.y, 2))
            let distance = (((point.x - center.x) as CoordF).powi(2)
                + ((point.y - center.y) as CoordF).powi(2))
            .sqrt();
            distances.push(distance);
        }

        // Polygon.cpp:117 — max_dist = *std::max_element(...)
        let max_dist = distances.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        // Polygon.cpp:118 — min_dist = *std::min_element(...)
        let min_dist = distances.iter().cloned().fold(f64::INFINITY, f64::min);

        // Polygon.cpp:120-122
        if (max_dist - min_dist) > max_deviation {
            return None;
        }

        // Polygon.cpp:124 — avg_dist = accumulate(...) / distances.size()
        let avg_dist: CoordF = distances.iter().sum::<CoordF>() / distances.len() as CoordF;
        // Polygon.cpp:125-127
        let mut variance = 0.0_f64;
        for d in &distances {
            variance += (d - avg_dist).powi(2);
        }
        variance /= distances.len() as CoordF;

        // Polygon.cpp:129-131
        if variance > max_variance {
            return None;
        }

        // Polygon.cpp:133-134 — diameter = 2 * avg_dist; return true;
        let diameter = 2.0 * avg_dist;
        Some((center, diameter))
    }

    /// Does an unoriented polygon contain a point?
    /// Polygon.hpp:69 — `bool contains(const Point &point) const { return Slic3r::contains(*this, point, true); }`
    #[inline]
    pub fn contains(&self, point: &Point) -> bool {
        contains_polygon(self, point, true)
    }

    /// (crate helper) Alias used widely across the crate.
    #[inline]
    pub fn contains_point(&self, point: &Point) -> bool {
        contains_polygon(self, point, true)
    }

    /// Approximate on boundary test.
    /// Polygon.hpp:71 — `bool on_boundary(const Point &point, double eps) const`
    #[inline]
    pub fn on_boundary(&self, point: &Point, eps: CoordF) -> bool {
        // Polygon.hpp:72 — (point_projection(point) - point).cast<double>().squaredNorm() < eps*eps
        let proj = self.point_projection(point);
        (proj.distance_squared(point) as CoordF) < eps * eps
    }

    /// Works on CCW polygons only, CW contour will be reoriented to CCW by Clipper's simplify_polygons()!
    /// Polygon.cpp:137 — `Polygons Polygon::simplify(double tolerance) const`
    pub fn simplify(&self, tolerance: CoordF) -> Polygons {
        // Polygon.cpp:140 — assert(this->is_counter_clockwise());
        debug_assert!(self.is_counter_clockwise());

        // Polygon.cpp:144-145 — repeat first point at the end for Douglas-Peucker
        let mut points = self.points.clone();
        points.push(points[0]);
        // Polygon.cpp:146 — Polygon p(MultiPoint::_douglas_peucker(points, tolerance));
        let mut p = Polygon::from_points(crate::multi_point::douglas_peucker(&points, tolerance));
        // Polygon.cpp:147 — p.points.pop_back();
        p.points.pop();

        // Polygon.cpp:149-151
        let pp = vec![p];
        simplify_polygons_clipper(&pp)
    }

    /// Only call this on convex polygons or it will return invalid results.
    /// Polygon.cpp:155 — `void Polygon::triangulate_convex(Polygons* polygons) const`
    pub fn triangulate_convex(&self, polygons: &mut Polygons) {
        // Polygon.cpp:157 — for (it = points.begin()+2; it != points.end(); ++it)
        for it in 2..self.points.len() {
            // Polygon.cpp:158-162
            let mut p = Polygon::new();
            p.points.reserve(3);
            p.points.push(self.points[0]);
            p.points.push(self.points[it - 1]);
            p.points.push(self.points[it]);

            // Polygon.cpp:164-165 — if (p.area() > 0) polygons->push_back(p);
            if p.area() > 0.0 {
                polygons.push(p);
            }
        }
    }

    /// center of mass
    /// source: https://en.wikipedia.org/wiki/Centroid
    /// Polygon.cpp:171 — `Point Polygon::centroid() const`
    pub fn centroid(&self) -> Point {
        // Polygon.cpp:173-174
        let mut area_sum = 0.0_f64;
        let mut c = PointF::new(0.0, 0.0);
        // Polygon.cpp:175
        if self.points.len() >= 3 {
            // Polygon.cpp:176 — Vec2d p1 = points.back().cast<double>();
            let mut p1 = PointF::new(
                self.points[self.points.len() - 1].x as CoordF,
                self.points[self.points.len() - 1].y as CoordF,
            );
            // Polygon.cpp:177-183
            for p in &self.points {
                let p2 = PointF::new(p.x as CoordF, p.y as CoordF);
                let a = cross2f(p1, p2);
                area_sum += a;
                c = c + (p1 + p2) * a;
                p1 = p2;
            }
        }
        // Polygon.cpp:185 — return Point(Vec2d(c / (3. * area_sum)));
        // Point(Vec2d) constructs via coord_t cast (truncation toward zero).
        let cd = c / (3.0 * area_sum);
        Point::new(cd.x as Coord, cd.y as Coord)
    }

    /// Polygon.cpp:188 — `bool Polygon::intersection(const Line &line, Point *intersection) const`
    pub fn intersection(&self, line: &Line, intersection: &mut Point) -> bool {
        // Polygon.cpp:190-191
        if self.points.len() < 2 {
            return false;
        }
        // Polygon.cpp:192-193 — closing edge first
        if let Some(ip) = Line::new(self.points[0], self.points[self.points.len() - 1]).intersection(line) {
            *intersection = ip;
            return true;
        }
        // Polygon.cpp:194-196
        for i in 1..self.points.len() {
            if let Some(ip) = Line::new(self.points[i - 1], self.points[i]).intersection(line) {
                *intersection = ip;
                return true;
            }
        }
        // Polygon.cpp:197
        false
    }

    /// Polygon.cpp:200 — `bool Polygon::first_intersection(const Line& line, Point* intersection) const`
    pub fn first_intersection(&self, line: &Line, intersection: &mut Point) -> bool {
        // Polygon.cpp:202-203
        if self.points.len() < 2 {
            return false;
        }

        // Polygon.cpp:205-207
        let mut found = false;
        let mut dmin = 0.0_f64;
        let mut l = Line::new(self.points[self.points.len() - 1], self.points[0]);
        // Polygon.cpp:208
        for i in 0..self.points.len() {
            // Polygon.cpp:209 — l.b = this->points[i];
            l.b = self.points[i];
            // Polygon.cpp:210-211
            if let Some(ip) = l.intersection(line) {
                // Polygon.cpp:212-214
                if !found {
                    found = true;
                    // dmin = (line.a - ip).cast<double>().squaredNorm();
                    dmin = line.a.distance_squared(&ip) as CoordF;
                    *intersection = ip;
                } else {
                    // Polygon.cpp:216-221
                    let d = line.a.distance_squared(&ip) as CoordF;
                    if d < dmin {
                        dmin = d;
                        *intersection = ip;
                    }
                }
            }
            // Polygon.cpp:224 — l.a = l.b;
            l.a = l.b;
        }
        // Polygon.cpp:226
        found
    }

    /// Polygon.cpp:229 — `bool Polygon::intersections(const Line &line, Points *intersections) const`
    pub fn intersections(&self, line: &Line, intersections: &mut Vec<Point>) -> bool {
        // Polygon.cpp:231-232
        if self.points.len() < 2 {
            return false;
        }

        // Polygon.cpp:234 — size_t intersections_size = intersections->size();
        let intersections_size = intersections.len();
        // Polygon.cpp:235
        let mut l = Line::new(self.points[self.points.len() - 1], self.points[0]);
        // Polygon.cpp:236
        for i in 0..self.points.len() {
            // Polygon.cpp:237 — l.b = this->points[i];
            l.b = self.points[i];
            // Polygon.cpp:238-240
            if let Some(intersection) = l.intersection(line) {
                intersections.push(intersection);
            }
            // Polygon.cpp:241 — l.a = l.b;
            l.a = l.b;
        }
        // Polygon.cpp:243
        intersections.len() > intersections_size
    }

    /// Polygon.cpp:245 — `bool Polygon::overlaps(const Polygons& other) const`
    pub fn overlaps(&self, other: &Polygons) -> bool {
        // Polygon.cpp:247-248
        if self.is_empty() || other.is_empty() {
            return false;
        }
        // Polygon.cpp:249 — Polylines pl_out = intersection_pl(to_polylines(other), *this);
        // FIDELITY-NOTE(F1): geo-clipper approximation vs C++ ClipperLib (intersection_pl).
        let pl_out = crate::clipper_utils::intersection_pl(
            &to_polylines(other),
            &[crate::geometry::ExPolygon::new(self.clone())],
        );

        // Polygon.cpp:253-255
        // See unit test SCENARIO("Clipper diff with polyline", "[Clipper]")
        // for in which case the intersection_pl produces any intersection.
        !pl_out.is_empty() ||
            // If *this is completely inside other, then pl_out is empty, but the expolygons overlap.
            other.iter().any(|poly| poly.contains(&self.points[0]))
    }

    /// Considering CCW orientation of this polygon, find all convex points
    /// with the angle at the vertex larger than a threshold.
    /// Polygon.cpp:299 — `Points Polygon::convex_points(double angle_threshold) const`
    pub fn convex_points(&self, angle_threshold: CoordF) -> Vec<Point> {
        // Polygon.cpp:301 — filter cross2(v1, v2) > 0.
        filter_convex_concave_points_by_angle_threshold(&self.points, angle_threshold, |v1, v2| {
            cross2f(v1, v2) > 0.0
        })
    }

    /// Considering CCW orientation of this polygon, find all concave points
    /// with the angle at the vertex larger than a threshold.
    /// Polygon.cpp:304 — `Points Polygon::concave_points(double angle_threshold) const`
    pub fn concave_points(&self, angle_threshold: CoordF) -> Vec<Point> {
        // Polygon.cpp:306 — filter cross2(v1, v2) < 0.
        filter_convex_concave_points_by_angle_threshold(&self.points, angle_threshold, |v1, v2| {
            cross2f(v1, v2) < 0.0
        })
    }

    /// Projection of a point onto the polygon.
    /// Polygon.cpp:310 — `Point Polygon::point_projection(const Point &point) const`
    pub fn point_projection(&self, point: &Point) -> Point {
        // Polygon.cpp:312-313
        let mut proj = *point;
        let mut dmin = f64::MAX;
        // Polygon.cpp:314
        if !self.points.is_empty() {
            // Polygon.cpp:315
            for i in 0..self.points.len() {
                // Polygon.cpp:316-317
                let pt0 = self.points[i];
                let pt1 = self.points[if i + 1 == self.points.len() { 0 } else { i + 1 }];
                // Polygon.cpp:318-322
                let mut d = point.distance(&pt0);
                if d < dmin {
                    dmin = d;
                    proj = pt0;
                }
                // Polygon.cpp:323-327
                d = point.distance(&pt1);
                if d < dmin {
                    dmin = d;
                    proj = pt1;
                }
                // Polygon.cpp:328 — Vec2d v1(coordf_t(pt1(0)-pt0(0)), coordf_t(pt1(1)-pt0(1)))
                let v1 = PointF::new((pt1.x - pt0.x) as CoordF, (pt1.y - pt0.y) as CoordF);
                // Polygon.cpp:329 — coordf_t div = v1.squaredNorm();
                let div = v1.length_squared();
                // Polygon.cpp:330
                if div > 0.0 {
                    // Polygon.cpp:331
                    let v2 = PointF::new((point.x - pt0.x) as CoordF, (point.y - pt0.y) as CoordF);
                    // Polygon.cpp:332 — coordf_t t = v1.dot(v2) / div;
                    let t = v1.dot(&v2) / div;
                    // Polygon.cpp:333
                    if t > 0.0 && t < 1.0 {
                        // Polygon.cpp:334
                        // Point foot(coord_t(floor(pt0(0) + t*v1(0) + 0.5)), coord_t(floor(pt0(1) + t*v1(1) + 0.5)))
                        let foot = Point::new(
                            (pt0.x as CoordF + t * v1.x + 0.5).floor() as Coord,
                            (pt0.y as CoordF + t * v1.y + 0.5).floor() as Coord,
                        );
                        // Polygon.cpp:335
                        d = point.distance(&foot);
                        // Polygon.cpp:336-339
                        if d < dmin {
                            dmin = d;
                            proj = foot;
                        }
                    }
                }
            }
        }
        // Polygon.cpp:344
        proj
    }

    /// Polygon.cpp:347 — `std::vector<float> Polygon::parameter_by_length() const`
    /// Parametrize the polygon by its length. Returns size = points.size()+1.
    pub fn parameter_by_length(&self) -> Vec<f32> {
        // Polygon.cpp:350 — std::vector<float> lengths(points.size()+1, 0.);
        let mut lengths = vec![0.0_f32; self.points.len() + 1];
        // Polygon.cpp:351-352
        for i in 1..self.points.len() {
            // lengths[i] = lengths[i-1] + (points[i] - points[i-1]).cast<float>().norm();
            let dx = (self.points[i].x - self.points[i - 1].x) as f32;
            let dy = (self.points[i].y - self.points[i - 1].y) as f32;
            lengths[i] = lengths[i - 1] + (dx * dx + dy * dy).sqrt();
        }
        // Polygon.cpp:353 — lengths.back() = lengths[size-2] + (front - back).cast<float>().norm();
        if !self.points.is_empty() {
            let n = lengths.len();
            let dx = (self.points[0].x - self.points[self.points.len() - 1].x) as f32;
            let dy = (self.points[0].y - self.points[self.points.len() - 1].y) as f32;
            lengths[n - 1] = lengths[n - 2] + (dx * dx + dy * dy).sqrt();
        }
        // Polygon.cpp:354
        lengths
    }

    /// Polygon.cpp:357 — `void Polygon::densify(float min_length, std::vector<float>* lengths_ptr)`
    pub fn densify(&mut self, min_length: f32, lengths_ptr: Option<&mut Vec<f32>>) {
        // Polygon.cpp:359-365
        let mut lengths_local: Vec<f32>;
        let lengths: &mut Vec<f32> = match lengths_ptr {
            Some(l) => l,
            None => {
                // Length parametrization has not been provided. Calculate our own.
                lengths_local = self.parameter_by_length();
                &mut lengths_local
            }
        };

        // Polygon.cpp:367 — assert(points.size() == lengths.size() - 1);
        debug_assert_eq!(self.points.len(), lengths.len() - 1);

        // Polygon.cpp:369
        let mut j = 1;
        while j <= self.points.len() {
            // Polygon.cpp:370-371
            let last = j == self.points.len();
            let i = if last { 0 } else { j };

            // Polygon.cpp:373
            if lengths[j] - lengths[j - 1] > min_length {
                // Polygon.cpp:374 — Point diff = points[i] - points[j-1];
                let diff = self.points[i] - self.points[j - 1];
                // Polygon.cpp:375 — float diff_len = lengths[j] - lengths[j-1];
                let diff_len = lengths[j] - lengths[j - 1];
                // Polygon.cpp:376 — float r = (min_length/diff_len);
                let r = min_length / diff_len;
                // Polygon.cpp:377 — Point new_pt = points[j-1] + Point(r*diff[0], r*diff[1]);
                // Point(float,float) -> coord_t cast (truncation toward zero).
                let new_pt = self.points[j - 1]
                    + Point::new((r * diff.x as f32) as Coord, (r * diff.y as f32) as Coord);
                // Polygon.cpp:378
                self.points.insert(j, new_pt);
                // Polygon.cpp:379
                lengths.insert(j, lengths[j - 1] + min_length);
            }
            j += 1;
        }
        // Polygon.cpp:382 — assert(points.size() == lengths.size() - 1);
        debug_assert_eq!(self.points.len(), lengths.len() - 1);
    }

    /// BBS
    /// Polygon.cpp:385 — `Polygon Polygon::transform(const Transform3d& trafo) const`
    pub fn transform(&self, trafo: &super::Transform3D) -> Polygon {
        // Polygon.cpp:387-389
        let vertices_count = self.points.len();
        let mut dstpoly = Polygon::new();
        dstpoly.points.resize(vertices_count, Point::new(0, 0));
        // Polygon.cpp:390-391
        if vertices_count == 0 {
            return dstpoly;
        }

        // Polygon.cpp:395-407 — src column = {x, y, 0}; dst = trafo * src.homogeneous();
        // For a 4x4 affine with z=0: dst(0)=m00*x+m01*y+m03, dst(1)=m10*x+m11*y+m13.
        for i in 0..vertices_count {
            let x = self.points[i].x as CoordF;
            let y = self.points[i].y as CoordF;
            let dx = trafo.get(0, 0) * x + trafo.get(0, 1) * y + trafo.get(0, 3);
            let dy = trafo.get(1, 0) * x + trafo.get(1, 1) * y + trafo.get(1, 3);
            // Polygon.cpp:406 — dstpoly.points[i] = { dst(0,i), dst(1,i) };
            // Point{double,double} -> coord_t cast (truncation toward zero).
            dstpoly.points[i] = Point::new(dx as Coord, dy as Coord);
        }
        // Polygon.cpp:408
        dstpoly
    }

    /// Polygon.hpp:61 — `bool is_valid() const { return this->points.size() >= 3; }`
    #[inline]
    pub fn is_valid(&self) -> bool {
        self.points.len() >= 3
    }

    /// (crate helper) Bounding box of the polygon.
    #[inline]
    pub fn bounding_box(&self) -> BoundingBox {
        BoundingBox::from_points(&self.points)
    }

    /// (crate helper) Convert to a polyline (open path, no repeated point).
    pub fn to_polyline(&self) -> Polyline {
        Polyline::from_points(self.points.clone())
    }

    // ------------------------------------------------------------------
    // Crate-extension helpers (not in C++ Polygon, but relied upon by
    // existing crate consumers). Kept here so the canonical Polygon stays
    // a drop-in for the rest of the crate while the C++ surface above is
    // faithful. These are NOT line-for-line ports.
    // ------------------------------------------------------------------

    /// (crate helper) Edge from point[i] to point[i+1] (wrapping).
    #[inline]
    pub fn edge(&self, index: usize) -> Line {
        let len = self.points.len();
        Line::new(self.points[index % len], self.points[(index + 1) % len])
    }

    /// (crate helper) All edges (closing edge included), matching `to_lines(*this)`.
    pub fn edges(&self) -> Vec<Line> {
        if self.points.len() < 2 {
            return Vec::new();
        }
        let mut edges = Vec::with_capacity(self.points.len());
        for i in 0..self.points.len() {
            edges.push(self.edge(i));
        }
        edges
    }

    /// (crate helper) Number of edges.
    #[inline]
    pub fn edge_count(&self) -> usize {
        if self.points.len() < 2 {
            0
        } else {
            self.points.len()
        }
    }

    /// (crate helper) Point at index, wrapping.
    #[inline]
    pub fn point_at(&self, index: usize) -> Point {
        self.points[index % self.points.len()]
    }

    /// (crate helper) Total edge length including the closing edge.
    pub fn perimeter(&self) -> CoordF {
        if self.points.len() < 2 {
            return 0.0;
        }
        let mut total = 0.0;
        for i in 0..self.points.len() {
            total += self.edge(i).length();
        }
        total
    }

    /// (crate helper) Scale every point about the origin.
    pub fn scale(&mut self, factor: CoordF) {
        for p in &mut self.points {
            *p = *p * factor;
        }
    }

    /// (crate helper) Return a scaled copy.
    pub fn scaled(&self, factor: CoordF) -> Self {
        let mut result = self.clone();
        result.scale(factor);
        result
    }

    /// (crate helper) Closest point on the boundary.
    pub fn closest_point(&self, p: &Point) -> Point {
        if self.points.is_empty() {
            return Point::new(0, 0);
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

    /// (crate helper) Distance from a point to the boundary.
    pub fn distance_to_point(&self, p: &Point) -> CoordF {
        let closest = self.closest_point(p);
        p.distance(&closest)
    }

    /// (crate helper) Whether a point lies on the boundary within `tolerance`.
    pub fn is_point_on_boundary(&self, p: &Point, tolerance: Coord) -> bool {
        for edge in self.edges() {
            if edge.contains_point(p, tolerance) {
                return true;
            }
        }
        false
    }

    /// (crate helper) Convert to a closed polyline (first point repeated at end).
    pub fn to_closed_polyline(&self) -> Polyline {
        let mut points = self.points.clone();
        if !points.is_empty() {
            points.push(points[0]);
        }
        Polyline::from_points(points)
    }

    /// (crate helper) Create a regular n-gon centered at origin.
    pub fn regular(n: usize, radius: Coord) -> Self {
        if n < 3 {
            return Self::new();
        }
        let mut points = Vec::with_capacity(n);
        for i in 0..n {
            let angle = 2.0 * std::f64::consts::PI * i as CoordF / n as CoordF;
            points.push(Point::new(
                (radius as CoordF * angle.cos()).round() as Coord,
                (radius as CoordF * angle.sin()).round() as Coord,
            ));
        }
        Self::from_points(points)
    }

    /// (crate helper) Circle approximation with `segments` sides centered at `center`.
    pub fn circle(center: Point, radius: Coord, segments: usize) -> Self {
        let mut poly = Self::regular(segments, radius);
        poly.translate(center);
        poly
    }

    /// (crate helper) Axis-aligned rectangle polygon (CCW).
    pub fn rectangle(min: Point, max: Point) -> Self {
        Self::from_points(vec![
            min,
            Point::new(max.x, min.y),
            max,
            Point::new(min.x, max.y),
        ])
    }

    /// (crate helper) Square centered at `center` with given half size.
    pub fn square(center: Point, half_size: Coord) -> Self {
        Self::rectangle(
            Point::new(center.x - half_size, center.y - half_size),
            Point::new(center.x + half_size, center.y + half_size),
        )
    }

    /// (crate helper) In-place simplification removing duplicate/collinear points
    /// within `tolerance` (scaled units). Used by `ExPolygon::simplify`.
    pub fn simplify_in_place(&mut self, tolerance: Coord) {
        if self.points.len() < 3 {
            return;
        }
        let mut new_points = Vec::with_capacity(self.points.len());
        let mut prev_idx = self.points.len() - 1;
        for i in 0..self.points.len() {
            let next_idx = (i + 1) % self.points.len();
            if self.points[i].coincides_with(&self.points[next_idx], tolerance) {
                continue;
            }
            let prev = self.points[prev_idx];
            let curr = self.points[i];
            let next = self.points[next_idx];
            let line = Line::new(prev, next);
            let dist = line.distance_to_point(&curr);
            if dist > tolerance as CoordF {
                new_points.push(curr);
            }
            prev_idx = i;
        }
        self.points = new_points;
    }
}

// ----------------------------------------------------------------------------
// Free functions over the point sequence (filtering helpers).
// ----------------------------------------------------------------------------

/// Filter points from poly to the output with the help of FilterFn.
/// filter function receives two vectors:
///   v1: this_point - previous_point
///   v2: next_point - this_point
/// and returns true if the point is to be copied to the output.
/// Polygon.cpp:262 — `template<typename FilterFn> Points filter_points_by_vectors(...)`
fn filter_points_by_vectors<F: Fn(PointF, PointF) -> bool>(poly: &[Point], filter: F) -> Vec<Point> {
    // Polygon.cpp:266 — Point p1 = poly.back();
    let mut p1 = poly[poly.len() - 1];
    // Polygon.cpp:268 — Vec2d v1 = (p1 - *(poly.end() - 2)).cast<double>();
    let prev = poly[poly.len() - 2];
    let mut v1 = PointF::new((p1.x - prev.x) as CoordF, (p1.y - prev.y) as CoordF);

    // Polygon.cpp:270
    let mut out: Vec<Point> = Vec::new();
    // Polygon.cpp:271
    for &p2 in poly {
        // Polygon.cpp:273 — Vec2d v2 = (p2 - p1).cast<double>();
        let v2 = PointF::new((p2.x - p1.x) as CoordF, (p2.y - p1.y) as CoordF);
        // Polygon.cpp:274-275
        if filter(v1, v2) {
            out.push(p1);
        }
        // Polygon.cpp:276-277
        v1 = v2;
        p1 = p2;
    }

    // Polygon.cpp:280
    out
}

/// Polygon.cpp:283 — `template<...> Points filter_convex_concave_points_by_angle_threshold(...)`
fn filter_convex_concave_points_by_angle_threshold<F: Fn(PointF, PointF) -> bool>(
    poly: &[Point],
    angle_threshold: CoordF,
    convex_concave_filter: F,
) -> Vec<Point> {
    // Polygon.cpp:286 — assert(angle_threshold >= 0.);
    debug_assert!(angle_threshold >= 0.0);
    // Polygon.cpp:287
    if angle_threshold > crate::libslic3r::EPSILON {
        // Polygon.cpp:288 — double cos_angle = cos(angle_threshold);
        let cos_angle = angle_threshold.cos();
        // Polygon.cpp:289-291
        filter_points_by_vectors(poly, |v1, v2| {
            convex_concave_filter(v1, v2) && v1.normalize().dot(&v2.normalize()) < cos_angle
        })
    } else {
        // Polygon.cpp:292-295
        filter_points_by_vectors(poly, |v1, v2| convex_concave_filter(v1, v2))
    }
}

// ----------------------------------------------------------------------------
// ClipperLib helpers (faithful semantics for the functions Polygon.cpp uses).
// ----------------------------------------------------------------------------

/// ClipperLib::Orientation(path) — returns Area(path) >= 0.
/// clipper.cpp: `bool Orientation(const Path &poly) { return Area(poly) >= 0; }`
fn clipper_orientation(points: &[Point]) -> bool {
    clipper_area(points) >= 0.0
}

/// ClipperLib::Area(path) — signed area (same convention as Polygon::area).
/// clipper.cpp: `double Area(const Path &poly)`
fn clipper_area(points: &[Point]) -> CoordF {
    // clipper.cpp: int size = poly.size(); if (size < 3) return 0;
    let size = points.len();
    if size < 3 {
        return 0.0;
    }
    // double a = 0; for (j=size-1,i=0; i<size; ++i) { a += (xj+xi)*(yj-yi); j=i; }
    let mut a = 0.0_f64;
    let mut j = size - 1;
    for i in 0..size {
        a += (points[j].x as CoordF + points[i].x as CoordF)
            * (points[j].y as CoordF - points[i].y as CoordF);
        j = i;
    }
    // return -a * 0.5;
    -a * 0.5
}

/// ClipperLib::PointInPolygon(pt, path) — returns 0 (outside), 1 (inside), -1 (on boundary).
/// clipper.cpp:4793 — `int PointInPolygon(const IntPoint &pt, const Path &path)`
fn point_in_polygon(pt: &Point, path: &[Point]) -> i32 {
    // clipper.cpp:4793
    let mut result = 0i32;
    // clipper.cpp:4794
    let cnt = path.len();
    // clipper.cpp:4795
    if cnt < 3 {
        return 0;
    }
    // clipper.cpp:4796
    let mut ip = path[0];
    // clipper.cpp:4797
    for i in 1..=cnt {
        // clipper.cpp:4799
        let ip_next = if i == cnt { path[0] } else { path[i] };
        // clipper.cpp:4800-4801
        if ip_next.y == pt.y
            && ((ip_next.x == pt.x) || (ip.y == pt.y && ((ip_next.x > pt.x) == (ip.x < pt.x))))
        {
            return -1;
        }
        // clipper.cpp:4802
        if (ip.y < pt.y) != (ip_next.y < pt.y) {
            // clipper.cpp:4804
            if ip.x >= pt.x {
                // clipper.cpp:4805-4806
                if ip_next.x > pt.x {
                    result = 1 - result;
                } else {
                    // clipper.cpp:4808-4811
                    let d = (ip.x as f64 - pt.x as f64) * (ip_next.y as f64 - pt.y as f64)
                        - (ip_next.x as f64 - pt.x as f64) * (ip.y as f64 - pt.y as f64);
                    if d == 0.0 {
                        return -1;
                    }
                    if (d > 0.0) == (ip_next.y > ip.y) {
                        result = 1 - result;
                    }
                }
            } else {
                // clipper.cpp:4813-4814
                if ip_next.x > pt.x {
                    // clipper.cpp:4815-4817
                    let d = (ip.x as f64 - pt.x as f64) * (ip_next.y as f64 - pt.y as f64)
                        - (ip_next.x as f64 - pt.x as f64) * (ip.y as f64 - pt.y as f64);
                    if d == 0.0 {
                        return -1;
                    }
                    if (d > 0.0) == (ip_next.y > ip.y) {
                        result = 1 - result;
                    }
                }
            }
        }
        // clipper.cpp:4822
        ip = ip_next;
    }
    // clipper.cpp:4824
    result
}

// ----------------------------------------------------------------------------
// Free functions (Polygon.cpp), in source order.
// ----------------------------------------------------------------------------

/// Polygon.cpp:411 — `BoundingBox get_extents(const Polygon &poly)`
pub fn get_extents(poly: &Polygon) -> BoundingBox {
    // Polygon.cpp:413 — return poly.bounding_box();
    poly.bounding_box()
}

/// Polygon.cpp:416 — `BoundingBox get_extents(const Polygons &polygons)`
pub fn get_extents_polygons(polygons: &[Polygon]) -> BoundingBox {
    // Polygon.cpp:418
    let mut bb = BoundingBox::new();
    // Polygon.cpp:419
    if !polygons.is_empty() {
        // Polygon.cpp:420
        bb = get_extents(&polygons[0]);
        // Polygon.cpp:421-422
        for i in 1..polygons.len() {
            bb.merge(&get_extents(&polygons[i]));
        }
    }
    // Polygon.cpp:424
    bb
}

/// Polygon.cpp:427 — `BoundingBox get_extents_rotated(const Polygon &poly, double angle)`
pub fn get_extents_rotated(poly: &Polygon, angle: CoordF) -> BoundingBox {
    // Polygon.cpp:429 — return get_extents_rotated(poly.points, angle);
    get_extents_rotated_points(&poly.points, angle)
}

/// Polygon.cpp:432 — `BoundingBox get_extents_rotated(const Polygons &polygons, double angle)`
pub fn get_extents_rotated_polygons(polygons: &[Polygon], angle: CoordF) -> BoundingBox {
    // Polygon.cpp:434
    let mut bb = BoundingBox::new();
    // Polygon.cpp:435
    if !polygons.is_empty() {
        // Polygon.cpp:436
        bb = get_extents_rotated_points(&polygons[0].points, angle);
        // Polygon.cpp:437-438
        for i in 1..polygons.len() {
            bb.merge(&get_extents_rotated_points(&polygons[i].points, angle));
        }
    }
    // Polygon.cpp:440
    bb
}

/// BoundingBox of points rotated by `angle` about the origin.
/// `get_extents_rotated(const Points &points, double angle)` (Point.cpp helper).
fn get_extents_rotated_points(points: &[Point], angle: CoordF) -> BoundingBox {
    let mut bb = BoundingBox::new();
    if !points.is_empty() {
        let cos_angle = angle.cos();
        let sin_angle = angle.sin();
        let rot = |p: &Point| -> Point {
            Point::new(
                (cos_angle * p.x as CoordF - sin_angle * p.y as CoordF).round() as Coord,
                (sin_angle * p.x as CoordF + cos_angle * p.y as CoordF).round() as Coord,
            )
        };
        let mut it = points.iter();
        let first = rot(it.next().unwrap());
        bb = BoundingBox::from_points_minmax(first, first);
        for p in it {
            bb.merge_point(rot(p));
        }
    }
    bb
}

/// Polygon.cpp:443 — `std::vector<BoundingBox> get_extents_vector(const Polygons &polygons)`
pub fn get_extents_vector(polygons: &[Polygon]) -> Vec<BoundingBox> {
    // Polygon.cpp:445-446
    let mut out: Vec<BoundingBox> = Vec::new();
    out.reserve(polygons.len());
    // Polygon.cpp:447-448
    for poly in polygons {
        out.push(get_extents(poly));
    }
    // Polygon.cpp:449
    out
}

/// Polygon must be valid (at least three points), collinear points and duplicate points removed.
/// Polygon.cpp:453 — `bool polygon_is_convex(const Points &poly)`
pub fn polygon_is_convex(poly: &[Point]) -> bool {
    // Polygon.cpp:455-456
    if poly.len() < 3 {
        return false;
    }
    // Polygon.cpp:458-459
    let mut p0 = poly[poly.len() - 2];
    let mut p1 = poly[poly.len() - 1];
    // Polygon.cpp:460
    for i in 0..poly.len() {
        // Polygon.cpp:461
        let p2 = poly[i];
        // Polygon.cpp:462 — cross2((p1-p0).cast<int64_t>(), (p2-p1).cast<int64_t>())
        let det = cross2(p1 - p0, p2 - p1);
        // Polygon.cpp:463-464
        if det < 0 {
            return false;
        }
        // Polygon.cpp:465-466
        p0 = p1;
        p1 = p2;
    }
    // Polygon.cpp:468
    true
}

/// Polygon.hpp:112 — `inline bool polygon_is_convex(const Polygon &poly) { return polygon_is_convex(poly.points); }`
#[inline]
pub fn polygon_is_convex_poly(poly: &Polygon) -> bool {
    polygon_is_convex(&poly.points)
}

/// Polygon.cpp:471 — `bool has_duplicate_points(const Polygons &polys)`
pub fn has_duplicate_points(polys: &[Polygon]) -> bool {
    // Polygon.cpp:473-482 (#if 1 — check globally)
    // Polygon.cpp:475-477
    let mut cnt = 0usize;
    for poly in polys {
        cnt += poly.points.len();
    }
    // Polygon.cpp:478-481
    let mut allpts: Vec<Point> = Vec::new();
    allpts.reserve(cnt);
    for poly in polys {
        allpts.extend_from_slice(&poly.points);
    }
    // Polygon.cpp:482 — return has_duplicate_points(std::move(allpts));
    super::has_duplicate_points(allpts)
}

/// Return True when erase some otherwise False.
/// Polygon.cpp:492 — `bool remove_same_neighbor(Polygon &polygon)`
pub fn remove_same_neighbor(polygon: &mut Polygon) -> bool {
    // Polygon.cpp:494-495
    let points = &mut polygon.points;
    if points.is_empty() {
        return false;
    }
    // Polygon.cpp:496 — auto last = std::unique(points.begin(), points.end());
    // std::unique collapses consecutive equal elements; `last` is the new logical end.
    let mut last = 0usize; // index of last written element
    for read in 1..points.len() {
        if points[read] != points[last] {
            last += 1;
            points[last] = points[read];
        }
    }
    let mut last = last + 1; // one-past-the-last logical end

    // Polygon.cpp:498-499 — remove first and last neighbor duplication
    if points[last - 1] == points[0] {
        last -= 1;
    }

    // Polygon.cpp:501-502 — no duplicits
    if last == points.len() {
        return false;
    }

    // Polygon.cpp:504 — points.erase(last, points.end());
    points.truncate(last);
    // Polygon.cpp:505
    true
}

/// Polygon.cpp:508 — `bool remove_same_neighbor(Polygons &polygons)`
pub fn remove_same_neighbor_polygons(polygons: &mut Polygons) -> bool {
    // Polygon.cpp:510
    if polygons.is_empty() {
        return false;
    }
    // Polygon.cpp:511-512
    let mut exist = false;
    for polygon in polygons.iter_mut() {
        exist |= remove_same_neighbor(polygon);
    }
    // Polygon.cpp:514 — remove empty polygons (size <= 2)
    polygons.retain(|p| p.points.len() > 2);
    // Polygon.cpp:515
    exist
}

/// Polygon.cpp:518 — `static inline bool is_stick(const Point &p1, const Point &p2, const Point &p3)`
fn is_stick(p1: &Point, p2: &Point, p3: &Point) -> bool {
    // Polygon.cpp:520-521
    let v1 = *p2 - *p1;
    let v2 = *p3 - *p2;
    // Polygon.cpp:522 — int64_t dir = v1(0)*v2(0) + v1(1)*v2(1);
    let dir = v1.x as i64 * v2.x as i64 + v1.y as i64 * v2.y as i64;
    // Polygon.cpp:523-525
    if dir > 0 {
        // p3 does not turn back to p1. Do not remove p2.
        return false;
    }
    // Polygon.cpp:526-527
    let l2_1 = v1.x as f64 * v1.x as f64 + v1.y as f64 * v1.y as f64;
    let l2_2 = v2.x as f64 * v2.x as f64 + v2.y as f64 * v2.y as f64;
    // Polygon.cpp:528-531
    if dir == 0 {
        // p1, p2, p3 may make a perpendicular corner, or there is a zero edge length.
        // Remove p2 if it is coincident with p1 or p2.
        return l2_1 == 0.0 || l2_2 == 0.0;
    }
    // Polygon.cpp:532-536 — collinearity test via distance to the longer segment
    let cross = v1.x as f64 * v2.y as f64 - v2.x as f64 * v1.y as f64;
    let dist2 = cross * cross / l2_1.max(l2_2);
    // Polygon.cpp:537 — return dist2 < EPSILON * EPSILON;
    dist2 < crate::libslic3r::EPSILON * crate::libslic3r::EPSILON
}

/// Remove sticks (tentacles with zero area) from the polygon.
/// Polygon.cpp:540 — `bool remove_sticks(Polygon &poly)`
pub fn remove_sticks(poly: &mut Polygon) -> bool {
    // Polygon.cpp:542
    let mut modified = false;
    // Polygon.cpp:543
    let mut j = 1usize;
    // Polygon.cpp:544
    let mut i = 1usize;
    while i + 1 < poly.points.len() {
        // Polygon.cpp:545
        if !is_stick(&poly.points[j - 1], &poly.points[i], &poly.points[i + 1]) {
            // Keep the point.
            // Polygon.cpp:547-548
            if j < i {
                poly.points[j] = poly.points[i];
            }
            // Polygon.cpp:549
            j += 1;
        }
        i += 1;
    }
    // Polygon.cpp:552 — if (++ j < poly.points.size())
    j += 1;
    if j < poly.points.len() {
        // Polygon.cpp:553-554
        poly.points[j - 1] = poly.points[poly.points.len() - 1];
        poly.points.truncate(j);
        // Polygon.cpp:555
        modified = true;
    }
    // Polygon.cpp:557-560
    while poly.points.len() >= 3
        && is_stick(
            &poly.points[poly.points.len() - 2],
            &poly.points[poly.points.len() - 1],
            &poly.points[0],
        )
    {
        poly.points.pop();
        modified = true;
    }
    // Polygon.cpp:561-562
    while poly.points.len() >= 3
        && is_stick(
            &poly.points[poly.points.len() - 1],
            &poly.points[0],
            &poly.points[1],
        )
    {
        poly.points.remove(0);
    }
    // Polygon.cpp:563
    modified
}

/// Polygon.cpp:566 — `bool remove_sticks(Polygons &polys)`
pub fn remove_sticks_polygons(polys: &mut Polygons) -> bool {
    // Polygon.cpp:568-569
    let mut modified = false;
    let mut j = 0usize;
    // Polygon.cpp:570
    for i in 0..polys.len() {
        // Polygon.cpp:571
        modified |= remove_sticks(&mut polys[i]);
        // Polygon.cpp:572-576
        if polys[i].points.len() >= 3 {
            if j < i {
                polys.swap(i, j);
            }
            j += 1;
        }
    }
    // Polygon.cpp:578-579
    if j < polys.len() {
        polys.truncate(j);
    }
    // Polygon.cpp:580
    modified
}

/// Remove polygons with less than 3 edges.
/// Polygon.cpp:583 — `bool remove_degenerate(Polygons &polys)`
pub fn remove_degenerate(polys: &mut Polygons) -> bool {
    // Polygon.cpp:585-586
    let mut modified = false;
    let mut j = 0usize;
    // Polygon.cpp:587
    for i in 0..polys.len() {
        // Polygon.cpp:588-594
        if polys[i].points.len() >= 3 {
            if j < i {
                polys.swap(i, j);
            }
            j += 1;
        } else {
            modified = true;
        }
    }
    // Polygon.cpp:595-596
    if j < polys.len() {
        polys.truncate(j);
    }
    // Polygon.cpp:597
    modified
}

/// Polygon.cpp:600 — `bool remove_small(Polygons &polys, double min_area)`
pub fn remove_small(polys: &mut Polygons, min_area: CoordF) -> bool {
    // Polygon.cpp:602-603
    let mut modified = false;
    let mut j = 0usize;
    // Polygon.cpp:604
    for i in 0..polys.len() {
        // Polygon.cpp:605 — std::abs(polys[i].area()) >= min_area
        if polys[i].area().abs() >= min_area {
            if j < i {
                polys.swap(i, j);
            }
            j += 1;
        } else {
            modified = true;
        }
    }
    // Polygon.cpp:612-613
    if j < polys.len() {
        polys.truncate(j);
    }
    // Polygon.cpp:614
    modified
}

/// Polygon.cpp:617 — `void remove_collinear(Polygon &poly)`
pub fn remove_collinear(poly: &mut Polygon) {
    // Polygon.cpp:619
    if poly.points.len() > 2 {
        // Polygon.cpp:621-625 — copy points and append both first and last in place
        let mut pp: Vec<Point> = Vec::new();
        pp.reserve(poly.points.len() + 2);
        pp.push(poly.points[poly.points.len() - 1]);
        // pp.insert(pp.begin()+1, poly.points.begin(), poly.points.end());
        for p in &poly.points {
            pp.push(*p);
        }
        pp.push(poly.points[0]);
        // Polygon.cpp:627 — delete old points vector. Will be re-filled in the loop
        poly.points.clear();

        // Polygon.cpp:629-630
        let mut i = 0usize;
        let mut k;
        // Polygon.cpp:631
        while i < pp.len() - 2 {
            // Polygon.cpp:632
            k = i + 1;
            // Polygon.cpp:633
            let p1 = pp[i];
            // Polygon.cpp:634
            while k < pp.len() - 1 {
                // Polygon.cpp:635-636
                let p2 = pp[k];
                let p3 = pp[k + 1];
                // Polygon.cpp:637
                let l = Line::new(p1, p3);
                // Polygon.cpp:638 — if (l.distance_to(p2) < SCALED_EPSILON)
                if l.distance_to_point(&p2) < crate::libslic3r::SCALED_EPSILON {
                    // Polygon.cpp:639
                    k += 1;
                } else {
                    // Polygon.cpp:641 — if (i > 0) poly.points.push_back(p1);
                    if i > 0 {
                        poly.points.push(p1);
                    }
                    // Polygon.cpp:642-643
                    i = k;
                    break;
                }
            }
            // Polygon.cpp:646 — if (k > pp.size()-2) break;
            if k > pp.len() - 2 {
                break;
            }
        }
        // Polygon.cpp:648 — poly.points.push_back(pp[i]);
        poly.points.push(pp[i]);
    }
}

/// Polygon.cpp:652 — `void remove_collinear(Polygons &polys)`
pub fn remove_collinear_polygons(polys: &mut Polygons) {
    // Polygon.cpp:654-655
    for poly in polys.iter_mut() {
        remove_collinear(poly);
    }
}

/// Polygon.cpp:658 — `Polygons polygons_simplify(const Polygons &source_polygons, double tolerance)`
pub fn polygons_simplify(source_polygons: &[Polygon], tolerance: CoordF) -> Polygons {
    // Polygon.cpp:660-661
    let mut out: Polygons = Vec::new();
    out.reserve(source_polygons.len());
    // Polygon.cpp:662
    for source_polygon in source_polygons {
        // Polygon.cpp:664 — _douglas_peucker(to_polyline(source_polygon).points, tolerance)
        let mut simplified = crate::multi_point::douglas_peucker(&to_polyline(source_polygon).points, tolerance);
        // Polygon.cpp:666 — then remove the last (repeated) point.
        simplified.pop();
        // Polygon.cpp:668 — bool ccw = ClipperLib::Area(simplified) > 0.;
        let ccw = clipper_area(&simplified) > 0.0;
        // Polygon.cpp:669 — for (path : SimplifyPolygons(SinglePathProvider(simplified), pftNonZero))
        for mut path in clipper_simplify_polygons_single_path(&simplified) {
            // Polygon.cpp:670-672
            if !ccw {
                // ClipperLib likely reoriented negative area contours to become positive.
                // Reverse holes back to CW.
                path.reverse();
            }
            // Polygon.cpp:673
            out.push(Polygon::from_points(path));
        }
    }
    // Polygon.cpp:676
    out
}

/// Do polygons match? If they match, they must have the same topology,
/// however their contours may be rotated.
/// Polygon.cpp:681 — `bool polygons_match(const Polygon &l, const Polygon &r)`
pub fn polygons_match(l: &Polygon, r: &Polygon) -> bool {
    // Polygon.cpp:683-684
    if l.len() != r.len() {
        return false;
    }
    // Polygon.cpp:685 — auto it_l = std::find(l.points.begin(), l.points.end(), r.points.front());
    let mut idx_l = match l.points.iter().position(|p| *p == r.points[0]) {
        // Polygon.cpp:686-687
        None => return false,
        Some(idx) => idx,
    };
    // Polygon.cpp:688-691 — walk from it_l to end of l, in parallel with r from begin
    let mut idx_r = 0usize;
    while idx_l < l.points.len() {
        if l.points[idx_l] != r.points[idx_r] {
            return false;
        }
        idx_l += 1;
        idx_r += 1;
    }
    // Polygon.cpp:692-695 — wrap l back to begin, continue r
    idx_l = 0;
    while idx_r < r.points.len() {
        if l.points[idx_l] != r.points[idx_r] {
            return false;
        }
        idx_l += 1;
        idx_r += 1;
    }
    // Polygon.cpp:696
    true
}

/// Polygon.cpp:699 — `bool overlaps(const Polygons& polys1, const Polygons& polys2)`
pub fn overlaps(polys1: &[Polygon], polys2: &Polygons) -> bool {
    // Polygon.cpp:701-705
    for poly1 in polys1 {
        if poly1.overlaps(polys2) {
            return true;
        }
    }
    // Polygon.cpp:705
    false
}

/// Returns true if inside. Returns border_result if on boundary.
/// Polygon.cpp:708 — `bool contains(const Polygon &polygon, const Point &p, bool border_result)`
pub fn contains_polygon(polygon: &Polygon, p: &Point, border_result: bool) -> bool {
    // Polygon.cpp:710-714
    let poly_count_inside = point_in_polygon(p, &polygon.points);
    if poly_count_inside == -1 {
        border_result
    } else {
        (poly_count_inside % 2) == 1
    }
}

/// Returns true if inside. Returns border_result if on boundary.
/// Polygon.cpp:717 — `bool contains(const Polygons &polygons, const Point &p, bool border_result)`
pub fn contains_polygons(polygons: &[Polygon], p: &Point, border_result: bool) -> bool {
    // Polygon.cpp:719
    let mut poly_count_inside = 0i32;
    // Polygon.cpp:720
    for poly in polygons {
        // Polygon.cpp:721-724
        let is_inside_this_poly = point_in_polygon(p, &poly.points);
        if is_inside_this_poly == -1 {
            return border_result;
        }
        poly_count_inside += is_inside_this_poly;
    }
    // Polygon.cpp:726
    (poly_count_inside % 2) == 1
}

/// Polygon.cpp:729 — `Polygon make_circle(double radius, double error)`
pub fn make_circle(radius: CoordF, error: CoordF) -> Polygon {
    // Polygon.cpp:731 — double angle = 2. * acos(1. - error / radius);
    let angle = 2.0 * (1.0 - error / radius).acos();
    // Polygon.cpp:732 — size_t num_segments = size_t(ceil(2. * M_PI / angle));
    let num_segments = (2.0 * std::f64::consts::PI / angle).ceil() as usize;
    // Polygon.cpp:733
    make_circle_num_segments(radius, num_segments)
}

/// Polygon.cpp:736 — `Polygon make_circle_num_segments(double radius, size_t num_segments)`
pub fn make_circle_num_segments(radius: CoordF, num_segments: usize) -> Polygon {
    // Polygon.cpp:738-739
    let mut out = Polygon::new();
    out.points.reserve(num_segments);
    // Polygon.cpp:740 — double angle_inc = 2.0 * M_PI / num_segments;
    let angle_inc = 2.0 * std::f64::consts::PI / num_segments as CoordF;
    // Polygon.cpp:741
    for i in 0..num_segments {
        // Polygon.cpp:742 — const double angle = angle_inc * i;
        let angle = angle_inc * i as CoordF;
        // Polygon.cpp:743 — coord_t cast (truncation toward zero)
        out.points.push(Point::new(
            (angle.cos() * radius) as Coord,
            (angle.sin() * radius) as Coord,
        ));
    }
    // Polygon.cpp:745
    out
}

// ----------------------------------------------------------------------------
// Polygon.hpp inline free functions, in source order.
// ----------------------------------------------------------------------------

/// Polygon.hpp:123 — `inline double total_length(const Polygons &polylines)`
pub fn total_length(polylines: &[Polygon]) -> CoordF {
    // Polygon.hpp:124-127
    let mut total = 0.0_f64;
    for it in polylines {
        total += it.length();
    }
    total
}

/// Polygon.hpp:130 — `inline double area(const Polygon &poly) { return poly.area(); }`
#[inline]
pub fn area(poly: &Polygon) -> CoordF {
    poly.area()
}

/// Polygon.hpp:132 — `inline double area(const Polygons &polys)`
pub fn area_polygons(polys: &[Polygon]) -> CoordF {
    // Polygon.hpp:134-137
    let mut s = 0.0_f64;
    for p in polys {
        s += p.area();
    }
    s
}

/// Append a vector of polygons at the end of another vector of polygons.
/// Polygon.hpp:151 — `inline void polygons_append(Polygons &dst, const Polygons &src)`
#[inline]
pub fn polygons_append(dst: &mut Polygons, src: &[Polygon]) {
    dst.extend_from_slice(src);
}

/// Polygon.hpp:165 — `inline void polygons_rotate(Polygons &polys, double angle)`
pub fn polygons_rotate(polys: &mut Polygons, angle: CoordF) {
    // Polygon.hpp:167-170
    let cos_angle = angle.cos();
    let sin_angle = angle.sin();
    for p in polys.iter_mut() {
        p.rotate_by_cos_sin(cos_angle, sin_angle);
    }
}

/// Polygon.hpp:173 — `inline void polygons_reverse(Polygons &polys)`
pub fn polygons_reverse(polys: &mut Polygons) {
    // Polygon.hpp:175-176
    for p in polys.iter_mut() {
        p.reverse();
    }
}

/// Polygon.hpp:179 — `inline Points to_points(const Polygon &poly) { return poly.points; }`
#[inline]
pub fn to_points_poly(poly: &Polygon) -> Vec<Point> {
    poly.points.clone()
}

/// Polygon.hpp:184 — `inline size_t count_points(const Polygons &polys)`
pub fn count_points(polys: &[Polygon]) -> usize {
    // Polygon.hpp:185-187
    let mut n_points = 0usize;
    for poly in polys {
        n_points += poly.points.len();
    }
    n_points
}

/// Polygon.hpp:190 — `inline Points to_points(const Polygons &polys)`
pub fn to_points(polys: &[Polygon]) -> Vec<Point> {
    // Polygon.hpp:192-196
    let mut points: Vec<Point> = Vec::new();
    points.reserve(count_points(polys));
    for poly in polys {
        points.extend_from_slice(&poly.points);
    }
    points
}

/// Polygon.hpp:199 — `inline Lines to_lines(const Polygon &poly)`
pub fn to_lines(poly: &Polygon) -> Vec<Line> {
    // Polygon.hpp:201-202
    let mut lines: Vec<Line> = Vec::new();
    lines.reserve(poly.points.len());
    // Polygon.hpp:203
    if poly.points.len() > 2 {
        // Polygon.hpp:204-205
        for it in 0..poly.points.len() - 1 {
            lines.push(Line::new(poly.points[it], poly.points[it + 1]));
        }
        // Polygon.hpp:206
        lines.push(Line::new(
            poly.points[poly.points.len() - 1],
            poly.points[0],
        ));
    }
    // Polygon.hpp:208
    lines
}

/// Polygon.hpp:211 — `inline Lines to_lines(const Polygons &polys)`
pub fn to_lines_polygons(polys: &[Polygon]) -> Vec<Line> {
    // Polygon.hpp:213-214
    let mut lines: Vec<Line> = Vec::new();
    lines.reserve(count_points(polys));
    // Polygon.hpp:215
    for i in 0..polys.len() {
        // Polygon.hpp:216
        let poly = &polys[i];
        // Polygon.hpp:217-218
        for it in 0..poly.points.len() - 1 {
            lines.push(Line::new(poly.points[it], poly.points[it + 1]));
        }
        // Polygon.hpp:219
        lines.push(Line::new(
            poly.points[poly.points.len() - 1],
            poly.points[0],
        ));
    }
    // Polygon.hpp:221
    lines
}

/// Polygon.hpp:224 — `inline Polyline to_polyline(const Polygon &polygon)`
pub fn to_polyline(polygon: &Polygon) -> Polyline {
    // Polygon.hpp:226-229
    let mut out = Polyline::new();
    out.points.reserve(polygon.len() + 1);
    out.points.extend_from_slice(&polygon.points);
    out.points.push(polygon.points[0]);
    // Polygon.hpp:230
    out
}

/// Polygon.hpp:233 — `inline Polylines to_polylines(const Polygons &polygons)`
pub fn to_polylines(polygons: &[Polygon]) -> Vec<Polyline> {
    // Polygon.hpp:235-238
    let mut out: Vec<Polyline> = Vec::new();
    out.reserve(polygons.len());
    for polygon in polygons {
        out.push(to_polyline(polygon));
    }
    // Polygon.hpp:239
    out
}

/// close polyline to polygon (connect first and last point in polyline)
/// Polygon.hpp:257 — `inline Polygons to_polygons(const Polylines &polylines)`
pub fn to_polygons(polylines: &[Polyline]) -> Polygons {
    // Polygon.hpp:259-263
    let mut out: Polygons = Vec::new();
    out.reserve(polylines.len());
    for polyline in polylines {
        if !polyline.points.is_empty() {
            out.push(Polygon::from_points(polyline.points.clone()));
        }
    }
    out
}

/// Polygon.hpp:267 — `inline Polygons to_polygons(const std::vector<Points> &paths)`
pub fn to_polygons_paths(paths: &[Vec<Point>]) -> Polygons {
    // Polygon.hpp:269-272
    let mut out: Polygons = Vec::new();
    out.reserve(paths.len());
    for path in paths {
        out.push(Polygon::from_points(path.clone()));
    }
    out
}

// ----------------------------------------------------------------------------
// ClipperLib::SimplifyPolygons over a single open path (pftNonZero).
// Faithful to the C++ usage in polygons_simplify: removes self-intersections
// and merges into non-self-intersecting contours.
// ----------------------------------------------------------------------------

/// ClipperLib::SimplifyPolygons(SinglePathProvider(path), pftNonZero).
/// Implemented via the crate's clipper backend by unioning the single contour
/// with itself under the NonZero fill rule (which is what SimplifyPolygons does:
/// it cleans self-intersections by re-executing a union with the given fill rule).
/// ClipperUtils `simplify_polygons(const Polygons &subject, bool preserve_collinear=false)`.
/// ClipperUtils.cpp:1026 — executes a NonZero union of the subject paths, cleaning
/// self-intersections/overlaps. preserve_collinear=false (the default at the call site).
pub fn simplify_polygons_clipper(subject: &[Polygon]) -> Polygons {
    if subject.is_empty() {
        return Vec::new();
    }
    // FIDELITY-NOTE(F1): geo-clipper approximation vs C++ ClipperLib simplify_polygons.
    let unioned = crate::clipper_utils::union_polygons_ex(subject);
    let mut out: Polygons = Vec::new();
    for ex in unioned {
        out.push(ex.contour);
        for hole in ex.holes {
            out.push(hole);
        }
    }
    out
}

fn clipper_simplify_polygons_single_path(path: &[Point]) -> Vec<Vec<Point>> {
    if path.len() < 3 {
        // Degenerate: nothing to simplify.
        return if path.is_empty() {
            Vec::new()
        } else {
            vec![path.to_vec()]
        };
    }
    let subject = vec![Polygon::from_points(path.to_vec())];
    // The crate clipper backend exposes union over polygons returning ExPolygons.
    // ClipperLib::SimplifyPolygons returns a flat Polygons (outer + holes); flatten.
    // FIDELITY-NOTE(F1): geo-clipper approximation vs C++ ClipperLib SimplifyPolygons (pftNonZero).
    let unioned = crate::clipper_utils::union_polygons_ex(&subject);
    let mut out: Vec<Vec<Point>> = Vec::new();
    for ex in unioned {
        out.push(ex.contour.points);
        for hole in ex.holes {
            out.push(hole.points);
        }
    }
    out
}

// ----------------------------------------------------------------------------
// Trait impls + crate-extension type alias.
// ----------------------------------------------------------------------------

impl fmt::Debug for Polygon {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Polygon({} points)", self.points.len())
    }
}

impl fmt::Display for Polygon {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Polygon[")?;
        for (i, p) in self.points.iter().enumerate() {
            if i > 0 {
                write!(f, ", ")?;
            }
            write!(f, "{}", p)?;
        }
        write!(f, "]")
    }
}

impl Deref for Polygon {
    type Target = [Point];

    fn deref(&self) -> &Self::Target {
        &self.points
    }
}

impl DerefMut for Polygon {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.points
    }
}

impl Index<usize> for Polygon {
    type Output = Point;

    fn index(&self, index: usize) -> &Self::Output {
        &self.points[index]
    }
}

impl IndexMut<usize> for Polygon {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.points[index]
    }
}

impl FromIterator<Point> for Polygon {
    fn from_iter<I: IntoIterator<Item = Point>>(iter: I) -> Self {
        Self {
            points: iter.into_iter().collect(),
        }
    }
}

impl IntoIterator for Polygon {
    type Item = Point;
    type IntoIter = std::vec::IntoIter<Point>;

    fn into_iter(self) -> Self::IntoIter {
        self.points.into_iter()
    }
}

impl<'a> IntoIterator for &'a Polygon {
    type Item = &'a Point;
    type IntoIter = std::slice::Iter<'a, Point>;

    fn into_iter(self) -> Self::IntoIter {
        self.points.iter()
    }
}

impl<'a> IntoIterator for &'a mut Polygon {
    type Item = &'a mut Point;
    type IntoIter = std::slice::IterMut<'a, Point>;

    fn into_iter(self) -> Self::IntoIter {
        self.points.iter_mut()
    }
}

impl From<Vec<Point>> for Polygon {
    fn from(points: Vec<Point>) -> Self {
        Self::from_points(points)
    }
}

impl From<Polygon> for Vec<Point> {
    fn from(polygon: Polygon) -> Self {
        polygon.into_points()
    }
}

/// Polygon.hpp:14 — `using Polygons = std::vector<Polygon>;`
pub type Polygons = Vec<Polygon>;

#[cfg(test)]
mod tests {
    use super::*;

    fn make_square() -> Polygon {
        Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 100),
            Point::new(0, 100),
        ])
    }

    #[test]
    fn test_polygon_area() {
        let poly = make_square();
        assert!((poly.area() - 10000.0).abs() < 1.0);
    }

    #[test]
    fn test_polygon_is_counter_clockwise() {
        let poly = make_square();
        assert!(poly.is_counter_clockwise());
        assert!(!poly.is_clockwise());
        let cw = poly.reversed();
        assert!(cw.is_clockwise());
    }

    #[test]
    fn test_polygon_length() {
        let poly = make_square();
        assert!((poly.length() - 400.0).abs() < 1.0);
    }

    #[test]
    fn test_polygon_centroid() {
        let poly = make_square();
        let centroid = poly.centroid();
        assert_eq!(centroid.x, 50);
        assert_eq!(centroid.y, 50);
    }

    #[test]
    fn test_polygon_contains_point() {
        let poly = make_square();
        assert!(poly.contains_point(&Point::new(50, 50)));
        assert!(!poly.contains_point(&Point::new(-10, 50)));
        assert!(!poly.contains_point(&Point::new(110, 50)));
        // Boundary treated as inside (border_result = true).
        assert!(poly.contains_point(&Point::new(0, 0)));
        assert!(poly.contains_point(&Point::new(50, 0)));
        // border_result = false on boundary.
        assert!(!contains_polygon(&poly, &Point::new(50, 0), false));
    }

    #[test]
    fn test_polygon_is_convex() {
        let poly = make_square();
        // CCW square is convex per polygon_is_convex (det >= 0).
        assert!(polygon_is_convex(&poly.points));
    }

    #[test]
    fn test_make_circle_num_segments() {
        let c = make_circle_num_segments(1000.0, 8);
        assert_eq!(c.points.len(), 8);
    }

    #[test]
    fn test_remove_collinear() {
        let mut poly = Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(50, 0),
            Point::new(100, 0),
            Point::new(100, 100),
            Point::new(0, 100),
        ]);
        remove_collinear(&mut poly);
        // Collinear midpoint (50,0) removed.
        assert!(poly.points.len() <= 4);
    }
}
