//! MultiPoint - base class for polylines and polygons
//!
//! C++ Reference:
//! - MultiPoint.hpp (153 lines)
//! - MultiPoint.cpp (479 lines)
//!
//! C++ `MultiPoint` is an abstract base class; `Polyline` and `Polygon` are the
//! concrete subclasses that supply the virtual `lines()` / `last_point()`. Rust
//! has no inheritance, so the geometry-agnostic `MultiPoint` methods are ported
//! here as free functions over the point sequence (`&[Point]`). The methods that
//! depend on the virtual `lines()` (open polyline edges vs. closed polygon edges)
//! — `intersection`, `first_intersection`, `intersections` — live on the concrete
//! `Polygon`/`Polyline` types instead, and `get_extents_rotated`/`symmetric_y`
//! are hosted in `geometry::polygon` / `ex_polygon`.

use crate::geometry::{cross2f, BoundingBox, Line, Point};
use crate::Coord;

/// Douglas-Peucker line simplification algorithm
/// Reduces the number of points in a polyline while maintaining shape fidelity
/// MultiPoint.cpp:179 `MultiPoint::_douglas_peucker`
pub fn douglas_peucker(pts: &[Point], tolerance: f64) -> Vec<Point> {
    // MultiPoint.cpp:181
    let mut result_pts = Vec::new();

    // MultiPoint.cpp:182
    let tolerance_sq = tolerance * tolerance;

    // MultiPoint.cpp:183
    if pts.is_empty() {
        return result_pts;
    }

    // MultiPoint.cpp:184-187 — anchor = front, floater = back
    let anchor_idx = 0;
    let mut floater_idx = pts.len() - 1;

    // MultiPoint.cpp:188
    result_pts.reserve(pts.len());

    // MultiPoint.cpp:189 — always include the first point (anchor)
    result_pts.push(pts[anchor_idx]);

    // MultiPoint.cpp:190 — if (anchor_idx != floater_idx)
    if anchor_idx == floater_idx {
        return result_pts;
    }

    // MultiPoint.cpp:191
    // assert!(pts.len() > 1);

    // MultiPoint.cpp:192-194 — dpStack, seeded with floater_idx
    let mut dp_stack: Vec<usize> = Vec::new();
    dp_stack.reserve(pts.len());
    dp_stack.push(floater_idx);

    let mut anchor_idx = anchor_idx;

    // MultiPoint.cpp:195 — for (;;)
    loop {
        // MultiPoint.cpp:196-197
        let mut max_dist_sq = 0.0;
        let mut furthest_idx = anchor_idx;

        // MultiPoint.cpp:199 — find point furthest from segment (anchor, floater)
        for i in (anchor_idx + 1)..floater_idx {
            // MultiPoint.cpp:200
            let dist_sq = Line::distance_to_squared(pts[i], pts[anchor_idx], pts[floater_idx]);

            // MultiPoint.cpp:201-204
            if dist_sq > max_dist_sq {
                max_dist_sq = dist_sq;
                furthest_idx = i;
            }
        }

        // MultiPoint.cpp:207 — remove point if less than tolerance
        if max_dist_sq <= tolerance_sq {
            // MultiPoint.cpp:208
            result_pts.push(pts[floater_idx]);

            // MultiPoint.cpp:209-210
            anchor_idx = floater_idx;

            // MultiPoint.cpp:211-212 — assert(dpStack.back() == floater_idx); dpStack.pop_back();
            dp_stack.pop();

            // MultiPoint.cpp:213-214
            if dp_stack.is_empty() {
                break;
            }

            // MultiPoint.cpp:215
            floater_idx = *dp_stack.last().unwrap();
        } else {
            // MultiPoint.cpp:217-218
            floater_idx = furthest_idx;
            dp_stack.push(floater_idx);
        }
        // MultiPoint.cpp:220 — floater = &pts[floater_idx]; (index already updated above)
    }

    // MultiPoint.cpp:223-224
    debug_assert_eq!(result_pts.first(), pts.first());
    debug_assert_eq!(result_pts.last(), pts.last());

    // MultiPoint.cpp:244
    result_pts
}

// ----------------------------------------------------------------------------
// MultiPoint methods, ported as free functions over the point sequence (Rust has
// no C++ inheritance; Polyline/Polygon are the concrete `MultiPoint` subclasses).
// ----------------------------------------------------------------------------

/// Scale every point uniformly. MultiPoint.cpp:6 `MultiPoint::scale(double)`.
pub fn scale(points: &mut [Point], factor: f64) {
    // MultiPoint.cpp:8-9 — pt *= factor; Point::operator*=(double) truncates via
    // coord_t(...) (Point.hpp:199), i.e. truncation toward zero.
    for pt in points.iter_mut() {
        pt.x = (pt.x as f64 * factor) as Coord;
        pt.y = (pt.y as f64 * factor) as Coord;
    }
}

/// Scale every point by independent x/y factors. MultiPoint.cpp:12 `MultiPoint::scale(double,double)`.
pub fn scale_xy(points: &mut [Point], factor_x: f64, factor_y: f64) {
    // MultiPoint.cpp:16-17 — coord_t(pt(i) * factor) truncation toward zero.
    for pt in points.iter_mut() {
        pt.x = (pt.x as f64 * factor_x) as Coord;
        pt.y = (pt.y as f64 * factor_y) as Coord;
    }
}

/// Translate every point by `v`. MultiPoint.cpp:21 `MultiPoint::translate`.
pub fn translate(points: &mut [Point], v: Point) {
    // MultiPoint.cpp:23-24
    for pt in points.iter_mut() {
        *pt = *pt + v;
    }
}

/// Rotate every point by a precomputed (cos, sin). MultiPoint.cpp:27 `MultiPoint::rotate(cos,sin)`.
pub fn rotate(points: &mut [Point], cos_angle: f64, sin_angle: f64) {
    // MultiPoint.cpp:29-34
    for pt in points.iter_mut() {
        let cur_x = pt.x as f64;
        let cur_y = pt.y as f64;
        pt.x = (cos_angle * cur_x - sin_angle * cur_y).round() as Coord;
        pt.y = (cos_angle * cur_y + sin_angle * cur_x).round() as Coord;
    }
}

/// Rotate every point by `angle` (radians) around `center`. MultiPoint.cpp:37 `MultiPoint::rotate(angle,center)`.
pub fn rotate_around(points: &mut [Point], angle: f64, center: Point) {
    // MultiPoint.cpp:39-40
    let s = angle.sin();
    let c = angle.cos();
    // MultiPoint.cpp:41-45
    for pt in points.iter_mut() {
        let vx = (pt.x - center.x) as f64;
        let vy = (pt.y - center.y) as f64;
        pt.x = (center.x as f64 + c * vx - s * vy).round() as Coord;
        pt.y = (center.y as f64 + c * vy + s * vx).round() as Coord;
    }
}

/// Total length of the open polyline through `points`. MultiPoint.cpp:48 `MultiPoint::length`.
/// (Equivalent to the header free `length(const Points&)`, MultiPoint.hpp:134; the
/// polygon closing-edge variant is `Polygon::length`.)
pub fn length(points: &[Point]) -> f64 {
    // MultiPoint.cpp:50-55 — sum of line lengths over lines()
    let mut len = 0.0;
    for w in points.windows(2) {
        let dx = (w[1].x - w[0].x) as f64;
        let dy = (w[1].y - w[0].y) as f64;
        len += (dx * dx + dy * dy).sqrt();
    }
    len
}

/// Index of the first point exactly equal to `point`, or -1. MultiPoint.cpp:58 `MultiPoint::find_point`.
pub fn find_point(points: &[Point], point: &Point) -> i32 {
    // MultiPoint.cpp:60-62
    for (i, pt) in points.iter().enumerate() {
        if *pt == *point {
            return i as i32;
        }
    }
    -1 // MultiPoint.cpp:63 — not found
}

/// Index of the nearest point within `scaled_epsilon`, or -1. MultiPoint.cpp:66 `MultiPoint::find_point(eps)`.
pub fn find_point_eps(points: &[Point], point: &Point, scaled_epsilon: f64) -> i32 {
    // MultiPoint.cpp:68
    if scaled_epsilon == 0.0 {
        return find_point(points, point);
    }
    // MultiPoint.cpp:70-72
    let mut dist2_min = f64::MAX;
    let eps2 = scaled_epsilon * scaled_epsilon;
    let mut idx_min: i32 = -1;
    // MultiPoint.cpp:73-79
    for (i, pt) in points.iter().enumerate() {
        // MultiPoint.cpp:74 — (pt - point).cast<double>().squaredNorm() (raw coords, no unscale)
        let dx = (pt.x - point.x) as f64;
        let dy = (pt.y - point.y) as f64;
        let d2 = dx * dx + dy * dy;
        if d2 < dist2_min {
            idx_min = i as i32;
            dist2_min = d2;
        }
    }
    // MultiPoint.cpp:80
    if dist2_min < eps2 {
        idx_min
    } else {
        -1
    }
}

/// True if `point` lies within SCALED_EPSILON of this multipoint's boundary.
/// MultiPoint.cpp:83 `MultiPoint::has_boundary_point`.
pub fn has_boundary_point(points: &[Point], point: &Point) -> bool {
    // MultiPoint.cpp:85 — dist = (point.projection_onto(*this) - point).cast<double>().norm()
    // projection_onto walks the polyline's lines(); use raw-coord norm (no unscale).
    let proj = point.projection_onto_multipoint(points);
    let dx = (proj.x - point.x) as f64;
    let dy = (proj.y - point.y) as f64;
    let dist = (dx * dx + dy * dy).sqrt();
    // MultiPoint.cpp:86
    dist < crate::libslic3r::SCALED_EPSILON
}

/// Bounding box of `points`. MultiPoint.cpp:89 `MultiPoint::bounding_box`.
pub fn bounding_box(points: &[Point]) -> BoundingBox {
    // MultiPoint.cpp:91 — return BoundingBox(this->points); the Points constructor is construct<false>
    // (BoundingBox.hpp:28-35, 88-102): min/max of the points, defined only with a positive area (a single
    // point, or points on one horizontal or vertical line, give an undefined box). R933: was a merge point by
    // point that defined such boxes; nothing live calls this function.
    let mut bb = BoundingBox::new();
    BoundingBox::construct(&mut bb, points, false);
    bb
}

/// True if any two consecutive points are equal. MultiPoint.cpp:94 `MultiPoint::has_duplicate_points`.
pub fn has_duplicate_points(points: &[Point]) -> bool {
    // MultiPoint.cpp:96-98
    for i in 1..points.len() {
        if points[i - 1] == points[i] {
            return true;
        }
    }
    false
}

/// Remove consecutive duplicate points in place; returns true if any were removed.
/// MultiPoint.cpp:102 `MultiPoint::remove_duplicate_points`.
pub fn remove_duplicate_points(points: &mut Vec<Point>) -> bool {
    if points.is_empty() {
        return false;
    }
    // MultiPoint.cpp:104
    let mut j = 0usize;
    // MultiPoint.cpp:105-113
    for i in 1..points.len() {
        if points[j] == points[i] {
            // Just increase index i.
        } else {
            j += 1;
            if j < i {
                points[j] = points[i];
            }
        }
    }
    // MultiPoint.cpp:114-118
    j += 1;
    if j < points.len() {
        points.truncate(j);
        true
    } else {
        false
    }
}

/// Remove interior collinear points in place; returns true if any were removed.
/// MultiPoint.cpp:121 `MultiPoint::remove_colinear_points`.
pub fn remove_colinear_points(points: &mut Vec<Point>) -> bool {
    // MultiPoint.cpp:122
    if points.len() < 3 {
        return false;
    }
    // MultiPoint.cpp:123
    let mut changed = false;
    // MultiPoint.cpp:124-131
    let mut i = 1usize;
    while i + 1 < points.len() {
        if Line::distance_to_infinite_squared(points[i], points[i - 1], points[i + 1])
            < crate::libslic3r::SCALED_EPSILON
        {
            points.remove(i);
            changed = true;
        } else {
            i += 1;
        }
    }
    // MultiPoint.cpp:132
    if points.len() < 3 {
        points.clear();
    }
    changed
}

/// Visivalingam simplification. MultiPoint.cpp:269 `MultiPoint::visivalingam`.
///
/// Effective-area greedy simplification: repeatedly retire the point with the
/// smallest triangle area, propagating a running minimum so a point is never
/// dropped before a more-important predecessor. Returns the kept points.
pub fn visivalingam(pts: &[Point], tolerance: f64) -> Vec<Point> {
    // MultiPoint.cpp:272 — assert(pts.size() >= 2);
    // MultiPoint.cpp:274
    let mut results: Vec<Point> = Vec::new();

    // MultiPoint.cpp:277-286 — effective area spanned by curr and its prev/next neighbours.
    // 0.5 * |cross2((next-curr), (prev-curr))| with raw-coord (cast<double>) vectors.
    let effective_area = |curr_pt_idx: usize, prev_pt_idx: usize, next_pt_idx: usize| -> f64 {
        let curr = pts[curr_pt_idx];
        let prev = pts[prev_pt_idx];
        let next = pts[next_pt_idx];
        let curr_to_next = crate::geometry::PointF {
            x: (next.x - curr.x) as f64,
            y: (next.y - curr.y) as f64,
        };
        let prev_to_next = crate::geometry::PointF {
            x: (prev.x - curr.x) as f64,
            y: (prev.y - curr.y) as f64,
        };
        0.50 * cross2f(curr_to_next, prev_to_next).abs()
    };

    // MultiPoint.cpp:288-289 — per-node effective areas (filled in as nodes retire).
    let mut areas: Vec<f64> = vec![0.0; pts.len()];

    // The C++ uses a binary-heap of vis_node*; we model the doubly-linked structure
    // with parallel arrays (prev_idx/next_idx/area) and a `present` flag, and resort
    // the candidate set the same way std::make_heap would re-prioritise it. Indices
    // mirror node_list[]; entries 0 and pts.len()-1 are the (never-retired) endpoints.
    // MultiPoint.cpp:292-302
    if pts.len() < 2 {
        return pts.to_vec();
    }
    let mut prev_idx: Vec<usize> = vec![0; pts.len()];
    let mut next_idx: Vec<usize> = vec![0; pts.len()];
    let mut node_area: Vec<f64> = vec![0.0; pts.len()];
    let mut present: Vec<bool> = vec![false; pts.len()];
    // MultiPoint.cpp:296-302 — for i in 1..size-1: build node and push to heap.
    for i in 1..pts.len() - 1 {
        prev_idx[i] = i - 1;
        next_idx[i] = i + 1;
        node_area[i] = effective_area(i, i - 1, i + 1);
        present[i] = true;
    }

    // MultiPoint.cpp:307 — min_area = -max(double); running maximum of retired areas.
    let mut min_area = f64::MIN;
    // MultiPoint.cpp:308 — while (!heap.empty())
    loop {
        // MultiPoint.cpp:310-314 — pop the present node with the smallest area.
        let mut curr: Option<usize> = None;
        let mut curr_area = f64::INFINITY;
        for i in 1..pts.len() - 1 {
            if present[i] && node_area[i] < curr_area {
                curr_area = node_area[i];
                curr = Some(i);
            }
        }
        let curr = match curr {
            Some(c) => c,
            None => break,
        };
        present[curr] = false;

        // MultiPoint.cpp:320 — min_area = std::max(min_area, curr->area);
        min_area = min_area.max(node_area[curr]);

        // MultiPoint.cpp:322-328 — update prev neighbour if it is still a live node.
        let p = prev_idx[curr];
        if p != 0 && present[p] {
            next_idx[p] = next_idx[curr];
            node_area[p] = effective_area(p, prev_idx[p], next_idx[p]);
        }
        // MultiPoint.cpp:330-335 — update next neighbour if it is still a live node.
        let n = next_idx[curr];
        if n != pts.len() - 1 && present[n] {
            prev_idx[n] = prev_idx[curr];
            node_area[n] = effective_area(n, prev_idx[n], next_idx[n]);
        }
        // MultiPoint.cpp:336 — areas[curr->pt_idx] = min_area;
        areas[curr] = min_area;
    }

    // MultiPoint.cpp:344-354 — keep endpoints, and any interior point whose area > tolerance.
    let use_point = |idx: usize| -> bool {
        if idx == 0 || idx == areas.len() - 1 {
            true
        } else {
            areas[idx] > tolerance
        }
    };
    // MultiPoint.cpp:356-360
    for i in 0..pts.len() {
        if use_point(i) {
            results.push(pts[i]);
        }
    }
    // MultiPoint.cpp:362 — assert(results.size() >= 2);
    // MultiPoint.cpp:364
    results
}

/// Calculate 2D concave hull of a polygon within `tolerence`.
/// MultiPoint.cpp:368 `MultiPoint::concave_hull_2d`.
pub fn concave_hull_2d(pts: &[Point], tolerence: f64) -> Vec<Point> {
    // MultiPoint.cpp:370
    let mut hull: Vec<Point> = Vec::new();
    // MultiPoint.cpp:371
    let n = pts.len() as i32;
    // MultiPoint.cpp:372
    if n >= 3 {
        // MultiPoint.cpp:373-374
        let mut k: usize = 0;
        hull.resize(n as usize, Point::zero());
        // MultiPoint.cpp:375
        for i in 0..n as usize {
            // MultiPoint.cpp:376-377 — pop while CCW <= 0 and the (negated, normalised)
            // turn is within tolerance. ccw / norm use raw-coord (cast<double>) values.
            while k >= 2
                && pts[i].ccw(&hull[k - 2], &hull[k - 1]) <= 0.0
                && {
                    // `(pts[i] - hull[k - 1]).norm()` is Eigen's norm of an integer (coord_t = int32_t) vector:
                    // numext::sqrt(squaredNorm()) with the squared norm in int32 and std::sqrt's double result
                    // converted back to int32 (truncated); the quotient is then a double divided by that integer.
                    // R933: was the double norm (concave_hull_2d_native: 9 of 3,000 hulls differed).
                    let d = hull[k - 1];
                    let dx = (pts[i].x - d.x) as i32;
                    let dy = (pts[i].y - d.y) as i32;
                    let sq = dx.wrapping_mul(dx).wrapping_add(dy.wrapping_mul(dy));
                    let norm = (sq as f64).sqrt() as i32;
                    -pts[i].ccw(&hull[k - 2], &hull[k - 1]) / (norm as f64) < tolerence
                }
            {
                k -= 1;
            }
            // MultiPoint.cpp:378 — hull[k++] = pts[i];
            hull[k] = pts[i];
            k += 1;
        }
        // MultiPoint.cpp:380
        hull.truncate(k);
        // MultiPoint.cpp:381-382
        if !hull.is_empty() && hull.first() == hull.last() {
            hull.pop();
        }
    }
    // MultiPoint.cpp:384
    hull
}

/// Mirror every point across the vertical line `x = x_axis`.
/// MultiPoint.cpp:472 `MultiPoint::symmetric_y`.
/// (Polygon-level callers also use `ex_polygon::symmetric_y_polygon`.)
pub fn symmetric_y(points: &mut [Point], x_axis: Coord) {
    // MultiPoint.cpp:474-476
    for pt in points.iter_mut() {
        pt.x = 2 * x_axis - pt.x;
    }
}

/// Bounding box of `points`. MultiPoint.cpp:436 `get_extents(const MultiPoint&)`.
pub fn get_extents(points: &[Point]) -> BoundingBox {
    // MultiPoint.cpp:438 — return BoundingBox(mp.points);
    bounding_box(points)
}

/// Header free function `length(const Points&)`. MultiPoint.hpp:134.
/// Open-polyline length (sum of consecutive segment lengths, raw-coord norm).
pub fn length_points(pts: &[Point]) -> f64 {
    // MultiPoint.hpp:135-141
    let mut total = 0.0;
    if !pts.is_empty() {
        for w in pts.windows(2) {
            let dx = (w[1].x - w[0].x) as f64;
            let dy = (w[1].y - w[0].y) as f64;
            total += (dx * dx + dy * dy).sqrt();
        }
    }
    total
}

/// Header free function `area(const Points&)`. MultiPoint.hpp:144.
/// Twice the signed shoelace area of the closed polygon described by `polygon`.
pub fn area(polygon: &[Point]) -> f64 {
    // MultiPoint.hpp:145-148
    let mut area = 0.0;
    if polygon.is_empty() {
        return area;
    }
    // MultiPoint.hpp:147 — `area += double(polygon[i](0) + polygon[j](0)) * double(polygon[i](1) - polygon[j](1));`
    // is one expression, so the scalar loop fuses the product into the add. At -O3 clang vectorizes the loop as
    // an ordered reduction, 16 iterations per vector step, entered when there are at least 16 points: those
    // iterations multiply in vector registers and add the products in order (two roundings); only the remainder
    // runs the fused scalar loop. Measured on the pin (tests/data/multi_point/area.txt, n = 3..99: 0 of 2,328
    // differ with this split). R933: was two roundings everywhere. Sum and difference are coord_t (int32) before
    // the conversion.
    let n = polygon.len();
    let vectorized = if n >= 16 { n / 16 * 16 } else { 0 };
    let mut j = n - 1;
    for i in 0..n {
        let sx = (polygon[i].x as i32).wrapping_add(polygon[j].x as i32) as f64;
        let dy = (polygon[i].y as i32).wrapping_sub(polygon[j].y as i32) as f64;
        if i < vectorized {
            area += sx * dy;
        } else {
            area = sx.mul_add(dy, area);
        }
        j = i;
    }
    area
}

/// MultiPoint.hpp:111-125 `class MultiPoint3`: a sequence of 3D points. The C++ class is abstract (`lines()` is
/// pure virtual); its one concrete subclass is `Polyline3` (Polyline.hpp), whose lines are the open chain, which
/// is what `length` walks here.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MultiPoint3 {
    pub points: Vec<crate::geometry::Point3>,
}

impl MultiPoint3 {
    /// MultiPoint.hpp:116 `void append(const Vec3crd& point) { this->points.push_back(point); }`
    pub fn append(&mut self, point: crate::geometry::Point3) {
        self.points.push(point);
    }

    /// MultiPoint.cpp:388-394 `void MultiPoint3::translate(double x, double y)`: `p(0) += coord_t(x);
    /// p(1) += coord_t(y);` (the conversion truncates).
    pub fn translate(&mut self, x: f64, y: f64) {
        for p in self.points.iter_mut() {
            p.x += x as Coord;
            p.y += y as Coord;
        }
    }

    /// MultiPoint.cpp:396-399 `void MultiPoint3::translate(const Point& vector)`:
    /// `this->translate(vector(0), vector(1));`
    pub fn translate_point(&mut self, vector: &Point) {
        self.translate(vector.x as f64, vector.y as f64);
    }

    /// MultiPoint.cpp:401-407 `double MultiPoint3::length() const`: the sum of `Line3::length()`
    /// (`(b - a).cast<double>().norm()`) over the open chain of `Polyline3::lines()`.
    pub fn length(&self) -> f64 {
        let mut len = 0.0;
        for w in self.points.windows(2) {
            let dx = (w[1].x - w[0].x) as f64;
            let dy = (w[1].y - w[0].y) as f64;
            let dz = (w[1].z - w[0].z) as f64;
            len += (dx * dx + dy * dy + dz * dz).sqrt();
        }
        len
    }

    /// MultiPoint.hpp:122 `bool is_valid() const { return this->points.size() >= 2; }`
    pub fn is_valid(&self) -> bool {
        self.points.len() >= 2
    }

    /// MultiPoint.cpp:409-412 `BoundingBox3 MultiPoint3::bounding_box() const { return BoundingBox3(points); }`:
    /// the Points constructor of BoundingBox3Base (BoundingBox.hpp:116-130) throws on an empty set (here a panic
    /// with the same message), takes the coordinate-wise min and max and is defined only with a positive volume.
    pub fn bounding_box(&self) -> crate::geometry::BoundingBox3 {
        if self.points.is_empty() {
            panic!("Empty point set supplied to BoundingBox3Base constructor");
        }
        let mut min = self.points[0];
        let mut max = min;
        for p in &self.points[1..] {
            min = crate::geometry::Point3::new(min.x.min(p.x), min.y.min(p.y), min.z.min(p.z));
            max = crate::geometry::Point3::new(max.x.max(p.x), max.y.max(p.y), max.z.max(p.z));
        }
        // BoundingBox.hpp:129 — defined = min < max in all three coordinates.
        if min.x < max.x && min.y < max.y && min.z < max.z {
            return crate::geometry::BoundingBox3::from_points_minmax(min, max);
        }
        let mut bb = crate::geometry::BoundingBox3::new();
        bb.min = min;
        bb.max = max;
        bb
    }

    /// MultiPoint.cpp:414-433 `bool MultiPoint3::remove_duplicate_points()`: drop consecutive equal points;
    /// true when any was dropped.
    pub fn remove_duplicate_points(&mut self) -> bool {
        let mut j = 0usize;
        for i in 1..self.points.len() {
            if self.points[j] == self.points[i] {
                // Just increase index i.
            } else {
                j += 1;
                if j < i {
                    self.points[j] = self.points[i];
                }
            }
        }
        j += 1;
        if j < self.points.len() {
            self.points.truncate(j);
            return true;
        }
        false
    }
}

// ---------------------------------------------------------------------------
// `class MultiPoint` (MultiPoint.hpp:17-115) as a trait under the C++ names: the abstract base of
// Polygon and Polyline, with the pure virtual `lines()`; the overloaded members are impls of one
// generic trait each, for every MultiPoint.
// ---------------------------------------------------------------------------

/// `class MultiPoint` (MultiPoint.hpp:17-115).
pub trait MultiPoint {
    fn points(&self) -> &Vec<Point>;
    fn points_mut(&mut self) -> &mut Vec<Point>;
    /// `virtual Lines lines() const = 0` (MultiPoint.hpp:39)
    fn lines(&self) -> Vec<Line>;

    /// MultiPoint.hpp:33 (`std::reverse(points.begin(), points.end())`)
    fn reverse(&mut self) {
        self.points_mut().reverse();
    }
    /// MultiPoint.hpp:40
    fn size(&self) -> usize {
        self.points().len()
    }
    /// MultiPoint.hpp:41
    fn empty(&self) -> bool {
        self.points().is_empty()
    }
    /// MultiPoint.hpp:43
    fn is_valid(&self) -> bool {
        self.points().len() >= 2
    }
    /// MultiPoint.hpp:52-66: the first point at the smallest `(p - point).cast<double>().norm()`.
    fn closest_point_index(&self, point: &Point) -> i32 {
        let pts = self.points();
        let mut idx: i32 = -1;
        if !pts.is_empty() {
            idx = 0;
            let mut dist_min = eigen_norm2d(point.x - pts[0].x, point.y - pts[0].y);
            for i in 1..pts.len() {
                let d = eigen_norm2d(pts[i].x - point.x, pts[i].y - point.y);
                if d < dist_min {
                    dist_min = d;
                    idx = i as i32;
                }
            }
        }
        idx
    }
    /// MultiPoint.hpp:67
    fn closest_point(&self, point: &Point) -> Option<&Point> {
        if self.points().is_empty() {
            None
        } else {
            Some(&self.points()[self.closest_point_index(point) as usize])
        }
    }
    /// MultiPoint.hpp:70-73 (the C++ dereferences the closest point without a check)
    fn distance_to(&self, point: &Point) -> f64 {
        let cl = *self.closest_point(point).expect("MultiPoint::distance_to of an empty MultiPoint");
        eigen_norm2d(cl.x - point.x, cl.y - point.y)
    }
    /// MultiPoint.hpp:81
    fn clear(&mut self) {
        self.points_mut().clear();
    }
    /// MultiPoint.cpp:136-143: the first line of `lines()` that intersects `line`.
    fn intersection(&self, line: &Line, intersection: &mut Point) -> bool {
        for it in self.lines() {
            if let Some(ip) = it.intersection(line) {
                *intersection = ip;
                return true;
            }
        }
        false
    }
    /// MultiPoint.cpp:145-166: the intersection nearest to `line.a`.
    fn first_intersection(&self, line: &Line, intersection: &mut Point) -> bool {
        let mut found = false;
        let mut dmin = 0.0_f64;
        for l in self.lines() {
            if let Some(ip) = l.intersection(line) {
                if !found {
                    found = true;
                    dmin = eigen_norm2d(line.a.x - ip.x, line.a.y - ip.y);
                    *intersection = ip;
                } else {
                    let d = eigen_norm2d(line.a.x - ip.x, line.a.y - ip.y);
                    if d < dmin {
                        dmin = d;
                        *intersection = ip;
                    }
                }
            }
        }
        found
    }
    /// MultiPoint.cpp:168-177: appends every intersection, true when any was added.
    fn intersections(&self, line: &Line, intersections: &mut Vec<Point>) -> bool {
        let intersections_size = intersections.len();
        for polygon_line in self.lines() {
            if let Some(ip) = polygon_line.intersection(line) {
                intersections.push(ip);
            }
        }
        intersections.len() > intersections_size
    }
}

/// `(a - b).cast<double>().norm()` of a Vec2crd: the coord_t difference (int32 at the pin), then
/// Eigen's `sqrt(x*x + y*y)` with the two products rounded before the sum.
#[inline]
fn eigen_norm2d(dx: Coord, dy: Coord) -> f64 {
    let x = dx as i32 as f64;
    let y = dy as i32 as f64;
    (x * x + y * y).sqrt()
}

impl MultiPoint for crate::geometry::Polygon {
    fn points(&self) -> &Vec<Point> {
        &self.points
    }
    fn points_mut(&mut self) -> &mut Vec<Point> {
        &mut self.points
    }
    /// Polygon::lines (Polygon.cpp:79-82)
    fn lines(&self) -> Vec<Line> {
        crate::geometry::Polygon::lines(self)
    }
}

impl MultiPoint for crate::geometry::Polyline {
    fn points(&self) -> &Vec<Point> {
        &self.points
    }
    fn points_mut(&mut self) -> &mut Vec<Point> {
        &mut self.points
    }
    /// Polyline::lines (Polyline.cpp:99-108)
    fn lines(&self) -> Vec<Line> {
        crate::geometry::Polyline::lines(self)
    }
}

/// `MultiPoint::scale` (MultiPoint.cpp:6-19): the two overloads.
pub trait MultiPointScale<A> {
    fn scale(&mut self, args: A);
}

impl<T: MultiPoint + ?Sized> MultiPointScale<f64> for T {
    /// MultiPoint.cpp:6-10: `pt *= factor` (Point.hpp:197, `coord_t(x * rhs)`, truncating).
    fn scale(&mut self, factor: f64) {
        for pt in self.points_mut().iter_mut() {
            pt.x = ((pt.x as i32) as f64 * factor) as i32 as Coord;
            pt.y = ((pt.y as i32) as f64 * factor) as i32 as Coord;
        }
    }
}

impl<T: MultiPoint + ?Sized> MultiPointScale<(f64, f64)> for T {
    /// MultiPoint.cpp:12-19
    fn scale(&mut self, (factor_x, factor_y): (f64, f64)) {
        for pt in self.points_mut().iter_mut() {
            pt.x = ((pt.x as i32) as f64 * factor_x) as i32 as Coord;
            pt.y = ((pt.y as i32) as f64 * factor_y) as i32 as Coord;
        }
    }
}

/// `MultiPoint::translate` (MultiPoint.hpp:28-29, MultiPoint.cpp:21-25): the two overloads.
pub trait MultiPointTranslate<A> {
    fn translate(&mut self, args: A);
}

impl<T: MultiPoint + ?Sized> MultiPointTranslate<&Point> for T {
    /// MultiPoint.cpp:21-25: `pt += v`, in coord_t (int32 at the pin).
    fn translate(&mut self, v: &Point) {
        for pt in self.points_mut().iter_mut() {
            pt.x = (pt.x as i32).wrapping_add(v.x as i32) as Coord;
            pt.y = (pt.y as i32).wrapping_add(v.y as i32) as Coord;
        }
    }
}

impl<T: MultiPoint + ?Sized> MultiPointTranslate<(f64, f64)> for T {
    /// MultiPoint.hpp:28: `translate(Point(coord_t(x), coord_t(y)))`, truncating.
    fn translate(&mut self, (x, y): (f64, f64)) {
        let v = Point::new(x as i32 as Coord, y as i32 as Coord);
        MultiPointTranslate::<&Point>::translate(self, &v);
    }
}

/// `MultiPoint::rotate` (MultiPoint.hpp:30-32, MultiPoint.cpp:27-46): the three overloads.
pub trait MultiPointRotate<A> {
    fn rotate(&mut self, args: A);
}

impl<T: MultiPoint + ?Sized> MultiPointRotate<(f64, f64)> for T {
    /// MultiPoint.cpp:27-35: `coord_t(round(cos * x - sin * y))`, `coord_t(round(cos * y + sin * x))`,
    /// the left product of each fused by clang.
    fn rotate(&mut self, (cos_angle, sin_angle): (f64, f64)) {
        for pt in self.points_mut().iter_mut() {
            let cur_x = (pt.x as i32) as f64;
            let cur_y = (pt.y as i32) as f64;
            pt.x = cos_angle.mul_add(cur_x, -(sin_angle * cur_y)).round() as i32 as Coord;
            pt.y = cos_angle.mul_add(cur_y, sin_angle * cur_x).round() as i32 as Coord;
        }
    }
}

impl<T: MultiPoint + ?Sized> MultiPointRotate<f64> for T {
    /// MultiPoint.hpp:30: `rotate(cos(angle), sin(angle))`
    fn rotate(&mut self, angle: f64) {
        MultiPointRotate::<(f64, f64)>::rotate(self, (angle.cos(), angle.sin()));
    }
}

impl<T: MultiPoint + ?Sized> MultiPointRotate<(f64, &Point)> for T {
    /// MultiPoint.cpp:37-46: `v = pt - center` (coord_t), `round(center.x + c * v.x - s * v.y)`,
    /// `round(center.y + c * v.y + s * v.x)`, each fused twice by clang.
    fn rotate(&mut self, (angle, center): (f64, &Point)) {
        let s = angle.sin();
        let c = angle.cos();
        let cx = (center.x as i32) as f64;
        let cy = (center.y as i32) as f64;
        for pt in self.points_mut().iter_mut() {
            let vx = (pt.x as i32).wrapping_sub(center.x as i32) as f64;
            let vy = (pt.y as i32).wrapping_sub(center.y as i32) as f64;
            pt.x = (-s).mul_add(vy, c.mul_add(vx, cx)).round() as i32 as Coord;
            pt.y = s.mul_add(vx, c.mul_add(vy, cy)).round() as i32 as Coord;
        }
    }
}

/// `MultiPoint::append` (MultiPoint.hpp:82-93): the four overloads.
pub trait MultiPointAppend<A> {
    fn append(&mut self, args: A);
}

impl<T: MultiPoint + ?Sized> MultiPointAppend<&Point> for T {
    /// MultiPoint.hpp:82
    fn append(&mut self, point: &Point) {
        self.points_mut().push(*point);
    }
}

impl<T: MultiPoint + ?Sized> MultiPointAppend<&[Point]> for T {
    /// MultiPoint.hpp:83 (`append(const Points &src)`)
    fn append(&mut self, src: &[Point]) {
        self.points_mut().extend_from_slice(src);
    }
}

impl<'a, T: MultiPoint + ?Sized> MultiPointAppend<std::slice::Iter<'a, Point>> for T {
    /// MultiPoint.hpp:84 (`append(const_iterator begin, const_iterator end)`)
    fn append(&mut self, range: std::slice::Iter<'a, Point>) {
        self.points_mut().extend(range.copied());
    }
}

impl<T: MultiPoint + ?Sized> MultiPointAppend<Vec<Point>> for T {
    /// MultiPoint.hpp:85-93 (`append(Points &&src)`): takes the vector when empty.
    fn append(&mut self, src: Vec<Point>) {
        if self.points().is_empty() {
            *self.points_mut() = src;
        } else {
            self.points_mut().extend_from_slice(&src);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_douglas_peucker_empty() {
        let pts = vec![];
        let result = douglas_peucker(&pts, 0.1);
        assert_eq!(result.len(), 0);
    }

    #[test]
    fn test_douglas_peucker_single_point() {
        let pts = vec![Point::new(0, 0)];
        let result = douglas_peucker(&pts, 0.1);
        assert_eq!(result.len(), 1);
        assert_eq!(result[0], pts[0]);
    }

    #[test]
    fn test_douglas_peucker_two_points() {
        let pts = vec![Point::new(0, 0), Point::new(100, 100)];
        let result = douglas_peucker(&pts, 0.1);
        assert_eq!(result.len(), 2);
        assert_eq!(result[0], pts[0]);
        assert_eq!(result[1], pts[1]);
    }

    #[test]
    fn test_douglas_peucker_straight_line() {
        // Points on a straight line should be reduced to endpoints
        let pts = vec![
            Point::new(0, 0),
            Point::new(10, 10),
            Point::new(20, 20),
            Point::new(30, 30),
            Point::new(40, 40),
        ];
        let result = douglas_peucker(&pts, 1.0);
        // Should keep only start and end
        assert_eq!(result.len(), 2);
        assert_eq!(result[0], pts[0]);
        assert_eq!(result[1], pts[4]);
    }

    #[test]
    fn test_douglas_peucker_zigzag() {
        // Zigzag pattern should keep significant points
        let pts = vec![
            Point::new(0, 0),
            Point::new(10, 0),
            Point::new(10, 10),
            Point::new(20, 10),
            Point::new(20, 0),
        ];
        // With tight tolerance, should keep most points
        let result = douglas_peucker(&pts, 0.1);
        assert!(result.len() >= 3); // At least start, middle turn, end
        assert_eq!(result[0], pts[0]);
        assert_eq!(result[result.len() - 1], pts[pts.len() - 1]);
    }

    #[test]
    fn test_douglas_peucker_preserves_endpoints() {
        // First and last points must always be preserved
        let pts = vec![
            Point::new(0, 0),
            Point::new(5, 1),
            Point::new(10, 2),
            Point::new(15, 1),
            Point::new(20, 0),
        ];
        let result = douglas_peucker(&pts, 10.0);
        assert!(result.len() >= 2);
        assert_eq!(result[0], pts[0]);
        assert_eq!(result[result.len() - 1], pts[pts.len() - 1]);
    }
}
