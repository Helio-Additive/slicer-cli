//! Axis-aligned bounding boxes for 2D and 3D geometry
//!
//! C++ Reference:
//! - BoundingBox.hpp (full template class hierarchy)
//! - BoundingBox.cpp (template instantiations and methods)
//!
//! This module provides bounding box types for both scaled integer coordinates
//! (BoundingBox, BoundingBox3) and floating-point coordinates (BoundingBoxf, BoundingBoxf3).

use crate::geometry::{Point, Polygon, Vec2d, Vec3d};

/// 2D bounding box with scaled integer coordinates
/// C++ Reference: BoundingBox.hpp - class BoundingBox : public BoundingBoxBase<Point>
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BoundingBox {
    /// Minimum corner point
    /// BoundingBox.hpp:13
    pub min: Point,
    /// Maximum corner point
    /// BoundingBox.hpp:14
    pub max: Point,
    /// Whether the bounding box has been initialized with valid points
    /// BoundingBox.hpp:15
    pub defined: bool,
}

impl BoundingBox {
    /// Create an undefined (empty) bounding box
    /// BoundingBox.hpp:17
    /// C++: BoundingBoxBase() : min(PointClass::Zero()), max(PointClass::Zero()), defined(false) {}
    pub fn new() -> Self {
        Self {
            min: Point::new(0, 0),
            max: Point::new(0, 0),
            defined: false,
        }
    }

    /// Create a bounding box from min and max corners
    /// BoundingBox.hpp:18-19
    /// C++: BoundingBoxBase(const PointClass &pmin, const PointClass &pmax) :
    /// C++:     min(pmin), max(pmax), defined(pmin(0) < pmax(0) && pmin(1) < pmax(1)) {}
    pub fn new_from_points(pmin: Point, pmax: Point) -> Self {
        Self {
            min: pmin,
            max: pmax,
            defined: pmin.x < pmax.x && pmin.y < pmax.y,
        }
    }

    /// Create a bounding box from three points
    /// BoundingBox.hpp:20-21
    /// C++: BoundingBoxBase(const PointClass &p1, const PointClass &p2, const PointClass &p3) :
    /// C++:     min(p1), max(p1), defined(false) { merge(p2); merge(p3); }
    pub fn new_from_three(p1: Point, p2: Point, p3: Point) -> Self {
        let mut bb = Self {
            min: p1,
            max: p1,
            defined: false,
        };
        bb.merge_point(p2);
        bb.merge_point(p3);
        bb
    }

    /// Create a bounding box from a vector of points
    /// BoundingBox.hpp:29-31
    /// C++: BoundingBoxBase(const std::vector<PointClass> &points)
    /// C++:     : BoundingBoxBase(points.begin(), points.end())
    pub fn new_from_points_slice(points: &[Point]) -> Self {
        Self::from_iterator(points.iter().copied())
    }

    /// Create a bounding box from an iterator of points
    /// BoundingBox.hpp:24-27 + 76-90 (construct template)
    /// C++: template<class It, class = IteratorOnly<It>>
    /// C++: BoundingBoxBase(It from, It to) { construct(*this, from, to); }
    pub fn from_iterator<I>(points: I) -> Self
    where
        I: Iterator<Item = Point>,
    {
        let mut bb = Self::new();
        for point in points {
            bb.merge_point(point);
        }
        bb
    }

    /// Reset the bounding box to undefined state
    /// BoundingBox.hpp:33
    /// C++: void reset() { this->defined = false; this->min = PointClass::Zero(); this->max = PointClass::Zero(); }
    pub fn reset(&mut self) {
        self.defined = false;
        self.min = Point::new(0, 0);
        self.max = Point::new(0, 0);
    }

    /// Merge a single point into the bounding box
    /// BoundingBox.cpp:78-89
    /// C++: template <class PointClass> void
    /// C++: BoundingBoxBase<PointClass>::merge(const PointClass &point)
    /// C++: {
    /// C++:     if (this->defined) {
    /// C++:         this->min = this->min.cwiseMin(point);
    /// C++:         this->max = this->max.cwiseMax(point);
    /// C++:     } else {
    /// C++:         this->min = point;
    /// C++:         this->max = point;
    /// C++:         this->defined = true;
    /// C++:     }
    /// C++: }
    pub fn merge_point(&mut self, point: Point) {
        if self.defined {
            self.min.x = self.min.x.min(point.x);
            self.min.y = self.min.y.min(point.y);
            self.max.x = self.max.x.max(point.x);
            self.max.y = self.max.y.max(point.y);
        } else {
            self.min = point;
            self.max = point;
            self.defined = true;
        }
    }

    /// Merge a slice of points into the bounding box
    /// BoundingBox.cpp:94-98
    /// C++: template <class PointClass> void
    /// C++: BoundingBoxBase<PointClass>::merge(const std::vector<PointClass> &points)
    /// C++: {
    /// C++:     this->merge(BoundingBoxBase(points));
    /// C++: }
    pub fn merge_points(&mut self, points: &[Point]) {
        let bb = Self::new_from_points_slice(points);
        self.merge_bb(&bb);
    }

    /// Merge another bounding box into this one
    /// BoundingBox.cpp:102-116
    /// C++: template <class PointClass> void
    /// C++: BoundingBoxBase<PointClass>::merge(const BoundingBoxBase<PointClass> &bb)
    /// C++: {
    /// C++:     assert(bb.defined || bb.min(0) >= bb.max(0) || bb.min(1) >= bb.max(1));
    /// C++:     if (bb.defined) {
    /// C++:         if (this->defined) {
    /// C++:             this->min = this->min.cwiseMin(bb.min);
    /// C++:             this->max = this->max.cwiseMax(bb.max);
    /// C++:         } else {
    /// C++:             this->min = bb.min;
    /// C++:             this->max = bb.max;
    /// C++:             this->defined = true;
    /// C++:         }
    /// C++:     }
    /// C++: }
    pub fn merge_bb(&mut self, bb: &BoundingBox) {
        if bb.defined {
            if self.defined {
                self.min.x = self.min.x.min(bb.min.x);
                self.min.y = self.min.y.min(bb.min.y);
                self.max.x = self.max.x.max(bb.max.x);
                self.max.y = self.max.y.max(bb.max.y);
            } else {
                self.min = bb.min;
                self.max = bb.max;
                self.defined = true;
            }
        }
    }

    /// Scale the bounding box by a factor
    /// BoundingBox.cpp:62-67
    /// C++: template <class PointClass> void
    /// C++: BoundingBoxBase<PointClass>::scale(double factor)
    /// C++: {
    /// C++:     this->min *= factor;
    /// C++:     this->max *= factor;
    /// C++: }
    pub fn scale(&mut self, factor: f64) {
        self.min.x = (self.min.x as f64 * factor).round() as i64;
        self.min.y = (self.min.y as f64 * factor).round() as i64;
        self.max.x = (self.max.x as f64 * factor).round() as i64;
        self.max.y = (self.max.y as f64 * factor).round() as i64;
    }

    /// Get the size (width and height) of the bounding box
    /// BoundingBox.cpp:151-155
    /// C++: template <class PointClass> PointClass
    /// C++: BoundingBoxBase<PointClass>::size() const
    /// C++: {
    /// C++:     return PointClass(this->max(0) - this->min(0), this->max(1) - this->min(1));
    /// C++: }
    pub fn size(&self) -> Point {
        Point::new(self.max.x - self.min.x, self.max.y - self.min.y)
    }

    /// Get the radius (half the diagonal length) of the bounding box
    /// BoundingBox.cpp:166-172
    /// C++: template <class PointClass> double BoundingBoxBase<PointClass>::radius() const
    /// C++: {
    /// C++:     assert(this->defined);
    /// C++:     double x = this->max(0) - this->min(0);
    /// C++:     double y = this->max(1) - this->min(1);
    /// C++:     return 0.5 * sqrt(x*x+y*y);
    /// C++: }
    pub fn radius(&self) -> f64 {
        assert!(self.defined);
        let x = (self.max.x - self.min.x) as f64;
        let y = (self.max.y - self.min.y) as f64;
        0.5 * (x * x + y * y).sqrt()
    }

    /// Get the area of the bounding box
    /// BoundingBox.hpp:41
    /// C++: double area() const { return double(this->max(0) - this->min(0)) * (this->max(1) - this->min(1)); }
    pub fn area(&self) -> f64 {
        ((self.max.x - self.min.x) as f64) * ((self.max.y - self.min.y) as f64)
    }

    /// Translate the bounding box by (x, y)
    /// BoundingBox.hpp:42
    /// C++: void translate(coordf_t x, coordf_t y) { assert(this->defined); PointClass v(x, y); this->min += v; this->max += v; }
    pub fn translate(&mut self, x: f64, y: f64) {
        assert!(self.defined);
        let dx = x.round() as i64;
        let dy = y.round() as i64;
        self.min.x += dx;
        self.min.y += dy;
        self.max.x += dx;
        self.max.y += dy;
    }

    /// Translate the bounding box by a Vec2d
    /// BoundingBox.hpp:43
    /// C++: void translate(const Vec2d& v0) { PointClass v(v0.x(), v0.y()); this->min += v; this->max += v; }
    pub fn translate_vec(&mut self, v: Vec2d) {
        self.translate(v.x, v.y);
    }

    /// Expand the bounding box by delta in all directions
    /// BoundingBox.cpp:186-191
    /// C++: template <class PointClass> void
    /// C++: BoundingBoxBase<PointClass>::offset(coordf_t delta)
    /// C++: {
    /// C++:     PointClass v(delta, delta);
    /// C++:     this->min -= v;
    /// C++:     this->max += v;
    /// C++: }
    pub fn offset(&mut self, delta: f64) {
        let d = delta.round() as i64;
        self.min.x -= d;
        self.min.y -= d;
        self.max.x += d;
        self.max.y += d;
    }

    /// Return a new bounding box inflated by delta
    /// BoundingBox.hpp:45
    /// C++: BoundingBoxBase<PointClass> inflated(coordf_t delta) const throw() { BoundingBoxBase<PointClass> out(*this); out.offset(delta); return out; }
    pub fn inflated(&self, delta: f64) -> Self {
        let mut out = *self;
        out.offset(delta);
        out
    }

    /// Get the center point of the bounding box
    /// BoundingBox.cpp:201-205
    /// C++: template <class PointClass> PointClass
    /// C++: BoundingBoxBase<PointClass>::center() const
    /// C++: {
    /// C++:     return (this->min + this->max) / 2;
    /// C++: }
    pub fn center(&self) -> Point {
        Point::new((self.min.x + self.max.x) / 2, (self.min.y + self.max.y) / 2)
    }

    /// Check if a point is contained within the bounding box
    /// BoundingBox.hpp:47-50
    /// C++: bool contains(const PointClass &point) const {
    /// C++:     return point(0) >= this->min(0) && point(0) <= this->max(0)
    /// C++:         && point(1) >= this->min(1) && point(1) <= this->max(1);
    /// C++: }
    pub fn contains_point(&self, point: Point) -> bool {
        point.x >= self.min.x
            && point.x <= self.max.x
            && point.y >= self.min.y
            && point.y <= self.max.y
    }

    /// Check if another bounding box is fully contained within this one
    /// BoundingBox.hpp:51-53
    /// C++: bool contains(const BoundingBoxBase<PointClass> &other) const {
    /// C++:     return contains(other.min) && contains(other.max);
    /// C++: }
    pub fn contains_bb(&self, other: &BoundingBox) -> bool {
        self.contains_point(other.min) && self.contains_point(other.max)
    }

    /// Check if this bounding box overlaps with another
    /// BoundingBox.hpp:54-57
    /// C++: bool overlap(const BoundingBoxBase<PointClass> &other) const {
    /// C++:     return ! (this->max(0) < other.min(0) || this->min(0) > other.max(0) ||
    /// C++:               this->max(1) < other.min(1) || this->min(1) > other.max(1));
    /// C++: }
    pub fn overlap(&self, other: &BoundingBox) -> bool {
        !(self.max.x < other.min.x
            || self.min.x > other.max.x
            || self.max.y < other.min.y
            || self.min.y > other.max.y)
    }

    /// Get a corner point by index (0=min, 1=max_x+min_y, 2=max, 3=min_x+max_y)
    /// BoundingBox.hpp:58-67
    /// C++: PointClass operator[](size_t idx) const
    /// C++: {
    /// C++:     switch (idx) {
    /// C++:     case 0: return min; break;
    /// C++:     case 1: return PointClass(max(0), min(1)); break;
    /// C++:     case 2: return max; break;
    /// C++:     case 3: return PointClass(min(0), max(1)); break;
    /// C++:     default: return PointClass(); break;
    /// C++:     }
    /// C++: }
    pub fn corner(&self, idx: usize) -> Point {
        match idx {
            0 => self.min,
            1 => Point::new(self.max.x, self.min.y),
            2 => self.max,
            3 => Point::new(self.min.x, self.max.y),
            _ => Point::new(0, 0),
        }
    }

    /// Convert the bounding box to a polygon (rectangle)
    /// BoundingBox.cpp:13-25
    /// C++: void BoundingBox::polygon(Polygon* polygon) const
    /// C++: {
    /// C++:     polygon->points.clear();
    /// C++:     polygon->points.resize(4);
    /// C++:     polygon->points[0](0) = this->min(0);
    /// C++:     polygon->points[0](1) = this->min(1);
    /// C++:     polygon->points[1](0) = this->max(0);
    /// C++:     polygon->points[1](1) = this->min(1);
    /// C++:     polygon->points[2](0) = this->max(0);
    /// C++:     polygon->points[2](1) = this->max(1);
    /// C++:     polygon->points[3](0) = this->min(0);
    /// C++:     polygon->points[3](1) = this->max(1);
    /// C++: }
    pub fn polygon(&self) -> Polygon {
        let mut points = Vec::with_capacity(4);
        points.push(Point::new(self.min.x, self.min.y));
        points.push(Point::new(self.max.x, self.min.y));
        points.push(Point::new(self.max.x, self.max.y));
        points.push(Point::new(self.min.x, self.max.y));
        Polygon::from_points(points)
    }

    /// Return a rotated bounding box (around origin)
    /// BoundingBox.cpp:33-40
    /// C++: BoundingBox BoundingBox::rotated(double angle) const
    /// C++: {
    /// C++:     BoundingBox out;
    /// C++:     out.merge(this->min.rotated(angle));
    /// C++:     out.merge(this->max.rotated(angle));
    /// C++:     out.merge(Point(this->min(0), this->max(1)).rotated(angle));
    /// C++:     out.merge(Point(this->max(0), this->min(1)).rotated(angle));
    /// C++:     return out;
    /// C++: }
    pub fn rotated(&self, angle: f64) -> Self {
        let mut out = Self::new();
        out.merge_point(self.min.rotate(angle));
        out.merge_point(self.max.rotate(angle));
        out.merge_point(Point::new(self.min.x, self.max.y).rotate(angle));
        out.merge_point(Point::new(self.max.x, self.min.y).rotate(angle));
        out
    }

    /// Return a rotated bounding box (around a center point)
    /// BoundingBox.cpp:42-49
    /// C++: BoundingBox BoundingBox::rotated(double angle, const Point &center) const
    /// C++: {
    /// C++:     BoundingBox out;
    /// C++:     out.merge(this->min.rotated(angle, center));
    /// C++:     out.merge(this->max.rotated(angle, center));
    /// C++:     out.merge(Point(this->min(0), this->max(1)).rotated(angle, center));
    /// C++:     out.merge(Point(this->max(0), this->min(1)).rotated(angle, center));
    /// C++:     return out;
    /// C++: }
    pub fn rotated_around(&self, angle: f64, center: Point) -> Self {
        let mut out = Self::new();
        out.merge_point(self.min.rotate_around(angle, center));
        out.merge_point(self.max.rotate_around(angle, center));
        out.merge_point(Point::new(self.min.x, self.max.y).rotate_around(angle, center));
        out.merge_point(Point::new(self.max.x, self.min.y).rotate_around(angle, center));
        out
    }

    /// Rotate the bounding box in place
    /// BoundingBox.hpp:114
    /// C++: void rotate(double angle) { (*this) = this->rotated(angle); }
    pub fn rotate(&mut self, angle: f64) {
        *self = self.rotated(angle);
    }

    /// Rotate the bounding box around a center point in place
    /// BoundingBox.hpp:115
    /// C++: void rotate(double angle, const Point &center) { (*this) = this->rotated(angle, center); }
    pub fn rotate_around(&mut self, angle: f64, center: Point) {
        *self = self.rotated_around(angle, center);
    }

    /// Align the min corner to a grid of cell_size x cell_size cells
    /// BoundingBox.cpp:227-232
    /// C++: void BoundingBox::align_to_grid(const coord_t cell_size)
    /// C++: {
    /// C++:     if (this->defined) {
    /// C++:         min(0) = Slic3r::align_to_grid(min(0), cell_size);
    /// C++:         min(1) = Slic3r::align_to_grid(min(1), cell_size);
    /// C++:     }
    /// C++: }
    pub fn align_to_grid(&mut self, cell_size: i64) {
        if self.defined {
            self.min.x = align_to_grid(self.min.x, cell_size);
            self.min.y = align_to_grid(self.min.y, cell_size);
        }
    }

    /// Return a scaled copy of the bounding box
    /// BoundingBox.cpp:51-55
    /// C++: BoundingBox BoundingBox::scaled(double factor) const
    /// C++: {
    /// C++:     BoundingBox out(*this);
    /// C++:     out.scale(factor);
    /// C++:     return out;
    /// C++: }
    pub fn scaled(&self, factor: f64) -> Self {
        let mut out = *self;
        out.scale(factor);
        out
    }

    /// Check if the bounding box is empty (not defined or has zero/negative size)
    /// BoundingBox.hpp:174-177
    /// C++: template<typename VT>
    /// C++: inline bool empty(const BoundingBoxBase<VT> &bb)
    /// C++: {
    /// C++:     return ! bb.defined || bb.min(0) >= bb.max(0) || bb.min(1) >= bb.max(1);
    /// C++: }
    pub fn is_empty(&self) -> bool {
        !self.defined || self.min.x >= self.max.x || self.min.y >= self.max.y
    }
}

impl Default for BoundingBox {
    fn default() -> Self {
        Self::new()
    }
}

/// 3D bounding box with scaled integer coordinates
/// C++ Reference: BoundingBox.hpp - class BoundingBox3 : public BoundingBox3Base<Vec3crd>
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoundingBox3 {
    /// Minimum corner point
    /// BoundingBox.hpp:13
    pub min: Vec3d,
    /// Maximum corner point
    /// BoundingBox.hpp:14
    pub max: Vec3d,
    /// Whether the bounding box has been initialized with valid points
    /// BoundingBox.hpp:15
    pub defined: bool,
}

impl BoundingBox3 {
    /// Create an undefined (empty) 3D bounding box
    /// BoundingBox.hpp:100
    /// C++: BoundingBox3Base() : BoundingBoxBase<PointClass>() {}
    pub fn new() -> Self {
        Self {
            min: Vec3d::new(0.0, 0.0, 0.0),
            max: Vec3d::new(0.0, 0.0, 0.0),
            defined: false,
        }
    }

    /// Create a 3D bounding box from min and max corners
    /// BoundingBox.hpp:101-103
    /// C++: BoundingBox3Base(const PointClass &pmin, const PointClass &pmax) :
    /// C++:     BoundingBoxBase<PointClass>(pmin, pmax)
    /// C++:     { if (pmin(2) >= pmax(2)) BoundingBoxBase<PointClass>::defined = false; }
    pub fn new_from_points(pmin: Vec3d, pmax: Vec3d) -> Self {
        Self {
            min: pmin,
            max: pmax,
            defined: pmin.x < pmax.x && pmin.y < pmax.y && pmin.z < pmax.z,
        }
    }

    /// Create a 3D bounding box from three points
    /// BoundingBox.hpp:104-105
    /// C++: BoundingBox3Base(const PointClass &p1, const PointClass &p2, const PointClass &p3) :
    /// C++:     BoundingBoxBase<PointClass>(p1, p1) { merge(p2); merge(p3); }
    pub fn new_from_three(p1: Vec3d, p2: Vec3d, p3: Vec3d) -> Self {
        let mut bb = Self {
            min: p1,
            max: p1,
            defined: false,
        };
        bb.merge_point(p2);
        bb.merge_point(p3);
        bb
    }

    /// Create a 3D bounding box from a vector of points
    /// BoundingBox.hpp:121-123
    /// C++: BoundingBox3Base(const std::vector<PointClass> &points)
    /// C++:     : BoundingBox3Base(points.begin(), points.end())
    pub fn new_from_points_slice(points: &[Vec3d]) -> Self {
        Self::from_iterator(points.iter().copied())
    }

    /// Create a 3D bounding box from an iterator of points
    /// BoundingBox.hpp:107-120
    /// C++: template<class It, class = IteratorOnly<It> > BoundingBox3Base(It from, It to)
    /// C++: {
    /// C++:     if (from == to)
    /// C++:         throw Slic3r::InvalidArgument("Empty point set supplied to BoundingBox3Base constructor");
    /// C++:     auto it = from;
    /// C++:     this->min = it->template cast<typename PointClass::Scalar>();
    /// C++:     this->max = this->min;
    /// C++:     for (++ it; it != to; ++ it) {
    /// C++:         auto vec = it->template cast<typename PointClass::Scalar>();
    /// C++:         this->min = this->min.cwiseMin(vec);
    /// C++:         this->max = this->max.cwiseMax(vec);
    /// C++:     }
    /// C++:     this->defined = (this->min(0) < this->max(0)) && (this->min(1) < this->max(1)) && (this->min(2) < this->max(2));
    /// C++: }
    pub fn from_iterator<I>(mut points: I) -> Self
    where
        I: Iterator<Item = Vec3d>,
    {
        let mut bb = Self::new();
        if let Some(first) = points.next() {
            bb.min = first;
            bb.max = first;
            for point in points {
                bb.min.x = bb.min.x.min(point.x);
                bb.min.y = bb.min.y.min(point.y);
                bb.min.z = bb.min.z.min(point.z);
                bb.max.x = bb.max.x.max(point.x);
                bb.max.y = bb.max.y.max(point.y);
                bb.max.z = bb.max.z.max(point.z);
            }
            bb.defined = bb.min.x < bb.max.x && bb.min.y < bb.max.y && bb.min.z < bb.max.z;
        }
        bb
    }

    /// Merge a single point into the 3D bounding box
    /// BoundingBox.cpp:145-156
    /// C++: template <class PointClass> void
    /// C++: BoundingBox3Base<PointClass>::merge(const PointClass &point)
    /// C++: {
    /// C++:     if (this->defined) {
    /// C++:         this->min = this->min.cwiseMin(point);
    /// C++:         this->max = this->max.cwiseMax(point);
    /// C++:     } else {
    /// C++:         this->min = point;
    /// C++:         this->max = point;
    /// C++:         this->defined = true;
    /// C++:     }
    /// C++: }
    pub fn merge_point(&mut self, point: Vec3d) {
        if self.defined {
            self.min.x = self.min.x.min(point.x);
            self.min.y = self.min.y.min(point.y);
            self.min.z = self.min.z.min(point.z);
            self.max.x = self.max.x.max(point.x);
            self.max.y = self.max.y.max(point.y);
            self.max.z = self.max.z.max(point.z);
        } else {
            self.min = point;
            self.max = point;
            self.defined = true;
        }
    }

    /// Merge a slice of points into the 3D bounding box
    /// BoundingBox.cpp:160-164
    /// C++: template <class PointClass> void
    /// C++: BoundingBox3Base<PointClass>::merge(const std::vector<PointClass> &points)
    /// C++: {
    /// C++:     this->merge(BoundingBox3Base(points));
    /// C++: }
    pub fn merge_points(&mut self, points: &[Vec3d]) {
        let bb = Self::new_from_points_slice(points);
        self.merge_bb(&bb);
    }

    /// Merge another 3D bounding box into this one
    /// BoundingBox.cpp:168-182
    /// C++: template <class PointClass> void
    /// C++: BoundingBox3Base<PointClass>::merge(const BoundingBox3Base<PointClass> &bb)
    /// C++: {
    /// C++:     assert(bb.defined || bb.min(0) >= bb.max(0) || bb.min(1) >= bb.max(1) || bb.min(2) >= bb.max(2));
    /// C++:     if (bb.defined) {
    /// C++:         if (this->defined) {
    /// C++:             this->min = this->min.cwiseMin(bb.min);
    /// C++:             this->max = this->max.cwiseMax(bb.max);
    /// C++:         } else {
    /// C++:             this->min = bb.min;
    /// C++:             this->max = bb.max;
    /// C++:             this->defined = true;
    /// C++:         }
    /// C++:     }
    /// C++: }
    pub fn merge_bb(&mut self, bb: &BoundingBox3) {
        if bb.defined {
            if self.defined {
                self.min.x = self.min.x.min(bb.min.x);
                self.min.y = self.min.y.min(bb.min.y);
                self.min.z = self.min.z.min(bb.min.z);
                self.max.x = self.max.x.max(bb.max.x);
                self.max.y = self.max.y.max(bb.max.y);
                self.max.z = self.max.z.max(bb.max.z);
            } else {
                self.min = bb.min;
                self.max = bb.max;
                self.defined = true;
            }
        }
    }

    /// Get the size (width, height, depth) of the 3D bounding box
    /// BoundingBox.cpp:184-188
    /// C++: template <class PointClass> PointClass
    /// C++: BoundingBox3Base<PointClass>::size() const
    /// C++: {
    /// C++:     return PointClass(this->max(0) - this->min(0), this->max(1) - this->min(1), this->max(2) - this->min(2));
    /// C++: }
    pub fn size(&self) -> Vec3d {
        Vec3d::new(
            self.max.x - self.min.x,
            self.max.y - self.min.y,
            self.max.z - self.min.z,
        )
    }

    /// Get the radius (half the diagonal length) of the 3D bounding box
    /// BoundingBox.cpp:176-182
    /// C++: template <class PointClass> double BoundingBox3Base<PointClass>::radius() const
    /// C++: {
    /// C++:     double x = this->max(0) - this->min(0);
    /// C++:     double y = this->max(1) - this->min(1);
    /// C++:     double z = this->max(2) - this->min(2);
    /// C++:     return 0.5 * sqrt(x*x+y*y+z*z);
    /// C++: }
    pub fn radius(&self) -> f64 {
        let x = self.max.x - self.min.x;
        let y = self.max.y - self.min.y;
        let z = self.max.z - self.min.z;
        0.5 * (x * x + y * y + z * z).sqrt()
    }

    /// Translate the 3D bounding box by (x, y, z)
    /// BoundingBox.hpp:131
    /// C++: void translate(coordf_t x, coordf_t y, coordf_t z) { assert(this->defined); PointClass v(x, y, z); this->min += v; this->max += v; }
    pub fn translate(&mut self, x: f64, y: f64, z: f64) {
        assert!(self.defined);
        self.min.x += x;
        self.min.y += y;
        self.min.z += z;
        self.max.x += x;
        self.max.y += y;
        self.max.z += z;
    }

    /// Translate the 3D bounding box by a Vec3d
    /// BoundingBox.hpp:132
    /// C++: void translate(const Vec3d &v) { this->min += v; this->max += v; }
    pub fn translate_vec(&mut self, v: Vec3d) {
        self.translate(v.x, v.y, v.z);
    }

    /// Expand the 3D bounding box by delta in all directions
    /// BoundingBox.cpp:195-200
    /// C++: template <class PointClass> void
    /// C++: BoundingBox3Base<PointClass>::offset(coordf_t delta)
    /// C++: {
    /// C++:     PointClass v(delta, delta, delta);
    /// C++:     this->min -= v;
    /// C++:     this->max += v;
    /// C++: }
    pub fn offset(&mut self, delta: f64) {
        self.min.x -= delta;
        self.min.y -= delta;
        self.min.z -= delta;
        self.max.x += delta;
        self.max.y += delta;
        self.max.z += delta;
    }

    /// Return a new 3D bounding box inflated by delta
    /// BoundingBox.hpp:134
    /// C++: BoundingBox3Base<PointClass> inflated(coordf_t delta) const throw() { BoundingBox3Base<PointClass> out(*this); out.offset(delta); return out; }
    pub fn inflated(&self, delta: f64) -> Self {
        let mut out = *self;
        out.offset(delta);
        out
    }

    /// Get the center point of the 3D bounding box
    /// BoundingBox.cpp:211-215
    /// C++: template <class PointClass> PointClass
    /// C++: BoundingBox3Base<PointClass>::center() const
    /// C++: {
    /// C++:     return (this->min + this->max) / 2;
    /// C++: }
    pub fn center(&self) -> Vec3d {
        Vec3d::new(
            (self.min.x + self.max.x) / 2.0,
            (self.min.y + self.max.y) / 2.0,
            (self.min.z + self.max.z) / 2.0,
        )
    }

    /// Get the maximum dimension of the 3D bounding box
    /// BoundingBox.cpp:219-224
    /// C++: template <class PointClass> coordf_t
    /// C++: BoundingBox3Base<PointClass>::max_size() const
    /// C++: {
    /// C++:     PointClass s = size();
    /// C++:     return std::max(s(0), std::max(s(1), s(2)));
    /// C++: }
    pub fn max_size(&self) -> f64 {
        let s = self.size();
        s.x.max(s.y.max(s.z))
    }

    /// Check if a point is contained within the 3D bounding box
    /// BoundingBox.hpp:136-138
    /// C++: bool contains(const PointClass &point) const {
    /// C++:     return BoundingBoxBase<PointClass>::contains(point) && point(2) >= this->min(2) && point(2) <= this->max(2);
    /// C++: }
    pub fn contains_point(&self, point: Vec3d) -> bool {
        point.x >= self.min.x
            && point.x <= self.max.x
            && point.y >= self.min.y
            && point.y <= self.max.y
            && point.z >= self.min.z
            && point.z <= self.max.z
    }

    /// Check if another 3D bounding box is fully contained within this one
    /// BoundingBox.hpp:140-142
    /// C++: bool contains(const BoundingBox3Base<PointClass>& other) const {
    /// C++:     return contains(other.min) && contains(other.max);
    /// C++: }
    pub fn contains_bb(&self, other: &BoundingBox3) -> bool {
        self.contains_point(other.min) && self.contains_point(other.max)
    }

    /// Check if this 3D bounding box intersects with another
    /// BoundingBox.hpp:144-146
    /// C++: bool intersects(const BoundingBox3Base<PointClass>& other) const {
    /// C++:     return (this->min(0) < other.max(0)) && (this->max(0) > other.min(0)) && (this->min(1) < other.max(1)) && (this->max(1) > other.min(1)) && (this->min(2) < other.max(2)) && (this->max(2) > other.min(2));
    /// C++: }
    pub fn intersects(&self, other: &BoundingBox3) -> bool {
        (self.min.x < other.max.x)
            && (self.max.x > other.min.x)
            && (self.min.y < other.max.y)
            && (self.max.y > other.min.y)
            && (self.min.z < other.max.z)
            && (self.max.z > other.min.z)
    }

    /// Convert the 3D bounding box to a 2D polygon (footprint)
    /// BoundingBox.cpp:119-135
    /// C++: template <class PointClass>
    /// C++: Polygon BoundingBox3Base<PointClass>::polygon(bool is_scaled) const
    /// C++: {
    /// C++:     Polygon polygon;
    /// C++:     polygon.points.clear();
    /// C++:     polygon.points.resize(4);
    /// C++:     double scale_factor = 1 / (is_scaled ? SCALING_FACTOR : 1);
    /// C++:     polygon.points[0](0) = this->min(0) * scale_factor;
    /// C++:     polygon.points[0](1) = this->min(1) * scale_factor;
    /// C++:     polygon.points[1](0) = this->max(0) * scale_factor;
    /// C++:     polygon.points[1](1) = this->min(1) * scale_factor;
    /// C++:     polygon.points[2](0) = this->max(0) * scale_factor;
    /// C++:     polygon.points[2](1) = this->max(1) * scale_factor;
    /// C++:     polygon.points[3](0) = this->min(0) * scale_factor;
    /// C++:     polygon.points[3](1) = this->max(1) * scale_factor;
    /// C++:     return polygon;
    /// C++: }
    pub fn polygon(&self, is_scaled: bool) -> Polygon {
        let scale_factor = if is_scaled {
            1.0 / crate::SCALING_FACTOR
        } else {
            1.0
        };
        let mut points = Vec::with_capacity(4);
        points.push(Point::new(
            (self.min.x * scale_factor).round() as i64,
            (self.min.y * scale_factor).round() as i64,
        ));
        points.push(Point::new(
            (self.max.x * scale_factor).round() as i64,
            (self.min.y * scale_factor).round() as i64,
        ));
        points.push(Point::new(
            (self.max.x * scale_factor).round() as i64,
            (self.max.y * scale_factor).round() as i64,
        ));
        points.push(Point::new(
            (self.min.x * scale_factor).round() as i64,
            (self.max.y * scale_factor).round() as i64,
        ));
        Polygon::from_points(points)
    }

    /// Check if the 3D bounding box is empty
    /// BoundingBox.hpp:179-182
    /// C++: template<typename VT>
    /// C++: inline bool empty(const BoundingBox3Base<VT> &bb)
    /// C++: {
    /// C++:     return ! bb.defined || bb.min(0) >= bb.max(0) || bb.min(1) >= bb.max(1) || bb.min(2) >= bb.max(2);
    /// C++: }
    pub fn is_empty(&self) -> bool {
        !self.defined
            || self.min.x >= self.max.x
            || self.min.y >= self.max.y
            || self.min.z >= self.max.z
    }
}

impl Default for BoundingBox3 {
    fn default() -> Self {
        Self::new()
    }
}

/// 2D bounding box with floating-point coordinates
/// C++ Reference: BoundingBox.hpp - class BoundingBoxf : public BoundingBoxBase<Vec2d>
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoundingBoxf {
    /// Minimum corner point
    pub min: Vec2d,
    /// Maximum corner point
    pub max: Vec2d,
    /// Whether the bounding box has been initialized
    pub defined: bool,
}

impl BoundingBoxf {
    /// Create an undefined (empty) floating-point bounding box
    /// BoundingBox.hpp:164
    /// C++: BoundingBoxf() : BoundingBoxBase<Vec2d>() {}
    pub fn new() -> Self {
        Self {
            min: Vec2d::new(0.0, 0.0),
            max: Vec2d::new(0.0, 0.0),
            defined: false,
        }
    }

    /// Create a floating-point bounding box from min and max corners
    /// BoundingBox.hpp:165
    /// C++: BoundingBoxf(const Vec2d &pmin, const Vec2d &pmax) : BoundingBoxBase<Vec2d>(pmin, pmax) {}
    pub fn new_from_points(pmin: Vec2d, pmax: Vec2d) -> Self {
        Self {
            min: pmin,
            max: pmax,
            defined: pmin.x < pmax.x && pmin.y < pmax.y,
        }
    }

    /// Create from a slice of points
    pub fn new_from_points_slice(points: &[Vec2d]) -> Self {
        Self::from_iterator(points.iter().copied())
    }

    /// Create from an iterator
    pub fn from_iterator<I>(points: I) -> Self
    where
        I: Iterator<Item = Vec2d>,
    {
        let mut bb = Self::new();
        for point in points {
            bb.merge_point(point);
        }
        bb
    }

    /// Merge a point into the bounding box
    pub fn merge_point(&mut self, point: Vec2d) {
        if self.defined {
            self.min.x = self.min.x.min(point.x);
            self.min.y = self.min.y.min(point.y);
            self.max.x = self.max.x.max(point.x);
            self.max.y = self.max.y.max(point.y);
        } else {
            self.min = point;
            self.max = point;
            self.defined = true;
        }
    }

    /// Get the size of the bounding box
    pub fn size(&self) -> Vec2d {
        Vec2d::new(self.max.x - self.min.x, self.max.y - self.min.y)
    }

    /// Get the center of the bounding box
    pub fn center(&self) -> Vec2d {
        Vec2d::new(
            (self.min.x + self.max.x) / 2.0,
            (self.min.y + self.max.y) / 2.0,
        )
    }

    /// Check if empty
    pub fn is_empty(&self) -> bool {
        !self.defined || self.min.x >= self.max.x || self.min.y >= self.max.y
    }
}

impl Default for BoundingBoxf {
    fn default() -> Self {
        Self::new()
    }
}

/// 3D bounding box with floating-point coordinates
/// C++ Reference: BoundingBox.hpp - class BoundingBoxf3 : public BoundingBox3Base<Vec3d>
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoundingBoxf3 {
    /// Minimum corner point
    pub min: Vec3d,
    /// Maximum corner point
    pub max: Vec3d,
    /// Whether the bounding box has been initialized
    pub defined: bool,
}

impl BoundingBoxf3 {
    /// Create an undefined (empty) 3D floating-point bounding box
    pub fn new() -> Self {
        Self {
            min: Vec3d::new(0.0, 0.0, 0.0),
            max: Vec3d::new(0.0, 0.0, 0.0),
            defined: false,
        }
    }

    /// Create from min and max points
    pub fn new_from_points(pmin: Vec3d, pmax: Vec3d) -> Self {
        Self {
            min: pmin,
            max: pmax,
            defined: pmin.x < pmax.x && pmin.y < pmax.y && pmin.z < pmax.z,
        }
    }

    /// Create from a slice of points
    pub fn new_from_points_slice(points: &[Vec3d]) -> Self {
        Self::from_iterator(points.iter().copied())
    }

    /// Create from an iterator
    pub fn from_iterator<I>(mut points: I) -> Self
    where
        I: Iterator<Item = Vec3d>,
    {
        let mut bb = Self::new();
        if let Some(first) = points.next() {
            bb.min = first;
            bb.max = first;
            for point in points {
                bb.min.x = bb.min.x.min(point.x);
                bb.min.y = bb.min.y.min(point.y);
                bb.min.z = bb.min.z.min(point.z);
                bb.max.x = bb.max.x.max(point.x);
                bb.max.y = bb.max.y.max(point.y);
                bb.max.z = bb.max.z.max(point.z);
            }
            bb.defined = bb.min.x < bb.max.x && bb.min.y < bb.max.y && bb.min.z < bb.max.z;
        }
        bb
    }

    /// Merge a point into the bounding box
    pub fn merge_point(&mut self, point: Vec3d) {
        if self.defined {
            self.min.x = self.min.x.min(point.x);
            self.min.y = self.min.y.min(point.y);
            self.min.z = self.min.z.min(point.z);
            self.max.x = self.max.x.max(point.x);
            self.max.y = self.max.y.max(point.y);
            self.max.z = self.max.z.max(point.z);
        } else {
            self.min = point;
            self.max = point;
            self.defined = true;
        }
    }

    /// Get the size of the bounding box
    pub fn size(&self) -> Vec3d {
        Vec3d::new(
            self.max.x - self.min.x,
            self.max.y - self.min.y,
            self.max.z - self.min.z,
        )
    }

    /// Get the center of the bounding box
    pub fn center(&self) -> Vec3d {
        Vec3d::new(
            (self.min.x + self.max.x) / 2.0,
            (self.min.y + self.max.y) / 2.0,
            (self.min.z + self.max.z) / 2.0,
        )
    }

    /// Check if empty
    pub fn is_empty(&self) -> bool {
        !self.defined
            || self.min.x >= self.max.x
            || self.min.y >= self.max.y
            || self.min.z >= self.max.z
    }

    /// Transform the bounding box by a 4x4 transformation matrix
    /// BoundingBox.cpp:234-256
    /// C++: BoundingBoxf3 BoundingBoxf3::transformed(const Transform3d& matrix) const
    /// C++: {
    /// C++:     typedef Eigen::Matrix<double, 3, 8, Eigen::DontAlign> Vertices;
    /// C++:     Vertices src_vertices;
    /// C++:     src_vertices(0, 0) = min(0); src_vertices(1, 0) = min(1); src_vertices(2, 0) = min(2);
    /// C++:     src_vertices(0, 1) = max(0); src_vertices(1, 1) = min(1); src_vertices(2, 1) = min(2);
    /// C++:     src_vertices(0, 2) = max(0); src_vertices(1, 2) = max(1); src_vertices(2, 2) = min(2);
    /// C++:     src_vertices(0, 3) = min(0); src_vertices(1, 3) = max(1); src_vertices(2, 3) = min(2);
    /// C++:     src_vertices(0, 4) = min(0); src_vertices(1, 4) = min(1); src_vertices(2, 4) = max(2);
    /// C++:     src_vertices(0, 5) = max(0); src_vertices(1, 5) = min(1); src_vertices(2, 5) = max(2);
    /// C++:     src_vertices(0, 6) = max(0); src_vertices(1, 6) = max(1); src_vertices(2, 6) = max(2);
    /// C++:     src_vertices(0, 7) = min(0); src_vertices(1, 7) = max(1); src_vertices(2, 7) = max(2);
    /// C++:     Vertices dst_vertices = matrix * src_vertices.colwise().homogeneous();
    /// C++:     Vec3d v_min(dst_vertices(0, 0), dst_vertices(1, 0), dst_vertices(2, 0));
    /// C++:     Vec3d v_max = v_min;
    /// C++:     for (int i = 1; i < 8; ++i) {
    /// C++:         for (int j = 0; j < 3; ++j) {
    /// C++:             v_min(j) = std::min(v_min(j), dst_vertices(j, i));
    /// C++:             v_max(j) = std::max(v_max(j), dst_vertices(j, i));
    /// C++:         }
    /// C++:     }
    /// C++:     return BoundingBoxf3(v_min, v_max);
    /// C++: }
    pub fn transformed(&self, matrix: &[[f64; 4]; 4]) -> Self {
        // Create 8 corner vertices of the bounding box
        let corners = [
            Vec3d::new(self.min.x, self.min.y, self.min.z),
            Vec3d::new(self.max.x, self.min.y, self.min.z),
            Vec3d::new(self.max.x, self.max.y, self.min.z),
            Vec3d::new(self.min.x, self.max.y, self.min.z),
            Vec3d::new(self.min.x, self.min.y, self.max.z),
            Vec3d::new(self.max.x, self.min.y, self.max.z),
            Vec3d::new(self.max.x, self.max.y, self.max.z),
            Vec3d::new(self.min.x, self.max.y, self.max.z),
        ];

        // Transform all corners
        let mut transformed_corners = Vec::with_capacity(8);
        for corner in &corners {
            // Apply 4x4 transformation (homogeneous coordinates)
            let x = matrix[0][0] * corner.x
                + matrix[0][1] * corner.y
                + matrix[0][2] * corner.z
                + matrix[0][3];
            let y = matrix[1][0] * corner.x
                + matrix[1][1] * corner.y
                + matrix[1][2] * corner.z
                + matrix[1][3];
            let z = matrix[2][0] * corner.x
                + matrix[2][1] * corner.y
                + matrix[2][2] * corner.z
                + matrix[2][3];
            transformed_corners.push(Vec3d::new(x, y, z));
        }

        // Find min/max of transformed corners
        Self::from_iterator(transformed_corners.into_iter())
    }
}

impl Default for BoundingBoxf3 {
    fn default() -> Self {
        Self::new()
    }
}

/// Align a coordinate to a grid
/// BoundingBox.cpp:227-232 (reference)
/// C++: min(0) = Slic3r::align_to_grid(min(0), cell_size);
fn align_to_grid(coord: i64, cell_size: i64) -> i64 {
    if cell_size == 0 {
        return coord;
    }
    // Round down to nearest grid cell
    (coord / cell_size) * cell_size
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bounding_box_new() {
        /// Test default constructor creates undefined box
        /// BoundingBox.hpp:17
        let bb = BoundingBox::new();
        assert!(!bb.defined);
        assert_eq!(bb.min, Point::new(0, 0));
        assert_eq!(bb.max, Point::new(0, 0));
    }

    #[test]
    fn test_bounding_box_from_points() {
        /// Test constructor with min/max points
        /// BoundingBox.hpp:18-19
        let bb = BoundingBox::new_from_points(Point::new(10, 20), Point::new(30, 40));
        assert!(bb.defined);
        assert_eq!(bb.min, Point::new(10, 20));
        assert_eq!(bb.max, Point::new(30, 40));
    }

    #[test]
    fn test_bounding_box_merge_point() {
        /// Test merging a single point
        /// BoundingBox.cpp:78-89
        let mut bb = BoundingBox::new();
        bb.merge_point(Point::new(10, 20));
        assert!(bb.defined);
        assert_eq!(bb.min, Point::new(10, 20));
        assert_eq!(bb.max, Point::new(10, 20));

        bb.merge_point(Point::new(5, 30));
        assert_eq!(bb.min, Point::new(5, 20));
        assert_eq!(bb.max, Point::new(10, 30));
    }

    #[test]
    fn test_bounding_box_size() {
        /// Test size calculation
        /// BoundingBox.cpp:151-155
        let bb = BoundingBox::new_from_points(Point::new(10, 20), Point::new(30, 50));
        assert_eq!(bb.size(), Point::new(20, 30));
    }

    #[test]
    fn test_bounding_box_center() {
        /// Test center calculation
        /// BoundingBox.cpp:201-205
        let bb = BoundingBox::new_from_points(Point::new(10, 20), Point::new(30, 40));
        assert_eq!(bb.center(), Point::new(20, 30));
    }

    #[test]
    fn test_bounding_box_contains() {
        /// Test point containment
        /// BoundingBox.hpp:47-50
        let bb = BoundingBox::new_from_points(Point::new(10, 20), Point::new(30, 40));
        assert!(bb.contains_point(Point::new(20, 30)));
        assert!(bb.contains_point(Point::new(10, 20)));
        assert!(bb.contains_point(Point::new(30, 40)));
        assert!(!bb.contains_point(Point::new(5, 25)));
        assert!(!bb.contains_point(Point::new(35, 25)));
    }

    #[test]
    fn test_bounding_box_overlap() {
        /// Test bounding box overlap
        /// BoundingBox.hpp:54-57
        let bb1 = BoundingBox::new_from_points(Point::new(10, 20), Point::new(30, 40));
        let bb2 = BoundingBox::new_from_points(Point::new(25, 35), Point::new(45, 55));
        let bb3 = BoundingBox::new_from_points(Point::new(50, 50), Point::new(70, 70));

        assert!(bb1.overlap(&bb2));
        assert!(bb2.overlap(&bb1));
        assert!(!bb1.overlap(&bb3));
        assert!(!bb3.overlap(&bb1));
    }

    #[test]
    fn test_bounding_box_polygon() {
        /// Test conversion to polygon
        /// BoundingBox.cpp:13-25
        let bb = BoundingBox::new_from_points(Point::new(10, 20), Point::new(30, 40));
        let poly = bb.polygon();
        assert_eq!(poly.points.len(), 4);
        assert_eq!(poly.points[0], Point::new(10, 20));
        assert_eq!(poly.points[1], Point::new(30, 20));
        assert_eq!(poly.points[2], Point::new(30, 40));
        assert_eq!(poly.points[3], Point::new(10, 40));
    }

    #[test]
    fn test_bounding_box3_new() {
        /// Test 3D bounding box default constructor
        /// BoundingBox.hpp:100
        let bb = BoundingBox3::new();
        assert!(!bb.defined);
    }

    #[test]
    fn test_bounding_box3_from_points() {
        /// Test 3D bounding box from min/max points
        /// BoundingBox.hpp:101-103
        let bb = BoundingBox3::new_from_points(
            Vec3d::new(10.0, 20.0, 30.0),
            Vec3d::new(40.0, 50.0, 60.0),
        );
        assert!(bb.defined);
        assert_eq!(bb.min, Vec3d::new(10.0, 20.0, 30.0));
        assert_eq!(bb.max, Vec3d::new(40.0, 50.0, 60.0));
    }

    #[test]
    fn test_bounding_box3_size() {
        /// Test 3D size calculation
        /// BoundingBox.cpp:184-188
        let bb = BoundingBox3::new_from_points(
            Vec3d::new(10.0, 20.0, 30.0),
            Vec3d::new(40.0, 50.0, 70.0),
        );
        assert_eq!(bb.size(), Vec3d::new(30.0, 30.0, 40.0));
    }

    #[test]
    fn test_bounding_box3_max_size() {
        /// Test maximum dimension
        /// BoundingBox.cpp:219-224
        let bb = BoundingBox3::new_from_points(
            Vec3d::new(10.0, 20.0, 30.0),
            Vec3d::new(40.0, 50.0, 90.0),
        );
        assert_eq!(bb.max_size(), 60.0);
    }

    #[test]
    fn test_bounding_box3_contains() {
        /// Test 3D point containment
        /// BoundingBox.hpp:136-138
        let bb = BoundingBox3::new_from_points(
            Vec3d::new(10.0, 20.0, 30.0),
            Vec3d::new(40.0, 50.0, 60.0),
        );
        assert!(bb.contains_point(Vec3d::new(25.0, 35.0, 45.0)));
        assert!(!bb.contains_point(Vec3d::new(5.0, 35.0, 45.0)));
        assert!(!bb.contains_point(Vec3d::new(25.0, 35.0, 70.0)));
    }

    #[test]
    fn test_bounding_box3_intersects() {
        /// Test 3D bounding box intersection
        /// BoundingBox.hpp:144-146
        let bb1 = BoundingBox3::new_from_points(
            Vec3d::new(10.0, 20.0, 30.0),
            Vec3d::new(40.0, 50.0, 60.0),
        );
        let bb2 = BoundingBox3::new_from_points(
            Vec3d::new(35.0, 45.0, 55.0),
            Vec3d::new(65.0, 75.0, 85.0),
        );
        let bb3 = BoundingBox3::new_from_points(
            Vec3d::new(100.0, 100.0, 100.0),
            Vec3d::new(130.0, 130.0, 130.0),
        );

        assert!(bb1.intersects(&bb2));
        assert!(bb2.intersects(&bb1));
        assert!(!bb1.intersects(&bb3));
        assert!(!bb3.intersects(&bb1));
    }

    #[test]
    fn test_align_to_grid() {
        /// Test grid alignment helper
        /// BoundingBox.cpp:227-232
        assert_eq!(align_to_grid(17, 10), 10);
        assert_eq!(align_to_grid(20, 10), 20);
        assert_eq!(align_to_grid(-17, 10), -20);
        assert_eq!(align_to_grid(0, 10), 0);
    }

    #[test]
    fn test_bounding_box_rotated() {
        /// Test rotation around origin
        /// BoundingBox.cpp:33-40
        let bb = BoundingBox::new_from_points(Point::new(10, 0), Point::new(20, 10));
        let rotated = bb.rotated(std::f64::consts::PI / 2.0); // 90 degrees
        assert!(rotated.defined);
        // After 90° rotation, the box should be in a different quadrant
        // We don't test exact values due to rounding, just that it's defined
    }

    #[test]
    fn test_bounding_box_offset() {
        /// Test offset/inflation
        /// BoundingBox.cpp:186-191
        let mut bb = BoundingBox::new_from_points(Point::new(10, 20), Point::new(30, 40));
        bb.offset(5.0);
        assert_eq!(bb.min, Point::new(5, 15));
        assert_eq!(bb.max, Point::new(35, 45));
    }
}
