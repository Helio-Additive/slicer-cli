//! Axis-aligned bounding boxes for 2D and 3D geometry
//!
//! C++ Reference:
//! - BoundingBox.hpp (full template class hierarchy)
//! - BoundingBox.cpp (template instantiations and methods)
//!
//! This module provides bounding box types for both scaled integer coordinates
//! (BoundingBox, BoundingBox3) and floating-point coordinates (BoundingBoxf, BoundingBoxf3).
//!
//! Type mapping (per BoundingBox.hpp):
//! - `BoundingBox`   : BoundingBoxBase<Point>   -> integer 2D (Vec2crd / i64)
//! - `BoundingBox3`  : BoundingBox3Base<Vec3crd> -> integer 3D (Point3 / i64)
//! - `BoundingBoxf`  : BoundingBoxBase<Vec2d>    -> float 2D (f64)
//! - `BoundingBoxf3` : BoundingBox3Base<Vec3d>   -> float 3D (f64)

use crate::geometry::{Point, Point3, Polygon, Vec2d, Vec3d};

/// 2D bounding box with scaled integer coordinates
/// C++ Reference: BoundingBox.hpp - class BoundingBox : public BoundingBoxBase<Point>
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BoundingBox {
    /// Minimum corner point
    /// BoundingBox.hpp:16
    pub min: Point,
    /// Maximum corner point
    /// BoundingBox.hpp:17
    pub max: Point,
    /// Whether the bounding box has been initialized with valid points
    /// BoundingBox.hpp:18
    pub defined: bool,
}

impl BoundingBox {
    /// Create an undefined (empty) bounding box
    /// BoundingBox.hpp:20
    /// C++: BoundingBoxBase() : min(PointClass::Zero()), max(PointClass::Zero()), defined(false) {}
    pub fn new() -> Self {
        Self {
            min: Point::new(0, 0),
            max: Point::new(0, 0),
            defined: false,
        }
    }

    /// Create a bounding box from min and max corners
    /// BoundingBox.hpp:21-22
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
    /// BoundingBox.hpp:23-24
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
    /// BoundingBox.hpp:33-35
    /// C++: BoundingBoxBase(const std::vector<PointClass> &points)
    /// C++:     : BoundingBoxBase(points.begin(), points.end())
    pub fn new_from_points_slice(points: &[Point]) -> Self {
        Self::from_iterator(points.iter().copied())
    }

    /// Create a bounding box from an iterator of points
    /// BoundingBox.hpp:27-31 + 88-102 (construct template, IncludeBoundary = false)
    /// C++: template<class It, class = IteratorOnly<It>>
    /// C++: BoundingBoxBase(It from, It to) { construct(*this, from, to); }
    /// C++: static void construct(BoundingBoxType &out, It from, It to) {
    /// C++:     if (from != to) {
    /// C++:         auto it = from;
    /// C++:         out.min = it->...; out.max = out.min;
    /// C++:         for (++ it; it != to; ++ it) {
    /// C++:             out.min = out.min.cwiseMin(vec);
    /// C++:             out.max = out.max.cwiseMax(vec);
    /// C++:         }
    /// C++:         out.defined = IncludeBoundary || (out.min.x() < out.max.x() && out.min.y() < out.max.y());
    /// C++:     }
    /// C++: }
    pub fn from_iterator<I>(mut points: I) -> Self
    where
        I: Iterator<Item = Point>,
    {
        let mut bb = Self::new();
        // BoundingBox.hpp:91 - if (from != to)
        if let Some(first) = points.next() {
            // BoundingBox.hpp:93-94
            bb.min = first;
            bb.max = first;
            // BoundingBox.hpp:95-99
            for point in points {
                bb.min.x = bb.min.x.min(point.x);
                bb.min.y = bb.min.y.min(point.y);
                bb.max.x = bb.max.x.max(point.x);
                bb.max.y = bb.max.y.max(point.y);
            }
            // BoundingBox.hpp:100 - IncludeBoundary == false
            bb.defined = bb.min.x < bb.max.x && bb.min.y < bb.max.y;
        }
        bb
    }

    /// Reset the bounding box to undefined state
    /// BoundingBox.hpp:37
    /// C++: void reset() { this->defined = false; this->min = PointClass::Zero(); this->max = PointClass::Zero(); }
    pub fn reset(&mut self) {
        self.defined = false;
        self.min = Point::new(0, 0);
        self.max = Point::new(0, 0);
    }

    /// Merge a single point into the bounding box
    /// BoundingBox.cpp:73-84
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
    /// BoundingBox.cpp:89-93
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
    /// BoundingBox.cpp:97-111
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
        debug_assert!(bb.defined || bb.min.x >= bb.max.x || bb.min.y >= bb.max.y);
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
    /// BoundingBox.cpp:63-68
    /// C++: template <class PointClass> void
    /// C++: BoundingBoxBase<PointClass>::scale(double factor)
    /// C++: {
    /// C++:     this->min *= factor;
    /// C++:     this->max *= factor;
    /// C++: }
    /// NOTE: For integer Point, Eigen's `coord_t *= double` computes in double and
    /// assigns back via static_cast<coord_t>, which truncates toward zero. Use `as i64`.
    pub fn scale(&mut self, factor: f64) {
        self.min.x = (self.min.x as f64 * factor) as i64;
        self.min.y = (self.min.y as f64 * factor) as i64;
        self.max.x = (self.max.x as f64 * factor) as i64;
        self.max.y = (self.max.y as f64 * factor) as i64;
    }

    /// Get the size (width and height) of the bounding box
    /// BoundingBox.cpp:176-180
    /// C++: template <class PointClass> PointClass
    /// C++: BoundingBoxBase<PointClass>::size() const
    /// C++: {
    /// C++:     return PointClass(this->max(0) - this->min(0), this->max(1) - this->min(1));
    /// C++: }
    pub fn size(&self) -> Point {
        Point::new(self.max.x - self.min.x, self.max.y - self.min.y)
    }

    /// Get the radius (half the diagonal length) of the bounding box
    /// BoundingBox.cpp:193-199
    /// C++: template <class PointClass> double BoundingBoxBase<PointClass>::radius() const
    /// C++: {
    /// C++:     assert(this->defined);
    /// C++:     double x = this->max(0) - this->min(0);
    /// C++:     double y = this->max(1) - this->min(1);
    /// C++:     return 0.5 * sqrt(x*x+y*y);
    /// C++: }
    pub fn radius(&self) -> f64 {
        debug_assert!(self.defined);
        let x = (self.max.x - self.min.x) as f64;
        let y = (self.max.y - self.min.y) as f64;
        0.5 * (x * x + y * y).sqrt()
    }

    /// Get the area of the bounding box
    /// BoundingBox.hpp:44
    /// C++: double area() const { return double(this->max(0) - this->min(0)) * (this->max(1) - this->min(1)); }
    pub fn area(&self) -> f64 {
        ((self.max.x - self.min.x) as f64) * ((self.max.y - self.min.y) as f64)
    }

    /// Translate the bounding box by (x, y)
    /// BoundingBox.hpp:45
    /// C++: void translate(coordf_t x, coordf_t y) { assert(this->defined); PointClass v(x, y); this->min += v; this->max += v; }
    /// NOTE: PointClass(x, y) for integer Point truncates the coordf_t values toward zero.
    pub fn translate(&mut self, x: f64, y: f64) {
        debug_assert!(self.defined);
        let dx = x as i64;
        let dy = y as i64;
        self.min.x += dx;
        self.min.y += dy;
        self.max.x += dx;
        self.max.y += dy;
    }

    /// Translate the bounding box by a Vec2d
    /// BoundingBox.hpp:46
    /// C++: void translate(const Vec2d& v0) { PointClass v(v0.x(), v0.y()); this->min += v; this->max += v; }
    pub fn translate_vec(&mut self, v: Vec2d) {
        let dx = v.x as i64;
        let dy = v.y as i64;
        self.min.x += dx;
        self.min.y += dy;
        self.max.x += dx;
        self.max.y += dy;
    }

    /// Expand the bounding box by delta in all directions
    /// BoundingBox.cpp:212-218
    /// C++: template <class PointClass> void
    /// C++: BoundingBoxBase<PointClass>::offset(coordf_t delta)
    /// C++: {
    /// C++:     PointClass v(delta, delta);
    /// C++:     this->min -= v;
    /// C++:     this->max += v;
    /// C++: }
    /// NOTE: PointClass(delta, delta) for integer Point truncates delta toward zero.
    pub fn offset(&mut self, delta: f64) {
        let d = delta as i64;
        self.min.x -= d;
        self.min.y -= d;
        self.max.x += d;
        self.max.y += d;
    }

    /// Return a new bounding box inflated by delta
    /// BoundingBox.hpp:48
    /// C++: BoundingBoxBase<PointClass> inflated(coordf_t delta) const throw() { BoundingBoxBase<PointClass> out(*this); out.offset(delta); return out; }
    pub fn inflated(&self, delta: f64) -> Self {
        let mut out = *self;
        out.offset(delta);
        out
    }

    /// Get the center point of the bounding box
    /// BoundingBox.cpp:231-235
    /// C++: template <class PointClass> PointClass
    /// C++: BoundingBoxBase<PointClass>::center() const
    /// C++: {
    /// C++:     return (this->min + this->max) / 2;
    /// C++: }
    /// NOTE: integer division truncates toward zero (Eigen `Vector2i / int`).
    pub fn center(&self) -> Point {
        Point::new((self.min.x + self.max.x) / 2, (self.min.y + self.max.y) / 2)
    }

    /// Check if a point is contained within the bounding box
    /// BoundingBox.hpp:50-53
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
    /// BoundingBox.hpp:54-56
    /// C++: bool contains(const BoundingBoxBase<PointClass> &other) const {
    /// C++:     return contains(other.min) && contains(other.max);
    /// C++: }
    pub fn contains_bb(&self, other: &BoundingBox) -> bool {
        self.contains_point(other.min) && self.contains_point(other.max)
    }

    /// Check if this bounding box overlaps with another
    /// BoundingBox.hpp:57-60
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

    /// Get a corner point by index (0=min, 1=(max_x,min_y), 2=max, 3=(min_x,max_y))
    /// BoundingBox.hpp:61-71
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

    /// Fill an existing polygon with the bounding box rectangle (out-parameter form)
    /// BoundingBox.cpp:15-27
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
    pub fn polygon_into(&self, polygon: &mut Polygon) {
        polygon.points.clear();
        polygon.points.push(Point::new(self.min.x, self.min.y));
        polygon.points.push(Point::new(self.max.x, self.min.y));
        polygon.points.push(Point::new(self.max.x, self.max.y));
        polygon.points.push(Point::new(self.min.x, self.max.y));
    }

    /// Convert the bounding box to a polygon (rectangle)
    /// BoundingBox.cpp:29-34
    /// C++: Polygon BoundingBox::polygon() const
    /// C++: {
    /// C++:     Polygon p;
    /// C++:     this->polygon(&p);
    /// C++:     return p;
    /// C++: }
    pub fn polygon(&self) -> Polygon {
        let mut p = Polygon::new();
        self.polygon_into(&mut p);
        p
    }

    /// Return a rotated bounding box (around origin)
    /// BoundingBox.cpp:36-44
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
    /// BoundingBox.cpp:46-54
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
    /// BoundingBox.hpp:205
    /// C++: void rotate(double angle) { (*this) = this->rotated(angle); }
    pub fn rotate(&mut self, angle: f64) {
        *self = self.rotated(angle);
    }

    /// Rotate the bounding box around a center point in place
    /// BoundingBox.hpp:206
    /// C++: void rotate(double angle, const Point &center) { (*this) = this->rotated(angle, center); }
    pub fn rotate_around(&mut self, angle: f64, center: Point) {
        *self = self.rotated_around(angle, center);
    }

    /// Align the min corner to a grid of cell_size x cell_size cells
    /// BoundingBox.cpp:257-263
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
    /// BoundingBox.cpp:56-61
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
    /// BoundingBox.hpp:248-252
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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BoundingBox3 {
    /// Minimum corner point (Vec3crd / integer)
    /// BoundingBox.hpp:16
    pub min: Point3,
    /// Maximum corner point (Vec3crd / integer)
    /// BoundingBox.hpp:17
    pub max: Point3,
    /// Whether the bounding box has been initialized with valid points
    /// BoundingBox.hpp:18
    pub defined: bool,
}

impl BoundingBox3 {
    /// Create an undefined (empty) 3D bounding box
    /// BoundingBox.hpp:109
    /// C++: BoundingBox3Base() : BoundingBoxBase<PointClass>() {}
    pub fn new() -> Self {
        Self {
            min: Point3::new(0, 0, 0),
            max: Point3::new(0, 0, 0),
            defined: false,
        }
    }

    /// Create a 3D bounding box from min and max corners
    /// BoundingBox.hpp:110-112
    /// C++: BoundingBox3Base(const PointClass &pmin, const PointClass &pmax) :
    /// C++:     BoundingBoxBase<PointClass>(pmin, pmax)
    /// C++:     { if (pmin(2) >= pmax(2)) BoundingBoxBase<PointClass>::defined = false; }
    /// NOTE: base ctor sets defined = (pmin(0) < pmax(0) && pmin(1) < pmax(1)),
    ///       then this resets it to false if pmin(2) >= pmax(2).
    pub fn new_from_points(pmin: Point3, pmax: Point3) -> Self {
        let mut defined = pmin.x < pmax.x && pmin.y < pmax.y;
        if pmin.z >= pmax.z {
            defined = false;
        }
        Self {
            min: pmin,
            max: pmax,
            defined,
        }
    }

    /// Create a 3D bounding box from three points
    /// BoundingBox.hpp:113-114
    /// C++: BoundingBox3Base(const PointClass &p1, const PointClass &p2, const PointClass &p3) :
    /// C++:     BoundingBoxBase<PointClass>(p1, p1) { merge(p2); merge(p3); }
    /// NOTE: base ctor BoundingBoxBase(p1, p1) sets defined = (p1<p1 ...) = false.
    pub fn new_from_three(p1: Point3, p2: Point3, p3: Point3) -> Self {
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
    /// BoundingBox.hpp:132-134
    /// C++: BoundingBox3Base(const std::vector<PointClass> &points)
    /// C++:     : BoundingBox3Base(points.begin(), points.end())
    pub fn new_from_points_slice(points: &[Point3]) -> Self {
        Self::from_iterator(points.iter().copied())
    }

    /// Create a 3D bounding box from an iterator of points
    /// BoundingBox.hpp:116-130
    /// C++: template<class It, class = IteratorOnly<It> > BoundingBox3Base(It from, It to)
    /// C++: {
    /// C++:     if (from == to)
    /// C++:         throw Slic3r::InvalidArgument("Empty point set supplied to BoundingBox3Base constructor");
    /// C++:     auto it = from;
    /// C++:     this->min = it->...; this->max = this->min;
    /// C++:     for (++ it; it != to; ++ it) {
    /// C++:         this->min = this->min.cwiseMin(vec);
    /// C++:         this->max = this->max.cwiseMax(vec);
    /// C++:     }
    /// C++:     this->defined = (this->min(0) < this->max(0)) && (this->min(1) < this->max(1)) && (this->min(2) < this->max(2));
    /// C++: }
    /// NOTE: C++ throws on empty input; here an empty iterator yields an undefined box.
    pub fn from_iterator<I>(mut points: I) -> Self
    where
        I: Iterator<Item = Point3>,
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
    /// BoundingBox.cpp:137-148
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
    pub fn merge_point(&mut self, point: Point3) {
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
    /// BoundingBox.cpp:152-156
    /// C++: template <class PointClass> void
    /// C++: BoundingBox3Base<PointClass>::merge(const std::vector<PointClass> &points)
    /// C++: {
    /// C++:     this->merge(BoundingBox3Base(points));
    /// C++: }
    pub fn merge_points(&mut self, points: &[Point3]) {
        let bb = Self::new_from_points_slice(points);
        self.merge_bb(&bb);
    }

    /// Merge another 3D bounding box into this one
    /// BoundingBox.cpp:159-173
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
        debug_assert!(
            bb.defined || bb.min.x >= bb.max.x || bb.min.y >= bb.max.y || bb.min.z >= bb.max.z
        );
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
    /// BoundingBox.cpp:185-189
    /// C++: template <class PointClass> PointClass
    /// C++: BoundingBox3Base<PointClass>::size() const
    /// C++: {
    /// C++:     return PointClass(this->max(0) - this->min(0), this->max(1) - this->min(1), this->max(2) - this->min(2));
    /// C++: }
    pub fn size(&self) -> Point3 {
        Point3::new(
            self.max.x - self.min.x,
            self.max.y - self.min.y,
            self.max.z - self.min.z,
        )
    }

    /// Get the radius (half the diagonal length) of the 3D bounding box
    /// BoundingBox.cpp:203-210
    /// C++: template <class PointClass> double BoundingBox3Base<PointClass>::radius() const
    /// C++: {
    /// C++:     double x = this->max(0) - this->min(0);
    /// C++:     double y = this->max(1) - this->min(1);
    /// C++:     double z = this->max(2) - this->min(2);
    /// C++:     return 0.5 * sqrt(x*x+y*y+z*z);
    /// C++: }
    pub fn radius(&self) -> f64 {
        let x = (self.max.x - self.min.x) as f64;
        let y = (self.max.y - self.min.y) as f64;
        let z = (self.max.z - self.min.z) as f64;
        0.5 * (x * x + y * y + z * z).sqrt()
    }

    /// Translate the 3D bounding box by (x, y, z)
    /// BoundingBox.hpp:142
    /// C++: void translate(coordf_t x, coordf_t y, coordf_t z) { assert(this->defined); PointClass v(x, y, z); this->min += v; this->max += v; }
    /// NOTE: PointClass(x, y, z) for integer Vec3crd truncates the coordf_t values toward zero.
    pub fn translate(&mut self, x: f64, y: f64, z: f64) {
        debug_assert!(self.defined);
        let dx = x as i64;
        let dy = y as i64;
        let dz = z as i64;
        self.min.x += dx;
        self.min.y += dy;
        self.min.z += dz;
        self.max.x += dx;
        self.max.y += dy;
        self.max.z += dz;
    }

    /// Translate the 3D bounding box by a Vec3d
    /// BoundingBox.hpp:143
    /// C++: void translate(const Vec3d &v) { this->min += v; this->max += v; }
    /// NOTE: adding a Vec3d to an integer Vec3crd is well-defined in Eigen only for the
    ///       same scalar; in practice this is invoked on float boxes. For the integer
    ///       box we truncate the Vec3d components toward zero.
    pub fn translate_vec(&mut self, v: Vec3d) {
        self.translate(v.x, v.y, v.z);
    }

    /// Expand the 3D bounding box by delta in all directions
    /// BoundingBox.cpp:222-228
    /// C++: template <class PointClass> void
    /// C++: BoundingBox3Base<PointClass>::offset(coordf_t delta)
    /// C++: {
    /// C++:     PointClass v(delta, delta, delta);
    /// C++:     this->min -= v;
    /// C++:     this->max += v;
    /// C++: }
    /// NOTE: PointClass(delta, delta, delta) for integer Vec3crd truncates delta toward zero.
    pub fn offset(&mut self, delta: f64) {
        let d = delta as i64;
        self.min.x -= d;
        self.min.y -= d;
        self.min.z -= d;
        self.max.x += d;
        self.max.y += d;
        self.max.z += d;
    }

    /// Return a new 3D bounding box inflated by delta
    /// BoundingBox.hpp:145
    /// C++: BoundingBox3Base<PointClass> inflated(coordf_t delta) const throw() { BoundingBox3Base<PointClass> out(*this); out.offset(delta); return out; }
    pub fn inflated(&self, delta: f64) -> Self {
        let mut out = *self;
        out.offset(delta);
        out
    }

    /// Get the center point of the 3D bounding box
    /// BoundingBox.cpp:240-246
    /// C++: template <class PointClass> PointClass
    /// C++: BoundingBox3Base<PointClass>::center() const
    /// C++: {
    /// C++:     return (this->min + this->max) / 2;
    /// C++: }
    /// NOTE: integer division truncates toward zero (Eigen `Vector3i / int`).
    pub fn center(&self) -> Point3 {
        Point3::new(
            (self.min.x + self.max.x) / 2,
            (self.min.y + self.max.y) / 2,
            (self.min.z + self.max.z) / 2,
        )
    }

    /// Get the maximum dimension of the 3D bounding box
    /// BoundingBox.cpp:248-255
    /// C++: template <class PointClass> coordf_t
    /// C++: BoundingBox3Base<PointClass>::max_size() const
    /// C++: {
    /// C++:     PointClass s = size();
    /// C++:     return std::max(s(0), std::max(s(1), s(2)));
    /// C++: }
    /// NOTE: max_size() returns coordf_t (double); for an integer box the components
    ///       are widened to double before comparison (matching coordf_t return type).
    pub fn max_size(&self) -> f64 {
        let s = self.size();
        (s.x as f64).max((s.y as f64).max(s.z as f64))
    }

    /// Get the volume of the 3D bounding box
    /// BoundingBox.hpp:148
    /// C++: double volume() const { const PointClass s = size(); return double(s(0)) * double(s(1)) * double(s(2)); }
    pub fn volume(&self) -> f64 {
        let s = self.size();
        (s.x as f64) * (s.y as f64) * (s.z as f64)
    }

    /// Check if a point is contained within the 3D bounding box
    /// BoundingBox.hpp:150-152
    /// C++: bool contains(const PointClass &point) const {
    /// C++:     return BoundingBoxBase<PointClass>::contains(point) && point(2) >= this->min(2) && point(2) <= this->max(2);
    /// C++: }
    pub fn contains_point(&self, point: Point3) -> bool {
        point.x >= self.min.x
            && point.x <= self.max.x
            && point.y >= self.min.y
            && point.y <= self.max.y
            && point.z >= self.min.z
            && point.z <= self.max.z
    }

    /// Check if another 3D bounding box is fully contained within this one
    /// BoundingBox.hpp:154-156
    /// C++: bool contains(const BoundingBox3Base<PointClass>& other) const {
    /// C++:     return contains(other.min) && contains(other.max);
    /// C++: }
    pub fn contains_bb(&self, other: &BoundingBox3) -> bool {
        self.contains_point(other.min) && self.contains_point(other.max)
    }

    /// Check if this 3D bounding box intersects with another
    /// BoundingBox.hpp:158-160
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
    /// BoundingBox.cpp:117-135
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
    /// NOTE: scale_factor uses the C++ libslic3r SCALING_FACTOR (0.000001), so when
    ///       is_scaled is true the factor is 1/0.000001 = 1e6 (matching C++ exactly).
    ///       The coordinate products are assigned into Polygon's integer points, which
    ///       truncate toward zero (Eigen int assignment).
    pub fn polygon(&self, is_scaled: bool) -> Polygon {
        let scale_factor = 1.0 / (if is_scaled { crate::libslic3r::SCALING_FACTOR } else { 1.0 });
        let mut points = Vec::with_capacity(4);
        points.push(Point::new(
            (self.min.x as f64 * scale_factor) as i64,
            (self.min.y as f64 * scale_factor) as i64,
        ));
        points.push(Point::new(
            (self.max.x as f64 * scale_factor) as i64,
            (self.min.y as f64 * scale_factor) as i64,
        ));
        points.push(Point::new(
            (self.max.x as f64 * scale_factor) as i64,
            (self.max.y as f64 * scale_factor) as i64,
        ));
        points.push(Point::new(
            (self.min.x as f64 * scale_factor) as i64,
            (self.max.y as f64 * scale_factor) as i64,
        ));
        Polygon::from_points(points)
    }

    /// Check if the 3D bounding box is empty
    /// BoundingBox.hpp:254-258
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
    /// BoundingBox.hpp:16
    pub min: Vec2d,
    /// Maximum corner point
    /// BoundingBox.hpp:17
    pub max: Vec2d,
    /// Whether the bounding box has been initialized
    /// BoundingBox.hpp:18
    pub defined: bool,
}

impl BoundingBoxf {
    /// Create an undefined (empty) floating-point bounding box
    /// BoundingBox.hpp:235
    /// C++: BoundingBoxf() : BoundingBoxBase<Vec2d>() {}
    pub fn new() -> Self {
        Self {
            min: Vec2d::new(0.0, 0.0),
            max: Vec2d::new(0.0, 0.0),
            defined: false,
        }
    }

    /// Create a floating-point bounding box from min and max corners
    /// BoundingBox.hpp:236
    /// C++: BoundingBoxf(const Vec2d &pmin, const Vec2d &pmax) : BoundingBoxBase<Vec2d>(pmin, pmax) {}
    pub fn new_from_points(pmin: Vec2d, pmax: Vec2d) -> Self {
        Self {
            min: pmin,
            max: pmax,
            defined: pmin.x < pmax.x && pmin.y < pmax.y,
        }
    }

    /// Create from a slice of points
    /// BoundingBox.hpp:237
    /// C++: BoundingBoxf(const std::vector<Vec2d> &points) : BoundingBoxBase<Vec2d>(points) {}
    pub fn new_from_points_slice(points: &[Vec2d]) -> Self {
        Self::from_iterator(points.iter().copied())
    }

    /// Create from an iterator (construct template, IncludeBoundary = false)
    /// BoundingBox.hpp:88-102
    pub fn from_iterator<I>(mut points: I) -> Self
    where
        I: Iterator<Item = Vec2d>,
    {
        let mut bb = Self::new();
        if let Some(first) = points.next() {
            bb.min = first;
            bb.max = first;
            for point in points {
                bb.min.x = bb.min.x.min(point.x);
                bb.min.y = bb.min.y.min(point.y);
                bb.max.x = bb.max.x.max(point.x);
                bb.max.y = bb.max.y.max(point.y);
            }
            bb.defined = bb.min.x < bb.max.x && bb.min.y < bb.max.y;
        }
        bb
    }

    /// Reset the bounding box to undefined state
    /// BoundingBox.hpp:37
    pub fn reset(&mut self) {
        self.defined = false;
        self.min = Vec2d::new(0.0, 0.0);
        self.max = Vec2d::new(0.0, 0.0);
    }

    /// Merge a point into the bounding box
    /// BoundingBox.cpp:73-84 (BoundingBoxBase<Vec2d>::merge instantiated at line 87)
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

    /// Merge a slice of points into the bounding box
    /// BoundingBox.cpp:89-93 (instantiated for Pointfs at line 95)
    pub fn merge_points(&mut self, points: &[Vec2d]) {
        let bb = Self::new_from_points_slice(points);
        self.merge_bb(&bb);
    }

    /// Merge another bounding box into this one
    /// BoundingBox.cpp:97-111 (instantiated for Vec2d at line 114)
    pub fn merge_bb(&mut self, bb: &BoundingBoxf) {
        debug_assert!(bb.defined || bb.min.x >= bb.max.x || bb.min.y >= bb.max.y);
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
    /// BoundingBox.cpp:63-68 (instantiated for Vec2d at line 70)
    /// NOTE: float coordinates, plain multiply (no rounding).
    pub fn scale(&mut self, factor: f64) {
        self.min.x *= factor;
        self.min.y *= factor;
        self.max.x *= factor;
        self.max.y *= factor;
    }

    /// Get the size of the bounding box
    /// BoundingBox.cpp:176-180 (instantiated for Vec2d at line 183)
    pub fn size(&self) -> Vec2d {
        Vec2d::new(self.max.x - self.min.x, self.max.y - self.min.y)
    }

    /// Get the radius (half the diagonal length)
    /// BoundingBox.cpp:193-199 (instantiated for Vec2d at line 201)
    pub fn radius(&self) -> f64 {
        debug_assert!(self.defined);
        let x = self.max.x - self.min.x;
        let y = self.max.y - self.min.y;
        0.5 * (x * x + y * y).sqrt()
    }

    /// Get the area of the bounding box
    /// BoundingBox.hpp:44
    pub fn area(&self) -> f64 {
        (self.max.x - self.min.x) * (self.max.y - self.min.y)
    }

    /// Translate the bounding box by (x, y)
    /// BoundingBox.hpp:45
    pub fn translate(&mut self, x: f64, y: f64) {
        debug_assert!(self.defined);
        self.min.x += x;
        self.min.y += y;
        self.max.x += x;
        self.max.y += y;
    }

    /// Translate the bounding box by a Vec2d
    /// BoundingBox.hpp:46
    pub fn translate_vec(&mut self, v: Vec2d) {
        self.min.x += v.x;
        self.min.y += v.y;
        self.max.x += v.x;
        self.max.y += v.y;
    }

    /// Expand the bounding box by delta in all directions
    /// BoundingBox.cpp:212-218 (instantiated for Vec2d at line 220)
    pub fn offset(&mut self, delta: f64) {
        self.min.x -= delta;
        self.min.y -= delta;
        self.max.x += delta;
        self.max.y += delta;
    }

    /// Return a new bounding box inflated by delta
    /// BoundingBox.hpp:48
    pub fn inflated(&self, delta: f64) -> Self {
        let mut out = *self;
        out.offset(delta);
        out
    }

    /// Get the center of the bounding box
    /// BoundingBox.cpp:231-235 (instantiated for Vec2d at line 238)
    pub fn center(&self) -> Vec2d {
        Vec2d::new(
            (self.min.x + self.max.x) / 2.0,
            (self.min.y + self.max.y) / 2.0,
        )
    }

    /// Check if a point is contained within the bounding box
    /// BoundingBox.hpp:50-53
    pub fn contains_point(&self, point: Vec2d) -> bool {
        point.x >= self.min.x
            && point.x <= self.max.x
            && point.y >= self.min.y
            && point.y <= self.max.y
    }

    /// Check if another bounding box is fully contained within this one
    /// BoundingBox.hpp:54-56
    pub fn contains_bb(&self, other: &BoundingBoxf) -> bool {
        self.contains_point(other.min) && self.contains_point(other.max)
    }

    /// Check if this bounding box overlaps with another
    /// BoundingBox.hpp:57-60
    pub fn overlap(&self, other: &BoundingBoxf) -> bool {
        !(self.max.x < other.min.x
            || self.min.x > other.max.x
            || self.max.y < other.min.y
            || self.min.y > other.max.y)
    }

    /// Get a corner point by index
    /// BoundingBox.hpp:61-71
    pub fn corner(&self, idx: usize) -> Vec2d {
        match idx {
            0 => self.min,
            1 => Vec2d::new(self.max.x, self.min.y),
            2 => self.max,
            3 => Vec2d::new(self.min.x, self.max.y),
            _ => Vec2d::new(0.0, 0.0),
        }
    }

    /// Check if empty
    /// BoundingBox.hpp:248-252
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
    /// BoundingBox.hpp:16
    pub min: Vec3d,
    /// Maximum corner point
    /// BoundingBox.hpp:17
    pub max: Vec3d,
    /// Whether the bounding box has been initialized
    /// BoundingBox.hpp:18
    pub defined: bool,
}

impl BoundingBoxf3 {
    /// Create an undefined (empty) 3D floating-point bounding box
    /// BoundingBox.hpp:242-243 (inherits BoundingBox3Base ctors via `using`)
    pub fn new() -> Self {
        Self {
            min: Vec3d::new(0.0, 0.0, 0.0),
            max: Vec3d::new(0.0, 0.0, 0.0),
            defined: false,
        }
    }

    /// Create from min and max points
    /// BoundingBox.hpp:110-112 (inherited): defined = (x<x && y<y), reset if z>=z
    pub fn new_from_points(pmin: Vec3d, pmax: Vec3d) -> Self {
        let mut defined = pmin.x < pmax.x && pmin.y < pmax.y;
        if pmin.z >= pmax.z {
            defined = false;
        }
        Self {
            min: pmin,
            max: pmax,
            defined,
        }
    }

    /// Create from a slice of points
    /// BoundingBox.hpp:132-134 (inherited)
    pub fn new_from_points_slice(points: &[Vec3d]) -> Self {
        Self::from_iterator(points.iter().copied())
    }

    /// Create from an iterator
    /// BoundingBox.hpp:116-130 (inherited)
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

    /// Reset the bounding box to undefined state
    /// BoundingBox.hpp:37
    pub fn reset(&mut self) {
        self.defined = false;
        self.min = Vec3d::new(0.0, 0.0, 0.0);
        self.max = Vec3d::new(0.0, 0.0, 0.0);
    }

    /// Merge a point into the bounding box
    /// BoundingBox.cpp:137-148 (instantiated for Vec3d at line 150)
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

    /// Merge a slice of points into the bounding box
    /// BoundingBox.cpp:152-156 (instantiated for Pointf3s at line 157)
    pub fn merge_points(&mut self, points: &[Vec3d]) {
        let bb = Self::new_from_points_slice(points);
        self.merge_bb(&bb);
    }

    /// Merge another bounding box into this one
    /// BoundingBox.cpp:159-173 (instantiated for Vec3d at line 174)
    pub fn merge_bb(&mut self, bb: &BoundingBoxf3) {
        debug_assert!(
            bb.defined || bb.min.x >= bb.max.x || bb.min.y >= bb.max.y || bb.min.z >= bb.max.z
        );
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

    /// Get the size of the bounding box
    /// BoundingBox.cpp:185-189 (instantiated for Vec3d at line 191)
    pub fn size(&self) -> Vec3d {
        Vec3d::new(
            self.max.x - self.min.x,
            self.max.y - self.min.y,
            self.max.z - self.min.z,
        )
    }

    /// Get the radius (half the diagonal length)
    /// BoundingBox.cpp:203-210 (instantiated for Vec3d at line 210)
    pub fn radius(&self) -> f64 {
        let x = self.max.x - self.min.x;
        let y = self.max.y - self.min.y;
        let z = self.max.z - self.min.z;
        0.5 * (x * x + y * y + z * z).sqrt()
    }

    /// Translate the bounding box by (x, y, z)
    /// BoundingBox.hpp:142
    pub fn translate(&mut self, x: f64, y: f64, z: f64) {
        debug_assert!(self.defined);
        self.min.x += x;
        self.min.y += y;
        self.min.z += z;
        self.max.x += x;
        self.max.y += y;
        self.max.z += z;
    }

    /// Translate the bounding box by a Vec3d
    /// BoundingBox.hpp:143
    pub fn translate_vec(&mut self, v: Vec3d) {
        self.min.x += v.x;
        self.min.y += v.y;
        self.min.z += v.z;
        self.max.x += v.x;
        self.max.y += v.y;
        self.max.z += v.z;
    }

    /// Expand the bounding box by delta in all directions
    /// BoundingBox.cpp:222-228 (instantiated for Vec3d at line 229)
    pub fn offset(&mut self, delta: f64) {
        self.min.x -= delta;
        self.min.y -= delta;
        self.min.z -= delta;
        self.max.x += delta;
        self.max.y += delta;
        self.max.z += delta;
    }

    /// Return a new bounding box inflated by delta
    /// BoundingBox.hpp:145
    pub fn inflated(&self, delta: f64) -> Self {
        let mut out = *self;
        out.offset(delta);
        out
    }

    /// Get the center of the bounding box
    /// BoundingBox.cpp:240-246 (instantiated for Vec3d at line 246)
    pub fn center(&self) -> Vec3d {
        Vec3d::new(
            (self.min.x + self.max.x) / 2.0,
            (self.min.y + self.max.y) / 2.0,
            (self.min.z + self.max.z) / 2.0,
        )
    }

    /// Get the maximum dimension of the bounding box
    /// BoundingBox.cpp:248-255 (instantiated for Vec3d at line 255)
    pub fn max_size(&self) -> f64 {
        let s = self.size();
        s.x.max(s.y.max(s.z))
    }

    /// Get the volume of the bounding box
    /// BoundingBox.hpp:148
    pub fn volume(&self) -> f64 {
        let s = self.size();
        s.x * s.y * s.z
    }

    /// Check if a point is contained within the bounding box
    /// BoundingBox.hpp:150-152
    pub fn contains_point(&self, point: Vec3d) -> bool {
        point.x >= self.min.x
            && point.x <= self.max.x
            && point.y >= self.min.y
            && point.y <= self.max.y
            && point.z >= self.min.z
            && point.z <= self.max.z
    }

    /// Check if another bounding box is fully contained within this one
    /// BoundingBox.hpp:154-156
    pub fn contains_bb(&self, other: &BoundingBoxf3) -> bool {
        self.contains_point(other.min) && self.contains_point(other.max)
    }

    /// Check if this bounding box intersects with another
    /// BoundingBox.hpp:158-160
    pub fn intersects(&self, other: &BoundingBoxf3) -> bool {
        (self.min.x < other.max.x)
            && (self.max.x > other.min.x)
            && (self.min.y < other.max.y)
            && (self.max.y > other.min.y)
            && (self.min.z < other.max.z)
            && (self.max.z > other.min.z)
    }

    /// Convert the bounding box to a 2D polygon (footprint)
    /// BoundingBox.cpp:117-135 (template BoundingBox3Base<Vec3d>::polygon)
    /// See BoundingBox3::polygon for the scale_factor semantics.
    pub fn polygon(&self, is_scaled: bool) -> Polygon {
        let scale_factor = 1.0 / (if is_scaled { crate::libslic3r::SCALING_FACTOR } else { 1.0 });
        let mut points = Vec::with_capacity(4);
        points.push(Point::new(
            (self.min.x * scale_factor) as i64,
            (self.min.y * scale_factor) as i64,
        ));
        points.push(Point::new(
            (self.max.x * scale_factor) as i64,
            (self.min.y * scale_factor) as i64,
        ));
        points.push(Point::new(
            (self.max.x * scale_factor) as i64,
            (self.max.y * scale_factor) as i64,
        ));
        points.push(Point::new(
            (self.min.x * scale_factor) as i64,
            (self.max.y * scale_factor) as i64,
        ));
        Polygon::from_points(points)
    }

    /// Check if empty
    /// BoundingBox.hpp:254-258
    pub fn is_empty(&self) -> bool {
        !self.defined
            || self.min.x >= self.max.x
            || self.min.y >= self.max.y
            || self.min.z >= self.max.z
    }

    /// Transform the bounding box by a 4x4 transformation matrix
    /// BoundingBox.cpp:265-294
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
    /// `matrix` is in row-major [row][col] order representing the 4x4 Transform3d.
    pub fn transformed(&self, matrix: &[[f64; 4]; 4]) -> Self {
        // BoundingBox.cpp:270-277 - eight source corner vertices
        let src_vertices = [
            Vec3d::new(self.min.x, self.min.y, self.min.z),
            Vec3d::new(self.max.x, self.min.y, self.min.z),
            Vec3d::new(self.max.x, self.max.y, self.min.z),
            Vec3d::new(self.min.x, self.max.y, self.min.z),
            Vec3d::new(self.min.x, self.min.y, self.max.z),
            Vec3d::new(self.max.x, self.min.y, self.max.z),
            Vec3d::new(self.max.x, self.max.y, self.max.z),
            Vec3d::new(self.min.x, self.max.y, self.max.z),
        ];

        // BoundingBox.cpp:279 - dst = matrix * homogeneous(src)
        let mut dst_vertices = [Vec3d::new(0.0, 0.0, 0.0); 8];
        for (i, corner) in src_vertices.iter().enumerate() {
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
            dst_vertices[i] = Vec3d::new(x, y, z);
        }

        // BoundingBox.cpp:281-282 - seed v_min/v_max from vertex 0
        let mut v_min = dst_vertices[0];
        let mut v_max = v_min;

        // BoundingBox.cpp:284-291 - fold remaining vertices
        for vertex in dst_vertices.iter().skip(1) {
            v_min.x = v_min.x.min(vertex.x);
            v_min.y = v_min.y.min(vertex.y);
            v_min.z = v_min.z.min(vertex.z);
            v_max.x = v_max.x.max(vertex.x);
            v_max.y = v_max.y.max(vertex.y);
            v_max.z = v_max.z.max(vertex.z);
        }

        // BoundingBox.cpp:293 - return BoundingBoxf3(v_min, v_max)
        Self::new_from_points(v_min, v_max)
    }
}

impl Default for BoundingBoxf3 {
    fn default() -> Self {
        Self::new()
    }
}

/// Align a coordinate to a grid.
/// Point.hpp:581-590 (Slic3r::align_to_grid used by BoundingBox::align_to_grid).
///
/// The coordinate may be negative; the aligned value will never be bigger than
/// the original one. C++ integer division rounds toward zero, so negatives need
/// the `(coord - spacing + 1) / spacing` correction to round down.
/// C++:
/// inline coord_t align_to_grid(const coord_t coord, const coord_t spacing) {
///     coord_t aligned = (coord < 0) ?
///             ((coord - spacing + 1) / spacing) * spacing :
///             (coord / spacing) * spacing;
///     assert(aligned <= coord);
///     return aligned;
/// }
fn align_to_grid(coord: i64, spacing: i64) -> i64 {
    let aligned = if coord < 0 {
        ((coord - spacing + 1) / spacing) * spacing
    } else {
        (coord / spacing) * spacing
    };
    debug_assert!(aligned <= coord);
    aligned
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bounding_box_new() {
        // Test default constructor creates undefined box
        // BoundingBox.hpp:20
        let bb = BoundingBox::new();
        assert!(!bb.defined);
        assert_eq!(bb.min, Point::new(0, 0));
        assert_eq!(bb.max, Point::new(0, 0));
    }

    #[test]
    fn test_bounding_box_from_points() {
        // Test constructor with min/max points
        // BoundingBox.hpp:21-22
        let bb = BoundingBox::new_from_points(Point::new(10, 20), Point::new(30, 40));
        assert!(bb.defined);
        assert_eq!(bb.min, Point::new(10, 20));
        assert_eq!(bb.max, Point::new(30, 40));
    }

    #[test]
    fn test_bounding_box_merge_point() {
        // Test merging a single point
        // BoundingBox.cpp:73-84
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
        // Test size calculation
        // BoundingBox.cpp:176-180
        let bb = BoundingBox::new_from_points(Point::new(10, 20), Point::new(30, 50));
        assert_eq!(bb.size(), Point::new(20, 30));
    }

    #[test]
    fn test_bounding_box_center() {
        // Test center calculation
        // BoundingBox.cpp:231-235
        let bb = BoundingBox::new_from_points(Point::new(10, 20), Point::new(30, 40));
        assert_eq!(bb.center(), Point::new(20, 30));
    }

    #[test]
    fn test_bounding_box_contains() {
        // Test point containment
        // BoundingBox.hpp:50-53
        let bb = BoundingBox::new_from_points(Point::new(10, 20), Point::new(30, 40));
        assert!(bb.contains_point(Point::new(20, 30)));
        assert!(bb.contains_point(Point::new(10, 20)));
        assert!(bb.contains_point(Point::new(30, 40)));
        assert!(!bb.contains_point(Point::new(5, 25)));
        assert!(!bb.contains_point(Point::new(35, 25)));
    }

    #[test]
    fn test_bounding_box_overlap() {
        // Test bounding box overlap
        // BoundingBox.hpp:57-60
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
        // Test conversion to polygon
        // BoundingBox.cpp:15-27
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
        // Test 3D bounding box default constructor
        // BoundingBox.hpp:109
        let bb = BoundingBox3::new();
        assert!(!bb.defined);
    }

    #[test]
    fn test_bounding_box3_from_points() {
        // Test 3D bounding box from min/max points
        // BoundingBox.hpp:110-112
        let bb = BoundingBox3::new_from_points(Point3::new(10, 20, 30), Point3::new(40, 50, 60));
        assert!(bb.defined);
        assert_eq!(bb.min, Point3::new(10, 20, 30));
        assert_eq!(bb.max, Point3::new(40, 50, 60));
    }

    #[test]
    fn test_bounding_box3_from_points_zero_z() {
        // pmin(2) >= pmax(2) must reset defined to false
        // BoundingBox.hpp:110-112
        let bb = BoundingBox3::new_from_points(Point3::new(10, 20, 30), Point3::new(40, 50, 30));
        assert!(!bb.defined);
    }

    #[test]
    fn test_bounding_box3_size() {
        // Test 3D size calculation
        // BoundingBox.cpp:185-189
        let bb = BoundingBox3::new_from_points(Point3::new(10, 20, 30), Point3::new(40, 50, 70));
        assert_eq!(bb.size(), Point3::new(30, 30, 40));
    }

    #[test]
    fn test_bounding_box3_max_size() {
        // Test maximum dimension
        // BoundingBox.cpp:248-255
        let bb = BoundingBox3::new_from_points(Point3::new(10, 20, 30), Point3::new(40, 50, 90));
        assert_eq!(bb.max_size(), 60.0);
    }

    #[test]
    fn test_bounding_box3_volume() {
        // Test volume
        // BoundingBox.hpp:148
        let bb = BoundingBox3::new_from_points(Point3::new(0, 0, 0), Point3::new(2, 3, 4));
        assert_eq!(bb.volume(), 24.0);
    }

    #[test]
    fn test_bounding_box3_contains() {
        // Test 3D point containment
        // BoundingBox.hpp:150-152
        let bb = BoundingBox3::new_from_points(Point3::new(10, 20, 30), Point3::new(40, 50, 60));
        assert!(bb.contains_point(Point3::new(25, 35, 45)));
        assert!(!bb.contains_point(Point3::new(5, 35, 45)));
        assert!(!bb.contains_point(Point3::new(25, 35, 70)));
    }

    #[test]
    fn test_bounding_box3_intersects() {
        // Test 3D bounding box intersection
        // BoundingBox.hpp:158-160
        let bb1 = BoundingBox3::new_from_points(Point3::new(10, 20, 30), Point3::new(40, 50, 60));
        let bb2 = BoundingBox3::new_from_points(Point3::new(35, 45, 55), Point3::new(65, 75, 85));
        let bb3 =
            BoundingBox3::new_from_points(Point3::new(100, 100, 100), Point3::new(130, 130, 130));

        assert!(bb1.intersects(&bb2));
        assert!(bb2.intersects(&bb1));
        assert!(!bb1.intersects(&bb3));
        assert!(!bb3.intersects(&bb1));
    }

    #[test]
    fn test_align_to_grid() {
        // Test grid alignment helper
        // Point.hpp:581-590
        assert_eq!(align_to_grid(17, 10), 10);
        assert_eq!(align_to_grid(20, 10), 20);
        assert_eq!(align_to_grid(-17, 10), -20);
        assert_eq!(align_to_grid(-20, 10), -20);
        assert_eq!(align_to_grid(0, 10), 0);
    }

    #[test]
    fn test_bounding_box_rotated() {
        // Test rotation around origin
        // BoundingBox.cpp:36-44
        let bb = BoundingBox::new_from_points(Point::new(10, 0), Point::new(20, 10));
        let rotated = bb.rotated(std::f64::consts::PI / 2.0); // 90 degrees
        assert!(rotated.defined);
        // After 90 rotation, the box should be in a different quadrant.
        // We don't test exact values due to rounding, just that it's defined.
    }

    #[test]
    fn test_bounding_box_offset() {
        // Test offset/inflation
        // BoundingBox.cpp:212-218
        let mut bb = BoundingBox::new_from_points(Point::new(10, 20), Point::new(30, 40));
        bb.offset(5.0);
        assert_eq!(bb.min, Point::new(5, 15));
        assert_eq!(bb.max, Point::new(35, 45));
    }

    #[test]
    fn test_bounding_box_scale_truncates() {
        // Integer scale truncates toward zero (matches Eigen int *= double).
        // BoundingBox.cpp:63-68
        let mut bb = BoundingBox::new_from_points(Point::new(3, 3), Point::new(7, 7));
        bb.scale(1.5);
        // 3*1.5 = 4.5 -> 4 ; 7*1.5 = 10.5 -> 10
        assert_eq!(bb.min, Point::new(4, 4));
        assert_eq!(bb.max, Point::new(10, 10));
    }

    #[test]
    fn test_bounding_boxf_full() {
        // Float box: scale/center/offset/radius
        let mut bb = BoundingBoxf::new_from_points(Vec2d::new(0.0, 0.0), Vec2d::new(4.0, 2.0));
        assert_eq!(bb.center(), Vec2d::new(2.0, 1.0));
        assert_eq!(bb.size(), Vec2d::new(4.0, 2.0));
        bb.scale(2.0);
        assert_eq!(bb.max, Vec2d::new(8.0, 4.0));
        bb.offset(1.0);
        assert_eq!(bb.min, Vec2d::new(-1.0, -1.0));
    }

    #[test]
    fn test_bounding_boxf3_transformed_identity() {
        // Identity transform leaves the box unchanged.
        // BoundingBox.cpp:265-294
        let bb = BoundingBoxf3::new_from_points(
            Vec3d::new(1.0, 2.0, 3.0),
            Vec3d::new(4.0, 5.0, 6.0),
        );
        let identity = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ];
        let out = bb.transformed(&identity);
        assert_eq!(out.min, Vec3d::new(1.0, 2.0, 3.0));
        assert_eq!(out.max, Vec3d::new(4.0, 5.0, 6.0));
    }
}
