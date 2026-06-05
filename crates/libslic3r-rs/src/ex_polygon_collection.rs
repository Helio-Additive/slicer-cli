//! Faithful 1:1 port of BambuStudio `src/libslic3r/ExPolygonCollection.{hpp,cpp}`.
//!
//! A thin wrapper around an `ExPolygons` (a `Vec<ExPolygon>`) providing
//! conversion operators, affine transforms, containment queries, simplification,
//! convex-hull, line extraction and contour extraction.
//!
//! C++ source layout mirrored here:
//! * `ExPolygonCollection.hpp` — class declaration
//! * `ExPolygonCollection.cpp` — method bodies

// ExPolygonCollection.cpp:1-3
// C++: #include "ExPolygonCollection.hpp"
// C++: #include "Geometry/ConvexHull.hpp"
// C++: #include "BoundingBox.hpp"
use crate::clipper_utils::{diff_pl, union_polygons_ex};
use crate::geometry::convex_hull_points;
use crate::geometry::get_extents as get_extents_expolygons;
use crate::geometry::ExPolygons;
use crate::{BoundingBox, ExPolygon, Line, Point, Polygon, Polyline};

// ExPolygonCollection.hpp:9
// C++: namespace Slic3r {

// ExPolygonCollection.hpp:11-12
// C++: class ExPolygonCollection;
// C++: typedef std::vector<ExPolygonCollection> ExPolygonCollections;
pub type ExPolygonCollections = Vec<ExPolygonCollection>;

/// ExPolygonCollection.hpp:14-35
/// C++: class ExPolygonCollection { public: ExPolygons expolygons; ... };
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ExPolygonCollection {
    // ExPolygonCollection.hpp:17
    // C++: ExPolygons expolygons;
    pub expolygons: ExPolygons,
}

impl ExPolygonCollection {
    /// ExPolygonCollection.hpp:19
    /// C++: ExPolygonCollection() {}
    #[inline]
    pub fn new() -> Self {
        Self {
            expolygons: ExPolygons::new(),
        }
    }

    /// ExPolygonCollection.cpp:7-10
    /// C++: ExPolygonCollection::ExPolygonCollection(const ExPolygon &expolygon)
    /// C++: {
    /// C++:     this->expolygons.push_back(expolygon);
    /// C++: }
    pub fn from_expolygon(expolygon: ExPolygon) -> Self {
        let mut this = Self {
            expolygons: ExPolygons::new(),
        };
        this.expolygons.push(expolygon);
        this
    }

    /// ExPolygonCollection.hpp:21
    /// C++: explicit ExPolygonCollection(const ExPolygons &expolygons) : expolygons(expolygons) {}
    pub fn from_expolygons(expolygons: ExPolygons) -> Self {
        Self { expolygons }
    }

    /// ExPolygonCollection.cpp:12-21
    /// C++: ExPolygonCollection::operator Points() const
    /// C++: {
    /// C++:     Points points;
    /// C++:     Polygons pp = (Polygons)*this;
    /// C++:     for (Polygons::const_iterator poly = pp.begin(); poly != pp.end(); ++poly) {
    /// C++:         for (Points::const_iterator point = poly->points.begin(); point != poly->points.end(); ++point)
    /// C++:             points.push_back(*point);
    /// C++:     }
    /// C++:     return points;
    /// C++: }
    pub fn to_points(&self) -> Vec<Point> {
        let mut points: Vec<Point> = Vec::new();
        let pp: Vec<Polygon> = self.to_polygons();
        for poly in pp.iter() {
            for point in poly.points.iter() {
                points.push(*point);
            }
        }
        points
    }

    /// ExPolygonCollection.cpp:23-33
    /// C++: ExPolygonCollection::operator Polygons() const
    /// C++: {
    /// C++:     Polygons polygons;
    /// C++:     for (ExPolygons::const_iterator it = this->expolygons.begin(); it != this->expolygons.end(); ++it) {
    /// C++:         polygons.push_back(it->contour);
    /// C++:         for (Polygons::const_iterator ith = it->holes.begin(); ith != it->holes.end(); ++ith) {
    /// C++:             polygons.push_back(*ith);
    /// C++:         }
    /// C++:     }
    /// C++:     return polygons;
    /// C++: }
    pub fn to_polygons(&self) -> Vec<Polygon> {
        let mut polygons: Vec<Polygon> = Vec::new();
        for it in self.expolygons.iter() {
            polygons.push(it.contour.clone());
            for ith in it.holes.iter() {
                polygons.push(ith.clone());
            }
        }
        polygons
    }

    /// ExPolygonCollection.cpp:35-38
    /// C++: ExPolygonCollection::operator ExPolygons&()
    /// C++: {
    /// C++:     return this->expolygons;
    /// C++: }
    #[inline]
    pub fn as_expolygons_mut(&mut self) -> &mut ExPolygons {
        &mut self.expolygons
    }

    /// ExPolygonCollection.cpp:40-46
    /// C++: void ExPolygonCollection::scale(double factor)
    /// C++: {
    /// C++:     for (ExPolygons::iterator it = expolygons.begin(); it != expolygons.end(); ++it) {
    /// C++:         (*it).scale(factor);
    /// C++:     }
    /// C++: }
    pub fn scale(&mut self, factor: f64) {
        for it in self.expolygons.iter_mut() {
            it.scale(factor);
        }
    }

    /// ExPolygonCollection.cpp:48-54
    /// C++: void ExPolygonCollection::translate(double x, double y)
    /// C++: {
    /// C++:    for (ExPolygons::iterator it = expolygons.begin(); it != expolygons.end(); ++it) {
    /// C++:         (*it).translate(x, y);
    /// C++:     }
    /// C++: }
    ///
    /// `ExPolygon::translate(double x, double y)` is the inline overload
    /// (ExPolygon.hpp:41) which forwards to `translate(Point(coord_t(x), coord_t(y)))`.
    pub fn translate(&mut self, x: f64, y: f64) {
        for it in self.expolygons.iter_mut() {
            // ExPolygon.hpp:41
            // C++: void translate(double x, double y) { this->translate(Point(coord_t(x), coord_t(y))); }
            it.translate(Point::new(x as i64, y as i64));
        }
    }

    /// ExPolygonCollection.cpp:56-62
    /// C++: void ExPolygonCollection::rotate(double angle, const Point &center)
    /// C++: {
    /// C++:     for (ExPolygons::iterator it = expolygons.begin(); it != expolygons.end(); ++it) {
    /// C++:         (*it).rotate(angle, center);
    /// C++:     }
    /// C++: }
    pub fn rotate(&mut self, angle: f64, center: &Point) {
        for it in self.expolygons.iter_mut() {
            it.rotate_around(angle, *center);
        }
    }

    /// ExPolygonCollection.cpp:64-74
    /// C++: template <class T>
    /// C++: bool ExPolygonCollection::contains(const T &item) const
    /// C++: {
    /// C++:     for (const ExPolygon &poly : this->expolygons)
    /// C++:         if (poly.contains(item))
    /// C++:             return true;
    /// C++:     return false;
    /// C++: }
    /// C++: template bool ExPolygonCollection::contains<Point>(const Point &item) const;
    /// C++: template bool ExPolygonCollection::contains<Line>(const Line &item) const;
    /// C++: template bool ExPolygonCollection::contains<Polyline>(const Polyline &item) const;
    ///
    /// The C++ template is monomorphised over `Point`, `Line`, and `Polyline`.
    /// In Rust this is expressed via the `ExPolygonCollectionContains` trait
    /// implemented for those three types, dispatching to `ExPolygon::contains`.
    pub fn contains<T: ExPolygonCollectionContains>(&self, item: &T) -> bool {
        for poly in self.expolygons.iter() {
            if item.contained_in(poly) {
                return true;
            }
        }
        false
    }

    /// ExPolygonCollection.cpp:76-83
    /// C++: bool ExPolygonCollection::contains_b(const Point &point) const
    /// C++: {
    /// C++:     for (ExPolygons::const_iterator it = this->expolygons.begin(); it != this->expolygons.end(); ++it) {
    /// C++:         if (it->contains_b(point)) return true;
    /// C++:     }
    /// C++:     return false;
    /// C++: }
    ///
    /// `ExPolygon::contains_b` is the legacy "contains including boundary"
    /// predicate; in current BambuStudio it is equivalent to
    /// `ExPolygon::contains(point, border_result = true)`, which is the default
    /// of `contains(const Point&, bool)`. The Rust `ExPolygon::contains_point`
    /// implements exactly that border-inclusive point-in-ExPolygon test.
    pub fn contains_b(&self, point: &Point) -> bool {
        for it in self.expolygons.iter() {
            if it.contains_point(point) {
                return true;
            }
        }
        false
    }

    /// ExPolygonCollection.cpp:85-93
    /// C++: void ExPolygonCollection::simplify(double tolerance)
    /// C++: {
    /// C++:     ExPolygons expp;
    /// C++:     for (ExPolygons::const_iterator it = this->expolygons.begin(); it != this->expolygons.end(); ++it) {
    /// C++:         it->simplify(tolerance, &expp);
    /// C++:     }
    /// C++:     this->expolygons = expp;
    /// C++: }
    ///
    /// `ExPolygon::simplify(tolerance, &expp)` (ExPolygon.cpp:258) appends
    /// `union_ex(this->simplify_p(tolerance))` to `expp`. We reproduce that by
    /// running `ExPolygon::simplify_p` per ExPolygon and unioning the resulting
    /// `Polygons` via `union_polygons_ex` (the crate's faithful equivalent of
    /// `union_ex(simplify_p(...))`).
    pub fn simplify(&mut self, tolerance: f64) {
        let mut expp: ExPolygons = ExPolygons::new();
        for it in self.expolygons.iter() {
            // ExPolygon.cpp:258-261
            // C++: void ExPolygon::simplify(double tolerance, ExPolygons* expolygons) const
            // C++: { append(*expolygons, this->simplify(tolerance)); }
            // ExPolygon.cpp:253-256
            // C++: ExPolygons ExPolygon::simplify(double tolerance) const
            // C++: { return union_ex(this->simplify_p(tolerance)); }
            let pp = it.simplify_p(tolerance);
            expp.extend(union_polygons_ex(&pp));
        }
        self.expolygons = expp;
    }

    /// ExPolygonCollection.cpp:95-102
    /// C++: Polygon ExPolygonCollection::convex_hull() const
    /// C++: {
    /// C++:     Points pp;
    /// C++:     for (ExPolygons::const_iterator it = this->expolygons.begin(); it != this->expolygons.end(); ++it)
    /// C++:         pp.insert(pp.end(), it->contour.points.begin(), it->contour.points.end());
    /// C++:     return Slic3r::Geometry::convex_hull(pp);
    /// C++: }
    pub fn convex_hull(&self) -> Polygon {
        let mut pp: Vec<Point> = Vec::new();
        for it in self.expolygons.iter() {
            pp.extend_from_slice(&it.contour.points);
        }
        convex_hull_points(pp)
    }

    /// ExPolygonCollection.cpp:104-113
    /// C++: Lines ExPolygonCollection::lines() const
    /// C++: {
    /// C++:     Lines lines;
    /// C++:     for (ExPolygons::const_iterator it = this->expolygons.begin(); it != this->expolygons.end(); ++it) {
    /// C++:         Lines ex_lines = it->lines();
    /// C++:         lines.insert(lines.end(), ex_lines.begin(), ex_lines.end());
    /// C++:     }
    /// C++:     return lines;
    /// C++: }
    ///
    /// `ExPolygon::lines()` (ExPolygon.cpp:433) is the per-ExPolygon edge list,
    /// reproduced here by the crate's `to_lines_expoly` free function.
    pub fn lines(&self) -> Vec<Line> {
        use crate::geometry::to_lines_expoly;
        let mut lines: Vec<Line> = Vec::new();
        for it in self.expolygons.iter() {
            let ex_lines = to_lines_expoly(it);
            lines.extend(ex_lines);
        }
        lines
    }

    /// ExPolygonCollection.cpp:115-123
    /// C++: Polygons ExPolygonCollection::contours() const
    /// C++: {
    /// C++:     Polygons contours;
    /// C++:     contours.reserve(this->expolygons.size());
    /// C++:     for (ExPolygons::const_iterator it = this->expolygons.begin(); it != this->expolygons.end(); ++it)
    /// C++:         contours.push_back(it->contour);
    /// C++:     return contours;
    /// C++: }
    pub fn contours(&self) -> Vec<Polygon> {
        let mut contours: Vec<Polygon> = Vec::with_capacity(self.expolygons.len());
        for it in self.expolygons.iter() {
            contours.push(it.contour.clone());
        }
        contours
    }

    /// ExPolygonCollection.cpp:125-129
    /// C++: void ExPolygonCollection::append(const ExPolygons &expp)
    /// C++: {
    /// C++:     this->expolygons.insert(this->expolygons.end(), expp.begin(), expp.end());
    /// C++: }
    pub fn append(&mut self, expp: &ExPolygons) {
        self.expolygons.extend_from_slice(expp);
    }
}

/// Helper trait expressing the `ExPolygonCollection::contains<T>` template
/// monomorphisations (`Point`, `Line`, `Polyline`). Each implementation maps to
/// the corresponding `ExPolygon::contains` overload from `ExPolygon.cpp`.
pub trait ExPolygonCollectionContains {
    fn contained_in(&self, poly: &ExPolygon) -> bool;
}

impl ExPolygonCollectionContains for Point {
    /// ExPolygon.cpp:109-119
    /// C++: bool ExPolygon::contains(const Point &point, bool border_result /* = true */) const
    ///
    /// Border-inclusive point-in-ExPolygon test (inside contour, outside holes).
    fn contained_in(&self, poly: &ExPolygon) -> bool {
        poly.contains_point(self)
    }
}

impl ExPolygonCollectionContains for Line {
    /// ExPolygon.cpp:76-79
    /// C++: bool ExPolygon::contains(const Line &line) const
    /// C++: { return this->contains(Polyline(line.a, line.b)); }
    fn contained_in(&self, poly: &ExPolygon) -> bool {
        let polyline = Polyline::from_points(vec![self.a, self.b]);
        polyline.contained_in(poly)
    }
}

impl ExPolygonCollectionContains for Polyline {
    /// ExPolygon.cpp:81-90
    /// C++: bool ExPolygon::contains(const Polyline &polyline) const
    /// C++: {
    /// C++:     BoundingBox bbox1 = get_extents(*this);
    /// C++:     BoundingBox bbox2 = get_extents(polyline);
    /// C++:     bbox2.inflated(1);
    /// C++:     if (!bbox1.overlap(bbox2))
    /// C++:         return false;
    /// C++:     return diff_pl(polyline, *this).empty();
    /// C++: }
    fn contained_in(&self, poly: &ExPolygon) -> bool {
        // ExPolygon.cpp:83-84
        // C++: BoundingBox bbox1 = get_extents(*this);
        // C++: BoundingBox bbox2 = get_extents(polyline);
        let bbox1 = poly.bounding_box();
        let bbox2 = self.bounding_box();
        // ExPolygon.cpp:85
        // C++: bbox2.inflated(1);
        // NOTE: `BoundingBoxBase::inflated(coordf_t)` (BoundingBox.hpp:48) is a
        // CONST method returning an inflated *copy*; here the C++ discards that
        // return value, so this statement is a no-op and `bbox2` is unchanged.
        // We faithfully reproduce the no-op by not mutating `bbox2`.
        // ExPolygon.cpp:86-87
        // C++: if (!bbox1.overlap(bbox2)) return false;
        // `BoundingBox::overlap` (BoundingBox.hpp:57-60) is the boundary-inclusive
        // overlap test, matching `BoundingBox::intersects`.
        if !bbox1.intersects(&bbox2) {
            return false;
        }
        // ExPolygon.cpp:89
        // C++: return diff_pl(polyline, *this).empty();
        let clip = vec![poly.clone()];
        diff_pl(std::slice::from_ref(self), &clip).is_empty()
    }
}

/// ExPolygonCollection.cpp:131-134
/// C++: BoundingBox get_extents(const ExPolygonCollection &expolygon)
/// C++: {
/// C++:     return get_extents(expolygon.expolygons);
/// C++: }
pub fn get_extents(expolygon: &ExPolygonCollection) -> BoundingBox {
    get_extents_expolygons(&expolygon.expolygons)
}

// ExPolygonCollection.cpp:136
// C++: }  // namespace Slic3r
