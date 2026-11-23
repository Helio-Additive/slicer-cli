//! ExPolygon type for polygons with holes.
//!
//! This module provides the ExPolygon type representing a polygon with holes
//! (exterior contour + interior hole contours), mirroring BambuStudio's ExPolygon class.

use super::medial_axis::{compute_medial_axis_thick, MedialAxis, MedialAxisThickPolyline};
use super::thick_polyline::ThickPolylines;
use super::{BoundingBox, Line, Point, Polygon, Polyline};
use crate::{Coord, CoordF};
use serde::{Deserialize, Serialize};
use std::fmt;

/// A polygon with holes (exterior polygon + interior hole polygons).
///
/// The contour is the outer boundary (should be counter-clockwise for positive area).
/// The holes are interior boundaries (should be clockwise).
#[derive(Clone, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct ExPolygon {
    /// The outer contour of the polygon.
    pub contour: Polygon,
    /// The holes (interior contours) of the polygon.
    pub holes: Vec<Polygon>,
}

impl ExPolygon {
    // Create a new ExPolygon with only a contour and no holes.
    #[inline]
    pub fn new(contour: Polygon) -> Self {
        Self {
            contour,
            holes: Vec::new(),
        }
    }

    /// Create a new ExPolygon with a contour and holes.
    #[inline]
    pub fn with_holes(contour: Polygon, holes: Vec<Polygon>) -> Self {
        Self { contour, holes }
    }

    /// Create an empty ExPolygon.
    #[inline]
    pub fn empty() -> Self {
        Self {
            contour: Polygon::new(),
            holes: Vec::new(),
        }
    }

    /// Check if the ExPolygon is empty (no contour points).
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.contour.is_empty()
    }

    /// Get the number of holes.
    #[inline]
    pub fn hole_count(&self) -> usize {
        self.holes.len()
    }

    /// Check if this ExPolygon has any holes.
    #[inline]
    pub fn has_holes(&self) -> bool {
        !self.holes.is_empty()
    }

    /// Get the total number of contours (1 exterior + N holes).
    ///
    /// ExPolygon.hpp (implicit)
    #[inline]
    pub fn num_contours(&self) -> usize {
        1 + self.holes.len()
    }

    /// Add a hole to the ExPolygon.
    #[inline]
    pub fn add_hole(&mut self, hole: Polygon) {
        self.holes.push(hole);
    }

    /// Clear all holes.
    #[inline]
    pub fn clear_holes(&mut self) {
        self.holes.clear();
    }

    /// Calculate the area of the ExPolygon (contour area minus hole areas).
    ///
    /// Faithful port of `double ExPolygon::area() const` (ExPolygon.cpp:52-58).
    /// C++ `Polygon::area()` returns the *signed* area (CCW positive, CW
    /// negative), so we use `signed_area()` here. The C++ body is:
    /// ```cpp
    /// double a = this->contour.area();
    /// for (const Polygon &hole : holes)
    ///     a -= - hole.area();  // holes have negative area  (== a += hole.area())
    /// return a;
    /// ```
    pub fn area(&self) -> CoordF {
        // ExPolygon.cpp:54
        let mut a = self.contour.signed_area();
        // ExPolygon.cpp:55-56  (the double-negative `a -= -x` is `a += x`)
        for hole in &self.holes {
            a -= -hole.signed_area();
        }
        // ExPolygon.cpp:57
        a
    }

    /// Calculate the signed area of the ExPolygon.
    pub fn signed_area(&self) -> CoordF {
        let contour_area = self.contour.signed_area();
        let holes_area: CoordF = self.holes.iter().map(|h| h.signed_area().abs()).sum();
        if contour_area >= 0.0 {
            contour_area - holes_area
        } else {
            contour_area + holes_area
        }
    }

    /// Calculate the total perimeter (contour + all holes).
    pub fn perimeter(&self) -> CoordF {
        let contour_perim = self.contour.perimeter();
        let holes_perim: CoordF = self.holes.iter().map(|h| h.perimeter()).sum();
        contour_perim + holes_perim
    }

    /// Get the bounding box of the ExPolygon (same as contour's bounding box).
    #[inline]
    pub fn bounding_box(&self) -> BoundingBox {
        self.contour.bounding_box()
    }

    /// Check if a point is inside the ExPolygon (inside contour and not inside any hole).
    pub fn contains_point(&self, p: &Point) -> bool {
        if !self.contour.contains_point(p) {
            return false;
        }

        // Check that point is not inside any hole
        for hole in &self.holes {
            if hole.contains_point(p) {
                return false;
            }
        }

        true
    }

    /// Check if a point is on the boundary of the ExPolygon.
    pub fn is_point_on_boundary(&self, p: &Point, tolerance: Coord) -> bool {
        if self.contour.is_point_on_boundary(p, tolerance) {
            return true;
        }

        for hole in &self.holes {
            if hole.is_point_on_boundary(p, tolerance) {
                return true;
            }
        }

        false
    }

    /// Get the centroid of the ExPolygon.
    /// This is an approximation that uses the contour's centroid.
    #[inline]
    pub fn centroid(&self) -> Point {
        self.contour.centroid()
    }

    /// Ensure the contour is counter-clockwise and holes are clockwise.
    pub fn make_canonical(&mut self) {
        self.contour.make_counter_clockwise();
        for hole in &mut self.holes {
            hole.make_clockwise();
        }
    }

    /// Check if the ExPolygon has canonical orientation
    /// (contour CCW, holes CW).
    pub fn is_canonical(&self) -> bool {
        if !self.contour.is_counter_clockwise() {
            return false;
        }

        for hole in &self.holes {
            if !hole.is_clockwise() {
                return false;
            }
        }

        true
    }

    /// Translate the ExPolygon by a vector.
    pub fn translate(&mut self, v: Point) {
        self.contour.translate(v);
        for hole in &mut self.holes {
            hole.translate(v);
        }
    }

    /// Return a translated copy of the ExPolygon.
    pub fn translated(&self, v: Point) -> Self {
        let mut result = self.clone();
        result.translate(v);
        result
    }

    /// Scale the ExPolygon about the origin.
    pub fn scale(&mut self, factor: CoordF) {
        self.contour.scale(factor);
        for hole in &mut self.holes {
            hole.scale(factor);
        }
    }

    /// Return a scaled copy of the ExPolygon.
    pub fn scaled(&self, factor: CoordF) -> Self {
        let mut result = self.clone();
        result.scale(factor);
        result
    }

    /// Rotate the ExPolygon about the origin.
    pub fn rotate(&mut self, angle: CoordF) {
        self.contour.rotate(angle);
        for hole in &mut self.holes {
            hole.rotate(angle);
        }
    }

    /// Return a rotated copy of the ExPolygon.
    pub fn rotated(&self, angle: CoordF) -> Self {
        let mut result = self.clone();
        result.rotate(angle);
        result
    }

    /// Rotate the ExPolygon about a center point.
    pub fn rotate_around(&mut self, angle: CoordF, center: Point) {
        self.contour.rotate_around(angle, center);
        for hole in &mut self.holes {
            hole.rotate_around(angle, center);
        }
    }

    /// Return a copy rotated about a center point.
    pub fn rotated_around(&self, angle: CoordF, center: Point) -> Self {
        let mut result = self.clone();
        result.rotate_around(angle, center);
        result
    }

    /// Faithful port of `ExPolygon::simplify_p(double tolerance)` from
    /// BambuStudio `src/libslic3r/ExPolygon.cpp:231-251`.
    ///
    /// C++:
    /// ```cpp
    /// Polygons ExPolygon::simplify_p(double tolerance) const {
    ///     Polygons pp;
    ///     pp.reserve(this->holes.size() + 1);
    ///     // contour
    ///     {
    ///         Polygon p = this->contour;
    ///         p.points.push_back(p.points.front());
    ///         p.points = MultiPoint::_douglas_peucker(p.points, tolerance);
    ///         p.points.pop_back();
    ///         pp.emplace_back(std::move(p));
    ///     }
    ///     // holes
    ///     for (Polygon p : this->holes) {
    ///         p.points.push_back(p.points.front());
    ///         p.points = MultiPoint::_douglas_peucker(p.points, tolerance);
    ///         p.points.pop_back();
    ///         pp.emplace_back(std::move(p));
    ///     }
    ///     return simplify_polygons(pp);
    /// }
    /// ```
    ///
    /// The final `return simplify_polygons(pp);` (ExPolygon.cpp:250) is faithfully
    /// reproduced via `super::simplify_polygons_clipper(&pp)` — a NonZero Clipper
    /// union of the Douglas-Peucker-simplified contour + holes, which cleans
    /// self-intersections and re-derives holes (ClipperUtils.cpp:1026). This is
    /// byte-exact with the C++ pipeline regardless of whether the caller later
    /// wraps the result in `union_ex(...)`.
    ///
    /// `tolerance` is the UNSCALED resolution in mm (e.g. `print_config.resolution`);
    /// `douglas_peucker` re-scales it internally, mirroring C++ where
    /// `_douglas_peucker` squares the already-scaled tolerance.
    pub fn simplify_p(&self, tolerance: CoordF) -> Vec<Polygon> {
        // ExPolygon.cpp:250 — return simplify_polygons(pp);
        super::simplify_polygons_clipper(&self.simplify_p_dp_rings(tolerance))
    }

    /// The Douglas-Peucker stage of [`ExPolygon::simplify_p`] only: DP-simplify the
    /// contour + each hole and return the rings, WITHOUT the final
    /// `simplify_polygons(pp)` clean/union. Callers that need to route that final
    /// step through the vertex-exact ClipperLib (avoiding geo-clipper's
    /// GEO_CLIPPER_SCALE=1000 micron gridding) use this + `simplify_polygons_clib`
    /// + `union_ex_clib` — the R91 slice-path chain. `simplify_p` itself keeps the
    /// default geo-clipper path so its callers are byte-unchanged.
    pub fn simplify_p_dp_rings(&self, tolerance: CoordF) -> Vec<Polygon> {
        use super::douglas_peucker;

        // ExPolygon.cpp:233-234 — Polygons pp; pp.reserve(this->holes.size() + 1);
        let mut pp: Vec<Polygon> = Vec::with_capacity(self.holes.len() + 1);

        // ExPolygon.cpp:235-242 — contour
        {
            let mut points = self.contour.points().to_vec();
            if !points.is_empty() {
                // p.points.push_back(p.points.front());
                points.push(points[0]);
                // p.points = MultiPoint::_douglas_peucker(p.points, tolerance);
                points = douglas_peucker(&points, tolerance);
                // p.points.pop_back();
                points.pop();
            }
            pp.push(Polygon::from_points(points));
        }

        // ExPolygon.cpp:243-249 — holes
        for hole in &self.holes {
            let mut points = hole.points().to_vec();
            if !points.is_empty() {
                points.push(points[0]);
                points = douglas_peucker(&points, tolerance);
                points.pop();
            }
            pp.push(Polygon::from_points(points));
        }

        pp
    }

    /// R122: identical to [`simplify_p_dp_rings`] but runs the faithful DP
    /// (`multi_point::douglas_peucker`, native pure-double point-to-segment distance)
    /// instead of the crate's rounded-projection `Line::distance_to_squared`. Used
    /// by the gated (F1_UNION) slice-simplify path so near-tolerance vertices match
    /// native `ExPolygon::simplify_p` (which the rounded distance drops/keeps
    /// differently). Default path keeps `simplify_p_dp_rings`.
    pub fn simplify_p_dp_rings_faithful(&self, tolerance: CoordF) -> Vec<Polygon> {
        use crate::multi_point::douglas_peucker;
        let mut pp: Vec<Polygon> = Vec::with_capacity(self.holes.len() + 1);
        {
            let mut points = self.contour.points().to_vec();
            if !points.is_empty() {
                points.push(points[0]);
                points = douglas_peucker(&points, tolerance);
                points.pop();
            }
            pp.push(Polygon::from_points(points));
        }
        for hole in &self.holes {
            let mut points = hole.points().to_vec();
            if !points.is_empty() {
                points.push(points[0]);
                points = douglas_peucker(&points, tolerance);
                points.pop();
            }
            pp.push(Polygon::from_points(points));
        }
        pp
    }

    /// Faithful port of the overload
    /// `void ExPolygon::simplify_p(double tolerance, Polygons* polygons) const`
    /// (BambuStudio `ExPolygon.cpp:225-229`): appends the simplified contour and
    /// hole polygons into `out`.
    pub fn simplify_p_into(&self, tolerance: CoordF, out: &mut Vec<Polygon>) {
        out.extend(self.simplify_p(tolerance));
    }

    /// Simplify the ExPolygon by removing collinear and duplicate points.
    pub fn simplify(&mut self, tolerance: Coord) {
        self.contour.simplify_in_place(tolerance);
        for hole in &mut self.holes {
            hole.simplify_in_place(tolerance);
        }
        // Remove degenerate holes
        self.holes.retain(|h| h.len() >= 3);
    }

    /// Return a simplified copy of the ExPolygon.
    pub fn simplified(&self, tolerance: Coord) -> Self {
        let mut result = self.clone();
        result.simplify(tolerance);
        result
    }

    /// Check if the ExPolygon is valid.
    ///
    /// Faithful port of `bool ExPolygon::is_valid() const` (ExPolygon.cpp:60-67):
    /// the contour must be valid AND counter-clockwise, and every hole must be
    /// valid AND NOT counter-clockwise (i.e. clockwise).
    pub fn is_valid(&self) -> bool {
        // ExPolygon.cpp:62
        if !self.contour.is_valid() || !self.contour.is_counter_clockwise() {
            return false;
        }
        // ExPolygon.cpp:63-65
        for hole in &self.holes {
            if !hole.is_valid() || hole.is_counter_clockwise() {
                return false;
            }
        }
        // ExPolygon.cpp:66
        true
    }

    /// Get all polygons (contour and holes) as a vector.
    pub fn all_polygons(&self) -> Vec<&Polygon> {
        let mut result = Vec::with_capacity(1 + self.holes.len());
        result.push(&self.contour);
        result.extend(self.holes.iter());
        result
    }

    /// Get all polygons as mutable references.
    pub fn all_polygons_mut(&mut self) -> Vec<&mut Polygon> {
        let mut result = Vec::with_capacity(1 + self.holes.len());
        result.push(&mut self.contour);
        result.extend(self.holes.iter_mut());
        result
    }

    /// Convert to a vector of polylines (contour and holes as open paths).
    pub fn to_polylines(&self) -> Vec<Polyline> {
        let mut result = Vec::with_capacity(1 + self.holes.len());
        result.push(self.contour.to_closed_polyline());
        for hole in &self.holes {
            result.push(hole.to_closed_polyline());
        }
        result
    }

    /// Convert to a vector of polygons (contour and holes).
    pub fn to_polygons(&self) -> Vec<Polygon> {
        let mut result = Vec::with_capacity(1 + self.holes.len());
        result.push(self.contour.clone());
        result.extend(self.holes.iter().cloned());
        result
    }

    /// Create a rectangular ExPolygon.
    pub fn rectangle(min: Point, max: Point) -> Self {
        Self::new(Polygon::rectangle(min, max))
    }

    /// Create a square ExPolygon.
    pub fn square(center: Point, half_size: Coord) -> Self {
        Self::new(Polygon::square(center, half_size))
    }

    /// Create a circular ExPolygon approximation.
    pub fn circle(center: Point, radius: Coord, segments: usize) -> Self {
        Self::new(Polygon::circle(center, radius, segments))
    }

    /// Get the total number of points in the ExPolygon.
    pub fn point_count(&self) -> usize {
        self.contour.len() + self.holes.iter().map(|h| h.len()).sum::<usize>()
    }

    /// Find the closest point on any boundary to the given point.
    pub fn closest_point(&self, p: &Point) -> Point {
        let mut closest = self.contour.closest_point(p);
        let mut min_dist = p.distance_squared(&closest);

        for hole in &self.holes {
            let hole_closest = hole.closest_point(p);
            let dist = p.distance_squared(&hole_closest);
            if dist < min_dist {
                min_dist = dist;
                closest = hole_closest;
            }
        }

        closest
    }

    /// Distance from a point to the nearest boundary.
    pub fn distance_to_point(&self, p: &Point) -> CoordF {
        let closest = self.closest_point(p);
        p.distance(&closest)
    }

    /// Remove holes that are too small.
    pub fn remove_small_holes(&mut self, min_area: CoordF) {
        self.holes.retain(|h| h.area() >= min_area);
    }

    /// Compute medial axis with variable width (for gap fill)
    /// ExPolygon.cpp:263-371
    /// C++: void ExPolygon::medial_axis(double min_width, double max_width, ThickPolylines* polylines) const
    ///
    /// With the widths in SCALED units, as the C++ callers pass them: they compute min/max
    /// as doubles from scaled coord_t values (PerimeterGenerator.cpp:972/979, 1329-1330),
    /// and passing mm through a ×SCALING_FACTOR round trip would bring ulp drift. Runs
    /// [`Self::medial_axis_thick_polylines`] (ExPolygon.cpp:263-371) and converts its
    /// polylines to the crate's `ThickPolyline`.
    pub fn medial_axis_scaled(
        &self,
        scaled_min: f64,
        scaled_max: f64,
        polylines: &mut ThickPolylines,
    ) {
        // Widths arrive as the C++ `width` array (two per segment, scaled); the crate's
        // ThickPolyline keeps one per point: the first of the array for point 0, then each
        // segment's end width.
        let mut push = |points: &[(i64, i64)], width: &[f64], endpoints: (bool, bool)| {
            if points.len() < 2 || width.is_empty() {
                return;
            }
            let mut tp = super::thick_polyline::ThickPolyline::with_capacity(points.len());
            for (i, &(x, y)) in points.iter().enumerate() {
                let w = if i == 0 { width[0] } else { width[2 * i - 1] };
                tp.push(Point::new(x, y), w / crate::SCALING_FACTOR);
            }
            tp.endpoints = [endpoints.0, endpoints.1];
            tp.edge_widths = width.iter().map(|w| w / crate::SCALING_FACTOR).collect();
            polylines.push(tp);
        };
        // ExPolygon.cpp:263-371 in Rust, over the Rust port of boost's Voronoi builder.
        let mut pp: Vec<MedialAxisThickPolyline> = Vec::new();
        self.medial_axis_thick_polylines(scaled_min, scaled_max, &mut pp);
        for pl in &pp {
            let points: Vec<(i64, i64)> = pl.points.iter().map(|p| (p.x, p.y)).collect();
            push(&points, &pl.width, pl.endpoints);
        }
    }

    /// ExPolygon.cpp:263-371 `ExPolygon::medial_axis(double min_width, double max_width,
    /// ThickPolylines* polylines)`: the medial axis of `Geometry::MedialAxis`, its endpoints
    /// extended to the contour, the polylines that are too short removed and the rest joined
    /// where they meet. `min_width` and `max_width` are in scaled units, like the widths of
    /// the result, which is appended to `polylines` in the C++ layout
    /// ([`MedialAxisThickPolyline`]).
    ///
    /// The arithmetic follows crates/clipper-z-sys/shim/medial_axis_shim.cpp, the C++ this
    /// replaces, where that differs from the engine's (see the FIDELITY-NOTEs).
    pub fn medial_axis_thick_polylines(
        &self,
        min_width: f64,
        max_width: f64,
        polylines: &mut Vec<MedialAxisThickPolyline>,
    ) {
        // init helper object
        let mut ma = MedialAxis::new(min_width, max_width, self);

        // compute the Voronoi diagram and extract medial axis polylines
        let mut pp: Vec<MedialAxisThickPolyline> = Vec::new();
        ma.build(&mut pp);

        // ExPolygon.cpp:279-283. Find the maximum width returned; we're going to use this for
        // validating and filtering the output segments.
        // `max_w = fmaxf(max_w, *std::max_element(...))`: fmaxf takes and returns float, so
        // both operands are rounded to single precision and `max_w` holds a float's value.
        let mut max_w: f64 = 0.;
        for it in &pp {
            // std::max_element: the first of the largest.
            let mut m = it.width[0];
            for &w in &it.width[1..] {
                if m < w {
                    m = w;
                }
            }
            max_w = (max_w as f32).max(m as f32) as f64;
        }

        // ExPolygon.cpp:285-330. Loop through all returned polylines in order to extend their
        // endpoints to the expolygon boundaries
        let mut removed = false;
        let mut i = 0usize;
        while i < pp.len() {
            let polyline = &mut pp[i];

            // extend initial and final segments of each polyline if they're actual endpoints
            // We assign new endpoints to temporary variables because in case of a single-line
            // polyline, after we extend the start point it will be caught by the intersection()
            // call, so we keep the inner point until we perform the second intersection() as well
            let mut new_front = polyline.points[0];
            let mut new_back = *polyline.points.last().unwrap();
            if polyline.endpoints.0 && !self.ma_on_boundary(new_front, MA_SCALED_EPSILON) {
                let mut p1x = polyline.points[0].x as f64;
                let mut p1y = polyline.points[0].y as f64;
                let mut p2x = polyline.points[1].x as f64;
                let mut p2y = polyline.points[1].y as f64;
                // prevent the line from touching on the other side, otherwise intersection()
                // might return that solution
                if polyline.points.len() == 2 {
                    p2x = (p1x + p2x) * 0.5;
                    p2y = (p1y + p2y) * 0.5;
                }
                // Extend the start of the segment: `p1 -= (p2 - p1).normalized() * max_width`.
                let (nx, ny) = ma_normalized(p2x - p1x, p2y - p1y);
                p1x = (-nx).mul_add(max_width, p1x);
                p1y = (-ny).mul_add(max_width, p1y);
                if ma_polygon_intersection(
                    &self.contour.points,
                    Point::new(p1x as Coord, p1y as Coord),
                    Point::new(p2x as Coord, p2y as Coord),
                    &mut new_front,
                ) {
                    count(&MA_FRONT_EXTENDED);
                }
            }
            if polyline.endpoints.1 && !self.ma_on_boundary(new_back, MA_SCALED_EPSILON) {
                let n = polyline.points.len();
                let mut p1x = polyline.points[n - 2].x as f64;
                let mut p1y = polyline.points[n - 2].y as f64;
                let mut p2x = polyline.points[n - 1].x as f64;
                let mut p2y = polyline.points[n - 1].y as f64;
                // prevent the line from touching on the other side, otherwise intersection()
                // might return that solution
                if polyline.points.len() == 2 {
                    p1x = (p1x + p2x) * 0.5;
                    p1y = (p1y + p2y) * 0.5;
                }
                // Extend the end of the segment: `p2 += (p2 - p1).normalized() * max_width`.
                let (nx, ny) = ma_normalized(p2x - p1x, p2y - p1y);
                p2x = nx.mul_add(max_width, p2x);
                p2y = ny.mul_add(max_width, p2y);
                if ma_polygon_intersection(
                    &self.contour.points,
                    Point::new(p1x as Coord, p1y as Coord),
                    Point::new(p2x as Coord, p2y as Coord),
                    &mut new_back,
                ) {
                    count(&MA_BACK_EXTENDED);
                }
            }
            polyline.points[0] = new_front;
            *polyline.points.last_mut().unwrap() = new_back;

            // remove too short polylines
            // (we can't do this check before endpoints extension and clipping because we don't
            // know how long will the endpoints be extended since it depends on polygon thickness
            // which is variable - extension will be <= max_width/2 on each side)
            if (polyline.endpoints.0 || polyline.endpoints.1) && ma_polyline_length(&polyline.points) < max_w * 2. {
                pp.remove(i);
                removed = true;
                count(&MA_SHORT_REMOVED);
                continue;
            }
            i += 1;
        }

        // ExPolygon.cpp:332-368. If we removed any short polylines we now try to connect
        // consecutive polylines in order to allow loop detection. Note that this algorithm is
        // greedier than MedialAxis::process_edge_neighbors() as it will connect random pairs of
        // polylines even when more than two start from the same point.
        if removed {
            let mut i = 0usize;
            while i < pp.len() {
                if pp[i].endpoints.0 && pp[i].endpoints.1 {
                    i += 1;
                    continue; // optimization
                }

                // find another polyline starting here
                let mut j = i + 1;
                while j < pp.len() {
                    let (head, tail) = pp.split_at_mut(j);
                    let polyline = &mut head[i];
                    let other = &mut tail[0];
                    let polyline_first = polyline.points[0];
                    let polyline_last = *polyline.points.last().unwrap();
                    let other_first = other.points[0];
                    let other_last = *other.points.last().unwrap();
                    if polyline_last == other_last {
                        other.reverse();
                    } else if polyline_first == other_last {
                        polyline.reverse();
                        other.reverse();
                    } else if polyline_first == other_first {
                        polyline.reverse();
                    } else if polyline_last != other_first {
                        j += 1;
                        continue;
                    }

                    polyline.points.extend_from_slice(&other.points[1..]);
                    polyline.width.extend_from_slice(&other.width);
                    polyline.endpoints.1 = other.endpoints.1;
                    debug_assert!(polyline.width.len() == polyline.points.len() * 2 - 2);

                    pp.remove(j);
                    count(&MA_JOINED);
                    j = i + 1; // restart search from i+1
                }
                i += 1;
            }
        }

        polylines.append(&mut pp);
    }

    /// ExPolygon.cpp:121-129 `ExPolygon::on_boundary(point, eps)` as the medial-axis shim has
    /// it (medial_axis_shim.cpp:588-595), over [`ma_polygon_on_boundary`].
    fn ma_on_boundary(&self, point: Point, eps: f64) -> bool {
        if ma_polygon_on_boundary(&self.contour.points, point, eps) {
            return true;
        }
        self.holes.iter().any(|hole| ma_polygon_on_boundary(&hole.points, point, eps))
    }

    pub fn medial_axis(&self, min_width: f64, max_width: f64, polylines: &mut ThickPolylines) {
        // MEDIALAXIS_NATIVE (R274): route through the clipper-z-sys boost-voronoi
        // shim — bit-exact native Voronoi vertex doubles + verbatim MedialAxis
        // walk + the ExPolygon.cpp:281-368 post-processing this rust path OMITS.
        // Widths come back as the C++ 2*(n-1) edge-pair array (scaled); collapse
        // to the crate's per-vertex model with the walk's convention (seed start
        // width for vertex 0, then each segment's end width).
        if crate::faithful_gate("MEDIALAXIS_NATIVE") {
            self.medial_axis_scaled(
                min_width * crate::SCALING_FACTOR,
                max_width * crate::SCALING_FACTOR,
                polylines,
            );
            return;
        }

        // ExPolygon.cpp:266 — Slic3r::Geometry::MedialAxis ma(min_width, max_width, *this);
        // ExPolygon.cpp:269-270 — ThickPolylines pp; ma.build(&pp);
        use super::medial_axis::MedialAxisConfig;
        let config = MedialAxisConfig {
            min_width,
            max_width,
        };
        let mut pp = compute_medial_axis_thick(self, &config);

        // ExPolygon.cpp:281-368 — endpoint extension / short-polyline removal /
        // reconnection OMITTED (ThickPolyline width-representation mismatch; see above).

        // ExPolygon.cpp:370 — polylines->insert(polylines->end(), pp.begin(), pp.end());
        polylines.append(&mut pp);
    }

    /// Compute medial axis as simple polylines (no width information)
    /// ExPolygon.cpp:229-236
    /// C++: void ExPolygon::medial_axis(double min_width, double max_width, Polylines* polylines) const
    pub fn medial_axis_polylines(&self, min_width: f64, max_width: f64) -> Vec<Polyline> {
        // Compute thick polylines first
        // ExPolygon.cpp:231
        // C++: ThickPolylines tp;
        // C++: this->medial_axis(min_width, max_width, &tp);
        let mut tp = Vec::new();
        self.medial_axis(min_width, max_width, &mut tp);

        // Convert to simple polylines (discard width information)
        // ExPolygon.cpp:232-235
        // C++: polylines->reserve(polylines->size() + tp.size());
        // C++: for (auto &pl : tp)
        // C++:     polylines->emplace_back(pl.points);
        let mut result = Vec::with_capacity(tp.len());
        for thick_polyline in tp {
            result.push(Polyline::from_points(thick_polyline.points));
        }
        result
    }
}

impl fmt::Debug for ExPolygon {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "ExPolygon(contour: {} points, {} holes)",
            self.contour.len(),
            self.holes.len()
        )
    }
}

impl fmt::Display for ExPolygon {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "ExPolygon[contour: {}", self.contour)?;
        for (i, hole) in self.holes.iter().enumerate() {
            write!(f, ", hole{}: {}", i, hole)?;
        }
        write!(f, "]")
    }
}

impl From<Polygon> for ExPolygon {
    fn from(polygon: Polygon) -> Self {
        Self::new(polygon)
    }
}

impl From<ExPolygon> for Polygon {
    // Convert to the contour polygon, discarding holes.
    fn from(expoly: ExPolygon) -> Self {
        expoly.contour
    }
}

/// Type alias for a collection of ExPolygons.
pub type ExPolygons = Vec<ExPolygon>;

// ============================================================================
// Free Functions for ExPolygon and ExPolygons
// ============================================================================

/// Count total number of points in an ExPolygon (contour + all holes)
/// ExPolygon.hpp:104-110
pub fn count_points_expoly(expoly: &ExPolygon) -> usize {
    // Start with contour points
    // ExPolygon.hpp:106
    let mut n_points = expoly.contour.len();
    // Add points from all holes
    // ExPolygon.hpp:107-109
    for hole in &expoly.holes {
        n_points += hole.len();
    }
    n_points
}

/// Count total number of points in a collection of ExPolygons
/// ExPolygon.hpp:93-102
pub fn count_points(expolys: &[ExPolygon]) -> usize {
    // Initialize counter
    // ExPolygon.hpp:95
    let mut n_points = 0;
    // Sum points from all ExPolygons
    // ExPolygon.hpp:96-100
    for expoly in expolys {
        n_points += expoly.contour.len();
        for hole in &expoly.holes {
            n_points += hole.len();
        }
    }
    n_points
}

/// Count total number of polygons in a collection of ExPolygons
/// Useful for pre-allocating space when converting ExPolygons to Polygons
/// ExPolygon.hpp:114-120
pub fn number_polygons(expolys: &[ExPolygon]) -> usize {
    // Initialize counter
    // ExPolygon.hpp:116
    let mut n_polygons = 0;
    // Each ExPolygon has 1 contour + N holes
    // ExPolygon.hpp:117-119
    for expoly in expolys {
        n_polygons += expoly.holes.len() + 1;
    }
    n_polygons
}

// ---------------------------------------------------------------------------
// What `ExPolygon::medial_axis_thick_polylines` calls, written as the medial-axis
// shim writes it (crates/clipper-z-sys/shim/medial_axis_shim.cpp:484-581).
//
// FIDELITY-NOTE (all of the `ma_*` functions): the engine's C++ computes these with
// Eigen vectors (`Vec2d` dot, `cross2`, `squaredNorm`, `norm`, `a + t * v`), which
// the compiler does not contract. The shim spells the same steps as scalar
// expressions, and the compiler contracts those: `a * b + c * d` to
// `fma(a, b, c * d)`, `a * b - c * d` to `fma(a, b, -(c * d))`, `a + t * v` to
// `fma(t, v, a)`. The shim is what the engine has been running, so its arithmetic
// is followed, one `mul_add` per contracted expression.
// ---------------------------------------------------------------------------

/// SCALED_EPSILON (libslic3r.h): `scale_(EPSILON)`, 10 in the shim (`kMaScaledEpsilon`).
const MA_SCALED_EPSILON: f64 = 10.0;
/// EPSILON (libslic3r.h), `kMaEpsilon` in the shim.
const MA_EPSILON: f64 = 1e-4;

/// How often the post-processing of `ExPolygon::medial_axis_thick_polylines` took each
/// of its branches, process-wide. For the tests, which assert that they were reached;
/// nothing reads them to take a decision.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MedialAxisCounters {
    /// ExPolygon.cpp:297-306: a first point moved to where the extended first segment
    /// meets the contour.
    pub front_extended: u64,
    /// ExPolygon.cpp:307-316: the same for a last point.
    pub back_extended: u64,
    /// ExPolygon.cpp:324-330: polylines removed for being shorter than twice the largest width.
    pub short_removed: u64,
    /// ExPolygon.cpp:345-365: polylines appended to another after a removal.
    pub joined: u64,
}

static MA_FRONT_EXTENDED: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
static MA_BACK_EXTENDED: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
static MA_SHORT_REMOVED: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
static MA_JOINED: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

#[inline]
fn count(c: &std::sync::atomic::AtomicU64) {
    c.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
}

/// The counters since the process started.
pub fn medial_axis_counters() -> MedialAxisCounters {
    let g = |c: &std::sync::atomic::AtomicU64| c.load(std::sync::atomic::Ordering::Relaxed);
    MedialAxisCounters {
        front_extended: g(&MA_FRONT_EXTENDED),
        back_extended: g(&MA_BACK_EXTENDED),
        short_removed: g(&MA_SHORT_REMOVED),
        joined: g(&MA_JOINED),
    }
}

/// `(b - a).cast<double>().norm()`, the shim's `ma_seg_length` (medial_axis_shim.cpp:487-490).
#[inline]
fn ma_seg_length(a: Point, b: Point) -> f64 {
    let dx = (b.x - a.x) as f64;
    let dy = (b.y - a.y) as f64;
    dx.mul_add(dx, dy * dy).sqrt()
}

/// `Polyline::length()` (Polyline.cpp), the shim's `ma_polyline_length`
/// (medial_axis_shim.cpp:492-497).
fn ma_polyline_length(points: &[Point]) -> f64 {
    let mut len = 0.;
    for i in 1..points.len() {
        len += ma_seg_length(points[i - 1], points[i]);
    }
    len
}

/// Eigen's `normalized()` of `(vx, vy)`: `v / sqrt(squaredNorm)`, and `v` itself where the
/// squared norm is not positive (medial_axis_shim.cpp:643-648).
#[inline]
fn ma_normalized(vx: f64, vy: f64) -> (f64, f64) {
    let z = vx.mul_add(vx, vy * vy);
    if z > 0. {
        let n = z.sqrt();
        (vx / n, vy / n)
    } else {
        (vx, vy)
    }
}

/// Line.hpp:123-150 `line_alg::intersection(l1, l2, &intersection_pt)`, the shim's
/// `ma_line_intersection` (medial_axis_shim.cpp:507-527). The result is
/// `(l1.a + t1 * v1).cast<coord_t>()`: truncated.
fn ma_line_intersection(l1a: Point, l1b: Point, l2a: Point, l2b: Point, out: &mut Point) -> bool {
    let v1x = (l1b.x - l1a.x) as f64;
    let v1y = (l1b.y - l1a.y) as f64;
    let v2x = (l2b.x - l2a.x) as f64;
    let v2y = (l2b.y - l2a.y) as f64;
    let denom = v1x.mul_add(v2y, -(v1y * v2x));
    if denom.abs() < MA_EPSILON {
        // Lines are collinear.
        return false;
    }
    let v12x = (l1a.x - l2a.x) as f64;
    let v12y = (l1a.y - l2a.y) as f64;
    let nume_a = v2x.mul_add(v12y, -(v2y * v12x));
    let nume_b = v1x.mul_add(v12y, -(v1y * v12x));
    let t1 = nume_a / denom;
    let t2 = nume_b / denom;
    if t1 >= 0. && t1 <= 1.0 && t2 >= 0. && t2 <= 1.0 {
        // Get the intersection point.
        out.x = t1.mul_add(v1x, l1a.x as f64) as Coord;
        out.y = t1.mul_add(v1y, l1a.y as f64) as Coord;
        return true;
    }
    false
}

/// Polygon.cpp:188-199 `Polygon::intersection(line, &intersection)`, the shim's
/// `ma_polygon_intersection` (medial_axis_shim.cpp:531-543): the closing edge
/// `(front, back)` first, then the consecutive edges; the first hit wins.
fn ma_polygon_intersection(pts: &[Point], la: Point, lb: Point, out: &mut Point) -> bool {
    if pts.len() < 2 {
        return false;
    }
    if ma_line_intersection(pts[0], pts[pts.len() - 1], la, lb, out) {
        return true;
    }
    for i in 1..pts.len() {
        if ma_line_intersection(pts[i - 1], pts[i], la, lb, out) {
            return true;
        }
    }
    false
}

/// Polygon.cpp:310-345 `Polygon::point_projection(point)`, the shim's `ma_point_projection`
/// (medial_axis_shim.cpp:546-573). The foot of the perpendicular is `floor(v + 0.5)`.
fn ma_point_projection(pts: &[Point], point: Point) -> Point {
    let mut proj = point;
    let mut dmin = f64::MAX;
    for i in 0..pts.len() {
        let pt0 = pts[i];
        let pt1 = pts[if i + 1 == pts.len() { 0 } else { i + 1 }];
        let mut d = ma_seg_length(pt0, point);
        if d < dmin {
            dmin = d;
            proj = pt0;
        }
        d = ma_seg_length(pt1, point);
        if d < dmin {
            dmin = d;
            proj = pt1;
        }
        let v1x = (pt1.x - pt0.x) as f64;
        let v1y = (pt1.y - pt0.y) as f64;
        let div = v1x.mul_add(v1x, v1y * v1y);
        if div > 0. {
            let v2x = (point.x - pt0.x) as f64;
            let v2y = (point.y - pt0.y) as f64;
            let t = v1x.mul_add(v2x, v1y * v2y) / div;
            if t > 0. && t < 1. {
                let foot = Point::new(
                    (t.mul_add(v1x, pt0.x as f64) + 0.5).floor() as Coord,
                    (t.mul_add(v1y, pt0.y as f64) + 0.5).floor() as Coord,
                );
                d = ma_seg_length(foot, point);
                if d < dmin {
                    dmin = d;
                    proj = foot;
                }
            }
        }
    }
    proj
}

/// Polygon.hpp:71-72 `Polygon::on_boundary(point, eps)`:
/// `(point_projection(point) - point).cast<double>().squaredNorm() < eps * eps`
/// (medial_axis_shim.cpp:576-581).
fn ma_polygon_on_boundary(pts: &[Point], point: Point, eps: f64) -> bool {
    let proj = ma_point_projection(pts, point);
    let ex = (proj.x - point.x) as f64;
    let ey = (proj.y - point.y) as f64;
    ex.mul_add(ex, ey * ey) < eps * eps
}

/// Convert an ExPolygon to a collection of Lines (all edges)
/// ExPolygon.hpp:122-133
pub fn to_lines_expoly(src: &ExPolygon) -> Vec<Line> {
    // Pre-allocate space for all edges
    // ExPolygon.hpp:124
    let mut lines = Vec::with_capacity(count_points_expoly(src));

    // Process contour and all holes
    // ExPolygon.hpp:125-131
    for i in 0..=src.holes.len() {
        // Get reference to contour or hole[i-1]
        // ExPolygon.hpp:126
        let poly = if i == 0 {
            &src.contour
        } else {
            &src.holes[i - 1]
        };

        // Create line segments between consecutive points
        // ExPolygon.hpp:127-129
        for j in 0..poly.len() - 1 {
            lines.push(Line::new(poly[j], poly[j + 1]));
        }
        // Close the polygon with line from last to first point
        // ExPolygon.hpp:130
        lines.push(Line::new(*poly.last().unwrap(), poly[0]));
    }
    lines
}

/// Convert a collection of ExPolygons to Lines (all edges)
/// ExPolygon.hpp:135-148
pub fn to_lines(src: &[ExPolygon]) -> Vec<Line> {
    // Pre-allocate space for all edges
    // ExPolygon.hpp:137
    let mut lines = Vec::with_capacity(count_points(src));

    // Process each ExPolygon
    // ExPolygon.hpp:138-145
    for expoly in src {
        // Process contour and all holes
        // ExPolygon.hpp:139-145
        for i in 0..=expoly.holes.len() {
            // Get reference to contour or hole
            // ExPolygon.hpp:140
            let points = if i == 0 {
                &expoly.contour.points
            } else {
                &expoly.holes[i - 1].points
            };

            // Create line segments between consecutive points
            // ExPolygon.hpp:141-143
            for j in 0..points.len() - 1 {
                lines.push(Line::new(points[j], points[j + 1]));
            }
            // Close the polygon
            // ExPolygon.hpp:144
            lines.push(Line::new(*points.last().unwrap(), points[0]));
        }
    }
    lines
}

/// Convert ExPolygons to Points (flattened list of all vertices)
/// ExPolygon.hpp:201-212
pub fn to_points(src: &[ExPolygon]) -> Vec<Point> {
    // Pre-allocate space for all points
    // ExPolygon.hpp:203-204
    let count = count_points(src);
    let mut points = Vec::with_capacity(count);

    // Collect points from all contours and holes
    // ExPolygon.hpp:205-210
    for expoly in src {
        points.extend_from_slice(&expoly.contour.points);
        for hole in &expoly.holes {
            points.extend_from_slice(&hole.points);
        }
    }
    points
}

/// Convert an ExPolygon to Polylines (contour + holes as closed polylines)
/// ExPolygon.hpp:214-229
pub fn to_polylines_expoly(src: &ExPolygon) -> Vec<Polyline> {
    // Allocate space for contour + all holes
    // ExPolygon.hpp:216-217
    let mut polylines = Vec::with_capacity(src.holes.len() + 1);

    // Convert contour to polyline
    // ExPolygon.hpp:218-221
    let mut pl = Polyline::new();
    pl.points = src.contour.points.clone();
    pl.points.push(pl.points[0]); // Close the loop
    polylines.push(pl);

    // Convert each hole to polyline
    // ExPolygon.hpp:222-227
    for hole in &src.holes {
        let mut pl = Polyline::new();
        pl.points = hole.points.clone();
        pl.points.push(pl.points[0]); // Close the loop
        polylines.push(pl);
    }

    polylines
}

/// Convert a collection of ExPolygons to Polylines
/// ExPolygon.hpp:231-248
pub fn to_polylines(src: &[ExPolygon]) -> Vec<Polyline> {
    // Allocate space for all polygons (contours + holes)
    // ExPolygon.hpp:233-234
    let num_polys = number_polygons(src);
    let mut polylines = Vec::with_capacity(num_polys);

    // Process each ExPolygon
    // ExPolygon.hpp:235-246
    for expoly in src {
        // Convert contour
        // ExPolygon.hpp:236-239
        let mut pl = Polyline::new();
        pl.points = expoly.contour.points.clone();
        pl.points.push(pl.points[0]);
        polylines.push(pl);

        // Convert each hole
        // ExPolygon.hpp:240-245
        for hole in &expoly.holes {
            let mut pl = Polyline::new();
            pl.points = hole.points.clone();
            pl.points.push(pl.points[0]);
            polylines.push(pl);
        }
    }

    polylines
}

/// Convert an ExPolygon to Polygons (contour + holes as separate polygons)
/// ExPolygon.hpp:286-293
pub fn to_polygons_expoly(src: &ExPolygon) -> Vec<Polygon> {
    // Allocate space for contour + all holes
    // ExPolygon.hpp:288
    let mut polygons = Vec::with_capacity(1 + src.holes.len());
    // Add contour
    // ExPolygon.hpp:289
    polygons.push(src.contour.clone());
    // Add all holes
    // ExPolygon.hpp:290-292
    polygons.extend(src.holes.iter().cloned());
    polygons
}

/// Convert a collection of ExPolygons to Polygons
/// ExPolygon.hpp:295-304
pub fn to_polygons(src: &[ExPolygon]) -> Vec<Polygon> {
    // Pre-allocate space for all polygons
    // ExPolygon.hpp:297-298
    let num_polys = number_polygons(src);
    let mut polygons = Vec::with_capacity(num_polys);

    // Collect all contours and holes
    // ExPolygon.hpp:299-303
    for expoly in src {
        polygons.push(expoly.contour.clone());
        polygons.extend(expoly.holes.iter().cloned());
    }
    polygons
}

/// Convert Polygons to ExPolygons (simple conversion, no hole detection)
/// Each Polygon becomes an ExPolygon with no holes
/// ExPolygon.hpp:352-359
pub fn to_expolygons_simple(src: &[Polygon]) -> Vec<ExPolygon> {
    // Pre-allocate space
    // ExPolygon.hpp:354-355
    let mut ex_polys = Vec::with_capacity(src.len());
    // Convert each Polygon to ExPolygon
    // ExPolygon.hpp:356-358
    for poly in src {
        ex_polys.push(ExPolygon::new(poly.clone()));
    }
    ex_polys
}

/// Translate all ExPolygons in a collection by a vector
/// ExPolygon.hpp:380-383
pub fn translate_expolygons(expolys: &mut [ExPolygon], p: Point) {
    // Translate each ExPolygon
    // ExPolygon.hpp:382
    for expoly in expolys {
        expoly.translate(p);
    }
}

/// Rotate all ExPolygons in a collection by an angle (radians)
/// ExPolygon.hpp:437-441
pub fn expolygons_rotate(expolys: &mut [ExPolygon], angle: CoordF) {
    // Rotate each ExPolygon
    // ExPolygon.hpp:438-439
    for expoly in expolys {
        expoly.rotate(angle);
    }
}

/// Append ExPolygon's contour and holes to a Polygon vector
/// ExPolygon.hpp:385-390
pub fn polygons_append_expoly(dst: &mut Vec<Polygon>, src: &ExPolygon) {
    // Reserve space for contour + holes
    // ExPolygon.hpp:387
    dst.reserve(dst.len() + src.holes.len() + 1);
    // Add contour
    // ExPolygon.hpp:388
    dst.push(src.contour.clone());
    // Add all holes
    // ExPolygon.hpp:389
    dst.extend(src.holes.iter().cloned());
}

/// Append all ExPolygons' contours and holes to a Polygon vector
/// ExPolygon.hpp:392-399
pub fn polygons_append(dst: &mut Vec<Polygon>, src: &[ExPolygon]) {
    // Reserve space for all polygons
    // ExPolygon.hpp:394
    dst.reserve(dst.len() + number_polygons(src));
    // Add all contours and holes
    // ExPolygon.hpp:395-398
    for expoly in src {
        dst.push(expoly.contour.clone());
        dst.extend(expoly.holes.iter().cloned());
    }
}

/// Append one ExPolygons collection to another
/// ExPolygon.hpp:421-424
pub fn expolygons_append(dst: &mut Vec<ExPolygon>, src: &[ExPolygon]) {
    // Extend destination with source
    // ExPolygon.hpp:423
    dst.extend_from_slice(src);
}

/// Check if any ExPolygon in a collection contains a point
/// ExPolygon.hpp:443-449
pub fn expolygons_contain(expolys: &[ExPolygon], pt: Point) -> bool {
    // Check each ExPolygon
    // ExPolygon.hpp:444-447
    for expoly in expolys {
        if expoly.contains_point(&pt) {
            return true;
        }
    }
    // Point not contained in any ExPolygon
    // ExPolygon.hpp:448
    false
}

/// Simplify all ExPolygons in a collection
/// ExPolygon.hpp:451-458
pub fn expolygons_simplify(expolys: &[ExPolygon], tolerance: CoordF) -> Vec<ExPolygon> {
    // Pre-allocate output
    // ExPolygon.hpp:453
    let mut out = Vec::with_capacity(expolys.len());
    // Simplify each ExPolygon
    // ExPolygon.hpp:454-456
    // Convert tolerance from mm to scaled units
    use crate::scale;
    let tolerance_scaled = scale(tolerance);
    for expoly in expolys {
        out.push(expoly.simplified(tolerance_scaled));
    }
    out
}

/// Calculate total area of all ExPolygons in a collection
/// ExPolygon.hpp:488
pub fn area_expolygons(polys: &[ExPolygon]) -> CoordF {
    // Sum areas of all ExPolygons
    // ExPolygon.hpp:488
    polys.iter().map(|p| p.area()).sum()
}

/// Get bounding box of an ExPolygon
/// ExPolygon.hpp:472 (declared, implemented in ExPolygon.cpp)
pub fn get_extents_expoly(expolygon: &ExPolygon) -> BoundingBox {
    // Use the ExPolygon's bounding_box method
    expolygon.bounding_box()
}

/// Get bounding box of a collection of ExPolygons
/// ExPolygon.hpp:473 (declared, implemented in ExPolygon.cpp)
pub fn get_extents(expolygons: &[ExPolygon]) -> BoundingBox {
    // Start with empty bounding box
    let mut bbox = BoundingBox::new();
    // Merge all ExPolygon bounding boxes
    for expoly in expolygons {
        bbox.merge(&expoly.bounding_box());
    }
    bbox
}

/// Get vector of bounding boxes (one per ExPolygon)
/// ExPolygon.hpp:476 (declared, implemented in ExPolygon.cpp)
pub fn get_extents_vector(expolygons: &[ExPolygon]) -> Vec<BoundingBox> {
    // Calculate bounding box for each ExPolygon
    expolygons
        .iter()
        .map(|expoly| expoly.bounding_box())
        .collect()
}

/// Check if two ExPolygons collections overlap
/// ExPolygon.hpp:467 (declared, implemented in ExPolygon.cpp)
/// Note: This is a placeholder - full implementation requires Clipper intersection
pub fn overlaps_expolygons(expolys1: &[ExPolygon], expolys2: &[ExPolygon]) -> bool {
    // Quick bounding box check first
    let bbox1 = get_extents(expolys1);
    let bbox2 = get_extents(expolys2);

    if !bbox1.intersects(&bbox2) {
        return false;
    }

    // Full overlap check would require Clipper intersection
    // For now, do point-in-polygon tests as approximation
    for expoly1 in expolys1 {
        for expoly2 in expolys2 {
            // Check if any point of expoly1 is in expoly2
            if !expoly1.contour.points.is_empty()
                && expoly2.contains_point(&expoly1.contour.points[0])
            {
                return true;
            }
            // Check if any point of expoly2 is in expoly1
            if !expoly2.contour.points.is_empty()
                && expoly1.contains_point(&expoly2.contour.points[0])
            {
                return true;
            }
        }
    }

    false
}

/// Check if an ExPolygon overlaps with a collection
/// ExPolygon.hpp:468 (declared, implemented in ExPolygon.cpp)
pub fn overlaps_expoly(expolys: &[ExPolygon], expoly: &ExPolygon) -> bool {
    // Quick bounding box check
    let bbox_expoly = expoly.bounding_box();

    for other in expolys {
        if !other.bounding_box().intersects(&bbox_expoly) {
            continue;
        }

        // Check point containment
        if !other.contour.points.is_empty() && expoly.contains_point(&other.contour.points[0]) {
            return true;
        }
        if !expoly.contour.points.is_empty() && other.contains_point(&expoly.contour.points[0]) {
            return true;
        }
    }

    false
}

/// Faithful port of `bool remove_same_neighbor(ExPolygons &expolygons)`
/// (BambuStudio `ExPolygon.cpp:582-595`). Collapses consecutive-duplicate points
/// (std::unique semantics, NOT tolerance-based) on every contour and hole via the
/// Polygon/Polygons `remove_same_neighbor`, then erases any ExPolygon whose
/// contour collapsed to <= 2 points (only when a contour actually changed).
/// Returns true when anything was erased.
pub fn remove_same_neighbor(expolygons: &mut Vec<ExPolygon>) -> bool {
    use super::remove_same_neighbor_polygon;
    use super::remove_same_neighbor_polygons;

    // ExPolygon.cpp:584
    if expolygons.is_empty() {
        return false;
    }
    // ExPolygon.cpp:585-586
    let mut remove_from_holes = false;
    let mut remove_from_contour = false;
    // ExPolygon.cpp:587-590
    for expoly in expolygons.iter_mut() {
        // ExPolygon.cpp:588 — remove_from_contour |= remove_same_neighbor(expoly.contour);
        remove_from_contour |= remove_same_neighbor_polygon(&mut expoly.contour);
        // ExPolygon.cpp:589 — remove_from_holes |= remove_same_neighbor(expoly.holes);
        remove_from_holes |= remove_same_neighbor_polygons(&mut expoly.holes);
    }
    // ExPolygon.cpp:592-593 — Removing of expolygons without contour
    if remove_from_contour {
        expolygons.retain(|p| p.contour.points.len() > 2);
    }
    // ExPolygon.cpp:594
    remove_from_holes || remove_from_contour
}

/// Faithful port of `void keep_largest_contour_only(ExPolygons &polygons)`
/// (BambuStudio `ExPolygon.cpp:623-640`): when there is more than one ExPolygon,
/// keep ONLY the single ExPolygon whose CONTOUR has the largest (signed) area
/// across the whole collection; otherwise leave the collection untouched.
pub fn keep_largest_contour_only(polygons: &mut Vec<ExPolygon>) {
    // ExPolygon.cpp:625
    if polygons.len() > 1 {
        // ExPolygon.cpp:626-627
        let mut max_area = 0.0;
        let mut max_area_polygon: Option<usize> = None;
        // ExPolygon.cpp:628-634
        for (idx, p) in polygons.iter().enumerate() {
            // ExPolygon.cpp:629 — double a = p.contour.area();  (Polygon::area() is signed)
            let a = p.contour.area();
            // ExPolygon.cpp:630-633
            if a > max_area {
                max_area = a;
                max_area_polygon = Some(idx);
            }
        }
        // ExPolygon.cpp:635 — assert(max_area_polygon != nullptr);
        debug_assert!(max_area_polygon.is_some());
        // ExPolygon.cpp:636-638
        if let Some(idx) = max_area_polygon {
            let p = polygons[idx].clone();
            polygons.clear();
            polygons.push(p);
        }
    }
}

// ----------------------------------------------------------------------------------------------------
// R929: `simplify_p` / `simplify` of ExPolygon.cpp at the pin in the pin's shape and unit, beside the
// older Rust-shaped ones above (additive: nothing on the slicing path calls them). The other members of
// ExPolygon.cpp under their C++ names are in `crate::ex_polygon`. Compared with the pin by
// tests/ex_polygon_native_parity.rs.
// ----------------------------------------------------------------------------------------------------
impl ExPolygon {
    /// `Polygons ExPolygon::simplify_p(double tolerance) const` with the tolerance in the pin's unit
    /// (scaled coordinates, handed to `MultiPoint::_douglas_peucker` as it is); `simplify_p` above takes
    /// millimetres and another clipper.
    ///
    /// ExPolygon.cpp:231-251
    pub fn simplify_p_scaled_tolerance(&self, tolerance: f64) -> Vec<Polygon> {
        let mut pp: Vec<Polygon> = Vec::with_capacity(self.holes.len() + 1);
        // contour: push_back(front), _douglas_peucker, pop_back (Polygon::douglas_peucker is that).
        {
            let mut p = self.contour.clone();
            p.douglas_peucker(tolerance);
            pp.push(p);
        }
        // holes
        for hole in &self.holes {
            let mut p = hole.clone();
            p.douglas_peucker(tolerance);
            pp.push(p);
        }
        crate::clipper_utils::cpp::simplify_polygons(&pp, false)
    }

    /// `ExPolygons ExPolygon::simplify(double tolerance) const` (the name `simplify` is the in-place Rust
    /// function above; `simplified` is its copy and neither is this one).
    ///
    /// ExPolygon.cpp:253-256
    pub fn simplify_scaled_tolerance(&self, tolerance: f64) -> Vec<ExPolygon> {
        use crate::clipper_utils::cpp::UnionEx;
        // ClipperUtils.hpp: `union_ex(const Polygons &subject, PolyFillType fill_type = pftNonZero)`.
        UnionEx::union_ex(self.simplify_p_scaled_tolerance(tolerance).as_slice(), crate::clipper::PolyFillType::pftNonZero)
    }

    /// `void ExPolygon::simplify(double tolerance, ExPolygons* expolygons) const`.
    ///
    /// ExPolygon.cpp:258-261
    pub fn simplify_scaled_tolerance_into(&self, tolerance: f64, expolygons: &mut Vec<ExPolygon>) {
        expolygons.extend(self.simplify_scaled_tolerance(tolerance));
    }
}

/// `ExPolygons expolygons_simplify(const ExPolygons &expolys, double tolerance)` with the tolerance in
/// the pin's unit (see `ExPolygon::simplify_scaled_tolerance`).
///
/// ExPolygon.hpp:451-458
pub fn expolygons_simplify_scaled_tolerance(expolys: &[ExPolygon], tolerance: f64) -> Vec<ExPolygon> {
    let mut out: Vec<ExPolygon> = Vec::with_capacity(expolys.len());
    for exp in expolys {
        exp.simplify_scaled_tolerance_into(tolerance, &mut out);
    }
    out
}

/// `bool expolygons_contain(ExPolygons &expolys, const Point &pt, bool border_result)` over
/// `ExPolygon::contains(point, border_result)` of `crate::ex_polygon`.
///
/// ExPolygon.hpp:443-449
pub fn expolygons_contain_border_result(expolys: &[ExPolygon], pt: &Point, border_result: bool) -> bool {
    for expoly in expolys {
        if expoly.contains(pt, border_result) {
            return true;
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_square_with_hole() -> ExPolygon {
        // Outer square 0-100
        let contour = Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 100),
            Point::new(0, 100),
        ]);

        // Inner square (hole) 25-75, clockwise
        let hole = Polygon::from_points(vec![
            Point::new(25, 25),
            Point::new(25, 75),
            Point::new(75, 75),
            Point::new(75, 25),
        ]);

        ExPolygon::with_holes(contour, vec![hole])
    }

    #[test]
    fn test_expolygon_new() {
        let contour = Polygon::rectangle(Point::new(0, 0), Point::new(100, 100));
        let expoly = ExPolygon::new(contour);
        assert!(!expoly.is_empty());
        assert!(!expoly.has_holes());
        assert_eq!(expoly.hole_count(), 0);
    }

    #[test]
    fn test_expolygon_with_holes() {
        let expoly = make_square_with_hole();
        assert!(!expoly.is_empty());
        assert!(expoly.has_holes());
        assert_eq!(expoly.hole_count(), 1);
    }

    #[test]
    fn test_expolygon_area() {
        let expoly = make_square_with_hole();
        let area = expoly.area();
        // 100x100 = 10000, minus 50x50 = 2500, equals 7500
        assert!((area - 7500.0).abs() < 1.0);
    }

    #[test]
    fn test_expolygon_perimeter() {
        let expoly = make_square_with_hole();
        let perim = expoly.perimeter();
        // Outer: 400, Inner: 200, Total: 600
        assert!((perim - 600.0).abs() < 1.0);
    }

    #[test]
    fn test_expolygon_bounding_box() {
        let expoly = make_square_with_hole();
        let bb = expoly.bounding_box();
        assert_eq!(bb.min.x, 0);
        assert_eq!(bb.min.y, 0);
        assert_eq!(bb.max.x, 100);
        assert_eq!(bb.max.y, 100);
    }

    #[test]
    fn test_expolygon_contains_point() {
        let expoly = make_square_with_hole();

        // Point inside contour but outside hole
        assert!(expoly.contains_point(&Point::new(10, 10)));
        assert!(expoly.contains_point(&Point::new(90, 90)));

        // Point inside hole
        assert!(!expoly.contains_point(&Point::new(50, 50)));

        // Point outside contour
        assert!(!expoly.contains_point(&Point::new(-10, -10)));
        assert!(!expoly.contains_point(&Point::new(110, 110)));
    }

    #[test]
    fn test_expolygon_translate() {
        let mut expoly = make_square_with_hole();
        expoly.translate(Point::new(10, 20));

        assert_eq!(expoly.contour[0], Point::new(10, 20));
        assert_eq!(expoly.holes[0][0], Point::new(35, 45));
    }

    #[test]
    fn test_expolygon_scale() {
        let mut expoly = make_square_with_hole();
        expoly.scale(2.0);

        assert_eq!(expoly.contour[2], Point::new(200, 200));
        let area = expoly.area();
        // Original area 7500, scaled by 4 = 30000
        assert!((area - 30000.0).abs() < 1.0);
    }

    #[test]
    fn test_expolygon_make_canonical() {
        // Create with wrong orientations
        let contour = Polygon::from_points(vec![
            Point::new(0, 100),
            Point::new(100, 100),
            Point::new(100, 0),
            Point::new(0, 0),
        ]); // Clockwise

        let hole = Polygon::from_points(vec![
            Point::new(25, 25),
            Point::new(75, 25),
            Point::new(75, 75),
            Point::new(25, 75),
        ]); // Counter-clockwise

        let mut expoly = ExPolygon::with_holes(contour, vec![hole]);
        assert!(!expoly.is_canonical());

        expoly.make_canonical();
        assert!(expoly.is_canonical());
    }

    #[test]
    fn test_expolygon_is_valid() {
        let expoly = make_square_with_hole();
        assert!(expoly.is_valid());

        // Invalid: contour with only 2 points
        let invalid = ExPolygon::new(Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(100, 0),
        ]));
        assert!(!invalid.is_valid());
    }

    #[test]
    fn test_expolygon_all_polygons() {
        let expoly = make_square_with_hole();
        let all = expoly.all_polygons();
        assert_eq!(all.len(), 2);
    }

    #[test]
    fn test_expolygon_to_polylines() {
        let expoly = make_square_with_hole();
        let polylines = expoly.to_polylines();
        assert_eq!(polylines.len(), 2);
        // Each polyline should be closed (first point repeated at end)
        assert!(polylines[0].is_closed());
        assert!(polylines[1].is_closed());
    }

    #[test]
    fn test_expolygon_point_count() {
        let expoly = make_square_with_hole();
        assert_eq!(expoly.point_count(), 8); // 4 + 4
    }

    #[test]
    fn test_expolygon_rectangle() {
        let expoly = ExPolygon::rectangle(Point::new(0, 0), Point::new(100, 50));
        assert_eq!(expoly.contour.len(), 4);
        assert!(!expoly.has_holes());
        assert!((expoly.area() - 5000.0).abs() < 1.0);
    }

    #[test]
    fn test_expolygon_closest_point() {
        let expoly = make_square_with_hole();

        // Point outside - closest to contour
        let p1 = Point::new(50, -20);
        let closest1 = expoly.closest_point(&p1);
        assert_eq!(closest1.x, 50);
        assert_eq!(closest1.y, 0);

        // Point inside hole - closest to hole boundary
        let p2 = Point::new(50, 50);
        let closest2 = expoly.closest_point(&p2);
        // Should be on one of the hole edges, distance should be 25
        let dist = p2.distance(&closest2);
        assert!((dist - 25.0).abs() < 1.0);
    }

    #[test]
    fn test_expolygon_remove_small_holes() {
        let contour = Polygon::rectangle(Point::new(0, 0), Point::new(100, 100));
        let big_hole = Polygon::rectangle(Point::new(10, 10), Point::new(50, 50)); // area = 1600
        let small_hole = Polygon::rectangle(Point::new(60, 60), Point::new(65, 65)); // area = 25

        let mut expoly = ExPolygon::with_holes(contour, vec![big_hole, small_hole]);
        assert_eq!(expoly.hole_count(), 2);

        expoly.remove_small_holes(100.0);
        assert_eq!(expoly.hole_count(), 1);
    }

    #[test]
    fn test_expolygon_from_polygon() {
        let poly = Polygon::rectangle(Point::new(0, 0), Point::new(100, 100));
        let expoly: ExPolygon = poly.into();
        assert!(!expoly.has_holes());
        assert!((expoly.area() - 10000.0).abs() < 1.0);
    }

    // ========================================================================
    // Tests for Free Functions
    // ========================================================================

    #[test]
    fn test_count_points_expoly() {
        let expoly = make_square_with_hole();
        let count = count_points_expoly(&expoly);
        assert_eq!(count, 8); // 4 contour + 4 hole
    }

    #[test]
    fn test_count_points() {
        let expoly1 = make_square_with_hole();
        let expoly2 = ExPolygon::new(Polygon::rectangle(Point::new(0, 0), Point::new(50, 50)));
        let expolys = vec![expoly1, expoly2];
        let count = count_points(&expolys);
        assert_eq!(count, 12); // 8 from first + 4 from second
    }

    #[test]
    fn test_number_polygons() {
        let expoly1 = make_square_with_hole();
        let expoly2 = ExPolygon::new(Polygon::rectangle(Point::new(0, 0), Point::new(50, 50)));
        let expolys = vec![expoly1, expoly2];
        let count = number_polygons(&expolys);
        assert_eq!(count, 3); // 2 polygons from first (contour+hole) + 1 from second
    }

    #[test]
    fn test_to_lines_expoly() {
        let expoly = make_square_with_hole();
        let lines = to_lines_expoly(&expoly);
        assert_eq!(lines.len(), 8); // 4 edges from contour + 4 from hole
    }

    #[test]
    fn test_to_lines() {
        let expoly1 = ExPolygon::new(Polygon::rectangle(Point::new(0, 0), Point::new(50, 50)));
        let expoly2 = ExPolygon::new(Polygon::rectangle(
            Point::new(100, 100),
            Point::new(150, 150),
        ));
        let expolys = vec![expoly1, expoly2];
        let lines = to_lines(&expolys);
        assert_eq!(lines.len(), 8); // 4 edges per square
    }

    #[test]
    fn test_to_points() {
        let expoly = make_square_with_hole();
        let expolys = vec![expoly];
        let points = to_points(&expolys);
        assert_eq!(points.len(), 8); // 4 contour + 4 hole
    }

    #[test]
    fn test_to_polylines_expoly() {
        let expoly = make_square_with_hole();
        let polylines = to_polylines_expoly(&expoly);
        assert_eq!(polylines.len(), 2); // contour + hole
        assert_eq!(polylines[0].len(), 5); // 4 points + closing point
        assert_eq!(polylines[1].len(), 5); // 4 points + closing point
    }

    #[test]
    fn test_to_polylines() {
        let expoly1 = make_square_with_hole();
        let expoly2 = ExPolygon::new(Polygon::rectangle(Point::new(0, 0), Point::new(50, 50)));
        let expolys = vec![expoly1, expoly2];
        let polylines = to_polylines(&expolys);
        assert_eq!(polylines.len(), 3); // 2 from first + 1 from second
    }

    #[test]
    fn test_to_polygons_expoly() {
        let expoly = make_square_with_hole();
        let polygons = to_polygons_expoly(&expoly);
        assert_eq!(polygons.len(), 2); // contour + hole
    }

    #[test]
    fn test_to_polygons() {
        let expoly1 = make_square_with_hole();
        let expoly2 = ExPolygon::new(Polygon::rectangle(Point::new(0, 0), Point::new(50, 50)));
        let expolys = vec![expoly1, expoly2];
        let polygons = to_polygons(&expolys);
        assert_eq!(polygons.len(), 3); // 2 from first + 1 from second
    }

    #[test]
    fn test_to_expolygons_simple() {
        let poly1 = Polygon::rectangle(Point::new(0, 0), Point::new(100, 100));
        let poly2 = Polygon::rectangle(Point::new(200, 200), Point::new(300, 300));
        let polygons = vec![poly1, poly2];
        let expolys = to_expolygons_simple(&polygons);
        assert_eq!(expolys.len(), 2);
        assert!(!expolys[0].has_holes());
        assert!(!expolys[1].has_holes());
    }

    #[test]
    fn test_translate_expolygons() {
        let expoly = ExPolygon::new(Polygon::rectangle(Point::new(0, 0), Point::new(100, 100)));
        let mut expolys = vec![expoly];
        translate_expolygons(&mut expolys, Point::new(50, 50));
        assert_eq!(expolys[0].contour.points[0], Point::new(50, 50));
    }

    #[test]
    fn test_expolygons_rotate() {
        let expoly = ExPolygon::new(Polygon::rectangle(Point::new(0, 0), Point::new(100, 100)));
        let mut expolys = vec![expoly];
        expolygons_rotate(&mut expolys, std::f64::consts::PI / 2.0);
        // After 90-degree rotation, points should be rotated
        assert!(expolys[0].contour.points.len() > 0);
    }

    #[test]
    fn test_polygons_append_expoly() {
        let expoly = make_square_with_hole();
        let mut dst = Vec::new();
        polygons_append_expoly(&mut dst, &expoly);
        assert_eq!(dst.len(), 2); // contour + hole
    }

    #[test]
    fn test_polygons_append() {
        let expoly1 = make_square_with_hole();
        let expoly2 = ExPolygon::new(Polygon::rectangle(Point::new(0, 0), Point::new(50, 50)));
        let expolys = vec![expoly1, expoly2];
        let mut dst = Vec::new();
        polygons_append(&mut dst, &expolys);
        assert_eq!(dst.len(), 3); // 2 from first + 1 from second
    }

    #[test]
    fn test_expolygons_append() {
        let expoly1 = ExPolygon::new(Polygon::rectangle(Point::new(0, 0), Point::new(100, 100)));
        let expoly2 = ExPolygon::new(Polygon::rectangle(
            Point::new(200, 200),
            Point::new(300, 300),
        ));
        let mut dst = vec![expoly1];
        expolygons_append(&mut dst, &[expoly2]);
        assert_eq!(dst.len(), 2);
    }

    #[test]
    fn test_expolygons_contain() {
        let expoly = ExPolygon::new(Polygon::rectangle(Point::new(0, 0), Point::new(100, 100)));
        let expolys = vec![expoly];
        assert!(expolygons_contain(&expolys, Point::new(50, 50)));
        assert!(!expolygons_contain(&expolys, Point::new(200, 200)));
    }

    #[test]
    fn test_expolygons_simplify() {
        let expoly = ExPolygon::new(Polygon::rectangle(Point::new(0, 0), Point::new(100, 100)));
        let expolys = vec![expoly];
        let simplified = expolygons_simplify(&expolys, 1.0);
        assert_eq!(simplified.len(), 1);
    }

    #[test]
    fn test_area_expolygons() {
        let expoly1 = ExPolygon::new(Polygon::rectangle(Point::new(0, 0), Point::new(100, 100)));
        let expoly2 = ExPolygon::new(Polygon::rectangle(Point::new(0, 0), Point::new(50, 50)));
        let expolys = vec![expoly1, expoly2];
        let area = area_expolygons(&expolys);
        assert!((area - 12500.0).abs() < 1.0); // 10000 + 2500
    }

    #[test]
    fn test_get_extents_expoly() {
        let expoly = ExPolygon::new(Polygon::rectangle(Point::new(10, 20), Point::new(110, 120)));
        let bbox = get_extents_expoly(&expoly);
        assert_eq!(bbox.min, Point::new(10, 20));
        assert_eq!(bbox.max, Point::new(110, 120));
    }

    #[test]
    fn test_get_extents() {
        let expoly1 = ExPolygon::new(Polygon::rectangle(Point::new(0, 0), Point::new(100, 100)));
        let expoly2 = ExPolygon::new(Polygon::rectangle(Point::new(50, 50), Point::new(200, 200)));
        let expolys = vec![expoly1, expoly2];
        let bbox = get_extents(&expolys);
        assert_eq!(bbox.min, Point::new(0, 0));
        assert_eq!(bbox.max, Point::new(200, 200));
    }

    #[test]
    fn test_get_extents_vector() {
        let expoly1 = ExPolygon::new(Polygon::rectangle(Point::new(0, 0), Point::new(100, 100)));
        let expoly2 = ExPolygon::new(Polygon::rectangle(
            Point::new(200, 200),
            Point::new(300, 300),
        ));
        let expolys = vec![expoly1, expoly2];
        let bboxes = get_extents_vector(&expolys);
        assert_eq!(bboxes.len(), 2);
        assert_eq!(bboxes[0].min, Point::new(0, 0));
        assert_eq!(bboxes[1].min, Point::new(200, 200));
    }

    #[test]
    fn test_overlaps_expolygons() {
        let expoly1 = ExPolygon::new(Polygon::rectangle(Point::new(0, 0), Point::new(100, 100)));
        let expoly2 = ExPolygon::new(Polygon::rectangle(Point::new(50, 50), Point::new(150, 150)));
        let expoly3 = ExPolygon::new(Polygon::rectangle(
            Point::new(200, 200),
            Point::new(300, 300),
        ));

        assert!(overlaps_expolygons(&[expoly1.clone()], &[expoly2.clone()]));
        assert!(!overlaps_expolygons(&[expoly1.clone()], &[expoly3.clone()]));
    }

    #[test]
    fn test_overlaps_expoly() {
        let expoly1 = ExPolygon::new(Polygon::rectangle(Point::new(0, 0), Point::new(100, 100)));
        let expoly2 = ExPolygon::new(Polygon::rectangle(Point::new(50, 50), Point::new(150, 150)));
        let expoly3 = ExPolygon::new(Polygon::rectangle(
            Point::new(200, 200),
            Point::new(300, 300),
        ));

        assert!(overlaps_expoly(&[expoly1.clone()], &expoly2));
        assert!(!overlaps_expoly(&[expoly1.clone()], &expoly3));
    }

    #[test]
    fn test_remove_same_neighbor() {
        let mut poly = Polygon::new();
        poly.points = vec![
            Point::new(0, 0),
            Point::new(0, 0), // duplicate
            Point::new(100, 0),
            Point::new(100, 100),
        ];
        let mut expolys = vec![ExPolygon::new(poly)];
        let modified = remove_same_neighbor(&mut expolys);
        assert!(modified);
        assert_eq!(expolys[0].contour.len(), 3);
    }

    #[test]
    fn test_keep_largest_contour_only() {
        // Faithful C++ ExPolygon.cpp:623-640 semantics: with more than one
        // ExPolygon, keep ONLY the single ExPolygon whose CONTOUR has the largest
        // (signed) area; with a single ExPolygon, leave the collection untouched.
        let small = ExPolygon::new(Polygon::rectangle(Point::new(0, 0), Point::new(50, 50)));
        let large = ExPolygon::new(Polygon::rectangle(Point::new(0, 0), Point::new(100, 100)));
        let mut expolys = vec![small, large];

        keep_largest_contour_only(&mut expolys);

        // Only the larger-contour ExPolygon survives.
        assert_eq!(expolys.len(), 1);
        assert!((expolys[0].contour.area() - 10000.0).abs() < 1.0);

        // A single ExPolygon is left untouched (no >1 collection).
        let solo = ExPolygon::with_holes(
            Polygon::rectangle(Point::new(0, 0), Point::new(50, 50)),
            vec![Polygon::rectangle(Point::new(10, 10), Point::new(20, 20))],
        );
        let mut single = vec![solo];
        keep_largest_contour_only(&mut single);
        assert_eq!(single.len(), 1);
        assert_eq!(single[0].holes.len(), 1);
    }
}
