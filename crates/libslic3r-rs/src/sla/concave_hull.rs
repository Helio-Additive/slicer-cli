//! Faithful port of libslic3r/SLA/ConcaveHull.{hpp,cpp}
//!
//! C++ Reference:
//! - SLA/ConcaveHull.hpp
//! - SLA/ConcaveHull.cpp

use crate::clipper_utils::{
    offset_expolygons_round, offset_polygon, offset_polygons_round, union_polygons_ex,
    OffsetJoinType,
};
use crate::geometry::{
    to_expolygons_simple, to_polygons, ExPolygon, ExPolygons, Line, Point, Points, Polygon,
    Polygons, Vec2d, Vec3d,
};
use crate::sla::spat_index::{PointIndex, PointIndexEl};
use crate::{unscale, Coord};

/// ConcaveHull.hpp:9-15 — `inline Polygons get_contours(const ExPolygons &poly)`
#[inline]
pub fn get_contours(poly: &ExPolygons) -> Polygons {
    // ConcaveHull.hpp:11
    let mut ret: Polygons = Vec::with_capacity(poly.len());
    // ConcaveHull.hpp:12
    for p in poly {
        ret.push(p.contour.clone());
    }
    // ConcaveHull.hpp:14
    ret
}

/// ConcaveHull.hpp:17 — `using ThrowOnCancel = std::function<void()>;`
pub type ThrowOnCancel<'a> = &'a dyn Fn();

/// libslic3r.h `scaled<coord_t>()`: `Tout(v / Tin(SCALING_FACTOR))`.
/// NOTE: this is a truncating cast (toward zero), unlike `crate::scaled()`
/// which rounds; ConcaveHull.cpp:97/117 rely on the C++ semantics.
#[inline]
fn scaled_trunc(v: f64) -> Coord {
    (v / crate::libslic3r::SCALING_FACTOR) as Coord
}

// ConcaveHull.cpp:12 — `inline Vec3d to_vec3(const Vec2crd &v2) { return {double(v2(X)), double(v2(Y)), 0.}; }`
#[inline]
fn to_vec3(v2: &Point) -> Vec3d {
    Vec3d::new(v2.x as f64, v2.y as f64, 0.)
}

// ConcaveHull.cpp:13 — `inline Vec3d to_vec3(const Vec2d &v2) { return {v2(X), v2(Y), 0.}; }`
// (overload on Vec2d; unused inside ConcaveHull.cpp, kept for parity)
#[inline]
#[allow(dead_code)]
fn to_vec3_d(v2: &Vec2d) -> Vec3d {
    Vec3d::new(v2.x, v2.y, 0.)
}

// ConcaveHull.cpp:14 — `inline Vec2crd to_vec2(const Vec3d &v3) { return {coord_t(v3(X)), coord_t(v3(Y))}; }`
#[inline]
fn to_vec2(v3: &Vec3d) -> Point {
    // C++ `coord_t(double)` casts truncate toward zero, as does Rust `as`.
    Point::new(v3.x as Coord, v3.y as Coord)
}

/// A fake concave hull that is constructed by connecting separate shapes
/// with explicit bridges. Bridges are generated from each shape's centroid
/// to the center of the "scene" which is the centroid calculated from the shape
/// centroids (a star is created...)
///
/// ConcaveHull.hpp:23 — `class ConcaveHull`
#[derive(Debug, Clone, Default)]
pub struct ConcaveHull {
    /// ConcaveHull.hpp:24 — `Polygons m_polys;`
    m_polys: Polygons,
}

impl ConcaveHull {
    /// ConcaveHull.cpp:16-41 — `Point ConcaveHull::centroid(const Points &pp)`
    fn centroid(pp: &Points) -> Point {
        // ConcaveHull.cpp:18 — `Point c;` (default constructed = {0, 0})
        let mut c = Point::new(0, 0);
        // ConcaveHull.cpp:19
        match pp.len() {
            // ConcaveHull.cpp:20
            0 => {}
            // ConcaveHull.cpp:21
            1 => c = pp[0],
            // ConcaveHull.cpp:22 — integer division, truncates like C++
            2 => c = (pp[0] + pp[1]) / 2,
            // ConcaveHull.cpp:23-37
            _ => {
                // ConcaveHull.cpp:24-25
                let max_lim = Coord::MAX;
                let min_lim = Coord::MIN;
                // ConcaveHull.cpp:26
                let mut min = Point::new(max_lim, max_lim);
                let mut max = Point::new(min_lim, min_lim);

                // ConcaveHull.cpp:28-33
                for p in pp {
                    if p.x < min.x {
                        min.x = p.x;
                    }
                    if p.y < min.y {
                        min.y = p.y;
                    }
                    if p.x > max.x {
                        max.x = p.x;
                    }
                    if p.y > max.y {
                        max.y = p.y;
                    }
                }
                // ConcaveHull.cpp:34-35
                c.x = min.x + (max.x - min.x) / 2;
                c.y = min.y + (max.y - min.y) / 2;
            }
        }

        // ConcaveHull.cpp:40
        c
    }

    /// ConcaveHull.hpp:28 — `static inline Point centroid(const Polygon &poly) { return poly.centroid(); }`
    #[inline]
    fn centroid_poly(poly: &Polygon) -> Point {
        poly.centroid()
    }

    /// ConcaveHull.cpp:43-52 — `Points ConcaveHull::calculate_centroids() const`
    fn calculate_centroids(&self) -> Points {
        // We get the centroids of all the islands in the 2D slice
        // ConcaveHull.cpp:46-49 — std::transform over m_polys with the
        // Polygon overload of centroid() (ConcaveHull.hpp:28).
        let centroids: Points = self.m_polys.iter().map(Self::centroid_poly).collect();

        // ConcaveHull.cpp:51
        centroids
    }

    /// ConcaveHull.cpp:54 — `void ConcaveHull::merge_polygons() { m_polys = get_contours(union_ex(m_polys)); }`
    fn merge_polygons(&mut self) {
        self.m_polys = get_contours(&union_polygons_ex(&self.m_polys));
    }

    /// ConcaveHull.cpp:56-104 — `void ConcaveHull::add_connector_rectangles(...)`
    fn add_connector_rectangles(&mut self, centroids: &Points, max_dist: Coord, thr: ThrowOnCancel) {
        // Centroid of the centroids of islands. This is where the additional
        // connector sticks are routed.
        // ConcaveHull.cpp:62
        let cc = Self::centroid(centroids);

        // ConcaveHull.cpp:64-66
        let mut ctrindex = PointIndex::new();
        let mut idx: u32 = 0;
        for ct in centroids {
            ctrindex.insert_point(to_vec3(ct), idx);
            idx += 1;
        }

        // ConcaveHull.cpp:68 — `m_polys.reserve(m_polys.size() + centroids.size());`
        self.m_polys.reserve(centroids.len());

        // ConcaveHull.cpp:70-71
        let mut idx: usize = 0;
        for c in centroids {
            // ConcaveHull.cpp:72
            thr();

            // ConcaveHull.cpp:74-76 — coord_t arithmetic, then converted to double
            let dx = (c.x - cc.x) as f64;
            let dy = (c.y - cc.y) as f64;
            let l = (dx * dx + dy * dy).sqrt();
            let nx = dx / l;
            let ny = dy / l;

            // ConcaveHull.cpp:78
            let ct = &centroids[idx];

            // ConcaveHull.cpp:80
            let result: Vec<PointIndexEl> = ctrindex.nearest(&to_vec3(ct), 2);

            // ConcaveHull.cpp:82-87
            let mut dist = max_dist as f64;
            for el in &result {
                if el.1 as usize != idx {
                    dist = Line::new(to_vec2(&el.0), *ct).length();
                    break;
                }
            }

            // ConcaveHull.cpp:89
            idx += 1;

            // ConcaveHull.cpp:91 — NOTE: returns from the whole function
            // (not `continue`), exactly as the C++ does.
            if dist >= max_dist as f64 {
                return;
            }

            // ConcaveHull.cpp:93-95
            let mut r = Polygon::new();
            r.points.reserve(3);
            r.points.push(cc);

            // ConcaveHull.cpp:97-99
            let n = Point::new(scaled_trunc(nx), scaled_trunc(ny));
            r.points.push(*c + Point::new(n.y, -n.x));
            r.points.push(*c + Point::new(-n.y, n.x));
            // ConcaveHull.cpp:100 — `offset(r, scaled<float>(1.));`
            // ClipperUtils `offset(const Polygon&, float)` returns Polygons *by
            // value*; the C++ discards the return value, so `r` is left
            // unchanged. Replicate the call (default jtMiter join) and discard
            // the result.  scaled<float>(1.) == 1 mm in the crate's mm-based
            // clipper layer.
            let _ = offset_polygon(&r, 1.0, OffsetJoinType::Miter);

            // ConcaveHull.cpp:102
            self.m_polys.push(r);
        }
    }

    /// ConcaveHull.hpp:39-40 —
    /// `ConcaveHull(const ExPolygons &polys, double merge_dist, ThrowOnCancel thr)
    ///      : ConcaveHull{to_polygons(polys), merge_dist, thr} {}`
    pub fn from_ex_polygons(polys: &ExPolygons, merge_dist: f64, thr: ThrowOnCancel) -> Self {
        Self::new(&to_polygons(polys), merge_dist, thr)
    }

    /// ConcaveHull.cpp:106-120 — `ConcaveHull::ConcaveHull(const Polygons &polys, double mergedist, ThrowOnCancel thr)`
    pub fn new(polys: &Polygons, mergedist: f64, thr: ThrowOnCancel) -> Self {
        let mut this = ConcaveHull { m_polys: Polygons::new() };

        // ConcaveHull.cpp:108
        if polys.is_empty() {
            return this;
        }

        // ConcaveHull.cpp:110-111
        this.m_polys = polys.clone();
        this.merge_polygons();

        // ConcaveHull.cpp:113
        if this.m_polys.len() == 1 {
            return this;
        }

        // ConcaveHull.cpp:115
        let centroids = this.calculate_centroids();

        // ConcaveHull.cpp:117 — `scaled(mergedist)` (truncating cast)
        this.add_connector_rectangles(&centroids, scaled_trunc(mergedist), thr);

        // ConcaveHull.cpp:119
        this.merge_polygons();

        this
    }

    /// ConcaveHull.hpp:44 — `const Polygons & polygons() const { return m_polys; }`
    #[inline]
    pub fn polygons(&self) -> &Polygons {
        &self.m_polys
    }

    /// ConcaveHull.cpp:122-127 — `ExPolygons ConcaveHull::to_expolygons() const`
    pub fn to_expolygons(&self) -> ExPolygons {
        // ConcaveHull.cpp:124
        let mut ret: ExPolygons = Vec::with_capacity(self.m_polys.len());
        // ConcaveHull.cpp:125
        for p in &self.m_polys {
            ret.push(ExPolygon::new(p.clone()));
        }
        // ConcaveHull.cpp:126
        ret
    }
}

/// ConcaveHull.cpp:129-132 — `ExPolygons offset_waffle_style_ex(const ConcaveHull &hull, coord_t delta)`
pub fn offset_waffle_style_ex(hull: &ConcaveHull, delta: Coord) -> ExPolygons {
    // ConcaveHull.cpp:131 — `to_expolygons(offset_waffle_style(hull, delta))`
    // (ExPolygon.hpp:352-359 free function: each Polygon becomes an ExPolygon
    // contour — ported as `to_expolygons_simple`).
    to_expolygons_simple(&offset_waffle_style(hull, delta))
}

/// ConcaveHull.cpp:134-143 — `Polygons offset_waffle_style(const ConcaveHull &hull, coord_t delta)`
pub fn offset_waffle_style(hull: &ConcaveHull, delta: Coord) -> Polygons {
    // ConcaveHull.cpp:136 — `auto arc_tolerance = scaled<double>(0.01);`
    let arc_tolerance: f64 = 0.01 / crate::libslic3r::SCALING_FACTOR;
    // ConcaveHull.cpp:137 —
    // `Polygons res = closing(hull.polygons(), 2 * delta, delta, ClipperLib::jtRound, arc_tolerance);`
    // ClipperUtils.hpp `closing(polys, delta, delta2, jt, tol)` is
    // `offset2(polys, +delta, -delta2, jt, tol)`: grow by 2*delta, then shrink
    // by delta.  The crate's clipper wrappers operate in unscaled mm, hence
    // the unscale()/SCALING_FACTOR conversions of the scaled arguments.
    let arc_tolerance_mm = arc_tolerance * crate::libslic3r::SCALING_FACTOR;
    let grown = offset_polygons_round(hull.polygons(), unscale(2 * delta), arc_tolerance_mm);
    let closed = offset_expolygons_round(&grown, -unscale(delta), arc_tolerance_mm);
    // C++ `closing` flattens the result paths into Polygons (outer contours
    // CCW, holes CW), matching to_polygons() on canonical ExPolygons.
    let mut res: Polygons = to_polygons(&closed);

    // ConcaveHull.cpp:139-140 —
    // `auto it = std::remove_if(res.begin(), res.end(), [](Polygon &p) { return p.is_clockwise(); });`
    // `res.erase(it, res.end());`
    res.retain(|p| !p.is_clockwise());

    // ConcaveHull.cpp:142
    res
}
