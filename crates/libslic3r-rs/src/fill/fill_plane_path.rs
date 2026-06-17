//! Space-filling curve infill patterns (Archimedean Chords, Hilbert Curve, Octagram Spiral).
//!
//! Faithful 1:1 line-by-line port of `Slic3r::FillPlanePath` and friends.
//!
//! C++ Reference:
//! - `src/libslic3r/Fill/FillPlanePath.cpp`
//! - `src/libslic3r/Fill/FillPlanePath.hpp`
//!
//! The original Perl code used path generators from Math::PlanePath library:
//! - http://user42.tuxfamily.org/math-planepath/
//! - http://user42.tuxfamily.org/math-planepath/gallery.html

// FillPlanePath.cpp:1-5
//   #include "../ClipperUtils.hpp"
//   #include "../ShortestPath.hpp"
//   #include "../Surface.hpp"
//   #include "FillPlanePath.hpp"
use super::{connect_infill_expolygon, FillParams};
use crate::clipper_utils::intersection_pl;
use crate::geometry::{get_extents_expoly, BoundingBox, ExPolygon, Point, PointF, Polyline};
use crate::libslic3r::SCALED_EPSILON;
use crate::shortest_path::chain_polylines;
use crate::{Coord, CoordF, SCALING_FACTOR};
use std::f64::consts::PI;

// FillPlanePath.cpp:7 — namespace Slic3r

// libslic3r.h:84 — `#define SCALED_EPSILON scale_(EPSILON)` = EPSILON / SCALING_FACTOR
//   = 1e-4 / 1e-5 = 10. The canonical crate constant `crate::libslic3r::SCALED_EPSILON`
//   (= 10.0) is the faithful value; do NOT use a local 1000.

// =============================================================================
// InfillPolylineOutput / InfillPolylineClipper
//
// FillPlanePath.hpp:16-34 — `class InfillPolylineOutput`
// FillPlanePath.cpp:9-38  — `class InfillPolylineClipper : public InfillPolylineOutput`
//
// The C++ uses runtime polymorphism (virtual `add_point`/`clips`) with exactly two
// concrete classes: the base `InfillPolylineOutput` (no clipping) and
// `InfillPolylineClipper` (clips against a bounding box). We model both with one
// struct that branches on `m_clips`, faithfully reproducing both `add_point` bodies.
// =============================================================================

// FillPlanePath.cpp:18-23 — enum class Side
#[derive(Clone, Copy)]
enum Side {
    Left = 1,
    Right = 2,
    Top = 4,
    Bottom = 8,
}

/// FillPlanePath.hpp:16 — `class InfillPolylineOutput`
/// FillPlanePath.cpp:9 — `class InfillPolylineClipper : public InfillPolylineOutput`
pub struct InfillPolylineOutput {
    // FillPlanePath.hpp:29 — Output polyline. `Points m_out;`
    m_out: Vec<Point>,

    // FillPlanePath.hpp:33 — Scaling coefficient of the generated points before tested
    // against m_bbox and clipped by bbox. `double m_scale_out;`
    m_scale_out: CoordF,

    // FillPlanePath.hpp:23 — `virtual bool clips() const { return false; }`
    // FillPlanePath.cpp:15 — `bool clips() const override { return true; }`
    m_clips: bool,

    // FillPlanePath.cpp:33 — Bounding box to clip the polyline with. `BoundingBox m_bbox;`
    m_bbox: BoundingBox,

    // FillPlanePath.cpp:36-37 — Classification of the two last points processed.
    // `int m_sides_prev; int m_sides_this;`
    m_sides_prev: i32,
    m_sides_this: i32,
}

impl InfillPolylineOutput {
    /// FillPlanePath.hpp:18 — `InfillPolylineOutput(const double scale_out) : m_scale_out(scale_out) {}`
    pub fn new(scale_out: CoordF) -> Self {
        Self {
            m_out: Vec::new(),
            m_scale_out: scale_out,
            m_clips: false,
            m_bbox: BoundingBox::new(),
            m_sides_prev: 0,
            m_sides_this: 0,
        }
    }

    /// FillPlanePath.cpp:11 — `InfillPolylineClipper(const BoundingBox bbox, const double scale_out)`
    pub fn new_clipper(bbox: BoundingBox, scale_out: CoordF) -> Self {
        Self {
            m_out: Vec::new(),
            m_scale_out: scale_out,
            m_clips: true,
            m_bbox: bbox,
            m_sides_prev: 0,
            m_sides_this: 0,
        }
    }

    /// FillPlanePath.hpp:20 — `void reserve(size_t n) { m_out.reserve(n); }`
    pub fn reserve(&mut self, n: usize) {
        self.m_out.reserve(n);
    }

    /// FillPlanePath.hpp:22 — `Points&& result() { return std::move(m_out); }`
    /// FillPlanePath.cpp:14 — clipper `Points&& result() { return std::move(m_out); }`
    pub fn result(self) -> Vec<Point> {
        self.m_out
    }

    /// FillPlanePath.hpp:23 / FillPlanePath.cpp:15 — `clips()`.
    pub fn clips(&self) -> bool {
        self.m_clips
    }

    /// FillPlanePath.hpp:26 — `const Point scaled(const Vec2d& fpt) const`
    /// `{ return { coord_t(floor(fpt.x() * m_scale_out + 0.5)), coord_t(floor(fpt.y() * m_scale_out + 0.5)) }; }`
    fn scaled(&self, fpt: PointF) -> Point {
        Point::new(
            (fpt.x() * self.m_scale_out + 0.5).floor() as Coord,
            (fpt.y() * self.m_scale_out + 0.5).floor() as Coord,
        )
    }

    /// FillPlanePath.cpp:25-30 — `int sides(const Point &p) const`
    fn sides(&self, p: &Point) -> i32 {
        // FillPlanePath.cpp:26-29
        ((p.x() < self.m_bbox.min.x()) as i32) * (Side::Left as i32)
            + ((p.x() > self.m_bbox.max.x()) as i32) * (Side::Right as i32)
            + ((p.y() < self.m_bbox.min.y()) as i32) * (Side::Bottom as i32)
            + ((p.y() > self.m_bbox.max.y()) as i32) * (Side::Top as i32)
    }

    /// FillPlanePath.hpp:21 — base `void add_point(const Vec2d& pt) { m_out.emplace_back(this->scaled(pt)); }`
    /// FillPlanePath.cpp:40-67 — clipper `void InfillPolylineClipper::add_point(const Vec2d &fpt)`
    fn add_point(&mut self, fpt: PointF) {
        if !self.m_clips {
            // FillPlanePath.hpp:21 — base behavior.
            let p = self.scaled(fpt);
            self.m_out.push(p);
            return;
        }

        // FillPlanePath.cpp:42 — const Point pt{ this->scaled(fpt) };
        let pt = self.scaled(fpt);

        // FillPlanePath.cpp:44
        if self.m_out.len() < 2 {
            // FillPlanePath.cpp:45 — Collect the two first points and their status.
            // FillPlanePath.cpp:46 — (m_out.empty() ? m_sides_prev : m_sides_this) = sides(pt);
            let s = self.sides(&pt);
            if self.m_out.is_empty() {
                self.m_sides_prev = s;
            } else {
                self.m_sides_this = s;
            }
            // FillPlanePath.cpp:47 — m_out.emplace_back(pt);
            self.m_out.push(pt);
        } else {
            // FillPlanePath.cpp:49 — Classify the last inserted point, possibly remove it.
            // FillPlanePath.cpp:50 — int sides_next = sides(pt);
            let sides_next = self.sides(&pt);
            // FillPlanePath.cpp:51-55
            if
            // This point is inside. Take it.
            self.m_sides_this == 0 ||
                // Either this point is outside and previous or next is inside, or
                // the edge possibly cuts corner of the bounding box.
                (self.m_sides_prev & self.m_sides_this & sides_next) == 0
            {
                // FillPlanePath.cpp:56 — Keep the last point.
                // FillPlanePath.cpp:57 — m_sides_prev = m_sides_this;
                self.m_sides_prev = self.m_sides_this;
            } else {
                // FillPlanePath.cpp:59-60 — All the three points (this, prev, next) are
                // outside at the same side. Ignore the last point.
                // FillPlanePath.cpp:61 — m_out.pop_back();
                self.m_out.pop();
            }
            // FillPlanePath.cpp:63-64 — And save the current point. m_out.emplace_back(pt);
            self.m_out.push(pt);
            // FillPlanePath.cpp:65 — m_sides_this = sides_next;
            self.m_sides_this = sides_next;
        }
    }
}

// =============================================================================
// FillPlanePath
//
// FillPlanePath.hpp:36-56 — `class FillPlanePath : public Fill`
// =============================================================================

/// Pattern selector for `FillPlanePath`. The C++ models the three patterns as
/// subclasses overriding `generate()` and `centered()`; we carry the pattern as a
/// discriminant so a single struct can dispatch faithfully.
/// FillPlanePath.hpp:58-89
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PlanPathPattern {
    /// FillPlanePath.hpp:58 — `class FillArchimedeanChords`.
    ArchimedeanChords,
    /// FillPlanePath.hpp:69 — `class FillHilbertCurve`.
    #[default]
    HilbertCurve,
    /// FillPlanePath.hpp:80 — `class FillOctagramSpiral`.
    OctagramSpiral,
}

/// FillPlanePath.hpp:36 — `class FillPlanePath : public Fill`.
///
/// Holds the inherited `Fill` members that `_fill_surface_single` reads:
/// `this->spacing` (FillBase.hpp:115) and `this->bounding_box` (the object-level
/// bounding box used to align sparse infill across layers, FillBase.hpp).
#[derive(Debug, Clone)]
pub struct FillPlanePath {
    /// Base `Fill::spacing`, in unscaled coordinates (FillBase.hpp:115).
    pub spacing: CoordF,

    /// Base `Fill::bounding_box` — object bounding box for cross-layer alignment.
    pub bounding_box: BoundingBox,

    /// Which space-filling curve to generate.
    pub pattern: PlanPathPattern,
}

impl FillPlanePath {
    pub fn new(pattern: PlanPathPattern) -> Self {
        Self {
            spacing: 0.0,
            bounding_box: BoundingBox::new(),
            pattern,
        }
    }

    /// FillPlanePath.hpp:40 — `bool is_self_crossing() override { return false; }`
    pub fn is_self_crossing(&self) -> bool {
        false
    }

    /// FillPlanePath.hpp:50 — `float _layer_angle(size_t idx) const override { return 0.f; }`
    pub fn _layer_angle(&self, _idx: usize) -> f32 {
        0.0
    }

    /// FillPlanePath.hpp:51 — `virtual bool centered() const = 0;`
    /// FillArchimedeanChords / FillOctagramSpiral: true; FillHilbertCurve: false.
    /// FillPlanePath.hpp:65, 76, 87.
    fn centered(&self) -> bool {
        match self.pattern {
            PlanPathPattern::ArchimedeanChords => true, // FillPlanePath.hpp:65
            PlanPathPattern::HilbertCurve => false,     // FillPlanePath.hpp:76
            PlanPathPattern::OctagramSpiral => true,    // FillPlanePath.hpp:87
        }
    }

    /// FillPlanePath.hpp:55 / FillPlanePath.cpp:159-280 — `virtual void generate(...)`.
    /// Dispatches to the per-pattern generator, mirroring the three `FillXxx::generate`
    /// overrides which select the clipping vs non-clipping template instantiation.
    fn generate(
        &self,
        min_x: Coord,
        min_y: Coord,
        max_x: Coord,
        max_y: Coord,
        resolution: CoordF,
        output: &mut InfillPolylineOutput,
    ) {
        match self.pattern {
            // FillPlanePath.cpp:159-165 — FillArchimedeanChords::generate
            PlanPathPattern::ArchimedeanChords => {
                generate_archimedean_chords(min_x, min_y, max_x, max_y, resolution, output)
            }
            // FillPlanePath.cpp:234-240 — FillHilbertCurve::generate
            PlanPathPattern::HilbertCurve => {
                generate_hilbert_curve(min_x, min_y, max_x, max_y, output)
            }
            // FillPlanePath.cpp:274-280 — FillOctagramSpiral::generate
            PlanPathPattern::OctagramSpiral => {
                generate_octagram_spiral(min_x, min_y, max_x, max_y, output)
            }
        }
    }

    /// FillPlanePath.cpp:69-134 — `void FillPlanePath::_fill_surface_single(...)`.
    pub fn _fill_surface_single(
        &mut self,
        params: &FillParams,
        _thickness_layers: u32,
        direction: &(f32, Point),
        mut expolygon: ExPolygon,
        polylines_out: &mut Vec<Polyline>,
    ) {
        // FillPlanePath.cpp:76
        expolygon.rotate(-(direction.0 as CoordF));

        // FillPlanePath.cpp:78-79
        //FIXME Vojtech: We are not sure whether the user expects the fill patterns on visible surfaces to be aligned across all the islands of a single layer.
        // One may align for this->centered() to align the patterns for Archimedean Chords and Octagram Spiral patterns.
        // FillPlanePath.cpp:80
        let align = params.density < 0.995;

        // FillPlanePath.cpp:82 — get_extents(expolygon).inflated(SCALED_EPSILON)
        let mut snug_bounding_box: BoundingBox =
            bbox_inflated(&get_extents_expoly(&expolygon), SCALED_EPSILON);

        // FillPlanePath.cpp:84-90
        // Rotated bounding box of the area to fill in with the pattern.
        let mut bounding_box: BoundingBox = if align {
            // Sparse infill needs to be aligned across layers. Align infill across layers using the object's bounding box.
            // FillPlanePath.cpp:87
            bbox_rotated(&self.bounding_box, -(direction.0 as CoordF))
        } else {
            // Solid infill does not need to be aligned across layers, generate the infill pattern
            // around the clipping expolygon only.
            // FillPlanePath.cpp:90
            snug_bounding_box
        };

        // FillPlanePath.cpp:92-94
        let shift: Point = if self.centered() {
            bounding_box.center()
        } else {
            bounding_box.min
        };
        // FillPlanePath.cpp:95
        expolygon.translate(Point::new(-shift.x(), -shift.y()));
        // FillPlanePath.cpp:96
        bounding_box.translate(Point::new(-shift.x(), -shift.y()));

        // FillPlanePath.cpp:98
        let mut polyline = Polyline::default();
        {
            // FillPlanePath.cpp:100 — auto distance_between_lines = scaled<double>(this->spacing) / params.density;
            //   scaled<double>(v) = v / SCALING_FACTOR (a double, NOT rounded to coord_t);
            //   in crate terms = v * SCALING_FACTOR (= v * 1e5). Keep full float precision.
            let distance_between_lines = (self.spacing * SCALING_FACTOR) / params.density as CoordF;
            // FillPlanePath.cpp:101
            let min_x =
                (bounding_box.min.x() as CoordF / distance_between_lines).ceil() as Coord;
            // FillPlanePath.cpp:102
            let min_y =
                (bounding_box.min.y() as CoordF / distance_between_lines).ceil() as Coord;
            // FillPlanePath.cpp:103
            let max_x =
                (bounding_box.max.x() as CoordF / distance_between_lines).ceil() as Coord;
            // FillPlanePath.cpp:104
            let max_y =
                (bounding_box.max.y() as CoordF / distance_between_lines).ceil() as Coord;
            // FillPlanePath.cpp:105 — auto resolution = scaled<double>(params.resolution) / distance_between_lines;
            //   scaled<double>(v) = v * SCALING_FACTOR (full float, not rounded).
            let resolution = (params.resolution * SCALING_FACTOR) / distance_between_lines;
            // FillPlanePath.cpp:106
            if align {
                // FillPlanePath.cpp:107
                // Filling in a bounding box over the whole object, clip generated polyline against the snug bounding box.
                // FillPlanePath.cpp:108
                snug_bounding_box.translate(Point::new(-shift.x(), -shift.y()));
                // FillPlanePath.cpp:109
                let mut output =
                    InfillPolylineOutput::new_clipper(snug_bounding_box, distance_between_lines);
                // FillPlanePath.cpp:110
                self.generate(min_x, min_y, max_x, max_y, resolution, &mut output);
                // FillPlanePath.cpp:111
                polyline.points = output.result();
            } else {
                // FillPlanePath.cpp:113
                // Filling in a snug bounding box, no need to clip.
                // FillPlanePath.cpp:114
                let mut output = InfillPolylineOutput::new(distance_between_lines);
                // FillPlanePath.cpp:115
                self.generate(min_x, min_y, max_x, max_y, resolution, &mut output);
                // FillPlanePath.cpp:116
                polyline.points = output.result();
            }
        }

        // FillPlanePath.cpp:120
        if polyline.size() >= 2 {
            // FillPlanePath.cpp:121 — Polylines polylines = intersection_pl(polyline, expolygon);
            // FIDELITY-NOTE(F1): geo-clipper approximation vs C++ ClipperLib (fixed scale 1000).
            let polylines =
                intersection_pl(std::slice::from_ref(&polyline), std::slice::from_ref(&expolygon));
            // FillPlanePath.cpp:122
            let mut chained: Vec<Polyline>;
            // FillPlanePath.cpp:123
            if params.dont_connect() || params.density > 0.5 || polylines.len() <= 1 {
                // FillPlanePath.cpp:124
                chained = chain_polylines(polylines, None);
            } else {
                // FillPlanePath.cpp:126
                chained = Vec::new();
                connect_infill_expolygon(polylines, &expolygon, self.spacing, params, &mut chained);
            }
            // FillPlanePath.cpp:127
            // paths must be repositioned and rotated back
            // FillPlanePath.cpp:128
            for pl in chained.iter_mut() {
                // FillPlanePath.cpp:129
                pl.translate(Point::new(shift.x(), shift.y()));
                // FillPlanePath.cpp:130
                pl.rotate(direction.0 as CoordF);
            }
            // FillPlanePath.cpp:132
            polylines_out.extend(chained);
        }
    }
}

// =============================================================================
// BoundingBox helpers (BoundingBox.cpp:36-44, BoundingBox.hpp:48/215)
// =============================================================================

/// BoundingBox.hpp:48,215 — `inflated(coordf_t delta)` = `offset(delta)` on all sides.
/// BoundingBox::offset expands min by -delta and max by +delta.
fn bbox_inflated(bb: &BoundingBox, delta: CoordF) -> BoundingBox {
    bb.expanded(delta.round() as Coord)
}

/// BoundingBox.cpp:36-44 — `BoundingBox BoundingBox::rotated(double angle) const`.
fn bbox_rotated(bb: &BoundingBox, angle: CoordF) -> BoundingBox {
    // FillPlanePath: the object bounding box rotates its four corners and re-merges.
    let mut out = BoundingBox::new();
    // BoundingBox.cpp:39
    out.merge_point(bb.min.rotate(angle));
    // BoundingBox.cpp:40
    out.merge_point(bb.max.rotate(angle));
    // BoundingBox.cpp:41 — Point(min(0), max(1))
    out.merge_point(Point::new(bb.min.x(), bb.max.y()).rotate(angle));
    // BoundingBox.cpp:42 — Point(max(0), min(1))
    out.merge_point(Point::new(bb.max.x(), bb.min.y()).rotate(angle));
    out
}

// =============================================================================
// Archimedean Chords (FillPlanePath.cpp:136-165)
// =============================================================================

/// FillPlanePath.cpp:136-157 — `generate_archimedean_chords`.
/// Follow an Archimedean spiral, in polar coordinates: r=a+b\theta
fn generate_archimedean_chords(
    _min_x: Coord,
    _min_y: Coord,
    max_x: Coord,
    max_y: Coord,
    resolution: CoordF,
    output: &mut InfillPolylineOutput,
) {
    // FillPlanePath.cpp:140-141 — Radius to achieve.
    let rmax: CoordF = ((max_x as CoordF) * (max_x as CoordF) + (max_y as CoordF) * (max_y as CoordF))
        .sqrt()
        * (2.0_f64).sqrt()
        + 1.5;
    // FillPlanePath.cpp:142 — Now unwind the spiral.
    // FillPlanePath.cpp:143
    let a: CoordF = 1.0;
    // FillPlanePath.cpp:144
    let b: CoordF = 1.0 / (2.0 * PI);
    // FillPlanePath.cpp:145
    let mut theta: CoordF = 0.0;
    // FillPlanePath.cpp:146
    let mut r: CoordF = 1.0;
    // FillPlanePath.cpp:148 —FIXME Vojtech: If used as a solid infill, there is a gap left at the center.
    // FillPlanePath.cpp:149
    output.add_point(PointF::new(0.0, 0.0));
    // FillPlanePath.cpp:150
    output.add_point(PointF::new(1.0, 0.0));
    // FillPlanePath.cpp:151
    while r < rmax {
        // FillPlanePath.cpp:152-153 — Discretization angle to achieve a discretization error lower than resolution.
        theta += 2.0 * (1.0 - resolution / r).acos();
        // FillPlanePath.cpp:154
        r = a + b * theta;
        // FillPlanePath.cpp:155
        output.add_point(PointF::new(r * theta.cos(), r * theta.sin()));
    }
}

// =============================================================================
// Hilbert Curve (FillPlanePath.cpp:167-240)
// =============================================================================

// FillPlanePath.cpp:167-183 — Adapted from
// http://cpansearch.perl.org/src/KRYDE/Math-PlanePath-122/lib/Math/PlanePath/HilbertCurve.pm
//
// state=0    3--2   plain
//               |
//            0--1
//
// state=4    1--2  transpose
//            |  |
//            0  3
//
// state=8
//
// state=12   3  0  rot180 + transpose
//            |  |
//            2--1
/// FillPlanePath.cpp:184-210 — `static inline Point hilbert_n_to_xy(const size_t n)`.
fn hilbert_n_to_xy(n: usize) -> Point {
    // FillPlanePath.cpp:186
    const NEXT_STATE: [i32; 16] = [4, 0, 0, 12, 0, 4, 4, 8, 12, 8, 8, 4, 8, 12, 12, 0];
    // FillPlanePath.cpp:187
    const DIGIT_TO_X: [Coord; 16] = [0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0];
    // FillPlanePath.cpp:188
    const DIGIT_TO_Y: [Coord; 16] = [0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1, 0, 0, 1];

    // FillPlanePath.cpp:190-198 — Number of 2 bit digits.
    let mut ndigits: usize = 0;
    {
        let mut nc = n;
        while nc > 0 {
            nc >>= 2;
            ndigits += 1;
        }
    }
    // FillPlanePath.cpp:199
    let mut state: i32 = if ndigits & 1 != 0 { 4 } else { 0 };
    // FillPlanePath.cpp:200
    let mut x: Coord = 0;
    // FillPlanePath.cpp:201
    let mut y: Coord = 0;
    // FillPlanePath.cpp:202
    let mut i: i32 = ndigits as i32 - 1;
    while i >= 0 {
        // FillPlanePath.cpp:203
        let digit: usize = ((n >> (i * 2)) & 3) as usize;
        // FillPlanePath.cpp:204
        state += digit as i32;
        // FillPlanePath.cpp:205
        x |= DIGIT_TO_X[state as usize] << i;
        // FillPlanePath.cpp:206
        y |= DIGIT_TO_Y[state as usize] << i;
        // FillPlanePath.cpp:207
        state = NEXT_STATE[state as usize];
        i -= 1;
    }
    // FillPlanePath.cpp:209
    Point::new(x, y)
}

/// FillPlanePath.cpp:212-232 — `generate_hilbert_curve`.
fn generate_hilbert_curve(
    min_x: Coord,
    min_y: Coord,
    max_x: Coord,
    max_y: Coord,
    output: &mut InfillPolylineOutput,
) {
    // FillPlanePath.cpp:215-224 — Minimum power of two square to fit the domain.
    let mut sz: usize = 2;
    let mut _pw: usize = 1;
    {
        // FillPlanePath.cpp:219 — size_t sz0 = std::max(max_x + 1 - min_x, max_y + 1 - min_y);
        // FIDELITY-NOTE(F2): C++ computes this in coord_t=int32; crate Coord=i64. The grid
        // extents here are small (ceil(bbox / distance_between_lines)), so no divergence in
        // practice; the int32-vs-i64 width only matters for pathological inputs.
        let sz0: usize = std::cmp::max(max_x + 1 - min_x, max_y + 1 - min_y) as usize;
        while sz < sz0 {
            sz <<= 1;
            _pw += 1;
        }
    }

    // FillPlanePath.cpp:226
    let sz2: usize = sz * sz;
    // FillPlanePath.cpp:227
    output.reserve(sz2);
    // FillPlanePath.cpp:228
    for i in 0..sz2 {
        // FillPlanePath.cpp:229
        let p = hilbert_n_to_xy(i);
        // FillPlanePath.cpp:230
        output.add_point(PointF::new((p.x() + min_x) as CoordF, (p.y() + min_y) as CoordF));
    }
}

// =============================================================================
// Octagram Spiral (FillPlanePath.cpp:242-280)
// =============================================================================

/// FillPlanePath.cpp:242-272 — `generate_octagram_spiral`.
fn generate_octagram_spiral(
    _min_x: Coord,
    _min_y: Coord,
    max_x: Coord,
    max_y: Coord,
    output: &mut InfillPolylineOutput,
) {
    // FillPlanePath.cpp:245-246 — Radius to achieve.
    let rmax: CoordF = ((max_x as CoordF) * (max_x as CoordF) + (max_y as CoordF) * (max_y as CoordF))
        .sqrt()
        * (2.0_f64).sqrt()
        + 1.5;
    // FillPlanePath.cpp:247 — Now unwind the spiral.
    // FillPlanePath.cpp:248
    let mut r: CoordF = 0.0;
    // FillPlanePath.cpp:249
    let r_inc: CoordF = (2.0_f64).sqrt();
    // FillPlanePath.cpp:250
    output.add_point(PointF::new(0.0, 0.0));
    // FillPlanePath.cpp:251
    while r < rmax {
        // FillPlanePath.cpp:252
        r += r_inc;
        // FillPlanePath.cpp:253
        let rx: CoordF = r / (2.0_f64).sqrt();
        // FillPlanePath.cpp:254
        let r2: CoordF = r + rx;
        // FillPlanePath.cpp:255
        output.add_point(PointF::new(r, 0.0));
        // FillPlanePath.cpp:256
        output.add_point(PointF::new(r2, rx));
        // FillPlanePath.cpp:257
        output.add_point(PointF::new(rx, rx));
        // FillPlanePath.cpp:258
        output.add_point(PointF::new(rx, r2));
        // FillPlanePath.cpp:259
        output.add_point(PointF::new(0.0, r));
        // FillPlanePath.cpp:260
        output.add_point(PointF::new(-rx, r2));
        // FillPlanePath.cpp:261
        output.add_point(PointF::new(-rx, rx));
        // FillPlanePath.cpp:262
        output.add_point(PointF::new(-r2, rx));
        // FillPlanePath.cpp:263
        output.add_point(PointF::new(-r, 0.0));
        // FillPlanePath.cpp:264
        output.add_point(PointF::new(-r2, -rx));
        // FillPlanePath.cpp:265
        output.add_point(PointF::new(-rx, -rx));
        // FillPlanePath.cpp:266
        output.add_point(PointF::new(-rx, -r2));
        // FillPlanePath.cpp:267
        output.add_point(PointF::new(0.0, -r));
        // FillPlanePath.cpp:268
        output.add_point(PointF::new(rx, -r2));
        // FillPlanePath.cpp:269
        output.add_point(PointF::new(rx, -rx));
        // FillPlanePath.cpp:270
        output.add_point(PointF::new(r2 + r_inc, -rx));
    }
}

// FillPlanePath.cpp:282 — } // namespace Slic3r

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Polygon;
    use crate::scale;

    fn make_square_boundary(size_mm: CoordF) -> ExPolygon {
        let size = scale(size_mm);
        let contour = Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(size, 0),
            Point::new(size, size),
            Point::new(0, size),
        ]);
        ExPolygon::new(contour)
    }

    fn default_params(density: f64) -> FillParams {
        FillParams {
            density,
            anchor_length_max: 1000.0,
            resolution: 0.0125,
            ..Default::default()
        }
    }

    #[test]
    fn test_hilbert_n_to_xy() {
        // First point is always origin.
        assert_eq!(hilbert_n_to_xy(0), Point::new(0, 0));
        // The first four points of the Hilbert curve are distinct and within [0,1].
        let pts: Vec<Point> = (0..4).map(hilbert_n_to_xy).collect();
        for p in &pts {
            assert!(p.x() >= 0 && p.x() <= 1);
            assert!(p.y() >= 0 && p.y() <= 1);
        }
        for i in 0..4 {
            for j in (i + 1)..4 {
                assert_ne!(pts[i], pts[j]);
            }
        }
    }

    #[test]
    fn test_archimedean_first_points() {
        let mut output = InfillPolylineOutput::new(1.0);
        generate_archimedean_chords(0, 0, 10, 10, 0.1, &mut output);
        let pts = output.result();
        assert!(pts.len() > 2);
        // FillPlanePath.cpp:149-150 — first two points are (0,0) and scaled (1,0).
        assert_eq!(pts[0], Point::new(0, 0));
        assert_eq!(pts[1], Point::new(1, 0));
    }

    #[test]
    fn test_octagram_first_point() {
        let mut output = InfillPolylineOutput::new(1.0);
        generate_octagram_spiral(0, 0, 10, 10, &mut output);
        let pts = output.result();
        assert!(pts.len() > 1);
        // FillPlanePath.cpp:250 — first point is (0,0).
        assert_eq!(pts[0], Point::new(0, 0));
    }

    #[test]
    fn test_hilbert_curve_point_count() {
        let mut output = InfillPolylineOutput::new(1.0);
        generate_hilbert_curve(0, 0, 3, 3, &mut output);
        let pts = output.result();
        // 4x4 grid -> 16 points.
        assert_eq!(pts.len(), 16);
        assert_eq!(pts[0], Point::new(0, 0));
    }

    #[test]
    fn test_clipper_keeps_inside_points() {
        // A bounding box [0,0]-[10,10]; points outside on the same side collapse.
        let bbox = BoundingBox::from_points_minmax(Point::new(0, 0), Point::new(10, 10));
        let mut clip = InfillPolylineOutput::new_clipper(bbox, 1.0);
        assert!(clip.clips());
        clip.add_point(PointF::new(5.0, 5.0));
        clip.add_point(PointF::new(6.0, 6.0));
        clip.add_point(PointF::new(7.0, 7.0));
        let pts = clip.result();
        assert_eq!(pts.len(), 3);
    }

    #[test]
    fn test_fill_surface_single_hilbert() {
        let boundary = make_square_boundary(50.0);
        let mut fill = FillPlanePath::new(PlanPathPattern::HilbertCurve);
        fill.spacing = 0.45;
        // For non-aligned (solid) path, density >= 0.995 avoids needing object bbox.
        let params = default_params(1.0);
        let mut out: Vec<Polyline> = Vec::new();
        fill._fill_surface_single(&params, 1, &(0.0, Point::new(0, 0)), boundary, &mut out);
        // Hilbert should produce some clipped paths for a solid square.
        assert!(!out.is_empty());
    }

    #[test]
    fn test_fill_surface_single_archimedean_solid() {
        let boundary = make_square_boundary(50.0);
        let mut fill = FillPlanePath::new(PlanPathPattern::ArchimedeanChords);
        fill.spacing = 0.45;
        let params = default_params(1.0);
        let mut out: Vec<Polyline> = Vec::new();
        fill._fill_surface_single(&params, 1, &(0.0, Point::new(0, 0)), boundary, &mut out);
        assert!(!out.is_empty());
    }

    #[test]
    fn test_centered_flags() {
        assert!(FillPlanePath::new(PlanPathPattern::ArchimedeanChords).centered());
        assert!(!FillPlanePath::new(PlanPathPattern::HilbertCurve).centered());
        assert!(FillPlanePath::new(PlanPathPattern::OctagramSpiral).centered());
    }
}
