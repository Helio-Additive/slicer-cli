//! Line segmentation algorithm
//!
//! 1:1 faithful port of `Algorithm/LineSegmentation/LineSegmentation.cpp`
//! (BambuStudio) and its header `LineSegmentation.hpp`.
//!
//! C++ Reference: src/libslic3r/Algorithm/LineSegmentation/LineSegmentation.cpp
//!               src/libslic3r/Algorithm/LineSegmentation/LineSegmentation.hpp
//!
//! coord_t  -> i64
//! coordf_t -> f64
//!
//! ## BLOCKED symbols (see notes at the bottom of this module)
//!
//! FIDELITY-NOTE(F1): geometry-backend blocker (NOT a per-file fix).
//!
//! The core of this file, `intersection_with_region`, requires the *legacy*
//! `ClipperLib_Z::Clipper` engine with a user-supplied four-endpoint
//! `ZFillFunction` callback (computing `new_pt.z()` from the four edge
//! endpoints), an *open*-subject intersection
//! `Execute(ctIntersection, PolyTree, pftNonZero, pftNonZero)` and
//! `PolyTreeToPaths`. None of the crate's three available clipper backends
//! provide this:
//!   - `clipper_utils` uses the `geo` crate (geo-clipper, fixed scale 1000),
//!   - `clipper2_utils` uses `clipper2c-sys` whose `ClipperPoint64{x,y}` has NO
//!     `z` field and exposes no `ZFillFunction` callback,
//!   - `clipper2_z_utils` is a header-only conversion + intersection-*visitor*
//!     helper (negative-index `z` scheme) with NO wired engine, no `PolyTree`
//!     traversal, and no open-subject Z `Execute`. Its negative-index `z`
//!     provenance scheme is also fundamentally incompatible with the 30/31-bit
//!     `ZAttributes` encoding this algorithm relies on.
//! Re-routing this to a faithful `ClipperLib_Z` path is a CROSS-CUTTING
//! architectural change and adding a native legacy-Clipper dep is forbidden
//! (wasm-safety). This is the same blocker documented in `overhang_detector.rs`
//! (`clip_extrusion`). Consequently `intersection_with_region`,
//! `subject_segmentation`, and all public `*_segmentation` entry points are NOT
//! PORTED. Everything tractable (the pure helpers) is ported faithfully below.

use crate::arachne::utils::extrusion_junction::ExtrusionJunction;
use crate::arachne::utils::extrusion_line::ExtrusionLine;
use crate::clipper_z_utils::{ZPath, ZPoint};
use crate::geometry::{Coord, CoordF, Point, PointF, Polyline};

use crate::region_config::PrintRegionConfig;

// Point.hpp:298-302
// inline Point lerp(const Point &a, const Point &b, double t)
// {
//     assert((t >= -EPSILON) && (t <= 1. + EPSILON));
//     return ((1. - t) * a.cast<double>() + t * b.cast<double>()).cast<coord_t>();
// }
//
// NOTE: We define the faithful Point lerp locally because the `point` submodule is
// private and the *exported* `geometry::lerp` rounds (`(a + (b-a)*t).round()`)
// whereas C++ truncates via `.cast<coord_t>()`. Using the exported one would
// diverge by a sub-unit in rounding; we mirror Point.hpp:298 exactly.
#[inline]
fn lerp(a: &Point, b: &Point, t: CoordF) -> Point {
    debug_assert!(t >= -crate::libslic3r::EPSILON && t <= 1.0 + crate::libslic3r::EPSILON);
    Point::new(
        ((1.0 - t) * a.x as CoordF + t * b.x as CoordF) as Coord,
        ((1.0 - t) * a.y as CoordF + t * b.y as CoordF) as Coord,
    )
}

// LineSegmentation.cpp:19 — namespace Slic3r::Algorithm::LineSegmentation

// LineSegmentation.cpp:21
// const constexpr coord_t POINT_IS_ON_LINE_THRESHOLD_SQR = Slic3r::sqr(scaled<coord_t>(EPSILON));
//
// scaled<coord_t>(EPSILON) == round(EPSILON * SCALING_FACTOR) == round(1e-4 * 100000.0) == 10.
// Slic3r::sqr(10) == 100. (libslic3r.h:275-277 — sqr(x) == x * x.)
const POINT_IS_ON_LINE_THRESHOLD_SQR: i64 = {
    let s: i64 = (crate::libslic3r::EPSILON * crate::SCALING_FACTOR) as i64; // == 10
    s * s
};

// ---------------------------------------------------------------------------
// LineSegmentation.hpp:26-57 — public segment types
// ---------------------------------------------------------------------------

// LineSegmentation.hpp:26-30
// struct PolylineSegment { Polyline polyline; size_t clip_idx; };
#[derive(Debug, Clone)]
pub struct PolylineSegment {
    pub polyline: Polyline,
    pub clip_idx: usize,
}

// LineSegmentation.hpp:32-38
// struct PolylineRegionSegment { Polyline polyline; const PrintRegionConfig &config; ... };
#[derive(Debug, Clone)]
pub struct PolylineRegionSegment {
    pub polyline: Polyline,
    pub config: PrintRegionConfig,
}

impl PolylineRegionSegment {
    // LineSegmentation.hpp:37
    // PolylineRegionSegment(const Polyline &polyline, const PrintRegionConfig &config) : polyline(polyline), config(config) {}
    pub fn new(polyline: Polyline, config: PrintRegionConfig) -> Self {
        Self { polyline, config }
    }
}

// LineSegmentation.hpp:40-44
// struct ExtrusionSegment { Arachne::ExtrusionLine extrusion; size_t clip_idx; };
#[derive(Debug, Clone)]
pub struct ExtrusionSegment {
    pub extrusion: ExtrusionLine,
    pub clip_idx: usize,
}

// LineSegmentation.hpp:46-52
// struct ExtrusionRegionSegment { Arachne::ExtrusionLine extrusion; const PrintRegionConfig &config; ... };
#[derive(Debug, Clone)]
pub struct ExtrusionRegionSegment {
    pub extrusion: ExtrusionLine,
    pub config: PrintRegionConfig,
}

impl ExtrusionRegionSegment {
    // LineSegmentation.hpp:51
    // ExtrusionRegionSegment(const Arachne::ExtrusionLine &extrusion, const PrintRegionConfig &config) : extrusion(extrusion), config(config) {}
    pub fn new(extrusion: ExtrusionLine, config: PrintRegionConfig) -> Self {
        Self { extrusion, config }
    }
}

// LineSegmentation.hpp:54-57
// using PolylineSegments        = std::vector<PolylineSegment>;
// using ExtrusionSegments       = std::vector<ExtrusionSegment>;
// using PolylineRegionSegments  = std::vector<PolylineRegionSegment>;
// using ExtrusionRegionSegments = std::vector<ExtrusionRegionSegment>;
pub type PolylineSegments = Vec<PolylineSegment>;
pub type ExtrusionSegments = Vec<ExtrusionSegment>;
pub type PolylineRegionSegments = Vec<PolylineRegionSegment>;
pub type ExtrusionRegionSegments = Vec<ExtrusionRegionSegment>;

// ---------------------------------------------------------------------------
// LineSegmentation.cpp:23-56 — struct ZAttributes
// ---------------------------------------------------------------------------

// LineSegmentation.cpp:23-56
#[derive(Debug, Clone, Copy)]
pub struct ZAttributes {
    // LineSegmentation.cpp:25 — bool is_clip_point = false;
    pub is_clip_point: bool,
    // LineSegmentation.cpp:26 — bool is_new_point  = false;
    pub is_new_point: bool,
    // LineSegmentation.cpp:27 — uint32_t point_index = 0;
    pub point_index: u32,
}

impl ZAttributes {
    // LineSegmentation.cpp:29 — ZAttributes() = default;
    pub fn default_z() -> Self {
        Self {
            is_clip_point: false,
            is_new_point: false,
            point_index: 0,
        }
    }

    // LineSegmentation.cpp:31-32
    // explicit ZAttributes(const uint32_t clipper_coord) :
    //     is_clip_point((clipper_coord >> 31) & 0x1), is_new_point((clipper_coord >> 30) & 0x1), point_index(clipper_coord & 0x3FFFFFFF) {}
    pub fn from_clipper_coord(clipper_coord: u32) -> Self {
        Self {
            is_clip_point: ((clipper_coord >> 31) & 0x1) != 0,
            is_new_point: ((clipper_coord >> 30) & 0x1) != 0,
            point_index: clipper_coord & 0x3FFF_FFFF,
        }
    }

    // LineSegmentation.cpp:34 — explicit ZAttributes(const ClipperLib_Z::IntPoint &clipper_pt) : ZAttributes(clipper_pt.z()) {}
    pub fn from_clipper_pt(clipper_pt: &ZPoint) -> Self {
        // The z field of a ZPoint is i64; the encoded value is a uint32_t.
        ZAttributes::from_clipper_coord(clipper_pt.2 as u32)
    }

    // LineSegmentation.cpp:36-40
    // ZAttributes(const bool is_clip_point, const bool is_new_point, const uint32_t point_index) :
    //     is_clip_point(is_clip_point), is_new_point(is_new_point), point_index(point_index)
    // { assert(this->point_index < (1u << 30) && "point_index exceeds 30 bits!"); }
    pub fn new(is_clip_point: bool, is_new_point: bool, point_index: u32) -> Self {
        debug_assert!(point_index < (1u32 << 30), "point_index exceeds 30 bits!");
        Self {
            is_clip_point,
            is_new_point,
            point_index,
        }
    }

    // LineSegmentation.cpp:42-47
    // Encode the structure to uint32_t.
    // constexpr uint32_t encode() const
    // {
    //     assert(this->point_index < (1u << 30) && "point_index exceeds 30 bits!");
    //     return (this->is_clip_point << 31) | (this->is_new_point << 30) | (this->point_index & 0x3FFFFFFF);
    // }
    pub fn encode(&self) -> u32 {
        debug_assert!(
            self.point_index < (1u32 << 30),
            "point_index exceeds 30 bits!"
        );
        ((self.is_clip_point as u32) << 31)
            | ((self.is_new_point as u32) << 30)
            | (self.point_index & 0x3FFF_FFFF)
    }

    // LineSegmentation.cpp:49-53
    // Decode the uint32_t to the structure.
    // static ZAttributes decode(const uint32_t clipper_coord)
    // {
    //     return { bool((clipper_coord >> 31) & 0x1), bool((clipper_coord >> 30) & 0x1), clipper_coord & 0x3FFFFFFF };
    // }
    pub fn decode(clipper_coord: u32) -> Self {
        ZAttributes::new(
            ((clipper_coord >> 31) & 0x1) != 0,
            ((clipper_coord >> 30) & 0x1) != 0,
            clipper_coord & 0x3FFF_FFFF,
        )
    }

    // LineSegmentation.cpp:55 — static ZAttributes decode(const ClipperLib_Z::IntPoint &clipper_pt) { return ZAttributes::decode(clipper_pt.z()); }
    pub fn decode_pt(clipper_pt: &ZPoint) -> Self {
        ZAttributes::decode(clipper_pt.2 as u32)
    }
}

// ---------------------------------------------------------------------------
// LineSegmentation.cpp:58-102 — struct LineRegionRange
// ---------------------------------------------------------------------------

// LineSegmentation.cpp:58-102
#[derive(Debug, Clone, Copy)]
pub struct LineRegionRange {
    // LineSegmentation.cpp:60 — size_t begin_idx; // Index of the line on which the region begins.
    pub begin_idx: usize,
    // LineSegmentation.cpp:61 — double begin_t; // Scalar position on the begin_idx line in which the region begins. The value is from range <0., 1.>.
    pub begin_t: f64,
    // LineSegmentation.cpp:62 — size_t end_idx; // Index of the line on which the region ends.
    pub end_idx: usize,
    // LineSegmentation.cpp:63 — double end_t; // Scalar position on the end_idx line in which the region ends. The value is from range <0., 1.>.
    pub end_t: f64,
    // LineSegmentation.cpp:64 — size_t clip_idx; // Index of clipping ExPolygons to identified which ExPolygons group contains this line.
    pub clip_idx: usize,
}

impl LineRegionRange {
    // LineSegmentation.cpp:66-67
    // LineRegionRange(size_t begin_idx, double begin_t, size_t end_idx, double end_t, size_t clip_idx)
    //     : begin_idx(begin_idx), begin_t(begin_t), end_idx(end_idx), end_t(end_t), clip_idx(clip_idx) {}
    pub fn new(begin_idx: usize, begin_t: f64, end_idx: usize, end_t: f64, clip_idx: usize) -> Self {
        Self {
            begin_idx,
            begin_t,
            end_idx,
            end_t,
            clip_idx,
        }
    }

    // LineSegmentation.cpp:69-81
    // Check if 'other' overlaps with this LineRegionRange.
    // bool is_overlap(const LineRegionRange &other) const
    pub fn is_overlap(&self, other: &LineRegionRange) -> bool {
        // LineSegmentation.cpp:72-78
        if self.end_idx < other.begin_idx || self.begin_idx > other.end_idx {
            return false;
        } else if self.end_idx == other.begin_idx && self.end_t <= other.begin_t {
            return false;
        } else if self.begin_idx == other.end_idx && self.begin_t >= other.end_t {
            return false;
        }

        // LineSegmentation.cpp:80
        true
    }

    // LineSegmentation.cpp:83-94
    // Check if 'inner' is whole inside this LineRegionRange.
    // bool is_inside(const LineRegionRange &inner) const
    pub fn is_inside(&self, inner: &LineRegionRange) -> bool {
        // LineSegmentation.cpp:86-88
        if !self.is_overlap(inner) {
            return false;
        }

        // LineSegmentation.cpp:90
        let starts_after = (self.begin_idx < inner.begin_idx)
            || (self.begin_idx == inner.begin_idx && self.begin_t <= inner.begin_t);
        // LineSegmentation.cpp:91
        let ends_before = (self.end_idx > inner.end_idx)
            || (self.end_idx == inner.end_idx && self.end_t >= inner.end_t);

        // LineSegmentation.cpp:93
        starts_after && ends_before
    }

    // LineSegmentation.cpp:96 — bool is_zero_length() const { return this->begin_idx == this->end_idx && this->begin_t == this->end_t; }
    pub fn is_zero_length(&self) -> bool {
        self.begin_idx == self.end_idx && self.begin_t == self.end_t
    }

    // LineSegmentation.cpp:98-101
    // bool operator<(const LineRegionRange &rhs) const
    // {
    //     return this->begin_idx < rhs.begin_idx || (this->begin_idx == rhs.begin_idx && this->begin_t < rhs.begin_t);
    // }
    pub fn less(&self, rhs: &LineRegionRange) -> bool {
        self.begin_idx < rhs.begin_idx
            || (self.begin_idx == rhs.begin_idx && self.begin_t < rhs.begin_t)
    }
}

// LineSegmentation.cpp:104 — using LineRegionRanges = std::vector<LineRegionRange>;
pub type LineRegionRanges = Vec<LineRegionRange>;

// LineSegmentation.cpp:106 — inline Point make_point(const ClipperLib_Z::IntPoint &clipper_pt) { return { clipper_pt.x(), clipper_pt.y() }; }
#[inline]
pub fn make_point(clipper_pt: &ZPoint) -> Point {
    Point::new(clipper_pt.0, clipper_pt.1)
}

// LineSegmentation.cpp:108
// inline ClipperLib_Z::Paths to_clip_zpaths(const ExPolygons &clips) { return ClipperZUtils::expolygons_to_zpaths_with_same_z<false>(clips, coord_t(ZAttributes(true, false, 0).encode())); }
//
// NOTE: tractable on its own — relies only on `expolygons_to_zpaths_with_same_z`.
#[inline]
pub fn to_clip_zpaths(clips: &[crate::geometry::ExPolygon]) -> crate::clipper_z_utils::ZPaths {
    crate::clipper_z_utils::expolygons_to_zpaths_with_same_z(
        clips,
        ZAttributes::new(true, false, 0).encode() as i64,
        false,
    )
}

// ---------------------------------------------------------------------------
// LineSegmentation.cpp:110-150 — subject_to_zpath overloads
// ---------------------------------------------------------------------------

// LineSegmentation.cpp:110-129
// static ClipperLib_Z::Path subject_to_zpath(const Points &subject, const bool is_closed)
fn subject_to_zpath_points(subject: &[Point], is_closed: bool) -> ZPath {
    // LineSegmentation.cpp:112 — ZAttributes z_attributes(false, false, 0);
    let mut z_attributes = ZAttributes::new(false, false, 0);

    // LineSegmentation.cpp:114 — ClipperLib_Z::Path out;
    let mut out: ZPath = Vec::new();
    // LineSegmentation.cpp:115 — if (!subject.empty()) {
    if !subject.is_empty() {
        // LineSegmentation.cpp:116 — out.reserve((subject.size() + is_closed) ? 1 : 0);
        out.reserve(if (subject.len() + is_closed as usize) != 0 {
            1
        } else {
            0
        });
        // LineSegmentation.cpp:117-120
        // for (const Point &p : subject) {
        //     out.emplace_back(p.x(), p.y(), z_attributes.encode());
        //     ++z_attributes.point_index;
        // }
        for p in subject {
            out.push((p.x(), p.y(), z_attributes.encode() as i64));
            z_attributes.point_index += 1;
        }

        // LineSegmentation.cpp:122-125
        if is_closed {
            // If it is closed, then duplicate the first point at the end to make a closed path open.
            out.push((
                subject[0].x(),
                subject[0].y(),
                z_attributes.encode() as i64,
            ));
        }
    }

    // LineSegmentation.cpp:128
    out
}

// LineSegmentation.cpp:131-146
// static ClipperLib_Z::Path subject_to_zpath(const Arachne::ExtrusionLine &subject)
fn subject_to_zpath_extrusion(subject: &ExtrusionLine) -> ZPath {
    // Closed Arachne::ExtrusionLine already has duplicated the last point.
    // LineSegmentation.cpp:134 — ZAttributes z_attributes(false, false, 0);
    let mut z_attributes = ZAttributes::new(false, false, 0);

    // LineSegmentation.cpp:136 — ClipperLib_Z::Path out;
    let mut out: ZPath = Vec::new();
    // LineSegmentation.cpp:137 — if (!subject.empty()) {
    if !subject.is_empty() {
        // LineSegmentation.cpp:138 — out.reserve(subject.size());
        out.reserve(subject.size());
        // LineSegmentation.cpp:139-142
        // for (const Arachne::ExtrusionJunction &junction : subject) {
        //     out.emplace_back(junction.p.x(), junction.p.y(), z_attributes.encode());
        //     ++z_attributes.point_index;
        // }
        for junction in &subject.junctions {
            out.push((
                junction.p.x(),
                junction.p.y(),
                z_attributes.encode() as i64,
            ));
            z_attributes.point_index += 1;
        }
    }

    // LineSegmentation.cpp:145
    out
}

// LineSegmentation.cpp:148 — static ClipperLib_Z::Path subject_to_zpath(const Polyline &subject) { return subject_to_zpath(subject.points, false); }
fn subject_to_zpath_polyline(subject: &Polyline) -> ZPath {
    subject_to_zpath_points(&subject.points, false)
}

// LineSegmentation.cpp:150 — [[maybe_unused]] static ClipperLib_Z::Path subject_to_zpath(const Polygon &subject) { return subject_to_zpath(subject.points, true); }
#[allow(dead_code)]
fn subject_to_zpath_polygon(subject: &crate::geometry::Polygon) -> ZPath {
    subject_to_zpath_points(&subject.points, true)
}

// ---------------------------------------------------------------------------
// LineSegmentation.cpp:152-179 — ProjectionInfo / project_point_on_line
// ---------------------------------------------------------------------------

// LineSegmentation.cpp:152-156
// struct ProjectionInfo { double projected_t; double distance_sqr; };
#[derive(Debug, Clone, Copy)]
pub struct ProjectionInfo {
    pub projected_t: f64,
    pub distance_sqr: f64,
}

// LineSegmentation.cpp:158-179
// static ProjectionInfo project_point_on_line(const Point &line_from_pt, const Point &line_to_pt, const Point &query_pt)
pub fn project_point_on_line(
    line_from_pt: &Point,
    line_to_pt: &Point,
    query_pt: &Point,
) -> ProjectionInfo {
    // LineSegmentation.cpp:160 — const Vec2d line_vec = (line_to_pt - line_from_pt).template cast<double>();
    let line_vec = PointF::new(
        (line_to_pt.x() - line_from_pt.x()) as f64,
        (line_to_pt.y() - line_from_pt.y()) as f64,
    );
    // LineSegmentation.cpp:161 — const Vec2d query_vec = (query_pt - line_from_pt).template cast<double>();
    let query_vec = PointF::new(
        (query_pt.x() - line_from_pt.x()) as f64,
        (query_pt.y() - line_from_pt.y()) as f64,
    );
    // LineSegmentation.cpp:162 — const double line_length_sqr = line_vec.squaredNorm();
    let line_length_sqr = line_vec.x() * line_vec.x() + line_vec.y() * line_vec.y();

    // LineSegmentation.cpp:164-166
    // if (line_length_sqr <= 0.) {
    //     return { std::numeric_limits<double>::max(), std::numeric_limits<double>::max() };
    // }
    if line_length_sqr <= 0. {
        return ProjectionInfo {
            projected_t: f64::MAX,
            distance_sqr: f64::MAX,
        };
    }

    // LineSegmentation.cpp:168 — const double projected_t = query_vec.dot(line_vec);
    let projected_t = query_vec.x() * line_vec.x() + query_vec.y() * line_vec.y();
    // LineSegmentation.cpp:169 — const double projected_t_normalized = std::clamp(projected_t / line_length_sqr, 0., 1.);
    let projected_t_normalized = (projected_t / line_length_sqr).clamp(0., 1.);
    // Projected point have to line on the line.
    // LineSegmentation.cpp:171-173
    // if (projected_t < 0. || projected_t > line_length_sqr) {
    //     return { projected_t_normalized, std::numeric_limits<double>::max() };
    // }
    if projected_t < 0. || projected_t > line_length_sqr {
        return ProjectionInfo {
            projected_t: projected_t_normalized,
            distance_sqr: f64::MAX,
        };
    }

    // LineSegmentation.cpp:175 — const Vec2d projected_vec = projected_t_normalized * line_vec;
    let projected_vec = PointF::new(
        projected_t_normalized * line_vec.x(),
        projected_t_normalized * line_vec.y(),
    );
    // LineSegmentation.cpp:176 — const double distance_sqr = (projected_vec - query_vec).squaredNorm();
    let diff = PointF::new(
        projected_vec.x() - query_vec.x(),
        projected_vec.y() - query_vec.y(),
    );
    let distance_sqr = diff.x() * diff.x() + diff.y() * diff.y();

    // LineSegmentation.cpp:178
    ProjectionInfo {
        projected_t: projected_t_normalized,
        distance_sqr,
    }
}

// ---------------------------------------------------------------------------
// LineSegmentation.cpp:181-209 — find_closest_line_to_point
// ---------------------------------------------------------------------------

// LineSegmentation.cpp:181-209
// static int32_t find_closest_line_to_point(const ClipperLib_Z::Path &subject, const ClipperLib_Z::IntPoint &query)
pub fn find_closest_line_to_point(subject: &ZPath, query: &ZPoint) -> i32 {
    // LineSegmentation.cpp:183 — auto it_min = subject.end();
    let mut it_min: Option<usize> = None;
    // LineSegmentation.cpp:184 — double distance_sqr_min = std::numeric_limits<double>::max();
    let mut distance_sqr_min = f64::MAX;

    // LineSegmentation.cpp:186 — const Point query_pt = make_point(query);
    let query_pt = make_point(query);
    // LineSegmentation.cpp:187 — Point prev_pt = make_point(subject.front());
    let mut prev_pt = make_point(&subject[0]);
    // LineSegmentation.cpp:188 — for (auto it_curr = std::next(subject.begin()); it_curr != subject.end(); ++it_curr) {
    for it_curr in 1..subject.len() {
        // LineSegmentation.cpp:189 — const Point curr_pt = make_point(*it_curr);
        let curr_pt = make_point(&subject[it_curr]);

        // LineSegmentation.cpp:191 — const double distance_sqr = project_point_on_line(prev_pt, curr_pt, query_pt).distance_sqr;
        let distance_sqr = project_point_on_line(&prev_pt, &curr_pt, &query_pt).distance_sqr;
        // LineSegmentation.cpp:192-194
        // if (distance_sqr <= POINT_IS_ON_LINE_THRESHOLD_SQR) {
        //     return int32_t(std::distance(subject.begin(), std::prev(it_curr)));
        // }
        if distance_sqr <= POINT_IS_ON_LINE_THRESHOLD_SQR as f64 {
            return (it_curr - 1) as i32;
        }

        // LineSegmentation.cpp:196-199
        // if (distance_sqr < distance_sqr_min) {
        //     distance_sqr_min = distance_sqr;
        //     it_min           = std::prev(it_curr);
        // }
        if distance_sqr < distance_sqr_min {
            distance_sqr_min = distance_sqr;
            it_min = Some(it_curr - 1);
        }

        // LineSegmentation.cpp:201 — prev_pt = curr_pt;
        prev_pt = curr_pt;
    }

    // LineSegmentation.cpp:204-206
    // if (it_min != subject.end()) {
    //     return int32_t(std::distance(subject.begin(), it_min));
    // }
    if let Some(idx) = it_min {
        return idx as i32;
    }

    // LineSegmentation.cpp:208 — return -1;
    -1
}

// ---------------------------------------------------------------------------
// LineSegmentation.cpp:211-289 — create_line_region_range
// ---------------------------------------------------------------------------

// LineSegmentation.cpp:211
// std::optional<LineRegionRange> create_line_region_range(ClipperLib_Z::Path &&intersection, const ClipperLib_Z::Path &subject, const size_t region_idx)
pub fn create_line_region_range(
    mut intersection: ZPath,
    subject: &ZPath,
    region_idx: usize,
) -> Option<LineRegionRange> {
    // LineSegmentation.cpp:213-215
    if intersection.len() < 2 {
        return None;
    }

    // LineSegmentation.cpp:217-250
    // auto need_reverse = [&subject](const ClipperLib_Z::Path &intersection) -> bool { ... };
    let need_reverse = |intersection: &ZPath| -> bool {
        // LineSegmentation.cpp:218 — for (size_t curr_idx = 1; curr_idx < intersection.size(); ++curr_idx) {
        for curr_idx in 1..intersection.len() {
            // LineSegmentation.cpp:219 — Point pre_pos = Point(intersection[curr_idx - 1].x(), intersection[curr_idx - 1].y());
            let pre_pos = Point::new(intersection[curr_idx - 1].0, intersection[curr_idx - 1].1);
            // LineSegmentation.cpp:220 — Point cur_pos = Point(intersection[curr_idx].x(), intersection[curr_idx].y());
            let cur_pos = Point::new(intersection[curr_idx].0, intersection[curr_idx].1);
            // LineSegmentation.cpp:221 — ZAttributes prev_z(intersection[curr_idx - 1]);
            let prev_z = ZAttributes::from_clipper_pt(&intersection[curr_idx - 1]);
            // LineSegmentation.cpp:222 — ZAttributes curr_z(intersection[curr_idx]);
            let curr_z = ZAttributes::from_clipper_pt(&intersection[curr_idx]);

            // LineSegmentation.cpp:224 — if (!prev_z.is_clip_point && !curr_z.is_clip_point) {
            if !prev_z.is_clip_point && !curr_z.is_clip_point {
                // There may be repeated intersections on different line segments
                // LineSegmentation.cpp:226 — int max_point_idx = subject.size() - 1;
                let max_point_idx: i32 = subject.len() as i32 - 1;
                // LineSegmentation.cpp:227 — bool is_valid_order = prev_z.point_index <= curr_z.point_index;
                let mut is_valid_order = prev_z.point_index <= curr_z.point_index;
                // LineSegmentation.cpp:228-229
                // if ((curr_z.point_index == max_point_idx) && (prev_z.point_index == 0)) is_valid_order = false;
                if (curr_z.point_index as i32 == max_point_idx) && (prev_z.point_index == 0) {
                    is_valid_order = false;
                }
                // LineSegmentation.cpp:230-231
                // if ((curr_z.point_index == 0) && (prev_z.point_index == max_point_idx)) is_valid_order = true;
                if (curr_z.point_index == 0) && (prev_z.point_index as i32 == max_point_idx) {
                    is_valid_order = true;
                }
                // LineSegmentation.cpp:232-233
                // if (!is_valid_order && (pre_pos != cur_pos)) { return true; }
                if !is_valid_order && (pre_pos != cur_pos) {
                    return true;
                // LineSegmentation.cpp:234 — } else if (curr_z.point_index == prev_z.point_index) {
                } else if curr_z.point_index == prev_z.point_index {
                    // LineSegmentation.cpp:235 — assert(curr_z.point_index < subject.size());
                    debug_assert!((curr_z.point_index as usize) < subject.len());
                    // LineSegmentation.cpp:236 — const Point subject_pt = make_point(subject[curr_z.point_index]);
                    let subject_pt = make_point(&subject[curr_z.point_index as usize]);
                    // LineSegmentation.cpp:237 — const Point prev_pt = make_point(intersection[curr_idx - 1]);
                    let prev_pt = make_point(&intersection[curr_idx - 1]);
                    // LineSegmentation.cpp:238 — const Point curr_pt = make_point(intersection[curr_idx]);
                    let curr_pt = make_point(&intersection[curr_idx]);

                    // LineSegmentation.cpp:240 — const double prev_dist = (prev_pt - subject_pt).cast<double>().squaredNorm();
                    let prev_dx = (prev_pt.x() - subject_pt.x()) as f64;
                    let prev_dy = (prev_pt.y() - subject_pt.y()) as f64;
                    let prev_dist = prev_dx * prev_dx + prev_dy * prev_dy;
                    // LineSegmentation.cpp:241 — const double curr_dist = (curr_pt - subject_pt).cast<double>().squaredNorm();
                    let curr_dx = (curr_pt.x() - subject_pt.x()) as f64;
                    let curr_dy = (curr_pt.y() - subject_pt.y()) as f64;
                    let curr_dist = curr_dx * curr_dx + curr_dy * curr_dy;
                    // LineSegmentation.cpp:242-244 — if (prev_dist > curr_dist) { return true; }
                    if prev_dist > curr_dist {
                        return true;
                    }
                }
            }
        }

        // LineSegmentation.cpp:249
        false
    };

    // LineSegmentation.cpp:252 — for (ClipperLib_Z::IntPoint &clipper_pt : intersection) {
    for clipper_pt in intersection.iter_mut() {
        // LineSegmentation.cpp:253 — const ZAttributes clipper_pt_z(clipper_pt);
        let clipper_pt_z = ZAttributes::from_clipper_pt(clipper_pt);
        // LineSegmentation.cpp:254-256 — if (!clipper_pt_z.is_clip_point) { continue; }
        if !clipper_pt_z.is_clip_point {
            continue;
        }

        // FIXME @hejllukas: We could save searing for the source line in some cases using other intersection points,
        //                   but in reality, the clip point will be inside the intersection in very rare cases.
        // LineSegmentation.cpp:260-262
        // if (int32_t subject_line_idx = find_closest_line_to_point(subject, clipper_pt); subject_line_idx != -1) {
        //     clipper_pt.z() = coord_t(ZAttributes(false, true, subject_line_idx).encode());
        // }
        let subject_line_idx = find_closest_line_to_point(subject, clipper_pt);
        if subject_line_idx != -1 {
            clipper_pt.2 = ZAttributes::new(false, true, subject_line_idx as u32).encode() as i64;
        }

        // LineSegmentation.cpp:264 — assert(!ZAttributes(clipper_pt).is_clip_point);
        debug_assert!(!ZAttributes::from_clipper_pt(clipper_pt).is_clip_point);
        // LineSegmentation.cpp:265-267 — if (ZAttributes(clipper_pt).is_clip_point) { return std::nullopt; }
        if ZAttributes::from_clipper_pt(clipper_pt).is_clip_point {
            return None;
        }
    }

    // Ensure that indices of source input are ordered in increasing order.
    // LineSegmentation.cpp:271-273 — if (need_reverse(intersection)) { std::reverse(intersection.begin(), intersection.end()); }
    if need_reverse(&intersection) {
        intersection.reverse();
    }

    // LineSegmentation.cpp:275 — ZAttributes begin_z(intersection.front());
    let begin_z = ZAttributes::from_clipper_pt(&intersection[0]);
    // LineSegmentation.cpp:276 — ZAttributes end_z(intersection.back());
    let end_z = ZAttributes::from_clipper_pt(&intersection[intersection.len() - 1]);

    // LineSegmentation.cpp:278 — assert(begin_z.point_index <= subject.size() && end_z.point_index <= subject.size());
    debug_assert!(
        (begin_z.point_index as usize) <= subject.len()
            && (end_z.point_index as usize) <= subject.len()
    );
    // LineSegmentation.cpp:279 — const size_t begin_idx = begin_z.point_index;
    let begin_idx = begin_z.point_index as usize;
    // LineSegmentation.cpp:280 — const size_t end_idx = end_z.point_index;
    let end_idx = end_z.point_index as usize;
    // LineSegmentation.cpp:281
    // const double begin_t = begin_z.is_new_point ? project_point_on_line(make_point(subject[begin_idx]), make_point(subject[begin_idx + 1]), make_point(intersection.front())).projected_t : 0.;
    let begin_t = if begin_z.is_new_point {
        project_point_on_line(
            &make_point(&subject[begin_idx]),
            &make_point(&subject[begin_idx + 1]),
            &make_point(&intersection[0]),
        )
        .projected_t
    } else {
        0.
    };
    // LineSegmentation.cpp:282
    // const double end_t = end_z.is_new_point ? project_point_on_line(make_point(subject[end_idx]), make_point(subject[end_idx + 1]), make_point(intersection.back())).projected_t : 0.;
    let end_t = if end_z.is_new_point {
        project_point_on_line(
            &make_point(&subject[end_idx]),
            &make_point(&subject[end_idx + 1]),
            &make_point(&intersection[intersection.len() - 1]),
        )
        .projected_t
    } else {
        0.
    };

    // LineSegmentation.cpp:284-286
    // if (begin_t == std::numeric_limits<double>::max() || end_t == std::numeric_limits<double>::max()) {
    //     return std::nullopt;
    // }
    if begin_t == f64::MAX || end_t == f64::MAX {
        return None;
    }

    // LineSegmentation.cpp:288 — return LineRegionRange{ begin_idx, begin_t, end_idx, end_t, region_idx };
    Some(LineRegionRange::new(
        begin_idx, begin_t, end_idx, end_t, region_idx,
    ))
}

// ---------------------------------------------------------------------------
// LineSegmentation.cpp:291-333 — intersection_with_region  [BLOCKED]
//
// FIDELITY-NOTE(F1): geo/clipper2 backends cannot reproduce the legacy
// `ClipperLib_Z::Clipper` open-subject intersection with a four-endpoint
// `ZFillFunction` + `PolyTree` traversal that this function depends on.
//
// NOTE (BLOCKED): Requires the legacy `ClipperLib_Z::Clipper` engine:
//   - clipper.PreserveCollinear(true);
//   - clipper.ZFillFunction(<four-endpoint callback computing new_pt.z()>);
//   - clipper.AddPath(subject, ptSubject, /*closed=*/false);
//   - clipper.AddPaths(clips, ptClip, /*closed=*/true);
//   - clipper.Execute(ctIntersection, PolyTree, pftNonZero, pftNonZero);
//   - ClipperLib_Z::PolyTreeToPaths(std::move(clipped_polytree), intersections);
//
// The crate's clipper backend is Clipper2 (f64 / Centi scaling) whose Z-fill
// scheme (`Clipper2ZIntersectionVisitor`, negative-index `z`) is incompatible
// with the 30/31-bit `ZAttributes` provenance encoding used here, and exposes
// neither a `PolyTree` traversal nor the four-endpoint `ZFillFunction` callback.
// Byte-exactly reproducing this requires porting the bundled `clipper/clipper_z`
// engine, which is not yet available. NOT PORTED.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// LineSegmentation.cpp:335-395 — create_continues_line_region_ranges
// ---------------------------------------------------------------------------

// LineSegmentation.cpp:335
// LineRegionRanges create_continues_line_region_ranges(LineRegionRanges &&line_region_ranges, const size_t default_clip_idx, const size_t total_lines_cnt)
pub fn create_continues_line_region_ranges(
    mut line_region_ranges: LineRegionRanges,
    default_clip_idx: usize,
    total_lines_cnt: usize,
) -> LineRegionRanges {
    // LineSegmentation.cpp:337-339
    if line_region_ranges.is_empty() {
        return line_region_ranges;
    }

    // LineSegmentation.cpp:341 — std::sort(line_region_ranges.begin(), line_region_ranges.end());
    line_region_ranges.sort_by(|a, b| {
        if a.less(b) {
            std::cmp::Ordering::Less
        } else if b.less(a) {
            std::cmp::Ordering::Greater
        } else {
            std::cmp::Ordering::Equal
        }
    });

    // Resolve overlapping regions if it happens, but it should never happen.
    // LineSegmentation.cpp:344 — for (size_t region_range_idx = 1; region_range_idx < line_region_ranges.size(); ++region_range_idx) {
    for region_range_idx in 1..line_region_ranges.len() {
        // LineSegmentation.cpp:345 — LineRegionRange &prev_range = line_region_ranges[region_range_idx - 1];
        // LineSegmentation.cpp:346 — LineRegionRange &curr_range = line_region_ranges[region_range_idx];
        let prev_range = line_region_ranges[region_range_idx - 1];
        let curr_range = line_region_ranges[region_range_idx];

        // LineSegmentation.cpp:348 — assert(!prev_range.is_overlap(curr_range));
        debug_assert!(!prev_range.is_overlap(&curr_range));
        // LineSegmentation.cpp:349 — if (prev_range.is_inside(curr_range)) {
        if prev_range.is_inside(&curr_range) {
            // Make the previous range zero length to remove it later.
            // LineSegmentation.cpp:351-355
            // curr_range           = prev_range;
            // prev_range.begin_idx = curr_range.begin_idx;
            // prev_range.begin_t   = curr_range.begin_t;
            // prev_range.end_idx   = curr_range.begin_idx;
            // prev_range.end_t     = curr_range.begin_t;
            //
            // NOTE: after `curr_range = prev_range;`, all subsequent reads of
            // `curr_range` use the (now overwritten) value, i.e. the original
            // `prev_range`. Mirror that aliasing exactly.
            let new_curr = prev_range;
            line_region_ranges[region_range_idx] = new_curr;
            line_region_ranges[region_range_idx - 1].begin_idx = new_curr.begin_idx;
            line_region_ranges[region_range_idx - 1].begin_t = new_curr.begin_t;
            line_region_ranges[region_range_idx - 1].end_idx = new_curr.begin_idx;
            line_region_ranges[region_range_idx - 1].end_t = new_curr.begin_t;
        // LineSegmentation.cpp:356 — } else if (prev_range.is_overlap(curr_range)) {
        } else if prev_range.is_overlap(&curr_range) {
            // LineSegmentation.cpp:357 — curr_range.begin_idx = prev_range.end_idx;
            // LineSegmentation.cpp:358 — curr_range.begin_t   = prev_range.end_t;
            line_region_ranges[region_range_idx].begin_idx = prev_range.end_idx;
            line_region_ranges[region_range_idx].begin_t = prev_range.end_t;
        }
    }

    // Fill all gaps between regions with the default region.
    // LineSegmentation.cpp:363 — LineRegionRanges line_region_ranges_out;
    let mut line_region_ranges_out: LineRegionRanges = Vec::new();
    // LineSegmentation.cpp:364 — size_t prev_line_idx = 0.;
    let mut prev_line_idx: usize = 0;
    // LineSegmentation.cpp:365 — double prev_t = 0.;
    let mut prev_t: f64 = 0.;
    // LineSegmentation.cpp:366 — for (const LineRegionRange &curr_line_region : line_region_ranges) {
    for curr_line_region in &line_region_ranges {
        // LineSegmentation.cpp:367-369 — if (curr_line_region.is_zero_length()) { continue; }
        if curr_line_region.is_zero_length() {
            continue;
        }

        // LineSegmentation.cpp:371
        // assert(prev_line_idx < curr_line_region.begin_idx || (prev_line_idx == curr_line_region.begin_idx && prev_t <= curr_line_region.begin_t));
        debug_assert!(
            prev_line_idx < curr_line_region.begin_idx
                || (prev_line_idx == curr_line_region.begin_idx
                    && prev_t <= curr_line_region.begin_t)
        );

        // Fill the gap if it is necessary.
        // LineSegmentation.cpp:374-376
        // if (prev_line_idx != curr_line_region.begin_idx || prev_t != curr_line_region.begin_t) {
        //     line_region_ranges_out.emplace_back(prev_line_idx, prev_t, curr_line_region.begin_idx, curr_line_region.begin_t, default_clip_idx);
        // }
        if prev_line_idx != curr_line_region.begin_idx || prev_t != curr_line_region.begin_t {
            line_region_ranges_out.push(LineRegionRange::new(
                prev_line_idx,
                prev_t,
                curr_line_region.begin_idx,
                curr_line_region.begin_t,
                default_clip_idx,
            ));
        }

        // Add the current region.
        // LineSegmentation.cpp:379 — line_region_ranges_out.emplace_back(curr_line_region);
        line_region_ranges_out.push(*curr_line_region);
        // LineSegmentation.cpp:380 — prev_line_idx = curr_line_region.end_idx;
        prev_line_idx = curr_line_region.end_idx;
        // LineSegmentation.cpp:381 — prev_t = curr_line_region.end_t;
        prev_t = curr_line_region.end_t;
    }

    // Fill the last remaining gap if it exists.
    // LineSegmentation.cpp:385 — const size_t last_line_idx = total_lines_cnt - 1;
    let last_line_idx = total_lines_cnt - 1;
    // LineSegmentation.cpp:386-389
    // if ((prev_line_idx == last_line_idx && prev_t == 1.) || ((prev_line_idx == total_lines_cnt && prev_t == 0.))) {
    //     // There is no gap at the end.
    //     return line_region_ranges_out;
    // }
    if (prev_line_idx == last_line_idx && prev_t == 1.)
        || (prev_line_idx == total_lines_cnt && prev_t == 0.)
    {
        // There is no gap at the end.
        return line_region_ranges_out;
    }

    // Fill the last remaining gap.
    // LineSegmentation.cpp:392 — line_region_ranges_out.emplace_back(prev_line_idx, prev_t, last_line_idx, 1., default_clip_idx);
    line_region_ranges_out.push(LineRegionRange::new(
        prev_line_idx,
        prev_t,
        last_line_idx,
        1.,
        default_clip_idx,
    ));

    // LineSegmentation.cpp:394
    line_region_ranges_out
}

// ---------------------------------------------------------------------------
// LineSegmentation.cpp:397-407 — subject_segmentation  [BLOCKED]
//
// FIDELITY-NOTE(F1): transitively blocked — body calls `intersection_with_region`.
//
// NOTE (BLOCKED): The body calls `intersection_with_region` (BLOCKED, above)
// for each clip group, so it cannot be faithfully ported. NOT PORTED.
//
// LineRegionRanges subject_segmentation(const ClipperLib_Z::Path &subject, const std::vector<ExPolygons> &expolygons_clips, const size_t default_clip_idx = 0)
// {
//     LineRegionRanges line_region_ranges;
//     for (const ExPolygons &expolygons_clip : expolygons_clips) {
//         const size_t              expolygons_clip_idx = &expolygons_clip - expolygons_clips.data();
//         const ClipperLib_Z::Paths clips               = to_clip_zpaths(expolygons_clip);
//         Slic3r::append(line_region_ranges, intersection_with_region(subject, clips, expolygons_clip_idx + default_clip_idx + 1));
//     }
//     return create_continues_line_region_ranges(std::move(line_region_ranges), default_clip_idx, subject.size() - 1);
// }
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// LineSegmentation.cpp:409-436 — create_polyline_segment
// ---------------------------------------------------------------------------

// LineSegmentation.cpp:409
// PolylineSegment create_polyline_segment(const LineRegionRange &line_region_range, const Polyline &subject)
pub fn create_polyline_segment(
    line_region_range: &LineRegionRange,
    subject: &Polyline,
) -> PolylineSegment {
    // LineSegmentation.cpp:411 — Polyline polyline_out;
    let mut polyline_out = Polyline::new();
    // LineSegmentation.cpp:412-418
    // if (line_region_range.begin_t == 0.) {
    //     polyline_out.points.emplace_back(subject[line_region_range.begin_idx]);
    // } else {
    //     assert(line_region_range.begin_idx <= subject.size());
    //     Point interpolated_start_pt = lerp(subject[line_region_range.begin_idx], subject[line_region_range.begin_idx + 1], line_region_range.begin_t);
    //     polyline_out.points.emplace_back(interpolated_start_pt);
    // }
    if line_region_range.begin_t == 0. {
        polyline_out
            .points
            .push(subject.points[line_region_range.begin_idx]);
    } else {
        debug_assert!(line_region_range.begin_idx <= subject.points.len());
        let interpolated_start_pt = lerp(
            &subject.points[line_region_range.begin_idx],
            &subject.points[line_region_range.begin_idx + 1],
            line_region_range.begin_t,
        );
        polyline_out.points.push(interpolated_start_pt);
    }

    // LineSegmentation.cpp:420-422
    // for (size_t line_idx = line_region_range.begin_idx + 1; line_idx <= line_region_range.end_idx; ++line_idx) {
    //     polyline_out.points.emplace_back(subject[line_idx]);
    // }
    for line_idx in (line_region_range.begin_idx + 1)..=line_region_range.end_idx {
        polyline_out.points.push(subject.points[line_idx]);
    }

    // LineSegmentation.cpp:424-433
    // if (line_region_range.end_t == 0.) {
    //     polyline_out.points.emplace_back(subject[line_region_range.end_idx]);
    // } else if (line_region_range.end_t == 1.) {
    //     assert(line_region_range.end_idx <= subject.size());
    //     polyline_out.points.emplace_back(subject[line_region_range.end_idx + 1]);
    // } else {
    //     assert(line_region_range.end_idx <= subject.size());
    //     Point interpolated_end_pt = lerp(subject[line_region_range.end_idx], subject[line_region_range.end_idx + 1], line_region_range.end_t);
    //     polyline_out.points.emplace_back(interpolated_end_pt);
    // }
    if line_region_range.end_t == 0. {
        polyline_out
            .points
            .push(subject.points[line_region_range.end_idx]);
    } else if line_region_range.end_t == 1. {
        debug_assert!(line_region_range.end_idx <= subject.points.len());
        polyline_out
            .points
            .push(subject.points[line_region_range.end_idx + 1]);
    } else {
        debug_assert!(line_region_range.end_idx <= subject.points.len());
        let interpolated_end_pt = lerp(
            &subject.points[line_region_range.end_idx],
            &subject.points[line_region_range.end_idx + 1],
            line_region_range.end_t,
        );
        polyline_out.points.push(interpolated_end_pt);
    }

    // LineSegmentation.cpp:435 — return { polyline_out, line_region_range.clip_idx };
    PolylineSegment {
        polyline: polyline_out,
        clip_idx: line_region_range.clip_idx,
    }
}

// LineSegmentation.cpp:438-447
// PolylineSegments create_polyline_segments(const LineRegionRanges &line_region_ranges, const Polyline &subject)
pub fn create_polyline_segments(
    line_region_ranges: &LineRegionRanges,
    subject: &Polyline,
) -> PolylineSegments {
    // LineSegmentation.cpp:440 — PolylineSegments polyline_segments;
    let mut polyline_segments: PolylineSegments = Vec::new();
    // LineSegmentation.cpp:441 — polyline_segments.reserve(line_region_ranges.size());
    polyline_segments.reserve(line_region_ranges.len());
    // LineSegmentation.cpp:442-444
    // for (const LineRegionRange &region_range : line_region_ranges) {
    //     polyline_segments.emplace_back(create_polyline_segment(region_range, subject));
    // }
    for region_range in line_region_ranges {
        polyline_segments.push(create_polyline_segment(region_range, subject));
    }

    // LineSegmentation.cpp:446
    polyline_segments
}

// ---------------------------------------------------------------------------
// LineSegmentation.cpp:449-500 — create_extrusion_segment / create_extrusion_segments
// ---------------------------------------------------------------------------

// LineSegmentation.cpp:449
// ExtrusionSegment create_extrusion_segment(const LineRegionRange &line_region_range, const Arachne::ExtrusionLine &subject)
pub fn create_extrusion_segment(
    line_region_range: &LineRegionRange,
    subject: &ExtrusionLine,
) -> ExtrusionSegment {
    // When we call this function, we split ExtrusionLine into at least two segments, so none of those segments are closed.
    // LineSegmentation.cpp:452 — Arachne::ExtrusionLine extrusion_out(subject.inset_idx, subject.is_odd);
    let mut extrusion_out = ExtrusionLine::new(subject.inset_idx, subject.is_odd);
    // LineSegmentation.cpp:453-465
    // if (line_region_range.begin_t == 0.) {
    //     extrusion_out.junctions.emplace_back(subject[line_region_range.begin_idx]);
    // } else {
    //     assert(line_region_range.begin_idx <= subject.size());
    //     const Arachne::ExtrusionJunction &junction_from = subject[line_region_range.begin_idx];
    //     const Arachne::ExtrusionJunction &junction_to   = subject[line_region_range.begin_idx + 1];
    //     const Point   interpolated_start_pt = lerp(junction_from.p, junction_to.p, line_region_range.begin_t);
    //     const coord_t interpolated_start_w  = lerp(junction_from.w, junction_to.w, line_region_range.begin_t);
    //     assert(junction_from.perimeter_index == junction_to.perimeter_index);
    //     extrusion_out.junctions.emplace_back(interpolated_start_pt, interpolated_start_w, junction_from.perimeter_index);
    // }
    if line_region_range.begin_t == 0. {
        extrusion_out
            .junctions
            .push(subject.junctions[line_region_range.begin_idx]);
    } else {
        debug_assert!(line_region_range.begin_idx <= subject.size());
        let junction_from = &subject.junctions[line_region_range.begin_idx];
        let junction_to = &subject.junctions[line_region_range.begin_idx + 1];

        let interpolated_start_pt = lerp(&junction_from.p, &junction_to.p, line_region_range.begin_t);
        let interpolated_start_w = lerp_coord(junction_from.w, junction_to.w, line_region_range.begin_t);

        debug_assert!(junction_from.perimeter_index == junction_to.perimeter_index);
        extrusion_out.junctions.push(ExtrusionJunction::new(
            interpolated_start_pt,
            interpolated_start_w,
            junction_from.perimeter_index,
        ));
    }

    // LineSegmentation.cpp:467-469
    // for (size_t line_idx = line_region_range.begin_idx + 1; line_idx <= line_region_range.end_idx; ++line_idx) {
    //     extrusion_out.junctions.emplace_back(subject[line_idx]);
    // }
    for line_idx in (line_region_range.begin_idx + 1)..=line_region_range.end_idx {
        extrusion_out.junctions.push(subject.junctions[line_idx]);
    }

    // LineSegmentation.cpp:471-486
    // if (line_region_range.end_t == 0.) {
    //     extrusion_out.junctions.emplace_back(subject[line_region_range.end_idx]);
    // } else if (line_region_range.end_t == 1.) {
    //     assert(line_region_range.end_idx <= subject.size());
    //     extrusion_out.junctions.emplace_back(subject[line_region_range.end_idx + 1]);
    // } else {
    //     assert(line_region_range.end_idx <= subject.size());
    //     const Arachne::ExtrusionJunction &junction_from = subject[line_region_range.end_idx];
    //     const Arachne::ExtrusionJunction &junction_to   = subject[line_region_range.end_idx + 1];
    //     const Point   interpolated_end_pt = lerp(junction_from.p, junction_to.p, line_region_range.end_t);
    //     const coord_t interpolated_end_w  = lerp(junction_from.w, junction_to.w, line_region_range.end_t);
    //     assert(junction_from.perimeter_index == junction_to.perimeter_index);
    //     extrusion_out.junctions.emplace_back(interpolated_end_pt, interpolated_end_w, junction_from.perimeter_index);
    // }
    if line_region_range.end_t == 0. {
        extrusion_out
            .junctions
            .push(subject.junctions[line_region_range.end_idx]);
    } else if line_region_range.end_t == 1. {
        debug_assert!(line_region_range.end_idx <= subject.size());
        extrusion_out
            .junctions
            .push(subject.junctions[line_region_range.end_idx + 1]);
    } else {
        debug_assert!(line_region_range.end_idx <= subject.size());
        let junction_from = &subject.junctions[line_region_range.end_idx];
        let junction_to = &subject.junctions[line_region_range.end_idx + 1];

        let interpolated_end_pt = lerp(&junction_from.p, &junction_to.p, line_region_range.end_t);
        let interpolated_end_w = lerp_coord(junction_from.w, junction_to.w, line_region_range.end_t);

        debug_assert!(junction_from.perimeter_index == junction_to.perimeter_index);
        extrusion_out.junctions.push(ExtrusionJunction::new(
            interpolated_end_pt,
            interpolated_end_w,
            junction_from.perimeter_index,
        ));
    }

    // LineSegmentation.cpp:488 — return { extrusion_out, line_region_range.clip_idx };
    ExtrusionSegment {
        extrusion: extrusion_out,
        clip_idx: line_region_range.clip_idx,
    }
}

// LineSegmentation.cpp:491-500
// ExtrusionSegments create_extrusion_segments(const LineRegionRanges &line_region_ranges, const Arachne::ExtrusionLine &subject)
pub fn create_extrusion_segments(
    line_region_ranges: &LineRegionRanges,
    subject: &ExtrusionLine,
) -> ExtrusionSegments {
    // LineSegmentation.cpp:493 — ExtrusionSegments extrusion_segments;
    let mut extrusion_segments: ExtrusionSegments = Vec::new();
    // LineSegmentation.cpp:494 — extrusion_segments.reserve(line_region_ranges.size());
    extrusion_segments.reserve(line_region_ranges.len());
    // LineSegmentation.cpp:495-497
    // for (const LineRegionRange &region_range : line_region_ranges) {
    //     extrusion_segments.emplace_back(create_extrusion_segment(region_range, subject));
    // }
    for region_range in line_region_ranges {
        extrusion_segments.push(create_extrusion_segment(region_range, subject));
    }

    // LineSegmentation.cpp:499
    extrusion_segments
}

// libslic3r.h:281-285 — template<typename T, typename Number> constexpr inline T lerp(const T& a, const T& b, Number t)
// Used for coord_t width interpolation in create_extrusion_segment.
// return (Number(1) - t) * a + t * b;  (then implicitly converted back to coord_t)
#[inline]
fn lerp_coord(a: i64, b: i64, t: f64) -> i64 {
    debug_assert!(t >= -crate::libslic3r::EPSILON && t <= 1. + crate::libslic3r::EPSILON);
    ((1. - t) * a as f64 + t * b as f64) as i64
}

// ---------------------------------------------------------------------------
// LineSegmentation.cpp:502-581 — public *_segmentation entry points  [BLOCKED]
//
// FIDELITY-NOTE(F1): transitively blocked via `subject_segmentation` /
// `intersection_with_region` (legacy ClipperLib_Z engine). The
// `PerimeterRegions` overloads are additionally blocked by an incomplete
// `PerimeterRegion` (config-only view; see note below).
//
// NOTE (BLOCKED): Every public entry point routes through `subject_segmentation`
// (BLOCKED, above) which calls `intersection_with_region` (the legacy
// `ClipperLib_Z::Clipper` engine). They cannot be faithfully ported until that
// engine is available. The `PerimeterRegions`-based overloads are additionally
// blocked because the crate's `PerimeterRegion` (fuzzy_skin.rs) is a config-only
// view and does NOT carry `expolygons` nor a `region` pointer (C++
// PerimeterGenerator.hpp:15-20), so `to_expolygons_clips` and
// `perimeter_regions_clips[i].region->config()` cannot be reproduced. NOT PORTED.
//
// Affected symbols:
//   - intersection_with_region                                  (cpp:291-333)
//   - subject_segmentation                                      (cpp:397-407)
//   - create_polyline_segment / create_polyline_segments        (PORTED — pure)
//   - polyline_segmentation(Polyline, std::vector<ExPolygons>)  (cpp:502-512)
//   - polygon_segmentation(Polygon, std::vector<ExPolygons>)    (cpp:514-517)
//   - extrusion_segmentation(ExtrusionLine, std::vector<ExPolygons>) (cpp:519-529)
//   - to_expolygons_clips                                       (cpp:531-540)
//   - polyline_segmentation(Polyline, PrintRegionConfig, PerimeterRegions)  (cpp:542-558)
//   - polygon_segmentation(Polygon, PrintRegionConfig, PerimeterRegions)    (cpp:560-563)
//   - extrusion_segmentation(ExtrusionLine, PrintRegionConfig, PerimeterRegions) (cpp:565-581)
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // ZAttributes round-trips through encode/decode for the three flag combinations
    // exercised by the algorithm (subject points, clip points, new points).
    #[test]
    fn test_zattributes_encode_decode() {
        for (clip, new_, idx) in [
            (false, false, 0u32),
            (true, false, 0),
            (false, true, 7),
            (false, false, 0x3FFF_FFFE),
        ] {
            let z = ZAttributes::new(clip, new_, idx);
            let encoded = z.encode();
            let decoded = ZAttributes::decode(encoded);
            assert_eq!(decoded.is_clip_point, clip);
            assert_eq!(decoded.is_new_point, new_);
            assert_eq!(decoded.point_index, idx);

            // Constructor-from-coord path (cpp:31-32) must agree with decode (cpp:49-53).
            let from_coord = ZAttributes::from_clipper_coord(encoded);
            assert_eq!(from_coord.is_clip_point, clip);
            assert_eq!(from_coord.is_new_point, new_);
            assert_eq!(from_coord.point_index, idx);
        }
    }

    #[test]
    fn test_project_point_on_line_midpoint() {
        // Point projecting exactly onto the middle of a horizontal segment.
        let from = Point::new(0, 0);
        let to = Point::new(100, 0);
        let query = Point::new(50, 0);
        let info = project_point_on_line(&from, &to, &query);
        assert_eq!(info.projected_t, 0.5);
        assert_eq!(info.distance_sqr, 0.0);
    }

    #[test]
    fn test_project_point_on_line_degenerate() {
        // Zero-length line returns MAX/MAX (cpp:164-166).
        let from = Point::new(10, 10);
        let to = Point::new(10, 10);
        let query = Point::new(20, 20);
        let info = project_point_on_line(&from, &to, &query);
        assert_eq!(info.projected_t, f64::MAX);
        assert_eq!(info.distance_sqr, f64::MAX);
    }

    #[test]
    fn test_line_region_range_overlap_inside() {
        let a = LineRegionRange::new(0, 0.0, 2, 0.5, 1);
        let b = LineRegionRange::new(1, 0.0, 1, 0.5, 1);
        assert!(a.is_overlap(&b));
        assert!(a.is_inside(&b));
        let c = LineRegionRange::new(3, 0.0, 4, 0.0, 1);
        assert!(!a.is_overlap(&c));
    }

    #[test]
    fn test_subject_to_zpath_polyline() {
        let pl = Polyline::from_points(vec![
            Point::new(0, 0),
            Point::new(10, 0),
            Point::new(10, 10),
        ]);
        let zp = subject_to_zpath_polyline(&pl);
        assert_eq!(zp.len(), 3);
        // Each subject point is a non-clip, non-new point with increasing index.
        for (i, p) in zp.iter().enumerate() {
            let z = ZAttributes::from_clipper_coord(p.2 as u32);
            assert!(!z.is_clip_point);
            assert!(!z.is_new_point);
            assert_eq!(z.point_index as usize, i);
        }
    }
}
