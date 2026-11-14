//! Voronoi diagram utility functions.
//!
//! C++ Reference:
//! - Geometry/VoronoiUtils.hpp
//! - Geometry/VoronoiUtils.cpp
//!
//! Provides utility functions for working with Voronoi diagrams in the context
//! of the Arachne variable-width algorithm.
//!
//! `get_source_segment` (VoronoiUtils.cpp:40), `get_source_point` (VoronoiUtils.cpp:56)
//! and `compute_segment_cell_range` (VoronoiUtils.cpp:205) are at the end of this file,
//! written over `VdAccess`, which both diagrams of this crate implement (the
//! `boostvoronoi` crate's and `boost_polygon_voronoi`'s). `get_source_point_index`
//! (VoronoiUtils.cpp:83) only exists for Arachne's segment type and is in
//! `arachne/skeletal_trapezoidation.rs`. `voronoi_utils_cgal.rs` keeps its own copies
//! over the `boostvoronoi` diagram for `get_parabolic_segment`.

use crate::geometry::{perp, Point};
use crate::Coord;

/// Represents the range of edges around a trapezoid-shaped Voronoi cell
/// that belongs to a line segment source.
///
/// Geometry/VoronoiUtils.hpp: SegmentCellRange
#[derive(Debug, Clone)]
pub struct SegmentCellRange {
    /// The start point of the source segment of this cell.
    pub segment_start_point: Point,
    /// The end point of the source segment of this cell.
    pub segment_end_point: Point,
    /// Index of the edge where the loop around the cell starts (None if invalid).
    pub edge_begin: Option<usize>,
    /// Index of the edge where the loop around the cell ends (None if invalid).
    pub edge_end: Option<usize>,
}

impl SegmentCellRange {
    /// Create a new SegmentCellRange for the given segment endpoints.
    pub fn new(segment_start_point: Point, segment_end_point: Point) -> Self {
        Self {
            segment_start_point,
            segment_end_point,
            edge_begin: None,
            edge_end: None,
        }
    }

    /// Check if the cell range is valid (both edges set and different).
    pub fn is_valid(&self) -> bool {
        match (self.edge_begin, self.edge_end) {
            (Some(begin), Some(end)) => begin != end,
            _ => false,
        }
    }
}

/// Utility functions for working with Voronoi diagrams.
///
/// Geometry/VoronoiUtils.hpp: VoronoiUtils
pub struct VoronoiUtils;

impl VoronoiUtils {
    /// Convert a Voronoi vertex to an integer Point by rounding coordinates.
    ///
    /// VoronoiUtils.cpp:255 `VoronoiUtils::to_point(const VD::vertex_type &vertex)`.
    pub fn to_point(x: f64, y: f64) -> Point {
        // VoronoiUtils.cpp:259 assert(std::isfinite(x) && std::isfinite(y));
        debug_assert!(x.is_finite() && y.is_finite());
        // VoronoiUtils.cpp:262 return {std::llround(x), std::llround(y)};
        // f64::round() rounds half away from zero, matching std::llround.
        Point::new(x.round() as i64, y.round() as i64)
    }

    /// Check if a Voronoi vertex has finite coordinates.
    ///
    /// VoronoiUtils.cpp:265 `VoronoiUtils::is_finite`.
    pub fn is_finite(x: f64, y: f64) -> bool {
        // VoronoiUtils.cpp:267 return std::isfinite(vertex.x()) && std::isfinite(vertex.y());
        x.is_finite() && y.is_finite()
    }

    /// Create a rotated copy of a vertex position.
    ///
    /// VoronoiUtils.cpp:270 `VoronoiUtils::make_rotated_vertex`.
    /// Returns the rotated `(x, y)`; the C++ also copies `incident_edge`/`color`,
    /// which are boost.polygon vertex fields without a coordinate-level analogue.
    pub fn make_rotated_vertex(x: f64, y: f64, angle: f64) -> (f64, f64) {
        // VoronoiUtils.cpp:272-273
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        // Each is one source expression, which the reference build (Apple clang arm64,
        // -ffp-contract=on) contracts: the left product is fused, the right one is rounded
        // first. Nothing called this function before the Voronoi repair below did, so no
        // earlier output depends on the unfused form it had.
        // VoronoiUtils.cpp:275 const double rotated_x = (cos_a * vertex.x() - sin_a * vertex.y());
        let rotated_x = cos_a.mul_add(x, -(sin_a * y));
        // VoronoiUtils.cpp:276 const double rotated_y = (cos_a * vertex.y() + sin_a * vertex.x());
        let rotated_y = cos_a.mul_add(y, sin_a * x);
        (rotated_x, rotated_y)
    }
}

/// Convert Voronoi vertex coordinates to a Point by rounding.
///
/// Geometry/VoronoiUtils.hpp: to_point (free function form)
pub fn to_point(x: f64, y: f64) -> Point {
    VoronoiUtils::to_point(x, y)
}

/// Create a rotated vertex from the given coordinates and angle.
///
/// Geometry/VoronoiUtils.hpp: make_rotated_vertex (free function form)
pub fn make_rotated_vertex(x: f64, y: f64, angle: f64) -> (f64, f64) {
    VoronoiUtils::make_rotated_vertex(x, y, angle)
}

/// Check if coordinates are finite.
///
/// Geometry/VoronoiUtils.hpp: is_finite (free function form)
pub fn is_finite(x: f64, y: f64) -> bool {
    VoronoiUtils::is_finite(x, y)
}

/// Discretize the parabolic Voronoi edge equidistant from a point-site `source_point`
/// and a segment-site `seg_a`→`seg_b`, between `start` and `end`, into a polyline.
/// VoronoiUtils.cpp:107 `VoronoiUtils::discretize_parabola`.
pub fn discretize_parabola(
    source_point: Point,
    seg_a: Point,
    seg_b: Point,
    start: Point,
    end: Point,
    approximate_step_size: Coord,
    transitioning_angle: f32,
) -> Vec<Point> {
    // VoronoiUtils.cpp:109 Points discretized;
    let mut discretized: Vec<Point> = Vec::new();
    // VoronoiUtils.cpp:110-111
    // x is distance of point projected on the segment ab
    // xx is point projected on the segment ab
    // VoronoiUtils.cpp:112-113 const Point a = source_segment.from(); const Point b = source_segment.to();
    let a = seg_a;
    let b = seg_b;
    // VoronoiUtils.cpp:114-116
    let ab = b - a;
    let as_ = start - a;
    let ae = end - a;

    // C++ casts the integer Point vectors to int64_t and uses Eigen .dot()/.norm().
    // Point::dot already returns i128 (no overflow), and .norm() is the truncated
    // integer sqrt of the squared length (Eigen casts the double sqrt back to the
    // scalar type, truncating toward zero).
    let isqrt = |p: Point| -> i64 {
        ((p.x as i128 * p.x as i128 + p.y as i128 * p.y as i128) as f64).sqrt() as i64
    };
    // VoronoiUtils.cpp:117 const coord_t ab_size = ab.cast<int64_t>().norm();
    // FIDELITY-NOTE(F2): coord_t is int32 in C++; the crate's Coord is i64, so this
    // keeps the i64 width rather than truncating to int32 (crate-wide decision).
    let ab_size = isqrt(ab);
    // VoronoiUtils.cpp:118 const coord_t sx = as.cast<int64_t>().dot(ab.cast<int64_t>()) / ab_size;
    let sx = (as_.dot(&ab) / ab_size as i128) as i64;
    // VoronoiUtils.cpp:119 const coord_t ex = ae.cast<int64_t>().dot(ab.cast<int64_t>()) / ab_size;
    let ex = (ae.dot(&ab) / ab_size as i128) as i64;
    // VoronoiUtils.cpp:120 const coord_t sxex = ex - sx;
    let sxex = ex - sx;

    // VoronoiUtils.cpp:122 const Point ap = source_point - a;
    let ap = source_point - a;
    // VoronoiUtils.cpp:123 const coord_t px = ap.cast<int64_t>().dot(ab.cast<int64_t>()) / ab_size;
    let px = (ap.dot(&ab) / ab_size as i128) as i64;

    // VoronoiUtils.cpp:125-126
    // Point pxx; Line(a, b).distance_to_infinite_squared(source_point, &pxx);
    // The closest point on the infinite line a-b is a + t*(b-a) with t = (ap.v)/|v|^2,
    // cast back to integer by truncation toward zero (Eigen .cast<coord_t>()), NOT
    // rounding. Computed inline here to match C++ exactly (project_point_infinite
    // rounds instead, so it is not equivalent).
    let pxx = {
        let v = ab; // (b - a)
        let l2 = (v.x as i128 * v.x as i128 + v.y as i128 * v.y as i128) as f64;
        if l2 == 0.0 {
            a // a == b case: closest point is a (Line.hpp:95)
        } else {
            let t = ap.dot(&v) as f64 / l2;
            Point::new(
                (a.x as f64 + t * v.x as f64) as Coord,
                (a.y as f64 + t * v.y as f64) as Coord,
            )
        }
    };
    // VoronoiUtils.cpp:127-128 const Point ppxx = pxx - source_point; const coord_t d = ppxx.cast<int64_t>().norm();
    let ppxx = pxx - source_point;
    let d = isqrt(ppxx);

    // VoronoiUtils.cpp:130-132 const Vec2d rot = perp(ppxx).cast<double>().normalized();
    // perp(p) = (-p.y, p.x) (Point.hpp:99).
    let perp_ppxx = perp(ppxx);
    let perp_x = perp_ppxx.x as f64;
    let perp_y = perp_ppxx.y as f64;
    // FIDELITY-NOTE: Eigen's `normalized()` divides by `sqrt(x*x + y*y)`; `hypot` can
    // differ from that in the last place. `Point::rotated(cos, sin)` (Point.hpp:226-231,
    // used four times below through `rotate_by_cos_sin`) is `round(cos_a * cur_x - sin_a *
    // cur_y)` and `round(cos_a * cur_y + sin_a * cur_x)`, which the reference build
    // contracts (left product fused); `rotate_by_cos_sin` does not fuse. Both were tried
    // as the C++ has them when Arachne moved onto the port of boost's builder: the G-code
    // of the three fixtures did not change by a byte, so the arithmetic that other
    // callers share is left as it is.
    let rot_len = perp_x.hypot(perp_y);
    // VoronoiUtils.cpp:131-132 const double rot_cos_theta = rot.x(); const double rot_sin_theta = rot.y();
    let rot_cos_theta = perp_x / rot_len;
    let rot_sin_theta = perp_y / rot_len;

    // VoronoiUtils.cpp:134-138
    if d == 0 {
        discretized.push(start);
        discretized.push(end);
        return discretized;
    }

    // VoronoiUtils.cpp:140 const double marking_bound = atan(transitioning_angle * 0.5);
    let marking_bound = (transitioning_angle as f64 * 0.5).atan();
    // VoronoiUtils.cpp:141 int64_t msx = -marking_bound * int64_t(d); // projected marking_start
    let mut msx = (-marking_bound * d as f64) as i64;
    // VoronoiUtils.cpp:142 int64_t mex = marking_bound * int64_t(d);  // projected marking_end
    let mut mex = (marking_bound * d as f64) as i64;

    // VoronoiUtils.cpp:144 const coord_t marking_start_end_h = msx * msx / (2 * d) + d / 2;
    let marking_start_end_h = msx * msx / (2 * d) + d / 2;
    // VoronoiUtils.cpp:145 Point marking_start = Point(coord_t(msx), marking_start_end_h).rotated(rot_cos_theta, rot_sin_theta) + pxx;
    let mut marking_start = Point::new(msx as Coord, marking_start_end_h as Coord)
        .rotate_by_cos_sin(rot_cos_theta, rot_sin_theta)
        + pxx;
    // VoronoiUtils.cpp:146 Point marking_end = Point(coord_t(mex), marking_start_end_h).rotated(rot_cos_theta, rot_sin_theta) + pxx;
    let mut marking_end = Point::new(mex as Coord, marking_start_end_h as Coord)
        .rotate_by_cos_sin(rot_cos_theta, rot_sin_theta)
        + pxx;
    // VoronoiUtils.cpp:147 const int dir = (sx > ex) ? -1 : 1;
    let dir: i64 = if sx > ex { -1 } else { 1 };
    // VoronoiUtils.cpp:148-151
    if dir < 0 {
        std::mem::swap(&mut marking_start, &mut marking_end);
        std::mem::swap(&mut msx, &mut mex);
    }

    // VoronoiUtils.cpp:153
    let mut add_marking_start = msx * dir > (sx - px) * dir && msx * dir < (ex - px) * dir;
    // VoronoiUtils.cpp:154
    let mut add_marking_end = mex * dir > (sx - px) * dir && mex * dir < (ex - px) * dir;

    // VoronoiUtils.cpp:156 const Point apex = Point(0, d / 2).rotated(rot_cos_theta, rot_sin_theta) + pxx;
    let apex = Point::new(0, (d / 2) as Coord)
        .rotate_by_cos_sin(rot_cos_theta, rot_sin_theta)
        + pxx;
    // VoronoiUtils.cpp:157 bool add_apex = int64_t(sx - px) * int64_t(dir) < 0 && int64_t(ex - px) * int64_t(dir) > 0;
    let mut add_apex = (sx - px) * dir < 0 && (ex - px) * dir > 0;

    // VoronoiUtils.cpp:159-161 assert + warning when discretization cannot place an apex/endpoint.
    debug_assert!(!add_marking_start || !add_marking_end || add_apex);
    if add_marking_start && add_marking_end && !add_apex {
        log::warn!("Failing to discretize parabola! Must add an apex or one of the endpoints.");
    }

    // VoronoiUtils.cpp:163 const coord_t step_count = lround(std::abs(ex - sx) / approximate_step_size);
    // lround rounds half away from zero, matching f64::round().
    let step_count = ((ex - sx).abs() as f64 / approximate_step_size as f64).round() as i64;
    // VoronoiUtils.cpp:164 discretized.emplace_back(start);
    discretized.push(start);
    // VoronoiUtils.cpp:165 for (coord_t step = 1; step < step_count; ++step)
    let mut step: i64 = 1;
    while step < step_count {
        // VoronoiUtils.cpp:166 const int64_t x = int64_t(sx) + int64_t(sxex) * int64_t(step) / int64_t(step_count) - int64_t(px);
        let x = sx + sxex * step / step_count - px;
        // VoronoiUtils.cpp:167 const int64_t y = int64_t(x) * int64_t(x) / int64_t(2 * d) + int64_t(d / 2);
        let y = x * x / (2 * d) + d / 2;

        // VoronoiUtils.cpp:169-172
        if add_marking_start && msx * dir < x * dir {
            discretized.push(marking_start);
            add_marking_start = false;
        }
        // VoronoiUtils.cpp:174-177
        if add_apex && x * dir > 0 {
            discretized.push(apex);
            add_apex = false; // only add the apex just before the
        }
        // VoronoiUtils.cpp:179-182
        if add_marking_end && mex * dir < x * dir {
            discretized.push(marking_end);
            add_marking_end = false;
        }
        // VoronoiUtils.cpp:185-186 const Point result = Point(x, y).rotated(...) + pxx; discretized.emplace_back(result);
        let result = Point::new(x as Coord, y as Coord)
            .rotate_by_cos_sin(rot_cos_theta, rot_sin_theta)
            + pxx;
        discretized.push(result);
        step += 1;
    }

    // VoronoiUtils.cpp:189-190
    if add_apex {
        discretized.push(apex);
    }
    // VoronoiUtils.cpp:192-193
    if add_marking_end {
        discretized.push(marking_end);
    }
    // VoronoiUtils.cpp:195-196 discretized.emplace_back(end); return discretized;
    discretized.push(end);
    discretized
}

// ---------------------------------------------------------------------------
// The diagram the functions below walk.
//
// The C++ walks `Geometry::VoronoiDiagram` through `VD::cell_type` / `VD::edge_type` /
// `VD::vertex_type` pointers. Two diagrams stand behind that in this crate: the
// `boostvoronoi` crate's (`bv::Diagram`, typed indices), which the engine used for every
// diagram until R900, and `boost_polygon_voronoi::VoronoiDiagram`, the port of
// boost::polygon's own builder (plain indices in boost's container order). `VdAccess`
// is the part of the boost interface the walks need, with a cell, an edge and a vertex
// named by its index in the diagram's container, so one walk serves both.
// ---------------------------------------------------------------------------

use crate::boost_polygon_voronoi as bpv;
use boostvoronoi::prelude as bv;

/// `boost::polygon::voronoi_diagram<double>` as the walks read it. A cell, an edge and a
/// vertex are indices into `cells()`, `edges()` and `vertices()`.
pub trait VdAccess {
    /// voronoi_diagram.hpp:319-321 `num_cells()`.
    fn num_cells(&self) -> usize;
    /// voronoi_diagram.hpp:323-325 `num_edges()`.
    fn num_edges(&self) -> usize;
    /// voronoi_diagram.hpp:327-329 `num_vertices()`.
    fn num_vertices(&self) -> usize;
    /// voronoi_diagram.hpp:72-73 `voronoi_cell::incident_edge()`, null for a degenerate cell.
    fn cell_incident_edge(&self, cell: usize) -> Option<usize>;
    /// voronoi_diagram.hpp:50-53 `voronoi_cell::contains_point()`.
    fn cell_contains_point(&self, cell: usize) -> bool;
    /// voronoi_diagram.hpp:56-59 `voronoi_cell::contains_segment()`.
    fn cell_contains_segment(&self, cell: usize) -> bool;
    /// voronoi_diagram.hpp:61-63 `voronoi_cell::source_index()`.
    fn cell_source_index(&self, cell: usize) -> usize;
    /// voronoi_diagram.hpp:65-67 `voronoi_cell::source_category()`, as one of boost's
    /// `SOURCE_CATEGORY_*` values (`boost_polygon_voronoi`).
    fn cell_source_category(&self, cell: usize) -> usize;
    /// voronoi_diagram.hpp:171-172 `voronoi_edge::cell()`.
    fn edge_cell(&self, edge: usize) -> usize;
    /// voronoi_diagram.hpp:182-183 `voronoi_edge::twin()`.
    fn edge_twin(&self, edge: usize) -> usize;
    /// voronoi_diagram.hpp:186-187 `voronoi_edge::next()`.
    fn edge_next(&self, edge: usize) -> usize;
    /// voronoi_diagram.hpp:175-176 `voronoi_edge::vertex0()`, null at infinity.
    fn edge_vertex0(&self, edge: usize) -> Option<usize>;
    /// voronoi_diagram.hpp:179-180 `voronoi_edge::vertex1()`, null at infinity.
    fn edge_vertex1(&self, edge: usize) -> Option<usize>;
    /// voronoi_diagram.hpp:206 `voronoi_edge::is_finite()`.
    fn edge_is_finite(&self, edge: usize) -> bool;
    /// voronoi_diagram.hpp:210 `voronoi_edge::is_infinite()`.
    fn edge_is_infinite(&self, edge: usize) -> bool;
    /// voronoi_diagram.hpp:232-234 `voronoi_edge::is_secondary()`.
    fn edge_is_secondary(&self, edge: usize) -> bool;
    /// voronoi_diagram.hpp:112 `voronoi_vertex::x()`.
    fn vertex_x(&self, vertex: usize) -> f64;
    /// voronoi_diagram.hpp:113 `voronoi_vertex::y()`.
    fn vertex_y(&self, vertex: usize) -> f64;
}

/// The `boostvoronoi` crate's diagram. Every call is the one the walks made on it before
/// they were written over `VdAccess`, so what they compute on it is unchanged.
impl VdAccess for bv::Diagram {
    fn num_cells(&self) -> usize {
        self.cells().len()
    }
    fn num_edges(&self) -> usize {
        self.edges().len()
    }
    fn num_vertices(&self) -> usize {
        self.vertices().len()
    }
    fn cell_incident_edge(&self, cell: usize) -> Option<usize> {
        self.cells()[cell].get_incident_edge().map(|e| e.usize())
    }
    fn cell_contains_point(&self, cell: usize) -> bool {
        self.cells()[cell].contains_point()
    }
    fn cell_contains_segment(&self, cell: usize) -> bool {
        self.cells()[cell].contains_segment()
    }
    fn cell_source_index(&self, cell: usize) -> usize {
        self.cells()[cell].source_index().usize()
    }
    fn cell_source_category(&self, cell: usize) -> usize {
        match self.cells()[cell].source_category() {
            bv::SourceCategory::SinglePoint => bpv::SOURCE_CATEGORY_SINGLE_POINT,
            bv::SourceCategory::SegmentStart => bpv::SOURCE_CATEGORY_SEGMENT_START_POINT,
            bv::SourceCategory::SegmentEnd => bpv::SOURCE_CATEGORY_SEGMENT_END_POINT,
            // The crate folds INITIAL_SEGMENT and REVERSE_SEGMENT into one category; the
            // walks only tell a point cell's two kinds apart.
            bv::SourceCategory::Segment => bpv::SOURCE_CATEGORY_INITIAL_SEGMENT,
        }
    }
    fn edge_cell(&self, edge: usize) -> usize {
        self.edges()[edge].cell().unwrap().usize()
    }
    fn edge_twin(&self, edge: usize) -> usize {
        self.edges()[edge].twin().unwrap().usize()
    }
    fn edge_next(&self, edge: usize) -> usize {
        self.edges()[edge].next().unwrap().usize()
    }
    fn edge_vertex0(&self, edge: usize) -> Option<usize> {
        self.edges()[edge].vertex0().map(|v| v.usize())
    }
    fn edge_vertex1(&self, edge: usize) -> Option<usize> {
        self.edge_get_vertex1(self.edge_index_unchecked(edge)).ok().flatten().map(|v| v.usize())
    }
    fn edge_is_finite(&self, edge: usize) -> bool {
        !VdAccess::edge_is_infinite(self, edge)
    }
    fn edge_is_infinite(&self, edge: usize) -> bool {
        bv::Diagram::edge_is_infinite(self, self.edge_index_unchecked(edge)).unwrap_or(true)
    }
    fn edge_is_secondary(&self, edge: usize) -> bool {
        self.edges()[edge].is_secondary()
    }
    fn vertex_x(&self, vertex: usize) -> f64 {
        self.vertices()[vertex].x()
    }
    fn vertex_y(&self, vertex: usize) -> f64 {
        self.vertices()[vertex].y()
    }
}

/// The port of boost::polygon's diagram.
impl VdAccess for bpv::VoronoiDiagram {
    fn num_cells(&self) -> usize {
        bpv::VoronoiDiagram::num_cells(self)
    }
    fn num_edges(&self) -> usize {
        bpv::VoronoiDiagram::num_edges(self)
    }
    fn num_vertices(&self) -> usize {
        bpv::VoronoiDiagram::num_vertices(self)
    }
    fn cell_incident_edge(&self, cell: usize) -> Option<usize> {
        self.cells()[cell].incident_edge()
    }
    fn cell_contains_point(&self, cell: usize) -> bool {
        self.cells()[cell].contains_point()
    }
    fn cell_contains_segment(&self, cell: usize) -> bool {
        self.cells()[cell].contains_segment()
    }
    fn cell_source_index(&self, cell: usize) -> usize {
        self.cells()[cell].source_index()
    }
    fn cell_source_category(&self, cell: usize) -> usize {
        self.cells()[cell].source_category()
    }
    fn edge_cell(&self, edge: usize) -> usize {
        self.edges()[edge].cell().expect("voronoi_edge::cell: null")
    }
    fn edge_twin(&self, edge: usize) -> usize {
        self.edges()[edge].twin().expect("voronoi_edge::twin: null")
    }
    fn edge_next(&self, edge: usize) -> usize {
        self.edges()[edge].next().expect("voronoi_edge::next: null")
    }
    fn edge_vertex0(&self, edge: usize) -> Option<usize> {
        self.edges()[edge].vertex0()
    }
    fn edge_vertex1(&self, edge: usize) -> Option<usize> {
        self.vertex1(edge)
    }
    fn edge_is_finite(&self, edge: usize) -> bool {
        self.is_finite(edge)
    }
    fn edge_is_infinite(&self, edge: usize) -> bool {
        self.is_infinite(edge)
    }
    fn edge_is_secondary(&self, edge: usize) -> bool {
        self.edges()[edge].is_secondary()
    }
    fn vertex_x(&self, vertex: usize) -> f64 {
        self.vertices()[vertex].x()
    }
    fn vertex_y(&self, vertex: usize) -> f64 {
        self.vertices()[vertex].y()
    }
}

/// The rest of the interface, for the walks that annotate the diagram: a vertex's incident
/// edge, and the `color` member of vertices and edges, which boost declares `mutable` so
/// that a `const` diagram can be annotated (voronoi_diagram.hpp:121-125, 236-240).
pub trait VdColors: VdAccess {
    /// What `color()` returns after `color(-1)` (`color_type` is `std::size_t`): the mark
    /// of a vertex nothing was assigned to. boost keeps the colour above 5 bits of flags.
    fn color_unset() -> usize;
    /// voronoi_diagram.hpp:117-118 `voronoi_vertex::incident_edge()`.
    fn vertex_incident_edge(&self, vertex: usize) -> Option<usize>;
    /// voronoi_diagram.hpp:121 `voronoi_vertex::color()`.
    fn vertex_color(&self, vertex: usize) -> usize;
    /// voronoi_diagram.hpp:122-125 `voronoi_vertex::color(color_type)`.
    fn set_vertex_color(&mut self, vertex: usize, color: usize);
    /// voronoi_diagram.hpp:236 `voronoi_edge::color()`.
    fn edge_color(&self, edge: usize) -> usize;
    /// voronoi_diagram.hpp:237-240 `voronoi_edge::color(color_type)`.
    fn set_edge_color(&mut self, edge: usize, color: usize);
}

/// The `boostvoronoi` crate keeps the colour in the upper 27 bits of a `u32`, so `color(-1)`
/// reads back as `(1 << 27) - 1`. Every call is the one the walks made on it before they
/// were written over `VdColors`.
impl VdColors for bv::Diagram {
    fn color_unset() -> usize {
        (1usize << 27) - 1
    }
    fn vertex_incident_edge(&self, vertex: usize) -> Option<usize> {
        self.vertex_get_incident_edge(self.vertices()[vertex].get_id()).map(|e| e.usize())
    }
    fn vertex_color(&self, vertex: usize) -> usize {
        self.vertex_get_color(self.vertices()[vertex].get_id()).map_or(Self::color_unset(), |c| c as usize)
    }
    fn set_vertex_color(&mut self, vertex: usize, color: usize) {
        let id = self.vertices()[vertex].get_id();
        let _ = self.vertex_set_color(id, color as u32);
    }
    fn edge_color(&self, edge: usize) -> usize {
        self.edge_get_color(self.edge_index_unchecked(edge)).unwrap_or(0) as usize
    }
    fn set_edge_color(&mut self, edge: usize, color: usize) {
        let id = self.edge_index_unchecked(edge);
        let _ = bv::Diagram::edge_set_color(self, id, color as u32);
    }
}

/// The port of boost::polygon's diagram: `color_ >> BITS_SHIFT` of a `std::size_t`.
impl VdColors for bpv::VoronoiDiagram {
    fn color_unset() -> usize {
        usize::MAX >> 5
    }
    fn vertex_incident_edge(&self, vertex: usize) -> Option<usize> {
        self.vertices()[vertex].incident_edge()
    }
    fn vertex_color(&self, vertex: usize) -> usize {
        self.vertices()[vertex].color()
    }
    fn set_vertex_color(&mut self, vertex: usize, color: usize) {
        bpv::VoronoiDiagram::set_vertex_color(self, vertex, color);
    }
    fn edge_color(&self, edge: usize) -> usize {
        self.edges()[edge].color()
    }
    fn set_edge_color(&mut self, edge: usize, color: usize) {
        bpv::VoronoiDiagram::set_edge_color(self, edge, color);
    }
}

/// `boost::polygon::segment_traits<Segment>::get(segment, LOW | HIGH)` for the segment
/// types the diagrams are built from: `Line` (Line.hpp:258-263), Arachne's
/// `PolygonsSegmentIndex` (PolygonsSegmentIndex.hpp:39-50), `ColoredLine`
/// (MultiMaterialSegmentation.hpp:38-46) and `VoronoiDiagram::Segment` (Voronoi.hpp:191-198).
pub trait VoronoiSegment {
    /// `get(segment, boost::polygon::LOW)`.
    fn low(&self) -> Point;
    /// `get(segment, boost::polygon::HIGH)`.
    fn high(&self) -> Point;
}

impl VoronoiSegment for crate::geometry::Line {
    fn low(&self) -> Point {
        self.a
    }
    fn high(&self) -> Point {
        self.b
    }
}

impl VoronoiSegment for crate::arachne::utils::polygons_segment_index::PolygonsSegmentIndex<'_> {
    fn low(&self) -> Point {
        self.segment_get(crate::arachne::utils::polygons_segment_index::Direction1d::Low)
    }
    fn high(&self) -> Point {
        self.segment_get(crate::arachne::utils::polygons_segment_index::Direction1d::High)
    }
}

/// VoronoiUtils.cpp:34-49 `VoronoiUtils::get_source_segment(cell, segment_begin, segment_end)`.
pub fn get_source_segment<'s, D: VdAccess + ?Sized, S>(vd: &D, cell: usize, segments: &'s [S]) -> &'s S {
    // VoronoiUtils.cpp:42-43 if (!cell.contains_segment()) throw InvalidArgument
    assert!(vd.cell_contains_segment(cell), "Voronoi cell doesn't contain a source segment!");
    // VoronoiUtils.cpp:45-46 if (cell.source_index() >= distance(begin, end)) throw OutOfRange
    let source_index = vd.cell_source_index(cell);
    assert!(source_index < segments.len(), "Voronoi cell source index is out of range!");
    // VoronoiUtils.cpp:48 return *(segment_begin + cell.source_index());
    &segments[source_index]
}

/// VoronoiUtils.cpp:51-76 `VoronoiUtils::get_source_point(cell, segment_begin, segment_end)`.
pub fn get_source_point<D: VdAccess + ?Sized, S: VoronoiSegment>(vd: &D, cell: usize, segments: &[S]) -> Point {
    // VoronoiUtils.cpp:60-61 if (!cell.contains_point()) throw InvalidArgument
    assert!(vd.cell_contains_point(cell), "Voronoi cell doesn't contain a source point!");
    let source_index = vd.cell_source_index(cell);
    let source_category = vd.cell_source_category(cell);
    if source_category == bpv::SOURCE_CATEGORY_SEGMENT_START_POINT {
        // VoronoiUtils.cpp:63-66 the segment's LOW point
        debug_assert!(source_index < segments.len());
        segments[source_index].low()
    } else if source_category == bpv::SOURCE_CATEGORY_SEGMENT_END_POINT {
        // VoronoiUtils.cpp:67-70 the segment's HIGH point
        debug_assert!(source_index < segments.len());
        segments[source_index].high()
    } else if source_category == bpv::SOURCE_CATEGORY_SINGLE_POINT {
        // VoronoiUtils.cpp:71-72
        panic!("Voronoi diagram is always constructed using segments, so cell.source_category() shouldn't be SOURCE_CATEGORY_SINGLE_POINT!");
    } else {
        // VoronoiUtils.cpp:73-74
        panic!("Function get_source_point() should only be called on point cells!");
    }
}

/// VoronoiUtils.cpp:198-243 `VoronoiUtils::compute_segment_cell_range(cell, segment_begin, segment_end)`.
/// `edge_begin` and `edge_end` are indices into the diagram's edges.
pub fn compute_segment_cell_range<D: VdAccess + ?Sized, S: VoronoiSegment>(
    vd: &D,
    cell: usize,
    segments: &[S],
) -> SegmentCellRange {
    // VoronoiUtils.cpp:211 const Segment &source_segment = get_source_segment(cell, ...);
    let source_segment = get_source_segment(vd, cell, segments);
    // VoronoiUtils.cpp:212 const Point from = segment_traits::get(source_segment, LOW);
    let from = source_segment.low();
    // VoronoiUtils.cpp:213 const Point to = segment_traits::get(source_segment, HIGH);
    let to = source_segment.high();
    // VoronoiUtils.cpp:214-215 const Vec2i64 from_i64 = from.cast<int64_t>(); to_i64 = to.cast<int64_t>();
    let from_i64 = (from.x, from.y);
    let to_i64 = (to.x, to.y);

    // VoronoiUtils.cpp:218 SegmentCellRange cell_range(to, from);
    let mut cell_range = SegmentCellRange::new(to, from);

    // VoronoiUtils.cpp:221-223 Find starting edge and end edge
    let mut seen_possible_start = false;
    let mut after_start = false;
    let mut ending_edge_is_set_before_start = false;
    // VoronoiUtils.cpp:224 const VD::edge_type *edge = cell.incident_edge();
    let incident_edge = vd.cell_incident_edge(cell).unwrap();
    let mut edge = incident_edge;
    // VoronoiUtils.cpp:225-241 do { ... } while (edge = edge->next(), edge != cell.incident_edge());
    loop {
        // VoronoiUtils.cpp:226-227 if (edge->is_infinite()) continue;
        if vd.edge_is_infinite(edge) {
            edge = vd.edge_next(edge);
            if edge == incident_edge {
                break;
            }
            continue;
        }

        // VoronoiUtils.cpp:229-230 Vec2i64 v0 = to_point(edge->vertex0()); v1 = to_point(edge->vertex1());
        let v0i = vd.edge_vertex0(edge).unwrap();
        let v1i = vd.edge_vertex1(edge).unwrap();
        let v0 = (vd.vertex_x(v0i).round() as i64, vd.vertex_y(v0i).round() as i64);
        let v1 = (vd.vertex_x(v1i).round() as i64, vd.vertex_y(v1i).round() as i64);
        // VoronoiUtils.cpp:231 assert(v0 != to_i64 || v1 != from_i64); compiled out of the
        // reference build, and `detect_known_voronoi_cell_issues` (Voronoi.cpp:192-234)
        // runs this function on the very diagrams the assertion describes.

        // VoronoiUtils.cpp:233-238 Use the last edge which starts in source_segment.to
        if v0 == to_i64 && !after_start {
            cell_range.edge_begin = Some(edge);
            seen_possible_start = true;
        } else if seen_possible_start {
            after_start = true;
        }

        // VoronoiUtils.cpp:240-243
        if v1 == from_i64 && (cell_range.edge_end.is_none() || ending_edge_is_set_before_start) {
            ending_edge_is_set_before_start = !after_start;
            cell_range.edge_end = Some(edge);
        }

        // VoronoiUtils.cpp:244 while (edge = edge->next(), edge != cell.incident_edge());
        edge = vd.edge_next(edge);
        if edge == incident_edge {
            break;
        }
    }

    // VoronoiUtils.cpp:246 return cell_range;
    cell_range
}
