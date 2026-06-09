//! Voronoi diagram visualization utilities.
//!
//! 1:1 port of `Geometry/VoronoiVisualUtils.hpp` (header-only).
//!
//! The following code for the visualization of the boost Voronoi diagram is based on:
//!
//! Boost.Polygon library voronoi_graphic_utils.hpp header file
//!          Copyright Andrii Sydorchuk 2010-2012.
//! Distributed under the Boost Software License, Version 1.0.
//!    (See accompanying file LICENSE_1_0.txt or copy at
//!          http://www.boost.org/LICENSE_1_0.txt)
//!
//! ## Porting notes
//!
//! The C++ source lives entirely in a header. It has two parts:
//!
//! 1. `boost::polygon::voronoi_visual_utils<CT>` — a templated helper class.
//!    In every actual instantiation in Slic3r `CT == double` (see
//!    `Voronoi::Internal::coordinate_type`). This is pure floating-point math
//!    and is ported faithfully below. `Point<CT>`/`Segment<CT>` are modeled by
//!    the local [`Point`]/[`Segment`] structs (boost::polygon point/segment
//!    concepts with `double` coordinates). `coordf_t -> f64`.
//!
//! 2. `Slic3r` namespace: the `Voronoi::Internal` helpers and the
//!    `dump_voronoi_to_svg` debug dumper. The math-level helpers
//!    (`retrieve_point`, `sample_curved_edge`) are ported here. The remaining
//!    helpers (`color_exterior`, `clip_infinite_edge`) and `dump_voronoi_to_svg`
//!    are blocked: they require the boost::polygon `voronoi_diagram` cell-source
//!    iteration API and the `Slic3r::SVG` drawing API (`svg.draw(Point, ...)`,
//!    `svg.draw(Line, ...)`, `svg.draw_outline`, `Voronoi::vertex_category`),
//!    none of which are available in a matching shape against the
//!    `boostvoronoi` crate / the diverged Rust `svg.rs` port. They produce no
//!    G-code (debug visualization only). See the bottom of this file.

// VoronoiVisualUtils.hpp:200-202
//   typedef double coordinate_type;
//   typedef boost::polygon::point_data<coordinate_type> point_type;
//   typedef boost::polygon::segment_data<coordinate_type> segment_type;
/// boost::polygon `point_data<double>` (a point concept with `double` coords).
///
/// VoronoiVisualUtils.hpp: `Point<CT>` / `point_type`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

impl Point {
    #[inline]
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
    /// `x(point)` accessor (boost::polygon point concept).
    #[inline]
    pub fn x(&self) -> f64 {
        self.x
    }
    /// `y(point)` accessor (boost::polygon point concept).
    #[inline]
    pub fn y(&self) -> f64 {
        self.y
    }
}

/// boost::polygon `segment_data<double>` (a segment concept with `double` coords).
///
/// VoronoiVisualUtils.hpp: `Segment<CT>` / `segment_type`
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Segment {
    pub low: Point,
    pub high: Point,
}

impl Segment {
    #[inline]
    pub fn new(low: Point, high: Point) -> Self {
        Self { low, high }
    }
    /// `low(segment)` accessor (boost::polygon segment concept).
    #[inline]
    pub fn low(&self) -> Point {
        self.low
    }
    /// `high(segment)` accessor (boost::polygon segment concept).
    #[inline]
    pub fn high(&self) -> Point {
        self.high
    }
}

// =============================================================================
// namespace boost::polygon { template <typename CT> class voronoi_visual_utils
// =============================================================================
//
// In Slic3r `CT == double`, so the class becomes a set of free functions over
// `f64`. The full boost::polygon `enable_if<gtl_and<...>>` SFINAE machinery on
// the template only constrains which `Point`/`Segment` concepts are accepted at
// compile time; it has no runtime effect and is elided here.

/// `voronoi_visual_utils<double>`
///
/// VoronoiVisualUtils.hpp:19-181
pub struct VoronoiVisualUtils;

impl VoronoiVisualUtils {
    /// Discretize parabolic Voronoi edge.
    /// Parabolic Voronoi edges are always formed by one point and one segment
    /// from the initial input set.
    ///
    /// Args:
    ///   point: input point.
    ///   segment: input segment.
    ///   max_dist: maximum discretization distance.
    ///   discretization: point discretization of the given Voronoi edge.
    ///
    /// Important:
    ///   discretization should contain both edge endpoints initially.
    ///
    /// VoronoiVisualUtils.hpp:39-132
    pub fn discretize(
        point: &Point,
        segment: &Segment,
        max_dist: f64,
        discretization: &mut Vec<Point>,
    ) {
        // VoronoiVisualUtils.hpp:62-67
        // Apply the linear transformation to move start point of the segment to
        // the point with coordinates (0, 0) and the direction of the segment to
        // coincide the positive direction of the x-axis.
        let segm_vec_x: f64 = Self::cast(x(high(segment))) - Self::cast(x(low(segment)));
        let segm_vec_y: f64 = Self::cast(y(high(segment))) - Self::cast(y(low(segment)));
        let sqr_segment_length: f64 = segm_vec_x * segm_vec_x + segm_vec_y * segm_vec_y;

        // VoronoiVisualUtils.hpp:69-75
        // Compute x-coordinates of the endpoints of the edge
        // in the transformed space.
        let projection_start: f64 =
            sqr_segment_length * Self::get_point_projection(&discretization[0], segment);
        let projection_end: f64 =
            sqr_segment_length * Self::get_point_projection(&discretization[1], segment);
        debug_assert!(projection_start != projection_end);

        // VoronoiVisualUtils.hpp:77-83
        // Compute parabola parameters in the transformed space.
        // Parabola has next representation:
        // f(x) = ((x-rot_x)^2 + rot_y^2) / (2.0*rot_y).
        let point_vec_x: f64 = Self::cast(x(*point)) - Self::cast(x(low(segment)));
        let point_vec_y: f64 = Self::cast(y(*point)) - Self::cast(y(low(segment)));
        let rot_x: f64 = segm_vec_x * point_vec_x + segm_vec_y * point_vec_y;
        let rot_y: f64 = segm_vec_x * point_vec_y - segm_vec_y * point_vec_x;

        // VoronoiVisualUtils.hpp:85-87
        // Save the last point.
        let last_point: Point = discretization[1];
        discretization.pop();

        // VoronoiVisualUtils.hpp:89-93
        // Use stack to avoid recursion.
        let mut point_stack: Vec<f64> = Vec::new();
        point_stack.push(projection_end);
        let mut cur_x: f64 = projection_start;
        let mut cur_y: f64 = Self::parabola_y(cur_x, rot_x, rot_y);

        // VoronoiVisualUtils.hpp:95-96
        // Adjust max_dist parameter in the transformed space.
        let max_dist_transformed: f64 = max_dist * max_dist * sqr_segment_length;
        // VoronoiVisualUtils.hpp:97
        while !point_stack.is_empty() {
            // VoronoiVisualUtils.hpp:98-99
            let new_x: f64 = *point_stack.last().unwrap();
            let new_y: f64 = Self::parabola_y(new_x, rot_x, rot_y);

            // VoronoiVisualUtils.hpp:101-106
            // Compute coordinates of the point of the parabola that is
            // furthest from the current line segment.
            let mid_x: f64 = (new_y - cur_y) / (new_x - cur_x) * rot_y + rot_x;
            let mid_y: f64 = Self::parabola_y(mid_x, rot_x, rot_y);
            debug_assert!(mid_x != cur_x || mid_y != cur_y);
            debug_assert!(mid_x != new_x || mid_y != new_y);

            // VoronoiVisualUtils.hpp:108-114
            // Compute maximum distance between the given parabolic arc
            // and line segment that discretize it.
            let mut dist: f64 = (new_y - cur_y) * (mid_x - cur_x) - (new_x - cur_x) * (mid_y - cur_y);
            let div: f64 =
                (new_y - cur_y) * (new_y - cur_y) + (new_x - cur_x) * (new_x - cur_x);
            debug_assert!(div != 0.0);
            dist = dist * dist / div;
            // VoronoiVisualUtils.hpp:115-127
            if dist <= max_dist_transformed {
                // Distance between parabola and line segment is less than max_dist.
                point_stack.pop();
                let inter_x: f64 = (segm_vec_x * new_x - segm_vec_y * new_y) / sqr_segment_length
                    + Self::cast(x(low(segment)));
                let inter_y: f64 = (segm_vec_x * new_y + segm_vec_y * new_x) / sqr_segment_length
                    + Self::cast(y(low(segment)));
                discretization.push(Point::new(inter_x, inter_y));
                cur_x = new_x;
                cur_y = new_y;
            } else {
                point_stack.push(mid_x);
            }
        }

        // VoronoiVisualUtils.hpp:130-131
        // Update last point.
        if let Some(last) = discretization.last_mut() {
            *last = last_point;
        }
    }

    // VoronoiVisualUtils.hpp:135-138
    // Compute y(x) = ((x - a) * (x - a) + b * b) / (2 * b).
    #[inline]
    fn parabola_y(x: f64, a: f64, b: f64) -> f64 {
        ((x - a) * (x - a) + b * b) / (b + b)
    }

    // VoronoiVisualUtils.hpp:140-175
    // Get normalized length of the distance between:
    //   1) point projection onto the segment
    //   2) start point of the segment
    // Return this length divided by the segment length. This is made to avoid
    // sqrt computation during transformation from the initial space to the
    // transformed one and vice versa. The assumption is made that projection of
    // the point lies between the start-point and endpoint of the segment.
    fn get_point_projection(point: &Point, segment: &Segment) -> f64 {
        let segment_vec_x: f64 = Self::cast(x(high(segment))) - Self::cast(x(low(segment)));
        let segment_vec_y: f64 = Self::cast(y(high(segment))) - Self::cast(y(low(segment)));
        let point_vec_x: f64 = x(*point) - Self::cast(x(low(segment)));
        let point_vec_y: f64 = y(*point) - Self::cast(y(low(segment)));
        let sqr_segment_length: f64 =
            segment_vec_x * segment_vec_x + segment_vec_y * segment_vec_y;
        let vec_dot: f64 = segment_vec_x * point_vec_x + segment_vec_y * point_vec_y;
        vec_dot / sqr_segment_length
    }

    // VoronoiVisualUtils.hpp:177-180
    // template <typename InCT> static CT cast(const InCT& value) {
    //   return static_cast<CT>(value); }
    //
    // Every instantiation here has `InCT == CT == double`, so this is the
    // identity. Kept as a function to mirror the C++ call-sites verbatim.
    #[inline]
    fn cast(value: f64) -> f64 {
        value
    }
}

// boost::polygon point/segment concept free accessors, mirroring the
// `x(...)`, `y(...)`, `low(...)`, `high(...)` calls used verbatim above.
#[inline]
fn x(p: Point) -> f64 {
    p.x
}
#[inline]
fn y(p: Point) -> f64 {
    p.y
}
#[inline]
fn low(s: &Segment) -> Point {
    s.low
}
#[inline]
fn high(s: &Segment) -> Point {
    s.high
}

// =============================================================================
// namespace Slic3r { namespace Voronoi { namespace Internal {
// =============================================================================
//
// The following code for the visualization of the boost Voronoi diagram is
// based on:
//
// Boost.Polygon library voronoi_visualizer.cpp file
//          Copyright Andrii Sydorchuk 2010-2012.
// Distributed under the Boost Software License, Version 1.0.

/// VoronoiVisualUtils.hpp:214
/// `static const std::size_t EXTERNAL_COLOR = 1;`
pub const EXTERNAL_COLOR: usize = 1;

/// `retrieve_point` — return the source point for a Voronoi cell.
///
/// The C++ takes a `cell_type` and reads its `source_category()` /
/// `source_index()` to look up either a `Points[i]` (single point site) or a
/// segment endpoint (`low`/`high` of `segments[i]`). Threading the full
/// `boost::polygon` cell-source category through `boostvoronoi` is the part of
/// the visualizer that is blocked (see notes at end of file), so this exposes
/// the underlying decision at the value level: given the resolved segment and
/// whether the cell is a SEGMENT_START_POINT, return the matching endpoint.
///
/// VoronoiVisualUtils.hpp:233-241
pub fn retrieve_point_segment(segment: &Segment, is_segment_start_point: bool) -> Point {
    // cell.source_category() == SOURCE_CATEGORY_SEGMENT_START_POINT ?
    //     low(segments[...]) : high(segments[...])
    if is_segment_start_point {
        low(segment)
    } else {
        high(segment)
    }
}

/// `retrieve_point` — single point site branch.
///
/// VoronoiVisualUtils.hpp:237-238
/// `point_type(double(points[i].x()), double(points[i].y()))`
pub fn retrieve_point_single(point_x: i64, point_y: i64) -> Point {
    Point::new(point_x as f64, point_y as f64)
}

/// `sample_curved_edge` — discretize a curved (parabolic) Voronoi edge.
///
/// The C++ resolves the `point`/`segment` site pair off the edge's cell/twin
/// and forwards to `voronoi_visual_utils<double>::discretize`. The cell-source
/// resolution is blocked (see notes), but the discretization math itself is the
/// tractable, parity-relevant part: callers that have already resolved the
/// `point` and `segment` sites can use this directly.
///
/// VoronoiVisualUtils.hpp:281-290
pub fn sample_curved_edge(
    point: &Point,
    segment: &Segment,
    sampled_edge: &mut Vec<Point>,
    max_dist: f64,
) {
    // ::boost::polygon::voronoi_visual_utils<coordinate_type>::discretize(
    //     point, segment, max_dist, &sampled_edge);
    VoronoiVisualUtils::discretize(point, segment, max_dist, sampled_edge);
}

// ---------------------------------------------------------------------------
// BLOCKED symbols (debug-only visualization; produce no G-code)
// ---------------------------------------------------------------------------
//
// The following are NOT ported. They depend on APIs that are not available in
// a matching shape and are debug-visualization only. They are listed here (not
// stubbed) so a later porter can pick them up once the dependencies land.
//
// * `Voronoi::Internal::color_exterior(const VD::edge_type*)`
//     VoronoiVisualUtils.hpp:216-231
//     Recursively walks `edge->twin()`, `vertex1()->incident_edge()` and
//     `rot_next()`, mutating edge/vertex `color()` through const pointers.
//     boostvoronoi exposes `edge_get_twin` / `edge_rot_next` /
//     `vertex_get_incident_edge` / `*_set_color`, but the C++ relies on
//     pointer-identity `do { } while (e != v->incident_edge())` and const
//     mutation; a faithful, byte-exact translation needs the diagram threaded
//     as `&mut` through a recursion that also reads identity — deferred.
//
// * `Voronoi::Internal::clip_infinite_edge(...)`
//     VoronoiVisualUtils.hpp:243-279
//     Needs `cell.contains_point()/contains_segment()`,
//     `cell.source_index()`, `edge.is_secondary()`, and `retrieve_point` over
//     the live cell-source category. Tractable but only consumed by
//     `dump_voronoi_to_svg`.
//
// * `Slic3r::dump_voronoi_to_svg(...)`
//     VoronoiVisualUtils.hpp:296-451
//     `static inline` debug dumper. Requires the `Slic3r::SVG` drawing API
//     (`svg.draw(Point, color, radius)`, `svg.draw(Line, color, width)`,
//     `svg.draw_outline(Polygons, ...)`), the `get_extents` overloads for
//     Points/Lines/Polygons, `BoundingBox` merge/scale arithmetic, and
//     `Voronoi::vertex_category` over a live `voronoi_diagram<double>`. The
//     Rust `svg.rs` port has a divergent surface (no matching `draw`
//     overloads), so a faithful 1:1 translation is blocked. No G-code impact.
