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
//!    `dump_voronoi_to_svg` debug dumper. These are now fully ported. The
//!    cell-source category, `contains_point()`/`contains_segment()`,
//!    `source_index()`, `is_secondary()`, `edge_rot_next()`, `edge_get_twin()`
//!    and vertex/edge color accessors are all available on the `boostvoronoi`
//!    crate's `Diagram`/`Cell`/`Edge` types, and the `Slic3r::SVG` drawing API
//!    (`draw_point`, `draw_line`, `draw_lines`, `draw_outline_polygons`,
//!    `close`) plus `Voronoi::vertex_category` (`geometry::voronoi_annotation`)
//!    are all present. `dump_voronoi_to_svg` is a debug-only dumper that
//!    produces no G-code, but it is ported here faithfully for parity.

use boostvoronoi::prelude as bv;

use crate::geometry::bounding_box::BoundingBox;
use crate::geometry::line::get_extents as get_extents_lines;
use crate::geometry::polygon::get_extents_polygons;
use crate::geometry::voronoi_annotation::vertex_category;
use crate::geometry::voronoi_diagram::VoronoiDiagram;
use crate::geometry::{Line, PointF};
use crate::geometry::Point as IPoint;
use crate::geometry::Polygons;
use crate::svg::SVG;
use crate::{Coord, SCALING_FACTOR};

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

// VoronoiVisualUtils.hpp:198-211
//   using VD = Geometry::VoronoiDiagram;
//   typedef double coordinate_type;
//   typedef boost::polygon::point_data<coordinate_type> point_type;
//   typedef boost::polygon::segment_data<coordinate_type> segment_type;
//   ... cell/edge/vertex iterator typedefs ...
// In Rust the diagram is the `boostvoronoi` crate's `Diagram` (wrapped by
// `geometry::voronoi_diagram::VoronoiDiagram`). `point_type`/`segment_type`
// are the local [`Point`]/[`Segment`] (`double`) structs above.

/// `color_exterior` — recursively color exterior Voronoi edges/vertices.
///
/// VoronoiVisualUtils.hpp:216-231
///
/// ```cpp
/// inline void color_exterior(const VD::edge_type* edge) {
///     if (edge->color() == EXTERNAL_COLOR) return;
///     edge->color(EXTERNAL_COLOR);
///     edge->twin()->color(EXTERNAL_COLOR);
///     const VD::vertex_type* v = edge->vertex1();
///     if (v == NULL || !edge->is_primary()) return;
///     v->color(EXTERNAL_COLOR);
///     const VD::edge_type* e = v->incident_edge();
///     do { color_exterior(e); e = e->rot_next(); } while (e != v->incident_edge());
/// }
/// ```
///
/// The `boostvoronoi` `Diagram` stores colors on edges/vertices, so the
/// const-mutation of `edge->color(...)` maps to `&mut Diagram` here.
/// Pointer-identity comparisons (`e != v->incident_edge()`) map to comparing
/// `EdgeIndex` values.
pub fn color_exterior(diagram: &mut bv::Diagram, edge: bv::EdgeIndex) {
    // VoronoiVisualUtils.hpp:218-219
    //   if (edge->color() == EXTERNAL_COLOR) return;
    if diagram
        .edge_get_color(edge)
        .map(|c| c == EXTERNAL_COLOR as bv::ColorType)
        .unwrap_or(false)
    {
        return;
    }
    // VoronoiVisualUtils.hpp:220   edge->color(EXTERNAL_COLOR);
    let _ = diagram.edge_set_color(edge, EXTERNAL_COLOR as bv::ColorType);
    // VoronoiVisualUtils.hpp:221   edge->twin()->color(EXTERNAL_COLOR);
    if let Ok(twin) = diagram.edge_get_twin(edge) {
        let _ = diagram.edge_set_color(twin, EXTERNAL_COLOR as bv::ColorType);
    }
    // VoronoiVisualUtils.hpp:222   const VD::vertex_type* v = edge->vertex1();
    let v = diagram.edge_get_vertex1(edge).ok().flatten();
    // VoronoiVisualUtils.hpp:223-224   if (v == NULL || !edge->is_primary()) return;
    let is_primary = diagram.edge(edge).map(|e| e.is_primary()).unwrap_or(false);
    let v = match v {
        Some(v) if is_primary => v,
        _ => return,
    };
    // VoronoiVisualUtils.hpp:225   v->color(EXTERNAL_COLOR);
    let _ = diagram.vertex_set_color(v, EXTERNAL_COLOR as bv::ColorType);
    // VoronoiVisualUtils.hpp:226   const VD::edge_type* e = v->incident_edge();
    let incident = match diagram.vertex_get_incident_edge(v) {
        Some(e) => e,
        None => return,
    };
    // VoronoiVisualUtils.hpp:227-230
    //   do { color_exterior(e); e = e->rot_next(); } while (e != v->incident_edge());
    let mut e = incident;
    loop {
        color_exterior(diagram, e);
        e = match diagram.edge_rot_next(e) {
            Some(next) => next,
            None => break,
        };
        if e == incident {
            break;
        }
    }
}

/// `retrieve_point` — return the source point for a Voronoi cell.
///
/// VoronoiVisualUtils.hpp:233-241
///
/// ```cpp
/// inline point_type retrieve_point(const Points &points, const std::vector<segment_type> &segments, const cell_type& cell) {
///     assert(cell.source_category() == SOURCE_CATEGORY_SEGMENT_START_POINT || ...);
///     return cell.source_category() == SOURCE_CATEGORY_SINGLE_POINT ?
///         point_type(double(points[cell.source_index()].x()), double(points[cell.source_index()].y())) :
///         (cell.source_category() == SOURCE_CATEGORY_SEGMENT_START_POINT) ?
///             low(segments[cell.source_index()]) : high(segments[cell.source_index()]);
/// }
/// ```
pub fn retrieve_point(points: &[IPoint], segments: &[Segment], cell: &bv::Cell) -> Point {
    // VoronoiVisualUtils.hpp:235-236   assert(cell.source_category() == ... );
    debug_assert!(
        cell.source_category() == bv::SourceCategory::SegmentStart
            || cell.source_category() == bv::SourceCategory::SegmentEnd
            || cell.source_category() == bv::SourceCategory::SinglePoint
    );
    // VoronoiVisualUtils.hpp:237-240
    let source_index = cell.source_index().usize();
    if cell.source_category() == bv::SourceCategory::SinglePoint {
        // point_type(double(points[i].x()), double(points[i].y()))
        Point::new(points[source_index].x() as f64, points[source_index].y() as f64)
    } else if cell.source_category() == bv::SourceCategory::SegmentStart {
        // low(segments[cell.source_index()])
        low(&segments[source_index])
    } else {
        // high(segments[cell.source_index()])
        high(&segments[source_index])
    }
}

/// `clip_infinite_edge` — clip an infinite Voronoi edge to a finite segment.
///
/// VoronoiVisualUtils.hpp:243-279
pub fn clip_infinite_edge(
    diagram: &bv::Diagram,
    points: &[IPoint],
    segments: &[Segment],
    edge: bv::EdgeIndex,
    bbox_max_size: f64,
    clipped_edge: &mut Vec<Point>,
) {
    // VoronoiVisualUtils.hpp:245-246
    //   assert(edge.is_infinite());
    //   assert((edge.vertex0() == nullptr) != (edge.vertex1() == nullptr));
    debug_assert!(diagram.edge_is_infinite(edge).unwrap_or(false));
    let v0 = diagram.edge_get_vertex0(edge).ok().flatten();
    let v1 = diagram.edge_get_vertex1(edge).ok().flatten();
    debug_assert!(v0.is_none() != v1.is_none());

    // VoronoiVisualUtils.hpp:248-249
    //   const cell_type& cell1 = *edge.cell();
    //   const cell_type& cell2 = *edge.twin()->cell();
    let cell1_id = diagram.edge_get_cell(edge).unwrap();
    let twin = diagram.edge_get_twin(edge).unwrap();
    let cell2_id = diagram.edge_get_cell(twin).unwrap();
    let cell1 = *diagram.cell(cell1_id).unwrap();
    let cell2 = *diagram.cell(cell2_id).unwrap();
    // VoronoiVisualUtils.hpp:250-255
    //   Infinite edges could not be created by two segment sites.
    //   assert(cell1.contains_point() || cell2.contains_point());
    debug_assert!(cell1.contains_point() || cell2.contains_point());
    if !cell1.contains_point() && !cell2.contains_point() {
        // printf("Error! clip_infinite_edge - infinite edge separates two segment cells\n");
        eprintln!("Error! clip_infinite_edge - infinite edge separates two segment cells");
        return;
    }
    // VoronoiVisualUtils.hpp:256   point_type direction;
    let mut direction = Point::default();
    // VoronoiVisualUtils.hpp:257-264
    if cell1.contains_point() && cell2.contains_point() {
        // assert(! edge.is_secondary());
        debug_assert!(!diagram.edge(edge).map(|e| e.is_secondary()).unwrap_or(true));
        // point_type p1 = retrieve_point(points, segments, cell1);
        // point_type p2 = retrieve_point(points, segments, cell2);
        let mut p1 = retrieve_point(points, segments, &cell1);
        let mut p2 = retrieve_point(points, segments, &cell2);
        // if (edge.vertex0() == nullptr) std::swap(p1, p2);
        if v0.is_none() {
            std::mem::swap(&mut p1, &mut p2);
        }
        // direction.x(p1.y() - p2.y());
        // direction.y(p2.x() - p1.x());
        direction.x = p1.y() - p2.y();
        direction.y = p2.x() - p1.x();
    } else {
        // VoronoiVisualUtils.hpp:265-269
        // assert(edge.is_secondary());
        debug_assert!(diagram.edge(edge).map(|e| e.is_secondary()).unwrap_or(false));
        // segment_type segment = cell1.contains_segment() ? segments[cell1.source_index()] : segments[cell2.source_index()];
        let segment = if cell1.contains_segment() {
            segments[cell1.source_index().usize()]
        } else {
            segments[cell2.source_index().usize()]
        };
        // direction.x(high(segment).y() - low(segment).y());
        // direction.y(low(segment).x() - high(segment).x());
        direction.x = high(&segment).y() - low(&segment).y();
        direction.y = low(&segment).x() - high(&segment).x();
    }
    // VoronoiVisualUtils.hpp:271
    //   coordinate_type koef = bbox_max_size / (std::max)(fabs(direction.x()), fabs(direction.y()));
    let koef: f64 = bbox_max_size / direction.x().abs().max(direction.y().abs());
    // VoronoiVisualUtils.hpp:272-278
    if v0.is_none() {
        // edge.vertex0() == nullptr
        let v1 = v1.unwrap();
        let v1x = diagram.vertices()[v1.usize()].x();
        let v1y = diagram.vertices()[v1.usize()].y();
        clipped_edge.push(Point::new(
            v1x + direction.x() * koef,
            v1y + direction.y() * koef,
        ));
        clipped_edge.push(Point::new(v1x, v1y));
    } else {
        let v0 = v0.unwrap();
        let v0x = diagram.vertices()[v0.usize()].x();
        let v0y = diagram.vertices()[v0.usize()].y();
        clipped_edge.push(Point::new(v0x, v0y));
        clipped_edge.push(Point::new(
            v0x + direction.x() * koef,
            v0y + direction.y() * koef,
        ));
    }
}

/// `sample_curved_edge` — discretize a curved (parabolic) Voronoi edge.
///
/// VoronoiVisualUtils.hpp:281-290
///
/// ```cpp
/// inline void sample_curved_edge(const Points &points, const std::vector<segment_type> &segments, const edge_type& edge, std::vector<point_type> &sampled_edge, coordinate_type max_dist) {
///     point_type point = edge.cell()->contains_point() ?
///         retrieve_point(points, segments, *edge.cell()) : retrieve_point(points, segments, *edge.twin()->cell());
///     segment_type segment = edge.cell()->contains_point() ?
///         segments[edge.twin()->cell()->source_index()] : segments[edge.cell()->source_index()];
///     ::boost::polygon::voronoi_visual_utils<coordinate_type>::discretize(point, segment, max_dist, &sampled_edge);
/// }
/// ```
pub fn sample_curved_edge(
    diagram: &bv::Diagram,
    points: &[IPoint],
    segments: &[Segment],
    edge: bv::EdgeIndex,
    sampled_edge: &mut Vec<Point>,
    max_dist: f64,
) {
    // VoronoiVisualUtils.hpp:283-285
    let cell_id = diagram.edge_get_cell(edge).unwrap();
    let cell = *diagram.cell(cell_id).unwrap();
    let twin = diagram.edge_get_twin(edge).unwrap();
    let twin_cell_id = diagram.edge_get_cell(twin).unwrap();
    let twin_cell = *diagram.cell(twin_cell_id).unwrap();
    let point = if cell.contains_point() {
        retrieve_point(points, segments, &cell)
    } else {
        retrieve_point(points, segments, &twin_cell)
    };
    // VoronoiVisualUtils.hpp:286-288
    let segment = if cell.contains_point() {
        segments[twin_cell.source_index().usize()]
    } else {
        segments[cell.source_index().usize()]
    };
    // VoronoiVisualUtils.hpp:289
    //   ::boost::polygon::voronoi_visual_utils<coordinate_type>::discretize(point, segment, max_dist, &sampled_edge);
    VoronoiVisualUtils::discretize(&point, &segment, max_dist, sampled_edge);
}

// =============================================================================
// namespace Slic3r { ... dump_voronoi_to_svg
// =============================================================================

/// `get_extents(const Points &)` (the `IncludeBoundary == false` instantiation
/// used by `dump_voronoi_to_svg`).
///
/// Point.cpp:251-257 — `BoundingBox::construct<false>(out, pts.begin(), pts.end())`
/// merges every point into the bounding box.
fn get_extents_points(pts: &[IPoint]) -> BoundingBox {
    let mut out = BoundingBox::new();
    for p in pts {
        out.merge_point(*p);
    }
    out
}

/// `dump_voronoi_to_svg` — debug dumper rendering the Voronoi diagram to SVG.
///
/// VoronoiVisualUtils.hpp:296-451
///
/// `static inline` debug visualization only — produces no G-code. Ported
/// faithfully for parity. The C++ default-argument overload is realized by
/// `Default::default()` for the optional `offset_curves`/`helper_lines`/`scale`.
#[allow(clippy::too_many_arguments)]
pub fn dump_voronoi_to_svg(
    path: &str,
    vd: &VoronoiDiagram,
    points: &[IPoint],
    lines: &[Line],
    offset_curves: &Polygons,
    helper_lines: &[Line],
    mut scale: f64,
) {
    let diagram = vd.diagram();

    // VoronoiVisualUtils.hpp:305   const bool internalEdgesOnly = false;
    let internal_edges_only: bool = false;

    // VoronoiVisualUtils.hpp:307-316
    let mut bbox = BoundingBox::new();
    bbox.merge(&get_extents_points(points));
    bbox.merge(&get_extents_lines(lines));
    bbox.merge(&get_extents_polygons(offset_curves));
    bbox.merge(&get_extents_lines(helper_lines));
    // for (... const_vertex_iterator it ...) if (!internalEdgesOnly || it->color() != EXTERNAL_COLOR) bbox.merge(Point(it->x(), it->y()));
    for v in diagram.vertices() {
        if !internal_edges_only || v.get_color() != EXTERNAL_COLOR as bv::ColorType {
            bbox.merge_point(IPoint::new(v.x() as Coord, v.y() as Coord));
        }
    }
    // bbox.min -= (0.01 * bbox.size().cast<double>()).cast<coord_t>();
    // bbox.max += (0.01 * bbox.size().cast<double>()).cast<coord_t>();
    // NOTE: these are two sequential statements in C++; the `-=` on `bbox.min`
    // mutates the box, so the second `bbox.size()` (for max) reads the ALREADY
    // enlarged size (= original_size + min offset). Mirror that ordering exactly.
    let size_min = bbox.size();
    let off_min_x = (0.01 * size_min.x() as f64) as Coord;
    let off_min_y = (0.01 * size_min.y() as f64) as Coord;
    bbox.min.x -= off_min_x;
    bbox.min.y -= off_min_y;
    let size_max = bbox.size();
    let off_max_x = (0.01 * size_max.x() as f64) as Coord;
    let off_max_y = (0.01 * size_max.y() as f64) as Coord;
    bbox.max.x += off_max_x;
    bbox.max.y += off_max_y;

    // VoronoiVisualUtils.hpp:318-324
    if scale == 0.0 {
        // scale = 0.01 * std::min(bbox.size().x(), bbox.size().y());
        scale = 0.01 * std::cmp::min(bbox.size().x(), bbox.size().y()) as f64;
    } else {
        // scale *= SCALING_FACTOR;
        // C++ SCALING_FACTOR is 0.00001 (libslic3r.h:58); the crate constant is its
        // reciprocal 100_000.0 (lib.rs:418), so C++'s `*= 0.00001` is `/= SCALING_FACTOR` here.
        scale /= SCALING_FACTOR;
    }

    // VoronoiVisualUtils.hpp:326-344
    let input_segment_point_color = "lightseagreen";
    let input_segment_point_radius: Coord = std::cmp::max(1, (0.09 * scale) as Coord);
    let input_segment_color = "lightseagreen";
    let input_segment_line_width: Coord = (0.03 * scale) as Coord;

    let voronoi_point_color = "black";
    let voronoi_point_color_outside = "red";
    let voronoi_point_color_inside = "blue";
    let voronoi_point_radius: Coord = std::cmp::max(1, (0.06 * scale) as Coord);
    let voronoi_line_color_primary = "black";
    let voronoi_line_color_secondary = "green";
    let voronoi_arc_color = "red";
    let voronoi_line_width: Coord = (0.02 * scale) as Coord;

    let offset_curve_color = "magenta";
    let offset_curve_line_width: Coord = (0.02 * scale) as Coord;

    let helper_line_color = "orange";
    let helper_line_width: Coord = (0.04 * scale) as Coord;

    // VoronoiVisualUtils.hpp:346   const bool primaryEdgesOnly = false;
    let primary_edges_only: bool = false;

    // VoronoiVisualUtils.hpp:348   ::Slic3r::SVG svg(path, bbox);
    let mut svg = SVG::new_bbox_default(path, &bbox);

    // VoronoiVisualUtils.hpp:350-354
    // For clipping of half-lines to some reasonable value.
    let bbox_dim_max: f64 = std::cmp::max(bbox.size().x(), bbox.size().y()) as f64;
    // For the discretization of the Voronoi parabolic segments.
    let discretization_step: f64 = 0.0002 * bbox_dim_max;

    // VoronoiVisualUtils.hpp:356-361
    // Make a copy of the input segments with the double type.
    let mut segments: Vec<Segment> = Vec::new();
    for it in lines {
        segments.push(Segment::new(
            Point::new(it.a.x() as f64, it.a.y() as f64),
            Point::new(it.b.x() as f64, it.b.y() as f64),
        ));
    }

    // VoronoiVisualUtils.hpp:363-368
    // Color exterior edges.
    if internal_edges_only {
        // for (... const_edge_iterator it ...) if (!it->is_finite()) color_exterior(&(*it));
        // (debug-only; `internalEdgesOnly` is hard-coded false so this branch is dead.)
    }

    // VoronoiVisualUtils.hpp:370-374
    // Draw the end points of the input polygon.
    for it in lines {
        svg.draw_point(&it.a, input_segment_point_color, input_segment_point_radius);
        svg.draw_point(&it.b, input_segment_point_color, input_segment_point_radius);
    }
    // VoronoiVisualUtils.hpp:375-377
    // Draw the input polygon.
    for it in lines {
        svg.draw_line(
            &Line::new(
                IPoint::new(it.a.x(), it.a.y()),
                IPoint::new(it.b.x(), it.b.y()),
            ),
            input_segment_color,
            input_segment_line_width as f64,
        );
    }

    // VoronoiVisualUtils.hpp:379-394   #if 1 ... Draw voronoi vertices.
    for vi in 0..diagram.vertices().len() {
        let v = &diagram.vertices()[vi];
        if !internal_edges_only || v.get_color() != EXTERNAL_COLOR as bv::ColorType {
            // VoronoiVisualUtils.hpp:383-389
            let color = match vertex_category(diagram, v.get_id()) {
                crate::geometry::voronoi_annotation::VertexCategory::OnContour => voronoi_point_color,
                crate::geometry::voronoi_annotation::VertexCategory::Outside => {
                    voronoi_point_color_outside
                }
                crate::geometry::voronoi_annotation::VertexCategory::Inside => {
                    voronoi_point_color_inside
                }
                // default: color = &voronoiPointColor; // assert(false);
                _ => voronoi_point_color,
            };
            // VoronoiVisualUtils.hpp:390-393
            // FIDELITY-NOTE(F2): C++ casts `it->x()` (double) to `coord_t` (int32);
            // the `it->x() * pt.x() >= 0.` check detects int32 overflow/wraparound
            // (sign flip). With crate `Coord = i64` the wrap threshold differs, so the
            // validity test diverges for coordinates outside the int32 range.
            let pt = IPoint::new(v.x() as Coord, v.y() as Coord);
            if v.x() * pt.x() as f64 >= 0. && v.y() * pt.y() as f64 >= 0. {
                // Conversion to coord_t is valid.
                svg.draw_point(
                    &IPoint::new(v.x() as Coord, v.y() as Coord),
                    color,
                    voronoi_point_radius,
                );
            }
        }
    }

    // VoronoiVisualUtils.hpp:396-444
    for ei in 0..diagram.edges().len() {
        let edge_id = diagram.edge_index_unchecked(ei);
        let edge = diagram.edges()[ei];
        // VoronoiVisualUtils.hpp:397-398   if (primaryEdgesOnly && !it->is_primary()) continue;
        if primary_edges_only && !edge.is_primary() {
            continue;
        }
        // VoronoiVisualUtils.hpp:399-400   if (internalEdgesOnly && (it->color() == EXTERNAL_COLOR)) continue;
        if internal_edges_only && edge.get_color() == EXTERNAL_COLOR as bv::ColorType {
            continue;
        }
        // VoronoiVisualUtils.hpp:401-402
        let mut samples: Vec<Point> = Vec::new();
        let mut color = voronoi_line_color_primary;
        // VoronoiVisualUtils.hpp:403   if (!it->is_finite()) {
        if !diagram.edge_is_finite(edge_id).unwrap_or(false) {
            // VoronoiVisualUtils.hpp:404-406
            clip_infinite_edge(diagram, points, &segments, edge_id, bbox_dim_max, &mut samples);
            if !edge.is_primary() {
                color = voronoi_line_color_secondary;
            }
        } else {
            // VoronoiVisualUtils.hpp:407-417
            // Store both points of the segment into samples. sample_curved_edge will split the initial line
            // until the discretization_step is reached.
            let v0 = diagram.edge_get_vertex0(edge_id).ok().flatten().unwrap();
            let v1 = diagram.edge_get_vertex1(edge_id).ok().flatten().unwrap();
            samples.push(Point::new(
                diagram.vertices()[v0.usize()].x(),
                diagram.vertices()[v0.usize()].y(),
            ));
            samples.push(Point::new(
                diagram.vertices()[v1.usize()].x(),
                diagram.vertices()[v1.usize()].y(),
            ));
            if edge.is_curved() {
                sample_curved_edge(
                    diagram,
                    points,
                    &segments,
                    edge_id,
                    &mut samples,
                    discretization_step,
                );
                color = voronoi_arc_color;
            } else if !edge.is_primary() {
                color = voronoi_line_color_secondary;
            }
        }
        // VoronoiVisualUtils.hpp:418-443
        let mut i = 0usize;
        while i + 1 < samples.len() {
            // Vec2d a(samples[i].x(), samples[i].y());
            // Vec2d b(samples[i+1].x(), samples[i+1].y());
            let a = PointF::new(samples[i].x(), samples[i].y());
            let b = PointF::new(samples[i + 1].x(), samples[i + 1].y());
            // Convert to coord_t.
            // Point ia = a.cast<coord_t>();
            // Point ib = b.cast<coord_t>();
            let mut ia = IPoint::new(a.x() as Coord, a.y() as Coord);
            let mut ib = IPoint::new(b.x() as Coord, b.y() as Coord);
            // Is the conversion possible? Do the resulting points fit into int32_t?
            // auto in_range = [](const Point &ip, const Vec2d &p) { return p.x() * ip.x() >= 0. && p.y() * ip.y() >= 0.; };
            // FIDELITY-NOTE(F2): the in_range predicate exists to detect a
            // double->int32 (`coord_t`) overflow via sign-flip. With crate
            // `Coord = i64` the cast `as Coord` only wraps past the i64 range, so
            // points that overflow int32 (but fit i64) are accepted here where C++
            // would clip/skip them.
            let in_range = |ip: &IPoint, p: &PointF| -> bool {
                p.x() * ip.x() as f64 >= 0. && p.y() * ip.y() as f64 >= 0.
            };
            let a_in_range = in_range(&ia, &a);
            let b_in_range = in_range(&ib, &b);
            // VoronoiVisualUtils.hpp:428-441
            if !a_in_range || !b_in_range {
                if !a_in_range && !b_in_range {
                    // None fits, ignore.
                    i += 1;
                    continue;
                }
                // One fit, the other does not. Try to clip.
                // Vec2d v = b - a; v.normalize(); v *= bbox.size().cast<double>().norm();
                // C++ Eigen normalize() divides by the L2 norm UNCONDITIONALLY (PointF::normalize
                // guards len>0); reproduce the bare divide so degenerate a==b matches C++ (NaN).
                let mut v = PointF::new(b.x() - a.x(), b.y() - a.y());
                let v_norm = (v.x() * v.x() + v.y() * v.y()).sqrt();
                v = PointF::new(v.x() / v_norm, v.y() / v_norm);
                let size_norm =
                    ((bbox.size().x() as f64).powi(2) + (bbox.size().y() as f64).powi(2)).sqrt();
                v = PointF::new(v.x() * size_norm, v.y() * size_norm);
                // auto p = a_in_range ? Vec2d(a + v) : Vec2d(b - v);
                let p = if a_in_range {
                    PointF::new(a.x() + v.x(), a.y() + v.y())
                } else {
                    PointF::new(b.x() - v.x(), b.y() - v.y())
                };
                // Point ip = p.cast<coord_t>();
                let ip = IPoint::new(p.x() as Coord, p.y() as Coord);
                // if (! in_range(ip, p)) continue;
                if !in_range(&ip, &p) {
                    i += 1;
                    continue;
                }
                // (a_in_range ? ib : ia) = ip;
                if a_in_range {
                    ib = ip;
                } else {
                    ia = ip;
                }
            }
            // svg.draw(Line(ia, ib), color, voronoiLineWidth);
            svg.draw_line(&Line::new(ia, ib), color, voronoi_line_width as f64);
            i += 1;
        }
    }
    // VoronoiVisualUtils.hpp:445   #endif

    // VoronoiVisualUtils.hpp:447-448
    svg.draw_outline_polygons(offset_curves, offset_curve_color, offset_curve_line_width as f64);
    svg.draw_lines(&helper_lines.to_vec(), helper_line_color, helper_line_width as f64);

    // VoronoiVisualUtils.hpp:450   svg.Close();
    svg.close();
}
