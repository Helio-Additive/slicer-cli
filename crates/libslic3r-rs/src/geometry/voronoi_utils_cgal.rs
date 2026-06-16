//! CGAL-based Voronoi diagram planarity utilities.
//!
//! C++ Reference:
//! - Geometry/VoronoiUtilsCgal.hpp
//! - Geometry/VoronoiUtilsCgal.cpp
//!
//! Faithful 1:1 port of `Slic3r::Geometry::VoronoiUtilsCgal`.
//!
//! # Kernel / native-dependency notes
//!
//! The C++ original is written on top of several CGAL kernels:
//!   * `impl::K   = CGAL::Simple_cartesian<double>`           (inexact, `double`)
//!   * `impl::FK  = CGAL::Simple_cartesian<Interval_nt_advanced>` (interval filter)
//!   * `impl::EK  = CGAL::Simple_cartesian<MP_Float>`         (exact fallback)
//! and wraps the two parabola-tangent orientation predicates in
//! `CGAL::Filtered_predicate`, which evaluates the cheap `double` filter first
//! and only falls back to the exact `MP_Float` kernel when the `double` result
//! is not provably correct.
//!
//! CGAL is a native, non-wasm-safe C++ library and is intentionally NOT added to
//! this crate (matching the established policy in `triangulation.rs`,
//! `mesh_boolean.rs`, etc.). The *mathematical content* of the parabola-tangent
//! predicates is fully expressible in `double` arithmetic — the exact/interval
//! kernels exist purely as a robustness wrapper. We therefore port the predicate
//! geometry and control flow faithfully on `f64` (== `impl::K`), and the
//! `CGAL::sign(...)` calls become exact sign tests on the computed `f64`.
//! See `divergences` in the port report.
//!
//! BLOCKED SYMBOL (native, non-wasm dependency):
//!   `is_voronoi_diagram_planar_intersection` is built on
//!   `CGAL::compute_intersection_points` (the `Surface_sweep_2` segment
//!   intersection enumeration over `Exact_predicates_exact_constructions_kernel`).
//!   There is no pure-Rust drop-in for the CGAL exact sweep in this crate, so it
//!   is left as a documented stub returning the planar-by-construction default,
//!   mirroring the prior placeholder. The angle-based check below is the faithful
//!   port and is the variant actually used by Arachne via `VoronoiDiagram`.

use boostvoronoi::prelude as bv;

use crate::geometry::voronoi_diagram::VoronoiDiagram;
use crate::geometry::{Line, Point};

// VoronoiUtilsCgal.cpp:14 `using VD = Slic3r::Geometry::VoronoiDiagram;`
// In Rust the boost::polygon-backed diagram is `voronoi_diagram::VoronoiDiagram`,
// which exposes the underlying `boostvoronoi::Diagram` via `.diagram()`.

// VoronoiUtilsCgal.cpp:28-30
// The tangent vector of the parabola is computed based on the Proof of the reflective property.
// https://en.wikipedia.org/wiki/Parabola#Proof_of_the_reflective_property
// https://math.stackexchange.com/q/2439647/2439663#comment5039739_2439663
// VoronoiUtilsCgal.cpp:31 `namespace impl`
mod impl_ {
    // VoronoiUtilsCgal.cpp:32 `using K = CGAL::Simple_cartesian<double>;`
    // CGAL kernel `K::Point_2` / `K::Vector_2` are modeled as `[f64; 2]`.
    pub type CgalPoint = [f64; 2];
    pub type CgalVector = [f64; 2];

    // VoronoiUtilsCgal.cpp:81 etc. `CGAL::Orientation`
    // CGAL::Orientation: LEFT_TURN == +1, RIGHT_TURN == -1, COLLINEAR == 0.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum Orientation {
        // CGAL::Orientation::RIGHT_TURN / CLOCKWISE (-1)
        RightTurn,
        // CGAL::Orientation::COLLINEAR (0)
        Collinear,
        // CGAL::Orientation::LEFT_TURN / COUNTERCLOCKWISE (+1)
        LeftTurn,
    }

    // CGAL::opposite(Orientation)
    #[inline]
    pub fn opposite(o: Orientation) -> Orientation {
        match o {
            Orientation::RightTurn => Orientation::LeftTurn,
            Orientation::Collinear => Orientation::Collinear,
            Orientation::LeftTurn => Orientation::RightTurn,
        }
    }

    // CGAL::sign(double) -> Orientation-as-sign (LEFT_TURN/+1, RIGHT_TURN/-1, COLLINEAR/0).
    #[inline]
    pub fn sign(v: f64) -> Orientation {
        if v > 0.0 {
            Orientation::LeftTurn
        } else if v < 0.0 {
            Orientation::RightTurn
        } else {
            Orientation::Collinear
        }
    }

    // CGAL::orientation(p, q, r): sign of the cross product (q-p) x (r-p).
    #[inline]
    pub fn orientation(p: CgalPoint, q: CgalPoint, r: CgalPoint) -> Orientation {
        let v = (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
        sign(v)
    }

    // CGAL::scalar_product(a, b)
    #[inline]
    fn scalar_product(a: CgalVector, b: CgalVector) -> f64 {
        a[0] * b[0] + a[1] * b[1]
    }

    // Vector_2::perpendicular(orientation):
    //   COUNTERCLOCKWISE/LEFT_TURN -> (-y,  x)
    //   CLOCKWISE/RIGHT_TURN       -> ( y, -x)
    #[inline]
    fn perpendicular(v: CgalVector, o: Orientation) -> CgalVector {
        match o {
            Orientation::LeftTurn => [-v[1], v[0]],
            Orientation::RightTurn => [v[1], -v[0]],
            // perpendicular(COLLINEAR) is undefined in CGAL; never used here
            // because callers assert LEFT_TURN || RIGHT_TURN.
            Orientation::Collinear => [-v[1], v[0]],
        }
    }

    // VoronoiUtilsCgal.cpp:39-59
    // template<typename K>
    // inline typename K::Vector_2 calculate_parabolic_tangent_vector(...)
    #[inline]
    pub fn calculate_parabolic_tangent_vector(
        // Test point on the parabola, where the tangent will be calculated.
        p: CgalPoint,
        // Focus point of the parabola.
        f: CgalPoint,
        // Points of a directrix of the parabola.
        u: CgalPoint,
        v: CgalPoint,
        // On which side of the parabolic segment endpoints the focus point is, which determines the orientation of the tangent.
        tangent_orientation: Orientation,
    ) -> CgalVector {
        // VoronoiUtilsCgal.cpp:54 const Vector_2 directrix_vec = v - u;
        let directrix_vec: CgalVector = [v[0] - u[0], v[1] - u[1]];
        // VoronoiUtilsCgal.cpp:55 const RT directrix_vec_sqr_length = CGAL::scalar_product(directrix_vec, directrix_vec);
        let directrix_vec_sqr_length = scalar_product(directrix_vec, directrix_vec);
        // VoronoiUtilsCgal.cpp:56
        // Vector_2 focus_vec = (f - u) * directrix_vec_sqr_length - directrix_vec * CGAL::scalar_product(directrix_vec, p - u);
        let f_minus_u: CgalVector = [f[0] - u[0], f[1] - u[1]];
        let p_minus_u: CgalVector = [p[0] - u[0], p[1] - u[1]];
        let proj = scalar_product(directrix_vec, p_minus_u);
        let focus_vec: CgalVector = [
            f_minus_u[0] * directrix_vec_sqr_length - directrix_vec[0] * proj,
            f_minus_u[1] * directrix_vec_sqr_length - directrix_vec[1] * proj,
        ];
        // VoronoiUtilsCgal.cpp:57 Vector_2 tangent_vec = focus_vec.perpendicular(tangent_orientation);
        let tangent_vec = perpendicular(focus_vec, tangent_orientation);
        // VoronoiUtilsCgal.cpp:58 return tangent_vec;
        tangent_vec
    }

    // VoronoiUtilsCgal.cpp:61-88
    // template<typename K> struct ParabolicTangentToSegmentOrientationPredicate
    // result_type operator()(...)
    #[allow(clippy::too_many_arguments)]
    pub fn parabolic_tangent_to_segment_orientation(
        // Test point on the parabola, where the tangent will be calculated.
        p: CgalPoint,
        // End of the linear segment (p, q), for which orientation towards the tangent to parabola will be evaluated.
        q: CgalPoint,
        // Focus point of the parabola.
        f: CgalPoint,
        // Points of a directrix of the parabola.
        u: CgalPoint,
        v: CgalPoint,
        // On which side of the parabolic segment endpoints the focus point is, which determines the orientation of the tangent.
        tangent_orientation: Orientation,
    ) -> Orientation {
        // VoronoiUtilsCgal.cpp:81
        debug_assert!(
            tangent_orientation == Orientation::LeftTurn
                || tangent_orientation == Orientation::RightTurn
        );

        // VoronoiUtilsCgal.cpp:83 Vector_2 tangent_vec = calculate_parabolic_tangent_vector<K>(p, f, u, v, tangent_orientation);
        let tangent_vec = calculate_parabolic_tangent_vector(p, f, u, v, tangent_orientation);
        // VoronoiUtilsCgal.cpp:84 Vector_2 linear_vec = q - p;
        let linear_vec: CgalVector = [q[0] - p[0], q[1] - p[1]];

        // VoronoiUtilsCgal.cpp:86 return CGAL::sign(tangent_vec.x() * linear_vec.y() - tangent_vec.y() * linear_vec.x());
        sign(tangent_vec[0] * linear_vec[1] - tangent_vec[1] * linear_vec[0])
    }

    // VoronoiUtilsCgal.cpp:90-123
    // template<typename K> struct ParabolicTangentToParabolicTangentOrientationPredicate
    // result_type operator()(...)
    #[allow(clippy::too_many_arguments)]
    pub fn parabolic_tangent_to_parabolic_tangent_orientation(
        // Common point on both parabolas, where the tangent will be calculated.
        p: CgalPoint,
        // Focus point of the first parabola.
        f_0: CgalPoint,
        // Points of a directrix of the first parabola.
        u_0: CgalPoint,
        v_0: CgalPoint,
        // On which side of the parabolic segment endpoints the focus point is, which determines the orientation of the tangent.
        tangent_orientation_0: Orientation,
        // Focus point of the second parabola.
        f_1: CgalPoint,
        // Points of a directrix of the second parabola.
        u_1: CgalPoint,
        v_1: CgalPoint,
        // On which side of the parabolic segment endpoints the focus point is, which determines the orientation of the tangent.
        tangent_orientation_1: Orientation,
    ) -> Orientation {
        // VoronoiUtilsCgal.cpp:115-116
        debug_assert!(
            tangent_orientation_0 == Orientation::LeftTurn
                || tangent_orientation_0 == Orientation::RightTurn
        );
        debug_assert!(
            tangent_orientation_1 == Orientation::LeftTurn
                || tangent_orientation_1 == Orientation::RightTurn
        );

        // VoronoiUtilsCgal.cpp:118 Vector_2 tangent_vec_0 = calculate_parabolic_tangent_vector<K>(p, f_0, u_0, v_0, tangent_orientation_0);
        let tangent_vec_0 =
            calculate_parabolic_tangent_vector(p, f_0, u_0, v_0, tangent_orientation_0);
        // VoronoiUtilsCgal.cpp:119 Vector_2 tangent_vec_1 = calculate_parabolic_tangent_vector<K>(p, f_1, u_1, v_1, tangent_orientation_1);
        let tangent_vec_1 =
            calculate_parabolic_tangent_vector(p, f_1, u_1, v_1, tangent_orientation_1);

        // VoronoiUtilsCgal.cpp:121 return CGAL::sign(tangent_vec_0.x() * tangent_vec_1.y() - tangent_vec_0.y() * tangent_vec_1.x());
        sign(tangent_vec_0[0] * tangent_vec_1[1] - tangent_vec_0[1] * tangent_vec_1[0])
    }
} // namespace impl

use impl_::Orientation;

// VoronoiUtilsCgal.cpp:129-130
// using ParabolicTangentToSegmentOrientation = impl::ParabolicTangentToSegmentOrientationPredicateFiltered;
// using ParabolicTangentToParabolicTangentOrientation = impl::ParabolicTangentToParabolicTangentOrientationPredicateFiltered;
// (The "Filtered" exact/interval kernel wrapper is replaced by the direct `f64`
//  predicate above; see module notes.)

// VoronoiUtilsCgal.cpp:131 using CGAL_Point = impl::K::Point_2;
type CgalPoint = impl_::CgalPoint;

// VoronoiUtilsCgal.cpp:133 inline CGAL_Point to_cgal_point(const VD::vertex_type *pt) { return {pt->x(), pt->y()}; }
#[inline]
fn to_cgal_point_vertex(pt: &bv::Vertex) -> CgalPoint {
    [pt.x(), pt.y()]
}
// VoronoiUtilsCgal.cpp:134 inline CGAL_Point to_cgal_point(const Point &pt) { return {pt.x(), pt.y()}; }
#[inline]
fn to_cgal_point_point(pt: &Point) -> CgalPoint {
    [pt.x as f64, pt.y as f64]
}
// VoronoiUtilsCgal.cpp:135 inline CGAL_Point to_cgal_point(const Vec2d &pt) { return {pt.x(), pt.y()}; }
#[inline]
fn to_cgal_point_vec2d(pt: crate::geometry::Vec2d) -> CgalPoint {
    [pt.x, pt.y]
}

// VoronoiUtilsCgal.cpp:137-142
// inline Linef make_linef(const VD::edge_type &edge)
// {
//     const VD::vertex_type *v0 = edge.vertex0();
//     const VD::vertex_type *v1 = edge.vertex1();
//     return {Vec2d(v0->x(), v0->y()), Vec2d(v1->x(), v1->y())};
// }
#[inline]
fn make_linef(diagram: &bv::Diagram, edge_id: bv::EdgeIndex) -> Linef {
    let v0 = diagram.edge_get_vertex0(edge_id).ok().flatten().unwrap();
    let v1 = diagram.edge_get_vertex1(edge_id).ok().flatten().unwrap();
    let v0 = &diagram.vertices()[v0.usize()];
    let v1 = &diagram.vertices()[v1.usize()];
    Linef {
        a: crate::geometry::Vec2d::new(v0.x(), v0.y()),
        b: crate::geometry::Vec2d::new(v1.x(), v1.y()),
    }
}

// Minimal `Linef` analogue (a pair of `Vec2d` endpoints, mirroring C++ `Linef`).
#[derive(Debug, Clone, Copy)]
struct Linef {
    a: crate::geometry::Vec2d,
    b: crate::geometry::Vec2d,
}

// VoronoiUtilsCgal.cpp:144
// [[maybe_unused]] inline bool is_equal(const VD::vertex_type &vertex_first, const VD::vertex_type &vertex_second)
//     { return vertex_first.x() == vertex_second.x() && vertex_first.y() == vertex_second.y(); }
#[inline]
#[allow(dead_code)]
fn is_equal(vertex_first: &bv::Vertex, vertex_second: &bv::Vertex) -> bool {
    vertex_first.x() == vertex_second.x() && vertex_first.y() == vertex_second.y()
}

// ---------------------------------------------------------------------------
// `VoronoiUtils::get_source_point` / `get_source_segment` (VoronoiUtils.cpp:35-76)
//
// These are templated members of `VoronoiUtils` in C++; the Rust
// `voronoi_utils.rs` only ports the coordinate-level helpers, so the
// cell-based source lookups (needed by `get_parabolic_segment`) are ported here
// against the `Line` segment iterator (== `LinesIt`, the primary instantiation
// used by Arachne).
// ---------------------------------------------------------------------------

// VoronoiUtils.cpp:40-49 VoronoiUtils::get_source_segment
fn get_source_segment(cell: &bv::Cell, segments: &[Line]) -> Line {
    // VoronoiUtils.cpp:42 if (!cell.contains_segment()) throw ...
    assert!(
        cell.contains_segment(),
        "Voronoi cell doesn't contain a source segment!"
    );
    // VoronoiUtils.cpp:45 if (cell.source_index() >= ...) throw ...
    let source_index = cell.source_index().usize();
    assert!(
        source_index < segments.len(),
        "Voronoi cell source index is out of range!"
    );
    // VoronoiUtils.cpp:48 return *(segment_begin + cell.source_index());
    segments[source_index]
}

// VoronoiUtils.cpp:56-76 VoronoiUtils::get_source_point
fn get_source_point(cell: &bv::Cell, segments: &[Line]) -> Point {
    // VoronoiUtils.cpp:60 if (!cell.contains_point()) throw ...
    assert!(
        cell.contains_point(),
        "Voronoi cell doesn't contain a source point!"
    );

    let source_index = cell.source_index().usize();
    match cell.source_category() {
        // VoronoiUtils.cpp:63-66 SOURCE_CATEGORY_SEGMENT_START_POINT -> segment LOW (from)
        bv::SourceCategory::SegmentStart => {
            debug_assert!(source_index < segments.len());
            segments[source_index].a
        }
        // VoronoiUtils.cpp:67-70 SOURCE_CATEGORY_SEGMENT_END_POINT -> segment HIGH (to)
        bv::SourceCategory::SegmentEnd => {
            debug_assert!(source_index < segments.len());
            segments[source_index].b
        }
        // VoronoiUtils.cpp:71-72 SOURCE_CATEGORY_SINGLE_POINT
        bv::SourceCategory::SinglePoint => {
            panic!("Voronoi diagram is always constructed using segments, so cell.source_category() shouldn't be SOURCE_CATEGORY_SINGLE_POINT!");
        }
        // VoronoiUtils.cpp:73-74 default
        bv::SourceCategory::Segment => {
            panic!("Function get_source_point() should only be called on point cells!");
        }
    }
}

// VoronoiUtilsCgal.cpp:180-188 struct ParabolicSegment
struct ParabolicSegment {
    // VoronoiUtilsCgal.cpp:182 const Point focus;
    focus: Point,
    // VoronoiUtilsCgal.cpp:183 const Line directrix;
    directrix: Line,
    // VoronoiUtilsCgal.cpp:184-185 Two points on the parabola; const Linef segment;
    segment: Linef,
    // VoronoiUtilsCgal.cpp:186-187 Indicate if focus point is on the left side or right side relative to parabolic segment endpoints.
    is_focus_on_left: Orientation,
}

// VoronoiUtilsCgal.cpp:190-212
// template<typename SegmentIterator> ... ParabolicSegment
// get_parabolic_segment(const VD::edge_type &edge, const SegmentIterator segment_begin, const SegmentIterator segment_end)
fn get_parabolic_segment(diagram: &bv::Diagram, edge_id: bv::EdgeIndex, segments: &[Line]) -> ParabolicSegment {
    // VoronoiUtilsCgal.cpp:198 assert(edge.is_curved());
    let edge = &diagram.edges()[edge_id.usize()];
    debug_assert!(edge.is_curved());

    // VoronoiUtilsCgal.cpp:200 const VD::cell_type *left_cell = edge.cell();
    let left_cell_id = edge.cell().unwrap();
    // VoronoiUtilsCgal.cpp:201 const VD::cell_type *right_cell = edge.twin()->cell();
    let twin_id = edge.twin().unwrap();
    let right_cell_id = diagram.edges()[twin_id.usize()].cell().unwrap();

    let left_cell = diagram.cell(left_cell_id).unwrap();
    let right_cell = diagram.cell(right_cell_id).unwrap();

    // VoronoiUtilsCgal.cpp:203
    // const Point focus_pt = VoronoiUtils::get_source_point(*(left_cell->contains_point() ? left_cell : right_cell), segment_begin, segment_end);
    let focus_pt = get_source_point(
        if left_cell.contains_point() {
            left_cell
        } else {
            right_cell
        },
        segments,
    );
    // VoronoiUtilsCgal.cpp:204
    // const Segment &directrix = VoronoiUtils::get_source_segment(*(left_cell->contains_point() ? right_cell : left_cell), segment_begin, segment_end);
    let directrix = get_source_segment(
        if left_cell.contains_point() {
            right_cell
        } else {
            left_cell
        },
        segments,
    );
    // VoronoiUtilsCgal.cpp:205
    // CGAL::Orientation focus_side = CGAL::opposite(CGAL::orientation(to_cgal_point(edge.vertex0()), to_cgal_point(edge.vertex1()), to_cgal_point(focus_pt)));
    let v0 = edge.vertex0().unwrap();
    let v1 = diagram.edge_get_vertex1(edge_id).ok().flatten().unwrap();
    let v0 = &diagram.vertices()[v0.usize()];
    let v1 = &diagram.vertices()[v1.usize()];
    let focus_side = impl_::opposite(impl_::orientation(
        to_cgal_point_vertex(v0),
        to_cgal_point_vertex(v1),
        to_cgal_point_point(&focus_pt),
    ));

    // VoronoiUtilsCgal.cpp:207
    debug_assert!(focus_side == Orientation::LeftTurn || focus_side == Orientation::RightTurn);

    // VoronoiUtilsCgal.cpp:209 const Point directrix_from = boost::polygon::segment_traits<Segment>::get(directrix, boost::polygon::LOW);
    let directrix_from = directrix.a;
    // VoronoiUtilsCgal.cpp:210 const Point directrix_to = boost::polygon::segment_traits<Segment>::get(directrix, boost::polygon::HIGH);
    let directrix_to = directrix.b;
    // VoronoiUtilsCgal.cpp:211 return {focus_pt, Line(directrix_from, directrix_to), make_linef(edge), focus_side};
    ParabolicSegment {
        focus: focus_pt,
        directrix: Line::new(directrix_from, directrix_to),
        segment: make_linef(diagram, edge_id),
        is_focus_on_left: focus_side,
    }
}

// VoronoiUtilsCgal.cpp:214-255
// template<typename SegmentIterator> ... CGAL::Orientation
// orientation_of_two_edges(const VD::edge_type &edge_a, const VD::edge_type &edge_b, ...)
fn orientation_of_two_edges(
    diagram: &bv::Diagram,
    edge_a_id: bv::EdgeIndex,
    edge_b_id: bv::EdgeIndex,
    segments: &[Line],
) -> Orientation {
    let edge_a = &diagram.edges()[edge_a_id.usize()];
    let edge_b = &diagram.edges()[edge_b_id.usize()];

    // VoronoiUtilsCgal.cpp:221 assert(is_equal(*edge_a.vertex0(), *edge_b.vertex0()));
    debug_assert!({
        let a0 = &diagram.vertices()[edge_a.vertex0().unwrap().usize()];
        let b0 = &diagram.vertices()[edge_b.vertex0().unwrap().usize()];
        is_equal(a0, b0)
    });
    // VoronoiUtilsCgal.cpp:222 CGAL::Orientation orientation;
    let orientation;
    // VoronoiUtilsCgal.cpp:223 if (edge_a.is_linear() && edge_b.is_linear()) {
    if edge_a.is_linear() && edge_b.is_linear() {
        // VoronoiUtilsCgal.cpp:224
        // orientation = CGAL::orientation(to_cgal_point(edge_a.vertex0()), to_cgal_point(edge_a.vertex1()), to_cgal_point(edge_b.vertex1()));
        let a0 = &diagram.vertices()[edge_a.vertex0().unwrap().usize()];
        let a1 = &diagram.vertices()[diagram
            .edge_get_vertex1(edge_a_id)
            .ok()
            .flatten()
            .unwrap()
            .usize()];
        let b1 = &diagram.vertices()[diagram
            .edge_get_vertex1(edge_b_id)
            .ok()
            .flatten()
            .unwrap()
            .usize()];
        orientation = impl_::orientation(
            to_cgal_point_vertex(a0),
            to_cgal_point_vertex(a1),
            to_cgal_point_vertex(b1),
        );
    } else if edge_a.is_curved() && edge_b.is_curved() {
        // VoronoiUtilsCgal.cpp:225-237
        // const ParabolicSegment parabolic_a = get_parabolic_segment(edge_a, segment_begin, segment_end);
        let parabolic_a = get_parabolic_segment(diagram, edge_a_id, segments);
        // const ParabolicSegment parabolic_b = get_parabolic_segment(edge_b, segment_begin, segment_end);
        let parabolic_b = get_parabolic_segment(diagram, edge_b_id, segments);
        // orientation = ParabolicTangentToParabolicTangentOrientation{}(...);
        let orientation = impl_::parabolic_tangent_to_parabolic_tangent_orientation(
            to_cgal_point_vec2d(parabolic_a.segment.a),
            to_cgal_point_point(&parabolic_a.focus),
            to_cgal_point_point(&parabolic_a.directrix.a),
            to_cgal_point_point(&parabolic_a.directrix.b),
            parabolic_a.is_focus_on_left,
            to_cgal_point_point(&parabolic_b.focus),
            to_cgal_point_point(&parabolic_b.directrix.a),
            to_cgal_point_point(&parabolic_b.directrix.b),
            parabolic_b.is_focus_on_left,
        );
        // VoronoiUtilsCgal.cpp:237 return orientation;
        return orientation;
    } else {
        // VoronoiUtilsCgal.cpp:238-252
        // assert(edge_a.is_curved() != edge_b.is_curved());
        debug_assert!(edge_a.is_curved() != edge_b.is_curved());

        // VoronoiUtilsCgal.cpp:241 const VD::edge_type &linear_edge = edge_a.is_curved() ? edge_b : edge_a;
        let linear_edge_id = if edge_a.is_curved() { edge_b_id } else { edge_a_id };
        // VoronoiUtilsCgal.cpp:242 const VD::edge_type &parabolic_edge = edge_a.is_curved() ? edge_a : edge_b;
        let parabolic_edge_id = if edge_a.is_curved() { edge_a_id } else { edge_b_id };
        // VoronoiUtilsCgal.cpp:243 const ParabolicSegment parabolic = get_parabolic_segment(parabolic_edge, segment_begin, segment_end);
        let parabolic = get_parabolic_segment(diagram, parabolic_edge_id, segments);
        // VoronoiUtilsCgal.cpp:244-248
        // orientation = ParabolicTangentToSegmentOrientation{}(to_cgal_point(parabolic.segment.a), to_cgal_point(linear_edge.vertex1()), ...);
        let linear_v1 = &diagram.vertices()[diagram
            .edge_get_vertex1(linear_edge_id)
            .ok()
            .flatten()
            .unwrap()
            .usize()];
        orientation = impl_::parabolic_tangent_to_segment_orientation(
            to_cgal_point_vec2d(parabolic.segment.a),
            to_cgal_point_vertex(linear_v1),
            to_cgal_point_point(&parabolic.focus),
            to_cgal_point_point(&parabolic.directrix.a),
            to_cgal_point_point(&parabolic.directrix.b),
            parabolic.is_focus_on_left,
        );

        // VoronoiUtilsCgal.cpp:250-251 if (edge_b.is_curved()) orientation = CGAL::opposite(orientation);
        if edge_b.is_curved() {
            return impl_::opposite(orientation);
        }
        return orientation;
    }

    // VoronoiUtilsCgal.cpp:254 return orientation;
    orientation
}

// VoronoiUtilsCgal.cpp:257-288
// template<typename SegmentIterator> ... bool
// check_if_three_edges_are_ccw(const VD::edge_type &edge_first, const VD::edge_type &edge_second, const VD::edge_type &edge_third, ...)
fn check_if_three_edges_are_ccw(
    diagram: &bv::Diagram,
    edge_first: bv::EdgeIndex,
    edge_second: bv::EdgeIndex,
    edge_third: bv::EdgeIndex,
    segments: &[Line],
) -> bool {
    // VoronoiUtilsCgal.cpp:268 assert(is_equal(*edge_first.vertex0(), *edge_second.vertex0()) && is_equal(*edge_second.vertex0(), *edge_third.vertex0()));
    debug_assert!({
        let f0 = &diagram.vertices()[diagram.edges()[edge_first.usize()].vertex0().unwrap().usize()];
        let s0 = &diagram.vertices()[diagram.edges()[edge_second.usize()].vertex0().unwrap().usize()];
        let t0 = &diagram.vertices()[diagram.edges()[edge_third.usize()].vertex0().unwrap().usize()];
        is_equal(f0, s0) && is_equal(s0, t0)
    });

    // VoronoiUtilsCgal.cpp:270 CGAL::Orientation orientation = orientation_of_two_edges(edge_first, edge_second, segment_begin, segment_end);
    let orientation = orientation_of_two_edges(diagram, edge_first, edge_second, segments);
    // VoronoiUtilsCgal.cpp:271 if (orientation == CGAL::Orientation::COLLINEAR) {
    if orientation == Orientation::Collinear {
        // VoronoiUtilsCgal.cpp:272-273
        // The first two edges are collinear, so the third edge must be on the right side on the first of them.
        orientation_of_two_edges(diagram, edge_first, edge_third, segments) == Orientation::RightTurn
    } else if orientation == Orientation::LeftTurn {
        // VoronoiUtilsCgal.cpp:274-279
        // CCW oriented angle between vectors (common_pt, pt1) and (common_pt, pt2) is bellow PI.
        // So we need to check if test_pt isn't between them.
        let orientation1 = orientation_of_two_edges(diagram, edge_first, edge_third, segments);
        let orientation2 = orientation_of_two_edges(diagram, edge_second, edge_third, segments);
        orientation1 != Orientation::LeftTurn || orientation2 != Orientation::RightTurn
    } else {
        // VoronoiUtilsCgal.cpp:280-287
        debug_assert!(orientation == Orientation::RightTurn);
        // CCW oriented angle between vectors (common_pt, pt1) and (common_pt, pt2) is upper PI.
        // So we need to check if test_pt is between them.
        let orientation1 = orientation_of_two_edges(diagram, edge_first, edge_third, segments);
        let orientation2 = orientation_of_two_edges(diagram, edge_second, edge_third, segments);
        orientation1 == Orientation::RightTurn || orientation2 == Orientation::LeftTurn
    }
}

/// CGAL-based Voronoi diagram utilities.
///
/// Geometry/VoronoiUtilsCgal.hpp: VoronoiUtilsCgal
pub struct VoronoiUtilsCgal;

impl VoronoiUtilsCgal {
    // VoronoiUtilsCgal.cpp:146-178
    // FIXME Lukas H.: Also includes parabolic segments.
    // bool VoronoiUtilsCgal::is_voronoi_diagram_planar_intersection(const VD &voronoi_diagram)
    //
    // FIDELITY-NOTE(BLOCKED-DEP): native, non-wasm. The C++ body builds CGAL exact
    // segments (`Exact_predicates_exact_constructions_kernel`) and calls
    // `CGAL::compute_intersection_points` (the `Surface_sweep_2` segment-intersection
    // enumeration). No pure-Rust exact segment-sweep equivalent is available in this
    // crate, and CGAL is intentionally not a dependency (wasm policy). This is left as
    // the planar-by-construction default (matching the prior placeholder). The
    // angle-based variant below is the faithful, used-by-Arachne port.
    //
    // Geometry/VoronoiUtilsCgal.hpp: is_voronoi_diagram_planar_intersection
    pub fn is_voronoi_diagram_planar_intersection(_voronoi_diagram: &VoronoiDiagram) -> bool {
        // A valid Voronoi diagram is planar by construction; non-planarity only
        // arises from numerical issues in boost::polygon. Without the CGAL exact
        // sweep we cannot enumerate self-intersections here.
        true
    }

    // VoronoiUtilsCgal.cpp:290-324
    // template<typename SegmentIterator> ... bool
    // VoronoiUtilsCgal::is_voronoi_diagram_planar_angle(const VD &voronoi_diagram, const SegmentIterator segment_begin, const SegmentIterator segment_end)
    //
    // `segments` is the input segment range (== `[segment_begin, segment_end)`),
    // the primary `LinesIt` instantiation used by Arachne / `VoronoiDiagram`.
    pub fn is_voronoi_diagram_planar_angle(
        voronoi_diagram: &VoronoiDiagram,
        segments: &[Line],
    ) -> bool {
        let diagram = voronoi_diagram.diagram();

        // VoronoiUtilsCgal.cpp:299 for (const VD::vertex_type &vertex : voronoi_diagram.vertices()) {
        for vertex_idx in 0..diagram.vertices().len() {
            let vertex = &diagram.vertices()[vertex_idx];
            // VoronoiUtilsCgal.cpp:300 std::vector<const VD::edge_type *> edges;
            let mut edges: Vec<bv::EdgeIndex> = Vec::new();
            // VoronoiUtilsCgal.cpp:301 const VD::edge_type *edge = vertex.incident_edge();
            let incident_edge = match vertex.get_incident_edge() {
                Ok(e) => e,
                Err(_) => continue,
            };
            let mut edge = incident_edge;

            // VoronoiUtilsCgal.cpp:303-308 do { ... } while (edge != vertex.incident_edge());
            loop {
                // VoronoiUtilsCgal.cpp:304-305
                // if (edge->is_finite() && edge->vertex0() != nullptr && edge->vertex1() != nullptr && VoronoiUtils::is_finite(*edge->vertex0()) && VoronoiUtils::is_finite(*edge->vertex1()))
                //     edges.emplace_back(edge);
                let is_finite = diagram.edge_is_finite(edge).unwrap_or(false);
                let v0 = diagram.edge_get_vertex0(edge).ok().flatten();
                let v1 = diagram.edge_get_vertex1(edge).ok().flatten();
                if is_finite {
                    if let (Some(v0i), Some(v1i)) = (v0, v1) {
                        let v0v = &diagram.vertices()[v0i.usize()];
                        let v1v = &diagram.vertices()[v1i.usize()];
                        if crate::geometry::voronoi_utils::is_finite(v0v.x(), v0v.y())
                            && crate::geometry::voronoi_utils::is_finite(v1v.x(), v1v.y())
                        {
                            edges.push(edge);
                        }
                    }
                }

                // VoronoiUtilsCgal.cpp:307 edge = edge->rot_next();
                // C++ `rot_next()` is infallible and always returns a valid edge, so the
                // do-while always closes the incident-edge cycle. boostvoronoi's
                // `edge_rot_next` is `Option`; for a well-formed diagram it returns `Some`,
                // matching C++. The `None` arm is a defensive guard with no C++ analogue.
                edge = match diagram.edge_rot_next(edge) {
                    Some(e) => e,
                    None => break,
                };
                // VoronoiUtilsCgal.cpp:308 } while (edge != vertex.incident_edge());
                if edge == incident_edge {
                    break;
                }
            }

            // VoronoiUtilsCgal.cpp:310-311 Checking for CCW make sense for three and more edges.
            if edges.len() > 2 {
                // VoronoiUtilsCgal.cpp:312 for (auto edge_it = edges.begin(); edge_it != edges.end(); ++edge_it) {
                for i in 0..edges.len() {
                    // VoronoiUtilsCgal.cpp:313 const VD::edge_type *prev_edge = edge_it == edges.begin() ? edges.back() : *std::prev(edge_it);
                    let prev_edge = if i == 0 {
                        edges[edges.len() - 1]
                    } else {
                        edges[i - 1]
                    };
                    // VoronoiUtilsCgal.cpp:314 const VD::edge_type *curr_edge = *edge_it;
                    let curr_edge = edges[i];
                    // VoronoiUtilsCgal.cpp:315 const VD::edge_type *next_edge = std::next(edge_it) == edges.end() ? edges.front() : *std::next(edge_it);
                    let next_edge = if i + 1 == edges.len() {
                        edges[0]
                    } else {
                        edges[i + 1]
                    };

                    // VoronoiUtilsCgal.cpp:317-318
                    // if (!check_if_three_edges_are_ccw(*prev_edge, *curr_edge, *next_edge, segment_begin, segment_end)) return false;
                    if !check_if_three_edges_are_ccw(diagram, prev_edge, curr_edge, next_edge, segments) {
                        return false;
                    }
                }
            }
        }

        // VoronoiUtilsCgal.cpp:323 return true;
        true
    }
}
