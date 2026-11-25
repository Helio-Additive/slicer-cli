//! Geometry/Voronoi.cpp + Voronoi.hpp: `Slic3r::Geometry::VoronoiDiagram`, the pin's wrapper around
//! `boost::polygon::voronoi_diagram<double>` that detects degenerate diagrams
//! (`detect_known_issues`, `has_finite_edge_with_non_finite_vertex`,
//! `detect_known_voronoi_cell_issues`) and repairs them by rotation
//! (`try_to_repair_degenerated_voronoi_diagram*`), over `boost_polygon_voronoi::VoronoiDiagram`, the
//! port of boost::polygon's builder (vertices, edges and cells in boost's container order, doubles
//! equal to boost's).
//!
//! This is the type `geometry::voronoi_diagram::boost_polygon::VoronoiDiagram` names (moved here in
//! R954 so that the C++ names sit in the file paired with Voronoi.cpp; the old path re-exports it).
//! The older wrapper around the `boostvoronoi` crate stays `geometry::voronoi_diagram::VoronoiDiagram`.
//! Checked against the pin by tests/voronoi_native_parity.rs.

pub use crate::geometry::voronoi_diagram::VoronoiDiagram;

/// The diagram backed by the Rust port of Boost.Polygon.
pub mod boost_polygon {
    use crate::geometry::voronoi_diagram::{IssueType, RepairState as State};
    use crate::boost_polygon_voronoi as bpv;
    use crate::geometry::voronoi_utils::{
        compute_segment_cell_range, get_source_segment, VoronoiSegment, VoronoiUtils,
    };
    use crate::geometry::{Line, Point};
    use crate::Coord;

    /// Voronoi.hpp:167-174 `VoronoiDiagram::Segment`: a rotated copy of an input segment.
    /// `segment_traits<Segment>::get` (Voronoi.hpp:191-198) gives `from` for LOW and `to`
    /// for HIGH, which is what `VoronoiSegment for Line` gives for `a` and `b`.
    type Segment = Line;

    /// Voronoi.hpp:20-158 `class VoronoiDiagram`.
    #[derive(Clone, Debug)]
    pub struct VoronoiDiagram {
        /// Voronoi.hpp:150 `voronoi_diagram_type m_voronoi_diagram`.
        m_voronoi_diagram: bpv::VoronoiDiagram,
        /// Voronoi.hpp:151-153 `m_vertices`, `m_edges`, `m_cells`: the diagram of the
        /// rotated input, with its vertices rotated back. The three containers are kept
        /// as one diagram, whose edges name cells, vertices and other edges by index as
        /// the C++ ones do by pointer into these same containers.
        m_local: bpv::VoronoiDiagram,
        /// Voronoi.hpp:154 `bool m_is_modified = false`.
        m_is_modified: bool,
        /// Voronoi.hpp:155 `State m_state = State::UNKNOWN`.
        m_state: State,
        /// Voronoi.hpp:156 `IssueType m_issue_type = IssueType::UNKNOWN`.
        m_issue_type: IssueType,
    }

    impl Default for VoronoiDiagram {
        fn default() -> Self {
            Self::new()
        }
    }

    impl VoronoiDiagram {
        /// Voronoi.hpp:55 `VoronoiDiagram() = default`.
        pub fn new() -> Self {
            VoronoiDiagram {
                m_voronoi_diagram: bpv::VoronoiDiagram::new(),
                m_local: bpv::VoronoiDiagram::new(),
                m_is_modified: false,
                m_state: State::Unknown,
                m_issue_type: IssueType::Unknown,
            }
        }

        /// Voronoi.hpp:59 `get_issue_type()`.
        pub fn get_issue_type(&self) -> IssueType {
            self.m_issue_type
        }

        /// Voronoi.hpp:61 `get_state()`.
        pub fn get_state(&self) -> State {
            self.m_state
        }

        /// Voronoi.hpp:63 `is_valid()`.
        pub fn is_valid(&self) -> bool {
            self.m_state != State::RepairUnsuccessful
        }

        /// Voronoi.cpp:77-90 `VoronoiDiagram::clear()`.
        pub fn clear(&mut self) {
            if self.m_is_modified {
                self.m_local.clear();
                self.m_is_modified = false;
            } else {
                self.m_voronoi_diagram.clear();
            }

            self.m_state = State::Unknown;
            self.m_issue_type = IssueType::Unknown;
        }

        /// Voronoi.hpp:67-77 `vertices()`, `edges()`, `cells()`, `num_*()`: the local
        /// containers once the diagram was modified, boost's diagram otherwise.
        pub fn diagram(&self) -> &bpv::VoronoiDiagram {
            if self.m_is_modified {
                &self.m_local
            } else {
                &self.m_voronoi_diagram
            }
        }

        /// The same containers for the callers that colour vertices and edges: `color` is
        /// a `mutable` member in boost (voronoi_diagram.hpp:121-125, 236-240), which the
        /// C++ sets through the `const` references `vertices()` and `edges()` give.
        pub fn diagram_mut(&mut self) -> &mut bpv::VoronoiDiagram {
            if self.m_is_modified {
                &mut self.m_local
            } else {
                &mut self.m_voronoi_diagram
            }
        }

        /// Voronoi.hpp:77 `num_vertices()`: `m_is_modified ? m_vertices.size() : m_voronoi_diagram.num_vertices()`.
        pub fn num_vertices(&self) -> usize {
            self.diagram().num_vertices()
        }

        /// Voronoi.hpp:79 `num_edges()`.
        pub fn num_edges(&self) -> usize {
            self.diagram().num_edges()
        }

        /// Voronoi.hpp:81 `num_cells()`.
        pub fn num_cells(&self) -> usize {
            self.diagram().num_cells()
        }

        /// `boost::polygon::construct_voronoi(segment_begin, segment_end, &vd)`
        /// (voronoi.hpp:98-104 → `insert`, voronoi.hpp:63-75): every segment goes in as
        /// `insert_segment(x(low), y(low), x(high), y(high))`, whose parameters are `int32`
        /// (voronoi_builder.hpp:83), so the `coord_t` ones are converted as C++ converts them.
        fn boost_construct_voronoi<S: VoronoiSegment>(segments: &[S], vd: &mut bpv::VoronoiDiagram) {
            let input: Vec<[i32; 4]> = segments
                .iter()
                .map(|s| {
                    let (low, high) = (s.low(), s.high());
                    [low.x as i32, low.y as i32, high.x as i32, high.y as i32]
                })
                .collect();
            bpv::construct_voronoi(&input, vd);
        }

        /// Voronoi.cpp:20-75 `VoronoiDiagram::construct_voronoi(segment_begin, segment_end,
        /// try_to_repair_if_needed, throw_on_cancel)`. `try_to_repair_if_needed` is `true`
        /// where the C++ caller leaves it out (Voronoi.hpp:84).
        pub fn construct_voronoi<S: VoronoiSegment>(&mut self, segments: &[S], try_to_repair_if_needed: bool) {
            // Voronoi.cpp:28 boost::polygon::construct_voronoi(segment_begin, segment_end, &m_voronoi_diagram);
            Self::boost_construct_voronoi(segments, &mut self.m_voronoi_diagram);
            // Voronoi.cpp:30-70
            if try_to_repair_if_needed {
                self.m_issue_type = Self::detect_known_issues(self, segments);
                if self.m_issue_type != IssueType::NoIssue {
                    // Voronoi.cpp:32-42
                    match self.m_issue_type {
                        IssueType::MissingVoronoiVertex => {
                            log::warn!("Detected missing Voronoi vertex, input polygons will be rotated back and forth.")
                        }
                        IssueType::NonPlanarDiagram => {
                            log::warn!("Detected non-planar Voronoi diagram, input polygons will be rotated back and forth.")
                        }
                        IssueType::EdgeIntersectingInputSegment => log::warn!(
                            "Detected Voronoi edge intersecting input segment, input polygons will be rotated back and forth."
                        ),
                        IssueType::FiniteEdgeWithNonFiniteVertex => log::warn!(
                            "Detected finite Voronoi vertex with non finite vertex, input polygons will be rotated back and forth."
                        ),
                        _ => log::error!("Detected unknown Voronoi diagram issue, input polygons will be rotated back and forth."),
                    }

                    // Voronoi.cpp:44-60
                    self.m_issue_type = self.try_to_repair_degenerated_voronoi_diagram(segments);
                    if self.m_issue_type != IssueType::NoIssue {
                        match self.m_issue_type {
                            IssueType::MissingVoronoiVertex => {
                                log::error!("Detected missing Voronoi vertex even after the rotation of input.")
                            }
                            IssueType::NonPlanarDiagram => {
                                log::error!("Detected non-planar Voronoi diagram even after the rotation of input.")
                            }
                            IssueType::EdgeIntersectingInputSegment => {
                                log::error!("Detected Voronoi edge intersecting input segment even after the rotation of input.")
                            }
                            IssueType::FiniteEdgeWithNonFiniteVertex => {
                                log::error!("Detected finite Voronoi vertex with non finite vertex even after the rotation of input.")
                            }
                            _ => log::error!("Detected unknown Voronoi diagram issue even after the rotation of input."),
                        }

                        self.m_state = State::RepairUnsuccessful;
                    } else {
                        self.m_state = State::RepairSuccessful;
                    }
                } else {
                    // Voronoi.cpp:62-64
                    self.m_state = State::RepairNotNeeded;
                    self.m_issue_type = IssueType::NoIssue;
                }
            } else {
                // Voronoi.cpp:66-69
                self.m_state = State::Unknown;
                self.m_issue_type = IssueType::Unknown;
            }
        }

        /// Voronoi.cpp:92-150 `VoronoiDiagram::copy_to_local(voronoi_diagram)`: the edges,
        /// cells and vertices of `voronoi_diagram` in their order, each with its colour, and
        /// every pointer re-aimed at the element of the same index. With indices for
        /// pointers that is a copy of the diagram.
        fn copy_to_local(&mut self, voronoi_diagram: &bpv::VoronoiDiagram) {
            self.m_local = voronoi_diagram.clone();
        }

        /// Voronoi.cpp:152-172 `VoronoiDiagram::detect_known_issues(voronoi_diagram,
        /// segment_begin, segment_end)`. It reads `voronoi_diagram.edges()` and `.cells()`,
        /// which are boost's diagram until `m_is_modified` is set and the local copy after.
        pub fn detect_known_issues<S: VoronoiSegment>(voronoi_diagram: &VoronoiDiagram, segments: &[S]) -> IssueType {
            // Voronoi.cpp:159-160
            if Self::has_finite_edge_with_non_finite_vertex(voronoi_diagram) {
                return IssueType::FiniteEdgeWithNonFiniteVertex;
            }
            // Voronoi.cpp:161-163
            let cell_issue_type = Self::detect_known_voronoi_cell_issues(voronoi_diagram, segments);
            if cell_issue_type != IssueType::NoIssue {
                return cell_issue_type;
            }
            // Voronoi.cpp:164-168 "BBS: test no problem in BBS": the check of
            // VoronoiUtilsCgal::is_voronoi_diagram_planar_angle is commented out at the pin.

            // Voronoi.cpp:170
            IssueType::NoIssue
        }

        /// Voronoi.cpp:174-216 `VoronoiDiagram::detect_known_voronoi_cell_issues(voronoi_diagram,
        /// segment_begin, segment_end)`.
        fn detect_known_voronoi_cell_issues<S: VoronoiSegment>(voronoi_diagram: &VoronoiDiagram, segments: &[S]) -> IssueType {
            let vd = voronoi_diagram.diagram();
            // Voronoi.cpp:186 for (VD::cell_type cell : voronoi_diagram.cells())
            for cell in 0..vd.num_cells() {
                // Voronoi.cpp:187-188 Skip degenerated cell that has no spoon. Also, skip a
                // cell that doesn't contain a segment.
                if vd.cells()[cell].is_degenerate() || !vd.cells()[cell].contains_segment() {
                    continue;
                }

                // Voronoi.cpp:190
                let cell_range = compute_segment_cell_range(vd, cell, segments);
                if cell_range.is_valid() {
                    // Voronoi.cpp:191-197 Detection if Voronoi edge is intersecting input segment.
                    let source_segment = get_source_segment(vd, cell, segments);
                    let low = source_segment.low();
                    let high = source_segment.high();
                    let source_segment_from = (low.x as f64, low.y as f64);
                    let source_segment_to = (high.x as f64, high.y as f64);
                    let source_segment_vec =
                        (source_segment_to.0 - source_segment_from.0, source_segment_to.1 - source_segment_from.1);

                    // Voronoi.cpp:199-208 All Voronoi vertices must be on the left side of the
                    // source segment, otherwise the Voronoi diagram is invalid.
                    let mut edge = cell_range.edge_begin.unwrap();
                    let edge_end = cell_range.edge_end.unwrap();
                    while edge != edge_end {
                        if vd.is_infinite(edge) {
                            // Voronoi.cpp:201-204 When there is a missing Voronoi vertex, we may
                            // encounter an infinite Voronoi edge.
                            return IssueType::MissingVoronoiVertex;
                        }
                        // Voronoi.cpp:205 const Vec2d edge_v1(edge->vertex1()->x(), edge->vertex1()->y());
                        let v1 = vd.vertex1(edge).unwrap();
                        let edge_v1 = (vd.vertices()[v1].x(), vd.vertices()[v1].y());
                        // Voronoi.cpp:205 Slic3r::cross2(source_segment_vec, edge_v1 - source_segment_from) < 0.
                        // `cross2` (Point.hpp:91-96) is `v1.x() * v2.y() - v1.y() * v2.x()`, one
                        // scalar expression: contracted by the reference build, the left product
                        // fused and the right one rounded first.
                        let v2 = (edge_v1.0 - source_segment_from.0, edge_v1.1 - source_segment_from.1);
                        if source_segment_vec.0.mul_add(v2.1, -(source_segment_vec.1 * v2.0)) < 0.0 {
                            return IssueType::EdgeIntersectingInputSegment;
                        }
                        edge = vd.edges()[edge].next().unwrap();
                    }
                } else {
                    // Voronoi.cpp:210-213 When there is a missing Voronoi vertex (especially at
                    // one of the endpoints of the input segment), the returned cell_range is
                    // marked as invalid.
                    return IssueType::MissingVoronoiVertex;
                }
            }

            // Voronoi.cpp:215
            IssueType::NoIssue
        }

        /// Voronoi.cpp:218-228 `VoronoiDiagram::has_finite_edge_with_non_finite_vertex(voronoi_diagram)`.
        fn has_finite_edge_with_non_finite_vertex(voronoi_diagram: &VoronoiDiagram) -> bool {
            let vd = voronoi_diagram.diagram();
            for edge in 0..vd.num_edges() {
                if vd.is_finite(edge) {
                    // Voronoi.cpp:222-224
                    let (v0, v1) = (vd.edges()[edge].vertex0(), vd.vertex1(edge));
                    match (v0, v1) {
                        (Some(v0), Some(v1)) => {
                            let (v0, v1) = (&vd.vertices()[v0], &vd.vertices()[v1]);
                            if !VoronoiUtils::is_finite(v0.x(), v0.y()) || !VoronoiUtils::is_finite(v1.x(), v1.y()) {
                                return true;
                            }
                        }
                        _ => return true,
                    }
                }
            }
            false
        }

        /// Voronoi.cpp:230-251 `VoronoiDiagram::try_to_repair_degenerated_voronoi_diagram(segment_begin,
        /// segment_end, throw_on_cancel)`.
        pub fn try_to_repair_degenerated_voronoi_diagram<S: VoronoiSegment>(&mut self, segments: &[S]) -> IssueType {
            // Voronoi.cpp:239
            let mut issue_type = self.m_issue_type;

            // Voronoi.cpp:241 const std::vector<double> fix_angles = {PI / 6, PI / 5, PI / 7, PI / 11};
            // `PI` is libslic3r.h:59 `3.141592653589793238`, the double `std::f64::consts::PI` is.
            const PI: f64 = std::f64::consts::PI;
            let fix_angles = [PI / 6.0, PI / 5.0, PI / 7.0, PI / 11.0];
            for fix_angle in fix_angles {
                // Voronoi.cpp:244-247
                issue_type = self.try_to_repair_degenerated_voronoi_diagram_by_rotation(segments, fix_angle);
                if issue_type == IssueType::NoIssue {
                    return issue_type;
                }
            }

            // Voronoi.cpp:250
            issue_type
        }

        /// Point.hpp:225-231, 234 `Point::rotated(double angle)`: `rotate(cos(angle), sin(angle))`
        /// with `x = (coord_t) round(cos_a * cur_x - sin_a * cur_y)` and
        /// `y = (coord_t) round(cos_a * cur_y + sin_a * cur_x)`, each one source expression
        /// that the reference build contracts (left product fused, right one rounded first).
        /// `Point::rotate` of this crate (geometry/point.rs) does not fuse and has other
        /// callers, so the arithmetic is written out here.
        fn point_rotated(p: Point, angle: f64) -> Point {
            let cos_a = angle.cos();
            let sin_a = angle.sin();
            let cur_x = p.x as f64;
            let cur_y = p.y as f64;
            Point::new(
                cos_a.mul_add(cur_x, -(sin_a * cur_y)).round() as Coord,
                cos_a.mul_add(cur_y, sin_a * cur_x).round() as Coord,
            )
        }

        /// Voronoi.cpp:276-361 `VoronoiDiagram::try_to_repair_degenerated_voronoi_diagram_by_rotation(segment_begin,
        /// segment_end, fix_angle, throw_on_cancel)`.
        pub fn try_to_repair_degenerated_voronoi_diagram_by_rotation<S: VoronoiSegment>(
            &mut self,
            segments: &[S],
            fix_angle: f64,
        ) -> IssueType {
            // Voronoi.cpp:289-296 Copy all segments and rotate their vertices.
            let mut segments_rotated: Vec<Segment> = Vec::with_capacity(segments.len());
            for segment in segments {
                let from = segment.low();
                let to = segment.high();
                segments_rotated.push(Line::new(Self::point_rotated(from, fix_angle), Self::point_rotated(to, fix_angle)));
            }

            // Voronoi.cpp:299-300
            let mut voronoi_diagram_rotated = bpv::VoronoiDiagram::new();
            Self::boost_construct_voronoi(&segments_rotated, &mut voronoi_diagram_rotated);

            // Voronoi.cpp:303-304
            self.copy_to_local(&voronoi_diagram_rotated);
            // `detect_known_issues(*this, ...)` reads `this->edges()` and `this->cells()`.
            // On the first angle `m_is_modified` is still false, so what it examines against
            // the rotated segments is boost's diagram of the input as it was, not the copy
            // made one line above; from the second angle on it is the copy of the rotated
            // diagram. `diagram()` gives the same container the C++ accessors give.
            // Measured on the one diagram of the `nu3mf` fixture that needs the repair (a
            // multi-material layer of 1,047 segments, MISSING_VORONOI_VERTEX): as written
            // here the first angle, PI/6, reports the issue again and PI/5 repairs it, and
            // the G-code gains 1,543 lines verbatim on the oracle's; examining the copy on
            // the first angle accepts PI/6 and loses 25.
            let issue_type = Self::detect_known_issues(self, &segments_rotated);

            // Voronoi.cpp:306-333 We want to remap all Voronoi vertices at the endpoints of
            // input segments to ensure that Voronoi vertices at endpoints will be preserved
            // after rotation. So we assign every Voronoi vertices color to map this Vertex
            // into input segments.
            for cell in 0..self.m_local.num_cells() {
                // Voronoi.cpp:310-311
                if self.m_local.cells()[cell].is_degenerate() {
                    continue;
                }

                // Voronoi.cpp:313-331
                if self.m_local.cells()[cell].contains_segment() {
                    let cell_range = compute_segment_cell_range(&self.m_local, cell, &segments_rotated);
                    if cell_range.is_valid() {
                        let source_index = self.m_local.cells()[cell].source_index();
                        // Voronoi.cpp:315-319 Vertex 1 of edge_end points to the starting
                        // endpoint of the input segment (from() or line.a).
                        let end_v1 = self.m_local.vertex1(cell_range.edge_end.unwrap()).unwrap();
                        if self.m_local.vertices()[end_v1].color() == 0 {
                            let color = encode_input_segment_endpoint(source_index, false);
                            self.m_local.set_vertex_color(end_v1, color);
                        }

                        // Voronoi.cpp:321-325 Vertex 0 of edge_begin points to the ending
                        // endpoint of the input segment (to() or line.b).
                        let begin_v0 = self.m_local.edges()[cell_range.edge_begin.unwrap()].vertex0().unwrap();
                        if self.m_local.vertices()[begin_v0].color() == 0 {
                            let color = encode_input_segment_endpoint(source_index, true);
                            self.m_local.set_vertex_color(begin_v0, color);
                        }
                    }
                    // Voronoi.cpp:326-329 else: "This could happen when there is a missing
                    // Voronoi vertex even after rotation." (an assert, nothing else)
                }

                // Voronoi.cpp:332 FIXME @hejllukas: Implement mapping also for source points
                // and not just for source segments.
            }

            // Voronoi.cpp:335-349 Rotate all Voronoi vertices back. When a Voronoi vertex can
            // be mapped to the input segment endpoint, then we don't need to do rotation back.
            for vertex in 0..self.m_local.num_vertices() {
                let v = self.m_local.vertices()[vertex];
                if v.color() == 0 {
                    // Voronoi.cpp:338-340 This vertex isn't mapped to any vertex, so we rotate it back.
                    let (x, y) = VoronoiUtils::make_rotated_vertex(v.x(), v.y(), -fix_angle);
                    self.m_local.set_vertex_coordinates(vertex, x, y);
                } else {
                    // Voronoi.cpp:341-347 This vertex can be mapped to the input segment endpoint.
                    let endpoint = decode_input_segment_endpoint(v.color(), segments);
                    self.m_local.set_vertex_coordinates(vertex, endpoint.x as f64, endpoint.y as f64);
                }
            }

            // Voronoi.cpp:351-354 We have to clear all marked vertices because some
            // algorithms expect that all vertices have a color equal to 0.
            for vertex in 0..self.m_local.num_vertices() {
                self.m_local.set_vertex_color(vertex, 0);
            }

            // Voronoi.cpp:356-357
            self.m_voronoi_diagram.clear();
            self.m_is_modified = true;

            // Voronoi.cpp:359
            issue_type
        }
    }

    /// Voronoi.cpp:256-259 `encode_input_segment_endpoint(cell_source_index, dir)` (a free inline function
    /// at the pin); `high` is `dir.to_int() != 0`.
    #[inline]
    pub fn encode_input_segment_endpoint(cell_source_index: usize, high: bool) -> usize {
        (cell_source_index + 1) << 1 | usize::from(high)
    }

    /// Voronoi.cpp:261-274 `decode_input_segment_endpoint(color, segment_begin, segment_end)` (a free inline
    /// function at the pin).
    #[inline]
    pub fn decode_input_segment_endpoint<S: VoronoiSegment>(color: usize, segments: &[S]) -> Point {
        let segment_idx = (color >> 1) - 1;
        let segment = &segments[segment_idx];
        if color & 1 != 0 {
            segment.high()
        } else {
            segment.low()
        }
    }

    /// Voronoi.hpp:86-102 `construct_voronoi(first, last)` over input points: boost's
    /// `construct_voronoi(first, last, &m_voronoi_diagram)` (every point inserted as
    /// `insert_point(x(p), y(p))`, whose parameters are `int32`), then state and issue type `UNKNOWN`.
    /// `m_is_modified` is left as it is, as at the pin.
    pub trait ConstructVoronoiPoints {
        fn construct_voronoi(&mut self, points: &[Point]);
    }

    impl ConstructVoronoiPoints for VoronoiDiagram {
        fn construct_voronoi(&mut self, points: &[Point]) {
            let input: Vec<[i32; 2]> = points.iter().map(|p| [p.x as i32, p.y as i32]).collect();
            bpv::construct_voronoi_points(&input, &mut self.m_voronoi_diagram);
            self.m_state = State::Unknown;
            self.m_issue_type = IssueType::Unknown;
        }
    }

    /// Voronoi.hpp:104-116 `construct_voronoi(p_first, p_last, s_first, s_last)` over input points and
    /// segments: boost's `construct_voronoi(p_first, p_last, s_first, s_last, &m_voronoi_diagram)`, then
    /// state and issue type `UNKNOWN`.
    pub trait ConstructVoronoiPointsSegments {
        fn construct_voronoi<S: VoronoiSegment>(&mut self, points: &[Point], segments: &[S]);
    }

    impl ConstructVoronoiPointsSegments for VoronoiDiagram {
        fn construct_voronoi<S: VoronoiSegment>(&mut self, points: &[Point], segments: &[S]) {
            let pts: Vec<[i32; 2]> = points.iter().map(|p| [p.x as i32, p.y as i32]).collect();
            let segs: Vec<[i32; 4]> = segments
                .iter()
                .map(|s| {
                    let (low, high) = (s.low(), s.high());
                    [low.x as i32, low.y as i32, high.x as i32, high.y as i32]
                })
                .collect();
            bpv::construct_voronoi_points_segments(&pts, &segs, &mut self.m_voronoi_diagram);
            self.m_state = State::Unknown;
            self.m_issue_type = IssueType::Unknown;
        }
    }

    /// Voronoi.hpp:189-198 `boost::polygon::segment_traits<VoronoiDiagram::Segment>`.
    pub struct SegmentTraits;

    impl SegmentTraits {
        /// Voronoi.hpp:196 `get(segment, dir)`: `dir.to_int() ? segment.to : segment.from`; `high` is
        /// `dir.to_int() != 0`.
        pub fn get(segment: &Line, high: bool) -> Point {
            if high {
                segment.b
            } else {
                segment.a
            }
        }
    }
}
