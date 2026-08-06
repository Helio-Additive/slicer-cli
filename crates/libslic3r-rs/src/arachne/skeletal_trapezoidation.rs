//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.
//
//! Faithful 1:1 port of Arachne/SkeletalTrapezoidation.{hpp,cpp}
//!
//! C++ Reference:
//! - Arachne/SkeletalTrapezoidation.hpp
//! - Arachne/SkeletalTrapezoidation.cpp
//!
//! Main class of the dynamic beading strategies.
//!
//! The input polygon region is decomposed into trapezoids and represented as a
//! half-edge data-structure. We determine which edges are 'central' according to
//! the transitioning_angle of the beading strategy, and determine the bead count
//! for these central regions and apply them outward when generating toolpaths.
//!
//! Pointer model: like `skeletal_trapezoidation_graph.rs`, the C++ `edge_t*` /
//! `node_t*` raw cross-references are kept as `NonNull<HalfEdge<..>>` /
//! `NonNull<HalfEdgeNode<..>>` into the pointer-stable graph lists, and the
//! traversal logic is replayed verbatim under `unsafe`.
//!
//! BLOCKED (needs the boost::polygon Voronoi pointer-traversal layer, which the
//! crate's `boostvoronoi`-backed `VoronoiDiagram`/`VoronoiUtils` do not expose):
//! - `construct_from_polygons`
//! - `transfer_edge`
//! - `discretize`
//! - `compute_point_cell_range`
//! - `make_node`
//! Everything downstream of `construct_from_polygons` (transitioning + toolpath
//! generation, plus `separate_pointy_quad_end_nodes`) operates purely on the
//! already-built `graph` + `beading_strategy` and is ported faithfully.

use crate::arachne::beading_strategy::beading_strategy::{Beading, BeadingStrategy};
use crate::arachne::skeletal_trapezoidation_edge::{
    EdgeType, TransitionEnd, TransitionMiddle,
};
use crate::arachne::skeletal_trapezoidation_graph::{
    as_st_edge, as_st_node, EdgeData, EdgePtr, NodeData, NodePtr, SkeletalTrapezoidationGraph,
};
use crate::arachne::skeletal_trapezoidation_joint::BeadingPropagation;
use crate::arachne::utils::extrusion_junction::{ExtrusionJunction, LineJunctions};
use crate::arachne::utils::extrusion_line::{ExtrusionLine, VariableWidthLines};
use crate::arachne::utils::half_edge::HalfEdge;
use crate::arachne::utils::half_edge_node::HalfEdgeNode;
use crate::arachne::skeletal_trapezoidation_graph::STHalfEdgeNode;
use crate::arachne::utils::linear_alg2d::is_inside_corner;
use crate::arachne::utils::polygons_point_index::PolygonsPointIndex;
use crate::arachne::utils::polygons_segment_index::{Direction1d, PolygonsSegmentIndex};
use crate::geometry::voronoi_diagram::VoronoiDiagram;
use crate::geometry::{perp, shorter_then, Line, Point, Polygons};
use crate::{scaled, Coord};
use boostvoronoi::prelude as bv;
use parking_lot::RwLock;
use std::collections::BinaryHeap;
use std::sync::Arc;

// SkeletalTrapezoidation.cpp:22
// #define SKELETAL_TRAPEZOIDATION_BEAD_SEARCH_MAX 1000 //A limit to how long it'll
// keep searching for adjacent beads.
const SKELETAL_TRAPEZOIDATION_BEAD_SEARCH_MAX: Coord = 1000;

/// `scaled<coord_t>(mm)` — multiply by SCALING_FACTOR and round.
/// `crate::scaled` is not a `const fn`, so the `static constexpr` members below are
/// expressed as `const fn` for use in `const`-contexts.
const fn scaled_c(mm: f64) -> Coord {
    // crate::SCALING_FACTOR == 100_000.0; round to nearest.
    (mm * crate::SCALING_FACTOR + 0.5) as Coord
}

// SkeletalTrapezoidation.hpp:70 static constexpr coord_t central_filter_dist = scaled<coord_t>(0.02);
const CENTRAL_FILTER_DIST: Coord = scaled_c(0.02);
// SkeletalTrapezoidation.hpp:71 static constexpr coord_t snap_dist = scaled<coord_t>(0.02);
const SNAP_DIST: Coord = scaled_c(0.02);

// SkeletalTrapezoidation.hpp:145-153 struct TransitionMidRef
//
// References one transition along an edge which may contain multiple transitions.
// In C++ this holds `edge_t* edge` and a `std::list<TransitionMiddle>::iterator`.
// Here, the transition list is an `Arc<RwLock<Vec<TransitionMiddle>>>` reached via
// the edge's `getTransitions()`, so we hold the edge pointer plus the *index* into
// that vector (the analogue of the list iterator that the dissolve code erases).
#[derive(Clone)]
pub struct TransitionMidRef {
    // SkeletalTrapezoidation.hpp:147 edge_t* edge;
    pub edge: EdgePtr,
    // SkeletalTrapezoidation.hpp:148 std::list<TransitionMiddle>::iterator transition_it;
    pub transition_idx: usize,
}

impl TransitionMidRef {
    // SkeletalTrapezoidation.hpp:149-152
    pub fn new(edge: EdgePtr, transition_idx: usize) -> Self {
        Self {
            edge,
            transition_idx,
        }
    }
}

// SkeletalTrapezoidation.hpp:50 class SkeletalTrapezoidation
pub struct SkeletalTrapezoidation<'a> {
    // SkeletalTrapezoidation.hpp:63 bool enable_hole_compensation;
    pub enable_hole_compensation: bool,
    // SkeletalTrapezoidation.hpp:64 std::vector<int> hole_indices;
    pub hole_indices: Vec<i32>,
    // SkeletalTrapezoidation.hpp:65 double transitioning_angle;
    pub transitioning_angle: f64,
    // SkeletalTrapezoidation.hpp:66 coord_t discretization_step_size;
    pub discretization_step_size: Coord,
    // SkeletalTrapezoidation.hpp:67 coord_t transition_filter_dist;
    pub transition_filter_dist: Coord,
    // SkeletalTrapezoidation.hpp:68 coord_t allowed_filter_deviation;
    pub allowed_filter_deviation: Coord,
    // SkeletalTrapezoidation.hpp:69 coord_t beading_propagation_transition_dist;
    pub beading_propagation_transition_dist: Coord,

    // SkeletalTrapezoidation.hpp:81 const BeadingStrategy& beading_strategy;
    pub beading_strategy: &'a dyn BeadingStrategy,

    // SkeletalTrapezoidation.hpp:125 graph_t graph;
    pub graph: SkeletalTrapezoidationGraph,

    // SkeletalTrapezoidation.hpp:181 std::vector<VariableWidthLines>* p_generated_toolpaths;
    // Stored as a raw pointer to mirror the C++ member that aliases the caller's output vector.
    p_generated_toolpaths: *mut Vec<VariableWidthLines>,

    // SkeletalTrapezoidation.hpp:158 std::unordered_map<vd_t::vertex_type *, node_t *> vd_node_to_he_node;
    // SkeletalTrapezoidation.hpp:159 std::unordered_map<vd_t::edge_type *, edge_t *> vd_edge_to_he_edge;
    //
    // The C++ keys are boost::polygon VD pointers; here the pointer-stable analogue
    // is the `boostvoronoi` index (`VertexIndex`/`EdgeIndex`) into the diagram's
    // vertex/edge lists. Populated by `construct_from_polygons` (the graph builder).
    vd_node_to_he_node: std::collections::HashMap<bv::VertexIndex, NodePtr>,
    vd_edge_to_he_edge: std::collections::HashMap<bv::EdgeIndex, EdgePtr>,
}

impl<'a> SkeletalTrapezoidation<'a> {
    // SkeletalTrapezoidation.cpp:374-389
    // SkeletalTrapezoidation::SkeletalTrapezoidation(const Polygons& polys, const BeadingStrategy& beading_strategy, ...)
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        beading_strategy: &'a dyn BeadingStrategy,
        transitioning_angle: f64,
        discretization_step_size: Coord,
        transition_filter_dist: Coord,
        allowed_filter_deviation: Coord,
        beading_propagation_transition_dist: Coord,
        enable_hole_compensation: bool,
        hole_indices: Vec<i32>,
    ) -> Self {
        // NOTE: the C++ constructor body calls `constructFromPolygons(polys)`, which is
        // blocked on the boost VD layer (see module docs). The caller must build
        // `graph` via the (not-yet-ported) VD path before invoking `generate_toolpaths`.
        Self {
            // SkeletalTrapezoidation.cpp:379
            transitioning_angle,
            // SkeletalTrapezoidation.cpp:380
            discretization_step_size,
            // SkeletalTrapezoidation.cpp:381
            transition_filter_dist,
            // SkeletalTrapezoidation.cpp:382
            allowed_filter_deviation,
            // SkeletalTrapezoidation.cpp:383
            beading_propagation_transition_dist,
            // SkeletalTrapezoidation.cpp:384
            beading_strategy,
            // SkeletalTrapezoidation.cpp:385
            enable_hole_compensation,
            // SkeletalTrapezoidation.cpp:386
            hole_indices,
            graph: SkeletalTrapezoidationGraph::new(),
            p_generated_toolpaths: std::ptr::null_mut(),
            vd_node_to_he_node: std::collections::HashMap::new(),
            vd_edge_to_he_edge: std::collections::HashMap::new(),
        }
    }

    // =====================================================================
    //    GRAPH CONSTRUCTION (VD -> half-edge graph)
    //
    // Faithful 1:1 port of SkeletalTrapezoidation.cpp:92-504 (makeNode,
    // transferEdge, discretize, computePointCellRange, constructFromPolygons),
    // previously blocked. The C++ keys its `vd_*_to_he_*` maps on boost::polygon
    // VD pointers; here the pointer-stable analogue is the `boostvoronoi` index
    // (`bv::VertexIndex` / `bv::EdgeIndex`) into the diagram's vertex/edge lists.
    // The VD navigation (`twin`/`next`/`prev`/`vertex0`/`vertex1`/`cell`) all maps
    // to the index-based `bv::Diagram` API (see voronoi_utils_cgal.rs for the
    // same navigation pattern). The half-edge graph mutators (`make_node`,
    // `graph.make_rib`, `graph.nodes/edges` allocation) are unchanged from C++.
    // =====================================================================

    // SkeletalTrapezoidation.cpp:91-105
    // SkeletalTrapezoidation::node_t &SkeletalTrapezoidation::makeNode(const VD::vertex_type &vd_node, Point p)
    //
    // `vd_node` is identified by its `bv::VertexIndex` (the map key).
    fn make_node(&mut self, vd_node: bv::VertexIndex, p: Point) -> NodePtr {
        // SkeletalTrapezoidation.cpp:93
        if let Some(&node) = self.vd_node_to_he_node.get(&vd_node) {
            // SkeletalTrapezoidation.cpp:103 return *he_node_it->second;
            node
        } else {
            // SkeletalTrapezoidation.cpp:96 graph.nodes.emplace_front(SkeletalTrapezoidationJoint(), p);
            self.graph
                .nodes
                .push_front(Box::new(STHalfEdgeNode::new(crate::arachne::skeletal_trapezoidation_joint::SkeletalTrapezoidationJoint::new(), p)));
            // SkeletalTrapezoidation.cpp:97 node_t& node = graph.nodes.front();
            let node = SkeletalTrapezoidationGraph::node_ptr(self.graph.nodes.front().unwrap());
            // SkeletalTrapezoidation.cpp:98 vd_node_to_he_node.emplace(&vd_node, &node);
            self.vd_node_to_he_node.insert(vd_node, node);
            // SkeletalTrapezoidation.cpp:99 return node;
            node
        }
    }

    // SkeletalTrapezoidation.cpp:107-217
    // void SkeletalTrapezoidation::transferEdge(Point from, Point to, const VD::edge_type &vd_edge, edge_t *&prev_edge, Point &start_source_point, Point &end_source_point, const std::vector<Segment> &segments, const bool hole_compensation_flag)
    #[allow(clippy::too_many_arguments)]
    fn transfer_edge(
        &mut self,
        diagram: &bv::Diagram,
        from: Point,
        to: Point,
        vd_edge: bv::EdgeIndex,
        prev_edge: &mut Option<EdgePtr>,
        start_source_point: Point,
        end_source_point: Point,
        segments: &[PolygonsSegmentIndex],
        hole_compensation_flag: bool,
    ) {
        // SkeletalTrapezoidation.cpp:108 auto he_edge_it = vd_edge_to_he_edge.find(vd_edge.twin());
        let vd_twin = diagram.edges()[vd_edge.usize()].twin().unwrap();
        let he_edge = self.vd_edge_to_he_edge.get(&vd_twin).copied();
        if let Some(source_twin) = he_edge {
            // SkeletalTrapezoidation.cpp:110-111 Twin segment(s) have already been made
            unsafe {
                // SkeletalTrapezoidation.cpp:114 auto end_node_it = vd_node_to_he_node.find(vd_edge.vertex1());
                let vd_v1 = diagram.edge_get_vertex1(vd_edge).ok().flatten().unwrap();
                // SkeletalTrapezoidation.cpp:116 node_t* end_node = end_node_it->second;
                let end_node = *self.vd_node_to_he_node.get(&vd_v1).unwrap();
                // SkeletalTrapezoidation.cpp:117 for (edge_t* twin = source_twin; ; twin = twin->prev->twin->prev)
                let mut twin = Some(source_twin);
                loop {
                    // SkeletalTrapezoidation.cpp:119-124 if(!twin){ warning; continue; }
                    // (a None twin can only arise if the chain is malformed; the C++
                    //  `continue` re-loops on the same broken state — guard against an
                    //  infinite loop by breaking.)
                    let twin_p = match twin {
                        Some(t) => t,
                        None => {
                            log::warn!("Encountered a voronoi edge without twin.");
                            break;
                        }
                    };

                    // SkeletalTrapezoidation.cpp:126 graph.edges.emplace_front(SkeletalTrapezoidationEdge());
                    self.graph.edges.push_front(Box::new(
                        crate::arachne::skeletal_trapezoidation_graph::STHalfEdge::new(
                            crate::arachne::skeletal_trapezoidation_edge::SkeletalTrapezoidationEdge::new(),
                        ),
                    ));
                    // SkeletalTrapezoidation.cpp:127 edge_t* edge = &graph.edges.front();
                    let edge = SkeletalTrapezoidationGraph::edge_ptr(self.graph.edges.front().unwrap());
                    {
                        let edge_mut = edge.as_ptr().as_mut().unwrap();
                        let twin_ref = twin_p.as_ref();
                        // SkeletalTrapezoidation.cpp:128 edge->from = twin->to;
                        edge_mut.from = twin_ref.to;
                        // SkeletalTrapezoidation.cpp:129 edge->to = twin->from;
                        edge_mut.to = twin_ref.from;
                        // SkeletalTrapezoidation.cpp:130 edge->twin = twin;
                        edge_mut.twin = Some(twin_p);
                    }
                    // SkeletalTrapezoidation.cpp:131 twin->twin = edge;
                    twin_p.as_ptr().as_mut().unwrap().twin = Some(edge);
                    // SkeletalTrapezoidation.cpp:132 edge->from->incident_edge = edge;
                    edge.as_ref().from.unwrap().as_ptr().as_mut().unwrap().incident_edge = Some(edge);
                    // SkeletalTrapezoidation.cpp:133 edge->data.setHoleCompensationFlag(hole_compensation_flag);
                    edge.as_ptr().as_mut().unwrap().data.set_hole_compensation_flag(hole_compensation_flag);
                    // SkeletalTrapezoidation.cpp:134-138 if (prev_edge) { edge->prev = prev_edge; prev_edge->next = edge; }
                    if let Some(prev) = *prev_edge {
                        edge.as_ptr().as_mut().unwrap().prev = Some(prev);
                        prev.as_ptr().as_mut().unwrap().next = Some(edge);
                    }
                    // SkeletalTrapezoidation.cpp:140 prev_edge = edge;
                    *prev_edge = Some(edge);

                    // SkeletalTrapezoidation.cpp:142-145 if (prev_edge->to == end_node) return;
                    if edge.as_ref().to == Some(end_node) {
                        return;
                    }

                    // SkeletalTrapezoidation.cpp:147-151 if (!twin->prev || !twin->prev->twin || !twin->prev->twin->prev) { error; return; }
                    let twin_prev = twin_p.as_ref().prev;
                    let twin_prev_twin = twin_prev.and_then(|tp| tp.as_ref().twin);
                    let twin_prev_twin_prev = twin_prev_twin.and_then(|tpt| tpt.as_ref().prev);
                    if twin_prev.is_none() || twin_prev_twin.is_none() || twin_prev_twin_prev.is_none() {
                        log::error!("Discretized segment behaves oddly!");
                        return;
                    }

                    // SkeletalTrapezoidation.cpp:161 graph.makeRib(prev_edge, start_source_point, end_source_point, is_not_next_to_start_or_end);
                    let mut pe = edge;
                    self.graph.make_rib(&mut pe, start_source_point, end_source_point, false);
                    *prev_edge = Some(pe);

                    // SkeletalTrapezoidation.cpp:117 twin = twin->prev->twin->prev
                    twin = twin_prev_twin_prev;
                }
            }
        } else {
            // SkeletalTrapezoidation.cpp:162 Points discretized = discretize(vd_edge, segments);
            let discretized = self.discretize(diagram, vd_edge, segments);
            if crate::probe_enabled("GBUILD") {
                gbuild_disc(discretized.len());
            }
            // SkeletalTrapezoidation.cpp:163-167 assert/warn discretized.size() >= 2
            debug_assert!(discretized.len() >= 2);
            if discretized.len() < 2 {
                log::warn!("Discretized Voronoi edge is degenerate.");
            }

            unsafe {
                // SkeletalTrapezoidation.cpp:169-173 assert/warn prev_edge->to
                // SkeletalTrapezoidation.cpp:174 node_t* v0 = (prev_edge)? prev_edge->to : &makeNode(*vd_edge.vertex0(), from);
                let mut v0: NodePtr = if let Some(prev) = *prev_edge {
                    prev.as_ref().to.unwrap()
                } else {
                    let vd_v0 = diagram.edges()[vd_edge.usize()].vertex0().unwrap();
                    self.make_node(vd_v0, from)
                };
                // SkeletalTrapezoidation.cpp:175 Point p0 = discretized.front();
                // (p0 is only used by C++ to advance; we index discretized directly.)
                // SkeletalTrapezoidation.cpp:176 for (size_t p1_idx = 1; ...)
                for p1_idx in 1..discretized.len() {
                    // SkeletalTrapezoidation.cpp:178 Point p1 = discretized[p1_idx];
                    let p1 = discretized[p1_idx];
                    // SkeletalTrapezoidation.cpp:179-189 node_t* v1
                    let v1: NodePtr = if p1_idx < discretized.len() - 1 {
                        // SkeletalTrapezoidation.cpp:182-183 graph.nodes.emplace_front(..., p1); v1 = &graph.nodes.front();
                        self.graph.nodes.push_front(Box::new(STHalfEdgeNode::new(
                            crate::arachne::skeletal_trapezoidation_joint::SkeletalTrapezoidationJoint::new(),
                            p1,
                        )));
                        SkeletalTrapezoidationGraph::node_ptr(self.graph.nodes.front().unwrap())
                    } else {
                        // SkeletalTrapezoidation.cpp:187 v1 = &makeNode(*vd_edge.vertex1(), to);
                        let vd_v1 = diagram.edge_get_vertex1(vd_edge).ok().flatten().unwrap();
                        self.make_node(vd_v1, to)
                    };

                    // SkeletalTrapezoidation.cpp:191 graph.edges.emplace_front(SkeletalTrapezoidationEdge());
                    self.graph.edges.push_front(Box::new(
                        crate::arachne::skeletal_trapezoidation_graph::STHalfEdge::new(
                            crate::arachne::skeletal_trapezoidation_edge::SkeletalTrapezoidationEdge::new(),
                        ),
                    ));
                    // SkeletalTrapezoidation.cpp:192 edge_t* edge = &graph.edges.front();
                    let edge = SkeletalTrapezoidationGraph::edge_ptr(self.graph.edges.front().unwrap());
                    {
                        let edge_mut = edge.as_ptr().as_mut().unwrap();
                        // SkeletalTrapezoidation.cpp:193 edge->from = v0;
                        edge_mut.from = Some(v0);
                        // SkeletalTrapezoidation.cpp:194 edge->to = v1;
                        edge_mut.to = Some(v1);
                    }
                    // SkeletalTrapezoidation.cpp:195 edge->from->incident_edge = edge;
                    v0.as_ptr().as_mut().unwrap().incident_edge = Some(edge);
                    // SkeletalTrapezoidation.cpp:196 edge->data.setHoleCompensationFlag(hole_compensation_flag);
                    edge.as_ptr().as_mut().unwrap().data.set_hole_compensation_flag(hole_compensation_flag);

                    // SkeletalTrapezoidation.cpp:198-202 if (prev_edge) { edge->prev = prev_edge; prev_edge->next = edge; }
                    if let Some(prev) = *prev_edge {
                        edge.as_ptr().as_mut().unwrap().prev = Some(prev);
                        prev.as_ptr().as_mut().unwrap().next = Some(edge);
                    }

                    // SkeletalTrapezoidation.cpp:204 prev_edge = edge;
                    *prev_edge = Some(edge);
                    // SkeletalTrapezoidation.cpp:205-206 p0 = p1; v0 = v1;
                    v0 = v1;

                    // SkeletalTrapezoidation.cpp:208-212 if (p1_idx < discretized.size() - 1) { makeRib }
                    if p1_idx < discretized.len() - 1 {
                        let mut pe = edge;
                        self.graph.make_rib(&mut pe, start_source_point, end_source_point, false);
                        *prev_edge = Some(pe);
                    }
                }
                // SkeletalTrapezoidation.cpp:214 assert(prev_edge);
                // SkeletalTrapezoidation.cpp:215 vd_edge_to_he_edge.emplace(&vd_edge, prev_edge);
                self.vd_edge_to_he_edge.insert(vd_edge, prev_edge.unwrap());
            }
        }
    }

    // SkeletalTrapezoidation.cpp:219-329
    // Points SkeletalTrapezoidation::discretize(const VD::edge_type& vd_edge, const std::vector<Segment>& segments)
    fn discretize(
        &self,
        diagram: &bv::Diagram,
        vd_edge: bv::EdgeIndex,
        segments: &[PolygonsSegmentIndex],
    ) -> Vec<Point> {
        // SkeletalTrapezoidation.cpp:223 const VD::cell_type *left_cell = vd_edge.cell();
        let left_cell_id = diagram.edges()[vd_edge.usize()].cell().unwrap();
        // SkeletalTrapezoidation.cpp:224 const VD::cell_type *right_cell = vd_edge.twin()->cell();
        let twin_id = diagram.edges()[vd_edge.usize()].twin().unwrap();
        let right_cell_id = diagram.edges()[twin_id.usize()].cell().unwrap();
        let left_cell = diagram.cell(left_cell_id).unwrap();
        let right_cell = diagram.cell(right_cell_id).unwrap();

        // SkeletalTrapezoidation.cpp:226 Point start = ...vertex0()...; SkeletalTrapezoidation.cpp:227 Point end = ...vertex1()...;
        let v0i = diagram.edges()[vd_edge.usize()].vertex0().unwrap();
        let v1i = diagram.edge_get_vertex1(vd_edge).ok().flatten().unwrap();
        let v0 = &diagram.vertices()[v0i.usize()];
        let v1 = &diagram.vertices()[v1i.usize()];
        let start = crate::geometry::voronoi_utils::to_point(v0.x(), v0.y());
        let end = crate::geometry::voronoi_utils::to_point(v1.x(), v1.y());

        // SkeletalTrapezoidation.cpp:229 bool point_left = left_cell->contains_point();
        let point_left = left_cell.contains_point();
        // SkeletalTrapezoidation.cpp:230 bool point_right = right_cell->contains_point();
        let point_right = right_cell.contains_point();
        let is_secondary = diagram.edges()[vd_edge.usize()].is_secondary();

        // SkeletalTrapezoidation.cpp:231-234 if ((!point_left && !point_right) || vd_edge.is_secondary())
        if (!point_left && !point_right) || is_secondary {
            // SkeletalTrapezoidation.cpp:233 return Points({ start, end });
            vec![start, end]
        } else if point_left != point_right {
            // SkeletalTrapezoidation.cpp:235-241 parabolic edge between a point and a line.
            // SkeletalTrapezoidation.cpp:237 Point p = get_source_point(*(point_left ? left_cell : right_cell), ...);
            let p = source_point(if point_left { left_cell } else { right_cell }, segments);
            // SkeletalTrapezoidation.cpp:238 const Segment& s = get_source_segment(*(point_left ? right_cell : left_cell), ...);
            let s = source_segment(if point_left { right_cell } else { left_cell }, segments);
            // SkeletalTrapezoidation.cpp:239 return discretize_parabola(p, s, start, end, discretization_step_size, transitioning_angle);
            crate::geometry::voronoi_utils::discretize_parabola(
                p,
                s.from(),
                s.to(),
                start,
                end,
                self.discretization_step_size,
                // C++ `transitioning_angle` is `double`; the crate's
                // discretize_parabola takes `f32` (the C++ callee likewise narrows).
                self.transitioning_angle as f32,
            )
        } else {
            // SkeletalTrapezoidation.cpp:242-328 straight edge between two points.
            // SkeletalTrapezoidation.cpp:248 Point left_point = get_source_point(*left_cell, ...);
            let left_point = source_point(left_cell, segments);
            // SkeletalTrapezoidation.cpp:249 Point right_point = get_source_point(*right_cell, ...);
            let right_point = source_point(right_cell, segments);
            // SkeletalTrapezoidation.cpp:250 coord_t d = (right_point - left_point).cast<int64_t>().norm();
            let d = ((right_point - left_point).length()) as i64;
            // SkeletalTrapezoidation.cpp:251 Point middle = (left_point + right_point) / 2;
            let middle = Point::new((left_point.x + right_point.x) / 2, (left_point.y + right_point.y) / 2);
            // SkeletalTrapezoidation.cpp:252 Point x_axis_dir = perp(Point(right_point - left_point));
            let x_axis_dir = perp(right_point - left_point);
            // SkeletalTrapezoidation.cpp:253 coord_t x_axis_length = x_axis_dir.cast<int64_t>().norm();
            let x_axis_length = (x_axis_dir.length()) as i64;

            // SkeletalTrapezoidation.cpp:255-261 projected_x lambda
            let projected_x = |fromp: Point| -> i64 {
                let vec = fromp - middle;
                // coord_t x = vec.dot(x_axis_dir) / x_axis_length;
                (vec.x as i64 * x_axis_dir.x as i64 + vec.y as i64 * x_axis_dir.y as i64) / x_axis_length
            };

            // SkeletalTrapezoidation.cpp:263 coord_t start_x = projected_x(start);
            let start_x = projected_x(start);
            // SkeletalTrapezoidation.cpp:264 coord_t end_x = projected_x(end);
            let end_x = projected_x(end);

            // SkeletalTrapezoidation.cpp:267 float bound = 0.5 / tan((M_PI - transitioning_angle) * 0.5);
            let bound = 0.5 / ((std::f64::consts::PI - self.transitioning_angle) * 0.5).tan();
            // SkeletalTrapezoidation.cpp:268 int64_t marking_start_x = - int64_t(d) * bound;
            let mut marking_start_x = (-(d as f64) * bound) as i64;
            // SkeletalTrapezoidation.cpp:269 int64_t marking_end_x = int64_t(d) * bound;
            let mut marking_end_x = ((d as f64) * bound) as i64;

            // SkeletalTrapezoidation.cpp:275-276
            // Point marking_start = middle + (x_axis_dir * marking_start_x / x_axis_length).cast<coord_t>();
            let mut marking_start = Point::new(
                middle.x + (x_axis_dir.x as i64 * marking_start_x / x_axis_length),
                middle.y + (x_axis_dir.y as i64 * marking_start_x / x_axis_length),
            );
            // Point marking_end = middle + (x_axis_dir * marking_end_x / x_axis_length).cast<coord_t>();
            let mut marking_end = Point::new(
                middle.x + (x_axis_dir.x as i64 * marking_end_x / x_axis_length),
                middle.y + (x_axis_dir.y as i64 * marking_end_x / x_axis_length),
            );
            // SkeletalTrapezoidation.cpp:277 int64_t direction = 1;
            let mut direction: i64 = 1;

            // SkeletalTrapezoidation.cpp:279-284 if (start_x > end_x) { direction = -1; swap... }
            if start_x > end_x {
                direction = -1;
                std::mem::swap(&mut marking_start, &mut marking_end);
                std::mem::swap(&mut marking_start_x, &mut marking_end_x);
            }

            // SkeletalTrapezoidation.cpp:287-289 Point a = start; Point b = end; Points ret; ret.emplace_back(a);
            let a = start;
            let b = end;
            let mut ret: Vec<Point> = Vec::new();
            ret.push(a);

            // SkeletalTrapezoidation.cpp:292-293
            let mut add_marking_start = marking_start_x * direction > start_x * direction;
            let mut add_marking_end = marking_end_x * direction > start_x * direction;

            // SkeletalTrapezoidation.cpp:296 Point ab = b - a;
            let ab = b - a;
            // SkeletalTrapezoidation.cpp:297 coord_t ab_size = ab.cast<int64_t>().norm();
            let ab_size = (ab.length()) as i64;
            // SkeletalTrapezoidation.cpp:298 coord_t step_count = (ab_size + discretization_step_size / 2) / discretization_step_size;
            let mut step_count = (ab_size + self.discretization_step_size / 2) / self.discretization_step_size;
            // SkeletalTrapezoidation.cpp:299-302 if (step_count % 2 == 1) step_count++;
            if step_count % 2 == 1 {
                step_count += 1;
            }
            // SkeletalTrapezoidation.cpp:303-318
            for step in 1..step_count {
                // SkeletalTrapezoidation.cpp:305 Point here = a + (ab * step / step_count).cast<coord_t>();
                let here = Point::new(
                    a.x + (ab.x as i64 * step / step_count),
                    a.y + (ab.y as i64 * step / step_count),
                );
                // SkeletalTrapezoidation.cpp:306 coord_t x_here = projected_x(here);
                let x_here = projected_x(here);
                // SkeletalTrapezoidation.cpp:307-311
                if add_marking_start && marking_start_x * direction < x_here * direction {
                    ret.push(marking_start);
                    add_marking_start = false;
                }
                // SkeletalTrapezoidation.cpp:312-316
                if add_marking_end && marking_end_x * direction < x_here * direction {
                    ret.push(marking_end);
                    add_marking_end = false;
                }
                // SkeletalTrapezoidation.cpp:317 ret.emplace_back(here);
                ret.push(here);
            }
            // SkeletalTrapezoidation.cpp:319-322 if (add_marking_end && marking_end_x*direction < end_x*direction) ret.emplace_back(marking_end);
            if add_marking_end && marking_end_x * direction < end_x * direction {
                ret.push(marking_end);
            }
            // SkeletalTrapezoidation.cpp:323 ret.emplace_back(b);
            ret.push(b);
            // SkeletalTrapezoidation.cpp:324 return ret;
            ret
        }
    }

    // SkeletalTrapezoidation.cpp:330-371
    // bool SkeletalTrapezoidation::computePointCellRange(const VD::cell_type &cell, Point &start_source_point, Point &end_source_point, const VD::edge_type *&starting_vd_edge, const VD::edge_type *&ending_vd_edge, const std::vector<Segment> &segments)
    //
    // Returns `Some((start_source_point, end_source_point, starting_vd_edge, ending_vd_edge))`
    // when the cell should be copied, `None` otherwise (the C++ `return false`).
    fn compute_point_cell_range(
        &self,
        diagram: &bv::Diagram,
        cell: &bv::Cell,
        segments: &[PolygonsSegmentIndex],
    ) -> Option<(Point, Point, bv::EdgeIndex, bv::EdgeIndex)> {
        // SkeletalTrapezoidation.cpp:331-332 if (cell.incident_edge()->is_infinite()) return false;
        let incident_edge = cell.get_incident_edge().unwrap();
        if diagram.edge_is_infinite(incident_edge).unwrap_or(true) {
            return None;
        }

        // SkeletalTrapezoidation.cpp:338-341 If incident_edge->vertex0() doesn't fit in Vec2i64, bail.
        let inc_v0i = diagram.edges()[incident_edge.usize()].vertex0().unwrap();
        let inc_v0 = &diagram.vertices()[inc_v0i.usize()];
        if inc_v0.x() >= i64::MAX as f64
            || inc_v0.x() <= i64::MIN as f64
            || inc_v0.y() >= i64::MAX as f64
            || inc_v0.y() <= i64::MIN as f64
        {
            return None;
        }

        // SkeletalTrapezoidation.cpp:343 const Point source_point = get_source_point(cell, ...);
        let source_point = source_point(cell, segments);
        // SkeletalTrapezoidation.cpp:344 const PolygonsPointIndex source_point_index = get_source_point_index(cell, ...);
        let source_point_index = source_point_index(cell, segments);
        // SkeletalTrapezoidation.cpp:345 Vec2i64 some_point = to_point(cell.incident_edge()->vertex0());
        let mut some_point = (inc_v0.x().round() as i64, inc_v0.y().round() as i64);
        // SkeletalTrapezoidation.cpp:346-347 if (some_point == source_point) some_point = to_point(cell.incident_edge()->vertex1());
        if some_point == (source_point.x, source_point.y) {
            let inc_v1i = diagram.edge_get_vertex1(incident_edge).ok().flatten().unwrap();
            let inc_v1 = &diagram.vertices()[inc_v1i.usize()];
            some_point = (inc_v1.x().round() as i64, inc_v1.y().round() as i64);
        }

        // SkeletalTrapezoidation.cpp:352-353
        // if (!LinearAlg2D::isInsideCorner(source_point_index.prev().p(), source_point_index.p(), source_point_index.next().p(), some_point)) return false;
        if !is_inside_corner(
            source_point_index.prev().p(),
            source_point_index.p(),
            source_point_index.next().p(),
            Point::new(some_point.0, some_point.1),
        ) {
            return None;
        }

        // SkeletalTrapezoidation.cpp:355 const VD::edge_type* vd_edge = cell.incident_edge();
        let mut starting_vd_edge: Option<bv::EdgeIndex> = None;
        let mut ending_vd_edge: Option<bv::EdgeIndex> = None;
        let mut start_source_point = Point::new(0, 0);
        let mut end_source_point = Point::new(0, 0);
        let mut vd_edge = incident_edge;
        // SkeletalTrapezoidation.cpp:356-368 do { ... } while (vd_edge = vd_edge->next(), vd_edge != cell.incident_edge());
        loop {
            // SkeletalTrapezoidation.cpp:359 if (Vec2i64 p1 = to_point(vd_edge->vertex1()); p1 == source_point)
            let v1i = diagram.edge_get_vertex1(vd_edge).ok().flatten().unwrap();
            let v1 = &diagram.vertices()[v1i.usize()];
            let p1 = (v1.x().round() as i64, v1.y().round() as i64);
            if p1 == (source_point.x, source_point.y) {
                // SkeletalTrapezoidation.cpp:360-363
                start_source_point = source_point;
                end_source_point = source_point;
                starting_vd_edge = Some(diagram.edges()[vd_edge.usize()].next().unwrap());
                ending_vd_edge = Some(vd_edge);
            }
            // SkeletalTrapezoidation.cpp:368 while (vd_edge = vd_edge->next(), vd_edge != cell.incident_edge());
            vd_edge = diagram.edges()[vd_edge.usize()].next().unwrap();
            if vd_edge == incident_edge {
                break;
            }
        }
        // SkeletalTrapezoidation.cpp:369-370 assert(starting_vd_edge && ending_vd_edge); assert(starting_vd_edge != ending_vd_edge);
        let s = starting_vd_edge?;
        let e = ending_vd_edge?;
        debug_assert!(s != e);
        // SkeletalTrapezoidation.cpp:371 return true;
        Some((start_source_point, end_source_point, s, e))
    }

    // SkeletalTrapezoidation.cpp:391-504
    // void SkeletalTrapezoidation::constructFromPolygons(const Polygons& polys)
    pub fn construct_from_polygons(&mut self, polys: &Polygons) {
        use std::collections::HashSet;

        // SkeletalTrapezoidation.cpp:397 std::set<int> hole_indices_(...);
        let hole_indices_: HashSet<i32> = self.hole_indices.iter().copied().collect();

        // SkeletalTrapezoidation.cpp:416-417 vd_edge_to_he_edge.clear(); vd_node_to_he_node.clear();
        self.vd_edge_to_he_edge.clear();
        self.vd_node_to_he_node.clear();

        // SkeletalTrapezoidation.cpp:419-422 std::vector<Segment> segments; for poly,point: segments.emplace_back(&polys, poly_idx, point_idx);
        // Build the parallel (`Line` for VD construction) + (`PolygonsSegmentIndex`
        // for source lookup) arrays in identical order so cell.source_index() maps
        // to the right segment (the crate's VD is built from `&[Line]`).
        let mut segments: Vec<PolygonsSegmentIndex> = Vec::new();
        let mut lines: Vec<Line> = Vec::new();
        for poly_idx in 0..polys.len() {
            for point_idx in 0..polys[poly_idx].points.len() {
                let seg = PolygonsSegmentIndex::with_indices(polys, poly_idx, point_idx);
                lines.push(Line::new(seg.from(), seg.to()));
                segments.push(seg);
            }
        }

        // SkeletalTrapezoidation.cpp:432-433 VD voronoi_diagram; voronoi_diagram.construct_voronoi(segments...);
        let mut vd = VoronoiDiagram::new();
        // C++ constructs directly from the segments without the repair wrapper.
        if vd.construct_voronoi(&lines, false).is_err() {
            return;
        }
        let diagram = vd.diagram();

        // GBUILD (R589): bracket where the 1.25x graph density (R588) appears --
        // Voronoi INPUT (one segment per polygon point) versus raw Voronoi OUTPUT
        // before any filtering. If the input already differs the cause is upstream
        // outline discretisation, not the skeleton.
        if crate::probe_enabled("GBUILD") {
            gbuild_tick(
                polys.len(),
                segments.len(),
                diagram.vertices().len(),
                diagram.edges().len(),
                diagram.cells().len(),
            );
        }

        // SkeletalTrapezoidation.cpp:443 for (const VD::cell_type &cell : voronoi_diagram.cells())
        for cell_idx in 0..diagram.cells().len() {
            let cell = &diagram.cells()[cell_idx];
            if crate::probe_enabled("CONV") {
                conv_cell(cell.get_incident_edge().is_none());
            }
            // SkeletalTrapezoidation.cpp:444-445 if (!cell.incident_edge()) continue;
            let incident_edge = match cell.get_incident_edge() {
                Some(e) => e,
                None => continue,
            };
            let _ = incident_edge;

            // SkeletalTrapezoidation.cpp:447-450 local vars
            let start_source_point;
            let end_source_point;
            let starting_voronoi_edge;
            let ending_voronoi_edge;

            // SkeletalTrapezoidation.cpp:452 bool apply_hole_compensation = this->enable_hole_compensation;
            let mut apply_hole_compensation = self.enable_hole_compensation;

            // SkeletalTrapezoidation.cpp:454-475 if (cell.contains_point()) { ... } else { ... }
            if cell.contains_point() {
                // SkeletalTrapezoidation.cpp:455-457 if (!computePointCellRange(...)) continue;
                match self.compute_point_cell_range(diagram, cell, &segments) {
                    Some((ssp, esp, s, e)) => {
                        start_source_point = ssp;
                        end_source_point = esp;
                        starting_voronoi_edge = Some(s);
                        ending_voronoi_edge = Some(e);
                    }
                    None => continue,
                }
                // SkeletalTrapezoidation.cpp:459-460
                // const PolygonsPointIndex source_point_idx = get_source_point_index(cell, ...);
                // apply_hole_compensation &= hole_indices_.find(source_point_idx.poly_idx) != end();
                let source_point_idx = source_point_index(cell, &segments);
                apply_hole_compensation &= hole_indices_.contains(&(source_point_idx.poly_idx as i32));
            } else {
                // SkeletalTrapezoidation.cpp:462 assert(cell.contains_segment());
                debug_assert!(cell.contains_segment());
                // SkeletalTrapezoidation.cpp:463 SegmentCellRange cell_range = compute_segment_cell_range(cell, ...);
                let cell_range = compute_segment_cell_range(diagram, cell, &segments);
                // SkeletalTrapezoidation.cpp:464 assert(cell_range.is_valid());
                debug_assert!(cell_range.is_valid());
                // SkeletalTrapezoidation.cpp:465-468
                start_source_point = cell_range.segment_start_point;
                end_source_point = cell_range.segment_end_point;
                starting_voronoi_edge = cell_range.edge_begin;
                ending_voronoi_edge = cell_range.edge_end;

                // SkeletalTrapezoidation.cpp:470-471
                // const Segment& source_segment = get_source_segment(cell, ...);
                // apply_hole_compensation &= hole_indices_.find(source_segment.poly_idx) != end();
                let source_segment = source_segment(cell, &segments);
                apply_hole_compensation &= hole_indices_.contains(&(source_segment.poly_idx() as i32));
            }

            // SkeletalTrapezoidation.cpp:477-481 if (!starting_voronoi_edge || !ending_voronoi_edge) { assert(false); continue; }
            let (starting_voronoi_edge, ending_voronoi_edge) =
                match (starting_voronoi_edge, ending_voronoi_edge) {
                    (Some(s), Some(e)) => (s, e),
                    _ => continue,
                };

            // SkeletalTrapezoidation.cpp:484-487 Copy start to end edge to graph
            let mut prev_edge: Option<EdgePtr> = None;
            // SkeletalTrapezoidation.cpp:486 transferEdge(start_source_point, to_point(starting_voronoi_edge->vertex1()), *starting_voronoi_edge, prev_edge, ...);
            let s_v1i = diagram.edge_get_vertex1(starting_voronoi_edge).ok().flatten().unwrap();
            let s_v1 = &diagram.vertices()[s_v1i.usize()];
            let s_v1_pt = crate::geometry::voronoi_utils::to_point(s_v1.x(), s_v1.y());
            self.transfer_edge(
                diagram,
                start_source_point,
                s_v1_pt,
                starting_voronoi_edge,
                &mut prev_edge,
                start_source_point,
                end_source_point,
                &segments,
                apply_hole_compensation,
            );
            unsafe {
                // SkeletalTrapezoidation.cpp:487 node_t* starting_node = vd_node_to_he_node[starting_voronoi_edge->vertex0()];
                let s_v0i = diagram.edges()[starting_voronoi_edge.usize()].vertex0().unwrap();
                let starting_node = *self.vd_node_to_he_node.get(&s_v0i).unwrap();
                // SkeletalTrapezoidation.cpp:488 starting_node->data.distance_to_boundary = 0;
                starting_node.as_ptr().as_mut().unwrap().data.distance_to_boundary = 0;
            }

            // SkeletalTrapezoidation.cpp:490-491 makeRib(prev_edge, start_source_point, end_source_point, true);
            {
                let mut pe = prev_edge.unwrap();
                self.graph.make_rib(&mut pe, start_source_point, end_source_point, true);
                prev_edge = Some(pe);
            }
            // SkeletalTrapezoidation.cpp:492-498 for (vd_edge = starting->next(); vd_edge != ending; vd_edge = vd_edge->next())
            let mut vd_edge = diagram.edges()[starting_voronoi_edge.usize()].next().unwrap();
            while vd_edge != ending_voronoi_edge {
                // SkeletalTrapezoidation.cpp:496-497
                let ve_v0i = diagram.edges()[vd_edge.usize()].vertex0().unwrap();
                let ve_v1i = diagram.edge_get_vertex1(vd_edge).ok().flatten().unwrap();
                let ve_v0 = &diagram.vertices()[ve_v0i.usize()];
                let ve_v1 = &diagram.vertices()[ve_v1i.usize()];
                let v1pt = crate::geometry::voronoi_utils::to_point(ve_v0.x(), ve_v0.y());
                let v2pt = crate::geometry::voronoi_utils::to_point(ve_v1.x(), ve_v1.y());
                self.transfer_edge(
                    diagram,
                    v1pt,
                    v2pt,
                    vd_edge,
                    &mut prev_edge,
                    start_source_point,
                    end_source_point,
                    &segments,
                    apply_hole_compensation,
                );
                // SkeletalTrapezoidation.cpp:498 makeRib(prev_edge, ..., vd_edge->next() == ending_voronoi_edge);
                let next_is_ending = diagram.edges()[vd_edge.usize()].next().unwrap() == ending_voronoi_edge;
                {
                    let mut pe = prev_edge.unwrap();
                    self.graph.make_rib(&mut pe, start_source_point, end_source_point, next_is_ending);
                    prev_edge = Some(pe);
                }
                vd_edge = diagram.edges()[vd_edge.usize()].next().unwrap();
            }

            // SkeletalTrapezoidation.cpp:500-501 transferEdge(to_point(ending->vertex0()), end_source_point, *ending, ...);
            let e_v0i = diagram.edges()[ending_voronoi_edge.usize()].vertex0().unwrap();
            let e_v0 = &diagram.vertices()[e_v0i.usize()];
            let e_v0_pt = crate::geometry::voronoi_utils::to_point(e_v0.x(), e_v0.y());
            self.transfer_edge(
                diagram,
                e_v0_pt,
                end_source_point,
                ending_voronoi_edge,
                &mut prev_edge,
                start_source_point,
                end_source_point,
                &segments,
                apply_hole_compensation,
            );
            // SkeletalTrapezoidation.cpp:501 prev_edge->to->data.distance_to_boundary = 0;
            unsafe {
                prev_edge.unwrap().as_ref().to.unwrap().as_ptr().as_mut().unwrap().data.distance_to_boundary = 0;
            }
        }

        // CONV (R590): three edge counts decide whether the 1.25x density gap comes
        // from CREATING fewer half-edges or REMOVING more. Creation is transfer_edge
        // plus make_rib (2 EXTRA_VD edges per call); removal is collapse_small_edges.
        if crate::probe_enabled("CONV") {
            conv_stage(0, self.graph.edges.len(), self.graph.nodes.len());
        }

        // SkeletalTrapezoidation.cpp:507 separatePointyQuadEndNodes();
        self.separate_pointy_quad_end_nodes();
        if crate::probe_enabled("CONV") {
            conv_stage(1, self.graph.edges.len(), self.graph.nodes.len());
        }

        // SkeletalTrapezoidation.cpp:509 graph.collapseSmallEdges();
        //
        // R590: C++ has TWO distinct snap distances and this port conflated them.
        // `SkeletalTrapezoidation::snap_dist` (hpp:71) is scaled<coord_t>(0.02) =
        // 2000 and is used ONLY for transition ends -- "Only used to determine
        // whether a transition really needs to insert an extra edge". The collapse
        // uses a SEPARATE parameter, `collapseSmallEdges(coord_t snap_dist = 5)`
        // (SkeletalTrapezoidationGraph.hpp:84), and C++ calls it with NO argument,
        // i.e. 5. Passing SNAP_DIST here made our snap distance 400x C++'s, which
        // is why collapse kept only 67.7% of our edges against C++'s 77.8% and left
        // our skeletal graph ~25% sparser (R588/R589).
        let collapse_snap_dist = if crate::faithful_gate("ARACHNE_COLLAPSE_SNAP_5") {
            5
        } else {
            SNAP_DIST
        };
        self.graph.collapse_small_edges(collapse_snap_dist);
        if crate::probe_enabled("CONV") {
            conv_stage(2, self.graph.edges.len(), self.graph.nodes.len());
        }

        // SkeletalTrapezoidation.cpp:513-515 for (edge_t& edge : graph.edges) if (!edge.prev) edge.from->incident_edge = &edge;
        unsafe {
            let edges: Vec<EdgePtr> = self
                .graph
                .edges
                .iter()
                .map(|e| SkeletalTrapezoidationGraph::edge_ptr(e))
                .collect();
            for edge in edges {
                if edge.as_ref().prev.is_none() {
                    edge.as_ref().from.unwrap().as_ptr().as_mut().unwrap().incident_edge = Some(edge);
                }
            }
        }
    }

    // SkeletalTrapezoidation.cpp:510-533
    // void SkeletalTrapezoidation::separatePointyQuadEndNodes()
    pub fn separate_pointy_quad_end_nodes(&mut self) {
        unsafe {
            // SkeletalTrapezoidation.cpp:512 NodeSet visited_nodes;
            let mut visited_nodes: std::collections::HashSet<*const HalfEdgeNode<EdgeData, NodeData>> =
                std::collections::HashSet::new();
            // SkeletalTrapezoidation.cpp:513 for (edge_t& edge : graph.edges)
            //
            // The body may `nodes.emplace_back(...)` (line 526) which appends to the same
            // list; collecting the quad-start edge pointers up front then iterating
            // reproduces the C++ traversal order without invalidating an in-flight
            // iterator (LinkedList iteration would otherwise borrow `self.graph`).
            let edges: Vec<EdgePtr> = self
                .graph
                .edges
                .iter()
                .map(|e| SkeletalTrapezoidationGraph::edge_ptr(e))
                .collect();
            for edge in edges {
                let edge_ref = edge.as_ref();
                // SkeletalTrapezoidation.cpp:515 if (edge.prev)
                if edge_ref.prev.is_some() {
                    // SkeletalTrapezoidation.cpp:517 continue;
                    continue;
                }
                // SkeletalTrapezoidation.cpp:519 edge_t* quad_start = &edge;
                let quad_start = edge;
                let quad_start_from = quad_start.as_ref().from.unwrap();
                // SkeletalTrapezoidation.cpp:520 if (visited_nodes.find(quad_start->from) == visited_nodes.end())
                if !visited_nodes.contains(&(quad_start_from.as_ptr() as *const _)) {
                    // SkeletalTrapezoidation.cpp:522 visited_nodes.emplace(quad_start->from);
                    visited_nodes.insert(quad_start_from.as_ptr() as *const _);
                } else {
                    // SkeletalTrapezoidation.cpp:524-531 Needs to be duplicated
                    // SkeletalTrapezoidation.cpp:526 graph.nodes.emplace_back(*quad_start->from);
                    let dup = as_st_node(quad_start_from).clone_node();
                    self.graph.nodes.push_back(Box::new(dup));
                    // SkeletalTrapezoidation.cpp:527 node_t* new_node = &graph.nodes.back();
                    let new_node = SkeletalTrapezoidationGraph::node_ptr(self.graph.nodes.back().unwrap());
                    // SkeletalTrapezoidation.cpp:528 new_node->incident_edge = quad_start;
                    new_node.as_ptr().as_mut().unwrap().incident_edge = Some(quad_start);
                    // SkeletalTrapezoidation.cpp:529 quad_start->from = new_node;
                    quad_start.as_ptr().as_mut().unwrap().from = Some(new_node);
                    // SkeletalTrapezoidation.cpp:530 quad_start->twin->to = new_node;
                    quad_start.as_ref().twin.unwrap().as_ptr().as_mut().unwrap().to = Some(new_node);
                }
            }
        }
    }

    //
    // ^^^^^^^^^^^^^^^^^^^^^
    //    INITIALIZATION
    // =====================
    //
    // =====================
    //    TRANSTISIONING
    // vvvvvvvvvvvvvvvvvvvvv
    //

    // SkeletalTrapezoidation.cpp:545-601
    // void SkeletalTrapezoidation::generateToolpaths(std::vector<VariableWidthLines>& generated_toolpaths, bool filter_outermost_central_edges)
    pub fn generate_toolpaths(
        &mut self,
        generated_toolpaths: &mut Vec<VariableWidthLines>,
        filter_outermost_central_edges: bool,
    ) {
        // SkeletalTrapezoidation.cpp:551 p_generated_toolpaths = &generated_toolpaths;
        self.p_generated_toolpaths = generated_toolpaths as *mut _;

        // SkeletalTrapezoidation.cpp:553 updateIsCentral();
        self.update_is_central();
        self.central_census("0 after updateIsCentral");

        // SkeletalTrapezoidation.cpp:559 filterCentral(central_filter_dist);
        self.filter_central(CENTRAL_FILTER_DIST);
        self.central_census("1 after filterCentral");

        // SkeletalTrapezoidation.cpp:565 if (filter_outermost_central_edges)
        if filter_outermost_central_edges {
            // SkeletalTrapezoidation.cpp:566 filterOuterCentral();
            self.filter_outer_central();
        }

        // SkeletalTrapezoidation.cpp:568 updateBeadCount();
        self.update_bead_count();
        self.central_census("2 after updateBeadCount");

        // SkeletalTrapezoidation.cpp:574 filterNoncentralRegions();
        self.filter_noncentral_regions();
        self.central_census("3 after filterNoncentralRegions");

        // SkeletalTrapezoidation.cpp:580 generateTransitioningRibs();
        self.generate_transitioning_ribs();
        self.central_census("4 after generateTransitioningRibs");

        // SkeletalTrapezoidation.cpp:586 generateExtraRibs();
        self.generate_extra_ribs();

        // SkeletalTrapezoidation.cpp:592 generateSegments();
        self.generate_segments();

        // LINEPROBE2 (R573) — per-ASSEMBLED-LINE width variety, measured here
        // because the beading is out of scope at every assembly point
        // (add_toolpath_segment receives only junctions). This is the earliest
        // per-loop measurement possible without tagging ExtrusionJunction.
        if crate::probe_enabled("LINEPROBE2") {
            use std::sync::atomic::{AtomicU64, Ordering::Relaxed};
            static LINES: AtomicU64 = AtomicU64::new(0);
            static JUNCS: AtomicU64 = AtomicU64::new(0);
            static DISTINCT: AtomicU64 = AtomicU64::new(0);
            static FLAT: AtomicU64 = AtomicU64::new(0);
            static CHANGES: AtomicU64 = AtomicU64::new(0);
            static O_LINES: AtomicU64 = AtomicU64::new(0);
            static O_JUNCS: AtomicU64 = AtomicU64::new(0);
            static O_DISTINCT: AtomicU64 = AtomicU64::new(0);
            static O_FLAT: AtomicU64 = AtomicU64::new(0);
            static O_CHANGES: AtomicU64 = AtomicU64::new(0);
            // R584: decompose the outer-wall width changes by MECHANISM. Within one
            // graph edge every junction draws from a single beading (edge->to's)
            // with junction_idx descending, so a change between consecutive
            // junctions is either an index step or the SAME index resolving to a
            // different beading on the next edge. R583 put the 1.378x change-density
            // gap at birth here; this says which mechanism supplies it.
            static M_PAIRS: AtomicU64 = AtomicU64::new(0);
            static M_CH_IDX: AtomicU64 = AtomicU64::new(0);
            static M_CH_BEAD: AtomicU64 = AtomicU64::new(0);
            static M_IDX_SAME_W: AtomicU64 = AtomicU64::new(0);
            for (inset, lines) in generated_toolpaths.iter().enumerate() {
                for line in lines.iter() {
                    let n = line.junctions.len() as u64;
                    if n == 0 {
                        continue;
                    }
                    let mut w: Vec<i64> = line.junctions.iter().map(|j| j.w).collect();
                    let changes =
                        (1..w.len()).filter(|&k| w[k] != w[k - 1]).count() as u64;
                    w.sort_unstable();
                    w.dedup();
                    let d = w.len() as u64;
                    LINES.fetch_add(1, Relaxed);
                    JUNCS.fetch_add(n, Relaxed);
                    DISTINCT.fetch_add(d, Relaxed);
                    CHANGES.fetch_add(changes, Relaxed);
                    if d == 1 {
                        FLAT.fetch_add(1, Relaxed);
                    }
                    if inset == 0 {
                        O_LINES.fetch_add(1, Relaxed);
                        O_JUNCS.fetch_add(n, Relaxed);
                        O_DISTINCT.fetch_add(d, Relaxed);
                        O_CHANGES.fetch_add(changes, Relaxed);
                        if d == 1 {
                            O_FLAT.fetch_add(1, Relaxed);
                        }
                        for k in 1..line.junctions.len() {
                            let a = &line.junctions[k - 1];
                            let b = &line.junctions[k];
                            M_PAIRS.fetch_add(1, Relaxed);
                            if b.w != a.w {
                                if b.perimeter_index != a.perimeter_index {
                                    M_CH_IDX.fetch_add(1, Relaxed);
                                } else {
                                    M_CH_BEAD.fetch_add(1, Relaxed);
                                }
                            } else if b.perimeter_index != a.perimeter_index {
                                M_IDX_SAME_W.fetch_add(1, Relaxed);
                            }
                        }
                    }
                }
            }
            let l = LINES.load(Relaxed);
            if l > 0 {
                eprintln!(
                    "[LINEPROBE2] lines={} juncs={} distinct={} flat={} changes={} | OUTER lines={} juncs={} distinct={} flat={} changes={} | MECH pairs={} ch_idx={} ch_bead={} idx_same_w={}",
                    l, JUNCS.load(Relaxed), DISTINCT.load(Relaxed), FLAT.load(Relaxed), CHANGES.load(Relaxed),
                    O_LINES.load(Relaxed), O_JUNCS.load(Relaxed), O_DISTINCT.load(Relaxed),
                    O_FLAT.load(Relaxed), O_CHANGES.load(Relaxed),
                    M_PAIRS.load(Relaxed), M_CH_IDX.load(Relaxed), M_CH_BEAD.load(Relaxed),
                    M_IDX_SAME_W.load(Relaxed),
                );
            }
        }
    }

    // SkeletalTrapezoidation.cpp:603-651
    // void SkeletalTrapezoidation::updateIsCentral()
    pub fn update_is_central(&mut self) {
        unsafe {
            // SkeletalTrapezoidation.cpp:618 coord_t outer_edge_filter_length = beading_strategy.getTransitionThickness(0) / 2;
            let outer_edge_filter_length = self.beading_strategy.get_transition_thickness(0) / 2;

            // SkeletalTrapezoidation.cpp:620 float cap = sin(beading_strategy.getTransitioningAngle() * 0.5);
            let cap = (self.beading_strategy.get_transitioning_angle() * 0.5).sin();
            // SkeletalTrapezoidation.cpp:621 for (edge_t& edge: graph.edges)
            for edge in self.graph.edges.iter_mut() {
                let twin = edge.base.twin;
                // SkeletalTrapezoidation.cpp:623 assert(edge.twin); if(!edge.twin) { ...; continue; }
                if twin.is_none() {
                    log::warn!("Encountered a Voronoi edge without twin!");
                    continue;
                }
                let twin = twin.unwrap();
                // SkeletalTrapezoidation.cpp:629 if(edge.twin->data.centralIsSet())
                if twin.as_ref().data.central_is_set() {
                    // SkeletalTrapezoidation.cpp:631 edge.data.setIsCentral(edge.twin->data.isCentral());
                    let twin_central = twin.as_ref().data.is_central();
                    edge.base.data.set_is_central(twin_central);
                    iscprobe(0, twin_central, outer_edge_filter_length, cap);
                }
                // SkeletalTrapezoidation.cpp:633 else if(edge.data.type == EdgeType::EXTRA_VD)
                else if edge.base.data.edge_type == EdgeType::ExtraVd {
                    // SkeletalTrapezoidation.cpp:635 edge.data.setIsCentral(false);
                    edge.base.data.set_is_central(false);
                    iscprobe(1, false, outer_edge_filter_length, cap);
                }
                // SkeletalTrapezoidation.cpp:637 else if(std::max(edge.from->...dtb, edge.to->...dtb) < outer_edge_filter_length)
                else {
                    let from_dtb = edge.base.from.unwrap().as_ref().data.distance_to_boundary;
                    let to_dtb = edge.base.to.unwrap().as_ref().data.distance_to_boundary;
                    if std::cmp::max(from_dtb, to_dtb) < outer_edge_filter_length {
                        // SkeletalTrapezoidation.cpp:639 edge.data.setIsCentral(false);
                        edge.base.data.set_is_central(false);
                        iscprobe(2, false, outer_edge_filter_length, cap);
                    } else {
                        // SkeletalTrapezoidation.cpp:643 Point a = edge.from->p;
                        let a = edge.base.from.unwrap().as_ref().p;
                        // SkeletalTrapezoidation.cpp:644 Point b = edge.to->p;
                        let b = edge.base.to.unwrap().as_ref().p;
                        // SkeletalTrapezoidation.cpp:645 Point ab = b - a;
                        let ab = b - a;
                        // SkeletalTrapezoidation.cpp:646 coord_t dR = std::abs(edge.to->...dtb - edge.from->...dtb);
                        let d_r = (to_dtb - from_dtb).abs();
                        // SkeletalTrapezoidation.cpp:647 coord_t dD = ab.cast<int64_t>().norm();
                        let d_d = ab.length() as Coord;
                        // SkeletalTrapezoidation.cpp:648 edge.data.setIsCentral(dR < dD * cap);
                        edge.base.data.set_is_central((d_r as f64) < (d_d as f64) * cap);
                        iscprobe(
                            3,
                            (d_r as f64) < (d_d as f64) * cap,
                            outer_edge_filter_length,
                            cap,
                        );
                        geomprobe(d_r, d_d, cap);
                    }
                }
            }
        }
    }

    // SkeletalTrapezoidation.cpp:653-662
    // void SkeletalTrapezoidation::filterCentral(coord_t max_length)
    pub fn filter_central(&mut self, max_length: Coord) {
        unsafe {
            // SkeletalTrapezoidation.cpp:655 for (edge_t& edge : graph.edges)
            let edges: Vec<EdgePtr> = self
                .graph
                .edges
                .iter()
                .map(|e| SkeletalTrapezoidationGraph::edge_ptr(e))
                .collect();
            for edge in edges {
                let edge_st = as_st_edge(edge);
                let to = edge.as_ref().to.unwrap();
                // SkeletalTrapezoidation.cpp:657 if (isEndOfCentral(edge) && edge.to->isLocalMaximum() && !edge.to->isLocalMaximum())
                if self.is_end_of_central(edge)
                    && as_st_node(to).is_local_maximum(false)
                    && !as_st_node(to).is_local_maximum(false)
                {
                    // SkeletalTrapezoidation.cpp:659 filterCentral(edge.twin, 0, max_length);
                    let _ = edge_st; // edge_st unused except as readability anchor
                    self.filter_central_rec(edge.as_ref().twin.unwrap(), 0, max_length);
                }
            }
        }
    }

    // SkeletalTrapezoidation.cpp:664-688
    // bool SkeletalTrapezoidation::filterCentral(edge_t* starting_edge, coord_t traveled_dist, coord_t max_length)
    pub fn filter_central_rec(
        &mut self,
        starting_edge: EdgePtr,
        traveled_dist: Coord,
        max_length: Coord,
    ) -> bool {
        unsafe {
            let se = starting_edge.as_ref();
            // SkeletalTrapezoidation.cpp:666 coord_t length = (starting_edge->from->p - starting_edge->to->p).cast<int64_t>().norm();
            let length = (se.from.unwrap().as_ref().p - se.to.unwrap().as_ref().p).length() as Coord;
            // SkeletalTrapezoidation.cpp:667 if (traveled_dist + length > max_length)
            if traveled_dist + length > max_length {
                // SkeletalTrapezoidation.cpp:669 return false;
                return false;
            }

            // SkeletalTrapezoidation.cpp:672 bool should_dissolve = true;
            let mut should_dissolve = true;
            // SkeletalTrapezoidation.cpp:673 for (edge_t* next_edge = starting_edge->next; next_edge && next_edge != starting_edge->twin; next_edge = next_edge->twin->next)
            let twin = se.twin;
            let mut next_edge = se.next;
            while next_edge.is_some() && next_edge != twin {
                let ne = next_edge.unwrap();
                // SkeletalTrapezoidation.cpp:675 if (next_edge->data.isCentral())
                if ne.as_ref().data.is_central() {
                    // SkeletalTrapezoidation.cpp:677 should_dissolve &= filterCentral(next_edge, traveled_dist + length, max_length);
                    should_dissolve &= self.filter_central_rec(ne, traveled_dist + length, max_length);
                }
                next_edge = ne.as_ref().twin.unwrap().as_ref().next;
            }

            // SkeletalTrapezoidation.cpp:681 should_dissolve &= !starting_edge->to->isLocalMaximum();
            should_dissolve &= !as_st_node(se.to.unwrap()).is_local_maximum(false);
            // SkeletalTrapezoidation.cpp:682 if (should_dissolve)
            if should_dissolve {
                // SkeletalTrapezoidation.cpp:684 starting_edge->data.setIsCentral(false);
                starting_edge.as_ptr().as_mut().unwrap().data.set_is_central(false);
                // SkeletalTrapezoidation.cpp:685 starting_edge->twin->data.setIsCentral(false);
                se.twin.unwrap().as_ptr().as_mut().unwrap().data.set_is_central(false);
            }
            // SkeletalTrapezoidation.cpp:687 return should_dissolve;
            should_dissolve
        }
    }

    // SkeletalTrapezoidation.cpp:690-700
    // void SkeletalTrapezoidation::filterOuterCentral()
    pub fn filter_outer_central(&mut self) {
        unsafe {
            // SkeletalTrapezoidation.cpp:692 for (edge_t& edge : graph.edges)
            for edge in self.graph.edges.iter_mut() {
                // SkeletalTrapezoidation.cpp:694 if (!edge.prev)
                if edge.base.prev.is_none() {
                    // SkeletalTrapezoidation.cpp:696 edge.data.setIsCentral(false);
                    edge.base.data.set_is_central(false);
                    // SkeletalTrapezoidation.cpp:697 edge.twin->data.setIsCentral(false);
                    edge.base.twin.unwrap().as_ptr().as_mut().unwrap().data.set_is_central(false);
                }
            }
        }
    }

    // SkeletalTrapezoidation.cpp:702-731
    // void SkeletalTrapezoidation::updateBeadCount()
    pub fn update_bead_count(&mut self) {
        unsafe {
            // SkeletalTrapezoidation.cpp:704 for (edge_t& edge : graph.edges)
            for edge in self.graph.edges.iter_mut() {
                // SkeletalTrapezoidation.cpp:706 if (edge.data.isCentral())
                if edge.base.data.is_central() {
                    let to = edge.base.to.unwrap();
                    let dtb = to.as_ref().data.distance_to_boundary;
                    // SkeletalTrapezoidation.cpp:708 edge.to->data.bead_count = beading_strategy.getOptimalBeadCount(edge.to->data.distance_to_boundary * 2);
                    to.as_ptr().as_mut().unwrap().data.bead_count =
                        self.beading_strategy.get_optimal_bead_count(dtb * 2);
                }
            }

            // SkeletalTrapezoidation.cpp:712-730 Fix bead count at locally maximal R
            let nodes: Vec<NodePtr> = self
                .graph
                .nodes
                .iter()
                .map(|n| SkeletalTrapezoidationGraph::node_ptr(n))
                .collect();
            // SkeletalTrapezoidation.cpp:713 for (node_t& node : graph.nodes)
            for node in nodes {
                // SkeletalTrapezoidation.cpp:715 if (node.isLocalMaximum())
                if as_st_node(node).is_local_maximum(false) {
                    // SkeletalTrapezoidation.cpp:717 if (node.data.distance_to_boundary < 0)
                    if node.as_ref().data.distance_to_boundary < 0 {
                        log::warn!("Distance to boundary not yet computed for local maximum!");
                        // SkeletalTrapezoidation.cpp:720 node.data.distance_to_boundary = std::numeric_limits<coord_t>::max();
                        node.as_ptr().as_mut().unwrap().data.distance_to_boundary = Coord::MAX;
                        // SkeletalTrapezoidation.cpp:721 edge_t* edge = node.incident_edge;
                        let incident_edge = node.as_ref().incident_edge;
                        let mut edge = incident_edge;
                        // SkeletalTrapezoidation.cpp:722-725 do { ... } while (edge = edge->twin->next, edge != node.incident_edge);
                        loop {
                            let e = edge.unwrap();
                            // SkeletalTrapezoidation.cpp:724 node.data.distance_to_boundary = std::min(..., edge->to->data.distance_to_boundary + coord_t((edge->from->p - edge->to->p).cast<int64_t>().norm()));
                            let edge_to_dtb = e.as_ref().to.unwrap().as_ref().data.distance_to_boundary;
                            let seg_len = (e.as_ref().from.unwrap().as_ref().p
                                - e.as_ref().to.unwrap().as_ref().p)
                                .length() as Coord;
                            let cur = node.as_ref().data.distance_to_boundary;
                            node.as_ptr().as_mut().unwrap().data.distance_to_boundary =
                                std::cmp::min(cur, edge_to_dtb + seg_len);
                            edge = e.as_ref().twin.unwrap().as_ref().next;
                            if edge == incident_edge {
                                break;
                            }
                        }
                    }
                    // SkeletalTrapezoidation.cpp:727 coord_t bead_count = beading_strategy.getOptimalBeadCount(node.data.distance_to_boundary * 2);
                    let bead_count = self
                        .beading_strategy
                        .get_optimal_bead_count(node.as_ref().data.distance_to_boundary * 2);
                    // SkeletalTrapezoidation.cpp:728 node.data.bead_count = bead_count;
                    node.as_ptr().as_mut().unwrap().data.bead_count = bead_count;
                }
            }
        }
    }

    // SkeletalTrapezoidation.cpp:733-749
    // void SkeletalTrapezoidation::filterNoncentralRegions()
    pub fn filter_noncentral_regions(&mut self) {
        unsafe {
            // SkeletalTrapezoidation.cpp:735 for (edge_t& edge : graph.edges)
            let edges: Vec<EdgePtr> = self
                .graph
                .edges
                .iter()
                .map(|e| SkeletalTrapezoidationGraph::edge_ptr(e))
                .collect();
            for edge in edges {
                // SkeletalTrapezoidation.cpp:737 if (!isEndOfCentral(edge))
                if !self.is_end_of_central(edge) {
                    // SkeletalTrapezoidation.cpp:739 continue;
                    continue;
                }
                let to = edge.as_ref().to.unwrap();
                let bead_count = to.as_ref().data.bead_count;
                let dtb = to.as_ref().data.distance_to_boundary;
                // SkeletalTrapezoidation.cpp:741 if(edge.to->data.bead_count < 0 && edge.to->data.distance_to_boundary != 0)
                if bead_count < 0 && dtb != 0 {
                    log::warn!("Encountered an uninitialized bead at the boundary!");
                }
                // SkeletalTrapezoidation.cpp:745 assert(edge.to->data.bead_count >= 0 || edge.to->data.distance_to_boundary == 0);
                debug_assert!(bead_count >= 0 || dtb == 0);
                // SkeletalTrapezoidation.cpp:746 constexpr coord_t max_dist = scaled<coord_t>(0.4);
                let max_dist: Coord = scaled(0.4);
                // SkeletalTrapezoidation.cpp:747 filterNoncentralRegions(&edge, edge.to->data.bead_count, 0, max_dist);
                self.filter_noncentral_regions_rec(edge, bead_count, 0, max_dist);
            }
        }
    }

    // SkeletalTrapezoidation.cpp:751-793
    // bool SkeletalTrapezoidation::filterNoncentralRegions(edge_t* to_edge, coord_t bead_count, coord_t traveled_dist, coord_t max_dist)
    pub fn filter_noncentral_regions_rec(
        &mut self,
        to_edge: EdgePtr,
        bead_count: Coord,
        traveled_dist: Coord,
        max_dist: Coord,
    ) -> bool {
        unsafe {
            // SkeletalTrapezoidation.cpp:753 coord_t r = to_edge->to->data.distance_to_boundary;
            let r = to_edge.as_ref().to.unwrap().as_ref().data.distance_to_boundary;

            // SkeletalTrapezoidation.cpp:755 edge_t* next_edge = to_edge->next;
            let mut next_edge = to_edge.as_ref().next;
            let twin = to_edge.as_ref().twin;
            // SkeletalTrapezoidation.cpp:756 for (; next_edge && next_edge != to_edge->twin; next_edge = next_edge->twin->next)
            while next_edge.is_some() && next_edge != twin {
                let ne = next_edge.unwrap();
                let ne_to_dtb = ne.as_ref().to.unwrap().as_ref().data.distance_to_boundary;
                let seg = ne.as_ref().to.unwrap().as_ref().p - ne.as_ref().from.unwrap().as_ref().p;
                // SkeletalTrapezoidation.cpp:758 if (next_edge->to->data.distance_to_boundary >= r || shorter_then(..., scaled<coord_t>(0.01)))
                if ne_to_dtb >= r || shorter_then(&seg, scaled(0.01)) {
                    // SkeletalTrapezoidation.cpp:760 break;
                    break;
                }
                next_edge = ne.as_ref().twin.unwrap().as_ref().next;
            }
            // SkeletalTrapezoidation.cpp:763 if (next_edge == to_edge->twin || ! next_edge)
            if next_edge == twin || next_edge.is_none() {
                // SkeletalTrapezoidation.cpp:765 return false;
                return false;
            }
            let next_edge = next_edge.unwrap();

            // SkeletalTrapezoidation.cpp:768 const coord_t length = (next_edge->to->p - next_edge->from->p).cast<int64_t>().norm();
            let length = (next_edge.as_ref().to.unwrap().as_ref().p
                - next_edge.as_ref().from.unwrap().as_ref().p)
                .length() as Coord;

            // SkeletalTrapezoidation.cpp:770 bool dissolve = false;
            let dissolve;
            let next_to = next_edge.as_ref().to.unwrap();
            let next_bead_count = next_to.as_ref().data.bead_count;
            // SkeletalTrapezoidation.cpp:771 if (next_edge->to->data.bead_count == bead_count)
            if next_bead_count == bead_count {
                // SkeletalTrapezoidation.cpp:773 dissolve = true;
                dissolve = true;
            }
            // SkeletalTrapezoidation.cpp:775 else if (next_edge->to->data.bead_count < 0)
            else if next_bead_count < 0 {
                // SkeletalTrapezoidation.cpp:777 dissolve = filterNoncentralRegions(next_edge, bead_count, traveled_dist + length, max_dist);
                dissolve = self.filter_noncentral_regions_rec(
                    next_edge,
                    bead_count,
                    traveled_dist + length,
                    max_dist,
                );
            }
            // SkeletalTrapezoidation.cpp:779 else // Upward bead count is different
            else {
                // SkeletalTrapezoidation.cpp:782 dissolve = (traveled_dist + length < max_dist) && std::abs(next_edge->to->data.bead_count - bead_count) == 1;
                dissolve =
                    (traveled_dist + length < max_dist) && (next_bead_count - bead_count).abs() == 1;
            }

            // SkeletalTrapezoidation.cpp:785 if (dissolve)
            if dissolve {
                // SkeletalTrapezoidation.cpp:787 next_edge->data.setIsCentral(true);
                next_edge.as_ptr().as_mut().unwrap().data.set_is_central(true);
                // SkeletalTrapezoidation.cpp:788 next_edge->twin->data.setIsCentral(true);
                next_edge.as_ref().twin.unwrap().as_ptr().as_mut().unwrap().data.set_is_central(true);
                // SkeletalTrapezoidation.cpp:789 next_edge->to->data.bead_count = beading_strategy.getOptimalBeadCount(next_edge->to->data.distance_to_boundary * 2);
                let dtb = next_to.as_ref().data.distance_to_boundary;
                next_to.as_ptr().as_mut().unwrap().data.bead_count =
                    self.beading_strategy.get_optimal_bead_count(dtb * 2);
                // SkeletalTrapezoidation.cpp:790 next_edge->to->data.transition_ratio = 0;
                next_to.as_ptr().as_mut().unwrap().data.transition_ratio = 0.0;
            }
            // SkeletalTrapezoidation.cpp:792 return dissolve;
            dissolve
        }
    }

    // SkeletalTrapezoidation.cpp:795-830
    // void SkeletalTrapezoidation::generateTransitioningRibs()
    pub fn generate_transitioning_ribs(&mut self) {
        // SkeletalTrapezoidation.cpp:799 ptr_vector_t<std::list<TransitionMiddle>> edge_transitions;
        let mut edge_transitions: Vec<Arc<RwLock<Vec<TransitionMiddle>>>> = Vec::new();
        // SkeletalTrapezoidation.cpp:800 generateTransitionMids(edge_transitions);
        self.generate_transition_mids(&mut edge_transitions);
        self.transition_census("0 after generateTransitionMids");

        unsafe {
            // SkeletalTrapezoidation.cpp:802 for (edge_t& edge : graph.edges)
            for edge in self.graph.edges.iter() {
                // SkeletalTrapezoidation.cpp:804 if (edge.data.isCentral() && edge.from->data.bead_count != edge.to->data.bead_count)
                if edge.base.data.is_central()
                    && edge.base.from.unwrap().as_ref().data.bead_count
                        != edge.base.to.unwrap().as_ref().data.bead_count
                {
                    // SkeletalTrapezoidation.cpp:806 assert(edge.data.hasTransitions() || edge.twin->data.hasTransitions());
                    debug_assert!(
                        edge.base.data.has_transitions(false)
                            || edge.base.twin.unwrap().as_ref().data.has_transitions(false)
                    );
                }
            }
        }

        // SkeletalTrapezoidation.cpp:810 filterTransitionMids();
        self.filter_transition_mids();
        self.transition_census("1 after filterTransitionMids");

        // SkeletalTrapezoidation.cpp:817 ptr_vector_t<std::list<TransitionEnd>> edge_transition_ends;
        let mut edge_transition_ends: Vec<Arc<RwLock<Vec<TransitionEnd>>>> = Vec::new();
        // SkeletalTrapezoidation.cpp:818 generateAllTransitionEnds(edge_transition_ends);
        self.generate_all_transition_ends(&mut edge_transition_ends);
        self.transition_census("2 after generateAllTransitionEnds");

        // SkeletalTrapezoidation.cpp:824 applyTransitions(edge_transition_ends);
        self.apply_transitions(&mut edge_transition_ends);
        self.transition_census("3 after applyTransitions");
        // SkeletalTrapezoidation.cpp:825 Note: the shared pointer lists go out of scope and are destroyed here.
        drop(edge_transitions);
    }

    // SkeletalTrapezoidation.cpp:833-911
    // void SkeletalTrapezoidation::generateTransitionMids(ptr_vector_t<std::list<TransitionMiddle>>& edge_transitions)
    pub fn generate_transition_mids(
        &mut self,
        edge_transitions: &mut Vec<Arc<RwLock<Vec<TransitionMiddle>>>>,
    ) {
        unsafe {
            // SkeletalTrapezoidation.cpp:835 for (edge_t& edge : graph.edges)
            for edge in self.graph.edges.iter() {
                // SkeletalTrapezoidation.cpp:837 assert(edge.data.centralIsSet());
                debug_assert!(edge.base.data.central_is_set());
                // SkeletalTrapezoidation.cpp:838 if (!edge.data.isCentral())
                if !edge.base.data.is_central() {
                    // SkeletalTrapezoidation.cpp:840 continue;
                    continue;
                }
                let from = edge.base.from.unwrap();
                let to = edge.base.to.unwrap();
                // SkeletalTrapezoidation.cpp:842 coord_t start_R = edge.from->data.distance_to_boundary;
                let start_r = from.as_ref().data.distance_to_boundary;
                // SkeletalTrapezoidation.cpp:843 coord_t end_R = edge.to->data.distance_to_boundary;
                let end_r = to.as_ref().data.distance_to_boundary;
                // SkeletalTrapezoidation.cpp:844 int start_bead_count = edge.from->data.bead_count;
                let start_bead_count = from.as_ref().data.bead_count;
                // SkeletalTrapezoidation.cpp:845 int end_bead_count = edge.to->data.bead_count;
                let end_bead_count = to.as_ref().data.bead_count;

                // SkeletalTrapezoidation.cpp:847 if (start_R == end_R)
                if start_r == end_r {
                    // SkeletalTrapezoidation.cpp:849 assert(edge.from->data.bead_count == edge.to->data.bead_count);
                    debug_assert!(start_bead_count == end_bead_count);
                    // SkeletalTrapezoidation.cpp:850 if(edge.from->data.bead_count != edge.to->data.bead_count)
                    if start_bead_count != end_bead_count {
                        log::warn!(
                            "Bead count {} is different from {} even though distance to boundary is the same.",
                            start_bead_count,
                            end_bead_count
                        );
                    }
                    // SkeletalTrapezoidation.cpp:854 continue;
                    continue;
                }
                // SkeletalTrapezoidation.cpp:856 else if (start_R > end_R)
                else if start_r > end_r {
                    // SkeletalTrapezoidation.cpp:858 continue;
                    continue;
                }

                // SkeletalTrapezoidation.cpp:861 if (edge.from->data.bead_count == edge.to->data.bead_count)
                if start_bead_count == end_bead_count {
                    // SkeletalTrapezoidation.cpp:863 continue;
                    continue;
                }

                // SkeletalTrapezoidation.cpp:866 if (start_bead_count > beading_strategy.getOptimalBeadCount(start_R * 2) || end_bead_count > beading_strategy.getOptimalBeadCount(end_R * 2))
                if start_bead_count > self.beading_strategy.get_optimal_bead_count(start_r * 2)
                    || end_bead_count > self.beading_strategy.get_optimal_bead_count(end_r * 2)
                {
                    // SkeletalTrapezoidation.cpp:869 BOOST_LOG_TRIVIAL(error) << "transitioning segment overlap! (?)";
                    log::error!("transitioning segment overlap! (?)");
                }
                // SkeletalTrapezoidation.cpp:871 assert(start_R < end_R);
                debug_assert!(start_r < end_r);
                // SkeletalTrapezoidation.cpp:872 if(start_R >= end_R)
                if start_r >= end_r {
                    log::warn!(
                        "Transitioning the wrong way around! This function expects to transition from small R to big R, but was transitioning from {} to {}",
                        start_r,
                        end_r
                    );
                }
                // SkeletalTrapezoidation.cpp:876 coord_t edge_size = (edge.from->p - edge.to->p).cast<int64_t>().norm();
                let edge_size = (from.as_ref().p - to.as_ref().p).length() as Coord;
                // SkeletalTrapezoidation.cpp:877 for (int transition_lower_bead_count = start_bead_count; transition_lower_bead_count < end_bead_count; transition_lower_bead_count++)
                let mut transition_lower_bead_count = start_bead_count;
                while transition_lower_bead_count < end_bead_count {
                    // SkeletalTrapezoidation.cpp:879 coord_t mid_R = beading_strategy.getTransitionThickness(transition_lower_bead_count) / 2;
                    let mut mid_r =
                        self.beading_strategy.get_transition_thickness(transition_lower_bead_count) / 2;
                    // SkeletalTrapezoidation.cpp:880 if (mid_R > end_R)
                    if mid_r > end_r {
                        // SkeletalTrapezoidation.cpp:882 BOOST_LOG_TRIVIAL(error) << "transition on segment lies outside of segment!";
                        log::error!("transition on segment lies outside of segment!");
                        // SkeletalTrapezoidation.cpp:883 mid_R = end_R;
                        mid_r = end_r;
                    }
                    // SkeletalTrapezoidation.cpp:885 if (mid_R < start_R)
                    if mid_r < start_r {
                        // SkeletalTrapezoidation.cpp:887 BOOST_LOG_TRIVIAL(error) << "transition on segment lies outside of segment!";
                        log::error!("transition on segment lies outside of segment!");
                        // SkeletalTrapezoidation.cpp:888 mid_R = start_R;
                        mid_r = start_r;
                    }
                    // SkeletalTrapezoidation.cpp:890 coord_t mid_pos = int64_t(edge_size) * int64_t(mid_R - start_R) / int64_t(end_R - start_R);
                    let mid_pos = (edge_size as i64) * ((mid_r - start_r) as i64)
                        / ((end_r - start_r) as i64);

                    // SkeletalTrapezoidation.cpp:892 assert(mid_pos >= 0);
                    debug_assert!(mid_pos >= 0);
                    // SkeletalTrapezoidation.cpp:893 assert(mid_pos <= edge_size);
                    debug_assert!(mid_pos <= edge_size);
                    // SkeletalTrapezoidation.cpp:894 if(mid_pos < 0 || mid_pos > edge_size)
                    if mid_pos < 0 || mid_pos > edge_size {
                        log::warn!("Transition mid is out of bounds of the edge.");
                    }
                    // SkeletalTrapezoidation.cpp:898 auto transitions = edge.data.getTransitions();
                    let mut transitions = edge.base.data.get_transitions();
                    // SkeletalTrapezoidation.cpp:899 constexpr bool ignore_empty = true;
                    let ignore_empty = true;
                    // SkeletalTrapezoidation.cpp:900 assert((! edge.data.hasTransitions(ignore_empty)) || mid_pos >= transitions->back().pos);
                    debug_assert!(
                        !edge.base.data.has_transitions(ignore_empty)
                            || mid_pos
                                >= transitions
                                    .as_ref()
                                    .map(|t| t.read().last().map(|m| m.pos).unwrap_or(0))
                                    .unwrap_or(0)
                    );
                    // SkeletalTrapezoidation.cpp:901 if (! edge.data.hasTransitions(ignore_empty))
                    if !edge.base.data.has_transitions(ignore_empty) {
                        // SkeletalTrapezoidation.cpp:903 edge_transitions.emplace_back(std::make_shared<std::list<TransitionMiddle>>());
                        let new_list = Arc::new(RwLock::new(Vec::<TransitionMiddle>::new()));
                        edge_transitions.push(new_list.clone());
                        // SkeletalTrapezoidation.cpp:904 edge.data.setTransitions(edge_transitions.back());
                        SkeletalTrapezoidationGraph::edge_ptr(edge)
                            .as_ptr()
                            .as_mut()
                            .unwrap()
                            .data
                            .set_transitions(new_list);
                        // SkeletalTrapezoidation.cpp:905 transitions = edge.data.getTransitions();
                        transitions = edge.base.data.get_transitions();
                    }
                    // SkeletalTrapezoidation.cpp:907 transitions->emplace_back(mid_pos, transition_lower_bead_count, mid_R);
                    transitions.unwrap().write().push(TransitionMiddle::new(
                        mid_pos,
                        transition_lower_bead_count as i32,
                        mid_r,
                    ));

                    transition_lower_bead_count += 1;
                }
                // SkeletalTrapezoidation.cpp:909 assert((edge.from->data.bead_count == edge.to->data.bead_count) || edge.data.hasTransitions());
                debug_assert!(
                    start_bead_count == end_bead_count || edge.base.data.has_transitions(false)
                );
            }
        }
    }

    // SkeletalTrapezoidation.cpp:913-980
    // void SkeletalTrapezoidation::filterTransitionMids()
    pub fn filter_transition_mids(&mut self) {
        unsafe {
            // SkeletalTrapezoidation.cpp:915 for (edge_t& edge : graph.edges)
            let edges: Vec<EdgePtr> = self
                .graph
                .edges
                .iter()
                .map(|e| SkeletalTrapezoidationGraph::edge_ptr(e))
                .collect();
            for edge in edges {
                let edge_ref = edge.as_ref();
                // SkeletalTrapezoidation.cpp:917 if (! edge.data.hasTransitions())
                if !edge_ref.data.has_transitions(false) {
                    // SkeletalTrapezoidation.cpp:919 continue;
                    continue;
                }
                // SkeletalTrapezoidation.cpp:921 auto& transitions = *edge.data.getTransitions();
                let transitions_arc = edge_ref.data.get_transitions().unwrap();

                // SkeletalTrapezoidation.cpp:924 assert(transitions.front().lower_bead_count <= transitions.back().lower_bead_count);
                // SkeletalTrapezoidation.cpp:925 assert(edge.from->data.distance_to_boundary <= edge.to->data.distance_to_boundary);
                debug_assert!(
                    edge_ref.from.unwrap().as_ref().data.distance_to_boundary
                        <= edge_ref.to.unwrap().as_ref().data.distance_to_boundary
                );

                // SkeletalTrapezoidation.cpp:927 const Point a = edge.from->p;
                let a = edge_ref.from.unwrap().as_ref().p;
                // SkeletalTrapezoidation.cpp:928 const Point b = edge.to->p;
                let b = edge_ref.to.unwrap().as_ref().p;
                // SkeletalTrapezoidation.cpp:929 Point ab = b - a;
                let ab = b - a;
                // SkeletalTrapezoidation.cpp:930 coord_t ab_size = ab.cast<int64_t>().norm();
                let ab_size = ab.length() as Coord;

                // Snapshot of the back/front transition (pos + lower_bead_count) is taken before the
                // dissolve walks, mirroring the C++ which holds `transitions.back()` by reference.
                // SkeletalTrapezoidation.cpp:932 bool going_up = true;
                let mut going_up = true;
                let back = {
                    let t = transitions_arc.read();
                    t.last().cloned().unwrap()
                };
                // SkeletalTrapezoidation.cpp:933 std::list<TransitionMidRef> to_be_dissolved_back = dissolveNearbyTransitions(&edge, transitions.back(), ab_size - transitions.back().pos, transition_filter_dist, going_up);
                let to_be_dissolved_back = self.dissolve_nearby_transitions(
                    edge,
                    &back,
                    ab_size - back.pos,
                    self.transition_filter_dist,
                    going_up,
                );
                // SkeletalTrapezoidation.cpp:934 bool should_dissolve_back = !to_be_dissolved_back.empty();
                let mut should_dissolve_back = !to_be_dissolved_back.is_empty();
                // SkeletalTrapezoidation.cpp:935 for (TransitionMidRef& ref : to_be_dissolved_back)
                for r in &to_be_dissolved_back {
                    // SkeletalTrapezoidation.cpp:937 dissolveBeadCountRegion(&edge, transitions.back().lower_bead_count + 1, transitions.back().lower_bead_count);
                    self.dissolve_bead_count_region(
                        edge,
                        (back.lower_bead_count + 1) as Coord,
                        back.lower_bead_count as Coord,
                    );
                    // SkeletalTrapezoidation.cpp:938 ref.edge->data.getTransitions()->erase(ref.transition_it);
                    r.edge.as_ref().data.get_transitions().unwrap().write().remove(r.transition_idx);
                }

                {
                    // SkeletalTrapezoidation.cpp:942 coord_t trans_bead_count = transitions.back().lower_bead_count;
                    let trans_bead_count = back.lower_bead_count as Coord;
                    // SkeletalTrapezoidation.cpp:943 coord_t upper_transition_half_length = (1.0 - beading_strategy.getTransitionAnchorPos(trans_bead_count)) * beading_strategy.getTransitioningLength(trans_bead_count);
                    let upper_transition_half_length = ((1.0
                        - self.beading_strategy.get_transition_anchor_pos(trans_bead_count) as f64)
                        * self.beading_strategy.get_transitioning_length(trans_bead_count) as f64)
                        as Coord;
                    // SkeletalTrapezoidation.cpp:944 should_dissolve_back |= filterEndOfCentralTransition(&edge, ab_size - transitions.back().pos, upper_transition_half_length, trans_bead_count);
                    should_dissolve_back |= self.filter_end_of_central_transition(
                        edge,
                        ab_size - back.pos,
                        upper_transition_half_length,
                        trans_bead_count,
                    );
                }

                // SkeletalTrapezoidation.cpp:947 if (should_dissolve_back)
                if should_dissolve_back {
                    // SkeletalTrapezoidation.cpp:949 transitions.pop_back();
                    transitions_arc.write().pop();
                }
                // SkeletalTrapezoidation.cpp:951 if (transitions.empty())
                if transitions_arc.read().is_empty() {
                    // SkeletalTrapezoidation.cpp:953 continue;
                    continue;
                }

                // SkeletalTrapezoidation.cpp:956 going_up = false;
                going_up = false;
                let front = {
                    let t = transitions_arc.read();
                    t.first().cloned().unwrap()
                };
                // SkeletalTrapezoidation.cpp:957 std::list<TransitionMidRef> to_be_dissolved_front = dissolveNearbyTransitions(edge.twin, transitions.front(), transitions.front().pos, transition_filter_dist, going_up);
                let to_be_dissolved_front = self.dissolve_nearby_transitions(
                    edge_ref.twin.unwrap(),
                    &front,
                    front.pos,
                    self.transition_filter_dist,
                    going_up,
                );
                // SkeletalTrapezoidation.cpp:958 bool should_dissolve_front = !to_be_dissolved_front.empty();
                let mut should_dissolve_front = !to_be_dissolved_front.is_empty();
                // SkeletalTrapezoidation.cpp:959 for (TransitionMidRef& ref : to_be_dissolved_front)
                for r in &to_be_dissolved_front {
                    // SkeletalTrapezoidation.cpp:961 dissolveBeadCountRegion(edge.twin, transitions.front().lower_bead_count, transitions.front().lower_bead_count + 1);
                    self.dissolve_bead_count_region(
                        edge_ref.twin.unwrap(),
                        front.lower_bead_count as Coord,
                        (front.lower_bead_count + 1) as Coord,
                    );
                    // SkeletalTrapezoidation.cpp:962 ref.edge->data.getTransitions()->erase(ref.transition_it);
                    r.edge.as_ref().data.get_transitions().unwrap().write().remove(r.transition_idx);
                }

                {
                    // SkeletalTrapezoidation.cpp:966 coord_t trans_bead_count = transitions.front().lower_bead_count;
                    let trans_bead_count = front.lower_bead_count as Coord;
                    // SkeletalTrapezoidation.cpp:967 coord_t lower_transition_half_length = beading_strategy.getTransitionAnchorPos(trans_bead_count) * beading_strategy.getTransitioningLength(trans_bead_count);
                    let lower_transition_half_length = (self
                        .beading_strategy
                        .get_transition_anchor_pos(trans_bead_count)
                        as f64
                        * self.beading_strategy.get_transitioning_length(trans_bead_count) as f64)
                        as Coord;
                    // SkeletalTrapezoidation.cpp:968 should_dissolve_front |= filterEndOfCentralTransition(edge.twin, transitions.front().pos, lower_transition_half_length, trans_bead_count + 1);
                    should_dissolve_front |= self.filter_end_of_central_transition(
                        edge_ref.twin.unwrap(),
                        front.pos,
                        lower_transition_half_length,
                        trans_bead_count + 1,
                    );
                }

                // SkeletalTrapezoidation.cpp:971 if (should_dissolve_front)
                if should_dissolve_front {
                    // SkeletalTrapezoidation.cpp:973 transitions.pop_front();
                    transitions_arc.write().remove(0);
                }
                // SkeletalTrapezoidation.cpp:975 if (transitions.empty())
                if transitions_arc.read().is_empty() {
                    // SkeletalTrapezoidation.cpp:977 continue;
                    continue;
                }
            }
        }
    }

    // SkeletalTrapezoidation.cpp:982-1039
    // std::list<SkeletalTrapezoidation::TransitionMidRef> SkeletalTrapezoidation::dissolveNearbyTransitions(edge_t* edge_to_start, TransitionMiddle& origin_transition, coord_t traveled_dist, coord_t max_dist, bool going_up)
    pub fn dissolve_nearby_transitions(
        &mut self,
        edge_to_start: EdgePtr,
        origin_transition: &TransitionMiddle,
        traveled_dist: Coord,
        max_dist: Coord,
        going_up: bool,
    ) -> Vec<TransitionMidRef> {
        unsafe {
            // SkeletalTrapezoidation.cpp:984 std::list<TransitionMidRef> to_be_dissolved;
            let mut to_be_dissolved: Vec<TransitionMidRef> = Vec::new();
            // SkeletalTrapezoidation.cpp:985 if (traveled_dist > max_dist)
            if traveled_dist > max_dist {
                // SkeletalTrapezoidation.cpp:986 return to_be_dissolved;
                return to_be_dissolved;
            }

            // SkeletalTrapezoidation.cpp:988 bool should_dissolve = true;
            let mut should_dissolve = true;
            // SkeletalTrapezoidation.cpp:989 for (edge_t* edge = edge_to_start->next; edge && edge != edge_to_start->twin; edge = edge->twin->next)
            let twin = edge_to_start.as_ref().twin;
            let mut edge_opt = edge_to_start.as_ref().next;
            while edge_opt.is_some() && edge_opt != twin {
                let edge = edge_opt.unwrap();
                let edge_ref = edge.as_ref();
                // advance pointer is computed at loop end
                // SkeletalTrapezoidation.cpp:990 if (!edge->data.isCentral())
                if !edge_ref.data.is_central() {
                    // SkeletalTrapezoidation.cpp:991 continue;
                    edge_opt = edge_ref.twin.unwrap().as_ref().next;
                    continue;
                }

                // SkeletalTrapezoidation.cpp:993 Point a = edge->from->p;
                let a = edge_ref.from.unwrap().as_ref().p;
                // SkeletalTrapezoidation.cpp:994 Point b = edge->to->p;
                let b = edge_ref.to.unwrap().as_ref().p;
                // SkeletalTrapezoidation.cpp:995 Point ab = b - a;
                let ab = b - a;
                // SkeletalTrapezoidation.cpp:996 coord_t ab_size = ab.cast<int64_t>().norm();
                let ab_size = ab.length() as Coord;
                // SkeletalTrapezoidation.cpp:997 bool is_aligned = edge->isUpward();
                let is_aligned = as_st_edge(edge).is_upward();
                // SkeletalTrapezoidation.cpp:998 edge_t* aligned_edge = is_aligned? edge : edge->twin;
                let aligned_edge = if is_aligned { edge } else { edge_ref.twin.unwrap() };
                // SkeletalTrapezoidation.cpp:999 bool seen_transition_on_this_edge = false;
                let mut seen_transition_on_this_edge = false;

                // SkeletalTrapezoidation.cpp:1001 const coord_t origin_radius = origin_transition.feature_radius;
                let origin_radius = origin_transition.feature_radius;
                // SkeletalTrapezoidation.cpp:1002 const coord_t radius_here = edge->from->data.distance_to_boundary;
                let radius_here = edge_ref.from.unwrap().as_ref().data.distance_to_boundary;
                // SkeletalTrapezoidation.cpp:1003 const bool dissolve_result_is_odd = bool(origin_transition.lower_bead_count % 2) == going_up;
                let dissolve_result_is_odd =
                    (origin_transition.lower_bead_count % 2 != 0) == going_up;
                // SkeletalTrapezoidation.cpp:1004 const coord_t width_deviation = std::abs(origin_radius - radius_here) * 2;
                let width_deviation = (origin_radius - radius_here).abs() * 2;
                // SkeletalTrapezoidation.cpp:1005 const coord_t line_width_deviation = dissolve_result_is_odd ? width_deviation : width_deviation / 2;
                let line_width_deviation = if dissolve_result_is_odd {
                    width_deviation
                } else {
                    width_deviation / 2
                };
                // SkeletalTrapezoidation.cpp:1006 if (line_width_deviation > allowed_filter_deviation)
                if line_width_deviation > self.allowed_filter_deviation {
                    // SkeletalTrapezoidation.cpp:1007 should_dissolve = false;
                    should_dissolve = false;
                }

                // SkeletalTrapezoidation.cpp:1009 if (should_dissolve && aligned_edge->data.hasTransitions())
                if should_dissolve && aligned_edge.as_ref().data.has_transitions(false) {
                    // SkeletalTrapezoidation.cpp:1010 auto& transitions = *aligned_edge->data.getTransitions();
                    let transitions_arc = aligned_edge.as_ref().data.get_transitions().unwrap();
                    let transitions = transitions_arc.read();
                    // SkeletalTrapezoidation.cpp:1011 for (auto transition_it = transitions.begin(); transition_it != transitions.end(); ++transition_it)
                    for (transition_idx, transition_it) in transitions.iter().enumerate() {
                        // SkeletalTrapezoidation.cpp:1013 coord_t pos = is_aligned? transition_it->pos : ab_size - transition_it->pos;
                        let pos = if is_aligned {
                            transition_it.pos
                        } else {
                            ab_size - transition_it.pos
                        };
                        // SkeletalTrapezoidation.cpp:1014 if (traveled_dist + pos < max_dist && transition_it->lower_bead_count == origin_transition.lower_bead_count)
                        if traveled_dist + pos < max_dist
                            && transition_it.lower_bead_count == origin_transition.lower_bead_count
                        {
                            // SkeletalTrapezoidation.cpp:1015 if (traveled_dist + pos < beading_strategy.getTransitioningLength(transition_it->lower_bead_count))
                            if traveled_dist + pos
                                < self
                                    .beading_strategy
                                    .get_transitioning_length(transition_it.lower_bead_count as Coord)
                            {
                                // SkeletalTrapezoidation.cpp:1017 assert(going_up != is_aligned || transition_it->lower_bead_count == 0);
                                debug_assert!(
                                    going_up != is_aligned || transition_it.lower_bead_count == 0
                                );
                            }
                            // SkeletalTrapezoidation.cpp:1019 to_be_dissolved.emplace_back(aligned_edge, transition_it);
                            to_be_dissolved
                                .push(TransitionMidRef::new(aligned_edge, transition_idx));
                            // SkeletalTrapezoidation.cpp:1020 seen_transition_on_this_edge = true;
                            seen_transition_on_this_edge = true;
                        }
                    }
                }
                // SkeletalTrapezoidation.cpp:1024 if (should_dissolve && !seen_transition_on_this_edge)
                if should_dissolve && !seen_transition_on_this_edge {
                    // SkeletalTrapezoidation.cpp:1025 std::list<...> to_be_dissolved_here = dissolveNearbyTransitions(edge, origin_transition, traveled_dist + ab_size, max_dist, going_up);
                    let mut to_be_dissolved_here = self.dissolve_nearby_transitions(
                        edge,
                        origin_transition,
                        traveled_dist + ab_size,
                        max_dist,
                        going_up,
                    );
                    // SkeletalTrapezoidation.cpp:1026 if (to_be_dissolved_here.empty())
                    if to_be_dissolved_here.is_empty() {
                        // SkeletalTrapezoidation.cpp:1027 to_be_dissolved.clear();
                        to_be_dissolved.clear();
                        // SkeletalTrapezoidation.cpp:1028 return to_be_dissolved;
                        return to_be_dissolved;
                    }
                    // SkeletalTrapezoidation.cpp:1030 to_be_dissolved.splice(to_be_dissolved.end(), to_be_dissolved_here);
                    to_be_dissolved.append(&mut to_be_dissolved_here);
                    // SkeletalTrapezoidation.cpp:1031 should_dissolve = should_dissolve && !to_be_dissolved.empty();
                    should_dissolve = should_dissolve && !to_be_dissolved.is_empty();
                }

                edge_opt = edge_ref.twin.unwrap().as_ref().next;
            }

            // SkeletalTrapezoidation.cpp:1035 if (!should_dissolve)
            if !should_dissolve {
                // SkeletalTrapezoidation.cpp:1036 to_be_dissolved.clear();
                to_be_dissolved.clear();
            }

            // SkeletalTrapezoidation.cpp:1038 return to_be_dissolved;
            to_be_dissolved
        }
    }

    // SkeletalTrapezoidation.cpp:1042-1057
    // void SkeletalTrapezoidation::dissolveBeadCountRegion(edge_t* edge_to_start, coord_t from_bead_count, coord_t to_bead_count)
    pub fn dissolve_bead_count_region(
        &mut self,
        edge_to_start: EdgePtr,
        from_bead_count: Coord,
        to_bead_count: Coord,
    ) {
        unsafe {
            // SkeletalTrapezoidation.cpp:1044 assert(from_bead_count != to_bead_count);
            debug_assert!(from_bead_count != to_bead_count);
            let to = edge_to_start.as_ref().to.unwrap();
            // SkeletalTrapezoidation.cpp:1045 if (edge_to_start->to->data.bead_count != from_bead_count)
            if to.as_ref().data.bead_count != from_bead_count {
                // SkeletalTrapezoidation.cpp:1046 return;
                return;
            }

            // SkeletalTrapezoidation.cpp:1048 edge_to_start->to->data.bead_count = to_bead_count;
            to.as_ptr().as_mut().unwrap().data.bead_count = to_bead_count;
            // SkeletalTrapezoidation.cpp:1049 for (edge_t* edge = edge_to_start->next; edge && edge != edge_to_start->twin; edge = edge->twin->next)
            let twin = edge_to_start.as_ref().twin;
            let mut edge_opt = edge_to_start.as_ref().next;
            while edge_opt.is_some() && edge_opt != twin {
                let edge = edge_opt.unwrap();
                // SkeletalTrapezoidation.cpp:1051 if (!edge->data.isCentral())
                if !edge.as_ref().data.is_central() {
                    // SkeletalTrapezoidation.cpp:1053 continue;
                    edge_opt = edge.as_ref().twin.unwrap().as_ref().next;
                    continue;
                }
                // SkeletalTrapezoidation.cpp:1055 dissolveBeadCountRegion(edge, from_bead_count, to_bead_count);
                self.dissolve_bead_count_region(edge, from_bead_count, to_bead_count);
                edge_opt = edge.as_ref().twin.unwrap().as_ref().next;
            }
        }
    }

    // SkeletalTrapezoidation.cpp:1059-1087
    // bool SkeletalTrapezoidation::filterEndOfCentralTransition(edge_t* edge_to_start, coord_t traveled_dist, coord_t max_dist, coord_t replacing_bead_count)
    pub fn filter_end_of_central_transition(
        &mut self,
        edge_to_start: EdgePtr,
        traveled_dist: Coord,
        max_dist: Coord,
        replacing_bead_count: Coord,
    ) -> bool {
        unsafe {
            // SkeletalTrapezoidation.cpp:1061 if (traveled_dist > max_dist)
            if traveled_dist > max_dist {
                // SkeletalTrapezoidation.cpp:1063 return false;
                return false;
            }

            // SkeletalTrapezoidation.cpp:1066 bool is_end_of_central = true;
            let mut is_end_of_central = true;
            // SkeletalTrapezoidation.cpp:1067 bool should_dissolve = false;
            let mut should_dissolve = false;
            // SkeletalTrapezoidation.cpp:1068 for (edge_t* next_edge = edge_to_start->next; next_edge && next_edge != edge_to_start->twin; next_edge = next_edge->twin->next)
            let twin = edge_to_start.as_ref().twin;
            let mut next_edge_opt = edge_to_start.as_ref().next;
            while next_edge_opt.is_some() && next_edge_opt != twin {
                let next_edge = next_edge_opt.unwrap();
                // SkeletalTrapezoidation.cpp:1070 if (next_edge->data.isCentral())
                if next_edge.as_ref().data.is_central() {
                    // SkeletalTrapezoidation.cpp:1072 coord_t length = (next_edge->to->p - next_edge->from->p).cast<int64_t>().norm();
                    let length = (next_edge.as_ref().to.unwrap().as_ref().p
                        - next_edge.as_ref().from.unwrap().as_ref().p)
                        .length() as Coord;
                    // SkeletalTrapezoidation.cpp:1073 should_dissolve |= filterEndOfCentralTransition(next_edge, traveled_dist + length, max_dist, replacing_bead_count);
                    should_dissolve |= self.filter_end_of_central_transition(
                        next_edge,
                        traveled_dist + length,
                        max_dist,
                        replacing_bead_count,
                    );
                    // SkeletalTrapezoidation.cpp:1074 is_end_of_central = false;
                    is_end_of_central = false;
                }
                next_edge_opt = next_edge.as_ref().twin.unwrap().as_ref().next;
            }
            // SkeletalTrapezoidation.cpp:1077 if (is_end_of_central && traveled_dist < max_dist)
            if is_end_of_central && traveled_dist < max_dist {
                // SkeletalTrapezoidation.cpp:1079 should_dissolve = true;
                should_dissolve = true;
            }

            // SkeletalTrapezoidation.cpp:1082 if (should_dissolve)
            if should_dissolve {
                // SkeletalTrapezoidation.cpp:1084 edge_to_start->to->data.bead_count = replacing_bead_count;
                edge_to_start.as_ref().to.unwrap().as_ptr().as_mut().unwrap().data.bead_count =
                    replacing_bead_count;
            }
            // SkeletalTrapezoidation.cpp:1086 return should_dissolve;
            should_dissolve
        }
    }

    // SkeletalTrapezoidation.cpp:1089-1107
    // void SkeletalTrapezoidation::generateAllTransitionEnds(ptr_vector_t<std::list<TransitionEnd>>& edge_transition_ends)
    pub fn generate_all_transition_ends(
        &mut self,
        edge_transition_ends: &mut Vec<Arc<RwLock<Vec<TransitionEnd>>>>,
    ) {
        unsafe {
            // SkeletalTrapezoidation.cpp:1091 for (edge_t& edge : graph.edges)
            let edges: Vec<EdgePtr> = self
                .graph
                .edges
                .iter()
                .map(|e| SkeletalTrapezoidationGraph::edge_ptr(e))
                .collect();
            for edge in edges {
                let edge_ref = edge.as_ref();
                // SkeletalTrapezoidation.cpp:1093 if (! edge.data.hasTransitions())
                if !edge_ref.data.has_transitions(false) {
                    // SkeletalTrapezoidation.cpp:1095 continue;
                    continue;
                }
                // SkeletalTrapezoidation.cpp:1097 auto& transition_positions = *edge.data.getTransitions();
                let transition_positions_arc = edge_ref.data.get_transitions().unwrap();

                // SkeletalTrapezoidation.cpp:1099 assert(edge.from->data.distance_to_boundary <= edge.to->data.distance_to_boundary);
                debug_assert!(
                    edge_ref.from.unwrap().as_ref().data.distance_to_boundary
                        <= edge_ref.to.unwrap().as_ref().data.distance_to_boundary
                );
                // SkeletalTrapezoidation.cpp:1100 for (TransitionMiddle& transition_middle : transition_positions)
                //
                // Snapshot the list because generateTransitionEnds appends to *other*
                // edges' transition-end lists, not this transition-mid list, but the
                // recursion must observe a stable view of the mids.
                let mids: Vec<TransitionMiddle> = transition_positions_arc.read().clone();
                let front_pos = mids.first().map(|m| m.pos).unwrap_or(0);
                let back_pos = mids.last().map(|m| m.pos).unwrap_or(0);
                for transition_middle in &mids {
                    // SkeletalTrapezoidation.cpp:1102 assert(transition_positions.front().pos <= transition_middle.pos);
                    debug_assert!(front_pos <= transition_middle.pos);
                    // SkeletalTrapezoidation.cpp:1103 assert(transition_middle.pos <= transition_positions.back().pos);
                    debug_assert!(transition_middle.pos <= back_pos);
                    // SkeletalTrapezoidation.cpp:1104 generateTransitionEnds(edge, transition_middle.pos, transition_middle.lower_bead_count, edge_transition_ends);
                    self.generate_transition_ends(
                        edge,
                        transition_middle.pos,
                        transition_middle.lower_bead_count as Coord,
                        edge_transition_ends,
                    );
                }
            }
        }
    }

    // SkeletalTrapezoidation.cpp:1109-1144
    // void SkeletalTrapezoidation::generateTransitionEnds(edge_t& edge, coord_t mid_pos, coord_t lower_bead_count, ptr_vector_t<std::list<TransitionEnd>>& edge_transition_ends)
    pub fn generate_transition_ends(
        &mut self,
        edge: EdgePtr,
        mid_pos: Coord,
        lower_bead_count: Coord,
        edge_transition_ends: &mut Vec<Arc<RwLock<Vec<TransitionEnd>>>>,
    ) {
        unsafe {
            // SkeletalTrapezoidation.cpp:1111 const Point a = edge.from->p;
            let a = edge.as_ref().from.unwrap().as_ref().p;
            // SkeletalTrapezoidation.cpp:1112 const Point b = edge.to->p;
            let b = edge.as_ref().to.unwrap().as_ref().p;
            // SkeletalTrapezoidation.cpp:1113 const Point ab = b - a;
            let ab = b - a;
            // SkeletalTrapezoidation.cpp:1114 const coord_t ab_size = ab.cast<int64_t>().norm();
            let ab_size = ab.length() as Coord;

            // SkeletalTrapezoidation.cpp:1116 const coord_t transition_length = beading_strategy.getTransitioningLength(lower_bead_count);
            let transition_length = self.beading_strategy.get_transitioning_length(lower_bead_count);
            // SkeletalTrapezoidation.cpp:1117 const float transition_mid_position = beading_strategy.getTransitionAnchorPos(lower_bead_count);
            let transition_mid_position = self.beading_strategy.get_transition_anchor_pos(lower_bead_count);
            // SkeletalTrapezoidation.cpp:1118 constexpr float inner_bead_width_ratio_after_transition = 1.0;
            let inner_bead_width_ratio_after_transition: f32 = 1.0;

            // SkeletalTrapezoidation.cpp:1120 constexpr coord_t start_rest = 0;
            let start_rest: f64 = 0.0;
            // SkeletalTrapezoidation.cpp:1121 const float mid_rest = transition_mid_position * inner_bead_width_ratio_after_transition;
            let mid_rest = (transition_mid_position * inner_bead_width_ratio_after_transition) as f64;
            // SkeletalTrapezoidation.cpp:1122 constexpr float end_rest = inner_bead_width_ratio_after_transition;
            let end_rest = inner_bead_width_ratio_after_transition as f64;

            // SkeletalTrapezoidation.cpp:1124 { // Lower bead count transition end
                // SkeletalTrapezoidation.cpp:1125 const coord_t start_pos = ab_size - mid_pos;
                let start_pos = ab_size - mid_pos;
                // SkeletalTrapezoidation.cpp:1126 const coord_t transition_half_length = transition_mid_position * int64_t(transition_length);
                let transition_half_length =
                    (transition_mid_position as f64 * transition_length as f64) as Coord;
                // SkeletalTrapezoidation.cpp:1127 const coord_t end_pos = start_pos + transition_half_length;
                let end_pos = start_pos + transition_half_length;
                // SkeletalTrapezoidation.cpp:1128 generateTransitionEnd(*edge.twin, start_pos, end_pos, transition_half_length, mid_rest, start_rest, lower_bead_count, edge_transition_ends);
                self.generate_transition_end(
                    edge.as_ref().twin.unwrap(),
                    start_pos,
                    end_pos,
                    transition_half_length,
                    mid_rest,
                    start_rest,
                    lower_bead_count,
                    edge_transition_ends,
                );
            // SkeletalTrapezoidation.cpp:1129 }

            // SkeletalTrapezoidation.cpp:1131 { // Upper bead count transition end
                // SkeletalTrapezoidation.cpp:1132 const coord_t start_pos = mid_pos;
                let start_pos = mid_pos;
                // SkeletalTrapezoidation.cpp:1133 const coord_t transition_half_length = (1.0 - transition_mid_position) * transition_length;
                let transition_half_length =
                    ((1.0 - transition_mid_position as f64) * transition_length as f64) as Coord;
                // SkeletalTrapezoidation.cpp:1134 const coord_t end_pos = mid_pos + transition_half_length;
                let end_pos = mid_pos + transition_half_length;
                // SkeletalTrapezoidation.cpp:1141 generateTransitionEnd(edge, start_pos, end_pos, transition_half_length, mid_rest, end_rest, lower_bead_count, edge_transition_ends);
                self.generate_transition_end(
                    edge,
                    start_pos,
                    end_pos,
                    transition_half_length,
                    mid_rest,
                    end_rest,
                    lower_bead_count,
                    edge_transition_ends,
                );
            // SkeletalTrapezoidation.cpp:1143 }
        }
    }

    // SkeletalTrapezoidation.cpp:1146-1249
    // bool SkeletalTrapezoidation::generateTransitionEnd(edge_t& edge, coord_t start_pos, coord_t end_pos, coord_t transition_half_length, double start_rest, double end_rest, coord_t lower_bead_count, ptr_vector_t<std::list<TransitionEnd>>& edge_transition_ends)
    #[allow(clippy::too_many_arguments)]
    pub fn generate_transition_end(
        &mut self,
        edge: EdgePtr,
        start_pos: Coord,
        end_pos: Coord,
        transition_half_length: Coord,
        start_rest: f64,
        end_rest: f64,
        lower_bead_count: Coord,
        edge_transition_ends: &mut Vec<Arc<RwLock<Vec<TransitionEnd>>>>,
    ) -> bool {
        unsafe {
            // SkeletalTrapezoidation.cpp:1148 Point a = edge.from->p;
            let a = edge.as_ref().from.unwrap().as_ref().p;
            // SkeletalTrapezoidation.cpp:1149 Point b = edge.to->p;
            let b = edge.as_ref().to.unwrap().as_ref().p;
            // SkeletalTrapezoidation.cpp:1150 Point ab = b - a;
            let ab = b - a;
            // SkeletalTrapezoidation.cpp:1151 coord_t ab_size = ab.cast<int64_t>().norm();
            let ab_size = ab.length() as Coord;

            // SkeletalTrapezoidation.cpp:1153 assert(start_pos <= ab_size);
            debug_assert!(start_pos <= ab_size);
            // SkeletalTrapezoidation.cpp:1154 if(start_pos > ab_size)
            if start_pos > ab_size {
                log::warn!("Start position of edge is beyond edge range.");
            }

            // SkeletalTrapezoidation.cpp:1159 bool going_up = end_rest > start_rest;
            let going_up = end_rest > start_rest;

            // SkeletalTrapezoidation.cpp:1161 assert(edge.data.isCentral());
            debug_assert!(edge.as_ref().data.is_central());
            // SkeletalTrapezoidation.cpp:1162 if (!edge.data.isCentral())
            if !edge.as_ref().data.is_central() {
                log::warn!("This function shouldn't generate ends in or beyond non-central regions.");
                // SkeletalTrapezoidation.cpp:1165 return false;
                return false;
            }

            // SkeletalTrapezoidation.cpp:1168 if (end_pos > ab_size)
            if end_pos > ab_size {
                // SkeletalTrapezoidation.cpp:1170 float rest = end_rest - (start_rest - end_rest) * (end_pos - ab_size) / (start_pos - end_pos);
                // C++ `rest` is `float`: the RHS is evaluated in double then truncated to f32.
                let rest: f64 = (end_rest
                    - (start_rest - end_rest) * (end_pos - ab_size) as f64
                        / (start_pos - end_pos) as f64) as f32 as f64;
                // SkeletalTrapezoidation.cpp:1171-1173 asserts
                debug_assert!(rest >= 0.0);
                debug_assert!(rest <= end_rest.max(start_rest));
                debug_assert!(rest >= end_rest.min(start_rest));

                // SkeletalTrapezoidation.cpp:1175 coord_t central_edge_count = 0;
                let mut central_edge_count: Coord = 0;
                // SkeletalTrapezoidation.cpp:1176 for (edge_t* outgoing = edge.next; outgoing && outgoing != edge.twin; outgoing = outgoing->twin->next)
                let twin = edge.as_ref().twin;
                let mut outgoing_opt = edge.as_ref().next;
                while outgoing_opt.is_some() && outgoing_opt != twin {
                    let outgoing = outgoing_opt.unwrap();
                    // SkeletalTrapezoidation.cpp:1178 if (!outgoing->data.isCentral()) continue;
                    if !outgoing.as_ref().data.is_central() {
                        outgoing_opt = outgoing.as_ref().twin.unwrap().as_ref().next;
                        continue;
                    }
                    // SkeletalTrapezoidation.cpp:1179 central_edge_count++;
                    central_edge_count += 1;
                    outgoing_opt = outgoing.as_ref().twin.unwrap().as_ref().next;
                }

                // SkeletalTrapezoidation.cpp:1182 bool is_only_going_down = true;
                let mut is_only_going_down = true;
                // SkeletalTrapezoidation.cpp:1183 bool has_recursed = false;
                let mut has_recursed = false;
                // SkeletalTrapezoidation.cpp:1184 for (edge_t* outgoing = edge.next; outgoing && outgoing != edge.twin;)
                let mut outgoing_opt = edge.as_ref().next;
                while outgoing_opt.is_some() && outgoing_opt != twin {
                    let outgoing = outgoing_opt.unwrap();
                    // SkeletalTrapezoidation.cpp:1186 edge_t* next = outgoing->twin->next;
                    let next = outgoing.as_ref().twin.unwrap().as_ref().next;
                    // SkeletalTrapezoidation.cpp:1187 if (!outgoing->data.isCentral())
                    if !outgoing.as_ref().data.is_central() {
                        // SkeletalTrapezoidation.cpp:1189 outgoing = next; continue;
                        outgoing_opt = next;
                        continue;
                    }
                    // SkeletalTrapezoidation.cpp:1192 if (central_edge_count > 1 && going_up && isGoingDown(outgoing, 0, end_pos - ab_size + transition_half_length, lower_bead_count))
                    if central_edge_count > 1
                        && going_up
                        && self.is_going_down(
                            outgoing,
                            0,
                            end_pos - ab_size + transition_half_length,
                            lower_bead_count,
                        )
                    {
                        // SkeletalTrapezoidation.cpp:1196 outgoing = next; continue;
                        outgoing_opt = next;
                        continue;
                    }
                    // SkeletalTrapezoidation.cpp:1199 bool is_going_down = generateTransitionEnd(*outgoing, 0, end_pos - ab_size, transition_half_length, rest, end_rest, lower_bead_count, edge_transition_ends);
                    let is_going_down = self.generate_transition_end(
                        outgoing,
                        0,
                        end_pos - ab_size,
                        transition_half_length,
                        rest,
                        end_rest,
                        lower_bead_count,
                        edge_transition_ends,
                    );
                    // SkeletalTrapezoidation.cpp:1200 is_only_going_down &= is_going_down;
                    is_only_going_down &= is_going_down;
                    // SkeletalTrapezoidation.cpp:1201 outgoing = next;
                    outgoing_opt = next;
                    // SkeletalTrapezoidation.cpp:1202 has_recursed = true;
                    has_recursed = true;
                }
                // SkeletalTrapezoidation.cpp:1204 if (!going_up || (has_recursed && !is_only_going_down))
                if !going_up || (has_recursed && !is_only_going_down) {
                    // SkeletalTrapezoidation.cpp:1206 edge.to->data.transition_ratio = rest;
                    edge.as_ref().to.unwrap().as_ptr().as_mut().unwrap().data.transition_ratio =
                        rest as f32;
                    // SkeletalTrapezoidation.cpp:1207 edge.to->data.bead_count = lower_bead_count;
                    edge.as_ref().to.unwrap().as_ptr().as_mut().unwrap().data.bead_count =
                        lower_bead_count;
                }
                // SkeletalTrapezoidation.cpp:1209 return is_only_going_down;
                is_only_going_down
            }
            // SkeletalTrapezoidation.cpp:1211 else // end_pos < ab_size
            else {
                // SkeletalTrapezoidation.cpp:1213 bool is_lower_end = end_rest == 0;
                let is_lower_end = end_rest == 0.0;
                // SkeletalTrapezoidation.cpp:1214 coord_t pos = -1;
                let pos: Coord;

                // SkeletalTrapezoidation.cpp:1216 edge_t* upward_edge = nullptr;
                let upward_edge: EdgePtr;
                // SkeletalTrapezoidation.cpp:1217 if (edge.isUpward())
                if as_st_edge(edge).is_upward() {
                    // SkeletalTrapezoidation.cpp:1219 upward_edge = &edge;
                    upward_edge = edge;
                    // SkeletalTrapezoidation.cpp:1220 pos = end_pos;
                    pos = end_pos;
                } else {
                    // SkeletalTrapezoidation.cpp:1224 upward_edge = edge.twin;
                    upward_edge = edge.as_ref().twin.unwrap();
                    // SkeletalTrapezoidation.cpp:1225 pos = ab_size - end_pos;
                    pos = ab_size - end_pos;
                }

                // SkeletalTrapezoidation.cpp:1228 if(!upward_edge->data.hasTransitionEnds())
                if !upward_edge.as_ref().data.has_transition_ends(false) {
                    // SkeletalTrapezoidation.cpp:1231 edge_transition_ends.emplace_back(std::make_shared<std::list<TransitionEnd>>());
                    let new_list = Arc::new(RwLock::new(Vec::<TransitionEnd>::new()));
                    edge_transition_ends.push(new_list.clone());
                    // SkeletalTrapezoidation.cpp:1232 upward_edge->data.setTransitionEnds(edge_transition_ends.back());
                    upward_edge.as_ptr().as_mut().unwrap().data.set_transition_ends(new_list);
                }
                // SkeletalTrapezoidation.cpp:1234 auto transitions = upward_edge->data.getTransitionEnds();
                let transitions = upward_edge.as_ref().data.get_transition_ends().unwrap();

                // SkeletalTrapezoidation.cpp:1238 assert(pos <= ab_size);
                debug_assert!(pos <= ab_size);
                // SkeletalTrapezoidation.cpp:1239 if (transitions->empty() || pos < transitions->front().pos)
                let front_pos = transitions.read().first().map(|t| t.pos);
                if front_pos.is_none() || pos < front_pos.unwrap() {
                    // SkeletalTrapezoidation.cpp:1241 transitions->emplace_front(pos, lower_bead_count, is_lower_end);
                    transitions.write().insert(
                        0,
                        TransitionEnd::new(pos, lower_bead_count as i32, is_lower_end),
                    );
                } else {
                    // SkeletalTrapezoidation.cpp:1245 transitions->emplace_back(pos, lower_bead_count, is_lower_end);
                    transitions
                        .write()
                        .push(TransitionEnd::new(pos, lower_bead_count as i32, is_lower_end));
                }
                // SkeletalTrapezoidation.cpp:1247 return false;
                false
            }
        }
    }

    // SkeletalTrapezoidation.cpp:1252-1308
    // bool SkeletalTrapezoidation::isGoingDown(edge_t* outgoing, coord_t traveled_dist, coord_t max_dist, coord_t lower_bead_count) const
    pub fn is_going_down(
        &self,
        outgoing: EdgePtr,
        traveled_dist: Coord,
        max_dist: Coord,
        lower_bead_count: Coord,
    ) -> bool {
        unsafe {
            let to = outgoing.as_ref().to.unwrap();
            let from = outgoing.as_ref().from.unwrap();
            // SkeletalTrapezoidation.cpp:1256 if (outgoing->to->data.distance_to_boundary == 0)
            if to.as_ref().data.distance_to_boundary == 0 {
                // SkeletalTrapezoidation.cpp:1258 return true;
                return true;
            }
            // SkeletalTrapezoidation.cpp:1260 bool is_upward = outgoing->to->data.distance_to_boundary >= outgoing->from->data.distance_to_boundary;
            let is_upward =
                to.as_ref().data.distance_to_boundary >= from.as_ref().data.distance_to_boundary;
            // SkeletalTrapezoidation.cpp:1261 edge_t* upward_edge = is_upward? outgoing : outgoing->twin;
            let upward_edge = if is_upward {
                outgoing
            } else {
                outgoing.as_ref().twin.unwrap()
            };
            // SkeletalTrapezoidation.cpp:1262 if (outgoing->to->data.bead_count > lower_bead_count + 1)
            if to.as_ref().data.bead_count > lower_bead_count + 1 {
                // SkeletalTrapezoidation.cpp:1264 assert(upward_edge->data.hasTransitions() ...);
                debug_assert!(
                    upward_edge.as_ref().data.has_transitions(false),
                    "If the bead count is going down there has to be a transition mid!"
                );
                // SkeletalTrapezoidation.cpp:1265 if(!upward_edge->data.hasTransitions())
                if !upward_edge.as_ref().data.has_transitions(false) {
                    log::warn!("If the bead count is going down there has to be a transition mid!");
                }
                // SkeletalTrapezoidation.cpp:1269 return false;
                return false;
            }
            // SkeletalTrapezoidation.cpp:1271 coord_t length = (outgoing->to->p - outgoing->from->p).cast<int64_t>().norm();
            let length = (to.as_ref().p - from.as_ref().p).length() as Coord;
            // SkeletalTrapezoidation.cpp:1272 if (upward_edge->data.hasTransitions())
            if upward_edge.as_ref().data.has_transitions(false) {
                // SkeletalTrapezoidation.cpp:1274 auto& transition_mids = *upward_edge->data.getTransitions();
                let transition_mids_arc = upward_edge.as_ref().data.get_transitions().unwrap();
                let transition_mids = transition_mids_arc.read();
                // SkeletalTrapezoidation.cpp:1275 TransitionMiddle& mid = is_upward? transition_mids.front() : transition_mids.back();
                let mid = if is_upward {
                    transition_mids.first().unwrap()
                } else {
                    transition_mids.last().unwrap()
                };
                // SkeletalTrapezoidation.cpp:1276-1281
                if mid.lower_bead_count as Coord == lower_bead_count
                    && ((is_upward && mid.pos + traveled_dist < max_dist)
                        || (!is_upward && length - mid.pos + traveled_dist < max_dist))
                {
                    // SkeletalTrapezoidation.cpp:1282 return true;
                    return true;
                }
            }
            // SkeletalTrapezoidation.cpp:1285 if (traveled_dist + length > max_dist)
            if traveled_dist + length > max_dist {
                // SkeletalTrapezoidation.cpp:1287 return false;
                return false;
            }
            // SkeletalTrapezoidation.cpp:1289 if (outgoing->to->data.bead_count <= lower_bead_count && !(outgoing->to->data.bead_count == lower_bead_count && outgoing->to->data.transition_ratio > 0.0))
            if to.as_ref().data.bead_count <= lower_bead_count
                && !(to.as_ref().data.bead_count == lower_bead_count
                    && to.as_ref().data.transition_ratio > 0.0)
            {
                // SkeletalTrapezoidation.cpp:1292 return true;
                return true;
            }

            // SkeletalTrapezoidation.cpp:1295 bool is_only_going_down = true;
            let mut is_only_going_down = true;
            // SkeletalTrapezoidation.cpp:1296 bool has_recursed = false;
            let mut has_recursed = false;
            // SkeletalTrapezoidation.cpp:1297 for (edge_t* next = outgoing->next; next && next != outgoing->twin; next = next->twin->next)
            let twin = outgoing.as_ref().twin;
            let mut next_opt = outgoing.as_ref().next;
            while next_opt.is_some() && next_opt != twin {
                let next = next_opt.unwrap();
                // SkeletalTrapezoidation.cpp:1299 if (!next->data.isCentral())
                if !next.as_ref().data.is_central() {
                    // SkeletalTrapezoidation.cpp:1301 continue;
                    next_opt = next.as_ref().twin.unwrap().as_ref().next;
                    continue;
                }
                // SkeletalTrapezoidation.cpp:1303 bool is_going_down = isGoingDown(next, traveled_dist + length, max_dist, lower_bead_count);
                let is_going_down = self.is_going_down(next, traveled_dist + length, max_dist, lower_bead_count);
                // SkeletalTrapezoidation.cpp:1304 is_only_going_down &= is_going_down;
                is_only_going_down &= is_going_down;
                // SkeletalTrapezoidation.cpp:1305 has_recursed = true;
                has_recursed = true;
                next_opt = next.as_ref().twin.unwrap().as_ref().next;
            }
            // SkeletalTrapezoidation.cpp:1307 return has_recursed && is_only_going_down;
            has_recursed && is_only_going_down
        }
    }

    // SkeletalTrapezoidation.cpp:1318-1382
    // void SkeletalTrapezoidation::applyTransitions(ptr_vector_t<std::list<TransitionEnd>>& edge_transition_ends)
    pub fn apply_transitions(
        &mut self,
        edge_transition_ends: &mut Vec<Arc<RwLock<Vec<TransitionEnd>>>>,
    ) {
        unsafe {
            // SkeletalTrapezoidation.cpp:1320 for (edge_t& edge : graph.edges)
            let edges: Vec<EdgePtr> = self
                .graph
                .edges
                .iter()
                .map(|e| SkeletalTrapezoidationGraph::edge_ptr(e))
                .collect();
            for edge in &edges {
                let edge = *edge;
                let edge_ref = edge.as_ref();
                let twin = edge_ref.twin.unwrap();
                // SkeletalTrapezoidation.cpp:1322 if (edge.twin->data.hasTransitionEnds())
                if twin.as_ref().data.has_transition_ends(false) {
                    // SkeletalTrapezoidation.cpp:1324 coord_t length = (edge.from->p - edge.to->p).cast<int64_t>().norm();
                    let length = (edge_ref.from.unwrap().as_ref().p - edge_ref.to.unwrap().as_ref().p)
                        .length() as Coord;
                    // SkeletalTrapezoidation.cpp:1325 auto& twin_transition_ends = *edge.twin->data.getTransitionEnds();
                    let twin_transition_ends = twin.as_ref().data.get_transition_ends().unwrap();
                    // SkeletalTrapezoidation.cpp:1326 if (! edge.data.hasTransitionEnds())
                    if !edge_ref.data.has_transition_ends(false) {
                        // SkeletalTrapezoidation.cpp:1328 edge_transition_ends.emplace_back(std::make_shared<std::list<TransitionEnd>>());
                        let new_list = Arc::new(RwLock::new(Vec::<TransitionEnd>::new()));
                        edge_transition_ends.push(new_list.clone());
                        // SkeletalTrapezoidation.cpp:1329 edge.data.setTransitionEnds(edge_transition_ends.back());
                        edge.as_ptr().as_mut().unwrap().data.set_transition_ends(new_list);
                    }
                    // SkeletalTrapezoidation.cpp:1331 auto& transition_ends = *edge.data.getTransitionEnds();
                    let transition_ends = edge_ref.data.get_transition_ends().unwrap();
                    // SkeletalTrapezoidation.cpp:1332 for (TransitionEnd& end : twin_transition_ends)
                    let twin_ends: Vec<TransitionEnd> = twin_transition_ends.read().clone();
                    for end in &twin_ends {
                        // SkeletalTrapezoidation.cpp:1334 transition_ends.emplace_back(length - end.pos, end.lower_bead_count, end.is_lower_end);
                        transition_ends.write().push(TransitionEnd::new(
                            length - end.pos,
                            end.lower_bead_count,
                            end.is_lower_end,
                        ));
                    }
                    // SkeletalTrapezoidation.cpp:1336 twin_transition_ends.clear();
                    twin_transition_ends.write().clear();
                }
            }

            // SkeletalTrapezoidation.cpp:1340 for (edge_t& edge : graph.edges)
            //
            // insertNode appends to graph.edges / graph.nodes, so snapshot the edge
            // set up front (matches the C++ which iterates the pre-existing list while
            // the new edges are appended after `end()`).
            for edge in &edges {
                let edge = *edge;
                let edge_ref = edge.as_ref();
                // SkeletalTrapezoidation.cpp:1342 if (! edge.data.hasTransitionEnds())
                if !edge_ref.data.has_transition_ends(false) {
                    // SkeletalTrapezoidation.cpp:1344 continue;
                    continue;
                }

                // SkeletalTrapezoidation.cpp:1347 assert(edge.data.isCentral());
                debug_assert!(edge_ref.data.is_central());

                // SkeletalTrapezoidation.cpp:1349 auto& transitions = *edge.data.getTransitionEnds();
                let transitions_arc = edge_ref.data.get_transition_ends().unwrap();
                // SkeletalTrapezoidation.cpp:1350 transitions.sort([](const TransitionEnd& a, const TransitionEnd& b) { return a.pos < b.pos; });
                transitions_arc.write().sort_by(|a, b| a.pos.cmp(&b.pos));

                // SkeletalTrapezoidation.cpp:1352 node_t* from = edge.from;
                let from = edge_ref.from.unwrap();
                // SkeletalTrapezoidation.cpp:1353 node_t* to = edge.to;
                let to = edge_ref.to.unwrap();
                // SkeletalTrapezoidation.cpp:1354 Point a = from->p;
                let a = from.as_ref().p;
                // SkeletalTrapezoidation.cpp:1355 Point b = to->p;
                let b = to.as_ref().p;
                // SkeletalTrapezoidation.cpp:1356 Point ab = b - a;
                let ab = b - a;
                // SkeletalTrapezoidation.cpp:1357 coord_t ab_size = (ab).cast<int64_t>().norm();
                let ab_size = ab.length() as Coord;

                // SkeletalTrapezoidation.cpp:1359 edge_t* last_edge_replacing_input = &edge;
                let mut last_edge_replacing_input = edge;
                // SkeletalTrapezoidation.cpp:1360 for (TransitionEnd& transition_end : transitions)
                let transitions: Vec<TransitionEnd> = transitions_arc.read().clone();
                for transition_end in &transitions {
                    // SkeletalTrapezoidation.cpp:1362 coord_t new_node_bead_count = transition_end.is_lower_end? transition_end.lower_bead_count : transition_end.lower_bead_count + 1;
                    let new_node_bead_count: Coord = if transition_end.is_lower_end {
                        transition_end.lower_bead_count as Coord
                    } else {
                        transition_end.lower_bead_count as Coord + 1
                    };
                    // SkeletalTrapezoidation.cpp:1363 coord_t end_pos = transition_end.pos;
                    let end_pos = transition_end.pos;
                    // SkeletalTrapezoidation.cpp:1364 node_t* close_node = (end_pos < ab_size / 2)? from : to;
                    let close_node = if end_pos < ab_size / 2 { from } else { to };
                    // SkeletalTrapezoidation.cpp:1365 if ((end_pos < snap_dist || end_pos > ab_size - snap_dist) && close_node->data.bead_count == new_node_bead_count)
                    if (end_pos < SNAP_DIST || end_pos > ab_size - SNAP_DIST)
                        && close_node.as_ref().data.bead_count == new_node_bead_count
                    {
                        // SkeletalTrapezoidation.cpp:1369 assert(end_pos <= ab_size);
                        debug_assert!(end_pos <= ab_size);
                        // SkeletalTrapezoidation.cpp:1370 close_node->data.transition_ratio = 0;
                        close_node.as_ptr().as_mut().unwrap().data.transition_ratio = 0.0;
                        // SkeletalTrapezoidation.cpp:1371 continue;
                        continue;
                    }
                    // SkeletalTrapezoidation.cpp:1373 Point mid = a + normal(ab, end_pos);
                    let mid = a + normal(ab, end_pos);

                    // SkeletalTrapezoidation.cpp:1375-1379 asserts + insertNode
                    debug_assert!(last_edge_replacing_input.as_ref().data.is_central());
                    debug_assert!(
                        last_edge_replacing_input.as_ref().data.edge_type != EdgeType::ExtraVd
                    );
                    // SkeletalTrapezoidation.cpp:1377 last_edge_replacing_input = graph.insertNode(last_edge_replacing_input, mid, new_node_bead_count);
                    last_edge_replacing_input =
                        self.graph.insert_node(last_edge_replacing_input, mid, new_node_bead_count);
                    debug_assert!(
                        last_edge_replacing_input.as_ref().data.edge_type != EdgeType::ExtraVd
                    );
                    debug_assert!(last_edge_replacing_input.as_ref().data.is_central());
                }
            }
        }
    }

    // SkeletalTrapezoidation.cpp:1384-1403
    // bool SkeletalTrapezoidation::isEndOfCentral(const edge_t& edge_to) const
    pub fn is_end_of_central(&self, edge_to: EdgePtr) -> bool {
        unsafe {
            // SkeletalTrapezoidation.cpp:1386 if (!edge_to.data.isCentral())
            if !edge_to.as_ref().data.is_central() {
                // SkeletalTrapezoidation.cpp:1388 return false;
                return false;
            }
            // SkeletalTrapezoidation.cpp:1390 if (!edge_to.next)
            if edge_to.as_ref().next.is_none() {
                // SkeletalTrapezoidation.cpp:1392 return true;
                return true;
            }
            // SkeletalTrapezoidation.cpp:1394 for (const edge_t* edge = edge_to.next; edge && edge != edge_to.twin; edge = edge->twin->next)
            let twin = edge_to.as_ref().twin;
            let mut edge_opt = edge_to.as_ref().next;
            while edge_opt.is_some() && edge_opt != twin {
                let edge = edge_opt.unwrap();
                // SkeletalTrapezoidation.cpp:1396 if (edge->data.isCentral())
                if edge.as_ref().data.is_central() {
                    // SkeletalTrapezoidation.cpp:1398 return false;
                    return false;
                }
                // SkeletalTrapezoidation.cpp:1400 assert(edge->twin);
                debug_assert!(edge.as_ref().twin.is_some());
                edge_opt = edge.as_ref().twin.unwrap().as_ref().next;
            }
            // SkeletalTrapezoidation.cpp:1402 return true;
            true
        }
    }

    //
    // ^^^^^^^^^^^^^^^^^^^^^
    //    TRANSTISIONING
    // =====================
    //  TOOLPATH GENERATION
    // vvvvvvvvvvvvvvvvvvvvv
    //

    // SkeletalTrapezoidation.cpp:1477-1581
    // void SkeletalTrapezoidation::generateSegments()
    pub fn generate_segments(&mut self) {
        unsafe {
            // R547: graph size at the same point as the C++ GRAPHPROBE, to test
            // whether our ~5x deficit in `compute` calls is a smaller skeleton or
            // a different share of nodes carrying a bead count.
            if crate::probe_enabled("GRAPHPROBE") {
                let n_up = self
                    .graph
                    .edges
                    .iter()
                    .filter(|e| e.base.prev.is_some() && e.base.next.is_some() && e.is_upward())
                    .count();
                let n_bead = self
                    .graph
                    .nodes
                    .iter()
                    .filter(|n| n.base.data.bead_count > 0)
                    .count();
                graphprobe(self.graph.nodes.len(), self.graph.edges.len(), n_up, n_bead);
            }
            // SkeletalTrapezoidation.cpp:1479 std::vector<edge_t*> upward_quad_mids;
            let mut upward_quad_mids: Vec<EdgePtr> = Vec::new();
            // SkeletalTrapezoidation.cpp:1480 for (edge_t& edge : graph.edges)
            for edge in self.graph.edges.iter() {
                // SkeletalTrapezoidation.cpp:1482 if (edge.prev && edge.next && edge.isUpward())
                if edge.base.prev.is_some() && edge.base.next.is_some() && edge.is_upward() {
                    // SkeletalTrapezoidation.cpp:1484 upward_quad_mids.emplace_back(&edge);
                    upward_quad_mids.push(SkeletalTrapezoidationGraph::edge_ptr(edge));
                }
            }

            // SkeletalTrapezoidation.cpp:1488 std::sort(upward_quad_mids.begin(), upward_quad_mids.end(), [](edge_t* a, edge_t* b) { ... });
            upward_quad_mids.sort_by(|&a, &b| {
                let a_to_dtb = a.as_ref().to.unwrap().as_ref().data.distance_to_boundary;
                let b_to_dtb = b.as_ref().to.unwrap().as_ref().data.distance_to_boundary;
                let a_from_dtb = a.as_ref().from.unwrap().as_ref().data.distance_to_boundary;
                let b_from_dtb = b.as_ref().from.unwrap().as_ref().data.distance_to_boundary;
                // The C++ comparator returns a `bool` (a<b ordering). We translate `less(a,b)`
                // into a std::cmp::Ordering. The `less` body:
                let less = || -> bool {
                    // SkeletalTrapezoidation.cpp:1490 if (a->to->...dtb == b->to->...dtb)
                    if a_to_dtb == b_to_dtb {
                        // SkeletalTrapezoidation.cpp:1492-1493 if (a->from==a->to && b->from==b->to)
                        if a_from_dtb == a_to_dtb && b_from_dtb == b_to_dtb {
                            // SkeletalTrapezoidation.cpp:1495 coord_t max = std::numeric_limits<coord_t>::max();
                            let max = Coord::MAX;
                            // SkeletalTrapezoidation.cpp:1496 a_dist_from_up = std::min(a->distToGoUp(max), a->twin->distToGoUp(max)) - (a->to->p - a->from->p).norm();
                            let a_seg = (a.as_ref().to.unwrap().as_ref().p
                                - a.as_ref().from.unwrap().as_ref().p)
                                .length() as Coord;
                            let a_dist_from_up = std::cmp::min(
                                as_st_edge(a).dist_to_go_up().unwrap_or(max),
                                as_st_edge(a.as_ref().twin.unwrap()).dist_to_go_up().unwrap_or(max),
                            ) - a_seg;
                            // SkeletalTrapezoidation.cpp:1497 b_dist_from_up similarly
                            let b_seg = (b.as_ref().to.unwrap().as_ref().p
                                - b.as_ref().from.unwrap().as_ref().p)
                                .length() as Coord;
                            let b_dist_from_up = std::cmp::min(
                                as_st_edge(b).dist_to_go_up().unwrap_or(max),
                                as_st_edge(b.as_ref().twin.unwrap()).dist_to_go_up().unwrap_or(max),
                            ) - b_seg;
                            // SkeletalTrapezoidation.cpp:1498 return a_dist_from_up < b_dist_from_up;
                            a_dist_from_up < b_dist_from_up
                        }
                        // SkeletalTrapezoidation.cpp:1500 else if (a->from==a->to)
                        else if a_from_dtb == a_to_dtb {
                            // SkeletalTrapezoidation.cpp:1502 return true;
                            true
                        }
                        // SkeletalTrapezoidation.cpp:1504 else if (b->from==b->to)
                        else if b_from_dtb == b_to_dtb {
                            // SkeletalTrapezoidation.cpp:1506 return false;
                            false
                        } else {
                            // SkeletalTrapezoidation.cpp:1510 Ordering is not important -> falls through to line 1513
                            a_to_dtb > b_to_dtb
                        }
                    } else {
                        // SkeletalTrapezoidation.cpp:1513 return a->to->...dtb > b->to->...dtb;
                        a_to_dtb > b_to_dtb
                    }
                };
                if less() {
                    std::cmp::Ordering::Less
                } else {
                    std::cmp::Ordering::Greater
                }
            });

            // SkeletalTrapezoidation.cpp:1516 ptr_vector_t<BeadingPropagation> node_beadings;
            let mut node_beadings: Vec<Arc<RwLock<BeadingPropagation>>> = Vec::new();
            // SkeletalTrapezoidation.cpp:1517 { // Store beading
                // SkeletalTrapezoidation.cpp:1518 for (node_t& node : graph.nodes)
                let nodes: Vec<NodePtr> = self
                    .graph
                    .nodes
                    .iter()
                    .map(|n| SkeletalTrapezoidationGraph::node_ptr(n))
                    .collect();
                for node in nodes {
                    let node_data_bead_count = node.as_ref().data.bead_count;
                    // SkeletalTrapezoidation.cpp:1520 if (node.data.bead_count <= 0)
                    if node_data_bead_count <= 0 {
                        // SkeletalTrapezoidation.cpp:1522 continue;
                        continue;
                    }
                    let dtb = node.as_ref().data.distance_to_boundary;
                    // SkeletalTrapezoidation.cpp:1524 if (node.data.transition_ratio == 0)
                    if node.as_ref().data.transition_ratio == 0.0 {
                        // SkeletalTrapezoidation.cpp:1526 node_beadings.emplace_back(new BeadingPropagation(beading_strategy.compute(node.data.distance_to_boundary * 2, node.data.bead_count)));
                        let _c = self.beading_strategy.compute(dtb * 2, node_data_bead_count);
                        if crate::probe_enabled("BEADPROBE") {
                            beadprobe(dtb * 2, node_data_bead_count, &_c.bead_widths);
                        }
                        let bp = Arc::new(RwLock::new(BeadingPropagation::new(_c)));
                        node_beadings.push(bp.clone());
                        // SkeletalTrapezoidation.cpp:1527 node.data.setBeading(node_beadings.back());
                        node.as_ptr().as_mut().unwrap().data.set_beading(bp.clone());
                        // SkeletalTrapezoidation.cpp:1528 assert(node_beadings.back()->beading.total_thickness == node.data.distance_to_boundary * 2);
                        debug_assert!(bp.read().beading.total_thickness == dtb * 2);
                        // SkeletalTrapezoidation.cpp:1529 if(node_beadings.back()->beading.total_thickness != node.data.distance_to_boundary * 2)
                        if bp.read().beading.total_thickness != dtb * 2 {
                            log::warn!("If transitioning to an endpoint (ratio 0), the node should be exactly in the middle.");
                        }
                    } else {
                        // SkeletalTrapezoidation.cpp:1536 Beading low_count_beading = beading_strategy.compute(node.data.distance_to_boundary * 2, node.data.bead_count);
                        let low_count_beading = self.beading_strategy.compute(dtb * 2, node_data_bead_count);
                        // SkeletalTrapezoidation.cpp:1537 Beading high_count_beading = beading_strategy.compute(node.data.distance_to_boundary * 2, node.data.bead_count + 1);
                        let high_count_beading =
                            self.beading_strategy.compute(dtb * 2, node_data_bead_count + 1);
                        // SkeletalTrapezoidation.cpp:1538 Beading merged = interpolate(low_count_beading, 1.0 - node.data.transition_ratio, high_count_beading);
                        let merged = self.interpolate2(
                            &low_count_beading,
                            1.0 - node.as_ref().data.transition_ratio as f64,
                            &high_count_beading,
                        );
                        // SkeletalTrapezoidation.cpp:1539 node_beadings.emplace_back(new BeadingPropagation(merged));
                        let bp = Arc::new(RwLock::new(BeadingPropagation::new(merged.clone())));
                        node_beadings.push(bp.clone());
                        // SkeletalTrapezoidation.cpp:1540 node.data.setBeading(node_beadings.back());
                        node.as_ptr().as_mut().unwrap().data.set_beading(bp);
                        // SkeletalTrapezoidation.cpp:1541 assert(merged.total_thickness == node.data.distance_to_boundary * 2);
                        debug_assert!(merged.total_thickness == dtb * 2);
                        // SkeletalTrapezoidation.cpp:1542 if(merged.total_thickness != node.data.distance_to_boundary * 2)
                        if merged.total_thickness != dtb * 2 {
                            log::warn!("If merging two beads, the new bead must be exactly in the middle.");
                        }
                    }
                }
            // SkeletalTrapezoidation.cpp:1548 }

            // SkeletalTrapezoidation.cpp:1555 propagateBeadingsUpward(upward_quad_mids, node_beadings);
            self.propagate_beadings_upward(&mut upward_quad_mids, &mut node_beadings);

            // SkeletalTrapezoidation.cpp:1561 propagateBeadingsDownward(upward_quad_mids, node_beadings);
            self.propagate_beadings_downward(&mut upward_quad_mids, &mut node_beadings);

            // SkeletalTrapezoidation.cpp:1567 ptr_vector_t<LineJunctions> edge_junctions;
            let mut edge_junctions: Vec<Arc<RwLock<LineJunctions>>> = Vec::new();
            // SkeletalTrapezoidation.cpp:1568 generateJunctions(node_beadings, edge_junctions);
            self.generate_junctions(&mut node_beadings, &mut edge_junctions);

            // SkeletalTrapezoidation.cpp:1574 connectJunctions(edge_junctions);
            self.connect_junctions(&mut edge_junctions);

            // SkeletalTrapezoidation.cpp:1576 generateLocalMaximaSingleBeads();
            self.generate_local_maxima_single_beads();
        }
    }

    // SkeletalTrapezoidation.cpp:1583-1606
    // SkeletalTrapezoidation::edge_t* SkeletalTrapezoidation::getQuadMaxRedgeTo(edge_t* quad_start_edge)
    pub fn get_quad_max_redge_to(&self, quad_start_edge: EdgePtr) -> EdgePtr {
        unsafe {
            // SkeletalTrapezoidation.cpp:1585 assert(quad_start_edge->prev == nullptr);
            debug_assert!(quad_start_edge.as_ref().prev.is_none());
            // SkeletalTrapezoidation.cpp:1586 assert(quad_start_edge->from->data.distance_to_boundary == 0);
            debug_assert!(
                quad_start_edge.as_ref().from.unwrap().as_ref().data.distance_to_boundary == 0
            );
            // SkeletalTrapezoidation.cpp:1587 coord_t max_R = -1;
            let mut max_r: Coord = -1;
            // SkeletalTrapezoidation.cpp:1588 edge_t* ret = nullptr;
            let mut ret: Option<EdgePtr> = None;
            // SkeletalTrapezoidation.cpp:1589 for (edge_t* edge = quad_start_edge; edge; edge = edge->next)
            let mut edge_opt = Some(quad_start_edge);
            while let Some(edge) = edge_opt {
                // SkeletalTrapezoidation.cpp:1591 coord_t r = edge->to->data.distance_to_boundary;
                let r = edge.as_ref().to.unwrap().as_ref().data.distance_to_boundary;
                // SkeletalTrapezoidation.cpp:1592 if (r > max_R)
                if r > max_r {
                    // SkeletalTrapezoidation.cpp:1594 max_R = r;
                    max_r = r;
                    // SkeletalTrapezoidation.cpp:1595 ret = edge;
                    ret = Some(edge);
                }
                edge_opt = edge.as_ref().next;
            }

            let mut ret = ret.unwrap();
            // SkeletalTrapezoidation.cpp:1599 if (!ret->next && ret->to->data.distance_to_boundary - scaled<coord_t>(0.005) < ret->from->data.distance_to_boundary)
            if ret.as_ref().next.is_none()
                && ret.as_ref().to.unwrap().as_ref().data.distance_to_boundary - scaled(0.005)
                    < ret.as_ref().from.unwrap().as_ref().data.distance_to_boundary
            {
                // SkeletalTrapezoidation.cpp:1601 ret = ret->prev;
                ret = ret.as_ref().prev.unwrap();
            }
            // SkeletalTrapezoidation.cpp:1603 assert(ret);
            // SkeletalTrapezoidation.cpp:1604 assert(ret->next);
            debug_assert!(ret.as_ref().next.is_some());
            // SkeletalTrapezoidation.cpp:1605 return ret;
            ret
        }
    }

    // SkeletalTrapezoidation.cpp:1608-1635
    // void SkeletalTrapezoidation::propagateBeadingsUpward(std::vector<edge_t*>& upward_quad_mids, ptr_vector_t<BeadingPropagation>& node_beadings)
    pub fn propagate_beadings_upward(
        &mut self,
        upward_quad_mids: &mut [EdgePtr],
        node_beadings: &mut Vec<Arc<RwLock<BeadingPropagation>>>,
    ) {
        // CENSUS (R588): the per-NODE `bead_count >= 0` share is what the upward
        // guard actually tests, and the per-EDGE central share is R587's handoff.
        // Measured directly on the graph here, once per generate() call, so both are
        // order-independent rates over the whole population.
        if crate::probe_enabled("CENSUS") {
            let mut cs_nodes = 0usize;
            let mut cs_bc = 0usize;
            let mut cs_hasb = 0usize;
            for nd in self.graph.nodes.iter() {
                cs_nodes += 1;
                if nd.base.data.bead_count >= 0 {
                    cs_bc += 1;
                }
                if nd.base.data.has_beading() {
                    cs_hasb += 1;
                }
            }
            let mut cs_edges = 0usize;
            let mut cs_central = 0usize;
            for eg in self.graph.edges.iter() {
                cs_edges += 1;
                if eg.base.data.is_central() {
                    cs_central += 1;
                }
            }
            census_tick(cs_nodes, cs_bc, cs_hasb, cs_edges, cs_central);
        }

        unsafe {
            // SkeletalTrapezoidation.cpp:1610 for (auto upward_quad_mids_it = upward_quad_mids.rbegin(); ...; ++it)
            for &upward_edge in upward_quad_mids.iter().rev() {
                let to = upward_edge.as_ref().to.unwrap();
                let from = upward_edge.as_ref().from.unwrap();
                if crate::probe_enabled("UPPROBE") {
                    upprobe_tick(
                        to.as_ref().data.bead_count >= 0,
                        !from.as_ref().data.has_beading(),
                        to.as_ref().data.has_beading(),
                    );
                }
                // SkeletalTrapezoidation.cpp:1613 if (upward_edge->to->data.bead_count >= 0)
                if to.as_ref().data.bead_count >= 0 {
                    // SkeletalTrapezoidation.cpp:1615 continue;
                    continue;
                }
                // SkeletalTrapezoidation.cpp:1617 if (! upward_edge->from->data.hasBeading())
                if !from.as_ref().data.has_beading() {
                    // SkeletalTrapezoidation.cpp:1619 continue;
                    continue;
                }
                // SkeletalTrapezoidation.cpp:1621 BeadingPropagation& lower_beading = *upward_edge->from->data.getBeading();
                let lower_beading_arc = from.as_ref().data.get_beading().unwrap();
                // SkeletalTrapezoidation.cpp:1622 if (upward_edge->to->data.hasBeading())
                if to.as_ref().data.has_beading() {
                    // SkeletalTrapezoidation.cpp:1624 continue;
                    continue;
                }
                // SkeletalTrapezoidation.cpp:1627 coord_t length = (upward_edge->to->p - upward_edge->from->p).cast<int64_t>().norm();
                let length = (to.as_ref().p - from.as_ref().p).length() as Coord;
                // SkeletalTrapezoidation.cpp:1628 BeadingPropagation upper_beading = lower_beading;
                let mut upper_beading = lower_beading_arc.read().clone();
                // SkeletalTrapezoidation.cpp:1629 upper_beading.dist_to_bottom_source += length;
                upper_beading.dist_to_bottom_source += length;
                // SkeletalTrapezoidation.cpp:1630 upper_beading.is_upward_propagated_only = true;
                upper_beading.is_upward_propagated_only = true;
                // SkeletalTrapezoidation.cpp:1631 node_beadings.emplace_back(new BeadingPropagation(upper_beading));
                let total_thickness = upper_beading.beading.total_thickness;
                let bp = Arc::new(RwLock::new(upper_beading));
                node_beadings.push(bp.clone());
                // SkeletalTrapezoidation.cpp:1632 upward_edge->to->data.setBeading(node_beadings.back());
                to.as_ptr().as_mut().unwrap().data.set_beading(bp);
                // SkeletalTrapezoidation.cpp:1633 assert(upper_beading.beading.total_thickness <= upward_edge->to->data.distance_to_boundary * 2);
                debug_assert!(total_thickness <= to.as_ref().data.distance_to_boundary * 2);
            }
        }
    }

    // SkeletalTrapezoidation.cpp:1637-1658
    // void SkeletalTrapezoidation::propagateBeadingsDownward(std::vector<edge_t*>& upward_quad_mids, ptr_vector_t<BeadingPropagation>& node_beadings)
    pub fn propagate_beadings_downward(
        &mut self,
        upward_quad_mids: &mut [EdgePtr],
        node_beadings: &mut Vec<Arc<RwLock<BeadingPropagation>>>,
    ) {
        unsafe {
            // SkeletalTrapezoidation.cpp:1639 for (edge_t* upward_quad_mid : upward_quad_mids)
            for &upward_quad_mid in upward_quad_mids.iter() {
                if crate::probe_enabled("UPPROBE") {
                    let dn_central = upward_quad_mid.as_ref().data.is_central();
                    let f = upward_quad_mid.as_ref().from.unwrap();
                    let t = upward_quad_mid.as_ref().to.unwrap();
                    let dn_equi = !dn_central
                        && f.as_ref().data.distance_to_boundary
                            == t.as_ref().data.distance_to_boundary
                        && f.as_ref().data.has_beading()
                        && !t.as_ref().data.has_beading();
                    dnprobe_tick(dn_central, dn_equi);
                }
                // SkeletalTrapezoidation.cpp:1642 if (!upward_quad_mid->data.isCentral())
                if !upward_quad_mid.as_ref().data.is_central() {
                    let from = upward_quad_mid.as_ref().from.unwrap();
                    let to = upward_quad_mid.as_ref().to.unwrap();
                    // SkeletalTrapezoidation.cpp:1645-1647 if (from->...dtb == to->...dtb && from->hasBeading() && !to->hasBeading())
                    if from.as_ref().data.distance_to_boundary == to.as_ref().data.distance_to_boundary
                        && from.as_ref().data.has_beading()
                        && !to.as_ref().data.has_beading()
                    {
                        // SkeletalTrapezoidation.cpp:1650 propagateBeadingsDownward(upward_quad_mid->twin, node_beadings);
                        self.propagate_beadings_downward_edge(
                            upward_quad_mid.as_ref().twin.unwrap(),
                            node_beadings,
                        );
                    } else {
                        // SkeletalTrapezoidation.cpp:1654 propagateBeadingsDownward(upward_quad_mid, node_beadings);
                        self.propagate_beadings_downward_edge(upward_quad_mid, node_beadings);
                    }
                }
            }
        }
    }

    // SkeletalTrapezoidation.cpp:1660-1706
    // void SkeletalTrapezoidation::propagateBeadingsDownward(edge_t* edge_to_peak, ptr_vector_t<BeadingPropagation>& node_beadings)
    pub fn propagate_beadings_downward_edge(
        &mut self,
        edge_to_peak: EdgePtr,
        node_beadings: &mut Vec<Arc<RwLock<BeadingPropagation>>>,
    ) {
        unsafe {
            let to = edge_to_peak.as_ref().to.unwrap();
            let from = edge_to_peak.as_ref().from.unwrap();
            // SkeletalTrapezoidation.cpp:1662 coord_t length = (edge_to_peak->to->p - edge_to_peak->from->p).cast<int64_t>().norm();
            let length = (to.as_ref().p - from.as_ref().p).length() as Coord;
            // SkeletalTrapezoidation.cpp:1663 BeadingPropagation& top_beading = *getOrCreateBeading(edge_to_peak->to, node_beadings);
            let top_beading_arc = self.get_or_create_beading(to, node_beadings);
            // SkeletalTrapezoidation.cpp:1664 assert(top_beading.beading.total_thickness >= edge_to_peak->to->data.distance_to_boundary * 2);
            debug_assert!(
                top_beading_arc.read().beading.total_thickness >= to.as_ref().data.distance_to_boundary * 2
            );
            // SkeletalTrapezoidation.cpp:1665 if(top_beading.beading.total_thickness < edge_to_peak->to->data.distance_to_boundary * 2)
            if top_beading_arc.read().beading.total_thickness < to.as_ref().data.distance_to_boundary * 2 {
                log::warn!("Top bead is beyond the center of the total width.");
            }
            // SkeletalTrapezoidation.cpp:1669 assert(!top_beading.is_upward_propagated_only);
            debug_assert!(!top_beading_arc.read().is_upward_propagated_only);

            // SkeletalTrapezoidation.cpp:1671 if(!edge_to_peak->from->data.hasBeading())
            if !from.as_ref().data.has_beading() {
                // SkeletalTrapezoidation.cpp:1673 BeadingPropagation propagated_beading = top_beading;
                let mut propagated_beading = top_beading_arc.read().clone();
                // SkeletalTrapezoidation.cpp:1674 propagated_beading.dist_from_top_source += length;
                propagated_beading.dist_from_top_source += length;
                propclass_tick(1);
                // SkeletalTrapezoidation.cpp:1675 node_beadings.emplace_back(new BeadingPropagation(propagated_beading));
                let total_thickness = propagated_beading.beading.total_thickness;
                let bp = Arc::new(RwLock::new(propagated_beading));
                node_beadings.push(bp.clone());
                // SkeletalTrapezoidation.cpp:1676 edge_to_peak->from->data.setBeading(node_beadings.back());
                from.as_ptr().as_mut().unwrap().data.set_beading(bp);
                // SkeletalTrapezoidation.cpp:1677 assert(propagated_beading.beading.total_thickness >= edge_to_peak->from->data.distance_to_boundary * 2);
                debug_assert!(total_thickness >= from.as_ref().data.distance_to_boundary * 2);
                // SkeletalTrapezoidation.cpp:1678 if(propagated_beading.beading.total_thickness < edge_to_peak->from->data.distance_to_boundary * 2)
                if total_thickness < from.as_ref().data.distance_to_boundary * 2 {
                    log::warn!("Propagated bead is beyond the center of the total width.");
                }
            } else {
                // SkeletalTrapezoidation.cpp:1685 BeadingPropagation& bottom_beading = *edge_to_peak->from->data.getBeading();
                let bottom_beading_arc = from.as_ref().data.get_beading().unwrap();
                // SkeletalTrapezoidation.cpp:1686 coord_t total_dist = top_beading.dist_from_top_source + length + bottom_beading.dist_to_bottom_source;
                let total_dist = top_beading_arc.read().dist_from_top_source
                    + length
                    + bottom_beading_arc.read().dist_to_bottom_source;
                // SkeletalTrapezoidation.cpp:1687 double ratio_of_top = static_cast<float>(bottom_beading.dist_to_bottom_source) / std::min(total_dist, beading_propagation_transition_dist);
                // C++ evaluates `float / int` in f32 (the int operand is promoted to float),
                // then widens the f32 result to the `double` target.
                let mut ratio_of_top = ((bottom_beading_arc.read().dist_to_bottom_source as f32)
                    / std::cmp::min(total_dist, self.beading_propagation_transition_dist) as f32)
                    as f64;
                // SkeletalTrapezoidation.cpp:1688 ratio_of_top = std::max(0.0, ratio_of_top);
                ratio_of_top = ratio_of_top.max(0.0);
                // R546 probe (PROPPROBE=1): which branch runs? `ratio_of_top >= 1.0`
                // is a PURE COPY of the top beading onto the bottom node (:1691),
                // which is exactly the sharing R545 identified by exact equality.
                // The `else` interpolates. Measure the split and the ratio, plus the
                // runtime value of `beading_propagation_transition_dist` (R490/R525:
                // read the constant, do not assume it).
                if crate::probe_enabled("PROPPROBE") {
                    propprobe(ratio_of_top, self.beading_propagation_transition_dist, total_dist);
                }
                // SkeletalTrapezoidation.cpp:1689 if (ratio_of_top >= 1.0)
                if ratio_of_top >= 1.0 {
                    propclass_tick(2);
                    // SkeletalTrapezoidation.cpp:1691 bottom_beading = top_beading;
                    let mut new_bottom = top_beading_arc.read().clone();
                    // SkeletalTrapezoidation.cpp:1692 bottom_beading.dist_from_top_source += length;
                    new_bottom.dist_from_top_source += length;
                    *bottom_beading_arc.write() = new_bottom;
                } else {
                    // SkeletalTrapezoidation.cpp:1696 Beading merged_beading = interpolate(top_beading.beading, ratio_of_top, bottom_beading.beading, edge_to_peak->from->data.distance_to_boundary);
                    let top_beading_b = top_beading_arc.read().beading.clone();
                    let bottom_beading_b = bottom_beading_arc.read().beading.clone();
                    let pc_w_before = bottom_beading_b.bead_widths.first().copied();
                    let merged_beading = self.interpolate4(
                        &top_beading_b,
                        ratio_of_top,
                        &bottom_beading_b,
                        from.as_ref().data.distance_to_boundary,
                    );
                    if crate::probe_enabled("PROPCLASS") {
                        if let (Some(wb), Some(wa)) =
                            (pc_w_before, merged_beading.bead_widths.first().copied())
                        {
                            propclass_interp_delta((wa - wb).abs());
                        }
                    }
                    propclass_tick(3);
                    // SkeletalTrapezoidation.cpp:1697 bottom_beading = BeadingPropagation(merged_beading);
                    let mut new_bottom = BeadingPropagation::new(merged_beading.clone());
                    // SkeletalTrapezoidation.cpp:1698 bottom_beading.is_upward_propagated_only = false;
                    new_bottom.is_upward_propagated_only = false;
                    *bottom_beading_arc.write() = new_bottom;
                    // SkeletalTrapezoidation.cpp:1699 assert(merged_beading.total_thickness >= edge_to_peak->from->data.distance_to_boundary * 2);
                    debug_assert!(
                        merged_beading.total_thickness >= from.as_ref().data.distance_to_boundary * 2
                    );
                    // SkeletalTrapezoidation.cpp:1700 if(merged_beading.total_thickness < edge_to_peak->from->data.distance_to_boundary * 2)
                    if merged_beading.total_thickness < from.as_ref().data.distance_to_boundary * 2 {
                        log::warn!("Merged bead is beyond the center of the total width.");
                    }
                }
            }
        }
    }

    // SkeletalTrapezoidation.cpp:1709-1749
    // SkeletalTrapezoidation::Beading SkeletalTrapezoidation::interpolate(const Beading& left, double ratio_left_to_whole, const Beading& right, coord_t switching_radius) const
    pub fn interpolate4(
        &self,
        left: &Beading,
        ratio_left_to_whole: f64,
        right: &Beading,
        switching_radius: Coord,
    ) -> Beading {
        // SkeletalTrapezoidation.cpp:1711 assert(ratio_left_to_whole >= 0.0 && ratio_left_to_whole <= 1.0);
        debug_assert!((0.0..=1.0).contains(&ratio_left_to_whole));
        // SkeletalTrapezoidation.cpp:1712 Beading ret = interpolate(left, ratio_left_to_whole, right);
        let ret = self.interpolate2(left, ratio_left_to_whole, right);

        // SkeletalTrapezoidation.cpp:1716 coord_t next_inset_idx;
        let mut next_inset_idx: i64;
        // SkeletalTrapezoidation.cpp:1717 for (next_inset_idx = left.toolpath_locations.size() - 1; next_inset_idx >= 0; next_inset_idx--)
        next_inset_idx = left.toolpath_locations.len() as i64 - 1;
        while next_inset_idx >= 0 {
            // SkeletalTrapezoidation.cpp:1719 if (switching_radius > left.toolpath_locations[next_inset_idx])
            if switching_radius > left.toolpath_locations[next_inset_idx as usize] {
                // SkeletalTrapezoidation.cpp:1721 break;
                break;
            }
            next_inset_idx -= 1;
        }
        // SkeletalTrapezoidation.cpp:1724 if (next_inset_idx < 0)
        if next_inset_idx < 0 {
            // SkeletalTrapezoidation.cpp:1726 assert(left.toolpath_locations.empty() || left.toolpath_locations.front() >= switching_radius);
            debug_assert!(
                left.toolpath_locations.is_empty()
                    || left.toolpath_locations[0] >= switching_radius
            );
            // SkeletalTrapezoidation.cpp:1727 return ret;
            return ret;
        }
        // SkeletalTrapezoidation.cpp:1729 if (next_inset_idx + 1 == coord_t(left.toolpath_locations.size()))
        if next_inset_idx + 1 == left.toolpath_locations.len() as i64 {
            // SkeletalTrapezoidation.cpp:1731 return ret;
            return ret;
        }
        // SkeletalTrapezoidation.cpp:1733-1735 asserts
        debug_assert!(next_inset_idx < left.toolpath_locations.len() as i64);
        debug_assert!(left.toolpath_locations[next_inset_idx as usize] <= switching_radius);
        debug_assert!(left.toolpath_locations[(next_inset_idx + 1) as usize] >= switching_radius);
        // SkeletalTrapezoidation.cpp:1736 if (ret.toolpath_locations[next_inset_idx] > switching_radius)
        if ret.toolpath_locations[next_inset_idx as usize] > switching_radius {
            // SkeletalTrapezoidation.cpp:1744 float new_ratio = static_cast<float>(switching_radius - right.toolpath_locations[next_inset_idx]) / static_cast<float>(left.toolpath_locations[next_inset_idx] - right.toolpath_locations[next_inset_idx]);
            let new_ratio = (switching_radius - right.toolpath_locations[next_inset_idx as usize])
                as f32
                / (left.toolpath_locations[next_inset_idx as usize]
                    - right.toolpath_locations[next_inset_idx as usize]) as f32;
            // SkeletalTrapezoidation.cpp:1745 new_ratio = std::min(1.0, new_ratio + 0.1);
            // C++ `new_ratio` is `float`: the std::min result (double) is truncated back to f32.
            let new_ratio: f32 = (new_ratio as f64 + 0.1_f64).min(1.0) as f32;
            // SkeletalTrapezoidation.cpp:1746 return interpolate(left, new_ratio, right);
            return self.interpolate2(left, new_ratio as f64, right);
        }
        // SkeletalTrapezoidation.cpp:1748 return ret;
        ret
    }

    // SkeletalTrapezoidation.cpp:1752-1771
    // SkeletalTrapezoidation::Beading SkeletalTrapezoidation::interpolate(const Beading& left, double ratio_left_to_whole, const Beading& right) const
    pub fn interpolate2(&self, left: &Beading, ratio_left_to_whole: f64, right: &Beading) -> Beading {
        // SkeletalTrapezoidation.cpp:1754 assert(ratio_left_to_whole >= 0.0 && ratio_left_to_whole <= 1.0);
        debug_assert!((0.0..=1.0).contains(&ratio_left_to_whole));
        // SkeletalTrapezoidation.cpp:1755 float ratio_right_to_whole = 1.0 - ratio_left_to_whole;
        // C++ `ratio_right_to_whole` is `float`: the double subtraction is truncated to f32.
        let ratio_right_to_whole: f32 = (1.0 - ratio_left_to_whole) as f32;

        // SkeletalTrapezoidation.cpp:1757 Beading ret = (left.total_thickness > right.total_thickness)? left : right;
        let mut ret = if left.total_thickness > right.total_thickness {
            left.clone()
        } else {
            right.clone()
        };
        // SkeletalTrapezoidation.cpp:1758 for (size_t inset_idx = 0; inset_idx < std::min(left.bead_widths.size(), right.bead_widths.size()); inset_idx++)
        let n = std::cmp::min(left.bead_widths.len(), right.bead_widths.len());
        for inset_idx in 0..n {
            // SkeletalTrapezoidation.cpp:1760 if(left.bead_widths[inset_idx] == 0 || right.bead_widths[inset_idx] == 0)
            if left.bead_widths[inset_idx] == 0 || right.bead_widths[inset_idx] == 0 {
                // SkeletalTrapezoidation.cpp:1762 ret.bead_widths[inset_idx] = 0;
                ret.bead_widths[inset_idx] = 0;
            } else {
                // SkeletalTrapezoidation.cpp:1766 ret.bead_widths[inset_idx] = ratio_left_to_whole * left.bead_widths[inset_idx] + ratio_right_to_whole * right.bead_widths[inset_idx];
                // Left term is `double * int -> double`; right term is `float * int -> float`,
                // promoted to double for the sum, then truncated to coord_t.
                ret.bead_widths[inset_idx] = (ratio_left_to_whole * left.bead_widths[inset_idx] as f64
                    + (ratio_right_to_whole * right.bead_widths[inset_idx] as f32) as f64)
                    as Coord;
            }
            // SkeletalTrapezoidation.cpp:1768 ret.toolpath_locations[inset_idx] = ratio_left_to_whole * left.toolpath_locations[inset_idx] + ratio_right_to_whole * right.toolpath_locations[inset_idx];
            ret.toolpath_locations[inset_idx] = (ratio_left_to_whole
                * left.toolpath_locations[inset_idx] as f64
                + (ratio_right_to_whole * right.toolpath_locations[inset_idx] as f32) as f64)
                as Coord;
        }
        // SkeletalTrapezoidation.cpp:1770 return ret;
        ret
    }

    // SkeletalTrapezoidation.cpp:1773-1850
    // void SkeletalTrapezoidation::generateJunctions(ptr_vector_t<BeadingPropagation>& node_beadings, ptr_vector_t<LineJunctions>& edge_junctions)
    pub fn generate_junctions(
        &mut self,
        node_beadings: &mut Vec<Arc<RwLock<BeadingPropagation>>>,
        edge_junctions: &mut Vec<Arc<RwLock<LineJunctions>>>,
    ) {
        unsafe {
            // SkeletalTrapezoidation.cpp:1775 for (edge_t& edge_ : graph.edges)
            let edges: Vec<EdgePtr> = self
                .graph
                .edges
                .iter()
                .map(|e| SkeletalTrapezoidationGraph::edge_ptr(e))
                .collect();
            for edge in edges {
                // SkeletalTrapezoidation.cpp:1777 edge_t* edge = &edge_;
                let edge_ref = edge.as_ref();
                let from = edge_ref.from.unwrap();
                let to = edge_ref.to.unwrap();
                // SkeletalTrapezoidation.cpp:1778 if (edge->from->data.distance_to_boundary > edge->to->data.distance_to_boundary)
                if from.as_ref().data.distance_to_boundary > to.as_ref().data.distance_to_boundary {
                    // SkeletalTrapezoidation.cpp:1780 continue;
                    continue;
                }

                // SkeletalTrapezoidation.cpp:1783 coord_t start_R = edge->to->data.distance_to_boundary;
                let start_r = to.as_ref().data.distance_to_boundary;
                // SkeletalTrapezoidation.cpp:1784 coord_t end_R = edge->from->data.distance_to_boundary;
                let end_r = from.as_ref().data.distance_to_boundary;

                // SkeletalTrapezoidation.cpp:1786-1787 if ((edge->from->data.bead_count == edge->to->data.bead_count && edge->from->data.bead_count >= 0) || end_R >= start_R)
                if (from.as_ref().data.bead_count == to.as_ref().data.bead_count
                    && from.as_ref().data.bead_count >= 0)
                    || end_r >= start_r
                {
                    // SkeletalTrapezoidation.cpp:1789 continue;
                    continue;
                }

                // SkeletalTrapezoidation.cpp:1792 bool apply_hole_compensation = edge->data.getHoleCompensationFlag();
                let apply_hole_compensation = edge_ref.data.get_hole_compensation_flag();

                // SkeletalTrapezoidation.cpp:1794 Beading* beading = &getOrCreateBeading(edge->to, node_beadings)->beading;
                let beading_arc = self.get_or_create_beading(to, node_beadings);
                let beading = beading_arc.read().beading.clone();
                // SkeletalTrapezoidation.cpp:1795 edge_junctions.emplace_back(std::make_shared<LineJunctions>());
                let ret_arc = Arc::new(RwLock::new(LineJunctions::new()));
                edge_junctions.push(ret_arc.clone());
                // SkeletalTrapezoidation.cpp:1796 edge_.data.setExtrusionJunctions(edge_junctions.back());
                edge.as_ptr().as_mut().unwrap().data.set_extrusion_junctions(ret_arc.clone());
                // SkeletalTrapezoidation.cpp:1797 LineJunctions& ret = *edge_junctions.back();
                let mut ret = ret_arc.write();

                // R584: count edges that emit junctions. Every outer-wall width
                // change happens at an edge boundary (LINEPROBE2 MECH: ch_idx=0),
                // so junctions-per-edge is the denominator of the 1.378x gap.
                if crate::probe_enabled("BEADPROBE") {
                    crate::arachne::skeletal_trapezoidation::EP_EDGES
                        .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                }

                // BEADPAIR (R585): P(adjacent beadings differ in bead_widths[0]).
                // R584 showed junctions-per-edge is at parity (1.022x) while the
                // outer-wall change density is 1.378x, and that every change is one
                // bead index resolving to a different beading at an edge boundary --
                // so the whole gap must live in this probability. A per-edge
                // Bernoulli rate is ORDER-INDEPENDENT, unlike the prefix distinct
                // count R584 had to retract. Read-only: `get_beading` never creates,
                // so this cannot perturb the run.
                if crate::probe_enabled("BEADPAIR") {
                    let w_to = beading.bead_widths.first().copied();
                    let w_from = from
                        .as_ref()
                        .data
                        .get_beading()
                        .and_then(|fb| fb.read().beading.bead_widths.first().copied());
                    let t_from = from
                        .as_ref()
                        .data
                        .get_beading()
                        .map(|fb| fb.read().beading.total_thickness);
                    let same_obj = from
                        .as_ref()
                        .data
                        .get_beading()
                        .map(|fb| std::sync::Arc::ptr_eq(&fb, &beading_arc))
                        .unwrap_or(false);
                    beadpair(w_to, w_from, t_from, beading.total_thickness, same_obj);
                }

                // SkeletalTrapezoidation.cpp:1799 assert(beading->total_thickness >= edge->to->data.distance_to_boundary * 2);
                debug_assert!(beading.total_thickness >= to.as_ref().data.distance_to_boundary * 2);
                // SkeletalTrapezoidation.cpp:1800 if(beading->total_thickness < edge->to->data.distance_to_boundary * 2)
                if beading.total_thickness < to.as_ref().data.distance_to_boundary * 2 {
                    log::warn!("Generated junction is beyond the center of total width.");
                }

                // SkeletalTrapezoidation.cpp:1805 Point a = edge->to->p;
                let a = to.as_ref().p;
                // SkeletalTrapezoidation.cpp:1806 Point b = edge->from->p;
                let b = from.as_ref().p;
                // SkeletalTrapezoidation.cpp:1807 Point ab = b - a;
                let ab = b - a;

                // SkeletalTrapezoidation.cpp:1809 const size_t num_junctions = beading->toolpath_locations.size();
                let num_junctions = beading.toolpath_locations.len();
                // SkeletalTrapezoidation.cpp:1810 size_t junction_idx;
                // SkeletalTrapezoidation.cpp:1812 for (junction_idx = (std::max(size_t(1), beading->toolpath_locations.size()) - 1) / 2; junction_idx < num_junctions; junction_idx--)
                //
                // `junction_idx` is `size_t`: decrement underflows to a huge value which is
                // `>= num_junctions`, terminating the loop — exactly as in C++.
                let mut junction_idx: usize =
                    (std::cmp::max(1, beading.toolpath_locations.len()) - 1) / 2;
                while junction_idx < num_junctions {
                    // SkeletalTrapezoidation.cpp:1814 coord_t bead_R = beading->toolpath_locations[junction_idx];
                    let bead_r = beading.toolpath_locations[junction_idx];
                    // SkeletalTrapezoidation.cpp:1818 if (bead_R <= start_R + 1)
                    if bead_r <= start_r + 1 {
                        // SkeletalTrapezoidation.cpp:1820 break;
                        break;
                    }
                    junction_idx = junction_idx.wrapping_sub(1);
                }

                // SkeletalTrapezoidation.cpp:1826-1829 if (junction_idx + 1 < num_junctions && beading->toolpath_locations[junction_idx + 1] <= start_R + scaled<coord_t>(0.005) && beading->total_thickness < start_R + scaled<coord_t>(0.005))
                // C++ `junction_idx` is size_t; the prior loop may leave it at
                // SIZE_MAX (unsigned underflow), in which case `junction_idx + 1`
                // wraps to 0 — matching the index used here. Use wrapping_add for
                // both the bound check and the index to avoid a debug overflow.
                if junction_idx.wrapping_add(1) < num_junctions
                    && beading.toolpath_locations[junction_idx.wrapping_add(1)]
                        <= start_r + scaled(0.005)
                    && beading.total_thickness < start_r + scaled(0.005)
                {
                    // SkeletalTrapezoidation.cpp:1831 junction_idx++;
                    junction_idx += 1;
                }

                // SkeletalTrapezoidation.cpp:1834 for (; junction_idx < num_junctions; junction_idx--)
                while junction_idx < num_junctions {
                    // SkeletalTrapezoidation.cpp:1836 coord_t bead_R = beading->toolpath_locations[junction_idx];
                    let bead_r = beading.toolpath_locations[junction_idx];
                    // SkeletalTrapezoidation.cpp:1837 assert(bead_R >= 0);
                    debug_assert!(bead_r >= 0);
                    // SkeletalTrapezoidation.cpp:1838 if (bead_R < end_R)
                    if bead_r < end_r {
                        // SkeletalTrapezoidation.cpp:1840 break;
                        break;
                    }
                    // SkeletalTrapezoidation.cpp:1842 Point junction(a + (ab.cast<int64_t>() * int64_t(bead_R - start_R) / int64_t(end_R - start_R)).cast<coord_t>());
                    let mut junction = a
                        + Point::new(
                            (ab.x as i64) * ((bead_r - start_r) as i64) / ((end_r - start_r) as i64),
                            (ab.y as i64) * ((bead_r - start_r) as i64) / ((end_r - start_r) as i64),
                        );
                    // SkeletalTrapezoidation.cpp:1843 if (bead_R > start_R - scaled<coord_t>(0.005))
                    if bead_r > start_r - scaled(0.005) {
                        // SkeletalTrapezoidation.cpp:1845 junction = a;
                        junction = a;
                    }
                    // SkeletalTrapezoidation.cpp:1847 ret.emplace_back(ExtrusionJunction(junction, beading->bead_widths[junction_idx], junction_idx, apply_hole_compensation));
                    if crate::probe_enabled("BEADPROBE") {
                        junctionprobe(
                            beading.bead_widths[junction_idx],
                            beading.total_thickness,
                            junction_idx,
                        );
                    }
                    ret.push(ExtrusionJunction::with_hole_compensation(
                        junction,
                        beading.bead_widths[junction_idx],
                        junction_idx,
                        apply_hole_compensation,
                    ));
                    junction_idx = junction_idx.wrapping_sub(1);
                }
            }
        }
    }

    // SkeletalTrapezoidation.cpp:1852-1892
    // std::shared_ptr<SkeletalTrapezoidationJoint::BeadingPropagation> SkeletalTrapezoidation::getOrCreateBeading(node_t* node, ptr_vector_t<BeadingPropagation>& node_beadings)
    pub fn get_or_create_beading(
        &mut self,
        node: NodePtr,
        node_beadings: &mut Vec<Arc<RwLock<BeadingPropagation>>>,
    ) -> Arc<RwLock<BeadingPropagation>> {
        unsafe {
            if node.as_ref().data.has_beading() {
                gnbprobe(true, false, false);
            }
            // SkeletalTrapezoidation.cpp:1854 if (! node->data.hasBeading())
            if !node.as_ref().data.has_beading() {
                // SkeletalTrapezoidation.cpp:1856 if (node->data.bead_count == -1)
                if node.as_ref().data.bead_count == -1 {
                    // SkeletalTrapezoidation.cpp:1858 constexpr coord_t nearby_dist = scaled<coord_t>(0.1);
                    let nearby_dist: Coord = scaled(0.1);
                    // SkeletalTrapezoidation.cpp:1859 auto nearest_beading = getNearestBeading(node, nearby_dist);
                    let nearest_beading = self.get_nearest_beading(node, nearby_dist);
                    gnbprobe(false, true, nearest_beading.is_some());
                    // SkeletalTrapezoidation.cpp:1860 if (nearest_beading)
                    if let Some(nearest_beading) = nearest_beading {
                        // SkeletalTrapezoidation.cpp:1862 return nearest_beading;
                        return nearest_beading;
                    }

                    // SkeletalTrapezoidation.cpp:1866 bool has_central_edge = false;
                    let mut has_central_edge = false;
                    // SkeletalTrapezoidation.cpp:1867 bool first = true;
                    let mut first = true;
                    // SkeletalTrapezoidation.cpp:1868 coord_t dist = std::numeric_limits<coord_t>::max();
                    let mut dist = Coord::MAX;
                    // SkeletalTrapezoidation.cpp:1869 for (edge_t* edge = node->incident_edge; edge && (first || edge != node->incident_edge); edge = edge->twin->next)
                    let incident_edge = node.as_ref().incident_edge;
                    let mut edge_opt = incident_edge;
                    while edge_opt.is_some() && (first || edge_opt != incident_edge) {
                        let edge = edge_opt.unwrap();
                        // SkeletalTrapezoidation.cpp:1871 if (edge->data.isCentral())
                        if edge.as_ref().data.is_central() {
                            // SkeletalTrapezoidation.cpp:1873 has_central_edge = true;
                            has_central_edge = true;
                        }
                        // SkeletalTrapezoidation.cpp:1875 assert(edge->to->data.distance_to_boundary >= 0);
                        debug_assert!(edge.as_ref().to.unwrap().as_ref().data.distance_to_boundary >= 0);
                        // SkeletalTrapezoidation.cpp:1876 dist = std::min(dist, edge->to->data.distance_to_boundary + coord_t((edge->to->p - edge->from->p).cast<int64_t>().norm()));
                        let seg = (edge.as_ref().to.unwrap().as_ref().p
                            - edge.as_ref().from.unwrap().as_ref().p)
                            .length() as Coord;
                        dist = std::cmp::min(
                            dist,
                            edge.as_ref().to.unwrap().as_ref().data.distance_to_boundary + seg,
                        );
                        // SkeletalTrapezoidation.cpp:1877 first = false;
                        first = false;
                        edge_opt = edge.as_ref().twin.unwrap().as_ref().next;
                    }
                    // SkeletalTrapezoidation.cpp:1879 if (!has_central_edge)
                    if !has_central_edge {
                        log::error!("Unknown beading for non-central node!");
                    }
                    // SkeletalTrapezoidation.cpp:1883 assert(dist != std::numeric_limits<coord_t>::max());
                    debug_assert!(dist != Coord::MAX);
                    // SkeletalTrapezoidation.cpp:1884 node->data.bead_count = beading_strategy.getOptimalBeadCount(dist * 2);
                    node.as_ptr().as_mut().unwrap().data.bead_count =
                        self.beading_strategy.get_optimal_bead_count(dist * 2);
                }
                // SkeletalTrapezoidation.cpp:1886 assert(node->data.bead_count != -1);
                debug_assert!(node.as_ref().data.bead_count != -1);
                // SkeletalTrapezoidation.cpp:1887 node_beadings.emplace_back(new BeadingPropagation(beading_strategy.compute(node->data.distance_to_boundary * 2, node->data.bead_count)));
                propclass_tick(0);
                let _computed = self.beading_strategy.compute(
                    node.as_ref().data.distance_to_boundary * 2,
                    node.as_ref().data.bead_count,
                );
                if crate::probe_enabled("BEADPROBE") {
                    beadprobe(
                        node.as_ref().data.distance_to_boundary * 2,
                        node.as_ref().data.bead_count,
                        &_computed.bead_widths,
                    );
                }
                let bp = Arc::new(RwLock::new(BeadingPropagation::new(_computed)));
                node_beadings.push(bp.clone());
                // SkeletalTrapezoidation.cpp:1888 node->data.setBeading(node_beadings.back());
                node.as_ptr().as_mut().unwrap().data.set_beading(bp);
            }
            // SkeletalTrapezoidation.cpp:1890 assert(node->data.hasBeading());
            debug_assert!(node.as_ref().data.has_beading());
            // SkeletalTrapezoidation.cpp:1891 return node->data.getBeading();
            node.as_ref().data.get_beading().unwrap()
        }
    }

    // SkeletalTrapezoidation.cpp:1894-1933
    // std::shared_ptr<SkeletalTrapezoidationJoint::BeadingPropagation> SkeletalTrapezoidation::getNearestBeading(node_t* node, coord_t max_dist)
    pub fn get_nearest_beading(
        &mut self,
        node: NodePtr,
        max_dist: Coord,
    ) -> Option<Arc<RwLock<BeadingPropagation>>> {
        unsafe {
            // SkeletalTrapezoidation.cpp:1896-1903 struct DistEdge { edge_t* edge_to; coord_t dist; };
            // SkeletalTrapezoidation.cpp:1905 auto compare = [](const DistEdge& l, const DistEdge& r) -> bool { return l.dist > r.dist; };
            // SkeletalTrapezoidation.cpp:1906 std::priority_queue<DistEdge, ..., decltype(compare)> further_edges(compare);
            //
            // C++'s priority_queue with `l.dist > r.dist` pops the *smallest* dist first.
            // We use a BinaryHeap on `Reverse(dist)` (min-heap) keyed by dist; ties pop in
            // an unspecified order in both, matching the C++ behaviour.
            #[derive(Eq, PartialEq)]
            struct DistEdge {
                dist: Coord,
                edge_to: EdgePtr,
            }
            impl Ord for DistEdge {
                fn cmp(&self, other: &Self) -> std::cmp::Ordering {
                    // min-heap on dist: reverse the natural ordering
                    other.dist.cmp(&self.dist)
                }
            }
            impl PartialOrd for DistEdge {
                fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
                    Some(self.cmp(other))
                }
            }
            let mut further_edges: BinaryHeap<DistEdge> = BinaryHeap::new();
            // SkeletalTrapezoidation.cpp:1907 bool first = true;
            let mut first = true;
            // SkeletalTrapezoidation.cpp:1908 for (edge_t* outgoing = node->incident_edge; outgoing && (first || outgoing != node->incident_edge); outgoing = outgoing->twin->next)
            let incident_edge = node.as_ref().incident_edge;
            let mut outgoing_opt = incident_edge;
            while outgoing_opt.is_some() && (first || outgoing_opt != incident_edge) {
                let outgoing = outgoing_opt.unwrap();
                // SkeletalTrapezoidation.cpp:1910 further_edges.emplace(outgoing, (outgoing->to->p - outgoing->from->p).cast<int64_t>().norm());
                let d = (outgoing.as_ref().to.unwrap().as_ref().p
                    - outgoing.as_ref().from.unwrap().as_ref().p)
                    .length() as Coord;
                further_edges.push(DistEdge { dist: d, edge_to: outgoing });
                // SkeletalTrapezoidation.cpp:1911 first = false;
                first = false;
                outgoing_opt = outgoing.as_ref().twin.unwrap().as_ref().next;
            }

            // SkeletalTrapezoidation.cpp:1914 for (coord_t counter = 0; counter < SKELETAL_TRAPEZOIDATION_BEAD_SEARCH_MAX; counter++)
            let mut counter: Coord = 0;
            while counter < SKELETAL_TRAPEZOIDATION_BEAD_SEARCH_MAX {
                // SkeletalTrapezoidation.cpp:1916 if (further_edges.empty()) return nullptr;
                if further_edges.is_empty() {
                    return None;
                }
                // SkeletalTrapezoidation.cpp:1917 DistEdge here = further_edges.top();
                // SkeletalTrapezoidation.cpp:1918 further_edges.pop();
                let here = further_edges.pop().unwrap();
                // SkeletalTrapezoidation.cpp:1919 if (here.dist > max_dist) return nullptr;
                if here.dist > max_dist {
                    return None;
                }
                // SkeletalTrapezoidation.cpp:1920 if (here.edge_to->to->data.hasBeading())
                if here.edge_to.as_ref().to.unwrap().as_ref().data.has_beading() {
                    // SkeletalTrapezoidation.cpp:1922 return here.edge_to->to->data.getBeading();
                    return here.edge_to.as_ref().to.unwrap().as_ref().data.get_beading();
                } else {
                    // SkeletalTrapezoidation.cpp:1926 for (edge_t* further_edge = here.edge_to->next; further_edge && further_edge != here.edge_to->twin; further_edge = further_edge->twin->next)
                    let twin = here.edge_to.as_ref().twin;
                    let mut further_edge_opt = here.edge_to.as_ref().next;
                    while further_edge_opt.is_some() && further_edge_opt != twin {
                        let further_edge = further_edge_opt.unwrap();
                        // SkeletalTrapezoidation.cpp:1928 further_edges.emplace(further_edge, here.dist + (further_edge->to->p - further_edge->from->p).cast<int64_t>().norm());
                        let seg = (further_edge.as_ref().to.unwrap().as_ref().p
                            - further_edge.as_ref().from.unwrap().as_ref().p)
                            .length() as Coord;
                        further_edges.push(DistEdge {
                            dist: here.dist + seg,
                            edge_to: further_edge,
                        });
                        further_edge_opt = further_edge.as_ref().twin.unwrap().as_ref().next;
                    }
                }
                counter += 1;
            }
            // SkeletalTrapezoidation.cpp:1932 return nullptr;
            None
        }
    }

    // SkeletalTrapezoidation.cpp:1935-1980
    // void SkeletalTrapezoidation::addToolpathSegment(const ExtrusionJunction& from, const ExtrusionJunction& to, bool is_odd, bool force_new_path, bool from_is_3way, bool to_is_3way)
    #[allow(clippy::too_many_arguments)]
    pub fn add_toolpath_segment(
        &mut self,
        from: &ExtrusionJunction,
        to: &ExtrusionJunction,
        is_odd: bool,
        mut force_new_path: bool,
        from_is_3way: bool,
        to_is_3way: bool,
    ) {
        unsafe {
            // SkeletalTrapezoidation.cpp:1937 if (from == to) return;
            if from == to {
                return;
            }

            // SkeletalTrapezoidation.cpp:1939 std::vector<VariableWidthLines>& generated_toolpaths = *p_generated_toolpaths;
            let generated_toolpaths = &mut *self.p_generated_toolpaths;

            // SkeletalTrapezoidation.cpp:1941 size_t inset_idx = from.perimeter_index;
            let inset_idx = from.perimeter_index;
            // SkeletalTrapezoidation.cpp:1942 if (inset_idx >= generated_toolpaths.size())
            if inset_idx >= generated_toolpaths.len() {
                // SkeletalTrapezoidation.cpp:1944 generated_toolpaths.resize(inset_idx + 1);
                generated_toolpaths.resize(inset_idx + 1, VariableWidthLines::new());
            }
            // SkeletalTrapezoidation.cpp:1946 assert((generated_toolpaths[inset_idx].empty() || !generated_toolpaths[inset_idx].back().junctions.empty()) && ...);
            debug_assert!(
                generated_toolpaths[inset_idx].is_empty()
                    || !generated_toolpaths[inset_idx].last().unwrap().junctions.is_empty()
            );
            // NEWLINEPROBE (R576) — which condition starts each new ExtrusionLine?
            // C++ assembles 1.96x more outer-wall lines than we do (R574); this
            // attributes every new-line decision to its cause. Outer wall only.
            let nlp = crate::probe_enabled("NEWLINEPROBE") && inset_idx == 0;
            let nlp_caller = force_new_path;
            let nlp_empty = generated_toolpaths[inset_idx].is_empty();
            let nlp_odd = !nlp_empty
                && generated_toolpaths[inset_idx].last().unwrap().is_odd != is_odd;
            let nlp_perim = !nlp_empty
                && !nlp_odd
                && generated_toolpaths[inset_idx]
                    .last()
                    .unwrap()
                    .junctions
                    .last()
                    .unwrap()
                    .perimeter_index
                    != inset_idx;

            // ODDPROBE (R577) — the `odd` new-line cause is 3.21x (R576, 41.5% of
            // factor 1). This counts EVERY segment reaching this function at inset
            // 0, split by `is_odd`, plus alternations in the call sequence. That
            // distinguishes "C++ generates more odd walls" (share differs) from
            // "C++ interleaves them differently" (share matches, alternations differ).
            if crate::probe_enabled("ODDPROBE") && inset_idx == 0 {
                use std::sync::atomic::{AtomicU64, Ordering::Relaxed};
                static CALLS: AtomicU64 = AtomicU64::new(0);
                static ODD: AtomicU64 = AtomicU64::new(0);
                static ALT: AtomicU64 = AtomicU64::new(0);
                static PREV: AtomicU64 = AtomicU64::new(2); // 2 = unset
                let cur = u64::from(is_odd);
                if PREV.swap(cur, Relaxed) != cur {
                    ALT.fetch_add(1, Relaxed);
                }
                if is_odd {
                    ODD.fetch_add(1, Relaxed);
                }
                let n = CALLS.fetch_add(1, Relaxed) + 1;
                if n % 5_000 == 0 {
                    eprintln!(
                        "[ODDPROBE] segments={} odd={} even={} alternations={}",
                        n,
                        ODD.load(Relaxed),
                        n - ODD.load(Relaxed),
                        ALT.load(Relaxed),
                    );
                }
            }

            // SkeletalTrapezoidation.cpp:1947-1950 if (empty || back().is_odd != is_odd || back().junctions.back().perimeter_index != inset_idx)
            if generated_toolpaths[inset_idx].is_empty()
                || generated_toolpaths[inset_idx].last().unwrap().is_odd != is_odd
                || generated_toolpaths[inset_idx]
                    .last()
                    .unwrap()
                    .junctions
                    .last()
                    .unwrap()
                    .perimeter_index
                    != inset_idx
            {
                // SkeletalTrapezoidation.cpp:1952 force_new_path = true;
                force_new_path = true;
            }
            // SkeletalTrapezoidation.cpp:1954-1958 if (!force_new_path && shorter_then(back().junctions.back().p - from.p, scaled<coord_t>(0.010)) && std::abs(back().junctions.back().w - from.w) < scaled<coord_t>(0.010) && ! from_is_3way)
            if !force_new_path
                && shorter_then(
                    &(generated_toolpaths[inset_idx].last().unwrap().junctions.last().unwrap().p - from.p),
                    scaled(0.010),
                )
                && (generated_toolpaths[inset_idx].last().unwrap().junctions.last().unwrap().w - from.w)
                    .abs()
                    < scaled(0.010)
                && !from_is_3way
            {
                // SkeletalTrapezoidation.cpp:1960 generated_toolpaths[inset_idx].back().junctions.push_back(to);
                generated_toolpaths[inset_idx].last_mut().unwrap().junctions.push(to.clone());
            }
            // SkeletalTrapezoidation.cpp:1962-1966 else if (!force_new_path && shorter_then(back().junctions.back().p - to.p, ...) && std::abs(...w - to.w) < ... && ! to_is_3way)
            else if !force_new_path
                && shorter_then(
                    &(generated_toolpaths[inset_idx].last().unwrap().junctions.last().unwrap().p - to.p),
                    scaled(0.010),
                )
                && (generated_toolpaths[inset_idx].last().unwrap().junctions.last().unwrap().w - to.w)
                    .abs()
                    < scaled(0.010)
                && !to_is_3way
            {
                // SkeletalTrapezoidation.cpp:1968 if ( ! is_odd)
                if !is_odd {
                    log::error!("Reversing even wall line causes it to be printed CCW instead of CW!");
                }
                // SkeletalTrapezoidation.cpp:1972 generated_toolpaths[inset_idx].back().junctions.push_back(from);
                generated_toolpaths[inset_idx].last_mut().unwrap().junctions.push(from.clone());
            } else {
                // NEWLINEPROBE (R576) — classify the cause of this new line.
                if nlp {
                    use std::sync::atomic::{AtomicU64, Ordering::Relaxed};
                    static N: AtomicU64 = AtomicU64::new(0);
                    static EMPTY: AtomicU64 = AtomicU64::new(0);
                    static ODD: AtomicU64 = AtomicU64::new(0);
                    static PERIM: AtomicU64 = AtomicU64::new(0);
                    static CALLER: AtomicU64 = AtomicU64::new(0);
                    static GAP: AtomicU64 = AtomicU64::new(0);
                    static WIDTH: AtomicU64 = AtomicU64::new(0);
                    static THREEWAY: AtomicU64 = AtomicU64::new(0);
                    if nlp_empty {
                        EMPTY.fetch_add(1, Relaxed);
                    } else if nlp_odd {
                        ODD.fetch_add(1, Relaxed);
                    } else if nlp_perim {
                        PERIM.fetch_add(1, Relaxed);
                    } else if nlp_caller {
                        CALLER.fetch_add(1, Relaxed);
                    } else {
                        // Neither continuation test passed. Attribute to the first
                        // failing sub-condition of the `from` test (cpp:1954-1958).
                        let last = generated_toolpaths[inset_idx]
                            .last()
                            .unwrap()
                            .junctions
                            .last()
                            .unwrap()
                            .clone();
                        let gap_ok = shorter_then(&(last.p - from.p), scaled(0.010));
                        let w_ok = (last.w - from.w).abs() < scaled(0.010);
                        if !gap_ok {
                            GAP.fetch_add(1, Relaxed);
                        } else if !w_ok {
                            WIDTH.fetch_add(1, Relaxed);
                        } else {
                            THREEWAY.fetch_add(1, Relaxed);
                        }
                    }
                    let n = N.fetch_add(1, Relaxed) + 1;
                    if n % 2_000 == 0 {
                        eprintln!(
                            "[NEWLINEPROBE] newlines={} empty={} odd={} perim={} caller={} gap={} width={} threeway={}",
                            n,
                            EMPTY.load(Relaxed),
                            ODD.load(Relaxed),
                            PERIM.load(Relaxed),
                            CALLER.load(Relaxed),
                            GAP.load(Relaxed),
                            WIDTH.load(Relaxed),
                            THREEWAY.load(Relaxed),
                        );
                    }
                }
                // SkeletalTrapezoidation.cpp:1976 generated_toolpaths[inset_idx].emplace_back(inset_idx, is_odd);
                generated_toolpaths[inset_idx].push(ExtrusionLine::new(inset_idx, is_odd));
                // SkeletalTrapezoidation.cpp:1977 generated_toolpaths[inset_idx].back().junctions.push_back(from);
                generated_toolpaths[inset_idx].last_mut().unwrap().junctions.push(from.clone());
                // SkeletalTrapezoidation.cpp:1978 generated_toolpaths[inset_idx].back().junctions.push_back(to);
                generated_toolpaths[inset_idx].last_mut().unwrap().junctions.push(to.clone());
            }
        }
    }

    // SkeletalTrapezoidation.cpp:1982-2100
    // void SkeletalTrapezoidation::connectJunctions(ptr_vector_t<LineJunctions>& edge_junctions)
    pub fn connect_junctions(&mut self, edge_junctions: &mut Vec<Arc<RwLock<LineJunctions>>>) {
        unsafe {
            // SkeletalTrapezoidation.cpp:1984 EdgeSet unprocessed_quad_starts(graph.edges.size() * 5 / 2);
            let mut unprocessed_quad_starts: std::collections::HashSet<*const HalfEdge<EdgeData, NodeData>> =
                std::collections::HashSet::new();
            // SkeletalTrapezoidation.cpp:1985 for (edge_t& edge : graph.edges)
            for edge in self.graph.edges.iter() {
                // SkeletalTrapezoidation.cpp:1987 if (!edge.prev)
                if edge.base.prev.is_none() {
                    // SkeletalTrapezoidation.cpp:1989 unprocessed_quad_starts.emplace(&edge);
                    unprocessed_quad_starts
                        .insert(SkeletalTrapezoidationGraph::edge_ptr(edge).as_ptr() as *const _);
                }
            }

            // SkeletalTrapezoidation.cpp:1993 EdgeSet passed_odd_edges;
            let mut passed_odd_edges: std::collections::HashSet<*const HalfEdge<EdgeData, NodeData>> =
                std::collections::HashSet::new();

            // SkeletalTrapezoidation.cpp:1995 while (!unprocessed_quad_starts.empty())
            while !unprocessed_quad_starts.is_empty() {
                // SkeletalTrapezoidation.cpp:1997 edge_t* poly_domain_start = *unprocessed_quad_starts.begin();
                let poly_domain_start: EdgePtr = {
                    // R99 determinism: the set is POINTER-keyed → its iteration order
                    // leaks ASLR + RandomState per run (C++ `*unordered_set.begin()`,
                    // SkeletalTrapezoidation.cpp:1997, is stable-per-run). Pick the
                    // unprocessed start by a stable GEOMETRIC key (min endpoint coords);
                    // min is order-invariant, so the ptr iteration order is irrelevant.
                    let p = *unprocessed_quad_starts
                        .iter()
                        .min_by_key(|&&e| {
                            let er = &*e;
                            let f = er.from.unwrap().as_ref().p;
                            let t = er.to.unwrap().as_ref().p;
                            (f.x, f.y, t.x, t.y)
                        })
                        .unwrap();
                    EdgePtr::new(p as *mut _).unwrap()
                };
                // SkeletalTrapezoidation.cpp:1998 edge_t* quad_start = poly_domain_start;
                let mut quad_start = poly_domain_start;
                // SkeletalTrapezoidation.cpp:1999 bool new_domain_start = true;
                let mut new_domain_start = true;
                // SkeletalTrapezoidation.cpp:2000 do { ... } while(quad_start = quad_start->getNextUnconnected(), quad_start != poly_domain_start);
                loop {
                    // SkeletalTrapezoidation.cpp:2002 edge_t* quad_end = quad_start;
                    let mut quad_end = quad_start;
                    // SkeletalTrapezoidation.cpp:2003 while (quad_end->next) quad_end = quad_end->next;
                    while let Some(next) = quad_end.as_ref().next {
                        quad_end = next;
                    }

                    // SkeletalTrapezoidation.cpp:2008 edge_t* edge_to_peak = getQuadMaxRedgeTo(quad_start);
                    let edge_to_peak = self.get_quad_max_redge_to(quad_start);
                    // SkeletalTrapezoidation.cpp:2010 edge_t* edge_from_peak = edge_to_peak->next; assert(edge_from_peak);
                    let edge_from_peak = edge_to_peak.as_ref().next.unwrap();

                    // SkeletalTrapezoidation.cpp:2012 unprocessed_quad_starts.erase(quad_start);
                    unprocessed_quad_starts.remove(&(quad_start.as_ptr() as *const _));

                    // SkeletalTrapezoidation.cpp:2014 if (! edge_to_peak->data.hasExtrusionJunctions())
                    if !edge_to_peak.as_ref().data.has_extrusion_junctions(false) {
                        // SkeletalTrapezoidation.cpp:2016 edge_junctions.emplace_back(std::make_shared<LineJunctions>());
                        let lj = Arc::new(RwLock::new(LineJunctions::new()));
                        edge_junctions.push(lj.clone());
                        // SkeletalTrapezoidation.cpp:2017 edge_to_peak->data.setExtrusionJunctions(edge_junctions.back());
                        edge_to_peak.as_ptr().as_mut().unwrap().data.set_extrusion_junctions(lj);
                    }
                    // SkeletalTrapezoidation.cpp:2020 LineJunctions from_junctions = *edge_to_peak->data.getExtrusionJunctions();
                    let mut from_junctions: LineJunctions =
                        edge_to_peak.as_ref().data.get_extrusion_junctions().unwrap().read().clone();
                    // SkeletalTrapezoidation.cpp:2021 if (! edge_from_peak->twin->data.hasExtrusionJunctions())
                    let edge_from_peak_twin = edge_from_peak.as_ref().twin.unwrap();
                    if !edge_from_peak_twin.as_ref().data.has_extrusion_junctions(false) {
                        // SkeletalTrapezoidation.cpp:2023 edge_junctions.emplace_back(std::make_shared<LineJunctions>());
                        let lj = Arc::new(RwLock::new(LineJunctions::new()));
                        edge_junctions.push(lj.clone());
                        // SkeletalTrapezoidation.cpp:2024 edge_from_peak->twin->data.setExtrusionJunctions(edge_junctions.back());
                        edge_from_peak_twin.as_ptr().as_mut().unwrap().data.set_extrusion_junctions(lj);
                    }
                    // SkeletalTrapezoidation.cpp:2027 LineJunctions to_junctions = *edge_from_peak->twin->data.getExtrusionJunctions();
                    let mut to_junctions: LineJunctions =
                        edge_from_peak_twin.as_ref().data.get_extrusion_junctions().unwrap().read().clone();
                    // SkeletalTrapezoidation.cpp:2028 if (edge_to_peak->prev)
                    if let Some(edge_to_peak_prev) = edge_to_peak.as_ref().prev {
                        // SkeletalTrapezoidation.cpp:2030 LineJunctions from_prev_junctions = *edge_to_peak->prev->data.getExtrusionJunctions();
                        let from_prev_junctions: LineJunctions =
                            edge_to_peak_prev.as_ref().data.get_extrusion_junctions().unwrap().read().clone();
                        // SkeletalTrapezoidation.cpp:2031 while (!from_junctions.empty() && !from_prev_junctions.empty() && from_junctions.back().perimeter_index <= from_prev_junctions.front().perimeter_index)
                        while !from_junctions.is_empty()
                            && !from_prev_junctions.is_empty()
                            && from_junctions.last().unwrap().perimeter_index
                                <= from_prev_junctions.first().unwrap().perimeter_index
                        {
                            // SkeletalTrapezoidation.cpp:2033 from_junctions.pop_back();
                            from_junctions.pop();
                        }
                        // SkeletalTrapezoidation.cpp:2035-2036 from_junctions.insert(end, from_prev_junctions.begin(), from_prev_junctions.end());
                        from_junctions.extend(from_prev_junctions.iter().cloned());
                        // SkeletalTrapezoidation.cpp:2037 assert(!edge_to_peak->prev->prev);
                        // SkeletalTrapezoidation.cpp:2038 if(edge_to_peak->prev->prev)
                        if edge_to_peak_prev.as_ref().prev.is_some() {
                            log::warn!("The edge we're about to connect is already connected.");
                        }
                    }
                    // SkeletalTrapezoidation.cpp:2043 if (edge_from_peak->next)
                    if let Some(edge_from_peak_next) = edge_from_peak.as_ref().next {
                        // SkeletalTrapezoidation.cpp:2045 LineJunctions to_next_junctions = *edge_from_peak->next->twin->data.getExtrusionJunctions();
                        let to_next_junctions: LineJunctions = edge_from_peak_next
                            .as_ref()
                            .twin
                            .unwrap()
                            .as_ref()
                            .data
                            .get_extrusion_junctions()
                            .unwrap()
                            .read()
                            .clone();
                        // SkeletalTrapezoidation.cpp:2046 while (!to_junctions.empty() && !to_next_junctions.empty() && to_junctions.back().perimeter_index <= to_next_junctions.front().perimeter_index)
                        while !to_junctions.is_empty()
                            && !to_next_junctions.is_empty()
                            && to_junctions.last().unwrap().perimeter_index
                                <= to_next_junctions.first().unwrap().perimeter_index
                        {
                            // SkeletalTrapezoidation.cpp:2048 to_junctions.pop_back();
                            to_junctions.pop();
                        }
                        // SkeletalTrapezoidation.cpp:2050-2051 to_junctions.insert(end, to_next_junctions.begin(), to_next_junctions.end());
                        to_junctions.extend(to_next_junctions.iter().cloned());
                        // SkeletalTrapezoidation.cpp:2052 assert(!edge_from_peak->next->next);
                        // SkeletalTrapezoidation.cpp:2053 if(edge_from_peak->next->next)
                        if edge_from_peak_next.as_ref().next.is_some() {
                            log::warn!("The edge we're about to connect is already connected!");
                        }
                    }
                    // SkeletalTrapezoidation.cpp:2058 assert(std::abs(int(from_junctions.size()) - int(to_junctions.size())) <= 1);
                    // SkeletalTrapezoidation.cpp:2059 if(std::abs(int(from_junctions.size()) - int(to_junctions.size())) > 1)
                    if (from_junctions.len() as i64 - to_junctions.len() as i64).abs() > 1 {
                        log::warn!(
                            "Can't create a transition when connecting two perimeters where the number of beads differs too much! {} vs. {}",
                            from_junctions.len(),
                            to_junctions.len()
                        );
                    }

                    // SkeletalTrapezoidation.cpp:2064 size_t segment_count = std::min(from_junctions.size(), to_junctions.size());
                    let segment_count = std::cmp::min(from_junctions.len(), to_junctions.len());
                    // SkeletalTrapezoidation.cpp:2065 for (size_t junction_rev_idx = 0; junction_rev_idx < segment_count; junction_rev_idx++)
                    for junction_rev_idx in 0..segment_count {
                        // SkeletalTrapezoidation.cpp:2067 ExtrusionJunction& from = from_junctions[from_junctions.size() - 1 - junction_rev_idx];
                        let from_j = from_junctions[from_junctions.len() - 1 - junction_rev_idx].clone();
                        // SkeletalTrapezoidation.cpp:2068 ExtrusionJunction& to = to_junctions[to_junctions.size() - 1 - junction_rev_idx];
                        let to_j = to_junctions[to_junctions.len() - 1 - junction_rev_idx].clone();
                        // R545 probe (CJPROBE=1): does connect_junctions RECEIVE width
                        // variation? Each segment pairs a `from` junction (this quad's
                        // peak side) with a `to` junction (the other side). If the two
                        // ends of a segment already carry the same width, the flatness
                        // is upstream of this function; if they differ, something here
                        // drops it. Probe the input, not the transform (R541).
                        if crate::probe_enabled("CJPROBE") {
                            cjprobe(from_j.w, to_j.w);
                        }
                        // SkeletalTrapezoidation.cpp:2069 assert(from.perimeter_index == to.perimeter_index);
                        // SkeletalTrapezoidation.cpp:2070 if(from.perimeter_index != to.perimeter_index)
                        if from_j.perimeter_index != to_j.perimeter_index {
                            log::warn!(
                                "Connecting two perimeters with different indices! Perimeter {} and {}",
                                from_j.perimeter_index,
                                to_j.perimeter_index
                            );
                        }
                        // SkeletalTrapezoidation.cpp:2074-2078 const bool from_is_odd = ...;
                        let quad_start_to = quad_start.as_ref().to.unwrap();
                        let from_is_odd = quad_start_to.as_ref().data.bead_count > 0
                            && quad_start_to.as_ref().data.bead_count % 2 == 1
                            && quad_start_to.as_ref().data.transition_ratio == 0.0
                            && junction_rev_idx == segment_count - 1
                            && shorter_then(&(from_j.p - quad_start_to.as_ref().p), scaled(0.005));
                        // SkeletalTrapezoidation.cpp:2079-2083 const bool to_is_odd = ...;
                        let quad_end_from = quad_end.as_ref().from.unwrap();
                        let to_is_odd = quad_end_from.as_ref().data.bead_count > 0
                            && quad_end_from.as_ref().data.bead_count % 2 == 1
                            && quad_end_from.as_ref().data.transition_ratio == 0.0
                            && junction_rev_idx == segment_count - 1
                            && shorter_then(&(to_j.p - quad_end_from.as_ref().p), scaled(0.005));
                        // SkeletalTrapezoidation.cpp:2084 const bool is_odd_segment = from_is_odd && to_is_odd;
                        let is_odd_segment = from_is_odd && to_is_odd;
                        // SkeletalTrapezoidation.cpp:2085-2086 if (is_odd_segment && passed_odd_edges.count(quad_start->next->twin) > 0)
                        if is_odd_segment
                            && passed_odd_edges.contains(
                                &(quad_start.as_ref().next.unwrap().as_ref().twin.unwrap().as_ptr()
                                    as *const _),
                            )
                        {
                            // SkeletalTrapezoidation.cpp:2088 continue;
                            continue;
                        }
                        // SkeletalTrapezoidation.cpp:2090 bool from_is_3way = from_is_odd && quad_start->to->isMultiIntersection();
                        let from_is_3way =
                            from_is_odd && as_st_node(quad_start_to).is_multi_intersection();
                        // SkeletalTrapezoidation.cpp:2091 bool to_is_3way = to_is_odd && quad_end->from->isMultiIntersection();
                        let to_is_3way = to_is_odd && as_st_node(quad_end_from).is_multi_intersection();
                        // SkeletalTrapezoidation.cpp:2092 passed_odd_edges.emplace(quad_start->next);
                        passed_odd_edges.insert(quad_start.as_ref().next.unwrap().as_ptr() as *const _);

                        // SkeletalTrapezoidation.cpp:2094 addToolpathSegment(from, to, is_odd_segment, new_domain_start, from_is_3way, to_is_3way);
                        self.add_toolpath_segment(
                            &from_j,
                            &to_j,
                            is_odd_segment,
                            new_domain_start,
                            from_is_3way,
                            to_is_3way,
                        );
                    }
                    // SkeletalTrapezoidation.cpp:2096 new_domain_start = false;
                    new_domain_start = false;

                    // SkeletalTrapezoidation.cpp:2098 while(quad_start = quad_start->getNextUnconnected(), quad_start != poly_domain_start);
                    match as_st_edge(quad_start).get_next_unconnected() {
                        Some(next) if next != poly_domain_start => {
                            quad_start = next;
                        }
                        _ => break,
                    }
                }
            }
        }
    }

    // SkeletalTrapezoidation.cpp:2102-2136
    // void SkeletalTrapezoidation::generateLocalMaximaSingleBeads()
    pub fn generate_local_maxima_single_beads(&mut self) {
        unsafe {
            // SkeletalTrapezoidation.cpp:2104 std::vector<VariableWidthLines>& generated_toolpaths = *p_generated_toolpaths;
            let generated_toolpaths = &mut *self.p_generated_toolpaths;

            // SkeletalTrapezoidation.cpp:2106 for (auto& node : graph.nodes)
            for node in self.graph.nodes.iter() {
                // SkeletalTrapezoidation.cpp:2108 if (! node.data.hasBeading())
                if !node.base.data.has_beading() {
                    // SkeletalTrapezoidation.cpp:2110 continue;
                    continue;
                }
                // SkeletalTrapezoidation.cpp:2112 Beading& beading = node.data.getBeading()->beading;
                let beading_arc = node.base.data.get_beading().unwrap();
                let beading = beading_arc.read().beading.clone();
                // SkeletalTrapezoidation.cpp:2113 if (beading.bead_widths.size() % 2 == 1 && node.isLocalMaximum(true) && !node.isCentral())
                if beading.bead_widths.len() % 2 == 1
                    && node.is_local_maximum(true)
                    && !node.is_central()
                {
                    // SkeletalTrapezoidation.cpp:2115 const size_t inset_index = beading.bead_widths.size() / 2;
                    let inset_index = beading.bead_widths.len() / 2;
                    // SkeletalTrapezoidation.cpp:2116 constexpr bool is_odd = true;
                    let is_odd = true;
                    // SkeletalTrapezoidation.cpp:2117 if (inset_index >= generated_toolpaths.size())
                    if inset_index >= generated_toolpaths.len() {
                        // SkeletalTrapezoidation.cpp:2119 generated_toolpaths.resize(inset_index + 1);
                        generated_toolpaths.resize(inset_index + 1, VariableWidthLines::new());
                    }
                    // SkeletalTrapezoidation.cpp:2121 generated_toolpaths[inset_index].emplace_back(inset_index, is_odd);
                    generated_toolpaths[inset_index].push(ExtrusionLine::new(inset_index, is_odd));
                    // SkeletalTrapezoidation.cpp:2122 ExtrusionLine& line = generated_toolpaths[inset_index].back();
                    let line = generated_toolpaths[inset_index].last_mut().unwrap();
                    // SkeletalTrapezoidation.cpp:2123 const coord_t width = beading.bead_widths[inset_index];
                    let width = beading.bead_widths[inset_index];
                    // SkeletalTrapezoidation.cpp:2128 const coord_t r = width / 8;
                    let r = width / 8;
                    // SkeletalTrapezoidation.cpp:2129 constexpr coord_t n_segments = 6;
                    let n_segments: Coord = 6;
                    // SkeletalTrapezoidation.cpp:2130 for (coord_t segment = 0; segment < n_segments; segment++)
                    for segment in 0..n_segments {
                        // SkeletalTrapezoidation.cpp:2131 float a = 2.0 * M_PI / n_segments * segment;
                        // C++ `a` is `float`: the double expression is truncated to f32, then
                        // `cos(a)`/`sin(a)` use the float overloads and `r * cos(a)` is f32.
                        let a: f32 =
                            (2.0_f64 * std::f64::consts::PI / n_segments as f64 * segment as f64) as f32;
                        // SkeletalTrapezoidation.cpp:2132 line.junctions.emplace_back(ExtrusionJunction(node.p + Point(r * cos(a), r * sin(a)), width, inset_index, false));
                        line.junctions.push(ExtrusionJunction::with_hole_compensation(
                            node.base.p
                                + Point::new(
                                    (r as f32 * a.cos()) as Coord,
                                    (r as f32 * a.sin()) as Coord,
                                ),
                            width,
                            inset_index,
                            false,
                        ));
                    }
                }
            }
        }
    }

    // SkeletalTrapezoidation.cpp:1405-1467
    // void SkeletalTrapezoidation::generateExtraRibs()
    pub fn generate_extra_ribs(&mut self) {
        unsafe {
            // SkeletalTrapezoidation.cpp:1407 for (auto edge_it = graph.edges.begin(); edge_it != graph.edges.end(); ++edge_it)
            //
            // insertNode appends new edges after `end()`; snapshot the original edge set.
            let edges: Vec<EdgePtr> = self
                .graph
                .edges
                .iter()
                .map(|e| SkeletalTrapezoidationGraph::edge_ptr(e))
                .collect();
            for edge in edges {
                // SkeletalTrapezoidation.cpp:1409 edge_t& edge = *edge_it;
                let edge_ref = edge.as_ref();
                let from = edge_ref.from.unwrap();
                let to = edge_ref.to.unwrap();

                // SkeletalTrapezoidation.cpp:1411-1414 if (!edge.data.isCentral() || shorter_then(edge.to->p - edge.from->p, discretization_step_size) || edge.from->data.distance_to_boundary >= edge.to->data.distance_to_boundary)
                if !edge_ref.data.is_central()
                    || shorter_then(&(to.as_ref().p - from.as_ref().p), self.discretization_step_size)
                    || from.as_ref().data.distance_to_boundary >= to.as_ref().data.distance_to_boundary
                {
                    // SkeletalTrapezoidation.cpp:1415 continue;
                    continue;
                }

                // SkeletalTrapezoidation.cpp:1419 std::vector<coord_t> rib_thicknesses = beading_strategy.getNonlinearThicknesses(edge.from->data.bead_count);
                let rib_thicknesses = self
                    .beading_strategy
                    .get_nonlinear_thicknesses(from.as_ref().data.bead_count);

                // SkeletalTrapezoidation.cpp:1421 if (rib_thicknesses.empty()) continue;
                if rib_thicknesses.is_empty() {
                    continue;
                }

                // SkeletalTrapezoidation.cpp:1424 node_t* from = edge.from;
                // SkeletalTrapezoidation.cpp:1425 node_t* to = edge.to;
                // SkeletalTrapezoidation.cpp:1426 Point a = from->p;
                let a = from.as_ref().p;
                // SkeletalTrapezoidation.cpp:1427 Point b = to->p;
                let b = to.as_ref().p;
                // SkeletalTrapezoidation.cpp:1428 Point ab = b - a;
                let ab = b - a;
                // SkeletalTrapezoidation.cpp:1429 coord_t ab_size = ab.cast<int64_t>().norm();
                let ab_size = ab.length() as Coord;
                // SkeletalTrapezoidation.cpp:1430 coord_t a_R = edge.from->data.distance_to_boundary;
                let a_r = from.as_ref().data.distance_to_boundary;
                // SkeletalTrapezoidation.cpp:1431 coord_t b_R = edge.to->data.distance_to_boundary;
                let b_r = to.as_ref().data.distance_to_boundary;

                // SkeletalTrapezoidation.cpp:1433 edge_t* last_edge_replacing_input = &edge;
                let mut last_edge_replacing_input = edge;
                // SkeletalTrapezoidation.cpp:1434 for (coord_t rib_thickness : rib_thicknesses)
                for rib_thickness in rib_thicknesses {
                    // SkeletalTrapezoidation.cpp:1436 if (rib_thickness / 2 <= a_R)
                    if rib_thickness / 2 <= a_r {
                        // SkeletalTrapezoidation.cpp:1438 continue;
                        continue;
                    }
                    // SkeletalTrapezoidation.cpp:1440 if (rib_thickness / 2 >= b_R)
                    if rib_thickness / 2 >= b_r {
                        // SkeletalTrapezoidation.cpp:1442 break;
                        break;
                    }

                    // SkeletalTrapezoidation.cpp:1445 coord_t new_node_bead_count = std::min(edge.from->data.bead_count, edge.to->data.bead_count);
                    let new_node_bead_count = std::cmp::min(
                        from.as_ref().data.bead_count,
                        to.as_ref().data.bead_count,
                    );
                    // SkeletalTrapezoidation.cpp:1446 coord_t end_pos = int64_t(ab_size) * int64_t(rib_thickness / 2 - a_R) / int64_t(b_R - a_R);
                    let end_pos =
                        (ab_size as i64) * ((rib_thickness / 2 - a_r) as i64) / ((b_r - a_r) as i64);
                    // SkeletalTrapezoidation.cpp:1447 assert(end_pos > 0);
                    debug_assert!(end_pos > 0);
                    // SkeletalTrapezoidation.cpp:1448 assert(end_pos < ab_size);
                    debug_assert!(end_pos < ab_size);
                    // SkeletalTrapezoidation.cpp:1449 node_t* close_node = (end_pos < ab_size / 2)? from : to;
                    let close_node = if end_pos < ab_size / 2 { from } else { to };
                    // SkeletalTrapezoidation.cpp:1450 if ((end_pos < snap_dist || end_pos > ab_size - snap_dist) && close_node->data.bead_count == new_node_bead_count)
                    if (end_pos < SNAP_DIST || end_pos > ab_size - SNAP_DIST)
                        && close_node.as_ref().data.bead_count == new_node_bead_count
                    {
                        // SkeletalTrapezoidation.cpp:1454 assert(end_pos <= ab_size);
                        debug_assert!(end_pos <= ab_size);
                        // SkeletalTrapezoidation.cpp:1455 close_node->data.transition_ratio = 0;
                        close_node.as_ptr().as_mut().unwrap().data.transition_ratio = 0.0;
                        // SkeletalTrapezoidation.cpp:1456 continue;
                        continue;
                    }
                    // SkeletalTrapezoidation.cpp:1458 Point mid = a + normal(ab, end_pos);
                    let mid = a + normal(ab, end_pos);

                    // SkeletalTrapezoidation.cpp:1460-1464 asserts + insertNode
                    debug_assert!(last_edge_replacing_input.as_ref().data.is_central());
                    debug_assert!(
                        last_edge_replacing_input.as_ref().data.edge_type != EdgeType::ExtraVd
                    );
                    // SkeletalTrapezoidation.cpp:1462 last_edge_replacing_input = graph.insertNode(last_edge_replacing_input, mid, new_node_bead_count);
                    last_edge_replacing_input =
                        self.graph.insert_node(last_edge_replacing_input, mid, new_node_bead_count);
                    debug_assert!(
                        last_edge_replacing_input.as_ref().data.edge_type != EdgeType::ExtraVd
                    );
                    debug_assert!(last_edge_replacing_input.as_ref().data.is_central());
                }
            }
        }
    }
}

// SkeletalTrapezoidation.cpp:1310-1316
// static inline Point normal(const Point& p0, coord_t len)
#[inline]
fn normal(p0: Point, len: Coord) -> Point {
    // SkeletalTrapezoidation.cpp:1312 int64_t _len = p0.cast<int64_t>().norm();
    let _len = p0.length() as i64;
    // SkeletalTrapezoidation.cpp:1313 if (_len < 1)
    if _len < 1 {
        // SkeletalTrapezoidation.cpp:1314 return Point(len, 0);
        return Point::new(len, 0);
    }
    // SkeletalTrapezoidation.cpp:1315 return (p0.cast<int64_t>() * int64_t(len) / _len).cast<coord_t>();
    Point::new(p0.x * len / _len, p0.y * len / _len)
}

// ---------------------------------------------------------------------------
// VoronoiUtils source-lookup helpers, specialised to the `PolygonsSegmentIndex`
// segment type used by Arachne (== C++ `Segment`). These mirror the templated
// `Geometry::VoronoiUtils` members; the `Line`-based instantiations already live
// in `voronoi_utils_cgal.rs`. Faithful 1:1 of VoronoiUtils.cpp.
// ---------------------------------------------------------------------------

// VoronoiUtils.cpp:40-49  VoronoiUtils::get_source_segment
fn source_segment<'a>(
    cell: &bv::Cell,
    segments: &'a [PolygonsSegmentIndex<'a>],
) -> &'a PolygonsSegmentIndex<'a> {
    // VoronoiUtils.cpp:42 if (!cell.contains_segment()) throw ...
    assert!(cell.contains_segment(), "Voronoi cell doesn't contain a source segment!");
    // VoronoiUtils.cpp:45-46 if (cell.source_index() >= ...) throw ...
    let source_index = cell.source_index().usize();
    assert!(source_index < segments.len(), "Voronoi cell source index is out of range!");
    // VoronoiUtils.cpp:48 return *(segment_begin + cell.source_index());
    &segments[source_index]
}

// VoronoiUtils.cpp:56-76  VoronoiUtils::get_source_point
fn source_point(cell: &bv::Cell, segments: &[PolygonsSegmentIndex]) -> Point {
    // VoronoiUtils.cpp:60 if (!cell.contains_point()) throw ...
    assert!(cell.contains_point(), "Voronoi cell doesn't contain a source point!");
    let source_index = cell.source_index().usize();
    match cell.source_category() {
        // VoronoiUtils.cpp:63-66 SEGMENT_START_POINT -> segment LOW (from)
        bv::SourceCategory::SegmentStart => {
            debug_assert!(source_index < segments.len());
            segments[source_index].segment_get(Direction1d::Low)
        }
        // VoronoiUtils.cpp:67-70 SEGMENT_END_POINT -> segment HIGH (to)
        bv::SourceCategory::SegmentEnd => {
            debug_assert!(source_index < segments.len());
            segments[source_index].segment_get(Direction1d::High)
        }
        // VoronoiUtils.cpp:71-72 SINGLE_POINT
        bv::SourceCategory::SinglePoint => {
            panic!("Voronoi diagram is always constructed using segments, so cell.source_category() shouldn't be SOURCE_CATEGORY_SINGLE_POINT!");
        }
        // VoronoiUtils.cpp:73-74 default
        bv::SourceCategory::Segment => {
            panic!("Function get_source_point() should only be called on point cells!");
        }
    }
}

// VoronoiUtils.cpp (get_source_point_index) — returns the `PolygonsPointIndex`
// of the cell's source point.
fn source_point_index<'a>(
    cell: &bv::Cell,
    segments: &'a [PolygonsSegmentIndex<'a>],
) -> PolygonsPointIndex<'a> {
    assert!(cell.contains_point(), "Voronoi cell doesn't contain a source point!");
    let source_index = cell.source_index().usize();
    match cell.source_category() {
        // SEGMENT_START_POINT -> the segment's own point index
        bv::SourceCategory::SegmentStart => {
            debug_assert!(source_index < segments.len());
            *segments[source_index].point_index()
        }
        // SEGMENT_END_POINT -> the next point index
        bv::SourceCategory::SegmentEnd => {
            debug_assert!(source_index < segments.len());
            segments[source_index].point_index().next()
        }
        bv::SourceCategory::SinglePoint => {
            panic!("Voronoi diagram is always constructed using segments, so cell.source_category() shouldn't be SOURCE_CATEGORY_SINGLE_POINT!");
        }
        bv::SourceCategory::Segment => {
            panic!("Function get_source_point_index() should only be called on point cells!");
        }
    }
}

/// Result of `compute_segment_cell_range` — mirrors C++ `SegmentCellRange<Point>`
/// but carries the boost-VD edge handles as live `bv::EdgeIndex` (the crate's
/// `SegmentCellRange` stores `Option<usize>`, and `bv::EdgeIndex`'s inner u32 is
/// not constructible from outside `boostvoronoi`, so the cell range is kept local).
struct SegmentCellRangeBv {
    segment_start_point: Point,
    segment_end_point: Point,
    edge_begin: Option<bv::EdgeIndex>,
    edge_end: Option<bv::EdgeIndex>,
}

impl SegmentCellRangeBv {
    fn is_valid(&self) -> bool {
        match (self.edge_begin, self.edge_end) {
            (Some(b), Some(e)) => b != e,
            _ => false,
        }
    }
}

// VoronoiUtils.cpp:205-243  VoronoiUtils::compute_segment_cell_range
fn compute_segment_cell_range(
    diagram: &bv::Diagram,
    cell: &bv::Cell,
    segments: &[PolygonsSegmentIndex],
) -> SegmentCellRangeBv {
    // VoronoiUtils.cpp:211 const Segment &source_segment = get_source_segment(cell, ...);
    let source_segment = source_segment(cell, segments);
    // VoronoiUtils.cpp:212 const Point from = segment_traits::get(source_segment, LOW);
    let from = source_segment.segment_get(Direction1d::Low);
    // VoronoiUtils.cpp:213 const Point to = segment_traits::get(source_segment, HIGH);
    let to = source_segment.segment_get(Direction1d::High);
    // VoronoiUtils.cpp:214-215 const Vec2i64 from_i64 = from; to_i64 = to;
    let from_i64 = (from.x, from.y);
    let to_i64 = (to.x, to.y);

    // VoronoiUtils.cpp:218 SegmentCellRange cell_range(to, from);
    let mut cell_range = SegmentCellRangeBv {
        segment_start_point: to,
        segment_end_point: from,
        edge_begin: None,
        edge_end: None,
    };

    // VoronoiUtils.cpp:221-223
    let mut seen_possible_start = false;
    let mut after_start = false;
    let mut ending_edge_is_set_before_start = false;
    // VoronoiUtils.cpp:224 const VD::edge_type *edge = cell.incident_edge();
    let incident_edge = cell.get_incident_edge().unwrap();
    let mut edge = incident_edge;
    // VoronoiUtils.cpp:225-241 do { ... } while (edge = edge->next(), edge != cell.incident_edge());
    loop {
        // VoronoiUtils.cpp:226-227 if (edge->is_infinite()) continue;
        if diagram.edge_is_infinite(edge).unwrap_or(true) {
            edge = diagram.edges()[edge.usize()].next().unwrap();
            if edge == incident_edge {
                break;
            }
            continue;
        }

        // VoronoiUtils.cpp:229-230 Vec2i64 v0 = to_point(edge->vertex0()); v1 = to_point(edge->vertex1());
        let v0i = diagram.edges()[edge.usize()].vertex0().unwrap();
        let v1i = diagram.edge_get_vertex1(edge).ok().flatten().unwrap();
        let v0v = &diagram.vertices()[v0i.usize()];
        let v1v = &diagram.vertices()[v1i.usize()];
        let v0 = (v0v.x().round() as i64, v0v.y().round() as i64);
        let v1 = (v1v.x().round() as i64, v1v.y().round() as i64);
        // VoronoiUtils.cpp:231 assert(v0 != to_i64 || v1 != from_i64);
        debug_assert!(v0 != to_i64 || v1 != from_i64);

        // VoronoiUtils.cpp:233-237
        if v0 == to_i64 && !after_start {
            cell_range.edge_begin = Some(edge);
            seen_possible_start = true;
        } else if seen_possible_start {
            after_start = true;
        }

        // VoronoiUtils.cpp:239-241
        if v1 == from_i64 && (cell_range.edge_end.is_none() || ending_edge_is_set_before_start) {
            ending_edge_is_set_before_start = !after_start;
            cell_range.edge_end = Some(edge);
        }

        // VoronoiUtils.cpp:241 while (edge = edge->next(), edge != cell.incident_edge());
        edge = diagram.edges()[edge.usize()].next().unwrap();
        if edge == incident_edge {
            break;
        }
    }

    // VoronoiUtils.cpp:243 return cell_range;
    cell_range
}

/// R543 probe (BEADPROBE=1): the input side of `BeadingStrategy::compute`.
///
/// R541/R542 established that the path builder is faithful and that our
/// per-loop bead widths are 98% flat while C++ varies them along the wall.
/// Applying R541's own lesson one level further up: measure what `compute` is
/// being ASKED for before suspecting what it returns.
#[allow(dead_code)]
pub(crate) fn beadprobe(thickness: i64, bead_count: i64, widths: &[i64]) {
    use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
    use std::sync::Mutex;
    static CALLS: AtomicUsize = AtomicUsize::new(0);
    static THICK: Mutex<Vec<i64>> = Mutex::new(Vec::new());
    static W0: Mutex<Vec<i64>> = Mutex::new(Vec::new());
    static FLATW: AtomicUsize = AtomicUsize::new(0);
    let n = CALLS.fetch_add(1, Relaxed) + 1;
    if widths.len() > 1 {
        let mn = widths.iter().min().copied().unwrap_or(0);
        let mx = widths.iter().max().copied().unwrap_or(0);
        if mn == mx {
            FLATW.fetch_add(1, Relaxed);
        }
    }
    if let (Ok(mut t), Ok(mut w)) = (THICK.lock(), W0.lock()) {
        t.push(thickness);
        if let Some(&w0) = widths.first() {
            w.push(w0);
        }
        if n % 20_000 == 0 || n == 2_000 {
            let mut td: Vec<i64> = t.clone();
            td.sort_unstable();
            td.dedup();
            let mut wd: Vec<i64> = w.clone();
            wd.sort_unstable();
            wd.dedup();
            let tmin = t.iter().min().copied().unwrap_or(0);
            let tmax = t.iter().max().copied().unwrap_or(0);
            let wmin = w.iter().min().copied().unwrap_or(0);
            let wmax = w.iter().max().copied().unwrap_or(0);
            eprintln!(
                "[BEADPROBE] compute calls={n} | thickness distinct={} range={:.3}..{:.3}mm | \
                 bead_widths[0] distinct={} range={:.3}..{:.3}mm | multi-bead beadings with all-equal widths={}",
                td.len(), tmin as f64 / 1e5, tmax as f64 / 1e5,
                wd.len(), wmin as f64 / 1e5, wmax as f64 / 1e5,
                FLATW.load(Relaxed),
            );
        }
    }
}

/// R543 probe (BEADPROBE=1): the width actually stamped on each ExtrusionJunction
/// at creation, i.e. AFTER propagation/interpolation but BEFORE WallToolPaths
/// post-processing. Compare against `beadprobe` (what `compute` produced) and
/// `ARACHWIDTH` (what the perimeter generator finally sees).
#[allow(dead_code)]
// R571: the junction site is the FORK. `bead_widths[idx]` comes from a Beading
// whose input is `total_thickness`. Counting DISTINCT values of each (order-
// independent, so safe under rayon — R559) says which side is flat:
//   many thicknesses, few widths  -> compute/interpolate flattens
//   few thicknesses               -> the skeleton is flat, upstream of compute
pub(crate) static EP_EDGES: std::sync::atomic::AtomicUsize =
    std::sync::atomic::AtomicUsize::new(0);

/// R585 probe (BEADPAIR=1): the fraction of graph edges whose two endpoints carry
/// beadings that DISAGREE on `bead_widths[0]`, plus the size distribution of the
/// disagreement and a quantisation histogram of `bead_widths[0] % 100`.
///
/// R584 reduced the outer-wall change-density gap to exactly this quantity:
/// junctions-per-edge is at parity (1.022x) and every width change is one bead
/// index resolving to a different beading at an edge boundary, so
/// `changes/junction ~ P(differ) / (junctions per edge)`. A per-edge Bernoulli
/// rate is order-independent, so it is safe to compare across engines -- the
/// prefix distinct-count R584 retracted was not.
///
/// The `% 100` histogram is the side-check: coord_t is 1e-5 mm, so 100 units is
/// one micron. If our widths land on a coarser grid than C++'s, neighbouring
/// beadings collapse to equal and P(differ) falls with everything structural at
/// parity.
pub(crate) fn beadpair(
    w_to: Option<i64>,
    w_from: Option<i64>,
    t_from: Option<i64>,
    t_to: i64,
    same_obj: bool,
) {
    use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
    use std::sync::Mutex;
    static N: AtomicUsize = AtomicUsize::new(0);
    static BOTH: AtomicUsize = AtomicUsize::new(0);
    static DIFF: AtomicUsize = AtomicUsize::new(0);
    static TDIFF: AtomicUsize = AtomicUsize::new(0);
    static D1: AtomicUsize = AtomicUsize::new(0);
    static D10: AtomicUsize = AtomicUsize::new(0);
    static D100: AtomicUsize = AtomicUsize::new(0);
    static DBIG: AtomicUsize = AtomicUsize::new(0);
    static MOD: Mutex<[usize; 100]> = Mutex::new([0; 100]);
    // SAME: the two endpoints resolve to the SAME BeadingPropagation object, so
    // their widths are identical by construction rather than by computation. If we
    // share beadings across neighbouring nodes more than C++ does, that alone
    // depresses P(differ).
    static SAME: AtomicUsize = AtomicUsize::new(0);
    let n = N.fetch_add(1, Relaxed) + 1;
    if let Some(wt) = w_to {
        if let Ok(mut m) = MOD.lock() {
            m[(wt.rem_euclid(100)) as usize] += 1;
        }
        if let Some(wf) = w_from {
            BOTH.fetch_add(1, Relaxed);
            if same_obj {
                SAME.fetch_add(1, Relaxed);
            }
            let d = (wt - wf).abs();
            if d != 0 {
                DIFF.fetch_add(1, Relaxed);
            }
            if t_from.map(|tf| tf != t_to).unwrap_or(false) {
                TDIFF.fetch_add(1, Relaxed);
            }
            // coord_t is 1e-5 mm, so 100 units == 1 micron.
            if d == 0 {
            } else if d < 100 {
                D1.fetch_add(1, Relaxed);
            } else if d < 1000 {
                D10.fetch_add(1, Relaxed);
            } else if d < 10000 {
                D100.fetch_add(1, Relaxed);
            } else {
                DBIG.fetch_add(1, Relaxed);
            }
        }
    }
    if n % 500_000 == 0 {
        let both = BOTH.load(Relaxed).max(1);
        let nz = MOD
            .lock()
            .map(|m| m.iter().filter(|&&c| c > 0).count())
            .unwrap_or(0);
        eprintln!(
            "[BEADPAIR] edges={n} both={} differ={} ({:.4}) tdiff={} ({:.4}) same_obj={} ({:.4}) | d<1um={} 1-10um={} 10-100um={} >100um={} | w0_mod100_nonzero={}/100",
            BOTH.load(Relaxed), DIFF.load(Relaxed),
            DIFF.load(Relaxed) as f64 / both as f64,
            TDIFF.load(Relaxed), TDIFF.load(Relaxed) as f64 / both as f64,
            SAME.load(Relaxed), SAME.load(Relaxed) as f64 / both as f64,
            D1.load(Relaxed), D10.load(Relaxed), D100.load(Relaxed), DBIG.load(Relaxed), nz,
        );
    }
}

pub(crate) fn junctionprobe(w: i64, total_thickness: i64, idx: usize) {
    use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
    use std::sync::Mutex;
    static N: AtomicUsize = AtomicUsize::new(0);
    static W: Mutex<Vec<i64>> = Mutex::new(Vec::new());
    static T: Mutex<Vec<i64>> = Mutex::new(Vec::new());
    static PAIRS: Mutex<Vec<(i64, i64)>> = Mutex::new(Vec::new());
    static IDX0: AtomicUsize = AtomicUsize::new(0);
    // R584: the outer wall draws every junction from bead_widths[0], so the global
    // distinct count (mixed across all bead indices) does not speak to its spread.
    static W0: Mutex<Vec<i64>> = Mutex::new(Vec::new());
    if idx == 0 {
        IDX0.fetch_add(1, Relaxed);
        if let Ok(mut v0) = W0.lock() {
            v0.push(w);
        }
    }
    let n = N.fetch_add(1, Relaxed) + 1;
    if let (Ok(mut v), Ok(mut t), Ok(mut pr)) = (W.lock(), T.lock(), PAIRS.lock()) {
        v.push(w);
        t.push(total_thickness);
        pr.push((total_thickness, w));
        // R584: was 20_000; the dedup sorts grow with the vector, so frequent
        // checkpoints made this quadratic and stalled the C++ run past 9 minutes.
        if n % 500_000 == 0 {
            let mut d = v.clone();
            d.sort_unstable();
            d.dedup();
            let mut dt = t.clone();
            dt.sort_unstable();
            dt.dedup();
            let mut dp = pr.clone();
            dp.sort_unstable();
            dp.dedup();
            let mn = v.iter().min().copied().unwrap_or(0);
            let mx = v.iter().max().copied().unwrap_or(0);
            let tmn = t.iter().min().copied().unwrap_or(0);
            let tmx = t.iter().max().copied().unwrap_or(0);
            let d0len = W0
                .lock()
                .map(|v0| {
                    let mut d0 = v0.clone();
                    d0.sort_unstable();
                    d0.dedup();
                    d0.len()
                })
                .unwrap_or(0);
            eprintln!(
                "[JUNCPROBE] junctions={n} idx0={} | distinct_width={} distinct_thick={} distinct_pairs={} | w_range={:.3}..{:.3}mm t_range={:.3}..{:.3}mm | edges={} juncs/edge={:.4} distinct_w0={} w0_per_idx0={:.6}",
                IDX0.load(Relaxed), d.len(), dt.len(), dp.len(),
                mn as f64 / 1e5, mx as f64 / 1e5, tmn as f64 / 1e5, tmx as f64 / 1e5,
                EP_EDGES.load(Relaxed),
                n as f64 / EP_EDGES.load(Relaxed).max(1) as f64,
                d0len,
                d0len as f64 / IDX0.load(Relaxed).max(1) as f64,
            );
        }
    }
}

/// R545 probe (CJPROBE=1): width agreement across each segment `connect_junctions`
/// builds. Reports how often the two ends of a segment carry equal widths and the
/// distribution of their difference.
#[allow(dead_code)]
pub(crate) fn cjprobe(from_w: i64, to_w: i64) {
    use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
    static N: AtomicUsize = AtomicUsize::new(0);
    static EQ: AtomicUsize = AtomicUsize::new(0);
    static D1: AtomicUsize = AtomicUsize::new(0);
    static D10: AtomicUsize = AtomicUsize::new(0);
    static DBIG: AtomicUsize = AtomicUsize::new(0);
    let n = N.fetch_add(1, Relaxed) + 1;
    let d = (from_w - to_w).abs();
    if d == 0 {
        EQ.fetch_add(1, Relaxed);
    } else if d < 100 {
        D1.fetch_add(1, Relaxed);
    } else if d < 1000 {
        D10.fetch_add(1, Relaxed);
    } else {
        DBIG.fetch_add(1, Relaxed);
    }
    if n == 1 || n % 500_000 == 0 {
        eprintln!(
            "[CJPROBE] segments={n} | from.w==to.w: {} ({:.1}%) | diff<1um: {} | 1-10um: {} | >10um: {}",
            EQ.load(Relaxed),
            100.0 * EQ.load(Relaxed) as f64 / n as f64,
            D1.load(Relaxed),
            D10.load(Relaxed),
            DBIG.load(Relaxed),
        );
    }
}

/// R546 probe (PROPPROBE=1): the copy-vs-interpolate split in
/// `propagate_beadings_downward_edge` (SkeletalTrapezoidation.cpp:1687-1704).
#[allow(dead_code)]
pub(crate) fn propprobe(ratio_of_top: f64, transition_dist: i64, total_dist: i64) {
    use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
    static N: AtomicUsize = AtomicUsize::new(0);
    static COPY: AtomicUsize = AtomicUsize::new(0);
    static CLAMPED: AtomicUsize = AtomicUsize::new(0);
    static TDLT: AtomicUsize = AtomicUsize::new(0);
    let n = N.fetch_add(1, Relaxed) + 1;
    if ratio_of_top >= 1.0 {
        COPY.fetch_add(1, Relaxed);
    }
    // how often is total_dist the binding term vs the transition dist?
    if total_dist < transition_dist {
        TDLT.fetch_add(1, Relaxed);
    }
    if ratio_of_top == 0.0 {
        CLAMPED.fetch_add(1, Relaxed);
    }
    if n == 1 || n % 5_000 == 0 {
        eprintln!(
            "[PROPPROBE] calls={n} | ratio>=1.0 (pure COPY)={} ({:.1}%) | ratio==0={} | \
             total_dist<transition_dist={} | transition_dist={:.3}mm",
            COPY.load(Relaxed),
            100.0 * COPY.load(Relaxed) as f64 / n as f64,
            CLAMPED.load(Relaxed),
            TDLT.load(Relaxed),
            transition_dist as f64 / 1e5,
        );
    }
}

/// R547: skeleton size at the head of `generate_segments`, mirroring the C++
/// `[CPP-GRAPHPROBE]` counter. Tests whether our ~5x deficit in
/// `BeadingStrategy::compute` calls comes from a smaller graph or from a
/// different share of nodes carrying a bead count.
pub(crate) fn graphprobe(nodes: usize, edges: usize, upward: usize, beaded: usize) {
    use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
    static CALLS: AtomicUsize = AtomicUsize::new(0);
    static N: AtomicUsize = AtomicUsize::new(0);
    static E: AtomicUsize = AtomicUsize::new(0);
    static U: AtomicUsize = AtomicUsize::new(0);
    static B: AtomicUsize = AtomicUsize::new(0);
    let c = CALLS.fetch_add(1, Relaxed) + 1;
    let n = N.fetch_add(nodes, Relaxed) + nodes;
    let e = E.fetch_add(edges, Relaxed) + edges;
    let u = U.fetch_add(upward, Relaxed) + upward;
    let b = B.fetch_add(beaded, Relaxed) + beaded;
    if c == 1 || c % 200 == 0 {
        eprintln!(
            "[GRAPHPROBE] generate_segments calls={c} | nodes={n} | edges={e} | \
             upward_quad_mids={u} | nodes with bead_count>0={b}"
        );
    }
}

impl SkeletalTrapezoidation<'_> {
    /// R548: census of central marking and `bead_count` after each marking stage,
    /// mirroring the C++ `[CPP-CENTRALPROBE]` counter. `bead_count` is assigned to
    /// `edge.to` of every central edge (`update_bead_count`), so this separates
    /// "we mark fewer edges central" from "we mark the same edges and assign
    /// `bead_count <= 0` more often".
    pub(crate) fn central_census(&self, stage: &str) {
        if !crate::probe_enabled("CENTRALPROBE") {
            return;
        }
        let mut central_set = 0usize;
        let mut central = 0usize;
        for e in self.graph.edges.iter() {
            if e.base.data.central_is_set() {
                central_set += 1;
                if e.base.data.is_central() {
                    central += 1;
                }
            }
        }
        let mut bc = [0usize; 6];
        for n in self.graph.nodes.iter() {
            let c = n.base.data.bead_count;
            let slot = if c < 0 {
                0
            } else if c > 3 {
                5
            } else {
                (c + 1) as usize
            };
            bc[slot] += 1;
        }
        centralprobe(
            stage,
            self.graph.edges.len(),
            central_set,
            central,
            self.graph.nodes.len(),
            &bc,
        );
    }
}

fn centralprobe(
    stage: &str,
    edges: usize,
    central_set: usize,
    central: usize,
    nodes: usize,
    bc: &[usize; 6],
) {
    use std::collections::BTreeMap;
    use std::sync::Mutex;
    #[derive(Default, Clone, Copy)]
    struct Acc {
        edges: usize,
        central_set: usize,
        central: usize,
        nodes: usize,
        bc: [usize; 6],
    }
    static ACC: Mutex<Option<BTreeMap<String, Acc>>> = Mutex::new(None);
    static ROUNDS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
    let Ok(mut g) = ACC.lock() else { return };
    let m = g.get_or_insert_with(BTreeMap::new);
    let e = m.entry(stage.to_string()).or_default();
    e.edges += edges;
    e.central_set += central_set;
    e.central += central;
    e.nodes += nodes;
    for i in 0..6 {
        e.bc[i] += bc[i];
    }
    if stage.starts_with('4')
        && ROUNDS.fetch_add(1, std::sync::atomic::Ordering::Relaxed) % 4_000 == 3_999
    {
        eprintln!("[CENTRALPROBE] ---- cumulative ----");
        for (k, a) in m.iter() {
            eprintln!(
                "  {k:<28} edges={:9} central={:9} ({:5.1}%) set={:9} | nodes={:9} \
                 bc[-1]={:8} bc[0]={:8} bc[1]={:8} bc[2]={:8} bc[3]={:8} bc[4+]={:8}",
                a.edges,
                a.central,
                100.0 * a.central as f64 / a.edges.max(1) as f64,
                a.central_set,
                a.nodes,
                a.bc[0],
                a.bc[1],
                a.bc[2],
                a.bc[3],
                a.bc[4],
                a.bc[5],
            );
        }
    }
}

/// R548: which branch of `update_is_central` decides each edge, and the two
/// constants the last two branches turn on. Mirrors `[CPP-ISCPROBE]`.
fn iscprobe(branch: usize, central: bool, oefl: Coord, cap: f64) {
    use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
    if !crate::probe_enabled("ISCPROBE") {
        return;
    }
    static N: AtomicUsize = AtomicUsize::new(0);
    static TAKEN: [AtomicUsize; 4] = [
        AtomicUsize::new(0),
        AtomicUsize::new(0),
        AtomicUsize::new(0),
        AtomicUsize::new(0),
    ];
    static YES: [AtomicUsize; 4] = [
        AtomicUsize::new(0),
        AtomicUsize::new(0),
        AtomicUsize::new(0),
        AtomicUsize::new(0),
    ];
    let n = N.fetch_add(1, Relaxed) + 1;
    TAKEN[branch].fetch_add(1, Relaxed);
    if central {
        YES[branch].fetch_add(1, Relaxed);
    }
    if n == 1 || n % 2_000_000 == 0 {
        eprintln!(
            "[ISCPROBE] edges={n} | twin-copy={}(central {}) extra_vd={} short={} \
             geom={}(central {}) | outer_edge_filter_length={:.4}mm cap={:.6}",
            TAKEN[0].load(Relaxed),
            YES[0].load(Relaxed),
            TAKEN[1].load(Relaxed),
            TAKEN[2].load(Relaxed),
            TAKEN[3].load(Relaxed),
            YES[3].load(Relaxed),
            oefl as f64 / 1e5,
            cap,
        );
    }
}

/// R549: self-checking version of R548's removed GEOMPROBE. R548's histogram and
/// its direct comparison disagreed on inputs where that is arithmetically
/// impossible (`dR/dD < cap/2` implies `dR < dD*cap`), so this counts the
/// violation itself and dumps the offending values rather than reporting either
/// number on trust.
fn geomprobe(d_r: Coord, d_d: Coord, cap: f64) {
    use std::sync::Mutex;
    if !crate::probe_enabled("ISCPROBE") {
        return;
    }
    struct G {
        n: usize,
        half: usize,     // ratio < cap/2
        direct: usize,   // d_r < d_d * cap
        violations: usize,
        samples: Vec<(i64, i64, f64, bool)>,
    }
    static ACC: Mutex<Option<G>> = Mutex::new(None);
    let Ok(mut guard) = ACC.lock() else { return };
    let g = guard.get_or_insert(G {
        n: 0,
        half: 0,
        direct: 0,
        violations: 0,
        samples: Vec::new(),
    });
    g.n += 1;
    let ratio = if d_d > 0 { d_r as f64 / d_d as f64 } else { f64::INFINITY };
    let in_half = ratio < cap / 2.0;
    let direct = (d_r as f64) < (d_d as f64) * cap;
    if in_half {
        g.half += 1;
    }
    if direct {
        g.direct += 1;
    }
    // The invariant: in_half implies direct. Any violation is a probe or type bug.
    if in_half && !direct {
        g.violations += 1;
        if g.samples.len() < 5 {
            g.samples.push((d_r as i64, d_d as i64, ratio, direct));
        }
    }
    if g.n == 1 || g.n % 500_000 == 0 {
        eprintln!(
            "[GEOMPROBE2] n={} ratio<cap/2={} direct(dR<dD*cap)={} VIOLATIONS={} cap={:.9}",
            g.n, g.half, g.direct, g.violations, cap
        );
        for (r, d, ratio, direct) in g.samples.iter() {
            eprintln!("   violation sample: d_r={r} d_d={d} ratio={ratio:.6} direct={direct}");
        }
    }
}

impl SkeletalTrapezoidation<'_> {
    /// R551: transition census after each stage of `generate_transitioning_ribs`,
    /// mirroring the C++ `[CPP-TRANSPROBE]` counter. This is the direct analogue
    /// of the failing G-code metric: how often the bead width changes along a wall.
    pub(crate) fn transition_census(&self, stage: &str) {
        if !crate::probe_enabled("TRANSPROBE") {
            return;
        }
        let mut edges_with = 0usize;
        let mut items = 0usize;
        for e in self.graph.edges.iter() {
            if e.base.data.has_transitions(true) {
                edges_with += 1;
                if let Some(t) = e.base.data.get_transitions() {
                    items += t.read().len();
                }
            }
        }
        transprobe(stage, edges_with, items);
    }
}

fn transprobe(stage: &str, edges_with: usize, items: usize) {
    use std::collections::BTreeMap;
    use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
    use std::sync::Mutex;
    #[derive(Default, Clone, Copy)]
    struct Acc {
        calls: usize,
        edges_with: usize,
        items: usize,
    }
    static ACC: Mutex<Option<BTreeMap<String, Acc>>> = Mutex::new(None);
    static ROUNDS: AtomicUsize = AtomicUsize::new(0);
    let Ok(mut g) = ACC.lock() else { return };
    let m = g.get_or_insert_with(BTreeMap::new);
    let a = m.entry(stage.to_string()).or_default();
    a.calls += 1;
    a.edges_with += edges_with;
    a.items += items;
    if stage.starts_with('3') && ROUNDS.fetch_add(1, Relaxed) % 4_000 == 3_999 {
        eprintln!("[TRANSPROBE] ---- cumulative ----");
        for (k, v) in m.iter() {
            eprintln!(
                "  {k:<34} calls={:7} edges_with_transitions={:9} items={:9}",
                v.calls, v.edges_with, v.items
            );
        }
    }
}

/// R551: how often `get_or_create_beading` reuses a NEIGHBOUR's beading object
/// instead of computing a fresh one. A high reuse rate means adjacent nodes
/// SHARE a beading -> uniform width along the wall. Mirrors `[CPP-GNBPROBE]`.
fn gnbprobe(had_beading: bool, bead_count_minus_one: bool, nearest_hit: bool) {
    use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
    if !crate::probe_enabled("GNBPROBE") {
        return;
    }
    static N: AtomicUsize = AtomicUsize::new(0);
    static ALREADY: AtomicUsize = AtomicUsize::new(0);
    static BC_M1: AtomicUsize = AtomicUsize::new(0);
    static HITS: AtomicUsize = AtomicUsize::new(0);
    let n = N.fetch_add(1, Relaxed) + 1;
    if had_beading {
        ALREADY.fetch_add(1, Relaxed);
    }
    if bead_count_minus_one {
        BC_M1.fetch_add(1, Relaxed);
    }
    if nearest_hit {
        HITS.fetch_add(1, Relaxed);
    }
    if n == 1 || n % 200_000 == 0 {
        eprintln!(
            "[GNBPROBE] calls={n} | already_had_beading={} | bead_count==-1={} | \
             getNearestBeading HIT={}",
            ALREADY.load(Relaxed),
            BC_M1.load(Relaxed),
            HITS.load(Relaxed),
        );
    }
}

/// R586 probe (PROPCLASS=1): how a node's beading comes to exist.
///
/// R585 proved a node's beading is NOT a pure function of its thickness -- it is
/// propagated -- and that C++ injects ~2x the width variation per unit of
/// thickness variation. There are exactly four creation sites:
///
///   0 fresh       `get_or_create_beading` -> `beading_strategy.compute()`
///   1 copy_new    `propagate_beadings_downward_edge`, `from` had no beading
///   2 copy_ratio  same, `ratio_of_top >= 1.0`: straight copy of the top beading
///   3 interp      same, else: `interpolate4`
///
/// A COPY is bit-identical to its source and so cannot produce a width change
/// between neighbours; only fresh and interp can. The mix is the whole question.
/// Mirrors `propclass_tick` in scripts/inject-arachne-probes.py.
pub(crate) fn propclass_tick(cls: usize) {
    use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
    static PC: [AtomicUsize; 4] = [
        AtomicUsize::new(0),
        AtomicUsize::new(0),
        AtomicUsize::new(0),
        AtomicUsize::new(0),
    ];
    if !crate::probe_enabled("PROPCLASS") {
        return;
    }
    // Increment the class counter, then take the checkpoint off a SINGLE atomic.
    // Summing four separate loads is racy under rayon and skips the exact
    // multiple, which is why the first attempt printed nothing at all.
    static TOTAL: AtomicUsize = AtomicUsize::new(0);
    PC[cls].fetch_add(1, Relaxed);
    let n = TOTAL.fetch_add(1, Relaxed) + 1;
    if n == 1 || n % 100_000 == 0 {
        let f = PC[0].load(Relaxed);
        let c1 = PC[1].load(Relaxed);
        let c2 = PC[2].load(Relaxed);
        let ip = PC[3].load(Relaxed);
        let tot = n as f64;
        let (z, sm, bg) = interp_delta_counts();
        eprintln!(
            "[PROPCLASS] total={n} fresh={f} ({:.4}) copy_new={c1} ({:.4}) copy_ratio={c2} ({:.4}) interp={ip} ({:.4}) | copies={:.4} | interp_zero={z} interp_small={sm} interp_big={bg}",
            f as f64 / tot, c1 as f64 / tot, c2 as f64 / tot, ip as f64 / tot,
            (c1 + c2) as f64 / tot,
        );
    }
}

use std::sync::atomic::AtomicUsize as PcAtomic;
static PC_INTERP_ZERO: PcAtomic = PcAtomic::new(0);
static PC_INTERP_SMALL: PcAtomic = PcAtomic::new(0);
static PC_INTERP_BIG: PcAtomic = PcAtomic::new(0);

fn interp_delta_counts() -> (usize, usize, usize) {
    use std::sync::atomic::Ordering::Relaxed;
    (
        PC_INTERP_ZERO.load(Relaxed),
        PC_INTERP_SMALL.load(Relaxed),
        PC_INTERP_BIG.load(Relaxed),
    )
}

/// R586: did `interpolate4` actually move `bead_widths[0]`? coord_t is 1e-5 mm, so
/// 100 units is one micron. An interpolation that returns the incoming width is
/// indistinguishable from a copy as far as neighbour disagreement is concerned.
pub(crate) fn propclass_interp_delta(d: i64) {
    use std::sync::atomic::Ordering::Relaxed;
    if d == 0 {
        PC_INTERP_ZERO.fetch_add(1, Relaxed);
    } else if d < 100 {
        PC_INTERP_SMALL.fetch_add(1, Relaxed);
    } else {
        PC_INTERP_BIG.fetch_add(1, Relaxed);
    }
}

/// R587 probe (UPPROBE=1): why does C++ reach the has-beading branch 2.25x more
/// often than we do (R586: 14.25% vs 6.33%)? Two candidates, both per-item rates.
///
/// `upprobe_tick` classifies every `propagate_beadings_upward` iteration by the
/// FIRST guard that skips it, in source order, so it says how many nodes the
/// upward pass actually SEEDS. `dnprobe_tick` classifies every downward-dispatcher
/// iteration into central-skip / equidistant-twin / normal, so it says whether the
/// two engines walk the same set of edges.
///
/// Both take the checkpoint off a single atomic (R586: summing several atomic
/// loads is racy under rayon and silently skips the `% N` boundary).
pub(crate) fn upprobe_tick(s1: bool, s2: bool, s3: bool) {
    use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
    static TOTAL: AtomicUsize = AtomicUsize::new(0);
    static SKIP_BC: AtomicUsize = AtomicUsize::new(0);
    static SKIP_NOFROM: AtomicUsize = AtomicUsize::new(0);
    static SKIP_TOHAS: AtomicUsize = AtomicUsize::new(0);
    static SEED: AtomicUsize = AtomicUsize::new(0);
    let n = TOTAL.fetch_add(1, Relaxed) + 1;
    if s1 {
        SKIP_BC.fetch_add(1, Relaxed);
    } else if s2 {
        SKIP_NOFROM.fetch_add(1, Relaxed);
    } else if s3 {
        SKIP_TOHAS.fetch_add(1, Relaxed);
    } else {
        SEED.fetch_add(1, Relaxed);
    }
    if n == 1 || n % 100_000 == 0 {
        let d = n as f64;
        eprintln!(
            "[UPPROBE] up_total={n} skip_beadcount={} ({:.4}) skip_no_from={} ({:.4}) skip_to_has={} ({:.4}) SEEDED={} ({:.4})",
            SKIP_BC.load(Relaxed), SKIP_BC.load(Relaxed) as f64 / d,
            SKIP_NOFROM.load(Relaxed), SKIP_NOFROM.load(Relaxed) as f64 / d,
            SKIP_TOHAS.load(Relaxed), SKIP_TOHAS.load(Relaxed) as f64 / d,
            SEED.load(Relaxed), SEED.load(Relaxed) as f64 / d,
        );
    }
}

pub(crate) fn dnprobe_tick(central: bool, equi: bool) {
    use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
    static TOTAL: AtomicUsize = AtomicUsize::new(0);
    static CENTRAL: AtomicUsize = AtomicUsize::new(0);
    static TWIN: AtomicUsize = AtomicUsize::new(0);
    static NORMAL: AtomicUsize = AtomicUsize::new(0);
    let n = TOTAL.fetch_add(1, Relaxed) + 1;
    if central {
        CENTRAL.fetch_add(1, Relaxed);
    } else if equi {
        TWIN.fetch_add(1, Relaxed);
    } else {
        NORMAL.fetch_add(1, Relaxed);
    }
    if n == 1 || n % 100_000 == 0 {
        let d = n as f64;
        eprintln!(
            "[DNPROBE] dn_total={n} central_skip={} ({:.4}) twin={} ({:.4}) normal={} ({:.4})",
            CENTRAL.load(Relaxed), CENTRAL.load(Relaxed) as f64 / d,
            TWIN.load(Relaxed), TWIN.load(Relaxed) as f64 / d,
            NORMAL.load(Relaxed), NORMAL.load(Relaxed) as f64 / d,
        );
    }
}

/// R588 probe (CENSUS=1): the two quantities R587 left open, measured directly on
/// the graph at the moment `propagate_beadings_upward` runs.
///
/// R587 showed the upward pass seeds half as many nodes as C++'s, with the entire
/// deficit in the `to->bead_count >= 0` guard, and that our dispatcher sees more
/// central edges (60.10% vs 56.67%). Those were per-ITERATION rates; these are
/// per-NODE and per-EDGE rates over the whole graph, which is the population the
/// guard actually tests. Summed across generate() calls, so order-independent.
/// Mirrors `census_tick` in scripts/inject-arachne-probes.py.
pub(crate) fn census_tick(
    nodes: usize,
    bc: usize,
    hasb: usize,
    edges: usize,
    central: usize,
) {
    use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
    static CALLS: AtomicUsize = AtomicUsize::new(0);
    static NODES: AtomicUsize = AtomicUsize::new(0);
    static BC: AtomicUsize = AtomicUsize::new(0);
    static HASB: AtomicUsize = AtomicUsize::new(0);
    static EDGES: AtomicUsize = AtomicUsize::new(0);
    static CENTRAL: AtomicUsize = AtomicUsize::new(0);
    NODES.fetch_add(nodes, Relaxed);
    BC.fetch_add(bc, Relaxed);
    HASB.fetch_add(hasb, Relaxed);
    EDGES.fetch_add(edges, Relaxed);
    CENTRAL.fetch_add(central, Relaxed);
    let n = CALLS.fetch_add(1, Relaxed) + 1;
    if n == 1 || n % 2000 == 0 {
        let nn = NODES.load(Relaxed).max(1) as f64;
        let ne = EDGES.load(Relaxed).max(1) as f64;
        eprintln!(
            "[CENSUS] calls={n} nodes={} bead_count>=0={} ({:.4}) hasBeading={} ({:.4}) | edges={} central={} ({:.4})",
            NODES.load(Relaxed), BC.load(Relaxed), BC.load(Relaxed) as f64 / nn,
            HASB.load(Relaxed), HASB.load(Relaxed) as f64 / nn,
            EDGES.load(Relaxed), CENTRAL.load(Relaxed), CENTRAL.load(Relaxed) as f64 / ne,
        );
    }
}

/// R589 probe (GBUILD=1): where does the 1.25x skeletal-graph density come from?
///
/// R588 found C++ builds 1.254x the nodes and 1.256x the edges per `generate()`
/// call while every per-item rate inside the graph matches -- there is simply more
/// graph. This brackets the build: the Voronoi INPUT (one segment per polygon
/// point), the raw Voronoi OUTPUT before any filtering, and how many points
/// `discretize` emits per Voronoi edge. Input already 1.25x => upstream outline
/// discretisation. Input matched but output 1.25x => Voronoi construction.
/// Both matched => the discretise/filter chain.
/// Mirrors `gbuild_tick`/`gbuild_disc` in scripts/inject-arachne-probes.py.
pub(crate) fn gbuild_disc(n: usize) {
    use std::sync::atomic::Ordering::Relaxed;
    GB_DISC_CALLS.fetch_add(1, Relaxed);
    GB_DISC_PTS.fetch_add(n, Relaxed);
}

static GB_DISC_CALLS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
static GB_DISC_PTS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

pub(crate) fn gbuild_tick(polys: usize, segs: usize, vv: usize, ve: usize, vc: usize) {
    use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
    static CALLS: AtomicUsize = AtomicUsize::new(0);
    static POLYS: AtomicUsize = AtomicUsize::new(0);
    static SEGS: AtomicUsize = AtomicUsize::new(0);
    static VV: AtomicUsize = AtomicUsize::new(0);
    static VE: AtomicUsize = AtomicUsize::new(0);
    static VC: AtomicUsize = AtomicUsize::new(0);
    POLYS.fetch_add(polys, Relaxed);
    SEGS.fetch_add(segs, Relaxed);
    VV.fetch_add(vv, Relaxed);
    VE.fetch_add(ve, Relaxed);
    VC.fetch_add(vc, Relaxed);
    let n = CALLS.fetch_add(1, Relaxed) + 1;
    if n == 1 || n % 2000 == 0 {
        let d = n as f64;
        let dc = GB_DISC_CALLS.load(Relaxed);
        eprintln!(
            "[GBUILD] calls={n} polys/call={:.3} segs/call={:.3} | vd_verts/call={:.3} vd_edges/call={:.3} vd_cells/call={:.3} | disc_calls={dc} disc_pts/call={:.3} pts_per_disc={:.4}",
            POLYS.load(Relaxed) as f64 / d,
            SEGS.load(Relaxed) as f64 / d,
            VV.load(Relaxed) as f64 / d,
            VE.load(Relaxed) as f64 / d,
            VC.load(Relaxed) as f64 / d,
            GB_DISC_PTS.load(Relaxed) as f64 / d,
            GB_DISC_PTS.load(Relaxed) as f64 / dc.max(1) as f64,
        );
    }
}

/// R590 probe (CONV=1): does the 1.25x graph-density gap come from CREATING fewer
/// half-edges or from REMOVING more?
///
/// R589 localised the gap to the Voronoi->half-edge conversion (graph edges per
/// Voronoi edge: 0.9333 for us against 1.0878 for C++). Creation happens in
/// `transfer_edge` (from discretized points) plus `make_rib` (two EXTRA_VD edges
/// per call); removal happens in `collapse_small_edges`. Edge counts at three
/// points -- after the cell loop, after `separate_pointy_quad_end_nodes`, after
/// `collapse_small_edges` -- separate the two. Also counts cells seen vs skipped
/// for want of an incident edge. Mirrors `conv_cell`/`conv_stage` in the injector.
pub(crate) fn conv_cell(skipped: bool) {
    use std::sync::atomic::Ordering::Relaxed;
    CV_CELLS.fetch_add(1, Relaxed);
    if skipped {
        CV_CELLS_SKIPPED.fetch_add(1, Relaxed);
    }
}

static CV_CELLS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
static CV_CELLS_SKIPPED: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

pub(crate) fn conv_stage(stage: usize, edges: usize, nodes: usize) {
    use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
    static CALLS: AtomicUsize = AtomicUsize::new(0);
    static E0: AtomicUsize = AtomicUsize::new(0);
    static E1: AtomicUsize = AtomicUsize::new(0);
    static E2: AtomicUsize = AtomicUsize::new(0);
    static N0: AtomicUsize = AtomicUsize::new(0);
    static N2: AtomicUsize = AtomicUsize::new(0);
    match stage {
        0 => {
            E0.fetch_add(edges, Relaxed);
            N0.fetch_add(nodes, Relaxed);
        }
        1 => {
            E1.fetch_add(edges, Relaxed);
        }
        _ => {
            E2.fetch_add(edges, Relaxed);
            N2.fetch_add(nodes, Relaxed);
            let n = CALLS.fetch_add(1, Relaxed) + 1;
            if n == 1 || n % 2000 == 0 {
                let d = n as f64;
                eprintln!(
                    "[CONV] calls={n} cells/call={:.3} skipped/call={:.3} | e_after_cells/call={:.3} e_after_separate/call={:.3} e_after_collapse/call={:.3} | n_after_cells/call={:.3} n_after_collapse/call={:.3} | collapse_keep={:.4}",
                    CV_CELLS.load(Relaxed) as f64 / d,
                    CV_CELLS_SKIPPED.load(Relaxed) as f64 / d,
                    E0.load(Relaxed) as f64 / d,
                    E1.load(Relaxed) as f64 / d,
                    E2.load(Relaxed) as f64 / d,
                    N0.load(Relaxed) as f64 / d,
                    N2.load(Relaxed) as f64 / d,
                    E2.load(Relaxed) as f64 / E1.load(Relaxed).max(1) as f64,
                );
            }
        }
    }
}
