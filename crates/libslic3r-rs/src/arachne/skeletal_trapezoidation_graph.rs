//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.
//
//! Faithful 1:1 port of Arachne/SkeletalTrapezoidationGraph.{hpp,cpp}
//!
//! C++ Reference:
//! - Arachne/SkeletalTrapezoidationGraph.hpp
//! - Arachne/SkeletalTrapezoidationGraph.cpp
//!
//! Pointer model: the C++ uses CRTP (`edge_t = STHalfEdge`, `node_t = STHalfEdgeNode`)
//! and stores them in `std::list`, with raw `edge_t*`/`node_t*` cross-references.
//! Here the concrete `STHalfEdge`/`STHalfEdgeNode` are stored directly in
//! `std::collections::LinkedList` (pointer-stable like `std::list`), and the
//! cross-reference pointers live in the embedded `HalfEdge`/`HalfEdgeNode` `base`
//! (typed `HalfEdge<EdgeData, NodeData>` / `HalfEdgeNode<EdgeData, NodeData>`).
//! Both wrappers are `#[repr(C)]` with `base` as the sole field, so the address of
//! a wrapper equals the address of its `base` and `*STHalfEdge <-> *HalfEdge` casts
//! are valid — this is the Rust analogue of the C++ CRTP upcast.

use crate::arachne::skeletal_trapezoidation_edge::{EdgeType, SkeletalTrapezoidationEdge};
use crate::arachne::skeletal_trapezoidation_joint::SkeletalTrapezoidationJoint;
use crate::arachne::utils::half_edge::HalfEdge;
use crate::arachne::utils::half_edge_graph::HalfEdgeGraph;
use crate::arachne::utils::half_edge_node::HalfEdgeNode;
use crate::geometry::{shorter_then, Line, Point};
use crate::Coord;
use std::ptr::NonNull;

// SkeletalTrapezoidationGraph.hpp:18 STHalfEdge : public HalfEdge<...>
pub type EdgeData = SkeletalTrapezoidationEdge;
pub type NodeData = SkeletalTrapezoidationJoint;

/// Raw `HalfEdge` pointer (the C++ `edge_t*`).
pub type EdgePtr = NonNull<HalfEdge<EdgeData, NodeData>>;
/// Raw `HalfEdgeNode` pointer (the C++ `node_t*`).
pub type NodePtr = NonNull<HalfEdgeNode<EdgeData, NodeData>>;

// SkeletalTrapezoidationGraph.hpp:18-47
// class STHalfEdge : public HalfEdge<SkeletalTrapezoidationJoint, SkeletalTrapezoidationEdge, STHalfEdgeNode, STHalfEdge>
//
// `#[repr(C)]` with the single `base` field guarantees `&STHalfEdge as *_ == &self.base`.
#[repr(C)]
#[derive(Debug)]
pub struct STHalfEdge {
    pub base: HalfEdge<EdgeData, NodeData>,
}

// SkeletalTrapezoidationGraph.hpp:49-66
// class STHalfEdgeNode : public HalfEdgeNode<SkeletalTrapezoidationJoint, SkeletalTrapezoidationEdge, STHalfEdgeNode, STHalfEdge>
#[repr(C)]
#[derive(Debug)]
pub struct STHalfEdgeNode {
    pub base: HalfEdgeNode<EdgeData, NodeData>,
}

/// Reinterpret a generic `*HalfEdge` (the C++ `edge_t*`) as `*STHalfEdge`.
/// Mirrors the C++ CRTP identity `edge_t* == STHalfEdge*`.
///
/// # Safety
/// The pointer must originate from an `STHalfEdge` stored in a graph list.
#[inline]
pub unsafe fn as_st_edge<'a>(p: EdgePtr) -> &'a STHalfEdge {
    &*(p.as_ptr() as *const STHalfEdge)
}

/// Reinterpret a generic `*HalfEdgeNode` (the C++ `node_t*`) as `*STHalfEdgeNode`.
/// Mirrors the C++ CRTP identity `node_t* == STHalfEdgeNode*`.
///
/// # Safety
/// The pointer must originate from an `STHalfEdgeNode` stored in a graph list.
#[inline]
pub unsafe fn as_st_node<'a>(p: NodePtr) -> &'a STHalfEdgeNode {
    &*(p.as_ptr() as *const STHalfEdgeNode)
}

impl STHalfEdge {
    // SkeletalTrapezoidationGraph.cpp:15
    // STHalfEdge::STHalfEdge(SkeletalTrapezoidationEdge data) : HalfEdge(data) {}
    pub fn new(data: EdgeData) -> Self {
        Self {
            base: HalfEdge::new(data),
        }
    }

    /// `&STHalfEdge` -> `*HalfEdge` (C++ `this` upcast).
    #[inline]
    pub fn as_edge_ptr(&self) -> EdgePtr {
        // repr(C) guarantees base is at offset 0.
        NonNull::new(&self.base as *const _ as *mut _).unwrap()
    }

    // SkeletalTrapezoidationGraph.cpp:17-39
    // bool STHalfEdge::canGoUp(bool strict) const
    pub fn can_go_up(&self, strict: bool) -> bool {
        unsafe {
            let to = self.base.to.unwrap().as_ref();
            let from = self.base.from.unwrap().as_ref();
            // SkeletalTrapezoidationGraph.cpp:19
            if to.data.distance_to_boundary > from.data.distance_to_boundary {
                // SkeletalTrapezoidationGraph.cpp:21
                return true;
            }
            // SkeletalTrapezoidationGraph.cpp:23
            if to.data.distance_to_boundary < from.data.distance_to_boundary || strict {
                // SkeletalTrapezoidationGraph.cpp:25
                return false;
            }

            // SkeletalTrapezoidationGraph.cpp:28 Edge is between equidistqant verts; recurse!
            // SkeletalTrapezoidationGraph.cpp:29 for (edge_t* outgoing = next; outgoing != twin; outgoing = outgoing->twin->next)
            let twin = self.base.twin;
            let mut outgoing = self.base.next;
            while outgoing.is_some() && outgoing != twin {
                let outgoing_p = outgoing.unwrap();
                let outgoing_ref = outgoing_p.as_ref();
                // SkeletalTrapezoidationGraph.cpp:31
                if as_st_edge(outgoing_p).can_go_up(false) {
                    // SkeletalTrapezoidationGraph.cpp:33
                    return true;
                }
                // SkeletalTrapezoidationGraph.cpp:35 assert(outgoing->twin); if (!outgoing->twin) return false;
                if outgoing_ref.twin.is_none() {
                    return false;
                }
                let outgoing_twin = outgoing_ref.twin.unwrap().as_ref();
                // SkeletalTrapezoidationGraph.cpp:36 assert(outgoing->twin->next); if (!outgoing->twin->next) return true;
                if outgoing_twin.next.is_none() {
                    return true; // This point is on the boundary?! Should never occur
                }
                outgoing = outgoing_twin.next;
            }
            // SkeletalTrapezoidationGraph.cpp:38
            false
        }
    }

    // SkeletalTrapezoidationGraph.cpp:41-70
    // bool STHalfEdge::isUpward() const
    pub fn is_upward(&self) -> bool {
        unsafe {
            let to = self.base.to.unwrap().as_ref();
            let from = self.base.from.unwrap().as_ref();
            // SkeletalTrapezoidationGraph.cpp:43
            if to.data.distance_to_boundary > from.data.distance_to_boundary {
                // SkeletalTrapezoidationGraph.cpp:45
                return true;
            }
            // SkeletalTrapezoidationGraph.cpp:47
            if to.data.distance_to_boundary < from.data.distance_to_boundary {
                // SkeletalTrapezoidationGraph.cpp:49
                return false;
            }

            // SkeletalTrapezoidationGraph.cpp:52 Equidistant edge case:
            // SkeletalTrapezoidationGraph.cpp:53
            let forward_up_dist: Option<Coord> = self.dist_to_go_up();
            // SkeletalTrapezoidationGraph.cpp:54
            let backward_up_dist: Option<Coord> = as_st_edge(self.base.twin.unwrap()).dist_to_go_up();
            // SkeletalTrapezoidationGraph.cpp:55
            if forward_up_dist.is_some() && backward_up_dist.is_some() {
                // SkeletalTrapezoidationGraph.cpp:57
                return forward_up_dist < backward_up_dist;
            }

            // SkeletalTrapezoidationGraph.cpp:60
            if forward_up_dist.is_some() {
                // SkeletalTrapezoidationGraph.cpp:62
                return true;
            }

            // SkeletalTrapezoidationGraph.cpp:65
            if backward_up_dist.is_some() {
                // SkeletalTrapezoidationGraph.cpp:67
                return false;
            }
            // SkeletalTrapezoidationGraph.cpp:69 Arbitrary ordering, which returns the opposite for the twin edge
            to.p < from.p
        }
    }

    // SkeletalTrapezoidationGraph.cpp:72-107
    // std::optional<coord_t> STHalfEdge::distToGoUp() const
    pub fn dist_to_go_up(&self) -> Option<Coord> {
        unsafe {
            let to = self.base.to.unwrap().as_ref();
            let from = self.base.from.unwrap().as_ref();
            // SkeletalTrapezoidationGraph.cpp:74
            if to.data.distance_to_boundary > from.data.distance_to_boundary {
                // SkeletalTrapezoidationGraph.cpp:76
                return Some(0);
            }
            // SkeletalTrapezoidationGraph.cpp:78
            if to.data.distance_to_boundary < from.data.distance_to_boundary {
                // SkeletalTrapezoidationGraph.cpp:80
                return None;
            }

            // SkeletalTrapezoidationGraph.cpp:83 Edge is between equidistqant verts; recurse!
            // SkeletalTrapezoidationGraph.cpp:84 std::optional<coord_t> ret;
            let mut ret: Option<Coord> = None;
            // SkeletalTrapezoidationGraph.cpp:85 for (edge_t* outgoing = next; outgoing != twin; outgoing = outgoing->twin->next)
            let twin = self.base.twin;
            let mut outgoing = self.base.next;
            while outgoing.is_some() && outgoing != twin {
                let outgoing_p = outgoing.unwrap();
                let outgoing_ref = outgoing_p.as_ref();
                // SkeletalTrapezoidationGraph.cpp:87
                let dist_to_up = as_st_edge(outgoing_p).dist_to_go_up();
                // SkeletalTrapezoidationGraph.cpp:88
                if let Some(dist_to_up) = dist_to_up {
                    // SkeletalTrapezoidationGraph.cpp:90
                    if let Some(r) = ret {
                        // SkeletalTrapezoidationGraph.cpp:92
                        ret = Some(std::cmp::min(r, dist_to_up));
                    } else {
                        // SkeletalTrapezoidationGraph.cpp:96
                        ret = Some(dist_to_up);
                    }
                }
                // SkeletalTrapezoidationGraph.cpp:99 assert(outgoing->twin); if (!outgoing->twin) return std::optional<coord_t>();
                if outgoing_ref.twin.is_none() {
                    return None;
                }
                let outgoing_twin = outgoing_ref.twin.unwrap().as_ref();
                // SkeletalTrapezoidationGraph.cpp:100 assert(outgoing->twin->next); if (!outgoing->twin->next) return 0;
                if outgoing_twin.next.is_none() {
                    return Some(0); // This point is on the boundary?! Should never occur
                }
                outgoing = outgoing_twin.next;
            }
            // SkeletalTrapezoidationGraph.cpp:102
            if let Some(r) = ret {
                // SkeletalTrapezoidationGraph.cpp:104 ret = *ret + (to->p - from->p).cast<int64_t>().norm();
                ret = Some(r + (to.p - from.p).length() as Coord);
            }
            // SkeletalTrapezoidationGraph.cpp:106
            ret
        }
    }

    // SkeletalTrapezoidationGraph.cpp:109-121
    // STHalfEdge* STHalfEdge::getNextUnconnected()
    pub fn get_next_unconnected(&self) -> Option<EdgePtr> {
        unsafe {
            // SkeletalTrapezoidationGraph.cpp:111 edge_t* result = static_cast<STHalfEdge*>(this);
            let this = self.as_edge_ptr();
            let mut result = this;
            // SkeletalTrapezoidationGraph.cpp:112 while (result->next)
            while let Some(next) = result.as_ref().next {
                // SkeletalTrapezoidationGraph.cpp:114 result = result->next;
                result = next;
                // SkeletalTrapezoidationGraph.cpp:115 if (result == this)
                if result == this {
                    // SkeletalTrapezoidationGraph.cpp:117
                    return None;
                }
            }
            // SkeletalTrapezoidationGraph.cpp:120 return result->twin;
            result.as_ref().twin
        }
    }
}

impl STHalfEdgeNode {
    // SkeletalTrapezoidationGraph.cpp:123
    // STHalfEdgeNode::STHalfEdgeNode(SkeletalTrapezoidationJoint data, Point p) : HalfEdgeNode(data, p) {}
    pub fn new(data: NodeData, p: Point) -> Self {
        Self {
            base: HalfEdgeNode::new(data, p),
        }
    }

    /// `&STHalfEdgeNode` -> `*HalfEdgeNode`.
    #[inline]
    pub fn as_node_ptr(&self) -> NodePtr {
        NonNull::new(&self.base as *const _ as *mut _).unwrap()
    }

    /// Mirror of the C++ implicit copy constructor `STHalfEdgeNode(*other)`:
    /// copies `data` (Clone), `p` (Copy) and the raw `incident_edge` pointer.
    /// (used by `separatePointyQuadEndNodes` — `graph.nodes.emplace_back(*quad_start->from)`.)
    pub fn clone_node(&self) -> Self {
        Self {
            base: HalfEdgeNode {
                data: self.base.data.clone(),
                p: self.base.p,
                incident_edge: self.base.incident_edge,
            },
        }
    }

    // SkeletalTrapezoidationGraph.cpp:125-141
    // bool STHalfEdgeNode::isMultiIntersection()
    pub fn is_multi_intersection(&self) -> bool {
        unsafe {
            // SkeletalTrapezoidationGraph.cpp:127 int odd_path_count = 0;
            let mut odd_path_count = 0;
            // SkeletalTrapezoidationGraph.cpp:128 edge_t* outgoing = this->incident_edge;
            let incident_edge = self.base.incident_edge;
            let mut outgoing = incident_edge;
            // SkeletalTrapezoidationGraph.cpp:129 do { ... } while (outgoing = outgoing->twin->next, outgoing != this->incident_edge);
            loop {
                // SkeletalTrapezoidationGraph.cpp:131 if ( ! outgoing) { return false; }
                if outgoing.is_none() {
                    // SkeletalTrapezoidationGraph.cpp:132 This is a node on the outside
                    return false;
                }
                let outgoing_ref = outgoing.unwrap().as_ref();
                // SkeletalTrapezoidationGraph.cpp:135 if (outgoing->data.isCentral())
                if outgoing_ref.data.is_central() {
                    // SkeletalTrapezoidationGraph.cpp:137 odd_path_count++;
                    odd_path_count += 1;
                }
                // SkeletalTrapezoidationGraph.cpp:139 outgoing = outgoing->twin->next, outgoing != this->incident_edge
                outgoing = outgoing_ref.twin.unwrap().as_ref().next;
                if outgoing == incident_edge {
                    break;
                }
            }
            // SkeletalTrapezoidationGraph.cpp:140 return odd_path_count > 2;
            odd_path_count > 2
        }
    }

    // SkeletalTrapezoidationGraph.cpp:143-155
    // bool STHalfEdgeNode::isCentral() const
    pub fn is_central(&self) -> bool {
        unsafe {
            // SkeletalTrapezoidationGraph.cpp:145 edge_t* edge = incident_edge;
            let incident_edge = self.base.incident_edge;
            let mut edge = incident_edge;
            // SkeletalTrapezoidationGraph.cpp:146 do { ... } while (edge = edge->twin->next, edge != incident_edge);
            loop {
                let edge_ref = edge.unwrap().as_ref();
                // SkeletalTrapezoidationGraph.cpp:148 if (edge->data.isCentral())
                if edge_ref.data.is_central() {
                    // SkeletalTrapezoidationGraph.cpp:150
                    return true;
                }
                // SkeletalTrapezoidationGraph.cpp:152 assert(edge->twin); if (!edge->twin) return false;
                if edge_ref.twin.is_none() {
                    return false;
                }
                // SkeletalTrapezoidationGraph.cpp:153 edge = edge->twin->next, edge != incident_edge
                edge = edge_ref.twin.unwrap().as_ref().next;
                if edge == incident_edge {
                    break;
                }
            }
            // SkeletalTrapezoidationGraph.cpp:154
            false
        }
    }

    // SkeletalTrapezoidationGraph.cpp:157-179
    // bool STHalfEdgeNode::isLocalMaximum(bool strict) const
    pub fn is_local_maximum(&self, strict: bool) -> bool {
        unsafe {
            // SkeletalTrapezoidationGraph.cpp:159 if (data.distance_to_boundary == 0)
            if self.base.data.distance_to_boundary == 0 {
                // SkeletalTrapezoidationGraph.cpp:161
                return false;
            }

            // SkeletalTrapezoidationGraph.cpp:164 edge_t* edge = incident_edge;
            let incident_edge = self.base.incident_edge;
            let mut edge = incident_edge;
            // SkeletalTrapezoidationGraph.cpp:165 do { ... } while (edge = edge->twin->next, edge != incident_edge);
            loop {
                let edge_p = edge.unwrap();
                let edge_ref = edge_p.as_ref();
                // SkeletalTrapezoidationGraph.cpp:167 if (edge->canGoUp(strict))
                if as_st_edge(edge_p).can_go_up(strict) {
                    // SkeletalTrapezoidationGraph.cpp:169
                    return false;
                }
                // SkeletalTrapezoidationGraph.cpp:171 assert(edge->twin); if (!edge->twin) return false;
                if edge_ref.twin.is_none() {
                    return false;
                }
                let edge_twin = edge_ref.twin.unwrap().as_ref();
                // SkeletalTrapezoidationGraph.cpp:173 if (!edge->twin->next)
                if edge_twin.next.is_none() {
                    // SkeletalTrapezoidationGraph.cpp:175 This point is on the boundary
                    return false;
                }
                // SkeletalTrapezoidationGraph.cpp:177 edge = edge->twin->next, edge != incident_edge
                edge = edge_twin.next;
                if edge == incident_edge {
                    break;
                }
            }
            // SkeletalTrapezoidationGraph.cpp:178
            true
        }
    }
}

// SkeletalTrapezoidationGraph.hpp:68-102
// class SkeletalTrapezoidationGraph: public HalfEdgeGraph<...>
//
// The base `HalfEdgeGraph<EdgeData, NodeData>` provides the `std::list`-equivalent
// `LinkedList` members `edges` / `nodes` (pointer-stable). We store the concrete
// `STHalfEdge` / `STHalfEdgeNode` via separate, dedicated lists here so the stored
// elements are the *derived* CRTP types (matching C++ `std::list<edge_t>`).
#[derive(Debug)]
pub struct SkeletalTrapezoidationGraph {
    // HalfEdgeGraph.hpp:24 std::list<edge_t> edges;  (derived type: STHalfEdge)
    //
    // The elements are `Box`ed so a survivor's payload address is STABLE across a
    // list rebuild: the C++ `std::list<edge_t>` keeps every node at a fixed address
    // and `std::list::erase` removes one node without touching the others, which the
    // raw `edge_t*`/`node_t*` cross-references depend on. `std::collections::LinkedList`
    // has no stable middle-removal (the `cursor_mut` API is unstable), so removal in
    // `collapse_small_edges` rebuilds the list — which would MOVE plain `STHalfEdge`
    // values to new addresses and dangle every pointer. Boxing the element means the
    // rebuild only moves the `Box` (a pointer); the heap `STHalfEdge`/`STHalfEdgeNode`
    // payload (and thus `&payload.base`, the `edge_t*`/`node_t*`) stays put.
    pub edges: std::collections::LinkedList<Box<STHalfEdge>>,
    // HalfEdgeGraph.hpp:25 std::list<node_t> nodes;  (derived type: STHalfEdgeNode)
    pub nodes: std::collections::LinkedList<Box<STHalfEdgeNode>>,
    /// Carries the (currently unused) generic base graph members for completeness.
    _base: HalfEdgeGraph<EdgeData, NodeData>,
}

impl SkeletalTrapezoidationGraph {
    pub fn new() -> Self {
        Self {
            edges: std::collections::LinkedList::new(),
            nodes: std::collections::LinkedList::new(),
            _base: HalfEdgeGraph::new(),
        }
    }

    /// `&mut STHalfEdge` -> `*HalfEdge` (used to populate base pointer fields).
    #[inline]
    pub fn edge_ptr(edge: &STHalfEdge) -> EdgePtr {
        NonNull::new(&edge.base as *const _ as *mut _).unwrap()
    }

    /// `&STHalfEdgeNode` -> `*HalfEdgeNode`.
    #[inline]
    pub fn node_ptr(node: &STHalfEdgeNode) -> NodePtr {
        NonNull::new(&node.base as *const _ as *mut _).unwrap()
    }

    // SkeletalTrapezoidationGraph.cpp:181-314
    // void SkeletalTrapezoidationGraph::collapseSmallEdges(coord_t snap_dist)
    pub fn collapse_small_edges(&mut self, snap_dist: Coord) {
        unsafe {
            // SkeletalTrapezoidationGraph.cpp:183-184
            //   std::unordered_map<edge_t*, std::list<edge_t>::iterator> edge_locator;
            //   std::unordered_map<node_t*, std::list<node_t>::iterator> node_locator;
            //
            // The C++ locators map a pointer back to its `std::list` iterator so an
            // arbitrary element can be erased in O(1). With a `LinkedList` the only
            // stable way to remove a specific element by pointer identity is to splice
            // the list around it; we collect the set of pointers to be removed and
            // rebuild the list at the end, preserving order. (`std::list::erase` does
            // not change the relative order of surviving elements either.)
            //
            // SkeletalTrapezoidationGraph.cpp:196-208
            //   safelyRemoveEdge(...) — erase an edge, advancing the loop iterator when
            //   it is the one being erased.
            let mut edges_to_remove: std::collections::HashSet<*const HalfEdge<EdgeData, NodeData>> =
                std::collections::HashSet::new();
            let mut nodes_to_remove: std::collections::HashSet<
                *const HalfEdgeNode<EdgeData, NodeData>,
            > = std::collections::HashSet::new();

            // SkeletalTrapezoidationGraph.cpp:210-213
            // auto should_collapse = [snap_dist](node_t* a, node_t* b)
            //     { return shorter_then(a->p - b->p, snap_dist); };
            let should_collapse = |a: NodePtr, b: NodePtr| -> bool {
                shorter_then(&(a.as_ref().p - b.as_ref().p), snap_dist)
            };

            // SkeletalTrapezoidationGraph.cpp:215 for (auto edge_it = edges.begin(); edge_it != edges.end();)
            for edge_it in self.edges.iter() {
                let quad_start_p = Self::edge_ptr(edge_it);
                // In C++ an edge removed via safelyRemoveEdge is erased from the list and
                // is therefore never visited again. With deferred removal the element is
                // still physically present during this pass, so skip it explicitly to
                // mirror the C++ iteration (a removed quad_start would otherwise be
                // re-processed since its `prev` may be null).
                if edges_to_remove.contains(&(&edge_it.base as *const _)) {
                    continue;
                }
                // SkeletalTrapezoidationGraph.cpp:217 if (edge_it->prev)
                if edge_it.base.prev.is_some() {
                    // SkeletalTrapezoidationGraph.cpp:219 edge_it++; continue;
                    continue;
                }

                // SkeletalTrapezoidationGraph.cpp:223 edge_t* quad_start = &*edge_it;
                let quad_start = quad_start_p;
                // SkeletalTrapezoidationGraph.cpp:224 edge_t* quad_end = quad_start; while (quad_end->next) quad_end = quad_end->next;
                let mut quad_end = quad_start;
                while let Some(next) = quad_end.as_ref().next {
                    quad_end = next;
                }
                // SkeletalTrapezoidationGraph.cpp:225 edge_t* quad_mid = (quad_start->next == quad_end)? nullptr : quad_start->next;
                let quad_mid: Option<EdgePtr> = if quad_start.as_ref().next == Some(quad_end) {
                    None
                } else {
                    quad_start.as_ref().next
                };

                // SkeletalTrapezoidationGraph.cpp:228 if (quad_mid && should_collapse(quad_mid->from, quad_mid->to))
                if let Some(quad_mid) = quad_mid {
                    let quad_mid_ref = quad_mid.as_ref();
                    if should_collapse(quad_mid_ref.from.unwrap(), quad_mid_ref.to.unwrap()) {
                        // SkeletalTrapezoidationGraph.cpp:230-235 assert(quad_mid->twin); ... continue;
                        if quad_mid_ref.twin.is_none() {
                            log::warn!("Encountered quad edge without a twin.");
                            continue; //Prevent accessing unallocated memory.
                        }
                        let quad_mid_twin = quad_mid_ref.twin.unwrap();
                        let quad_mid_from = quad_mid_ref.from.unwrap();
                        // SkeletalTrapezoidationGraph.cpp:236 int count = 0;
                        let mut count = 0;
                        // SkeletalTrapezoidationGraph.cpp:237 for (edge_t* edge_from_3 = quad_end; edge_from_3 && edge_from_3 != quad_mid->twin; edge_from_3 = edge_from_3->twin->next)
                        let mut edge_from_3: Option<EdgePtr> = Some(quad_end);
                        while edge_from_3.is_some() && edge_from_3 != Some(quad_mid_twin) {
                            let mut ef3 = edge_from_3.unwrap();
                            // SkeletalTrapezoidationGraph.cpp:239 edge_from_3->from = quad_mid->from;
                            ef3.as_mut().from = Some(quad_mid_from);
                            // SkeletalTrapezoidationGraph.cpp:240 edge_from_3->twin->to = quad_mid->from;
                            ef3.as_ref().twin.unwrap().as_ptr().as_mut().unwrap().to =
                                Some(quad_mid_from);
                            // SkeletalTrapezoidationGraph.cpp:241-244 if (count > 50) { std::cerr << ... }
                            if count > 50 {
                                let from_p = ef3.as_ref().from.unwrap().as_ref().p;
                                let to_p = ef3.as_ref().to.unwrap().as_ref().p;
                                eprintln!("{:?} - {:?}", from_p, to_p);
                            }
                            // SkeletalTrapezoidationGraph.cpp:245 if (++count > 1000) break;
                            count += 1;
                            if count > 1000 {
                                break;
                            }
                            edge_from_3 = ef3.as_ref().twin.unwrap().as_ref().next;
                        }

                        // SkeletalTrapezoidationGraph.cpp:251-255 collapse top comment
                        // SkeletalTrapezoidationGraph.cpp:256 if (quad_mid->from->incident_edge == quad_mid)
                        if quad_mid_from.as_ref().incident_edge == Some(quad_mid) {
                            // SkeletalTrapezoidationGraph.cpp:258 if (quad_mid->twin->next)
                            if let Some(qm_twin_next) = quad_mid_twin.as_ref().next {
                                // SkeletalTrapezoidationGraph.cpp:260 quad_mid->from->incident_edge = quad_mid->twin->next;
                                quad_mid_from.as_ptr().as_mut().unwrap().incident_edge =
                                    Some(qm_twin_next);
                            } else {
                                // SkeletalTrapezoidationGraph.cpp:264 quad_mid->from->incident_edge = quad_mid->prev->twin;
                                quad_mid_from.as_ptr().as_mut().unwrap().incident_edge =
                                    quad_mid_ref.prev.unwrap().as_ref().twin;
                            }
                        }

                        // SkeletalTrapezoidationGraph.cpp:268 nodes.erase(node_locator[quad_mid->to]);
                        nodes_to_remove.insert(quad_mid_ref.to.unwrap().as_ptr() as *const _);

                        // SkeletalTrapezoidationGraph.cpp:270 quad_mid->prev->next = quad_mid->next;
                        quad_mid_ref.prev.unwrap().as_ptr().as_mut().unwrap().next =
                            quad_mid_ref.next;
                        // SkeletalTrapezoidationGraph.cpp:271 quad_mid->next->prev = quad_mid->prev;
                        quad_mid_ref.next.unwrap().as_ptr().as_mut().unwrap().prev =
                            quad_mid_ref.prev;
                        // SkeletalTrapezoidationGraph.cpp:272 quad_mid->twin->next->prev = quad_mid->twin->prev;
                        quad_mid_twin
                            .as_ref()
                            .next
                            .unwrap()
                            .as_ptr()
                            .as_mut()
                            .unwrap()
                            .prev = quad_mid_twin.as_ref().prev;
                        // SkeletalTrapezoidationGraph.cpp:273 quad_mid->twin->prev->next = quad_mid->twin->next;
                        quad_mid_twin
                            .as_ref()
                            .prev
                            .unwrap()
                            .as_ptr()
                            .as_mut()
                            .unwrap()
                            .next = quad_mid_twin.as_ref().next;

                        // SkeletalTrapezoidationGraph.cpp:275 safelyRemoveEdge(quad_mid->twin, ...);
                        edges_to_remove.insert(quad_mid_twin.as_ptr() as *const _);
                        // SkeletalTrapezoidationGraph.cpp:276 safelyRemoveEdge(quad_mid, ...);
                        edges_to_remove.insert(quad_mid.as_ptr() as *const _);
                    }
                }

                // SkeletalTrapezoidationGraph.cpp:279-282 collapse sides comment +
                // if (should_collapse(quad_start->from, quad_end->to) && should_collapse(quad_start->to, quad_end->from))
                if should_collapse(
                    quad_start.as_ref().from.unwrap(),
                    quad_end.as_ref().to.unwrap(),
                ) && should_collapse(
                    quad_start.as_ref().to.unwrap(),
                    quad_end.as_ref().from.unwrap(),
                ) {
                    // SkeletalTrapezoidationGraph.cpp:283 Collapse start and end edges and remove whole cell
                    let quad_start_twin = quad_start.as_ref().twin.unwrap();
                    let quad_end_twin = quad_end.as_ref().twin.unwrap();
                    let quad_end_to = quad_end.as_ref().to.unwrap();
                    let quad_end_from = quad_end.as_ref().from.unwrap();

                    // SkeletalTrapezoidationGraph.cpp:285 quad_start->twin->to = quad_end->to;
                    quad_start_twin.as_ptr().as_mut().unwrap().to = Some(quad_end_to);
                    // SkeletalTrapezoidationGraph.cpp:286 quad_end->to->incident_edge = quad_end->twin;
                    quad_end_to.as_ptr().as_mut().unwrap().incident_edge = Some(quad_end_twin);
                    // SkeletalTrapezoidationGraph.cpp:287 if (quad_end->from->incident_edge == quad_end)
                    if quad_end_from.as_ref().incident_edge == Some(quad_end) {
                        // SkeletalTrapezoidationGraph.cpp:289 if (quad_end->twin->next)
                        if let Some(qe_twin_next) = quad_end_twin.as_ref().next {
                            // SkeletalTrapezoidationGraph.cpp:291 quad_end->from->incident_edge = quad_end->twin->next;
                            quad_end_from.as_ptr().as_mut().unwrap().incident_edge =
                                Some(qe_twin_next);
                        } else {
                            // SkeletalTrapezoidationGraph.cpp:295 quad_end->from->incident_edge = quad_end->prev->twin;
                            quad_end_from.as_ptr().as_mut().unwrap().incident_edge =
                                quad_end.as_ref().prev.unwrap().as_ref().twin;
                        }
                    }
                    // SkeletalTrapezoidationGraph.cpp:298 nodes.erase(node_locator[quad_start->from]);
                    nodes_to_remove.insert(quad_start.as_ref().from.unwrap().as_ptr() as *const _);

                    // SkeletalTrapezoidationGraph.cpp:300 quad_start->twin->twin = quad_end->twin;
                    quad_start_twin.as_ptr().as_mut().unwrap().twin = Some(quad_end_twin);
                    // SkeletalTrapezoidationGraph.cpp:301 quad_end->twin->twin = quad_start->twin;
                    quad_end_twin.as_ptr().as_mut().unwrap().twin = Some(quad_start_twin);
                    // SkeletalTrapezoidationGraph.cpp:302 safelyRemoveEdge(quad_start, ...);
                    edges_to_remove.insert(quad_start.as_ptr() as *const _);
                    // SkeletalTrapezoidationGraph.cpp:303 safelyRemoveEdge(quad_end, ...);
                    edges_to_remove.insert(quad_end.as_ptr() as *const _);
                }
                // SkeletalTrapezoidationGraph.cpp:305-312 comment + iterator advance (handled by for loop)
            }

            // Apply the deferred erasures (std::list::erase preserves order of survivors).
            //
            // The elements are `Box`ed, so popping/re-pushing moves only the `Box`
            // pointer — the heap `STHalfEdge`/`STHalfEdgeNode` payload (and thus
            // `&payload.base`, the live `edge_t*`/`node_t*`) stays at its address.
            // The removal set was filled with `quad_*.as_ptr()` (== `&payload.base`),
            // and `&e.base` below (via `Box` deref) is that same stable address, so
            // the identity comparison matches and survivors keep valid pointers.
            // R667 — COLLAPSEPROBE. R666 put the near-boundary edge deficit at 1.62x
            // with local connectivity 1.36x thinner than the node count predicts. This
            // counts what this function actually removes, so it can be compared against
            // the same counter in C++ rather than inferred from the source (R654).
            if crate::probe_enabled("COLLAPSEPROBE") {
                use std::sync::atomic::{AtomicU64, Ordering::Relaxed};
                static CALLS: AtomicU64 = AtomicU64::new(0);
                static E_BEFORE: AtomicU64 = AtomicU64::new(0);
                static E_REMOVED: AtomicU64 = AtomicU64::new(0);
                static N_REMOVED: AtomicU64 = AtomicU64::new(0);
                E_BEFORE.fetch_add(self.edges.iter().count() as u64, Relaxed);
                E_REMOVED.fetch_add(edges_to_remove.len() as u64, Relaxed);
                N_REMOVED.fetch_add(nodes_to_remove.len() as u64, Relaxed);
                let c = CALLS.fetch_add(1, Relaxed) + 1;
                if c % 2_000 == 0 {
                    let (b, e, n) = (
                        E_BEFORE.load(Relaxed),
                        E_REMOVED.load(Relaxed),
                        N_REMOVED.load(Relaxed),
                    );
                    eprintln!(
                        "[COLLAPSEPROBE] calls={c} edges_before/call={:.2} edges_removed/call={:.2} \
                         ({:.4} of them) nodes_removed/call={:.2}",
                        b as f64 / c as f64,
                        e as f64 / c as f64,
                        e as f64 / b.max(1) as f64,
                        n as f64 / c as f64,
                    );
                }
            }
            if !edges_to_remove.is_empty() {
                let mut new_edges: std::collections::LinkedList<Box<STHalfEdge>> =
                    std::collections::LinkedList::new();
                while let Some(e) = self.edges.pop_front() {
                    let p = &e.base as *const _;
                    if !edges_to_remove.contains(&p) {
                        new_edges.push_back(e);
                    }
                }
                self.edges = new_edges;
            }
            if !nodes_to_remove.is_empty() {
                let mut new_nodes: std::collections::LinkedList<Box<STHalfEdgeNode>> =
                    std::collections::LinkedList::new();
                while let Some(n) = self.nodes.pop_front() {
                    let p = &n.base as *const _;
                    if !nodes_to_remove.contains(&p) {
                        new_nodes.push_back(n);
                    }
                }
                self.nodes = new_nodes;
            }
        }
    }

    // SkeletalTrapezoidationGraph.cpp:316-346
    // void SkeletalTrapezoidationGraph::makeRib(edge_t*& prev_edge, Point start_source_point, Point end_source_point, bool is_next_to_start_or_end)
    pub fn make_rib(
        &mut self,
        prev_edge: &mut EdgePtr,
        start_source_point: Point,
        end_source_point: Point,
        _is_next_to_start_or_end: bool,
    ) {
        unsafe {
            // SkeletalTrapezoidationGraph.cpp:318 Point p;
            // SkeletalTrapezoidationGraph.cpp:319 Line(start_source_point, end_source_point).distance_to_infinite_squared(prev_edge->to->p, &p);
            let prev_to_p = prev_edge.as_ref().to.unwrap().as_ref().p;
            let p = distance_to_infinite_squared_closest_point(
                start_source_point,
                end_source_point,
                prev_to_p,
            );
            // SkeletalTrapezoidationGraph.cpp:320 coord_t dist = (prev_edge->to->p - p).cast<int64_t>().norm();
            let dist = (prev_to_p - p).length() as Coord;
            // SkeletalTrapezoidationGraph.cpp:321 prev_edge->to->data.distance_to_boundary = dist;
            prev_edge
                .as_ref()
                .to
                .unwrap()
                .as_ptr()
                .as_mut()
                .unwrap()
                .data
                .distance_to_boundary = dist;
            // SkeletalTrapezoidationGraph.cpp:322 assert(dist >= 0);
            debug_assert!(dist >= 0);

            // SkeletalTrapezoidationGraph.cpp:324 nodes.emplace_front(SkeletalTrapezoidationJoint(), p);
            self.nodes
                .push_front(Box::new(STHalfEdgeNode::new(SkeletalTrapezoidationJoint::new(), p)));
            // SkeletalTrapezoidationGraph.cpp:325 node_t* node = &nodes.front();
            let node = Self::node_ptr(self.nodes.front().unwrap());
            // SkeletalTrapezoidationGraph.cpp:326 node->data.distance_to_boundary = 0;
            node.as_ptr().as_mut().unwrap().data.distance_to_boundary = 0;

            // SkeletalTrapezoidationGraph.cpp:328 edges.emplace_front(SkeletalTrapezoidationEdge(SkeletalTrapezoidationEdge::EdgeType::EXTRA_VD));
            self.edges.push_front(Box::new(STHalfEdge::new(
                SkeletalTrapezoidationEdge::with_type(EdgeType::ExtraVd),
            )));
            // SkeletalTrapezoidationGraph.cpp:329 edge_t* forth_edge = &edges.front();
            let forth_edge = Self::edge_ptr(self.edges.front().unwrap());
            // SkeletalTrapezoidationGraph.cpp:330 forth_edge->data.setHoleCompensationFlag(prev_edge->data.getHoleCompensationFlag());
            forth_edge
                .as_ptr()
                .as_mut()
                .unwrap()
                .data
                .set_hole_compensation_flag(prev_edge.as_ref().data.get_hole_compensation_flag());
            // SkeletalTrapezoidationGraph.cpp:331 edges.emplace_front(SkeletalTrapezoidationEdge(SkeletalTrapezoidationEdge::EdgeType::EXTRA_VD));
            self.edges.push_front(Box::new(STHalfEdge::new(
                SkeletalTrapezoidationEdge::with_type(EdgeType::ExtraVd),
            )));
            // SkeletalTrapezoidationGraph.cpp:332 edge_t* back_edge = &edges.front();
            let back_edge = Self::edge_ptr(self.edges.front().unwrap());
            // SkeletalTrapezoidationGraph.cpp:333 back_edge->data.setHoleCompensationFlag(prev_edge->data.getHoleCompensationFlag());
            back_edge
                .as_ptr()
                .as_mut()
                .unwrap()
                .data
                .set_hole_compensation_flag(prev_edge.as_ref().data.get_hole_compensation_flag());

            // SkeletalTrapezoidationGraph.cpp:335 prev_edge->next = forth_edge;
            prev_edge.as_ptr().as_mut().unwrap().next = Some(forth_edge);
            {
                let forth = forth_edge.as_ptr().as_mut().unwrap();
                // SkeletalTrapezoidationGraph.cpp:336 forth_edge->prev = prev_edge;
                forth.prev = Some(*prev_edge);
                // SkeletalTrapezoidationGraph.cpp:337 forth_edge->from = prev_edge->to;
                forth.from = prev_edge.as_ref().to;
                // SkeletalTrapezoidationGraph.cpp:338 forth_edge->to = node;
                forth.to = Some(node);
                // SkeletalTrapezoidationGraph.cpp:339 forth_edge->twin = back_edge;
                forth.twin = Some(back_edge);
            }
            {
                let back = back_edge.as_ptr().as_mut().unwrap();
                // SkeletalTrapezoidationGraph.cpp:340 back_edge->twin = forth_edge;
                back.twin = Some(forth_edge);
                // SkeletalTrapezoidationGraph.cpp:341 back_edge->from = node;
                back.from = Some(node);
                // SkeletalTrapezoidationGraph.cpp:342 back_edge->to = prev_edge->to;
                back.to = prev_edge.as_ref().to;
            }
            // SkeletalTrapezoidationGraph.cpp:343 node->incident_edge = back_edge;
            node.as_ptr().as_mut().unwrap().incident_edge = Some(back_edge);

            // SkeletalTrapezoidationGraph.cpp:345 prev_edge = back_edge;
            *prev_edge = back_edge;
        }
    }

    // SkeletalTrapezoidationGraph.cpp:348-434
    // std::pair<edge_t*, edge_t*> SkeletalTrapezoidationGraph::insertRib(edge_t& edge, node_t* mid_node)
    pub fn insert_rib(&mut self, edge: EdgePtr, mid_node: NodePtr) -> (EdgePtr, EdgePtr) {
        unsafe {
            let edge_ref = edge.as_ref();
            // SkeletalTrapezoidationGraph.cpp:350 edge_t* edge_before = edge.prev;
            let edge_before = edge_ref.prev;
            // SkeletalTrapezoidationGraph.cpp:351 edge_t* edge_after = edge.next;
            let edge_after = edge_ref.next;
            // SkeletalTrapezoidationGraph.cpp:352 node_t* node_before = edge.from;
            let node_before = edge_ref.from.unwrap();
            // SkeletalTrapezoidationGraph.cpp:353 node_t* node_after = edge.to;
            let node_after = edge_ref.to.unwrap();

            // SkeletalTrapezoidationGraph.cpp:355 Point p = mid_node->p;
            let p = mid_node.as_ref().p;

            // SkeletalTrapezoidationGraph.cpp:357 bool apply_hole_compensation = edge.data.getHoleCompensationFlag();
            let apply_hole_compensation = edge_ref.data.get_hole_compensation_flag();

            // SkeletalTrapezoidationGraph.cpp:359 const Line source_segment = getSource(edge);
            let source_segment = self.get_source(edge);
            // SkeletalTrapezoidationGraph.cpp:360 Point px;
            // SkeletalTrapezoidationGraph.cpp:361 source_segment.distance_to_squared(p, &px);
            let px = distance_to_squared_closest_point(source_segment.a, source_segment.b, p);
            // SkeletalTrapezoidationGraph.cpp:362 coord_t dist = (p - px).cast<int64_t>().norm();
            let dist = (p - px).length() as Coord;
            // SkeletalTrapezoidationGraph.cpp:363 assert(dist > 0);
            debug_assert!(dist > 0);
            // SkeletalTrapezoidationGraph.cpp:364 mid_node->data.distance_to_boundary = dist;
            mid_node.as_ptr().as_mut().unwrap().data.distance_to_boundary = dist;
            // SkeletalTrapezoidationGraph.cpp:365 mid_node->data.transition_ratio = 0;
            // Both transition end should have rest = 0, because at the ends a whole number of beads fits without rest
            mid_node.as_ptr().as_mut().unwrap().data.transition_ratio = 0.0_f32;

            // SkeletalTrapezoidationGraph.cpp:367 nodes.emplace_back(SkeletalTrapezoidationJoint(), px);
            self.nodes
                .push_back(Box::new(STHalfEdgeNode::new(SkeletalTrapezoidationJoint::new(), px)));
            // SkeletalTrapezoidationGraph.cpp:368 node_t* source_node = &nodes.back();
            let source_node = Self::node_ptr(self.nodes.back().unwrap());
            // SkeletalTrapezoidationGraph.cpp:369 source_node->data.distance_to_boundary = 0;
            source_node.as_ptr().as_mut().unwrap().data.distance_to_boundary = 0;

            // SkeletalTrapezoidationGraph.cpp:371 edge_t* first = &edge;
            let first = edge;
            // SkeletalTrapezoidationGraph.cpp:372 edges.emplace_back(SkeletalTrapezoidationEdge());
            self.edges
                .push_back(Box::new(STHalfEdge::new(SkeletalTrapezoidationEdge::new())));
            // SkeletalTrapezoidationGraph.cpp:373 edge_t* second = &edges.back();
            let second = Self::edge_ptr(self.edges.back().unwrap());
            // SkeletalTrapezoidationGraph.cpp:374 edges.emplace_back(SkeletalTrapezoidationEdge(SkeletalTrapezoidationEdge::EdgeType::TRANSITION_END));
            self.edges.push_back(Box::new(STHalfEdge::new(
                SkeletalTrapezoidationEdge::with_type(EdgeType::TransitionEnd),
            )));
            // SkeletalTrapezoidationGraph.cpp:375 edge_t* outward_edge = &edges.back();
            let outward_edge = Self::edge_ptr(self.edges.back().unwrap());
            // SkeletalTrapezoidationGraph.cpp:376 edges.emplace_back(SkeletalTrapezoidationEdge(SkeletalTrapezoidationEdge::EdgeType::TRANSITION_END));
            self.edges.push_back(Box::new(STHalfEdge::new(
                SkeletalTrapezoidationEdge::with_type(EdgeType::TransitionEnd),
            )));
            // SkeletalTrapezoidationGraph.cpp:377 edge_t* inward_edge = &edges.back();
            let inward_edge = Self::edge_ptr(self.edges.back().unwrap());

            // SkeletalTrapezoidationGraph.cpp:379 first->data.setHoleCompensationFlag(apply_hole_compensation);
            first.as_ptr().as_mut().unwrap().data.set_hole_compensation_flag(apply_hole_compensation);
            // SkeletalTrapezoidationGraph.cpp:380 second->data.setHoleCompensationFlag(apply_hole_compensation);
            second.as_ptr().as_mut().unwrap().data.set_hole_compensation_flag(apply_hole_compensation);
            // SkeletalTrapezoidationGraph.cpp:381 outward_edge->data.setHoleCompensationFlag(apply_hole_compensation);
            outward_edge.as_ptr().as_mut().unwrap().data.set_hole_compensation_flag(apply_hole_compensation);
            // SkeletalTrapezoidationGraph.cpp:382 inward_edge->data.setHoleCompensationFlag(apply_hole_compensation);
            inward_edge.as_ptr().as_mut().unwrap().data.set_hole_compensation_flag(apply_hole_compensation);

            // SkeletalTrapezoidationGraph.cpp:384 if (edge_before)
            if let Some(edge_before) = edge_before {
                // SkeletalTrapezoidationGraph.cpp:386 edge_before->next = first;
                edge_before.as_ptr().as_mut().unwrap().next = Some(first);
            }
            // SkeletalTrapezoidationGraph.cpp:388 first->next = outward_edge;
            first.as_ptr().as_mut().unwrap().next = Some(outward_edge);
            // SkeletalTrapezoidationGraph.cpp:389 outward_edge->next = nullptr;
            outward_edge.as_ptr().as_mut().unwrap().next = None;
            // SkeletalTrapezoidationGraph.cpp:390 inward_edge->next = second;
            inward_edge.as_ptr().as_mut().unwrap().next = Some(second);
            // SkeletalTrapezoidationGraph.cpp:391 second->next = edge_after;
            second.as_ptr().as_mut().unwrap().next = edge_after;

            // SkeletalTrapezoidationGraph.cpp:393 if (edge_after)
            if let Some(edge_after) = edge_after {
                // SkeletalTrapezoidationGraph.cpp:395 edge_after->prev = second;
                edge_after.as_ptr().as_mut().unwrap().prev = Some(second);
            }
            // SkeletalTrapezoidationGraph.cpp:397 second->prev = inward_edge;
            second.as_ptr().as_mut().unwrap().prev = Some(inward_edge);
            // SkeletalTrapezoidationGraph.cpp:398 inward_edge->prev = nullptr;
            inward_edge.as_ptr().as_mut().unwrap().prev = None;
            // SkeletalTrapezoidationGraph.cpp:399 outward_edge->prev = first;
            outward_edge.as_ptr().as_mut().unwrap().prev = Some(first);
            // SkeletalTrapezoidationGraph.cpp:400 first->prev = edge_before;
            first.as_ptr().as_mut().unwrap().prev = edge_before;

            // SkeletalTrapezoidationGraph.cpp:402 first->to = mid_node;
            first.as_ptr().as_mut().unwrap().to = Some(mid_node);
            // SkeletalTrapezoidationGraph.cpp:403 outward_edge->to = source_node;
            outward_edge.as_ptr().as_mut().unwrap().to = Some(source_node);
            // SkeletalTrapezoidationGraph.cpp:404 inward_edge->to = mid_node;
            inward_edge.as_ptr().as_mut().unwrap().to = Some(mid_node);
            // SkeletalTrapezoidationGraph.cpp:405 second->to = node_after;
            second.as_ptr().as_mut().unwrap().to = Some(node_after);

            // SkeletalTrapezoidationGraph.cpp:407 first->from = node_before;
            first.as_ptr().as_mut().unwrap().from = Some(node_before);
            // SkeletalTrapezoidationGraph.cpp:408 outward_edge->from = mid_node;
            outward_edge.as_ptr().as_mut().unwrap().from = Some(mid_node);
            // SkeletalTrapezoidationGraph.cpp:409 inward_edge->from = source_node;
            inward_edge.as_ptr().as_mut().unwrap().from = Some(source_node);
            // SkeletalTrapezoidationGraph.cpp:410 second->from = mid_node;
            second.as_ptr().as_mut().unwrap().from = Some(mid_node);

            // SkeletalTrapezoidationGraph.cpp:412 node_before->incident_edge = first;
            node_before.as_ptr().as_mut().unwrap().incident_edge = Some(first);
            // SkeletalTrapezoidationGraph.cpp:413 mid_node->incident_edge = outward_edge;
            mid_node.as_ptr().as_mut().unwrap().incident_edge = Some(outward_edge);
            // SkeletalTrapezoidationGraph.cpp:414 source_node->incident_edge = inward_edge;
            source_node.as_ptr().as_mut().unwrap().incident_edge = Some(inward_edge);
            // SkeletalTrapezoidationGraph.cpp:415 if (edge_after)
            if let Some(_edge_after) = edge_after {
                // SkeletalTrapezoidationGraph.cpp:417 node_after->incident_edge = edge_after;
                node_after.as_ptr().as_mut().unwrap().incident_edge = edge_after;
            }

            // SkeletalTrapezoidationGraph.cpp:420 first->data.setIsCentral(true);
            first.as_ptr().as_mut().unwrap().data.set_is_central(true);
            // SkeletalTrapezoidationGraph.cpp:421 outward_edge->data.setIsCentral(false); // TODO verify this is always the case.
            outward_edge.as_ptr().as_mut().unwrap().data.set_is_central(false);
            // SkeletalTrapezoidationGraph.cpp:422 inward_edge->data.setIsCentral(false);
            inward_edge.as_ptr().as_mut().unwrap().data.set_is_central(false);
            // SkeletalTrapezoidationGraph.cpp:423 second->data.setIsCentral(true);
            second.as_ptr().as_mut().unwrap().data.set_is_central(true);

            // SkeletalTrapezoidationGraph.cpp:425 outward_edge->twin = inward_edge;
            outward_edge.as_ptr().as_mut().unwrap().twin = Some(inward_edge);
            // SkeletalTrapezoidationGraph.cpp:426 inward_edge->twin = outward_edge;
            inward_edge.as_ptr().as_mut().unwrap().twin = Some(outward_edge);

            // SkeletalTrapezoidationGraph.cpp:428 first->twin = nullptr; // we don't know these yet!
            first.as_ptr().as_mut().unwrap().twin = None;
            // SkeletalTrapezoidationGraph.cpp:429 second->twin = nullptr;
            second.as_ptr().as_mut().unwrap().twin = None;

            // SkeletalTrapezoidationGraph.cpp:431 assert(second->prev->from->data.distance_to_boundary == 0);
            debug_assert!(
                second
                    .as_ref()
                    .prev
                    .unwrap()
                    .as_ref()
                    .from
                    .unwrap()
                    .as_ref()
                    .data
                    .distance_to_boundary
                    == 0
            );

            // SkeletalTrapezoidationGraph.cpp:433 return std::make_pair(first, second);
            (first, second)
        }
    }

    // SkeletalTrapezoidationGraph.cpp:436-461
    // SkeletalTrapezoidationGraph::edge_t* SkeletalTrapezoidationGraph::insertNode(edge_t* edge, Point mid, coord_t mide_node_bead_count)
    pub fn insert_node(&mut self, edge: EdgePtr, mid: Point, mide_node_bead_count: Coord) -> EdgePtr {
        unsafe {
            // SkeletalTrapezoidationGraph.cpp:438 edge_t* last_edge_replacing_input = edge;
            let mut last_edge_replacing_input = edge;

            // SkeletalTrapezoidationGraph.cpp:440 nodes.emplace_back(SkeletalTrapezoidationJoint(), mid);
            self.nodes
                .push_back(Box::new(STHalfEdgeNode::new(SkeletalTrapezoidationJoint::new(), mid)));
            // SkeletalTrapezoidationGraph.cpp:441 node_t* mid_node = &nodes.back();
            let mid_node = Self::node_ptr(self.nodes.back().unwrap());

            // SkeletalTrapezoidationGraph.cpp:443 edge_t* twin = last_edge_replacing_input->twin;
            let twin = last_edge_replacing_input.as_ref().twin.unwrap();
            // SkeletalTrapezoidationGraph.cpp:444 last_edge_replacing_input->twin = nullptr;
            last_edge_replacing_input.as_ptr().as_mut().unwrap().twin = None;
            // SkeletalTrapezoidationGraph.cpp:445 twin->twin = nullptr;
            twin.as_ptr().as_mut().unwrap().twin = None;
            // SkeletalTrapezoidationGraph.cpp:446 std::pair<edge_t*, edge_t*> left_pair = insertRib(*last_edge_replacing_input, mid_node);
            let left_pair = self.insert_rib(last_edge_replacing_input, mid_node);
            // SkeletalTrapezoidationGraph.cpp:447 std::pair<edge_t*, edge_t*> right_pair = insertRib(*twin, mid_node);
            let right_pair = self.insert_rib(twin, mid_node);
            // SkeletalTrapezoidationGraph.cpp:448 edge_t* first_edge_replacing_input = left_pair.first;
            let first_edge_replacing_input = left_pair.0;
            // SkeletalTrapezoidationGraph.cpp:449 last_edge_replacing_input = left_pair.second;
            last_edge_replacing_input = left_pair.1;
            // SkeletalTrapezoidationGraph.cpp:450 edge_t* first_edge_replacing_twin = right_pair.first;
            let first_edge_replacing_twin = right_pair.0;
            // SkeletalTrapezoidationGraph.cpp:451 edge_t* last_edge_replacing_twin = right_pair.second;
            let last_edge_replacing_twin = right_pair.1;

            // SkeletalTrapezoidationGraph.cpp:453 first_edge_replacing_input->twin = last_edge_replacing_twin;
            first_edge_replacing_input.as_ptr().as_mut().unwrap().twin = Some(last_edge_replacing_twin);
            // SkeletalTrapezoidationGraph.cpp:454 last_edge_replacing_twin->twin = first_edge_replacing_input;
            last_edge_replacing_twin.as_ptr().as_mut().unwrap().twin = Some(first_edge_replacing_input);
            // SkeletalTrapezoidationGraph.cpp:455 last_edge_replacing_input->twin = first_edge_replacing_twin;
            last_edge_replacing_input.as_ptr().as_mut().unwrap().twin = Some(first_edge_replacing_twin);
            // SkeletalTrapezoidationGraph.cpp:456 first_edge_replacing_twin->twin = last_edge_replacing_input;
            first_edge_replacing_twin.as_ptr().as_mut().unwrap().twin = Some(last_edge_replacing_input);

            // SkeletalTrapezoidationGraph.cpp:458 mid_node->data.bead_count = mide_node_bead_count;
            mid_node.as_ptr().as_mut().unwrap().data.bead_count = mide_node_bead_count;

            // SkeletalTrapezoidationGraph.cpp:460 return last_edge_replacing_input;
            last_edge_replacing_input
        }
    }

    // SkeletalTrapezoidationGraph.cpp:463-474
    // Line SkeletalTrapezoidationGraph::getSource(const edge_t &edge) const
    pub fn get_source(&self, edge: EdgePtr) -> Line {
        unsafe {
            // SkeletalTrapezoidationGraph.cpp:465 const edge_t *from_edge = &edge;
            let mut from_edge = edge;
            // SkeletalTrapezoidationGraph.cpp:466 while (from_edge->prev) from_edge = from_edge->prev;
            while let Some(prev) = from_edge.as_ref().prev {
                from_edge = prev;
            }

            // SkeletalTrapezoidationGraph.cpp:469 const edge_t *to_edge = &edge;
            let mut to_edge = edge;
            // SkeletalTrapezoidationGraph.cpp:470 while (to_edge->next) to_edge = to_edge->next;
            while let Some(next) = to_edge.as_ref().next {
                to_edge = next;
            }

            // SkeletalTrapezoidationGraph.cpp:473 return Line(from_edge->from->p, to_edge->to->p);
            Line::new(
                from_edge.as_ref().from.unwrap().as_ref().p,
                to_edge.as_ref().to.unwrap().as_ref().p,
            )
        }
    }
}

impl Default for SkeletalTrapezoidationGraph {
    fn default() -> Self {
        Self::new()
    }
}

// SkeletalTrapezoidationGraph is safe to send/share if the data types are (matches
// the sibling HalfEdge/HalfEdgeNode/HalfEdgeGraph Send/Sync impls — the graph stores
// raw pointers internally).
unsafe impl Send for SkeletalTrapezoidationGraph {}
unsafe impl Sync for SkeletalTrapezoidationGraph {}

/// Faithful port of `Line::distance_to_squared(const Point&, Point*)` returning the
/// nearest point (clamped to the segment), per Line.hpp:43-69.
fn distance_to_squared_closest_point(a: Point, b: Point, point: Point) -> Point {
    // Line.hpp:45 const Vec v  = (get_b(line) - get_a(line)).cast<double>();
    let vx = (b.x - a.x) as f64;
    let vy = (b.y - a.y) as f64;
    // Line.hpp:46 const Vec va = (point - get_a(line)).cast<double>();
    let vax = (point.x - a.x) as f64;
    let vay = (point.y - a.y) as f64;
    // Line.hpp:47 const double l2 = v.squaredNorm();
    let l2 = vx * vx + vy * vy;
    // Line.hpp:48 if (l2 == 0.0) { *nearest_point = get_a(line); ... }
    if l2 == 0.0 {
        return a;
    }
    // Line.hpp:55 const double t = va.dot(v) / l2;
    let t = (vax * vx + vay * vy) / l2;
    // Line.hpp:56 if (t <= 0.0) { *nearest_point = get_a(line); ... }
    if t <= 0.0 {
        return a;
    } else if t >= 1.0 {
        // Line.hpp:59 } else if (t >= 1.0) { *nearest_point = get_b(line); ... }
        return b;
    }
    // Line.hpp:64 *nearest_point = (get_a(line).cast<double>() + t * v).cast<Scalar>();
    Point::new((a.x as f64 + t * vx) as Coord, (a.y as f64 + t * vy) as Coord)
}

/// Faithful port of `Line::distance_to_infinite_squared(const Point&, Point*)`
/// returning the closest point on the infinite line, per Line.hpp:88-104.
fn distance_to_infinite_squared_closest_point(a: Point, b: Point, point: Point) -> Point {
    // Line.hpp:90 const Vec v  = (get_b(line) - get_a(line)).cast<double>();
    let vx = (b.x - a.x) as f64;
    let vy = (b.y - a.y) as f64;
    // Line.hpp:91 const Vec va = (point - get_a(line)).cast<double>();
    let vax = (point.x - a.x) as f64;
    let vay = (point.y - a.y) as f64;
    // Line.hpp:92 const double l2 = v.squaredNorm();
    let l2 = vx * vx + vy * vy;
    // Line.hpp:93 if (l2 == 0.) { *closest_point = get_a(line); ... }
    if l2 == 0.0 {
        return a;
    }
    // Line.hpp:100 const double t = va.dot(v) / l2;
    let t = (vax * vx + vay * vy) / l2;
    // Line.hpp:101 *closest_point = (get_a(line).cast<double>() + t * v).cast<Scalar>();
    Point::new((a.x as f64 + t * vx) as Coord, (a.y as f64 + t * vy) as Coord)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_graph_creation() {
        // Test basic graph creation
        let graph = SkeletalTrapezoidationGraph::new();
        assert_eq!(graph.edges.len(), 0);
        assert_eq!(graph.nodes.len(), 0);
    }

    #[test]
    fn test_st_half_edge_creation() {
        // SkeletalTrapezoidationGraph.cpp:15
        let edge_data = SkeletalTrapezoidationEdge::new();
        let _st_edge = STHalfEdge::new(edge_data);
    }

    #[test]
    fn test_st_half_edge_node_creation() {
        // SkeletalTrapezoidationGraph.cpp:123
        let node_data = SkeletalTrapezoidationJoint::new();
        let point = Point::new(100, 200);
        let st_node = STHalfEdgeNode::new(node_data, point);
        assert_eq!(st_node.base.p.x, 100);
        assert_eq!(st_node.base.p.y, 200);
    }

    #[test]
    fn test_distance_to_infinite_closest_point() {
        // Projection of (5,5) onto the infinite line through (0,0)-(10,0) is (5,0).
        let cp = distance_to_infinite_squared_closest_point(
            Point::new(0, 0),
            Point::new(10, 0),
            Point::new(5, 5),
        );
        assert_eq!(cp, Point::new(5, 0));
    }

    #[test]
    fn test_distance_to_squared_closest_point_clamps() {
        // Projection of (20,5) onto segment (0,0)-(10,0) clamps to b=(10,0).
        let cp =
            distance_to_squared_closest_point(Point::new(0, 0), Point::new(10, 0), Point::new(20, 5));
        assert_eq!(cp, Point::new(10, 0));
    }
}
