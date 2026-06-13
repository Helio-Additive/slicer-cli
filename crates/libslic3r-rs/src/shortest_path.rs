//! Faithful 1:1 line-by-line port of BambuStudio `src/libslic3r/ShortestPath.cpp`.
//!
//! C++ Reference:
//! - ShortestPath.hpp
//! - ShortestPath.cpp
//!
//! Greedy multi-fragment Traveling-Salesman-Problem chaining of extrusion paths,
//! polylines and points by an approximate shortest path.
//!
//! Pointer semantics note: the C++ implementation is heavily pointer based. End
//! points and chains live in `std::vector`s and are referenced via raw pointers,
//! and pointer arithmetic (`&ep - &end_points.front()`) is used to recover the
//! element index. This port replaces raw pointers with `usize` indices into the
//! corresponding vectors (`NULLPTR` == `usize::MAX` mirrors `nullptr`), and the
//! `MutablePriorityQueue` holds those indices. The `heap_idx`/`chain_id`/`edge_*`
//! fields stay byte-faithful, only their storage moves from pointer to index.

use std::cell::RefCell;
use std::rc::Rc;

use crate::ex_polygon::get_extents_expoly;
use crate::extrusion_entity::{ExtrusionEntityType, ExtrusionPath};
use crate::geometry::{ExPolygons, Point, Polyline, Polylines};
use crate::kd_tree_indirect::{find_closest_point, KDTreeIndirect};
use crate::mutable_priority_queue::make_mutable_priority_queue;

// Sentinel for a null `EndPoint*` / `Chain*` pointer.
const NULLPTR: usize = usize::MAX;
// std::numeric_limits<size_t>::max()
const SIZE_MAX: usize = usize::MAX;

// Convert a Point (scaled i64 coords) to a [f64; 2] (the C++ `.cast<double>()`).
#[inline]
fn pt_to_vec2d(p: &Point) -> [f64; 2] {
    [p.x as f64, p.y as f64]
}

#[inline]
fn squared_norm(a: &[f64; 2], b: &[f64; 2]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    dx * dx + dy * dy
}

#[inline]
fn norm(a: &[f64; 2], b: &[f64; 2]) -> f64 {
    squared_norm(a, b).sqrt()
}

// Naive implementation of the Traveling Salesman Problem, it works by always taking the next closest neighbor.
// This implementation will always produce valid result even if some segments cannot reverse.
// ShortestPath.cpp:18-56
//
// `EndPointClosest` mirrors the `EndPointType` used by the closest-point fallback.
// It only needs `pos` and `chain_id`.
struct EndPointClosest {
    pos: [f64; 2],
    chain_id: usize,
}

// ShortestPath.cpp:20-56
fn chain_segments_closest_point<CouldReverseFunc>(
    end_points: &mut Vec<EndPointClosest>,
    kdtree: &KDTreeIndirect<2, f64, impl Fn(usize, usize) -> f64>,
    could_reverse_func: &CouldReverseFunc,
    first_point_idx: usize,
) -> Vec<(usize, bool)>
where
    CouldReverseFunc: Fn(usize) -> bool,
{
    // ShortestPath.cpp:23-24
    assert_eq!(end_points.len() & 1, 0);
    let num_segments = end_points.len() / 2;
    // ShortestPath.cpp:25
    assert!(num_segments >= 2);
    // ShortestPath.cpp:26-27
    for ep in end_points.iter_mut() {
        ep.chain_id = 0;
    }
    // ShortestPath.cpp:28-29
    let mut out: Vec<(usize, bool)> = Vec::new();
    out.reserve(num_segments);
    // ShortestPath.cpp:30-32
    // size_t first_point_idx = &first_point - end_points.data();
    out.push((first_point_idx / 2, (first_point_idx & 1) != 0));
    end_points[first_point_idx].chain_id = 1;
    // ShortestPath.cpp:33
    let mut this_idx = first_point_idx ^ 1;
    // ShortestPath.cpp:34
    let mut iter = num_segments as i64 - 2;
    while iter >= 0 {
        // ShortestPath.cpp:35-36
        end_points[this_idx].chain_id = 1;
        let this_pos = end_points[this_idx].pos;
        // Find the closest point to this end_point, which lies on a different extrusion path (filtered by the lambda).
        // Ignore the starting point as the starting point is considered to be occupied, no end point coud connect to it.
        // ShortestPath.cpp:39-42
        let next_idx = find_closest_point(kdtree, &this_pos, |idx: usize| {
            (idx ^ this_idx) > 1
                && end_points[idx].chain_id == 0
                && ((idx & 1) == 0 || could_reverse_func(idx >> 1))
        });
        // ShortestPath.cpp:43
        assert!(next_idx < end_points.len());
        // ShortestPath.cpp:44-45
        end_points[next_idx].chain_id = 1;
        // ShortestPath.cpp:46
        assert!((next_idx & 1) == 0 || could_reverse_func(next_idx >> 1));
        // ShortestPath.cpp:47
        out.push((next_idx / 2, (next_idx & 1) != 0));
        // ShortestPath.cpp:48
        this_idx = next_idx ^ 1;
        iter -= 1;
    }
    // ShortestPath.cpp:50-54 (NDEBUG validation block omitted)
    // ShortestPath.cpp:55
    out
}

// End point of a segment, used by `chain_segments_greedy_constrained_reversals_`.
// ShortestPath.cpp:84-95
#[derive(Clone)]
struct EndPoint1 {
    pos: [f64; 2],
    // Identifier of the chain, to which this end point belongs. Zero means unassigned.
    chain_id: usize,
    // Link to the closest currently valid end point. (nullptr == NULLPTR)
    edge_out: usize,
    // Distance to the next end point following the link.
    distance_out: f64,
    heap_idx: usize,
}

// Helper to detect loops in already connected paths.
// ShortestPath.cpp:110-163
struct EquivalentChains {
    // Unique chain ID assigned to chains of end points of segments.
    m_last_chain_id: usize,
    m_equivalent_with: Vec<usize>,
}

impl EquivalentChains {
    // Zero'th chain ID is invalid.
    // ShortestPath.cpp:113
    fn new(reserve: usize) -> Self {
        let mut m_equivalent_with = Vec::with_capacity(reserve);
        m_equivalent_with.push(0);
        EquivalentChains {
            m_last_chain_id: 0,
            m_equivalent_with,
        }
    }

    // Generate next equivalence class.
    // ShortestPath.cpp:115-118
    fn next(&mut self) -> usize {
        self.m_last_chain_id += 1;
        self.m_equivalent_with.push(self.m_last_chain_id);
        self.m_last_chain_id
    }

    // Get equivalence class for chain ID.
    // ShortestPath.cpp:120-133
    fn equivalent(&mut self, mut chain_id: usize) -> usize {
        if chain_id != 0 {
            let mut last = chain_id;
            loop {
                let lower = self.m_equivalent_with[last];
                if lower == last {
                    self.m_equivalent_with[chain_id] = lower;
                    chain_id = lower;
                    break;
                }
                last = lower;
            }
        }
        chain_id
    }

    // ShortestPath.cpp:134-139
    fn merge(&mut self, chain_id1: usize, chain_id2: usize) -> usize {
        let e1 = self.equivalent(chain_id1);
        let e2 = self.equivalent(chain_id2);
        let chain_id = std::cmp::min(e1, e2);
        self.m_equivalent_with[chain_id1] = chain_id;
        self.m_equivalent_with[chain_id2] = chain_id;
        chain_id
    }
}

// Chain perimeters (always closed) and thin fills (closed or open) using a greedy algorithm.
// Solving a Traveling Salesman Problem (TSP) with the modification, that the sites are not always points, but points and segments.
// Solving using a greedy algorithm, where a shortest edge is added to the solution if it does not produce a bifurcation or a cycle.
// Return index and "reversed" flag.
// https://en.wikipedia.org/wiki/Multi-fragment_algorithm
// ShortestPath.cpp:58-394
fn chain_segments_greedy_constrained_reversals_<SegmentEndPointFunc, CouldReverseFunc>(
    end_point_func: SegmentEndPointFunc,
    could_reverse_func: CouldReverseFunc,
    num_segments: usize,
    start_near: Option<&Point>,
    reverse_could_fail: bool,
) -> Vec<(usize, bool)>
where
    SegmentEndPointFunc: Fn(usize, bool) -> Point,
    CouldReverseFunc: Fn(usize) -> bool,
{
    // ShortestPath.cpp:69
    let mut out: Vec<(usize, bool)> = Vec::new();

    if num_segments == 0 {
        // Nothing to do.
        // ShortestPath.cpp:71-73
    } else if num_segments == 1 {
        // Just sort the end points so that the first point visited is closest to start_near.
        // ShortestPath.cpp:74-79
        let reversed = could_reverse_func(0)
            && start_near.is_some()
            && squared_norm(&pt_to_vec2d(&end_point_func(0, false)), &pt_to_vec2d(start_near.unwrap()))
                < squared_norm(
                    &pt_to_vec2d(&end_point_func(0, true)),
                    &pt_to_vec2d(start_near.unwrap()),
                );
        out.push((0, reversed));
    } else {
        // ShortestPath.cpp:96-101
        // End points of segments for the KD tree closest point search.
        let end_points: Rc<RefCell<Vec<EndPoint1>>> =
            Rc::new(RefCell::new(Vec::with_capacity(num_segments * 2)));
        {
            let mut ep = end_points.borrow_mut();
            for i in 0..num_segments {
                ep.push(EndPoint1 {
                    pos: pt_to_vec2d(&end_point_func(i, true)),
                    chain_id: 0,
                    edge_out: NULLPTR,
                    distance_out: f64::MAX,
                    heap_idx: SIZE_MAX,
                });
                ep.push(EndPoint1 {
                    pos: pt_to_vec2d(&end_point_func(i, false)),
                    chain_id: 0,
                    edge_out: NULLPTR,
                    distance_out: f64::MAX,
                    heap_idx: SIZE_MAX,
                });
            }
        }

        // Construct the closest point KD tree over end points of segments.
        // ShortestPath.cpp:104-105
        let kd_points = Rc::clone(&end_points);
        let coordinate_fn =
            move |idx: usize, dimension: usize| -> f64 { kd_points.borrow()[idx].pos[dimension] };
        let n = end_points.borrow().len();
        let kdtree: KDTreeIndirect<2, f64, _> = KDTreeIndirect::with_indices(coordinate_fn, n);

        // Helper to detect loops in already connected paths.
        // ShortestPath.cpp:110-163
        let mut equivalent_chain = EquivalentChains::new(num_segments);

        // Find the first end point closest to start_near.
        // ShortestPath.cpp:166-177
        let mut first_point: usize = NULLPTR;
        let mut first_point_idx: usize = SIZE_MAX;
        if let Some(sn) = start_near {
            let sn_v = pt_to_vec2d(sn);
            let idx = find_closest_point(&kdtree, &sn_v, |idx: usize| {
                // Don't start with a reverse segment, if flipping of the segment is not allowed.
                (idx & 1) == 0 || could_reverse_func(idx >> 1)
            });
            assert!(idx < end_points.borrow().len());
            {
                let mut ep = end_points.borrow_mut();
                ep[idx].distance_out = 0.;
            }
            let cid = equivalent_chain.next();
            end_points.borrow_mut()[idx].chain_id = cid;
            first_point = idx;
            first_point_idx = idx;
        }
        // ShortestPath.cpp:178-179
        let initial_point = first_point;
        let mut last_point: usize = NULLPTR;

        // Assign the closest point and distance to the end points.
        // ShortestPath.cpp:182-195
        let len = end_points.borrow().len();
        for this_idx in 0..len {
            assert!(end_points.borrow()[this_idx].edge_out == NULLPTR);
            if this_idx != first_point {
                let this_pos = end_points.borrow()[this_idx].pos;
                // Find the closest point to this end_point, which lies on a different extrusion path (filtered by the lambda).
                // Ignore the starting point as the starting point is considered to be occupied, no end point coud connect to it.
                // ShortestPath.cpp:188-189
                let next_idx = find_closest_point(&kdtree, &this_pos, |idx: usize| {
                    idx != first_point_idx && (idx ^ this_idx) > 1
                });
                assert!(next_idx < end_points.borrow().len());
                let next_pos = end_points.borrow()[next_idx].pos;
                let mut ep = end_points.borrow_mut();
                ep[this_idx].edge_out = next_idx;
                ep[this_idx].distance_out = squared_norm(&next_pos, &this_pos);
            }
        }

        // Initialize a heap of end points sorted by the lowest distance to the next valid point of a path.
        // ShortestPath.cpp:198-204
        let q_setter = Rc::clone(&end_points);
        let q_less = Rc::clone(&end_points);
        let mut queue = make_mutable_priority_queue::<usize, _, _>(
            false,
            move |ep: &usize, idx: usize| {
                q_setter.borrow_mut()[*ep].heap_idx = idx;
            },
            move |l: &usize, r: &usize| -> bool {
                let v = q_less.borrow();
                v[*l].distance_out < v[*r].distance_out
            },
        );
        queue.reserve(end_points.borrow().len() * 2 - 1);
        for ep in 0..end_points.borrow().len() {
            if first_point != ep {
                queue.push(ep);
            }
        }

        // Chain the end points: find (num_segments - 1) shortest links not forming bifurcations or loops.
        // ShortestPath.cpp:243
        assert!(num_segments >= 2);
        // ShortestPath.cpp:247
        let mut iter: i64 = num_segments as i64 - 2;
        loop {
            // ShortestPath.cpp:250
            // Take the first end point, for which the link points to the currently closest valid neighbor.
            let end_point1: usize = *queue.top().unwrap();
            assert!(end_points.borrow()[end_point1].edge_out != NULLPTR);
            // No point on the queue may be connected yet.
            assert!(end_points.borrow()[end_point1].chain_id == 0);
            // Take the closest end point to the first end point,
            // ShortestPath.cpp:260
            let end_point2: usize = end_points.borrow()[end_point1].edge_out;
            let mut valid = true;
            let mut end_point1_other_chain_id: usize = 0;
            let mut end_point2_other_chain_id: usize = 0;
            // ShortestPath.cpp:264-274
            if end_points.borrow()[end_point2].chain_id > 0 {
                // The other side is part of the output path. Don't connect to end_point2, update end_point1 and try another one.
                valid = false;
            } else {
                // End points of the opposite ends of the segments.
                let c1 = end_points.borrow()[end_point1 ^ 1].chain_id;
                let c2 = end_points.borrow()[end_point2 ^ 1].chain_id;
                end_point1_other_chain_id = equivalent_chain.equivalent(c1);
                end_point2_other_chain_id = equivalent_chain.equivalent(c2);
                if end_point1_other_chain_id == end_point2_other_chain_id
                    && end_point1_other_chain_id != 0
                {
                    // This edge forms a loop. Update end_point1 and try another one.
                    valid = false;
                }
            }
            if valid {
                // Remove the first and second point from the queue.
                // ShortestPath.cpp:277-281
                queue.pop();
                let ep2_heap = end_points.borrow()[end_point2].heap_idx;
                queue.remove(ep2_heap);
                {
                    let d1 = end_points.borrow()[end_point1].distance_out;
                    let mut ep = end_points.borrow_mut();
                    ep[end_point2].edge_out = end_point1;
                    ep[end_point2].distance_out = d1;
                }
                // Assign chain IDs to the newly connected end points, set equivalent_chain if two chains were merged.
                // ShortestPath.cpp:283-291
                let chain_id = if end_point1_other_chain_id == 0 {
                    if end_point2_other_chain_id == 0 {
                        equivalent_chain.next()
                    } else {
                        end_point2_other_chain_id
                    }
                } else if end_point2_other_chain_id == 0 {
                    end_point1_other_chain_id
                } else if end_point1_other_chain_id == end_point2_other_chain_id {
                    end_point1_other_chain_id
                } else {
                    equivalent_chain.merge(end_point1_other_chain_id, end_point2_other_chain_id)
                };
                {
                    let mut ep = end_points.borrow_mut();
                    ep[end_point1].chain_id = chain_id;
                    ep[end_point2].chain_id = chain_id;
                }
                // ShortestPath.cpp:293-306
                if iter == 0 {
                    // Last iteration. There shall be exactly one or two end points waiting to be connected.
                    assert_eq!(queue.size(), if first_point == NULLPTR { 2 } else { 1 });
                    if first_point == NULLPTR {
                        first_point = *queue.top().unwrap();
                        queue.pop();
                        end_points.borrow_mut()[first_point].edge_out = NULLPTR;
                    }
                    last_point = *queue.top().unwrap();
                    end_points.borrow_mut()[last_point].edge_out = NULLPTR;
                    queue.pop();
                    assert!(queue.is_empty());
                    break;
                }
            } else {
                // This edge forms a loop. Update end_point1 and try another one.
                // ShortestPath.cpp:309-333
                iter += 1;
                let this_idx = end_point1;
                {
                    let mut ep = end_points.borrow_mut();
                    ep[end_point1].edge_out = NULLPTR;
                }
                let ep1_pos = end_points.borrow()[end_point1].pos;
                // Find the closest point to this end_point, which lies on a different extrusion path (filtered by the filter lambda).
                // ShortestPath.cpp:314-324
                let next_idx = {
                    let ep_rc = Rc::clone(&end_points);
                    // equivalent_chain must be mutated in the filter; use a RefCell wrapper
                    // around a raw pointer is unsafe; instead borrow via closure capturing &mut.
                    let eq = RefCell::new(&mut equivalent_chain);
                    find_closest_point(&kdtree, &ep1_pos, |idx: usize| {
                        let ep = ep_rc.borrow();
                        debug_assert!(ep[this_idx].edge_out == NULLPTR);
                        debug_assert!(ep[this_idx].chain_id == 0);
                        if (idx ^ this_idx) <= 1 || ep[idx].chain_id != 0 {
                            // Points of the same segment shall not be connected,
                            // cannot connect to an already connected point.
                            return false;
                        }
                        let c1 = ep[this_idx ^ 1].chain_id;
                        let c2 = ep[idx ^ 1].chain_id;
                        drop(ep);
                        let mut eqm = eq.borrow_mut();
                        let chain1 = eqm.equivalent(c1);
                        let chain2 = eqm.equivalent(c2);
                        chain1 != chain2 || chain1 == 0
                    })
                };
                assert!(next_idx < end_points.borrow().len());
                let next_pos = end_points.borrow()[next_idx].pos;
                {
                    let mut ep = end_points.borrow_mut();
                    ep[end_point1].edge_out = next_idx;
                    ep[end_point1].distance_out = squared_norm(&next_pos, &ep1_pos);
                }
                // Update position of this end point in the queue based on the distance calculated at the line above.
                // ShortestPath.cpp:333
                let ep1_heap = end_points.borrow()[end_point1].heap_idx;
                queue.update(ep1_heap);
            }
            iter -= 1;
        }
        assert!(queue.is_empty());

        // Now interconnect pairs of segments into a chain.
        // ShortestPath.cpp:342-389
        assert!(first_point != NULLPTR);
        out.reserve(num_segments);
        let mut failed = false;
        {
            let mut fp = first_point;
            loop {
                assert!(out.len() < num_segments);
                let first_point_id = fp;
                let segment_id = first_point_id >> 1;
                let reverse = (first_point_id & 1) != 0;
                let second_point = first_point_id ^ 1;
                if reverse_could_fail {
                    if reverse && !could_reverse_func(segment_id) {
                        failed = true;
                        break;
                    }
                } else {
                    assert!(!reverse || could_reverse_func(segment_id));
                }
                out.push((segment_id, reverse));
                fp = end_points.borrow()[second_point].edge_out;
                if fp == NULLPTR {
                    break;
                }
            }
        }
        if reverse_could_fail {
            if failed {
                if start_near.is_none() {
                    // We may try the reverse order.
                    // ShortestPath.cpp:364-382
                    out.clear();
                    let mut fp = last_point;
                    failed = false;
                    loop {
                        assert!(out.len() < num_segments);
                        let first_point_id = fp;
                        let segment_id = first_point_id >> 1;
                        let reverse = (first_point_id & 1) != 0;
                        let second_point = first_point_id ^ 1;
                        if reverse && !could_reverse_func(segment_id) {
                            failed = true;
                            break;
                        }
                        out.push((segment_id, reverse));
                        fp = end_points.borrow()[second_point].edge_out;
                        if fp == NULLPTR {
                            break;
                        }
                    }
                }
            }
            if failed {
                // As a last resort, try a dumb algorithm, which is not sensitive to edge reversal constraints.
                // ShortestPath.cpp:386
                let mut closest: Vec<EndPointClosest> = end_points
                    .borrow()
                    .iter()
                    .map(|ep| EndPointClosest {
                        pos: ep.pos,
                        chain_id: ep.chain_id,
                    })
                    .collect();
                let start = if initial_point != NULLPTR {
                    initial_point
                } else {
                    0
                };
                out = chain_segments_closest_point(
                    &mut closest,
                    &kdtree,
                    &could_reverse_func,
                    start,
                );
            }
        } else {
            assert!(!failed);
        }
    }

    // ShortestPath.cpp:392-393
    assert_eq!(out.len(), num_segments);
    out
}

// ShortestPath.cpp:975-979
fn chain_segments_greedy_constrained_reversals<SegmentEndPointFunc, CouldReverseFunc>(
    end_point_func: SegmentEndPointFunc,
    could_reverse_func: CouldReverseFunc,
    num_segments: usize,
    start_near: Option<&Point>,
) -> Vec<(usize, bool)>
where
    SegmentEndPointFunc: Fn(usize, bool) -> Point,
    CouldReverseFunc: Fn(usize) -> bool,
{
    chain_segments_greedy_constrained_reversals_(
        end_point_func,
        could_reverse_func,
        num_segments,
        start_near,
        true,
    )
}

// ShortestPath.cpp:981-986
fn chain_segments_greedy<SegmentEndPointFunc>(
    end_point_func: SegmentEndPointFunc,
    num_segments: usize,
    start_near: Option<&Point>,
) -> Vec<(usize, bool)>
where
    SegmentEndPointFunc: Fn(usize, bool) -> Point,
{
    // ShortestPath.cpp:984
    let could_reverse_func = |_idx: usize| -> bool { true };
    chain_segments_greedy_constrained_reversals_(
        end_point_func,
        could_reverse_func,
        num_segments,
        start_near,
        false,
    )
}

// Entity-level accessors mirroring the `ExtrusionEntity*` virtual dispatch the C++
// chaining lambdas use. The Rust port stores a `Vec<ExtrusionEntityType>` enum, so the
// `ee->first_point()` / `ee->last_point()` / `ee->is_loop()` / `ee->can_reverse()` /
// `ee->reverse()` calls dispatch over the enum instead of through a vtable.
fn entity_first_point(ee: &ExtrusionEntityType) -> Point {
    match ee {
        ExtrusionEntityType::Path(p) => p.first_point(),
        ExtrusionEntityType::Loop(l) => l.first_point(),
        // ExtrusionEntityCollection.hpp:105 `front()->first_point()`
        ExtrusionEntityType::Collection(c) => c.first_point().expect("first_point on empty collection"),
    }
}

fn entity_last_point(ee: &ExtrusionEntityType) -> Point {
    match ee {
        ExtrusionEntityType::Path(p) => p.last_point(),
        ExtrusionEntityType::Loop(l) => l.last_point(),
        // ExtrusionEntityCollection.hpp:106 `back()->last_point()`
        ExtrusionEntityType::Collection(c) => c.last_point().expect("last_point on empty collection"),
    }
}

fn entity_is_loop(ee: &ExtrusionEntityType) -> bool {
    // ExtrusionEntity.hpp:165 `virtual bool is_loop() const { return false; }`
    // ExtrusionEntity.hpp:506 (ExtrusionLoop) `bool is_loop() const override { return true; }`
    matches!(ee, ExtrusionEntityType::Loop(_))
}

fn entity_can_reverse(ee: &ExtrusionEntityType) -> bool {
    match ee {
        // ExtrusionEntity.hpp:166 default + ExtrusionPath::can_reverse() (m_can_reverse).
        ExtrusionEntityType::Path(p) => p.can_reverse(),
        // ExtrusionEntity.hpp:507 (ExtrusionLoop) `bool can_reverse() const override { return false; }`
        ExtrusionEntityType::Loop(_) => false,
        // ExtrusionEntityCollection.hpp:63-69
        ExtrusionEntityType::Collection(c) => c.can_reverse(),
    }
}

fn entity_reverse(ee: &mut ExtrusionEntityType) {
    match ee {
        ExtrusionEntityType::Path(p) => p.reverse(),
        ExtrusionEntityType::Loop(l) => l.reverse(),
        ExtrusionEntityType::Collection(c) => c.reverse(),
    }
}

// ShortestPath.cpp:1001-1015
// std::vector<std::pair<size_t, bool>> chain_extrusion_entities(std::vector<ExtrusionEntity*> &entities, const Point *start_near)
pub fn chain_extrusion_entities(
    entities: &[ExtrusionEntityType],
    start_near: Option<&Point>,
) -> Vec<(usize, bool)> {
    // ShortestPath.cpp:1003
    // auto segment_end_point = [&entities](size_t idx, bool first_point) -> const Point& { return first_point ? entities[idx]->first_point() : entities[idx]->last_point(); };
    let segment_end_point = |idx: usize, first_point: bool| -> Point {
        if first_point {
            entity_first_point(&entities[idx])
        } else {
            entity_last_point(&entities[idx])
        }
    };
    // ShortestPath.cpp:1004
    // auto could_reverse = [&entities](size_t idx) { const ExtrusionEntity *ee = entities[idx]; return ee->is_loop() || ee->can_reverse(); };
    let could_reverse = |idx: usize| -> bool {
        let ee = &entities[idx];
        entity_is_loop(ee) || entity_can_reverse(ee)
    };
    // ShortestPath.cpp:1005
    // std::vector<std::pair<size_t, bool>> out = chain_segments_greedy_constrained_reversals<...>(segment_end_point, could_reverse, entities.size(), start_near);
    let mut out =
        chain_segments_greedy_constrained_reversals(segment_end_point, could_reverse, entities.len(), start_near);
    // ShortestPath.cpp:1006-1013
    // for (std::pair<size_t, bool> &segment : out) {
    //     ExtrusionEntity *ee = entities[segment.first];
    //     if (ee->is_loop())
    //         // Ignore reversals for loops, as the start point equals the end point.
    //         segment.second = false;
    //     // Is can_reverse() respected by the reversals?
    //     assert(ee->can_reverse() || ! segment.second);
    // }
    for segment in &mut out {
        let ee = &entities[segment.0];
        if entity_is_loop(ee) {
            // Ignore reversals for loops, as the start point equals the end point.
            segment.1 = false;
        }
        // Is can_reverse() respected by the reversals?
        debug_assert!(entity_can_reverse(ee) || !segment.1);
    }
    // ShortestPath.cpp:1014
    // return out;
    out
}

// ShortestPath.cpp:1017-1029
// void reorder_extrusion_entities(std::vector<ExtrusionEntity*> &entities, const std::vector<std::pair<size_t, bool>> &chain)
pub fn reorder_extrusion_entities(
    entities: &mut Vec<ExtrusionEntityType>,
    chain: &[(usize, bool)],
) {
    // ShortestPath.cpp:1019
    assert_eq!(entities.len(), chain.len());
    // ShortestPath.cpp:1020-1021
    let mut out: Vec<ExtrusionEntityType> = Vec::with_capacity(entities.len());
    // ShortestPath.cpp:1022-1027
    // for (const std::pair<size_t, bool> &idx : chain) {
    //     assert(entities[idx.first] != nullptr);
    //     out.emplace_back(entities[idx.first]);
    //     if (idx.second)
    //         out.back()->reverse();
    // }
    // C++ transplants the owned pointer into `out`; here we move the owned entity out of
    // `entities`. The chain is a permutation, so each source slot is consumed exactly once.
    let mut src: Vec<Option<ExtrusionEntityType>> =
        std::mem::take(entities).into_iter().map(Some).collect();
    for idx in chain {
        let mut ee = src[idx.0].take().expect("chain index moved twice");
        if idx.1 {
            entity_reverse(&mut ee);
        }
        out.push(ee);
    }
    // ShortestPath.cpp:1028
    // entities.swap(out);
    *entities = out;
}

// ShortestPath.cpp:1031-1037
// void chain_and_reorder_extrusion_entities(std::vector<ExtrusionEntity*> &entities, const Point *start_near)
pub fn chain_and_reorder_extrusion_entities(
    entities: &mut Vec<ExtrusionEntityType>,
    start_near: Option<&Point>,
) {
    // ShortestPath.cpp:1033-1035
    // this function crashes if there are empty elements in entities
    // entities.erase(std::remove_if(entities.begin(), entities.end(), [](ExtrusionEntity *entity) {
    //     return static_cast<ExtrusionEntityCollection *>(entity)->empty(); }), entities.end());
    //
    // NOTE: the C++ unconditionally `static_cast`s every entity to ExtrusionEntityCollection*
    // and tests `empty()`. That is only well-defined when the entities are in fact collections
    // (the documented use, e.g. chaining a vector of region collections). We mirror the intent:
    // drop entities that are empty collections.
    entities.retain(|entity| match entity {
        ExtrusionEntityType::Collection(c) => !c.is_empty(),
        // Non-collection entities are not erased (the C++ cast would be UB; in practice the
        // caller only passes collections here).
        _ => true,
    });
    // ShortestPath.cpp:1036
    // reorder_extrusion_entities(entities, chain_extrusion_entities(entities, start_near));
    let chain = chain_extrusion_entities(entities, start_near);
    reorder_extrusion_entities(entities, &chain);
}

// ShortestPath.cpp:1039-1043
pub fn chain_extrusion_paths(
    extrusion_paths: &[ExtrusionPath],
    start_near: Option<&Point>,
) -> Vec<(usize, bool)> {
    // ShortestPath.cpp:1041
    let segment_end_point = |idx: usize, first_point: bool| -> Point {
        if first_point {
            extrusion_paths[idx].first_point()
        } else {
            extrusion_paths[idx].last_point()
        }
    };
    // ShortestPath.cpp:1042
    chain_segments_greedy(segment_end_point, extrusion_paths.len(), start_near)
}

// ShortestPath.cpp:1045-1056
pub fn reorder_extrusion_paths(extrusion_paths: &mut Vec<ExtrusionPath>, chain: &[(usize, bool)]) {
    // ShortestPath.cpp:1047
    assert_eq!(extrusion_paths.len(), chain.len());
    // ShortestPath.cpp:1048-1049
    let mut out: Vec<ExtrusionPath> = Vec::with_capacity(extrusion_paths.len());
    // ShortestPath.cpp:1050-1054
    // C++ moves the paths out of `extrusion_paths[idx.first]` (std::move). As
    // ExtrusionPath has no Default, take ownership through an Option slot vector,
    // which mirrors the move-out semantics (each source is consumed exactly once;
    // the C++ chain is a permutation, so every index is moved at most once).
    let mut src: Vec<Option<ExtrusionPath>> =
        std::mem::take(extrusion_paths).into_iter().map(Some).collect();
    for idx in chain {
        let mut path = src[idx.0].take().expect("chain index moved twice");
        if idx.1 {
            path.reverse();
        }
        out.push(path);
    }
    // ShortestPath.cpp:1055
    *extrusion_paths = out;
}

// ShortestPath.cpp:1058-1061
pub fn chain_and_reorder_extrusion_paths(
    extrusion_paths: &mut Vec<ExtrusionPath>,
    start_near: Option<&Point>,
) {
    let chain = chain_extrusion_paths(extrusion_paths, start_near);
    reorder_extrusion_paths(extrusion_paths, &chain);
}

// ShortestPath.cpp:1063-1071
pub fn chain_expolygons(input_exploy: &ExPolygons) -> Vec<usize> {
    // ShortestPath.cpp:1064
    let mut points: Vec<Point> = Vec::new();
    // ShortestPath.cpp:1065-1069
    for exploy in input_exploy.iter() {
        // BoundingBox bbox; bbox = get_extents(exploy); points.push_back(bbox.center());
        let bbox = get_extents_expoly(exploy);
        points.push(bbox.center());
    }
    // ShortestPath.cpp:1070
    chain_points(&points, None)
}

// ShortestPath.cpp:1073-1082
pub fn chain_points(points: &[Point], start_near: Option<&Point>) -> Vec<usize> {
    // ShortestPath.cpp:1075
    let segment_end_point = |idx: usize, _first_point: bool| -> Point { points[idx] };
    // ShortestPath.cpp:1076
    let ordered = chain_segments_greedy(segment_end_point, points.len(), start_near);
    // ShortestPath.cpp:1077-1081
    let mut out: Vec<usize> = Vec::with_capacity(ordered.len());
    for segment_and_reversal in &ordered {
        out.push(segment_and_reversal.0);
    }
    out
}

// ShortestPath.cpp:1272-1278
#[derive(Clone)]
struct FlipEdge {
    p1: [f64; 2],
    p2: [f64; 2],
    source_index: usize,
}

impl FlipEdge {
    // ShortestPath.cpp:1273
    fn new(p1: [f64; 2], p2: [f64; 2], source_index: usize) -> Self {
        FlipEdge {
            p1,
            p2,
            source_index,
        }
    }
    // ShortestPath.cpp:1274
    fn flip(&mut self) {
        std::mem::swap(&mut self.p1, &mut self.p2);
    }
}

// ShortestPath.cpp:1280-1287
#[derive(Clone, Copy)]
struct ConnectionCost {
    cost: f64,
    cost_flipped: f64,
}

impl ConnectionCost {
    // ShortestPath.cpp:1282 (default)
    fn default_() -> Self {
        ConnectionCost {
            cost: 0.,
            cost_flipped: 0.,
        }
    }
    // ShortestPath.cpp:1287 operator-
    fn sub(&self, rhs: &ConnectionCost) -> ConnectionCost {
        ConnectionCost {
            cost: self.cost - rhs.cost,
            cost_flipped: self.cost_flipped - rhs.cost_flipped,
        }
    }
}

// ShortestPath.cpp:1289-1353
fn minimum_crossover_cost(
    edges: &[FlipEdge],
    span1: (usize, usize),
    cost1: &ConnectionCost,
    span2: (usize, usize),
    cost2: &ConnectionCost,
    span3: (usize, usize),
    cost3: &ConnectionCost,
    cost_current: f64,
) -> (f64, usize) {
    // ShortestPath.cpp:1296-1322
    let connection_cost = |span1: (usize, usize),
                           cost1: &ConnectionCost,
                           reversed1: bool,
                           flipped1: bool,
                           span2: (usize, usize),
                           cost2: &ConnectionCost,
                           reversed2: bool,
                           flipped2: bool,
                           span3: (usize, usize),
                           cost3: &ConnectionCost,
                           reversed3: bool,
                           flipped3: bool|
     -> f64 {
        // ShortestPath.cpp:1300
        let first_point = |span: (usize, usize), flipped: bool| -> [f64; 2] {
            if flipped {
                edges[span.0].p2
            } else {
                edges[span.0].p1
            }
        };
        // ShortestPath.cpp:1301
        let last_point = |span: (usize, usize), flipped: bool| -> [f64; 2] {
            if flipped {
                edges[span.1 - 1].p1
            } else {
                edges[span.1 - 1].p2
            }
        };
        // ShortestPath.cpp:1302
        let point = |span: (usize, usize), start: bool, flipped: bool| -> [f64; 2] {
            if start {
                first_point(span, flipped)
            } else {
                last_point(span, flipped)
            }
        };
        // ShortestPath.cpp:1303-1306
        let cost = |acost: &ConnectionCost, flipped: bool| -> f64 {
            if flipped {
                acost.cost_flipped
            } else {
                acost.cost
            }
        };
        // Ignore reversed single segment spans.
        // ShortestPath.cpp:1308-1310
        let simple_span_ignore =
            |span: (usize, usize), reversed: bool| -> bool { span.0 + 1 == span.1 && reversed };
        // ShortestPath.cpp:1311-1313
        debug_assert!(span1.0 < span1.1);
        debug_assert!(span2.0 < span2.1);
        debug_assert!(span3.0 < span3.1);
        // ShortestPath.cpp:1314-1321
        if simple_span_ignore(span1, reversed1)
            || simple_span_ignore(span2, reversed2)
            || simple_span_ignore(span3, reversed3)
        {
            // Don't perform unnecessary calculations simulating reversion of single segment spans.
            f64::MAX
        } else {
            // Calculate the cost of reverting chains and / or flipping segment orientations.
            cost(cost1, flipped1)
                + cost(cost2, flipped2)
                + cost(cost3, flipped3)
                + norm(
                    &point(span2, !reversed2, flipped2),
                    &point(span1, reversed1, flipped1),
                )
                + norm(
                    &point(span3, !reversed3, flipped3),
                    &point(span2, reversed2, flipped2),
                )
        }
    };

    // ShortestPath.cpp:1331-1332
    let mut cost_min = cost_current;
    let mut flip_min: usize = 0; // no flip, no improvement
                                 // ShortestPath.cpp:1333-1351
    for i in 0..(1usize << 6) {
        // From the three combinations of 1,2,3 ordering, the other three are reversals of the first three.
        let c1 = if i == 0 {
            cost_current
        } else {
            connection_cost(
                span1,
                cost1,
                (i & 1) != 0,
                (i & (1 << 1)) != 0,
                span2,
                cost2,
                (i & (1 << 2)) != 0,
                (i & (1 << 3)) != 0,
                span3,
                cost3,
                (i & (1 << 4)) != 0,
                (i & (1 << 5)) != 0,
            )
        };
        let c2 = connection_cost(
            span1,
            cost1,
            (i & 1) != 0,
            (i & (1 << 1)) != 0,
            span3,
            cost3,
            (i & (1 << 2)) != 0,
            (i & (1 << 3)) != 0,
            span2,
            cost2,
            (i & (1 << 4)) != 0,
            (i & (1 << 5)) != 0,
        );
        let c3 = connection_cost(
            span2,
            cost2,
            (i & 1) != 0,
            (i & (1 << 1)) != 0,
            span1,
            cost1,
            (i & (1 << 2)) != 0,
            (i & (1 << 3)) != 0,
            span3,
            cost3,
            (i & (1 << 4)) != 0,
            (i & (1 << 5)) != 0,
        );
        if c1 < cost_min {
            cost_min = c1;
            flip_min = i;
        }
        if c2 < cost_min {
            cost_min = c2;
            flip_min = i + (1 << 6);
        }
        if c3 < cost_min {
            cost_min = c3;
            flip_min = i + (2 << 6);
        }
    }
    // ShortestPath.cpp:1352
    (cost_min, flip_min)
}

// ShortestPath.cpp:1433-1472
fn do_crossover(
    edges_in: &[FlipEdge],
    edges_out: &mut [FlipEdge],
    span1: (usize, usize),
    span2: (usize, usize),
    span3: (usize, usize),
    i: usize,
) {
    // ShortestPath.cpp:1437
    assert_eq!(edges_in.len(), edges_out.len());
    // ShortestPath.cpp:1438-1459
    // do_it writes the three spans (optionally reversed and/or flipped) into edges_out.
    let do_it = |edges_out: &mut [FlipEdge],
                 s1: (usize, usize),
                 r1: bool,
                 f1: bool,
                 s2: (usize, usize),
                 r2: bool,
                 f2: bool,
                 s3: (usize, usize),
                 r3: bool,
                 f3: bool| {
        // it_edges_out tracks the write cursor (edges_out.begin()).
        let mut it_edges_out: usize = 0;
        // ShortestPath.cpp:1443-1455
        let mut copy_span = |edges_out: &mut [FlipEdge], span: (usize, usize), reversed: bool, flipped: bool| {
            debug_assert!(span.0 < span.1);
            let it = it_edges_out;
            if reversed {
                // std::reverse_copy
                let mut w = it_edges_out;
                for s in (span.0..span.1).rev() {
                    edges_out[w] = edges_in[s].clone();
                    w += 1;
                }
            } else {
                // std::copy
                let mut w = it_edges_out;
                for s in span.0..span.1 {
                    edges_out[w] = edges_in[s].clone();
                    w += 1;
                }
            }
            it_edges_out += span.1 - span.0;
            if reversed != flipped {
                for e in edges_out.iter_mut().take(it_edges_out).skip(it) {
                    e.flip();
                }
            }
        };
        copy_span(edges_out, s1, r1, f1);
        copy_span(edges_out, s2, r2, f2);
        copy_span(edges_out, s3, r3, f3);
    };
    // ShortestPath.cpp:1460-1470
    match i >> 6 {
        0 => do_it(
            edges_out,
            span1,
            (i & 1) != 0,
            (i & (1 << 1)) != 0,
            span2,
            (i & (1 << 2)) != 0,
            (i & (1 << 3)) != 0,
            span3,
            (i & (1 << 4)) != 0,
            (i & (1 << 5)) != 0,
        ),
        1 => do_it(
            edges_out,
            span1,
            (i & 1) != 0,
            (i & (1 << 1)) != 0,
            span3,
            (i & (1 << 2)) != 0,
            (i & (1 << 3)) != 0,
            span2,
            (i & (1 << 4)) != 0,
            (i & (1 << 5)) != 0,
        ),
        _ => {
            assert_eq!(i >> 6, 2);
            do_it(
                edges_out,
                span2,
                (i & 1) != 0,
                (i & (1 << 1)) != 0,
                span1,
                (i & (1 << 2)) != 0,
                (i & (1 << 3)) != 0,
                span3,
                (i & (1 << 4)) != 0,
                (i & (1 << 5)) != 0,
            );
        }
    }
    // ShortestPath.cpp:1471
    assert_eq!(edges_in.len(), edges_out.len());
}

// Worst time complexity:    O(min(n, 100) * (n * log n + n^2)
// ShortestPath.cpp:1553-1625
fn reorder_by_two_exchanges_with_segment_flipping(edges: &mut Vec<FlipEdge>) {
    // ShortestPath.cpp:1555-1556
    if edges.len() < 2 {
        return;
    }

    // ShortestPath.cpp:1558-1562
    let mut connections: Vec<ConnectionCost> =
        vec![ConnectionCost::default_(); edges.len()];
    let mut edges_tmp: Vec<FlipEdge> = edges.clone();
    let mut connection_lengths: Vec<(f64, usize)> = vec![(0., 0); edges.len() - 1];
    let mut connection_tried: Vec<bool> = vec![false; edges.len()];
    let max_iterations = std::cmp::min(edges.len(), 100usize);
    // ShortestPath.cpp:1563
    for _iter in 0..max_iterations {
        // Initialize connection costs and connection lengths.
        // ShortestPath.cpp:1565-1574
        for i in 1..edges.len() {
            let (e1, e2) = (&edges[i - 1], &edges[i]);
            let prev = connections[i - 1];
            let mut c = prev;
            let l = norm(&e2.p1, &e1.p2);
            c.cost += l;
            c.cost_flipped += norm(&e2.p2, &e1.p1);
            connections[i] = c;
            connection_lengths[i - 1] = (l, i);
        }
        // ShortestPath.cpp:1575
        connection_lengths.sort_by(|l, r| r.0.partial_cmp(&l.0).unwrap());
        // ShortestPath.cpp:1576
        for c in connection_tried.iter_mut() {
            *c = false;
        }
        // ShortestPath.cpp:1577-1579
        let mut crossover1_pos_final: usize = SIZE_MAX;
        let mut crossover2_pos_final: usize = SIZE_MAX;
        let mut crossover_flip_final: usize = 0;
        // ShortestPath.cpp:1580-1612
        for first_crossover_candidate in connection_lengths.iter() {
            let longest_connection_idx = first_crossover_candidate.1;
            connection_tried[longest_connection_idx] = true;
            // Find the second crossover connection with the lowest total chain cost.
            // ShortestPath.cpp:1584-1586
            let mut crossover_pos_min: usize = SIZE_MAX;
            let mut crossover_cost_min: f64 = connections[connections.len() - 1].cost;
            let mut crossover_flip_min: usize = 0;
            // ShortestPath.cpp:1587-1602
            for j in 1..connections.len() {
                if !connection_tried[j] {
                    let mut a = j;
                    let mut b = longest_connection_idx;
                    if a > b {
                        std::mem::swap(&mut a, &mut b);
                    }
                    let back = connections[connections.len() - 1];
                    let cost_and_flip = minimum_crossover_cost(
                        edges,
                        (0, a),
                        &connections[a - 1],
                        (a, b),
                        &connections[b - 1].sub(&connections[a]),
                        (b, edges.len()),
                        &back.sub(&connections[b]),
                        back.cost,
                    );
                    if cost_and_flip.1 > 0 && cost_and_flip.0 < crossover_cost_min {
                        crossover_pos_min = j;
                        crossover_cost_min = cost_and_flip.0;
                        crossover_flip_min = cost_and_flip.1;
                    }
                }
            }
            // ShortestPath.cpp:1603-1611
            if crossover_cost_min < connections[connections.len() - 1].cost {
                // The cost of the chain with the proposed two crossovers has a lower total cost than the current chain. Apply the crossover.
                crossover1_pos_final = longest_connection_idx;
                crossover2_pos_final = crossover_pos_min;
                crossover_flip_final = crossover_flip_min;
                break;
            } else {
                // Continue with another long candidate edge.
            }
        }
        // ShortestPath.cpp:1613-1623
        if crossover_flip_final > 0 {
            // Pair of cross over positions and flip / reverse constellation has been found, which improves the total cost of the connection.
            // Perform a crossover.
            if crossover1_pos_final > crossover2_pos_final {
                std::mem::swap(&mut crossover1_pos_final, &mut crossover2_pos_final);
            }
            do_crossover(
                edges,
                &mut edges_tmp,
                (0, crossover1_pos_final),
                (crossover1_pos_final, crossover2_pos_final),
                (crossover2_pos_final, edges.len()),
                crossover_flip_final,
            );
            std::mem::swap(edges, &mut edges_tmp);
        } else {
            // No valid pair of cross over positions was found improving the total cost. Giving up.
            break;
        }
    }
}

// Flip the sequences of polylines to lower the total length of connecting lines.
// Used by the infill generator if the infill is not connected with perimeter lines
// and to order the brim lines.
// ShortestPath.cpp:1882-1931
fn improve_ordering_by_two_exchanges_with_segment_flipping(polylines: &mut Polylines, _fixed_start: bool) {
    // ShortestPath.cpp:1900-1903
    let mut edges: Vec<FlipEdge> = Vec::with_capacity(polylines.len());
    for (i, pl) in polylines.iter().enumerate() {
        edges.push(FlipEdge::new(
            pt_to_vec2d(&pl.first_point()),
            pt_to_vec2d(&pl.last_point()),
            i,
        ));
    }
    // ShortestPath.cpp:1904-1905
    reorder_by_two_exchanges_with_segment_flipping(&mut edges);
    // ShortestPath.cpp:1910-1922
    let mut out: Polylines = Vec::with_capacity(polylines.len());
    for edge in &edges {
        let pl = &polylines[edge.source_index];
        out.push(pl.clone());
        if edge.p2 == pt_to_vec2d(&pl.first_point()) {
            // Polyline is flipped.
            out.last_mut().unwrap().reverse();
        } else {
            // Polyline is not flipped.
            debug_assert!(edge.p1 == pt_to_vec2d(&pl.first_point()));
        }
    }
    // ShortestPath.cpp:1931 (write back)
    std::mem::swap(polylines, &mut out);
}

// Used to optimize order of infill lines and brim lines.
// ShortestPath.cpp:1934-1962
pub fn chain_polylines(mut polylines: Polylines, start_near: Option<&Point>) -> Polylines {
    // ShortestPath.cpp:1942
    let mut out: Polylines = Vec::new();
    // ShortestPath.cpp:1943
    if !polylines.is_empty() {
        // ShortestPath.cpp:1944
        let segment_end_point = {
            let pls = &polylines;
            move |idx: usize, first_point: bool| -> Point {
                if first_point {
                    pls[idx].first_point()
                } else {
                    pls[idx].last_point()
                }
            }
        };
        // ShortestPath.cpp:1945
        // chain_segments_greedy2 (the "2" variant) is selected; see note below.
        let ordered = chain_segments_greedy(&segment_end_point, polylines.len(), start_near);
        drop(segment_end_point);
        // ShortestPath.cpp:1946-1951
        out.reserve(polylines.len());
        for segment_and_reversal in &ordered {
            // std::move(polylines[segment_and_reversal.first])
            out.push(std::mem::take(&mut polylines[segment_and_reversal.0]));
            if segment_and_reversal.1 {
                out.last_mut().unwrap().reverse();
            }
        }
        // ShortestPath.cpp:1952-1955
        if out.len() > 1 && start_near.is_none() {
            improve_ordering_by_two_exchanges_with_segment_flipping(&mut out, start_near.is_some());
        }
    }
    // ShortestPath.cpp:1961
    out
}

// ShortestPath.hpp:27 (overload taking const &)
#[inline]
pub fn chain_polylines_ref(src: &Polylines, start_near: Option<&Point>) -> Polylines {
    let tmp: Polylines = src.clone();
    chain_polylines(tmp, start_near)
}

// ShortestPath.hpp:28-40
// template<typename T> inline void reorder_by_shortest_traverse(std::vector<T> &polylines_out)
//
// The C++ template only requires that `T` exposes `T::points` with a `.front()`.
// In Rust this is modelled by the `ReorderByShortestTraverse` trait, which yields
// the first point of the polyline-like `T`. Implemented for `Polyline` and
// `ThickPolyline` (the two instantiations used in libslic3r).
pub trait ReorderByShortestTraverse {
    /// `contour.points.front()` — first point of the polyline-like contour.
    fn front_point(&self) -> Point;
}

impl ReorderByShortestTraverse for Polyline {
    #[inline]
    fn front_point(&self) -> Point {
        self.points[0]
    }
}

impl ReorderByShortestTraverse for crate::geometry::ThickPolyline {
    #[inline]
    fn front_point(&self) -> Point {
        self.points[0]
    }
}

pub fn reorder_by_shortest_traverse<T: ReorderByShortestTraverse + Default>(
    polylines_out: &mut Vec<T>,
) {
    // ShortestPath.hpp:30-32
    let mut start_point: Vec<Point> = Vec::with_capacity(polylines_out.len());
    for contour in polylines_out.iter() {
        start_point.push(contour.front_point());
    }
    // ShortestPath.hpp:34
    let order = chain_points(&start_point, None);
    // ShortestPath.hpp:36-39
    let mut temp = std::mem::take(polylines_out);
    let mut result: Vec<T> = Vec::with_capacity(temp.len());
    for i in order {
        result.push(std::mem::take(&mut temp[i]));
    }
    *polylines_out = result;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Point, Polyline};

    // Runtime smoke test: exercises the greedy chaining (priority queue +
    // RefCell-borrowing closures + kd-tree) to ensure no borrow panics and a
    // valid permutation is produced.
    #[test]
    fn chain_points_smoke() {
        let pts = vec![
            Point::new(0, 0),
            Point::new(1_000_000, 0),
            Point::new(2_000_000, 0),
            Point::new(2_000_000, 1_000_000),
            Point::new(0, 1_000_000),
        ];
        let order = chain_points(&pts, None);
        assert_eq!(order.len(), pts.len());
        let mut sorted = order.clone();
        sorted.sort_unstable();
        assert_eq!(sorted, vec![0, 1, 2, 3, 4]);
    }

    #[test]
    fn chain_points_with_start_near() {
        let pts = vec![
            Point::new(0, 0),
            Point::new(5_000_000, 0),
            Point::new(10_000_000, 0),
        ];
        let start = Point::new(10_000_000, 0);
        let order = chain_points(&pts, Some(&start));
        assert_eq!(order.len(), 3);
        // First visited must be the one closest to start_near.
        assert_eq!(order[0], 2);
    }

    #[test]
    fn chain_polylines_smoke() {
        let mut pls: Polylines = Vec::new();
        for i in 0..6i64 {
            let mut pl = Polyline::default();
            pl.points.push(Point::new(i * 1_000_000, 0));
            pl.points.push(Point::new(i * 1_000_000, 500_000));
            pls.push(pl);
        }
        let out = chain_polylines(pls, None);
        assert_eq!(out.len(), 6);
    }
}
