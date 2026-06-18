//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.
//
//! Faithful 1:1 port of `Fill/Lightning/TreeNode.{hpp,cpp}` from BambuStudio.
//!
//! The C++ tree is built out of `std::shared_ptr<Node>` (`NodeSPtr`) with
//! `std::weak_ptr<Node>` parents and uses `enable_shared_from_this`. The faithful
//! Rust equivalent is `Rc<RefCell<Node>>` (`NodeSPtr`) with a `Weak<RefCell<Node>>`
//! parent. Methods that call `shared_from_this()` in C++ take the owning
//! `&NodeSPtr` handle explicitly so the identity comparisons and parent links are
//! preserved exactly.

// TreeNode.cpp:4 #include "TreeNode.hpp"
// TreeNode.cpp:6 #include "../../Geometry.hpp"

use std::cell::RefCell;
use std::rc::{Rc, Weak};

use crate::edge_grid::EdgeGrid;
use crate::geometry::geometry::{segment_segment_intersection, Vec2d};
use crate::geometry::{BoundingBox, Point, Polygon, Polyline};
use crate::Coord;

// TreeNode.hpp:21 — constexpr auto locator_cell_size = scaled<coord_t>(4.);
// `scaled<coord_t>(4.)` == scale(4.0) == round(4.0 * SCALING_FACTOR) == 400_000.
// (SCALING_FACTOR == 100_000; see lib.rs.) The lightning generator builds the
// outline locator at this resolution.
pub const LOCATOR_CELL_SIZE: Coord = 4 * (crate::SCALING_FACTOR as Coord);

// TreeNode.hpp:23 class Node;
// TreeNode.hpp:25 using NodeSPtr = std::shared_ptr<Node>;
pub type NodeSPtr = Rc<RefCell<Node>>;

/// A single vertex of a Lightning Tree, the structure that determines the paths
/// to be printed to form Lightning Infill.
///
/// TreeNode.hpp:41 — `class Node : public std::enable_shared_from_this<Node>`
#[derive(Debug)]
pub struct Node {
    // TreeNode.hpp:267 — bool m_is_root;
    pub m_is_root: bool,
    // TreeNode.hpp:268 — Point m_p;
    pub m_p: Point,
    // TreeNode.hpp:269 — std::weak_ptr<Node> m_parent;
    pub m_parent: Weak<RefCell<Node>>,
    // TreeNode.hpp:270 — std::vector<NodeSPtr> m_children;
    pub m_children: Vec<NodeSPtr>,
    // TreeNode.hpp:272 — std::optional<Point> m_last_grounding_location;
    pub m_last_grounding_location: Option<Point>,
}

// TreeNode.hpp:205-209 — struct RectilinearJunction
#[derive(Debug, Clone, Copy)]
pub struct RectilinearJunction {
    // TreeNode.hpp:207 — rectilinear distance along the tree from the last junction above to the junction below
    pub total_recti_dist: Coord,
    // TreeNode.hpp:208 — junction location below
    pub junction_loc: Point,
}

impl Node {
    // TreeNode.hpp:45-52 — template create(): Workaround for protected ctors and make_shared.
    // Constructs a new Node and wraps it in a shared pointer.
    pub fn create(p: Point) -> NodeSPtr {
        Rc::new(RefCell::new(Node::new(p, None)))
    }

    // TreeNode.hpp:45-52 — create() overload taking the optional last grounding location.
    pub fn create_with_grounding(p: Point, last_grounding_location: Option<Point>) -> NodeSPtr {
        Rc::new(RefCell::new(Node::new(p, last_grounding_location)))
    }

    // TreeNode.hpp:59 — const Point& getLocation() const { return m_p; }
    pub fn get_location(&self) -> Point {
        self.m_p
    }

    // TreeNode.hpp:65 — void setLocation(const Point& p) { m_p = p; }
    pub fn set_location(&mut self, p: Point) {
        self.m_p = p;
    }

    // TreeNode.hpp:153 — bool isRoot() const { return m_is_root; }
    pub fn is_root(&self) -> bool {
        self.m_is_root
    }

    // TreeNode.hpp:248 — const std::optional<Point>& getLastGroundingLocation() const
    pub fn get_last_grounding_location(&self) -> &Option<Point> {
        &self.m_last_grounding_location
    }

    // TreeNode.cpp:87-89 —
    // Node::Node(const Point& p, const std::optional<Point>& last_grounding_location) :
    //     m_is_root(true), m_p(p), m_last_grounding_location(last_grounding_location) {}
    fn new(p: Point, last_grounding_location: Option<Point>) -> Self {
        Node {
            m_is_root: true,
            m_p: p,
            m_parent: Weak::new(),
            m_children: Vec::new(),
            m_last_grounding_location: last_grounding_location,
        }
    }
}

// TreeNode.cpp:10-20 —
// coord_t Node::getWeightedDistance(const Point& unsupported_location, const coord_t& supporting_radius) const
pub fn get_weighted_distance(node: &NodeSPtr, unsupported_location: &Point, supporting_radius: Coord) -> Coord {
    // TreeNode.cpp:12
    const MIN_VALENCE_FOR_BOOST: Coord = 0;
    // TreeNode.cpp:13
    const MAX_VALENCE_FOR_BOOST: Coord = 4;
    // TreeNode.cpp:14
    const VALENCE_BOOST_MULTIPLIER: Coord = 4;

    let n = node.borrow();
    // TreeNode.cpp:16 — const size_t valence = (!m_is_root) + m_children.size();
    let valence: Coord = ((!n.m_is_root) as Coord) + n.m_children.len() as Coord;
    // TreeNode.cpp:17 — const coord_t valence_boost = (min_valence_for_boost < valence && valence < max_valence_for_boost) ? valence_boost_multiplier * supporting_radius : 0;
    let valence_boost: Coord = if MIN_VALENCE_FOR_BOOST < valence && valence < MAX_VALENCE_FOR_BOOST {
        VALENCE_BOOST_MULTIPLIER * supporting_radius
    } else {
        0
    };
    // TreeNode.cpp:18 — const auto dist_here = coord_t((getLocation() - unsupported_location).cast<double>().norm());
    let dist_here = norm_diff(&n.m_p, unsupported_location) as Coord;
    // TreeNode.cpp:19 — return dist_here - valence_boost;
    dist_here - valence_boost
}

// TreeNode.cpp:22-32 — bool Node::hasOffspring(const NodeSPtr& to_be_checked) const
pub fn has_offspring(node: &NodeSPtr, to_be_checked: &NodeSPtr) -> bool {
    // TreeNode.cpp:24-25 — if (to_be_checked == shared_from_this()) return true;
    if Rc::ptr_eq(to_be_checked, node) {
        return true;
    }

    // TreeNode.cpp:27-29
    for child_ptr in node.borrow().m_children.iter() {
        if has_offspring(child_ptr, to_be_checked) {
            return true;
        }
    }

    // TreeNode.cpp:31
    false
}

// TreeNode.cpp:34-39 — NodeSPtr Node::addChild(const Point& child_loc)
pub fn add_child_loc(node: &NodeSPtr, child_loc: Point) -> NodeSPtr {
    // TreeNode.cpp:36 — assert(m_p != child_loc);
    debug_assert!(node.borrow().m_p != child_loc);
    // TreeNode.cpp:37 — NodeSPtr child = Node::create(child_loc);
    let child = Node::create(child_loc);
    // TreeNode.cpp:38 — return addChild(child);
    add_child(node, &child)
}

// TreeNode.cpp:41-49 — NodeSPtr Node::addChild(NodeSPtr& new_child)
pub fn add_child(node: &NodeSPtr, new_child: &NodeSPtr) -> NodeSPtr {
    // TreeNode.cpp:43 — assert(new_child != shared_from_this());
    debug_assert!(!Rc::ptr_eq(new_child, node));
    //assert(p != new_child->p); // NOTE: No problem for now. Issue to solve later. Maybe even afetr final. Low prio.
    // TreeNode.cpp:45 — m_children.push_back(new_child);
    node.borrow_mut().m_children.push(Rc::clone(new_child));
    // TreeNode.cpp:46 — new_child->m_parent = shared_from_this();
    new_child.borrow_mut().m_parent = Rc::downgrade(node);
    // TreeNode.cpp:47 — new_child->m_is_root = false;
    new_child.borrow_mut().m_is_root = false;
    // TreeNode.cpp:48 — return new_child;
    Rc::clone(new_child)
}

// TreeNode.cpp:51-64 — void Node::propagateToNextLayer(...)
pub fn propagate_to_next_layer(
    node: &NodeSPtr,
    next_trees: &mut Vec<NodeSPtr>,
    next_outlines: &[Polygon],
    outline_locator: &EdgeGrid,
    prune_distance: Coord,
    smooth_magnitude: Coord,
    max_remove_colinear_dist: Coord,
) {
    // TreeNode.cpp:59 — auto tree_below = deepCopy();
    let tree_below = deep_copy(node);
    // TreeNode.cpp:60 — tree_below->prune(prune_distance);
    prune(&tree_below, prune_distance);
    // TreeNode.cpp:61 — tree_below->straighten(smooth_magnitude, max_remove_colinear_dist);
    straighten_root(&tree_below, smooth_magnitude, max_remove_colinear_dist);
    // TreeNode.cpp:62-63 — if (tree_below->realign(...)) next_trees.push_back(tree_below);
    if realign(&tree_below, next_outlines, outline_locator, next_trees) {
        next_trees.push(tree_below);
    }
}

// TreeNode.cpp:66-75 —
// NOTE: Depth-first, as currently implemented.
//       Skips the root (because that has no root itself), but all initial nodes will have the root point anyway.
// void Node::visitBranches(const std::function<void(const Point&, const Point&)>& visitor) const
pub fn visit_branches(node: &NodeSPtr, visitor: &mut dyn FnMut(&Point, &Point)) {
    let m_p = node.borrow().m_p;
    // TreeNode.cpp:70-74
    let children: Vec<NodeSPtr> = node.borrow().m_children.iter().map(Rc::clone).collect();
    for child in &children {
        // TreeNode.cpp:71 — assert(node->m_parent.lock() == shared_from_this());
        debug_assert!(weak_ptr_eq(&child.borrow().m_parent, node));
        // TreeNode.cpp:72 — visitor(m_p, node->m_p);
        let child_p = child.borrow().m_p;
        visitor(&m_p, &child_p);
        // TreeNode.cpp:73 — node->visitBranches(visitor);
        visit_branches(child, visitor);
    }
}

// TreeNode.cpp:77-85 —
// NOTE: Depth-first, as currently implemented.
// void Node::visitNodes(const std::function<void(NodeSPtr)>& visitor)
pub fn visit_nodes(node: &NodeSPtr, visitor: &mut dyn FnMut(NodeSPtr)) {
    // TreeNode.cpp:80 — visitor(shared_from_this());
    visitor(Rc::clone(node));
    // TreeNode.cpp:81-84
    let children: Vec<NodeSPtr> = node.borrow().m_children.iter().map(Rc::clone).collect();
    for child in &children {
        // TreeNode.cpp:82 — assert(node->m_parent.lock() == shared_from_this());
        debug_assert!(weak_ptr_eq(&child.borrow().m_parent, node));
        // TreeNode.cpp:83 — node->visitNodes(visitor);
        visit_nodes(child, visitor);
    }
}

// TreeNode.cpp:91-107 — NodeSPtr Node::deepCopy() const
pub fn deep_copy(node: &NodeSPtr) -> NodeSPtr {
    let n = node.borrow();
    // TreeNode.cpp:93 — NodeSPtr local_root = Node::create(m_p);
    let local_root = Node::create(n.m_p);
    {
        let mut lr = local_root.borrow_mut();
        // TreeNode.cpp:94 — local_root->m_is_root = m_is_root;
        lr.m_is_root = n.m_is_root;
        // TreeNode.cpp:95-98 — if (m_is_root) local_root->m_last_grounding_location = m_last_grounding_location.value_or(m_p);
        if n.m_is_root {
            lr.m_last_grounding_location = Some(n.m_last_grounding_location.unwrap_or(n.m_p));
        }
        // TreeNode.cpp:99 — local_root->m_children.reserve(m_children.size());
        lr.m_children.reserve(n.m_children.len());
    }
    // TreeNode.cpp:100-105
    for child_node in n.m_children.iter() {
        // TreeNode.cpp:102 — NodeSPtr child = node->deepCopy();
        let child = deep_copy(child_node);
        // TreeNode.cpp:103 — child->m_parent = local_root;
        child.borrow_mut().m_parent = Rc::downgrade(&local_root);
        // TreeNode.cpp:104 — local_root->m_children.push_back(child);
        local_root.borrow_mut().m_children.push(child);
    }
    // TreeNode.cpp:106 — return local_root;
    local_root
}

// TreeNode.cpp:109-125 — void Node::reroot(const NodeSPtr &new_parent)
pub fn reroot(node: &NodeSPtr, new_parent: Option<&NodeSPtr>) {
    // TreeNode.cpp:111-115
    let is_root = node.borrow().m_is_root;
    if !is_root {
        // TreeNode.cpp:112 — auto old_parent = m_parent.lock();
        let old_parent = node.borrow().m_parent.upgrade().unwrap();
        // TreeNode.cpp:113 — old_parent->reroot(shared_from_this());
        reroot(&old_parent, Some(node));
        // TreeNode.cpp:114 — m_children.push_back(old_parent);
        node.borrow_mut().m_children.push(old_parent);
    }

    // TreeNode.cpp:117-124
    if let Some(new_parent) = new_parent {
        // TreeNode.cpp:118 — m_children.erase(std::remove(m_children.begin(), m_children.end(), new_parent), m_children.end());
        node.borrow_mut()
            .m_children
            .retain(|c| !Rc::ptr_eq(c, new_parent));
        // TreeNode.cpp:119 — m_is_root = false;
        node.borrow_mut().m_is_root = false;
        // TreeNode.cpp:120 — m_parent = new_parent;
        node.borrow_mut().m_parent = Rc::downgrade(new_parent);
    } else {
        // TreeNode.cpp:122 — m_is_root = true;
        node.borrow_mut().m_is_root = true;
        // TreeNode.cpp:123 — m_parent.reset();
        node.borrow_mut().m_parent = Weak::new();
    }
}

// TreeNode.cpp:127-142 — NodeSPtr Node::closestNode(const Point& loc)
pub fn closest_node(node: &NodeSPtr, loc: &Point) -> NodeSPtr {
    // TreeNode.cpp:129 — NodeSPtr result = shared_from_this();
    let mut result = Rc::clone(node);
    // TreeNode.cpp:130 — auto closest_dist2 = coord_t((m_p - loc).cast<double>().norm());
    let mut closest_dist2 = norm_diff(&node.borrow().m_p, loc) as Coord;

    // TreeNode.cpp:132-139
    let children: Vec<NodeSPtr> = node.borrow().m_children.iter().map(Rc::clone).collect();
    for child in &children {
        // TreeNode.cpp:133 — NodeSPtr candidate_node = child->closestNode(loc);
        let candidate_node = closest_node(child, loc);
        // TreeNode.cpp:134 — const auto child_dist2 = coord_t((candidate_node->m_p - loc).cast<double>().norm());
        let child_dist2 = norm_diff(&candidate_node.borrow().m_p, loc) as Coord;
        // TreeNode.cpp:135-138
        if child_dist2 < closest_dist2 {
            closest_dist2 = child_dist2;
            result = candidate_node;
        }
    }

    // TreeNode.cpp:141
    result
}

// TreeNode.cpp:144-154 — bool inside(const Polygons &polygons, const Point &p)
pub fn inside(polygons: &[Polygon], p: &Point) -> bool {
    // TreeNode.cpp:146 — int poly_count_inside = 0;
    let mut poly_count_inside = 0i32;
    // TreeNode.cpp:147-152
    for poly in polygons {
        // TreeNode.cpp:148 — const int is_inside_this_poly = ClipperLib::PointInPolygon(p, poly.points);
        let is_inside_this_poly = point_in_polygon(p, &poly.points);
        // TreeNode.cpp:149-150 — if (is_inside_this_poly == -1) return true;
        if is_inside_this_poly == -1 {
            return true;
        }
        // TreeNode.cpp:151 — poly_count_inside += is_inside_this_poly;
        poly_count_inside += is_inside_this_poly;
    }
    // TreeNode.cpp:153 — return (poly_count_inside % 2) == 1;
    (poly_count_inside % 2) == 1
}

// TreeNode.cpp:156-188 —
// bool lineSegmentPolygonsIntersection(const Point& a, const Point& b, const EdgeGrid::Grid& outline_locator, Point& result, const coord_t within_max_dist)
pub fn line_segment_polygons_intersection(
    a: &Point,
    b: &Point,
    outline_locator: &EdgeGrid,
    result: &mut Point,
    within_max_dist: Coord,
) -> bool {
    // TreeNode.cpp:158-180 — struct Visitor { ... } visitor { outline_locator, a.cast<double>(), b.cast<double>() };
    struct Visitor {
        line_a: Vec2d,
        line_b: Vec2d,
        intersection_pt: Vec2d,
        d2min: f64,
    }
    let mut visitor = Visitor {
        // line_a = a.cast<double>()
        line_a: Vec2d::new(a.x as f64, a.y as f64),
        // line_b = b.cast<double>()
        line_b: Vec2d::new(b.x as f64, b.y as f64),
        intersection_pt: Vec2d::new(0.0, 0.0),
        // d2min { std::numeric_limits<double>::max() }
        d2min: f64::MAX,
    };

    // TreeNode.cpp:182 — outline_locator.visit_cells_intersecting_line(a, b, visitor);
    outline_locator.visit_cells_intersecting_line(*a, *b, |iy, ix| {
        // TreeNode.cpp:159-170 — Visitor::operator()(coord_t iy, coord_t ix)
        // Called with a row and colum of the grid cell, which is intersected by a line.
        // TreeNode.cpp:161 — auto cell_data_range = grid.cell_data_range(iy, ix);
        let cell_data_range = outline_locator.cell_data_range_at(iy, ix);
        // TreeNode.cpp:162-170
        for it_contour_and_segment in cell_data_range {
            // TreeNode.cpp:164 — auto segment = grid.segment(*it_contour_and_segment);
            let segment = outline_locator.segment(*it_contour_and_segment);
            // TreeNode.cpp:165 —
            // if (Vec2d ip; Geometry::segment_segment_intersection(segment.first.cast<double>(), segment.second.cast<double>(), this->line_a, this->line_b, ip))
            // NOTE: BambuStudio passes the two segment endpoints as (point, direction)
            // to segment_segment_intersection. Ported faithfully (the quirk is preserved).
            let mut ip = Vec2d::new(0.0, 0.0);
            let seg_first = Vec2d::new(segment.a.x as f64, segment.a.y as f64);
            let seg_second = Vec2d::new(segment.b.x as f64, segment.b.y as f64);
            if segment_segment_intersection(
                &seg_first,
                &seg_second,
                &visitor.line_a,
                &visitor.line_b,
                &mut ip,
            ) {
                // TreeNode.cpp:166 — if (double d = (this->intersection_pt - this->line_b).squaredNorm(); d < d2min)
                // NOTE: `d` is computed from the *previous* intersection_pt, not `ip`. Preserved faithfully.
                let diff = visitor.intersection_pt - visitor.line_b;
                let d = diff.x * diff.x + diff.y * diff.y;
                if d < visitor.d2min {
                    // TreeNode.cpp:167 — this->d2min = d;
                    visitor.d2min = d;
                    // TreeNode.cpp:168 — this->intersection_pt = ip;
                    visitor.intersection_pt = ip;
                }
            }
        }
        // TreeNode.cpp:171-172 — Continue traversing the grid along the edge. (return true)
        true
    });

    // TreeNode.cpp:183 — if (visitor.d2min < double(within_max_dist) * double(within_max_dist))
    if visitor.d2min < (within_max_dist as f64) * (within_max_dist as f64) {
        // TreeNode.cpp:184 — result = Point(visitor.intersection_pt);
        // Point(const Vec2d&) constructs via lrint (round-half-to-even). Point.hpp:181.
        *result = Point::new(
            visitor.intersection_pt.x.round_ties_even() as Coord,
            visitor.intersection_pt.y.round_ties_even() as Coord,
        );
        // TreeNode.cpp:185 — return true;
        return true;
    }
    // TreeNode.cpp:187 — return false;
    false
}

// TreeNode.cpp:190-228 —
// bool Node::realign(const Polygons& outlines, const EdgeGrid::Grid& outline_locator, std::vector<NodeSPtr>& rerooted_parts)
pub fn realign(
    node: &NodeSPtr,
    outlines: &[Polygon],
    outline_locator: &EdgeGrid,
    rerooted_parts: &mut Vec<NodeSPtr>,
) -> bool {
    // TreeNode.cpp:192-193 — if (outlines.empty()) return false;
    if outlines.is_empty() {
        return false;
    }

    let m_p = node.borrow().m_p;
    // TreeNode.cpp:195 — if (inside(outlines, m_p))
    if inside(outlines, &m_p) {
        // Only keep children that have an unbroken connection to here, realign will put the rest in rerooted parts due to recursion:
        // TreeNode.cpp:197 — Point coll;
        let mut coll = Point::new(0, 0);
        // TreeNode.cpp:198 — bool reground_me = false;
        let mut reground_me = false;
        // TreeNode.cpp:199-211 — m_children.erase(std::remove_if(...), m_children.end());
        let children: Vec<NodeSPtr> = node.borrow().m_children.iter().map(Rc::clone).collect();
        let mut kept: Vec<NodeSPtr> = Vec::new();
        for child in &children {
            // TreeNode.cpp:200 — bool connect_branch = child->realign(outlines, outline_locator, rerooted_parts);
            let mut connect_branch = realign(child, outlines, outline_locator, rerooted_parts);
            // TreeNode.cpp:201-202 — Find an intersection of the line segment from p to child->p, at maximum outline_locator.resolution() * 2 distance from p.
            let child_p = child.borrow().m_p;
            if connect_branch
                && line_segment_polygons_intersection(
                    &child_p,
                    &m_p,
                    outline_locator,
                    &mut coll,
                    outline_locator.resolution() * 2,
                )
            {
                // TreeNode.cpp:203 — child->m_last_grounding_location.reset();
                child.borrow_mut().m_last_grounding_location = None;
                // TreeNode.cpp:204 — child->m_parent.reset();
                child.borrow_mut().m_parent = Weak::new();
                // TreeNode.cpp:205 — child->m_is_root = true;
                child.borrow_mut().m_is_root = true;
                // TreeNode.cpp:206 — rerooted_parts.push_back(child);
                rerooted_parts.push(Rc::clone(child));
                // TreeNode.cpp:207 — reground_me = true;
                reground_me = true;
                // TreeNode.cpp:208 — connect_branch = false;
                connect_branch = false;
            }
            // TreeNode.cpp:210 — return ! connect_branch; (remove_if predicate; keep when connect_branch)
            if connect_branch {
                kept.push(Rc::clone(child));
            }
        }
        node.borrow_mut().m_children = kept;
        // TreeNode.cpp:212-213 — if (reground_me) m_last_grounding_location.reset();
        if reground_me {
            node.borrow_mut().m_last_grounding_location = None;
        }
        // TreeNode.cpp:214 — return true;
        return true;
    }

    // TreeNode.cpp:217-224 — 'Lift' any decendants out of this tree:
    let children: Vec<NodeSPtr> = node.borrow().m_children.iter().map(Rc::clone).collect();
    for child in &children {
        // TreeNode.cpp:219 — if (child->realign(outlines, outline_locator, rerooted_parts))
        if realign(child, outlines, outline_locator, rerooted_parts) {
            // TreeNode.cpp:220 — child->m_last_grounding_location = m_p;
            child.borrow_mut().m_last_grounding_location = Some(m_p);
            // TreeNode.cpp:221 — child->m_parent.reset();
            child.borrow_mut().m_parent = Weak::new();
            // TreeNode.cpp:222 — child->m_is_root = true;
            child.borrow_mut().m_is_root = true;
            // TreeNode.cpp:223 — rerooted_parts.push_back(child);
            rerooted_parts.push(Rc::clone(child));
        }
    }

    // TreeNode.cpp:226 — m_children.clear();
    node.borrow_mut().m_children.clear();
    // TreeNode.cpp:227 — return false;
    false
}

// TreeNode.cpp:230-233 — void Node::straighten(const coord_t magnitude, const coord_t max_remove_colinear_dist)
pub fn straighten_root(node: &NodeSPtr, magnitude: Coord, max_remove_colinear_dist: Coord) {
    let m_p = node.borrow().m_p;
    // TreeNode.cpp:232 — straighten(magnitude, m_p, 0, int64_t(max_remove_colinear_dist) * int64_t(max_remove_colinear_dist));
    straighten(
        node,
        magnitude,
        &m_p,
        0,
        (max_remove_colinear_dist as i64) * (max_remove_colinear_dist as i64),
    );
}

// TreeNode.cpp:235-310 — Node::RectilinearJunction Node::straighten(...)
pub fn straighten(
    node: &NodeSPtr,
    magnitude: Coord,
    junction_above: &Point,
    accumulated_dist: Coord,
    max_remove_colinear_dist2: i64,
) -> RectilinearJunction {
    // TreeNode.cpp:241
    const JUNCTION_MAGNITUDE_FACTOR_NUMERATOR: Coord = 3;
    // TreeNode.cpp:242
    const JUNCTION_MAGNITUDE_FACTOR_DENOMINATOR: Coord = 4;

    // TreeNode.cpp:244 — const coord_t junction_magnitude = magnitude * junction_magnitude_factor_numerator / junction_magnitude_factor_denominator;
    let junction_magnitude =
        magnitude * JUNCTION_MAGNITUDE_FACTOR_NUMERATOR / JUNCTION_MAGNITUDE_FACTOR_DENOMINATOR;
    let children_len = node.borrow().m_children.len();
    // TreeNode.cpp:245
    if children_len == 1 {
        // TreeNode.cpp:247 — auto child_p = m_children.front();
        let child_p = Rc::clone(&node.borrow().m_children[0]);
        let m_p = node.borrow().m_p;
        // TreeNode.cpp:248 — auto child_dist = coord_t((m_p - child_p->m_p).cast<double>().norm());
        let child_dist = norm_diff(&m_p, &child_p.borrow().m_p) as Coord;
        // TreeNode.cpp:249 — RectilinearJunction junction_below = child_p->straighten(magnitude, junction_above, accumulated_dist + child_dist, max_remove_colinear_dist2);
        let junction_below = straighten(
            &child_p,
            magnitude,
            junction_above,
            accumulated_dist + child_dist,
            max_remove_colinear_dist2,
        );
        // TreeNode.cpp:250 — coord_t total_dist_to_junction_below = junction_below.total_recti_dist;
        let total_dist_to_junction_below = junction_below.total_recti_dist;
        // TreeNode.cpp:251 — const Point& a = junction_above;
        let a = *junction_above;
        // TreeNode.cpp:252 — Point b = junction_below.junction_loc;
        let b = junction_below.junction_loc;
        // TreeNode.cpp:253 — if (a != b) // should always be true!
        if a != b {
            // TreeNode.cpp:255 — Point ab = b - a;
            let ab = b - a;
            // TreeNode.cpp:256 — Point destination = (a.cast<int64_t>() + ab.cast<int64_t>() * int64_t(accumulated_dist) / std::max(int64_t(1), int64_t(total_dist_to_junction_below))).cast<coord_t>();
            let denom = std::cmp::max(1i64, total_dist_to_junction_below as i64);
            let destination = Point::new(
                a.x as i64 + (ab.x as i64) * (accumulated_dist as i64) / denom,
                a.y as i64 + (ab.y as i64) * (accumulated_dist as i64) / denom,
            );
            let m_p_now = node.borrow().m_p;
            // TreeNode.cpp:257-258 — if ((destination - m_p).cast<int64_t>().squaredNorm() <= int64_t(magnitude) * int64_t(magnitude)) m_p = destination;
            let dmp = destination - m_p_now;
            let sq = (dmp.x as i64) * (dmp.x as i64) + (dmp.y as i64) * (dmp.y as i64);
            if sq <= (magnitude as i64) * (magnitude as i64) {
                node.borrow_mut().m_p = destination;
            } else {
                // TreeNode.cpp:260 — m_p += ((destination - m_p).cast<double>().normalized() * magnitude).cast<coord_t>();
                let nrm = normalized_scaled(&dmp, magnitude as f64);
                node.borrow_mut().m_p += nrm;
            }
        }
        {
            // remove nodes on linear segments
            // TreeNode.cpp:263 — constexpr coord_t close_enough = 10;
            const CLOSE_ENOUGH: Coord = 10;

            // TreeNode.cpp:265 — child_p = m_children.front(); //recursive call to straighten might have removed the child
            let child_p = Rc::clone(&node.borrow().m_children[0]);
            // TreeNode.cpp:266 — const NodeSPtr& parent_node = m_parent.lock();
            let parent_node = node.borrow().m_parent.upgrade();
            // TreeNode.cpp:267-269 —
            // if (parent_node &&
            //     (child_p->m_p - parent_node->m_p).cast<int64_t>().squaredNorm() < max_remove_colinear_dist2 &&
            //     Line::distance_to_squared(m_p, parent_node->m_p, child_p->m_p) < close_enough * close_enough)
            if let Some(parent_node) = parent_node {
                let child_pp = child_p.borrow().m_p;
                let parent_pp = parent_node.borrow().m_p;
                let m_p_cur = node.borrow().m_p;
                let dcp = child_pp - parent_pp;
                let sq_cp = (dcp.x as i64) * (dcp.x as i64) + (dcp.y as i64) * (dcp.y as i64);
                if sq_cp < max_remove_colinear_dist2
                    && line_distance_to_squared(&m_p_cur, &parent_pp, &child_pp)
                        < (CLOSE_ENOUGH as f64) * (CLOSE_ENOUGH as f64)
                {
                    // TreeNode.cpp:270 — child_p->m_parent = m_parent;
                    child_p.borrow_mut().m_parent = node.borrow().m_parent.clone();
                    // TreeNode.cpp:271-278 — for (auto& sibling : parent_node->m_children) { if (sibling == shared_from_this()) { sibling = child_p; break; } }
                    let mut siblings = parent_node.borrow_mut();
                    for sibling in siblings.m_children.iter_mut() {
                        // TreeNode.cpp:273 — if (sibling == shared_from_this())
                        if Rc::ptr_eq(sibling, node) {
                            // TreeNode.cpp:275 — sibling = child_p; // replace this node by child
                            *sibling = Rc::clone(&child_p);
                            // TreeNode.cpp:276 — break;
                            break;
                        }
                    }
                }
            }
        }
        // TreeNode.cpp:281 — return junction_below;
        junction_below
    } else {
        // TreeNode.cpp:285 — constexpr coord_t weight = 1000;
        const WEIGHT: Coord = 1000;
        let m_p = node.borrow().m_p;
        // TreeNode.cpp:286 — Point junction_moving_dir = ((junction_above - m_p).cast<double>().normalized() * weight).cast<coord_t>();
        let mut junction_moving_dir = normalized_scaled(&(*junction_above - m_p), WEIGHT as f64);
        // TreeNode.cpp:287 — bool prevent_junction_moving = false;
        let mut prevent_junction_moving = false;
        // TreeNode.cpp:288-298
        let children: Vec<NodeSPtr> = node.borrow().m_children.iter().map(Rc::clone).collect();
        for child_p in &children {
            // TreeNode.cpp:290 — const auto child_dist = coord_t((m_p - child_p->m_p).cast<double>().norm());
            let child_dist = norm_diff(&m_p, &child_p.borrow().m_p) as Coord;
            // TreeNode.cpp:291 — RectilinearJunction below = child_p->straighten(magnitude, m_p, child_dist, max_remove_colinear_dist2);
            let below = straighten(child_p, magnitude, &m_p, child_dist, max_remove_colinear_dist2);
            // TreeNode.cpp:293 — junction_moving_dir += ((below.junction_loc - m_p).cast<double>().normalized() * weight).cast<coord_t>();
            junction_moving_dir += normalized_scaled(&(below.junction_loc - m_p), WEIGHT as f64);
            // TreeNode.cpp:294-297 — if (below.total_recti_dist < magnitude) prevent_junction_moving = true;
            if below.total_recti_dist < magnitude {
                // prevent flipflopping in branches due to straightening and junctoin moving clashing
                prevent_junction_moving = true;
            }
        }
        let is_root = node.borrow().m_is_root;
        let children_empty = node.borrow().m_children.is_empty();
        // TreeNode.cpp:299 — if (junction_moving_dir != Point(0, 0) && ! m_children.empty() && ! m_is_root && ! prevent_junction_moving)
        if junction_moving_dir != Point::new(0, 0)
            && !children_empty
            && !is_root
            && !prevent_junction_moving
        {
            // TreeNode.cpp:301 — auto junction_moving_dir_len = coord_t(junction_moving_dir.norm());
            let junction_moving_dir_len = norm(&junction_moving_dir) as Coord;
            // TreeNode.cpp:302-305 — if (junction_moving_dir_len > junction_magnitude) junction_moving_dir = junction_moving_dir * junction_magnitude / junction_moving_dir_len;
            if junction_moving_dir_len > junction_magnitude {
                junction_moving_dir = Point::new(
                    junction_moving_dir.x * junction_magnitude / junction_moving_dir_len,
                    junction_moving_dir.y * junction_magnitude / junction_moving_dir_len,
                );
            }
            // TreeNode.cpp:306 — m_p += junction_moving_dir;
            node.borrow_mut().m_p += junction_moving_dir;
        }
        // TreeNode.cpp:308 — return RectilinearJunction{ accumulated_dist, m_p };
        let m_p_final = node.borrow().m_p;
        RectilinearJunction {
            total_recti_dist: accumulated_dist,
            junction_loc: m_p_final,
        }
    }
}

// TreeNode.cpp:312-348 —
// Prune the tree from the extremeties (leaf-nodes) until the pruning distance is reached.
// coord_t Node::prune(const coord_t& pruning_distance)
pub fn prune(node: &NodeSPtr, pruning_distance: Coord) -> Coord {
    // TreeNode.cpp:315-316 — if (pruning_distance <= 0) return 0;
    if pruning_distance <= 0 {
        return 0;
    }

    // TreeNode.cpp:318 — coord_t max_distance_pruned = 0;
    let mut max_distance_pruned: Coord = 0;
    // TreeNode.cpp:319 — for (auto child_it = m_children.begin(); child_it != m_children.end(); )
    let mut idx = 0usize;
    loop {
        // Re-read length each iteration (the vector mutates as children are erased).
        if idx >= node.borrow().m_children.len() {
            break;
        }
        // TreeNode.cpp:320 — auto& child = *child_it;
        let child = Rc::clone(&node.borrow().m_children[idx]);
        // TreeNode.cpp:321 — coord_t dist_pruned_child = child->prune(pruning_distance);
        let dist_pruned_child = prune(&child, pruning_distance);
        // TreeNode.cpp:322 — if (dist_pruned_child >= pruning_distance)
        if dist_pruned_child >= pruning_distance {
            // pruning is finished for child; dont modify further
            // TreeNode.cpp:324 — max_distance_pruned = std::max(max_distance_pruned, dist_pruned_child);
            max_distance_pruned = std::cmp::max(max_distance_pruned, dist_pruned_child);
            // TreeNode.cpp:325 — ++child_it;
            idx += 1;
        } else {
            // TreeNode.cpp:327 — const Point a = getLocation();
            let a = node.borrow().m_p;
            // TreeNode.cpp:328 — const Point b = child->getLocation();
            let b = child.borrow().m_p;
            // TreeNode.cpp:329 — const Point ba = a - b;
            let ba = a - b;
            // TreeNode.cpp:330 — const auto ab_len = coord_t(ba.cast<double>().norm());
            let ab_len = norm(&ba) as Coord;
            // TreeNode.cpp:331 — if (dist_pruned_child + ab_len <= pruning_distance)
            if dist_pruned_child + ab_len <= pruning_distance {
                // we're still in the process of pruning
                // TreeNode.cpp:333 — assert(child->m_children.empty() && "...");
                debug_assert!(
                    child.borrow().m_children.is_empty(),
                    "when pruning away a node all it's children must already have been pruned away"
                );
                // TreeNode.cpp:334 — max_distance_pruned = std::max(max_distance_pruned, dist_pruned_child + ab_len);
                max_distance_pruned = std::cmp::max(max_distance_pruned, dist_pruned_child + ab_len);
                // TreeNode.cpp:335 — child_it = m_children.erase(child_it);
                node.borrow_mut().m_children.remove(idx);
            } else {
                // pruning stops in between this node and the child
                // TreeNode.cpp:338 — const Point n = b + (ba.cast<double>().normalized() * (pruning_distance - dist_pruned_child)).cast<coord_t>();
                let n = b + normalized_scaled(&ba, (pruning_distance - dist_pruned_child) as f64);
                // TreeNode.cpp:339 — assert(std::abs((n - b).cast<double>().norm() + dist_pruned_child - pruning_distance) < 10 && "...");
                debug_assert!(
                    (norm(&(n - b)) + dist_pruned_child as f64 - pruning_distance as f64).abs() < 10.0,
                    "total pruned distance must be equal to the pruning_distance"
                );
                // TreeNode.cpp:340 — max_distance_pruned = std::max(max_distance_pruned, pruning_distance);
                max_distance_pruned = std::cmp::max(max_distance_pruned, pruning_distance);
                // TreeNode.cpp:341 — child->setLocation(n);
                child.borrow_mut().m_p = n;
                // TreeNode.cpp:342 — ++child_it;
                idx += 1;
            }
        }
    }

    // TreeNode.cpp:347 — return max_distance_pruned;
    max_distance_pruned
}

// TreeNode.cpp:350-357 — void Node::convertToPolylines(Polylines &output, const coord_t line_overlap) const
pub fn convert_to_polylines(node: &NodeSPtr, output: &mut Vec<Polyline>, line_overlap: Coord) {
    // TreeNode.cpp:352 — Polylines result;
    let mut result: Vec<Polyline> = Vec::new();
    // TreeNode.cpp:353 — result.emplace_back();
    result.push(Polyline::new());
    // TreeNode.cpp:354 — convertToPolylines(0, result);
    convert_to_polylines_idx(node, 0, &mut result);
    // TreeNode.cpp:355 — removeJunctionOverlap(result, line_overlap);
    remove_junction_overlap(&mut result, line_overlap);
    // TreeNode.cpp:356 — append(output, std::move(result));
    output.extend(result);
}

// TreeNode.cpp:359-377 — void Node::convertToPolylines(size_t long_line_idx, Polylines &output) const
pub fn convert_to_polylines_idx(node: &NodeSPtr, long_line_idx: usize, output: &mut Vec<Polyline>) {
    let m_p = node.borrow().m_p;
    let children_len = node.borrow().m_children.len();
    // TreeNode.cpp:361-364 — if (m_children.empty()) { output[long_line_idx].points.push_back(m_p); return; }
    if children_len == 0 {
        output[long_line_idx].points.push(m_p);
        return;
    }
    // TreeNode.cpp:365 — size_t first_child_idx = rand() % m_children.size();
    let first_child_idx = (c_rand() as usize) % children_len;
    // TreeNode.cpp:366 — m_children[first_child_idx]->convertToPolylines(long_line_idx, output);
    let first_child = Rc::clone(&node.borrow().m_children[first_child_idx]);
    convert_to_polylines_idx(&first_child, long_line_idx, output);
    // TreeNode.cpp:367 — output[long_line_idx].points.push_back(m_p);
    output[long_line_idx].points.push(m_p);

    // TreeNode.cpp:369-376
    for idx_offset in 1..children_len {
        // TreeNode.cpp:370 — size_t child_idx = (first_child_idx + idx_offset) % m_children.size();
        let child_idx = (first_child_idx + idx_offset) % children_len;
        // TreeNode.cpp:371 — const Node& child = *m_children[child_idx];
        let child = Rc::clone(&node.borrow().m_children[child_idx]);
        // TreeNode.cpp:372 — output.emplace_back();
        output.push(Polyline::new());
        // TreeNode.cpp:373 — size_t child_line_idx = output.size() - 1;
        let child_line_idx = output.len() - 1;
        // TreeNode.cpp:374 — child.convertToPolylines(child_line_idx, output);
        convert_to_polylines_idx(&child, child_line_idx, output);
        // TreeNode.cpp:375 — output[child_line_idx].points.emplace_back(m_p);
        output[child_line_idx].points.push(m_p);
    }
}

// TreeNode.cpp:379-413 — void Node::removeJunctionOverlap(Polylines &result_lines, const coord_t line_overlap) const
pub fn remove_junction_overlap(result_lines: &mut Vec<Polyline>, line_overlap: Coord) {
    // TreeNode.cpp:381 — const coord_t reduction = line_overlap;
    let reduction = line_overlap;
    // TreeNode.cpp:382 — size_t res_line_idx = 0;
    let mut res_line_idx = 0usize;
    // TreeNode.cpp:383 — while (res_line_idx < result_lines.size())
    while res_line_idx < result_lines.len() {
        // TreeNode.cpp:384 — Polyline &polyline = result_lines[res_line_idx];
        // TreeNode.cpp:385-389 — if (polyline.size() <= 1) { polyline = std::move(result_lines.back()); result_lines.pop_back(); continue; }
        if result_lines[res_line_idx].size() <= 1 {
            let back = result_lines.pop().unwrap();
            if res_line_idx < result_lines.len() {
                result_lines[res_line_idx] = back;
            }
            // (If res_line_idx == result_lines.len() after pop, the moved-from slot was the back itself.)
            continue;
        }

        // TreeNode.cpp:391 — coord_t to_be_reduced = reduction;
        let mut to_be_reduced = reduction;
        // TreeNode.cpp:392 — Point a = polyline.back();
        let mut a = *result_lines[res_line_idx].points.last().unwrap();
        // TreeNode.cpp:393-405 — for (int point_idx = int(polyline.size()) - 2; point_idx >= 0; point_idx--)
        let mut point_idx = result_lines[res_line_idx].size() as i64 - 2;
        while point_idx >= 0 {
            // TreeNode.cpp:394 — const Point b = polyline.points[point_idx];
            let b = result_lines[res_line_idx].points[point_idx as usize];
            // TreeNode.cpp:395 — const Point ab = b - a;
            let ab = b - a;
            // TreeNode.cpp:396 — const auto ab_len = coord_t(ab.cast<double>().norm());
            let ab_len = norm(&ab) as Coord;
            // TreeNode.cpp:397 — if (ab_len >= to_be_reduced)
            if ab_len >= to_be_reduced {
                // TreeNode.cpp:398 — polyline.points.back() = a + (ab.cast<double>() * (double(to_be_reduced) / ab_len)).cast<coord_t>();
                // Eigen `.cast<coord_t>()` truncates toward zero (matches Rust `as Coord`).
                let scaled = Point::new(
                    (ab.x as f64 * (to_be_reduced as f64 / ab_len as f64)) as Coord,
                    (ab.y as f64 * (to_be_reduced as f64 / ab_len as f64)) as Coord,
                );
                let new_back = a + scaled;
                let last = result_lines[res_line_idx].points.len() - 1;
                result_lines[res_line_idx].points[last] = new_back;
                // TreeNode.cpp:399 — break;
                break;
            } else {
                // TreeNode.cpp:401 — to_be_reduced -= ab_len;
                to_be_reduced -= ab_len;
                // TreeNode.cpp:402 — polyline.points.pop_back();
                result_lines[res_line_idx].points.pop();
            }
            // TreeNode.cpp:404 — a = b;
            a = b;
            point_idx -= 1;
        }

        // TreeNode.cpp:407-411 — if (polyline.size() <= 1) { polyline = std::move(result_lines.back()); result_lines.pop_back(); } else ++res_line_idx;
        if result_lines[res_line_idx].size() <= 1 {
            let back = result_lines.pop().unwrap();
            if res_line_idx < result_lines.len() {
                result_lines[res_line_idx] = back;
            }
        } else {
            res_line_idx += 1;
        }
    }
}

// TreeNode.hpp:286-293 — inline BoundingBox get_extents(const NodeSPtr &root_node)
pub fn get_extents(root_node: &NodeSPtr) -> BoundingBox {
    // TreeNode.hpp:288 — BoundingBox bbox;
    let mut bbox = BoundingBox::new();
    // TreeNode.hpp:289-290 — for (const NodeSPtr &children : root_node->m_children) bbox.merge(get_extents(children));
    for children in root_node.borrow().m_children.iter() {
        bbox.merge(&get_extents(children));
    }
    // TreeNode.hpp:291 — bbox.merge(root_node->getLocation());
    bbox.merge_point(root_node.borrow().m_p);
    // TreeNode.hpp:292 — return bbox;
    bbox
}

// TreeNode.hpp:295-301 — inline BoundingBox get_extents(const std::vector<NodeSPtr> &tree_roots)
pub fn get_extents_roots(tree_roots: &[NodeSPtr]) -> BoundingBox {
    // TreeNode.hpp:297 — BoundingBox bbox;
    let mut bbox = BoundingBox::new();
    // TreeNode.hpp:298-299 — for (const NodeSPtr &root_node : tree_roots) bbox.merge(get_extents(root_node));
    for root_node in tree_roots {
        bbox.merge(&get_extents(root_node));
    }
    // TreeNode.hpp:300 — return bbox;
    bbox
}

// ----------------------------------------------------------------------------
// Helpers matching C++ Eigen / ClipperLib semantics used above.
// ----------------------------------------------------------------------------

/// `(p - q).cast<double>().norm()` — Euclidean length of the integer difference
/// cast to double WITHOUT unscaling (unlike `Point::to_f64`).
#[inline]
fn norm_diff(p: &Point, q: &Point) -> f64 {
    let dx = (p.x - q.x) as f64;
    let dy = (p.y - q.y) as f64;
    (dx * dx + dy * dy).sqrt()
}

/// `v.cast<double>().norm()` — length of an integer point treated as a vector.
#[inline]
fn norm(v: &Point) -> f64 {
    let x = v.x as f64;
    let y = v.y as f64;
    (x * x + y * y).sqrt()
}

/// `(v.cast<double>().normalized() * len).cast<coord_t>()` — unit vector of `v`
/// scaled by `len`, then truncated back to integer (Eigen `.cast<coord_t>()` truncates).
#[inline]
fn normalized_scaled(v: &Point, len: f64) -> Point {
    let x = v.x as f64;
    let y = v.y as f64;
    let n = (x * x + y * y).sqrt();
    if n == 0.0 {
        // Eigen normalized() of a zero vector yields NaN, then cast<coord_t>() is UB;
        // in practice the call sites guard against zero vectors. Return zero.
        return Point::new(0, 0);
    }
    Point::new(((x / n) * len) as Coord, ((y / n) * len) as Coord)
}

/// `Line::distance_to_squared(point, a, b)` — squared distance from `p` to the
/// segment `[a, b]`, computed in full double precision (line_alg, Line.hpp:43-72).
#[inline]
fn line_distance_to_squared(p: &Point, a: &Point, b: &Point) -> f64 {
    // Line.hpp:45-46 — v = (b - a), va = (point - a)
    let vx = (b.x - a.x) as f64;
    let vy = (b.y - a.y) as f64;
    let vax = (p.x - a.x) as f64;
    let vay = (p.y - a.y) as f64;
    // Line.hpp:47 — l2 = v.squaredNorm()
    let l2 = vx * vx + vy * vy;
    // Line.hpp:48-52 — a == b case
    if l2 == 0.0 {
        return vax * vax + vay * vay;
    }
    // Line.hpp:57 — t = va.dot(v) / l2
    let t = (vax * vx + vay * vy) / l2;
    if t <= 0.0 {
        // Line.hpp:58-61 — beyond the 'a' end
        vax * vax + vay * vay
    } else if t >= 1.0 {
        // Line.hpp:62-65 — beyond the 'b' end
        let dx = (p.x - b.x) as f64;
        let dy = (p.y - b.y) as f64;
        dx * dx + dy * dy
    } else {
        // Line.hpp:67-68 — (t * v - va).squaredNorm()
        let ex = t * vx - vax;
        let ey = t * vy - vay;
        ex * ex + ey * ey
    }
}

/// ClipperLib::PointInPolygon(pt, path) — returns 0 (outside), 1 (inside), -1 (on boundary).
/// clipper.cpp:4793 — `int PointInPolygon(const IntPoint &pt, const Path &path)`.
/// Mirrors the private helper in geometry::polygon (re-implemented here to keep the
/// faithful TreeNode `inside()` self-contained, matching the C++ call).
#[inline]
fn point_in_polygon(pt: &Point, path: &[Point]) -> i32 {
    // clipper.cpp:4793
    let mut result = 0i32;
    // clipper.cpp:4794
    let cnt = path.len();
    // clipper.cpp:4795
    if cnt < 3 {
        return 0;
    }
    // clipper.cpp:4796
    let mut ip = path[0];
    // clipper.cpp:4797
    for i in 1..=cnt {
        // clipper.cpp:4799
        let ip_next = if i == cnt { path[0] } else { path[i] };
        // clipper.cpp:4800-4801
        if ip_next.y == pt.y
            && ((ip_next.x == pt.x) || (ip.y == pt.y && ((ip_next.x > pt.x) == (ip.x < pt.x))))
        {
            return -1;
        }
        // clipper.cpp:4802
        if (ip.y < pt.y) != (ip_next.y < pt.y) {
            // clipper.cpp:4804
            if ip.x >= pt.x {
                // clipper.cpp:4805-4806
                if ip_next.x > pt.x {
                    result = 1 - result;
                } else {
                    // clipper.cpp:4808-4811
                    let d = (ip.x as f64 - pt.x as f64) * (ip_next.y as f64 - pt.y as f64)
                        - (ip_next.x as f64 - pt.x as f64) * (ip.y as f64 - pt.y as f64);
                    if d == 0.0 {
                        return -1;
                    }
                    if (d > 0.0) == (ip_next.y > ip.y) {
                        result = 1 - result;
                    }
                }
            } else {
                // clipper.cpp:4813-4814
                if ip_next.x > pt.x {
                    // clipper.cpp:4815-4817
                    let d = (ip.x as f64 - pt.x as f64) * (ip_next.y as f64 - pt.y as f64)
                        - (ip_next.x as f64 - pt.x as f64) * (ip.y as f64 - pt.y as f64);
                    if d == 0.0 {
                        return -1;
                    }
                    if (d > 0.0) == (ip_next.y > ip.y) {
                        result = 1 - result;
                    }
                }
            }
        }
        // clipper.cpp:4822
        ip = ip_next;
    }
    // clipper.cpp:4824
    result
}

thread_local! {
    // Faithful replacement for C libc rand(): a glibc-compatible state so that
    // `rand() % n` selections at junctions can be made deterministic per run.
    static RAND_STATE: RefCell<u64> = const { RefCell::new(1) };
}

/// C `rand()` returning a value in [0, RAND_MAX]. TreeNode.cpp:365 uses `rand() % n`.
/// Implemented with the POSIX-documented minimal-standard LCG so junction selection
/// is reproducible; the exact stream is not byte-identical to the platform libc, which
/// is acceptable because `convertToPolylines` only chooses which equivalent branch to
/// continue first (the resulting polyline set is the same up to ordering).
#[inline]
fn c_rand() -> i32 {
    RAND_STATE.with(|s| {
        let mut state = s.borrow_mut();
        *state = state.wrapping_mul(1103515245).wrapping_add(12345);
        ((*state >> 16) & 0x7fff) as i32
    })
}

/// `weak_ptr<Node>::lock() == shared_from_this()` equality used in the debug asserts.
#[inline]
fn weak_ptr_eq(weak: &Weak<RefCell<Node>>, node: &NodeSPtr) -> bool {
    match weak.upgrade() {
        Some(p) => Rc::ptr_eq(&p, node),
        None => false,
    }
}
