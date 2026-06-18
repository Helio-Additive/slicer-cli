//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.
//
//! Faithful 1:1 port of `Fill/Lightning/Layer.{hpp,cpp}` from BambuStudio.
//!
//! Each Layer holds a forest of tree nodes for one print layer. Trees
//! are grown from unsupported (overhang) points toward grounded regions,
//! and the edges of each tree become infill lines.
//!
//! C++ uses TBB (`tbb::parallel_for` over a `blocked_range2d`) inside
//! `getBestGroundingLocation`. The C++ code is explicitly written so the
//! parallel result is identical to a non-parallel scan (see the comment at
//! Layer.cpp:177-182). This port performs that scan serially in the exact same
//! grid order (y outer, x inner), which reproduces the non-parallel result the
//! C++ guarantees. FIDELITY-NOTE: serial scan (no TBB); logically equivalent.

use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Weak;

use super::tree_node::{self, NodeSPtr, Node, LOCATOR_CELL_SIZE};
use crate::edge_grid::EdgeGrid;
use crate::geometry::segments_intersect;
use crate::geometry::{BoundingBox, Point, PointF, Polygon, Polyline};
use crate::Coord;

// Layer.hpp:21 — using SparseNodeGrid = std::unordered_multimap<Point, std::weak_ptr<Node>, PointHash>;
// `Point` derives `Hash`/`Eq` (Point.hpp PointHash), so it is used directly as the
// key. The multimap is modelled as `HashMap<Point, Vec<Weak<..>>>`; insertion
// order within a bucket is preserved (push_back), matching C++ `equal_range`.
type SparseNodeGrid = HashMap<Point, Vec<Weak<RefCell<Node>>>>;

/// Layer.hpp:23-28 — struct GroundingLocation.
///
/// `tree_node` is non-null if the grounding location is on a tree; otherwise
/// `boundary_location` holds a point on the boundary.
#[derive(Debug, Clone, Default)]
pub struct GroundingLocation {
    // Layer.hpp:25 — NodeSPtr tree_node;
    pub tree_node: Option<NodeSPtr>,
    // Layer.hpp:26 — std::optional<Point> boundary_location;
    pub boundary_location: Option<Point>,
}

impl GroundingLocation {
    // Layer.cpp:24-28 — Point GroundingLocation::p() const
    pub fn p(&self) -> Point {
        // Layer.cpp:26 — assert(tree_node || boundary_location);
        debug_assert!(self.tree_node.is_some() || self.boundary_location.is_some());
        // Layer.cpp:27 — return tree_node ? tree_node->getLocation() : *boundary_location;
        match &self.tree_node {
            Some(tn) => tn.borrow().get_location(),
            None => self.boundary_location.unwrap(),
        }
    }
}

/// Layer.hpp:35-88 — class Layer.
#[derive(Debug, Clone, Default)]
pub struct Layer {
    // Layer.hpp:38 — std::vector<NodeSPtr> tree_roots;
    pub tree_roots: Vec<NodeSPtr>,
}

// Layer.cpp:19-22 — coord_t Layer::getWeightedDistance(const Point& boundary_loc, const Point& unsupported_location)
//
// This is the free Layer-level weighted distance (distinct from
// `Node::getWeightedDistance`). It is a plain Euclidean norm cast to coord_t.
pub fn get_weighted_distance(boundary_loc: &Point, unsupported_location: &Point) -> Coord {
    // Layer.cpp:21 — return coord_t((boundary_loc - unsupported_location).cast<double>().norm());
    let dx = (boundary_loc.x - unsupported_location.x) as f64;
    let dy = (boundary_loc.y - unsupported_location.y) as f64;
    (dx * dx + dy * dy).sqrt() as Coord
}

// Layer.cpp:30-33 — inline static Point to_grid_point(const Point &point, const BoundingBox &bbox)
#[inline]
fn to_grid_point(point: &Point, bbox: &BoundingBox) -> Point {
    // Layer.cpp:32 — return (point - bbox.min) / locator_cell_size;
    (*point - bbox.min) / LOCATOR_CELL_SIZE
}

// line_alg::distance_to_squared (Line.hpp:42-68) for the integer `Line`
// instantiation: `point`/`*nearest_point` are integer coord_t, all intermediates
// are double, and the nearest point is truncated to the integer grid by the final
// `.cast<Scalar<L>>()`. Used by `getBestGroundingLocation`'s closest-point scan
// (Layer.cpp:138). Mirrors `aabb_tree_lines::line_alg::distance_to_squared`.
fn line_distance_to_squared(a: &Point, b: &Point, p: &Point, nearest_point: &mut Point) -> f64 {
    // Line.hpp:45-46
    let v = PointF::new((b.x - a.x) as f64, (b.y - a.y) as f64);
    let va = PointF::new((p.x - a.x) as f64, (p.y - a.y) as f64);
    // Line.hpp:47
    let l2 = v.x * v.x + v.y * v.y;
    if l2 == 0.0 {
        // Line.hpp:49-51 — *nearest_point = get_a(line)
        *nearest_point = *a;
        return va.x * va.x + va.y * va.y;
    }
    // Line.hpp:56
    let t = (va.x * v.x + va.y * v.y) / l2;
    if t <= 0.0 {
        // Line.hpp:57-60
        *nearest_point = *a;
        va.x * va.x + va.y * va.y
    } else if t >= 1.0 {
        // Line.hpp:61-64 — *nearest_point = get_b(line)
        *nearest_point = *b;
        let vb = PointF::new((p.x - b.x) as f64, (p.y - b.y) as f64);
        vb.x * vb.x + vb.y * vb.y
    } else {
        // Line.hpp:65-67 — Vec<Dim,double> w = (v * t); *nearest_point = (a + w).cast<Scalar>();
        let wx = v.x * t;
        let wy = v.y * t;
        *nearest_point = Point::new((a.x as f64 + wx) as Coord, (a.y as f64 + wy) as Coord);
        let dx = va.x - wx;
        let dy = va.y - wy;
        dx * dx + dy * dy
    }
}

impl Layer {
    /// Create a new empty layer.
    pub fn new() -> Self {
        Self {
            tree_roots: Vec::new(),
        }
    }

    // Layer.cpp:35-42 — void Layer::fillLocator(SparseNodeGrid &tree_node_locator, const BoundingBox& current_outlines_bbox)
    fn fill_locator(&self, tree_node_locator: &mut SparseNodeGrid, current_outlines_bbox: &BoundingBox) {
        // Layer.cpp:37-39 — add_node_to_locator_func inserts (to_grid_point(loc), node).
        // Layer.cpp:40-41 — for (auto& tree : tree_roots) tree->visitNodes(add_node_to_locator_func);
        for tree in &self.tree_roots {
            tree_node::visit_nodes(tree, &mut |node: NodeSPtr| {
                let key = to_grid_point(&node.borrow().get_location(), current_outlines_bbox);
                tree_node_locator
                    .entry(key)
                    .or_default()
                    .push(std::rc::Rc::downgrade(&node));
            });
        }
    }

    // Layer.cpp:44-86 — void Layer::generateNewTrees(...)
    #[allow(clippy::too_many_arguments)]
    pub fn generate_new_trees<F: Fn()>(
        &mut self,
        current_overhang: &[Polygon],
        current_outlines: &[Polygon],
        current_outlines_bbox: &BoundingBox,
        outlines_locator: &EdgeGrid,
        supporting_radius: Coord,
        wall_supporting_radius: Coord,
        throw_on_cancel_callback: &F,
    ) {
        // Layer.cpp:55 — DistanceField distance_field(supporting_radius, current_outlines, current_outlines_bbox, current_overhang);
        let mut distance_field = super::distance_field::DistanceField::new(
            supporting_radius,
            current_outlines,
            current_outlines_bbox,
            current_overhang,
        );
        // Layer.cpp:56 — throw_on_cancel_callback();
        throw_on_cancel_callback();

        // Layer.cpp:58-59 — SparseNodeGrid tree_node_locator; fillLocator(tree_node_locator, current_outlines_bbox);
        let mut tree_node_locator: SparseNodeGrid = HashMap::new();
        self.fill_locator(&mut tree_node_locator, current_outlines_bbox);

        // Layer.cpp:63-64
        let mut unsupported_cell_idx: usize = 0;
        let mut unsupported_location = Point::new(0, 0);
        // Layer.cpp:65 — while (distance_field.tryGetNextPoint(&unsupported_location, &unsupported_cell_idx, unsupported_cell_idx))
        // The third argument is the current value of `unsupported_cell_idx` (the
        // search start index); the same variable is also written via the out-param.
        loop {
            let start_idx = unsupported_cell_idx;
            if !distance_field.try_get_next_point(
                &mut unsupported_location,
                &mut unsupported_cell_idx,
                start_idx,
            ) {
                break;
            }
            // Layer.cpp:66 — throw_on_cancel_callback();
            throw_on_cancel_callback();
            // Layer.cpp:67-68 — GroundingLocation grounding_loc = getBestGroundingLocation(...);
            let grounding_loc = self.get_best_grounding_location(
                &unsupported_location,
                current_outlines,
                current_outlines_bbox,
                outlines_locator,
                supporting_radius,
                wall_supporting_radius,
                &tree_node_locator,
                None,
            );

            // Layer.cpp:70-72 — NodeSPtr new_parent; NodeSPtr new_child; this->attach(unsupported_location, grounding_loc, new_child, new_parent);
            let mut new_child: Option<NodeSPtr> = None;
            let mut new_parent: Option<NodeSPtr> = None;
            self.attach(
                &unsupported_location,
                &grounding_loc,
                &mut new_child,
                &mut new_parent,
            );
            // Layer.cpp:73 — tree_node_locator.insert(make_pair(to_grid_point(new_child->getLocation()), new_child));
            if let Some(child) = &new_child {
                let key = to_grid_point(&child.borrow().get_location(), current_outlines_bbox);
                tree_node_locator
                    .entry(key)
                    .or_default()
                    .push(std::rc::Rc::downgrade(child));
            }
            // Layer.cpp:74-75 — if (new_parent) tree_node_locator.insert(make_pair(to_grid_point(new_parent->getLocation()), new_parent));
            if let Some(parent) = &new_parent {
                let key = to_grid_point(&parent.borrow().get_location(), current_outlines_bbox);
                tree_node_locator
                    .entry(key)
                    .or_default()
                    .push(std::rc::Rc::downgrade(parent));
            }
            // Layer.cpp:77 — distance_field.update(grounding_loc.p(), unsupported_location);
            distance_field.update(&grounding_loc.p(), &unsupported_location);
        }
    }

    // Layer.cpp:117-200 — GroundingLocation Layer::getBestGroundingLocation(...)
    #[allow(clippy::too_many_arguments)]
    pub fn get_best_grounding_location(
        &self,
        unsupported_location: &Point,
        current_outlines: &[Polygon],
        current_outlines_bbox: &BoundingBox,
        outline_locator: &EdgeGrid,
        supporting_radius: Coord,
        wall_supporting_radius: Coord,
        tree_node_locator: &SparseNodeGrid,
        exclude_tree: Option<&NodeSPtr>,
    ) -> GroundingLocation {
        // Layer.cpp:129-145 — Closest point on current_outlines to unsupported_location.
        let mut node_location = Point::new(0, 0);
        {
            // Layer.cpp:132 — double d2 = std::numeric_limits<double>::max();
            let mut d2 = f64::MAX;
            // Layer.cpp:133-144
            for contour in current_outlines {
                // Layer.cpp:134 — if (contour.size() > 2)
                if contour.points.len() > 2 {
                    // Layer.cpp:135 — Point prev = contour.points.back();
                    let mut prev = *contour.points.last().unwrap();
                    // Layer.cpp:136-143
                    for p2 in &contour.points {
                        // Layer.cpp:137-138 — if (double d = line_alg::distance_to_squared(Line{prev, p2}, unsupported_location, &closest_point); d < d2)
                        let mut closest_point = Point::new(0, 0);
                        let d = line_distance_to_squared(&prev, p2, unsupported_location, &mut closest_point);
                        if d < d2 {
                            // Layer.cpp:139-140
                            d2 = d;
                            node_location = closest_point;
                        }
                        // Layer.cpp:142 — prev = p2;
                        prev = *p2;
                    }
                }
            }
        }

        // Layer.cpp:147 — const auto within_dist = coord_t((node_location - unsupported_location).cast<double>().norm());
        let within_dist: Coord = {
            let dx = (node_location.x - unsupported_location.x) as f64;
            let dy = (node_location.y - unsupported_location.y) as f64;
            (dx * dx + dy * dy).sqrt() as Coord
        };

        // Layer.cpp:149-150
        let mut sub_tree: Option<NodeSPtr> = None;
        let mut current_dist: Coord = get_weighted_distance(&node_location, unsupported_location);
        // Layer.cpp:151 — if (current_dist >= wall_supporting_radius)
        if current_dist >= wall_supporting_radius {
            // Layer.cpp:152 — const coord_t search_radius = std::min(current_dist, within_dist);
            let search_radius = current_dist.min(within_dist);
            // Layer.cpp:153 — BoundingBox region(unsupported_location - Point(search_radius, search_radius), unsupported_location + Point(search_radius + locator_cell_size, search_radius + locator_cell_size));
            let mut region = BoundingBox::from_points_minmax(
                *unsupported_location - Point::new(search_radius, search_radius),
                *unsupported_location
                    + Point::new(
                        search_radius + LOCATOR_CELL_SIZE,
                        search_radius + LOCATOR_CELL_SIZE,
                    ),
            );
            // Layer.cpp:154-155 — region.min/max = to_grid_point(...)
            region.min = to_grid_point(&region.min, current_outlines_bbox);
            region.max = to_grid_point(&region.max, current_outlines_bbox);

            // Layer.cpp:157 — Point current_dist_grid_addr{lowest(), lowest()};
            let mut current_dist_grid_addr = Point::new(Coord::MIN, Coord::MIN);

            // Layer.cpp:159-194 — tbb::parallel_for over blocked_range2d.
            // Serial scan in identical order (rows = y, cols = x) reproduces the
            // non-parallel result the C++ tie-break logic guarantees.
            for grid_addr_y in region.min.y..region.max.y {
                for grid_addr_x in region.min.x..region.max.x {
                    // Layer.cpp:162-164
                    let local_grid_addr = Point::new(grid_addr_x, grid_addr_y);
                    let mut local_sub_tree: Option<NodeSPtr> = None;
                    let mut local_current_dist: Coord = current_dist;
                    // Layer.cpp:165-166 — equal_range(local_grid_addr)
                    if let Some(bucket) = tree_node_locator.get(&local_grid_addr) {
                        for weak in bucket {
                            // Layer.cpp:167 — const NodeSPtr candidate_sub_tree = it->second.lock();
                            let candidate_sub_tree = match weak.upgrade() {
                                Some(c) => c,
                                None => continue,
                            };
                            // Layer.cpp:168-170 —
                            // if ((candidate && candidate != exclude_tree) &&
                            //     !(exclude_tree && exclude_tree->hasOffspring(candidate)) &&
                            //     !polygonCollidesWithLineSegment(unsupported_location, candidate->getLocation(), outline_locator))
                            let not_excluded = match exclude_tree {
                                Some(ex) => !std::rc::Rc::ptr_eq(&candidate_sub_tree, ex),
                                None => true,
                            };
                            let no_offspring = match exclude_tree {
                                Some(ex) => !tree_node::has_offspring(ex, &candidate_sub_tree),
                                None => true,
                            };
                            if not_excluded
                                && no_offspring
                                && !polygon_collides_with_line_segment(
                                    unsupported_location,
                                    &candidate_sub_tree.borrow().get_location(),
                                    outline_locator,
                                )
                            {
                                // Layer.cpp:171 — if (const coord_t candidate_dist = candidate->getWeightedDistance(unsupported_location, supporting_radius); candidate_dist < local_current_dist)
                                let candidate_dist = tree_node::get_weighted_distance(
                                    &candidate_sub_tree,
                                    unsupported_location,
                                    supporting_radius,
                                );
                                if candidate_dist < local_current_dist {
                                    // Layer.cpp:172-173
                                    local_current_dist = candidate_dist;
                                    local_sub_tree = Some(candidate_sub_tree);
                                }
                            }
                        }
                    }
                    // Layer.cpp:183-192 — tie-break/selection under lock.
                    if local_current_dist < current_dist
                        || (local_current_dist == current_dist
                            && (grid_addr_y < current_dist_grid_addr.y
                                || (grid_addr_y == current_dist_grid_addr.y
                                    && grid_addr_x < current_dist_grid_addr.x)))
                    {
                        // Layer.cpp:188-190
                        current_dist = local_current_dist;
                        sub_tree = local_sub_tree;
                        current_dist_grid_addr = local_grid_addr;
                    }
                }
            }
        }

        // Layer.cpp:197-199
        match sub_tree {
            None => GroundingLocation {
                tree_node: None,
                boundary_location: Some(node_location),
            },
            Some(st) => GroundingLocation {
                tree_node: Some(st),
                boundary_location: None,
            },
        }
    }

    // Layer.cpp:202-218 — bool Layer::attach(const Point& unsupported_location, const GroundingLocation& grounding_loc, NodeSPtr& new_child, NodeSPtr& new_root)
    pub fn attach(
        &mut self,
        unsupported_location: &Point,
        grounding_loc: &GroundingLocation,
        new_child: &mut Option<NodeSPtr>,
        new_root: &mut Option<NodeSPtr>,
    ) -> bool {
        // Layer.cpp:209 — if (grounding_loc.boundary_location)
        if grounding_loc.boundary_location.is_some() {
            // Layer.cpp:210 — new_root = Node::create(grounding_loc.p(), std::make_optional(grounding_loc.p()));
            let p = grounding_loc.p();
            let root = Node::create_with_grounding(p, Some(p));
            // Layer.cpp:211 — new_child = new_root->addChild(unsupported_location);
            *new_child = Some(tree_node::add_child_loc(&root, *unsupported_location));
            // Layer.cpp:212 — tree_roots.push_back(new_root);
            self.tree_roots.push(std::rc::Rc::clone(&root));
            *new_root = Some(root);
            // Layer.cpp:213 — return true;
            true
        } else {
            // Layer.cpp:215 — new_child = grounding_loc.tree_node->addChild(unsupported_location);
            let tn = grounding_loc.tree_node.as_ref().unwrap();
            *new_child = Some(tree_node::add_child_loc(tn, *unsupported_location));
            // Layer.cpp:216 — return false;
            false
        }
    }

    // Layer.cpp:220-304 — void Layer::reconnectRoots(...)
    #[allow(clippy::too_many_arguments)]
    pub fn reconnect_roots(
        &mut self,
        to_be_reconnected_tree_roots: &[NodeSPtr],
        current_outlines: &[Polygon],
        current_outlines_bbox: &BoundingBox,
        outline_locator: &EdgeGrid,
        supporting_radius: Coord,
        wall_supporting_radius: Coord,
    ) {
        // Layer.cpp:230 — constexpr coord_t tree_connecting_ignore_offset = 100;
        const TREE_CONNECTING_IGNORE_OFFSET: Coord = 100;

        // Layer.cpp:232-233 — SparseNodeGrid tree_node_locator; fillLocator(tree_node_locator, current_outlines_bbox);
        let mut tree_node_locator: SparseNodeGrid = HashMap::new();
        self.fill_locator(&mut tree_node_locator, current_outlines_bbox);

        // Layer.cpp:235 — const coord_t within_max_dist = outline_locator.resolution() * 2;
        let within_max_dist: Coord = outline_locator.resolution() * 2;
        // Layer.cpp:236-303 — for (const auto &root_ptr : to_be_reconnected_tree_roots)
        for root_ptr in to_be_reconnected_tree_roots {
            // Layer.cpp:238 — auto old_root_it = std::find(tree_roots.begin(), tree_roots.end(), root_ptr);
            let old_root_idx = self
                .tree_roots
                .iter()
                .position(|r| std::rc::Rc::ptr_eq(r, root_ptr));

            // Layer.cpp:240 — if (root_ptr->getLastGroundingLocation())
            let last_grounding = *root_ptr.borrow().get_last_grounding_location();
            if let Some(ground_loc) = last_grounding {
                // Layer.cpp:243 — if (ground_loc != root_ptr->getLocation())
                let root_loc = root_ptr.borrow().get_location();
                if ground_loc != root_loc {
                    // Layer.cpp:245-247 — find intersection of segment, at within_max_dist from ground_loc.
                    let mut new_root_pt = Point::new(0, 0);
                    if tree_node::line_segment_polygons_intersection(
                        &root_loc,
                        &ground_loc,
                        outline_locator,
                        &mut new_root_pt,
                        within_max_dist,
                    ) {
                        // Layer.cpp:248 — auto new_root = Node::create(new_root_pt, new_root_pt);
                        let new_root = Node::create_with_grounding(new_root_pt, Some(new_root_pt));
                        // Layer.cpp:249 — root_ptr->addChild(new_root);
                        tree_node::add_child(root_ptr, &new_root);
                        // Layer.cpp:250 — new_root->reroot();
                        tree_node::reroot(&new_root, None);

                        // Layer.cpp:252 — tree_node_locator.insert(make_pair(to_grid_point(new_root->getLocation()), new_root));
                        let key = to_grid_point(&new_root.borrow().get_location(), current_outlines_bbox);
                        tree_node_locator
                            .entry(key)
                            .or_default()
                            .push(std::rc::Rc::downgrade(&new_root));

                        // Layer.cpp:254 — *old_root_it = std::move(new_root);
                        if let Some(idx) = old_root_idx {
                            self.tree_roots[idx] = new_root;
                        }
                        // Layer.cpp:255 — continue;
                        continue;
                    }
                }
            }

            // Layer.cpp:260 — const coord_t tree_connecting_ignore_width = wall_supporting_radius - tree_connecting_ignore_offset;
            let tree_connecting_ignore_width = wall_supporting_radius - TREE_CONNECTING_IGNORE_OFFSET;
            // Layer.cpp:261-272 — GroundingLocation ground = getBestGroundingLocation(root_ptr->getLocation(), ..., root_ptr);
            let root_loc = root_ptr.borrow().get_location();
            let mut ground = self.get_best_grounding_location(
                &root_loc,
                current_outlines,
                current_outlines_bbox,
                outline_locator,
                supporting_radius,
                tree_connecting_ignore_width,
                &tree_node_locator,
                Some(root_ptr),
            );
            // Layer.cpp:273 — if (ground.boundary_location)
            if let Some(boundary_location) = ground.boundary_location {
                // Layer.cpp:275-276 — if (*ground.boundary_location == root_ptr->getLocation()) continue;
                if boundary_location == root_ptr.borrow().get_location() {
                    continue;
                }

                // Layer.cpp:278 — auto new_root = Node::create(ground.p(), ground.p());
                let gp = ground.p();
                let new_root = Node::create_with_grounding(gp, Some(gp));
                // Layer.cpp:279 — auto attach_ptr = root_ptr->closestNode(new_root->getLocation());
                let attach_ptr =
                    tree_node::closest_node(root_ptr, &new_root.borrow().get_location());
                // Layer.cpp:280 — attach_ptr->reroot();
                tree_node::reroot(&attach_ptr, None);

                // Layer.cpp:282 — new_root->addChild(attach_ptr);
                tree_node::add_child(&new_root, &attach_ptr);
                // Layer.cpp:283 — tree_node_locator.insert(make_pair(to_grid_point(new_root->getLocation()), new_root));
                let key = to_grid_point(&new_root.borrow().get_location(), current_outlines_bbox);
                tree_node_locator
                    .entry(key)
                    .or_default()
                    .push(std::rc::Rc::downgrade(&new_root));

                // Layer.cpp:285 — *old_root_it = std::move(new_root);
                if let Some(idx) = old_root_idx {
                    self.tree_roots[idx] = new_root;
                }
            } else {
                // Layer.cpp:289-292 — asserts about ground.tree_node.
                let ground_tree_node = ground.tree_node.take().unwrap();
                debug_assert!(!std::rc::Rc::ptr_eq(&ground_tree_node, root_ptr));
                debug_assert!(!tree_node::has_offspring(root_ptr, &ground_tree_node));
                debug_assert!(!tree_node::has_offspring(&ground_tree_node, root_ptr));

                // Layer.cpp:294 — auto attach_ptr = root_ptr->closestNode(ground.tree_node->getLocation());
                let attach_ptr =
                    tree_node::closest_node(root_ptr, &ground_tree_node.borrow().get_location());
                // Layer.cpp:295 — attach_ptr->reroot();
                tree_node::reroot(&attach_ptr, None);

                // Layer.cpp:297 — ground.tree_node->addChild(attach_ptr);
                tree_node::add_child(&ground_tree_node, &attach_ptr);

                // Layer.cpp:300-301 — *old_root_it = std::move(tree_roots.back()); tree_roots.pop_back();
                if let Some(idx) = old_root_idx {
                    let back = self.tree_roots.pop().unwrap();
                    if idx < self.tree_roots.len() {
                        self.tree_roots[idx] = back;
                    }
                    // If idx was the last element, pop already removed it (matching
                    // the C++ move-from-back-then-pop on a one-past case).
                }
            }
        }
    }

    // Layer.cpp:436-446 — Polylines Layer::convertToLines(const Polygons& limit_to_outline, const coord_t line_overlap) const
    pub fn convert_to_lines(&self, limit_to_outline: &[Polygon], line_overlap: Coord) -> Vec<Polyline> {
        // Layer.cpp:438-439
        if self.tree_roots.is_empty() {
            return Vec::new();
        }

        // Layer.cpp:441
        let mut result_lines: Vec<Polyline> = Vec::new();
        // Layer.cpp:442-443
        for tree in &self.tree_roots {
            tree_node::convert_to_polylines(tree, &mut result_lines, line_overlap);
        }

        // Layer.cpp:445 — return intersection_pl(result_lines, limit_to_outline);
        crate::clipper2_utils::intersection_pl_2(&result_lines, limit_to_outline)
    }
}

// Layer.cpp:88-115 — static bool polygonCollidesWithLineSegment(const Point &from, const Point &to, const EdgeGrid::Grid &loc_to_line)
fn polygon_collides_with_line_segment(from: &Point, to: &Point, loc_to_line: &EdgeGrid) -> bool {
    // Layer.cpp:90-111 — struct Visitor { ... } visitor(loc_to_line, {from, to});
    // The visitor checks each segment in each intersected cell against the line
    // {from, to}; it stops (returns false) on the first intersection.
    let line_a = *from;
    let line_b = *to;
    let mut intersect = false;
    // Layer.cpp:113 — loc_to_line.visit_cells_intersecting_line(from, to, visitor);
    loc_to_line.visit_cells_intersecting_line(*from, *to, |iy, ix| {
        // Layer.cpp:93-106 — Visitor::operator()(coord_t iy, coord_t ix)
        // Layer.cpp:95 — auto cell_data_range = grid.cell_data_range(iy, ix);
        let cell_data_range = loc_to_line.cell_data_range_at(iy, ix);
        // Layer.cpp:96-103
        for it_contour_and_segment in cell_data_range {
            // Layer.cpp:98 — auto segment = grid.segment(*it_contour_and_segment);
            let segment = loc_to_line.segment(*it_contour_and_segment);
            // Layer.cpp:99 — if (Geometry::segments_intersect(segment.first, segment.second, line.a, line.b))
            if segments_intersect(segment.a, segment.b, line_a, line_b) {
                // Layer.cpp:100-101 — this->intersect = true; return false;
                intersect = true;
                return false;
            }
        }
        // Layer.cpp:105 — Continue traversing the grid along the edge.
        true
    });
    // Layer.cpp:114 — return visitor.intersect;
    intersect
}
