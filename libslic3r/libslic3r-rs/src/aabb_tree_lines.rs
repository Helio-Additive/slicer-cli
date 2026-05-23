//! AABBTreeLines - 2D line segment spatial acceleration structure
//!
//! Provides AABB tree for fast queries on 2D line segments.
//! C++ Reference: AABBTreeLines.hpp

use crate::geometry::{BoundingBox, Line, Point};
use crate::Coord;

/// AABB tree for 2D line segments
/// AABBTreeLines.hpp:295-360
pub struct AABBTreeLines {
    lines: Vec<Line>,
    tree: AABBTree,
}

/// AABB tree for 2D spatial queries
/// AABBTreeLines.hpp:167-200
#[derive(Debug, Clone)]
struct AABBTree {
    nodes: Vec<AABBNode>,
    root: usize,
}

/// AABB tree node for 2D
/// AABBTreeLines.hpp (internal structure)
#[derive(Debug, Clone)]
struct AABBNode {
    min: Point,
    max: Point,
    left: usize,
    right: usize,
    line_index: Option<usize>,
}

impl AABBTreeLines {
    /// Create from a set of lines
    /// AABBTreeLines.hpp:305-308
    pub fn from_lines(lines: Vec<Line>) -> Self {
        let tree = AABBTree::build(&lines);
        Self { lines, tree }
    }

    /// Find all lines intersecting a query point
    /// AABBTreeLines.hpp (query operations)
    pub fn query_point(&self, point: Point) -> Vec<(usize, &Line)> {
        self.tree
            .query_point(point, &self.lines)
            .into_iter()
            .map(|idx| (idx, &self.lines[idx]))
            .collect()
    }

    /// Find all lines within a distance of a query point
    /// AABBTreeLines.hpp:347-350
    pub fn query_distance(&self, point: Point, max_distance: Coord) -> Vec<(usize, &Line, Coord)> {
        self.tree
            .query_distance(point, max_distance, &self.lines)
            .into_iter()
            .map(|(idx, dist)| (idx, &self.lines[idx], dist))
            .collect()
    }

    /// Find the closest line to a query point
    /// AABBTreeLines.hpp:341-345
    pub fn closest_line(&self, point: Point) -> Option<(usize, &Line, Coord)> {
        self.tree
            .closest_line(point, &self.lines)
            .map(|(idx, dist)| (idx, &self.lines[idx], dist))
    }
}

impl AABBTree {
    /// Build AABB tree over indexed lines
    /// AABBTreeLines.hpp:167-200
    fn build(lines: &[Line]) -> Self {
        if lines.is_empty() {
            return Self {
                nodes: vec![AABBNode::empty()],
                root: 0,
            };
        }

        let mut line_bounds: Vec<(usize, BoundingBox)> = lines
            .iter()
            .enumerate()
            .map(|(idx, line)| {
                let mut bbox = BoundingBox::new();
                bbox.merge_point(line.a);
                bbox.merge_point(line.b);
                (idx, bbox)
            })
            .collect();

        let mut nodes = Vec::new();
        let lb_len = line_bounds.len();
        let root = Self::build_recursive(&mut nodes, &mut line_bounds, 0, lb_len);

        Self { nodes, root }
    }

    /// Recursive tree building with axis-aligned partitioning
    /// AABBTreeLines.hpp:167-200 (internal recursion)
    fn build_recursive(
        nodes: &mut Vec<AABBNode>,
        line_bounds: &mut [(usize, BoundingBox)],
        start: usize,
        end: usize,
    ) -> usize {
        let count = end - start;

        if count == 0 {
            nodes.push(AABBNode::empty());
            return nodes.len() - 1;
        }

        if count == 1 {
            let (line_idx, bbox) = &line_bounds[start];
            nodes.push(AABBNode::leaf(*line_idx, bbox.min, bbox.max));
            return nodes.len() - 1;
        }

        let mut overall_min = line_bounds[start].1.min;
        let mut overall_max = line_bounds[start].1.max;

        for i in (start + 1)..end {
            overall_min.x = overall_min.x.min(line_bounds[i].1.min.x);
            overall_min.y = overall_min.y.min(line_bounds[i].1.min.y);
            overall_max.x = overall_max.x.max(line_bounds[i].1.max.x);
            overall_max.y = overall_max.y.max(line_bounds[i].1.max.y);
        }

        let size_x = overall_max.x - overall_min.x;
        let size_y = overall_max.y - overall_min.y;

        let axis = if size_x >= size_y { 0 } else { 1 };

        line_bounds[start..end].sort_by(|a, b| {
            let center_a = (a.1.min.get_axis(axis) + a.1.max.get_axis(axis)) / 2;
            let center_b = (b.1.min.get_axis(axis) + b.1.max.get_axis(axis)) / 2;
            center_a.cmp(&center_b)
        });

        let mid = start + count / 2;
        let left = Self::build_recursive(nodes, line_bounds, start, mid);
        let right = Self::build_recursive(nodes, line_bounds, mid, end);

        nodes.push(AABBNode::internal(left, right, overall_min, overall_max));
        nodes.len() - 1
    }

    /// Query point against tree
    /// AABBTreeLines.hpp (query operations)
    fn query_point(&self, point: Point, lines: &[Line]) -> Vec<usize> {
        let mut result = Vec::new();
        self.query_point_recursive(self.root, point, lines, &mut result);
        result
    }

    fn query_point_recursive(
        &self,
        node_idx: usize,
        point: Point,
        lines: &[Line],
        result: &mut Vec<usize>,
    ) {
        let Some(node) = self.nodes.get(node_idx) else {
            return;
        };

        if !point_in_bbox(point, node.min, node.max) {
            return;
        }

        if let Some(line_idx) = node.line_index {
            if let Some(line) = lines.get(line_idx) {
                if point_on_line(point, line) {
                    result.push(line_idx);
                }
            }
            return;
        }

        self.query_point_recursive(node.left, point, lines, result);
        self.query_point_recursive(node.right, point, lines, result);
    }

    /// Query lines within distance
    /// AABBTreeLines.hpp:347-350
    fn query_distance(
        &self,
        point: Point,
        max_distance: Coord,
        lines: &[Line],
    ) -> Vec<(usize, Coord)> {
        let mut result = Vec::new();
        self.query_distance_recursive(self.root, point, max_distance, lines, &mut result);
        result
    }

    fn query_distance_recursive(
        &self,
        node_idx: usize,
        point: Point,
        max_distance: Coord,
        lines: &[Line],
        result: &mut Vec<(usize, Coord)>,
    ) {
        let Some(node) = self.nodes.get(node_idx) else {
            return;
        };

        let dist_to_bbox = point_bbox_distance(point, node.min, node.max);
        if dist_to_bbox > max_distance * max_distance {
            return;
        }

        if let Some(line_idx) = node.line_index {
            if let Some(line) = lines.get(line_idx) {
                let dist = point_to_line_distance_sq(point, line);
                if dist <= max_distance * max_distance {
                    result.push((line_idx, dist));
                }
            }
            return;
        }

        self.query_distance_recursive(node.left, point, max_distance, lines, result);
        self.query_distance_recursive(node.right, point, max_distance, lines, result);
    }

    /// Find closest line to point
    /// AABBTreeLines.hpp:322-339
    fn closest_line(&self, point: Point, lines: &[Line]) -> Option<(usize, Coord)> {
        self.closest_line_recursive(self.root, point, lines, None)
    }

    fn closest_line_recursive(
        &self,
        node_idx: usize,
        point: Point,
        lines: &[Line],
        best: Option<(usize, Coord)>,
    ) -> Option<(usize, Coord)> {
        let node = self.nodes.get(node_idx)?;

        let dist_to_bbox = point_bbox_distance(point, node.min, node.max);
        if let Some((_, best_dist)) = best {
            if dist_to_bbox > best_dist {
                return best;
            }
        }

        if let Some(line_idx) = node.line_index {
            let line = lines.get(line_idx)?;
            let dist = point_to_line_distance_sq(point, line);
            return match best {
                Some((_, best_dist)) if dist < best_dist => Some((line_idx, dist)),
                None => Some((line_idx, dist)),
                _ => best,
            };
        }

        let (first, second) = {
            let first_dist =
                point_bbox_distance(point, self.nodes[node.left].min, self.nodes[node.left].max);
            let second_dist = point_bbox_distance(
                point,
                self.nodes[node.right].min,
                self.nodes[node.right].max,
            );
            if first_dist < second_dist {
                (node.left, node.right)
            } else {
                (node.right, node.left)
            }
        };

        let first_result = self.closest_line_recursive(first, point, lines, best);
        let best_dist = first_result.map_or(Coord::MAX, |(_, d)| d);
        let second_result = self.closest_line_recursive(second, point, lines, Some((0, best_dist)));

        match (first_result, second_result) {
            (Some(f), Some(s)) => Some(if f.1 < s.1 { f } else { s }),
            (Some(f), None) => Some(f),
            (None, Some(s)) => Some(s),
            (None, None) => None,
        }
    }
}

impl AABBNode {
    /// Create empty node
    /// AABBTreeLines.hpp (internal)
    fn empty() -> Self {
        Self {
            min: Point::new(0, 0),
            max: Point::new(0, 0),
            left: 0,
            right: 0,
            line_index: None,
        }
    }

    /// Create leaf node
    /// AABBTreeLines.hpp (internal)
    fn leaf(line_idx: usize, min: Point, max: Point) -> Self {
        Self {
            min,
            max,
            left: 0,
            right: 0,
            line_index: Some(line_idx),
        }
    }

    /// Create internal node
    /// AABBTreeLines.hpp (internal)
    fn internal(left: usize, right: usize, min: Point, max: Point) -> Self {
        Self {
            min,
            max,
            left,
            right,
            line_index: None,
        }
    }
}

impl Point {
    fn get_axis(&self, axis: usize) -> Coord {
        if axis == 0 {
            self.x
        } else {
            self.y
        }
    }
}

/// Check if point is inside bounding box
/// AABBTreeLines.hpp (geometric utilities)
fn point_in_bbox(point: Point, min: Point, max: Point) -> bool {
    point.x >= min.x && point.x <= max.x && point.y >= min.y && point.y <= max.y
}

/// Check if point lies on line segment
/// AABBTreeLines.hpp (geometric utilities)
fn point_on_line(point: Point, line: &Line) -> bool {
    let ab = line.b - line.a;
    let ap = point - line.a;
    let cross = ab.x * ap.y - ab.y * ap.x;

    if cross != 0 {
        return false;
    }

    let dot = ap.x * ab.x + ap.y * ab.y;
    let len_sq = ab.x * ab.x + ab.y * ab.y;

    dot >= 0 && dot <= len_sq
}

/// Squared distance from point to bounding box
/// AABBTreeLines.hpp:206-219
fn point_bbox_distance(point: Point, min: Point, max: Point) -> Coord {
    let dx = if point.x < min.x {
        min.x - point.x
    } else if point.x > max.x {
        point.x - max.x
    } else {
        0
    };

    let dy = if point.y < min.y {
        min.y - point.y
    } else if point.y > max.y {
        point.y - max.y
    } else {
        0
    };

    dx * dx + dy * dy
}

/// Squared distance from point to line segment
/// AABBTreeLines.hpp:29-35
fn point_to_line_distance_sq(point: Point, line: &Line) -> Coord {
    let ab = line.b - line.a;
    let ap = point - line.a;

    let ab_len_sq = ab.x * ab.x + ab.y * ab.y;
    if ab_len_sq == 0 {
        let dx = point.x - line.a.x;
        let dy = point.y - line.a.y;
        return dx * dx + dy * dy;
    }

    let t = ((ap.x * ab.x + ap.y * ab.y) as f64 / ab_len_sq as f64).clamp(0.0, 1.0);
    let t = (t * Coord::MAX as f64) as Coord;

    let closest = Point::new(
        line.a.x + (t * ab.x) / Coord::MAX,
        line.a.y + (t * ab.y) / Coord::MAX,
    );

    let dx = point.x - closest.x;
    let dy = point.y - closest.y;
    dx * dx + dy * dy
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_aabb_tree_lines() {
        let lines = vec![
            Line::new(Point::new(0, 0), Point::new(10, 0)),
            Line::new(Point::new(5, 5), Point::new(15, 5)),
        ];

        let tree = AABBTreeLines::from_lines(lines);
        let results = tree.query_point(Point::new(5, 0));
        assert_eq!(results.len(), 1);
    }
}
