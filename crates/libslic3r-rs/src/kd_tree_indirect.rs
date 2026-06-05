//! KD-tree for N-dimensional spatial search using indirect indexing
//!
//! C++ Reference: KDTreeIndirect.hpp
//!
//! This module provides a generic K-dimensional tree for efficient spatial queries:
//! - Closest point search
//! - K-nearest neighbors search
//! - Range queries (find all points within a distance)
//! - Bounding box queries
//!
//! The tree uses indirect indexing - it stores indices into external data rather
//! than copying the data itself. This allows efficient tree construction and
//! memory usage when working with large datasets.
//!
//! # Type Parameters
//!
//! The tree is generic over:
//! - `N`: Number of dimensions (const generic)
//! - `T`: Coordinate type (typically f64 or i64)
//! - `F`: Coordinate accessor function
//!
//! # Example
//!
//! ```ignore
//! use slicer::kd_tree_indirect::{KDTreeIndirect, find_closest_point};
//!
//! // Build a 2D tree over point data
//! let points = vec![(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)];
//! let coord_fn = |idx: usize, dim: usize| -> f64 {
//!     if dim == 0 { points[idx].0 } else { points[idx].1 }
//! };
//!
//! let mut tree = KDTreeIndirect::<2, f64, _>::new(coord_fn);
//! tree.build(points.len());
//!
//! // Find closest point to (0.5, 0.5)
//! let closest = find_closest_point(&tree, &[0.5, 0.5], |_| true);
//! ```

use crate::libslic3r::EPSILON;
use crate::utils::next_highest_power_of_2;

/// Sentinel value indicating "no position"
/// KDTreeIndirect.hpp:28-30
pub const NPOS: usize = usize::MAX;

/// Visitor return mask controlling tree traversal
/// KDTreeIndirect.hpp:14-17
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VisitorReturnMask {
    /// Continue traversing left subtree
    ContinueLeft = 1,
    /// Continue traversing right subtree
    ContinueRight = 2,
    /// Stop traversal
    Stop = 4,
}

/// KD-tree for N-dimensional spatial search using indirect indexing
/// KDTreeIndirect.hpp:19-176
pub struct KDTreeIndirect<const N: usize, T, F>
where
    T: Copy + PartialOrd,
    F: Fn(usize, usize) -> T,
{
    /// Tree nodes storing indices into external data
    /// KDTreeIndirect.hpp:173
    nodes: Vec<usize>,

    /// Coordinate accessor function
    /// KDTreeIndirect.hpp:82
    coordinate: F,

    /// Phantom data for type parameter
    _phantom: std::marker::PhantomData<T>,
}

impl<const N: usize, T, F> KDTreeIndirect<N, T, F>
where
    T: Copy + PartialOrd + Default + std::ops::AddAssign + std::ops::Mul<Output = T>,
    F: Fn(usize, usize) -> T,
{
    /// Create a new empty KD-tree with the given coordinate accessor
    /// KDTreeIndirect.hpp:32
    pub fn new(coordinate: F) -> Self {
        Self {
            nodes: Vec::new(),
            coordinate,
            _phantom: std::marker::PhantomData,
        }
    }

    /// Create a new KD-tree and build it with the given number of indices
    /// KDTreeIndirect.hpp:34
    pub fn with_indices(coordinate: F, num_indices: usize) -> Self {
        let mut tree = Self::new(coordinate);
        tree.build(num_indices);
        tree
    }

    /// Create a new KD-tree and build it with the given index vector
    /// KDTreeIndirect.hpp:33
    pub fn with_index_vec(coordinate: F, indices: Vec<usize>) -> Self {
        let mut tree = Self::new(coordinate);
        tree.build_from_vec(indices);
        tree
    }

    /// Clear the tree
    /// KDTreeIndirect.hpp:39
    pub fn clear(&mut self) {
        self.nodes.clear();
    }

    /// Build tree from 0..num_indices
    /// KDTreeIndirect.hpp:41-48
    pub fn build(&mut self, num_indices: usize) {
        let indices: Vec<usize> = (0..num_indices).collect();
        self.build_from_vec(indices);
    }

    /// Build tree from index vector
    /// KDTreeIndirect.hpp:49-59
    pub fn build_from_vec(&mut self, mut indices: Vec<usize>) {
        // KDTreeIndirect.hpp:51-52
        if indices.is_empty() {
            self.clear();
        } else {
            // Allocate enough memory for a full binary tree.
            // std::vector::assign replaces the entire contents, resetting every node to npos.
            // KDTreeIndirect.hpp:55
            let size = next_highest_power_of_2(indices.len() + 1);
            self.nodes.clear();
            self.nodes.resize(size, NPOS);
            // KDTreeIndirect.hpp:56
            let indices_len = indices.len();
            self.build_recursive(&mut indices, 0, 0, 0, indices_len - 1);
        }
        // indices.clear(); // KDTreeIndirect.hpp:58 (input vector consumed by value)
        indices.clear();
    }

    /// Check if tree is empty
    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }

    /// Get number of nodes in the tree
    pub fn len(&self) -> usize {
        self.nodes.iter().filter(|&&n| n != NPOS).count()
    }

    /// Build a balanced tree recursively
    /// KDTreeIndirect.hpp:87-113
    fn build_recursive(
        &mut self,
        input: &mut [usize],
        node: usize,
        dimension: usize,
        left: usize,
        right: usize,
    ) {
        if left > right {
            return;
        }

        debug_assert!(node < self.nodes.len());

        if left == right {
            // Insert a node into the balanced tree
            self.nodes[node] = input[left];
            return;
        }

        // Partition to produce a balanced tree
        let center = (left + right) / 2;
        self.partition_input(input, dimension, left, right, center);

        // Insert node into tree
        self.nodes[node] = input[center];

        // Build left/right subtrees
        let next_dimension = (dimension + 1) % N;
        if center > left {
            self.build_recursive(input, node * 2 + 1, next_dimension, left, center - 1);
        }
        self.build_recursive(input, node * 2 + 2, next_dimension, center + 1, right);
    }

    // Partition the input m_nodes <left, right> at "k" and "dimension" using the QuickSelect method:
    // https://en.wikipedia.org/wiki/Quickselect
    // Items left of the k'th item are lower than the k'th item in the "dimension",
    // items right of the k'th item are higher than the k'th item in the "dimension",
    /// KDTreeIndirect.hpp:114-167
    fn partition_input(
        &self,
        input: &mut [usize],
        dimension: usize,
        mut left: usize,
        mut right: usize,
        k: usize,
    ) {
        while left < right {
            // KDTreeIndirect.hpp:117
            let center = (left + right) / 2;
            let pivot: T;
            {
                // Bubble sort the input[left], input[center], input[right], so that a median of the three values
                // will end up in input[center].
                // KDTreeIndirect.hpp:122-124
                let mut left_value = (self.coordinate)(input[left], dimension);
                let mut center_value = (self.coordinate)(input[center], dimension);
                let mut right_value = (self.coordinate)(input[right], dimension);
                // KDTreeIndirect.hpp:125-128
                if left_value > center_value {
                    input.swap(left, center);
                    std::mem::swap(&mut left_value, &mut center_value);
                }
                // KDTreeIndirect.hpp:129-132
                if left_value > right_value {
                    input.swap(left, right);
                    right_value = left_value;
                }
                // KDTreeIndirect.hpp:133-136
                if center_value > right_value {
                    input.swap(center, right);
                    center_value = right_value;
                }
                // KDTreeIndirect.hpp:137
                pivot = center_value;
            }
            // KDTreeIndirect.hpp:139-141
            if right <= left + 2 {
                // The <left, right> interval is already sorted.
                break;
            }
            // KDTreeIndirect.hpp:142-144
            let mut i = left;
            let mut j = right - 1;
            input.swap(center, j);
            // Partition the set based on the pivot.
            // KDTreeIndirect.hpp:146
            loop {
                // Skip left points that are already at correct positions.
                // Search will certainly stop at position (right - 1), which stores the pivot.
                // KDTreeIndirect.hpp:149
                loop {
                    i += 1;
                    if !((self.coordinate)(input[i], dimension) < pivot) {
                        break;
                    }
                }
                // Skip right points that are already at correct positions.
                // KDTreeIndirect.hpp:151
                loop {
                    j -= 1;
                    if !((self.coordinate)(input[j], dimension) > pivot && i < j) {
                        break;
                    }
                }
                // KDTreeIndirect.hpp:152-153
                if i >= j {
                    break;
                }
                // KDTreeIndirect.hpp:154
                input.swap(i, j);
            }
            // Restore pivot to the center of the sequence.
            // KDTreeIndirect.hpp:157
            input.swap(i, right - 1);
            // Which side the kth element is in?
            // KDTreeIndirect.hpp:159-165
            if k < i {
                right = i - 1;
            } else if k == i {
                // Sequence is partitioned, kth element is at its place.
                break;
            } else {
                left = i + 1;
            }
        }
    }

    /// Calculate descent mask for tree traversal
    /// KDTreeIndirect.hpp:61-70
    pub fn descent_mask(
        &self,
        point_coord: T,
        search_radius: T,
        idx: usize,
        dimension: usize,
    ) -> u32
    where
        T: std::ops::Sub<Output = T> + std::ops::Add<Output = T> + From<f64>,
    {
        // KDTreeIndirect.hpp:64
        let dist = point_coord - (self.coordinate)(idx, dimension);
        // KDTreeIndirect.hpp:65-69
        if dist * dist < search_radius + T::from(EPSILON) {
            // The plane intersects a hypersphere centered at point_coord of search_radius.
            (VisitorReturnMask::ContinueLeft as u32) | (VisitorReturnMask::ContinueRight as u32)
        } else if dist > T::default() {
            // The plane does not intersect the hypersphere.
            VisitorReturnMask::ContinueRight as u32
        } else {
            VisitorReturnMask::ContinueLeft as u32
        }
    }

    /// Visit tree nodes with a visitor function
    /// KDTreeIndirect.hpp:75-79
    pub fn visit<V>(&self, mut visitor: V)
    where
        V: FnMut(usize, usize) -> u32,
    {
        if !self.nodes.is_empty() {
            self.visit_recursive(0, 0, &mut visitor);
        }
    }

    /// Visit tree recursively
    /// KDTreeIndirect.hpp:177-199
    fn visit_recursive<V>(&self, node: usize, dimension: usize, visitor: &mut V)
    where
        V: FnMut(usize, usize) -> u32,
    {
        if node >= self.nodes.len() || self.nodes[node] == NPOS {
            return;
        }

        let left = node * 2 + 1;
        let right = left + 1;

        let mask = visitor(self.nodes[node], dimension);

        if (mask & (VisitorReturnMask::Stop as u32)) == 0 {
            let next_dimension = (dimension + 1) % N;

            if (mask & (VisitorReturnMask::ContinueLeft as u32)) != 0 {
                self.visit_recursive(left, next_dimension, visitor);
            }
            if (mask & (VisitorReturnMask::ContinueRight as u32)) != 0 {
                self.visit_recursive(right, next_dimension, visitor);
            }
        }
    }
}

/// Find K closest points to a target point
/// KDTreeIndirect.hpp:202-263
pub fn find_closest_points<const K: usize, const N: usize, T, F, P, Filter>(
    kdtree: &KDTreeIndirect<N, T, F>,
    point: &P,
    filter: Filter,
) -> [usize; K]
where
    T: Copy
        + PartialOrd
        + Default
        + std::ops::AddAssign
        + std::ops::Sub<Output = T>
        + std::ops::Add<Output = T>
        + std::ops::Mul<Output = T>
        + From<f64>,
    F: Fn(usize, usize) -> T,
    P: std::ops::Index<usize, Output = T>,
    Filter: Fn(usize) -> bool,
{
    // results.fill(std::make_pair(npos, numeric_limits<CoordT>::max()));
    // KDTreeIndirect.hpp:218-220
    let mut results = [(NPOS, T::from(f64::MAX)); K];

    // KDTreeIndirect.hpp:221-244
    let mut visitor = |idx: usize, dimension: usize| -> u32 {
        if filter(idx) {
            // KDTreeIndirect.hpp:224-228
            let mut dist = T::default();
            for i in 0..N {
                let d = point[i] - (kdtree.coordinate)(idx, i);
                dist += d * d;
            }

            // auto res = std::make_pair(idx, dist);
            // KDTreeIndirect.hpp:230
            let res = (idx, dist);
            // auto it = std::lower_bound(results.begin(), results.end(), res,
            //     [](auto &r1, auto &r2) { return r1.second < r2.second; });
            // lower_bound returns the first position whose value does not compare less than dist,
            // i.e. the first i for which !(results[i].1 < dist).
            // KDTreeIndirect.hpp:231-234
            let mut it = K;
            for i in 0..K {
                if !(results[i].1 < dist) {
                    it = i;
                    break;
                }
            }

            // KDTreeIndirect.hpp:236-239
            if it != K {
                // std::rotate(it, std::prev(results.end()), results.end());
                // Move the last element to position `it`, shifting [it, K-1) right by one.
                for i in ((it + 1)..K).rev() {
                    results[i] = results[i - 1];
                }
                // *it = res;
                results[it] = res;
            }
        }

        // KDTreeIndirect.hpp:241-243
        kdtree.descent_mask(point[dimension], results[0].1, idx, dimension)
    };

    // KDTreeIndirect.hpp:247
    kdtree.visit(visitor);
    // KDTreeIndirect.hpp:248-249
    let mut ret = [NPOS; K];
    for i in 0..K {
        ret[i] = results[i].0;
    }
    // KDTreeIndirect.hpp:251
    ret
}

/// Find K closest points without filter
/// KDTreeIndirect.hpp:265-270
pub fn find_closest_points_unfiltered<const K: usize, const N: usize, T, F, P>(
    kdtree: &KDTreeIndirect<N, T, F>,
    point: &P,
) -> [usize; K]
where
    T: Copy
        + PartialOrd
        + Default
        + std::ops::AddAssign
        + std::ops::Sub<Output = T>
        + std::ops::Add<Output = T>
        + std::ops::Mul<Output = T>
        + From<f64>,
    F: Fn(usize, usize) -> T,
    P: std::ops::Index<usize, Output = T>,
{
    find_closest_points(kdtree, point, |_| true)
}

/// Find single closest point with filter
/// KDTreeIndirect.hpp:272-281
pub fn find_closest_point<const N: usize, T, F, P, Filter>(
    kdtree: &KDTreeIndirect<N, T, F>,
    point: &P,
    filter: Filter,
) -> usize
where
    T: Copy
        + PartialOrd
        + Default
        + std::ops::AddAssign
        + std::ops::Sub<Output = T>
        + std::ops::Add<Output = T>
        + std::ops::Mul<Output = T>
        + From<f64>,
    F: Fn(usize, usize) -> T,
    P: std::ops::Index<usize, Output = T>,
    Filter: Fn(usize) -> bool,
{
    find_closest_points::<1, N, T, F, P, Filter>(kdtree, point, filter)[0]
}

/// Find single closest point without filter
/// KDTreeIndirect.hpp:283-287
pub fn find_closest_point_unfiltered<const N: usize, T, F, P>(
    kdtree: &KDTreeIndirect<N, T, F>,
    point: &P,
) -> usize
where
    T: Copy
        + PartialOrd
        + Default
        + std::ops::AddAssign
        + std::ops::Sub<Output = T>
        + std::ops::Add<Output = T>
        + std::ops::Mul<Output = T>
        + From<f64>,
    F: Fn(usize, usize) -> T,
    P: std::ops::Index<usize, Output = T>,
{
    find_closest_point(kdtree, point, |_| true)
}

/// Find all points within a spherical radius
/// KDTreeIndirect.hpp:290-328
pub fn find_nearby_points<const N: usize, T, F, P, Filter>(
    kdtree: &KDTreeIndirect<N, T, F>,
    center: &P,
    max_distance: T,
    filter: Filter,
) -> Vec<usize>
where
    T: Copy
        + PartialOrd
        + Default
        + std::ops::AddAssign
        + std::ops::Sub<Output = T>
        + std::ops::Add<Output = T>
        + std::ops::Mul<Output = T>
        + From<f64>,
    F: Fn(usize, usize) -> T,
    P: std::ops::Index<usize, Output = T>,
    Filter: Fn(usize) -> bool,
{
    let max_distance_squared = max_distance * max_distance;
    let mut result = Vec::new();

    let mut visitor = |idx: usize, dimension: usize| -> u32 {
        if filter(idx) {
            let mut dist = T::default();
            for i in 0..N {
                let d = center[i] - (kdtree.coordinate)(idx, i);
                dist += d * d;
            }
            if dist < max_distance_squared {
                result.push(idx);
            }
        }

        kdtree.descent_mask(center[dimension], max_distance_squared, idx, dimension)
    };

    kdtree.visit(visitor);
    result
}

/// Find all points within a spherical radius (no filter)
/// KDTreeIndirect.hpp:330-337
pub fn find_nearby_points_unfiltered<const N: usize, T, F, P>(
    kdtree: &KDTreeIndirect<N, T, F>,
    center: &P,
    max_distance: T,
) -> Vec<usize>
where
    T: Copy
        + PartialOrd
        + Default
        + std::ops::AddAssign
        + std::ops::Sub<Output = T>
        + std::ops::Add<Output = T>
        + std::ops::Mul<Output = T>
        + From<f64>,
    F: Fn(usize, usize) -> T,
    P: std::ops::Index<usize, Output = T>,
{
    find_nearby_points(kdtree, center, max_distance, |_| true)
}

/// Find all points within an axis-aligned bounding box
/// KDTreeIndirect.hpp:340-387
pub fn find_nearby_points_bbox<const N: usize, T, F, P, Filter>(
    kdtree: &KDTreeIndirect<N, T, F>,
    bb_min: &P,
    bb_max: &P,
    filter: Filter,
) -> Vec<usize>
where
    T: Copy + PartialOrd + Default + std::ops::AddAssign + std::ops::Mul<Output = T>,
    F: Fn(usize, usize) -> T,
    P: std::ops::Index<usize, Output = T>,
    Filter: Fn(usize) -> bool,
{
    let mut result = Vec::new();

    let mut visitor = |idx: usize, dimension: usize| -> u32 {
        let mut ret =
            (VisitorReturnMask::ContinueLeft as u32) | (VisitorReturnMask::ContinueRight as u32);

        if filter(idx) {
            let mut contains = true;
            let mut p_dim = T::default();

            for i in 0..N {
                let p = (kdtree.coordinate)(idx, i);
                if i == dimension {
                    p_dim = p;
                }
                contains = contains && bb_min[i] <= p && p <= bb_max[i];
            }

            if p_dim < bb_min[dimension] {
                ret = VisitorReturnMask::ContinueRight as u32;
            }
            if p_dim > bb_max[dimension] {
                ret = VisitorReturnMask::ContinueLeft as u32;
            }

            if contains {
                result.push(idx);
            }
        }

        ret
    };

    kdtree.visit(visitor);
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_kdtree_2d_creation() {
        let points = vec![(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)];
        let coord_fn = |idx: usize, dim: usize| -> f64 {
            if dim == 0 {
                points[idx].0
            } else {
                points[idx].1
            }
        };

        let tree = KDTreeIndirect::<2, f64, _>::with_indices(coord_fn, points.len());
        assert_eq!(tree.len(), 3);
        assert!(!tree.is_empty());
    }

    #[test]
    fn test_kdtree_clear() {
        let points = vec![(0.0, 0.0), (1.0, 1.0)];
        let coord_fn = |idx: usize, dim: usize| -> f64 {
            if dim == 0 {
                points[idx].0
            } else {
                points[idx].1
            }
        };

        let mut tree = KDTreeIndirect::<2, f64, _>::with_indices(coord_fn, points.len());
        assert_eq!(tree.len(), 2);

        tree.clear();
        assert_eq!(tree.len(), 0);
        assert!(tree.is_empty());
    }

    #[test]
    fn test_find_closest_point() {
        let points = vec![(0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0)];
        let coord_fn = |idx: usize, dim: usize| -> f64 {
            if dim == 0 {
                points[idx].0
            } else {
                points[idx].1
            }
        };

        let tree = KDTreeIndirect::<2, f64, _>::with_indices(coord_fn, points.len());

        // Find closest to (0.1, 0.1) - should be (0, 0)
        let query = [0.1, 0.1];
        let closest = find_closest_point_unfiltered(&tree, &query);

        assert_eq!(closest, 0);
    }

    #[test]
    fn test_find_k_closest_points() {
        let points = vec![(0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (2.0, 0.0), (0.0, 2.0)];
        let coord_fn = |idx: usize, dim: usize| -> f64 {
            if dim == 0 {
                points[idx].0
            } else {
                points[idx].1
            }
        };

        let tree = KDTreeIndirect::<2, f64, _>::with_indices(coord_fn, points.len());

        // Find 3 closest to origin
        let query = [0.0, 0.0];
        let closest = find_closest_points_unfiltered::<3, 2, _, _, _>(&tree, &query);

        assert_eq!(closest[0], 0); // (0, 0) - distance 0
                                   // Next two should be (1, 0) and (0, 1) in some order
        assert!(closest[1] == 1 || closest[1] == 2);
        assert!(closest[2] == 1 || closest[2] == 2);
        assert_ne!(closest[1], closest[2]);
    }

    #[test]
    fn test_find_nearby_points() {
        let points = vec![(0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (5.0, 5.0)];
        let coord_fn = |idx: usize, dim: usize| -> f64 {
            if dim == 0 {
                points[idx].0
            } else {
                points[idx].1
            }
        };

        let tree = KDTreeIndirect::<2, f64, _>::with_indices(coord_fn, points.len());

        // Find all points within distance 1.5 of origin
        let center = [0.0, 0.0];
        let nearby = find_nearby_points_unfiltered(&tree, &center, 1.5);

        assert_eq!(nearby.len(), 3);
        assert!(nearby.contains(&0));
        assert!(nearby.contains(&1));
        assert!(nearby.contains(&2));
        assert!(!nearby.contains(&3)); // (5, 5) is too far
    }

    #[test]
    fn test_find_nearby_points_bbox() {
        let points = vec![(0.0, 0.0), (1.0, 1.0), (2.0, 2.0), (5.0, 5.0)];
        let coord_fn = |idx: usize, dim: usize| -> f64 {
            if dim == 0 {
                points[idx].0
            } else {
                points[idx].1
            }
        };

        let tree = KDTreeIndirect::<2, f64, _>::with_indices(coord_fn, points.len());

        // Find all points in bbox [0, 0] to [2.5, 2.5]
        let bb_min = [0.0, 0.0];
        let bb_max = [2.5, 2.5];
        let in_bbox = find_nearby_points_bbox(&tree, &bb_min, &bb_max, |_| true);

        assert_eq!(in_bbox.len(), 3);
        assert!(in_bbox.contains(&0));
        assert!(in_bbox.contains(&1));
        assert!(in_bbox.contains(&2));
        assert!(!in_bbox.contains(&3)); // (5, 5) is outside
    }

    #[test]
    fn test_filter_functionality() {
        let points = vec![(0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0)];
        let coord_fn = |idx: usize, dim: usize| -> f64 {
            if dim == 0 {
                points[idx].0
            } else {
                points[idx].1
            }
        };

        let tree = KDTreeIndirect::<2, f64, _>::with_indices(coord_fn, points.len());

        // Find closest point with even index
        let query = [0.5, 0.5];
        let closest = find_closest_point(&tree, &query, |idx| idx % 2 == 0);

        assert!(closest == 0 || closest == 2);
    }

    #[test]
    fn test_3d_tree() {
        let points = vec![
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        ];
        let coord_fn = |idx: usize, dim: usize| -> f64 {
            match dim {
                0 => points[idx].0,
                1 => points[idx].1,
                2 => points[idx].2,
                _ => 0.0,
            }
        };

        let tree = KDTreeIndirect::<3, f64, _>::with_indices(coord_fn, points.len());
        assert_eq!(tree.len(), 4);

        let query = [0.1, 0.1, 0.1];
        let closest = find_closest_point_unfiltered(&tree, &query);
        assert_eq!(closest, 0);
    }

    #[test]
    fn test_empty_tree() {
        let points: Vec<(f64, f64)> = vec![];
        let coord_fn = |idx: usize, dim: usize| -> f64 {
            if dim == 0 {
                points[idx].0
            } else {
                points[idx].1
            }
        };

        let tree = KDTreeIndirect::<2, f64, _>::with_indices(coord_fn, 0);
        assert_eq!(tree.len(), 0);
        assert!(tree.is_empty());
    }

    #[test]
    fn test_single_point() {
        let points = vec![(1.0, 2.0)];
        let coord_fn = |idx: usize, dim: usize| -> f64 {
            if dim == 0 {
                points[idx].0
            } else {
                points[idx].1
            }
        };

        let tree = KDTreeIndirect::<2, f64, _>::with_indices(coord_fn, 1);
        assert_eq!(tree.len(), 1);

        let query = [0.0, 0.0];
        let closest = find_closest_point_unfiltered(&tree, &query);
        assert_eq!(closest, 0);
    }
}
