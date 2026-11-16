//! AABB tree built upon external data set, referencing the external data by integer indices.
//!
//! The AABB tree balancing and traversal (ray casting, closest triangle of an indexed triangle mesh)
//! were adapted from libigl AABB.{cpp,hpp} Copyright (C) 2015 Alec Jacobson <alecjacobson@gmail.com>
//! while the implicit balanced tree representation and memory optimizations are Vojtech's.
//!
//! C++ Reference: AABBTreeIndirect.hpp

use crate::geometry::aabb_tree::Vec3;
use crate::geometry::{BoundingBox3F, Point3F};

/// Special index values for AABB tree nodes
/// AABBTreeIndirect.hpp:47-52
pub mod constants {
    // Node is not used
    // AABBTreeIndirect.hpp:49
    pub const NPOS: usize = usize::MAX;

    // Inner node (not leaf)
    // AABBTreeIndirect.hpp:51
    pub const INNER: usize = usize::MAX - 1;
}

/// Single node of the implicit balanced AABB tree
/// AABBTreeIndirect.hpp:56-72
#[derive(Debug, Clone)]
pub struct Node {
    // Index of the external source entity, NPOS for internal nodes
    // AABBTreeIndirect.hpp:58
    pub idx: usize,

    // Bounding box around this entity, possibly with epsilons applied
    // AABBTreeIndirect.hpp:61
    pub bbox: BoundingBox3F,
}

impl Node {
    // Check if node is valid (not NPOS)
    // AABBTreeIndirect.hpp:63
    pub fn is_valid(&self) -> bool {
        // AABBTreeIndirect.hpp:63
        self.idx != constants::NPOS
    }

    // Check if this is an inner node
    // AABBTreeIndirect.hpp:64
    pub fn is_inner(&self) -> bool {
        // AABBTreeIndirect.hpp:64
        self.idx == constants::INNER
    }

    // Check if this is a leaf node
    // AABBTreeIndirect.hpp:65
    pub fn is_leaf(&self) -> bool {
        // AABBTreeIndirect.hpp:65
        !self.is_inner()
    }
}

/// Source node trait for building AABB tree
/// AABBTreeIndirect.hpp:76-90
pub trait SourceNode {
    // Index to the outside entity (triangle, edge, point etc)
    // AABBTreeIndirect.hpp:78-79
    fn idx(&self) -> usize;

    // Centroid of this node, used for balancing the tree
    // AABBTreeIndirect.hpp:80-81
    fn centroid(&self) -> Point3F;

    // Bounding box of this node, likely expanded with epsilon
    // AABBTreeIndirect.hpp:82-85
    fn bbox(&self) -> BoundingBox3F;
}

/// Static balanced AABB tree for raycasting and closest triangle search
/// AABBTreeIndirect.hpp:39-214
#[derive(Debug, Clone)]
pub struct Tree {
    // The balanced tree storage - nodes addressed implicitly using power of two rule
    // AABBTreeIndirect.hpp:213
    nodes: Vec<Node>,
}

impl Tree {
    // Create an empty tree
    // AABBTreeIndirect.hpp:74
    pub fn new() -> Self {
        Self { nodes: Vec::new() }
    }

    // Clear the tree
    // AABBTreeIndirect.hpp:74
    pub fn clear(&mut self) {
        // AABBTreeIndirect.hpp:74
        self.nodes.clear();
    }

    // Build tree from input, consuming the input vector
    // AABBTreeIndirect.hpp:86-90
    pub fn build<S: SourceNode>(&mut self, mut input: Vec<S>) {
        // AABBTreeIndirect.hpp:87-90
        self.build_modify_input(&mut input);
        input.clear();
    }

    // Build tree from input, modifying it in place
    // AABBTreeIndirect.hpp:93-102
    pub fn build_modify_input<S: SourceNode>(&mut self, input: &mut [S]) {
        // AABBTreeIndirect.hpp:94-101
        if input.is_empty() {
            // AABBTreeIndirect.hpp:95
            self.clear();
        } else {
            // Allocate enough memory for a full binary tree
            // AABBTreeIndirect.hpp:98
            let capacity = next_highest_power_of_2(input.len()) * 2 - 1;

            // AABBTreeIndirect.hpp:98
            self.nodes = vec![
                Node {
                    idx: constants::NPOS,
                    bbox: BoundingBox3F::new_empty(),
                };
                capacity
            ];

            // AABBTreeIndirect.hpp:99
            self.build_recursive(input, 0, 0, input.len() - 1);
        }
    }

    // Get reference to nodes vector
    // AABBTreeIndirect.hpp:104
    pub fn nodes(&self) -> &[Node] {
        // AABBTreeIndirect.hpp:104
        &self.nodes
    }

    // Get reference to specific node
    // AABBTreeIndirect.hpp:105
    pub fn node(&self, idx: usize) -> &Node {
        // AABBTreeIndirect.hpp:105
        &self.nodes[idx]
    }

    // Check if tree is empty
    // AABBTreeIndirect.hpp:106
    pub fn empty(&self) -> bool {
        // AABBTreeIndirect.hpp:106
        self.nodes.is_empty()
    }

    // Calculate left child index using power of two rule
    // AABBTreeIndirect.hpp:109
    pub fn left_child_idx(idx: usize) -> usize {
        // AABBTreeIndirect.hpp:109
        idx * 2 + 1
    }

    // Calculate right child index using power of two rule
    // AABBTreeIndirect.hpp:110
    pub fn right_child_idx(idx: usize) -> usize {
        // AABBTreeIndirect.hpp:110
        Self::left_child_idx(idx) + 1
    }

    // Get reference to left child node
    // AABBTreeIndirect.hpp:111
    pub fn left_child(&self, idx: usize) -> &Node {
        // AABBTreeIndirect.hpp:111
        &self.nodes[Self::left_child_idx(idx)]
    }

    // Get reference to right child node
    // AABBTreeIndirect.hpp:112
    pub fn right_child(&self, idx: usize) -> &Node {
        // AABBTreeIndirect.hpp:112
        &self.nodes[Self::right_child_idx(idx)]
    }

    // Build a balanced tree by splitting the input sequence by an axis aligned plane
    // AABBTreeIndirect.hpp:124-150
    fn build_recursive<S: SourceNode>(
        &mut self,
        input: &mut [S],
        node_idx: usize,
        left: usize,
        right: usize,
    ) {
        // AABBTreeIndirect.hpp:125-126
        assert!(node_idx < self.nodes.len());
        assert!(left <= right);

        // AABBTreeIndirect.hpp:128-132
        if left == right {
            // Insert a node into the balanced tree
            // AABBTreeIndirect.hpp:129-131
            self.nodes[node_idx].idx = input[left].idx();
            self.nodes[node_idx].bbox = input[left].bbox();
            return;
        }

        // Calculate bounding box of the input
        // AABBTreeIndirect.hpp:135-137
        let mut bbox = input[left].bbox();
        // AABBTreeIndirect.hpp:136
        for i in (left + 1)..=right {
            bbox.extend(&input[i].bbox());
        }

        // Find dimension with maximum extent
        // AABBTreeIndirect.hpp:138-139
        let size = bbox.size();
        let dimension = if size.x >= size.y && size.x >= size.z {
            0
        } else if size.y >= size.z {
            1
        } else {
            2
        };

        // Partition the input to left/right pieces to produce balanced tree
        // AABBTreeIndirect.hpp:142-143
        let center = (left + right) / 2;
        self.partition_input(input, dimension, left, right, center);

        // Insert an inner node into the tree
        // AABBTreeIndirect.hpp:145-147
        self.nodes[node_idx].idx = constants::INNER;
        self.nodes[node_idx].bbox = bbox;

        // AABBTreeIndirect.hpp:148-149
        self.build_recursive(input, node_idx * 2 + 1, left, center);
        self.build_recursive(input, node_idx * 2 + 2, center + 1, right);
    }

    // Partition using QuickSelect to find k-th element
    // AABBTreeIndirect.hpp:157-210
    fn partition_input<S: SourceNode>(
        &self,
        input: &mut [S],
        dimension: usize,
        mut left: usize,
        mut right: usize,
        k: usize,
    ) {
        // AABBTreeIndirect.hpp:159
        while left < right {
            // Calculate center for pivot selection
            // AABBTreeIndirect.hpp:160
            let center = (left + right) / 2;

            // Median-of-three pivot selection with bubble sort
            // AABBTreeIndirect.hpp:161-179
            let pivot = {
                // AABBTreeIndirect.hpp:165-167
                let left_value = self.get_centroid_coord(&input[left], dimension);
                let center_value = self.get_centroid_coord(&input[center], dimension);
                let right_value = self.get_centroid_coord(&input[right], dimension);

                // AABBTreeIndirect.hpp:168-170
                let (left_value, mut center_value, mut right_value) =
                    if left_value > center_value {
                        input.swap(left, center);
                        (center_value, left_value, right_value)
                    } else {
                        (left_value, center_value, right_value)
                    };

                // AABBTreeIndirect.hpp:171-174
                if left_value > right_value {
                    input.swap(left, right);
                    right_value = left_value;
                }

                // AABBTreeIndirect.hpp:175-178
                if center_value > right_value {
                    input.swap(center, right);
                    center_value = right_value;
                }

                // AABBTreeIndirect.hpp:179
                center_value
            };

            // AABBTreeIndirect.hpp:181-183
            if right <= left + 2 {
                // The <left, right> interval is already sorted
                // AABBTreeIndirect.hpp:182
                break;
            }

            // Partition the set based on the pivot
            // AABBTreeIndirect.hpp:184-186
            let mut i = left;
            let mut j = right - 1;

            // AABBTreeIndirect.hpp:187
            input.swap(center, j);

            // AABBTreeIndirect.hpp:189
            loop {
                // Skip left points that are already at correct positions.
                // Search will certainly stop at position (right - 1), which stores the pivot.
                // C++: while (input[++ i].centroid()(dimension) < pivot) ;
                // AABBTreeIndirect.hpp:191-192
                loop {
                    i += 1;
                    if !(self.get_centroid_coord(&input[i], dimension) < pivot) {
                        break;
                    }
                }

                // Skip right points that are already at correct positions.
                // C++: while (input[-- j].centroid()(dimension) > pivot && i < j) ;
                // The pre-decrement happens first, then the > pivot test, then i < j.
                // AABBTreeIndirect.hpp:194
                loop {
                    j -= 1;
                    if !(self.get_centroid_coord(&input[j], dimension) > pivot && i < j) {
                        break;
                    }
                }

                // AABBTreeIndirect.hpp:195-196
                if i >= j {
                    break;
                }

                // AABBTreeIndirect.hpp:197
                input.swap(i, j);
            }

            // Restore pivot to the center of the sequence
            // AABBTreeIndirect.hpp:201
            input.swap(i, right - 1);

            // Which side the kth element is in?
            // AABBTreeIndirect.hpp:203-209
            if k < i {
                // AABBTreeIndirect.hpp:204
                right = i - 1;
            } else if k == i {
                // Sequence is partitioned, kth element is at its place
                // AABBTreeIndirect.hpp:206-207
                break;
            } else {
                // AABBTreeIndirect.hpp:208
                left = i + 1;
            }
        }
    }

    // Helper to get coordinate from centroid at given dimension
    // AABBTreeIndirect.hpp:165-167 (inline helper)
    fn get_centroid_coord<S: SourceNode>(&self, node: &S, dimension: usize) -> f64 {
        let centroid = node.centroid();
        match dimension {
            0 => centroid.x,
            1 => centroid.y,
            2 => centroid.z,
            _ => panic!("Invalid dimension"),
        }
    }
}

impl Default for Tree {
    fn default() -> Self {
        Self::new()
    }
}

/// Calculate next highest power of 2
/// AABBTreeIndirect.hpp:16 (Utils.hpp reference)
fn next_highest_power_of_2(mut v: usize) -> usize {
    if v == 0 {
        return 1;
    }
    v -= 1;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v |= v >> 32;
    v + 1
}

// ---------------------------------------------------------------------------
// Type Aliases
// ---------------------------------------------------------------------------

/// Type alias for 3D float AABB tree
/// AABBTreeIndirect.hpp:217
/// C++: using Tree3f = Tree<3, float>;
pub type Tree3F = Tree;

/// Type alias for 3D double AABB tree
/// AABBTreeIndirect.hpp:219
/// C++: using Tree3d = Tree<3, double>;
pub type Tree3D = Tree;

// ---------------------------------------------------------------------------
// Ray-Box Intersection
// ---------------------------------------------------------------------------

/// Ray-box intersection test using inverted direction
/// AABBTreeIndirect.hpp:269-306
pub fn ray_box_intersect_invdir(
    origin: &Point3F,
    inv_dir: &Point3F,
    mut bbox: BoundingBox3F,
    t0: f64,
    t1: f64,
) -> bool {
    // Swap box coordinates based on inverted direction signs
    // AABBTreeIndirect.hpp:278-282
    if inv_dir.x < 0.0 {
        std::mem::swap(&mut bbox.min.x, &mut bbox.max.x);
    }
    if inv_dir.y < 0.0 {
        std::mem::swap(&mut bbox.min.y, &mut bbox.max.y);
    }

    // Calculate X interval
    // AABBTreeIndirect.hpp:281-286
    let mut tmin = (bbox.min.x - origin.x) * inv_dir.x;
    let tymax = (bbox.max.y - origin.y) * inv_dir.y;
    if tmin > tymax {
        return false;
    }

    // AABBTreeIndirect.hpp:285-289
    let mut tmax = (bbox.max.x - origin.x) * inv_dir.x;
    let tymin = (bbox.min.y - origin.y) * inv_dir.y;
    if tymin > tmax {
        return false;
    }

    // AABBTreeIndirect.hpp:290-293
    if tymin > tmin {
        tmin = tymin;
    }
    if tymax < tmax {
        tmax = tymax;
    }

    // Handle Z dimension
    // AABBTreeIndirect.hpp:294-305
    if inv_dir.z < 0.0 {
        std::mem::swap(&mut bbox.min.z, &mut bbox.max.z);
    }

    // AABBTreeIndirect.hpp:295-298
    let tzmin = (bbox.min.z - origin.z) * inv_dir.z;
    if tzmin > tmax {
        return false;
    }

    // AABBTreeIndirect.hpp:298-301
    let tzmax = (bbox.max.z - origin.z) * inv_dir.z;
    if tmin > tzmax {
        return false;
    }

    // AABBTreeIndirect.hpp:302-305
    if tzmin > tmin {
        tmin = tzmin;
    }
    if tzmax < tmax {
        tmax = tzmax;
    }

    // AABBTreeIndirect.hpp:306
    tmin < t1 && tmax > t0
}

/// Ray-triangle intersection using Möller-Trumbore algorithm
/// AABBTreeIndirect.hpp:315-363
pub fn intersect_triangle(
    origin: &Point3F,
    dir: &Point3F,
    v0: &Point3F,
    v1: &Point3F,
    v2: &Point3F,
    eps: f64,
) -> Option<(f64, f64, f64)> {
    // Find vectors for two edges sharing v0
    // AABBTreeIndirect.hpp:319-320
    let edge1 = Point3F::new(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
    let edge2 = Point3F::new(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);

    // Begin calculating determinant - also used to calculate U parameter
    // AABBTreeIndirect.hpp:322
    let pvec = cross_product(dir, &edge2);

    // If determinant is near zero, ray lies in plane of triangle
    // AABBTreeIndirect.hpp:324
    let det = dot_product(&edge1, &pvec);

    // AABBTreeIndirect.hpp:327-359
    let (u, v, qvec) = if det > eps {
        // Calculate distance from v0 to ray origin
        // AABBTreeIndirect.hpp:329
        let tvec = Point3F::new(origin.x - v0.x, origin.y - v0.y, origin.z - v0.z);

        // Calculate U parameter and test bounds
        // AABBTreeIndirect.hpp:331-333
        let u = dot_product(&tvec, &pvec);
        if u < 0.0 || u > det {
            return None;
        }

        // Prepare to test V parameter
        // AABBTreeIndirect.hpp:335
        let qvec = cross_product(&tvec, &edge1);

        // Calculate V parameter and test bounds
        // AABBTreeIndirect.hpp:337-339
        let v = dot_product(dir, &qvec);
        if v < 0.0 || u + v > det {
            return None;
        }

        (u, v, qvec)
    } else if det < -eps {
        // Calculate distance from v0 to ray origin
        // AABBTreeIndirect.hpp:342
        let tvec = Point3F::new(origin.x - v0.x, origin.y - v0.y, origin.z - v0.z);

        // Calculate U parameter and test bounds
        // AABBTreeIndirect.hpp:344-346
        let u = dot_product(&tvec, &pvec);
        if u > 0.0 || u < det {
            return None;
        }

        // Prepare to test V parameter
        // AABBTreeIndirect.hpp:348
        let qvec = cross_product(&tvec, &edge1);

        // Calculate V parameter and test bounds
        // AABBTreeIndirect.hpp:350-352
        let v = dot_product(dir, &qvec);
        if v > 0.0 || u + v < det {
            return None;
        }

        (u, v, qvec)
    } else {
        // Ray is parallel to the plane of the triangle
        // AABBTreeIndirect.hpp:354-355
        return None;
    };

    // Calculate t, ray intersects triangle
    // AABBTreeIndirect.hpp:357-360
    let inv_det = 1.0 / det;
    let t = dot_product(&edge2, &qvec) * inv_det;
    let u = u * inv_det;
    let v = v * inv_det;

    // AABBTreeIndirect.hpp:361
    Some((t, u, v))
}

/// Closest point on triangle to given point
/// AABBTreeIndirect.hpp:475-521
pub fn closest_point_to_triangle(p: &Point3F, a: &Point3F, b: &Point3F, c: &Point3F) -> Point3F {
    // Check if P in vertex region outside A
    // AABBTreeIndirect.hpp:479-485
    let ab = Point3F::new(b.x - a.x, b.y - a.y, b.z - a.z);
    let ac = Point3F::new(c.x - a.x, c.y - a.y, c.z - a.z);
    let ap = Point3F::new(p.x - a.x, p.y - a.y, p.z - a.z);
    let d1 = dot_product(&ab, &ap);
    let d2 = dot_product(&ac, &ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return *a;
    }

    // Check if P in vertex region outside B
    // AABBTreeIndirect.hpp:487-491
    let bp = Point3F::new(p.x - b.x, p.y - b.y, p.z - b.z);
    let d3 = dot_product(&ab, &bp);
    let d4 = dot_product(&ac, &bp);
    if d3 >= 0.0 && d4 <= d3 {
        return *b;
    }

    // Check if P in edge region of AB
    // AABBTreeIndirect.hpp:493-497
    let vc = d1 * d4 - d3 * d2;
    if a != b && vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return Point3F::new(a.x + ab.x * v, a.y + ab.y * v, a.z + ab.z * v);
    }

    // Check if P in vertex region outside C
    // AABBTreeIndirect.hpp:499-503
    let cp = Point3F::new(p.x - c.x, p.y - c.y, p.z - c.z);
    let d5 = dot_product(&ab, &cp);
    let d6 = dot_product(&ac, &cp);
    if d6 >= 0.0 && d5 <= d6 {
        return *c;
    }

    // Check if P in edge region of AC
    // AABBTreeIndirect.hpp:505-509
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return Point3F::new(a.x + ac.x * w, a.y + ac.y * w, a.z + ac.z * w);
    }

    // Check if P in edge region of BC
    // AABBTreeIndirect.hpp:511-515
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        let bc = Point3F::new(c.x - b.x, c.y - b.y, c.z - b.z);
        return Point3F::new(b.x + bc.x * w, b.y + bc.y * w, b.z + bc.z * w);
    }

    // P inside face region - compute via barycentric coordinates
    // AABBTreeIndirect.hpp:517-520
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;

    // AABBTreeIndirect.hpp:520
    Point3F::new(
        a.x + ab.x * v + ac.x * w,
        a.y + ab.y * v + ac.y * w,
        a.z + ab.z * v + ac.z * w,
    )
}

/// Traverse tree with a predicate and callback
/// AABBTreeIndirect.hpp:981-987
pub fn traverse<P, F>(tree: &Tree, mut predicate: P, mut callback: F)
where
    P: FnMut(&Node) -> bool,
    F: FnMut(&Node) -> bool,
{
    // AABBTreeIndirect.hpp:983-984
    if tree.empty() {
        return;
    }

    // AABBTreeIndirect.hpp:986
    traverse_recurse(tree, 0, &mut predicate, &mut callback);
}

/// Recursive traversal helper
/// AABBTreeIndirect.hpp:944-971
fn traverse_recurse<P, F>(tree: &Tree, idx: usize, predicate: &mut P, callback: &mut F) -> bool
where
    P: FnMut(&Node) -> bool,
    F: FnMut(&Node) -> bool,
{
    // AABBTreeIndirect.hpp:945
    assert!(tree.node(idx).is_valid());

    // AABBTreeIndirect.hpp:947-949
    if !predicate(tree.node(idx)) {
        // Continue traversal
        // AABBTreeIndirect.hpp:948
        return true;
    }

    // AABBTreeIndirect.hpp:951-971
    if tree.node(idx).is_leaf() {
        // Callback returns true to continue, false to stop
        // AABBTreeIndirect.hpp:952-953
        callback(tree.node(idx))
    } else {
        // Traverse both children
        // AABBTreeIndirect.hpp:955-970
        let left_idx = Tree::left_child_idx(idx);
        let right_idx = Tree::right_child_idx(idx);

        // AABBTreeIndirect.hpp:968-969
        traverse_recurse(tree, left_idx, predicate, callback)
            && traverse_recurse(tree, right_idx, predicate, callback)
    }
}

/// Cross product of two 3D vectors
/// AABBTreeIndirect.hpp:322 (inline)
fn cross_product(a: &Point3F, b: &Point3F) -> Point3F {
    // `Vec3d::cross` as the C++ build evaluates it, each component
    // `fma(a, b, -(c * d))` (the order found for `Vec3f`, R876).
    Point3F::new(
        a.y.mul_add(b.z, -(a.z * b.y)),
        a.z.mul_add(b.x, -(a.x * b.z)),
        a.x.mul_add(b.y, -(a.y * b.x)),
    )
}

/// Dot product of two 3D vectors
/// AABBTreeIndirect.hpp:324 (inline)
fn dot_product(a: &Point3F, b: &Point3F) -> f64 {
    // `Vec3d::dot` as `fma(a.z, b.z, a.x * b.x + a.y * b.y)` (the order found
    // for `Vec3f`, R880).
    a.z.mul_add(b.z, a.x * b.x + a.y * b.y)
}

// ---------------------------------------------------------------------------
// Indexed Triangle Set Functions
// ---------------------------------------------------------------------------

/// Build AABB tree over indexed triangle set
/// AABBTreeIndirect.hpp:672-720
/// C++: inline Tree<3, typename VertexType::Scalar> build_aabb_tree_over_indexed_triangle_set(
/// C++:     const std::vector<VertexType> &vertices,
/// C++:     const std::vector<IndexedFaceType> &faces,
/// C++:     const typename VertexType::Scalar eps = 0)
///
/// The C++ trailing `eps` argument inflates every leaf bbox by `(eps,eps,eps)`
/// (AABBTreeIndirect.hpp:697,709-710). Both crate callers use the default `eps = 0`,
/// for which the inflation is a no-op, so this port reproduces the `eps = 0` behaviour.
pub fn build_aabb_tree_over_indexed_triangle_set(
    vertices: &[Point3F],
    faces: &[[usize; 3]],
) -> Tree {
    // Create input nodes for each triangle
    // AABBTreeIndirect.hpp:695-713
    // C++: std::vector<InputType> input;
    // C++: input.reserve(faces.size());
    // C++: for (size_t i = 0; i < faces.size(); ++i) {
    // C++:     const IndexedFaceType &face = faces[i];
    // C++:     const VertexType &v1 = vertices[face(0)];
    // C++:     const VertexType &v2 = vertices[face(1)];
    // C++:     const VertexType &v3 = vertices[face(2)];
    // C++:     InputType n;
    // C++:     n.m_idx = i;
    // C++:     n.m_centroid = (1./3.) * (v1 + v2 + v3);
    // C++:     n.m_bbox = BoundingBox(v1, v1);
    // C++:     n.m_bbox.extend(v2);
    // C++:     n.m_bbox.extend(v3);
    // C++:     input.emplace_back(n);
    // C++: }
    struct TriangleNode {
        idx: usize,
        bbox: BoundingBox3F,
        centroid: Point3F,
    }

    impl SourceNode for TriangleNode {
        fn idx(&self) -> usize {
            self.idx
        }
        fn centroid(&self) -> Point3F {
            self.centroid
        }
        fn bbox(&self) -> BoundingBox3F {
            self.bbox.clone()
        }
    }

    let mut input: Vec<TriangleNode> = Vec::with_capacity(faces.len());
    for (i, face) in faces.iter().enumerate() {
        let v1 = &vertices[face[0]];
        let v2 = &vertices[face[1]];
        let v3 = &vertices[face[2]];

        // AABBTreeIndirect.hpp:707, `n.m_centroid = (1./3.) * (v1 + v2 + v3)`. The
        // seam placer's tree is a `Tree3f`: the vertices and the centroid are
        // floats, the sum is a float sum and the third is `float(1./3.)`. This
        // crate's tree holds doubles, so a triangle whose coordinates are all
        // floats gets the float centroid, and any other one the double product.
        let is_f32 = |p: &Point3F| {
            (p.x as f32) as f64 == p.x && (p.y as f32) as f64 == p.y && (p.z as f32) as f64 == p.z
        };
        let centroid = if is_f32(v1) && is_f32(v2) && is_f32(v3) {
            let third = (1.0f64 / 3.0) as f32;
            let c = |a: f64, b: f64, c: f64| (third * ((a as f32 + b as f32) + c as f32)) as f64;
            Point3F { x: c(v1.x, v2.x, v3.x), y: c(v1.y, v2.y, v3.y), z: c(v1.z, v2.z, v3.z) }
        } else {
            let third = 1.0f64 / 3.0;
            Point3F {
                x: third * ((v1.x + v2.x) + v3.x),
                y: third * ((v1.y + v2.y) + v3.y),
                z: third * ((v1.z + v2.z) + v3.z),
            }
        };

        let mut bbox = BoundingBox3F::new();
        bbox.extend_point(v1);
        bbox.extend_point(v2);
        bbox.extend_point(v3);

        input.push(TriangleNode {
            idx: i,
            bbox,
            centroid,
        });
    }

    // Build and return tree
    // AABBTreeIndirect.hpp:716-718
    // C++: TreeType out;
    // C++: out.build(std::move(input));
    // C++: return out;
    let mut tree = Tree::new();
    tree.build(input);
    tree
}

/// Find first intersection of ray with indexed triangle set
/// AABBTreeIndirect.hpp:723-753
/// C++: inline bool intersect_ray_first_hit(
/// C++:     const std::vector<VertexType> &vertices,
/// C++:     const std::vector<IndexedFaceType> &faces,
/// C++:     const Tree<3, typename VertexType::Scalar> &tree,
/// C++:     const VectorType &origin,
/// C++:     const VectorType &dir,
/// C++:     igl::Hit &hit)
pub fn intersect_ray_first_hit(
    vertices: &[Point3F],
    faces: &[[usize; 3]],
    tree: &Tree,
    origin: &Point3F,
    dir: &Point3F,
) -> Option<(f64, usize, Point3F)> {
    // AABBTreeIndirect.hpp:737: `const double eps = 0.000001` (default argument).
    intersect_ray_first_hit_eps(vertices, faces, tree, origin, dir, 0.000001)
}

/// `intersect_ray_first_hit` with an explicit ray-triangle intersection epsilon.
/// AABBTreeIndirect.hpp:723-753 — the C++ function takes the epsilon as a trailing
/// defaulted parameter ("it should be proportional to an average triangle edge
/// length"); SLA/IndexedMesh.cpp:41-42 passes its `m_triangle_ray_epsilon` here.
pub fn intersect_ray_first_hit_eps(
    vertices: &[Point3F],
    faces: &[[usize; 3]],
    tree: &Tree,
    origin: &Point3F,
    dir: &Point3F,
    eps: f64,
) -> Option<(f64, usize, Point3F)> {
    // AABBTreeIndirect.hpp:740-746
    // C++: auto ray_intersector = detail::RayIntersector<...> {
    // C++:     vertices, faces, tree, origin, dir, VectorType(dir.cwiseInverse()), eps };
    // C++: return ! tree.empty() && detail::intersect_ray_recursive_first_hit(
    // C++:     ray_intersector, size_t(0), std::numeric_limits<Scalar>::infinity(), hit);
    if tree.empty() {
        return None;
    }
    let invdir = Point3F::new(1.0 / dir.x, 1.0 / dir.y, 1.0 / dir.z);
    let mut hit: Option<RayHit> = None;
    intersect_ray_recursive_first_hit(
        vertices, faces, tree, origin, dir, &invdir, eps, 0, f64::INFINITY, &mut hit,
    );
    hit.map(|h| {
        let hit_point = Point3F {
            x: origin.x + dir.x * h.t,
            y: origin.y + dir.y * h.t,
            z: origin.z + dir.z * h.t,
        };
        (h.t, h.face_idx, hit_point)
    })
}

/// Hit record mirroring igl::Hit (id + barycentric u/v + ray parameter t).
/// AABBTreeIndirect.hpp:417 `igl::Hit { int(node.idx), -1, float(u), float(v), float(t) }`
#[derive(Debug, Clone, Copy)]
struct RayHit {
    face_idx: usize,
    // Barycentric coordinates stored to mirror igl::Hit; the public wrappers
    // currently expose only `t`/`face_idx`, so these are kept for fidelity.
    #[allow(dead_code)]
    u: f64,
    #[allow(dead_code)]
    v: f64,
    t: f64,
}

/// Recursive first-hit ray traversal with AABB pruning.
/// AABBTreeIndirect.hpp:396-440 `intersect_ray_recursive_first_hit`
#[allow(clippy::too_many_arguments)]
fn intersect_ray_recursive_first_hit(
    vertices: &[Point3F],
    faces: &[[usize; 3]],
    tree: &Tree,
    origin: &Point3F,
    dir: &Point3F,
    invdir: &Point3F,
    eps: f64,
    node_idx: usize,
    mut min_t: f64,
    hit: &mut Option<RayHit>,
) -> bool {
    // AABBTreeIndirect.hpp:402-403
    let node = tree.node(node_idx);
    debug_assert!(node.is_valid());

    // AABBTreeIndirect.hpp:405-406
    if !ray_box_intersect_invdir(origin, invdir, node.bbox.clone(), 0.0, min_t) {
        return false;
    }

    // AABBTreeIndirect.hpp:408-420
    if node.is_leaf() {
        // shoot ray, record hit
        let face = &faces[node.idx];
        if let Some((t, u, v)) = intersect_triangle(
            origin,
            dir,
            &vertices[face[0]],
            &vertices[face[1]],
            &vertices[face[2]],
            eps,
        ) {
            if t > 0.0 {
                // AABBTreeIndirect.hpp:417
                // AABBTreeIndirect.hpp:417,
                // `hit = igl::Hit { int(node.idx), -1, float(u), float(v), float(t) }`:
                // the hit's members are floats, and it is the float `t` that the
                // callers above compare with `min_t` and copy into it.
                let (t, u, v) = ((t as f32) as f64, (u as f32) as f64, (v as f32) as f64);
                *hit = Some(RayHit {
                    face_idx: node.idx,
                    u,
                    v,
                    t,
                });
                return true;
            }
        }
        false
    } else {
        // Left / right child node index.
        // AABBTreeIndirect.hpp:423-424
        let left = node_idx * 2 + 1;
        let right = left + 1;

        // AABBTreeIndirect.hpp:425-432
        let mut left_hit: Option<RayHit> = None;
        let mut left_ret = intersect_ray_recursive_first_hit(
            vertices, faces, tree, origin, dir, invdir, eps, left, min_t, &mut left_hit,
        );
        if left_ret && left_hit.map(|h| h.t).unwrap_or(f64::INFINITY) < min_t {
            min_t = left_hit.unwrap().t;
            *hit = left_hit;
        } else {
            left_ret = false;
        }

        // AABBTreeIndirect.hpp:433-437
        let mut right_hit: Option<RayHit> = None;
        let mut right_ret = intersect_ray_recursive_first_hit(
            vertices, faces, tree, origin, dir, invdir, eps, right, min_t, &mut right_hit,
        );
        if right_ret && right_hit.map(|h| h.t).unwrap_or(f64::INFINITY) < min_t {
            *hit = right_hit;
        } else {
            right_ret = false;
        }

        // AABBTreeIndirect.hpp:438
        left_ret || right_ret
    }
}

/// Find all intersections of ray with indexed triangle set
/// AABBTreeIndirect.hpp:755-794
/// C++: inline bool intersect_ray_all_hits(
/// C++:     const std::vector<VertexType> &vertices,
/// C++:     const std::vector<IndexedFaceType> &faces,
/// C++:     const Tree<3, typename VertexType::Scalar> &tree,
/// C++:     const VectorType &origin,
/// C++:     const VectorType &dir,
/// C++:     std::vector<igl::Hit> &hits)
pub fn intersect_ray_all_hits(
    vertices: &[Point3F],
    faces: &[[usize; 3]],
    tree: &Tree,
    origin: &Point3F,
    dir: &Point3F,
) -> Vec<(f64, usize, Point3F)> {
    // AABBTreeIndirect.hpp:770: `const double eps = 0.000001` (default argument).
    intersect_ray_all_hits_eps(vertices, faces, tree, origin, dir, 0.000001)
}

/// `intersect_ray_all_hits` with an explicit ray-triangle intersection epsilon.
/// AABBTreeIndirect.hpp:755-794 — the C++ function takes the epsilon as a trailing
/// defaulted parameter; SLA/IndexedMesh.cpp:50-51 passes its
/// `m_triangle_ray_epsilon` here.
pub fn intersect_ray_all_hits_eps(
    vertices: &[Point3F],
    faces: &[[usize; 3]],
    tree: &Tree,
    origin: &Point3F,
    dir: &Point3F,
    eps: f64,
) -> Vec<(f64, usize, Point3F)> {
    // AABBTreeIndirect.hpp:771-787
    // C++: ray_intersector.hits.clear(); detail::intersect_ray_recursive_all_hits(ray_intersector, 0);
    // C++: std::sort(hits.begin(), hits.end(), [](const auto &l, const auto &r) { return l.t < r.t; });
    let mut hits: Vec<RayHit> = Vec::new();
    if !tree.empty() {
        let invdir = Point3F::new(1.0 / dir.x, 1.0 / dir.y, 1.0 / dir.z);
        intersect_ray_recursive_all_hits(
            vertices, faces, tree, origin, dir, &invdir, eps, 0, &mut hits,
        );
    }
    // AABBTreeIndirect.hpp:785 — sort the output hits by the ray parameter t.
    hits.sort_by(|l, r| l.t.partial_cmp(&r.t).unwrap_or(std::cmp::Ordering::Equal));

    hits.into_iter()
        .map(|h| {
            let hit_point = Point3F {
                x: origin.x + dir.x * h.t,
                y: origin.y + dir.y * h.t,
                z: origin.z + dir.z * h.t,
            };
            (h.t, h.face_idx, hit_point)
        })
        .collect()
}

/// Recursive all-hits ray traversal with AABB pruning.
/// AABBTreeIndirect.hpp:443-471 `intersect_ray_recursive_all_hits`
#[allow(clippy::too_many_arguments)]
fn intersect_ray_recursive_all_hits(
    vertices: &[Point3F],
    faces: &[[usize; 3]],
    tree: &Tree,
    origin: &Point3F,
    dir: &Point3F,
    invdir: &Point3F,
    eps: f64,
    node_idx: usize,
    hits: &mut Vec<RayHit>,
) {
    // AABBTreeIndirect.hpp:447-448
    let node = tree.node(node_idx);
    debug_assert!(node.is_valid());

    // AABBTreeIndirect.hpp:450-452 — t0 = 0, t1 = +inf
    if !ray_box_intersect_invdir(origin, invdir, node.bbox.clone(), 0.0, f64::INFINITY) {
        return;
    }

    // AABBTreeIndirect.hpp:454-463
    if node.is_leaf() {
        let face = &faces[node.idx];
        if let Some((t, u, v)) = intersect_triangle(
            origin,
            dir,
            &vertices[face[0]],
            &vertices[face[1]],
            &vertices[face[2]],
            eps,
        ) {
            if t > 0.0 {
                // AABBTreeIndirect.hpp:462
                hits.push(RayHit {
                    face_idx: node.idx,
                    u,
                    v,
                    t,
                });
            }
        }
    } else {
        // Left / right child node index.
        // AABBTreeIndirect.hpp:466-469
        let left = node_idx * 2 + 1;
        let right = left + 1;
        intersect_ray_recursive_all_hits(
            vertices, faces, tree, origin, dir, invdir, eps, left, hits,
        );
        intersect_ray_recursive_all_hits(
            vertices, faces, tree, origin, dir, invdir, eps, right, hits,
        );
    }
}

/// Find squared distance from point to indexed triangle set
/// AABBTreeIndirect.hpp:796-858
/// C++: inline typename VectorType::Scalar squared_distance_to_indexed_triangle_set(
/// C++:     const std::vector<VertexType> &vertices,
/// C++:     const std::vector<IndexedFaceType> &faces,
/// C++:     const Tree<3, typename VertexType::Scalar> &tree,
/// C++:     const VectorType &point,
/// C++:     size_t &i,
/// C++:     VectorType &closest)
pub fn squared_distance_to_indexed_triangle_set(
    vertices: &[Point3F],
    faces: &[[usize; 3]],
    tree: &Tree,
    point: Vec3,
) -> (f64, usize, Vec3) {
    // AABBTreeIndirect.hpp:811-814
    // C++: auto distancer = detail::IndexedTriangleSetDistancer<...> { vertices, faces, tree, point };
    // C++: return tree.empty() ? Scalar(-1) :
    // C++:     detail::squared_distance_to_indexed_primitives_recursive(distancer, 0, 0, +inf, hit_idx_out, hit_point_out);
    let mut hit_idx: usize = 0;
    let mut hit_point = point;
    if tree.empty() {
        // C++ returns -1 for an empty tree.
        return (-1.0, hit_idx, hit_point);
    }
    let origin = Point3F::new(point.x, point.y, point.z);
    let dist = squared_distance_to_indexed_primitives_recursive(
        vertices,
        faces,
        tree,
        &origin,
        0,
        0.0,
        f64::INFINITY,
        &mut hit_idx,
        &mut hit_point,
    );
    (dist, hit_idx, hit_point)
}

/// Closest point on the indexed triangle `primitive_index` to `origin`, and its squared distance.
/// AABBTreeIndirect.hpp:539-548 `IndexedTriangleSetDistancer::closest_point_to_origin`
fn closest_point_to_origin(
    vertices: &[Point3F],
    faces: &[[usize; 3]],
    origin: &Point3F,
    primitive_index: usize,
) -> (Vec3, f64) {
    // AABBTreeIndirect.hpp:541-547
    let triangle = &faces[primitive_index];
    let closest_point = closest_point_to_triangle(
        origin,
        &vertices[triangle[0]],
        &vertices[triangle[1]],
        &vertices[triangle[2]],
    );
    let dx = origin.x - closest_point.x;
    let dy = origin.y - closest_point.y;
    let dz = origin.z - closest_point.z;
    let squared_distance = dx * dx + dy * dy + dz * dz;
    (
        Vec3::new(closest_point.x, closest_point.y, closest_point.z),
        squared_distance,
    )
}

/// Squared exterior distance from `p` to a 3D bounding box (0 when inside).
/// Eigen `AlignedBox::squaredExteriorDistance`.
fn bbox_squared_exterior_distance(bbox: &BoundingBox3F, p: &Point3F) -> f64 {
    let dx = if p.x < bbox.min.x {
        bbox.min.x - p.x
    } else if p.x > bbox.max.x {
        p.x - bbox.max.x
    } else {
        0.0
    };
    let dy = if p.y < bbox.min.y {
        bbox.min.y - p.y
    } else if p.y > bbox.max.y {
        p.y - bbox.max.y
    } else {
        0.0
    };
    let dz = if p.z < bbox.min.z {
        bbox.min.z - p.z
    } else if p.z > bbox.max.z {
        p.z - bbox.max.z
    } else {
        0.0
    };
    dx * dx + dy * dy + dz * dz
}

/// Recursive closest-primitive search with AABB pruning.
/// AABBTreeIndirect.hpp:552-632 `squared_distance_to_indexed_primitives_recursive`
#[allow(clippy::too_many_arguments)]
fn squared_distance_to_indexed_primitives_recursive(
    vertices: &[Point3F],
    faces: &[[usize; 3]],
    tree: &Tree,
    origin: &Point3F,
    node_idx: usize,
    low_sqr_d: f64,
    mut up_sqr_d: f64,
    i: &mut usize,
    c: &mut Vec3,
) -> f64 {
    // AABBTreeIndirect.hpp:562-563
    if low_sqr_d > up_sqr_d {
        return low_sqr_d;
    }

    // AABBTreeIndirect.hpp:574-575
    let node = tree.node(node_idx);
    debug_assert!(node.is_valid());

    // AABBTreeIndirect.hpp:576-580
    if node.is_leaf() {
        let (c_candidate, sqr_dist) = closest_point_to_origin(vertices, faces, origin, node.idx);
        // set_min — AABBTreeIndirect.hpp:566-572
        if sqr_dist < up_sqr_d {
            *i = node.idx;
            *c = c_candidate;
            up_sqr_d = sqr_dist;
        }
    } else {
        // AABBTreeIndirect.hpp:584-589
        let left_node_idx = node_idx * 2 + 1;
        let right_node_idx = left_node_idx + 1;
        let bbox_left = tree.node(left_node_idx).bbox.clone();
        let bbox_right = tree.node(right_node_idx).bbox.clone();

        let mut looked_left = false;
        let mut looked_right = false;

        // look_left — AABBTreeIndirect.hpp:593-600
        macro_rules! look_left {
            () => {{
                let mut i_left: usize = 0;
                let mut c_left: Vec3 = *c;
                let sqr_d_left = squared_distance_to_indexed_primitives_recursive(
                    vertices, faces, tree, origin, left_node_idx, low_sqr_d, up_sqr_d,
                    &mut i_left, &mut c_left,
                );
                if sqr_d_left < up_sqr_d {
                    *i = i_left;
                    *c = c_left;
                    up_sqr_d = sqr_d_left;
                }
                looked_left = true;
            }};
        }
        // look_right — AABBTreeIndirect.hpp:601-608
        macro_rules! look_right {
            () => {{
                let mut i_right: usize = 0;
                let mut c_right: Vec3 = *c;
                let sqr_d_right = squared_distance_to_indexed_primitives_recursive(
                    vertices, faces, tree, origin, right_node_idx, low_sqr_d, up_sqr_d,
                    &mut i_right, &mut c_right,
                );
                if sqr_d_right < up_sqr_d {
                    *i = i_right;
                    *c = c_right;
                    up_sqr_d = sqr_d_right;
                }
                looked_right = true;
            }};
        }

        // must look left or right if in box — AABBTreeIndirect.hpp:610-615
        let origin_p3f = Point3F::new(origin.x, origin.y, origin.z);
        if bbox_left.contains_point(&origin_p3f) {
            look_left!();
        }
        if bbox_right.contains_point(&origin_p3f) {
            look_right!();
        }
        // if haven't looked left/right and could be less than current min, then look
        // AABBTreeIndirect.hpp:617-629
        let left_up_sqr_d = bbox_squared_exterior_distance(&bbox_left, &origin_p3f);
        let right_up_sqr_d = bbox_squared_exterior_distance(&bbox_right, &origin_p3f);
        if left_up_sqr_d < right_up_sqr_d {
            if !looked_left && left_up_sqr_d < up_sqr_d {
                look_left!();
            }
            if !looked_right && right_up_sqr_d < up_sqr_d {
                look_right!();
            }
        } else {
            if !looked_right && right_up_sqr_d < up_sqr_d {
                look_right!();
            }
            if !looked_left && left_up_sqr_d < up_sqr_d {
                look_left!();
            }
        }
    }
    // AABBTreeIndirect.hpp:631
    up_sqr_d
}

/// Decides if there exists some triangle within radius `sqrt(max_distance_squared)` of `point`.
/// AABBTreeIndirect.hpp:822-849 `is_any_triangle_in_radius`
pub fn is_any_triangle_in_radius(
    vertices: &[Point3F],
    faces: &[[usize; 3]],
    tree: &Tree,
    point: Vec3,
    max_distance_squared: f64,
) -> bool {
    // AABBTreeIndirect.hpp:838-839
    let mut hit_idx: usize = 0;
    // hit_point = NaN; allFinite() is false unless a hit overwrites it.
    let mut hit_point = Vec3::new(f64::NAN, f64::NAN, f64::NAN);

    // AABBTreeIndirect.hpp:841-844
    if tree.empty() {
        return false;
    }

    // AABBTreeIndirect.hpp:846 — up_sqr_d bound = max_distance_squared
    let origin = Point3F::new(point.x, point.y, point.z);
    squared_distance_to_indexed_primitives_recursive(
        vertices,
        faces,
        tree,
        &origin,
        0,
        0.0,
        max_distance_squared,
        &mut hit_idx,
        &mut hit_point,
    );

    // AABBTreeIndirect.hpp:848 — return hit_point.allFinite();
    hit_point.x.is_finite() && hit_point.y.is_finite() && hit_point.z.is_finite()
}

/// Returns all triangles within the given radius limit.
/// AABBTreeIndirect.hpp:853-876 `all_triangles_in_radius`
pub fn all_triangles_in_radius(
    vertices: &[Point3F],
    faces: &[[usize; 3]],
    tree: &Tree,
    point: Vec3,
    max_distance_squared: f64,
) -> Vec<usize> {
    // AABBTreeIndirect.hpp:868-871
    if tree.empty() {
        return Vec::new();
    }
    // AABBTreeIndirect.hpp:873-875
    let mut found_triangles: Vec<usize> = Vec::new();
    let origin = Point3F::new(point.x, point.y, point.z);
    indexed_primitives_within_distance_squared_recurisve(
        vertices,
        faces,
        tree,
        &origin,
        0,
        max_distance_squared,
        &mut found_triangles,
    );
    found_triangles
}

/// Recursively collects primitives whose closest point lies within the squared-distance limit.
/// AABBTreeIndirect.hpp:635-663 `indexed_primitives_within_distance_squared_recurisve`
fn indexed_primitives_within_distance_squared_recurisve(
    vertices: &[Point3F],
    faces: &[[usize; 3]],
    tree: &Tree,
    origin: &Point3F,
    node_idx: usize,
    squared_distance_limit: f64,
    found_primitives_indices: &mut Vec<usize>,
) {
    // AABBTreeIndirect.hpp:640-641
    let node = tree.node(node_idx);
    debug_assert!(node.is_valid());

    // AABBTreeIndirect.hpp:642-645
    if node.is_leaf() {
        let (_c, sqr_dist) = closest_point_to_origin(vertices, faces, origin, node.idx);
        if sqr_dist < squared_distance_limit {
            found_primitives_indices.push(node.idx);
        }
    } else {
        // AABBTreeIndirect.hpp:647-661
        let left_node_idx = node_idx * 2 + 1;
        let right_node_idx = left_node_idx + 1;
        let origin_p3f = Point3F::new(origin.x, origin.y, origin.z);
        let bbox_left = &tree.node(left_node_idx).bbox;
        let bbox_right = &tree.node(right_node_idx).bbox;

        if bbox_squared_exterior_distance(bbox_left, &origin_p3f) < squared_distance_limit {
            indexed_primitives_within_distance_squared_recurisve(
                vertices,
                faces,
                tree,
                origin,
                left_node_idx,
                squared_distance_limit,
                found_primitives_indices,
            );
        }
        if bbox_squared_exterior_distance(bbox_right, &origin_p3f) < squared_distance_limit {
            indexed_primitives_within_distance_squared_recurisve(
                vertices,
                faces,
                tree,
                origin,
                right_node_idx,
                squared_distance_limit,
                found_primitives_indices,
            );
        }
    }
}

/// Collect the leaf primitive indices whose AABB contains `v`.
/// AABBTreeIndirect.hpp:882 `get_candidate_idxs`.
pub fn get_candidate_idxs(tree: &Tree, v: Point3F, candidates: &mut Vec<usize>, node_idx: usize) {
    if tree.empty() || !tree.node(node_idx).bbox.contains_point(&v) {
        return;
    }
    let node = tree.node(node_idx);
    if !node.is_leaf() {
        if tree.left_child(node_idx).bbox.contains_point(&v) {
            get_candidate_idxs(tree, v, candidates, Tree::left_child_idx(node_idx));
        }
        if tree.right_child(node_idx).bbox.contains_point(&v) {
            get_candidate_idxs(tree, v, candidates, Tree::right_child_idx(node_idx));
        }
    } else {
        candidates.push(node.idx);
    }
}

// ---------------------------------------------------------------------------
// Tree<3, float> (`AABBTreeIndirect::Tree3f`) in float.
//
// `Tree` above stores its boxes and centroids in double. A tree built over float vertices
// compares, extends and measures in float in the C++ (`Tree<3, float>`, `Eigen::AlignedBox<float,
// 3>`), and the split dimension, the partition and the traversal order follow from those float
// values; this is that tree. Floating point follows the C++ build (clang, `-ffp-contract=on`,
// Eigen 5): `Vec3f::dot` is `fma(a.z, b.z, a.x * b.x + a.y * b.y)`, `squaredNorm` is
// `(x * x + y * y) + z * z` unfused, the box's exterior distance accumulates
// `dist2 += aux * aux` fused, and coefficient-wise vector sums do not fuse.
// ---------------------------------------------------------------------------

use crate::triangle_mesh::{Vec3f, Vec3i};

/// `Eigen::AlignedBox<float, 3>` (`Tree3f::BoundingBox`). The default box is empty: min at
/// `FLT_MAX`, max at `-FLT_MAX` (`AlignedBox()` calls `setEmpty()`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoundingBox3f {
    pub min: Vec3f,
    pub max: Vec3f,
}

impl Default for BoundingBox3f {
    fn default() -> Self {
        // Eigen AlignedBox::setEmpty(): m_min = highest, m_max = lowest
        BoundingBox3f { min: Vec3f::new(f32::MAX, f32::MAX, f32::MAX), max: Vec3f::new(f32::MIN, f32::MIN, f32::MIN) }
    }
}

/// `std::min(a, b)` as Eigen's `cwiseMin` takes it: `b < a ? b : a`.
#[inline]
fn std_min_f32(a: f32, b: f32) -> f32 {
    if b < a {
        b
    } else {
        a
    }
}

/// `std::max(a, b)`: `a < b ? b : a`.
#[inline]
fn std_max_f32(a: f32, b: f32) -> f32 {
    if a < b {
        b
    } else {
        a
    }
}

impl BoundingBox3f {
    /// `AlignedBox(const OtherVectorType1 &_min, const OtherVectorType2 &_max)`
    pub fn new(min: Vec3f, max: Vec3f) -> Self {
        BoundingBox3f { min, max }
    }

    /// `AlignedBox::extend(const MatrixBase &p)`: `m_min = m_min.cwiseMin(p); m_max = m_max.cwiseMax(p)`
    pub fn extend(&mut self, p: &Vec3f) {
        self.min = Vec3f::new(std_min_f32(self.min.x, p.x), std_min_f32(self.min.y, p.y), std_min_f32(self.min.z, p.z));
        self.max = Vec3f::new(std_max_f32(self.max.x, p.x), std_max_f32(self.max.y, p.y), std_max_f32(self.max.z, p.z));
    }

    /// `AlignedBox::extend(const AlignedBox &b)`
    pub fn extend_box(&mut self, b: &BoundingBox3f) {
        self.min = Vec3f::new(std_min_f32(self.min.x, b.min.x), std_min_f32(self.min.y, b.min.y), std_min_f32(self.min.z, b.min.z));
        self.max = Vec3f::new(std_max_f32(self.max.x, b.max.x), std_max_f32(self.max.y, b.max.y), std_max_f32(self.max.z, b.max.z));
    }

    /// `AlignedBox::diagonal()` (= `sizes()`): `m_max - m_min`
    pub fn diagonal(&self) -> Vec3f {
        self.max - self.min
    }

    /// `AlignedBox::center()`: `(m_min + m_max) / 2`
    pub fn center(&self) -> Vec3f {
        let s = self.min + self.max;
        Vec3f::new(s.x / 2.0, s.y / 2.0, s.z / 2.0)
    }

    /// `AlignedBox::contains(const MatrixBase &p)`
    pub fn contains(&self, p: &Vec3f) -> bool {
        (self.min.x <= p.x && self.min.y <= p.y && self.min.z <= p.z) && (p.x <= self.max.x && p.y <= self.max.y && p.z <= self.max.z)
    }

    /// `AlignedBox::intersects(const AlignedBox &b)`
    pub fn intersects(&self, b: &BoundingBox3f) -> bool {
        (self.min.x <= b.max.x && self.min.y <= b.max.y && self.min.z <= b.max.z) && (b.min.x <= self.max.x && b.min.y <= self.max.y && b.min.z <= self.max.z)
    }

    /// `AlignedBox::squaredExteriorDistance(const MatrixBase &p)`: per axis outside the box,
    /// `aux = m_min[k] - p[k]` (or `p[k] - m_max[k]`) and `dist2 += aux * aux`, fused.
    pub fn squared_exterior_distance(&self, p: &Vec3f) -> f32 {
        let mut dist2 = 0.0f32;
        for k in 0..3 {
            if self.min[k] > p[k] {
                let aux = self.min[k] - p[k];
                dist2 = aux.mul_add(aux, dist2);
            } else if p[k] > self.max[k] {
                let aux = p[k] - self.max[k];
                dist2 = aux.mul_add(aux, dist2);
            }
        }
        dist2
    }
}

/// `Tree<3, float>::Node` (AABBTreeIndirect.hpp:56-72)
#[derive(Debug, Clone, Copy)]
pub struct NodeF {
    pub idx: usize,
    pub bbox: BoundingBox3f,
}

impl Default for NodeF {
    fn default() -> Self {
        NodeF { idx: constants::NPOS, bbox: BoundingBox3f::default() }
    }
}

impl NodeF {
    /// AABBTreeIndirect.hpp:63
    pub fn is_valid(&self) -> bool {
        self.idx != constants::NPOS
    }
    /// AABBTreeIndirect.hpp:64
    pub fn is_inner(&self) -> bool {
        self.idx == constants::INNER
    }
    /// AABBTreeIndirect.hpp:65
    pub fn is_leaf(&self) -> bool {
        !self.is_inner()
    }
}

/// The source node of a `Tree<3, float>` build (AABBTreeIndirect.hpp:76-84): an index, a float
/// box and a float centroid.
pub trait SourceNodeF {
    fn idx(&self) -> usize;
    fn bbox(&self) -> BoundingBox3f;
    fn centroid(&self) -> Vec3f;
}

/// `AABBTreeIndirect::Tree<3, float>` (`Tree3f`), AABBTreeIndirect.hpp:38-214, in float.
#[derive(Debug, Clone, Default)]
pub struct TreeF {
    nodes: Vec<NodeF>,
}

impl TreeF {
    pub fn new() -> Self {
        TreeF { nodes: Vec::new() }
    }

    /// AABBTreeIndirect.hpp:74
    pub fn clear(&mut self) {
        self.nodes.clear();
    }

    /// AABBTreeIndirect.hpp:85-90 `build(std::vector<SourceNode> &&input)`
    pub fn build<S: SourceNodeF>(&mut self, mut input: Vec<S>) {
        self.build_modify_input(&mut input);
        input.clear();
    }

    /// AABBTreeIndirect.hpp:92-102
    pub fn build_modify_input<S: SourceNodeF>(&mut self, input: &mut [S]) {
        if input.is_empty() {
            self.clear();
        } else {
            // Allocate enough memory for a full binary tree.
            self.nodes = vec![NodeF::default(); next_highest_power_of_2(input.len()) * 2 - 1];
            let right = input.len() - 1;
            self.build_recursive(input, 0, 0, right);
        }
    }

    /// AABBTreeIndirect.hpp:104
    pub fn nodes(&self) -> &[NodeF] {
        &self.nodes
    }
    /// AABBTreeIndirect.hpp:105
    pub fn node(&self, idx: usize) -> &NodeF {
        &self.nodes[idx]
    }
    /// AABBTreeIndirect.hpp:106
    pub fn empty(&self) -> bool {
        self.nodes.is_empty()
    }
    /// AABBTreeIndirect.hpp:109
    pub fn left_child_idx(idx: usize) -> usize {
        idx * 2 + 1
    }
    /// AABBTreeIndirect.hpp:110
    pub fn right_child_idx(idx: usize) -> usize {
        Self::left_child_idx(idx) + 1
    }

    /// AABBTreeIndirect.hpp:122-150
    fn build_recursive<S: SourceNodeF>(&mut self, input: &mut [S], node: usize, left: usize, right: usize) {
        debug_assert!(node < self.nodes.len());
        debug_assert!(left <= right);

        if left == right {
            // Insert a node into the balanced tree.
            self.nodes[node].idx = input[left].idx();
            self.nodes[node].bbox = input[left].bbox();
            return;
        }

        // Calculate bounding box of the input.
        let mut bbox = input[left].bbox();
        for i in (left + 1)..=right {
            bbox.extend_box(&input[i].bbox());
        }
        // bbox.diagonal().maxCoeff(&dimension): the first index of the largest coefficient
        let d = bbox.diagonal();
        let mut dimension = 0usize;
        for k in 1..3 {
            if d[k] > d[dimension] {
                dimension = k;
            }
        }

        // Partition the input to left / right pieces of the same length to produce a balanced tree.
        let center = (left + right) / 2;
        Self::partition_input(input, dimension, left, right, center);
        // Insert an inner node into the tree.
        self.nodes[node].idx = constants::INNER;
        self.nodes[node].bbox = bbox;
        self.build_recursive(input, node * 2 + 1, left, center);
        self.build_recursive(input, node * 2 + 2, center + 1, right);
    }

    /// AABBTreeIndirect.hpp:152-210 (QuickSelect)
    fn partition_input<S: SourceNodeF>(input: &mut [S], dimension: usize, mut left: usize, mut right: usize, k: usize) {
        while left < right {
            let center = (left + right) / 2;
            let pivot;
            {
                // Bubble sort the input[left], input[center], input[right], so that a median of the three values
                // will end up in input[center].
                let mut left_value = input[left].centroid()[dimension];
                let mut center_value = input[center].centroid()[dimension];
                let mut right_value = input[right].centroid()[dimension];
                if left_value > center_value {
                    input.swap(left, center);
                    std::mem::swap(&mut left_value, &mut center_value);
                }
                if left_value > right_value {
                    input.swap(left, right);
                    right_value = left_value;
                }
                if center_value > right_value {
                    input.swap(center, right);
                    center_value = right_value;
                }
                pivot = center_value;
            }
            if right <= left + 2 {
                // The <left, right> interval is already sorted.
                break;
            }
            let mut i = left;
            let mut j = right - 1;
            input.swap(center, j);
            // Partition the set based on the pivot.
            loop {
                // Skip left points that are already at correct positions.
                // Search will certainly stop at position (right - 1), which stores the pivot.
                loop {
                    i += 1;
                    if !(input[i].centroid()[dimension] < pivot) {
                        break;
                    }
                }
                // Skip right points that are already at correct positions.
                loop {
                    j -= 1;
                    if !(input[j].centroid()[dimension] > pivot && i < j) {
                        break;
                    }
                }
                if i >= j {
                    break;
                }
                input.swap(i, j);
            }
            // Restore pivot to the center of the sequence.
            input.swap(i, right - 1);
            // Which side the kth element is in?
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
}

/// The input node of `build_aabb_tree_over_indexed_triangle_set` (AABBTreeIndirect.hpp:704-712)
struct TriangleInputF {
    m_idx: usize,
    m_bbox: BoundingBox3f,
    m_centroid: Vec3f,
}

impl SourceNodeF for TriangleInputF {
    fn idx(&self) -> usize {
        self.m_idx
    }
    fn bbox(&self) -> BoundingBox3f {
        self.m_bbox
    }
    fn centroid(&self) -> Vec3f {
        self.m_centroid
    }
}

/// AABBTreeIndirect.hpp:686-736 `build_aabb_tree_over_indexed_triangle_set` over float vertices
/// (`Tree3f`). The centroid `(1./3.) * (v1 + v2 + v3)` is `float(1./3.) * ((v1 + v2) + v3)`.
pub fn build_aabb_tree_over_indexed_triangle_set_f(vertices: &[Vec3f], faces: &[Vec3i], eps: f32) -> TreeF {
    let mut input: Vec<TriangleInputF> = Vec::with_capacity(faces.len());
    let veps = Vec3f::new(eps, eps, eps);
    let third = (1.0f64 / 3.0f64) as f32;
    for (i, face) in faces.iter().enumerate() {
        let v1 = vertices[face[0] as usize];
        let v2 = vertices[face[1] as usize];
        let v3 = vertices[face[2] as usize];
        let s = (v1 + v2) + v3;
        let mut bbox = BoundingBox3f::new(v1, v1);
        bbox.extend(&v2);
        bbox.extend(&v3);
        bbox.min -= veps;
        bbox.max += veps;
        input.push(TriangleInputF { m_idx: i, m_bbox: bbox, m_centroid: Vec3f::new(third * s.x, third * s.y, third * s.z) });
    }
    let mut out = TreeF::new();
    out.build(input);
    out
}

/// `Vec3f::dot`, `fma(a.z, b.z, a.x * b.x + a.y * b.y)` (R880)
#[inline]
fn dot_f(a: &Vec3f, b: &Vec3f) -> f32 {
    a.z.mul_add(b.z, a.x * b.x + a.y * b.y)
}

/// AABBTreeIndirect.hpp:473-521 `closest_point_to_triangle` with `Vector = Vec3f`
pub fn closest_point_to_triangle_f(p: &Vec3f, a: &Vec3f, b: &Vec3f, c: &Vec3f) -> Vec3f {
    // Check if P in vertex region outside A
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = dot_f(&ab, &ap);
    let d2 = dot_f(&ac, &ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return *a;
    }
    // Check if P in vertex region outside B
    let bp = p - b;
    let d3 = dot_f(&ab, &bp);
    let d4 = dot_f(&ac, &bp);
    if d3 >= 0.0 && d4 <= d3 {
        return *b;
    }
    // Check if P in edge region of AB, if so return projection of P onto AB
    let vc = d1.mul_add(d4, -(d3 * d2));
    if a != b && vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return Vec3f::new(a.x + v * ab.x, a.y + v * ab.y, a.z + v * ab.z);
    }
    // Check if P in vertex region outside C
    let cp = p - c;
    let d5 = dot_f(&ab, &cp);
    let d6 = dot_f(&ac, &cp);
    if d6 >= 0.0 && d5 <= d6 {
        return *c;
    }
    // Check if P in edge region of AC, if so return projection of P onto AC
    let vb = d5.mul_add(d2, -(d1 * d6));
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return Vec3f::new(a.x + w * ac.x, a.y + w * ac.y, a.z + w * ac.z);
    }
    // Check if P in edge region of BC, if so return projection of P onto BC
    let va = d3.mul_add(d6, -(d5 * d4));
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        let cb = c - b;
        return Vec3f::new(b.x + w * cb.x, b.y + w * cb.y, b.z + w * cb.z);
    }
    // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
    let denom = 1.0f32 / ((va + vb) + vc);
    let v = vb * denom;
    let w = vc * denom;
    // a + ab * v + ac * w, coefficient by coefficient, not fused
    Vec3f::new((a.x + ab.x * v) + ac.x * w, (a.y + ab.y * v) + ac.y * w, (a.z + ab.z * v) + ac.z * w)
}

/// AABBTreeIndirect.hpp:570-651 `squared_distance_to_indexed_primitives_recursive` over an
/// `IndexedTriangleSetDistancer` (539-548) with `VectorType = Vec3f`.
#[allow(clippy::too_many_arguments)]
fn squared_distance_to_indexed_triangles_recursive_f(
    vertices: &[Vec3f],
    faces: &[Vec3i],
    tree: &TreeF,
    origin: &Vec3f,
    node_idx: usize,
    low_sqr_d: f32,
    mut up_sqr_d: f32,
    i: &mut usize,
    c: &mut Vec3f,
) -> f32 {
    if low_sqr_d > up_sqr_d {
        return low_sqr_d;
    }

    // Save the best achieved hit.
    let set_min = |sqr_d_candidate: f32, i_candidate: usize, c_candidate: &Vec3f, i: &mut usize, c: &mut Vec3f, up_sqr_d: &mut f32| {
        if sqr_d_candidate < *up_sqr_d {
            *i = i_candidate;
            *c = *c_candidate;
            *up_sqr_d = sqr_d_candidate;
        }
    };

    let node = *tree.node(node_idx);
    debug_assert!(node.is_valid());
    if node.is_leaf() {
        // IndexedTriangleSetDistancer::closest_point_to_origin
        let triangle = &faces[node.idx];
        let c_candidate = closest_point_to_triangle_f(origin, &vertices[triangle[0] as usize], &vertices[triangle[1] as usize], &vertices[triangle[2] as usize]);
        let diff = origin - c_candidate;
        let sqr_dist = (diff.x * diff.x + diff.y * diff.y) + diff.z * diff.z;
        set_min(sqr_dist, node.idx, &c_candidate, i, c, &mut up_sqr_d);
    } else {
        let left_node_idx = node_idx * 2 + 1;
        let right_node_idx = left_node_idx + 1;
        let node_left = *tree.node(left_node_idx);
        let node_right = *tree.node(right_node_idx);
        debug_assert!(node_left.is_valid());
        debug_assert!(node_right.is_valid());

        let mut looked_left = false;
        let mut looked_right = false;
        macro_rules! look_left {
            () => {{
                let mut i_left: usize = 0;
                let mut c_left = *c;
                let sqr_d_left = squared_distance_to_indexed_triangles_recursive_f(vertices, faces, tree, origin, left_node_idx, low_sqr_d, up_sqr_d, &mut i_left, &mut c_left);
                set_min(sqr_d_left, i_left, &c_left, i, c, &mut up_sqr_d);
                looked_left = true;
                let _ = looked_left;
            }};
        }
        macro_rules! look_right {
            () => {{
                let mut i_right: usize = 0;
                let mut c_right = *c;
                let sqr_d_right = squared_distance_to_indexed_triangles_recursive_f(vertices, faces, tree, origin, right_node_idx, low_sqr_d, up_sqr_d, &mut i_right, &mut c_right);
                set_min(sqr_d_right, i_right, &c_right, i, c, &mut up_sqr_d);
                looked_right = true;
                let _ = looked_right;
            }};
        }

        // must look left or right if in box
        if node_left.bbox.contains(origin) {
            look_left!();
        }
        if node_right.bbox.contains(origin) {
            look_right!();
        }
        // if haven't looked left and could be less than current min, then look
        let left_up_sqr_d = node_left.bbox.squared_exterior_distance(origin);
        let right_up_sqr_d = node_right.bbox.squared_exterior_distance(origin);
        if left_up_sqr_d < right_up_sqr_d {
            if !looked_left && left_up_sqr_d < up_sqr_d {
                look_left!();
            }
            if !looked_right && right_up_sqr_d < up_sqr_d {
                look_right!();
            }
        } else {
            if !looked_right && right_up_sqr_d < up_sqr_d {
                look_right!();
            }
            if !looked_left && left_up_sqr_d < up_sqr_d {
                look_left!();
            }
        }
    }
    up_sqr_d
}

/// AABBTreeIndirect.hpp:809-834 `squared_distance_to_indexed_triangle_set` with float vertices, a
/// `Tree3f` and a `Vec3f` point: the squared distance to the closest point (in float), or -1 if
/// the tree is empty.
pub fn squared_distance_to_indexed_triangle_set_f(vertices: &[Vec3f], faces: &[Vec3i], tree: &TreeF, point: &Vec3f, hit_idx_out: &mut usize, hit_point_out: &mut Vec3f) -> f32 {
    if tree.empty() {
        -1.0
    } else {
        squared_distance_to_indexed_triangles_recursive_f(vertices, faces, tree, point, 0, 0.0, f32::INFINITY, hit_idx_out, hit_point_out)
    }
}

/// AABBTreeIndirect.hpp:994-1006 `traverse` over a `Tree3f`
pub fn traverse_f<P, F>(tree: &TreeF, mut pred: P, mut callback: F)
where
    P: FnMut(&NodeF) -> bool,
    F: FnMut(&NodeF) -> bool,
{
    if tree.empty() {
        return;
    }
    traverse_recurse_f(tree, 0, &mut pred, &mut callback);
}

/// AABBTreeIndirect.hpp:960-990 `detail::traverse_recurse` over a `Tree3f`
fn traverse_recurse_f<P, F>(tree: &TreeF, idx: usize, pred: &mut P, callback: &mut F) -> bool
where
    P: FnMut(&NodeF) -> bool,
    F: FnMut(&NodeF) -> bool,
{
    debug_assert!(tree.node(idx).is_valid());
    if !pred(tree.node(idx)) {
        // Continue traversal.
        return true;
    }
    if tree.node(idx).is_leaf() {
        // Callback returns true to continue traversal, false to stop traversal.
        callback(tree.node(idx))
    } else {
        traverse_recurse_f(tree, TreeF::left_child_idx(idx), pred, callback) && traverse_recurse_f(tree, TreeF::right_child_idx(idx), pred, callback)
    }
}

/// AABBTreeIndirect.hpp:926-939 `intersecting(const Eigen::AlignedBox<float, 3> &)`: the predicate
/// `box.intersects(node.bbox)`.
pub fn intersecting_f(bbox: BoundingBox3f) -> impl FnMut(&NodeF) -> bool {
    move |node: &NodeF| bbox.intersects(&node.bbox)
}

#[cfg(test)]
mod tests {
    use super::*;

    // Simple test source node
    struct TestNode {
        idx: usize,
        bbox: BoundingBox3F,
        centroid: Point3F,
    }

    impl SourceNode for TestNode {
        fn idx(&self) -> usize {
            self.idx
        }

        fn centroid(&self) -> Point3F {
            self.centroid
        }

        fn bbox(&self) -> BoundingBox3F {
            self.bbox.clone()
        }
    }

    #[test]
    fn test_next_highest_power_of_2() {
        assert_eq!(next_highest_power_of_2(0), 1);
        assert_eq!(next_highest_power_of_2(1), 1);
        assert_eq!(next_highest_power_of_2(2), 2);
        assert_eq!(next_highest_power_of_2(3), 4);
        assert_eq!(next_highest_power_of_2(7), 8);
        assert_eq!(next_highest_power_of_2(8), 8);
        assert_eq!(next_highest_power_of_2(9), 16);
    }

    #[test]
    fn test_tree_build() {
        let nodes = vec![
            TestNode {
                idx: 0,
                bbox: BoundingBox3F::from_points_minmax(
                    Point3F::new(0.0, 0.0, 0.0),
                    Point3F::new(1.0, 1.0, 1.0),
                ),
                centroid: Point3F::new(0.5, 0.5, 0.5),
            },
            TestNode {
                idx: 1,
                bbox: BoundingBox3F::from_points_minmax(
                    Point3F::new(2.0, 0.0, 0.0),
                    Point3F::new(3.0, 1.0, 1.0),
                ),
                centroid: Point3F::new(2.5, 0.5, 0.5),
            },
        ];

        let mut tree = Tree::new();
        tree.build(nodes);

        assert!(!tree.empty());
        assert!(tree.nodes().len() > 0);
    }

    #[test]
    fn test_ray_triangle_intersection() {
        let origin = Point3F::new(0.0, 0.0, 5.0);
        let dir = Point3F::new(0.0, 0.0, -1.0);

        let v0 = Point3F::new(-1.0, -1.0, 0.0);
        let v1 = Point3F::new(1.0, -1.0, 0.0);
        let v2 = Point3F::new(0.0, 1.0, 0.0);

        let result = intersect_triangle(&origin, &dir, &v0, &v1, &v2, 0.000001);

        assert!(result.is_some());
        let (t, _u, _v) = result.unwrap();
        assert!(t > 0.0);
    }

    #[test]
    fn test_closest_point_to_triangle() {
        let a = Point3F::new(0.0, 0.0, 0.0);
        let b = Point3F::new(1.0, 0.0, 0.0);
        let c = Point3F::new(0.0, 1.0, 0.0);

        // Point above triangle center
        let p = Point3F::new(0.3, 0.3, 1.0);
        let closest = closest_point_to_triangle(&p, &a, &b, &c);

        // Should project onto triangle plane
        assert!((closest.z - 0.0).abs() < 0.001);
    }
}
