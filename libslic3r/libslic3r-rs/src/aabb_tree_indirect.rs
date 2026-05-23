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
                // Skip left points that are already at correct positions
                // AABBTreeIndirect.hpp:191-192
                loop {
                    i += 1;
                    if !(self.get_centroid_coord(&input[i], dimension) < pivot) {
                        break;
                    }
                }

                // Skip right points that are already at correct positions
                // AABBTreeIndirect.hpp:194-195
                loop {
                    if j == 0 || i >= j {
                        break;
                    }
                    if !(self.get_centroid_coord(&input[j - 1], dimension) > pivot) {
                        break;
                    }
                    j -= 1;
                }

                // AABBTreeIndirect.hpp:196-197
                if i >= j {
                    break;
                }

                // AABBTreeIndirect.hpp:198
                input.swap(i, j - 1);
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
    Point3F::new(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    )
}

/// Dot product of two 3D vectors
/// AABBTreeIndirect.hpp:324 (inline)
fn dot_product(a: &Point3F, b: &Point3F) -> f64 {
    a.x * b.x + a.y * b.y + a.z * b.z
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

        let centroid = Point3F {
            x: (v1.x + v2.x + v3.x) / 3.0,
            y: (v1.y + v2.y + v3.y) / 3.0,
            z: (v1.z + v2.z + v3.z) / 3.0,
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
    // Traverse tree looking for ray-triangle intersections
    // AABBTreeIndirect.hpp:738-750
    // C++: hit.t = std::numeric_limits<Scalar>::infinity();
    // C++: traverse(tree, [&origin, &dir](const BoundingBox &bbox) {
    // C++:     return intersect_ray_bbox(origin, dir, bbox);
    // C++: }, [&vertices, &faces, &origin, &dir, &hit](size_t face_idx) {
    // C++:     // Test ray-triangle intersection
    // C++: });
    let mut best_hit: Option<(f64, usize, Point3F)> = None;
    let mut best_t = f64::INFINITY;

    for node in tree.nodes() {
        if !node.is_leaf() {
            continue;
        }

        let face_idx = node.idx;
        if face_idx >= faces.len() {
            continue;
        }

        let face = &faces[face_idx];
        let v0 = &vertices[face[0]];
        let v1 = &vertices[face[1]];
        let v2 = &vertices[face[2]];

        if let Some((t, _u, _v)) = intersect_triangle(origin, dir, v0, v1, v2, 0.000001) {
            if t > 0.0 && t < best_t {
                let hit_point = Point3F {
                    x: origin.x + dir.x * t,
                    y: origin.y + dir.y * t,
                    z: origin.z + dir.z * t,
                };
                best_t = t;
                best_hit = Some((t, face_idx, hit_point));
            }
        }
    }

    best_hit
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
    // Traverse tree collecting all ray-triangle intersections
    // AABBTreeIndirect.hpp:769-791
    // C++: hits.clear();
    // C++: traverse(tree, [&origin, &dir](const BoundingBox &bbox) {
    // C++:     return intersect_ray_bbox(origin, dir, bbox);
    // C++: }, [&vertices, &faces, &origin, &dir, &hits](size_t face_idx) {
    // C++:     // Test ray-triangle intersection and collect all hits
    // C++: });
    let mut hits = Vec::new();

    for node in tree.nodes() {
        if !node.is_leaf() {
            continue;
        }

        let face_idx = node.idx;
        if face_idx >= faces.len() {
            continue;
        }

        let face = &faces[face_idx];
        let v0 = &vertices[face[0]];
        let v1 = &vertices[face[1]];
        let v2 = &vertices[face[2]];

        if let Some((t, _u, _v)) = intersect_triangle(origin, dir, v0, v1, v2, 0.000001) {
            if t > 0.0 {
                let hit_point = Point3F {
                    x: origin.x + dir.x * t,
                    y: origin.y + dir.y * t,
                    z: origin.z + dir.z * t,
                };
                hits.push((t, face_idx, hit_point));
            }
        }
    }

    hits
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
    // Traverse tree finding closest point on any triangle
    // AABBTreeIndirect.hpp:817-855
    // C++: Scalar min_sqr_d = std::numeric_limits<Scalar>::infinity();
    // C++: size_t min_face = 0;
    // C++: VectorType min_point;
    // C++: traverse(tree, [&point, &min_sqr_d](const BoundingBox &bbox) {
    // C++:     return bbox.squaredExteriorDistance(point) < min_sqr_d;
    // C++: }, [&vertices, &faces, &point, &min_sqr_d, &min_face, &min_point](size_t face_idx) {
    // C++:     // Find closest point on triangle and update minimum
    // C++: });
    let mut min_dist_sq = f64::INFINITY;
    let mut min_face = 0;
    let mut min_point = point;

    for node in tree.nodes() {
        if !node.is_leaf() {
            continue;
        }

        let face_idx = node.idx;
        if face_idx >= faces.len() {
            continue;
        }

        let face = &faces[face_idx];
        let v0 = &vertices[face[0]];
        let v1 = &vertices[face[1]];
        let v2 = &vertices[face[2]];

        let point_p3f = Point3F::new(point.x, point.y, point.z);
        let closest = closest_point_to_triangle(&point_p3f, v0, v1, v2);
        let dx = closest.x - point.x;
        let dy = closest.y - point.y;
        let dz = closest.z - point.z;
        let dist_sq = dx * dx + dy * dy + dz * dz;

        if dist_sq < min_dist_sq {
            min_dist_sq = dist_sq;
            min_face = face_idx;
            min_point = Vec3::new(closest.x, closest.y, closest.z);
        }
    }

    (min_dist_sq, min_face, min_point)
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
                bbox: BoundingBox3F::new(Point3F::new(0.0, 0.0, 0.0), Point3F::new(1.0, 1.0, 1.0)),
                centroid: Point3F::new(0.5, 0.5, 0.5),
            },
            TestNode {
                idx: 1,
                bbox: BoundingBox3F::new(Point3F::new(2.0, 0.0, 0.0), Point3F::new(3.0, 1.0, 1.0)),
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
