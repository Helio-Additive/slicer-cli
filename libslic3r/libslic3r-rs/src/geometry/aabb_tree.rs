//! AABB Tree for spatial acceleration of mesh queries.
//!
//! This module provides an axis-aligned bounding box tree for efficient:
//! - Ray casting (first hit, all hits)
//! - Closest point queries
//! - Distance queries
//! - Range queries (find all primitives within a distance)
//!
//! # Algorithm
//!
//! The tree is a balanced binary tree built over bounding boxes of primitives
//! (triangles, line segments, points). The tree is balanced by splitting
//! primitives at each level along the longest axis of their combined bounding box.
//!
//! Tree storage uses an implicit indexing scheme where children of node `i` are
//! at positions `2*i + 1` (left) and `2*i + 2` (right). This eliminates the need
//! for explicit child pointers and improves cache locality.
//!
//! # BambuStudio Reference
//!
//! This corresponds to:
//! - `src/libslic3r/AABBTreeIndirect.hpp`
//! - `src/libslic3r/AABBTreeLines.hpp`
//!
//! The implementation is based on libigl's AABB tree with memory optimizations
//! from PrusaSlicer/BambuStudio.

use crate::geometry::{BoundingBox3, Point3};
use crate::CoordF;

/// A 3D vector type for AABB tree calculations.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec3 {
    pub x: CoordF,
    pub y: CoordF,
    pub z: CoordF,
}

impl Vec3 {
    #[inline]
    pub fn new(x: CoordF, y: CoordF, z: CoordF) -> Self {
        Self { x, y, z }
    }

    #[inline]
    pub fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    #[inline]
    pub fn splat(v: CoordF) -> Self {
        Self::new(v, v, v)
    }

    /// Get the x coordinate.
    #[inline]
    pub const fn x(&self) -> CoordF {
        self.x
    }

    /// Get the y coordinate.
    #[inline]
    pub const fn y(&self) -> CoordF {
        self.y
    }

    /// Get the z coordinate.
    #[inline]
    pub const fn z(&self) -> CoordF {
        self.z
    }

    #[inline]
    pub fn dot(&self, other: &Self) -> CoordF {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    #[inline]
    pub fn cross(&self, other: &Self) -> Self {
        Self {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }

    #[inline]
    pub fn length_squared(&self) -> CoordF {
        self.dot(self)
    }

    #[inline]
    pub fn length(&self) -> CoordF {
        self.length_squared().sqrt()
    }

    /// Alias for length() to match C++ API
    /// C++: T norm() const { return length(); }
    #[inline]
    pub fn norm(&self) -> CoordF {
        self.length()
    }

    #[inline]
    pub fn normalized(&self) -> Self {
        let len = self.length();
        if len > 1e-10 {
            Self::new(self.x / len, self.y / len, self.z / len)
        } else {
            *self
        }
    }

    #[inline]
    pub fn min(&self, other: &Self) -> Self {
        Self::new(
            self.x.min(other.x),
            self.y.min(other.y),
            self.z.min(other.z),
        )
    }

    #[inline]
    pub fn max(&self, other: &Self) -> Self {
        Self::new(
            self.x.max(other.x),
            self.y.max(other.y),
            self.z.max(other.z),
        )
    }

    #[inline]
    pub fn component(&self, idx: usize) -> CoordF {
        match idx {
            0 => self.x,
            1 => self.y,
            _ => self.z,
        }
    }

    #[inline]
    pub fn set_component(&mut self, idx: usize, value: CoordF) {
        match idx {
            0 => self.x = value,
            1 => self.y = value,
            _ => self.z = value,
        }
    }
}

impl std::ops::Add for Vec3 {
    type Output = Self;
    #[inline]
    fn add(self, other: Self) -> Self {
        Self::new(self.x + other.x, self.y + other.y, self.z + other.z)
    }
}

impl std::ops::Sub for Vec3 {
    type Output = Self;
    #[inline]
    fn sub(self, other: Self) -> Self {
        Self::new(self.x - other.x, self.y - other.y, self.z - other.z)
    }
}

impl std::ops::Mul<CoordF> for Vec3 {
    type Output = Self;
    #[inline]
    fn mul(self, s: CoordF) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }
}

impl std::ops::Div<f32> for Vec3 {
    type Output = Self;
    #[inline]
    fn div(self, s: f32) -> Self {
        Self::new(self.x / s as f64, self.y / s as f64, self.z / s as f64)
    }
}

impl std::ops::Div<f64> for Vec3 {
    type Output = Self;
    #[inline]
    fn div(self, s: f64) -> Self {
        Self::new(self.x / s, self.y / s, self.z / s)
    }
}

// CoordF is f64, so Div<f64> already covers this case

impl std::ops::Neg for Vec3 {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}

impl From<Point3> for Vec3 {
    fn from(p: Point3) -> Self {
        Self::new(p.x as CoordF, p.y as CoordF, p.z as CoordF)
    }
}

/// 3D axis-aligned bounding box for the AABB tree.
#[derive(Debug, Clone, Copy)]
pub struct AABB3 {
    pub min: Vec3,
    pub max: Vec3,
}

impl AABB3 {
    // Create a new AABB from min and max corners.
    #[inline]
    pub fn new(min: Vec3, max: Vec3) -> Self {
        Self { min, max }
    }

    /// Create an empty (inverted) AABB.
    #[inline]
    pub fn empty() -> Self {
        Self {
            min: Vec3::splat(CoordF::MAX),
            max: Vec3::splat(CoordF::MIN),
        }
    }

    /// Create an AABB containing a single point.
    #[inline]
    pub fn from_point(p: Vec3) -> Self {
        Self { min: p, max: p }
    }

    /// Create an AABB from three triangle vertices.
    #[inline]
    pub fn from_triangle(v0: &Vec3, v1: &Vec3, v2: &Vec3) -> Self {
        Self {
            min: v0.min(v1).min(v2),
            max: v0.max(v1).max(v2),
        }
    }

    /// Extend the AABB to include a point.
    #[inline]
    pub fn extend_point(&mut self, p: &Vec3) {
        self.min = self.min.min(p);
        self.max = self.max.max(p);
    }

    /// Extend the AABB to include another AABB.
    #[inline]
    pub fn extend_box(&mut self, other: &AABB3) {
        self.min = self.min.min(&other.min);
        self.max = self.max.max(&other.max);
    }

    /// Get the center of the AABB.
    #[inline]
    pub fn center(&self) -> Vec3 {
        Vec3::new(
            (self.min.x + self.max.x) * 0.5,
            (self.min.y + self.max.y) * 0.5,
            (self.min.z + self.max.z) * 0.5,
        )
    }

    /// Get the diagonal (size) of the AABB.
    #[inline]
    pub fn diagonal(&self) -> Vec3 {
        self.max - self.min
    }

    /// Get the index of the longest axis (0=X, 1=Y, 2=Z).
    #[inline]
    pub fn longest_axis(&self) -> usize {
        let d = self.diagonal();
        if d.x >= d.y && d.x >= d.z {
            0
        } else if d.y >= d.z {
            1
        } else {
            2
        }
    }

    /// Check if the AABB contains a point.
    #[inline]
    pub fn contains(&self, p: &Vec3) -> bool {
        p.x >= self.min.x
            && p.x <= self.max.x
            && p.y >= self.min.y
            && p.y <= self.max.y
            && p.z >= self.min.z
            && p.z <= self.max.z
    }

    /// Calculate the squared distance from a point to the AABB exterior.
    /// Returns 0 if the point is inside the AABB.
    #[inline]
    pub fn squared_exterior_distance(&self, p: &Vec3) -> CoordF {
        let mut dist_sq = 0.0;

        for i in 0..3 {
            let v = p.component(i);
            let min_v = self.min.component(i);
            let max_v = self.max.component(i);

            if v < min_v {
                let d = min_v - v;
                dist_sq += d * d;
            } else if v > max_v {
                let d = v - max_v;
                dist_sq += d * d;
            }
        }

        dist_sq
    }

    /// Inflate the AABB by epsilon in all directions.
    #[inline]
    pub fn inflate(&mut self, eps: CoordF) {
        self.min = self.min - Vec3::splat(eps);
        self.max = self.max + Vec3::splat(eps);
    }

    /// Create an inflated copy of the AABB.
    #[inline]
    pub fn inflated(&self, eps: CoordF) -> Self {
        Self {
            min: self.min - Vec3::splat(eps),
            max: self.max + Vec3::splat(eps),
        }
    }
}

impl Default for AABB3 {
    fn default() -> Self {
        Self::empty()
    }
}

impl From<BoundingBox3> for AABB3 {
    fn from(bb: BoundingBox3) -> Self {
        Self {
            min: Vec3::new(bb.min.x as CoordF, bb.min.y as CoordF, bb.min.z as CoordF),
            max: Vec3::new(bb.max.x as CoordF, bb.max.y as CoordF, bb.max.z as CoordF),
        }
    }
}

/// Special index values for tree nodes.
const NPOS: usize = usize::MAX;
const INNER: usize = usize::MAX - 1;

/// A single node in the AABB tree.
#[derive(Debug, Clone)]
pub struct AABBNode {
    /// Index of the primitive (triangle, etc.) for leaf nodes.
    /// INNER for internal nodes, NPOS for invalid nodes.
    pub idx: usize,
    /// Bounding box of this node.
    pub bbox: AABB3,
}

impl AABBNode {
    // Create an invalid (empty) node.
    #[inline]
    pub fn empty() -> Self {
        Self {
            idx: NPOS,
            bbox: AABB3::empty(),
        }
    }

    /// Check if this node is valid.
    #[inline]
    pub fn is_valid(&self) -> bool {
        self.idx != NPOS
    }

    /// Check if this is an internal (non-leaf) node.
    #[inline]
    pub fn is_inner(&self) -> bool {
        self.idx == INNER
    }

    /// Check if this is a leaf node.
    #[inline]
    pub fn is_leaf(&self) -> bool {
        self.is_valid() && !self.is_inner()
    }
}

impl Default for AABBNode {
    fn default() -> Self {
        Self::empty()
    }
}

/// Input item for building the AABB tree.
#[derive(Debug, Clone)]
struct BuildInput {
    /// Index of the primitive.
    idx: usize,
    /// Bounding box of the primitive.
    bbox: AABB3,
    /// Centroid of the primitive (used for balancing).
    centroid: Vec3,
}

/// Result of a ray intersection test.
#[derive(Debug, Clone, Copy)]
pub struct RayHit {
    /// Index of the hit primitive (triangle).
    pub primitive_idx: usize,
    /// Distance along the ray to the hit point.
    pub t: CoordF,
    /// Barycentric U coordinate.
    pub u: CoordF,
    /// Barycentric V coordinate.
    pub v: CoordF,
}

impl RayHit {
    // Create a new ray hit.
    pub fn new(primitive_idx: usize, t: CoordF, u: CoordF, v: CoordF) -> Self {
        Self {
            primitive_idx,
            t,
            u,
            v,
        }
    }
}

/// Result of a closest point query on an AABB tree.
#[derive(Debug, Clone, Copy)]
pub struct AABBClosestPointResult {
    /// Index of the closest primitive.
    pub primitive_idx: usize,
    /// The closest point on the primitive.
    pub point: Vec3,
    /// Squared distance to the closest point.
    pub squared_distance: CoordF,
}

impl AABBClosestPointResult {
    // Create a new closest point result.
    pub fn new(primitive_idx: usize, point: Vec3, squared_distance: CoordF) -> Self {
        Self {
            primitive_idx,
            point,
            squared_distance,
        }
    }

    /// Get the distance to the closest point.
    pub fn distance(&self) -> CoordF {
        self.squared_distance.sqrt()
    }
}

/// A balanced AABB tree for spatial queries.
///
/// The tree is built over a collection of primitives (typically triangles)
/// and supports efficient ray casting and closest point queries.
#[derive(Debug, Clone)]
pub struct AABBTree {
    /// The nodes of the tree stored in a flat array.
    /// Children of node i are at 2*i+1 (left) and 2*i+2 (right).
    nodes: Vec<AABBNode>,
}

impl AABBTree {
    // Create an empty AABB tree.
    pub fn new() -> Self {
        Self { nodes: Vec::new() }
    }

    /// Check if the tree is empty.
    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }

    /// Get the number of nodes in the tree.
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Get a reference to a node by index.
    pub fn node(&self, idx: usize) -> Option<&AABBNode> {
        self.nodes.get(idx)
    }

    /// Get the root node.
    pub fn root(&self) -> Option<&AABBNode> {
        self.nodes.first()
    }

    /// Get the left child index of a node.
    #[inline]
    pub fn left_child_idx(idx: usize) -> usize {
        idx * 2 + 1
    }

    /// Get the right child index of a node.
    #[inline]
    pub fn right_child_idx(idx: usize) -> usize {
        idx * 2 + 2
    }

    /// Build the tree from a list of triangles.
    ///
    /// # Arguments
    /// * `vertices` - The vertex positions
    /// * `triangles` - Indices into the vertex array (3 indices per triangle)
    /// * `eps` - Epsilon to inflate bounding boxes (for numerical stability)
    pub fn build_from_triangles(vertices: &[Vec3], triangles: &[[usize; 3]], eps: CoordF) -> Self {
        if triangles.is_empty() {
            return Self::new();
        }

        // Build input with bounding boxes and centroids
        let mut input: Vec<BuildInput> = triangles
            .iter()
            .enumerate()
            .map(|(idx, tri)| {
                let v0 = &vertices[tri[0]];
                let v1 = &vertices[tri[1]];
                let v2 = &vertices[tri[2]];

                let mut bbox = AABB3::from_triangle(v0, v1, v2);
                if eps > 0.0 {
                    bbox.inflate(eps);
                }

                let centroid = Vec3::new(
                    (v0.x + v1.x + v2.x) / 3.0,
                    (v0.y + v1.y + v2.y) / 3.0,
                    (v0.z + v1.z + v2.z) / 3.0,
                );

                BuildInput {
                    idx,
                    bbox,
                    centroid,
                }
            })
            .collect();

        let input_len = input.len();
        let mut tree = Self {
            nodes: vec![AABBNode::empty(); next_power_of_2(input_len) * 2 - 1],
        };

        tree.build_recursive(&mut input, 0, 0, input_len - 1);
        tree
    }

    /// Build the tree recursively.
    fn build_recursive(
        &mut self,
        input: &mut [BuildInput],
        node_idx: usize,
        left: usize,
        right: usize,
    ) {
        debug_assert!(node_idx < self.nodes.len());
        debug_assert!(left <= right);

        if left == right {
            // Leaf node
            self.nodes[node_idx].idx = input[left].idx;
            self.nodes[node_idx].bbox = input[left].bbox;
            return;
        }

        // Calculate combined bounding box
        let mut bbox = input[left].bbox;
        for i in (left + 1)..=right {
            bbox.extend_box(&input[i].bbox);
        }

        // Find the longest axis to split on
        let dimension = bbox.longest_axis();

        // Partition around the median
        let center = (left + right) / 2;
        Self::partition_input(input, dimension, left, right, center);

        // Set up this node as inner node
        self.nodes[node_idx].idx = INNER;
        self.nodes[node_idx].bbox = bbox;

        // Recursively build children
        self.build_recursive(input, Self::left_child_idx(node_idx), left, center);
        self.build_recursive(input, Self::right_child_idx(node_idx), center + 1, right);
    }

    /// Partition input using QuickSelect algorithm.
    /// After partitioning, elements < k are smaller than element[k] in the given dimension.
    fn partition_input(
        input: &mut [BuildInput],
        dimension: usize,
        mut left: usize,
        mut right: usize,
        k: usize,
    ) {
        while left < right {
            let center = (left + right) / 2;

            // Median-of-three pivot selection - get values first
            let left_val = input[left].centroid.component(dimension);
            let center_val = input[center].centroid.component(dimension);
            let _right_val = input[right].centroid.component(dimension);

            // Sort left, center, right
            if left_val > center_val {
                input.swap(left, center);
            }
            // Re-read values after potential swap
            let new_left_val = input[left].centroid.component(dimension);
            let new_right_val = input[right].centroid.component(dimension);
            if new_left_val > new_right_val {
                input.swap(left, right);
            }
            // Re-read values after potential swap
            let new_center_val = input[center].centroid.component(dimension);
            let final_right_val = input[right].centroid.component(dimension);
            if new_center_val > final_right_val {
                input.swap(center, right);
            }

            let pivot = input[center].centroid.component(dimension);

            if right <= left + 2 {
                // Already sorted
                break;
            }

            let mut i = left;
            let mut j = right - 1;
            input.swap(center, j);

            // Partition
            loop {
                loop {
                    i += 1;
                    if input[i].centroid.component(dimension) >= pivot {
                        break;
                    }
                }
                loop {
                    j -= 1;
                    if input[j].centroid.component(dimension) <= pivot || i >= j {
                        break;
                    }
                }
                if i >= j {
                    break;
                }
                input.swap(i, j);
            }

            // Restore pivot
            input.swap(i, right - 1);

            // Narrow the search
            if k < i {
                right = i - 1;
            } else if k == i {
                break;
            } else {
                left = i + 1;
            }
        }
    }
}

impl Default for AABBTree {
    fn default() -> Self {
        Self::new()
    }
}

/// Ray-box intersection test using the slab method.
///
/// Returns true if the ray intersects the box within the range [t0, t1].
pub fn ray_box_intersect(
    origin: &Vec3,
    inv_dir: &Vec3,
    bbox: &AABB3,
    t0: CoordF,
    t1: CoordF,
) -> bool {
    let mut tmin;
    let mut tmax;

    // X slab
    if inv_dir.x >= 0.0 {
        tmin = (bbox.min.x - origin.x) * inv_dir.x;
        tmax = (bbox.max.x - origin.x) * inv_dir.x;
    } else {
        tmin = (bbox.max.x - origin.x) * inv_dir.x;
        tmax = (bbox.min.x - origin.x) * inv_dir.x;
    }

    // Y slab
    let (tymin, tymax) = if inv_dir.y >= 0.0 {
        (
            (bbox.min.y - origin.y) * inv_dir.y,
            (bbox.max.y - origin.y) * inv_dir.y,
        )
    } else {
        (
            (bbox.max.y - origin.y) * inv_dir.y,
            (bbox.min.y - origin.y) * inv_dir.y,
        )
    };

    if tmin > tymax || tymin > tmax {
        return false;
    }

    if tymin > tmin {
        tmin = tymin;
    }
    if tymax < tmax {
        tmax = tymax;
    }

    // Z slab
    let (tzmin, tzmax) = if inv_dir.z >= 0.0 {
        (
            (bbox.min.z - origin.z) * inv_dir.z,
            (bbox.max.z - origin.z) * inv_dir.z,
        )
    } else {
        (
            (bbox.max.z - origin.z) * inv_dir.z,
            (bbox.min.z - origin.z) * inv_dir.z,
        )
    };

    if tmin > tzmax || tzmin > tmax {
        return false;
    }

    if tzmin > tmin {
        tmin = tzmin;
    }
    if tzmax < tmax {
        tmax = tzmax;
    }

    tmin < t1 && tmax > t0
}

/// Möller–Trumbore ray-triangle intersection algorithm.
///
/// Returns (t, u, v) where t is the distance along the ray and (u, v) are
/// barycentric coordinates.
pub fn ray_triangle_intersect(
    origin: &Vec3,
    dir: &Vec3,
    v0: &Vec3,
    v1: &Vec3,
    v2: &Vec3,
    eps: CoordF,
) -> Option<(CoordF, CoordF, CoordF)> {
    let edge1 = *v1 - *v0;
    let edge2 = *v2 - *v0;

    let pvec = dir.cross(&edge2);
    let det = edge1.dot(&pvec);

    if det.abs() < eps {
        // Ray is parallel to triangle
        return None;
    }

    let inv_det = 1.0 / det;
    let tvec = *origin - *v0;

    let u = tvec.dot(&pvec) * inv_det;
    if u < 0.0 || u > 1.0 {
        return None;
    }

    let qvec = tvec.cross(&edge1);
    let v = dir.dot(&qvec) * inv_det;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }

    let t = edge2.dot(&qvec) * inv_det;

    if t > eps {
        Some((t, u, v))
    } else {
        None
    }
}

/// Find the closest point on a triangle to a given point.
///
/// Uses the algorithm from "Real-Time Collision Detection" by Christer Ericson.
pub fn closest_point_on_triangle(p: &Vec3, a: &Vec3, b: &Vec3, c: &Vec3) -> Vec3 {
    let ab = *b - *a;
    let ac = *c - *a;
    let ap = *p - *a;

    let d1 = ab.dot(&ap);
    let d2 = ac.dot(&ap);

    // Check if P in vertex region outside A
    if d1 <= 0.0 && d2 <= 0.0 {
        return *a;
    }

    // Check if P in vertex region outside B
    let bp = *p - *b;
    let d3 = ab.dot(&bp);
    let d4 = ac.dot(&bp);
    if d3 >= 0.0 && d4 <= d3 {
        return *b;
    }

    // Check if P in edge region of AB
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return *a + ab * v;
    }

    // Check if P in vertex region outside C
    let cp = *p - *c;
    let d5 = ab.dot(&cp);
    let d6 = ac.dot(&cp);
    if d6 >= 0.0 && d5 <= d6 {
        return *c;
    }

    // Check if P in edge region of AC
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return *a + ac * w;
    }

    // Check if P in edge region of BC
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return *b + (*c - *b) * w;
    }

    // P inside face region
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    *a + ab * v + ac * w
}

/// Indexed triangle set for AABB tree queries.
pub struct IndexedTriangleSet {
    /// Vertex positions.
    pub vertices: Vec<Vec3>,
    /// Triangle indices (3 indices per triangle).
    pub triangles: Vec<[usize; 3]>,
    /// AABB tree for spatial queries.
    pub tree: AABBTree,
}

impl IndexedTriangleSet {
    // Create a new indexed triangle set from vertices and triangles.
    pub fn new(vertices: Vec<Vec3>, triangles: Vec<[usize; 3]>) -> Self {
        Self::with_epsilon(vertices, triangles, 1e-6)
    }

    /// Create a new indexed triangle set with a custom epsilon.
    pub fn with_epsilon(vertices: Vec<Vec3>, triangles: Vec<[usize; 3]>, eps: CoordF) -> Self {
        let tree = AABBTree::build_from_triangles(&vertices, &triangles, eps);
        Self {
            vertices,
            triangles,
            tree,
        }
    }

    /// Check if the triangle set is empty.
    pub fn is_empty(&self) -> bool {
        self.triangles.is_empty()
    }

    /// Get the number of triangles.
    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    /// Get a triangle's vertices.
    pub fn triangle_vertices(&self, idx: usize) -> (&Vec3, &Vec3, &Vec3) {
        let tri = &self.triangles[idx];
        (
            &self.vertices[tri[0]],
            &self.vertices[tri[1]],
            &self.vertices[tri[2]],
        )
    }

    /// Cast a ray and find the first intersection.
    pub fn ray_cast_first(&self, origin: &Vec3, direction: &Vec3) -> Option<RayHit> {
        if self.tree.is_empty() {
            return None;
        }

        let inv_dir = Vec3::new(1.0 / direction.x, 1.0 / direction.y, 1.0 / direction.z);

        let eps = self.ray_epsilon();

        self.ray_cast_first_recursive(origin, direction, &inv_dir, 0, CoordF::MAX, eps)
    }

    /// Cast a ray and find all intersections.
    pub fn ray_cast_all(&self, origin: &Vec3, direction: &Vec3) -> Vec<RayHit> {
        if self.tree.is_empty() {
            return Vec::new();
        }

        let inv_dir = Vec3::new(1.0 / direction.x, 1.0 / direction.y, 1.0 / direction.z);

        let eps = self.ray_epsilon();
        let mut hits = Vec::new();

        self.ray_cast_all_recursive(origin, direction, &inv_dir, 0, eps, &mut hits);

        // Sort by distance
        hits.sort_by(|a, b| a.t.partial_cmp(&b.t).unwrap_or(std::cmp::Ordering::Equal));
        hits
    }

    /// Find the closest point on the mesh to a given point.
    pub fn closest_point(&self, point: &Vec3) -> Option<AABBClosestPointResult> {
        if self.tree.is_empty() {
            return None;
        }

        let mut result = AABBClosestPointResult::new(NPOS, Vec3::zero(), CoordF::MAX);
        self.closest_point_recursive(point, 0, 0.0, CoordF::MAX, &mut result);

        if result.primitive_idx != NPOS {
            Some(result)
        } else {
            None
        }
    }

    /// Find all triangles within a given distance from a point.
    pub fn triangles_within_distance(&self, point: &Vec3, distance: CoordF) -> Vec<usize> {
        if self.tree.is_empty() {
            return Vec::new();
        }

        let dist_sq = distance * distance;
        let mut found = Vec::new();

        self.triangles_within_distance_recursive(point, 0, dist_sq, &mut found);

        found
    }

    /// Check if any triangle is within a given distance from a point.
    pub fn any_triangle_in_radius(&self, point: &Vec3, radius: CoordF) -> bool {
        if self.tree.is_empty() {
            return false;
        }

        let radius_sq = radius * radius;
        self.any_triangle_in_radius_recursive(point, 0, radius_sq)
    }

    /// Calculate the epsilon for ray-triangle intersection based on mesh size.
    fn ray_epsilon(&self) -> CoordF {
        let mut eps = 1e-6;
        if let Some(root) = self.tree.root() {
            let diag = root.bbox.diagonal();
            let max_dim = diag.x.max(diag.y).max(diag.z);
            if max_dim > 0.0 {
                eps = 1e-6 / (max_dim * max_dim);
            }
        }
        eps
    }

    fn ray_cast_first_recursive(
        &self,
        origin: &Vec3,
        dir: &Vec3,
        inv_dir: &Vec3,
        node_idx: usize,
        mut min_t: CoordF,
        eps: CoordF,
    ) -> Option<RayHit> {
        let node = self.tree.node(node_idx)?;

        if !node.is_valid() {
            return None;
        }

        if !ray_box_intersect(origin, inv_dir, &node.bbox, 0.0, min_t) {
            return None;
        }

        if node.is_leaf() {
            let (v0, v1, v2) = self.triangle_vertices(node.idx);
            if let Some((t, u, v)) = ray_triangle_intersect(origin, dir, v0, v1, v2, eps) {
                if t > 0.0 && t < min_t {
                    return Some(RayHit::new(node.idx, t, u, v));
                }
            }
            return None;
        }

        let left_idx = AABBTree::left_child_idx(node_idx);
        let right_idx = AABBTree::right_child_idx(node_idx);

        let mut best_hit: Option<RayHit> = None;

        if let Some(left_hit) =
            self.ray_cast_first_recursive(origin, dir, inv_dir, left_idx, min_t, eps)
        {
            if left_hit.t < min_t {
                min_t = left_hit.t;
                best_hit = Some(left_hit);
            }
        }

        if let Some(right_hit) =
            self.ray_cast_first_recursive(origin, dir, inv_dir, right_idx, min_t, eps)
        {
            if right_hit.t < min_t {
                best_hit = Some(right_hit);
            }
        }

        best_hit
    }

    fn ray_cast_all_recursive(
        &self,
        origin: &Vec3,
        dir: &Vec3,
        inv_dir: &Vec3,
        node_idx: usize,
        eps: CoordF,
        hits: &mut Vec<RayHit>,
    ) {
        let node = match self.tree.node(node_idx) {
            Some(n) if n.is_valid() => n,
            _ => return,
        };

        if !ray_box_intersect(origin, inv_dir, &node.bbox, 0.0, CoordF::MAX) {
            return;
        }

        if node.is_leaf() {
            let (v0, v1, v2) = self.triangle_vertices(node.idx);
            if let Some((t, u, v)) = ray_triangle_intersect(origin, dir, v0, v1, v2, eps) {
                if t > 0.0 {
                    hits.push(RayHit::new(node.idx, t, u, v));
                }
            }
            return;
        }

        let left_idx = AABBTree::left_child_idx(node_idx);
        let right_idx = AABBTree::right_child_idx(node_idx);

        self.ray_cast_all_recursive(origin, dir, inv_dir, left_idx, eps, hits);
        self.ray_cast_all_recursive(origin, dir, inv_dir, right_idx, eps, hits);
    }

    fn closest_point_recursive(
        &self,
        point: &Vec3,
        node_idx: usize,
        low_sqr_d: CoordF,
        mut up_sqr_d: CoordF,
        result: &mut AABBClosestPointResult,
    ) -> CoordF {
        if low_sqr_d > up_sqr_d {
            return low_sqr_d;
        }

        let node = match self.tree.node(node_idx) {
            Some(n) if n.is_valid() => n,
            _ => return up_sqr_d,
        };

        if node.is_leaf() {
            let (v0, v1, v2) = self.triangle_vertices(node.idx);
            let closest = closest_point_on_triangle(point, v0, v1, v2);
            let sqr_dist = (*point - closest).length_squared();

            if sqr_dist < up_sqr_d {
                result.primitive_idx = node.idx;
                result.point = closest;
                result.squared_distance = sqr_dist;
                up_sqr_d = sqr_dist;
            }
        } else {
            let left_idx = AABBTree::left_child_idx(node_idx);
            let right_idx = AABBTree::right_child_idx(node_idx);

            let left_node = self.tree.node(left_idx);
            let right_node = self.tree.node(right_idx);

            let left_dist = left_node
                .map(|n| n.bbox.squared_exterior_distance(point))
                .unwrap_or(CoordF::MAX);
            let right_dist = right_node
                .map(|n| n.bbox.squared_exterior_distance(point))
                .unwrap_or(CoordF::MAX);

            // Visit closer child first
            if left_dist < right_dist {
                if left_dist < up_sqr_d {
                    up_sqr_d =
                        self.closest_point_recursive(point, left_idx, low_sqr_d, up_sqr_d, result);
                }
                if right_dist < up_sqr_d {
                    up_sqr_d =
                        self.closest_point_recursive(point, right_idx, low_sqr_d, up_sqr_d, result);
                }
            } else {
                if right_dist < up_sqr_d {
                    up_sqr_d =
                        self.closest_point_recursive(point, right_idx, low_sqr_d, up_sqr_d, result);
                }
                if left_dist < up_sqr_d {
                    up_sqr_d =
                        self.closest_point_recursive(point, left_idx, low_sqr_d, up_sqr_d, result);
                }
            }
        }

        up_sqr_d
    }

    fn triangles_within_distance_recursive(
        &self,
        point: &Vec3,
        node_idx: usize,
        dist_sq_limit: CoordF,
        found: &mut Vec<usize>,
    ) {
        let node = match self.tree.node(node_idx) {
            Some(n) if n.is_valid() => n,
            _ => return,
        };

        if node.is_leaf() {
            let (v0, v1, v2) = self.triangle_vertices(node.idx);
            let closest = closest_point_on_triangle(point, v0, v1, v2);
            let sqr_dist = (*point - closest).length_squared();

            if sqr_dist < dist_sq_limit {
                found.push(node.idx);
            }
        } else {
            let left_idx = AABBTree::left_child_idx(node_idx);
            let right_idx = AABBTree::right_child_idx(node_idx);

            if let Some(left_node) = self.tree.node(left_idx) {
                if left_node.is_valid()
                    && left_node.bbox.squared_exterior_distance(point) < dist_sq_limit
                {
                    self.triangles_within_distance_recursive(point, left_idx, dist_sq_limit, found);
                }
            }

            if let Some(right_node) = self.tree.node(right_idx) {
                if right_node.is_valid()
                    && right_node.bbox.squared_exterior_distance(point) < dist_sq_limit
                {
                    self.triangles_within_distance_recursive(
                        point,
                        right_idx,
                        dist_sq_limit,
                        found,
                    );
                }
            }
        }
    }

    fn any_triangle_in_radius_recursive(
        &self,
        point: &Vec3,
        node_idx: usize,
        radius_sq: CoordF,
    ) -> bool {
        let node = match self.tree.node(node_idx) {
            Some(n) if n.is_valid() => n,
            _ => return false,
        };

        if node.bbox.squared_exterior_distance(point) >= radius_sq {
            return false;
        }

        if node.is_leaf() {
            let (v0, v1, v2) = self.triangle_vertices(node.idx);
            let closest = closest_point_on_triangle(point, v0, v1, v2);
            let sqr_dist = (*point - closest).length_squared();
            return sqr_dist < radius_sq;
        }

        let left_idx = AABBTree::left_child_idx(node_idx);
        let right_idx = AABBTree::right_child_idx(node_idx);

        self.any_triangle_in_radius_recursive(point, left_idx, radius_sq)
            || self.any_triangle_in_radius_recursive(point, right_idx, radius_sq)
    }
}

/// Calculate the next power of 2 >= n.
fn next_power_of_2(n: usize) -> usize {
    if n == 0 {
        return 1;
    }
    let mut v = n - 1;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v |= v >> 32;
    v + 1
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_test_cube() -> (Vec<Vec3>, Vec<[usize; 3]>) {
        // Simple cube from 0 to 1
        let vertices = vec![
            Vec3::new(0.0, 0.0, 0.0), // 0
            Vec3::new(1.0, 0.0, 0.0), // 1
            Vec3::new(1.0, 1.0, 0.0), // 2
            Vec3::new(0.0, 1.0, 0.0), // 3
            Vec3::new(0.0, 0.0, 1.0), // 4
            Vec3::new(1.0, 0.0, 1.0), // 5
            Vec3::new(1.0, 1.0, 1.0), // 6
            Vec3::new(0.0, 1.0, 1.0), // 7
        ];

        let triangles = vec![
            // Bottom (z=0)
            [0, 2, 1],
            [0, 3, 2],
            // Top (z=1)
            [4, 5, 6],
            [4, 6, 7],
            // Front (y=0)
            [0, 1, 5],
            [0, 5, 4],
            // Back (y=1)
            [2, 3, 7],
            [2, 7, 6],
            // Left (x=0)
            [0, 4, 7],
            [0, 7, 3],
            // Right (x=1)
            [1, 2, 6],
            [1, 6, 5],
        ];

        (vertices, triangles)
    }

    #[test]
    fn test_vec3_operations() {
        let a = Vec3::new(1.0, 2.0, 3.0);
        let b = Vec3::new(4.0, 5.0, 6.0);

        let sum = a + b;
        assert!((sum.x - 5.0).abs() < 1e-10);
        assert!((sum.y - 7.0).abs() < 1e-10);
        assert!((sum.z - 9.0).abs() < 1e-10);

        let diff = b - a;
        assert!((diff.x - 3.0).abs() < 1e-10);
        assert!((diff.y - 3.0).abs() < 1e-10);
        assert!((diff.z - 3.0).abs() < 1e-10);

        let dot = a.dot(&b);
        assert!((dot - 32.0).abs() < 1e-10);

        let cross = a.cross(&b);
        assert!((cross.x - (-3.0)).abs() < 1e-10);
        assert!((cross.y - 6.0).abs() < 1e-10);
        assert!((cross.z - (-3.0)).abs() < 1e-10);
    }

    #[test]
    fn test_aabb3_operations() {
        let mut bbox = AABB3::from_point(Vec3::zero());
        bbox.extend_point(&Vec3::new(1.0, 2.0, 3.0));

        assert!((bbox.min.x - 0.0).abs() < 1e-10);
        assert!((bbox.max.z - 3.0).abs() < 1e-10);

        let center = bbox.center();
        assert!((center.x - 0.5).abs() < 1e-10);
        assert!((center.y - 1.0).abs() < 1e-10);
        assert!((center.z - 1.5).abs() < 1e-10);

        assert!(bbox.contains(&Vec3::new(0.5, 1.0, 1.5)));
        assert!(!bbox.contains(&Vec3::new(2.0, 1.0, 1.5)));
    }

    #[test]
    fn test_aabb3_squared_exterior_distance() {
        let bbox = AABB3::new(Vec3::zero(), Vec3::new(1.0, 1.0, 1.0));

        // Point inside
        let dist = bbox.squared_exterior_distance(&Vec3::new(0.5, 0.5, 0.5));
        assert!((dist - 0.0).abs() < 1e-10);

        // Point outside on X axis
        let dist = bbox.squared_exterior_distance(&Vec3::new(2.0, 0.5, 0.5));
        assert!((dist - 1.0).abs() < 1e-10);

        // Point outside on corner
        let dist = bbox.squared_exterior_distance(&Vec3::new(2.0, 2.0, 2.0));
        assert!((dist - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_build_tree() {
        let (vertices, triangles) = make_test_cube();
        let tree = AABBTree::build_from_triangles(&vertices, &triangles, 0.0);

        assert!(!tree.is_empty());
        assert!(tree.root().is_some());

        // Root should contain all triangles
        let root = tree.root().unwrap();
        assert!(root.is_inner());
        assert!(root.bbox.contains(&Vec3::new(0.5, 0.5, 0.5)));
    }

    #[test]
    fn test_ray_triangle_intersect() {
        let v0 = Vec3::new(0.0, 0.0, 0.0);
        let v1 = Vec3::new(1.0, 0.0, 0.0);
        let v2 = Vec3::new(0.5, 1.0, 0.0);

        // Ray hitting the triangle
        let origin = Vec3::new(0.5, 0.5, 1.0);
        let dir = Vec3::new(0.0, 0.0, -1.0);
        let hit = ray_triangle_intersect(&origin, &dir, &v0, &v1, &v2, 1e-6);
        assert!(hit.is_some());
        let (t, _u, _v) = hit.unwrap();
        assert!((t - 1.0).abs() < 1e-6);

        // Ray missing the triangle
        let origin2 = Vec3::new(5.0, 5.0, 1.0);
        let hit2 = ray_triangle_intersect(&origin2, &dir, &v0, &v1, &v2, 1e-6);
        assert!(hit2.is_none());
    }

    #[test]
    fn test_closest_point_on_triangle() {
        let a = Vec3::new(0.0, 0.0, 0.0);
        let b = Vec3::new(1.0, 0.0, 0.0);
        let c = Vec3::new(0.0, 1.0, 0.0);

        // Point above triangle center
        let p = Vec3::new(0.25, 0.25, 1.0);
        let closest = closest_point_on_triangle(&p, &a, &b, &c);
        assert!((closest.x - 0.25).abs() < 1e-6);
        assert!((closest.y - 0.25).abs() < 1e-6);
        assert!((closest.z - 0.0).abs() < 1e-6);

        // Point near vertex A
        let p2 = Vec3::new(-1.0, -1.0, 0.0);
        let closest2 = closest_point_on_triangle(&p2, &a, &b, &c);
        assert!((closest2.x - 0.0).abs() < 1e-6);
        assert!((closest2.y - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_indexed_triangle_set_ray_cast() {
        let (vertices, triangles) = make_test_cube();
        let mesh = IndexedTriangleSet::new(vertices, triangles);

        // Ray from outside hitting the cube
        let origin = Vec3::new(0.5, 0.5, 2.0);
        let dir = Vec3::new(0.0, 0.0, -1.0);
        let hit = mesh.ray_cast_first(&origin, &dir);

        assert!(hit.is_some());
        let hit = hit.unwrap();
        assert!((hit.t - 1.0).abs() < 1e-5); // Should hit top face at z=1

        // Ray missing the cube
        let origin2 = Vec3::new(5.0, 5.0, 2.0);
        let hit2 = mesh.ray_cast_first(&origin2, &dir);
        assert!(hit2.is_none());
    }

    #[test]
    fn test_indexed_triangle_set_closest_point() {
        let (vertices, triangles) = make_test_cube();
        let mesh = IndexedTriangleSet::new(vertices, triangles);

        // Point inside the cube
        let p = Vec3::new(0.5, 0.5, 0.5);
        let result = mesh.closest_point(&p);
        assert!(result.is_some());
        let result = result.unwrap();
        assert!(result.distance() < 0.6); // Should be close to a face

        // Point outside the cube
        let p2 = Vec3::new(2.0, 0.5, 0.5);
        let result2 = mesh.closest_point(&p2);
        assert!(result2.is_some());
        let result2 = result2.unwrap();
        assert!((result2.distance() - 1.0).abs() < 1e-5);
    }

    #[test]
    fn test_triangles_within_distance() {
        let (vertices, triangles) = make_test_cube();
        let mesh = IndexedTriangleSet::new(vertices, triangles);

        // Point on the surface of the cube
        let p = Vec3::new(0.5, 0.5, 1.0);
        let found = mesh.triangles_within_distance(&p, 0.1);

        // Should find the top face triangles
        assert!(!found.is_empty());
    }

    #[test]
    fn test_any_triangle_in_radius() {
        let (vertices, triangles) = make_test_cube();
        let mesh = IndexedTriangleSet::new(vertices, triangles);

        // Point close to cube
        assert!(mesh.any_triangle_in_radius(&Vec3::new(0.5, 0.5, 1.0), 0.1));

        // Point far from cube
        assert!(!mesh.any_triangle_in_radius(&Vec3::new(10.0, 10.0, 10.0), 1.0));
    }

    #[test]
    fn test_next_power_of_2() {
        assert_eq!(next_power_of_2(0), 1);
        assert_eq!(next_power_of_2(1), 1);
        assert_eq!(next_power_of_2(2), 2);
        assert_eq!(next_power_of_2(3), 4);
        assert_eq!(next_power_of_2(5), 8);
        assert_eq!(next_power_of_2(16), 16);
        assert_eq!(next_power_of_2(17), 32);
    }

    #[test]
    fn test_ray_box_intersect() {
        let bbox = AABB3::new(Vec3::zero(), Vec3::new(1.0, 1.0, 1.0));

        // Ray hitting the box
        let origin = Vec3::new(0.5, 0.5, 2.0);
        let dir = Vec3::new(0.0, 0.0, -1.0);
        let inv_dir = Vec3::new(1.0 / dir.x, 1.0 / dir.y, 1.0 / dir.z);
        assert!(ray_box_intersect(
            &origin,
            &inv_dir,
            &bbox,
            0.0,
            CoordF::MAX
        ));

        // Ray missing the box
        let origin2 = Vec3::new(5.0, 5.0, 2.0);
        assert!(!ray_box_intersect(
            &origin2,
            &inv_dir,
            &bbox,
            0.0,
            CoordF::MAX
        ));
    }

    #[test]
    fn test_ray_cast_all() {
        let (vertices, triangles) = make_test_cube();
        let mesh = IndexedTriangleSet::new(vertices, triangles);

        // Ray going through the cube should hit faces
        let origin = Vec3::new(0.5, 0.5, 2.0);
        let dir = Vec3::new(0.0, 0.0, -1.0);
        let hits = mesh.ray_cast_all(&origin, &dir);

        // Should hit top and bottom faces (may hit multiple triangles per face
        // if ray goes through triangle edges)
        assert!(hits.len() >= 2);
        assert!(hits[0].t < hits[hits.len() - 1].t); // Sorted by distance

        // First hit should be around z=1 (top face)
        assert!((hits[0].t - 1.0).abs() < 0.1);
    }
}
