//! AABBMesh - Axis-Aligned Bounding Box accelerated mesh queries.
//!
//! This module provides AABBMesh for efficient ray casting and distance queries
//! on triangle meshes, mirroring BambuStudio's AABBMesh class.
//!
//! # Features
//!
//! - Ray casting (first hit, all hits)
//! - Closest point queries
//! - Distance calculations
//! - Normal extraction
//!
//! # BambuStudio Reference
//!
//! - `src/libslic3r/AABBMesh.hpp`
//! - `src/libslic3r/AABBMesh.cpp`

use crate::geometry::{Point3F, Vec3};
use crate::triangle_mesh::TriangleMesh;
use crate::CoordF;

/// Result of a ray-mesh intersection query.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HitResult {
    /// Index of the triangle that was hit.
    pub triangle_index: usize,
    /// Distance along the ray to the hit point.
    pub distance: CoordF,
    /// Barycentric coordinate u.
    pub u: CoordF,
    /// Barycentric coordinate v.
    pub v: CoordF,
    /// Point of intersection in world space.
    pub point: Point3F,
}

/// AABB-accelerated mesh for efficient spatial queries.
///
/// This struct wraps a TriangleMesh with an AABB tree for fast ray casting
/// and closest-point queries.
pub struct AABBMesh {
    vertices: Vec<Point3F>,
    indices: Vec<[u32; 3]>,
    tree: AABBTree,
    epsilon: CoordF,
}

/// AABB tree node for spatial acceleration.
#[derive(Debug, Clone)]
struct AABBNode {
    /// Bounding box minimum corner.
    min: Point3F,
    /// Bounding box maximum corner.
    max: Point3F,
    /// Left child index (or triangle index if leaf).
    left: usize,
    /// Right child index (or sentinel if leaf).
    right: usize,
    /// Triangle index for leaf nodes.
    triangle_index: Option<usize>,
}

/// AABB tree for fast mesh queries.
#[derive(Debug, Clone)]
struct AABBTree {
    nodes: Vec<AABBNode>,
    root: usize,
}

impl AABBMesh {
    // Create an AABBMesh from a TriangleMesh.
    //
    // # Arguments
    //
    // * `mesh` - The input triangle mesh.
    // * `calculate_epsilon` - Whether to auto-calculate epsilon value.
    pub fn from_mesh(mesh: &TriangleMesh, calculate_epsilon: bool) -> Self {
        let vertices: Vec<Point3F> = mesh.vertices().to_vec();
        let indices: Vec<[u32; 3]> = mesh.indices().iter().map(|tri| tri.indices).collect();

        let tree = AABBTree::build(&vertices, &indices);

        let epsilon = if calculate_epsilon {
            Self::calculate_epsilon(&vertices)
        } else {
            1e-6
        };

        Self {
            vertices,
            indices,
            tree,
            epsilon,
        }
    }

    /// Calculate default epsilon from mesh bounding box.
    fn calculate_epsilon(vertices: &[Point3F]) -> CoordF {
        if vertices.is_empty() {
            return 1e-6;
        }

        let mut min = vertices[0];
        let mut max = vertices[0];

        for v in &vertices[1..] {
            min.x = min.x.min(v.x);
            min.y = min.y.min(v.y);
            min.z = min.z.min(v.z);
            max.x = max.x.max(v.x);
            max.y = max.y.max(v.y);
            max.z = max.z.max(v.z);
        }

        let size = Point3F::new(max.x - min.x, max.y - min.y, max.z - min.z);
        let max_size = size.x.max(size.y).max(size.z);

        max_size * 1e-6
    }

    /// Get the number of triangles.
    pub fn triangle_count(&self) -> usize {
        self.indices.len()
    }

    /// Get the number of vertices.
    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    /// Get a vertex by index.
    pub fn vertex(&self, idx: usize) -> Option<Point3F> {
        self.vertices.get(idx).copied()
    }

    /// Get triangle indices by triangle index.
    pub fn triangle(&self, idx: usize) -> Option<[u32; 3]> {
        self.indices.get(idx).copied()
    }

    /// Get triangle vertices by triangle index.
    pub fn triangle_vertices(&self, idx: usize) -> Option<[Point3F; 3]> {
        let indices = self.triangle(idx)?;
        Some([
            self.vertices.get(indices[0] as usize).copied()?,
            self.vertices.get(indices[1] as usize).copied()?,
            self.vertices.get(indices[2] as usize).copied()?,
        ])
    }

    /// Calculate the normal of a triangle.
    pub fn triangle_normal(&self, idx: usize) -> Option<Vec3> {
        let [v0, v1, v2] = self.triangle_vertices(idx)?;

        let e1 = Vec3::new(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
        let e2 = Vec3::new(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);

        let normal = Vec3::new(
            e1.y * e2.z - e1.z * e2.y,
            e1.z * e2.x - e1.x * e2.z,
            e1.x * e2.y - e1.y * e2.x,
        );

        let len = (normal.x * normal.x + normal.y * normal.y + normal.z * normal.z).sqrt();
        if len > 0.0 {
            Some(Vec3::new(normal.x / len, normal.y / len, normal.z / len))
        } else {
            None
        }
    }

    /// Cast a ray and find the first intersection.
    ///
    /// # Arguments
    ///
    /// * `origin` - Ray origin point.
    /// * `direction` - Ray direction vector (should be normalized).
    ///
    /// # Returns
    ///
    /// The closest hit result, or None if no intersection.
    pub fn query_ray_hit(&self, origin: &Point3F, direction: &Vec3) -> Option<HitResult> {
        self.tree.query_ray_hit(
            &self.vertices,
            &self.indices,
            origin,
            direction,
            self.epsilon,
        )
    }

    /// Cast a ray and find all intersections.
    ///
    /// # Arguments
    ///
    /// * `origin` - Ray origin point.
    /// * `direction` - Ray direction vector (should be normalized).
    ///
    /// # Returns
    ///
    /// A vector of all hit results, sorted by distance.
    pub fn query_ray_hits(&self, origin: &Point3F, direction: &Vec3) -> Vec<HitResult> {
        self.tree.query_ray_hits(
            &self.vertices,
            &self.indices,
            origin,
            direction,
            self.epsilon,
        )
    }

    /// Find the closest point on the mesh to a query point.
    ///
    /// # Arguments
    ///
    /// * `point` - The query point.
    ///
    /// # Returns
    ///
    /// Tuple of (closest_point, triangle_index, squared_distance).
    pub fn closest_point(&self, point: &Point3F) -> Option<(Point3F, usize, CoordF)> {
        self.tree
            .closest_point(&self.vertices, &self.indices, point, self.epsilon)
    }

    /// Calculate the squared distance from a point to the mesh.
    ///
    /// # Arguments
    ///
    /// * `point` - The query point.
    ///
    /// # Returns
    ///
    /// The squared distance to the closest point on the mesh.
    pub fn squared_distance(&self, point: &Point3F) -> CoordF {
        self.closest_point(point)
            .map(|(_, _, dist_sq)| dist_sq)
            .unwrap_or(f64::INFINITY)
    }

    /// Check if a point is inside the mesh (using ray casting parity).
    ///
    /// # Arguments
    ///
    /// * `point` - The point to check.
    ///
    /// # Returns
    ///
    /// true if the point is inside the mesh, false otherwise.
    pub fn is_point_inside(&self, point: &Point3F) -> bool {
        // Cast a ray in +X direction and count intersections
        let direction = Vec3::new(1.0, 0.0, 0.0);
        let hits = self.query_ray_hits(point, &direction);

        // Odd number of hits means inside
        hits.len() % 2 == 1
    }
}

impl AABBTree {
    // Build an AABB tree from vertices and triangle indices.
    fn build(vertices: &[Point3F], indices: &[[u32; 3]]) -> Self {
        if indices.is_empty() {
            return Self {
                nodes: vec![AABBNode::empty()],
                root: 0,
            };
        }

        // Create initial triangle bounds
        let mut tri_bounds: Vec<(usize, Point3F, Point3F)> = indices
            .iter()
            .enumerate()
            .map(|(idx, tri)| {
                let v0 = vertices[tri[0] as usize];
                let v1 = vertices[tri[1] as usize];
                let v2 = vertices[tri[2] as usize];

                let min = Point3F::new(
                    v0.x.min(v1.x).min(v2.x),
                    v0.y.min(v1.y).min(v2.y),
                    v0.z.min(v1.z).min(v2.z),
                );
                let max = Point3F::new(
                    v0.x.max(v1.x).max(v2.x),
                    v0.y.max(v1.y).max(v2.y),
                    v0.z.max(v1.z).max(v2.z),
                );

                (idx, min, max)
            })
            .collect();

        let mut nodes = Vec::new();
        let tb_len = tri_bounds.len();
        let root = Self::build_recursive(&mut nodes, &mut tri_bounds, 0, tb_len);

        Self { nodes, root }
    }

    /// Recursively build the tree.
    fn build_recursive(
        nodes: &mut Vec<AABBNode>,
        tri_bounds: &mut [(usize, Point3F, Point3F)],
        start: usize,
        end: usize,
    ) -> usize {
        let count = end - start;

        if count == 0 {
            // Empty node
            nodes.push(AABBNode::empty());
            return nodes.len() - 1;
        }

        if count == 1 {
            // Leaf node
            let (tri_idx, min, max) = tri_bounds[start];
            nodes.push(AABBNode::leaf(tri_idx, min, max));
            return nodes.len() - 1;
        }

        // Find the bounds of all triangles in this range
        let mut overall_min = tri_bounds[start].1;
        let mut overall_max = tri_bounds[start].2;

        for i in (start + 1)..end {
            overall_min.x = overall_min.x.min(tri_bounds[i].1.x);
            overall_min.y = overall_min.y.min(tri_bounds[i].1.y);
            overall_min.z = overall_min.z.min(tri_bounds[i].1.z);
            overall_max.x = overall_max.x.max(tri_bounds[i].2.x);
            overall_max.y = overall_max.y.max(tri_bounds[i].2.y);
            overall_max.z = overall_max.z.max(tri_bounds[i].2.z);
        }

        // Find the longest axis
        let size_x = overall_max.x - overall_min.x;
        let size_y = overall_max.y - overall_min.y;
        let size_z = overall_max.z - overall_min.z;

        let axis = if size_x >= size_y && size_x >= size_z {
            0 // X
        } else if size_y >= size_z {
            1 // Y
        } else {
            2 // Z
        };

        // Sort triangles by their centroid on the chosen axis
        tri_bounds[start..end].sort_by(|a, b| {
            let centroid_a = (a.1.get_axis(axis) + a.2.get_axis(axis)) * 0.5;
            let centroid_b = (b.1.get_axis(axis) + b.2.get_axis(axis)) * 0.5;
            centroid_a.partial_cmp(&centroid_b).unwrap()
        });

        // Split at median
        let mid = start + count / 2;

        // Recursively build children
        let left = Self::build_recursive(nodes, tri_bounds, start, mid);
        let right = Self::build_recursive(nodes, tri_bounds, mid, end);

        nodes.push(AABBNode::internal(left, right, overall_min, overall_max));
        nodes.len() - 1
    }

    /// Query ray for first hit.
    fn query_ray_hit(
        &self,
        vertices: &[Point3F],
        indices: &[[u32; 3]],
        origin: &Point3F,
        direction: &Vec3,
        epsilon: CoordF,
    ) -> Option<HitResult> {
        self.query_ray_hit_recursive(
            self.root,
            vertices,
            indices,
            origin,
            direction,
            epsilon,
            f64::INFINITY,
        )
    }

    fn query_ray_hit_recursive(
        &self,
        node_idx: usize,
        vertices: &[Point3F],
        indices: &[[u32; 3]],
        origin: &Point3F,
        direction: &Vec3,
        epsilon: CoordF,
        current_best: CoordF,
    ) -> Option<HitResult> {
        let node = self.nodes.get(node_idx)?;

        // Check if ray hits this node's AABB
        if !ray_aabb_intersect(
            origin,
            direction,
            &node.min,
            &node.max,
            epsilon,
            current_best,
        ) {
            return None;
        }

        if let Some(tri_idx) = node.triangle_index {
            // Leaf node - test triangle
            let tri = indices.get(tri_idx)?;
            let v0 = vertices.get(tri[0] as usize)?;
            let v1 = vertices.get(tri[1] as usize)?;
            let v2 = vertices.get(tri[2] as usize)?;

            return ray_triangle_intersect(origin, direction, v0, v1, v2, epsilon).map(
                |(t, u, v, point)| HitResult {
                    triangle_index: tri_idx,
                    distance: t,
                    u,
                    v,
                    point,
                },
            );
        }

        // Internal node - query children
        let left_hit = self.query_ray_hit_recursive(
            node.left,
            vertices,
            indices,
            origin,
            direction,
            epsilon,
            current_best,
        );

        let best_t = left_hit.as_ref().map_or(current_best, |h| h.distance);

        let right_hit = self.query_ray_hit_recursive(
            node.right, vertices, indices, origin, direction, epsilon, best_t,
        );

        // Return closer hit
        match (left_hit, right_hit) {
            (Some(l), Some(r)) => {
                if l.distance < r.distance {
                    Some(l)
                } else {
                    Some(r)
                }
            }
            (Some(l), None) => Some(l),
            (None, Some(r)) => Some(r),
            (None, None) => None,
        }
    }

    /// Query ray for all hits.
    fn query_ray_hits(
        &self,
        vertices: &[Point3F],
        indices: &[[u32; 3]],
        origin: &Point3F,
        direction: &Vec3,
        epsilon: CoordF,
    ) -> Vec<HitResult> {
        let mut hits = Vec::new();
        self.query_ray_hits_recursive(
            self.root, vertices, indices, origin, direction, epsilon, &mut hits,
        );

        // Sort by distance
        hits.sort_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap());
        hits
    }

    fn query_ray_hits_recursive(
        &self,
        node_idx: usize,
        vertices: &[Point3F],
        indices: &[[u32; 3]],
        origin: &Point3F,
        direction: &Vec3,
        epsilon: CoordF,
        hits: &mut Vec<HitResult>,
    ) {
        let Some(node) = self.nodes.get(node_idx) else {
            return;
        };

        // Check if ray hits this node's AABB
        if !ray_aabb_intersect(
            origin,
            direction,
            &node.min,
            &node.max,
            epsilon,
            f64::INFINITY,
        ) {
            return;
        }

        if let Some(tri_idx) = node.triangle_index {
            // Leaf node - test triangle
            if let Some(tri) = indices.get(tri_idx) {
                if let (Some(v0), Some(v1), Some(v2)) = (
                    vertices.get(tri[0] as usize),
                    vertices.get(tri[1] as usize),
                    vertices.get(tri[2] as usize),
                ) {
                    if let Some((t, u, v, point)) =
                        ray_triangle_intersect(origin, direction, v0, v1, v2, epsilon)
                    {
                        hits.push(HitResult {
                            triangle_index: tri_idx,
                            distance: t,
                            u,
                            v,
                            point,
                        });
                    }
                }
            }
            return;
        }

        // Internal node - query children
        self.query_ray_hits_recursive(
            node.left, vertices, indices, origin, direction, epsilon, hits,
        );
        self.query_ray_hits_recursive(
            node.right, vertices, indices, origin, direction, epsilon, hits,
        );
    }

    /// Find closest point on mesh.
    fn closest_point(
        &self,
        vertices: &[Point3F],
        indices: &[[u32; 3]],
        point: &Point3F,
        _epsilon: CoordF,
    ) -> Option<(Point3F, usize, CoordF)> {
        self.closest_point_recursive(self.root, vertices, indices, point, f64::INFINITY)
    }

    fn closest_point_recursive(
        &self,
        node_idx: usize,
        vertices: &[Point3F],
        indices: &[[u32; 3]],
        point: &Point3F,
        current_best: CoordF,
    ) -> Option<(Point3F, usize, CoordF)> {
        let node = self.nodes.get(node_idx)?;

        // Check if this node can possibly contain a closer point
        let dist_sq = point_aabb_distance_sq(point, &node.min, &node.max);
        if dist_sq >= current_best {
            return None;
        }

        if let Some(tri_idx) = node.triangle_index {
            // Leaf node - test triangle
            let tri = indices.get(tri_idx)?;
            let v0 = vertices.get(tri[0] as usize)?;
            let v1 = vertices.get(tri[1] as usize)?;
            let v2 = vertices.get(tri[2] as usize)?;

            let closest = closest_point_on_triangle(point, v0, v1, v2);
            let dist_sq = point_distance_sq(point, &closest);

            return Some((closest, tri_idx, dist_sq));
        }

        // Internal node - query children (closest first)
        let left_dist = point_aabb_distance_sq(
            point,
            &self.nodes[node.left].min,
            &self.nodes[node.left].max,
        );
        let right_dist = point_aabb_distance_sq(
            point,
            &self.nodes[node.right].min,
            &self.nodes[node.right].max,
        );

        let (first, second) = if left_dist < right_dist {
            (node.left, node.right)
        } else {
            (node.right, node.left)
        };

        let first_result =
            self.closest_point_recursive(first, vertices, indices, point, current_best);
        let best = first_result.as_ref().map_or(current_best, |(_, _, d)| *d);

        let second_result = self.closest_point_recursive(second, vertices, indices, point, best);

        // Return best result
        match (first_result, second_result) {
            (Some(f), Some(s)) => {
                if f.2 < s.2 {
                    Some(f)
                } else {
                    Some(s)
                }
            }
            (Some(f), None) => Some(f),
            (None, Some(s)) => Some(s),
            (None, None) => None,
        }
    }
}

impl AABBNode {
    fn empty() -> Self {
        Self {
            min: Point3F::new(0.0, 0.0, 0.0),
            max: Point3F::new(0.0, 0.0, 0.0),
            left: 0,
            right: 0,
            triangle_index: None,
        }
    }

    fn leaf(tri_idx: usize, min: Point3F, max: Point3F) -> Self {
        Self {
            min,
            max,
            left: 0,
            right: 0,
            triangle_index: Some(tri_idx),
        }
    }

    fn internal(left: usize, right: usize, min: Point3F, max: Point3F) -> Self {
        Self {
            min,
            max,
            left,
            right,
            triangle_index: None,
        }
    }
}

impl Point3F {
    fn get_axis(&self, axis: usize) -> CoordF {
        match axis {
            0 => self.x,
            1 => self.y,
            2 => self.z,
            _ => 0.0,
        }
    }
}

/// Ray-AABB intersection test (slab method).
fn ray_aabb_intersect(
    origin: &Point3F,
    dir: &Vec3,
    min: &Point3F,
    max: &Point3F,
    t_min: CoordF,
    t_max: CoordF,
) -> bool {
    let inv_dir = Vec3::new(1.0 / dir.x, 1.0 / dir.y, 1.0 / dir.z);

    let t1 = (min.x - origin.x) * inv_dir.x;
    let t2 = (max.x - origin.x) * inv_dir.x;
    let t3 = (min.y - origin.y) * inv_dir.y;
    let t4 = (max.y - origin.y) * inv_dir.y;
    let t5 = (min.z - origin.z) * inv_dir.z;
    let t6 = (max.z - origin.z) * inv_dir.z;

    let t_min = t_min.max(t1.min(t2).max(t3.min(t4)).max(t5.min(t6)));
    let t_max = t_max.min(t1.max(t2).min(t3.max(t4)).min(t5.max(t6)));

    t_max > t_min && t_max > 0.0
}

/// Ray-triangle intersection (Möller–Trumbore algorithm).
fn ray_triangle_intersect(
    origin: &Point3F,
    dir: &Vec3,
    v0: &Point3F,
    v1: &Point3F,
    v2: &Point3F,
    epsilon: CoordF,
) -> Option<(CoordF, CoordF, CoordF, Point3F)> {
    let edge1 = Vec3::new(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
    let edge2 = Vec3::new(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);

    let h = Vec3::new(
        dir.y * edge2.z - dir.z * edge2.y,
        dir.z * edge2.x - dir.x * edge2.z,
        dir.x * edge2.y - dir.y * edge2.x,
    );

    let a = edge1.x * h.x + edge1.y * h.y + edge1.z * h.z;

    if a > -epsilon && a < epsilon {
        return None; // Ray parallel to triangle
    }

    let f = 1.0 / a;
    let s = Vec3::new(origin.x - v0.x, origin.y - v0.y, origin.z - v0.z);

    let u = f * (s.x * h.x + s.y * h.y + s.z * h.z);
    if u < 0.0 || u > 1.0 {
        return None;
    }

    let q = Vec3::new(
        s.y * edge2.z - s.z * edge2.y,
        s.z * edge2.x - s.x * edge2.z,
        s.x * edge2.y - s.y * edge2.x,
    );

    let v = f * (dir.x * q.x + dir.y * q.y + dir.z * q.z);
    if v < 0.0 || u + v > 1.0 {
        return None;
    }

    let t = f * (edge2.x * q.x + edge2.y * q.y + edge2.z * q.z);

    if t > epsilon {
        let point = Point3F::new(
            origin.x + t * dir.x,
            origin.y + t * dir.y,
            origin.z + t * dir.z,
        );
        Some((t, u, v, point))
    } else {
        None
    }
}

/// Calculate squared distance from point to AABB.
fn point_aabb_distance_sq(point: &Point3F, min: &Point3F, max: &Point3F) -> CoordF {
    let dx = if point.x < min.x {
        min.x - point.x
    } else if point.x > max.x {
        point.x - max.x
    } else {
        0.0
    };

    let dy = if point.y < min.y {
        min.y - point.y
    } else if point.y > max.y {
        point.y - max.y
    } else {
        0.0
    };

    let dz = if point.z < min.z {
        min.z - point.z
    } else if point.z > max.z {
        point.z - max.z
    } else {
        0.0
    };

    dx * dx + dy * dy + dz * dz
}

/// Calculate squared distance between two points.
fn point_distance_sq(a: &Point3F, b: &Point3F) -> CoordF {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    let dz = a.z - b.z;
    dx * dx + dy * dy + dz * dz
}

/// Find closest point on triangle to query point.
fn closest_point_on_triangle(point: &Point3F, a: &Point3F, b: &Point3F, c: &Point3F) -> Point3F {
    // From Real-Time Collision Detection by Christer Ericson
    let ab = Vec3::new(b.x - a.x, b.y - a.y, b.z - a.z);
    let ac = Vec3::new(c.x - a.x, c.y - a.y, c.z - a.z);
    let ap = Vec3::new(point.x - a.x, point.y - a.y, point.z - a.z);

    let d1 = ab.x * ap.x + ab.y * ap.y + ab.z * ap.z;
    let d2 = ac.x * ap.x + ac.y * ap.y + ac.z * ap.z;

    if d1 <= 0.0 && d2 <= 0.0 {
        return *a; // Barycentric coords (1,0,0)
    }

    let bp = Vec3::new(point.x - b.x, point.y - b.y, point.z - b.z);

    let d3 = ab.x * bp.x + ab.y * bp.y + ab.z * bp.z;
    let d4 = ac.x * bp.x + ac.y * bp.y + ac.z * bp.z;

    if d3 >= 0.0 && d4 <= d3 {
        return *b; // Barycentric coords (0,1,0)
    }

    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return Point3F::new(a.x + v * ab.x, a.y + v * ab.y, a.z + v * ab.z);
    }

    let cp = Vec3::new(point.x - c.x, point.y - c.y, point.z - c.z);

    let d5 = ab.x * cp.x + ab.y * cp.y + ab.z * cp.z;
    let d6 = ac.x * cp.x + ac.y * cp.y + ac.z * cp.z;

    if d6 >= 0.0 && d5 <= d6 {
        return *c; // Barycentric coords (0,0,1)
    }

    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return Point3F::new(a.x + w * ac.x, a.y + w * ac.y, a.z + w * ac.z);
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        let bc = Vec3::new(c.x - b.x, c.y - b.y, c.z - b.z);
        return Point3F::new(b.x + w * bc.x, b.y + w * bc.y, b.z + w * bc.z);
    }

    // Inside triangle
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    Point3F::new(
        a.x + ab.x * v + ac.x * w,
        a.y + ab.y * v + ac.y * w,
        a.z + ab.z * v + ac.z * w,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_aabb_mesh_creation() {
        // Create a simple triangle mesh
        let mut mesh = TriangleMesh::new();
        let v0 = mesh.add_vertex(Point3F::new(0.0, 0.0, 0.0));
        let v1 = mesh.add_vertex(Point3F::new(1.0, 0.0, 0.0));
        let v2 = mesh.add_vertex(Point3F::new(0.0, 1.0, 0.0));
        mesh.add_triangle_indices(v0, v1, v2);

        let aabb_mesh = AABBMesh::from_mesh(&mesh, false);

        assert_eq!(aabb_mesh.triangle_count(), 1);
        assert_eq!(aabb_mesh.vertex_count(), 3);
    }

    #[test]
    fn test_ray_triangle_intersection() {
        let origin = Point3F::new(0.5, 0.5, 1.0);
        let dir = Vec3::new(0.0, 0.0, -1.0);
        let v0 = Point3F::new(0.0, 0.0, 0.0);
        let v1 = Point3F::new(1.0, 0.0, 0.0);
        let v2 = Point3F::new(0.0, 1.0, 0.0);

        let result = ray_triangle_intersect(&origin, &dir, &v0, &v1, &v2, 1e-6);

        assert!(result.is_some());
        let (t, u, v, point) = result.unwrap();
        assert!((t - 1.0).abs() < 1e-6);
        assert!(point.z.abs() < 1e-6);
    }

    #[test]
    fn test_closest_point_on_triangle() {
        let a = Point3F::new(0.0, 0.0, 0.0);
        let b = Point3F::new(1.0, 0.0, 0.0);
        let c = Point3F::new(0.0, 1.0, 0.0);

        // Point above triangle center
        let query = Point3F::new(0.25, 0.25, 1.0);
        let closest = closest_point_on_triangle(&query, &a, &b, &c);

        assert!((closest.x - 0.25).abs() < 1e-6);
        assert!((closest.y - 0.25).abs() < 1e-6);
        assert!(closest.z.abs() < 1e-6);
    }
}
