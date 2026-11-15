//! Port of `libslic3r/MeshRaycast.hpp` and `MeshRaycast.cpp`.
//!
//! All the hits of a ray with an indexed triangle set, over the AABB tree of
//! `aabb_tree_indirect`. `mesh_diagnostics` casts its rays through this.
//!
//! C++ reference: `references/BambuStudio/src/libslic3r/MeshRaycast.{hpp,cpp}`.

use crate::aabb_tree_indirect::{intersect_ray_all_hits_eps, intersect_triangle, SourceNode, Tree, Tree3F};
use crate::geometry::{BoundingBox3F, Point3F};
use crate::triangle_set_sampling::{indexed_triangle_set, Vec3d};

/// MeshRaycast.hpp:13-18
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RayMeshHit {
    pub face: i32,
    pub t: f64,
    /// P = (1-u-v)*v0 + u*v1 + v*v2
    pub u: f64,
    pub v: f64,
}

impl Default for RayMeshHit {
    /// MeshRaycast.hpp:14-17 — `face = -1`, the rest 0.
    fn default() -> Self {
        Self { face: -1, t: 0.0, u: 0.0, v: 0.0 }
    }
}

/// MeshRaycast.cpp:11-15 — `TriangleRaycaster::Impl`
///
/// FIDELITY-NOTE: the C++ keeps a pointer to the caller's `indexed_triangle_set` and an
/// `AABBTreeIndirect::Tree3f`. The tree functions of `aabb_tree_indirect` take the vertices as
/// `Point3F` (doubles) and the faces as `[usize; 3]`, so the mesh is copied into that form here.
/// A float converts to a double exactly; the boxes are computed in floats as the C++ does and
/// then converted, which is the `node.bbox.cast<double>()` of the traversal.
struct Impl {
    vertices: Vec<Point3F>,
    faces: Vec<[usize; 3]>,
    tree: Tree3F,
    eps: f64,
}

/// MeshRaycast.hpp:20-33
pub struct TriangleRaycaster {
    m_impl: Impl,
}

/// One triangle of the tree's input (AABBTreeIndirect.hpp:687-695, `InputType`).
struct InputType {
    m_idx: usize,
    m_bbox: BoundingBox3F,
    m_centroid: Point3F,
}

impl SourceNode for InputType {
    fn idx(&self) -> usize {
        self.m_idx
    }
    fn centroid(&self) -> Point3F {
        self.m_centroid
    }
    fn bbox(&self) -> BoundingBox3F {
        self.m_bbox.clone()
    }
}

/// `std::min` / Eigen `cwiseMin` on floats: `(b < a) ? b : a`.
fn min_f32(a: f32, b: f32) -> f32 {
    if b < a {
        b
    } else {
        a
    }
}

/// `std::max` / Eigen `cwiseMax` on floats: `(a < b) ? b : a`.
fn max_f32(a: f32, b: f32) -> f32 {
    if a < b {
        b
    } else {
        a
    }
}

impl TriangleRaycaster {
    /// MeshRaycast.cpp:17-36
    ///
    /// The face indices must be valid, as for the C++ (which reads the vertices unchecked).
    pub fn new(its: &indexed_triangle_set) -> Self {
        let mut m_impl = Impl { vertices: Vec::new(), faces: Vec::new(), tree: Tree::new(), eps: 0.000001 };
        if its.indices.is_empty() || its.vertices.is_empty() {
            return Self { m_impl };
        }
        // Float AABBs with eps=0 drop leaves whose box face/edge the ray grazes
        // (axis rays on a silhouette shared edge). Expansion only affects
        // traversal; the triangle test is unchanged. Scale with the mesh AABB
        // so a 2 mm part and a 300 mm part share one policy.
        let mut bmin = its.vertices[0];
        let mut bmax = bmin;
        for v in &its.vertices {
            for k in 0..3 {
                bmin[k] = min_f32(bmin[k], v[k]);
                bmax[k] = max_f32(bmax[k], v[k]);
            }
        }
        // `(bmax - bmin).maxCoeff()`
        let mut max_ext: f32 = bmax[0] - bmin[0];
        for k in 1..3 {
            let value = bmax[k] - bmin[k];
            if value > max_ext {
                max_ext = value;
            }
        }
        let build_eps: f32 = max_f32(1e-4f32 * max_ext, 1e-5f32);

        // AABBTreeIndirect.hpp:687-735, `build_aabb_tree_over_indexed_triangle_set(vertices, faces, eps)`
        // for a `Tree3f`: centroid and box in floats, the box grown by eps on every side.
        let mut input: Vec<InputType> = Vec::with_capacity(its.indices.len());
        let third = (1.0f64 / 3.0) as f32;
        for (i, face) in its.indices.iter().enumerate() {
            let v1 = &its.vertices[face[0] as usize];
            let v2 = &its.vertices[face[1] as usize];
            let v3 = &its.vertices[face[2] as usize];
            let mut centroid = [0.0f64; 3];
            let mut lo = [0.0f64; 3];
            let mut hi = [0.0f64; 3];
            for k in 0..3 {
                centroid[k] = f64::from(third * ((v1[k] + v2[k]) + v3[k]));
                // `BoundingBox(v1, v1)`, `extend(v2)`, `extend(v3)`, then `min() -= veps`, `max() += veps`.
                lo[k] = f64::from(min_f32(min_f32(v1[k], v2[k]), v3[k]) - build_eps);
                hi[k] = f64::from(max_f32(max_f32(v1[k], v2[k]), v3[k]) + build_eps);
            }
            let bbox =
                BoundingBox3F::from_points_minmax(Point3F::new(lo[0], lo[1], lo[2]), Point3F::new(hi[0], hi[1], hi[2]));
            input.push(InputType { m_idx: i, m_bbox: bbox, m_centroid: Point3F::new(centroid[0], centroid[1], centroid[2]) });
        }
        m_impl.tree.build(input);

        m_impl.vertices =
            its.vertices.iter().map(|v| Point3F::new(f64::from(v[0]), f64::from(v[1]), f64::from(v[2]))).collect();
        m_impl.faces = its.indices.iter().map(|f| [f[0] as usize, f[1] as usize, f[2] as usize]).collect();
        Self { m_impl }
    }

    /// MeshRaycast.cpp:40-66
    ///
    /// Hits are sorted by the ray parameter t. A ray coplanar with a triangle
    /// intersects it along a segment rather than at a point, so it yields no hit;
    /// `coplanar_count` is set to 0 (the tree reports such a triangle as a miss).
    pub fn all_hits(&self, origin: &Vec3d, dir: &Vec3d, coplanar_count: Option<&mut i32>) -> Vec<RayMeshHit> {
        if let Some(coplanar_count) = coplanar_count {
            *coplanar_count = 0;
        }

        let mut hits: Vec<RayMeshHit> = Vec::new();
        // `dir.squaredNorm() == 0.`: zero exactly when every component is zero.
        if self.m_impl.tree.empty() || (dir[0] == 0.0 && dir[1] == 0.0 && dir[2] == 0.0) {
            return hits;
        }

        // AABBTreeIndirect reports a miss when the ray is coplanar with a triangle
        // (Möller–Trumbore det ~ 0), so coplanar_count stays 0. Odd-hit discard in
        // the watertight test already treats those leaks as invalid rays.
        let origin = Point3F::new(origin[0], origin[1], origin[2]);
        let dir = Point3F::new(dir[0], dir[1], dir[2]);
        let igl_hits = intersect_ray_all_hits_eps(
            &self.m_impl.vertices,
            &self.m_impl.faces,
            &self.m_impl.tree,
            &origin,
            &dir,
            self.m_impl.eps,
        );

        hits.reserve(igl_hits.len());
        for (_, face_idx, _) in igl_hits {
            // `igl::Hit{ int(node.idx), -1, float(u), float(v), float(t) }` (AABBTreeIndirect.hpp:462):
            // the hit holds floats. `intersect_ray_all_hits_eps` returns the double t and no u, v, so
            // the triangle is intersected once more for the three; the result is the same as the
            // one the traversal computed, and the order by double t is an order by float t.
            let face = &self.m_impl.faces[face_idx];
            let Some((t, u, v)) = intersect_triangle(
                &origin,
                &dir,
                &self.m_impl.vertices[face[0]],
                &self.m_impl.vertices[face[1]],
                &self.m_impl.vertices[face[2]],
                self.m_impl.eps,
            ) else {
                continue;
            };
            let mut hit = RayMeshHit::default();
            hit.face = face_idx as i32;
            hit.t = f64::from(t as f32);
            hit.u = f64::from(u as f32);
            hit.v = f64::from(v as f32);
            hits.push(hit);
        }
        hits
    }
}
