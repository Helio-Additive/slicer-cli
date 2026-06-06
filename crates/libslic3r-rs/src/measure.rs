//! Faithful 1:1 port of `src/libslic3r/Measure.cpp` (+ `Measure.hpp`) from
//! BambuStudio libslic3r.
//!
//! ///|/ Copyright (c) Prusa Research 2022 - 2023 Lukáš Matěna @lukasmatena,
//! ///|/ Enrico Turri @enricoturri1966, Vojtěch Bubník @bubnikv, Pavel Mikuš @Godrak
//! ///|/
//! ///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
//! ///|/
//!
//! Line references below point into `Measure.cpp` / `Measure.hpp`.
//!
//! Fidelity notes (byte-exact G-code parity):
//! - This module is the 3D measurement tool used by the GUI; it does not feed
//!   the slicing/G-code path. It is ported here for full libslic3r parity.
//! - `Vec3d`/`Vec2d`/`Transform3d` map to nalgebra `Vector3<f64>`/`Vector2<f64>`/
//!   `Matrix4<f64>`, matching `crate::geometry::geometry`.
//! - Eigen's `Hyperplane`, `ParametrizedLine` and `Quaterniond` have no nalgebra
//!   equivalents, so they are reimplemented locally below, reproducing Eigen's
//!   exact formulas (un-normalized hyperplane coeffs, etc.).
//! - `Geometry::circle_ransac` returns `crate::geometry::circle::Circle` whose
//!   `center` is `PointF`; we convert to/from `Vector2<f64>` at the boundary.
//! - `RootsPolynomial`/`Polynomial1`/`get_orthogonal` mirror `MeasureUtils.hpp`;
//!   `Polynomial1`/`RootsPolynomial` are reused from `crate::measure_utils`.
//!   `get_orthogonal` is re-ported locally because the `measure_utils` copy
//!   returns the incompatible `aabb_tree::Vec3` type.

#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(clippy::needless_range_loop)]
#![allow(clippy::too_many_arguments)]

use std::rc::Rc;

use nalgebra::{Matrix4, UnitQuaternion, Vector2, Vector3};

use crate::geometry::circle_ransac;
use crate::geometry::PointF;
use crate::libslic3r::EPSILON;
use crate::measure_utils::{Polynomial1, RootsPolynomial};
use crate::surface_mesh::{Face_index, SurfaceMesh, Vertex_index};
use crate::triangle_set_sampling::{indexed_triangle_set, Vec3f, Vec3i};

// `Vec2i` == nalgebra `Vector2<i32>` (stl edge indices).
type Vec2i = nalgebra::Vector2<i32>;

// Eigen `Vec3d` == nalgebra `Vector3<f64>`.
pub type Vec3d = Vector3<f64>;
// Eigen `Vec2d` == nalgebra `Vector2<f64>`.
pub type Vec2d = Vector2<f64>;
// Eigen `Transform3d` == homogeneous affine 4x4 matrix.
pub type Transform3d = Matrix4<f64>;
// admesh `stl_normal` == `Vec3f` (single precision normal).
pub type stl_normal = Vec3f;

// C++ `sqr` from libslic3r.h.
#[inline]
fn sqr(x: f64) -> f64 {
    x * x
}

// `M_PI` / `PI`.
const M_PI: f64 = std::f64::consts::PI;
const PI: f64 = std::f64::consts::PI;

// `Slic3r::is_approx(a, b, tol)` (libslic3r.h) — scalar with explicit precision.
#[inline]
fn is_approx_tol(value: f64, test_value: f64, precision: f64) -> bool {
    (value - test_value).abs() < precision
}

// `Slic3r::is_approx(Vec3d, Vec3d)` — uses default EPSILON precision per coord.
#[inline]
fn is_approx_vec(a: &Vec3d, b: &Vec3d) -> bool {
    is_approx_tol(a.x, b.x, EPSILON) && is_approx_tol(a.y, b.y, EPSILON) && is_approx_tol(a.z, b.z, EPSILON)
}

// Eigen `m.isApprox(n)` for vectors (relative comparison, default precision).
#[inline]
fn is_approx_eigen(a: &Vec3d, b: &Vec3d) -> bool {
    // Eigen's isApprox: (a-b).squaredNorm() <= prec^2 * min(a.sqNorm, b.sqNorm)
    let prec = f64::EPSILON.sqrt();
    (a - b).norm_squared() <= prec * prec * a.norm_squared().min(b.norm_squared())
}

// Apply an affine `Transform3d` to a point (Eigen `Transform3d * Vec3d`).
#[inline]
fn tran_pt(m: &Transform3d, v: &Vec3d) -> Vec3d {
    let linear = m.fixed_view::<3, 3>(0, 0);
    let t = Vec3d::new(m[(0, 3)], m[(1, 3)], m[(2, 3)]);
    linear * v + t
}

// `Transform3d::Identity()` then `.rotate(q)` — pure-rotation affine.
#[inline]
fn rotation_transform_from_quat(q: &UnitQuaternion<f64>) -> Transform3d {
    let r = q.to_rotation_matrix();
    let mut m = Transform3d::identity();
    m.fixed_view_mut::<3, 3>(0, 0).copy_from(r.matrix());
    m
}

// `Eigen::Quaterniond::FromTwoVectors(a, b)` / `q.setFromTwoVectors(a, b)`.
#[inline]
fn quat_from_two_vectors(a: &Vec3d, b: &Vec3d) -> UnitQuaternion<f64> {
    UnitQuaternion::rotation_between(a, b).unwrap_or_else(UnitQuaternion::identity)
}

// ---------------------------------------------------------------------------
// TriangleMesh.{hpp,cpp} helpers used by Measure.cpp (its_face_normals /
// its_face_neighbors). Ported locally against the `triangle_set_sampling`
// `indexed_triangle_set`, matching the type used by `SurfaceMesh`.
// ---------------------------------------------------------------------------

// TriangleMesh.hpp — face_normal_normalized(vertices)
#[inline]
fn face_normal(vertices: &[Vec3f; 3]) -> Vec3f {
    (vertices[1] - vertices[0]).cross(&(vertices[2] - vertices[1]))
}
#[inline]
fn face_normal_normalized(vertices: &[Vec3f; 3]) -> Vec3f {
    let n = face_normal(vertices);
    n.normalize()
}

// TriangleMesh.hpp:333-334 — its_face_normal
#[inline]
fn its_face_normal(its: &indexed_triangle_set, face: &Vec3i) -> Vec3f {
    let vertices: [Vec3f; 3] = [
        its.vertices[face[0] as usize],
        its.vertices[face[1] as usize],
        its.vertices[face[2] as usize],
    ];
    face_normal_normalized(&vertices)
}

// TriangleMesh.cpp:1938-1945 — its_face_normals
fn its_face_normals(its: &indexed_triangle_set) -> Vec<Vec3f> {
    let mut normals: Vec<Vec3f> = Vec::new();
    normals.reserve(its.indices.len());
    for face in its.indices.iter() {
        normals.push(its_face_normal(its, face));
    }
    normals
}

// TriangleMesh.hpp:249-254 — its_triangle_vertex_index
#[inline]
fn its_triangle_vertex_index(triangle_indices: &Vec3i, vertex_idx: i32) -> i32 {
    if vertex_idx == triangle_indices[0] {
        0
    } else if vertex_idx == triangle_indices[1] {
        1
    } else if vertex_idx == triangle_indices[2] {
        2
    } else {
        -1
    }
}

// TriangleMesh.hpp:256-260 — its_triangle_edge
#[inline]
fn its_triangle_edge(triangle_indices: &Vec3i, edge_idx: i32) -> Vec2i {
    let next_edge_idx: i32 = if edge_idx == 2 { 0 } else { edge_idx + 1 };
    Vec2i::new(
        triangle_indices[edge_idx as usize],
        triangle_indices[next_edge_idx as usize],
    )
}

// TriangleMesh.hpp:168-191 — VertexFaceIndex
struct VertexFaceIndex {
    m_vertex_to_face_start: Vec<usize>,
    m_vertex_faces_all: Vec<usize>,
}

impl VertexFaceIndex {
    // TriangleMesh.hpp:173
    fn new(its: &indexed_triangle_set) -> Self {
        let mut idx = VertexFaceIndex {
            m_vertex_to_face_start: Vec::new(),
            m_vertex_faces_all: Vec::new(),
        };
        idx.create(its);
        idx
    }

    // TriangleMesh.cpp:1903-1926 — create
    fn create(&mut self, its: &indexed_triangle_set) {
        self.m_vertex_to_face_start = vec![0usize; its.vertices.len() + 1];
        for face in its.indices.iter() {
            self.m_vertex_to_face_start[face[0] as usize + 1] += 1;
            self.m_vertex_to_face_start[face[1] as usize + 1] += 1;
            self.m_vertex_to_face_start[face[2] as usize + 1] += 1;
        }
        for i in 2..self.m_vertex_to_face_start.len() {
            self.m_vertex_to_face_start[i] += self.m_vertex_to_face_start[i - 1];
        }
        self.m_vertex_faces_all = vec![0usize; *self.m_vertex_to_face_start.last().unwrap()];
        for face_idx in 0..its.indices.len() {
            let face = &its.indices[face_idx];
            for i in 0..3 {
                let slot = self.m_vertex_to_face_start[face[i] as usize];
                self.m_vertex_faces_all[slot] = face_idx;
                self.m_vertex_to_face_start[face[i] as usize] += 1;
            }
        }
        let mut i = self.m_vertex_to_face_start.len() as i32 - 1;
        while i > 0 {
            self.m_vertex_to_face_start[i as usize] = self.m_vertex_to_face_start[i as usize - 1];
            i -= 1;
        }
        self.m_vertex_to_face_start[0] = 0;
    }

    // TriangleMesh.hpp:180-185 — faces incident with vertex_id
    #[inline]
    fn faces(&self, vertex_id: usize) -> &[usize] {
        let begin = self.m_vertex_to_face_start[vertex_id];
        let end = self.m_vertex_to_face_start[vertex_id + 1];
        &self.m_vertex_faces_all[begin..end]
    }
}

// MeshSplitImpl.hpp:293-342 — create_face_neighbors_index
fn create_face_neighbors_index(its: &indexed_triangle_set) -> Vec<Vec3i> {
    let indices = &its.indices;
    if indices.is_empty() {
        return Vec::new();
    }
    debug_assert!(!its.vertices.is_empty());

    let vertex_triangles = VertexFaceIndex::new(its);
    const NO_VALUE: i32 = -1;
    let mut neighbors: Vec<Vec3i> = vec![Vec3i::new(NO_VALUE, NO_VALUE, NO_VALUE); indices.len()];

    for face_idx in 0..indices.len() {
        let triangle_indices = indices[face_idx];
        for edge_index in 0..3usize {
            if neighbors[face_idx][edge_index] != NO_VALUE {
                continue; // This edge already has a neighbor assigned.
            }
            let edge_indices = its_triangle_edge(&triangle_indices, edge_index as i32);
            for &other_face in vertex_triangles.faces(edge_indices[0] as usize) {
                if other_face <= face_idx {
                    continue;
                }
                let face_indices = indices[other_face];
                let vertex_index = its_triangle_vertex_index(&face_indices, edge_indices[1]);
                if vertex_index < 0 {
                    continue; // NOT Contain second vertex?
                }
                if edge_indices[0] != face_indices[((vertex_index + 1) % 3) as usize] {
                    continue; // Has NOT opposite direction?
                }
                if neighbors[other_face][vertex_index as usize] != NO_VALUE {
                    continue; // already marked before, skip it
                }
                neighbors[face_idx][edge_index] = other_face as i32;
                neighbors[other_face][vertex_index as usize] = face_idx as i32;
                break;
            }
        }
    }

    neighbors
}

// TriangleMesh.cpp:1933-1936 — its_face_neighbors
#[allow(dead_code)]
fn its_face_neighbors(its: &indexed_triangle_set) -> Vec<Vec3i> {
    create_face_neighbors_index(its)
}

// ---------------------------------------------------------------------------
// Eigen geometry analogs (Hyperplane / ParametrizedLine).
// ---------------------------------------------------------------------------

// `Eigen::Hyperplane<double, 3>`. Coeffs are [n.x, n.y, n.z, d] with
// d = -n.dot(point); the normal is NOT normalized by the (n, p) constructor.
struct Hyperplane3 {
    normal: Vec3d,
    offset: f64,
}

impl Hyperplane3 {
    // Hyperplane(const Vec3d& n, const Vec3d& e) : normal(n), offset(-n.dot(e)) {}
    fn new(n: Vec3d, e: Vec3d) -> Self {
        Hyperplane3 { normal: n, offset: -n.dot(&e) }
    }

    // RealScalar signedDistance(const VectorType& p) const { return normal().dot(p) + offset(); }
    #[inline]
    fn signed_distance(&self, p: &Vec3d) -> f64 {
        self.normal.dot(p) + self.offset
    }

    // RealScalar absDistance(const VectorType& p) const { return numext::abs(signedDistance(p)); }
    #[inline]
    fn abs_distance(&self, p: &Vec3d) -> f64 {
        self.signed_distance(p).abs()
    }

    // VectorType projection(const VectorType& p) const { return p - signedDistance(p) * normal(); }
    #[inline]
    fn projection(&self, p: &Vec3d) -> Vec3d {
        p - self.signed_distance(p) * self.normal
    }
}

// `Eigen::ParametrizedLine<double, 3>`.
struct ParametrizedLine3 {
    origin: Vec3d,
    direction: Vec3d,
}

impl ParametrizedLine3 {
    // ParametrizedLine(const VectorType& origin, const VectorType& direction)
    fn new(origin: Vec3d, direction: Vec3d) -> Self {
        ParametrizedLine3 { origin, direction }
    }

    // static Through(p0, p1) { return ParametrizedLine(p0, (p1 - p0).normalized()); }
    fn through(p0: Vec3d, p1: Vec3d) -> Self {
        ParametrizedLine3::new(p0, (p1 - p0).normalize())
    }

    // VectorType projection(const VectorType& p) const
    //   { return origin() + (p - origin()).dot(direction()) * direction(); }
    #[inline]
    fn projection(&self, p: &Vec3d) -> Vec3d {
        self.origin + (p - self.origin).dot(&self.direction) * self.direction
    }

    // RealScalar distance(const VectorType& p) const { return sqrt(squaredDistance(p)); }
    // squaredDistance(p) = (origin() - p) - ((origin() - p).dot(dir())) * dir()).squaredNorm()
    #[inline]
    fn distance(&self, p: &Vec3d) -> f64 {
        let diff = self.origin - p;
        (diff - diff.dot(&self.direction) * self.direction).norm()
    }

    // Scalar intersectionParameter(const Hyperplane& hyperplane) const
    //   { return -(hyperplane.offset() + hyperplane.normal().dot(origin()))
    //              / hyperplane.normal().dot(direction()); }
    // VectorType intersectionPoint(const Hyperplane& hyperplane) const
    //   { return origin() + intersectionParameter(hyperplane) * direction(); }
    #[inline]
    fn intersection_point(&self, hyperplane: &Hyperplane3) -> Vec3d {
        let t = -(hyperplane.offset + hyperplane.normal.dot(&self.origin))
            / hyperplane.normal.dot(&self.direction);
        self.origin + t * self.direction
    }
}

// `Eigen::Hyperplane<double, 2>`. Coeffs [n.x, n.y, d], d = -n.dot(point).
struct Hyperplane2 {
    coeffs: nalgebra::Vector3<f64>,
}

impl Hyperplane2 {
    // static Through(p0, p1):
    //   result.normal() = (p1 - p0).unitOrthogonal();
    //   result.offset() = -p0.dot(result.normal());
    fn through(p0: Vec2d, p1: Vec2d) -> Self {
        let d = p1 - p0;
        // Eigen Vector2::unitOrthogonal() returns (-y, x).normalized().
        let normal = Vec2d::new(-d.y, d.x).normalize();
        let offset = -p0.dot(&normal);
        Hyperplane2 {
            coeffs: nalgebra::Vector3::new(normal.x, normal.y, offset),
        }
    }

    // VectorType intersection(const Hyperplane& other) const  (Hyperplane.h, dim==2)
    fn intersection(&self, other: &Hyperplane2) -> Vec2d {
        let det = self.coeffs.x * other.coeffs.y - self.coeffs.y * other.coeffs.x;
        // Eigen branches on |det| relative to magnitudes to avoid /0; we mirror
        // the general (non-degenerate) branch which is what the caller relies on.
        let invdet = 1.0 / det;
        Vec2d::new(
            invdet * (self.coeffs.y * other.coeffs.z - other.coeffs.y * self.coeffs.z),
            invdet * (other.coeffs.x * self.coeffs.z - self.coeffs.x * other.coeffs.z),
        )
    }
}

// ---------------------------------------------------------------------------
// Measure.hpp — SurfaceFeatureType / SurfaceFeature
// ---------------------------------------------------------------------------

// Measure.hpp:20-26
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum SurfaceFeatureType {
    Undef = 0,
    Point = 1 << 0,
    Edge = 1 << 1,
    Circle = 1 << 2,
    Plane = 1 << 3,
}

// Measure.cpp:20-27
pub fn get_point_projection_to_plane(
    pt: &Vec3d,
    plane_origin: &Vec3d,
    plane_normal: &Vec3d,
    intersection_pt: &mut Vec3d,
) -> bool {
    let normal = plane_normal.normalize();
    let ba = plane_origin - pt;
    let length = ba.dot(&normal);
    *intersection_pt = pt + length * normal;
    true
}

// Measure.cpp:29-40
pub fn get_one_point_in_plane(plane_origin: &Vec3d, plane_normal: &Vec3d) -> Vec3d {
    let mut dir = Vec3d::new(1.0, 0.0, 0.0);
    let eps: f32 = 1e-3;
    if (plane_normal.dot(&dir)).abs() > 1.0 - eps as f64 {
        dir = Vec3d::new(0.0, 1.0, 0.0);
    }
    let new_pt = plane_origin + dir;
    let mut retult = Vec3d::zeros();
    get_point_projection_to_plane(&new_pt, plane_origin, plane_normal, &mut retult);
    retult
}

// Measure.hpp:31-116
#[derive(Debug, Clone)]
pub struct SurfaceFeature {
    // public fields (Measure.hpp:99-103)
    pub plane_indices: Option<Vec<i32>>,
    pub world_tran: Transform3d,
    pub world_plane_features: Option<Rc<Vec<SurfaceFeature>>>,
    pub origin_surface_feature: Option<Rc<SurfaceFeature>>,

    // private (Measure.hpp:110-115)
    m_type: SurfaceFeatureType,
    m_pt1: Vec3d,
    m_pt2: Vec3d,
    m_pt3: Option<Vec3d>,
    m_value: f64,
}

impl SurfaceFeature {
    // Measure.hpp:34-35
    pub fn new(
        type_: SurfaceFeatureType,
        pt1: Vec3d,
        pt2: Vec3d,
        pt3: Option<Vec3d>,
        value: f64,
    ) -> Self {
        SurfaceFeature {
            plane_indices: None,
            world_tran: Transform3d::identity(),
            world_plane_features: None,
            origin_surface_feature: None,
            m_type: type_,
            m_pt1: pt1,
            m_pt2: pt2,
            m_pt3: pt3,
            m_value: value,
        }
    }

    // Measure.hpp:37-38 — SurfaceFeature(const Vec3d& pt)
    pub fn from_point(pt: Vec3d) -> Self {
        SurfaceFeature {
            plane_indices: None,
            world_tran: Transform3d::identity(),
            world_plane_features: None,
            origin_surface_feature: None,
            m_type: SurfaceFeatureType::Point,
            m_pt1: pt,
            m_pt2: Vec3d::zeros(),
            m_pt3: None,
            m_value: 0.0,
        }
    }

    // Measure.hpp:57 — void translate(const Vec3d& displacement);
    // Measure.cpp:1355-1381
    pub fn translate(&mut self, displacement: &Vec3d) {
        match self.get_type() {
            // Measure.cpp:1357-1360
            SurfaceFeatureType::Point => {
                self.m_pt1 += displacement;
            }
            // Measure.cpp:1361-1368
            SurfaceFeatureType::Edge => {
                self.m_pt1 += displacement;
                self.m_pt2 += displacement;
                if let Some(p3) = self.m_pt3 {
                    self.m_pt3 = Some(p3 + displacement);
                }
            }
            // Measure.cpp:1369-1373 — m_pt1 is normal
            SurfaceFeatureType::Plane => {
                self.m_pt2 += displacement;
            }
            // Measure.cpp:1374-1378 — m_pt2 is normal
            SurfaceFeatureType::Circle => {
                self.m_pt1 += displacement;
            }
            // Measure.cpp:1379
            SurfaceFeatureType::Undef => {}
        }
    }

    // Measure.hpp:58 — void translate(const Transform3d& tran);
    // Measure.cpp:1383-1431
    pub fn translate_tran(&mut self, tran: &Transform3d) {
        match self.get_type() {
            // Measure.cpp:1386-1389
            SurfaceFeatureType::Point => {
                self.m_pt1 = tran_pt(tran, &self.m_pt1);
            }
            // Measure.cpp:1390-1397
            SurfaceFeatureType::Edge => {
                self.m_pt1 = tran_pt(tran, &self.m_pt1);
                self.m_pt2 = tran_pt(tran, &self.m_pt2);
                if let Some(p3) = self.m_pt3 {
                    self.m_pt3 = Some(tran_pt(tran, &p3));
                }
            }
            // Measure.cpp:1398-1405 — m_pt1 is normal
            SurfaceFeatureType::Plane => {
                let temp_pt1 = self.m_pt2 + self.m_pt1;
                let temp_pt1 = tran_pt(tran, &temp_pt1);
                self.m_pt2 = tran_pt(tran, &self.m_pt2);
                self.m_pt1 = (temp_pt1 - self.m_pt2).normalize();
            }
            // Measure.cpp:1406-1428 — m_pt1 center, m_pt2 normal
            SurfaceFeatureType::Circle => {
                let local_normal = self.m_pt2;
                let local_center = self.m_pt1;
                let temp_pt2 = local_normal + local_center;
                let temp_pt2 = tran_pt(tran, &temp_pt2);
                self.m_pt1 = tran_pt(tran, &self.m_pt1);
                let world_center = self.m_pt1;
                self.m_pt2 = (temp_pt2 - self.m_pt1).normalize();

                // Measure.cpp:1417-1423 — calc_world_radius lambda
                let calc_world_radius = |pt: &Vec3d, value: &mut f64| {
                    let mut intersection_pt = Vec3d::zeros();
                    get_point_projection_to_plane(pt, &local_center, &local_normal, &mut intersection_pt);
                    let local_radius_pt =
                        (intersection_pt - local_center).normalize() * *value + local_center;
                    let radius_pt = tran_pt(tran, &local_radius_pt);
                    *value = (radius_pt - world_center).norm();
                };
                // Measure.cpp:1425-1426 — m_value is radius
                let new_pt = get_one_point_in_plane(&local_center, &local_normal);
                let mut value = self.m_value;
                calc_world_radius(&new_pt, &mut value);
                self.m_value = value;
            }
            // Measure.cpp:1429
            SurfaceFeatureType::Undef => {}
        }
    }

    // Measure.hpp:60 — Get type of this feature.
    pub fn get_type(&self) -> SurfaceFeatureType {
        self.m_type
    }

    // Measure.hpp:63 — For points, return the point.
    pub fn get_point(&self) -> Vec3d {
        debug_assert!(self.m_type == SurfaceFeatureType::Point);
        self.m_pt1
    }

    // Measure.hpp:65 — For edges, return start and end.
    pub fn get_edge(&self) -> (Vec3d, Vec3d) {
        debug_assert!(self.m_type == SurfaceFeatureType::Edge);
        (self.m_pt1, self.m_pt2)
    }

    // Measure.hpp:68 — For circles, return center, radius and normal.
    pub fn get_circle(&self) -> (Vec3d, f64, Vec3d) {
        debug_assert!(self.m_type == SurfaceFeatureType::Circle);
        (self.m_pt1, self.m_value, self.m_pt2)
    }

    // Measure.hpp:71 — For planes, return (index, normal, point).
    pub fn get_plane(&self) -> (i32, Vec3d, Vec3d) {
        debug_assert!(self.m_type == SurfaceFeatureType::Plane);
        (self.m_value as i32, self.m_pt1, self.m_pt2)
    }

    // Measure.hpp:74 — For anything, return an extra point that should also be part of this.
    pub fn get_extra_point(&self) -> Option<Vec3d> {
        debug_assert!(self.m_type != SurfaceFeatureType::Undef);
        self.m_pt3
    }

    // Measure.hpp:105-108
    pub fn get_pt1(&self) -> Vec3d {
        self.m_pt1
    }
    pub fn get_pt2(&self) -> Vec3d {
        self.m_pt2
    }
    pub fn get_pt3(&self) -> &Option<Vec3d> {
        &self.m_pt3
    }
    pub fn get_value(&self) -> f64 {
        self.m_value
    }
}

// Measure.hpp:76-93 — operator ==
impl PartialEq for SurfaceFeature {
    fn eq(&self, other: &Self) -> bool {
        if self.m_type != other.m_type {
            return false;
        }
        match self.m_type {
            SurfaceFeatureType::Undef => false,
            SurfaceFeatureType::Point => is_approx_eigen(&self.m_pt1, &other.m_pt1),
            SurfaceFeatureType::Edge => {
                (is_approx_eigen(&self.m_pt1, &other.m_pt1) && is_approx_eigen(&self.m_pt2, &other.m_pt2))
                    || (is_approx_eigen(&self.m_pt1, &other.m_pt2)
                        && is_approx_eigen(&self.m_pt2, &other.m_pt1))
            }
            SurfaceFeatureType::Plane | SurfaceFeatureType::Circle => {
                is_approx_eigen(&self.m_pt1, &other.m_pt1)
                    && is_approx_eigen(&self.m_pt2, &other.m_pt2)
                    && (self.m_value - other.m_value).abs() < EPSILON
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Measure.cpp:42-80 — internal helpers
// ---------------------------------------------------------------------------

// Measure.cpp:42 — how close to a feature the mouse must be to highlight it
const FEATURE_HOVER_LIMIT: f64 = 0.5;

// Measure.cpp:44-62
fn get_center_and_radius(
    points: &[Vec3d],
    trafo: &Transform3d,
    _trafo_inv: &Transform3d,
) -> (Vec3d, f64, f64) {
    let mut out: Vec<PointF> = Vec::new();
    let mut z = 0.0;
    for pt in points.iter() {
        let pt_transformed = tran_pt(trafo, pt);
        z = pt_transformed.z;
        out.push(PointF::new(pt_transformed.x, pt_transformed.y));
    }

    // Measure.cpp:54-56
    let iter = if points.len() < 10 {
        2
    } else if points.len() < 100 {
        4
    } else {
        6
    };

    // Measure.cpp:58-59
    let mut error = f64::MAX;
    let circle = circle_ransac(&out, iter, Some(&mut error));

    // Measure.cpp:61
    let center3 = tran_pt(
        &trafo.try_inverse().unwrap_or_else(Transform3d::identity),
        &Vec3d::new(circle.center.x(), circle.center.y(), z),
    );
    (center3, circle.radius, error)
}

// Measure.cpp:66-80
fn orthonormal_basis(v: &Vec3d) -> [Vec3d; 3] {
    let mut ret = [Vec3d::zeros(); 3];
    ret[2] = v.normalize();
    // index of max abs coefficient
    let cw = ret[2].abs();
    let mut index = 0usize;
    if cw[1] > cw[index] {
        index = 1;
    }
    if cw[2] > cw[index] {
        index = 2;
    }
    match index {
        0 => {
            ret[0] = Vec3d::new(ret[2].y, -ret[2].x, 0.0).normalize();
        }
        1 => {
            ret[0] = Vec3d::new(0.0, ret[2].z, -ret[2].y).normalize();
        }
        2 => {
            ret[0] = Vec3d::new(-ret[2].z, 0.0, ret[2].x).normalize();
        }
        _ => {}
    }
    ret[1] = ret[2].cross(&ret[0]).normalize();
    ret
}

// MeasureUtils.hpp:359-385 — get_orthogonal (re-ported locally on nalgebra Vec3d;
// the `measure_utils` copy returns the incompatible `aabb_tree::Vec3`).
fn get_orthogonal(v: &Vec3d, unit_length: bool) -> Vec3d {
    let mut cmax = v[0].abs();
    let mut imax = 0usize;
    for i in 1..3 {
        let c = v[i].abs();
        if c > cmax {
            cmax = c;
            imax = i;
        }
    }
    let _ = cmax;

    let mut result = Vec3d::zeros();
    let mut inext = imax + 1;
    if inext == 3 {
        inext = 0;
    }

    result[imax] = v[inext];
    result[inext] = -v[imax];
    if unit_length {
        let sqr_distance = result[imax] * result[imax] + result[inext] * result[inext];
        let inv_length = 1.0 / sqr_distance.sqrt();
        result[imax] *= inv_length;
        result[inext] *= inv_length;
    }
    result
}

// ---------------------------------------------------------------------------
// Measure.hpp / Measure.cpp — free inline helpers from the header
// ---------------------------------------------------------------------------

// Measure.hpp:206
#[inline]
pub fn edge_direction(from: &Vec3d, to: &Vec3d) -> Vec3d {
    (to - from).normalize()
}
// Measure.hpp:207
#[inline]
pub fn edge_direction_pair(e: &(Vec3d, Vec3d)) -> Vec3d {
    edge_direction(&e.0, &e.1)
}
// Measure.hpp:208-211
#[inline]
pub fn edge_direction_feature(edge: &SurfaceFeature) -> Vec3d {
    debug_assert!(edge.get_type() == SurfaceFeatureType::Edge);
    let e = edge.get_edge();
    edge_direction(&e.0, &e.1)
}

// Measure.hpp:213-216
#[inline]
pub fn plane_normal(plane: &SurfaceFeature) -> Vec3d {
    debug_assert!(plane.get_type() == SurfaceFeatureType::Plane);
    plane.get_plane().1
}

// Measure.hpp:218
#[inline]
pub fn are_parallel(v1: &Vec3d, v2: &Vec3d) -> bool {
    (v1.dot(v2).abs() - 1.0).abs() < EPSILON
}
// Measure.hpp:219
#[inline]
pub fn are_perpendicular(v1: &Vec3d, v2: &Vec3d) -> bool {
    v1.dot(v2).abs() < EPSILON
}

// Measure.hpp:221-223
#[inline]
pub fn are_parallel_edges(e1: &(Vec3d, Vec3d), e2: &(Vec3d, Vec3d)) -> bool {
    are_parallel(&(e1.1 - e1.0), &(e2.1 - e2.0))
}
// Measure.hpp:224-231
pub fn are_parallel_features(f1: &SurfaceFeature, f2: &SurfaceFeature) -> bool {
    if f1.get_type() == SurfaceFeatureType::Edge && f2.get_type() == SurfaceFeatureType::Edge {
        are_parallel(&edge_direction_feature(f1), &edge_direction_feature(f2))
    } else if f1.get_type() == SurfaceFeatureType::Edge && f2.get_type() == SurfaceFeatureType::Plane {
        are_perpendicular(&edge_direction_feature(f1), &plane_normal(f2))
    } else {
        false
    }
}

// Measure.hpp:233-240
pub fn are_perpendicular_features(f1: &SurfaceFeature, f2: &SurfaceFeature) -> bool {
    if f1.get_type() == SurfaceFeatureType::Edge && f2.get_type() == SurfaceFeatureType::Edge {
        are_perpendicular(&edge_direction_feature(f1), &edge_direction_feature(f2))
    } else if f1.get_type() == SurfaceFeatureType::Edge && f2.get_type() == SurfaceFeatureType::Plane {
        are_parallel(&edge_direction_feature(f1), &plane_normal(f2))
    } else {
        false
    }
}

// ---------------------------------------------------------------------------
// Measure.cpp:84-111 — MeasuringImpl
// ---------------------------------------------------------------------------

// Measure.cpp:87-94 — PlaneData
#[derive(Clone)]
struct PlaneData {
    facets: Vec<i32>,
    // FIXME: should be in fact local in update_planes()
    borders: Vec<Vec<Vec3d>>,
    surface_features: Vec<SurfaceFeature>,
    normal: Vec3d,
    #[allow(dead_code)]
    area: f32,
    features_extracted: bool,
}

impl PlaneData {
    fn new() -> Self {
        PlaneData {
            facets: Vec::new(),
            borders: Vec::new(),
            surface_features: Vec::new(),
            normal: Vec3d::zeros(),
            area: 0.0,
            features_extracted: false,
        }
    }
}

pub struct MeasuringImpl {
    m_planes: Vec<PlaneData>,
    m_face_to_plane: Vec<usize>,
    m_its: indexed_triangle_set,
}

// Measure.cpp:145-147 — is_same_normal lambda (used by update_planes).
#[inline]
fn is_same_normal(a: &stl_normal, b: &stl_normal) -> bool {
    (a[0] - b[0]).abs() < 0.001 && (a[1] - b[1]).abs() < 0.001 && (a[2] - b[2]).abs() < 0.001
}

const SIZE_T_MINUS_ONE: usize = usize::MAX; // size_t(-1)

impl MeasuringImpl {
    // Measure.cpp:118-129
    pub fn new(its: &indexed_triangle_set) -> Self {
        let mut impl_ = MeasuringImpl {
            m_planes: Vec::new(),
            m_face_to_plane: Vec::new(),
            m_its: its.clone(),
        };
        impl_.update_planes();
        // Extracting features will be done as needed.
        // (DEBUG_EXTRACT_ALL_FEATURES_AT_ONCE == 0)
        impl_
    }

    // Measure.cpp:132-292
    fn update_planes(&mut self) {
        // Now we'll go through all the facets and append Points of facets sharing the same normal.
        // This part is still performed in mesh coordinate system.
        let num_of_facets = self.m_its.indices.len();
        self.m_face_to_plane = vec![SIZE_T_MINUS_ONE; num_of_facets];
        let face_normals: Vec<Vec3f> = its_face_normals(&self.m_its);
        let face_neighbors: Vec<Vec3i> = create_face_neighbors_index(&self.m_its);
        let mut facet_queue: Vec<i32> = vec![0; num_of_facets];
        let mut facet_queue_cnt: usize = 0;
        let mut normal_ptr: Option<stl_normal> = None;
        let mut seed_facet_idx: usize = 0;

        self.m_planes.clear();
        self.m_planes.reserve(num_of_facets / 5);

        // First go through all the triangles and fill in m_planes vector.
        loop {
            // Find next unvisited triangle:
            while seed_facet_idx < num_of_facets {
                if self.m_face_to_plane[seed_facet_idx] == SIZE_T_MINUS_ONE {
                    facet_queue[facet_queue_cnt] = seed_facet_idx as i32;
                    facet_queue_cnt += 1;
                    normal_ptr = Some(face_normals[seed_facet_idx]);
                    self.m_face_to_plane[seed_facet_idx] = self.m_planes.len();
                    self.m_planes.push(PlaneData::new());
                    break;
                }
                seed_facet_idx += 1;
            }
            if seed_facet_idx == num_of_facets {
                break; // Everything was visited already
            }

            while facet_queue_cnt > 0 {
                facet_queue_cnt -= 1;
                let facet_idx = facet_queue[facet_queue_cnt] as usize;
                let this_normal = face_normals[facet_idx];
                if is_same_normal(&this_normal, normal_ptr.as_ref().unwrap()) {
                    self.m_face_to_plane[facet_idx] = self.m_planes.len() - 1;
                    self.m_planes.last_mut().unwrap().facets.push(facet_idx as i32);
                    for j in 0..3 {
                        let neighbor_idx = face_neighbors[facet_idx][j];
                        if neighbor_idx >= 0
                            && self.m_face_to_plane[neighbor_idx as usize] == SIZE_T_MINUS_ONE
                        {
                            facet_queue[facet_queue_cnt] = neighbor_idx;
                            facet_queue_cnt += 1;
                        }
                    }
                }
            }

            let np = normal_ptr.unwrap();
            self.m_planes.last_mut().unwrap().normal =
                Vec3d::new(np[0] as f64, np[1] as f64, np[2] as f64);
            self.m_planes.last_mut().unwrap().facets.sort();
        }

        // Check that each facet is part of one of the planes.
        debug_assert!(!self.m_face_to_plane.iter().any(|&val| val == SIZE_T_MINUS_ONE));

        // Now we will walk around each of the planes and save vertices which form the border.
        let sm = SurfaceMesh::new(&self.m_its, face_neighbors.clone());

        let face_to_plane = &self.m_face_to_plane;
        // Measure.cpp:197-290 — tbb::parallel_for over planes; the per-plane work
        // is independent, so a sequential loop produces identical results.
        for plane_id in 0..self.m_planes.len() {
            update_plane_borders(&mut self.m_planes[plane_id], plane_id, face_to_plane, &face_neighbors, &sm);
        }
        self.m_planes.shrink_to_fit();
    }

    // Measure.cpp:294-527
    fn extract_features(&mut self, plane_idx: usize) {
        debug_assert!(!self.m_planes[plane_idx].features_extracted);

        // We take the plane out by cloning the parts we need read-only and
        // mutate surface_features in place; keep the same control flow.
        let normal = self.m_planes[plane_idx].normal;
        self.m_planes[plane_idx].surface_features.clear();

        // Measure.cpp:302-306
        let q = quat_from_two_vectors(&normal, &Vec3d::z());
        let trafo = rotation_transform_from_quat(&q);
        let trafo_inv = trafo.try_inverse().unwrap_or_else(Transform3d::identity);

        let mut angles: Vec<f64> = Vec::new();
        let mut lengths: Vec<f64> = Vec::new();

        let borders = std::mem::take(&mut self.m_planes[plane_idx].borders);

        // Collect new surface features to push, preserving exact order.
        let mut surface_features: Vec<SurfaceFeature> = Vec::new();

        for border in borders.iter() {
            if border.len() <= 1 {
                continue;
            }

            let mut done = false;

            // Measure.cpp:317-343
            if border.len() > 4 {
                let (center, radius, err) = get_center_and_radius(border, &trafo, &trafo_inv);

                if err < 0.05 {
                    // The whole border is one circle.
                    let is_polygon = border.len() > 4 && border.len() <= 8;
                    let lengths_match = {
                        // std::all_of(border.begin()+2, border.end(), ...)
                        let mut ok = true;
                        for i in 2..border.len() {
                            let a = (border[i] - border[i - 1]).norm_squared();
                            let b = (border[i - 1] - border[i - 2]).norm_squared();
                            if !is_approx_tol(a, b, if is_polygon { 0.01 } else { 0.01 }) {
                                ok = false;
                                break;
                            }
                        }
                        ok
                    };

                    if lengths_match && (is_polygon || border.len() > 8) {
                        if is_polygon {
                            // This is a polygon, add the separate edges with the center.
                            for j in 0..border.len() {
                                surface_features.push(SurfaceFeature::new(
                                    SurfaceFeatureType::Edge,
                                    border[if j == 0 { border.len() - 1 } else { j - 1 }],
                                    border[j],
                                    Some(center),
                                    0.0,
                                ));
                            }
                        } else {
                            // The fit went well and it has more than 8 points - circle.
                            surface_features.push(SurfaceFeature::new(
                                SurfaceFeatureType::Circle,
                                center,
                                normal,
                                None,
                                radius,
                            ));
                        }
                        done = true;
                    }
                }
            }

            if !done {
                // The border is not a circle and may contain circular segments.
                let are_angles_same = |a: f64, b: f64| is_approx_tol(a, b, 0.01);
                let are_lengths_same = |a: f64, b: f64| is_approx_tol(a, b, 0.01);

                // Measure.cpp:356-365 — offset_to_index
                let border_size = border.len() as i32;
                let offset_to_index = |idx: i32, offset: i32| -> i32 {
                    debug_assert!(offset.abs() < border_size);
                    let mut out = idx + offset;
                    if out >= border_size {
                        out -= border_size;
                    } else if out < 0 {
                        out += border_size;
                    }
                    out
                };

                // First calculate angles at all the vertices.
                angles.clear();
                lengths.clear();
                let mut first_different_angle_idx: i32 = 0;
                for i in 0..border.len() {
                    let v2 = border[i] - (if i == 0 { border[border.len() - 1] } else { border[i - 1] });
                    let v1 = (if i == border.len() - 1 { border[0] } else { border[i + 1] }) - border[i];
                    let mut angle = (-normal.dot(&v1.cross(&v2))).atan2(-v1.dot(&v2)) + M_PI;
                    if angle > M_PI {
                        angle = 2.0 * M_PI - angle;
                    }

                    angles.push(angle);
                    lengths.push(v2.norm());
                    if first_different_angle_idx == 0 && angles.len() > 1 {
                        if !are_angles_same(angles[angles.len() - 1], angles[angles.len() - 2]) {
                            first_different_angle_idx = (angles.len() - 1) as i32;
                        }
                    }
                }
                debug_assert!(border.len() == angles.len());
                debug_assert!(border.len() == lengths.len());

                // First go around the border and pick what might be circular segments.
                let mut start_idx: i32 = -1;
                let mut circle = false;
                let mut first_iter = true;
                let mut circles: Vec<SurfaceFeature> = Vec::new();
                let mut edges: Vec<SurfaceFeature> = Vec::new();
                let mut circles_idxs: Vec<(i32, i32)> = Vec::new();
                let mut single_circle: Vec<Vec3d> = Vec::new();
                let mut single_circle_length = 0.0;
                let first_pt_idx = offset_to_index(first_different_angle_idx, 1);
                let mut i = first_pt_idx;
                while i != first_pt_idx || first_iter {
                    if are_angles_same(angles[i as usize], angles[offset_to_index(i, -1) as usize])
                        && i != offset_to_index(first_pt_idx, -1) // not the last point
                        && i != start_idx
                    {
                        // circle
                        if !circle {
                            circle = true;
                            single_circle.clear();
                            single_circle_length = 0.0;
                            start_idx = offset_to_index(i, -2);
                            single_circle =
                                vec![border[start_idx as usize], border[offset_to_index(start_idx, 1) as usize]];
                            single_circle_length += lengths[offset_to_index(i, -1) as usize];
                        }
                        single_circle.push(border[i as usize]);
                        single_circle_length += lengths[i as usize];
                    } else {
                        if circle && single_circle.len() >= 5 {
                            // Less than 5 vertices? Not a circle.
                            single_circle.push(border[i as usize]);
                            single_circle_length += lengths[i as usize];

                            let mut accept_circle = true;
                            {
                                // Check that lengths of internal (!!!) edges match.
                                let mut j = offset_to_index(start_idx, 3);
                                while j != i {
                                    if !are_lengths_same(
                                        lengths[offset_to_index(j, -1) as usize],
                                        lengths[j as usize],
                                    ) {
                                        accept_circle = false;
                                        break;
                                    }
                                    j = offset_to_index(j, 1);
                                }
                            }

                            if accept_circle {
                                let (center, radius, err) =
                                    get_center_and_radius(&single_circle, &trafo, &trafo_inv);

                                // Reject complete failures.
                                accept_circle &= err < 0.05;
                                // If the segment subtends less than 90 degrees, throw it away.
                                accept_circle &= single_circle_length / radius > 0.9 * M_PI / 2.0;

                                if accept_circle {
                                    circles_idxs.push((start_idx, i));
                                    circles.push(SurfaceFeature::new(
                                        SurfaceFeatureType::Circle,
                                        center,
                                        normal,
                                        None,
                                        radius,
                                    ));
                                }
                            }
                        }
                        circle = false;
                    }
                    // Take care of the wrap around.
                    first_iter = false;
                    i = offset_to_index(i, 1);
                }

                // We have the circles. Now go around again and pick edges, jumping over circles.
                if circles_idxs.is_empty() {
                    // Just add all edges.
                    for i in 1..border.len() {
                        edges.push(SurfaceFeature::new(
                            SurfaceFeatureType::Edge,
                            border[i - 1],
                            border[i],
                            None,
                            0.0,
                        ));
                    }
                    edges.push(SurfaceFeature::new(
                        SurfaceFeatureType::Edge,
                        border[0],
                        border[border.len() - 1],
                        None,
                        0.0,
                    ));
                } else if circles_idxs.len() > 1 || circles_idxs[0].0 != circles_idxs[0].1 {
                    // There is at least one circular segment. Start at its end and add edges
                    // until the start of the next one.
                    let mut i = circles_idxs[0].1;
                    let mut circle_idx = 1usize;
                    loop {
                        i = offset_to_index(i, 1);
                        edges.push(SurfaceFeature::new(
                            SurfaceFeatureType::Edge,
                            border[offset_to_index(i, -1) as usize],
                            border[i as usize],
                            None,
                            0.0,
                        ));
                        if circle_idx < circles_idxs.len() && i == circles_idxs[circle_idx].0 {
                            i = circles_idxs[circle_idx].1;
                            circle_idx += 1;
                        }
                        if i == circles_idxs[0].0 {
                            break;
                        }
                    }
                }

                // Merge adjacent edges where needed.
                debug_assert!(edges.iter().all(|f| f.get_type() == SurfaceFeatureType::Edge));
                let mut i = edges.len() as i32 - 1;
                while i >= 0 {
                    let prev_idx = if i == 0 { edges.len() - 1 } else { (i - 1) as usize };
                    let (first_start, first_end) = edges[prev_idx].get_edge();
                    let (second_start, second_end) = edges[i as usize].get_edge();

                    if is_approx_vec(&first_end, &second_start)
                        && is_approx_tol(
                            (first_end - first_start)
                                .normalize()
                                .dot(&(second_end - second_start).normalize()),
                            1.0,
                            EPSILON,
                        )
                    {
                        // Same direction and share a point. Merge them.
                        edges[prev_idx] = SurfaceFeature::new(
                            SurfaceFeatureType::Edge,
                            first_start,
                            second_end,
                            None,
                            0.0,
                        );
                        edges.remove(i as usize);
                    }
                    i -= 1;
                }

                // Now move the circles and edges into the feature list for the plane.
                debug_assert!(circles.iter().all(|f| f.get_type() == SurfaceFeatureType::Circle));
                debug_assert!(edges.iter().all(|f| f.get_type() == SurfaceFeatureType::Edge));
                surface_features.extend(circles);
                surface_features.extend(edges);
            }
        }

        // The last surface feature is the plane itself.
        let mut cog = Vec3d::zeros();
        let mut counter: usize = 0;
        for b in borders.iter() {
            for i in 0..b.len() {
                cog += b[i];
                counter += 1;
            }
        }
        cog /= counter as f64;
        surface_features.push(SurfaceFeature::new(
            SurfaceFeatureType::Plane,
            normal,
            cog,
            None,
            plane_idx as f64 + 0.0001,
        ));

        let plane = &mut self.m_planes[plane_idx];
        plane.surface_features = surface_features;
        plane.borders.clear();
        plane.borders.shrink_to_fit();
        plane.features_extracted = true;
    }

    // Measure.cpp:529-598
    pub fn get_feature(
        &mut self,
        face_idx: usize,
        point: &Vec3d,
        world_tran: &Transform3d,
        only_select_plane: bool,
    ) -> Option<SurfaceFeature> {
        if face_idx >= self.m_face_to_plane.len() {
            return None;
        }

        let plane_idx = self.m_face_to_plane[face_idx];

        if !self.m_planes[plane_idx].features_extracted {
            self.extract_features(plane_idx);
        }

        let mut closest_feature_idx: usize = SIZE_T_MINUS_ONE;
        let mut min_dist = f64::MAX;

        let point_sf = SurfaceFeature::from_point(*point);

        debug_assert!(
            self.m_planes[plane_idx].surface_features.is_empty()
                || self.m_planes[plane_idx].surface_features.last().unwrap().get_type()
                    == SurfaceFeatureType::Plane
        );

        if !only_select_plane {
            let n = self.m_planes[plane_idx].surface_features.len();
            for i in 0..n - 1 {
                // The -1 prevents measuring distance to the plane itself.
                let res = get_measurement(
                    &self.m_planes[plane_idx].surface_features[i],
                    &point_sf,
                    false,
                );
                if let Some(ds) = &res.distance_strict {
                    let dist = ds.dist;
                    if dist < FEATURE_HOVER_LIMIT && dist < min_dist {
                        min_dist = dist.min(min_dist);
                        closest_feature_idx = i;
                    }
                }
            }

            if closest_feature_idx != SIZE_T_MINUS_ONE {
                let f = self.m_planes[plane_idx].surface_features[closest_feature_idx].clone();
                if f.get_type() == SurfaceFeatureType::Edge {
                    // If close to an endpoint, include the endpoint as well.
                    // Close = 10% of edge length, clamped between 0.025 and 0.5 mm.
                    let (sp, ep) = f.get_edge();
                    let len_sq = (ep - sp).norm_squared();
                    let limit_sq = (0.025 * 0.025_f64).max((0.5 * 0.5_f64).min(0.1 * 0.1 * len_sq));
                    if (point - sp).norm_squared() < limit_sq {
                        let mut local_f = SurfaceFeature::from_point(sp);
                        local_f.origin_surface_feature = Some(Rc::new(local_f.clone()));
                        local_f.translate_tran(world_tran);
                        return Some(local_f);
                    }

                    if (point - ep).norm_squared() < limit_sq {
                        let mut local_f = SurfaceFeature::from_point(ep);
                        local_f.origin_surface_feature = Some(Rc::new(local_f.clone()));
                        local_f.translate_tran(world_tran);
                        return Some(local_f);
                    }
                }
                let mut f_tran = f.clone();
                f_tran.origin_surface_feature = Some(Rc::new(f));
                f_tran.translate_tran(world_tran);
                return Some(f_tran);
            }
        }

        // Nothing detected, return the plane as a whole.
        debug_assert!(
            self.m_planes[plane_idx].surface_features.last().unwrap().get_type()
                == SurfaceFeatureType::Plane
        );
        let back = self.m_planes[plane_idx].surface_features.last().unwrap().clone();
        let mut f_tran = back.clone();
        f_tran.origin_surface_feature = Some(Rc::new(back));
        f_tran.translate_tran(world_tran);
        Some(f_tran)
    }

    // Measure.cpp:604-607
    pub fn get_num_of_planes(&self) -> i32 {
        self.m_planes.len() as i32
    }

    // Measure.cpp:611-615
    pub fn get_plane_triangle_indices(&self, idx: i32) -> &Vec<i32> {
        debug_assert!(idx >= 0 && idx < self.m_planes.len() as i32);
        &self.m_planes[idx as usize].facets
    }

    // Measure.cpp:617-621
    pub fn get_plane_tri_indices(&mut self, idx: i32) -> &mut Vec<i32> {
        debug_assert!(idx >= 0 && idx < self.m_planes.len() as i32);
        &mut self.m_planes[idx as usize].facets
    }

    // Measure.cpp:623-629
    pub fn get_plane_features(&mut self, plane_id: u32) -> &Vec<SurfaceFeature> {
        debug_assert!((plane_id as usize) < self.m_planes.len());
        if !self.m_planes[plane_id as usize].features_extracted {
            self.extract_features(plane_id as usize);
        }
        &self.m_planes[plane_id as usize].surface_features
    }

    // Measure.cpp:631-636
    pub fn get_plane_features_pointer(&mut self, plane_id: u32) -> &mut Vec<SurfaceFeature> {
        debug_assert!((plane_id as usize) < self.m_planes.len());
        if !self.m_planes[plane_id as usize].features_extracted {
            self.extract_features(plane_id as usize);
        }
        &mut self.m_planes[plane_id as usize].surface_features
    }

    // Measure.cpp:638-641
    pub fn get_its(&self) -> &indexed_triangle_set {
        &self.m_its
    }
}

// Measure.cpp:197-290 — the body of the per-plane tbb::parallel_for lambda.
// `goto PLANE_FAILURE` is modelled with a closure returning `Result`, where the
// `Err` path clears the borders (Measure.cpp:288-289).
fn update_plane_borders(
    plane: &mut PlaneData,
    plane_id: usize,
    face_to_plane: &[usize],
    face_neighbors: &[Vec3i],
    sm: &SurfaceMesh,
) {
    let facets = plane.facets.clone();
    plane.borders.clear();
    // std::vector<std::array<bool, 3>> visited(facets.size(), {false, false, false});
    let mut visited: Vec<[bool; 3]> = vec![[false, false, false]; facets.len()];

    let result: Result<(), ()> = (|| {
        for face_id in 0..facets.len() as i32 {
            debug_assert!(face_to_plane[facets[face_id as usize] as usize] == plane_id);

            for edge_id in 0..3i32 {
                // Every facet's edge with a neighbor from a different plane is part of an
                // edge to walk around. Skip the others.
                let neighbor_idx = face_neighbors[facets[face_id as usize] as usize][edge_id as usize];
                if neighbor_idx == -1 {
                    return Err(()); // goto PLANE_FAILURE
                }
                if visited[face_id as usize][edge_id as usize]
                    || face_to_plane[neighbor_idx as usize] == plane_id
                {
                    visited[face_id as usize][edge_id as usize] = true;
                    continue;
                }

                let mut he = sm.halfedge_face(Face_index::new(facets[face_id as usize]));
                while he.side() as i32 != edge_id {
                    he = sm.next(he);
                }

                // he is the first halfedge on the border. Walk around and append points.
                plane.borders.push(Vec::new());
                let last_border_idx = plane.borders.len() - 1;
                {
                    let lb = &mut plane.borders[last_border_idx];
                    lb.reserve(4);
                    let src = sm.source(he);
                    let p = sm.point(src);
                    lb.push(Vec3d::new(p[0] as f64, p[1] as f64, p[2] as f64));
                }
                let he_start = he;

                let fi: Face_index = he.face();
                let face_it = lower_bound(&facets, fi.0);
                debug_assert!(face_it != facets.len());
                debug_assert!(facets[face_it] == fi.0);
                visited[face_it][he.side() as usize] = true;

                loop {
                    let he_orig = he;
                    he = sm.next_around_target(he);
                    if he.is_invalid() {
                        return Err(()); // goto PLANE_FAILURE
                    }

                    // For broken meshes, the iteration might never return to he_orig.
                    // Remember all halfedges we saw to break out of infinite loops.
                    let mut he_seen: Vec<crate::surface_mesh::Halfedge_index> = Vec::new();

                    while face_to_plane[sm.face(he).0 as usize] == plane_id && he != he_orig {
                        he_seen.push(he);
                        he = sm.next_around_target(he);
                        if he.is_invalid() || he_seen.iter().any(|&x| x == he) {
                            return Err(()); // goto PLANE_FAILURE
                        }
                    }
                    he = sm.opposite(he);
                    if he.is_invalid() {
                        return Err(()); // goto PLANE_FAILURE
                    }

                    let fi: Face_index = he.face();
                    let face_it = lower_bound(&facets, fi.0);
                    if face_it == facets.len() || facets[face_it] != fi.0 {
                        // This indicates a broken mesh.
                        return Err(()); // goto PLANE_FAILURE
                    }

                    if visited[face_it][he.side() as usize] && he != he_start {
                        plane.borders[last_border_idx].truncate(1); // resize(1)
                        break;
                    }
                    visited[face_it][he.side() as usize] = true;

                    {
                        let src = sm.source(he);
                        let p = sm.point(src);
                        plane.borders[last_border_idx]
                            .push(Vec3d::new(p[0] as f64, p[1] as f64, p[2] as f64));
                    }

                    // In case of broken meshes, break out if it is clearly going bad.
                    if plane.borders[last_border_idx].len() > 3 * facets.len() + 1 {
                        return Err(()); // goto PLANE_FAILURE
                    }

                    if he == he_start {
                        break;
                    }
                }

                if plane.borders[last_border_idx].len() == 1 {
                    plane.borders.pop();
                } else {
                    debug_assert!(is_approx_eigen(
                        plane.borders[last_border_idx].first().unwrap(),
                        plane.borders[last_border_idx].last().unwrap()
                    ));
                    plane.borders[last_border_idx].pop();
                }
            }
        }
        Ok(()) // continue; There was no failure.
    })();

    if result.is_err() {
        // PLANE_FAILURE:
        plane.borders.clear();
    }
}

// std::lower_bound over a sorted ascending `&[i32]`: returns the index of the
// first element not less than `value` (== facets.size() if none).
#[inline]
fn lower_bound(sorted: &[i32], value: i32) -> usize {
    match sorted.binary_search(&value) {
        Ok(idx) => {
            // binary_search may land on any equal element; back up to the first.
            let mut i = idx;
            while i > 0 && sorted[i - 1] == value {
                i -= 1;
            }
            i
        }
        Err(idx) => idx,
    }
}

// Helper for `Vertex_index::point` indexing through SurfaceMesh.
#[allow(dead_code)]
fn vertex_point(sm: &SurfaceMesh, v: Vertex_index) -> Vec3d {
    let p = sm.point(v);
    Vec3d::new(p[0] as f64, p[1] as f64, p[2] as f64)
}

// ---------------------------------------------------------------------------
// Measure.hpp:123-148 / Measure.cpp:643-679 — Measuring
// ---------------------------------------------------------------------------

pub struct Measuring {
    priv_: MeasuringImpl,
}

impl Measuring {
    // Measure.cpp:643-645
    pub fn new(its: &indexed_triangle_set) -> Self {
        Measuring {
            priv_: MeasuringImpl::new(its),
        }
    }

    // Measure.cpp:651-657
    pub fn get_feature(
        &mut self,
        face_idx: usize,
        point: &Vec3d,
        world_tran: &Transform3d,
        only_select_plane: bool,
    ) -> Option<SurfaceFeature> {
        // Measure.cpp:653-655 — debug breakpoint for face_idx 7516/7517 (no-op).
        self.priv_.get_feature(face_idx, point, world_tran, only_select_plane)
    }

    // Measure.cpp:660-663
    pub fn get_num_of_planes(&self) -> i32 {
        self.priv_.get_num_of_planes()
    }

    // Measure.cpp:666-669
    pub fn get_plane_triangle_indices(&self, idx: i32) -> &Vec<i32> {
        self.priv_.get_plane_triangle_indices(idx)
    }

    // Measure.cpp:671-674
    pub fn get_plane_features(&mut self, plane_id: u32) -> &Vec<SurfaceFeature> {
        self.priv_.get_plane_features(plane_id)
    }

    // Measure.cpp:676-679
    pub fn get_its(&self) -> &indexed_triangle_set {
        self.priv_.get_its()
    }
}

// ---------------------------------------------------------------------------
// Measure.hpp:151-204 — DistAndPoints / AngleAndEdges / MeasurementResult
// ---------------------------------------------------------------------------

// Measure.hpp:151-156
#[derive(Debug, Clone)]
pub struct DistAndPoints {
    pub dist: f64,
    pub from: Vec3d,
    pub to: Vec3d,
}

impl DistAndPoints {
    pub fn new(dist_: f64, from_: Vec3d, to_: Vec3d) -> Self {
        DistAndPoints {
            dist: dist_,
            from: from_,
            to: to_,
        }
    }
}

// Measure.hpp:158-169
#[derive(Debug, Clone)]
pub struct AngleAndEdges {
    pub angle: f64,
    pub center: Vec3d,
    pub e1: (Vec3d, Vec3d),
    pub e2: (Vec3d, Vec3d),
    pub radius: f64,
    pub coplanar: bool,
}

impl AngleAndEdges {
    pub fn new(
        angle_: f64,
        center_: Vec3d,
        e1_: (Vec3d, Vec3d),
        e2_: (Vec3d, Vec3d),
        radius_: f64,
        coplanar_: bool,
    ) -> Self {
        AngleAndEdges {
            angle: angle_,
            center: center_,
            e1: e1_,
            e2: e2_,
            radius: radius_,
            coplanar: coplanar_,
        }
    }

    // Measure.hpp:168 / Measure.cpp:681
    pub fn dummy() -> Self {
        AngleAndEdges {
            angle: 0.0,
            center: Vec3d::zeros(),
            e1: (Vec3d::zeros(), Vec3d::zeros()),
            e2: (Vec3d::zeros(), Vec3d::zeros()),
            radius: 0.0,
            coplanar: true,
        }
    }
}

// Measure.hpp:171-184
#[derive(Debug, Clone, Default)]
pub struct MeasurementResult {
    pub angle: Option<AngleAndEdges>,
    pub distance_infinite: Option<DistAndPoints>,
    pub distance_strict: Option<DistAndPoints>,
    pub distance_xyz: Option<Vec3d>,
}

impl MeasurementResult {
    // Measure.hpp:177-179
    pub fn has_distance_data(&self) -> bool {
        self.distance_infinite.is_some() || self.distance_strict.is_some()
    }

    // Measure.hpp:181-183
    pub fn has_any_data(&self) -> bool {
        self.angle.is_some()
            || self.distance_infinite.is_some()
            || self.distance_strict.is_some()
            || self.distance_xyz.is_some()
    }
}

// ---------------------------------------------------------------------------
// Measure.cpp:683-834 — angle helpers
// ---------------------------------------------------------------------------

// Measure.cpp:683-745
fn angle_edge_edge(e1: &(Vec3d, Vec3d), e2: &(Vec3d, Vec3d)) -> AngleAndEdges {
    if are_parallel_edges(e1, e2) {
        return AngleAndEdges::dummy();
    }

    let mut e1_unit = edge_direction(&e1.0, &e1.1);
    let mut e2_unit = edge_direction(&e2.0, &e2.1);

    // project edges on the plane defined by them
    let normal = e1_unit.cross(&e2_unit).normalize();
    let plane = Hyperplane3::new(normal, e1.0);
    let e11_proj = plane.projection(&e1.0);
    let e12_proj = plane.projection(&e1.1);
    let e21_proj = plane.projection(&e2.0);
    let e22_proj = plane.projection(&e2.1);
    let mut e11_proj = e11_proj;
    let mut e12_proj = e12_proj;
    let mut e21_proj = e21_proj;
    let mut e22_proj = e22_proj;

    let coplanar = (e2.0 - e21_proj).norm() < EPSILON && (e2.1 - e22_proj).norm() < EPSILON;

    // rotate the plane to become the XY plane
    let qp = quat_from_two_vectors(&normal, &Vec3d::z());
    let qp_inverse = qp.inverse();
    let e11_rot = qp * e11_proj;
    let e12_rot = qp * e12_proj;
    let e21_rot = qp * e21_proj;
    let e22_rot = qp * e22_proj;

    // discard Z
    let e11_rot_2d = Vec2d::new(e11_rot.x, e11_rot.y);
    let e12_rot_2d = Vec2d::new(e12_rot.x, e12_rot.y);
    let e21_rot_2d = Vec2d::new(e21_rot.x, e21_rot.y);
    let e22_rot_2d = Vec2d::new(e22_rot.x, e22_rot.y);

    // find intersection (arc center) of edges in XY plane
    let e1_rot_2d_line = Hyperplane2::through(e11_rot_2d, e12_rot_2d);
    let e2_rot_2d_line = Hyperplane2::through(e21_rot_2d, e22_rot_2d);
    let center_rot_2d = e1_rot_2d_line.intersection(&e2_rot_2d_line);

    // arc center in original coordinate
    let center = qp_inverse * Vec3d::new(center_rot_2d.x, center_rot_2d.y, e11_rot.z);

    // ensure the edges are pointing away from the center
    let mut out_e1 = *e1;
    let mut out_e2 = *e2;
    if (center_rot_2d - e11_rot_2d).norm_squared() > (center_rot_2d - e12_rot_2d).norm_squared() {
        std::mem::swap(&mut e11_proj, &mut e12_proj);
        std::mem::swap(&mut out_e1.0, &mut out_e1.1);
        e1_unit = -e1_unit;
    }
    if (center_rot_2d - e21_rot_2d).norm_squared() > (center_rot_2d - e22_rot_2d).norm_squared() {
        std::mem::swap(&mut e21_proj, &mut e22_proj);
        std::mem::swap(&mut out_e2.0, &mut out_e2.1);
        e2_unit = -e2_unit;
    }

    // arc angle
    let angle = e1_unit.dot(&e2_unit).clamp(-1.0, 1.0).acos();
    // arc radius
    let e1_proj_mid = 0.5 * (e11_proj + e12_proj);
    let e2_proj_mid = 0.5 * (e21_proj + e22_proj);
    let radius = (center - e1_proj_mid).norm().min((center - e2_proj_mid).norm());

    AngleAndEdges::new(angle, center, out_e1, out_e2, radius, coplanar)
}

// Measure.cpp:747-796
fn angle_edge_plane(e: &(Vec3d, Vec3d), p: &(i32, Vec3d, Vec3d)) -> AngleAndEdges {
    let (_idx, normal, origin) = (p.0, p.1, p.2);
    let mut e1e2_unit = edge_direction_pair(e);
    if are_perpendicular(&e1e2_unit, &normal) {
        return AngleAndEdges::dummy();
    }

    // ensure the edge is pointing away from the intersection
    // 1st calculate intersection between edge and plane
    let plane = Hyperplane3::new(normal, origin);
    let line = ParametrizedLine3::through(e.0, e.1);
    let inters = line.intersection_point(&plane);

    // then verify edge direction and revert it, if needed
    let mut e1 = e.0;
    let mut e2 = e.1;
    if (e1 - inters).norm_squared() > (e2 - inters).norm_squared() {
        std::mem::swap(&mut e1, &mut e2);
        e1e2_unit = -e1e2_unit;
    }

    if are_parallel(&e1e2_unit, &normal) {
        let basis = orthonormal_basis(&e1e2_unit);
        let radius = (0.5 * (e1 + e2) - inters).norm();
        let edge_on_plane_dir = if basis[1].dot(&(origin - inters)) >= 0.0 {
            basis[1]
        } else {
            -basis[1]
        };
        let mut edge_on_plane = (inters, inters + radius * edge_on_plane_dir);
        if !is_approx_eigen(&inters, &e1) {
            edge_on_plane.0 += radius * edge_on_plane_dir;
            edge_on_plane.1 += radius * edge_on_plane_dir;
        }
        return AngleAndEdges::new(
            0.5 * PI,
            inters,
            (e1, e2),
            edge_on_plane,
            radius,
            is_approx_eigen(&inters, &e1),
        );
    }

    let e1e2 = e2 - e1;
    let e1e2_len = e1e2.norm();

    // calculate 2nd edge (on the plane)
    let temp = normal.cross(&e1e2);
    let edge_on_plane_unit = normal.cross(&temp).normalize();
    let mut edge_on_plane = (origin, origin + e1e2_len * edge_on_plane_unit);

    // ensure the 2nd edge is pointing in the correct direction
    let test_edge = (edge_on_plane.1 - edge_on_plane.0).cross(&e1e2);
    if test_edge.dot(&temp) < 0.0 {
        edge_on_plane = (origin, origin - e1e2_len * edge_on_plane_unit);
    }

    let mut ret = angle_edge_edge(&(e1, e2), &edge_on_plane);
    ret.radius = (inters - 0.5 * (e1 + e2)).norm();
    ret
}

// Measure.cpp:798-834
fn angle_plane_plane(p1: &(i32, Vec3d, Vec3d), p2: &(i32, Vec3d, Vec3d)) -> AngleAndEdges {
    let (_idx1, normal1, origin1) = (p1.0, p1.1, p1.2);
    let (_idx2, normal2, origin2) = (p2.0, p2.1, p2.2);

    // are planes parallel ?
    if are_parallel(&normal1, &normal2) {
        return AngleAndEdges::dummy();
    }

    // Measure.cpp:807-814 — intersection_plane_plane lambda
    let intersection_plane_plane = |n1: &Vec3d, o1: &Vec3d, n2: &Vec3d, o2: &Vec3d| -> (Vec3d, Vec3d) {
        // m is 2x3, b is 2x1; solve via column-pivot QR (least-norm / least-squares).
        let m = nalgebra::Matrix2x3::<f64>::new(n1.x, n1.y, n1.z, n2.x, n2.y, n2.z);
        let b = nalgebra::Vector2::<f64>::new(o1.dot(n1), o2.dot(n2));
        // Eigen colPivHouseholderQr on an underdetermined system returns a basic
        // solution. Use the minimum-norm solution via the Moore-Penrose pseudo-
        // inverse, which matches Eigen's behaviour for this rank-2 2x3 system on
        // the resulting line origin (the direction below is exact regardless).
        let mt = m.transpose();
        let mmt = m * mt; // 2x2
        let x = match mmt.try_inverse() {
            Some(inv) => mt * (inv * b),
            None => Vec3d::zeros(),
        };
        (n1.cross(n2).normalize(), Vec3d::new(x[0], x[1], x[2]))
    };

    // Calculate intersection line between planes
    let (intersection_line_direction, intersection_line_origin) =
        intersection_plane_plane(&normal1, &origin1, &normal2, &origin2);

    // Project planes' origin on intersection line
    let intersection_line = ParametrizedLine3::new(intersection_line_origin, intersection_line_direction);
    let origin1_proj = intersection_line.projection(&origin1);
    let origin2_proj = intersection_line.projection(&origin2);

    // Calculate edges on planes
    let edge_on_plane1_unit = (origin1 - origin1_proj).normalize();
    let edge_on_plane2_unit = (origin2 - origin2_proj).normalize();
    let radius = 10.0_f64.max((origin1 - origin1_proj).norm().max((origin2 - origin2_proj).norm()));
    let edge_on_plane1 = (
        origin1_proj + radius * edge_on_plane1_unit,
        origin1_proj + 2.0 * radius * edge_on_plane1_unit,
    );
    let edge_on_plane2 = (
        origin2_proj + radius * edge_on_plane2_unit,
        origin2_proj + 2.0 * radius * edge_on_plane2_unit,
    );

    let mut ret = angle_edge_edge(&edge_on_plane1, &edge_on_plane2);
    ret.radius = radius;
    ret
}

// ---------------------------------------------------------------------------
// Measure.cpp:836-1300 — get_measurement
// ---------------------------------------------------------------------------

// Measure.hpp:187 — MeasurementResult get_measurement(a, b, deal_circle_result=false);
pub fn get_measurement(a: &SurfaceFeature, b: &SurfaceFeature, deal_circle_result: bool) -> MeasurementResult {
    debug_assert!(
        a.get_type() != SurfaceFeatureType::Undef && b.get_type() != SurfaceFeatureType::Undef
    );

    let swap = (a.get_type() as i32) > (b.get_type() as i32);
    let f1 = if swap { b } else { a };
    let f2 = if swap { a } else { b };

    let mut result = MeasurementResult::default();

    if f1.get_type() == SurfaceFeatureType::Point {
        if f2.get_type() == SurfaceFeatureType::Point {
            let diff = f2.get_point() - f1.get_point();
            result.distance_strict =
                Some(DistAndPoints::new(diff.norm(), f1.get_point(), f2.get_point()));
            result.distance_xyz = Some(diff);
        } else if f2.get_type() == SurfaceFeatureType::Edge {
            let (s, e) = f2.get_edge();
            let line = ParametrizedLine3::new(s, (e - s).normalize());
            let dist_inf = line.distance(&f1.get_point());
            let proj = line.projection(&f1.get_point());
            let len_sq = (e - s).norm_squared();
            let dist_start_sq = (proj - s).norm_squared();
            let dist_end_sq = (proj - e).norm_squared();
            if dist_start_sq < len_sq && dist_end_sq < len_sq {
                // projection falls on the line - strict distance equals infinite
                result.distance_strict = Some(DistAndPoints::new(dist_inf, f1.get_point(), proj));
            } else {
                // the result is the closer of the endpoints
                let s_is_closer = dist_start_sq < dist_end_sq;
                result.distance_strict = Some(DistAndPoints::new(
                    (dist_start_sq.min(dist_end_sq) + sqr(dist_inf)).sqrt(),
                    f1.get_point(),
                    if s_is_closer { s } else { e },
                ));
            }
            result.distance_infinite = Some(DistAndPoints::new(dist_inf, f1.get_point(), proj));
        } else if f2.get_type() == SurfaceFeatureType::Circle {
            // Find a plane containing normal, center and the point.
            let (c, radius, n) = f2.get_circle();
            let circle_plane = Hyperplane3::new(n, c);
            let proj = circle_plane.projection(&f1.get_point());
            if is_approx_eigen(&proj, &c) {
                let p_on_circle = c + radius * get_orthogonal(&n, true);
                result.distance_strict = Some(DistAndPoints::new(radius, c, p_on_circle));
            } else if !deal_circle_result {
                let circle_plane = Hyperplane3::new(n, c);
                let proj = circle_plane.projection(&f1.get_point());
                let dist = (((proj - c).norm() - radius).powf(2.0)
                    + (f1.get_point() - proj).norm_squared())
                .sqrt();

                let p_on_circle = c + radius * (proj - c).normalize();
                result.distance_strict = Some(DistAndPoints::new(dist, f1.get_point(), p_on_circle));
            } else {
                let dist = (f1.get_point() - c).norm();
                result.distance_strict = Some(DistAndPoints::new(dist, f1.get_point(), c));
            }
        } else if f2.get_type() == SurfaceFeatureType::Plane {
            let (_idx, normal, pt) = f2.get_plane();
            let plane = Hyperplane3::new(normal, pt);
            result.distance_infinite = Some(DistAndPoints::new(
                plane.abs_distance(&f1.get_point()),
                f1.get_point(),
                plane.projection(&f1.get_point()),
            )); // TODO
                // TODO: result.distance_strict =
        }
    } else if f1.get_type() == SurfaceFeatureType::Edge {
        if f2.get_type() == SurfaceFeatureType::Edge {
            let mut distances: Vec<DistAndPoints> = Vec::new();

            // Measure.cpp:911-920 — add_point_edge_distance lambda
            let mut add_point_edge_distance = |v: Vec3d, e: &(Vec3d, Vec3d), distances: &mut Vec<DistAndPoints>| {
                let res = get_measurement(
                    &SurfaceFeature::from_point(v),
                    &SurfaceFeature::new(SurfaceFeatureType::Edge, e.0, e.1, None, 0.0),
                    false,
                );
                let ds = res.distance_strict.unwrap();
                let distance = ds.dist;
                let v2 = ds.to;

                let e1e2 = e.1 - e.0;
                let e1v2 = v2 - e.0;
                if e1v2.dot(&e1e2) >= 0.0 && e1v2.norm() < e1e2.norm() {
                    distances.push(DistAndPoints::new(distance, v, v2));
                }
            };

            let e1 = f1.get_edge();
            let e2 = f2.get_edge();

            distances.push(DistAndPoints::new((e2.0 - e1.0).norm(), e1.0, e2.0));
            distances.push(DistAndPoints::new((e2.1 - e1.0).norm(), e1.0, e2.1));
            distances.push(DistAndPoints::new((e2.0 - e1.1).norm(), e1.1, e2.0));
            distances.push(DistAndPoints::new((e2.1 - e1.1).norm(), e1.1, e2.1));
            add_point_edge_distance(e1.0, &e2, &mut distances);
            add_point_edge_distance(e1.1, &e2, &mut distances);
            add_point_edge_distance(e2.0, &e1, &mut distances);
            add_point_edge_distance(e2.1, &e1, &mut distances);
            let it = min_element_dist(&distances);
            result.distance_infinite = Some(distances[it].clone());

            result.angle = Some(angle_edge_edge(&f1.get_edge(), &f2.get_edge()));
        } else if f2.get_type() == SurfaceFeatureType::Circle {
            let e = f1.get_edge();
            let (center, _radius, _normal) = f2.get_circle();
            let e1e2 = e.1 - e.0;
            let e1e2_unit = e1e2.normalize();

            let mut distances: Vec<DistAndPoints> = Vec::new();
            distances.push(
                get_measurement(&SurfaceFeature::from_point(e.0), f2, false)
                    .distance_strict
                    .unwrap(),
            );
            distances.push(
                get_measurement(&SurfaceFeature::from_point(e.1), f2, false)
                    .distance_strict
                    .unwrap(),
            );

            let plane = Hyperplane3::new(e1e2_unit, center);
            let line = ParametrizedLine3::through(e.0, e.1);
            let inter = line.intersection_point(&plane);
            let e1inter = inter - e.0;
            if e1inter.dot(&e1e2) >= 0.0 && e1inter.norm() < e1e2.norm() {
                distances.push(
                    get_measurement(&SurfaceFeature::from_point(inter), f2, false)
                        .distance_strict
                        .unwrap(),
                );
            }

            let it = min_element_dist(&distances);
            if !deal_circle_result {
                result.distance_infinite = Some(DistAndPoints::new(
                    distances[it].dist,
                    distances[it].from,
                    distances[it].to,
                ));
            } else {
                let dist = (distances[it].from - center).norm();
                result.distance_infinite = Some(DistAndPoints::new(dist, distances[it].from, center));
            }
        } else if f2.get_type() == SurfaceFeatureType::Plane {
            let (from, to) = f1.get_edge();
            let (_idx, normal, origin) = f2.get_plane();

            let edge_unit = (to - from).normalize();
            if are_perpendicular(&edge_unit, &normal) {
                let mut distances: Vec<DistAndPoints> = Vec::new();
                let plane = Hyperplane3::new(normal, origin);
                distances.push(DistAndPoints::new(
                    plane.abs_distance(&from),
                    from,
                    plane.projection(&from),
                ));
                distances.push(DistAndPoints::new(
                    plane.abs_distance(&to),
                    to,
                    plane.projection(&to),
                ));
                let it = min_element_dist(&distances);
                result.distance_infinite = Some(DistAndPoints::new(
                    distances[it].dist,
                    distances[it].from,
                    distances[it].to,
                ));
            } else {
                let plane_features = f2.world_plane_features.clone();
                let mut distances: Vec<DistAndPoints> = Vec::new();
                if let Some(pf) = &plane_features {
                    for sf in pf.iter() {
                        if sf.get_type() == SurfaceFeatureType::Edge {
                            let m = get_measurement(sf, f1, false);
                            if m.distance_infinite.is_none() {
                                distances.clear();
                                break;
                            } else {
                                distances.push(m.distance_infinite.unwrap());
                            }
                        }
                    }
                }
                if !distances.is_empty() {
                    let it = min_element_dist(&distances);
                    result.distance_infinite = Some(DistAndPoints::new(
                        distances[it].dist,
                        distances[it].from,
                        distances[it].to,
                    ));
                }
            }
            result.angle = Some(angle_edge_plane(&f1.get_edge(), &f2.get_plane()));
        }
    } else if f1.get_type() == SurfaceFeatureType::Circle {
        if f2.get_type() == SurfaceFeatureType::Circle {
            let (c0, r0, n0) = f1.get_circle();
            let (c1, r1, n1) = f2.get_circle();

            // Adaptation of DistCircle3Circle3 (GeometricTools).
            #[derive(Clone, Copy)]
            struct ClosestInfo {
                sqr_distance: f64,
                circle0_closest: Vec3d,
                circle1_closest: Vec3d,
            }
            let mut candidates: [ClosestInfo; 16] = [ClosestInfo {
                sqr_distance: 0.0,
                circle0_closest: Vec3d::zeros(),
                circle1_closest: Vec3d::zeros(),
            }; 16];

            let zero = 0.0;

            let d = c1 - c0;

            let mut num_pairs: usize = 0;

            if !are_parallel(&n0, &n1) {
                // Get parameters for constructing the degree-8 polynomial phi.
                let one = 1.0;
                let two = 2.0;
                let r0sqr = sqr(r0);
                let r1sqr = sqr(r1);

                // Compute U1 and V1 for the plane of circle1.
                let basis = orthonormal_basis(&n1);
                let u1 = basis[0];
                let v1 = basis[1];

                // Construct the polynomial phi(cos(theta)).
                let n0xd = n0.cross(&d);
                let n0xu1 = n0.cross(&u1);
                let n0xv1 = n0.cross(&v1);
                let a0 = r1 * d.dot(&u1);
                let a1 = r1 * d.dot(&v1);
                let a2 = n0xd.dot(&n0xd);
                let a3 = r1 * n0xd.dot(&n0xu1);
                let a4 = r1 * n0xd.dot(&n0xv1);
                let a5 = r1sqr * n0xu1.dot(&n0xu1);
                let a6 = r1sqr * n0xu1.dot(&n0xv1);
                let a7 = r1sqr * n0xv1.dot(&n0xv1);
                let p0 = Polynomial1::from_values(&[a2 + a7, two * a3, a5 - a7]);
                let p1 = Polynomial1::from_values(&[two * a4, two * a6]);
                let p2 = Polynomial1::from_values(&[zero, a1]);
                let p3 = Polynomial1::from_values(&[-a0]);
                let p4 = Polynomial1::from_values(&[-a6, a4, two * a6]);
                let p5 = Polynomial1::from_values(&[-a3, a7 - a5]);
                let tmp0 = Polynomial1::from_values(&[one, zero, -one]);
                let tmp1 = &(&p2 * &p2) + &(&(&tmp0 * &p3) * &p3);
                let tmp2 = &(2.0 * &p2) * &p3;
                let tmp3 = &(&p4 * &p4) + &(&(&tmp0 * &p5) * &p5);
                let tmp4 = &(2.0 * &p4) * &p5;
                let p6 = &(&(&p0 * &tmp1) + &(&(&tmp0 * &p1) * &tmp2)) - &(r0sqr * &tmp3);
                let p7 = &(&(&p0 * &tmp2) + &(&p1 * &tmp1)) - &(r0sqr * &tmp4);

                let max_iterations: u32 = 128;
                let mut degree: i32;
                let mut num_roots: usize;
                let mut roots: [f64; 8] = [0.0; 8];
                let mut unique_roots: std::collections::BTreeSet<OrderedF64> = std::collections::BTreeSet::new();
                let mut pairs: [(f64, f64); 16] = [(0.0, 0.0); 16];
                let mut temp;
                let mut sn;

                if p7.get_degree() > 0 || p7[0] != zero {
                    // H(cs,sn) = p6(cs) + sn * p7(cs)
                    let phi = &(&p6 * &p6) - &(&(&tmp0 * &p7) * &p7);
                    degree = phi.get_degree() as i32;
                    debug_assert!(degree > 0);
                    num_roots =
                        RootsPolynomial::find(degree, &poly_coeffs(&phi), max_iterations, &mut roots) as usize;
                    for i in 0..num_roots {
                        unique_roots.insert(OrderedF64(roots[i]));
                    }

                    for cs_o in unique_roots.iter() {
                        let cs = cs_o.0;
                        if cs.abs() <= one {
                            temp = p7.eval(cs);
                            if temp != zero {
                                sn = -p6.eval(cs) / temp;
                                pairs[num_pairs] = (cs, sn);
                                num_pairs += 1;
                            } else {
                                temp = (one - sqr(cs)).max(zero);
                                sn = temp.sqrt();
                                pairs[num_pairs] = (cs, sn);
                                num_pairs += 1;
                                if sn != zero {
                                    pairs[num_pairs] = (cs, -sn);
                                    num_pairs += 1;
                                }
                            }
                        }
                    }
                } else {
                    // H(cs,sn) = p6(cs)
                    degree = p6.get_degree() as i32;
                    debug_assert!(degree > 0);
                    num_roots =
                        RootsPolynomial::find(degree, &poly_coeffs(&p6), max_iterations, &mut roots) as usize;
                    for i in 0..num_roots {
                        unique_roots.insert(OrderedF64(roots[i]));
                    }

                    for cs_o in unique_roots.iter() {
                        let cs = cs_o.0;
                        if cs.abs() <= one {
                            temp = (one - sqr(cs)).max(zero);
                            sn = temp.sqrt();
                            pairs[num_pairs] = (cs, sn);
                            num_pairs += 1;
                            if sn != zero {
                                pairs[num_pairs] = (cs, -sn);
                                num_pairs += 1;
                            }
                        }
                    }
                }
                let _ = num_roots;

                for i in 0..num_pairs {
                    let info = &mut candidates[i];
                    let mut delta = d + r1 * (pairs[i].0 * u1 + pairs[i].1 * v1);
                    info.circle1_closest = c0 + delta;
                    let n0d_delta = n0.dot(&delta);
                    let len_n0x_delta = n0.cross(&delta).norm();
                    if len_n0x_delta > 0.0 {
                        let diff = len_n0x_delta - r0;
                        info.sqr_distance = sqr(n0d_delta) + sqr(diff);
                        delta -= n0d_delta * n0;
                        delta = delta.normalize();
                        info.circle0_closest = c0 + r0 * delta;
                    } else {
                        let r0u0 = r0 * get_orthogonal(&n0, true);
                        let diff = delta - r0u0;
                        info.sqr_distance = diff.dot(&diff);
                        info.circle0_closest = c0 + r0u0;
                    }
                }

                // std::sort(candidates.begin(), candidates.begin() + numPairs)
                candidates[..num_pairs]
                    .sort_by(|a, b| a.sqr_distance.partial_cmp(&b.sqr_distance).unwrap());
            } else {
                num_pairs = 1;
                let info = &mut candidates[0];

                let n0d_d = n0.dot(&d);
                let norm_proj = n0d_d * n0;
                let comp_proj = d - norm_proj;
                let mut u = comp_proj;
                let dd = u.norm();
                u = u.normalize();

                // Configuration determined by relative location of projection intervals.
                let dmr1 = dd - r1;
                let distance;
                if dmr1 >= r0 {
                    // d >= r0 + r1 : separated or tangent (one outside the other).
                    distance = dmr1 - r0;
                    info.circle0_closest = c0 + r0 * u;
                    info.circle1_closest = c1 - r1 * u;
                } else {
                    // d < r0 + r1 ; implicitly d >= 0.
                    let dpr1 = dd + r1;
                    if dpr1 <= r0 {
                        // Circle1 is inside circle0.
                        distance = r0 - dpr1;
                        if dd > 0.0 {
                            info.circle0_closest = c0 + r0 * u;
                            info.circle1_closest = c1 + r1 * u;
                        } else {
                            // Concentric, U = (0,0,0).
                            u = get_orthogonal(&n0, true);
                            info.circle0_closest = c0 + r0 * u;
                            info.circle1_closest = c1 + r1 * u;
                        }
                    } else if dmr1 <= -r0 {
                        // Circle0 is inside circle1.
                        distance = -r0 - dmr1;
                        if dd > 0.0 {
                            info.circle0_closest = c0 - r0 * u;
                            info.circle1_closest = c1 - r1 * u;
                        } else {
                            // Concentric, U = (0,0,0).
                            u = get_orthogonal(&n0, true);
                            info.circle0_closest = c0 + r0 * u;
                            info.circle1_closest = c1 + r1 * u;
                        }
                    } else {
                        distance = (c1 - c0).norm();
                        info.circle0_closest = c0;
                        info.circle1_closest = c1;
                    }
                }

                info.sqr_distance = distance * distance;
            }
            let _ = num_pairs;
            if !deal_circle_result {
                result.distance_infinite = Some(DistAndPoints::new(
                    candidates[0].sqr_distance.sqrt(),
                    candidates[0].circle0_closest,
                    candidates[0].circle1_closest,
                )); // TODO
            } else {
                let dist = (c0 - c1).norm();
                result.distance_strict = Some(DistAndPoints::new(dist, c0, c1));
            }
        } else if f2.get_type() == SurfaceFeatureType::Plane {
            let (center, _radius, normal1) = f1.get_circle();
            let (_idx2, normal2, origin2) = f2.get_plane();

            let coplanar = are_parallel(&normal1, &normal2)
                && Hyperplane3::new(normal1, center).abs_distance(&origin2) < EPSILON;
            if !coplanar {
                let plane_features = f2.world_plane_features.clone();
                let mut distances: Vec<DistAndPoints> = Vec::new();
                if let Some(pf) = &plane_features {
                    for sf in pf.iter() {
                        if sf.get_type() == SurfaceFeatureType::Edge {
                            let m = get_measurement(sf, f1, false);
                            if m.distance_infinite.is_none() {
                                distances.clear();
                                break;
                            } else {
                                distances.push(m.distance_infinite.unwrap());
                            }
                        }
                    }
                }
                if !distances.is_empty() {
                    let it = min_element_dist(&distances);
                    result.distance_infinite = Some(DistAndPoints::new(
                        distances[it].dist,
                        distances[it].from,
                        distances[it].to,
                    ));
                } else {
                    let plane = Hyperplane3::new(normal2, origin2);
                    result.distance_infinite = Some(DistAndPoints::new(
                        plane.abs_distance(&center),
                        center,
                        plane.projection(&center),
                    ));
                }
            } else {
                result.distance_strict = Some(DistAndPoints::new(0.0, center, origin2));
            }
        }
    } else if f1.get_type() == SurfaceFeatureType::Plane {
        let (_idx1, normal1, pt1) = f1.get_plane();
        let (_idx2, normal2, pt2) = f2.get_plane();

        if are_parallel(&normal1, &normal2) {
            // The planes are parallel, calculate distance.
            let plane = Hyperplane3::new(normal2, pt2);
            result.distance_infinite = Some(DistAndPoints::new(
                plane.abs_distance(&pt1),
                pt1,
                plane.projection(&pt1),
            ));
        } else {
            result.angle = Some(angle_plane_plane(&f1.get_plane(), &f2.get_plane()));
        }
    }

    if swap {
        // Measure.cpp:1287-1297 — swap_dist_and_points lambda
        if let Some(dp) = result.distance_infinite.as_mut() {
            std::mem::swap(&mut dp.from, &mut dp.to);
        }
        if let Some(dp) = result.distance_strict.as_mut() {
            std::mem::swap(&mut dp.from, &mut dp.to);
        }
    }
    result
}

// std::min_element over distances by `.dist` (returns the index of the first min).
#[inline]
fn min_element_dist(distances: &[DistAndPoints]) -> usize {
    let mut best = 0usize;
    for i in 1..distances.len() {
        // std::min_element keeps the first element for which the comparator is
        // never true; comparator is `item1.dist < item2.dist`.
        if distances[i].dist < distances[best].dist {
            best = i;
        }
    }
    best
}

// Helper to expose a Polynomial1's coefficient slice (`&phi[0]`) for RootsPolynomial::find.
#[inline]
fn poly_coeffs(p: &Polynomial1) -> Vec<f64> {
    let mut v = Vec::with_capacity(p.get_degree() as usize + 1);
    for i in 0..=p.get_degree() {
        v.push(p[i]);
    }
    v
}

// Total-order wrapper over f64 so it can live in a `std::set<double>` (BTreeSet),
// reproducing `std::set<double>` ordering (ascending; NaN does not occur here).
#[derive(Clone, Copy)]
struct OrderedF64(f64);
impl PartialEq for OrderedF64 {
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}
impl Eq for OrderedF64 {}
impl PartialOrd for OrderedF64 {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for OrderedF64 {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.0.partial_cmp(&other.0).unwrap_or(std::cmp::Ordering::Equal)
    }
}

// Measure.cpp:1302-1317
pub fn can_set_xyz_distance(a: &SurfaceFeature, b: &SurfaceFeature) -> bool {
    let swap = (a.get_type() as i32) > (b.get_type() as i32);
    let f1 = if swap { b } else { a };
    let f2 = if swap { a } else { b };
    if f1.get_type() == SurfaceFeatureType::Point {
        if f2.get_type() == SurfaceFeatureType::Point {
            return true;
        }
    } else if f1.get_type() == SurfaceFeatureType::Circle {
        if f2.get_type() == SurfaceFeatureType::Circle {
            return true;
        }
    }
    false
}

// Measure.hpp:190-203
#[derive(Debug, Clone)]
pub struct AssemblyAction {
    pub can_set_to_parallel: bool,
    pub can_set_to_center_coincidence: bool,
    pub can_set_feature_1_reverse_rotation: bool,
    pub can_set_feature_2_reverse_rotation: bool,
    pub can_around_center_of_faces: bool,
    pub has_parallel_distance: bool,
    pub parallel_distance: f32,
    pub angle_radian: f32,
    pub tran_for_parallel: Transform3d,
    pub tran_for_center_coincidence: Transform3d,
    pub tran_for_reverse_rotation: Transform3d,
}

impl Default for AssemblyAction {
    fn default() -> Self {
        AssemblyAction {
            can_set_to_parallel: false,
            can_set_to_center_coincidence: false,
            can_set_feature_1_reverse_rotation: false,
            can_set_feature_2_reverse_rotation: false,
            can_around_center_of_faces: false,
            has_parallel_distance: false,
            parallel_distance: 0.0,
            angle_radian: 0.0,
            tran_for_parallel: Transform3d::identity(),
            tran_for_center_coincidence: Transform3d::identity(),
            tran_for_reverse_rotation: Transform3d::identity(),
        }
    }
}

// Measure.cpp:1319-1353
pub fn get_assembly_action(a: &SurfaceFeature, b: &SurfaceFeature) -> AssemblyAction {
    let mut action = AssemblyAction::default();
    let f1 = a;
    let f2 = b;
    if f1.get_type() == SurfaceFeatureType::Plane {
        action.can_set_feature_1_reverse_rotation = true;
        if f2.get_type() == SurfaceFeatureType::Plane {
            let (_idx1, normal1, pt1) = f1.get_plane();
            let (_idx2, normal2, pt2) = f2.get_plane();
            action.can_set_to_center_coincidence = true;
            action.can_set_feature_2_reverse_rotation = true;
            if are_parallel(&normal1, &normal2) {
                action.can_set_to_parallel = false;
                action.has_parallel_distance = true;
                action.can_around_center_of_faces = true;
                let mut proj_pt2 = Vec3d::zeros();
                get_point_projection_to_plane(&pt2, &pt1, &normal1, &mut proj_pt2);
                action.parallel_distance = (pt2 - proj_pt2).norm() as f32;
                if (pt2 - proj_pt2).dot(&normal1) < 0.0 {
                    action.parallel_distance = -action.parallel_distance;
                }
                action.angle_radian = 0.0;
            } else {
                action.can_set_to_parallel = true;
                action.has_parallel_distance = false;
                action.can_around_center_of_faces = false;
                action.parallel_distance = 0.0;
                action.angle_radian = normal2.dot(&-normal1).clamp(-1.0, 1.0).acos() as f32;
            }
        }
    }
    action
}
